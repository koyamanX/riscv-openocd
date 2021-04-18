/***************************************************************************
 *   Copyright (C) 2021 by Chihiro Koyama                                  *
 *   ckoyama.1996@gmail.com                                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/* Most of the codes below are derived from src/target/openrisc/or1k_tap_vjtag.c */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>

/* Contains constants relevant to the Altera Virtual JTAG
 * device, which are not included in the BSDL.
 * As of this writing, these are constant across every
 * device which supports virtual JTAG.
 */

/* These are commands for the FPGA's IR. */
#define ALTERA_CYCLONE_CMD_USER1	0x0E
#define ALTERA_CYCLONE_CMD_USER0	0x0C

#define RISCV_DEBUG_DTMCS			0x10
#define RISCV_DEBUG_DMI				0x11

/* SLD node ID. */
#define JTAG_TO_AVALON_NODE_ID		0x84
#define VJTAG_NODE_ID			0x08
#define SIGNAL_TAP_NODE_ID		0x00
#define SERIAL_FLASH_LOADER_NODE_ID	0x04

#define VER(x)				((x >> 27) & 0x1f)
#define NB_NODES(x)			((x >> 19) & 0xff)
#define ID(x)				((x >> 19) & 0xff)
#define MANUF(x)			((x >> 8)  & 0x7ff)
#define M_WIDTH(x)			((x >> 0)  & 0xff)
#define INST_ID(x)			((x >> 0)  & 0xff)

/* tap instructions - Mohor JTAG TAP */
#define OR1K_TAP_INST_IDCODE 0x2
#define OR1K_TAP_INST_DEBUG 0x8

static const char *id_to_string(unsigned char id)
{
	switch (id) {
	case VJTAG_NODE_ID:
		return "Virtual JTAG";
	case JTAG_TO_AVALON_NODE_ID:
		return "JTAG to avalon bridge";
	case SIGNAL_TAP_NODE_ID:
		return "Signal TAP";
	case SERIAL_FLASH_LOADER_NODE_ID:
		return "Serial Flash Loader";
	}
	return "unknown";
}

static unsigned char guess_addr_width(unsigned char number_of_nodes)
{
	unsigned char width = 0;

	while (number_of_nodes) {
		number_of_nodes >>= 1;
		width++;
	}

	return width;
}

static int nb_nodes;
static int m_width;
static int vjtag_node_address = -1;
int vjtag_vir_scan(struct jtag_tap *tap, uint32_t vir);

int riscv_tap_vjtag_init(struct jtag_tap *tap)
{
	LOG_DEBUG("Initialising Altera Virtual JTAG TAP");

	/* Put TAP into state where it can talk to the debug interface
	 * by shifting in correct value to IR.
	 */

	/* Ensure TAP is reset - maybe not necessary*/
	jtag_add_tlr();

	/* You can use a custom JTAG controller to discover transactions
	 * necessary to enumerate all Virtual JTAG megafunction instances
	 * from your design at runtime. All SLD nodes and the virtual JTAG
	 * registers that they contain are targeted by two Instruction Register
	 * values, USER0 and USER1.
	 *
	 * The USER1 instruction targets the virtual IR of either the sld_hub
	 * or a SLD node. That is,when the USER1 instruction is issued to
	 * the device, the subsequent DR scans target a specific virtual
	 * IR chain based on an address field contained within the DR scan.
	 * The table below shows how the virtual IR, the DR target of the
	 * USER1 instruction is interpreted.
	 *
	 * The VIR_VALUE in the table below is the virtual IR value for the
	 * target SLD node. The width of this field is m bits in length,
	 * where m is the length of the largest VIR for all of the SLD nodes
	 * in the design. All SLD nodes with VIR lengths of fewer than m
	 * bits must pad VIR_VALUE with zeros up to a length of m.
	 *
	 * -------------------------------+-------------------------------
	 * m + n - 1                   m  |  m -1                       0
	 * -------------------------------+-------------------------------
	 *     ADDR [(n – 1)..0]          |     VIR_VALUE [(m – 1)..0]
	 * -------------------------------+-------------------------------
	 *
	 * The ADDR bits act as address values to signal the active SLD node
	 * that the virtual IR shift targets. ADDR is n bits in length, where
	 * n bits must be long enough to encode all SLD nodes within the design,
	 * as shown below.
	 *
	 * n = CEIL(log2(Number of SLD_nodes +1))
	 *
	 * The SLD hub is always 0 in the address map.
	 *
	 * Discovery and enumeration of the SLD instances within a design
	 * requires interrogation of the sld_hub to determine the dimensions
	 * of the USER1 DR (m and n) and associating each SLD instance, specifically
	 * the Virtual JTAG megafunction instances, with an address value
	 * contained within the ADDR bits of the USER1 DR.
	 *
	 * The SLD hub contains the HUB IP Configuration Register and SLD_NODE_INFO
	 * register for each SLD node in the design. The HUB IP configuration register provides
	 * information needed to determine the dimensions of the USER1 DR chain. The
	 * SLD_NODE_INFO register is used to determine the address mapping for Virtual
	 * JTAG instance in your design. This register set is shifted out by issuing the
	 * HUB_INFO instruction. Both the ADDR bits for the SLD hub and the HUB_INFO
	 * instruction is 0 × 0.
	 * Because m and n are unknown at this point, the DR register
	 * (ADDR bits + VIR_VALUE) must be filled with zeros. Shifting a sequence of 64 zeroes
	 * into the USER1 DR is sufficient to cover the most conservative case for m and n.
	 */

	uint8_t t[4] = { 0 };
	struct scan_field field;

	/* Select VIR */
	buf_set_u32(t, 0, tap->ir_length, ALTERA_CYCLONE_CMD_USER1);
	field.num_bits = tap->ir_length;
	field.out_value = t;
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	/* Select the SLD Hub */
	field.num_bits = 64;
	field.out_value = NULL;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	/* HUB IP Configuration Register
	 *
	 * When the USER1 and HUB_INFO instruction sequence is issued, the
	 * USER0 instruction must be applied to enable the target register
	 * of the HUB_INFO instruction. The HUB IP configuration register
	 * is shifted out using eight four-bit nibble scans of the DR register.
	 * Each four-bit scan must pass through the UPDATE_DR state before
	 * the next four-bit scan. The 8 scans are assembled into a 32-bit
	 * value with the definitions shown in the table below.
	 *
	 * --------------------------------------------------------------------------------
	 *  NIBBLE7 | NIBBLE6 | NIBBLE5 | NIBBLE4 | NIBBLE3 | NIBBLE2 | NIBBLE1 | NIBBLE0
	 * ----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+-----
	 *     |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
	 * ----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+-----
	 * HUB IP version|         N         | ALTERA_MFG_ID (0x06E)  |     SUM (m, n)
	 * --------------+-------------------+------------------------+--------------------
	 */

	/* Select VDR */
	buf_set_u32(t, 0, tap->ir_length, ALTERA_CYCLONE_CMD_USER0);
	field.num_bits = tap->ir_length;
	field.out_value = t;
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	uint8_t nibble;
	uint32_t hub_info = 0;

	for (int i = 0; i < 8; i++) {
		field.num_bits = 4;
		field.out_value = NULL;
		field.in_value = &nibble;
		jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;
		hub_info = ((hub_info >> 4) | ((nibble & 0xf) << 28));
	}

	nb_nodes = NB_NODES(hub_info);
	m_width = M_WIDTH(hub_info);

	LOG_DEBUG("SLD HUB Configuration register");
	LOG_DEBUG("------------------------------");
	LOG_DEBUG("m_width         = %d", m_width);
	LOG_DEBUG("manufacturer_id = 0x%02" PRIx32, MANUF(hub_info));
	LOG_DEBUG("nb_of_node      = %d", nb_nodes);
	LOG_DEBUG("version         = %" PRIu32, VER(hub_info));
	LOG_DEBUG("VIR length      = %d", guess_addr_width(nb_nodes) + m_width);

	/* Because the number of SLD nodes is now known, the Nodes on the hub can be
	 * enumerated by repeating the 8 four-bit nibble scans, once for each Node,
	 * to yield the SLD_NODE_INFO register of each Node. The DR nibble shifts
	 * are a continuation of the HUB_INFO DR shift used to shift out the Hub IP
	 * Configuration register.
	 *
	 * The order of the Nodes as they are shifted out determines the ADDR
	 * values for the Nodes, beginning with, for the first Node SLD_NODE_INFO
	 * shifted out, up to and including, for the last node on the hub. The
	 * tables below show the SLD_NODE_INFO register and a their functional descriptions.
	 *
	 *  --------------+-----------+---------------+----------------
	 *   31        27 | 26     19 | 18          8 | 7            0
	 *  --------------+-----------+---------------+----------------
	 *   Node Version |  NODE ID  |  NODE MFG_ID  |  NODE INST ID
	 *
	 */

	vjtag_node_address = -1;
	int node_index;
	uint32_t node_info = 0;
	for (node_index = 0; node_index < nb_nodes; node_index++) {

		for (int i = 0; i < 8; i++) {
			field.num_bits = 4;
			field.out_value = NULL;
			field.in_value = &nibble;
			jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;
			node_info = ((node_info >> 4) | ((nibble & 0xf) << 28));
		}

		LOG_DEBUG("Node info register");
		LOG_DEBUG("--------------------");
		LOG_DEBUG("instance_id     = %" PRIu32, ID(node_info));
		LOG_DEBUG("manufacturer_id = 0x%02" PRIx32, MANUF(node_info));
		LOG_DEBUG("node_id         = %" PRIu32 " (%s)", ID(node_info),
						       id_to_string(ID(node_info)));
		LOG_DEBUG("version         = %" PRIu32, VER(node_info));

		if (ID(node_info) == VJTAG_NODE_ID)
			vjtag_node_address = node_index + 1;
	}

	if (vjtag_node_address < 0) {
		LOG_ERROR("No VJTAG TAP instance found !");
		return ERROR_FAIL;
	}

	return vjtag_vir_scan(tap, RISCV_DEBUG_DTMCS);

#if 0
	/* Select VIR */
	buf_set_u32(t, 0, tap->ir_length, ALTERA_CYCLONE_CMD_USER1);
	field.num_bits = tap->ir_length;
	field.out_value = t;
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	/* Set address of DTMCS to the VJTAG IR */
	int dr_length = guess_addr_width(nb_nodes) + m_width;
	buf_set_u32(t, 0, dr_length, (vjtag_node_address << m_width) | RISCV_DEBUG_DTMCS);
	field.num_bits = dr_length;
	field.out_value = t;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	/* Select the VJTAG DR */
	buf_set_u32(t, 0, tap->ir_length, ALTERA_CYCLONE_CMD_USER0);
	field.num_bits = tap->ir_length;
	field.out_value = t;
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	return jtag_execute_queue();
#endif
}

int vjtag_vir_scan(struct jtag_tap *tap, uint32_t vir)
{
	uint8_t t[4] = { 0 };
	struct scan_field field;

	/* Select VIR */
	buf_set_u32(t, 0, tap->ir_length, ALTERA_CYCLONE_CMD_USER1);
	field.num_bits = tap->ir_length;
	field.out_value = t;
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	/* Set address of DTMCS to the VJTAG IR */
	int dr_length = guess_addr_width(nb_nodes) + m_width;
	buf_set_u32(t, 0, dr_length, (vjtag_node_address << m_width) | vir);
	field.num_bits = dr_length;
	field.out_value = t;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	/* Select the VJTAG DR */
	buf_set_u32(t, 0, tap->ir_length, ALTERA_CYCLONE_CMD_USER0);
	field.num_bits = tap->ir_length;
	field.out_value = t;
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	return jtag_execute_queue();
}
