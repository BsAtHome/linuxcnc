// Copyright (C) 2025 B Stultiens
// Parts from mesa_modbus.c.tmpl Copyright (C) 2023 Andy Pugh
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//

/* A generic configurable Modbus component using Mesa PktUART interfaces */


#include "rtapi.h"
#include "rtapi_slab.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_byteorder.h"
#include "rtapi_ctype.h"
#include "hal.h"
#include "hostmot2-serial.h"

#include "hm2_modbus.h"

#if !defined(__KERNEL__)
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <endian.h>
#include <math.h>
static inline double myfabs(double d) { return fabs(d); }
static inline rtapi_u32 be32_to_cpu(rtapi_u32 v) { return be32toh(v); }
static inline rtapi_u16 be16_to_cpu(rtapi_u16 v) { return be16toh(v); }
#else
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
static inline double myfabs(double d) { return d < 0 ? -d : d; }
#include <linux/byteorder/generic.h>
#endif

// Define to compile in debug messages
#define DEBUG


/* module information */
MODULE_AUTHOR("B.Stultiens");
MODULE_DESCRIPTION("Modbus control using Mesa PktUART");
MODULE_LICENSE("GPL");

// all doubles within fabs(EPSILON) are considered zero
#define EPSILON		1e-12

#define MAX_PORTS 8

static const char *error_codes[] = {
	"Invalid exception code: 0",	// 0 - Not a valid error
	"Illegal Function",
	"Illegal Data Address",
	"Illegal Data Value",
	"Server Device Failure",
	"Acknowledge",
	"Server Device Busy",
	"Negative Acknowledge",			// 7 - Not defined in V1.1b3
	"Memory Parity Error",
	"Unknown exception code: 9",
	"Gateway Path Unavailable",
	"Gateway Failed to Respond",
	"Comm Timeout"					// 12 - Not defined in V1.1b3
};

// Byte-order markers for types
#define _MB_ABCD		0x100
#define _MB_BADC		0x200
#define _MB_CDAB		0x300
#define _MB_DCBA		0x400
#define _MB_XXXX_MASK	0xf00

#define HAL_FLOAT_ABCD	(HAL_FLOAT | _MB_ABCD)
#define HAL_FLOAT_BADC	(HAL_FLOAT | _MB_BADC)
#define HAL_FLOAT_CDAB	(HAL_FLOAT | _MB_CDAB)
#define HAL_FLOAT_DCBA	(HAL_FLOAT | _MB_DCBA)
#define HAL_U32_ABCD	(HAL_U32 | _MB_ABCD)
#define HAL_U32_BADC	(HAL_U32 | _MB_BADC)
#define HAL_U32_CDAB	(HAL_U32 | _MB_CDAB)
#define HAL_U32_DCBA	(HAL_U32 | _MB_DCBA)
#define HAL_S32_ABCD	(HAL_S32 | _MB_ABCD)
#define HAL_S32_BADC	(HAL_S32 | _MB_BADC)
#define HAL_S32_CDAB	(HAL_S32 | _MB_CDAB)
#define HAL_S32_DCBA	(HAL_S32 | _MB_DCBA)

// Supported Modbus commands
#define MBCMD_R_COILS      1
#define MBCMD_R_INPUTS     2
#define MBCMD_R_REGISTERS  3
#define MBCMD_R_INPUTREGS  4
#define MBCMD_W_COIL       5
#define MBCMD_W_REGISTER   6
#define MBCMD_W_COILS     15
#define MBCMD_W_REGISTERS 16

#define xstr(x) #x
#define str(x) xstr(x)

#define COMP_NAME "hm2_modbus"

// Maximum PDU data payload
#define MAX_MSG_LEN 252
// One byte device address, one byte command, two bytes CRC
#define MAX_PKT_LEN (1 + 1 + MAX_MSG_LEN + 2)

#ifndef DEBUG
#define MSG_DBG(fmt...)		do{}while(0)
#else
#define MSG_DBG(fmt...)		do { rtapi_print_msg(RTAPI_MSG_DBG,  fmt); } while(0)
#endif
#define MSG_INFO(fmt...)	do { rtapi_print_msg(RTAPI_MSG_INFO, fmt); } while(0)
#define MSG_ERR(fmt...)		do { rtapi_print_msg(RTAPI_MSG_ERR,  fmt); } while(0)
#define MSG_WARN(fmt...)	do { rtapi_print_msg(RTAPI_MSG_WARN, fmt); } while(0)

// State-machine states
enum {
	STATE_START,
	STATE_WAIT_FOR_TIMEOUT,
	STATE_WAIT_FOR_SEND_BEGIN,
	STATE_WAIT_FOR_SEND_COMPLETE,
	STATE_WAIT_FOR_DATA_FRAME,
	STATE_WAIT_FOR_FRAME_SIZES,
	STATE_WAIT_FOR_DATA,
	STATE_FETCH_MORE_DATA,
	STATE_WAIT_A_BIT,
	STATE_WAIT_FOR_RX_CLEAR,
	STATE_RESET_WAIT,
};

// The rate of the in/out cycle through all Modbus commands. The lower bound
// results in 1e9/1e-3 nanoseconds (1000 seconds between cycled). The upper
// bound is currently at 1e9/1e4 100us between updates.
#define RATE_MIN	(1e-3)	// Cycle at 0.001 Hz. Must be larger than zero.
#define RATE_MAX	(1e4)	// Cycle at 10000 Hz (too fast for most)
// Note that Modbus is a very slow communication process. The fastest
// packet-set (command+reply) is normally (8+3.5 + 8+3.5)*11 bit-times. That is
// about 2 milliseconds @115200 and more than 26 milliseconds @9600. The
// effective frequency of sending/receiving data is practically never higher
// than 100 Hz. A more realistic, still very fast, down-to-earth frequency
// estimate will be half of that. However, YMMV. When you have more devices and
// commands on one channel, or slower baud-rates, then the achievable update
// rate will decrease rapidly to below 10 Hz.

// Overlapping types to handle byte-ordering
typedef union {
	rtapi_u8	b[4];
	rtapi_u16	w[2];
	rtapi_u32	u;
	rtapi_s32	s;
	float		f;
} mb_types_u;

typedef struct {
	hal_data_u **pins;	// All Modbus data pins
	hal_bit_t *fault;
	hal_u32_t *faultcmd;
	hal_u32_t *lasterror;
	hal_u32_t baudrate;	// RO
	hal_u32_t parity;	// RO
	hal_u32_t stopbits;	// RO
	hal_u32_t txdelay;	// RW Inter frame delay for packets sent
	hal_u32_t rxdelay;	// RW Inter frame delay for packet end detection in receive
	hal_u32_t drvdelay;	// RW Delay before sending data (in bit times)
	hal_u32_t interval;	// RW Loop rate in microseconds
} hm2_modbus_hal_t;

// The command structure and data buffer.
// Note: The buffer is wasted on init lines and delay commands. But wasting a
// few kilobytes is not fatal in modern times.
typedef struct {
	hm2_modbus_mbccb_cmds_t cmd;	// In host order
	int pinref;		// What pin to start with
	int disabled;	// Skipped if set
	int datalen;	// Number of bytes in 'data' buffer
	rtapi_u8 data[MAX_PKT_LEN]; // PDU: 2-byte header, MAX_MSG_LEN payload, 2-byte CRC
} hm2_modbus_cmd_t;

typedef struct {
	char		name[HAL_NAME_LEN+1];		// What we call ourselves (hm2_modbus.X)
	char		uart[HAL_NAME_LEN+1];		// The PktUART we attached to (like hm2_5i25.Y.pktuart.Z)

	hm2_modbus_mbccb_header_t *mbccb;		// Modbus command control binary
	ssize_t		mbccbsize;						// Buffer/file size
	const hm2_modbus_mbccb_cmds_t *initptr;	// Pointer to mbccb init section
	hm2_modbus_mbccb_cmds_t *cmdsptr;		// Pointer to mbccb cmds section
	const rtapi_u8 *dataptr;				// Pointer to mbccb data section
	unsigned	ninit;		// Total number of inits
	unsigned	ncmds;		// Total number of commands
	unsigned	npins;		// Total number of pins

	hm2_modbus_hal_t *hal;	// HAL pins and params
	hm2_modbus_cmd_t *_init;	// List of inits sent initially
	hm2_modbus_cmd_t *_cmds;	// List of commands sent in loop
	hm2_modbus_cmd_t *cmds;	// List of commands sent in loop *or* inits at startup
	unsigned	cmdidx;		// Where are we in the list

	rtapi_u32	baudrate;
	rtapi_u32	parity;
	rtapi_u32	stopbits;
	rtapi_u32	txdelay;	// Local copies to track param changes
	rtapi_u32	rxdelay;
	rtapi_u32	drvdelay;

	int			state;		// State-machine state
	int			ignoredata;	// Ignore received packet is set

	unsigned	frameidx;	// Which frame we are handling (should only ever be 0)
	rtapi_u32	fsizes[16];	// See HM2_PKTUART_RCR_* defines for bit-fields
	rtapi_u32	rxdata[256];	// 0x400 bytes, 0x100 32-bit words

	long		interval;	// Interval timer (for tracking update-hz rate)
	long		timeout;	// Timeout timer for commands
} hm2_modbus_inst_t;

typedef struct {
	int ninsts;
	hm2_modbus_inst_t *insts;
} hm2_modbus_t;


static int comp_id = -1;	// HAL component ID
static hm2_modbus_t mb;		// Our instances

// Forward declarations
static int parse_data_frame(hm2_modbus_inst_t *inst);
static int build_data_frame(hm2_modbus_inst_t *inst);
static int do_setup(hm2_modbus_inst_t *inst);
static rtapi_u16 crc_modbus(rtapi_u8 *buffer, size_t len);


/*
 * The PktUART interfaces to be linked to the hm2_modbus module. This consists
 * of a list of HAL names like:
 *   ports="hm2_7i95.0.pktuart.0","hm2_5i25.0.pktuart.7"
 */
static char *ports[MAX_PORTS];
RTAPI_MP_ARRAY_STRING(ports, MAX_PORTS, "PktUART HAL names");

/*
 * The Modbus configuration and command structure files for each PktUART
 * interface to be read by the hm2_modbus module. This is a list of absolute
 * path file names like:
 *   files="/usr/share/linuxcnc/modbus/spindle.mbccb","/home/test/xyz.mbccb"
 */
static char *mbccbs[MAX_PORTS];
RTAPI_MP_ARRAY_STRING(mbccbs, MAX_PORTS, "Binary Modbus configuration and command sequence absolute path file names");

/*
 * Set the message level for debugging purpose. This has the (side-)effect that
 * all modules within this process will start spitting out messages at the
 * requested level.
 * The upstream message level is not touched if debug == -1.
 */
static int debug = -1;
RTAPI_MP_INT(debug, "Set message level for debugging purpose [0...5] where 0=none and 5=all (default: -1; upstream defined)");


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

//
// Note on do_setup():
// It will bypass the queue commands and send changes immediately. This may be
// detrimental to consistency, so it should not be used lightly when running in
// the normal loop.
// Returns zero (0) is no change is required. Positive one (1) is returned if a
// configuration change was performed. A negative errno is returned on error.
//
static int do_setup(hm2_modbus_inst_t *inst)
{
	int r;

	if    (inst->txdelay  == inst->hal->txdelay
		&& inst->rxdelay  == inst->hal->rxdelay
		&& inst->drvdelay == inst->hal->drvdelay) return 0;

	if(inst->hal->txdelay > 0xFF) inst->hal->txdelay = 0xFF;
	if(inst->hal->rxdelay > 0xFF) inst->hal->rxdelay = 0xFF;
	inst->txdelay  = inst->hal->txdelay;
	inst->rxdelay  = inst->hal->rxdelay;
	inst->drvdelay = inst->hal->drvdelay;

	if((r = hm2_pktuart_setup_rx(inst->uart, inst->baudrate, inst->baudrate *2, inst->parity, inst->rxdelay, 1, 1)) < 0) {
		MSG_ERR("%s: error: RX setup problem: error=%d\n", inst->name, r);
		return r;
	}
	// FIXME: Stopbits
	if((r = hm2_pktuart_setup_tx(inst->uart, inst->baudrate, inst->parity, inst->txdelay, 1, 1, inst->drvdelay)) < 0) {
		MSG_ERR("%s: error: TX setup problem: error=%d\n", inst->name, r);
		return r;
	}

	return 1;
}

static int send_modbus_pkt(hm2_modbus_inst_t *inst)
{
	hm2_modbus_cmd_t *ch = &(inst->cmds[inst->cmdidx]);

	// The +2 is because the buffer contains the header and the message data.
	// There must be room for the CRC. The buffer has the room allocated, now
	// it is checked to be available.
	if(ch->datalen >= MAX_MSG_LEN + 2) {
		MSG_ERR("%s: error: Data buffer overflow in send_modbus_pkt(), channel %d\n", inst->name, inst->cmdidx);
		return -EMSGSIZE;
	}

	// Append the CRC to the packet
	rtapi_u16 checksum = crc_modbus(ch->data, ch->datalen);
	ch->data[ch->datalen++] = checksum & 0xff;
	ch->data[ch->datalen++] = (checksum >> 8) & 0xff;

#ifdef DEBUG
	MSG_DBG("Sending to '%s' %i bytes", inst->name, ch->datalen);
	for(int i = 0; i < ch->datalen; i++) MSG_DBG(" 0x%02x", ch->data[i]);
	MSG_DBG("\n");
#endif

	rtapi_u8  frames = 1;
	rtapi_u16 fsizes[1] = { ch->datalen };
	return hm2_pktuart_send(inst->uart, ch->data, &frames, fsizes);
}

//
// Write invalid command data on the current channel to force re-send on next
// round
//
static inline void force_resend(hm2_modbus_inst_t *inst)
{
	inst->cmds[inst->cmdidx].data[1] = 0xff;
}

//
// Change state in the state machine
// Performs:
// - old state exit actions
// - change state
// - new state entry actions
//
static inline void set_state(hm2_modbus_inst_t *inst, int newstate)
{
	// Exit actions
	switch(inst->state) {
	}

	inst->state = newstate;	// Switch over

	// Entry actions
	switch(newstate) {
	case STATE_RESET_WAIT:
		// Reset wait uses a different timeout
		inst->timeout = 25000000;	// 25 milliseconds
		break;
	case STATE_START:
		inst->ignoredata = 0;	// Re-enable data handling
		/* Fallthrough */
	default:
		// Always set the timeout counter on state change
		inst->timeout = inst->cmds[inst->cmdidx].cmd.timeout * 1000;
		break;
	}
}

//
// Perform a queued reset.
// The PktUART RX and TX are cleared and reset in the next period.
//
static inline void queue_reset(hm2_modbus_inst_t *inst)
{
	hm2_pktuart_queue_reset(inst->uart);
	set_state(inst, STATE_RESET_WAIT);
}

static void do_timeout(hm2_modbus_inst_t *inst)
{
	if(inst->timeout < 0) {
		MSG_DBG("\n %lu TIMEOUT_RESET %i\n", inst->timeout, inst->state);
		queue_reset(inst);
		*(inst->hal->lasterror) = ETIME;
		*(inst->hal->fault) = 1;
		*(inst->hal->faultcmd) = inst->cmdidx;
		force_resend(inst);
	}
	MSG_DBG("%lu timeout %i\r", inst->timeout, inst->state);
}

static inline int next_command(hm2_modbus_inst_t *inst)
{
	// While running the init list
	if(inst->cmds == inst->_init) {
		inst->cmdidx++;
		if(inst->cmdidx >= inst->ninit) {
			// The inits have have ended, move to commands
			inst->cmds = inst->_cmds;
			inst->cmdidx = 0;
			return 1;
		}
		return 0;
	}

	// Find the next non-disabled command
	do {
		inst->cmdidx++;
	} while(inst->cmdidx < inst->ncmds && inst->cmds[inst->cmdidx].disabled);

	if(inst->cmdidx >= inst->ncmds) {
		// The cycle has ended, reset the command counter
		inst->cmdidx = 0;
		return 1;
	}
	return 0;
}

//
// The main process Modbus state-machine.
//
// It is essentially a synchronous communication machine that:
//   a) sends a command
//   b) wait for send complete
//   c) wait for answer
//   d) read and handle reply
// That way we can assure no wrong replies being attached to a sent command.
//
static void process(void *arg, long period)
{
	hm2_modbus_inst_t *inst = (hm2_modbus_inst_t *)arg;

	int r;

	rtapi_u32 rxstatus = hm2_pktuart_get_rx_status(inst->uart);
	rtapi_u32 txstatus = hm2_pktuart_get_tx_status(inst->uart);

	inst->interval -= period;	// Keep counting nanoseconds
	inst->timeout  -= period;

	switch(inst->state) {
	case STATE_START:
#ifdef DEBUG
		{
			static rtapi_u32 oldtx = 0, oldrx = 0;
			if(oldrx != rxstatus || oldtx != txstatus) {
				MSG_DBG("START txstatus=0x%08x rxstatus=0x%08x\n", txstatus, rxstatus);
				oldrx = rxstatus;
				oldtx = txstatus;
			}
		}
#endif
		// Check for received data
		// FIXME: This data would be out-of-band. It should never happen,
		//        unless there is more than one master on the bus.
		if(rxstatus & HM2_PKTUART_RXMODE_HASDATA) {
			inst->ignoredata = 1;
			set_state(inst, STATE_WAIT_FOR_DATA_FRAME);
			MSG_WARN("%s: warning: Out-of-band data received. Might not belong to current command %d\n", inst->name, inst->cmdidx);
			// Jump to the next state. There is no point in delaying queueing
			// the necessary read(s).
			goto wait_for_data_frame;
		}

		// Are we handling init commands?
		if(inst->cmds == inst->_init) {
			// Yes, prepare and send
retry_next_init:
			hm2_modbus_cmd_t *ch = &inst->cmds[inst->cmdidx];
			if(0 == ch->cmd.func) {
				// This is a delay command
				// The timeout will be set in set_state()
				set_state(inst, STATE_WAIT_FOR_TIMEOUT);
			} else {
				const rtapi_u8 *dptr = inst->dataptr + ch->cmd.dataptr;
				memcpy(ch->data, dptr + 1, *dptr);	// Packet is prepared as data
				if((r = send_modbus_pkt(inst)) < 0) {	// This will attach CRC
					// Failure means we just move to the next
					MSG_ERR("%s: error: Failed to send init command %u\n", inst->name, inst->cmdidx);
					if(!next_command(inst))	// Until we wrap the command list
						goto retry_next_init;

					// We get to do commands next time around because
					// next_command() switches pointers on end of inits.
					break;
				}
				set_state(inst, STATE_WAIT_FOR_SEND_BEGIN);
			}
			break;
		}

		// Handling normal commands loop

		// At the start of a cycle, test the interval timer.
		if(!inst->cmdidx) {
			long iv = (long)inst->hal->interval;
			if(iv) {
				if(inst->interval > 0)
					break;	// Timer still running
				// Reset the timer. The cycle is now starting.
				inst->interval = iv * 1000;
			} else {
				inst->interval = 0;
			}

			// We are at the start of a cycle. If the first command message is
			// disabled, then try the next.
			if(inst->cmds[inst->cmdidx].disabled) {
				// If the next command is again the first, then we have no
				// messages we can send.
				if(next_command(inst)) {
					*(inst->hal->lasterror) = ENODATA;
					*(inst->hal->fault) = 1;
					*(inst->hal->faultcmd) = 0;
					break;
				}
			}
		}

		// See if the comms params have changed and if so, change them.
		if((r = do_setup(inst)) < 0) {
			MSG_DBG("Failed port setup (%d), resetting\n", r);
			queue_reset(inst);
			// Tampering with the txdelay ensures we really try to setup again
			// in the next round because the "current" txdelay is not what the
			// hal param says it should be.
			inst->txdelay ^= 0x55;
			break;
		}
		if(r > 0) {
			// The setup changed, wait one period to flush
			break;
		}

		// No incoming data, we are free to send.
		// Cycle through the commands until one is found to have data that
		// needs to be sent.
		do {
			if(0 == inst->cmds[inst->cmdidx].cmd.func) {
				// This is a delay command
				// The timeout will be set in set_state()
				set_state(inst, STATE_WAIT_FOR_TIMEOUT);
				break;
			}

			if((r = build_data_frame(inst)) < 0) {
				// We cannot recover from data frames that cannot be build. The
				// next round would end in the same error. Therefore, disabling
				// the command is our only option.
				MSG_ERR("%s: error: Build data frame failed ccommand %d, disabling\n", inst->name, inst->cmdidx);
				inst->cmds[inst->cmdidx].disabled = 1;
				continue; // Just try next command
			}
			if(r) { // if data has changed
				if((r = send_modbus_pkt(inst)) < 0) {
					if(r == -EMSGSIZE) {
						// We cannot recover if the error occurred because the
						// buffer overflowed (no room for CRC). Resetting would
						// just give us an infinite stream of errors because it
						// would overflow in the next round too.
						// We disable this particular command so it will not
						// give problems again. It will probably cause havoc in
						// the application, but that should be OK.
						MSG_ERR("%s: error: Command %d disabled\n", inst->name, inst->cmdidx);
						inst->cmds[inst->cmdidx].disabled = 1;
						continue; // Just try next command
					}
					MSG_ERR("%s: error: Send PDU failed (error %d) command %d, resetting\n", inst->name, r, inst->cmdidx);
					queue_reset(inst);
					force_resend(inst);
					next_command(inst);	// Or we we may get stuck on this command
					break;
				} else {
					set_state(inst, STATE_WAIT_FOR_SEND_BEGIN);
					break;
				}
			}
			// Nothing to do for this command, try next
		} while(!next_command(inst));	// Until we wrap the command list

		break;

	case STATE_WAIT_FOR_TIMEOUT:
		MSG_DBG("WAIT_FOR_TIMEOUT RX 0x%08x TX 0x%08x\n", txstatus, rxstatus);
		// Check for received data
		// FIXME: This data would be out-of-band. It should never happen,
		//        unless there is more than one master on the bus.
		if(rxstatus & HM2_PKTUART_RXMODE_HASDATA) {
			inst->ignoredata = 1;
			set_state(inst, STATE_WAIT_FOR_DATA_FRAME);
			MSG_WARN("%s: warning: Out-of-band data received in WAIT_FOR_TIMEOUT in command %d\n", inst->name, inst->cmdidx);
			// Jump to the next state. There is no point in delaying queueing
			// the necessary read(s).
			goto wait_for_data_frame;
		}

		if(inst->timeout < 0) {
			// Timeout of delay, proceed to next command
			next_command(inst);
			set_state(inst, STATE_START);
			*(inst->hal->fault) = 0;
		}
		break;

	case STATE_WAIT_FOR_SEND_BEGIN:
		// Single cycle delay to allow for queued flush
		MSG_DBG("WAIT_FOR_SEND_BEGIN txstatus=0x%08x rxstatus=0x%08x\n", txstatus, rxstatus);
		set_state(inst, STATE_WAIT_FOR_SEND_COMPLETE);
		break;

	case STATE_WAIT_FOR_SEND_COMPLETE:
		MSG_DBG("WAIT_FOR_SEND_COMPLETE txstatus=0x%08x rxstatus=0x%08x\n", txstatus, rxstatus);
		if(!(txstatus & HM2_PKTUART_TXMODE_TXBUSY)) {
			if(!inst->cmds[inst->cmdidx].cmd.mbid && !(inst->cmds[inst->cmdidx].cmd.func & 0x80)) {
				// Broadcasts have no reply
				// Unless the equipment is badly behaved and we have set the
				// error-bit in the function code.
				set_state(inst, STATE_START);
			} else {
				set_state(inst, STATE_WAIT_FOR_DATA_FRAME);
			}
		} else {
			do_timeout(inst);
		}
		break;

	case STATE_WAIT_FOR_DATA_FRAME:
wait_for_data_frame:
		MSG_DBG("WAIT_FOR_DATA_FRAME rxstatus=0x%08x\n", rxstatus);
		if(!HM2_PKTUART_RXMODE_NFRAMES_VAL(rxstatus)) {
			// No data yet, continue waiting.
			do_timeout(inst);
			break;
		}
		// Got frame(s) in receive buffer
		inst->frameidx = 0;
		memset(inst->fsizes, 0, sizeof(inst->fsizes));
		// Get the frame size(s)
		if((r = hm2_pktuart_queue_get_frame_sizes(inst->uart, inst->fsizes)) < 0)
			MSG_ERR("%s: error: hm2_pktuart_queue_get_frame_sizes() returned an error: %d\n", inst->name, r);
		set_state(inst, STATE_WAIT_FOR_FRAME_SIZES);
		break;

	case STATE_WAIT_FOR_FRAME_SIZES:
	case STATE_FETCH_MORE_DATA:
		MSG_DBG("WAIT_FOR_FRAME_SIZES Index %u Frames 0x%04x 0x%04x 0x%04x 0x%04x\n",
			inst->frameidx, inst->fsizes[0], inst->fsizes[1], inst->fsizes[2], inst->fsizes[3]);
		//do_timeout(inst);
		if((inst->fsizes[inst->frameidx] & (HM2_PKTUART_RCR_ERROROVERRUN | HM2_PKTUART_RCR_ERRORSTARTBIT))
			||  (HM2_PKTUART_RCR_NBYTES_VAL(inst->fsizes[inst->frameidx]) > MAX_PKT_LEN)) { // indicates an overrun
			MSG_DBG("RESET\n");
			queue_reset(inst);
			break;
		}
		r = hm2_pktuart_queue_read_data(inst->uart, inst->rxdata, HM2_PKTUART_RCR_NBYTES_VAL(inst->fsizes[inst->frameidx]));
		if(r < 0)
			MSG_ERR("%s: error: hm2_pktuart_queue_read_data() returned an error: %d\n", inst->name, r);	// What to do...
		// The above queued read doesn't resolve until, at least, the next cycle.
		set_state(inst, STATE_WAIT_A_BIT);
		break;

	case STATE_WAIT_A_BIT:
		set_state(inst, STATE_WAIT_FOR_DATA);
		break;

	case STATE_WAIT_FOR_DATA:
		MSG_DBG("WAIT_FOR_DATA\n");
		if(!inst->ignoredata)
			parse_data_frame(inst);
		if(HM2_PKTUART_RCR_NBYTES_VAL(inst->fsizes[++(inst->frameidx)]) > 0) {
			// FIXME: We should never get multiple replies. Only one
			// outstanding packet should be waiting for a reply.
			MSG_WARN("%s: warning: Multiple data packets as reply? Maybe out-of-band? Command %d\n", inst->name, inst->cmdidx);
			inst->ignoredata = 1;
			set_state(inst, STATE_FETCH_MORE_DATA);
		} else {
			set_state(inst, STATE_WAIT_FOR_RX_CLEAR);
		}
		break;

	case STATE_WAIT_FOR_RX_CLEAR:
		MSG_DBG("WAIT_FOR_RX_CLEAR rxstatus=0x%08x\r", rxstatus);
		if(rxstatus & HM2_PKTUART_RXMODE_HASDATA) {
			do_timeout(inst);
			break;
		}
		MSG_DBG("\n");
		set_state(inst, STATE_START);
		*(inst->hal->fault) = 0;
		next_command(inst);
		break;

	case STATE_RESET_WAIT:
		if(inst->timeout < 0)
			set_state(inst, STATE_START);
		break;

	default:
		MSG_ERR("%s: error: Unknown state (%d) in process(), setting START state\n", inst->name, inst->state);
		*(inst->hal->fault) = 1;
		*(inst->hal->lasterror) = EINVAL;
		*(inst->hal->faultcmd) = 0;
		set_state(inst, STATE_START);
		break;
	}
}

//
// Build a frame one byte at a time.
// The ch_append8() returns:
// *  0        On success
// * -EMSGSIZE On failure
// * +1        On data change
//
static inline int ch_append8(hm2_modbus_cmd_t *ch, rtapi_u8 v)
{
	int r = 0;
	// The +2 is from the header (address and command), already written to the
	// buffer. The CRC is not counted while building the frame. The data buffer
	// has four extra bytes allocated to store header and CRC.
	if(ch->datalen >= MAX_MSG_LEN + 2)
		return -EMSGSIZE;
	if(ch->data[ch->datalen] != v) r = 1; // flag data changed
	ch->data[ch->datalen] = v;
	ch->datalen++;
	return r;
}

// Abort when -1 is returned and continue on 0 and +1
// Positive returns are summed. Negative drops out of the function.
#define CHK_RV(x)	do { \
						int rv = (x); \
						if(rv < 0) \
							return rv; \
						r += rv; \
					} while(0)
static inline int ch_append16(hm2_modbus_cmd_t *ch, rtapi_u16 v)
{
	int r = 0;
	CHK_RV(ch_append8(ch, (rtapi_u8)(v >> 8)));
	CHK_RV(ch_append8(ch, (rtapi_u8)(v & 0xFF)));
	return r;
}

static inline int ch_append32(hm2_modbus_cmd_t *ch, const mb_types_u *v)
{
	int r = 0;
	switch(ch->cmd.type & _MB_XXXX_MASK) {
#if RTAPI_BIG_ENDIAN
	case _MB_ABCD:
#else
	case _MB_DCBA:
#endif
		CHK_RV(ch_append8(ch, v->b[0]));
		CHK_RV(ch_append8(ch, v->b[1]));
		CHK_RV(ch_append8(ch, v->b[2]));
		CHK_RV(ch_append8(ch, v->b[3]));
		break;
#if RTAPI_BIG_ENDIAN
	case _MB_BADC:
#else
	case _MB_CDAB:
#endif
		CHK_RV(ch_append8(ch, v->b[1]));
		CHK_RV(ch_append8(ch, v->b[0]));
		CHK_RV(ch_append8(ch, v->b[3]));
		CHK_RV(ch_append8(ch, v->b[2]));
		break;
#if RTAPI_BIG_ENDIAN
	case _MB_CDAB:
#else
	case _MB_BADC:
#endif
		CHK_RV(ch_append8(ch, v->b[2]));
		CHK_RV(ch_append8(ch, v->b[3]));
		CHK_RV(ch_append8(ch, v->b[0]));
		CHK_RV(ch_append8(ch, v->b[1]));
		break;
#if RTAPI_BIG_ENDIAN
	case _MB_DCBA:
#else
	case _MB_ABCD:
#endif
		CHK_RV(ch_append8(ch, v->b[3]));
		CHK_RV(ch_append8(ch, v->b[2]));
		CHK_RV(ch_append8(ch, v->b[1]));
		CHK_RV(ch_append8(ch, v->b[0]));
		break;
	default:
		return -EINVAL;
	}
	return r;
}

static inline int ch_init(hm2_modbus_cmd_t *ch)
{
	int r = 0;
	ch->datalen = 0;
	CHK_RV(ch_append8(ch, ch->cmd.mbid));
	CHK_RV(ch_append8(ch, ch->cmd.func & 0x7f));
	return r;
}

static int build_data_frame(hm2_modbus_inst_t *inst)
{
	hm2_modbus_cmd_t *ch = &(inst->cmds[inst->cmdidx]);
	hm2_modbus_hal_t *hal = inst->hal;
	rtapi_u8 acc = 0;
	int r = 0;
	int p = ch->pinref;
	mb_types_u val32;

	MSG_DBG("Building PDU 0x%02x 0x%04x start pin %i\n", ch->cmd.func & 0x7f, ch->cmd.addr, p);

	if((r = ch_init(ch)) < 0) {
		MSG_ERR("%s: error: Failed to initialize frame, channel %d\n", inst->name, inst->cmdidx);
		return r;
	}

	switch(ch->cmd.func & 0x7f) {
	case MBCMD_R_COILS: // Read coils
	case MBCMD_R_INPUTS: // Read discrete inputs
	case MBCMD_R_REGISTERS: // Read holding registers
	case MBCMD_R_INPUTREGS: // Read input registers
		CHK_RV(ch_append16(ch, ch->cmd.addr));
		if((ch->cmd.func & 0x7f) == MBCMD_R_REGISTERS && (ch->cmd.type & _MB_XXXX_MASK)) {
			// Compound values are two registers large
			CHK_RV(ch_append16(ch, ch->cmd.pincnt * 2));
		} else {
			CHK_RV(ch_append16(ch, ch->cmd.pincnt));
		}
		r += 1; // trigger a read PDU every time
		break;
	case MBCMD_W_COIL: // Write single coil
		CHK_RV(ch_append16(ch, ch->cmd.addr));
		CHK_RV(ch_append16(ch, hal->pins[p]->b ? 0xFF00 : 0x0000));
		break;
	case MBCMD_W_REGISTER: // Write single register
		CHK_RV(ch_append16(ch, ch->cmd.addr));
		switch(ch->cmd.type) {
		case HAL_U32:
			CHK_RV(ch_append16(ch, (rtapi_u16)hal->pins[p]->u));
			break;
		case HAL_S32:
			// FIXME: We have pins s32, scale and scaled. We only use s32.
			val32.s = hal->pins[p]->s;
			// Clamp the output
			if(val32.s < -32768) val32.s = -32768;
			if(val32.s > +32767) val32.s = +32767;
			CHK_RV(ch_append16(ch, (rtapi_s16)val32.s));
			break;
		case HAL_FLOAT:
			if(myfabs(hal->pins[p+1]->f) >= EPSILON) {
				// (pin - offset) / scale
				CHK_RV(ch_append16(ch, (rtapi_u16)((hal->pins[p+0]->f - hal->pins[p+2]->f) / hal->pins[p+1]->f)));
			}
			break;
		default:
			break;
		}
		break;
	case MBCMD_W_COILS: // Write multiple coils
		CHK_RV(ch_append16(ch, ch->cmd.addr));
		CHK_RV(ch_append16(ch, ch->cmd.pincnt));
		CHK_RV(ch_append8(ch, (ch->cmd.pincnt + 7) / 8));
		for(unsigned i = 0; i < ch->cmd.pincnt; i++) {
			if(hal->pins[p++]->b)
				acc |= 1u << (i % 8);
			if((i % 8) == 7 || i == ch->cmd.pincnt - 1u) { // time for the next byte
				CHK_RV(ch_append8(ch, acc));
				acc = 0;
			}
		}
		break;
	case MBCMD_W_REGISTERS: // write multiple holding registers
		CHK_RV(ch_append16(ch, ch->cmd.addr));
		if(ch->cmd.type & _MB_XXXX_MASK) {
			// Compound values are two registers large
			CHK_RV(ch_append16(ch, ch->cmd.pincnt * 2));
			CHK_RV(ch_append8(ch, ch->cmd.pincnt * 4));
		} else {
			CHK_RV(ch_append16(ch, ch->cmd.pincnt));
			CHK_RV(ch_append8(ch, ch->cmd.pincnt * 2));
		}
		for(unsigned i = 0; i < ch->cmd.pincnt; i++) {
			switch(ch->cmd.type) {
			case HAL_U32:
				CHK_RV(ch_append16(ch, (rtapi_u16)hal->pins[p]->u));
				p++;
				break;
			case HAL_S32:
				// FIXME: We have pins s32, scale and scaled. We only use s32.
				val32.s = hal->pins[p]->s;
				// Clamp the pin
				if(val32.s < -32768) val32.s = -32768;
				if(val32.s > +32767) val32.s = +32767;
				CHK_RV(ch_append16(ch, (rtapi_s16)val32.s));
				p += 3;
				break;
			case HAL_FLOAT:
				if(myfabs(hal->pins[p+1]->f) >= EPSILON) {
					// (value - offset) * scale
					val32.f = (hal->pins[p+0]->f - hal->pins[p+2]->f) / hal->pins[p+1]->f;
					// Clamp
					if(val32.f > +32767.0)	val32.f = +32767.0;
					if(val32.f < -32768.0)	val32.f = -32768.0;
					CHK_RV(ch_append16(ch, (rtapi_u16)val32.f));
				}
				p += 3;
				break;
			case HAL_FLOAT_ABCD:
			case HAL_FLOAT_BADC:
			case HAL_FLOAT_CDAB:
			case HAL_FLOAT_DCBA:
				val32.f = hal->pins[p]->f;	// Convert double to float
				CHK_RV(ch_append32(ch, &val32));
				p++;
				break;
			case HAL_U32_ABCD:
			case HAL_U32_BADC:
			case HAL_U32_CDAB:
			case HAL_U32_DCBA:
				val32.u = hal->pins[p]->u;
				CHK_RV(ch_append32(ch, &val32));
				p++;
				break;
			case HAL_S32_ABCD:
			case HAL_S32_BADC:
			case HAL_S32_CDAB:
			case HAL_S32_DCBA:
				val32.s = hal->pins[p]->s;
				CHK_RV(ch_append32(ch, &val32));
				p++;
				break;
			default:
				// Oops...
				break;
			}
		}
		break;
	default:
		MSG_ERR("%s: error: Unknown Modbus instruction, channel %d\n", inst->name, inst->cmdidx);
		return -EINVAL;
	}
	return r;
}
#undef CHK_RV

static int test_bytecount(const hm2_modbus_inst_t *inst, const rtapi_u8 *bytes, unsigned pkt_len, rtapi_u32 type)
{
	// The byte count cannot be zero and there is a hard limit on
	// coils/inputs/registers read in one pass.
	// 2000 bits  : n = 125 bytes (commands 1 and 2)
	//  125 words : n = 250 bytes (commands 3 and 4)
	//   62 dwords: n = 248 bytes (commands 3 and 4)
	unsigned mini, maxi;
	if(type & _MB_XXXX_MASK) {	// Compound types are multiple of 4 bytes (2 regs)
		mini = 4;	// Only mod 4 counts
		maxi = 248;	// 4 * 62
	} else if(type == HAL_BIT) {
		mini = 1;	// Any count allowed
		maxi = 125;	// 2000 / 8
	} else {
		mini = 2;	// Only even counts
		maxi = 250;	// 2 * 125
	}
	if(bytes[2] < mini || bytes[2] > maxi) {
		MSG_ERR("%s: error: Invalid byte count %u in received PDU not in [%u, %u], cmd %u\n", inst->name, bytes[2], mini, maxi, bytes[1]);
		return -1;
	}
	// Byte count cannot be odd with words.
	if(0 != bytes[2] % mini) {
		MSG_ERR("%s: error: Invalid odd byte count %u in received PDU, expected mod %u, cmd %u\n", inst->name, bytes[2], mini, bytes[1]);
		return -1;
	}
	// The byte count must match the PDU's content size.
	// The -4 is subtracting the mbid, function code and CRC.
	if(bytes[2] > pkt_len - 4) {
		MSG_ERR("%s: error: Byte count in received PDU too large (%u > %u), cmd %u\n", inst->name, bytes[2], pkt_len - 4, bytes[1]);
		return -1;
	}
	if(bytes[2] < pkt_len - 4) {
		MSG_ERR("%s: error: Byte count in received PDU too small (%u < %u), cmd %u\n", inst->name, bytes[2], pkt_len - 4, bytes[1]);
		return -1;
	}
	return 0;
}

static inline mb_types_u get32(rtapi_u8 *b, unsigned order)
{
	mb_types_u v;
	switch(order) {
	default:
#if RTAPI_BIG_ENDIAN
	case _MB_ABCD:
#else
	case _MB_DCBA:
#endif
		v.b[0] = b[0];
		v.b[1] = b[1];
		v.b[2] = b[2];
		v.b[3] = b[3];
		break;
#if RTAPI_BIG_ENDIAN
	case _MB_BADC:
#else
	case _MB_CDAB:
#endif
		v.b[0] = b[1];
		v.b[1] = b[0];
		v.b[2] = b[3];
		v.b[3] = b[2];
		break;
#if RTAPI_BIG_ENDIAN
	case _MB_CDAB:
#else
	case _MB_BADC:
#endif
		v.b[0] = b[2];
		v.b[1] = b[3];
		v.b[2] = b[0];
		v.b[3] = b[1];
		break;
#if RTAPI_BIG_ENDIAN
	case _MB_DCBA:
#else
	case _MB_ABCD:
#endif
		v.b[0] = b[3];
		v.b[1] = b[2];
		v.b[2] = b[1];
		v.b[3] = b[0];
		break;
	}
	return v;
}

static int parse_data_frame(hm2_modbus_inst_t *inst)
{
	hm2_modbus_cmd_t *ch = &(inst->cmds[inst->cmdidx]);
	hm2_modbus_hal_t hal = *inst->hal;
	rtapi_u32 *data = inst->rxdata;
	unsigned rxcount = HM2_PKTUART_RCR_NBYTES_VAL(inst->fsizes[inst->frameidx]);
	int w = 0;
	int b = 0;
	int p;
	rtapi_u8 bytes[MAX_PKT_LEN] = {};
	rtapi_u16 checksum;
	mb_types_u val32;

	if(rxcount > sizeof(bytes)) {
		MSG_ERR("%s: error: Received PDU larger than buffer (%u > %zu), truncating\n", inst->name, rxcount, sizeof(bytes));
		rxcount = sizeof(bytes);
	}

	// Worst case PDU size:
	// 1 byte slave address
	// 1 byte command
	// 1 byte exception code
	// 2 bytes CRC
	if(rxcount < 5) {
		MSG_ERR("%s: error: Received PDU too small, size=%u\n", inst->name, rxcount);
		force_resend(inst);
		return -1;
	}

	MSG_DBG("Return PDU is");
	for(unsigned i = 0; i < rxcount; i++) {
		bytes[i] = (data[w] >> b) & 0xFF;
		MSG_DBG(" %02x", bytes[i]);
		if((b += 8) >= 32) { b = 0; w++; }
	}
	MSG_DBG("\n");

	if(bytes[0] != ch->cmd.mbid) {
		MSG_ERR("%s: error: Modbus device address mismatch: got 0x%02x, expected 0x%02x\n", inst->name, bytes[0], ch->cmd.mbid);
		// FIXME: Actually don't know whether we should resend here...
		return -1;
	}

	if((bytes[1] & 0x7f ) != (ch->cmd.func & 0x7f)) {
		MSG_ERR("%s: error: Call/response function number mismatch: got 0x%02x, expected 0x%02x)\n", inst->name, ch->cmd.func & 0x7f, bytes[1]);
		force_resend(inst);
		return -1;
	}

	checksum = crc_modbus(bytes, rxcount - 2);
	rtapi_u16 retcrc = ((rtapi_u16)bytes[rxcount - 1] << 8) | bytes[rxcount - 2];
	if(retcrc != checksum) {
		MSG_ERR("%s: error: Modbus checksum error: got 0x%04x, expected 0x%04x\n", inst->name, retcrc, checksum);
		force_resend(inst);
		return -1;
	}

	p = ch->pinref;

	switch(bytes[1]) {
	case MBCMD_R_COILS: // read coils
	case MBCMD_R_INPUTS: // read inputs
		if(test_bytecount(inst, bytes, rxcount, ch->cmd.type) < 0) {
			force_resend(inst);
			break;
		}
		w = 3;	// First data return byte {mbid, func, bytecount, ...}
		b = 0;
		for(unsigned i = 0; i < ch->cmd.pincnt; i++) {
			hal.pins[p++]->b = !!(bytes[w] & (1u << b));
			if(++b >= 8) { b = 0; w++; }
		}
		break;
	case MBCMD_R_REGISTERS: // read holding registers
		// Compound values must have bytecount % 4 == 0
		if((ch->cmd.type & _MB_XXXX_MASK) && (bytes[2] & 0x3)) {
			force_resend(inst);
			break;
		}
		/* Fallthrough */
	case MBCMD_R_INPUTREGS: // read input registers
		if(test_bytecount(inst, bytes, rxcount, ch->cmd.type) < 0) {
			force_resend(inst);
			break;
		}
		w = 3;
		for(int i = 0; i < bytes[2] / 2; i++) {
			switch((int)ch->cmd.type) {
			case HAL_U32:
				hal.pins[p]->u = ((rtapi_u32)bytes[w] << 8) + bytes[w + 1];
				w += 2;
				p++;
				break;
			case HAL_S32:
				hal.pins[p]->s = val32.s = 256 * (rtapi_s32)(rtapi_s8)bytes[w] + bytes[w + 1];
				w += 2;
				// .scaled = pin * .scale
				hal.pins[p+2]->f = val32.s * hal.pins[p+1]->f;
				p += 3;
				break;
			case HAL_FLOAT:
				hal.pins[p]->f = (256 * (rtapi_s8)bytes[w] + bytes[w + 1]) * hal.pins[p+1]->f + hal.pins[p+2]->f;
				w += 2;
				p += 3;
				break;
			case HAL_FLOAT_ABCD:
			case HAL_FLOAT_BADC:
			case HAL_FLOAT_CDAB:
			case HAL_FLOAT_DCBA:
				if(!(i & 1)) {
					val32 = get32(bytes, ch->cmd.type & _MB_XXXX_MASK);
					hal.pins[p]->f = val32.f;
					p++;
				}
				break;
			case HAL_U32_ABCD:
			case HAL_U32_BADC:
			case HAL_U32_CDAB:
			case HAL_U32_DCBA:
				if(!(i & 1)) {
					val32 = get32(bytes, ch->cmd.type & _MB_XXXX_MASK);
					hal.pins[p]->u = val32.u;
					p++;
				}
				break;
			case HAL_S32_ABCD:
			case HAL_S32_BADC:
			case HAL_S32_CDAB:
			case HAL_S32_DCBA:
				if(!(i & 1)) {
					val32 = get32(bytes, ch->cmd.type & _MB_XXXX_MASK);
					hal.pins[p]->s = val32.s;
					p++;
				}
				break;
			default:
				break;
			}
		}
		break;
	// Nothing to do for write commands 5, 6, 15, 16 ??
	case MBCMD_W_COIL:
	case MBCMD_W_REGISTER:
		// The above two should be an echo of the PDU sent
	case MBCMD_W_COILS:
	case MBCMD_W_REGISTERS:
		// The above two should echo the first 4 bytes of the sent frame contents
		break;

	// The following are error codes
	case (128 + MBCMD_R_COILS):		//  1 + error bit
	case (128 + MBCMD_R_INPUTS):	//  2 + error bit
	case (128 + MBCMD_R_REGISTERS):	//  3 + error bit
	case (128 + MBCMD_R_INPUTREGS):	//  4 + error bit
	case (128 + MBCMD_W_COIL):		//  5 + error bit
	case (128 + MBCMD_W_REGISTER):	//  6 + error bit
	case (128 + MBCMD_W_COILS):		// 15 + error bit
	case (128 + MBCMD_W_REGISTERS):	// 16 + error bit
		force_resend(inst);
		if(bytes[2] >= (sizeof(error_codes) / sizeof(*error_codes))) {
			MSG_ERR("%s: error: Modbus error response function %u with invalid error code %u\n", inst->name, bytes[1], bytes[2]);
		} else {
			MSG_ERR("%s: error: Modbus error response function %u error %s (%u)\n", inst->name, bytes[1], error_codes[bytes[2]], bytes[2]);
		}
		return -1;
	default:
		MSG_ERR("%s: error: Unknown or unsupported Modbus function code: 0x%02x (mbid=%u, cmd=%u)\n", inst->name, bytes[1], bytes[0], inst->cmdidx);
		return -1;
	}
	return 0;
}

//
// Adapted from: MODBUS over serial line specification and implementation guide V1.02
// Crc poly x^16 + x^15 + x^2 + 1
// hex 0x8005 (reversed: 0xA001)
//
static rtapi_u16 crc_modbus(rtapi_u8 *buffer, size_t len)
{
	static const rtapi_u8 crctabhi[256] = { // Table of CRC values for high–order byte
		0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
		0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
		0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
		0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
		0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
		0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
		0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
		0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
		0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
		0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
		0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
		0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
		0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
		0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
		0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
		0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40
	};

	static const rtapi_u8 crctablo[256] = { // Table of CRC values for low–order byte
		0x00, 0xc0, 0xc1, 0x01, 0xc3, 0x03, 0x02, 0xc2, 0xc6, 0x06, 0x07, 0xc7, 0x05, 0xc5, 0xc4, 0x04,
		0xcc, 0x0c, 0x0d, 0xcd, 0x0f, 0xcf, 0xce, 0x0e, 0x0a, 0xca, 0xcb, 0x0b, 0xc9, 0x09, 0x08, 0xc8,
		0xd8, 0x18, 0x19, 0xd9, 0x1b, 0xdb, 0xda, 0x1a, 0x1e, 0xde, 0xdf, 0x1f, 0xdd, 0x1d, 0x1c, 0xdc,
		0x14, 0xd4, 0xd5, 0x15, 0xd7, 0x17, 0x16, 0xd6, 0xd2, 0x12, 0x13, 0xd3, 0x11, 0xd1, 0xd0, 0x10,
		0xf0, 0x30, 0x31, 0xf1, 0x33, 0xf3, 0xf2, 0x32, 0x36, 0xf6, 0xf7, 0x37, 0xf5, 0x35, 0x34, 0xf4,
		0x3c, 0xfc, 0xfd, 0x3d, 0xff, 0x3f, 0x3e, 0xfe, 0xfa, 0x3a, 0x3b, 0xfb, 0x39, 0xf9, 0xf8, 0x38,
		0x28, 0xe8, 0xe9, 0x29, 0xeb, 0x2b, 0x2a, 0xea, 0xee, 0x2e, 0x2f, 0xef, 0x2d, 0xed, 0xec, 0x2c,
		0xe4, 0x24, 0x25, 0xe5, 0x27, 0xe7, 0xe6, 0x26, 0x22, 0xe2, 0xe3, 0x23, 0xe1, 0x21, 0x20, 0xe0,
		0xa0, 0x60, 0x61, 0xa1, 0x63, 0xa3, 0xa2, 0x62, 0x66, 0xa6, 0xa7, 0x67, 0xa5, 0x65, 0x64, 0xa4,
		0x6c, 0xac, 0xad, 0x6d, 0xaf, 0x6f, 0x6e, 0xae, 0xaa, 0x6a, 0x6b, 0xab, 0x69, 0xa9, 0xa8, 0x68,
		0x78, 0xb8, 0xb9, 0x79, 0xbb, 0x7b, 0x7a, 0xba, 0xbe, 0x7e, 0x7f, 0xbf, 0x7d, 0xbd, 0xbc, 0x7c,
		0xb4, 0x74, 0x75, 0xb5, 0x77, 0xb7, 0xb6, 0x76, 0x72, 0xb2, 0xb3, 0x73, 0xb1, 0x71, 0x70, 0xb0,
		0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
		0x9c, 0x5c, 0x5d, 0x9d, 0x5f, 0x9f, 0x9e, 0x5e, 0x5a, 0x9a, 0x9b, 0x5b, 0x99, 0x59, 0x58, 0x98,
		0x88, 0x48, 0x49, 0x89, 0x4b, 0x8b, 0x8a, 0x4a, 0x4e, 0x8e, 0x8f, 0x4f, 0x8d, 0x4d, 0x4c, 0x8c,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
	};

	rtapi_u8 crch = 0xff;
	rtapi_u8 crcl = 0xff;
	while(len--) {
		unsigned idx = crcl ^ *buffer++;
		crcl = crch ^ crctabhi[idx];
		crch = crctablo[idx];
	}
	return ((rtapi_u16)crch << 8) | crcl;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                     Mbccb file read and validation                      */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#if !defined(__KERNEL__)
// Userspace file read
static ssize_t read_mbccb(const hm2_modbus_inst_t *inst, const char *fname, hm2_modbus_mbccb_header_t **pmbccb)
{
	if(!pmbccb)
		return -EINVAL;

	*pmbccb = NULL;

	// Open the file
	int fd = open(fname, O_RDONLY);
	if(fd < 0) {
		ssize_t rv = -errno;
		MSG_ERR("%s: error: Failed to open '%s' for reading (error %d)\n", inst->name, fname, errno);
		return rv;
	}

	// Get the file size
	struct stat sb;
	if(fstat(fd, &sb) < 0) {
		ssize_t rv = -errno;
		MSG_ERR("%s: error: Failed to fstat '%s' (error %d)\n", inst->name, fname, errno);
		close(fd);
		return rv;
	}

	// Allocate memory
	*pmbccb = rtapi_kzalloc(sb.st_size, RTAPI_GFP_KERNEL);
	if(!*pmbccb) {
		MSG_ERR("%s: error: Failed to allocate %zd bytes memory for mbccb buffer\n", inst->name, (ssize_t)sb.st_size);
		close(fd);
		return -ENOMEM;
	}

	// Read the entire file
retry_read:
	ssize_t err = read(fd, *pmbccb, sb.st_size);
	if(err < 0) {
		ssize_t rv = -errno;
		if(errno == EINTR)
			goto retry_read;	// Interrupted syscall
		MSG_ERR("%s: error: Failed to read from '%s' (error %d)\n", inst->name, fname, errno);
		rtapi_kfree(*pmbccb);
		*pmbccb = NULL;
		close(fd);
		return rv;
	}
	if(err != (ssize_t)sb.st_size) {
		MSG_ERR("%s: error: Read %zd bytes instead of %zd bytes from '%s', aborting\n", inst->name, err, (ssize_t)sb.st_size, fname);
		rtapi_kfree(*pmbccb);
		*pmbccb = NULL;
		close(fd);
		return -EIO;
	}
	close(fd);
	return err;	// Read it all, return the size
}

#else

// In-kernel file read
static ssize_t read_mbccb(const hm2_modbus_inst_t *inst, const char *fname, hm2_modbus_mbccb_header_t **pmbccb)
{
	if(!pmbccb)
		return -EINVAL;

	struct file *fp;
	*pmbccb = NULL;

	// Open the file
	fp = filp_open(fname, O_RDONLY, 0);
	if(IS_ERR(fp)) {
		MSG_ERR("%s: error: Failed to open '%s' for reading (error %d)\n", inst->name, fname, (int)PTR_ERR(fp));
		// FIXME: is it necessary to check negative?
		return PTR_ERR(fp) < 0 ? PTR_ERR(fp) : -PTR_ERR(fp);
	}

	ssize_t fsize = fp->f_inode->i_size;	// File's inode file size

	// Allocate memory
	*pmbccb = rtapi_kzalloc(fsize, RTAPI_GFP_KERNEL);
	if(!*pmbccb) {
		MSG_ERR("%s: error: Failed to allocate %zd bytes memory for mbccb buffer\n", inst->name, fsize);
		filp_close(fp, NULL);
		return -ENOMEM;
	}

	// Read the entire file
	ssize_t err = kernel_read(fp, *pmbccb, fsize, NULL);
	if(err < 0) {
		MSG_ERR("%s: error: Failed to read from '%s' (error %zd)\n", inst->name, fname, err);
		rtapi_kfree(*pmbccb);
		*pmbccb = NULL;
		filp_close(fp, NULL);
		return err;
	}
	if(err != fsize) {
		MSG_ERR("%s: error: Read %zd bytes instead of %zd bytes from '%s', aborting\n", inst->name, err, fsize, fname);
		rtapi_kfree(*pmbccb);
		*pmbccb = NULL;
		filp_close(fp, NULL);
		return -EIO;	// Assume IO error if the sizes do not match
	}
	filp_close(fp, NULL);
	return err;	// Read it all, return the size
}
#endif


static int check_pintype(unsigned type)
{
	switch(type) {
	case HAL_BIT:
	case HAL_U32:
	case HAL_S32:
	case HAL_FLOAT:
	case HAL_FLOAT_ABCD:
	case HAL_FLOAT_BADC:
	case HAL_FLOAT_CDAB:
	case HAL_FLOAT_DCBA:
	case HAL_U32_ABCD:
	case HAL_U32_BADC:
	case HAL_U32_CDAB:
	case HAL_U32_DCBA:
	case HAL_S32_ABCD:
	case HAL_S32_BADC:
	case HAL_S32_CDAB:
	case HAL_S32_DCBA:
		return 0;
	}
	return -1;
}

//
// Load a binary Modbus command control structure and verify its content.
// Returns 0 on success and -errno on failure. A message has been printed in
// case of error.
//
static int load_mbccb(hm2_modbus_inst_t *inst, const char *fname)
{
	int rv = -EINVAL;

	hm2_modbus_mbccb_header_t *mbccb;
	ssize_t mbccblen = read_mbccb(inst, fname, &mbccb);
	if(mbccblen < 0)
		return (int)mbccblen;

	if(mbccblen < (ssize_t)sizeof(*mbccb)) {
		MSG_ERR("%s: error: Mbccb file too small (only read %zd bytes)\n", inst->name, mbccblen);
		goto errout;
	}

	// Done reading, now test format
	static const rtapi_u8 signature[8] = {'M','e','s','a','M','B','0','1'};
	if(memcmp(signature, mbccb->sig, sizeof(signature))) {
		MSG_ERR("%s: error: Invalid signature in mbccb file: '", inst->name);
		for(unsigned i = 0; i < sizeof(mbccb->sig); i++)
			MSG_ERR("%c", isprint(mbccb->sig[i]) ? mbccb->sig[i] : '?');
		MSG_ERR("', expected 'MesaMB01'\n");
		goto errout;
	}

	// Make header's byte sex native from big-endian
	// Skip the signature words and byte values
	mbccb->baudrate = be32_to_cpu(mbccb->baudrate);
	mbccb->interval = be32_to_cpu(mbccb->interval);
	//for(unsigned i = 0; i < sizeof(mbccb->unused)/sizeof(mbccb->unused[0]); i++)
	//	mbccb->unused[i] = be32_to_cpu(mbccb->unused[i]);
	mbccb->initlen  = be32_to_cpu(mbccb->initlen);
	mbccb->cmdslen  = be32_to_cpu(mbccb->cmdslen);
	mbccb->datalen  = be32_to_cpu(mbccb->datalen);

	if(mbccb->baudrate < 1200) {
		MSG_WARN("%s: warning: Mbccb baudrate %u slower than 1200 baud. May fail to setup\n", inst->name, mbccb->baudrate);
	} else if(mbccb->baudrate > 115200) {
		MSG_WARN("%s: warning: Mbccb baudrate %u faster than 115200 baud. May fail to setup\n", inst->name, mbccb->baudrate);
	}

	switch(MBCCB_FORMAT_PARITY_VAL(mbccb->format)) {
	case MBCCB_FORMAT_PARITY_NONE:
	case MBCCB_FORMAT_PARITY_ODD:
	case MBCCB_FORMAT_PARITY_EVEN:
		break;
	default:
		MSG_ERR("%s: error: Mbccb parity value 0x%02x unrecognized\n", inst->name, MBCCB_FORMAT_PARITY_VAL(mbccb->format));
		goto errout;
	}

	// Must have whole init records
	if(0 != mbccb->initlen % sizeof(hm2_modbus_mbccb_cmds_t)) {
		MSG_ERR("%s: error: Mbccb init section has invalid size %zu\n", inst->name, (size_t)mbccb->initlen);
		goto errout;
	}
	// Must have whole cmds records
	if(0 != mbccb->cmdslen % sizeof(hm2_modbus_mbccb_cmds_t)) {
		MSG_ERR("%s: error: Mbccb commands section has invalid size %zu\n", inst->name, (size_t)mbccb->cmdslen);
		goto errout;
	}
	// All lengths plus header must be file size
	ssize_t len = sizeof(*mbccb) + mbccb->initlen + mbccb->cmdslen + mbccb->datalen;
	if(mbccblen != len) {
		MSG_ERR("%s: error: Mbccb size mismatch. Read %zd and calculated %zd\n", inst->name, mbccblen, len);
		goto errout;
	}

	// Calculate pointers and sizes
	size_t ofs = sizeof(hm2_modbus_mbccb_header_t);
	hm2_modbus_mbccb_cmds_t *initptr = (hm2_modbus_mbccb_cmds_t *)((rtapi_u8 *)mbccb + ofs);
	ofs += mbccb->initlen;
	hm2_modbus_mbccb_cmds_t *cmdsptr = (hm2_modbus_mbccb_cmds_t *)((rtapi_u8 *)mbccb + ofs);
	ofs += mbccb->cmdslen;
	const rtapi_u8 *dataptr = ((rtapi_u8 *)mbccb + ofs);
	unsigned ninit = mbccb->initlen / sizeof(hm2_modbus_mbccb_cmds_t);
	unsigned ncmds = mbccb->cmdslen / sizeof(hm2_modbus_mbccb_cmds_t);

	// There must be something to do
	// You can still have 'nothing' if you only have one delay command.
	if(!ncmds) {
		MSG_ERR("%s: error: Mbccb has no commands\n", inst->name);
		goto errout;
	}

	// Check the data segment fragments to add up to the reported length
	rtapi_u32 dl = 0;
	for(const rtapi_u8 *dptr = dataptr; dptr < dataptr + mbccb->datalen; dptr += *dptr + 1) {
		dl += *dptr;
	}
	if(dl != mbccb->datalen) {
		MSG_ERR("%s: error: Mbccb data segment size mismatch. Read %u and calculated %u\n", inst->name, mbccb->datalen, dl);
		goto errout;
	}

	const rtapi_u8 *dataptrend = dataptr + mbccb->datalen;

	// Check all init data packets
	for(unsigned i = 0; i < ninit; i++) {
		initptr[i].addr    = be16_to_cpu(initptr[i].addr);
		initptr[i].pincnt  = be16_to_cpu(initptr[i].pincnt);
		initptr[i].type    = be16_to_cpu(initptr[i].type);
		initptr[i].timeout = be32_to_cpu(initptr[i].timeout);
		initptr[i].dataptr = be32_to_cpu(initptr[i].dataptr);

		if(initptr[i].pincnt || initptr[i].type) {
			MSG_ERR("%s: error: Mbccb init %u zero fields are not zero\n", inst->name, i);
			goto errout;
		}

		const rtapi_u8 *dp = dataptr + initptr[i].dataptr;
		if(initptr[i].dataptr >= mbccb->datalen) {
			MSG_ERR("%s: error: Mbccb init %u data size mismatch. Read %u is beyond segment size %u\n", inst->name, i, initptr[i].dataptr, mbccb->datalen);
			goto errout;
		}

		// Check init's data length
		// Length must be larger or equal smallest packet and it must be within
		// the file boundary.
		if(dp + *dp >= dataptrend) {
			MSG_ERR("%s: error: Mbccb init %u data outside data segment\n", inst->name, i);
			goto errout;
		}
		if(*dp < 2) {
			MSG_ERR("%s: error: Mbccb init %u data packet size less than 2\n", inst->name, i);
			goto errout;
		}

		// Can never be more than MAXDELAY
		if(initptr[i].timeout > MAXDELAY) {
			MSG_ERR("%s: error: Mbccb init %u timeout larger than %u microseconds\n", inst->name, i, (unsigned)MAXDELAY);
			goto errout;
		}

		// Support only a strict set of command functions
		switch(initptr[i].func) {
		case 0: // The delay command
			if(0 != initptr[i].mbid) {
				MSG_ERR("%s: error: Mbccb init %u mbid not zero for delay command\n", inst->name, i);
				goto errout;
			}
			break;
		case MBCMD_W_COIL:
		case MBCMD_W_REGISTER:
		case MBCMD_W_COILS:
		case MBCMD_W_REGISTERS:
			break;
		default:
			MSG_ERR("%s: error: Mbccb init %u has unsupported Modbus command function %u\n", inst->name, i, initptr[i].func);
			goto errout;
		}
	}

	unsigned npins = 0;

	// Check that all pins names are valid and in bounds
	for(unsigned c = 0; c < ncmds; c++) {
		cmdsptr[c].addr    = be16_to_cpu(cmdsptr[c].addr);
		cmdsptr[c].pincnt  = be16_to_cpu(cmdsptr[c].pincnt);
		cmdsptr[c].type    = be16_to_cpu(cmdsptr[c].type);
		cmdsptr[c].timeout = be32_to_cpu(cmdsptr[c].timeout);
		cmdsptr[c].dataptr = be32_to_cpu(cmdsptr[c].dataptr);

		// Data pointer can never be beyond data segment
		if(cmdsptr[c].dataptr >= mbccb->datalen) {
			MSG_ERR("%s: error: Mbccb cmds' datasize mismatch. Read %u is beyond segment size %u\n", inst->name, cmdsptr[c].dataptr, mbccb->datalen);
			goto errout;
		}

		// Can never be more than MAXDELAY
		if(cmdsptr[c].timeout > MAXDELAY) {
			MSG_ERR("%s: error: Mbccb cmds %u timeout larger than %u microseconds\n", inst->name, c, (unsigned)MAXDELAY);
			goto errout;
		}

		// Support only a strict set of command functions
		switch(cmdsptr[c].func) {
		case 0: // The delay command
			if(0 != cmdsptr[c].mbid || 0 != cmdsptr[c].pincnt || 0 != cmdsptr[c].addr) {
				MSG_ERR("%s: error: Mbccb cmds %u mbid, pin count or address not zero for delay command\n", inst->name, c);
				goto errout;
			}
			continue;	// No pins, need to move to the next command
		case MBCMD_R_COILS:
		case MBCMD_R_INPUTS:
		case MBCMD_R_REGISTERS:
		case MBCMD_R_INPUTREGS:
		case MBCMD_W_COIL:
		case MBCMD_W_REGISTER:
		case MBCMD_W_COILS:
		case MBCMD_W_REGISTERS:
			break;
		default:
			MSG_ERR("%s: error: Mbccb cmds %u has unsupported Modbus command function %u\n", inst->name, c, cmdsptr[c].func);
			goto errout;
		}

		// Check pin type
		if(check_pintype(cmdsptr[c].type) < 0) {
			MSG_ERR("%s: error: Mbccb cmds %u invalid pin/data type %u\n", inst->name, c, cmdsptr[c].type);
			goto errout;
		}

		// Check each pin
		// Strings are pascal strings with 'leading byte == length' and must
		// additionally be NUL terminated. The NUL is counted in the length.
		const rtapi_u8 *dp = dataptr + cmdsptr[c].dataptr;
		for(unsigned p = 0; p < cmdsptr[c].pincnt; p++) {
			if(dp + *dp >= dataptrend) {
				MSG_ERR("%s: error: Mbccb pin %u:%u outside data segment\n", inst->name, c, p);
				goto errout;
			}
			if(*dp < 2) {	// Should contain at least one char and the terminating zero
				MSG_ERR("%s: error: Mbccb pin %u:%u is empty\n", inst->name, c, p);
				goto errout;
			}
			if(*dp > MAXPINNAME) {
				MSG_ERR("%s: error: Mbccb pin %u:%u is too long (%u > %u)\n", inst->name, c, p, *dp, (unsigned)MAXPINNAME);
				goto errout;
			}
			if(dp[*dp]) {	// End of string
				MSG_ERR("%s: error: Mbccb pin %u:%u not NUL terminated\n", inst->name, c, p);
				goto errout;
			}
			// Check all chars, exclude terminating NUL char: [a-z0-9.-]
			for(unsigned ch = *dp - 1; ch > 0; ch--) {
				if(!((dp[ch] >= 'a' && dp[ch] <= 'z') || (dp[ch] >= '0' && dp[ch] <= '9') || dp[ch] == '.' || dp[ch] == '-')) {
					MSG_ERR("%s: error: Mbccb Invalid character(s) in pin %u:%u\n", inst->name, c, p);
					goto errout;
				}
			}

			// Count the pins too
			npins++;
			if(HAL_FLOAT == cmdsptr[c].type || HAL_S32 == cmdsptr[c].type)
				npins += 2;	// the 'scale/offset' or 'scale/scaled' pins

			dp += *dp + 1;	// Next data fragment
		}
	}

	// Copy validated data refs into the instance
	inst->mbccb = mbccb;
	inst->initptr = initptr;
	inst->cmdsptr = cmdsptr;
	inst->dataptr = dataptr;
	inst->ninit = ninit;
	inst->ncmds = ncmds;
	inst->npins = npins;

	return 0;	// Success

errout:
	rtapi_kfree(mbccb);
	return rv;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*                        Main entry and exit point                        */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void cleanup()
{
	if(comp_id >= 0)
		hal_exit(comp_id);

	if(mb.insts) {
		for(int i = 0; i < mb.ninsts; i++) {
			if(mb.insts[i].cmds)
				rtapi_kfree(mb.insts[i].cmds);
			if(mb.insts[i].mbccb)
				rtapi_kfree(mb.insts[i].mbccb);
		}
		rtapi_kfree(mb.insts);
	}
}

int rtapi_app_main(void)
{
	int retval;

	// Only touch the message level if requested
	if(debug >= 0)
		rtapi_set_msg_level(debug);

	if(!ports[0]) {
		MSG_ERR(COMP_NAME": The component requires at least one valid pktuart port, eg ports=\"hm2_5i25.0.pktuart.7\"\n");
		return -EINVAL;
	}

	comp_id = hal_init(COMP_NAME);
	if(comp_id < 0) {
		MSG_ERR(COMP_NAME": hal_init() failed\n");
		return comp_id;
	}

	// Count the instances.
	for(mb.ninsts = 0; mb.ninsts < MAX_PORTS && ports[mb.ninsts]; mb.ninsts++) {}
	// Allocate memory for the instances
	if(!(mb.insts = (hm2_modbus_inst_t *)rtapi_kzalloc(mb.ninsts * sizeof(*mb.insts), RTAPI_GFP_KERNEL))) {
		MSG_ERR(COMP_NAME": Allocate instance memory failed\n");
		hal_exit(comp_id);
		return -ENOMEM;
	}

	// Parse the config string and assign to instances
	for(int i = 0; i < mb.ninsts; i++) {
		hm2_modbus_inst_t *inst = &mb.insts[i];

		rtapi_snprintf(inst->name, sizeof(inst->name), COMP_NAME".%d", i);
		rtapi_strlcpy(inst->uart, ports[i], sizeof(inst->uart)-1);

		if(!mbccbs[i]) {
			MSG_ERR("%s: error: Missing mbccb file path missing for instance %d in 'mbccbs' argument\n", inst->name, i);
			retval = -EINVAL;
			goto errout;
		}
		if('/' != mbccbs[i][0]) {
			MSG_ERR("%s: error: The 'mbccb' file path '%s' for instance %d in 'mbccbs' argument is not absolute\n", inst->name, mbccbs[i], i);
			retval = -EINVAL;
			goto errout;
		}

		if((retval = load_mbccb(inst, mbccbs[i])) < 0) {
			// Messages printed in load function
			goto errout;
		}

		// All pointers and counts have been setup in load_mbccb()

		// Allocate HAL memory
		if(!(inst->hal =  (hm2_modbus_hal_t *)hal_malloc(sizeof(*inst->hal)))) {
			MSG_ERR("%s: error: Failed to allocate HAL memory\n", inst->name);
			retval = -ENOMEM;
			goto errout;
		}
		if(!(inst->hal->pins = (hal_data_u **)hal_malloc(inst->npins * sizeof(*inst->hal->pins)))) {
			MSG_ERR("%s: error: Failed to allocate HAL memory\n", inst->name);
			retval = -ENOMEM;
			goto errout;
		}

		if(inst->ninit > 0) {
			// Allocate inits memory
			if(!(inst->_init = rtapi_kzalloc(inst->ncmds * sizeof(*inst->_init), RTAPI_GFP_KERNEL))) {
				MSG_ERR("%s: error: Failed to allocate init commands memory\n", inst->name);
				retval = -ENOMEM;
				goto errout;
			}
		}

		// Allocate commands memory
		if(!(inst->_cmds = rtapi_kzalloc(inst->ncmds * sizeof(*inst->_cmds), RTAPI_GFP_KERNEL))) {
			MSG_ERR("%s: error: Failed to allocate commands memory\n", inst->name);
			retval = -ENOMEM;
			goto errout;
		}

		// Copy the init command control structure data
		for(unsigned i = 0; i < inst->ninit; i++) {
			inst->_init[i].cmd = inst->initptr[i];
		}
		// Copy the loop command control structure data
		for(unsigned i = 0; i < inst->ncmds; i++) {
			inst->_cmds[i].cmd = inst->cmdsptr[i];
		}

		// Export the HAL process function
		if((retval = hal_export_functf(process, inst, 1, 0, comp_id, COMP_NAME".%d", i)) < 0) {
			MSG_ERR("%s: error: Function export failed\n", inst->name);
			goto errout;
		}

#define CHECK(x) do { \
					retval = (x); \
					if(retval < 0) { \
						MSG_ERR("%s: error: Failed to create pin or parameter\n", inst->name); \
						goto errout; \
					} \
				} while(0)
		CHECK(hal_param_u32_newf(HAL_RO, &(inst->hal->baudrate), comp_id, "%s.baudrate", inst->name));
		CHECK(hal_param_u32_newf(HAL_RO, &(inst->hal->parity),   comp_id, "%s.parity", inst->name));
		CHECK(hal_param_u32_newf(HAL_RO, &(inst->hal->stopbits), comp_id, "%s.stopbits", inst->name));
		CHECK(hal_param_u32_newf(HAL_RW, &(inst->hal->txdelay),  comp_id, "%s.txdelay", inst->name));
		CHECK(hal_param_u32_newf(HAL_RW, &(inst->hal->rxdelay),  comp_id, "%s.rxdelay", inst->name));
		CHECK(hal_param_u32_newf(HAL_RW, &(inst->hal->drvdelay), comp_id, "%s.drive-delay", inst->name));
		CHECK(hal_param_u32_newf(HAL_RW, &(inst->hal->interval), comp_id, "%s.interval", inst->name));

		CHECK(hal_pin_bit_newf(HAL_OUT, &(inst->hal->fault),     comp_id, "%s.fault", inst->name));
		CHECK(hal_pin_u32_newf(HAL_OUT, &(inst->hal->faultcmd),  comp_id, "%s.fault-command", inst->name));
		CHECK(hal_pin_u32_newf(HAL_OUT, &(inst->hal->lasterror), comp_id, "%s.last-error", inst->name));

		inst->hal->interval = inst->mbccb->interval;
		inst->hal->baudrate = inst->baudrate = inst->mbccb->baudrate;
		inst->hal->parity   = inst->parity   = MBCCB_FORMAT_PARITY_VAL(inst->mbccb->format);
		inst->hal->stopbits = inst->stopbits = MBCCB_FORMAT_STOPBITS_VAL(inst->mbccb->format);
		// The following three will be copied to the instance in do_setup()
		inst->hal->rxdelay  = inst->mbccb->rxdelay;
		inst->hal->txdelay  = inst->mbccb->txdelay;
		inst->hal->drvdelay = inst->mbccb->drvdelay;

		MSG_INFO("%s: PktUART serial configured to 8%c%c@%d\n",
					inst->name,
					inst->parity ? (inst->parity == 1 ? 'O' : 'E') : 'N',
					inst->stopbits ? '2' : '1',
					inst->baudrate);

		*(inst->hal->fault)     = 0;
		*(inst->hal->faultcmd)  = 0;
		*(inst->hal->lasterror) = 0;


		unsigned p = 0;
#define CPTR(x)	((const char *)((x) + 1))
		for(unsigned c = 0; c < inst->ncmds; c++) {
			hm2_modbus_cmd_t *cmd = &inst->cmds[c];
			int dir = HAL_IN;
			const rtapi_u8 *dptr = inst->dataptr + cmd->cmd.dataptr;
			cmd->pinref = p;
			for(int j = 0; j < cmd->cmd.pincnt; j++) {
				switch(cmd->cmd.func & 0x7f) {
				case MBCMD_R_COILS:
				case MBCMD_R_INPUTS:
					dir = HAL_OUT;
					/* Fallthrough */
				case MBCMD_W_COIL:
				case MBCMD_W_COILS:
					if(HAL_BIT != cmd->cmd.type) {
						MSG_ERR("%s: error: Invalid pin type %d in command function %d\n", inst->name, cmd->cmd.type, cmd->cmd.func & 0x7f);
						retval = -EINVAL;
						goto errout;
					}
					CHECK(hal_pin_bit_newf(dir, (hal_bit_t**)&(inst->hal->pins[p++]),
							comp_id, "%s.%s", inst->name, CPTR(dptr)));
					break;

				case MBCMD_R_INPUTREGS:
					if(HAL_U32 != cmd->cmd.type) {
						MSG_ERR("%s: error: Invalid pin type %d in command function %d\n", inst->name, cmd->cmd.type, cmd->cmd.func & 0x7f);
						retval = -EINVAL;
						goto errout;
					}
					CHECK(hal_pin_u32_newf(HAL_OUT, (hal_u32_t**)&(inst->hal->pins[p++]),
							comp_id, "%s.%s", inst->name, CPTR(dptr)));
					break;

				case MBCMD_R_REGISTERS:
					dir = HAL_OUT;
					/* Fallthrough */
				case MBCMD_W_REGISTER:
				case MBCMD_W_REGISTERS:
					switch(cmd->cmd.type) {
					default:
					case HAL_BIT:
						MSG_ERR("%s: error: Invalid pin type %d in command function %d\n", inst->name, cmd->cmd.type, cmd->cmd.func & 0x7f);
						retval = -EINVAL;
						goto errout;

					case HAL_U32:
						CHECK(hal_pin_u32_newf(dir, (hal_u32_t**)&(inst->hal->pins[p++]),
								comp_id, "%s.%s", inst->name, CPTR(dptr)));
						break;

					case HAL_S32:
						CHECK(hal_pin_s32_newf(dir, (hal_s32_t**)&(inst->hal->pins[p+0]),
								comp_id, "%s.%s", inst->name, CPTR(dptr)));
						// FIXME: -scale and -scaled will be unused when dir == HAL_IN
						CHECK(hal_pin_float_newf(HAL_IN, (hal_float_t**)&(inst->hal->pins[p+1]),
								comp_id, "%s.%s-scale", inst->name, CPTR(dptr)));
						inst->hal->pins[p+1]->f = 1.0;
						CHECK(hal_pin_float_newf(dir, (hal_float_t**)&(inst->hal->pins[p+2]),
								comp_id, "%s.%s-scaled", inst->name, CPTR(dptr)));
						p += 3;
						break;

					case HAL_FLOAT:
						CHECK(hal_pin_float_newf(dir, (hal_float_t**)&(inst->hal->pins[p+0]),
								comp_id, "%s.%s", inst->name, CPTR(dptr)));
						CHECK(hal_pin_float_newf(HAL_IN, (hal_float_t**)&(inst->hal->pins[p+1]),
								comp_id, "%s.%s-scale", inst->name, CPTR(dptr)));
						inst->hal->pins[p+1]->f = 1.0;
						CHECK(hal_pin_float_newf(HAL_IN, (hal_float_t**)&(inst->hal->pins[p+2]),
								comp_id, "%s.%s-offset", inst->name, CPTR(dptr)));
						p += 3;
						break;

					case HAL_FLOAT_ABCD:
					case HAL_FLOAT_BADC:
					case HAL_FLOAT_CDAB:
					case HAL_FLOAT_DCBA:
						CHECK(hal_pin_float_newf(dir, (hal_float_t**)&(inst->hal->pins[p++]),
								comp_id, "%s.%s", inst->name, CPTR(dptr)));
						break;

					case HAL_S32_ABCD:
					case HAL_S32_BADC:
					case HAL_S32_CDAB:
					case HAL_S32_DCBA:
						CHECK(hal_pin_s32_newf(dir, (hal_s32_t**)&(inst->hal->pins[p++]),
								comp_id, "%s.%s", inst->name, CPTR(dptr)));
						break;

					case HAL_U32_ABCD:
					case HAL_U32_BADC:
					case HAL_U32_CDAB:
					case HAL_U32_DCBA:
						CHECK(hal_pin_u32_newf(dir, (hal_u32_t**)&(inst->hal->pins[p++]),
								comp_id, "%s.%s", inst->name, CPTR(dptr)));
						break;
					}
					break;
				}
				dptr += *dptr + 1;	// Add length byte to get to next data record/pin name
			}
		}
#undef CPTR
#undef CHECK

		inst->state = STATE_START;

		// Configure PktUART
		if((retval = do_setup(inst) < 0)) {
			MSG_ERR("%s: error: Failure to setup PktUART\n", inst->name);
			goto errout;
		}

		// Ensure clear RX and TX FIFOs
		hm2_pktuart_reset(inst->uart);

	}
	hal_ready(comp_id);
	return 0;

errout:
	cleanup();
	return retval;
}

void rtapi_app_exit(void)
{
	cleanup();
}
// vim: syn=c ts=4
