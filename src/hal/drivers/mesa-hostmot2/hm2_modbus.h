#ifndef __HOSTMOT2_HM2_MODBUS_H
#define __HOSTMOT2_HM2_MODBUS_H

//
// Binary Modbus command config file format:
// - hm2_modbus_mbccb_header_t [1]
// - hm2_modbus_mbccb_cmd_t    [0..x]	Init data
// - hm2_modbus_mbccb_cmd_t    [1..y]	Command sequence
// - data table
//
// Filesize = sizeof(header) + header.initlen + header.cmdlen + header.datalen.
//
#define MBCCB_FORMAT_PARITY_NONE      0
#define MBCCB_FORMAT_PARITY_ODD       1
#define MBCCB_FORMAT_PARITY_EVEN      2

#define MBCCB_FORMAT_PARITY_BIT       0 // bits 0..1  00=8Nx 01=8Ox 10=8Ex 11=invalid
#define MBCCB_FORMAT_STOPBITS2_BIT    2 // bit  2     0=8x1 1=8x2
#define MBCCB_FORMAT_DUPLEX_BIT       3 // bit  3     Set for full-duplex (rx-mask off)
#define MBCCB_FORMAT_IFSCALE_BIT      7 // bit  7     Inter-frame delay scale 0=1x, 1=4x
#define MBCCB_FORMAT_PARITY_MASK     (3u << MBCCB_FORMAT_PARITY_BIT)
#define MBCCB_FORMAT_PARITY(x)       (((x) & 3u) << MBCCB_FORMAT_PARITY_BIT)
#define MBCCB_FORMAT_PARITY_VAL(x)   (((x) >> MBCCB_FORMAT_PARITY_BIT) & 3u)
#define MBCCB_FORMAT_STOPBITS2        (1u << MBCCB_FORMAT_STOPBITS2_BIT)
#define MBCCB_FORMAT_STOPBITS2_VAL(x) (((x) >> MBCCB_FORMAT_STOPBITS2_BIT) & 1u)
#define MBCCB_FORMAT_IFSCALE         (1u << MBCCB_FORMAT_IFSCALE_BIT)
#define MBCCB_FORMAT_IFSCALE_VAL(x)  (((x) >> MBCCB_FORMAT_IFSCALE_BIT) & 1u)
#define MBCCB_FORMAT_DUPLEX          (1u << MBCCB_FORMAT_IFSCALE_BIT)
#define MBCCB_FORMAT_DUPLEX_VAL(x)   (((x) >> MBCCB_FORMAT_IFSCALE_BIT) & 1u)

// XXX: keep in sync with mesamodbus.py
// Max one minute delay between init commands (in microseconds)
#define MAXDELAY 60000000
// Max chars for a name
#define MAXPINNAME 24
//
// 16*4 byte structure
// All values in Big-Endian
// Must be 32-bit aligned and sizeof() % 4 == 0
typedef struct {
	rtapi_u8	sig[8];		// Signature and version {'M','e','s','a','M','B','0','1'}
	rtapi_u32	baudrate;
	rtapi_u8	format;		// Parity and stopbits
	rtapi_u8	txdelay;	// Tx inter-frame timeout (t3.5)
	rtapi_u8	rxdelay;	// Rx inter-frame timeout (t3.5)
	rtapi_u8	drvdelay;	// Delay from output enable to tx start
	rtapi_u32	interval;	// Update rate in _micro_second interval time (0 = as-fast-as-possible)
	rtapi_u32	unused[8];
	rtapi_u32	initlen;	// Length of init section
	rtapi_u32	cmdslen;	// Length of command section
	rtapi_u32	datalen;	// Length of data table
} hm2_modbus_mbccb_header_t;

// 32 byte structure
// All values in Big-Endian
typedef struct {
	rtapi_u8	mbid;	// Modbus device ID
	rtapi_u8	func;	// Function code, 0 for delay command
	rtapi_u16	addr;	// Address, 0 for init
	rtapi_u16	pincnt;	// Number of pins, 0 for init
	rtapi_u8	htype;	// HAL data type, 0 for init
	rtapi_u8	mtype;	// Modbus data type, 0 for init
	rtapi_u16	flags;	// Mostly quirks to handle, see MBCCB_CMDF_* defines
	rtapi_u16	unused1;
	rtapi_u32	unused2[3];
	rtapi_u32	timeout; // Response timeout or delay in microseconds
	rtapi_u32	dataptr; // Pin names, packet data for init
} hm2_modbus_mbccb_cmds_t;

#define MBCCB_CMDF_TIMESOUT	0x0001	// Don't treat timeout as an error
#define MBCCB_CMDF_BCANSWER	0x0002	// Broadcasts will get an answer, ignore it
#define MBCCB_CMDF_NOANSWER	0x0004	// Don't expect an answer
#define MBCCB_CMDF_SCALE	0x0008	// Add scale/offset pins
#define MBCCB_CMDF_CLAMP	0x0010	// Clamp values to fit target
#define MBCCB_CMDF_RESEND	0x0020	// Resend the write even if no pins are changed
#define MBCCB_CMDF_INITMASK	0x0007	// sum of allowed flags in init
#define MBCCB_CMDF_MASK		0x003f	// sum of all above flags

#endif
// vim: ts=4
