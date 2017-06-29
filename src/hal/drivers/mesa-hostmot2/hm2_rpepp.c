/*
 * This is a component for RaspberryPi to hostmot2 over EPP for linuxcnc.
 * Copyright 2017 B.Stultiens <lcnc@vagrearg.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <hal.h>
#include <rtapi_app.h>
#include <rtapi_slab.h>

#include "hostmot2.h"
#include "spi_common_rpspi.h"	// This has the GPIO definitions

MODULE_LICENSE("GPL");
MODULE_AUTHOR("B. Stultiens");
MODULE_DESCRIPTION("Driver for HostMot2 devices connected via EPP on the RaspberryPi");
MODULE_SUPPORTED_DEVICE("Mesa-AnythingIO-7i90,Mesa-AnythingIO-7i43");

// Forced inline expansion
#define RPEPP_ALWAYS_INLINE	__attribute__((always_inline))

//#define RPEPP_GPIO_GUARD	1	// Define to protect GPIO indices

// Enable rising edge detection on the WAIT signal to end a read or write
// cycle. The kernel will throw an exception the first time the edge is seen
// because interrupts are enabled by default, see /proc/interrupts. On kernel
// 4.4.4-rt9-v7+ it is normally mapped to #79, device "3f200000.gpio:bank0".
// The kernel will simply ignore the interrupt and disable it. However, it is
// not "nice". Alternative is to write a kernel driver and handle it there.
//
// Detection of the rising edge is marginally faster than spinning and reading
// the GPIO level. This needs to be evaluated more closely. Maybe it is
// preferable to have one kernel complaint and faster operation than slightly
// slower performance. Throughput numbers will have to decide.
#define RPEPP_WAIT_EDGEDETECT	1

#define RPEPP_MAX_BOARDS	2

#ifndef _BV
#define _BV(x)		(1 << (x))
#endif

#ifndef NELEM
#define NELEM(x)	(sizeof(x) / sizeof(*(x)))
#endif

#define RPEPP_DBG	RTAPI_MSG_DBG
#define RPEPP_ERR	RTAPI_MSG_ERR
#define RPEPP_WARN	RTAPI_MSG_WARN
#define RPEPP_INFO	RTAPI_MSG_INFO

/*
 * GPIO pin definitions
 * The default layout is compatible with two EPP ports. Port EPP0 shares pins
 * with SPI0 and EPP1 shares pins with SPI1. EPP and SPI can run in parallel on
 * alternate ports i.e. EPP0/SPI1_CE0 and EPP1/SPI0_CE0. The pin-layout is such
 * that no reentrancy problems occur. No pins are shared between the two EPP
 * ports or SPI ports. Therefore, the ports can operate simultaneously without
 * a mutex mechanism.
 *
 * The pin-layout is based on ease to route on a PCB but could be changed
 * without problem. The only pins with fixed positions are the SPI lines. The
 * SPI signals need to be connected at the specific location to support dual
 * function EPP/SPI on the same physical connectors.
 */
#define EPP0_PIN_D0		22	// pin 15
#define EPP0_PIN_D1		23	// pin 16
#define EPP0_PIN_D2		27	// pin 13
#define EPP0_PIN_D3		7	// pin 26
#define EPP0_PIN_D4		11	// pin 23 (also SPI0-SCLK)
#define EPP0_PIN_D5		10	// pin 19 (also SPI0-MOSI)
#define EPP0_PIN_D6		9	// pin 21 (also SPI0-MISO)
#define EPP0_PIN_D7		8	// pin 24 (also SPI0-CE0)
#define EPP0_PIN_DS		24	// pin 18 Data strobe (active low)
#define EPP0_PIN_AS		17	// pin 11 Address strobe (active low)
#define EPP0_PIN_RW		25	// pin 22 Read=1, write=0
#define EPP0_PIN_WAIT		4	// pin  7 Input (active low)

#define EPP1_PIN_D0		13	// pin 33
#define EPP1_PIN_D1		6	// pin 31
#define EPP1_PIN_D2		5	// pin 29
#define EPP1_PIN_D3		3	// pin  5
#define EPP1_PIN_D4		21	// pin 40 (also SPI1-SCLK)
#define EPP1_PIN_D5		20	// pin 37 (also SPI1-MOSI)
#define EPP1_PIN_D6		19	// pin 35 (also SPI1-MISO)
#define EPP1_PIN_D7		18	// pin 12 (also SPI1-CE0)
#define EPP1_PIN_DS		26	// pin 37 Data strobe (active low)
#define EPP1_PIN_AS		12	// pin 32 Address strobe (active low)
#define EPP1_PIN_RW		16	// pin 36 Read=1, write=0
#define EPP1_PIN_WAIT		2	// pin  3 Input (active low)

// The FSEL values are combined into three register access values for
// gpfsel[0..2]. A shortcut is taken for input, which value is zero.
//
// We are lucky with the RPI, which only exports GPIOs up to number 27.
// Therefore, all GPIOs are always contained within the first three gpfsel
// registers.
//
// Note that "input" is not defined here. The mask is used to zero the
// appropriate FSEL bits and zero is the value needed for input.
//
#define RPEPP_GM		7			// FSEL Mask value
#define RPEPP_GO		GPIO_FSEL_X_GPIO_OUTPUT	// FSEL output

typedef struct hm2_rpepp_struct {
	hm2_lowlevel_io_t llio;		// Upstream container
	int		nr;		// Board number
	int		eppid;		// Which port is mapped
	uint32_t	msk_data[8];	// Individual data bit masks
	uint32_t	msk_alldata;	// All data bits in one mask
	uint32_t	msk_rw;		// RW bit mask
	uint32_t	msk_as;		// AS bit mask
	uint32_t	msk_ds;		// DS bit mask
	uint32_t	msk_wait;	// WAIT bit mask
	uint32_t	fsel_msk[3];	// Input/output switch (and-mask)
	uint32_t	fsel_out[3];	// Input/output switch (or-mask)
	uint32_t	bytetable[256];	// Bit translation table (input -> data byte)
	int		pin_data[8];	// GPIO pins belonging to this port
	int		pin_rw;
	int		pin_as;
	int		pin_ds;
	int		pin_wait;
} hm2_rpepp_t;

typedef enum {
	RPI_UNSUPPORTED,
	RPI_1,		// Version 1
	RPI_2		// Version 2 and 3
} platform_t;

static uint32_t *peripheralmem = (uint32_t *)MAP_FAILED;	// mmap'ed peripheral memory
static size_t peripheralsize;					// Size of the mmap'ed block
static bcm2835_gpio_t *gpio;					// GPIO peripheral structure in mmap'ed address space
static platform_t platform;					// The RPI version

static hm2_rpepp_t boards[RPEPP_MAX_BOARDS];	// Connected boards
static int comp_id;				// Upstream assigned component ID

/*
 * Configuration parameters
 */
static char *config[RPEPP_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, RPEPP_MAX_BOARDS, "config string for the AnyIO boards (see hostmot2(9) manpage)")

/*
 * RPI3 NOTE:
 * The core frequency is wildly variable when the ondemand cpufreq governor is
 * active. This may result bad communication performance depending on the
 * system load. You must set the performance governor for stable frequency. To
 * set a stable 1.2GHz core frequency, across all cores, put something like
 * this in /etc/rc.local:
 *   echo -n 1200000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_min_freq
 *   echo -n performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
 */

/*
 * epp_probe: Set which boards (EPP ports) to probe
 */
#define RPEPP_PROBE_0	1
#define RPEPP_PROBE_1	2

static int epp_probe = RPEPP_PROBE_0;
RTAPI_MP_INT(epp_probe, "Bitmask of EPP ports to probe; 1=EPP0, 2=EPP1 (default: 1 = EPP0)")

/*
 * pin_*[]: EPP data port and control signal GPIO pin connection mapping. The
 * defaults are documented above and in in hm2_rpepp(9).
 */
static int pin_data[8*RPEPP_MAX_BOARDS] = {
	EPP0_PIN_D0, EPP0_PIN_D1, EPP0_PIN_D2, EPP0_PIN_D3,
	EPP0_PIN_D4, EPP0_PIN_D5, EPP0_PIN_D6, EPP0_PIN_D7,
	EPP1_PIN_D0, EPP1_PIN_D1, EPP1_PIN_D2, EPP1_PIN_D3,
	EPP1_PIN_D4, EPP1_PIN_D5, EPP1_PIN_D6, EPP1_PIN_D7,
};
static int pin_ds[1*RPEPP_MAX_BOARDS] = {EPP0_PIN_DS, EPP1_PIN_DS};
static int pin_as[1*RPEPP_MAX_BOARDS] = {EPP0_PIN_AS, EPP1_PIN_AS};
static int pin_rw[1*RPEPP_MAX_BOARDS] = {EPP0_PIN_RW, EPP1_PIN_RW};
static int pin_wait[1*RPEPP_MAX_BOARDS] = {EPP0_PIN_WAIT, EPP1_PIN_WAIT};
RTAPI_MP_ARRAY_INT(pin_data, NELEM(pin_data), "Data GPIO pins (comma separated). For defaults see hm2_rpepp(9)")
RTAPI_MP_ARRAY_INT(pin_ds,   NELEM(pin_ds),   "Data strobe GPIO pins (comma separated). For defaults see hm2_rpepp(9)")
RTAPI_MP_ARRAY_INT(pin_as,   NELEM(pin_as),   "Address strobe GPIO pins (comma separated). For defaults see hm2_rpepp(9)")
RTAPI_MP_ARRAY_INT(pin_rw,   NELEM(pin_rw),   "Read/write GPIO pins (comma separated). For defaults see hm2_rpepp(9)")
RTAPI_MP_ARRAY_INT(pin_wait, NELEM(pin_wait), "Wait GPIO pins (comma separated). For defaults see hm2_rpepp(9)")

/*
 * epp_spi_check: Enable/disable checking overlap between EPP and SPI mappings.
 * SPI lines must be connected to a specific location or there is a possibility
 * that hm2_rpspi and hm2_rpepp bite each other.
 */
static int epp_spi_check = 1;
RTAPI_MP_INT(epp_spi_check, "Check whether EPP pin definitions overlap SPI port pins (default 1, enabled)")

/*
 * boardids[]: define the attached board if it cannot be probed
 * Supported values are (case sensitive):
 * - MESA7I90
 * - MESA7I43
 */
static char *boardids[RPEPP_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(boardids, RPEPP_MAX_BOARDS, "Board identifiers (comma separated) if it cannot be probed due to lack of firmware (default: not set)")

/*
 * epp_debug: Set the message level for debugging purpose. This has the
 * (side-)effect that all modules within this process will start spitting out
 * messages at the requested level.
 * The upstream message level is not touched if epp_debug == -1.
 */
static int epp_debug = -1;
RTAPI_MP_INT(epp_debug, "Set message level for debugging purpose [0...5] where 0=none and 5=all (default: -1; upstream defined)")

/*
 * Code comments:
 * All conversion and support routines are forced to expand inline because they
 * are rather timing sensitive in the read/write loops. Inline functions here
 * preserve the functional abstraction without the overhead of calling
 * functions.
 */

/*
 * The port's data-bit positions are mangled with respect to the actual value
 * we want to write or read. These routines do the bit-mangling/unmangling for
 * us.
 *
 * Byte->word conversion is faster using tables, which are created at startup.
 * The word->byte conversion is more tricky and tables are not a real option.
 * The ARM instruction set is at our advantage allowing for conditional 'or',
 * making the actual conversion 16 register-register and 8 memory-register
 * instructions. The memory accesses are scheduled to overlap and it will all
 * be cached after the first loop. This would be much worse using the thumb
 * instruction set.
 */
RPEPP_ALWAYS_INLINE static inline uint8_t epp_port_to_data(const hm2_rpepp_t *port, uint32_t val)
{
	uint8_t data = 0;
	if(val & port->msk_data[0])	data |= 0x01;
	if(val & port->msk_data[1])	data |= 0x02;
	if(val & port->msk_data[2])	data |= 0x04;
	if(val & port->msk_data[3])	data |= 0x08;
	if(val & port->msk_data[4])	data |= 0x10;
	if(val & port->msk_data[5])	data |= 0x20;
	if(val & port->msk_data[6])	data |= 0x40;
	if(val & port->msk_data[7])	data |= 0x80;
	return data;
}

RPEPP_ALWAYS_INLINE static inline uint32_t epp_data_to_port(const hm2_rpepp_t *port, uint8_t val)
{
	return port->bytetable[val];
}

static void rpepp_init_datatable(hm2_rpepp_t *port)
{
	unsigned i;
	// Crude but effective
	for(i = 0; i < 256; i++) {
		uint32_t pdat = 0;
		if(i & 0x01) { pdat |= port->msk_data[0]; }
		if(i & 0x02) { pdat |= port->msk_data[1]; }
		if(i & 0x04) { pdat |= port->msk_data[2]; }
		if(i & 0x08) { pdat |= port->msk_data[3]; }
		if(i & 0x10) { pdat |= port->msk_data[4]; }
		if(i & 0x20) { pdat |= port->msk_data[5]; }
		if(i & 0x40) { pdat |= port->msk_data[6]; }
		if(i & 0x80) { pdat |= port->msk_data[7]; }
		port->bytetable[i] = pdat;
	}
}

/*
 * Synchronized read and write to peripheral memory.
 * Ensures coherency between cores, cache and peripherals
 */
#define rmb()	__sync_synchronize()	// Read sync (finish all reads before continuing)
#define wmb()	__sync_synchronize()	// Write sync (finish all write before continuing)

RPEPP_ALWAYS_INLINE static inline uint32_t reg_rd(const volatile void *addr)
{
	uint32_t val;
	val = *(volatile uint32_t *)addr;
	rmb();
	return val;
}

RPEPP_ALWAYS_INLINE static inline void reg_wr(const volatile void *addr, uint32_t val)
{
	wmb();
	*(volatile uint32_t *)addr = val;
}

/*
 * Note for low-level functions
 * ============================
 * Call these routines with a constant arguments if possible. The compiler
 * cannot eliminate index calculations and the conditional statements if the
 * argumemts are not constant and will therefore have a significant performance
 * impact.
 *
 * The constant rule is already broken in the setup/restore functions, but
 * these are not timing critical and just result in more code. Not a real
 * problem.
 */

/*
 * Set/Clear a GPIO pin
 */
RPEPP_ALWAYS_INLINE static inline void gpio_set(uint32_t pin)
{
#ifdef RPEPP_GPIO_GUARD
	if(pin <= 53) {	/* There are 54 GPIOs */
#endif
		reg_wr(&gpio->gpset[pin / 32], 1 << (pin % 32));
#ifdef RPEPP_GPIO_GUARD
	}
#endif
}

RPEPP_ALWAYS_INLINE static inline void gpio_clr(uint32_t pin)
{
#ifdef RPEPP_GPIO_GUARD
	if(pin <= 53) {	/* There are 54 GPIOs */
#endif
		reg_wr(&gpio->gpclr[pin / 32], 1 << (pin % 32));
#ifdef RPEPP_GPIO_GUARD
	}
#endif
}

/*
 * Function select on GPIO pins
 */
RPEPP_ALWAYS_INLINE static inline void gpio_fsel(uint32_t pin, uint32_t func)
{
#ifdef RPEPP_GPIO_GUARD
	if(pin <= 53) {	/* There are 54 GPIOs */
#endif
		uint32_t bits = pin * 3;	/* Three bits per fsel field and 10 gpio per uint32_t */
		reg_wr(&gpio->gpfsel[bits / 30], (reg_rd(&gpio->gpfsel[bits / 30]) & ~(7 << (bits % 30))) | ((func & 7) << (bits % 30)));
#ifdef RPEPP_GPIO_GUARD
	}
#endif
}

/*
 * Flip input/output on the EPP data port lines
 *
 * The FSEL register values are already combined to allow for three register
 * read-modify-write accesses.
 *
 * The change to input is short-cut by the fact that input sets FSEL=0. This
 * also allows for the change to output to be short-cut by a simple 'or'
 * because the base-value is already masked by FSEL=0.
 *
 * The assumtpion here is that the FSEL registers have been setup properly
 * before these functions are called. That is the case here (see pins_setup()
 * below).
 *
 * We control the R/W pin here too. This allows for a bi-directional buffer to
 * be attached on the data-lines and it switches properly timed on basis of the
 * R/W pin. The actual control of the buffer may be slightly more difficult.
 * The buffer enable, as seen from the host (this machine), could be:
 *	/CE = RW * /WAIT = /(/RW + WAIT)
 * An additional buffer should be used to handle the control-lines.
 *
 * It is also possible to ignore the WAIT signal and to enable the buffer all
 * the time, only controlling the direction with RW. The tricky part is to
 * ensure that the slave side has its buffers in the right direction at the
 * same time.
 */
RPEPP_ALWAYS_INLINE static inline void epp_dataportdir_output(const hm2_rpepp_t *port)
{
	reg_wr(&gpio->gpclr0, port->msk_rw);		// Clear R/W
	reg_wr(&gpio->gpfsel0, reg_rd(&gpio->gpfsel0) | port->fsel_out[0]);
	reg_wr(&gpio->gpfsel1, reg_rd(&gpio->gpfsel1) | port->fsel_out[1]);
	reg_wr(&gpio->gpfsel2, reg_rd(&gpio->gpfsel2) | port->fsel_out[2]);
}

RPEPP_ALWAYS_INLINE static inline void epp_dataportdir_input(const hm2_rpepp_t *port)
{
	reg_wr(&gpio->gpfsel0, reg_rd(&gpio->gpfsel0) & port->fsel_msk[0]);
	reg_wr(&gpio->gpfsel1, reg_rd(&gpio->gpfsel1) & port->fsel_msk[1]);
	reg_wr(&gpio->gpfsel2, reg_rd(&gpio->gpfsel2) & port->fsel_msk[2]);
	reg_wr(&gpio->gpset0, port->msk_rw);		// Set R/W
}

/*
 * Perform EPP transactions
 */
RPEPP_ALWAYS_INLINE static inline void epp_assert_strobe(const hm2_rpepp_t *port, bool ad_strobe, bool active)
{
	// Strobes are active low
	if(active) {
		if(ad_strobe)
			reg_wr(&gpio->gpclr0, port->msk_as);	// Address strobe to low
		else
			reg_wr(&gpio->gpclr0, port->msk_ds);	// Data strobe to low
	} else {
		if(ad_strobe)
			reg_wr(&gpio->gpset0, port->msk_as);	// Address strobe to high
		else
			reg_wr(&gpio->gpset0, port->msk_ds);	// Data strobe to high
	}
}

#define TIMEOUT_SPINS	1000	// Many clock cycles (>80 us; spec says ~10 us)

RPEPP_ALWAYS_INLINE static inline bool epp_write_cycle(const hm2_rpepp_t *port, uint8_t data, bool ad_strobe)
{
	// It is assumed that the data port is set to output at this time and
	// the R/W pin is set low
	uint32_t portdata;
	unsigned timeout;

	portdata = epp_data_to_port(port, data);

	reg_wr(&gpio->gpclr0, portdata ^ port->msk_alldata);	// Clear data bits to clear
	reg_wr(&gpio->gpset0, portdata);			// Set data bits to set

	// Wait for the WAIT line to be(come) low
	timeout = TIMEOUT_SPINS;
	while((reg_rd(&gpio->gplev0) & port->msk_wait) && --timeout)
		;
	if(!timeout) {
		// The WAIT line is active --> no board attached?
		return false;
	}

#ifdef RPEPP_WAIT_EDGEDETECT
	reg_wr(&gpio->gpeds0, port->msk_wait);		// Wait is now low, clear rising edge event
#endif
	epp_assert_strobe(port, ad_strobe, true);	// Assert strobe

	// Wait for the WAIT line to go high (detect rising edge)
	timeout = TIMEOUT_SPINS;
#ifdef RPEPP_WAIT_EDGEDETECT
	while(!(reg_rd(&gpio->gpeds0) & port->msk_wait) && --timeout)
		;
#else
	while(!(reg_rd(&gpio->gplev0) & port->msk_wait) && --timeout)
		;
#endif
	if(!timeout) {
		// Timeout on WAIT line --> maybe no board attached?
		epp_assert_strobe(port, ad_strobe, false);
		return false;
	}
	//rtapi_print_msg(RPEPP_DBG, "write: spins %u\n", TIMEOUT_SPINS - timeout);

	epp_assert_strobe(port, ad_strobe, false);	// Done transaction (we don't wait for the WAIT signal)
	return true;
}

RPEPP_ALWAYS_INLINE static inline bool epp_read_cycle(const hm2_rpepp_t *port, uint8_t *data, bool ad_strobe)
{
	// It is assumed that the data port is set to input at this time and
	// the R/W pin set high
	uint32_t portdata;
	unsigned timeout;

	// Wait for the WAIT line to be(come) low
	timeout = TIMEOUT_SPINS;
	while((reg_rd(&gpio->gplev0) & port->msk_wait) && --timeout)
		;
	if(!timeout) {
		// The WAIT line is active --> no board attached?
		return false;
	}

#ifdef RPEPP_WAIT_EDGEDETECT
	reg_wr(&gpio->gpeds0, port->msk_wait);		// Wait is now low, clear rising edge event
#endif
	epp_assert_strobe(port, ad_strobe, true);	// Assert strobe

	// Wait for the WAIT line to go high (detect rising edge)
	timeout = TIMEOUT_SPINS;
#ifdef RPEPP_WAIT_EDGEDETECT
	while(!(reg_rd(&gpio->gpeds0) & port->msk_wait) && --timeout)
		;
#else
	while(!(reg_rd(&gpio->gplev0) & port->msk_wait) && --timeout)
		;
#endif
	if(!timeout) {
		// Timeout on WAIT line --> maybe no board attached?
		epp_assert_strobe(port, ad_strobe, false);
		return false;
	}
	//rtapi_print_msg(RPEPP_DBG, "read: spins %u\n", TIMEOUT_SPINS - timeout);

	// The spec says that the data is output from the slave on the rising
	// edge of WAIT and is subsequently read on the rising edge of DSTROBE.
	// We cannot do this and have to read it just before the DSTROBE is
	// deasserted.
	// The only problem we may run into is that the WAIT signal was high
	// too short before we read the data. There are 5..8 instructions
	// before we read the data from the detection of the rising edge of
	// WAIT. We might want to insert a couple of NOPs here, just to let the
	// data-lines settle.
	// An alternative way is to read the data twice. The reg_rd() function
	// stalls the core and is therefore rather slow. The wait for WAIT loop
	// is about 65ns per spin (@1.2GHz). One dummy synchronous read should
	// give us some 20+ns settling time.
	//
	// The Mesa EPP implementation guarantees that data is valid /before/
	// the WAIT line is set high. Therefore, we do not need to wait any
	// longer here as could be required on normal EPP transactions.
	//reg_rd(&gpio->gplev0);	// Dummy read to allow for data settling

	portdata = reg_rd(&gpio->gplev0);		// Get the data
	epp_assert_strobe(port, ad_strobe, false);	// Done transaction (we don't wait for the WAIT signal)

	*data = epp_port_to_data(port, portdata);	// Store the data

	return true;
}

RPEPP_ALWAYS_INLINE static inline bool epp_addr_write16(const hm2_rpepp_t *port, uint16_t val)
{
	// Write two bytes to address port in lo-hi-byte order
	// The data port must have been set to output
	if(!epp_write_cycle(port, val & 0xff, true)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.addr_write16: Write timeout on address write byte 0\n", port->eppid);
		return false;
	}
	if(!epp_write_cycle(port, (val >> 8) & 0xff, true)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.addr_write16: Write timeout on address write byte 1\n", port->eppid);
		return false;
	}
	return true;
}

RPEPP_ALWAYS_INLINE static inline bool epp_addr_write8(const hm2_rpepp_t *port, uint8_t val)
{
	// Write a single byte to address port
	// The data port must have been set to output
	if(!epp_write_cycle(port, val & 0xff, true)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.addr_write8: Write timeout\n", port->eppid);
		return false;
	}
	return true;
}

RPEPP_ALWAYS_INLINE static inline bool epp_data_write8(const hm2_rpepp_t *port, uint8_t val)
{
	// Write a single byte to data port
	// The data port must have been set to output
	if(!epp_write_cycle(port, val, false)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.data_write8: Write timeout\n", port->eppid);
		return false;
	}
	return true;
}

/*********************************************************************/
/*
 * HM2 interface: Write buffer to an EPP port
 */
static int rpepp_epp_write(hm2_lowlevel_io_t *llio, uint32_t addr, void *buffer, int size)
{
	const hm2_rpepp_t *port = (const hm2_rpepp_t *)llio;
	int wsize;
	uint32_t *wbuf = (uint32_t *)buffer;

	if(size <= 0)
		return 0;
	if(size & 3) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.write: Unaligned write, size=%d is not multiple of 4\n", port->eppid, size);
		return 0;
	}

	epp_dataportdir_output(port);

	// Write the address
	if(!epp_addr_write16(port, addr | HM2_ADDR_AUTOINCR)) {
		if(llio->io_error)
			*llio->io_error = 1;
		llio->needs_reset = 1;
		epp_dataportdir_input(port);
		return 0;
	}

	// Write the data from the buffer
	for(wsize = size / 4; wsize; wsize--) {
		uint32_t word = htobe32(*wbuf++);
		uint8_t *pbuf = (uint8_t *)&word;
		unsigned bsize;
		for(bsize = 4; bsize; bsize--) {
			if(!epp_write_cycle(port, *pbuf++, false)) {
				rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.write: Write error on data byte nr. %d of %d\n",
						port->eppid, size - (wsize-1)*4-bsize+1, size);
				if(llio->io_error)
					*llio->io_error = 1;
				llio->needs_reset = 1;
				epp_dataportdir_input(port);
				return 0;
			}
		}
	}

	epp_dataportdir_input(port);
	return 1;
}

/*
 * HM2 interface: Read buffer from an EPP port
 */
static int rpepp_epp_read(hm2_lowlevel_io_t *llio, uint32_t addr, void *buffer, int size)
{
	const hm2_rpepp_t *port = (const hm2_rpepp_t *)llio;
	int wsize;
	uint32_t *wbuf = (uint32_t *)buffer;

	if(size <= 0)
		return 0;
	if(size & 3) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.read: Unaligned read, size=%d is not multiple of 4\n", port->eppid, size);
		return 0;
	}

	epp_dataportdir_output(port);

	// Write the address
	if(!epp_addr_write16(port, addr | HM2_ADDR_AUTOINCR)) {
		if(llio->io_error)
			*llio->io_error = 1;
		llio->needs_reset = 1;
		epp_dataportdir_input(port);
		return 0;
	}

	// Read the data into the buffer
	epp_dataportdir_input(port);
	for(wsize = size / 4; wsize; wsize--) {
		uint32_t word;
		uint8_t *pbuf = (uint8_t *)&word;
		unsigned bsize;
		for(bsize = 4; bsize; pbuf++, bsize--) {
			if(!epp_read_cycle(port, pbuf, false)) {
				rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.read: Read error on data byte nr. %d of %d\n",
						port->eppid, size - (wsize-1)*4-bsize+1, size);
				if(llio->io_error)
					*llio->io_error = 1;
				llio->needs_reset = 1;
				return 0;
			}
		}
		*wbuf++ = be32toh(word);
	}

	return 1;
}

RPEPP_ALWAYS_INLINE static inline int epp_target_reset(const hm2_rpepp_t *port)
{
	unsigned i;
	uint8_t byte;

	// FIXME: Are both reset methods applicable to both 7i43 and 7i90? And,
	// what happens if there is firmware in the FPGA, how does it react to
	// having the CPLD in control again? Are there side-effects?

	epp_dataportdir_output(port);

	//
	// Note: Code adapted from the hm2_7i43.c and hm2_7i90.c sources.
	//

	// This resets the FPGA *only* if it's currently configured with the
	// HostMot2 or GPIO firmware
	// XXX: Why is this done two times?
	for(i = 0; i < 2; i++) {
		if(!epp_addr_write16(port, 0x7f7f)) {	// Write reset address
			epp_dataportdir_input(port);
			return -EIO;
		}
		if(!epp_data_write8(port, 0x5a)) {	// Write reset code
			epp_dataportdir_input(port);
			return -EIO;
		}
	}

	// This code resets the FPGA *only* if the CPLD is in charge of the
	// parallel port. Procedure:
	// 1 - bring the Spartan3's PROG_B line low for 1 us (the specs require
	//     300-500 ns or longer)
	// 2 - bring the Spartan3's PROG_B line high and wait for 2 ms before
	//     sending firmware (required by spec)
	//
	// FIXME: no error checking done on writes
	//
	epp_addr_write8(port, 1);	// Select the control register
	epp_data_write8(port, 0);	// Assert PROG_B
	rtapi_delay(1000);		// Wait 1us
	epp_data_write8(port, 1);	// Deassert PROG_B
	for(i = 0; i < 2*1000; i++)	// Wait 2ms (realtime nightmare)
		rtapi_delay(1000);

	// make sure the FPGA is not asserting its /DONE bit
	epp_dataportdir_input(port);
	if(!epp_read_cycle(port, &byte, false)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.reset: Read data timeout\n", port->eppid);
		return -EIO;
	}

	if(byte & 0x01) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.reset: /DONE is not low after CPLD reset.\n", port->eppid);
		return -EBUSY;
	}
	return 0;
}

/*
 * HM2 interface: Perform a board reset on EPP0
 */
static int rpepp_epp_reset(hm2_lowlevel_io_t *llio)
{
	int rv;

	llio->needs_reset = 0;	// If the reset fails, we cannot do much more
	if((rv = epp_target_reset((const hm2_rpepp_t *)llio)))
		return rv;

	// Getting here means we should be fine
	if(llio->io_error)
		*llio->io_error = 0;

	return 0;
}

RPEPP_ALWAYS_INLINE static inline int epp_program_fpga(const hm2_rpepp_t *port, uint8_t *data, int size)
{
	epp_dataportdir_output(port);
	if(!epp_addr_write8(port, 0)) {		// Select the CPLD's data address
		epp_dataportdir_input(port);
		return -EIO;
	}

	for(; size; size--) {
		if(!epp_data_write8(port, bitfile_reverse_bits(*data++))) {
			epp_dataportdir_input(port);
			return -EIO;
		}
	}

	epp_dataportdir_input(port);
	return 0;
}

static inline void epp_report_program_speed(const hm2_rpepp_t *port, uint64_t ts, uint64_t te, int size)
{
	uint32_t delta = (te - ts) / 1000;	// in microseconds
	rtapi_print_msg(RPEPP_INFO, "EPP%d.program_fpga: %d bytes sent in %u ms (%u kB/s)\n",
		port->eppid, size, delta / 1000, size * 1000 / (delta * 1024));
}

/*
 * HM2 interface: Perform a firmware programming cycle
 */
static int rpepp_epp_program_fpga(hm2_lowlevel_io_t *llio, const bitfile_t *bf)
{
	const hm2_rpepp_t *port = (const hm2_rpepp_t *)llio;
	int rv;
	int64_t ts, te;

	ts = rtapi_get_time();
	rv = epp_program_fpga(port, (uint8_t *)bf->e.data, bf->e.size);
	te = rtapi_get_time();
	if(!rv)
		epp_report_program_speed(port, ts, te, bf->e.size);
	else
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d: firmware programming failed\n", port->eppid);
	return rv;
}

/*********************************************************************/
static int peripheral_map(void)
{
	int fd;
	int err;
	uint32_t membase = BCM2835_GPIO_BASE;	// We know that the GPIO peripheral is first in the map
	long pagesize = sysconf(_SC_PAGESIZE);	// Default mapped page size multiple

	// error, zero or not power of two
	if(-1 == pagesize || !pagesize || (pagesize & (pagesize-1))) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Pagesize is bad (%ld), assuming 4kByte\n", pagesize);
		pagesize = 4096;	// We assume this is a 12-bit (4k) page system, pretty safe
	}

	// Size is independent of the Pi we are running on
	peripheralsize = BCM2835_GPIO_END - membase;	// We know that the GPIO peripheral is last in the map
	peripheralsize += pagesize - 1;			// Round up to next full page
	peripheralsize &= ~(uint32_t)(pagesize - 1);	// Round up to next full page

	switch(platform) {
	case RPI_1:
		break;
	case RPI_2:	// for RPI 3 too
		membase += BCM2709_OFFSET;
		break;
	default:
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Platform not supported\n");
		return -1;
	}

	if((fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: can't open /dev/mem\n");
		return -errno;
	}

	/* mmap BCM2835 GPIO peripheral */
	peripheralmem = mmap(NULL, peripheralsize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, membase);
	err = errno;
	close(fd);

	if(peripheralmem == MAP_FAILED) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Can't map peripherals\n");
		return -err;
	}

	gpio = (bcm2835_gpio_t *)peripheralmem;

	rtapi_print_msg(RPEPP_INFO, "hm2_rpepp: Mapped GPIO from 0x%08x (size 0x%08x) to 0x%p\n",
			membase, (uint32_t)peripheralsize, gpio);

	return 0;
}

/*
 * GPIO pin setup routines
 */
static void waste_150_cycles(void)
{
	unsigned i;
	// A read, memory barrier, an increment, a test and a jump. Should be at least 150 cycles
	for(i = 0; i < 40; i++)
		reg_rd(&gpio->gplev0);	// Just read the pin register, nothing interesting to do here
}

static void inline gpio_pull(unsigned pin, uint32_t pud)
{
	// Enable/disable pullups on the pins on request
	reg_wr(&gpio->gppudclk0, 0);	// We are not sure about the previous state, make sure
	reg_wr(&gpio->gppudclk1, 0);
	waste_150_cycles();		// See GPPUDCLKn description
	reg_wr(&gpio->gppud, pud);
	waste_150_cycles();
	if(pin <= 31) {
		reg_wr(&gpio->gppudclk0, 1 << pin);
		waste_150_cycles();
		reg_wr(&gpio->gppudclk0, 0);
	} else if(pin <= 53) {
		reg_wr(&gpio->gppudclk1, 1 << (pin - 32));
		waste_150_cycles();
		reg_wr(&gpio->gppudclk1, 0);
	}
}

/*
 * GPIO pin setup and restore
 * Provide mapping of the pins such that configured overlaps are detected and
 * complained about.
 */
#define EPP_GPIO_PIN_MIN	2	// Only GPIOs 2...27 are available for use
#define EPP_GPIO_PIN_MAX	27

enum {
	EPP_GPIO_PIN_RESERVED = -1,
	EPP_GPIO_PIN_UNUSED = 0,
	EPP_GPIO_PIN_D0,
	EPP_GPIO_PIN_D1,
	EPP_GPIO_PIN_D2,
	EPP_GPIO_PIN_D3,
	EPP_GPIO_PIN_D4,
	EPP_GPIO_PIN_D5,
	EPP_GPIO_PIN_D6,
	EPP_GPIO_PIN_D7,
	EPP_GPIO_PIN_AS,
	EPP_GPIO_PIN_DS,
	EPP_GPIO_PIN_RW,
	EPP_GPIO_PIN_WAIT,
};

static void pin_setup(int pin, uint32_t io, uint32_t pullup, unsigned state)
{
	gpio_fsel(pin, io);
	gpio_pull(pin, pullup);
	if(state)
		gpio_set(pin);
	else
		gpio_clr(pin);
}

static void pin_restore(int pin)
{
	if(pin >= EPP_GPIO_PIN_MIN && pin <= EPP_GPIO_PIN_MAX) {
		gpio_fsel(pin, GPIO_FSEL_X_GPIO_INPUT);
		gpio_pull(pin, GPIO_GPPUD_PULLUP);
	}
}

static const char *pinfunc_str(int pin)
{
	switch(pin) {
	case EPP_GPIO_PIN_D0: return "EPP_GPIO_PIN_D0";
	case EPP_GPIO_PIN_D1: return "EPP_GPIO_PIN_D1";
	case EPP_GPIO_PIN_D2: return "EPP_GPIO_PIN_D2";
	case EPP_GPIO_PIN_D3: return "EPP_GPIO_PIN_D3";
	case EPP_GPIO_PIN_D4: return "EPP_GPIO_PIN_D4";
	case EPP_GPIO_PIN_D5: return "EPP_GPIO_PIN_D5";
	case EPP_GPIO_PIN_D6: return "EPP_GPIO_PIN_D6";
	case EPP_GPIO_PIN_D7: return "EPP_GPIO_PIN_D7";
	case EPP_GPIO_PIN_AS: return "EPP_GPIO_PIN_AS";
	case EPP_GPIO_PIN_DS: return "EPP_GPIO_PIN_DS";
	case EPP_GPIO_PIN_RW: return "EPP_GPIO_PIN_RW";
	case EPP_GPIO_PIN_WAIT: return "EPP_GPIO_PIN_WAIT";
	}
	return "<unknown function assigned to GPIO pin>";
}

/*
 * Record of the pin mapping between GPIO and EPP function to ensure
 * non-overlapping pin assignment.
 *
 * We are lucky that the RPi has all available GPIO pins combined on one word.
 */
static struct {
	int	port;
	int	func;
} pin_map[EPP_GPIO_PIN_MAX+1] = {
	{ -1, EPP_GPIO_PIN_RESERVED },	// GPIO  0 ID_SD, hat-spec eeprom
	{ -1, EPP_GPIO_PIN_RESERVED },	// GPIO  1 ID_SC, hat-spec eeprom
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO  2 SDA
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO  3 SCL
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO  4 GPCLK0
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO  5
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO  6
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO  7 SPI0/CE1 (D7)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO  8 SPI0/CE0 (D7)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO  9 SPI0/MISO (D6)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 10 SPI0/MOSI (D5)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 11 SPI0/SCLK (D4)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 12
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 13
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 14 UART TXD (/dev/ttyAMA0)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 15 UART RXD (/dev/ttyAMA0)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 16 SPI1/CE2 (D7)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 17 SPI1/CE1 (D7)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 18 SPI1/CE0 (D7)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 19 SPI1/MISO (D6)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 20 SPI1/MOSI (D5)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 21 SPI1/SCLK (D4)
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 22
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 23
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 24
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 25
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 26
	{ -1, EPP_GPIO_PIN_UNUSED },	// GPIO 27
/*
 *	{ -1, EPP_GPIO_PIN_RESERVED },	// GPIO 28, unavailable
 *	{ -1, EPP_GPIO_PIN_RESERVED },	// GPIO 29, unavailable
 *	{ -1, EPP_GPIO_PIN_RESERVED },	// GPIO 30, unavailable
 *	{ -1, EPP_GPIO_PIN_RESERVED }	// GPIO 31, unavailable
 */
};

static bool check_pin(int pin, int func, int port, const char *name)
{
	if(pin < 0 || pin >= NELEM(pin_map)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d: Mapping '%s' to GPIO %d: pin out of range\n", port, name, pin);
		return false;
	}
	if(pin_map[pin].func == EPP_GPIO_PIN_RESERVED) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d: Mapping '%s' to GPIO %d: pin is inaccessible or reserved for other function\n",
				port, name, pin);
		return false;
	}
	if(pin_map[pin].func != EPP_GPIO_PIN_UNUSED) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d: Mapping '%s' to GPIO %d: pin already assigned to function EPP%d:%s\n",
				port, name, pin, pin_map[pin].port, pinfunc_str(pin_map[pin].func));
		return false;
	}
	pin_map[pin].func = func;
	pin_map[pin].port = port;
	return true;
}

/*
 * Return non-zero if a pin is on one of the SPI pins
 */
#define ONSPI_SPI0_CE	0x01
#define ONSPI_SPI0_SIG	0x02
#define ONSPI_SPI1_CE	0x04
#define ONSPI_SPI1_SIG	0x08
static unsigned is_spi_pin(int pin)
{
	switch(pin) {
	case 7:		// SPI0/CE1
	case 8:		// SPI0/CE0
		return ONSPI_SPI0_CE;
	case 9:		// SPI0/MISO
	case 10:	// SPI0/MOSI
	case 11:	// SPI0/SCLK
		return ONSPI_SPI0_SIG;
	case 16:	// SPI1/CE2
	case 17:	// SPI1/CE1
	case 18:	// SPI1/CE0
		return ONSPI_SPI1_CE;
	case 19:	// SPI1/MISO
	case 20:	// SPI1/MOSI
	case 21:	// SPI1/SCLK
		return ONSPI_SPI1_SIG;
	}
	return 0;
}

#define TIMEOUT_SPIN_WAITPIN	125000		// Many cycles >10 ms
static int peripheral_setup(hm2_rpepp_t *board)
{
	uint32_t to;
	int pin;
	int i;
	int offset = board->nr;	// Index offset for module parameter pin layout arrays
	unsigned onspi = 0;
	char name[3];

	board->fsel_msk[0] = 0xffffffff;	// And-mask
	board->fsel_msk[1] = 0xffffffff;
	board->fsel_msk[2] = 0xffffffff;
	board->fsel_out[0] = 0x00000000;	// Or-mask
	board->fsel_out[1] = 0x00000000;
	board->fsel_out[2] = 0x00000000;
	name[0] = 'D';
	name[2] = '\0';
	for(i = 0; i < 8; i++) {
		pin = pin_data[i + 8*offset];
		name[1] = '0' + i;
		if(!check_pin(pin, EPP_GPIO_PIN_D0 + i, board->eppid, name))
			return -EINVAL;
		pin_setup(pin, GPIO_FSEL_X_GPIO_INPUT, GPIO_GPPUD_PULLUP, 1);
		board->pin_data[i] = pin;	// Port pin
		board->msk_data[i] = _BV(pin);	// Port mask
		board->msk_alldata |= _BV(pin);	// Combined mask
		board->fsel_msk[pin / 10] &= ~(RPEPP_GM << (3*(pin % 10)));
		board->fsel_out[pin / 10] |=   RPEPP_GO << (3*(pin % 10));
		onspi |= is_spi_pin(pin);
		rtapi_print_msg(RPEPP_DBG, "hm2_rpepp: EPP%d: GPIO %d -> D%d\n", board->eppid, pin, i);
	}
	rtapi_print_msg(RPEPP_DBG, "hm2_rpepp: EPP%d: fsel_msk[0] = 0x%08x, fsel_out[0] = 0x%08x\n",
			board->eppid, board->fsel_msk[0], board->fsel_out[0]);
	rtapi_print_msg(RPEPP_DBG, "hm2_rpepp: EPP%d: fsel_msk[1] = 0x%08x, fsel_out[1] = 0x%08x\n",
			board->eppid, board->fsel_msk[1], board->fsel_out[1]);
	rtapi_print_msg(RPEPP_DBG, "hm2_rpepp: EPP%d: fsel_msk[2] = 0x%08x, fsel_out[2] = 0x%08x\n",
			board->eppid, board->fsel_msk[2], board->fsel_out[2]);

	rpepp_init_datatable(board);

	pin = pin_rw[1*offset];
	if(!check_pin(pin, EPP_GPIO_PIN_RW, board->eppid, "R/W"))
		return -EINVAL;
	pin_setup(pin, GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF, 1);
	board->pin_rw = pin;
	board->msk_rw = _BV(pin);
	onspi |= is_spi_pin(pin);
	rtapi_print_msg(RPEPP_DBG, "hm2_rpepp: EPP%d: GPIO %d -> R/W\n", board->eppid, pin);

	pin = pin_ds[1*offset];
	if(!check_pin(pin, EPP_GPIO_PIN_DS, board->eppid, "DSTROBE"))
		return -EINVAL;
	pin_setup(pin, GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF, 1);
	board->pin_ds = pin;
	board->msk_ds = _BV(pin);
	onspi |= is_spi_pin(pin);
	rtapi_print_msg(RPEPP_DBG, "hm2_rpepp: EPP%d: GPIO %d -> DSTROBE\n", board->eppid, pin);

	pin = pin_as[1*offset];
	if(!check_pin(pin, EPP_GPIO_PIN_AS, board->eppid, "ASTROBE"))
		return -EINVAL;
	pin_setup(pin, GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF, 1);
	board->pin_as = pin;
	board->msk_as = _BV(pin);
	onspi |= is_spi_pin(pin);
	rtapi_print_msg(RPEPP_DBG, "hm2_rpepp: EPP%d: GPIO %d -> ASTROBE\n", board->eppid, pin);

	pin = pin_wait[1*offset];
	if(!check_pin(pin, EPP_GPIO_PIN_WAIT, board->eppid, "WAIT"))
		return -EINVAL;
	pin_setup(pin, GPIO_FSEL_X_GPIO_INPUT, GPIO_GPPUD_PULLDOWN, 0);
	board->pin_wait = pin;
	board->msk_wait = _BV(pin);
	onspi |= is_spi_pin(pin);
	rtapi_print_msg(RPEPP_DBG, "hm2_rpepp: EPP%d: GPIO %d -> WAIT\n", board->eppid, pin);

#ifdef RPEPP_WAIT_EDGEDETECT
	// Setup riging edge-detect on the WAIT pin
	reg_wr(&gpio->gpren0, reg_rd(&gpio->gpren0) | board->msk_wait);		// Enable rising edge event
	reg_wr(&gpio->gpfen0, reg_rd(&gpio->gpfen0) & ~board->msk_wait);	// Disable falling edge event
	reg_wr(&gpio->gphen0, reg_rd(&gpio->gphen0) & ~board->msk_wait);	// Disable high level event
	reg_wr(&gpio->gplen0, reg_rd(&gpio->gplen0) & ~board->msk_wait);	// Disable low level event
	reg_wr(&gpio->gparen0, reg_rd(&gpio->gparen0) & ~board->msk_wait);	// Disable async falling edge event
	reg_wr(&gpio->gpafen0, reg_rd(&gpio->gpafen0) & ~board->msk_wait);	// Disable async high level event
#endif

	/*
	 * We need to check the pin assignments against SPI mappings and warn
	 * if the pins are not compatible. Otherwise, combining EPP and SPI may
	 * cause failure in some unexpected way.
	 */
	if(epp_spi_check && onspi) {
		if((onspi & ONSPI_SPI0_SIG) && (onspi & ONSPI_SPI1_SIG))
			rtapi_print_msg(RPEPP_WARN, "hm2_rpepp: EPP%d occupies both signals from SPI0 and SPI1\n", board->eppid);
		if(onspi & ONSPI_SPI0_SIG) {
			if(board->pin_data[4] != 11 || board->pin_data[5] != 10 || board->pin_data[6] != 9)
				rtapi_print_msg(RPEPP_WARN, "hm2_rpepp: EPP%d: SPI0 signals not on D[456] or not in right order.\n", board->eppid);
			else if(board->pin_data[7] != 8 && board->pin_data[7] != 7)
				rtapi_print_msg(RPEPP_WARN, "hm2_rpepp: EPP%d: SPI0 has no CE signal on D7.\n", board->eppid);
		}
		if(onspi & ONSPI_SPI1_SIG) {
			if(board->pin_data[4] != 21 || board->pin_data[5] != 20 || board->pin_data[6] != 19)
				rtapi_print_msg(RPEPP_WARN, "hm2_rpepp: EPP%d: SPI1 signals not on D[456] or not in right order.\n", board->eppid);
			else if(board->pin_data[7] != 18 && board->pin_data[7] != 17 && board->pin_data[7] != 16)
				rtapi_print_msg(RPEPP_WARN, "hm2_rpepp: EPP%d: SPI1 has no CE signal on D7.\n", board->eppid);
		}
	}

	/*
	 * Only EPP 1.9 requires the WAIT line to be low to start a cycle.
	 * Older versions (EPP 1.7, pre IEEE 1284) state that the cycle may
	 * start while WAIT is high. However, doing so would not permit us to
	 * have a complete handshake for each and every transfer.
	 * The distinction is not a problem here since we only interface with
	 * known hardware. It would seem that adding all variants is
	 * superfluous. We simply force the handshake to comply to EPP 1.9 and
	 * complain if we meet resistance.
	 */
	for(to = TIMEOUT_SPIN_WAITPIN; to; to--) {
		if(!(reg_rd(&gpio->gplev0) & board->msk_wait))
			break;
	}
	if(!to)
		rtapi_print_msg(RPEPP_WARN, "hm2_rpepp: EPP%d: 'WAIT' line active timeout. Line stuck at high level.\n", board->eppid);

	return 0;
}

static void peripheral_restore(const hm2_rpepp_t *board)
{
	unsigned i;

	for(i = 0; i < 8; i++)
		pin_restore(board->pin_data[i]);

	pin_restore(board->pin_rw);
	pin_restore(board->pin_ds);
	pin_restore(board->pin_as);
	pin_restore(board->pin_wait);
}

/*
 * Get information about the platform we are running on. It is a bit of a crude
 * hack to use /proc/cpuinfo, but it gets the job done. Anyway, this driver is
 * specifically for the RPI.
 */
#define CPUINFO_BUFSIZE	(16*1024)	// Should be large enough for now
static platform_t check_platform(void)
{
	FILE *fp;
	char *buf;
	size_t fsize;
	platform_t rv = RPI_UNSUPPORTED;

	if(!(buf = rtapi_kmalloc(CPUINFO_BUFSIZE, RTAPI_GFP_KERNEL))) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: No dynamic memory\n");
		return RPI_UNSUPPORTED;
	}
	if(!(fp = fopen("/proc/cpuinfo", "r"))) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Failed to open /proc/cpuinfo\n");
		goto check_exit;
	}
	fsize = fread(buf, 1, CPUINFO_BUFSIZE - 1, fp);
	fclose(fp);

	// we have truncated cpuinfo return unsupported
	if(!fsize || fsize == CPUINFO_BUFSIZE - 1) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Platform detection memory buffer too small\n");
		goto check_exit;
	}

	/* NUL terminate the buffer */
	buf[fsize] = '\0';

	if(strstr(buf, "BCM2708")) {
		rv = RPI_1;
	} else if(strstr(buf, "BCM2709")) {
		rv = RPI_2;	//for RPI 3 too
	} else if(strstr(buf, "BCM2835")) {	// starting with 4.8 kernels revision tag has board details
		char *rev_val = strstr(buf, "Revision");
		if(rev_val) {
			char *rev_start = strstr(rev_val, ": ");
			unsigned long rev = strtol(rev_start + 2, NULL, 16);

			if(rev <= 0xffff)
				rv = RPI_1; // pre pi2 revision scheme
			else {
				switch((rev & 0xf000) >> 12) {
				case 0: //bcm2835
					rv = RPI_1;
					break;
				case 1: //bcm2836
				case 2: //bcm2837
					rv = RPI_2;	// peripheral base is same on pi2/3
					break;
				default:
					break;
				}
			}
		}
	}

check_exit:
	rtapi_kfree(buf);
	return rv;
}

/*
 * Setup driver
 * See if we run on a supported platform, setup peripherals and probe boards.
 */
static int32_t check_cookie(hm2_rpepp_t *board)
{
	// The secondary and tertiary cookie values are "HOSTMOT2", but we use
	// binary here to prevent bytesex problems and multibyte character
	// formats. Alternatively, we could use a byte-array, but then the
	// IOCOOKIE values has to be split. Doomed if you do, doomed if you
	// don't...
	static const uint32_t xcookie[3] = {HM2_IOCOOKIE, 0x54534f48, 0x32544f4d};
	uint32_t cookie[4];

	// We read four (4) 32-bit words. The first three are the cookie and
	// the fourth entry is the idrom address offset. The offset is used in
	// the call to get the idrom if we successfully match a cookie.
	if(!board->llio.read(&board->llio, HM2_ADDR_IOCOOKIE, cookie, sizeof(cookie)))
		return -ENODEV;

	if(!memcmp(cookie, xcookie, sizeof(xcookie)))
		return (int32_t)cookie[3];	// The cookie got read correctly

	rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d Invalid cookie, read: %08x %08x %08x,"
			" expected: %08x %08x %08x\n",
			board->eppid,
			cookie[0], cookie[1], cookie[2],
			xcookie[0], xcookie[1], xcookie[2]);

	return -ENODEV;
}

static int probe_board(hm2_rpepp_t *board, const char *fixident)
{
	int32_t ret;
	hm2_idrom_t idrom;
	char *base;

	if(!fixident || !*fixident) {
		// We should be able to probe the FPGA if there is no explicit
		// ident given as parameter. Both the cookie and the board ID
		// (or idrom) should be readable.
		// The check_cookie() call should return the address of the
		// idrom for this particular board and firmware.
		if((ret = check_cookie(board)) < 0) {
			rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Maybe this boards needs FPGA firmware before running (use boardids=xxx).\n");
			return ret;
		}

		rtapi_print_msg(RPEPP_INFO, "hm2_rpepp: EPP%d Valid cookie matched\n", board->eppid);

		// Read the board identfication from idrom (address returned
		// in the cookie check)
		if(!board->llio.read(&board->llio, (uint32_t)ret, &idrom, sizeof(idrom))) {
			rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d Board idrom read failed\n", board->eppid);
			return -EIO;	// We do have a device, but we can't talk to it
		}

		rtapi_print_msg(RPEPP_INFO, "hm2_rpepp: EPP%d Successfully read IDROM from 0x%08x\n", board->eppid, ret);
	} else {
		// We assume that there is no firmware in the board if the
		// command-line specifies a particular board. That means it is
		// a 7i43 and we need to find the fpga size. I'm not sure if
		// this also is applicable for the 7i90, but it should not
		// hurt, probably.
		uint8_t byte;

		strncpy((char *)idrom.board_name, fixident, 8);

		// Find out what size FPGA this 7i43 has
		if((ret = board->llio.reset(&board->llio)) < 0) {	// Make sure the CPLD is in charge of the parallel port
			rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.probe_board: Reset board failed (errno=%d)\n", board->eppid, ret);
			return ret;
		}

		epp_dataportdir_output(board);		// Set port in output mode
		epp_addr_write8(board, 0);		// Select CPLD data register

		epp_dataportdir_input(board);		// Set port in input mode
		if(!epp_read_cycle(board, &byte, false)) {
			rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.probe_board: Read data timeout\n", board->eppid);
			return -EIO;
		}
		if(byte & 0x01)
			idrom.fpga_size = 400;
		else
			idrom.fpga_size = 200;
	}

	if(!memcmp(idrom.board_name, "MESA7I90", 8)) {
		base = "hm2_7i90";
		// XXX: some of these values may/should be taken from the
		// idrom. Do they ever change?
		board->llio.num_ioport_connectors = 3;
		board->llio.pins_per_connector = 24;
		board->llio.ioport_connector_name[0] = "P1";
		board->llio.ioport_connector_name[1] = "P2";
		board->llio.ioport_connector_name[2] = "P3";
		board->llio.num_leds = 2;
		board->llio.fpga_part_number = "xc6slx9tq144";
	} else if(!memcmp(idrom.board_name, "MESA7I43", 8)) {
		base = "hm2_7i43";
		board->llio.num_ioport_connectors = 2;
		board->llio.pins_per_connector = 24;
		board->llio.ioport_connector_name[0] = "P4";
		board->llio.ioport_connector_name[1] = "P3";
		board->llio.num_leds = 8;
		switch(idrom.fpga_size) {
		case 200: board->llio.fpga_part_number = "3s200tq144"; break;
		case 400: board->llio.fpga_part_number = "3s400tq144"; break;
		default:
			rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: 7i43 board at EPP%d has unsupported FPGA size of %u\n", board->eppid, idrom.fpga_size);
			return -1;
		}
	} else {
		int i;
		for(i = 0; i < sizeof(idrom.board_name); i++) {
			if(!isprint(idrom.board_name[i]))
				idrom.board_name[i] = '?';
		}
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Unknown board at EPP%d: %.8s\n", board->eppid, idrom.board_name);
		return -1;
	}

	rtapi_print_msg(RPEPP_INFO, "hm2_rpepp: EPP%d Base: %s.%d\n", board->eppid, base, board->nr);
	rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "%s.%d", base, board->nr);

	return 0;
}

static int rpepp_setup(void)
{
	int i, j;
	int retval = -1;

	// Set process-level message level if requested
	if(epp_debug >= RTAPI_MSG_NONE && epp_debug <= RTAPI_MSG_ALL)
		rtapi_set_msg_level(epp_debug);

	// Must run on a Raspberry Pi
	if((platform = check_platform()) == RPI_UNSUPPORTED) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Unsupported Platform, only Raspberry1/2/3 supported.\n");
		return -1;
	}

	// Check sanity
	if(platform == RPI_1 && (epp_probe & RPEPP_PROBE_1)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP port 1 requested, but Raspberry Pi 1 cannot address EPP port 1 due to missing pins\n");
		return -EINVAL;
	}

	if((retval = peripheral_map()) < 0) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: cannot map peripheral memory.\n");
		return retval;
	}

	// Init the board descriptors with sane data
	memset(boards, 0, sizeof(boards));
	for(i = 0; i < RPEPP_MAX_BOARDS; i++) {
		boards[i].pin_rw = -1;		// Init pins to -1 so we can unwind in case of error
		boards[i].pin_ds = -1;
		boards[i].pin_as = -1;
		boards[i].pin_wait = -1;
		for(j = 0; j < 8; j++)
			boards[i].pin_data[j] = -1;

		boards[i].llio.private = &boards[i].llio;	// Self reference
		boards[i].llio.comp_id = comp_id;		// Upstream reference

		boards[i].llio.read = rpepp_epp_read;
		boards[i].llio.write = rpepp_epp_write;
		boards[i].llio.reset = rpepp_epp_reset;
		boards[i].llio.program_fpga = rpepp_epp_program_fpga;
	}

	// Perform the probes
	for(j = i = 0; i < RPEPP_MAX_BOARDS; i++) {
		if(!(epp_probe & (1 << i)))		// Probe only if requested
			continue;

		boards[j].eppid = i;			// Port reference
		boards[j].nr = j;			// Detect order reference

		if((retval = peripheral_setup(&boards[j])) < 0)
			return retval;

		if((retval = probe_board(&boards[j], boardids[j])) < 0)
			return retval;

		if((retval = hm2_register(&boards[j].llio, config[j])) < 0) {
			rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: hm2_register() failed for EPP%d.\n", boards[j].eppid);
			return retval;
		}

		j++;	// Next board
	}

	return j > 0 ? 0 : -ENODEV;
}

/*
 * Release allocated resources and restore the peripherals used to a stable
 * state.
 */
static void rpepp_cleanup(void)
{
	if((void *)peripheralmem != MAP_FAILED) {
		unsigned i;
		for(i = 0; i < RPEPP_MAX_BOARDS; i++)
			peripheral_restore(&boards[i]);
		munmap(peripheralmem, peripheralsize);
	}
}

/*
 * Main entry and exit points from upstream
 */
int rtapi_app_main()
{
	int ret;

	if((comp_id = ret = hal_init("hm2_rpepp")) < 0)
		goto fail;

	if((ret = rpepp_setup()) < 0)
		goto fail;

	hal_ready(comp_id);
	return 0;

fail:
	rpepp_cleanup();
	return ret;
}

void rtapi_app_exit(void)
{
	rpepp_cleanup();
	hal_exit(comp_id);
}
