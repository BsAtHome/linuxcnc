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
MODULE_SUPPORTED_DEVICE("Mesa-AnythingIO-7i90");

// Forced inline expansion
#define RPEPP_ALWAYS_INLINE	__attribute__((always_inline))

//#define RPEPP_GPIO_GUARD	1	// Define to protect GPIO indices

#define RPEPP_MAX_BOARDS	2
#define HM2_ADDR_AUTOINCR	0x8000

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
#define EPP0_PIN_WAIT		4	// pin  7 Input

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
#define EPP1_PIN_WAIT		2	// pin  3 Input

#define EPP0_DMASK	(	_BV(EPP0_PIN_D0) | _BV(EPP0_PIN_D1) | \
				_BV(EPP0_PIN_D2) | _BV(EPP0_PIN_D3) | \
				_BV(EPP0_PIN_D4) | _BV(EPP0_PIN_D5) | \
				_BV(EPP0_PIN_D6) | _BV(EPP0_PIN_D7))

#define EPP1_DMASK	(	_BV(EPP1_PIN_D0) | _BV(EPP1_PIN_D1) | \
				_BV(EPP1_PIN_D2) | _BV(EPP1_PIN_D3) | \
				_BV(EPP1_PIN_D4) | _BV(EPP1_PIN_D5) | \
				_BV(EPP1_PIN_D6) | _BV(EPP1_PIN_D7))

// The FSEL values are combined into three register access values for
// gpfsel[0..2]. A shortcut is taken for input, which value is zero.
//
// We are lucky with the RPI, which only exports GPIOs up to number 27.
// Therefore, all GPIOs are always contained within the first three gpfsel
// registers.
//
// XXX: These MUST be updated if the pin-assignments change.
//
// Note that input is not defined here. The mask is used to zero the
// appropriate FSEL bits and zero is the value neede for input.
//
#define RPEPP_GM		7			// FSEL Mask value
#define RPEPP_GO		GPIO_FSEL_X_GPIO_OUTPUT	// FSEL output

#define EPP0_FSEL_0_MASK	((RPEPP_GM << (3*(EPP0_PIN_D3 % 10))) | (RPEPP_GM << (3*(EPP0_PIN_D6 % 10))) | (RPEPP_GM << (3*(EPP0_PIN_D7 % 10))))
#define EPP0_FSEL_1_MASK	((RPEPP_GM << (3*(EPP0_PIN_D5 % 10))) | (RPEPP_GM << (3*(EPP0_PIN_D4 % 10))))
#define EPP0_FSEL_2_MASK	((RPEPP_GM << (3*(EPP0_PIN_D0 % 10))) | (RPEPP_GM << (3*(EPP0_PIN_D1 % 10))) | (RPEPP_GM << (3*(EPP0_PIN_D2 % 10))))
#define EPP0_FSEL_0_OUT		((RPEPP_GO << (3*(EPP0_PIN_D3 % 10))) | (RPEPP_GO << (3*(EPP0_PIN_D6 % 10))) | (RPEPP_GO << (3*(EPP0_PIN_D7 % 10))))
#define EPP0_FSEL_1_OUT		((RPEPP_GO << (3*(EPP0_PIN_D5 % 10))) | (RPEPP_GO << (3*(EPP0_PIN_D4 % 10))))
#define EPP0_FSEL_2_OUT		((RPEPP_GO << (3*(EPP0_PIN_D0 % 10))) | (RPEPP_GO << (3*(EPP0_PIN_D1 % 10))) | (RPEPP_GO << (3*(EPP0_PIN_D2 % 10))))

#define EPP1_FSEL_0_MASK	((RPEPP_GM << (3*(EPP1_PIN_D1 % 10))) | (RPEPP_GM << (3*(EPP1_PIN_D2 % 10))) | (RPEPP_GM << (3*(EPP1_PIN_D3 % 10))))
#define EPP1_FSEL_1_MASK	((RPEPP_GM << (3*(EPP1_PIN_D0 % 10))) | (RPEPP_GM << (3*(EPP1_PIN_D6 % 10))) | (RPEPP_GM << (3*(EPP1_PIN_D7 % 10))))
#define EPP1_FSEL_2_MASK	((RPEPP_GM << (3*(EPP1_PIN_D4 % 10))) | (RPEPP_GM << (3*(EPP1_PIN_D5 % 10))))
#define EPP1_FSEL_0_OUT		((RPEPP_GO << (3*(EPP1_PIN_D1 % 10))) | (RPEPP_GO << (3*(EPP1_PIN_D2 % 10))) | (RPEPP_GO << (3*(EPP1_PIN_D3 % 10))))
#define EPP1_FSEL_1_OUT		((RPEPP_GO << (3*(EPP1_PIN_D0 % 10))) | (RPEPP_GO << (3*(EPP1_PIN_D6 % 10))) | (RPEPP_GO << (3*(EPP1_PIN_D7 % 10))))
#define EPP1_FSEL_2_OUT		((RPEPP_GO << (3*(EPP1_PIN_D4 % 10))) | (RPEPP_GO << (3*(EPP1_PIN_D5 % 10))))

#define EPP_PORT_0		0	// Identifiers for port selection. Must be
#define EPP_PORT_1		1	// constants for inline to work effectively.

typedef struct hm2_rpepp_struct {
	hm2_lowlevel_io_t llio;		// Upstream container
	int		nr;		// Board number
	int		eppid;		// Which port is mapped
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
 * boardid[]: define the attached board if it cannot be probed
 * Supported values are (case sensitive):
 * - MESA7I90
 * - MESA7I43
 */
static char *boardid[RPEPP_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(boardid, RPEPP_MAX_BOARDS, "Board identifiers (comma separated) if it cannot be probed due to lack of firmware (default: not set)")

/*
 * Set the message level for debugging purpose. This has the (side-)effect that
 * all modules within this process will start spitting out messages at the
 * requested level.
 * The upstream message level is not touched if epp_debug == -1.
 */
static int epp_debug = -1;
RTAPI_MP_INT(epp_debug, "Set message level for debugging purpose [0...5] where 0=none and 5=all (default: -1; upstream defined)")

/*
 * Code comments:
 * All conversion and support routines are forced to expand inline because they
 * are rather timing sensitive in the read/write loops.
 *
 * Many routines are duplicated for EPP0 and EPP1. The reason for duplication
 * is that they must call upstream (inline) functions with constants. Using
 * constants in inline expansions allows the compiler to eliminate dead code
 * and reduce conditionals to only the fraction that is actually executed.
 *
 * Inline functions here preserve the functional abstraction without the
 * overhead of calling functions.
 */

/*
 * The port's data-bit positions are mangled with respect to the actual value
 * we want to write or read. These routines do the bit-mangling/unmangling for
 * us.
 *
 * Byte->word conversion is faster using tables, which are created at startup.
 * The word->byte conversion is more tricky and tables are not a real option.
 */
RPEPP_ALWAYS_INLINE static inline uint8_t epp_port_to_data(unsigned port, uint32_t val)
{
	uint8_t data = 0;
	if(!port) {
		if(val & _BV(EPP0_PIN_D0))	data |= 0x01;
		if(val & _BV(EPP0_PIN_D1))	data |= 0x02;
		if(val & _BV(EPP0_PIN_D2))	data |= 0x04;
		if(val & _BV(EPP0_PIN_D3))	data |= 0x08;
		if(val & _BV(EPP0_PIN_D4))	data |= 0x10;
		if(val & _BV(EPP0_PIN_D5))	data |= 0x20;
		if(val & _BV(EPP0_PIN_D6))	data |= 0x40;
		if(val & _BV(EPP0_PIN_D7))	data |= 0x80;
	} else {
		if(val & _BV(EPP1_PIN_D0))	data |= 0x01;
		if(val & _BV(EPP1_PIN_D1))	data |= 0x02;
		if(val & _BV(EPP1_PIN_D2))	data |= 0x04;
		if(val & _BV(EPP1_PIN_D3))	data |= 0x08;
		if(val & _BV(EPP1_PIN_D4))	data |= 0x10;
		if(val & _BV(EPP1_PIN_D5))	data |= 0x20;
		if(val & _BV(EPP1_PIN_D6))	data |= 0x40;
		if(val & _BV(EPP1_PIN_D7))	data |= 0x80;
	}
	return data;
}

static uint32_t epp0_data_to_port_bytetable[256];	// Bit translation tables
static uint32_t epp1_data_to_port_bytetable[256];

RPEPP_ALWAYS_INLINE static inline uint32_t epp_data_to_port(unsigned port, uint8_t val)
{
	return port ? epp1_data_to_port_bytetable[val] : epp0_data_to_port_bytetable[val];
}

static void rpepp_init_datatables(void)
{
	unsigned i;
	// Crude but effective
	for(i = 0; i < 256; i++) {
		uint32_t port0 = 0;
		uint32_t port1 = 0;
		if(i & 0x01) { port0 |= _BV(EPP0_PIN_D0); port1 |= _BV(EPP1_PIN_D0); }
		if(i & 0x02) { port0 |= _BV(EPP0_PIN_D1); port1 |= _BV(EPP1_PIN_D1); }
		if(i & 0x04) { port0 |= _BV(EPP0_PIN_D2); port1 |= _BV(EPP1_PIN_D2); }
		if(i & 0x08) { port0 |= _BV(EPP0_PIN_D3); port1 |= _BV(EPP1_PIN_D3); }
		if(i & 0x10) { port0 |= _BV(EPP0_PIN_D4); port1 |= _BV(EPP1_PIN_D4); }
		if(i & 0x20) { port0 |= _BV(EPP0_PIN_D5); port1 |= _BV(EPP1_PIN_D5); }
		if(i & 0x40) { port0 |= _BV(EPP0_PIN_D6); port1 |= _BV(EPP1_PIN_D6); }
		if(i & 0x80) { port0 |= _BV(EPP0_PIN_D7); port1 |= _BV(EPP1_PIN_D7); }
		epp0_data_to_port_bytetable[i] = port0;
		epp1_data_to_port_bytetable[i] = port1;
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
 * Call these routines with a constant arguments. The compiler cannot eliminate
 * index calculations and the conditional statements if the argumemts are not
 * constant and will therefore have a significant performance impact.
 *
 * The performance has to be reevaluated if the code is adapted to configurable
 * pins from module parameters. A whole lot of memory accesses are required
 * when the pins are no longer a constant.
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
 */
RPEPP_ALWAYS_INLINE static inline void epp_dataportdir_output(unsigned port)
{
	if(!port) {
		reg_wr(&gpio->gpfsel0, reg_rd(&gpio->gpfsel0) | EPP0_FSEL_0_OUT);
		reg_wr(&gpio->gpfsel1, reg_rd(&gpio->gpfsel1) | EPP0_FSEL_1_OUT);
		reg_wr(&gpio->gpfsel2, reg_rd(&gpio->gpfsel2) | EPP0_FSEL_2_OUT);
	} else {
		reg_wr(&gpio->gpfsel0, reg_rd(&gpio->gpfsel0) | EPP1_FSEL_0_OUT);
		reg_wr(&gpio->gpfsel1, reg_rd(&gpio->gpfsel1) | EPP1_FSEL_1_OUT);
		reg_wr(&gpio->gpfsel2, reg_rd(&gpio->gpfsel2) | EPP1_FSEL_2_OUT);
	}
}

RPEPP_ALWAYS_INLINE static inline void epp_dataportdir_input(unsigned port)
{
	if(!port) {
		reg_wr(&gpio->gpfsel0, reg_rd(&gpio->gpfsel0) & ~EPP0_FSEL_0_MASK);
		reg_wr(&gpio->gpfsel1, reg_rd(&gpio->gpfsel1) & ~EPP0_FSEL_1_MASK);
		reg_wr(&gpio->gpfsel2, reg_rd(&gpio->gpfsel2) & ~EPP0_FSEL_2_MASK);
	} else {
		reg_wr(&gpio->gpfsel0, reg_rd(&gpio->gpfsel0) & ~EPP1_FSEL_0_MASK);
		reg_wr(&gpio->gpfsel1, reg_rd(&gpio->gpfsel1) & ~EPP1_FSEL_1_MASK);
		reg_wr(&gpio->gpfsel2, reg_rd(&gpio->gpfsel2) & ~EPP1_FSEL_2_MASK);
	}
}

/*
 * Perform EPP transactions
 */
RPEPP_ALWAYS_INLINE static inline void epp_assert_strobe(unsigned port, bool ad_strobe, bool active)
{
	// Strobes are active low
	if(!port) {
		if(active) {
			if(ad_strobe)
				reg_wr(&gpio->gpclr0, _BV(EPP0_PIN_AS));	// Address strobe to low
			else
				reg_wr(&gpio->gpclr0, _BV(EPP0_PIN_DS));	// Data strobe to low
		} else {
			if(ad_strobe)
				reg_wr(&gpio->gpset0, _BV(EPP0_PIN_AS));	// Address strobe to high
			else
				reg_wr(&gpio->gpset0, _BV(EPP0_PIN_DS));	// Data strobe to high
		}
	} else {
		if(active) {
			if(ad_strobe)
				reg_wr(&gpio->gpclr0, _BV(EPP1_PIN_AS));	// Address strobe to low
			else
				reg_wr(&gpio->gpclr0, _BV(EPP1_PIN_DS));	// Data strobe to low
		} else {
			if(ad_strobe)
				reg_wr(&gpio->gpset0, _BV(EPP1_PIN_AS));	// Address strobe to high
			else
				reg_wr(&gpio->gpset0, _BV(EPP1_PIN_DS));	// Data strobe to high
		}
	}
}

#define TIMEOUT_SPINS	1000	// Many clock cycles (>80 us)

RPEPP_ALWAYS_INLINE static inline bool epp_write_cycle(unsigned port, uint8_t data, bool ad_strobe)
{
	// It is assumed that the data port is set to output at this time
	uint32_t portdata;
	unsigned timeout;

	portdata = epp_data_to_port(port, data);

	reg_wr(&gpio->gpclr0, _BV(port ? EPP1_PIN_RW : EPP0_PIN_RW));		// Clear R/W
	reg_wr(&gpio->gpclr0, portdata ^ (port ? EPP1_DMASK : EPP0_DMASK));	// Clear data bits to clear
	reg_wr(&gpio->gpset0, portdata);					// Set data bits to set

	// Wait for the WAIT line to be(come) low
	timeout = TIMEOUT_SPINS;
	while((reg_rd(&gpio->gplev0) & _BV(port ? EPP1_PIN_WAIT : EPP0_PIN_WAIT)) && --timeout)
		;
	if(!timeout) {
		// The WAIT line is active --> no board attached?
		return false;
	}

	epp_assert_strobe(port, ad_strobe, true);

	// Wait for the WAIT line to go high
	timeout = TIMEOUT_SPINS;
	while(!(reg_rd(&gpio->gplev0) & _BV(port ? EPP1_PIN_WAIT : EPP0_PIN_WAIT)) && --timeout)
		;
	if(!timeout) {
		// Timeout on WAIT line --> maybe no board attached?
		epp_assert_strobe(port, ad_strobe, false);
		return false;
	}

	epp_assert_strobe(port, ad_strobe, false);	// Done transaction (we don't wait for the WAIT signal)
	return true;
}

RPEPP_ALWAYS_INLINE static inline bool epp_read_cycle(unsigned port, uint8_t *data, bool ad_strobe)
{
	// It is assumed that the data port is set to input at this time
	uint32_t portdata;
	unsigned timeout;

	reg_wr(&gpio->gpset0, _BV(port ? EPP1_PIN_RW : EPP0_PIN_RW));	// Set R/W

	// Wait for the WAIT line to be(come) low
	timeout = TIMEOUT_SPINS;
	while((reg_rd(&gpio->gplev0) & _BV(port ? EPP1_PIN_WAIT : EPP0_PIN_WAIT)) && --timeout)
		;
	if(!timeout) {
		// The WAIT line is active --> no board attached?
		return false;
	}

	epp_assert_strobe(port, ad_strobe, true);

	// Wait for the WAIT line to go high
	timeout = TIMEOUT_SPINS;
	while(!(reg_rd(&gpio->gplev0) & _BV(port ? EPP1_PIN_WAIT : EPP0_PIN_WAIT)) && --timeout)
		;
	if(!timeout) {
		// Timeout on WAIT line --> maybe no board attached?
		epp_assert_strobe(port, ad_strobe, false);
		return false;
	}

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
	// is about 80ns per spin (@1.2GHz). One dummy synchronous read should
	// give us some 20+ns settling time.

	reg_rd(&gpio->gplev0);	// Dummy read to allow for data settling

	portdata = reg_rd(&gpio->gplev0);		// Get the data
	epp_assert_strobe(port, ad_strobe, false);	// Done transaction (we don't wait for the WAIT signal)

	*data = epp_port_to_data(port, portdata);	// Store the data

	return true;
}

RPEPP_ALWAYS_INLINE static inline bool epp_addr_write16(unsigned port, uint16_t val)
{
	// Write two bytes to address port in lo-hi-byte order
	if(!epp_write_cycle(port, val & 0xff, true)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%u.addr_write16: Write timeout on address write byte 0\n", port);
		return false;
	}
	if(!epp_write_cycle(port, (val >> 8) & 0xff, true)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%u.addr_write16: Write timeout on address write byte 1\n", port);
		return false;
	}
	return true;
}

RPEPP_ALWAYS_INLINE static inline bool epp_addr_write8(unsigned port, uint8_t val)
{
	// Write a single byte to address port
	if(!epp_write_cycle(port, val & 0xff, true)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%u.addr_write8: Write timeout\n", port);
		return false;
	}
	return true;
}

RPEPP_ALWAYS_INLINE static inline bool epp_data_write8(unsigned port, uint8_t val)
{
	// Write a single byte to data port
	if(!epp_write_cycle(port, val, false)) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%u.data_write8: Write timeout\n", port);
		return false;
	}
	return true;
}

/*********************************************************************/
/*
 * HM2 interface: Write buffer to EPP0
 */
static int rpepp_epp0_write(hm2_lowlevel_io_t *llio, uint32_t addr, void *buffer, int size)
{
	int wsize;
	uint32_t *wbuf = (uint32_t *)buffer;

	if(size <= 0)
		return 0;
	if(size & 3) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP0.write: Unaligned write, size=%d is not multiple of 4\n", size);
		return 0;
	}

	epp_dataportdir_output(EPP_PORT_0);

	// Write the address
	if(!epp_addr_write16(EPP_PORT_0, addr | HM2_ADDR_AUTOINCR)) {
		if(llio->io_error)
			*llio->io_error = 1;
		llio->needs_reset = 1;
		epp_dataportdir_input(EPP_PORT_0);
		return 0;
	}

	// Write the data from the buffer
	for(wsize = size / 4; wsize; wsize--) {
		uint32_t word = htobe32(*wbuf++);
		uint8_t *pbuf = (uint8_t *)&word;
		unsigned bsize;
		for(bsize = 4; bsize; bsize--) {
			if(!epp_write_cycle(EPP_PORT_0, *pbuf++, false)) {
				rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP0.write: Write error on data byte nr. %d of %d\n", size - (wsize-1)*4-bsize+1, size);
				if(llio->io_error)
					*llio->io_error = 1;
				llio->needs_reset = 1;
				epp_dataportdir_input(EPP_PORT_0);
				return 0;
			}
		}
	}

	epp_dataportdir_input(EPP_PORT_0);
	return 1;
}

/*
 * HM2 interface: Read buffer from EPP0
 */
static int rpepp_epp0_read(hm2_lowlevel_io_t *llio, uint32_t addr, void *buffer, int size)
{
	int wsize;
	uint32_t *wbuf = (uint32_t *)buffer;

	if(size <= 0)
		return 0;
	if(size & 3) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP0.read: Unaligned read, size=%d is not multiple of 4\n", size);
		return 0;
	}

	epp_dataportdir_output(EPP_PORT_0);

	// Write the address
	if(!epp_addr_write16(EPP_PORT_0, addr | HM2_ADDR_AUTOINCR)) {
		if(llio->io_error)
			*llio->io_error = 1;
		llio->needs_reset = 1;
		epp_dataportdir_input(EPP_PORT_0);
		return 0;
	}

	// Read the data into the buffer
	epp_dataportdir_input(EPP_PORT_0);
	for(wsize = size / 4; wsize; wsize--) {
		uint32_t word;
		uint8_t *pbuf = (uint8_t *)&word;
		unsigned bsize;
		for(bsize = 4; bsize; pbuf++, bsize--) {
			if(!epp_read_cycle(EPP_PORT_0, pbuf, false)) {
				rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP0.read: Read error on data byte nr. %d of %d\n", size - (wsize-1)*4-bsize+1, size);
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

/*
 * HM2 interface: Write buffer to EPP1
 */
static int rpepp_epp1_write(hm2_lowlevel_io_t *llio, uint32_t addr, void *buffer, int size)
{
	int wsize;
	uint32_t *wbuf = (uint32_t *)buffer;

	if(size <= 0)
		return 0;
	if(size & 3) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP1.write: Unaligned write, size=%d is not multiple of 4\n", size);
		return 0;
	}

	epp_dataportdir_output(EPP_PORT_1);

	// Write the address
	if(!epp_addr_write16(EPP_PORT_1, addr | HM2_ADDR_AUTOINCR)) {
		if(llio->io_error)
			*llio->io_error = 1;
		llio->needs_reset = 1;
		epp_dataportdir_input(EPP_PORT_1);
		return 0;
	}

	// Write the data from the buffer
	for(wsize = size / 4; wsize; wsize--) {
		uint32_t word = htobe32(*wbuf++);
		uint8_t *pbuf = (uint8_t *)&word;
		unsigned bsize;
		for(bsize = 4; bsize; bsize--) {
			if(!epp_write_cycle(EPP_PORT_1, *pbuf++, false)) {
				rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP1.write: Write error on data byte nr. %d of %d\n", size - (wsize-1)*4-bsize+1, size);
				if(llio->io_error)
					*llio->io_error = 1;
				llio->needs_reset = 1;
				epp_dataportdir_input(EPP_PORT_1);
				return 0;
			}
		}
	}

	epp_dataportdir_input(EPP_PORT_1);
	return 1;
}

/*
 * HM2 interface: Read buffer from EPP0
 */
static int rpepp_epp1_read(hm2_lowlevel_io_t *llio, uint32_t addr, void *buffer, int size)
{
	int wsize;
	uint32_t *wbuf = (uint32_t *)buffer;

	if(size <= 0)
		return 0;
	if(size & 3) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP1.write: Unaligned read, size=%d is not multiple of 4\n", size);
		return 0;
	}

	epp_dataportdir_output(EPP_PORT_1);

	// Write the address
	if(!epp_addr_write16(EPP_PORT_1, addr | HM2_ADDR_AUTOINCR)) {
		if(llio->io_error)
			*llio->io_error = 1;
		llio->needs_reset = 1;
		epp_dataportdir_input(EPP_PORT_1);
		return 0;
	}

	// Read the data into the buffer
	epp_dataportdir_input(EPP_PORT_1);
	for(wsize = size / 4; wsize; wsize--) {
		uint32_t word;
		uint8_t *pbuf = (uint8_t *)&word;
		unsigned bsize;
		for(bsize = 4; bsize; pbuf++, bsize--) {
			if(!epp_read_cycle(EPP_PORT_1, pbuf, false)) {
				rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP1.read: Read error on data byte nr. %d of %d\n", size - (wsize-1)*4-bsize+1, size);
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

RPEPP_ALWAYS_INLINE static inline int epp_target_reset(unsigned port)
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
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%u.reset: Read data timeout\n", port);
		return -EIO;
	}

	if(byte & 0x01) {
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%u.reset: /DONE is not low after CPLD reset.\n", port);
		return -EBUSY;
	}
	return 0;
}

/*
 * HM2 interface: Perform a board reset on EPP0
 */
static int rpepp_epp0_reset(hm2_lowlevel_io_t *llio)
{
	int rv;

	llio->needs_reset = 0;	// If the reset fails, we cannot do much more
	if((rv = epp_target_reset(EPP_PORT_0)))
		return rv;

	// Getting here means we should be fine
	if(llio->io_error)
		*llio->io_error = 0;

	return 0;
}

/*
 * HM2 interface: Perform a board reset on EPP1
 */
static int rpepp_epp1_reset(hm2_lowlevel_io_t *llio)
{
	int rv;

	llio->needs_reset = 0;	// If the reset fails, we cannot do much more
	if((rv = epp_target_reset(EPP_PORT_1)))
		return rv;

	// Getting here means we should be fine
	if(llio->io_error)
		*llio->io_error = 0;

	return 0;
}

RPEPP_ALWAYS_INLINE static inline int epp_program_fpga(unsigned port, uint8_t *data, int size)
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

static inline void epp_report_program_speed(unsigned port, uint64_t ts, uint64_t te, int size)
{
	uint32_t delta = (te - ts) / 1000;	// in microseconds
	rtapi_print_msg(RPEPP_INFO, "EPP%u.program_fpga: %d bytes sent in %u ms (%u kB/s)\n",
		port, size, delta / 1000, size * 1000 / (delta * 1024));
}

/*
 * HM2 interface: Perform a firmware programming cycle on EPP0
 */
static int rpepp_epp0_program_fpga(hm2_lowlevel_io_t *llio, const bitfile_t *bf)
{
	int rv;
	int64_t ts, te;

	(void)llio;

	ts = rtapi_get_time();
	rv = epp_program_fpga(EPP_PORT_0, (uint8_t *)bf->e.data, bf->e.size);
	te = rtapi_get_time();
	if(!rv)
		epp_report_program_speed(EPP_PORT_0, ts, te, bf->e.size);
	else
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP0: firmware programming failed\n");
	return rv;
}

/*
 * HM2 interface: Perform a firmware programming cycle on EPP0
 */
static int rpepp_epp1_program_fpga(hm2_lowlevel_io_t *llio, const bitfile_t *bf)
{
	int rv;
	int64_t ts, te;

	(void)llio;

	ts = rtapi_get_time();
	rv = epp_program_fpga(EPP_PORT_1, (uint8_t *)bf->e.data, bf->e.size);
	te = rtapi_get_time();
	if(!rv)
		epp_report_program_speed(EPP_PORT_1, ts, te, bf->e.size);
	else
		rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP1: firmware programming failed\n");
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
 * Pin setup definitions
 * Tables are a quick and clean way to define all pins and settings.
 */
typedef struct __epp_pinsetup_t {
	uint8_t	pin;		// The GPIO pin number
	uint8_t	io;		// Input/output FSEL selector
	uint8_t pullup;		// Pull-up/pull-down/off
	uint8_t state;		// Output level after init
} pinsetup_t;

static const pinsetup_t epp0_pinsetup[] = {
	{EPP0_PIN_D0,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP0_PIN_D1,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP0_PIN_D2,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP0_PIN_D3,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP0_PIN_D4,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP0_PIN_D5,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP0_PIN_D6,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP0_PIN_D7,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP0_PIN_DS,   GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF,      1},
	{EPP0_PIN_AS,   GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF,      1},
	{EPP0_PIN_RW,   GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF,      1},
	{EPP0_PIN_WAIT, GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLDOWN, 0}
};

static const pinsetup_t epp1_pinsetup[] = {
	{EPP1_PIN_D0,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP1_PIN_D1,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP1_PIN_D2,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP1_PIN_D3,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP1_PIN_D4,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP1_PIN_D5,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP1_PIN_D6,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP1_PIN_D7,   GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLUP,   1},
	{EPP1_PIN_DS,   GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF,      1},
	{EPP1_PIN_AS,   GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF,      1},
	{EPP1_PIN_RW,   GPIO_FSEL_X_GPIO_OUTPUT, GPIO_GPPUD_OFF,      1},
	{EPP1_PIN_WAIT, GPIO_FSEL_X_GPIO_INPUT,  GPIO_GPPUD_PULLDOWN, 0}
};

static void pins_setup(const pinsetup_t *pinsetup, unsigned n)
{
	unsigned i;

	for(i = 0; i < n; i++) {
		gpio_fsel(pinsetup[i].pin, pinsetup[i].io);
		gpio_pull(pinsetup[i].pin, pinsetup[i].pullup);
		if(pinsetup[i].state)
			gpio_set(pinsetup[i].pin);
		else
			gpio_clr(pinsetup[i].pin);
	}
}

static void pins_restore(const pinsetup_t *pinsetup, unsigned n)
{
	unsigned i;

	for(i = 0; i < n; i++) {
		gpio_fsel(pinsetup[i].pin, GPIO_FSEL_X_GPIO_INPUT);
		gpio_pull(pinsetup[i].pin, GPIO_GPPUD_PULLUP);
	}
}

#define TIMEOUT_SPIN_WAITPIN	125000		// Many cycles >10 ms
static void peripheral_setup(void)
{
	uint32_t to;
	if(epp_probe & RPEPP_PROBE_0) {	// Port EPP0 pin setup
		pins_setup(epp0_pinsetup, NELEM(epp0_pinsetup));
		for(to = TIMEOUT_SPIN_WAITPIN; to; to--) {
			if(!(reg_rd(&gpio->gplev0) & _BV(EPP0_PIN_WAIT)))
				break;
		}
		if(!to)
			rtapi_print_msg(RPEPP_WARN, "hm2_rpepp: EPP0 'wait' line active timeout. Line stuck at high level.\n");
	}

	if(epp_probe & RPEPP_PROBE_1) {	// Port EPP1 pin setup
		pins_setup(epp1_pinsetup, NELEM(epp1_pinsetup));
		for(to = TIMEOUT_SPIN_WAITPIN; to; to--) {
			if(!(reg_rd(&gpio->gplev0) & _BV(EPP1_PIN_WAIT)))
				break;
		}
		if(!to)
			rtapi_print_msg(RPEPP_WARN, "hm2_rpepp: EPP1 'wait' line active timeout. Line stuck at high level.\n");
	}
}

static void peripheral_restore(void)
{
	if(epp_probe & RPEPP_PROBE_0)	// Port EPP0 pin setup
		pins_restore(epp0_pinsetup, NELEM(epp0_pinsetup));

	if(epp_probe & RPEPP_PROBE_1)	// Port EPP1 pin setup
		pins_restore(epp1_pinsetup, NELEM(epp1_pinsetup));
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
			rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Maybe this boards needs FPGA firmware before running (use boardid=xxx).\n");
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
		if(board->eppid) {
			epp_dataportdir_output(EPP_PORT_1);	// Set port in output mode
			epp_addr_write8(EPP_PORT_1, 0);		// Select CPLD data register

			epp_dataportdir_input(EPP_PORT_1);	// Set port in input mode
			if(!epp_read_cycle(EPP_PORT_1, &byte, false)) {
				rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.probe_board: Read data timeout\n", board->eppid);
				return -EIO;
			}
		} else {
			epp_dataportdir_output(EPP_PORT_0);	// Set port in output mode
			epp_addr_write8(EPP_PORT_0, 0);		// Select CPLD data register

			epp_dataportdir_input(EPP_PORT_0);	// Set port in input mode
			if(!epp_read_cycle(EPP_PORT_0, &byte, false)) {
				rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: EPP%d.probe_board: Read data timeout\n", board->eppid);
				return -EIO;
			}
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

	rpepp_init_datatables();	// Bit translation tables

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

	peripheral_setup();

	memset(boards, 0, sizeof(boards));

	for(j = i = 0; i < RPEPP_MAX_BOARDS; i++) {
		if(!(epp_probe & (1 << i)))		// Probe only if requested
			continue;

		boards[j].llio.private = &boards[j].llio;	// Self reference
		boards[j].llio.comp_id = comp_id;		// Upstream reference
		boards[j].eppid = i;				// Port reference
		boards[j].nr = j;				// Detect order reference

		switch(i) {
		case EPP_PORT_0:
			boards[j].llio.read = rpepp_epp0_read;
			boards[j].llio.write = rpepp_epp0_write;
			boards[j].llio.reset = rpepp_epp0_reset;
			boards[j].llio.program_fpga = rpepp_epp0_program_fpga;
			break;
		case EPP_PORT_1:
			boards[j].llio.read = rpepp_epp1_read;
			boards[j].llio.write = rpepp_epp1_write;
			boards[j].llio.reset = rpepp_epp1_reset;
			boards[j].llio.program_fpga = rpepp_epp1_program_fpga;
			break;
		default:
			// This should never happen...
			rtapi_print_msg(RPEPP_ERR, "hm2_rpepp: Internal error. Trying to probe non-existent EPP port\n");
			return -EFAULT;
		}

		if((retval = probe_board(&boards[j], boardid[j])) < 0) {
			return retval;
		}

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
		peripheral_restore();
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
