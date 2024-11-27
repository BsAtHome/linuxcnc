/*
 * This is a component for RaspberryPi 5 to hostmot2 over SPI for linuxcnc.
 * Copyright (c) 2024 B.Stultiens <lcnc@vagrearg.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <https://www.gnu.org/licenses/>.
 */

/* Without Source Tree */
#undef WOST

#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <errno.h>

#include <hal.h>
#include <rtapi.h>
#include <rtapi_app.h>
#include <rtapi_slab.h>

#define HM2_LLIO_NAME "hm2_rp5spi"

#include "hostmot2-lowlevel.h"
#include "hostmot2.h"
#include "llio_info.h"
#include "eshellf.h"

#include "dtcboards.h"
#include "rp1dev.h"

//#define RPSPI_DEBUG_PIN	23	// Define for pin-debugging

MODULE_LICENSE("GPL");
MODULE_AUTHOR("B.Stultiens");
MODULE_DESCRIPTION("Driver for HostMot2 devices connected via SPI to RaspberryPi5");
MODULE_SUPPORTED_DEVICE("Mesa-AnythingIO-7i90,7c80,7c81");

#define RPSPI_MAX_BOARDS	5			// One on each (traditional) CE for SPI ports 0 and 1
#define RPSPI_MAX_MSG		(127+1)		// The docs say that the max. burstlen == 127 words (i.e. cmd+message <= 1+127)
#define RPSPI_MAX_SPI		2			// SPI0 and SPI1

// The min/max allowed frequencies of the SPI clock
#define SCLK_FREQ_MIN	4
#define SCLK_FREQ_MAX	50000
#define SCLK_FREQ_DEF	25000	// Default

// Preprocessor stringize
#define __PPSTR(x)	#x
#define PPSTR(x)	__PPSTR(x)

// GPIO pin definitions				// (header pin location)
#define SPI0_PIN_CE_1		7		// (pin 26)
#define SPI0_PIN_CE_0		8		// (pin 24)
#define SPI0_PIN_MISO		9		// (pin 21)
#define SPI0_PIN_MOSI		10		// (pin 19)
#define SPI0_PIN_SCLK		11		// (pin 23)
#define SPI1_PIN_CE_2		16		// (pin 36)
#define SPI1_PIN_CE_1		17		// (pin 11)
#define SPI1_PIN_CE_0		18		// (pin 12)
#define SPI1_PIN_MISO		19		// (pin 35)
#define SPI1_PIN_MOSI		20		// (pin 38)
#define SPI1_PIN_SCLK		21		// (pin 40)

typedef struct __buffer_t {
	void		*ptr;	// Actual buffer
	size_t		n;		// Number of elements in buffer
	size_t		na;		// Allocated size of buffer in number of elements
} buffer_t;

typedef struct __rxref_t {
	void	*ptr;		// The read buffer from the queue_read call
	int		size;		// Size of read buffer from the queue_read call
	int		idx;		// Data position index into the board's rbuf data
} rxref_t;

// Our data container
typedef struct hm2_rp5spi_struct {
	hm2_lowlevel_io_t llio;	// Upstream container
	int			nr;			// Board number
	dw_ssi_t	*port;		// The SPI port I/O memory mapping
	uint32_t	cemask;		// The active CE mapping for this board
	uint32_t	clkdivw;	// SPI write clock divider value
	uint32_t	clkdivr;	// SPI read clock divider value
	buffer_t	wbuf;		// Queued writes buffer
	buffer_t	rbuf;		// Queued reads buffer
	buffer_t	rref;		// Queued read buffer references
	int			spidevid;	// The SPI device id [01]
	int			spiceid;	// The SPI CE id [012]
} hm2_rp5spi_t;

// Originals of the io_bank0.gpio[X].ctrl and pads_bank0.gpio registers so they
// can be restored later on exit.
typedef struct __spisave_t {
	uint32_t	bank_sclk;
	uint32_t	bank_mosi;
	uint32_t	bank_miso;
	uint32_t	bank_ce_0;
	uint32_t	bank_ce_1;
	uint32_t	bank_ce_2;
	uint32_t	pads_sclk;
	uint32_t	pads_mosi;
	uint32_t	pads_miso;
	uint32_t	pads_ce_0;
	uint32_t	pads_ce_1;
	uint32_t	pads_ce_2;
} spisave_t;

static spisave_t spi0save;	// Settings before our setup
static spisave_t spi1save;
static int has_spi_module;	// Set to non-zero when the kernel modules dw_spi and dw_spi_mmio are loaded

static void *peripheralmem = MAP_FAILED;	// mmap'ed peripheral memory
static size_t peripheralsize;		// Size of the mmap'ed block
static rp1_rio_t *rio0;				// GPIO pin access structure in mmap'ed address space
static rp1_io_bank0_t *iobank0;		// GPIO pin config structure in mmap'ed address space
static rp1_pads_bank0_t *padsbank0;	// GPIO pin pads structure in mmap'ed address space
static dw_ssi_t *spi0;				// SPI0 peripheral structure in mmap'ed address space
static dw_ssi_t *spi1;				// SPI1 peripheral structure in mmap'ed address space

static hm2_rp5spi_t boards[RPSPI_MAX_BOARDS];	// Connected boards
static int comp_id;				// Upstream assigned component ID

// The value of the cookie when read from the board. An actual cookie read
// consists of four words. The fourth value is the idrom address, which may
// vary per board.
static const uint32_t iocookie[3] = {
	HM2_IOCOOKIE,
	// The following words spell HOSTMOT2
	0x54534f48,	// TSOH
	0x32544f4d	// 2TOM
};

/*
 * Configuration parameters
 */
static char *config[RPSPI_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, RPSPI_MAX_BOARDS, "config string for the AnyIO boards (see hostmot2(9) manpage)")

static int spiclk_rate[RPSPI_MAX_BOARDS] = {SCLK_FREQ_DEF, -1, -1, -1, -1};
static int spiclk_rate_rd[RPSPI_MAX_BOARDS] = {-1, -1, -1, -1, -1};
RTAPI_MP_ARRAY_INT(spiclk_rate, RPSPI_MAX_BOARDS, "SPI clock rates in kHz (default " PPSTR(SCLK_FREQ_DEF) " kHz, slowest " PPSTR(SCLK_FREQ_MIN) " kHz)")
RTAPI_MP_ARRAY_INT(spiclk_rate_rd, RPSPI_MAX_BOARDS, "SPI clock rates for reading in kHz (default same as spiclk_rate)")

/*
 * Enable/disable pullup/pulldown on the SPI pins
 */
#define SPI_PULL_OFF	0
#define SPI_PULL_DOWN	1
#define SPI_PULL_UP		2
#define SPI_PULL_MISO_DEF	SPI_PULL_DOWN
#define SPI_PULL_MOSI_DEF	SPI_PULL_OFF
#define SPI_PULL_SCLK_DEF	SPI_PULL_OFF
#define SPI_PULL_CE_X_DEF	SPI_PULL_UP
static int spi_pull_miso[RPSPI_MAX_SPI] = { SPI_PULL_MISO_DEF, SPI_PULL_MISO_DEF };
static int spi_pull_mosi[RPSPI_MAX_SPI] = { SPI_PULL_MOSI_DEF, SPI_PULL_MOSI_DEF };
static int spi_pull_sclk[RPSPI_MAX_SPI] = { SPI_PULL_SCLK_DEF, SPI_PULL_SCLK_DEF };
RTAPI_MP_ARRAY_INT(spi_pull_miso, RPSPI_MAX_SPI, "Enable/disable pull-{up,down} on SPIx MISO (default pulldown, 0=off, 1=pulldown, 2=pullup)")
RTAPI_MP_ARRAY_INT(spi_pull_mosi, RPSPI_MAX_SPI, "Enable/disable pull-{up,down} on SPIx MOSI (default off, 0=off, 1=pulldown, 2=pullup)")
RTAPI_MP_ARRAY_INT(spi_pull_sclk, RPSPI_MAX_SPI, "Enable/disable pull-{up,down} on SPIx SCLK (default off, 0=off, 1=pulldown, 2=pullup)")

/*
 * Select which SPI channel(s) to probe. There are many SPI interfaces exposed
 * on the 40-pin I/O header. We only use the traditional ones, SPI0 and SPI1.
 * SPI0 has four chip selects and SPI1 has three chip selects. The extra chip
 * selects for SPI0 (CE2 and CE3) are not used because they collide with the
 * I2C port on the header.
 *
 * GPIO pin setup:
 *      | MOSI | MISO | SCLK | CE0 | CE1 | CE2 | CE3
 * -----+------+------+------+-----+-----+-----+-----
 * SPI0 |  10  |   9  |  11  |   8 |   7 | (2) | (3)
 * SPI1 |  20  |  19  |  21  |  18 |  17 |  16 |
 *
 * Boards will be numbered in the order found. The probe scan is ordered in the
 * following way:
 * - SPI0 - CE0
 * - SPI0 - CE1
 * - SPI1 - CE0
 * - SPI1 - CE1
 * - SPI1 - CE2
 *
 * There are other possible SPI ports and CE combinations. However, most will
 * collide in one or another way with older RPi assignments and uses. Two
 * accessible ports should be more than enough for all practical uses.
 */
#define SPI0_PROBE_CE0	(1 << 0)
#define SPI0_PROBE_CE1	(1 << 1)
#define SPI0_PROBE_MASK	(SPI0_PROBE_CE0 | SPI0_PROBE_CE1)
#define SPI1_PROBE_CE0	(1 << 2)
#define SPI1_PROBE_CE1	(1 << 3)
#define SPI1_PROBE_CE2	(1 << 4)
#define SPI1_PROBE_MASK	(SPI1_PROBE_CE0 | SPI1_PROBE_CE1 | SPI1_PROBE_CE2)
static int spi_probe = SPI0_PROBE_CE0;
RTAPI_MP_INT(spi_probe, "Bit-field to select which SPI/CE combinations to probe (default 1 (SPI0/CE0))")

/*
 * Normally, all requests are queued if requested by upstream and sent in one
 * bulk transfer. This reduces overhead significantly. Disabling the queue make
 * each transfer visible and more easily debugable.
 */
static int spi_noqueue = 0;
RTAPI_MP_INT(spi_noqueue, "Disable queued SPI requests, use for debugging only (default 0 (off))")

/*
 * Set the message level for debugging purpose. This has the (side-)effect that
 * all modules within this process will start spitting out messages at the
 * requested level.
 * The upstream message level is not touched if spi_debug == -1.
 */
static int spi_debug = -1;
RTAPI_MP_INT(spi_debug, "Set message level for debugging purpose [0...5] where 0=none and 5=all (default: -1; upstream defined)")

/*********************************************************************/
#ifdef RPSPI_DEBUG_PIN
HWREGACCESS_ALWAYS_INLINE static inline void gpio_set(int pin)
{
	if(pin >= 0 && pin < 28)
		reg_wr_raw(&rio0->set.out, 1 << pin);
}

HWREGACCESS_ALWAYS_INLINE static inline void gpio_clr(int pin)
{
	if(pin >= 0 && pin < 28)
		reg_wr_raw(&rio0->clr.out, 1 << pin);
}
#endif

/*********************************************************************/
#define CMD_7I90_READ		(0x0a << 12)
#define CMD_7I90_WRITE		(0x0b << 12)
#define CMD_7I90_ADDRINC	(1 << 11)
// aib (address increment bit)
static inline uint32_t mk_read_cmd(uint32_t addr, uint32_t msglen, bool aib)
{
	return (addr << 16) | CMD_7I90_READ | (aib ? CMD_7I90_ADDRINC : 0) | (msglen << 4);
}

static inline uint32_t mk_write_cmd(uint32_t addr, uint32_t msglen, bool aib)
{
	return (addr << 16) | CMD_7I90_WRITE | (aib ? CMD_7I90_ADDRINC : 0) | (msglen << 4);
}

/*
 * Buffer managenemt for queued transfers.
 */
static int buffer_check_room(buffer_t *b, size_t n, size_t elmsize)
{
	if(!b->ptr || !b->na) {
		b->na = 64;	// Default to this many elements
		b->n = 0;
		b->ptr = rtapi_kmalloc(elmsize * b->na, RTAPI_GPF_KERNEL);
		return b->ptr == NULL;
	}

	if(b->n + n > b->na) {
		do {
			b->na *= 2;	// Double storage capacity
		} while(b->n + n > b->na);	// Until we have enough room
		void *p = rtapi_krealloc(b->ptr, elmsize * b->na, RTAPI_GPF_KERNEL);
		if(!p)
			return 1;
		b->ptr = p;
	}
	return 0;
}

static void buffer_free(buffer_t *b)
{
	if(b->ptr) {
		rtapi_kfree(b->ptr);
		b->ptr = NULL;
		b->n = b->na = 0;
	}
}

/*********************************************************************/
/*
 * Calculate the clock divider for any spi port
 */
static inline int32_t spi_clkdiv_calc(uint32_t rate)
{
	uint32_t clkdiv = RP1_SPI_CLK / rate;
	// The documentation states: bit 0 is always zero, therefore, only even
	// divider values supported. Divider value 0 disables the SCLK output.
	if(clkdiv > 65534)
		clkdiv = 65534;			// Slowest possible
	else
		clkdiv += clkdiv & 1;	// Must be multiple of 2 (round to lower frequency)
	if(!clkdiv)
		clkdiv = 2;				// Do not disable, set it to absolute maximum frequency
	return clkdiv;
}

/*
 * Reset the SPI peripheral to inactive state and flushed
 */
static inline void spi_reset(dw_ssi_t *port)
{
	uint32_t dummy __attribute__((unused));
	reg_wr_raw(&port->ssienr, 0);	// Disable chip, will also clear fifo
	reg_wr_raw(&port->ser, 0);		// Clear all chip selects
	dummy = reg_rd_raw(&port->icr);	// Clear all interrupts
	// The chip will be enabled when a transfer is started
	// reg_wr(port->ssienr, DW_SSI_SSIENR_SSI_EN);	// Enable chip
}

/*
 * Transfer a buffer of words to the SPI port and fill the same buffer with the
 * data coming from the SPI port.
 */
static int spi_transfer(hm2_rp5spi_t *hm2, uint32_t *wptr, size_t txlen, int rw)
{
	dw_ssi_t *port = hm2->port;
	size_t rxlen = txlen;			// words to receive
	uint32_t *rptr = wptr;			// read into write buffer
	int fifo = RP1_SPI_FIFO_LEN;	// fifo level counter before starting transfer
	int rv = 1;						// function return value

#ifdef RPSPI_DEBUG_PIN
	gpio_clr(RPSPI_DEBUG_PIN);
#endif

	// Setup transfer
	// 32-bit, transmit/receive transfers, SPI mode 0 (CPHA=0, CPOL=0)
	reg_wr(&port->ctrlr0, DW_SSI_CTRLR0_DFS_32(32-1) | DW_SSI_CTRLR0_TMOD(DW_SSI_CTRLR0_TMOD_TXRX));
	reg_wr_raw(&port->baudr, rw ? hm2->clkdivr : hm2->clkdivw);
	reg_wr_raw(&port->ser, hm2->cemask);

	reg_wr_raw(&port->ssienr, DW_SSI_SSIENR_SSI_EN);	// Enable port

	// Stuff the fifo until full or no more data words to write
	while(txlen > 0 && fifo > 0) {
		reg_wr_raw(&port->dr0, *wptr);	// Write data
		wptr++;
		txlen--;
		fifo--;
	}

	// We don't need to add a memory barrier. The next loop runs about rxlen
	// and the code will stall on the register reads. There is no read/write
	// overlap that may be problematic.

	while(rxlen > 0) {
		// Get the rx fifo level and read as many as available
		uint32_t tff = fifo = reg_rd_raw(&port->rxflr);
		// Already get the int status register; this read will pipeline
		uint32_t risr = reg_rd_raw(&port->risr);
		while(rxlen > 0 && fifo > 0) {
			*rptr = reg_rd_raw(&port->dr0);
			rptr++;
			rxlen--;
			fifo--;
		}

		// If we still have queued data, blindly write to stuff the tx-fifo.
		// For each received word we have one less entry in the transmit fifo.
		// It has to be. Therefore, we can use the receive level as a proxy for
		// how many words can be written
		while(txlen > 0 && tff > 0) {
			reg_wr_raw(&port->dr0, *wptr);
			wptr++;
			txlen--;
			tff--;
		}

		// Check for receive errors
		if(risr & (DW_SSI_RISR_RXOIR)) {
			// The receive fifo overflowed. A bad sign...
			// Abort the transfer
			rv = -EIO;
			LL_ERR("SPI%d/CE%d Receive FIFO overflow during transfer\n", hm2->spidevid, hm2->spiceid);
			goto abort_rx;
		}
	}

	// We should no longer have to wait. Each word transmitted has been
	// received (rxlen == 0). Therefore, the transfer must be complete and
	// done. No (busy) wait required.

abort_rx:
	spi_reset(port);

#ifdef RPSPI_DEBUG_PIN
	gpio_set(RPSPI_DEBUG_PIN);
#endif
	return rv;
}

/*
 * HM2 interface: Write buffer to SPI
 *
 * Writes the buffer to SPI, prepended with a command word.
 */
static int hm2_rp5spi_write(hm2_lowlevel_io_t *llio, rtapi_u32 addr, const void *buffer, int size)
{
	hm2_rp5spi_t *hm2 = (hm2_rp5spi_t *)llio;
	int txlen = size / sizeof(uint32_t);	// uint32_t words to transmit
	uint32_t txbuf[RPSPI_MAX_MSG];			// local buffer for entire transfer

	if(size == 0)
		return 0;
	if((size % sizeof(uint32_t)) || txlen + 1 > RPSPI_MAX_MSG)
		return -EINVAL;

	txbuf[0] = mk_write_cmd(addr, txlen, true);	// Setup write command
	memcpy(&txbuf[1], buffer, size);			// Setup write data
	return spi_transfer(hm2, txbuf, txlen + 1, 0);	// Do transfer
}

/*
 * HM2 interface: Read buffer from SPI
 *
 * Reads from SPI after sending the appropriate command. Sends one word with
 * the command followed by writing zeros while reading.
 */
static int hm2_rp5spi_read(hm2_lowlevel_io_t *llio, rtapi_u32 addr, void *buffer, int size)
{
	hm2_rp5spi_t *hm2 = (hm2_rp5spi_t *)llio;
	int rxlen = size / sizeof(uint32_t);	// uint32_t words to receive
	uint32_t rxbuf[RPSPI_MAX_MSG];			// local buffer for entire transfer
	int rv;

	if(size == 0)
		return 0;
	if((size % sizeof(uint32_t)) || rxlen + 1 > RPSPI_MAX_MSG)
		return -EINVAL;

	memset(rxbuf, 0, sizeof(rxbuf));			// Clear buffer; reads stuff zero writes
	rxbuf[0] = mk_read_cmd(addr, rxlen, true);	// Setup read command
	rv = spi_transfer(hm2, rxbuf, rxlen + 1, 1);	// Do transfer
	memcpy(buffer, &rxbuf[1], size);			// Copy received data (even with errors...)
	return rv;
}

/*
 * HM2 interface: Queue read
 * Collects the read address and buffer for bulk-read later on.
 */
static int hm2_rp5spi_queue_read(hm2_lowlevel_io_t *llio, rtapi_u32 addr, void *buffer, int size)
{
	hm2_rp5spi_t *hm2 = (hm2_rp5spi_t *)llio;
	int rxlen = size / sizeof(uint32_t);

	if(size == 0)
		return 0;
	if((size % sizeof(uint32_t)) || rxlen + 1 > RPSPI_MAX_MSG)
		return -EINVAL;

	if(buffer_check_room(&hm2->rbuf, rxlen + 1, sizeof(uint32_t))) {
		LL_ERR("Failed to allocate read buffer memory\n");
		return -ENOMEM;
	}

	if(buffer_check_room(&hm2->rref, 1, sizeof(rxref_t))) {
		LL_ERR("Failed to allocate read queue reference memory\n");
		return -ENOMEM;
	}

	// Add a reference control structure to remember where the data will be put
	// after the read is executed
	rxref_t *ref = &((rxref_t *)hm2->rref.ptr)[hm2->rref.n];
	ref->ptr = buffer;
	ref->size = size;
	ref->idx = hm2->rbuf.n + 1;	// offset 0 is command, 1 is data
	hm2->rref.n += 1;

	uint32_t *rbptr = (uint32_t *)hm2->rbuf.ptr;
	rbptr[hm2->rbuf.n] = mk_read_cmd(addr, rxlen, true);			// The read command
	memset(&rbptr[hm2->rbuf.n + 1], 0, rxlen * sizeof(uint32_t));	// Fill zeros as data
	hm2->rbuf.n += rxlen + 1;

	return 1;
}

/*
 * HM2 interface: Send queued reads
 * Performs a SPI transfer of all collected read requests in one burst and
 * copies back the data received in the individual buffers.
 */
static int hm2_rp5spi_send_queued_reads(hm2_lowlevel_io_t *llio)
{
	uint32_t cookie[3] = {0, 0, 0};
	hm2_rp5spi_t *hm2 = (hm2_rp5spi_t *)llio;
	int rv;

	// Add a cookie read at the end of the queued reads to verify comms
	hm2_rp5spi_queue_read(llio, HM2_ADDR_IOCOOKIE, cookie, sizeof(cookie));

	rv = spi_transfer(hm2, hm2->rbuf.ptr, hm2->rbuf.n, 1);
	if(rv >= 0) {
		// The transfer read the data into the read buffer. Now copy it into
		// the individual read requests' buffers.
		size_t i;
		for(i = 0; i < hm2->rref.n; i++) {
			rxref_t *rxref = &((rxref_t *)hm2->rref.ptr)[i];
			memcpy(rxref->ptr, &((uint32_t *)hm2->rbuf.ptr)[rxref->idx], rxref->size);
		}

		// Check the cookie read. Its an IO error if it does not match
		if(memcmp(cookie, iocookie, sizeof(iocookie)))
			rv = -EIO;
	}
	hm2->rbuf.n = 0;	// Reset the queue buffers
	hm2->rref.n = 0;

	return rv;
}

/*
 * HM2 interface: Receive queued reads
 * This is a no-op in SPI. The data was already received when the transfer was
 * performed in hm2_rp5spi_send_queued_reads() above. The data was copied to
 * the requester(s) immediately after the transfer.
 */
static int hm2_rp5spi_receive_queued_reads(hm2_lowlevel_io_t *llio)
{
	(void)llio;
	return 1;
}

/*
 * HM2 interface: Queue write
 * Collects the write address and data for bulk-write later on.
 */
static int hm2_rp5spi_queue_write(hm2_lowlevel_io_t *llio, rtapi_u32 addr, const void *buffer, int size)
{
	hm2_rp5spi_t *hm2 = (hm2_rp5spi_t *)llio;
	int txlen = size / sizeof(uint32_t);

	if(size == 0)
		return 0;
	if((size % sizeof(uint32_t)) || txlen + 1 > RPSPI_MAX_MSG)
		return -EINVAL;

	if(buffer_check_room(&hm2->wbuf, txlen + 1, sizeof(uint32_t))) {
		LL_ERR("Failed to allocate write buffer memory\n");
		return -ENOMEM;
	}

	uint32_t *wbptr = (uint32_t *)hm2->wbuf.ptr;
	wbptr[hm2->wbuf.n] = mk_write_cmd(addr, txlen, true);				// The write command
	memcpy(&wbptr[hm2->wbuf.n + 1], buffer, txlen * sizeof(uint32_t));	// The data
	hm2->wbuf.n += txlen + 1;
	return 1;
}

/*
 * HM2 interface: Send queued writes
 * Performs a SPI transfer of all collected write requests in one burst.
 */
static int hm2_rp5spi_send_queued_writes(hm2_lowlevel_io_t *llio)
{
	hm2_rp5spi_t *hm2 = (hm2_rp5spi_t *)llio;
	int rv = spi_transfer(hm2, hm2->wbuf.ptr, hm2->wbuf.n, 0);
	hm2->wbuf.n = 0;	// Reset the queue buffer
	return rv;
}

/*********************************************************************/

// Counting ones in a word is the also known as the Hamming weight or
// population count. The "best" algorithm is SWAR. The nibble-lookup seems to
// be a good alternative. Anyway, this routine is only called on a cookie
// error and has no real speed criteria.
// Newer GCC has a __builtin_popcount() to get the right number, but that may
// not be available on the current compiler.
static inline unsigned count_ones(uint32_t val)
{
	// Number of ones in a nibble
	static const unsigned nibble_table[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4 };
	unsigned i;
	for(i = 0; val; val >>= 4) {
		i += nibble_table[val & 0x0f];
	}
	return i;
}

static int32_t check_cookie(hm2_rp5spi_t *board)
{
	uint32_t cookie[4] = {0, 0, 0, 0};
	uint32_t ca;
	uint32_t co;

	// We read four (4) 32-bit words. The first three are the cookie and
	// the fourth entry is the idrom address offset. The offset is used in
	// the call to get the idrom if we successfully match a cookie.
	if(!board->llio.read(&board->llio, HM2_ADDR_IOCOOKIE, cookie, sizeof(cookie)))
		return -ENODEV;

	if(!memcmp(cookie, iocookie, sizeof(iocookie))) {
		LL_INFO("Cookie read: %08x %08x %08x, idrom@%08x\n", cookie[0], cookie[1], cookie[2], cookie[3]);
		return (int32_t)cookie[3];	// The cookie got read correctly
	}

	LL_ERR("SPI%d/CE%d Invalid cookie, read: %08x %08x %08x,"
			" expected: %08x %08x %08x\n",
			board->spidevid, board->spiceid,
			cookie[0], cookie[1], cookie[2],
			iocookie[0], iocookie[1], iocookie[2]);

	// Lets see if we can tell why it went wrong
	ca = cookie[0] & cookie[1] & cookie[2];	// All ones -> ca == ones
	co = cookie[0] | cookie[1] | cookie[2];	// All zero -> co == zero

	if((!co && spi_pull_miso[board->spidevid] == SPI_PULL_DOWN) || (ca == 0xffffffff && spi_pull_miso[board->spidevid] == SPI_PULL_UP)) {
		LL_ERR("SPI%d/CE%d No drive seen on MISO line (kept at pull-%s level)."
			" No board connected or bad connection?\n",
			board->spidevid, board->spiceid, spi_pull_miso[board->spidevid] == SPI_PULL_DOWN ? "down" : "up");
	} else if(!co || ca == 0xffffffff) {
		LL_ERR("SPI%d/CE%d MISO line stuck at %s level."
			" Maybe bad connection, a short-circuit or no board attached?\n",
			board->spidevid, board->spiceid, !co ? "low" : "high");
	} else {
		// If you read too fast, then the bit-setup times are not satisfied and
		// the input may be shifted by one bit-clock. Every 32nd bit may not
		// arrive soon enough and the previous data will be clocked in.
		//
		// We can detect this eventuality by checking the cookie against a
		// bit-shifted version and mask the bits that may have fallen off the
		// cliff. If the cookie matches (all zeroes in the XOR result), then it
		// is most likely that the read-clock frequency is too high.
		//
		unsigned ones;
		unsigned i;
		for(ones = i = 0; i < 3; i++) {
			ones += count_ones((iocookie[i] ^ (cookie[i] << 1)) & ~0x00000001);
		}
		if(!ones) {
			// No ones in the XOR result -> the cookie is probably bit-shifted
			LL_ERR("SPI%d/CE%d MISO input is bit-shifted by one bit."
				" SPI read clock frequency probably too high.\n",
				board->spidevid, board->spiceid);
		} else {
			// More bits are wrong, erratic behaviour
			LL_ERR("SPI%d/CE%d MISO input does not match any expected bit-pattern (>= %u bit difference)."
				" Maybe SPI read clock frequency too high or noise on the input?\n",
				board->spidevid, board->spiceid, ones);
		}
	}
	return -ENODEV;
}

/*************************************************/
static int probe_board(hm2_rp5spi_t *board)
{
	int32_t ret;
	hm2_idrom_t idrom;
	const char *base;

	if((ret = check_cookie(board)) < 0)
		return ret;

	LL_INFO("SPI%d/CE%d Valid cookie matched, idrom@%04x\n", board->spidevid, board->spiceid, ret);

	// Read the board identification.
	// The IDROM address offset is returned in the cookie check and the
	// board_name offset is added (see hm2_idrom_t in hostmot2.h)
	if(!board->llio.read(&board->llio, (uint32_t)ret, &idrom, sizeof(hm2_idrom_t))) {
		LL_ERR("SPI%d/CE%d Board idrom read failed\n", board->spidevid, board->spiceid);
		return -EIO;	// Cookie could be read, so this is a comms error
	}

	// Detect board name and fill in informational values
	if(!(base = set_llio_info_spi(&board->llio, &idrom)))
		return -ENOENT;

	LL_INFO("SPI%d/CE%d Base: %s.%d\n", board->spidevid, board->spiceid, base, board->nr);
	rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "%s.%d", base, board->nr);
	board->llio.comp_id = comp_id;
	board->llio.private = board;	// Self reference

	return 0;
}

/*************************************************/
static int peripheral_map(uintptr_t membase, size_t memsize)
{
	int fd;
	int err;

	peripheralsize = memsize;

	if((fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		LL_ERR("Can't open /dev/mem\n");
		return -errno;
	}

	/* mmap PCIe translation address of the RP1 peripherals */
	peripheralmem = mmap(NULL, peripheralsize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off_t)membase);
	err = errno;
	close(fd);
	if(peripheralmem == MAP_FAILED) {
		LL_ERR("Can't map peripherals: %s\n", strerror(err));
		if(err == EPERM) {
			LL_ERR("Try adding 'iomem=relaxed' to your kernel command-line.\n");
		}
		return -err;
	}

	// Calculate the addresses of the individual peripherals
	spi0 = (dw_ssi_t *)((void *)((uintptr_t)peripheralmem + RP1_SPI0_OFFSET));
	spi1 = (dw_ssi_t *)((void *)((uintptr_t)peripheralmem + RP1_SPI1_OFFSET));
	iobank0 = (rp1_io_bank0_t *)((void *)((uintptr_t)peripheralmem + RP1_IO_BANK0_OFFSET));
	padsbank0 = (rp1_pads_bank0_t *)((void *)((uintptr_t)peripheralmem + RP1_PADS_BANK0_OFFSET));
	rio0 = (rp1_rio_t *)((void *)((uintptr_t)peripheralmem + RP1_SYS_RIO0_OFFSET));

	LL_INFO("Mapped peripherals from 0x%p (size 0x%08zx) to rio:0x%p, spi0:0x%p, spi1:0x%p\n",
			(void *)membase, peripheralsize, rio0, spi0, spi1);

	return 0;
}

/*
 * Convert the pull-up/down enum into a register mask. Perform boundary checks
 * on the value and set it to a valid value if it was outside valid range.
 */
static uint32_t pud_val_to_mask(int *pud)
{
	switch(*pud) {
	default:
		*pud = SPI_PULL_OFF;
		/* Fallthrough */
	case SPI_PULL_OFF:	return 0;
	case SPI_PULL_DOWN:	return RP1_PADS_PDE;
	case SPI_PULL_UP:	return RP1_PADS_PUE;
	}
}
static void peripheral_setup(void)
{
	//
	// We do not touch the voltage_select register in pads_bank0. We simply
	// hope that the voltage_select register is at 3.3V. That should be a safe
	// assumption, considering the normal setup. Otherwise, the hardware may
	// already be fried at boot-time with high input voltages on input pins.
	// Raspberry Pi RP1 Peripherals bottom of section 3.1.3:
	//   "Using VDDIO voltages greater than 1.8V, with the input thresholds set
	//   for 1.8V may result in damage to the chip."
	// We are pretty sure that the stock board uses 3.3V VDDIO because of
	// compatibility with existing hardware. Therefore, fiddling with the
	// voltage_select register may be fatal to the hardware.
	//

// The required drive level in pads_bank0 registers (8mA drive and high slewrate)
#define DRIVE_LEVEL	(RP1_PADS_DRIVE(RP1_PADS_DRIVE_8) | RP1_PADS_SLEWFAST)

	// Setup SPI pins for SPI0 and save the original setup
	if(spi_probe & SPI0_PROBE_MASK) {
		uint32_t pud_miso = pud_val_to_mask(&spi_pull_miso[0]);
		uint32_t pud_mosi = pud_val_to_mask(&spi_pull_mosi[0]);
		uint32_t pud_sclk = pud_val_to_mask(&spi_pull_sclk[0]);

		spi0save.bank_sclk = reg_rd(&iobank0->gpio[SPI0_PIN_SCLK].ctrl);
		spi0save.bank_mosi = reg_rd(&iobank0->gpio[SPI0_PIN_MOSI].ctrl);
		spi0save.bank_miso = reg_rd(&iobank0->gpio[SPI0_PIN_MISO].ctrl);
		spi0save.bank_ce_0 = reg_rd(&iobank0->gpio[SPI0_PIN_CE_0].ctrl);
		spi0save.bank_ce_1 = reg_rd(&iobank0->gpio[SPI0_PIN_CE_1].ctrl);
		spi0save.pads_sclk = reg_rd(&padsbank0->gpio[SPI0_PIN_SCLK]);
		spi0save.pads_mosi = reg_rd(&padsbank0->gpio[SPI0_PIN_MOSI]);
		spi0save.pads_miso = reg_rd(&padsbank0->gpio[SPI0_PIN_MISO]);
		spi0save.pads_ce_0 = reg_rd(&padsbank0->gpio[SPI0_PIN_CE_0]);
		spi0save.pads_ce_1 = reg_rd(&padsbank0->gpio[SPI0_PIN_CE_1]);

		reg_wr(&iobank0->gpio[SPI0_PIN_SCLK].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI0_PIN_SCLK], RP1_PADS_IE | DRIVE_LEVEL | pud_sclk);
		reg_wr(&iobank0->gpio[SPI0_PIN_MOSI].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI0_PIN_MOSI], RP1_PADS_IE | DRIVE_LEVEL | pud_mosi);
		reg_wr(&iobank0->gpio[ SPI0_PIN_MISO].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI0_PIN_MISO], RP1_PADS_IE | DRIVE_LEVEL | pud_miso);
		if(spi_probe & SPI0_PROBE_CE0) {
			reg_wr(&iobank0->gpio[SPI0_PIN_CE_0].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI0_PIN_CE_0], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		if(spi_probe & SPI0_PROBE_CE1) {
			reg_wr(&iobank0->gpio[SPI0_PIN_CE_1].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI0_PIN_CE_1], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		spi_reset(spi0);
	}

	// Setup SPI pins for SPI1 and save the original setup
	if(spi_probe & SPI1_PROBE_MASK) {
		uint32_t pud_miso = pud_val_to_mask(&spi_pull_miso[1]);
		uint32_t pud_mosi = pud_val_to_mask(&spi_pull_mosi[1]);
		uint32_t pud_sclk = pud_val_to_mask(&spi_pull_sclk[1]);

		spi1save.bank_sclk = reg_rd(&iobank0->gpio[SPI1_PIN_SCLK].ctrl);
		spi1save.bank_mosi = reg_rd(&iobank0->gpio[SPI1_PIN_MOSI].ctrl);
		spi1save.bank_miso = reg_rd(&iobank0->gpio[SPI1_PIN_MISO].ctrl);
		spi1save.bank_ce_0 = reg_rd(&iobank0->gpio[SPI1_PIN_CE_0].ctrl);
		spi1save.bank_ce_1 = reg_rd(&iobank0->gpio[SPI1_PIN_CE_1].ctrl);
		spi1save.bank_ce_2 = reg_rd(&iobank0->gpio[SPI1_PIN_CE_2].ctrl);
		spi1save.pads_sclk = reg_rd(&padsbank0->gpio[SPI1_PIN_SCLK]);
		spi1save.pads_mosi = reg_rd(&padsbank0->gpio[SPI1_PIN_MOSI]);
		spi1save.pads_miso = reg_rd(&padsbank0->gpio[SPI1_PIN_MISO]);
		spi1save.pads_ce_0 = reg_rd(&padsbank0->gpio[SPI1_PIN_CE_0]);
		spi1save.pads_ce_1 = reg_rd(&padsbank0->gpio[SPI1_PIN_CE_1]);
		spi1save.pads_ce_2 = reg_rd(&padsbank0->gpio[SPI1_PIN_CE_2]);

		reg_wr(&iobank0->gpio[SPI1_PIN_SCLK].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI1_PIN_SCLK], RP1_PADS_IE | DRIVE_LEVEL | pud_sclk);
		reg_wr(&iobank0->gpio[SPI1_PIN_MOSI].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI1_PIN_MOSI], RP1_PADS_IE | DRIVE_LEVEL | pud_mosi);
		reg_wr(&iobank0->gpio[SPI1_PIN_MISO].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI1_PIN_MISO], RP1_PADS_IE | DRIVE_LEVEL | pud_miso);
		if(spi_probe & SPI1_PROBE_CE0) {
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_0].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_0], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		if(spi_probe & SPI1_PROBE_CE1) {
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_1].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_1], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		if(spi_probe & SPI1_PROBE_CE2) {
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_2].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_2], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		spi_reset(spi1);
	}

#ifdef RPSPI_DEBUG_PIN
	reg_wr(&rio0->set.out, 1 << RPSPI_DEBUG_PIN);	// Set pin high
	reg_wr(&rio0->set.oe,  1 << RPSPI_DEBUG_PIN);	// Set pin to output
	reg_wr(&padsbank0->gpio[RPSPI_DEBUG_PIN], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
	reg_wr(&iobank0->gpio[RPSPI_DEBUG_PIN].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_SYS_RIO));
#endif
}

/*************************************************/
static void peripheral_restore(void)
{
	// Restore all SPI pins to inputs and enable pull-up (no dangling inputs)
	if(spi_probe & SPI0_PROBE_MASK) {
		spi_reset(spi0);	// Disable SPI0 peripheral

		// Restore SPI0 pins
		reg_wr(&padsbank0->gpio[SPI0_PIN_SCLK], spi0save.pads_sclk);
		reg_wr(&padsbank0->gpio[SPI0_PIN_MOSI], spi0save.pads_mosi);
		reg_wr(&padsbank0->gpio[SPI0_PIN_MISO], spi0save.pads_miso);
		reg_wr(&iobank0->gpio[SPI0_PIN_SCLK].ctrl, spi0save.bank_sclk);
		reg_wr(&iobank0->gpio[SPI0_PIN_MOSI].ctrl, spi0save.bank_mosi);
		reg_wr(&iobank0->gpio[SPI0_PIN_MISO].ctrl, spi0save.bank_miso);
		if(spi_probe & SPI0_PROBE_CE0) {
			reg_wr(&padsbank0->gpio[SPI0_PIN_CE_0], spi0save.pads_ce_0);
			reg_wr(&iobank0->gpio[SPI0_PIN_CE_0].ctrl, spi0save.bank_ce_0);
		}
		if(spi_probe & SPI0_PROBE_CE1) {
			reg_wr(&padsbank0->gpio[SPI0_PIN_CE_1], spi0save.pads_ce_1);
			reg_wr(&iobank0->gpio[SPI0_PIN_CE_1].ctrl, spi0save.bank_ce_1);
		}
	}

	if(spi_probe & SPI1_PROBE_MASK) {
		spi_reset(spi1);	// Disable SPI1 peripheral

		// Restore SPI1 pins
		reg_wr(&padsbank0->gpio[SPI1_PIN_SCLK], spi1save.pads_sclk);
		reg_wr(&padsbank0->gpio[SPI1_PIN_MOSI], spi1save.pads_mosi);
		reg_wr(&padsbank0->gpio[SPI1_PIN_MISO], spi1save.pads_miso);
		reg_wr(&iobank0->gpio[SPI1_PIN_SCLK].ctrl, spi1save.bank_sclk);
		reg_wr(&iobank0->gpio[SPI1_PIN_MOSI].ctrl, spi1save.bank_mosi);
		reg_wr(&iobank0->gpio[SPI1_PIN_MISO].ctrl, spi1save.bank_miso);
		if(spi_probe & SPI1_PROBE_CE0) {
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_0], spi1save.pads_ce_0);
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_0].ctrl, spi1save.bank_ce_0);
		}
		if(spi_probe & SPI1_PROBE_CE1) {
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_1], spi1save.pads_ce_1);
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_1].ctrl, spi1save.bank_ce_1);
		}
		if(spi_probe & SPI1_PROBE_CE2) {
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_2], spi1save.pads_ce_2);
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_2].ctrl, spi1save.bank_ce_2);
		}
	}

#ifdef RPSPI_DEBUG_PIN
	// This leaves the pin in SYS_RIO mode so we may trigger the scope on the
	// next run
	reg_wr(&rio0->clr.oe,  1 << RPSPI_DEBUG_PIN);	// Set pin to input
#endif
}

/*************************************************/

/* Read at most bufsize-1 bytes from fname */
static ssize_t read_file(const char *fname, void *buffer, size_t bufsize)
{
	int fd;
	ssize_t len;

	memset(buffer, 0, bufsize);

	if(!(fd = open(fname, O_RDONLY))) {
		LL_ERR("Cannot open '%s' for read (errno=%d: %s)\n", fname, errno, strerror(errno));
		return -1;
	}

	while(1) {
		len = read(fd, buffer, bufsize - 1);
		if(len == 0) {
			LL_ERR("Nothing read from '%s', file contains no data\n", fname);
		} else if(len < 0) {
			if(errno == EINTR)
				continue;	// Interrupted syscall, retry read
			LL_ERR("Error reading from '%s' (errno=%d: %s)\n", fname, errno, strerror(errno));
		}
		break;
	}
	close(fd);
	return len;
}

/*************************************************/

static int hm2_rp5spi_setup(void)
{
	int i, j;
	int retval = -1;
	char buf[256];
	ssize_t buflen;
	char *cptr;

	// Set process-level message level if requested
	if(spi_debug >= RTAPI_MSG_NONE && spi_debug <= RTAPI_MSG_ALL)
		rtapi_set_msg_level(spi_debug);

	buflen = read_file("/proc/device-tree/compatible", buf, sizeof(buf));
	if(buflen <= 0) {
		LL_ERR("Failed to read platform identity.\n");
		return -1;
	}

	// Check each string in the stringlist and test for our compatibility
	// string. Don't go beyond the buffer's size.
	for(cptr = buf; cptr;) {
		// The "Raspberry Pi 5 Model B", "Raspberry Pi Compute Module 5"
		// and "Raspberry Pi Compute Module 5 Lite" are the boards
		// supported by this driver.
		if(!strcmp(cptr, DTC_RPI_MODEL_5B) || !strcmp(cptr, DTC_RPI_MODEL_5CM)) {
			break;	// Found our board
		}
		j = strlen(cptr);
		if((cptr - buf) + j + 1 < buflen)
			cptr += j + 1;
		else
			cptr = NULL;
	}

	if(!cptr) {
		LL_ERR("Unsupported platform: '%s'\n", buf);
		return -1;
	}

	LL_INFO("Platform identity: %s\n", buf);
	// Human readable platform name
	if(read_file("/proc/device-tree/model", buf, sizeof(buf)) > 0)
		LL_INFO("Platform: %s\n", buf);

	// Check the clock rate settings from the config
	for(i = 0; i < RPSPI_MAX_BOARDS; i++) {
		if(-1 == spiclk_rate[i])	// If not specified
			spiclk_rate[i] = spiclk_rate[0];	// use first

		if(-1 == spiclk_rate_rd[i])	// If not specified
			spiclk_rate_rd[i] = spiclk_rate[i];	// use write rate as read rate

		// Lowest frequency : 200 MHz / 65534 ~ 3052 Hz
		// Highest frequency: 200 MHz /     2 ~ 100000000  Hz
		// Both are rather extreme. Therefore, we align the limits somewhat with
		// the hm2_rpspi driver. We limit the frequency to 4 kHz ... 50 MHz. This
		// is too low for any realtime stuff, but nice for debugging the interface.
		// It is also rather too fast for most hardware interfaces.
		if(spiclk_rate[i] < SCLK_FREQ_MIN || spiclk_rate[i] > SCLK_FREQ_MAX) {
			LL_ERR("SPI clock rate '%d' at index %d too slow/fast. Must be >= %d kHz and <= %d kHz\n",
					spiclk_rate[i], i, SCLK_FREQ_MIN, SCLK_FREQ_MAX);
			return -EINVAL;
		}

		if(spiclk_rate_rd[i] < SCLK_FREQ_MIN || spiclk_rate_rd[i] > SCLK_FREQ_MAX) {
			LL_ERR("SPI clock rate at for reading '%d' at index %d too slow/fast. Must be >= %d kHz and <= %d kHz\n",
					spiclk_rate_rd[i], i, SCLK_FREQ_MIN, SCLK_FREQ_MAX);
			return -EINVAL;
		}
	}

	// The IO address for the RPi5 is at a fixed address. No need to do fancy
	// stuff.
	if((retval = peripheral_map(RP1_PCIE_BAR1_ADDR, RP1_PCIE_BAR1_LEN)) < 0) {
		LL_ERR("Cannot map peripheral memory.\n");
		return retval;
	}

	peripheral_setup();

	memset(boards, 0, sizeof(boards));

	// Follows SPI0/CE0, SPI0/CE1, SPI1/CE0, SPI1/CE1 and SPI1/CE2
	for(j = i = 0; i < RPSPI_MAX_BOARDS; i++) {
		static const int probe_iddev[RPSPI_MAX_BOARDS] = {0, 0, 1, 1, 1};
		static const int probe_idce[RPSPI_MAX_BOARDS]  = {0, 1, 0, 1, 2};
		int iddev = probe_iddev[i];
		int idce = probe_idce[i];

		if(!(spi_probe & (1 << i)))		// Only probe if enabled
			continue;

		boards[j].nr = j;
		boards[j].port     = iddev == 0 ? spi0 : spi1;
		boards[j].cemask   = 1 << idce;
		boards[j].clkdivw  = spi_clkdiv_calc(spiclk_rate[j] * 1000);	// Command line specifies in kHz
		boards[j].clkdivr  = spi_clkdiv_calc(spiclk_rate_rd[j] * 1000);
		boards[j].spidevid = iddev;
		boards[j].spiceid  = idce;

		LL_INFO("SPI%d/CE%d\n", iddev, idce);
		boards[j].llio.read  = hm2_rp5spi_read;
		boards[j].llio.write = hm2_rp5spi_write;
		if(!spi_noqueue) {
			boards[j].llio.queue_read  = hm2_rp5spi_queue_read;
			boards[j].llio.send_queued_reads  = hm2_rp5spi_send_queued_reads;
			boards[j].llio.receive_queued_reads  = hm2_rp5spi_receive_queued_reads;
			boards[j].llio.queue_write  = hm2_rp5spi_queue_write;
			boards[j].llio.send_queued_writes  = hm2_rp5spi_send_queued_writes;
		}
		LL_INFO("SPI%d/CE%d write clock rate calculated: %d Hz (clkdiv=%u)\n", iddev, idce, RP1_SPI_CLK / boards[j].clkdivw, boards[j].clkdivw);
		LL_INFO("SPI%d/CE%d read clock rate calculated: %d Hz (clkdiv=%u)\n",  iddev, idce, RP1_SPI_CLK / boards[j].clkdivr, boards[j].clkdivr);

		if((retval = probe_board(&boards[j])) < 0) {
			return retval;
		}

		if((retval = hm2_register(&boards[j].llio, config[j])) < 0) {
			LL_ERR("hm2_register() failed for SPI%d/CE%d.\n", iddev, idce);
			return retval;
		}

		j++;	// Next board
	}

	return j > 0 ? 0 : -ENODEV;
}

/*************************************************/
static void hm2_rp5spi_cleanup(void)
{
	int i;
	// Cleanup memory allocations
	for(i = 0; i < RPSPI_MAX_BOARDS; i++) {
		buffer_free(&boards[i].wbuf);
		buffer_free(&boards[i].rbuf);
		buffer_free(&boards[i].rref);
	}

	if(peripheralmem != MAP_FAILED) {
		peripheral_restore();
		munmap(peripheralmem, peripheralsize);
	}

	// Restore kernel SPI module if it was detected before
	if(has_spi_module)
		shell("/sbin/modprobe dw_spi_mmio");
}

/*************************************************/
int rtapi_app_main()
{
	int ret;

	// Remove kernel SPI module if detected
	if((has_spi_module = (0 == shell("/usr/bin/grep -qw ^dw_spi_mmio /proc/modules")))) {
		if(shell("/sbin/rmmod dw_spi_mmio dw_spi"))
			LL_ERR("Unable to remove kernel SPI modules dw_spi_mmio and dw_spi. "
					"Your system may become unstable using LinuxCNC with the " HM2_LLIO_NAME " driver.");
	}

	if((comp_id = ret = hal_init(HM2_LLIO_NAME)) < 0)
		goto fail;

	if((ret = hm2_rp5spi_setup()) < 0)
		goto fail;

	hal_ready(comp_id);
	return 0;

fail:
	hm2_rp5spi_cleanup();
	return ret;
}

/*************************************************/
void rtapi_app_exit(void)
{
	hm2_rp5spi_cleanup();
	hal_exit(comp_id);
}

// vim: ts=4
