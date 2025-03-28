//
//    Copyright (C) 2007-2008 Sebastian Kuzminsky
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#ifndef __HOSTMOT2_UART_H
#define __HOSTMOT2_UART_H

#include <rtapi.h>

/*
 * http://freeby.mesanet.com/regmap
 * The PktUARTxMode register is used for setting and checking the
 * PktUARTx's operation mode, timing and status:
 * Bit  21     RO FrameBuffer Has Data
 * Bits 20..16 RO Frames to send (input and output ports can overlap with an FPGA)
 * Bit  19     WO Number of stopbits (0=one, 1=two)
 * Bit  18     WO Odd Parity (1=odd, 0=even)
 * Bit  17     WO Parity enable
 * Bits 15..8  RW InterFrame delay in bit times
 * Bit   7     RO Transmit Logic active, not an error
 * Bit   6     RW Drive Enable bit (enables external RS-422/485 Driver when set)
 * Bit   5     RW Drive enable Auto (Automatic external drive enable)
 * Bit   4     RO SCFIFO Error
 * Bits  3..0  RW Drive enable delay (delay from asserting drive enable
 *                to start of data transmit). In CLock Low periods
 */
#define HM2_PKTUART_TXMODE_MASK              ((1u << 22) - 1)
#define HM2_PKTUART_TXMODE_HASDATA_BIT       21  // RO
#define HM2_PKTUART_TXMODE_STOPBITS_BIT      19  // WO (new firmware)
#define HM2_PKTUART_TXMODE_PARITYODD_BIT     18  // WO
#define HM2_PKTUART_TXMODE_PARITYEN_BIT      17  // WO
#define HM2_PKTUART_TXMODE_NFRAMES_BIT       16  // RO (16..20)
#define HM2_PKTUART_TXMODE_INTERFRAMEDLY_BIT  8  // RW ( 8..15)
#define HM2_PKTUART_TXMODE_TXBUSY_BIT         7  // RO
#define HM2_PKTUART_TXMODE_DRIVEEN_BIT        6  // RW
#define HM2_PKTUART_TXMODE_DRIVEAUTO_BIT      5  // RW
#define HM2_PKTUART_TXMODE_ERRORSCFIFO_BIT    4  // RO
#define HM2_PKTUART_TXMODE_DRIVEENDLY_BIT     0  // RW ( 0..3)
#define HM2_PKTUART_TXMODE_HASDATA           (1u << HM2_PKTUART_TXMODE_HASDATA_BIT)
#define HM2_PKTUART_TXMODE_NFRAMES_MASK      (0x1fu << HM2_PKTUART_TXMODE_NFRAMES_BIT)
#define HM2_PKTUART_TXMODE_NFRAMES(x)        (((x) & 0x1fu) << HM2_PKTUART_TXMODE_NFRAMES_BIT)
#define HM2_PKTUART_TXMODE_NFRAMES_VAL(x)    (((x) >> HM2_PKTUART_TXMODE_NFRAMES_BIT) & 0x1fu)
#define HM2_PKTUART_TXMODE_PARITYEN          (1u << HM2_PKTUART_TXMODE_PARITYEN_BIT)
#define HM2_PKTUART_TXMODE_PARITYODD         (1u << HM2_PKTUART_TXMODE_PARITYODD_BIT)
#define HM2_PKTUART_TXMODE_INTERFRAMEDLY_MASK   (0xffu << HM2_PKTUART_TXMODE_INTERFRAMEDLY_BIT)
#define HM2_PKTUART_TXMODE_INTERFRAMEDLY(x)     (((x) & 0xffu) << HM2_PKTUART_TXMODE_INTERFRAMEDLY_BIT)
#define HM2_PKTUART_TXMODE_INTERFRAMEDLY_VAL(x) (((x) >> HM2_PKTUART_TXMODE_INTERFRAMEDLY_BIT) & 0xffu)
#define HM2_PKTUART_TXMODE_TXBUSY            (1u << HM2_PKTUART_TXMODE_TXBUSY_BIT)
#define HM2_PKTUART_TXMODE_DRIVEEN           (1u << HM2_PKTUART_TXMODE_DRIVEEN_BIT)
#define HM2_PKTUART_TXMODE_DRIVEAUTO         (1u << HM2_PKTUART_TXMODE_DRIVEAUTO_BIT)
#define HM2_PKTUART_TXMODE_ERRORSCFIFO       (1u << HM2_PKTUART_TXMODE_ERRORSCFIFO_BIT)
#define HM2_PKTUART_TXMODE_DRIVEENDLY_MASK   (0xfu << HM2_PKTUART_TXMODE_DRIVEENDLY_BIT)
#define HM2_PKTUART_TXMODE_DRIVEENDLY(x)     (((x) & 0xfu) << HM2_PKTUART_TXMODE_DRIVEENDLY_BIT)
#define HM2_PKTUART_TXMODE_DRIVEENDLY_VAL(x) (((x) >> HM2_PKTUART_TXMODE_DRIVEENDLY_BIT) & 0xfu)

/*
 * http://freeby.mesanet.com/regmap
 * The PktUARTrMode register is used for setting and checking the PktUARTr's
 * operation mode, timing, and status
 * Bit  30     RO BadPop Error (read data FIFO with no data) RO
 * Bits 29..22 RW RX data digital filter (in ClockLow periods)
 * 	          Should be set to 1/2 bit time
 *                (or max=255 if it cannot be set long enough)
 * Bit  21     RO FrameBuffer has data
 * Bit  18     WO Odd Parity  WO  (1=odd, 0=even)
 * Bit  17     WO Parity enable WO
 * Bits 20..16 RW Frames received
 * Bits 15..8  RW InterFrame delay in bit times
 * Bit   7     RO Receive Logic active, not an error
 * Bit   6     RO RXMask
 * Bit   5     RW Parity error
 * Bit   4     RW RCFIFO Error
 * Bit   3     RW RXEnable (must be set to receive packets)
 * Bit   2     RW RXMask Enable (enables input data masking when transmitting)
 * Bit   1     RW Overrun error (no stop bit when expected) (sticky)
 * Bit   0     RW False Start bit error (sticky)
 */
#define HM2_PKTUART_RXMODE_MASK              ((1u << 31) - 1)
#define HM2_PKTUART_RXMODE_ERRORBADPOP_BIT   30  // RO
#define HM2_PKTUART_RXMODE_RXFILTER_BIT      22  // RW (22..29)
#define HM2_PKTUART_RXMODE_HASDATA_BIT       21  // RO
#define HM2_PKTUART_RXMODE_PARITYODD_BIT     18  // WO
#define HM2_PKTUART_RXMODE_PARITYEN_BIT      17  // WO
#define HM2_PKTUART_RXMODE_NFRAMES_BIT       16  // RO (16..20)
#define HM2_PKTUART_RXMODE_INTERFRAMEDLY_BIT  8  // RW ( 8..15)
#define HM2_PKTUART_RXMODE_RXBUSY_BIT         7  // RO
#define HM2_PKTUART_RXMODE_RXMASK_BIT         6  // RO
#define HM2_PKTUART_RXMODE_ERRORPARITY_BIT    5  // RW
#define HM2_PKTUART_RXMODE_ERRORRCFIFO_BIT    4  // RW
#define HM2_PKTUART_RXMODE_RXEN_BIT           3  // RW
#define HM2_PKTUART_RXMODE_RXMASKEN_BIT       2  // RW
#define HM2_PKTUART_RXMODE_ERROROVERRUN_BIT   1  // RW
#define HM2_PKTUART_RXMODE_ERRORSTARTBIT_BIT  0  // RW
#define HM2_PKTUART_RXMODE_ERRORBADPOP        (1u << HM2_PKTUART_RXMODE_ERRORBADPOP_BIT)
#define HM2_PKTUART_RXMODE_RXFILTER_MASK      (0xffu << HM2_PKTUART_RXMODE_RXFILTER_BIT)
#define HM2_PKTUART_RXMODE_RXFILTER(x)        (((x) & 0xffu) << HM2_PKTUART_RXMODE_RXFILTER_BIT)
#define HM2_PKTUART_RXMODE_RXFILTER_VAL(x)    (((x) >> HM2_PKTUART_RXMODE_RXFILTER_BIT) & 0xffu)
#define HM2_PKTUART_RXMODE_HASDATA            (1u << HM2_PKTUART_RXMODE_HASDATA_BIT)
#define HM2_PKTUART_RXMODE_NFRAMES_MASK       (0x1fu << HM2_PKTUART_RXMODE_NFRAMES_BIT)
#define HM2_PKTUART_RXMODE_NFRAMES(x)         (((x) & 0x1fu) << HM2_PKTUART_RXMODE_NFRAMES_BIT)
#define HM2_PKTUART_RXMODE_NFRAMES_VAL(x)     (((x) >> HM2_PKTUART_RXMODE_NFRAMES_BIT) & 0x1fu)
#define HM2_PKTUART_RXMODE_PARITYEN           (1u << HM2_PKTUART_RXMODE_PARITYEN_BIT)
#define HM2_PKTUART_RXMODE_PARITYODD          (1u << HM2_PKTUART_RXMODE_PARITYODD_BIT)
#define HM2_PKTUART_RXMODE_INTERFRAMEDLY_MASK   (0xffu << HM2_PKTUART_RXMODE_INTERFRAMEDLY_BIT)
#define HM2_PKTUART_RXMODE_INTERFRAMEDLY(x)     (((x) & 0xffu) << HM2_PKTUART_RXMODE_INTERFRAMEDLY_BIT)
#define HM2_PKTUART_RXMODE_INTERFRAMEDLY_VAL(x) (((x) >> HM2_PKTUART_RXMODE_INTERFRAMEDLY_BIT) & 0xffu)
#define HM2_PKTUART_RXMODE_RXBUSY             (1u << HM2_PKTUART_RXMODE_RXBUSY_BIT)
#define HM2_PKTUART_RXMODE_RXMASK             (1u << HM2_PKTUART_RXMODE_RXMASK_BIT)
#define HM2_PKTUART_RXMODE_ERRORPARITY        (1u << HM2_PKTUART_RXMODE_ERRORPARITY_BIT)
#define HM2_PKTUART_RXMODE_ERRORRCFIFO        (1u << HM2_PKTUART_RXMODE_ERRORRCFIFO_BIT)
#define HM2_PKTUART_RXMODE_RXEN               (1u << HM2_PKTUART_RXMODE_RXEN_BIT)
#define HM2_PKTUART_RXMODE_RXMASKEN           (1u << HM2_PKTUART_RXMODE_RXMASKEN_BIT)
#define HM2_PKTUART_RXMODE_ERROROVERRUN       (1u << HM2_PKTUART_RXMODE_ERROROVERRUN_BIT)
#define HM2_PKTUART_RXMODE_ERRORSTARTBIT      (1u << HM2_PKTUART_RXMODE_ERRORSTARTBIT_BIT)

/*
 * http://freeby.mesanet.com/regmap
 * The PktUARTx/PktUARTr mode register has a special data command that clears
 * the PktUARTx/PktUARTr Clearing aborts any sends/receives in process, clears
 * the data FIFO and clears the send count FIFO. To issue a clear command, you
 * write 0x80010000 to the PktUARTx/PktUARTr mode register.
*/
#define HM2_PKTUART_CLEAR 0x80010000

// Receive count register
#define HM2_PKTUART_RCR_MASK              0xffff
#define HM2_PKTUART_RCR_ERROROVERRUN_BIT  15
#define HM2_PKTUART_RCR_ERRORSTARTBIT_BIT 14
#define HM2_PKTUART_RCR_NBYTES_BIT         0
#define HM2_PKTUART_RCR_ERROROVERRUN      (1u << HM2_PKTUART_RCR_ERROROVERRUN_BIT)
#define HM2_PKTUART_RCR_ERRORSTARTBIT     (1u << HM2_PKTUART_RCR_ERRORSTARTBIT_BIT)
#define HM2_PKTUART_RCR_NBYTES_MASK       (0x3ffu << HM2_PKTUART_RCR_NBYTES_BIT)
#define HM2_PKTUART_RCR_NBYTES(x)         (((x) & 0x3ffu) << HM2_PKTUART_RCR_NBYTES_BIT)
#define HM2_PKTUART_RCR_NBYTES_VAL(x)     (((x) >> HM2_PKTUART_RCR_NBYTES_BIT) & 0x3ffu)

/* Exported PktUART functions */
RTAPI_BEGIN_DECLS
// hm2_pktuart_setup() should be marked deprecated?
// Is replaced by hm2_pktuart_setup_{r,t}x() in V2.
int hm2_pktuart_setup(const char *name, unsigned bitrate, rtapi_s32 tx_mode, rtapi_s32 rx_mode, int txclear, int rxclear);
int hm2_pktuart_setup_rx(const char *name, unsigned int bitrate, unsigned int filter_hz, unsigned int parity, int frame_delay, bool rx_enable, bool rx_mask);
int hm2_pktuart_setup_tx(const char *name, unsigned int bitrate, unsigned int parity, int frame_delay, bool drive_enable, bool drive_auto, int enable_delay);

// Immediate out-of-band reset
void hm2_pktuart_reset(const char *name);
// Reset but with normal queue processing
void hm2_pktuart_queue_reset(const char *name);

int hm2_pktuart_send(const char *name, const unsigned char data[], rtapi_u8 *num_frames, const rtapi_u16 frame_sizes[]);
// The hm2_pkuart_read() function should be declared deprecated because it
// reads directly bypassing the queue_read function.
// Unfortunately, it is still used in hal/components/mesa_pktgyro_test.comp
int hm2_pktuart_read(const char *name, unsigned char data[],  rtapi_u8 *num_frames, rtapi_u16 *max_frame_length, rtapi_u16 frame_sizes[]);
int hm2_pktuart_queue_get_frame_sizes(const char *name, rtapi_u32 fsizes[]);
int hm2_pktuart_queue_read_data(const char *name, rtapi_u32 *data, int bytes);
int hm2_pktuart_get_clock(const char *name);
int hm2_pktuart_get_version(const char *name);
rtapi_u32 hm2_pktuart_get_rx_status(const char *name);
rtapi_u32 hm2_pktuart_get_tx_status(const char *name);
RTAPI_END_DECLS


/* Exported UART functions */
RTAPI_BEGIN_DECLS
int hm2_uart_setup(char *name, int bitrate, rtapi_s32 tx_mode, rtapi_s32 rx_mode);
int hm2_uart_send(char *name, unsigned char data[], int count);
int hm2_uart_read(char *name, unsigned char data[]);
RTAPI_END_DECLS


/* Exported Buffered SPI functions */
RTAPI_BEGIN_DECLS
int hm2_bspi_setup_chan(char *name, int chan, int cs, int bits, double mhz,
                        int delay, int cpol, int cpha, int noclear, int noecho,
                        int samplelate);
int hm2_bspi_set_read_function(char *name, int (*func)(void *subdata), void *subdata);
int hm2_bspi_set_write_function(char *name, int (*func)(void *subdata), void *subdata);
int hm2_bspi_write_chan(char* name, int chan, rtapi_u32 val);
int hm2_allocate_bspi_tram(char* name);
int hm2_tram_add_bspi_frame(char *name, int chan, rtapi_u32 **wbuff, rtapi_u32 **rbuff);
int hm2_bspi_clear_fifo(char * name);
RTAPI_END_DECLS

#endif
