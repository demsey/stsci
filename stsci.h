/*
 *  drivers/serial/stsci.h
 *
 *  Open ST40 Smartcard Card Interface (SCI) driver
 *  Derived from sh-sci.h and stasc.h
 *  Copyright (c) PKT Polish Kathi Team
 *  Author: Robert Demski (August 2010)
 *
 *  Know-how for the Asynchronous Serial Controller in the ST40 SoC
 *  can be found in the following places:
 *
 *    1) ASC - STM kernel source drivers/serial/stasc.c
 *    2) SYSCON - STM kernel source includes/linux/stm/syscon.h
 */

#ifndef _STSCI_H
#define _STSCI_H

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
#define STM23
#else
#define STM22
#define SYS_CFG	0 //fake FIXME
#endif

/* Register offsets */

#define ASC_BAUDRATE                    0x00
#define ASC_TXBUF                       0x04
#define ASC_RXBUF                       0x08
#define ASC_CTL                         0x0C
#define ASC_INTEN                       0x10
#define ASC_STA                         0x14
#define ASC_GUARDTIME                   0x18
#define ASC_TIMEOUT                     0x1C
#define ASC_TXRESET                     0x20
#define ASC_RXRESET                     0x24
#define ASC_RETRIES                     0x28

/* ASC_RXBUF */
#define ASC_RXBUF_PE                    0x100
#define ASC_RXBUF_FE                    0x200

/* ASC_CTL */

#define ASC_CTL_MODE_MSK		0x0007
#define ASC_CTL_MODE_8BIT		0x0001
#define ASC_CTL_MODE_7BIT_PAR		0x0003
#define ASC_CTL_MODE_9BIT		0x0004
#define ASC_CTL_MODE_8BIT_WKUP		0x0005
#define ASC_CTL_MODE_8BIT_PAR		0x0007
#define ASC_CTL_STOP_MSK		0x0018
#define ASC_CTL_STOP_HALFBIT		0x0000
#define ASC_CTL_STOP_1BIT		0x0008
#define ASC_CTL_STOP_1_HALFBIT		0x0010
#define ASC_CTL_STOP_2BIT		0x0018
#define ASC_CTL_PARITYODD		0x0020
#define ASC_CTL_LOOPBACK		0x0040
#define ASC_CTL_RUN			0x0080
#define ASC_CTL_RXENABLE		0x0100
#define ASC_CTL_SCENABLE		0x0200
#define ASC_CTL_FIFOENABLE		0x0400
#define ASC_CTL_CTSENABLE		0x0800
#define ASC_CTL_BAUDMODE		0x1000

/* ASC_GUARDTIME */

#define ASC_GUARDTIME_MSK               0x00FF

/* ASC_INTEN */

#define ASC_INTEN_RBE			0x0001	//Receive Buffer full interrupt enable
#define ASC_INTEN_TE			0x0002	//Transmitter empty interrupt enable
#define ASC_INTEN_THE			0x0004	//Transmitter buffer half empty interrupt enable
#define ASC_INTEN_PE			0x0008	//Parity error interrupt enable
#define ASC_INTEN_FE			0x0010	//Framing error interrupt enable
#define ASC_INTEN_OE			0x0020	//Overrun error interrupt enable
#define ASC_INTEN_TNE			0x0040	//Time out when the receiver not empty interrupt enable
#define ASC_INTEN_TOI			0x0080	//Time out when the receiver is empty interrupt enable
#define ASC_INTEN_RHF			0x0100	//Receiver buffer is Half Full interrupt enable


/* ASC_STA */

#define ASC_STA_RBF			0x0001	//Receive Buffer Full flag
#define ASC_STA_TE			0x0002	//Transmitter empty flag
#define ASC_STA_THE			0x0004	//Transmitter at least half empty flag
#define ASC_STA_PE                      0x0008	//Input parity error flag
#define ASC_STA_FE                      0x0010	//Input error frame flag (stop bits not found)
#define ASC_STA_OE                      0x0020	//Overrun error flag
#define ASC_STA_TNE                     0x0040	//Timeout when the receiver FIFO is not empty
#define ASC_STA_TOI                     0x0080	//Timeout when the receiver FIFO is empty
#define ASC_STA_RHF                     0x0100	//Receive buffer at least half Full flag
#define ASC_STA_TF                      0x0200	//Transmitter buffer full flag
#define ASC_STA_NKD                     0x0400	//Transmission failure acknowledgement by receiver in smartcard mode



/*---- Access macros ------------------------------------------*/

#define ASC_FUNC(name, offset)          \
  static inline unsigned int asc_ ## name ## _in (struct uart_port* port)       \
  {                                                                             \
    return (readl(port->membase + (offset)));                                   \
  }                                                                             \
  static inline void asc_ ## name ## _out (struct uart_port* port, unsigned int value)  \
  {                                                                             \
    writel(value, port->membase + (offset));                                    \
  }

ASC_FUNC(BAUDRATE,  ASC_BAUDRATE)
ASC_FUNC(TXBUF,     ASC_TXBUF)
ASC_FUNC(RXBUF,     ASC_RXBUF)
ASC_FUNC(CTL,       ASC_CTL)
ASC_FUNC(INTEN,     ASC_INTEN)
ASC_FUNC(STA,       ASC_STA)
ASC_FUNC(GUARDTIME, ASC_GUARDTIME)
ASC_FUNC(TIMEOUT,   ASC_TIMEOUT)
ASC_FUNC(TXRESET,   ASC_TXRESET)
ASC_FUNC(RXRESET,   ASC_RXRESET)
ASC_FUNC(RETRIES,   ASC_RETRIES)

#define asc_in(port, reg)               asc_ ## reg ## _in (port)
#define asc_out(port, reg, value)       asc_ ## reg ## _out ((port), (value))

#define ADJ 1
#define BAUDRATE_VAL_M0(bps, clk)       ((clk) / (16 * (bps)))
#define BAUDRATE_VAL_M1(bps, clk)       ( ((bps * (1 << 14)) / ((clk) / (1 << 6)) ) + ADJ )


#ifdef STM22
#define SCI_MAJOR		253
#else
#define SCI_MAJOR		260	//in stm23 253 is reserved
#endif
#define SCI_MINOR_START		0


#define FIFO_SIZE               16

#define SCI_NPORTS		2

#define IRQ_TEST_LEVEL		0


#define SCG0			0xb8048000	/* SCI0 clock generator base address */
#define SCG1			0xb8049000	/* SCI1 clock generator base address */

#define SCI_CLK_VAL_MASK	0x0000001f	/* SCI clock divider value mask */
#define SCI_CLK_CTRL_EN		0x00000002	/* bit 1 - enable SCI clock generator */


#define PIO0_SCCLK_NOT_CLK_DSS	0x00000010	/* SmardCard0 clock source: 0 - CLK_DSS, 1 - SmardCard clock generator (SCG0) */
#define PIO1_SCCLK_NOT_CLK_DSS	0x00000020	/* SmardCard1 clock source: 0 - CLK_DSS, 1 - SmardCard clock generator (SCG1) */
#define SC_COND_VCC_EN		0x00000080	/* bit 7 - enable automatic enable VCC for smartcard after detect insertion */
#define SC_DETECT_POL		0x00000100	/* bit 8 - reverse polarity of SC_DETECT if SC_COND_VCC_EN selected */
#define EMPI_DRACK_SEL		0x00002000

#define SYS_CFG_BASE		0xb9001000	/* System Configuration base */
#define SYS_CFG7		(SYS_CFG_BASE + 0x11C)	/* COMMs configuration register */


#define TS_DIRECT	0x3b			/* Convention indicator */

#endif /* _STSCI_H */
