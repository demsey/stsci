/*
 *  drivers/serial/stsci.c
 *  Open ST40 SmartCard Interface (SCI) driver
 */

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
#define STM23
#else
#define STM22
#endif

#include <linux/module.h>
#include <linux/init.h>
#ifdef STM22
#include "bitrev.h"
#include <linux/stpio.h>
#else
#include <linux/bitrev.h>
#include <linux/stm/pio.h>
#include <linux/stm/sysconf.h>
#endif
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>


#include <asm/io.h>
#include <asm/clock.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include "stsci.h"
#include "scif.h"



struct sci_port;
static void sci_socket_interrupt(struct stpio_pin *pin, void *dev);
static void scg_enable_clock( struct uart_port *port );
static void scg_disable_clock( struct uart_port *port );
static void syscfg_enable_scg_clock(struct uart_port *port);
static int sci_set_baud (struct uart_port *port, int baud);
static void syscfg_enable_auto_vcc(struct uart_port *port);
static void syscfg_disable_auto_vcc(struct uart_port *port);
static int sci_request_irq(struct uart_port *port);
static void sci_free_irq(struct uart_port *port);
static int sci_pios_init(struct uart_port *port);
static int sci_pios_free(struct uart_port *port);
static void sci_transmit_chars(struct uart_port *port);
static int sci_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg );
#ifdef STM22
static inline void sci_enable(struct uart_port *);
#else
static inline void sci_enable(struct work_struct *);
#endif
static char *sci_hexdump(int m, unsigned char *buf, size_t n);


#define UART_TXD	0
#define UART_RXD	1
#define SC_EXTCLKIN	2
#define SC_CLK_OUT	3
#define UART_CTS	4
#define SC_COND_VCC	5
#define UART_DIR	6
#define SC_DETECT	7


#define to_sci_port(x) container_of((x), struct sci_port, port)

#define CONFIG_TIMEOUT	50	//Timeout for wait for uart configuration
#define RECEIVE_TIMEOUT 500	//Receive timeout (in ms) after receiving last char

struct sci_pio
{
	const char*		name;
	unsigned char		port;
	unsigned char		pin;
	unsigned char		dir;
	struct stpio_pin*	pio;
	void (*handler)(struct stpio_pin *pin, void *dev);
};

struct cfg_bits
{
	const unsigned int	addr;
	const unsigned long	mask;
	const unsigned int	group;
	const unsigned int	reg;
#ifdef STM22
#else
	struct sysconf_field*	field;
#endif

};

struct sci_port
{
	struct uart_port port;
	struct sci_pio*	pios;	/* PIO pin settings for driver */
	unsigned char	pios_nr;
	int		break_flag;
	int		dma_enabled;
	unsigned int	baud;
	unsigned int	ctrl;
	struct cfg_bits	scgclkval;
	struct cfg_bits	scgclken;
	struct cfg_bits	clksrc;
	struct cfg_bits	autovcc;
	struct cfg_bits	scdetpol;
#ifdef STM22
	struct work_struct	work;
#else
	struct delayed_work	work;
#endif
	wait_queue_head_t	initwq;
	struct kfifo*	sendq;
	spinlock_t*	sendq_lock;
	struct kfifo*	receq;
	spinlock_t*	receq_lock;
	struct kfifo*	flagq;
	unsigned char	configured;
	unsigned char	iscard;
	unsigned char	isatr;
	unsigned char	ts;
	unsigned char	atr_timeout_cnt;
	
};

static struct sci_pio sci_pios0[] = {
	[0] = {
		.name	= "UART0_TxD",
		.port	= 0,
		.pin	= 0,
		.dir 	= STPIO_ALT_BIDIR,
	},
	[1] = {
		.name  	= "UART0_RxD",
		.port	= 0,
		.pin	= 1,
		.dir 	= STPIO_IN,
	},
	[3] = {
		.name   = "SC_CLK_OUT",
		.port	= 0,
		.pin	= 3,
		.dir 	= STPIO_ALT_BIDIR,
	},
	[4] = {
		.name  	= "UART0_CTS",
		.port	= 0,
		.pin	= 4,
		.dir 	= STPIO_IN,
	},
	[5] = {
		.name  	= "SC_COND_VCC",
		.port	= 0,
		.pin	= 5,
		.dir 	= STPIO_ALT_OUT,
	},
	[7] = {
		.name  	= "SC_DETECT",
		.port	= 0,
		.pin	= 7,
		.dir 	= STPIO_IN,
		.handler= sci_socket_interrupt,
	},
};

/*---- Inline function definitions ---------------------------*/

/* Some simple utility functions to enable and disable interrupts.
 * Note that these need to be called with interrupts disabled.
 */
static inline void sci_disable_tx_interrupts(struct uart_port *port)
{
	unsigned long intenable;

	/* Clear TE (Transmitter empty) interrupt enable in INTEN */
	intenable = asc_in(port, INTEN);
//	intenable &= ~ASC_INTEN_THE;
	intenable &= ~ASC_INTEN_TE;
	asc_out(port, INTEN, intenable);
}

static inline void sci_enable_tx_interrupts(struct uart_port *port)
{
	unsigned long intenable;

	/* Set TE (Transmitter empty) interrupt enable in INTEN */
	intenable = asc_in(port, INTEN);
//	intenable |= ASC_INTEN_THE;
	intenable |= ASC_INTEN_TE;
	asc_out(port, INTEN, intenable);
}

static inline void sci_disable_rx_interrupts(struct uart_port *port)
{
	unsigned long intenable;

	/* Clear RBE (Receive Buffer Full Interrupt Enable) bit in INTEN */
	intenable = asc_in(port, INTEN);
	intenable &= ~ASC_INTEN_RBE;
//	intenable &= ~ASC_INTEN_RHF;
	asc_out(port, INTEN, intenable);
}

static inline void sci_enable_rx_interrupts(struct uart_port *port)
{
	unsigned long intenable;

	/* Set RBE (Receive Buffer Full Interrupt Enable) bit in INTEN */
	intenable = asc_in(port, INTEN);
	intenable |= ASC_INTEN_RBE;
//	intenable |= ASC_INTEN_RHF;
	asc_out(port, INTEN, intenable);
}

/*----------------------------------------------------------------------*/



/*
 * UART Functions
 */

static unsigned int sci_tx_empty(struct uart_port *port)
{
	unsigned long status;

	status = asc_in(port, STA);
	if (status & ASC_STA_TE)
		return TIOCSER_TEMT;
	return 0;
}

static void sci_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* This routine is used for seting signals of: DTR, DCD, CTS/RTS
	 * We use ASC's hardware for CTS/RTS, so don't need any for that.
	 * Some boards have DTR and DCD implemented using PIO pins,
	 * code to do this should be hooked in here.
	 */
}

static unsigned int sci_get_mctrl(struct uart_port *port)
{
	/* This routine is used for geting signals of: DTR, DCD, DSR, RI,
	   and CTS/RTS */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

/*
 * There are probably characters waiting to be transmitted.
 * Start doing so.
 * The port lock is held and interrupts are disabled.
 */
static void sci_start_tx(struct uart_port *port)
{
        printk(KERN_INFO "STSCI start_tx.\n");
	sci_transmit_chars(port);
}

/*
 * Transmit stop - interrupts disabled on entry
 */
static void sci_stop_tx(struct uart_port *port)
{
	sci_disable_tx_interrupts(port);
        printk(KERN_INFO "STSCI stop_tx.\n");
}

/*
 * Receive stop - interrupts still enabled on entry
 */
static void sci_stop_rx(struct uart_port *port)
{
	syscfg_disable_auto_vcc(port);
	scg_disable_clock(port);
	sci_disable_rx_interrupts(port);
        printk(KERN_INFO "STSCI stop_rx.\n");
}


/*
 * Force modem status interrupts on - no-op for us
 */
static void sci_enable_ms(struct uart_port *port)
{
	/* Nothing here yet .. */
}

/*
 * Handle breaks - ignored by us
 */
static void sci_break_ctl(struct uart_port *port, int break_state)
{
	/* Nothing here yet .. */
}

#ifdef STM22
static void sci_reset(struct uart_port *port)
{
#else
static void sci_reset(struct work_struct *work)
{
	struct sci_port *sciport = container_of(work, struct sci_port, work.work);
	struct uart_port *port = &sciport->port;
#endif

	sci_pios_free(port);
	sci_pios_init(port);
	scg_enable_clock(port);
}

/*
 * Enable port for reception.
 * port_sem held and interrupts disabled
 */
static int sci_startup(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);

        printk(KERN_INFO "STSCI startup.\n");

//	sci_pios_init(port);
//	sci_request_irq(port);

//	sci_transmit_chars(port);
	asc_out(port, CTL, asc_in(port,CTL) & ~ASC_CTL_RUN);
	asc_out(port, CTL, asc_in(port,CTL) & ~ASC_CTL_RXENABLE);
	asc_out(port, INTEN, asc_in(port,INTEN) & ~ASC_INTEN_TOI);
	sci_enable_rx_interrupts(port);

	spin_lock_init(sciport->sendq_lock);
	sciport->sendq = kfifo_alloc(256, GFP_KERNEL, sciport->sendq_lock);
	spin_lock_init(sciport->receq_lock);
	sciport->receq = kfifo_alloc(256, GFP_KERNEL, sciport->receq_lock);
	sciport->flagq = kfifo_alloc(256, GFP_KERNEL, sciport->receq_lock);
	sciport->configured = 0;
	sciport->isatr = 0;
	sciport->ts    = 0;

	syscfg_enable_scg_clock(port);

	syscfg_enable_auto_vcc(port);
	cancel_delayed_work(&sciport->work);

#ifdef STM22
	INIT_WORK(&sciport->work, (void (*)(void *))sci_reset, port);
#else
	INIT_DELAYED_WORK(&sciport->work, (work_func_t)sci_reset);
#endif
	schedule_delayed_work(&sciport->work,msecs_to_jiffies(CONFIG_TIMEOUT));
	wake_up_interruptible(&sciport->initwq);

	return 0;
}

static void sci_shutdown(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);
	asc_out (port, CTL, asc_in(port, CTL) & ~ASC_CTL_RUN);
	sci_disable_tx_interrupts(port);
	asc_out(port, INTEN, asc_in(port,INTEN) & ~ASC_INTEN_TOI);
	sci_disable_rx_interrupts(port);

	scg_disable_clock(port);

//	sci_pios_free(port);
//	sci_free_irq(port);
	sciport->configured = 0;
	kfifo_free(sciport->sendq);
	kfifo_free(sciport->receq);
	kfifo_free(sciport->flagq);

        printk(KERN_INFO "STSCI shutdown.\n");
}
static void sci_release_port(struct uart_port *port)
{
	/* Nothing here yet .. */
	struct sci_port *sciport = to_sci_port(port);

	release_mem_region(port->mapbase, 0x100+1);
	iounmap(port->membase);
#ifdef STM22
#else
	if (sciport->clksrc.field)
		sysconf_release(sciport->clksrc.field);
	if (sciport->autovcc.field)
		sysconf_release(sciport->autovcc.field);
	if (sciport->scdetpol.field)
		sysconf_release(sciport->scdetpol.field);
#endif
}

#ifdef STM22
#else
static void sci_configure_field(struct cfg_bits* f )
{
	int lsb = ffs(f->mask)-1;
	int msb = 32-ffs(bitrev32(f->mask));
	f->field = sysconf_claim(f->group,f->reg,lsb,msb,"stsci");
}
#endif

static int sci_request_port(struct uart_port *port)
{
	/* Nothing here yet .. */
	struct sci_port *sciport = to_sci_port(port);

	if (!request_mem_region(port->mapbase, 0x100+1, "stsci"))
		return -EBUSY;

#ifdef STM22
#else
	sci_configure_field(&sciport->clksrc);
	sci_configure_field(&sciport->autovcc);
	sci_configure_field(&sciport->scdetpol);
#endif
	return 0;
}

/* Called when the port is opened, and UPF_BOOT_AUTOCONF flag is set */
/* Set type field if successful */
static void sci_config_port(struct uart_port *port, int flags)
{
	struct sci_port *sciport = to_sci_port(port);

	sci_request_port(port);

	if (!port->membase)
		port->membase = ioremap(port->mapbase, 0x100+1);
	if (!port->membase)
		return;

	sciport->configured = 0;

//	syscfg_enable_auto_vcc(port);
	syscfg_enable_scg_clock(port);

	init_waitqueue_head(&sciport->initwq);
	
	sci_pios_init(port);
	sci_request_irq(port);

	port->type = PORT_SCI;
	port->fifosize = FIFO_SIZE;
        printk(KERN_INFO "STSCI config.\n");
}

#ifdef STM22
static void sci_set_termios(struct uart_port *port,
                            struct termios *termios, struct termios *old)
#else
static void sci_set_termios(struct uart_port *port,
                            struct ktermios *termios, struct ktermios *old)
#endif
{
	struct sci_port *sciport = to_sci_port(port);
	unsigned int ctrl_val;
	unsigned long flags;
	unsigned int baud;

        printk(KERN_INFO "STSCI set_termios.\n");

	spin_lock_irqsave(&port->lock, flags);

	baud = uart_get_baud_rate(port, termios, old, 0,
				port->uartclk/16);

	/* wait for end of current transmission */
//	while (!sci_tx_empty(port)){};

        /* read control register */
        ctrl_val = asc_in (port, CTL);

        asc_out (port, CTL, ctrl_val & ~ASC_CTL_RUN);

        /* reset fifo rx & tx */
        asc_out (port, TXRESET, 1);
        asc_out (port, RXRESET, 1);
	

	/* set default control configuration */	
	ctrl_val = sciport->ctrl;

        /* set speed and baud generator mode */
        ctrl_val |= sci_set_baud (port, sciport->baud);
	uart_update_timeout(port, termios->c_cflag, baud);

        asc_out (port, GUARDTIME, 2); //sci
        asc_out (port, TIMEOUT, 254);  //sci
        asc_out (port, RETRIES, 3);   //sci

        /* write final value and enable port */
        asc_out (port, CTL, ctrl_val);

	sciport->configured = 1;
	wake_up_interruptible(&sciport->initwq);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *sci_type(struct uart_port *port)
{
	return "sci";
}

static int
sci_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* No user changeable parameters */
	return -EINVAL;
}

/*----------------------------------------------------------------------*/
static struct uart_ops sci_uart_ops = {
	.tx_empty	= sci_tx_empty,
	.set_mctrl	= sci_set_mctrl,
	.get_mctrl	= sci_get_mctrl,
	.start_tx	= sci_start_tx,
	.stop_tx	= sci_stop_tx,
	.stop_rx	= sci_stop_rx,
	.enable_ms	= sci_enable_ms,
	.break_ctl	= sci_break_ctl,
	.startup	= sci_startup,
	.shutdown	= sci_shutdown,
	.set_termios	= sci_set_termios,
	.type		= sci_type,
	.release_port	= sci_release_port,
	.request_port	= sci_request_port,
	.config_port	= sci_config_port,
	.verify_port	= sci_verify_port,
	.ioctl		= sci_ioctl,
};

struct sci_port sci_ports[SCI_NPORTS] = {
	/* UART0 */
	{
		.port	= {
			.mapbase	= 0x18030000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 123,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.fifosize	= FIFO_SIZE,
			.line		= 0,
		},
		.pios		= sci_pios0,
		.pios_nr	= ARRAY_SIZE(sci_pios0),
		.baud		= 9600,
		.ctrl		= (ASC_CTL_MODE_8BIT_PAR |
				ASC_CTL_STOP_1_HALFBIT |
				ASC_CTL_PARITYODD |
				ASC_CTL_SCENABLE |
				ASC_CTL_RXENABLE |
				ASC_CTL_FIFOENABLE |
				ASC_CTL_RUN),
		.scgclkval = {
			.addr	= SCG0 + 0x00,		/* SCI clock 0 divider value */
			.mask	= SCI_CLK_VAL_MASK,
		},
		.scgclken = {
			.addr	= SCG0 + 0x04,		/* SCI clock 0 control register */
			.mask	= SCI_CLK_CTRL_EN,
		},
		.clksrc = {
			.addr	= SYS_CFG7,
			.mask	= PIO0_SCCLK_NOT_CLK_DSS,
			.group	= SYS_CFG,
			.reg	= 7,
		},
		.autovcc = {
			.addr	= SYS_CFG7,
			.mask	= SC_COND_VCC_EN,
			.group	= SYS_CFG,
			.reg	= 7,
		},
		.scdetpol = {
			.addr	= SYS_CFG7,
			.mask	= SC_DETECT_POL,
			.group	= SYS_CFG,
			.reg	= 7,
		},
	},
	/* UART1 */
	{
		.port	= {
			.mapbase	= 0x18031000,
			.iotype		= SERIAL_IO_MEM,
			.irq		= 122,
			.ops		= &sci_uart_ops,
			.flags		= ASYNC_BOOT_AUTOCONF,
			.fifosize	= FIFO_SIZE,
			.line		= 1,
		},
//		.pio_port	= 1,
//		.pio_pin	= {0, 1, 4, 5},
		.baud		= 9600,
		.ctrl		= (ASC_CTL_MODE_8BIT_PAR |
				ASC_CTL_STOP_1_HALFBIT |
				ASC_CTL_SCENABLE |
				ASC_CTL_RXENABLE |
				ASC_CTL_FIFOENABLE |
				ASC_CTL_RUN),
		.scgclkval = {
			.addr	= SCG1 + 0x00,		/* SCI clock 1 divider value */
			.mask	= SCI_CLK_VAL_MASK,
		},
		.scgclken = {
			.addr	= SCG1 + 0x04,		/* SCI clock 1 control register */
			.mask	= SCI_CLK_CTRL_EN,
		},
		.clksrc = {
			.addr	= SYS_CFG7,		/* COMMs configuration register */
			.mask	= PIO1_SCCLK_NOT_CLK_DSS,
		},
	},
};

static struct uart_driver sci_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "sci",
//	.devfs_name	= "sci/",
	.dev_name	= "sci",
	.major		= SCI_MAJOR,
	.minor		= SCI_MINOR_START,
	.nr		= SCI_NPORTS,
};

static void sci_flush_tty(struct uart_port *port)
{
	struct tty_struct *tty = port->info->tty;
	tty_flip_buffer_push(tty);
}

static void do_atr(void *private)
{
	struct sci_port *sciport = to_sci_port(private);
	struct uart_port *port = private;
	unsigned char *rb, *fb;
	int l = kfifo_len(sciport->receq);
	tty_prepare_flip_string_flags(port->info->tty, &rb, (char **)&fb, l);
	kfifo_get(sciport->receq, rb, l);
	kfifo_get(sciport->flagq, fb, l);
	printk(KERN_NOTICE "ATR: %s (%d)\n", sci_hexdump(1,rb,l), l );
	sciport->isatr = 1;
	sci_flush_tty(port);
}

static inline void sci_disable(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);
//	struct sci_pio *pio = &(sciport->pios[SC_COND_VCC]);
//	stpio_set_pin(pio->pio, 0);	//FIXME
	scg_disable_clock(port);
	cancel_delayed_work(&sciport->work);
}
#ifdef STM22
static inline void sci_enable(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);
#else
static inline void sci_enable(struct work_struct *work)
{
	struct sci_port *sciport = container_of(work, struct sci_port, work.work);
	struct uart_port *port = &sciport->port;
#endif
//	struct sci_pio *pio = &(sciport->pios[SC_COND_VCC]);
//	stpio_set_pin(pio->pio, 1);	//FIXME

	if (!sciport->configured)
		return;

        /* wait for the configure uart  */
//	wait_event_interruptible_timeout(sciport->initwq,
//		(sciport->configured), msecs_to_jiffies(CONFIG_TIMEOUT));
	printk(KERN_DEBUG "enable: configured: %d\n",sciport->configured);
	scg_enable_clock(port);

}

static void sci_socket_interrupt(struct stpio_pin *pin, void *dev)
{
	struct uart_port *port = dev;
	struct sci_port *sciport = to_sci_port(dev);
	struct sci_pio *pio = &(sciport->pios[SC_COND_VCC]);
	sciport->iscard = stpio_get_pin(pin);

	sciport->isatr	= 0;
	sciport->ts	= 0;
	stpio_disable_irq(pin);
	stpio_enable_irq(pin, sciport->iscard);
	if ( sciport->iscard == IRQ_TEST_LEVEL )
		sci_disable(port);
	else {
	  	stpio_set_pin(pio->pio, 1);
#ifdef STM22
		INIT_WORK(&sciport->work, (void (*)(void *))sci_enable, port);
#else
		INIT_DELAYED_WORK(&sciport->work, (work_func_t)sci_enable);
#endif
		schedule_delayed_work(&sciport->work, 0);
	}
}

static inline unsigned sci_hw_txroom(struct uart_port* port)
{
	unsigned long status;

	status = asc_in(port, STA);
	if (status & ASC_STA_THE) {
		return FIFO_SIZE/2;
	} else if (! (status & ASC_STA_TF)) {
		return 1;
	} else {
		return 0;
	}
}

/*
 * Start transmitting chars.
 * This is called from both interrupt and task level.
 * Either way interrupts are disabled.
 */
static void sci_transmit_chars(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);
	struct circ_buf *xmit = &port->info->xmit;
	int txroom;
	unsigned char c;
	unsigned char ol;

	txroom = sci_hw_txroom(port);

	if ((txroom != 0) && port->x_char) {
		c = port->x_char;
		port->x_char = 0;
		asc_out (port, TXBUF, c);
		port->icount.tx++;
		printk(KERN_DEBUG "Transmitted 0x%02x\n", (unsigned char)c);
		txroom = sci_hw_txroom(port);
	}
	while (txroom > 0) {
		if (uart_tx_stopped(port) || uart_circ_empty(xmit)) {
			break;
		}

		do {
			ol = c = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			if (sciport->ts != TS_DIRECT)
				c = bitrev8((unsigned char)c)^0xff;
//			printk(KERN_DEBUG "Transmitted 0x%02x (0x%02x), status: 0x%03x\n", (unsigned char)c, (unsigned char)ol, (unsigned char)asc_in(port, STA) );
			asc_out (port, TXBUF, c);
			kfifo_put(sciport->sendq, (unsigned char *)&ol, sizeof(ol));
			port->icount.tx++;
			txroom--;
		} while ((txroom > 0) && (!uart_circ_empty(xmit)));

		txroom = sci_hw_txroom(port);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}

	if (port->x_char || (!uart_circ_empty(xmit))) {
		sci_enable_tx_interrupts(port);
	} else {
		int l = kfifo_len(sciport->sendq);
		unsigned char sb[l];
		kfifo_get(sciport->sendq, sb, l); //fill sb buffer
		kfifo_put(sciport->sendq, sb, l); //put back sb buffer to fifo
		printk(KERN_INFO "APDU: %s (%d)\n",sci_hexdump(1,sb,l),l);
		sci_disable_tx_interrupts(port);
	}
}

#ifdef STM22
static inline void sci_receive_chars(struct uart_port *port,
                                     struct pt_regs *regs)
#else
static inline void sci_receive_chars(struct uart_port *port)
#endif
{
	int count;
	struct tty_struct *tty = port->info->tty;
	struct sci_port *sciport = to_sci_port(port);
	int copied=0;
	volatile unsigned long status;
	unsigned long c = 0;
	unsigned long b;
	char flag;
	int overrun;

	while (1) {
		status = asc_in(port, STA);
		if (status & ASC_STA_RHF) {
			count = FIFO_SIZE / 2;
		} else if (status & ASC_STA_RBF) {
			count = 1;
		} else {
			count = 0;
		}
		/* Check for overrun before reading any data from the
		 * RX FIFO, as this clears the overflow error condition. */
		overrun = status & ASC_STA_OE;

		/* Don't copy more bytes than there are room for in the buffer */
		count = tty_buffer_request_room(tty, count);

		/* If for any reason we can't copy more data, we're done! */
		if (count == 0)
			break;

		for ( ; count != 0; count--) {
			status = asc_in(port, STA);
			c = asc_in(port, RXBUF);
			flag = TTY_NORMAL;
			port->icount.rx++;
			if (unlikely(!sciport->ts)){
				unsigned long ctl = asc_in(port, CTL);
				if ( c & ASC_RXBUF_PE){
					asc_out(port, CTL, ctl ^ ASC_CTL_PARITYODD);
					break;
				}
				printk(KERN_NOTICE "Parity %s\n", ctl & ASC_CTL_PARITYODD?"ODD":"EVEN");
				sciport->ts = (c & 0xff)==TS_DIRECT?(c & 0xff):bitrev8(c)^0xff;
				printk(KERN_NOTICE "Convention TS 0x%02x\n", sciport->ts);
			}
			sciport->atr_timeout_cnt = unlikely(!sciport->isatr)?15:2;
			if ( ~(asc_in(port,INTEN) & ASC_INTEN_TOI) )
				asc_out(port, INTEN, asc_in(port,INTEN) | ASC_INTEN_TOI);

			b = (sciport->ts == TS_DIRECT)?c:bitrev8((unsigned char)c)^0xff;
//			printk(KERN_DEBUG "Recieved 0x%02x (0x%02x) count: %d status: 0x%03x\n", (unsigned char)c, (unsigned char)b, count, (unsigned int)status);

			if (unlikely(c & ASC_RXBUF_FE)) {
				if (c == ASC_RXBUF_FE) {
					port->icount.brk++;
					if (uart_handle_break(port))
						continue;
					flag = TTY_BREAK;
				} else {
					port->icount.frame++;
					flag = TTY_FRAME;
				}
			} else if (unlikely(c & ASC_RXBUF_PE)) {
				port->icount.parity++;
				flag = TTY_PARITY;
			}

#ifdef STM22
			if (uart_handle_sysrq_char(port, b, regs))
#else
			if (uart_handle_sysrq_char(port, b))
#endif
				continue;
//			tty_insert_flip_char(tty, b & 0xff, flag);
			kfifo_put(sciport->receq, (unsigned char *)&b, 1);
			kfifo_put(sciport->flagq, (unsigned char *)&flag, 1);
		}
                if (overrun) {
			unsigned char ov[] = {0,TTY_OVERRUN};
			port->icount.overrun++;
//			tty_insert_flip_char(tty, 0, TTY_OVERRUN);
			kfifo_put(sciport->receq, ov+0, 1);
			kfifo_put(sciport->flagq, ov+1, 1);
                }

		copied=1;
        }
	if (copied ) {
		/* Tell the rest of the system the news. New characters! */
//		sci_flush_tty(port);
	}
}


static inline void sci_timeout(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);

//	printk(KERN_DEBUG "Receive soft-timeout nr %d\n", (unsigned int)sciport->atr_timeout_cnt);
        asc_out (port, TIMEOUT, 254);  //sci
	if (--sciport->atr_timeout_cnt == 0 ){
        	asc_out (port, TIMEOUT, 254);
		asc_out(port, INTEN, asc_in(port,INTEN) & ~ASC_INTEN_TOI);
		if (!sciport->isatr){
			do_atr(port);
		} else {
			unsigned char *rb, *fb;
			int l = kfifo_len(sciport->sendq);
			unsigned char  db[l]; //drop buffer
			kfifo_get(sciport->sendq, db, l); //drop send fifo
			kfifo_get(sciport->receq, db, l); //drop echoed part of receive fifo
			kfifo_get(sciport->flagq, db, l); //drop echoed part of receive fifo
			l = kfifo_len(sciport->receq);
			tty_prepare_flip_string_flags(port->info->tty, &rb, (char **)&fb, l);
			kfifo_get(sciport->receq, rb, l);
			kfifo_get(sciport->flagq, fb, l);
			printk(KERN_INFO "SW:   %s (%d)\n", sci_hexdump(1,rb,l), l );
//			printk(KERN_INFO "SW:   %s (%d)\n", sci_hexdump(1,fb,l), l );
			sci_flush_tty(port);
		}
//		wake_up_interruptible(&sciport->initwq);
		return;
	}
}

#ifdef STM22
static irqreturn_t sci_interrupt(int irq, void *ptr, struct pt_regs *regs)
#else
static irqreturn_t sci_interrupt(int irq, void *ptr)
#endif
{
	struct uart_port *port = ptr;
//	struct sci_port *sciport = to_sci_port(port);
	unsigned long status;
	unsigned long inten;

	spin_lock(&port->lock);
	
	status = asc_in (port, STA);
	inten = asc_in(port, INTEN);
	
//	if ((status & ASC_STA_TNE) && (inten & ASC_INTEN_TNE) ||
//	    (status & ASC_STA_TOI) && (inten & ASC_INTEN_TOI) ){
	if ( status & ASC_STA_TOI ){
		sci_timeout(port);
	}
//	if (status & ASC_STA_RBF) {
	if (status & (ASC_STA_RBF | ASC_STA_RHF)) {
		/* Receive FIFO not empty */
#ifdef STM22
		sci_receive_chars(port, regs);
#else
		sci_receive_chars(port);
#endif
//		if (!asc_in(port, INTEN) | (ASC_INTEN_TNE | ASC_INTEN_TOI))
//			if (!sciport->isatr){
//				printk(KERN_DEBUG "Enable timeout interrupt\n");
//				asc_out(port, INTEN, asc_in(port, INTEN) | (ASC_INTEN_TNE | ASC_INTEN_TOI) );
//			}
	}

//	if ((status & ASC_STA_THE) && (asc_in(port, INTEN) & ASC_INTEN_THE)) {
	if ((status & ASC_STA_TE) && (asc_in(port, INTEN) & ASC_INTEN_TE)) {
		/* Transmitter FIFO at least half empty */
		sci_transmit_chars(port);
	}

	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}


static int sci_set_baud (struct uart_port *port, int baud)
{
        unsigned int t;
//        struct clk *clk;
        unsigned long rate;

        rate = port->uartclk;

        if (baud < 19200) {
                t = BAUDRATE_VAL_M0(baud, rate);
                asc_out (port, BAUDRATE, t);
                return 0;
        } else {
                t = BAUDRATE_VAL_M1(baud, rate);
                asc_out (port, BAUDRATE, t);
                return ASC_CTL_BAUDMODE;
        }
}

static int sci_set_modes(struct uart_port *port, struct sci_modes __user *modes)
{
	struct sci_modes m;
        if (copy_from_user(&m, modes, sizeof(*modes)))
                return -EFAULT;
	printk(KERN_DEBUG "mode.emv2000: 0x%04x\n", m.emv2000);
	printk(KERN_DEBUG "mode.dma:     0x%04x\n", m.dma);
	printk(KERN_DEBUG "mode.man_act: 0x%04x\n", m.man_act);
	printk(KERN_DEBUG "mode.rw_mode: 0x%04x\n", m.rw_mode);
	return 0;
}

static int sci_get_modes(struct uart_port *port, struct sci_modes __user *modes)
{
	struct sci_modes m;
	m.emv2000 = 0x11;
	m.dma     = 0x22;
	m.man_act = 0x33;
	m.rw_mode = 0x44;
        if (copy_to_user(modes, &m, sizeof(*modes)))
                return -EFAULT;
	return 0;
}

static int sci_set_parameters(struct uart_port *port, struct sci_parameters __user *parameters)
{
	struct sci_parameters p;
        if (copy_from_user(&p, parameters, sizeof(*parameters)))
                return -EFAULT;
        printk(KERN_DEBUG "param.T:       0x%04x\n", (unsigned int)p.T);
        printk(KERN_DEBUG "param.fs:      0x%08x\n", (unsigned int)p.fs);
        printk(KERN_DEBUG "param.ETU:     0x%04x\n", (unsigned int)p.ETU);
        printk(KERN_DEBUG "param.WWT:     0x%04x\n", (unsigned int)p.WWT);
        printk(KERN_DEBUG "param.BWT:     0x%04x\n", (unsigned int)p.BWT);
        printk(KERN_DEBUG "param.CWT:     0x%04x\n", (unsigned int)p.CWT);
        printk(KERN_DEBUG "param.EGT:     0x%04x\n", (unsigned int)p.EGT);
        printk(KERN_DEBUG "param.clock_stop_polarity:     0x%04x\n", (unsigned int)p.clock_stop_polarity);
        printk(KERN_DEBUG "param.check:   0x%04x\n", p.check);
        printk(KERN_DEBUG "param.P:       0x%04x\n", p.P);
        printk(KERN_DEBUG "param.I:       0x%04x\n", p.I);
        printk(KERN_DEBUG "param.U:       0x%04x\n", p.U);
	return 0;
}

static int sci_get_parameters(struct uart_port *port, struct sci_parameters __user *parameters)
{
	struct sci_parameters p;
	p.T		= 0x11;
	p.fs		= 0x22;
	p.ETU		= 0x3333;
	p.WWT		= 0x4444;
	p.BWT		= 0x5555;
	p.CWT		= 0x6666;
	p.EGT		= 0x7777;
	p.clock_stop_polarity	= 0x88;
	p.check		= 0x99;
	p.P		= 0xaa;
	p.I		= 0xbb;
	p.U		= 0xcc;
        if (copy_to_user(parameters, &p, sizeof(*parameters)))
                return -EFAULT;
	return 0;
}

static int sci_ioctl_reset(struct uart_port *port)
{
	sci_disable(port);
	sci_shutdown(port);
	msleep(10);
	sci_startup(port);
	return 0;
}

static int sci_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg ){
	int ret = 0;
	struct sci_port *sciport = to_sci_port(port);

	/*
	 * All these rely on hardware being present and need to be
	 * protected against the tty being hung up.
	 */
	switch (cmd) {
	case IOCTL_SET_RESET: /* Set reset */
		printk(KERN_DEBUG "received IOCTL_SET_RESET\n");
//		ret = sci_ioctl_reset(port);
		break;

	case IOCTL_SET_CLOCK_START: /* Start clock */
		printk(KERN_DEBUG "received IOCTL_SET_CLOCK_START\n");
		break;

	case IOCTL_SET_CLOCK_STOP: /* Stop clock */
		printk(KERN_DEBUG "received IOCTL_SET_CLOCK_STOP\n");
		break;

	case IOCTL_GET_IS_CARD_PRESENT: /* Get present card status */
//		printk(KERN_DEBUG "received IOCTL_GET_IS_CARD_PRESENT\n");
		{
			unsigned long present = sciport->iscard;
			copy_to_user(arg,&present,sizeof(arg));
		}
		break;

	case IOCTL_GET_IS_CARD_ACTIVATED: /* Get activated card status */
		printk(KERN_DEBUG "received IOCTL_GET_IS_CARD_ACTIVATED\n");
		break;

	case IOCTL_SET_DEACTIVATE: /* Get deactivate card */
		printk(KERN_DEBUG "received IOCTL_SET_DEACTIVATE\n");
		break;

	case IOCTL_SET_ATR_READY: /* Set ATR ready status */
		{
			unsigned long ready = sciport->isatr;
			printk(KERN_DEBUG "received IOCTL_SET_ATR_READY\n");
			copy_to_user(arg,&ready,sizeof(arg));
		}
		break;

	case IOCTL_GET_ATR_STATUS: /* Get ATR status */
		printk(KERN_DEBUG "received IOCTL_GET_ATR_STATUS\n");
		break;

	case IOCTL_GET_MODES: /* Get modes */
		printk(KERN_DEBUG "received IOCTL_GET_MODES\n");
		ret = sci_get_modes(port, (struct sci_modes *)arg);
		break;

	case IOCTL_SET_PARAMETERS: /* Set parameters */
		printk(KERN_DEBUG "received IOCTL_SET_PARAMETERS\n");
		ret = sci_set_parameters(port, (struct sci_parameters *)arg);
		break;

	case IOCTL_GET_PARAMETERS: /* Get parameters */
		printk(KERN_DEBUG "received IOCTL_GET_PARAMETERS\n");
		ret = sci_get_parameters(port, (struct sci_parameters *)arg);
		break;

	default: {
		ret = -ENOIOCTLCMD;
		break;
	}
	}

	return ret;
}

static char *sci_hexdump(int m, unsigned char *buf, size_t n)
{
	int i;
	static char dump[520];

	dump[i=0]='\0';
	m=(m)?3:2;
	if (m*n>=(int)sizeof(dump))
		n=(sizeof(dump)/m)-1;
	while (i++<n)
		sprintf(dump+(m*(i-1)), "%02x%s", *buf++, (m>2 && i<n)?" ":"");
	return dump;
}


static inline void bits_set(struct cfg_bits *b)
{
	writel(readl(b->addr) | b->mask, b->addr);
}

static inline void bits_clear(struct cfg_bits *b)
{
	writel(readl(b->addr) & ~b->mask, b->addr);
}

static inline void bits_set_val(struct cfg_bits *b, unsigned long val)
{
	writel((readl(b->addr) & ~b->mask) | (val & b->mask), b->addr);
}

static void syscfg_enable_scg_clock(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);
#ifdef STM22
	bits_set(&sciport->clksrc);
#else
	sysconf_write(sciport->clksrc.field, 1);
#endif
}

static void scg_enable_clock( struct uart_port *port )
{
	struct sci_port *sciport = to_sci_port(port);
	bits_set_val(&sciport->scgclkval, 0x0000000e);	//FIXME
	bits_set(&sciport->scgclken);
	printk(KERN_DEBUG "SCG%d clock enabled\n", port->line);
}

static void scg_disable_clock( struct uart_port *port )
{
	struct sci_port *sciport = to_sci_port(port);
	bits_clear(&sciport->scgclken);
	printk(KERN_DEBUG "SCG%d clock disabled\n", port->line);
}

static void syscfg_disable_auto_vcc(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);
#ifdef STM22
	bits_clear(&sciport->autovcc);
	bits_clear(&sciport->scdetpol);
#else
	sysconf_write(sciport->autovcc.field, 0);
	sysconf_write(sciport->scdetpol.field, 0);
#endif
}

static void syscfg_enable_auto_vcc(struct uart_port *port)
{
	struct sci_port *sciport = to_sci_port(port);
#ifdef STM22
	bits_set(&sciport->autovcc);
	bits_set(&sciport->scdetpol);
#else
	sysconf_write(sciport->autovcc.field, 1);
	sysconf_write(sciport->scdetpol.field, 1);
#endif
}


static int sci_request_irq(struct uart_port *port)
{
#ifdef STM22
        if (request_irq(port->irq, sci_interrupt, SA_INTERRUPT,
                        "stsci", port)) {
#else
        if (request_irq(port->irq, sci_interrupt, IRQF_DISABLED,
                        "stsci", port)) {
#endif
                printk(KERN_ERR "STSCI: cannot allocate irq (%d).\n",port->irq);
                return -ENODEV;
        }
        return 0;
}

static void sci_free_irq(struct uart_port *port)
{
        free_irq(port->irq, port);
}


static int sci_pios_init(struct uart_port *port){
	
	struct sci_port *sciport = to_sci_port(port);
	int i;

	for( i=0;i<sciport->pios_nr; i++){
		struct sci_pio *pio = &(sciport->pios[i]);
		if (!pio->name)
			continue;
		pio->pio = stpio_request_pin(pio->port, pio->pin, pio->name, pio->dir);
		if (pio->pio){
			printk(KERN_DEBUG "STSCI pio[%d,%d] \"%s\" allocated\n", pio->port,pio->pin,pio->name);
		} else {
			printk(KERN_ERR "STSCI pio %s allocation failed\n", pio->name);
			return -EBUSY;
		}
		if (pio->handler != 0)
#ifdef STM22 
			stpio_request_irq(pio->pio, !stpio_get_pin(pio->pio), pio->handler, (void *)port);
#else
			stpio_flagged_request_irq(pio->pio, !stpio_get_pin(pio->pio), pio->handler, (void *)port, 2);
#endif
	}
	
	return 0;
}

static int sci_pios_free(struct uart_port *port){
	
	struct sci_port *sciport = to_sci_port(port);
	int i;

	printk(KERN_DEBUG "STSCI pins released\n");
	for( i=0;i<sciport->pios_nr; i++){
		struct sci_pio *pio = &(sciport->pios[i]);
		if (!pio->name)
			continue;
		if (pio->handler != NULL)
			stpio_free_irq(pio->pio);
		if (pio->pio != NULL)
			stpio_free_pin(pio->pio);
	}
	return 0;
}

static void sci_tty_config(struct tty_driver *tty_driver)
{
#ifdef STM22
	struct termios *ios = &tty_driver->init_termios;
#else
	struct ktermios *ios = &tty_driver->init_termios;
#endif
	/* Selects raw (non-canonical) input and output
	   simple glibc cfmakeraw macro */
	ios->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	ios->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	ios->c_oflag &= ~OPOST;
}

static int __init sci_init(void)
{
	int line, ret;
        struct clk *clk;
        unsigned long rate;

#ifdef STM22 
        clk = clk_get("comms_clk");
        if (IS_ERR(clk)) clk = clk_get("bus_clk");
#else
        clk = clk_get(NULL,"comms_clk");
        if (IS_ERR(clk)) clk = clk_get(NULL,"bus_clk");
#endif
        rate = clk_get_rate(clk);
	printk(KERN_DEBUG "Rate: %d\n", (int)rate);

	ret = uart_register_driver(&sci_uart_driver);
	sci_tty_config(sci_uart_driver.tty_driver);
	if (ret == 0) {
//		for (line=0; line<SCI_NPORTS; line++) {
		for (line=0; line<1; line++) {
			struct sci_port *sciport = &sci_ports[line];
			sciport->port.uartclk = rate;
			uart_add_one_port(&sci_uart_driver, &sciport->port);
		}
	}

	printk(KERN_INFO "STSCI registered\n");

	return ret;

}

static void __exit sci_exit(void)
{
	int line;

//	for (line = 0; line < SCI_NPORTS; line++)
	for (line = 0; line < 1; line++){
		struct uart_port *port = &(sci_ports[line]).port;
		uart_remove_one_port(&sci_uart_driver, port);
		sci_pios_free(port);
		sci_free_irq(port);
	}
	uart_unregister_driver(&sci_uart_driver);

 	printk(KERN_INFO "STSCI unregistered.\n");
        return;
}

module_init(sci_init);
module_exit(sci_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robert Demski <demsey@users.sourceforge.net>");
MODULE_DESCRIPTION("Open ST40 SmartCard Interface (SCI) driver");
