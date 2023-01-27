/* sc16is7xx.c - SC16IS7XX serial driver */

#define DT_DRV_COMPAT nxp_sc16is7xx

/*
 * Copyright (c) 2022 Opito.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief SC16IS7XX Serial Driver
 *
 * This is the driver for the NXP SC16IS7XX I2C UART Chip 
 *
 * Before individual UART port can be used, sc16is7xx_port_init() has to be
 * called to setup the port.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/types.h>

#include <zephyr/init.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include "sc16is7xx.h"
#include <zephyr/drivers/gpio.h>


LOG_MODULE_REGISTER(sc16is7xx, CONFIG_SC16IS7XX_LOG_LEVEL);

/* register definitions */

#define REG_THR 0x00  /* Transmitter holding reg.       */
#define REG_RDR 0x00  /* Receiver data reg.             */
#define REG_BRDL 0x00 /* Baud rate divisor (LSB)        */
#define REG_BRDH 0x01 /* Baud rate divisor (MSB)        */
#define REG_IER 0x01  /* Interrupt enable reg.          */
#define REG_IIR 0x02  /* Interrupt ID reg.              */
#define REG_FCR 0x02  /* FIFO control reg.              */
#define REG_LCR 0x03  /* Line control reg.              */
#define REG_MCR 0x04  /* Modem control reg.             */
#define REG_LSR 0x05  /* Line status reg.               */
#define REG_MSR 0x06  /* Modem status reg.              */
#define REG_SPR 0x07  /* Scratch Pad Register			*/
#define REG_EFR 0x02  /* Enhanced Features Register     */
#define REG_TXLVL 0x08
#define REG_RXLVL 0x09
#define REG_IODIR 0x0a
#define REG_IOSTATE 0x0b
#define REG_IOINTENA 0x0c
#define REG_IOCONTROL 0x0e
#define REG_EFCR 0x0f


/* equates for interrupt enable register */

#define IER_RXRDY 0x01 /* receiver data ready */
#define IER_TBE 0x02   /* transmit bit enable */
#define IER_LSR 0x04   /* line status interrupts */
#define IER_MSI 0x08   /* modem status interrupts */

/* equates for interrupt identification register */

#define IIR_MSTAT 0x00 /* modem status interrupt  */
#define IIR_NIP   0x01 /* no interrupt pending    */
#define IIR_THRE  0x02 /* transmit holding register empty interrupt */
#define IIR_RBRF  0x04 /* receiver buffer register full interrupt */
#define IIR_LS    0x06 /* receiver line status interrupt */
#define IIR_MASK  0x07 /* interrupt id bits mask  */
#define IIR_ID    0x06 /* interrupt ID mask without NIP */
#define IIR_FE    0xC0 /* FIFO mode enabled */
#define IIR_CH    0x0C /* Character timeout*/

/* equates for FIFO control register */

#define FCR_FIFO 0x01    /* enable XMIT and RCVR FIFO */
#define FCR_RCVRCLR 0x02 /* clear RCVR FIFO */
#define FCR_XMITCLR 0x04 /* clear XMIT FIFO */

/* RCVR FIFO interrupt levels: trigger interrupt with this bytes in FIFO */
#define FCR_FIFO_8 0x00  /* 8 byte in RCVR FIFO */
#define FCR_FIFO_16 0x40  /* 16 bytes in RCVR FIFO */
#define FCR_FIFO_56 0x80  /* 56 bytes in RCVR FIFO */
#define FCR_FIFO_60 0xC0 /*  60 bytes in RCVR FIFO */
#define FCR_FIFO_64 0xC0 /* Enable 64 bytes FIFO */

/* FCR register bits */
#define SC16IS7XX_FCR_FIFO_BIT		(1 << 0) /* Enable FIFO */
#define SC16IS7XX_FCR_RXRESET_BIT	(1 << 1) /* Reset RX FIFO */
#define SC16IS7XX_FCR_TXRESET_BIT	(1 << 2) /* Reset TX FIFO */
#define SC16IS7XX_FCR_RXLVLL_BIT	(1 << 6) /* RX Trigger level LSB */
#define SC16IS7XX_FCR_RXLVLH_BIT	(1 << 7) /* RX Trigger level MSB */

/* FCR register bits - write only if (EFR[4] == 1) */
#define SC16IS7XX_FCR_TXLVLL_BIT	(1 << 4) /* TX Trigger level LSB */
#define SC16IS7XX_FCR_TXLVLH_BIT	(1 << 5) /* TX Trigger level MSB */


/* GPIO serial mode bits */
#define CHANNEL_A_HALF_FULL   	 (1 << 4)		//GPIO4 (half= 1 full=0) this is the rs422/rs485 mode select
#define CHANNEL_A_RE   	   		 (1 << 5)	    //GPIO5 (recive_enable = 0 ) rs485 ~re the de is on the RTS pin
#define CHANNEL_A_RS232_RS485 	 (1 << 7)		//GPIO7 (rs232 = 0, rs485 = 1)


#define  CHANNEL_B_HALF_FULL     (1 << 0)		//GPIO0 (half= 1 full=0) this is the rs422/rs485 mode select
#define  CHANNEL_B_RE   	     (1 << 1)	    //GPIO1 (recive_enable = 0  rs485 ~re the de is on the RTS pin
#define  CHANNEL_B_RS232_RS485   (1 << 3)		//GPIO7 (rs232 = 0, rs485 = 1)


#define  CHANNEL_A_B_SHUTDOWN    (1 << 2)		//GPIO2 (shutdown = 0, external 10k pullup on this to make sure it is active)
#define  CHANNEL_A_B_SLEW  	     (1 << 6)		//GPIO6 (slew = 0, external 10k pullup on this to make sure it is active)


#define  CHANNEL_A_MASK 		 (CHANNEL_A_HALF_FULL | CHANNEL_A_RE | CHANNEL_A_RS232_RS485 | CHANNEL_A_B_SHUTDOWN | CHANNEL_A_B_SLEW)
#define  CHANNEL_B_MASK 		 (CHANNEL_B_HALF_FULL | CHANNEL_B_RE | CHANNEL_B_RS232_RS485 | CHANNEL_A_B_SHUTDOWN | CHANNEL_A_B_SLEW)

#define  SET_CHANNEL_A_RS232	 ((CHANNEL_A_B_SLEW | CHANNEL_A_B_SHUTDOWN) & ~CHANNEL_A_RS232_RS485)
#define  SET_CHANNEL_B_RS232	 ((CHANNEL_A_B_SLEW | CHANNEL_A_B_SHUTDOWN) & ~CHANNEL_B_RS232_RS485)

#define  SET_CHANNEL_A_RS485	 ((CHANNEL_A_B_SLEW | CHANNEL_A_B_SHUTDOWN | CHANNEL_A_RS232_RS485 | CHANNEL_A_HALF_FULL) | CHANNEL_A_RE) 	//this will still need the /re and the de set 
#define  SET_CHANNEL_B_RS485	 ((CHANNEL_A_B_SLEW | CHANNEL_A_B_SHUTDOWN | CHANNEL_B_RS232_RS485 | CHANNEL_B_HALF_FULL) & ~CHANNEL_B_RE) 	//this will still need the /re and the de set  	//this will still need the /re and the de set 

#define  SET_CHANNEL_A_RS422	 ((CHANNEL_A_B_SLEW | CHANNEL_A_B_SHUTDOWN | CHANNEL_A_RS232_RS485) & ~CHANNEL_A_HALF_FULL & ~CHANNEL_A_RE) 	//this will still need the de set 
#define  SET_CHANNEL_B_RS422	 ((CHANNEL_A_B_SLEW | CHANNEL_A_B_SHUTDOWN | CHANNEL_B_RS232_RS485) & ~CHANNEL_B_HALF_FULL & ~CHANNEL_B_RE) 	//this will still need the de set 





//MCR reg bit definitions. 
#define RS422_RTS_DIRECTION   (1 << 1)	 //bit 1 of the MCR_REG is to be clear for RTS_Direction to be always high. 


//EFCR reg bit defintions
#define RS485_RTS_DIRECTION   (1 << 4)   //bit 4 of the EFCR_REG is to be set for auto RTS_DIRECTION 
#define RS485_RTS_INVERSION   (1 << 5)	 //bit 5 of the EFCR REG is to be set for RTS = 1 during transmission  as this is wired to the DE pin. 




/* constants for line control register */

#define LCR_CS5 0x00   /* 5 bits data size */
#define LCR_CS6 0x01   /* 6 bits data size */
#define LCR_CS7 0x02   /* 7 bits data size */
#define LCR_CS8 0x03   /* 8 bits data size */
#define LCR_2_STB 0x04 /* 2 stop bits */
#define LCR_1_STB 0x00 /* 1 stop bit */
#define LCR_PEN 0x08   /* parity enable */
#define LCR_PDIS 0x00  /* parity disable */
#define LCR_EPS 0x10   /* even parity select */
#define LCR_SP 0x20    /* stick parity select */
#define LCR_SBRK 0x40  /* break control bit */
#define LCR_DLAB 0x80  /* divisor latch access enable */
#define LCR_EFR_ACCESS 0xBF  /* Enhanced features access enable */

/* constants for the modem control register */
#define MCR_DTR_BIT		(1 << 0) /* DTR complement
						  * - only on 75x/76x
						  */
#define MCR_RTS_BIT		(1 << 1) /* RTS complement */
#define MCR_TCRTLR_BIT	(1 << 2) /* TCR/TLR register enable */
#define MCR_LOOP_BIT	(1 << 4) /* Enable loopback test mode */
#define MCR_XONANY_BIT	(1 << 5) /* Enable Xon Any
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define MCR_IRDA_BIT	(1 << 6) /* Enable IrDA mode
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define MCR_CLKSEL_BIT	(1 << 7) /* Divide clock by 4
						  * - write enabled
						  * if (EFR[4] == 1)
						  */


/* constants for line status register */

#define LSR_RXRDY 0x01 /* receiver data available */
#define LSR_OE 0x02    /* overrun error */
#define LSR_PE 0x04    /* parity error */
#define LSR_FE 0x08    /* framing error */
#define LSR_BI 0x10    /* break interrupt */
#define LSR_EOB_MASK 0x1E /* Error or Break mask */
#define LSR_THRE 0x20  /* transmit holding register empty */
#define LSR_TEMT 0x40  /* transmitter empty */

/* constants for modem status register */

#define MSR_DCTS 0x01 /* cts change */
#define MSR_DDSR 0x02 /* dsr change */
#define MSR_DRI 0x04  /* ring change */
#define MSR_DDCD 0x08 /* data carrier change */
#define MSR_CTS 0x10  /* complement of cts */
#define MSR_DSR 0x20  /* complement of dsr */
#define MSR_RI 0x40   /* complement of ring signal */
#define MSR_DCD 0x80  /* complement of dcd */




/* EFCR register bits */
#define EFCR_9BIT_MODE_BIT	(1 << 0) /* Enable 9-bit or Multidrop
						  * mode (RS485) */
#define EFCR_RXDISABLE_BIT	(1 << 1) /* Disable receiver */
#define EFCR_TXDISABLE_BIT	(1 << 2) /* Disable transmitter */
#define EFCR_AUTO_RS485_BIT	(1 << 4) /* Auto RS485 RTS direction */
#define EFCR_RTS_INVERT_BIT	(1 << 5) /* RTS output inversion */
#define EFCR_IRDA_MODE_BIT	(1 << 7) /* IrDA mode
						  * 0 = rate upto 115.2 kbit/s
						  *   - Only 750/760
						  * 1 = rate upto 1.152 Mbit/s
						  *   - Only 760
						  */

/* EFR register bits */
#define EFR_AUTORTS_BIT	(1 << 6) /* Auto RTS flow ctrl enable */
#define EFR_AUTOCTS_BIT	(1 << 7) /* Auto CTS flow ctrl enable */
#define EFR_XOFF2_DETECT_BIT	(1 << 5) /* Enable Xoff2 detection */
#define EFR_ENABLE_BIT	(1 << 4) /* Enable enhanced functions
						  * and writing to IER[7:4],
						  * FCR[5:4], MCR[7:5]
						  */
#define EFR_SWFLOW3_BIT	(1 << 3) /* SWFLOW bit 3 */
#define EFR_SWFLOW2_BIT	(1 << 2) /* SWFLOW bit 2
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no transmitter flow
						  *       control
						  * 01 -> transmitter generates
						  *       XON2 and XOFF2
						  * 10 -> transmitter generates
						  *       XON1 and XOFF1
						  * 11 -> transmitter generates
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */
#define EFR_SWFLOW1_BIT	(1 << 1) /* SWFLOW bit 2 */
#define EFR_SWFLOW0_BIT	(1 << 0) /* SWFLOW bit 3
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no received flow
						  *       control
						  * 01 -> receiver compares
						  *       XON2 and XOFF2
						  * 10 -> receiver compares
						  *       XON1 and XOFF1
						  * 11 -> receiver compares
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */
#define EFR_FLOWCTRL_BITS	(EFR_AUTORTS_BIT | \
					EFR_AUTOCTS_BIT | \
					EFR_XOFF2_DETECT_BIT | \
					EFR_SWFLOW3_BIT | \
					EFR_SWFLOW2_BIT | \
					EFR_SWFLOW1_BIT | \
					EFR_SWFLOW0_BIT)




#define THR(dev) (REG_THR << reg_interval(dev)| get_port(dev))
#define RDR(dev) (REG_RDR << reg_interval(dev)| get_port(dev))
#define BRDL(dev)(REG_BRDL << reg_interval(dev)| get_port(dev))
#define BRDH(dev)(REG_BRDH << reg_interval(dev)| get_port(dev))
#define IER(dev) (REG_IER << reg_interval(dev)| get_port(dev))
#define IIR(dev) (REG_IIR << reg_interval(dev)| get_port(dev))
#define FCR(dev) (REG_FCR << reg_interval(dev)| get_port(dev))
#define LCR(dev) (REG_LCR << reg_interval(dev)| get_port(dev))
#define MCR(dev) (REG_MCR << reg_interval(dev)| get_port(dev))
#define LSR(dev) (REG_LSR << reg_interval(dev)| get_port(dev))
#define MSR(dev) (REG_MSR << reg_interval(dev)| get_port(dev))
#define SPR(dev) (REG_SPR << reg_interval(dev) | get_port(dev))
#define EFR(dev) (REG_EFR << reg_interval(dev) | get_port(dev))
#define TXLVL(dev) REG_TXLVL << reg_interval(dev)
#define RXLVL(dev) REG_RXLVL << reg_interval(dev)
#define IODIR(dev) REG_IODIR << reg_interval(dev)
#define IOSTATE(dev) REG_IOSTATE << reg_interval(dev)
#define IOINTENA(dev) REG_IOINTENA << reg_interval(dev)
#define IOCONTROL(dev) REG_IOCONTROL << reg_interval(dev)
#define EFCR(dev) REG_EFCR << reg_interval(dev)


#define IIRC(dev) (((struct sc16is7xx_dev_data *)(dev)->data)->iir_cache)

int sc16is17xx_reg_read(const struct device *dev, uint8_t reg);
int sc16is17xx_reg_write(const struct device *dev, uint8_t reg, uint8_t val);
static void sc16is7xx_isr(const struct device *dev);
static int sc16is7xx_irq_update(const struct device *dev);

#define INBYTE(dev, x) sc16is17xx_reg_read(dev, x)
#define OUTBYTE(dev, x, d) sc16is17xx_reg_write(dev, x, d)


/** @brief Parity modes */
//this could do with an ifdef around it for other users that don;t have our board fitted with sp330.
enum uart_serial_mode {
	UART_MODE_RS232,
	UART_MODE_RS485,
	UART_MODE_RS422
};


/* device config */
struct sc16is7xx_device_config {
	uint32_t port;
	uint8_t serport_num; 
	uint8_t serial_mode;
	struct i2c_dt_spec bus;
	uint32_t sys_clk_freq;
#if defined(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN) 
	struct gpio_dt_spec int_gpio;
#endif
	uint8_t reg_interval;
};

/** Device data structure */
struct sc16is7xx_dev_data {
	struct uart_config uart_config;
	struct k_spinlock lock;
	uint8_t fifo_size;


#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
	/* Self-reference to the driver instance */
	const struct device *instance;
	struct gpio_callback gpio_callback;
	struct k_work interrupt_worker;
	bool interrupt_active;
	uint8_t iir_cache;						/**< cache of IIR since it clears when read */
	uart_irq_callback_user_data_t cb;  		/**< Callback function pointer */
	void *cb_data;							/**< Callback function arg */
#endif

#if defined(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN) 
	bool tx_stream_on;
#endif
};



#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN


static void sc16is7xx_interrupt_worker(struct k_work *work)
{
	struct sc16is7xx_dev_data * const drv_data = CONTAINER_OF(
	work, struct sc16is7xx_dev_data, interrupt_worker);
	int ret;

 		//something happened or we wouldn't be here. lets see what it is.
		//the user is supposed to call this function first thing in their callback,
		//sc16is7xx_irq_update(drv_data->instance);
		sc16is7xx_isr(drv_data->instance);
		
}




static void sc16is7xx_interrupt_callback(const struct device *dev,
					    struct gpio_callback *cb,
					    gpio_port_pins_t pins)
{
	struct sc16is7xx_dev_data * const drv_data =
		CONTAINER_OF(cb, struct sc16is7xx_dev_data, gpio_callback);

	ARG_UNUSED(pins);

	/* Cannot read sc16is7xx registers from ISR context, queue worker */
	k_work_submit(&drv_data->interrupt_worker);
}


#endif



int sc16is17xx_reg_read(const struct device *dev, uint8_t reg){
	const struct sc16is7xx_device_config *const config = dev->config;
	uint8_t data;
	i2c_burst_read_dt(&config->bus, reg, &data,1);
	return data;
}


int sc16is17xx_reg_write(const struct device *dev, uint8_t reg, uint8_t val){
	const struct sc16is7xx_device_config *const config = dev->config;
	
	uint8_t tx_buf[2];
	tx_buf[0] = reg;
	tx_buf[1] = val;
	
	return i2c_write_dt(&config->bus, tx_buf, sizeof(tx_buf));
}


static inline uint8_t reg_interval(const struct device *dev){
	const struct sc16is7xx_device_config *config = dev->config;
	return config->reg_interval;
}


static const struct uart_driver_api sc16is7xx_driver_api;

static inline uintptr_t get_port(const struct device *dev){
	const struct sc16is7xx_device_config *config = dev->config;
	
	return config->serport_num << 1;
}


static void set_reset(const struct device *dev){
	OUTBYTE(dev,IOCONTROL(dev),0x08);// 
}


static void set_baud_rate(const struct device *dev, uint32_t baud_rate, uint32_t pclk)
{
	struct sc16is7xx_dev_data * const dev_data = dev->data;
	uint32_t divisor; /* baud rate divisor */
	uint8_t lcr_cache;
	
	if ((baud_rate != 0U) && (pclk != 0U)) {

		divisor = pclk /(baud_rate *16);

		/* set the DLAB to access the baud rate divisor registers */
		
		lcr_cache = INBYTE(dev,LCR(dev));
		
		OUTBYTE(dev,LCR(dev),LCR_DLAB);// magic number to program baud rate 

		OUTBYTE(dev,BRDL(dev), (unsigned char)(divisor & 0xff));
		OUTBYTE(dev,BRDH(dev), (unsigned char)((divisor >> 8) & 0xff));

		//magic number to unlock ench reg access
		OUTBYTE(dev,LCR(dev), LCR_EFR_ACCESS);
		OUTBYTE(dev,EFR(dev), 0x00); //no software flow control yet. 

		/* restore the DLAB to access the baud rate divisor registers */
		OUTBYTE(dev,LCR(dev), lcr_cache);

		dev_data->uart_config.baudrate = baud_rate;
  	
	
	}
}

//this could do with an ifdef around it for other users that don;t have our board fitted with sp330.
static void set_receive_enable_low(const struct device *dev, bool state){
	const struct sc16is7xx_device_config * const dev_cfg = dev->config;
	uint8_t current_reg_val = 0;
	current_reg_val = INBYTE(dev,REG_IOSTATE);

	if (dev_cfg->serport_num == 0){
		if (state == true) { //if we are true then lower RE
			current_reg_val &= ~CHANNEL_A_RE;
		}
		else {				//else raise it. 
			current_reg_val |= CHANNEL_A_RE;
		}
	}
	else if (dev_cfg->serport_num == 1){
		if (state == true) { //if we are true then lower RE
			current_reg_val &= ~CHANNEL_B_RE;
		}
		else {				//else raise it. 
			current_reg_val |= CHANNEL_B_RE;
		}
	}
	OUTBYTE(dev, REG_IOSTATE, current_reg_val);
}


//this could do with an ifdef around it for other users that don;t have our board fitted with sp330.
static void set_serial_mode (const struct device *dev){
	const struct sc16is7xx_device_config * const dev_cfg = dev->config;
	uint8_t io_cache = 0; //read the IO before messing it up.....
	//set the GPIO direction to all outputs
	OUTBYTE(dev,IOCONTROL(dev),0x00);
	OUTBYTE(dev,IODIR(dev)	 ,0xFF);

	io_cache = INBYTE(dev,IOSTATE(dev));
	LOG_DBG("GPIO before write value = 0x%02X", io_cache);
	
	if (dev_cfg->serport_num ==0){
		io_cache &= ~CHANNEL_A_MASK; 	//clear all bits associated with channel 0
		switch (dev_cfg->serial_mode){
		case UART_MODE_RS485:
			io_cache |= SET_CHANNEL_A_RS485;
			OUTBYTE(dev,IOSTATE(dev), io_cache);
			io_cache = INBYTE(dev,EFCR(dev));
			io_cache |= RS485_RTS_DIRECTION;
			io_cache |= RS485_RTS_INVERSION;
			OUTBYTE(dev, EFCR(dev),io_cache);
			LOG_DBG("setmode  ch=%d mode=%s reg=%d value=%d", dev_cfg->serport_num, "RS485", REG_EFCR, io_cache);
			io_cache = INBYTE(dev,EFCR(dev));
			LOG_DBG("getmode  ch=%d mode=%s reg=%d value=%d", dev_cfg->serport_num, "RS485", REG_EFCR, io_cache);
			set_receive_enable_low(dev,true); //this is definitely needed to be able to receive on RS485
			break;
		case UART_MODE_RS422:
			io_cache |= SET_CHANNEL_A_RS422;
			OUTBYTE(dev,IOSTATE(dev), io_cache);
			io_cache = INBYTE(dev,MCR(dev));
			io_cache &= ~RS422_RTS_DIRECTION;
			OUTBYTE(dev, MCR(dev), io_cache);
			break;
		case UART_MODE_RS232:
		default:
			io_cache |= SET_CHANNEL_A_RS232;
			OUTBYTE(dev,IOSTATE(dev), io_cache);
		}
	}

	else if (dev_cfg->serport_num ==1){
		io_cache &= ~CHANNEL_B_MASK; 	//clear all bits associated with channel 0
		switch (dev_cfg->serial_mode){
		case UART_MODE_RS485:
			io_cache |= SET_CHANNEL_B_RS485;
			OUTBYTE(dev,IOSTATE(dev), io_cache);
			io_cache = INBYTE(dev,EFCR(dev));
			io_cache |= RS485_RTS_DIRECTION;
			io_cache |= RS485_RTS_INVERSION;
			OUTBYTE(dev, EFCR(dev),io_cache);
			LOG_DBG("setmode  ch=%d mode=%s reg=%d value=%d", dev_cfg->serport_num, "RS485", REG_EFCR, io_cache);
			io_cache = INBYTE(dev,EFCR(dev));
			LOG_DBG("getmode  ch=%d mode=%s reg=%d value=%d", dev_cfg->serport_num, "RS485", REG_EFCR, io_cache);
			set_receive_enable_low(dev,true); //this is definitely needed to be able to receive on RS485
			break;
		case UART_MODE_RS422:
			io_cache |= SET_CHANNEL_B_RS422;
			OUTBYTE(dev,IOSTATE(dev), io_cache);
			io_cache = INBYTE(dev,MCR(dev));
			io_cache &= ~RS422_RTS_DIRECTION;
			OUTBYTE(dev, MCR(dev), io_cache);
			break;
		case UART_MODE_RS232:
		default:
			io_cache |= SET_CHANNEL_B_RS232;
			OUTBYTE(dev,IOSTATE(dev), io_cache);
		}
	}
	io_cache = INBYTE(dev,IOSTATE(dev));
	LOG_DBG("GPIO after write value = 0x%02X", io_cache);

};

static int sc16is7xx_configure(const struct device *dev,
				  const struct uart_config *cfg)
{
	struct sc16is7xx_dev_data * const dev_data = dev->data;
	const struct sc16is7xx_device_config * const dev_cfg = dev->config;

	/* temp for return value if error occurs in this locked region */
	int ret = 0;
	uint8_t mcr = 0U;
	uint32_t pclk = 0U;


	k_spinlock_key_t key = k_spin_lock(&dev_data->lock);

	ARG_UNUSED(dev_data);
	ARG_UNUSED(dev_cfg);

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
	dev_data->iir_cache = 0U;
#endif

	set_reset(dev);
	
	k_sleep(K_MSEC(10));

	//set up the GPIO for our modes
	set_serial_mode(dev);



	/*
	 * set clock frequency from clock_frequency property
	 */
	pclk = dev_cfg->sys_clk_freq;
	
	set_baud_rate(dev, cfg->baudrate, pclk);

	/* Local structure to hold temporary values to pass to OUTBYTE(dev,) */
	struct uart_config uart_cfg;

	switch (cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		uart_cfg.data_bits = LCR_CS5;
		break;
	case UART_CFG_DATA_BITS_6:
		uart_cfg.data_bits = LCR_CS6;
		break;
	case UART_CFG_DATA_BITS_7:
		uart_cfg.data_bits = LCR_CS7;
		break;
	case UART_CFG_DATA_BITS_8:
		uart_cfg.data_bits = LCR_CS8;
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		uart_cfg.stop_bits = LCR_1_STB;
		break;
	case UART_CFG_STOP_BITS_2:
		uart_cfg.stop_bits = LCR_2_STB;
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		uart_cfg.parity = LCR_PDIS;
		break;
	case UART_CFG_PARITY_EVEN:
		uart_cfg.parity = LCR_EPS | LCR_PEN;
		break;

	case UART_CFG_PARITY_ODD:
		uart_cfg.parity = LCR_PEN;
		break;

	default:
		ret = -ENOTSUP;
		goto out;
	}

	dev_data->uart_config = *cfg;



	/* data bits, stop bits, parity, clear DLAB */
	OUTBYTE(dev, LCR(dev),
		uart_cfg.data_bits | uart_cfg.stop_bits | uart_cfg.parity);

	
	
	mcr =  MCR_RTS_BIT | MCR_DTR_BIT;
	if (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_RTS_CTS) {
		//mcr |= MCR_AFCE;
	}
	//OUTBYTE(dev,MCR(dev), mcr);

	
	//OUTBYTE(dev,EFR(dev),EFR_ENABLE_BIT); 			//unlock the enhanced features register
	//OUTBYTE(dev,MCR(dev),MCR_TCRTLR_BIT); 			//Set the Modem control register bit 2



	OUTBYTE(dev,FCR(dev),
		FCR_FIFO | FCR_FIFO_8 | FCR_RCVRCLR | FCR_XMITCLR | FCR_FIFO_64);

	if ((INBYTE(dev,IIR(dev)) & IIR_FE) == IIR_FE) {
		dev_data->fifo_size = 64;
	} 
	else {
		dev_data->fifo_size = 1;
	}

	/* clear the port */
	INBYTE(dev,RDR(dev));

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
	/* enable interrupts   */
	OUTBYTE(dev,IER(dev), 0x00); //disable interrupts user enables them when the init their driver
#endif	



out:
	k_spin_unlock(&dev_data->lock, key);
	return ret;
};

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int sc16is7xx_config_get(const struct device *dev,
				   struct uart_config *cfg)
{
	struct sc16is7xx_dev_data *data = dev->data;

	cfg->baudrate = data->uart_config.baudrate;
	cfg->parity = data->uart_config.parity;
	cfg->stop_bits = data->uart_config.stop_bits;
	cfg->data_bits = data->uart_config.data_bits;
	cfg->flow_ctrl = data->uart_config.flow_ctrl;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

/**
 * @brief Initialize individual UART port
 *
 * This routine is called to reset the chip in a quiescent state.
 *
 * @param dev UART device struct
 *
 * @return 0 if successful, failed otherwise
 */
static int sc16is7xx_init(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	const struct sc16is7xx_device_config *const config = dev->config;
	int ret;
	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C bus %s is not ready", config->bus.bus->name);
		return -ENODEV;
	}

	ret = sc16is17xx_reg_write(dev, SPR(dev), 123);
	if (ret < 0) {
		LOG_ERR("Failed to write configuration register!");
		return ret;
	}


	uint8_t ping_req = sc16is17xx_reg_read(dev, SPR(dev));
    if (ping_req == 123){
        LOG_DBG("Read from UART OK");
    }


	ret = sc16is7xx_configure(dev, &data->uart_config);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
	/* Store self-reference for interrupt handling */
	data->instance = dev;

		
	/* Prepare interrupt worker */
	k_work_init(&data->interrupt_worker,
			sc16is7xx_interrupt_worker);

	/* Configure GPIO interrupt pin */
	if (!device_is_ready(config->int_gpio.port)) {
		LOG_ERR("sc16is7xx[0x%X]: interrupt GPIO not ready",
			config->bus.addr);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("sc16is7xx[0x%X]: failed to configure interrupt"
			" pin %d (%d)", config->bus.addr,
			config->int_gpio.pin, ret);
		return ret;
	}

	/* Prepare GPIO callback for interrupt pin */
	gpio_init_callback(&data->gpio_callback,
				sc16is7xx_interrupt_callback,
				BIT(config->int_gpio.pin));
	gpio_add_callback(config->int_gpio.port, &data->gpio_callback);
#endif

	return 0;
}




/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int sc16is7xx_poll_in(const struct device *dev, unsigned char *c)
{
	struct sc16is7xx_dev_data *data = dev->data;
	int ret = -1;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if ((INBYTE(dev,LSR(dev)) & LSR_RXRDY) != 0) {
		/* got a character */
		*c = INBYTE(dev,RDR(dev));
		ret = 0;
	}

	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * If the hardware flow control is enabled then the handshake signal CTS has to
 * be asserted in order to send a character.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void sc16is7xx_poll_out(const struct device *dev,
					   unsigned char c)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while ((INBYTE(dev,LSR(dev)) & LSR_THRE) == 0) {
	}

	OUTBYTE(dev,THR(dev), c);

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Check if an error was received
 *
 * @param dev UART device struct
 *
 * @return one of UART_ERROR_OVERRUN, UART_ERROR_PARITY, UART_ERROR_FRAMING,
 * UART_BREAK if an error was detected, 0 otherwise.
 */
static int sc16is7xx_err_check(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int check = (INBYTE(dev,LSR(dev)) & LSR_EOB_MASK);

	k_spin_unlock(&data->lock, key);

	return check >> 1;
}

#if CONFIG_SC16IS7XX_INTERRUPT_DRIVEN

/**
 * @brief Fill FIFO with data
 *
 * @param dev UART device struct
 * @param tx_data Data to transmit
 * @param size Number of bytes to send
 *
 * @return Number of bytes sent
 */
static int sc16is7xx_fifo_fill(const struct device *dev,
				  const uint8_t *tx_data,
				  int size)
{
	struct sc16is7xx_dev_data *data = dev->data;
	int i;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	for (i = 0; (i < size) && (i < data->fifo_size); i++) {
		OUTBYTE(dev,THR(dev), tx_data[i]);
	}

	k_spin_unlock(&data->lock, key);

	return i;
}

/**
 * @brief Read data from FIFO
 *
 * @param dev UART device struct
 * @param rxData Data container
 * @param size Container size
 *
 * @return Number of bytes read
 */
static int sc16is7xx_fifo_read(const struct device *dev, uint8_t *rx_data,
				  const int size)
{
	struct sc16is7xx_dev_data *data = dev->data;
	int i;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	for (i = 0; (i < size) && (INBYTE(dev,LSR(dev)) & LSR_RXRDY) != 0; i++) {
		rx_data[i] = INBYTE(dev,RDR(dev));
	}

	k_spin_unlock(&data->lock, key);

	return i;
}

/**
 * @brief Enable TX interrupt in IER
 *
 * @param dev UART device struct
 */
static void sc16is7xx_irq_tx_enable(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

#if defined(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN) && defined(CONFIG_PM)
	struct sc16is7xx_dev_data *const dev_data = dev->data;

	if (!dev_data->tx_stream_on) {
		dev_data->tx_stream_on = true;
		uint8_t num_cpu_states;
		const struct pm_state_info *cpu_states;

		num_cpu_states = pm_state_cpu_get_all(0U, &cpu_states);

		/*
		 * Power state to be disabled. Some platforms have multiple
		 * states and need to be given a constraint set according to
		 * different states.
		 */
		for (uint8_t i = 0U; i < num_cpu_states; i++) {
			pm_policy_state_lock_get(cpu_states[i].state, PM_ALL_SUBSTATES);
		}
	}
#endif
	OUTBYTE(dev,IER(dev), INBYTE(dev,IER(dev)) | IER_TBE);

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Disable TX interrupt in IER
 *
 * @param dev UART device struct
 */
static void sc16is7xx_irq_tx_disable(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	OUTBYTE(dev,IER(dev), INBYTE(dev,IER(dev)) & (~IER_TBE));

#if defined(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN) && defined(CONFIG_PM)
	struct sc16is7xx_dev_data *const dev_data = dev->data;

	if (dev_data->tx_stream_on) {
		dev_data->tx_stream_on = false;
		uint8_t num_cpu_states;
		const struct pm_state_info *cpu_states;

		num_cpu_states = pm_state_cpu_get_all(0U, &cpu_states);

		/*
		 * Power state to be enabled. Some platforms have multiple
		 * states and need to be given a constraint release according
		 * to different states.
		 */
		for (uint8_t i = 0U; i < num_cpu_states; i++) {
			pm_policy_state_lock_put(cpu_states[i].state, PM_ALL_SUBSTATES);
		}
	}
#endif
	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Check if Tx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int sc16is7xx_irq_tx_ready(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	int ret = ((IIRC(dev) & IIR_ID) == IIR_THRE) ? 1 : 0;

	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Check if nothing remains to be transmitted
 *
 * @param dev UART device struct
 *
 * @return 1 if nothing remains to be transmitted, 0 otherwise
 */
static int sc16is7xx_irq_tx_complete(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	int ret = ((INBYTE(dev,LSR(dev)) & (LSR_TEMT | LSR_THRE))
				== (LSR_TEMT | LSR_THRE)) ? 1 : 0;

	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Enable RX interrupt in IER
 *
 * @param dev UART device struct
 */
static void sc16is7xx_irq_rx_enable(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	OUTBYTE(dev,IER(dev), INBYTE(dev,IER(dev)) | IER_RXRDY);

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Disable RX interrupt in IER
 *
 * @param dev UART device struct
 */
static void sc16is7xx_irq_rx_disable(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	OUTBYTE(dev,IER(dev), INBYTE(dev,IER(dev)) & (~IER_RXRDY));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Check if Rx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int sc16is7xx_irq_rx_ready(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	int ret = ((IIRC(dev) & IIR_ID) == IIR_RBRF) ? 1 : 0;

	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Enable error interrupt in IER
 *
 * @param dev UART device struct
 */
static void sc16is7xx_irq_err_enable(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	OUTBYTE(dev,IER(dev), INBYTE(dev,IER(dev)) | IER_LSR);

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Disable error interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static void sc16is7xx_irq_err_disable(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	OUTBYTE(dev,IER(dev), INBYTE(dev,IER(dev)) & (~IER_LSR));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Check if any IRQ is pending
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is pending, 0 otherwise
 */
static int sc16is7xx_irq_is_pending(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	int ret = (!(IIRC(dev) & IIR_NIP)) ? 1 : 0;

	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Update cached contents of IIR
 *
 * @param dev UART device struct
 *
 * @return Always 1
 */
static int sc16is7xx_irq_update(const struct device *dev)
{
	struct sc16is7xx_dev_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	IIRC(dev) = INBYTE(dev,IIR(dev));

	k_spin_unlock(&data->lock, key);

	return 1;
}

/**
 * @brief Set the callback function pointer for IRQ.
 *
 * @param dev UART device struct
 * @param cb Callback function pointer.
 */
static void sc16is7xx_irq_callback_set(const struct device *dev,
					  uart_irq_callback_user_data_t cb,
					  void *cb_data)
{
	struct sc16is7xx_dev_data * const dev_data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&dev_data->lock);

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;

	k_spin_unlock(&dev_data->lock, key);
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 */
static void sc16is7xx_isr(const struct device *dev)
{
	struct sc16is7xx_dev_data * const dev_data = dev->data;

	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}
}

#endif /* CONFIG_SC16IS7XX_INTERRUPT_DRIVEN */

#ifdef CONFIG_SC16IS7XX_LINE_CTRL

/**
 * @brief Manipulate line control for UART.
 *
 * @param dev UART device struct
 * @param ctrl The line control to be manipulated
 * @param val Value to set the line control
 *
 * @return 0 if successful, failed otherwise
 */
static int sc16is7xx_line_ctrl_set(const struct device *dev,
				      uint32_t ctrl, uint32_t val)
{
	struct sc16is7xx_dev_data *data = dev->data;
	uint32_t mcr, chg;
	k_spinlock_key_t key;

	switch (ctrl) {
	case UART_LINE_CTRL_BAUD_RATE:
		set_baud_rate(dev, val);
		return 0;

	case UART_LINE_CTRL_RTS:
	case UART_LINE_CTRL_DTR:
		key = k_spin_lock(&data->lock);
		mcr = INBYTE(dev,MCR(dev));

		if (ctrl == UART_LINE_CTRL_RTS) {
			chg = MCR_RTS;
		} else {
			chg = MCR_DTR;
		}

		if (val) {
			mcr |= chg;
		} else {
			mcr &= ~(chg);
		}
		OUTBYTE(dev,MCR(dev), mcr);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	return -ENOTSUP;
}

#endif /* CONFIG_SC16IS7XX_LINE_CTRL */



static const struct uart_driver_api sc16is7xx_driver_api = {
	.poll_in = sc16is7xx_poll_in,
	.poll_out = sc16is7xx_poll_out,
	.err_check = sc16is7xx_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = sc16is7xx_configure,
	.config_get = sc16is7xx_config_get,
#endif
#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN

	.fifo_fill = sc16is7xx_fifo_fill,
	.fifo_read = sc16is7xx_fifo_read,
	.irq_tx_enable = sc16is7xx_irq_tx_enable,
	.irq_tx_disable = sc16is7xx_irq_tx_disable,
	.irq_tx_ready = sc16is7xx_irq_tx_ready,
	.irq_tx_complete = sc16is7xx_irq_tx_complete,
	.irq_rx_enable = sc16is7xx_irq_rx_enable,
	.irq_rx_disable = sc16is7xx_irq_rx_disable,
	.irq_rx_ready = sc16is7xx_irq_rx_ready,
	.irq_err_enable = sc16is7xx_irq_err_enable,
	.irq_err_disable = sc16is7xx_irq_err_disable,
	.irq_is_pending = sc16is7xx_irq_is_pending,
	.irq_update = sc16is7xx_irq_update,
	.irq_callback_set = sc16is7xx_irq_callback_set,

#endif

#ifdef CONFIG_SC16IS7XX_LINE_CTRL
	.line_ctrl_set = sc16is7xx_line_ctrl_set,
#endif

};



#define DEV_DATA_FLOW_CTRL0 UART_CFG_FLOW_CTRL_NONE
#define DEV_DATA_FLOW_CTRL1 UART_CFG_FLOW_CTRL_RTS_CTS
#define DEV_DATA_FLOW_CTRL(n) \
	_CONCAT(DEV_DATA_FLOW_CTRL, DT_INST_PROP_OR(n, hw_flow_control, 0))


//refer to uart_rtt.c for an example on how to add a second instance of the driver
//as we'll want to have a different config for the second port, as well as a second instance.
#define SC16IS7XX_DEVICE_INIT(n)                                                  \
	static const struct sc16is7xx_device_config sc16is7xx_dev_cfg_##n = {  \
		.port = DT_INST_REG_ADDR(n),										\
		.serport_num = (n),													\
		.bus = I2C_DT_SPEC_INST_GET(n),		    				            \
		.sys_clk_freq = DT_INST_PROP(n, clock_frequency),   				 \
		.reg_interval = DT_INST_PROP(n, reg_shift),               		     \
		.serial_mode = DT_INST_ENUM_IDX_OR(n, serial_mode, UART_MODE_RS232),  \
	IF_ENABLED(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN,							\
	(.int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, interrupt_gpios, {}),)) \
	};                                                                       \
	static struct sc16is7xx_dev_data sc16is7xx_dev_data_##n = {            \
		.uart_config.baudrate = DT_INST_PROP_OR(n, current_speed, 0),        \
		.uart_config.parity = DT_INST_ENUM_IDX_OR(n, parity, UART_CFG_PARITY_NONE),  \
		.uart_config.stop_bits = DT_INST_PROP_OR(n, stop_bits, UART_CFG_STOP_BITS_1),                       \
		.uart_config.data_bits = DT_INST_PROP_OR(n, data_bits, UART_CFG_DATA_BITS_8),                       \
		.uart_config.flow_ctrl = DEV_DATA_FLOW_CTRL(n),                      \
	};                                                                       \
	DEVICE_DT_INST_DEFINE(n, 												\
				  &sc16is7xx_init, 									\
				  NULL,                           							\
			      &sc16is7xx_dev_data_##n,								\
				  &sc16is7xx_dev_cfg_##n, 								\
			      POST_KERNEL, 												\
				  CONFIG_SERIAL_INIT_PRIORITY,             					\
			      &sc16is7xx_driver_api);                             	\

DT_INST_FOREACH_STATUS_OKAY(SC16IS7XX_DEVICE_INIT)
