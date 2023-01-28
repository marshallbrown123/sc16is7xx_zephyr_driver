# sc16is7xx_zephyr_driver
Zephyr driver for the NXP I2C SC16IS7XX dual UART

This is a cut and paste driver from the Zephyr 16550 driver (zephyr\drivers\serial\uart_ns16550.c) a few register definitions from the linux kernel (https://github.com/torvalds/linux/blob/master/drivers/tty/serial/sc16is7xx.c) with a few mods to drive the I2C from Zephyr. 

It's a bit messy but hopefully will be enough to get you started, and you can contribute back to make it better for all of us. 

Add this to your overlay file

	opito_uart:sc16is752@48 {
		status = "okay";
		compatible = "nxp,sc16is7xx";
		reg-shift = < 3 >;
		reg = <0x48>;
		current-speed = < 115200 >;
		clock-frequency = < 14745600 >;
		//parity = "odd"; 	//NRF9160 doesn't support ODD parity but the sc16is752 does.
		interrupt-gpios = < &opito_header 19 (GPIO_ACTIVE_LOW)>;
		num-ports = <2>; 
		serial-mode = "rs232";
		config = <123>;

		};
	};
	
	
Then you can use the device as a normal UART like so, (add these functions to main.c or whatever)

	#include "zephyr/drivers/uart.h"
	void uart_cb(struct device *dev)
	{
	    uart_irq_update(dev);
	    int data_length = 0;
	    if (uart_irq_rx_ready(dev)) {
		data_length = uart_fifo_read(dev, uart_buf, sizeof(uart_buf));
		uart_buf[data_length] = 0;
			LOG_WRN("Rx from i2c uart %s", uart_buf);
	    }
	}


	static bool initUART2(){

	    if (uart_dev == NULL) {
		LOG_WRN("Cannot bind %s\n", "opito_uart");
		return false;
	    }
	    uart_irq_callback_set(uart_dev, uart_cb);
		uart_irq_rx_enable(uart_dev);
	    return true;
	}

	void uart2_start(void){
	    	initUART2();
		for (int i = 0; i < strlen(poll_data); i++) {
			uart_poll_out(uart_dev, poll_data[i]);
		}
	}


Currently only supports the first serial port, but should have hooks in there for the second port, without too many issues, Ideally someone will tidy this up and integrate it into the Zephyr mainline code for the benefit of all. 

