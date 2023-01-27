# sc16is7xx_zephyr_driver
Zephyr driver for the NXP I2C SC16IS7XX dual UART


Add something like this to your overlay.
	
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



Here's how you test it. 


#include "zephyr/drivers/uart.h"

#define FEATHER_UART DT_NODELABEL(opito_uart)
#if DT_NODE_HAS_STATUS(FEATHER_UART,okay)
    const struct device *uart_dev = DEVICE_DT_GET(FEATHER_UART);
#else
    #error "uart2 device is disabled."
#endif
static uint8_t uart_buf[256];
static const char *poll_data = "This is a POLL test.\r\n";




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

void uart2_test(void){
    initUART2();
    for (int i = 0; i < strlen(poll_data); i++) {
        uart_poll_out(uart_dev, poll_data[i]);
    }
