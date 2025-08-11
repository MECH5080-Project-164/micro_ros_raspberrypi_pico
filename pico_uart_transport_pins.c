#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

// UART configuration
#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 4  // GPIO4 (UART1 TX)
#define UART_RX_PIN 5  // GPIO5 (UART1 RX)

// You can change these pins to any available UART pins:
// UART0: TX=0,12,16 RX=1,13,17
// UART1: TX=4,8    RX=5,9

bool pico_uart_transport_open(struct uxrCustomTransport * transport)
{
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    
    // Set the GPIO pin mux to the UART - connect UART to the GPIO pins
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Configure UART settings
    uart_set_hw_flow(UART_ID, false, false);    // No hardware flow control
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);  // 8 data bits, 1 stop bit, no parity
    uart_set_fifo_enabled(UART_ID, true);       // Enable FIFO
    
    return true;
}

bool pico_uart_transport_close(struct uxrCustomTransport * transport)
{
    uart_deinit(UART_ID);
    return true;
}

size_t pico_uart_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    size_t written = 0;
    
    for (size_t i = 0; i < len; i++)
    {
        // Wait for space in TX FIFO
        while (!uart_is_writable(UART_ID))
        {
            tight_loop_contents();
        }
        
        uart_putc_raw(UART_ID, buf[i]);
        written++;
    }
    
    *errcode = 0;
    return written;
}

size_t pico_uart_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    uint64_t start_time_us = time_us_64();
    size_t read_bytes = 0;
    
    for (size_t i = 0; i < len; i++)
    {
        // Check timeout
        int64_t elapsed_time_us = time_us_64() - start_time_us;
        if (elapsed_time_us >= (timeout * 1000))
        {
            *errcode = 1;
            return read_bytes;
        }
        
        // Wait for data with timeout
        uint64_t byte_start_time = time_us_64();
        while (!uart_is_readable(UART_ID))
        {
            if ((time_us_64() - byte_start_time) >= (timeout * 1000))
            {
                *errcode = 1;
                return read_bytes;
            }
            tight_loop_contents();
        }
        
        buf[i] = uart_getc(UART_ID);
        read_bytes++;
    }
    
    *errcode = 0;
    return read_bytes;
}
