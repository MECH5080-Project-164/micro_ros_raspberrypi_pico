#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK

#include <stdio.h>
#include <stdint.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

// Serial transport functions (for USB/stdio)
bool pico_serial_transport_open(struct uxrCustomTransport * transport);
bool pico_serial_transport_close(struct uxrCustomTransport * transport);
size_t pico_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t pico_serial_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

// UART transport functions (for specific GPIO pins)
bool pico_uart_transport_open(struct uxrCustomTransport * transport);
bool pico_uart_transport_close(struct uxrCustomTransport * transport);
size_t pico_uart_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t pico_uart_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);

#endif //MICRO_ROS_PICOSDK
