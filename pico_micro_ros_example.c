#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;
const uint PWM_PIN = 16; // GPIO pin for PWM output

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

// PWM configuration
uint slice_num;
uint channel;

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg_in = (const std_msgs__msg__Int32 *)msgin;
    
    // Constrain PWM value to valid range (0-100 for percentage)
    int32_t pwm_value = msg_in->data;
    if (pwm_value < 0) pwm_value = 0;
    if (pwm_value > 100) pwm_value = 100;
    
    // Convert percentage to PWM level (0-65535 for 16-bit PWM)
    uint16_t pwm_level = (uint16_t)((pwm_value * 65535) / 100);
    
    // Set PWM duty cycle
    pwm_set_chan_level(slice_num, channel, pwm_level);
    
    // Toggle LED to indicate message received
    static bool led_state = false;
    led_state = !led_state;
    gpio_put(LED_PIN, led_state);
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // Initialize LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Initialize PWM
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    channel = pwm_gpio_to_channel(PWM_PIN);
    
    // Set PWM frequency to 1 kHz (adjust as needed)
    pwm_set_clkdiv(slice_num, 125.0f);  // 125 MHz / 125 = 1 MHz base frequency
    pwm_set_wrap(slice_num, 999);       // 1 MHz / 1000 = 1 kHz PWM frequency
    
    // Start PWM with 0% duty cycle
    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    
    // Initialize subscriber for PWM control
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pwm_control");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
