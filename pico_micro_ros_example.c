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

// Timeout configuration
const uint32_t PWM_TIMEOUT_MS = 5000; // 5 seconds timeout (configurable)

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

// PWM configuration
uint slice_num;
uint channel;

// Timeout tracking
volatile uint32_t last_message_time_ms = 0;

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg_in = (const std_msgs__msg__Int32 *)msgin;
    
    // Update last message time
    last_message_time_ms = to_ms_since_boot(get_absolute_time());
    
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
    
    // Set PWM frequency to 30 Hz (adjust as needed)
    pwm_set_clkdiv(slice_num, 125.0f);  // 125 MHz / 125 = 1 MHz base frequency
    pwm_set_wrap(slice_num, 33333);     // 1 MHz / 33334 ≈ 30 Hz PWM frequency
    
    // Start PWM with 0% duty cycle
    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);

    // Initialize timeout tracking
    last_message_time_ms = to_ms_since_boot(get_absolute_time());

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Keep trying to connect to agent indefinitely
    const int timeout_ms = 1000; 
    const uint8_t attempts_per_cycle = 1; // Try 1 time per cycle

    rcl_ret_t ret;
    do {
        ret = rmw_uros_ping_agent(timeout_ms, attempts_per_cycle);
        if (ret != RCL_RET_OK)
        {
            // Flash LED to indicate connection attempt
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
        }
    } while (ret != RCL_RET_OK);

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    
    // Initialize subscriber for PWM control
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pump_pwm_control"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    while (true)
    {
        rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        
        // Check if agent disconnected
        if (spin_ret != RCL_RET_OK)
        {
            // Agent disconnected, cleanup and restart
            rclc_executor_fini(&executor);
            rcl_subscription_fini(&subscriber, &node);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            
            // Reset PWM to 0% for safety
            pwm_set_chan_level(slice_num, channel, 0);
            
            // Start reconnection process
            do {
                ret = rmw_uros_ping_agent(timeout_ms, attempts_per_cycle);
                if (ret != RCL_RET_OK)
                {
                    // Flash LED to indicate connection attempt
                    gpio_put(LED_PIN, 1);
                    sleep_ms(100);
                    gpio_put(LED_PIN, 0);
                    sleep_ms(100);
                }
            } while (ret != RCL_RET_OK);
            
            // Reinitialize everything
            rclc_support_init(&support, 0, NULL, &allocator);
            rclc_node_init_default(&node, "pico_node", "", &support);
            rclc_subscription_init_default(
                &subscriber,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                "pump_pwm_control"
            );
            rclc_executor_init(&executor, &support.context, 1, &allocator);
            rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
            
            // Reset timeout tracking
            last_message_time_ms = to_ms_since_boot(get_absolute_time());
            gpio_put(LED_PIN, 1);
            continue;
        }
        
        // Check for timeout and set PWM to 0% if no recent messages
        uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
        if ((current_time_ms - last_message_time_ms) > PWM_TIMEOUT_MS)
        {
            pwm_set_chan_level(slice_num, channel, 0);
        }
    }
    return 0;
}
