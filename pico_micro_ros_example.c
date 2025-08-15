#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico_uart_transports.h"
#include "ds18b20.h"
#include "pico/bootrom.h"

// Pin definitions
const uint LED_PIN = 25;
const uint PWM_PIN = 16; // GPIO pin for PWM output
const uint PWM_PIN_13 = 13; // GPIO pin 13 for PWM output
const uint PWM_PIN_14 = 14; // GPIO pin 14 for PWM output
const uint PWM_PIN_15 = 15; // GPIO pin 15 for PWM output
const uint DS18B20_PIN = 18; // GPIO pin for DS18B20 1-Wire bus

// Timeout configuration
const uint32_t PWM_TIMEOUT_MS = 1000; // 1 seconds timeout (configurable)
const uint32_t TEMP_PUBLISH_INTERVAL_MS = 1000; // 1 seconds between temperature readings
const uint32_t AGENT_PING_INTERVAL_MS = 5000; // Check agent connection every 5 seconds

// PWM configuration structure
typedef struct {
    uint pin;
    uint slice_num;
    uint channel;
    const char* topic_name;
    bool has_timeout;
} pwm_config_t;

// PWM channel definitions
typedef enum {
    PWM_PUMP = 0,    // Pin 16 - has timeout
    PWM_13 = 1,      // Pin 13 - no timeout
    PWM_14 = 2,      // Pin 14 - no timeout
    PWM_15 = 3,      // Pin 15 - no timeout
    NUM_PWM_CHANNELS
} pwm_channel_e;

static pwm_config_t pwm_configs[NUM_PWM_CHANNELS] = {
    [PWM_PUMP] = {PWM_PIN, 0, 0, "pump_pwm_control", true},
    [PWM_13] = {PWM_PIN_13, 0, 0, "pwm_control_13", false},
    [PWM_14] = {PWM_PIN_14, 0, 0, "pwm_control_14", false},
    [PWM_15] = {PWM_PIN_15, 0, 0, "pwm_control_15", false}
};

// ROS communication structures
rcl_subscription_t subscribers[NUM_PWM_CHANNELS];
std_msgs__msg__Int32 pwm_msgs[NUM_PWM_CHANNELS];

// Logging publisher
rcl_publisher_t log_publisher;
std_msgs__msg__String log_msg;
char log_buffer[256];

// Timeout tracking
volatile uint32_t last_message_time_ms = 0;
volatile uint32_t last_pump_message_time_ms = 0;

// Software reset function
void reset_to_bootloader() {
    reset_usb_boot(0, 0);
}

// DS18B20 configuration
ds18b20_bus_t temp_bus;
rcl_publisher_t temp_publishers[MAX_DS18B20_SENSORS];
std_msgs__msg__Float32 temp_msgs[MAX_DS18B20_SENSORS];
char temp_topic_names[MAX_DS18B20_SENSORS][64];
uint32_t last_temp_publish_time_ms = 0;
uint32_t last_temp_conversion_start_ms = 0;
bool temp_conversion_in_progress = false;
uint32_t last_agent_ping_ms = 0;

// Sensor name mappings
typedef struct {
    char rom_id[17];        // 16 hex chars + null terminator
    char friendly_name[32]; // Friendly name for the sensor
} sensor_mapping_t;

// Define sensor mappings
static const sensor_mapping_t sensor_mappings[] = {
    {"28BBF1500000005", "Vine_Air"},
    {"28B0A754000000A", "Vine_TEP"}
    // Add more mappings as needed
};

static const uint8_t num_sensor_mappings = sizeof(sensor_mappings) / sizeof(sensor_mappings[0]);

// Logging function
void publish_log(const char* message) {
    snprintf(log_buffer, sizeof(log_buffer), "%s", message);
    log_msg.data.data = log_buffer;
    log_msg.data.size = strlen(log_buffer);
    log_msg.data.capacity = sizeof(log_buffer);
    (void)rcl_publish(&log_publisher, &log_msg, NULL);  // Cast to void to suppress warning
}

// Function to get friendly name for a ROM ID
const char* get_sensor_friendly_name(const char* rom_id) {
    for (uint8_t i = 0; i < num_sensor_mappings; i++) {
        if (strcmp(rom_id, sensor_mappings[i].rom_id) == 0) {
            return sensor_mappings[i].friendly_name;
        }
    }
    return NULL; // No mapping found
}

// PWM helper functions
void init_single_pwm(pwm_config_t* config) {
    gpio_set_function(config->pin, GPIO_FUNC_PWM);
    config->slice_num = pwm_gpio_to_slice_num(config->pin);
    config->channel = pwm_gpio_to_channel(config->pin);

    // Set PWM frequency to 30 Hz
    pwm_set_clkdiv(config->slice_num, 125.0f);  // 125 MHz / 125 = 1 MHz base frequency
    pwm_set_wrap(config->slice_num, 33333);     // 1 MHz / 33334 ≈ 30 Hz PWM frequency

    // Start PWM with 0% duty cycle
    pwm_set_chan_level(config->slice_num, config->channel, 0);
    pwm_set_enabled(config->slice_num, true);
}

void init_all_pwm_channels(void) {
    for (int i = 0; i < NUM_PWM_CHANNELS; i++) {
        init_single_pwm(&pwm_configs[i]);
    }
}

void set_pwm_level(pwm_config_t* config, uint16_t level) {
    pwm_set_chan_level(config->slice_num, config->channel, level);
}

void reset_pwm_channels_with_timeout(void) {
    for (int i = 0; i < NUM_PWM_CHANNELS; i++) {
        if (pwm_configs[i].has_timeout) {
            set_pwm_level(&pwm_configs[i], 0);
        }
    }
}

void reset_all_pwm_channels(void) {
    for (int i = 0; i < NUM_PWM_CHANNELS; i++) {
        set_pwm_level(&pwm_configs[i], 0);
    }
}

// Generic PWM callback function
void handle_pwm_message(pwm_channel_e channel, const std_msgs__msg__Int32* msg_in) {
    // Update last message time for general timeout tracking
    last_message_time_ms = to_ms_since_boot(get_absolute_time());

    // Update pump-specific timeout if this is the pump channel
    if (channel == PWM_PUMP) {
        last_pump_message_time_ms = to_ms_since_boot(get_absolute_time());

        // Special reset command: PWM value of 999 triggers bootloader reset
        if (msg_in->data == 999) {
            publish_log("Reset command received, entering bootloader mode...");
            sleep_ms(100); // Give time for message to be sent
            reset_to_bootloader();
            return; // Should never reach here
        }
    }

    // Constrain PWM value to valid range (0-100 for percentage)
    int32_t pwm_value = msg_in->data;
    if (pwm_value < 0) pwm_value = 0;
    if (pwm_value > 100) pwm_value = 100;

    // Convert percentage to PWM level (0-65535 for 16-bit PWM)
    uint16_t pwm_level = (uint16_t)((pwm_value * 65535) / 100);

    // Set PWM duty cycle
    set_pwm_level(&pwm_configs[channel], pwm_level);

    // Log the PWM change
    char log_msg_local[64];
    snprintf(log_msg_local, sizeof(log_msg_local), "PWM%d: Set to %d%% (level: %d)",
             pwm_configs[channel].pin, pwm_value, pwm_level);
    publish_log(log_msg_local);

    // Toggle LED for pump channel to indicate message received
    if (channel == PWM_PUMP) {
        static bool led_state = false;
        led_state = !led_state;
        gpio_put(LED_PIN, led_state);
    }
}

// Individual callback functions
void subscription_callback(const void * msgin) {
    handle_pwm_message(PWM_PUMP, (const std_msgs__msg__Int32 *)msgin);
}

void subscription_callback_pwm13(const void * msgin) {
    handle_pwm_message(PWM_13, (const std_msgs__msg__Int32 *)msgin);
}

void subscription_callback_pwm14(const void * msgin) {
    handle_pwm_message(PWM_14, (const std_msgs__msg__Int32 *)msgin);
}

void subscription_callback_pwm15(const void * msgin) {
    handle_pwm_message(PWM_15, (const std_msgs__msg__Int32 *)msgin);
}

// ROS helper functions
void init_pwm_subscribers(rcl_node_t* node) {
    // Array of callback functions
    void (*callbacks[])(const void*) = {
        subscription_callback,
        subscription_callback_pwm13,
        subscription_callback_pwm14,
        subscription_callback_pwm15
    };

    for (int i = 0; i < NUM_PWM_CHANNELS; i++) {
        rclc_subscription_init_default(
            &subscribers[i],
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            pwm_configs[i].topic_name
        );
    }
}

void cleanup_pwm_subscribers(rcl_node_t* node) {
    for (int i = 0; i < NUM_PWM_CHANNELS; i++) {
        rcl_subscription_fini(&subscribers[i], node);
    }
}

void add_pwm_subscriptions_to_executor(rclc_executor_t* executor) {
    // Array of callback functions
    void (*callbacks[])(const void*) = {
        subscription_callback,
        subscription_callback_pwm13,
        subscription_callback_pwm14,
        subscription_callback_pwm15
    };

    for (int i = 0; i < NUM_PWM_CHANNELS; i++) {
        rclc_executor_add_subscription(executor, &subscribers[i], &pwm_msgs[i], callbacks[i], ON_NEW_DATA);
    }
}

void wait_for_agent_connection(void) {
    const int timeout_ms = 1000;
    const uint8_t attempts_per_cycle = 1;
    rcl_ret_t ret;

    do {
        ret = rmw_uros_ping_agent(timeout_ms, attempts_per_cycle);
        if (ret != RCL_RET_OK) {
            // Flash LED to indicate connection attempt
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
        }
        } while (ret != RCL_RET_OK);
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_uart_transport_open,
		pico_uart_transport_close,
		pico_uart_transport_write,
		pico_uart_transport_read
	);

    // Initialise LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialise all PWM channels
    init_all_pwm_channels();    // Initialise DS18B20 temperature sensors
    ds18b20_init(&temp_bus, DS18B20_PIN);
    uint8_t sensor_count = ds18b20_scan_sensors(&temp_bus);

    // Initialise timeout tracking
    last_message_time_ms = to_ms_since_boot(get_absolute_time());
    last_pump_message_time_ms = to_ms_since_boot(get_absolute_time());
    last_temp_publish_time_ms = to_ms_since_boot(get_absolute_time());
    last_temp_conversion_start_ms = 0;
    temp_conversion_in_progress = false;
    last_agent_ping_ms = to_ms_since_boot(get_absolute_time());

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_ret_t ret;

    allocator = rcl_get_default_allocator();

    // Wait for agent connection
    wait_for_agent_connection();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    // Initialise logging publisher
    rclc_publisher_init_default(
        &log_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "pico_logs"
    );

    // Initialise PWM subscribers
    init_pwm_subscribers(&node);

    // Initialise temperature publishers for each DS18B20 sensor
    char startup_msg[128];
    snprintf(startup_msg, sizeof(startup_msg), "DS18B20: Found %d sensors on GPIO %d", sensor_count, DS18B20_PIN);
    publish_log(startup_msg);

    // Log PWM initialisation
    publish_log("PWM: Initialised pins 13, 14, 15, 16 at 30Hz");
    publish_log("Topics: pwm_control_13, pwm_control_14, pwm_control_15, pump_pwm_control");
    publish_log("Safety: Only pump_pwm_control (pin 16) has 1s timeout, others maintain values");

    for (uint8_t i = 0; i < sensor_count; i++) {
        char rom_str[17];
        ds18b20_rom_to_string(temp_bus.sensors[i].rom, rom_str, sizeof(rom_str));

        // Log ROM ID
        char rom_log[64];
        snprintf(rom_log, sizeof(rom_log), "DS18B20: Sensor %d ROM: %s", i, rom_str);
        publish_log(rom_log);

        // Check if we have a friendly name mapping for this sensor
        const char* friendly_name = get_sensor_friendly_name(rom_str);

        if (friendly_name != NULL) {
            // Use friendly name
            snprintf(temp_topic_names[i], sizeof(temp_topic_names[i]), "temperature/%s", friendly_name);
            char topic_log[96];
            snprintf(topic_log, sizeof(topic_log), "DS18B20: Sensor %d -> Topic: %s", i, temp_topic_names[i]);
            publish_log(topic_log);
        } else {
            // Use ROM ID as fallback
            snprintf(temp_topic_names[i], sizeof(temp_topic_names[i]), "temperature/sensor_%s", rom_str);
            char topic_log[96];
            snprintf(topic_log, sizeof(topic_log), "DS18B20: Sensor %d -> Topic: %s (no mapping)", i, temp_topic_names[i]);
            publish_log(topic_log);
        }

        rcl_ret_t pub_ret = rclc_publisher_init_default(
            &temp_publishers[i],
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            temp_topic_names[i]
        );

        if (pub_ret == RCL_RET_OK) {
            char success_log[64];
            snprintf(success_log, sizeof(success_log), "DS18B20: Publisher %d initialised successfully", i);
            publish_log(success_log);
        } else {
            char error_log[64];
            snprintf(error_log, sizeof(error_log), "DS18B20: Failed to init publisher %d (error: %d)", i, pub_ret);
            publish_log(error_log);
        }
    }
    publish_log("DS18B20: Publisher initialisation complete");

    rclc_executor_init(&executor, &support.context, NUM_PWM_CHANNELS, &allocator);
    add_pwm_subscriptions_to_executor(&executor);

    gpio_put(LED_PIN, 1);

    while (true)
    {
        rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());

        // Periodically ping the agent to detect disconnection
        if ((current_time_ms - last_agent_ping_ms) > AGENT_PING_INTERVAL_MS)
        {
            rcl_ret_t ping_ret = rmw_uros_ping_agent(1000, 1); // 1 second timeout, 1 attempt
            if (ping_ret != RCL_RET_OK)
            {
                // Agent is not responding, force reconnection
                publish_log("Agent ping failed, forcing reconnection...");
                spin_ret = RCL_RET_ERROR; // Force the reconnection logic
            } else {
                publish_log("Agent ping successful");
            }
            last_agent_ping_ms = current_time_ms;
        }

        // Check if agent disconnected (either from spin failure or ping failure)
        if (spin_ret != RCL_RET_OK)
        {
            publish_log("Connection lost - starting cleanup and reconnection...");

            // Agent disconnected, cleanup and restart
            rclc_executor_fini(&executor);
            cleanup_pwm_subscribers(&node);
            rcl_publisher_fini(&log_publisher, &node);

            // Cleanup temperature publishers
            for (uint8_t i = 0; i < sensor_count; i++) {
                if (temp_bus.sensors[i].valid) {
                    rcl_publisher_fini(&temp_publishers[i], &node);
                }
            }

            rcl_node_fini(&node);
            rclc_support_fini(&support);

            // Reset PWM to 0% for safety
            reset_all_pwm_channels();

            // Start reconnection process
            wait_for_agent_connection();

            // Reinitialise everything
            rclc_support_init(&support, 0, NULL, &allocator);
            rclc_node_init_default(&node, "pico_node", "", &support);

            // Reinitialise logging publisher
            rclc_publisher_init_default(
                &log_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                "pico_logs"
            );

            // Reinitialise PWM subscribers
            init_pwm_subscribers(&node);

            // Reinitialise temperature publishers
            for (uint8_t i = 0; i < sensor_count; i++) {
                if (temp_bus.sensors[i].valid) {
                    rclc_publisher_init_default(
                        &temp_publishers[i],
                        &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                        temp_topic_names[i]
                    );
                }
            }

            rclc_executor_init(&executor, &support.context, NUM_PWM_CHANNELS, &allocator);
            add_pwm_subscriptions_to_executor(&executor);

            // Reset timeout tracking
            last_message_time_ms = to_ms_since_boot(get_absolute_time());
            last_pump_message_time_ms = to_ms_since_boot(get_absolute_time());
            last_temp_publish_time_ms = to_ms_since_boot(get_absolute_time());
            last_temp_conversion_start_ms = 0;
            temp_conversion_in_progress = false;
            last_agent_ping_ms = to_ms_since_boot(get_absolute_time());
            gpio_put(LED_PIN, 1);
            continue;
        }

        // Check for timeout and set pump PWM to 0% if no recent pump messages
        // PWM pins 13, 14, 15 maintain their values (no timeout)
        if ((current_time_ms - last_pump_message_time_ms) > PWM_TIMEOUT_MS)
        {
            set_pwm_level(&pwm_configs[PWM_PUMP], 0); // Only pump PWM (pin 16) has timeout
        }

        // Publish temperature readings at regular intervals
        if ((current_time_ms - last_temp_publish_time_ms) > TEMP_PUBLISH_INTERVAL_MS)
        {
            if (!temp_conversion_in_progress) {
                // Start conversion for all sensors simultaneously
                char conv_log[64];
                snprintf(conv_log, sizeof(conv_log), "DS18B20: Starting conversion for %d sensors", sensor_count);
                publish_log(conv_log);

                for (uint8_t i = 0; i < sensor_count; i++) {
                    if (temp_bus.sensors[i].valid) {
                        ds18b20_start_conversion(&temp_bus, NULL); // NULL = all sensors at once
                        break; // Only need to send command once for all sensors
                    }
                }
                temp_conversion_in_progress = true;
                last_temp_conversion_start_ms = current_time_ms;
            } else if ((current_time_ms - last_temp_conversion_start_ms) >= 750) {
                // Conversion should be complete, read all sensors
                publish_log("DS18B20: Reading temperatures and publishing...");
                for (uint8_t i = 0; i < sensor_count; i++) {
                    if (temp_bus.sensors[i].valid) {
                        float temperature = ds18b20_read_temperature(&temp_bus, temp_bus.sensors[i].rom);

                        if (temperature != -999.0f) {
                            temp_msgs[i].data = temperature;
                            rcl_ret_t pub_ret = rcl_publish(&temp_publishers[i], &temp_msgs[i], NULL);

                            char temp_log[128];
                            snprintf(temp_log, sizeof(temp_log), "DS18B20: Sensor %d: %.2f°C -> %s (ret: %d)",
                                   i, temperature, temp_topic_names[i], pub_ret);
                            publish_log(temp_log);
                        } else {
                            char error_log[64];
                            snprintf(error_log, sizeof(error_log), "DS18B20: Sensor %d: Failed to read temperature", i);
                            publish_log(error_log);
                        }
                    }
                }
                temp_conversion_in_progress = false;
                last_temp_publish_time_ms = current_time_ms;
            }
        }
    }
    return 0;
}
