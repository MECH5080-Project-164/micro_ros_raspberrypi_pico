#include "ds18b20.h"
#include <string.h>
#include <stdio.h>

// Timing constants for DS18B20 (in microseconds)
#define DS18B20_RESET_PULSE_TIME    480
#define DS18B20_RESET_WAIT_TIME     480
#define DS18B20_WRITE_1_LOW_TIME    1
#define DS18B20_WRITE_1_HIGH_TIME   60
#define DS18B20_WRITE_0_LOW_TIME    60
#define DS18B20_WRITE_0_HIGH_TIME   1
#define DS18B20_READ_LOW_TIME       1
#define DS18B20_READ_HIGH_TIME      60
#define DS18B20_RECOVERY_TIME       1

bool ds18b20_init(ds18b20_bus_t* bus, uint gpio_pin) {
    bus->gpio_pin = gpio_pin;
    bus->count = 0;
    
    gpio_init(gpio_pin);
    gpio_set_dir(gpio_pin, GPIO_OUT);
    gpio_put(gpio_pin, 1);
    
    return true;
}

bool ds18b20_reset(ds18b20_bus_t* bus) {
    // Pull bus low for reset pulse
    gpio_set_dir(bus->gpio_pin, GPIO_OUT);
    gpio_put(bus->gpio_pin, 0);
    sleep_us(DS18B20_RESET_PULSE_TIME);
    
    // Release bus and wait for presence pulse
    gpio_set_dir(bus->gpio_pin, GPIO_IN);
    gpio_pull_up(bus->gpio_pin);
    sleep_us(70);
    
    // Check for presence pulse (should be low)
    bool presence = !gpio_get(bus->gpio_pin);
    
    sleep_us(DS18B20_RESET_WAIT_TIME - 70);
    
    return presence;
}

void ds18b20_write_bit(ds18b20_bus_t* bus, uint8_t bit) {
    gpio_set_dir(bus->gpio_pin, GPIO_OUT);
    gpio_put(bus->gpio_pin, 0);
    
    if (bit) {
        sleep_us(DS18B20_WRITE_1_LOW_TIME);
        gpio_put(bus->gpio_pin, 1);
        sleep_us(DS18B20_WRITE_1_HIGH_TIME);
    } else {
        sleep_us(DS18B20_WRITE_0_LOW_TIME);
        gpio_put(bus->gpio_pin, 1);
        sleep_us(DS18B20_WRITE_0_HIGH_TIME);
    }
    sleep_us(DS18B20_RECOVERY_TIME);
}

uint8_t ds18b20_read_bit(ds18b20_bus_t* bus) {
    gpio_set_dir(bus->gpio_pin, GPIO_OUT);
    gpio_put(bus->gpio_pin, 0);
    sleep_us(DS18B20_READ_LOW_TIME);
    
    gpio_set_dir(bus->gpio_pin, GPIO_IN);
    gpio_pull_up(bus->gpio_pin);
    sleep_us(15);
    
    uint8_t bit = gpio_get(bus->gpio_pin);
    sleep_us(DS18B20_READ_HIGH_TIME - 15);
    sleep_us(DS18B20_RECOVERY_TIME);
    
    return bit;
}

void ds18b20_write_byte(ds18b20_bus_t* bus, uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(bus, (byte >> i) & 1);
    }
}

uint8_t ds18b20_read_byte(ds18b20_bus_t* bus) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (ds18b20_read_bit(bus)) {
            byte |= (1 << i);
        }
    }
    return byte;
}

uint8_t ds18b20_scan_sensors(ds18b20_bus_t* bus) {
    uint8_t rom_search[8];
    uint8_t last_discrepancy = 0;
    uint8_t done_flag = 0;
    bus->count = 0;
    
    while (!done_flag && bus->count < MAX_DS18B20_SENSORS) {
        if (!ds18b20_reset(bus)) {
            break;
        }
        
        ds18b20_write_byte(bus, DS18B20_SEARCH_ROM);
        
        uint8_t last_zero = 0;
        uint8_t rom_byte_number = 0;
        uint8_t rom_bit_number = 1;
        uint8_t search_direction = 0;
        uint8_t crc8 = 0;
        
        for (int id_bit_number = 1; id_bit_number < 65; id_bit_number++) {
            uint8_t id_bit = ds18b20_read_bit(bus);
            uint8_t cmp_id_bit = ds18b20_read_bit(bus);
            
            if ((id_bit == 1) && (cmp_id_bit == 1)) {
                break; // No devices found
            } else {
                if (id_bit != cmp_id_bit) {
                    search_direction = id_bit;
                } else {
                    if (id_bit_number < last_discrepancy) {
                        search_direction = ((rom_search[rom_byte_number] & rom_bit_number) > 0);
                    } else {
                        search_direction = (id_bit_number == last_discrepancy);
                    }
                    
                    if (search_direction == 0) {
                        last_zero = id_bit_number;
                    }
                }
                
                if (search_direction == 1) {
                    rom_search[rom_byte_number] |= rom_bit_number;
                } else {
                    rom_search[rom_byte_number] &= ~rom_bit_number;
                }
                
                ds18b20_write_bit(bus, search_direction);
                
                rom_bit_number <<= 1;
                if (rom_bit_number == 0) {
                    rom_byte_number++;
                    rom_bit_number = 1;
                }
            }
        }
        
        if (rom_byte_number >= 8) {
            // Valid ROM found
            memcpy(bus->sensors[bus->count].rom, rom_search, 8);
            bus->sensors[bus->count].valid = true;
            bus->count++;
        }
        
        last_discrepancy = last_zero;
        if (last_discrepancy == 0) {
            done_flag = 1;
        }
    }
    
    return bus->count;
}

bool ds18b20_start_conversion(ds18b20_bus_t* bus, uint8_t* rom) {
    if (!ds18b20_reset(bus)) {
        return false;
    }
    
    if (rom) {
        ds18b20_write_byte(bus, DS18B20_MATCH_ROM);
        for (int i = 0; i < 8; i++) {
            ds18b20_write_byte(bus, rom[i]);
        }
    } else {
        ds18b20_write_byte(bus, DS18B20_SKIP_ROM);
    }
    
    ds18b20_write_byte(bus, DS18B20_CONVERT_T);
    return true;
}

float ds18b20_read_temperature(ds18b20_bus_t* bus, uint8_t* rom) {
    if (!ds18b20_reset(bus)) {
        return -999.0f;
    }
    
    if (rom) {
        ds18b20_write_byte(bus, DS18B20_MATCH_ROM);
        for (int i = 0; i < 8; i++) {
            ds18b20_write_byte(bus, rom[i]);
        }
    } else {
        ds18b20_write_byte(bus, DS18B20_SKIP_ROM);
    }
    
    ds18b20_write_byte(bus, DS18B20_READ_SCRATCHPAD);
    
    uint8_t scratchpad[9];
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = ds18b20_read_byte(bus);
    }
    
    // Calculate temperature from scratchpad data
    int16_t temp_raw = (scratchpad[1] << 8) | scratchpad[0];
    float temperature = (float)temp_raw / 16.0f;
    
    return temperature;
}

void ds18b20_rom_to_string(uint8_t* rom, char* str, size_t str_len) {
    snprintf(str, str_len, "%02X%02X%02X%02X%02X%02X%02X%02X",
             rom[0], rom[1], rom[2], rom[3], rom[4], rom[5], rom[6], rom[7]);
}
