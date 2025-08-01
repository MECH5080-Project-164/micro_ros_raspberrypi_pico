#ifndef DS18B20_H
#define DS18B20_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// DS18B20 Commands
#define DS18B20_SKIP_ROM        0xCC
#define DS18B20_SEARCH_ROM      0xF0
#define DS18B20_READ_ROM        0x33
#define DS18B20_MATCH_ROM       0x55
#define DS18B20_CONVERT_T       0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

// DS18B20 Configuration
#define DS18B20_RESOLUTION_9BIT  0x1F
#define DS18B20_RESOLUTION_10BIT 0x3F
#define DS18B20_RESOLUTION_11BIT 0x5F
#define DS18B20_RESOLUTION_12BIT 0x7F

// Maximum number of sensors supported
#define MAX_DS18B20_SENSORS 8

// DS18B20 ROM structure
typedef struct {
    uint8_t rom[8];  // 64-bit ROM code
    bool valid;
} ds18b20_rom_t;

// DS18B20 sensor list
typedef struct {
    ds18b20_rom_t sensors[MAX_DS18B20_SENSORS];
    uint8_t count;
    uint gpio_pin;
} ds18b20_bus_t;

// Function prototypes
bool ds18b20_init(ds18b20_bus_t* bus, uint gpio_pin);
bool ds18b20_reset(ds18b20_bus_t* bus);
void ds18b20_write_bit(ds18b20_bus_t* bus, uint8_t bit);
uint8_t ds18b20_read_bit(ds18b20_bus_t* bus);
void ds18b20_write_byte(ds18b20_bus_t* bus, uint8_t byte);
uint8_t ds18b20_read_byte(ds18b20_bus_t* bus);
uint8_t ds18b20_scan_sensors(ds18b20_bus_t* bus);
bool ds18b20_start_conversion(ds18b20_bus_t* bus, uint8_t* rom);
float ds18b20_read_temperature(ds18b20_bus_t* bus, uint8_t* rom);
void ds18b20_rom_to_string(uint8_t* rom, char* str, size_t str_len);

#endif // DS18B20_H
