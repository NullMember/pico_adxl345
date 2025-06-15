#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "adxl_345.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

int main() {
    // Initialize the standard library for Pico
    stdio_init_all();

    sleep_ms(1000); // Wait for 1 second to ensure the system is ready

    // Initialize the ADXL345 sensor
    adxl_345 sensor(spi1);
    sensor.init();

    uint8_t watermark_level = 16;
    uint8_t range = 0;
    uint8_t rate = 15;

    // Check if the ADXL345 is present
    if (sensor.is_adxl345_present()) {
        printf("ADXL345 sensor is present.\n");
    } else {
        printf("ADXL345 sensor not found.\n");
        return -1; // Exit if sensor is not found
    }

    sensor.set_power_mode(adxl345_power_mode::MEASURE);
    sensor.set_full_resolution(true);
    sensor.set_fifo_mode(adxl345_fifo_mode::FIFO_STREAM);
    
    sensor.set_range(adxl345_range(range));
    sensor.set_rate(adxl345_rate(rate));
    sensor.set_watermark_level(watermark_level); // Set watermark level for FIFO

    uint8_t start_of_transfer[3] = {range, rate, watermark_level};
    uint8_t fifo_buffer_raw[32 * 6]; // Buffer to hold raw FIFO data
    uint8_t end_of_transfer[6] = {0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF}; // End of transfer marker

    int ser_in;
    int ser_data;
    char send_mode = 'n';
    
    while (true) {
        sensor.read_interrupt_register(); // Read the interrupt register to check for data ready
        if (sensor.is_fifo_ready()) {
            for (size_t i = 0; i < watermark_level; i++)
            {
                sensor.read(adxl345_register::REG_DATAX0, fifo_buffer_raw + i * 6, 6);
            }
            stdio_put_string((char*)start_of_transfer, 3, false, false);
            stdio_put_string((char*)fifo_buffer_raw, watermark_level * 6, false, false);
            stdio_put_string((char*)end_of_transfer, 6, false, false);
        }
        ser_in = getchar_timeout_us(0);
        if (ser_in != PICO_ERROR_TIMEOUT) {
            ser_data = ser_in;
            if (ser_data == 'r') {
                int new_range;
                scanf("%d", &new_range);
                if (new_range >= 0 && new_range <= 3) {
                    sensor.set_range(static_cast<adxl345_range>(new_range));
                    range = new_range;
                    start_of_transfer[0] = range; // Update start of transfer with new range
                }
            } else if (ser_data == 's') {
                int new_rate;
                scanf("%d", &new_rate);
                if (new_rate >= 4 && new_rate <= 15) {
                    sensor.set_rate(static_cast<adxl345_rate>(new_rate));
                    rate = new_rate;
                    start_of_transfer[1] = rate; // Update start of transfer with new rate
                }
            } else if (ser_data == 'w') {
                int new_watermark_level;
                scanf("%d", &new_watermark_level);
                if (new_watermark_level >= 1 && new_watermark_level <= 32) {
                    sensor.set_watermark_level(new_watermark_level);
                    watermark_level = new_watermark_level;
                    start_of_transfer[2] = watermark_level; // Update start of transfer with new watermark level
                }
            }
        }
    }
    return 0; // Program should never reach here in this infinite loop
}