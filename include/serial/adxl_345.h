#ifndef ADXL_345_H
#define ADXL_345_H
#include <stdint.h>
#include <stdio.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>

typedef struct {
    uint8_t x_low;
    uint8_t x_high;
    uint8_t y_low;
    uint8_t y_high;
    uint8_t z_low;
    uint8_t z_high;
} adxl345_raw_data_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} adxl345_data_t;

typedef struct {
    float x;
    float y;
    float z;
} adxl345_float_data_t;

enum adxl345_power_mode {
    STANDBY,
    MEASURE
};

enum adxl345_range {
    RANGE_2G,
    RANGE_4G,
    RANGE_8G,
    RANGE_16G
};

enum adxl345_rate {
    RATE_1_56HZ = 0x04,
    RATE_3_13HZ,
    RATE_6_25HZ,
    RATE_12_5HZ,
    RATE_25HZ,
    RATE_50HZ,
    RATE_100HZ,
    RATE_200HZ,
    RATE_400HZ,
    RATE_800HZ,
    RATE_1600HZ,
    RATE_3200HZ,
};

enum adxl345_fifo_mode {
    FIFO_BYPASS = 0x00,
    FIFO_FIFO   = 0x01,
    FIFO_STREAM = 0x02,
    FIFO_TRIGGER = 0x03,
};

enum adxl345_interrupts {
    INT_DATA_READY = 0x80, // Data ready interrupt
    INT_SINGLE_TAP = 0x40, // Single tap interrupt
    INT_DOUBLE_TAP = 0x20, // Double tap interrupt
    INT_ACTIVITY   = 0x10, // Activity interrupt
    INT_INACTIVITY = 0x08, // Inactivity interrupt
    INT_FREE_FALL  = 0x04, // Free fall interrupt
    INT_WATERMARK  = 0x02, // Watermark interrupt
    INT_OVERRUN    = 0x01, // FIFO overrun interrupt
};

enum adxl345_register {
    REG_DEVICE_ID,
    REG_THRESH_TAP,
    REG_OFSX,
    REG_OFSY,
    REG_OFSZ,
    REG_DUR,
    REG_LATENT,
    REG_WINDOW,
    REG_THRESH_ACT,
    REG_THRESH_INACT,
    REG_TIME_INACT,
    REG_ACT_INACT_CTL,
    REG_THRESH_FF,
    REG_TAP_AXES,
    REG_ACT_TAP_STATUS,
    REG_BW_RATE,
    REG_POWER_CTL,
    REG_INT_ENABLE,
    REG_INT_MAP,
    REG_INT_SOURCE,
    REG_DATA_FORMAT,
    REG_DATAX0,
    REG_DATAX1,
    REG_DATAY0,
    REG_DATAY1,
    REG_DATAZ0,
    REG_DATAZ1,
    REG_FIFO_CTL,
    REG_FIFO_STATUS,
    REG_MAX
};

const uint8_t adxl345_register_address_array[] = {
    0x00, // REG_DEVICE_ID
    0x1D, // REG_THRESH_TAP
    0x1E, // REG_OFSX
    0x1F, // REG_OFSY
    0x20, // REG_OFSZ
    0x21, // REG_DUR
    0x22, // REG_LATENT
    0x23, // REG_WINDOW
    0x24, // REG_THRESH_ACT
    0x25, // REG_THRESH_INACT
    0x26, // REG_TIME_INACT
    0x27, // REG_ACT_INACT_CTL
    0x28, // REG_THRESH_FF
    0x2A, // REG_TAP_AXES
    0x2B, // REG_ACT_TAP_STATUS
    0x2C, // REG_BW_RATE
    0x2D, // REG_POWER_CTL
    0x2E, // REG_INT_ENABLE
    0x2F, // REG_INT_MAP
    0x30, // REG_INT_SOURCE
    0x31, // REG_DATA_FORMAT
    0x32, // REG_DATAX0
    0x33, // REG_DATAX1
    0x34, // REG_DATAY0
    0x35, // REG_DATAY1
    0x36, // REG_DATAZ0
    0x37, // REG_DATAZ1
    0x38, // REG_FIFO_CTL
    0x39  // REG_FIFO_STATUS
};

struct adxl345_register_map {
    uint8_t device_id;        // Device ID register
    uint8_t thresh_tap;       // Tap threshold register
    uint8_t ofs_x;            // X-axis offset register
    uint8_t ofs_y;            // Y-axis offset register
    uint8_t ofs_z;            // Z-axis offset register
    uint8_t dur;              // Tap duration register
    uint8_t latent;           // Latent time register
    uint8_t window;           // Window time register
    uint8_t thresh_act;       // Activity threshold register
    uint8_t thresh_inact;     // Inactivity threshold register
    uint8_t time_inact;       // Inactivity time register
    uint8_t act_inact_ctl;    // Activity/Inactivity control register
    uint8_t thresh_ff;        // Free-fall threshold register
    uint8_t tap_axes;         // Tap axes control register
    uint8_t act_tap_status;   // Activity/Tap status register
    uint8_t bw_rate;          // Bandwidth/Rate control register
    uint8_t power_ctl;        // Power control register
    uint8_t int_enable;       // Interrupt enable register
    uint8_t int_map;          // Interrupt mapping register
    uint8_t int_source;       // Interrupt source register
    uint8_t data_format;      // Data format control register
    uint8_t data_x0;          // X-axis data low byte
    uint8_t data_x1;          // X-axis data high byte
    uint8_t data_y0;          // Y-axis data low byte
    uint8_t data_y1;          // Y-axis data high byte
    uint8_t data_z0;          // Z-axis data low byte
    uint8_t data_z1;          // Z-axis data high byte
    uint8_t fifo_ctl;         // FIFO control register
    uint8_t fifo_status;      // FIFO status register
};

#define ADXL_345_ACT_INACT_ACT_ACDC_MASK 0x80 // Activity/Inactivity control register ACDC bit mask
#define ADXL_345_ACT_INACT_ACT_X_ENABLE_MASK 0x40 // Activity/Inactivity control register X-axis enable bit mask
#define ADXL_345_ACT_INACT_ACT_Y_ENABLE_MASK 0x20 // Activity/Inactivity control register Y-axis enable bit mask
#define ADXL_345_ACT_INACT_ACT_Z_ENABLE_MASK 0x10 // Activity/Inactivity control register Z-axis enable bit mask
#define ADXL_345_ACT_INACT_INACT_ACDC_MASK  0x08 // Activity/Inactivity control register Inactivity ACDC bit mask
#define ADXL_345_ACT_INACT_INACT_X_ENABLE_MASK 0x04 // Activity/Inactivity control register Inactivity X-axis enable bit mask
#define ADXL_345_ACT_INACT_INACT_Y_ENABLE_MASK 0x02 // Activity/Inactivity control register Inactivity Y-axis enable bit mask
#define ADXL_345_ACT_INACT_INACT_Z_ENABLE_MASK 0x01 // Activity/Inactivity control register Inactivity Z-axis enable bit mask

#define ADXL_345_TAP_AXES_SUPPRESS_MASK 0x08 // Tap axes control register Suppress bit mask
#define ADXL_345_TAP_AXES_X_ENABLE_MASK 0x04 // Tap axes control register X-axis enable bit mask
#define ADXL_345_TAP_AXES_Y_ENABLE_MASK 0x02 // Tap axes control register Y-axis enable bit mask
#define ADXL_345_TAP_AXES_Z_ENABLE_MASK 0x01 // Tap axes control register Z-axis enable bit mask

#define ADXL_345_BW_RATE_LOW_POWER_MASK 0x10 // Bandwidth/Rate control register Low Power mask
#define ADXL_345_BW_RATE_RATE_MASK 0x0F // Bandwidth/Rate control register Rate mask

#define ADXL_345_BW_RATE_3200HZ 0x0F // Bandwidth/Rate control register 3200Hz
#define ADXL_345_BW_RATE_1600HZ 0x0E // Bandwidth/Rate control register 1600Hz
#define ADXL_345_BW_RATE_800HZ  0x0D // Bandwidth/Rate control register 800Hz
#define ADXL_345_BW_RATE_400HZ  0x0C // Bandwidth/Rate control register 400Hz
#define ADXL_345_BW_RATE_200HZ  0x0B // Bandwidth/Rate control register 200Hz
#define ADXL_345_BW_RATE_100HZ  0x0A // Bandwidth/Rate control register 100Hz
#define ADXL_345_BW_RATE_50HZ   0x09 // Bandwidth/Rate control register 50Hz
#define ADXL_345_BW_RATE_25HZ   0x08 // Bandwidth/Rate control register 25Hz
#define ADXL_345_BW_RATE_12_5HZ 0x07 // Bandwidth/Rate control register 12.5Hz
#define ADXL_345_BW_RATE_6_25HZ 0x06 // Bandwidth/Rate control register 6.25Hz
#define ADXL_345_BW_RATE_3_13HZ 0x05 // Bandwidth/Rate control register 3.13Hz
#define ADXL_345_BW_RATE_1_56HZ 0x04 // Bandwidth/Rate control register 1.56Hz

#define ADXL_345_POWER_CTL_LINK_MASK 0x20 // Power control register Link bit mask
#define ADXL_345_POWER_CTL_AUTO_SLEEP_MASK 0x10 // Power control register Auto Sleep bit mask
#define ADXL_345_POWER_CTL_MEASURE_MASK 0x08 // Power control register Measure bit mask
#define ADXL_345_POWER_CTL_SLEEP_MASK 0x04 // Power control register Sleep bit mask
#define ADXL_345_POWER_CTL_WAKEUP_MASK 0x03 // Power control register Wakeup mask

#define ADXL_345_POWER_CTL_WAKEUP_8HZ 0x00 // Power control register Wakeup 8Hz
#define ADXL_345_POWER_CTL_WAKEUP_4HZ 0x01 // Power control register Wakeup 4Hz
#define ADXL_345_POWER_CTL_WAKEUP_2HZ 0x02 // Power control register Wakeup 2Hz
#define ADXL_345_POWER_CTL_WAKEUP_1HZ 0x03 // Power control register Wakeup 1Hz

#define ADXL_345_INT_ENABLE_DATA_READY_MASK 0x80 // Interrupt enable register Data Ready bit mask
#define ADXL_345_INT_ENABLE_SINGLE_TAP_MASK 0x40 // Interrupt enable register Single Tap bit mask
#define ADXL_345_INT_ENABLE_DOUBLE_TAP_MASK 0x20 // Interrupt enable register Double Tap bit mask
#define ADXL_345_INT_ENABLE_ACTIVITY_MASK 0x10 // Interrupt enable register Activity bit mask
#define ADXL_345_INT_ENABLE_INACTIVITY_MASK 0x08 // Interrupt enable register Inactivity bit mask
#define ADXL_345_INT_ENABLE_FREE_FALL_MASK 0x04 // Interrupt enable register Free Fall bit mask
#define ADXL_345_INT_ENABLE_WATERMARK_MASK 0x02 // Interrupt enable register Watermark bit mask
#define ADXL_345_INT_ENABLE_OVERRUN_MASK 0x01 // Interrupt enable register Overrun bit mask

#define ADXL_345_INT_MAP_DATA_READY_MASK 0x80 // Interrupt mapping register Data Ready bit mask
#define ADXL_345_INT_MAP_SINGLE_TAP_MASK 0x40 // Interrupt mapping register Single Tap bit mask
#define ADXL_345_INT_MAP_DOUBLE_TAP_MASK 0x20 // Interrupt mapping register Double Tap bit mask
#define ADXL_345_INT_MAP_ACTIVITY_MASK 0x10 // Interrupt mapping register Activity bit mask
#define ADXL_345_INT_MAP_INACTIVITY_MASK 0x08 // Interrupt mapping register Inactivity bit mask
#define ADXL_345_INT_MAP_FREE_FALL_MASK 0x04 // Interrupt mapping register Free Fall bit mask
#define ADXL_345_INT_MAP_WATERMARK_MASK 0x02 // Interrupt mapping register Watermark bit mask
#define ADXL_345_INT_MAP_OVERRUN_MASK 0x01 // Interrupt mapping register Overrun bit mask

#define ADXL_345_INT_SOURCE_DATA_READY_MASK 0x80 // Interrupt source register Data Ready bit mask
#define ADXL_345_INT_SOURCE_SINGLE_TAP_MASK 0x40 // Interrupt source register Single Tap bit mask
#define ADXL_345_INT_SOURCE_DOUBLE_TAP_MASK 0x20 // Interrupt source register Double Tap bit mask
#define ADXL_345_INT_SOURCE_ACTIVITY_MASK 0x10 // Interrupt source register Activity bit mask
#define ADXL_345_INT_SOURCE_INACTIVITY_MASK 0x08 // Interrupt source register Inactivity bit mask
#define ADXL_345_INT_SOURCE_FREE_FALL_MASK 0x04 // Interrupt source register Free Fall bit mask
#define ADXL_345_INT_SOURCE_WATERMARK_MASK 0x02 // Interrupt source register Watermark bit mask
#define ADXL_345_INT_SOURCE_OVERRUN_MASK 0x01 // Interrupt source register Overrun bit mask

#define ADXL_345_DATA_FORMAT_SELF_TEST_MASK 0x80 // Data format control register Self Test bit mask
#define ADXL_345_DATA_FORMAT_SPI_MASK 0x40 // Data format control register SPI bit mask
#define ADXL_345_DATA_FORMAT_INT_INVERT_MASK 0x20 // Data format control register Interrupt Invert bit mask
#define ADXL_345_DATA_FORMAT_FULL_RES_MASK 0x08 // Data format control register Full Resolution bit mask
#define ADXL_345_DATA_FORMAT_JUSTIFY_MASK 0x04 // Data format control register Justify bit mask
#define ADXL_345_DATA_FORMAT_RANGE_MASK 0x03 // Data format control register Range mask

#define ADXL_345_DATA_FORMAT_RANGE_2G 0x00 // Data format control register Range 2G
#define ADXL_345_DATA_FORMAT_RANGE_4G 0x01 // Data format control register Range 4G
#define ADXL_345_DATA_FORMAT_RANGE_8G 0x02 // Data format control register Range 8G
#define ADXL_345_DATA_FORMAT_RANGE_16G 0x03 // Data format control register Range 16G

#define ADXL_345_FIFO_CTL_FIFO_MODE_MASK 0xC0 // FIFO control register FIFO Mode mask
#define ADXL_345_FIFO_CTL_TRIGGER_MASK 0x20 // FIFO control register Trigger bit mask
#define ADXL_345_FIFO_CTL_SAMPLES_MASK 0x1F // FIFO control register Samples mask

#define ADXL_345_FIFO_CTL_FIFO_MODE_BYPASS 0x00 // FIFO control register Bypass mode
#define ADXL_345_FIFO_CTL_FIFO_MODE_FIFO   0x01 // FIFO control register FIFO mode
#define ADXL_345_FIFO_CTL_FIFO_MODE_STREAM 0x02 // FIFO control register Stream mode
#define ADXL_345_FIFO_CTL_FIFO_MODE_TRIGGER 0x03 // FIFO control register Trigger mode

#define ADXL_345_FIFO_STATUS_FIFO_READY_MASK 0x80 // FIFO status register FIFO Ready bit mask
#define ADXL_345_FIFO_STATUS_FIFO_ENTRIES_MASK 0x3F // FIFO status register FIFO Entries mask

class adxl_345
{
private:
    spi_inst_t *spi;
    uint64_t spi_frequency = 250000; // Default SPI frequency in Hz
    int cs_pin = 9;
    int sclk_pin = 10;
    int mosi_pin = 11;
    int miso_pin = 12;
    bool enabled = false;

    float gain_x = 0.0039;		// 3.9mg/LSB, full resolution
	float gain_y = 0.0039;		// 3.9mg/LSB, full resolution
	float gain_z = 0.0039;		// 3.9mg/LSB, full resolution

    enum adxl345_power_mode power_mode = adxl345_power_mode::STANDBY;
    enum adxl345_range range_settings = adxl345_range::RANGE_2G;
    enum adxl345_rate rate_settings = adxl345_rate::RATE_3200HZ;
    enum adxl345_fifo_mode fifo_mode = adxl345_fifo_mode::FIFO_BYPASS;
    bool full_resolution = false; // Full resolution mode
    uint8_t full_resolution_shift = 6;
    uint8_t watermark_level = 0x20; // Default watermark level for FIFO
    uint8_t interrupt_register = 0x00;

    float standard_g = 9.80665f; // Standard gravity in m/s^2
    const uint8_t read_mask = 0x80; // Read mask for SPI communication
    const uint8_t multibyte_mask = 0x40; // Multibyte mask for SPI communication
    const uint8_t nmaxvalues_fifo = 32; // Maximum number of values in FIFO
    const uint8_t bytes_per_axes = 2; // Bytes per axis for ADXL345 data
    const uint8_t bytes_per_all_axes = 6; // Total bytes for all axes data
    const uint8_t device_id = 0xE5; // Device ID for ADXL345

    const uint8_t addr_device        = 0x53;
    const uint8_t regaddr_devid      = 0x00;
    const uint8_t regaddr_acc        = 0x32;
    const uint8_t regaddr_freq       = 0x2C;
    const uint8_t regaddr_pwr        = 0x2D;
    const uint8_t regaddr_intsource  = 0x30;
    const uint8_t regaddr_grange     = 0x31;
    const uint8_t regaddr_fifoctl    = 0x38;
    const uint8_t regaddr_fifostatus = 0x39;

    uint8_t fifo_buffer[32*6];
    adxl345_raw_data_t fifo_raw_data[32]; // Raw data structure for FIFO
    adxl345_data_t fifo_int_data[32]; // Data structure for FIFO
    adxl345_float_data_t fifo_float_data[32]; // Float data structure for FIFO

    uint8_t raw_buffer[6];
    adxl345_raw_data_t raw_data; // Raw data structure for ADXL345
    adxl345_data_t int_data; // Data structure for ADXL345
    adxl345_float_data_t float_data; // Float data structure for ADXL345

    adxl345_register_map reg_map;

public:
    adxl_345(spi_inst_t *spi, int cs_pin = 9, int sclk_pin = 10, int mosi_pin = 11, int miso_pin = 12, uint64_t spi_frequency = 250000);
    ~adxl_345();

    void init();
    void write(adxl345_register reg, uint8_t value);
    void set(adxl345_register reg, uint8_t mask, uint8_t value);
    void read(adxl345_register reg, uint8_t *buffer, size_t length);
    uint8_t get(adxl345_register reg, uint8_t mask);
    bool is_adxl345_present();
    uint8_t read_interrupt_register();
    bool is_data_ready();
    bool is_fifo_ready();
    
    void set_power_mode(adxl345_power_mode mode);
    adxl345_power_mode get_power_mode();
    void set_range(adxl345_range range);
    adxl345_range get_range();
    void set_rate(adxl345_rate rate);
    adxl345_rate get_rate();
    void set_full_resolution(bool enable);
    bool is_full_resolution();
    void set_fifo_mode(adxl345_fifo_mode mode);
    adxl345_fifo_mode get_fifo_mode();
    void set_watermark_level(uint8_t level);
    uint8_t get_watermark_level();

    void set_interrupt(adxl345_interrupts interrupt, bool enable);
    bool is_interrupt_enabled(adxl345_interrupts interrupt);
    
    void read_single_raw(adxl345_raw_data_t *raw_data);
    void read_single(adxl345_data_t *data);
    void read_single_float(adxl345_float_data_t *data);
    void read_single_float_g(adxl345_float_data_t *data);
    void read_single_float_ms2(adxl345_float_data_t *data);

    void read_fifo_raw(adxl345_raw_data_t *raw_data, size_t length);
    void read_fifo(adxl345_data_t *data, size_t length);
    void read_fifo_float(adxl345_float_data_t *data, size_t length);
    void read_fifo_float_g(adxl345_float_data_t *data, size_t length);
    void read_fifo_float_ms2(adxl345_float_data_t *data, size_t length);
};

#endif // ADXL_345_H