#include <adxl_345.h>

adxl_345::adxl_345(spi_inst_t *spi, int cs_pin, int sclk_pin, int mosi_pin, int miso_pin, uint64_t spi_frequency)
    : spi(spi), cs_pin(cs_pin), sclk_pin(sclk_pin), mosi_pin(mosi_pin), miso_pin(miso_pin), spi_frequency(spi_frequency)
{
    // Initialize the SPI and GPIO pins here
    // This is a placeholder for actual initialization code
}

adxl_345::~adxl_345()
{
    // Cleanup code if necessary
}

void adxl_345::init()
{
    // Initialize the ADXL345 sensor
    // This is a placeholder for actual initialization code
    spi_init(spi, spi_frequency);
    spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(cs_pin, GPIO_FUNC_SIO);
    gpio_set_function(sclk_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);

    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // Set CS high (inactive)

    for (size_t i = 0; i < static_cast<size_t>(adxl345_register::REG_MAX); i++)
    {
        read(adxl345_register(i), &(reg_map.device_id) + i, 1);
        printf("Register %02X: %02X\n", adxl345_register_address_array[i], *(&(reg_map.device_id) + i));
    }
    
}

void adxl_345::write(adxl345_register reg, uint8_t value)
{
    uint8_t address = adxl345_register_address_array[static_cast<size_t>(reg)];
    gpio_put(cs_pin, 0); // Set CS low (active)
    spi_write_blocking(spi, &address, 1);
    spi_write_blocking(spi, &value, 1);
    gpio_put(cs_pin, 1); // Set CS high (inactive)
    *(&(reg_map.device_id) + static_cast<size_t>(reg)) = value;
}

void adxl_345::set(adxl345_register reg, uint8_t mask, uint8_t value)
{
    uint8_t current_value = *(&(reg_map.device_id) + static_cast<size_t>(reg));
    current_value = (current_value & ~mask) | ((value << __builtin_ctz(mask)) & mask); // Clear bits specified by mask and set new value
    write(reg, current_value);
}

void adxl_345::read(adxl345_register reg, uint8_t *buffer, size_t length)
{
    uint8_t address = adxl345_register_address_array[static_cast<size_t>(reg)];
    uint8_t reg_with_read = address | read_mask | (length > 1 ? multibyte_mask : 0); // Set read mask
    gpio_put(cs_pin, 0); // Set CS low (active)
    spi_write_blocking(spi, &reg_with_read, 1);
    spi_read_blocking(spi, 0, buffer, length);
    gpio_put(cs_pin, 1); // Set CS high (inactive)
    for (size_t i = 0; i < length; i++)
    {
        *(&(reg_map.device_id) + static_cast<size_t>(reg) + i) = buffer[i]; // Store read values in register map
    }
}

uint8_t adxl_345::get(adxl345_register reg, uint8_t mask)
{
    uint8_t value = *(&(reg_map.device_id) + static_cast<size_t>(reg));
    value &= mask; // Apply mask to the value
    value >>= __builtin_ctz(mask); // Shift right to get the masked value
    return value; // Return the masked value
}

bool adxl_345::is_adxl345_present()
{
    uint8_t device_id;
    read(adxl345_register::REG_DEVICE_ID, &device_id, 1);
    return (device_id == this->device_id);
}

uint8_t adxl_345::read_interrupt_register()
{
    read(adxl345_register::REG_INT_SOURCE, &interrupt_register, 1);
    return interrupt_register; // Return the interrupt source register value
}

bool adxl_345::is_data_ready()
{
    return (interrupt_register & adxl345_interrupts::INT_DATA_READY) != 0; // Check if data ready bit is set
}

bool adxl_345::is_fifo_ready()
{
    return (interrupt_register & adxl345_interrupts::INT_WATERMARK) != 0; // Check if FIFO is ready
}

void adxl_345::set_power_mode(adxl345_power_mode mode)
{
    power_mode = mode;
    set(adxl345_register::REG_POWER_CTL, ADXL_345_POWER_CTL_MEASURE_MASK, static_cast<uint8_t>(mode));
}

adxl345_power_mode adxl_345::get_power_mode()
{
    return power_mode;
}

void adxl_345::set_range(adxl345_range range)
{
    range_settings = range;
    set(adxl345_register::REG_DATA_FORMAT, ADXL_345_DATA_FORMAT_RANGE_MASK, static_cast<uint8_t>(range));
    full_resolution_shift = (range == adxl345_range::RANGE_2G) ? 6 : (range == adxl345_range::RANGE_4G) ? 5 : (range == adxl345_range::RANGE_8G) ? 4 : 3;
}

adxl345_range adxl_345::get_range()
{
    return range_settings;
}

void adxl_345::set_rate(adxl345_rate rate)
{
    rate_settings = rate;
    set(adxl345_register::REG_BW_RATE, ADXL_345_BW_RATE_RATE_MASK, static_cast<uint8_t>(rate));
}
adxl345_rate adxl_345::get_rate()
{
    return rate_settings;
}

void adxl_345::set_full_resolution(bool enable)
{
    full_resolution = enable;
    if (enable) {
        set(adxl345_register::REG_DATA_FORMAT, ADXL_345_DATA_FORMAT_FULL_RES_MASK, 0x01); // Set full resolution bit
    } else {
        set(adxl345_register::REG_DATA_FORMAT, ADXL_345_DATA_FORMAT_FULL_RES_MASK, 0x00); // Clear full resolution bit
    }
}

bool adxl_345::is_full_resolution()
{
    return full_resolution;
}
void adxl_345::set_fifo_mode(adxl345_fifo_mode mode)
{
    fifo_mode = mode;
    set(adxl345_register::REG_FIFO_CTL, ADXL_345_FIFO_CTL_FIFO_MODE_MASK, static_cast<uint8_t>(mode));
}
adxl345_fifo_mode adxl_345::get_fifo_mode()
{
    return fifo_mode;
}
void adxl_345::set_watermark_level(uint8_t level)
{
    watermark_level = level;
    set(adxl345_register::REG_FIFO_CTL, ADXL_345_FIFO_CTL_SAMPLES_MASK, level & 0x1F); // Set watermark level (5 bits)
}
uint8_t adxl_345::get_watermark_level()
{
    return watermark_level;
}
void adxl_345::set_interrupt(adxl345_interrupts interrupt, bool enable)
{
    uint8_t current_value = get(adxl345_register::REG_INT_ENABLE, 0xFF);
    if (enable) {
        current_value |= static_cast<uint8_t>(interrupt); // Enable the interrupt
    } else {
        current_value &= ~static_cast<uint8_t>(interrupt); // Disable the interrupt
    }
    write(adxl345_register::REG_INT_ENABLE, current_value);
}
bool adxl_345::is_interrupt_enabled(adxl345_interrupts interrupt)
{
    uint8_t current_value = get(adxl345_register::REG_INT_ENABLE, 0xFF);
    return (current_value & static_cast<uint8_t>(interrupt)) != 0; // Check if the interrupt is enabled
}

void adxl_345::read_single_raw(adxl345_raw_data_t *raw_data)
{
    read(adxl345_register::REG_DATAX0, raw_buffer, bytes_per_all_axes);

    raw_data->x_low = raw_buffer[0];
    raw_data->x_high = raw_buffer[1];
    raw_data->y_low = raw_buffer[2];
    raw_data->y_high = raw_buffer[3];
    raw_data->z_low = raw_buffer[4];
    raw_data->z_high = raw_buffer[5];
}

void adxl_345::read_single(adxl345_data_t *data)
{
    read_single_raw(&raw_data);

    data->x = (static_cast<int16_t>(raw_data.x_low) | (static_cast<int16_t>(raw_data.x_high) << 8));
    data->y = (static_cast<int16_t>(raw_data.y_low) | (static_cast<int16_t>(raw_data.y_high) << 8));
    data->z = (static_cast<int16_t>(raw_data.z_low) | (static_cast<int16_t>(raw_data.z_high) << 8));
}

void adxl_345::read_single_float(adxl345_float_data_t *data)
{
    read_single_raw(&raw_data);

    data->x = static_cast<float>(static_cast<int16_t>(raw_data.x_low) | (static_cast<int16_t>(raw_data.x_high) << 8));
    data->y = static_cast<float>(static_cast<int16_t>(raw_data.y_low) | (static_cast<int16_t>(raw_data.y_high) << 8));
    data->z = static_cast<float>(static_cast<int16_t>(raw_data.z_low) | (static_cast<int16_t>(raw_data.z_high) << 8));
}

void adxl_345::read_single_float_g(adxl345_float_data_t *data)
{
    read_single(&int_data);

    float range_factor = 1.0f;
    if (!full_resolution) {
        switch (range_settings) {
        case adxl345_range::RANGE_2G:
            range_factor = 1.0f;
            break;
        case adxl345_range::RANGE_4G:
            range_factor = 2.0f;
            break;
        case adxl345_range::RANGE_8G:
            range_factor = 4.0f;
            break;
        case adxl345_range::RANGE_16G:
            range_factor = 8.0f;
            break;
        }
    }

    data->x = static_cast<float>(int_data.x) * gain_x * range_factor;
    data->y = static_cast<float>(int_data.y) * gain_y * range_factor;
    data->z = static_cast<float>(int_data.z) * gain_z * range_factor;
}

void adxl_345::read_single_float_ms2(adxl345_float_data_t *data)
{
    read_single_float_g(data);
    data->x *= standard_g; // Convert to m/s^2
    data->y *= standard_g; // Convert to m/s^2
    data->z *= standard_g; // Convert to m/s^2
}

void adxl_345::read_fifo_raw(adxl345_raw_data_t *raw_data, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        read(adxl345_register::REG_DATAX0, fifo_buffer + i * bytes_per_all_axes, bytes_per_all_axes);
        raw_data[i].x_low = fifo_buffer[i * bytes_per_all_axes];
        raw_data[i].x_high = fifo_buffer[i * bytes_per_all_axes + 1];
        raw_data[i].y_low = fifo_buffer[i * bytes_per_all_axes + 2];
        raw_data[i].y_high = fifo_buffer[i * bytes_per_all_axes + 3];
        raw_data[i].z_low = fifo_buffer[i * bytes_per_all_axes + 4];
        raw_data[i].z_high = fifo_buffer[i * bytes_per_all_axes + 5];
    }
}

void adxl_345::read_fifo(adxl345_data_t *data, size_t length)
{
    read_fifo_raw(fifo_raw_data, length);
    for (size_t i = 0; i < length; i++) {
        data[i].x = (static_cast<int16_t>(fifo_raw_data[i].x_low) | (static_cast<int16_t>(fifo_raw_data[i].x_high) << 8));
        data[i].y = (static_cast<int16_t>(fifo_raw_data[i].y_low) | (static_cast<int16_t>(fifo_raw_data[i].y_high) << 8));
        data[i].z = (static_cast<int16_t>(fifo_raw_data[i].z_low) | (static_cast<int16_t>(fifo_raw_data[i].z_high) << 8));
    }
}
void adxl_345::read_fifo_float(adxl345_float_data_t *data, size_t length)
{
    read_fifo(fifo_int_data, length);
    for (size_t i = 0; i < length; i++) {
        data[i].x = static_cast<float>(fifo_int_data[i].x) * gain_x;
        data[i].y = static_cast<float>(fifo_int_data[i].y) * gain_y;
        data[i].z = static_cast<float>(fifo_int_data[i].z) * gain_z;
    }
}
void adxl_345::read_fifo_float_g(adxl345_float_data_t *data, size_t length)
{
    read_fifo_float(data, length);
    float range_factor = 1.0f;
    if (!full_resolution) {
        switch (range_settings) {
        case adxl345_range::RANGE_2G:
            range_factor = 1.0f;
            break;
        case adxl345_range::RANGE_4G:
            range_factor = 2.0f;
            break;
        case adxl345_range::RANGE_8G:
            range_factor = 4.0f;
            break;
        case adxl345_range::RANGE_16G:
            range_factor = 8.0f;
            break;
        }
    }
    for (size_t i = 0; i < length; i++) {
        data[i].x *= gain_x * range_factor;
        data[i].y *= gain_y * range_factor;
        data[i].z *= gain_z * range_factor;
    }
}
void adxl_345::read_fifo_float_ms2(adxl345_float_data_t *data, size_t length)
{
    read_fifo_float_g(data, length);
    for (size_t i = 0; i < length; i++) {
        data[i].x *= standard_g; // Convert to m/s^2
        data[i].y *= standard_g; // Convert to m/s^2
        data[i].z *= standard_g; // Convert to m/s^2
    }
}