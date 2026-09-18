#include "sensors_internal.h"

/*
 * Thin I2C register access shared by the IMU and encoder drivers. The board
 * module owns the two buses; this file only knows how to talk to a device on
 * one of them.
 */

esp_err_t sensors_add_device(i2c_master_bus_handle_t bus, uint8_t address,
                             i2c_master_dev_handle_t *device)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(bus, &config, device);
}

esp_err_t sensors_write_register(i2c_master_dev_handle_t device, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(device, data, sizeof(data), 100);
}

esp_err_t sensors_read_registers(i2c_master_dev_handle_t device, uint8_t reg,
                                 uint8_t *data, size_t length, int timeout_ms)
{
    return i2c_master_transmit_receive(device, &reg, 1, data, length, timeout_ms);
}
