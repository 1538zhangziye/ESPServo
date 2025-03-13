#include "PWMServoDriver.h"

static esp_err_t i2c_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    return i2c_master_transmit(dev_handle, buffer, 2, 1000);
}

static esp_err_t i2c_read_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, value, 1, 1000);
}

void PWMServoDriver_setPWMFreq(i2c_master_dev_handle_t dev_handle, float freq)
{
    i2c_write_byte(dev_handle, PCA9685_MODE1, 0x00); // Wake up
    if (freq < 1)
        freq = 1;
    if (freq > 3500)
        freq = 3500;

    float prescaleval = (25000000.0 / (freq * 4096.0)) - 1;
    if (prescaleval < 3)
        prescaleval = 3;
    if (prescaleval > 255)
        prescaleval = 255;
    uint8_t prescale = (uint8_t)prescaleval;

    /*uint8_t oldmode;
    i2c_read_byte(dev_handle, PCA9685_MODE1, &oldmode);
    uint8_t newmode = (oldmode & ~0x10) | 0x10;             // sleep
    i2c_write_byte(dev_handle, PCA9685_MODE1, newmode);     // go to sleep
    i2c_write_byte(dev_handle, PCA9685_PRESCALE, prescale); // set the prescaler
    i2c_write_byte(dev_handle, PCA9685_MODE1, oldmode);
    vTaskDelay(pdMS_TO_TICKS(5));
    newmode |= 0x80; // restart
    i2c_write_byte(dev_handle, PCA9685_MODE1, newmode);*/
    i2c_write_byte(dev_handle, PCA9685_MODE1, 0x10); // Sleep
    i2c_write_byte(dev_handle, PCA9685_PRESCALE, prescale);
    i2c_write_byte(dev_handle, PCA9685_MODE1, 0x80); // Restart
}

void PWMServoDriver_setPWM(i2c_master_dev_handle_t dev_handle, uint8_t num, uint16_t on, uint16_t off)
{
    printf("setPWM: %d %d %d\n", num, on, off);

    // 计算寄存器地址
    uint8_t on_low = (uint8_t)(on & 0xFF);
    uint8_t on_high = (uint8_t)(on >> 8);
    uint8_t off_low = (uint8_t)(off & 0xFF);
    uint8_t off_high = (uint8_t)(off >> 8);

    // 写入 LEDn_ON_L 和 LEDn_ON_H
    i2c_write_byte(dev_handle, PCA9685_LED0_ON_L + 4 * num, on_low);
    i2c_write_byte(dev_handle, PCA9685_LED0_ON_L + 4 * num + 1, on_high);

    // 写入 LEDn_OFF_L 和 LEDn_OFF_H
    i2c_write_byte(dev_handle, PCA9685_LED0_OFF_L + 4 * num, off_low);
    i2c_write_byte(dev_handle, PCA9685_LED0_OFF_L + 4 * num + 1, off_high);
}

void PWMServoDriver_setPin(i2c_master_dev_handle_t dev_handle, uint8_t num, float angle)
{
    uint16_t val = (uint16_t)(angle * 490 / 180.0 + 127); // 0.5ms ~ 2.5ms  127 ~ 617
    val = val > 4095 ? 4095 : val;
    PWMServoDriver_setPWM(dev_handle, num, 0, val);
}
