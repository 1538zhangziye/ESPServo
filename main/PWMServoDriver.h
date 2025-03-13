#ifndef _PWMServoDriver_H
#define _PWMServoDriver_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define PCA9685_I2C_ADDRESS 0x40 /**< Default PCA9685 I2C Slave Address */
#define PCA9685_MODE1 0x00       /**< Mode Register 1 */
#define PCA9685_PRESCALE 0xFE    /**< Prescaler for PWM output frequency */
#define PCA9685_LED0_ON_L 0x06   /**< LED0 on tick, low byte */
#define PCA9685_LED0_OFF_L 0x08  /**< LED0 off tick, low byte */

void PWMServoDriver_setPWMFreq(i2c_master_dev_handle_t dev_handle, float freq);
void PWMServoDriver_setPWM(i2c_master_dev_handle_t dev_handle, uint8_t num, uint16_t on, uint16_t off);
void PWMServoDriver_setPin(i2c_master_dev_handle_t dev_handle, uint8_t num, float angle);

#endif
