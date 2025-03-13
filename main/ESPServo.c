#include "driver/i2c_master.h"
#include "PWMServoDriver.h"
#include "esp_log.h"

void app_main()
{
    printf("Hello world!\n");
    // 初始化 I2C 主机
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_20,
        .scl_io_num = GPIO_NUM_21,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;                            // I2C 总线句柄
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle)); // 创建 I2C 总线

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9685_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev_handle;                                               // I2C 设备句柄
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle)); // 添加 I2C 设备

    // 设置 PWM 频率
    PWMServoDriver_setPWMFreq(dev_handle, 50); // 50 Hz for servo

    // 控制舵机角度
    while (1)
    {
        for(int i = 0; i < 10; i++)
            PWMServoDriver_setPin(dev_handle, i, 80);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 稍作延迟
        for(int i = 0; i < 10; i++)
            PWMServoDriver_setPin(dev_handle, i, 100);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 稍作延迟
    }
}
