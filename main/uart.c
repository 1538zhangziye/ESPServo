/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/i2c_master.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "PWMServoDriver.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

static const char *TAG = "UART";

#define uart_num UART_NUM_0
#define BUF_SIZE (1024)

static void uart_task(void *arg);
i2c_master_dev_handle_t dev_handle; // I2C 设备句柄

void app_main()
{
    printf("Hello world!\n");
    // 初始化 I2C 主机
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_18,
        .scl_io_num = GPIO_NUM_17,
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
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle)); // 添加 I2C 设备

    // 设置 PWM 频率
    PWMServoDriver_setPWMFreq(dev_handle, 50); // 50 Hz for servo

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL); // 创建 UART 任务
}

static void uart_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(uart_num, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        // Write data back to the UART
        if (len)
        {
            uart_write_bytes(uart_num, (const char *)data, len);
            // data[len] = '\0';
            // ESP_LOGI(TAG, "Recv str: %s", data);
            int servo_num, servo_angle;
            if (sscanf((const char *)data, "%d %d", &servo_num, &servo_angle) == 2)
            {
                ESP_LOGI(TAG, "Parsed: Servo %d, Angle %d", servo_num, servo_angle);
                PWMServoDriver_setPin(dev_handle, servo_num, servo_angle);
            }
        }
    }
}
