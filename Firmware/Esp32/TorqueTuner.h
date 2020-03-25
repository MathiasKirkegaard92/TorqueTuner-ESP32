#ifndef __TORQUETUNER_H
#define __TORQUETUNER_H

// namespace i2c {
#define I2C_MASTER_0_NUM 0
#define I2C_MASTER_0_SCL_IO 25
#define I2C_MASTER_0_SDA_IO 26
#define I2C_MASTER_0_FREQ_HZ 1000000
#define I2C_MASTER_0_TIMEOUT 1048575   /*!< APB cycles (80 MhZ) Default is 640 */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

// #define I2C_DATA_MODE_MSB_FIRST 0
// #define I2C_DATA_MODE_LSB_FIRST 1

// const uint8_t I2C_BUF_SIZE = 10;
// const uint8_t CHECKSUMSIZE = 2;

#include <stdint.h>
#include "esp_err.h"

class HapticDev
{
public:
    uint8_t i2c_adress = 0x08;
    void init(int i2c_adress_, int i2c_gpio_sda, int i2c_gpio_scl);
    esp_err_t initI2C(int i2c_freq, int i2c_gpio_sda, int i2c_gpio_scl);
    esp_err_t write(uint8_t* tx_data, uint8_t length);
    esp_err_t read(uint8_t* rx_data, uint8_t length);
};

// }
#endif
