/*******************************************************************************
Configuration for the Mechaduino based rotary force feedback device TorqueTuner
 *******************************************************************************/


#include "TorqueTuner.h"
#include "driver/i2c.h"
#include <cstring> // memcpy

HapticDev::HapticDev(int i2c_adress_) {
	i2c_adress = i2c_adress_;
}

void HapticDev::init() {
	initI2C(I2C_MASTER_0_FREQ_HZ, 21, 22);
	uint8_t data[8];
	char mode = 't'; // Set to torque mode
	memcpy(data + 6, &mode, 1);
	I2C_write(data, 1);
}

void HapticDev::initI2C(int i2c_freq, int i2c_gpio_sda, int i2c_gpio_scl) {
	i2c_port_t i2c_master_port = (i2c_port_t) I2C_MASTER_0_NUM;
	i2c_config_t i2c_config;
	i2c_config.mode = I2C_MODE_MASTER;
	i2c_config.sda_io_num = (gpio_num_t) i2c_gpio_sda;
	i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
	i2c_config.scl_io_num = (gpio_num_t) i2c_gpio_scl;
	i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
	i2c_config.master.clk_speed = i2c_freq;

	i2c_param_config(i2c_master_port, &i2c_config);
	i2c_driver_install(i2c_master_port, i2c_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void HapticDev::I2C_write(uint8_t* tx_data, uint8_t length) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2c_adress << 1) | I2C_MASTER_WRITE, (i2c_ack_type_t)ACK_CHECK_EN);
	i2c_master_write(cmd, tx_data, length, (i2c_ack_type_t)ACK_CHECK_EN);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin((i2c_port_t) I2C_MASTER_0_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
}

void HapticDev::I2C_read(uint8_t* rx_data, uint8_t length) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2c_adress << 1) | READ_BIT, (i2c_ack_type_t)ACK_CHECK_EN);
	if (length > 1) {
		i2c_master_read(cmd, rx_data, length - 1, (i2c_ack_type_t)ACK_VAL);
	}
	i2c_master_read_byte(cmd, rx_data + length - 1, (i2c_ack_type_t)NACK_VAL);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin((i2c_port_t) I2C_MASTER_0_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
}