// Initializises interrupt based spi slave for recieving and transmitting

#ifndef __SPISLAVE_H__
#define __SPISLAVE_H__

#include <SPI.h>
#include <Arduino.h>


extern const uint8_t SIZE;
extern const uint8_t CHECKSUMSIZE;
extern volatile uint8_t rx_buffer[];
extern uint8_t tx_buffer[];

extern volatile bool rx_done;
extern uint8_t readIdx;
extern uint8_t writeIdx;

typedef uint16_t fixed_point_t;
const int FRACTIONAL_BITS = 4;

int spi_slave_rx_data();
void set_spi_slave_tx_data(uint8_t data[], uint8_t length);
void set_spi_slave_tx_data(float* val);
uint8_t* int16_to_bytes(int16_t val, uint8_t data[]);
int32_t calcsum(volatile uint8_t buf[], int length);
void SERCOM1_Handler();
void spi_slave_init();
fixed_point_t float_to_fixed(float val);
float fixed_to_float(fixed_point_t val);

#endif