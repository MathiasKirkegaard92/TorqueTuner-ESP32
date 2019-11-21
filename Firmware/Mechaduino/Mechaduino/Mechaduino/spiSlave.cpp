#include "spiSlave.h"

const uint8_t SIZE = 6;
const uint8_t CHECKSUMSIZE = 4;
const uint8_t tx_delay = 2;
volatile uint8_t rx_buffer[SIZE + CHECKSUMSIZE];
uint8_t tx_buffer[SIZE + CHECKSUMSIZE]; // 2 last data fields cannot be used.

// Fixed point format 12.4 (16/bit)
volatile bool rx_done = false;
uint8_t readIdx = 0, writeIdx = 0;

int spi_slave_rx_data()
{
	if (rx_done) {
		readIdx = 0;
		for (int k = 0; k < SIZE; k++) {
			// SerialUSB.print("data[ "); SerialUSB.print(k); SerialUSB.print("] : "); SerialUSB.println(rx_buffer[k]);
		}
		// SERCOM1->SPI.DATA.reg = tx_buffer[0];
		writeIdx = 0;
		// SERCOM1->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC | SERCOM_SPI_INTENSET_DRE;
		return 1;
	}
	else {
		return 0;
		// SerialUSB.println("not ready");
	}
}

void set_spi_slave_tx_data(uint8_t data[], uint8_t length) {
	for (int i = tx_delay; i < length + tx_delay; ++i)
	{
		tx_buffer[i] = data[i - tx_delay];
	}
}

int32_t calcsum(volatile uint8_t buf[], int length) {
	int32_t sum;
	for (int k = 0; k < length; k++) {
		sum += buf[k];
	}
	return sum;
}

void set_spi_slave_tx_data(float* val) {
	memcpy(tx_buffer, val, 4);
}

uint8_t * int16_to_bytes(int16_t val, uint8_t data[]) {
	data[0] = (val >> 8) & 0xff;
	data[1] = val & 0xff;
	return data;
}

fixed_point_t float_to_fixed(float val) {
	return (fixed_point_t)(round(val * (1 << FRACTIONAL_BITS)));
}

float fixed_to_float(fixed_point_t val) {
	return (float)val / (float)(1 << FRACTIONAL_BITS);
}

void SERCOM1_Handler()
{
//    In SPI Interrupt
	uint8_t interrupts = SERCOM1->SPI.INTFLAG.reg; //Read SPI interrupt register

// SPI SSL Interupt
	if (interrupts & (1 << 3)) // 1000 + 8
	{
		SERCOM1->SPI.INTFLAG.bit.SSL = 1; //clear slave select interrupt
		rx_done = false; // Check if this is correct!!
	}

// SPI Data Received Complete Interrupt
	if (interrupts & (1 << 2)) //0100 = 4
	{

		// SerialUSB.print("i : "); SerialUSB.println(i);
		rx_buffer[readIdx++] = SERCOM1->SPI.DATA.reg;
		if (readIdx == SIZE) {
			rx_done = true;
		}
		SERCOM1->SPI.INTFLAG.bit.RXC = 1; //clear receive complete interrupt
	}

// SPI Data Transmit Complete Interrupt
	if (interrupts & (1 << 1)) //0010 = 2
	{

		SERCOM1->SPI.INTFLAG.bit.TXC = 1; //clear receive complete interrupt
	}
// SPI Data Register Empty Interrupt
	if (interrupts & (1 << 0)) //0001 = 1
	{
		// SerialUSB.print("j : "); SerialUSB.println(j);
		SERCOM1->SPI.DATA.reg = tx_buffer[writeIdx++];
	}
}

void spi_slave_init()
{
	PORT->Group[PORTA].PINCFG[16].bit.PMUXEN = 0x1; //Enable Peripheral Multiplexing for SERCOM1 SPI PA18 Arduino PIN10
	PORT->Group[PORTA].PMUX[8].bit.PMUXE = 0x2; //SERCOM 1 is selected for peripherial use of this pad
	PORT->Group[PORTA].PINCFG[17].bit.PMUXEN = 0x1; //Enable Peripheral Multiplexing for SERCOM1 SPI PA18 Arduino PIN10
	PORT->Group[PORTA].PMUX[8].bit.PMUXO = 0x2; //SERCOM 1 is selected for peripherial use of this pad
	PORT->Group[PORTA].PINCFG[18].bit.PMUXEN = 0x1; //Enable Peripheral Multiplexing for SERCOM1 SPI PA18 Arduino PIN10
	PORT->Group[PORTA].PMUX[9].bit.PMUXE = 0x2; //SERCOM 1 is selected for peripherial use of this pad
	PORT->Group[PORTA].PINCFG[19].bit.PMUXEN = 0x1; //Enable Peripheral Multiplexing for SERCOM1 SPI PA18 Arduino PIN10
	PORT->Group[PORTA].PMUX[9].bit.PMUXO = 0x2; //SERCOM 1 is selected for peripherial use of this pad


	//Disable SPI 1
	SERCOM1->SPI.CTRLA.bit.ENABLE = 0;
	while (SERCOM1->SPI.SYNCBUSY.bit.ENABLE);

	//Reset SPI 1
	SERCOM1->SPI.CTRLA.bit.SWRST = 1;
	while (SERCOM1->SPI.CTRLA.bit.SWRST || SERCOM1->SPI.SYNCBUSY.bit.SWRST);

	//Setting up NVIC
	NVIC_EnableIRQ(SERCOM1_IRQn);
	NVIC_SetPriority(SERCOM1_IRQn, 2);

	//Setting Generic Clock Controller!!!!
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM1_CORE) | //Generic Clock 0
	                    GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is the source
	                    GCLK_CLKCTRL_CLKEN; // Enable Generic Clock Generator

	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); //Wait for synchronisation


	//Set up SPI Control A Register in mode0
	SERCOM1->SPI.CTRLA.bit.DORD = 0; //MSB first
	SERCOM1->SPI.CTRLA.bit.CPOL = 0; //SCK is low when idle, leading edge is rising edge
	SERCOM1->SPI.CTRLA.bit.CPHA = 0; //data sampled on leading sck edge and changed on a trailing sck edge
	SERCOM1->SPI.CTRLA.bit.FORM = 0x0; //Frame format = SPI
	SERCOM1->SPI.CTRLA.bit.DIPO = 0; //DATA PAD0 MOSI is used as input (slave mode)
	SERCOM1->SPI.CTRLA.bit.DOPO = 0x2; //DATA PAD3 MISO is used as output
	SERCOM1->SPI.CTRLA.bit.MODE = 0x2; //SPI in Slave mode
	SERCOM1->SPI.CTRLA.bit.IBON = 0x1; //Buffer Overflow notification
	SERCOM1->SPI.CTRLA.bit.RUNSTDBY = 1; //wake on receiver complete

	//Set up SPI control B register
	//SERCOM1->SPI.CTRLB.bit.RXEN = 0x1; //Enable Receiver
	SERCOM1->SPI.CTRLB.bit.SSDE = 0x1; //Slave Selecte Detection Enabled
	SERCOM1->SPI.CTRLB.bit.CHSIZE = 0; //character size 8 Bit
	//SERCOM1->SPI.CTRLB.bit.PLOADEN = 0x1; //Enable Preload Data Register
	//while (SERCOM1->SPI.SYNCBUSY.bit.CTRLB);

	//Set up SPI interrupts
	SERCOM1->SPI.INTENSET.bit.SSL = 0x1; //Enable Slave Select low interrupt
	SERCOM1->SPI.INTENSET.bit.RXC = 0x1; //Receive complete interrupt
	SERCOM1->SPI.INTENSET.bit.TXC = 0x1; //Transmit complete interrupt
	SERCOM1->SPI.INTENSET.bit.ERROR = 0x1; //Receive complete interrupt
	SERCOM1->SPI.INTENSET.bit.DRE = 0x1; //Data Register Empty interrupt
	//init SPI CLK
	//SERCOM1->SPI.BAUD.reg = SERCOM_FREQ_REF / (2*4000000u)-1;
	//Enable SPI
	SERCOM1->SPI.CTRLA.bit.ENABLE = 1;
	while (SERCOM1->SPI.SYNCBUSY.bit.ENABLE);
	SERCOM1->SPI.CTRLB.bit.RXEN = 0x1; //Enable Receiver, this is done here due to errate issue
	while (SERCOM1->SPI.SYNCBUSY.bit.CTRLB); //wait until receiver is enabled

}


