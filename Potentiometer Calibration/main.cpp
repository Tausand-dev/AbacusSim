#include "Serial/serial.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

volatile UART serial(57600);

// const uint16_t THOLD[16] = {64, 236, 380, 499, 598, 680, 749, 806,
// 	 				853, 893, 925, 952, 975, 994, 1010, 1023};

// const uint16_t THOLD[16] = {96, 192, 288, 384, 480, 576, 672, 768, 800,
// 	 									832, 864, 896, 928, 960, 992, 1023};

const uint16_t THOLD[16] = {85, 170, 256, 341, 426, 512, 597, 682, 725,
	 									768, 810, 853, 896, 938, 981, 1023};

void initADC(void)
{
	ADMUX = (1 << REFS0); // AREF = AVcc
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC Enable and prescaler of 128
}

uint16_t readADC(uint8_t channel)
{
  uint8_t mask = 0b00000111;
  channel &= mask;  // AND operation with 7
  ADMUX = (ADMUX & ~mask) | channel; // clears the bottom 3 bits before ORing

  ADCSRA |= (1 << ADSC); // start single convertion

  while(ADCSRA & (1<<ADSC));
  return (ADC);
}

uint8_t getRange(uint8_t channel)
{
	uint8_t i;
	uint16_t adc = readADC(channel);
	for(i = 0; i < 16; i++)
	{
		if(adc <= THOLD[i])
		{
			return i;
		}
	}
 return 0xFF;
}

int main(void)
{
	uint16_t adc;
	uint8_t range;

	initADC();
	serial.setUART();
	sei();

	while(1)
	{
		adc = readADC(0);
		range = getRange(0);
		serial.write(adc);
		serial.print(", ");
		serial.write(range);
		serial.println("");
		_delay_ms(1000);
	}
	return 0;
}
