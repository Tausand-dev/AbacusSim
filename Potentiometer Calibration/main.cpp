#include "Serial/serial.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

volatile UART serial(57600);

const uint16_t THOLD[16] = {54, 109, 164, 219, 274, 329, 384, 438, 512,
															 585, 658, 731, 804, 877, 950, 1023};

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
