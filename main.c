#include "lcd.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define PIN_A PB1
#define PIN_B PB2
#define PIN_C PB3
#define PIN_D PB4

char text_buffer[32];

const uint16_t RANGES[16] = {0, 31250, 15625, 7102, 3397, 1645, 772, 368, 176, 84, 40, 19, 9, 4, 2, 1};
const uint16_t FREQS[16] = {0, 1, 2, 4, 9, 19, 40, 85, 178, 372, 781, 1645, 3472, 7813, 15625, 31250};

const uint16_t THOLD[16] = {85, 170, 256, 341, 426, 512, 597, 682, 725,
	 									768, 810, 853, 896, 938, 981, 1023};


volatile uint16_t counterA, counterB, counterC, counterD;
volatile uint16_t interA, interB, interC, interD;

uint8_t port_bits;

ISR(TIMER0_OVF_vect)
{
	counterA += 1;
	counterB += 1;
	counterC += 1;
	counterD += 1;

	port_bits = PORTB;

	if((counterA == interA) & (interA > 0))
	{
		port_bits ^= (1 << PIN_A);
		counterA = 0;
	}
	if((counterB == interB) & (interB > 0))
	{
		port_bits ^= (1 << PIN_B);
		counterB = 0;
	}
	if((counterC == interC) & (interC > 0))
	{
		port_bits ^= (1 << PIN_C);
		counterC = 0;
	}
	if((counterD == interD) & (interD > 0))
	{
		port_bits ^= (1 << PIN_D);
		counterD = 0;
	}

	PORTB = port_bits;
}

void initADC(void)
{
	ADMUX = (1 << REFS0); // AREF = AVcc
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC Enable and prescaler of 128
}

void initTimer(void)
{
	TCCR0B |= (1 << CS00);
	TIMSK0 |= (1 << TOIE0);
}

void initOuts(void)
{
	DDRB |= (1 << PIN_A) | (1 << PIN_B) | (1 << PIN_C) | (1 << PIN_D);
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

// uint8_t getRange(uint8_t channel)
// {
//  return readADC(channel) / 64;
// }

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

void resetCounters(void)
{
	// counters to 0
	counterA = 0;
	counterB = 0;
	counterC = 0;
	counterD = 0;

	// start low
	PORTB &= ~((1 << PIN_A) | (1 << PIN_B) | (1 << PIN_C) | (1 << PIN_D));
}

void welcomeScreen(void)
{
	lcd_home();
	snprintf(text_buffer, sizeof(text_buffer), "Abacus\n Tester 30 kHz");
	lcd_puts(text_buffer);
	_delay_ms(3000);
	lcd_clrscr();
	lcd_init(LCD_DISP_ON);
	_delay_ms(500);
}

void toLCD(uint16_t freqA, uint16_t freqB, uint16_t freqC, uint16_t freqD)
{
	lcd_home();
	snprintf(text_buffer, sizeof(text_buffer), "A:%05d B:%05d\nC:%05d D:%05d", freqA, freqB, freqC, freqD);
	lcd_puts(text_buffer);
}

int main(void)
{
	uint8_t reset;
	uint16_t cA, cB, cC, cD;
	uint16_t iA, iB, iC, iD;
	lcd_init(LCD_DISP_ON_BLINK);

	initADC();
	initTimer();
	initOuts();

	lcd_clrscr();
	resetCounters();

	welcomeScreen();
	toLCD(0, 0, 0, 0);

	sei();

	while(1)
	{
		reset = 0;

		cA = getRange(0);
		cB = getRange(1);
		cC = getRange(2);
		cD = getRange(3);

		iA = RANGES[cA];
		iB = RANGES[cB];
		iC = RANGES[cC];
		iD = RANGES[cD];

		if(iA != interA)
		{
			interA = iA;
			reset = 1;
		}
		else if(iB != interB)
		{
			interB = iB;
			reset = 1;
		}
		else if(iC != interC)
		{
			interC = iC;
			reset = 1;
		}
		else if(iD != interD)
		{
			interD = iD;
			reset = 1;
		}

		if(reset)
		{
			resetCounters();
			toLCD(FREQS[cA], FREQS[cB], FREQS[cC], FREQS[cD]);
		}
		_delay_ms(100);
	}
	return 0;
}
