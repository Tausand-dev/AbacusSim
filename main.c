#include "lcd.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define DDR_A DDRB
#define DDR_B DDRB
#define DDR_C DDRB
#define DDR_D DDRB

#define PORT_A PORTB
#define PORT_B PORTB
#define PORT_C PORTB
#define PORT_D PORTB

#define PIN_A PB1
#define PIN_B PB2
#define PIN_C PB3
#define PIN_D PB4

#define RANGE0 0
#define RANGE1 31250
#define RANGE2 6250
#define RANGE3 279
#define RANGE4 56
#define RANGE5 12
#define RANGE6 3
#define RANGE7 1

#define FREQ0 0
#define FREQ1 1
#define FREQ2 5
#define FREQ3 112
#define FREQ4 558
#define FREQ5 2604
#define FREQ6 10417
#define FREQ7 31250

char text_buffer[32];

volatile uint16_t counterA, counterB, counterC, counterD;
volatile uint16_t interA, interB, interC, interD;

ISR(TIMER0_OVF_vect)
{
	counterA += 1;
	counterB += 1;
	counterC += 1;
	counterD += 1;

	if((counterA == interA) & (interA > 0))
	{
		PORT_A ^= (1 << PIN_A);
		counterA = 0;
	}
	if((counterB == interB) & (interB > 0))
	{
		PORT_B ^= (1 << PIN_B);
		counterB = 0;
	}
	if((counterC == interC) & (interC > 0))
	{
		PORT_C ^= (1 << PIN_C);
		counterC = 0;
	}
	if((counterD == interD) & (interD > 0))
	{
		PORT_D ^= (1 << PIN_D);
		counterD = 0;
	}
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
	DDR_A |= (1 << PIN_A);
	DDR_B |= (1 << PIN_B);
	DDR_C |= (1 << PIN_C);
	DDR_D |= (1 << PIN_D);
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
	return readADC(channel) / 128;
}

uint16_t getInterval(uint8_t range)
{
	if(range == 0)
	{
		return RANGE0;
	}
	else if(range == 1)
	{
		return RANGE1;
	}
	else if(range == 2)
	{
		return RANGE2;
	}
	else if(range == 3)
	{
		return RANGE3;
	}
	else if(range == 4)
	{
		return RANGE4;
	}
	else if(range == 5)
	{
		return RANGE5;
	}
	else if(range == 6)
	{
		return RANGE6;
	}
	else
	{
		return RANGE7;
	}
}

uint16_t getFreq(uint8_t range)
{
	if(range == 0)
	{
		return FREQ0;
	}
	else if(range == 1)
	{
		return FREQ1;
	}
	else if(range == 2)
	{
		return FREQ2;
	}
	else if(range == 3)
	{
		return FREQ3;
	}
	else if(range == 4)
	{
		return FREQ4;
	}
	else if(range == 5)
	{
		return FREQ5;
	}
	else if(range == 6)
	{
		return FREQ6;
	}
	else
	{
		return FREQ7;
	}
}

void resetCounters(void)
{
	// counters to 0
	counterA = 0;
	counterB = 0;
	counterC = 0;
	counterD = 0;

	// start low
	PORT_A &= ~(1 << PIN_A);
	PORT_B &= ~(1 << PIN_B);
	PORT_C &= ~(1 << PIN_C);
	PORT_D &= ~(1 << PIN_D);
}

void welcomeScreen(void)
{
	lcd_home();
	snprintf(text_buffer, sizeof(text_buffer), "Tausand\n  AbacusSim v4 ");
	lcd_puts(text_buffer);
	_delay_ms(2000);
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
	uint8_t reset, cleanLCD;
	uint16_t cA, cB, cC, cD;
	uint16_t iA, iB, iC, iD;
	uint16_t fA = 0, fB = 0, fC = 0, fD = 0;
	lcd_init(LCD_DISP_ON_BLINK);

	initADC();
	initTimer();
	initOuts();

	lcd_clrscr();
	resetCounters();

	welcomeScreen();
	toLCD(fA, fB, fC, fD);

	sei();

	while(1)
	{
		reset = 0;

		cA = getRange(0);
		cB = getRange(1);
		cC = getRange(2);
		cD = getRange(3);

		iA = getInterval(cA);
		iB = getInterval(cB);
		iC = getInterval(cC);
		// iD = getInterval(cD);

		if(iA != interA)
		{
			interA = iA;
			fA = getFreq(cA);
			reset = 1;
		}
		else if(iB != interB)
		{
			interB = iB;
			fB = getFreq(cB);
			reset = 1;
		}
		else if(iC != interC)
		{
			interC = iC;
			fC = getFreq(cC);
			reset = 1;
		}
		else if(iD != interD)
		{
			interD = iD;
			fD = getFreq(cD);
			reset = 1;
		}

		if(reset)
		{
			resetCounters();
			toLCD(fA, fB, fC, fD);
		}
		_delay_ms(100);
	}
	return 0;
}
