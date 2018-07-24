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

char text_buffer[32];

const uint16_t RANGES[16] = {0, 31250, 15625, 7102, 3397, 1645, 772, 368, 176, 84, 40, 19, 9, 4, 2, 1};
const uint16_t FREQS[16] = {0, 1, 2, 4, 9, 19, 40, 85, 178, 372, 781, 1645, 3472, 7813, 15625, 31250};

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
 return readADC(channel) / 64;
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
