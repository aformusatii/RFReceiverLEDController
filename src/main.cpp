/********************************************************************************
	Includes
********************************************************************************/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>

#include "../nrf24l01/RF24.h"
#include "../atmega328/mtimer.h"
#include "../common/util.h"

extern "C" {
#include "../atmega328/usart.h"
}

/********************************************************************************
	Macros and Defines
********************************************************************************/

/********************************************************************************
	Function Prototypes
********************************************************************************/
void initGPIO();
void initTimers();
void fixZeroValueOCR();

/********************************************************************************
	Global Variables
********************************************************************************/
RF24 radio;
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

volatile uint16_t index_intr = 0;


volatile uint8_t SET_TCCR0A = 0
	    |(1<<COM0A1)    // Bits 7:6 – COM0A1:0: Set OC0A on Compare Match
	    |(1<<COM0A0)    //
	    |(1<<COM0B1)    // Bits 5:4 – COM0B1:0: Set OC0B on Compare Match
	    |(1<<COM0B0)
	    |(0<<WGM01)     // Bits 1:0 – WGM01:0: Normal
	    |(0<<WGM00)
	    ;

volatile uint8_t CLEAR_TCCR0A = 0
	    |(1<<COM0A1)    // Bits 7:6 – COM0A1:0: Clear OC0A on Compare Match
	    |(0<<COM0A0)    //
	    |(1<<COM0B1)    // Bits 5:4 – COM0B1:0: Clear OC0B on Compare Match
	    |(0<<COM0B0)
	    |(0<<WGM01)     // Bits 1:0 – WGM01:0: Normal
	    |(0<<WGM00)
	    ;

volatile uint8_t SET_TCCR2A = 0
	    |(0<<COM2A1)    // Bits 7:6 – COM2A1:0: Normal port operation, OC2A disconnected.
	    |(0<<COM2A0)    //
	    |(1<<COM2B1)    // Bits 5:4 – COM2B1:0: Set OC2B on Compare Match
	    |(1<<COM2B0)
	    |(0<<WGM21)     // Bits 1:0 – WGM21:0: Normal
	    |(0<<WGM20)
	    ;

volatile uint8_t CLEAR_TCCR2A = 0
		|(0<<COM2A1)    // Bits 7:6 – COM2A1:0: Normal port operation, OC2A disconnected.
		|(0<<COM2A0)    //
	    |(1<<COM2B1)    // Bits 5:4 – COM2B1:0: Clear OC2B on Compare Match
	    |(0<<COM2B0)
	    |(0<<WGM21)     // Bits 1:0 – WGM21:0: Normal
	    |(0<<WGM20)
	    ;

/********************************************************************************
	Interrupt Service
********************************************************************************/
ISR(USART_RX_vect)
{
	handle_usart_interrupt();
}

ISR(TIMER1_OVF_vect)
{
	incrementOvf();
}

ISR(INT0_vect)
{

	printf("\n int[%d]", index_intr++);

    bool tx_ok, tx_fail, rx_ok;
    radio.whatHappened(tx_ok, tx_fail, rx_ok);

    if (rx_ok) {

        uint8_t data[50];
        uint8_t len = radio.getDynamicPayloadSize();
        radio.read(data, len);

        OCR0A = data[0];
		OCR0B = data[1];
		OCR2B = data[2];

		fixZeroValueOCR();

        radio.flush_rx();
    }
}

ISR(TIMER0_OVF_vect)
{

	TCCR0A = SET_TCCR0A;

	TCNT0 = 0;

	_delay_us(2);

	TCCR0B |= (1<<FOC0A)|(1<<FOC0B); // Force Output Compare

	TCCR0A = CLEAR_TCCR0A;
}

ISR(TIMER2_OVF_vect)
{

	TCCR2A = SET_TCCR2A;

	TCNT2 = 0;

	TCCR2B |= (1<<FOC2A)|(1<<FOC2B); // Force Output Compare

	TCCR2A = CLEAR_TCCR2A;
}

/********************************************************************************
	Main
********************************************************************************/
int main(void) {
    // initialize usart module
	usart_init();

    // enable interrupts
    sei();

    // Init GPIO
    initGPIO();

    // Init Timer 1
    initTimer();

    // Init Timer 0 & 2
    initTimers();

    _delay_ms(1000);

    fixZeroValueOCR();

	// Console friendly output
    printf("Start...");
    printf(CONSOLE_PREFIX);

    // Init NRF24L01+
    radio.begin();
    radio.setRetries(15,15);
    radio.setPayloadSize(8);
    radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(115);

    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);

    radio.startListening();

    // Some RF module diagnostics logs
    radio.printDetails();

	// main loop
    while (1) {
    	// main usart loop for console
    	usart_check_loop();
    }
}

/********************************************************************************
	Functions
********************************************************************************/
void initGPIO() {
    _in(DDD2, DDRD); // INT0 input

    // GPIO Interrupt INT0
    EICRA = (0<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00); // The falling edge of INT0 generates an interrupt request.
    EIMSK = (0<<INT1)|(1<<INT0); // Enable INT0

	_out(DDD3, DDRD); // OC2B
	_out(DDD5, DDRD); // OC0B
	_out(DDD6, DDRD); // OC0A

	_on(PD3, PORTD); // OC2B
	_on(PD5, PORTD); // OC0B
	_on(PD6, PORTD); // OC0A
}

void handle_usart_cmd(char *cmd, char *args) {
	if (strcmp(cmd, "test") == 0) {
		printf("\n TEST [%s]", args);
	}

	if (strcmp(cmd, "set1") == 0) {
		OCR0A = atoi(args);
		fixZeroValueOCR();
	}

	if (strcmp(cmd, "set2") == 0) {
		OCR0B = atoi(args);
		fixZeroValueOCR();
	}

	if (strcmp(cmd, "set3") == 0) {
		OCR2B = atoi(args);
		fixZeroValueOCR();
	}
}

void initTimers() {
	OCR0A = 0;
	OCR0B = 0;

	// Reset all timers
	TCNT0 = 0;

	// Setup Timer/Counter0
	TCCR0A = 0
		|(1<<COM0A1)    // Bits 7:6 – COM0A1:0: Clear OC0A on Compare Match
		|(0<<COM0A0)    //
		|(1<<COM0B1)    // Bits 5:4 – COM0B1:0: Clear OC0B on Compare Match
		|(0<<COM0B0)
	    |(0<<WGM01)     // Bits 1:0 – WGM01:0: Normal
	    |(0<<WGM00)
	    ;

	TCCR0B = 0
	    |(0<<FOC0A)     // Force Output Compare A
	    |(0<<FOC0B)     // Force Output Compare B
	    |(0<<WGM02)     // Bit 3 – WGM02: Waveform Generation Mode
	    |(0<<CS02)      // Bits 2:0 – CS02:0: Clock Select
	    |(1<<CS01)
	    |(1<<CS00)      // clkT2S/64 (From prescaler)
	    ;

	_on(TOIE0, TIMSK0);

	_delay_ms(10);

	OCR2A = 0;
	OCR2B = 0;

	TCNT2 = 0;

	// Setup Timer/Counter2
	TCCR2A = 0
	    |(0<<COM2A1)    // Bits 7:6 – COM2A1:0: Normal port operation, OC2A disconnected.
	    |(0<<COM2A0)    //
	    |(1<<COM2B1)    // Bits 5:4 – COM2B1:0: Clear OC2B on Compare Match
	    |(0<<COM2B0)
	    |(0<<WGM21)     // Bits 1:0 – WGM21:0: Normal
	    |(0<<WGM20)
	    ;

	TCCR2B = 0
	    |(0<<FOC2A)     // Force Output Compare A
	    |(0<<FOC2B)     // Force Output Compare B
	    |(0<<WGM22)     // Bit 3 – WGM22: Normal
	    |(1<<CS22)      // Bits 2:0 – CS22:0: Clock Select
	    |(0<<CS21)
	    |(0<<CS20)      // clkT2S/64 (From prescaler)
	    ;

	_on(TOIE2, TIMSK2);
}

void fixZeroValueOCR() {
	if (OCR0A == 0) {
		SET_TCCR0A &= ~( _BV(COM0A1) | _BV(COM0A0) );
		CLEAR_TCCR0A &= ~( _BV(COM0A1) | _BV(COM0A0) );
		_off(PD6, PORTD);
	} else {
		SET_TCCR0A |= (1<<COM0A1) | (1<<COM0A0);
		CLEAR_TCCR0A |= (1<<COM0A1) | (0<<COM0A0);
	}

	if (OCR0B == 0) {
		SET_TCCR0A &= ~( _BV(COM0B1) | _BV(COM0B0) );
		CLEAR_TCCR0A &= ~( _BV(COM0B1) | _BV(COM0B0) );
		_off(PD5, PORTD);
	} else {
		SET_TCCR0A |= (1<<COM0B1) | (1<<COM0B0);
		CLEAR_TCCR0A |= (1<<COM0B1) | (0<<COM0B0);
	}

	if (OCR2B == 0) {
		SET_TCCR2A &= ~( _BV(COM2B1) | _BV(COM2B0) );
		CLEAR_TCCR2A &= ~( _BV(COM2B1) | _BV(COM2B0) );
		_off(PD3, PORTD);
	} else {
		SET_TCCR2A |= (1<<COM2B1) | (1<<COM2B0);
		CLEAR_TCCR2A |= (1<<COM2B1) | (0<<COM2B0);
	}
}
