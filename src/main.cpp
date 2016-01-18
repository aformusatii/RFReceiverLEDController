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

	OCR0A = 255;
	OCR0B = 255;
	OCR2A = 255;
	OCR2B = 255;

    _delay_ms(1000);

	OCR0A = 0;
	OCR0B = 0;
	OCR2A = 0;
	OCR2B = 0;

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
    // The falling edge of INT0 generates an interrupt request.
    EICRA = (0<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00);
    // Enable INT0
    EIMSK = (1<<INT0);

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
	// Setup Timer/Counter0
	// Mode 3 - Fast PWM
	// Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode).
	TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
	// clkI/O/64 (From prescaler)
	TCCR0B = (0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);

	// Setup Timer/Counter2
	// Mode 3 - Fast PWM
	// Clear OC2B on Compare Match, set OC2B at BOTTOM (non-inverting mode)
	TCCR2A = (1<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (1<<WGM20);
	// clkT2S/64 (From prescaler)
	TCCR2B = (0<<WGM22) | (1<<CS22) | (0<<CS21) | (0<<CS20);
}

void fixZeroValueOCR() {
	if (OCR0A == 0) {
		TCCR0A &= ~( _BV(COM0A1) | _BV(COM0A0) );
		_off(PD6, PORTD);
	} else {
		TCCR0A |= (1<<COM0A1) | (0<<COM0A0);
	}

	if (OCR0B == 0) {
		TCCR0A &= ~( _BV(COM0B1) | _BV(COM0B0) );
		_off(PD5, PORTD);
	} else {
		TCCR0A |= (1<<COM0B1) | (0<<COM0B0);
	}

	if (OCR2B == 0) {
		TCCR2A &= ~( _BV(COM2B1) | _BV(COM2B0) );
		_off(PD3, PORTD);
	} else {
		TCCR2A |= (1<<COM2B1) | (0<<COM2B0);
	}
}
