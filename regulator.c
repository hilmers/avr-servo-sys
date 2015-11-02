#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>		// watch dog timer
#define BAUD 2400
#include <util/setbaud.h>

/* "Automatic" control parameters: */
#define K 2
#define T_i 3
#define T_r	1
#define beta 1

static void usart_init();
void usart_transmit(uint8_t data);
uint8_t usart_recieve();
void usart_interrupt_init();
void toggle_status_led(uint8_t debug_pin);
void watchdog_timer_init();
void pwm_init();
uint8_t saturate(int16_t val);
void set_pwm_output(uint8_t k);
void read_input();
void clockprescale();
static void inc_counter();
static void dec_counter();

/* Global variables */
static uint8_t max;
static uint8_t min;
static uint8_t old_state;
static uint8_t state;

static volatile uint16_t tic;
static volatile uint8_t u;
static volatile uint8_t y;
static double Integrator;
static uint8_t y_ref;
static double sample_h = 0.5;
static uint8_t temp = 0;
static uint16_t n = 0;

int main(void) {
	max = 255;
	min = 0;
	old_state =	0xFF;
	tic = 0;
	u =	0;
	y =	0;
	Integrator = 0;
	y_ref =	0;
	/* Tell me a reset occured */
	DDRB &= ~((1 << DDB1) | (1 << DDB2)); 
	// PB1,PB2 (PCINT0, PCINT1, PCINT2 pin) are now cleared and inputs

	/* Enable pull-up on PB1 and PB2 */
	PORTB |= ((1 << PORTB1) | (1 << PORTB2));
	
	/* Enables interrupts on Pins 16-23 and Pins 0-7*/
	PCICR |= (1 << PCIE0); 
	
	/* Triggers interrupt on PCINT1 and PCINT2 on state change, only need one? */
	PCMSK0 |= (1 << PCINT1)  | (1 << PCINT2);
	
	/* Enabled AC pins, used for debugging */
	DDRC = 0xFF; 
	
	/* Serial comm */
	usart_init();
	usart_interrupt_init();
	pwm_init();
	watchdog_timer_init();

	/* Check if reset was cause by Watchdog */
	if (MCUSR & (1 << WDRF)) {
		MCUSR &= ~(1 << WDRF); // Clear the WDT reset flag
		WDTCSR |= (1 << WDCE) | (1 << WDE); // Enable the WD change bit
		WDTCSR = 0x00; // Disable the WDT
		toggle_status_led(1); // Show me the error
	}
	
	PORTC = 0x01;
	sei();
	while(1) {
		read_input();
	}
	return 0;
}


void watchdog_timer_init(void) 
{
	cli();
	wdt_reset();
	/* Sets up the watchdog interrupt */
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	
	/* Starts the timer with 250ms prescaller */
	WDTCSR = (1 << WDIE) | (1 << WDP2) | (0 << WDP1) | (1 << WDP0);
	
	sei();
}

void pwm_init(void)
{
	TCCR0A	|= (1 << WGM00) | (1 << WGM01) | (1 << COM0B1);
	TCCR0B	|= (1 << CS00);
	DDRD	|= (1 << PD5);
}

static void usart_init(void)
{
	/* Set baud rate, from page 178 in datasheet */
	/* Constants from util/setbaud */
	UBRR0L = UBRRL_VALUE;
	UBRR0H = UBRRH_VALUE;

	/*Enable receiver and transmitter */
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0) | (1<< TXCIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1 << UCSZ01)|(1 << UCSZ00);
}

void usart_transmit(uint8_t data)
{
	/* Loops until transmit buffer is empty */
	while(!(UCSR0A & (1 << UDRE0))){
		;
	}

	/* Put data into buffer, a.k.a send the data */
	UDR0 = data;
}

uint8_t usart_recieve(void)
{
	while(!(UCSR0A & (1 << RXC0))){
		;
	}
	return UDR0;
}

void usart_interrupt_init(void) 
{
	UCSR0B |= (1<<RXCIE0);
}

void read_input()
{
	uint8_t input = usart_recieve();
	_delay_ms(5);
	if (input == 'r') {
		usart_transmit(y);
	} else {
		
		if (input <= 85 && input > 10) { 
			y_ref = input;
		} else {
			set_pwm_output(0);
			y_ref = 0;
		}
	}
}

void toggle_status_led(uint8_t debug_pin) 
{
	if (PORTC & 1 << debug_pin) {
		_delay_ms(30);
		PORTC &= ~(1 << debug_pin);
	} else {
		PORTC |= 1 << debug_pin;
	}
}

uint8_t saturate(int16_t val)
{
	if (val > max) return max;
	if (val < min) return min;
	return (uint8_t) val;
}

void set_pwm_output(uint8_t k) 
{
	OCR0B = 255 - k;
}

ISR(PCINT0_vect) 
{
	
 	/* Rene Sommer algorithm */
 	
 	state = PINB & ((1 << PB1) | (1 << PB2)); 
 	state = state >> PB1;
 	
 	switch (state) {
 		case 0 : if (old_state == 1) n--; else n++; break;
 		case 1 : if (old_state == 3) n--; else n++; break;
 		case 2 : if (old_state == 0) n--; else n++; break;
 		case 3 : if (old_state == 2) n--; else n++; break;
 	}
 	
 	old_state = state;

 	if (n == 3) {
 		tic = tic + 1;
 		n = 0;
 	} 
}



ISR(USART_RX_vect) 
{

}

ISR(USART_TX_vect)
{
	
}

ISR(WDT_vect)
{
 	/* Watchdog interrupt! Update the PI-controller */
 	y = tic * 2 * 60/96;
		
 	int e = y_ref - y;
 	int v = K * (beta * y_ref - y) + Integrator;
 	u = saturate(v);
 	set_pwm_output(u);
 	
 	Integrator = Integrator + (K * sample_h / (double)T_i) * e + (sample_h/(double)T_r) * (u - v);
	tic = 0;
}



