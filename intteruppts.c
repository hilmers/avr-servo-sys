#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint8_t portbhistory = 0xFF; 
volatile uint8_t grey_code = 0b000000001;
volatile int emit_light = 1;

int main(void)
{
    DDRB &= ~((1 << DDB1) | (1 << DDB2));
    PORTB |= ((1 << PB1) | (1 << PB2));
    
    PCMSK0 |= (1 << PCINT0);    // set PCINT0 to trigger an interrupt on state change
    PCICR |= (1 << PCIE0);        // set PCIE0 to enable PCMSK0 scan
    
    sei();                      // Enable interrupts.
    
    DDRC = 0x0F;                // Enable some PBC/ADC
    
    while(1){
    
    }
}
    
    ISR(PCINT0_vect){
        cli();              // Disable interrupt (safety reason).
        uint8_t changedbits;

        changedbits = PINB ^ portbhistory;
        portbhistory = PINB;


        if(changedbits & (1 << PB1))
        {
            
            emit_light = emit_light*2;  
        }

        if(changedbits & (1 << PB2))
        {
            emit_light = emit_light/2;
            
        }

        PORTC = emit_light;
        sei();              // Enables interrupts again. 
}