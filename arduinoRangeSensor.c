/* arduinoRangeSensor.c This code operates the HC-SR04 ultrasound sensor 
and displays measured range on a 7-Segment LED

Operation: code provides a 10us trigger pulse on PB1 then 
waits for an echo pulse on PB0. Duration of echo pulse is 
the round-trip time delay to the target. With Fcpu=16 MHz 
and 1024 divider, Fclock = 16 MHz/1024 = 15.625 KHz. Each 
clock pulse is therefore 1/15,625 = 64 us. 
Speed of sound is 343 m/s or 34300 cm/s so each clock pulse 
is 34300 cm/s x 64e-6 s = 2.195 cm round trip or 1.098 cm one way
8 bit counter with 256 states can measure distances 0 to 255*1.098
= 2.80 m or 9.2 feet with 1.1 cm resolution. 
Timer0 will be set free-running, 0-255 then repeating 
Note: objects >2.80m will have round-trip delay longer
than the clock count-up time; such echos are not printed to UART.
W. Stone 5/14/24 Initial Code for lab8.3 ECE--231 Spring 2024 */
#define TRIG PB1    //PB1 = pin 15
#define ECHO PB0    //PB0 = pin 14
#define RANGE_PER_CLOCK 1.098
#define PERSISTENCE 5
#include <avr/io.h>
#include <util/delay.h>

void timer0_init(void);

int main(void){
    unsigned char ledDigits[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67};
    unsigned int i=0;
    unsigned char DIG1, DIG2, DIG3, DIG4;

    DDRD=0xFF;
    DDRC=0xFF;
    PORTC=0xFF;

    unsigned char rising_edge_clocks, falling_edge_clocks, 
                                        echo_width_clocks;
    unsigned int target_range;
    DDRB = 1<<TRIG;         // TRIG is output pin;
    PORTB &= ~(1<<TRIG);    // Set the TRIG pin low
    timer0_init();          // Initialize timer0

    while(1){
        TCNT0 = 0;          // Load counter with 0
        PORTB |= 1<<TRIG;   // These three lines of code
        _delay_us(10);      // Put a 10 usec pulse on the
        PORTB &= ~(1<<TRIG);// TRIG pin.

        // Wait till the ECHO pulse goes high
        while ((PINB & (1<<ECHO)) ==0);
        rising_edge_clocks = TCNT0; // Note the time
        // Now wait till the ECHO pulse goes low
        while (!(PINB & (1<<ECHO))==0);
        falling_edge_clocks = TCNT0;
        
        if (falling_edge_clocks > rising_edge_clocks){
            // Compute target range and send it to the serial monitor
            echo_width_clocks = falling_edge_clocks - rising_edge_clocks;
            target_range = echo_width_clocks * RANGE_PER_CLOCK;
        }
        //7segment display code each section presents one digit to the LED
        DIG4=(target_range)%10;
        PORTD=ledDigits[DIG4];
        PORTC= ~(1<<0);
        _delay_ms(PERSISTENCE);

        DIG3=(target_range/10)%10;
        PORTD=ledDigits[DIG3];
        PORTC= ~(1<<1);
        _delay_ms(PERSISTENCE);

        DIG2=(target_range/100)%10;
        PORTD=ledDigits[DIG2];
        PORTC= ~(1<<2);
        _delay_ms(PERSISTENCE);

        DIG1=(target_range/1000)%10;
        PORTD=ledDigits[DIG1];
        PORTC= ~(1<<3);
        _delay_ms(PERSISTENCE);
    }
}

// Initialize timer0: normal mode (count up), divide clock by 1024
void timer0_init(){
    TCCR0A = 0;         // Timer 1 Normal mode (count up)
    TCCR0B = 5;         // Divide clock by 1024
    TCNT0=0;            // Start the timer at 0

}