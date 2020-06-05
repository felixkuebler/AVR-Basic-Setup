
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define LED_GREEN_BIT PB0
#define LED_YELLOW_BIT PB1
#define LED_RED_BIT PB2

#define BTN_INPUT_BIT PB3
#define POT_INPUT_BIT PB4

#define setBit(reg, bit) reg |= (1<<bit)
#define clearBit(reg, bit) reg &= ~(1<<bit)
#define toggleBit(reg, bit) reg ^= (1<<bit)
#define readBit(reg, bit) (reg & (1 << bit)) >> bit

#define RED_TIME_THRESH 1000
#define YELLOW_TIME_THRESH 1000
#define GREEN_TIME_THRESH 2000

#define DELAY_TIME  50

volatile uint64_t timer = 0;
volatile uint8_t timer_running = 0;


void mcuSleep()
{
    // backup adc status register
    uint8_t adcsra = ADCSRA;

    // disable adc
    clearBit(ADCSRA, ADEN);

    // configure power down mode
    setBit(MCUCR, SM1);
    clearBit(MCUCR, SM0);

    // enable sleep
    setBit(MCUCR, SE);

    //sleep
    sleep_cpu();

    // disable sleep
    clearBit(MCUCR, SE);

    // recover adc status register
    ADCSRA = adcsra; //ADCSRA-Register rückspeichern
}

void ledInit()
{
    // data direction for leds as output
    setBit(DDRB, LED_GREEN_BIT);
    setBit(DDRB, LED_YELLOW_BIT);
    setBit(DDRB, LED_RED_BIT);
}

void adcInit()
{
    // mux for adc
    clearBit(ADMUX, MUX0);
    setBit(ADMUX, MUX1);

    // adc frequency
    setBit(ADCSRA, ADPS0);
    setBit(ADCSRA, ADPS1);

    // enable adc
    setBit(ADCSRA, ADEN);
}

void interruptInit()
{   
    // disable global interrupts
    clearBit(SREG, 7);

    // set interrupt trigger set to io change
    setBit(GIMSK, PCIE);

    // interrupt mask to pin PB3
    setBit(PCMSK, PCINT3);

    // enable global interrupts
    setBit(SREG, 7);
}

void main(void)
{
    ledInit();

    adcInit();

    interruptInit();


    while(1)
    {
        // read analog value from adc
        setBit(ADCSRA, ADSC);

        // wait till read is finished
        while (readBit(ADCSRA, ADSC)) {}
        uint16_t sensor_val = ADCW;

        // map time scale to sensor value
        uint8_t time_scale = 1;

        if (sensor_val < 1024/3) time_scale = 1;
        else if (sensor_val < 2*1024/3) time_scale = 2;
        else time_scale = 3;

        if (timer_running == 1)
        {
            if(timer < RED_TIME_THRESH * time_scale)
            {
                // turn on red led
                clearBit(PORTB, LED_GREEN_BIT);
                clearBit(PORTB, LED_YELLOW_BIT);
                setBit(PORTB, LED_RED_BIT);  
            }
            else if (timer < (YELLOW_TIME_THRESH + RED_TIME_THRESH) * time_scale)
            {
                // turn on yellow led
                clearBit(PORTB, LED_GREEN_BIT);
                clearBit(PORTB, LED_RED_BIT);
                setBit(PORTB, LED_YELLOW_BIT);    
            }
            else if (timer < (GREEN_TIME_THRESH + YELLOW_TIME_THRESH + RED_TIME_THRESH) * time_scale)
            {
                // turn on green led
                clearBit(PORTB, LED_YELLOW_BIT);
                clearBit(PORTB, LED_RED_BIT);
                setBit(PORTB, LED_GREEN_BIT);    
            }
            else
            {
                // turn all leds off
                clearBit(PORTB, LED_GREEN_BIT);
                clearBit(PORTB, LED_YELLOW_BIT);
                clearBit(PORTB, LED_RED_BIT);

                // reset timer
                timer = 0;
                timer_running = 0;
            }

            timer += DELAY_TIME;
        }
        else
        {
            //trigger sleep to power down
            mcuSleep();
        }

        _delay_ms(DELAY_TIME);  
    }    
}

// interrupt service routine
ISR(PCINT0_vect)
{
    // starting timer
    timer = 0;
    timer_running = 1;
}