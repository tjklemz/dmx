/* Name: main.c
 * Author: Thomas Klemz
 * License: GPL v3
 * 2013-05-24
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


/*******************************************************************
 * DMX Section
 *******************************************************************/

#define DMX_NUM_CHANNELS 4

enum
{
    DMX_IDLE,
    DMX_BREAK,
    DMX_START,
    DMX_RUN
};

static volatile unsigned char dmx_state;

static unsigned int dmx_start_addr;
static unsigned int dmx_cur_addr;

static unsigned char chan_cnt;
static unsigned char dmx_data[DMX_NUM_CHANNELS] = {0};

static void setupDMX()
{
    dmx_start_addr = 1;
    chan_cnt = 0;
    dmx_state = DMX_IDLE;
    
    //init UART for DMX
    //250 kbps, 8 bits, no parity, 2 stop bits

    UBRRL = 1;
    UBRRH = (1>>8); 
    UCSRB = (1<<RXEN);
    UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);

    return;
}

static void update()
{
    unsigned char chan;

    for(chan = 0; chan < 4; chan++) {
        unsigned char chan_val = dmx_data[chan];

        if(chan_val == 0) {
            PORTC &= ~(1<<chan);
        } else if(chan_val == 255) {
            PORTC |= (1<<chan);
        } else { 
            /*
            unsigned int i;
            for(i = 0; i < (255-chan_val); ++i)
              _delay_us(0.001);
            */
            PORTC ^= (1<<chan); // toggle LED
        }
    }
}

static void handleDMX()
{
    unsigned char status, data;

    while(!(UCSRA & (1<<RXC)));

    status = UCSRA;
    data = UDR;

    switch(dmx_state) {
    case DMX_IDLE:
    if(status & (1<<FE)) {
        dmx_cur_addr = 0;
        dmx_state = DMX_BREAK;
    }
    break;

    case DMX_BREAK:
    if(data == 0) {
        dmx_state = DMX_START;
    } else {
        //something went wrong, start over
        dmx_state = DMX_IDLE;
    }
    break;

    case DMX_START:
    dmx_cur_addr++;
    // if we are at our address, it's time to start collecting data
    // otherwise, just wait (we'll keep incrementing the cur addr)
    if(dmx_cur_addr == dmx_start_addr)
    {
        chan_cnt = 0;
        dmx_data[chan_cnt++] = data;
        dmx_state = DMX_RUN;
    }
    break;

    case DMX_RUN:
    dmx_data[chan_cnt++] = data;
    //keep collecting data for each of *our* channels (don't go past)
    if(chan_cnt >= DMX_NUM_CHANNELS) {
        //we got what we needed so just wait for DMX to loop
        dmx_state = DMX_IDLE;
    } 
    break;

    default:
    //shouldn't get here...
    dmx_state = DMX_IDLE;
    break;
    }
}


/*******************************************************************
 * PWM Section
 *******************************************************************/

#define NUMBER_OF_PWMS 4    //the number of generated PWM signals
#define PHASE_MAX 255
#define WAIT_TIME 20

volatile uint8_t pwms[NUMBER_OF_PWMS]; //storage unit for the pwms setting
volatile uint8_t t = 0;
volatile uint8_t out = 0;

static void setupPWM()
{
    uint8_t i;

    cli();

    TCCR0 |= (1<<CS00) | (1<<CS02);
    TIMSK |= (1<<TOIE0); //interrupts for the timer 0
    TCNT0 = 0;

    GICR = (1<<INT0);
    MCUCR = (1<<ISC01) | (1<<ISC00);

    for(i = 0; i < NUMBER_OF_PWMS; i++)
        pwms[i] = 0;

    sei();
}

ISR(TIMER0_OVF_vect)	//timer 0 interrupt for the pwm core.
{
	uint8_t i;

	static uint8_t desired_pwms[NUMBER_OF_PWMS];
	static uint16_t wait_time[NUMBER_OF_PWMS];

	t = (t+1) % PHASE_MAX;

	for(i = 0; i < NUMBER_OF_PWMS; i++)
	{
		if(pwms[i] > desired_pwms[i]) pwms[i]--;
		if(pwms[i] < desired_pwms[i]) pwms[i]++;
		if(pwms[i] == desired_pwms[i])
		{
			if(wait_time[i] == 0)
			{
        uint8_t j;
        for(j = 0; j < 4; ++j)
          PORTC ^= (1<<j);
				wait_time[i] = WAIT_TIME;
				desired_pwms[i] = dmx_data[i];
			}
			else wait_time[i]--;
		}
	}
}


/*******************************************************************
 * Main
 *******************************************************************/

int main()
{
    //_delay_ms(50);
    //output pins (4 of them) 
    DDRC = 0xF;

    setupDMX();
    setupPWM();

    PORTC |= 1;
        
    for(;;) {
        handleDMX();
    }
 
    return 0;   //never reached
}

