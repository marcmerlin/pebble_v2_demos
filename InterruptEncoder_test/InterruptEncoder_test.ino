/*
A (relatively) simple sketch for using PCint for interrupt on encoder
rotation on the pebble v2.
Inspired by code from Brett Downing 2012, and mostly written by
Marc MERLIN <marc_soft@merlins.org> for pebble v2 and aiko.
I changed clockwise to be +1, not -1


Notes from Brett Downing:
----------------------------------------------------------------------------
One thing you'll notice about the pebble board is the enormous number of encoder
errrors it generates even though position measurement is rock solid. (it returns
to zero at the same angle) The reason for this is contact bounce.
This sketch, run on high quality optical encoder modules ($30 from sparkfun)
results in almost zero errors.

The contact bounce is a very fast signal, much faster than the encoder
can actually rotate by two quadrants.  With the interrupt running as
fast as it does, the gray code signal is almost entirely bounce immune.
----------------------------------------------------------------------------

Useful pages to read, the poll based method::
http://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino
and the better ISR based method that this code is based on:
http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros
(you'll want to read the comments of the first page to understand the encoder
lookup table).
*/

const int PIN_ROTARY_A = 6; // First bit number within port D
const int PIN_ROTARY_B = 7; // 2nd interrupt to see which way the rotation was.
const int PIN_ROTARY_PUSH = 8; // Rotary push down switch (Pin 8, Port B, bit 0)

#define PORTD_ENCODER_A_MASK (1<<PIN_ROTARY_A)
#define PORTD_ENCODER_B_MASK (1<<PIN_ROTARY_B)
#define PORTD_ENCODERS_MASK (PORTD_ENCODER_B_MASK | PORTD_ENCODER_A_MASK)

// D8 is bit 0 on port B.
#define PORTB_BUTTON_MASK (1<< (PIN_ROTARY_PUSH-8))

uint8_t PINB_input = 0x00;  // states read immediately at interrupt
uint8_t PINB_old_states = 0x00;  // the states at last interrupt
uint8_t PINB_bits_changed = 0x00;  // XOR of the old and new states
boolean PortBInterruptFired = false;

uint8_t PIND_input = 0x00;  // states read immediately at interrupt
uint8_t PIND_old_states = 0x00;  // the states at last interrupt
uint8_t PIND_bits_changed = 0x00;  // XOR of the old and new states
uint8_t PIND_AB_states = 0;  // lookup table index
uint8_t PIND_AB_bits = 0;    // shifted rotary bits read and stored in 1, 0
boolean PortDInterruptFired = false;

// This is reset to 0 in the ISR each time it is entered.
int8_t  rotary_button_value  = 0;   // 0 if error or -1/+1 if pending processing
// This value is only reset when it is read or replaced by another non null one.
int8_t  rotary_button_change = 0;   // 0 if read, or -1/+1 if pending processing
int16_t rotary_error = 0;  // when 2 pins change at once (undefined direction)
boolean button_status, old_button_status = 1;


/* For interrupts info, read
 * http://forums.trossenrobotics.com/tutorials/how-to-diy-128/an-introduction-to-interrupts-3248/
 *
 * Pin to interrupt map, see http://arduino.cc/playground/Main/PinChangeInt
 * D0-D7           = PCINT 16-23 = PCIR2 = Port D = PCIE2 = pcmsk2
 * D8-D13          = PCINT 0-5   = PCIR0 = Port B = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13  = PCIR1 = Port C = PCIE1 = pcmsk1
 */

#define BUTTON_PUSHED 0
#define BUTTON_NOTPUSHED 1

// Interrupt Service Routine for PORTB. D8 is PCINT0
ISR(PCINT0_vect) {
    PINB_input = PINB;  // This macro/function returns pin states for port D
    PINB_bits_changed = PINB_input ^ PINB_old_states; 

    // Make sure that a change received was actually on the pins we care about
    // and not other ones on that bank:
    if (PINB_bits_changed & PORTB_BUTTON_MASK) 
    {
	button_status = PINB_input & PORTB_BUTTON_MASK;
    }
    PINB_old_states = PINB_input;
    PortBInterruptFired = true;
}


// Interrupt Service Routine for PORTD. D6 and D7 are PCINT22 and PCINT23
// Expects encoder with four state changes between detents and both pins
// open on detent.
ISR(PCINT2_vect)
{
    // Make variables static to save time by not recreating them at each
    // interrupt call.
    static const int8_t enc_states [] PROGMEM = 
				    {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    PIND_AB_states <<= 2; // move previous 2 bits from position 1,0 to 3,2 
    // Merge in new bits 7, 6 to positions 1, 0
    PIND_AB_bits = ( (PIND & PORTD_ENCODERS_MASK) >> PIN_ROTARY_A ); 
    PIND_AB_states |= PIND_AB_bits; 
    // We now have a 4 bit value with the 2 old positions and 2 new positions.
    // This can used to look in the enc_states lookup table 

    // pgm_read_byte is used to read from the array in flash.
    // I've inverted the table because I prefer -1 for CCW and 1 for CW.
    rotary_button_value = -
		    pgm_read_byte( &(enc_states[( PIND_AB_states & 0x0f )]) );
    if (rotary_button_value) 
    {
	rotary_button_change = rotary_button_value;
    }   
    else
    {
	rotary_error++;
    }   
    PortDInterruptFired = true;
}

void setup() {
    // 38400 is a common speed for xbee modules, so we want USB-serial speed to
    // match in case we use Xbee serial reporting.
    Serial.begin(38400); 

    pinMode(PIN_ROTARY_A,    INPUT);
    pinMode(PIN_ROTARY_B,    INPUT);
    pinMode(PIN_ROTARY_PUSH, INPUT);

    // Pin change interrupt control register - enables interrupt vectors
    // Bit 2 = enable PC vector 2 (PCINT23..16)
    // Bit 1 = enable PC vector 1 (PCINT14..8)
    // Bit 0 = enable PC vector 0 (PCINT7..0)
    
    // The values below are correct for pebble v2
    // see "/usr/lib/avr/include/avr/iom328p.h" for aliases for other pins etc.
    PCICR  |= (1 << PCIE0);   // enable port-change int on PORTB
    PCICR  |= (1 << PCIE2);   // enable port-change int on PORTD

    // Pin change mask registers decide which pins are enabled as triggers
    PCMSK0 |= (1 << PCINT0);  // port change interrupt 0  for pin 8
    PCMSK2 |= (1 << PCINT22); // port change interrupt 22 for pin 6
    PCMSK2 |= (1 << PCINT23); // port change interrupt 23 for pin 7
}

void loop(){
    if (button_status != old_button_status)
    {
	if (button_status == BUTTON_NOTPUSHED && old_button_status == BUTTON_PUSHED)
	{
	    //button_clicked = true;
	    Serial.println(">>> BUTTON CLICKED");
	}
	old_button_status = button_status;
    }
    if (rotary_button_change)
    {
	Serial.print(">>> Rotary button turned: ");
	Serial.println(rotary_button_change);
	// reset value after 'consuming' it.
	rotary_button_change = 0;
    }   

    if (PortBInterruptFired  == true) {
	Serial.print("button status    : ");
	Serial.println(button_status);
	Serial.print("old button status: ");
	Serial.println(old_button_status);

	Serial.print("PINB_bits_changed        : ");
	Serial.println(PINB_bits_changed, BIN);
	Serial.print("PORTB_BUTTON_MASK        : ");
	Serial.println(PORTB_BUTTON_MASK, BIN);
	Serial.print("PINB_input (PINB)        : ");
	Serial.println(PINB_input, BIN);
	Serial.println();
	// Allow Interrupt handler to do something again.
	PortBInterruptFired = false;
    }
    if (PortDInterruptFired  == true) {
	Serial.print("rotary_button_value      : ");
	Serial.println(rotary_button_value);
	Serial.print("Encoder Errors           : ");
	Serial.println(rotary_error);

	// Note, this PIND value is likely not the one fromthe interrupt
	Serial.print("PIND                     : ");
	Serial.println(PIND, BIN);
	Serial.print("PORTD_ENCODERS_MASK      : ");
	Serial.println(PORTD_ENCODERS_MASK, BIN);
	Serial.print("PIND_AB_bits             : ");
	Serial.println(PIND_AB_bits, BIN);
	Serial.print("PIND_AB_states & 0x0f    : ");
	Serial.println(PIND_AB_states & 0x0f, BIN);
	Serial.println();
	// Allow Interrupt handler to do something again.
	PortDInterruptFired = false;
    }
}

// vim:sts=4:sw=4
