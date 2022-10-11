/*
 * Aufgabe4ss22.cpp
 *
 * Created: 3/27/2022 2:07:17 PM
 * Author : Ameen
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Timer.h"
#include "LCD.h"
#include "DigiPort.h"

// Defining the beginnings and the ends of operations

#define START				2
#define NORMAL				1
#define END					3


// Initializations
LCD			mein_display	(PC, LCD_Type_24x2);			//  for the LCD
DigiPortRaw	leds			(PA, SET_OUT_PORT);				// LED - PORT Initialization

static inline void next_state(void);
Timer16	ticker	(TC1, next_state);
DigiPortRaw keys	(PK, SET_IN_PORT);

class Ampel {
	private:
	struct state {
		uint8_t led;
		uint8_t duration;
		uint8_t next;
	};
	
	typedef state state_t;
	static const state_t state_table [28];
	
	uint8_t loop_count;
	uint8_t ms_count;
	uint8_t upcoming_state;										// shows the upcoming state
	volatile uint8_t execute;									// shows what process is being executed right now
	public:
	Ampel();
	
	void start_up_request();
	void shut_down_request();
	void next_state();
	void display_settings();
	
};

Ampel dieAmpel;

int main(void)
{
	

	dieAmpel.display_settings();
	
	sei();										// activating the interrupts
	ticker.start_ms(1000);						// The timer is called every second

	dieAmpel.start_up_request();				// So the function knows start up needs to be executed when execute = START is

	while (1)
	{
		if(keys.read_raw() == 0b00000001) {
			dieAmpel.shut_down_request();				// So the function knows what button needs to be pressed in order to stop
		}
		
		
	}
}
static inline void next_state() {dieAmpel.next_state();}

Ampel :: Ampel () {

}

const Ampel :: state_t Ampel :: state_table [28] =
{
	// Normal operations
	{0b00100001, 5, 1},		// #0 rot - grün
	{0b00100010, 1, 2},		// #1 rot / gelb
	{0b00110010, 1, 3},		// #2 rot-gelb / grün
	{0b00110100, 1, 4},		// #3 rot-gelb / rot
	{0b00001100, 5, 5},		// #4 grün/rot
	{0b00010100, 1, 6},		// #5 gelb/rot
	{0b00010110, 1, 7},		// #6 gelb/rot-gelb
	{0b00100110, 1, 0},		// #7 rot/ rot-gelb
	// Start_Up
	{0b01010010, 1, 9},		// #8 blink yellow / blink yellow   
	{0b01000000, 1, 10},	// #9 blink yellow / blink yellow
	{0b01010010, 1, 11},	// #10 blink yellow / blink yellow
	{0b01000000, 1, 12},	// #11 blink yellow / blink yellow
	{0b01010010, 1, 13},	// #12 blink yellow / blink yellow
	{0b01000000, 1, 14},	// #13 blink yellow / blink yellow
	{0b01010010, 1, 15},	// #14 blink yellow / blink yellow
	{0b01000000, 1, 16},	// #15 blink yellow / blink yellow
	{0b01010010, 1, 17},	// #16 blink yellow / blink yellow
	{0b01100100, 5, 18},	// #17 rot/rot
	// Shut down
	{0b10100100, 5, 19},	// #18 rot/rot
	{0b10010010, 1, 20},	// #19 blink yellow / blink yellow
	{0b10000000, 1, 21},	// #20 blink yellow / blink yellow
	{0b10010010, 1, 22},	// #21 blink yellow / blink yellow
	{0b10000000, 1, 23},	// #22 blink yellow / blink yellow
	{0b10010010, 1, 24},	// #23 blink yellow / blink yellow
	{0b10000000, 1, 25},	// #24 blink yellow / blink yellow
	{0b10010010, 1, 26},	// #25 blink yellow / blink yellow
	{0b10000000, 1, 27},	// #26 blink yellow / blink yellow
	{0b10010010, 1, 28},	// #27 blink yellow / blink yellow
};



void Ampel :: start_up_request() {
	leds.off();
	execute = START;
	upcoming_state = 9;
	ms_count = 0;
	mein_display.set_pos(0,7);
	mein_display.write_number(START, 1, '0');
	loop_count = 0;
}

void Ampel :: shut_down_request() {
	if(upcoming_state <= 18) {
		mein_display.set_pos(0,7);
		mein_display.write_number(END, 1, '0');
		ms_count = 0;
		// upcoming_state = 19;
		execute = END;
	}
}

void Ampel :: next_state() {

	if(upcoming_state <= 28) {
		leds.write(state_table[upcoming_state].led);
	}
	
	if(state_table[upcoming_state].duration != ms_count) {
		ms_count++;
	}
	
	if(state_table[upcoming_state].duration == ms_count) {
		upcoming_state = state_table[upcoming_state].next;
		ms_count = 0;
	}
	
	if(upcoming_state == 18) {

		if(loop_count == 0) {
			upcoming_state = 2;
			upcoming_state = state_table[upcoming_state].next;
		}
		if(loop_count >= 1) {
			upcoming_state = 1;
			upcoming_state = state_table[upcoming_state].next;
		}
		
		loop_count++;
	}
	
	if(upcoming_state == 6 && execute == END) {
		upcoming_state = 19;
		upcoming_state = state_table[upcoming_state].next;
	}
	
	if(upcoming_state >= 28) {
		leds.off();
		loop_count = 0;
		ticker.stop();
	}
}

void Ampel :: display_settings () {
	mein_display.set_pos(0,0);
	mein_display.write_SRAM_text("State: ");
	
	mein_display.set_pos(1,0);
	mein_display.write_SRAM_text("--> ");
}

