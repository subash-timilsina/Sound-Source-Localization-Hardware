/*
 * Base_Omni.cpp
 *
 * Created: 9/17/2017 11:52:27 AM
 * Author : Sherlock
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Wheel.h"
#include "headers.h"

int main(void)
{    
	sei();
	
	Wheel w;
	w.init();
	
	
	while(1)
	{
		
		w.get_joystick_data();
		w.calculate_wheel_velocity();
		w.update_wheel_velocity();
	}

	return 0;
}
