#ifndef WHEEL_H
#define WHEEL_H


#include "uart.h"
#include "Motor.h"
#include <math.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define MAX_RPM		   400.0							//In RPM	



class Wheel
{
     private:
     float velocity_robot[3];
	 int robot_rpm;
	 float velocity_motor[3];
     int ocr_motor[3];
     Motor m1,m2,m3;
	 int azimuth,elevation;  


    public:

        

        void init();
		void reset_robvel();
		void preprocess_data();
        void get_joystick_data();
        void calculate_wheel_velocity();
        void update_wheel_velocity();

};

#endif // WHEEL_H
