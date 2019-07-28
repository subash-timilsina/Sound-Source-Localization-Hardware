#include "Wheel.h"


float coupling_matrix[3][3] = {{-0.6667,0,0.3333},{0.3333,-0.5774,0.3333},{0.3333,0.5774,0.3333}}; 

void Wheel::init()
{
	sei();
	initUART0();
	initUART3();
	robot_rpm = 100;
	for(int i=0;i<3;i++)
	{
		velocity_motor[i] = 0;
		velocity_robot[i] = 0;
	}
	m1.Initialise(1);
	m2.Initialise(2);
	m3.Initialise(3);
}

void Wheel::preprocess_data()
{
	/******************************** Azimuth ********************************/
	if (rcvdata[0] >= rcvdata[1])
	{
		azimuth = rcvdata[0] + rcvdata[1];
	}
	else
	{
		azimuth = -(rcvdata[0] + rcvdata[1]);
	}
	
	
	/******************************** Elevation *****************************************/
	if (rcvdata[2] >= rcvdata[3])
	{
		elevation = rcvdata[2] + rcvdata[3];
	}
	else
	{
		elevation = -(rcvdata[2] + rcvdata[3]);
		}	
	
}

void Wheel::reset_robvel()
{
	for(int i=0;i<3;i++)
		velocity_robot[i] = 0;
}

void Wheel::get_joystick_data()
{
	/*char data;
	data = UART0Receive();
	
	switch(data)
	{
		case 'F':
			velocity_robot[1] = robot_rpm;
			break;
		case 'B':
			velocity_robot[1] = -robot_rpm;
			break;
		case 'L':
			velocity_robot[0] = -robot_rpm;
			break;
		case 'R':
			velocity_robot[0] = robot_rpm;
			break;
		case 'G':
			velocity_robot[2] = robot_rpm;
			break;
		case 'I':
			velocity_robot[2] = -robot_rpm;
			break;
		case 'S':
			reset_robvel();
			break;
	}*/
	
	
	if(!rcvflag)
		preprocess_data();
		
	
	if (rcvdata[4] == 1)
	{
		velocity_robot[0] = -robot_rpm * sin(azimuth*(PI/180.0));
		velocity_robot[1] = robot_rpm * cos(azimuth*(PI/180.0));
	}
	else if (rcvdata[4] == 90)
	{
		velocity_robot[0] = 0;
		velocity_robot[1] = 0;
	}
	
	//UART0TransmitData(azimuth);
	//UART0Transmit('\t');
	//UART0TransmitData(elevation);
	//UART0Transmit('\t');
	//UART0TransmitData(velocity_robot[0]);
	//UART0Transmit('\t');
	//UART0TransmitData(velocity_robot[1]);
	//UART0TransmitString("\n\r");
	
}

void Wheel::calculate_wheel_velocity()
{
	
    for(int i=0;i<3;i++)
    {
        velocity_motor[i] = 0;
        for(int j=0;j<3;j++)
        {
             velocity_motor[i] += velocity_robot[j] * coupling_matrix[i][j];
        }
    }
    for(int i=0;i<3;i++)
    {
        ocr_motor[i] = (249.0*velocity_motor[i]/(MAX_RPM));//multiply by icr_top value 
    }
}

void Wheel::update_wheel_velocity()
{
    m1.SetOcrValue(ocr_motor[0]);
    m2.SetOcrValue(ocr_motor[1]);
    m3.SetOcrValue(ocr_motor[2]);
}
