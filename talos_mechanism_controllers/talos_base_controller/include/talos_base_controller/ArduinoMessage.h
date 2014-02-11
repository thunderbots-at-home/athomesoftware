#ifndef ARDUINO_MESSAGE_H
#define ARDUINO_MESSAGE_H

struct ArduinoMessage 
{

	bool e_stop;

  float time_stamp;
	float l_motor_rpm;
	float l_pid;
	float l_feedback;

	float r_motor_rpm;
	float r_pid;
	float r_feedback;

};


#endif
