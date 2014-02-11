

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <string>
#include <talos_base_controller/ArduinoMessage.h>


class Parser
{

	

	public:

	Parser()
	{

	}

	// Given a string composed of comma seperated values from the arduino, parses it into a struct.
	ArduinoMessage parse(std::string& message)
	{

		struct ArduinoMessage msg;
		int last_index = 0;

		for (int i = 0; i < 7; i++)
		{
	
			int comma_index = message.find(',', last_index);
			std::string argument;
		
			argument = message.substr(last_index, comma_index);
			float fixed_arg = atof(argument.c_str());
	
			switch(i)
			{
				case 0:
					if (fixed_arg > 0)
					{
						msg.e_stop = 1;
					} else
					{
						msg.e_stop = 0;
					}

					break;
				case 1:
					msg.l_motor_rpm = fixed_arg;
					break;
				case 2:
					msg.r_motor_rpm = fixed_arg;
					break;
				case 3:
					msg.l_pid = fixed_arg;
					break;
				case 4:
					msg.r_pid = fixed_arg;
					break;
				case 5:
					msg.l_feedback = fixed_arg;
					break;
				case 6:
					msg.r_feedback = fixed_arg;
					break;
			}

			last_index = comma_index;
		}

		return msg;
	}

};
