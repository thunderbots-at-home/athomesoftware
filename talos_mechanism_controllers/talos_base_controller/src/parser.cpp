#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <talos_base_controller/ArduinoMessage.h>
#include <iostream>

class Parser
{
  public:

    Parser()
    {

    }

    // Given a string composed of comma seperated values from the arduino, parses it into a struct.
    ArduinoMessage parse(const std::string& message)
    {

      struct ArduinoMessage msg;
      std::stringstream line(message);
      std::string cell;
      int count = 0;
      while (std::getline(line, cell, ','))
      {
        switch(count)
        {
          case 0:
            msg.time_stamp = strtoul(cell.c_str(), NULL, 0);
            break;
          case 1:
            msg.e_stop = atoi(cell.c_str());
            break;
          case 2:
            msg.l_motor_rpm = atof(cell.c_str());
            break;
          case 3:
            msg.r_motor_rpm = atof(cell.c_str());
            break;
          case 4:
            msg.l_pid = atof(cell.c_str());
            break;
          case 5:
            msg.r_pid = atof(cell.c_str());
            break;
          case 6:
            msg.l_feedback = atof(cell.c_str());
            break;
          case 7:
            msg.r_feedback = atof(cell.c_str());
            break;
        }
        count++;
      }
      return msg;
    }
};
