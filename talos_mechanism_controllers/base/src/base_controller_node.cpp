#include <unistd.h>

/* ROS libraries */
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"

/* Serial library */
#include "TimeoutSerial.h"

namespace base_controller {
	TimeoutSerial * serial_pointer;
}

using boost::lexical_cast;
using std::string;

void cmdCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
	if ( ros::ok() ) {

		string linear_x = lexical_cast<string>(msg->linear.x);
		string linear_y = lexical_cast<string>(msg->linear.y);
		string angular_z = lexical_cast<string>(msg->angular.z);

		string usb_msg = linear_x + ", " + linear_y + ", " + angular_z + "\n";

		ROS_INFO( "sending: %s", usb_msg.c_str() );

		try {
			TimeoutSerial serial( "/dev/arduino-mega256", 9600 );
			serial.setTimeout( boost::posix_time::seconds(5) ); // TODO needs to be better way to do this
			serial.writeString(usb_msg);
			serial.close();
		} catch ( boost::system::system_error& e ) {
			ROS_ERROR( "Error: %s", e.what() );
		}
	}
}

int main( int argc, char **argv )
{
	ROS_INFO( "initializing program" );
	ros::init( argc, argv, "base_controller" );
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdCallBack );

	try {
		ROS_INFO( "sending test signal" );
		TimeoutSerial serial( "/dev/arduino-mega256", 9600 );
		serial.setTimeout( boost::posix_time::seconds(5) );
		serial.writeString("initializing\n");
		serial.close();
	} catch ( boost::system::system_error& e ) {
		std::cout << "Error: " << e.what() << std::endl;
		return 1;
	}

	ros::spin();
	return 0;
}
