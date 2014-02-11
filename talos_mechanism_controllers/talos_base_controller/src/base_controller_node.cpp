#include <signal.h>

#include <boost/thread.hpp>

/* ROS libraries */
#include "ros/ros.h"
#include "ros/xmlrpc_manager.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include "BufferedAsyncSerial.h"
#include "parser.h"
#include "odom_manager.h"

namespace base_controller {
    BufferedAsyncSerial * async_serial;

    /* signal-safe flag for whether shutdown is requested */
    sig_atomic_t volatile g_request_shutdown = 0;
}

/* Replacement SIGINT handler */
void mySigIntHandler(int sig)
{
    base_controller::g_request_shutdown = 1;
    ROS_WARN( "Received SIGINT, shutting down" );
}

/* Replacement "shutdown" XMLRPC callback */
void shutdown_callback (XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if ( params.getType() == XmlRpc::XmlRpcValue::TypeArray )
        num_params = params.size();
    if ( num_params > 1 )
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: %s]", reason.c_str());
        base_controller::g_request_shutdown = 1; // Set flag
    }
    ROS_WARN("custom shutdown functionality goes here");

    result = ros::xmlrpc::responseInt(1, "", 0);
}

void cmdCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
    if ( ros::ok() ) {
        std::string linear_x = boost::lexical_cast<std::string>(msg->linear.x);
        std::string linear_y = boost::lexical_cast<std::string>(msg->linear.y);
        std::string angular_z = boost::lexical_cast<std::string>(msg->angular.z);

        std::string usb_msg = linear_x + ", " + linear_y + ", " + angular_z + "\n";

        ROS_INFO( "sending: %s", usb_msg.c_str() );

        try {
            base_controller::async_serial->writeString( usb_msg );
        } catch ( boost::system::system_error& e ) {
            ROS_ERROR( "Error: %s", e.what() );
        }
    }
}

int main( int argc, char **argv ) {
    ROS_INFO( "initializing base_controller" );
    //ros::init( argc, argv, "talos_base_controller" );
    ros::init( argc, argv, "base_controller", ros::init_options::NoSigintHandler );
    ros::NodeHandle n;

    /* Override SIGINT handler */
    signal( SIGINT, mySigIntHandler );

    /* Override XMLRPC shutdown */
    ros::XMLRPCManager::instance()->unbind( "shutdown" );
    ros::XMLRPCManager::instance()->bind( "shutdown", shutdown_callback );

    /* create publisher and subscribers */
    ROS_INFO( "subscribing to cmd_vel" );
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmdCallBack );
    ROS_INFO( "publishing to odom" );
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    ROS_INFO( "publishing tfs" );
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(20);

    try {

        ROS_INFO( "initializing asynchronous serial communication" );
        std::string device_file;
        n.param<std::string>("device_file", device_file, "/dev/ttyACM0");
        ROS_INFO( "device_file: %s", device_file.c_str() );

        base_controller::async_serial = new BufferedAsyncSerial( device_file , 9600 );
        ROS_INFO( "reading serial comm buffer" );

        //while ( ros::ok() ) {
        while ( !(base_controller::g_request_shutdown) ) {
            std::string readValue;
            readValue = base_controller::async_serial->readStringUntil("\n");
            
            if (readValue != "") {
                ROS_INFO( "received message: %s", readValue.c_str() );
            //    std_msgs::String msg;        
            //    msg.data = readValue;
            //    odom_pub.publish(msg);
            }

            ros::spinOnce();
            usleep( 100000 ); 
        }

    } catch ( boost::system::system_error& e ) {
        ROS_ERROR ( "Error: %s", e.what() );
        ROS_ERROR ( "closing serial connection" );
        base_controller::async_serial->close();
        delete base_controller::async_serial;
        ros::shutdown();
    }    

    ROS_INFO ( "closing serial connection" );
    base_controller::async_serial->close();
    delete base_controller::async_serial;
    ros::shutdown();
}
