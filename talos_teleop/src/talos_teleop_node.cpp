#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class TeleopTalos
{
  public:
  TeleopTalos();

  private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

TeleopTalos::TeleopTalos():
  linear_(1),
  angular_(2)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("scale_angular", a_scale_, a_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTalos::joyCallback, this);
}

void TeleopTalos::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // 0 = 2: left stick f/b
    // 1: left stick l/r
    // 3: right stick f/b
  geometry_msgs::Twist twist;
  twist.linear.x = l_scale_*joy->axes[linear_];
  twist.angular.z = a_scale_*joy->axes[angular_];
  vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_talos");
  TeleopTalos teleop_talos;

  ros::spin();
}
