#include "odometry_manager.h"

OdometryManager::OdometryManager() :
x_global_(0.0), x_current_(0.0), x_vel_(0.0),
y_global_(0.0), y_current_(0.0), y_vel_(0.0),
th_global_(0.0), th_current_(0.0), th_vel_(0.0),
wheel_width_(0.455), meters_per_tick_((6 * 2 * M_PI * 0.0525)/4680), previous_timestamp_(0.0),
current_timestamp_(0.0)
{}

OdometryManager::~OdometryManager() {}

void OdometryManager::UpdateOdometry(ArduinoMessage message)
{
  /* update odometry state */
  double left_delta = this->meters_per_tick_ * message.l_feedback;
  double right_delta = this->meters_per_tick_ * message.r_feedback;
  double distance = ( left_delta + right_delta ) / 2;
  double theta_delta = ( left_delta  - right_delta ) / this->wheel_width_;
  double x_delta = distance * cosf(theta_delta + this->th_global_);
  double y_delta = distance * sinf(theta_delta + this->th_global_);

  this->x_global_  += x_delta;
  this->y_global_  += y_delta;
  this->th_global_ += theta_delta;

  this->x_vel_ = x_delta / ToSeconds(message.time_stamp - this->previous_timestamp_);
  this->y_vel_ = y_delta / ToSeconds(message.time_stamp - this->previous_timestamp_);
  this->th_vel_ = theta_delta / ToSeconds(message.time_stamp - this->previous_timestamp_);

  this->previous_timestamp_ = message.time_stamp;

}

/* presuppose: duration is in milliseconds */
double OdometryManager::ToSeconds(unsigned long duration) {
  return duration * (1/1000.0);
}

geometry_msgs::TransformStamped OdometryManager::GetCurrentTransform()
{
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_timestamp();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  odom_trans.transform.translation.x = this->x_global_;
  odom_trans.transform.translation.y = this->y_global_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(this->th_global_);
}

nav_msgs::Odometry OdometryManager::GetCurrentOdom()
{
  nav_msgs::Odometry odom;
  odom.header.stamp = current_timestamp();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  /* position */
  odom.pose.pose.position.x = this->x_global_;
  odom.pose.pose.position.y = this->y_global_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = 
    tf::createQuaternionMsgFromRollPitchYaw(0, 0, this->th_global_);

  /* velocity */
  /* TODO */
  return odom;
}

