#include "odometry_manager.h"

OdometryManager::OdometryManager() :
  x_(0), y_(0), th_(0), last_time_stamp_(0),
  current_time_(0.0) {}

OdometryManager::~OdometryManager() {}

void OdometryManager::UpdateOdometry(std::string input)
{
  /* parse input string*/

}

geometry_msgs::TransformStamped OdometryManager::GetCurrentTransform()
{
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th_);
}

nav_msgs::Odometry OdometryManager::GetCurrentOdom()
{
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  /* position */
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = 
    tf::createQuaternionMsgFromRollPitchYaw(0, 0, th_);

  /* velocity */
  /* TODO */
  return odom;
}

