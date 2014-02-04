#ifndef ODOMETRY_MANAGER_H

#include <string>

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

#define ODOMETRY_MANAGER_H
class OdometryManager
{
  public :
    OdometryManager();
    virtual void UpdateOdometry(std::string received_string);
    virtual geometry_msgs::TransformStamped GetCurrentTransform();
    virtual nav_msgs::Odometry GetCurrentOdom();
    virtual ~OdometryManager();

    /* getters and setters */
    double x() const { return this->x_; }
    double y() const { return this->y_; }
    double th() const { return this->th_; }
    double last_time_stamp() const { return this->last_time_stamp_; }
    ros::Time current_time() const { return this->current_time_; }

    void x(const double& x) { this->x_ = x; }
    void y(const double& y) { this->y_ = y; }
    void th(const double& th) { this->th_ = th; }
    void last_time_stamp(const double& last_time_stamp) { this->last_time_stamp_ = last_time_stamp; }

  private :
    double x_;
    double y_;
    double th_;
    unsigned long last_time_stamp_;
    ros::Time current_time_;

};

#endif
