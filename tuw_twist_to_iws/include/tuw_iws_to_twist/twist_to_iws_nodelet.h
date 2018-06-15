#ifndef TUW_TWIST_TO_IWS_NODELET_H
#define TUW_TWIST_TO_IWS_NODELET_H

#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <string>
#include <memory>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tuw_nav_msgs/JointsIWS.h>

#include <dynamic_reconfigure/server.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace tuw {
  /**
   * @brief Class to generate twist messages base on TeleopBase written by Willow Garage
   * @author Markus Bader <markus.bader@tuwien.ac.at>
   * The default parameters are set for a Logitech Gamepad F710 with "MODE" off and the "XID/HID Switch" should be set on X
   **/
  class TwistToIwsNodelet : public nodelet::Nodelet
  {
  public:
    TwistToIwsNodelet();
    virtual ~TwistToIwsNodelet();
    virtual void onInit();

  private:
    ros::Subscriber sub_odometry_;
    ros::Publisher pub_joint_iws_;
    void odom_cb(const nav_msgs::OdometryPtr &msg);
    tuw_nav_msgs::JointsIWSPtr joint_iws_;

  };
}


#endif 
