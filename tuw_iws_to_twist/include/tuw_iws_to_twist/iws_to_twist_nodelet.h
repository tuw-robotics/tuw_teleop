#ifndef TUW_IWS_TO_TWIST_NODELET_H
#define TUW_IWS_TO_TWIST_NODELET_H

#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <string>
#include <memory>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
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
  class IwsToTwistNodelet : public nodelet::Nodelet
  {
  public:
    IwsToTwistNodelet();
    virtual ~IwsToTwistNodelet();
    virtual void onInit();

  private:
    ros::Subscriber sub_joint_iws_;
    ros::Publisher pub_joint_iws_;
    void iws_cb(const tuw_nav_msgs::JointsIWSConstPtr &msg);
    geometry_msgs::TwistStampedPtr twist_stamped_;

  };
}


#endif 
