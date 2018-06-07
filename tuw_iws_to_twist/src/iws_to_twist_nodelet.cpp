/**
 * @file joy2cmd_.cpp
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date June 2015
 * @brief Generates twist messages based on joy messages
 * The code is based on the teleop_base.cpp, 2008, Willow Garage and
 * 2010  David Feil-Seifer [dfseifer@usc.edu], Edward T. Kaszubski [kaszubsk@usc.edu]
 */

#include <boost/algorithm/string.hpp>
#include <tuw_iws_to_twist/iws_to_twist_nodelet.h>


using namespace tuw;

IwsToTwistNodelet::IwsToTwistNodelet()
{
}

IwsToTwistNodelet::~IwsToTwistNodelet()
{
}


void IwsToTwistNodelet::onInit()
{
//  sub_joint_iws_ = getPrivateNodeHandle().subscribe("joint_cmds", 1, (boost::function <void(const tuw_nav_msgs::JointsIWSConstPtr&)>)
//                                                    boost::bind(&IwsToTwistNodelet::iws_cb, this, _1 ));
  sub_joint_iws_ = getPrivateNodeHandle().subscribe("joint_cmds", 1, &IwsToTwistNodelet::iws_cb, this);
  pub_joint_iws_ = getPrivateNodeHandle().advertise<geometry_msgs::TwistStamped>("joint_cmds_twist",1000);
  std::cout << "node initialized" << std::endl;
}

void IwsToTwistNodelet::iws_cb(const tuw_nav_msgs::JointsIWSConstPtr &msg)
{
  if (msg->revolute.size() != 2)
  {
    ROS_ERROR("IwsToTwistNodelet::iws_cb: message does not contain two revolute commands. Will not set the command.");
    return;
  }
  std::string revolute_mode = msg->type_revolute;
  if (revolute_mode.compare("joint_cmds"))
  {
    ROS_ERROR("IwsToTwistNodelet::iws_cb: revolute command type not supported. Will not set the command.");
    return;
  }

  double dummy_wheel_distance = 1.5;
  const double vL = msg->revolute[1];
  const double vR = msg->revolute[0];

  double omega = (vR - vL) / dummy_wheel_distance;
  double v = (vR - vL) / 2.0;

  twist_stamped_->twist.linear.x = v;
  twist_stamped_->twist.linear.y = 0.0;
  twist_stamped_->twist.linear.z = 0.0;

  twist_stamped_->twist.angular.x = 0;
  twist_stamped_->twist.angular.y = 0;
  twist_stamped_->twist.angular.z = omega;
}

PLUGINLIB_EXPORT_CLASS(tuw::IwsToTwistNodelet, nodelet::Nodelet)
