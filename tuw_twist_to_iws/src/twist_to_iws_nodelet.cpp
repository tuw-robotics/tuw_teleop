/**
 * @file iws_to_twist_nodelet.cpp
 * @author Felix König <felix.koenig@tuwien.ac.at>
 * @date June 2018
 * @brief Generates twist messages based on iws messages
 *
 */

#include <boost/algorithm/string.hpp>
#include <tuw_iws_to_twist/twist_to_iws_nodelet.h>

using namespace tuw;

TwistToIwsNodelet::TwistToIwsNodelet()
{
}

TwistToIwsNodelet::~TwistToIwsNodelet()
{
}

void TwistToIwsNodelet::dynamic_reconfigureCB(tuw_twist_to_iws::twist_to_iws_nodeConfig &config, uint32_t level)
{
  wheeldiameter = config.wheeldiameter;
  wheeldistance = config.wheeldistance;
}

void TwistToIwsNodelet::onInit()
{
  joint_iws_ = boost::make_shared<tuw_nav_msgs::JointsIWS>();
  sub_odometry_ = getNodeHandle().subscribe("pose", 1, &TwistToIwsNodelet::odom_cb, this);
  pub_joint_iws_ = getNodeHandle().advertise<tuw_nav_msgs::JointsIWS>("joint_cmd",1000);
}

void TwistToIwsNodelet::odom_cb(const nav_msgs::OdometryPtr &msg)
{
//  if (msg->revolute.size() != 2)
//  {
//    ROS_ERROR("TwistToIwsNodelet::iws_cb: message does not contain two revolute commands. Will not set the command.");
//    return;
//  }
//  std::string revolute_mode = msg->type_revolute;
//  if (revolute_mode.compare("cmd_velocity"))
//  {
//    ROS_ERROR("TwistToIwsNodelet::iws_cb: revolute command type not supported. Will not set the command.");
//    return;
//  }

//  double dummy_wheel_distance = 1.5;
//  const double vL = msg->revolute[1];
//  const double vR = msg->revolute[0];

//  double omega = (vR - vL) / dummy_wheel_distance;
//  double v = (vR - vL) / 2.0;

  joint_iws_ = boost::make_shared<tuw_nav_msgs::JointsIWS>();
  pub_joint_iws_.publish(joint_iws_);
}

PLUGINLIB_EXPORT_CLASS(tuw::TwistToIwsNodelet, nodelet::Nodelet)
