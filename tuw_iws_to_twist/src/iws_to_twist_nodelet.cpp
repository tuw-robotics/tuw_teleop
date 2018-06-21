/**
 * @file iws_to_twist_nodelet.cpp
 * @author Felix König <felix.koenig@tuwien.ac.at>
 * @date June 2018
 * @brief Generates twist messages based on iws messages
 *
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
  twist_ = boost::make_shared<geometry_msgs::Twist>();
  dynamic_reconfigure_server = new dynamic_reconfigure::Server<tuw_iws_to_twist::iws_to_twist_nodeConfig>;
  sub_joint_iws_ = getNodeHandle().subscribe("joint_cmds", 1, &IwsToTwistNodelet::iws_cb, this);
  pub_joint_iws_ = getNodeHandle().advertise<geometry_msgs::Twist>("cmd_vel",1000);
}

void IwsToTwistNodelet::dynamic_reconfigureCB(tuw_iws_to_twist::iws_to_twist_nodeConfig &config, uint32_t level)
{
  wheeldiameter = config.wheeldiameter;
  wheeldistance = config.wheeldistance;
  wheelradius = wheeldiameter / 2.0;
}

void IwsToTwistNodelet::iws_cb(const tuw_nav_msgs::JointsIWSConstPtr &msg)
{
//  if (msg->revolute.size() != 2)
//  {
//    ROS_ERROR("IwsToTwistNodelet::iws_cb: message does not contain two revolute commands. Will not set the command.");
//    return;
//  }
//  std::string revolute_mode = msg->type_revolute;
//  if (revolute_mode.compare("cmd_velocity"))
//  {
//    ROS_ERROR("IwsToTwistNodelet::iws_cb: revolute command type not supported. Will not set the command.");
//    return;
//  }

  const double vL = msg->revolute[1] * wheelradius;
  const double vR = msg->revolute[0] * wheelradius;

  double omega = (vR - vL) / wheeldistance;
  double v = (vR + vL) / 2.0;

  twist_ = boost::make_shared<geometry_msgs::Twist>();
  twist_->linear.x = v;
  twist_->linear.y = 0.0;
  twist_->linear.z = 0.0;

  twist_->angular.x = 0;
  twist_->angular.y = 0;
  twist_->angular.z = omega;

  pub_joint_iws_.publish(twist_);
}

PLUGINLIB_EXPORT_CLASS(tuw::IwsToTwistNodelet, nodelet::Nodelet)
