/**
 * @file joy2cmd_.cpp
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date June 2015
 * @brief Generates twist messages based on joy messages
 * The code is based on the teleop_base.cpp, 2008, Willow Garage and
 * 2010  David Feil-Seifer [dfseifer@usc.edu], Edward T. Kaszubski [kaszubsk@usc.edu]
 */

#include <boost/algorithm/string.hpp>
#include <tuw_gamepad/gamepad.h>

int main (int argc, char **argv) {
  ros::init (argc, argv, "teleop");
  ros::NodeHandle nh;
  const char* opt_no_publish = "--deadman_no_publish";

  bool no_publish = false;
  for (int i = 1; i < argc; i++) {
    if (!strncmp (argv[i], opt_no_publish, strlen (opt_no_publish)))
      no_publish = true;
  }

  GamepadNode teleop (nh, no_publish);

  exit (0);
  return 0;
}


GamepadNode::GamepadNode (ros::NodeHandle & n, bool deadman_no_publish) 
: n_ (n), n_param_ ("~"),
  req_vx_ (0), req_vy_ (0), req_vw_ (0), req_scale_ (1.0),
  deadman_no_publish_ (deadman_no_publish) {

  //reconfigure stuff
  reconfigureFnc_    = boost::bind (&GamepadNode::callbackConfig, this,  _1, _2);
  reconfigureServer_.setCallback (reconfigureFnc_);

  n_param_.param<std::string> ("command_type", command_type, "twist_diffdrive");
  if (boost::iequals (command_type, "twist_diffdrive"))  publisher_type_ = TWIST_DIFFDRIVE_COMMANDS;
  else if (boost::iequals (command_type, "iws_ackermann"))  publisher_type_ = IWS_ACKERMANN_COMMANDS;
  else if (boost::iequals (command_type, "iws_diffdrive"))  publisher_type_ = IWS_DIFFDRIVE_COMMANDS;
  else {
    ROS_ERROR ("command_type must be: twist_diffdrive, iws_ackermann or iws_diffdrive");
    exit (0);
  }

  switch (publisher_type_) {
  case TWIST_DIFFDRIVE_COMMANDS:
    ROS_INFO ("publisher_type_:  TWIST_DIFFDRIVE_COMMANDS");
    pub_cmd_ = n_.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
    cmd_twist_.linear.x = cmd_twist_.linear.y = cmd_twist_.angular.z = 0.0;
    cmd_twist_passthrough_ = cmd_twist_;
    sub_cmd_passthrough_ = n_.subscribe ("cmd_vel_passthrough", 10, &GamepadNode::callback_twist_passthrough, this);
    break;
  case IWS_ACKERMANN_COMMANDS:
    ROS_INFO ("publisher_type_:  IWS_ACKERMANN_COMMANDS");
    pub_cmd_ = n_.advertise < tuw_nav_msgs::JointsIWS > ("cmd_vel", 1);
    cmd_iws_.header.seq = 0;
    cmd_iws_.header.stamp = ros::Time::now();
    cmd_iws_.type_steering = "cmd_position";
    cmd_iws_.type_revolute = "cmd_velocity";
    cmd_iws_.revolute.resize (1);
    cmd_iws_.steering.resize (1);
    cmd_iws_passthrough_ = cmd_iws_;
    sub_cmd_passthrough_ = n_.subscribe ("cmd_vel_passthrough", 10, &GamepadNode::callback_twist_passthrough, this);
    break;
  case IWS_DIFFDRIVE_COMMANDS:
    ROS_INFO ("publisher_type_:  IWS_DIFFDRIVE_COMMANDS");
    pub_cmd_ = n_.advertise < tuw_nav_msgs::JointsIWS > ("joint_cmds", 1);
    cmd_iws_.header.seq = 0;
    cmd_iws_.header.stamp = ros::Time::now();
    cmd_iws_.type_steering = "";
    cmd_iws_.type_revolute = "cmd_velocity";
    cmd_iws_.revolute.resize (2);
    cmd_iws_.steering.resize (0);
    cmd_iws_.revolute[0] = 0;
    cmd_iws_.revolute[1] = 0;
    pub_cmd_.publish (cmd_iws_);
    cmd_iws_passthrough_ = cmd_iws_;
    sub_cmd_passthrough_ = n_.subscribe ("joint_cmds_passthrough", 10, &GamepadNode::callback_iws_passthrough, this);
    break;
  default:
    ROS_ERROR ("No such publisher type");
  }

  sub_joy_ = n_.subscribe ("joy", 10, &GamepadNode::joy_cb, this);
  


  while (ros::ok()) {
    ros::spinOnce();
    publish_commands();
    rate_->sleep();
  }
}

GamepadNode::~GamepadNode() {
}

void GamepadNode::callback_iws_passthrough (const tuw_nav_msgs::JointsIWS& msg) {
  ///@ToDo ROS_DEBUG ("callback_iws_passthrough: [%s,%s]", msg->type_steering.c_str(), msg->type_revolute.c_str());
  ///@ToDo cmd_iws_passthrough_ = *msg;
}

void GamepadNode::callback_twist_passthrough (const geometry_msgs::TwistConstPtr& msg) {
  ROS_DEBUG ("cmd_twist_passthrough_: [%f,%f]", msg->linear.x, msg->angular.z);
  cmd_twist_passthrough_ = *msg;
}

bool GamepadNode::buttonsOK (const sensor_msgs::Joy::ConstPtr& joy_msg) {

  if ( (config_.scale_button >= 0) && (joy_msg->buttons.size() <= (unsigned int) config_.scale_button))  {
    ROS_ERROR ("Button scale_button %i does not exit!", config_.axis_vx);
    return false;
  }
  if ( (config_.deadman_button >= 0) && (joy_msg->buttons.size() <= (unsigned int) config_.deadman_button))  {
    ROS_ERROR ("Button deadman_button %i does not exit!", config_.axis_vx);
    return false;
  }
  if ( (config_.axis_vx >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vx))  {
    ROS_ERROR ("Axis axis_vx %i does not exit!", config_.axis_vx);
    return false;
  }
  if ( (config_.axis_vy >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vy))  {
    ROS_ERROR ("Axis axis_vy %i does not exit!", config_.axis_vy);
    return false;
  }
  if ( (config_.axis_vw >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vw))  {
    ROS_ERROR ("Axis axis_vw %i does not exit!", config_.axis_vw);
    return false;
  }
  if ( (config_.axis_vx_discrete >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vx_discrete))  {
    ROS_ERROR ("Axis axis_vx_discrete %i does not exit!", config_.axis_vx_discrete);
    return false;
  }
  if ( (config_.axis_vy_discrete >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vy_discrete))  {
    ROS_ERROR ("Axis axis_vy_discrete %i does not exit!", config_.axis_vy_discrete);
    return false;
  }
  if ( (config_.axis_vw_discrete >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vw_discrete))  {
    ROS_ERROR ("Axis axis_vw_discrete %i does not exit!", config_.axis_vw_discrete);
    return false;
  }
  return true;
}

void GamepadNode::joy_cb (const sensor_msgs::Joy::ConstPtr& joy_msg) {

  deadman_ = joy_msg->buttons[config_.deadman_button];

  if (!deadman_) return;

  if ( (config_.scale_button >= 0) && joy_msg->buttons[config_.scale_button])
    req_scale_ = config_.scale;
  else
    req_scale_ = 1.0;

  if (config_.debug) {
    ROS_INFO ("------- ");
    ROS_INFO ("max_vx:    %.3f", config_.max_vx);
    ROS_INFO ("max_vy:    %.3f", config_.max_vy);
    ROS_INFO ("max_vw:    %.3f", config_.max_vw);
    ROS_INFO ("req_scale: %.3f", req_scale_);
    std::stringstream ss_axis;
    for (unsigned int i = 0; i < joy_msg->axes.size(); i++) {
      ss_axis << (i==0?" ":", ") << "[" << i << "] = ";
      ss_axis << std::fixed << std::setw (9) << std::setprecision (6) << joy_msg->axes[i];
    }
    ROS_INFO ("axis    %s", ss_axis.str().c_str());
    std::stringstream ss_button;
    for (unsigned int i = 0; i < joy_msg->buttons.size(); i++) {
      ss_button << (i==0?" ":", ") << "[" << i << "] = ";
      ss_button << std::fixed << std::setw (4) << joy_msg->buttons[i];
    }
    ROS_INFO ("buttons %s", ss_button.str().c_str());
  }

  //Record this message receipt
  last_recieved_joy_message_time_ = ros::Time::now();

  // Base

  req_vx_ = req_vy_ = req_vw_ = 0.0;

  if (!buttonsOK (joy_msg)) return;

  if (config_.axis_vx >= 0) req_vx_ = joy_msg->axes[config_.axis_vx] * config_.max_vx * req_scale_;
  if (config_.axis_vy >= 0) req_vy_ = joy_msg->axes[config_.axis_vy] * config_.max_vy * req_scale_;
  if (config_.axis_vw >= 0) req_vw_ = joy_msg->axes[config_.axis_vw] * config_.max_vw * req_scale_;

  if (config_.debug) {
    ROS_INFO (">>> Analog ");
    ROS_INFO ("axis_vx: %3i", config_.axis_vx);
    ROS_INFO ("axis_vy: %3i", config_.axis_vy);
    ROS_INFO ("axis_vw: %3i", config_.axis_vw);
    ROS_INFO ("joy_msg->axes[axis_vx]: %.3f", joy_msg->axes[config_.axis_vx]);
    ROS_INFO ("joy_msg->axes[axis_vy]: %.3f", joy_msg->axes[config_.axis_vy]);
    ROS_INFO ("joy_msg->axes[axis_vw]: %.3f", joy_msg->axes[config_.axis_vw]);
    ROS_INFO ("req_vx: %.3f", req_vx_);
    ROS_INFO ("req_vy: %.3f", req_vy_);
    ROS_INFO ("req_vw: %.3f", req_vw_);
  }

  if (fabs (joy_msg->axes[config_.axis_vx_discrete]) > 0.9) {
    if (config_.axis_vx_discrete >= 0) req_vx_ = joy_msg->axes[config_.axis_vx_discrete] * config_.max_vx * req_scale_;
  }
  if (fabs (joy_msg->axes[config_.axis_vy_discrete]) > 0.9) {
    if (config_.axis_vy_discrete >= 0) req_vy_ = joy_msg->axes[config_.axis_vy_discrete] * config_.max_vy * req_scale_;
  }
  if (fabs (joy_msg->axes[config_.axis_vw_discrete]) > 0.9) {
    if (config_.axis_vw_discrete >= 0) req_vw_ = joy_msg->axes[config_.axis_vw_discrete] * config_.max_vw * req_scale_;
  }

  if (config_.debug) {
    ROS_INFO (">>> Discrete");
    ROS_INFO ("axis_vx_discrete: %3i", config_.axis_vx_discrete);
    ROS_INFO ("axis_vy_discrete: %3i", config_.axis_vy_discrete);
    ROS_INFO ("axis_vw_discrete: %3i", config_.axis_vw_discrete);
    ROS_INFO ("joy_msg->axes[axis_vx_discrete]: %.3f", joy_msg->axes[config_.axis_vx_discrete]);
    ROS_INFO ("joy_msg->axes[axis_vy_discrete]: %.3f", joy_msg->axes[config_.axis_vy_discrete]);
    ROS_INFO ("joy_msg->axes[axis_vw_discrete]: %.3f", joy_msg->axes[config_.axis_vw_discrete]);
    ROS_INFO ("req_vx: %.3f", req_vx_);
    ROS_INFO ("req_vy: %.3f", req_vy_);
    ROS_INFO ("req_vw: %.3f", req_vw_);
  }
}

void GamepadNode::publish_commands() {
  if (!deadman_ || (last_recieved_joy_message_time_ + joy_msg_timeout_  < ros::Time::now())) {
    req_vx_ = cmd_twist_passthrough_.linear.x;
    req_vy_ = cmd_twist_passthrough_.linear.y;
    req_vw_ = cmd_twist_passthrough_.angular.z;
  }

  switch (publisher_type_) {
  case TWIST_DIFFDRIVE_COMMANDS:
    cmd_twist_.linear.x = req_vx_;
    cmd_twist_.linear.y = req_vy_;
    cmd_twist_.angular.z = req_vw_;
    pub_cmd_.publish (cmd_twist_);
    break;
  case IWS_ACKERMANN_COMMANDS:
    cmd_iws_.header.seq++;
    cmd_iws_.header.stamp = ros::Time::now();
    cmd_iws_.revolute[0] = req_vx_;
    cmd_iws_.steering[0] = req_vw_;
    pub_cmd_.publish (cmd_iws_);
    break;
  case IWS_DIFFDRIVE_COMMANDS:
    cmd_iws_.header.seq++;
    cmd_iws_.header.stamp = ros::Time::now();
    double v = req_vx_, w = req_vw_;
    double vl = v, vr = v;
    if (fabs (w) > std::numeric_limits<double>::min()) {
      double R = v*w, l = config_.wheel_displacement;
      vl = w * (R-l/2.);
      vr = w * (R+l/2.);
    }
    cmd_iws_.revolute[0] = vr; //config_.wheel_radius;
    cmd_iws_.revolute[1] = vl; //config_.wheel_radius;
    pub_cmd_.publish (cmd_iws_);
    break;
  }

}

void GamepadNode::callbackConfig (tuw_gamepad::GamepadControlConfig &_config, uint32_t _level) {
  config_ = _config;
  
  rate_ = std::make_shared<ros::Rate>(config_.rate);
  
  if (config_.joy_msg_timeout <= 0) {
    joy_msg_timeout_ = ros::Duration().fromSec (9999999);   //DURATION_MAX;
    ROS_DEBUG ("joy_msg_timeout <= 0 -> no timeout");
  } else {
    joy_msg_timeout_.fromSec (config_.joy_msg_timeout);
    ROS_DEBUG ("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
  }  
  ROS_DEBUG ("callbackGamepadControlConfig!");
}

