/***************************************************************************
 *   Copyright (C) 2010 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <tuw_keyboard/tuw_keyboard_node.h>
#include <boost/thread.hpp>
#include <ncurses.h>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/algorithm/string.hpp>

using namespace tuw;

int main ( int argc, char **argv ) {
    std::cout << "\n";
    ros::init ( argc, argv, "ar_pose" );
    ros::NodeHandle n;
    KeyboardNode control ( n );
    return 0;
}

KeyboardNode::KeyboardNode ( ros::NodeHandle & n ) :
    Keyboard(), n_ ( n ), n_param_ ( "~" ), frequency_ ( 10.0 ) {

    /// defines a publisher for velocity commands
    
    //reconfigure stuff
    reconfigureFnc_    = boost::bind (&KeyboardNode::callbackConfig, this,  _1, _2);
    reconfigureServer_.setCallback (reconfigureFnc_);
    
    
    std::string command_type;
    n_param_.param<std::string> ("command_type", command_type, "twist_diffdrive");
    if (boost::iequals (command_type, "twist_diffdrive"))  publisher_type_ = TWIST_DIFFDRIVE_COMMANDS;
    else if (boost::iequals (command_type, "iws_ackermann"))  publisher_type_ = IWS_ACKERMANN_COMMANDS;
    else if (boost::iequals (command_type, "iws_diffdrive"))  publisher_type_ = IWS_DIFFDRIVE_COMMANDS;
    else {
      ROS_ERROR ("command_type must be: twist_diffdrive, iws_ackermann or iws_diffdrive");
      exit (0);
    }

	
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1 );

    

  switch (publisher_type_) {
  case TWIST_DIFFDRIVE_COMMANDS:
    ROS_INFO ("publisher_type_:  TWIST_DIFFDRIVE_COMMANDS");
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1 );
    cmd_twist_.linear.x = cmd_twist_.linear.y = cmd_twist_.angular.z = 0.0;
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
    break;
  default:
    ROS_ERROR ("No such publisher type");
  }

    initTeleop();
    ros::Rate rate ( frequency_ );
    while ( ros::ok() && !quit() ) {
        publishCmd();
        ros::spinOnce();
        rate.sleep();
    }
    publishCmd();
}

KeyboardNode::~KeyboardNode () {
}

void KeyboardNode::callbackConfig (tuw_keyboard::KeyboardControlConfig &_config, uint32_t _level){
  config_ = _config;
  frequency_ = config_.rate;
  velocity_forward_ = config_.init_v;
  velocity_angular_ = config_.init_w;
  velocity_forward_max_ = config_.max_v;
  velocity_angular_max_ = config_.max_w;
  velocity_forward_steps_ = config_.steps_v;
  velocity_angular_steps_ = config_.steps_w;
  
  ROS_DEBUG ("callbackKeyboardControlConfig!");
}

    
void KeyboardNode::publishCmd () {
    boost::interprocess::scoped_lock<boost::mutex> scoped_lock ( mutex_ );

  switch (publisher_type_) {
  case TWIST_DIFFDRIVE_COMMANDS:
    cmd_twist_.linear.x = cmd_.v();
    cmd_twist_.angular.z = cmd_.w();
    pub_cmd_.publish ( cmd_twist_ );
    break;
  case IWS_DIFFDRIVE_COMMANDS:
    {
     cmd_iws_.header.seq++;
     cmd_iws_.header.stamp = ros::Time::now();
     double v = cmd_.v(), w = cmd_.w();
     double vl = v, vr = v;
     if (fabs (w) > std::numeric_limits<double>::min()) {
       double R = v*w, l = config_.wheel_displacement;
       vl = w * (R-l/2.);
       vr = w * (R+l/2.);
     }
     cmd_iws_.revolute[0] = vr/config_.wheel_radius;
     cmd_iws_.revolute[1] = vl/config_.wheel_radius;
     pub_cmd_.publish (cmd_iws_);
    }
    break;
  default:
    ROS_ERROR ("No such publisher type");
  }
}
