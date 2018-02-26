/***************************************************************************
 *   Copyright (C) 2013 by Markus Bader                                    *
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

#ifndef KEYBOARD_NODE_H
#define KEYBOARD_NODE_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tuw_nav_msgs/JointsIWS.h>

#include <tuw_keyboard/tuw_keyboard.h>
#include <tuw_keyboard/KeyboardControlConfig.h>

#include <dynamic_reconfigure/server.h>


#define KEY_ESC   0027
#define KEY_S     0330

namespace tuw {
  
class KeyboardNode : public Keyboard
{
public:

    KeyboardNode (ros::NodeHandle & n);
    ~KeyboardNode ();
    double frequency() { 
      return frequency_;
    }
    bool cancel(){
      return cancel_;
    };
    void publishCmd() ;  
private:
    enum PublisherType {
      TWIST_DIFFDRIVE_COMMANDS = 0,                // Geometry Msgs / Twist
      IWS_DIFFDRIVE_COMMANDS = 1,                   // tuw_nav_msgs / JointsIWS [0]/[0]
      IWS_ACKERMANN_COMMANDS = 2,                  // tuw_nav_msgs / JointsIWS [0]/[0]
    };
    
    // ROS local copy of messages
    geometry_msgs::Twist cmd_twist_, cmd_twist_passthrough_;
    tuw_nav_msgs::JointsIWS cmd_iws_, cmd_iws_passthrough_;
    
    PublisherType publisher_type_;
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    double frequency_;    
    bool cancel_;
    bool secureMode_;
    ros::Publisher pub_cmd_;
    ros::Subscriber sub_laser_;
    
    // ROS Dynamic reconfigure
    dynamic_reconfigure::Server<tuw_keyboard::KeyboardControlConfig> reconfigureServer_; ///< parameter server stuff
    dynamic_reconfigure::Server<tuw_keyboard::KeyboardControlConfig>::CallbackType reconfigureFnc_;///< parameter server stuff
    void callbackConfig (tuw_keyboard::KeyboardControlConfig &_config, uint32_t _level); ///< parameter server stuff
    tuw_keyboard::KeyboardControlConfig config_; ///< parameter server stuff
  
};

};
#endif // KEYBOARD_2_TWIST_NODE_H
