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

#ifndef KEYBOARD_2_TWIST_NODE_H
#define KEYBOARD_2_TWIST_NODE_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <tuw_keyboard2twist/tuw_keyboard2twist.h>

#define KEY_ESC   0027
#define KEY_S     0330

namespace tuw {
  
class Keyboard2TwistNode : public Keyboard2Twist
{
public:

    Keyboard2TwistNode (ros::NodeHandle & n);
    ~Keyboard2TwistNode ();
    double frequency() { 
      return frequency_;
    }
    bool cancel(){
      return cancel_;
    };
    void publishCmd() ;  
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    double frequency_;    
    bool cancel_;
    bool secureMode_;
    ros::Publisher pub_cmd_;
    ros::Subscriber sub_laser_;
};

};
#endif // KEYBOARD_2_TWIST_NODE_H
