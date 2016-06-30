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

#include <tuw_keyboard2twist/tuw_keyboard2twist_node.h>
#include <boost/thread.hpp>
#include <ncurses.h>
#include <boost/interprocess/sync/scoped_lock.hpp>

using namespace tuw;

int main ( int argc, char **argv ) {
    std::cout << "\n";
    ros::init ( argc, argv, "ar_pose" );
    ros::NodeHandle n;
    Keyboard2TwistNode control ( n );
    ros::Rate rate ( control.frequency() );
    while ( ros::ok() && !control.quit() ) {
        control.publishCmd();
        ros::spinOnce();
	rate.sleep();
    }
    control.publishCmd();
    return 0;
}

Keyboard2TwistNode::Keyboard2TwistNode ( ros::NodeHandle & n ) :
    Keyboard2Twist(), n_ ( n ), n_param_ ( "~" ), frequency_ ( 10.0 ) {

    /// defines a publisher for velocity commands
    

    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "\tfrequency: %5.2f", frequency_ );

    n_param_.getParam ( "velocity_forward", velocity_forward_ );
    ROS_INFO ( "\tvelocity_forward %f", velocity_forward_ );	
	
    n_param_.getParam ( "velocity_angular", velocity_angular_ );
    ROS_INFO ( "\tvelocity_angular %f", velocity_angular_ );

    n_param_.getParam ( "velocity_forward_max", velocity_forward_max_ );
    ROS_INFO ( "\tvelocity_forward_max %f", velocity_forward_max_ );	
	
    n_param_.getParam ( "velocity_angular_max", velocity_angular_max_ );
    ROS_INFO ( "\tvelocity_angular_max %f", velocity_angular_max_ );

    n_param_.getParam ( "velocity_forward_steps", velocity_forward_steps_ );
    ROS_INFO ( "\tvelocity_forward_steps %f", velocity_forward_steps_ );	
	
    n_param_.getParam ( "velocity_angular_steps", velocity_angular_steps_ );
    ROS_INFO ( "\tvelocity_angular_steps %f", velocity_angular_steps_ );
	
    std::string mode ( CONTROL_MODE );
    n_param_.getParam ( "control_mode", mode );
    ROS_INFO ( "\tcontrol_mode: %s", mode.c_str() );
    
    
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1 );

    if ( mode.compare ( CONTROL_MODE_TELEOP ) == 0 ) {
        control_mode_ = TELEOP;
    } else if ( mode.compare ( CONTROL_MODE_WANDERER ) == 0 ) {
        control_mode_ = WANDERER;
    } else {
        ROS_INFO ( "\t incorrect mode: %s", mode.c_str() );
		quit_ = true;
    }

    initTeleop();
}

Keyboard2TwistNode::~Keyboard2TwistNode () {
}

void Keyboard2TwistNode::publishCmd () {
    boost::interprocess::scoped_lock<boost::mutex> scoped_lock ( mutex_ );
    geometry_msgs::Twist twist;
    twist.linear.x = cmd_.v();
    twist.angular.z = cmd_.w();
    pub_cmd_.publish ( twist );
}
