/*
 * teleop_base
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*       modifications to teleop_base to work with p2os
 *       Copyright (C) 2010  David Feil-Seifer [dfseifer@usc.edu], Edward T. Kaszubski [kaszubsk@usc.edu]
 *
 *       This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *       the Free Software Foundation; either version 2 of the License, or
 *       (at your option) any later version.
 *
 *       This program is distributed in the hope that it will be useful,
 *       but WITHOUT ANY WARRANTY; without even the implied warranty of
 *       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *       GNU General Public License for more details.
 *
 *       You should have received a copy of the GNU General Public License
 *       along with this program; if not, write to the Free Software
 *       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *       MA 02110-1301, USA.
 */

#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class Teleop {
public:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    geometry_msgs::Twist cmd, passthrough_cmd;
    bool debug;
    double req_vx, req_vy, req_vw;
    double max_vx, max_vy, max_vw;
    int axis_vx, axis_vy, axis_vw;
    int axis_vx_discrete, axis_vw_discrete, axis_vy_discrete;
    int deadman_button;
    bool deadman_no_publish_;
    bool deadman_;

    ros::Time last_recieved_joy_message_time_;
    ros::Duration joy_msg_timeout_;

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber passthrough_sub_;

    Teleop ( ros::NodeHandle & n, bool deadman_no_publish = false ) :
        n_ ( n ), n_param_ ( "~" ), debug(false), req_vx(0), req_vy(0), req_vw(0), max_vx ( 0.25 ), max_vy ( 0.0 ), max_vw ( 0.25 ), deadman_no_publish_ (
            deadman_no_publish ) {
    }

    void init() {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0.0;
	
        n_param_.param ( "debug", debug, debug );
	
        n_param_.param ( "axis_vx", axis_vx, 1 );
        n_param_.param ( "axis_vy", axis_vy, -1 );
        n_param_.param ( "axis_vw", axis_vw, 0 );

        n_param_.param ( "axis_vx_discrete", axis_vx_discrete, 5 );
        n_param_.param ( "axis_vy_discrete", axis_vy_discrete, -1 );
        n_param_.param ( "axis_vw_discrete", axis_vw_discrete, 4 );

        n_param_.param ( "max_vx", max_vx, max_vx );
        n_param_.param ( "max_vy", max_vy, max_vy );
        n_param_.param ( "max_vw", max_vw, max_vw );

        n_param_.param ( "deadman_button", deadman_button, 5 );

        double joy_msg_timeout;
        n_param_.param ( "joy_msg_timeout", joy_msg_timeout, -1.0 ); //default to no timeout
        if ( joy_msg_timeout <= 0 ) {
            joy_msg_timeout_ = ros::Duration().fromSec ( 9999999 ); //DURATION_MAX;
            ROS_DEBUG ( "joy_msg_timeout <= 0 -> no timeout" );
        } else {
            joy_msg_timeout_.fromSec ( joy_msg_timeout );
            ROS_DEBUG ( "joy_msg_timeout: %.3f", joy_msg_timeout_.toSec() );
        }

        ROS_INFO ( "axis_vx is %s: %d", (axis_vw < 0)?"off":"on", axis_vx );
        ROS_INFO ( "axis_vy is %s: %d", (axis_vy < 0)?"off":"on", axis_vy );
        ROS_INFO ( "axis_vw is %s: %d", (axis_vw < 0)?"off":"on", axis_vw );
	
        ROS_INFO ( "axis_vx_discrete is %s: %d", (axis_vx_discrete < 0)?"off":"on", axis_vx_discrete );
        ROS_INFO ( "axis_vy_discrete is %s: %d", (axis_vy_discrete < 0)?"off":"on", axis_vy_discrete );
        ROS_INFO ( "axis_vw_discrete is %s: %d", (axis_vw_discrete < 0)?"off":"on", axis_vw_discrete );
	
        ROS_INFO ( "max_vx: %.3f   m/s", max_vx );
        ROS_INFO ( "max_vy: %.3f   m/s", max_vy );
        ROS_INFO ( "max_vw: %.3f rad/s", max_vw );


        ROS_INFO ( "deadman_button: %d", deadman_button );
        ROS_INFO ( "joy_msg_timeout: %f", joy_msg_timeout );

        vel_pub_ = n_.advertise < geometry_msgs::Twist > ( "cmd_vel", 1 );
        passthrough_sub_ = n_.subscribe ( "des_vel", 10,
                                          &Teleop::passthrough_cb, this );
        joy_sub_ = n_.subscribe ( "joy", 10, &Teleop::joy_cb, this );

    }

    ~Teleop() {
    }

    void passthrough_cb ( const geometry_msgs::TwistConstPtr& pass_msg ) {
        ROS_DEBUG ( "passthrough_cmd: [%f,%f]", passthrough_cmd.linear.x,
                    passthrough_cmd.angular.z );
        passthrough_cmd = *pass_msg;
    }

    bool buttonsOK ( const sensor_msgs::Joy::ConstPtr& joy_msg ) {

        if ((axis_vx >= 0) && ( joy_msg->axes.size() <= ( unsigned int ) axis_vx ) )  {
            ROS_ERROR ( "Botton axis_vx %i does not exit!", axis_vx );
            return false;
        }
        if ((axis_vy >= 0) && ( joy_msg->axes.size() <= ( unsigned int ) axis_vy ) )  {
            ROS_ERROR ( "Botton axis_vy %i does not exit!", axis_vy );
            return false;
        }
        if ((axis_vw >= 0) && ( joy_msg->axes.size() <= ( unsigned int ) axis_vw ) )  {
            ROS_ERROR ( "Botton axis_vw %i does not exit!", axis_vw );
            return false;
        }
        if ((axis_vx_discrete >= 0) && ( joy_msg->axes.size() <= ( unsigned int ) axis_vx_discrete ) )  {
            ROS_ERROR ( "Botton axis_vx_discrete %i does not exit!", axis_vx_discrete );
            return false;
        }
        if ((axis_vy_discrete >= 0) && ( joy_msg->axes.size() <= ( unsigned int ) axis_vy_discrete ) )  {
            ROS_ERROR ( "Botton axis_vy_discrete %i does not exit!", axis_vy_discrete );
            return false;
        }
        if ((axis_vw_discrete >= 0) && ( joy_msg->axes.size() <= ( unsigned int ) axis_vw_discrete ) )  {
            ROS_ERROR ( "Botton axis_vw_discrete %i does not exit!", axis_vw_discrete );
            return false;
        }
        return true;
    }
    void joy_cb ( const sensor_msgs::Joy::ConstPtr& joy_msg ) {

        deadman_ = ( ( ( unsigned int ) deadman_button < joy_msg->buttons.size() )
                     && joy_msg->buttons[deadman_button] );

        if ( !deadman_ ) return;

        //Record this message reciept
        last_recieved_joy_message_time_ = ros::Time::now();

        // Base

	req_vx = req_vy = req_vw = 0.0;
	
        if ( !buttonsOK ( joy_msg ) ) return;

	if(axis_vx > 0) req_vx = joy_msg->axes[axis_vx] * max_vx;
        if(axis_vy > 0) req_vy = joy_msg->axes[axis_vy] * max_vy;
        if(axis_vw > 0) req_vw = joy_msg->axes[axis_vw] * max_vw;

        if ( fabs ( joy_msg->axes[axis_vx_discrete] ) > 0.9 ) {
                if(axis_vx_discrete > 0) req_vx = joy_msg->axes[axis_vx_discrete] * max_vx;
        }        
        if ( fabs ( joy_msg->axes[axis_vy_discrete] ) > 0.9 ) {
                if(axis_vy_discrete > 0) req_vy = joy_msg->axes[axis_vy_discrete] * max_vy;
        }
        if ( fabs ( joy_msg->axes[axis_vw_discrete] ) > 0.9 ) {
                if(axis_vw_discrete > 0) req_vw = joy_msg->axes[axis_vw_discrete] * max_vw;
        }

        if(debug){
	  ROS_INFO ( "------------");
	  ROS_INFO ( "req_vx: %.3f", req_vx );
	  ROS_INFO ( "req_vy: %.3f", req_vy );
	  ROS_INFO ( "req_vw: %.3f", req_vw );
	  ROS_INFO ( "max_vx: %.3f", max_vx );
	  ROS_INFO ( "max_vy: %.3f", max_vy );
	  ROS_INFO ( "max_vw: %.3f", max_vw );
	  ROS_INFO ( "axis_vx: %3i", axis_vx );
	  ROS_INFO ( "axis_vy: %3i", axis_vy );
	  ROS_INFO ( "axis_vw: %3i", axis_vw );
	  ROS_INFO ( "joy_msg->axes[axis_vx]: %.3f", joy_msg->axes[axis_vx] );
	  ROS_INFO ( "joy_msg->axes[axis_vy]: %.3f", joy_msg->axes[axis_vy] );
	  ROS_INFO ( "joy_msg->axes[axis_vw]: %.3f", joy_msg->axes[axis_vw] );
	}
    }

    void send_cmd_vel() {
        if ( deadman_ && (last_recieved_joy_message_time_ + joy_msg_timeout_  > ros::Time::now()) ) {
            cmd.linear.x = req_vx;
            cmd.linear.y = req_vy;
            cmd.angular.z = req_vw;
            vel_pub_.publish ( cmd );
        } else {
            //cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
            cmd = passthrough_cmd;
            //if (!deadman_no_publish_)
            {
                vel_pub_.publish ( cmd ); //Only publish if deadman_no_publish is enabled

            }
        }
    }
};

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "teleop" );
    ros::NodeHandle nh;
    const char* opt_no_publish = "--deadman_no_publish";

    bool no_publish = false;
    for ( int i = 1; i < argc; i++ ) {
        if ( !strncmp ( argv[i], opt_no_publish, strlen ( opt_no_publish ) ) )
            no_publish = true;
    }

    ros::Rate pub_rate ( 20 );

    Teleop teleop_base ( nh, no_publish );
    teleop_base.init();

    while ( ros::ok() ) {
        ros::spinOnce();
        teleop_base.send_cmd_vel();
        pub_rate.sleep();
    }

    exit ( 0 );
    return 0;
}
