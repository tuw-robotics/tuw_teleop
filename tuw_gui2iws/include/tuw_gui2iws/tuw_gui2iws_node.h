/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2015 by Horatiu George Todoran <todorangrg@gmail.com>   *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/


#ifndef GUI_2_IWS_NODE_H
#define GUI_2_IWS_NODE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>

#include <tuw_iws_msgs/IwsCmd_VRAT_Vec.h>
#include <tuw_iws_msgs/JointsIWS.h>
#include <tuw_gui2iws/tuw_gui2iws.h>

namespace tuw {
/**
 * class to cover the ros communication
 **/
class Gui2IwsNode : public Gui2Iws {
public:
    Gui2IwsNode ( ros::NodeHandle & n ); /// Constructor
    void publishJntsCmds ();      /// publishes the motion commands
private:
    ros::NodeHandle n_;         /// node handler to the root node
    ros::NodeHandle n_param_;   /// node handler to the current node
    //ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
    ros::Publisher pub_jnts_cmds_;    /// publisher for the motion commands

    ros::Publisher pub_jnts_cmd_;    /// publisher for the motion commands

    ros::Subscriber sub_joint_states_;
    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_cmds_;
    //void callbackLaser ( const sensor_msgs::LaserScan& );   /// callback function to execute on incoming sensor data
    void callbackJointStates( const tuw_iws_msgs::JointsIWS::ConstPtr &      );
    void callbackOdometry   ( const nav_msgs   ::Odometry  ::ConstPtr & odom_);
    void callbackConfigBlueControl ( tuw_teleop::Gui2IwsConfig &config, uint32_t level ); /// callback function on incoming parameter changes
    dynamic_reconfigure::Server<tuw_teleop::Gui2IwsConfig> reconfigureServer_; /// parameter server stuff
    dynamic_reconfigure::Server<tuw_teleop::Gui2IwsConfig>::CallbackType reconfigureFnc_;  /// parameter server stuff
    
    std::shared_ptr<tf::TransformListener> tf_listener_;
    
    double xBody, yBody, aBody;
};

}

#endif // GUI_2_IWS_NODE_H
