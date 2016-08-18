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


#include <tuw_gui2iws/tuw_gui2iws_node.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <boost/graph/graph_concepts.hpp>

using namespace tuw;
using namespace std;

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "blue_control" );  /// initializes the ros node with default name
    ros::NodeHandle n;

    double figurePixSize = 1024;
    double figureRadius  = 2;
    double figureGrid  = 0.0;

    Gui2IwsNode gui2IwsNode ( n );
    gui2IwsNode.init();
    gui2IwsNode.initFigure(figurePixSize, figureRadius, figureGrid);

    ros::Rate rate ( 30. );
    while ( ros::ok() ) {

	gui2IwsNode.plot();

        gui2IwsNode.publishJntsCmds();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/**
 * Constructor
 **/
Gui2IwsNode::Gui2IwsNode ( ros::NodeHandle & n )
    : Gui2Iws(ros::NodeHandle("~").getNamespace()),
    n_ ( n ),
    n_param_ ( "~" ){

    pub_jnts_cmd_     = n.advertise<tuw_iws_msgs::IwsCmd_VRAT_Vec>("base_cmds"  , 1);
    sub_joint_states_ = n.subscribe(      "joint_states", 1, &Gui2IwsNode::callbackJointStates, this );
    sub_odometry_     = n.subscribe(              "odom", 1, &Gui2IwsNode::callbackOdometry   , this );

    tf_listener_ = std::make_shared<tf::TransformListener>();
    
    reconfigureFnc_ = boost::bind ( &Gui2IwsNode::callbackConfigBlueControl, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}

void Gui2IwsNode::callbackConfigBlueControl ( tuw_teleop::Gui2IwsConfig &config, uint32_t level ) {
    ROS_DEBUG ( "callbackConfigBlueControl!" );
    config_ = config;
    init();
}

void Gui2IwsNode::callbackJointStates( const sensor_msgs::JointState::ConstPtr &joint_ ){
    size_t jointStatesSize = joint_->name.size() / 2;
    jointStates_.resize(jointStatesSize);
    legInfo     .resize(jointStatesSize);
    for ( std::size_t i = 0; i < jointStates_.size(); i++ ) {
	
	tf::StampedTransform transform;
	std::vector<std::string> substrings;
	std::string linkName = joint_->name[i]; 
	
	typedef const boost::iterator_range<std::string::const_iterator> StringRange;
	std::string a2("2");
	if(!boost::find_first(StringRange(linkName.begin(), linkName.end()), StringRange(a2.begin(),a2.end()))){ return; }
	boost::split(substrings, linkName, boost::is_any_of("2") );
	boost::replace_all(substrings[1], "joint", "link"); 
	try {
	    tf_listener_->lookupTransform ( tf::resolve ( n_.getNamespace(), "base_link" ),  tf::resolve ( n_.getNamespace(), substrings[1] ), ros::Time ( 0 ), transform );
	} catch ( tf::TransformException ex ) {
	    ROS_ERROR ( "%s",ex.what() );
	    continue;
	}
	legInfo[i] = cv::Point2d ( - transform.getOrigin().y(),  transform.getOrigin().x() );
	
	jointStates_[i][asInt(JointsTypes::REVOL)] = joint_->velocity[i];
	jointStates_[i][asInt(JointsTypes::STEER)] = normalizeAngle(joint_->position[i+jointStatesSize]);
    }
    ///@todo compute body state now estimate from those ones lol
}


void Gui2IwsNode::callbackOdometry ( const nav_msgs::Odometry::ConstPtr& odom_){
    double vx = odom_->twist.twist. linear.x;
    double vy = odom_->twist.twist. linear.y;
    double w  = odom_->twist.twist.angular.z;

    double roll, pitch, yaw;
    tf::Quaternion qt = tf::Quaternion ( odom_->pose.pose.orientation.x,
		                	 odom_->pose.pose.orientation.y,
					 odom_->pose.pose.orientation.z,
					 odom_->pose.pose.orientation.w );
    tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);

    xBody = odom_->pose.pose.position.x;
    yBody = odom_->pose.pose.position.y;
    aBody = yaw;
    
    double vNorm = sqrt(vx*vx + vy*vy);
    if(fabs(vNorm) < 1e-1){//if robot is stationary, current parametric state is estimated along the pre-planned trajectory
// 	if(shouldCurState.size() == 0){ shouldCurState.resize(1); idxShouldCurrState = 0; shouldCurState[0].ICC.rho() = 0.0000001; }
// 	curState = shouldCurState[fmin(idxShouldCurrState,shouldCurState.size() - 1)];
	///@todo compute body state now estimate from those ones lol
    }
    else{//otherwise it is extracted from the robot base state
	bodyStateNow_[asInt(BodyVRP::V  )] = vNorm;
	bodyStateNow_[asInt(BodyVRP::RHO)] = - w  /  vNorm ;
	bodyStateNow_[asInt(BodyVRP::PHI)] = normalizeAngle( atan2(vy,vx) - yaw );
    }
}

void Gui2IwsNode::publishJntsCmds () {

    if(!new_trajectory){ return; }
    new_trajectory = false;

    tuw_iws_msgs::IwsCmd_VRAT_Vec jnts_cmd;
    jnts_cmd.header.stamp = ros::Time::now();
    jnts_cmd.deltaT.resize(2);
    jnts_cmd.v     .resize(2); 
    jnts_cmd.rho   .resize(2); 
    jnts_cmd.phi   .resize(2); 
    
    jnts_cmd.deltaT[0] = 0;
    jnts_cmd.v     [0] = bodyStateNow_[asInt(BodyVRP::V  )];
    jnts_cmd.rho   [0] = bodyStateNow_[asInt(BodyVRP::RHO)];
    jnts_cmd.phi   [0] = bodyStateNow_[asInt(BodyVRP::PHI)]/* - M_PI/2.*/;
    
    jnts_cmd.deltaT[1] = computeBodyStateTargetDeltaT();
    jnts_cmd.v     [1] = bodyStateTarget_[asInt(BodyVRP::V  )];
    jnts_cmd.rho   [1] = bodyStateTarget_[asInt(BodyVRP::RHO)];
    jnts_cmd.phi   [1] = bodyStateTarget_[asInt(BodyVRP::PHI)]/* - M_PI/2.*/;
    
    jnts_cmd.state0.resize(asInt(StateSimIwsVRP::StateVars::ENUM_SIZE) );
    
    jnts_cmd.state0[asInt(StateSimIwsVRP::StateVars::X    )] = xBody;
    jnts_cmd.state0[asInt(StateSimIwsVRP::StateVars::Y    )] = yBody;
    jnts_cmd.state0[asInt(StateSimIwsVRP::StateVars::THETA)] = aBody;
    jnts_cmd.state0[asInt(StateSimIwsVRP::StateVars::V    )] = jnts_cmd.v     [0];
    jnts_cmd.state0[asInt(StateSimIwsVRP::StateVars::RHO  )] = jnts_cmd.rho   [0];
    jnts_cmd.state0[asInt(StateSimIwsVRP::StateVars::ALPHA)] = jnts_cmd.phi   [0];
    jnts_cmd.state0[asInt(StateSimIwsVRP::StateVars::T    )] = 0;
    jnts_cmd.state0[asInt(StateSimIwsVRP::StateVars::S    )] = 0;

    ROS_INFO("Begin Param Funcs State: v=%lf, rho=%lf, phi=%lf", jnts_cmd.v[0], jnts_cmd.rho[0], jnts_cmd.phi[0]);
    ROS_INFO("End Param Funcs State: v=%lf, rho=%lf, phi=%lf", jnts_cmd.v[1], jnts_cmd.rho[1], jnts_cmd.phi[1]);
    
    pub_jnts_cmd_.publish ( jnts_cmd );
}
