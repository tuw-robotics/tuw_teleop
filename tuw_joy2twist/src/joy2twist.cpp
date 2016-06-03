/**
 * @file joy2cmd.cpp
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date June 2015
 * @brief Generates twist messages based on joy messages
 * The code is based on the teleop_base.cpp, 2008, Willow Garage and 
 * 2010  David Feil-Seifer [dfseifer@usc.edu], Edward T. Kaszubski [kaszubsk@usc.edu]
 */

#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief Class to generate twist messages base on TeleopBase written by Willow Garage
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * The default parameters are set for a Logitech Gamepad F710 with "MODE" off and the "XID/HID Switch" should be set on X
 **/
class Joy2Twist {
public:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    geometry_msgs::Twist cmd, passthrough_cmd;
    bool debug;
    double scale;
    double req_vx, req_vy, req_vw, req_scale;
    double max_vx, max_vy, max_vw;
    int axis_vx, axis_vy, axis_vw;
    int axis_vx_discrete, axis_vy_discrete, axis_vw_discrete;
    int deadman_button, scale_button;
    bool deadman_no_publish_;
    bool deadman_;

    ros::Time last_recieved_joy_message_time_;
    ros::Duration joy_msg_timeout_;

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber passthrough_sub_;

    Joy2Twist ( ros::NodeHandle & n, bool deadman_no_publish = false ) :
        n_ ( n ), n_param_ ( "~" ), 
        req_vx(0), req_vy(0), req_vw(0), req_scale(1.0),
        deadman_no_publish_ (deadman_no_publish ) {
    }

    void init() {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0.0;
	
        n_param_.param ( "debug", debug, false );
	
        n_param_.param ( "scale", scale,  2.0);  /// if the scale button is pressed it will scale all values
	
        n_param_.param ( "axis_vx", axis_vx,  1);
        n_param_.param ( "axis_vy", axis_vy, -1);
        n_param_.param ( "axis_vw", axis_vw,  0);

        n_param_.param ( "axis_vx_discrete", axis_vx_discrete,  7);
        n_param_.param ( "axis_vy_discrete", axis_vy_discrete, -1);
        n_param_.param ( "axis_vw_discrete", axis_vw_discrete,  6);

        n_param_.param ( "max_vx", max_vx, 1. );
        n_param_.param ( "max_vy", max_vy, 0. );
        n_param_.param ( "max_vw", max_vw, 1. );

        n_param_.param ( "deadman_button", deadman_button, 5 );
        n_param_.param ( "scale_button", scale_button, 4 );

        double joy_msg_timeout;
        n_param_.param ( "joy_msg_timeout", joy_msg_timeout, -1.0 ); //default to no timeout
        if ( joy_msg_timeout <= 0 ) {
            joy_msg_timeout_ = ros::Duration().fromSec ( 9999999 ); //DURATION_MAX;
            ROS_DEBUG ( "joy_msg_timeout <= 0 -> no timeout" );
        } else {
            joy_msg_timeout_.fromSec ( joy_msg_timeout );
            ROS_DEBUG ( "joy_msg_timeout: %.3f", joy_msg_timeout_.toSec() );
        }

        ROS_INFO ( "Negative button or axis index indicates an unused functionality!!!");
	
        ROS_INFO ( "         axis_vx = %2d. %s", axis_vx, (axis_vx < 0)?"  >> unused << ":"  >> used << " );
        ROS_INFO ( "         axis_vy = %2d. %s", axis_vy, (axis_vy < 0)?"  >> unused << ":"  >> used << " );
        ROS_INFO ( "         axis_vw = %2d. %s", axis_vw, (axis_vw < 0)?"  >> unused << ":"  >> used << " );
	
        ROS_INFO ( "axis_vx_discrete = %2d. %s", axis_vx_discrete, (axis_vx_discrete < 0)?"  >> unused << ":"  >> used << " );
        ROS_INFO ( "axis_vy_discrete = %2d. %s", axis_vy_discrete, (axis_vy_discrete < 0)?"  >> unused << ":"  >> used << " );
        ROS_INFO ( "axis_vw_discrete = %2d. %s", axis_vw_discrete, (axis_vw_discrete < 0)?"  >> unused << ":"  >> used << " );
	
        ROS_INFO ( "deadman_button   = %2d. %s", deadman_button, (deadman_button < 0)?"  >> unused << ":"  >> used << " );
        ROS_INFO ( "scale_button     = %2d. %s", scale_button, (scale_button < 0)?"  >> unused << ":"  >> used << " );
        ROS_INFO ( "joy_msg_timeout: %f", joy_msg_timeout );
	
        ROS_INFO ( "max_vx: %.3f   m/s", max_vx );
        ROS_INFO ( "max_vy: %.3f   m/s", max_vy );
        ROS_INFO ( "max_vw: %.3f rad/s", max_vw );



        vel_pub_ = n_.advertise < geometry_msgs::Twist > ( "cmd_vel", 1 );
        passthrough_sub_ = n_.subscribe ( "des_vel", 10,
                                          &Joy2Twist::passthrough_cb, this );
        joy_sub_ = n_.subscribe ( "joy", 10, &Joy2Twist::joy_cb, this );

    }

    ~Joy2Twist() {
    }

    void passthrough_cb ( const geometry_msgs::TwistConstPtr& pass_msg ) {
        ROS_DEBUG ( "passthrough_cmd: [%f,%f]", passthrough_cmd.linear.x,
                    passthrough_cmd.angular.z );
        passthrough_cmd = *pass_msg;
    }

    bool buttonsOK ( const sensor_msgs::Joy::ConstPtr& joy_msg ) {

        if ((scale_button >= 0) && ( joy_msg->buttons.size() <= ( unsigned int ) scale_button ) )  {
            ROS_ERROR ( "Botton scale_button %i does not exit!", axis_vx );
            return false;
        }
        if ((deadman_button >= 0) && ( joy_msg->buttons.size() <= ( unsigned int ) deadman_button ) )  {
            ROS_ERROR ( "Botton deadman_button %i does not exit!", axis_vx );
            return false;
        }
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

        deadman_ = joy_msg->buttons[deadman_button];

        if ( !deadman_ ) return;

	if((scale_button >= 0) && joy_msg->buttons[scale_button])
	  req_scale = scale;
	else
	  req_scale = 1.0;
		
        if(debug){  
	  ROS_INFO ( "------- ");
	  ROS_INFO ( "max_vx:    %.3f", max_vx );
	  ROS_INFO ( "max_vy:    %.3f", max_vy );
	  ROS_INFO ( "max_vw:    %.3f", max_vw );
	  ROS_INFO ( "req_scale: %.3f", req_scale );
	  std::stringstream ss_axis;
	  for (unsigned int i = 0; i < joy_msg->axes.size(); i++){	    
	    ss_axis << (i==0?" ":", ") << "[" << i << "] = ";
	    ss_axis << std::fixed << std::setw( 9 ) << std::setprecision( 6 ) << joy_msg->axes[i];
	  }
	  ROS_INFO ( "axis    %s", ss_axis.str().c_str() );
	  std::stringstream ss_button;
	  for (unsigned int i = 0; i < joy_msg->buttons.size(); i++){	    
	    ss_button << (i==0?" ":", ") << "[" << i << "] = ";
	    ss_button << std::fixed << std::setw( 4 ) << joy_msg->buttons[i];
	  }
	  ROS_INFO ( "buttons %s", ss_button.str().c_str() );
	}
	  
        //Record this message reciept
        last_recieved_joy_message_time_ = ros::Time::now();

        // Base

	req_vx = req_vy = req_vw = 0.0;
	
        if ( !buttonsOK ( joy_msg ) ) return;

	if(axis_vx >= 0) req_vx = joy_msg->axes[axis_vx] * max_vx * req_scale;
        if(axis_vy >= 0) req_vy = joy_msg->axes[axis_vy] * max_vy * req_scale;
        if(axis_vw >= 0) req_vw = joy_msg->axes[axis_vw] * max_vw * req_scale;
	
        if(debug){
	  ROS_INFO ( ">>> Analog ");
	  ROS_INFO ( "axis_vx: %3i", axis_vx );
	  ROS_INFO ( "axis_vy: %3i", axis_vy );
	  ROS_INFO ( "axis_vw: %3i", axis_vw );
	  ROS_INFO ( "joy_msg->axes[axis_vx]: %.3f", joy_msg->axes[axis_vx] );
	  ROS_INFO ( "joy_msg->axes[axis_vy]: %.3f", joy_msg->axes[axis_vy] );
	  ROS_INFO ( "joy_msg->axes[axis_vw]: %.3f", joy_msg->axes[axis_vw] );
	  ROS_INFO ( "req_vx: %.3f", req_vx );
	  ROS_INFO ( "req_vy: %.3f", req_vy );
	  ROS_INFO ( "req_vw: %.3f", req_vw );
	}

        if ( fabs ( joy_msg->axes[axis_vx_discrete] ) > 0.9 ) {
                if(axis_vx_discrete >= 0) req_vx = joy_msg->axes[axis_vx_discrete] * max_vx * req_scale;
        }        
        if ( fabs ( joy_msg->axes[axis_vy_discrete] ) > 0.9 ) {
                if(axis_vy_discrete >= 0) req_vy = joy_msg->axes[axis_vy_discrete] * max_vy * req_scale;
        }
        if ( fabs ( joy_msg->axes[axis_vw_discrete] ) > 0.9 ) {
                if(axis_vw_discrete >= 0) req_vw = joy_msg->axes[axis_vw_discrete] * max_vw * req_scale;
        }

        if(debug){
	  ROS_INFO ( ">>> Discrete");
	  ROS_INFO ( "axis_vx_discrete: %3i", axis_vx_discrete );
	  ROS_INFO ( "axis_vy_discrete: %3i", axis_vy_discrete );
	  ROS_INFO ( "axis_vw_discrete: %3i", axis_vw_discrete );
	  ROS_INFO ( "joy_msg->axes[axis_vx_discrete]: %.3f", joy_msg->axes[axis_vx_discrete] );
	  ROS_INFO ( "joy_msg->axes[axis_vy_discrete]: %.3f", joy_msg->axes[axis_vy_discrete] );
	  ROS_INFO ( "joy_msg->axes[axis_vw_discrete]: %.3f", joy_msg->axes[axis_vw_discrete] );
	  ROS_INFO ( "req_vx: %.3f", req_vx );
	  ROS_INFO ( "req_vy: %.3f", req_vy );
	  ROS_INFO ( "req_vw: %.3f", req_vw );	  
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

    Joy2Twist teleop_base ( nh, no_publish );
    teleop_base.init();

    while ( ros::ok() ) {
        ros::spinOnce();
        teleop_base.send_cmd_vel();
        pub_rate.sleep();
    }

    exit ( 0 );
    return 0;
}

