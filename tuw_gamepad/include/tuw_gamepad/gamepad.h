/**
 * @file joy2cmd_.cpp
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date June 2015
 * @brief Generates twist messages based on joy messages
 * The code is based on the teleop_base.cpp, 2008, Willow Garage and
 * 2010  David Feil-Seifer [dfseifer@usc.edu], Edward T. Kaszubski [kaszubsk@usc.edu]
 */


#ifndef TUW_GAMEPAD_GAMEPAD_NODE_H
#define TUW_GAMEPAD_GAMEPAD_NODE_H

#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <string>
#include <memory>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <tuw_nav_msgs/JointsIWS.h>

#include <dynamic_reconfigure/server.h>
#include <tuw_gamepad/GamepadControlConfig.h>

/**
 * @brief Class to generate twist messages base on TeleopBase written by Willow Garage
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * The default parameters are set for a Logitech Gamepad F710 with "MODE" off and the "XID/HID Switch" should be set on X
 **/
namespace tuw_teleop
{
    class GamepadNode
    {
        public:
            enum PublisherType {
                TWIST_DIFFDRIVE_COMMANDS = 0,                 // Geometry Msgs / Twist
                IWS_DIFFDRIVE_COMMANDS = 1,                   // tuw_nav_msgs / JointsIWS [2]/[0]
                IWS_ACKERMANN_COMMANDS = 2,                   // tuw_nav_msgs / JointsIWS [2]/[2]
                IWOS_DIFFDRIVE_COMMANDS = 3                   // tuw_nav_msgs / JointsIWS [2]/[2]
            };

             GamepadNode(ros::NodeHandle & n);
            ~GamepadNode();
        private:
            // ROS Handle
            ros::NodeHandle n_;
            ros::NodeHandle n_param_;
            std::shared_ptr<ros::Rate> rate_;

            // ROS Dynamic reconfigure
            dynamic_reconfigure::Server<tuw_gamepad::GamepadControlConfig> reconfigureServer_; ///< parameter server stuff
            dynamic_reconfigure::Server<tuw_gamepad::GamepadControlConfig>::CallbackType reconfigureFnc_;///< parameter server stuff
            void callbackConfig (tuw_gamepad::GamepadControlConfig &_config, uint32_t _level); ///< parameter server stuff
            tuw_gamepad::GamepadControlConfig config_; ///< parameter server stuff


            // ROS local copy of messages
            geometry_msgs::Twist cmd_twist_;
            geometry_msgs::Twist cmd_twist_passthrough_;
            tuw_nav_msgs::JointsIWS cmd_iws_;
            tuw_nav_msgs::JointsIWS cmd_iws_passthrough_;

            double req_vx_;     /// velocity along X, on IWOS left  wheel velocity
            double req_vy_;     /// velocity along Y, on IWOS right wheel velocity
            double req_al_;     /// for IWOS only
            double req_ar_;     /// for IWOS only
            double req_vw_;     /// velocity around Z, on IWOS not used
            double req_scale_;  /// scales velocity, on IWOS not used

            std::string command_type;
            bool deadman_;
            bool passthrough_;

            bool l_trigger_active_;
            bool r_trigger_active_;

            bool button_rev_l_;
            bool button_rev_r_;

            double steering_maximum_;


            PublisherType publisher_type_;

            ros::Time last_received_joy_message_time_;
            ros::Duration joy_msg_timeout_;

            ros::Publisher pub_cmd_;
            ros::Subscriber sub_cmd_passthrough_;
            ros::Subscriber sub_joy_;

            void callback_iws_passthrough (const tuw_nav_msgs::JointsIWSPtr& msg);

            void callback_twist_passthrough (const geometry_msgs::TwistConstPtr& msg);

            bool buttonsOK (const sensor_msgs::Joy::ConstPtr& joy_msg);
            void joy_cb (const sensor_msgs::Joy::ConstPtr& joy_msg);

            void publish_commands();

    };
}

#endif 
