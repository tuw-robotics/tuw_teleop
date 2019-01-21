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

int main(int argc, char **argv) {
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;
    tuw_teleop::GamepadNode teleop(nh);

    exit(0);
    return 0;
}

namespace tuw_teleop
{

    GamepadNode::GamepadNode(ros::NodeHandle &n) : n_(n), n_param_("~"), req_vx_(0), req_vy_(0), req_al_(0), req_ar_(0), req_vw_(0), req_scale_(1.0) {

        //reconfigure stuff
        reconfigureFnc_ = boost::bind(&GamepadNode::callbackConfig, this, _1, _2);
        reconfigureServer_.setCallback(reconfigureFnc_);

        n_param_.param<std::string>("command_type", command_type, "twist_diffdrive");
        if (this->command_type == "twist_diffdrive") {
            this->publisher_type_ = PublisherType::TWIST_DIFFDRIVE_COMMANDS;
        } else if (this->command_type == "iws_ackermann") {
            this->publisher_type_ = PublisherType::IWS_ACKERMANN_COMMANDS;
        } else if (this->command_type == "iws_diffdrive") {
            this->publisher_type_ = PublisherType::IWS_DIFFDRIVE_COMMANDS;
        } else if (this->command_type == "iwos_diffdrive") {
            this->publisher_type_ = PublisherType ::IWOS_DIFFDRIVE_COMMANDS;
        } else {
            ROS_ERROR ("command_type must be: twist_diffdrive, iws_ackermann, iws_diffdrive, iwos_diffdrive");
            exit(0);
        }

        switch (publisher_type_) {
            case TWIST_DIFFDRIVE_COMMANDS:
                ROS_INFO ("publisher_type_:  TWIST_DIFFDRIVE_COMMANDS");
                pub_cmd_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
                cmd_twist_.linear.x = cmd_twist_.linear.y = cmd_twist_.angular.z = 0.0;
                cmd_twist_passthrough_ = cmd_twist_;
                sub_cmd_passthrough_ = n_.subscribe("cmd_vel_passthrough", 10, &GamepadNode::callback_twist_passthrough, this);
                break;
            case IWS_DIFFDRIVE_COMMANDS:
                ROS_INFO ("publisher_type_:  IWS_DIFFDRIVE_COMMANDS");
                pub_cmd_ = n_.advertise<tuw_nav_msgs::JointsIWS>("joint_cmds", 1);
                cmd_iws_.header.seq = 0;
                cmd_iws_.header.stamp = ros::Time::now();
                cmd_iws_.type_steering = "";
                cmd_iws_.type_revolute = "cmd_velocity";
                cmd_iws_.revolute.resize(2);
                cmd_iws_.steering.resize(0);
                cmd_iws_.revolute[0] = 0;
                cmd_iws_.revolute[1] = 0;
                pub_cmd_.publish(cmd_iws_);
                cmd_iws_passthrough_ = cmd_iws_;
                sub_cmd_passthrough_ = n_.subscribe("joint_cmds_passthrough", 10, &GamepadNode::callback_iws_passthrough, this);
                break;
            case IWS_ACKERMANN_COMMANDS:
                ROS_INFO ("publisher_type_:  IWS_ACKERMANN_COMMANDS");
                pub_cmd_ = n_.advertise<tuw_nav_msgs::JointsIWS>("joint_cmds", 1);
                cmd_iws_.header.seq = 0;
                cmd_iws_.header.stamp = ros::Time::now();
                cmd_iws_.type_steering = "cmd_position";
                cmd_iws_.type_revolute = "cmd_velocity";
                cmd_iws_.revolute.resize(2);
                cmd_iws_.steering.resize(2);
                cmd_iws_passthrough_ = cmd_iws_;
                sub_cmd_passthrough_ = n_.subscribe("cmd_vel_passthrough", 10, &GamepadNode::callback_iws_passthrough, this);
                break;
            case IWOS_DIFFDRIVE_COMMANDS:
                this->l_trigger_active_ = false;
                this->r_trigger_active_ = false;
                this->n_param_.param<double>("steering_maximum", steering_maximum_, 0.4);
                ROS_INFO("publisher type: IWOS_DIFFDRIVE_COMMANDS");
                this->pub_cmd_ = this->n_.advertise<tuw_nav_msgs::JointsIWS>("joint_cmds", 1);
                this->cmd_iws_.header.seq = 0;
                this->cmd_iws_.header.stamp = ros::Time::now();
                this->cmd_iws_.type_steering = "cmd_position";
                this->cmd_iws_.type_revolute = "cmd_velocity";
                this->cmd_iws_.revolute.resize(2);
                this->cmd_iws_.revolute[0] = 0.0;
                this->cmd_iws_.revolute[1] = 0.0;
                this->cmd_iws_.steering.resize(2);
                this->cmd_iws_.steering[0] = 0.0;
                this->cmd_iws_.steering[1] = 0.0;
                this->pub_cmd_.publish(this->cmd_iws_);
                this->cmd_iws_passthrough_ = this->cmd_iws_;
                break;
            default:
                ROS_ERROR ("No such publisher type");
        }

        sub_joy_ = n_.subscribe("joy", 10, &GamepadNode::joy_cb, this);


        while (ros::ok()) {
            ros::spinOnce();
            publish_commands();
            rate_->sleep();
        }
    }

    GamepadNode::~GamepadNode() = default;

    void GamepadNode::callback_iws_passthrough(const tuw_nav_msgs::JointsIWSPtr &msg)
    {
        ROS_DEBUG ("callback_iws_passthrough: [%s,%s]", msg->type_steering.c_str(), msg->type_revolute.c_str());
        this->cmd_iws_passthrough_ = *msg;
    }

    void GamepadNode::callback_twist_passthrough(const geometry_msgs::TwistConstPtr &msg)
    {
        ROS_DEBUG ("cmd_twist_passthrough_: [%f,%f]", msg->linear.x, msg->angular.z);
        this->cmd_twist_passthrough_ = *msg;
    }

    bool GamepadNode::buttonsOK(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {

        if ((config_.scale_button >= 0) && (joy_msg->buttons.size() <= (unsigned int) config_.scale_button)) {
            ROS_ERROR ("Button scale_button %i does not exit!", config_.scale_button);
            return false;
        }
        if ((config_.deadman_button >= 0) && (joy_msg->buttons.size() <= (unsigned int) config_.deadman_button)) {
            ROS_ERROR ("Button deadman_button %i does not exit!", config_.deadman_button);
            return false;
        }
        if ((config_.passthrough_button >= 0) && (joy_msg->buttons.size() <= (unsigned int) config_.passthrough_button)) {
            ROS_ERROR ("Button passthrough_button %i does not exit!", config_.passthrough_button);
            return false;
        }
        if ((config_.axis_vx >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vx)) {
            ROS_ERROR ("Axis axis_vx %i does not exit!", config_.axis_vx);
            return false;
        }
        if ((config_.axis_vy >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vy)) {
            ROS_ERROR ("Axis axis_vy %i does not exit!", config_.axis_vy);
            return false;
        }
        if ((config_.axis_vw >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vw)) {
            ROS_ERROR ("Axis axis_vw %i does not exit!", config_.axis_vw);
            return false;
        }
        if ((config_.axis_vx_discrete >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vx_discrete)) {
            ROS_ERROR ("Axis axis_vx_discrete %i does not exit!", config_.axis_vx_discrete);
            return false;
        }
        if ((config_.axis_vy_discrete >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vy_discrete)) {
            ROS_ERROR ("Axis axis_vy_discrete %i does not exit!", config_.axis_vy_discrete);
            return false;
        }
        if ((config_.axis_vw_discrete >= 0) && (joy_msg->axes.size() <= (unsigned int) config_.axis_vw_discrete)) {
            ROS_ERROR ("Axis axis_vw_discrete %i does not exit!", config_.axis_vw_discrete);
            return false;
        }
        return true;
    }

    void GamepadNode::joy_cb(const sensor_msgs::Joy::ConstPtr &joy_msg) {

        if (this->config_.deadman_button >= 0){
            this->deadman_ = joy_msg->buttons[config_.deadman_button] != 0;
        }
        if (this->config_.passthrough_button>= 0) {
            this->passthrough_ = joy_msg->buttons[config_.passthrough_button] != 0;
        }

        if ((config_.scale_button >= 0) && joy_msg->buttons[config_.scale_button]) {
            req_scale_ = config_.scale;
        } else {
            req_scale_ = 1.0;
        }

        if (config_.debug) {
            ROS_INFO ("------- ");
            ROS_INFO ("max_vx:    %.3f", config_.max_vx);
            ROS_INFO ("max_vy:    %.3f", config_.max_vy);
            ROS_INFO ("max_vw:    %.3f", config_.max_vw);
            ROS_INFO ("req_scale: %.3f", req_scale_);
            std::stringstream ss_axis;
            for (unsigned int i = 0; i < joy_msg->axes.size(); i++) {
                ss_axis << (i == 0 ? " " : ", ") << "[" << i << "] = ";
                ss_axis << std::fixed << std::setw(9) << std::setprecision(6) << joy_msg->axes[i];
            }
            ROS_INFO ("axis    %s", ss_axis.str().c_str());
            std::stringstream ss_button;
            for (unsigned int i = 0; i < joy_msg->buttons.size(); i++) {
                ss_button << (i == 0 ? " " : ", ") << "[" << i << "] = ";
                ss_button << std::fixed << std::setw(4) << joy_msg->buttons[i];
            }
            ROS_INFO ("buttons %s", ss_button.str().c_str());
        }

        //Record this message receipt
        last_received_joy_message_time_ = ros::Time::now();

        // Base

        this->req_vx_ = 0.0;
        this->req_vy_ = 0.0;

        this->req_al_ = 0.0;
        this->req_ar_ = 0.0;

        this->req_vw_ = 0.0;

        if (!buttonsOK(joy_msg)) return;

        if (this->publisher_type_ == PublisherType::IWOS_DIFFDRIVE_COMMANDS) {

            if (this->config_.axis_vx >= 0) {
                if (this->l_trigger_active_  == false) {
                    this->req_vx_ = 1.0;
                    if (joy_msg->axes[this->config_.axis_vx] != 0) {
                        this->l_trigger_active_ = true;
                    }
                } else {
                    this->req_vx_ = joy_msg->axes[this->config_.axis_vx];
                }
            }
            if (this->config_.axis_vy >= 0) {
                if (this->r_trigger_active_ == false) {
                    this->req_vy_ = 1.0;
                    if (joy_msg->axes[this->config_.axis_vy] != 0) {
                        this->r_trigger_active_ = true;
                    }
                } else {
                    this->req_vy_ = joy_msg->axes[this->config_.axis_vy];
                }
            }
            if (this->config_.axis_al >= 0) {
                this->req_al_ = joy_msg->axes[this->config_.axis_al];
            }
            if (this->config_.axis_ar >= 0) {
                this->req_ar_ = joy_msg->axes[this->config_.axis_ar];
            }
            if (this->config_.button_rev_l >= 0) {
                this->button_rev_l_ = joy_msg->buttons[config_.button_rev_l] != 0;
            }
            if (this->config_.button_rev_r >= 0) {
                this->button_rev_r_ = joy_msg->buttons[config_.button_rev_r] != 0;
            }

        } else {
            if (this->config_.axis_vx >= 0) {
                this->req_vx_ = joy_msg->axes[this->config_.axis_vx] * this->config_.max_vx * this->req_scale_;
            }
            if (this->config_.axis_vy >= 0) {
                this->req_vy_ = joy_msg->axes[this->config_.axis_vy] * this->config_.max_vy * this->req_scale_;
            }
            if (this->config_.axis_vw >= 0) {
                this->req_vw_ = joy_msg->axes[this->config_.axis_vw] * this->config_.max_vw * this->req_scale_;
            }

            if (fabs(joy_msg->axes[config_.axis_vx_discrete]) > 0.9) {
                if (config_.axis_vx_discrete >= 0) {
                    req_vx_ = joy_msg->axes[config_.axis_vx_discrete] * config_.max_vx * req_scale_;
                }
            }
            if (fabs(joy_msg->axes[config_.axis_vy_discrete]) > 0.9) {
                if (config_.axis_vy_discrete >= 0) {
                    req_vy_ = joy_msg->axes[config_.axis_vy_discrete] * config_.max_vy * req_scale_;
                }
            }
            if (fabs(joy_msg->axes[config_.axis_vw_discrete]) > 0.9) {
                if (config_.axis_vw_discrete >= 0) {
                    req_vw_ = joy_msg->axes[config_.axis_vw_discrete] * config_.max_vw * req_scale_;
                }
            }
        }

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
        if (publisher_type_ == PublisherType::IWOS_DIFFDRIVE_COMMANDS) {
            deadman_ = true;
        }

        if (!deadman_ || (last_received_joy_message_time_ + joy_msg_timeout_ < ros::Time::now())) {
            req_vx_ = 0;
            req_vy_ = 0;
            req_vw_ = 0;
        }

        switch (publisher_type_) {
            case PublisherType::TWIST_DIFFDRIVE_COMMANDS:
                if (passthrough_) {
                    pub_cmd_.publish(cmd_twist_passthrough_);
                } else {
                    cmd_twist_.linear.x = req_vx_;
                    cmd_twist_.linear.y = req_vy_;
                    cmd_twist_.angular.z = req_vw_;
                    pub_cmd_.publish(cmd_twist_);
                }
                break;
            case PublisherType::IWS_DIFFDRIVE_COMMANDS:
                if (passthrough_) {
                    pub_cmd_.publish(cmd_iws_passthrough_);
                } else {
                    cmd_iws_.header.seq++;
                    cmd_iws_.header.stamp = ros::Time::now();
                    double v = req_vx_, w = req_vw_;
                    double vl = v, vr = v;
                    if (fabs(w) > std::numeric_limits<double>::min()) {
                        double R = v * w, l = config_.wheel_displacement;
                        vl = w * (R - l / 2.);
                        vr = w * (R + l / 2.);
                    }
                    cmd_iws_.revolute[0] = vr / config_.wheel_radius;
                    cmd_iws_.revolute[1] = vl / config_.wheel_radius;
                    pub_cmd_.publish(cmd_iws_);
                }
                break;
            case PublisherType::IWS_ACKERMANN_COMMANDS:
                if (passthrough_) {
                    pub_cmd_.publish(cmd_iws_passthrough_);
                } else {
                    cmd_iws_.header.seq++;
                    cmd_iws_.header.stamp = ros::Time::now();
                    cmd_iws_.steering[0] = req_vw_;
                    cmd_iws_.revolute[0] = std::nan("1");
                    cmd_iws_.steering[1] = std::nan("1");
                    cmd_iws_.revolute[1] = req_vx_;
                    pub_cmd_.publish(cmd_iws_);
                }
                break;
            case PublisherType::IWOS_DIFFDRIVE_COMMANDS:
                if (this->passthrough_) {
                    this->pub_cmd_.publish(cmd_iws_passthrough_);
                } else {
                    this->cmd_iws_.header.seq++;
                    this->cmd_iws_.header.stamp = ros::Time::now();
                    this->cmd_iws_.revolute[0] = (-this->req_vx_ + 1.0) / 2.0 * this->req_scale_ * (this->button_rev_l_ ? -1 : 1);
                    this->cmd_iws_.revolute[1] = (-this->req_vy_ + 1.0) / 2.0 * this->req_scale_ * (this->button_rev_r_ ? -1 : 1);
                    this->cmd_iws_.steering[0] = this->req_al_ * this->steering_maximum_;
                    this->cmd_iws_.steering[1] = this->req_ar_ * this->steering_maximum_;
                    this->pub_cmd_.publish(this->cmd_iws_);
                }
                break;
        }
        return;
    }

    void GamepadNode::callbackConfig(tuw_gamepad::GamepadControlConfig &_config, uint32_t _level) {
        config_ = _config;

        rate_ = std::make_shared<ros::Rate>(config_.rate);

        if (config_.joy_msg_timeout <= 0) {
            joy_msg_timeout_ = ros::Duration().fromSec(9999999);   //DURATION_MAX;
            ROS_DEBUG ("joy_msg_timeout <= 0 -> no timeout");
        } else {
            joy_msg_timeout_.fromSec(config_.joy_msg_timeout);
            ROS_DEBUG ("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
        }
        ROS_DEBUG ("callbackGamepadControlConfig!");
    }
}

