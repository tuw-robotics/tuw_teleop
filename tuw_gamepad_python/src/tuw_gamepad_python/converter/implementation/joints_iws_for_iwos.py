#!/usr/bin/env python3

import rospy
import tf

from typing import Any
from typing import Dict
from typing import Optional

from dynamic_reconfigure.server import Server

from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from tuw_nav_msgs.msg import JointsIWS

from tuw_gamepad_python.cfg import JointsIwsForIwosConfig
from tuw_gamepad_python.device.index_constants import *
from tuw_gamepad_python.converter.abstract_converter import AbstractConverter


class JointsIwsForIwos(AbstractConverter[JointsIWS]):
    NAME = "JointsIwsForIwos"

    def __init__(self):
        super().__init__(message_type=JointsIWS)
        self.reconfigure_server: Optional[Server] = None
        self.config: Optional[Dict[str: Any]] = None
        self.control = {
            "linear_velocity": LEFT_JOY_UP_DOWN,
            "angular_velocity": LEFT_JOY_LEFT_RIGHT,
            "orientation": RIGHT_JOY_LEFT_RIGHT}
        self.deadman = [RIGHT_SHOULDER_BUTTON, LEFT_SHOULDER_BUTTON]
        self.last_alive_message: Optional[JointsIWS] = None
        self.message: JointsIWS = JointsIWS(
            header=Header(seq=0, stamp=rospy.get_rostime(), frame_id="base_link"),
            type_steering="cmd_position",
            type_revolute="cmd_velocity",
            steering=[0.0, 0.0],
            revolute=[0.0, 0.0])
        self.reconfigure_server = Server(type=JointsIwsForIwosConfig,
                                         callback=self.reconfigure_callback,
                                         namespace=self.get_name())
        self.wheel_displacement: float = 0.4

    @staticmethod
    def get_name() -> str:
        return JointsIwsForIwos.NAME

    def convert(self, input_message: Joy) -> Optional[JointsIWS]:
        if not any(input_message.buttons[button] for button in self.deadman):
            if self.last_alive_message is None:
                return None
            else:
                return JointsIWS(
                    header=Header(stamp=rospy.get_rostime(), frame_id="base_link"),
                    type_steering=self.message.type_steering,
                    type_revolute=self.message.type_revolute,
                    steering=self.message.steering,
                    revolute=[0.0, 0.0])

        linear_velocity: float = input_message.axes[self.control["linear_velocity"]] * self.config["linear_velocity_scale"]
        angular_velocity: float = input_message.axes[self.control["angular_velocity"]] * self.config["angular_velocity_scale"]
        orientation: float = input_message.axes[self.control["orientation"]] * self.config["orientation_scale"]

        target_revolute: Dict[str, float] = {"left": 0.0, "right": 0.0}
        target_steering: Dict[str, float] = {"left": -orientation, "right": -orientation}

        if abs(angular_velocity) < self.config.angular_velocity_threashold:
            target_revolute["left"] = linear_velocity
            target_revolute["right"] = linear_velocity

        if not abs(angular_velocity) < self.config.angular_velocity_threashold:
            v: float = linear_velocity
            w: float = angular_velocity
            b: float = self.lookup_wheel_displacement()

            target_revolute["left"] = w * ((v / w) - b / 2.0)
            target_revolute["right"] = w * ((v / w) + b / 2.0)

        # self.message.header.seq = self.message.header.seq + 1
        self.message.header.stamp = rospy.get_rostime()

        self.message.revolute = [target_revolute["left"], target_revolute["right"]]
        self.message.steering = [target_steering["left"], target_steering["right"]]

        self.last_alive_message = self.message

        return self.message

    def reconfigure_callback(self, config: Dict[str, Any], level: int) -> Dict[str, Any]:
        for config_parameter in ["linear_velocity_scale", "angular_velocity_scale", "orientation_scale"]:
            if config_parameter not in config:
                rospy.logerr("missing parameter {} in dynamic reconfigure - "
                             "changes have not been applied".format("orientation_scale"))

        if self.config != config:
            self.config = config

        return config

    def lookup_wheel_displacement(self) -> float:
        tf_listener = tf.TransformListener()

        translation = {"left": None, "right": None}  # {[x,y,z]}

        for side, value in translation.items():
            target_link = "wheel_link_" + side
            try:
                value, _ = tf_listener.lookupTransform('base_link', target_link,  rospy.Time(0))
                if value is not None:
                    translation[side] = value
                if value is None:
                    rospy.log_debug("failed to fetch current wheel displacement")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logdebug("failed to fetch current wheel displacement")
                continue

        if translation["left"] is not None and translation["right"] is not None:
            return translation["left"][1] - translation["right"][1]
        else:
            return self.wheel_displacement
