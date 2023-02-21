#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tuw_geometry_msgs.msg import TwistWithOrientation
from typing import Any
from typing import Dict
from typing import Optional

from tuw_gamepad_python.cfg import IWOSOrientationConverterDynamicConfig
from tuw_gamepad_python.device.index_constants import *
from tuw_gamepad_python.converter.abstract_converter import AbstractConverter


class JoyToTwistWithOrientation(AbstractConverter[TwistWithOrientation]):
    NAME = "TwistWithOrientation"

    def __init__(self):
        super().__init__(message_type=TwistWithOrientation)
        self.reconfigure_server: Optional[Server] = None
        self.config: Optional[Dict[str: Any]] = None
        self.bindings = {
            "linear_velocity": LEFT_JOY_UP_DOWN,
            "angular_velocity": LEFT_JOY_LEFT_RIGHT,
            "orientation": RIGHT_JOY_LEFT_RIGHT}
        self.deadman_buttons = [RIGHT_SHOULDER_BUTTON, LEFT_SHOULDER_BUTTON]
        self.last_alive_message: Optional[TwistWithOrientation] = None
        self.reconfigure_server = Server(type=IWOSOrientationConverterDynamicConfig,
                                         callback=self.reconfigure_callback,
                                         namespace=self.get_name())

    @staticmethod
    def get_name() -> str:
        return JoyToTwistWithOrientation.NAME

    def convert(self, input_message: Joy) -> Optional[TwistWithOrientation]:
        if not any(input_message.buttons[button] for button in self.deadman_buttons):
            if self.last_alive_message is None:
                return None
            else:
                no_motion_twist = Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0))
                no_motion_twist_with_orientation = TwistWithOrientation(
                    header=input_message.header,
                    twist=no_motion_twist,
                    orientation=self.last_alive_message.orientation)
                return no_motion_twist_with_orientation

        linear_x: float = input_message.axes[self.bindings["linear_velocity"]] * self.config["linear_velocity_scale"]
        angular_z: float = input_message.axes[self.bindings["angular_velocity"]] * self.config["angular_velocity_scale"]
        orientation: float = input_message.axes[self.bindings["orientation"]] * self.config["orientation_scale"]
        twist = Twist(
            linear=Vector3(x=linear_x, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angular_z))
        twist_with_orientation = TwistWithOrientation(
            header=input_message.header,
            twist=twist,
            orientation=orientation)
        self.last_alive_message = twist_with_orientation
        return twist_with_orientation

    def reconfigure_callback(self, config: Dict[str, Any], level: int) -> Dict[str, Any]:
        for config_parameter in ["linear_velocity_scale", "angular_velocity_scale", "orientation_scale"]:
            if config_parameter not in config:
                rospy.logerr("missing parameter {} in dynamic reconfigure - "
                             "changes have not been applied".format("orientation_scale"))

        if self.config != config:
            self.config = config

        return config
