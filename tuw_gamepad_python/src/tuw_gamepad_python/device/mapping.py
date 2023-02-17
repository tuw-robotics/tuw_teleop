#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Joy
from typing import List

from tuw_gamepad_python.device.mapping_reader import MappingReader


class Mapping:
    """
    Class to map sensor_msgs.Joy messages for ordered defined by device to defined order as below.
    The mapping is defined in a .yaml file where the position of the axis / button in the original massage list has to be defined.
    axes: ["x_left", "y_left", "x_right", "y_right", "trigger_left", "trigger_right", "x_cross", "y_cross"]
    buttons: ["a_button", "b_button", "x_button", "y_button", "shoulder_left", "shoulder_right", "joy_left", "joy_right", "select", "start"]
    """

    def __init__(self, device_name: str, device_config_path: str):
        """
        initializer for mapping class
        :param device_name: name of the device to load the mapping for (filename has to be the same as device name)
        :param device_config_path: path to the device config file folder (not to the file itself)
        """
        self.name = device_name
        mapping = MappingReader().read(file_path=device_config_path)
        self.axes_mapping = mapping["axes"]
        self.buttons_mapping = mapping["buttons"]
        self.axes_keys = self.get_axis_keys_list()
        self.buttons_keys = self.get_button_keys_list()

        for key in self.axes_mapping:
            if key not in self.axes_keys:
                rospy.logerr("invalid axis mapping: {}".format(key))

        for key in self.buttons_mapping:
            if key not in self.buttons_keys:
                rospy.logerr("invalid button mapping: {}".format(key))

    def get_name(self) -> str:
        """
        get the name of the current device for mapping
        :return: name of the current device for mapping
        """
        return self.name

    def map(self, input_message: Joy) -> Joy:
        """
        map sensor_msgs.Joy messages according to config to new order (order given in class documentation)
        :param input_message: message to map to new order
        :return: message mapped to new order
        """
        mapped_axes = self._map_axes(list(input_message.axes))
        mapped_buttons = self._map_buttons(list(input_message.buttons))

        if len(mapped_axes) != 8:
            rospy.logerr("too few axis values after mapping, check config file for device")

        if len(mapped_buttons) != 10:
            rospy.logerr("too few button values after mapping, check config file for device")

        return Joy(
            header=input_message.header,
            axes=mapped_axes,
            buttons=mapped_buttons)

    def _map_axes(self, axis_value_list: List[float]):
        mapped_axis_value_list = []
        for key in self.axes_keys:
            if self.axes_mapping[key] is not None:
                mapped_axis_value_list.append(axis_value_list[self.axes_mapping[key]])
            if self.axes_mapping[key] is None:
                mapped_axis_value_list.append(0.0)

        return mapped_axis_value_list

    def _map_buttons(self, button_value_list: List[float]) -> List[float]:
        mapped_button_value_list = []
        for key in self.buttons_keys:
            if self.buttons_mapping[key] is not None:
                mapped_button_value_list.append(button_value_list[self.buttons_mapping[key]])
            if self.buttons_mapping[key] is None:
                mapped_button_value_list.append(0.0)

        return mapped_button_value_list

    @staticmethod
    def get_axis_keys_list():
        return ["x_left",
                "y_left",
                "x_right",
                "y_right",
                "trigger_left",
                "trigger_right",
                "x_cross",
                "y_cross"]

    @staticmethod
    def get_button_keys_list():
        return ["a_button",
                "b_button",
                "x_button",
                "y_button",
                "shoulder_left",
                "shoulder_right",
                "joy_left",
                "joy_right",
                "select",
                "start"]
