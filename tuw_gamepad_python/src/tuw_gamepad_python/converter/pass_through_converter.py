#!/usr/bin/env python3

from sensor_msgs.msg import Joy

from tuw_gamepad_python.converter.abstract_converter import AbstractConverter


class PassThroughConverter(AbstractConverter[Joy]):

    NAME = "PassThrough"

    def __init__(self):
        super().__init__(message_type=Joy)

    @staticmethod
    def get_name() -> str:
        return PassThroughConverter.NAME

    def convert(self, input_message: Joy) -> Joy:
        return input_message
