#!/usr/bin/env python3

import rospy
import yaml

from typing import Dict
from typing import Union


class MappingReader:
    """
    class to read a .yaml config file
    """

    def __int__(self):
        pass

    @staticmethod
    def read(file_path: str) -> Union[Dict, None]:
        """
        read a .yaml config file as a dict
        :param file_path: full path to the file to read
        :return: file as a dict
        """
        with open(file_path) as file:
            try:
                return dict(yaml.safe_load(file))
            except yaml.YAMLError as exception:
                rospy.logerr(exception)
                return None
