#!/usr/bin/env python3

from abc import ABC
from abc import abstractmethod
from sensor_msgs.msg import Joy
from rospy import Publisher
from typing import Generic
from typing import Optional
from typing import TypeVar

T = TypeVar("T")


class AbstractConverter(Generic[T], ABC):
    """
    abstract class for converters
    implementations convert sensor_msgs.Joy to T
    """

    # overwrite in implementing classes
    NAME = "Abstract"

    @abstractmethod
    def __init__(self, message_type: T) -> None:
        """
        initializer for converter class
        the publisher should not be registered here
        """
        self.message_type: T = message_type

    @staticmethod
    @abstractmethod
    def get_name() -> str:
        """
        return the name of the converter
        :return: name of the converter (equal to the classname without the pending "Converter")
        """
        pass

    def get_message_type(self) -> T:
        """
        return the type of the converted message
        :return: type of the converted message
        """
        return self.message_type

    @abstractmethod
    def convert(self, input_message: Joy) -> Optional[T]:
        """
        convert a sensor_msgs.Joy message to T
        :param input_message: message to convert
        :return: converted message or None if the message should not be published
        """
        pass
