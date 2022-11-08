from __future__ import annotations

from typing import Final, Generic, Type, TypeVar

import genpy
import rospy
from rospy import Publisher, Subscriber

T = TypeVar("T", bound=genpy.Message)


class ManualSubscriber(Generic[T]):

    _impl: Final[Subscriber]
    _msg: (T | None)

    def __init__(self, topic: str, data_class: Type[T]):
        self._impl = Subscriber(topic, data_class, self._callback)
        self._msg = None

    def _callback(self, msg: T):
        self._msg = msg

    def wait_for_message(self) -> None:
        if self._msg is not None:
            return

        r = rospy.Rate(30)
        while True:
            if self._msg is not None:
                return
            if rospy.is_shutdown():
                break
            r.sleep()

    def get_msg(self) -> T:
        return self._msg

    def wait_for_connections(self) -> None:
        r = rospy.Rate(30)
        while True:
            if 0 < self._impl.get_num_connections():
                return
            if rospy.is_shutdown():
                break
            r.sleep()


class ManualPublisher(Generic[T]):

    _impl: Final[Publisher]

    def __init__(self, topic: str, data_class: Type[T]):
        self._impl = Publisher(topic, data_class, queue_size=1)

    def publish(self, msg: T):
        self._impl.publish(msg)

    def wait_for_connections(self) -> None:
        r = rospy.Rate(30)
        while True:
            if 0 < self._impl.get_num_connections():
                return
            if rospy.is_shutdown():
                break
            r.sleep()
