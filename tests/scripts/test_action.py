from __future__ import annotations

import multiprocessing

import numpy as np
import pytest
import rosgraph
import roslaunch
import rospy
from actionlib import SimpleActionClient
from cv_bridge import CvBridge
from yolov5_ros_msgs.msg import DetectionAction, DetectionGoal

ACTION_TOPIC = "/yolov5_ros/action/detection"

COMMAND = "roslaunch yolov5_ros yolov5_objects365.launch"


def run_roslaunch(command):
    roslaunch.main(argv=command.split(" "))


@pytest.fixture(scope="function", autouse=True)
def scope_function():
    process = multiprocessing.Process(target=run_roslaunch, args=(COMMAND,))
    process.start()

    yield

    process.terminate()
    process.join()


class TestAction:
    @pytest.mark.timeout(10)
    def test(self) -> None:
        cv_bridge = CvBridge()

        while (not rospy.is_shutdown()) and (not rosgraph.masterapi.is_online()):
            rospy.sleep(0.1)

        client = SimpleActionClient(ACTION_TOPIC, DetectionAction)
        client.wait_for_server()

        image_msg = cv_bridge.cv2_to_compressed_imgmsg((np.random.randint(0, 256, (480, 640), np.uint8)))
        client.send_goal(DetectionGoal(image_msg))
        client.wait_for_result()

        result = client.get_result()
        assert result is not None
