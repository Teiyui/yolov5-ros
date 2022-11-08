from __future__ import annotations

import multiprocessing

import numpy as np
import pytest
import rosgraph
import roslaunch
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from yolov5_ros_msgs.msg import BoundingBoxes

from ._manuals import ManualPublisher, ManualSubscriber

INPUT_IMAGE_TOPIC = "/head_camera/rgb/image_rect_color/compressed"

OUTPUT_IMAGE_TOPIC = "/yolov5_ros/output/image/compressed"
OUTPUT_BBOX_TOPIC = "/yolov5_ros/output/bboxes"

COMMAND = "roslaunch yolov5_ros yolov5.launch"

IMAGE_SIZE = (480, 640)


def run_roslaunch(command):
    roslaunch.main(argv=command.split(" "))


@pytest.fixture(scope="function", autouse=True)
def scope_function():
    print(COMMAND)
    process = multiprocessing.Process(target=run_roslaunch, args=(COMMAND,))
    process.start()

    yield

    process.terminate()
    process.join()


class TestPubSub:
    @pytest.mark.timeout(10)
    def test(self) -> None:
        cv_bridge = CvBridge()

        while (not rospy.is_shutdown()) and (not rosgraph.masterapi.is_online()):
            rospy.sleep(0.1)

        image_pub = ManualPublisher(INPUT_IMAGE_TOPIC, CompressedImage)
        output_image_sub = ManualSubscriber(OUTPUT_IMAGE_TOPIC, CompressedImage)
        output_bbox_sub = ManualSubscriber(OUTPUT_BBOX_TOPIC, BoundingBoxes)

        image_pub.wait_for_connections()
        output_image_sub.wait_for_connections()
        output_bbox_sub.wait_for_connections()

        image_msg = cv_bridge.cv2_to_compressed_imgmsg((np.random.randint(0, 256, IMAGE_SIZE, np.uint8)))
        image_msg.header = Header()
        image_pub.publish(image_msg)

        output_image_sub.wait_for_message()
        output_bbox_sub.wait_for_message()

        output_image = cv_bridge.compressed_imgmsg_to_cv2(output_image_sub.get_msg())

        assert output_image.shape == (*IMAGE_SIZE, 3)
