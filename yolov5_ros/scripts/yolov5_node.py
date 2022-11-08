#! /usr/bin/env python
from __future__ import annotations

import time
from threading import Lock
from types import TracebackType
from typing import Final, Type

import cv2
import numpy as np
import rospy
import torch
from actionlib import SimpleActionServer
from cv_bridge import CvBridge
from numpy._typing import NDArray
from rospy import Publisher, Service, Subscriber
from sensor_msgs.msg import CompressedImage
from submodules.yolov5.models.yolo import Model
from submodules.yolov5.utils.general import non_max_suppression
from submodules.yolov5.utils.plots import colors
from torch import nn
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes, DetectionAction, DetectionGoal, DetectionResult
from yolov5_ros_msgs.srv import Detection, DetectionRequest, DetectionResponse

_ = torch.set_grad_enabled(False)


class _TimeLogger:

    _name: Final[str]
    _start_time: float

    def __init__(self, name: str):
        self._name = name
        self._start_time = 0

    def __enter__(self):
        self._start_time = time.perf_counter()

    def __exit__(self, exc_type: (Type[Exception] | None), exc_val: (Exception | None), exc_tb: (TracebackType | None)):
        elapsed = time.perf_counter() - self._start_time
        rospy.loginfo(f"Processing Time ({self._name}): {elapsed}[sec].")


class YoloV5Node:

    _score_threshold: Final[float]
    _iou_threshold: Final[float]
    _encoding: Final[str]
    _device: Final[torch.device]
    _lock: Final[Lock]
    _bridge: Final[CvBridge]

    _model: Final[nn.Module]
    _classes: Final[list[str]]

    _service: Final[Service]
    _server: Final[SimpleActionServer]
    _image_publisher: Final[Publisher]
    _bboxes_publisher: Final[Publisher]

    _subscribed_msg: (CompressedImage | None)

    def __init__(self):
        # ------------------------------------------------ #
        #   ROS Parameters
        # ------------------------------------------------ #
        self._score_threshold = rospy.get_param("~score_threshold")
        self._iou_threshold = rospy.get_param("~iou_threshold")
        self._encoding = rospy.get_param("~encoding")
        self._device = torch.device(rospy.get_param("~device"))
        config_path = rospy.get_param("~config_path")
        weight_path = rospy.get_param("~weights_path")
        # ------------------------------------------------ #
        #   Modules
        # ------------------------------------------------ #
        self._model, self._classes = self._load_model(config_path, weight_path, self._device)
        _ = self._model(torch.zeros((1, 3, 128, 128), dtype=torch.float16, device=self._device))
        self._lock = Lock()
        self._bridge = CvBridge()
        self._subscribed_msg = None

        # ------------------------------------------------ #
        #   ServiceProxy
        # ------------------------------------------------ #
        self._service = Service("/service/detection", Detection, self._service_callback)

        # ------------------------------------------------ #
        #   ActionServer
        # ------------------------------------------------ #
        self._server = SimpleActionServer("/action/detection", DetectionAction, self._action_call_back, False)

        # ------------------------------------------------ #
        #   ROS Publisher
        # ------------------------------------------------ #
        self._image_publisher = Publisher("/output/image/compressed", CompressedImage, queue_size=1)
        self._bboxes_publisher = Publisher("/output/bboxes", BoundingBoxes, queue_size=1)

        # ------------------------------------------------ #
        #   ROS Subscriber
        # ------------------------------------------------ #
        _ = Subscriber("/input/image/compressed", CompressedImage, self._image_callback, queue_size=10)

        # ------------------------------------------------ #
        #   Start
        # ------------------------------------------------ #
        self._server.start()
        rospy.loginfo("Ready...")

    @staticmethod
    def _load_model(config_path: str, weight_path: str, device: torch.device) -> tuple[nn.Module, list[str]]:
        rospy.loginfo("Loading model ...")

        model = Model(cfg=config_path)
        load_pt = torch.load(weight_path)
        pretrain_model = load_pt["model"]
        _ = model.load_state_dict(pretrain_model.state_dict())
        _ = model.to(torch.float16).to(device).eval()

        classes = pretrain_model.names

        return model, classes

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   Public Method
    #
    # ------------------------------------------------------------------------------------------------------------------
    def main(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self._subscribed_msg is None:
                r.sleep()
                continue

            with self._lock:
                msg = self._subscribed_msg
                self._subscribed_msg = None

            with _TimeLogger("Subscriber Callback"):
                image = self._bridge.compressed_imgmsg_to_cv2(msg, self._encoding)
                bboxes = self._predict_image(image)

                prediction_image = self._generate_result_images(image, bboxes)
                prediction_image_msg = self._bridge.cv2_to_compressed_imgmsg(prediction_image)
                prediction_image_msg.header.stamp = rospy.Time.now()
                self._image_publisher.publish(prediction_image_msg)

                bboxes_msg = self._generate_bounding_box_msg(bboxes)
                self._bboxes_publisher.publish(bboxes_msg)

            r.sleep()

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   Callback
    #
    # ------------------------------------------------------------------------------------------------------------------
    def _image_callback(self, msg: CompressedImage):
        with self._lock:
            self._subscribed_msg = msg

    def _action_call_back(self, goal: DetectionGoal):
        with _TimeLogger("Action Callback"):
            image = self._bridge.compressed_imgmsg_to_cv2(goal.image, self._encoding)
            bboxes = self._predict_image(image)
            if not self._server.is_preempt_requested():
                result = DetectionResult()
                result.boxes = self._generate_bounding_box_msg(bboxes)
                self._server.set_succeeded(result)

    def _service_callback(self, request: DetectionRequest):
        with _TimeLogger("Service Callback"):
            image = self._bridge.compressed_imgmsg_to_cv2(request.image, self._encoding)
            bboxes = self._predict_image(image)
            bboxes_msg = self._generate_bounding_box_msg(bboxes)
            return DetectionResponse(bboxes_msg)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   Private Method
    #
    # ------------------------------------------------------------------------------------------------------------------
    def _predict_image(self, image: NDArray[np.uint8]):
        tensor = torch.from_numpy(image).to(torch.float16).to(self._device) / 255.0
        tensor = tensor.permute(2, 0, 1)[None, ...]
        result = self._model(tensor)[0]
        # [B, N, 6]
        result = non_max_suppression(result, conf_thres=self._score_threshold, iou_thres=self._iou_threshold)[0]
        bboxes = []
        for detection in result.detach().cpu().numpy()[::-1]:
            x0, y0, x1, y1 = detection[:4].round().astype(np.int32).tolist()
            confidence = detection[4].item()
            class_id = int(detection[5].item())
            bboxes.append((x0, y0, x1, y1, confidence, class_id))

        # [K, [6]]
        return bboxes

    # ------------------------------------------------------------------------------------------------------------------
    #
    #   Instance Method (Private)
    #
    # ------------------------------------------------------------------------------------------------------------------
    def _generate_result_images(self, image: NDArray[np.uint8], bboxes: list[tuple[int, int, int, int, float, int]]):
        output_image = image.copy()
        text_config = {"fontFace": cv2.FONT_HERSHEY_DUPLEX, "fontScale": 0.6, "thickness": 1}
        for bbox in bboxes:
            x0, y0, x1, y1, confidence, class_id = bbox
            color = colors(class_id, bgr=True)
            cv2.rectangle(output_image, (x0, y0), (x1, y1), color, thickness=2)
            label_name = self._classes[class_id]
            label_str = f"{label_name} {confidence * 100:.1f}"
            size, _ = cv2.getTextSize(text=label_str, **text_config)
            cv2.rectangle(output_image, (x0, y0), (x0 + size[0], y0 + size[1]), (255, 255, 255), cv2.FILLED)
            cv2.putText(output_image, org=(x0, y0 + size[1]), color=(255, 0, 0), text=label_str, **text_config)

        return cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)

    def _generate_bounding_box_msg(self, bboxes: list[tuple[int, int, int, int, float, int]]) -> BoundingBoxes:
        msg = BoundingBoxes()
        msg.header.stamp = rospy.Time.now()
        for i, bounding_box in enumerate(bboxes):
            x0, y0, x1, y1, confidence, class_id = bounding_box
            bbox_msg = BoundingBox(self._classes[class_id], i, confidence, x0, y0, x1, y1)
            msg.boxes.append(bbox_msg)

        return msg


if __name__ == "__main__":
    rospy.init_node("yolov5_ros")
    node = YoloV5Node()
    node.main()
