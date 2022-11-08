#!/bin/bash

cd $(dirname $0);

wget -q https://github.com/ultralytics/yolov5/releases/download/v6.2/yolov5m.pt
wget -q https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5m_Objects365.pt
