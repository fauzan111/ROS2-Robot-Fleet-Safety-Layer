"""Person detector: YOLOv4-tiny (COCO) through OpenCV's DNN module.

Chosen because it runs on the OpenCV already in the ROS image (no PyTorch or ONNX Runtime),
takes about 10 ms per 320x240 frame on a laptop CPU at 416 px input, and found a person in
17/18 recorded simulation frames, including legs-only views from the 12 cm-high camera.
Weights: AlexeyAB/darknet release darknet_yolo_v4_pre (pinned by checksum in the Dockerfile).
"""

from __future__ import annotations

import numpy as np

from fleet_ml.perception import Detection

PERSON = 0  # COCO class id


class PersonDetector:
    def __init__(self, cfg: str, weights: str, input_size: int = 416,
                 conf_threshold: float = 0.3, nms_threshold: float = 0.4,
                 threads: int | None = None) -> None:
        import cv2  # imported here so the rest of fleet_ml works without OpenCV

        if threads:
            # Process-wide. Each robot's monitor is its own process; without a cap, every
            # detector grabs all cores and three of them thrash (850% CPU in the first run).
            cv2.setNumThreads(threads)

        net = cv2.dnn.readNetFromDarknet(cfg, weights)
        self.model = cv2.dnn_DetectionModel(net)
        self.model.setInputParams(scale=1 / 255.0, size=(input_size, input_size), swapRB=True)
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold

    def __call__(self, bgr: np.ndarray) -> list[Detection]:
        classes, scores, boxes = self.model.detect(
            bgr, confThreshold=self.conf_threshold, nmsThreshold=self.nms_threshold)
        return [
            Detection(int(x), int(y), int(w), int(h), float(s))
            for c, s, (x, y, w, h) in zip(np.array(classes).flatten(),
                                          np.array(scores).flatten(), boxes, strict=True)
            if int(c) == PERSON
        ]
