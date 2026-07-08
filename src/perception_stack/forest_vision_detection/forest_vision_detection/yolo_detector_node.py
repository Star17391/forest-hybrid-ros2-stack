#!/usr/bin/env python3
"""YOLO inference node for forest object detection.

Loads a trained ONNX or PyTorch YOLO model and publishes Detection2DArray
on /perception/camera/detections (param 'detections_topic').

Each detection carries:
  - 2D bounding box (centre + size, image pixels)
  - class_id and class probability vector
  - source_frame = camera_front_optical

The downstream probabilistic landmark node (forest_tree_slam / costmap)
subscribes here to fuse camera evidence with LiDAR landmarks.

Usage (after training):
  ros2 run forest_vision_detection yolo_detector \
    --ros-args -p model_path:=/path/to/best.pt
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from forest_vision_detection.geometry import CLASS_NAMES

# Cor por classe (BGR, p/ cv2). tree=verde, rock=vermelho, bush=amarelo, fallen_log=laranja.
CLASS_COLORS_BGR = {
    0: (0, 255, 0),
    1: (0, 0, 255),
    2: (0, 255, 255),
    3: (0, 165, 255),
}


def draw_detection_overlay(bgr, boxes):
    """Desenha as caixas (cls_id, conf, x1,y1,x2,y2) sobre uma cópia da imagem BGR.

    Função PURA (só cv2/numpy, sem ROS) — núcleo testável do overlay. Cor por
    classe + etiqueta `nome conf`. Devolve a imagem anotada (não muta a original).
    """
    import cv2

    out = bgr.copy()
    for cls_id, conf, x1, y1, x2, y2 in boxes:
        color = CLASS_COLORS_BGR.get(int(cls_id), (255, 255, 255))
        p1 = (int(round(x1)), int(round(y1)))
        p2 = (int(round(x2)), int(round(y2)))
        cv2.rectangle(out, p1, p2, color, 2)
        name = CLASS_NAMES[cls_id] if 0 <= cls_id < len(CLASS_NAMES) else str(cls_id)
        label = f"{name} {conf:.2f}"
        cv2.putText(out, label, (p1[0], max(12, p1[1] - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out


def imgmsg_from_bgr(bgr, header):
    """Converte uma imagem BGR (numpy HxWx3) numa sensor_msgs/Image (sem cv_bridge)."""
    msg = Image()
    msg.header = header
    msg.height = int(bgr.shape[0])
    msg.width = int(bgr.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(bgr.shape[1]) * 3
    msg.data = bgr.tobytes()
    return msg


def build_detection_array(boxes, header):
    """Constrói uma Detection2DArray a partir de caixas YOLO já decodificadas.

    `boxes`: iterável de (cls_id:int, conf:float, x1, y1, x2, y2) em pixels.
    Função PURA (sem modelo nem ROS) — é o núcleo testável da F1. As 4 classes do
    detetor passam HONESTAS como string (class_id); a taxonomia 4→3
    (bush/fallen_log → obstáculo) faz-se na fusão, no SLAM.
    """
    out = Detection2DArray()
    out.header = header
    for cls_id, conf, x1, y1, x2, y2 in boxes:
        det = Detection2D()
        det.header = header
        det.bbox = BoundingBox2D()
        det.bbox.center.position.x = 0.5 * (x1 + x2)
        det.bbox.center.position.y = 0.5 * (y1 + y2)
        det.bbox.center.theta = 0.0
        det.bbox.size_x = float(x2 - x1)
        det.bbox.size_y = float(y2 - y1)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = (
            CLASS_NAMES[cls_id] if 0 <= cls_id < len(CLASS_NAMES) else str(cls_id)
        )
        hyp.hypothesis.score = float(conf)
        det.results.append(hyp)
        out.detections.append(det)
    return out


class YoloDetectorNode(Node):
    """
    Placeholder — replace body once ultralytics / ONNX runtime is installed
    and a trained model is available.

    Install: pip install ultralytics
    Train:   cd forest-vision-training && python train.py
    Export:  yolo export model=runs/detect/forest/weights/best.pt format=onnx
    """

    def __init__(self) -> None:
        super().__init__("yolo_detector")

        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/perception/camera/detections")
        self.declare_parameter("overlay_topic", "/perception/camera/detections_image")
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_range_m", 18.0)

        # Publishers criados SEMPRE (mesmo sem modelo): a jusante pode subscrever e
        # ficar sem mensagens — não rebenta.
        det_topic = self.get_parameter("detections_topic").value
        self._det_pub = self.create_publisher(Detection2DArray, det_topic, 10)
        self._publish_overlay = bool(self.get_parameter("publish_overlay").value)
        self._overlay_pub = self.create_publisher(
            Image, self.get_parameter("overlay_topic").value, 5)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            self.get_logger().warn(
                "Parameter 'model_path' not set — node idle. "
                "Train a model with forest-vision-training/train.py first."
            )
            return

        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError:
            self.get_logger().error(
                "ultralytics not installed. Run: pip install ultralytics"
            )
            return

        self._model = YOLO(model_path)
        self._conf = float(self.get_parameter("conf_threshold").value)
        self._iou = float(self.get_parameter("iou_threshold").value)

        img_topic = self.get_parameter("image_topic").value
        # Câmaras/sensores publicam BEST_EFFORT — subscrever com sensor data QoS,
        # senão o RViz/este nó não recebem (RELIABILITY incompatível).
        self.create_subscription(
            Image, img_topic, self._on_image, qos_profile_sensor_data)

        self.get_logger().info(f"YOLO detector ready  model={model_path}")

    def _on_image(self, msg: Image) -> None:
        import numpy as np

        try:
            import cv2
            np_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding in ("rgb8",):
                bgr = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
            else:
                bgr = np_img
        except Exception as exc:
            self.get_logger().error(f"Image decode failed: {exc}")
            return

        results = self._model.predict(
            bgr, conf=self._conf, iou=self._iou, verbose=False
        )

        # Decodifica as caixas e constrói a mensagem com a função pura. Propaga o
        # header da IMAGEM (stamp + frame óptico): a jusante a fusão sincroniza por
        # stamp e projeta no frame da câmara (camera_front_optical).
        boxes = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                boxes.append((int(box.cls[0]), float(box.conf[0]), x1, y1, x2, y2))

        out = build_detection_array(boxes, msg.header)
        self._det_pub.publish(out)

        # Overlay para o RViz: desenha as caixas sobre a imagem e republica.
        if self._publish_overlay:
            annotated = draw_detection_overlay(bgr, boxes)
            self._overlay_pub.publish(imgmsg_from_bgr(annotated, msg.header))

        if boxes:
            self.get_logger().debug(
                f"publicadas {len(boxes)} deteções", throttle_duration_sec=1.0
            )


def main() -> None:
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
