"""Testes da F1 da fusão câmara→LiDAR: construção da Detection2DArray.

Testa a função PURA `build_detection_array` (sem modelo YOLO nem ROS a correr),
que é o núcleo do `yolo_detector_node`: mapeia caixas YOLO (xyxy + classe + score)
para a mensagem que a fusão no SLAM vai consumir.
"""
import numpy as np
import pytest
from std_msgs.msg import Header

from forest_vision_detection.yolo_detector_node import (
    build_detection_array,
    draw_detection_overlay,
    imgmsg_from_bgr,
)


def test_maps_boxes_to_centre_size_and_class():
    header = Header()
    header.frame_id = "camera_front_optical"
    boxes = [
        (0, 0.90, 100.0, 50.0, 140.0, 250.0),   # tree
        (1, 0.70, 200.0, 200.0, 260.0, 240.0),  # rock
    ]
    arr = build_detection_array(boxes, header)

    assert arr.header.frame_id == "camera_front_optical"
    assert len(arr.detections) == 2

    d0 = arr.detections[0]
    # xyxy -> centro + tamanho
    assert d0.bbox.center.position.x == pytest.approx(120.0)  # (100+140)/2
    assert d0.bbox.center.position.y == pytest.approx(150.0)  # (50+250)/2
    assert d0.bbox.size_x == pytest.approx(40.0)
    assert d0.bbox.size_y == pytest.approx(200.0)
    # classe HONESTA (string), score propagado
    assert d0.results[0].hypothesis.class_id == "tree"
    assert d0.results[0].hypothesis.score == pytest.approx(0.90)
    assert arr.detections[1].results[0].hypothesis.class_id == "rock"
    # header propagado para cada deteção (frame óptico p/ a projeção a jusante)
    assert d0.header.frame_id == "camera_front_optical"


def test_keeps_four_classes_honest():
    # bush e fallen_log passam como estão (4→3 só na fusão, no SLAM).
    arr = build_detection_array(
        [(2, 0.5, 0, 0, 10, 10), (3, 0.5, 0, 0, 10, 10)], Header())
    assert arr.detections[0].results[0].hypothesis.class_id == "bush"
    assert arr.detections[1].results[0].hypothesis.class_id == "fallen_log"


def test_empty_input_gives_empty_array():
    arr = build_detection_array([], Header())
    assert len(arr.detections) == 0


def test_unknown_class_id_falls_back_to_string():
    arr = build_detection_array([(9, 0.5, 0, 0, 10, 10)], Header())
    assert arr.detections[0].results[0].hypothesis.class_id == "9"


# --- Overlay para o RViz ---------------------------------------------------

def test_overlay_preserves_shape_and_does_not_mutate_input():
    img = np.zeros((48, 64, 3), dtype=np.uint8)
    boxes = [(0, 0.9, 10, 10, 40, 40)]  # tree
    out = draw_detection_overlay(img, boxes)
    assert out.shape == img.shape
    assert out.dtype == np.uint8
    # desenhou alguma coisa (a imagem deixou de ser toda preta)
    assert out.any()
    # não mutou a original
    assert not img.any()


def test_overlay_empty_boxes_is_just_a_copy():
    img = np.full((20, 20, 3), 7, dtype=np.uint8)
    out = draw_detection_overlay(img, [])
    assert np.array_equal(out, img)


def test_imgmsg_encodes_bgr8_correctly():
    img = np.zeros((4, 5, 3), dtype=np.uint8)
    h = Header()
    h.frame_id = "camera_front_optical"
    msg = imgmsg_from_bgr(img, h)
    assert msg.height == 4
    assert msg.width == 5
    assert msg.encoding == "bgr8"
    assert msg.step == 15  # width * 3
    assert len(msg.data) == 4 * 5 * 3
    assert msg.header.frame_id == "camera_front_optical"
