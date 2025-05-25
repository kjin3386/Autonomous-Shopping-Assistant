#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import math
import pyrealsense2 as rs
import numpy as np
import cv2
import random
from ultralytics import YOLO
import time

import sys
import os

sys.path.append(
    os.path.join(
        os.path.dirname(__file__),
        '..', '..', 'src', '3rdparty', 'OC_SORT'
    )
)
from trackers.ocsort_tracker.ocsort import OCSort


class FrontTrackingNode(Node):
    def __init__(self):
        super().__init__('front_tracking_node')

        self.debug_view = True
        self.cloud_pub = self.create_publisher(PointCloud2, 'dynamic_obstacles', 10)

        self.model = YOLO("yolov8n.pt")
        self.model.overrides['verbose'] = False
        self.conf = 0.4

        # person 클래스 인덱스
        self.person_class_idx = None
        for idx, name in self.model.names.items():
            if name == "person":
                self.person_class_idx = idx
                break

        self.tracker = OCSort(
            det_thresh=self.conf,
            iou_threshold=0.4,
            max_age=5,
            min_hits=2,
            use_byte=False
        )
        self.id_colors = {}

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # 카메라 파라미터
        self.img_width = 640
        self.img_height = 480
        self.fov = 87
        self.fov_half = self.fov / 2
        self.img_width_half = self.img_width / 2

        # YOLO 실행 주기 제어
        self.frame_count = 0
        self.yolo_interval = 5  # 매 5프레임마다 YOLO 실행
        self.last_detections = []  # YOLO 결과 저장
        self.last_time = time.time()

        self.timer = self.create_timer(0.05, self.process_frame)  # 20Hz

    def process_frame(self):
        start_time = time.time()
        self.frame_count += 1

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        detections_yolo = []

        # YOLO 실행 (n프레임마다)
        if self.frame_count % self.yolo_interval == 0:
            results = self.model.predict(color_image, conf=self.conf, imgsz=320, verbose=False)

            for result in results:
                if result.boxes is not None and len(result.boxes) > 0:
                    cls_ids = result.boxes.cls.cpu().numpy()
                    person_mask = cls_ids == self.person_class_idx

                    if np.any(person_mask):
                        boxes = result.boxes.xyxy[person_mask].cpu().numpy()
                        confs = result.boxes.conf[person_mask].cpu().numpy()

                        for box, conf in zip(boxes, confs):
                            detections_yolo.append([box[0], box[1], box[2], box[3], conf])

            self.last_detections = detections_yolo
        else:
            detections_yolo = self.last_detections

        # 검출 없음 → 빈 pointcloud 발행
        if not detections_yolo:
            self.publish_pointcloud([])
            return

        detections_np = np.array(detections_yolo)
        img_info = (self.img_height, self.img_width)
        img_size = (480, 640)

        tracks = self.tracker.update(detections_np, img_info, img_size)

        pointcloud_points = []
        valid_track_ids = []
        frame = None
        if self.debug_view:
            frame = color_image.copy()
        for track in tracks:
            x1, y1, x2, y2, track_id = track
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            if cx < 0 or cx >= self.img_width or cy < 0 or cy >= self.img_height:
                continue  # 인덱스 예외 방지

            depth_val = depth_image[cy, cx]
            r_meters = depth_val / 1000.0
            theta_deg = ((cx - self.img_width_half) / self.img_width_half) * self.fov_half

            x = r_meters * math.cos(math.radians(theta_deg))
            y = r_meters * math.sin(math.radians(theta_deg))
            z = 0.0

            pointcloud_points.append([float(x), float(y), float(z)])
            valid_track_ids.append(int(track_id))
            if self.debug_view:
                if track_id not in self.id_colors:
                    self.id_colors[track_id] = (
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255)
                    )
                color = self.id_colors[track_id]
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                label = f"ID:{track_id}, {r_meters:.2f}m"
                cv2.putText(frame, label, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.publish_pointcloud(pointcloud_points)

        processing_time = (time.time() - start_time) * 1000
        print(f"Detected: {len(detections_yolo)} | Tracked: {valid_track_ids} | {processing_time:.1f}ms")

        if self.frame_count % 50 == 0:
            current_time = time.time()
            fps = 50 / (current_time - self.last_time)
            self.last_time = current_time
            print(f"FPS: {fps:.1f}")

        if self.debug_view and frame is not None:
            cv2.imshow("Front Tracking View", frame)
            cv2.waitKey(1)

    def publish_pointcloud(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = pc2.create_cloud(header, fields, points)
        self.cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = FrontTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

