#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from front_tracking_interfaces.msg import PersonTrackArray, PersonTrack
from geometry_msgs.msg import Point
import math

import pyrealsense2 as rs
import numpy as np
import cv2
import random
from ultralytics import YOLO

import sys
import os

sys.path.append(
    os.path.join(
        os.path.dirname(__file__),
        '..',  # front_tracking_node
        '3rdparty', 'OC_SORT'
    )
)
# OC_SORT 모듈 경로 설정
from trackers.ocsort_tracker.ocsort import OCSort



class FrontTrackingNode(Node):
    def __init__(self):
        super().__init__('front_tracking_node')

        self.debug_view = False
        DESIRED_SERIAL = '241122305809'

        self.publisher = self.create_publisher(PersonTrackArray, 'person_tracks', 10)

        self.model = YOLO("yolov8n.pt")
        self.conf = 0.3
        self.target_classes = ["person"]

        self.tracker = OCSort(
            det_thresh=self.conf,
            iou_threshold = 0.5,
            max_age = 5,
            min_hits=3,
            use_byte=False
        )
        self.id_colors = {}

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(DESIRED_SERIAL)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.timer = self.create_timer(0.05, self.process_frame)  # 20Hz

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            self.get_logger().warning("One of the frames did not come")
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        frame = color_image.copy()
        img_width = color_image.shape[1]
        fov = 87

        results = self.model.predict(color_image, conf=self.conf, imgsz=320)
        detections_yolo = []
        mask_only = []

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                if class_name != "person":
                    continue
                xyxy = box.xyxy[0].cpu().numpy() if hasattr(box.xyxy[0], 'cpu') else box.xyxy[0]
                conf_score = float(box.conf[0])
                detections_yolo.append([xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf_score])
        detections_np = np.array(detections_yolo)
        
        if detections_np is None or len(detections_np) == 0:
            return
            
        img_height, img_width = frame.shape[:2]
        img_info = (img_height, img_width)
        img_size = (480,640)


        tracks = self.tracker.update(detections_np, img_info, img_size)
        person_array_msg = PersonTrackArray()

        for track in tracks:
            x1,y1,x2,y2,track_id = track
            cx = int((x1+x2)/2)
            cy = int((y1+y2)/2)
            
            depth_val = depth_image[cy,cx]
            r_meters = round(depth_val / 1000.0, 2)
            theta_deg = ((cx-img_width / 2) / (img_width / 2)) * (fov / 2)
            x = r_meters * math.cos(math.radians(theta_deg))
            y = r_meters * math.sin(math.radians(theta_deg))
            z = 0
        
            track_msg = PersonTrack()
            track_msg.id = int(track_id)
            track_msg.x = float(x)
            track_msg.y = float(y)
            track_msg.z = float(z)
            track_msg.vx = 0.0
            track_msg.vy = 0.0
            track_msg.vz = 0.0
            person_array_msg.tracks.append(track_msg)
            
            # Debug view
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

        # Publish all tracks
        self.publisher.publish(person_array_msg)

        if self.debug_view:
            cv2.imshow("Front Tracking View", frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FrontTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
