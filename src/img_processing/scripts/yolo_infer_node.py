#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import openvino as ov
import threading
import time
import os

class YoloInferNode(Node):
    def __init__(self):
        super().__init__('yolo_infer_node')
        self.bridge = CvBridge()

        self.declare_parameter('video_path', '')
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        if not self.video_path or not os.path.exists(self.video_path):
            self.get_logger().error(f'Video file not found: {self.video_path}')
            raise FileNotFoundError(self.video_path)

        self.declare_parameter('model_path', 'model/0526.xml')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        core = ov.Core()
        model = core.read_model(model_path)
        self.compiled_model = core.compile_model(model, "GPU")
        self.infer_request = self.compiled_model.create_infer_request()
        self.get_logger().info('YOLO model loaded, warming up...')

        # 预热：使用 f16 数据，匹配模型输入
        dummy_input = np.random.uniform(0, 1, (1, 3, 640, 640)).astype(np.float16)
        self.infer_request.set_input_tensor(ov.Tensor(dummy_input))
        self.infer_request.infer()
        self.get_logger().info('Warmup complete!')

        self.conf_threshold = 0.65
        self.nms_threshold = 0.45
        self.target_color = 0   # 0=红, 1=蓝 (敌方颜色)

        self.pub = self.create_publisher(Image, 'yolo_result', 10)
        self.fps = 0.0
        self.inference_time = 0.0
        self._last_target_count = 0

        self.thread = threading.Thread(target=self.video_loop, daemon=True)
        self.thread.start()

    def sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-x))

    def video_loop(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            self.get_logger().error(f'Cannot open video: {self.video_path}')
            return

        fps_counter = 0
        fps_start = time.time()

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            t0 = time.time()
            result_img = self.process_frame(frame)
            t1 = time.time()
            self.inference_time = (t1 - t0) * 1000

            fps_counter += 1
            now = time.time()
            if now - fps_start >= 1.0:
                self.fps = fps_counter / (now - fps_start)
                fps_counter = 0
                fps_start = now

            # 显示 FPS + 推理耗时 + 目标数
            cv2.putText(result_img, f'FPS: {self.fps:.1f}  Infer: {self.inference_time:.1f}ms', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(result_img, f'Targets: {self._last_target_count}', (10, result_img.shape[0]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

            msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
            self.pub.publish(msg)

    def process_frame(self, img):
        h, w = img.shape[:2]
        scale_x, scale_y = w / 640.0, h / 640.0

        # 预处理：resize, BGR2RGB, 归一化 -> float16
        input_img = cv2.resize(img, (640, 640))
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        input_img = (input_img.astype(np.float16) / 255.0)   # 使用 float16
        input_img = np.transpose(input_img, (2, 0, 1))       # [3,640,640]
        input_img = np.expand_dims(input_img, axis=0)         # [1,3,640,640]

        # 重用 infer_request，设置 f16 张量
        self.infer_request.set_input_tensor(ov.Tensor(input_img))
        self.infer_request.infer()
        output_tensor = self.infer_request.get_output_tensor(0)
        output = output_tensor.data[0]   # [25200,22]

        boxes, confidences, detections = [], [], []
        for r in range(output.shape[0]):
            conf = self.sigmoid(output[r, 8])
            if conf < self.conf_threshold:
                continue
            color_id = np.argmax(output[r, 9:13])
            class_id = np.argmax(output[r, 13:22])
            if color_id != self.target_color:
                continue

            pts = []
            for i in range(4):
                kx = output[r, i*2] * scale_x
                ky = output[r, i*2+1] * scale_y
                pts.append([int(kx), int(ky)])
            x_min = min(p[0] for p in pts)
            y_min = min(p[1] for p in pts)
            x_max = max(p[0] for p in pts)
            y_max = max(p[1] for p in pts)

            boxes.append([x_min, y_min, x_max - x_min, y_max - y_min])
            confidences.append(conf)
            detections.append((class_id, color_id, conf, pts))

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)

        class_map = {0: 'G', 1: '1', 2: '2', 3: '3', 4: '4', 5: '5', 6: 'O', 7: 'Bs', 8: 'Bb'}
        for idx in indices:
            i = idx[0] if isinstance(idx, (list, tuple)) else idx
            cls_id, col_id, conf, pts = detections[i]
            pts_arr = np.array(pts, dtype=np.int32)
            cv2.polylines(img, [pts_arr], True, (0, 255, 0), 2)
            for p in pts:
                cv2.circle(img, p, 3, (0, 0, 255), -1)
            label = f'{class_map.get(cls_id, "?")} {conf:.2f}'
            cv2.putText(img, label, (pts[0][0], pts[0][1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        self._last_target_count = len(indices)
        return img

def main():
    rclpy.init()
    node = YoloInferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()