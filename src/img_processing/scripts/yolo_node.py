#!/usr/bin/env python3
"""
YOLO 推理节点 (Python + OpenVINO)
负责：
1. 订阅原始图像话题 `/image_raw`
2. 使用 OpenVINO 加载 IR 模型进行目标检测
3. 发布检测结果话题 `/yolo_detections` 和可视化图像话题 `/yolo_result`
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import openvino as ov

# 自定义消息（稍后编译生成）
from img_processing.msg import YoloDetection, YoloDetectionArray


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # 声明参数，可在 launch 文件中覆盖
        self.declare_parameter('model_path', '/home/cly/project/src/img_processing/model/yolo11.xml')
        self.declare_parameter('score_threshold', 0.7)
        self.declare_parameter('nms_threshold', 0.3)
        self.declare_parameter('input_size', 640)

        # 获取参数值
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.score_threshold = self.get_parameter('score_threshold').get_parameter_value().double_value
        self.nms_threshold = self.get_parameter('nms_threshold').get_parameter_value().double_value
        self.input_size = self.get_parameter('input_size').get_parameter_value().integer_value

        # ROS 通信
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.detection_pub = self.create_publisher(YoloDetectionArray, '/yolo_detections', 10)
        self.image_pub = self.create_publisher(Image, '/yolo_result', 10)

        # 加载 OpenVINO 模型
        self.core = ov.Core()
        self.model = self.core.read_model(model_path)
        self.compiled_model = self.core.compile_model(self.model, 'CPU')
        self.infer_request = self.compiled_model.create_infer_request()
        self.get_logger().info(f'YOLO 模型加载成功: {model_path}')

        # 输出形状信息
        output = self.compiled_model.output()
        self.output_shape = output.get_partial_shape()
        self.get_logger().info(f'输出形状: {self.output_shape}')

        # 固定类别数
        self.class_num = 38

    def image_callback(self, msg):
        """每收到一帧图像，执行检测并发布结果"""
        # 1. 将 ROS 图像消息转为 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 2. 预处理
        scale, padded_img = self.letterbox(frame)

        # 3. 推理
        detections = self.infer(padded_img, scale)

        # 4. 发布检测结果
        self.publish_detections(detections, msg.header)

        # 5. 发布可视化图像（用于调试显示）
        vis_img = self.visualize(frame, detections)
        self.publish_visualization(vis_img, msg.header)

    def letterbox(self, img):
        """等比例缩放并填充到 input_size x input_size，保持宽高比"""
        h, w = img.shape[:2]
        scale = min(self.input_size / w, self.input_size / h)
        new_w, new_h = int(w * scale), int(h * scale)

        # 创建黑色画布
        canvas = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
        # 将缩放后的图像放到画布上
        resized = cv2.resize(img, (new_w, new_h))
        canvas[0:new_h, 0:new_w] = resized
        return scale, canvas

    def infer(self, padded_img, scale):
        """执行推理并解析结果"""
        # 转为 RGB (同济 PPP 配置会将 BGR 转为 RGB 并归一化)
        rgb = cv2.cvtColor(padded_img, cv2.COLOR_BGR2RGB)
        # 归一化到 [0,1] 并转为 float32
        blob = rgb.astype(np.float32) / 255.0
        # 转换为 NCHW 格式
        blob = blob.transpose(2, 0, 1)[None]

        # 创建 OpenVINO Tensor 并设置输入
        input_tensor = ov.Tensor(blob)
        self.infer_request.set_input_tensor(input_tensor)

        # 执行推理
        self.infer_request.invoke()

        # 获取输出 tensor
        output_tensor = self.infer_request.get_output_tensor()
        output_data = output_tensor.data  # shape: (1, 50, 8400)

        # 解析输出
        # 输出形状: (1, 50, 8400)，转置为 (8400, 50)
        out = output_data[0].T

        boxes = []
        class_ids = []
        confidences = []
        keypoints = []

        for row in out:
            scores = row[4:4 + self.class_num]
            max_score = np.max(scores)
            if max_score < self.score_threshold:
                continue

            class_id = np.argmax(scores)
            cx, cy, bw, bh = row[0:4]

            # 还原到原图坐标
            left = int((cx - 0.5 * bw) / scale)
            top = int((cy - 0.5 * bh) / scale)
            width = int(bw / scale)
            height = int(bh / scale)

            # 提取 4 个关键点并转换为原图坐标
            kpts = []
            for i in range(4):
                kx = row[4 + self.class_num + i * 2] / scale
                ky = row[4 + self.class_num + i * 2 + 1] / scale
                kpts.append((kx, ky))

            # 按照同济代码排序：左上、右上、右下、左下
            sorted_kpts = self.sort_keypoints(kpts)

            boxes.append([left, top, width, height])
            class_ids.append(class_id)
            confidences.append(max_score)
            keypoints.append(sorted_kpts)

        # NMS 去重
        indices = []
        if boxes:
            bboxes_for_nms = [[b[0], b[1], b[0] + b[2], b[1] + b[3]] for b in boxes]
            indices = cv2.dnn.NMSBoxes(
                bboxes_for_nms, confidences, self.score_threshold, self.nms_threshold
            )

        detections = []
        for i in indices.flatten():
            detections.append({
                'box': boxes[i],
                'class_id': class_ids[i],
                'confidence': confidences[i],
                'keypoints': keypoints[i]
            })
        return detections

    def sort_keypoints(self, points):
        """对关键点排序：左上、右上、右下、左下（同济标准）"""
        # 先按 y 坐标排序，取上两个点
        sorted_by_y = sorted(points, key=lambda p: p[1])
        top_points = sorted(sorted_by_y[:2], key=lambda p: p[0])    # 上方两个按 x 排序
        bottom_points = sorted(sorted_by_y[2:], key=lambda p: p[0]) # 下方两个按 x 排序

        # 返回顺序：左上、右上、右下、左下
        return [top_points[0], top_points[1], bottom_points[1], bottom_points[0]]

    def publish_detections(self, detections, header):
        """发布检测结果到 /yolo_detections"""
        msg = YoloDetectionArray()
        msg.header = header
        for det in detections:
            d = YoloDetection()
            d.class_id = det['class_id']
            d.confidence = det['confidence']
            d.x = det['box'][0]
            d.y = det['box'][1]
            d.width = det['box'][2]
            d.height = det['box'][3]
            # 填充关键点 (4个点)
            for kp in det['keypoints']:
                p = geometry_msgs.msg.Point32()  # 将在消息定义中使用 Point32
                p.x = kp[0]
                p.y = kp[1]
                d.keypoints.append(p)
            msg.detections.append(d)
        self.detection_pub.publish(msg)

    def visualize(self, img, detections):
        """绘制检测结果并返回图像"""
        img = img.copy()
        for det in detections:
            x, y, w, h = det['box']
            class_id = det['class_id']
            conf = det['confidence']
            # 画矩形框
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # 显示类别和置信度
            cv2.putText(img, f'{class_id}: {conf:.2f}', (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # 画关键点
            for kp in det['keypoints']:
                cv2.circle(img, (int(kp[0]), int(kp[1])), 3, (0, 0, 255), -1)
        return img

    def publish_visualization(self, vis_img, header):
        """发布可视化图像"""
        msg = self.bridge.cv2_to_imgmsg(vis_img, 'bgr8')
        msg.header = header
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()