#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import os
import time
from datetime import datetime

class KerasImageClassifier(Node):
    def __init__(self):
        super().__init__('keras_detector')

        # Paths relativos al script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, "../models/keras_model.h5")
        labels_path = os.path.join(script_dir, "../models/labels.txt")

        # Modelo y etiquetas
        self.model = load_model(model_path)
        self.labels = self.load_labels(labels_path)
        self.input_shape = self.model.input_shape[1:3]

        # Imagen
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/predicted_class', 10)

        # Capturas
        self.capture_enabled = True
        self.capture_dir = os.path.expanduser("~/TFG_2025/ROS2/ros2_ws/src/limo_vision/rUBot_captures")
        os.makedirs(self.capture_dir, exist_ok=True)
        self.create_class_dirs()
        self.last_capture_time = time.time()
        self.capture_interval = 1.0

        # Sub para activar/desactivar captura
        self.create_subscription(Bool, '/capture_toggle', self.toggle_callback, 10)

        self.get_logger().info("Nodo keras_detector en ROS 2 activo")

    def load_labels(self, path):
        with open(path, 'r') as f:
            lines = f.readlines()
            return [line.strip().split(' ', 1)[1] for line in lines]

    def create_class_dirs(self):
        for class_name in self.labels:
            class_path = os.path.join(self.capture_dir, class_name)
            os.makedirs(class_path, exist_ok=True)

    def toggle_callback(self, msg):
        self.capture_enabled = msg.data
        state = "ACTIVADA" if self.capture_enabled else "DESACTIVADA"
        self.get_logger().info(f"Captura automática {state}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            resized = cv2.resize(cv_image, self.input_shape)
            img = resized.astype(np.float32) / 255.0
            img = np.expand_dims(img, axis=0)

            predictions = self.model.predict(img)
            class_index = np.argmax(predictions)
            class_name = self.labels[class_index]

            self.get_logger().info(f"Detectado: {class_name}")
            self.publisher.publish(String(data=class_name))

            if self.capture_enabled:
                current_time = time.time()
                if current_time - self.last_capture_time >= self.capture_interval:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"{class_name}_{timestamp}.jpg"
                    filepath = os.path.join(self.capture_dir, class_name, filename)
                    cv2.imwrite(filepath, cv_image)
                    self.get_logger().info(f"Imagen guardada: {filepath}")
                    self.last_capture_time = current_time

        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KerasImageClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
