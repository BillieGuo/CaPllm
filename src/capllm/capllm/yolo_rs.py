import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLOWorld
import numpy as np
import cv2

VISUALIZE = False

class Yolo(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.publisher_str = self.create_publisher(
            String, 
            'yolo_detections_str', 
            10
            )
        self.publisher_rgb = self.create_publisher(
            Image,
            'yolo_detections_rgb',
            10
            )
        self.rs_rgb_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.rs_rgb_callback,
            10
            )
        self.rs_align_sub = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.rs_align_callback,
            10
            )
        self.llm_sub = self.create_subscription(
            String,
            "vision_llm",
            self.llm_callback,
            10
            )
        
        self.rgb_flag = False
        self.align_flag = False
        self.rgb_frame = None
        self.align_frame = None
        
        self.process_lock = False
        
        self.model = YOLOWorld("yolov8x-worldv2.pt")
        self.model.info()
        self.model.to("cuda")
        
        
    def run(self):
        
        while rclpy.ok():    
            rclpy.spin_once(self)
            try:
                if not self.rgb_frame or not self.align_frame:
                    continue
                self.process_lock = True
                # Convert images to numpy arrays
                rgb_img = np.frombuffer(self.rgb_frame.data, dtype=np.uint8).reshape(self.rgb_frame.height, self.rgb_frame.width, -1)
                align_img = np.frombuffer(self.align_frame.data, dtype=np.uint16).reshape(self.align_frame.height, self.align_frame.width)
                self.rgb_flag = False
                self.align_flag = False
                
                results = self.model.predict(rgb_img)
                # print(results)
                detect_obj_dict = {}
                for box in results[0].boxes:
                    center_x, center_y, w, h = box.xywh.tolist()[0]
                    # Get the depth value at the center of the bounding box
                    depth_value = align_img[int(center_y), int(center_x)]
                    detect_obj_dict[int(box.cls)] = {
                        "name": self.model.names[int(box.cls)],
                        "depth": depth_value
                    }
                self.detected_names = [obj["name"] for obj in detect_obj_dict.values()]
                detected_objects = []
                for box in results[0].boxes:
                    center_x, center_y, w, h = box.xywh.tolist()[0]
                    # Get the depth value at the center of the bounding box
                    depth_value = align_img[int(center_y), int(center_x)]
                    detected_objects.append({
                        "class_id": int(box.cls),
                        "name": self.model.names[int(box.cls)],
                        "depth": depth_value,
                        "bbox": [center_x, center_y, w, h]
                    })
                self.ordered_detected_objects = sorted(detected_objects, key=lambda x: x["bbox"][0])  # Sort by the x-coordinate of the bounding box center (left to right)
                annotated_frame = results[0].plot()
                self.annotated_frame = annotated_frame 
                self.publish_detections()
                
                if VISUALIZE:
                    for obj in self.ordered_detected_objects:
                        cv2.putText(annotated_frame, f"{obj['depth']}mm", (int(obj["bbox"][0]), int(obj["bbox"][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    # Display the annotated frame
                    cv2.imshow("YOLO Inference", annotated_frame)
                    cv2.waitKey(1)
                
                self.process_lock = False
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                self.get_logger().info("Exiting...")
                self.destroy_node()

    def rs_rgb_callback(self, msg):
        if self.process_lock:
            return
        self.rgb_flag = True
        self.rgb_frame = msg
        return
    
    def rs_align_callback(self, msg):
        if self.process_lock:
            return
        self.align_flag = True
        self.align_frame = msg
        return

    def llm_callback(self, msg):
        if msg.data == "oneshot":
            self.publish_detections()
            return
        return
            
    def publish_detections(self):
        send_str = String()
        send_str.data = str(self.detected_names)
        self.publisher_str.publish(send_str)
        return


def main(args=None):
    rclpy.init(args=args)
    yolo_publisher = Yolo()
    yolo_publisher.run()
    # rclpy.spin(yolo_publisher)
    yolo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()