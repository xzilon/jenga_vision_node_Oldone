import yaml
import os
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        config_path = os.path.join(get_package_share_directory('jenga_vision'), 'config', 'C270_Calibration.yaml')
        with open(config_path, 'r') as f:
            calib_data = yaml.safe_load(f)
            
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
        
        self.raw_publisher_ = self.create_publisher(Image, 'videoframe', 10)
        self.publisher_ = self.create_publisher(CompressedImage,'videoframe/compressed',10)
        self.mask_publisher_ = self.create_publisher(CompressedImage, 'mask/compressed', 10)
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)
        
        #hsv parameters blue
        self.declare_parameter('h_min', 100)
        self.declare_parameter('h_max', 130)
        self.declare_parameter('s_min', 100)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 50)
        self.declare_parameter('v_max', 255)
        
        #hsv parameters red
        self.declare_parameter('r_h1_min', 0)
        self.declare_parameter('r_h1_max', 10)
        self.declare_parameter('r_h2_min', 170)
        self.declare_parameter('r_h2_max', 180)
        self.declare_parameter('r_s_min', 100)
        self.declare_parameter('r_s_max', 255)
        self.declare_parameter('r_v_min', 50)
        self.declare_parameter('r_v_max', 255)
        
        self.declare_parameter('color_mode', 'blue')
        #Blue range
        self.lower_blue = np.array([100, 100, 50])
        self.upper_blue = np.array([130, 255, 255])
        #Red range
        self.lower_red1 = np.array([0, 100, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 50])
        self.upper_red2 = np.array([180, 255, 255])
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        camera_matrix_data = calib_data['camera_matrix']['data']
        self.camera_matrix = np.array(camera_matrix_data).reshape(3, 3)
        self.dist_coeffs = np.array(calib_data['distortion_coefficients']['data'])
        self.focal_length_px = camera_matrix_data[0]
        self.known_width_mm = 15
        self.know_height_mm = 75
        half_w = self.known_width_mm/2
        half_h = self.know_height_mm/2
        
        self.filtered_x = None
        self.filtered_y = None
        self.filtered_z = None
        self.alpha = 0.7
        self.object_points = np.array([[-half_w,  -half_h, 0],
                                       [ half_w,  -half_h, 0],
                                       [ half_w, half_h, 0],
                                       [-half_w, half_h, 0],]
                                       ,dtype=np.float32)
        
        color_mode = self.get_parameter('color_mode').get_parameter_value().string_value
        self.get_logger().info('cv publisher has started ...')
    
    def frame_to_imgmsg(self, frame):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'  
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        return msg
        
    def frame_to_compressed_imgmsg(self, frame):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.format = 'jpeg' 
        success, encoded = cv2.imencode('.jpg',frame)
        if success:
            frame_byte = encoded.tobytes()
            msg.data = frame_byte
        else:
            self.get_logger().error('failed to encoded frame...')
            return None
        return msg
    
    def jenga_detection(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_mode = self.get_parameter('color_mode').get_parameter_value().string_value
        
        # mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        if color_mode == 'red':
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:  
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None, None, closing, None
                
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        epsilon = 0.02*(cv2.arcLength(largest_contour, True))
        approx_corners = cv2.approxPolyDP(largest_contour, epsilon, True)
        conner_count = len(approx_corners)
        self.get_logger().info(f'Conner founded: {conner_count}')
        if conner_count != 4 :
            return x, y, w, h, closing, None
        ordered_corners = self.sorting_conner(approx_corners)
        
        return x, y, w, h, closing , ordered_corners
    
    def sorting_conner(self, corners):
        pts = corners.reshape(4, 2)
        # Top_L && Bottom_R 
        Sum = pts.sum(axis=1)
        Top_L = pts[np.argmin(Sum)]
        Bottom_R = pts[np.argmax(Sum)]
        
        # Top_R && Bottom_L
        diff = np.diff(pts, axis=1)
        Top_R = pts[np.argmin(diff)]
        Bottom_L = pts[np.argmax(diff)]
        
        return Top_L, Top_R, Bottom_R, Bottom_L
       
        
    
    #timer for detection
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return
        
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        Raw_msg = self.frame_to_imgmsg(undistorted_frame)
        
        if Raw_msg is not None:
            self.raw_publisher_.publish(Raw_msg)
        
        
        x, y, w, h, closing, ordered_corners = self.jenga_detection(undistorted_frame)
        mask_msg = self.frame_to_compressed_imgmsg(closing)
        if mask_msg is not None:
            self.mask_publisher_.publish(mask_msg)  
               
        if ordered_corners is not None:
            image_points = np.array(ordered_corners, dtype = np.float32)
            success, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE)
            if success :
                raw_x = tvec[0][0]
                raw_y = tvec[1][0]
                raw_z = tvec[2][0]
                
                if self.filtered_z is None:
                    self.filtered_x = raw_x
                    self.filtered_y = raw_y
                    self.filtered_z = raw_z
                else:
                    self.filtered_x = self.alpha * raw_x + (1.0 - self.alpha) * self.filtered_x
                    self.filtered_y = self.alpha * raw_y + (1.0 - self.alpha) * self.filtered_y
                    self.filtered_z = self.alpha * raw_z + (1.0 - self.alpha) * self.filtered_z
                
                SETPOINT_X = -5.9
                
                diff_x = self.filtered_x - SETPOINT_X
                self.get_logger().info(f'[Gripper Error] Diff_X: {diff_x:+.1f} mm')
                self.get_logger().info(f'[Target Pos] X: {self.filtered_x:.1f} mm | Y: {self.filtered_y:.1f} mm | Distance (Z): {self.filtered_z:.1f} mm')
                # self.get_logger().info(f'x(tvec0) = {tvec[0]}, y(tvec1) = {tvec[1]}, z(tvec2) = {tvec[2]}')
                
                # R, _ = cv2.Rodrigues(rvec)
                # nx = R[0, 2]
                # ny = R[1, 2]
                # nz = R[2, 2]
                # yaw_error = np.arctan2(nx,nz)
                # yaw_error_degree = np.degrees(yaw_error)
                # self.get_logger().info(f'Yaw error : {yaw_error_degree:.2f}')
                
                

        
        if x is None:
            self.get_logger().info('No blue obj in frame...')
        msg = self.frame_to_compressed_imgmsg(undistorted_frame)
        if msg is not None:
            self.publisher_.publish(msg)
        
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'h_min':
                self.lower_blue[0] = param.value
            elif param.name == 'h_max':
                self.upper_blue[0] = param.value
            elif param.name == 's_min':
                self.lower_blue[1] = param.value
            elif param.name == 's_max':
                self.upper_blue[1] = param.value
            elif param.name == 'v_min':
                self.lower_blue[2] = param.value
            elif param.name == 'v_max':
                self.upper_blue[2] = param.value

        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
