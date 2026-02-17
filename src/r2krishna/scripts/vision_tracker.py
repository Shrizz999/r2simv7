#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

# --- STATES ---
STATE_SEARCH = 0      
STATE_APPROACH = 1    
STATE_ALIGNING = 2    
STATE_CLIMBING = 3    
STATE_FINISHED = 4

class PrecisionLevelManager(Node):
    def __init__(self):
        super().__init__('precision_level_manager')
        self.get_logger().info('--- R2KRISHNA: V19.3 CLOCKWISE SEARCH LOCKED ---')

        # --- THE MASTER TOGGLES ---
        # 1. SEARCH DIRECTION: LOCKED to -1.0 for Clockwise Search.
        self.SEARCH_DIR = -1.0    
        
        # 2. STEERING POLARITY: Master switch for the "Repelling" fix.
        # Change to -1.0 only if it still pushes away from the block.
        self.POLARITY_SHIFTER = 1.0  

        self.MAX_ALLOWED_DISTANCE = 5.0
        self.linear_speed = 0.18
        self.search_speed = 0.22
        self.deadzone = 30
        self.max_angular_vel = 0.50
        self.smoothing = 0.60 
        self.prev_turn = 0.0

        self.objectives = [200, 400, 600]
        self.current_obj_idx = 0
        self.state = STATE_SEARCH
        
        self.br = CvBridge()
        self.block_x = 320
        self.valid_target_locked = False
        
        self.lidar_min_dist = 99.9
        self.lidar_error = 0.0
        self.current_pitch = 0.0
        self.align_stable_start = 0.0

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_front = self.create_publisher(Twist, '/cmd_vel_front', 10)

        self.create_timer(0.05, self.control_loop)
        cv2.namedWindow("Robot Vision", cv2.WINDOW_NORMAL)

    def get_hsv_range(self):
        obj = self.objectives[self.current_obj_idx]
        if obj == 200: 
            return np.array([35, 100, 50]), np.array([65, 255, 255])
        elif obj == 400: 
            return np.array([62, 100, 40]), np.array([75, 255, 180])
        elif obj == 600: 
            return np.array([25, 80, 80]), np.array([40, 255, 255])
        return np.array([0,0,0]), np.array([180,255,255])

    def imu_callback(self, msg):
        q = msg.orientation
        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.current_pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0
        ranges[ranges < 0.15] = 10.0 

        count = len(ranges)
        mid_idx = count // 2
        window = 45 
        front_ranges = ranges[mid_idx - window : mid_idx + window]
        
        if len(front_ranges) > 0:
            self.lidar_min_dist = np.min(front_ranges)
            local_mid = len(front_ranges) // 2
            left_sector = front_ranges[local_mid+5 : local_mid+20]
            right_sector = front_ranges[local_mid-20 : local_mid-5]
            self.lidar_error = np.mean(left_sector) - np.mean(right_sector)
        else:
            self.lidar_min_dist = 99.9
            self.lidar_error = 0.0

    def image_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower, upper = self.get_hsv_range()
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 600:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        self.block_x = int(M["m10"] / M["m00"])
                        self.valid_target_locked = True
                        cv2.circle(frame, (self.block_x, 240), 10, (0, 255, 0), -1)
                else: self.valid_target_locked = False
            else: self.valid_target_locked = False

            cv2.imshow("Robot Vision", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Vision Error: {e}")

    def drive(self, fwd, rot):
        # Master polarity logic
        rot = rot * self.POLARITY_SHIFTER

        rot = np.clip(rot, -self.max_angular_vel, self.max_angular_vel)
        actual_rot = (self.smoothing * self.prev_turn) + ((1 - self.smoothing) * rot)
        self.prev_turn = actual_rot
        
        cmd = Twist()
        cmd.linear.x = float(fwd)
        cmd.angular.z = float(actual_rot)
        self.pub_vel.publish(cmd)
        self.pub_front.publish(cmd if self.state == STATE_CLIMBING else Twist())

    def control_loop(self):
        if self.state == STATE_FINISHED:
            self.drive(0, 0); return

        if self.state == STATE_SEARCH:
            if self.valid_target_locked and self.lidar_min_dist < self.MAX_ALLOWED_DISTANCE:
                self.state = STATE_APPROACH
            else:
                self.drive(0.0, self.search_speed * self.SEARCH_DIR)

        elif self.state == STATE_APPROACH:
            if not self.valid_target_locked:
                self.state = STATE_SEARCH; return
            
            # --- VECTOR SNAP LOGIC ---
            # If target is left of center, turn CCW.
            if self.block_x > (320 + self.deadzone):
                turn_cmd = 0.4
            elif self.block_x < (320 - self.deadzone):
                turn_cmd = -0.4
            else:
                turn_cmd = 0.0
            
            fwd_speed = 0.0 if abs(turn_cmd) > 0.1 else self.linear_speed

            if self.lidar_min_dist < 0.70:
                self.state = STATE_ALIGNING
                self.align_stable_start = 0.0
            else:
                self.drive(fwd_speed, turn_cmd)

        elif self.state == STATE_ALIGNING:
            if self.lidar_error > 0.005: turn_cmd = 0.2
            elif self.lidar_error < -0.005: turn_cmd = -0.2
            else: turn_cmd = 0.0
                
            dist_remaining = self.lidar_min_dist - 0.18
            fwd_speed = 0.08 if dist_remaining > 0.10 else (0.04 if dist_remaining > 0.0 else 0.0)

            if abs(self.lidar_error) < 0.01 and self.lidar_min_dist < 0.22 and fwd_speed == 0.0:
                 if self.align_stable_start == 0.0: self.align_stable_start = time.time()
                 if (time.time() - self.align_stable_start) > 1.2:
                    self.state = STATE_CLIMBING
            else:
                 self.align_stable_start = 0.0
                 self.drive(fwd_speed, turn_cmd)

        elif self.state == STATE_CLIMBING:
            self.drive(1.2, 0.0)
            if abs(self.current_pitch) < 0.03 and self.lidar_min_dist > 2.0:
                self.current_obj_idx += 1
                self.state = STATE_FINISHED if self.current_obj_idx >= 3 else STATE_SEARCH

if __name__ == '__main__':
    rclpy.init()
    node = PrecisionLevelManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
