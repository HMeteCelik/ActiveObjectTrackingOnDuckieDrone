#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time
import math
from collections import deque

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)
        
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.bridge = CvBridge()
        self.target_detected = False
        self.target_center_x = 0
        self.target_center_y = 0
        self.image_width = 640
        self.image_height = 480
        
        self.initial_position_set = False
        self.initial_x = 0.0
        self.initial_y = 0.0


        self.velocity_history = deque(maxlen=10)
        self.last_detection_time = time.time()
        self.momentum_mode = False
        self.momentum_start_time = 0
        self.momentum_duration = 5.0
        self.momentum_velocity_x = 0.0
        self.momentum_velocity_y = 0.0

        try:
            self.model = YOLO('best.pt')
            rospy.loginfo("YOLO model loaded successfully")
        except Exception as e:
            rospy.logerr("Failed to load YOLO model: %s", str(e))
        
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/bottom_camera/image_raw', Image, self.image_callback)

        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.pose_goal_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.kp_x = 0.01
        self.kp_y = 0.01
        self.target_altitude = 20
        self.detection_threshold = 0.8
        self.max_move_distance = 5.0

    def state_callback(self, state):
        self.current_state = state

    def pose_callback(self, pose):
        self.current_pose = pose

        if not self.initial_position_set and self.current_state.mode == "GUIDED":
            self.initial_x = pose.pose.position.x
            self.initial_y = pose.pose.position.y
            self.initial_position_set = True
            rospy.loginfo(f"Initial position set: x={self.initial_x:.2f}, y={self.initial_y:.2f}")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_height, self.image_width, _ = cv_image.shape
            
            if not hasattr(self, 'model') or self.model is None:
                rospy.logwarn("YOLO model not loaded")
                self.target_detected = False
                self.draw_crosshair(cv_image)
                cv2.imshow("Object Detection", cv_image)
                cv2.waitKey(1)
                return
            
            results = self.model(cv_image, verbose=False)
            self.target_detected = False
            
            for r in results:
                if r.boxes and len(r.boxes) > 0:
                    for box in r.boxes:
                        confidence = box.conf.item()
                        if confidence > self.detection_threshold:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            self.target_center_x = center_x
                            self.target_center_y = center_y
                            self.target_detected = True
                            self.last_detection_time = time.time()
                            
                            if self.momentum_mode:
                                self.momentum_mode = False
                                rospy.loginfo("Target reacquired! Exiting momentum mode.")
                            
                            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                            cv2.putText(cv_image, f'Conf: {confidence:.2f}',
                                        (int(x1), int(y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            break
                if self.target_detected:
                    break

            self.draw_crosshair(cv_image)
            cv2.imshow("Object Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error in image processing: %s", str(e))

    def draw_crosshair(self, image):
        center_x = self.image_width // 2
        center_y = self.image_height // 2
        cv2.line(image, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 2)
        cv2.line(image, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 2)

    def calculate_average_velocity(self):
        if len(self.velocity_history) < 2:
            return 0.0, 0.0
        
        recent_velocities = list(self.velocity_history)[-5:]
        avg_vel_x = sum(vel[0] for vel in recent_velocities) / len(recent_velocities)
        avg_vel_y = sum(vel[1] for vel in recent_velocities) / len(recent_velocities)
        
        return avg_vel_x, avg_vel_y

    def calculate_movement(self):
        current_time = time.time()
        
        if not self.target_detected and (current_time - self.last_detection_time) > 1.0 and not self.momentum_mode:
            self.momentum_mode = True
            self.momentum_start_time = current_time
            self.momentum_velocity_x, self.momentum_velocity_y = self.calculate_average_velocity()
            
            rospy.loginfo("No detection for 1 second. Entering momentum mode...")
            rospy.loginfo(f"Momentum velocity: X={self.momentum_velocity_x:.3f}, Y={self.momentum_velocity_y:.3f}")
        
        if self.momentum_mode:
            if (current_time - self.momentum_start_time) > self.momentum_duration:
                self.momentum_mode = False
                rospy.loginfo("Momentum mode completed. Stopping movement...")
                if not self.target_detected:
                    return 0.0, 0.0
            else:
                move_x = self.momentum_velocity_x * 2
                move_y = self.momentum_velocity_y * 2
                
                move_x = max(-self.max_move_distance, min(self.max_move_distance, move_x))
                move_y = max(-self.max_move_distance, min(self.max_move_distance, move_y))
                
                rospy.loginfo(f"[MOMENTUM MODE] Moving with velocity: ({move_x:.3f}, {move_y:.3f})")
                return move_x, move_y
        
        elif not self.target_detected:
            return 0.0, 0.0
        
        error_x = self.target_center_x - (self.image_width / 2)
        error_y = self.target_center_y - (self.image_height / 2)
        
        move_x = self.kp_x * error_x
        move_y = self.kp_y * error_y
        
        move_x = max(-self.max_move_distance, min(self.max_move_distance, move_x))
        move_y = -1 * max(-self.max_move_distance, min(self.max_move_distance, move_y))
        
        self.velocity_history.append((move_x, move_y))
        
        rospy.loginfo(f"[TRACKING] Target: ({self.target_center_x}, {self.target_center_y}), "
                    f"Error: ({error_x:.1f}, {error_y:.1f}), "
                    f"Move: ({move_x:.3f}, {move_y:.3f})")
        
        return move_x, move_y

    def publish_position_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header = Header()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        
        yaw_angle = math.pi / 2
        qz = math.sin(yaw_angle / 2)
        qw = math.cos(yaw_angle / 2)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        
        self.pose_goal_pub.publish(goal)

    def track_target(self):
        rate = rospy.Rate(10)
        
        while not self.initial_position_set and not rospy.is_shutdown():
            rospy.loginfo("Waiting for initial position...")
            rate.sleep()
        
        rospy.loginfo("Starting target tracking...")
        
        while not rospy.is_shutdown():
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y
            
            move_x, move_y = self.calculate_movement()
            target_x = current_x + move_x
            target_y = current_y + move_y
            
            target_z = self.target_altitude
            if self.momentum_mode:
                target_z = self.target_altitude + 10
                
            self.publish_position_goal(target_x, target_y, target_z)
            rate.sleep()

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.set_mode('GUIDED')
            time.sleep(2)
            response = self.arming_client(True)
            if response.success:
                rospy.loginfo("Drone armed successfully.")
            else:
                rospy.logerr("Failed to arm drone: %s", response)
        except rospy.ServiceException as e:
            rospy.logerr("Arming service call failed: %s", e)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            response = self.set_mode_client(0, mode)
            if response.mode_sent:
                rospy.loginfo("Mode set to %s successfully.", mode)
            else:
                rospy.logerr("Failed to set mode to %s: %s", mode, response)
        except rospy.ServiceException as e:
            rospy.logerr("Set mode service call failed: %s", e)

    def takeoff(self):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_client(altitude=self.target_altitude)
            rospy.loginfo("Takeoff command sent. Response: %s", response)
        except rospy.ServiceException as e:
            rospy.logerr("Takeoff service call failed: %s", e)

    def wait_for_altitude(self, target_alt, tolerance=1.0):
        rate = rospy.Rate(5)
        start_time = time.time()
        timeout = 30
        
        while not rospy.is_shutdown():
            current_alt = self.current_pose.pose.position.z
            if abs(current_alt - target_alt) < tolerance:
                rospy.loginfo("Target altitude %.2f m reached (current: %.2f m).", target_alt, current_alt)
                break
            
            if time.time() - start_time > timeout:
                rospy.logwarn("Timeout waiting for altitude. Current: %.2f m, Target: %.2f m", 
                             current_alt, target_alt)
                break
                
            rospy.loginfo("Current altitude: %.2f m, Target: %.2f m", current_alt, target_alt)
            rate.sleep()

    def land(self):
        rospy.loginfo("Landing...")
        self.set_mode('LAND')

if __name__ == "__main__":
    controller = None
    try:
        controller = DroneController()
        
        rospy.loginfo("Starting drone operations...")
        controller.arm()
        time.sleep(3)
        
        controller.takeoff()
        controller.wait_for_altitude(controller.target_altitude)
        
        rospy.loginfo("Starting target tracking mode...")
        controller.track_target()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        if controller:
            rospy.loginfo("Cleaning up...")
            cv2.destroyAllWindows()
            
            if (controller.current_state.armed and 
                controller.current_pose.pose.position.z > 1.0):
                rospy.logwarn("Drone still armed and airborne. Landing...")
                controller.land()
                time.sleep(10)
