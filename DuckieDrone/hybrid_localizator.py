#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Range
from raspicam_node.msg import MotionVectors
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from threading import Lock


class HybridLocalization:
    def __init__(self):
        rospy.init_node('hybrid_localization', anonymous=True)

        # Position and velocity
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
        self.position_lock = Lock()

        # Optical flow info
        self.optical_flow_velocity = np.array([0.0, 0.0])  # vx, vy
        self.optical_flow_error = np.array([0.0, 0.0])     # accumulated error
        self.last_optical_flow_time = None
        self.altitude = 0.0

        # AprilTag info
        self.tag_detected = False
        self.last_tag_time = None
        self.tag_timeout = 0.5  # 0.5 seconds timeout for tag detection

        # Weighting parameters (similar to Kalman filter)
        self.alpha_tag = 0.8    # tag data weight
        self.alpha_flow = 0.3   # optical flow weight
        self.flow_scale = 1.0   # optical flow scaling factor

        # Subscribers
        self._sub_tag = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,
                                         self.tag_callback, queue_size=1)
        self._sub_mv = rospy.Subscriber('/raspicam_node/motion_vectors', MotionVectors,
                                        self.motion_cb, queue_size=1)
        self._sub_alt = rospy.Subscriber('/pidrone/range', Range,
                                         self.altitude_cb, queue_size=1)

        # Publishers
        self.pub_pose = rospy.Publisher('/hybrid_localization/pose', PoseStamped, queue_size=1)
        self.pub_velocity = rospy.Publisher('/hybrid_localization/velocity', TwistStamped, queue_size=1)

        # Timer for continuous updates
        self.update_timer = rospy.Timer(rospy.Duration(0.02), self.update_callback)  # 50Hz

        rospy.loginfo("Hybrid Localization Node Started")

    def tag_callback(self, msg):
        """AprilTag detection callback"""
        current_time = rospy.Time.now()

        if len(msg.detections) == 0:
            return

        detection = msg.detections[0]
        tag_pose = detection.pose.pose.pose

        with self.position_lock:
            # Flip tag's X axis to match optical flow convention
            new_position = np.array([
                -tag_pose.position.x,
                tag_pose.position.y,
                tag_pose.position.z
            ])

            if self.last_tag_time is not None:
                dt = (current_time - self.last_tag_time).to_sec()
                if dt > 0:
                    tag_velocity = (new_position - self.position[:3]) / dt
                    self.velocity[:3] = (self.alpha_tag * tag_velocity +
                                         (1 - self.alpha_tag) * self.velocity[:3])

            self.position[:3] = new_position

            self.optical_flow_error = np.array([0.0, 0.0])

            self.tag_detected = True
            self.last_tag_time = current_time

            rospy.logdebug("Tag detected (x inverted): {}".format(new_position))

    def motion_cb(self, msg):
        """Optical flow motion vectors callback"""
        current_time = rospy.Time.now()

        if not msg.x or not msg.y or not msg.sad:
            return

        sad = np.array(msg.sad, dtype=np.float32)

        # Filter out low SAD values (likely noise)
        sad[sad < 5] = 0.0

        if np.sum(sad) == 0:
            return

        flow_x = np.average(msg.x, weights=sad)
        flow_y = np.average(msg.y, weights=sad)

        if self.last_optical_flow_time is not None:
            dt = (current_time - self.last_optical_flow_time).to_sec()
            if dt > 0:

                if self.altitude < 0.04:
                    self.optical_flow_velocity = np.array([0.0, 0.0])
                    self.last_optical_flow_time = current_time
                    return

                scale_factor = self.altitude * self.flow_scale if self.altitude > 0 else 1.0

                flow_velocity_x = flow_x * scale_factor
                flow_velocity_y = flow_y * scale_factor

                self.optical_flow_velocity = np.array([flow_velocity_x, flow_velocity_y])

                if not self.is_tag_valid():
                    with self.position_lock:
                        position_delta = self.optical_flow_velocity * dt
                        self.position[:2] += position_delta

                        self.velocity[:2] = (self.alpha_flow * self.optical_flow_velocity +
                                             (1 - self.alpha_flow) * self.velocity[:2])

                        self.optical_flow_error += position_delta

        self.last_optical_flow_time = current_time

    def altitude_cb(self, msg):
        """Altitude sensor callback"""
        self.altitude = msg.range

        with self.position_lock:
            self.position[2] = self.altitude

    def is_tag_valid(self):
        """Check whether the tag detection is still valid"""
        if not self.tag_detected or self.last_tag_time is None:
            return False

        current_time = rospy.Time.now()
        time_since_tag = (current_time - self.last_tag_time).to_sec()

        if time_since_tag > self.tag_timeout:
            self.tag_detected = False
            return False

        return True

    def update_callback(self, event):
        """Main update loop - runs at 50Hz"""
        current_time = rospy.Time.now()

        with self.position_lock:
            if not self.is_tag_valid() and self.position[2] < 0.04:
                self.position[0:2] = [0.0, 0.0]
                self.velocity[0:2] = [0.0, 0.0]
                self.optical_flow_error = np.array([0.0, 0.0])

            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "map"

            pose_msg.pose.position.x = self.position[0]
            pose_msg.pose.position.y = self.position[1]
            pose_msg.pose.position.z = self.position[2]

            pose_msg.pose.orientation.w = 1.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0

            vel_msg = TwistStamped()
            vel_msg.header.stamp = current_time
            vel_msg.header.frame_id = "map"

            vel_msg.twist.linear.x = self.velocity[0]
            vel_msg.twist.linear.y = self.velocity[1]
            vel_msg.twist.linear.z = self.velocity[2]

            self.pub_pose.publish(pose_msg)
            self.pub_velocity.publish(vel_msg)

    def get_position(self):
        """Public getter for current position"""
        with self.position_lock:
            return self.position.copy()

    def get_velocity(self):
        """Public getter for current velocity"""
        with self.position_lock:
            return self.velocity.copy()

    def reset_position(self):
        """Reset position and velocity"""
        with self.position_lock:
            self.position = np.array([0.0, 0.0, 0.0])
            self.velocity = np.array([0.0, 0.0, 0.0])
            self.optical_flow_error = np.array([0.0, 0.0])

        rospy.loginfo("Position reset")


if __name__ == '__main__':
    try:
        localization = HybridLocalization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
