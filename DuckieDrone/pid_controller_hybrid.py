#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import signal
import sys

import command_values as cmds
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
from pidrone_pkg.msg import Mode, RC, State
from std_msgs.msg import Empty, Bool
from three_dim_vec import Position, Velocity, Error, RPY

from pid_class import PID, PIDaxis


class PIDController(object):
    ''' Improved PID controller with smoother tag tracking and drift prevention '''

    def __init__(self):
        # Initialize the current and desired modes
        self.current_mode = Mode('DISARMED')
        self.desired_mode = Mode('DISARMED')

        # Always use position control for hovering
        self.position_control = True

        # Initialize positions - FIXED HOVER POSITION
        self.current_position = Position()
        self.desired_position = Position(x=0.0, y=0.0, z=0.5)

        # Initialize errors
        self.position_error = Error()
        self.velocity_error = Error()

        # Initialize velocities
        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()

        # Initialize the primary PID
        self.pid = PID()
        self.pid_error = Error()

        # # More conservative PID parameters for smoother control
        # self.lr_pid = PIDaxis(kp=2.5, ki=0, kd=0, midpoint=0, control_range=(-3.0, 3.0))
        # self.fb_pid = PIDaxis(kp=2.5, ki=0, kd=0, midpoint=0, control_range=(-3.0, 3.0))
        #
        # # Z-axis PID for altitude control - More aggressive for better altitude tracking
        # self.z_pid = PIDaxis(kp=15.0, ki=0, kd=0, midpoint=0, control_range=(-60.0, 60.0))

        # Additional smoothing layers
        self.ultra_smooth_x = 0.0
        self.ultra_smooth_y = 0.0
        self.velocity_smooth_x = 0.0
        self.velocity_smooth_y = 0.0

        # Initialize timing
        self.last_pose_time = None

        # Initialize RPY
        self.current_rpy = RPY()
        self.previous_rpy = RPY()

        # Initialize states
        self.current_state = State()
        self.previous_state = State()

        # Command publisher
        self.cmdpub = None
        self.position_control_pub = None

        # Safety and tracking parameters
        self.safety_threshold = 1.0
        self.lost = False
        self.lost_counter = 0
        self.tracking_confidence = 1.0

        # Tag detection history for stability
        self.tag_detection_history = []
        self.max_history_length = 10

        # Drift compensation
        self.last_known_good_position = Position(x=0.0, y=0.0, z=0.5)
        self.drift_compensation_active = False

        # Z-axis filtering
        self.z_filtered = 0.0

        # Data received flags
        self.pose_received = False
        self.velocity_received = False

    def current_state_callback(self, state):
        """ Store the drone's current state for calculations (only orientation now) """
        self.previous_state = self.current_state
        self.current_state = state
        self.update_orientation_from_state()

    def pose_callback(self, msg):
        """ Update position from hybrid localization pose topic """
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y
        self.current_position.z = msg.pose.position.z

        # Update orientation from pose message
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w)
        r, p, y = tf.transformations.euler_from_quaternion(quaternion)
        self.previous_rpy = self.current_rpy
        self.current_rpy = RPY(r, p, y)

        self.pose_received = True

    def velocity_callback(self, msg):
        """ Update velocity from hybrid localization velocity topic """
        self.current_velocity.x = msg.twist.linear.x
        self.current_velocity.y = msg.twist.linear.y
        self.current_velocity.z = msg.twist.linear.z

        self.velocity_received = True

    def update_orientation_from_state(self):
        """
        Update orientation from state message as backup
        """
        pose = self.current_state.pose_with_covariance.pose
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        r, p, y = tf.transformations.euler_from_quaternion(quaternion)

        # Only update if we haven't received pose from hybrid localization recently
        if not self.pose_received:
            self.previous_rpy = self.current_rpy
            self.current_rpy = RPY(r, p, y)

    def current_mode_callback(self, msg):
        """ Update the current mode """
        self.current_mode = msg.mode

    def desired_mode_callback(self, msg):
        """ Update the desired mode """
        self.desired_mode = msg.mode

    def lost_callback(self, msg):
        """ Update lost status with confidence tracking """
        was_lost = self.lost
        self.lost = msg.data

        # Track detection history
        self.tag_detection_history.append(not self.lost)
        if len(self.tag_detection_history) > self.max_history_length:
            self.tag_detection_history.pop(0)

        # Calculate tracking confidence based on recent history
        if len(self.tag_detection_history) > 0:
            self.tracking_confidence = sum(self.tag_detection_history) / len(self.tag_detection_history)

        # Lost counter for hysteresis
        if self.lost:
            self.lost_counter = min(self.lost_counter + 1, 30)
        else:
            self.lost_counter = max(self.lost_counter - 2, 0)
            # Update last known good position when tracking is stable
            if self.tracking_confidence > 0.7:
                self.last_known_good_position.x = self.current_position.x
                self.last_known_good_position.y = self.current_position.y
                self.drift_compensation_active = False

    def desired_pose_callback(self, msg):
        """ Ignore pose commands - we hover at fixed position """
        pass

    def desired_twist_callback(self, msg):
        """ Ignore twist commands - we hover at fixed position """
        pass

    def position_control_callback(self, msg):
        """ Ignore position control changes - always use position control """
        pass

    def reset_callback(self, empty):
        """ Reset to hover position """
        self.desired_position = Position(x=0.0, y=0.0, z=0.5)
        self.desired_velocity = Velocity()
        self.last_known_good_position = Position(x=0.0, y=0.0, z=0.5)

    def step(self):
        """ Returns the commands generated by the pid for hovering """
        # Check if we have received necessary data
        if not (self.pose_received and self.velocity_received):
            print("Waiting for pose and velocity data...")
            return cmds.disarm_cmd

        self.calc_error()

        cmd = self.pid.step(self.pid_error, 0)

        print("PID error (x,y,z): ({:.3f}, {:.3f}, {:.3f})".format(
            self.pid_error.x, self.pid_error.y, self.pid_error.z))
        print("PID output (roll, pitch, yaw, throttle): ({:.1f}, {:.1f}, {:.1f}, {:.1f})".format(
            cmd[0], cmd[1], cmd[2], cmd[3]))

        return cmd

    def calc_error(self):
        """Calculate unsmoothed position error and write into self.pid_error."""

        # ---------- desired set-point ----------
        target_x = self.desired_position.x
        target_y = self.desired_position.y
        target_z = self.desired_position.z

        # ---------- raw errors ----------
        err_x = target_x - self.current_position.x
        err_y = target_y - self.current_position.y * -1
        err_z = target_z - self.current_position.z  # <-- fixed: no trailing comma

        self.position_error.x = err_x
        self.position_error.y = err_y
        self.position_error.z = err_z

        # ---------- deadbands ----------
        deadband_xy = 0.005  # 5 mm
        deadband_z = 0.02  # 2 cm

        if abs(err_x) < deadband_xy:
            err_x = 0.0
        if abs(err_y) < deadband_xy:
            err_y = 0.0
        if abs(err_z) < deadband_z:
            err_z = 0.0

        # ---------- expose to PID ----------
        self.pid_error.x = err_x
        self.pid_error.y = err_y
        self.pid_error.z = err_z

    def reset(self):
        """ Reset PID controllers and filters """
        self.position_error = Error()
        self.velocity_error = Error()
        self.desired_position = Position(x=0.0, y=0.0, z=0.5)
        self.desired_velocity = Velocity()

        # Reset PIDs
        self.pid.reset()
        # self.lr_pid.reset()
        # self.fb_pid.reset()
        # self.z_pid.reset()

        # Reset all filters
        # self.filtered_x = 0.0
        # self.filtered_y = 0.0
        # self.smoothed_pos_x = 0.0
        # self.smoothed_pos_y = 0.0
        self.ultra_smooth_x = 0.0
        self.ultra_smooth_y = 0.0
        self.velocity_smooth_x = 0.0
        self.velocity_smooth_y = 0.0
        self.z_filtered = 0.0

        # Reset tracking
        self.lost_counter = 0
        self.tracking_confidence = 1.0
        self.tag_detection_history = []
        self.drift_compensation_active = False

        # Reset data flags
        self.pose_received = False
        self.velocity_received = False

    def ctrl_c_handler(self, signal, frame):
        """ Gracefully handles ctrl-c """
        print('Caught ctrl-c\nStopping Controller')
        sys.exit()

    def publish_cmd(self, cmd):
        """Publish the controls to /pidrone/fly_commands """
        msg = RC()
        msg.roll = cmd[0] + 14
        msg.pitch = cmd[1] + 6
        msg.yaw = cmd[2]
        msg.throttle = cmd[3]
        self.cmdpub.publish(msg)


def main(ControllerClass):
    parser = argparse.ArgumentParser(prog='Improved Hover PID Controller',
                                     description='Controls drone with smoother tag tracking and drift prevention',
                                     epilog='Enhanced stability and drift compensation')

    parser.add_argument('-v', '--verbose', choices=[0, 1, 2], default=0,
                        help="Verbosity between 0 and 2, 2 is most verbose")

    args = parser.parse_args()

    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    # Create the PIDController object
    pid_controller = ControllerClass()

    # Publishers
    ############
    pid_controller.cmdpub = rospy.Publisher('/pidrone/fly_commands', RC, queue_size=1)
    pid_controller.position_control_pub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
    pid_controller.heartbeat_pub = rospy.Publisher('/pidrone/heartbeat/pid_controller', Empty, queue_size=1)

    # Subscribers
    #############
    rospy.Subscriber('/pidrone/state', State, pid_controller.current_state_callback)
    rospy.Subscriber('/hybrid_localization/pose', PoseStamped, pid_controller.pose_callback)
    rospy.Subscriber('/hybrid_localization/velocity', TwistStamped, pid_controller.velocity_callback)
    rospy.Subscriber('/pidrone/desired/pose', Pose, pid_controller.desired_pose_callback)
    rospy.Subscriber('/pidrone/desired/twist', Twist, pid_controller.desired_twist_callback)
    rospy.Subscriber('/pidrone/mode', Mode, pid_controller.current_mode_callback)
    rospy.Subscriber('/pidrone/desired/mode', Mode, pid_controller.desired_mode_callback)
    rospy.Subscriber('/pidrone/position_control', Bool, pid_controller.position_control_callback)
    rospy.Subscriber('/pidrone/reset_transform', Empty, pid_controller.reset_callback)
    rospy.Subscriber('/pidrone/picamera/lost', Bool, pid_controller.lost_callback)

    # Non-ROS Setup
    ###############
    signal.signal(signal.SIGINT, pid_controller.ctrl_c_handler)
    loop_rate = rospy.Rate(20)

    print('Improved Hover PID Controller Started - Using hybrid localization topics')

    while not rospy.is_shutdown():
        pid_controller.heartbeat_pub.publish(Empty())

        # Get PID command
        fly_command = pid_controller.step()

        # State machine for arm/disarm
        if pid_controller.current_mode == 'DISARMED':
            if pid_controller.desired_mode == 'ARMED':
                pid_controller.reset()
                print("Armed - Ready to hover with improved stability")

        elif pid_controller.current_mode == 'ARMED':
            if pid_controller.desired_mode == 'FLYING':
                pid_controller.reset()
                print("Starting improved hover at (0, 0, 0.5)")

        elif pid_controller.current_mode == 'FLYING':
            if pid_controller.desired_mode == 'FLYING':
                # Safety check - altitude limit
                current_z = pid_controller.current_position.z
                if current_z > 0.8:
                    fly_command = cmds.disarm_cmd
                    print("SAFETY: Disarming - altitude too high: {:.3f}".format(current_z))
                    break

                # Publish hover command
                pid_controller.publish_cmd(fly_command)

            elif pid_controller.desired_mode == 'DISARMED':
                pid_controller.reset()
                print('Hover session ended')

        # Debug output
        if args.verbose >= 2:
            print('Current pos: ({:.3f}, {:.3f}, {:.3f})'.format(pid_controller.current_position.x,
                                                                 pid_controller.current_position.y,
                                                                 pid_controller.current_position.z))
            print('Current vel: ({:.3f}, {:.3f}, {:.3f})'.format(pid_controller.current_velocity.x,
                                                                 pid_controller.current_velocity.y,
                                                                 pid_controller.current_velocity.z))
            print('Tracking confidence: {:.2f}, Lost counter: {}'.format(pid_controller.tracking_confidence,
                                                                         pid_controller.lost_counter))
            print('Data received - Pose: {}, Velocity: {}'.format(pid_controller.pose_received,
                                                                  pid_controller.velocity_received))

        if args.verbose >= 1:
            print('Commands (r,p,y,t): {}'.format(fly_command))

        loop_rate.sleep()


if __name__ == '__main__':
    main(PIDController)