# -*- coding: utf-8 -*-

#!/usr/bin/env python

import argparse
import rospy
from pidrone_pkg.msg import State, StateGroundTruth, UkfStats
from apriltag_ros.msg import AprilTagDetectionArray
import subprocess
import os
import numpy as np


class StateEstimator(object):
    """
    This class is intended to unify the different state estimators so that the
    user only has to call this script to interact with state estimators. This
    node publishes to /pidrone/state, which offers the best state estimate
    based on whichever state estimators are to be used, depending on the
    command-line arguments that the user passes into this script.

    The different state estimators are:
        - EMA: uses an exponential moving average
        - UKF with a 2D state vector
        - UKF with a 7D state vector
        - UKF with a 12D state vector
        - MoCap: provides ground truth
        - Simulation (drone_simulator.py): provides simulated ground truth
        - AprilTag: provides absolute positioning when tags are detected

    This script runs the state estimators in non-blocking subprocesses. When
    this script is terminating, there is a finally clause that will attempt to
    terminate the subprocesses. Note that this needs to be tested well, and that
    the shell=True argument for these subprocesses could be a security hazard
    that should be investigated further.
    """

    def __init__(self, primary, others, ir_throttled=False, imu_throttled=False,
                 optical_flow_throttled=False, camera_pose_throttled=False,
                 sdim=1, student_ukf=False, ir_var=None, loop_hz=None,
                 apriltag_enabled=False, apriltag_reset_threshold=0.1):
        self.state_msg = State()
        self.ema_state_msg = State()  # EMA fallback için

        self.ir_throttled = ir_throttled
        self.imu_throttled = imu_throttled
        self.optical_flow_throttled = optical_flow_throttled
        self.camera_pose_throttled = camera_pose_throttled
        self.loop_hz = loop_hz

        # AprilTag integration parameters
        self.apriltag_enabled = apriltag_enabled
        self.apriltag_reset_threshold = apriltag_reset_threshold
        self.last_apriltag_detection = None
        self.apriltag_detection_timeout = 2.0  # seconds
        self.position_bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.velocity_bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_reset_time = rospy.Time.now()

        self.primary_estimator = primary
        self.other_estimators = others
        if self.other_estimators is None:
            self.other_estimators = []
        self.estimators = list(self.other_estimators)
        self.estimators.append(self.primary_estimator)

        # AprilTag primary ise EMA'yı otomatik olarak fallback olarak ekle
        if self.primary_estimator == 'apriltag' and 'ema' not in self.estimators:
            self.estimators.append('ema')
            if self.other_estimators is None:
                self.other_estimators = []
            if 'ema' not in self.other_estimators:
                self.other_estimators.append('ema')

        # Add apriltag to estimators if enabled
        if self.apriltag_enabled and 'apriltag' not in self.estimators:
            self.estimators.append('apriltag')

        student_project_pkg_dir = 'pidrone_project2_ukf'
        pidrone_pkg_dir = 'pidrone_pkg'

        if student_ukf:
            program_str = 'rosrun ' + student_project_pkg_dir + ' StateEstimators/student_'
        else:
            program_str = 'rosrun ' + pidrone_pkg_dir + ' scripts/StateEstimators/'

        # TODO: Test this IR variance argument passing
        # TODO: Test if it is necessary to include "scripts/" in this command.
        #       It might have worked beforehand without it, with rosrun...
        sim_cmd = 'rosrun pidrone_pkg scripts/drone_simulator.py --dim ' + str(sdim)
        if ir_var is not None:
            # Pass in the IR variance value given as a command-line argument
            sim_cmd += ' --ir_var ' + str(ir_var)

        self.process_cmds_dict = {
            'ema': 'rosrun pidrone_pkg scripts/StateEstimators/state_estimator_ema.py',
            'ukf2d': '{}state_estimator_ukf_2d.py'.format(program_str),
            'ukf7d': '{}state_estimator_ukf_7d.py'.format(program_str),
            'ukf12d': 'rosrun pidrone_pkg scripts/StateEstimators/state_estimator_ukf_12d.py',
            'mocap': 'rosrun pidrone_pkg scripts/state_estimator_mocap.py',  # TODO: Implement this
            'simulator': sim_cmd,
            'apriltag': None  # AprilTag handled internally, no separate process
        }

        self.can_use_throttled_ir = ['ukf2d', 'ukf7d', 'ukf12d']
        self.can_use_throttled_imu = ['ukf2d', 'ukf7d', 'ukf12d']
        self.can_use_throttled_optical_flow = ['ukf7d', 'ukf12d']
        self.can_use_throttled_camera_pose = ['ukf7d']
        self.can_use_loop_hz = ['ukf2d', 'ukf7d']

        # List to store the process objects from subprocess.Popen()
        self.processes = []

        self.ukf_topics = {2: '/pidrone/state/ukf_2d',
                           7: '/pidrone/state/ukf_7d',
                           12: '/pidrone/state/ukf_12d'}
        self.ema_topic = '/pidrone/state/ema'
        # NOTE: Currently not supported is running both a simulator and mocap at
        #       the same time for state estimation. There is no apparent need
        #       for such functionality.
        self.mocap_topic = '/pidrone/state/ground_truth'
        self.simulator_topic = '/pidrone/state/ground_truth'
        self.apriltag_topic = '/tag_detections'  # AprilTag detection topic

        self.state_pub = rospy.Publisher('/pidrone/state', State, queue_size=1,
                                         tcp_nodelay=False)

        # Publisher for bias correction commands
        self.bias_correction_pub = rospy.Publisher('/pidrone/bias_correction',
                                                   State, queue_size=1, tcp_nodelay=False)

        self.setup_ukf_with_ground_truth()
        self.start_estimator_subprocess_cmds()


    def start_estimator_subprocess_cmds(self):
        cmd = self.process_cmds_dict[self.primary_estimator]
        if cmd is not None:  # Skip None commands (like apriltag)
            cmd = self.append_throttle_flags(cmd, self.primary_estimator)
        process_cmds = [cmd] if cmd is not None else []

        if self.primary_estimator == 'ukf2d':
            # We want the EMA to provide x and y position and velocity
            # estimates, for example, to supplement the 2D UKF's z position and
            # velocity estimates.
            process_cmds.append(self.process_cmds_dict['ema'])
            self.ema_state_msg = State()
            rospy.Subscriber(self.ema_topic, State, self.ema_helper_callback)
            rospy.Subscriber(self.ukf_topics[2], State, self.state_callback)
        elif self.primary_estimator == 'ukf7d':
            rospy.Subscriber(self.ukf_topics[7], State, self.state_callback)
        elif self.primary_estimator == 'ukf12d':
            rospy.Subscriber(self.ukf_topics[12], State, self.state_callback)
        elif self.primary_estimator == 'ema':
            rospy.Subscriber(self.ema_topic, State, self.state_callback)
        elif self.primary_estimator == 'mocap':
            rospy.Subscriber(self.mocap_topic, State, self.state_callback)
        elif self.primary_estimator == 'simulator':
            rospy.Subscriber(self.simulator_topic, State, self.state_callback)
        elif self.primary_estimator == 'apriltag':
            # AprilTag as primary estimator - subscribe to apriltag detections
            rospy.Subscriber(self.apriltag_topic, AprilTagDetectionArray,
                             self.apriltag_primary_callback)
            # EMA fallback için subscriber ekle
            rospy.Subscriber(self.ema_topic, State, self.ema_fallback_callback)

        # Set up AprilTag subscriber if enabled (for bias correction)
        if self.apriltag_enabled:
            rospy.Subscriber(self.apriltag_topic, AprilTagDetectionArray,
                             self.apriltag_bias_correction_callback)

        # Set up the process commands for the non-primary estimators
        for other_estimator in self.other_estimators:
            # Avoid running a subprocess more than once
            other_cmd = self.process_cmds_dict[other_estimator]
            if other_cmd is not None and other_cmd not in process_cmds:
                other_cmd = self.append_throttle_flags(other_cmd, other_estimator)
                process_cmds.append(other_cmd)

        for p in process_cmds:
            print('Starting:', p)
            # NOTE: shell=True could be security hazard
            self.processes.append((p, subprocess.Popen(p, shell=True)))

    def ema_fallback_callback(self, msg):
        """
        AprilTag primary olduğunda EMA fallback için callback
        """
        self.ema_state_msg = msg

    def apriltag_primary_callback(self, msg):
        """
        Callback when AprilTag is the primary estimator
        """
        current_time = rospy.Time.now()

        if len(msg.detections) > 0:
            # AprilTag tespit edildi - pozisyon için AprilTag, hız için EMA kullan
            detection = msg.detections[0]
            self.last_apriltag_detection = current_time

            self.state_msg.header.stamp = current_time

            # AprilTag'den pozisyon al
            self.state_msg.pose_with_covariance.pose.position = detection.pose.pose.pose.position
            self.state_msg.pose_with_covariance.pose.orientation = detection.pose.pose.pose.orientation
            self.state_msg.pose_with_covariance.covariance = detection.pose.pose.covariance

            # EMA'dan hız al (eğer mevcut ise)
            if hasattr(self, 'ema_state_msg') and self.ema_state_msg.header.stamp.to_sec() > 0:
                self.state_msg.twist_with_covariance.twist.linear = self.ema_state_msg.twist_with_covariance.twist.linear
                self.state_msg.twist_with_covariance.twist.angular = self.ema_state_msg.twist_with_covariance.twist.angular
                self.state_msg.twist_with_covariance.covariance = self.ema_state_msg.twist_with_covariance.covariance

            print("AprilTag detected - using AprilTag for position, EMA for velocity")

        else:
            # AprilTag tespit edilmedi - tamamen EMA'ya fallback yap
            if hasattr(self, 'ema_state_msg') and self.ema_state_msg.header.stamp.to_sec() > 0:
                self.state_msg = self.ema_state_msg
                self.state_msg.header.stamp = current_time
                print("No AprilTag detected - falling back to EMA")
            else:
                # EMA da yoksa önceki değerleri koru
                self.state_msg.header.stamp = current_time
                print("No AprilTag or EMA available - keeping last known state")

        self.state_pub.publish(self.state_msg)

    def apriltag_bias_correction_callback(self, msg):
        """
        Handle AprilTag detections for bias correction
        """
        if len(msg.detections) == 0:
            return  # No tags detected

        current_time = rospy.Time.now()
        self.last_apriltag_detection = current_time

        # Use the first detected tag
        detection = msg.detections[0]

        # Get current state estimate from primary estimator
        if hasattr(self, 'state_msg') and self.state_msg.header.stamp.to_sec() > 0:
            # Calculate position error
            apriltag_pos = detection.pose.pose.pose.position
            current_pos = self.state_msg.pose_with_covariance.pose.position

            position_error = {
                'x': apriltag_pos.x - current_pos.x,
                'y': apriltag_pos.y - current_pos.y,
                'z': apriltag_pos.z - current_pos.z
            }

            # Check if error exceeds threshold - if so, apply bias correction
            error_magnitude = np.sqrt(position_error['x'] ** 2 +
                                      position_error['y'] ** 2 +
                                      position_error['z'] ** 2)

            if error_magnitude > self.apriltag_reset_threshold:

                # Update bias values
                self.position_bias['x'] += position_error['x']
                self.position_bias['y'] += position_error['y']
                self.position_bias['z'] += position_error['z']

                # Don't reset velocity bias - keep velocity estimates as they were

                # Publish bias correction message
                bias_msg = State()
                bias_msg.header.stamp = current_time
                bias_msg.pose_with_covariance.pose.position.x = self.position_bias['x']
                bias_msg.pose_with_covariance.pose.position.y = self.position_bias['y']
                bias_msg.pose_with_covariance.pose.position.z = self.position_bias['z']
                bias_msg.twist_with_covariance.twist.linear.x = self.velocity_bias['x']
                bias_msg.twist_with_covariance.twist.linear.y = self.velocity_bias['y']
                bias_msg.twist_with_covariance.twist.linear.z = self.velocity_bias['z']

                self.bias_correction_pub.publish(bias_msg)
                self.last_reset_time = current_time

    def is_apriltag_available(self):
        """
        Check if AprilTag detection is recent enough to be considered available
        """
        if self.last_apriltag_detection is None:
            return False

        time_since_detection = (rospy.Time.now() - self.last_apriltag_detection).to_sec()
        return time_since_detection < self.apriltag_detection_timeout

    def setup_ukf_with_ground_truth(self):
        """
        Determine if a UKF is being run simultaneously with ground truth. This
        informs whether or not we can provide certain analytics about the UKF's
        performance to the user in the web interface
        """
        do_setup = (('ukf2d' in self.estimators or 'ukf7d' in self.estimators or
                     'ukf12d' in self.estimators) and
                    ('simulator' in self.estimators or 'mocap' in self.estimators))
        if do_setup:
            self.last_ground_truth_height = None
            self.last_ukf_height = None
            self.ukf_stats_pub = rospy.Publisher('/pidrone/ukf_stats', UkfStats, queue_size=1,
                                                 tcp_nodelay=False)
            if 'ukf' in self.primary_estimator:
                # If the primary estimator is a UKF, use this one
                ukf_to_use = self.primary_estimator
            else:
                # Search through the other estimators
                possible_ukfs = []
                for estimator in self.other_estimators:
                    if 'ukf' in estimator:
                        possible_ukfs.append(estimator)
                if len(possible_ukfs) > 1:
                    # Give user the option
                    got_good_input = False
                    while not got_good_input:
                        print('Please enter the list number of which UKF to use'
                              ' for comparison against ground truth:')
                        for num, ukf in enumerate(possible_ukfs):
                            print('{}: {}'.format(num + 1, ukf))
                        selection = input()
                        try:
                            selection = int(selection)
                            if selection <= 0 or selection > len(possible_ukfs):
                                raise ValueError
                            ukf_to_use = possible_ukfs[selection - 1]
                            got_good_input = True
                        except ValueError:
                            print('Invalid input.')
                elif len(possible_ukfs) == 1:
                    # This is the only other option; otherwise, do_setup would
                    # be False
                    ukf_to_use = possible_ukfs[0]
            if ukf_to_use == 'ukf2d':
                rospy.Subscriber(self.ukf_topics[2], State, self.ukf_analytics_callback)
            elif ukf_to_use == 'ukf7d':
                rospy.Subscriber(self.ukf_topics[7], State, self.ukf_analytics_callback)
            elif ukf_to_use == 'ukf12d':
                rospy.Subscriber(self.ukf_topics[12], State, self.ukf_analytics_callback)
            for estimator in self.estimators:
                if estimator == 'simulator':
                    topic = self.simulator_topic
                elif estimator == 'mocap':
                    topic = self.mocap_topic
            rospy.Subscriber(topic, StateGroundTruth, self.ground_truth_analytics_callback)

    def append_throttle_flags(self, cmd, estimator):
        """
        Append throttle flags and/or a loop Hz flag to the given command.
        """
        if self.ir_throttled and estimator in self.can_use_throttled_ir:
            cmd += ' --ir_throttled'
        if self.imu_throttled and estimator in self.can_use_throttled_imu:
            cmd += ' --imu_throttled'
        if self.optical_flow_throttled and estimator in self.can_use_throttled_optical_flow:
            cmd += ' --optical_flow_throttled'
        if self.camera_pose_throttled and estimator in self.can_use_throttled_camera_pose:
            cmd += ' --camera_pose_throttled'
        if self.loop_hz is not None and estimator in self.can_use_loop_hz:
            cmd += ' -hz {}'.format(self.loop_hz)
        return cmd

    def apply_bias_correction(self, x, y, z, vel_x, vel_y, vel_z):
        """
        Apply bias correction to position estimates only
        Velocities are kept as-is from the primary estimator
        """
        # Apply position bias correction
        corrected_x = x - self.position_bias['x']
        corrected_y = y - self.position_bias['y']
        corrected_z = z - self.position_bias['z']

        # Don't modify velocities - return them as they were
        return corrected_x, corrected_y, corrected_z, vel_x, vel_y, vel_z

    def state_callback(self, msg):
        """
        Callback that handles the primary estimator republishing.
        """
        # TODO: Consider creating a new State message rather than modifying just
        #       one State message
        self.state_msg.header.stamp = rospy.Time.now()

        if self.primary_estimator == 'ukf2d':
            # Use EMA data for x and y positions and velocities
            x = self.ema_state_msg.pose_with_covariance.pose.position.x
            y = self.ema_state_msg.pose_with_covariance.pose.position.y
            vel_x = self.ema_state_msg.twist_with_covariance.twist.linear.x
            vel_y = self.ema_state_msg.twist_with_covariance.twist.linear.y
        else:
            # Use primary_estimator data for x and y positions and velocities
            x = msg.pose_with_covariance.pose.position.x
            y = msg.pose_with_covariance.pose.position.y
            vel_x = msg.twist_with_covariance.twist.linear.x
            vel_y = msg.twist_with_covariance.twist.linear.y

        z = msg.pose_with_covariance.pose.position.z
        vel_z = msg.twist_with_covariance.twist.linear.z
        orientation = msg.pose_with_covariance.pose.orientation
        vel_angular = msg.twist_with_covariance.twist.angular

        # Apply bias correction if AprilTag integration is enabled
        if self.apriltag_enabled:
            x, y, z, vel_x, vel_y, vel_z = self.apply_bias_correction(
                x, y, z, vel_x, vel_y, vel_z)

        self.state_msg.pose_with_covariance.pose.position.x = x
        self.state_msg.pose_with_covariance.pose.position.y = y
        self.state_msg.pose_with_covariance.pose.position.z = z
        self.state_msg.pose_with_covariance.pose.orientation = orientation
        self.state_msg.twist_with_covariance.twist.linear.x = vel_x
        self.state_msg.twist_with_covariance.twist.linear.y = vel_y
        self.state_msg.twist_with_covariance.twist.linear.z = vel_z
        self.state_msg.twist_with_covariance.twist.angular = vel_angular

        # Include covariances
        self.state_msg.pose_with_covariance.covariance = msg.pose_with_covariance.covariance
        self.state_msg.twist_with_covariance.covariance = msg.twist_with_covariance.covariance

        # Adjust covariance if AprilTag is available (higher confidence)
        if self.apriltag_enabled and self.is_apriltag_available():
            # Reduce position covariance when AprilTag is available
            covariance_reduction_factor = 0.1
            for i in [0, 7, 14]:  # x, y, z position covariances
                self.state_msg.pose_with_covariance.covariance[i] *= covariance_reduction_factor

        self.state_pub.publish(self.state_msg)

    def ema_helper_callback(self, msg):
        """
        When the primary estimator is the 2D UKF, populate self.ema_state_msg
        in this callback.
        """
        self.ema_state_msg.pose_with_covariance.pose.position.x = msg.pose_with_covariance.pose.position.x
        self.ema_state_msg.pose_with_covariance.pose.position.y = msg.pose_with_covariance.pose.position.y
        self.ema_state_msg.twist_with_covariance.twist.linear.x = msg.twist_with_covariance.twist.linear.x
        self.ema_state_msg.twist_with_covariance.twist.linear.y = msg.twist_with_covariance.twist.linear.y

    def ukf_analytics_callback(self, msg):
        self.last_ukf_height = msg.pose_with_covariance.pose.position.z
        if self.last_ground_truth_height is not None:
            stats_msg = UkfStats()
            stats_msg.header.stamp = rospy.Time.now()
            stats_msg.error = self.last_ukf_height - self.last_ground_truth_height
            stats_msg.stddev = (msg.pose_with_covariance.covariance[14]) ** 0.5
            self.ukf_stats_pub.publish(stats_msg)

    def ground_truth_analytics_callback(self, msg):
        self.last_ground_truth_height = msg.pose.position.z
        # Ground truth value should come in before UKF value. Could perhaps
        # match timestamps or header sequence numbers to check or synchronize.


def check_positive_float_duration(val):
    """
    Function to check that the --loop_hz command-line argument is a positive
    float.
    """
    value = float(val)
    if value <= 0.0:
        raise argparse.ArgumentTypeError('Loop Hz must be positive')
    return value


def main():
    parser = argparse.ArgumentParser(description=('The state estimator node '
                                                  'can provide state estimates using a 1D UKF (2D state vector), '
                                                  'a 3D UKF (7D or 12D state vector), an EMA, MoCap, '
                                                  'simulated ground truth data, or AprilTag positioning. The default is the EMA. The '
                                                  'default UKF is the UKF with 2D state vector. The primary '
                                                  'state estimator determines what is published to '
                                                  '/pidrone/state, except that an incomplete state estimator '
                                                  'like the 2D UKF will also use EMA estimates to populate x and '
                                                  'y position, for example.'))

    arg_choices = ['ema', 'ukf2d', 'ukf7d', 'ukf12d', 'mocap', 'simulator', 'apriltag']

    parser.add_argument('--primary', '-p',
                        choices=arg_choices,
                        default='ema',
                        help='Select the primary state estimation method')
    parser.add_argument('--others', '-o',
                        choices=arg_choices,
                        nargs='+',
                        help=('Select other state estimation nodes to run '
                              'alongside the primary state estimator, e.g., '
                              'for visualization or debugging purposes'))

    # Arguments to determine if the throttle command is being used. E.g.:
    #   rosrun topic_tools throttle messages /pidrone/infrared 40.0
    # If one of these is passed in, it will act on all state estimators that can
    # take it in as a command-line argument.
    parser.add_argument('--ir_throttled', action='store_true',
                        help=('Use throttled infrared topic /pidrone/infrared_throttle'))
    parser.add_argument('--imu_throttled', action='store_true',
                        help=('Use throttled IMU topic /pidrone/imu_throttle'))
    parser.add_argument('--optical_flow_throttled', action='store_true',
                        help=('Use throttled optical flow topic /pidrone/picamera/twist_throttle'))
    parser.add_argument('--camera_pose_throttled', action='store_true',
                        help=('Use throttled camera pose topic /pidrone/picamera/pose_throttle'))

    parser.add_argument('--sdim', default=1, type=int, choices=[1, 2, 3],
                        help=('Number of spatial dimensions in which to '
                              'simulate the drone\'s motion, if running the '
                              'drone simulator (default: 1)'))

    parser.add_argument('--student_ukf', action='store_true',
                        help=('Use student UKF'))
    # TODO: Test out the --ir_var flag
    parser.add_argument('--ir_var', type=float,
                        help=('IR sensor variance to use in 1D simulation'))

    parser.add_argument('--loop_hz', '-hz', default=30.0,
                        type=check_positive_float_duration,
                        help=('Frequency at which to run the predict-update '
                              'loop of the UKF (default: 30)'))

    # AprilTag specific arguments
    parser.add_argument('--apriltag_enabled', action='store_true',
                        help=('Enable AprilTag bias correction for other estimators'))
    parser.add_argument('--apriltag_reset_threshold', type=float, default=0.1,
                        help=('Position error threshold (in meters) for AprilTag bias correction (default: 0.1)'))

    args = parser.parse_args()

    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    se = None

    try:
        se = StateEstimator(primary=args.primary,
                            others=args.others,
                            ir_throttled=args.ir_throttled,
                            imu_throttled=args.imu_throttled,
                            optical_flow_throttled=args.optical_flow_throttled,
                            camera_pose_throttled=args.camera_pose_throttled,
                            sdim=args.sdim,
                            student_ukf=args.student_ukf,
                            ir_var=args.ir_var,
                            loop_hz=args.loop_hz,
                            apriltag_enabled=args.apriltag_enabled,
                            apriltag_reset_threshold=args.apriltag_reset_threshold)

        rospy.spin()

    except Exception as e:
        print(e)
    finally:
        # Terminate the subprocess calls. Note, however, that if Ctrl-C is
        # entered in stdin, it seems that the subprocesses also get the Ctrl-C
        # input and are terminating based on KeyboardInterrupt
        print('Terminating subprocess calls...')
        for process_name, process in se.processes:
            print('Terminating:', process_name)
            process.terminate()
        print('Done.')


if __name__ == "__main__":
    main()