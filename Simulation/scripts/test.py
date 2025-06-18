#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import time
import math

def main():
    rospy.init_node('move_turtlebot3_stepwise')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    x = 9.590000  
    y = -6.000000
    z = 0.1
    
    rate = rospy.Rate(120)  
    step = 0.01  
    

    yaw_angle = math.pi / 2  
    qz = math.sin(yaw_angle / 2)  
    qw = math.cos(yaw_angle / 2)  
    
    while not rospy.is_shutdown():
        x += step  
        
        state_msg = ModelState()
        state_msg.model_name = 'car_polo'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = qz  
        state_msg.pose.orientation.w = qw  
        
        state_msg.twist.linear.x = 0.0
        state_msg.twist.linear.y = 0.0
        state_msg.twist.linear.z = 0.0
        state_msg.twist.angular.x = 0.0
        state_msg.twist.angular.y = 0.0
        state_msg.twist.angular.z = 0.0
        state_msg.reference_frame = 'world'
        
        try:
            set_state(state_msg)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rate.sleep()

if __name__ == '__main__':
    main()
