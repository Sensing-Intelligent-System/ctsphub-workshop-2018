#!/usr/bin/env python

import rospy
from math import pi

from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import MotorStateList

def encoder2rad(pos_in_encoder):
	pos_in_rad = [None, None, None]
	for i in range(0, len(pos_in_encoder)):
		pos_in_rad[i] = -5*pi/6 + float(pos_in_encoder[i]) / 1024 * 5*pi/3
	return pos_in_rad 

def cb_state(msg):
	if len(msg.motor_states) == 3:
		state = JointState()
		state.header.stamp = rospy.Time.now()
		state.name = ["pan_joint", "tilt_joint", "base_to_left", "base_to_right"]
	
		position_encoder = [msg.motor_states[0].position, 
			 	    msg.motor_states[1].position,
				    msg.motor_states[2].position]

		position = encoder2rad(position_encoder)
		gripper_dis = abs(position[2]) * 0.0065

		state.position = position[0:2]
		state.position.append(gripper_dis)
		state.position.append(gripper_dis)

		state.velocity = [msg.motor_states[0].speed, 
			  	  msg.motor_states[1].speed,
				  msg.motor_states[2].speed,
				  msg.motor_states[2].speed]

		pub_joint_state.publish(state)

if __name__ == "__main__":
	rospy.init_node("state_remap_node")
	rospy.Subscriber("/motor_states/pan_tilt_port", MotorStateList, cb_state, queue_size = 20)
	pub_joint_state = rospy.Publisher("/motor_states/joint_state", JointState, queue_size = 20)
	rospy.spin()
