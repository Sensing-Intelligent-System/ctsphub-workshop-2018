#!/usr/bin/env python
import rospy
import sys

from math import pi, atan2, asin, sqrt, sin, cos
from dynamixel_msgs.msg import MotorStateList
from std_msgs.msg import Float64

# arm dimension
a1 = 0.09 # joint1 to joint2
a2 = 0.07 # joint2 to motor3
H = 0.04 # base to joint1
L = 0.056 # motor3 to TCP

# Convert the position from encoder to radian
def position2rad(pos):
	pos_rad = [None, None]
	for i in range(0, len(pos)):
		pos_rad[i] = -5*pi/6 + float(pos[i]) / 1024 * 5*pi/3
	return pos_rad

class IK(object):
	def __init__(self, x, z, gripper_command):
		self.node_name = rospy.get_name()
		# Motor angles, in rad
		self.arm_pos = None
		# Target position
		self.x = x
		self.z = z
		# Gripper command
		# True: pinch up
		# False: lay down
		# None: not motion
		self.gripper_command = gripper_command
		# Publishers and subscribers
		self.pub_pan = rospy.Publisher("/pan_controller/command", Float64, queue_size = 1)
		self.pub_tilt = rospy.Publisher("/tilt_controller/command", Float64, queue_size = 1)
		self.pub_gripper = rospy.Publisher("/gripper_controller/command", Float64, queue_size = 1)
		self.sub_pos = rospy.Subscriber("/motor_states/pan_tilt_port", MotorStateList, self.cb_pos, queue_size = 10)
		
	# callback, get arm pose of the moment
	def cb_pos(self, msg):
		self.arm_pos = position2rad([msg.motor_states[0].position, msg.motor_states[1].position])

		try:
			# Process inverse kinematic
			self._IK()
			# Publish
			data_1 = Float64(self.nearest[0])
			data_2 = Float64(self.nearest[1])
			rospy.sleep(0.5)
			self.pub_pan.publish(data_1)
			rospy.sleep(0.5)
			self.pub_tilt.publish(data_2)
			if self.gripper_command == "True":
				rospy.sleep(0.5)
				self.pub_gripper.publish(Float64(1.5))
			elif self.gripper_command == "False":
				rospy.sleep(0.5)
				self.pub_gripper.publish(Float64(0))
			
			rospy.loginfo("End process")
			rospy.signal_shutdown("End process")
		except ValueError as e:
			print e
			rospy.loginfo("Given position not reachable")
			rospy.signal_shutdown("Failed")
	# IK process
	def _IK(self):
		x = self.x
		z = self.z
		
		theta = atan2(z-H, x)
		k = x**2 + (z-H)**2 + a1**2 - a2**2 - L**2
		theta_1_1 = asin(k/(2*a1*sqrt(x**2 + (z-H)**2))) - theta
		theta_1_2 = pi - asin(k/(2*a1*sqrt(x**2 + (z-H)**2))) - theta
		x_bar_1 = x - a1* sin(theta_1_1)
		z_bar_1 = (z-H) - a1 * cos(theta_1_1)
		x_bar_2 = x - a1* sin(theta_1_2)
		z_bar_2 = (z-H) - a1 * cos(theta_1_2)
		theta_2_1 = atan2((a2*x_bar_1 - L* z_bar_1), (a2*z_bar_1 + L*x_bar_1)) - theta_1_1
		theta_2_2 = atan2((a2*x_bar_2 - L* z_bar_2), (a2*z_bar_2 + L*x_bar_2)) - theta_1_2
		ans_1 = [theta_1_1, theta_2_1]
		ans_2 = [theta_1_2, theta_2_2]
		
		ans_1 = self._check_bound(ans_1)
		ans_2 = self._check_bound(ans_2)
		
		# Only one solution
		if ans_1 == None and ans_2 != None:
			self.nearest = ans_2
		elif ans_1 != None and ans_2 == None:
			self.nearest = ans_1
		# No solution
		elif ans_1 == None and ans_2 == None:
			self.nearest = None
			rospy.loginfo("Given position not reachable")
			rospy.signal_shutdown("Failed")
		# Two solutions
		# Find nearest solution
		# In L2 distance of joint space
		else:
			dis_1 = (ans_1[0]- self.arm_pos[0])**2 + (ans_1[1] - self.arm_pos[1]) ** 2
			dis_2 = (ans_2[0]- self.arm_pos[0])**2 + (ans_2[1] - self.arm_pos[1]) ** 2
			if dis_1 < dis_2:
				self.nearest = ans_1
			else:
				self.nearest = ans_2
		
		self.nearest[0] = float(format(self.nearest[0], '.3f'))
		self.nearest[1] = float(format(self.nearest[1], '.3f'))
		rospy.loginfo("[%s] Solution: %s" %(self.node_name, self.nearest))
		
	# joint 1 should be in [-1.743, 1.948]
	# joint 2 should be in [-1.626, 1.79]
	# Make sure the executable of the answer
	def _check_bound(self, ans):
		ans_ = ans
		# Change to [-pi, pi] branch
		for i in range(0, len(ans)):
			if ans[i] > pi:
				ans_[i] = 2* pi - ans[i]
			if ans[i] < -pi:
				ans_[i] = 2* pi + ans[i]
		if ans[0] > 1.948 or ans[0] < -1.743 or ans[1] > 1.79 or ans[1] < -1.626:
			ans_ = None
		return ans_

if __name__ == "__main__":
	if len(sys.argv) != 4:
		print "Not enough arguments!"
		print "Sample input: rosrun dynamixel_arm dynamixel_simple.py 0.056 0.2 None"
		sys.exit(0)
	rospy.init_node("arm_IK", disable_signals = True)
	x = float(sys.argv[1])
	z = float(sys.argv[2])
	if z < 0:
		print "Please give z greater than 0"
		sys.exit(0)
	gripper_command = str(sys.argv[3])
	k = x**2 + (z-H)**2 + a1**2 - a2**2 - L**2
	if abs(k/(2*a1*sqrt(x**2 + (z-H)**2))) > 1:
		print "Given position out of workspace"
		sys.exit(0)
	obj = IK(x , z, gripper_command)
	rospy.spin()

