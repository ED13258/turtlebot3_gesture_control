#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

fingers = 0
def callback(data):
	global fingers
	fingers = data.data
	return fingers

def makeSimpleProfile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output

def constrain(input, low, high):
	if input < low:
		input = low
	elif input > high:
		input = high
	else:
		input = input

	return input

def checkLinearLimitVelocity(vel):
	if turtlebot3_model == "burger":
		vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
	elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
		vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
	else:
		vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

	return vel

def checkAngularLimitVelocity(vel):
	if turtlebot3_model == "burger":
		vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
	elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
		vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
	else:
		vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

	return vel

if __name__=="__main__":
	rospy.init_node('teleop')
	cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	#rate = rospy.Rate(20)

	turtlebot3_model = rospy.get_param("model", "burger")

	status = 0
	target_linear_vel   = 0.0
	target_angular_vel  = 0.0
	control_linear_vel  = 0.0
	control_angular_vel = 0.0

	while not rospy.is_shutdown():

		#定義了一個Subscriber，會去監聽 num_of_fingers 這個topic，透過String格式的訊息傳輸，並在接收到訊息之後呼叫callback function
		rospy.Subscriber("num_of_fingers", String, callback)

		if fingers == '2':
			target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
			status = status + 1
			#print(vels(target_linear_vel,target_angular_vel))
			rospy.loginfo("driving forward")
		elif fingers == '3':
			target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
			status = status + 1
			#print(vels(target_linear_vel,target_angular_vel))
			rospy.loginfo("turning left")
		elif fingers == '4':
			target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
			status = status + 1
			#print(vels(target_linear_vel,target_angular_vel))
			rospy.loginfo("turning right")
		elif fingers == '5':
			target_linear_vel   = 0.0
			control_linear_vel  = 0.0
			target_angular_vel  = 0.0
			control_angular_vel = 0.0
			#print(vels(target_linear_vel, target_angular_vel))
			rospy.loginfo("stopped")
		else:
			target_linear_vel   = 0.0
			control_linear_vel  = 0.0
			target_angular_vel  = 0.0
			control_angular_vel = 0.0
			#print(vels(target_linear_vel, target_angular_vel))
			rospy.loginfo("stopped")

		if status == 20:
			#print(msg)
			status = 0

		twist = Twist()
		control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
		twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

		control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel


		cmd_vel_pub.publish(twist)
		#rate.sleep()
