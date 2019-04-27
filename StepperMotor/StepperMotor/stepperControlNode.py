#!/usr/bin/env python
import rospy
import serial
from motion_control.msg import Mobility

class StepperNode:
	def __init__(self):
		rospy.init_node("stepper_node")
		
		rospy.loginfo("connecting to stepper arduino")
		
		port = rospy.get_param("~stepper_port", "/dev/ttyUSB0")
        	baud_rate = int(rospy.get_param("~baud", "28800"))
		
		self.ard_com = serial.Serial(port,baud_rate, timeout=5)
		self.ard_com.write("0:0:0:0\n")

		rospy.Subscriber("steering", Mobility, self.Steering_callback)

		self.lp_fl = 0
		self.lp_fr = 0
		self.lp_rl = 0
		self.lp_rr = 0
	def run(self):
		rospy.spin()
	def Steering_callback(self, msg):

		if (msg.front_left != self.lp_fl or msg.front_right != self.lp_fr or msg.rear_left != self.lp_rl or msg.rear_right != self.lp_rr):
			self.ard_com.write(str(int(msg.front_left*2.44)) + ":" + str(int(msg.front_right*2.44)) + ":" + str(int(msg.rear_left*2.44)) + ":" + str(int(msg.rear_right*2.44)) + '\n')

			self.lp_fl = msg.front_right;
			self.lp_fr = msg.front_left;
			self.lp_rl = msg.rear_right;
			self.lp_rr = msg.rear_left;
			#rospy.loginfo(str(msg.front_left) + ":" + str(msg.front_right) + ":" + str(msg.rear_left) + ":" + str(msg.rear_right))
			#if (self.lastPos != msg.front_left):
			#	self.ard_com.write(str(int(msg.front_left)) + '\n')
			#	rospy.loginfo("front_left wheel: " +str(int(msg.front_left)))
			#	self.lastPos = msg.front_left
		if (self.ard_com.in_waiting > 0):
			rospy.loginfo(self.ard_com.readline())



if __name__ == "__main__":
    try:
        node = StepperNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
