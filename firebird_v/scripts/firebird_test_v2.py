#!/usr/bin/env python
#Inspired from https://github.com/badrobot15/firebird_ros

import rospy
from firebird_v.msg import LCD
from firebird_v.msg import SharpSensor, IR, Battery, WhiteLine
from std_msgs.msg import *
import time

class firebird():

	def __init__(self):

		self.buzzer_pub = rospy.Publisher('/toggle_buzzer', Empty, queue_size = 10)
		self.bargraph_led_pub = rospy.Publisher('/bargraph', UInt8, queue_size = 10)
		self.lcd_print_pub = rospy.Publisher('/lcd/print', LCD, queue_size = 10)
		self.lcd_clear_pub = rospy.Publisher('/lcd/clear', Empty, queue_size = 10)

		rospy.Subscriber('/sharp', SharpSensor, self.callback)
		rospy.Subscriber('/whiteline', WhiteLine, self.callback)
		rospy.Subscriber('/ir', IR, self.callback)
		
		rospy.init_node('firebird_control_test')

		self.lcd_msg = LCD()

		self.lcd_msg.row = 1
		self.lcd_msg.col = 1
		self.lcd_msg.val = ""


	def toggle_buzzer(self):
		self.buzzer_pub.publish()

	def bargraph(self, val):
		self.bargraph_led_pub.publish(val)

	def lcd_print(self, row, col, val):
		self.lcd_msg.row = row
		self.lcd_msg.col = col
		self.lcd_msg.val = val
		self.lcd_print_pub.publish(self.lcd_msg)

	def lcd_clear(self):
		self.lcd_clear_pub.publish()

	def callback(self, msg):
		print msg


	def testRobo(self):

		self.toggle_buzzer()
		time.sleep(4)
		self.toggle_buzzer()
		time.sleep(4)

		i = 0

		while i < 256:
			self.bargraph(i)
			i += 1
			time.sleep(0.2)

		self.bargraph(0)
		time.sleep(1)

		self.lcd_print(2,2,"Hello")
		time.sleep(4)
		self.lcd_clear()
		time.sleep(4)

		self.lcd_print(1,1,"Done")



if __name__ == '__main__':

	test = firebird()

	rate = rospy.Rate(1)
	
	while not rospy.is_shutdown():
	
		#TODO here
		test.testRobo()

		rate.sleep()
		
		rospy.spin()
