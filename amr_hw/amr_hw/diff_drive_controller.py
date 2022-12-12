#! /usr/bin/env python3

import rclpy
import sys, threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ControllerNode:
	def __init__(self):
		self.linearVelocity = 0.0
		self.angularVelocity = 0.0
		
		rclpy.init(args=sys.argv)
		self.node = rclpy.create_node('diff_drive_controller')
		
		thread = threading.Thread(target=rclpy.spin, args=(self.node, ), daemon=True)
		thread.start()
		
		self.leftPub = self.node.create_publisher(Float32, 'lwheel_desired_speed',10)
		self.rightPub = self.node.create_publisher(Float32, 'rwheel_desired_speed',10)

		self.cmd_vel_sub = self.node.create_subscription(Twist, 'cmd_vel', self.twistCallback, 10)
		
		self.wheelSeparation = self.node.declare_parameter('wheel_separation', 0.365).value
		assert isinstance(self.wheelSeparation, float), 'wheel_separation must be float'
		
		self.maxMotorSpeed = self.node.declare_parameter('max_motor_speed', 2.0).value
		assert isinstance(self.maxMotorSpeed, float), 'maximum speed must be float'
		
		self.rate = self.node.declare_parameter('rate', 10.0).value
		assert isinstance(self.rate, float), 'rate must be float'
		
		self.timeout = (1000000000) * self.node.declare_parameter('timeout', 1.0).value
		assert isinstance(self.timeout, float), 'timeout must be float'	

	def publish(self):
		left_speed = Float32()
		right_speed = Float32()
		
		if self.node.get_clock().now().nanoseconds - self.lastTwistTime < self.timeout:            
			left_speed.data = 1.0 * self.linearVelocity - self.angularVelocity * self.wheelSeparation / 2
			right_speed.data = 1.0 * self.linearVelocity + self.angularVelocity * self.wheelSeparation / 2
		  
		else:
			left_speed.data = 0.0
			right_speed.data = 0.0
		
		self.leftPub.publish(left_speed)
		self.rightPub.publish(right_speed)

	def twistCallback(self, twist):
		self.linearVelocity = twist.linear.x
		self.angularVelocity = twist.angular.z
		self.lastTwistTime = self.node.get_clock().now().nanoseconds
		
	def spin(self):
		self.node.get_logger().info("Start differential_drive_controller")
		self.lastTwistTime = self.node.get_clock().now().nanoseconds
		rate = self.node.create_rate(self.rate)
		while rclpy.ok():
			self.publish()
			rate.sleep()
		rclpy.spin();

def main():
	dif_controller = ControllerNode()
	dif_controller.spin()
	

if __name__ == '__main__':
	main()
	
