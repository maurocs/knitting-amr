#! /usr/bin/env python3
from __future__ import division

import rclpy
import sys, time, threading
from geometry_msgs.msg import Quaternion, Twist, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from tf2_ros import TransformBroadcaster
import tf_transformations

from math import sin, cos
from .utils.pose import Pose
from .utils.odometry import Odometry as Odom

class OdometryNode:

	def __init__(self):
		self.odometry = Odom()
		
		rclpy.init(args=sys.argv)
		self.node = rclpy.create_node('diff_drive_odometry')
		
		thread = threading.Thread(target=rclpy.spin, args=(self.node, ), daemon=True)
		thread.start()

		self.odomPub = self.node.create_publisher(Odometry, '/odom',10)
		self.tfPub = TransformBroadcaster(self.node)
		
		self.node.create_subscription(Int32, 'lwheel_ticks', self.leftCallback, 10)
		self.node.create_subscription(Int32, 'rwheel_ticks', self.rightCallback, 10)
		self.node.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initial_pose, 10)
		
		self.ticksPerMeter = self.node.declare_parameter('~ticks_per_meter', 5991.7155046360596995344475622593 ).value
		assert isinstance(self.ticksPerMeter, float), 'ticks_per_meter must be float'
		
		self.wheelSeparation = self.node.declare_parameter('~wheel_separation', 0.365).value
		assert isinstance(self.wheelSeparation, float), 'wheel_separation must be float'
		
		self.rate = self.node.declare_parameter('~rate', 10.0).value
		assert isinstance(self.rate, float), 'rate must be float'
		
		self.baseFrameID = self.node.declare_parameter('~base_frame_id', 'base_footprint').value
		assert isinstance(self.baseFrameID, str), 'base_frame_id must be string'
		
		self.odomFrameID = self.node.declare_parameter('~odom_frame_id', 'odom').value
		assert isinstance(self.odomFrameID, str), 'odom_frame_id must be string'
	
		self.encoderMin = self.node.declare_parameter('~encoder_min', -32768).value
		assert isinstance(self.encoderMin, int), 'encoder_min must be int'
		
		self.encoderMax = self.node.declare_parameter('~encoder_max', 32767).value
		assert isinstance(self.encoderMax, int), 'encoder_max must be int'

		self.odometry.setWheelSeparation(self.wheelSeparation)
		self.odometry.setTicksPerMeter(self.ticksPerMeter)
		self.odometry.setEncoderRange(self.encoderMin, self.encoderMax)
		self.odometry.setTime(self.node.get_clock().now().nanoseconds)

	def publish(self):
		self.odometry.updatePose(self.node.get_clock().now().nanoseconds)
		t = TransformStamped()
		now = self.node.get_clock().now().to_msg()
		pose = self.odometry.getPose()
		
		t.header.stamp = now
		t.header.frame_id = self.odomFrameID
		t.child_frame_id = self.baseFrameID
		
		t.transform.translation.x = pose.x
		t.transform.translation.y = pose.y
		t.transform.translation.z = 0.0

		q = tf_transformations.quaternion_from_euler(0, 0, pose.theta)
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]

		self.tfPub.sendTransform(t)

		odom = Odometry()
		odom.header.stamp = now
		odom.header.frame_id = self.odomFrameID
		odom.child_frame_id = self.baseFrameID
		odom.pose.pose.position.x = float(pose.x)
		odom.pose.pose.position.y = float(pose.y)
		odom.pose.pose.orientation.x = q[0]
		odom.pose.pose.orientation.y = q[1]
		odom.pose.pose.orientation.z = q[2]
		odom.pose.pose.orientation.w = q[3]
		odom.twist.twist.linear.x = float(pose.xVel)
		odom.twist.twist.angular.z = float(pose.thetaVel)
		self.odomPub.publish(odom)

	def on_initial_pose(self, msg):
		q = [msg.pose.pose.orientation.x,
			 msg.pose.pose.orientation.x,
			 msg.pose.pose.orientation.x,
			 msg.pose.pose.orientation.w]
		roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)

		pose = Pose()
		pose.x = msg.pose.pose.position.x
		pose.y = msg.pose.pose.position.y
		pose.theta = yaw
		self.odometry.setPose(pose)

	def leftCallback(self, msg):
		self.odometry.updateLeftWheel(msg.data)

	def rightCallback(self, msg):
		self.odometry.updateRightWheel(msg.data)
		
	def spin(self):
		self.node.get_logger().info("Start differential_drive_odometry")
		rate = self.node.create_rate(self.rate)
		while rclpy.ok():
			self.publish()
			rate.sleep()
		rclpy.spin()

def main():
	dif_odometry = OdometryNode()
	dif_odometry.spin()

if __name__ == '__main__':
	main()
