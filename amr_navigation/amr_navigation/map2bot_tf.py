#!/usr/bin/env python3 

import os
import math 
import rclpy 
from rclpy.node import Node 
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer 
from tf2_ros.transform_listener import TransformListener 
from std_msgs.msg import Float64MultiArray, String, Int32
from geometry_msgs.msg import PoseStamped



class FrameListener(Node):
  def __init__(self):
    super().__init__('map2bot_tf_listener')

    self.declare_parameter('target_frame', 'base_footprint')
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
      
    self.bot_2d_pose_pub = self.create_publisher(
      Float64MultiArray, 
      '/bot_2d_pose', 
      1)
    self.tag_2d_pose_pub = self.create_publisher(
      Float64MultiArray,
      '/tag_2d_pose',
      1
    )
    self.current_tag_sub = self.create_subscription(
      Int32,
      '/current_tag',
      self.get_current_tag,
      1
    )

    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.on_timer)

    self.current_x = 0.0 
    self.current_y = 0.0  
    self.current_yaw = 0.0
    self.current_tag = "404"

  def get_current_tag(self,msg):
    self.current_tag = f"tag_{msg.data}" 

  def on_timer(self):
    self.convert_pose(self.bot_2d_pose_pub,'map',self.target_frame)
    if self.current_tag != "404":
      self.convert_pose(self.tag_2d_pose_pub,'map',self.current_tag)

  def convert_pose(self,publisher,origin_frame,target_frame):
    from_frame_rel = target_frame
    to_frame_rel = origin_frame
  
    trans = None
    
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform(
                  to_frame_rel,
                  from_frame_rel,
                  now)

    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return
    
    self.current_x = trans.transform.translation.x
    self.current_y = trans.transform.translation.y    
    roll, pitch, yaw = self.euler_from_quaternion(
      trans.transform.rotation.x,
      trans.transform.rotation.y,
      trans.transform.rotation.z,
      trans.transform.rotation.w)      
    self.current_yaw = yaw    
    msg = Float64MultiArray()
    msg.data = [self.current_x, self.current_y, self.current_yaw]
    publisher.publish(msg)

  
  def euler_from_quaternion(self, x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

def main(args=None):
 
  rclpy.init(args=args)
 
  frame_listener_node = FrameListener()
 
  try:
    rclpy.spin(frame_listener_node)
  except KeyboardInterrupt:
    pass
 
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()