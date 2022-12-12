from enum import Enum
import time
import math
from simple_pid import PID
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool, Int32MultiArray
from rclpy.executors import MultiThreadedExecutor

class SequenceState(Enum):
    IDLE = 0
    PERPENDICULAR = 1
    ADJUSTING_X = 2
    PARALLEL = 3
    BREATHING = 4
    FORWARD = 5
    MINI_ADJ = 6
    DOCKED = 7
    PICKING = 8
    BACKING = 9
    RETRY = 10

class PseudoCmdVel:
    def __init__(self) -> None:
        self.linear = 0.0
        self.angular = 0.0
        
    def send(self):
        print(f"Cmd: {self.linear},{self.angular}")

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        self.last_bot_update = time.time()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_tag_update = time.time()
        self.tag_x = 0.0
        self.tag_y = 0.0
        self.tag_th = 0.0
        
        self.seq_step = SequenceState.IDLE
        
        self.timer = 0.01
        
        self.distance_goal_tolerance = 0.01
        self.heading_tolerance = 0.01
        self.yaw_goal_tolerance = 0.1
        
        self.linear_velocity = 0.1
        self.angular_velocity = 1.0

        self.last_angular_cmd = [time.time(),0.0]
        self.last_linear_prog = [time.time(),0.0]

        self.allowed_retries = 1
        self.current_retries = 0

        self.pause = [time.time(),SequenceState.IDLE,0.0]

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10)
        
        self.picker_pub = self.create_publisher(
            Bool,
            "/pick_gloves",
            10)

        self.picking_feedback = self.create_publisher(
            Int32MultiArray,
            '/pick_feedback',
            10)

        self.bot_pose_sub = self.create_subscription(
            Float64MultiArray,
            '/bot_2d_pose',
            self.update_bot_pose,
            1)

        self.tag_pose_sub = self.create_subscription(
            Float64MultiArray,
            '/tag_2d_pose',
            self.update_tag_pose,
            1)
        
        self.start_docking_sub = self.create_subscription(
            Bool,
            '/start_docking',
            self.start_docking,
            1)

        self.timer = self.create_timer(0.001,self.docking_sequence)

    def map_value(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def docking_sequence(self):
        current_time = time.time()
        """
        if (current_time-self.last_bot_update>2 or current_time-self.last_bot_update>2):
            print("Aborting for lack of TF transforms")
            self.send_feedback()"""
        
        dist_to_tag = self.get_distance_to_goal(self.x,self.y,self.tag_x,self.tag_y)
        head_to_tag = self.get_heading_error(self.x,self.y,self.tag_x,self.tag_y,self.th)
        rads_to_tag = self.get_radians_to_goal(self.th,-self.tag_th)
        x_dist = self.get_ortho_distance(self.x,self.tag_x)
        y_dist = self.get_ortho_distance(self.y,self.tag_y)

        if (math.fabs(x_dist)<=0.1 and self.seq_step == SequenceState.PERPENDICULAR):
            print("Skipping prealignment")
            self.seq_step=SequenceState.PARALLEL
        
        cmd_vel = Twist()

        if self.seq_step == SequenceState.PERPENDICULAR:
            #print(f"Y Dist: {y_dist}")
            if math.fabs(self.last_linear_prog[0]-time.time())>1.0:
                print(f"Last progress: {self.last_linear_prog}")
                delta = y_dist-self.last_linear_prog[1]
                print(delta)
                if math.fabs(delta)<0.015:
                    cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.seq_step = SequenceState.RETRY
                self.last_linear_prog = [time.time(),y_dist]

            if (math.fabs(y_dist) > 0.39):
                if (math.fabs(head_to_tag) > 0.1):
                    if head_to_tag > 0:
                        cmd_vel.angular.z = head_to_tag*2 if head_to_tag < 0.12 else 0.4
                    elif head_to_tag < 0:
                        cmd_vel.angular.z = -head_to_tag*2 if head_to_tag > 0.12 else -0.4
                else:
                    cmd_vel.angular.z = 0.0
                cmd_vel.linear.x=self.linear_velocity/1.3
            else:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                self.seq_step = SequenceState.BREATHING
            self.cmd_vel_pub.publish(cmd_vel)
        
        if self.seq_step == SequenceState.PERPENDICULAR:
            print("Perpendicularing")
            print(f"Error: {rads_to_tag}")
            if round(self.th,3) > 0:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,-self.angular_velocity
            elif round(self.th,3) < 0-self.yaw_goal_tolerance:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,self.angular_velocity
            else:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                self.seq_step = SequenceState.ADJUSTING_X
            self.cmd_vel_pub.publish(cmd_vel)

            
        if self.seq_step == SequenceState.ADJUSTING_X:
            print("Adjusting X")
            print(f"Error: {x_dist}")
            if  x_dist > self.distance_goal_tolerance:
                cmd_vel.linear.x,cmd_vel.angular.z = self.linear_velocity*1.0,0.0
            elif  x_dist < -self.distance_goal_tolerance:
                cmd_vel.linear.x,cmd_vel.angular.z = -self.linear_velocity*1.0,0.0
            else:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                self.seq_step = SequenceState.PARALLEL
            self.cmd_vel_pub.publish(cmd_vel)
            
        if self.seq_step == SequenceState.PARALLEL:
            print("Paralleling")
            print(f"Error: {rads_to_tag}")
            if self.th+math.pi > math.pi/2+self.yaw_goal_tolerance:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,-self.angular_velocity
            elif self.th+math.pi < math.pi/2-self.yaw_goal_tolerance:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,self.angular_velocity
            else:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                self.seq_step = SequenceState.FORWARD
            self.cmd_vel_pub.publish(cmd_vel)

        if self.seq_step == SequenceState.FORWARD:
            #print(f"Y Dist: {y_dist}")
            if math.fabs(self.last_linear_prog[0]-time.time())>1.0:
                print(f"Last progress: {self.last_linear_prog}")
                delta = y_dist-self.last_linear_prog[1]
                print(delta)
                if math.fabs(delta)<0.015:
                    cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.seq_step = SequenceState.RETRY
                self.last_linear_prog = [time.time(),y_dist]

            if (math.fabs(y_dist) > 0.39):
                if (math.fabs(head_to_tag) > 0.1):
                    if head_to_tag > 0:
                        cmd_vel.angular.z = head_to_tag*2 if head_to_tag < 0.12 else 0.4
                    elif head_to_tag < 0:
                        cmd_vel.angular.z = -head_to_tag*2 if head_to_tag > 0.12 else -0.4
                else:
                    cmd_vel.angular.z = 0.0
                cmd_vel.linear.x=self.linear_velocity/1.3
            else:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                self.pause = [time.time(),SequenceState.MINI_ADJ,1.0]
                self.seq_step = SequenceState.BREATHING
            self.cmd_vel_pub.publish(cmd_vel)

        if self.seq_step == SequenceState.MINI_ADJ:
            if (time.time()-self.last_angular_cmd[0] <= 0.35):
                return
            else:
                if self.last_angular_cmd[1] == 0.0:
                    print(f"MiniAdj error: {round(head_to_tag,1)}")
                    p = 0.95
                    if round(head_to_tag,1) > 0:
                        cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.4
                    elif round(head_to_tag,1) < 0:
                        cmd_vel.linear.x,cmd_vel.angular.z = 0.0,-0.4
                    else:
                        cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                        self.seq_step = SequenceState.DOCKED
                else:
                    cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
            self.last_angular_cmd = [time.time(),cmd_vel.angular.z]
            self.cmd_vel_pub.publish(cmd_vel)
        
        if self.seq_step == SequenceState.DOCKED:
            cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
            self.cmd_vel_pub.publish(cmd_vel)
            print("DOCKED!")
            self.pause = [time.time(),SequenceState.PICKING,1.0]
            self.seq_step = SequenceState.PICKING

        if self.seq_step == SequenceState.PICKING:
            pick_msg = Bool()
            pick_msg.data = True
            if time.time()-self.pause[0] > 0.2:
                pick_msg.data = False
                self.pause = [time.time(),SequenceState.BACKING,0.3]
                self.seq_step = SequenceState.BREATHING
            self.picker_pub.publish(pick_msg)
        
        if self.seq_step == SequenceState.BACKING:
            if (math.fabs(y_dist) < 0.7):
                cmd_vel.angular.z = 0.0
                cmd_vel.linear.x=-self.linear_velocity/1
            else:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                self.seq_step = SequenceState.IDLE
                self.current_retries = 0
                print("Done")
                self.send_feedback([1,1])
                self.tag_x = None
                self.tag_y = None
                self.tag_th = None
            self.cmd_vel_pub.publish(cmd_vel)

        if self.seq_step == SequenceState.BREATHING:
            if time.time()-self.pause[0] <= self.pause[2]:
                return
            else:
                self.seq_step = self.pause[1]
                self.pause = [time.time(),SequenceState.IDLE,1.0]

        if self.seq_step == SequenceState.RETRY:
            print("Retry requested")
            if (math.fabs(y_dist) < 0.65):
                cmd_vel.angular.z = 0.0
                cmd_vel.linear.x=-self.linear_velocity/1
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                cmd_vel.linear.x,cmd_vel.angular.z = 0.0,0.0
                self.cmd_vel_pub.publish(cmd_vel)
                if self.current_retries < self.allowed_retries:
                    self.current_retries += 1
                    self.seq_step = SequenceState.PERPENDICULAR
                else:
                    self.current_retries = 0
                    print("Failed")
                    self.send_feedback([1,0])
                    self.tag_x = None
                    self.tag_y = None
                    self.tag_th = None
                    self.seq_step = SequenceState.IDLE

    def get_ortho_distance(self,p1,p2):
        return p2-p1
            
    def get_distance_to_goal(self,x1,y1,x2,y2):
        distance_to_goal = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
        return distance_to_goal
         
    def get_heading_error(self,x1,y1,x2,y2,th1):
        delta_x = x2 - x1
        delta_y = y2 - y1
        desired_heading = math.atan2(delta_y, delta_x) 
        heading_error = desired_heading - th1
        if (heading_error > math.pi):
            heading_error = heading_error - (2 * math.pi)
        if (heading_error < -math.pi):
            heading_error = heading_error + (2 * math.pi)
        return heading_error
       
    def get_radians_to_goal(self,th1,th2):
        yaw_goal_angle_error = th1 - th2
        return yaw_goal_angle_error
            
    def start_docking(self,msg):
        if msg.data: 
            if self.tag_x == None:
                self.send_feedback([1,0])
            else:
                self.seq_step = SequenceState.PERPENDICULAR
        else:
            time.sleep(20)
            self.send_feedback([1,2])
        
    def send_feedback(self,fb):
        feedback_msg = Int32MultiArray()
        feedback_msg.data = fb
        self.picking_feedback.publish(feedback_msg)

    def update_bot_pose(self,msg:Float64MultiArray):
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.th = msg.data[2]
    
    def update_tag_pose(self,msg:Float64MultiArray):
        self.tag_x = msg.data[0]
        self.tag_y = msg.data[1]
        self.tag_th = msg.data[2]

def main(args=None):
    rclpy.init(args=args)

    try:
        docking_node = DockingNode()
        executor = MultiThreadedExecutor(num_threads=1)
        executor.add_node(docking_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            docking_node.destroy_node()

    finally:
        # Shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()