# Stdlib imports
import json
import time
import math

# Communication imports
from pymodbus.client.sync import ModbusTcpClient

# ROS imports
from rclpy.duration import Duration 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Float64MultiArray, Int32MultiArray, Bool

_file = open('Data/machine_positions.json')
machines = json.load(_file)

class Pose2d():
    def __init__(self, x, y, yaw) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw

    def __repr__(self) -> str:
        return (f"{self.x}, {self.y}, {self.yaw}")

class Machine():
    def __init__(self, number, tag, pose) -> None:
        # Set machine identification properties 
        self.number = number
        self.tag = tag
        self.active = True

        # Calculate IP
        mc = str(self.number).rjust(4, '0')
        channel = int(mc[:-2])
        address = str(mc[-2:]) if channel == 0 else int(mc[-2:])
        self.ip = f"172.20.{channel}.{address}"

        # Set PoseStamped
        self.pose = PoseStamped()
        self.pose.pose.position.x = float(pose[0])
        self.pose.pose.position.y = float(pose[1])
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        if pose[2] == "D":
            z_pos = -0.7071068
        elif pose[2] == "E":
            z_pos = 0.7071068
        else:
            z_pos = 0.0
        self.pose.pose.orientation.z = z_pos
        self.pose.pose.orientation.w = 1.0

        # Set variables
        self.current_gloves = 0
        self.last_glove = time.time()

        # Set Modbus data
        self.last_cnt = 0
        self.broken_yarn = False
        self.broken_y_L = False
        self.broken_y_R = False
        self.broken_bel = False
        self.on_off = False
        self.clutch = False

    def __repr__(self) -> str:
        return f"MC_{self.number}\t Pose: ({self.pose.pose.position.x},{self.pose.pose.position.y})\t Tag: {self.tag}\n" 

class RoutePlanner(Node):
    def __init__(self):
        super().__init__('route_planner_node')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_mc_idx = 0
        self.next_machine = 0
        self.mc_in_cell : list(Machine) = [] 
        self.furthest_mc = None
        self.heading = "up"        

        self.pose_2d_sub = self.create_subscription(
            Float64MultiArray,
            '/bot_2d_pose',
            self.set_pose,
            1)

        self.pick_feedback_sub = self.create_subscription(
            Int32MultiArray,
            '/picked_machine',
            self.mc_picked,
            1)

        self.next_mc_request_sub = self.create_subscription(
            Bool,
            '/machine_number_request',
            self.get_next_machine,
            1)
        
        self.next_machine_pub = self.create_publisher(
            Int32,
            '/next_machine_number',
            10
        )
    
    def setup(self, mc_dict:dict):
        for mc_name, mc in mc_dict.items():
            #print(mc["Number"])
            w = Machine(number=mc["Number"], tag=mc["Tag"],pose=mc["Position"])
            self.mc_in_cell.append(w)
            if (self.furthest_mc == None or self.distance_to_bot(w) > self.distance_to_bot(self.furthest_mc)):
                self.furthest_mc = w
        self.mc_in_cell.sort(key=lambda m:self.distance_to_bot(m), reverse=False)
        #print(self.mc_in_cell)

        self.initial_count()

        timer_period = 1.0
        self.timer_idx = 0
        self.timer = self.create_timer(timer_period, self.periodic_update)

    def periodic_update(self):
        periodic_print = [] 
        for mc in range(0,len(self.mc_in_cell)):
            if self.mc_in_cell[mc].active == False: continue
            self.update_mc_counter(mc)
            periodic_print.append(f"{mc} -> {self.mc_in_cell[mc].current_gloves}")
        print(periodic_print)
    
    def initial_count(self):
        for mc in range(0,len(self.mc_in_cell)):
            c = ModbusTcpClient(self.mc_in_cell[mc].ip)
            try:
                cur_cnt = int(c.read_holding_registers(432, 1).registers[0])
                self.mc_in_cell[mc].last_cnt = cur_cnt
                self.mc_in_cell[mc].last_glove = time.time()
                self.mc_in_cell[mc].active = True
            except Exception as e:
                #print(e)
                self.mc_in_cell[mc].active = False
            c.close()

    def update_mc_counter(self, mc):
        c = ModbusTcpClient(self.mc_in_cell[mc].ip)
        #print(f"Client of {self.mc_in_cell[mc].ip} started")
        try:
            cur_cnt = int(c.read_holding_registers(432, 1).registers[0])
            dif = cur_cnt-self.mc_in_cell[mc].last_cnt
            if dif > 0:
                self.mc_in_cell[mc].current_gloves += dif
                self.mc_in_cell[mc].last_cnt = cur_cnt
                self.mc_in_cell[mc].last_glove = time.time()
                self.mc_in_cell[mc].active = True
        except Exception as e:
            #print(e)
            self.mc_in_cell[mc].active = False
        c.close()

    def get_next_machine(self,msg):
        msg = Int32()
        msg.data = -1
        for mc in range(self.current_mc_idx,len(self.mc_in_cell)):
            if self.mc_in_cell[mc].active and self.mc_in_cell[mc].pose.pose.orientation.z<0:
                self.update_mc_counter(mc)
                score = self.get_pick_score(mc)
                print(score)
                if score >= 2:
                    msg.data = int(self.mc_in_cell[mc].number)
                    self.current_mc_idx = mc
                    break
        self.next_machine_pub.publish(msg)
        
    def mc_picked(self,msg:Int32MultiArray):
        mc=None
        for i,j in enumerate(self.mc_in_cell):
            if int(msg.data[0])==int(j.number):
                mc = i
                break
        if mc==None:return
        pick_feedback = msg.data[1]
        if pick_feedback == 1:
            c = ModbusTcpClient(self.mc_in_cell[mc].ip)
            try:
                self.mc_in_cell[mc].last_cnt = int(c.read_holding_registers(432, 1).registers[0])
                self.mc_in_cell[mc].active = True
            except:
                self.mc_in_cell[mc].active = False
            self.mc_in_cell[mc].current_gloves = 0
            c.close()

    def get_pick_score(self,mc):
        pick_score = self.mc_in_cell[mc].current_gloves + \
                            (self.distance_to_bot(self.mc_in_cell[mc])/self.distance_to_bot(self.furthest_mc)) + \
                            ((time.time() - self.mc_in_cell[mc].last_glove)/250)
        return pick_score

    def distance_to_bot(self,mc:Machine):
        try:
            return math.dist((self.current_x,self.current_y),
                            (mc.pose.pose.position.x , mc.pose.pose.position.y))
        except:
            return -1
    
    def set_pose(self,msg:Float64MultiArray):
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
    
def main(args=None):
 
  rclpy.init(args=args)
 
  route_planner_node = RoutePlanner()
  route_planner_node.setup(mc_dict=machines)
 
  try:
    rclpy.spin(route_planner_node)
  except KeyboardInterrupt:
    pass
 
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()