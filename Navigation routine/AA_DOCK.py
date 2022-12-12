#! /usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float64MultiArray, Int32, String, Bool, Int32MultiArray

class ProtMsg:
    def __init__(self) -> None:
        self.data = 0

# Holds the current pose of the robot
current_x = 0.0
current_y = 0.0
current_yaw_angle = 0.0

# Holds the pose of the current tag
current_tag_x = 0.0
current_tag_y = 0.0
current_tag_yaw_angle = 0.0

# Holds next machine on the queue
next_machine = None

# Holds the current state of the battery
this_battery_state = 0.0
prev_battery_state = 0.0

# Flag for detecting the change in the battery state
low_battery = False
low_battery_min_threshold = 0.15

# File with the current machines positions
_file = open("Data/machine_positions.json")
machines = json.load(_file)

#PoC Variables
sim_list = [0,675,674,673,672,671,670,669,668]
idx = 0
first_run = True

nav_start = 0
pick_start = 0

class PickingNavigator(Node):
    """
    Navigates and request docking routines
    """

    def __init__(self):
        super().__init__("navigate_to_machines")

        self.current_mc_tag_pub = self.create_publisher(Int32, "/current_tag", 10)

        self.pick_feedback_sub = self.create_publisher(
            Int32MultiArray, "/picked_machine", 10
        )

        self.next_mc_request_sub = self.create_publisher(
            Bool, "/machine_number_request", 10
        )

        self.start_docking_pub = self.create_publisher(Bool, "/start_docking", 10)

        self.next_machine = self.create_subscription(
            Int32, "/next_machine_number", self.set_next_machine, 10
        )

        self.picking_feedback = self.create_subscription(
            Int32MultiArray, "/pick_feedback", self.update_feedback, 10
        )

        self.request_machine_sim()

        # Execution timer
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.navigate_to_station)

        # Declare distance metrics in meters
        self.distance_goal_tolerance = 0.01
        self.reached_distance_goal = False

        # Declare angle metrics in radians
        self.heading_tolerance = 0.10
        self.yaw_goal_tolerance = 0.05

        # Picking related variables
        self.picking_feedback = [0, 0]
        self.picking = False

        self.nav_start = 0
        self.pick_start = 0

        # Navigation setup and feedback
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.done_navigating = False

        self.curr_record = [0]*4

    def set_next_machine(self, msg):
        """
        Sets the next machine in queue
        """
        global next_machine
        for mc_name, mc in machines.items():
            try:
                if mc["Number"] == msg.data:
                    next_machine = mc
            except:
                print("Machine number not found in 'machine_positions.json'")

    def update_feedback(self, msg):
        """
        Gets picking feedback from the docking node
        """
        self.picking_feedback = msg.data

    def request_machine_sim(self):
        global idx
        mc = sim_list[idx]
        idx = idx+1 if idx <= len(sim_list)-2 else 0
        msg = ProtMsg()
        msg.data = mc
        print("NEXT MACHINE: ")
        print(mc)
        self.set_next_machine(msg)

    def navigate_to_station(self):
        """
        Main routine: Goes to the selected machine and requests a docking sequence.
        Will end with "SUCCESS" flag if the docking sequence returns a "Done" value.
        Will stop with "FAIL" flag if the machine is unreachable/unconnected or if the 
        docking sequence fails.
        """
        global next_machine, first_run
        if self.done_navigating:
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                if not self.picking:
                    self.curr_record[0] = 1
                    self.curr_record[1] = time.time()-self.nav_start
                    start_picking_msg = Bool()
                    start_picking_msg.data = False if (next_machine!=None and next_machine["Number"]==0) else True
                    self.start_docking_pub.publish(start_picking_msg)
                    self.picking_feedback = [0, 0]
                    self.picking = True
                    self.pick_start = time.time()
                else:
                    if self.picking_feedback == [0, 0]:
                        print("Waiting for picking feedback")
                    else:
                        if self.picking_feedback[1] == 0: fb = "Failed"
                        elif self.picking_feedback[1] == 1: fb = "Picked"
                        elif self.picking_feedback[1] == 2: fb = "At home"
                        else: fb = "Unknown feedback code"
                        self.curr_record[2] = time.time()-self.pick_start
                        self.curr_record[3] = self.picking_feedback[1]
                        with open("records.txt", "a") as file_object:
                            file_object.write(f"\n{round(self.curr_record[0],2)},{round(self.curr_record[1],2)},{round(self.curr_record[2],2)},{round(self.curr_record[3],2)}")
                        print(fb)
                        self.picking = False
                        self.done_navigating = False
                        next_machine = None
                        self.request_machine_sim()
            elif result == TaskResult.CANCELED:
                print("Goal was canceled!")
                self.done_navigating = False
                next_machine = None
                self.request_machine_sim()
            elif result == TaskResult.FAILED:
                print("Goal failed!")
                self.done_navigating = False
                next_machine = None
                self.request_machine_sim()
            else:
                print("Goal has an invalid return status!")
                self.done_navigating = False
                next_machine = None
                self.request_machine_sim()
            return None
        
        msg_req = Bool()
        msg_req.data = True
        self.next_mc_request_sub.publish(msg_req)

        if next_machine == None:
            return None

        current_machine = next_machine
        tag_msg = Int32()
        tag_msg.data = int(current_machine["Tag"])
        self.current_mc_tag_pub.publish(tag_msg)

        self.get_logger().info("Navigate to station")

        # You may use the navigator to clear or obtain costmaps
        self.nav.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = self.nav.getGlobalCostmap()
        # local_costmap = self.nav.getLocalCostmap()

        # Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = current_machine["Position"][0]
        goal_pose.pose.position.y = current_machine["Position"][1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        if current_machine["Position"][2] == "D":
            z_pos = -0.7071068
        elif current_machine["Position"][2] == "E":
            z_pos = 0.7071068
        else:
            z_pos = 0.0
        goal_pose.pose.orientation.z = z_pos  # current_machine["Position"][2]
        goal_pose.pose.orientation.w = 0.7071068

        # Go to the goal pose
        self.nav.goToPose(goal_pose)
        self.nav_start = time.time()

        i = 0

        # Keep doing stuff as long as the robot is moving towards the goal
        while not self.nav.isTaskComplete():
            # Do something with the feedback
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    "Distance remaining: "
                    + "{:.2f}".format(feedback.distance_remaining)
                    + " meters."
                )

                # Some navigation timeout to demo cancellation
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=1800.0):
                # self.nav.cancelNav()
        self.done_navigating = True


class BatteryStateSubscriber(Node):
    """
    Subscriber node to the current battery state
    """

    def __init__(self):

        # Initialize the class using the constructor
        super().__init__("battery_state_subscriber")

        # Create a subscriber
        # This node subscribes to messages of type
        # sensor_msgs/BatteryState
        self.subscription_battery_state = self.create_subscription(
            Int32, "/battery_soc", self.get_battery_state, 10
        )

    def get_battery_state(self, msg):
        """
        Update the current battery state.
        """
        global this_battery_state
        global prev_battery_state
        global low_battery
        prev_battery_state = this_battery_state
        this_battery_state = float(msg.data) / 100

        # Check for low battery
        if (
            prev_battery_state >= low_battery_min_threshold
            and this_battery_state < low_battery_min_threshold
        ):
            low_battery = True


class PoseSubscriber(Node):
    """
    Subscriber node to the current 2D pose of the robot
    """

    def __init__(self):

        # Initialize the class using the constructor
        super().__init__("pose_subscriber")

        # Create a subscriber
        # This node subscribes to messages of type
        # std_msgs/Float64MultiArray
        self.subscription_pose = self.create_subscription(
            Float64MultiArray, "/bot_2d_pose", self.get_pose, 1
        )

        self.tag_pose_sub = self.create_subscription(
            Float64MultiArray, "/tag_2d_pose", self.get_tag_pose, 1
        )

    def get_pose(self, msg):
        global current_x
        global current_y
        global current_yaw_angle
        current_2d_pose = msg.data
        current_x = current_2d_pose[0]
        current_y = current_2d_pose[1]
        current_yaw_angle = current_2d_pose[2]

    def get_tag_pose(self, msg):
        global current_tag_x
        global current_tag_y
        global current_tag_yaw_angle
        current_2d_pose = msg.data
        current_tag_x = current_2d_pose[0]
        current_tag_y = current_2d_pose[1]
        current_tag_yaw_angle = current_2d_pose[2]


def main(args=None):
    rclpy.init(args=args)

    try:
        picking_navigator = PickingNavigator()
        battery_state_subscriber = BatteryStateSubscriber()
        pose_subscriber = PoseSubscriber()

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(picking_navigator)
        executor.add_node(battery_state_subscriber)
        executor.add_node(pose_subscriber)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            picking_navigator.destroy_node()
            battery_state_subscriber.destroy_node()
            pose_subscriber.destroy_node()

    finally:
        # Shutdown
        rclpy.shutdown()


if __name__ == "__main__":
    main()
