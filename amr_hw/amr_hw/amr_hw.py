#!/usr/bin/env python3

import rclpy
import sys
import threading
from serial import Serial
import re

from std_msgs.msg import Int32, Float32, Bool

class SerialController:
    def __init__(self, port="/dev/ttyS0", baudrate=115200):
        self.ser = Serial(port, baudrate=baudrate)
        if self.ser.isOpen(): self.ser.close()
        self.ser.open()
        self.ser.flushInput()
        self.ser.flushOutput()
        self.enc_L = 0
        self.enc_R = 0
        self.soc = 0
        self.state = 0
        self.picking = False

    def send(self, tx):
        self.ser.write(tx)

    def receive(self):
        data=""
        data = self.ser.readline().decode()
        return data

    def refresh(self, l_spd, r_spd, out):
        if l_spd>999:
            lspd_normalized = "L0999"
        elif l_spd<-999:
            lspd_normalized = "L-999"
        else:
            lspd_normalized = f"L{str(l_spd).zfill(4)}"

        if r_spd>999:
            rspd_normalized = "R0999"
        elif r_spd<-999:
            rspd_normalized = "R-999"
        else:
            rspd_normalized = f"R{str(r_spd).zfill(4)}"

        self.send(lspd_normalized.encode())
        self.send(rspd_normalized.encode())
        self.send("E0000".encode())
        self.send("S0000".encode())
        self.send(f"O{str(out).zfill(4)}".encode())
        self.send("T0".encode())
        self.send(">".encode())

        msg = self.receive()
        self.enc_L = int(re.search("L-*[0-9]*",msg).group()[1:])
        self.enc_R = int(re.search("R-*[0-9]*",msg).group()[1:])
        self.soc   = int(re.search("C-*[0-9]*",msg).group()[1:])
        self.state = int(re.search("S-*[0-9]*",msg).group()[1:])

        self.picking = True if (self.state == 100) else False

        return (f"{lspd_normalized},{rspd_normalized},E0000,S0000,O{str(out).zfill(4)},T0>")

class HardwareController:
    def __init__(self):
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('hardware_control_node')

        thread = threading.Thread(target=rclpy.spin, args=(self.node, ), daemon=True)
        thread.start()

        self.lwheel_ticks_pub = self.node.create_publisher(Int32, 'lwheel_ticks',10)
        self.rwheel_ticks_pub = self.node.create_publisher(Int32, 'rwheel_ticks',10)
        self.charge_pub = self.node.create_publisher(Int32, 'battery_soc',10)

        self.lwheel_des_sub = self.node.create_subscription(Float32, 'lwheel_desired_speed', self.leftCallback, 10)
        self.rwheel_des_sub = self.node.create_subscription(Float32, 'rwheel_desired_speed', self.rightCallback, 10)
        self.blade_sub = self.node.create_subscription(Bool, 'pick_gloves', self.pickCallback, 10)

        self.wheelDiameter = self.node.declare_parameter('wheel_diameter', 0.170).value
        assert isinstance(self.wheelDiameter, float), 'diameter must be float'

        self.rate = self.node.declare_parameter('rate', 30).value
        assert isinstance(self.rate, int), 'rate must be int'

        self.speed_mult_factor = self.node.declare_parameter('speed_mult', 50.0).value
        assert isinstance(self.speed_mult_factor, float), 'Speed multiplier factor must be float'

        self.lwheel_ticks = 0
        self.rwheel_ticks = 0
        self.battery_soc = 0

        self.lwheel_cmd = 0
        self.rwheel_cmd = 0

        self.leftSpeed = 0
        self.rightSpeed = 0
        self.outputs = 0


        self.ctrl = SerialController()
        self.node.get_logger().info("Establishing connection with motor controller...")

        while not self.ctrl.ser.isOpen():
            self.node.get_logger().info("Connection failed, retrying...")
            self.ctrl = SerialController()
        
        self.node.get_logger().info(f"Connection to motor controller successful")

    def update_ticks(self):
        if (self.ctrl.picking):
            pass
            self.ctrl.send(">".encode())
        else:
            self.ctrl.ser.flushOutput()
            out_str = self.ctrl.refresh(self.leftSpeed,self.rightSpeed,self.outputs)
            
        self.node.get_logger().info(f"{out_str}")

        self.lwheel_ticks = self.ctrl.enc_L
        self.rwheel_ticks = self.ctrl.enc_R
        self.battery_soc = self.ctrl.soc

        Lmsg = Int32()
        Rmsg = Int32()
        Vmsg = Int32()

        Lmsg.data = self.lwheel_ticks
        Rmsg.data = self.rwheel_ticks
        Vmsg.data = self.battery_soc

        self.lwheel_ticks_pub.publish(Lmsg)
        self.rwheel_ticks_pub.publish(Rmsg)
        self.charge_pub.publish(Vmsg)
    
    def leftCallback(self, leftSpeed):
        #print(leftSpeed.data)
        try:
            self.leftSpeed = int(leftSpeed.data * self.speed_mult_factor)
        except:
            self.leftSpeed = 0

    def rightCallback(self, rightSpeed):
        #print(rightSpeed.data)
        try:
            self.rightSpeed = int(rightSpeed.data * self.speed_mult_factor)
        except:
            self.rightSpeed = 0

    def pickCallback(self, msg):
        if msg.data:
            self.outputs = 1
        else:
            self.outputs = 0

    def handshake(self):
        connected = False
        print("Waiting for uC")
        while not connected:
            msg = self.ctrl.receive()
            if msg!="L0R0C0S0":
                print(msg)
                self.ctrl.send("L0000".encode())
                self.ctrl.send("R0000".encode())
                self.ctrl.send("E0000".encode())
                self.ctrl.send("S0001".encode())
                self.ctrl.send("O0000".encode())
                self.ctrl.send("T0".encode())
                self.ctrl.send(">".encode())
                connected = True

    def spin(self):
        self.node.get_logger().info("Starting motor controller")
        rate = self.node.create_rate(self.rate)
        #self.handshake()
        while rclpy.ok():
            self.update_ticks()
            rate.sleep()
        rclpy.spin(self)

    def shutdown(self):
        self.node.get_logger().info("Stoping motor controller")
        Lmsg = Int32()
        Rmsg = Int32()

        Lmsg.data = 0
        Rmsg.data = 0

        self.lwheel_ticks_pub.publish(Lmsg)
        self.rwheel_ticks_pub.publish(Rmsg)

def main():
    controller = HardwareController()
    rclpy.get_default_context().on_shutdown(controller.shutdown)
    controller.spin()

if __name__ == "__main__":
    main()