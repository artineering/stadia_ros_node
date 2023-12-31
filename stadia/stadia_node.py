#!/usr/bin/env python3
# FILEPATH: /home/strangeloop/ros2_ws/src/stadia/stadia/stadia_node.py

import rclpy
import evdev
import os

from rclpy.node import Node
from evdev import InputDevice, categorize, ecodes

btn_a = 304
btn_b = 305
btn_x = 307
btn_y = 308

btn_options = 314
btn_menu    = 315
btn_assist  = 704
btn_capture = 705
btn_stadia  = 316

btn_l1 = 310
btn_r1 = 311
btn_ljoy = 317
btn_rjoy = 318


class StadiaNode(Node):

    def __init__(self):
        super().__init__('stadia_node')
        self.dev = None
        self.dev_timer = self.create_timer(0.1, self.device_timer_callback)
        self.get_logger().info('Stadia Node has been started')
        self.get_logger().info('waiting for controller ...')

    def device_timer_callback(self):
        if self.dev is None:
            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
            for device in devices:
                if device.name == 'StadiaDT6S-8b46':
                    self.get_logger().info('Found Stadia Controller')
                    self.get_logger().info("Path: " + str(device.path))
                    self.get_logger().info("Name: " + str(device.name))
                    self.get_logger().info("Phys: " + str(device.phys))
                    self.dev = device
                    self.dev_timer.cancel()
                    self.isConnected = True
                    self.get_logger().info('Starting heartbeat timer ...')
                    self.heartbeat_timer = self.create_timer(30, self.is_device_connected)
                    self.get_logger().info('Listening for controller events ...')
                    self.evt_timer = self.create_timer(0.1, self.event_timer_callback)
                    break

    def is_device_connected(self):
        try:
            if os.path.exists(self.dev.path):
                self.isConnected = True
            else:
                self.isConnected = False
        except OSError:
            self.isConnected = False

        if not self.isConnected and self.dev is not None:
            self.get_logger().info('Controller disconnected')
            self.get_logger().info('Stopping heartbeat timer ...')
            self.get_logger().info('Stopping event timer ...')
            self.get_logger().info('Waiting for controller ...')
            self.heartbeat_timer.cancel()
            self.evt_timer.cancel()
            self.dev = None
            self.dev_timer = self.create_timer(1, self.device_timer_callback)
        
        return self.isConnected

    def event_timer_callback(self):
        try:
            for event in self.dev.read():
                if event.type == ecodes.EV_KEY:
                    if event.value == 1:
                        if event.code == btn_a:
                            print("A")
                        elif event.code == btn_b:
                            print("B")
                        elif event.code == btn_x:
                            print("X")
                        elif event.code == btn_y:
                            print("Y")
                        elif event.code == btn_options:
                            print("Options")
                        elif event.code == btn_menu:
                            print("Menu")
                        elif event.code == btn_assist:
                            print("Assistant")
                        elif event.code == btn_capture:
                            print("Capture")
                        elif event.code == btn_stadia:
                            print("Stadia")
                        elif event.code == btn_l1:
                            print("L1")
                        elif event.code == btn_r1:
                            print("R1")
                        elif event.code == btn_ljoy:
                            print("LJoy Button L3")
                        elif event.code == btn_rjoy:
                            print("RJoy Button R3")
                elif event.type == ecodes.EV_ABS:
                    self.get_logger().info("Axis: " + str(ecodes.ABS[event.code]))
                    self.get_logger().info("Value: " + str(event.value))
                
        except OSError:
            self.is_device_connected()

def main(args=None):
    rclpy.init(args=args)
    stadia_node = StadiaNode()
    rclpy.spin(stadia_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


