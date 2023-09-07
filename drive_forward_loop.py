import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import InterfaceButtons
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from ForwardActionClient import ForwardActionClient


class ForwardNode(runner.HdxNode):
    def __init__(self, namespace: str = "", distance = 0.1):
        super().__init__('forward_node')
        self.buttons = self.create_subscription(InterfaceButtons, namespace + '/interface_buttons', self.button_callback, qos_profile_sensor_data)
        self.distance = distance
        self.driver = ForwardActionClient(self.move_finished_callback, namespace)
        self.pressed = False
        self.driving = False
        self.started = False

    def has_started(self):
        return self.started

    def button_clear(self): 
        return self.pressed == False

    def is_driving(self):
        return self.driving

    def button_callback(self, msg):
        self.record_first_callback()
        self.started = True
        if self.pressed == False and not self.driving:
            if msg.button_1.is_pressed or msg.button_2.is_pressed:
                self.pressed = True
            if self.pressed == True:
                print(f"Detected {self.pressed}")    
            
    def start_move(self):
        goal = self.distance
        self.driver.send_goal(goal)
        self.driving = True
        print(f"Sending goal: {goal}")

    def move_finished_callback(self, future):
        print("ForwardNode: Move finished")
        self.pressed = False
        self.driving = False

    def add_self_recursive(self, executor):
        executor.add_node(self)
        executor.add_node(self.driver)


class ForwardBot(runner.WheelMonitorNode):
    def __init__(self, namespace: str = ""):
        super().__init__('forward_bot', namespace)
        self.forward_node = ForwardNode(namespace)
        self.create_timer(0.10, self.timer_callback)

    def timer_callback(self):
        self.record_first_callback()
        if self.forward_node.has_started() and not self.forward_node.is_driving():
            if self.wheels_stopped() and not self.forward_node.button_clear():
                self.forward_node.start_move()
                
    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.forward_node.add_self_recursive(executor)


if __name__ == '__main__':
    rclpy.init()
    bot = ForwardBot(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
