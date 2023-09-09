import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector, WheelStatus, WheelTicks
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from action_demo import RotateActionClient


class PatrollerBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('patrol_bot')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.wheel_status = self.create_subscription(WheelStatus, f'{namespace}/wheel_status', self.wheel_status_callback, qos_profile_sensor_data)
        self.location = self.create_subscription(Odometry, namespace + '/odom', self.odom_callback, qos_profile_sensor_data)
        self.bump = None
        self.last_wheel_status = None
        self.rotator = RotateActionClient(self.turn_finished_callback, namespace)
        self.true_loc = 0.0

    def wheels_stopped(self):
        return self.last_wheel_status is not None and self.last_wheel_status.current_ma_left == 0 and self.last_wheel_status.current_ma_right == 0

    def odom_callback(self, msg):
        loc = msg.pose.pose.position.y
        if self.true_loc == 0.0:
            self.true_loc = loc
        act_loc = loc - self.true_loc
        print(f"position: {act_loc}")
        if act_loc > 1.0 or act_loc < 0.0:
            self.publisher.publish(runner.straight_twist(0.0))
        elif act_loc < 1.0 and act_loc > 0.0:
            self.publisher.publish(runner.straight_twist(0.5))
        elif self.wheels_stopped():
            goal = math.pi
            print("Starting turn", goal)
            self.rotator.send_goal(goal)
            rclpy.spin_once(self.rotator)
        else:
            print("waiting on wheels")

    def turn_finished_callback(self, future):
        self.bump = None
        print("finished with turn")

    def wheel_status_callback(self, msg):
        self.record_first_callback()
        self.last_wheel_status = msg


if __name__ == '__main__':
    runner.run_single_node(lambda: PatrollerBot(f'/{sys.argv[1]}'))
