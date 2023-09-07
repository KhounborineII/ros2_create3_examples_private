# Based on:
# https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html

import rclpy
from rclpy.action import ActionClient
from irobot_create_msgs.action import NavigateToPosition

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from rclpy.node import Node
import math, sys, threading

class NavToActionClient(Node):
    def __init__(self, callback, namespace):
        super().__init__("Nav_To")
        self._action_client = ActionClient(self, NavigateToPosition, f'{namespace}/navigate_to_position')
        self.callback = callback
        
    def send_goal(self, achieve_goal_heading = True, goal_pose = [[0.1,0.0,0.0], [0.0,0.0,0.0,1.0]]):
        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = achieve_goal_heading
        ps = PoseStamped()
        ps.header = Header()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "test_id"
        p = Pose()
        p.position = Point()
        p.position.x, p.position.y, p.position.z = goal_pose[0]
        p.orientation = Quaternion()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = goal_pose[1]
        ps.pose = p
        goal_msg.goal_pose = ps
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected...")
        else:
            print("Goal accepted.")    
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.callback)

    def spin_thread(self):
        st = threading.Thread(target=lambda ac: rclpy.spin(ac), args=(self,))
        st.start()

def example_callback(future):
    print("Entering example_callback")
    result = future.result().result
    print("finished...", result)

def main(args=None, namespace=''):
    global finish_flag
    rclpy.init(args=args)

    action_client = NavToActionClient(example_callback, namespace)
    action_client.spin_thread()
    action_client.send_goal()
    input("Hit Enter when you are ready to send a second goal")
    action_client.send_goal()
    input("Hit Enter when you are ready to shut down")
    rclpy.shutdown()
    print("done")

if __name__ == '__main__':
    if len(sys.argv) >= 2:
        main(namespace=f'/{sys.argv[1]}')
    else:
        main()
