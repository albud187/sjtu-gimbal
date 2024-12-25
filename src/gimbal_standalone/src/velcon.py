#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ArmActionClient(Node):

    def __init__(self):
        super().__init__('arm_control_actionclient')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self, v1, v2):
        goal_msg = FollowJointTrajectory.Goal()

        joint_names = ["ax1_joint", "ax2_joint"]

        points = []
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=5, nanoseconds=0).to_msg()
        point.positions = [0.0, 0.0]
        point.velocities = [v1, v2]

        points.append(point)

        goal_msg.goal_time_tolerance = Duration(seconds=5, nanoseconds=0.1).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    

def main(args=None):
    
    rclpy.init()

    action_client = ArmActionClient()

    v1 = float(sys.argv[1])
    v2 = float(sys.argv[2])
    future = action_client.send_goal(v1, v2)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
