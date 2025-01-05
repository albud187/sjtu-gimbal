import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3


JOINT_ANGLE_TOPIC = "/gimbal_angles"

class JointTrajectoryPublisher(Node):
    def __init__(self):


        super().__init__('joint_trajectory_publisher')

        # Publisher to the joint trajectory topic
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Timer to periodically publish joint posiitons
        self.timer = self.create_timer(0.1, self.publish_joint_trajectory)  

        # Define joint names
        self.joint_names = ['ax1_joint', 'ax2_joint']

        # Example posiitons to publish
        self.positions = [0.0, -0.0]  # rad for each joint

        self.get_logger().info('Joint Trajectory Publisher Node has been started.')

        self.joint_angle_sub = self.create_subscription(Vector3, JOINT_ANGLE_TOPIC, self.joint_angle_cb, 60)
        
    def joint_angle_cb(self, msg):
        self.positions = [msg.x, msg.y]

    def publish_joint_trajectory(self):
        # Create a JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        # Create a JointTrajectoryPoint message
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 0  # Set the seconds part of the duration
        point.time_from_start.nanosec = 1_000_000  # 0.1 seconds in nanoseconds

        # Add the point to the trajectory message
        msg.points.append(point)

        # Publish the message
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Published joint posiitons: {self.positions}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node
    node = JointTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
