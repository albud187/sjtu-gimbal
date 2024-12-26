import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3


JOINT_STATE_TOPIC = "/joint_states"

class JointTrajectoryPublisher(Node):
    def __init__(self):


        super().__init__('joint_trajectory_publisher')

        # Publisher to the joint trajectory topic
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Timer to periodically publish joint velocities
        self.timer = self.create_timer(0.1, self.publish_joint_trajectory)  # 10 Hz

        # Define joint names
        self.joint_names = ['ax1_joint', 'ax2_joint']

        # Example velocities to publish
        self.positions = [0.0, -0.0]  # rad for each joint

        self.get_logger().info('Joint Trajectory Publisher Node has been started.')

        self.joint_angle_sub = self.create_subscription(Vector3, JOINT_STATE_TOPIC, self.joint_state_cb, 60)
        
    def joint_state_cb(self, msg):
        self.positions = [msg.x, msg.y]



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
