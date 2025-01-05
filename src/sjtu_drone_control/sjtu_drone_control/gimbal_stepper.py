import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

T_GIMBAL_STEP = "/gimbal_step"
T_JOINT_STATE = "/joint_states"
T_GIMBAL_STATE= "/gimbal_state"
T_JOINT_TRAJECTORY = "/joint_trajectory_controller/joint_trajectory"
class JointStepperNode(Node):
    def __init__(self):

        super().__init__('gimbal_stepper')

        self.gimbal_state = Vector3()
        self.gimbal_step = Vector3()

        self.get_logger().info('Joint Stepper Node')

        self.joint_state_sub = self.create_subscription(JointState, T_JOINT_STATE, self.gimbal_state_cb, 60)
        self.gimbal_state_pub = self.create_publisher(Vector3, T_GIMBAL_STATE, 60)

        #gimbal stepper
        self.joint_names = ['ax1_joint', 'ax2_joint']
        self.joint_angle_sub = self.create_subscription(Vector3, T_GIMBAL_STEP, self.gimbal_step_cb, 60)
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, T_JOINT_TRAJECTORY, 60)
        
       

    def gimbal_state_cb(self, msg):
        self.gimbal_state.x = msg.position[0]
        self.gimbal_state.y = msg.position[1]
        self.gimbal_state_pub.publish(self.gimbal_state)
        #print(self.gimbal_state)

    def gimbal_step_cb(self, msg):
        self.gimbal_step = msg
        self.do_gimbal_step()

    def do_gimbal_step(self):
        # Create a JointTrajectory message
        
        goal_state = [self.gimbal_state.x + float(self.gimbal_step.x), self.gimbal_state.y + float(self.gimbal_step.y)]
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = goal_state
       
        point.time_from_start.sec = 0  # Set the seconds part of the duration
        point.time_from_start.nanosec = 1_000_000  # 0.1 seconds in nanoseconds

        # Add the point to the trajectory message
        msg.points.append(point)

        # Publish the message
        self.joint_trajectory_pub.publish(msg)
        self.get_logger().info(f'Published joint posiitons: {point.positions}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node
    node = JointStepperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
