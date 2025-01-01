import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty, Bool, String
import threading
import sys
import termios
import tty


"""
w/s = accelerate translation forwards/backwards
a/d = accelerate translation left/right
i/k = accelerate translation up/down
j/l = accelerate angular rotation left/right
q = reset velocity to 0

e = manual flight mode
r = tracking mode
t = chasing mode
"""

LIN_VEL_INCREMENT = 0.1
ANG_VEL_INCREMENT = 0.1
FLIGHT_MODES = {'e':"MAN_FLT", 'r':"TRACK"}

T_flight_mode = "/sjtu_drone/mode"
T_cmd_vel = "/sjtu_drone/cmd_vel"
class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, T_cmd_vel, 120)
        self.mode_str_publisher = self.create_publisher(String, T_flight_mode, 120)
        # Start a separate thread to listen to keyboard inputs
        self.input_thread = threading.Thread(target=self.read_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def read_keyboard_input(self) -> None:

        cmd_vel = Twist()
        flight_mode = String()
        flight_mode.data = "e"
        while rclpy.ok():
            
            vel_report_str = f"vx, vy, vz, rz :  {str(round(cmd_vel.linear.x, 2))},  {str(round(cmd_vel.linear.y, 2))}, {str(round(cmd_vel.linear.z, 2))}, {str(round(cmd_vel.angular.z, 2))}"
            print(vel_report_str)
            print(" ")
            # Implement a non-blocking keyboard read
            key = self.get_key()
            
            if key.lower() == 'q':
                # zero
                cmd_vel = Twist()
                self.cmd_vel_publisher.publish(cmd_vel)
            elif key.lower() == 'w':
                # Move forward
                cmd_vel.linear.x = cmd_vel.linear.x + LIN_VEL_INCREMENT 
                self.cmd_vel_publisher.publish(cmd_vel)
            elif key.lower() == 's':
                #move backward
                cmd_vel.linear.x = cmd_vel.linear.x - LIN_VEL_INCREMENT 
                self.cmd_vel_publisher.publish(cmd_vel)
            elif key.lower() == 'a':
                # Move Left
                cmd_vel.linear.y = cmd_vel.linear.y + LIN_VEL_INCREMENT
                self.cmd_vel_publisher.publish(cmd_vel)
            elif key.lower() == 'd':
                # Move right
                cmd_vel.linear.y =  cmd_vel.linear.y - LIN_VEL_INCREMENT
                self.cmd_vel_publisher.publish(cmd_vel)
            if key.lower() == 'i':
                # Move up
                cmd_vel.linear.z =  cmd_vel.linear.z + LIN_VEL_INCREMENT 
                self.cmd_vel_publisher.publish(cmd_vel)
            elif key.lower() == 'k':
                #move down
                cmd_vel.linear.z =  cmd_vel.linear.z - LIN_VEL_INCREMENT 
                self.cmd_vel_publisher.publish(cmd_vel)
            elif key.lower() == 'j':
                # rotate left
                cmd_vel.angular.z = cmd_vel.angular.z + ANG_VEL_INCREMENT
                self.cmd_vel_publisher.publish(cmd_vel)
            elif key.lower() == 'l':
                # rotate right
                cmd_vel.angular.z = cmd_vel.angular.z - ANG_VEL_INCREMENT
                self.cmd_vel_publisher.publish(cmd_vel)

            elif key.lower() in list(FLIGHT_MODES.keys()): 
                flight_mode.data = FLIGHT_MODES[key]
                self.mode_str_publisher.publish(flight_mode)
            elif key == 'p':
                print("EXITING)")
                exit()           
            
    def get_key(self) -> str:
        """
        Function to capture keyboard input
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()