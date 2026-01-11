import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotSpinAround(Node):
    def __init__(self):
        super().__init__('robot_spin_around_node')
        # Cream un publisher pe topicul /cmd_vel
        # Acesta este topicul standard unde robotul asculta comenzi de miscare
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Setam un timer sa ruleze functia de miscare de 10 ori pe secunda (0.1s)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Robot Spin Around Node started!')

    def timer_callback(self):
        # Cream mesajul de tip Twist
        msg = Twist()
        
        # Setam viteza liniara (fata-spate) la 0
        msg.linear.x = 0.0
        
        # Setam viteza angulara (rotatie) la 0.5 radiani/secunda
        msg.angular.z = 0.5
        
        # Publicam mesajul catre robot
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSpinAround()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()