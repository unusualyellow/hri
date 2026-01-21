import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower_node')
        
        # Publisher pentru miscare
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Setup pentru TF (Sistemul care detecteaza pozitia)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer de control (ruleaza de 10 ori pe secunda)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Human Follower a pornit!')

    def control_loop(self):
        msg = Twist()
        
        # Numele frame-urilor
        # officebot_base_link = robotul
        # human/body_0/base_link = omul 
        from_frame_rel = 'person_ioana' 
        to_frame_rel = 'base_link'

        try:
            # 1. Incercam sa aflam unde e omul fata de robot
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            # Daca ajungem aici, OMUL ESTE VIZIBIL
            # Extragem coordonatele
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Calculam unghiul folosind atan2
            angle_to_turn = math.atan2(y, x)
            
            # Regula de 3 simpla (Proportional Controller):
            # Daca unghiul e mare -> rotim repede. Daca e mic -> rotim incet.
            msg.angular.z = 1.0 * angle_to_turn 
            msg.linear.x = 0.0 # Stam pe loc, doar ne rotim
            
            self.get_logger().info(f'Om detectat la unghiul: {angle_to_turn:.2f}')

        except TransformException as ex:
            # 2. Daca omul NU e vizibil (sau apare o eroare de transformare)
            # Robotul intra in modul SEEK (Cauta) -> se roteste constant
            msg.angular.z = 0.5
            msg.linear.x = 0.0
            self.get_logger().info('Caut omul...')

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
