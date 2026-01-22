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
        human_frame = 'person_ioana' 
        robot_frame = 'base_link'

        try:
            # Incercam sa aflam unde e omul fata de robot(Calculam vectorul de la Robot -> Om)
            t = self.tf_buffer.lookup_transform(
                robot_frame,
                human_frame,
                rclpy.time.Time())
            
	    #Daca ajungem aici, OMUL ESTE VIZIBIL
	    #Extragem coordonatele 
            x_h = t.transform.translation.x
            y_h = t.transform.translation.y
            
            # Rotim robotul spre om
            msg_vel = Twist()
            angle_to_turn = math.atan2(y_h, x_h)
            msg_vel.angular.z = 1.0 * angle_to_turn
            self.publisher_.publish(msg_vel)

           
            # Calculam vectorul invers: Om -> Robot
            # Asta ne spune unde e robotul "din punctul de vedere al omului"
            t_engage = self.tf_buffer.lookup_transform(
                human_frame,
                robot_frame,
                rclpy.time.Time())
            
            rx = t_engage.transform.translation.x # Distanta in fata omului
            ry = t_engage.transform.translation.y # Distanta laterala (stanga/dreapta)

                        
            # Conditiile pentru Engagement:
            # 1. rx > 0: Robotul e in FAȚA ta (nu la spate)
            # 2. rx < 3.0: Robotul e la mai putin de 3 metri
            # 3. abs(ry) < 0.5: Robotul e centrat pe directia privirii tale
            if rx > 0.0 and rx < 3.0 and abs(ry) < 0.5:
                self.get_logger().info("Engaged: Hello Ioana!")
            else:
                self.get_logger().info("Not Engaged")
            
        except TransformException as ex:
            # Daca nu te vede, te cauta
            msg_vel = Twist()
            msg_vel.angular.z = 0.5
            self.publisher_.publish(msg_vel)
            self.get_logger().info('Caut_omul...')

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
