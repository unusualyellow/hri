import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class GestureControl(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        
        # Trimitem comenzi de miscare la robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        topic_name = '/humans/bodies/iszed/joint_states' 
        
        self.subscription = self.create_subscription(
            JointState,
            topic_name,
            self.listener_callback,
            10)
            
        self.get_logger().info(f'Ascultam la: {topic_name}')
        self.get_logger().info('Ridica mana DREAPTA ca sa pornesti robotul!')

    def listener_callback(self, msg):
        try:
            # Gasim indecsii pentru mana dreapta si umarul drept
            # Numele standard in ROS4HRI sunt "r_wrist" si "r_shoulder"
            if 'r_wrist' in msg.name and 'r_shoulder' in msg.name:
                idx_wrist = msg.name.index('r_wrist')
                idx_shoulder = msg.name.index('r_shoulder')
                
                # In array-ul de pozitii, coordonatele sunt una dupa alta. 
                # JointState standard da doar rotatii, dar hri_fullbody trimite uneori pozitiile aici.
                # Daca msg.position contine pozitii 3D, verificam inaltimea (Z).
                # hri_fullbody mapeaza pozitia articulatiei.
                
                wrist_val = msg.position[idx_wrist]
                shoulder_val = msg.position[idx_shoulder]

                cmd = Twist()
                
                # Logica:
                # Daca incheietura e mai mica decat umarul (in acest sistem de coordonate, 
                # uneori sus inseamna valoare mai mica, depinde de frame, dar sa testam diferenta). 
                
                diff = wrist_val - shoulder_val
                
                # De obicei, cand ridici mana, articulatia se roteste/misca semnificativ.
                # Aici facem o simplificare: daca detectam o diferenta mare, mergem.
                
                
                self.get_logger().info(f'W: {wrist_val:.2f} | S: {shoulder_val:.2f}')

                # Testam empiric-daca ridici mana sus (valoare < umar in imaginea pixelilor de obicei, sau > in 3D)
   
                if abs(diff) > 0.3:
                     cmd.linear.x = 0.5
                     self.get_logger().info('MERGE! (Mana detectata sus)')
                else:
                     cmd.linear.x = 0.0
                     self.get_logger().info('STOP')
                
                self.publisher_.publish(cmd)

        except Exception as e:
            self.get_logger().error(f'Eroare: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = GestureControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
