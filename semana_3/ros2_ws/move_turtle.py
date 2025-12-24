import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_bot_controller')
        #Creamos un PUBLISHER
        #Topico: /turtle1/cmd_vel
        #Mensaje: Twist (vector lineal y angular)
        self.publisher_=self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer =self.create_timer(0.5, self.move_callback)
        self.get_logger().info('Nodo Iniciado: Mover la tortuga')

    def move_callback(self):
        msg = Twist()
        msg.linear.x=2.0
        msg.angular.z=1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: Vel lineal={msg.linear.x},Vel Angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()


