import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose  # <--- NUEVO: Importamos el mensaje de Pose

class TurtleClosedLoop(Node):
    def __init__(self):
        super().__init__('turtle_pro_controller')
        
        # 1. PUBLISHER (Para enviar órdenes)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 2. SUBSCRIBER (Para leer la realidad - Feedback)
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Variables de estado
        self.pose = None  # Aquí guardaremos la posición real
        self.state = 'FORWARD'
        
        # Metas (Usamos coordenadas relativas simples para este ejemplo)
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.init_pose_saved = False # Flag para guardar la posición inicial
        
        self.get_logger().info('Nodo PRO Iniciado: Esperando datos de Pose...')

    def pose_callback(self, msg):
        """Este callback se activa cada vez que la tortuga se mueve"""
        self.pose = msg

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        # Si aun no tenemos datos del sensor, no hacemos nada
        if self.pose is None:
            return

        # Guardamos la posición inicial al arrancar un movimiento
        if not self.init_pose_saved:
            self.start_x = self.pose.x
            self.start_y = self.pose.y
            self.start_theta = self.pose.theta
            self.init_pose_saved = True

        msg = Twist()
        
        # --- LOGICA DE CONTROL ---
        
        if self.state == 'FORWARD':
            # Calculamos distancia real recorrida usando Pitágoras (Euclidiana)
            distance_moved = math.sqrt(
                (self.pose.x - self.start_x)**2 + 
                (self.pose.y - self.start_y)**2
            )
            
            # Control Proporcional simple: Si estoy lejos, corro. Si estoy cerca, freno.
            target_dist = 3.0
            error = target_dist - distance_moved
            
            if error > 0.1: # Tolerancia de 10cm
                msg.linear.x = 1.0 # Velocidad constante por ahora
                msg.angular.z = 0.0
            else:
                self.state = 'TURN'
                self.init_pose_saved = False # Reset para el siguiente movimiento
                self.get_logger().info(f'Distancia recorrida: {distance_moved:.2f}. Iniciando GIRO')

        elif self.state == 'TURN':
            # Calculamos cuánto hemos girado respecto al inicio del giro
            # Nota: Calcular diferencias de ángulos es truculento por el salto de pi a -pi
            current_angle = self.pose.theta
            delta_angle = self.normalize_angle(current_angle - self.start_theta)
            
            target_angle = math.pi / 2 # 90 grados
            error = target_angle - abs(delta_angle)

            if error > 0.02: # Tolerancia pequeña (0.02 rad)
                msg.linear.x = 0.0
                msg.angular.z = 0.8 # Velocidad de giro
            else:
                self.state = 'FORWARD'
                self.init_pose_saved = False # Reset para el siguiente movimiento
                self.get_logger().info(f'Giro completado. Angulo actual: {current_angle:.2f}')

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleClosedLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
