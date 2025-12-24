import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Robot Autonomo Iniciado. ¡A rodar!')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Fallo al convertir: {e}')
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_yellow = np.array([15, 100, 70])
        upper_yellow = np.array([40, 255, 255])
        
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        M = cv2.moments(mask)
        
        twist = Twist()
        
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            setpoint = 400 
            error = cx - setpoint
            
            Kp = -0.005 
            
            twist.angular.z = Kp * error
            twist.linear.x = 0.3
            
            cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
            self.get_logger().info(f'Error: {error} | Giro: {twist.angular.z:.3f}')
            
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('LINEA PERDIDA - Buscando...')

        self.publisher.publish(twist)

        cv2.imshow("Camara Original", cv_image)
        cv2.imshow("Mascara", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
