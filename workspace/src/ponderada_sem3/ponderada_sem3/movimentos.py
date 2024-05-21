import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(Empty, 'stop_robot', self.stop_robot_callback)
        self.get_logger().info("Teleoperação inicializada")
        # self.service = self.create_service(Empty, 'stop_robot', self.stop_robot_callback)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.0)

    def publisher_callback(self,twist, direcao):
        print(f"{direcao} - Velocidade linear: {twist.linear.x}, velocidade angular: {twist.angular.z}")
        self.publisher.publish(twist)

    def stop_robot_callback(self, request, response):
        # twist=Twist()
        # twist.linear.x = 0.0
        # twist.angular.z = 0.0
        # self.publisher.publish(twist)
        self.get_logger().info("Kill button acionado. Desligando o robô...")
        rclpy.shutdown()
        sys.exit()
        return response

def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key


def move(teleop_turtle):
    print("""
    ### Controle do Robô ###

    Use as seguintes teclas para controlar o robô:

        W
      A S D
        X

    Comandos:
    - W: Move o robô para frente.
    - A: Vira o robô para a esquerda.
    - X: Move o robô para trás.
    - D: Vira o robô para a direita.
    - S: Para o robô imediatamente.
    """)
    while True:
        keyPressed = get_key()
        if ord(keyPressed) == 3:
            break
        twist = Twist()
        match keyPressed:
            case "a":
                twist.linear.x = 0.0
                twist.angular.z = teleop_turtle.get_parameter('angular_speed').get_parameter_value().double_value
                teleop_turtle.publisher_callback(twist, "Esquerda")
            case "d":
                twist.linear.x = 0.0
                twist.angular.z = -teleop_turtle.get_parameter('angular_speed').get_parameter_value().double_value
                teleop_turtle.publisher_callback(twist, "Direita")
            case "w":
                twist.linear.x = teleop_turtle.get_parameter('linear_speed').get_parameter_value().double_value
                twist.angular.z = 0.0
                teleop_turtle.publisher_callback(twist, "Frente")
            case "x":
                twist.linear.x = -teleop_turtle.get_parameter('linear_speed').get_parameter_value().double_value
                twist.angular.z = 0.0
                teleop_turtle.publisher_callback(twist, "Trás")
            case "s":
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                teleop_turtle.stop_robot_callback(None, None)

def main(args=None):
    rclpy.init(args=args)
    teleop_turtle = Teleop()
    move(teleop_turtle)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
