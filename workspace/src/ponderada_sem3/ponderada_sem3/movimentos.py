import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


# Classe que controla o robô
class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(Empty, 'stop_robot', self.kill_button)
        self.get_logger().info("Teleoperação inicializada")
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.0)
        self.should_shutdown = False

    # Função que publica a velocidade do robô
    def publisher_callback(self,twist, direcao):
        print(f"{direcao} - Velocidade linear: {twist.linear.x}, velocidade angular: {twist.angular.z}")
        self.publisher.publish(twist)

    # Função que para o robô (kill button)
    def kill_button(self, request, response):
        self.get_logger().info("Kill button acionado. Desligando o robô...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.should_shutdown = True
        response = Empty.Response()
        return response

# Função que lê a tecla pressionada
def get_key():
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key

# Função que controla o robô com as teclas e modifica o linear e angular speed
def move(teleop_turtle):
    print("""
    ### Teleoperação ###

    Use as seguintes teclas para controlar o robô:

        W
      A S D
        X

    Comandos:
    - W: Frente.
    - A: Esquerda.
    - X: Trás.
    - D: Direita.
    - S: Para o robô e fecha o programa(kill button).
    """)
    while rclpy.ok() and not teleop_turtle.should_shutdown:
        keyPressed = get_key()
        if ord(keyPressed) == 3:
            break
        twist = Twist()
        match keyPressed:
            case "w":
                twist.linear.x = teleop_turtle.get_parameter('linear_speed').get_parameter_value().double_value
                twist.angular.z = 0.0
                teleop_turtle.publisher_callback(twist, "Frente")

            case "a":
                twist.linear.x = 0.0
                twist.angular.z = teleop_turtle.get_parameter('angular_speed').get_parameter_value().double_value
                teleop_turtle.publisher_callback(twist, "Esquerda")

            case "x":
                twist.linear.x = -teleop_turtle.get_parameter('linear_speed').get_parameter_value().double_value
                twist.angular.z = 0.0
                teleop_turtle.publisher_callback(twist, "Trás")

            case "d":
                twist.linear.x = 0.0
                twist.angular.z = -teleop_turtle.get_parameter('angular_speed').get_parameter_value().double_value
                teleop_turtle.publisher_callback(twist, "Direita")

            case "s":
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                teleop_turtle.kill_button(Empty.Request(), Empty.Response())

def main(args=None):
    rclpy.init(args=args)
    teleop_turtle = Teleop()
    move(teleop_turtle)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
