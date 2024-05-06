import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
import time

# Lista de pontos para a titiruga "turtle1"
pontos_lua=[(0.0, -5.0, 3.0), (0.0,3.0,-2.5),(0.0,3.0,-2.5),(-3.1, -3.0, 3.0)]
# Lista de pontos para a titiruga "turtle2"
pontos_estrela=[(0.0, 0.0, 0.0), (1.0,2.0,0.0), (1.0,-2.0,0.0), (2.0, -1.0, 0.0), (-2.0, -1.0, 0.0), (1.0, -2.0, 0.0), (-2.0, 2.0, 0.0), (-2.0, -2.0, 0.0), (1.0, 2.0, 0.0), (-2.0, 0.85, 0.0), (2.0, 1.0, 0.0)]

class TurtleDraw(Node):
    def __init__(self):
        super().__init__("TurtleDraw")
        # Cria um publisher para enviar comandos de movimento para a titiruga "turtle1"
        self.publisher=self.create_publisher(
            msg_type=Twist,
            topic="turtle1/cmd_vel",
            qos_profile=10
        )
        # Cria um publisher para enviar comandos de movimento para a titiruga "turtle2"
        self.publisher2=self.create_publisher(
            msg_type=Twist,
            topic="turtle2/cmd_vel",
            qos_profile=10
        )

    def move_turtle(self,x,y,theta, publisher):
        # Cria uma mensagem Twist para definir o movimento da titiruga
        msg=Twist()
        msg.linear.x=x
        msg.linear.y=y
        msg.angular.z=theta
        # Publica a mensagem no tópico correto, dependendo da titiruga
        if publisher == 'turtle1':
            self.publisher.publish(msg)
            self.get_logger().info("Movendo titiruga 1")
        if publisher == 'turtle2':
            self.publisher2.publish(msg)
            self.get_logger().info("Movendo titiruga 2")
    
def set_pen_color(r, g, b, off):
    # Cria um nó para chamar o serviço de definição de cor da caneta da titiruga "turtle1"
    node = rclpy.create_node('set_pen_color')
    set_pen_client = node.create_client(SetPen, '/turtle1/set_pen')

    while not set_pen_client.wait_for_service(timeout_sec=1.0):
        print('Serviço não disponível, aguardando...')
    request = SetPen.Request()
    request.r = r
    request.g = g
    request.b = b
    request.width = 6
    request.off = off 
    future = set_pen_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

def main(args=None):
    rclpy.init(args=args)
    td = TurtleDraw()
    # Define a cor da caneta da titiruga "turtle1"
    set_pen_color( 190, 150, 50, 0)

    # Movimenta a titiruga "turtle1" para cada ponto da lista "pontos_lua"
    for ponto in pontos_lua:
        x, y, theta = ponto
        td.move_turtle(x, y, theta, "turtle1")
        time.sleep(1)

    # Cria uma requisição para matar a titiruga "turtle1"
    kill_request = Kill.Request()
    kill_request.name = 'turtle1'

    kill_client = td.create_client(Kill, 'kill')

    # Aguarda até que o serviço esteja disponível
    while not kill_client.wait_for_service(timeout_sec=1.0):
        td.get_logger().info('Serviço não disponível, aguardando novamente...')

    # Faz a chamada para o serviço de matar a titiruga "turtle1"
    future = kill_client.call_async(kill_request)
    rclpy.spin_until_future_complete(td, future)

    if future.result() is not None:
        td.get_logger().info('titiruga morta com sucesso')
    else:
        td.get_logger().info('Falha ao matar a titiruga')

    # Cria uma requisição para criar a titiruga "turtle2"
    spwan_request = Spawn.Request()
    spwan_request.x = 2.0
    spwan_request.y = 7.0
    spwan_request.theta = 0.0
    spwan_request.name = 'turtle2'
    
    spawn_client = td.create_client(Spawn, 'spawn')

    # Aguarda até que o serviço esteja disponível
    while not spawn_client.wait_for_service(timeout_sec=1.0):
        td.get_logger().info('Serviço não disponível, aguardando novamente...')
    
    # Faz a chamada para o serviço de criar a titiruga "turtle2"
    future = spawn_client.call_async(spwan_request)
    rclpy.spin_until_future_complete(td, future)

    if future.result() is not None:
        td.get_logger().info('titiruga criada com sucesso')
    else:
        td.get_logger().info('Falha ao criar a titiruga')

    # Movimenta a titiruga "turtle2" para cada ponto da lista "pontos_estrela"
    for ponto in pontos_estrela:
        x, y, theta = ponto
        td.move_turtle(x, y, theta, "turtle2")
        time.sleep(1)

    td.destroy_node() 
    rclpy.shutdown()

if __name__ =="__main__":
    main()

