import rclpy 

from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
import time

pontos2=[(0.0, -5.0, 3.0), (0.0,3.0,-2.5),(0.0,3.0,-2.5),(-3.0, -3.0, 3.0)]

class TurtleDraw(Node):
    def __init__(self):
        super().__init__("TurtleDraw")
        self.publisher=self.create_publisher(
            msg_type=Twist,
            topic="turtle1/cmd_vel",
            qos_profile=10
        )

        self.subscription=self.create_subscription(
            msg_type=Pose,
            topic="turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10
        )
        
    def pose_callback(self, msg: Pose):
        # Access the turtle's current position and orientation
        x = msg.x
        y = msg.y
        theta = msg.theta
        return x, y, theta


    def move_turtle(self,x,y,theta):
        msg=Twist()
        msg.linear.x=x
        msg.linear.y=y
        msg.angular.z=theta
        self.publisher.publish(msg)
        self.get_logger().info("publicando velocidades para a tartaruga")
    
def set_pen_color(r, g, b, off):

    node = rclpy.create_node('set_pen_color')
    set_pen_client = node.create_client(SetPen, '/turtle1/set_pen')

    while not set_pen_client.wait_for_service(timeout_sec=1.0):
        print('Service not available, waiting again...')
    request = SetPen.Request()
    request.r = r
    request.g = g
    request.b = b
    request.width = 6  # Mantém a largura da linha inalterada
    request.off = off    # Mantém a caneta ligada
    future = set_pen_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

def main(args=None):
    rclpy.init(args=args)
    td = TurtleDraw()
    set_pen_color( 190, 150, 50, 0)

    for ponto in pontos2:
        x, y, theta = ponto
        td.move_turtle(x, y, theta)
        time.sleep(1)
    
    set_pen_color( 0, 0, 0, 1)


    td.destroy_node() 
    rclpy.shutdown()

if __name__ =="__main__":
    main()

