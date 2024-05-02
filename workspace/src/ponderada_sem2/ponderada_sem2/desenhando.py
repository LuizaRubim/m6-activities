import rclpy 

from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
import time

pontos = [(1, 1, 3), (1, 5, 3), (5, 5, -3), (5, 1, 2), (1, 1, 1 )]

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
        
        timer_period=0.5
        self.timer=self.create_timer(
            timer_period_sec=timer_period,
            callback=self.move_callback
        )

    def pose_callback(self, msg: Pose):
        # Access the turtle's current position and orientation
        x = msg.x
        y = msg.y
        theta = msg.theta

        self.get_logger().info(f"Turtle position: x={x}, y={y}, theta={theta}")
        return x, y, theta


    def move_callback(self,x,y,theta):
        msg=Twist()
        msg.linear.x=x
        msg.linear.y=y
        msg.angular.z=theta
        self.publisher.publish(msg)
        self.get_logger().info("publicando velocidades para a tartaruga")
    
    

def main(args=None):
    rclpy.init(args=args)
    td = TurtleDraw()
    rclpy.spin(td)
    td.destroy_node() 
    rclpy.shutdown()

if __name__ =="__main__":
    main()

