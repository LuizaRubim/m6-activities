import rclpy 

from rclpy.node import Node

from geometry_msgs.msg import Twist

class TurtleDraw(Node):
    def __init__(self):
        super().__init__("TurtleDraw")
        self.publisher=self.create_publisher(
            msg_type=Twist,
            topic="turtle1/cmd_vel",
            qos_profile=10
        )

        timer_period=0.5
        