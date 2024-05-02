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
        self.timer=self.create_timer(
            timer_period_sec=timer_period,
            callback=self.timer_callback
        )

    def timer_callback(self):
        msg=Twist()
        msg.linear.x=1.0
        msg.angular.z=3.0
        self.publisher.publish(msg)
        self.get_logger().info("publicando velocidades para a tartaruga")
    
    def move_turtle(self):
        msg=Twist()
        msg.linear.x=1.0
        msg.angular.z=3.0
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

