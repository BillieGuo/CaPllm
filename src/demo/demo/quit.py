import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class QuitNode(Node):
    def __init__(self):
        super().__init__('quit_node')
        self.publisher = self.create_publisher(
            String,
            'chatter',
            10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = input("Enter 'q' to quit: ")
        self.publisher.publish(msg)
    
    def run(self):
        msg = String()
        msg.data = input("Enter 'q' to quit: ")
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = QuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()