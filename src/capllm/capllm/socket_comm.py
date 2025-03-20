import socket
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

import socket
import struct
import numpy as np

class SocketSender:
    def __init__(self, host='fyp', port=6000):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

    def send(self, query):
        self.socket.sendall(query.encode())
        data = self.socket.recv(12)
        return np.array(struct.unpack("<3f", data))

    def __del__(self):
        self.socket.close()


class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        self.point_pub = self.create_publisher(
            PointStamped,
            'clicked_point',
            10
        )
        
    def publish_point(self, target_point):
        point_msg = PointStamped()

        point_msg.point.x = float(target_point[0])
        point_msg.point.y = float(target_point[1])
        point_msg.point.z = float(target_point[2])
        
        self.point_pub.publish(point_msg)


def main():
    sender = SocketSender(host='fyp', port=6000)
    rclpy.init()
    rover = Rover()
     
    while True:
        try:
            query = input("Enter your query: ")
            target_location = sender.send(query)
            print(target_location)
            rover.publish_point(target_location)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(e)
            break


if __name__ == "__main__":
    main()
        
    