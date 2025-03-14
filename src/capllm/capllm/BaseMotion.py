import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, Twist, Point
from capllm.utils import to_euler, angle_calculation, value_CLIP, value_NORM
import math
from std_msgs.msg import Bool, String
import subprocess
from sensor_msgs.msg import Image
import cv2
import numpy as np

PI = math.pi
ANGLE_THRESHOLD = 0.09
MAX_OMEGA = 0.6
stopping_vocab_list = ['stop', 'Stop', 'STOP', 'Stop.', 'break', 'Break', 'BREAK', 'Break.', 'exit', 'Exit', 'EXIT', 'Exit.', 'Quit', 'quit', 'QUIT', 'Quit.']

# Node for communication with Rover system
class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        # Rover coordinates and orientation subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom_baselink',
            self.odom_callback,
            10
        )
        # Rover movement publisher
        self.point_pub = self.create_publisher(
            PointStamped,
            'clicked_point',
            10
        )
        # Rover navigation reply (Position) subscriber
        self.nav_reply_pos_sub = self.create_subscription(
            Bool,
            'dstar_arrive',
            self.nav_reply_callback,
            10
        )
        # Rover navigation reply (Angle) subscriber
        self.nav_reply_angle_sub = self.create_subscription(
            Bool,
            'angle_arrive',
            self.nav_angle_reply_callback,
            10
        )
        # Rover vision detect subscriber
        self.vision_detect_sub = self.create_subscription(
            PointStamped,
            'detect_point',
            self.vision_detect_callback,
            10
        )
        self.vision_pub = self.create_publisher(
            String,
            'vision_llm',
            10
        )

        self.last_pose= [0.0, 0.0, 0.0] #[x, y, theta]
        self.LocationLibrary = {}
        self.facing_flag = False
        self.facing_point = None
        self.nav_arrived_pos = False 
        self.nav_arrived_angle = False 
        self.delta_theta = 0.0
        self.vision_detect = [0.0, 0.0] # [x, y]
        self.watch_dog1 = 0
        self.watch_dog2 = 0

    def vision_detect_callback(self, msg):
        self.vision_detect = [msg.point.x, msg.point.y]

    def nav_reply_callback(self, msg):
        # self.get_logger().info(f'nav_reply: {msg.data}')
        self.watch_dog1 += 1
        if msg.data and self.watch_dog1 > 5:
            self.nav_arrived_pos = True
            self.get_logger().info(f'point reached')
            self.watch_dog1 = 0  
    
    def nav_angle_reply_callback(self, msg):
        # self.get_logger().info(f'nav_reply: {msg.data}')
        self.watch_dog2 += 1
        if msg.data and self.watch_dog2 > 5:
            self.nav_arrived_angle = True
            self.get_logger().info(f'angle reached')
            self.watch_dog2 = 0
            

    def odom_callback(self, msg):
        # self.get_logger().info(f'position: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}, orientation: {msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}, {msg.pose.pose.orientation.w}')
        rotation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, to_euler(rotation)[2]] # x, y, euler angle in z axis
                

    def publish_point(self, target_point):
        point_msg = PointStamped()

        point_msg.point.x = float(target_point[0])
        point_msg.point.y = float(target_point[1])
        point_msg.point.z = float(target_point[2])
        
        self.point_pub.publish(point_msg)


class BASEMOTION(Node):
    def __init__(self):
        super().__init__('BaseMotion')
        # Rover command from LLM subscriber 
        self.query_sub = self.create_subscription(
            String,
            'query',
            self.query_callback,
            10
        )
        # Rover response to LLM publishers
        self.response_pub = self.create_publisher(
            String,
            'response',
            10
        )
        self.vision_response_str_sub = self.create_subscription(
            String,
            'yolo_detections_str',
            self.vision_response_callback,
            10
        )
        
        self.rover = Rover()
        self.rover.current_pose = [0, 0, 0]
        self.command_map = {
             0: lambda command: stop(command, self.rover),
             1: lambda command: go_Xaxis(command, self.rover),
             2: lambda command: go_Yaxis(command, self.rover),
             3: lambda command: go_to_point(command, self.rover),
             4: lambda command: circle(command, self.rover),
             5: lambda command: setMark(command, self.rover),
             6: lambda command: turn(command, self.rover),
            20: lambda command: Detect(command, self.rover),
            21: lambda command: Recognize(command, self.rover),
        }   
        self.force_stop = False
        self.incomming_cmd = False
        self.vision_update = False
        self.get_logger().info('BaseMotion node initialized')

    def vision_response_callback(self, msg):
        if msg:
            if msg.data == "Object found" or msg.data == "Target not found":
                # self.get_logger().info(f'vision response')
                self.vision_update = True
                return
        # self.vision_update = False

    def query_callback(self, msg):
        # self.get_logger().info(f'cmd: {msg.data}')
        self.cmd = msg.data
        self.get_logger().info(f'Cmd list: {self.cmd}') 
        if self.cmd in stopping_vocab_list:
            self.force_stop = True
            self.rover.publish_point(self.rover.current_pose)
            print("Force stop...")            
            return
        self.incomming_cmd = True

    def publish_response(self, response):
        msg = String()
        msg.data = response
        self.response_pub.publish(msg)

    # Execute the command
    def exec_cmd(self):
        path = []
        self.cmd = eval(self.cmd)
        # For each single action
        for command in self.cmd:
            self.get_logger().info(f"Ongoing command: {command}") # current command
            # if (self.force_stop == True):
            #     self.rover.publish_point(self.rover.current_pose)
            #     self.force_stop = False
            #     return
            
            # Check for validation and specification of command
            if command[0] in self.command_map: 
                if 0 < command[0] < 10: # basic movement
                    path = self.command_map[command[0]](command[1:])
                    if command[0] == 5 and path[0].lower() != 'ff': # save current location
                        print(path)
                        self.get_logger().info(f'Saving current location: {path}')
                        self.rover.LocationLibrary[path[0]] = path[1]
                        continue
                if 20 <= command[0] < 30: # visual detection
                    self.command_map[command[0]](command[1])
                    if command[0] == 21:
                        while not self.vision_update:
                            rclpy.spin_once(self, timeout_sec=0.1)
                        self.vision_update = False
                    continue
                
            if len(path) > 0:
                # self.get_logger().debug("path: ",path) # debug purpose
                for target_point in path:
                    rclpy.spin_once(self.rover, timeout_sec=0.01)
                    self.rover.last_pose = self.rover.current_pose
                    self.rover.nav_arrived_pos = False
                    self.rover.nav_arrived_angle = False
                    self.get_logger().info("Ready to move... "+f'{command}' )
                    self.publish_response("Ready to move... "+f'{command}')
                    # continue # debug purpose
                    
                    while (self.rover.nav_arrived_pos == False) or (self.rover.nav_arrived_angle == False): # close loop control
                        rclpy.spin_once(self.rover, timeout_sec=0.1)
                        rclpy.spin_once(self, timeout_sec=0.01)
                        self.pose_now = self.rover.current_pose
                        self.get_logger().debug(f"target: {target_point}, current: {self.pose_now}, command: {command}, nav_arrived_pos: {self.rover.nav_arrived_pos}, nav_arrived_angle: {self.rover.nav_arrived_angle}")
                        
                        if (self.force_stop == True):
                            self.rover.publish_point(self.rover.current_pose)
                            self.get_logger().info("Force stop...")
                            # break
                            return # TBD which is better
                        else:
                            self.rover.publish_point(target_point)
                            
                    self.rover.facing_flag = False
                    self.rover.facing_point = None
                    self.rover.nav_arrived_pos = False
                    self.rover.nav_arrived_angle = False
        # self.publish_response("Task completed")    

    # Main loop
    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            rclpy.spin_once(self.rover, timeout_sec=0.01)
            if self.incomming_cmd == False:
                continue
            
            self.exec_cmd()
            self.force_stop = False
            self.incomming_cmd = False

                        

# forward (+ve) and backward 
def go_Xaxis(cmd, cmd_node): 
    dis          = cmd[0]
    namef        = cmd[1].lower()
    x_f          = cmd[2]
    y_f          = cmd[3]
    point        = []
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    euler = position[2]
    if dis == '+FF' or dis == 'FF':
        dis = 0.2
    elif dis == '-FF':
        dis = -0.2
    position[0] += dis*math.cos(euler)
    position[1] += dis*math.sin(euler)

    if namef in cmd_node.LocationLibrary:
        x_f      = cmd_node.LocationLibrary[namef][0]
        y_f      = cmd_node.LocationLibrary[namef][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif x_f == 'FF' and y_f == 'FF':
        x_f      = position[0]
        y_f      = position[1]
    # cmd_node.facing_flag = True
    # cmd_node.facing_point = [x_f, y_f]
    point.append(position)
    return point

# forward (+ve) and backward 
def go_Yaxis(cmd, cmd_node): 
    dis          = cmd[0]
    namef        = cmd[1].lower()
    x_f          = cmd[2]
    y_f          = cmd[3]
    point        = []
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    euler = position[2] + 90.0 / 180.0 * PI
    

    if dis == '+FF' or dis == 'FF':
        dis = 0.2
    elif dis == '-FF':
        dis = -0.2
    position[0] += dis*math.cos(euler)
    position[1] += dis*math.sin(euler)
    if namef in cmd_node.LocationLibrary:
        x_f      = cmd_node.LocationLibrary[namef][0]
        y_f      = cmd_node.LocationLibrary[namef][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif x_f == 'FF' and y_f == 'FF':
        x_f      = position[0]
        y_f      = position[1]
    # cmd_node.facing_flag = True
    # cmd_node.facing_point = [x_f, y_f]
    
    point.append(position)
    return point

# [Name x, y, NameF, xf, yf]
def go_to_point(cmd, cmd_node):
    name         = cmd[0]
    x_t          = cmd[1]
    y_t          = cmd[2]
    namef        = cmd[3].lower()
    x_f          = cmd[4]
    y_f          = cmd[5]
    point        = []
    
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    euler = position[2]
    
    if name in cmd_node.LocationLibrary:
        x_t      = cmd_node.LocationLibrary[name][0]
        y_t      = cmd_node.LocationLibrary[name][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif 'last' in name:   
        x_t     = cmd_node.last_pose[0]
        y_t     = cmd_node.last_pose[1]
        print("last postion:", x_t, y_t)
    else:
        x_t     = cmd[1]
        y_t     = cmd[2]
    if x_t == 'FF' and y_t == 'FF':
        x_t     = position[0]
        y_t     = position[1]
    position    = [x_t, y_t, euler]
    
    if namef in cmd_node.LocationLibrary:
        x_f     = cmd_node.LocationLibrary[namef][0]
        y_f     = cmd_node.LocationLibrary[namef][1]
    elif x_f == 'FF' and y_f == 'FF':
        x_f     = position[0]
        y_f     = position[1]
    # cmd_node.facing_flag = True
    # cmd_node.facing_point = [x_f, y_f]
    point.append(position)
    return point

# rotate around a given point with given radius
# [4, Name, x, y, radius, number of circles, IsFacing, xf, yf]
def circle(cmd, cmd_node):
    name          = cmd[0].lower()
    x_c           = cmd[1]
    y_c           = cmd[2]
    radius        = cmd[3]
    IsFacing      = cmd[5]
    x_f           = cmd[6]
    y_f           = cmd[7]
    point         = []
    
    if name in cmd_node.LocationLibrary:
        x_c       = cmd_node.LocationLibrary[name][0]
        y_c       = cmd_node.LocationLibrary[name][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif 'vision' in name:
        x_c       = cmd_node.vision_detect[0]
        y_c       = cmd_node.vision_detect[1]
        print("vision target", x_c, y_c)
    elif x_c == 'FF' or y_c == 'FF':
        x_c       = cmd_node.current_pose[0]
        y_c       = cmd_node.current_pose[1]
    if radius == 'FF':
        radius    = 0.5
    if IsFacing and (x_f == 'FF' or y_f == 'FF'):
        x_f       = x_c
        y_f       = y_c
        
    rclpy.spin_once(cmd_node)
    position      = cmd_node.current_pose
    # position      = [0.0,0.0,0.0]


    for i in range(100):
        x         = x_c + radius*math.cos(i/100*2*PI)
        y         = y_c + radius*math.sin(i/100*2*PI)
        if IsFacing:
            euler = angle_calculation(x, y, x_f, y_f)
        else:   
            euler = position[2]
                    
        position  = [x, y, euler]
        point.append(position)
    return point

def stop(cmd, cmd_node):
    point = []
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    euler = position[2]
    
    position     = [position[0], position[1], euler]
    point.append(position)
    return point

def setMark(cmd, cmd_node):
    name = cmd[3].lower()
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    position     = [position[0], position[1]]
    return [name, position]

# [Name, x, y, direction, angle]
def turn(cmd, cmd_node):
    point = []
    rclpy.spin_once(cmd_node)
    name = cmd[0].lower()
    if name in cmd_node.LocationLibrary:
        x_t      = cmd_node.LocationLibrary[name][0]
        y_t      = cmd_node.LocationLibrary[name][1]
        print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    elif cmd[1] == 'FF' or cmd[2] == 'FF':
        if cmd[3] == 'FF': 
            cmd[3] = 1
        euler    = cmd[3] * cmd[4] / 180.0 * PI
        position = [cmd_node.current_pose[0], cmd_node.current_pose[1], cmd_node.current_pose[2] + euler]
        point.append(position)
        return point
    else:
        x_t      = cmd[1]
        y_t      = cmd[2]
    euler = angle_calculation(cmd_node.current_pose[0], cmd_node.current_pose[1], x_t, y_t)
    position = [cmd_node.current_pose[0], cmd_node.current_pose[1], euler]
    point.append(position)
    return point

# vision detection cmd
def Detect(target, cmd_node):
    msg = String()
    msg.data = "target: " + target
    
    cmd_node.vision_pub.publish(msg)
    
    return 

# vision recognition cmd
def Recognize(_, cmd_node):
    msg = String()
    msg.data = "oneshot"
    for _ in range(2):
        cmd_node.vision_pub.publish(msg)
    return
    
def main(args=None):
    rclpy.init(args=args)
    # rover = Rover()
    BaseMotion = BASEMOTION()
    BaseMotion.run()
    # rclpy.spin(BaseMotion)
    BaseMotion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()