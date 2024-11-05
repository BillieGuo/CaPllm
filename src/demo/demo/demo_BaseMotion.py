import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, Twist, Point
import math
PI = math.pi

# turtlesim demo only
from turtlesim.msg import Pose
import time
from demo.utils import angle_calculation
from std_msgs.msg import String
# turtlesim demo only


# for turtlesim demo only
class TurtleSimDemo_pub(Node):
    def __init__(self):
        super().__init__('turtlesim_demo_pub')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',    
            self.pose_callback,
            10)
        self.vel_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)
        
        self.current_pose = None
        self.vel = Twist()
        self.LocationLibrary = {}
        self.facing_flag = False
        self.facing_point = None
        
        
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.quit_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.quit_flag = False


    def quit_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
        if msg.data == 'quit' or msg.data == 'q':
            self.quit_flag = True
        else:
            self.quit_flag = False
             

    def pose_callback(self, msg):
        # self.get_logger().info(f'pub:update:{msg.x}, {msg.y}, {msg.theta}, {msg.linear_velocity}, {msg.angular_velocity}')
        self.current_pose = [msg.x, msg.y, msg.theta] # x, y, z
        

    def publish_Twist(self, target_point, bias):
        twist_msg = Twist()
        timer_period = 0.5
        rclpy.spin_once(self)
        vx = (target_point[0] - self.current_pose[0])
        vy = (target_point[1] - self.current_pose[1])
        angle = angle_calculation(self.current_pose[0], self.current_pose[1], target_point[0], target_point[1])
        v = (vx**2 + vy**2)**0.5
        delta_theta = target_point[2] - self.current_pose[2]
        if delta_theta >= PI:
            delta_theta -= 2*PI
        elif delta_theta <= -PI:
            delta_theta += 2*PI
            
        twist_msg.linear.x = v * math.cos(angle-self.current_pose[2]) / timer_period
        twist_msg.linear.y = v * math.sin(angle-self.current_pose[2]) / timer_period
        twist_msg.angular.z = delta_theta / timer_period
        
        self.vel.linear.x = twist_msg.linear.x
        self.vel.linear.y = twist_msg.linear.y
        self.vel.angular.z = twist_msg.angular.z
        
        if twist_msg.linear.x is None:
            twist_msg.linear.x = 0
        if twist_msg.linear.y is None:
            twist_msg.linear.y = 0
        self.vel_pub.publish(twist_msg)
        

class Demo:
    def __init__(self, ):
        self.demo_pub = TurtleSimDemo_pub()
        self.demo_pub.current_pose = [0, 0, 0]
        self.demo_pub.current_orientation = [0, 0, 0, 0]
        self.position_now = [0, 0, 0]
        self.command_map = {
            0: lambda command: stop(command, self.demo_pub),
            1: lambda command: go_Xaxis(command, self.demo_pub),
            2: lambda command: go_Yaxis(command, self.demo_pub),
            3: lambda command: go_to_point(command, self.demo_pub),
            4: lambda command: circle(command, self.demo_pub),
            5: lambda command: setMark(command, self.demo_pub),
            6: lambda command: turn(command, self.demo_pub),
        }

    def run(self, cmd):
        path = []
        bias = 0
        cmd = eval(cmd)
        print("cmd: ",cmd)
        # single action
        for command in cmd:
            print("command: ",command)
            if command[0] in self.command_map:
                if 0 < command[0] < 10:
                    print('command:', command)
                    path = self.command_map[command[0]](command[1:])
                    if command[0] == 5:
                        print('set mark:', path)
                        self.demo_pub.LocationLibrary[path[0]] = path[1]
                        continue
            print("path: ",path)
            path = [[1.9265364335273012, -0.7389453756199589, 3.141592653589793], [1.9245631619555728, -0.6761548560906455, -3.0787608005179976], [1.9186511348417792, -0.6136121420556546, -3.0159289474462017], [1.90882368425599, -0.5515640610342343, -2.9530970943744057], [1.8951195946559323, -0.4902554884551041, -2.8902652413026098], [1.8775929498224548, -0.4299283812450115, -2.827433388230814], [1.8563129194155525, -0.37082082293528096, -2.764601535159018], [1.8313634859933208, -0.31316608405488616, -2.7017696820872223], [1.8028431135711647, -0.25719170151824355, -2.6389378290154264], [1.7708643590293163, -0.20311858064096222, -2.5761059759436304], [1.7355534279022486, -0.15116012332748574, -2.5132741228718345], [1.6970496763030904, -0.10152138587126924, -2.450442269800039], [1.6555050609487127, -0.05439826969127026, -2.387610416728243], [1.61108353945599, -0.009976748198547325, -2.324778563656447], [1.563960423275991, 0.031567867155830376, -2.261946710584651], [1.5143216858197743, 0.07007161875498857, -2.199114857512855], [1.4623632285062977, 0.1053825498820562, -2.1362830044410592], [1.4082901076290164, 0.13736130442390482, -2.0734511513692633], [1.3523157250923739, 0.1658816768460607, -2.0106192982974678], [1.2946609862119793, 0.19083111026829247, -1.947787445225672], [1.2355534279022486, 0.21211114067519465, -1.884955592153876], [1.1752263206921563, 0.2296377855086722, -1.8221237390820804], [1.113917748113026, 0.24334187510872973, -1.7592918860102844], [1.0518696670916055, 0.253169325694519, -1.6964600329384885], [0.9893269530566147, 0.2590813528083127, -1.6336281798666925], [0.9265364335273013, 0.2610546243800411, -1.5707963267948968], [0.8637459139979878, 0.2590813528083127, -1.5079644737231008], [0.8012031999629968, 0.2531693256945189, -1.4451326206513047], [0.7391551189415764, 0.24334187510872973, -1.382300767579509], [0.6778465463624466, 0.2296377855086723, -1.3194689145077134], [0.6175194391523539, 0.21211114067519476, -1.2566370614359175], [0.5584118808426233, 0.19083111026829247, -1.1938052083641215], [0.5007571419622285, 0.1658816768460606, -1.1309733552923256], [0.44478275942558576, 0.1373613044239046, -1.0681415022205296], [0.3907096385483043, 0.10538254988205609, -1.0053096491487337], [0.33875118123482817, 0.07007161875498857, -0.942477796076938], [0.28911244377861145, 0.031567867155830376, -0.8796459430051421], [0.24198932759861247, -0.009976748198547436, -0.8168140899333461], [0.19756780610588986, -0.05439826969127004, -0.7539822368615506], [0.15602319075151205, -0.10152138587126902, -0.6911503837897547], [0.11751943915235386, -0.15116012332748563, -0.6283185307179588], [0.08220850802528634, -0.2031185806409619, -0.5654866776461632], [0.05022975348343783, -0.2571917015182433, -0.5026548245743673], [0.02170938106128184, -0.313166084054886, -0.4398229715025713], [-0.003240052360950152, -0.37082082293528074, -0.3769911184307754], [-0.024520082767852336, -0.42992838124501137, -0.3141592653589794], [-0.04204672760132988, -0.490255488455104, -0.2513274122871835], [-0.055750817201387415, -0.5515640610342338, -0.18849555921538802], [-0.06557826778717657, -0.6136121420556544, -0.12566370614359196], [-0.07149029490097036, -0.6761548560906453, -0.06283185307179608], [-0.0734635664726988, -0.7389453756199588, -1.1102230246251565e-16], [-0.07149029490097036, -0.8017358951492722, 0.06283185307179585], [-0.06557826778717668, -0.8642786091842631, 0.12566370614359174], [-0.055750817201387415, -0.9263266902056837, 0.1884955592153878], [-0.04204672760132988, -0.987635262784814, 0.25132741228718375], [-0.024520082767852336, -1.0479623699949066, 0.3141592653589796], [-0.003240052360950041, -1.1070699283046372, 0.37699111843077565], [0.021709381061281507, -1.164724667185031, 0.4398229715025706], [0.0502297534834375, -1.2206990497216739, 0.5026548245743666], [0.08220850802528601, -1.2747721705989554, 0.5654866776461627], [0.11751943915235363, -1.3267306279124318, 0.6283185307179584], [0.15602319075151194, -1.3763693653686486, 0.6911503837897546], [0.19756780610588964, -1.4234924815486476, 0.7539822368615504], [0.24198932759861225, -1.4679140030413702, 0.8168140899333459], [0.28911244377861167, -1.5094586183957484, 0.8796459430051424], [0.33875118123482795, -1.5479623699949063, 0.9424777960769379], [0.39070963854830487, -1.5832733011219742, 1.005309649148734], [0.4447827594255859, -1.6152520556638224, 1.0681415022205296], [0.500757141962229, -1.6437724280859787, 1.130973355292326], [0.5584118808426226, -1.66872186150821, 1.1938052083641209], [0.6175194391523536, -1.6900018919151125, 1.2566370614359172], [0.6778465463624459, -1.70752853674859, 1.3194689145077128], [0.7391551189415766, -1.7212326263486477, 1.3823007675795091], [0.8012031999629966, -1.7310600769344366, 1.4451326206513044], [0.863745913997988, -1.7369721040482304, 1.5079644737231008], [0.926536433527301, -1.7389453756199589, 1.5707963267948963], [0.989326953056614, -1.7369721040482304, 1.6336281798666918], [1.0518696670916055, -1.7310600769344369, 1.6964600329384882], [1.1139177481130256, -1.7212326263486477, 1.759291886010284], [1.175226320692156, -1.70752853674859, 1.8221237390820801], [1.2355534279022484, -1.6900018919151125, 1.8849555921538756], [1.2946609862119796, -1.6687218615082102, 1.9477874452256723], [1.352315725092373, -1.643772428085979, 2.010619298297467], [1.4082901076290162, -1.6152520556638228, 2.0734511513692633], [1.4623632285062973, -1.5832733011219744, 2.136283004441059], [1.514321685819774, -1.5479623699949063, 2.199114857512855], [1.5639604232759905, -1.5094586183957484, 2.2619467105846507], [1.61108353945599, -1.4679140030413704, 2.324778563656447], [1.6555050609487125, -1.4234924815486478, 2.3876104167282426], [1.6970496763030907, -1.3763693653686486, 2.450442269800039], [1.7355534279022486, -1.3267306279124322, 2.5132741228718345], [1.7708643590293165, -1.2747721705989552, 2.576105975943631], [1.8028431135711647, -1.220699049721674, 2.6389378290154264], [1.8313634859933208, -1.164724667185031, 2.701769682087223], [1.8563129194155523, -1.1070699283046377, 2.764601535159017], [1.8775929498224548, -1.0479623699949066, 2.827433388230814], [1.8951195946559323, -0.9876352627848142, 2.8902652413026093], [1.90882368425599, -0.9263266902056836, 2.9530970943744057], [1.918651134841779, -0.8642786091842636, 3.015928947446201], [1.9245631619555728, -0.8017358951492721, 3.0787608005179976]]
            for i in range (len(path)):
                path[i] = [path[i][0]+4.0, path[i][1]+4.0, path[i][2]]
            if len(path) > 0:
                for target_point in path:
                    rclpy.spin_once(self.demo_pub)
                    self.pose_now = self.demo_pub.current_pose
                    delta_theta = target_point[2] - self.demo_pub.current_pose[2]
                    if delta_theta >= PI:
                        delta_theta -= 2*PI
                    elif delta_theta <= -PI:
                        delta_theta += 2*PI
                    
                    self.demo_pub.publish_Twist(target_point, bias)
                    
                    print('sending a point!')
                    print("target: ", target_point, "current: ", self.demo_pub.current_pose)
                    
                    bias = ((self.pose_now[0] - target_point[0])**2 + (self.pose_now[1] - target_point[1])**2 + (delta_theta)**2)**0.5
                    while bias > 0.05:
                        rclpy.spin_once(self.demo_pub)
                        
                        print('bias:', bias)
                        print('vel:', self.demo_pub.vel.linear.x, self.demo_pub.vel.linear.y, self.demo_pub.vel.angular.z)
                        delta_theta = target_point[2] - self.demo_pub.current_pose[2]
                        if delta_theta >= PI:
                            delta_theta -= 2*PI
                        elif delta_theta <= -PI:
                            delta_theta += 2*PI
                        self.pose_now = self.demo_pub.current_pose
                        
                        bias = ((self.pose_now[0] - target_point[0])**2 + (self.pose_now[1] - target_point[1])**2 + (delta_theta)**2)**0.5
                        self.demo_pub.publish_Twist(target_point, bias)
                    self.demo_pub.facing_flag = False
                    self.demo_pub.facing_point = None
                        



# forward (+ve) and backward 
# [distance, NameF, xf, yf]
def go_Xaxis(cmd, cmd_node): 
    dis          = cmd[0]
    namef        = cmd[1]
    x_f          = cmd[2]
    y_f          = cmd[3]
    point        = []
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    theta = position[2]

    if dis == '+FF':
        dis = 1
    elif dis == '-FF':
        dis = -1
    position[0] += dis*math.cos(theta)
    position[1] += dis*math.sin(theta)
    if namef in cmd_node.LocationLibrary:
        x_f      = cmd_node.LocationLibrary[namef][0]
        y_f      = cmd_node.LocationLibrary[namef][1]
    elif x_f == 'FF' and y_f == 'FF':
        x_f      = position[0]
        y_f      = position[1]
    cmd_node.facing_flag = True
    cmd_node.facing_point = [x_f, y_f]
    point.append(position)
    return point


# left (+ve) and right
def go_Yaxis(cmd, cmd_node):
    dis          = cmd[0]
    namef         = cmd[1]
    x_f          = cmd[2]
    y_f          = cmd[3]
    point        = []
    rclpy.spin_once(cmd_node)
    position     = cmd_node.current_pose
    theta = position[2] + math.pi/2.0
        
    if dis == 'FF':
        dis = 1
    elif dis == '-FF':
        dis = -1
    position[0] += dis*math.cos(theta)
    position[1] += dis*math.sin(theta)
    if namef in cmd_node.LocationLibrary:
        x_f      = cmd_node.LocationLibrary[namef][0]
        y_f      = cmd_node.LocationLibrary[namef][1]
    elif x_f == 'FF' and y_f == 'FF':
        x_f      = position[0]
        y_f      = position[1]
    cmd_node.facing_flag = True
    cmd_node.facing_point = [x_f, y_f]
    point.append(position)
    return point


# [Name x, y, NameF, xf, yf]
def go_to_point(cmd, cmd_node):
    name         = cmd[0]
    x_t          = cmd[1]
    y_t          = cmd[2]
    namef        = cmd[3]
    x_f          = cmd[4]
    y_f          = cmd[5]
    point        = []
    
    if name in cmd_node.LocationLibrary:
        x_t      = cmd_node.LocationLibrary[name][0]
        y_t      = cmd_node.LocationLibrary[name][1]
    else:
        x_t      = cmd[1]
        y_t      = cmd[2]
    position     = [x_t, y_t, 0]
    
    if namef in cmd_node.LocationLibrary:
        x_f      = cmd_node.LocationLibrary[namef][0]
        y_f      = cmd_node.LocationLibrary[namef][1]
    elif x_f == 'FF' and y_f == 'FF':
        x_f      = position[0]
        y_f      = position[1]
    cmd_node.facing_flag = True
    cmd_node.facing_point = [x_f, y_f]
    point.append(position)
    return point

# rotate around a given point with given radius
# [x, y, radius, number of circles, NameF, xf, yf]
def circle(cmd, cmd_node):
    x_c    = cmd[0]
    y_c    = cmd[1]
    radius = cmd[2]
    namef  = cmd[4]
    x_f    = cmd[5]
    y_f    = cmd[6]
    point  = []
    print("cmd: ",cmd)
    print("namef: ",namef, "x_f: ",x_f, "y_f: ",y_f)
    
    if namef in cmd_node.LocationLibrary:
        x_f      = cmd_node.LocationLibrary[namef][0]
        y_f      = cmd_node.LocationLibrary[namef][1]
        x_c      = x_f
        y_c      = y_f
        cmd_node.facing_flag = True
        cmd_node.facing_point = [x_f, y_f]
        print("face name")
    elif x_f == 'FF' and y_f == 'FF':
        cmd_node.facing_flag = False
        cmd_node.facing_point = None
    elif x_f != 'FF' and y_f != 'FF':
        x_f      = cmd[5]
        y_f      = cmd[6]
        x_c      = x_f
        y_c      = y_f
        cmd_node.facing_flag = True
        cmd_node.facing_point = [x_f, y_f]
        print("face point")
        
        
        
    rclpy.spin_once(cmd_node)
    if radius == 'FF':
        radius = 1
    if x_c == 'FF' and y_c == 'FF':
        x_c = cmd_node.current_pose[0]
        y_c = cmd_node.current_pose[1]
        
    print("x_c: ",x_c, "y_c: ",y_c)
    for i in range(100):
        x = x_c + radius*math.cos(i/100*2*math.pi)
        y = y_c + radius*math.sin(i/100*2*math.pi)
        if x_f == 'FF' and y_f == 'FF':
            theta = cmd_node.current_pose[2]
        else:   
            theta = angle_calculation(x, y, x_c, y_c)
            
        position = [x, y, theta]
        point.append(position)
    return point


def stop(cmd, cmd_node):
    point = []
    point.append(cmd_node.current_pose)
    return point


def setMark(cmd, cmd_node):
    name = cmd[3]
    IsCurrentLocation = cmd[0]
    if IsCurrentLocation:
        rclpy.spin_once(cmd_node)
        position = [cmd_node.current_pose[0], cmd_node.current_pose[1], cmd_node.current_pose[2]]
    else:
        position = [cmd[1], cmd[2], 0]
    return [name, position]

# [Name, x, y, direction, angle]
def turn(cmd, cmd_node):
    point = []
    rclpy.spin_once(cmd_node)
    name = cmd[0]
    if name in cmd_node.LocationLibrary:
        x_t      = cmd_node.LocationLibrary[name][0]
        y_t      = cmd_node.LocationLibrary[name][1]
    elif cmd[1] == 'FF' or cmd[2] == 'FF':
        theta    = cmd[3] * cmd[4] / 180.0 * math.pi
        position = [cmd_node.current_pose[0], cmd_node.current_pose[1], cmd_node.current_pose[2] + theta]
        point.append(position)
        return point
    else:
        x_t      = cmd[1]
        y_t      = cmd[2]
    angle = angle_calculation(cmd_node.current_pose[0], cmd_node.current_pose[1], x_t, y_t)
    position = [cmd_node.current_pose[0], cmd_node.current_pose[1], angle]
    print("position: ",position)
    print("cmd_nodeLibrary: ",cmd_node.LocationLibrary)
    point.append(position)
    return point


# test
def main(args=None):
    rclpy.init(args=args)

    demo = Demo()

    # test_action = [[1, 2], [2, 3], [4, 4, 3, 2, 1]]
    test_action = f"[[4, 4, 3, 2, 1, 'FF', 'FF', 'FF']]"
    
    cnt = 0
    while True:
        time.sleep(3)
        demo.run(test_action)
        print(cnt)
        cnt+=1

    demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()