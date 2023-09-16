import rclpy
from rclpy.node import Node
from our_msgs.msg import Raceline
from geometry_msgs.msg import Quaternion, Point
from pathlib import Path

MAP_NAME = "Nuerburgring_raceline"

class MapNode(Node):
    def __init__(self):
        super().__init__("map")
        
        self.pub = self.create_publisher(Raceline, "/map", 10)
        self.create_timer(1, self.publish_map)
        
    def publish_map(self):
        map = Raceline()
        with open(f"{Path(__file__).parent}/data/{MAP_NAME}.csv") as f:
            for line in f.readlines():
                if line[0] == "#":
                    continue
                _, x, y, *_ = line.strip("\n").split(";")
                map.x.append(float(x))
                map.y.append(float(y))
        map.pose.position = Point(x=map.x[0], y=map.y[0])
        self.pub.publish(map)
        
def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    rclpy.spin(node)
    rclpy.shutdown()