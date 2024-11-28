import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

x2 = 0
y2 = 2

x3 = 0
y3 = 2

def merge_maps(map1, map2, map3):
    merged_map = OccupancyGrid()
    merged_map.header = map1.header
    merged_map.header.frame_id = 'map'
    
    min_x = min(map1.info.origin.position.x, map2.info.origin.position.x + x2, map3.info.origin.position.x + x3)
    min_y = min(map1.info.origin.position.y, map2.info.origin.position.y + y3, map3.info.origin.position.y + y3)
    
    max_x = max(map1.info.origin.position.x + (map1.info.width * map1.info.resolution),
                 map2.info.origin.position.x + (map2.info.width * map2.info.resolution) + x2,
                 map3.info.origin.position.x + (map3.info.width * map3.info.resolution) + x3)
    
    max_y = max(map1.info.origin.position.y + (map1.info.height * map1.info.resolution),
                 map2.info.origin.position.y + (map2.info.height * map2.info.resolution) + y2,
                 map3.info.origin.position.y + (map3.info.height * map3.info.resolution) + y3)
    
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution, map3.info.resolution)
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
    
    for y in range(map1.info.height):
        for x in range(map1.info.width):
            i = x + y * map1.info.width
            merged_x = int(np.floor((map1.info.origin.position.x + x * map1.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map1.info.origin.position.y + y * map1.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            merged_map.data[merged_i] = map1.data[i]
    
    for y in range(map2.info.height):
        for x in range(map2.info.width):
            i = x + y * map2.info.width
            merged_x = int(np.floor((map2.info.origin.position.x + x2 + x * map2.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map2.info.origin.position.y + y2 + y * map2.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            if merged_map.data[merged_i] == -1:
                merged_map.data[merged_i] = map2.data[i]
            else:
                merged_map.data[merged_i] = int(0.5 * merged_map.data[merged_i] + 0.5 * map2.data[i])
    
    for y in range(map3.info.height):
        for x in range(map3.info.width):
            i = x + y * map3.info.width
            merged_x = int(np.floor((map3.info.origin.position.x + x3 + x * map3.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map3.info.origin.position.y + y3 + y * map3.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            if merged_map.data[merged_i] == -1:
                merged_map.data[merged_i] = map3.data[i]
            else:
                merged_map.data[merged_i] = int((merged_map.data[merged_i] + map3.data[i]) * 0.5)
    
    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.subscription1 = self.create_subscription(OccupancyGrid, '/map1', self.map1_callback, 10)
        self.subscription2 = self.create_subscription(OccupancyGrid, '/map2', self.map2_callback, 10)
        self.subscription3 = self.create_subscription(OccupancyGrid, '/map3', self.map3_callback, 10)
        self.map1 = None
        self.map2 = None
        self.map3 = None

    def map1_callback(self, msg):
        self.map1 = msg
        if self.map2 is not None and self.map3 is not None:
            merged_msg = merge_maps(self.map1, self.map2, self.map3)
            self.publisher.publish(merged_msg)
    
    def map2_callback(self, msg):
        self.map2 = msg
        if self.map1 is not None and self.map3 is not None:
            merged_msg = merge_maps(self.map1, self.map2, self.map3)
            self.publisher.publish(merged_msg)

    def map3_callback(self, msg):
        self.map3 = msg
        if self.map1 is not None and self.map2 is not None:
            merged_msg = merge_maps(self.map1, self.map2, self.map3)
            self.publisher.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
