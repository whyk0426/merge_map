import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

x2 = 0
y2 = 0.5

x3 = 0.5
y3 = 0.5

def merge_maps(map0, map1, map2):
    merged_map = OccupancyGrid()
    merged_map.header = map0.header
    merged_map.header.frame_id = 'map'
    
    min_x = min(map0.info.origin.position.x, map1.info.origin.position.x + x2, map2.info.origin.position.x + x3)
    min_y = min(map0.info.origin.position.y, map1.info.origin.position.y + y3, map2.info.origin.position.y + y3)
    
    max_x = max(map0.info.origin.position.x + (map0.info.width * map0.info.resolution),
                 map1.info.origin.position.x + (map1.info.width * map1.info.resolution) + x2,
                 map2.info.origin.position.x + (map2.info.width * map2.info.resolution) + x3)
    
    max_y = max(map0.info.origin.position.y + (map0.info.height * map0.info.resolution),
                 map1.info.origin.position.y + (map1.info.height * map1.info.resolution) + y2,
                 map2.info.origin.position.y + (map2.info.height * map2.info.resolution) + y3)
    
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = min(map0.info.resolution, map1.info.resolution, map2.info.resolution)
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
    
    for y in range(map0.info.height):
        for x in range(map0.info.width):
            i = x + y * map0.info.width
            merged_x = int(np.floor((map0.info.origin.position.x + x * map0.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map0.info.origin.position.y + y * map0.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            merged_map.data[merged_i] = map0.data[i]
    
    for y in range(map1.info.height):
        for x in range(map1.info.width):
            i = x + y * map1.info.width
            merged_x = int(np.floor((map1.info.origin.position.x + x2 + x * map1.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map1.info.origin.position.y + y2 + y * map1.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            if merged_map.data[merged_i] == -1:
                merged_map.data[merged_i] = map1.data[i]
            else:
                merged_map.data[merged_i] = int(0.5 * merged_map.data[merged_i] + 0.5 * map1.data[i])
    
    for y in range(map2.info.height):
        for x in range(map2.info.width):
            i = x + y * map2.info.width
            merged_x = int(np.floor((map2.info.origin.position.x + x3 + x * map2.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map2.info.origin.position.y + y3 + y * map2.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            if merged_map.data[merged_i] == -1:
                merged_map.data[merged_i] = map2.data[i]
            else:
                merged_map.data[merged_i] = int(0.5 * merged_map.data[merged_i] + 0.5 * map2.data[i])
    
    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.subscription = self.create_subscription(OccupancyGrid, '/map0', self.map0_callback, 10)
        self.subscription = self.create_subscription(OccupancyGrid, '/map1', self.map1_callback, 10)
        self.subscription = self.create_subscription(OccupancyGrid, '/map2', self.map2_callback, 10)
        self.map0 = None        
        self.map1 = None
        self.map2 = None

    def map0_callback(self, msg):
        self.map0 = msg
        if self.map1 is not None and self.map2 is not None:
            merged_msg = merge_maps(self.map0, self.map1, self.map2)
            self.publisher.publish(merged_msg)
    
    def map1_callback(self, msg):
        self.map1 = msg
        if self.map0 is not None and self.map2 is not None:
            merged_msg = merge_maps(self.map0, self.map1, self.map2)
            self.publisher.publish(merged_msg)

    def map2_callback(self, msg):
        self.map2 = msg
        if self.map0 is not None and self.map1 is not None:
            merged_msg = merge_maps(self.map0, self.map1, self.map2)
            self.publisher.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# class MergeMapNode(Node):
#     def __init__(self):
#         super().__init__('merge_map_node')
#         self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)

#         self.subscriptions2 = []
#         for i in range(n):
#             topic_name = f'/{maps_list[i]}'
#             callback_name = f'{maps_list[i]}_callback'
#             #self_callback = getattr(self, callback_name)
#             # setattr(self, f'subscription', self.create_subscription(
#             #     OccupancyGrid, 
#             #     topic_name, 
#             #     lambda msg: self.map_callback(msg, i), 10))
            
#             self.subscriptions2.append(self.create_subscription(
#                 OccupancyGrid, topic_name, 
#                 lambda msg: self.map_callback(msg, i), 10))
#             setattr(self, maps_list[i], None)
#             self.get_logger().info('##################################################')

#     def map_callback(self, msg, map_id):
#         self.get_logger().info('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
#         self.get_logger().info(f"map_id:{map_id}")
#         setattr(self, maps_list[map_id], msg)
#         if all(getattr(self, maps_list[i]) is not None for i in range(n) if i != map_id):
#             
#             self.publisher.publish(merged_msg)