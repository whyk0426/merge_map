import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

n = 3
xd = [0, 0, 0.5]  
yd = [0, 0.5, 0.5]  

maps_list = [f"map{i}" for i in range(n)] 

def merge_maps(*maps):
    merged_map = OccupancyGrid()
    merged_map.header = maps[0].header  
    merged_map.header.frame_id = 'map'
    
    min_x = min(map.info.origin.position.x + xd[i] for i, map in enumerate(maps))
    min_y = min(map.info.origin.position.y + yd[i] for i, map in enumerate(maps))
    
    max_x = max(map.info.origin.position.x + (map.info.width * map.info.resolution) + xd[i] for i, map in enumerate(maps))
    max_y = max(map.info.origin.position.y + (map.info.height * map.info.resolution) + yd[i] for i, map in enumerate(maps)) 
    
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = min(map.info.resolution for map in maps)
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
    ratio = [0] * (merged_map.info.width * merged_map.info.height)
    
    for k in range(len(maps)): 
        for y in range(maps[k].info.height):
            for x in range(maps[k].info.width):
                i = x + y * maps[k].info.width
                merged_x = int(np.floor((maps[k].info.origin.position.x + xd[k] + x * maps[k].info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((maps[k].info.origin.position.y + yd[k] + y * maps[k].info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width
                if merged_map.data[merged_i] == -1:
                    ratio[merged_i] = 1
                    merged_map.data[merged_i] = maps[k].data[i]
                else: 
                    ratio[merged_i] += 1  
                    merged_map.data[merged_i] = int(((ratio[merged_i] - 1) / ratio[merged_i]) * merged_map.data[merged_i] 
                                                    + (1 / ratio[merged_i]) * maps[k].data[i])  

    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, 'merge_map', 10)  

        for i in range(n):
            topic_name = f'/{maps_list[i]}'
            setattr(self, f'subscription_{i}', self.create_subscription(
                OccupancyGrid, 
                topic_name, 
                lambda msg, l=i: self.map_callback(msg, l), 10))
            setattr(self, maps_list[i], None)

    def map_callback(self, msg, map_id):
            setattr(self, maps_list[map_id], msg)
            if all(getattr(self, maps_list[j]) is not None for j in range(n) if j != map_id):
                merged_msg = merge_maps(*(getattr(self, maps_list[k]) for k in range(n)))
                self.publisher.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()