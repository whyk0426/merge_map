import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

PI = math.pi

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, 'merge_map', 10)  

        self.n = 1
        self.theta = [PI/4]
        self.xd = [0]
        self.yd = [0]

        self.maps_list = [f"map{i}" for i in range(self.n)] 

        for i in range(self.n):
            topic_name = f'/{self.maps_list[i]}'
            setattr(self, f'subscription_{i}', self.create_subscription(
                OccupancyGrid, 
                topic_name, 
                lambda msg, l=i: self.map_callback(msg, l), 10))
            setattr(self, self.maps_list[i], None)


    # def figure_out_diff(self, *maps):
    #     for k in range(len(maps)):
    #         robot_x = - maps[k].info.origin.position.x
    #         robot_y = - maps[k].info.origin.position.y

    #         for y in range(maps[k].info.height):
    #             for x in range(maps[k].info.width):
    #                 i = x + y * maps[k].info.width

    #                 x_diff = round(x * maps[k].info.resolution - robot_x, 2)
    #                 y_diff = round(y * maps[k].info.resolution - robot_y, 2)
    #                 theta_diff = math.atan2(y_diff, x_diff)
    #                 degree_diff = round(math.degrees(theta_diff), 2)
    #                 r_diff = round(math.sqrt(x_diff**2 + y_diff**2), 2)

    #                 if ((maps[k].data[i] > 70)and(r_diff < 1)):
    #                     self.get_logger().info(f"data = {maps[k].data[i]}, (r, theta) = ({r_diff, degree_diff})")


    def rotate_map(self, map_data, angle):
        robot_x = - map_data.info.origin.position.x
        robot_y = - map_data.info.origin.position.y
        height = map_data.info.height
        width = map_data.info.width
        resolution = map_data.info.resolution

        rotated_map_data = np.full((height, width), -1, dtype=np.int8)

        cos_th = np.cos(-angle) 
        sin_th = np.sin(-angle)

        for i in range(height):
            for j in range(width):
                rotated_x = j * resolution
                rotated_y = i * resolution

                orig_x = cos_th * (rotated_x - robot_x) - sin_th * (rotated_y - robot_y) + robot_x
                orig_y = sin_th * (rotated_x - robot_x) + cos_th * (rotated_y - robot_y) + robot_y

                orig_j = int(np.floor(orig_x / resolution))
                orig_i = int(np.floor(orig_y / resolution))

                if 0 <= orig_i < height and 0 <= orig_j < width:
                    rotated_map_data[i, j] = map_data.data[orig_i * width + orig_j]

        rotated_map_msg = OccupancyGrid()
        rotated_map_msg.header = map_data.header
        rotated_map_msg.info = map_data.info
        rotated_map_msg.data = rotated_map_data.flatten().tolist()

        return rotated_map_msg

    
    def merge_maps(self, *maps):
        merged_map = OccupancyGrid()
        merged_map.header = maps[0].header  
        merged_map.header.frame_id = 'map'

        min_x = min(map.info.origin.position.x + self.xd[i] for i, map in enumerate(maps))
        min_y = min(map.info.origin.position.y + self.yd[i] for i, map in enumerate(maps))

        max_x = max(map.info.origin.position.x + (map.info.width * map.info.resolution) + self.xd[i] for i, map in enumerate(maps))
        max_y = max(map.info.origin.position.y + (map.info.height * map.info.resolution) + self.yd[i] for i, map in enumerate(maps)) 

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
                    merged_x = int(np.floor((maps[k].info.origin.position.x + self.xd[k] + x * maps[k].info.resolution - min_x) / merged_map.info.resolution))
                    merged_y = int(np.floor((maps[k].info.origin.position.y + self.yd[k] + y * maps[k].info.resolution - min_y) / merged_map.info.resolution))
                    merged_i = merged_x + merged_y * merged_map.info.width
                    if merged_map.data[merged_i] == -1:
                        ratio[merged_i] = 1
                        merged_map.data[merged_i] = maps[k].data[i]
                    else: 
                        ratio[merged_i] += 1  
                        merged_map.data[merged_i] = int(((ratio[merged_i] - 1) / ratio[merged_i]) * merged_map.data[merged_i] 
                                                        + (1 / ratio[merged_i]) * maps[k].data[i])  

        return merged_map


    def map_callback(self, msg, map_id):
        setattr(self, self.maps_list[map_id], msg)
        if all(getattr(self, self.maps_list[j]) is not None for j in range(self.n) if j != map_id):
            rotated_maps = [self.rotate_map(getattr(self, self.maps_list[k]), self.theta[k]) for k in range(self.n)]  
            merged_msg = self.merge_maps(*rotated_maps) 
            self.publisher.publish(merged_msg)  


def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
