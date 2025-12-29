#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np 
from geometry_msgs.msg import Point
from ur5_msgs.msg import DetectedObject,DetectedObjects

class PointCloud(Node):
    def __init__(self):
        super().__init__('point_cloud')
        
        self.pcs = self.create_subscription(PointCloud2, '/camera/depth/points',self.pc_callback,10)
        self.obj_details = self.create_publisher(DetectedObjects , '/obj_pose',10)
        
    def get_color_name(self, cluster_colors):
        
        avg_rgb = np.mean(cluster_colors,axis=0)
        r,g,b = avg_rgb[0],avg_rgb[1],avg_rgb[2]
        
        if r > 0.5 and g < 0.3 and b < 0.3:
            return "red"
        elif g > 0.5 and r < 0.3 and b < 0.3:
            return "green"
        elif b > 0.5 and r < 0.3 and g < 0.3:
            return "blue"
        return "unknown"
        
    def pc_callback(self,msg):
        
        field_names = ("x","y","z","rgb")
        
        point_cloud_data = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))
        
        if len(point_cloud_data) == 0:
            return    
        
        points_data = np.array(point_cloud_data) # to numpy array 
                  
        x = points_data['x']
        y = points_data['y']
        z = points_data['z']
        
        points = np.stack([x , y , z ], axis=-1)
        
        rgb = points_data['rgb']
        rgb_packed = rgb.view(np.uint32)
        
        r = (rgb_packed >> 16) & 0x0000ff
        g = (rgb_packed >> 8) & 0x0000ff
        b = (rgb_packed ) & 0x0000ff
        
        colors = np.stack([r,g,b], axis=-1).astype(np.float32)/255.0
        
        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points= o3d.utility.Vector3dVector(points)
        o3d_pcd.colors= o3d.utility.Vector3dVector(colors)
        
        self.process_point_cloud(o3d_pcd, msg.header)
        
    def process_point_cloud(self, pcd, header):
        
        pcd_down = pcd.voxel_down_sample(voxel_size = 0.001)
        
        min_bound = [1.0, -0.37, -1.5]
        max_bound = [1.75, 0.37, 1.5]
        
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        pcd_cropped = pcd_down.crop(bbox)
                
        plane_model , inliers = pcd_cropped.segment_plane(distance_threshold=0.02,ransac_n = 3, num_iterations=1000)
        
        table_cloud = pcd_cropped.select_by_index(inliers)
        object_cloud = pcd_cropped.select_by_index(inliers, invert=True)
        
        if len(object_cloud.points) == 0:
            self.get_logger().info("No objects detected on the table")
            
            msg = DetectedObjects()
            msg.header = header
            self.obj_details.publish(msg)
            return
        
        # o3d.io.write_point_cloud("table.ply",table_cloud)
        o3d.io.write_point_cloud("object.ply",object_cloud)
        
        labels = np.array(object_cloud.cluster_dbscan(eps=0.045,min_points=50,print_progress=False))
        
        # frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        # o3d.visualization.draw_geometries([pcd, frame], "PLY Axes Check")
        
        max_label = labels.max()
        num_objects = max_label + 1
        
        if max_label < 0:
            self.get_logger().info("No valid objects detected on the table")
            # Publish empty message
            msg = DetectedObjects()
            msg.header = header
            self.obj_details.publish(msg)
            return
    
        self.get_logger().info(f"{num_objects}")
        
        msg = DetectedObjects()
        msg.header = header
        
        self.poses = Point()
        
        for i in range(num_objects):
            obj = (labels == i)
            
            obj_index = np.where(obj)[0]
            
            object_cloud_i = object_cloud.select_by_index(obj_index.tolist())
            
            obj_bb = object_cloud_i.get_axis_aligned_bounding_box()
            # obj_center = obj_bb.center 
            # obj_extent = obj_bb.extent 
            min_bound = obj_bb.get_min_bound()
            max_bound = obj_bb.get_max_bound()
            
            obj_center = (min_bound + max_bound) / 2.0
            obj_extent = max_bound - min_bound
            volume = obj_extent[0] * obj_extent[1] * obj_extent[2]  
            # print(obj_extent)
            shape = "box"
            cluster_colors = np.asarray(object_cloud_i.colors)
            color = self.get_color_name(cluster_colors)
            # print(color)
            obj_msg = DetectedObject()
            obj_msg.object_id = i
            obj_msg.color = color
            obj_msg.point.x = obj_center[0]
            obj_msg.point.y = obj_center[1]
            obj_msg.point.z = obj_center[2]
            
            msg.objects.append(obj_msg)
            
        self.obj_details.publish(msg)    
        
        
def main(args=None):
    rclpy.init(args=args)
    node = PointCloud()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=='__main__':
    main()