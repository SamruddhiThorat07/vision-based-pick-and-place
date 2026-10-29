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
        
        # queue of 1 so we always process the newest frame (with 10 it was working on frames 5-8 sec old)
        self.pcs = self.create_subscription(PointCloud2, '/camera/depth/points',self.pc_callback,1)
        self.obj_details = self.create_publisher(DetectedObjects , '/obj_pose',10)
        self.table_empty = False
        self.last_count = None

        # table area in camera_link frame
        self.min_bound = np.array([1.3, -0.37, -1.5])
        self.max_bound = np.array([1.75, 0.37, 0.3])

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
        
        # read_points already gives a numpy array, list() + np.array() was taking ~0.8 sec
        points_data = np.asarray(pc2.read_points(msg, field_names=field_names, skip_nans=True))

        if len(points_data) == 0:
            return

        x = points_data['x']
        y = points_data['y']
        z = points_data['z']

        points = np.stack([x , y , z ], axis=-1)

        # crop to the table before making the open3d cloud so it only copies ~30% of the points
        in_box = np.all((points >= self.min_bound) & (points <= self.max_bound), axis=1)
        points = points[in_box]

        rgb = points_data['rgb'][in_box]
        rgb_packed = rgb.view(np.uint32)
        
        r = (rgb_packed >> 16) & 0x0000ff
        g = (rgb_packed >> 8) & 0x0000ff
        b = (rgb_packed ) & 0x0000ff
        
        colors = np.stack([r,g,b], axis=-1).astype(np.float32)/255.0
        
        o3d_pcd = o3d.geometry.PointCloud()
        # open3d copies float64 arrays much faster than float32
        o3d_pcd.points= o3d.utility.Vector3dVector(points.astype(np.float64))
        o3d_pcd.colors= o3d.utility.Vector3dVector(colors.astype(np.float64))
        
        self.process_point_cloud(o3d_pcd, msg.header)
        
    def process_point_cloud(self, pcd, header):
        
        # cloud is already cropped to the table in pc_callback
        pcd_cropped = pcd.voxel_down_sample(voxel_size = 0.001)
                
        plane_model , inliers = pcd_cropped.segment_plane(distance_threshold=0.02,ransac_n = 3, num_iterations=1000)
        
        table_cloud = pcd_cropped.select_by_index(inliers)
        object_cloud = pcd_cropped.select_by_index(inliers, invert=True)
        
        if len(object_cloud.points) == 0:
            # print only once when the table becomes empty, not every frame
            if not self.table_empty:
                self.get_logger().info("No objects detected on the table")
                self.table_empty = True
            self.last_count = 0

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
            # same as above, print only once
            if not self.table_empty:
                self.get_logger().info("No valid objects detected on the table")
                self.table_empty = True
            self.last_count = 0
            # Publish empty message
            msg = DetectedObjects()
            msg.header = header
            self.obj_details.publish(msg)
            return
    
        self.table_empty = False

        # print the count only when it changes, not every frame
        if num_objects != self.last_count:
            self.get_logger().info(f"{num_objects}")
            self.last_count = num_objects
        
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