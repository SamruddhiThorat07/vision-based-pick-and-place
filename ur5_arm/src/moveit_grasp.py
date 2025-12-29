#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped,Pose,PointStamped
from ur5_msgs.msg import DetectedObject,DetectedObjects
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
import time
from pymoveit2 import MoveIt2 , GripperInterface
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from enum import Enum
from geometry_msgs.msg import Point

class State(Enum):
    IDLE = 0
    DETECT_OBJECTS = 1
    PRE_GRASP = 2
    GRASP = 3
    VERIFY_GRASP = 4
    LIFT = 5
    PLACE = 6
    RELEASE = 7
    RECOVERY = 8

class MoveArm(Node):
    def __init__(self):
        super().__init__("move_arm")
        
        self.callback_group = ReentrantCallbackGroup()
        
        arm_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        
        self.ur5_arm = MoveIt2(
            node=self,
            joint_names=arm_joint_names,
            base_link_name="base_link",
            end_effector_name="tool0",
            group_name="ur5_arm",
            callback_group=self.callback_group
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        
        self.add_table_collision()
        # self.control_gripper("open")
                
        self.create_subscription(DetectedObjects,"/obj_pose",self.detected_obj_cb,10,callback_group=self.callback_group)
        self.create_subscription(JointState, "/joint_states",self.joint_state_cb,10,callback_group=self.callback_group)
        self.gripper_client = ActionClient(self,GripperCommand,"/robotiq_gripper_controller/gripper_cmd",callback_group=self.callback_group)
        
        self.bin_locations = {
            "red": [-0.35, -0.5, 0.3], 
            "green": [-0.12, -0.5, 0.3],
            "blue": [ 0.12, -0.5, 0.3],
            "unknown": [ 0.35, -0.5, 0.3]
        }
        
        self.arm_home_pos = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self.detected_objects = []
        self.current_state = State.IDLE
        self.frame_id = None
        self.gripper_pos = None
        self.obj_point = None
        self.current_obj = None
        self.is_moving = False
        self.grasp_retry = 0 
        
        # self.timer = self.create_timer(0.5,self.check_joint_states,callback_group=self.callback_group)
        
        self.timer = self.create_timer(0.5,self.arm_callback,callback_group=self.callback_group)
        
    def joint_state_cb(self,msg):
        gripper_id = msg.name.index("robotiq_85_left_knuckle_joint")
        self.gripper_pos = msg.position[gripper_id]
        
    def check_joint_states(self):
        if self.ur5_arm.joint_state is not None:
            return True
        else:
            self.get_logger().info("joint state not available")
            return False
        
    def add_table_collision(self):
        
        self.ur5_arm.add_collision_box(
            id="table",
            size=(1.5, 0.8, 0.05),
            position=[0.0, 0.6, -0.05],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            frame_id="base_link"
        )
        
        self.get_logger().info("Table collision added to scene")

    def detected_obj_cb(self,msg):  
        self.frame_id = msg.header.frame_id  
        self.detected_objects = msg.objects
              
    def control_gripper(self, state):
        
        self.get_logger().info(f"trying to {state} gripper")
        self.is_moving = True
    
        goal = GripperCommand.Goal()
    
        if state == "open":
            goal.command.position = 0.0
            goal.command.max_effort = 50.0
        elif state == "close":
            goal.command.position = 0.8
            goal.command.max_effort = 100.0
        else:
            self.get_logger().error("Invalid gripper state")
            return False
    
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Gripper action server not available")
            self.is_moving = False
            return False
        
        goal_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=2.0)
        
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            self.is_moving = False
            return False
    
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=3.0)
    
        self.get_logger().info(f"changed to {state} gripper")
        self.is_moving = False
        return True
        
    def get_transform(self, obj_point):
        
        point_stamped = PointStamped()   
        point_stamped.header.frame_id = self.frame_id
        point_stamped.point = obj_point
        
        to_frame = "base_link"
        from_frame = point_stamped.header.frame_id
        
        try:
               t = self.tf_buffer.lookup_transform(to_frame,from_frame,rclpy.time.Time())
               transform_point = do_transform_point(point_stamped, t)
               return transform_point
        except TransformException as ex:
                self.get_logger().info(f'Could not transform {to_frame} to {from_frame}: {ex}')
                return
            
    def go_to_pose(self,obj_point,z_offset,is_cartesian):
        self.is_moving = True
        pose = Pose()
        
        pose.position.x = obj_point.point.x
        pose.position.y = obj_point.point.y
        pose.position.z = obj_point.point.z + z_offset
        
        pose.orientation.w = 0.0
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        
        self.ur5_arm.move_to_pose(
            position=[pose.position.x, pose.position.y, pose.position.z],
            quat_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            frame_id="base_link",
            cartesian=is_cartesian,
            tolerance_position=0.001,
            tolerance_orientation=0.001
        )
        
        self.ur5_arm.wait_until_executed()
        self.is_moving = False
        error = self.ur5_arm.get_last_execution_error_code()
        error_val = error.val if hasattr(error, 'val') else error
        
        if error_val is None or error_val == 1:
            return True
        else:
            self.get_logger().error(f"Motion failed with error: {error}")
            return False
        
    def go_to_home_pose(self):
        self.is_moving = True
        self.ur5_arm.move_to_configuration(
            joint_positions=self.arm_home_pos,
            tolerance=0.001
        )
        
        self.ur5_arm.wait_until_executed()
        self.is_moving = False
        error = self.ur5_arm.get_last_execution_error_code()
        error_val = error.val if hasattr(error, 'val') else error
        
        if error_val is None or error_val == 1:
            return True
        else:
            self.get_logger().error(f"Motion failed with error: {error}")
            return False        
        
    def arm_callback(self): 
        
        if self.is_moving:
            # self.get_logger().info(f"Skipping callback - is_moving: {self.is_moving}")
            return
        
        if not self.check_joint_states():
            return
        
        self.get_logger().info(f"Current state: {self.current_state.name}")
        
        # idle
        if self.current_state== State.IDLE:
            if len(self.detected_objects) > 0:
                self.get_logger().info("idle --> detected_obj")
                self.current_state = State.DETECT_OBJECTS
            else:
                self.get_logger().info("No object detected on the table pick and plcae successful")
                
        # object detection
        elif self.current_state == State.DETECT_OBJECTS:
            self.get_logger().info(f"{len(self.detected_objects)} objects detected")
            self.current_obj = self.detected_objects[0]
            self.get_logger().info(f"Selected {self.current_obj} to grasp")
            self.current_state = State.PRE_GRASP
            
        # go to pre grasp position
        elif self.current_state == State.PRE_GRASP:
            if self.obj_point is None: 
                self.obj_point = self.get_transform(self.current_obj.point)
                if self.obj_point is None:
                    self.get_logger().error("transform failed")
                    self.current_state = State.IDLE
                    return
            
            if self.go_to_pose(self.obj_point,z_offset=0.4,is_cartesian = True):
     
                if self.control_gripper("open"):              
                    self.current_state = State.GRASP
                    self.get_logger().info(f"going to grasp mode")
                    return
                else:
                    self.get_logger().info("trying again")
            else:
                self.get_logger().info("trying again")
                
        # grasp the object
        elif self.current_state == State.GRASP:
            
            self.get_logger().info("grasp mode")
            if self.go_to_pose(self.obj_point,z_offset=0.12,is_cartesian = True):
                if self.control_gripper("close"):  
                    self.current_state = State.VERIFY_GRASP
                elif self.grasp_retry>2:
                    self.get_logger().info("trying again")
                    self.grasp_retry +=1
                else:
                    self.current_state = State.RECOVERY
            else:
                self.get_logger().info("trying again")
                
        # verify the grasp 
        elif self.current_state ==State.VERIFY_GRASP:
            
            if self.gripper_pos == None:
                self.get_logger().info("gripper pose not available")
                return 
            
            if self.gripper_pos > 0.7 or self.gripper_pos<0.1:
                self.get_logger().info("Object grasp failed trying again")
                self.get_logger().info("Going to home position")
                self.current_state = State.RECOVERY
                
            else:
                self.get_logger().info(f"Object grasped successfully going to {self.current_obj.color} bin")
                self.current_state = State.LIFT
                
        # lift the arm
        elif self.current_state ==State.LIFT:
            
            if self.go_to_pose(self.obj_point,z_offset=0.6,is_cartesian = True):
            # if self.go_to_home_pose():
                self.current_state = State.PLACE
            else:
                self.get_logger().info("trying again")
        
        # place the object
        elif self.current_state ==State.PLACE:
            
            bin_point = PointStamped()
            bin_point.point.x, bin_point.point.y, bin_point.point.z = self.bin_locations[self.current_obj.color]
            
            if self.go_to_pose(bin_point,z_offset=0.0,is_cartesian = False):
                
                if self.gripper_pos>0.7:
                    self.get_logger().info("Object not present")
                    self.get_logger().info("trying again")
                    self.current_state = State.RECOVERY
                    return
                
                if self.control_gripper("open"):  
                    self.current_state = State.RECOVERY
                else:
                    self.get_logger().info("trying again")
            else:
                self.get_logger().info("trying again")
                
        # going to home pose 
        elif self.current_state ==State.RECOVERY:
            
            if self.go_to_home_pose():
                self.grasp_retry = 0
                self.obj_point = None
                self.get_logger().info("home pose")
                self.current_state = State.IDLE
            
                
def main():
    rclpy.init()
    move_arm = MoveArm()
    
    # executor = MultiThreadedExecutor(num_threads=5)
    # executor.add_node(move_arm)
    rclpy.spin(move_arm)
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

