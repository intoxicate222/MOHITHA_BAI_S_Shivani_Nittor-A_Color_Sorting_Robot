#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3, TransformStamped 
from std_msgs.msg import ColorRGBA, Header
import math
import time
from tf2_ros import TransformBroadcaster

def euler_to_quaternion(roll, pitch, yaw): # Unchanged
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr; q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr; q.z = sy * cp * cr - cy * sp * sr
    return q

class SorterLogicNode(Node):
    def __init__(self):
        super().__init__('sorter_logic_node')

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.current_base_pos = Point(x=0.0, y=0.0, z=0.0) 
        self.current_base_yaw = 0.0 
        self.target_base_pos = Point(x=0.0, y=0.0, z=0.0)
        self.target_base_yaw = 0.0

        self.timer_period = 0.1 
        self.state_timer = self.create_timer(self.timer_period, self.timer_callback)
        self.animation_period = 0.04 
        self.animation_timer = self.create_timer(self.animation_period, self.animation_step)

        self.robot_joint_names = ['joint1', 'joint2']
        self.current_joint_angles = [0.0, 0.0]
        self.target_joint_angles = [0.0, 0.0] 

        self.objects = [ 
            {'id': 0, 'color_name': 'red',   'initial_pos': Point(x=0.8, y=0.3, z=0.025)}, 
            {'id': 1, 'color_name': 'green', 'initial_pos': Point(x=0.8, y=0.0, z=0.025)},
            {'id': 2, 'color_name': 'blue',  'initial_pos': Point(x=0.8, y=-0.3, z=0.025)},
        ]
        for obj in self.objects:
            obj.update({ 'color_rgba': ColorRGBA(r=1.0 if obj['color_name']=='red' else 0.0, g=1.0 if obj['color_name']=='green' else 0.0, b=1.0 if obj['color_name']=='blue' else 0.0, a=1.0),
                'current_pos': Point(x=obj['initial_pos'].x, y=obj['initial_pos'].y, z=obj['initial_pos'].z),
                'frame_id': 'world', 'picked_up': False, 'size': 0.05, 'sorted': False })
        
        self.boxes = [ 
            {'id': 100, 'color_name': 'red',   'pos': Point(x=-0.4, y=0.8, z=0.05)}, 
            {'id': 101, 'color_name': 'green', 'pos': Point(x=0.0,  y=0.8, z=0.05)},
            {'id': 102, 'color_name': 'blue',  'pos': Point(x=0.4,  y=0.8, z=0.05)},
        ]
        for box in self.boxes:
             box.update({ 'color_rgba': ColorRGBA(r=0.8 if box['color_name']=='red' else 0.1, g=0.8 if box['color_name']=='green' else 0.1, b=0.8 if box['color_name']=='blue' else 0.1, a=0.5),
                'size': Vector3(x=0.15, y=0.15, z=0.1) })

        self.base_drive_target_for_pickup = {'pos': Point(x=0.35, y=0.0, z=0.0), 'yaw': math.radians(0)} 
        self.base_drive_target_for_place = { 
            'red':   {'pos': Point(x=-0.4, y=0.35, z=0.0), 'yaw': math.radians(90)}, 
            'green': {'pos': Point(x=0.0,  y=0.35, z=0.0), 'yaw': math.radians(90)},
            'blue':  {'pos': Point(x=0.4,  y=0.35, z=0.0), 'yaw': math.radians(90)},
        }

        self.arm_home_pose = [0.0, math.radians(-35)] 
        self.arm_approach_pickup_pose = [0.0, math.radians(20)] 
        
        self.arm_lower_for_action_offset_j2 = math.radians(30) 
        
        self.arm_aim_for_ball_j1 = { 
            'red':   math.radians(35),  
            'green': math.radians(0),   
            'blue':  math.radians(-35), 
        }
        self.arm_extend_for_place_pose = [math.radians(0), math.radians(60)] 
        
        self.current_state = "INIT_SEQUENCE"
        self.action_start_time = None 
        self.base_drive_total_duration = 2.0 
        self.arm_maneuver_duration = 1.0 
        self.pick_place_pause_duration = 0.5 
        self.active_object_dict = None
        self.next_object_idx_to_process = 0

        self.get_logger().info("Wheeled Sorter: v3 - Adjusted Lowering, More Logging.")
        self.publish_all_markers()
        self.set_robot_target_joints(self.arm_home_pose)

    
    def publish_robot_base_tf(self): 
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg(); t.header.frame_id = 'world' 
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.current_base_pos.x 
        t.transform.translation.y = self.current_base_pos.y
        t.transform.translation.z = self.current_base_pos.z
        t.transform.rotation = euler_to_quaternion(0.0, 0.0, self.current_base_yaw) 
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"TF Pub: Base Pos ({self.current_base_pos.x:.2f},{self.current_base_pos.y:.2f}), Yaw: {math.degrees(self.current_base_yaw):.1f}")

    def animation_step(self): 
        if self.current_state in ["DRIVING_TO_PICKUP_AREA", "DRIVING_TO_DROPOFF_ZONE"]: 
            elapsed_time = time.time() - self.action_start_time
            drive_ratio = min(elapsed_time / self.base_drive_total_duration, 1.0)
            if not hasattr(self, 'drive_start_pos_x'): 
                self.drive_start_pos_x = self.current_base_pos.x; self.drive_start_pos_y = self.current_base_pos.y
                self.drive_start_yaw = self.current_base_yaw
            self.current_base_pos.x = self.drive_start_pos_x + (self.target_base_pos.x - self.drive_start_pos_x) * drive_ratio
            self.current_base_pos.y = self.drive_start_pos_y + (self.target_base_pos.y - self.drive_start_pos_y) * drive_ratio
            yaw_diff = self.target_base_yaw - self.drive_start_yaw
            while yaw_diff > math.pi: yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi: yaw_diff += 2 * math.pi
            self.current_base_yaw = self.drive_start_yaw + yaw_diff * drive_ratio
            if drive_ratio >= 1.0: 
                if hasattr(self, 'drive_start_pos_x'): del self.drive_start_pos_x
                if hasattr(self, 'drive_start_pos_y'): del self.drive_start_pos_y
                if hasattr(self, 'drive_start_yaw'): del self.drive_start_yaw
        self.publish_robot_base_tf() 
        angular_step_size = math.radians(4.0) 
        moved_any_joint = False; new_angles = list(self.current_joint_angles)
        for i in range(len(self.robot_joint_names)):
            angle_difference = self.target_joint_angles[i] - self.current_joint_angles[i]
            if abs(angle_difference) > math.radians(0.5): 
                change_this_step = max(min(angle_difference, angular_step_size), -angular_step_size)
                new_angles[i] += change_this_step; moved_any_joint = True
        if moved_any_joint:
            self.current_joint_angles = new_angles
            js_msg = JointState(); js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = self.robot_joint_names; js_msg.position = self.current_joint_angles
            self.joint_state_pub.publish(js_msg)
        self.publish_all_markers() 

    def set_robot_target_joints(self, joint_values_list): self.target_joint_angles = list(joint_values_list) 
    def create_sphere_marker(self, obj_info_dict): 
        marker = Marker()
        marker.header.frame_id = obj_info_dict['frame_id'] 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objects_to_sort"
        marker.id = obj_info_dict['id']
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        if obj_info_dict['picked_up'] and obj_info_dict['frame_id'] != 'gripper_link':
            self.get_logger().warn(f"!!! MARKER BUG CHECK: Object ID {obj_info_dict['id']} ({obj_info_dict['color_name']}) is picked_up=True but frame_id is '{obj_info_dict['frame_id']}' during marker creation.")
        elif obj_info_dict['frame_id'] == 'gripper_link':
             self.get_logger().info(f"MARKER TRACE (CREATE): ID {obj_info_dict['id']} ATTACHED to gripper_link. Rel Pos: ({obj_info_dict['current_pos'].x:.2f}, {obj_info_dict['current_pos'].y:.2f}, {obj_info_dict['current_pos'].z:.2f})")
        elif not obj_info_dict['picked_up'] and obj_info_dict['frame_id'] == 'world':
             self.get_logger().debug(f"MARKER TRACE (CREATE): ID {obj_info_dict['id']} in WORLD. World Pos: ({obj_info_dict['current_pos'].x:.2f}, {obj_info_dict['current_pos'].y:.2f}, {obj_info_dict['current_pos'].z:.2f})")
        marker.pose.position = obj_info_dict['current_pos']
        marker.pose.orientation.w = 1.0 
        marker.scale.x = obj_info_dict['size']; marker.scale.y = obj_info_dict['size']; marker.scale.z = obj_info_dict['size']
        marker.color = obj_info_dict['color_rgba']
        marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg() 
        return marker
    def create_box_marker(self, box_info_dict): 
        marker = Marker()
        marker.header.frame_id = "world"; marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sorting_boxes"; marker.id = box_info_dict['id']; marker.type = Marker.CUBE
        marker.action = Marker.ADD; marker.pose.position = box_info_dict['pos']
        marker.pose.orientation.w = 1.0; marker.scale = box_info_dict['size']
        marker.color = box_info_dict['color_rgba']; marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
        return marker
    def publish_all_markers(self): 
        marker_array_msg = MarkerArray()
        for obj_dict in self.objects: marker_array_msg.markers.append(self.create_sphere_marker(obj_dict))
        for box_dict in self.boxes: marker_array_msg.markers.append(self.create_box_marker(box_dict))
        self.marker_pub.publish(marker_array_msg)

    def timer_callback(self):
        current_time = time.time()
        
        if self.current_state == "CHOOSE_OBJECT": 
            self.get_logger().info(f"--- Entering CHOOSE_OBJECT --- next_object_idx_to_process: {self.next_object_idx_to_process}")
            for idx, obj_s in enumerate(self.objects): # Log status of all objects
                 self.get_logger().info(f"  Obj ID {obj_s['id']} ({obj_s['color_name']}): sorted={obj_s['sorted']}, picked_up={obj_s['picked_up']}, frame_id='{obj_s['frame_id']}'")

            self.active_object_dict = None; found_object_this_cycle = False
            for i in range(self.next_object_idx_to_process, len(self.objects)):
                if not self.objects[i]['sorted']: 
                    self.active_object_dict = self.objects[i]
                    found_object_this_cycle = True
                    self.get_logger().info(f"CHOOSE_OBJECT: Found unsorted object: ID {self.active_object_dict['id']} ({self.active_object_dict['color_name']}) at index {i}")
                    break
            
            if found_object_this_cycle and self.active_object_dict:
                self.get_logger().info(f"Target: {self.active_object_dict['color_name']}. Driving to PICKUP_AREA.")
                self.target_base_pos = self.base_drive_target_for_pickup['pos']
                self.target_base_yaw = self.base_drive_target_for_pickup['yaw']
                self.drive_start_pos_x = self.current_base_pos.x; self.drive_start_pos_y = self.current_base_pos.y; self.drive_start_yaw = self.current_base_yaw
                self.current_state = "DRIVING_TO_PICKUP_AREA"
                self.action_start_time = current_time
            elif not found_object_this_cycle and self.next_object_idx_to_process >= len(self.objects):
                self.get_logger().info("All objects sorted. FINISHED."); self.current_state = "FINISHED"
            else: 
                self.get_logger().warn(f"No unsorted object found starting from index {self.next_object_idx_to_process}, but not all seem processed. Resetting index and retrying CHOOSE_OBJECT.")
                self.next_object_idx_to_process = 0 
                self.current_state = "CHOOSE_OBJECT" 
            return 

        
        if self.current_state == "INIT_SEQUENCE": 
            self.get_logger().info("State: INIT -> Arm HOME.")
            self.set_robot_target_joints(self.arm_home_pose)
            self.target_base_pos = Point(x=0.0,y=0.0,z=0.0); self.target_base_yaw = 0.0
            self.current_state = "CHOOSE_OBJECT" 
            

        elif self.current_state == "DRIVING_TO_PICKUP_AREA": 
            if current_time - self.action_start_time >= self.base_drive_total_duration:
                self.get_logger().info(f"Base at pickup. Arm: APPROACH for {self.active_object_dict['color_name']}.")
                self.set_robot_target_joints(self.arm_approach_pickup_pose)
                self.current_state = "ARM_APPROACHING_PICK"
                self.action_start_time = current_time
        elif self.current_state == "ARM_APPROACHING_PICK": 
            if current_time - self.action_start_time >= self.arm_maneuver_duration * 0.5:
                color = self.active_object_dict['color_name']
                self.get_logger().info(f"Arm approached. Arm: AIM for {color}.")
                aim_j1 = self.arm_aim_for_ball_j1.get(color, 0.0) 
                current_approach_j2 = self.arm_approach_pickup_pose[1] 
                self.set_robot_target_joints([aim_j1, current_approach_j2])
                self.current_state = "ARM_AIMING_PICK"
                self.action_start_time = current_time
        elif self.current_state == "ARM_AIMING_PICK": 
            if current_time - self.action_start_time >= self.arm_maneuver_duration * 0.5:
                self.get_logger().info(f"Arm aimed. Arm: LOWER for {self.active_object_dict['color_name']}.")
                lowered_pose = list(self.target_joint_angles) 
                lowered_pose[1] += self.arm_lower_for_action_offset_j2 
                self.set_robot_target_joints(lowered_pose)
                self.current_state = "ARM_LOWERING_PICK"
                self.action_start_time = current_time
        elif self.current_state == "ARM_LOWERING_PICK": 
            if current_time - self.action_start_time >= self.arm_maneuver_duration: 
                self.get_logger().info(f"Arm lowered. Simulating PICK of {self.active_object_dict['color_name']} (ID: {self.active_object_dict['id']}).")
                self.get_logger().info(f"ATTACHING ID {self.active_object_dict['id']}: Old frame: {self.active_object_dict['frame_id']}, Old picked: {self.active_object_dict['picked_up']}")
                self.active_object_dict['frame_id'] = 'gripper_link' 
                self.active_object_dict['current_pos'] = Point(x=0.0, y=0.0, z=0.035) 
                self.active_object_dict['picked_up'] = True
                self.get_logger().info(f"ATTACHED ID {self.active_object_dict['id']}: New frame: {self.active_object_dict['frame_id']}, New picked: {self.active_object_dict['picked_up']}")
                self.publish_all_markers() 
                self.current_state = "ARM_RETRACTING_AFTER_PICK" 
                self.action_start_time = current_time
        elif self.current_state == "ARM_RETRACTING_AFTER_PICK": 
             if current_time - self.action_start_time >= self.arm_maneuver_duration * 0.5: 
                self.get_logger().info(f"Arm retracting after pick.")
                color = self.active_object_dict['color_name']
                aim_j1 = self.arm_aim_for_ball_j1.get(color, 0.0)
                current_approach_j2 = self.arm_approach_pickup_pose[1] 
                self.set_robot_target_joints([aim_j1, current_approach_j2]) 
                self.current_state = "PAUSE_BEFORE_DRIVE_TO_PLACE"
                self.action_start_time = current_time
        elif self.current_state == "PAUSE_BEFORE_DRIVE_TO_PLACE":
            if current_time - self.action_start_time >= self.pick_place_pause_duration:
                color = self.active_object_dict['color_name']
                self.get_logger().info(f"Driving base to place {color}.")
                drive_target_dict_entry = self.base_drive_target_for_place.get(color)
                if drive_target_dict_entry:
                    self.target_base_pos = drive_target_dict_entry['pos']
                    self.target_base_yaw = drive_target_dict_entry['yaw']
                    self.get_logger().info(f"Target base for placing {color}: Pos({self.target_base_pos.x}, {self.target_base_pos.y}), Yaw({math.degrees(self.target_base_yaw)})")
                else: 
                    self.get_logger().error(f"No drive target defined for placing {color}! Using default pickup area.")
                    self.target_base_pos = self.base_drive_target_for_pickup['pos'] 
                    self.target_base_yaw = self.base_drive_target_for_pickup['yaw']
                self.drive_start_pos_x = self.current_base_pos.x; self.drive_start_pos_y = self.current_base_pos.y; self.drive_start_yaw = self.current_base_yaw
                self.current_state = "DRIVING_TO_DROPOFF_ZONE" 
                self.action_start_time = current_time
        elif self.current_state == "DRIVING_TO_DROPOFF_ZONE": 
            if current_time - self.action_start_time >= self.base_drive_total_duration: 
                self.get_logger().info(f"Base at dropoff. Arm: EXTEND for {self.active_object_dict['color_name']}.")
                self.set_robot_target_joints(self.arm_extend_for_place_pose)
                self.current_state = "ARM_EXTENDING_PLACE"
                self.action_start_time = current_time
        elif self.current_state == "ARM_EXTENDING_PLACE": 
            if current_time - self.action_start_time >= self.arm_maneuver_duration:
                self.get_logger().info(f"Arm extended. Simulating PLACE of {self.active_object_dict['color_name']} (ID: {self.active_object_dict['id']}).")
                self.get_logger().info(f"DETACHING ID {self.active_object_dict['id']}: Old frame: {self.active_object_dict['frame_id']}, Old picked: {self.active_object_dict['picked_up']}")
                self.active_object_dict['frame_id'] = 'world' 
                self.active_object_dict['picked_up'] = False
                self.get_logger().info(f"DETACHED ID {self.active_object_dict['id']}: New frame: {self.active_object_dict['frame_id']}, New picked: {self.active_object_dict['picked_up']}")
                # Mark as sorted
                for idx, obj in enumerate(self.objects): 
                    if obj['id'] == self.active_object_dict['id']: 
                        self.objects[idx]['sorted'] = True
                        self.get_logger().info(f"Marked object ID {obj['id']} as sorted = True")
                        break
                # Set final world position
                target_box_pos = self.active_object_dict['initial_pos'] 
                for box in self.boxes: 
                    if box['color_name'] == self.active_object_dict['color_name']: target_box_pos = box['pos']; break
                self.active_object_dict['current_pos'] = Point(x=target_box_pos.x, y=target_box_pos.y, z=target_box_pos.z + 0.035)
                self.publish_all_markers() 
                self.current_state = "ARM_RETRACTING_AFTER_PLACE" 
                self.action_start_time = current_time
        elif self.current_state == "ARM_RETRACTING_AFTER_PLACE": 
            if current_time - self.action_start_time >= self.arm_maneuver_duration * 0.5:
                self.get_logger().info(f"Arm retracting after place. Setting arm HOME.")
                self.next_object_idx_to_process +=1 
                self.get_logger().info(f"Incremented next_object_idx_to_process to: {self.next_object_idx_to_process}")
                self.set_robot_target_joints(self.arm_home_pose)
                self.current_state = "CHOOSE_OBJECT" 
                self.action_start_time = current_time 
        elif self.current_state == "FINISHED": 
            pass 

    def destroy_node(self): # Unchanged
        self.get_logger().info("Shutting down Sorter Logic Node.")
        if hasattr(self, 'state_timer') and self.state_timer: self.state_timer.cancel()
        if hasattr(self, 'animation_timer') and self.animation_timer: self.animation_timer.cancel()
        super().destroy_node()
def main(args=None): # Unchanged
    rclpy.init(args=args)
    node = SorterLogicNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()