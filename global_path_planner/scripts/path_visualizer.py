#!/usr/bin/env python3

import rospy
import rosbag
import subprocess
import threading
import time
import math
import json
import utm
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import signal
import sys
import os

class AllInOnePathSystem:
    def __init__(self):
        rospy.init_node('all_in_one_path_system', anonymous=True)
        
        # Configuration
        self.bag_file = "/media/danny/DB/JBNU_LIDAR_DATASET_EVAL/2025-01-11-12-05-15.bag"
        self.utm_origin = {"easting": None, "northing": None}
        self.rotation_angle = 0.0
        
        # Data storage
        self.bag_gps_trajectory = []
        self.current_gps = None
        self.current_mag = None
        self.faster_lio_odom = None
        self.latest_waypoints = None
        
        # ESKF state (simplified)
        self.fused_pose = PoseStamped()
        
        # Publishers
        self.bag_marker_pub = rospy.Publisher("/bag_gps_markers", Marker, queue_size=10)
        self.global_marker_pub = rospy.Publisher("/global_path_markers", Marker, queue_size=10)
        self.fused_pose_pub = rospy.Publisher("/fused_pose", PoseStamped, queue_size=10)
        self.fused_odom_pub = rospy.Publisher("/fused_odometry", Odometry, queue_size=10)
        
        # Subscribers (for real-time fusion)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/imu/mag", Vector3Stamped, self.mag_callback)
        rospy.Subscriber("/faster_lio/odometry", Odometry, self.flio_callback)
        
        # Timers
        rospy.Timer(rospy.Duration(1.0), self.publish_bag_trajectory)
        rospy.Timer(rospy.Duration(0.1), self.publish_fused_pose)  # 10Hz fusion
        
        # TF broadcaster for coordinate frame alignment
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Calculate and publish map→camera_init transform
        self.publish_map_to_camera_transform()
        
        # Start initialization
        self.initialize_system()
    
    def publish_map_to_camera_transform(self):
        """Publish transform from map (UTM) to camera_init (FasterLIO)"""
        if not self.bag_gps_trajectory:
            rospy.logwarn("⚠️ GPS 궤적이 없어 TF 생성 생략")
            return
            
        if self.utm_origin["easting"] is None:
            rospy.logwarn("⚠️ UTM 원점이 없어 TF 생성 생략")
            return
        
        # Use first GPS point as the alignment reference
        first_gps = self.bag_gps_trajectory[0]
        utm_x, utm_y = self.transform_coordinates(first_gps["lat"], first_gps["lon"])
        
        rospy.loginfo(f"🔗 TF 생성 시작: 회전각 {math.degrees(self.rotation_angle):.1f}°")
    
        # tf2_ros로 직접 TF 브로드캐스트 (안정적)
        def broadcast_static_tf():
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"
            transform.child_frame_id = "camera_init"
            
            transform.transform.translation.x = utm_x
            transform.transform.translation.y = utm_y
            transform.transform.translation.z = 0.0
            
            cos_half = math.cos(self.rotation_angle / 2.0)
            sin_half = math.sin(self.rotation_angle / 2.0)
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = sin_half
            transform.transform.rotation.w = cos_half
            
            self.tf_broadcaster.sendTransform(transform)
        def start_tf_timer():
            if rospy.is_shutdown():
                return
            rospy.Timer(rospy.Duration(0.1), lambda event: broadcast_static_tf())
            rospy.loginfo(f"🔗 자동 TF 브로드캐스트 시작 완료!")
        
        threading.Timer(1.0, start_tf_timer).start()
        rospy.loginfo(f"🔗 TF 생성 예약됨 (1초 후 시작)")

    def signal_handler(self, sig, frame):
        """Clean shutdown"""
        rospy.loginfo("🛑 시스템 종료 중...")
        sys.exit(0)
    
    def initialize_system(self):
        """Initialize the complete system"""
        rospy.loginfo("🚀 통합 Path 시스템 초기화 중...")
        
        # 1. Load UTM origin if available
        self.load_utm_origin()
        
        # 2. Extract trajectory from bag
        self.extract_bag_trajectory()
        
        # 3. Calculate rotation angle
        self.calculate_rotation_angle()

        self.publish_map_to_camera_transform()
        
        # 4. Start bag playback (after delay)
        threading.Timer(3.0, self.start_bag_playback).start()
        
        rospy.loginfo("✅ 시스템 초기화 완료!")
    
    def load_utm_origin(self):
        """Load UTM origin from file"""
        try:
            with open(os.path.expanduser("~/utm_origin.json"), "r") as f:
                config = json.load(f)
                self.utm_origin["easting"] = config["utm_origin"]["easting"]
                self.utm_origin["northing"] = config["utm_origin"]["northing"]
                rospy.loginfo(f"📍 UTM 원점 로드: {self.utm_origin}")
        except Exception as e:
            rospy.logwarn(f"⚠️ UTM 원점 파일 로드 실패: {e}")
    
    def extract_bag_trajectory(self):
        """Extract GPS trajectory from bag file"""
        if not os.path.exists(self.bag_file):
            rospy.logerr(f"❌ Bag 파일을 찾을 수 없습니다: {self.bag_file}")
            return
        
        rospy.loginfo(f"📂 Bag 파일에서 GPS 궤적 추출 중...")
        
        try:
            with rosbag.Bag(self.bag_file, 'r') as bag:
                gps_points = []
                prev_time = None
                
                for topic, msg, t in bag.read_messages(topics=['/ublox/fix']):
                    # Sample every 1 second
                    if prev_time and (t - prev_time).to_sec() < 1.0:
                        continue
                    
                    if msg.status.status >= 0:  # GPS Fix OK
                        gps_point = {
                            "lat": msg.latitude,
                            "lon": msg.longitude,
                            "timestamp": t.to_sec()
                        }
                        gps_points.append(gps_point)
                        prev_time = t
                
                # Filter points (5m minimum distance)
                filtered_points = []
                prev_utm = None
                
                for point in gps_points:
                    curr_utm = utm.from_latlon(point["lat"], point["lon"])
                    
                    if prev_utm is None or \
                       math.sqrt((curr_utm[0]-prev_utm[0])**2 + (curr_utm[1]-prev_utm[1])**2) > 5.0:
                        filtered_points.append(point)
                        prev_utm = curr_utm
                
                self.bag_gps_trajectory = filtered_points
                
                # Set UTM origin from first GPS point if not available
                if self.utm_origin["easting"] is None and self.bag_gps_trajectory:
                    first_point = self.bag_gps_trajectory[0]
                    first_utm = utm.from_latlon(first_point["lat"], first_point["lon"])
                    self.utm_origin["easting"] = first_utm[0]
                    self.utm_origin["northing"] = first_utm[1]
                    rospy.loginfo(f"📍 Bag 기준 UTM 원점 설정: {self.utm_origin}")
                
                rospy.loginfo(f"✅ GPS 궤적 추출 완료: {len(self.bag_gps_trajectory)}개 포인트")
                
        except Exception as e:
            rospy.logerr(f"❌ Bag 파일 읽기 실패: {e}")
    
    def calculate_rotation_angle(self):
        """Calculate rotation angle from magnetometer and GPS movement"""
        # Method 1: Magnetometer from bag
        theta_mag = self.calculate_theta_from_magnetometer()
        
        # Method 2: GPS movement direction  
        theta_gps = self.calculate_theta_from_gps_movement()
        
        # Use magnetometer if available, otherwise GPS movement
        self.rotation_angle = theta_mag if abs(theta_mag) > 0.001 else theta_gps
        
        rospy.loginfo(f"🧭 Magnetometer 회전각: {math.degrees(theta_mag):.1f}°")
        rospy.loginfo(f"📍 GPS 이동 회전각: {math.degrees(theta_gps):.1f}°")
        rospy.loginfo(f"🔄 최종 회전각: {math.degrees(self.rotation_angle):.1f}°")
    
    def calculate_theta_from_magnetometer(self):
        """Calculate rotation angle from magnetometer data in bag"""
        try:
            with rosbag.Bag(self.bag_file, 'r') as bag:
                mag_readings = []
                start_time = None
                
                for topic, msg, t in bag.read_messages(topics=['/imu/mag']):
                    if start_time is None:
                        start_time = t
                    elif (t - start_time).to_sec() > 10.0:  # First 10 seconds
                        break
                    
                    mag_readings.append({
                        "x": msg.vector.x,
                        "y": msg.vector.y,
                        "z": msg.vector.z
                    })
                
                if mag_readings:
                    # Average magnetometer values
                    avg_mag_x = sum(m["x"] for m in mag_readings) / len(mag_readings)
                    avg_mag_y = sum(m["y"] for m in mag_readings) / len(mag_readings)
                    
                    # Calculate heading (true north reference)
                    magnetic_heading = math.atan2(avg_mag_y, avg_mag_x)
                    
                    # Convert UTM North to ROS X-axis (forward)
                    utm_to_ros_rotation = magnetic_heading - math.pi/2
                    
                    return utm_to_ros_rotation
                    
        except Exception as e:
            rospy.logwarn(f"⚠️ Magnetometer 데이터 읽기 실패: {e}")
        
        return 0.0
    
    def calculate_theta_from_gps_movement(self):
        """Calculate rotation angle from GPS movement direction"""
        if len(self.bag_gps_trajectory) < 2:
            return 0.0
        
        # Use first two GPS points for movement direction
        p1 = self.bag_gps_trajectory[0]
        p2 = self.bag_gps_trajectory[1]
        
        utm1 = utm.from_latlon(p1["lat"], p1["lon"])
        utm2 = utm.from_latlon(p2["lat"], p2["lon"])
        
        # Movement vector in UTM coordinates
        dx_utm = utm2[0] - utm1[0]  # Easting
        dy_utm = utm2[1] - utm1[1]  # Northing
        
        # Heading in UTM coordinates (true north reference)
        utm_heading = math.atan2(dy_utm, dx_utm)
        
        # Convert to ROS coordinates (X forward, Y left)
        ros_heading = utm_heading - math.pi/2
        
        return ros_heading
    
    def start_bag_playback(self):
        """Start bag file playback"""
        rospy.loginfo("🎬 Bag 파일 재생 시작...")
        try:
            bag_cmd = f"rosbag play --clock --rate=1.0 {self.bag_file}"
            subprocess.Popen(bag_cmd, shell=True)
        except Exception as e:
            rospy.logerr(f"❌ Bag 재생 실패: {e}")
    
    def gps_callback(self, msg):
        """Real-time GPS data callback"""
        self.current_gps = msg
    
    def mag_callback(self, msg):
        """Real-time magnetometer data callback"""
        self.current_mag = msg
    
    def flio_callback(self, msg):
        """FasterLIO odometry callback"""
        self.faster_lio_odom = msg
    
    def waypoints_callback(self, msg):
        """Web waypoints callback"""
        try:
            waypoints_data = json.loads(msg.data)
            if "waypoints" in waypoints_data:
                self.latest_waypoints = waypoints_data["waypoints"]
                self.visualize_global_waypoints()
                rospy.loginfo(f"🗺️ Global waypoints 수신: {len(self.latest_waypoints)}개")
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 처리 오류: {e}")
    
    def transform_coordinates(self, lat, lon):
        """Transform GPS coordinates to local frame"""
        if self.utm_origin["easting"] is None:
            return 0.0, 0.0
        
        # Convert to UTM
        easting, northing, _, _ = utm.from_latlon(lat, lon)
        
        # Relative to origin
        x = easting - self.utm_origin["easting"]
        y = northing - self.utm_origin["northing"]
        
        # Apply rotation correction
        if abs(self.rotation_angle) > 0.001:
            cos_theta = math.cos(self.rotation_angle)
            sin_theta = math.sin(self.rotation_angle)
            x_rot = cos_theta * x - sin_theta * y
            y_rot = sin_theta * x + cos_theta * y
            return x_rot, y_rot
        
        return x, y
    
    def publish_bag_trajectory(self, event):
        """Publish bag GPS trajectory markers"""
        if not self.bag_gps_trajectory:
            return
        
        # Line marker for trajectory
        line_marker = Marker()
        line_marker.header.frame_id = "map"  # ✅ Bag GPS는 map 프레임 (절대좌표)
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "bag_gps_trajectory"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 3.0
        line_marker.color.r = 1.0  # Red
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        points = []
        for gps_point in self.bag_gps_trajectory:
            x_rot, y_rot = self.transform_coordinates(gps_point["lat"], gps_point["lon"])
            point = Point(x=x_rot, y=y_rot, z=0)
            points.append(point)
        
        line_marker.points = points
        self.bag_marker_pub.publish(line_marker)
        
        # Start and end markers
        if points:
            # Start marker (green sphere)
            start_marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = rospy.Time.now()
            start_marker.ns = "bag_gps_trajectory"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position = points[0]
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = 6.0
            start_marker.scale.y = 6.0
            start_marker.scale.z = 6.0
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0  # Green
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            self.bag_marker_pub.publish(start_marker)
            
            # End marker (yellow sphere)
            end_marker = Marker()
            end_marker.header.frame_id = "map"
            end_marker.header.stamp = rospy.Time.now()
            end_marker.ns = "bag_gps_trajectory"
            end_marker.id = 2
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose.position = points[-1]
            end_marker.pose.orientation.w = 1.0
            end_marker.scale.x = 6.0
            end_marker.scale.y = 6.0
            end_marker.scale.z = 6.0
            end_marker.color.r = 1.0
            end_marker.color.g = 1.0  # Yellow
            end_marker.color.b = 0.0
            end_marker.color.a = 1.0
            self.bag_marker_pub.publish(end_marker)
    
    def visualize_global_waypoints(self):
        """Visualize global waypoints from web"""
        if not self.latest_waypoints:
            return
        
        # Line marker for global path
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "global_waypoints"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 5.0
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0  # Blue
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        points = []
        for wp in self.latest_waypoints:
            x_rot, y_rot = self.transform_coordinates(wp["lat"], wp["lon"])
            point = Point(x=x_rot, y=y_rot, z=0)
            points.append(point)
        
        line_marker.points = points
        self.global_marker_pub.publish(line_marker)
        
        # Waypoint cubes
        for i, pt in enumerate(points):
            cube = Marker()
            cube.header.frame_id = "map"
            cube.header.stamp = rospy.Time.now()
            cube.ns = "global_waypoints"
            cube.id = 100 + i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position = pt
            cube.pose.orientation.w = 1.0
            cube.scale.x = 4.0
            cube.scale.y = 4.0
            cube.scale.z = 0.5
            cube.color.r = 0.0
            cube.color.g = 1.0
            cube.color.b = 1.0  # Cyan
            cube.color.a = 1.0
            self.global_marker_pub.publish(cube)
    
    def publish_fused_pose(self, event):
        """Publish sensor-fused pose (simplified ESKF)"""
        if not (self.current_gps and self.faster_lio_odom):
            return
        
        # Proper coordinate frame fusion
        try:
            # GPS absolute position in map frame
            gps_x, gps_y = self.transform_coordinates(
                self.current_gps.latitude, 
                self.current_gps.longitude
            )
            
            # FasterLIO position (camera_init frame) - need to transform to map frame
            flio_x_raw = self.faster_lio_odom.pose.pose.position.x
            flio_y_raw = self.faster_lio_odom.pose.pose.position.y
            
            # Transform FasterLIO coordinates to map frame using rotation angle
            cos_theta = math.cos(self.rotation_angle)
            sin_theta = math.sin(self.rotation_angle)
            
            # Apply same rotation as GPS coordinates  
            flio_x_map = cos_theta * flio_x_raw - sin_theta * flio_y_raw
            flio_y_map = sin_theta * flio_x_raw + cos_theta * flio_y_raw
            
            # Now both are in map frame - can fuse properly
            # GPS: absolute position but noisy
            # FasterLIO: smooth relative motion but drifts
            
            # Distance-based weighting
            distance_from_origin = math.sqrt(gps_x**2 + gps_y**2)
            
            if distance_from_origin < 50:  # Close to origin: trust FasterLIO more
                weight_gps = 0.2
                weight_flio = 0.8
            else:  # Far from origin: trust GPS more to prevent drift
                weight_gps = 0.6
                weight_flio = 0.4
            
            fused_x = weight_gps * gps_x + weight_flio * flio_x_map
            fused_y = weight_gps * gps_y + weight_flio * flio_y_map
            
            rospy.loginfo_throttle(5, 
                f"🔄 융합: GPS({gps_x:.1f},{gps_y:.1f}) + FasterLIO({flio_x_map:.1f},{flio_y_map:.1f}) "
                f"→ 결과({fused_x:.1f},{fused_y:.1f}) [가중치 GPS:{weight_gps:.1f}]")
            
            # Create fused pose
            self.fused_pose.header.frame_id = "map"  # ✅ 융합 결과는 map 프레임
            self.fused_pose.header.stamp = rospy.Time.now()
            self.fused_pose.pose.position.x = fused_x
            self.fused_pose.pose.position.y = fused_y
            self.fused_pose.pose.position.z = self.current_gps.altitude
            
            # Use magnetometer for orientation if available
            if self.current_mag:
                mag_heading = math.atan2(self.current_mag.vector.y, self.current_mag.vector.x)
                self.fused_pose.pose.orientation.z = math.sin(mag_heading / 2.0)
                self.fused_pose.pose.orientation.w = math.cos(mag_heading / 2.0)
            else:
                # Use FasterLIO orientation
                self.fused_pose.pose.orientation = self.faster_lio_odom.pose.pose.orientation
            
            # Publish fused pose
            self.fused_pose_pub.publish(self.fused_pose)
            
            # Also publish as odometry
            fused_odom = Odometry()
            fused_odom.header = self.fused_pose.header
            fused_odom.child_frame_id = "base_link"
            fused_odom.pose.pose = self.fused_pose.pose
            
            if self.faster_lio_odom:
                fused_odom.twist = self.faster_lio_odom.twist
            
            self.fused_odom_pub.publish(fused_odom)
            
        except Exception as e:
            rospy.logwarn(f"⚠️ 센서 융합 오류: {e}")

if __name__ == '__main__':
    try:
        system = AllInOnePathSystem()
        rospy.loginfo("🎉 통합 Path 시스템 실행 중...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")