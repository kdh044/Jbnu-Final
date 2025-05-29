#!/usr/bin/env python3

import rospy
import rosbag
import json
import subprocess
import threading
import time
import utm
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from geometry_msgs.msg import TransformStamped

class SimplifiedPathVisualizer:
    def __init__(self):
        rospy.init_node('simplified_path_visualizer', anonymous=True)
        
        # Configuration
        self.bag_file = "/media/danny/DB/JBNU_LIDAR_DATASET_EVAL/2025-01-11-11-55-34.bag"
        
        # GPS origin from bag file
        self.gps_origin = None  # {"lat": ..., "lon": ...}
        self.utm_origin = None  # {"easting": ..., "northing": ...}
        
        # Data storage
        self.fasterlio_trajectory = []  # FasterLIO 경로 저장
        self.latest_waypoints = None    # Global waypoints
        
        # Publishers
        self.path_marker_pub = rospy.Publisher("/fasterlio_path", Marker, queue_size=10)
        self.waypoints_marker_pub = rospy.Publisher("/global_waypoints", Marker, queue_size=10)
        self.gps_pub = rospy.Publisher("/gps_data", String, queue_size=10)  # GPS 데이터 발행용
        
        # TF broadcaster for map frame
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)  # FasterLIO 출력 토픽
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        
        # Timer for visualization
        rospy.Timer(rospy.Duration(0.5), self.publish_path_markers)
        
        # Timer for TF broadcasting (map -> odom)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_map_tf)
        
        # Timer for GPS data publishing (카카오맵 웹서버용)
        rospy.Timer(rospy.Duration(1.0), self.publish_gps_data)
        
        rospy.loginfo("🚀 간소화된 경로 시각화 시스템 시작!")
        
        # Initialize GPS origin from bag file
        self.extract_gps_origin()
        
        # Bag 파일 재생 시작 (3초 후)
        threading.Timer(3.0, self.start_bag_playback).start()
    
    def publish_gps_data(self, event):
        """GPS 원점 데이터를 /gps_data 토픽으로 발행 (카카오맵 웹서버용)"""
        if self.gps_origin:
            gps_data = {
                "latitude": self.gps_origin["lat"],
                "longitude": self.gps_origin["lon"]
            }
            self.gps_pub.publish(json.dumps(gps_data))
            rospy.loginfo_throttle(10, f"📡 GPS 데이터 발행: {gps_data}")
    
    def extract_gps_origin(self):
        """Bag 파일에서 첫 번째 GPS 좌표 추출"""
        rospy.loginfo("📍 Bag 파일에서 GPS 원점 추출 중...")
        try:
            with rosbag.Bag(self.bag_file, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=['/ublox/fix']):
                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                        if msg.status.status >= 0:  # GPS Fix OK
                            self.gps_origin = {
                                "lat": msg.latitude,
                                "lon": msg.longitude
                            }
                            
                            # UTM 좌표로 변환
                            easting, northing, zone_num, zone_letter = utm.from_latlon(
                                msg.latitude, msg.longitude
                            )
                            self.utm_origin = {
                                "easting": easting,
                                "northing": northing,
                                "zone": f"{zone_num}{zone_letter}"
                            }
                            
                            rospy.loginfo(f"🎯 GPS 원점 설정: {self.gps_origin}")
                            rospy.loginfo(f"🗺️ UTM 원점: {self.utm_origin}")
                            break
                            
            if not self.gps_origin:
                rospy.logwarn("⚠️ GPS 원점을 찾을 수 없습니다. 기본값 사용")
                # 기본값 (전북대 위치)
                self.gps_origin = {"lat": 35.8450893, "lon": 127.13304149999999}
                easting, northing, zone_num, zone_letter = utm.from_latlon(
                    self.gps_origin["lat"], self.gps_origin["lon"]
                )
                self.utm_origin = {
                    "easting": easting,
                    "northing": northing,
                    "zone": f"{zone_num}{zone_letter}"
                }
                
        except Exception as e:
            rospy.logerr(f"❌ GPS 원점 추출 실패: {e}")
            # 기본값 사용
            self.gps_origin = {"lat": 35.8450893, "lon": 127.13304149999999}
    
    def gps_to_local_utm(self, lat, lon):
        """GPS 좌표를 로컬 UTM 좌표로 변환"""
        if not self.utm_origin:
            return 0.0, 0.0
            
        easting, northing, _, _ = utm.from_latlon(lat, lon)
        x = easting - self.utm_origin["easting"]
        y = northing - self.utm_origin["northing"]
        return x, y
    
    def broadcast_map_tf(self, event):
        """map -> odom TF 브로드캐스트 (camera_init 대신 odom 사용)"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"  # camera_init -> odom 수정
        
        # Identity transform (map과 odom을 동일하게 설정)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
    
    def start_bag_playback(self):
        """Bag 파일 재생 시작"""
        rospy.loginfo("🎬 Bag 파일 재생 시작...")
        try:
            # LiDAR 및 IMU 토픽만 재생 (FasterLIO 입력용)
            bag_cmd = f"rosbag play --clock --rate=1.0 {self.bag_file}"
            rospy.loginfo(f"명령어: {bag_cmd}")
            subprocess.Popen(bag_cmd, shell=True)
        except Exception as e:
            rospy.logerr(f"❌ Bag 재생 실패: {e}")
    
    def fasterlio_callback(self, msg):
        """FasterLIO odometry 콜백"""
        pos = msg.pose.pose.position
        pt = {
            "x": pos.x, 
            "y": pos.y, 
            "z": pos.z, 
            "timestamp": msg.header.stamp.to_sec()
        }
        
        # 1미터 이상 이동했을 때만 경로에 추가 (메모리 절약)
        if not self.fasterlio_trajectory or (
            (pt["x"] - self.fasterlio_trajectory[-1]["x"])**2 + 
            (pt["y"] - self.fasterlio_trajectory[-1]["y"])**2
        ) > 1.0:
            self.fasterlio_trajectory.append(pt)
            rospy.loginfo_throttle(5, f"📍 FasterLIO 경로 포인트: {len(self.fasterlio_trajectory)}개")
    
    def waypoints_callback(self, msg):
        """Global waypoints 콜백"""
        try:
            waypoints_data = json.loads(msg.data)
            if "waypoints" in waypoints_data:
                self.latest_waypoints = waypoints_data["waypoints"]
                rospy.loginfo(f"🗺️ Global waypoints 수신: {len(self.latest_waypoints)}개")
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 처리 오류: {e}")
    
    def publish_path_markers(self, event):
        """경로 마커 발행"""
        # FasterLIO 경로 시각화
        if self.fasterlio_trajectory:
            self.visualize_fasterlio_path()
        
        # Global waypoints 시각화
        if self.latest_waypoints:
            self.visualize_global_waypoints()
    
    def visualize_fasterlio_path(self):
        """FasterLIO 경로 시각화"""
        line_marker = Marker()
        line_marker.header.frame_id = "odom"  # camera_init -> odom 수정
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "fasterlio_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 2.0  # 선 두께
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0  # 초록색
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        # 포인트 추가
        points = []
        for pt in self.fasterlio_trajectory:
            point = Point(x=pt["x"], y=pt["y"], z=pt["z"])
            points.append(point)
        
        line_marker.points = points
        self.path_marker_pub.publish(line_marker)
    
    def visualize_global_waypoints(self):
        """Global waypoints 시각화 (GPS->UTM 변환하여 odom 프레임에 표시)"""
        # 라인으로 연결
        line_marker = Marker()
        line_marker.header.frame_id = "odom"  # GPS waypoints도 odom 프레임에 표시
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "global_waypoints_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 3.0
        line_marker.color.r = 1.0  # 빨간색
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        points = []
        for wp in self.latest_waypoints:
            # GPS 좌표를 UTM으로 변환하여 로컬 좌표계에 맞춤
            x, y = self.gps_to_local_utm(wp["lat"], wp["lon"])
            point = Point(x=x, y=y, z=0)
            points.append(point)
        
        line_marker.points = points
        self.waypoints_marker_pub.publish(line_marker)
        
        # 각 waypoint를 큐브로 표시
        for i, wp in enumerate(self.latest_waypoints):
            x, y = self.gps_to_local_utm(wp["lat"], wp["lon"])
            
            cube = Marker()
            cube.header.frame_id = "odom"  # odom 프레임 사용
            cube.header.stamp = rospy.Time.now()
            cube.ns = "global_waypoints_cubes"
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = 0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 4.0  # UTM 좌표계에 맞게 크기 조정
            cube.scale.y = 4.0
            cube.scale.z = 1.0
            cube.color.r = 1.0
            cube.color.g = 1.0
            cube.color.b = 0.0  # 노란색
            cube.color.a = 1.0
            self.waypoints_marker_pub.publish(cube)

if __name__ == '__main__':
    try:
        visualizer = SimplifiedPathVisualizer()
        rospy.loginfo("🎉 경로 시각화 시스템 실행 중...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")