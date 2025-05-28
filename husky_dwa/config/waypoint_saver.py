#!/usr/bin/env python3
import rospy
import yaml
from geometry_msgs.msg import PointStamped

class WaypointSaver:
    def __init__(self):
        rospy.init_node('waypoint_saver')
        self.filename = rospy.get_param("~output", "/home/danny/catkin_ws/src/husky_dwa/config/waypoints.yaml")
        self.points = []
        rospy.Subscriber("/clicked_point", PointStamped, self.point_callback)
        rospy.on_shutdown(self.save_to_yaml)
        rospy.loginfo("📌 웨이포인트 저장 시작됨 (/clicked_point)")
        rospy.spin()

    def point_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.points.append({'x': x, 'y': y, 'z': 0.0})
        rospy.loginfo(f"저장: ({x:.2f}, {y:.2f}) → 총 {len(self.points)}개")

    def save_to_yaml(self):
        with open(self.filename, 'w') as f:
            yaml.dump(self.points, f)
        rospy.loginfo(f"✅ {len(self.points)}개의 웨이포인트를 {self.filename}에 저장 완료.")

if __name__ == "__main__":
    WaypointSaver()
