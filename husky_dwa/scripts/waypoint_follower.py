#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseStamped

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')

        # Waypoint 저장용 리스트
        self.waypoints = []
        self.received = False

        # /waypoint_markers 구독
        rospy.Subscriber('/waypoint_markers', MarkerArray, self.waypoint_callback)

        # MoveBaseAction 클라이언트
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        rospy.loginfo("Waiting for waypoint_markers...")
        rospy.wait_for_message('/waypoint_markers', MarkerArray)
        rospy.loginfo("Got waypoint_markers, starting...")

        self.send_goals()

    def waypoint_callback(self, msg):
        if not self.received:
            for marker in msg.markers:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position = marker.pose.position
                goal.target_pose.pose.orientation = marker.pose.orientation
                self.waypoints.append(goal)
            self.received = True

    def send_goals(self):
        rate = rospy.Rate(1)
        for i, goal in enumerate(self.waypoints):
            rospy.loginfo(f"Sending waypoint {i+1}/{len(self.waypoints)}")
            self.client.send_goal(goal)
            self.client.wait_for_result()
            rospy.loginfo(f"Reached waypoint {i+1}")
            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointFollower()
    except rospy.ROSInterruptException:
        pass
