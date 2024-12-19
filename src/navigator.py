#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment_1.msg import navigation_status


"""
Class used to handle the navigation among the waipoints in order to cover all the map 
"""

class WaypointNavigator:
    def __init__(self):

        # class variables
        self.target_ids = []
        self.detected_ids = []

        # Subscribe to topic /found_apriltag_id to obtain the (target) id already found during the motion
        #rospy.Subscriber('/found_apriltag_id', detected_id, self.detected_cb)

        # Subscribe to topic /target_apriltag_ids to obtain the ids target ids
        #rospy.Subscriber('/target_apriltag_ids', target_ids, self.target_cb)

        # Initialize the MoveBaseAction client 
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Waypoints definition
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "yaw": -1.57},
            {"x": 7.5, "y": -1.0, "yaw": 3.14},
            {"x": 8.5, "y": -3.5, "yaw": 3.14},
            {"x": 12.5, "y": -3, "yaw": -0.8},
            {"x": 12.5, "y": 0.5, "yaw": 1.57},
            {"x": 10, "y": 0.6, "yaw": 3.14}
        ]
        
        self.all_ids_found = False
        self.current_waypoint_index = 0

        rospy.Subscriber('/navigation_status', navigation_status, self.navigation_status_cb)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)



    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("All waypoints reached...not all the target IDs are been found!")
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        x = waypoint["x"]
        y = waypoint["y"]
        yaw = waypoint["yaw"]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        #rospy.loginfo(f"Navigating to waypoint: x={x}, y={y}, yaw={yaw}")
        self.client.send_goal(goal, done_cb = self.goal_done_callback)   


    def goal_done_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached!")

        self.current_waypoint_index += 1

        self.send_next_waypoint()

    """
    def execute_cb(self):
        rate = rospy.rate(1)

        while not rospy.is_shutdown():
            if self.all_ids_found:
                self.client.cancel_goal() # stop the robot
                break  
            rate.sleep()
    """

    def navigation_status_cb(self, msg):
        if msg.status:
            # Solo invia il prossimo waypoint se il robot non è già in movimento
            if self.client.get_state() != actionlib.GoalStatus.ACTIVE:
                self.send_next_waypoint()
        else:
            # Se il status è False, cancella il goal attuale per fermare il robot
            rospy.loginfo("Navigation stopped, cancelling current goal...")
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.client.cancel_all_goals()

    """
    def stop_navigation(self, flag):
        if flag:
            self.all_ids_found = True
        else:
            pass

    """


if __name__ == '__main__':
    try:
        # Initialize the ROS node navigator_node used to handle the navigation
        rospy.init_node('navigator_node', anonymous = True)
        WaypointNavigator()

        rospy.loginfo("Navigator node ready!")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error! Navigator node stopped!")