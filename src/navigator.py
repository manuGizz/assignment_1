#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment_1.msg import research_status


"""
Class used to handle the navigation among the waipoints in order to cover all the map. 
"""

class WaypointNavigator:
    def __init__(self):

        # Initialize the MoveBaseAction client 
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Waypoints definition
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "yaw": -2},
            {"x": 8.5, "y": -1, "yaw": 3.14},
            {"x": 8.5, "y": -3, "yaw": 3.14},
            {"x": 11.5, "y": -3.3, "yaw": -0.5},
            {"x": 12.5, "y": -0.5, "yaw": 3.14},
            {"x": 12.5, "y": 0.5, "yaw": 1.57},
            {"x": 10, "y": 0.6, "yaw": 3.14}
        ]

        self.current_waypoint_index = 0

        # Subscribe to topic /research_status to obtain infornation from node_b about the status of the action FindAprilTags
        rospy.Subscriber('/research_status', research_status, self.research_status_cb)

        #self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)


    """
    Method used to send the goal position to the robot robot
    """

    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.signal_shutdown("Stopping the navigator node...")
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

    """
    Callback function for the done_cb of the move_base action.
    """

    def goal_done_callback(self, status, result):
        if status == actionlib.GoalStatus.ABORTED:
            rospy.signal_shutdown("Navigation aborted by the action server! Stoping the navigator node...")
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.signal_shutdown("Stopping the navigator node...")
        else:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached!")
            self.current_waypoint_index += 1
            self.send_next_waypoint()

    """
    Callback of the subscriber /research_status. It starts the navigation when the IDs are received and stop it when all the target IDs are detected.
    """

    def research_status_cb(self, msg):
        if msg.status:
            self.send_next_waypoint()
        else:
            rospy.loginfo("All IDs found!")
            state = self.client.get_state()
            # Check to avoid errors with the last waypoint in case all the IDs haven't all been found yet
            if state == actionlib.GoalStatus.ACTIVE:
                self.client.cancel_goal()



if __name__ == '__main__':
    try:
        # Initialize the ROS node navigator_node used to handle the navigation
        rospy.init_node('navigator_node', anonymous = True)

        # Create an istance of the class WayPointNavigator
        WaypointNavigator()

        rospy.loginfo("Navigator node ready!")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error! Navigator node stopped!")