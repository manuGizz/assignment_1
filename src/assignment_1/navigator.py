#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment_1.msg import detected_id, target_ids


"""
Class used to handle the navigation among the waipoints in order to cover all the map 
"""

class WaypointNavigator:
    def __init__(self):
        # Initialize the ROS node navigator_node used to handle the navigation
        #rospy.init_node('navigator_node', anonymous = True)

        # class variables
        #self.target_ids = target_ids
        #self.detected_ids = []

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
            {"x": 7.5, "y": -1.0, "yaw": 0.0},
            {"x": 8.5, "y": -3.5, "yaw": 3.14},
            {"x": 12.5, "y": -3.5, "yaw": -0.8},
            {"x": 12.5, "y": 0.5, "yaw": 1.57},
            {"x": 10, "y": 0.6, "yaw": 3.14}
        ]
        
        self.all_ids_found = False
        #self.find_april_tags = find_april_tags


    def navigate_to_waypoint(self, x, y, yaw):
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

        rospy.loginfo(f"Navigating to waypoint: x={x}, y={y}, yaw={yaw}")
        self.client.send_goal(goal)
        #self.client.wait_for_result()
        
        rate = rospy.Rate(1)  
        try:
            while not rospy.is_shutdown():
                # Check if while moveing all the target ids are already been found
                if self.all_ids_found:
                    self.client.cancel_all_goals()
                    break

                rate.sleep() 
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS shutdown detected. Exiting gracefully.")
        

    def run(self):
        for waypoint in self.waypoints:
            # Check wheather all target ids are been found
            #if set(self.target_ids) == set(self.detected_ids):
                #rospy.loginfo("All target IDs detected. Stopping navigation.")
                #break

            # Naviga al waypoint corrente
            self.navigate_to_waypoint(waypoint["x"], waypoint["y"], waypoint["yaw"])

        # Stop of robot
        #self.client.cancel_all_goals()

    def stop_navigation(self, flag):
        if flag:
            self.all_ids_found = True
        else:
            pass

    
    """
    Callback function that is called when a message in pubblished on the the topic /target_apriltag_ids. It store the relative value of ids to find and start 
    the navigation of the robot.
    
    def target_cb(self, msg):
        self.target_ids = msg.ids
        rospy.loginfo("Target IDs arrived! Starting the reserch...")
            # starting the navigation
            #WE CAN ALSO MOVE THE PART OF TILTING THE HEAD HERE INSTED OF IN THE SERVER FILE, BEFORE THE CALL TO THE RUN() FUNCTION
        self.run()


    
    Callback function that is called every time a new message on the topic /detected_apriltag_ids is pubblished. It store in a list the id of the detected (target) id
    
    def detected_cb(self, msg):
        tag_id = msg.detected_id
        self.target_ids.append(tag_id)
    """
