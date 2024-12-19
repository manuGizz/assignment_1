#!/usr/bin/env python

import rospy
import actionlib
from assignment_1.search_ids import FindAprilTags
from assignment_1.navigator import WaypointNavigator
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == '__main__':
    try:
        # Initialize the ROS node_b_server (server node)
        rospy.init_node('node_b_server', anonymous = True)

        # Create a publisher for the /head_controller/command topic for tilting the head angle (to have a better view of the camera towards the Apriltags)
        pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size = 10)
        rospy.sleep(1)
        
        # Create the JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.joint_names = ['head_1_joint', 'head_2_joint']
        
        # Create a point for the trajectory
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.6]  # haed_2_joint allows to modify to the tilt angle
        point.time_from_start = rospy.Duration(1) # Duration of the movement
        
        # Add the point to the trajectory message and publish the message
        trajectory_msg.points.append(point)
        pub.publish(trajectory_msg)


        # Initialize the find apritags class
        find_april_tags = FindAprilTags('find_apriltags')

        # Initialize the navigator class
        #navigator = WaypointNavigator()

        rospy.loginfo("Waiting for target IDs from the action client...")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error! Node_b stopped!")

