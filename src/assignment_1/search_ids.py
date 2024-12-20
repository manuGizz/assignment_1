#!/usr/bin/env python

import rospy
import actionlib
import tf
from assignment_1.msg import FindAprilTagsAction, FindAprilTagsFeedback, FindAprilTagsResult, research_status
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped 

"""
This class is used to handle the recognition of the apriltags and the detection of the their poses with respect to the map frame. When the target IDs are received
from the action client, the navigator node is informed that it can begin navigation. When the research is finished, the navigator node is informed that it can stop
the navigation.
"""

class FindAprilTags:
    def __init__(self, name):
        self.server_name = name
        self.server = actionlib.SimpleActionServer(
            self.server_name,
            FindAprilTagsAction,
            execute_cb = self.execute_cb,
            auto_start = False
        )
        # Start the action server 
        self.server.start()

        # Class variables
        self.target_ids = [] # list of target ids to found
        self.detected_ids = [] # list of ids already found by Thiago while moving
        self.detected_poses = [] # list of detected poses

        # Publisher used to comunicate to the navigator node about the reserch status
        self.pub = rospy.Publisher('/research_status', research_status, queue_size = 10)
        self.status_msg = research_status()

        # Start the subscriber to tag_detection to detect the AprilTags during the robot's motion
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)

        # Initialize the TransformListener for the conversion from camera frame to 'map' frame
        self.listener = tf.TransformListener()
        self.target_frame = "map"
        self.source_frame = ""

    
    """
    Callback function that is invoked when the action server FindAprilTags receives a goal (target IDs) from the action client.
    """

    def execute_cb(self, goal):
        # Save the received target IDs 
        self.target_ids = goal.target_ids.ids

        # Inform the navigator node that it can start the navigation
        self.status_msg.status = True
        self.pub.publish(self.status_msg)

        rospy.loginfo("Search for Apriltags started!")
  
        # Set a flag used to check during the execution if all the IDs are been found
        flag = False

        try:
            timeout_duration = rospy.Duration(150)  # Timeout of 2 minutes and half to complete the searching
            start_time = rospy.Time.now()
            rate = rospy.Rate(1)

            while not rospy.is_shutdown():
                # Check if the client request to cancell the action
                if self.server.is_preempt_requested():
                    rospy.loginfo("Action preempted by the client.")
                    self.server.set_preempted()
                    self.status_msg.status = False
                    self.pub.publish(self.status_msg)
                    break

                # Check when all the IDs are been found
                if len(self.detected_ids) == len(self.target_ids):
                    rospy.loginfo("All AprilTags are been found! Stopping the navigation...")
                    self.status_msg.status = False
                    self.pub.publish(self.status_msg)
                    flag = True # Set flag to True to indicate that the action has been completed
                    break
                
                # Timeout check (if in 150 seconds the searching is not done the action is considered as failed)
                if rospy.Time.now() - start_time > timeout_duration:
                    self.server.set_aborted()
                    self.status_msg.status = False
                    self.pub.publish(self.status_msg)
                    break

                rate.sleep() 
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS shutdown detected! Exiting...")


        # If all IDs found we comunicate the obtained result to the action client, otherwise the action failed
        if flag:
            result = FindAprilTagsResult()

            # Set the result of the ation to seccess and compose the result message
            result.success = True 
            result.found_ids = self.detected_ids
            result.found_poses = self.detected_poses

            # Send the result to the action client
            self.server.set_succeeded(result)

            rospy.signal_shutdown("Action completed successfully! Stopping the server node...")
        else:
            rospy.signal_shutdown("Action failed! Stopping the server node...")

    """
    Callback function of AprilTags detection. This function is called every time that a message will be published on topic /tag_detections. 
    It perform the conversion from camera frame to base frame. Every time a new AprilTag is detected by the camera, this function checks whether 
    the ID is present in the list of target IDs and only if so, saves the relevant data and informs the action client by sending feedback. It handle
    even the transformation from camera frame to map frame.
    """

    def detection_callback(self, msg):
        # Obtain the source frame
        self.source_frame = msg.header.frame_id

        # Wait until transform is available
        while not self.listener.canTransform(self.target_frame, self.source_frame, rospy.Time(0)):
            rospy.sleep(0.5)

        for detection in msg.detections:
            tag_id = detection.id[0]
            # Check if the ID is the the target list and if it has not already been found before
            if tag_id in self.target_ids and tag_id not in self.detected_ids:

                pos_in = PoseStamped()
                pos_out = PoseStamped()

                # Obtain the pose wrt camera frame
                pos_in.header.frame_id = detection.pose.header.frame_id
                pos_in.pose.position = detection.pose.pose.pose.position
                pos_in.pose.orientation = detection.pose.pose.pose.orientation
                
                # Transform the pose in the map frame
                try:
                    pos_out = self.listener.transformPose(self.target_frame, pos_in)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(f"Error during transform: {e}")

                # Publish a feedback 
                feedback = FindAprilTagsFeedback()
                feedback.detected_id = tag_id
                feedback.detected_pose = pos_out
                self.server.publish_feedback(feedback)

                # save the id and the pose in the relative class variables
                self.detected_ids.append(tag_id)
                self.detected_poses.append(pos_out)