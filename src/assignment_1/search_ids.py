#!/usr/bin/env python

import rospy
import actionlib
import tf
from assignment_1.msg import FindAprilTagsAction, FindAprilTagsFeedback, FindAprilTagsResult, navigation_status
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped 

"""
Constructor of the class FindAprilTags. This class is used to handle the recognition of the apriltags and the detection of the their poses with respect 
to the map frame.
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
        self.server.start()
        self.result = FindAprilTagsResult()

        # Publisher used to comunicate to the navigator node about the reserch status
        self.pub = rospy.Publisher('/navigation_status', navigation_status, queue_size = 10)
        self.status_msg = navigation_status()

        # Start the subscriber to tag_detection
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback_tf)

        self.target_ids = [] # list of target ids to found
        self.detected_ids = [] # list of ids already found by Thiago while moving
        self.detected_poses = [] # list of detected poses

        self.listener = tf.TransformListener()


    """
    Callback function of AprilTags detection. Each time a new AprilTag is detected, a message will be published on topic
    /tag_detections and this function will be called. It perform the conversion from camera frame to base frame.
    """

    def detection_callback_tf(self, msg):
        # Initialize the TransformListener
        target_frame = "map"
        source_frame = msg.header.frame_id

        # Wait until transform is available
        while not self.listener.canTransform(target_frame, source_frame, rospy.Time(0)):
            rospy.sleep(0.5)

        # Perform the transformation
        for detection in msg.detections:
            tag_id = detection.id[0]
            # Perform the transformation only if the AprilTag is the the list of target_ids
            if tag_id in self.target_ids and tag_id not in self.detected_ids:

                pos_in = PoseStamped()
                pos_out = PoseStamped()

                # Obtain the pose wrt camera frame
                pos_in.header.frame_id = detection.pose.header.frame_id
                pos_in.pose.position = detection.pose.pose.pose.position
                pos_in.pose.orientation = detection.pose.pose.pose.orientation
                
                # Transform the pose in the base_frame
                try:
                    pos_out = self.listener.transformPose(target_frame, pos_in)
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

                # Create and pubblish the message with the new detected ids to inform the navigator node on the already found ids
                #detected_msg = detected_id()
                #detected_msg.detected_id = tag_id
                #self.pub.publish(detected_msg)
                

    """
    Callback function that is invoked when the action server FindAprilTags receives a goal from the action client (ids to found)
    """

    def execute_cb(self, goal):
        self.target_ids = goal.target_ids.ids

        self.status_msg.status = True
        self.pub.publish(self.status_msg)

        rospy.loginfo("Search for Apriltags started!")

      # Execute the action
        rate = rospy.Rate(1)  
        flag = True

        try:
            while not rospy.is_shutdown():
                # Check if the client request to cancell the action
                if self.server.is_preempt_requested():
                    rospy.loginfo("Action preempted by the client.")
                    self.server.set_preempted()
                    flag = False
                    break

                # Check when all the ids are been found
                if len(self.detected_ids) == len(self.target_ids):
                    rospy.loginfo("All AprilTags are been found! Stopping the navigation...")
                    self.status_msg.status = False
                    self.pub.publish(self.status_msg)
                    break

                rate.sleep() 
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS shutdown detected. Exiting gracefully.")
            flag = False


        # Comunicate the result to the action client
        if flag:
            if len(self.detected_ids) == len(self.target_ids):
                self.result.success = True # set success (of the action result) to true if all ids found

            self.result.found_ids = self.detected_ids
            self.result.found_poses = self.detected_poses

            # send the result
            self.server.set_succeeded(self.result)
        else:
            rospy.loginfo("Action failed!")

    """
    def getDetectedIds(self):
        return list(self.detected_ids)

    def getTargetIds(self):
        return list(self.target_ids)
    """


