#!/usr/bin/env python

import rospy
from tiago_iaslab_simulation.srv import Objs, ObjsRequest
from assignment_1.msg import FindAprilTagsAction, FindAprilTagsGoal, FindAprilTagsFeedback, target_ids
from assignment_1.msg import detected_id
import actionlib
from tf.transformations import euler_from_quaternion

"""
Function to get the list of IDs to find, provided by the service /apriltag_ids_srv
"""

def get_target_ids():
    rospy.wait_for_service('/apriltag_ids_srv')

    try:
        apriltag_ids_service = rospy.ServiceProxy('/apriltag_ids_srv', Objs)
        request = ObjsRequest()
        request.ready = True
        response = apriltag_ids_service(request)
        return response.ids

    except rospy.ServiceException as e:
        rospy.logerr(f"Error while calling the service: {e}")


"""
Feedback callback function of the action server FindAprilTagsAction the is called every time a new apriltag is found
"""

def feedback_callback(feedback):
    rospy.loginfo("###### NEW FEEDBACK ###### \n")
    # Extract id, position and oriantation from the feedback message
    detected_id = feedback.detected_id
    detected_position = feedback.detected_pose.pose.position
    detected_orientation = feedback.detected_pose.pose.orientation

    # Convert from quaternion to Euler angles
    quaternion = (detected_orientation.x, detected_orientation.y, detected_orientation.z, detected_orientation.w)
    _, _, yaw = euler_from_quaternion(quaternion)

    # Print the feedback message
    rospy.loginfo("New AprilTag detected! \n")
    rospy.loginfo(f"AprilTag ID: {detected_id} \n")
    rospy.loginfo(f"Detected position: x = {detected_position.x:.2f}, y = {detected_position.y:.2f}, orientation(yaw angle) = {yaw:.2f} rad")
    rospy.loginfo("###### FEEDBACK ENDS ###### \n ")


"""
Done callback function that is invoked when the FindAprilTagsAction is completed. It return the final position of each target ids.
"""

def done_callback(state, result):
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot has completed the search! \n")

        for i, pose in enumerate(result.found_poses):
            position = pose.pose.position
            orientation = pose.pose.orientation

            # Convert from quaternion to Euler angles
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            _, _, yaw = euler_from_quaternion(quaternion)

            rospy.loginfo(f"AprilTag ID {result.found_ids[i]} position: x = {position.x:.2f}, y = {position.y:.2f}, orientation(yaw angle) = {yaw:.2f} rad")

    elif state == actionlib.GoalStatus.PREEMPTED:
        rospy.logerr(f"\n The action was preempted by the client!")

    else:
        rospy.logerr(f"\n The action was aborted by the server! Time limit probably exceeded!")




if __name__ == '__main__':
    try:
        rospy.init_node('node_a_client', anonymous = True)
        
        # Obtain the IDs to find
        target = get_target_ids()
        rospy.loginfo(f"Target IDs received from ids_generator_node: {target}")

        # create an action client
        client = actionlib.SimpleActionClient('find_apriltags', FindAprilTagsAction)
        rospy.loginfo("Waiting for action server ...")
        client.wait_for_server()
        rospy.loginfo("Action server started!")

        goal = FindAprilTagsGoal()
        goal.target_ids = target_ids()  
        goal.target_ids.ids = list(target)

        client.send_goal(goal, feedback_cb = feedback_callback, done_cb = done_callback)

        client.wait_for_result()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node_a_client stop the execution!")

