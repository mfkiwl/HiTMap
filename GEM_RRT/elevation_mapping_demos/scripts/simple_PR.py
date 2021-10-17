#!/usr/bin/env python2

import rospy
import geometry_msgs.msg
import nav_msgs.msg
import tf
from dislam_msgs.msg import PrResult
from sklearn.neighbors import KDTree
import numpy as np
import time

def newKeyFrameCB(data):
    global keyframe, neighbor_distance

    tree = KDTree(keyframe)
    ind_nn = tree.query_radius([[data.twist.twist.linear.x, data.twist.twist.linear.y]], r=neighbor_distance)

    time.sleep(0.2)

    for i in range(len(ind_nn[0])):

        if ind_nn[0][i].size != 0:
            relPose = geometry_msgs.msg.Pose()
            relPose.position.x = data.twist.twist.linear.x - keyframe[ind_nn[0][i]][0]
            relPose.position.y = data.twist.twist.linear.y - keyframe[ind_nn[0][i]][1]
            relPose.position.z = 0
            relPose.orientation.x = 0
            relPose.orientation.y = 0
            relPose.orientation.z = 0
            relPose.orientation.w = 1

            output = PrResult()

            output.submapID = ind_nn[0][i]
            output.relPose = relPose
            publisher.publish(output)

    keyframe = np.append(keyframe, [[data.twist.twist.linear.x, data.twist.twist.linear.y]], axis = 0)


def callback(newPose):
    """Listens to a transform between from_frame and to_frame and publishes it
       as a pose with a zero covariance."""
    global publisher, tf_listener, to_frame, from_frame, prev_x, prev_y, distance, submapID

    # Listen to transform and throw exception if the transform is not
    # available.
    try:
        (trans, rot) = tf_listener.lookupTransform(
            from_frame, to_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        return


def main_program():
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    global publisher, tf_listener, neighbor_distance, from_frame, to_frame, prev_x, prev_y, distance, submapID, keyframe
    rospy.init_node('place_recognition_publisher')
    
    keyframe = np.array([[0, 0]])
    submapID = 0
    neighbor_distance = 5
    # from_frame = rospy.get_param("~from_frame")
    # to_frame = rospy.get_param("~to_frame")
    # neighbor_distance = rospy.get_param("~neighbor_distance")

    # tf_listener = tf.TransformListener()
    publisher = rospy.Publisher("/elevation_mapping/placeRec", PrResult, queue_size=1)

    # Set callback and start spinning
    rospy.Subscriber("/elevation_mapping/new_keyframe", nav_msgs.msg.Odometry, newKeyFrameCB)
    # rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass
