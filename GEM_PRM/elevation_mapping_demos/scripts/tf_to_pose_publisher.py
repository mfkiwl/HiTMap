#!/usr/bin/env python2

import rospy
import geometry_msgs.msg
import tf
import nav_msgs.msg
import numpy as np

def callback(newPose):
    """Listens to a transform between from_frame and to_frame and publishes it
       as a pose with a zero covariance."""
    global publisher, newKeyframePublisher, tf_listener, from_frame, to_frame, prev_x, prev_y, distance

    # Listen to transform and throw exception if the transform is not
    # available.
    try:
        (trans, rot) = tf_listener.lookupTransform(
            from_frame, to_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        return

    distance = np.sqrt((trans[0] - prev_x) ** 2 + (trans[1] - prev_y) ** 2)

    if distance >= keyframe_distance:
        prev_x = trans[0]
        prev_y = trans[1]
        odom = nav_msgs.msg.Odometry()
        odom.twist.twist.linear.x = trans[0]
        odom.twist.twist.linear.y = trans[1]
        newKeyframePublisher.publish(odom)

    # Create and fill pose message for publishing
    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = from_frame
    pose.pose.pose.position.x = trans[0]
    pose.pose.pose.position.y = trans[1]
    pose.pose.pose.position.z = trans[2]
    pose.pose.pose.orientation.x = rot[0]
    pose.pose.pose.orientation.y = rot[1]
    pose.pose.pose.orientation.z = rot[2]
    pose.pose.pose.orientation.w = rot[3]

    # Since tf transforms do not have a covariance, pose is filled with
    # a zero covariance.
    pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0]

    publisher.publish(pose)


def main_program():
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    global publisher, newKeyframePublisher, tf_listener, from_frame, to_frame, prev_x, prev_y, distance, keyframe_distance

    
    rospy.init_node('tf_to_pose_publisher')

    prev_x = 0
    prev_y = 0

    # Read frame id's for tf listener
    keyframe_distance = rospy.get_param("~keyframe_distance")
    from_frame = rospy.get_param("~from_frame")
    to_frame = rospy.get_param("~to_frame")
    pose_name = str(to_frame) + "_pose"

    tf_listener = tf.TransformListener()
    publisher = rospy.Publisher(pose_name, geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)

    newKeyframePublisher = rospy.Publisher("/elevation_mapping/new_keyframe", nav_msgs.msg.Odometry, queue_size=1)
    
    # Set callback and start spinning
    rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass
