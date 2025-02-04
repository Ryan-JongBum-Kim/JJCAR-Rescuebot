import tf2_ros
import tf2_geometry_msgs
import rospy

import geometry_msgs.msg

def pose_publisher():
    pub = rospy.Publisher(name='rescue_bot_pose', data_class=geometry_msgs.msg.TransformStamped, queue_size=10)

    target_frame = 'map'
    source_frame = 'base_footprint'

    rospy.init_node('pose_publisher_node', anonymous=True)

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # Wait for the transformation from source_frame to target_frame
    transform : geometry_msgs.msg.TransformStamped = buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))

    pub.publish(transform)
    rospy.loginfo(rospy.get_caller_id() + " is publishing")

if __name__ == '__main__':
    while not rospy.is_shutdown():
        pose_publisher()

        rospy.sleep(0.01)