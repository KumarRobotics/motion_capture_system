#!/usr/bin/env python

"""
Convienience node for evaluating localization or odometry
compared to a motion capture system.
Adds a service that syncs the position of the car with
the position of an object in the mocap system at that time point.
"""

from threading import Thread
import collections
import heapq
import bisect

import rospy
import rostopic
import roslib.message
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_multiply, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped, TwistStamped
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import std_msgs.msg
from qualisys.msg import Subject

#from synchronize_utils import extract_pose, pose_diff

__license__ = "MIT"
__maintainer__ = "Tobias Bolin"
__email__ = "tbolin@kth.se"
__status__ = "Development"



def extract_pose(msg):
    """ Extract the pose from a message
    :param msg: An Odometry, Subject or any variation of Pose message
    :return: The Pose contained in msg
    :rtype: :class:`geometry_msgs.msg.Pose`
    """
    if isinstance(msg, (Odometry, PoseWithCovarianceStamped)):
        return msg.pose.pose
    elif isinstance(msg, (PoseStamped, PoseWithCovariance)):
        return msg.pose
    elif isinstance(msg, Pose):
        return msg
    elif isinstance(msg, Subject):
        pose = Pose()
        pose.position = msg.position
        pose.orientation = msg.orientation
        return pose
    else:
        raise TypeError("Could not extract pose from message of type {}".format(type(msg)))


def iter_quat(quaternion):
    """Iterate over a quaternion in x, y, z, w order.
    
    :param quaternion: A quaternion i.e. any data type that have an x, y, z, and w field"""
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    # yield quaternion.x
    # yield quaternion.y
    # yield quaternion.z
    # yield quaternion.w


def pose_diff(msg1, msg2):
    """Calculate the difference between two messages containing a pose
    i.e. msg1 - msg2, and return the result as a transform.
    :param msg1: First message. An Odometry, Subject or any variation of Pose message 
    :param msg2: Second message. An Odometry, Subject or any variation of Pose message
    
    :return: Difference between the two essages as a geometry_msgs.msg.Pose
    :rtype:  :class:`geometry_msgs.msg.Transform`
    """
    pose1 = extract_pose(msg1)
    pose2 = extract_pose(msg2)
    pos1 = pose1.position
    pos2 = pose2.position
    ori1 = pose1.orientation
    
    ori2 = [pose2.orientation.x,
            pose2.orientation.y,
            pose2.orientation.z,
            -pose2.orientation.w]
    ret_tf = Transform()

    # Orientation differences
    rot = quaternion_multiply(iter_quat(ori1), ori2)
    ret_tf.rotation.x = rot[0]
    ret_tf.rotation.y = rot[1]
    ret_tf.rotation.z = rot[2]
    ret_tf.rotation.w = rot[3]
    # Position differences
    rot_inv = [rot[0], rot[1], rot[2], -rot[3]]
    trans_vec = [pos2.x, pos2.y, pos2.z, 0.0]
    trans_vec = quaternion_multiply(quaternion_multiply(rot, trans_vec), rot_inv)
    ret_tf.translation.x = pos1.x + trans_vec[0]
    ret_tf.translation.y = pos1.y + trans_vec[1]
    ret_tf.translation.z = pos1.z + trans_vec[2]
    return ret_tf


class PoseComparator(object):
    """A class for comparing a position (from the `child_topic`) to ground truth 
    (from the `parent_topic`).Both topics should be publishing either 
    class: `geometry_msgs.msg.PoseStamped`, 
    class: `geometry_msgs.msg.PoseWithCovarianceStamped`, or 
    class: `nav_msgs.msg.Odometry` messages.

    :param parent_topic: Name of the topic publishing the ground truth. 
    :type parent_topic: str
    :param parent_topic: Name of the topic publishing the ground truth. 
    :type parent_topic: str
    :param publish_transformed_child: True if the pose published on the child topic
    should be transformed to the frame of the parent topic and published.
    :type publish_transformed_child: bool
    :param publish_difference: True if the diffrence between the child topic
    and the parent topic should be published. 
    :type publish_difference: bool
    """
    def __init__(self, 
            parent_topic, 
            child_topic, 
            publish_trasformed_child=True, 
            publish_difference=True,
            parent_buffer_size=100,
            sync_at_init=False):
        """Constructor method
        """
        super(PoseComparator, self).__init__()
        self.parent_topic = parent_topic
        self.child_topic = child_topic
        self.publish_trasformed_child = publish_trasformed_child
        self.publish_difference = publish_difference
        self.parent_buffer_size = parent_buffer_size

        parent_data_type = rostopic.get_topic_type(parent_topic, blocking=True)[0]
        child_data_type = rostopic.get_topic_type(child_topic, blocking=True)[0]
        self.parent_msg_class = roslib.message.get_message_class(parent_data_type)
        self.child_msg_class = roslib.message.get_message_class(child_data_type)
        self.parent_msg_buffer = collections.deque(maxlen=self.parent_buffer_size)
        self.child_msg_buffer = collections.deque()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_brodcaster = tf2_ros.StaticTransformBroadcaster()
        self.sync_sub = rospy.Subscriber(
                '/comp/sync', 
                std_msgs.msg.Bool, 
                self.sync_callback
                )
        if self.publish_difference:
            self.diff_pub = rospy.Publisher('/comp/difference', 
                        TwistStamped, 
                        queue_size=1)
        if self.publish_trasformed_child:
            self.trans_pub = rospy.Publisher(
                    '/comp/tranformed', 
                    PoseStamped, 
                    queue_size=1)
        self.transform_received = False
        self.parent_frame = None
        self.child_frame = None
        self._seq_count = 0

        if sync_at_init:
            self.sync_callback(True)


    def start_publish(self):
        """Start publishing comparision and the transformed 
        child topic when a transform has been published.
        """
        if not self.transform_received:
            self.transform_received = True
            self.child_sub = rospy.Subscriber(
                        self.child_topic, 
                        self.child_msg_class
                        )
            if self.publish_difference:
                self.parent_sub = rospy.Subscriber(
                        self.parent_topic,
                        self.parent_msg_class,
                        self.parent_callback
                        )
                self.child_sub.impl.add_callback(self.child_difference_callback, None)

            if self.publish_trasformed_child:
                self.child_sub.impl.add_callback(self.child_transform_callback, None)


    def interpolate_pose(self, interp_time, pose1, pose2, t1=None, t2=None):
        """Calculate an interpolated pose between pose 2  and pose 1.

        :param interp_time: The time for the interpolated pose
        :type interp_time: class:`rospy.Time`
        :param pose1: The most recent pose. Can be most ros messages containing a pose.
        If the message is not stamped the time can either be provided as a the part of 
        a tuple or sepparately in t1.
        :param pose1: The less recent pose. Can be most ros messages containing a pose.
        If the message is not stamped the time can either be provided as a the part of 
        a tuple or sepparately in t2.
        :param t1: The time stamp for pose1
        :type t1: class:`rospy.Time`, optional
        :param t2: The time stamp for pose2
        :type t2: class:`rospy.Time`, optional
        :return: An interpolated pose between pose 1 and pose 2 at time interp_time.
        :rtype: class:`geometry_msgs.msg.PoseStamped`
        """
        interp = PoseStamped()
        interp.header.frame_id = self.parent_frame
        interp.header.stamp = interp_time

        if t1 is None:
            try:
                t1 = pose1[0]
                pose1 = extract_pose(pose1[1])
            except TypeError: 
                t1 = pose1.header.stamp
                pose1 = extract_pose(pose1)
        if t2 is None:
            try:
                t2 = pose2[0]
                pose2 = extract_pose(pose2[1])
            except TypeError: 
                t2 = pose2.header.stamp
                pose2 = extract_pose(pose2)

        dt = (t1 - t2).to_sec()
        dx = (pose1.position.x - pose2.position.x) * dt
        dy = (pose1.position.y - pose2.position.y) * dt
        dz = (pose1.position.z - pose2.position.z) * dt

        interp_dt = (interp_time - t2).to_sec()
        interp.pose.position.x = pose2.position.x + dx*interp_dt
        interp.pose.position.y = pose2.position.y + dy*interp_dt
        interp.pose.position.z = pose2.position.z + dz*interp_dt

        interp.pose.orientation = pose2.orientation
        return interp


    def parent_callback(self, msg):
        """Store the message from the parent frame topic in a buffer,
        and publish the difference between the parent topic and all  
        messages published on the child frame topic up to the timestamp
        of this message.

        :param msg: Any stamped ROS message containing a pose.
        """
        
        if self.parent_frame is None:
            self.parent_frame = msg.header.frame_id
        self.parent_msg_buffer.append((msg.header.stamp, extract_pose(msg)))
        
        # rospy.loginfo_throttle(1, self.parent_msg_buffer)
        while self.child_msg_buffer and self.child_msg_buffer[0][0] < msg.header.stamp:
            child_tuple = self.child_msg_buffer.popleft()
            child_time, child_msg = child_tuple 
            prev_pose_ix = bisect.bisect_left(self.parent_msg_buffer, child_tuple) - 1
            if prev_pose_ix >= len(self.parent_msg_buffer): 
                # Skip if the child message is to old, could indicate a clock desync
                rospy.logwarn("message from time {} skipped at time {}"\
                               "\ncurrent difference: {}"\
                               "\nlast difference: {}"
                               "\nix: {}"
                               "\nBuffer len: {}".format(
                        child_time.to_sec(), 
                        rospy.Time.now().to_sec(),
                        (msg.header.stamp - child_time).to_sec(),
                        (self.parent_msg_buffer[prev_pose_ix][0] - child_time).to_sec(),
                        prev_pose_ix,
                        len(self.parent_msg_buffer)
                    ))
                continue  
            else:
                child_msg = self.tf_buffer.transform(child_msg, self.parent_frame)
                preceeding_pose = self.parent_msg_buffer[prev_pose_ix]
                following_pose = self.parent_msg_buffer[prev_pose_ix + 1]
                interp = self.interpolate_pose(child_time, preceeding_pose, following_pose)
                diff_tf = pose_diff(interp, child_msg)

                diff_msg = TwistStamped()
                diff_msg.header.seq = self.seq_count
                diff_msg.header.stamp = child_time
                diff_msg.header.frame_id = self.parent_frame

                diff_msg.twist.linear.x = diff_tf.translation.x
                diff_msg.twist.linear.y = diff_tf.translation.y
                diff_msg.twist.linear.z = diff_tf.translation.z
                rpy = euler_from_quaternion(iter_quat(diff_tf.rotation))
                diff_msg.twist.angular.x = rpy[0]
                diff_msg.twist.angular.y = rpy[1]
                diff_msg.twist.angular.z = rpy[2]
                self.diff_pub.publish(diff_msg)


    def child_difference_callback(self, msg):
        """Store the message in a buffer so that it can later 
        be compared to the positions from the parent topic.
        
        :param msg: Any stamped ROS message containing a pose.
        """
        if self.child_frame is None:
            self.child_frame = msg.header.frame_id
        msg_pose = tf2_geometry_msgs.PoseStamped()
        msg_pose.header = msg.header
        msg_pose.pose = extract_pose(msg)
        self.child_msg_buffer.append((msg.header.stamp, msg_pose))
        # self.child_msg_buffer.append((msg.header.stamp, extract_pose(msg)))


    def child_transform_callback(self, msg):
        """Publish the topic used to define the child frame transformed into the parent frame.
        
        The message will be published as a class:`geometry_msgs.msg.PoseStamped`.
        :param msg: An Odometry, Subject or any variation of PoseStamped message containing the 
                    Pose of the tracked object in the chid frame.
        """
        if self.child_frame is None:
            self.child_frame = msg.header.frame_id

        if self.publish_trasformed_child and not self.parent_frame is None:
            msg_pose = tf2_geometry_msgs.PoseStamped()
            msg_pose.header = msg.header
            msg_pose.pose = extract_pose(msg)
            
            try:
                msg_trans = self.tf_buffer.transform(msg_pose, self.parent_frame)
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException), e:
                # No transform available yet, do not publish estimated pose in the mocap frame
                rospy.logerr_throttle(5, e.__repr__())
                return
            self.trans_pub.publish(msg_trans)


    def sync_callback(self, msg):
        """Create a transform between the parent frame and the 
        child frame based on the current difference between the 
        parrent topic and the child topic. i.e. The difference between
        the poses should be zero at the time that this function is called 
        in the transformed frame of reference 
        """
        parent_msg = rospy.wait_for_message(
                self.parent_topic, 
                self.parent_msg_class, 
                timeout=2.0)
        child_msg = rospy.wait_for_message(
                self.child_topic, 
                self.child_msg_class, 
                timeout=2.0)
        if self.parent_frame is None:
            self.parent_frame = parent_msg.header.frame_id
        if self.child_frame is None:
            self.child_frame = child_msg.header.frame_id

        if parent_msg is None or child_msg is None:
            if parent_msg is None and child_msg is None:
                rospy.logwarn("Positions from {} and {} not available, no transform created").format(
                    self.parent_topic,
                    self.child_topic
                )
            elif parent_msg is None:
                rospy.logwarn("Position from {} not available, no transform created").format(
                    self.parent_topic
                )
            else:
                rospy.logwarn("Position from {} not available, no transform created").format(
                    self.child_topic
                )
            return

        t = TransformStamped()
        t.header.stamp = child_msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform = pose_diff(parent_msg, child_msg)
        self.tf_brodcaster.sendTransform(t)
        self.start_publish()


    @property
    def seq_count(self):
        self._seq_count += 1
        return self._seq_count - 1


if __name__ == '__main__':
    rospy.init_node('sync_mocap')
    parent_topic = rospy.get_param('~parent_topic') 
    child_topic = rospy.get_param('~child_topic')
    buffer_size = int(rospy.get_param('~buffer_size'))
    comparator = PoseComparator(
            parent_topic, 
            child_topic, 
            parent_buffer_size=buffer_size
            )
    rospy.spin()
                     
