#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry

class InputCovarianceTester:

    def __init__(self):
        self._period = rospy.get_param('~period', 0.05)
        self._scale = rospy.get_param('~scale', 1e4)

        self._num_msgs = 0

        self._pub = rospy.Publisher('odom', Odometry, queue_size=1)

        self._timer = rospy.Timer(rospy.Duration(self._period), self._publish_odometry)

    def _publish_odometry(self, event):
        odom = Odometry()

        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.orientation.w = 1.0

        odom.pose.covariance[ 0] = self._scale * self._num_msgs
        odom.pose.covariance[ 7] = 1.0
        odom.pose.covariance[14] = 1.0
        odom.pose.covariance[21] = 1.0
        odom.pose.covariance[28] = 1.0
        odom.pose.covariance[35] = 1.0

        self._pub.publish(odom)

        self._num_msgs += 1


def main():
    rospy.init_node('test_input_covariance')

    try:
        InputCovarianceTester()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Ctrl-c received')


if __name__ == "__main__":
    main()
