#!/usr/bin/env python2

from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
import threading
from tf.transformations import quaternion_from_euler

def thread_job():
    rospy.spin()


class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.pos = PositionTarget()
        self.pos.type_mask = int('101111111000', 2)
        self.pos.coordinate_frame= 1

        self.pos_setpoint_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size = 1)
        # rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self.thrust_commands_cb)
        rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.thrust_commands_cb)
       
        self.pos_thread = threading.Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()

    def thrust_commands_cb(self,msg):
        rospy.loginfo("thrust: %f", msg.thrust)

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
    
    def pos_ctrl(self, position):
        self.pos.position.x = position[0]
        self.pos.position.y = position[1]
        self.pos.position.z = position[2]
        self.pos.yaw = position[3]

    def test_posctl(self):
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        self.log_topic_vars()
        
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        rospy.loginfo("run mission")
        position = [0, 0, 2, math.pi/2]
        self.pos_ctrl(position)
        rospy.spin()

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
