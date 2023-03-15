#!/usr/bin/env python2
from __future__ import division

PKG = 'px4'

import rospy
import math,threading
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
# from scipy.spatial.transform import Rotation as R
# from tf.transformations import quaternion_from_euler

def angel2rad(angel):
    return angel/180*math.pi

def thread_job():
    rospy.spin()

class MavrosOffboardPosctlTest(MavrosTestCommon):

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.target = PositionTarget()
        self.cur_pos = PoseStamped()
        self.target.type_mask = int('101111000111', 2)
        self.target.coordinate_frame= 1
        self.target_pos = [0.5, 0.5, 1.5]

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.refCb)
        self.sp_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size = 1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()

        # kp
        self.Kp1x = 0.2
        self.Kp1y = 0.2
        self.Kp1z = 0.2

        # initial
        self.target.velocity.x = 0
        self.target.velocity.y = 0
        self.target.velocity.z = 0
        self.target.yaw = 0

    def refCb(self, msg):
        self.cur_pos = msg
        # quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        # euler = quaternion_from_euler(quaternion)
        # print("current_pos: " , self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z)
        # print("current_pose: ", euler)

    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.target.header = Header()
        self.target.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.target.header.stamp = rospy.Time.now()
            self.sp_pub.publish(self.target)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def vel_control(self):
        self.target.velocity.x = self.Kp1x*(self.target_pos[0] - self.cur_pos.pose.position.x)
        self.target.velocity.y = self.Kp1y*(self.target_pos[1] - self.cur_pos.pose.position.y)
        self.target.velocity.z = self.Kp1z*(self.target_pos[2] - self.cur_pos.pose.position.z)
        self.target.yaw = 0
        
    # Test method
    def test_posctl(self):
        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")

        while(1):
            self.vel_control()

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)