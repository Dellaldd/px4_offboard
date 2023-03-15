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
import numpy as np
# from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_from_euler,euler_from_quaternion

def angel2rad(angel):
    return angel/180*math.pi

def thread_job():
    rospy.spin()

class MavrosOffboardPosctlTest(MavrosTestCommon):

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.target = AttitudeTarget()
        self.local_vel = []
        self.local_pos = PoseStamped()
        self.local_q = self.local_pos.pose.orientation
        self.target.type_mask = int('00111', 2)
        self.target_pos = [1, 1, 1.5]
        self.des_acc = [0,0,0]
        
        self.gra = 9.8

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.posCb)
        rospy.Subscriber("/mavros/local_position/velocity_local",TwistStamped,self.velCb)
        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size = 1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()

        self.thr2acc = 9.8 / 0.3

        # kp
        self.Kp1x = 1
        self.Kp1y = 1
        self.Kp1z = 1

        self.Kp2x = 1
        self.Kp2y = 1
        self.Kp2z = 1
        

    def posCb(self, msg):
        self.local_pos = msg
        self.local_q = msg.pose.orientation
    
    def velCb(self,msg):
        self.local_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

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

    def att_control(self):
        self.des_acc[0] = self.Kp2x*(self.Kp1x*(self.target_pos[0]-self.local_pos.pose.position.x)-self.local_vel[0])
        self.des_acc[1] = self.Kp2y*(self.Kp1y*(self.target_pos[1]-self.local_pos.pose.position.y)-self.local_vel[1])
        self.des_acc[2] = self.Kp2z*(self.Kp1z*(self.target_pos[2]-self.local_pos.pose.position.z)-self.local_vel[2])
        euler = euler_from_quaternion(np.array([self.local_q.x,self.local_q.y,self.local_q.z,self.local_q.w]), axes='sxyz')
        sin = math.sin(euler[2])
        cos = math.cos(euler[2])
        roll = (self.des_acc[0] * sin - self.des_acc[1] * cos )/self.gra
        pitch = (self.des_acc[0] * cos + self.des_acc[1] * sin )/self.gra
        q = quaternion_from_euler(roll,pitch,0, axes='sxyz')
        thrust = self.des_acc[2] + self.gra
        self.target.orientation.x = q[0]
        self.target.orientation.y = q[1]
        self.target.orientation.z = q[2]
        self.target.orientation.w = q[3]
        self.target.thrust = thrust/self.thr2acc
        # rospy.loginfo("q:%f,%f,%f,%f",q[0],q[1],q[2],q[3])
        # rospy.loginfo("thrust:%f",self.target.thrust)
        
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
            self.att_control()

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)