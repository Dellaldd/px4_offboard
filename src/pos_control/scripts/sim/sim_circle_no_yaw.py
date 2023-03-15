from __future__ import division

PKG = 'px4'

import rospy
import math
import rosbag
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ParamValue,PositionTarget
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
import threading
# from tf.transformations import euler_from_quaternion

def angel2rad(angel):
    return angel/180*math.pi

def thread_job():
    rospy.spin()

class MavrosOffboardPosctlTest(MavrosTestCommon):

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.target_sp = PoseStamped()
        self.local_pos = PoseStamped()
        self.cur_pos = PoseStamped()

        self.target_sp.pose.position.x = 0
        self.target_sp.pose.position.y = 0
        self.target_sp.pose.position.z = 1
        self.ifcircle = True

        self.start_time = 0
        self.cur_time = 0
        self.t = 6
        self.thred = 0.2

        self.sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 1)

        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def send_pos(self):
        rate = rospy.Rate(100)  # Hz
        self.target_sp.header = Header()
        self.target_sp.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.target_sp.header.stamp = rospy.Time.now()
            self.sp_pub.publish(self.target_sp)
            try:  
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def refCb(self, msg):
        self.cur_pos = msg
        # quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        print("current_pos: " , self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z)

    def distance(self):
        diff = np.array([self.target_sp.pose.position.x - self.cur_pos.pose.position.x, 
            self.target_sp.pose.position.y - self.cur_pos.pose.position.y,
            self.target_sp.pose.position.z - self.cur_pos.pose.position.z])
        dis = np.linalg.norm(diff)
        return dis

    def settarget(self):
        # self.cur_time = event.current_real
        self.cur_time = rospy.Time.now()
        dett = (self.cur_time.to_sec() - self.start_time.to_sec())
        rospy.loginfo("dett:{}".format(dett))
        # print("dett:",dett/1e8)
        self.target_sp.pose.position.x = 1*math.sin(angel2rad(360/self.t*dett))
        self.target_sp.pose.position.y = 1-1*math.cos(angel2rad(360/self.t*dett))
        self.target_sp.pose.position.z = 1
        
    # Test method
    def test_posctl(self):
        
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        rate = rospy.Rate(100)
        rospy.loginfo("run mission")

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.refCb)
        add_thread = threading.Thread(target = thread_job)
        add_thread.start()
        
        while not rospy.is_shutdown():
            if (self.distance < self.thred) and (not self.ifcircle):
                self.ifcircle = True
                self.start_time = rospy.Time.now()
                print("start circle!")

            if self.ifcircle:
                self.settarget()

            # self.target_sp.header.stamp = rospy.Time.now()
            # sp_pub.publish(self.target_sp) 
            rate.sleep()

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)