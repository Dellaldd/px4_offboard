#!/usr/bin/env python2
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

PKG = 'px4'

def angel2rad(angel):
    return angel/180*math.pi

def thread_job():
    rospy.spin()

def loaddata(path):
    data = np.loadtxt(open(path,"rb"),delimiter=",",skiprows=2) 
    q = np.column_stack((data[:,0],data[:,5],data[:,6],data[:,7],data[:,4],data[:,20],data[:,21],data[:,22],data[:,23]))
    return q

class MavrosOffboardPosctlTest(MavrosTestCommon):

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.target = AttitudeTarget()
        self.start_time = 0
        self.cur_time = 0

        self.target.orientation.x = 0
        self.target.orientation.y = 0
        self.target.orientation.z = 0
        self.target.orientation.w = 1
        self.target.thrust = 0

        self.local_pos = PoseStamped()
        self.init_pos = PoseStamped()
        self.init_pos.pose.position.x = -5
        self.init_pos.pose.position.y = 4.5
        self.init_pos.pose.position.z = 1.2

        self.target.type_mask = int('00111', 2)
        self.gra = 9.8
        self.local_q = self.local_pos.pose.orientation
        self.local_vel = []

        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size = 1)
        self.init_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 1)

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.posCb)
        rospy.Subscriber("/mavros/local_position/velocity_local",TwistStamped,self.velCb)

        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()

        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        self.thr2acc = 9.8 / 0.3
        self.thred = 1

        # kp
        self.Kp1x = 1
        self.Kp1y = 1
        self.Kp1z = 1

        self.Kp2x = 1
        self.Kp2y = 1
        self.Kp2z = 1

        self.des_acc = [0,0,0]
        
        self.gra = 9.8

    def settarget(self,data):  
        thrust = (data[5]+data[6]+data[7]+data[8])
        self.target.orientation.x = data[1]
        self.target.orientation.y = data[2]
        self.target.orientation.z = data[3]
        self.target.orientation.w = data[4]
        self.target.thrust = thrust/self.thr2acc
    
    def posCb(self, msg):
        self.local_pos = msg

    def velCb(self,msg):
        self.local_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

    def send_pos(self):
        rate = rospy.Rate(200)  # Hz
        self.target.header = Header()
        self.target.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.target.header.stamp = rospy.Time.now()
            self.sp_pub.publish(self.target)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

        # while not rospy.is_shutdown():
        #     if self.flag == 0:
        #         self.init_pos.header.stamp = rospy.Time.now()
        #         self.init_pub.publish(self.init_pos)
        #     else:
        #         self.target.header.stamp = rospy.Time.now()
        #         self.sp_pub.publish(self.target)
        #     try:  # prevent garbage in console output when thread is killed
        #         rate.sleep()
        #     except rospy.ROSInterruptException:
        #         pass
    
    def att_control(self):
        self.des_acc[0] = self.Kp2x*(self.Kp1x*(self.init_pos.pose.position.x-self.local_pos.pose.position.x)-self.local_vel[0])
        self.des_acc[1] = self.Kp2y*(self.Kp1y*(self.init_pos.pose.position.y-self.local_pos.pose.position.y)-self.local_vel[1])
        self.des_acc[2] = self.Kp2z*(self.Kp1z*(self.init_pos.pose.position.z-self.local_pos.pose.position.z)-self.local_vel[2])
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

    def distance(self):
        diff = np.array([self.init_pos.pose.position.x - self.local_pos.pose.position.x, 
            self.init_pos.pose.position.y - self.local_pos.pose.position.y,
            self.init_pos.pose.position.z - self.local_pos.pose.position.z])
        dis = np.linalg.norm(diff)
        return dis

    def test_posctl(self):

        path = "/home/ldd/UAV/time_optimal_trajectory/example/result.csv"
        data = loaddata(path)
        rospy.loginfo(data.shape)
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")

        rate = rospy.Rate(40)

        while not rospy.is_shutdown() and self.distance() > self.thred: 
            self.att_control()
            rate.sleep()
        
        rospy.loginfo("have arrived place!")

        self.start_time = rospy.Time.now()
        self.cur_time = self.start_time
        i = 0
        
        while not rospy.is_shutdown() and i < data.shape[0]-1:
            self.cur_time = rospy.Time.now()
            self.settarget(data[i,:])
            # rospy.loginfo(abs(self.cur_time.to_sec()-self.start_time.to_sec()-data[i,0]))
            # if math.isclose(self.cur_time.to_sec()-self.start_time.to_sec(), data[i,0], rel_tol=1e-3):
            if (self.cur_time.to_sec()-self.start_time.to_sec()) < data[i+1,0]:      
                i = i + 1
            self.target.header.stamp = rospy.Time.now()
            self.sp_pub.publish(self.target)
            rate.sleep()

        # while not rospy.is_shutdown() and i < data.shape[0]:
        #     self.settarget(data[i,:])
        #     i = i + 1
        #     rate.sleep()

        # while not rospy.is_shutdown():
        #     rate.sleep()
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)