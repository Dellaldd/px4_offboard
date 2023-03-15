#!/usr/bin/env python
# encoding: utf-8
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from scipy.spatial.transform import Rotation as R
import threading
import math
from std_msgs.msg import Header
from threading import Thread

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def thread_job():
    rospy.spin()

def angel2rad(angel):
    return angel/180*math.pi

class Controller:
    
    def __init__(self):

        # 无人机当前状态
        self.state = State()
        # 阈值
        self.thred = 0.1
        # 获取无人机的当前位置
        self.cur_pos = PoseStamped()
        # 无人机目标位置
        self.target_pos = PoseStamped()

        self.local_pos = PoseStamped()
        
        self.start_time = 0
        self.cur_time = 0
        self.t = 6

        # 订阅无人机的状态
        rospy.Subscriber('mavros/state', State, self.stateCb)
        rospy.Subscriber("/vrpn_client_node/UAV/pose", PoseStamped, self.posCb)
        # 订阅无人机当前位置
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.refCb)

        self.sp_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 1)

        # 回调函数运行
        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def send_pos(self):
        rate = rospy.Rate(100)  # Hz
        self.target_pos.header = Header()
        self.target_pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.target_pos.header.stamp = rospy.Time.now()
            self.sp_pub.publish(self.target_pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
       

    def stateCb(self, msg):
        self.state = msg
        print("current mode is: ",self.state.mode)

    def posCb(self, msg):
        self.local_pos = msg
        # quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        # euler = quaternion2euler(quaternion)
        self.local_pos.header.stamp = rospy.Time.now()
        vision_pub = rospy.Publisher("/mavros/vision_pose/pose",PoseStamped, queue_size = 1)
        vision_pub.publish(self.local_pos)

    def settarget(self,event):
        self.cur_time = event.current_real
        dett = (self.cur_time.to_sec() - self.start_time.to_sec())
        rospy.loginfo("dett:{}".format(dett))
        self.target_pos.pose.position.x = 1*math.sin(angel2rad(dett*60))
        self.target_pos.pose.position.y = 1*math.cos(angel2rad(dett*60))
        self.target_pos.pose.position.z = 1

    def refCb(self, msg):
        self.cur_pos = msg
        quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        euler = quaternion2euler(quaternion)
        print("current_pos: " , self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z)
        print("current_pose: ", euler)

# 主函数
def main():
    rospy.init_node('offboard_test_node', anonymous=True)
    cnt = Controller()
    # ROS main loop
    cnt.start_time = rospy.Time.now()
    cnt.cur_time = cnt.start_time
    rospy.loginfo("run mission")
    while (cnt.cur_time.to_sec()-cnt.start_time.to_sec()) < cnt.t:
        rospy.Timer(rospy.Duration(nsecs = 1e8), cnt.settarget, oneshot=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
