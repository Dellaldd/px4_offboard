import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Header
import threading,math
import numpy as np
from tf.transformations import quaternion_from_euler,euler_from_quaternion


def thread_job():
    rospy.spin()

class Controller:
    
    def __init__(self):

        # 无人机当前状态
        self.state = State()
        # 获取无人机的当前位置
        self.local_pos = PoseStamped()
        # 获取无人机当前位置作为参考
        self.cur_pos = PoseStamped()

        self.target = AttitudeTarget()
        self.local_vel = []
        self.local_pos = PoseStamped()
        self.local_q = self.local_pos.pose.orientation
        self.target.type_mask = int('00111', 2)
        self.target_pos = [1, 1, 1.5]
        self.des_acc = [0,0,0]
        
        self.gra = 9.8

        
        # 订阅无人机的状态
        rospy.Subscriber('mavros/state', State, self.stateCb)
        rospy.Subscriber("/vrpn_client_node/UAV/pose", PoseStamped, self.posCb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.refCb)
        rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.thrust_commands_cb)
        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size = 1)

        # 回调函数运行
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

    # 回调函数

    def stateCb(self, msg):
        self.state = msg
        print("current mode is: ",self.state.mode)

    def thrust_commands_cb(self,msg):
        rospy.loginfo("thrust: %f", msg.thrust)

    def refCb(self, msg):
        self.cur_pos = msg
        quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        euler = euler_from_quaternion(quaternion)
        self.local_q = msg.pose.orientation
        print("current_pos: " , self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z)
        # print("current_pose: ", euler)

    def posCb(self, msg):
        self.local_pos = msg
        self.local_pos.header.stamp = rospy.Time.now()
        vision_pub = rospy.Publisher("/mavros/vision_pose/pose",PoseStamped, queue_size = 1)
        vision_pub.publish(self.local_pos)
        self.local_q = msg.pose.orientation

    # def send_pos(self):
    #     rate = rospy.Rate(10)  # Hz
    #     self.target.header = Header()
    #     self.target.header.frame_id = "base_footprint"

    #     while not rospy.is_shutdown():
    #         self.target.header.stamp = rospy.Time.now()
    #         self.sp_pub.publish(self.target)
    #         try:  # prevent garbage in console output when thread is killed
    #             rate.sleep()
    #         except rospy.ROSInterruptException:
    #             pass

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
        self.target.header.stamp = rospy.Time.now()
        self.sp_pub.publish(self.target)

def main():
    print("start!")
    rospy.init_node('offboard_test_node', anonymous=True)
    cnt = Controller()
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        cnt.att_control() 
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
