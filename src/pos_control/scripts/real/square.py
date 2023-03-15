import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from scipy.spatial.transform import Rotation as R
import threading
import math
import numpy as np

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def thread_job():
    rospy.spin()

class Controller:
    
    def __init__(self):

        self.state = State()
        self.thred = 0.1
        self.cur_pos = PoseStamped()
        self.target_pos = PoseStamped()
        
        self.trajs = [[1, 0, 1.5], [-1, 0, 1.0], [-1, -1, 0.5], [1, -1, 1.5]]

        # current state
        rospy.Subscriber('mavros/state', State, self.stateCb)
        # current position
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.refCb)
        rospy.Subscriber("/vrpn_client_node/UAV/pose", PoseStamped, self.posCb)
        self.sp_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 1)

        # 回调函数运行
        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()
       
    def posCb(self, msg):
        self.local_pos = msg
        # quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        # euler = quaternion2euler(quaternion)
        self.local_pos.header.stamp = rospy.Time.now()
        vision_pub = rospy.Publisher("/mavros/vision_pose/pose",PoseStamped, queue_size = 1)
        vision_pub.publish(self.local_pos)

    def stateCb(self, msg):
        self.state = msg
        print("current mode is: ",self.state.mode)

    def settarget(self, traj):
        self.target_pos.pose.position.x = traj[0]
        self.target_pos.pose.position.y = traj[1]
        self.target_pos.pose.position.z = traj[2]

    def refCb(self, msg):
        self.cur_pos = msg
        quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        euler = quaternion2euler(quaternion)
        print("current_pos: " , self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z)
        print("current_pose: ", euler)


    def distance(self):
        diff = np.array([self.target_pos.pose.position.x - self.cur_pos.pose.position.x, 
            self.target_pos.pose.position.y - self.cur_pos.pose.position.y,
            self.target_pos.pose.position.z - self.cur_pos.pose.position.z])
        dis = np.linalg.norm(diff)
        # dis = math.sqrt(diff[0]*diff[0]+ diff[1]*diff[1]+ diff[2]*diff[2])
        return dis
# 主函数
def main():
    print("start!")
    rospy.init_node('offboard_test_node', anonymous=True)
    cnt = Controller()
    rate = rospy.Rate(100)

    # ROS main loop
    while(1):
        for traj in cnt.trajs:
            cnt.settarget(traj)
            while not rospy.is_shutdown() and cnt.distance() > cnt.thred:
                cnt.target_pos.header.stamp = rospy.Time.now()
                cnt.sp_pub.publish(cnt.target_pos) 
                rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
