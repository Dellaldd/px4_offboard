import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from scipy.spatial.transform import Rotation as R
import threading
import math

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def thread_job():
    rospy.spin()

class Controller:
    
    def __init__(self):

        # 无人机当前状态
        self.state = State()
        # 获取无人机的当前位置
        self.local_pos = PoseStamped()
        # 无人机目标位置
        self.target_pos = PoseStamped()
        # 获取无人机当前位置作为参考
        self.cur_pos = PoseStamped()
       

    # 回调函数
    # 获取当前位置
    def posCb(self, msg):
        self.local_pos = msg
        quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        euler = quaternion2euler(quaternion)
        self.local_pos.header.stamp = rospy.Time.now()
        vision_pub = rospy.Publisher("/mavros/vision_pose/pose",PoseStamped, queue_size = 1)
        vision_pub.publish(self.local_pos)
        # print("optitrack_pos:" , self.local_pos.pose.position.x,self.local_pos.pose.position.y,self.local_pos.pose.position.z)
        # print("optitrack_pose:",euler)

    def stateCb(self, msg):
        self.state = msg
        print("current mode is: ",self.state.mode)

    def refCb(self, msg):
        self.cur_pos = msg
        quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        euler = quaternion2euler(quaternion)
        print("current_pos: " , self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z)
        print("current_pose: ", euler)

    def settarget(self, traj):
        self.target_pos.pose.position.x = traj[0]
        self.target_pos.pose.position.y = traj[1]
        self.target_pos.pose.position.z = traj[2]

    def distance(self):
        diff = [self.target_pos.pose.position.x - self.local_pos.pose.position.x, 
            self.target_pos.pose.position.y - self.local_pos.pose.position.y,
            self.target_pos.pose.position.z - self.local_pos.pose.position.z]
        dis = math.sqrt(diff[0]*diff[0]+ diff[1]*diff[1]+ diff[2]*diff[2])
        return dis
# 主函数
def main():
    print("start!")
    thred = 0.1
    rospy.init_node('offboard_test_node', anonymous=True)
    cnt = Controller()
    rate = rospy.Rate(100)

    # 订阅无人机的状态
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    # 通过optitrack订阅无人机的当前位置
    rospy.Subscriber("/vrpn_client_node/UAV/pose", PoseStamped, cnt.posCb)
    # 订阅无人机当前位置作比较
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped,cnt.refCb)

    add_thread = threading.Thread(target = thread_job)
    add_thread.start()

    trajs = [[1, 0, 1.5], [-1, 0, 1.0], [-1, -1, 0.5], [1, -1, 1.5]]

    sp_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 1)
    
    # ROS main loop
    while(1):
        for traj in trajs:
            cnt.settarget(traj)
            while not rospy.is_shutdown() and cnt.distance() > thred:
                cnt.target_pos.header.stamp = rospy.Time.now()
                sp_pub.publish(cnt.target_pos) 
                rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
