import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from scipy.spatial.transform import Rotation as R
import threading
from std_msgs.msg import Header

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
        # 获取无人机当前位置作为参考
        self.cur_pos = PoseStamped()
        
        # 订阅无人机的状态
        rospy.Subscriber('mavros/state', State, self.stateCb)
        rospy.Subscriber("/vrpn_client_node/UAV/pose", PoseStamped, self.posCb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.refCb)
        rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.thrust_commands_cb)
        self.sp_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 1)

        # 回调函数运行
        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()

        self.pos_thread = threading.Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        # 无人机目标位置
        self.target = PoseStamped()
        self.target.pose.position.x = 0
        self.target.pose.position.y = 0
        self.target.pose.position.z = 1.5
       

    # 回调函数

    def stateCb(self, msg):
        self.state = msg
        print("current mode is: ",self.state.mode)

    def thrust_commands_cb(self,msg):
        rospy.loginfo("thrust: %f", msg.thrust)

    def refCb(self, msg):
        self.cur_pos = msg
        print("cur_pos:", self.cur_pos.pose.position.x, self.cur_pos.pose.position.y, self.cur_pos.pose.position.z)

    def posCb(self, msg):
        self.local_pos = msg
        # quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        # euler = quaternion2euler(quaternion)
        self.local_pos.header.stamp = rospy.Time.now()
        vision_pub = rospy.Publisher("/mavros/vision_pose/pose",PoseStamped, queue_size = 1)
        vision_pub.publish(self.local_pos)

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

# 主函数
def main():
    print("start!")
    rospy.init_node('offboard_test_node', anonymous=True)

    cnt = Controller()
    rate = rospy.Rate(100)

    # ROS main loop
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
