import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from scipy.spatial.transform import Rotation as R
import threading

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

        # 订阅无人机当前位置作比较
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,self.refCb)
        self.sp_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size = 1)

        # 回调函数运行
        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()

        # 无人机目标位置
        self.target = PositionTarget()
        self.target.type_mask = int('101111111000', 2)
        self.target.coordinate_frame= 1
        self.target_pos = [0, 0, 1.5]

        # kp
        self.Kp1x = 0.2
        self.Kp1y = 0.2
        self.Kp1z = 0.2
       
    # 回调函数

    def refCb(self, msg):
        self.cur_pos = msg
        quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        euler = quaternion2euler(quaternion)
        print("current_pos: " , self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z)
        print("current_pose: ", euler)

    def stateCb(self, msg):
        self.state = msg
        print("current mode is: ",self.state.mode)

    def send_ctrl(self):
        self.target.header.stamp = rospy.Time.now()
        self.sp_pub.publish(self.target)

    def vel_control(self):
        self.target.velocity.x = self.Kp1x*(self.target[0] - self.cur_pos.pose.position.x)
        self.target.velocity.y = self.Kp1y*(self.target[1] - self.cur_pos.pose.position.y)
        self.target.velocity.z = self.Kp1z*(self.target[2] - self.cur_pos.pose.position.z)
        self.target.yaw = 0
        self.send_ctrl()
        

# 主函数
def main():
    print("start!")
    rospy.init_node('offboard_test_node', anonymous=True)

    cnt = Controller()
    rate = rospy.Rate(100)

    # ROS main loop
    while not rospy.is_shutdown():
        cnt.vel_control()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
