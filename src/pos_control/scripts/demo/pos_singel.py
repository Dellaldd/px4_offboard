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

        
        self.state = State()
        
        self.local_pos = PoseStamped()
        
        self.target_sp = PoseStamped()
        self.target_sp.pose.position.x = 0
        self.target_sp.pose.position.y = 0
        self.target_sp.pose.position.z = 1.5
        
        self.cur_pos = PoseStamped()
       

    
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



def main():
    print("start!")

    rospy.init_node('offboard_test_node', anonymous=True)
    cnt = Controller()
    rate = rospy.Rate(100)

    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber("/vrpn_client_node/UAV/pose", PoseStamped, cnt.posCb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped,cnt.refCb)

    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    sp_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 1)

    while not rospy.is_shutdown():
        cnt.target_sp.header.stamp = rospy.Time.now()
        sp_pub.publish(cnt.target_sp) 
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
