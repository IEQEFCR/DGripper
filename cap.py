import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class rotate_cap:
    def rod_roll_callback(self, msg):
        self.tactile_msg = msg.data
    def __init__(self):
        rospy.init_node('rotate_cap')
        self.teleop_pub = rospy.Publisher('/teleop', String, queue_size=10)
        rospy.Subscriber('/rod_roll', Float32MultiArray, self.rod_roll_callback)
        self.tactile_msg = [0, 0, 0, 0,0,0,0]
        self.state_list = ['idle', 'rolling','pick', 'place', 'rotate', 'release']
        self.state = 'idle'
    
    def pick(self):
        self.teleop_pub.publish('2') #高处对正瓶子
        print("高对正瓶子")
        rospy.sleep(5)
        self.teleop_pub.publish('3') #低处对正瓶子
        print("低对正瓶子")
        rospy.sleep(5)
        for i in range(2):
            self.teleop_pub.publish('0') #高处对正瓶盖
            print("高对正瓶盖")
            rospy.sleep(3)
        for i in range(2):
            self.teleop_pub.publish('1') #高处对正瓶盖
            print("低对正瓶盖")
            rospy.sleep(3)
        self.teleop_pub.publish('o') #夹爪合拢
        print("夹爪合拢")
        while self.tactile_msg[5] + self.tactile_msg[6] < 50:
            print("等待触摸到瓶盖")
        self.teleop_pub.publish(' ') #夹爪停止
        self.teleop_pub.publish('0')
        rospy.sleep(2)
        self.state = 'place'

    def place(self):
        rospy.sleep(2)
        self.teleop_pub.publish('2')#高处对正瓶子
        print("高对正瓶子")
        rospy.sleep(3)
        self.teleop_pub.publish('3')#低处对正瓶子
        print("低对正瓶子")
        rospy.sleep(3)
        tactile_pose_init = self.tactile_msg[1] + self.tactile_msg[3]
        while 1:
            self.teleop_pub.publish('c')#下放瓶盖
            rospy.sleep(0.5)
            if tactile_pose_init-self.tactile_msg[1] - self.tactile_msg[3] > 50: #瓶盖已经对接
                print("瓶盖已经对接")
                break
        self.teleop_pub.publish('l') #夹爪松开
        print("夹爪松开")
        rospy.sleep(3)
        self.teleop_pub.publish(' ') #夹爪停止
        print("夹爪停止")



    def run (self):
        while 1:
            if self.state == 'idle':
                print("Ready to start, press 's' to start")
                key = input()
                if key == 's':
                    self.state = 'pick'
            elif self.state == 'pick':
                self.pick()
            elif self.state == 'place':
                self.place()
                


if __name__ == '__main__':
    rospy.init_node('rotate_cap')
    a= rotate_cap()
    a.run()

