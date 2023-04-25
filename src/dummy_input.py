#!/usr/bin/python3.8
import rospy
import time
from std_msgs.msg import Float32MultiArray
class DummyInput:
    def __init__(self):
        rospy.init_node('dummy_input', anonymous=True)
        self.rate = rospy.Rate(60) # 60hz
        self.dummypub = rospy.Publisher('/dummy_input', Float32MultiArray, queue_size=1)
    def main(self):
        while not rospy.is_shutdown():
            data = Float32MultiArray(data=[0, 0, 0])
            self.dummypub.publish(data)
            self.rate.sleep()
if __name__ == '__main__':
    dummy_input = DummyInput()
    dummy_input.main()
