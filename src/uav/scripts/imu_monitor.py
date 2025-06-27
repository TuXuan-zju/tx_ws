#!/usr/bin/env python3 
import rospy
from mavros_msgs.srv import MessageInterval
from sensor_msgs.msg import Imu
import time

class IMUFrequencyMonitor:
    def __init__(self):
        rospy.init_node('imu_frequency_monitor')
        self.last_msg_time = None
        self.msg_count = 0
        self.freq = 0
        self.window_start = time.time()

        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        
        #  等待服务并初始化代理
        rospy.loginfo("Waiting for service /mavros/set_message_interval...")
        rospy.wait_for_service('/mavros/set_message_interval')
        self.set_interval_srv = rospy.ServiceProxy('/mavros/set_message_interval', MessageInterval)

        rospy.loginfo("Service proxy established.")

        #  最后再启动定时器
        self.timer = rospy.Timer(rospy.Duration(2.0), self.check_frequency)

        self.threshold = 90  # Hz

        rospy.loginfo("IMU Frequency Monitor fully started.")

    def imu_callback(self, msg):
        self.msg_count += 1
        now = time.time()
        if now - self.window_start >= 1.0:
            self.freq = self.msg_count / (now - self.window_start)
            self.msg_count = 0
            self.window_start = now

    def check_frequency(self, event):
        rospy.loginfo(f"[IMU Monitor] Current freq: {self.freq:.1f} Hz")
        if self.freq < self.threshold:
            rospy.logwarn("[IMU Monitor] IMU freq too low! Attempting to reset...")
            try:
                resp1 = self.set_interval_srv(105, 500)  # data_raw
                resp2 = self.set_interval_srv(31, 500)   # data
                rospy.loginfo(f"Set message interval results: raw={resp1.success}, fused={resp2.success}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        monitor = IMUFrequencyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass