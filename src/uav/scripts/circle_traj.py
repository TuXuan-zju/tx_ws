#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node('circle_trajectory_node')

    state_sub = rospy.Subscriber('mavros/state', State, state_cb)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    rate = rospy.Rate(20)

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2  # 起飞高度

    # 预发布 setpoint，满足 OFFBOARD 模式进入条件
    for _ in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # 设置模式并解锁
    set_mode_client(custom_mode='OFFBOARD')
    arming_client(True)

    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        radius = 2.0
        speed = 0.5  # rad/s

        # 圆周轨迹
        pose.pose.position.x = radius * math.cos(speed * t)
        pose.pose.position.y = radius * math.sin(speed * t)
        pose.pose.position.z = 2  # 保持高度

        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    main()
