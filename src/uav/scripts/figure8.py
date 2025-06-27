#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node("figure8_flight_node")

    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("mavros/cmd/arming")
    arm_srv = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("mavros/set_mode")
    set_mode_srv = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    pose = PoseStamped()
    pose.pose.position.z = 2.0  # 起飞高度

    # 先发100个setpoint保证进入OFFBOARD
    for _ in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    set_mode_srv(custom_mode="OFFBOARD")
    arm_srv(True)

    # 八字轨迹参数
    R = 2.0  # 半径
    omega = 0.2  # 角速度
    t0 = rospy.Time.now().to_sec()

    rospy.loginfo("开始执行八字飞行")

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        pose.pose.position.x = R * math.sin(omega * t)
        pose.pose.position.y = R * math.sin(omega * t) * math.cos(omega * t)
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == "__main__":
    main()
