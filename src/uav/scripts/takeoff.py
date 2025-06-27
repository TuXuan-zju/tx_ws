#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node("offboard_takeoff_node")

    # 订阅 PX4 状态
    rospy.Subscriber("mavros/state", State, state_cb)
    
    # 发布目标位置
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # 等待服务可用
    rospy.loginfo("Waiting for /mavros services...")
    rospy.wait_for_service("mavros/cmd/arming")
    rospy.wait_for_service("mavros/set_mode")

    # 绑定服务
    arm_srv = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_srv = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # 发布频率需 > 2Hz（PX4 要求）
    rate = rospy.Rate(20)

    # 设置目标位置
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2  # 起飞高度

    # 等待 PX4 FCU 连接完成
    rospy.loginfo("Waiting for FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("FCU connected")

    # 发布一些 setpoint 以满足 OFFBOARD 模式条件
    rospy.loginfo("Sending initial setpoints...")
    for _ in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # 设置模式为 OFFBOARD
    offb_resp = set_mode_srv(custom_mode="OFFBOARD")
    if offb_resp.mode_sent:
        rospy.loginfo("OFFBOARD mode set")
    else:
        rospy.logwarn("Failed to set OFFBOARD mode")

    # 解锁（arming）
    arm_resp = arm_srv(True)
    if arm_resp.success:
        rospy.loginfo("Vehicle armed")
    else:
        rospy.logwarn("Failed to arm vehicle")

    # 持续发布 setpoint，保持悬停
    rospy.loginfo("Publishing position setpoint...")
    while not rospy.is_shutdown():
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass