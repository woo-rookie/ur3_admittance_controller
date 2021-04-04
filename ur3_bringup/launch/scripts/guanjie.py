#!/usr/bin/env python
# -*- coding: utf-8 -*-

#moveit_fk_demo.py
 
import rospy, sys
import moveit_commander
 
class MoveItFkDemo:
    def __init__(self):
 
        # 初始化move_group的API,出现roscpp是因为底层是通过C++进行实现的
        moveit_commander.roscpp_initialize(sys.argv)
 
        # 初始化ROS节点，节点名为'moveit_fk_demo'
        rospy.init_node('moveit_fk_demo', anonymous=True)       
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
        
        # 设置机械臂运动的允许误差值，单位弧度
        arm.set_goal_joint_tolerance(0.001)
 
        # 设置允许的最大速度和加速度，范围0~1
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 控制机械臂先回到初始化位置，home是setup assistant中设置的
        arm.set_named_target('home')
        arm.go()  #让机械臂先规划，再运动，阻塞指令，直到机械臂到达home后再向下执行
        rospy.sleep(1)
         
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [0, -1.536, 0, 0.0, 0, 0]
        arm.set_joint_value_target(joint_positions)  #设置关节值作为目标值
                 
        # 控制机械臂完成运动
        arm.go() 
        rospy.sleep(1)
 
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
