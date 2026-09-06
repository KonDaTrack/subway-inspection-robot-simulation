#!/usr/bin/env python3
import rospy
import yaml
import os
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
import sys
import numpy as np

# 自定义服务消息 - 注意：MoveArmToPose服务现在只有请求，没有响应字段
from arm_controller.srv import MoveArmToPose

class ArmMoveService:
    def __init__(self):
        rospy.init_node('arm_move_service', anonymous=True)
        
        # 初始化MoveIt!
        self.robot = RobotCommander()
        self.move_group = MoveGroupCommander("manipulator")
        self.scene = PlanningSceneInterface()
        
        # 配置参数
        self.move_group.set_planning_time(5.0)
        self.move_group.set_goal_joint_tolerance(0.001)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        # 位姿文件
        self.pose_file = "/home/robot/catkin_ws/src/arm_controller/arm_poses.yaml"
        rospy.loginfo(f"【Noetic】位姿文件路径: {self.pose_file}")
        self.poses = self._load_poses()
        
        # 创建服务
        self.move_arm_service = rospy.Service('/move_arm_to_pose', MoveArmToPose, self.move_arm_callback)
        
        rospy.loginfo("===== Noetic版机械臂服务启动成功 =====")
        rospy.loginfo(f"加载位姿列表: {list(self.poses.keys())}")

    def _load_poses(self):
        if not os.path.exists(self.pose_file):
            rospy.logerr(f"【Noetic】位姿文件不存在: {self.pose_file}")
            return {}
        try:
            with open(self.pose_file, 'r', encoding='utf-8') as f:
                poses = yaml.safe_load(f) or {}
            valid_poses = {}
            for name, data in poses.items():
                if len(data["joint_values"]) != 7:
                    rospy.logwarn(f"【Noetic】位姿{name}关节数≠7，跳过")
                    continue
                valid_poses[name] = data
            return valid_poses
        except Exception as e:
            rospy.logerr(f"【Noetic】加载位姿失败: {str(e)}")
            return {}

    # 四个机械臂动作函数
    def action_home(self):
        """机械臂回到初始位置 - 调用MoveIt配置的home状态"""
        rospy.loginfo("执行动作: 回到初始位置")
        rospy.loginfo("执行动作: home")
        self._execute_pose("home")
    
    def action1(self):
        rospy.loginfo("执行动作: pose1")
        self._execute_pose("pose1")
        self.move_group.stop()
    
    def action2(self):
        rospy.loginfo("执行动作: pose2")
        success = self._execute_pose("pose2")
        if success:
            rospy.sleep(0.5)
            rospy.loginfo("回到home位置")
            self.action_home()
        else:
            rospy.logerr("执行pose2失败，跳过回到home位置")
    
    def action_none(self):
        rospy.loginfo("执行动作: None")

    def _execute_pose(self, pose_name):
        """执行移动到指定位姿的通用函数"""
        if pose_name not in self.poses:
            rospy.logerr(f"位姿 '{pose_name}' 不存在")
            return False
        
        target_joints = self.poses[pose_name]['joint_values']
        
        try:
            rospy.loginfo(f"执行位姿 '{pose_name}'")
            
            # 停止当前运动并清除目标
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            # 等待一小段时间让机器人状态稳定
            rospy.sleep(0.1)
            
            # 设置起始状态为当前机器人状态
            self.move_group.set_start_state_to_current_state()
            
            # 对于home位置，使用更保守的速度
            if pose_name == "home":
                self.move_group.set_max_velocity_scaling_factor(0.3)
                self.move_group.set_max_acceleration_scaling_factor(0.3)
            else:
                self.move_group.set_max_velocity_scaling_factor(0.5)
                self.move_group.set_max_acceleration_scaling_factor(0.5)
            
            # 设置关节目标
            self.move_group.set_joint_value_target(target_joints)
            
            # 规划
            plan_result = self.move_group.plan()
            planning_success = plan_result[0]
            traj_msg = plan_result[1]
            
            if not planning_success or not traj_msg or not traj_msg.joint_trajectory.points:
                rospy.logwarn(f"规划到 '{pose_name}' 失败")
                return False
            
            # 检查轨迹起点
            if traj_msg.joint_trajectory.points:
                start_point = traj_msg.joint_trajectory.points[0]
                rospy.loginfo(f"轨迹起点关节位置: {start_point.positions}")
            
            # 执行
            execute_success = self.move_group.execute(traj_msg, wait=True)
            self.move_group.stop()
            
            if execute_success:
                rospy.loginfo(f"成功到达位姿 '{pose_name}'")
                return True
            else:
                rospy.logwarn(f"执行到 '{pose_name}' 失败")
                return False
                
        except Exception as e:
            rospy.logerr(f"执行位姿 '{pose_name}' 时发生异常: {str(e)}")
            return False

    def move_arm_callback(self, req):
        """服务回调函数，根据action_name调用对应的动作函数"""
        action_name = req.action_name.strip()
        rospy.loginfo(f"收到动作请求: {action_name}")
        
        # 根据action_name调用对应的动作函数
        if action_name == "home":
            self.action_home()
        elif action_name == "action1":
            self.action1()
        elif action_name == "action2":
            self.action2()
        else:
            self.action_none()
        
        # 返回空响应
        return MoveArmToPose._response_class()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.loginfo("【Noetic】等待move_group服务...")
        rospy.wait_for_service('/get_planning_scene', timeout=30.0)
        service = ArmMoveService()
        service.run()
    except rospy.ROSException as e:
        rospy.logerr(f"【Noetic】等待服务超时: {str(e)}")
        sys.exit(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("【Noetic】节点被中断")
    except Exception as e:
        rospy.logerr(f"【Noetic】启动失败: {str(e)}")
        sys.exit(1)
