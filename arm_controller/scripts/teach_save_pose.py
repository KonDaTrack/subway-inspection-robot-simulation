#!/usr/bin/env python3
import rospy
import yaml
import os
from std_msgs.msg import String
from moveit_commander import RobotCommander, MoveGroupCommander

class ArmPoseTeachSaver:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('arm_pose_teach_saver', anonymous=True)
        
        # 初始化MoveIt!
        self.robot = RobotCommander()
        self.move_group = MoveGroupCommander("manipulator")  # 替换为你的move_group名称
        
        # 保存文件路径
        self.save_path = "/home/robot/catkin_ws/src/arm_controller/arm_poses.yaml"
        
        # 加载已有位姿
        self.poses = self.load_poses()
        
        # 订阅保存指令话题
        self.save_sub = rospy.Subscriber('/save_arm_pose', String, self.save_pose_callback, queue_size=10)
        
        rospy.loginfo("机械臂示教保存节点已启动")
        rospy.loginfo(f"保存路径: {self.save_path}")
        rospy.loginfo("使用方法: 发布字符串到/save_arm_pose话题（如'pose1'），保存当前机械臂位姿")

    def load_poses(self):
        """加载已保存的位姿文件"""
        if os.path.exists(self.save_path):
            with open(self.save_path, 'r') as f:
                return yaml.safe_load(f) or {}
        return {}

    def save_pose_callback(self, msg):
        """保存当前机械臂位姿"""
        pose_name = msg.data.strip()
        if not pose_name:
            rospy.logwarn("保存失败：位姿名称为空")
            return
        
        # 获取当前关节角度（推荐，比位姿更稳定）
        joint_states = self.move_group.get_current_joint_values()
        joint_names = self.move_group.get_joints()
        
        # 可选：获取末端执行器位姿（笛卡尔坐标）
        ee_pose = self.move_group.get_current_pose().pose
        
        # 构造保存数据
        self.poses[pose_name] = {
            'joint_names': joint_names,
            'joint_values': joint_states,
            'ee_pose': {
                'position': {
                    'x': ee_pose.position.x,
                    'y': ee_pose.position.y,
                    'z': ee_pose.position.z
                },
                'orientation': {
                    'x': ee_pose.orientation.x,
                    'y': ee_pose.orientation.y,
                    'z': ee_pose.orientation.z,
                    'w': ee_pose.orientation.w
                }
            }
        }
        
        # 写入文件
        with open(self.save_path, 'w') as f:
            yaml.dump(self.poses, f, default_flow_style=False)
        
        rospy.loginfo(f"成功保存位姿: {pose_name}")
        rospy.loginfo(f"当前已保存位姿列表: {list(self.poses.keys())}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = ArmPoseTeachSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass