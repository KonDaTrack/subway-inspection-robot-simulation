#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import abc
from waterplus_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex, GetWaypointByName
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from arm_controller.srv import MoveArmToPose
from xy_first_planner.srv import MoveToPose

class PoseWithActionBase(abc.ABC):
    def __init__(self, pose=None, action_name=None):
        self.pose = pose
        self.action_name = action_name
        
        # 初始化XYFirstPlanner服务客户端
        rospy.loginfo("等待XYFirstPlanner服务...")
        self.xy_first_planner_client = rospy.ServiceProxy('/xy_first_planner/move_to_pose', MoveToPose)
        rospy.wait_for_service('/xy_first_planner/move_to_pose', timeout=10.0)
        rospy.loginfo("XYFirstPlanner服务连接成功")
        
        # 初始化arm_controller服务客户端
        rospy.loginfo("等待arm_controller服务...")
        self.move_arm_client = rospy.ServiceProxy('/move_arm_to_pose', MoveArmToPose)
        rospy.wait_for_service('/move_arm_to_pose', timeout=10.0)
        rospy.loginfo("arm_controller服务连接成功")
        
        # 保留move_base客户端（可选，用于向后兼容）
        self.move_base_client = None
    
    @abc.abstractmethod
    def action(self):
        pass
    
    def call_arm_action(self):
        """调用arm_controller服务执行机械臂动作"""
        if self.action_name is None:
            rospy.logwarn("未指定机械臂动作名称，跳过")
            return False
        
        try:
            rospy.loginfo("调用机械臂动作: %s", self.action_name)
            response = self.move_arm_client(self.action_name)
            
            # # 根据动作类型设置不同的等待时间
            # if self.action_name == "action2":
            #     # action2包含两个动作：pose2 + home，需要更多时间
            #     rospy.loginfo("action2包含两个动作，等待更长时间...")
            #     rospy.sleep(8.0)  # 给机械臂足够时间完成两个动作
            # elif self.action_name == "home":
            #     rospy.sleep(4.0)  # home动作需要时间
            # else:
            #     rospy.sleep(5.0)  # 其他动作等待时间
            
            rospy.loginfo("机械臂动作 %s 调用成功", self.action_name)
            return True
        except rospy.ServiceException as e:
            rospy.logerr("调用机械臂动作服务失败: %s", e)
            return False
        except Exception as e:
            rospy.logerr("调用机械臂动作时发生异常: %s", e)
            return False
    
    def send_navigation_goal(self):
        """发送导航目标到XYFirstPlanner"""
        if self.pose is None:
            rospy.logwarn("未设置导航点位置，无法发送导航目标")
            return False
        
        try:
            # 创建PoseStamped消息
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose = self.pose
            
            rospy.loginfo("发送导航目标到XYFirstPlanner: (%.2f, %.2f, %.2f)", 
                         self.pose.position.x, self.pose.position.y, self.pose.position.z)
            
            # 调用XYFirstPlanner服务
            response = self.xy_first_planner_client(target_pose)
            
            if response.success:
                rospy.loginfo("XYFirstPlanner导航成功")
                return True
            else:
                rospy.logwarn("XYFirstPlanner导航失败")
                return False
                
        except rospy.ServiceException as e:
            rospy.logerr("调用XYFirstPlanner服务失败: %s", e)
            return False
        except Exception as e:
            rospy.logerr("发送导航目标时发生异常: %s", e)
            return False

class MoveitAction1(PoseWithActionBase):
    def __init__(self, pose=None, action_name="action1"):
        super().__init__(pose, action_name)
    
    def action(self):
        rospy.loginfo("到达导航点: (%.2f, %.2f, %.2f),执行Action1", 
                     self.pose.position.x, self.pose.position.y, self.pose.position.z)
        # 调用机械臂动作
        if self.action_name:
            self.call_arm_action()

class MoveitAction2(PoseWithActionBase):
    def __init__(self, pose=None, action_name="action2"):
        super().__init__(pose, action_name)
    
    def action(self):
        rospy.loginfo("到达导航点: (%.2f, %.2f, %.2f),执行Action2", 
                     self.pose.position.x, self.pose.position.y, self.pose.position.z)
        # 调用机械臂动作
        if self.action_name:
            self.call_arm_action()

class MoveitAction3(PoseWithActionBase):
    def __init__(self, pose=None, action_name="action3"):
        super().__init__(pose, action_name)
    
    def action(self):
        rospy.loginfo("到达导航点: (%.2f, %.2f, %.2f),执行Action3", 
                     self.pose.position.x, self.pose.position.y, self.pose.position.z)
        # 调用机械臂动作
        if self.action_name:
            self.call_arm_action()

class MoveitActionHome(PoseWithActionBase):
    def __init__(self, pose=None, action_name="home"):
        super().__init__(pose, action_name)
    
    def action(self):
        rospy.loginfo("到达导航点: (%.2f, %.2f, %.2f),执行ActionHome", 
                     self.pose.position.x, self.pose.position.y, self.pose.position.z)
        # 调用机械臂动作
        if self.action_name:
            self.call_arm_action()

class MoveitActionNone(PoseWithActionBase):
    def __init__(self, pose=None, action_name="none"):
        super().__init__(pose, action_name)
    
    def action(self):
        rospy.loginfo("到达导航点: (%.2f, %.2f, %.2f),执行ActionNone", 
                     self.pose.position.x, self.pose.position.y, self.pose.position.z)
        # 调用机械臂动作
        if self.action_name:
            self.call_arm_action()

class MissionHandler:
    def __init__(self):
        try:
            rospy.init_node('mission_handler', anonymous=True)
        except:
            pass  
        
        rospy.loginfo("等待waterplus_map_tools服务...")
        self.get_num_waypoint_client = rospy.ServiceProxy(
            '/waterplus/get_num_waypoint', 
            GetNumOfWaypoints
        )
        self.get_waypoint_index_client = rospy.ServiceProxy(
            '/waterplus/get_waypoint_index', 
            GetWaypointByIndex
        )
        self.get_waypoint_name_client = rospy.ServiceProxy(
            '/waterplus/get_waypoint_name', 
            GetWaypointByName
        )
        rospy.wait_for_service('/waterplus/get_num_waypoint', timeout=10.0)
        rospy.wait_for_service('/waterplus/get_waypoint_index', timeout=10.0)
        rospy.wait_for_service('/waterplus/get_waypoint_name', timeout=10.0)
        rospy.loginfo("waterplus_map_tools服务连接成功")
    
    def get_num_waypoints(self):
        try:
            response = self.get_num_waypoint_client()
            rospy.loginfo("获取航点数量: %d", response.num)
            return response.num
        except rospy.ServiceException as e:
            rospy.logerr("获取航点数量服务调用失败: %s", e)
            return -1
    
    def get_waypoint_by_index(self, index):
        try:
            response = self.get_waypoint_index_client(index)
            rospy.loginfo("通过索引获取航点: 索引=%d, 名称=%s", index, response.name)
            return response.name, response.pose
        except rospy.ServiceException as e:
            rospy.logerr("通过索引获取航点服务调用失败: %s", e)
            return None, None
    
    def get_waypoint_by_name(self, name):
        try:
            response = self.get_waypoint_name_client(name)
            rospy.loginfo("通过名称获取航点: 名称=%s", response.name)
            return response.name, response.pose
        except rospy.ServiceException as e:
            rospy.logerr("通过名称获取航点服务调用失败: %s", e)
            return None, None
    
    def get_all_waypoints(self):
        waypoints = []
        num_waypoints = self.get_num_waypoints()
        action_class_map = {
            'A1': MoveitAction1,
            'A2': MoveitAction2,
            'Home': MoveitActionHome,
            'None': MoveitActionNone
        }
        
        if num_waypoints > 0:
            for i in range(num_waypoints):
                name, pose = self.get_waypoint_by_index(i)
                if name is not None:
                    if name in action_class_map:
                        action_instance = action_class_map[name](pose)
                        waypoints.append(action_instance)
                        rospy.loginfo("""航点 %d: 名称=%s, 已创建%s实例, 位置=(%.2f, %.2f, %.2f)""" % 
                                        (i, name, action_class_map[name].__name__,
                                        pose.position.x, pose.position.y, pose.position.z))
                    else:
                        rospy.logwarn("航点 %d: 名称=%s, 没有对应的动作类，跳过", i, name)
        rospy.loginfo("共获取 %d 个航点动作实例", len(waypoints))
        return waypoints
    

def main():
    """
    主函数，示例用法
    """
    # 创建任务处理器实例
    handler = MissionHandler()
    
    # 获取航点数量
    num_waypoints = handler.get_num_waypoints()
    rospy.loginfo("系统中共有 %d 个航点", num_waypoints)
    
    # 测试新的 get_all_waypoints 方法
    rospy.loginfo("=== 测试 get_all_waypoints 方法 ===")
    waypoint_actions = handler.get_all_waypoints()
    
    while True:
        # 演示如何使用获取的动作实例
        for i, action_instance in enumerate(waypoint_actions):
            rospy.loginfo("动作实例 %d: 类型=%s", i, type(action_instance).__name__)

            # 可选：发送导航目标（在实际使用中可能需要）
            rospy.loginfo("发送导航目标...")
            success = action_instance.send_navigation_goal()
            if success:
                rospy.loginfo("导航目标发送成功")
            else:
                rospy.logwarn("导航目标发送失败")
            action_instance.action()

            rospy.loginfo("---")
    
    rospy.loginfo("=== 测试完成 ===")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
