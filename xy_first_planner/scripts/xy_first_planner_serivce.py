#!/usr/bin/env python3
"""
XYFirst局部路径规划器服务 - 完整完善版
功能：
1. 修复了 last_control_time 报错
2. 实现了基于角度阈值(70度)的全向/巡航模式切换
3. 实现了 Yaw 的双 PID 策略 (巡航/对齐)
4. 修复了 TF 变换时间戳问题
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger, TriggerResponse
from xy_first_planner.srv import MoveToPose, MoveToPoseResponse

class XYFirstPlannerService:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('xy_first_planner_service')
        
        # 参数配置
        self.load_parameters()
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布器 - 控制指令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 服务
        self.move_to_pose_service = rospy.Service(
            '/xy_first_planner/move_to_pose', 
            MoveToPose, 
            self.handle_move_to_pose
        )
        
        self.stop_service = rospy.Service(
            '/xy_first_planner/stop',
            Trigger,
            self.handle_stop
        )
        
        # 状态变量
        self.current_target = None
        self.is_moving = False
        self.xy_goal_reached = False
        self.yaw_goal_reached = False

        # 【修复 1】初始化控制时间，防止 'object has no attribute' 报错
        self.last_control_time = rospy.Time.now()
        
        rospy.loginfo("XYFirstPlannerService 初始化完成")
        
    def load_parameters(self):
        """加载参数配置"""
        # 目标容差
        self.xy_goal_tolerance = rospy.get_param('~xy_goal_tolerance', 0.05)
        self.yaw_goal_tolerance = rospy.get_param('~yaw_goal_tolerance', 0.05)
        
        # 最大速度限制
        self.max_vel_x = rospy.get_param('~max_vel_x', 0.3)
        self.max_vel_y = rospy.get_param('~max_vel_y', 0.3)
        self.max_vel_theta = rospy.get_param('~max_vel_theta', 1.0)

        self.min_vel_theta = rospy.get_param('~min_vel_theta', 0.15)
        
        # PID控制器参数 - X方向
        self.kp_x = rospy.get_param('~kp_x', 1.0)
        self.ki_x = rospy.get_param('~ki_x', 0.0)
        self.kd_x = rospy.get_param('~kd_x', 0.0)
        
        # PID控制器参数 - Y方向
        self.kp_y = rospy.get_param('~kp_y', 1.0)
        self.ki_y = rospy.get_param('~ki_y', 0.0)
        self.kd_y = rospy.get_param('~kd_y', 0.0)
        
        # PID控制器参数 - 偏航角 (基础，可能会被双PID覆盖)
        self.kp_yaw = rospy.get_param('~kp_yaw', 4.0)
        self.ki_yaw = rospy.get_param('~ki_yaw', 0.0)
        self.kd_yaw = rospy.get_param('~kd_yaw', 0.0)
        
        # 【新增】PID控制器参数 - Yaw 双PID策略
        # 巡航模式参数 (用于运动中，平稳防撞)
        self.kp_yaw_cruise = rospy.get_param('~kp_yaw_cruise', 0.8)
        self.ki_yaw_cruise = rospy.get_param('~ki_yaw_cruise', 0.0)
        self.kd_yaw_cruise = rospy.get_param('~kd_yaw_cruise', 1.0)
        
        # 对齐模式参数 (用于到达后，快速精准)
        self.kp_yaw_align = rospy.get_param('~kp_yaw_align', 3.0)
        self.ki_yaw_align = rospy.get_param('~ki_yaw_align', 0.1)
        self.kd_yaw_align = rospy.get_param('~kd_yaw_align', 0.5)
        
        # 积分项限幅
        self.integral_limit_x = rospy.get_param('~integral_limit_x', 0.5)
        self.integral_limit_y = rospy.get_param('~integral_limit_y', 0.5)
        self.integral_limit_yaw = rospy.get_param('~integral_limit_yaw', 0.5)
        
        # 前瞻距离
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 0.5)
        
        # 控制频率
        self.control_rate = rospy.get_param('~control_rate', 10.0)
        
        # PID状态变量初始化
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_yaw = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0
        
        rospy.loginfo("参数配置:")
        rospy.loginfo(f"  xy_goal_tolerance: {self.xy_goal_tolerance} m")
        rospy.loginfo(f"  yaw_goal_tolerance: {self.yaw_goal_tolerance} rad")
        rospy.loginfo(f"  max_vel_x: {self.max_vel_x} m/s")
        rospy.loginfo(f"  max_vel_y: {self.max_vel_y} m/s")
        rospy.loginfo(f"  max_vel_theta: {self.max_vel_theta} rad/s")
        rospy.loginfo(f"  PID X: kp={self.kp_x}, ki={self.ki_x}, kd={self.kd_x}")
        rospy.loginfo(f"  PID Y: kp={self.kp_y}, ki={self.ki_y}, kd={self.kd_y}")
        rospy.loginfo(f"  PID Yaw Cruise: kp={self.kp_yaw_cruise}, ki={self.ki_yaw_cruise}, kd={self.kd_yaw_cruise}")
        rospy.loginfo(f"  PID Yaw Align: kp={self.kp_yaw_align}, ki={self.ki_yaw_align}, kd={self.kd_yaw_align}")
        rospy.loginfo(f"  lookahead_distance: {self.lookahead_distance} m")
        
    def get_yaw_from_pose(self, pose):
        """从位姿中提取偏航角"""
        from tf.transformations import euler_from_quaternion
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw
    
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def transform_pose_to_robot_frame(self, pose_map):
        try:
            # 【修复】将目标位姿的时间戳改为“最新可用时间”
            # 避免因为目标点时间戳太旧（比如是1秒前发的）而导致的转换失败
            target_pose = PoseStamped()
            target_pose.header = pose_map.header
            target_pose.header.stamp = rospy.Time(0) # 获取最新可用的变换，而不是特定时间点
            target_pose.pose = pose_map.pose

            # 检查转换是否可行
            if not self.tf_buffer.can_transform("base_footprint", target_pose.header.frame_id, rospy.Time(0), rospy.Duration(0.5)):
                return None
            
            # 执行转换
            pose_robot = self.tf_buffer.transform(target_pose, "base_footprint")
            return pose_robot
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"坐标变换失败: {e}")
            return None

    def compute_control_commands(self, target_robot_frame):
        """计算控制指令 - 完善版策略（双PID + 角度切换模式）"""
        cmd_vel = Twist()
        
        if target_robot_frame is None:
            return cmd_vel
        
        # --- 1. 基础时间与状态计算 ---
        current_time = rospy.Time.now()
        dt = (current_time - self.last_control_time).to_sec()
        self.last_control_time = current_time
        
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.control_rate

        # 提取目标信息 (机器人坐标系)
        target_x = target_robot_frame.pose.position.x
        target_y = target_robot_frame.pose.position.y
        target_yaw = self.get_yaw_from_pose(target_robot_frame.pose)
        
        # 计算目标点相对于车头的角度
        angle_to_target = math.atan2(target_y, target_x)
        abs_angle = abs(angle_to_target)
        
        # 计算直线距离用于到达判定
        xy_distance = math.sqrt(target_x**2 + target_y**2)
        
        # --- 2. 到达状态判定 ---
        self.xy_goal_reached = xy_distance <= self.xy_goal_tolerance
        self.yaw_goal_reached = abs(target_yaw) <= self.yaw_goal_tolerance
        
        rospy.logdebug_throttle(1.0, 
            f"Target: ({target_x:.2f}, {target_y:.2f}), Angle: {math.degrees(abs_angle):.1f}deg")

        # --- 3. 策略选择与 PID 计算 ---
        
        # === 3.1 选择 Yaw PID 参数 ===
        if not self.xy_goal_reached:
            # 阶段 A：移动中 - 使用巡航参数 (平稳)
            kp = self.kp_yaw_cruise
            ki = self.ki_yaw_cruise
            kd = self.kd_yaw_cruise
        else:
            # 阶段 B：对齐中 - 使用对齐参数 (快速)
            kp = self.kp_yaw_align
            ki = self.ki_yaw_align
            kd = self.kd_yaw_align
            # 切换到对齐模式时，清空旧的运动积分，避免超调
            self.integral_yaw = 0.0

        # === 3.2 X 轴 PID ===
        error_x = target_x
        if xy_distance > self.lookahead_distance:
            error_x = self.lookahead_distance * math.cos(angle_to_target)
        
        self.integral_x += error_x * dt
        self.integral_x = max(-self.integral_limit_x, min(self.integral_limit_x, self.integral_x))
        derivative_x = (error_x - self.prev_error_x) / dt
        vel_x = self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * derivative_x
        self.prev_error_x = error_x
        
        # === 3.3 Y 轴 PID ===
        error_y = target_y
        if xy_distance > self.lookahead_distance:
            error_y = self.lookahead_distance * math.sin(angle_to_target)
            
        self.integral_y += error_y * dt
        self.integral_y = max(-self.integral_limit_y, min(self.integral_limit_y, self.integral_y))
        derivative_y = (error_y - self.prev_error_y) / dt
        vel_y = self.kp_y * error_y + self.ki_y * self.integral_y + self.kd_y * derivative_y
        self.prev_error_y = error_y
        
        # === 3.4 Yaw PID (使用动态选择的 kp, ki, kd) ===
        error_yaw = target_yaw
        # 积分分离：误差大时不累积积分
        if abs(error_yaw) < 0.5:
            self.integral_yaw += error_yaw * dt
        else:
            self.integral_yaw = 0.0
            
        self.integral_yaw = max(-self.integral_limit_yaw, min(self.integral_limit_yaw, self.integral_yaw))
        derivative_yaw = (error_yaw - self.prev_error_yaw) / dt
        vel_theta = (kp * error_yaw) + (ki * self.integral_yaw) + (kd * derivative_yaw)
        self.prev_error_yaw = error_yaw

        # --- 4. 策略融合：输出速度指令 ---
        
        if not self.xy_goal_reached:
            # --- 阶段一：移动控制 (带智能 Z 轴控制) ---
            
            # 【关键修改】：设定角度误差阈值
            # 如果目标角度偏差大于 60 度，视为大转向，禁止旋转，只走 XY
            # 如果偏差小于 60 度，视为需要动态调整，允许旋转
            LARGE_YAW_ERROR_THRESHOLD = math.radians(60.0) 
            
            if abs(error_yaw) > LARGE_YAW_ERROR_THRESHOLD:
                # 情况 A：需要大角度掉头 (如 0 -> 180)
                # 强制禁止旋转，先去位置
                vel_theta = 0.0
                rospy.logdebug_throttle(1.0, "Move Phase: Large Heading Error, Ignore Z")
            else:
                # 情况 B：角度偏差小
                # 允许旋转，动态修正姿态
                vel_theta = max(-self.max_vel_theta, min(self.max_vel_theta, vel_theta))
                rospy.logdebug_throttle(1.0, "Move Phase: Small Heading Error, Adjusting Z")

            # 侧向移动触发角度阈值 (70度)
            LATERAL_MOVE_THRESHOLD = math.radians(70.0) 

            if abs_angle > LATERAL_MOVE_THRESHOLD:
                # === 模式 A：全向机动 ===
                cmd_vel.linear.x = max(-self.max_vel_x, min(self.max_vel_x, vel_x))
                cmd_vel.linear.y = max(-self.max_vel_y, min(self.max_vel_y, vel_y))
                cmd_vel.angular.z = vel_theta # 使用上面计算的 vel_theta (可能是 0)
                
                rospy.logdebug_throttle(2.0, "Mode: HOLONOMIC")
                
            else:
                # === 模式 B：巡航约束 ===
                # 目标在前方，禁止横向移动
                cmd_vel.linear.x = max(-self.max_vel_x, min(self.max_vel_x, vel_x))
                cmd_vel.linear.y = 0.0
                cmd_vel.angular.z = vel_theta # 使用上面计算的 vel_theta
                
                rospy.logdebug_throttle(2.0, "Mode: CONSTRAINED")

        elif not self.yaw_goal_reached:
            # --- 阶段二：原地对齐 ---
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = max(-self.max_vel_theta, min(self.max_vel_theta, vel_theta))
            # (这里不需要加死区补偿了，因为已经在之前算 vel_theta 时处理过，或者在这里加也可以，看上一段代码)
            
            
        else:
            # --- 状态 3：停止 ---
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            # 清空积分
            self.integral_x = 0.0
            self.integral_y = 0.0
            self.integral_yaw = 0.0

        if abs(cmd_vel.angular.z) > 0.0 and abs(cmd_vel.angular.z) < self.min_vel_theta:
            cmd_vel.angular.z = math.copysign(self.min_vel_theta, cmd_vel.angular.z)
            rospy.loginfo_throttle(1.0, "Applying Deadband Compensation for Z rotation")

        
        return cmd_vel

    def move_to_target(self, target_pose_map):
        """
        移动到目标点的主循环
        
        Args:
            target_pose_map: map坐标系下的目标位姿
            
        Returns:
            bool: 是否成功到达目标
        """
        rate = rospy.Rate(self.control_rate)
        self.is_moving = True
        self.xy_goal_reached = False
        self.yaw_goal_reached = False
        
        # 重置 PID 状态
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_yaw = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0
        
        # 【修复 2】重置本次移动的起始时间
        self.last_control_time = rospy.Time.now()

        # 记录目标点
        rospy.loginfo(f"开始移动到目标: ({target_pose_map.pose.position.x:.3f}, "
                     f"{target_pose_map.pose.position.y:.3f})")
        
        start_time = rospy.Time.now()
        timeout = rospy.Duration(60.0)  # 60秒超时
        
        while not rospy.is_shutdown() and self.is_moving:
            # 检查超时
            if rospy.Time.now() - start_time > timeout:
                rospy.logwarn("移动超时")
                break
            
            try:
                # 将目标点转换到机器人坐标系
                target_robot_frame = self.transform_pose_to_robot_frame(target_pose_map)
                
                if target_robot_frame is None:
                    rospy.logwarn_throttle(1.0, "无法获取目标点在机器人坐标系中的位置")
                    rospy.sleep(0.1)
                    continue
                
                # 计算控制指令
                cmd_vel = self.compute_control_commands(target_robot_frame)
                
                # 发布控制指令
                self.cmd_vel_pub.publish(cmd_vel)
                
                # 检查是否到达目标
                if self.xy_goal_reached and self.yaw_goal_reached:
                    rospy.loginfo("成功到达目标点")
                    # 发送停止指令
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    return True
                
            except Exception as e:
                rospy.logerr(f"控制循环错误: {e}")
                break
            
            rate.sleep()
        
        # 停止机器人
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.is_moving = False
        
        return False
    
    def handle_move_to_pose(self, request):
        """处理移动到目标点的服务请求"""
        response = MoveToPoseResponse()
        
        if self.is_moving:
            response.success = False
            rospy.logwarn("规划器正在执行其他任务")
            return response
        
        # 设置目标
        self.current_target = request.target_pose
        
        # 执行移动
        success = self.move_to_target(request.target_pose)
        
        # 重置状态
        self.is_moving = False
        self.current_target = None
        
        response.success = success
        
        if success:
            rospy.loginfo("移动成功完成")
        else:
            rospy.logwarn("移动失败或被中断")
            
        return response
    
    def handle_stop(self, request):
        """处理停止运动服务请求"""
        response = TriggerResponse()
        
        if self.is_moving:
            self.is_moving = False
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            response.success = True
            response.message = "运动已停止"
            rospy.loginfo(response.message)
        else:
            response.success = False
            response.message = "规划器当前未运动"
            
        return response
    
    def run(self):
        """运行主循环"""
        rospy.loginfo("XYFirstPlannerService 运行中...")
        rospy.spin()

def main():
    try:
        planner_service = XYFirstPlannerService()
        planner_service.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {e}")

if __name__ == '__main__':
    main()
