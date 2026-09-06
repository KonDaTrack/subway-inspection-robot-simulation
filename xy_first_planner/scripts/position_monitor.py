#!/usr/bin/env python3
"""
简化版位置监控脚本 - 用于调试XYFirst路径规划器的PID参数

该脚本监听tf话题中的base_footprint位置，以10Hz频率记录机器人的坐标，
并将数据保存到txt文件中，用于分析PID参数性能。
只记录当前位置，不计算误差。
"""

import rospy
import tf2_ros
import os
from datetime import datetime
from tf.transformations import euler_from_quaternion

class SimplePositionMonitor:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('position_monitor')
        
        # 用户指定的输出文件路径
        # 请在此处直接设置您想要的完整文件路径
        # 例如: "/home/user/position_log.txt" 或 "./position_log.txt"
        self.output_file = "/home/robot/catkin_ws/src/xy_first_planner/scripts/position_log.txt"
        
        # 参数配置
        self.monitor_rate = rospy.get_param('~monitor_rate', 10.0)  # Hz
        self.reference_frame = rospy.get_param('~reference_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')
        
        # 创建输出目录（如果不存在）
        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
            rospy.loginfo(f"创建输出目录: {output_dir}")
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 数据记录变量
        self.position_data = []
        self.start_time = rospy.Time.now()
        
        rospy.loginfo(f"简化版位置监控器初始化完成")
        rospy.loginfo(f"参考坐标系: {self.reference_frame}")
        rospy.loginfo(f"机器人坐标系: {self.robot_frame}")
        rospy.loginfo(f"监控频率: {self.monitor_rate} Hz")
        rospy.loginfo(f"输出文件: {self.output_file}")
        
    def get_robot_position(self):
        """获取机器人在参考坐标系中的位置"""
        try:
            # 获取从参考坐标系到机器人坐标系的变换
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame, 
                self.robot_frame, 
                rospy.Time(0)
            )
            
            # 提取位置和朝向
            position = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 计算偏航角
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            _, _, yaw = euler_from_quaternion(quaternion)
            
            return {
                'x': position.x,
                'y': position.y,
                'z': position.z,
                'yaw': yaw,
                'timestamp': rospy.Time.now(),
                'success': True
            }
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"获取机器人位置失败: {e}")
            return {'success': False}
    
    def record_position(self):
        """记录当前位置到内存"""
        robot_pos = self.get_robot_position()
        
        if not robot_pos['success']:
            return False
            
        # 计算时间戳（相对于开始时间）
        elapsed_time = (robot_pos['timestamp'] - self.start_time).to_sec()
        
        # 创建数据记录
        record = {
            'time': elapsed_time,
            'x': robot_pos['x'],
            'y': robot_pos['y'],
            'z': robot_pos['z'],
            'yaw': robot_pos['yaw']
        }
            
        self.position_data.append(record)
        
        # 定期显示状态
        if len(self.position_data) % 10 == 0:
            rospy.loginfo(f"已记录 {len(self.position_data)} 个位置点, 时间: {elapsed_time:.1f}s")
            rospy.loginfo(f"  当前位置: ({robot_pos['x']:.3f}, {robot_pos['y']:.3f}, {robot_pos['z']:.3f}), "
                         f"偏航角: {robot_pos['yaw']:.3f}rad")
        
        return True
    
    def save_to_file(self):
        """将记录的数据保存到txt文件"""
        if not self.position_data:
            rospy.logwarn("没有数据可保存")
            return False
            
        try:
            with open(self.output_file, 'w') as f:
                # 写入文件头
                f.write("# 机器人位置监控数据 - 简化版（只记录位置）\n")
                f.write(f"# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 参考坐标系: {self.reference_frame}\n")
                f.write(f"# 机器人坐标系: {self.robot_frame}\n")
                f.write(f"# 监控频率: {self.monitor_rate} Hz\n")
                f.write(f"# 总数据点: {len(self.position_data)}\n")
                f.write("#\n")
                f.write("# 列说明:\n")
                f.write("# 1. 时间(s) - 从监控开始经过的时间\n")
                f.write("# 2. X坐标(m)\n")
                f.write("# 3. Y坐标(m)\n")
                f.write("# 4. Z坐标(m)\n")
                f.write("# 5. 偏航角(rad)\n")
                f.write("#\n")
                
                # 写入列标题
                headers = ["time(s)", "x(m)", "y(m)", "z(m)", "yaw(rad)"]
                f.write("\t".join(headers) + "\n")
                
                # 写入数据
                for record in self.position_data:
                    row = [
                        f"{record['time']:.6f}",
                        f"{record['x']:.6f}",
                        f"{record['y']:.6f}",
                        f"{record['z']:.6f}",
                        f"{record['yaw']:.6f}"
                    ]
                    
                    f.write("\t".join(row) + "\n")
            
            rospy.loginfo(f"数据已保存到: {self.output_file}")
            rospy.loginfo(f"共保存 {len(self.position_data)} 个数据点")
            return True
            
        except Exception as e:
            rospy.logerr(f"保存文件失败: {e}")
            return False
    
    def run(self):
        """运行主监控循环"""
        rate = rospy.Rate(self.monitor_rate)
        
        rospy.loginfo("开始位置监控...")
        rospy.loginfo("按Ctrl+C停止监控并保存数据")
        
        try:
            while not rospy.is_shutdown():
                # 记录当前位置
                self.record_position()
                
                # 保持指定频率
                rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("监控被中断")
        except Exception as e:
            rospy.logerr(f"监控过程中发生错误: {e}")
        finally:
            # 保存数据
            rospy.loginfo("正在保存监控数据...")
            self.save_to_file()
            
            # 显示统计信息
            if self.position_data:
                first = self.position_data[0]
                last = self.position_data[-1]
                duration = last['time'] - first['time']
                rospy.loginfo(f"监控时长: {duration:.1f}秒")
                rospy.loginfo(f"平均频率: {len(self.position_data)/duration:.1f} Hz")

def main():
    try:
        monitor = SimplePositionMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {e}")

if __name__ == '__main__':
    main()
