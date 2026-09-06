#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

class MetroSceneBuilder:
    def __init__(self):
        rospy.init_node('metro_scene_builder')
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
    def create_pillar(self, name, x, y):
        """创建柱子"""
        # 柱子尺寸
        pillar_radius = 0.04  # 柱子半径
        pillar_height = 0.8   # 柱子高度
        
        pillar_sdf = f"""<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{name}">
            <static>true</static>
            <link name="link">
              <pose>0 0 0 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>{pillar_radius}</radius>
                    <length>{pillar_height}</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>{pillar_radius}</radius>
                    <length>{pillar_height}</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>0.5 0.5 0.5 1</ambient>
                  <diffuse>0.7 0.7 0.7 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
        
        # 柱子中心位置在高度一半处
        pose = Pose(position=Point(x, y, pillar_height/2), orientation=Quaternion(0,0,0,1))
        self.spawn_model(model_name=name, model_xml=pillar_sdf, 
                        initial_pose=pose, reference_frame='world')
        print(f"Created pillar: {name} at ({x}, {y}), radius={pillar_radius}m, height={pillar_height}m")
    
    def create_rail(self, name, x, y, z, length=20.0):
        """创建铁轨"""
        # 铁轨尺寸
        rail_width = 0.06   # 铁轨宽度
        rail_height = 0.04  # 铁轨高度
        
        rail_sdf = f"""<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{name}">
            <static>true</static>
            <link name="link">
              <pose>0 0 0 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{length} {rail_width} {rail_height}</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{length} {rail_width} {rail_height}</size>
                  </box>
                </geometry>
                <material>
                  <ambient>0.2 0.2 0.2 1</ambient>
                  <diffuse>0.3 0.3 0.3 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
        
        # 铁轨中心位置
        pose = Pose(position=Point(x, y, z), orientation=Quaternion(0,0,0,1))
        self.spawn_model(model_name=name, model_xml=rail_sdf, 
                        initial_pose=pose, reference_frame='world')
        print(f"Created rail: {name} at ({x}, {y}, {z}), length={length}m")
    
    def create_metro_car(self, name, x, y, z, car_height=0.6):
        """创建地铁车厢"""
        # 统一使用灰色
        color = (0.5, 0.5, 0.5)
        
        # 车厢尺寸
        car_length = 2.5    # 车厢长度
        car_width = 0.9     # 车厢宽度
        
        # 简单的灰色车厢，没有车窗（取消蓝色车窗）
        metro_sdf = f"""<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{name}">
            <link name="link">
              <pose>0 0 0 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{car_length} {car_width} {car_height}</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{car_length} {car_width} {car_height}</size>
                  </box>
                </geometry>
                <material>
                  <ambient>{color[0]} {color[1]} {color[2]} 1</ambient>
                  <diffuse>{color[0]+0.1} {color[1]+0.1} {color[2]+0.1} 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
        
        # 车厢中心位置
        pose = Pose(position=Point(x, y, z), orientation=Quaternion(0,0,0,1))
        self.spawn_model(model_name=name, model_xml=metro_sdf, 
                        initial_pose=pose, reference_frame='world')
        print(f"Created metro car: {name} at ({x}, {y}, {z}), height={car_height}m")
    
    def create_wall(self, name, x, y, z, length, width, height, orientation=0):
        """创建围墙"""
        # 墙的尺寸：length是长度，width是厚度，height是高度
        # orientation: 0表示沿X轴，1表示沿Y轴
        
        # 机器人高度假设为1.5米，一半是0.75米，这里设置墙高0.7米
        wall_height = height  # 0.7米，不超过机器人一半高度
        
        # 半透明材质设置 (RGBA)
        # A=0.5表示半透明
        wall_color = (0.8, 0.8, 0.8, 0.5)  # 浅灰色半透明
        
        # 根据方向确定尺寸
        if orientation == 0:  # 沿X轴
            size_x = length
            size_y = width
        else:  # 沿Y轴
            size_x = width
            size_y = length
        
        wall_sdf = f"""<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{name}">
            <static>true</static>
            <link name="link">
              <pose>0 0 0 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{size_x} {size_y} {wall_height}</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{size_x} {size_y} {wall_height}</size>
                  </box>
                </geometry>
                <material>
                  <ambient>{wall_color[0]} {wall_color[1]} {wall_color[2]} {wall_color[3]}</ambient>
                  <diffuse>{wall_color[0]} {wall_color[1]} {wall_color[2]} {wall_color[3]}</diffuse>
                  <specular>0.1 0.1 0.1 0.5</specular>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
        
        # 墙的中心位置
        pose = Pose(position=Point(x, y, z), orientation=Quaternion(0,0,0,1))
        self.spawn_model(model_name=name, model_xml=wall_sdf, 
                        initial_pose=pose, reference_frame='world')
        print(f"Created wall: {name} at ({x}, {y}, {z}), size={size_x}x{size_y}x{wall_height}m")
    
    def build_scene(self):
        """构建完整场景"""
        print("Building metro scene with walls...")
        
        # 关键参数 - 保持原来的参数不变
        car_length = 2.5  # 车厢长度
        
        # 单侧柱子数量 - 在每两个柱子之间插入一个柱子，所以数量翻倍
        # 原来每侧6个柱子，插入后变成11个（6个原始柱子 + 5个插入柱子）
        original_pillar_count_per_side = 6
        new_pillar_count_per_side = original_pillar_count_per_side + (original_pillar_count_per_side - 1)  # 6 + 5 = 11
        
        # 柱子间距（恢复为原来的1.25米，但插入柱子后实际间距减半）
        original_pillar_spacing = 1.25
        new_pillar_spacing = original_pillar_spacing / 2  # 0.625米
        
        # 计算铁轨长度（柱子覆盖的范围） - 保持不变
        rail_length = (original_pillar_count_per_side - 1) * original_pillar_spacing  # 5 * 1.25 = 6.25米
        
        # 铁轨间距（两侧间隔）
        rail_spacing = 0.92  # 2倍机器人宽度
        
        # 高度参数（所有高度都是模型中心的高度）
        # 柱子：中心高度 = 高度/2
        pillar_height = 0.8
        pillar_center_z = pillar_height / 2  # 0.4米
        
        # 铁轨：放在柱子顶部，中心高度 = 柱子顶部 + 铁轨高度/2
        rail_height = 0.04
        rail_center_z = pillar_height + rail_height/2  # 0.82米
        
        # 车厢：放在铁轨上，中心高度 = 铁轨顶部 + 车厢高度/2
        # 增加车厢高度到0.6米
        car_height = 0.6
        car_center_z = pillar_height + rail_height + car_height/2  # 1.04米 + 0.1 = 1.14米
        
        # ================== 计算围墙参数 ==================
        # 墙到整个模型边界的间距
        wall_margin = 2.0  # 2米间距
        
        # 计算围墙的边界
        # X方向：铁轨两端各留2米
        # Y方向：铁轨两侧各留2米
        wall_x_min = -rail_length/2 - wall_margin
        wall_x_max = rail_length/2 + wall_margin
        wall_y_min = -rail_spacing/2 - wall_margin
        wall_y_max = rail_spacing/2 + wall_margin
        
        # 围墙尺寸计算
        wall_length_x = (wall_x_max - wall_x_min)  # X方向围墙长度
        wall_length_y = (wall_y_max - wall_y_min)  # Y方向围墙长度
        wall_thickness = 0.1  # 墙厚度
        wall_height = 0.7     # 墙高度，不超过机器人高度的一半(1.5/2=0.75)
        wall_center_z = wall_height / 2  # 墙中心高度
        
        print(f"\n围墙参数:")
        print(f"围墙边界: X=[{wall_x_min:.2f}, {wall_x_max:.2f}], Y=[{wall_y_min:.2f}, {wall_y_max:.2f}]")
        print(f"围墙尺寸: X方向={wall_length_x:.2f}m, Y方向={wall_length_y:.2f}m")
        print(f"墙厚度: {wall_thickness}m, 墙高度: {wall_height}m")
        
        # ================== 创建柱子 ==================
        # 创建柱子（每侧11个，在每两个原始柱子之间插入一个柱子）
        start_x = -rail_length/2  # 从铁轨起点开始
        
        print(f"\n柱子配置:")
        print(f"原始柱子数量: {original_pillar_count_per_side}个/侧")
        print(f"插入后柱子数量: {new_pillar_count_per_side}个/侧")
        print(f"原始柱子间距: {original_pillar_spacing}m")
        print(f"新柱子间距: {new_pillar_spacing}m")
        print(f"铁轨长度: {rail_length}m")
        print(f"柱子总数量: {new_pillar_count_per_side*2}个")
        
        # 左侧柱子（使用新的间距0.625米，数量11个）
        for i in range(new_pillar_count_per_side):
            x_pos = start_x + i * new_pillar_spacing
            y_pos = rail_spacing/2  # 左侧
            # 标记哪些是原始柱子，哪些是插入的柱子
            if i % 2 == 0:
                pillar_type = "原始"
                pillar_name = f"pillar_left_original_{i//2}"
            else:
                pillar_type = "插入"
                pillar_name = f"pillar_left_inserted_{i//2}"
            self.create_pillar(pillar_name, x_pos, y_pos)
        
        # 右侧柱子（使用新的间距0.625米，数量11个）
        for i in range(new_pillar_count_per_side):
            x_pos = start_x + i * new_pillar_spacing
            y_pos = -rail_spacing/2  # 右侧
            # 标记哪些是原始柱子，哪些是插入的柱子
            if i % 2 == 0:
                pillar_type = "原始"
                pillar_name = f"pillar_right_original_{i//2}"
            else:
                pillar_type = "插入"
                pillar_name = f"pillar_right_inserted_{i//2}"
            self.create_pillar(pillar_name, x_pos, y_pos)
        
        # ================== 创建铁轨 ==================
        # 创建铁轨（放在柱子顶部）
        # 左侧铁轨
        self.create_rail("left_rail", 0, rail_spacing/2, rail_center_z, rail_length)
        # 右侧铁轨
        self.create_rail("right_rail", 0, -rail_spacing/2, rail_center_z, rail_length)
        
        # ================== 创建围墙 ==================
        # 创建四面围墙，半透明材质
        print(f"\n创建围墙:")
        
        # 北墙 (Y正方向)
        north_wall_x = (wall_x_min + wall_x_max) / 2
        north_wall_y = wall_y_max - wall_thickness/2
        self.create_wall("north_wall", north_wall_x, north_wall_y, wall_center_z, 
                        wall_length_x, wall_thickness, wall_height, orientation=0)
        
        # 南墙 (Y负方向)
        south_wall_x = (wall_x_min + wall_x_max) / 2
        south_wall_y = wall_y_min + wall_thickness/2
        self.create_wall("south_wall", south_wall_x, south_wall_y, wall_center_z,
                        wall_length_x, wall_thickness, wall_height, orientation=0)
        
        # 西墙 (X负方向)
        west_wall_x = wall_x_min + wall_thickness/2
        west_wall_y = (wall_y_min + wall_y_max) / 2
        self.create_wall("west_wall", west_wall_x, west_wall_y, wall_center_z,
                        wall_length_y, wall_thickness, wall_height, orientation=1)
        
        # 东墙 (X正方向)
        east_wall_x = wall_x_max - wall_thickness/2
        east_wall_y = (wall_y_min + wall_y_max) / 2
        self.create_wall("east_wall", east_wall_x, east_wall_y, wall_center_z,
                        wall_length_y, wall_thickness, wall_height, orientation=1)
        
        # ================== 创建地铁车厢 ==================
        # 创建地铁车厢（2节，放在铁轨上，位置保持不变）
        # 计算车厢总长度和间隙
        total_car_length = 2 * car_length  # 2.5 * 2 = 5.0米
        desired_gap = 0.75  # 期望的车厢间隙
        
        # 计算总占用长度
        total_occupied = total_car_length + desired_gap  # 5.0 + 0.75 = 5.75米
        
        # 计算两端的空闲长度（应该相等）
        end_gap = (rail_length - total_occupied) / 2  # (6.25 - 5.75) / 2 = 0.25米
        
        print(f"\n车厢位置计算:")
        print(f"铁轨长度: {rail_length}m")
        print(f"车厢总长度: {total_car_length}m (2 × {car_length}m)")
        print(f"期望车厢间隙: {desired_gap}m")
        print(f"总占用长度: {total_occupied}m")
        print(f"两端空闲长度: {end_gap}m (应该相等)")
        
        # 计算第一车厢的中心位置
        # 铁轨左端 + 左端空闲 + 车厢长度的一半
        car1_x = start_x + end_gap + car_length/2
        
        # 计算第二车厢的中心位置
        # 第一车厢中心 + 车厢长度 + 车厢间隙
        car2_x = car1_x + car_length + desired_gap
        
        # 验证第二车厢的位置
        # 第二车厢右端应该距离铁轨右端 end_gap 米
        car2_right_end = car2_x + car_length/2
        rail_right_end = -start_x
        
        # 计算车厢边缘之间的距离
        center_distance = abs(car2_x - car1_x)
        edge_distance = center_distance - car_length  # 车厢边缘之间的距离
        
        print(f"\n车厢位置:")
        print(f"第一车厢中心: X = {car1_x:.2f}m, 范围: [{car1_x-car_length/2:.2f}, {car1_x+car_length/2:.2f}]")
        print(f"第二车厢中心: X = {car2_x:.2f}m, 范围: [{car2_x-car_length/2:.2f}, {car2_x+car_length/2:.2f}]")
        print(f"车厢中心距: {center_distance:.2f}m")
        print(f"车厢边缘距: {edge_distance:.2f}m (应该为{desired_gap}m)")
        print(f"左端空闲: {car1_x-car_length/2 - start_x:.2f}m (应该为{end_gap}m)")
        print(f"右端空闲: {rail_right_end - (car2_x+car_length/2):.2f}m (应该为{end_gap}m)")
        
        # 创建车厢
        self.create_metro_car("metro_car_1", car1_x, 0, car_center_z, car_height)
        self.create_metro_car("metro_car_2", car2_x, 0, car_center_z, car_height)
        
        print("\n=== 场景构建完成 ===")
        print(f"柱子数量: {new_pillar_count_per_side*2}个 (每侧{new_pillar_count_per_side}个)")
        print(f"铁轨长度: {rail_length}m")
        print(f"铁轨间距: {rail_spacing}m (2 × 机器人宽度0.46m)")
        print(f"围墙尺寸: {wall_length_x:.2f}m × {wall_length_y:.2f}m × {wall_height}m")
        print(f"围墙位置: X=[{wall_x_min:.2f}, {wall_x_max:.2f}], Y=[{wall_y_min:.2f}, {wall_y_max:.2f}]")
        print(f"围墙到模型边界间距: {wall_margin}m")
        print(f"\n高度参数:")
        print(f"  柱子中心高度: {pillar_center_z}m (底部在地面)")
        print(f"  铁轨中心高度: {rail_center_z:.3f}m (放在柱子顶部)")
        print(f"  车厢中心高度: {car_center_z:.3f}m (放在铁轨上)")
        print(f"  围墙中心高度: {wall_center_z}m (半透明材质，高度{wall_height}m)")
        print(f"  机器人通过净空: {pillar_height - 0.5:.2f}m")
        print(f"\n围墙特点:")
        print(f"  1. 高度{wall_height}m (不超过机器人高度1.5m的一半)")
        print(f"  2. 半透明材质 (alpha=0.5)")
        print(f"  3. 四周均匀留出{wall_margin}m间距")
        print(f"  4. 围成一个封闭的地铁站区域")

if __name__ == '__main__':
    try:
        builder = MetroSceneBuilder()
        rospy.sleep(2)  # 等待Gazebo启动
        builder.build_scene()
        rospy.loginfo("Metro scene construction completed!")
    except rospy.ROSInterruptException:
        pass