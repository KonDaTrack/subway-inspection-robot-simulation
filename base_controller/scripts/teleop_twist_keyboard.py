#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
全向底盘键盘控制
---------------------------
控制键位:
   w : 向前移动
   s : 向后移动
   a : 向左横移
   d : 向右横移
   q : 逆时针旋转
   e : 顺时针旋转
   空格键 : 紧急停止

速度调整:
   + : 增加线性速度
   - : 减少线性速度
   ] : 增加角速度
   [ : 减少角速度

按键时运动，松开时停止
CTRL-C 退出
"""

# 速度设置
LINEAR_SPEED = 0.2  # 线性速度 m/s
ANGULAR_SPEED = 0.5  # 角速度 rad/s
LINEAR_SPEED_STEP = 0.05  # 线性速度调整步长
ANGULAR_SPEED_STEP = 0.1  # 角速度调整步长

# 键位映射: (linear.x, linear.y, angular.z)
# linear.x: 前后方向 (+前, -后)
# linear.y: 左右横移 (+左, -右)  # 修正：根据base_footprint坐标系，左是y正方向
# angular.z: 旋转 (+逆时针, -顺时针)
moveBindings = {
    'w': (LINEAR_SPEED, 0, 0),      # 向前
    's': (-LINEAR_SPEED, 0, 0),     # 向后
    'a': (0, LINEAR_SPEED, 0),      # 向左横移（修正：正y）
    'd': (0, -LINEAR_SPEED, 0),     # 向右横移（修正：负y）
    'q': (0, 0, ANGULAR_SPEED),     # 逆时针旋转
    'e': (0, 0, -ANGULAR_SPEED),    # 顺时针旋转
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('omni_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            
            # 初始化速度为0
            x = 0.0
            y = 0.0
            th = 0.0
            
            # 速度调整逻辑
            speed_changed = False
            if key == '+':
                LINEAR_SPEED += LINEAR_SPEED_STEP
                speed_changed = True
                print(f"线性速度增加至: {LINEAR_SPEED:.2f} m/s")
            elif key == '-':
                if LINEAR_SPEED > LINEAR_SPEED_STEP:
                    LINEAR_SPEED -= LINEAR_SPEED_STEP
                    speed_changed = True
                    print(f"线性速度减少至: {LINEAR_SPEED:.2f} m/s")
                else:
                    print(f"线性速度已达最小值: {LINEAR_SPEED:.2f} m/s")
            elif key == ']':
                ANGULAR_SPEED += ANGULAR_SPEED_STEP
                speed_changed = True
                print(f"角速度增加至: {ANGULAR_SPEED:.2f} rad/s")
            elif key == '[':
                if ANGULAR_SPEED > ANGULAR_SPEED_STEP:
                    ANGULAR_SPEED -= ANGULAR_SPEED_STEP
                    speed_changed = True
                    print(f"角速度减少至: {ANGULAR_SPEED:.2f} rad/s")
                else:
                    print(f"角速度已达最小值: {ANGULAR_SPEED:.2f} rad/s")
            
            # 如果速度发生变化，更新键位映射
            if speed_changed:
                moveBindings.update({
                    'w': (LINEAR_SPEED, 0, 0),      # 向前
                    's': (-LINEAR_SPEED, 0, 0),     # 向后
                    'a': (0, LINEAR_SPEED, 0),      # 向左横移（修正：正y）
                    'd': (0, -LINEAR_SPEED, 0),     # 向右横移（修正：负y）
                    'q': (0, 0, ANGULAR_SPEED),     # 逆时针旋转
                    'e': (0, 0, -ANGULAR_SPEED),    # 顺时针旋转
                })
                print(f"当前速度 - 线性: {LINEAR_SPEED:.2f} m/s, 角速度: {ANGULAR_SPEED:.2f} rad/s")
            
            if key in moveBindings.keys():
                # 按键时设置对应速度
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
            elif key == ' ':
                # 空格键紧急停止
                x = 0.0
                y = 0.0
                th = 0.0
            elif key == '\x03':
                # CTRL-C 退出
                break
            
            # 发布速度命令
            twist = Twist()
            twist.linear.x = x
            twist.linear.y = y
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th
            pub.publish(twist)
            
    except Exception as e:
        print(e)
    finally:
        # 退出时发送停止命令
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
