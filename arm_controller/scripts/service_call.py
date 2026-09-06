#!/usr/bin/env python3
import rospy
from arm_controller.srv import MoveArmToPose, MoveArmToPoseRequest

def call_arm_service(pose_name):
    rospy.wait_for_service('/move_arm_to_pose')
    try:
        move_arm = rospy.ServiceProxy('/move_arm_to_pose', MoveArmToPose)
        resp = move_arm(pose_name)
        return resp.success, resp.message
    except rospy.ServiceException as e:
        return False, f"服务调用失败: {e}"

if __name__ == '__main__':
    rospy.init_node('cart_arm_client', anonymous=True)
    # 小车到达目标点后调用
    success, msg = call_arm_service("pose1")
    print(f"调用结果: {success}, 信息: {msg}")