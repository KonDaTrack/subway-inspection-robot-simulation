#include <base_controller/xy_first_planner.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>

namespace base_controller {

XYFirstPlanner::XYFirstPlanner() :
    tf_(nullptr),
    costmap_ros_(nullptr),
    initialized_(false),
    xy_goal_reached_(false),
    yaw_goal_reached_(false),
    x_goal_reached_(false),
    y_goal_reached_(false)
{
}

XYFirstPlanner::~XYFirstPlanner()
{
}

void XYFirstPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_) {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        
        // Initialize node handles
        ros::NodeHandle nh("~/" + name);
        private_nh_ = nh;
        
        // Load parameters with defaults
        private_nh_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
        private_nh_.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.087); // ~5 degrees
        private_nh_.param("max_vel_x", max_vel_x_, 0.5);
        private_nh_.param("max_vel_y", max_vel_y_, 0.5);
        private_nh_.param("max_vel_theta", max_vel_theta_, 1.0);
        private_nh_.param("p_xy_gain", p_xy_gain_, 2.0);
        private_nh_.param("p_yaw_gain", p_yaw_gain_, 2.0);
        private_nh_.param("lookahead_distance", lookahead_distance_, 0.5);
        
        ROS_INFO("XYFirstPlanner initialized with parameters:");
        ROS_INFO("  xy_goal_tolerance: %.3f", xy_goal_tolerance_);
        ROS_INFO("  yaw_goal_tolerance: %.3f", yaw_goal_tolerance_);
        ROS_INFO("  max_vel_x: %.3f", max_vel_x_);
        ROS_INFO("  max_vel_y: %.3f", max_vel_y_);
        ROS_INFO("  max_vel_theta: %.3f", max_vel_theta_);
        ROS_INFO("  p_xy_gain: %.3f", p_xy_gain_);
        ROS_INFO("  p_yaw_gain: %.3f", p_yaw_gain_);
        ROS_INFO("  lookahead_distance: %.3f", lookahead_distance_);
        
        initialized_ = true;
    }
}

bool XYFirstPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_) {
        ROS_ERROR("XYFirstPlanner has not been initialized");
        return false;
    }
    
    global_plan_ = plan;
    xy_goal_reached_ = false;
    yaw_goal_reached_ = false;
    x_goal_reached_ = false;
    y_goal_reached_ = false;
    
    if (global_plan_.empty()) {
        ROS_WARN("Received empty plan");
        return false;
    }
    
    ROS_INFO("XYFirstPlanner: New plan received with %zu poses", global_plan_.size());
    return true;
}

bool XYFirstPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if (!initialized_) {
        ROS_ERROR("XYFirstPlanner has not been initialized");
        return false;
    }
    
    if (global_plan_.empty()) {
        ROS_WARN("No plan available");
        return false;
    }
    
    // Get current robot pose
    geometry_msgs::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
        ROS_ERROR("Failed to get robot pose");
        return false;
    }
    
    // Get target pose (last pose in plan)
    geometry_msgs::PoseStamped target_pose = global_plan_.back();
    
    // Reset cmd_vel
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    // 1. 获取机器人当前偏航角
    double robot_yaw = getYawFromPose(robot_pose.pose);
    
    // 2. 计算全局坐标系下的方向向量
    double global_dx = target_pose.pose.position.x - robot_pose.pose.position.x;
    double global_dy = target_pose.pose.position.y - robot_pose.pose.position.y;
    
    // 3. 将全局坐标系下的向量转换到机器人坐标系
    double robot_dx = cos(robot_yaw) * global_dx + sin(robot_yaw) * global_dy;
    double robot_dy = -sin(robot_yaw) * global_dx + cos(robot_yaw) * global_dy;
    
    // 4. 计算XY距离（在全局坐标系下）
    double xy_distance = sqrt(global_dx*global_dx + global_dy*global_dy);
    
    // 5. 计算偏航角误差
    double target_yaw = getYawFromPose(target_pose.pose);
    double yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);
    
    // 6. 检查X方向是否到达
    if (fabs(robot_dx) <= xy_goal_tolerance_) {
        x_goal_reached_ = true;
    } else {
        x_goal_reached_ = false;
    }
    
    // 7. 检查Y方向是否到达（只有在X到达后才检查）
    if (x_goal_reached_) {
        if (fabs(robot_dy) <= xy_goal_tolerance_) {
            y_goal_reached_ = true;
        } else {
            y_goal_reached_ = false;
        }
    } else {
        y_goal_reached_ = false; // X还没到达，不检查Y
    }
    
    // 8. 检查偏航角是否到达（只有在X和Y都到达后才检查）
    if (x_goal_reached_ && y_goal_reached_) {
        xy_goal_reached_ = true;
        if (fabs(yaw_error) <= yaw_goal_tolerance_) {
            yaw_goal_reached_ = true;
        } else {
            yaw_goal_reached_ = false;
        }
    } else {
        xy_goal_reached_ = false;
        yaw_goal_reached_ = false;
    }
    
    // ========== 新的策略：先X，后Y，最后偏转 ==========
    
    // 阶段1：X方向移动
    if (!x_goal_reached_) {
        // 归一化X方向向量
        double direction_x = 0.0;
        if (fabs(robot_dx) > 0.001) {
            direction_x = (robot_dx > 0) ? 1.0 : -1.0;
        }
        
        // 计算X方向有效距离
        double effective_distance_x = std::min(fabs(robot_dx), lookahead_distance_);
        
        // X方向控制
        double vel_x = p_xy_gain_ * direction_x * effective_distance_x;
        
        // 限制X方向速度
        cmd_vel.linear.x = std::max(-max_vel_x_, std::min(max_vel_x_, vel_x));
        
        ROS_DEBUG_THROTTLE(1.0, "Moving X: robot_dx=%.3f, vx=%.3f", robot_dx, cmd_vel.linear.x);
    }
    // 阶段2：Y方向移动（X已到达）
    else if (!y_goal_reached_) {
        // 归一化Y方向向量
        double direction_y = 0.0;
        if (fabs(robot_dy) > 0.001) {
            direction_y = (robot_dy > 0) ? 1.0 : -1.0;
        }
        
        // 计算Y方向有效距离
        double effective_distance_y = std::min(fabs(robot_dy), lookahead_distance_);
        
        // Y方向控制
        double vel_y = p_xy_gain_ * direction_y * effective_distance_y;
        
        // 限制Y方向速度
        cmd_vel.linear.y = std::max(-max_vel_y_, std::min(max_vel_y_, vel_y));
        
        ROS_DEBUG_THROTTLE(1.0, "Moving Y: robot_dy=%.3f, vy=%.3f", robot_dy, cmd_vel.linear.y);
    }
    // 阶段3：偏航角调整（X和Y都已到达）
    else if (!yaw_goal_reached_) {
        // 偏航角控制
        double vel_theta = p_yaw_gain_ * yaw_error;
        
        // 限制角速度
        cmd_vel.angular.z = std::max(-max_vel_theta_, std::min(max_vel_theta_, vel_theta));
        
        ROS_DEBUG_THROTTLE(1.0, "Adjusting yaw: error=%.3f, vtheta=%.3f", yaw_error, cmd_vel.angular.z);
    }
    else {
        // 所有目标都已到达
        ROS_DEBUG_THROTTLE(1.0, "All goals reached: robot_dx=%.3f, robot_dy=%.3f, yaw_error=%.3f", 
                          robot_dx, robot_dy, yaw_error);
    }
    
    return true;
}

bool XYFirstPlanner::isGoalReached()
{
    if (!initialized_) {
        ROS_ERROR("XYFirstPlanner has not been initialized");
        return false;
    }
    
    // Goal is reached when X, Y and yaw are all within tolerance
    return x_goal_reached_ && y_goal_reached_ && yaw_goal_reached_;
}

bool XYFirstPlanner::getRobotPose(geometry_msgs::PoseStamped& robot_pose)
{
    if (!tf_ || !costmap_ros_) {
        return false;
    }
    
    robot_pose.header.frame_id = costmap_ros_->getGlobalFrameID();
    robot_pose.header.stamp = ros::Time();
    
    // Get robot pose in global frame
    geometry_msgs::PoseStamped robot_pose_local;
    if (!costmap_ros_->getRobotPose(robot_pose_local)) {
        ROS_ERROR("Failed to get robot pose from costmap");
        return false;
    }
    
    try {
        tf_->transform(robot_pose_local, robot_pose, costmap_ros_->getGlobalFrameID());
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Failed to transform robot pose: %s", ex.what());
        return false;
    }
}

double XYFirstPlanner::distance2D(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return sqrt(dx*dx + dy*dy);
}

double XYFirstPlanner::getYawFromPose(const geometry_msgs::Pose& pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

} // namespace base_controller

// Register as a plugin
PLUGINLIB_EXPORT_CLASS(base_controller::XYFirstPlanner, nav_core::BaseLocalPlanner)