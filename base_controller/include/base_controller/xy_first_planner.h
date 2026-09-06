#ifndef BASE_CONTROLLER_XY_FIRST_PLANNER_H
#define BASE_CONTROLLER_XY_FIRST_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>

namespace base_controller {

class XYFirstPlanner : public nav_core::BaseLocalPlanner {
public:
    XYFirstPlanner();
    ~XYFirstPlanner();

    // BaseLocalPlanner interface implementation
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;

private:
    // ROS components
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    
    // Current plan
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    bool initialized_;
    
    // Parameters
    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;
    double max_vel_x_;
    double max_vel_y_;
    double max_vel_theta_;
    double p_xy_gain_;
    double p_yaw_gain_;
    double lookahead_distance_;
    double i_xy_gain_;
    double i_yaw_gain_;
    double d_yaw_gain_;
    double deadzone_compensation_;
    
    // State tracking
    bool xy_goal_reached_;
    bool yaw_goal_reached_;
    
    // 新的状态跟踪
    bool x_goal_reached_;
    bool y_goal_reached_;
    
    // Helper methods
    bool getRobotPose(geometry_msgs::PoseStamped& robot_pose);
    double distance2D(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);
    double getYawFromPose(const geometry_msgs::Pose& pose);
};

} // namespace base_controller

#endif // BASE_CONTROLLER_XY_FIRST_PLANNER_H