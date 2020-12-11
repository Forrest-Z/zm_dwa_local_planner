#ifndef ZM_DWA_LOCAL_PLANNER_ROS_H_
#define ZM_DWA_LOCAL_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <zm_dwa_local_planner/zmDWALocalPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <zm_dwa_local_planner/zm_dwa_local_planner.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

namespace zm_dwa_local_planner
{
    class zmDWALocalPlannerROS : public nav_core::BaseLocalPlanner
    {
        public:
           zmDWALocalPlannerROS();
           ~zmDWALocalPlannerROS();

           void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

           bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
           bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);
           bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
           bool isGoalReached();
           bool isInitialized();

        private:
           void reconfigureCB(zmDWALocalPlannerConfig &config, uint32_t level);
           void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
           void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

           bool setup_;
           bool initialized_;

           std::string odom_topic_;
           
           tf2_ros::Buffer* tf_;

           geometry_msgs::PoseStamped current_pose_;
           
           ros::Publisher g_plan_pub_, l_plan_pub_;

           boost::shared_ptr<zmDWALocalPlanner> dp_; 
           
           costmap_2d::Costmap2DROS* costmap_ros_;

           dynamic_reconfigure::Server<zmDWALocalPlannerConfig> *dsrv_;
           zm_dwa_local_planner::zmDWALocalPlannerConfig default_config_;
           
           base_local_planner::LocalPlannerUtil planner_util_;
           base_local_planner::LatchedStopRotateController latchedStopRotateController_;
           base_local_planner::OdometryHelperRos odom_helper_;
    };
};

#endif