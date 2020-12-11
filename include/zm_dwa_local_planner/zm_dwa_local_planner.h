#ifndef ZM_DWA_LOCAL_PLANNER_H_
#define ZM_DWA_LOCAL_PLANNER_H_

#include <vector>
#include <Eigen/Core>
#include <cmath>
#include <queue>
#include <angles/angles.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <zm_dwa_local_planner/zmDWALocalPlannerConfig.h>

#include <base_local_planner/map_grid_visualizer.h>
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/goal_functions.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace zm_dwa_local_planner
{
    class zmDWALocalPlanner
    {
        public:
           zmDWALocalPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

           void reconfigure(zmDWALocalPlannerConfig &cfg);

           bool checkTrajectory(const Eigen::Vector3f pos, const Eigen::Vector3f vel, const Eigen::Vector3f vel_samples);

           base_local_planner::Trajectory findBestPath(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities);

           void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan, const std::vector<geometry_msgs::Point>& footprint_spec);

           double getSimPeriod();

           bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);
           
           bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        private:
           double stop_time_buffer_;
           double path_distance_bias_, goal_distance_bias_, occdist_scale_;
           double sim_period_;
           double forward_point_distance_;
           double cheat_factor_;

           std::vector<geometry_msgs::PoseStamped> global_plan_;
           std::string frame_id_;
           
           bool publish_cost_grid_pc_;
           bool publish_traj_pc_;

           Eigen::Vector3f vsamples_;
           
           boost::mutex configuration_mutex_;

           ros::Publisher traj_cloud_pub_;

           base_local_planner::LocalPlannerUtil *planner_util_;
           base_local_planner::Trajectory result_traj_;
           base_local_planner::MapGridVisualizer map_viz_;

           // see constructor body for explanations
           base_local_planner::SimpleTrajectoryGenerator generator_;
           base_local_planner::OscillationCostFunction oscillation_costs_;
           base_local_planner::ObstacleCostFunction obstacle_costs_;
           base_local_planner::MapGridCostFunction path_costs_;
           base_local_planner::MapGridCostFunction goal_costs_;
           base_local_planner::MapGridCostFunction goal_front_costs_;
           base_local_planner::MapGridCostFunction alignment_costs_;
           base_local_planner::TwirlingCostFunction twirling_costs_;
           base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
    };
};

#endif