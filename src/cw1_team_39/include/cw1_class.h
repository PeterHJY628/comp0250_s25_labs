#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

/* 
 * cw1_class.h
 * 
 * Header file defining the cw1 class, which advertises ROS services for
 * tasks 1, 2, and 3. It uses MoveIt! to plan and execute motions of
 * the "panda_arm" and "hand" groups.
 */

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <vector>

// MoveIt includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

 // PCL specific includes
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/common/centroid.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/voxel_grid.h>
 #include <pcl/filters/passthrough.h>
 #include <pcl/filters/extract_indices.h>
 #include <pcl/features/normal_3d.h>
 #include <pcl/ModelCoefficients.h>
 #include <pcl/sample_consensus/method_types.h>
 #include <pcl/sample_consensus/model_types.h>
 #include <pcl/search/kdtree.h>
 #include <pcl/segmentation/sac_segmentation.h>

// Coursework service definitions
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

class cw1
{
public:
  /**
   * @brief Constructor: sets up the MoveGroupInterfaces and advertises services
   * @param nh ROS NodeHandle
   */
  cw1(ros::NodeHandle nh);

  /**
   * @brief Callback for Task1 (pick & place)
   */
  bool t1_callback(cw1_world_spawner::Task1Service::Request &request,
                   cw1_world_spawner::Task1Service::Response &response);

  /**
   * @brief Callback for Task2 (detect color)
   */
  bool t2_callback(cw1_world_spawner::Task2Service::Request &request,
                   cw1_world_spawner::Task2Service::Response &response);

  /**
   * @brief Callback for Task3 (multi-object pick & place)
   */
  bool t3_callback(cw1_world_spawner::Task3Service::Request &request,
                   cw1_world_spawner::Task3Service::Response &response);

  /**
   * @brief Move the arm to a desired pose (wrapper for MoveGroupInterface).
   * @param target_pose Desired pose for the end-effector
   * @return true if success
   */
  bool moveArm(const geometry_msgs::Pose target_pose);

  /**
   * @brief Move the gripper fingers to a certain width.
   * @param width Distance between fingers [m].
   * @return true if success
   */
  bool moveGripper(float width);

private:
  std::string base_frame_ = "panda_link0";
  ros::NodeHandle nh_;

  // Advertised services for tasks 1,2,3
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // MoveIt planning interfaces
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Some default finger positions for fully open/closed
  double gripper_open_   = 0.04; ///< 4cm open
  double gripper_closed_ = 0.00; ///< fully closed

  ////////////////////////////////////////////////////////////////////////////////
  // PCL object detection properties
  /** \brief Point cloud to hold different colors. */
  PointCPtr g_cloud_red, g_cloud_blue, g_cloud_purple;
  sensor_msgs::PointCloud2 g_cloud_red_msg;
  sensor_msgs::PointCloud2 g_cloud_blue_msg;
  sensor_msgs::PointCloud2 g_cloud_purple_msg;

  std::map<std::string, Eigen::Vector3f> color_map = {
    {"b", Eigen::Vector3f(0.1, 0.1, 0.8)},
    {"r", Eigen::Vector3f(0.8, 0.1, 0.1)},
    {"p", Eigen::Vector3f(0.8, 0.1, 0.8)}}; // Define target RGB values
  float color_thresh_ = 0.1; // Threshold for color detection
  float object_size_ = 0.04; // Size of objects to pick up
  float target_size_ = 0.1; // Size of target object to pick up

  /** \brief target and object pose */
  Eigen::Vector4f target_pose_red;
  Eigen::Vector4f target_pose_blue;
  Eigen::Vector4f target_pose_purple;

  std::list<Eigen::Vector4f> object_poses_red;
  std::list<Eigen::Vector4f> object_poses_blue;
  std::list<Eigen::Vector4f> object_poses_purple;
  
  // Object detection functions
  /**
   * @brief Segment the input point cloud by color, store results in g_cloud_red, g_cloud_blue, g_cloud_purple, and publish
   * @param in_cloud_ptr Input point cloud
   */
  void segColors(PointCPtr &in_cloud_ptr); 

  /**
   * @brief Find the pose of the object in the input point cloud
   * @param in_cloud_ptr Input point cloud
   * @return Map of detected object point clouds
   */
  std::map<std::string, std::vector<PointCPtr>> segObjects(PointCPtr &in_cloud_ptr, std::string color); 

  /**
   * @brief Find the pose of the object in the input point cloud, store them in 
   * object_poses_red, object_poses_blue, object_poses_purple and 
   * target_pose_red, target_pose_blue, target_pose_purple
   * @param in_cloud_ptr Input point cloud
   */
  void findObjectPoses(PointCPtr &in_cloud_ptr);

  /**
   * @brief Check if an object of any color is detected
   */
  std::bool isObjectDetected();

  /**
   * @brief Get the target and object poses for the current task
   * @return Pair of target and object poses
   */
  std::pair<Eigen::Vector4f, Eigen::Vector4f> getTargetAndObject();
};


#endif  // CW1_CLASS_H_
