#include "cw1_class.h"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>





cw1::cw1(ros::NodeHandle nh)
  : nh_(nh)
  , arm_group_("panda_arm")  // 在初始化列表中直接指定 group 名
  , hand_group_("hand")      // 同上
{
  // Advertise the three task services
  t1_service_ = nh_.advertiseService("/task1_start", &cw1::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw1::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw1::t3_callback, this);

  // Configure MoveGroup parameters
  arm_group_.setPlanningTime(10.0);   // arm
  hand_group_.setPlanningTime(5.0);   // gripper
  arm_group_.setPoseReferenceFrame("panda_link0"); // 若需要，可在 SRDF 中检查

  ROS_INFO("cw1 class initialised. Services for Task1/2/3 ready.");
}

// Helper function to print PoseStamped
void cw1::printPoseStamped(const geometry_msgs::PoseStamped &pose)
{
  std::cout << "PoseStamped:" << std::endl;
  std::cout << "  Header:" << std::endl;
  std::cout << "    seq: " << pose.header.seq << std::endl;
  std::cout << "    stamp: " << pose.header.stamp << std::endl;
  std::cout << "    frame_id: " << pose.header.frame_id << std::endl;
  std::cout << "  Pose:" << std::endl;
  std::cout << "    Position: ("
            << pose.pose.position.x << ", "
            << pose.pose.position.y << ", "
            << pose.pose.position.z << ")" << std::endl;
  std::cout << "    Orientation: ("
            << pose.pose.orientation.x << ", "
            << pose.pose.orientation.y << ", "
            << pose.pose.orientation.z << ", "
            << pose.pose.orientation.w << ")" << std::endl;
}

// Helper function to print PointStamped
void cw1::printPointStamped(const geometry_msgs::PointStamped &point)
{
  std::cout << "PointStamped:" << std::endl;
  std::cout << "  Header:" << std::endl;
  std::cout << "    seq: " << point.header.seq << std::endl;
  std::cout << "    stamp: " << point.header.stamp << std::endl;
  std::cout << "    frame_id: " << point.header.frame_id << std::endl;
  std::cout << "  Point: ("
            << point.point.x << ", "
            << point.point.y << ", "
            << point.point.z << ")" << std::endl;
}

/**
 * @brief Task1 callback: pick-and-place a single object.
 */
bool cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
                      cw1_world_spawner::Task1Service::Response &response)
{
  ROS_INFO("Task 1 callback triggered");

  // 1) Extract object and basket positions
  geometry_msgs::PoseStamped object_pose = request.object_loc;
  geometry_msgs::PointStamped basket_point = request.goal_loc;
  printPoseStamped(object_pose);
  printPointStamped(basket_point);
  // 2) Define a "pre-grasp" pose above the object
  geometry_msgs::PoseStamped pre_grasp_pose = object_pose;
  pre_grasp_pose.pose.position.z += 0.20; // 20cm above
  pre_grasp_pose.pose.orientation.w = 0; // keep orientation neutral
  pre_grasp_pose.pose.orientation.x = 1;
  pre_grasp_pose.pose.orientation.y = 0;
  pre_grasp_pose.pose.orientation.z = 0;
  // 3) Define a place pose above the basket
  object_pose.pose.orientation.w = 0; // keep orientation neutral
  object_pose.pose.orientation.x = 1;
  object_pose.pose.orientation.y = 0;
  object_pose.pose.orientation.z = 0;
  object_pose.pose.position.z += 0.13;
  // 4) Define a place pose above the basket
  geometry_msgs::PoseStamped place_pose;
  place_pose.header = basket_point.header;
  place_pose.pose.position = basket_point.point;
  place_pose.pose.position.z += 0.25; // 10cm above basket
  place_pose.pose.orientation.x = 1.0; // keep orientation neutral
  place_pose.pose.orientation.y = 0.0; // keep orientation neutral
  place_pose.pose.orientation.z = 0.0; // keep orientation neutral
  place_pose.pose.orientation.w = 0.0; // keep orientation neutral                                 
  ROS_WARN("START..........................................................TASK1");
  // 4) Open the gripper
  if(!moveGripper(gripper_open_))
  {
    ROS_ERROR("Failed to open gripper before pick");
    return false;
  }

  // 5) Move to pre-grasp
  if(!moveArm(pre_grasp_pose.pose))
  {
    ROS_ERROR("Failed to move arm to pre-grasp");
    return false;
  }

  // 6) Descend onto the object
  if(!moveArm(object_pose.pose))
  {
    ROS_ERROR("Failed to move arm to object");
    return false;
  }

  // 7) Close the gripper to grab
  if(!moveGripper(gripper_closed_))
  {
    ROS_ERROR("Failed to close gripper on object");
    return false;
  }

  // 8) Lift back up
  if(!moveArm(pre_grasp_pose.pose))
  {
    ROS_ERROR("Failed to retreat arm to pre-grasp");
    return false;
  }

  // 9) Move arm above the basket
  if(!moveArm(place_pose.pose))
  {
    ROS_ERROR("Failed to move arm to place pose");
    return false;
  }

  // 10) Open gripper to release
  if(!moveGripper(gripper_open_))
  {
    ROS_ERROR("Failed to open gripper at basket");
    return false;
  }

  ROS_INFO("Task 1 completed successfully");
  return true; // response is empty for Task1
}

/**
 * @brief Task2 callback (detect basket colors).
 */
bool cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
                      cw1_world_spawner::Task2Service::Response &response)
{
  ROS_INFO("Task 2 callback triggered");
  // TODO: implement color detection logic (PCL or similar)
  return true;
}

/**
 * @brief Task3 callback (multi-object color matching).
 */
bool cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
                      cw1_world_spawner::Task3Service::Response &response)
{
  ROS_INFO("Task 3 callback triggered");
  // TODO: implement multi-object detection + pick-and-place
  return true;
}
/**
 * @brief Helper function: move the arm to a given pose.
 */
bool cw1::moveArm(const geometry_msgs::Pose target_pose)
{
// setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  std::string planning_frame = arm_group_.getPlanningFrame();
  ROS_INFO("Planning frame: %s", planning_frame.c_str());

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  if (success) {
    arm_group_.move();
    // printf("Press any key to continue...");
    // getchar();
    return true;
  } else {
    return false;
  }
  
}


/**
 * @brief Helper function: move the gripper to a target width
 */
bool cw1::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    hand_group_.move();
    return true;
    // printf("Press any key to continue...");
    // getchar();
  } else {
    return false;
  }
  hand_group_.move();

  return success;
}

void cw1::segColors(PointCPtr &in_cloud_ptr)
{
  std::array<PointCPtr, 3> clouds = {g_cloud_red, g_cloud_blue, g_cloud_purple};
  for (auto& cloud : clouds) {
    cloud->points.clear();
  }

  // Iterate through point cloud and filter by color
  for (const auto &point : in_cloud_ptr->points) {
    std::string detected_color = "unknown";

    float r = static_cast<float>(point.r) / 255.0;
    float g = static_cast<float>(point.g) / 255.0;
    float b = static_cast<float>(point.b) / 255.0;

    Eigen::Vector3f point_rgb(r, g, b);
    
    for (const auto& [color_name, target_rgb] : cw1::color_map) {
      if ((point_rgb - target_rgb).norm() < thresh) {
      detected_color = color_name;
      break;
      }
    }

    if (detected_color != "unknown") {
      switch (detected_color[0]) {
      case 'r':
        g_cloud_red->points.push_back(point);
        break;
      case 'b':
        g_cloud_blue->points.push_back(point);
        break;
      case 'p':
        g_cloud_purple->points.push_back(point);
        break;
      }
    }
  }

  for (auto& cloud : clouds) {
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    pubFilteredPCMsg(g_pub_cloud, *cloud);
  }

  // Publish the segmented box result
  
  ROS_INFO_STREAM("Detected color: " << detected_color);
  ROS_INFO_STREAM("PointCloud representing the " << detected_color << " box component: " << g_cloud_box->size() << " data points.");
}

void cw1::pubFilteredPCMsg (ros::Publisher &pc_pub,PointC &pc)
{
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  return;
}

std::map<std::string, std::vector<PointCPtr>> cw1::segObjects(PointCPtr &in_cloud_ptr, std::string color)
{
    return std::map<std::string, std::vector<PointCPtr>>();
}

void cw1::findObjectPoses(PointCPtr &in_cloud_ptr)
{
}
