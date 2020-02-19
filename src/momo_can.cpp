#include <ros/ros.h>

// Moveit! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Actionlib headers
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>
#include <play_motion_msgs/PlayMotionGoal.h>

// tf
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// messages
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Services
// #include <lasr_object_detection_yolo/YoloDetection.h>
#include "lasr_pcl/TransformCloud.h"
#include "lasr_pcl/PlaneFromCloud.h"
#include "lasr_pcl/CylinderFromCloud.h"

// Other
// #include <lasr_pcl/pcl_utilities.h>
#include <math.h>
#include <cstdlib>

static const std::string ARM_PLANNING_GROUP = "arm";
static const std::string ARM_TORSO_PLANNING_GROUP = "arm_torso";
static const std::string GRIPPER_PLANNING_GROUP = "gripper";

typedef struct Vector3 {
  double x, y,z;
} Vector3;

using PlayMotionClient = actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction>;

bool doPlayMotion(std::string motion, PlayMotionClient &client) {
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  client.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  play_motion_msgs::PlayMotionGoal goal;
  goal.motion_name = motion;
  goal.skip_planning = true;
  client.sendGoal(goal);
  ROS_INFO("Play motion goal sent!");

  //wait for the action to return
  bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    return true;
  }
  else {    
    ROS_INFO("Action did not finish before the time out.");
    return false;
  }
}

void openGripper(trajectory_msgs::JointTrajectory& posture) {
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture) {
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

bool pickup(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose grasp_goal, Vector3 approach, Vector3 retreat) {
  // Grasp object
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Set up grasping pose
  grasps[0].grasp_pose.header.frame_id = "base_footprint";
  grasps[0].grasp_pose.pose = grasp_goal;

  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
  grasps[0].pre_grasp_approach.direction.vector.x = approach.x;
  grasps[0].pre_grasp_approach.direction.vector.y = approach.y;
  grasps[0].pre_grasp_approach.direction.vector.z = approach.z;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.9;

  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasps[0].post_grasp_retreat.direction.vector.x = retreat.x;
  grasps[0].post_grasp_retreat.direction.vector.y = retreat.y;
  grasps[0].post_grasp_retreat.direction.vector.z = retreat.z;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.2;

  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);
  move_group.setSupportSurfaceName("table");

  // Attempt pick
  if (move_group.pick("can", grasps) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_INFO("Found pickup plan! Executing..");
    return true;
  }
  else {
    ROS_INFO("Could not find pickup plan :( Failed..");
    return false;
  }
}

bool place(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose place_goal) {
  // Place object
  std::vector<moveit_msgs::PlaceLocation> places;
  places.resize(1);

  // Setting up place pose
  places[0].place_pose.header.frame_id = "base_footprint";
  places[0].place_pose.pose = place_goal;

  /* Defined with respect to frame_id */
  places[0].pre_place_approach.direction.header.frame_id = "base_footprint";
  places[0].pre_place_approach.direction.vector.z = -1.0;
  places[0].pre_place_approach.min_distance = 0.01;
  places[0].pre_place_approach.desired_distance = 0.1;

  /* Defined with respect to frame_id */
  places[0].post_place_retreat.direction.header.frame_id = "base_footprint";
  places[0].post_place_retreat.direction.vector.y = 1.0;
  places[0].post_place_retreat.min_distance = 0.01;
  places[0].post_place_retreat.desired_distance = 0.1;

  openGripper(places[0].post_place_posture);
  move_group.setSupportSurfaceName("back");

  // move_group.place("can", place_location);
  if (move_group.place("can", places) == moveit::planning_interface::MoveItErrorCode:: SUCCESS) {
    ROS_INFO("Found place plan! Executing..");
    return true;
  }
  else {
    ROS_INFO("Could not find place plan :( Failed..");
    return false;
  }
}

bool jointPlanning(moveit::planning_interface::MoveGroupInterface& move_group, ros::NodeHandle& nh, std::string parameter) {
  // Plan using Joint Values
  std::vector<std::string> joint_names;
  joint_names = move_group.getJoints();

  // Get the joint values
  std::map<std::string, double> joint_values;
  nh.getParam(parameter, joint_values);
  for (int i=0; i < joint_names.size(); i++) {
    ROS_INFO_STREAM("Joint " << i << " is " << joint_values[joint_names[i]]);
    move_group.setJointValueTarget(joint_names[i], joint_values[joint_names[i]]);
  }

  // Setup for planning
  move_group.setStartStateToCurrentState();
  move_group.setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.setPlanningTime(10.0);
  bool success = bool(move_group.plan(my_plan));

  if ( !success ) {
    throw std::runtime_error("No joint plan found");
    return false;
  }
  ROS_INFO("Found plan! Executing Joint Space goal..");
  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e = move_group.move();
  if (!bool(e)) {
    throw std::runtime_error("Error executing joint plan");
    return false;
  }
  else {
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    // ROS_INFO_STREAM(move_group.getCurrentPose());
    return true;
  }
}

void addCollisionObject(std::string id, double x_dimension, double y_dimension, double z_dimension, geometry_msgs::Pose object_pose, 
                        moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, bool add_safety_margin = false, double safety_margin = 0.0) {
  
  // Check if safety margin needed
  if (!add_safety_margin)
    safety_margin = 0.0;
  
  // Create the collision object
  moveit_msgs::CollisionObject object;
  object.header.frame_id = "base_footprint";
  object.id = id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = x_dimension + safety_margin;
  primitive.dimensions[1] = y_dimension + safety_margin;
  primitive.dimensions[2] = z_dimension;

  geometry_msgs::Pose box_pose;
  box_pose = object_pose;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(box_pose);
  object.operation = object.ADD;

  planning_scene_interface.applyCollisionObject(object);
}

void setMarker(std::string frame, geometry_msgs::Pose pose, ros::Publisher& publisher) {
  // Visualise pose
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = 0;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose = pose;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(20);
  publisher.publish(marker);
  ROS_INFO("Published marker!");
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "grasp_can");

  // ROS stuff
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // set execution montioring to false
  node_handle.setParam("/move_group/trajectory_execution/execution_duration_monitoring", false);
  bool check;
  node_handle.getParam("/move_group/trajectory_execution/execution_duration_monitoring", check);
  ROS_INFO_STREAM("Checking if its been disabled!: " << check << " is it?");

  // play mootion client
  PlayMotionClient play_motion_client("play_motion", true);
  bool look_down = doPlayMotion("look_down", play_motion_client);

  // Marker publisher for rviz visualisation
  ros::Publisher end_effector_pose_pub = node_handle.advertise<visualization_msgs::Marker>("end_effector_pose", 0, true);

  // Moveit stuff
  moveit::planning_interface::MoveGroupInterface move_group(ARM_PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);

  // Print Debug info
  ROS_INFO_NAMED("test", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("test", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Get a pointcloud msg to do the processing on
  boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedEdge;
  sensor_msgs::PointCloud2 cloud_msg;
  sharedEdge = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/xtion/depth_registered/points", ros::Duration(10));
  if(sharedEdge != NULL){
    cloud_msg = *sharedEdge;
  }
  ROS_INFO("Got cloud msg, frame is: ");
  ROS_INFO_STREAM(cloud_msg.header.frame_id);

  // Call service to transform cloud from camera frame to base_footprint
  ros::service::waitForService("transform_cloud");
  ros::ServiceClient transform_cloud_client = node_handle.serviceClient<lasr_pcl::TransformCloud>("transform_cloud");
  lasr_pcl::TransformCloud transform_srv;
  transform_srv.request.cloud = cloud_msg;
  transform_srv.request.target_frame = "base_footprint";
  if (transform_cloud_client.call(transform_srv)) {
    ROS_INFO_STREAM("Transformed cloud msg, frame is: ");
    ROS_INFO_STREAM(transform_srv.response.transformed_cloud.header.frame_id);
  }
  else {
    ROS_ERROR("Failed to call service transform_cloud");
  }
  

  // Call PlaneFromCloud service to get the plane information
  ros::service::waitForService("plane_from_cloud");
  ros::ServiceClient plane_from_cloud_client = node_handle.serviceClient<lasr_pcl::PlaneFromCloud>("plane_from_cloud");
  lasr_pcl::PlaneFromCloud plane_srv;
  plane_srv.request.cloud = transform_srv.response.transformed_cloud;
  plane_srv.request.z_filter_min = 0.6;
  plane_srv.request.z_filter_max = 2.0;
  plane_srv.request.x_filter_min = 0.0;
  plane_srv.request.x_filter_max = 1.25;
  if (plane_from_cloud_client.call(plane_srv)) {
    ROS_INFO_STREAM("Gotten plane information!");
    ROS_INFO_STREAM(plane_srv.response.plane_center);
    ROS_INFO_STREAM(plane_srv.response.plane_height);
    ROS_INFO_STREAM(plane_srv.response.plane_width);
  }
  else {
    ROS_ERROR("Failed to call service plane_from_cloud");
  }

  // Call CylinderFromCloud service to get the cylinder information
  ros::service::waitForService("cylinder_from_cloud");
  ros::ServiceClient cylinder_from_cloud_client = node_handle.serviceClient<lasr_pcl::CylinderFromCloud>("cylinder_from_cloud");
  lasr_pcl::CylinderFromCloud cylinder_srv;
  cylinder_srv.request.cloud = transform_srv.response.transformed_cloud;
  cylinder_srv.request.z_filter_min = plane_srv.response.plane_center.pose.position.z+0.03;
  cylinder_srv.request.z_filter_max = 2.0;
  cylinder_srv.request.x_filter_min = 0.0;
  cylinder_srv.request.x_filter_max = 1.25;
  if (cylinder_from_cloud_client.call(cylinder_srv)) {
    ROS_INFO_STREAM("Gotten cylinder information!");
    ROS_INFO_STREAM(cylinder_srv.response.cylinder_center);
    ROS_INFO_STREAM(cylinder_srv.response.cylinder_height);
    ROS_INFO_STREAM(cylinder_srv.response.cylinder_width);
  }
  else {
    ROS_ERROR("Failed to call service cylinder_from_cloud");
  }

  // DECIDE ON PICKUP APPROACH
  float distance_between_end_effector_and_gripper = 0.12;
  geometry_msgs::Pose grasp_goal;
  Vector3 approach, retreat;
  if (cylinder_srv.response.cylinder_height > cylinder_srv.response.cylinder_width) { // standing can
    // Grasp pose
    grasp_goal = cylinder_srv.response.cylinder_center.pose;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( M_PI/2, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians) - Horizontal Gripper
    grasp_goal.orientation = tf2::toMsg(myQuaternion);
    grasp_goal.position.x = grasp_goal.position.x - (distance_between_end_effector_and_gripper + (cylinder_srv.response.cylinder_thickness/2));

    // Approaching Grasp direction
    approach.x = 1.0; approach.y = 0.0; approach.z = 0.0;

    // Retreating from Grasp direction
    retreat.x = 0.0; retreat.y = 0.0; retreat.z = 1.0;
  }
  else if (cylinder_srv.response.cylinder_width > cylinder_srv.response.cylinder_height) { // not standing
    // Grasp pose
    grasp_goal = cylinder_srv.response.cylinder_center.pose;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, M_PI/2, 0);
    grasp_goal.orientation = tf2::toMsg(myQuaternion);
    grasp_goal.position.z = grasp_goal.position.z + (0.16 + (cylinder_srv.response.cylinder_height));

    // Approaching Grasp direction
    approach.x = 0.0; approach.y = 0.0; approach.z = -1.0;

    // Retreating from GRasp direction
    retreat.x = 0.0; retreat.y = 0.0; retreat.z = 1.0;
  }

  // Define the table as a huge box from floor to surface, adding some safety
  double table_z = plane_srv.response.plane_center.pose.position.z + 0.02;
  plane_srv.response.plane_center.pose.position.z = plane_srv.response.plane_center.pose.position.z/2;
  addCollisionObject("table", plane_srv.response.plane_height, plane_srv.response.plane_width, table_z, plane_srv.response.plane_center.pose, 
                    planning_scene_interface, true, 0.1);
  
  addCollisionObject("can", cylinder_srv.response.cylinder_thickness, cylinder_srv.response.cylinder_width, cylinder_srv.response.cylinder_height,
                    cylinder_srv.response.cylinder_center.pose, planning_scene_interface);


  setMarker("base_footprint", grasp_goal, end_effector_pose_pub);

  // Setting an attempt maximum for pickup and place planning attempts
  int num_of_attempts = 10;

  // Call our pickup function to grasp the object
  for (int i=0; i <= num_of_attempts; i++) {
    bool success = pickup(move_group, grasp_goal, approach, retreat);

    if (success)
      break;
    else
      i++;
  }

  /************************************************************************************/
  geometry_msgs::Pose back_pose, base_pose;
  back_pose.position.x = -0.1;
  back_pose.position.y = 0.0;
  back_pose.position.z = 1.19;
  back_pose.orientation.w = 1.0;

  base_pose.position.x = 0.0;
  base_pose.position.y = 0.175;
  base_pose.position.z = 0.3;
  base_pose.orientation.w = 1.0;

  addCollisionObject("back", 0.3, 0.3, 0.001, back_pose, planning_scene_interface);
  addCollisionObject("base", 0.4, 0.12, 0.001, base_pose, planning_scene_interface);
  /********************************************************************************/

  // Call our joint planning function
  for (int i=0; i <= num_of_attempts; i++) {
    bool success = jointPlanning(move_group, node_handle, "TIAGo_back_LHS2");

    if (success)
      break;
    else
      i++;
  }

  /************************************************************************************/
  ros::Duration(2.0).sleep(); // sleep for 5 seconds
  ROS_INFO("Getting ready to place it on back");

  geometry_msgs::Pose place_goal;
  place_goal.position.x = -0.1;
  place_goal.position.y = 0.0;
  place_goal.position.z = 1.28 + cylinder_srv.response.cylinder_height/2;
  place_goal.orientation.x = 0.011;
  place_goal.orientation.y = -0.0363;
  place_goal.orientation.z = -0.8309;
  place_goal.orientation.w = 0.555;

  setMarker("base_footprint", place_goal, end_effector_pose_pub);

  // Call our place function to place the object
  for (int i=0; i <= num_of_attempts; i++) {
    bool success = place(move_group, place_goal);

    if (success)
      break;
    else
      i++;
  }

  /*************************************************************************************/
  // doPlayMotion("home", play_motion_client); // home pose

  spinner.stop();

  ROS_INFO("Removing all collision objects from the world!");
  std::vector<std::string> objects_ids;
  objects_ids.push_back("table");
  objects_ids.push_back("can");
  objects_ids.push_back("back");
  objects_ids.push_back("base");
  planning_scene_interface.removeCollisionObjects(objects_ids);

  return 0;
}