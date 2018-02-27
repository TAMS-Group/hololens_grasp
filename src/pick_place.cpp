/*
  plan place
  execute place
  error handling (no pick when object is attached...)

*/

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_broadcaster.h>

ros::Publisher *planned_joint_states_publisher_ptr;
ros::Publisher *plan_success_publisher_ptr;

std::vector<moveit_msgs::RobotTrajectory> current_trajectories;

std::string current_object;

std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction>>
    pick_action_client;
std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction>>
    place_action_client;

moveit::planning_interface::MoveGroupInterface* arm;
moveit::planning_interface::MoveGroupInterface* gripper;

moveit::planning_interface::PlanningSceneInterface* psi;

bool top;

void spawnObject(std::string name, float x, float y, float radius,
                 float height) {
  moveit_msgs::CollisionObject object;

  object.header.frame_id = "table_top";
  object.id = name;

  // Type and dimensions
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.push_back(height);
  primitive.dimensions.push_back(radius);

  // Pose
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = height / 2;

  // Add object to scene
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);

  object.operation = object.ADD;

  psi->applyCollisionObject(object);
}

void publishPlannedTrajectory(
    std::vector<moveit_msgs::RobotTrajectory> trajectories) {
  //while (ros::ok() && !current_trajectories.empty()) {
    for (int i = 0; i < trajectories.size(); i++) {
      for (int j = 0; j < trajectories[i].joint_trajectory.points.size(); j++) {
        sensor_msgs::JointState joint_state;
        for (int k = 0; k < trajectories[i].joint_trajectory.joint_names.size();
             k++) {
          joint_state.name.push_back(
              trajectories[i].joint_trajectory.joint_names[k]);
          joint_state.position.push_back(
              trajectories[i].joint_trajectory.points[j].positions[k]);
        }
        planned_joint_states_publisher_ptr->publish(joint_state);
        if (j > 0)
          ros::Duration(
              trajectories[i].joint_trajectory.points[j].time_from_start -
              trajectories[i].joint_trajectory.points[j - 1].time_from_start)
              .sleep();
      }
    }
  //}
}

void jointValuesToJointTrajectory(
    std::map<std::string, double> target_values, ros::Duration duration,
    trajectory_msgs::JointTrajectory &grasp_pose) {
  grasp_pose.joint_names.reserve(target_values.size());
  grasp_pose.points.resize(1);
  grasp_pose.points[0].positions.reserve(target_values.size());

  for (std::map<std::string, double>::iterator it = target_values.begin();
       it != target_values.end(); ++it) {
    grasp_pose.joint_names.push_back(it->first);
    grasp_pose.points[0].positions.push_back(it->second);
  }
  grasp_pose.points[0].time_from_start = duration;
}

moveit_msgs::Grasp generateGrasp(geometry_msgs::PointStamped msg) {

  std::map<std::string, moveit_msgs::CollisionObject> objects = psi->getObjects(std::vector<std::string>{msg.header.frame_id});

  moveit_msgs::Grasp grasp;
  grasp.id = "grasp";

  jointValuesToJointTrajectory(gripper->getNamedTargetValues("open"),
                               ros::Duration(1.0), grasp.pre_grasp_posture);
  jointValuesToJointTrajectory(gripper->getNamedTargetValues("closed"),
                               ros::Duration(2.0), grasp.grasp_posture);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = msg.header.frame_id;

  // If grasp point is on the top of the cylinder, grasp from above
  if (msg.point.z <
      (objects.at(msg.header.frame_id).primitives[0].dimensions[0] / 2) -
          0.01) {
    top = false;
    float w = atan2(msg.point.y, msg.point.x) - atan2(0, 1);
    pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, w + M_PI);
    pose.pose.position.x = msg.point.x;
    pose.pose.position.y = msg.point.y;
    pose.pose.position.z = msg.point.z;
  } else {
    top = true;
    pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, M_PI / 2, 0.0);
    pose.pose.position.z =
        (objects.at(msg.header.frame_id).primitives[0].dimensions[0]) / 2;
  }
  grasp.grasp_pose = pose;

  grasp.pre_grasp_approach.min_distance = 0.08;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasp.pre_grasp_approach.direction.header.frame_id = "s_model_tool0";
  grasp.pre_grasp_approach.direction.vector.x = 1.0;

  grasp.post_grasp_retreat.min_distance = 0.08;
  grasp.post_grasp_retreat.desired_distance = 0.1;
  grasp.post_grasp_retreat.direction.header.frame_id = arm->getPlanningFrame();
  grasp.post_grasp_retreat.direction.vector.z = 1.0;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(1.0);
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.pose.orientation, quat);

/*
  for (int i = 0; i < 2; i++) {
    transform.setOrigin(tf::Vector3(pose.pose.position.x + 0.1,
                                    pose.pose.position.y,
                                    pose.pose.position.z + 0.1));
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "table_top", "grasp_point"));
    rate.sleep();
  }
*/
  return grasp;
}

void planPickCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  ROS_INFO("Planning pick");

  moveit_msgs::PickupGoal goal;

  goal.target_name = msg->header.frame_id;
  goal.group_name = "arm";
  goal.end_effector = "gripper";
  goal.support_surface_name = "table_top";
  goal.possible_grasps.push_back(generateGrasp(*msg));
  goal.allowed_planning_time = 30;
  goal.allow_gripper_support_collision = true;
  goal.planner_id = "RRTConnectkConfigDefault";

  goal.planning_options.plan_only = true;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  // Send goal
  pick_action_client->sendGoal(goal);
  pick_action_client->waitForResult();
  moveit_msgs::PickupResult result = *(pick_action_client->getResult());

  std_msgs::Bool success;

  current_trajectories.clear();
  current_object = "";

  if (result.error_code.val == 1) {
    ROS_INFO("Plan found");
    current_object = msg->header.frame_id;
    success.data = true;
    for (int i = 0; i < result.trajectory_stages.size(); i++)
      current_trajectories.push_back(result.trajectory_stages[i]);
    publishPlannedTrajectory(result.trajectory_stages);
  } else {
    ROS_INFO("No plan found");
    ROS_ERROR_STREAM(result.error_code);
    success.data = false;
  }
  // Publish bool to signal unity if the planning was successfull or not
  plan_success_publisher_ptr->publish(success);
}

void executePickCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Execute pick");

  if (current_trajectories.size() > 0) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // pre grasp pose
    plan.trajectory_ = current_trajectories[0];
    if (!arm->execute(plan)) {
      ROS_ERROR("Movement to pre grasp pose failed");
      return;
    }
    // open gripper
    plan.trajectory_ = current_trajectories[1];
    if (!arm->execute(plan)) {
      ROS_ERROR("Open gripper failed");
      return;
    }
    // cartesion movement to object
    plan.trajectory_ = current_trajectories[2];
    if (!arm->execute(plan)) {
      ROS_ERROR("Movement to grasp pose failed");
      return;
    }
    // close gripper
    plan.trajectory_ = current_trajectories[3];
    if (!arm->execute(plan)) {
      ROS_ERROR("Close gripper failed");
      return;
    }

    // Attach Object
    std::map<std::string, moveit_msgs::CollisionObject> objects = psi->getObjects(std::vector<std::string>{current_object});
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "s_model_tool0";
    attached_object.object = objects.at(current_object);
    attached_object.touch_links = { "s_model_finger_1_link_0",
        "s_model_finger_1_link_1", "s_model_finger_1_link_2",
        "s_model_finger_1_link_3", "s_model_finger_2_link_0",
        "s_model_finger_2_link_1", "s_model_finger_2_link_2",
        "s_model_finger_2_link_3", "s_model_finger_middle_link_0",
        "s_model_finger_middle_link_1", "s_model_finger_middle_link_2",
        "s_model_finger_middle_link_3", "s_model_palm", "s_model_tool0"};
    psi->applyAttachedCollisionObject(attached_object);

    // retreat
    plan.trajectory_ = current_trajectories[4];
    if (!arm->execute(plan)) {
      ROS_ERROR("Retreat failed");
      return;
    }
  }

  // delete planned trajectory
  current_trajectories.clear();
}

void openGripperCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Open gripper");

  // Detach object
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.object.operation = attached_object.object.REMOVE;
  psi->applyAttachedCollisionObject(attached_object);

  moveit_msgs::CollisionObject object;
  object.id = current_object;
  object.operation = object.REMOVE;
  psi->applyCollisionObject(object);

  current_object = "";

  gripper->setNamedTarget("open");
  if (!gripper->move())
    ROS_ERROR("Open gripper failed");
}

moveit_msgs::PlaceLocation generatePlaceLocation(geometry_msgs::PointStamped msg) {
  moveit_msgs::PlaceLocation place_location;

  place_location.id = "place_location";

  jointValuesToJointTrajectory(gripper->getNamedTargetValues("open"), ros::Duration(1.0), place_location.post_place_posture);


  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects = psi->getAttachedObjects(std::vector<std::string>{current_object});

  // Pose for the object frame
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "table_top";
  pose.pose.orientation.w = 1;
  pose.pose.position.x = msg.point.x;
  pose.pose.position.y = msg.point.y;
  pose.pose.position.z = (objects.at(current_object).object.primitives[0].dimensions[0])/2 + 0.01;
  place_location.place_pose = pose;

  place_location.pre_place_approach.min_distance = 0.02;
  place_location.pre_place_approach.desired_distance = 0.1;
  place_location.pre_place_approach.direction.header.frame_id = "s_model_tool0";

  // TODO top grasp vs side grasp
  if (top)
    place_location.pre_place_approach.direction.vector.x = 1.0;
  else
    place_location.pre_place_approach.direction.vector.z = -1.0;

  place_location.post_place_retreat.min_distance = 0.02;
  place_location.post_place_retreat.desired_distance = 0.1;
  place_location.post_place_retreat.direction.header.frame_id = "table_top";
  place_location.post_place_retreat.direction.vector.z = 1.0;


  return place_location;
}

void planPlaceCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  ROS_INFO("Planning place");

  moveit_msgs::PlaceGoal goal;

  goal.group_name = "arm";
  goal.attached_object_name = current_object;
//  goal.end_effector = "gripper";
  goal.support_surface_name = "table_top";
  goal.place_locations.push_back(generatePlaceLocation(*msg));
  goal.allowed_planning_time = 30;
  goal.allow_gripper_support_collision = true;
  goal.planner_id = "RRTConnectkConfigDefault";

  //goal.allowed_touch_objects.push_back(current_object);

  goal.planning_options.plan_only = true;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  // Send goal
  place_action_client->sendGoal(goal);
  place_action_client->waitForResult();
  moveit_msgs::PlaceResult result = *(place_action_client->getResult());

  std_msgs::Bool success;
  current_trajectories.clear();

  if (result.error_code.val == 1) {
    ROS_INFO("Plan found");
    success.data = true;
    for (int i = 0; i < result.trajectory_stages.size(); i++)
      current_trajectories.push_back(result.trajectory_stages[i]);
    publishPlannedTrajectory(result.trajectory_stages);
  } else {
    ROS_INFO("No plan found");
    ROS_ERROR_STREAM(result.error_code);
    success.data = false;
  }
  // Publish bool to signal unity if the planning was successfull or not
  plan_success_publisher_ptr->publish(success);

}

void executePlaceCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Execute place");

  if (current_trajectories.size() > 0) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // pre place pose
    plan.trajectory_ = current_trajectories[0];
    if (!arm->execute(plan)) {
      ROS_ERROR("Movement to pre place pose failed");
      return;
    }
    // cartesian movement to place pose
    plan.trajectory_ = current_trajectories[1];
    if (!arm->execute(plan)) {
      ROS_ERROR("Cartesian movement failed");
      return;
    }
    // Open gripper
    plan.trajectory_ = current_trajectories[2];
    if (!arm->execute(plan)) {
      ROS_ERROR("Open gripepr failed");
      return;
    }

    // Detach attached object
  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects = psi->getAttachedObjects(std::vector<std::string>{current_object});
    moveit_msgs::AttachedCollisionObject attached_object = objects.at(current_object);
    moveit_msgs::CollisionObject object = attached_object.object;

    attached_object.object.operation = attached_object.object.REMOVE;
    psi->applyAttachedCollisionObject(attached_object);
  
    // Apply Collision object
    object.operation = object.ADD;
    psi->applyCollisionObject(object);
  
    current_object = "";

    // retreat
    plan.trajectory_ = current_trajectories[3];
    if (!arm->execute(plan)) {
      ROS_ERROR("Retreat failed");
      return;
    }
  }

  // delete planned trajectory
  current_trajectories.clear();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "HololensPickPlace");
  ros::AsyncSpinner spinner(5);
  spinner.start();
  ros::NodeHandle node_handle;


  // Move group interfaces
  arm = new moveit::planning_interface::MoveGroupInterface("arm");
  gripper = new moveit::planning_interface::MoveGroupInterface(
      arm->getRobotModel()->getEndEffectors()[0]->getName());

  // Planning scene interface
  psi = new moveit::planning_interface::PlanningSceneInterface();

  // Action clients
  pick_action_client.reset(
      new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(
          node_handle, "pickup", false));
  pick_action_client->waitForServer();

  place_action_client.reset(
      new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(
          node_handle, "place", false));
  place_action_client->waitForServer();

  // Subscriber and publisher
  ros::Subscriber plan_pick_subscriber =
      node_handle.subscribe("hololens_plan_pick", 1, planPickCallback);
  ros::Subscriber execute_pick_subscriber =
      node_handle.subscribe("hololens_execute_pick", 1, executePickCallback);
  ros::Subscriber open_gripper_subscriber =
      node_handle.subscribe("hololens_open_gripper", 1, openGripperCallback);
  ros::Subscriber plan_place_subscriber =
      node_handle.subscribe("hololens_plan_place", 1, planPlaceCallback);
  ros::Subscriber execute_place_subscriber =
      node_handle.subscribe("hololens_execute_place", 1, executePlaceCallback);

  planned_joint_states_publisher_ptr =
      new ros::Publisher(node_handle.advertise<sensor_msgs::JointState>(
          "planned_joint_states", 1));
  plan_success_publisher_ptr = new ros::Publisher(
      node_handle.advertise<std_msgs::Bool>("planned_successful", 1));

  spawnObject("object1", 0, 0, 0.0375, 0.258);

  ros::waitForShutdown();
  return 0;
}
