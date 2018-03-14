#include <ros/ros.h>
#include <thread>

#include <geometry_msgs/PointStamped.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <hololens_grasp/State.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

ros::Publisher *planned_joint_states_publisher_ptr;
ros::Publisher *state_publisher_ptr;

std::vector<moveit_msgs::RobotTrajectory> current_trajectories;

std::string current_object;

std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction>>
    pick_action_client;
std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction>>
    place_action_client;

moveit::planning_interface::MoveGroupInterface *arm;
moveit::planning_interface::MoveGroupInterface *gripper;

moveit::planning_interface::PlanningSceneInterface *psi;

bool top;
hololens_grasp::State state;

std::thread *thread;

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

  robot_state::RobotStatePtr robot_state = arm->getCurrentState();
  sensor_msgs::JointState start_state;
  moveit::core::robotStateToJointStateMsg(*robot_state, start_state);

  while (ros::ok() && !current_trajectories.empty()) {
    sensor_msgs::JointState joint_state = start_state;
    for (int i = 0; i < trajectories.size(); i++) {
      for (int j = 0; j < trajectories[i].joint_trajectory.points.size(); j++) {
        for (int k = 0; k < trajectories[i].joint_trajectory.joint_names.size();
             k++) {

          ptrdiff_t index = std::distance(
              joint_state.name.begin(),
              find(joint_state.name.begin(), joint_state.name.end(),
                   trajectories[i].joint_trajectory.joint_names[k]));

          if (index < joint_state.name.size())
            joint_state.position[index] =
                trajectories[i].joint_trajectory.points[j].positions[k];
          else
            ROS_WARN_STREAM("Joint "
                            << trajectories[i].joint_trajectory.joint_names[k]
                            << " not found");
        }
        planned_joint_states_publisher_ptr->publish(joint_state);
        if (j > 0)
          ros::Duration(
              (trajectories[i].joint_trajectory.points[j].time_from_start -
               trajectories[i].joint_trajectory.points[j - 1].time_from_start) *
              0.5)
              .sleep();

      // return if new planning is requested, in this case the current trajectory is empty
      if (current_trajectories.empty())
        return;
      }
    }
  }
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

std::vector<moveit_msgs::Grasp> generateGrasp(geometry_msgs::PointStamped msg) {

  std::map<std::string, moveit_msgs::CollisionObject> objects =
      psi->getObjects(std::vector<std::string>{msg.header.frame_id});

  if (objects.size() == 0) {
    ROS_ERROR_STREAM("Object " << msg.header.frame_id << " not found");
    return std::vector<moveit_msgs::Grasp>{};
  }

  moveit_msgs::Grasp grasp;

  grasp.id = "grasp";

  jointValuesToJointTrajectory(gripper->getNamedTargetValues("open"),
                               ros::Duration(2.0), grasp.pre_grasp_posture);
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

  return std::vector<moveit_msgs::Grasp>{grasp};
}

void planPickCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  ROS_INFO("Planning pick");

  moveit_msgs::PickupGoal goal;

  goal.target_name = msg->header.frame_id;
  goal.group_name = "arm";
  goal.end_effector = "gripper";
  goal.support_surface_name = "table_top";
  goal.allowed_planning_time = 30;
  goal.allow_gripper_support_collision = true;
  goal.planner_id = "RRTstartkConfigDefault";

  goal.possible_grasps = generateGrasp(*msg);
  if (goal.possible_grasps.size() == 0)
    return;

  goal.planning_options.plan_only = true;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  // Send goal
  pick_action_client->sendGoal(goal);
  pick_action_client->waitForResult();
  moveit_msgs::PickupResult result = *(pick_action_client->getResult());

  if (result.error_code.val == 1) {
    ROS_INFO_STREAM("Plan found for object " << msg->header.frame_id);
    current_trajectories.clear();
    current_object = msg->header.frame_id;

    state.val = hololens_grasp::State::SUCCESS;
    state_publisher_ptr->publish(state);
    state.val = hololens_grasp::State::PLANNED_PICK;
    state_publisher_ptr->publish(state);

    if (thread != NULL)
      thread->join();
    for (int i = 0; i < result.trajectory_stages.size(); i++)
      current_trajectories.push_back(result.trajectory_stages[i]);
    thread = new std::thread(
        std::bind(publishPlannedTrajectory, result.trajectory_stages));
  } else {
    ROS_INFO_STREAM("No plan found for object " << msg->header.frame_id);
    ROS_ERROR_STREAM(result.error_code);
    hololens_grasp::State prev_state = state;
    state.val = hololens_grasp::State::PLANNING_FAILED;
    state_publisher_ptr->publish(state);
    state = prev_state;
    state_publisher_ptr->publish(state);
  }
}

void executePickCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Execute pick");

  state.val = hololens_grasp::State::EXECUTING;
  state_publisher_ptr->publish(state);

  if (current_trajectories.size() > 0) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // pre grasp pose
    plan.trajectory_ = current_trajectories[0];
    if (!arm->execute(plan)) {
      ROS_ERROR("Movement to pre grasp pose failed");
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }
    // open gripper
    plan.trajectory_ = current_trajectories[1];
    if (!arm->execute(plan)) {
      ROS_ERROR("Open gripper failed");
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }
    // cartesion movement to object
    plan.trajectory_ = current_trajectories[2];
    if (!arm->execute(plan)) {
      ROS_ERROR("Movement to grasp pose failed");
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }
    // close gripper
    plan.trajectory_ = current_trajectories[3];
    if (!arm->execute(plan)) {
      ROS_ERROR("Close gripper failed");
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }

    // Attach Object
    std::map<std::string, moveit_msgs::CollisionObject> objects =
        psi->getObjects(std::vector<std::string>{current_object});
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "s_model_tool0";
    attached_object.object = objects.at(current_object);
    attached_object.touch_links = {"s_model_finger_1_link_0",
                                   "s_model_finger_1_link_1",
                                   "s_model_finger_1_link_2",
                                   "s_model_finger_1_link_3",
                                   "s_model_finger_2_link_0",
                                   "s_model_finger_2_link_1",
                                   "s_model_finger_2_link_2",
                                   "s_model_finger_2_link_3",
                                   "s_model_finger_middle_link_0",
                                   "s_model_finger_middle_link_1",
                                   "s_model_finger_middle_link_2",
                                   "s_model_finger_middle_link_3",
                                   "s_model_palm",
                                   "s_model_tool0"};
    psi->applyAttachedCollisionObject(attached_object);

    // retreat
    plan.trajectory_ = current_trajectories[4];
    if (!arm->execute(plan)) {
      ROS_ERROR("Retreat failed");
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }
  }

  // delete planned trajectory
  current_trajectories.clear();

  state.val = hololens_grasp::State::HOLD_OBJECT;
  state_publisher_ptr->publish(state);
}

void openGripperCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Open gripper");

  state.val = hololens_grasp::State::EXECUTING;
  state_publisher_ptr->publish(state);

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

  state.val = hololens_grasp::State::IDLE;
  state_publisher_ptr->publish(state);
}

moveit_msgs::PlaceLocation
generatePlaceLocation(geometry_msgs::PointStamped msg) {
  moveit_msgs::PlaceLocation place_location;

  place_location.id = "place_location";

  jointValuesToJointTrajectory(gripper->getNamedTargetValues("open"),
                               ros::Duration(2.0),
                               place_location.post_place_posture);

  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
      psi->getAttachedObjects(std::vector<std::string>{current_object});

  // Pose for the object frame
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "table_top";
  pose.pose.orientation.w = 1;
  pose.pose.position.x = msg.point.x;
  pose.pose.position.y = msg.point.y;
  pose.pose.position.z =
      (objects.at(current_object).object.primitives[0].dimensions[0]) / 2 +
      0.01;
  place_location.place_pose = pose;

  place_location.pre_place_approach.min_distance = 0.02;
  place_location.pre_place_approach.desired_distance = 0.1;
  place_location.pre_place_approach.direction.header.frame_id = "s_model_tool0";

  // Top grasp vs side grasp
  if (top)
    place_location.pre_place_approach.direction.vector.x = 1.0;
  else
    place_location.pre_place_approach.direction.vector.z = -1.0;

  place_location.post_place_retreat.min_distance = 0.02;
  place_location.post_place_retreat.desired_distance = 0.1;
  place_location.post_place_retreat.direction.header.frame_id = "s_model_tool0";
  place_location.post_place_retreat.direction.vector.x = -1.0;

  return place_location;
}

void planPlaceCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  ROS_INFO("Planning place");

  moveit_msgs::PlaceGoal goal;

  goal.group_name = "arm";
  goal.attached_object_name = current_object;
  goal.support_surface_name = "table_top";
  goal.place_locations.push_back(generatePlaceLocation(*msg));
  goal.allowed_planning_time = 30;
  goal.allow_gripper_support_collision = true;
  goal.planner_id = "RRTConnectkConfigDefault";

  goal.planning_options.plan_only = true;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  // Send goal
  place_action_client->sendGoal(goal);
  place_action_client->waitForResult();
  moveit_msgs::PlaceResult result = *(place_action_client->getResult());


  if (result.error_code.val == 1) {
    ROS_INFO("Plan found");
    state.val = hololens_grasp::State::SUCCESS;
    state_publisher_ptr->publish(state);
    state.val = hololens_grasp::State::PLANNED_PLACE;
    state_publisher_ptr->publish(state);

    current_trajectories.clear();
    if (thread != NULL)
      thread->join();
    for (int i = 0; i < result.trajectory_stages.size(); i++)
      current_trajectories.push_back(result.trajectory_stages[i]);
    thread = new std::thread(
        std::bind(publishPlannedTrajectory, result.trajectory_stages));
  } else {
    ROS_INFO("No plan found");
    ROS_ERROR_STREAM(result.error_code);
    hololens_grasp::State prev_state = state;
    state.val = hololens_grasp::State::PLANNING_FAILED;
    state_publisher_ptr->publish(state);
    state = prev_state;
    state_publisher_ptr->publish(state);
  }
}

void executePlaceCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("Execute place");

  state.val = hololens_grasp::State::EXECUTING;
  state_publisher_ptr->publish(state);

  if (current_trajectories.size() > 0) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // pre place pose
    plan.trajectory_ = current_trajectories[0];
    if (!arm->execute(plan)) {
      ROS_ERROR("Movement to pre place pose failed");
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }
    // cartesian movement to place pose
    plan.trajectory_ = current_trajectories[1];
    if (!arm->execute(plan)) {
      ROS_ERROR("Cartesian movement failed");
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }
    // Open gripper
    plan.trajectory_ = current_trajectories[2];
    if (!arm->execute(plan)) {
      ROS_ERROR("Open gripepr failed");
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }

    // Detach attached object
    std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
        psi->getAttachedObjects(std::vector<std::string>{current_object});
    moveit_msgs::AttachedCollisionObject attached_object =
        objects.at(current_object);
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
      state.val = hololens_grasp::State::ERROR;
      state_publisher_ptr->publish(state);
      return;
    }
  }

  // delete planned trajectory
  current_trajectories.clear();

  state.val = hololens_grasp::State::IDLE;
  state_publisher_ptr->publish(state);
}

void resetCallback(const std_msgs::Empty::ConstPtr &msg) {
  // Detach attached object
  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
      psi->getAttachedObjects(std::vector<std::string>{current_object});

  current_trajectories.clear();
  if (thread != NULL) {
    thread->join();
    thread = NULL;
  }

  if (objects.size() > 0) {
    moveit_msgs::AttachedCollisionObject attached_object =
        objects.at(current_object);

    attached_object.object.operation = attached_object.object.REMOVE;
    psi->applyAttachedCollisionObject(attached_object);

    current_object = "";
  }


  spawnObject("object1", -0.25, 0.25, 0.0375, 0.258);
  spawnObject("object2", 0, -0.25, 0.0375, 0.258);
  spawnObject("object3", 0.25, -0.25, 0.0375, 0.258);

  state.val = hololens_grasp::State::IDLE;
  state_publisher_ptr->publish(state);
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
      node_handle.subscribe("hololens/plan_pick", 2, planPickCallback);
  ros::Subscriber execute_pick_subscriber =
      node_handle.subscribe("hololens/execute_pick", 1, executePickCallback);
  ros::Subscriber open_gripper_subscriber =
      node_handle.subscribe("hololens/open_gripper", 1, openGripperCallback);
  ros::Subscriber plan_place_subscriber =
      node_handle.subscribe("hololens/plan_place", 1, planPlaceCallback);
  ros::Subscriber execute_place_subscriber =
      node_handle.subscribe("hololens/execute_place", 1, executePlaceCallback);
  ros::Subscriber reset_subscriber =
      node_handle.subscribe("hololens/reset", 1, resetCallback);

  planned_joint_states_publisher_ptr =
      new ros::Publisher(node_handle.advertise<sensor_msgs::JointState>(
          "hololens/planned_joint_states", 1));
  state_publisher_ptr = new ros::Publisher(
      node_handle.advertise<hololens_grasp::State>("hololens/state", 1, true));

  state.val = hololens_grasp::State::IDLE;
  state_publisher_ptr->publish(state);

  spawnObject("object1", -0.25, -0.25, 0.0375, 0.258);
  spawnObject("object2", 0, -0.25, 0.0375, 0.258);
  spawnObject("object3", 0.25, -0.25, 0.0375, 0.258);

  ros::waitForShutdown();
  return 0;
}
