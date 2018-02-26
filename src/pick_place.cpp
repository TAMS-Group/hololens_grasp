/*
  1. spawn objects on table top defined in launch
  2. listen to topic and get point on object
  3. use pickup action to plan trajectories
  4. publish succ && "visualize" trajectories by publishing joint states on topic
  5. listen to execute topic
  6. execute trajectuies


*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/Grasp.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>


ros::Publisher* planned_joint_states_publisher_ptr;
ros::Publisher* plan_success_publisher_ptr;

moveit_msgs::RobotTrajectory *current_trajectories_ptr;

void spawnObject(ros::NodeHandle node_handle, std::string name, float x, float y, float radius, float height){
  moveit_msgs::ApplyPlanningScene srv;
  moveit_msgs::PlanningScene planning_scene;
  ros::ServiceClient planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();

  planning_scene.is_diff = true;
  //planning_scene.robot_state.is_diff = true;

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
  pose.position.z = height/2;

  // Add object to scene
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;
  planning_scene.world.collision_objects.push_back(object);

  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
}

void publishPlannedTrajectory(std::vector<moveit_msgs::RobotTrajectory> trajectories) {
  sensor_msgs::JointState joint_state;
  //while(ros::ok()) {
    for(int i=0; i<trajectories.size(); i++) {
      for(int j=0; j<trajectories[i].joint_trajectory.points.size(); j++) {
        for(int k=0; k<trajectories[i].joint_trajectory.joint_names.size(); k++) {
          joint_state.name[k] = trajectories[i].joint_trajectory.joint_names[k];
          joint_state.position[k] = trajectories[i].joint_trajectory.points[j].positions[k];
          joint_state.velocity[k] = trajectories[i].joint_trajectory.points[j].velocities[k];
          joint_state.effort[k] = trajectories[i].joint_trajectory.points[j].effort[k];
        }
        planned_joint_states_publisher_ptr->publish(joint_state);
        if(i==0)
          ros::Duration(trajectories[i].joint_trajectory.points[j].time_from_start - trajectories[i-1].joint_trajectory.points[j].time_from_start).sleep();
      }
    }
  //}
}

void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration, 
        trajectory_msgs::JointTrajectory &grasp_pose)
{
  grasp_pose.joint_names.reserve(target_values.size());
  grasp_pose.points.resize(1);
  grasp_pose.points[0].positions.reserve(target_values.size());

  for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it){
    grasp_pose.joint_names.push_back(it->first);
    grasp_pose.points[0].positions.push_back(it->second);
  }
  grasp_pose.points[0].time_from_start = duration;
}

moveit_msgs::Grasp generateGrasp(geometry_msgs::PointStamped msg) {
  moveit::planning_interface::MoveGroupInterface move_group("arm");
  moveit::planning_interface::MoveGroupInterface gripper(move_group.getRobotModel()->getEndEffectors()[0]->getName());

  moveit_msgs::Grasp grasp;
  grasp.id = "grasp";

  jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp.pre_grasp_posture);
  jointValuesToJointTrajectory(gripper.getNamedTargetValues("closed"), ros::Duration(2.0), grasp.grasp_posture);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = msg.header.frame_id;
//TODO
  float w = atan2(msg.point.y, msg.point.x)-atan2(0, 1);
ROS_ERROR_STREAM("angle " << w);
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, w+M_PI);
  pose.pose.position.x = msg.point.x;
  pose.pose.position.y = msg.point.y;
  pose.pose.position.z = msg.point.z;

/*
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, M_PI/2, 0.0);
  pose.pose.position.z = msg.point.z;
*/
  grasp.grasp_pose = pose;

  grasp.pre_grasp_approach.min_distance = 0.08;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasp.pre_grasp_approach.direction.header.frame_id = "s_model_tool0";
  grasp.pre_grasp_approach.direction.vector.x = 1.0;

  grasp.post_grasp_retreat.min_distance = 0.08;
  grasp.post_grasp_retreat.desired_distance = 0.1;
  grasp.post_grasp_retreat.direction.header.frame_id = move_group.getPlanningFrame();
  grasp.post_grasp_retreat.direction.vector.z = 1.0;


  tf::TransformBroadcaster br;
  tf::Transform transform;


  ros::Rate rate(1.0);
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.pose.orientation, quat);

  
  for (int i=0; i< 10; i++){
    transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z + 0.1) );
    transform.setRotation( quat);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_top", "grasp_point"));
    rate.sleep();
  }

  return grasp;
}

void planCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {

  actionlib::SimpleActionClient<moveit_msgs::PickupAction> action_client("pickup",true);
  ROS_INFO("Waiting for pickup action server to start.");
  action_client.waitForServer();

  moveit_msgs::PickupGoal goal;

  // Object name
  goal.target_name = msg->header.frame_id;

  // Move Group
  goal.group_name = "arm";

  // end-effector
  goal.end_effector = "gripper";

  goal.support_surface_name = "table_top";

  //goal.attached_object_touch_links = { , , };

  //goal.planning_options.plan_only = true;

  goal.possible_grasps.push_back(generateGrasp(*msg));

  goal.allowed_planning_time = 30;

  // Send goal
  action_client.sendGoal(goal);
  action_client.waitForResult(ros::Duration(30.0));
  moveit_msgs::PickupResult result = *(action_client.getResult());


  std_msgs::Bool success;

  if(result.error_code.val == 1) {
    ROS_INFO("Plan found");
    success.data = true;
    ROS_ERROR_STREAM(result.error_code);
    //current_trajectories_ptr = result.trajectory_stages;
    publishPlannedTrajectory(result.trajectory_stages);
  }
  else {
    ROS_INFO("No plan found");
    ROS_ERROR_STREAM(result.error_code);
    success.data = false;
    current_trajectories_ptr = NULL;
  }

  // Publish bool to signal unity if the planning was successfull 
  plan_success_publisher_ptr->publish(success);
}


void executeCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "HololensPickPlace");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node_handle;

  spawnObject(node_handle, "object1", 0, 0, 0.02, 0.2);

  ros::Subscriber plan_subscriber = node_handle.subscribe("hololens_plan_pick", 1, planCallback);
  ros::Subscriber execute_subscriber = node_handle.subscribe("hololens_execute_pick", 1, executeCallback);

  planned_joint_states_publisher_ptr = new ros::Publisher(node_handle.advertise<sensor_msgs::JointState>("planned_joint_states",1));
  plan_success_publisher_ptr = new ros::Publisher(node_handle.advertise<std_msgs::Bool>("planned_successfull",1));

  ros::waitForShutdown();
  return 0;
}
