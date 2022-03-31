// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/ikfast_moveit_state_adapter.h>

// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

// Includes the utility function for converting to trajectory_msgs::JointTrajectory's
#include <descartes_utilities/ros_conversions.h>

// ROS Messages
#include <sensor_msgs/JointState.h>


// ----------------------
// Function Declarations
// ----------------------

// Example path trajectory
std::vector<descartes_core::TrajectoryPtPtr> makePath(Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity());

// Send ROS formatted trajectory to the robot controller
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);


// ----------------------
// Main
// ----------------------

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "paper_glider_demo_descartes");
  ros::NodeHandle nh;

  // Keep ROS Node & Supporint Processes Alive (single thread)
  ros::AsyncSpinner spinner (1);
  spinner.start();


  // 1. Load kinematic model of robot using a moveit configuration.
  //    Supported IK solvers: ikfast, trac-ik
  descartes_core::RobotModelPtr model (new descartes_moveit::IkFastMoveitStateAdapter());

  // Robot Model Description, ROS Parameter name
  const std::string robot_description = "robot_description";

  // Kinematic group in robot's moveit_config
  const std::string group_name = "gp7";

  // Define reference frames
  const std::string world_frame = "base_link";
  const std::string tcp_frame = "tool0";

  // Initialize model. This will load robot models and sanity check the model.
  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  model->setCheckCollisions(true); // Let's turn on collision checking.


  // 2. Assemble an path plan (using an simple example via makePath() function)

  // define path origin as current robot flange-TCP position
  const std::string jointTopic = "/joint_states";
  sensor_msgs::JointStateConstPtr current_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(jointTopic, ros::Duration(5));

  Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
  model->getFK(current_state_ptr->position, pattern_origin);

  // assemble path about path origin
  std::vector<descartes_core::TrajectoryPtPtr> points = makePath(pattern_origin);

  // 3. Initialize one of Descarte's planners to build a trajectory from your path plan.
  // Planners:  DensePlanner. Naive, brute force method
  //            SparsePlanner. Faster for some problems (especially very dense ones), but less robust.
  descartes_planner::DensePlanner planner;

  // Like the model, you also need to call initialize on the planner
  if (!planner.initialize(model))
  {
    ROS_ERROR("Failed to initialize planner");
    return -2;
  }


  // 4. Use Descarte's planner to build a trajectory. Two step process.

  //    First, call planPath() to convert input trajectory into a large kinematic "graph".
  //           Failures at this point indicate that the input path may not have solutions at a given point
  //           (because of reach/collision) or has two points with no way to connect them.
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -3;
  }

  // Second, call 'getPath()' to searches the graph for a minimum cost path and return result.
  //        Failures here (assuming planPath was good) indicate that your path has solutions at every waypoint
  //        but constraints prevent a solution through the whole path. Usually this means a singularity is hanging out in the
  //        middle of your path: the robot can solve all the points but not in the same arm configuration.
  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -4;
  }

  // 5. Translate the result into something that you can execute. In ROS land, this means that we turn the result into
  // a trajectory_msgs::JointTrajectory that's executed through a control_msgs::FollowJointTrajectoryAction.

  // get joint names - this could be from the robot model, or from the parameter server.
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);

  // create a JointTrajectory
  trajectory_msgs::JointTrajectory joint_solution;
  joint_solution.joint_names = names;

  // Define a default velocity. Descartes points without specified timing will use this value to limit the
  // fastest moving joint. This usually effects the first point in your path the most.
  const static double default_joint_vel = 0.5; // rad/s
  if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel, joint_solution.points))
  {
    ROS_ERROR("Unable to convert Descartes trajectory to joint points");
    return -5;
  }

  // 6. Send the ROS trajectory to the robot for execution
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -6;
  }

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}



// ----------------------
// Support Functions
// ----------------------

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI / 12.0, AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
}



// ----------------------
// Example Path Creation
// ----------------------

std::vector<descartes_core::TrajectoryPtPtr> makePath(Eigen::Isometry3d pattern_origin)
{
  // In Descartes, trajectories are composed of "points". Each point describes what joint positions of the robot can
  // satisfy it. You can have a "joint point" for which only a single solution is acceptable. You might have a
  // fully defined cartesian point for which many (8 or 16) different robot configurations might work. You could
  // allow extra tolerances in any of these and even more points satisfy the constraints.

  // In this first tutorial, we're just going to describe a simple cartesian trajectory that moves the robot
  // along a line in the XY plane.

  // Step 1: Let's start by just doing the math to generate the poses we want.

  // First thing, let's generate a pattern with its origin at zero. We'll define another transform later that
  // can move it to somewere more convenient.
  const static double step_size = 0.010;
  const static int num_steps = 100;
  const static double time_between_points = 0.01;

  EigenSTL::vector_Isometry3d pattern_poses;
  for (int i = 0; i < num_steps / 2; ++i)
  {
    // create a pose and initialize it to identity
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    
    // set tool position (move along a line in '-Y' direction)(relative to pattern_origin)
    pose.translation() = Eigen::Vector3d(-i * step_size, 0, 0);
    
    // set tool orientation
    // pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()); // this flips the tool around so that Z points forward
    pattern_poses.push_back(pose);
  }

  std::vector<descartes_core::TrajectoryPtPtr> result;
  for (const auto& pose : pattern_poses)
  {
    // This creates a trajectory that searches around the tool Z and let's the robot move in that null space
    // descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_between_points);
    // This creates a trajectory that is rigid. The tool cannot float and must be at exactly this point.
    descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * pose, time_between_points);
    result.push_back(pt);
  }

  // Note that we could also add a joint point representing the starting location of the robot, or a joint point
  // representing the desired end pose of the robot to the front and back of the vector respectively.

  return result;
}



// ----------------------
// Function Definitions
// ----------------------

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  ROS_INFO("Connected to action server!");

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(2.0);
  
  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}
