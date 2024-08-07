#include <ros/ros.h>
//#include <trajectory_msgs/JointTrajectory.h>
#include "myworkcell_core/PlanCartesianPath.h"

//#include <ur5_demo_descartes/ur5_robot_model.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_utilities/ros_conversions.h>
#include <eigen_conversions/eigen_msg.h>


std::vector<double> getCurrentJointState(const std::string& topic)
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));
  if (!state) throw std::runtime_error("Joint state message capture failed");
  
  /// Prenons à la place de la position de gazebo, 
  /// la dernière position envoyé en joint
  return state->position;
}

EigenSTL::vector_Isometry3d makeLine(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, double ds)
{
  EigenSTL::vector_Isometry3d line;
  
  const Eigen::Vector3d travel = stop - start;
  const int steps = std::floor(travel.norm() / ds);

  // Linear interpolation
  for (int i = 0; i < steps; ++i)
  {
    double ratio = static_cast<float>(i) / steps;
    Eigen::Vector3d position = start + ratio * travel;
    Eigen::Isometry3d tr;
    tr = Eigen::Translation3d(position);
    line.push_back( tr );
  }

  return line;
}

class CartesianPlanner
{
public:
  CartesianPlanner(ros::NodeHandle& nh)
  {
    // first init descartes
    if (!initDescartes())
      throw std::runtime_error("There was an issue initializing Descartes");

    // init services
    server_ = nh.advertiseService("plan_path", &CartesianPlanner::planPath, this);
  }

  bool initDescartes()
  {
    // Create a robot model
    //Exemple avec ur5
    // model_ = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();
    // Prenon a robot model selon tutoriel 1 de descarte
    ROS_WARN("BEFORE MODEL_");
    model_ = descartes_core::RobotModelPtr(new descartes_moveit::IkFastMoveitStateAdapter());
    //ROS_WARN("AFTER  MODEL_"); // Ajouter par moi

    // Define the relevant "frames"
    const std::string robot_description = "robot_description";
    const std::string group_name = "manipulator";
    const std::string world_frame = "world"; // Frame in which tool poses are expressed
    const std::string tcp_frame = "TCP";

    // Using the desired frames, let's initialize Descartes
    ROS_INFO("initialize model");
    if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_WARN("Descartes RobotModel failed to initialize");
      return false;
    }

    ROS_INFO("initialize planner");
    if (!planner_.initialize(model_))
    {
      ROS_WARN("Descartes Planner failed to initialize");
      return false;
    }
    return true;
  }

  bool planPath(myworkcell_core::PlanCartesianPathRequest& req,
                myworkcell_core::PlanCartesianPathResponse& res)
  {
    ROS_INFO("Recieved cartesian planning request");

    // Step 1: Generate path poses
    ROS_INFO("descartes node : step 1");
    EigenSTL::vector_Isometry3d tool_poses = makeToolPoses();
    
    // Step 2: Translate that path by the input reference pose and convert to "Descartes points"
    ROS_INFO("descartes node : step 2");
    std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectory(req.pose, tool_poses);

    // Step 3: Tell Descartes to start at the "current" robot position
    ROS_INFO("descartes node : step 3");
    // std::vector<double> start_joints = getCurrentJointState("/MYROBOT/joint_states");
    std::vector<double> start_joints = getCurrentJointState("/joint_states");
    descartes_core::TrajectoryPtPtr pt (new descartes_trajectory::JointTrajectoryPt(start_joints));
    path.front() = pt;

    // Step 4: Plan with descartes
    ROS_INFO("descartes node : step 4");
    if (!planner_.planPath(path))
    {
      ROS_INFO("descartes node: Failed at planPath");
      return false;
    }

    std::vector<descartes_core::TrajectoryPtPtr> result;
    if (!planner_.getPath(result))
    {
      ROS_INFO("descartes node:Failed at getPath");
      return false;
    }

    // Step 5: Convert the output trajectory into a ROS-formatted message
    ROS_INFO("descartes node : step 5");
    res.trajectory.header.stamp = ros::Time::now();
    res.trajectory.header.frame_id = "world";
  
    res.trajectory.joint_names = getJointNames();
    
    descartes_utilities::toRosJointPoints(*model_, result, 1.0, res.trajectory.points);
    
    return true;
  }

  EigenSTL::vector_Isometry3d makeToolPoses()
  {
    EigenSTL::vector_Isometry3d path;

    // We assume that our path is centered at (0, 0, 0), so let's define the
    // corners of the AR marker
    const double side_length = 0.08; // All units are in meters (M)
    const double half_side = side_length / 2.0;
    const double step_size = 0.02;

    Eigen::Vector3d top_left (-half_side, half_side, 0);
    Eigen::Vector3d bot_left (-half_side, -half_side, 0);
    Eigen::Vector3d bot_right (half_side, -half_side, 0);
    Eigen::Vector3d top_right (half_side, half_side, 0);

    // Descartes requires you to guide it in how dense the points should be,
    // so you have to do your own "discretization".
    // NOTE that the makeLine function will create a sequence of points inclusive
    // of the start and exclusive of finish point, i.e. line = [start, stop)
    
    // TODO: Add the rest of the cartesian path
    auto segment1 = makeLine(top_left, bot_left, step_size);
    auto segment2 = makeLine(bot_left, bot_right, step_size);
    auto segment3 = makeLine(bot_right, top_right, step_size);
    auto segment4 = makeLine(top_right, top_left, step_size);

    path.insert(path.end(), segment1.begin(), segment1.end());
    path.insert(path.end(), segment2.begin(), segment2.end());
    path.insert(path.end(), segment3.begin(), segment3.end());
    path.insert(path.end(), segment4.begin(), segment4.end());

    return path;
  }

  std::vector<descartes_core::TrajectoryPtPtr>
  makeDescartesTrajectory(const geometry_msgs::Pose& reference,
                           const EigenSTL::vector_Isometry3d& path)
  {
    std::vector<descartes_core::TrajectoryPtPtr> descartes_path; // return value

    Eigen::Isometry3d ref;
    tf::poseMsgToEigen(reference, ref);

    for (auto& point : path)
    {
      // TODO: make a Descartes "cartesian" point with some kind of constraints
      descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(ref * point);
      descartes_path.push_back(pt);
    }
    return descartes_path;
  }

  descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;
    // modification pour avoir seulement un axe free
    // return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0, AxialSymmetricPt::Z_AXIS) );
  }

  // HELPER
  std::vector<std::string> getJointNames()
  {
    std::vector<std::string> names;
    nh_.getParam("controller_joint_names", names);
    return names;
  }


  descartes_core::RobotModelPtr model_;
  descartes_planner::DensePlanner planner_;
  ros::ServiceServer server_;
  ros::NodeHandle nh_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descartes_node");

  ros::NodeHandle nh;
  // ros::Duration(1.5).sleep(); // make sure parameter are loaded
  CartesianPlanner planner (nh);

  ROS_INFO("Cartesian planning node starting");
  ros::spin();
}
