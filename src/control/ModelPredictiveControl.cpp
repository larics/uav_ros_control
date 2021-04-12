#include <uav_ros_control/control/ModelPredictiveControl.hpp>
#include <uav_ros_lib/param_util.hpp>

uav_ros_control::ModelPredictiveControl::ModelPredictiveControl()
{
  ROS_INFO("ModelPredictiveControl::ModelPredictiveControl()");
  // TODO:
}

void uav_ros_control::ModelPredictiveControl::initialize(ros::NodeHandle &parent_nh,
  const std::string name,
  const double uav_mass)
{
  ROS_INFO("ModelPredictiveControl::initialize()");
  // TODO:

  // 0.step: Load all parameters
  double moj_param;
  Eigen::MatrixXd m_A_orig;

  param_util::getParamOrThrow(parent_nh, "solver_x/dt1", moj_param);
  m_A_orig = param_util::loadMatrixOrThrow(parent_nh, "A", 4, 4);

  // 1. step: Initialize m_solver private variables
  //  - m_solver_x = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>
  //                 ( ovdje, dolaze, argumenti, CVXWrapper, konstruktoria)


  m_solver_x = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(true,
    30,
    std::vector<double>{ 100, 50, 10 },
    std::vector<double>{ 100, 50, 10 },
    0.01,
    0.2,
    1,
    1);


  m_solver_y = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(true,
    30,
    std::vector<double>{ 100, 50, 10 },
    std::vector<double>{ 100, 50, 10 },
    0.01,
    0.2,
    1,
    1);

  m_solver_z = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(true,
    30,
    std::vector<double>{ 100, 50, 10 },
    std::vector<double>{ 100, 50, 10 },
    0.01,
    0.2,
    1,
    1);
}


bool uav_ros_control::ModelPredictiveControl::activate(
  const mavros_msgs::AttitudeTargetConstPtr &last_attitude_cmd)
{
  ROS_INFO("ModelPredictiveControl::activate");
  // TODO: You have to check if m_is_active is True.
  if (!m_is_active) { m_is_active = true; }
  return m_is_active;
}


void uav_ros_control::ModelPredictiveControl::deactivate()
{
  ROS_INFO("ModelPredictiveControl::deactivate");
  // TODO:
  m_is_active = false;
}


const mavros_msgs::AttitudeTarget uav_ros_control::ModelPredictiveControl::update(
  const nav_msgs::OdometryConstPtr &uav_state,
  const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &last_position_cmd)
{
  ROS_INFO_THROTTLE(5.0, "ModelPredictiveControl::update");

  if (!m_is_active) { return  mavros_msgs::AttitudeTarget{}; }

  // TODO: Do the control loop

  // 1. Use all the solvers to determine the desired acceleration for the UAV

  Eigen::MatrixXd initial_state;
  Eigen::MatrixXd reference;
  double max_speed, max_acc, max_u, max_du, dt, dt1, dt2;
  std::vector<double> Q, S;
  double u_x;

  // TODO(bgrujic): The code below is commented out becasue it casues a segmentation fault
  // m_solver_x->setInitialState(initial_state);
  // m_solver_x->loadReference(reference);
  // m_solver_x->setLimits(max_speed, max_acc, max_u, max_du, dt1, dt2);
  // m_solver_x->setDt(dt, dt2);// jeli to isto kao dt1 i dt2?
  // m_solver_x->setQ(Q);
  // m_solver_x->setS(S);
  // m_solver_x->setParams();

  // m_solver_x->solveCvx();
  // u_x = m_solver_x->getFirstControlInput();// je li u_x akceleracija u x-smjeru?


  // 2. Use the desired acceleration to calculate the orientation
  // 3. Use the desired acceleration to calculate thrust

  mavros_msgs::AttitudeTarget attitude_target;
  // 3. step - Fill out the atttitude_target object - VERY IMPORTANT!
  return attitude_target;
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_control::ModelPredictiveControl,
  uav_ros_control::controller_interface);