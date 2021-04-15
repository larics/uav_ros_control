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
  // ucitati sve parametre za solver u .yaml i .hpp file-ove

  double moj_param;
  Eigen::MatrixXd m_A_orig;

  param_util::getParamOrThrow(parent_nh, "solver_x/dt1", moj_param);
  m_A_orig = param_util::loadMatrixOrThrow(parent_nh, "A", 4, 4);




  // 1. step: Initialize m_solver private variables
  //  - m_solver_x = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>
  //                 ( ovdje, dolaze, argumenti, CVXWrapper, konstruktoria)



  // loading parameters to constructor variables:
  // m_solver_x:
  param_util::getParamOrThrow(parent_nh, "solver_x/verbose", m_verbose_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/max_iters", m_max_iters_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/Q", m_Q_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/Q_last", m_Q_last_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/dt1", m_dt1_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/dt2", m_dt2_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/p1", m_p1_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/p2", m_p2_x);

  // m_solver_y:
  param_util::getParamOrThrow(parent_nh, "solver_y/verbose", m_verbose_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/max_iters", m_max_iters_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/Q", m_Q_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/Q_last", m_Q_last_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/dt1", m_dt1_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/dt2", m_dt2_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/p1", m_p1_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/p2", m_p2_y);
  
  // m_solver_z:
  param_util::getParamOrThrow(parent_nh, "solver_z/verbose", m_verbose_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_iters", m_max_iters_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/Q", m_Q_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/Q_last", m_Q_last_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/dt1", m_dt1_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/dt2", m_dt2_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/p1", m_p1_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/p2", m_p2_z);

  // creating solver objects:
  m_solver_x = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(m_verbose_x, m_max_iters_x, m_Q_x, m_Q_last_x, m_dt1_x, m_dt2_x, m_p1_x, m_p2_x);
  m_solver_x = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(m_verbose_y, m_max_iters_y, m_Q_y, m_Q_last_y, m_dt1_y, m_dt2_y, m_p1_y, m_p2_y);
  m_solver_x = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(m_verbose_z, m_max_iters_z, m_Q_z, m_Q_last_z, m_dt1_z, m_dt2_z, m_p1_z, m_p2_z);


  // m_solver_z = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(true,
  //  30,
  //  std::vector<double>{ 100, 50, 10 },
  //  std::vector<double>{ 100, 50, 10 },
  //  0.01,
  //  0.2,
  //  1,
  //  1);
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

  Eigen::MatrixXd initial_state;  // (pozicija, brzina, akceleracija)
  Eigen::MatrixXd reference;
  double m_max_speed_x, max_acc, max_u, max_du, dt, dt1, dt2; // dodati u .hpp, dodati m_-----_x
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