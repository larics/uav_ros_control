#include <uav_ros_control/control/ModelPredictiveControl.hpp>
#include <uav_ros_lib/param_util.hpp>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <Eigen/Dense>



uav_ros_control::ModelPredictiveControl::ModelPredictiveControl()
{
  ROS_INFO("ModelPredictiveControl::ModelPredictiveControl()");
  // TODO:

  m_initial_state_x = Eigen::MatrixXd::Zero(3,1);
  m_initial_state_y = Eigen::MatrixXd::Zero(3,1);
  m_initial_state_z = Eigen::MatrixXd::Zero(3,1);
  m_reference_x = Eigen::MatrixXd::Zero(3,1);
  m_reference_y = Eigen::MatrixXd::Zero(3,1);
  m_reference_z = Eigen::MatrixXd::Zero(3,1);
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

  // loading constraints
  param_util::getParamOrThrow(parent_nh, "solver_x/max_speed", m_max_speed_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/max_acc", m_max_acc_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/max_du", m_max_du_x);
  param_util::getParamOrThrow(parent_nh, "solver_x/max_u", m_max_u_x);
  param_util::getParamOrThrow(parent_nh, "solver_y/max_speed", m_max_speed_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/max_acc", m_max_acc_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/max_du", m_max_du_y);
  param_util::getParamOrThrow(parent_nh, "solver_y/max_u", m_max_u_y);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_speed", m_max_speed_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_acc", m_max_acc_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_du", m_max_du_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_u", m_max_u_z);

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




  // getting data from Odometry and Trajectory
  initial_state = *uav_state;
  reference = *last_position_cmd;
  
  odom_pos_x = uav_state->pose.pose.position.x;
  odom_pos_y = uav_state->pose.pose.position.y;
  odom_pos_z = uav_state->pose.pose.position.z;
  odom_spd_x = uav_state->twist.twist.linear.x;
  odom_spd_y = uav_state->twist.twist.linear.y;
  odom_spd_z = uav_state->twist.twist.linear.z;
  odom_acc_x = 0;
  odom_acc_y = 0;
  odom_acc_z = 0;

  ref_pos_x = last_position_cmd->transforms[0].translation.x;
  ref_pos_y = last_position_cmd->transforms[0].translation.y;
  ref_pos_z = last_position_cmd->transforms[0].translation.z;
  ref_spd_x = last_position_cmd->velocities[0].linear.x;
  ref_spd_y = last_position_cmd->velocities[0].linear.y;
  ref_spd_z = last_position_cmd->velocities[0].linear.z;
  ref_acc_x = last_position_cmd->accelerations[0].linear.x;
  ref_acc_y = last_position_cmd->accelerations[0].linear.y;
  ref_acc_z = last_position_cmd->accelerations[0].linear.z;

  ref_orientation_Q = last_position_cmd->transforms[0].rotation;      // getting reference orientation (Quaternion)




  // filling in InitialState and Reference
  m_initial_state_x(0,0) = odom_pos_x;
  m_initial_state_x(1,0) = odom_spd_x;
  m_initial_state_x(2,0) = odom_acc_x;
  m_initial_state_y(0,0) = odom_pos_y;
  m_initial_state_y(1,0) = odom_spd_y;
  m_initial_state_y(2,0) = odom_acc_y;
  m_initial_state_z(0,0) = odom_pos_z;
  m_initial_state_z(1,0) = odom_spd_z;
  m_initial_state_z(2,0) = odom_acc_z;

  m_reference_x(0,0) = ref_pos_x;
  m_reference_x(1,0) = ref_spd_x;
  m_reference_x(2,0) = ref_acc_x;
  m_reference_y(0,0) = ref_pos_y;
  m_reference_y(1,0) = ref_spd_y;
  m_reference_y(2,0) = ref_acc_y;
  m_reference_z(0,0) = ref_pos_z;
  m_reference_z(1,0) = ref_spd_z;
  m_reference_z(2,0) = ref_acc_z;





  // neke linije bi mozda mogle u initialize buduci da se nemoraju izvrsavati stalno?

  
  m_solver_x->setInitialState(m_initial_state_x);
  m_solver_x->loadReference(m_reference_x);
  m_solver_x->setLimits(m_max_speed_x, m_max_acc_x, m_max_u_x, m_max_du_x, m_dt1_x, m_dt2_x);
  m_solver_x->setDt(m_dt1_x, m_dt2_x);
  m_solver_x->setQ(m_Q_x);
  m_solver_x->setS(m_Q_last_x);
  m_solver_x->setParams();
  m_solver_x->solveCvx();
  m_u_x = m_solver_x->getFirstControlInput();      // ovo je akceleracija u x-smjeru <double> dobivena iz solvera

  m_solver_y->setInitialState(m_initial_state_y);
  m_solver_y->loadReference(m_reference_y);
  m_solver_y->setLimits(m_max_speed_y, m_max_acc_y, m_max_u_y, m_max_du_y, m_dt1_y, m_dt2_y);
  m_solver_y->setDt(m_dt1_y, m_dt2_y);
  m_solver_y->setQ(m_Q_y);
  m_solver_y->setS(m_Q_last_y);
  m_solver_y->setParams();
  m_solver_y->solveCvx();
  m_u_y = m_solver_y->getFirstControlInput();      // ovo je akceleracija u y-smjeru <double> dobivena iz solvera

  m_solver_z->setInitialState(m_initial_state_z);
  m_solver_z->loadReference(m_reference_z);
  m_solver_z->setLimits(m_max_speed_z, m_max_acc_z, m_max_u_z, m_max_du_z, m_dt1_z, m_dt2_z);
  m_solver_z->setDt(m_dt1_z, m_dt2_z);
  m_solver_z->setQ(m_Q_z);
  m_solver_z->setS(m_Q_last_z);
  m_solver_z->setParams();
  m_solver_z->solveCvx();
  m_u_z = m_solver_z->getFirstControlInput();      // ovo je akceleracija u z-smjeru <double> dobivena iz solvera



  

  // 2. Use the desired acceleration to calculate the orientation

  eig_ref_orientation_Q.x() = ref_orientation_Q.x;          // getting reference orientation (transform to Eigen::Quaternionf)
  eig_ref_orientation_Q.y() = ref_orientation_Q.y; 
  eig_ref_orientation_Q.z() = ref_orientation_Q.z; 
  eig_ref_orientation_Q.w() = ref_orientation_Q.w;

  euler = eig_ref_orientation_Q.toRotationMatrix().eulerAngles(0, 1, 2);      // getting reference heading (yaw)
  roll = euler[0];
  pitch = euler[1];
  yaw = euler[2];

  Xc(0) = cos(yaw);       // heading constraints
  Xc(1) = sin(yaw);
  Xc(2) = 0;
  Yc(0) = -sin(yaw);
  Yc(1) = cos(yaw);
  Yc(2) = 0;

  a_des(0) = m_u_x;       // filling desired acceleration from solver
  a_des(1) = m_u_y;
  a_des(2) = m_u_z;

  Zb = a_des/a_des.norm();         // calculating desired orientation R_des
  Xb = (Yc.cross(Zb))/(Yc.cross(Zb).norm());
  Yb = Zb.cross(Xb);

  R_des << Xb, Yb, Zb;        // filling desired orientation

  eig_des_orientation_Q = R_des;      // matrix to quaternion conversion








  // 3. Use the desired acceleration to calculate thrust

  des_orientation_Q.x = eig_des_orientation_Q.x();      // filling in geometry_msgs/Quaternion
  des_orientation_Q.y = eig_des_orientation_Q.y();
  des_orientation_Q.z = eig_des_orientation_Q.z();
  des_orientation_Q.w = eig_des_orientation_Q.w();

  attitude_target.orientation = des_orientation_Q;

  return attitude_target;
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_control::ModelPredictiveControl,
  uav_ros_control::controller_interface);