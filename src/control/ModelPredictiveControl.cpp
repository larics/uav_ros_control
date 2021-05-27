#include <uav_ros_control/control/ModelPredictiveControl.hpp>
#include <uav_ros_lib/param_util.hpp>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>


uav_ros_control::ModelPredictiveControl::ModelPredictiveControl()
{
  ROS_INFO("ModelPredictiveControl::ModelPredictiveControl()");
  // TODO:

  m_initial_state_x = Eigen::MatrixXd::Zero(3, 1);
  m_initial_state_y = Eigen::MatrixXd::Zero(3, 1);
  m_initial_state_z = Eigen::MatrixXd::Zero(3, 1);
}


void uav_ros_control::ModelPredictiveControl::initialize(ros::NodeHandle&  parent_nh,
                                                         const std::string name,
                                                         const double      uav_mass)
{
  ROS_INFO("ModelPredictiveControl::initialize()");

  m_acc_desired_pub = parent_nh.advertise<geometry_msgs::Vector3>("debug/desired_acc", 1);
  m_thrust_force_pub  = parent_nh.advertise<std_msgs::Float64>("debug/thrust_force", 1);
  m_scaled_thrust_pub = parent_nh.advertise<std_msgs::Float64>("debug/scaled_thrust", 1);

  // loading parameters to constructor variables
  // m_solver_xy:
  param_util::getParamOrThrow(parent_nh, "solver_xy/verbose", m_verbose_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/max_iters", m_max_iters_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/Q", m_Q_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/Q_last", m_Q_last_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/dt1", m_dt1_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/dt2", m_dt2_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/p1", m_p1_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/p2", m_p2_xy);

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
  m_solver_x = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(m_verbose_xy,
                                                                          m_max_iters_xy,
                                                                          m_Q_xy,
                                                                          m_Q_last_xy,
                                                                          m_dt1_xy,
                                                                          m_dt2_xy,
                                                                          m_p1_xy,
                                                                          m_p2_xy);
  m_solver_y = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(m_verbose_xy,
                                                                          m_max_iters_xy,
                                                                          m_Q_xy,
                                                                          m_Q_last_xy,
                                                                          m_dt1_xy,
                                                                          m_dt2_xy,
                                                                          m_p1_xy,
                                                                          m_p2_xy);
  m_solver_z = std::make_unique<uav_ros_control::cvx_wrapper::CvxWrapper>(
    m_verbose_z, m_max_iters_z, m_Q_z, m_Q_last_z, m_dt1_z, m_dt2_z, m_p1_z, m_p2_z);

  // loading constraints
  param_util::getParamOrThrow(parent_nh, "solver_xy/max_speed", m_max_speed_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/max_acc", m_max_acc_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/max_du", m_max_du_xy);
  param_util::getParamOrThrow(parent_nh, "solver_xy/max_u", m_max_u_xy);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_speed", m_max_speed_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_acc", m_max_acc_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_du", m_max_du_z);
  param_util::getParamOrThrow(parent_nh, "solver_z/max_u", m_max_u_z);

  // loading other parameters
  param_util::getParamOrThrow(
    parent_nh, "feed_forward_acceleration_flag", feed_fwd_acc_flag);
  param_util::getParamOrThrow(parent_nh, "UAV_mass", UAV_mass);
  param_util::getParamOrThrow(parent_nh, "horizon_len", m_horizon_len);

  // initializing references for solver
  m_reference_x = Eigen::MatrixXd::Zero(3 * m_horizon_len, 1);
  m_reference_y = Eigen::MatrixXd::Zero(3 * m_horizon_len, 1);
  m_reference_z = Eigen::MatrixXd::Zero(3 * m_horizon_len, 1);
}


bool uav_ros_control::ModelPredictiveControl::activate(
  const mavros_msgs::AttitudeTargetConstPtr& last_attitude_cmd)
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
  const nav_msgs::OdometryConstPtr&                            uav_state,
  const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& last_position_cmd)
{
  ROS_INFO_THROTTLE(5.0, "ModelPredictiveControl::update");

  if (!m_is_active) { return mavros_msgs::AttitudeTarget{}; }

  // getting data from Odometry and Trajectory
  initial_state = *uav_state;
  reference     = *last_position_cmd;

  odom_pos_x = uav_state->pose.pose.position.x;
  odom_pos_y = uav_state->pose.pose.position.y;
  odom_pos_z = uav_state->pose.pose.position.z;
  odom_spd_x = uav_state->twist.twist.linear.x;
  odom_spd_y = uav_state->twist.twist.linear.y;
  odom_spd_z = -uav_state->twist.twist.linear.z;
  odom_acc_x = 0;
  odom_acc_y = 0;
  odom_acc_z = 0;
  R_Q        = uav_state->pose.pose.orientation;// getting actual uav orientation

  ref_pos_x = last_position_cmd->transforms[0].translation.x;
  ref_pos_y = last_position_cmd->transforms[0].translation.y;
  ref_pos_z = last_position_cmd->transforms[0].translation.z;
  ref_spd_x = last_position_cmd->velocities[0].linear.x;
  ref_spd_y = last_position_cmd->velocities[0].linear.y;
  ref_spd_z = last_position_cmd->velocities[0].linear.z;
  ref_acc_x = last_position_cmd->accelerations[0].linear.x;
  ref_acc_y = last_position_cmd->accelerations[0].linear.y;
  ref_acc_z = last_position_cmd->accelerations[0].linear.z;

  ref_orientation_Q = last_position_cmd->transforms[0]
                        .rotation;// getting reference orientation (Quaternion)

  ROS_INFO_THROTTLE(5.0,
                    "Current: [%.2f, %.2f, %.2f] Referent: [%.2f, %.2f, %.2f]",
                    odom_pos_x,
                    odom_pos_y,
                    odom_pos_z,
                    ref_pos_x,
                    ref_pos_y,
                    ref_pos_z);

  // filling in InitialState and Reference
  m_initial_state_x(0, 0) = odom_pos_x;
  m_initial_state_x(1, 0) = odom_spd_x;
  m_initial_state_x(2, 0) = odom_acc_x;
  m_initial_state_y(0, 0) = odom_pos_y;
  m_initial_state_y(1, 0) = odom_spd_y;
  m_initial_state_y(2, 0) = odom_acc_y;
  m_initial_state_z(0, 0) = odom_pos_z;
  m_initial_state_z(1, 0) = odom_spd_z;
  m_initial_state_z(2, 0) = odom_acc_z;

  for (int i = 0; i < m_horizon_len; i++) {
    m_reference_x(3 * i + 0, 0) = ref_pos_x;
    m_reference_x(3 * i + 1, 0) = ref_spd_x;
    m_reference_x(3 * i + 2, 0) = ref_acc_x;

    m_reference_y(3 * i + 0, 0) = ref_pos_y;
    m_reference_y(3 * i + 1, 0) = ref_spd_y;
    m_reference_y(3 * i + 2, 0) = ref_acc_y;

    m_reference_z(3 * i + 0, 0) = ref_pos_z;
    m_reference_z(3 * i + 1, 0) = ref_spd_z;
    m_reference_z(3 * i + 2, 0) = ref_acc_z;
  }


  // ----------------------------------//
  // using the solver for optimization //
  // ----------------------------------//

  m_solver_x->lock();
  m_solver_x->setInitialState(m_initial_state_x);
  m_solver_x->loadReference(m_reference_x);
  m_solver_x->setLimits(
    m_max_speed_xy, m_max_acc_xy, m_max_u_xy, m_max_du_xy, m_dt1_xy, m_dt2_xy);
  m_solver_x->setDt(m_dt1_xy, m_dt2_xy);
  m_solver_x->setQ(m_Q_xy);
  m_solver_x->setS(m_Q_last_xy);
  m_solver_x->setParams();
  m_solver_x->solveCvx();
  m_u_x = m_solver_x->getFirstControlInput();// acceleration in x-direction (double)
  m_solver_x->unlock();

  m_solver_y->lock();
  m_solver_y->setInitialState(m_initial_state_y);
  m_solver_y->loadReference(m_reference_y);
  m_solver_y->setLimits(
    m_max_speed_xy, m_max_acc_xy, m_max_u_xy, m_max_du_xy, m_dt1_xy, m_dt2_xy);
  m_solver_y->setDt(m_dt1_xy, m_dt2_xy);
  m_solver_y->setQ(m_Q_xy);
  m_solver_y->setS(m_Q_last_xy);
  m_solver_y->setParams();
  m_solver_y->solveCvx();
  m_u_y = m_solver_y->getFirstControlInput();// acceleration in y-direction (double)
  m_solver_y->unlock();

  m_solver_z->lock();
  m_solver_z->setInitialState(m_initial_state_z);
  m_solver_z->loadReference(m_reference_z);
  m_solver_z->setLimits(
    m_max_speed_z, m_max_acc_z, m_max_u_z, m_max_du_z, m_dt1_z, m_dt2_z);
  m_solver_z->setDt(m_dt1_z, m_dt2_z);
  m_solver_z->setQ(m_Q_z);
  m_solver_z->setS(m_Q_last_z);
  m_solver_z->setParams();
  m_solver_z->solveCvx();
  m_u_z = m_solver_z->getFirstControlInput();// acceleration in z-direction (double)
  m_solver_z->unlock();


  // Publish desired acceleration
  geometry_msgs::Vector3 acc_msg;
  acc_msg.x = m_u_x;
  acc_msg.y = m_u_y;
  acc_msg.z = m_u_z;
  m_acc_desired_pub.publish(acc_msg);

  // 2. Use the desired acceleration to calculate the orientation

  eig_ref_orientation_Q.x() = ref_orientation_Q.x;// getting reference orientation
  eig_ref_orientation_Q.y() = ref_orientation_Q.y;// (transform to Eigen::Quaternionf)
  eig_ref_orientation_Q.z() = ref_orientation_Q.z;
  eig_ref_orientation_Q.w() = ref_orientation_Q.w;

  euler = eig_ref_orientation_Q.toRotationMatrix().eulerAngles(
    0, 1, 2);// getting reference heading (yaw)
  yaw = euler[2];

  Xc(0) = cos(yaw);// heading constraints
  Xc(1) = sin(yaw);
  Xc(2) = 0;

  a_des(0) = m_u_x;// filling in desired acceleration from the solver
  a_des(1) = m_u_y;
  a_des(2) = m_u_z;

  Eigen::Vector3f force            = UAV_mass * (Eigen::Vector3f(0, 0, 9.81) + Ra);
  Eigen::Vector3f force_normalized = force.normalized();

  // TODO: Saturate the force vector

  R_des.col(2) = force_normalized;
  R_des.col(1) = R_des.col(2).cross(Xc);
  R_des.col(1).normalize();
  R_des.col(0) = R_des.col(1).cross(R_des.col(2));

  eig_des_orientation_Q = R_des;// matrix to quaternion conversion

  if (feed_fwd_acc_flag) {
    Ra << ref_acc_x + m_u_x, ref_acc_y + m_u_y, ref_acc_z + m_u_z;
  } else {
    Ra << m_u_x, m_u_y, m_u_z;
  }

  eig_R_Q.x()  = R_Q.x;
  eig_R_Q.y()  = R_Q.y;
  eig_R_Q.z()  = R_Q.z;
  eig_R_Q.w()  = R_Q.w;
  R            = eig_R_Q.toRotationMatrix();
  thrust_force = Ra.dot(R.col(2));

  // Publish thrust force
  std_msgs::Float64 thrust_force_msg;
  thrust_force_msg.data = thrust_force;
  m_thrust_force_pub.publish(thrust_force_msg);

  // Publish scaled thrust
  std_msgs::Float64 scaled_thrust_msg;
  scaled_thrust_msg.data = thrust;
  m_scaled_thrust_pub.publish(scaled_thrust_msg);

  // --------------------------------- //
  // filling in the AttitudeTarget msg //
  // --------------------------------- //


  des_orientation_Q.x = eig_des_orientation_Q.x();// filling in geometry_msgs/Quaternion
  des_orientation_Q.y = eig_des_orientation_Q.y();
  des_orientation_Q.z = eig_des_orientation_Q.z();
  des_orientation_Q.w = eig_des_orientation_Q.w();

  static constexpr auto type_mask = mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE
                                    + mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE
                                    + mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
  attitude_target.header.stamp = ros::Time::now();
  attitude_target.type_mask    = type_mask;
  attitude_target.orientation  = des_orientation_Q;
  attitude_target.thrust       = 0.5 + thrust_force;

  return attitude_target;
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_control::ModelPredictiveControl,
                       uav_ros_control::controller_interface);
