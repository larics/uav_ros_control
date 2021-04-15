#ifndef MODEL_PREDICTIVE_CONTROL_HPP
#define MODEL_PREDICTIVE_CONTROL_HPP

#include <uav_ros_control/control/controller_interface.hpp>
#include <uav_ros_control/control/cvx_wrapper.hpp>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
namespace uav_ros_control {
class ModelPredictiveControl : public controller_interface
{
public:
  
  /**
   * @brief Construct a new Model Predictive Control object.
   * 
   */
  ModelPredictiveControl();

  /* Override all pure virtual methods */
  void initialize(ros::NodeHandle &parent_nh,
    const std::string name,
    const double uav_mass) override;
  
  bool activate(const mavros_msgs::AttitudeTargetConstPtr &last_attitude_cmd) override;
  
  void deactivate() override;
  
  const mavros_msgs::AttitudeTarget update(const nav_msgs::OdometryConstPtr &uav_state,
    const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &last_position_cmd)
    override;

private:
  /* A space for private variables and methods */

  // Solver ptrs
  std::unique_ptr<uav_ros_control::cvx_wrapper::CvxWrapper> m_solver_x;
  std::unique_ptr<uav_ros_control::cvx_wrapper::CvxWrapper> m_solver_y;
  std::unique_ptr<uav_ros_control::cvx_wrapper::CvxWrapper> m_solver_z;

  bool m_is_active = false;

  // dodavati stvari ovdje

  // constructor variables
  bool m_verbose_x, m_verbose_y, m_verbose_z;
  int m_max_iters_x, m_max_iters_y, m_max_iters_z;
  std::vector<double> m_Q_x, m_Q_y, m_Q_z, m_Q_last_x, m_Q_last_y, m_Q_last_z;
  double m_dt1_x, m_dt2_x, m_p1_x, m_p2_x, m_dt1_y, m_dt2_y, m_p1_y, m_p2_y, m_dt1_z, m_dt2_z, m_p1_z, m_p2_z;

  // solver variables (states and constraints)
  Eigen::MatrixXd m_initial_state_x, m_reference_x;
  double m_max_speed_x, m_max_acc_x, m_max_u_x, m_max_du_x, m_u_x;
  Eigen::MatrixXd m_initial_state_y, m_reference_y;
  double m_max_speed_y, m_max_acc_y, m_max_u_y, m_max_du_y, m_u_y;
  Eigen::MatrixXd m_initial_state_z, m_reference_z;
  double m_max_speed_z, m_max_acc_z, m_max_u_z, m_max_du_z, m_u_z;


  // messages from Odometry and Trajectory
  nav_msgs::Odometry initial_state;
  trajectory_msgs::MultiDOFJointTrajectoryPoint reference;

  double odom_pos_x, odom_pos_y, odom_pos_z;
  double odom_spd_x, odom_spd_y, odom_spd_z;
  double odom_acc_x, odom_acc_y, odom_acc_z;
  double ref_pos_x, ref_pos_y, ref_pos_z;
  double ref_spd_x, ref_spd_y, ref_spd_z;
  double ref_acc_x, ref_acc_y, ref_acc_z;








};
}// namespace uav_ros_control

#endif /* MODEL_PREDICTIVE_CONTROL_HPP */