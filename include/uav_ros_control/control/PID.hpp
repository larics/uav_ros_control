#ifndef PID_H
#define PID_H

#include <uav_ros_control_msgs/PIDController.h>
#include <ros/ros.h>
#include <string>

struct PIDParamPrefix
{
  static const std::string KP;
  static const std::string KD;
  static const std::string KI;
  static const std::string LIM_LOW;
  static const std::string LIM_HIGH;
};

class PID
{
private:
  double kp, ki, kd, up, ui, ui_old, ud, u;
  double lim_high, lim_low, ref, meas, error_old;

  bool firstPass;
  std::string _name;

public:
  /**
   * @brief Initializes PID gains (proportional - kp, integral - ki, derivative - kd) and
   *  control values to zero.
   *
   */
  PID();

  /**
   * @brief Construct a new named PID object.
   *
   * @param name Name of the PID object.
   */
  explicit PID(std::string name);
  void resetPIDParams();
  void resetIntegrator();

  /**
   * @brief Set the kp value
   * 
   * @param kp_ New kp value.
   */
  void set_kp(double kp_);

  /**
   * @brief Set the ki value.
   * 
   * @param ki_ New ki value.
   */
  void set_ki(double ki_);

  /**
   * @brief Set the kd value.
   * 
   * @param kd_ New kd value.
   */
  void set_kd(double kd_);

  /**
   * @brief Set the lim high value.
   * 
   * @param lim_high_ New lim_high value.
   */
  void set_lim_high(double lim_high_);

  /**
   * @brief Set the lim low value.
   * 
   * @param lim_low_ New lim_low value.
   */
  void set_lim_low(double lim_low_);

  /**
   * @brief Get the kp value.
   * 
   * @return const double& 
   */
  const double &get_kp();

  /**
   * @brief Get the ki value.
   * 
   * @return const double& 
   */
  const double &get_ki();

  /**
   * @brief Get the kd value
   *
   * @return const double&
   */
  const double &get_kd();

  /**
   * @brief Get the lim high value.
   *
   * @return const double&
   */
  const double &get_lim_high();

  /**
   * @brief Get the lim low value.
   *
   * @return const double&
   */
  const double &get_lim_low();

  /**
   * @brief Returns P, I, D components and total control value
   *
   * @param up_ Proportuinal control value.
   * @param ui_ Integral control value.
   * @param ud_ Derivative control value.
   * @param u_ Total control value.
   */
  void get_pid_values(double &up_, double &ui_, double &ud_, double &u_);

  /**
   * @brief Returns ros message of type PIDController
   *
   * @param msg
   */
  void create_msg(uav_ros_control_msgs::PIDController &msg);

  /**
   * @brief Performs a PID computation and returns a control value based on
   * the elapsed time (dt) and the error signal.
   *
   * @param ref_  referent value
   * @param meas_ measured value
   * @param dt_ control value
   * @return Returns the total control value
   */
  double compute(double ref_, double meas_, double dt_);

  /**
   * Initialize PID parameters.
   *
   * @param nh - Node handle parameter
   * @param prefix - string prefix for parameter getting
   */
  void initializeParameters(ros::NodeHandle &nh, const std::string &prefix);
  friend std::ostream &operator<<(std::ostream &, const PID &);
};

#endif