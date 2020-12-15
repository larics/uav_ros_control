#include <uav_ros_control/control/PID.hpp>
#include <limits>
#include <iostream>

// Parameter path constants
const std::string PIDParamPrefix::KP = "/kp";
const std::string PIDParamPrefix::KI = "/ki";
const std::string PIDParamPrefix::KD = "/kd";
const std::string PIDParamPrefix::LIM_HIGH = "/lim_high";
const std::string PIDParamPrefix::LIM_LOW = "/lim_low";

PID::PID(std::string name) : _name(std::move(name))
{

  // initialize gains
  kp = 0;// proportional gain
  ki = 0;// integral gain
  kd = 0;// derivative gain

  // initialize control values
  up = 0;// P part
  ui = 0;// I part
  ui_old = 0;// I part from previous step
  ud = 0;// D part
  u = 0;// total control value
  lim_high = std::numeric_limits<double>::max();// control value upper limit
  lim_low = -std::numeric_limits<double>::max();// control value lower limit

  // init referent control value (set-value)
  ref = 0;

  // init measure control value
  meas = 0;

  // init error from the previous algorithm step
  error_old = 0;

  // flag indicates first step of the algorithm
  firstPass = true;
}

PID::PID() : PID("default") {}

void PID::resetPIDParams()
{
  // Resets pid algorithm by setting all P,I,D parts to zero
  up = 0;
  ui = 0;
  ui_old = 0;
  ud = 0;
  u = 0;
}

void PID::resetIntegrator()
{
  firstPass = false;
  ui_old = 0;
  error_old = 0;
  ROS_DEBUG("PID %s is reset.", _name.c_str());
}

void PID::set_kp(const double kp_)
{
  kp = kp_;
}

const double &PID::get_kp()
{
  return kp;
}

void PID::set_ki(const double ki_)
{
  ki = ki_;
}

const double &PID::get_ki()
{
  return ki;
}

void PID::set_kd(const double kd_)
{
  kd = kd_;
}

const double &PID::get_kd()
{
  return kd;
}

void PID::set_lim_high(const double lim_high_)
{
  lim_high = lim_high_;
}

const double &PID::get_lim_high()
{
  return lim_high;
}

void PID::set_lim_low(const double lim_low_)
{
  lim_low = lim_low_;
}

const double& PID::get_lim_low()
{
  return lim_low;
}

double PID::compute(const double ref_, const double meas_, const double dt_)
{
  double error;
  double de;
  double dt;

  ref = ref_;
  meas = meas_;
  dt = dt_;

  if (firstPass) {
    // This is the first step of the algorithm
    // Init time stamp and error
    error_old = ref - meas;
    firstPass = false;
  } else {
    error = ref - meas;
    de = error - error_old;// diff error
    up = kp * error;// proportional term

    if (ki == 0) {
      ui = 0;
    } else {
      ui = ui_old + ki * error * dt;// integral term
    }

    ud = kd * de / dt;// derivative term
    u = up + ui + ud;

    if (u > lim_high) {
      u = lim_high;
      ui = ui_old;// antiwind up
    } else if (u < lim_low) {
      u = lim_low;
      ui = ui_old;// antiwind up
    }

    ui_old = ui;// save ui for next step
    error_old = error;
  }

  return u;
}

void PID::get_pid_values(double &up_, double &ui_, double &ud_, double &u_)
{
  up_ = up;
  ui_ = ui;
  ud_ = ud;
  u_ = u;
}

void PID::create_msg(uav_ros_control_msgs::PIDController &msg)
{
  msg.ref = ref;
  msg.meas = meas;
  msg.P = up;
  msg.I = ui;
  msg.D = ud;
  msg.U = u;
  msg.header.stamp = ros::Time::now();
}

std::ostream &operator<<(std::ostream &out, const PID &pid)
{
  out << pid._name << " PID parameters are:"
      << "\nk_p=" << pid.kp << "\nk_i=" << pid.ki << "\nk_d=" << pid.kd
      << "\nlim_low=" << pid.lim_low << "\nlim_high=" << pid.lim_high << std::endl;
  return out;
}

void PID::initializeParameters(ros::NodeHandle &nh, const std::string& prefix)
{
  bool initialized = nh.getParam(prefix + PIDParamPrefix::KP, kp)
                     && nh.getParam(prefix + PIDParamPrefix::KI, ki)
                     && nh.getParam(prefix + PIDParamPrefix::KD, kd)
                     && nh.getParam(prefix + PIDParamPrefix::LIM_LOW, lim_low)
                     && nh.getParam(prefix + PIDParamPrefix::LIM_HIGH, lim_high);
  ROS_INFO_STREAM(*this);
  if (!initialized) {
    ROS_FATAL("PID() - parameter initialization failed.");
    throw std::runtime_error("PID parameters not properly set.");
  }
}