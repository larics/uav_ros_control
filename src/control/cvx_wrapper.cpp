/* author: Daniel Hert @ https://github.com/ctu-mrs/mrs_uav_controllers */

#include <eigen3/Eigen/Eigen>
#include <uav_ros_control/control/cvx_wrapper.hpp>

using namespace Eigen;

namespace uav_ros_control {

namespace cvx_wrapper {

  extern "C" {
#include "cvxgen/solver.h"
  }

  Vars vars;
  Params params;
  Workspace work;
  Settings settings;

  /* CvxWrapper() //{ */

  std::mutex CvxWrapper::mutex_main;

  CvxWrapper::CvxWrapper(bool verbose,
    int max_iters,
    std::vector<double> Q,
    std::vector<double> Q_last,
    double dt1,
    double dt2,
    double p1,
    double p2)
  {

    this->Q = Q;
    this->Q_last = Q_last;
    this->verbose = verbose;
    this->max_iters = max_iters;
    this->dt1 = dt1;
    this->dt2 = dt2;
    this->p1 = p1;
    this->p2 = p2;

    params.u_last[0] = 0;

    set_defaults();
    setup_indexing();
    setup_indexed_optvars();

    setParams();

    ROS_INFO("Cvx wrapper initiated");
  }

  //}

  /* setParams() //{ */

  void CvxWrapper::setParams(void)
  {

    settings.verbose = this->verbose;
    settings.max_iters = this->max_iters;

    for (int i = 0; i < 3; i++) { params.Q[i] = Q[i]; }

    for (int i = 0; i < 3; i++) { params.Q_last[i] = Q_last[i]; }

    params.Af[0] = 1;
    params.Af[1] = 1;
    params.Af[2] = p1;
    params.Af[3] = dt1;
    params.Af[4] = dt1;

    params.Bf[0] = p2;

    params.A[0] = 1;
    params.A[1] = 1;
    params.A[2] = p1;
    params.A[3] = dt2;
    params.A[4] = dt2;

    params.B[0] = p2;
  }

  //}

  /* setLimits() //{ */

  void CvxWrapper::setLimits(double max_speed,
    double max_acc,
    double max_u,
    double max_du,
    double dt1,
    double dt2)
  {
    params.x_max_2[0] = max_speed;
    params.x_max_3[0] = max_acc;
    params.u_max[0] = max_u;
    params.du_max_f[0] = max_du * dt1;
    params.du_max[0] = max_du * dt2;
  }

  //}

  /* setLastInput() //{ */

  void CvxWrapper::setLastInput(double last_input)
  {

    params.u_last[0] = last_input;
  }

  //}

  /* setDt() //{ */

  void CvxWrapper::setDt(double dt1, double dt2)
  {

    params.Af[2] = dt1;
    params.Af[3] = dt1;

    params.A[2] = dt2;
    params.A[3] = dt2;
  }

  //}

  /* setInitialState() //{ */

  void CvxWrapper::setInitialState(MatrixXd &x)
  {

    params.x_0[0] = x(0, 0);
    params.x_0[1] = x(1, 0);
    params.x_0[2] = x(2, 0);
  }

  //}

  /* loadReference() //{ */

  void CvxWrapper::loadReference(MatrixXd &reference)
  {

    for (int i = 0; i < horizon_len; i++) {

      params.x_ss[i + 1][0] = reference((3 * i) + 0, 0);
      params.x_ss[i + 1][1] = reference((3 * i) + 1, 0);
      params.x_ss[i + 1][2] = reference((3 * i) + 2, 0);
    }
  }

  //}

  /* setQ() //{ */

  void CvxWrapper::setQ(const std::vector<double> new_Q)
  {

    for (int i = 0; i < horizon_len; i++) { this->Q = new_Q; }
  }

  //}

  /* setS() //{ */

  void CvxWrapper::setS(const std::vector<double> new_S)
  {

    for (int i = 0; i < horizon_len; i++) { this->Q_last = new_S; }
  }

  //}

  /* solveCvx() //{ */

  int CvxWrapper::solveCvx() { return solve(); }
  //}

  /* getStates() //{ */

  void CvxWrapper::getStates(MatrixXd &future_traj)
  {

    for (int i = 0; i < horizon_len; i++) {

      future_traj(0 + (i * 3)) = *(vars.x[i + 1]);
      future_traj(1 + (i * 3)) = *(vars.x[i + 1] + 1);
      future_traj(2 + (i * 3)) = *(vars.x[i + 1] + 2);
    }
  }

  //}

  /* getFirstControlInput() //{ */

  double CvxWrapper::getFirstControlInput() { return *(vars.u_0); }

  //}

  /* lock() //{ */

  void CvxWrapper::lock(void) { this->mutex_main.lock(); }

  //}

  /* unlock() //{ */

  void CvxWrapper::unlock(void) { this->mutex_main.unlock(); }

  //}

}// namespace cvx_wrapper

}// namespace uav_ros_control