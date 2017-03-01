/**
 * \file mag.cpp
 * \author Jerel Nielsen <jerel.nielsen@gmail.com>
 */

#include <mavrosflight/sensors/mag.h>
#include <ros/ros.h>

namespace mavrosflight
{
namespace sensors
{

Mag::Mag() :
  calibrating_(false),
  calibration_time_(30.0)
{
  A_ = Eigen::MatrixXd::Zero(3, 3);
  b_ = Eigen::MatrixXd::Zero(3, 1);
}

void Mag::start_calibration()
{
  ROS_WARN("Magnetometer calibration started. Duration: %3.0f seconds.", calibration_time_);
  ROS_WARN("Turn the magnetometer +/-180 degrees along one inertial axis in 6 orthogonal orientations."); //TODO: need better description

  calibrating_ = true;

  first_time_ = true;
  start_time_ = 0;

  measurement_throttle_ = 0;
}

bool Mag::calibrate(mavlink_small_mag_t msg)
{
  if (first_time_)
  {
    first_time_ = false;
    start_time_ = ros::Time::now().toSec();
    displayed_time_ = -1;
  }

  double elapsed = ros::Time::now().toSec() - start_time_;

  // output time left
  if (((int)floor(elapsed) % 5) == 0 && floor(elapsed) != displayed_time_)
  {
    ROS_WARN("%3.0f seconds remaining", calibration_time_ - round(elapsed));
    displayed_time_ = floor(elapsed);
  }

  // if still in calibration mode
  if (elapsed < calibration_time_)
  {
    if (measurement_throttle_ > -1)
    {
      Eigen::Vector3d measurement;
      measurement << msg.xmag, msg.ymag, msg.zmag;

      // prevent bad measurements from coming in
      if (sqrt(msg.xmag * msg.xmag + msg.ymag * msg.ymag + msg.zmag * msg.zmag) < 1000)
      {
        measurements_.push_back(measurement);
      }      

      measurement_throttle_ = 0;
    }
    measurement_throttle_++;
  }
  else if (calibrating_)
  {
    // fit ellipsoid to the measurements according to Li paper
    ROS_WARN("Done collecting measurements. Fitting ellipsoid.");
    Eigen::MatrixXd u = ellipsoidLS(measurements_);

    // magnetometer calibration parameters according to Renaudin paper
    ROS_WARN("Computing calibration parameters.");
    magCal(u, A_, b_);
    calibrating_ = false;
  }

  return !calibrating_;
}

bool Mag::correct(mavlink_small_mag_t msg, double *xmag, double *ymag, double *zmag)
{
  *xmag = (double)msg.xmag;
  *ymag = (double)msg.ymag;
  *zmag = (double)msg.zmag;

  return true;
}

void Mag::eigSort(Eigen::MatrixXd &w, Eigen::MatrixXd &v)
{
    // create index array
    int idx[w.cols()];
    for (unsigned i = 0; i < w.cols(); i++)
    {
        idx[i] = i;
    }

    // do a bubble sort and keep track of where values go with the index array
    int has_changed = 1; // true
    Eigen::MatrixXd w_sort = w;
    while (has_changed == 1)
    {
        has_changed = 0; // false
        for (unsigned i = 0; i < w.cols()-1; i++)
        {
            if (w_sort(i) < w_sort(i+1))
            {
                // switch values
                double tmp = w_sort(i);
                w_sort(i) = w_sort(i+1);
                w_sort(i+1) = tmp;

                // switch indices
                tmp = idx[i];
                idx[i] = idx[i+1];
                idx[i+1] = tmp;

                has_changed = 1; // true
            }
        }
    }

    // add sorted eigenvalues/eigenvectors to output
    Eigen::MatrixXd w1 = w;
    Eigen::MatrixXd v1 = v;
    for (unsigned i = 0; i < w.cols(); i++)
    {
        w1(i) = w(idx[i]);
        v1.col(i) = v.col(idx[i]);
    }
    w = w1;
    v = v1;
}

Eigen::MatrixXd Mag::ellipsoidLS(std::deque<Eigen::Vector3d> meas)
{
    // form D matrix from eq. 6
    Eigen::MatrixXd D(10, meas.size());
    double mag_sum = 0;
    double mag_avg = 0;
    double mag_var = 0;
    for (unsigned i = 0; i < D.cols(); i++)
    {
        // unpack measurement components
        double x = meas[i](0);
        double y = meas[i](1);
        double z = meas[i](2);

        // fill in the D matrix
        D(0, i) = x * x;
        D(1, i) = y * y;
        D(2, i) = z * z;
        D(3, i) = 2 * y * z;
        D(4, i) = 2 * x * z;
        D(5, i) = 2 * x * y;
        D(6, i) = 2 * x;
        D(7, i) = 2 * y;
        D(8, i) = 2 * z;
        D(9, i) = 1;

        // compute average magnitude along the way
        double mag = sqrt(x * x + y * y + z * z);
        int t = i + 1;
        mag_avg = (t-1)/t * mag_avg + mag / t;
        if (i > 0)
        {
            mag_var = (t-1)/t * mag_var + ((mag - mag_avg) * (mag - mag_avg)) / (t-1);
        }
        mag_sum += mag;
    }
    ROS_INFO("No. of Measurementes: %lu, Distribution of Magnitude (mean, variance) = (%5.1f, %8.1f)", meas.size(), mag_sum / meas.size(), mag_var);

    // form the C1 matrix from eq. 7
    double k = 4;
    Eigen::MatrixXd C1 = Eigen::MatrixXd::Zero(6, 6);
    C1(0, 0) = -1;
    C1(0, 1) = k / 2 - 1;
    C1(0, 2) = k / 2 - 1;
    C1(1, 0) = k / 2 - 1;
    C1(1, 1) = -1;
    C1(1, 2) = k / 2 - 1;
    C1(2, 0) = k / 2 - 1;
    C1(2, 1) = k / 2 - 1;
    C1(2, 2) = -1;
    C1(3, 3) = -k;
    C1(4, 4) = -k;
    C1(5, 5) = -k;

    // decompose D*D^T according to eq. 11
    Eigen::MatrixXd DDt = D * D.transpose();
    Eigen::MatrixXd S11 = DDt.block(0, 0, 6, 6);
    Eigen::MatrixXd S12 = DDt.block(0, 6, 6, 4);
    Eigen::MatrixXd S22 = DDt.block(6, 6, 4, 4);

    // solve eigensystem in eq. 15
    Eigen::MatrixXd ES = C1.inverse() * (S11 - S12 * S22.inverse() * S12.transpose());
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(ES);
    if (eigensolver.info() != Eigen::Success)
    {
        ROS_ERROR("Eigensolver failed in ellipsoidLS");
        abort();
    }
    Eigen::MatrixXd w = eigensolver.eigenvalues().real().transpose();
    Eigen::MatrixXd V = eigensolver.eigenvectors().real();

    // sort eigenvalues and eigenvectors from most positive to most negative
    eigSort(w, V);

    // compute solution vector defined in paragraph below eq. 15
    Eigen::MatrixXd u1 = V.col(0);
    Eigen::MatrixXd u2 = -S22.inverse() * S12.transpose() * u1;
    Eigen::MatrixXd u(10, 1);
    u.block(0, 0, 6, 1) = u1;
    u.block(6, 0, 4, 1) = u2;

    return u;
}

void Mag::magCal(Eigen::MatrixXd u, Eigen::MatrixXd &A, Eigen::MatrixXd &bb)
{
    // unpack coefficients
    double a = u(0);
    double b = u(1);
    double c = u(2);
    double f = u(3);
    double g = u(4);
    double h = u(5);
    double p = u(6);
    double q = u(7);
    double r = u(8);
    double d = u(9);

    // compute Q, u, and k according to eq. 15 of Renaudin and eqs. 1 and 4 of Li
    Eigen::MatrixXd Q(3, 3);
    Q << a, h, g,
         h, b, f,
         g, f, c;

    Eigen::MatrixXd ub(3, 1);
    ub << 2 * p,
          2 * q,
          2 * r;
    double k = d;

    // extract bb according to eq. 21 of Renaudin (should be negative according to eq. 16)
    bb = -0.5 * Q.inverse() * ub;

    // eigendecomposition of Q according to eq. 22 of Renaudin
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(Q);
    if (eigensolver.info() != Eigen::Success)
    {
        ROS_ERROR("Eigensolver failed in magCal.");
        abort();
    }
    Eigen::MatrixXd D = eigensolver.eigenvalues().real().asDiagonal();
    Eigen::MatrixXd V = eigensolver.eigenvectors().real();

    // compute alpha according to eq. 27 of Renaudin (the denominator needs to be multiplied by -1)
    // double Hm = 51503.9; // (nT) Provo, UT magnetic field magnitude
    double Hm = 300;
    Eigen::MatrixXd utVDiVtu = ub.transpose() * V * D.inverse() * V.transpose() * ub;
    double alpha = (4. * Hm * Hm) / (utVDiVtu(0) - 4 * k);

    // now compute A from eq. 8 and 28 of Renaudin
    A = V * (alpha * D).cwiseSqrt() * V.transpose();
}

} // namespace sensors
} // namespace mavrosflight
