/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file mag.cpp
 * \author Jerel Nielsen <jerel.nielsen@gmail.com>
 */

#include <rosflight/mavrosflight/sensors/mag.h>
#include <ros/ros.h>

namespace mavrosflight
{
namespace sensors
{

Mag::Mag() :
  calibrating_(false),
  calibration_time_(30.0),
  reference_field_strength_(1.0)
{
  A_ = Eigen::MatrixXd::Zero(3, 3);
  b_ = Eigen::MatrixXd::Zero(3, 1);
  ransac_iters_ = 100;
  inlier_thresh_ = 200;
}

void Mag::set_refence_magnetic_field_strength(double reference_magnetic_field)
{
  reference_field_strength_ = reference_magnetic_field;
}

void Mag::start_calibration()
{
  ROS_WARN("Magnetometer calibration started. Duration: %3.0f seconds.", calibration_time_);
  ROS_WARN("Turn the magnetometer +/-180 degrees along one inertial axis in 6 orthogonal orientations."); //TODO: need better description

  calibrating_ = true;

  first_time_ = true;
  start_time_ = 0;

  measurement_prev_ = Eigen::Vector3d::Zero();
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
    ROS_INFO("%3.0f seconds remaining", calibration_time_ - round(elapsed));
    displayed_time_ = floor(elapsed);
  }

  // if still in calibration mode
  if (elapsed < calibration_time_)
  {
      Eigen::Vector3d measurement;
      measurement << msg.xmag, msg.ymag, msg.zmag;

      // prevent duplicate measurements from coming in
      if (measurement != measurement_prev_)
      {
        measurements_.push_back(measurement);
      }
      measurement_prev_ = measurement;
  }
  else if (calibrating_)
  {
    // fit ellipsoid to the measurements according to Li paper but in a RANSAC form
    ROS_INFO("Collected %lu measurements. Fitting ellipsoid.", measurements_.size());
    Eigen::MatrixXd u = ellipsoidRANSAC(measurements_, ransac_iters_, inlier_thresh_);

    // magnetometer calibration parameters according to Renaudin paper
    ROS_INFO("Computing calibration parameters.");
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

Eigen::MatrixXd Mag::ellipsoidRANSAC(std::deque<Eigen::Vector3d> meas, int iters, double inlier_thresh)
{
    // initialize random number generator
    std::random_device random_dev;
    std::mt19937 generator(random_dev());

    // RANSAC to find a good ellipsoid fit
    int inlier_count_best = 0; // number of inliers for best fit
    std::deque<Eigen::Vector3d> inliers_best; // container for inliers to best fit
    double dist_sum = 0; // sum distances of all measurements from ellipsoid surface
    int dist_count = 0; // count number distances of all measurements from ellipsoid surface
    for (unsigned i = 0; i < iters; i++)
    {
        // pick 9 random, unique measurements by shuffling measurements
        // and grabbing the first 9
        std::shuffle(meas.begin(), meas.end(), generator);
        std::deque<Eigen::Vector3d> meas_sample;
        for (unsigned j = 0; j < 9; j++)
        {
            meas_sample.push_back(meas[j]);
        }

        // fit ellipsoid to 9 random points
        Eigen::MatrixXd u = ellipsoidLS(meas_sample);

        // check if LS fit is actually an ellipsoid (paragraph of Li following eq. 2-4)
        // unpack needed parts of u
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

        double I = a + b + c;
        double J = a * b + b * c + a *c - f * f - g * g - h * h;

        if (4 * J - I * I <= 0)
        {
            continue;
        }

        // eq. 15 of Renaudin and eqs. 1 and 4 of Li
        Eigen::MatrixXd Q(3, 3);
        Q << a, h, g,
             h, b, f,
             g, f, c;

        Eigen::MatrixXd ub(3, 1);
        ub << 2 * p,
              2 * q,
              2 * r;
        double k = d;

        // eq. 21 of Renaudin (should be negative according to eq. 16)
        // this is the vector to the ellipsoid center
        Eigen::MatrixXd bb = -0.5 * Q.inverse() * ub;
        Eigen::Vector3d r_e; r_e << bb(0), bb(1), bb(2);

        // count inliers and store inliers
        int inlier_count = 0;
        std::deque<Eigen::Vector3d> inliers;
        for (unsigned j = 0; j < meas.size(); j++)
        {
            // compute the vector from ellipsoid center to surface along
            // measurement vector and a one from the perturbed measurement
            Eigen::Vector3d perturb = Eigen::Vector3d::Ones() * 0.1;
            Eigen::Vector3d r_int = intersect(meas[j], r_e, Q, ub, k);
            Eigen::Vector3d r_int_prime = intersect(meas[j] + perturb, r_e, Q, ub, k);

            // now compute the vector normal to the surface
            Eigen::Vector3d r_align = r_int_prime - r_int;
            Eigen::Vector3d r_normal = r_align.cross(r_int.cross(r_int_prime));
            Eigen::Vector3d i_normal = r_normal / r_normal.norm();

            // get vector from surface to measurement and take dot product
            // with surface normal vector to find distance from ellipsoid fit
            Eigen::Vector3d r_sm = meas[j] - r_e - r_int;
            double dist = r_sm.dot(i_normal);
            dist_sum += dist;
            dist_count++;

            // check measurement distance against inlier threshold
            if (fabs(dist) < inlier_thresh)
            {
                inliers.push_back(meas[j]);
                inlier_count++;
            }
        }

        // save best inliers and their count
        if (inlier_count > inlier_count_best)
        {
            inliers_best.clear();
            for (unsigned j = 0; j < inliers.size(); j++)
            {
                inliers_best.push_back(inliers[j]);
            }
            inlier_count_best = inlier_count;
        }
    }

    // check average measurement distance from surface
    double dist_avg = dist_sum / dist_count;
    if (inlier_thresh > fabs(dist_avg))
    {
      ROS_WARN("Inlier threshold is greater than average measurement distance from surface. Reduce inlier threshold.");
      ROS_INFO("Inlier threshold = %7.1f, Average measurement distance = %7.1f", inlier_thresh_, dist_avg);
    }

    // perform LS on set of best inliers
    Eigen::MatrixXd u_final = ellipsoidLS(inliers_best);

    return u_final;
}

Eigen::Vector3d Mag::intersect(Eigen::Vector3d r_m, Eigen::Vector3d r_e, Eigen::MatrixXd Q, Eigen::MatrixXd ub, double k)
{
    // form unit vector from ellipsoid center (r_e) pointing to measurement
    Eigen::Vector3d r_em = r_m - r_e;
    Eigen::Vector3d i_em = r_em / r_em.norm();

    // solve quadratic equation for alpha, which determines how much to scale
    // i_em to intersect the ellipsoid surface (this is in Jerel's notebook)
    double A = (i_em.transpose() * Q * i_em)(0);
    double B = (2 * (i_em.transpose() * Q * r_e + ub.transpose() * i_em))(0);
    double C = (ub.transpose() * r_e + r_e.transpose() * Q * r_e)(0) + k;
    double alpha = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);

    // compute vector from ellipsoid center to its surface along measurement vector
    Eigen::Vector3d r_int = r_e + alpha * i_em;

    return r_int;
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

/*
    This function gets ellipsoid parameters via least squares on ellipsoidal data
    according to the paper: Li, Qingde, and John G. Griffiths. "Least squares ellipsoid
    specific fitting." Geometric modeling and processing, 2004. proceedings. IEEE, 2004.
*/
Eigen::MatrixXd Mag::ellipsoidLS(std::deque<Eigen::Vector3d> meas)
{
    // form D matrix from eq. 6
    Eigen::MatrixXd D(10, meas.size());
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
    }

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

/*
    This function compute magnetometer calibration parameters according to Section 5.3 of the
    paper: Renaudin, Valérie, Muhammad Haris Afzal, and Gérard Lachapelle. "Complete triaxis
    magnetometer calibration in the magnetic domain." Journal of sensors 2010 (2010).
*/
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
        abort();
    }
    Eigen::MatrixXd D = eigensolver.eigenvalues().real().asDiagonal();
    Eigen::MatrixXd V = eigensolver.eigenvectors().real();

    // compute alpha according to eq. 27 of Renaudin (the denominator needs to be multiplied by -1)
    double Hm = reference_field_strength_; // (uT) Provo, UT magnetic field magnitude
    Eigen::MatrixXd utVDiVtu = ub.transpose() * V * D.inverse() * V.transpose() * ub;
    double alpha = (4. * Hm * Hm) / (utVDiVtu(0) - 4 * k);

    // now compute A from eq. 8 and 28 of Renaudin
    A = V * (alpha * D).cwiseSqrt() * V.transpose();
}

} // namespace sensors
} // namespace mavrosflight
