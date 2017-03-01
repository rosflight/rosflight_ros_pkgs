/**
 * \file mag.h
 * \author Jerel Nielsen <jerel.nielsen@gmail.com>
 */

#ifndef MAVROSFLIGHT_SENSORS_MAG_H
#define MAVROSFLIGHT_SENSORS_MAG_H

#include <mavrosflight/mavlink_bridge.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <deque>

namespace mavrosflight
{
namespace sensors
{

/**
 * \brief MAG sensor class
 */
class Mag
{
public:

  Mag();

  /**
   * \brief Begin the temperature calibration routine
   */
  void start_calibration();

  /**
   * \brief Check if a calibration is in progress
   * \return True if a calibration is currently in progress
   */
  bool is_calibrating() { return calibrating_; }

  /**
   * \brief Calibrate the magnetometer for hard and soft iron effects
   * \param msg The raw MagneticField message
   * \return True if the calibration is done
   */
  bool calibrate(mavlink_small_mag_t msg);

  /**
   * \brief Get corrected measurement values
   * \param msg The raw MagneticField message
   * \param[out] xmag The magnetometer X value (Tesla)
   * \param[out] ymag The magnetometer Y value (Tesla)
   * \param[out] zmag The magnetometer Z value (Tesla)
   * \return True if the measurement is valid
   */
  bool correct(mavlink_small_mag_t msg, double *xmag, double *ymag, double *zmag);

  /*
      sort eigenvalues and eigenvectors output from Eigen library
  */
  void eigSort(Eigen::MatrixXd &w, Eigen::MatrixXd &v);

  /*
      This function gets ellipsoid parameters via least squares on ellipsoidal data
      according to the paper, "Least squares ellipsoid specific fitting" by Li.
  */
  Eigen::MatrixXd ellipsoidLS(std::deque<Eigen::Vector3d> meas);

  /*
      This function computes magnetometer calibration parameters according to Section 5.3 of the
      paper, "Complete Triaxis Magnetometer Calibration in the Magnetic Domain" by Renaudin et al.
  */
  void magCal(Eigen::MatrixXd u, Eigen::MatrixXd &A, Eigen::MatrixXd &bb);

  /// These are the publicly available versions of the mag calibration
  /// The const stuff is to make it read-only
  const double a11() const { return A_(0, 0); }
  const double a12() const { return A_(0, 1); }
  const double a13() const { return A_(0, 2); }
  const double a21() const { return A_(1, 0); }
  const double a22() const { return A_(1, 1); }
  const double a23() const { return A_(1, 2); }
  const double a31() const { return A_(2, 0); }
  const double a32() const { return A_(2, 1); }
  const double a33() const { return A_(2, 2); }
  const double bx() const { return b_(0, 0); }
  const double by() const { return b_(1, 0); }
  const double bz() const { return b_(2, 0); }

private:
  Eigen::MatrixXd A_, b_;

  bool calibrating_; //!< whether a temperature calibration is in progress
  double calibration_time_; //!< seconds to record data for temperature compensation
  bool first_time_; //!< waiting for first measurement for calibration
  double start_time_; //!< timestamp of first calibration measurement
  double displayed_time_; //!< calibration time left already displayed
  int measurement_throttle_;
  std::deque<Eigen::Vector3d> measurements_;
};

} // namespace sensors
} // namespace mavrosflight

#endif // MAVROSFLIGHT_SENSORS_MAG_H
