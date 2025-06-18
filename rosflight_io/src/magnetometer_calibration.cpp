#include "rosflight_io/magnetometer_calibration.hpp"

namespace rosflight_io {

Eigen::VectorXd MagnetometerCalibrator::ellipsoid_least_squares(Eigen::MatrixXd data){
  Eigen::VectorXd x = data.col(0);
  Eigen::VectorXd y = data.col(1);
  Eigen::VectorXd z = data.col(2);

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(x.rows(),10);
  A.col(0) = x.cwiseProduct(x);
  A.col(1) = 2*x.cwiseProduct(y);
  A.col(2) = 2*x.cwiseProduct(z);
  A.col(3) = y.cwiseProduct(y);
  A.col(4) = 2*y.cwiseProduct(z);
  A.col(5) = z.cwiseProduct(z);
  A.col(6) = x;
  A.col(7) = y;
  A.col(8) = z;
  A.col(9) = Eigen::VectorXd::Ones(x.rows());
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::FullPivHouseholderQRPreconditioner | Eigen::ComputeFullU | Eigen::ComputeFullV);
  
  Eigen::MatrixXd V = svd.matrixV();

  return V.col(9);
}
  
MagnetometerCalibrator::MagnetometerCalibrator(float accel_orientation_threshold, int consecutive_orientation_threshold, float lpf_alpha) 
{
  accel_orientation_threshold_ = accel_orientation_threshold;
  consecutive_orientation_threshold_ = consecutive_orientation_threshold;
  current_state_ = CalibrationState::REORIENTING;
  mag_calibration_data_.resize(0,3);
  consecutive_orientation_ = 0;
  lpf_alpha_ = lpf_alpha;
  lpf_accels_ = Eigen::Vector3f::Zero();
}

void MagnetometerCalibrator::update_accel(Eigen::Vector3f accel){
  lpf_accels_ = lpf_accels_*lpf_alpha_ + accel*(1-lpf_alpha_);
}
  
Eigen::Matrix3d MagnetometerCalibrator::get_soft_iron_correction() 
{
  return soft_iron_correction_;
}

Eigen::Vector3d MagnetometerCalibrator::get_hard_iron_offset() 
{
  return hard_iron_offset_;
}

bool MagnetometerCalibrator::calibrating()
{
  if (current_state_ == CalibrationState::REORIENTING) {
    find_orientation();
    return true;
  }
  else if (current_state_ == CalibrationState::CHECKING) {
    check_orientation_data();
    return true;
  }
  else if (current_state_ == CalibrationState::ALL_ORIENTATIONS_GATHERED) {
    calibrate();
  }

  return false;
}

void MagnetometerCalibrator::calibrate()
{
    auto coeffs = ellipsoid_least_squares(mag_calibration_data_);

    Eigen::Matrix3d correction_matrix;
    correction_matrix << coeffs(0), coeffs(1), coeffs(2),
                         coeffs(1), coeffs(3), coeffs(4),
                         coeffs(2), coeffs(4), coeffs(5);

    float determinant = correction_matrix.determinant();

    if (determinant < 0) {
      correction_matrix = -correction_matrix;
      coeffs = -coeffs;
      determinant = -determinant;
    }

    Eigen::Vector3d offset_terms;
    offset_terms << coeffs(6), coeffs(7), coeffs(8);

    hard_iron_offset_ = -0.5*(correction_matrix.inverse()*offset_terms);

    float third_root_determinant = std::cbrt(determinant); // Scaling term. Adjusts the result close to the unit sphere.
    soft_iron_correction_ = (correction_matrix/third_root_determinant).sqrt();
}

void MagnetometerCalibrator::check_orientation_data()
{
  unsigned long rotation_complete_check_interval = 200; // Check every ~2 seconds.
  
  if (((mag_calibration_data_.rows() - start_of_current_orientation_data_) / (num_times_data_checked_ * rotation_complete_check_interval)) >= 1 && !all_orientation_data_gathered()) {

    num_times_data_checked_ += 1;

    Eigen::MatrixXd current_orientation_data = mag_calibration_data_.bottomRows(mag_calibration_data_.rows() - start_of_current_orientation_data_);
    // Calculate the centroid.
    Eigen::RowVector3d centroid = current_orientation_data.colwise().mean();

    // Subtact centroid, because it is sampled from a convex hull, the origin is now interior to the ellipse.
    Eigen::MatrixXd centered_data = current_orientation_data.rowwise() - centroid;

    // Bin the data since the orientation into 10 bins, must be at least 3 bins.
    int num_bins = 10;
    std::vector<Eigen::Vector3d> binned_values;

    int bin_index_start = 0;

    int length_of_bin = int(current_orientation_data.rows()/ num_bins);
    
    for (int i = 0; i < num_bins; i++) {
      binned_values.push_back(centered_data.middleRows(bin_index_start, length_of_bin).colwise().mean());
      bin_index_start += length_of_bin;
    }

    // Define a plane and project binned values onto it.

    // Grab 2 random binned values, use the first 2 of the shuffled vector.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<Eigen::Vector3d> shuffled_binned_values = binned_values;
    std::shuffle(shuffled_binned_values.begin(),shuffled_binned_values.end(), gen);

    // Define the normal vector.
    Eigen::Vector3d point_1 = shuffled_binned_values[0];
    Eigen::Vector3d point_2 = shuffled_binned_values[1];

    Eigen::Vector3d plane_basis_x = point_1; 
    plane_basis_x.normalize();

    Eigen::Vector3d normal = (plane_basis_x).cross(point_2);
    normal.normalize();

    Eigen::Vector3d plane_basis_y = plane_basis_x.cross(normal);

    // Project the binned values onto the plane. This is just in case the user doesn't rotate the vehile in a flat manner.
    // (turns pringles into ellipses.)
    // Use atan2 to find the angle.
    float bin_widths = 2*M_PI/num_bins;
    
    std::vector<int> angle_counts(num_bins, 0);
    for (int i = 0; i < centered_data.rows(); i++) {
      Eigen::Vector3d point = centered_data.row(i);
      Eigen::Vector3d projected_point = point - point.dot(normal)*normal;

      float x_coord_2D = plane_basis_x.dot(projected_point);
      float y_coord_2D = plane_basis_y.dot(projected_point);
      float angle = std::atan2(y_coord_2D, x_coord_2D);

      int bin = int((angle+M_PI)/bin_widths);
      if (bin < 0) {
        bin = 0;
      }
      else if (bin >= num_bins) {
        bin = num_bins - 1;
      }
      angle_counts[bin]++;
    }

    int min_number_of_points_in_bin = 100;

    if (*std::min_element(angle_counts.begin(),angle_counts.end()) > min_number_of_points_in_bin) {
      completed_orientations_.insert(current_orientation_);
      if (all_orientation_data_gathered()) {
        current_state_ = CalibrationState::ALL_ORIENTATIONS_GATHERED;
        feedback_ = "ALL ORIENTATIONS COVERED, PERFORMING CALIBRATION.";
      }
      else {
        current_state_ = CalibrationState::REORIENTING;
        feedback_ = "ENOUGH DATA GATHERED FOR THIS ORIENTATION, REORIENT.";
      }
    }
    else {
      feedback_ = "ADDITIONAL DATA REQUIRED, BE SURE TO COVER A FULL CIRCLE.";
    }
  }
}

bool MagnetometerCalibrator::all_orientation_data_gathered() {
  return completed_orientations_.size() == static_cast<int>(Orientation::NUM_ORIENTATIONS);
}

void MagnetometerCalibrator::find_orientation()
{
  float accel_x = lpf_accels_(0);
  float accel_y = lpf_accels_(1);
  float accel_z = lpf_accels_(2);

  Orientation prev_orientation = current_orientation_;
  
  if (abs(accel_x - 9.81) < accel_orientation_threshold_) {
    // Body x face down.
    current_orientation_ = Orientation::X_DOWN;
  }
  else if (abs(accel_x + 9.81) < accel_orientation_threshold_) {
    // Body x face up.
    current_orientation_ = Orientation::X_UP;
  }
  else if (abs(accel_y - 9.81) < accel_orientation_threshold_) {
    // Body y face down.
    current_orientation_ = Orientation::Y_DOWN;
  }
  else if (abs(accel_y + 9.81) < accel_orientation_threshold_) {
    // Body y face up.
    current_orientation_ = Orientation::Y_UP;
  }
  else if (abs(accel_z - 9.81) < accel_orientation_threshold_) {
    // Body z face down.
    current_orientation_ = Orientation::Z_DOWN;
  }
  else if (abs(accel_z + 9.81) < accel_orientation_threshold_) {
    // Body z face up.
    current_orientation_ = Orientation::Z_UP;
  }

  if (prev_orientation == current_orientation_) {
    consecutive_orientation_++;
  }
  else {
    consecutive_orientation_ = 0;
  }
  
  if (consecutive_orientation_ >= consecutive_orientation_threshold_ && !orientation_covered(current_orientation_)) {
    feedback_ = "ORIENTATION FIXED. BEGIN ROTATIONS.";
    start_of_current_orientation_data_ = mag_calibration_data_.rows() - 1;
    current_state_ = CalibrationState::CHECKING;
    num_times_data_checked_ = 1;
  }
  else if (orientation_covered(current_orientation_)){
    feedback_ = "THIS ORIENTATION WAS ALREADY COMPLETED.";
    return;
  }
  else {
    feedback_ = "NO ORIENTATION FOUND. HOLD IN DESIRED ORIENTATION FOR ~2 SEC.";
    return;
  }

}

bool MagnetometerCalibrator::orientation_covered(Orientation orientation) 
{
  return completed_orientations_.find(orientation) != completed_orientations_.end();
}

std::string MagnetometerCalibrator::feedback(){return feedback_;}

void MagnetometerCalibrator::update_mag(double mag_x, double mag_y, double mag_z) 
{
    Eigen::RowVector3d new_row;
    new_row << mag_x, mag_y, mag_z;

    mag_calibration_data_.conservativeResize(mag_calibration_data_.rows()+1, Eigen::NoChange);
    mag_calibration_data_.row(mag_calibration_data_.rows() - 1) = new_row;
}

} // end namespace rosfligt_io
  
