#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  use_laser_ = true;
  use_radar_ = true;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0,    0,
             0, 1, 0,    0,
             0, 0, 1000, 0,
             0, 0, 0,    1000;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      while (phi > M_PI) {
        phi -= 2.0 * M_PI;
      }
      while (phi < -M_PI) {
        phi += 2.0 * M_PI;
      }

      float x  = rho * cos(phi);
      float y  = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);

      ekf_.x_ << x, y, vx, vy;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      float x = measurement_pack.raw_measurements_[0];
      float y = measurement_pack.raw_measurements_[1];
      ekf_.x_ << x, y, 0, 0;
    }

    const float threshold = 0.0001;
    if (fabs(ekf_.x_(0)) < threshold && fabs(ekf_.x_(1)) < threshold) {
      ekf_.x_(0) = threshold;
      ekf_.x_(1) = threshold;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  float dt = (measurement_pack.timestamp_ - previous_timestamp_);
  dt /= 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1,  0,  dt, 0,
             0,  1,  0,  dt,
             0,  0,  1,  0,
             0,  0,  0,  1;
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  float dt_2   = dt * dt;
  float dt_3   = dt_2 * dt;
  float dt_4   = dt_3 * dt;
  float dt_4_4 = dt_4 / 4;
  float dt_3_2 = dt_3 / 2;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_4*noise_ax, 0,               dt_3_2*noise_ax, 0,
             0,               dt_4_4*noise_ay, 0,               dt_3_2*noise_ay,
             dt_3_2*noise_ax, 0,               dt_2*noise_ax,   0,
             0,               dt_3_2*noise_ay, 0,               dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
