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
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = measurement_pack.raw_measurements_[0];
  	  double phi = measurement_pack.raw_measurements_[1];
  	  double rho_dot = measurement_pack.raw_measurements_[2];
  	  double x = rho * cos(phi);
      if (x < 0.0001) {
        x = 0.0001;
      }
  	  double y = rho * sin(phi);
      if (y < 0.0001) {
        y = 0.0001;
      }
  	  double vx = rho_dot * cos(phi);
  	  double vy = rho_dot * sin(phi);
      ekf_.x_ << x, y, vx , vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_ ;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  double noise_ax = 9.0;
  double noise_ay = 9.0;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  double dt_4_4 = dt_4 / 4;
  double dt_3_2 = dt_3 / 2;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
	         0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
	         dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
 	         0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.R_ = R_radar_;
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);
  }

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
