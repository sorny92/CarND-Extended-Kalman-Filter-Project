#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    cout << measurement_pack.raw_measurements_ << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.Init();
    ekf_.x_ << 1, 1, 1, 1;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  long noise_ax = 9;
  long noise_ay = 9;
  //Convert to seconds
  float time_elapsed = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float time_elapsed_2 = time_elapsed*time_elapsed;
  float time_elapsed_3 = time_elapsed_2*time_elapsed;
  float time_elapsed_4 = time_elapsed_2*time_elapsed_2;

  ekf_.Q_ << (time_elapsed_4/4)*noise_ax, 0, (time_elapsed_3/2)*noise_ax, 0,
            0, (time_elapsed_4/4)*noise_ay, 0, (time_elapsed_3/2)*noise_ay,
            (time_elapsed_3/2)*noise_ax, 0, time_elapsed_2*noise_ax, 0,
            0, (time_elapsed_3/2)*noise_ay, 0, time_elapsed_2*noise_ay;

  ekf_.F_ << 1, 0, time_elapsed, 0,
             0, 1, 0, time_elapsed,
             0, 0, 1, 0,
             0, 0, 0, 1;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}
