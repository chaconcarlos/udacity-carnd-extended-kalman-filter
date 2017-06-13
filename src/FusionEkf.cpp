/* INCLUDES ******************************************************************/

#include "FusionEkf.h"

#include "Eigen/Dense"
#include <iostream>
#include <cmath>
#include "Tools.h"

/* USINGS ********************************************************************/

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* CLASS IMPLEMENTATION ******************************************************/

namespace Sdce
{

FusionEkf::FusionEkf()
{
  // initializing matrices
  isInitialized     = false;
  previousTimestamp = 0;
  R_laser           = MatrixXd(2, 2);
  R_radar           = MatrixXd(3, 3);
  H_laser           = MatrixXd(2, 4);
  Hj                = MatrixXd(3, 4);
  noise_ax          = 9;
  noise_ay          = 9;

  //measurement covariance matrix - laser
  R_laser << 0.0225,      0,
                  0, 0.0225;

  //measurement covariance matrix - radar
  R_radar << 0.09,      0,    0,
                0, 0.0009,    0,
                0,      0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //state covariance matrix P
  kalmanFilter.P_ = MatrixXd(4, 4);
  kalmanFilter.P_ << 1, 0,    0,    0,
                     0, 1,    0,    0,
                     0, 0, 1000,    0,
                     0, 0,    0, 1000;

  kalmanFilter.F_ = MatrixXd(4, 4);
  kalmanFilter.F_ << 1, 0, 1, 0,
                     0, 1, 0, 1,
                     0, 0, 1, 0,
                     0, 0, 0, 1;

  //measurement matrix
  H_laser = MatrixXd(2, 4);
  H_laser << 1, 0, 0, 0,
             0, 1, 0, 0;
}

FusionEkf::~FusionEkf()
{
}

KalmanFilter&
FusionEkf::getKalmanFilter()
{
  return kalmanFilter;
}

void
FusionEkf::ProcessMeasurement(const MeasurementPackage& package)
{
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (isInitialized == false)
  {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    kalmanFilter.Q_ = MatrixXd(4, 4);
    kalmanFilter.x_ = VectorXd(4);
    kalmanFilter.x_ << 1, 1, 1, 1;

    if (package.sensorType == SENSOR_TYPE_RADAR)
    {
      const float x = package.rawMeasurements[0] * std::cos(package.rawMeasurements[1]);
      const float y = package.rawMeasurements[0] * std::sin(package.rawMeasurements[1]);

      kalmanFilter.x_ << x, y, 0, 0;
    }
    else if (package.sensorType == SENSOR_TYPE_LASER)
    {
      kalmanFilter.x_ << package.rawMeasurements[0], package.rawMeasurements[1], 0, 0;
    }

    // done initializing, no need to predict or update
    previousTimestamp = package.timestamp;
    isInitialized     = true;

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
  float dt   = (package.timestamp - previousTimestamp) / 1000000.0; //dt - expressed in seconds
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  previousTimestamp = package.timestamp;

  //Modify the F matrix so that the time is integrated
  kalmanFilter.F_(0, 2) = dt;
  kalmanFilter.F_(1, 3) = dt;

  //set the process covariance matrix Q
  kalmanFilter.Q_ = MatrixXd(4, 4);
  kalmanFilter.Q_ <<  dt_4/4*noise_ax,               0, dt_3/2*noise_ax,               0,
                                    0, dt_4/4*noise_ay,               0, dt_3/2*noise_ay,
                      dt_3/2*noise_ax,               0,   dt_2*noise_ax,               0,
                                    0, dt_3/2*noise_ay,               0,   dt_2*noise_ay;

  if (dt > 0.0001)
    kalmanFilter.predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (package.sensorType == SENSOR_TYPE_RADAR)
  {
    Hj = Sdce::Tools::CalculateJacobian(kalmanFilter.x_);
    kalmanFilter.R_ = R_radar;
    kalmanFilter.H_ = Hj;
    kalmanFilter.updateEkf(package.rawMeasurements);
  }
  else
  {
    kalmanFilter.R_ = R_laser;
    kalmanFilter.H_ = H_laser;
    kalmanFilter.update(package.rawMeasurements);
  }
}

} /* namespace Sdce */
