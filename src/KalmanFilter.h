#ifndef SDCE_KALMANFILTER_H_
#define SDCE_KALMANFILTER_H_

/* INCLUDES ******************************************************************/

#include "Eigen/Dense"

/* CLASS DECLARATION *********************************************************/

namespace Sdce
{

/**
 * @brief Implmentation of the extended Kalman Filter.
 */
class KalmanFilter
{
  public:

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    /**
     * @brief Initializes an instance of the KalmanFilter class.
     */
    KalmanFilter();

    /**
     * @brief Finalizes an instance of the KalmanFilter class.
     */
    virtual ~KalmanFilter();

    /**
     * @brief Init Initializes Kalman filter
     *
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void initialize(
      Eigen::VectorXd& x_in,
      Eigen::MatrixXd& P_in,
      Eigen::MatrixXd& F_in,
      Eigen::MatrixXd& H_in,
      Eigen::MatrixXd& R_in,
      Eigen::MatrixXd& Q_in);

    /**
     * @brief Prediction Predicts the state and the state covariance using the process model.
     */
    void predict();

    /**
     * @brief Updates the state by using standard Kalman Filter equations
     *
     * @param z The measurement at k+1.
     */
    void update(const Eigen::VectorXd& z);

    /**
     * @brief Updates the state by using Extended Kalman Filter equations.
     *
     * @param z The measurement at k+1
     */
    void updateEkf(const Eigen::VectorXd& z);

  private:

    void doUpdate(const Eigen::VectorXd& y);
};

} /* namespace Sdce */

#endif /* SDCE_KALMANFILTER_H_ */
