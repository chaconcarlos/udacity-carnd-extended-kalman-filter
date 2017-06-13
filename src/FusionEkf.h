#ifndef SDCE_FUSIONEKF_H_
#define SDCE_FUSIONEKF_H_

/* INCLUDES ******************************************************************/

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

#include "KalmanFilter.h"
#include "MeasurementPackage.h"
#include "Tools.h"

/* CLASS DECLARATION *********************************************************/

namespace Sdce
{

/**
 * @brief Processes sensor measurements to predict the state of a tracked object.
 */
class FusionEkf
{
  public:

    /**
     * @brief Initializes an instance of the FusionEKF class.
     */
    FusionEkf();

    /**
     * @brief Finalizes an instance of the FusionEKF class.
     */
    virtual ~FusionEkf();

  public:

    /**
     * @brief Gets the current instance of the extended Kalman filter.
     *
     * @return The current instance of the extended Kalman filter.
     */
    KalmanFilter& getKalmanFilter();

    /**
     * @brief Processes a sensor measurement.
     *
     * @param package The measurements package.
     */
    void ProcessMeasurement(const MeasurementPackage& package);

  private:

    bool            isInitialized;
    long long       previousTimestamp;
    float           noise_ax;
    float           noise_ay;
    KalmanFilter    kalmanFilter;
    Eigen::MatrixXd R_laser;
    Eigen::MatrixXd R_radar;
    Eigen::MatrixXd H_laser;
    Eigen::MatrixXd Hj;
};

} /* namespace Sdce */

#endif /* SDCE_FUSIONEKF_H_ */
