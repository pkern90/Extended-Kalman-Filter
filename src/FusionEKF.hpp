#ifndef FusionEKF_HPP
#define FusionEKF_HPP

#include "measurement_package.hpp"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.hpp"
#include "tools.hpp"

class FusionEKF {
public:
    /**
    * Constructor.
    */
    FusionEKF();

    /**
    * Destructor.
    */
    virtual ~FusionEKF();

    /**
    * Run the whole flow of the Kalman Filter from here.
    */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
    * Kalman Filter update and prediction math lives in here.
    */
    KalmanFilter ekf_;

private:
    // check whether the tracking toolbox was initiallized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long previous_timestamp_;

    MatrixXd R_laser_;
    MatrixXd R_radar_;
    MatrixXd H_laser_;

    double process_noise_ax;
    double process_noise_ay;
};

#endif /* FusionEKF_HPP */
