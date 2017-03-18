#include "FusionEKF.hpp"
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
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    //state covariance matrix P
    MatrixXd P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    MatrixXd F_ = MatrixXd(4, 4);
    MatrixXd Q_ = MatrixXd(4, 4);

    VectorXd x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;

    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

    process_noise_ax = 9;
    process_noise_ay = 9;
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
        // first measurement
        cout << "EKF: " << endl;

        double px = 0;
        double py = 0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];

            px = rho * cos(phi);
            py = rho * sin(phi);

            // If initial values are zero they will set to an initial guess
            // and the uncertainty will be increased.
            // Initial zeros would cause the algorithm to fail when using only Radar data.
            if(fabs(px) < 0.0001){
                px = 1;
                ekf_.P_(0,0) = 1000;
            }
            if(fabs(py) < 0.0001){
                py = 1;
                ekf_.P_(1,1) = 1000;
            }

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];
        }

        ekf_.x_ << px, py, 0, 0;
        previous_timestamp_ = measurement_pack.timestamp_;

        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

    ekf_.Q_ << pow(dt, 4.0d) / 4 * process_noise_ax, 0, pow(dt, 3.0d) / 2 * process_noise_ax, 0,
            0, pow(dt, 4.0d) / 4 * process_noise_ay, 0, pow(dt, 3.0d) / 2 * process_noise_ay,
            pow(dt, 3.0d) / 2 * process_noise_ax, 0, dt * dt * process_noise_ax, 0,
            0, pow(dt, 3.0d) / 2 * process_noise_ay, 0, dt * dt * process_noise_ay;

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        ekf_.H_ = tools::CalculateJacobian(ekf_.x_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }
}
