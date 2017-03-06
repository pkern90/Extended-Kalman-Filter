#include "FusionEKF.h"
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
    R_radar_ << 0.0225, 0, 0,
            0, 0.0225, 0,
            0, 0, 0.0225;
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
    Hj_ = MatrixXd(3, 4);

    //create a 4D state vector, we don't know yet the values of the x state
    VectorXd x_ = VectorXd(4);

    //state covariance matrix P
    MatrixXd P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    MatrixXd F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    MatrixXd Q_ = MatrixXd(4, 4);

    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);


    process_noise_ax = 100;
    process_noise_ay = 100;
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
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        double p_x = 0;
        double p_y = 0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            cout << "Init Radar Raw" << measurement_pack.raw_measurements_ << endl;
            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            double rho_dot = measurement_pack.raw_measurements_[2];

            p_x = rho * cos(phi);
            p_y = rho * sin(phi);
            ekf_.R_ = R_radar_;
            ekf_.H_ = Hj_;

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            cout << "Init Laser Raw" << measurement_pack.raw_measurements_ << endl;
            p_x = measurement_pack.raw_measurements_[0];
            p_y = measurement_pack.raw_measurements_[1];
            ekf_.R_ = R_laser_;
            ekf_.H_ = H_laser_;
        }

        ekf_.x_ << p_x, p_y, 0, 0;
        previous_timestamp_ = measurement_pack.timestamp_;

        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "Delta Time: " << dt << endl;

    ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

    ekf_.Q_ << pow(dt, 4) / 4 * process_noise_ax, 0, pow(dt, 3) / 2 * process_noise_ax, 0,
            0, pow(dt, 4) / 4 * process_noise_ay, 0, pow(dt, 3) / 2 * process_noise_ay,
            pow(dt, 3) / 2 * process_noise_ax, 0, pow(dt, 2) * process_noise_ax, 0,
            0, pow(dt, 3) / 2 * process_noise_ay, 0, pow(dt, 2) * process_noise_ay;

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        cout << "Update Radar Raw" << measurement_pack.raw_measurements_ << endl;
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        cout << "Update Laser Raw" << measurement_pack.raw_measurements_ << endl;
    }
    ekf_.Update(measurement_pack.raw_measurements_);

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
