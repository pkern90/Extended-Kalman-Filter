#ifndef MEASUREMENT_PACKAGE_HPP
#define MEASUREMENT_PACKAGE_HPP

#include "lib/Eigen/Dense"

class MeasurementPackage {
public:
    long timestamp_;

    enum SensorType {
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;
};

#endif /* MEASUREMENT_PACKAGE_HPP */
