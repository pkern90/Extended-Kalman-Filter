#ifndef GROUND_TRUTH_PACKAGE_HPP
#define GROUND_TRUTH_PACKAGE_HPP

#include "lib/Eigen/Dense"

class GroundTruthPackage {
public:
    long timestamp_;

    enum SensorType {
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd gt_values_;

};

#endif /* MEASUREMENT_PACKAGE_HPP */
