#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <vector>
#include "lib/Eigen/Dense"

namespace tools {

    /**
    * A helper method to calculate RMSE.
    */
    Eigen::VectorXd
    CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

    /**
    * A helper method to calculate Jacobians.
    */
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);

};

#endif /* TOOLS_HPP */
