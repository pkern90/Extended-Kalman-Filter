#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <vector>
#include "lib/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

namespace tools {

        /**
        * A helper method to calculate RMSE.
        */
        VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

        /**
        * A helper method to calculate Jacobians.
        */
        MatrixXd CalculateJacobian(const VectorXd &x_state);

};

#endif /* TOOLS_HPP */
