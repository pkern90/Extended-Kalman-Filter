#include <iostream>
#include "tools.hpp"

namespace tools {
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                           const vector<VectorXd> &ground_truth) {
        VectorXd rmse(4);
        rmse << 0, 0, 0, 0;

        if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
            cout << "CalculateRMSE () - Error: Invalid input values." << endl;
            return rmse;
        }

        for (int i = 0; i < estimations.size(); ++i) {
            VectorXd res = estimations[i] - ground_truth[i];
            res = res.array() * res.array();
            rmse += res;
        }

        rmse /= estimations.size();
        rmse = rmse.array().sqrt();

        return rmse;
    }

    MatrixXd CalculateJacobian(const VectorXd &x_state) {

        MatrixXd Hj(3, 4);
        double px = x_state(0);
        double py = x_state(1);
        double vx = x_state(2);
        double vy = x_state(3);

        double p_sq_sum = px * px + py * py;
        double p_dist = sqrt(p_sq_sum);
        double p_sq_3_2 = p_sq_sum * p_dist;

        if (fabs(p_sq_sum) < 0.0001) {
            cout << "CalculateJacobian () - Error: Division by Zero" << endl;
            return Hj;
        }


        Hj << (px / p_dist), (py / p_dist), 0, 0,
                -(py / p_sq_sum), (px / p_sq_sum), 0, 0,
                py * (vx * py - vy * px) / p_sq_3_2, px * (px * vy - py * vx) / p_sq_3_2, (px / p_dist), (py / p_dist);

        return Hj;
    }
}


