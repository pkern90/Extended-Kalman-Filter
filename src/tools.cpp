#include <iostream>
#include "tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        cout << "Error: invalid input values." << endl;
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

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {

    MatrixXd Hj(3, 4);
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    double p_sq_sum = pow(px, 2) + pow(py, 2);
    double p_dist = sqrt(p_sq_sum);
    float p_sq_3_2 = pow(p_sq_sum, 3 / 2);

    if (fabs(p_sq_sum) < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    double H_1_1 = px / p_dist;
    double H_2_1 = py / p_dist;
    double H_1_2 = -(py / p_sq_sum);
    double H_2_2 = -(px / p_sq_sum);
    double H_1_3 = (py * (vx * py - vy * px)) / p_sq_3_2;
    double H_2_3 = (px * (vy * px - vx * py)) / p_sq_3_2;
    double H_3_3 = H_1_1;
    double H_4_3 = H_2_1;

    Hj << H_1_1, H_2_1, 0, 0,
            H_1_2, H_2_2, 0, 0,
            H_1_3, H_2_3, H_3_3, H_4_3;

    return Hj;
}
