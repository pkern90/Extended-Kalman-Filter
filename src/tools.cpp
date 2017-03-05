#include <iostream>
#include "tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
        cout << "Error: invalid input values." << endl;
        return rmse;
    }

    for(int i=0; i < estimations.size(); ++i){
        VectorXd res = estimations[i] - ground_truth[i];
        res = res.array() * res.array();
        rmse += res;
    }

    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float p_sq_sum = pow(px, 2) + pow(py, 2);
    float p_dist = sqrt(p_sq_sum);
    float p_sq_3_2 = pow(p_sq_sum, 3/2);

    if(fabs(p_sq_sum) < 0.0001){
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    float H_1_1 = px / p_dist;
    float H_2_1 = py / p_dist;
    float H_1_2 = - (py / p_sq_sum);
    float H_2_2 = - (px / p_sq_sum);
    float H_1_3 = (py * (vx * py - vy * px)) / p_sq_3_2;
    float H_2_3 = (px * (vy * px - vx * py)) / p_sq_3_2;
    float H_3_3 = H_1_1;
    float H_4_3 = H_2_1;

    Hj <<   H_1_1, H_2_1, 0,     0,
            H_1_2, H_2_2, 0,     0,
            H_1_3, H_2_3, H_3_3, H_4_3;

    return Hj;
}
