//
// Created by nick on 17-7-19.
//

#include "Tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {

}

Tools::~Tools() {

}

VectorXd Tools::calculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    //check the validity of the following inputs:
    // * the estimation vector size should not be zero
    // × the estimation vector size should equal to ground truth vector size
    if ( estimations.size() == 0 || ground_truth.size() == 0 ||
            (estimations.size() != ground_truth.size())) {
        cout << "Invalid estimation or ground truth data" << endl;
        return  rmse;
    }

    // accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd residuals(4);
        residuals = estimations[i] - ground_truth[i];
        residuals = residuals.array() * residuals.array();
        rmse += residuals;

    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    return  rmse;


}