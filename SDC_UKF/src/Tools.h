//
// Created by nick on 17-7-19.
//

#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include "Eigen/Dense"

using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class Tools {
public:
    /**
     *  Constructor
     */
    Tools();

    /**
     * Deconstructor
     */
    virtual ~Tools();

    /**
     * A helper method to calculate RMSE.
     */
    VectorXd calculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

};


#endif //TOOLS_H
