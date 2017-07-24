//
// Created by nick on 17-7-23.
//

#ifndef GROUND_TRUTH_PACKAGE_H
#define GROUND_TRUTH_PACKAGE_H

#include "Eigen/Dense"


class Ground_truth_package {
public:
    long long timestamp_;

    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd gt_values_;

};


#endif //GROUND_TRUTH_PACKAGE_H
