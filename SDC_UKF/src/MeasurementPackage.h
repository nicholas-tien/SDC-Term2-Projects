//
// Created by nick on 17-7-19.
//

#ifndef MEASUREMENTPACKAGE_H
#define MEASUREMENTPACKAGE_H

#include "Eigen/Dense"

using Eigen::VectorXd;
class MeasurementPackage {
public:
    long long timestamp_;

    enum SensorType{
            LASER,
            RADAR
    } sensor_type_;

    VectorXd raw_measurement_;

};


#endif //MEASUREMENTPACKAGE_H
