//
// Created by nick on 17-7-19.
//

#ifndef UKF_H
#define UKF_H

#include <vector>
#include "Eigen/Dense"
#include <string>
#include <fstream>
#include "Tools.h"
#include "MeasurementPackage.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    ///* initiallay set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for unit)
    bool  use_laser_;

    ///* if this is false, radar measurements will be ignored (except for unit)
    bool  use_radar_;

    ///* state vector:[pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///*Sigma point spreading parameter
    double  lambda_;

    ///* the current NIS for radar
    double  NIS_radar_;

    ///* the current NIS for laser
    double  NIS_laser_;

    MatrixXd R_laser_;

    MatrixXd R_radar_;

    long long previous_timestamp_;

    MatrixXd H_laser_;

    MeasurementPackage Previous_measument_;

    /**
     *  Constructor
     */
    UKF();

    /**
     *  Destructor
     */
    virtual ~UKF();

    /**
     *  ProcessMeasurement
     *  @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * Prediction predicts sigma points, the state, and the state covariance matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates teh state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdataRadar(MeasurementPackage meas_package);

private:
    MatrixXd generate_sigma_points();
    MatrixXd predictSigmaPoints(MatrixXd Xsig_aug, double delta_t);
    VectorXd calculate_transition(VectorXd sigma_points, double delta_t);
    void predict_mean_and_covariance();
    VectorXd transform_to_radar_measurement_space(VectorXd predictd_sigma_point);




};


#endif //UKF_H






