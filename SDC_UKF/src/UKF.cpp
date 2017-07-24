//
// Created by nick on 17-7-19.
//

#include "UKF.h"
#include "Tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

    /**
     * Initializes Unscented Kalman Filter
     */
    UKF::UKF() {

        //Initially the UKF if  not initialized
        is_initialized_ = false;

        // if this is false,laser measurement will be ignored
        use_laser_ = true;

        // if this is false, radar measurement  will be ignored
        use_radar_ = true;

        //initial state vector
        x_ = VectorXd(5);
        x_.fill(0);

        //initial covariance matrix
        P_ = MatrixXd(5,5);
        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1;

        //Number of rows in the state vector
        n_x_ = x_.rows();

        //Number of rows in the state vector + 2 rows for noise process
        n_aug_ = n_x_ + 2;

        //Process noise standard deviation longitudinal acceleration in m/s^2
        std_a_ = 0.5;    // Need tuning (ninety-fity percent of the time)

        // Process noise standard deviation yaw acceleration in rad/s^2
        std_yawdd_ = 0.55; // Need tuning (ninety-five percent of the time)

        // Laser measurement noise standard deviation positionl in m
        std_laspx_ = 0.15;

        // Laser measurement noise standard deviation position2 in m
        std_laspy_= 0.15;

        //Radar measurement noise standard deviation radius in m
        std_radr_ = 0.3;

        // Radar measurement noise standard deviation angle in rad
        std_radphi_ = 0.03;

        // Radar measurement noise standard deviation radius change in m/s
        std_radrd_ = 0.3;

        Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
        Xsig_pred_.fill(0);

        lambda_ = 3 - n_aug_;

        // Initialize weights
        weights_ = VectorXd(2 * n_aug_ + 1);

        weights_(0) = lambda_ / float(lambda_ + n_aug_);
        for (int i = 1; i < weights_.size(); ++i) {
            weights_(i) = 1 / float(2 * (n_aug_ + lambda_));
        }

        NIS_laser_ = 0;
        NIS_radar_ = 0;

        R_laser_ = MatrixXd(2,2);
        R_laser_ << pow(std_laspx_,2), 0,
                    0,          pow(std_laspy_,2);

        R_radar_ = MatrixXd(3,3);
        R_radar_ << pow(std_radr_,2),   0,  0,
                   0,   pow(std_radphi_,2), 0,
                   0,   0,   pow(std_radrd_,2);

        previous_timestamp_ = 0;

        H_laser_ = MatrixXd(2,n_x_);
        H_laser_ << 1, 0, 0, 0, 0,
                    0, 1, 0, 0, 0;


    }

    UKF::~UKF() {}

    void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
        //if not initialized,initialize
        if (!is_initialized_){
            if (meas_package.sensor_type_ == MeasurementPackage::LASER){
                x_ << meas_package.raw_measurement_[0],meas_package.raw_measurement_[1],0,0,0;
            } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
                double rho = meas_package.raw_measurement_[0];
                double phi = meas_package.raw_measurement_[1];
                double  rho_dot = meas_package.raw_measurement_[2];

                x_ << rho * cos(phi), rho * sin(phi),0, 0, 0;
            }

            if (x_(0) == 0 && x_(1) == 0){
                x_(0) = 0.01;
                x_(0) = 0.01;
            }
            previous_timestamp_ = meas_package.timestamp_;
            is_initialized_ = true;

            // not ready to prosess,must return to prosess next
            return;
        }

        double  dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
        previous_timestamp_ = meas_package.timestamp_;

        Prediction(dt);

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
            UpdataRadar(meas_package);
        } else {
            UpdateLidar(meas_package);
        }


    }


    MatrixXd UKF::generate_sigma_points() {
        //create sigma point matrix
        MatrixXd Xsig_aug = MatrixXd(n_aug_,2 * n_aug_ + 1);

        //create Augmented state mean vector x_aug and augmented state covariance matrix P_aug
        VectorXd x_aug = VectorXd(n_aug_);
        x_aug.fill(0.0);
        x_aug.segment(0,n_x_) = x_;

        //create augmented state covariance
        MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
        P_aug.fill(0.0);
        P_aug.topLeftCorner(n_x_,n_x_) = P_;
        P_aug(n_x_,n_x_) = pow(std_a_,2);
        P_aug(n_x_ + 1,n_x_ + 1) = pow(std_yawdd_,2);

        //for debug
        //cout << "P_aug" << P_aug << endl;

        //create square root matrix
        MatrixXd A = P_aug.llt().matrixL();
        if (P_aug.llt().info() == Eigen::NumericalIssue){
            //if decompostion fails,we have numrical issues
            std::cout << "LLT failed" << std::endl;
            throw std::range_error("LLT failed");
        }

        //create augmented sigma points
        Xsig_aug.col(0) = x_aug;
        MatrixXd term = sqrt(lambda_ + n_aug_) * A;
        for (int i = 0; i < n_aug_ ; ++i) {
            Xsig_aug.col(i + 1) = x_aug +term.col(i);
            Xsig_aug.col(i + n_aug_ + 1) = x_aug - term.col(i);

        }

        return  Xsig_aug;

    }

    MatrixXd UKF::predictSigmaPoints(MatrixXd Xsig_aug, double delta_t) {
        MatrixXd predictions(n_x_, 2 * n_aug_ + 1);
        for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
            predictions.col(i) = Xsig_aug.col(i).head(n_x_) + calculate_transition(Xsig_aug.col(i), delta_t);
        }
        return predictions;
    }

    VectorXd UKF::calculate_transition(VectorXd sigma_points, double delta_t) {
        VectorXd transition(n_x_);
        transition.fill(0.0);

        double p_x = sigma_points(0);
        double p_y = sigma_points(1);
        double v = sigma_points(2);
        double psi = sigma_points(3);
        double psi_dot = sigma_points(4);
        double long_acceleration = sigma_points(5);
        double yaw_rate_acceleration = sigma_points(6);

        VectorXd process_noise(n_x_);
        process_noise(0) = 1/2.0 * delta_t * delta_t * cos(psi) * long_acceleration;
        process_noise(1) = 1/2.0 * delta_t * delta_t * sin(psi) * long_acceleration;
        process_noise(2) = long_acceleration * delta_t;
        process_noise(3) = 1/2.0 * delta_t * delta_t * yaw_rate_acceleration;
        process_noise(4) = yaw_rate_acceleration * delta_t;

        if (psi_dot != 0) {
            transition(0) = (v/float(psi_dot) * (sin(psi + psi_dot * delta_t) - sin(psi)));
            transition(1) = (v/float(psi_dot) * (-cos(psi + psi_dot * delta_t) + cos(psi)));
            transition(2) = 0;
            transition(3) = psi_dot * delta_t;
            transition(4) = 0;

        } else {
            transition(0) = v * cos(psi) * delta_t;
            transition(1) = v * sin(psi) * delta_t;
            transition(2) = 0;
            transition(3) = 0;
            transition(4) = 0;

        }

        return transition + process_noise;

    }

    void UKF::predict_mean_and_covariance() {
        //create vector for predicted state
        VectorXd x = VectorXd(n_x_);
        x.fill(0.0);

        //create covariance matrix for prediction
        MatrixXd P = MatrixXd(n_x_,n_x_);
        P.fill(0.0);

        //for debug
        //cout << "Xsig_pred_: " << Xsig_pred_ << endl;

        //predicted state mean
        for (int i = 0; i < Xsig_pred_.cols(); ++i) {
            x = x + weights_(i) * Xsig_pred_.col(i);
        }

        //predicted state covariace matrix
        for (int j = 0; j < Xsig_pred_.cols(); ++j) {
            //state difference
            VectorXd x_diff = Xsig_pred_.col(j) - x;

            //angel normalization
            while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
            while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

            P = P + weights_(j) * x_diff * x_diff.transpose();
        }

        //write result
        x_ = x;
        P_ = P;

    //    cout << "x_ and P_" << x_ << endl << P_ << endl;
    }

    void UKF::Prediction(double delta_t) {
        //Generate Sigma points
        MatrixXd Xsig_aug = generate_sigma_points();

        //use the prediction function to predict the k+1 values for these sigma points
        Xsig_pred_ = predictSigmaPoints(Xsig_aug,delta_t);

        // compute mean and covariance the state predicted at time k+1
        predict_mean_and_covariance();

    }


/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} meas_package
*/
    void UKF::UpdateLidar(MeasurementPackage meas_package) {
        //process nonsense data (both data are too small)
        float min_sensor_value = 0.000001;
        if (fabs(x_(0)) <= min_sensor_value && fabs(x_(1)) <= min_sensor_value){
            x_(0) = (x_(0) < 0) ? -min_sensor_value:min_sensor_value;
            x_(1) = (x_(1) < 0) ? -min_sensor_value:min_sensor_value;
        }

        VectorXd z_pred = H_laser_ * x_;
        VectorXd z = meas_package.raw_measurement_;
        VectorXd y = z - z_pred;

        MatrixXd Ht = H_laser_.transpose();
        MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
        MatrixXd Si = S.inverse();
        MatrixXd PHt = P_ * Ht;
        MatrixXd K = PHt * Si;

        //new estimate
        x_ = x_ + (K * y);
        long x_size = x_.rows();
        MatrixXd I = MatrixXd::Identity(x_size,x_size);
        P_ = (I - K * H_laser_) * P_;

        NIS_laser_ = y.transpose() * Si * y;

        //DEBUG
        //cout << "x_ and P_" << x_ << endl << P_ << endl;
    }


    VectorXd UKF::transform_to_radar_measurement_space(VectorXd predictd_sigma_point) {
        VectorXd measurement_space_sigma_points(3);

        double px = predictd_sigma_point(0);
        double py = predictd_sigma_point(1);
        double v = predictd_sigma_point(2);
        double psi = predictd_sigma_point(3);
        double psi_dot = predictd_sigma_point(4);

        if (px == 0 && py == 0){
            measurement_space_sigma_points << 0, 0, 0;
            return measurement_space_sigma_points;
        }

        //Rho
        measurement_space_sigma_points(0) = sqrt(pow(px,2) + pow(py,2));

        //Phi
        measurement_space_sigma_points(1) = atan(py/float(px));

        //Rho-dot
        measurement_space_sigma_points(2) = ((px * v * cos(psi) + py * v * sin(psi))/float(measurement_space_sigma_points(0)));

        return measurement_space_sigma_points;
    }


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
    void UKF::UpdataRadar(MeasurementPackage meas_package) {
    //map the predicted state x_ to the measurement space
    int n_z = 3; // number to dimensions in the measurement space for RADAR
    MatrixXd Zsig = MatrixXd(n_z,2 * n_aug_ + 1);

    //transform sigma points into measurment space
    for (int i = 0; i < Xsig_pred_.cols(); ++i) {
        Zsig.col(i) = transform_to_radar_measurement_space(Xsig_pred_.col(i));
    }

    //Calculate the mean z_pred and covariance S of the predicted points
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);

    for (int j = 0; j < Zsig.cols(); ++j) {
        z_pred += (weights_(j) * Zsig.col(j));
    }

    //Measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S = R_radar_;
    for (int k = 0; k < Zsig.cols(); ++k) {
        VectorXd diff = Zsig.col(k) - z_pred;

        //check Phi
        while (diff(1) > M_PI) diff(1) -= 2.0*M_PI;
        while (diff(1) < -M_PI) diff(1) += 2.0*M_PI;

        S += weights_(k) * (diff * diff.transpose());
    }

    //Calculate the cross-correlation matrix
    MatrixXd Tc = MatrixXd(n_x_,n_z);
    Tc.fill(0.0);
    for(int i =0;i < 2 * n_aug_ + 1;++i){
        VectorXd diff_x = (Xsig_pred_.col(i) - x_);

        //check diff(3) psi angle
        while (diff_x(3) > M_PI) diff_x(3) -= 2.0 * M_PI;
        while (diff_x(3) < -M_PI) diff_x(3) += 2.0 * M_PI;

        VectorXd diff_z = Zsig.col(i) - z_pred;

        //check diff_z(1) phi angle
        while (diff_z(1) > M_PI) diff_z(1) -= 2.0 * M_PI;
        while (diff_z(1) < -M_PI) diff_z(1) += 2.0 * M_PI;

        Tc += weights_(i) * (diff_x * diff_z.transpose());
    }

    //Update the state x_ and P_ using Kalman filter
    MatrixXd K(n_x_,n_z);
    K = Tc * S.inverse();

    //residual
    VectorXd z = meas_package.raw_measurement_;
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    //Update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    NIS_radar_ = z_diff.transpose() * S.transpose() * z_diff;

    //DEBUG
    //cout << "x_ and P_" << x_ << endl << P_ << endl;

}





















