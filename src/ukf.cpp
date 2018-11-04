#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    //set state dimension
    n_x_ = 5;

    // initial state vector
    x_ = VectorXd(n_x_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    /**
    TODO:

    Complete the initialization. See ukf.h for other member properties.


    Hint: one or more values initialized above might be wildly off...
    */
    //set augmented dimension
    n_aug_ = n_x_ + 2;
    //define spreading parameter
    lambda_ = 3 - n_aug_;
    //set augmented dimension
    n_sigma_ = 2 * n_aug_ + 1;
    // define weights verctor
    weights_ = VectorXd(n_sigma_);
    // define measurement covariance matrix - radar
    ////set measurement dimension, radar can measure r, phi, and r_dot
    int n_z_radar = 3;
    R_radar_ = MatrixXd(n_z_radar, n_z_radar);
    // define measurement covariance matrix - laser
    ////set measurement dimension, laser can measure x, y
    int n_z_laser = 2;
    R_laser_ = MatrixXd(n_z_laser, n_z_laser);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */

    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        cout << "UKF: " << endl;
        x_ << 1, 1, 1, 1, 1;
        P_.setIdentity();

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            //cout << "Initializing radar package..." << endl;
            x_(0) = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
            x_(1) = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
            x_(2) = 0;
            x_(3) = 0;
            x_(4) = 0;

            R_radar_.fill(0.0);
            R_radar_(0, 0) = std_radr_ * std_radr_;
            R_radar_(1, 1) = std_radphi_ * std_radphi_;
            R_radar_(2, 2) = std_radrd_ * std_radrd_;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            //cout << "Initializing laser package... " << endl;
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
            R_laser_.fill(0.0);
            R_laser_(0, 0) = std_laspx_ * std_laspx_;
            R_laser_(1, 1) = std_laspy_ * std_laspy_;
        }

        //cout << " initialized x_: " << x_ << endl;

        previous_timestamp_ = meas_package.timestamp_;

        //set weights
        for (int i = 0; i < n_sigma_; i++)
        {
            if (i == 0)
                weights_(i) = lambda_ / (lambda_ + n_aug_);
            else
                weights_(i) = 1 / (2 * (lambda_ + n_aug_));
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    //////////////////////////////////////////////
    ////////////////// Prediction  ///////////////
    //////////////////////////////////////////////
    delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//delta_t - expressed in seconds

    Prediction(delta_t);

    //////////////////////////////////////////////
    //////////////////  Update  //////////////////
    //////////////////////////////////////////////
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        // Radar updates
        //cout << "Updating radar measurement ..." << endl;
        UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        // Laser updates
        //cout << "Updating laser measurement ..." << endl;
        UpdateLidar(meas_package);
    }

    // print the output
    cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
    //////////////////////////////////////////////
    //////  Create Augmented Sigma points  ///////
    //////////////////////////////////////////////

      //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_);

    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug(5, 5) = pow(std_a_, 2);
    P_aug(6, 6) = pow(std_yawdd_, 2);

    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 1; i < n_aug_ + 1; i++)
    {
        Xsig_aug.col(i) = x_aug + sqrt(lambda_ + n_aug_)*A.col(i - 1);
        Xsig_aug.col(i + n_aug_) = x_aug - sqrt(lambda_ + n_aug_)*A.col(i - 1);
    }

    //////////////////////////////////////////////
    //////////  Predict Sigma points  ////////////
    //////////////////////////////////////////////
      //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x_, n_sigma_);

    double px, py, v, psi, psi_rate, nu, nu_rate;
    double px_pred = 0, py_pred = 0, v_pred = 0, psi_pred, psi_rate_pred;
    const double epsilon = 1e-5;
    for (int i = 0; i < n_sigma_; i++)
    {
        px = Xsig_aug(0, i);
        py = Xsig_aug(1, i);
        v = Xsig_aug(2, i);
        psi = Xsig_aug(3, i);
        psi_rate = Xsig_aug(4, i);
        nu = Xsig_aug(5, i);
        nu_rate = Xsig_aug(6, i);

        v_pred = v + (delta_t * nu);
        psi_pred = psi +
            (psi_rate * delta_t) +
            (0.5 * delta_t * delta_t * nu_rate);
        psi_rate_pred = psi_rate + (delta_t * nu_rate);

        if (fabs(psi_rate) <= epsilon)
        {
            //std::cout << "psi_rate equals to zero: " << fabs(psi_rate) << std::endl;
            px_pred = px +
                (v * cos(psi) * delta_t) +
                (0.5 * delta_t * delta_t * cos(psi) * nu);
            py_pred = py +
                (v * sin(psi) * delta_t) +
                (0.5 * delta_t * delta_t * sin(psi) * nu);
        }
        else
        {
            //std::cout << "psi_rate greater than zero: " << fabs(psi_rate) << std::endl;
            px_pred = px +
                (v / psi_rate) * (sin(psi + (psi_rate * delta_t)) - sin(psi)) +
                (0.5 * delta_t * delta_t * cos(psi) * nu);
            //std::cout << "(nu/psi_rate) * (sin(psi + (psi_rate * delta_t)) - sin(psi)): " << (nu/psi_rate) * (sin(psi + (psi_rate * delta_t)) - sin(psi)) << std::endl;         
            //std::cout << "(0.5 * delta_t * delta_t * cos(psi) * nu): " << (0.5 * delta_t * delta_t * cos(psi) * nu) << std::endl;         
            py_pred = py +
                (v / psi_rate) * (-1 * cos(psi + (psi_rate * delta_t)) + cos(psi)) +
                (0.5 * delta_t * delta_t * sin(psi) * nu);
        }

        Xsig_pred(0, i) = px_pred;
        Xsig_pred(1, i) = py_pred;
        Xsig_pred(2, i) = v_pred;
        Xsig_pred(3, i) = psi_pred;
        Xsig_pred(4, i) = psi_rate_pred;

        //std::cout << Xsig_pred(i);

   //////////////////////////////////////////////
   ////////  Predict mean and covariance  ///////
   //////////////////////////////////////////////

        //predict state mean
        x_.fill(0.0);
        P_.fill(0.0);
        for (int i = 0; i < n_sigma_; i++)
        {
            x_ += weights_(i) * Xsig_pred.col(i);
        }
        //predict state covariance matrix
        VectorXd x_diff = VectorXd(n_x_);
        for (int i = 0; i < n_sigma_; i++)
        {
            x_diff = Xsig_pred.col(i) - x_;
            //angle normalization
            tools.PerformAngleNormalization(x_diff, i);

            P_ += weights_(i) * x_diff * x_diff.transpose();
        }
    }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
    /**
    * update the state by using Kalman Filter equations
    */
    //set measurement dimension, laser can measure x, y
    int n_z = 2;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sigma_);

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);

    //transform sigma points into measurement space
    Zsig.fill(0.0);
    for (int i = 0; i < n_sigma_; i++)
    {
        Zsig(0, i) = Xsig_pred_(0, i);
        Zsig(1, i) = Xsig_pred_(1, i);
    }

    //std::cout << "Zsig: " << Zsig << std::endl;
    //calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < n_sigma_; i++)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    //calculate innovation covariance matrix S
    S.fill(0.0);
    VectorXd z_diff = VectorXd(n_z);
    for (int i = 0; i < n_sigma_; i++)
    {
        z_diff = Zsig.col(i) - z_pred;

        S += weights_(i) * z_diff * z_diff.transpose();
    }
    S += R_laser_;

    //////////////////////////////////////////////
    ///////////////  Update Lidar  ///////////////
    //////////////////////////////////////////////
      //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    //create example vector for incoming laser measurement
    VectorXd z = VectorXd(n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    VectorXd x_diff;
    VectorXd z_diff;
    for (int i = 0; i < n_sigma_; i++)
    {
        x_diff = Xsig_pred_.col(i) - x_;

        z_diff = Zsig.col(i) - z_pred;

        Tc += weights_(i)*x_diff*z_diff.transpose();
    }
    //calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    //update state mean and covariance matrix
    z_diff = z - z_pred;

    //new estimate
    x_ += K * z_diff;
    P_ -= K * S*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.
    TODO
    You'll also need to calculate the radar NIS.
    */
    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sigma_);

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);

    //transform sigma points into measurement space
    Zsig.fill(0.0);
    double rho, phi, rho_dot;
    for (int i = 0; i < n_sigma_; i++)
    {
        rho = sqrt(Xsig_pred_(0, i) * Xsig_pred_(0, i) + Xsig_pred_(1, i) * Xsig_pred_(1, i));
        Zsig(0, i) = rho;
        phi = atan(Xsig_pred_(1, i) / Xsig_pred_(0, i));
        Zsig(1, i) = phi;
        rho_dot = ((Xsig_pred_(0, i) * cos(Xsig_pred_(3, i)) + Xsig_pred_(1, i) * sin(Xsig_pred_(3, i))) * Xsig_pred_(2, i)) / rho;
        Zsig(2, i) = rho_dot;
    }
    //std::cout << "Zsig: " << Zsig << std::endl;
    //calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < n_sigma_; i++)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    //calculate innovation covariance matrix S
    S.fill(0.0);
    VectorXd z_diff = VectorXd(n_z);
    for (int i = 0; i < n_sigma_; i++)
    {
        z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        tools.PerformAngleNormalization(z_diff, i);

        S += weights_(i) * z_diff * z_diff.transpose();
    }

    S += R_radar_;

    //////////////////////////////////////////////
    ///////////////  Update Radar  ///////////////
    //////////////////////////////////////////////
      //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    VectorXd x_diff;
    VectorXd z_diff;
    for (int i = 0; i < n_sigma_; i++)
    {
        x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        tools.PerformAngleNormalization(x_diff, i);

        z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        tools.PerformAngleNormalization(z_diff, i);

        Tc += weights_(i)*x_diff*z_diff.transpose();
    }
    //calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    //update state mean and covariance matrix
    z_diff = z - z_pred;
    //angle normalization
    tools.PerformAngleNormalization(z_diff, 1);
    //new estimate
    x_ += K * z_diff;
    P_ -= K * S*K.transpose();
}
