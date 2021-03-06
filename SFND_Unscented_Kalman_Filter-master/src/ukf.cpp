#include "ukf.h"
#include "Eigen/Dense"
#include<iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ =1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_) {
    // setting the state with the initial location and zero velocity for the obstacle

    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) 
    {
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1], 0, 0, 0;
      P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
            0, std_laspy_ * std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    } 
    else if (meas_package.sensor_type_ ==  MeasurementPackage::SensorType::RADAR) 
    {
      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      const double rhodot = meas_package.raw_measurements_(2);
      const double x = rho * cos(phi);
      const double y = rho * sin(phi);
      const double vx = rhodot * cos(phi);
      const double vy = rhodot * sin(phi);
      const double v = std::sqrt(vx * vx + vy * vy);
      x_ << x, y, v, rho, rhodot;
      P_ << std_radr_* std_radr_, 0, 0, 0, 0,
            0, std_radr_ * std_radr_, 0, 0, 0,
            0, 0, std_radrd_ * std_radrd_, 0, 0,
            0, 0, 0, std_radphi_ * std_radphi_, 0,
            0, 0, 0, 0, std_radphi_ * std_radphi_;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  const float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // Predict the next states and covariance matrix
  Prediction(dt);
  
  // Update the next states and covariance matrix
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true) 
  {
    UpdateLidar(meas_package);
  } 
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true) 
  {
    UpdateRadar(meas_package);
    /*double test = x_(0);
    if(std::isnan(test)==1){
    std::cout<<"x_ "<<x_<<std::endl;
    std::getchar();}*/
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  /*
  ****** Creating Sigma Points
  */

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  Xsig_pred_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_pred_.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_pred_.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_pred_.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  /*
  ****** Predicting Sigma Points
  */
  for (int i = 0; i< 2*n_aug_+1; ++i) {
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double yawd = Xsig_pred_(4,i);
    double nu_a = Xsig_pred_(5,i);
    double nu_yawdd = Xsig_pred_(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.000000001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /*
  ****** Predict mean and covariance
  */
  // create vector for predicted state
  VectorXd xtemp = VectorXd(n_x_);

  // create covariance matrix for prediction
  MatrixXd Ptemp = MatrixXd(n_x_, n_x_);

  // predicted state mean
  MatrixXd Xsig_pred_new(n_x_, 2*n_aug_+1);
  for (int i = 0; i < n_x_; i++)
  {
    Xsig_pred_new.row(i) = Xsig_pred_.row(i);
  }

  xtemp =Xsig_pred_new*weights_;
  

  // predicted state covariance matrix
  Ptemp.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_new.col(i) - xtemp;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Ptemp = Ptemp + weights_(i) * x_diff * x_diff.transpose() ;
  }

  P_ = Ptemp;
  x_ = xtemp;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Lidar linear - KF
  /*
  ****** Predict K
  */
  MatrixXd H, R_;
  H.setZero(2, n_x_);
  H(0, 0) = H(1, 1) = 1;  // Select out the position elements only
  VectorXd z_pred = H * x_;
  const VectorXd z = meas_package.raw_measurements_;

  // Calculate residual vector zdiff
  const VectorXd zdiff = z - z_pred;

  // New - define measurement noise matrix
  R_.setZero(2, 2);
  R_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

  // Create innovation covariance matrix S
  MatrixXd S = H * P_ * H.transpose() + R_;

  // Create Kalman gain matrix K
  MatrixXd K = P_ * H.transpose() * S.inverse();

  /*
  ****** Predict mean and covariance
  */
 // Create new estimate for states and covariance
  x_ = x_ + (K * zdiff);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H) * P_;

  // Calculate NIS for LiDAR
  NIS_laser_ = zdiff.transpose() * S.inverse() * zdiff;
  std::cout<<"x after laser update"<<x_<<std::endl;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  int n_z = 3;
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // transform sigma points to z sigma
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    // extract value
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double yawd = Xsig_pred_(4, i);

    // measurement model
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);    // r
    Zsig(1, i) = atan2(p_y, p_x);                // phi
    Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);    // r_dot
  }
  

  // calculate predict mean and covariance in measurement Z
  Eigen::VectorXd z_pred = VectorXd::Zero(n_z);
  Eigen::MatrixXd S = MatrixXd::Zero(n_z, n_z);

  // measurement mean
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // measurement covariance
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;
  S = S + R;


  // update state mean and covariance
  VectorXd z = meas_package.raw_measurements_; // true received measurement
  
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z); // cross correlation matrix
  MatrixXd Xsig_pred_new(n_x_, 2*n_aug_+1);
  for (int i = 0; i < n_x_; i++)
  {
    Xsig_pred_new.row(i) = Xsig_pred_.row(i);
  }
  // calculate Tc
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_new.col(i) - x_;

    while (x_diff(3) > M_PI) x_diff(3) -= 2.0*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

    // measurement difference
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }


  // calculate kalman gain K
  MatrixXd K = Tc * S.inverse();

  // residuals
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

  // update state mean and covariance
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}
