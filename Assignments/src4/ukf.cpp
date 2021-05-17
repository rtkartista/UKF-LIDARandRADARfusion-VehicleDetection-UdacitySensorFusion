#include <iostream>
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}

void UKF::AugmentedSigmaPoints(VectorXd* x, MatrixXd* P, MatrixXd* Xsig_out) {

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  // create augmented mean state
  x_aug.head(5) = *x;
  x_aug(5) = 0;
  x_aug(6) = 0;
  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = *P;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_pred, MatrixXd* Xsig_aug) {
  /**
   * Student part begin
   */
  // predict sigma points
  for (int i = 0; i< 2*n_aug+1; ++i) {
    // extract values for better readability
    double p_x = (*Xsig_aug)(0,i);
    double p_y = (*Xsig_aug)(1,i);
    double v = (*Xsig_aug)(2,i);
    double yaw = (*Xsig_aug)(3,i);
    double yawd = (*Xsig_aug)(4,i);
    double nu_a = (*Xsig_aug)(5,i);
    double nu_yawdd = (*Xsig_aug)(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
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
    (*Xsig_pred)(0,i) = px_p;
    (*Xsig_pred)(1,i) = py_p;
    (*Xsig_pred)(2,i) = v_p;
    (*Xsig_pred)(3,i) = yaw_p;
    (*Xsig_pred)(4,i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance(VectorXd* x_pred, MatrixXd* P_pred, MatrixXd* Xsig_pred) {

  // create vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  /**
   * Student part begin
   */

  // set weights
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; ++i) {  // 2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }
 // create vector for predicted state
  VectorXd xtemp = VectorXd(n_x);

  // create covariance matrix for prediction
  MatrixXd Ptemp = MatrixXd(n_x, n_x);

  // predicted state mean
  xtemp.fill(0.0);
  MatrixXd Xsig_pred_new(n_x, 2*n_aug+1);
  for (int i = 0; i < n_x; i++)
  {
    Xsig_pred_new.row(i) = (*Xsig_pred).row(i);
  }

  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // iterate over sigma points
    xtemp = xtemp + weights(i) * Xsig_pred_new.col(i);
  }

  // predicted state covariance matrix
  Ptemp.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_new.col(i) - xtemp;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Ptemp = Ptemp + weights(i) * x_diff * x_diff.transpose() ;
  }

  *x_pred = xtemp;
  *P_pred = Ptemp;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Xsig_pred, MatrixXd* Zsig) {

  // set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  double weight = 0.5/(lambda+n_aug);
  weights(0) = weight_0;

  for (int i=1; i<2*n_aug+1; ++i) {  
    weights(i) = weight;
  }

  /**
   * Student part begin
   */

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = (*Xsig_pred)(0,i);
    double p_y = (*Xsig_pred)(1,i);
    double v  = (*Xsig_pred)(2,i);
    double yaw = (*Xsig_pred)(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    (*Zsig)(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    (*Zsig)(1,i) = atan2(p_y,p_x);                                // phi
    (*Zsig)(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // create vector for predicted state
  VectorXd ztemp = VectorXd(n_z);

  // create covariance matrix for prediction
  MatrixXd Stemp = MatrixXd(n_z, n_z);

  // mean predicted measurement
  ztemp.fill(0.0);
  for (int i=0; i < 2*n_aug+1; ++i) {
    ztemp = ztemp + weights(i) * (*Zsig).col(i);
  }

  // innovation covariance matrix S
  Stemp.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = (*Zsig).col(i) - ztemp;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    Stemp = Stemp + weights(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr*std_radr, 0, 0,
        0, std_radphi*std_radphi, 0,
        0, 0,std_radrd*std_radrd;
  Stemp = Stemp + R;

  /**
   * Student part end
   */

  // write result
  *z_out = ztemp;
  *S_out = Stemp;
}

void UKF::UpdateState(VectorXd* x, MatrixXd* P,VectorXd* x_out, MatrixXd* P_out, MatrixXd* Xsig_pred, 
                        VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig, VectorXd* z_new ) {

  // set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  double weight = 0.5/(lambda+n_aug);
  weights(0) = weight_0;

  for (int i=1; i<2*n_aug+1; ++i) {  
    weights(i) = weight;
  }

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  /**
   * Student part begin
   */

  // calculate cross correlation matrix
  Tc.fill(0.0);

  MatrixXd Xsig_pred_new(n_x, 2*n_aug+1);
  for (int i = 0; i < n_x; i++)
  {
    Xsig_pred_new.row(i) = (*Xsig_pred).row(i);
  }
  
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = (*Zsig).col(i) - (*z_out);
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_new.col(i) - (*x);
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * (*S_out).inverse();

  // residual
  VectorXd z_diff = (*z_out) - (*z_new);

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  (*x) = (*x) + K * z_diff;
  (*P) = (*P) - K*(*S_out)*K.transpose();

  /**
   * Student part end
  */

  // write result
  *x_out = *x;
  *P_out = *P;
}
