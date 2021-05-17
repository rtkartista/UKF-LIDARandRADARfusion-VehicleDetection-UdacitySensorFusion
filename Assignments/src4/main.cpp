#include <iostream>
#include "../Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
int main() {

  // Create a UKF instance
  UKF ukf;

  MatrixXd Xsig_aug = MatrixXd(7, 15);
  MatrixXd Xsig_pred = MatrixXd(7, 15);
  MatrixXd Pout = MatrixXd(5, 5);
  VectorXd Xout = VectorXd(5);
  MatrixXd Sout = MatrixXd(5, 5);
  VectorXd zout = VectorXd(5);
  MatrixXd Pupdated = MatrixXd(5, 5);
  VectorXd Xupdated = VectorXd(5);
  MatrixXd Zsig = MatrixXd(3, 2 * 7 + 1);
  MatrixXd S = MatrixXd(3,3);
  
  // create example vector for predicted state mean
  VectorXd x = VectorXd(5);
  x <<
     5.93637,
     1.49035,
     2.20528,
    0.536853,
    0.353577;

  // create example matrix for predicted state covariance
  MatrixXd P = MatrixXd(5,5);
  P <<
    0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
    -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
    0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
   -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
   -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

  // create example vector for incoming radar measurement
  VectorXd z = VectorXd(3);
  z <<
     5.9214,   // rho in m
     0.2187,   // phi in rad
     2.0062;   // rho_dot in m/s

  ukf.AugmentedSigmaPoints(&x, &P, &Xsig_aug);
  std::cout << "X sig aug = " << std::endl << Xsig_aug << std::endl;

  ukf.SigmaPointPrediction(&Xsig_pred, &Xsig_aug);
  std::cout << "X sig predicted = " << std::endl << Xsig_pred << std::endl;

  ukf.PredictMeanAndCovariance( &Xout, &Pout, &Xsig_pred);
  std::cout << "X mean = " << std::endl << Xout << std::endl;

  ukf.PredictRadarMeasurement(&zout, &Sout, &Xsig_pred, &Zsig);
  std::cout << "Z measured = " << std::endl << zout << std::endl;

  ukf.UpdateState(&Xout, &Pout, &Xupdated, &Pupdated, &Xsig_pred, &zout, &Sout,&Zsig, &z);
  std::cout << "X updated = " << std::endl << Xupdated << std::endl;

  return 0;
}
