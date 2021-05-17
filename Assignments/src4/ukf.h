#ifndef UKF_H
#define UKF_H

#include "../Eigen/Dense"

class UKF {
 public:
  // set state dimension
  int n_x = 5;
  int n_z = 3;
  // define spreading parameter

  // set augmented dimension
  int n_aug = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  // radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  // radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  // radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  // define spreading parameter
  double lambda = 3 - n_aug;
  
  double delta_t = 0.1; // time diff in sec

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * Init Initializes Unscented Kalman filter
   */
  void Init();

  /**
   * Student assignment functions
   */
  void AugmentedSigmaPoints(Eigen::VectorXd* x, Eigen::MatrixXd* P, Eigen::MatrixXd* Xsig_out);
  void SigmaPointPrediction(Eigen::MatrixXd* Xsig_pred, Eigen::MatrixXd* Xsig_aug);
  void PredictMeanAndCovariance(Eigen::VectorXd* x_pred, Eigen::MatrixXd* P_pred, Eigen::MatrixXd* Xsig_pred);
  void PredictRadarMeasurement(Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd* Xsig_pred, Eigen::MatrixXd* Zsig);
  void UpdateState(Eigen::VectorXd* x, Eigen::MatrixXd* P, Eigen::VectorXd* x_out, Eigen::MatrixXd* P_out, Eigen::MatrixXd* Xsig_pred, 
                        Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out, Eigen::MatrixXd* Zsig, Eigen::VectorXd* z_new);

};

#endif  // UKF_H
