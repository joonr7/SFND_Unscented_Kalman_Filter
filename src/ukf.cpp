#include "ukf.h"
#include "Eigen/Dense"

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
  std_a_ = 3; // moving car: higher is better, right lane car: smaller is better.

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 6;
  
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
  
  time_us_ = 0; // long long, time when the state is true, in us
  is_initialized_ = false;
  
  // int, State dimension
  n_x_ = 5;

  // int, Augmented state dimension
  n_aug_ = 7;

  // double, Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1); // Eigen::MatrixXd, predicted sigma points matrix 

  weights_ = VectorXd(2*n_aug_+1); // Eigen::VectorXd  Weights of sigma points

  
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if(!is_initialized_){
    double px, py;
    // Initialize the state x_
    if(meas_package.sensor_type_ == meas_package.LASER && use_laser_){
      // std::cout << "LASER INIT" << std::endl;
      // std::cout << "values: ";
      // for(int i=0; i<meas_package.raw_measurements_.size(); i++){
      //   std::cout << meas_package.raw_measurements_[i] << " ";
      // }
      // std::cout << std::endl;
      px = meas_package.raw_measurements_(0);
      py = meas_package.raw_measurements_(1);
      
      x_ << px,py,0,0,0;
    }

    else if(meas_package.sensor_type_ == meas_package.RADAR && use_radar_){
      // std::cout << "RADAR INIT" << std::endl;
      double rho = meas_package.raw_measurements_(0); // radial distance
      double phi = meas_package.raw_measurements_(1); // angle between x
      
      px = rho * cos(phi);
      py = rho * sin(phi);

      x_ << px, py, 0, 0, 0;
    }
    
    else{
      std::cout << "ERROR: initialize fail. no sensor data input." << std::endl;
      return;
    }

    // Initialize the covariance P_
    P_ << 1,0,0,0,0,
          0,1,0,0,0,
          0,0,1,0,0,
          0,0,0,1,0,
          0,0,0,0,1;
    
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    
    std::cout << "Init x: " << x_.transpose() << std::endl;  
  } // end initialization
  
  if(meas_package.sensor_type_ == meas_package.LASER && use_laser_)
  {
    double delta_t = (meas_package.timestamp_ - time_us_) / 1e6;
    std::cout << delta_t << std::endl;
    time_us_ = meas_package.timestamp_; // update time
    Prediction(delta_t);
    UpdateLidar(meas_package);
  }
  else if(meas_package.RADAR && use_radar_){
    double delta_t = (meas_package.timestamp_ - time_us_) / 1e6;
    time_us_ = meas_package.timestamp_; // update time
    Prediction(delta_t);
    UpdateRadar(meas_package);
  }
  else{
    std::cout << "ERROR: initialize fail. no sensor data input." << std::endl;
      return;
  }
  std::cout << "x(k|k): " << x_.transpose() << std::endl;
  std::cout << "estimated x:" << x_[0] << std::endl;
  std::cout << "estimated y:" << x_[1] << std::endl;
  std::cout << "estimated vx:" << cos(x_[3])*x_[2] << std::endl;
  std::cout << "estimated vy:" << sin(x_[3])*x_[2] << std::endl;

		// viewer->addText("gt:", 930, 300, 20, 1, 1, 1, "gt");
		// viewer->addText(" X: "+std::to_string( traffic[0].position.x), 930, 275, 20, 1, 1, 1, "gt_x");
		// viewer->addText(" Y: "+std::to_string(traffic[0].position.y), 930, 250, 20, 1, 1, 1, "gt_y");
		// viewer->addText("Vx: "	+std::to_string(traffic[0].velocity*cos(traffic[0].angle)), 930, 225, 20, 1, 1, 1, "gt_vx");
		// viewer->addText("Vy: "	+std::to_string(traffic[0].velocity*sin(traffic[0].angle)), 930, 200, 20, 1, 1, 1, "gt_vy");


		// viewer->addText("est:", 1230, 300, 20, 1, 1, 1, "est");
		// viewer->addText(" X: "+std::to_string(traffic[0].ukf.x_[0]), 1130, 275, 20, 1, 1, 1, "est_x");
		// viewer->addText(" Y: "+std::to_string(traffic[0].ukf.x_[1]), 1130, 250, 20, 1, 1, 1, "est_y");
		// viewer->addText("Vx: "	+std::to_string(cos(traffic[0].ukf.x_(3))*traffic[0].ukf.x_(2)), 1130, 225, 20, 1, 1, 1, "est_vx");
		// viewer->addText("Vy: "	+std::to_string(sin(traffic[0].ukf.x_(3))*traffic[0].ukf.x_(2)), 1130, 200, 20, 1, 1, 1, "est_vy");
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // Generate sigma points.
  VectorXd x_aug = VectorXd(n_aug_); // augmented mean vector
  x_aug << x_, 0, 0; // nu ~ N(0, sigma^2)

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_); // augmented state covariance
  P_aug.setZero();
  P_aug.block(0,0,5,5) << P_;
  P_aug.block(5,5,2,2) << std_a_*std_a_, 0, 0, std_yawdd_*std_yawdd_;

  MatrixXd A_aug = P_aug.llt().matrixL(); // square root matrix (P_aug = A_aug * A_aug.transpose())
  
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1); // sigma point matrix
  Xsig_aug.col(0) = x_aug;
  
  for(int i=0; i<n_aug_; i++){
      Xsig_aug.col(i+1)          = x_aug + sqrt(lambda_+n_aug_)*A_aug.col(i);
      Xsig_aug.col(n_aug_+i+1)   = x_aug - sqrt(lambda_+n_aug_)*A_aug.col(i);
  }

  // std::cout << "Xsig_aug: " << std::endl << Xsig_aug << std::endl;
  
  // Predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++) {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

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
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  
  // Predict mean and covariance
  // calculate weight 
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for(int i=1; i<2*n_aug_+1; i++) {  // 2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }
  
  x_.setZero();
  // predicted state mean
  for(int i=0; i < 2*n_aug_+1; i++) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  
  P_.setZero();
  // predicted state covariance matrix
  for(int i=0; i < 2*n_aug_+1; i++) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = VectorXd(n_x_);
    x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  // std::cout << "x(k+1|k): " << x_.transpose() << std::endl;
  // std::cout << "Xsig_pred: " << std::endl << Xsig_pred_ << std::endl;
}


void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  std::cout << "\nUpdateLidar" << std:: endl;
  // std::cout << "Lidar meas data size: " << meas_package.raw_measurements_.size() << std::endl;
  // std::cout << "values: ";
  // for(int i=0; i<meas_package.raw_measurements_.size(); i++){
  //   std::cout << meas_package.raw_measurements_[i] << " ";
  // }
  // std::cout << std::endl;

  int n_z = 2; 
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1;i++) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  // std::cout << "Zsig: " << std::endl << Zsig << std::endl;

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
  S = S + R;
  
  /* ************************************ */
  /* **********    UPDATE    ************ */
  /* ************************************ */
  
  double laspx = meas_package.raw_measurements_[0]; // m
  double laspy = meas_package.raw_measurements_[1]; // m
  VectorXd z = VectorXd(n_z);
  z << laspx, laspy; 

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1;i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  // std::cout << "x(k+1|k+1): " << x_.transpose() <<std::endl;
  // std::cout << "NIS(Lidar): " << z_diff.transpose() * S.inverse() * z_diff << std::endl<<std::endl;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  std::cout << "\nUpdateRadar" << std:: endl;
  // std::cout << "Radar meas data size: " << meas_package.raw_measurements_.size() << std::endl;
  // std::cout << "values: ";
  // for(int i=0; i<meas_package.raw_measurements_.size(); i++){
  //   std::cout << meas_package.raw_measurements_[i] << " ";
  // }
  // std::cout << std::endl;
  int n_z = 3; 
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  // transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1;i++) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // std::cout << "Zsig: " << std::endl << Zsig << std::endl;

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;
  S = S + R;
  
  /* ************************************ */
  /* **********    UPDATE    ************ */
  /* ************************************ */
  
  double radr   = meas_package.raw_measurements_[0]; // rho in m
  double radphi = meas_package.raw_measurements_[1]; // phi in rad
  double radrd  = meas_package.raw_measurements_[2]; // rho_dot in m/s
  VectorXd z = VectorXd(n_z);
  z << radr, radphi, radrd; 

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1;i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  // std::cout << "x(k+1|k+1): " << x_.transpose() <<std::endl;
  // std::cout << "NIS(Radar): " << z_diff.transpose() * S.inverse() * z_diff << std::endl<<std::endl;
}