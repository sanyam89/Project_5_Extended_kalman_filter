#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::cin;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  //cout<<"BEFORE Prediction"<<endl;
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
  cout<<"Predicted: "<<endl;
  // cout << "x_ = " << x_ << endl;
  // cout << "P_ = " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y_ = z - H_*x_;
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  
  x_ = x_ + K_*y_;
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
  P_ = (I_ - K_*H_)*P_;
  cout<<"Updated by LIDAR"<<endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  float rho = sqrt(pow(x_[0],2)+pow(x_[1],2));
  float theta = atan2(x_[1],x_[0]);
  float rho_dot = (x_[0]*x_[2] + x_[1]*x_[3])/rho; //0;
  
  VectorXd H_ekf = VectorXd(3);
  H_ekf << rho, theta, rho_dot;
  VectorXd y_ = z - H_ekf;
  // Normalizing theta to be between -pi and pi
  if (y_[1] > M_PI){
    y_[1]-=(2*M_PI);
  }
  if (y_[1] < -M_PI){
    y_[1]+=(2*M_PI);
  }
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  
  x_ = x_ + K_*y_;
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
  P_ = (I_ - K_*H_)*P_;
  
  cout<<"Updated by RADAR"<<endl;
}
