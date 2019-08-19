#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if ((estimations.size() >0) && (ground_truth.size() == estimations.size()))
  {
  //  * the estimation vector size should equal ground truth vector size
  // TODO: accumulate squared residuals
        for (int i=0; (unsigned)i < estimations.size(); ++i) {
          VectorXd del = estimations[i]-ground_truth[i];
          del = del.array()*del.array();
          rmse+=del;
        }
        rmse = rmse/estimations.size();
        rmse = rmse.array().sqrt();
  }
  else
    cout<<"Vectors not equal";
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
  		0,0,0,0,
  		0,0,0,0;
  // recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // TODO: YOUR CODE HERE 
  float den = px*px + py*py;
  if (fabs(den) <0.0001){
    den = 0.0001;
    cout << "ERROR - CalculateJacobian () - Division by Zero" << endl;
	//return Hj;
  }
  float a00 = px/sqrt(den);
  float a01 = py/sqrt(den);
  float a02 = 0;
  float a03 = 0;
  float a10 = -py/den;
  float a11 = px/den;
  float a12 = 0;
  float a13 = 0; 
  float a20 = py*(vx*py-vy*px)/ pow(den,3/2);
  float a21 = px*(-vx*py+vy*px)/ pow(den,3/2);
  float a22 = px/sqrt(den);
  float a23 = py/sqrt(den);


  // check division by zero
  //if(fabs(px*px + py*py) < 0.0001){
	//	cout << "ERROR - CalculateJacobian () - Division by Zero" << endl;
	//	return Hj;
	//}
  // compute the Jacobian matrix
    Hj << a00,a01,a02,a03,
        a10,a11,a12,a13,
        a20,a21,a22,a23;
 

  return Hj;
}
