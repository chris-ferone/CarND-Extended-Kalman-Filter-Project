#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;
	x_ = x_ + (K * y);
	MatrixXd I;
	I = MatrixXd::Identity(4, 4);
	P_ = (I - K * H_) * P_;
	
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	Tools tools;
	
	MatrixXd Hj = tools.CalculateJacobian(x_);
	
	VectorXd hx(3);
	hx << sqrt(pow(x_(0), 2) + pow(x_(1),2)), atan2(x_(1),x_(0)), (x_(0)*x_(2)+x_(1)*x_(3))/sqrt(pow(x_(0), 2) + pow(x_(1),2));
	
  	VectorXd y = z -  hx;
	if (y(1) >= PI)
	{
	//cout << "OOR+" <<endl; 
	y(1) = 2*PI - y(1);
	}	
	if (y(1) <= -1*PI)
	{
	//cout << "OOR-" <<endl;
	y(1) = y(1) + 2*PI;
	}
	MatrixXd Hjt = Hj.transpose();
	MatrixXd S = Hj * P_ * Hjt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Hjt * Si;
	
	x_ = x_ + (K * y);
	MatrixXd I;
	I = MatrixXd::Identity(4, 4);
	P_ = (I - K * Hj) * P_;
	
}
