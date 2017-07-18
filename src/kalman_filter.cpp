#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

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
  P_ = F_ * P_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	//cout << "1" << endl;
	//cout << "H_: " << endl;
	//cout << H_ << endl;
	//cout << "x: "<< endl;
	//cout << x_ << endl;
	VectorXd y = z - H_ * x_;
	//cout << "2" << endl;
	MatrixXd Ht = H_.transpose();
	//cout << "3" << endl;
	//cout << "p: "<< endl;
	//cout << P_ << endl;
	MatrixXd S = H_ * P_ * Ht + R_;
	//cout << "4" << endl;
	MatrixXd Si = S.inverse();
	//cout << "5" << endl;
	MatrixXd K =  P_ * Ht * Si;
	//cout << "6" << endl;
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
	//cout << "z" << endl;
	Tools tools;
	
	MatrixXd Hj = tools.CalculateJacobian(x_);
	//cout << "y" << endl;
	VectorXd hx(3);
	hx << sqrt(pow(x_(0), 2) + pow(x_(1),2)), atan(x_(1)/x_(0)), (x_(0)*x_(2)+x_(1)*x_(3))/sqrt(pow(x_(0), 2) + pow(x_(1),2));
	//cout << "x" << endl;
	//cout << "z: " << endl;
	//cout << z << endl;
	//cout << "hx: "<< endl;
	//cout << hx << endl;
	
  	VectorXd y = z -  hx;
	//cout << "w" << endl;
	MatrixXd Hjt = Hj.transpose();
	//cout << "v" << endl;
	MatrixXd S = Hj * P_ * Hjt + R_;
	//cout << "u" << endl;
	MatrixXd Si = S.inverse();
	//cout << "t" << endl;
	MatrixXd K =  P_ * Hjt * Si;
	
	x_ = x_ + (K * y);
	MatrixXd I;
	I = MatrixXd::Identity(4, 4);
	P_ = (I - K * Hj) * P_;
	
}
