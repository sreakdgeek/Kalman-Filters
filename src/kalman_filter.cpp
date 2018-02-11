#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
    	* predict the state
  	*/
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  	/**
    	* update the state by using Kalman Filter equations
  	*/
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  	/**
    	* update the state by using Extended Kalman Filter equations
        * We do not compute the jacobian but directly use h(x') function to translate cartesian to polar coordinates.
  	*/

        double sqrt_term = sqrt(x_(0) * x_(0) + x_(1) * x_(1));

	double rho = sqrt_term;
	double phi = atan2(x_(1), x_(0)); 
	double rho_dot = ((x_(0) * x_(2) + x_(1) * x_(3)) / sqrt_term);

        VectorXd z_pred = VectorXd(3); 
	z_pred << rho, phi, rho_dot;

        VectorXd y = z - z_pred;

        while (y(1) < -M_PI || y(1) > M_PI)
	{
		if (y(1) < -M_PI) {
			y(1) += M_PI;
		}
		else if (y(1) > M_PI) {
			y(1) -= M_PI;
		}
	}

        MatrixXd Ht = H_.transpose();

        MatrixXd S = H_ * P_ * Ht + R_;
        MatrixXd Si = S.inverse();
        MatrixXd PHt = P_ * Ht;
        MatrixXd K = PHt * Si;

        //new estimate
        x_ = x_ + (K * y);
        long x_size = x_.size();
        MatrixXd I = MatrixXd::Identity(x_size, x_size);
        P_ = (I - K * H_) * P_;
}
