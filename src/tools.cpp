#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        VectorXd r = estimations[i] - ground_truth[i];
        r = r.array() * r.array();
        rmse += r;
	}
	// calculate the mean
	rmse /= estimations.size();
  
	// calculate the squared root
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	const float EPSILON = 0.0001;
	MatrixXd Hj = MatrixXd::Zero(3, 4);

	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation	
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = c1 * c2;

	if (c2 >= EPSILON) {
		//compute the Jacobian matrix
		float h11 = px / c2;
		float h12 = py / c2;
		float h21 = -py / c1;
		float h22 = px / c1;
		float h31 = py * (vx * py - vy * px ) / c3;
		float h32 = px * (vy * px - vx * py) / c3;

		Hj << h11, h12,   0,   0,
		      h21, h22,   0,   0,
			  h31, h32, h11, h11;
	}
      
	return Hj;
}

VectorXd Tools::to_polar(const VectorXd& c) {
	const float EPSILON = 0.0001;
	
	VectorXd p(3);

	float px = c(0);
	float py = c(1);
	float vx = c(2);
	float vy = c(3);

    float rho = sqrt(px * px + py * py);
    float theta = atan2(py, px);
	float rho_dot = (rho > EPSILON) ? (px * vx + py * vy) / rho : 0.0;

	p << rho, theta, rho_dot;
	return p;	
}

VectorXd Tools::to_cartesian(const VectorXd& p) {
	VectorXd c(4);

	std::cout << "to_cartesian: " << p << std::endl;

	float rho = p(0);
	float theta = p(1);
	float rho_dot = p(2);

	float px = rho * cos(theta);
	float py = rho * sin(theta);
	float vx = rho_dot * cos(theta);
	float vy = rho_dot * sin(theta);

	c << px, py, vx, vy;
	std::cout << "c" << c << std::endl;
	return c;
}
