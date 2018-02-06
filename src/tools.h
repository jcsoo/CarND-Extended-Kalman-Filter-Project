#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
MatrixXd CalculateJacobian(const VectorXd& x_state);

VectorXd ToPolar(const VectorXd& x);
VectorXd ToCartesian(const VectorXd& x);

#endif /* TOOLS_H_ */
