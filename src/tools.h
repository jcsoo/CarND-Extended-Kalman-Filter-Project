#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
MatrixXd CalculateJacobian(const VectorXd& x_state);

Vector3d ToPolar(const Vector4d& c);
Vector4d ToCartesian(const Vector3d& p);

#endif /* TOOLS_H_ */
