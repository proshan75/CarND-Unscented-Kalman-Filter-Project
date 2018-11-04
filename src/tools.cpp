#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    // ... your code here
    if (estimations.size() == 0 || estimations.size() != ground_truth.size())
    {
        cout << "estimation size cannot be zero.";
    }

    //accumulate squared residuals
    int size = estimations.size();
    VectorXd residual(4), squared_residual(4), total_residual(4);
    residual << 0, 0, 0, 0;
    squared_residual << 0, 0, 0, 0;
    total_residual << 0, 0, 0, 0;
    for (int i = 0; i < size; ++i) {
        // ... your code here
        residual = estimations[i] - ground_truth[i];
        squared_residual = (residual.array() * residual.array());
        total_residual = total_residual + squared_residual;
    }

    //calculate the mean
    // ... your code here
    VectorXd mean_residual = total_residual / (1.0 * size);

    //calculate the squared root
    // ... your code here
    rmse = mean_residual.array().sqrt();

    //return the result
    return rmse;
}

void Tools::PerformAngleNormalization(VectorXd diffVector, int index)
{
    //angle normalization
    while (diffVector(index) > M_PI) diffVector(index) -= 2.*M_PI;
    while (diffVector(index) < -M_PI) diffVector(index) += 2.*M_PI;
}
