#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Dense>
#include <iostream>

class KalmanFilter
{
public:

    /**
	* Create a Kalman filter with the specified matrices.
	*   F - System dynamics matrix/ State Transition Matrix
	*   B - Input matrix
	*   H - Measurement Function
	*   Q - Process noise covariance
	*   R - Measurement noise covariance (measurement uncertainty)
	*   P - Estimate error covariance (motion uncertainty)
    *   u - External Motion
	*   m - Number of states
	*	n - Number of measurements
	*	c - Number of Control Inputs [B.cols()]
	*/
	KalmanFilter(float deltaT);

    //Initialize the filter with initial states as zero.
	void init();

	//Initialize the filter with a guess for initial states.
	void init(const Eigen::VectorXd x0);

	//Update the prediction based on control input.
	void predict(float delT);

	//Update the estimated state based on measured values.
	void update(Eigen::VectorXd y);

	Eigen::VectorXd state() { return x_hat; };

private:

	// Matrices for computation
	Eigen::Matrix4d F, H, Q, R, P, K, P0;
	Eigen::VectorXd u;

	// System dimensions
	int m, n, c;

	// Is the filter initialized?
	bool initialized = false;

	// delta time between updates
	float delT;

	// n-size identity
	Eigen::MatrixXd I;

	// Estimated states
	Eigen::VectorXd x_hat;
};

#endif