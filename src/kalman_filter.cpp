#include "kalman_filter.h"

KalmanFilter::KalmanFilter(float delT): delT(delT)
    /**
	* Create a Kalman filter with the specified matrices.
	*   F - System dynamics matrix/ State Transition Matrix
	*   B - Input matrix
	*   H - Measurement Function
	*   Q - Process noise covariance
	*   R - Measurement noise covariance (measurement uncertainty)
	*   P - Estimate error covariance (motion uncertainty)
    *   u - External Motion
	*   m - Number of states         [H.rows()]
	*	n - Number of measurements   [F.rows()]
	*	c - Number of Control Inputs [B.cols()]
	*/   
{
	int n = 4; // Number of states
	int m = 4; // Number of measurements
	int c = 1; // Number of control inputs

	// Initialize the matrices

	H << 1,0,1,0,
		0,1,0,1,
		0,0,0,0,
		0,0,0,0;

	P0 << 0,0,0,0,
		0,0,0,0,
		0,0,0,0,
		0,0,0,0;

	//u.setZero();

	//the delta t is taken as constant as of now
	F << 1.0, 0,   delT,  0,
		0,   1.0, 0,     delT,
		0,   0,   1,     0,
		0,   0,   0,     1;

	Q << 0.1, 0,   0,   0,
		   0, 0.1, 0,   0,
	       0, 0,   0.1, 0,
		   0, 0,   0,   0.1;

	R << 0.1,   0, 0,   0,
		   0, 0.1, 0,   0,
		   0,   0, 0.1, 0,
		   0,   0, 0,   0.1;
		 
	I.setIdentity(n,n);
	initialized = false;

	//std::cout<< "  Kalman Filter Constructed" << std::endl;
}

void KalmanFilter::init(const Eigen::VectorXd x0) 
{
	// Initialize the variance and state with initial value given
	x_hat = x0;
	//std::cout << "  Kalman Filter Initialized" << x_hat.transpose() << std::endl;
	P = P0;
	initialized = true;
}

void KalmanFilter::init() 
{
	// Initialize the variance and state with zeros
	x_hat.setZero();
	P = P0;
	initialized = true;
}

void KalmanFilter::predict(float deltaT) 
{
	delT = deltaT;
	if(!initialized) {
		std::cout << "Filter is not initialized! Initializing with trivial state.\n";
		init();
	}
	// Predict the state and variance based on motion
	x_hat = F*x_hat ; //+ u;
	P = F*P*F.transpose() + Q;
}

void KalmanFilter::update(Eigen::VectorXd y) {
	// Update the state and variance based on y(Measurement)
	K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
	x_hat += K * (y - H*x_hat);
	P = (I - K*H)*P;
}
