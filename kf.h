
#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
	/**
	 * Constructor
	 */
	KalmanFilter();

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter();

	/**
	 * Predict Predicts the state and the state covariance
	 *   using the process model
	 */



	void Predict(const int& T, const VectorXd& X, const MatrixXd& P);

	/**
	 * Updates the state and
	 * @param z The measurement at k+1
	 */
	void Update1(const VectorXd& z, const double& bta0, const MatrixXd& vs, const VectorXd& v);

	void Update2( const double& bta0, const MatrixXd& vs, const VectorXd& v);

	// state vector
	VectorXd pre_x_;
	VectorXd x_;
	// state covariance matrix
	MatrixXd pre_P_;
	MatrixXd p_c;
	MatrixXd p_r;
	MatrixXd p_;
	// state transistion matrix
	MatrixXd F_;

	// process covariance matrix
	MatrixXd Q_;

	// measurement matrix
	MatrixXd H_;

	// measurement covariance matrix
	MatrixXd R_;

private:

	float noise_ax;
	float noise_ay;

};

#endif  // KALMAN_FILTER_H_
