#ifndef PDA_H_
#define PDA_H_
#include "Dense"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class PDA {
public:



	/**
	 * Constructor
	 */
	PDA();

	/**
	 * Destructor
	 */
	virtual ~PDA();


	void pda_distance (const int& z , const VectorXd& X, const MatrixXd& P, const MeasurementPackage& measurement_pack);
	void pda_probability(const int& m,  double& e, const double& e_, const VectorXd& z_ );

	int lamd;
	float PG;
	float PD;
	double beta;
	double beta0;
	double V;
	VectorXd y;
	VectorXd x_;

	// state covariance matrix
	MatrixXd P_;

	// state transistion matrix
	MatrixXd F_;

	// process covariance matrix
	MatrixXd Q_;

	// measurement matrix
	MatrixXd H_;

	// measurement covariance matrix
	MatrixXd R_;

	MatrixXd S;
	double distance;
	double distance_;
	double e__;
	double b;
	
private:
	bool is_initialized_;
};




#endif  // PDA_H_

