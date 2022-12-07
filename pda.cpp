#include "pda.h"
#include "kf.h"
#include <iostream>
#include<cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


PDA::PDA() {

	/* 因为传感器与预测值的残差符合卡方分布，所以lamd为卡方自由度 */
	lamd = 16;
	/* lamd对应的门概率PG */
	PG = 0.9997;
	PD = 1;

	H_ = MatrixXd(2, 4);
	H_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	F_ = MatrixXd(4, 4);
	F_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	float dt = 200 / 1000000.0;
	F_(0, 2) = dt;
	F_(1, 3) = dt;

	

	R_ = MatrixXd(2, 2);
	R_ << 0.0225, 0,
		0, 0.0225;



}

PDA::~PDA() {

}




void PDA::pda_distance (const int &z, const VectorXd& X, const MatrixXd& P, const MeasurementPackage& measurement_pack) {

	cout << "量测" << z << "是否落入波门？" << endl;


	x_ = VectorXd(4);
	x_ << X;
	
	P_ = MatrixXd(4, 4);
	P_ << P;

	VectorXd z_pred = H_ * x_;
	         y = (measurement_pack.raw_measurements_) - z_pred;
	MatrixXd Ht = H_.transpose();
	VectorXd yt = y.transpose();
	         S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();

    distance = y.transpose() * Si * y;
// 	distance = abs(distance);
	 e__= exp(( - 0.5)*distance);
	
	 cout << "ej =" << e__ << endl;
	  cout << "dis = " << distance << endl;

	 /* cout << "y = " << y << endl;
	  cout << "si = " << Si << endl;*/

}

void PDA::pda_probability (const int& m,  double& e, const double& e_, const VectorXd& z_ ) {

	cout << "第" << m << "个量测点的概率" << endl;
	
	double det=  S(0, 0) * S(1, 1) -  S(0, 1)  * S(1, 0);
	//det = abs(det);
	double det_ = (2 * 3.14 * S(0,0))  * ( 2 * 3.14 * S(1,1))  -  (2 * 3.14 * S(0,1))  *  (2 * 3.14 * S(1,0));
	//det_ = abs(det_);

//	cout << "det = " << det << endl;
	V = 3.14 * lamd * sqrt(det); // 相关波门体积

//	cout << "V = " << V << endl;

    b = (m / V)*sqrt(det_)*(1-PD*PG)/PD;

//	cout << "b = " << b << endl;

	beta = e_ / (b + e);
	cout << "beta k----- =" << beta << endl;
}