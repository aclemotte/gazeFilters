#pragma once

#include <Eigen/core>
#include <Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::Array;

class HeadFilter2
{
public:
	double K1_cov_omega;
	double K1_cov_nu;
	double K1_Ts, K1_b;
	//double K1_norm_residuo;

	// variables de headFilter2
	float K2_Ts, K2_b;
	float K2_H, K2_Q, K2_R, K2_QB, K2_RB;//K2_F
	float K2_xprevx2, K2_yprevx2, K2_cov_omega, K2_cov_nu, K2_x_pred, K2_y_pred, K2_x_Vpred, K2_y_Vpred;//K2_x_Vpred2 K2_y_Vpred2 K2_xprevV2, K2_yprevV2
	float K2_cte1, K2_cte2, K2_R2, K2_Q2, K2_xVpred2, K2_e, K2_S, K2_KK, K2_residuo, K2_re, K2_z, K2_yxnew2, K2_yVpred2, K2_yVnew2, K2_xxnew2, K2_xVnew2;

private:
	int *ptr;

public:
	struct EnlazaControlInfo
	{
		char typeOfControl;
		char kalmanOn = 0;
		float kalman_b;
		float sampFreq;
		bool lateralControl;
		float horizontalRange;
		float verticalRange;
		float screenWidth;
		float screenheight;
	} enlazaSetup;


	// methods for headfilter2


	MatrixXd  initializeMatrixes(MatrixXd M, int m, int n);
	void  initializeHeadFilter(float screenWitdh, float screenHeight);
	float  headFilterX(double inputX);
	float  headFilterY(double inputY);
	double headFilterXconstrained(EnlazaControlInfo setup, double posX, double tabW);
	double headFilterYconstrained(EnlazaControlInfo setup, double posY, double tabH);

	HeadFilter2(float width, float height);
	HeadFilter2(const HeadFilter2 &obj);
	~HeadFilter2(void);



};