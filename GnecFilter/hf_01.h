#pragma once

#include <Eigen\core>
#include <Eigen\Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::Array;

class HeadFilter1
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

	public:
		struct MatrixSet
		{
			MatrixXd  K1_MatrixXd;
			MatrixXd  K1_matriz2;
			MatrixXd  K1_e;

			//double[, ] K1_MatrixXd = new double[500, 9];
			//double[, ] K1_matriz2 = new double[500, 9];

			// 4x4 matrixes
			MatrixXd K1_ss;

			// 3x3 matrixes
			MatrixXd K1_rg1;
			MatrixXd K1_rg2;
			MatrixXd K1_matriz_aux1;
			MatrixXd K1_matriz_aux2;
			MatrixXd K1_matriz_aux3;
			MatrixXd K1_rginv;
			MatrixXd K1_rgs;
			MatrixXd K1_rgs2;
			//MatrixXd K1_sensor1MatrixXd;
			MatrixXd K1_sensor1matriz;

			// 2x2 matrixes
			MatrixXd K1_VVnew;
			MatrixXd K1_VVnew2;
			MatrixXd K1_Sinv;
			MatrixXd K1_Vnew;
			MatrixXd K1_xVnew;
			MatrixXd K1_yVnew;
			MatrixXd K1_Vnew2;
			MatrixXd K1_xVnew2;
			MatrixXd K1_yVnew2;
			MatrixXd K1_Vnew_aux1;
			MatrixXd K1_Vnew_aux2;
			MatrixXd K1_F;
			MatrixXd K1_limit;
			MatrixXd K1_xVpred;
			MatrixXd K1_yVpred;
			MatrixXd K1_Q;
			MatrixXd K1_Q2;
			MatrixXd K1_xprevV;
			MatrixXd K1_yprevV;
			MatrixXd K1_xprevV2;
			MatrixXd K1_yprevV2;
			MatrixXd K1_xVpred2;
			MatrixXd K1_yVpred2;

			//2x1
			MatrixXd K1_xnew_aux2;
			MatrixXd K1_KK;
			MatrixXd K1_C;
			MatrixXd K1_Maux;
			MatrixXd K1_B;
			MatrixXd K1_xprevx;
			MatrixXd K1_yprevx;
			MatrixXd K1_xprevx2;
			MatrixXd K1_yprevx2;
			MatrixXd K1_xxpred;
			MatrixXd K1_xxpred2;
			MatrixXd K1_xxpred_kalman;
			MatrixXd K1_yxpred_kalman;
			MatrixXd K1_xxpred_kalman2;
			MatrixXd K1_yxpred_kalman2;
			MatrixXd K1_matriz_signo;
			MatrixXd K1_xxnew;
			MatrixXd K1_yxnew;
			MatrixXd K1_xxnew2;
			MatrixXd K1_yxnew2;
			MatrixXd K1_xnew_aux1;

			//1x1 
			MatrixXd K1_S;
			//MatrixXd e;
			MatrixXd K1_residuo;
			MatrixXd K1_R;
			MatrixXd K1_R2;
		} myMatrixSet;

	// methods for headfilter1
	void initializeMatrixes(void);
	MatrixXd  initializeMatrixes2Zero(MatrixXd M, int m, int n);
	void  initializeHeadFilter(float screenWitdh, float screenHeight);
	float  headFilterX(double inputX);
	float  headFilterY(double inputY);


	HeadFilter1(float width, float height);
	HeadFilter1(const HeadFilter1 &obj);
	~HeadFilter1(void);



};