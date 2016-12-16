#include <Eigen/Core>
#include <Eigen/Dense>

#include "hf_01.h"

using namespace std;
using namespace Eigen;

HeadFilter1::HeadFilter1(float width, float height)
{

	initializeMatrixes();
	initializeHeadFilter(width, height);
}

HeadFilter1::HeadFilter1(const HeadFilter1 &obj)
{
	ptr = new int;
	*ptr = *obj.ptr; // copy the value
}

HeadFilter1::~HeadFilter1(void)
{

}

void HeadFilter1::initializeMatrixes(void)
{
	// initialize (1/1)
	myMatrixSet.K1_MatrixXd.resize(500, 9);
	myMatrixSet.K1_MatrixXd.resize(500, 9);
	myMatrixSet.K1_matriz2.resize(500, 9);

	// initialize matrix (1/2)
	// 4x4 matrixes
	myMatrixSet.K1_ss.resize(4, 4);

	// 3x3 matrixes
	myMatrixSet.K1_rg1.resize(3, 3);
	myMatrixSet.K1_rg2.resize(3, 3);
	myMatrixSet.K1_matriz_aux1.resize(3, 3);
	myMatrixSet.K1_matriz_aux2.resize(3, 3);
	myMatrixSet.K1_matriz_aux3.resize(3, 3);
	myMatrixSet.K1_rginv.resize(3, 3);
	myMatrixSet.K1_rgs.resize(3, 3);
	myMatrixSet.K1_rgs2.resize(3, 3);
	//K1_sensor1MatrixXd.resize(3, 3);
	myMatrixSet.K1_sensor1matriz.resize(3, 3);

	// 2x2 matrixes
	myMatrixSet.K1_VVnew.resize(2, 2);
	myMatrixSet.K1_VVnew2.resize(2, 2);
	myMatrixSet.K1_Sinv.resize(2, 2);
	myMatrixSet.K1_Vnew.resize(2, 2);
	myMatrixSet.K1_xVnew.resize(2, 2);
	myMatrixSet.K1_yVnew.resize(2, 2);
	myMatrixSet.K1_Vnew2.resize(2, 2);
	myMatrixSet.K1_xVnew2.resize(2, 2);
	myMatrixSet.K1_yVnew2.resize(2, 2);
	myMatrixSet.K1_Vnew_aux1.resize(2, 2);
	myMatrixSet.K1_Vnew_aux2.resize(2, 2);
	myMatrixSet.K1_F.resize(2, 2);
	myMatrixSet.K1_limit.resize(2, 2);
	myMatrixSet.K1_xVpred.resize(2, 2);
	myMatrixSet.K1_yVpred.resize(2, 2);
	myMatrixSet.K1_Q.resize(2, 2);
	myMatrixSet.K1_Q2.resize(2, 2);
	myMatrixSet.K1_xprevV.resize(2, 2);
	myMatrixSet.K1_yprevV.resize(2, 2);
	myMatrixSet.K1_xprevV2.resize(2, 2);
	myMatrixSet.K1_yprevV2.resize(2, 2);
	myMatrixSet.K1_xVpred2.resize(2, 2);
	myMatrixSet.K1_yVpred2.resize(2, 2);
	myMatrixSet.K1_B.resize(2, 2);

	//2x1
	myMatrixSet.K1_xnew_aux2.resize(1, 2);
	myMatrixSet.K1_KK.resize(1, 2);
	myMatrixSet.K1_C.resize(1, 2);
	myMatrixSet.K1_Maux.resize(2, 1);

	myMatrixSet.K1_xprevx.resize(1, 2);
	myMatrixSet.K1_yprevx.resize(1, 2);
	myMatrixSet.K1_xprevx2.resize(2, 1);
	myMatrixSet.K1_yprevx2.resize(2, 1);
	myMatrixSet.K1_xxpred.resize(1, 2);
	myMatrixSet.K1_xxpred2.resize(1, 2);
	myMatrixSet.K1_xxpred_kalman.resize(1, 2);
	myMatrixSet.K1_yxpred_kalman.resize(1, 2);
	myMatrixSet.K1_xxpred_kalman2.resize(1, 2);
	myMatrixSet.K1_yxpred_kalman2.resize(1, 2);
	myMatrixSet.K1_matriz_signo.resize(1, 2);
	myMatrixSet.K1_xxnew.resize(2, 1);
	myMatrixSet.K1_yxnew.resize(2, 1);
	myMatrixSet.K1_xxnew2.resize(2, 1);
	myMatrixSet.K1_yxnew2.resize(2, 1);
	myMatrixSet.K1_xnew_aux1.resize(2, 1);

	//1x1
	myMatrixSet.K1_S.resize(1, 1);
	//e.resize(1, 1);
	myMatrixSet.K1_residuo.resize(2, 1);
	myMatrixSet.K1_R.resize(1, 1);
	myMatrixSet.K1_R2.resize(1, 1);
}

MatrixXd HeadFilter1::initializeMatrixes2Zero(MatrixXd M, int m, int n)
{
	for (int i = 0; i < m ; i++)
	{
		for (int j = 0; j < n; j++)
		{
			M(i, j) = 0;
		}
	}
	return M;
}

void HeadFilter1::initializeHeadFilter(float screenWitdh, float screenHeight)
{

	enlazaSetup.sampFreq = 50;
	enlazaSetup.kalman_b = 0.3f;
	enlazaSetup.kalmanOn = 1;
	enlazaSetup.lateralControl = false;
	enlazaSetup.verticalRange = 20;
	enlazaSetup.horizontalRange = 30;
	enlazaSetup.screenheight = screenHeight;
	enlazaSetup.screenWidth = screenWitdh;

	K2_cte1 = enlazaSetup.screenheight / enlazaSetup.verticalRange;
	K2_cte2 = enlazaSetup.screenWidth / enlazaSetup.horizontalRange;

	K2_Ts = 1 / (enlazaSetup.sampFreq);
	//K2_F = 1f;
	K2_H = 1;
	K2_cov_omega = 50;     // Predicción Cuanto más alto menos filtra 50
	K2_cov_nu = 10;        // Medida 10

						   // Q
	K2_Q = K2_cov_omega;
	// R
	K2_R = K2_cov_nu;

	K2_cov_omega = 50;     // Predicción Cuanto más alto menos filtra
	K2_cov_nu = 10;        // Medida

						   // Q border
	K2_QB = K2_cov_omega;
	// R border
	K2_RB = K2_cov_nu;

	// Posición inicial en el centro de la pantalla

	//if (manual_calibration.Checked)
	//{
	K2_xprevx2 = enlazaSetup.screenWidth / 2;
	K2_yprevx2 = enlazaSetup.screenheight / 2;
	//}
	//else
	//{
	//    xprevx2 = 0;
	//    yprevx2 = 0;
	//}

	//b = 0.3;
	K2_b = enlazaSetup.kalman_b;
	//b = Convert.ToDouble(trackBar_kalman.Value);
	//b = b / 100;
	// P
	//K2_xprevV2 = 10f;
	//K2_yprevV2 = 10f;

	// initialize matrix
	//4x4
	myMatrixSet.K1_ss = initializeMatrixes2Zero(myMatrixSet.K1_ss, 4, 4);


	// 3x3
	myMatrixSet.K1_rg1 = initializeMatrixes2Zero(myMatrixSet.K1_rg1, 3, 3);
	myMatrixSet.K1_rginv = initializeMatrixes2Zero(myMatrixSet.K1_rginv, 3, 3);
	myMatrixSet.K1_sensor1matriz = initializeMatrixes2Zero(myMatrixSet.K1_sensor1matriz, 3, 3);
	myMatrixSet.K1_rgs = initializeMatrixes2Zero(myMatrixSet.K1_rgs, 3, 3);
	myMatrixSet.K1_rgs2 = initializeMatrixes2Zero(myMatrixSet.K1_rgs2, 3, 3);
	myMatrixSet.K1_matriz_aux1 = initializeMatrixes2Zero(myMatrixSet.K1_matriz_aux1, 3, 3);
	myMatrixSet.K1_matriz_aux2 = initializeMatrixes2Zero(myMatrixSet.K1_matriz_aux2, 3, 3);
	myMatrixSet.K1_matriz_aux2 = initializeMatrixes2Zero(myMatrixSet.K1_matriz_aux3, 3, 3);
	myMatrixSet.K1_rg2 = initializeMatrixes2Zero(myMatrixSet.K1_rg2, 3, 3);


	//2x2
	myMatrixSet.K1_VVnew = initializeMatrixes2Zero(myMatrixSet.K1_VVnew, 2, 2);
	myMatrixSet.K1_VVnew2 = initializeMatrixes2Zero(myMatrixSet.K1_VVnew2, 2, 2);
	myMatrixSet.K1_Sinv = initializeMatrixes2Zero(myMatrixSet.K1_Sinv, 2, 2);
	myMatrixSet.K1_Vnew = initializeMatrixes2Zero(myMatrixSet.K1_Vnew, 2, 2);
	myMatrixSet.K1_xVnew = initializeMatrixes2Zero(myMatrixSet.K1_xVnew, 2, 2);
	myMatrixSet.K1_yVnew = initializeMatrixes2Zero(myMatrixSet.K1_yVnew, 2, 2);
	myMatrixSet.K1_Vnew2 = initializeMatrixes2Zero(myMatrixSet.K1_Vnew2, 2, 2);
	myMatrixSet.K1_xVnew2 = initializeMatrixes2Zero(myMatrixSet.K1_xVnew2, 2, 2);
	myMatrixSet.K1_yVnew2 = initializeMatrixes2Zero(myMatrixSet.K1_yVnew2, 2, 2);
	myMatrixSet.K1_Vnew_aux1 = initializeMatrixes2Zero(myMatrixSet.K1_Vnew_aux1, 2, 2);
	myMatrixSet.K1_Vnew_aux2 = initializeMatrixes2Zero(myMatrixSet.K1_Vnew_aux2, 2, 2);
	myMatrixSet.K1_F = initializeMatrixes2Zero(myMatrixSet.K1_F, 2, 2);
	myMatrixSet.K1_limit = initializeMatrixes2Zero(myMatrixSet.K1_limit, 2, 2);
	myMatrixSet.K1_xVpred = initializeMatrixes2Zero(myMatrixSet.K1_xVpred, 2, 2);
	myMatrixSet.K1_yVpred = initializeMatrixes2Zero(myMatrixSet.K1_yVpred, 2, 2);
	myMatrixSet.K1_Q = initializeMatrixes2Zero(myMatrixSet.K1_Q, 2, 2);
	myMatrixSet.K1_Q2 = initializeMatrixes2Zero(myMatrixSet.K1_Q2, 2, 2);
	myMatrixSet.K1_xprevV = initializeMatrixes2Zero(myMatrixSet.K1_xprevV, 2, 2);
	myMatrixSet.K1_yprevV = initializeMatrixes2Zero(myMatrixSet.K1_yprevV, 2, 2);
	myMatrixSet.K1_xprevV2 = initializeMatrixes2Zero(myMatrixSet.K1_xprevV2, 2, 2);
	myMatrixSet.K1_yprevV2 = initializeMatrixes2Zero(myMatrixSet.K1_yprevV2, 2, 2);
	myMatrixSet.K1_xVpred2 = initializeMatrixes2Zero(myMatrixSet.K1_xVpred2, 2, 2);
	myMatrixSet.K1_yVpred2 = initializeMatrixes2Zero(myMatrixSet.K1_yVpred2, 2, 2);

	//2x1
	myMatrixSet.K1_xnew_aux2 = initializeMatrixes2Zero(myMatrixSet.K1_xnew_aux2, 1, 2);
	myMatrixSet.K1_KK = initializeMatrixes2Zero(myMatrixSet.K1_KK, 1, 2);
	myMatrixSet.K1_C = initializeMatrixes2Zero(myMatrixSet.K1_C, 1, 2);
	myMatrixSet.K1_Maux = initializeMatrixes2Zero(myMatrixSet.K1_Maux, 2, 1);
	myMatrixSet.K1_B = initializeMatrixes2Zero(myMatrixSet.K1_B, 1, 2);
	myMatrixSet.K1_xprevx = initializeMatrixes2Zero(myMatrixSet.K1_xprevx, 1, 2);
	myMatrixSet.K1_yprevx = initializeMatrixes2Zero(myMatrixSet.K1_yprevx, 1, 2);
	myMatrixSet.K1_xprevx2 = initializeMatrixes2Zero(myMatrixSet.K1_xprevx2, 2, 1);
	myMatrixSet.K1_yprevx2 = initializeMatrixes2Zero(myMatrixSet.K1_yprevx2, 2, 1);
	myMatrixSet.K1_xxpred = initializeMatrixes2Zero(myMatrixSet.K1_xxpred, 1, 2);
	myMatrixSet.K1_xxpred2 = initializeMatrixes2Zero(myMatrixSet.K1_xxpred2, 1, 2);
	myMatrixSet.K1_xxpred_kalman = initializeMatrixes2Zero(myMatrixSet.K1_xxpred_kalman, 1, 2);
	myMatrixSet.K1_yxpred_kalman = initializeMatrixes2Zero(myMatrixSet.K1_yxpred_kalman, 1, 2);
	myMatrixSet.K1_xxpred_kalman2 = initializeMatrixes2Zero(myMatrixSet.K1_xxpred_kalman2, 1, 2);
	myMatrixSet.K1_yxpred_kalman2 = initializeMatrixes2Zero(myMatrixSet.K1_yxpred_kalman2, 1, 2);
	myMatrixSet.K1_matriz_signo = initializeMatrixes2Zero(myMatrixSet.K1_matriz_signo, 1, 2);
	myMatrixSet.K1_xxnew = initializeMatrixes2Zero(myMatrixSet.K1_xxnew, 2, 1);
	myMatrixSet.K1_yxnew = initializeMatrixes2Zero(myMatrixSet.K1_yxnew, 2, 1);
	myMatrixSet.K1_xxnew2 = initializeMatrixes2Zero(myMatrixSet.K1_xxnew2, 2, 1);
	myMatrixSet.K1_yxnew2 = initializeMatrixes2Zero(myMatrixSet.K1_yxnew2, 2, 1);
	myMatrixSet.K1_xnew_aux1 = initializeMatrixes2Zero(myMatrixSet.K1_xnew_aux1, 2, 1);


	//1x1
	myMatrixSet.K1_S = initializeMatrixes2Zero(myMatrixSet.K1_S, 1, 1);
	//e = initializeMatrixes2Zero(e, 1, 1);
	myMatrixSet.K1_residuo = initializeMatrixes2Zero(myMatrixSet.K1_residuo, 2, 1);
	myMatrixSet.K1_R = initializeMatrixes2Zero(myMatrixSet.K1_R, 1, 1);
	myMatrixSet.K1_R2 = initializeMatrixes2Zero(myMatrixSet.K1_R2, 1, 1);


	// valores para los filtros
	K1_Ts = 1 / (enlazaSetup.sampFreq);
	myMatrixSet.K1_F(0, 0) = 1;
	myMatrixSet.K1_F(0, 1) = K1_Ts;

	myMatrixSet.K1_F(1, 0) = 0;
	myMatrixSet.K1_F(1, 1) = 1;

	myMatrixSet.K1_B(0, 0) = pow(K1_Ts, 2); //Math.Pow(K1_Ts, 2);
	myMatrixSet.K1_B(1, 0) = K1_Ts;

	myMatrixSet.K1_C(0, 0) = 1;
	myMatrixSet.K1_C(0, 1) = 0;

	K1_cov_omega = 50;     // Predicción Cuanto más alto menos filtra
	K1_cov_nu = 10;        // Medida

	// ss
	myMatrixSet.K1_ss(0, 0) = 1;
	myMatrixSet.K1_ss(0, 1) = 0;
	myMatrixSet.K1_ss(1, 0) = 0;
	myMatrixSet.K1_ss(1, 1) = 1;

	// Q
	myMatrixSet.K1_Q2(0, 0) = pow(K1_cov_omega, 2) * pow(K1_Ts, 4) / 4;
	myMatrixSet.K1_Q2(0, 1) = pow(K1_cov_omega, 2) * pow(K1_Ts, 3) / 2;
	myMatrixSet.K1_Q2(1, 0) = pow(K1_cov_omega, 2) * pow(K1_Ts, 3) / 2;
	myMatrixSet.K1_Q2(1, 1) = pow(K1_cov_omega, 2) * pow(K1_Ts, 2);
	// R
	myMatrixSet.K1_R2(0, 0) = pow(K1_cov_nu, 2);
	myMatrixSet.K1_R(0, 0) = pow(K1_cov_nu, 2);

	// Posición inicial en el centro de la pantalla
	myMatrixSet.K1_xprevx2(0, 0) = enlazaSetup.horizontalRange / 2;
	myMatrixSet.K1_xprevx2(1, 0) = 0;
	myMatrixSet.K1_yprevx2(0, 0) = enlazaSetup.verticalRange / 2;
	myMatrixSet.K1_yprevx2(1, 0) = 0;

	K1_b = 100;

	// P
	myMatrixSet.K1_xprevV2(0, 0) = myMatrixSet.K1_Q(0, 0);
	myMatrixSet.K1_xprevV2(0, 1) = myMatrixSet.K1_Q(0, 1);

	myMatrixSet.K1_xprevV2(1, 0) = myMatrixSet.K1_Q(1, 0);
	myMatrixSet.K1_xprevV2(1, 1) = myMatrixSet.K1_Q(1, 1);

	myMatrixSet.K1_yprevV2(0, 0) = myMatrixSet.K1_Q(0, 0);
	myMatrixSet.K1_yprevV2(0, 1) = myMatrixSet.K1_Q(0, 1);

	myMatrixSet.K1_yprevV2(1, 0) = myMatrixSet.K1_Q(1, 0);
	myMatrixSet.K1_yprevV2(1, 1) = myMatrixSet.K1_Q(1, 1);
}

float HeadFilter1::headFilterX(double inputX)
{
	//// F 2x2
	//// prevx 2x1
	//// prevV 2x2
	//// xpred 2x1
	//// Vpred 2x2
	//// Q 2x2
	//// C 1x2
	//// e 1x1
	//// residuo 2x1
	//// K 2x1
	//// S 1x1
	//// xnew 2x1
	//// Vnew 2x2

	// Kalman filter equations

	// Predicción
	myMatrixSet.K1_xxpred_kalman2 = myMatrixSet.K1_F*myMatrixSet.K1_xprevx2;
	double u = 1.5;
	myMatrixSet.K1_xxpred_kalman2(0, 0) = myMatrixSet.K1_xxpred_kalman2(0, 0) + myMatrixSet.K1_B(0, 0) * u;
	myMatrixSet.K1_xxpred_kalman2(1, 0) = myMatrixSet.K1_xxpred_kalman2(1, 0) + myMatrixSet.K1_B(1, 0) * u;

	//MatrixXd aux0;
	myMatrixSet.K1_Vnew_aux1 = (myMatrixSet.K1_F*myMatrixSet.K1_xprevV2)*myMatrixSet.K1_F.transpose();
	myMatrixSet.K1_xVpred2(0, 0) = myMatrixSet.K1_Vnew_aux1(0, 0) + myMatrixSet.K1_Q2(0, 0);
	myMatrixSet.K1_xVpred2(0, 1) = myMatrixSet.K1_Vnew_aux1(0, 1) + myMatrixSet.K1_Q2(0, 1);
	myMatrixSet.K1_xVpred2(1, 0) = myMatrixSet.K1_Vnew_aux1(1, 0) + myMatrixSet.K1_Q2(1, 0);
	myMatrixSet.K1_xVpred2(1, 1) = myMatrixSet.K1_Vnew_aux1(1, 1) + myMatrixSet.K1_Q2(1, 1);

	// Actualización
	myMatrixSet.K1_e = myMatrixSet.K1_C*myMatrixSet.K1_xxpred_kalman2;
	myMatrixSet.K1_e(0, 0) = (inputX - myMatrixSet.K1_e(0, 0));

	// S=H*Vpred*H'+R
	//aux0 = myMatrixSet.K1_C*myMatrixSet.K1_xVpred2;
	myMatrixSet.K1_S = (myMatrixSet.K1_C*myMatrixSet.K1_xVpred2)*myMatrixSet.K1_C.transpose();
	myMatrixSet.K1_S(0, 0) = myMatrixSet.K1_S(0, 0) + myMatrixSet.K1_R2(0, 0);

	//
	//aux0 = myMatrixSet.K1_xVpred2*myMatrixSet.K1_C.transpose();
	myMatrixSet.K1_KK = (myMatrixSet.K1_xVpred2*myMatrixSet.K1_C.transpose())*myMatrixSet.K1_S.inverse();
	myMatrixSet.K1_residuo = myMatrixSet.K1_KK*myMatrixSet.K1_e;


	//b = 100;
	if (abs(myMatrixSet.K1_e(0, 0)) <= K1_b *pow(myMatrixSet.K1_R2(0, 0), -0.5) * myMatrixSet.K1_S(0, 0))
	{
		/*aux0 = myMatrixSet.K1_xVpred2*myMatrixSet.K1_C.transpose();*/
		myMatrixSet.K1_KK = (myMatrixSet.K1_xVpred2*myMatrixSet.K1_C.transpose())*myMatrixSet.K1_S.inverse();
		myMatrixSet.K1_xxnew2(0, 0) = myMatrixSet.K1_residuo(0, 0) + myMatrixSet.K1_xxpred_kalman2(0, 0);
		myMatrixSet.K1_xxnew2(1, 0) = myMatrixSet.K1_residuo(1, 0) + myMatrixSet.K1_xxpred_kalman2(1, 0);
	}
	else
	{

		myMatrixSet.K1_Maux = myMatrixSet.K1_xVpred2*myMatrixSet.K1_C.transpose();
		bool sign = signbit(myMatrixSet.K1_e(0, 0));
		if (sign)
		{
			myMatrixSet.K1_xxnew2(0, 0) = myMatrixSet.K1_xxpred_kalman2(0, 0) + myMatrixSet.K1_Maux(0, 0) * pow(myMatrixSet.K1_R2(0, 0), -0.5) * K1_b * -1;
			myMatrixSet.K1_xxnew2(1, 0) = myMatrixSet.K1_xxpred_kalman2(1, 0) + myMatrixSet.K1_Maux(1, 0) * pow(myMatrixSet.K1_R2(0, 0), -0.5) * K1_b * -1;
		}
		else
		{
			myMatrixSet.K1_xxnew2(0, 0) = myMatrixSet.K1_xxpred_kalman2(0, 0) + myMatrixSet.K1_Maux(0, 0) * pow(myMatrixSet.K1_R2(0, 0), -0.5) * K1_b * 1;
			myMatrixSet.K1_xxnew2(1, 0) = myMatrixSet.K1_xxpred_kalman2(1, 0) + myMatrixSet.K1_Maux(1, 0) * pow(myMatrixSet.K1_R2(0, 0), -0.5) * K1_b * 1;
		}
		
	}

	myMatrixSet.K1_Vnew_aux1 = myMatrixSet.K1_KK*myMatrixSet.K1_C;

	myMatrixSet.K1_Vnew2(0, 0) = myMatrixSet.K1_ss(0, 0) - myMatrixSet.K1_Vnew_aux1(0, 0);
	myMatrixSet.K1_Vnew2(0, 1) = myMatrixSet.K1_ss(0, 1) - myMatrixSet.K1_Vnew_aux1(0, 1);
	myMatrixSet.K1_Vnew2(1, 0) = myMatrixSet.K1_ss(1, 0) - myMatrixSet.K1_Vnew_aux1(1, 0);
	myMatrixSet.K1_Vnew2(1, 1) = myMatrixSet.K1_ss(1, 1) - myMatrixSet.K1_Vnew_aux1(1, 1);

	myMatrixSet.K1_Vnew_aux1 = myMatrixSet.K1_Vnew2;

	myMatrixSet.K1_xVnew2 = myMatrixSet.K1_Vnew_aux1*myMatrixSet.K1_xVpred2;

	myMatrixSet.K1_xprevx2 = myMatrixSet.K1_xxnew2;
	myMatrixSet.K1_xprevV2 = myMatrixSet.K1_xVnew2;

	return myMatrixSet.K1_xxnew2(0, 0);
};

float HeadFilter1::headFilterY(double inputY)
{
	//// F 2x2
	//// prevx 2x1
	//// prevV 2x2
	//// xpred 2x1
	//// Vpred 2x2
	//// Q 2x2
	//// C 1x2
	//// e 1x1
	//// residuo 2x1
	//// K 2x1
	//// S 1x1
	//// xnew 2x1
	//// Vnew 2x2

	// Kalman filter equations

	// Predicción
	myMatrixSet.K1_yxpred_kalman2 = myMatrixSet.K1_F*myMatrixSet.K1_yprevx2;
	double K1_u = 1.5;
	myMatrixSet.K1_yxpred_kalman2(0, 0) = myMatrixSet.K1_yxpred_kalman2(0, 0) + myMatrixSet.K1_B(0, 0) * K1_u;
	myMatrixSet.K1_yxpred_kalman2(1, 0) = myMatrixSet.K1_yxpred_kalman2(1, 0) + myMatrixSet.K1_B(1, 0) * K1_u;

	//MatrixXd aux0;
	myMatrixSet.K1_Vnew_aux1 = (myMatrixSet.K1_F*myMatrixSet.K1_xprevV2)*myMatrixSet.K1_F.transpose();
	myMatrixSet.K1_yVpred2(0, 0) = myMatrixSet.K1_Vnew_aux1(0, 0) + myMatrixSet.K1_Q2(0, 0);
	myMatrixSet.K1_yVpred2(0, 1) = myMatrixSet.K1_Vnew_aux1(0, 1) + myMatrixSet.K1_Q2(0, 1);
	myMatrixSet.K1_yVpred2(1, 0) = myMatrixSet.K1_Vnew_aux1(1, 0) + myMatrixSet.K1_Q2(1, 0);
	myMatrixSet.K1_yVpred2(1, 1) = myMatrixSet.K1_Vnew_aux1(1, 1) + myMatrixSet.K1_Q2(1, 1);

	// Actualización
	myMatrixSet.K1_e = myMatrixSet.K1_C*myMatrixSet.K1_yxpred_kalman2;
	myMatrixSet.K1_e(0, 0) = (inputY - myMatrixSet.K1_e(0, 0));

	// S=H*Vpred*H'+R
	//aux0 = myMatrixSet.K1_C*myMatrixSet.K1_yVpred2;
	myMatrixSet.K1_S = (myMatrixSet.K1_C*myMatrixSet.K1_yVpred2)*myMatrixSet.K1_C.transpose();
	myMatrixSet.K1_S(0, 0) = myMatrixSet.K1_S(0, 0) + myMatrixSet.K1_R2(0, 0);

	//
	//aux0 = myMatrixSet.K1_yVpred2*myMatrixSet.K1_C.transpose();
	myMatrixSet.K1_KK = (myMatrixSet.K1_yVpred2*myMatrixSet.K1_C.transpose())*myMatrixSet.K1_S.inverse();
	myMatrixSet.K1_residuo = myMatrixSet.K1_KK*myMatrixSet.K1_e;


	//b = 100;
	if (abs(myMatrixSet.K1_e(0, 0)) <= K1_b *pow(myMatrixSet.K1_R2(0, 0), -0.5) * myMatrixSet.K1_S(0, 0))
	{
		/*aux0 = myMatrixSet.K1_yVpred2*myMatrixSet.K1_C.transpose();*/
		myMatrixSet.K1_KK = (myMatrixSet.K1_yVpred2*myMatrixSet.K1_C.transpose())*myMatrixSet.K1_S.inverse();
		myMatrixSet.K1_yxnew2(0, 0) = myMatrixSet.K1_residuo(0, 0) + myMatrixSet.K1_yxpred_kalman2(0, 0);
		myMatrixSet.K1_yxnew2(1, 0) = myMatrixSet.K1_residuo(1, 0) + myMatrixSet.K1_yxpred_kalman2(1, 0);
	}
	else
	{

		myMatrixSet.K1_Maux = myMatrixSet.K1_yVpred2*myMatrixSet.K1_C.transpose();
		bool sign = signbit(myMatrixSet.K1_e(0, 0));
		if (sign)
		{
			myMatrixSet.K1_yxnew2(0, 0) = myMatrixSet.K1_yxpred_kalman2(0, 0) + myMatrixSet.K1_Maux(0, 0) * pow(myMatrixSet.K1_R(0, 0), -0.5) * K1_b * -1;
			myMatrixSet.K1_yxnew2(1, 0) = myMatrixSet.K1_yxpred_kalman2(1, 0) + myMatrixSet.K1_Maux(1, 0) * pow(myMatrixSet.K1_R(0, 0), -0.5) * K1_b * -1;
		}
		else
		{
			myMatrixSet.K1_yxnew2(0, 0) = myMatrixSet.K1_yxpred_kalman2(0, 0) + myMatrixSet.K1_Maux(0, 0) * pow(myMatrixSet.K1_R(0, 0), -0.5) * K1_b * 1;
			myMatrixSet.K1_yxnew2(1, 0) = myMatrixSet.K1_yxpred_kalman2(1, 0) + myMatrixSet.K1_Maux(1, 0) * pow(myMatrixSet.K1_R(0, 0), -0.5) * K1_b * 1;
		}

	}

	myMatrixSet.K1_Vnew_aux1 = myMatrixSet.K1_KK*myMatrixSet.K1_C;

	myMatrixSet.K1_Vnew2(0, 0) = myMatrixSet.K1_ss(0, 0) - myMatrixSet.K1_Vnew_aux1(0, 0);
	myMatrixSet.K1_Vnew2(0, 1) = myMatrixSet.K1_ss(0, 1) - myMatrixSet.K1_Vnew_aux1(0, 1);
	myMatrixSet.K1_Vnew2(1, 0) = myMatrixSet.K1_ss(1, 0) - myMatrixSet.K1_Vnew_aux1(1, 0);
	myMatrixSet.K1_Vnew2(1, 1) = myMatrixSet.K1_ss(1, 1) - myMatrixSet.K1_Vnew_aux1(1, 1);

	myMatrixSet.K1_Vnew_aux1 = myMatrixSet.K1_Vnew2;

	myMatrixSet.K1_yVnew2 = myMatrixSet.K1_Vnew_aux1*myMatrixSet.K1_yVpred2;

	myMatrixSet.K1_yprevx2 = myMatrixSet.K1_yxnew2;
	myMatrixSet.K1_yprevV2 = myMatrixSet.K1_yVnew2;

	return myMatrixSet.K1_yxnew2(0, 0);
};

