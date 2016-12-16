#include <Eigen/core>
#include <Eigen/Dense>

#include "hf_02.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::Array;

HeadFilter2::HeadFilter2(float width, float height)
{
	initializeHeadFilter(width, height);
}

HeadFilter2::HeadFilter2(const HeadFilter2 &obj)
{
	ptr = new int;
	*ptr = *obj.ptr; // copy the value
}

HeadFilter2::~HeadFilter2(void)
{

}

MatrixXd HeadFilter2::initializeMatrixes(MatrixXd M, int m, int n)
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

void HeadFilter2::initializeHeadFilter(float screenWitdh, float screenHeight)
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

}

float HeadFilter2::headFilterX(double inputX)
{
	float posX = (float)inputX;

	if ((posX > 100) && (posX < enlazaSetup.screenWidth - 100))
	{
		K2_R2 = K2_R;
		K2_Q2 = K2_Q;
	}
	else
	{
		K2_R2 = K2_RB;
		K2_Q2 = K2_QB;
	}
	//b = 0.3;
	K2_b = enlazaSetup.kalman_b;

	K2_x_pred = K2_xprevx2;
	K2_x_Vpred = K2_xVpred2 + K2_Q2;

	K2_e = posX - K2_H * K2_x_pred;
	K2_S = K2_H * K2_x_Vpred * K2_H + K2_R2;
	K2_KK = K2_x_Vpred * K2_H / K2_S;

	K2_residuo = K2_KK * K2_e;
	K2_re = K2_S;
	K2_z = sqrt(K2_R2) * K2_e / K2_re;

	bool K2_z_sign = signbit(K2_z);

	if (abs(K2_z) <= K2_b)
	{
		K2_xxnew2 = K2_x_pred + (K2_re / sqrt(K2_R2)) * K2_z * K2_KK;
	}
	else
	{
		if (K2_z_sign)
		{
			K2_xxnew2 = K2_x_pred + (K2_re / sqrt(K2_R2)) * K2_b * -1 * K2_KK;
		}
		else
		{
			K2_xxnew2 = K2_x_pred + (K2_re / sqrt(K2_R2)) * K2_b * 1 * K2_KK;
		}
	}


	K2_xVnew2 = K2_x_Vpred + K2_Q2 - K2_re * K2_KK * K2_KK;


	K2_xprevx2 = K2_xxnew2;
	K2_xVpred2 = K2_xVnew2;

	return K2_xxnew2;
};

float HeadFilter2::headFilterY(double inputY)
{
	float posY = (float)inputY;

	if ((posY > 100) && (posY < enlazaSetup.screenheight - 100))
	{
		K2_R2 = K2_R;
		K2_Q2 = K2_Q;
	}
	else
	{
		K2_R2 = K2_RB;
		K2_Q2 = K2_QB;
	}

	//b = Convert.ToDouble(trackBar_kalman.Value);
	//b = b / 100;
	//b = 0.3;
	K2_b = enlazaSetup.kalman_b;

	K2_y_pred = K2_yprevx2;
	K2_y_Vpred = K2_yVpred2 + K2_Q2;

	K2_e = posY - K2_H * K2_y_pred;
	K2_S = K2_H * K2_y_Vpred * K2_H + K2_R2;
	K2_KK = K2_y_Vpred * K2_H / K2_S;

	K2_residuo = K2_KK * K2_e;
	K2_re = K2_S;
	K2_z = sqrt(K2_R) * K2_e / K2_re;

	bool K2_z_sign = signbit(K2_z);

	if (abs(K2_z) <= K2_b)
	{
		K2_yxnew2 = K2_y_pred + (K2_re / sqrt(K2_R2)) * K2_z * K2_KK;
	}

	else
	{
		if (K2_z_sign)
		{
			K2_yxnew2 = K2_y_pred + (K2_re / sqrt(K2_R2)) * K2_b * -1 * K2_KK;
		}
		else
		{
			K2_yxnew2 = K2_y_pred + (K2_re / sqrt(K2_R2)) * K2_b * 1 * K2_KK;
		}

	}


	K2_yVnew2 = K2_y_Vpred + K2_Q2 - K2_re * K2_KK * K2_KK;
	K2_yprevx2 = K2_yxnew2;
	K2_yVpred2 = K2_yVnew2;

	return K2_yxnew2;
};

double HeadFilter2::headFilterXconstrained(EnlazaControlInfo setup, double posX, double tabW)
{
	if ((posX > 100) && (posX < tabW - 100))
	{
		K2_R2 = K2_R;
		K2_Q2 = K2_Q;
	}
	else
	{
		K2_R2 = K2_RB;
		K2_Q2 = K2_QB;
	}
	K2_b = setup.kalman_b;
	//b = Convert.ToDouble(trackBar_kalman.Value);
	//b = b / 100;
	//b = b * cte2 / cte1;

	K2_x_pred = K2_xprevx2;
	K2_x_Vpred = K2_xVpred2 + K2_Q2;

	K2_e = posX - K2_H * K2_x_pred;
	K2_S = K2_H * K2_x_Vpred * K2_H + K2_R2;
	K2_KK = K2_x_Vpred * K2_H / K2_S;

	K2_residuo = K2_KK * K2_e;
	K2_re = K2_S;
	K2_z = (float)sqrt((float)K2_R2) * (float)K2_e / (float)K2_re;

	bool K2_z_sign = signbit(K2_z);

	if (abs(K2_z) <= K2_b)
	{
		K2_xxnew2 = K2_x_pred + (K2_re / (float)sqrt(K2_R2)) * K2_z * K2_KK;
	}

	else
	{
		if (K2_z_sign)
		{
			K2_yxnew2 = K2_x_pred + (K2_re / (float)sqrt(K2_R2)) * K2_b * -1 * K2_KK;
		}

		else
		{
			K2_yxnew2 = K2_x_pred + (K2_re / (float)sqrt(K2_R2)) * K2_b * 1 * K2_KK;
		}

	}


	K2_xVnew2 = K2_x_Vpred + K2_Q2 - K2_re * K2_KK * K2_KK;
	K2_xprevx2 = K2_xxnew2;
	K2_xVpred2 = K2_xVnew2;

	return K2_xxnew2;
};

double HeadFilter2::headFilterYconstrained(EnlazaControlInfo setup, double posY, double tabH)
{
	if ((posY > 100) && (posY < tabH - 100))
	{
		K2_R2 = K2_R;
		K2_Q2 = K2_Q;
	}
	else
	{
		K2_R2 = K2_RB;
		K2_Q2 = K2_QB;
	}

	//b = Convert.ToDouble(trackBar_kalman.Value);
	//b = b / 100;
	//b = 0.3;
	K2_b = setup.kalman_b;

	K2_y_pred = K2_yprevx2;
	K2_y_Vpred = K2_yVpred2 + K2_Q2;

	K2_e = posY - K2_H * K2_y_pred;
	K2_S = K2_H * K2_y_Vpred * K2_H + K2_R2;
	K2_KK = K2_y_Vpred * K2_H / K2_S;

	K2_residuo = K2_KK * K2_e;
	K2_re = K2_S;
	K2_z = (float)sqrt(K2_R) * K2_e / K2_re;

	bool K2_z_sign = signbit(K2_z);

	if (abs(K2_z) <= K2_b)
	{
		K2_yxnew2 = K2_y_pred + (K2_re / (float)sqrt(K2_R2)) * K2_z * K2_KK;
	}

	else
	{
		if (K2_z_sign)
		{
			K2_yxnew2 = K2_y_pred + (K2_re / (float)sqrt(K2_R2)) * K2_b * -1 * K2_KK;
		}

		else
		{
			K2_yxnew2 = K2_y_pred + (K2_re / (float)sqrt(K2_R2)) * K2_b * 1 * K2_KK;
		}

	}


	K2_yVnew2 = K2_y_Vpred + K2_Q2 - K2_re * K2_KK * K2_KK;
	K2_yprevx2 = K2_yxnew2;
	K2_yVpred2 = K2_yVnew2;

	return K2_yxnew2;
};