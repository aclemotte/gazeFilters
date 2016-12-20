#include <Eigen/Core>
#include <Eigen/Dense>

#include "hf_03.h"

using namespace std;
using namespace Eigen;

HeadFilter3::HeadFilter3(float width, float height)
{

	initializeMatrixes();
	initializeHeadFilter(width, height);
}

HeadFilter3::HeadFilter3(const HeadFilter3 &obj)
{
	ptr = new int;
	*ptr = *obj.ptr; // copy the value
}

HeadFilter3::~HeadFilter3(void)
{

}

void HeadFilter3::initializeMatrixes(void)
{
	// initialize matrix (1/2)

	// 3x3 matrixes
	myMatrixSet.RG1.resize(3, 3);
	myMatrixSet.RG2.resize(3, 3);
	myMatrixSet.matriz_aux1.resize(3, 3);
	myMatrixSet.matriz_aux2.resize(3, 3);
	myMatrixSet.matriz_aux3.resize(3, 3);
	myMatrixSet.RGinv.resize(3, 3);
	myMatrixSet.RGS.resize(3, 3);
	myMatrixSet.RGS2.resize(3, 3);
	myMatrixSet.sensor1matriz.resize(3, 3);

	// 2x2 matrixes
	myMatrixSet.I2.resize(2, 2);
	myMatrixSet.IKHxAux.resize(2, 2);
	myMatrixSet.IKHyAux.resize(2, 2);
	myMatrixSet.PxppAux.resize(2, 2);
	myMatrixSet.PyppAux.resize(2, 2);
	myMatrixSet.FPmmFtx.resize(2, 2);
	myMatrixSet.FPmmFty.resize(2, 2);
	myMatrixSet.KHx.resize(2, 2);
	myMatrixSet.KHy.resize(2, 2);
	myMatrixSet.IKHx.resize(2, 2);
	myMatrixSet.IKHy.resize(2, 2);
	myMatrixSet.Fx.resize(2, 2);
	myMatrixSet.Fy.resize(2, 2);
	myMatrixSet.Q.resize(2, 2);
	myMatrixSet.Pxpp.resize(2, 2);
	myMatrixSet.Pypp.resize(2, 2);
	myMatrixSet.Pxmm.resize(2, 2);
	myMatrixSet.Pymm.resize(2, 2);

	//2x1 matrixes
	myMatrixSet.Kx.resize(1, 2);
	myMatrixSet.Ky.resize(1, 2);
	myMatrixSet.Hx.resize(1, 2);
	myMatrixSet.Hy.resize(1, 2);
	myMatrixSet.B.resize(2, 1);
	myMatrixSet.PmmHtx.resize(2, 1);
	myMatrixSet.PmmHty.resize(2, 1);
	myMatrixSet.xpp.resize(2, 1);
	myMatrixSet.ypp.resize(2, 1);
	myMatrixSet.xmm.resize(1, 2);
	myMatrixSet.ymm.resize(1, 2);
	myMatrixSet.xppAux.resize(2, 1); 
	myMatrixSet.yppAux.resize(2, 1); 

	//1x1 matrixes
	myMatrixSet.Sx.resize(1, 1);
	myMatrixSet.Sy.resize(1, 1);
	myMatrixSet.Kzx.resize(1, 1);
	myMatrixSet.Kzy.resize(1, 1);
	myMatrixSet.Rx.resize(1, 1);
	myMatrixSet.Ry.resize(1, 1);
}

MatrixXd HeadFilter3::initializeMatrixes2Zero(MatrixXd M, int m, int n)
{
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			M(i, j) = 0;
		}
	}
	return M;
}

void HeadFilter3::initializeHeadFilter(float screenWitdh, float screenHeight)
{
	enlazaSetup.sampFreq = 50;
	enlazaSetup.kalman_b = 0.3f;
	enlazaSetup.kalmanOn = 1;
	enlazaSetup.lateralControl = false;
	enlazaSetup.verticalRange = 20;
	enlazaSetup.horizontalRange = 30;
	enlazaSetup.screenheight = screenHeight;
	enlazaSetup.screenWidth = screenWitdh;

	// initialize matrix (2/2)
	
	// 3x3
	myMatrixSet.RG1 = initializeMatrixes2Zero(myMatrixSet.RG1, 3, 3);
	myMatrixSet.RGinv = initializeMatrixes2Zero(myMatrixSet.RGinv, 3, 3);
	myMatrixSet.sensor1matriz = initializeMatrixes2Zero(myMatrixSet.sensor1matriz, 3, 3);
	myMatrixSet.RGS = initializeMatrixes2Zero(myMatrixSet.RGS, 3, 3);
	myMatrixSet.RG2 = initializeMatrixes2Zero(myMatrixSet.RG2, 3, 3);
	myMatrixSet.RGS2 = initializeMatrixes2Zero(myMatrixSet.RGS2, 3, 3);
	myMatrixSet.matriz_aux1 = initializeMatrixes2Zero(myMatrixSet.matriz_aux1, 3, 3);
	myMatrixSet.matriz_aux2 = initializeMatrixes2Zero(myMatrixSet.matriz_aux2, 3, 3);
	myMatrixSet.matriz_aux2 = initializeMatrixes2Zero(myMatrixSet.matriz_aux3, 3, 3);

	//2x2
	myMatrixSet.I2 = initializeMatrixes2Zero(myMatrixSet.I2, 2, 2);
	myMatrixSet.IKHxAux = initializeMatrixes2Zero(myMatrixSet.IKHxAux, 2, 2);
	myMatrixSet.IKHyAux = initializeMatrixes2Zero(myMatrixSet.IKHyAux, 2, 2); 
	myMatrixSet.PxppAux = initializeMatrixes2Zero(myMatrixSet.PxppAux, 2, 2);
	myMatrixSet.PyppAux = initializeMatrixes2Zero(myMatrixSet.PyppAux, 2, 2);
	myMatrixSet.FPmmFtx = initializeMatrixes2Zero(myMatrixSet.FPmmFtx, 2, 2);
	myMatrixSet.FPmmFty = initializeMatrixes2Zero(myMatrixSet.FPmmFty, 2, 2);
	myMatrixSet.KHx = initializeMatrixes2Zero(myMatrixSet.KHx, 2, 2);
	myMatrixSet.KHy = initializeMatrixes2Zero(myMatrixSet.KHy, 2, 2);
	myMatrixSet.IKHx = initializeMatrixes2Zero(myMatrixSet.IKHx, 2, 2);
	myMatrixSet.IKHy = initializeMatrixes2Zero(myMatrixSet.IKHy, 2, 2);
	myMatrixSet.Fx = initializeMatrixes2Zero(myMatrixSet.Fx, 2, 2);
	myMatrixSet.Fy = initializeMatrixes2Zero(myMatrixSet.Fy, 2, 2);
	myMatrixSet.Q = initializeMatrixes2Zero(myMatrixSet.Q, 2, 2);
	myMatrixSet.Pxpp = initializeMatrixes2Zero(myMatrixSet.Pxpp, 2, 2);
	myMatrixSet.Pypp = initializeMatrixes2Zero(myMatrixSet.Pypp, 2, 2);
	myMatrixSet.Pxmm = initializeMatrixes2Zero(myMatrixSet.Pxmm, 2, 2);
	myMatrixSet.Pymm = initializeMatrixes2Zero(myMatrixSet.Pymm, 2, 2);

	//2x1
	myMatrixSet.Kx = initializeMatrixes2Zero(myMatrixSet.Kx, 1, 2);
	myMatrixSet.Ky = initializeMatrixes2Zero(myMatrixSet.Ky, 1, 2);
	myMatrixSet.Hx = initializeMatrixes2Zero(myMatrixSet.Hx, 1, 2);
	myMatrixSet.Hy = initializeMatrixes2Zero(myMatrixSet.Hy, 1, 2);
	myMatrixSet.PmmHtx = initializeMatrixes2Zero(myMatrixSet.PmmHtx, 2, 1); 
	myMatrixSet.PmmHty = initializeMatrixes2Zero(myMatrixSet.PmmHty, 2, 1); 
	myMatrixSet.B = initializeMatrixes2Zero(myMatrixSet.B, 1, 2);
	myMatrixSet.xpp = initializeMatrixes2Zero(myMatrixSet.xpp, 2, 1);
	myMatrixSet.ypp = initializeMatrixes2Zero(myMatrixSet.ypp, 2, 1);
	myMatrixSet.xmm = initializeMatrixes2Zero(myMatrixSet.xmm, 1, 2);
	myMatrixSet.ymm = initializeMatrixes2Zero(myMatrixSet.ymm, 1, 2);
	myMatrixSet.xppAux = initializeMatrixes2Zero(myMatrixSet.xppAux, 2, 1);
	myMatrixSet.yppAux = initializeMatrixes2Zero(myMatrixSet.yppAux, 2, 1);

	//1x1
	 myMatrixSet.Sx = initializeMatrixes2Zero(myMatrixSet.Sx, 1, 1);
	 myMatrixSet.Sy = initializeMatrixes2Zero(myMatrixSet.Sy, 1, 1);
	 myMatrixSet.Kzx = initializeMatrixes2Zero(myMatrixSet.Kzx, 2, 1);
	 myMatrixSet.Kzy = initializeMatrixes2Zero(myMatrixSet.Kzy, 2, 1);
	 myMatrixSet.Rx = initializeMatrixes2Zero(myMatrixSet.Rx, 1, 1);
	 myMatrixSet.Ry = initializeMatrixes2Zero(myMatrixSet.Ry, 1, 1);

	// valores para los filtros
	Ts = 1 / (enlazaSetup.sampFreq);
	myMatrixSet.Fx(0, 0) = 1;
	myMatrixSet.Fx(0, 1) = Ts;
	myMatrixSet.Fx(1, 0) = 0;
	myMatrixSet.Fx(1, 1) = 1;	
	myMatrixSet.Fy(0, 0) = 1;
	myMatrixSet.Fy(0, 1) = Ts;
	myMatrixSet.Fy(1, 0) = 0;
	myMatrixSet.Fy(1, 1) = 1;	

	myMatrixSet.B(0, 0) = pow(Ts, 2);
	myMatrixSet.B(1, 0) = Ts;

	myMatrixSet.Hx(0, 0) = 1;
	myMatrixSet.Hx(0, 1) = 0;
	myMatrixSet.Hy(0, 0) = 1;
	myMatrixSet.Hy(0, 1) = 0;

	// pasar a argumentos de entrada!!!
	cov_omega = 50;		// Predicción Cuanto más alto menos filtra
	cov_nu = 10;		// Medida

	myMatrixSet.I2(0, 0) = 1;
	myMatrixSet.I2(0, 1) = 0;
	myMatrixSet.I2(1, 0) = 0;
	myMatrixSet.I2(1, 1) = 1;

	// Q ????????????
	myMatrixSet.Q(0, 0) = pow(cov_omega, 2) * pow(Ts, 4) / 4;
	myMatrixSet.Q(0, 1) = pow(cov_omega, 2) * pow(Ts, 3) / 2;
	myMatrixSet.Q(1, 0) = pow(cov_omega, 2) * pow(Ts, 3) / 2;
	myMatrixSet.Q(1, 1) = pow(cov_omega, 2) * pow(Ts, 2);
	// R
	myMatrixSet.Rx(0, 0) = pow(cov_nu, 2);
	myMatrixSet.Ry(0, 0) = pow(cov_nu, 2);

	// Posición inicial en el centro de la pantalla
	myMatrixSet.xpp(0, 0) = enlazaSetup.horizontalRange / 2;
	myMatrixSet.xpp(1, 0) = 0;
	myMatrixSet.ypp(0, 0) = enlazaSetup.verticalRange / 2;
	myMatrixSet.ypp(1, 0) = 0;

	// pasar a argumentos de entrada?
	bx = 100;
	by = 100;

	// P
	myMatrixSet.Pxpp(0, 0) = myMatrixSet.Q(0, 0);
	myMatrixSet.Pxpp(0, 1) = myMatrixSet.Q(0, 1);
	myMatrixSet.Pxpp(1, 0) = myMatrixSet.Q(1, 0);
	myMatrixSet.Pxpp(1, 1) = myMatrixSet.Q(1, 1);
	myMatrixSet.Pypp(0, 0) = myMatrixSet.Q(0, 0);
	myMatrixSet.Pypp(0, 1) = myMatrixSet.Q(0, 1);
	myMatrixSet.Pypp(1, 0) = myMatrixSet.Q(1, 0);
	myMatrixSet.Pypp(1, 1) = myMatrixSet.Q(1, 1);

}

float HeadFilter3::headFilterX(double inputX)
{

	// Predicción ////////////////////////////////////////////////////////////////////////////////////////////
	// x-- = F * x++
	myMatrixSet.xmm = myMatrixSet.Fx*myMatrixSet.xpp;
	double u = 1.5;
	// x-- = F * x++ + B * u
	myMatrixSet.xmm(0, 0) = myMatrixSet.xmm(0, 0) + myMatrixSet.B(0, 0) * u;
	myMatrixSet.xmm(1, 0) = myMatrixSet.xmm(1, 0) + myMatrixSet.B(1, 0) * u;

	// P-- = F * P++ *F' + Q
	// FPmmFtx = Fy * Px++ *Ftx
	myMatrixSet.FPmmFtx = myMatrixSet.Fx*myMatrixSet.Pxpp*myMatrixSet.Fx.transpose();
	myMatrixSet.Pxmm(0, 0) = myMatrixSet.FPmmFtx(0, 0) + myMatrixSet.Q(0, 0);
	myMatrixSet.Pxmm(0, 1) = myMatrixSet.FPmmFtx(0, 1) + myMatrixSet.Q(0, 1);
	myMatrixSet.Pxmm(1, 0) = myMatrixSet.FPmmFtx(1, 0) + myMatrixSet.Q(1, 0);
	myMatrixSet.Pxmm(1, 1) = myMatrixSet.FPmmFtx(1, 1) + myMatrixSet.Q(1, 1);

	// Actualización ////////////////////////////////////////////////////////////////////////////////////////////
	// z = x - H * x--
	myMatrixSet.zx = myMatrixSet.Hx*myMatrixSet.xmm;
	myMatrixSet.zx(0, 0) = (inputX - myMatrixSet.zx(0, 0));

	// S=H*Vpred*H'+R ------------------------------------------------------------------------------------------
	myMatrixSet.Sx = myMatrixSet.Hx*myMatrixSet.Pxmm*myMatrixSet.Hx.transpose();
	myMatrixSet.Sx(0, 0) = myMatrixSet.Sx(0, 0) + myMatrixSet.Rx(0, 0);

	// K = P-- * H' * S(-1) --------------------------------------------------------------------------------------
	myMatrixSet.Kx = myMatrixSet.PmmHtx*myMatrixSet.Sx.inverse();
	// K * z
	myMatrixSet.Kzx = myMatrixSet.Kx*myMatrixSet.zx;


	//b = 100;
	// if (|z| < S * b / sqrt(R)) --------------------------------------------------------------------------------
	if (abs(myMatrixSet.zx(0, 0)) <= bx *pow(myMatrixSet.Rx(0, 0), -0.5) * myMatrixSet.Sx(0, 0))
	{
		// PmmHtSy = P-- * H' * S(-1)
		// (Px-- * Hx')
		// K = (Py-- * H') * S(-1)
		myMatrixSet.PmmHtx = myMatrixSet.Pxmm*myMatrixSet.Hx.transpose();
		myMatrixSet.Kx = myMatrixSet.PmmHtx*myMatrixSet.Sx.inverse();
		// x++ = x-- + k * z ---------------------------------------------------------------------------------
		myMatrixSet.xppAux(0, 0) = myMatrixSet.xmm(0, 0) + myMatrixSet.Kzx(0, 0);
		myMatrixSet.xppAux(1, 0) = myMatrixSet.xmm(1, 0) + myMatrixSet.Kzx(1, 0); 
	}
	else
	{
		// x++ = x-- + (Px-- * Hx') * bx / sqrt(Rx) * sign(zx) -------------------------------------------
		// Px-- * Hx'
		myMatrixSet.PmmHtx = myMatrixSet.Pxmm*myMatrixSet.Hx.transpose();
		bool sign = signbit(myMatrixSet.zx(0, 0));
		if (sign)
		{
			// x++ = x-- + (P-- * H') * b / sqrt(R) * sign(z) ----------------------------------------
			myMatrixSet.xppAux(0, 0) = myMatrixSet.xmm(0, 0) + myMatrixSet.PmmHtx(0, 0) * pow(myMatrixSet.Rx(0, 0), -0.5) * bx * -1;
			myMatrixSet.xppAux(1, 0) = myMatrixSet.xmm(1, 0) + myMatrixSet.PmmHtx(1, 0) * pow(myMatrixSet.Rx(0, 0), -0.5) * bx * -1;
		}
		else
		{
			// x++ = x-- + (P-- * H') * b / sqrt(R) * sign(z) ----------------------------------------
			myMatrixSet.xppAux(0, 0) = myMatrixSet.xmm(0, 0) + myMatrixSet.PmmHtx(0, 0) * pow(myMatrixSet.Rx(0, 0), -0.5) * bx * 1;
			myMatrixSet.xppAux(1, 0) = myMatrixSet.xmm(1, 0) + myMatrixSet.PmmHtx(1, 0) * pow(myMatrixSet.Rx(0, 0), -0.5) * bx * 1;
		}

	}

	myMatrixSet.KHx = myMatrixSet.Kx*myMatrixSet.Hx;

	myMatrixSet.IKHxAux(0, 0) = myMatrixSet.I2(0, 0) - myMatrixSet.KHx(0, 0);
	myMatrixSet.IKHxAux(0, 1) = myMatrixSet.I2(0, 1) - myMatrixSet.KHx(0, 1);
	myMatrixSet.IKHxAux(1, 0) = myMatrixSet.I2(1, 0) - myMatrixSet.KHx(1, 0);
	myMatrixSet.IKHxAux(1, 1) = myMatrixSet.I2(1, 1) - myMatrixSet.KHx(1, 1);
	
	myMatrixSet.IKHx = myMatrixSet.IKHxAux;

	// Px++ = (I - K * H)x * Px-- --------------------------------------------------------------
	myMatrixSet.Pxpp = myMatrixSet.IKHx*myMatrixSet.Pxmm;

	// x++ = x++ ------------------------------------------------------------------------------
	myMatrixSet.xpp = myMatrixSet.xppAux; 
	// Px++ = Px++ ---------------------------------------------------------------------------
	myMatrixSet.Pxpp = myMatrixSet.PxppAux; 

	// return x++ -----------------------------------------------------------------------------
	return myMatrixSet.xpp(0,0);
};

float HeadFilter3::headFilterY(double inputY)
{
	// Kalman filter equations

	// Predicción ////////////////////////////////////////////////////////////////////////////////////////////
	// y-- = F * y++
	myMatrixSet.ymm = myMatrixSet.Fy*myMatrixSet.ypp;
	double u = 1.5;
	// y-- = F * y++ + B * u
	myMatrixSet.ymm(0, 0) = myMatrixSet.ymm(0, 0) + myMatrixSet.B(0, 0) * u;
	myMatrixSet.ymm(1, 0) = myMatrixSet.ymm(1, 0) + myMatrixSet.B(1, 0) * u;

	// P-- = F * P++ *F' + Q
	// FPmmFty = Fy * Py++ *Fty
	myMatrixSet.FPmmFty = myMatrixSet.Fy*myMatrixSet.Pypp*myMatrixSet.Fy.transpose();
	myMatrixSet.Pymm(0, 0) = myMatrixSet.FPmmFty(0, 0) + myMatrixSet.Q(0, 0);
	myMatrixSet.Pymm(0, 1) = myMatrixSet.FPmmFty(0, 1) + myMatrixSet.Q(0, 1);
	myMatrixSet.Pymm(1, 0) = myMatrixSet.FPmmFty(1, 0) + myMatrixSet.Q(1, 0);
	myMatrixSet.Pymm(1, 1) = myMatrixSet.FPmmFty(1, 1) + myMatrixSet.Q(1, 1);

	// Actualización ////////////////////////////////////////////////////////////////////////////////////////////
	// z = y - H * y--
	myMatrixSet.zy = myMatrixSet.Hy*myMatrixSet.ymm;
	myMatrixSet.zy(0, 0) = (inputY - myMatrixSet.zy(0, 0));

	// S=H*Vpred*H'+R ------------------------------------------------------------------------------------------
	myMatrixSet.Sy = myMatrixSet.Hy*myMatrixSet.Pymm*myMatrixSet.Hy.transpose();
	myMatrixSet.Sy(0, 0) = myMatrixSet.Sy(0, 0) + myMatrixSet.Ry(0, 0);

	// K = P-- * H' * S(-1) --------------------------------------------------------------------------------------
	myMatrixSet.Ky = myMatrixSet.PmmHty*myMatrixSet.Sy.inverse();
	// K * z
	myMatrixSet.Kzy = myMatrixSet.Ky*myMatrixSet.zy;


	//b = 100;
	// if (|z| < S * b / sqrt(R)) --------------------------------------------------------------------------------
	if (abs(myMatrixSet.zy(0, 0)) <= by *pow(myMatrixSet.Ry(0, 0), -0.5) * myMatrixSet.Sy(0, 0))
	{
		// PmmHtSy = P-- * H' * S(-1)
		// (Py-- * Hy')
		// K = (Py-- * H') * S(-1)
		myMatrixSet.PmmHty = myMatrixSet.Pymm*myMatrixSet.Hy.transpose();
		myMatrixSet.Ky = myMatrixSet.PmmHty*myMatrixSet.Sy.inverse();
		// y++ = y-- + k * z ---------------------------------------------------------------------------------
		myMatrixSet.yppAux(0, 0) = myMatrixSet.ymm(0, 0) + myMatrixSet.Kzy(0, 0);
		myMatrixSet.yppAux(1, 0) = myMatrixSet.ymm(1, 0) + myMatrixSet.Kzy(1, 0); 
	}
	else
	{
		// y++ = y-- + (Py-- * Hy') * by / sqrt(Ry) * sign(zy) -------------------------------------------
		// Py-- * Hy'
		myMatrixSet.PmmHty = myMatrixSet.Pymm*myMatrixSet.Hy.transpose();
		bool sign = signbit(myMatrixSet.zy(0, 0));
		if (sign)
		{
			// y++ = y-- + (P-- * H') * b / sqrt(R) * sign(z) ----------------------------------------
			myMatrixSet.yppAux(0, 0) = myMatrixSet.ymm(0, 0) + myMatrixSet.PmmHty(0, 0) * pow(myMatrixSet.Ry(0, 0), -0.5) * by * -1;
			myMatrixSet.yppAux(1, 0) = myMatrixSet.ymm(1, 0) + myMatrixSet.PmmHty(1, 0) * pow(myMatrixSet.Ry(0, 0), -0.5) * by * -1;
		}
		else
		{
			// y++ = y-- + (P-- * H') * b / sqrt(R) * sign(z) ----------------------------------------
			myMatrixSet.yppAux(0, 0) = myMatrixSet.ymm(0, 0) + myMatrixSet.PmmHty(0, 0) * pow(myMatrixSet.Ry(0, 0), -0.5) * by * 1;
			myMatrixSet.yppAux(1, 0) = myMatrixSet.ymm(1, 0) + myMatrixSet.PmmHty(1, 0) * pow(myMatrixSet.Ry(0, 0), -0.5) * by * 1;
		}

	}

	myMatrixSet.KHy = myMatrixSet.Ky*myMatrixSet.Hy;
	
	myMatrixSet.IKHyAux(0, 0) = myMatrixSet.I2(0, 0) - myMatrixSet.KHy(0, 0);
	myMatrixSet.IKHyAux(0, 1) = myMatrixSet.I2(0, 1) - myMatrixSet.KHy(0, 1);
	myMatrixSet.IKHyAux(1, 0) = myMatrixSet.I2(1, 0) - myMatrixSet.KHy(1, 0);
	myMatrixSet.IKHyAux(1, 1) = myMatrixSet.I2(1, 1) - myMatrixSet.KHy(1, 1);

	myMatrixSet.IKHy = myMatrixSet.IKHyAux;

	// Py++ = (I - K * H)y * Py-- --------------------------------------------------------------
	myMatrixSet.Pypp = myMatrixSet.IKHy*myMatrixSet.Pymm;
	
	// y++ = y++ ------------------------------------------------------------------------------
	myMatrixSet.ypp = myMatrixSet.yppAux; 
	// Py++ = Py++ ---------------------------------------------------------------------------
	myMatrixSet.Pypp = myMatrixSet.PyppAux; 

	// return y++ -----------------------------------------------------------------------------
	return myMatrixSet.ypp(0,0);

};