#pragma once

#include <Eigen\core>
#include <Eigen\Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::Array;

class HeadFilter3
{
public:
	//double cov_omega;
	//double cov_nu;
	double Ts, bx, by;
	
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
		MatrixXd zx;
		MatrixXd zy;

		// 3x3 matrixes
		MatrixXd RG1;
		MatrixXd RG2;
		MatrixXd matriz_aux1;
		MatrixXd matriz_aux2;
		MatrixXd matriz_aux3;
		MatrixXd RGinv;
		MatrixXd RGS;
		MatrixXd RGS2;
		MatrixXd sensor1matriz;

		// 2x2 matrixes
		MatrixXd I2;
		MatrixXd IKHxAux;
		MatrixXd IKHyAux;
		MatrixXd PxppAux;
		MatrixXd PyppAux;
		MatrixXd FPmmFtx;
		MatrixXd FPmmFty;
		MatrixXd KHx;
		MatrixXd KHy;
		MatrixXd IKHx;
		MatrixXd IKHy;
		MatrixXd Fx;
		MatrixXd Fy;
		MatrixXd Q;
		MatrixXd Pxpp;
		MatrixXd Pypp;
		MatrixXd Pxmm;
		MatrixXd Pymm;

		//2x1 matrixes
		MatrixXd Kx;
		MatrixXd Ky;
		MatrixXd Hx;
		MatrixXd Hy;
		MatrixXd PmmHtx;
		MatrixXd PmmHty;
		MatrixXd B;
		MatrixXd xpp;
		MatrixXd ypp;
		MatrixXd xmm;
		MatrixXd ymm;
		MatrixXd xppAux;
		MatrixXd yppAux;

		//1x1 
		MatrixXd Sx;
		MatrixXd Sy;
		MatrixXd Kzx;
		MatrixXd Kzy;
		MatrixXd Rx;
		MatrixXd Ry;
	} myMatrixSet;

	// methods for headfilter1
	void initializeMatrixes(void);
	MatrixXd  initializeMatrixes2Zero(MatrixXd M, int m, int n);
	void  initializeHeadFilter(double cov_omega, double cov_nu);
	float  headFilterX(double inputX);
	float  headFilterY(double inputY);

	HeadFilter3();
	HeadFilter3(double cov_omega, double cov_nu);
	HeadFilter3(const HeadFilter3 &obj);
	~HeadFilter3(void);



};