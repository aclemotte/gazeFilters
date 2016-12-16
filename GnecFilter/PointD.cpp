#include "PointD.h"
#include <math.h>


PointD::PointD()
{
	X = 0.0;
	Y = 0.0;
}

PointD::PointD(double x, double y)
{
	X = x;
	Y = y;
}

double PointD::distance(PointD a, PointD b)
{
	double xCuadrado = (a.X - b.X) * (a.X - b.X);
	double yCuadrado = (a.Y - b.Y) * (a.Y - b.Y);
	double distancia = sqrt(xCuadrado + yCuadrado);
	return distancia;
}

bool PointD::IsNaN()
{
	if (isnan(X) && isnan(Y))
		return true;
	else
		return false;
}

PointD::~PointD()
{

}