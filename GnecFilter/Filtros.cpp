#include <iostream>
#include "Filtros.h"
#include "PointD.h"

using namespace std;


filter1::filter1()
{
	gazeFilter = new GazeFilters(GazeFilters::filtertype::wa, 10, 50, 250);
}
	
void filter1::filter(double &x, double &y)
{
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
}



filter2::filter2()
{
	gazeFilter = new GazeFilters(GazeFilters::filtertype::average, 10, 50, 250);
}

void filter2::filter(double &x, double &y)
{
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
}




filter3::filter3()
{}

void filter3::filter(double &x, double &y)
{
	x = 500;
	y = 300;
}



filter4::filter4()
{}

void filter4::filter(double &x, double &y)
{
	x = 500;
	y = 400;
}




filter5::filter5()
{}

void filter5::filter(double &x, double &y)
{
	x = 500;
	y = 500;
}




filter6::filter6()
{}

void filter6::filter(double &x, double &y)
{
	x = 500;
	y = 600;
}





filter7::filter7()
{
	headFilter3 = new HeadFilter3();
}

void filter7::filter(double &x, double &y)
{
	// initializeHeadFilter(cov_omega, cov_nu);
	headFilter3->initializeHeadFilter(20,10);
	x = headFilter3->headFilterX(x);
	y = headFilter3->headFilterY(y);
}




filter8::filter8()
{}

void filter8::filter(double &x, double &y)
{
	x = 500;
	y = 800;
}



