#include "Filtros.h"
#include <iostream>
#include "PointD.h"

using namespace std;


filter1::filter1()
{
	gazeFilter = new GazeFilters();
}

void filter1::filter(double &x, double &y)
{
	//x = 500;
	//y = 100;
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
}



filter2::filter2()
{}

void filter2::filter(double &x, double &y)
{
	x = 500;
	y = 200;
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
{}

void filter7::filter(double &x, double &y)
{
	x = 500;
	y = 700;
}




filter8::filter8()
{}

void filter8::filter(double &x, double &y)
{
	x = 500;
	y = 800;
}



