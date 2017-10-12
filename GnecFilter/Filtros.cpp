#include <iostream>
#include "Filtros.h"

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
	//x = 600;
	//y = 300;
}



filter2::filter2()
{
	gazeFilter = new GazeFilters(GazeFilters::filtertype::wa, 10, 50, 250);
}

void filter2::filter(double &x, double &y)
{
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
	//x = 650;
	//y = 300;
}




filter3::filter3()
{
	gazeFilter = new GazeFilters(GazeFilters::filtertype::average, 20, 50, 250);
}

void filter3::filter(double &x, double &y)
{
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
	//x = 500;
	//y = 300;
}



filter4::filter4()
{
	gazeFilter = new GazeFilters(GazeFilters::filtertype::average, 50, 50, 250);
}

void filter4::filter(double &x, double &y)
{
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
	//x = 500;
	//y = 400;
}




filter5::filter5()
{
	gazeFilter = new GazeFilters(GazeFilters::filtertype::average, 100, 50, 1000);
}

void filter5::filter(double &x, double &y)
{
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
	//x = 500;
	//y = 500;
}




filter6::filter6()
{
	gazeFilter = new GazeFilters(GazeFilters::filtertype::meanMedian, 10, 50, 10);
}

void filter6::filter(double &x, double &y)
{
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
	//x = 500;
	//y = 600;
}





filter7::filter7()
{
	headFilter3 = new HeadFilter3();
	//gazeFilter = new GazeFilters(GazeFilters::filtertype::meanMedian, 10, 50, 250);
}

void filter7::filter(double &x, double &y)
{	
	headFilter3->initializeHeadFilter(20,10);//initializeHeadFilter(cov_omega, cov_nu);
	x = headFilter3->headFilterX(x);
	y = headFilter3->headFilterY(y);
	//PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	//x = dataFiltered.X;
	//y = dataFiltered.Y;
	//x = 500;
	//y = 700;
}




filter8::filter8()
{
	gazeFilter = new GazeFilters(GazeFilters::filtertype::meanMedian, 10, 50, 20);
}

void filter8::filter(double &x, double &y)
{
	PointD dataFiltered = gazeFilter->filterGazeData(PointD(x, y));
	x = dataFiltered.X;
	y = dataFiltered.Y;
	//x = 500;
	//y = 800;
}



