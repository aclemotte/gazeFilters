#include "GnecFilters.h"

#include "GazeFilters.h"
#include "hf_01.h"
#include "hf_02.h"
#include "PointD.h"


GnecGazeFilters::GnecGazeFilters()
{
	_gazeFilter = new GazeFilters();
}

GnecGazeFilters::~GnecGazeFilters()
{
	delete _gazeFilter;
}

void GnecGazeFilters::filterGazeData(float &x, float &y)
{
	PointD p(x, y);
	_gazeFilter->filterGazeData(PointD(x, y));
	x = p.X;
	y = p.Y;
}



GnecHeadFilter1::GnecHeadFilter1(float width, float height)
{
	_headFilter1 = new HeadFilter1(width, height);
}

GnecHeadFilter1::~GnecHeadFilter1()
{
	delete _headFilter1;
}

void GnecHeadFilter1::initializeHeadFilter(float screenWitdh, float screenHeight)
{
	_headFilter1->initializeHeadFilter(screenWitdh, screenHeight);
}

float GnecHeadFilter1::headFilterX(double inputX)
{
	return _headFilter1->headFilterX(inputX);
}

float GnecHeadFilter1::headFilterY(double inputY)
{
	return _headFilter1->headFilterY(inputY);
}



GnecHeadFilter2::GnecHeadFilter2(float width, float height)
{
	_headFilter2 = new HeadFilter2(width, height);
}

GnecHeadFilter2::~GnecHeadFilter2()
{
	delete _headFilter2;
}

void GnecHeadFilter2::initializeHeadFilter(float screenWitdh, float screenHeight)
{
	_headFilter2->initializeHeadFilter(screenWitdh, screenHeight);
}

float GnecHeadFilter2::headFilterX(double inputX)
{
	return _headFilter2->headFilterX(inputX);
}

float GnecHeadFilter2::headFilterY(double inputY)
{
	return _headFilter2->headFilterY(inputY);
}

