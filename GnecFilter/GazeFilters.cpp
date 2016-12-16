#include "GazeFilters.h"
#include <queue>
#include <iostream>
using namespace std;


GazeFilters::GazeFilters()
{
	filterTypeSelected = filtertype::wa;
	gazeBufferSize = 10;
	waBufferSize = 50;	
	CursorJumpThresholdNormalized = 250;
}

GazeFilters::GazeFilters(int _filterTypeSelected, int _gazeBufferSize, int _waBufferSize, double _CursorJumpThresholdNormalized)
{
	filterTypeSelected = filtertype(_filterTypeSelected);
	gazeBufferSize = _gazeBufferSize;
	waBufferSize = _waBufferSize;	
	CursorJumpThresholdNormalized = _CursorJumpThresholdNormalized;
}

GazeFilters::~GazeFilters()
{}


PointD GazeFilters::filterGazeData(PointD GazePoint)
{
	if (GazePoint.IsNaN())
		return GazePoint;
	else
	{
		if (gazeFix1(GazePoint))
		{
			if (filterTypeSelected == filtertype::meanMedian)
				lastFilterReturn = getMeanMedianGazeFiltered(GazePoint);
			if (filterTypeSelected == filtertype::average)
				lastFilterReturn = getMovingAverageGaze(GazePoint);			
			if (filterTypeSelected == filtertype::wa) {
				lastFilterReturn = getMovingAverageGaze(GazePoint);
				lastFilterReturn = getWA(lastFilterReturn);
			}
		}
		else
		{
			//lastFilterReturn = GazePoint;
			//clearBuffers();
			lastFilterReturn = getMovingAverageGaze(GazePoint);
			clearBuffersWA();
		}
		return lastFilterReturn;
	}
}

PointD GazeFilters::getMeanMedianGazeFiltered(PointD GazePoint)
{
	addPointD2Buffer(GazePoint);		

	//deben existir al menos tres puntos en el buffer para hacer lo siguiente
	if (GazeBufferX.size() > 2)
	{
		PointD gazeFiltered;
		//si es par el promedio de los 4 centrales
		//si es impar se promedia el del medio con sus vecinos

		if (GazeBufferX.size() % 2 == 0)
		{
			gazeFiltered.X = (
				GazeBufferX[(GazeBufferX.size() / 2) - 2] + 
				GazeBufferX[(GazeBufferX.size() / 2) - 1] +
				GazeBufferX[(GazeBufferX.size() / 2) + 0] + 
				GazeBufferX[(GazeBufferX.size() / 2) + 1] ) / 4;
			gazeFiltered.Y = (
				GazeBufferY[(GazeBufferX.size() / 2) - 2] + 
				GazeBufferY[(GazeBufferX.size() / 2) - 1] +
				GazeBufferY[(GazeBufferX.size() / 2) + 0] + 
				GazeBufferY[(GazeBufferX.size() / 2) + 1] ) / 4;
		}
		else
		{
			gazeFiltered.X = (
				GazeBufferX[(GazeBufferX.size() / 2) - 1] + 
				GazeBufferX[(GazeBufferX.size() / 2) + 0] + 
				GazeBufferX[(GazeBufferX.size() / 2) + 1] ) / 3;
			gazeFiltered.Y = (
				GazeBufferY[(GazeBufferX.size() / 2) - 1] + 
				GazeBufferY[(GazeBufferX.size() / 2) + 0] + 
				GazeBufferY[(GazeBufferX.size() / 2) + 1] ) / 3;
		}
		return gazeFiltered;
	}
	else //si el buffer tiene menos de 3 elementos, se devuelve el argumento
	{
		return GazePoint;
	}		
}

PointD GazeFilters::getMovingAverageGaze(PointD GazePoint)
{
	addPointD2Buffer(GazePoint);

	//Todo calcular el average
	PointD gazeFiltered;

	for (unsigned int indiceBuffer = 0; indiceBuffer < GazeBufferX.size(); indiceBuffer++)
	{
		gazeFiltered.X += GazeBufferX[indiceBuffer];
		gazeFiltered.Y += GazeBufferY[indiceBuffer];
	}
	gazeFiltered.X = gazeFiltered.X / GazeBufferX.size();
	gazeFiltered.Y = gazeFiltered.Y / GazeBufferY.size();

	return gazeFiltered;
}

PointD GazeFilters::getWA(PointD GazePoint)
{
	addPointD2BufferWA(GazePoint);
	
	double pxf = 0;
	double pyf = 0;
	double qlen = WaBufferX.size();
	for (int i = 0; i<qlen; i++) {
		pxf += WaBufferX.at(i) * (qlen - i) / (qlen*(qlen + 1) / 2);
		pyf += WaBufferY.at(i) * (qlen - i) / (qlen*(qlen + 1) / 2);
	}
	
	PointD gazeFiltered(pxf,pyf);
	return gazeFiltered;
}


bool GazeFilters::gazeFix1(PointD GazePoint)
{
	if (PointD::distance(GazePoint, lastFilterReturn) < CursorJumpThresholdNormalized)
		return true;
	else
		return false;
}

void GazeFilters::addPointD2Buffer(PointD GazePoint)
{
	if (GazeBufferX.size() >= gazeBufferSize)
	{
		GazeBufferX.pop_back();
		GazeBufferY.pop_back();
	}

	GazeBufferX.push_front(GazePoint.X);
	GazeBufferY.push_front(GazePoint.Y);
}

void GazeFilters::addPointD2BufferWA(PointD GazePoint)
{
	if (WaBufferX.size() >= waBufferSize)
	{
		WaBufferX.pop_back();
		WaBufferY.pop_back();
	}

	WaBufferX.push_front(GazePoint.X);
	WaBufferY.push_front(GazePoint.Y);
}

void GazeFilters::clearBuffers()
{
	GazeBufferX.clear(); 
	GazeBufferY.clear();
}

void GazeFilters::clearBuffersWA()
{
	WaBufferX.clear();
	WaBufferY.clear();
}