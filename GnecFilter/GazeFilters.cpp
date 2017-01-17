#include "GazeFilters.h"
#include <iostream>
using namespace std;


GazeFilters::GazeFilters()
{
	filterTypeSelected = filtertype::wa;
	GazeBuffer.maxBufferSize = 10;
	WaBuffer.maxBufferSize = 50;
	gazeStateClassifier.setupGazeFix1(250);
	gazeStateClassifier.setupGazeFix2(100, 5);
}

GazeFilters::GazeFilters(int _filterTypeSelected, int _gazeBufferSize, int _waBufferSize, double _CursorJumpThresholdNormalized)
{
	filterTypeSelected = filtertype(_filterTypeSelected);
	GazeBuffer.maxBufferSize = _gazeBufferSize;
	WaBuffer.maxBufferSize = _waBufferSize;
	gazeStateClassifier.setupGazeFix1(_CursorJumpThresholdNormalized);
	gazeStateClassifier.setupGazeFix2(100, 5);
}

GazeFilters::~GazeFilters(){}

PointD GazeFilters::filterGazeData(PointD GazePoint)
{
	if (GazePoint.IsNaN())
		return GazePoint;
	else
	{
		//if (gazeStateClassifier.gazeFix1(GazePoint, lastFilterReturn))
		if (gazeStateClassifier.gazeFix2(GazePoint))
		{
			if (filterTypeSelected == filtertype::meanMedian)
				lastFilterReturn = GazeBuffer.getMeanMedianBuffer(GazePoint);//retorna el argumento
			if (filterTypeSelected == filtertype::average)
				lastFilterReturn = GazeBuffer.getAverageBuffer(GazePoint);
			if (filterTypeSelected == filtertype::wa) {
				lastFilterReturn = GazeBuffer.getAverageBuffer(GazePoint);
				lastFilterReturn = WaBuffer.getWABuffer(GazePoint);
			}
		}
		else
		{
			//lastFilterReturn = GazePoint;
			lastFilterReturn = GazeBuffer.getAverageBuffer(GazePoint);
			WaBuffer.clearBuffer();
			GazeBuffer.clearBuffer();
		}
		return lastFilterReturn;
	}
}
