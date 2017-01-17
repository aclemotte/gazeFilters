#include "GazeFilters.h"
#include <iostream>
using namespace std;


GazeFilters::GazeFilters()
{
	filterTypeSelected = filtertype::wa;
	GazeBuffer.bufferSize = 10;
	WaBuffer.bufferSize = 50;
	gazeStateClassifier.CursorJumpThresholdNormalized = 250;
}

GazeFilters::GazeFilters(int _filterTypeSelected, int _gazeBufferSize, int _waBufferSize, double _CursorJumpThresholdNormalized)
{
	filterTypeSelected = filtertype(_filterTypeSelected);
	GazeBuffer.bufferSize = _gazeBufferSize;
	WaBuffer.bufferSize = _waBufferSize;
	gazeStateClassifier.CursorJumpThresholdNormalized = _CursorJumpThresholdNormalized;
}

GazeFilters::~GazeFilters()
{}

PointD GazeFilters::filterGazeData(PointD GazePoint)
{
	if (GazePoint.IsNaN())
		return GazePoint;
	else
	{
		if (gazeStateClassifier.gazeFix1(GazePoint, lastFilterReturn))
		{
			if (filterTypeSelected == filtertype::meanMedian)
				lastFilterReturn = GazeBuffer.getMeanMedianBuffer(GazePoint);
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
