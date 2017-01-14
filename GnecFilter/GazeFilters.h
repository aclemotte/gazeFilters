#pragma once

#include "PointD.h"
#include "GazeStateClassifier.h"
#include <queue>
using namespace std;

class GazeFilters
{
public:
	GazeFilters();
	GazeFilters(int _filterTypeSelected, int _gazeBufferSize, int _waBufferSize, double _CursorJumpThresholdNormalized);
	PointD filterGazeData(PointD GazePoint);
	~GazeFilters();
	enum filtertype { meanMedian, average, wa };

private:
	PointD getMeanMedianGazeFiltered(PointD GazePoint);
	PointD getMovingAverageGaze(PointD GazePoint);
	PointD getWA(PointD GazePoint);
	bool gazeFix1(PointD GazePoint);
	void addPointD2Buffer(PointD GazePoint);
	void addPointD2BufferWA(PointD GazePoint);
	void clearBuffers();
	void clearBuffersWA();
	GazeStateClassifier gazeStateClassifier;

private:	
	filtertype filterTypeSelected;
	int gazeBufferSize;
	int waBufferSize;

	deque<double> GazeBufferX;
	deque<double> GazeBufferY;
	deque<double> WaBufferX;
	deque<double> WaBufferY;

	//double CursorJumpThresholdNormalized;
	PointD lastFilterReturn;

};
