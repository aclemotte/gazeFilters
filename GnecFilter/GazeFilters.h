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

private://funciones
	PointD getMeanMedianGazeFiltered(PointD GazePoint);
	PointD getAverageGaze(PointD GazePoint);
	PointD getWA(PointD GazePoint);
	bool gazeFix1(PointD GazePoint);
	void addPointD2Buffer(PointD GazePoint);
	void addPointD2BufferWA(PointD GazePoint);
	void clearBuffers();
	void clearBuffersWA();

private://variables
	GazeStateClassifier gazeStateClassifier;
	filtertype filterTypeSelected;
	int gazeBufferSize;
	int waBufferSize;

	deque<double> GazeBufferX;
	deque<double> GazeBufferY;
	deque<double> WaBufferX;
	deque<double> WaBufferY;

	PointD lastFilterReturn;

};
