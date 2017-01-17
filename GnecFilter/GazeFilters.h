#pragma once

#include "PointD.h"
#include "BufferPointD.h"
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

private://variables
	GazeStateClassifier gazeStateClassifier;
	filtertype filterTypeSelected;
	BufferPointD GazeBuffer;
	BufferPointD WaBuffer;
	PointD lastFilterReturn;

};
