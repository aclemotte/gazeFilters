
#include "PointD.h"
#include <queue>
using namespace std;


class BufferPointD
{
public: 
	BufferPointD();
	~BufferPointD();
	PointD getMeanMedianBuffer(PointD GazePoint);
	PointD getAverageBuffer(PointD GazePoint);
	PointD getStdBuffer(PointD avgPoint);
	PointD getWABuffer(PointD GazePoint);
	void addPointD2Buffer(PointD GazePoint);
	void clearBuffer();
	int bufferSize;

private:
	deque<double> GazeBufferX;
	deque<double> GazeBufferY;
};