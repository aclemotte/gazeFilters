
#include "PointD.h"

class GazeStateClassifier
{
public:
	GazeStateClassifier();
	bool gazeFix1(PointD, PointD);
	double CursorJumpThresholdNormalized;

private:
};