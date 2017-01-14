#include "GazeStateClassifier.h"

GazeStateClassifier::GazeStateClassifier()
{

}

bool GazeStateClassifier::gazeFix1(PointD currentGazePoint, PointD lastGazePointFilteredReturned)
{
	if (PointD::distance(currentGazePoint, lastGazePointFilteredReturned) < CursorJumpThresholdNormalized)
		return true;
	else
		return false;
}