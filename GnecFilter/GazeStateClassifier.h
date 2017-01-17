
#include "BufferPointD.h"


class GazeStateClassifier
{
public:
	GazeStateClassifier();

	bool gazeFix1(PointD, PointD);
	void setupGazeFix1(double minimoSaltoSacada);
	bool gazeFix2(PointD);
	void setupGazeFix2(double umbralDispersion, int duracionMinimaFijacionSamples);

private:
	//gazeFix1
	double minimoSaltoSacada;//minima distancia de salto a partir de la cual se considera una sacada
	//gazeFix2
	double umbralDispersion;
	//int duracionMinimaFijacionSamples;
	BufferPointD bufferPointD;
};