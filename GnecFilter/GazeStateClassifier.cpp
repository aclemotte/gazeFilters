#include "GazeStateClassifier.h"

GazeStateClassifier::GazeStateClassifier()
{
	bufferPointD.clearBuffer();
}

//Detector de fijaciones basado en el tama�o de las sacadas I-ST (detector de sacadas)
//En algunos estudios este detector fue mejorado mediante el c�lculo de un umbral din�mico, de acuerdo con la se�al (Tole, 1981, Duchowski, 2007)
bool GazeStateClassifier::gazeFix1(PointD currentGazePoint, PointD lastGazePointFilteredReturned)
{
	if (PointD::distance(currentGazePoint, lastGazePointFilteredReturned) < minimoSaltoSacada)
		return true;
	else
		return false;
}

//Detector de fijaciones basado en la dispersi�n de los datos I-DT
//requiere dos par�metros:
//		1) el umbral de dispersi�n D y 2) el umbral de duraci�n m�nima de una fijaci�n.
//El umbral de dispersi�n se puede definir de muchas maneras: area, varianza (implementado), radio
//El umbral de duraci�n m�nima de una fijaci�n se establece normalmente a un valor entre 100 y 200 ms (Widdel, 1984).
bool GazeStateClassifier::gazeFix2(PointD currentGazePoint)
{
	PointD avgBuffer = bufferPointD.getAverageBuffer(currentGazePoint);
	PointD stdBuffer = bufferPointD.getStdBuffer(avgBuffer);

	if (bufferPointD.getCurrentBufferSize() >= bufferPointD.maxBufferSize)
	{
		if (stdBuffer.X < umbralDispersion && stdBuffer.Y < umbralDispersion)
		{
			return true;
		}
		else
		{
			return false;
		}
		return true;
	}
	else
	{
		return false;
	}

}

void GazeStateClassifier::setupGazeFix1(double _minimoSaltoSacada)
{
	minimoSaltoSacada = _minimoSaltoSacada;
}

void GazeStateClassifier::setupGazeFix2(double _umbralDispersion, int _duracionMinimaFijacionSamples)
{
	umbralDispersion = _umbralDispersion;
	bufferPointD.maxBufferSize = _duracionMinimaFijacionSamples;
}