
#include "BufferPointD.h"

BufferPointD::BufferPointD()
{
}

BufferPointD::~BufferPointD()
{
	clearBuffer();
}



//no hace lo que dice, falta hacer el sort
PointD BufferPointD::getMeanMedianBuffer(PointD GazePoint)
{
	addPointD2Buffer(GazePoint);

	//deben existir al menos tres puntos en el buffer para hacer lo siguiente
	if (GazeBufferX.size() > 2)
	{
		PointD gazeFiltered;
		//si es par el promedio de los 4 centrales
		//si es impar se promedia el del medio con sus vecinos

		if (GazeBufferX.size() % 2 == 0)//si es par
		{
			gazeFiltered.X = (
				GazeBufferX[(GazeBufferX.size() / 2) - 2] +
				GazeBufferX[(GazeBufferX.size() / 2) - 1] +
				GazeBufferX[(GazeBufferX.size() / 2) + 0] +
				GazeBufferX[(GazeBufferX.size() / 2) + 1]) / 4;
			gazeFiltered.Y = (
				GazeBufferY[(GazeBufferX.size() / 2) - 2] +
				GazeBufferY[(GazeBufferX.size() / 2) - 1] +
				GazeBufferY[(GazeBufferX.size() / 2) + 0] +
				GazeBufferY[(GazeBufferX.size() / 2) + 1]) / 4;
		}
		else//si es impar
		{
			gazeFiltered.X = (
				GazeBufferX[(GazeBufferX.size() / 2) - 1] +
				GazeBufferX[(GazeBufferX.size() / 2) + 0] +
				GazeBufferX[(GazeBufferX.size() / 2) + 1]) / 3;
			gazeFiltered.Y = (
				GazeBufferY[(GazeBufferX.size() / 2) - 1] +
				GazeBufferY[(GazeBufferX.size() / 2) + 0] +
				GazeBufferY[(GazeBufferX.size() / 2) + 1]) / 3;
		}
		return gazeFiltered;
	}
	else //si el buffer tiene menos de 3 elementos, se devuelve el argumento
	{
		return GazePoint;
	}
}

PointD BufferPointD::getAverageBuffer(PointD GazePoint)
{
	addPointD2Buffer(GazePoint);

	PointD gazeFiltered;

	for (unsigned int indiceBuffer = 0; indiceBuffer < GazeBufferX.size(); indiceBuffer++)
	{
		gazeFiltered.X += GazeBufferX[indiceBuffer];
		gazeFiltered.Y += GazeBufferY[indiceBuffer];
	}
	gazeFiltered.X = gazeFiltered.X / GazeBufferX.size();
	gazeFiltered.Y = gazeFiltered.Y / GazeBufferY.size();

	return gazeFiltered;
}

PointD BufferPointD::getStdBuffer(PointD avgPoint)
{
	PointD stdGaze;

	for (unsigned int indiceBuffer = 0; indiceBuffer < GazeBufferX.size(); indiceBuffer++)
	{
		stdGaze.X += (GazeBufferX[indiceBuffer] - avgPoint.X) * (GazeBufferX[indiceBuffer] - avgPoint.X);
		stdGaze.Y += (GazeBufferY[indiceBuffer] - avgPoint.Y) * (GazeBufferY[indiceBuffer] - avgPoint.Y);
	}

	stdGaze.X = sqrt((stdGaze.X / GazeBufferX.size()));
	stdGaze.Y = sqrt((stdGaze.Y / GazeBufferX.size()));

	return stdGaze;
}

PointD BufferPointD::getWABuffer(PointD GazePoint)
{
	addPointD2Buffer(GazePoint);

	double pxf = 0;
	double pyf = 0;
	double qlen = GazeBufferX.size();

	for (int i = 0; i<qlen; i++) {
		pxf += GazeBufferX.at(i) * (qlen - i) / (qlen*(qlen + 1) / 2);
		pyf += GazeBufferY.at(i) * (qlen - i) / (qlen*(qlen + 1) / 2);
	}

	PointD gazeFiltered(pxf, pyf);
	return gazeFiltered;
}



void BufferPointD::addPointD2Buffer(PointD GazePoint)
{
	if (GazeBufferX.size() >= bufferSize)
	{
		GazeBufferX.pop_back();
		GazeBufferY.pop_back();
	}

	GazeBufferX.push_front(GazePoint.X);
	GazeBufferY.push_front(GazePoint.Y);
}

void BufferPointD::clearBuffer()
{
	GazeBufferX.clear();
	GazeBufferY.clear();
}