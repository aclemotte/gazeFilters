#pragma once

class PointD
{
public:
	double X;
	double Y;

public:
	PointD();
	PointD(double x, double y);
	static double distance(PointD a, PointD b);
	bool IsNaN();
	~PointD();
};

