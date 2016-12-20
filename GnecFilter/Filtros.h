#ifndef FILTER_LIB_H
#define FILTER_LIB_H
#include "GazeFilters.h"
#include "hf_03.h"

#if !defined(WIN32) || defined(FILTERS_STATIC)
#define FILTER_API
#elif defined(FILTERS_EXPORT)
#define FILTER_API __declspec(dllexport)
#else
#define FILTER_API __declspec(dllimport)
#endif


class FILTER_API filter1
{
public:	
	filter1();
	void filter(double &x, double &y);

private:
	GazeFilters* gazeFilter;
};

class FILTER_API filter2
{
public:
	filter2();
	void filter(double &x, double &y);
};

class FILTER_API filter3
{
public:
	filter3();
	void filter(double &x, double &y);
};

class FILTER_API filter4
{
public:
	filter4();
	void filter(double &x, double &y);
};

class FILTER_API filter5
{
public:
	filter5();
	void filter(double &x, double &y);
};

class FILTER_API filter6
{
public:
	filter6();
	void filter(double &x, double &y);
};

class FILTER_API filter7
{
public:
	filter7();
	void filter(double &x, double &y);
private:
	HeadFilter3* headFilter3;
};

class FILTER_API filter8
{
public:
	filter8();
	void filter(double &x, double &y);
};


#endif
