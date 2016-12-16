#ifndef GNECFILTERS_H
#define GNECFILTERS_H


#if !defined(WIN32) || defined(ANDROID) || defined(GNECFILTERS_STATIC)
	#define GNEC
#elif defined(GNECFILTERS_EXPORT)
	#define GNEC __declspec(dllexport)
#else
	#define GNEC __declspec(dllimport)
#endif


class GazeFilters;
class HeadFilter1;
class HeadFilter2;

class GNEC GnecGazeFilters
{
public:
	GnecGazeFilters();
	~GnecGazeFilters();
	void filterGazeData(float &x, float &y);	
	
private:	
	GazeFilters* _gazeFilter;
};

class GNEC GnecHeadFilter1
{
public:
	GnecHeadFilter1(float width, float height);
	~GnecHeadFilter1(void);
	
	void initializeHeadFilter(float screenWitdh, float screenHeight);
	float headFilterX(double inputX);
	float headFilterY(double inputY);	

private:
	HeadFilter1* _headFilter1;
};

class GNEC GnecHeadFilter2
{
public:
	GnecHeadFilter2(float width, float height);
	~GnecHeadFilter2(void);
	
	void initializeHeadFilter(float screenWitdh, float screenHeight);
	float headFilterX(double inputX);
	float headFilterY(double inputY);
	
private:
	HeadFilter2* _headFilter2;
};

#endif // GNECFILTERS_H