#if _MSC_VER > 1000
#pragma once
#endif

using namespace RVL;



struct DESCRIPTOR_INFO
{
	float **ppDescriptors;
	int iDescriptorLength;
	int iNoOfFeatures;
	uchar *pDominantColor = NULL;
};


void GenerateCTIDescriptors(CRVLPCSVS *pVS, DESCRIPTOR_INFO *pDescInfo, Array2D<float> *pDescriptorTemplate);

void SaveSegmentsAndDescriptorsToFile(CRVLPCSVS *pVS, DESCRIPTOR_INFO *pDescInfo, int iPrevONISample, char *Extension, bool bAddCTIColor = false);

void SaveSegmentsAndDescriptors(
	CRVLMPtrChain *pTriangleList,
	int nSegments,
	DESCRIPTOR_INFO *pDescInfo,
	DWORD Flags,
	char *FileName,
	bool bAddCTIColor = false);

void ConvexTemplate(Array2D<float> &A);

void GenerateSHOTDescriptors(CRVLPCSVS* pVS, 
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene, 
	float *pKeypoints, 
	float r,
	DESCRIPTOR_INFO *pSHOTI, 
	float &tNormals,
	float &tDescriptors,
	bool bSHOTColor = true);
void GenerateSHOTKeypoints(CRVLPCSVS* pVS, float *pKeypoints);



class PixelPoint
{
public:
	int u, v;
	unsigned int r, g, b;
	PixelPoint(int mU = 0, int mV = 0, unsigned int mR = 0, unsigned int mG = 0, unsigned int mB = 0) :u(mU), v(mV), r(mR), g(mG), b(mB) //kada se pozivaju elementi svaki posebno
	{
	}
};


class Normal
{
public:
	double x, y, z;
	int n3DPoints;
	Normal(int mD = 0, double mX = 0.0, double mY = 0.0, double mZ = 0.0) : n3DPoints(mD), x(mX), y(mY), z(mZ) {}
};