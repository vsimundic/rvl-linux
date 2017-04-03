#include "RVLPlatform.h"
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Graph.h"
#include <Eigen\Eigenvalues>
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "PCLMeshBuilder.h"
#include "RGBDCamera.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "VisualServo.h"

using namespace RVL;

VisualServo::VisualServo()
{
	pMem0 = NULL;
	flags = 0x00000000;
	depthImage.Element = NULL;
	RGBImage = NULL;
	GSImage = NULL;
	depthDisplayImage = NULL;
}


VisualServo::~VisualServo()
{
	RVL_DELETE_ARRAY(depthImage.Element);
	cvReleaseImage(&RGBImage);
	cvReleaseImage(&GSImage);
	cvReleaseImage(&depthDisplayImage);
}


void VisualServo::Init(char *CfgFileName)
{
	flags = 0x00000000;

	CreateParamList();
	
	paramList.LoadParams(CfgFileName);

#ifdef RVLOPENNI
	// initialize kinect

	if (kinect.Init())
	{
		flags |= RVLVISUALSERVO_FLAG_KINECT;

		kinect.m_scale = 1;

		depthImage.w = 640;
		depthImage.h = 480;

		int nPix = depthImage.w * depthImage.h;

		RVL_DELETE_ARRAY(depthImage.Element);

		depthImage.Element = new short int[nPix];

		int wRGB = depthImage.w / 2;
		int hRGB = depthImage.h / 2;

		cvReleaseImage(&RGBImage);

		RGBImage = cvCreateImage(cvSize(wRGB, hRGB), IPL_DEPTH_8U, 3);

		cvReleaseImage(&GSImage);

		GSImage = cvCreateImage(cvSize(wRGB, hRGB), IPL_DEPTH_8U, 1);

		cvReleaseImage(&depthDisplayImage);

		depthDisplayImage = cvCreateImage(cvSize(depthImage.w, depthImage.h), IPL_DEPTH_8U, 3);
	}
#endif
}

void VisualServo::CreateParamList()
{
	paramList.m_pMem = pMem0;

	RVLPARAM_DATA *pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("MeshBuilder.BilateralFilter.sigmaS", RVLPARAM_TYPE_DOUBLE, &sigmaS);
	pParamData = paramList.AddParam("MeshBuilder.BilateralFilter.sigmaR", RVLPARAM_TYPE_DOUBLE, &sigmaR);
}