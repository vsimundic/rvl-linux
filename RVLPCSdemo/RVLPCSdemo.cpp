// RVLPCSdemo.cpp : Defines the entry point for the console application.
//

#include "Platform.h"
#include <stdio.h>
#include <time.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLCore.h"
#include "RVLPCS.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "RGBDCamera.h"

using namespace RVL;

#define DIPLOMSKI_RADOCAJ

int main(int argc, char* argv[])
{
	// create vision system

	CRVLPCSVS VS;

	// initialize vision system

	VS.CreateParamList();

	VS.Init("RVLPCSdemo.cfg");

#ifdef DIPLOMSKI_RADOCAJ
	// Create RGB-D camera.

	RGBDCamera camera;
	Array2D<short int> depthImage;

	// Create point cloud.

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(320, 240));
#endif

	// create GUI

	CRVLPCSGUI GUI;

#ifdef RVLPSD_SEGMENT_STRM_DEBUG
	VS.m_PSD.m_DebugData.pGUI = &GUI;
#endif

	GUI.Init(&VS);

	// If Kinect is not available, display a message.

	if (!(VS.m_Flags & RVLSYS_FLAGS_KINECT))
		GUI.Message("Kinect is not available.", 400, 100, cvScalar(0, 128, 255));

#ifdef RVLOPENNI
	if ((VS.m_Flags & RVLSYS_FLAGS_KINECT) && GUI.m_bRecord)
		VS.m_StereoVision.m_DisparityMap.Format = RVLKINECT_DEPTH_IMAGE_FORMAT_1MM;
#endif

	// get the pointer to the depth image

	RVLDISPARITYMAP *pDepthImage;

	pDepthImage = &(VS.m_StereoVision.m_DisparityMap);

	FILE *fpExecTime = fopen("ExecTime.txt", "a");

	fprintf(fpExecTime, "=======\n");

	clock_t t;
	bool bContinue;

	do
	{
#ifdef RVLOPENNI
		if (VS.m_Flags & RVLSYS_FLAGS_KINECT)
		{
			// acquire depth image from Kinect

			if (GUI.m_bNextImage)
				VS.m_Kinect.GetImages(pDepthImage->Disparity, GUI.m_pRGBImage, NULL, GUI.m_pGSImage, pDepthImage->Format, GUI.m_iONISample);
		}
		else
#endif
		// import depth image

		if(!VS.InputFromFile(pDepthImage, GUI.m_pRGBImage))
		{
			GUI.MessageCannotOpenFile(VS.m_ImageFileName);

			return 0;
		}

#ifdef DIPLOMSKI_RADOCAJ
		Array2D<short int> depthImage;

		depthImage.Element = pDepthImage->Disparity;
		depthImage.w = pDepthImage->Width;
		depthImage.h = pDepthImage->Height;
		camera.depthFu *= 0.5;
		camera.depthFv *= 0.5;
		camera.depthUc *= 0.5;
		camera.depthVc *= 0.5;

		camera.GetPointCloud(&depthImage, GUI.m_pRGBImage, PC);
#endif

		if (GUI.m_bRecord)
		{
			if (VS.m_Flags & RVLSYS_FLAGS_PC)
				VS.SavePC();
			else
				VS.SaveRGBDImageToFile(pDepthImage, GUI.m_pRGBImage, "-LW.bmp");
		}
		else
		{
			t = clock();				

			VS.Segment();

			//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(320, 240)); //Radocaj
			//depthImage.Element = pDepthImage->Disparity; //Radocaj
			//depthImage.w = pDepthImage->Width; //Radocaj
			//depthImage.h = pDepthImage->Height; //Radocaj
			//camera.depthFu *= 0.5; //Radocaj
			//camera.depthFv *= 0.5; //Radocaj
			//camera.depthUc *= 0.5; //Radocaj
			//camera.depthVc *= 0.5; //Radocaj
			//camera.GetPointCloud(&depthImage, GUI.m_pRGBImage, PC);//Radocaj

			t = clock() - t;

			GUI.m_ExecTime = 1000.0f * ((float)t) / CLOCKS_PER_SEC;

			fprintf(fpExecTime, "%d\t%lf\n", GUI.m_iONISample, GUI.m_ExecTime);

			fflush(fpExecTime);
		}	// if(!bRecord)

		// display the results

		bContinue = GUI.InteractiveVisualization();

		if(GUI.m_bNextImage)
		{
			// get new sample name/ID

			if (!(VS.m_Flags & RVLSYS_FLAGS_KINECT))
				RVLGetNextFileName(VS.m_ImageFileName, "00000-LW.bmp", 10000);
		}

		if(!GUI.m_bRecord)
			VS.m_Mem.Clear();
	}
	while(bContinue);

	// free memory

	fclose(fpExecTime);

	return 0;
}
