// RVLPCSdemo.cpp : Defines the entry point for the console application.
//

#include <Windows.h>	// for time measurement only!

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

//Karlo *********** START *********
#include "TestDescriptors.h"
//Karlo ***********  END  *********

#define GENERATE_DESCRIPTORS  //Karlo ***********
//#define RVLVTK


using namespace RVL;



void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	bool &bSHOT,
	bool &bSHOTColor,
	bool &bSkipImagesWithoutSelectedSegments,
	float &SHOTRadius)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SHOT", RVLPARAM_TYPE_BOOL, &bSHOT);
	pParamData = pParamList->AddParam("SHOTColor", RVLPARAM_TYPE_BOOL, &bSHOTColor);
	pParamData = pParamList->AddParam("skipImagesWithoutSelectedSegments", RVLPARAM_TYPE_BOOL, &bSkipImagesWithoutSelectedSegments);
	pParamData = pParamList->AddParam("SHOTRadius", RVLPARAM_TYPE_FLOAT, &SHOTRadius);
}

int main(int argc, char* argv[])
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	// Read parameters from a configuration file.

	bool bSHOT = false;
	bool bSHOTColor = false;
	bool bSkipImagesWithoutSelectedSegments = false;
	float SHOTRadius = 200.0f;

	CRVLParameterList ParamList;

	CreateParamList(
		&ParamList,
		&mem0,
		bSHOT,
		bSHOTColor,
		bSkipImagesWithoutSelectedSegments,
		SHOTRadius);

	ParamList.LoadParams("RVLPCSdemo.cfg");

	// create vision system

	CRVLPCSVS VS;

	// initialize vision system

	VS.CreateParamList();

	VS.Init("RVLPCSdemo.cfg");

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

	FILE *fpDescGenTime = fopen("DescGenTime.txt", "w");

#ifdef GENERATE_DESCRIPTORS

	Array2D<short int> depthImage;


	// Create RGB-D camera.
	RGBDCamera camera;
	camera.depthFu *= 0.5;
	camera.depthFv *= 0.5;
	camera.depthUc *= 0.5;
	camera.depthVc *= 0.5;

	// Create point cloud.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(pDepthImage->Width, pDepthImage->Height));

	//Create CTI template
	Array2D<float> CTIDescriptorTemplate;
	ConvexTemplate(CTIDescriptorTemplate);

	DESCRIPTOR_INFO CTIInfo;
	CTIInfo.iDescriptorLength = CTIDescriptorTemplate.h;
	CTIInfo.iNoOfFeatures = 0;

	//DESCRIPTOR_INFO CTIColorInfo;
	//CTIColorInfo.iDescriptorLength = CTIDescriptorTemplate.h + 2;
	//CTIColorInfo.iNoOfFeatures = 0;


	DESCRIPTOR_INFO SHOTColorInfo;
	SHOTColorInfo.iNoOfFeatures = 0;

	DESCRIPTOR_INFO SHOTGrayInfo;
	SHOTGrayInfo.iNoOfFeatures = 0;

	float tCTI, tCTIColor, tNormals, tSHOT, tSHOTColor;

	//FILE *fp = fopen("A.txt", "wb");

	//fwrite(CTIDescriptorTemplate.Element, sizeof(float), CTIDescriptorTemplate.w * CTIDescriptorTemplate.h, fp);

	//fclose(fp);
	
#endif

	float tSegmentation;
	bool bContinue = true;

	LARGE_INTEGER tStart, tEnd;
	LARGE_INTEGER frequency;
	QueryPerformanceFrequency((LARGE_INTEGER *)&frequency);
	
	do
	{
#ifdef RVLOPENNI
		if (VS.m_Flags & RVLSYS_FLAGS_KINECT)
		{
			// acquire depth image from Kinect

			if (GUI.m_bNextImage)
			{
				if (VS.m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
				{
					if (bSkipImagesWithoutSelectedSegments)
					{
						while (true)
						{
							char *SelectedSegmentsFileName = RVLKinectCreateONISampleFileName(VS.m_Kinect.m_ONIFileName, GUI.m_iONISample, "-SS.txt");

							FILE *fp = fopen(SelectedSegmentsFileName, "r");

							delete[] SelectedSegmentsFileName;

							if (fp)
								break;

							GUI.m_iONISample++;

							if (GUI.m_iONISample >= VS.m_Kinect.GetNoONIFrames())
							{
								bContinue = false;

								break;
							}
						}

						if (!bContinue)
							break;
					}

					printf("Processing ONI file %s, sample %d:\n", VS.m_Kinect.m_ONIFileName, GUI.m_iONISample);
				}

				VS.m_Kinect.GetImages(pDepthImage->Disparity, GUI.m_pRGBImage, NULL, GUI.m_pGSImage, pDepthImage->Format, GUI.m_iONISample);
			}
			
		}
		else
#endif
		// import depth image

		if(!VS.InputFromFile(pDepthImage, GUI.m_pRGBImage))
		{
			GUI.MessageCannotOpenFile(VS.m_ImageFileName);

			return 0;
		}

#ifdef GENERATE_DESCRIPTORS
		
		//
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
			printf("Segmentation...");

			QueryPerformanceCounter((LARGE_INTEGER *)&tStart);

			VS.Segment();

			QueryPerformanceCounter((LARGE_INTEGER *)&tEnd);
			tSegmentation = (tEnd.QuadPart - tStart.QuadPart) * 1000.0 / frequency.QuadPart;

			printf("completed.\n");

			GUI.m_ExecTime = tSegmentation;

			fprintf(fpExecTime, "%d\t%lf\n", GUI.m_iONISample, GUI.m_ExecTime);

			fflush(fpExecTime);

#ifdef GENERATE_DESCRIPTORS

			
			//If convex segments exist
			if (VS.m_nObjects > 0)
			{
				
				depthImage.Element = pDepthImage->Disparity;
				depthImage.w = pDepthImage->Width;
				depthImage.h = pDepthImage->Height;

				camera.GetPointCloud(&depthImage, GUI.m_pRGBImage, PC, false);

				CTIInfo.iNoOfFeatures = VS.m_nObjects;
				
				printf("Computimg CTI descriptors...");
				//Generate Descriptors
				//1. CTI

				QueryPerformanceCounter((LARGE_INTEGER *)&tStart);

				GenerateCTIDescriptors(&VS, &CTIInfo, &CTIDescriptorTemplate);

				QueryPerformanceCounter((LARGE_INTEGER *)&tEnd);
				tCTI = (tEnd.QuadPart - tStart.QuadPart) * 1000.0 / frequency.QuadPart;
				
				cvCvtColor(GUI.m_pRGBImage, GUI.m_pHSVImage, CV_BGR2RGB);
				GUI.m_pHSVImage->channelSeq[0] = 'R';
				GUI.m_pHSVImage->channelSeq[1] = 'G';
				GUI.m_pHSVImage->channelSeq[2] = 'B';

				RVLDominantSegmentColor2(&(VS.m_AImage.m_C2DRegion.m_ObjectList), VS.m_nObjects, GUI.m_pHSVImage, CTIInfo.pDominantColor);

				QueryPerformanceCounter((LARGE_INTEGER *)&tEnd);
				tCTIColor = (tEnd.QuadPart - tStart.QuadPart) * 1000.0 / frequency.QuadPart;

				printf("completed.\n");

				tSHOT = tSHOTColor = tNormals = 0;

				//2. SHOT
				//Get keypoints

				float *matKeyPoints = NULL;

				if (bSHOT || bSHOTColor)
					matKeyPoints = new float[3 * VS.m_nObjects];

				//Color SHOT

				if (bSHOTColor)
				{
					printf("Computimg SHOTColor descriptors...");
					SHOTColorInfo.iNoOfFeatures = VS.m_nObjects;
					GenerateSHOTKeypoints(&VS, matKeyPoints);
					GenerateSHOTDescriptors(&VS, PC, matKeyPoints, SHOTRadius, &SHOTColorInfo, tNormals, tSHOTColor);
					printf("completed.\n");
				}

				//Gray SHOT

				if (bSHOT)
				{
					printf("Computimg SHOT descriptors...");
					SHOTGrayInfo.iNoOfFeatures = VS.m_nObjects;
					GenerateSHOTKeypoints(&VS, matKeyPoints);
					GenerateSHOTDescriptors(&VS, PC, matKeyPoints, SHOTRadius, &SHOTGrayInfo, tNormals, tSHOT, false);
					printf("completed.\n");
				}

				if (matKeyPoints)
				delete[] matKeyPoints;

				fprintf(fpDescGenTime, "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n", 
					GUI.m_iONISample, 
					tSegmentation,
					1000.0f * ((float)tCTI) / CLOCKS_PER_SEC,
					1000.0f * ((float)tCTIColor) / CLOCKS_PER_SEC,
					1000.0f * ((float)tNormals) / CLOCKS_PER_SEC,
					1000.0f * ((float)tSHOT) / CLOCKS_PER_SEC,
					1000.0f * ((float)tSHOTColor) / CLOCKS_PER_SEC,
					VS.m_nObjects);

				fflush(fpDescGenTime);

			} //if (VS.m_nObjects>0)

#endif

		}	// if(!bRecord)


		// display the results
		bContinue = GUI.InteractiveVisualization();  
		
		if(GUI.m_bNextImage)
		{

#ifdef GENERATE_DESCRIPTORS
			//Save and Deallocate CTI
			if (CTIInfo.iNoOfFeatures > 0)
			{
				//Save
				//SaveSegmentsAndDescriptorsToFile(&VS, &CTIInfo, GUI.m_iONISample - 1, "-DescCTI.txt");

				SaveSegmentsAndDescriptorsToFile(&VS, &CTIInfo, GUI.m_iONISample - 1, "-DescCTIColor.txt",true);

				//Deallocate
				for (int i = 0; i < CTIInfo.iNoOfFeatures; i++)
				{
					delete[] CTIInfo.ppDescriptors[i];
				}
				delete[] CTIInfo.ppDescriptors;
				
			}
			//Reset
			CTIInfo.iNoOfFeatures = 0;



			//Save and Deallocate SHOT Color
			if (SHOTColorInfo.iNoOfFeatures > 0)
			{
				//Save
				SaveSegmentsAndDescriptorsToFile(&VS, &SHOTColorInfo, GUI.m_iONISample - 1, "-DescSHOTColor.txt");

				//Deallocate
				for (int i = 0; i < SHOTColorInfo.iNoOfFeatures; i++)
				{
					delete[] SHOTColorInfo.ppDescriptors[i];
				}
				delete[] SHOTColorInfo.ppDescriptors;

			}
			//Reset
			SHOTColorInfo.iNoOfFeatures = 0;

			//Save and Deallocate SHOT Gray
			if (SHOTGrayInfo.iNoOfFeatures > 0)
			{
				//Save
				SaveSegmentsAndDescriptorsToFile(&VS, &SHOTGrayInfo, GUI.m_iONISample - 1, "-DescSHOTGray.txt");

				//Deallocate
				for (int i = 0; i < SHOTGrayInfo.iNoOfFeatures; i++)
				{
					delete[] SHOTGrayInfo.ppDescriptors[i];
				}
				delete[] SHOTGrayInfo.ppDescriptors;

			}
			//Reset
			SHOTGrayInfo.iNoOfFeatures = 0;


#endif

			// get new sample name/ID

			if (!(VS.m_Flags & RVLSYS_FLAGS_KINECT))
				RVLGetNextFileName(VS.m_ImageFileName, "00000-LW.bmp", 10000);
		}

		if(!GUI.m_bRecord)
			VS.m_Mem.Clear();

		printf("\n");
	}
	while(bContinue);

	// free memory

	fclose(fpExecTime);
	fclose(fpDescGenTime);

	return 0;
}
