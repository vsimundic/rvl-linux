
//#include "stdafx.h"
#include "atlstr.h"
#include <iostream>
#include <string>
#include <sstream>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"

#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"




#include "GTTools.h"



#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "openni2_data_capture.h"







using namespace RVL;
using namespace cv;
using namespace std;
//using namespace pcl;

#define RVLPCGT_DEMO_FLAG_SAVE_PLY					0x00000001
#define RVLPCGT_DEMO_FLAG_DISABLE_TRANSFORM			0x00000100
#define RVLPCGT_DEMO_FLAG_USER_SURFEL				0x10000000

#define RVLPCGT_MODE_FLAG_RECORD					0
#define RVLPCGT_MODE_FLAG_GENERATE_GT				1
#define RVLPCGT_MODE_FLAG_GT_OBJECT_SELECTION		2

// COENE struct needed to pass to callback function
struct Group_Structure_VTK{
	vtkSmartPointer<vtkRenderWindow> *window;
	RVLGT_IMAGE_DETAILS* details;
	Mesh* mesh;
};

// COENE definition of needed functions
void flood(int* image, RVLGT_SEGMENTATION_PARAMS *pGTSegmentParams);
vtkSmartPointer<vtkRenderWindow> vtk_initialiser(Mesh& mesh, RVLGT_IMAGE_DETAILS *pImgDetails);
void DisplaySegmentationInMesh(vtkSmartPointer<vtkRenderWindow> window, Mesh &mesh, RVLGT_IMAGE_DETAILS *pImgDetails);
void userFunction(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata);
void zoomROI(RVLGT_IMAGE_DETAILS& CurrentImageDetails, Mat* image_roi, int* clickedpoints);
void my_mouse_callback_ClickPoint_ROI(int event, int x, int y, int flags, void* param);
////////

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void RVLflood(int* image, RVLGT_SEGMENTATION_PARAMS *pGTSegmentParams);

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	RVLGT_SEGMENTATION_PARAMS *pGTSegmentParams,
	RVLGT_PCLSUPERVOXEL_PARAMS *pPCLSuperVoxelParams,
	DWORD &flags,
	DWORD &mode,
	char **pSequenceFileName)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("GT.ImageLocation", RVLPARAM_TYPE_STRING, &(pGTSegmentParams->pDefaultFileLocation));
	pParamData = pParamList->AddParam("GT.SequenceFileName", RVLPARAM_TYPE_STRING, pSequenceFileName);	//VIDOVIC
	pParamData = pParamList->AddParam("GT.InitialImageNo", RVLPARAM_TYPE_INT, &(pGTSegmentParams->iImageNo));
	pParamData = pParamList->AddParam("GT.DifferenceThreshold", RVLPARAM_TYPE_INT, &(pGTSegmentParams->DiffThreshold));
	pParamData = pParamList->AddParam("GT.PercThreshold", RVLPARAM_TYPE_FLOAT, &(pGTSegmentParams->PercThreshold));
	pParamData = pParamList->AddParam("GT.MinNoOfPoints", RVLPARAM_TYPE_INT, &(pGTSegmentParams->MinNoOfPoints));
	pParamData = pParamList->AddParam("GT.MaxDist", RVLPARAM_TYPE_INT, &(pGTSegmentParams->MaxDist));
	pParamData = pParamList->AddParam("GT.minConnectedComponentSize", RVLPARAM_TYPE_INT, &(pGTSegmentParams->minConnectedComponentSize));
	pParamData = pParamList->AddParam("GT.mode", RVLPARAM_TYPE_ID, &mode);
	pParamList->AddID(pParamData, "GENERATE_GT", RVLPCGT_MODE_FLAG_GENERATE_GT);
	pParamList->AddID(pParamData, "OBJECT_SELECTION", RVLPCGT_MODE_FLAG_GT_OBJECT_SELECTION);
	pParamList->AddID(pParamData, "RECORD", RVLPCGT_MODE_FLAG_RECORD);

	pParamData = pParamList->AddParam("GT.UseSurfel", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLPCGT_DEMO_FLAG_USER_SURFEL);
	
		
	pParamData = pParamList->AddParam("SuperVoxel.disable_transform", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLPCGT_DEMO_FLAG_DISABLE_TRANSFORM);

	pParamData = pParamList->AddParam("SuperVoxel.voxel_resolution", RVLPARAM_TYPE_FLOAT, &(pPCLSuperVoxelParams->voxel_resolution));
	pParamData = pParamList->AddParam("SuperVoxel.seed_resolution", RVLPARAM_TYPE_FLOAT, &(pPCLSuperVoxelParams->seed_resolution));
	pParamData = pParamList->AddParam("SuperVoxel.color_importance", RVLPARAM_TYPE_FLOAT, &(pPCLSuperVoxelParams->color_importance));
	pParamData = pParamList->AddParam("SuperVoxel.spatial_importance", RVLPARAM_TYPE_FLOAT, &(pPCLSuperVoxelParams->spatial_importance));
	pParamData = pParamList->AddParam("SuperVoxel.normal_importance", RVLPARAM_TYPE_FLOAT, &(pPCLSuperVoxelParams->normal_importance));

	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLPCGT_DEMO_FLAG_SAVE_PLY);
}

//void DisplaySegmentedImage(RVLGT_IMAGE_DETAILS *pImgDetails)
//{
//	cv::Mat segmentedImage;
//
//	int iseg, currIdx, iLabel;
//	int *psegColor;
//	cv::Vec3b color;
//
//	float opacity = 0.7;
//
//	//Create copy of original
//	pImgDetails->pRGBImage->copyTo(segmentedImage);
//
//	for (int row = 0; row < pImgDetails->pRGBImage->rows; ++row)
//	{
//		for (int col = 0; col < pImgDetails->pRGBImage->cols; ++col)
//		{
//
//			currIdx = row*GT_IMWIDTH + col;
//			iLabel = pImgDetails->pGTMask[currIdx];
//
//			if (iLabel > -1)
//			{
//				iseg = iLabel % 37;
//			}
//			else
//			{
//				iseg = 37;
//			}
//
//			psegColor = pImgDetails->segmentColor + iseg * 3;
//
//			color = segmentedImage.at<cv::Vec3b>(cv::Point(col, row));
//
//			for (int i = 0; i < 3; i++)
//			{
//				color[i] = color[i] * (1 - opacity) + psegColor[i] * opacity;
//			}
//
//			//save pixel
//			segmentedImage.at<cv::Vec3b>(cv::Point(col, row)) = color;
//
//		}
//	}
//
//	cv::namedWindow(pImgDetails->pWindowTitle);
//	cv::imshow(pImgDetails->pWindowTitle, segmentedImage);
//	cv::waitKey(1);
//
//}

void DisplaySegmentedImage(RVLGT_IMAGE_DETAILS *pImgDetails, int startw = 0, int starth = 0, bool large = false)		//COENE, dynamicly adaptable to ROI
{
	cv::Mat segmentedImage;

	int iseg, currIdx, iLabel;
	int *psegColor;
	cv::Vec3b color;

	float opacity = 0.7;

	//Create copy of original
	pImgDetails->pRGBImage->copyTo(segmentedImage);

	for (int row = 0; row < pImgDetails->pRGBImage->rows; ++row)
	{
		for (int col = 0; col < pImgDetails->pRGBImage->cols; ++col)
		{
			//COENE, made calculation dynamicly adaptable to ROI
			currIdx = row*GT_IMWIDTH + col + ((starth * GT_IMWIDTH) + startw);
			/////////////
			iLabel = pImgDetails->pGTMask[currIdx];

			if (iLabel > -1)
			{
				iseg = iLabel % 37;
			}
			else
			{
				iseg = 37;
			}

			psegColor = pImgDetails->segmentColor + iseg * 3;

			color = segmentedImage.at<cv::Vec3b>(cv::Point(col, row));

			for (int i = 0; i < 3; i++)
			{
				color[i] = color[i] * (1 - opacity) + psegColor[i] * opacity;
			}

			//save pixel
			segmentedImage.at<cv::Vec3b>(cv::Point(col, row)) = color;

		}
	}

	// COENE, dynamicly adaptable to ROI
	if (large){
		namedWindow(pImgDetails->pWindowTitle, WINDOW_NORMAL);
	}
	else{
		cv::namedWindow(pImgDetails->pWindowTitle);
	}
	////////

	cv::imshow(pImgDetails->pWindowTitle, segmentedImage);
	cv::waitKey(1);

}


//void SwitchLabels(RVLGT_IMAGE_DETAILS *pImgDetails)
//{
//	//Make sure all clicked points exist
//	if (pImgDetails->pClickedPoints[0]>-1 && pImgDetails->pClickedPoints[1]>-1 && pImgDetails->pClickedPoints[2]>-1 && pImgDetails->pClickedPoints[3]>-1)
//	{
//		int idxSource = pImgDetails->pClickedPoints[1] * GT_IMWIDTH + pImgDetails->pClickedPoints[0];
//		int idxTarget = pImgDetails->pClickedPoints[3] * GT_IMWIDTH + pImgDetails->pClickedPoints[2];
//
//		int lblSource = pImgDetails->pLabelMap[idxSource];
//
//		int lblGTTarget = pImgDetails->pGTMask[idxTarget];
//
//		int currIdx;
//
//		//NOTE: lblSource and lblGTTarget do not have have the same meaning!!!
//		//lblSource is obtained from the surfel or supervoxel map
//		//lblGTTarget is obtained from the derived ground truth mask
//		//lblSource = -1 means undefined pixel in surfel or supervoxel map; 
//		//lblGTTarget = -1 means undefined pixel in surfel or supervoxel map + pixels with depth value greater than MaxDist; 
//		if (lblSource > -1 && lblSource < pImgDetails->maxLabelNo && lblGTTarget > -1)
//		{
//			for (int v = 0; v < GT_IMHEIGHT; v++)
//			{
//				for (int u = 0; u < GT_IMWIDTH; u++)
//				{
//					currIdx = v*GT_IMWIDTH + u;
//
//					if (pImgDetails->pLabelMap[currIdx] == lblSource)
//					{
//						pImgDetails->pGTMask[currIdx] = lblGTTarget;
//					}
//
//				}
//			}
//			DisplaySegmentedImage(pImgDetails);
//		}
//		
//	}
//	
//}

void SwitchLabels(RVLGT_IMAGE_DETAILS *pImgDetails, int startw = 0, int starth = 0, int width = GT_IMWIDTH, int height = GT_IMHEIGHT)	// COENE, dynamicly adaptable to ROI
{
	//Make sure all clicked points exist
	if (pImgDetails->pClickedPoints[0]>-1 && pImgDetails->pClickedPoints[1]>-1 && pImgDetails->pClickedPoints[2]>-1 && pImgDetails->pClickedPoints[3]>-1)
	{
		// COENE made calculation dynamicly adaptable to ROI
		int idxSource = pImgDetails->pClickedPoints[1] * GT_IMWIDTH + pImgDetails->pClickedPoints[0] + ((starth * GT_IMWIDTH) + startw);
		int idxTarget = pImgDetails->pClickedPoints[3] * GT_IMWIDTH + pImgDetails->pClickedPoints[2] + ((starth * GT_IMWIDTH) + startw);
		////

		bool* pixelmode = pImgDetails->pixelmode;

		int lblSource = pImgDetails->pLabelMap[idxSource];

		int lblGTTarget = pImgDetails->pGTMask[idxTarget];

		int currIdx;

		//NOTE: lblSource and lblGTTarget do not have have the same meaning!!!
		//lblSource is obtained from the surfel or supervoxel map
		//lblGTTarget is obtained from the derived ground truth mask
		//lblSource = -1 means undefined pixel in surfel or supervoxel map; 
		//lblGTTarget = -1 means undefined pixel in surfel or supervoxel map + pixels with depth value greater than MaxDist; 

		if (lblSource > -1 && lblSource < pImgDetails->maxLabelNo && lblGTTarget > -1)
		{
			if (!(*pixelmode)){
				for (int v = 0; v < height; v++)		// COENE, dynamicly adaptable to ROI
				{
					for (int u = 0; u < width; u++)		// COENE, dynamicly adaptable to ROI
					{
						// COENE, dynamicly adaptable to ROI
						currIdx = v*GT_IMWIDTH + u + ((starth * GT_IMWIDTH) + startw);
						////

						if (pImgDetails->pLabelMap[currIdx] == lblSource)
						{
							pImgDetails->pGTMask[currIdx] = lblGTTarget;
						}

					}
				}
			}
			else{
				pImgDetails->pGTMask[idxSource] = pImgDetails->pGTMask[idxTarget];
			}

			DisplaySegmentedImage(pImgDetails, startw, starth, startw != 0);	// COENE, dynamicly adaptable to ROI
		}

	}

}


void my_mouse_callback_ClickPoint(int event, int x, int y, int flags, void* param)
{
	RVLGT_IMAGE_DETAILS* pCurrentImageDetails = (RVLGT_IMAGE_DETAILS *)param;

	switch (event)
	{

	case CV_EVENT_LBUTTONUP:
		pCurrentImageDetails->pClickedPoints[0] = x;
		pCurrentImageDetails->pClickedPoints[1] = y;
		SwitchLabels(pCurrentImageDetails);
		break;

	case CV_EVENT_RBUTTONUP:
		pCurrentImageDetails->pClickedPoints[2] = x;
		pCurrentImageDetails->pClickedPoints[3] = y;
		break;

	}
}


bool RecordData(RVLGT_SEGMENTATION_PARAMS *pGTSegmentParams, PCLMeshBuilder *pMeshBuilder, DWORD *pFlags)
{
	//OpenNI2 streaming object
	openni2_data_capture::DataCapture* dc = new openni2_data_capture::DataCapture();

	//Initialize and start capture
	dc->initialize();
	dc->startDataCapture();

	//Create opencv windows for viewing purposes
	cvNamedWindow("RGBFeed", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("DepthFeed", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("DepthFiltered", CV_WINDOW_AUTOSIZE);

	//Create mats and iplimages for viewing and storage
	Mat rgbMat(GT_IMHEIGHT / 2, GT_IMWIDTH / 2, CV_8UC3);        //Mat rgbMat(GT_GT_IMHEIGHT, GT_IMWIDTH, CV_8UC3);
	Mat depthMatView(GT_IMHEIGHT, GT_IMWIDTH, CV_8UC1);
	Mat depthMatOrig(GT_IMHEIGHT, GT_IMWIDTH, CV_16UC1);

	Mat depthMatOrigSum = Mat::zeros(GT_IMHEIGHT, GT_IMWIDTH, CV_16UC1);
	Mat depthMatOrigCnt = Mat::zeros(GT_IMHEIGHT, GT_IMWIDTH, CV_16UC1);
	Mat depthMatFilteredView(GT_IMHEIGHT, GT_IMWIDTH, CV_8UC1);


	IplImage* rgbImage = cvCreateImageHeader(cvSize(rgbMat.cols, rgbMat.rows), IPL_DEPTH_8U, 3);
	IplImage* depthImg = cvCreateImageHeader(cvSize(depthMatView.cols, depthMatView.rows), IPL_DEPTH_8U, 1);
	IplImage* depthImgFiltered = cvCreateImageHeader(cvSize(depthMatOrigSum.cols, depthMatOrigSum.rows), IPL_DEPTH_8U, 1);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(GT_IMWIDTH, GT_IMHEIGHT));
	pcl::PolygonMesh PCLMesh;

	//Helper variables
	double minVal, maxVal; //for rescaling
	bool bStartImageSequence = false;
	int cntSequence = 0;
	int imageNo = -1;


	while (true)
	{
		//Capture image from sensor
		dc->captureOne();

		//Copy captured images to mat 
		memcpy(rgbMat.data, dc->getRGBData(), 3 * rgbMat.rows * rgbMat.cols * sizeof(uint8_t));
		memcpy(depthMatOrig.data, dc->getDepthData(), depthMatOrig.rows * depthMatOrig.cols * sizeof(uint16_t));

		//Rescale depth image for better viewing
		minMaxLoc(depthMatOrig, &minVal, &maxVal);
		depthMatOrig.convertTo(depthMatView, CV_8U, -255.0f / maxVal, 255.0f);

		//Copy mat to iplimages
		cvSetData(rgbImage, rgbMat.data, 3 * rgbMat.cols);
		cvSetData(depthImg, depthMatView.data, depthMatView.cols);

		//Convert BGR to RGB
		cvCvtColor(rgbImage, rgbImage, CV_BGR2RGB);

		//Display current images
		cvShowImage("DepthFeed", depthImg);
		cvShowImage("RGBFeed", rgbImage);


		//Store image sequences
		char c = cvWaitKey(33);
		if ((c == 32) && (bStartImageSequence == false))  //Space
		{
			imageNo++;

			//Reset everything
			cntSequence = 0;
			bStartImageSequence = true;

			depthMatOrigSum = Scalar::all(0);
			depthMatOrigCnt = Scalar::all(0);

			printf("Image %d\n", imageNo);
			printf(" >> Acquiring data...");

		}


		if (bStartImageSequence)
		{
			if (cntSequence < GT_SEQUENCE_LENGTH)
			{
				//Add images
				add(depthMatOrigSum, depthMatOrig, depthMatOrigSum);

				//Check values greater than 0
				for (int r = 0; r < depthMatOrig.rows; r++)
				{
					for (int c = 0; c < depthMatOrig.cols; c++)
					{
						if (depthMatOrig.at<ushort>(r, c)>0)
						{
							depthMatOrigCnt.at<ushort>(r, c) += 1;
						}
					}
				}

				cntSequence++;
			}
			else
			{
				bStartImageSequence = false;
				cntSequence = 0;

				minMaxLoc(depthMatOrigSum, &minVal, &maxVal);
				minMaxLoc(depthMatOrigCnt, &minVal, &maxVal);

				//Find average value

				for (int r = 0; r < depthMatOrigCnt.rows; r++)
				{
					for (int c = 0; c < depthMatOrigCnt.cols; c++)
					{
						if (depthMatOrigCnt.at<ushort>(r, c)>0)
						{
							depthMatOrigSum.at<ushort>(r, c) /= depthMatOrigCnt.at<ushort>(r, c);
						}
					}
				}

				//Rescale depth image for better viewing
				cv::Point min_loc;
				cv::Point max_loc;
				minMaxLoc(depthMatOrigSum, &minVal, &maxVal, &min_loc, &max_loc);
				depthMatOrigSum.convertTo(depthMatFilteredView, CV_8U, -255.0f / maxVal, 255.0f);

				cvSetData(depthImgFiltered, depthMatFilteredView.data, depthMatFilteredView.step);
				cvShowImage("DepthFiltered", depthImgFiltered);


				printf("completed.\n");
				printf(" >> Saving data...\n");
				
				SaveFiles(imageNo, rgbImage, &depthMatOrigSum, &depthMatOrig, depthMatOrigSum.cols, depthMatOrigSum.rows, pGTSegmentParams->pDefaultFileLocation);


				if ((*pFlags & RVLPCGT_DEMO_FLAG_SAVE_PLY) != 0)
				{
					//Save PLY file
					/*************/
					printf("  >> Saving PLY file...\n");

					Array2D<short int> depthImage;
					RGBDCamera camera;

					depthImage.w = GT_IMWIDTH;
					depthImage.h = GT_IMHEIGHT;
					depthImage.Element = new short int[GT_IMWIDTH * GT_IMHEIGHT];

					short int *pDisparity = depthImage.Element;

					for (int v = 0; v < GT_IMHEIGHT; v++)
					{
						for (int u = 0; u < GT_IMWIDTH; u++, pDisparity++)
						{
							*pDisparity = depthMatOrigSum.at<ushort>(v, u);
						}
					}


					printf("  >> >>Creating mesh from point cloud...");

					camera.GetPointCloud(&depthImage, rgbImage, PC);

					pMeshBuilder->CreateMesh(PC, PCLMesh);

					printf("completed.\n");


					char *plyFileName = CreateFileName(imageNo, pGTSegmentParams->pDefaultFileLocation, ".ply");

					printf("  >> >>Saving mesh...");

					PCLSavePLY(plyFileName, PCLMesh);

					printf("completed.\n");
				}
				
				printf(" >> Data saved.\n");
			}


		}


		if (c == 27) //Esc
			break;
	}

	cvDestroyAllWindows();
	return true;
}


bool GenerateGT(RVLGT_PCLSUPERVOXEL_PARAMS *pPCLSuperVoxelParams, RVLGT_SEGMENTATION_PARAMS *pGTSegmentParams, char *pSegmentLookupFile, PCLMeshBuilder *pMeshBuilder, PlanarSurfelDetector *pDetector, CRVLMem *pmem, DWORD *pFlags)
{
		
	//Surfel variables
	Mesh mesh;
	SurfelGraph surfels;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(GT_IMWIDTH, GT_IMHEIGHT));
	pcl::PolygonMesh PCLMesh;

	//Supervoxel variables
	PointCloudT::Ptr cloud = boost::shared_ptr <PointCloudT>(new PointCloudT());
	PointLCloudT::Ptr labeled_cloud;

	//Common variables
	char *pCurrentFileName;
	char *windowNameRGB = "Current RGB image";
	char *windowNameDepth = "Current depth image";
	

	if ((*pFlags & RVLPCGT_DEMO_FLAG_USER_SURFEL) != 0)
	{
		printf("*************************************************************\n");
		printf("*********  GROUND TRUTH SEGMENTATION USING SURFELS  *********\n");
		printf("*************************************************************\n");
		pCurrentFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, ".ply");
	}
	else
	{
		printf("*************************************************************\n");
		printf("*******  GROUND TRUTH SEGMENTATION USING SUPERVOXELS  *******\n");
		printf("*************************************************************\n");
		pCurrentFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, ".pcd");
	}


	int *pLabelCount = new int[GT_MAXLABELVAL + 1];
	int *pLabelTotal = new int[GT_MAXLABELVAL + 1];
	float *pLabelPerc = new float[GT_MAXLABELVAL + 1];
	int *pLabelMapHelper = new int[GT_IMWIDTH*GT_IMHEIGHT];
	int *pDepth0 = new int[GT_IMWIDTH*GT_IMHEIGHT];
	int *pDepth1 = new int[GT_IMWIDTH*GT_IMHEIGHT];
	int *pDepthDiffMat = new int[GT_IMWIDTH*GT_IMHEIGHT];
	int *pGTMask0 = new int[GT_IMWIDTH*GT_IMHEIGHT];
	int *pGTMask1 = new int[GT_IMWIDTH*GT_IMHEIGHT];
	int *pClickedPoints = new int[4];
	int *pSegmentColor = new int[3 * 38];

	//Reset mat values
	memset(pLabelCount, 0, (GT_MAXLABELVAL + 1) * sizeof(int));
	memset(pLabelTotal, 0, (GT_MAXLABELVAL + 1) * sizeof(int));
	memset(pLabelPerc, 0, (GT_MAXLABELVAL + 1) * sizeof(float));
	memset(pLabelMapHelper, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
	memset(pDepth0, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
	memset(pDepth1, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
	memset(pDepthDiffMat, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
	memset(pGTMask0, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
	memset(pGTMask1, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
	memset(pClickedPoints, -1, 4 * sizeof(int));

	//int *pLabelCount;
	//int *pLabelTotal;
	//float *pLabelPerc;
	int *pLabelMap;

	//Images to display
	Mat imageInput;
	Mat imageInputOrig;
	IplImage *depthImageInput;


	int currLabel, maxNoLabels;
	int currIdx;
	int depthDiff;

	char *pCurrentRGBFileName;
	char *pCurrentDepthFileName;
	char *pCurrentMaskFileName;
	
	RVLGT_IMAGE_DETAILS CurrentImageDetails;

	ReadDataFile(pSegmentColor, pSegmentLookupFile, false);

	CurrentImageDetails.segmentColor = pSegmentColor;
	CurrentImageDetails.pWindowTitle = windowNameRGB;

	// COENE, setting up variables

	Mat* image_roi = NULL;
	int clickedpoints_ROI[4];
	bool pixelmode = false;
	////////////

	bool bLoadPtCloud = true;

	while (true)
	{
		char c = cvWaitKey(33);

#pragma region Load point cloud

		//////////////////////////////  //////////////////////////////
		//////LOAD POINT CLOUD
		//////////////////////////////  //////////////////////////////
		if (bLoadPtCloud)
		{

			//Read RGB image
			pCurrentRGBFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, "_R.bmp");
			imageInputOrig = imread(pCurrentRGBFileName, CV_LOAD_IMAGE_COLOR);

			printf("\nImage: %d\n", pGTSegmentParams->iImageNo);

			//Read depth image
			pCurrentDepthFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, ".txt");

			Array2D<short int> depthImage;

			depthImage.Element = NULL;
			depthImage.w = depthImage.h = 0;

			unsigned int format;

			ImportDisparityImage(pCurrentDepthFileName, depthImage, format);

			depthImageInput = cvCreateImage(cvSize(depthImage.w, depthImage.h), IPL_DEPTH_8U, 3);
			DisplayDisparityMap(depthImage, (unsigned char *)(depthImageInput->imageData), true, RVLRGB_DEPTH_FORMAT_1MM);

			//Display images
			namedWindow(windowNameRGB);
			imshow(windowNameRGB, imageInputOrig);
			namedWindow(windowNameDepth);
			cvShowImage(windowNameDepth, depthImageInput);
			waitKey(1);

			cvReleaseImage(&depthImageInput);

			CurrentImageDetails.pRGBImage = &imageInputOrig;

			bLoadPtCloud = false;

			if ((*pFlags & RVLPCGT_DEMO_FLAG_USER_SURFEL) != 0)
			{
				printf(" >> Loading PLY file...");
				//if (mesh.Load(pCurrentFileName, pMeshBuilder, PC, PCLMesh, (*pFlags & RVLPCGT_DEMO_FLAG_SAVE_PLY) != 0))
				if (LoadMesh(pMeshBuilder, pCurrentFileName, &mesh, (*pFlags & RVLPCGT_DEMO_FLAG_SAVE_PLY) != 0))
				{
					surfels.Init(&mesh);
					pDetector->Init(&mesh, &surfels, pmem);
					printf(" >> Extracting surfels...");
					pDetector->Segment(&mesh, &surfels);
					printf("completed. Found %d surfels.\n", surfels.NodeArray.n);
					printf(">> Saving surfels...");								// CUPEC
					char *surfelFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, ".sur");		// CUPEC
					FILE *fpSurfels = fopen(surfelFileName, "wb");				// CUPEC
					surfels.Save(fpSurfels, pCurrentFileName, pDetector);		// CUPEC
					fclose(fpSurfels);											// CUPEC
					delete[] surfelFileName;									// CUPEC
					printf("completed.\n");										// CUPEC
					//Visualizer visualizer;
					//visualizer.Create();
					//unsigned char SelectionColor[3];
					//RVLSET3VECTOR(SelectionColor, 0, 255, 0);
					//surfels.NodeColors(SelectionColor);
					//surfels.InitDisplay(&visualizer, &mesh, pDetector);
					//surfels.Display(&visualizer, &mesh, -1, SelectionColor);
					//visualizer.Run();
				}
				else
				{
					printf("\n >> ERROR: Cannot load PLY file!\n");
					return false;
				}

				//Assign pointer to label map
				pLabelMap = surfels.surfelMap;
				maxNoLabels = surfels.NodeArray.n + 1;

				
			}	// if ((*pFlags & RVLPCGT_DEMO_FLAG_USER_SURFEL) != 0)
			else
			{
				printf(" >> Loading PCD file...");
				if (pcl::io::loadPCDFile<PointT>(pCurrentFileName, *cloud))
				{
					printf("\n >> ERROR: Cannot load PCD file!\n");
					return false;
				}
				printf("completed.\n");

				// Generate supervoxels
				pcl::SupervoxelClustering<PointT> super(pPCLSuperVoxelParams->voxel_resolution, pPCLSuperVoxelParams->seed_resolution);
				if (pPCLSuperVoxelParams->disable_transform)
					super.setUseSingleCameraTransform(false);
				super.setInputCloud(cloud);
				super.setColorImportance(pPCLSuperVoxelParams->color_importance);
				super.setSpatialImportance(pPCLSuperVoxelParams->spatial_importance);
				super.setNormalImportance(pPCLSuperVoxelParams->normal_importance);
				std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
				printf(" >> Extracting supervoxels...");
				super.extract(supervoxel_clusters);
				printf("completed. Found %d supervoxels.\n", supervoxel_clusters.size());

				//Get labelled point cloud
				labeled_cloud = super.getLabeledCloud();

				//Copy 
				for (int v = 0; v < GT_IMHEIGHT; v++)
				{
					for (int u = 0; u < GT_IMWIDTH; u++)
					{
						currIdx = v*GT_IMWIDTH + u;

						//Get current label of pixel
						currLabel = labeled_cloud->points[currIdx].label;

						//Store LabelMap
						pLabelMapHelper[currIdx] = currLabel;
					}
				}

				//Assign pointer to label map
				pLabelMap = pLabelMapHelper;
				maxNoLabels = supervoxel_clusters.size() + 1;
			}	// if ((*pFlags & RVLPCGT_DEMO_FLAG_USER_SURFEL) == 0)

			
			//Check if ground truth exists
			if (ReadGTMask(pGTSegmentParams->iImageNo, pGTMask1, pGTSegmentParams->pDefaultFileLocation))
			{
				CurrentImageDetails.pGTMask = pGTMask1;
				CurrentImageDetails.pClickedPoints = pClickedPoints;
				CurrentImageDetails.pLabelMap = pLabelMap;
				CurrentImageDetails.imageNumber = pGTSegmentParams->iImageNo;
				CurrentImageDetails.maxLabelNo = maxNoLabels;

				DisplaySegmentedImage(&CurrentImageDetails);
				printf(" >> Existing ground truth loaded!\n");
			}
			else //ground truth does not exist
			{
				//Initial image - create labels 0 and -1 labels for initial image
				if (pGTSegmentParams->iImageNo == 0)
				{
					//Read Depth Images
					ReadDepthImage(pGTSegmentParams->iImageNo, pDepth0, pGTSegmentParams->pDefaultFileLocation);

					for (int i = 0; i < GT_IMWIDTH * GT_IMHEIGHT; i++)
					{
						//Eliminate pts further than MaxDist
						if (pDepth0[i] >= pGTSegmentParams->MaxDist || pDepth0[i] == 0)
						{
							pGTMask1[i] = -1;
						}
					}


				}
				else   //Initial image - create single labels
				{
					//Read previous GT
					ReadGTMask(pGTSegmentParams->iImageNo - 1, pGTMask0, pGTSegmentParams->pDefaultFileLocation);

					//Read Depth Images
					ReadDepthImage(pGTSegmentParams->iImageNo - 1, pDepth0, pGTSegmentParams->pDefaultFileLocation);
					ReadDepthImage(pGTSegmentParams->iImageNo, pDepth1, pGTSegmentParams->pDefaultFileLocation);


					//Prepare depth images
					for (int i = 0; i < GT_IMWIDTH * GT_IMHEIGHT; i++)
					{
						//Copy GT
						if (pGTMask0[i]>-1)
						{
							pGTMask1[i] = pGTMask0[i];
						}


						//Eliminate pts further than MaxDist
						if (pDepth0[i] >= pGTSegmentParams->MaxDist)
						{
							pDepth0[i] = 0;
						}

						if (pDepth1[i] >= pGTSegmentParams->MaxDist)
						{
							pDepth1[i] = 0;
							pGTMask1[i] = -1;
						}


						//Make sure both depth have zeros at the same coordinates
						if (pDepth0[i] == 0)
						{
							pDepth1[i] = 0;
							pGTMask1[i] = -1;
						}
						if (pDepth1[i] == 0)
						{
							pDepth0[i] = 0;
							pGTMask1[i] = -1;
						}
					}

					//Get diff between depth maps and determine new object(s)
					for (int v = 0; v < GT_IMHEIGHT; v++)
					{
						for (int u = 0; u < GT_IMWIDTH; u++)
						{
							currIdx = v*GT_IMWIDTH + u;

							//Get depth difference
							depthDiff = pDepth1[currIdx] - pDepth0[currIdx];
							pDepthDiffMat[currIdx] = depthDiff;

							//Get current label of pixel
							currLabel = pLabelMap[currIdx];
							
							//if (currLabel<0 || currLabel>maxNoLabels)
							//{
							//	printf("ERROR!! Max no of labels exceeded. Label value: %d\n", currLabel);
							//}
							

							if (currLabel>-1 && currLabel < maxNoLabels)
							{
								//Get total no of pixels per label
								pLabelTotal[currLabel] += 1;

								//Get total no of pixels per label < threshold
								if (depthDiff <= pGTSegmentParams->DiffThreshold)
								{
									pLabelCount[currLabel] += 1;
								}

								//Get perc pixels per label
								pLabelPerc[currLabel] = (pLabelCount[currLabel] * 1.0) / (pLabelTotal[currLabel] * 1.0);
							}
							
						}
					}


					//Generate initial ground truth depth
					for (int v = 0; v < GT_IMHEIGHT; v++)
					{
						for (int u = 0; u < GT_IMWIDTH; u++)
						{
							currIdx = v*GT_IMWIDTH + u;

							//Get current label of pixel
							currLabel = pLabelMap[currIdx];
							//
							if (currLabel>0 && pLabelTotal[currLabel] >= pGTSegmentParams->MinNoOfPoints && pLabelPerc[currLabel] >= pGTSegmentParams->PercThreshold)
							{
								pGTMask1[currIdx] = pGTSegmentParams->iImageNo;
							}
						}
					}




				}

				CurrentImageDetails.pGTMask = pGTMask1;
				CurrentImageDetails.pClickedPoints = pClickedPoints;
				CurrentImageDetails.pLabelMap = pLabelMap;
				CurrentImageDetails.imageNumber = pGTSegmentParams->iImageNo;
				CurrentImageDetails.maxLabelNo = maxNoLabels;


				DisplaySegmentedImage(&CurrentImageDetails);
				printf(" >> Initial ground truth generated.\n");


			} //End Ground truth does not exist
			CurrentImageDetails.pixelmode = &pixelmode;
			
			// Set up the callback
			cvSetMouseCallback(windowNameRGB, my_mouse_callback_ClickPoint, (void *)&CurrentImageDetails);

			delete[] pCurrentDepthFileName;

		} // if (bLoadPtCloud)

#pragma endregion


#pragma region Save GT mask
		//////////////////////////////  //////////////////////////////
		//////SAVE CURRENT GT/mask image
		//////////////////////////////  //////////////////////////////
		if (c == 'S' || c == 's') //|| bSaveGT == true) //S or s -> save current GT/mask image
		{

			pCurrentMaskFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, "_GT.txt");
			SaveGTMask(pGTMask1, GT_IMWIDTH, GT_IMHEIGHT, pCurrentMaskFileName);
			printf(" >> Ground truth saved.\n");
		}

#pragma endregion

#pragma region Zoom, connected component and mesh
		// COENE , connected component

		if (c == 'C' || c == 'c') {
			flood(CurrentImageDetails.pGTMask, pGTSegmentParams);
			DisplaySegmentedImage(&CurrentImageDetails);
		}
		/////

		// Coene , Zoom ROI
		if (c == 'Z' || c == 'z') {
			zoomROI(CurrentImageDetails, image_roi, clickedpoints_ROI);
		}
		/////

		// COENE, generate mesh window when pressing P
		if (c == 'p' || c == 'P'){
			vtkSmartPointer<vtkRenderWindow> window = vtk_initialiser(mesh, &CurrentImageDetails);
		}
		if (c == 'o' || c == 'O'){
			pixelmode = !pixelmode;
		}

		if (c == 'r' || c == 'R')
			DisplaySegmentedImage(&CurrentImageDetails);
		/////
#pragma endregion

#pragma region Get next image
		//////////////////////////////  //////////////////////////////
		//////GET NEXT IMAGE
		//////////////////////////////  //////////////////////////////
		if (c == 'N' || c == 'n') //N or n -> get next image
		{
			//Save once again in case the user forgot to save
			pCurrentMaskFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, "_GT.txt");
			SaveGTMask(pGTMask1, GT_IMWIDTH, GT_IMHEIGHT, pCurrentMaskFileName);
			printf(" >> Ground truth saved.\n");


			//Reset values
			bLoadPtCloud = true;

			cvSetMouseCallback(windowNameRGB, NULL);  //Remove Callback

			//Reset mat values
			memset(pLabelCount, 0, (GT_MAXLABELVAL + 1) * sizeof(int));
			memset(pLabelTotal, 0, (GT_MAXLABELVAL + 1) * sizeof(int));
			memset(pLabelPerc, 0, (GT_MAXLABELVAL + 1) * sizeof(float));
			memset(pLabelMapHelper, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
			memset(pDepth0, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
			memset(pDepth1, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
			memset(pDepthDiffMat, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
			memset(pGTMask0, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
			memset(pGTMask1, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
			memset(pClickedPoints, -1, 4 * sizeof(int));
			
			
			//Increase file number 
			pGTSegmentParams->iImageNo += 1;

			//Get new file
			if ((*pFlags & RVLPCGT_DEMO_FLAG_USER_SURFEL) != 0)
			{
				pCurrentFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, ".ply");
			}
			else
			{
				pCurrentFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, ".pcd");
				//Clear cloud and labeled cloud
				cloud->clear();
				labeled_cloud->clear();
			}

			bLoadPtCloud = true; //Start all over
		}
#pragma endregion


		//////////////////////////////  //////////////////////////////
		//////EXIT APP
		//////////////////////////////  //////////////////////////////
		if (c == 27) //Esc -> exit app
		{
			pCurrentMaskFileName = CreateFileName(pGTSegmentParams->iImageNo, pGTSegmentParams->pDefaultFileLocation, "_GT.txt");
			SaveGTMask(pGTMask1, GT_IMWIDTH, GT_IMHEIGHT, pCurrentMaskFileName);
			cvSetMouseCallback(windowNameRGB, NULL);  //Remove Callback
			break;
		}

	}






	//Exit elegantly
	delete[] pLabelCount;
	delete[] pLabelPerc;
	delete[] pLabelTotal;
	delete[] pLabelMapHelper;
	delete[] pDepth0;
	delete[] pDepth1;
	delete[] pDepthDiffMat;
	delete[] pGTMask0;
	delete[] pGTMask1;
	delete[] pClickedPoints;
	delete[] pSegmentColor;

	// Coene, delete mem allocation of ROI
	if (image_roi != NULL){
		delete image_roi;
	}
	////////

	cvDestroyAllWindows();

	return true;

}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	CRVLMem mem;	// cycle memory

	mem.Create(100000000);

	// Read parameters from a configuration file.
	RVLGT_SEGMENTATION_PARAMS gtSegmentParams;
	gtSegmentParams.pDefaultFileLocation = NULL;

	RVLGT_PCLSUPERVOXEL_PARAMS gtSuperVoxelParams;

	DWORD flags = 0x00000000;
	DWORD mode;
	char *sequenceFileName = NULL;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &gtSegmentParams, &gtSuperVoxelParams, flags, mode, &sequenceFileName);

	ParamList.LoadParams("RVLPCGroundTruthDemo.cfg");

	gtSuperVoxelParams.disable_transform = ((flags & RVLPCGT_DEMO_FLAG_DISABLE_TRANSFORM) != 0);


	PCLMeshBuilder meshBuilder;
	meshBuilder.CreateParamList(&mem0);
	meshBuilder.ParamList.LoadParams("RVLPCGroundTruthDemo.cfg");

	PlanarSurfelDetector detector;
	detector.CreateParamList(&mem0);
	detector.ParamList.LoadParams("RVLPCGroundTruthDemo.cfg");


	//GENERATE GT
	if (mode == RVLPCGT_MODE_FLAG_GENERATE_GT)
	{
		
		char pSegmentLookupFile[17] = "SegmentColor.txt";

		//if ((flags & RVLPCGT_DEMO_FLAG_USER_SURFEL) != 0)
		//{
		//	//Use surfel
		//	


		//	GenerateGT(&gtSegmentParams, pSegmentLookupFile, &meshBuilder, &detector, &mem, &flags);
		//}
		//else
		//{
		//	//Use Supervoxel
		//	GenerateGT(&gtSegmentParams, &gtSuperVoxelParams, pSegmentLookupFile);
		//}
		
		GenerateGT(&gtSuperVoxelParams,  &gtSegmentParams, pSegmentLookupFile, &meshBuilder, &detector, &mem, &flags);
		
	}
	else if (mode == RVLPCGT_MODE_FLAG_GT_OBJECT_SELECTION)
	{
		char displayImageName[] = "Ground Truth";
		char RGBImageName[] = "RGB Image";

		cv::Mat displayImage(480, 640, CV_8UC3, cv::Scalar::all(0));

		char meshFileName[200];
		char *GTFileName;
		char *RGBFileName;
		cv::Mat GTLabImg;
		FileSequenceLoader sceneSequence;

		sceneSequence.Init(sequenceFileName);

		while (sceneSequence.GetNextPath(meshFileName))
		{
			printf("Image %s...\n", meshFileName);

			PCGT::DisplayGroundTruthSegmentation(meshFileName, GTLabImg, true);

			cv::waitKey();

			delete[] GTFileName;
		}
	}
	else if (mode == RVLPCGT_MODE_FLAG_RECORD)
	{
		RecordData(&gtSegmentParams, &meshBuilder, &flags);
	}




	

	return 0;
}

void RVLflood(int* image, RVLGT_SEGMENTATION_PARAMS *pGTSegmentParams){
	int seq = 1;
	//int labelmap[GT_IMWIDTH * GT_IMHEIGHT] = { 0 };
	int *labelmap = new int[GT_IMWIDTH * GT_IMHEIGHT];
	memset(labelmap, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
	queue<int> region;
	queue<int> floodregion;
	int *pGTMask3 = new int[GT_IMWIDTH*GT_IMHEIGHT];
	ReadGTMask(pGTSegmentParams->iImageNo - 1, pGTMask3, pGTSegmentParams->pDefaultFileLocation);
	for (int i = 0; i < GT_IMWIDTH * GT_IMHEIGHT; i++){
		if (labelmap[i] == 0 && pGTSegmentParams->iImageNo == image[i]){
			labelmap[i] = seq;
			floodregion.push(i);
			while (!floodregion.empty()){
				int coord = floodregion.front();
				floodregion.pop();
				int right = (coord + 1) % (GT_IMHEIGHT*GT_IMWIDTH);
				int left = (coord - 1) % (GT_IMHEIGHT*GT_IMWIDTH);
				int above = (coord - GT_IMWIDTH) % (GT_IMHEIGHT*GT_IMWIDTH);
				int under = (coord + GT_IMWIDTH) % (GT_IMHEIGHT*GT_IMWIDTH);
				int direction = right;
				if (direction>0 && labelmap[direction] == 0 && image[direction] == image[coord]){
					labelmap[direction] = seq;
					floodregion.push(direction);
				}
				direction = left;
				if (direction>0 && labelmap[direction] == 0 && image[direction] == image[coord]){
					labelmap[direction] = seq;
					floodregion.push(direction);
				}
				direction = above;
				if (direction>0 && labelmap[direction] == 0 && image[direction] == image[coord]){
					labelmap[direction] = seq;
					floodregion.push(direction);
				}
				direction = under;
				if (direction>0 && labelmap[direction] == 0 && image[direction] == image[coord]){
					labelmap[direction] = seq;
					floodregion.push(direction);
				}
				region.push(coord);
			}
			if (region.size() < pGTSegmentParams->minConnectedComponentSize && (pGTSegmentParams->iImageNo - 1) >= 0){
				int c;
				while (!region.empty()){
					c = region.front();
					region.pop();
					image[c] = pGTMask3[c];
				}
			}
			std::queue<int> empty;
			std::swap(region, empty);
			std::queue<int> empty2;
			std::swap(floodregion, empty2);
			seq++;
		}
	}

	delete[] labelmap;
	delete[] pGTMask3;
}

///////////////////////////////// 
////FROM HERE COENE ADDED FUNCTIONS
///////////////////////////////

// COENE, Connected component function
void flood(int* image, RVLGT_SEGMENTATION_PARAMS *pGTSegmentParams){
	int seq = 1;
	int *labelmap = new int[GT_IMWIDTH * GT_IMHEIGHT];
	memset(labelmap, 0, GT_IMWIDTH * GT_IMHEIGHT * sizeof(int));
	queue<int> region;
	queue<int> floodregion;
	int *pGTMask3 = new int[GT_IMWIDTH*GT_IMHEIGHT];
	ReadGTMask(pGTSegmentParams->iImageNo - 1, pGTMask3, pGTSegmentParams->pDefaultFileLocation);
	for (int i = 0; i < GT_IMWIDTH * GT_IMHEIGHT; i++){
		if (labelmap[i] == 0 && pGTSegmentParams->iImageNo == image[i]){
			labelmap[i] = seq;
			floodregion.push(i);
			while (!floodregion.empty()){
				int coord = floodregion.front();
				floodregion.pop();
				int right = (coord + 1) % (GT_IMHEIGHT*GT_IMWIDTH);
				int left = (coord - 1) % (GT_IMHEIGHT*GT_IMWIDTH);
				int above = (coord - GT_IMWIDTH) % (GT_IMHEIGHT*GT_IMWIDTH);
				int under = (coord + GT_IMWIDTH) % (GT_IMHEIGHT*GT_IMWIDTH);
				int direction = right;
				if (direction>0 && labelmap[direction] == 0 && image[direction] == image[coord]){
					labelmap[direction] = seq;
					floodregion.push(direction);
				}
				direction = left;
				if (direction>0 && labelmap[direction] == 0 && image[direction] == image[coord]){
					labelmap[direction] = seq;
					floodregion.push(direction);
				}
				direction = above;
				if (direction>0 && labelmap[direction] == 0 && image[direction] == image[coord]){
					labelmap[direction] = seq;
					floodregion.push(direction);
				}
				direction = under;
				if (direction>0 && labelmap[direction] == 0 && image[direction] == image[coord]){
					labelmap[direction] = seq;
					floodregion.push(direction);
				}
				region.push(coord);
			}
			if (region.size() < pGTSegmentParams->minConnectedComponentSize && (pGTSegmentParams->iImageNo - 1) >= 0){
				int c;
				while (!region.empty()){
					c = region.front();
					region.pop();
					image[c] = pGTMask3[c];
				}
			}
			std::queue<int> empty;
			std::swap(region, empty);
			std::queue<int> empty2;
			std::swap(floodregion, empty2);
			seq++;
		}
	}

	delete[] labelmap;
	delete[] pGTMask3;
}

////////


// COENE initialise VTK window function
vtkSmartPointer<vtkRenderWindow> vtk_initialiser(Mesh &mesh, RVLGT_IMAGE_DETAILS *pImgDetails){
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> window;
	vtkSmartPointer<vtkRenderWindowInteractor> interactor;
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style;
	vtkSmartPointer<vtkPolyDataMapper> map;
	vtkSmartPointer<vtkActor> actor;
	vtkSmartPointer<vtkPointPicker> pointPicker;

	//initialize vtk
	renderer = vtkSmartPointer<vtkRenderer>::New();
	window = vtkSmartPointer<vtkRenderWindow>::New();
	interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	// initialise the rest
	pointPicker = vtkSmartPointer<vtkPointPicker>::New();
	interactor->SetPicker(pointPicker);
	map = vtkSmartPointer<vtkPolyDataMapper>::New();
	map->SetInputData(mesh.pPolygonData);		// outside
	map->InterpolateScalarsBeforeMappingOff();
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(map);
	renderer->AddActor(actor);

	//RightMouseButton callback
	vtkSmartPointer<vtkCallbackCommand> mouseRButtonDownCallback = vtkSmartPointer<vtkCallbackCommand>::New();
	mouseRButtonDownCallback->SetCallback(userFunction);
	Group_Structure_VTK group;
	group.details = pImgDetails;
	group.mesh = &mesh;
	group.window = &window;
	mouseRButtonDownCallback->SetClientData(&group);
	interactor->AddObserver(vtkCommand::RightButtonPressEvent, mouseRButtonDownCallback);

	// color properly
	DisplaySegmentationInMesh(window, mesh, pImgDetails);
	// render image
	renderer->ResetCamera();
	window->Render();
	window->GetInteractor()->Start();
	return window;
}
////////////


// COENE function that displays the labels on the mesh
void DisplaySegmentationInMesh(vtkSmartPointer<vtkRenderWindow> window, Mesh& mesh, RVLGT_IMAGE_DETAILS *pImgDetails)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;
	rgbPointData = rgbPointData->SafeDownCast(mesh.pPolygonData->GetPointData()->GetArray("RGB"));
	int* psegColor;
	int iseg;
	int iLabel;
	unsigned char Color[3];
	for (int i = 0; i < GT_IMWIDTH *GT_IMHEIGHT; i++){
		iLabel = pImgDetails->pGTMask[i];

		if (iLabel > -1)
		{
			iseg = iLabel % 37;
		}
		else
		{
			iseg = 37;
		}
		psegColor = pImgDetails->segmentColor + iseg * 3;
		Color[0] = psegColor[2];
		Color[1] = psegColor[1];
		Color[2] = psegColor[1];
		rgbPointData->SetTupleValue(i, Color);
	}
	mesh.pPolygonData->Modified();
	window->Modified();
	window->GetInteractor()->GetRenderWindow()->Render();
}
/////////

// Coene callback function that invokes the switching of labels on the mesh and on the 2D image,
void userFunction(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor*>(caller);
	Group_Structure_VTK* group = (Group_Structure_VTK*)clientdata;
	interactor->GetPicker()->Pick(interactor->GetEventPosition()[0], interactor->GetEventPosition()[1], 0,
		interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	double p[4];
	int id = ((vtkPointPicker*)interactor->GetPicker())->GetPointId();
	int x = id % GT_IMWIDTH;
	my_mouse_callback_ClickPoint(CV_EVENT_LBUTTONUP, x, (id - x) / GT_IMWIDTH, 0, group->details);
	DisplaySegmentationInMesh(*(group->window), *(group->mesh), group->details);
}

void zoomROI(RVLGT_IMAGE_DETAILS& CurrentImageDetails, Mat* image_roi, int* clickedpoints){
	if (CurrentImageDetails.pClickedPoints[2] > -1 && CurrentImageDetails.pClickedPoints[3] > -1){
		const int imw = 51;
		const int imh = 51;
		int halfimw = (imw - 1) / 2;
		int halfimh = (imh - 1) / 2;
		int beginx = CurrentImageDetails.pClickedPoints[2] - halfimw;
		int beginy = CurrentImageDetails.pClickedPoints[3] - halfimh;

		if (beginx < 0)
			beginx = 0;
		if (beginy < 0)
			beginy = 0;
		if ((beginx + imw)>GT_IMWIDTH){
			beginx = GT_IMWIDTH - imw;
		}
		if ((beginy + imh)>GT_IMHEIGHT){
			beginy = GT_IMHEIGHT - imh;
		}

		RVLGT_IMAGE_DETAILS* ROI_Details = new RVLGT_IMAGE_DETAILS();

		cv::Rect region_of_interest = cv::Rect(beginx, beginy, imw, imh);
		Mat image_roi2 = (*CurrentImageDetails.pRGBImage)(region_of_interest);
		// Coene, delete mem allocation of ROI, if it already existed
		if (image_roi != NULL){
			delete image_roi;
		}
		image_roi = new Mat(image_roi2);
		ROI_Details->pRGBImage = image_roi;

		ROI_Details->segmentColor = CurrentImageDetails.segmentColor;
		ROI_Details->pGTMask = CurrentImageDetails.pGTMask;
		ROI_Details->pWindowTitle = "ROI";
		ROI_Details->pClickedPoints = clickedpoints;
		ROI_Details->pLabelMap = CurrentImageDetails.pLabelMap;
		ROI_Details->maxLabelNo = CurrentImageDetails.maxLabelNo;
		ROI_Details->imageNumber = CurrentImageDetails.imageNumber;
		ROI_Details->pixelmode = CurrentImageDetails.pixelmode;

		DisplaySegmentedImage(ROI_Details, beginx, beginy, true);

		RVLGT_IMAGE_DETAILS_ROI* ROI_Details_wrapped = new RVLGT_IMAGE_DETAILS_ROI();
		ROI_Details_wrapped->details = ROI_Details;
		ROI_Details_wrapped->startx = beginx;
		ROI_Details_wrapped->starty = beginy;
		ROI_Details_wrapped->width = imw;
		ROI_Details_wrapped->height = imh;
		cvSetMouseCallback(ROI_Details->pWindowTitle, my_mouse_callback_ClickPoint_ROI, (void *)ROI_Details_wrapped);
	}
}


// COENE, new callback function for ROI
void my_mouse_callback_ClickPoint_ROI(int event, int x, int y, int flags, void* param)
{
	RVLGT_IMAGE_DETAILS_ROI * pCurrentImageDetails2 = (RVLGT_IMAGE_DETAILS_ROI *)param;
	RVLGT_IMAGE_DETAILS* pCurrentImageDetails = pCurrentImageDetails2->details;
	int startw = pCurrentImageDetails2->startx;
	int starth = pCurrentImageDetails2->starty;
	int width = pCurrentImageDetails2->width;
	int height = pCurrentImageDetails2->height;

	switch (event)
	{

	case CV_EVENT_LBUTTONUP:
		pCurrentImageDetails->pClickedPoints[0] = x;
		pCurrentImageDetails->pClickedPoints[1] = y;
		SwitchLabels(pCurrentImageDetails, startw, starth, width, height);
		break;

	case CV_EVENT_RBUTTONUP:
		pCurrentImageDetails->pClickedPoints[2] = x;
		pCurrentImageDetails->pClickedPoints[3] = y;
		break;

	}
}
////////////