// RVLWork.cpp : Defines the entry point for the console application.
//

#include "opencv\highgui.h"
#include <stdio.h>
#include <time.h>
#include "RVLCore.h"
//#include "RVLPCS.h"
#include "RVLTMLeaf.h"
#include "RVLCropRowDetector.h"

// R = [cs, 0, sn;		move to RVLUtil.h!
//		0,  1, 0;
//	   -sn, 0, cs]
#define RVLROTY(cs, sn, R)\
{\
	RVLMXEL(R, 3, 0, 0) = cs;\
	RVLMXEL(R, 3, 0, 1) = 0.0;\
	RVLMXEL(R, 3, 0, 2) = sn;\
	RVLMXEL(R, 3, 1, 0) = 0.0;\
	RVLMXEL(R, 3, 1, 1) = 1.0;\
	RVLMXEL(R, 3, 1, 2) = 0.0;\
	RVLMXEL(R, 3, 2, 0) = -sn;\
	RVLMXEL(R, 3, 2, 1) = 0.0;\
	RVLMXEL(R, 3, 2, 2) = cs;\
}
//#define RVLTEST_TM
#define RVLTEST_CRD
#define RVL_LOAD_IMAGE
//#define RVL_CREATE_TEST_IMAGE

int main(int argc, char* argv[])
{
	// create vision system

	//CRVLPCSVS VS;
	CRVLVisionSystem VS;

	// initialize vision system

	char ConfigFileName[] = "RVLWork.cfg";

	VS.CreateParamList();

	VS.Init(ConfigFileName);

	int w = VS.m_AImage.m_Width;
	int h = VS.m_AImage.m_Height;

	// create GUI

	CRVLGUI GUI;

	GUI.m_pMem0 = &(VS.m_Mem0);
	GUI.m_pMem = &(VS.m_Mem);

	GUI.Init();

	int key;

	// create display image

	IplImage *pDisplay = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	unsigned char *DisplayPixArray = (unsigned char *)(pDisplay->imageData);

#ifdef RVLTEST_CRD
	// Create Crop Row Detector Instance

	CRVLCropRowDetector CRD;

	CRD.CreateParamList(&(VS.m_Mem0));

	CRD.m_ParamList.LoadParams(ConfigFileName);

	CRD.Init(h);

	// CRD parameters

	double C = 0.0;
	double D = 300.0;
	double f = 300.0;
	double beta = -30.0 * DEG2RAD;
	double z = 1500.0;
	int n = 31;
	double L = 500000.0;
	double s = 100.0;
	double std = 0.1 * s;
	double noiseDensity = 1.0 / (100.0 * 100.0);

	// camera matrix

	CRVL3DPose PoseCW;

	PoseCW.m_Alpha = 0.0;
	PoseCW.m_Beta = beta;
	PoseCW.m_Theta = 0.0;
	PoseCW.m_X[0] = 0.0;
	PoseCW.m_X[1] = 0.0;
	PoseCW.m_X[2] = z;
	PoseCW.UpdateRotLA();
	
	double *R = PoseCW.m_Rot;
	double *t = PoseCW.m_X;

	double P[3 * 3];
	RVLMXEL(P, 3, 0, 0) = f;
	RVLMXEL(P, 3, 0, 1) = 0.0;
	RVLMXEL(P, 3, 0, 2) = 0.5 * (double)w;
	RVLMXEL(P, 3, 1, 0) = 0.0;
	RVLMXEL(P, 3, 1, 1) = f;
	RVLMXEL(P, 3, 1, 2) = 0.5 * (double)h;
	RVLMXEL(P, 3, 2, 0) = 0.0;
	RVLMXEL(P, 3, 2, 1) = 0.0;
	RVLMXEL(P, 3, 2, 2) = 1.0;

#ifdef RVL_CREATE_TEST_IMAGE
	// create an image of crop rows

	cv::Mat InputImage(h, w, CV_32F);

	//IplImage *pInputImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);

	InputImage.setTo(cv::Scalar(0.0f));

	float *I = InputImage.ptr<float>();

	int imax = DOUBLE2INT(L / s);

	double X[3];
	double U[3];

	X[2] = 0.0;

	int i, j;
	int u, v;
	double r;
	double XC[3];
	double V3Tmp[3];

	for(i = 0; i <= imax; i++)
		for(j = -(n - 1)/2; j <= (n - 1)/2; j++)
		{
			X[0] = (double)i * s + RVLGaussRand(std);
			X[1] = (double)j * D + C + RVLGaussRand(std);

			RVLDIF3VECTORS(X, t, V3Tmp)
			RVLMULMX3X3TVECT(R, V3Tmp, XC)
			RVLMULMX3X3VECT(P, XC, U)

			u = DOUBLE2INT(U[0] / U[2]);
			v = DOUBLE2INT(U[1] / U[2]);

			if(u >= 0 && u < w && v >= 0 && v < h)	
			{
				r = sqrt(RVLDOTPRODUCT3(XC, XC));

				RVLMXEL(I, w, v, u) += (1000.0f / (float)r);
			}
		}

	int nNoise = DOUBLE2INT((double)(n - 1) * D * L * noiseDensity);

	for(i = 0; i < nNoise; i++)	
	{
		X[0] = RVLRandom(0.0, L);
		X[1] = RVLRandom(-(double)((n - 1)/2)*D, (double)((n - 1)/2)*D);

		RVLDIF3VECTORS(X, t, V3Tmp)
		RVLMULMX3X3TVECT(R, V3Tmp, XC)
		RVLMULMX3X3VECT(P, XC, U)

		u = DOUBLE2INT(U[0] / U[2]);
		v = DOUBLE2INT(U[1] / U[2]);

		if(u >= 0 && u < w && v >= 0 && v < h)	
		{
			r = sqrt(RVLDOTPRODUCT3(XC, XC));

			RVLMXEL(I, w, v, u) += (1000.0f / (float)r);
		}
	}

	cv::Mat GaussImage(h, w, CV_32F);

	cv::GaussianBlur(InputImage, GaussImage, cvSize(21, 21), 3.0, 3.0);

	//GaussImage = InputImage;

	//GaussImage.setTo(cv::Scalar(204));

	//cvCvtColor(pInputImage, pDisplay, CV_GRAY2BGR);

	double min, max;
	cv::minMaxLoc(InputImage, &min, &max);

	cv::Mat DisplayGS;

	InputImage.convertTo(DisplayGS, CV_8U, 255.0 / max);
	IplImage *pDisplayGS = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 1);
	cvSetImageData(pDisplayGS, DisplayGS.ptr(), w);
	cvShowImage("Crop Rows", pDisplayGS);
	cvReleaseImageHeader(&pDisplayGS);
	DisplayGS.release();

	cv::Mat FilteredImage;

	cv::minMaxLoc(GaussImage, &min, &max);
	GaussImage.convertTo(FilteredImage, CV_8U, 255.0 / max);

	CRD.Apply(FilteredImage.ptr(), w);

	pDisplayGS = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 1);
	cvSetImageData(pDisplayGS, FilteredImage.ptr(), w);
	cvCvtColor(pDisplayGS, pDisplay, CV_GRAY2RGB);

	CRD.Display((unsigned char *)(pDisplay->imageData), w);

	cvShowImage("Crop Rows Blured", pDisplay);
	cvSaveImage("debug.bmp", pDisplay);
	cvReleaseImageHeader(&pDisplayGS);
	DisplayGS.release();
	FilteredImage.release();	

	//cv::imshow("Crop Rows", InputImage);

	//cv::namedWindow( "Gauss");
	//cv::imshow("Gauss", GaussImage);

#endif //RVL_CREATE_TEST_IMAGE

#ifdef RVL_LOAD_IMAGE

	IplImage* pInputImage = cvLoadImage(VS.m_ImageFileName, CV_LOAD_IMAGE_UNCHANGED);
	IplImage* ExGImage=cvCreateImage(cvSize(w,h),8,1);
	IplImage* threshImage=cvCreateImage(cvSize(w,h),8,1);

	int experimentNum = 12;
	char naziv[40];

	cvShowImage("Crop Rows BGR", pInputImage);
	sprintf(naziv, "crop_row_BGR_%d.bmp", experimentNum);
	cvSaveImage(naziv, pInputImage);
	//cvSaveImage("crop_row_BGR.bmp", pInputImage);

	//Calculate ExG image and apply double Otsu threshold (see: Montalvo et all: Automatic detection of crop rows in maize fields with high weeds pressure)
	CRD.ExGImage((unsigned char *)pInputImage->imageData, (unsigned char *) ExGImage->imageData, pInputImage->width, pInputImage->height);
	CRD.DoubleOtsu(ExGImage, threshImage);

	cv::Mat GaussImage(h, w, CV_32F);

	cv::GaussianBlur(threshImage, GaussImage, cvSize(21, 21), 3.0, 3.0);

	double min, max;

	cv::Mat FilteredImage;

	cv::minMaxLoc(GaussImage, &min, &max);
	GaussImage.convertTo(FilteredImage, CV_8U, 255.0 / max);

	//pDisplayGS = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 1);
	//cvSetImageData(pDisplayGS, FilteredImage.ptr(), w);
	//cvShowImage("Filtered Otsu Image", pDisplayGS);

	//CRD.TripleOtsu(ExGImage, threshImage);

	cvShowImage("ExG image", ExGImage);
	sprintf(naziv, "crop_row_ExG_%d.bmp", experimentNum);
	cvSaveImage(naziv, ExGImage);
	//cvSaveImage("cropRows_ExG_1.bmp", ExGImage);

	cvShowImage("Double Otsu", threshImage);
	sprintf(naziv, "crop_row_doubleOtsu_original_%d.bmp", experimentNum);
	cvSaveImage(naziv, threshImage);
	//cvSaveImage("cropRows_doubleOtsu_1.bmp", threshImage);


	/*****************************************************/
	/******************HSV THRESHOLD**********************/
	//Apply HSV threshold
	////Experiment 1
	//CvScalar lower_bound = cvScalar(30, 110, 110); 
	//CvScalar upper_bound = cvScalar(60, 130, 130);

	////Experiment 2
	//CvScalar lower_bound = cvScalar(30, 100, 100); 
	//CvScalar upper_bound = cvScalar(60, 150, 150);

	////Experiment 3
	//CvScalar lower_bound = cvScalar(40, 160, 160); 
	//CvScalar upper_bound = cvScalar(60, 170, 170);

	////Experiment 4
	//CvScalar lower_bound = cvScalar(35, 120, 150); 
	//CvScalar upper_bound = cvScalar(45, 165, 204);

	//CRD.HSVThreshold(pInputImage, threshImage, lower_bound, upper_bound);	
	//cvShowImage("Crop Rows Thresholded", threshImage);
	//cvSaveImage("cropRows_thresholded.bmp", threshImage);
	/****************************************************/


	//Apply crop row detection method
	//CRD.Apply((unsigned char *)threshImage->imageData, w);
	CRD.Apply(FilteredImage.ptr(), w);

	cvCvtColor(threshImage, pDisplay, CV_GRAY2RGB);

	CRD.Display((unsigned char *)(pDisplay->imageData), w);

	cvShowImage("Crop Rows Detected", pDisplay);
	sprintf(naziv, "crop_row_detected_DOO_%d.bmp", experimentNum);
	cvSaveImage(naziv, pDisplay);
	//cvSaveImage("cropRows_detected_1.bmp", pDisplay);

	cvReleaseImage(&pInputImage);
	cvReleaseImage(&ExGImage);
	cvReleaseImage(&threshImage);

#endif //RVL_LOAD_IMAGE



	cvWaitKey();

	//cvReleaseImage(&pInputImage);
#endif // RVLTEST_CRD

#ifdef RVLTEST_TM
	// create model

	//CRVLTMLeaf Model_;
	CRVLTM Model_;

	Model_.m_Width = w;
	Model_.m_Height = h;
	Model_.m_pMem = &(VS.m_Mem);

	Model_.CreateParamList(&(VS.m_Mem0));

	Model_.m_ParamList.LoadParams(ConfigFileName);

	//Model_.Create();
	Model_.Load();

	RVLTMR_MODEL_NODE *Model = Model_.m_Node;	

	//// only for debugging purpose!!!

	//iNode = 1;
	//iParam = 1;

	//PDFParams[0] = 0.0;
	//PDFParams[1] = LeafL[4 * iNode + iParam];
	//PDFParams[2] = LeafH[4 * iNode + iParam];
	//minParamVal = 3.0 * PDFParams[1];
	//maxParamVal = 3.0 * PDFParams[2];
	//dVal = 1e-3 * (maxParamVal - minParamVal);

	//int nBins = 101;

	//double BinSize = (maxParamVal - minParamVal) / (double)(nBins - 1);

	//int *Histogram = new int[nBins];

	//memset(Histogram, 0, nBins * sizeof(int));

	//double x;
	//int iBin;

	//for(int i = 0; i < 100000; i++)
	//{
	//	x = RVLRandPDF(RVLBilateralGaussianPDF, PDFParams, minParamVal, maxParamVal, dVal);

	//	iBin = DOUBLE2INT((x - minParamVal) / BinSize);

	//	Histogram[iBin]++;
	//}

	//FILE *fp = fopen("debug.txt", "w");

	//for(iBin = 0; iBin < nBins; iBin++)
	//	fprintf(fp, "%lf\t%d\n", (double)iBin * BinSize + minParamVal, Histogram[iBin]);

	//fclose(fp);

	//delete[] Histogram;

	//return 0;

	//// Create and display a model instance

	//RVLTMR_NODE *Instance = new RVLTMR_NODE[Model_.m_nNodes];

	//do
	//{
	//	Model_.CreateInstance(Instance);

	//	cvSet(pDisplay, cvScalar(255, 255, 255));

	//	Model_.DisplayInstance(pDisplay, Instance, 0.5 * w, 0.5 * h, cvScalar(0, 128, 0), cvScalar(0, 0, 255), false);

	//	cvSaveImage("Sample.bmp", pDisplay);

	//	cvShowImage("Sample", pDisplay);

	//	key = cvWaitKey();
	//}
	//while(key != 27);

	//cvDestroyWindow("Sample");

	//delete[] Instance;

	//return 0;

	// create input image and its GS version

	IplImage *pInputImage = NULL;
	IplImage *pGSImage = NULL;

	// create mem. storage for contour detection

	CvMemStorage *pMemStorage;

	pMemStorage = cvCreateMemStorage(0);

	CvSeq* pContour = NULL;

	// main loop

	CRVL3DPose ParentPose;
	CvPoint *pPt;
	CvPoint *Contour;
	int nPts;
	unsigned char *pDisplayPix;

	do
	{
		// Load input image

		if(pInputImage)
			cvReleaseImage(&pInputImage);

		pInputImage = cvLoadImage(VS.m_ImageFileName);

		if(pInputImage->width != w || pInputImage->height != h)
		{
			char str[200];

			sprintf(str, "Image resolution must be %dx%d", w, h);

			GUI.Message(str, 400, 100, cvScalar(0, 128, 255));

			cvReleaseMemStorage(&pMemStorage);

			cvReleaseImage(&pDisplay);

			cvReleaseImage(&pInputImage);

			return 1;
		}

		// Find the leaf contour

		if(pGSImage)
			cvReleaseImage(&pGSImage);

		pGSImage = cvCreateImage(cvGetSize(pInputImage), IPL_DEPTH_8U, 1);

		cvCvtColor( pInputImage, pGSImage, CV_BGR2GRAY );

		cvThreshold( pGSImage, pGSImage, 250, 255, CV_THRESH_BINARY_INV);

		cvFindContours(pGSImage, pMemStorage, &pContour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		nPts = pContour->total;

		Contour = new CvPoint[nPts];

		for(int i = 0; i < nPts; i++)
		{
			pPt = CV_GET_SEQ_ELEM(CvPoint, pContour, i);

			Contour[i] = *pPt;
		}

		int nNodes;

		RVLTMR_NODE *Node = Model_.Detect(Contour, nPts, nNodes);

		RVLTMR_NODE *pNode = Node;

		// Display best fit

		cvCopyImage(pInputImage, pDisplay);

		double bestScore = 0.0;

		RVLTMR_NODE *pBestRoot;

		while(pNode->m_ID == 0)
		{
//#ifdef RVLTMR_DEBUG
//			if(pNode - Node >= 274 && pNode - Node <= 279)
//			{
//				pNode++;
//
//				continue;
//			}
//#endif

			if(pNode->m_score > bestScore)
			{
				pBestRoot = pNode;
				bestScore = pBestRoot->m_score;
			}

			pNode++;
		}

		RVLDisplayTMInstance(pDisplay, pBestRoot, Model);

		for(int i = 0; i < nNodes; i++)
		{
			pNode = Node + i;

			if(Model[pNode->m_ID].m_Flags & RVLTMR_MODEL_NODE_FLAG_CONTOUR)
			{
				pDisplayPix = DisplayPixArray + 3 * ((int)(pNode->m_PoseNC.m_X[0]) + w * (int)(pNode->m_PoseNC.m_X[1]));

				*(pDisplayPix++) = 255;
				*(pDisplayPix++) = 0;
				*(pDisplayPix++) = 0;
			}
		}

		cvSaveImage("Display.bmp", pDisplay);

		cvShowImage(VS.m_ImageFileName, pDisplay);

		key = cvWaitKey();
	}
	while(key != 27);

	cvReleaseMemStorage(&pMemStorage);

	if(pInputImage)
		cvReleaseImage(&pInputImage);

	if(pGSImage)
		cvReleaseImage(&pGSImage);
#endif	// RVLTEST_TM

	cvReleaseImage(&pDisplay);

	return 0;
}

