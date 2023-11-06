#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
// #include "ReconstructionEval.h"
#include "SurfelGraph.h"
// #include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "Voter.h"
#include "RVLBuffer.h"
#include "RVLPtrChain.h"
#include "RVLMPtrChain.h"
#include "Rect.h"
#include "RVLEDT.h"
#include "DDDetector.h"

using namespace RVL;
// using namespace RECOG;

DDDetector::DDDetector()
{
	surfelMask = NULL;
	models.n = 0;
	models.Element = NULL;
	sampleMask = NULL;
	normalSimilarity = 45.0f;
	beta = 0.05f;
	minSurfelSize = 1000;
	minEdgeSize = 20;
	minRefSurfelSize = 1000;
	nICPIterations = 3;
	bHypGenConvex = false;
	bHypGenFlat = false;
	bLoadMovingPartHypothesesFromFile = false;
	bLoadDDHypothesesFromFile = false;
	pVisualizationData = NULL;
	ROICalculationStep = 1;
	minPointShift = 0.05;
	ROISceneSubsampleCellSize = 4;
	ROISceneSubsampleNormalSimilarity = 45.0;
	ROIPointAssociationGridCellSize = 8;
	votingCellSizeX = 0.01;
	votingCellSizeY = 0.01;
	votingCellSizeZ = 0.01;
	shiftThreshX = 0.01;
	shiftThreshY = 0.01;
	shiftThreshZ = 0.01;
	minPointsWithDominantShift = 100;
	test = RVLDDD_TEST_DDD;
	storageVolumeWallThickness = 0.015f;
	ZBuffer.Element = NULL;
	ZBufferActivePtArray.Element = NULL;
	subImageMap = NULL;
	sceneSamplingResolution = 2;
	chamferDistThr = 0.040f;
	transparencyDepthThr = 0.040f;
	edgeModel = RVLDDD_EDGE_MODEL_CYLINDER;
	fit3DTo2DEDTMaxDist = 60;
	fit3DTo2DMinDefDoFs = 6;
	fit3DTo2DNoEdgeSamples = 100;
	fit3DTo2DLambdaR = 0.005f;
	fit3DTo2DLambdat = 0.005f;
	fit3DTo2DMaxOrientationCorrection = 60.0f;
	fit3DTo2DMaxPositionCorrection = 0.5f;
	fit3DTo2DStereo = false;
	nRANSACIterations = 5000;
	pointAssociationGridCellSize = 8;
	maxCoPlanarSurfelRefPtDist = 0.035f;
	maxCoPlanarSurfelNormalAngle = 30.0f; // deg
	openingDirectionTol = 5.0f;			  // deg
	surfelEdges.Element = NULL;
	surfelEdges_ = NULL;
	planarSurfeceEdges = NULL;
	planarSurfeceEdgeMem = NULL;
	frontFaceThickness = 0.018f;
	maxROIStep = 3;
	RVLSET3VECTOR(rectStructAlignmentCorrectionBounds, 0.2f, 0.2f, 0.2f); // m
	orthogonalViewEdgeLineAngleTol = 10.0f;								  // deg
	orthogonalViewPixSize = 0.005f;										  // m
}

DDDetector::~DDDetector()
{
	Clear();
}

void DDDetector::Create(char *cfgFileNameIn)
{
	cfgFileName = cfgFileNameIn;
	CreateParamList();
	paramList.LoadParams(cfgFileNameIn);
	if (test == RVLDDD_TEST_DDD2)
		bHypGenFlat = true;
}

void DDDetector::Clear()
{
	RVL_DELETE_ARRAY(surfelMask);
	RECOG::DDD::Model *pModel;
	for (int iModel = 0; iModel < models.n; iModel++)
	{
		pModel = models.Element + iModel;
		ClearModel(pModel);
	}
	RVL_DELETE_ARRAY(models.Element);
	RVL_DELETE_ARRAY(sampleMask);
	if (pVisualizationData)
	{
		if (pVisualizationData->bOwnVisualizer)
			delete pVisualizationData->pVisualizer;
	}
	RVL_DELETE_ARRAY(ZBuffer.Element);
	RVL_DELETE_ARRAY(ZBufferActivePtArray.Element);
	RVL_DELETE_ARRAY(subImageMap);
	RVL_DELETE_ARRAY(surfelEdges.Element);
	RVL_DELETE_ARRAY(surfelEdges_);
	RVL_DELETE_ARRAY(planarSurfeceEdges);
	RVL_DELETE_ARRAY(planarSurfeceEdgeMem);
}

void DDDetector::CreateParamList()
{
	paramList.m_pMem = pMem0;
	RVLPARAM_DATA *pParamData;
	paramList.Init();
	pParamData = paramList.AddParam("Recognition.mode", RVLPARAM_TYPE_ID, &mode);
	paramList.AddID(pParamData, "TRAINING", RVLRECOGNITION_MODE_TRAINING);
	paramList.AddID(pParamData, "RECOGNITION", RVLRECOGNITION_MODE_RECOGNITION);
	pParamData = paramList.AddParam("Recognition.test", RVLPARAM_TYPE_ID, &test);
	paramList.AddID(pParamData, "DDD", RVLDDD_TEST_DDD);
	paramList.AddID(pParamData, "CUBOIDS", RVLDDD_TEST_CUBOIDS);
	paramList.AddID(pParamData, "3DTO2DFIT", RVLDDD_TEST_3DTO2DFIT);
	paramList.AddID(pParamData, "DDD2", RVLDDD_TEST_DDD2);
	paramList.AddID(pParamData, "SVD", RVLDDD_TEST_SVD);
	paramList.AddID(pParamData, "DETECTRECTSTRUCT", RVLDDD_TEST_DETECT_RECTSTRUCT);
	paramList.AddID(pParamData, "RECOGRECTSTRUCT", RVLDDD_TEST_RECOGNIZE_RECTSTRUCT);
	paramList.AddID(pParamData, "RECOGAO", RVLDDD_TEST_RECOGNIZE_AO);
	pParamData = paramList.AddParam("DDD.model", RVLPARAM_TYPE_ID, &model);
	paramList.AddID(pParamData, "DOOR", RVLDDD_MODEL_DOOR);
	paramList.AddID(pParamData, "DRAWER", RVLDDD_MODEL_DRAWER);
	pParamData = paramList.AddParam("DDD.alphas", RVLPARAM_TYPE_FLOAT, &alphas);
	pParamData = paramList.AddParam("DDD.alphaR", RVLPARAM_TYPE_FLOAT, &alphaR);
	pParamData = paramList.AddParam("DDD.alphat", RVLPARAM_TYPE_FLOAT, &alphat);
	pParamData = paramList.AddParam("DDD.beta", RVLPARAM_TYPE_FLOAT, &beta);
	pParamData = paramList.AddParam("DDD.minSurfelSize", RVLPARAM_TYPE_INT, &minSurfelSize);
	pParamData = paramList.AddParam("DDD.minEdgeSize", RVLPARAM_TYPE_INT, &minEdgeSize);
	pParamData = paramList.AddParam("DDD.minRefSurfelSize", RVLPARAM_TYPE_INT, &minRefSurfelSize);
	pParamData = paramList.AddParam("DDD.no_ICP_iterations", RVLPARAM_TYPE_INT, &nICPIterations);
	pParamData = paramList.AddParam("DDD.loadMovingPartHypothesesFromFile", RVLPARAM_TYPE_BOOL, &bLoadMovingPartHypothesesFromFile);
	pParamData = paramList.AddParam("DDD.loadDDHypothesesFromFile", RVLPARAM_TYPE_BOOL, &bLoadDDHypothesesFromFile);
	pParamData = paramList.AddParam("DDD.visualize", RVLPARAM_TYPE_BOOL, &bVisualize);
	pParamData = paramList.AddParam("DDD.ROICalculationStep", RVLPARAM_TYPE_INT, &ROICalculationStep);
	pParamData = paramList.AddParam("DDD.minPointShift", RVLPARAM_TYPE_FLOAT, &minPointShift);
	pParamData = paramList.AddParam("DDD.ROISceneSubsampleCellSize", RVLPARAM_TYPE_INT, &ROISceneSubsampleCellSize);
	pParamData = paramList.AddParam("DDD.ROISceneSubsampleNormalSimilarity", RVLPARAM_TYPE_FLOAT, &ROISceneSubsampleNormalSimilarity);
	pParamData = paramList.AddParam("DDD.ROIPointAssociationGridCellSize", RVLPARAM_TYPE_INT, &ROIPointAssociationGridCellSize);
	pParamData = paramList.AddParam("DDD.votingCellSizeX", RVLPARAM_TYPE_FLOAT, &votingCellSizeX);
	pParamData = paramList.AddParam("DDD.votingCellSizeY", RVLPARAM_TYPE_FLOAT, &votingCellSizeY);
	pParamData = paramList.AddParam("DDD.votingCellSizeZ", RVLPARAM_TYPE_FLOAT, &votingCellSizeZ);
	pParamData = paramList.AddParam("DDD.shiftThreshX", RVLPARAM_TYPE_FLOAT, &shiftThreshX);
	pParamData = paramList.AddParam("DDD.shiftThreshY", RVLPARAM_TYPE_FLOAT, &shiftThreshY);
	pParamData = paramList.AddParam("DDD.shiftThreshZ", RVLPARAM_TYPE_FLOAT, &shiftThreshZ);
	pParamData = paramList.AddParam("DDD.minPointsWithDominantShift", RVLPARAM_TYPE_INT, &minPointsWithDominantShift);
	pParamData = paramList.AddParam("DDD.maxROIStep", RVLPARAM_TYPE_INT, &maxROIStep);
	pParamData = paramList.AddParam("DDD.edgeModel", RVLPARAM_TYPE_ID, &edgeModel);
	paramList.AddID(pParamData, "BOX", RVLDDD_EDGE_MODEL_BOX);
	paramList.AddID(pParamData, "CYLINDER", RVLDDD_EDGE_MODEL_CYLINDER);
	pParamData = paramList.AddParam("DDD.fit3DTo2D.EDT.maxDist", RVLPARAM_TYPE_INT, &fit3DTo2DEDTMaxDist);
	pParamData = paramList.AddParam("DDD.fit3DTo2D.minDefDoFs", RVLPARAM_TYPE_INT, &fit3DTo2DMinDefDoFs);
	pParamData = paramList.AddParam("DDD.fit3DTo2D.nEdgeSamples", RVLPARAM_TYPE_INT, &fit3DTo2DNoEdgeSamples);
	pParamData = paramList.AddParam("DDD.fit3DTo2D.lambdaR", RVLPARAM_TYPE_FLOAT, &fit3DTo2DLambdaR);
	pParamData = paramList.AddParam("DDD.fit3DTo2D.lambdat", RVLPARAM_TYPE_FLOAT, &fit3DTo2DLambdat);
	pParamData = paramList.AddParam("DDD.fit3DTo2D.stereo", RVLPARAM_TYPE_BOOL, &fit3DTo2DStereo);
	pParamData = paramList.AddParam("DDD.no_RANSAC_iterations", RVLPARAM_TYPE_INT, &nRANSACIterations);
	pParamData = paramList.AddParam("DDD.pointAssociationGridCellSize", RVLPARAM_TYPE_INT, &pointAssociationGridCellSize);
	pParamData = paramList.AddParam("DDD.maxCoPlanarSurfelRefPtDist", RVLPARAM_TYPE_FLOAT, &maxCoPlanarSurfelRefPtDist);
}

#define RVLDDD_DETECTRGBEDGES_VISUALIZE
// #define RVLDDD_DETECTRGBEDGES_VISUALIZE_HOUGH
#define RVLDDD_DETECTRGBEDGES_VISUALIZE_LINES

void DDDetector::DetectRGBEdgeLineSegments(
	cv::Mat RGB,
	int cannyThrL,
	int cannyThrH,
	int minHoughLineSize,
	cv::Mat &edges,
	cv::Mat &sobelx,
	cv::Mat &sobely,
	Array<RECOG::DDD::EdgeLineSegment> &lineSegmentsOut,
	int *&lineSegmentPixIdxMem,
	int *&lineSegmentMap)
{
	// Parameters.

	int linePtTol = 2;	// pix
	int maxLineGap = 5; // pix
	float mincsN = 0.8f;
	float lineSegmentRankingResolution = 0.1f;
	float minLineSegmentWeight = 20.0f;

	// Constants.

	int w = RGB.cols;
	int h = RGB.rows;
	int nPix = w * h;

	// Convert to graycsale.

	cv::Mat I;
	cv::cvtColor(RGB, I, cv::COLOR_BGR2GRAY);

	// Blur the image for better edge detection.

	cv::Mat IBlured;
	cv::GaussianBlur(I, IBlured, cv::Size(3, 3), 0);

	// Canny edge detection.

	cv::Canny(IBlured, edges, cannyThrL, cannyThrH, 3, false);

	// Display canny edge detected image.

	// cv::imshow("Canny edge detection", edges);
	// cv::waitKey(0);

	// Diference of Gausian.

	cv::GaussianBlur(I, IBlured, cv::Size(7, 7), 0);
	cv::Sobel(IBlured, sobelx, CV_64F, 1, 0, 5);
	cv::Sobel(IBlured, sobely, CV_64F, 0, 1, 5);
	double *IuMap = (double *)(sobelx.data);
	double *IvMap = (double *)(sobely.data);

	// Hough transform.

	std::vector<cv::Vec2f> houghLines;
	cv::HoughLines(edges, houghLines, 1, CV_PI / 180, minHoughLineSize, 0, 0);

	// Display Hough lines.

#ifdef RVLDDD_DETECTRGBEDGES_VISUALIZE
	cv::cvtColor(edges, *pVisualizationData->displayImg, cv::COLOR_GRAY2BGR);
#ifdef RVLDDD_DETECTRGBEDGES_VISUALIZE_HOUGH
	for (size_t i = 0; i < houghLines.size(); i++)
	{
		float rho = houghLines[i][0], theta = houghLines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		double c = w + h;
		pt1.x = cvRound(x0 + c * (-b));
		pt1.y = cvRound(y0 + c * (a));
		pt2.x = cvRound(x0 - c * (-b));
		pt2.y = cvRound(y0 - c * (a));
		cv::line(pVisualizationData->displayImg, pt1, pt2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		printf("line %d\n", i);
		if (i % 10 == 9)
		{
			cv::imshow("Hough Lines", pVisualizationData->displayImg);
			cv::waitKey();
		}
	}
#endif
#endif

	// Lines for segmentation.

	cv::Vec2f houghLine;
	int nHoughLines = houghLines.size();
	float rho, th;
	Array<RECOG::DDD::Line2D> houghLines_;
	houghLines_.Element = new RECOG::DDD::Line2D[nHoughLines];
	houghLines_.n = nHoughLines;
	RECOG::DDD::Line2D *pHoughLine = houghLines_.Element;
	size_t iHoughLine;
	for (iHoughLine = 0; iHoughLine < nHoughLines; iHoughLine++, pHoughLine++)
	{
		houghLine = houghLines[iHoughLine];
		th = houghLine[1];
		pHoughLine->N[0] = cos(th);
		pHoughLine->N[1] = sin(th);
		pHoughLine->d = houghLine[0];
	}

	// Segment lines.

	Array<RECOG::DDD::EdgeLineSegment> lineSegments;
	RECOG::DDD::EdgeLineSegmentPixel *edgeLineSegmentMem;
	int nClusterElements;
	// SegmentEdgeLines(edges, IuMap, IvMap, houghLines_, linePtTol, mincsN, maxLineGap, lineSegments, edgeLineSegmentMem, nClusterElements);
	SegmentEdgeLines2(edges, IuMap, IvMap, houghLines_, linePtTol, mincsN, maxLineGap, lineSegments, edgeLineSegmentMem, nClusterElements);

	// Remove redundant line segments.

	Array<QList<QLIST::Index>> Queue;
	Queue.n = w / lineSegmentRankingResolution + 1;
	Queue.Element = new QList<QLIST::Index>[Queue.n];
	QLIST::Index *lineSegmentIdxMem = new QLIST::Index[nClusterElements];
	QLIST::Index *pLineSegmentIdx = lineSegmentIdxMem;
	int iBin;
	QList<QLIST::Index> *pBin = Queue.Element;
	for (iBin = 0; iBin < Queue.n; iBin++, pBin++)
		RVLQLIST_INIT(pBin);
	int iLineSegment;
	int maxiBin = 0;
	RECOG::DDD::EdgeLineSegment *pLineSegment;
	for (iLineSegment = 0; iLineSegment < lineSegments.n; iLineSegment++)
	{
		pLineSegment = lineSegments.Element + iLineSegment;
		pLineSegmentIdx->Idx = iLineSegment;
		iBin = (int)floor(pLineSegment->w / lineSegmentRankingResolution);
		pBin = Queue.Element + iBin;
		RVLQLIST_ADD_ENTRY(pBin, pLineSegmentIdx);
		pLineSegmentIdx++;
		if (iBin > maxiBin)
			maxiBin = iBin;
	}
	int miniBin = (int)floor(minLineSegmentWeight / lineSegmentRankingResolution);
	lineSegmentMap = new int[nPix];
	memset(lineSegmentMap, 0xff, nPix * sizeof(int));
	float newLineSegmentWeight;
	int iBin_;
	QList<QLIST::Index> *pBin_;
	QLIST::Index **ppLineSegmentIdx;
	RECOG::DDD::EdgeLineSegmentPixel *pEdgeLineSegmentPix;
	QList<RECOG::DDD::EdgeLineSegmentPixel> *pSegmentPixList;
	RECOG::DDD::EdgeLineSegmentPixel **ppEdgeLineSegmentPix;
	int iPix;
	for (iBin = maxiBin; iBin >= miniBin; iBin--)
	{
		pBin = Queue.Element + iBin;
		pLineSegmentIdx = pBin->pFirst;
		ppLineSegmentIdx = &(pBin->pFirst);
		while (pLineSegmentIdx)
		{
			iLineSegment = pLineSegmentIdx->Idx;
			// if (iLineSegment == 57)
			//	int debug = 0;
			pLineSegment = lineSegments.Element + iLineSegment;
			pSegmentPixList = &(pLineSegment->pix);
			newLineSegmentWeight = pLineSegment->w;
			pEdgeLineSegmentPix = pSegmentPixList->pFirst;
			ppEdgeLineSegmentPix = &(pSegmentPixList->pFirst);
			while (pEdgeLineSegmentPix)
			{
				iPix = pEdgeLineSegmentPix->iPix;
				if (lineSegmentMap[iPix] >= 0 && lineSegmentMap[iPix] != iLineSegment)
				{
					newLineSegmentWeight -= pEdgeLineSegmentPix->e;
					RVLQLIST_REMOVE_ENTRY(pSegmentPixList, pEdgeLineSegmentPix, ppEdgeLineSegmentPix);
				}
				else
				{
					lineSegmentMap[iPix] = iLineSegment;
					ppEdgeLineSegmentPix = &(pEdgeLineSegmentPix->pNext);
				}
				pEdgeLineSegmentPix = *ppEdgeLineSegmentPix;
			}
			if (newLineSegmentWeight < pLineSegment->w)
			{
				pLineSegment->w = (newLineSegmentWeight >= 0.0f ? newLineSegmentWeight : 0.0f);
				RVLQLIST_REMOVE_ENTRY(pBin, pLineSegmentIdx, ppLineSegmentIdx);
				iBin_ = (int)floor(pLineSegment->w / lineSegmentRankingResolution);
				pBin_ = Queue.Element + iBin_;
				RVLQLIST_ADD_ENTRY(pBin_, pLineSegmentIdx);
			}
			else
				ppLineSegmentIdx = &(pLineSegmentIdx->pNext);
			pLineSegmentIdx = *ppLineSegmentIdx;
		}
	}
	delete[] Queue.Element;
	delete[] lineSegmentIdxMem;
	int iLineSegmentEndPix[2];
	int nValidLineSegments = 0;
	int nLineSegmentPixelsTotal = 0;
	int i;
	float e;
	float P[2];
	for (iLineSegment = 0; iLineSegment < lineSegments.n; iLineSegment++)
	{
		pLineSegment = lineSegments.Element + iLineSegment;
		if (pLineSegment->w < minLineSegmentWeight)
			continue;
		nValidLineSegments++;
		pSegmentPixList = &(pLineSegment->pix);
		pEdgeLineSegmentPix = pSegmentPixList->pFirst;
		iLineSegmentEndPix[0] = pEdgeLineSegmentPix->iPix;
		while (pEdgeLineSegmentPix)
		{
			iLineSegmentEndPix[1] = pEdgeLineSegmentPix->iPix;
			nLineSegmentPixelsTotal++;
			pEdgeLineSegmentPix = pEdgeLineSegmentPix->pNext;
		}
		iHoughLine = pLineSegment->iCluster / 2;
		pHoughLine = houghLines_.Element + iHoughLine;
		for (i = 0; i < 2; i++)
		{
			iPix = iLineSegmentEndPix[i];
			P[0] = (float)(iPix % w);
			P[1] = (float)(iPix / w);
			e = pHoughLine->N[0] * P[0] + pHoughLine->N[1] * P[1] - pHoughLine->d;
			pLineSegment->P[i][0] = P[0] - e * pHoughLine->N[0];
			pLineSegment->P[i][1] = P[1] - e * pHoughLine->N[1];
		}
	}

	// Copy valid line segments and their belonging pixels to lineSegmentsOut.

	lineSegmentsOut.Element = new RECOG::DDD::EdgeLineSegment[nValidLineSegments];
	lineSegmentsOut.n = 0;
	lineSegmentPixIdxMem = new int[nLineSegmentPixelsTotal];
	int *piLineSegmentPix = lineSegmentPixIdxMem;
	memset(lineSegmentMap, 0xff, nPix * sizeof(int));
	RECOG::DDD::EdgeLineSegment *pLineSegment_;
	Moments2D<double> moments;
	double P_[2], P0[2];
	cv::Mat cvC(2, 2, CV_64FC1);
	double *C = (double *)(cvC.data);
	cv::Mat cvEigC, cvEigVC;
	// double* eigC;
	double *eigVC;
	for (iLineSegment = 0; iLineSegment < lineSegments.n; iLineSegment++)
	{
		pLineSegment = lineSegments.Element + iLineSegment;
		if (pLineSegment->w < minLineSegmentWeight)
			continue;
		pLineSegment_ = lineSegmentsOut.Element + lineSegmentsOut.n;
		pLineSegment_->iCluster = pLineSegment->iCluster;
		pLineSegment_->w = pLineSegment->w;
		InitMoments2D<double>(moments);
		pLineSegment_->pix_.Element = piLineSegmentPix;
		pSegmentPixList = &(pLineSegment_->pix);
		RVLQLIST_INIT(pSegmentPixList);
		pSegmentPixList = &(pLineSegment->pix);
		pEdgeLineSegmentPix = pSegmentPixList->pFirst;
		while (pEdgeLineSegmentPix)
		{
			iPix = pEdgeLineSegmentPix->iPix;
			*(piLineSegmentPix++) = iPix;
			lineSegmentMap[iPix] = lineSegmentsOut.n;
			P_[0] = (float)(iPix % w);
			P_[1] = (float)(iPix / w);
			UpdateMoments2D<double>(moments, P_);
			pEdgeLineSegmentPix = pEdgeLineSegmentPix->pNext;
		}
		pLineSegment_->pix_.n = moments.n;
		GetCovMatrix2<double>(&moments, C, P0);
		pLineSegment_->P0[0] = P0[0];
		pLineSegment_->P0[1] = P0[1];
		cv::eigen(cvC, cvEigC, cvEigVC);
		// eigC = (double*)(cvEigC.data);
		eigVC = (double *)(cvEigVC.data);
		pLineSegment_->N[0] = eigVC[2];
		pLineSegment_->N[1] = eigVC[3];
		for (i = 0; i < 2; i++)
		{
			e = pLineSegment_->N[0] * (pLineSegment->P[i][0] - pLineSegment_->P0[0]) + pLineSegment_->N[1] * (pLineSegment->P[i][1] - pLineSegment_->P0[1]);
			pLineSegment_->P[i][0] = pLineSegment->P[i][0] - e * pLineSegment_->N[0];
			pLineSegment_->P[i][1] = pLineSegment->P[i][1] - e * pLineSegment_->N[1];
		}
		lineSegmentsOut.n++;
	}

	// Display line segments.

	cv::cvtColor(edges, *pVisualizationData->displayImg, cv::COLOR_GRAY2BGR);
	for (iLineSegment = 0; iLineSegment < lineSegmentsOut.n; iLineSegment++)
	{
		pLineSegment = lineSegmentsOut.Element + iLineSegment;
		cv::line(*pVisualizationData->displayImg,
				 cv::Point(pLineSegment->P[0][0], pLineSegment->P[0][1]),
				 cv::Point(pLineSegment->P[1][0], pLineSegment->P[1][1]),
				 cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
	}
	cv::imshow("Hough Lines", *pVisualizationData->displayImg);
	cv::waitKey();

	delete[] houghLines_.Element;
	delete[] lineSegments.Element;
	delete[] edgeLineSegmentMem;
}

void DDDetector::SegmentEdgeLines(
	cv::Mat &edges,
	double *IuMap,
	double *IvMap,
	Array<RECOG::DDD::Line2D> lines,
	int linePtTolIn,
	float mincsN,
	int maxLineGap,
	Array<RECOG::DDD::EdgeLineSegment> &lineSegments,
	RECOG::DDD::EdgeLineSegmentPixel *&edgeLineSegmentMem,
	int &nClusterElements)
{
	// Constants.

	int w = edges.cols;
	int h = edges.rows;
	int nPix = w * h;
	float linePtTol = (float)linePtTolIn;

	// Edge pixels.

	Array<int> iEdgePix;
	iEdgePix.Element = new int[nPix];
	iEdgePix.n = 0;
	int iPix;
	for (iPix = 0; iPix < nPix; iPix++)
		if (edges.data[iPix])
			iEdgePix.Element[iEdgePix.n++] = iPix;

	// Line clusters.

	struct LineCluster
	{
		std::vector<RECOG::DDD::EdgeLineSegmentPixel> pix;
	};
	int i, j;
	float e;
	float P[2], N[2];
	Array<LineCluster> lineClusters;
	lineClusters.Element = new LineCluster[2 * lines.n];
	float gradIMagnitude;
	float Iu, Iv;
	float csN;
	RECOG::DDD::EdgeLineSegmentPixel clusterElement;
	RECOG::DDD::Line2D *pLine;
	int iLine;
	for (i = 0; i < iEdgePix.n; i++)
	{
		iPix = iEdgePix.Element[i];
		P[0] = (float)(iPix % w);
		P[1] = (float)(iPix / w);
		Iu = IuMap[iPix];
		Iv = IvMap[iPix];
		gradIMagnitude = sqrt(Iu * Iu + Iv * Iv);
		N[0] = Iu / gradIMagnitude;
		N[1] = Iv / gradIMagnitude;
		pLine = lines.Element;
		for (iLine = 0; iLine < lines.n; iLine++, pLine++)
		{
			e = pLine->N[0] * P[0] + pLine->N[1] * P[1] - pLine->d;
			if (e < 0.0f)
				e = -e;
			if (e > linePtTol)
				continue;
			csN = pLine->N[0] * N[0] + pLine->N[1] * N[1];
			if (csN > mincsN)
				j = 0;
			else if (csN < -mincsN)
				j = 1;
			else
				continue;
			clusterElement.iPix = iPix;
			clusterElement.e = e;
			lineClusters.Element[2 * iLine + j].pix.push_back(clusterElement);
		}
	}
	delete[] iEdgePix.Element;

	// Line segments.

	nClusterElements = 0;
	int nLineClusters = 2 * lines.n;
	int iLineCluster;
	for (iLineCluster = 0; iLineCluster < nLineClusters; iLineCluster++)
		nClusterElements += lineClusters.Element[iLineCluster].pix.size();
	edgeLineSegmentMem = new RECOG::DDD::EdgeLineSegmentPixel[nClusterElements];
	RECOG::DDD::EdgeLineSegmentPixel *pEdgeLineSegmentPix = edgeLineSegmentMem;
	LineCluster *pLineCluster;
	lineSegments.Element = new RECOG::DDD::EdgeLineSegment[nClusterElements];
	lineSegments.n = 0;
	RECOG::DDD::EdgeLineSegment *pLineSegment = lineSegments.Element - 1;
	QList<RECOG::DDD::EdgeLineSegmentPixel> *pSegmentPixList;
	int imageSize = RVLMAX(w, h);
	RECOG::DDD::EdgeLineSegmentPixel *linePixBuff = new RECOG::DDD::EdgeLineSegmentPixel[imageSize];
	memset(linePixBuff, 0xff, imageSize * sizeof(Pair<int, float>));
	Array<int> idxInLinePixBuff;
	idxInLinePixBuff.Element = new int[imageSize];
	int iP[2];
	int lineDir;
	int s;
	int mins, maxs;
	int gap;
	for (iLineCluster = 0; iLineCluster < nLineClusters; iLineCluster++)
	{
		pLineCluster = lineClusters.Element + iLineCluster;
		iLine = iLineCluster / 2;
		// if (iHoughLine == 9)
		//	int debug = 0;
		pLine = lines.Element + iLine;
		lineDir = (RVLABS(pLine->N[0]) >= RVLABS(pLine->N[1]) ? 1 : 0);
		idxInLinePixBuff.n = 0;
		mins = w;
		maxs = -1;
		for (i = 0; i < pLineCluster->pix.size(); i++)
		{
			clusterElement = pLineCluster->pix[i];
			iPix = clusterElement.iPix;
			iP[0] = iPix % w;
			iP[1] = iPix / w;
			s = iP[lineDir];
			if (linePixBuff[s].iPix < 0)
				linePixBuff[s] = clusterElement;
			else if (clusterElement.e < linePixBuff[s].e)
				linePixBuff[s] = clusterElement;
			idxInLinePixBuff.Element[idxInLinePixBuff.n++] = s;
			if (s < mins)
				mins = s;
			if (s > maxs)
				maxs = s;
		}
		gap = w;
		for (s = mins; s <= maxs; s++)
		{
			iPix = linePixBuff[s].iPix;
			if (iPix >= 0)
			{
				if (gap > maxLineGap)
				{
					pLineSegment++;
					pSegmentPixList = &(pLineSegment->pix);
					RVLQLIST_INIT(pSegmentPixList);
					pLineSegment->w = 0.0f;
					pLineSegment->iCluster = iLineCluster;
				}
				gap = 0;
				pEdgeLineSegmentPix->iPix = iPix;
				pEdgeLineSegmentPix->e = (1.0f - linePixBuff[s].e / linePtTol);
				RVLQLIST_ADD_ENTRY(pSegmentPixList, pEdgeLineSegmentPix);
				pLineSegment->w += pEdgeLineSegmentPix->e;
				pEdgeLineSegmentPix++;
			}
			else
				gap++;
		}
		for (i = 0; i < idxInLinePixBuff.n; i++)
			linePixBuff[idxInLinePixBuff.Element[i]].iPix = -1;
	}
	lineSegments.n = pLineSegment - lineSegments.Element + 1;

	delete[] lineClusters.Element;
	delete[] linePixBuff;
	delete[] idxInLinePixBuff.Element;
}

void DDDetector::SegmentEdgeLines2(
	cv::Mat &edges,
	double *IuMap,
	double *IvMap,
	Array<RECOG::DDD::Line2D> lines,
	int linePtTol,
	float mincsN,
	int maxLineGap,
	Array<RECOG::DDD::EdgeLineSegment> &lineSegments,
	RECOG::DDD::EdgeLineSegmentPixel *&edgeLineSegmentMem,
	int &nClusterElements)
{
	// Constants.

	int w = edges.cols;
	int h = edges.rows;
	int nPix = w * h;
	float fLinePtTol = (float)linePtTol;

	/// Line segments.

	float imgRectNMem[8];
	memset(imgRectNMem, 0, 8 * sizeof(float));
	imgRectNMem[0 * 2 + 0] = 1.0f;
	imgRectNMem[1 * 2 + 0] = -1.0f;
	imgRectNMem[2 * 2 + 1] = 1.0f;
	imgRectNMem[3 * 2 + 1] = -1.0f;
	float *imgRectN;
	float imgRectd[4];
	imgRectd[0] = (float)w;
	imgRectd[1] = 0.0f;
	imgRectd[2] = (float)h;
	imgRectd[3] = 0.0f;
	float imgDiagonal = sqrt(imgRectd[0] * imgRectd[0] + imgRectd[2] * imgRectd[2]);
	float s, smin, smax;
	float V[2];
	float C[2];
	float N[2];
	int i, j;
	float k;
	float lineEndPt[2][2];
	int x1, y1, x2, y2;
	BresenhamData bresenhamData;
	int x, y;
	int imgRectSize = RVLMAX(w, h);
	uchar *visPix;
	int u, v, iPix;
	int move[5] = {0, 1, -1, 2, -2};
	int nMoves = 2 * linePtTol + 1;
	RECOG::DDD::Line2D *pLine = lines.Element;
	int iLine;
	float gradIMagnitude;
	float Iu, Iv;
	float csN;
	bool bHorizontal;
	int iLineOrient;
	float lineN[2];
	edgeLineSegmentMem = new RECOG::DDD::EdgeLineSegmentPixel[lines.n * imgRectSize];
	RECOG::DDD::EdgeLineSegmentPixel **firstPixBuff = new RECOG::DDD::EdgeLineSegmentPixel *[lines.n * imgRectSize];
	RECOG::DDD::EdgeLineSegmentPixel **ppFirstPix = firstPixBuff - 1;
	RECOG::DDD::EdgeLineSegmentPixel *pPix = edgeLineSegmentMem;
	RECOG::DDD::EdgeLineSegmentPixel *pPixDummy;
	RECOG::DDD::EdgeLineSegmentPixel **ppPix = &pPixDummy;
	int gap;
	float e;
	int *lineIdxMem = new int[lines.n * imgRectSize];
	int *pLineIdx = lineIdxMem;
	for (iLine = 0; iLine < lines.n; iLine++, pLine++)
	{
		V[0] = -pLine->N[1];
		V[1] = pLine->N[0];
		C[0] = pLine->d * pLine->N[0];
		C[1] = pLine->d * pLine->N[1];
		bHorizontal = (RVLABS(V[0]) >= RVLABS(V[1]));

		// Crop line to fit in the image.

		smin = -imgDiagonal;
		smax = imgDiagonal;
		imgRectN = imgRectNMem;
		for (i = 0; i < 4; i++, imgRectN += 2)
		{
			k = imgRectN[0] * V[0] + imgRectN[1] * V[1];
			if (RVLABS(k) < 1e-7)
				continue;
			s = (imgRectd[i] - (imgRectN[0] * C[0] + imgRectN[1] * C[1])) / k;
			if (k < 0.0f)
			{
				if (s > smin)
					smin = s;
			}
			else
			{
				if (s < smax)
					smax = s;
			}
		}
		lineEndPt[0][0] = C[0] + smin * V[0];
		lineEndPt[0][1] = C[1] + smin * V[1];
		lineEndPt[1][0] = C[0] + smax * V[0];
		lineEndPt[1][1] = C[1] + smax * V[1];

		// Line segments along the Hough line.

		x1 = (int)lineEndPt[0][0];
		if (x1 < 0)
			x1 = 0;
		else if (x1 >= w)
			x1 = w - 1;
		y1 = (int)lineEndPt[0][1];
		if (y1 < 0)
			y1 = 0;
		else if (y1 >= h)
			y1 = h - 1;
		x2 = (int)lineEndPt[1][0];
		if (x2 < 0)
			x2 = 0;
		else if (x2 >= w)
			x2 = w - 1;
		y2 = (int)lineEndPt[1][1];
		if (y2 < 0)
			y2 = 0;
		else if (y2 >= h)
			y2 = h - 1;

		lineN[0] = pLine->N[0];
		lineN[1] = pLine->N[1];
		for (iLineOrient = 0; iLineOrient < 2; iLineOrient++, lineN[0] = -lineN[0], lineN[1] = -lineN[1])
		{
			RVLBRESENHAMINIT(x1, y1, x2, y2, x, y, bresenhamData);
			gap = imgRectSize;

			while (true)
			{
				if (bHorizontal)
					u = x;
				else
					v = y;
				for (i = 0; i < nMoves; i++)
				{
					if (bHorizontal)
					{
						v = y + move[i];
						if (v < 0)
							continue;
						if (v >= h)
							continue;
					}
					else
					{
						u = x + move[i];
						if (u < 0)
							continue;
						if (u >= w)
							continue;
					}
					iPix = u + v * w;
					if (!edges.data[iPix])
						continue;
					e = pLine->N[0] * (float)u + pLine->N[1] * (float)v - pLine->d;
					if (e < 0.0f)
						e = -e;
					if (e > fLinePtTol)
						continue;
					Iu = IuMap[iPix];
					Iv = IvMap[iPix];
					gradIMagnitude = sqrt(Iu * Iu + Iv * Iv);
					N[0] = Iu / gradIMagnitude;
					N[1] = Iv / gradIMagnitude;
					csN = lineN[0] * N[0] + lineN[1] * N[1];
					if (csN >= mincsN)
						break;
				}
				if (i < nMoves)
				{
					if (gap > maxLineGap)
					{
						*ppPix = NULL;
						ppFirstPix++;
						ppPix = ppFirstPix;
						*(pLineIdx++) = 2 * iLine + iLineOrient;
					}
					gap = 0;
					pPix->iPix = iPix;
					pPix->e = (1.0f - e / fLinePtTol);
					*ppPix = pPix;
					ppPix = &(pPix->pNext);
					pPix++;
#ifdef RVLDDD_DETECTRGBEDGES_VISUALIZE
#ifdef RVLDDD_DETECTRGBEDGES_VISUALIZE_LINES
					visPix = (unsigned char*) *pVisualizationData->displayImg->data + 3 * iPix;
					RVLSET3VECTOR(visPix, 255, 0, 0);
#endif
#endif
				}
				else
					gap++;
				if (bresenhamData.bCompleted)
					break;
				RVLBRESENHAMUPDATE(bresenhamData, x, y);
			}
		}
	}
	nClusterElements = pPix - edgeLineSegmentMem;
	lineSegments.n = ppFirstPix - firstPixBuff;
	lineSegments.Element = new RECOG::DDD::EdgeLineSegment[lineSegments.n];
	int iLineSegment;
	RECOG::DDD::EdgeLineSegment *pLineSegment;
	for (iLineSegment = 0; iLineSegment < lineSegments.n; iLineSegment++)
	{
		pLineSegment = lineSegments.Element + iLineSegment;
		pLineSegment->iCluster = lineIdxMem[iLineSegment];
		pPix = pLineSegment->pix.pFirst = firstPixBuff[iLineSegment];
		pLineSegment->w = 0;
		while (pPix)
		{
			pLineSegment->w += pPix->e;
			pPix = pPix->pNext;
		}
	}
	delete[] firstPixBuff;
	delete[] lineIdxMem;

	///

#ifdef RVLDDD_DETECTRGBEDGES_VISUALIZE
	cv::imshow("Lines", *pVisualizationData->displayImg);
	cv::waitKey();
#endif

	///
}

void DDDetector::CreateModels(
	Array<Mesh> modelMeshes,
	std::vector<std::string> modelFileNames)
{
	for (int iModel = 0; iModel < modelMeshes.n; iModel++)
	{
		Mesh *pMesh = modelMeshes.Element + iModel;

		// Detect surfels.

		// pSurfels->Init(pMesh);
		// pSurfelDetector->Init(pMesh, pSurfels, pMem);
		// printf("Segmentation to surfels...");
		// pSurfelDetector->Segment(pMesh, pSurfels);
		// printf("completed.\n");
		// int nSurfels = pSurfels->NodeArray.n;
		// printf("No. of surfels = %d\n", nSurfels);

		// Visualize surfels.

		// uchar SelectionColor[] = { 0, 255, 0 };
		// pSurfels->NodeColors(SelectionColor);
		// pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);
		// pSurfels->Display(pVisualizer, pMesh);
		// pVisualizer->Run();
		// pVisualizer->renderer->RemoveAllViewProps();

		// Subsample mesh.

		VoxelGrid MSubsamplingGrid;
		MSubsamplingGrid.Create(pMesh->NodeArray, 0.04f);
		Array<int> MPointSubset;
		MSubsamplingGrid.SubSample(pMesh->NodeArray, 22.5f, MPointSubset);

		// Write subsampled mesh to a file.

		Array<OrientedPoint> model;
		model.Element = new OrientedPoint[MPointSubset.n];
		model.n = MPointSubset.n;
		OrientedPoint *pTgtPt = model.Element;
		Point *pSrcPt;
		int iPt;
		for (int i = 0; i < model.n; i++, pTgtPt++)
		{
			iPt = MPointSubset.Element[i];
			pSrcPt = pMesh->NodeArray.Element + iPt;
			RVLCOPY3VECTOR(pSrcPt->P, pTgtPt->P);
			RVLCOPY3VECTOR(pSrcPt->N, pTgtPt->N);
		}
		std::string modelInFileName = modelFileNames[iModel];
		std::string modelOutFileName = modelInFileName.substr(0, modelInFileName.rfind('.') + 1) + "dat";
		FILE *fpModel = fopen(modelOutFileName.data(), "wb");
		fwrite(&(model.n), sizeof(int), 1, fpModel);
		fwrite(model.Element, sizeof(OrientedPoint), model.n, fpModel);
		fclose(fpModel);
		delete[] model.Element;
	}
}

void DDDetector::CreateCuboidModel(
	float *size,
	float sampleDensity,
	RECOG::DDD::Model *pModel)
{
	RVLCOPY3VECTOR(size, pModel->bboxSize);
	RVLNULL3VECTOR(pModel->bboxCenter);
	int iSide, iAxis1, iAxis2, iAxis3, nSideSamples, iSample;
	float sign;
	float sampleDensity2 = sampleDensity * sampleDensity;
	pModel->points.n = 0;
	for (iSide = 0; iSide < 3; iSide++)
	{
		iAxis2 = (iSide + 1) % 3;
		iAxis3 = (iAxis2 + 1) % 3;
		nSideSamples = (int)round(size[iAxis2] * size[iAxis3] / sampleDensity2);
		pModel->points.n += nSideSamples;
	}
	pModel->points.n *= 2;
	pModel->points.Element = new OrientedPoint[pModel->points.n];
	OrientedPoint *pPt = pModel->points.Element;
	for (iSide = 0; iSide < 6; iSide++)
	{
		iAxis1 = iSide % 3;
		iAxis2 = (iAxis1 + 1) % 3;
		iAxis3 = (iAxis2 + 1) % 3;
		sign = (iSide < 3 ? 1.0f : -1.0f);
		nSideSamples = (int)round(size[iAxis2] * size[iAxis3] / sampleDensity2);
		for (iSample = 0; iSample < nSideSamples; iSample++, pPt++)
		{
			pPt->P[iAxis1] = 0.5f * sign * size[iAxis1];
			pPt->P[iAxis2] = size[iAxis2] * ((float)rand() / (float)RAND_MAX - 0.5f);
			pPt->P[iAxis3] = size[iAxis3] * ((float)rand() / (float)RAND_MAX - 0.5f);
			pPt->N[iAxis1] = sign;
			pPt->N[iAxis2] = pPt->N[iAxis3] = 0.0f;
		}
	}
	// Visualize cuboid point cloud.
	// Array<Point> points;
	// MESH::CreatePointArrayFromOrientedPointArray(pModel->points, points);
	// uchar green[] = { 0, 255, 0};
	// pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(points, green, 6.0f);
	// delete[] points.Element;
	// pVisualizationData->pVisualizer->Run();
}

void DDDetector::CreateSurfNetModel(
	float *A,
	int *AID,
	int nSurfaces,
	float *M,
	std::vector<std::vector<std::vector<int>>> SN,
	float minSamplingDensity,
	float *q,
	RECOG::DDD::Model *pModel,
	bool bVisualize)
{
	///	Sample points.

	int i, j;
	float *d = new float[nSurfaces];
	float *mxRow;
	RVLMULMXVECT(M, q, nSurfaces, 7, d, i, j, mxRow);
	int *axis = new int[nSurfaces];
	float *sign = new float[nSurfaces];
	for (i = 0; i < nSurfaces; i++)
	{
		axis[i] = AID[i] / 2;
		sign[i] = (AID[i] & 1 ? 1.0f : -1.0f);
	}
	int maxnContourEdges = 0;
	int nEdges = 0;
	int maxGridSize = 0;
	int normalAxis, planeAxis1;
	int gridSize[2];
	int iFace_, iFace__, iEdge;
	std::vector<std::vector<int>> face;
	std::vector<int> contour;
	for (int iFace = 0; iFace < SN.size(); iFace++)
	{
		face = SN[iFace];
		gridSize[0] = gridSize[1] = 0;
		normalAxis = axis[iFace];
		planeAxis1 = (normalAxis + 1) % 3;
		for (int iContour = 0; iContour < face.size(); iContour++)
		{
			contour = face[iContour];
			if (contour.size() > maxnContourEdges)
				maxnContourEdges = contour.size();
			nEdges += contour.size();
			for (iEdge = 0; iEdge < contour.size(); iEdge++)
			{
				iFace_ = contour[iEdge];
				gridSize[axis[iFace_] == planeAxis1 ? 0 : 1]++;
			}
			if (gridSize[0] > maxGridSize)
				maxGridSize = gridSize[0];
			if (gridSize[1] > maxGridSize)
				maxGridSize = gridSize[1];
		}
	}
	nEdges /= 2;
	pModel->edges.Element = new RECOG::DDD::ModelEdge[nEdges];
	pModel->edges.n = 0;
	RECOG::DDD::ModelPoint *MPtMem = new RECOG::DDD::ModelPoint[maxnContourEdges];
	RECOG::DDD::ModelPoint *pMPt;
	float *m, *s;
	Array<Point> vertices;
	vertices.Element = new Point[2 * nEdges];
	vertices.n = 0;
	Array<Pair<int, int>> edges;
	edges.Element = new Pair<int, int>[2 * nEdges];
	edges.n = 0;
	int iVertex, iVertex_;
	Point *pPt;
	std::vector<RECOG::DDD::ModelPoint> MPts;
	RECOG::DDD::ModelPoint MPt;
	int maxnCells = maxGridSize * maxGridSize;
	int *edgeMap = new int[maxnCells];
	Array<SortIndex<float>> grid[2];
	grid[0].Element = new SortIndex<float>[maxGridSize];
	grid[1].Element = new SortIndex<float>[maxGridSize];
	SortIndex<float> *pGrid, *pGrid_;
	int *grid_ = new int[SN.size()];
	int iGridAxis, edgeStartRow, edgeEndRow, edgeCol;
	int sign_;
	int surf;
	float cellSize;
	int nCellSamples[2];
	float cellStep[2];
	int k, l, u, v, i_;
	float fTmp;
	int iGridFace[2][2];
	float s_[7];
	float lambda[2];
	int iSample;
	int nContourEdges;
	int iTmp;
	int iEndFace[2];
	float edgeLen;
	int nEdgeSamples;
	float edgeStep;
	float *N, *N_;
	float V3Tmp[3], N__[3];
	float sign__;
	RECOG::DDD::ModelEdge *pEdge;
	int *support = new int[SN.size()];
	int iFace;
	for (iFace = 0; iFace < SN.size(); iFace++)
	{
		face = SN[iFace];
		N = A + 3 * AID[iFace];
		normalAxis = axis[iFace];
		planeAxis1 = (normalAxis + 1) % 3;
		grid[0].n = grid[1].n = 0;
		for (int iContour = 0; iContour < face.size(); iContour++)
		{
			contour = face[iContour];
			for (iEdge = 0; iEdge < contour.size(); iEdge++)
			{
				iFace_ = contour[iEdge];
				iGridAxis = (axis[iFace_] == planeAxis1 ? 0 : 1);
				pGrid = grid[iGridAxis].Element + grid[iGridAxis].n++;
				pGrid->idx = iFace_;
				pGrid->cost = d[iFace_] * sign[iFace_];
			}
		}
		for (i = 0; i < 2; i++)
		{
			BubbleSort<SortIndex<float>>(grid[i]);
			for (j = 0; j < grid[i].n; j++)
				grid_[grid[i].Element[j].idx] = j;
		}
		memset(edgeMap, 0, maxnCells * sizeof(int));
		for (int iContour = 0; iContour < face.size(); iContour++)
		{
			contour = face[iContour];
			nContourEdges = contour.size();
			if (bVisualize)
			{
				iFace_ = contour[nContourEdges - 1];
				for (iVertex = 0; iVertex < nContourEdges; iVertex++)
				{
					iFace__ = contour[iVertex];
					pMPt = MPtMem + iVertex;
					m = M + 7 * iFace;
					s = pMPt->S + 7 * axis[iFace];
					RVLSCALEVECTOR(m, sign[iFace], s, 7, i);
					m = M + 7 * iFace_;
					s = pMPt->S + 7 * axis[iFace_];
					RVLSCALEVECTOR(m, sign[iFace_], s, 7, i);
					m = M + 7 * iFace__;
					s = pMPt->S + 7 * axis[iFace__];
					RVLSCALEVECTOR(m, sign[iFace__], s, 7, i);
					iFace_ = iFace__;
				}
				iVertex_ = vertices.n + nContourEdges - 1;
				for (iVertex = 0; iVertex < nContourEdges; iVertex++)
				{
					pPt = vertices.Element + vertices.n;
					pMPt = MPtMem + iVertex;
					RVLMULMXVECT(pMPt->S, q, 3, 7, pPt->P, i, j, mxRow);
					edges.Element[edges.n].a = iVertex_;
					edges.Element[edges.n].b = vertices.n;
					iVertex_ = vertices.n;
					vertices.n++;
					edges.n++;
				}
			}
			for (iEdge = 0; iEdge < nContourEdges; iEdge++)
			{
				iFace_ = contour[iEdge];
				iEndFace[0] = contour[(iEdge + nContourEdges - 1) % nContourEdges];
				iEndFace[1] = contour[(iEdge + 1) % nContourEdges];
				if (iFace < iFace_)
				{
					N_ = A + 3 * AID[iFace_];
					RVLSUM3VECTORS(N, N_, MPt.N);
					RVLNORM3(MPt.N, fTmp);
					RVLCROSSPRODUCT3(N, N_, V3Tmp);
					edgeLen = d[iEndFace[1]] * sign[iEndFace[1]] - d[iEndFace[0]] * sign[iEndFace[0]];
					sign__ = (edgeLen >= 0.0f ? 1.0f : -1.0f);
					RVLNULL3VECTOR(N__);
					N__[axis[iEndFace[0]]] = sign__;
					if (RVLDOTPRODUCT3(V3Tmp, N__) > 0.0f)
					{
						pEdge = pModel->edges.Element + pModel->edges.n;
						pEdge->iFace = iFace;
						pEdge->iFace_ = iFace_;
						RVLSUM3VECTORS(N, N_, pEdge->N);
						RVLNORM3(pEdge->N, fTmp);
						pModel->edges.n++;
						edgeLen *= sign__;
						m = M + 7 * iFace;
						s = MPt.S + 7 * axis[iFace];
						RVLSCALEVECTOR(m, sign[iFace], s, 7, i);
						m = M + 7 * iFace_;
						s = MPt.S + 7 * axis[iFace_];
						RVLSCALEVECTOR(m, sign[iFace_], s, 7, i);
						fTmp = ceil(edgeLen * minSamplingDensity);
						nEdgeSamples = (int)fTmp;
						edgeStep = 1.0f / fTmp;
						s = MPt.S + 7 * axis[iEndFace[0]];
						for (u = 0; u < nEdgeSamples; u++)
						{
							memset(s, 0, 7 * sizeof(float));
							lambda[0] = ((float)u + 0.5f) * edgeStep;
							lambda[1] = 1.0f - lambda[0];
							for (l = 0; l < 2; l++)
							{
								iFace__ = iEndFace[l];
								m = M + 7 * iFace__;
								fTmp = sign[iFace__] * lambda[l];
								RVLSCALEVECTOR(m, fTmp, s_, 7, i_);
								RVLSUMVECTORS(s, s_, 7, s, i_);
							}
							MPts.push_back(MPt);
						}
					}
				}
				if (axis[iFace_] == planeAxis1)
				{
					edgeCol = grid_[iFace_];
					edgeStartRow = grid_[iEndFace[0]];
					edgeEndRow = grid_[iEndFace[1]];
					sign_ = (edgeStartRow < edgeEndRow ? 1 : -1);
					if (sign_ < 0.0f)
					{
						iTmp = edgeStartRow;
						edgeStartRow = edgeEndRow;
						edgeEndRow = iTmp;
					}
					for (i = edgeStartRow; i < edgeEndRow; i++)
						edgeMap[edgeCol + i * grid[0].n] = sign_;
				}
			}
		}
		surf = 0;
		RVLCOPY3VECTOR(N, MPt.N);
		m = M + 7 * iFace;
		s = MPt.S + 7 * axis[iFace];
		RVLSCALEVECTOR(m, sign[iFace], s, 7, i_);
		iTmp = MPts.size();
		for (j = 0; j < grid[1].n - 1; j++)
			for (i = 0; i < grid[0].n; i++)
			{
				surf += edgeMap[i + j * grid[0].n];
				if (surf != 0)
				{
					for (k = 0; k < 2; k++)
					{
						pGrid = grid[k].Element + (k ? j : i);
						pGrid_ = pGrid + 1;
						cellSize = pGrid_->cost - pGrid->cost;
						iGridFace[k][0] = pGrid->idx;
						iGridFace[k][1] = pGrid_->idx;
						fTmp = ceil(cellSize * minSamplingDensity);
						nCellSamples[k] = (int)fTmp;
						cellStep[k] = 1.0f / fTmp;
					}
					for (v = 0; v < nCellSamples[1]; v++)
						for (u = 0; u < nCellSamples[0]; u++)
						{
							for (k = 0; k < 2; k++)
							{
								s = MPt.S + 7 * axis[iGridFace[k][0]];
								memset(s, 0, 7 * sizeof(float));
								iSample = (k ? v : u);
								lambda[0] = ((float)iSample + 0.5f) * cellStep[k];
								lambda[1] = 1.0f - lambda[0];
								for (l = 0; l < 2; l++)
								{
									iFace_ = iGridFace[k][l];
									m = M + 7 * iFace_;
									fTmp = sign[iFace_] * lambda[l];
									RVLSCALEVECTOR(m, fTmp, s_, 7, i_);
									RVLSUMVECTORS(s, s_, 7, s, i_);
								}
							}
							MPts.push_back(MPt);
						}
				}
			}
		support[iFace] = MPts.size() - iTmp;
	}
	pModel->points_.Element = new RECOG::DDD::ModelPoint[MPts.size()];
	pMPt = pModel->points_.Element;
	for (i = 0; i < MPts.size(); i++, pMPt++)
		*pMPt = MPts[i];
	pModel->points_.n = MPts.size();
	pModel->nSurfaces = SN.size();
	pModel->d = new float[SN.size()];
	memcpy(pModel->d, d, SN.size() * sizeof(float));
	pModel->A = new float[3 * 6];
	memcpy(pModel->A, A, 3 * 6 * sizeof(float));
	pModel->AID = new int[SN.size()];
	memcpy(pModel->AID, AID, SN.size() * sizeof(int));
	pModel->M = new float[nSurfaces * 7];
	memcpy(pModel->M, M, nSurfaces * 7 * sizeof(float));

	// QR matrices.

	float *dominantAxisCandidateMem = new float[6 * SN.size()];
	int nDominantAxisCandidates = 0;
	int iCandidate;
	float *axisMem = new float[6 * SN.size()];
	float *axis_;
	float *c = new float[SN.size()];
	float *dominantAxisCandidate;
	int *dominantAxisCandidateSupport = new int[SN.size()];
	for (iFace = 0; iFace < SN.size(); iFace++)
	{
		m = M + 7 * iFace + 1;
		RVLDOTPRODUCT(m, m, 6, fTmp, i_);
		c[iFace] = sqrt(fTmp);
		fTmp = 1.0f / c[iFace];
		axis_ = axisMem + 6 * iFace;
		RVLSCALEVECTOR(m, fTmp, axis_, 6, i_);
		for (iCandidate = 0; iCandidate < nDominantAxisCandidates; iCandidate++)
		{
			dominantAxisCandidate = dominantAxisCandidateMem + 6 * iCandidate;
			RVLDOTPRODUCT(dominantAxisCandidate, axis_, 6, fTmp, i_);
			if (1.0f - RVLABS(fTmp) <= 1e-3)
				break;
		}
		if (iCandidate < nDominantAxisCandidates)
			dominantAxisCandidateSupport[iCandidate] += support[iFace];
		else
		{
			dominantAxisCandidate = dominantAxisCandidateMem + 6 * nDominantAxisCandidates;
			memcpy(dominantAxisCandidate, axis_, 6 * sizeof(float));
			dominantAxisCandidateSupport[nDominantAxisCandidates] = support[iFace];
			nDominantAxisCandidates++;
		}
	}
	int iDominantAxis = 0;
	for (iCandidate = 1; iCandidate < nDominantAxisCandidates; iCandidate++)
		if (dominantAxisCandidateSupport[iCandidate] > dominantAxisCandidateSupport[iDominantAxis])
			iDominantAxis = iCandidate;
	pModel->Q = new float[6 * 6];
	float *q_ = pModel->Q;
	dominantAxisCandidate = dominantAxisCandidateMem + 6 * iDominantAxis;
	memcpy(q_, dominantAxisCandidate, 6 * sizeof(float));
	float b[6];
	float e, maxe;
	pModel->R = new float[6 * SN.size()];
	float *r = pModel->R;
	for (i = 1; i < 6; i++)
	{
		maxe = 0.0f;
		q_ = pModel->Q + 6 * i;
		for (iFace = 0; iFace < SN.size(); iFace++)
		{
			axis_ = axisMem + 6 * iFace;
			RECOG::DDD::ProjectToBase(axis_, 6, pModel->Q, i, r, b, e);
			if (e > maxe)
			{
				maxe = e;
				memcpy(q_, b, 6 * sizeof(float));
			}
		}
	}
	pModel->info = new int[SN.size()];
	for (iFace = 0; iFace < SN.size(); iFace++)
	{
		r = pModel->R + 6 * iFace;
		axis_ = axisMem + 6 * iFace;
		RECOG::DDD::ProjectToBase(axis_, 6, pModel->Q, 6, r, b, e);
		for (i = 0; i < 6; i++)
			if (RVLABS(r[i]) > 0.5f)
				pModel->info[iFace] = i;
	}

	// Visualization.

	if (bVisualize)
	{
		Array<Point> samplePts;
		Array<Pair<int, int>> normals;
		RECOG::DDD::HypothesisSV hyp;
		memcpy(hyp.s, q + 1, 6 * sizeof(float));
		RVLUNITMX3(hyp.RMS);
		VisualizeStorageVolumeModel(pModel, hyp, samplePts, &normals);
		Visualizer visualizer;
		visualizer.Create();
		uchar red[] = {255, 0, 0};
		uchar green[] = {0, 255, 0};
		visualizer.DisplayLines(vertices, edges, red);
		visualizer.DisplayPointSet<float, Point>(samplePts, green, 6.0f);
		samplePts.n *= 2;
		visualizer.DisplayLines(samplePts, normals, red);
		visualizer.Run();
		delete[] samplePts.Element;
		delete[] normals.Element;
	}

	// Free memory.

	delete[] MPtMem;
	delete[] vertices.Element;
	delete[] edges.Element;
	delete[] edgeMap;
	delete[] grid[0].Element;
	delete[] grid[1].Element;
	delete[] grid_;
	delete[] support;
	delete[] dominantAxisCandidateMem;
	delete[] dominantAxisCandidateSupport;
	delete[] axisMem;
	delete[] c;
	delete[] d;
	delete[] axis;
	delete[] sign;

	///
}

void DDDetector::BoxNormals(float *A)
{
	memset(A, 0, 3 * 6 * sizeof(float));
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 2; j++)
			A[3 * (2 * i + j) + i] = (float)(2 * j - 1);
}

void DDDetector::CreateCuboidModel2(
	float *size,
	float minSamplingDensity,
	RECOG::DDD::Model *pModel,
	bool bVisualize)
{
	// Parameters.

	float q[7];
	memset(q, 0, 7 * sizeof(float));
	float *s = q + 1;
	RVLCOPY3VECTOR(size, s);
	RVLSCALE3VECTOR(s, 0.5f, s);

	/// SurfNet model.

	float A[3 * 6];
	BoxNormals(A);
	int AID[6] = {0, 1, 2, 3, 4, 5};
	float M[6 * 7] = {
		0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0};
	std::vector<std::vector<std::vector<int>>> SN;
	std::vector<std::vector<int>> face;
	std::vector<int> contour;
	// Surface 0.
	contour.push_back(2);
	contour.push_back(5);
	contour.push_back(3);
	contour.push_back(4);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 1.
	face.clear();
	contour.clear();
	contour.push_back(2);
	contour.push_back(4);
	contour.push_back(3);
	contour.push_back(5);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 2.
	face.clear();
	contour.clear();
	contour.push_back(0);
	contour.push_back(4);
	contour.push_back(1);
	contour.push_back(5);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 3.
	face.clear();
	contour.clear();
	contour.push_back(0);
	contour.push_back(5);
	contour.push_back(1);
	contour.push_back(4);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 4.
	face.clear();
	contour.clear();
	contour.push_back(0);
	contour.push_back(3);
	contour.push_back(1);
	contour.push_back(2);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 5.
	face.clear();
	contour.clear();
	contour.push_back(0);
	contour.push_back(2);
	contour.push_back(1);
	contour.push_back(3);
	face.push_back(contour);
	SN.push_back(face);

	///

	CreateSurfNetModel(A, AID, 6, M, SN, minSamplingDensity, q, pModel, bVisualize);

	RVLCOPY3VECTOR(size, pModel->bboxSize);
	RVLNULL3VECTOR(pModel->bboxCenter);
	pModel->points.Element = new OrientedPoint[pModel->points_.n];
	pModel->points.n = pModel->points_.n;
	int i, j, iPt;
	OrientedPoint *pPt;
	RECOG::DDD::ModelPoint *pPt_;
	float *mxRow;
	for (iPt = 0; iPt < pModel->points_.n; iPt++)
	{
		pPt = pModel->points.Element + iPt;
		pPt_ = pModel->points_.Element + iPt;
		RVLMULMXVECT(pPt_->S, q, 3, 7, pPt->P, i, j, mxRow);
		RVLCOPY3VECTOR(pPt_->N, pPt->N);
	}
}

void DDDetector::CreateStorageVolumeModel(
	RECOG::DDD::Model *pModel,
	bool bVisualize)
{
	// Parameters.

	float q[7] = {0.0, 0.3f, 0.4f, 0.2f, 0.0f, 0.0f, 0.0f}; // m
	q[0] = storageVolumeWallThickness;
	float minSamplingDensity = 1.0f / 0.05f; // m^-1

	/// SurfNet model.

	float A[3 * 6];
	BoxNormals(A);
	int AID[11] = {0, 1, 2, 3, 4, 5, 1, 3, 2, 5, 4};
	float M[11 * 7] = {
		0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
		1.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		1.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0,
		1.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0,
		1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0,
		1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0};
	std::vector<std::vector<std::vector<int>>> SN;
	std::vector<std::vector<int>> face;
	std::vector<int> contour;
	// Surface 0.
	contour.push_back(2);
	contour.push_back(5);
	contour.push_back(3);
	contour.push_back(4);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 1.
	face.clear();
	contour.clear();
	contour.push_back(2);
	contour.push_back(4);
	contour.push_back(3);
	contour.push_back(5);
	face.push_back(contour);
	contour.clear();
	contour.push_back(7);
	contour.push_back(10);
	contour.push_back(8);
	contour.push_back(9);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 2.
	face.clear();
	contour.clear();
	contour.push_back(0);
	contour.push_back(4);
	contour.push_back(1);
	contour.push_back(5);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 3.
	face.clear();
	contour.clear();
	contour.push_back(0);
	contour.push_back(5);
	contour.push_back(1);
	contour.push_back(4);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 4.
	face.clear();
	contour.clear();
	contour.push_back(0);
	contour.push_back(3);
	contour.push_back(1);
	contour.push_back(2);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 5.
	face.clear();
	contour.clear();
	contour.push_back(0);
	contour.push_back(2);
	contour.push_back(1);
	contour.push_back(3);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 6.
	face.clear();
	contour.clear();
	contour.push_back(7);
	contour.push_back(9);
	contour.push_back(8);
	contour.push_back(10);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 7.
	face.clear();
	contour.clear();
	contour.push_back(1);
	contour.push_back(9);
	contour.push_back(6);
	contour.push_back(10);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 8.
	face.clear();
	contour.clear();
	contour.push_back(1);
	contour.push_back(10);
	contour.push_back(6);
	contour.push_back(9);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 9.
	face.clear();
	contour.clear();
	contour.push_back(1);
	contour.push_back(8);
	contour.push_back(6);
	contour.push_back(7);
	face.push_back(contour);
	SN.push_back(face);
	// Surface 10.
	face.clear();
	contour.clear();
	contour.push_back(1);
	contour.push_back(7);
	contour.push_back(6);
	contour.push_back(8);
	face.push_back(contour);
	SN.push_back(face);

	///

	CreateSurfNetModel(A, AID, 11, M, SN, minSamplingDensity, q, pModel, bVisualize);
}

void DDDetector::CreateBox(
	Mesh *pMesh,
	float size[3])
{
	// Vertices.

	float halfSize[3];
	RVLSCALE3VECTOR(size, 0.5f, halfSize);
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertPoint(0, halfSize[0], -halfSize[1], -halfSize[2]);
	points->InsertPoint(1, -halfSize[0], -halfSize[1], -halfSize[2]);
	points->InsertPoint(2, -halfSize[0], halfSize[1], -halfSize[2]);
	points->InsertPoint(3, halfSize[0], halfSize[1], -halfSize[2]);
	points->InsertPoint(4, halfSize[0], -halfSize[1], halfSize[2]);
	points->InsertPoint(5, halfSize[0], halfSize[1], halfSize[2]);
	points->InsertPoint(6, -halfSize[0], halfSize[1], halfSize[2]);
	points->InsertPoint(7, -halfSize[0], -halfSize[1], halfSize[2]);

	// Faces.

	int faces_[6][4] = {{0, 1, 2, 3}, {0, 3, 5, 4}, {3, 2, 6, 5}, {2, 1, 7, 6}, {1, 0, 4, 7}, {4, 5, 6, 7}};
	vtkSmartPointer<vtkCellArray> faces = vtkSmartPointer<vtkCellArray>::New();
	int iFace;
	for (iFace = 0; iFace < 6; iFace++)
	{
		vtkSmartPointer<vtkIdList> face = vtkSmartPointer<vtkIdList>::New();
		for (int i = 0; i < 4; i++)
			face->InsertNextId(faces_[iFace][i]);
		faces->InsertNextCell(face);
	}

	// Create poly data.

	pMesh->pPolygonData = vtkSmartPointer<vtkPolyData>::New();
	pMesh->pPolygonData->SetPoints(points);
	pMesh->pPolygonData->SetPolys(faces);
}

void DDDetector::CreateCylinder(
	Mesh *pMesh,
	float r,
	float h,
	int resolution)
{
	// Vertices.

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	float fResolution = (float)resolution;
	float dPhi = 2 * PI / fResolution;
	int i;
	float phi;
	float hhalf = 0.5f * h;
	for (i = 0; i < resolution; i++)
	{
		phi = (float)i * dPhi;
		points->InsertPoint(i, r * cos(phi), r * sin(phi), -hhalf);
		points->InsertPoint(resolution + i, r * cos(phi), r * sin(phi), hhalf);
	}

	// Faces.

	vtkSmartPointer<vtkCellArray> faces = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkIdList> topFace = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> bottomFace = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> sideFace = vtkSmartPointer<vtkIdList>::New();
	for (i = 0; i < resolution; i++)
	{
		topFace->InsertNextId(resolution + i);
		bottomFace->InsertNextId(resolution - i - 1);
		sideFace = vtkSmartPointer<vtkIdList>::New();
		sideFace->InsertNextId(i);
		sideFace->InsertNextId((i + 1) % resolution);
		sideFace->InsertNextId(resolution + (i + 1) % resolution);
		sideFace->InsertNextId(resolution + i);
		faces->InsertNextCell(sideFace);
	}
	faces->InsertNextCell(topFace);
	faces->InsertNextCell(bottomFace);

	// Create poly data.

	pMesh->pPolygonData = vtkSmartPointer<vtkPolyData>::New();
	pMesh->pPolygonData->SetPoints(points);
	pMesh->pPolygonData->SetPolys(faces);
}

void DDDetector::CreateLHTCPModel(
	Mesh *pMesh,
	float r1,
	float h1,
	float r2,
	float h2,
	int resolution)
{
	// Vertices.

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	float fResolution = (float)resolution;
	float dPhi = 2 * PI / fResolution;
	int i;
	float phi;
	float hhalf1 = 0.5f * h1;
	// float hhalf2 = 0.5f * h2;
	for (i = 0; i < resolution; i++)
	{
		phi = (float)i * dPhi;
		points->InsertPoint(i, r1 * cos(phi), r1 * sin(phi), -hhalf1);
		points->InsertPoint(resolution + i, r1 * cos(phi), r1 * sin(phi), hhalf1);

		// Second cylinder

		points->InsertPoint(2 * resolution + i, r2 * cos(phi), r2 * sin(phi), hhalf1);
		points->InsertPoint(3 * resolution + i, r2 * cos(phi), r2 * sin(phi), hhalf1 + h2);
	}

	// Faces.

	vtkSmartPointer<vtkCellArray> faces = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkIdList> topFace1 = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> bottomFace1 = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> topFace2 = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> bottomFace2 = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> sideFace1 = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> sideFace2 = vtkSmartPointer<vtkIdList>::New();
	for (i = 0; i < resolution; i++)
	{
		topFace1->InsertNextId(resolution + i);
		bottomFace1->InsertNextId(resolution - i - 1);

		sideFace1 = vtkSmartPointer<vtkIdList>::New();
		sideFace1->InsertNextId(i);									// 0
		sideFace1->InsertNextId((i + 1) % resolution);				// 12
		sideFace1->InsertNextId(resolution + (i + 1) % resolution); // 25
		sideFace1->InsertNextId(resolution + i);					// 13
		faces->InsertNextCell(sideFace1);

		topFace2->InsertNextId(3 * resolution + i);
		bottomFace2->InsertNextId(3 * resolution - i - 1);

		sideFace2 = vtkSmartPointer<vtkIdList>::New();
		sideFace2->InsertNextId(2 * resolution + i);					// 26
		sideFace2->InsertNextId(2 * resolution + (i + 1) % resolution); // 38
		sideFace2->InsertNextId(3 * resolution + (i + 1) % resolution); // 51
		sideFace2->InsertNextId(3 * resolution + i);					// 39

		faces->InsertNextCell(sideFace2);
	}
	faces->InsertNextCell(topFace1);
	faces->InsertNextCell(bottomFace1);
	faces->InsertNextCell(topFace2);
	faces->InsertNextCell(bottomFace2);

	// Create poly data.

	pMesh->pPolygonData = vtkSmartPointer<vtkPolyData>::New();
	pMesh->pPolygonData->SetPoints(points);
	pMesh->pPolygonData->SetPolys(faces);
}

void DDDetector::RotateRectStruct(
	RECOG::DDD::RectStruct *pMRectStruct,
	int *iMAxisMap,
	float *dirMAxisMap,
	int *iQAxisMap,
	Array<RECOG::DDD::Rect3D> &MRects)
{
	int iMRect, iMAxis;
	RECOG::DDD::Rect3D *pMRect = MRects.Element;
	RECOG::DDD::Rect3D *pMRect_ = pMRectStruct->rects.Element;
	for (iMRect = 0; iMRect < MRects.n; iMRect++, pMRect++, pMRect_++)
	{
		for (iMAxis = 0; iMAxis < 3; iMAxis++)
			pMRect->c[iMAxisMap[iMAxis]] = pMRect_->c[iMAxis] * dirMAxisMap[iMAxis];
		pMRect->iAxis = iMAxisMap[pMRect_->iAxis];
		if ((pMRect_->iAxis + 1) % 3 == iQAxisMap[(pMRect->iAxis + 1) % 3])
		{
			pMRect->s[0] = pMRect_->s[0];
			pMRect->s[1] = pMRect_->s[1];
		}
		else
		{
			pMRect->s[1] = pMRect_->s[0];
			pMRect->s[0] = pMRect_->s[1];
		}
		pMRect->direction = pMRect_->direction * dirMAxisMap[pMRect_->iAxis];
		pMRect->iSurfel = pMRect_->iSurfel;
		pMRect->iParent = pMRect_->iParent;
	}
}

void DDDetector::LoadModels(std::vector<std::string> modelFileNames)
{
	int iModel;
	RVL_DELETE_ARRAY(models.Element);
	models.n = modelFileNames.size();
	models.Element = new RECOG::DDD::Model[models.n];
	RECOG::DDD::Model *pModel;
	for (iModel = 0; iModel < modelFileNames.size(); iModel++)
	{
		std::string modelInFileName = modelFileNames[iModel];
		std::string modelOutFileName = modelInFileName.substr(0, modelInFileName.rfind('.') + 1) + "dat";
		FILE *fpModel = fopen(modelOutFileName.data(), "rb");
		pModel = models.Element + iModel;
		fread(&(pModel->points.n), sizeof(int), 1, fpModel);
		pModel->points.Element = new OrientedPoint[pModel->points.n];
		fread(pModel->points.Element, sizeof(OrientedPoint), pModel->points.n, fpModel);
		fclose(fpModel);
		Box<float> modelBBox;
		InitBoundingBox<float>(&modelBBox, pModel->points.Element[0].P);
		for (int iMPt = 1; iMPt < pModel->points.n; iMPt++)
			UpdateBoundingBox<float>(&modelBBox, pModel->points.Element[iMPt].P);
		BoxSize<float>(&modelBBox, pModel->bboxSize[0], pModel->bboxSize[1], pModel->bboxSize[2]);
		BoxCenter<float>(&modelBBox, pModel->bboxCenter);
	}
}

void CreateOrientedPointArrayFromPointArray(
	Array<Point> PtArraySrc,
	Array<OrientedPoint> &PtArrayTgt)
{
	if (PtArrayTgt.Element == NULL)
		PtArrayTgt.Element = new OrientedPoint[PtArraySrc.n];
	PtArrayTgt.n = PtArraySrc.n;
	int iPt;
	OrientedPoint *pPtTgt;
	Point *pPtSrc;
	for (iPt = 0; iPt < PtArraySrc.n; iPt++)
	{
		pPtSrc = PtArraySrc.Element + iPt;
		pPtTgt = PtArrayTgt.Element + iPt;
		RVLCOPY3VECTOR(pPtSrc->P, pPtTgt->P);
		RVLCOPY3VECTOR(pPtSrc->N, pPtTgt->N);
	}
}

void CreateOrientedPointArrayFromPointArray(
	Array<Point> PtArraySrc,
	Array<int> PtIdx,
	Array<OrientedPoint> &PtArrayTgt)
{
	if (PtArrayTgt.Element == NULL)
		PtArrayTgt.Element = new OrientedPoint[PtIdx.n];
	PtArrayTgt.n = PtIdx.n;
	int iPt;
	OrientedPoint *pPtTgt = PtArrayTgt.Element;
	Point *pPtSrc;
	for (int i = 0; i < PtIdx.n; i++, pPtTgt++)
	{
		iPt = PtIdx.Element[i];
		pPtSrc = PtArraySrc.Element + iPt;
		RVLCOPY3VECTOR(pPtSrc->P, pPtTgt->P);
		RVLCOPY3VECTOR(pPtSrc->N, pPtTgt->N);
	}
}

void DDDetector::Detect(
	Array<Mesh> meshSeq,
	RECOG::DDD::HypothesisDoorDrawer *pFinalHyp,
	char *hypFileName,
	std::vector<cv::Mat> *pRGBSeq)
{
	// Parameters.

	float maxDoorStateChange = 65.0f;	// deg
	float maxDrawerStateChange = 0.25f; // m
	float minDoorStateChange = 10.0f;	// deg
	float minDrawerStateChange = 0.05f; // m
	float minDoorStateRange = 45.0f;	// deg
	float minDrawerStateRange = 0.15f;	// m

	// Constants.

	float maxDoorStateChangeRad = maxDoorStateChange * DEG2RAD;
	float minDoorStateChangeRad = minDoorStateChange * DEG2RAD;
	float minDoorStateRangeRad = minDoorStateRange * DEG2RAD;

	/// Moving part hypotheses.

	RECOG::DDD::Model *pModel = models.Element;
	std::vector<RECOG::DDD::Hypothesis> hyps;
	RECOG::DDD::Hypothesis hyp;
	int iHyp, iMesh, iMesh_;
	Mesh *pMesh;
	if (bLoadMovingPartHypothesesFromFile)
	{
		// Load moving part hypotheses.

		if (hypFileName == NULL)
		{
			printf("ERROR: Hypothesis file name is not defined!\n");
			return;
		}
		printf("Loading moving part hypotheses from file %s.\n", hypFileName);
		FILE *fpHyps = fopen(hypFileName, "r");
		if (fpHyps == NULL)
		{
			printf("ERROR: Cannot open hypothesis file %s!\n", hypFileName);
			return;
		}
		while (fscanf(fpHyps, "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", &(hyp.imgID),
					  hyp.pose.R, hyp.pose.R + 1, hyp.pose.R + 2, hyp.pose.R + 3, hyp.pose.R + 4, hyp.pose.R + 5, hyp.pose.R + 6, hyp.pose.R + 7, hyp.pose.R + 8,
					  hyp.pose.t, hyp.pose.t + 1, hyp.pose.t + 2,
					  hyp.pose.s, hyp.pose.s + 1, hyp.pose.s + 2) == 16)
		{
			hyp.bboxSize[0] = pModel->bboxSize[2];
			hyp.bboxSize[1] = pModel->bboxSize[1] * hyp.pose.s[1];
			hyp.bboxSize[2] = pModel->bboxSize[0] * hyp.pose.s[2];
			hyp.assoc.Element = NULL;
			hyps.push_back(hyp);
		}
		fclose(fpHyps);
	}
	else
	{
		// ROI.

		// DEBUG
		Box<float> ROI;
		ROI.minx = -0.300;
		ROI.maxx = 0.200;
		ROI.miny = 0.150;
		ROI.maxy = 0.400;
		ROI.minz = 0.900;
		ROI.maxz = 1.600;
		Pose3D ROIPose;
		RVLUNITMX3(ROIPose.R);
		RVLNULL3VECTOR(ROIPose.t);
		Array<int> SOI;
		int SOIMem;
		SOI.Element = &SOIMem;
		SOI.n = 1;
		// SOI.Element[0] = 762;
		SOI.Element[0] = 2855;
		// Array<int>* pSOI = (test == RVLDDD_TEST_DDD2 ? &SOI : NULL);
		Array<int> *pSOI = NULL;
		float invRFR[9];
		float *X = invRFR;
		float *Y = invRFR + 3;
		float *Z = invRFR + 6;
		float fTmp;
		Pose3D RF;
		//// FERIT, 1. drawer
		// RVLSET3VECTOR(Z, 0.500f, 0.480f, -0.721f);
		// RVLSET3VECTOR(Y, -0.009f, -0.809f, -0.588f);
		// RVLSET3VECTOR(RF.t, 0.260f, 0.177f, 1.411f);
		//  IRI, half
		// RVLSET3VECTOR(Z, 0.168, 0.379, -0.910);
		// RVLSET3VECTOR(Y, -0.166, -0.787, -0.595);
		// RVLSET3VECTOR(RF.t, 0.220, -0.028, 1.558);
		//  IRI, open
		RVLSET3VECTOR(Z, -0.937, 0.158, -0.310);
		RVLSET3VECTOR(Y, 0.082, -0.507, -0.858);
		RVLSET3VECTOR(RF.t, 0.407, -0.032, 1.812);
		//
		RVLNORM3(Z, fTmp);
		RVLCROSSPRODUCT3(Y, Z, X);
		RVLNORM3(X, fTmp);
		RVLCROSSPRODUCT3(Z, X, Y);
		RVLCOPYMX3X3T(invRFR, RF.R);
		// Pose3D* pRF = (test == RVLDDD_TEST_DDD2 ? &RF : NULL);
		Pose3D *pRF = NULL;

		// Direction.

		float U[3];
		// RVLSET3VECTOR(U, 0.0f, 0.0f, -1.0f);
		RVLSET3VECTOR(U, 0.0f, COS45, -COS45);

		//

		pVisualizationData->bPointToPlane = true;

		Mesh *pMesh_;
		PointAssociationData pointAssociationData;
		Array<int> ptBuff;
		ptBuff.Element = new int[camera.w * camera.h];
		Array<Point> pointsMS;
		pointsMS.Element = new Point[camera.w * camera.h];
		Array<OrientedPoint> pointsS;
		pointsS.Element = new OrientedPoint[camera.w * camera.h];
		int *pointSIdx = new int[camera.w * camera.h];
		AffinePose3D APoseRef, APose;
		float *t = new float[3 * meshSeq.n];
		float *t_ = t + 3 * (meshSeq.n - 1);
		RVLNULL3VECTOR(t_);
		Array<int> dominantShiftPointsIdx;
		dominantShiftPointsIdx.Element = new int[0];
		int ROIStep;
		int nHypsPrev = 0;
		int iFirstOfLastDetectedMotionHyps, iLastDetectedMotionHypsEnd;
		iFirstOfLastDetectedMotionHyps = iLastDetectedMotionHypsEnd = 0;
		std::vector<AffinePose3D> ROIs;
		AffinePose3D ROI_;
		bool bMovingSurfelDetected;
		Vector3<float> dominantShift;
		// Calculate ROI from dominant ShiftPoints
		// pMesh = meshSeq.Element + (meshSeq.n - 1);
		// CalculateROI(pMesh, dominantShiftPointsIdx, &ROI);

		for (iMesh = meshSeq.n - 1; iMesh >= 0; iMesh--)
		{
			ROIs.clear();
			printf("Image %d\n", iMesh);
			pMesh = meshSeq.Element + iMesh;
			ROIStep = ROICalculationStep;
			dominantShiftPointsIdx.n = 0;
			nHypsPrev = hyps.size();
			bMovingSurfelDetected = false;
			while (!bMovingSurfelDetected)
			{
				iMesh_ = iMesh + ROIStep;
				if (iMesh_ >= 0 && iMesh_ < meshSeq.n)
				{
					pMesh_ = meshSeq.Element + iMesh_;
					printf("Detecting dominant shift between images %d and %d... ", iMesh, iMesh_);
					// if (ROIStep > iMesh || ROIStep > maxROIStep)
					//	break;
					// printf("Detecting dominant shift between images %d and %d... ", iMesh, iMesh - ROIStep);
					// pMesh_ = pMesh - ROIStep++;
					DetectDominantShiftPoints(pMesh, pMesh_, &dominantShiftPointsIdx, dominantShift, true);
					printf("Number of dominant shift points = %d\n", dominantShiftPointsIdx.n);
					if (dominantShiftPointsIdx.n >= minPointsWithDominantShift)
						bMovingSurfelDetected = GenerateHypotheses(pMesh, ROIs, &dominantShiftPointsIdx, &dominantShift, hyps, NULL, pSOI, pRF);
				}
				if (ROIStep > 0)
					ROIStep = -ROIStep;
				else
				{
					ROIStep = -ROIStep + 1;
					if (ROIStep > maxROIStep)
						break;
				}
			}
			// if (dominantShiftPointsIdx.n >= minPointsWithDominantShift)
			//{
			//	CalculateROI(pMesh, dominantShiftPointsIdx, &ROI);
			//	RVLSET3VECTOR(ROI_.s, ROI.maxx - ROI.minx, ROI.maxy - ROI.miny, ROI.maxz - ROI.minz);
			//	RVLSET3VECTOR(ROI_.t, 0.5f * (ROI.maxx + ROI.minx), 0.5f * (ROI.maxy + ROI.miny), 0.5f * (ROI.maxz + ROI.minz));
			//	RVLUNITMX3(ROI_.R);
			//	ROIs.push_back(ROI_);
			// }
			// else
			//	printf("There is no enough dominant shift points.\n");

			if (bMovingSurfelDetected)
			{
				iFirstOfLastDetectedMotionHyps = nHypsPrev;
				iLastDetectedMotionHypsEnd = hyps.size();
			}
			else
			{
				// if (ROIs.size() == 0)
				//{
				//	printf("No hypotheses are generated.\n");
				//	continue;
				// }
				// else
				{
					for (iHyp = iFirstOfLastDetectedMotionHyps; iHyp < iLastDetectedMotionHypsEnd; iHyp++)
					{
						hyp = hyps[iHyp];
						RVLCOPYMX3X3(hyp.pose.R, ROI_.R);
						RVLCOPY3VECTOR(hyp.pose.t, ROI_.t);
						RVLCOPY3VECTOR(hyp.bboxSize, ROI_.s);
						ROI_.s[0] += 0.25f;
						ROI_.s[1] += 0.25f;
						ROI_.s[2] += 0.25f;
						ROIs.push_back(ROI_);
					}
					GenerateHypotheses(pMesh, ROIs, NULL, NULL, hyps, NULL, pSOI, pRF);
				}
			}
			// GenerateHypotheses(pMesh, ROI, hyps, U, pSOI, pRF);
			if (pVisualizationData->bVisualizeInitialHypothesis && GetRGBImageVisualization() && pRGBSeq)
			{
				cv::Mat display;
				(*pRGBSeq)[iMesh].copyTo(display);
				if (pVisualizationData->bVisualizeSurfels)
					planarSurfaces.DisplayRGB(display);
				for (iHyp = nHypsPrev; iHyp < hyps.size(); iHyp++)
				{
					hyp = hyps[iHyp];
					Pose3D poseBC;
					RVLCOPY3DPOSE(hyp.pose.R, hyp.pose.t, poseBC.R, poseBC.t);
					VisualizeBoxFrontFaceRGB(poseBC, hyp.bboxSize, &display);
				}
				cv::imshow("Surfels", display);
				cv::waitKey();
			}

			// GenerateHypotheses(pMesh, ROI, ROIPose, NULL, hyps, NULL, pSOI, pRF);
			if (hyps.size() == nHypsPrev)
			{
				printf("No hypotheses are generated.\n\n");
				continue;
			}

			for (iHyp = nHypsPrev; iHyp < hyps.size(); iHyp++)
				hyps[iHyp].imgID = iMesh;

			// Random perturbance of affine pose for debugging purpose.

			// float maxOrientationPerturbance = 20.0;	// deg
			// float maxPositionPerturbance = 0.1f; // m
			// float maxScalePerturbance = 0.3f;
			// srand(time(NULL));
			// float u[3];
			// u[0] = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
			// u[1] = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
			// u[2] = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
			// float fTmp;
			// RVLNORM3(u, fTmp);
			// float th = maxOrientationPerturbance * DEG2RAD * (2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f);
			// float dR[9];
			// AngleAxisToRot<float>(u, th, dR);
			// float newR[9];
			// RVLMXMUL3X3(dR, hyp.pose.R, newR);
			// RVLCOPYMX3X3(newR, hyp.pose.R);
			// float dt[3];
			// dt[0] = maxPositionPerturbance * (2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f);
			// dt[1] = maxPositionPerturbance * (2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f);
			// dt[2] = maxPositionPerturbance * (2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f);
			// RVLSUM3VECTORS(hyp.pose.t, dt, hyp.pose.t);
			// float ds[3];
			// ds[0] = maxScalePerturbance * (2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f) + 1.0f;
			// ds[1] = maxScalePerturbance * (2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f) + 1.0f;
			// ds[2] = maxScalePerturbance * (2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f) + 1.0f;
			// RVLSCALE3VECTOR3(hyp.pose.s, ds, hyp.pose.s);

			if (pVisualizationData->b3DVisualization && pVisualizationData->bVisualizeInitialHypothesis)
			{
				pVisualizationData->pVisualizer->SetMesh(pMesh);
				for (iHyp = nHypsPrev; iHyp < hyps.size(); iHyp++)
				{
					// Hypothesis bounding box visualization.

					hyp = hyps[iHyp];
					VisualizeHypothesisBoundingBox(&hyp);
#ifdef NEVER
					// imgGrid.Create(pMesh->NodeArray, &camera, pointAssociationGridCellSize);
					// CreateOrientedPointArrayFromPointArray(pMesh->NodeArray, pointsS);
					ROI.minx = -0.5f * hyp.bboxSize[0];
					ROI.maxx = 0.5f * hyp.bboxSize[0];
					ROI.miny = -0.5f * hyp.bboxSize[1];
					ROI.maxy = 0.5f * hyp.bboxSize[1];
					ROI.minz = -0.5f * hyp.bboxSize[2];
					ROI.maxz = 0.5f * hyp.bboxSize[2];
					int iSPt;
					float PQROI[3];
					Point *pSPt;
					OrientedPoint *pQPt;
					int iSurfel;
					Surfel *pSurfel;
					pointsS.n = 0;
					float ZROI[3];
					RVLCOPYCOLMX3X3(hyp.pose.R, 2, ZROI);
					float V3Tmp[3];
					for (iSPt = 0; iSPt < pMesh->NodeArray.n; iSPt++)
					{
						// if (iSPt == 155955)
						//	int debug = 0;
						if (!sampleMask[iSPt])
							continue;
						iSurfel = pSurfels->surfelMap[iSPt];
						// if (iSurfel == 1118)
						//	int debug = 0;
						if (iSurfel < 0 || iSurfel >= pSurfels->NodeArray.n)
							continue;
						pSPt = pMesh->NodeArray.Element + iSPt;
						RVLINVTRANSF3(pSPt->P, hyp.pose.R, hyp.pose.t, PQROI, V3Tmp);
						if (InBoundingBox<float>(&ROI, PQROI))
						{
							pQPt = pointsS.Element + pointsS.n;
							RVLCOPY3VECTOR(pSPt->P, pQPt->P);
							RVLCOPY3VECTOR(pSPt->N, pQPt->N);
							pSurfel = pSurfels->NodeArray.Element + iSurfel;
							if (pSurfel->bEdge)
							{
								RVLCROSSPRODUCT3(pSurfel->V, ZROI, V3Tmp);
								fTmp = RVLDOTPRODUCT3(V3Tmp, V3Tmp);
								if (fTmp >= 1e-6)
								{
									RVLSCALE3VECTOR2(V3Tmp, fTmp, pQPt->N);
								}
							}
							pointSIdx[pointsS.n] = iSPt;
							pointsS.n++;
						}
					}
					// pointAssociationData.Create(pModel->points.n, pointsS.n, true);
					pointAssociationData.Create(pModel->nSurfaces, pointsS.n, true);
					// PointAssociation(pModel->points, &(hyp.pose), pointsS, pointAssociationData, ptBuff, true, &pointsMS);
					SetSceneForHypothesisVisualization(pMesh);
					SetBBoxForHypothesisVisualization(&hyp);
					// if (iMesh == meshSeq.n - 1)
					{
						// AICP(pModel->points, pointsS, hyp.pose, nICPIterations, APoseRef, pointAssociationData, &pointsMS);
						RICP(pModel, pointsS, hyp.pose, nICPIterations, APoseRef, pointAssociationData, &pointsMS);
						APose = APoseRef;
					}
					// else
					//{
					//	TICP(pModel->points, pointsS, APose, nICPIterations, APose, pointAssociationData, &pointsMS);
					//	t_ = t + 3 * iMesh;
					//	RVLCOPY3VECTOR(APose.t, t_);
					// }

					// Change axes: z becomes x and -x becomes z.
					// If the new z axis is not facing the camera, then its direction is changed to the opposite direction.
					// Scale the model bbox with the APose scale.
					// Set the fixed box size in x-direction.

					int axisCuboidToAO[6] = {5, 4, 2, 3, 0, 1};
					// int axisCuboidToAO[6] = { 0, 1, 2, 3, 4, 5 };
					float RSM[9];
					float *XMS = RSM;
					float *YMS = XMS + 3;
					float *ZMS = YMS + 3;
					RVLCOPYCOLMX3X3(APose.R, 2, XMS);
					RVLCOPYCOLMX3X3(APose.R, 0, ZMS);
					RVLNEGVECT3(ZMS, ZMS);
					RVLCROSSPRODUCT3(ZMS, XMS, YMS);
					RVLCOPYMX3X3T(RSM, hyp.pose.R);
					hyp.bboxSize[0] = pModel->bboxSize[2];
					hyp.bboxSize[1] = pModel->bboxSize[1] * APose.s[1];
					hyp.bboxSize[2] = pModel->bboxSize[0] * APose.s[0];
					float xCorrection = 0.5f * pModel->bboxSize[2] * (APose.s[2] - 1.0f);
					RVLSCALE3VECTOR(XMS, xCorrection, V3Tmp);
					RVLCOPY3VECTOR(APose.t, hyp.pose.t);
					RVLSUM3VECTORS(hyp.pose.t, V3Tmp, hyp.pose.t);
					hyp.pose.s[0] = 1.0f;
					hyp.pose.s[1] = APose.s[1];
					hyp.pose.s[2] = APose.s[0];
					hyp.assoc.Element = new Pair<int, int>[pointsS.n];
					int i;
					int iMSurf;
					int iAssoc = 0;
					Pair<int, int> assoc;
					for (i = 0; i < pointsS.n; i++)
					{
						iMSurf = pointAssociationData.MNN[i];
						if (iMSurf < 0)
							continue;
						assoc.a = pointSIdx[i];
						assoc.b = axisCuboidToAO[iMSurf];
						hyp.assoc.Element[iAssoc++] = assoc;
					}
					hyp.assoc.n = iAssoc;
#endif
					// Visualize hypothesis.

					// VisualizeHypothesisBoundingBox(&hyp, pMesh);
					// VisualizeHypothesisBoundingBox(&hyp);
					// int j, k;
					// Array<Point> boxVertices;
					// boxVertices.Element = new Point[8];
					// boxVertices.n = 0;
					// Point* pPt;
					// float PC[3], PM[3];
					// for(k = -1; k <= 1; k += 2)
					//	for (j = -1; j <= 1; j += 2)
					//		for (i = -1; i <= 1; i += 2)
					//		{
					//			pPt = boxVertices.Element + boxVertices.n;
					//			RVLSET3VECTOR(PC, i, j, k);
					//			RVLSCALE3VECTOR(PC, 0.5f, PC);
					//			RVLSCALE3VECTOR3(PC, hyp.bboxSize, PM);
					//			RVLTRANSF3(PM, hyp.pose.R, hyp.pose.t, pPt->P);
					//			boxVertices.n++;
					//		}
					// uchar cyan[] = {0, 255, 255};
					// pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(boxVertices, cyan, 10.0f);
					// delete[] boxVertices.Element;
				}
				pVisualizationData->pVisualizer->Run();
				pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
				// pointAssociationData.Clear();
			}
		}

		delete[] ptBuff.Element;
		delete[] pointsMS.Element;
		delete[] pointsS.Element;
		delete[] t;
		delete[] pointSIdx;

		if (hypFileName)
		{
			// Save hypotheses.

			FILE *fpHyps = fopen(hypFileName, "w");
			if (fpHyps)
			{
				for (iHyp = 0; iHyp < hyps.size(); iHyp++)
				{
					hyp = hyps[iHyp];
					fprintf(fpHyps, "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", hyp.imgID,
							hyp.pose.R[0], hyp.pose.R[1], hyp.pose.R[2], hyp.pose.R[3], hyp.pose.R[4], hyp.pose.R[5], hyp.pose.R[6], hyp.pose.R[7], hyp.pose.R[8],
							hyp.pose.t[0], hyp.pose.t[1], hyp.pose.t[2],
							hyp.pose.s[0], hyp.pose.s[1], hyp.pose.s[2]);
				}
				fclose(fpHyps);
			}
		}
	}

	//// Door/drawer hypotheses. Integration of moving part hypotheses.

	RECOG::DDD::HypothesisDoorDrawer movingPartHyp;
	int m;
	std::vector<RECOG::DDD::HypothesisDoorDrawer> movingPartHyps;
	std::string hypFileName_;
	if (hypFileName)
		hypFileName_ = hypFileName;
	RECOG::DDD::Model *movingPartModel = NULL;
	int *SMCorrespondence = NULL;
	if (bLoadDDHypothesesFromFile)
	{
		// Load door/drawer hypotheses.

		std::string DDHypFileName = hypFileName_.substr(0, hypFileName_.rfind(RVLFILEPATH_SEPARATOR) + 1) + "DD.txt";
		FILE *fp = fopen(DDHypFileName.data(), "r");
		if (fp)
		{
			LoadDD(fp, movingPartHyps);
			fclose(fp);
		}
		else
			printf("ERROR: Cannot open door/drawer hypothesis file %s!\n", DDHypFileName);

		// Load door/drawer hypothesis states.

		std::string statesFileName = hypFileName_.substr(0, hypFileName_.rfind(RVLFILEPATH_SEPARATOR) + 1) + "states.txt";
		FILE *fpStates = fopen(statesFileName.data(), "r");
		if (fpStates)
		{
			LoadDDStates(fpStates, movingPartHyps);
			fclose(fpStates);
		}
		else
			printf("ERROR: Cannot open door/drawer state hypothesis file %s!\n", statesFileName);
	}
	else
	{
		// Generate door/drawer hypotheses.

		printf("Generating door hypotheses...");
		GenerateDoorHypotheses(&hyps, movingPartHyps);
		printf("completed. %d door hypotheses generated.\n\n", movingPartHyps.size());
		int nDoors = movingPartHyps.size();
		printf("Generating drawer hypotheses...");
		GenerateDrawerHypotheses(&hyps, movingPartHyps);
		printf("completed. %d drawer hypotheses generated.\n\n", movingPartHyps.size() - nDoors);

		/// Evaluate door/drawer hypotheses.

		printf("\n\n*** Hypothesis evaluation ***\n\n");
		movingPartModel = new RECOG::DDD::Model[movingPartHyps.size()];
		float cuboidSize[3];
		cuboidSize[0] = pModel->bboxSize[2];
		for (iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
		{
			movingPartHyp = movingPartHyps[iHyp];
			cuboidSize[1] = movingPartHyp.s[0];
			cuboidSize[2] = movingPartHyp.s[1];
			CreateCuboidModel2(cuboidSize, 1.0f / 0.02f, movingPartModel + iHyp);
			for (m = 0; m < movingPartHyp.state.n; m++)
				movingPartHyp.state.Element[m].score = 0.0f;
		}
		SMCorrespondence = NULL;
		float sceneFittingScore;
		// float bestScoreForImage;
		int nTransparentPts;
		float s[7];
		memset(s, 0, 7 * sizeof(float));
		float *s_ = s + 1;
		AffinePose3D movingPartBox;
		RECOG::InitZBuffer(meshSeq.Element, sceneSamplingResolution, ZBuffer, ZBufferActivePtArray, subImageMap);
		CreateImage3x3NeighborhoodLT(ZBuffer.w, image3x3Neighborhood);
		if (SMCorrespondence == NULL)
			SMCorrespondence = new int[ZBuffer.w * ZBuffer.h];
		// int iValidState;
		for (iMesh = 0; iMesh < meshSeq.n; iMesh++)
		{
			printf("Image %d\n", iMesh);
			pMesh = meshSeq.Element + iMesh;
			pMem->Clear();
			pSurfels->Init(pMesh);
			pSurfelDetector->Init(pMesh, pSurfels, pMem);
			printf("Segmentation to surfels...");
			pSurfelDetector->Segment(pMesh, pSurfels);
			printf("completed.\n");
			int nSurfels = pSurfels->NodeArray.n;
			printf("No. of surfels = %d\n", nSurfels);
			pSurfelDetector->MergeSurfels(pMesh, pSurfels, minSurfelSize, minEdgeSize, maxCoPlanarSurfelNormalAngle, maxCoPlanarSurfelRefPtDist, &planarSurfaces);
			printf("No. of planar surfaces = %d\n", planarSurfaces.NodeArray.n);
			RVL_DELETE_ARRAY(surfelMask);
			surfelMask = new bool[planarSurfaces.NodeArray.n];
			for (int iSurfel = 0; iSurfel < planarSurfaces.NodeArray.n; iSurfel++)
				surfelMask[iSurfel] = true;
			if (pVisualizationData->bVisualizeFittingScoreCalculation)
				pVisualizationData->pVisualizer->SetMesh(pMesh->pPolygonData);
			for (iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
			{
				printf("Hypothesis %d: ", iHyp);
				movingPartHyp = movingPartHyps[iHyp];
				if (movingPartHyp.objClass == RVLDDD_MODEL_DOOR)
					printf("door\n");
				else if (movingPartHyp.objClass == RVLDDD_MODEL_DRAWER)
					printf("drawer\n");
				else
					printf("unknown object class!\n");
				// iValidState = -1;
				for (m = 0; m < movingPartHyp.state.n; m++)
				{
					if (movingPartHyp.state.Element[m].imgID == iMesh)
					{
						DDBox(&movingPartHyp, m, &movingPartBox);
						RVLSCALE3VECTOR(movingPartBox.s, 0.5f, s_);
						sceneFittingScore = EvaluateHypothesis(pMesh, movingPartModel + iHyp, 7, s, movingPartBox.R, chamferDistThr, nTransparentPts, SMCorrespondence, NULL, movingPartBox.t);
						sceneFittingScore -= (float)nTransparentPts;
						movingPartHyps[iHyp].state.Element[m].score = sceneFittingScore;
						printf("Scene fitting score: %f\t(nTransparentPts = %d)\n", sceneFittingScore, nTransparentPts);
						// if(iMesh >= 8 && (iHyp == 1 || iHyp == 7))
						if (pVisualizationData->bVisualizeFittingScoreCalculation)
							VisualizeFittingScoreCalculation(pMesh, nTransparentPts, SMCorrespondence, false);
						// if (iValidState < 0 || sceneFittingScore > bestScoreForImage)
						//{
						//	bestScoreForImage = sceneFittingScore;
						//	iValidState = m;
						// }
					}
				}
				// if (iValidState >= 0)
				//{
				//	movingPartHyp.state.Element[iValidState].bValid = true;
				//	score[iHyp] += bestScoreForImage;
				// }
			}
			printf("\n");
			pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
		}

		///

		if (hypFileName)
		{
			// Save door/drawer hypotheses.

			std::string DDHypFileName = hypFileName_.substr(0, hypFileName_.rfind(RVLFILEPATH_SEPARATOR) + 1) + "DD.txt";
			FILE *fp = fopen(DDHypFileName.data(), "w");
			std::vector<RECOG::DDD::HypothesisDoorDrawer> DDHyp;
			for (iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
				DDHyp.push_back(movingPartHyps[iHyp]);
			SaveDD(fp, DDHyp);
			fclose(fp);

			// Save door/drawer hypothesis states.

			std::string statesFileName = hypFileName_.substr(0, hypFileName_.rfind(RVLFILEPATH_SEPARATOR) + 1) + "states.txt";
			FILE *fStates = fopen(statesFileName.data(), "w");
			SaveDDStates(fStates, movingPartHyps);
			fclose(fStates);
		}
	}

	////

	// Hypothesis tracking.

	printf("Hypothesis tracking...\n");
	std::vector<RECOG::DDD::AOHypothesisStateTrack> tracks;
	RECOG::DDD::AOHypothesisStateTrack track;
	RECOG::DDD::AOHypothesisStateTrack *pTrack, *pTrack_;
	RECOG::DDD::AOHypothesisState state, state_;
	int iTrack, iNextState;
	float e;
	bool bAddStateToTrack;
	std::vector<int> tracksToExtend;
	bool bStateAddedToTrack;
	int iBestTrack;
	float bestScore;
	float score;
	int k;
	int kmin, kmax;
	RECOG::DDD::HypothesisDoorDrawer *pMovingPartHyp;
	int nTracks;
	for (iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
	{
		printf("  Hypothesis %d/%d\n", iHyp, movingPartHyps.size());
		movingPartHyp = movingPartHyps[iHyp];
		kmin = kmax = movingPartHyp.state.Element[0].imgID;
		for (m = 1; m < movingPartHyp.state.n; m++)
		{
			state = movingPartHyp.state.Element[m];
			if (state.score < 0.0f)
				continue;
			k = state.imgID;
			if (k < kmin)
				kmin = k;
			else if (k > kmax)
				kmax = k;
		}
		tracks.clear();
		for (k = kmin; k <= kmax; k++)
		{
			nTracks = tracks.size();
			for (m = 0; m < movingPartHyp.state.n; m++)
			{
				state_ = movingPartHyp.state.Element[m];
				if (state_.score < 0.0f)
					continue;
				if (state_.imgID != k)
					continue;
				bStateAddedToTrack = false;
				for (iTrack = 0; iTrack < nTracks; iTrack++)
				{
					pTrack = tracks.data() + iTrack;
					state = pTrack->states[pTrack->states.size() - 1];
					e = state.q - state_.q;
					if (movingPartHyp.objClass == RVLDDD_MODEL_DOOR)
					{
						if (e > PI)
							e -= (2.0f * PI);
						else if (e < -PI)
							e += (2.0f * PI);
						bAddStateToTrack = (RVLABS(e) <= maxDoorStateChangeRad);
					}
					else if (movingPartHyp.objClass == RVLDDD_MODEL_DRAWER)
						bAddStateToTrack = (RVLABS(e) <= maxDrawerStateChange);
					if (bAddStateToTrack)
					{
						pTrack->nextStates.push_back(state_);
						bStateAddedToTrack = true;
					}
				}
				if (!bStateAddedToTrack)
				{
					track.iHypothesis = iHyp;
					track.states.clear();
					track.nextStates.clear();
					track.states.push_back(state_);
					tracks.push_back(track);
				}
			}
			for (iTrack = 0; iTrack < tracks.size(); iTrack++)
			{
				pTrack = tracks.data() + iTrack;
				tracksToExtend.clear();
				tracksToExtend.push_back(iTrack);
				for (iNextState = 1; iNextState < pTrack->nextStates.size(); iNextState++)
				{
					track.iHypothesis = pTrack->iHypothesis;
					track.states.clear();
					track.nextStates.clear();
					std::copy(pTrack->states.begin(), pTrack->states.end(), std::back_inserter(track.states));
					tracksToExtend.push_back(tracks.size());
					tracks.push_back(track);
					pTrack = tracks.data() + iTrack;
				}
				for (iNextState = 0; iNextState < pTrack->nextStates.size(); iNextState++)
				{
					pTrack_ = tracks.data() + tracksToExtend[iNextState];
					pTrack_->states.push_back(pTrack->nextStates[iNextState]);
				}
				pTrack->nextStates.clear();
			}
		}
		bestScore = 0.0f;
		for (iTrack = 0; iTrack < tracks.size(); iTrack++)
		{
			pTrack = tracks.data() + iTrack;
			score = 0;
			for (m = 0; m < pTrack->states.size(); m++)
				score += pTrack->states[m].score;
			if (score > bestScore)
			{
				bestScore = score;
				iBestTrack = iTrack;
			}
		}
		pMovingPartHyp = movingPartHyps.data() + iHyp;
		if (bestScore > 0.0f)
		{
			pTrack = tracks.data() + iBestTrack;
			pMovingPartHyp->state.n = pTrack->states.size();
			for (m = 0; m < pMovingPartHyp->state.n; m++)
				pMovingPartHyp->state.Element[m] = pTrack->states[m];
		}
		else
			pMovingPartHyp->state.n = 0;
		pMovingPartHyp->score = bestScore;
		//}
	}
	printf("completed.\n\n\n");

	// Ignoring initial state score.
	// Eliminating static hypotheses.

	float q, q0;
	int imgID0;
	for (iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
	{
		pMovingPartHyp = movingPartHyps.data() + iHyp;
		imgID0 = -1;
		RECOG::DDD::AOHypothesisState *pState;
		float minq, maxq;
		minq = maxq = pMovingPartHyp->state.Element[0].q;
		for (m = 0; m < pMovingPartHyp->state.n; m++)
		{
			pState = pMovingPartHyp->state.Element + m;
			q = pState->q;
			if (pState->imgID < imgID0 || imgID0 < 0)
			{
				imgID0 = pState->imgID;
				q0 = q;
			}
			if (q < minq)
				minq = q;
			else if (q > maxq)
				maxq = q;
		}
		for (m = 0; m < pMovingPartHyp->state.n; m++)
		{
			q = pMovingPartHyp->state.Element[m].q;
			e = q - q0;
			if (pMovingPartHyp->objClass == RVLDDD_MODEL_DOOR)
			{
				if (e > PI)
					e -= (2.0f * PI);
				else if (e < -PI)
					e += (2.0f * PI);
				if (RVLABS(e) <= minDoorStateChangeRad)
					pMovingPartHyp->score -= pMovingPartHyp->state.Element[m].score;
			}
			else if (movingPartHyp.objClass == RVLDDD_MODEL_DRAWER)
				if (RVLABS(e) <= minDrawerStateChange)
					pMovingPartHyp->score -= pMovingPartHyp->state.Element[m].score;
		}
		float minStateRange = (pMovingPartHyp->objClass == RVLDDD_MODEL_DOOR ? minDoorStateRangeRad : minDrawerStateRange);
		if (maxq - minq < minStateRange)
			pMovingPartHyp->score = 0.0f;
	}

	// Selecting the best hypothesis.

	printf("\nAO hypothesis scores:\n");
	bestScore = 0.0f;
	int iBestHyp;
	for (iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
	// for (iHyp = 6; iHyp < movingPartHyps.size(); iHyp++)
	{
		printf("%d: %f\n", iHyp, movingPartHyps[iHyp].score);
		if (movingPartHyps[iHyp].score > bestScore)
		{
			bestScore = movingPartHyps[iHyp].score;
			iBestHyp = iHyp;
		}
	}

	if (bestScore > 0.0f)
	{
		printf("The best hypothesis is %d\n\n", iBestHyp);

		// Zeroing of the best hypothesis.

		pMovingPartHyp = movingPartHyps.data() + iBestHyp;
		int imgID0 = -1;
		RECOG::DDD::AOHypothesisState *pState;
		for (m = 0; m < pMovingPartHyp->state.n; m++)
		{
			pState = pMovingPartHyp->state.Element + m;
			if (pState->imgID < imgID0 || imgID0 < 0)
			{
				imgID0 = pState->imgID;
				q0 = pState->q;
			}
		}
		if (pMovingPartHyp->objClass == RVLDDD_MODEL_DOOR)
		{
			float RA_A[9];
			RVLROTZ(cos(q0), sin(q0), RA_A);
			float RA_C[9];
			RVLMXMUL3X3(pMovingPartHyp->pose.R, RA_A, RA_C);
			RVLCOPYMX3X3(RA_C, pMovingPartHyp->pose.R);
		}
		else if (pMovingPartHyp->objClass == RVLDDD_MODEL_DRAWER)
		{
			float V3Tmp[3];
			RVLCOPYCOLMX3X3(pMovingPartHyp->pose.R, 0, V3Tmp);
			RVLSCALE3VECTOR(V3Tmp, q0, V3Tmp);
			RVLSUM3VECTORS(pMovingPartHyp->pose.t, V3Tmp, pMovingPartHyp->pose.t);
		}
		for (m = 0; m < pMovingPartHyp->state.n; m++)
		{
			q = pMovingPartHyp->state.Element[m].q - q0;
			if (pMovingPartHyp->objClass == RVLDDD_MODEL_DOOR)
			{
				if (q < -PI)
					q += (2.0f * PI);
				else if (q > PI)
					q -= (2.0f * PI);
			}
			pMovingPartHyp->state.Element[m].q = q;
		}

		// Determining the opening direction.

		if (pMovingPartHyp->objClass == RVLDDD_MODEL_DOOR)
		{
			float sumq = 0.0f;
			for (m = 0; m < pMovingPartHyp->state.n; m++)
			{
				q = pMovingPartHyp->state.Element[m].q;
				sumq += q;
			}
			pMovingPartHyp->openingDirection = (sumq > 0.0f ? 1.0f : -1.0f);
		}
		else
			pMovingPartHyp->openingDirection = 1.0f;

		// Save the final door/drawer hypothesis.

		*pFinalHyp = *pMovingPartHyp;
		//*pFinalHyp = movingPartHyps[iBestHyp];
		// pFinalHyp->objClass = pMovingPartHyp->objClass;
		// pFinalHyp->objClass = 0;
		// pFinalHyp->pose = pMovingPartHyp->pose;
		// pFinalHyp->s[0] = pMovingPartHyp->s[0];
		// pFinalHyp->s[1] = pMovingPartHyp->s[1];
		// pFinalHyp->r[0] = pMovingPartHyp->r[0];
		// pFinalHyp->r[1] = pMovingPartHyp->r[1];
		// pFinalHyp->openingDirection = pMovingPartHyp->openingDirection;
		// pFinalHyp->state.n = 0;
		// pFinalHyp->state.Element = NULL;

		if (hypFileName)
		{
			std::string AOHypFileName = hypFileName_.substr(0, hypFileName_.rfind(RVLFILEPATH_SEPARATOR) + 1) + "DDT.txt";
			FILE *fp = fopen(AOHypFileName.data(), "w");
			std::vector<RECOG::DDD::HypothesisDoorDrawer> movingPart_;
			movingPart_.push_back(movingPartHyps[iBestHyp]);
			SaveDD(fp, movingPart_);
			fclose(fp);
		}

		// Visualize best door or drawer hypothesis.

		if (GetVisualizeDoorHypotheses())
		{
			printf("\n\n");
			// for (iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
			iHyp = iBestHyp;
			{
				movingPartHyp = movingPartHyps[iHyp];
				if (pVisualizationData->b3DVisualization)
				{
					for (iMesh = 0; iMesh < meshSeq.n; iMesh++)
					{
						pMesh = meshSeq.Element + iMesh;
						pVisualizationData->pVisualizer->SetMesh(pMesh);
						VisualizeDDHypothesis(movingPartHyp, iMesh);
						pVisualizationData->pVisualizer->Run();
						pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
					}
				}
			}
			if (GetRGBImageVisualization() && pRGBSeq)
			{
				AffinePose3D box;
				Pose3D poseBC;
				float s[3];
				cv::Mat display;
				int iState;
				for (iMesh = 0; iMesh < meshSeq.n; iMesh++)
				{
					for (iState = 0; iState < movingPartHyp.state.n; iState++)
						if (movingPartHyp.state.Element[iState].imgID == iMesh)
							break;
					if (iState >= movingPartHyp.state.n)
						continue;
					(*pRGBSeq)[iMesh].copyTo(display);
					DDBox(&movingPartHyp, iState, &box);
					RVLCOPY3DPOSE(box.R, box.t, poseBC.R, poseBC.t);
					pVisualizationData->pVisualizer->DisplayBox(frontFaceThickness, movingPartHyp.s[0], movingPartHyp.s[1], &poseBC, 255.0, 0.0, 0.0, false, 5.0f);
					RVLSET3VECTOR(s, frontFaceThickness, movingPartHyp.s[0], movingPartHyp.s[1]);
					VisualizeBoxFrontFaceRGB(poseBC, s, &display);
					cv::imshow("Surfels", display);
					cv::waitKey();
				}
			}
		}
	}
	else
		printf("\nNeither a door nor a drawer was found.\n");

	// Free memory.

	for (int iHyp = 0; iHyp < hyps.size(); iHyp++)
		RVL_DELETE_ARRAY(hyps[iHyp].assoc.Element);
	for (int iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
		delete[] movingPartHyps[iHyp].state.Element;
	if (movingPartModel)
		for (int iHyp = 0; iHyp < movingPartHyps.size(); iHyp++)
			ClearModel(movingPartModel + iHyp);
	RVL_DELETE_ARRAY(ZBuffer.Element);
	RVL_DELETE_ARRAY(ZBufferActivePtArray.Element);
	RVL_DELETE_ARRAY(subImageMap);
	RVL_DELETE_ARRAY(SMCorrespondence);
}

void DDDetector::Detect3(
	Array2D<uchar> mask,
	Array<RECOG::DDD::Line2D> *pOrthogonalViewLines,
	Array<Rect<int>> *pDDRects)
{
	// Parameters.

	int samplingRate = 5;
	int minDDSize = 20;
	int NMSThr = 5;
	float wTexture = 0.0f;
	int maskedThr = 10; // %

	// Constants.

	int w = mask.w;
	int h = mask.h;
	int nPix = h * w;
	int maxDDCost = w * h;

	// Edge image.

	cv::Mat cvHEdges(h, w, CV_8SC1);
	uchar *bHEdge = cvHEdges.data;
	memset(bHEdge, 0, nPix);
	cv::Mat cvVEdges(h, w, CV_8SC1);
	uchar *bVEdge = cvVEdges.data;
	memset(bVEdge, 0, nPix);
	RECOG::DDD::Line2D *pEdgeLine = pOrthogonalViewLines->Element;
	int iEdgeLine;
	for (iEdgeLine = 0; iEdgeLine < pOrthogonalViewLines->n; iEdgeLine++, pEdgeLine++)
		if (RVLABS(pEdgeLine->N[0]) >= RVLABS(pEdgeLine->N[1]))
			cv::line(cvVEdges, cv::Point(pEdgeLine->P[0][0], pEdgeLine->P[0][1]), cv::Point(pEdgeLine->P[1][0], pEdgeLine->P[1][1]), cv::Scalar(1));
		else
			cv::line(cvHEdges, cv::Point(pEdgeLine->P[0][0], pEdgeLine->P[0][1]), cv::Point(pEdgeLine->P[1][0], pEdgeLine->P[1][1]), cv::Scalar(1));
	cv::line(cvHEdges, cv::Point(0, 0), cv::Point(w - 1, 0), cv::Scalar(1));
	cv::line(cvVEdges, cv::Point(0, 0), cv::Point(0, h - 1), cv::Scalar(1));
	cv::line(cvHEdges, cv::Point(w - 1, 0), cv::Point(w - 1, h - 1), cv::Scalar(1));
	cv::line(cvVEdges, cv::Point(0, h - 1), cv::Point(w - 1, h - 1), cv::Scalar(1));

	// Integral images.

	int u, v, iPix, dist;

	int *leftDist = new int[nPix];
	iPix = 0;
	for (v = 0; v < h; v++)
	{
		dist = -1;
		for (u = 0; u < w; u++, iPix++)
		{
			if (bVEdge[iPix])
				dist = 0;
			else
				dist++;
			leftDist[iPix] = dist;
		}
	}

	int *rightDist = new int[nPix];
	iPix = -w - 1;
	for (v = 0; v < h; v++)
	{
		dist = -1;
		iPix += (2 * w);
		for (u = w - 1; u >= 0; u--, iPix--)
		{
			if (bVEdge[iPix])
				dist = 0;
			else
				dist++;
			rightDist[iPix] = dist;
		}
	}

	int *upDist = new int[nPix];
	for (u = 0; u < w; u++)
	{
		dist = -1;
		iPix = u;
		for (v = 0; v < h; v++, iPix += w)
		{
			if (bHEdge[iPix])
				dist = 0;
			else
				dist++;
			upDist[iPix] = dist;
		}
	}

	int *downDist = new int[nPix];
	for (u = 0; u < w; u++)
	{
		dist = -1;
		iPix = u + w * (h - 1);
		for (v = h - 1; v >= 0; v--, iPix -= w)
		{
			if (bHEdge[iPix])
				dist = 0;
			else
				dist++;
			downDist[iPix] = dist;
		}
	}

	int *nEdgePoints = new int[nPix];
	int *nMaskedPoints = new int[nPix];
	int nEdgePoints_ = 0;
	int nMaskedPoints_ = 0;
	iPix = 0;
	for (u = 0; u < w; u++, iPix++)
	{
		if (bHEdge[iPix] || bVEdge[iPix])
			nEdgePoints_++;
		nEdgePoints[iPix] = nEdgePoints_;
		if (mask.Element[iPix] == 0)
			nMaskedPoints_++;
		nMaskedPoints[iPix] = nMaskedPoints_;
	}
	for (v = 1; v < h; v++)
	{
		nEdgePoints_ = 0;
		nMaskedPoints_ = 0;
		for (u = 0; u < w; u++, iPix++)
		{
			if (bHEdge[iPix] || bVEdge[iPix])
				nEdgePoints_++;
			nEdgePoints[iPix] = nEdgePoints[iPix - w] + nEdgePoints_;
			if (mask.Element[iPix] == 0)
				nMaskedPoints_++;
			nMaskedPoints[iPix] = nMaskedPoints[iPix - w] + nMaskedPoints_;
		}
	}

	float *scoreH = new float[nPix];
	float score;
	iPix = 0;
	for (v = 0; v < h; v++)
	{
		score = 0.0f;
		for (u = 0; u < w; u++, iPix++)
		{
			dist = RVLMIN(upDist[iPix], downDist[iPix]);
			score += exp(-0.5 * (float)(dist * dist));
			scoreH[iPix] = score;
		}
	}

	float *scoreV = new float[nPix];
	for (u = 0; u < w; u++)
	{
		score = 0.0f;
		iPix = u;
		for (v = 0; v < h; v++, iPix += w)
		{
			dist = RVLMIN(leftDist[iPix], rightDist[iPix]);
			score += exp(-0.5 * (float)(dist * dist));
			scoreV[iPix] = score;
		}
	}

	// Hypothesis generation.

	Rect<int> imgRect;
	imgRect.minx = 0;
	imgRect.maxx = w - 1;
	imgRect.miny = 0;
	imgRect.maxy = h - 1;
	int samplingOffset = (samplingRate - 1) / 2;
	int maxnSamples = (w / samplingRate + 1) * (h / samplingRate + 1);
	Array<Rect<int>> DDRects;
	DDRects.Element = new Rect<int>[maxnSamples];
	Rect<int> *pRect;
	int iPixTL, iPixTR, iPixBL, iPixBR;
	float edgeScore;
	int textureCost;
	int ddWidth, ddHeight, ddArea;
	std::vector<SortIndex<float>> ddIdxSorted(maxnSamples);
	DDRects.n = 0;
	for (v = samplingOffset; v < h; v += samplingRate)
	{
		for (u = samplingOffset; u < w; u += samplingRate)
		{
			pRect = DDRects.Element + DDRects.n;
			iPix = u + v * w;
			pRect->minx = u - leftDist[iPix];
			pRect->maxx = u + rightDist[iPix];
			pRect->miny = v - upDist[iPix];
			pRect->maxy = v + downDist[iPix];
			ddWidth = pRect->maxx - pRect->minx + 1;
			ddHeight = pRect->maxy - pRect->miny + 1;
			if (ddWidth < minDDSize || ddHeight < minDDSize)
				continue;
			iPixTL = pRect->minx + pRect->miny * w;
			iPixTR = pRect->maxx + pRect->miny * w;
			iPixBL = pRect->minx + pRect->maxy * w;
			iPixBR = pRect->maxx + pRect->maxy * w;
			nMaskedPoints_ = nMaskedPoints[iPixBR - 2 - 2 * w] - nMaskedPoints[iPixBL + 2 - 2 * w] - nMaskedPoints[iPixTR - 2 + 2 * w] + nMaskedPoints[iPixTL + 2 + 2 * w];
			ddArea = ddWidth * ddHeight;
			if (100 * nMaskedPoints_ / ddArea > maskedThr)
				continue;
			edgeScore = scoreH[iPixTR] + scoreH[iPixBR] + scoreV[iPixBL] + scoreV[iPixBR];
			textureCost = nEdgePoints[iPixBR - 2 - 2 * w] - nEdgePoints[iPixBL + 2 - 2 * w] - nEdgePoints[iPixTR - 2 + 2 * w] + nEdgePoints[iPixTL + 2 + 2 * w];
			if (pRect->minx > 0)
				edgeScore -= (scoreH[iPixTL - 1] + scoreH[iPixBL - 1]);
			if (pRect->miny > 0)
				edgeScore -= (scoreV[iPixTL - w] + scoreV[iPixTR - w]);
			ddIdxSorted[DDRects.n].cost = (float)(2 * (ddWidth + ddHeight) - 4) - edgeScore + wTexture * (float)textureCost;
			ddIdxSorted[DDRects.n].idx = DDRects.n;
			DDRects.n++;
		}
	}
	ddIdxSorted.resize(DDRects.n);
	std::sort(ddIdxSorted.begin(), ddIdxSorted.end(), RECOG::DDD::SortCompare);

	// Non-maximum suppression.

	Array<int> validDDs;
	validDDs.Element = new int[DDRects.n];
	validDDs.n = 0;
	int i;
	int iDD, iDD_;
	Rect<int> *pRect_;
	Rect<int> intersection;
	int NMSThr_ = NMSThr - 1;
	for (i = 0; i < DDRects.n; i++)
	{
		iDD = ddIdxSorted[i].idx;
		pRect = DDRects.Element + iDD;
		for (iDD_ = 0; iDD_ < validDDs.n; iDD_++)
		{
			pRect_ = DDRects.Element + validDDs.Element[iDD_];
			intersection = *pRect;
			CropRect<int>(intersection, *pRect_);
			if (intersection.maxx - intersection.minx > NMSThr_ && intersection.maxy - intersection.miny > NMSThr_)
				break;
		}
		if (iDD_ < validDDs.n)
			continue;
		validDDs.Element[validDDs.n++] = iDD;
	}
	pDDRects->Element = new Rect<int>[validDDs.n];
	pDDRects->n = validDDs.n;
	for (iDD = 0; iDD < validDDs.n; iDD++)
		pDDRects->Element[iDD] = DDRects.Element[validDDs.Element[iDD]];
	delete[] DDRects.Element;
	delete[] validDDs.Element;

	// Visualization.

	cv::Mat displayImg(h, w, CV_8UC3);
	uchar *visPix = displayImg.data;
	for (iPix = 0; iPix < nPix; iPix++, visPix += 3)
		if (bHEdge[iPix] || bVEdge[iPix])
			RVLSET3VECTOR(visPix, 255, 255, 255)
		else
			RVLSET3VECTOR(visPix, 0, 0, 0)
	for (iDD = 0; iDD < pDDRects->n; iDD++)
	{
		pRect = pDDRects->Element + iDD;
		cv::rectangle(displayImg, cv::Point(pRect->minx, pRect->miny), cv::Point(pRect->maxx, pRect->maxy), cv::Scalar(0, 255, 0));
	}
	cv::imshow("detected door/drawer", displayImg);
	cv::waitKey();

	//

	delete[] leftDist;
	delete[] rightDist;
	delete[] upDist;
	delete[] downDist;
	delete[] nEdgePoints;
	delete[] nMaskedPoints;
	delete[] scoreH;
	delete[] scoreV;
}

bool DDDetector::GenerateHypotheses(
	Mesh *pMesh,
	std::vector<AffinePose3D> ROIs,
	Array<int> *dominantShiftPointsIdx,
	Vector3<float> *pDominantShift,
	std::vector<RECOG::DDD::Hypothesis> &hyps,
	float *U,
	Array<int> *pSOI,
	Pose3D *pRF)
{
	// Parameters.

	// int minSurfelSize = 1000;
	// int minEdgeSize = 20;
	float minHypRefSurfaceAngle = 45.0f;
	// float minHypRefSurfaceAngle = 30.0f;
	float maxROIXNormalAngle = 30.0f; // deg

	// Constants.

	float csMinHypRefSurfaceAngle = cos(minHypRefSurfaceAngle * DEG2RAD);
	float csMaxROIXNormalAngle = cos(maxROIXNormalAngle * DEG2RAD);

	//

	Vector3<float> pDominantShiftDir;
	float dominantShiftSize;
	if (pDominantShift)
	{
		dominantShiftSize = sqrt(RVLDOTPRODUCT3(pDominantShift->Element, pDominantShift->Element));
		RVLSCALE3VECTOR2(pDominantShift->Element, dominantShiftSize, pDominantShiftDir.Element);
	}

	// Detect surfels.

	pMem->Clear();
	pSurfels->Init(pMesh);
	pSurfelDetector->Init(pMesh, pSurfels, pMem);
	printf("Segmentation to surfels...");
	pSurfelDetector->Segment(pMesh, pSurfels);
	printf("completed.\n");
	int nSurfels = pSurfels->NodeArray.n;
	printf("No. of surfels = %d\n", nSurfels);
	pSurfels->EdgePointNormals(pMesh);
	pSurfels->DetectVertices(pMesh);

	// Display surfels.

	uchar green[] = {0, 255, 0};
	// if (pVisualizationData->bVisualizeSurfels)
	//{
	//	pSurfels->NodeColors(green);
	//	pSurfels->InitDisplay(pVisualizationData->pVisualizer, pMesh, pSurfelDetector);
	//	pSurfels->Display(pVisualizationData->pVisualizer, pMesh);
	//	pSurfels->DisplayEdgeFeatures();
	//	pVisualizationData->pVisualizer->Run();
	// }

	// Only for purpose of testing PlanarSurfelDetector::CreatePolygons()!!!

	// pSurfelDetector->CreatePolygons(pMesh, pSurfels, NULL);

	// Sample Image.

	SampleImage(pMesh);

	// Merge approximatelly coplanar adjacent surfels into planar surfaces.

	pSurfelDetector->MergeSurfels(pMesh, pSurfels, minSurfelSize, minEdgeSize, maxCoPlanarSurfelNormalAngle, maxCoPlanarSurfelRefPtDist, &planarSurfaces);
	// pSurfelDetector->MergeSurfels(pMesh, pSurfels, minSurfelSize, minEdgeSize, maxCoPlanarSurfelNormalAngle, maxCoPlanarSurfelRefPtDist, &planarSurfaces, pVisualizationData->pVisualizer);

	// Visualize planar surfaces.

	if (pVisualizationData->bVisualizeSurfels)
	{
		planarSurfaces.NodeColors(green);
		planarSurfaces.InitDisplay(pVisualizationData->pVisualizer, pMesh, pSurfelDetector);
		planarSurfaces.Display(pVisualizationData->pVisualizer, pMesh);
		pVisualizationData->pVisualizer->Run();
		pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
	}

	///

	// sortedPlanarSurfaces <- sorted list of suficiently large planar surfaces within the ROI.
	//				           or sorted list of planar surfaces with minimum minPointsWithDominantShift from dominantShiftPointsIdx

	Array<SortIndex<int>> sortedPlanarSurfaces;
	sortedPlanarSurfaces.Element = new SortIndex<int>[planarSurfaces.NodeArray.n];
	sortedPlanarSurfaces.n = 0;
	SortIndex<int> *pPlanarSurfacelIdx = sortedPlanarSurfaces.Element;
	bool *bRelevant = new bool[planarSurfaces.NodeArray.n];
	memset(bRelevant, 0, planarSurfaces.NodeArray.n * sizeof(bool));
	bool bDominantShiftPoints = false;
	if (dominantShiftPointsIdx)
		bDominantShiftPoints = (dominantShiftPointsIdx->n >= minPointsWithDominantShift);
	Surfel *pSurfel;
	float csDominantShift;
	if (bDominantShiftPoints)
	{
		int *nSufrelPoints = new int[planarSurfaces.NodeArray.n];
		memset(nSufrelPoints, 0, planarSurfaces.NodeArray.n * sizeof(int));
		int iSurfel;
		int maxnDominantShiftPointsPerSurfel = 0;
		for (int iPt = 0; iPt < dominantShiftPointsIdx->n; iPt++)
		{
			iSurfel = planarSurfaces.surfelMap[dominantShiftPointsIdx->Element[iPt]];
			if ((iSurfel >= 0) && (iSurfel < planarSurfaces.NodeArray.n))
			{
				nSufrelPoints[iSurfel]++;
				if (nSufrelPoints[iSurfel] > maxnDominantShiftPointsPerSurfel)
					maxnDominantShiftPointsPerSurfel = nSufrelPoints[iSurfel];
			}
		}
		printf("max. no. dominant shift points per surfel = %d\n", maxnDominantShiftPointsPerSurfel);
		for (iSurfel = 0; iSurfel < planarSurfaces.NodeArray.n; iSurfel++)
		{
			pSurfel = planarSurfaces.NodeArray.Element + iSurfel;
			if (pSurfel->bEdge)
				continue;
			if (nSufrelPoints[iSurfel] >= minPointsWithDominantShift)
			{
				csDominantShift = RVLDOTPRODUCT3(pDominantShiftDir.Element, pSurfel->N);
				if (RVLABS(csDominantShift) >= 0.9f)
				{
					bRelevant[iSurfel] = true;
					pPlanarSurfacelIdx->idx = iSurfel;
					pPlanarSurfacelIdx->cost = nSufrelPoints[iSurfel];
					pPlanarSurfacelIdx++;
				}
			}
		}
		delete[] nSufrelPoints;
	}
	// if dominantShiftPointsIdx == NULL -> find dominant surfel within ROI
	sortedPlanarSurfaces.n = pPlanarSurfacelIdx - sortedPlanarSurfaces.Element;
	int iPlanarSurface;
	Surfel *pPlanarSurface, *pMemberSurfel;
	QLIST::Index *pMemberSurfelIdx;
	float fTmp;
	float V3Tmp[3];
	float *P;
	if ((!bDominantShiftPoints || sortedPlanarSurfaces.n == 0) && ROIs.size() > 0)
	{
		bool bInROI;
		int i;
		Box<float> surfelBBox;
		Array<int> surfelVertices;
		surfelVertices.Element = new int[pSurfels->vertexArray.n];
		float invRFR[9];
		float invRFt[3];
		float *Z = invRFR + 6;
		if (pRF)
			RVLINVTRANSF3D(pRF->R, pRF->t, invRFR, invRFt);
		float PROI[3];
		AffinePose3D ROI_;
		Box<float> ROI;
		Pose3D ROIPose;
		Box<float> planarSurfaceBBox, BBoxIntersection;
		bool bBBoxInitialized;
		for (int iROI = 0; iROI < ROIs.size(); iROI++)
		{
			ROI_ = ROIs[iROI];
			RVLCOPYMX3X3(ROI_.R, ROIPose.R);
			RVLCOPY3VECTOR(ROI_.t, ROIPose.t);
			if (pVisualizationData->bVisualizeROI)
			{
				pVisualizationData->pVisualizer->SetMesh(pMesh);
				pVisualizationData->pVisualizer->DisplayBox(ROI_.s[0], ROI_.s[1], ROI_.s[2], &ROIPose, 255.0, 0.0, 0.0);
			}
			ROI.minx = -0.5f * ROI_.s[0];
			ROI.maxx = 0.5f * ROI_.s[0];
			ROI.miny = -0.5f * ROI_.s[1];
			ROI.maxy = 0.5f * ROI_.s[1];
			ROI.minz = -0.5f * ROI_.s[2];
			ROI.maxz = 0.5f * ROI_.s[2];
			for (iPlanarSurface = 0; iPlanarSurface < planarSurfaces.NodeArray.n; iPlanarSurface++)
			{
				pPlanarSurface = planarSurfaces.NodeArray.Element + iPlanarSurface;
				if (RVLMULVECTORCOL3(pPlanarSurface->N, ROIPose.R, 0) < csMaxROIXNormalAngle)
					continue;
				bBBoxInitialized = false;
				pMemberSurfelIdx = pPlanarSurface->children.pFirst;
				while (pMemberSurfelIdx)
				{
					pMemberSurfel = pSurfels->NodeArray.Element + pMemberSurfelIdx->Idx;
					P = pMemberSurfel->representativePts;
					for (i = 0; i < 4; i++, P += 3)
					{
						RVLINVTRANSF3(P, ROIPose.R, ROIPose.t, PROI, V3Tmp);
						if (bBBoxInitialized)
							UpdateBoundingBox<float>(&planarSurfaceBBox, PROI);
						else
						{
							InitBoundingBox<float>(&planarSurfaceBBox, PROI);
							bBBoxInitialized = true;
						}
					}
					pMemberSurfelIdx = pMemberSurfelIdx->pNext;
				}
				bInROI = BoxIntersection(&ROI, &planarSurfaceBBox, &BBoxIntersection);
				if (bInROI)
				{
					if (pPlanarSurface->size >= minSurfelSize)
					{
						bRelevant[iPlanarSurface] = true;
						pPlanarSurfacelIdx->idx = iPlanarSurface;
						pPlanarSurfacelIdx->cost = pPlanarSurface->size;
						pPlanarSurfacelIdx++;
					}
				}
			}
		}
		delete[] surfelVertices.Element;
		if (pVisualizationData->bVisualizeROI)
		{
			pVisualizationData->pVisualizer->Run();
			pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
		}
	}
	sortedPlanarSurfaces.n = pPlanarSurfacelIdx - sortedPlanarSurfaces.Element;
	if (sortedPlanarSurfaces.n == 0)
	{
		printf("No relevant surfels!\n\n");
		delete[] bRelevant;
		return false;
	}
	BubbleSort<SortIndex<int>>(sortedPlanarSurfaces, true);

	delete[] bRelevant;

	/// Generate hypotheses.

	printf("Generating moving part hypotheses...\n");

	// Reference frames:
	//
	//     S  - scene RF: standard camera RF
	//     B  - hypotheis bbox RF: origin: hypothesis bbox center; axis orientation: aligned with bbox axes
	//     B0 - origin = origin of S; axis orientation = axis orientation of B
	//     M  - model RF

	int bboxAxesIdxM[3] = {1, 2, 0};
	float bboxAxesSignM[3] = {1, 1, 1};
	float RBM[9];
	RVLNULLMX3X3(RBM);
	float *axisMB = RBM;
	for (int i = 0; i < 3; i++, axisMB += 3)
		axisMB[bboxAxesIdxM[i]] = bboxAxesSignM[i];
	float RMB[9];
	RVLCOPYMX3X3T(RBM, RMB);
	Surfel *pPlanarSurface_;
	RECOG::DDD::Model *pModel = models.Element;
	float csHypRefSurfaceAngle;
	float RSB[9], RBS[9];
	float *XBS = RSB;
	float *YBS = RSB + 3;
	float *ZBS = RSB + 6;
	RECOG::DDD::Hypothesis hyp;
	float bboxSizeB[3];
	Box<float> bbox;
	Array<int> cluster;
	cluster.Element = new int[2 * pSurfelDetector->maxnSurfelsPerPlanarSurface];
	int iTmp;
	float area, minArea;
	Box<float> bestBBox;
	float bestRSB[9];
	Rect<float> rect;
	QList<QLIST::Index> *pSurfelList;
	SURFEL::EdgePtr *pAdjacentSurfelEdgePtr;
	SURFEL::Edge *pAdjacentSurfelEdge;
	int iSurfel, iSurfel_;
	QLIST::Index *pSurfelIdx_;
	float dP[3];
	Surfel *pSurfel_;
	for (int i = 0; i < sortedPlanarSurfaces.n; i++)
	{
		iPlanarSurface = sortedPlanarSurfaces.Element[i].idx;
		pPlanarSurface = planarSurfaces.NodeArray.Element + iPlanarSurface;
		if (U)
			if (RVLDOTPRODUCT3(pPlanarSurface->N, U) < csMinHypRefSurfaceAngle)
				continue;
		pSurfelList = &(pPlanarSurface->children);
		cluster.n = 0;
		pSurfelIdx_ = pSurfelList->pFirst;
		while (pSurfelIdx_)
		{
			cluster.Element[cluster.n++] = pSurfelIdx_->Idx;
			pSurfelIdx_ = pSurfelIdx_->pNext;
		}
		RVLCOPY3VECTOR(pPlanarSurface->N, XBS);
		if (bHypGenFlat)
		{
			minArea = -1.0f;
			pSurfelIdx_ = pSurfelList->pFirst;
			while (pSurfelIdx_)
			{
				iSurfel = pSurfelIdx_->Idx;
				pSurfel = pSurfels->NodeArray.Element + iSurfel;
				pAdjacentSurfelEdgePtr = pSurfel->EdgeList.pFirst;
				while (pAdjacentSurfelEdgePtr)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iSurfel, pAdjacentSurfelEdgePtr, pAdjacentSurfelEdge, iSurfel_);
					pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;
					if (pSurfel_->bEdge && pSurfel_->size >= minEdgeSize || !pSurfel_->bEdge && pSurfel_->size >= minSurfelSize)
					{
						// csHypRefSurfaceAngle = RVLDOTPRODUCT3(pPlanarSurface->N, pSurfel_->N);
						// if (RVLABS(csHypRefSurfaceAngle) < csMinHypRefSurfaceAngle)
						{
							RVLCROSSPRODUCT3(pSurfel_->N, XBS, YBS);
							RVLNORM3(YBS, fTmp);
							RVLCROSSPRODUCT3(XBS, YBS, ZBS);

							// Minimum area method.

							BoundingBox(pMesh, cluster, RSB, bbox);
							BoxSize<float>(&bbox, bboxSizeB[0], bboxSizeB[1], bboxSizeB[2]);
							area = bboxSizeB[1] * bboxSizeB[2];
							if (minArea < 0.0f || area < minArea)
							{
								minArea = area;
								RVLCOPYMX3X3(RSB, bestRSB);
								bestBBox = bbox;
							}

							// Rectangular boundary method.
						}
					}
					pAdjacentSurfelEdgePtr = pAdjacentSurfelEdgePtr->pNext;
				}
				pSurfelIdx_ = pSurfelIdx_->pNext;
			}
			if (minArea >= 0.0f)
			{
				RVLCOPYMX3X3T(bestRSB, RBS);
				FitRectangle(pMesh, &planarSurfaces, iPlanarSurface, RBS, bestBBox, 0.02f, &rect);
				// GenerateHypothesis(bestBBox, pModel, RMB, bboxAxesIdxM, bboxAxesSignM, RBS, hyp);
				// FitBBoxToPlanarSurface(&hyp, pPlanarSurface);
				RVLCOPYMX3X3(RBS, hyp.pose.R);
				RVLSET3VECTOR(V3Tmp, pPlanarSurface->d - 0.5f * pModel->bboxSize[2], 0.5f * (rect.minx + rect.maxx), 0.5f * (rect.miny + rect.maxy));
				RVLMULMX3X3VECT(RBS, V3Tmp, hyp.pose.t);
				RVLSET3VECTOR(hyp.bboxSize, pModel->bboxSize[2], rect.maxx - rect.minx, rect.maxy - rect.miny);
				RVLSET3VECTOR(hyp.pose.s, 1.0f, hyp.bboxSize[1] / pModel->bboxSize[1], hyp.bboxSize[2] / pModel->bboxSize[0]);
				hyp.assoc.Element = NULL;
				hyps.push_back(hyp);
				printf("  Hypothesis: %f x %f\n", hyp.bboxSize[1], hyp.bboxSize[2]);
			}
		}
		else
		{
			for (int j = i + 1; j < sortedPlanarSurfaces.n; j++)
			{
				pPlanarSurface_ = planarSurfaces.NodeArray.Element + sortedPlanarSurfaces.Element[j].idx;
				csHypRefSurfaceAngle = RVLDOTPRODUCT3(pPlanarSurface->N, pPlanarSurface_->N);
				if (RVLABS(csHypRefSurfaceAngle) >= csMinHypRefSurfaceAngle)
					continue;
				RVLDIF3VECTORS(pPlanarSurface_->P, pPlanarSurface->P, dP);
				if (bHypGenConvex && (RVLDOTPRODUCT3(pPlanarSurface->N, dP) >= 0.0f || RVLDOTPRODUCT3(pPlanarSurface_->N, dP) <= 0.0f))
					continue;
				RVLCROSSPRODUCT3(pPlanarSurface_->N, XBS, YBS);
				RVLNORM3(YBS, fTmp);
				RVLCROSSPRODUCT3(XBS, YBS, ZBS);
				iTmp = cluster.n;
				pSurfelList = &(pPlanarSurface_->children);
				pSurfelIdx_ = pSurfelList->pFirst;
				while (pSurfelIdx_)
				{
					cluster.Element[cluster.n++] = pSurfelIdx_->Idx;
					pSurfelIdx_ = pSurfelIdx_->pNext;
				}
				BoundingBox(pMesh, cluster, RSB, bbox);
				cluster.n = iTmp;
				// PS = pMesh->NodeArray.Element[pSurfels->NodeArray.Element[pPlanarSurface->surfels.pFirst->Idx].PtList.pFirst->Idx].P;
				// RVLMULMX3X3VECT(RSB, PS, PB);
				// InitBoundingBox<float>(&bbox, PB);
				// for (int k = 0; k < 2; k++)
				//{
				//	pSurfelList = &(pPlanarSurface__[k]->surfels);
				//	pSurfelIdx_ = pSurfelList->pFirst;
				//	while (pSurfelIdx_)
				//	{
				//		pSurfel = pSurfels->NodeArray.Element + pSurfelIdx_->Idx;
				//		pPtList = &(pSurfel->PtList);
				//		pPtIdx = pPtList->pFirst;
				//		while (pPtIdx)
				//		{
				//			PS = pMesh->NodeArray.Element[pPtIdx->Idx].P;
				//			RVLMULMX3X3VECT(RSB, PS, PB);
				//			UpdateBoundingBox<float>(&bbox, PB);
				//			pPtIdx = pPtIdx->pNext;
				//		}
				//		pSurfelIdx_ = pSurfelIdx_->pNext;
				//	}
				// }
				RVLCOPYMX3X3T(RSB, RBS);
				GenerateHypothesis(bbox, pModel, RMB, bboxAxesIdxM, bboxAxesSignM, RBS, hyp);
				// BoxSize<float>(&bbox, bboxSizeB[0], bboxSizeB[1], bboxSizeB[2]);
				// for (int k = 0; k < 3; k++)
				//{
				//	hyp.bboxSize[k] = bboxSizeB[bboxAxesIdxM[k]];
				//	hyp.pose.s[k] = hyp.bboxSize[k] / pModel->bboxSize[k];
				//	tMB[bboxAxesIdxM[k]] = -bboxAxesSignM[k] * hyp.pose.s[k] * pModel->bboxCenter[k];
				// }
				// RVLCOPYMX3X3T(RSB, RBS);
				// BoxCenter<float>(&bbox, bboxCenterB0);
				// RVLMULMX3X3VECT(RBS, bboxCenterB0, tBS);
				// RVLCOMPTRANSF3D(RBS, tBS, RMB, tMB, hyp.pose.R, hyp.pose.t);
				// RVLINVTRANSL(RMB, tMB, hyp.bboxCenter);
				hyps.push_back(hyp);
			}
		}
	}
	printf("completed.\n\n");

	delete[] cluster.Element;
	delete[] sortedPlanarSurfaces.Element;

	return true;
}

void DDDetector::GenerateDoorHypotheses(
	std::vector<RECOG::DDD::Hypothesis> *pHyps,
	std::vector<RECOG::DDD::HypothesisDoorDrawer> &doorHyps)
{
	// Parameters.

	float ZThr = 0.1f;
	float ZAngleThr = 20.0f; // deg
	int minClusterSize = 3;
	int clusterSizeThrCoeff = 50;	   //%
	float jointAxisClusterThr = 0.06f; // m
	float dThr = 0.03f;				   // m
	bool bEstimateInnerFace = false;

	// Constants.

	float ZThr2 = ZThr * ZThr;
	float csZThr = cos(ZAngleThr * DEG2RAD);
	float jointAxisClusterThr2 = jointAxisClusterThr * jointAxisClusterThr;
	float dThr2 = dThr * dThr;

	// Visualize moving part hypotheses.

	// VisualizeHypothesisBBoxes(pHyps);
	// pVisualizationData->pVisualizer->Run();
	// pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();

	// Z-axis clustering.

	int iHyp;
	int nHyps = pHyps->size();
	int nHypPairs = nHyps * (nHyps - 1) / 2;
	int nZ = nHypPairs;
	int i, j;
	float R__[9];
	float *X = R__;
	float *Y = R__ + 3;
	float *Z = R__ + 6;
	float X_[3];
	float *R, *R_;
	float fTmp;
	struct ZHypothesis
	{
		int iHyp[2];
		float Z[3];
	};
	Array<ZHypothesis> ZHyps;
	ZHyps.Element = new ZHypothesis[nZ];
	ZHyps.n = 0;
	ZHypothesis *pZHyp;
	for (i = 0; i < nHyps; i++)
	{
		R = pHyps->at(i).pose.R;
		RVLCOPYCOLMX3X3(R, 0, X);
		for (j = 0; j < i; j++)
		{
			R_ = pHyps->at(j).pose.R;
			RVLCOPYCOLMX3X3(R_, 0, X_);
			pZHyp = ZHyps.Element + ZHyps.n;
			RVLCROSSPRODUCT3(X, X_, pZHyp->Z);
			fTmp = RVLDOTPRODUCT3(pZHyp->Z, pZHyp->Z);
			if (fTmp > 0.01f)
			{
				fTmp = sqrt(fTmp);
				RVLSCALE3VECTOR2(pZHyp->Z, fTmp, pZHyp->Z);
				pZHyp->iHyp[0] = i;
				pZHyp->iHyp[1] = j;
				ZHyps.n++;
			}
		}
	}
	float *EZ = new float[ZHyps.n * ZHyps.n];
	float cs, e;
	float *EZRow = EZ;
	ZHypothesis *pZHyp_;
	for (i = 0; i < ZHyps.n; i++, EZRow += ZHyps.n)
	{
		pZHyp = ZHyps.Element + i;
		for (j = 0; j < i; j++)
		{
			pZHyp_ = ZHyps.Element + j;
			cs = RVLDOTPRODUCT3(pZHyp->Z, pZHyp_->Z);
			e = 1.0 - RVLABS(cs);
			if (e < 0.0f)
				e = 0.0f;
			EZRow[j] = EZ[j * ZHyps.n + i] = e;
		}
		EZRow[i] = 0.0f;
	}
	int *ZClusterID = NULL;
	Array<Array<int>> ZClusters;
	ZClusters.Element = NULL;
	int *ZClusterMem = NULL;
	Clustering(NULL, ZHyps.n, 3, ZThr2, ZClusterID, ZClusters, ZClusterMem, EZ);
	delete[] EZ;

	//// For every Z-axis cluster.

	int dominantClusterSize = 0;
	int iZ;
	for (iZ = 0; iZ < ZClusters.n; iZ++)
		if (ZClusters.Element[iZ].n > dominantClusterSize)
			dominantClusterSize = ZClusters.Element[iZ].n;
	int minZClusterSize_ = dominantClusterSize * clusterSizeThrCoeff / 100;
	minZClusterSize_ = RVLMAX(minClusterSize, minZClusterSize_);
	Array<int> ZCluster;
	cv::Mat cvCX(3, 3, CV_64FC1);
	double *CX = (double *)(cvCX.data);
	float Mx3x3Tmp[9];
	bool *bInZCluster = new bool[nHyps];
	memset(bInZCluster, 0, nHyps * sizeof(bool));
	bool *bInJointAxisCluster = new bool[nHyps];
	memset(bInJointAxisCluster, 0, nHyps * sizeof(bool));
	Array<int> ZClusterHyps;
	ZClusterHyps.Element = new int[nHyps];
	cv::Mat cvEigVCX;
	double *eigVCX;
	cv::Mat cvEigCX;
	float *ZHypMem = new float[3 * ZClusters.n];
	float *ZHyp;
	double *mxRowDouble;
	float sn;
	bool bValid;
	RECOG::DDD::Hypothesis hyp;
	RECOG::DDD::Hypothesis *pHyp, *pHyp_;
	float RX[9];
	float RZ[9];
	RVLROTZ(-1.0f, 0.0f, RZ);
	Array<RECOG::DDD::Hypothesis> ZClusterHyps_;
	ZClusterHyps_.Element = new RECOG::DDD::Hypothesis[nHyps];
	float d;
	int k;
	float RDC[9], RCD[9];
	float *XDMem = new float[2 * nHyps];
	float *dD = new float[nHyps];
	float *XD, *XD_;
	float *tDMem = new float[2 * nHyps];
	float *tD;
	float V3Tmp[3];
	float *PAMem = new float[2 * nHypPairs];
	float *PA;
	struct HypothesisPair
	{
		int iHyp[2];
	};
	Array<HypothesisPair> PAHypPairs;
	PAHypPairs.Element = new HypothesisPair[nHypPairs];
	int *PAClusterID;
	Array<Array<int>> PAClusters;
	PAClusters.Element = NULL;
	int *PAClusterMem;
	int iJointAxis;
	Array<int> jointAxisCluster;
	HypothesisPair PAHypPair;
	Array<RECOG::DDD::Hypothesis> JointAxisClusterHyps_;
	JointAxisClusterHyps_.Element = new RECOG::DDD::Hypothesis[nHyps];
	Array<int> JointAxisClusterHyps;
	JointAxisClusterHyps.Element = new int[nHyps];
	float meanPA[2];
	cv::Mat cvA(3, 3, CV_64FC1);
	double *A = (double *)(cvA.data);
	cv::Mat cvx(3, 1, CV_64FC1);
	double *x = (double *)(cvx.data);
	cv::Mat cvb(3, 1, CV_64FC1);
	double *b = (double *)(cvb.data);
	float rx;
	float *dMem = new float[2 * nHyps];
	int *dClusterID = new int[nHyps];
	Array<Array<int>> dClusters[4];
	Array<int> *dClusterArrayMem = new Array<int>[4 * nHyps];
	Array<float> dHyp[4];
	float *dHypMem = new float[4 * nHyps];
	for (i = 0; i < 4; i++)
	{
		dClusters[i].Element = dClusterArrayMem + i * nHyps;
		dHyp[i].Element = dHypMem + i * nHyps;
	}
	int *dClusterMem = new int[4 * nHyps];
	int *dClusterMem_;
	Array<int> dCluster;
	int l, m;
	RECOG::DDD::HypothesisDoorDrawer doorHyp;
	int defaultStateImgID;
	int defaultStateHyp;
	float defaultStateXD[2];
	Pose3D poseAD;
	int mindClusterSize_, minJointAxisClusterSize_;
	// For visualization purpose.
	Point *pPt;
	Pose3D pose;
	Point jointAxisEndPt[2];
	uchar green[] = {0, 255, 0};
	uchar blue[] = {0, 0, 255};
	//
	for (iZ = 0; iZ < ZClusters.n; iZ++)
	{
		ZCluster = ZClusters.Element[iZ];
		if (ZCluster.n < minZClusterSize_)
			continue;
		ZClusterHyps.n = 0;
		for (i = 0; i < ZCluster.n; i++)
		{
			pZHyp = ZHyps.Element + ZCluster.Element[i];
			for (j = 0; j < 2; j++)
			{
				iHyp = pZHyp->iHyp[j];
				if (!bInZCluster[iHyp])
				{
					bInZCluster[iHyp] = true;
					ZClusterHyps.Element[ZClusterHyps.n++] = iHyp;
				}
			}
		}
		RVLNULLMX3X3(CX);
		for (i = 0; i < ZClusterHyps.n; i++)
		{
			iHyp = ZClusterHyps.Element[i];
			R = pHyps->at(iHyp).pose.R;
			RVLCOPYCOLMX3X3(R, 0, X);
			RVLVECTCOV3(X, Mx3x3Tmp);
			RVLSUMMX3X3UT(CX, Mx3x3Tmp, CX);
			bInZCluster[iHyp] = false;
		}
		RVLCOMPLETESIMMX3(CX);
		cv::eigen(cvCX, cvEigCX, cvEigVCX);
		eigVCX = (double *)(cvEigVCX.data);
		mxRowDouble = eigVCX + 6;
		ZHyp = ZHypMem + 3 * iZ;
		RVLCOPY3VECTOR(mxRowDouble, ZHyp);
		if (ZHyp[1] > 0.0f)
		{
			RVLNEGVECT3(ZHyp, ZHyp);
		}

		// Prune z-axis clusters and re-orient the member hypotheses to obtain a consistent hypothesis set.

		ZClusterHyps_.n = 0;
		for (i = 0; i < ZClusterHyps.n; i++)
		{
			iHyp = ZClusterHyps.Element[i];
			hyp = pHyps->at(iHyp);
			R = hyp.pose.R;
			RVLCOPYCOLMX3X3(R, 2, Z);
			cs = RVLDOTPRODUCT3(Z, ZHyp);
			if (cs >= csZThr)
			{
				cs = 1.0f;
				sn = 0.0f;
				bValid = true;
			}
			else if (-cs >= csZThr)
			{
				cs = -1.0f;
				sn = 0.0f;
				bValid = true;
			}
			else
			{
				RVLCOPYCOLMX3X3(R, 1, Z);
				cs = RVLDOTPRODUCT3(Z, ZHyp);
				if (cs >= csZThr)
				{
					cs = 0.0f;
					sn = -1.0f;
					bValid = true;
					fTmp = hyp.bboxSize[1];
					hyp.bboxSize[1] = hyp.bboxSize[2];
					hyp.bboxSize[2] = fTmp;
				}
				else if (-cs >= csZThr)
				{
					cs = 0.0f;
					sn = 1.0f;
					bValid = true;
					fTmp = hyp.bboxSize[1];
					hyp.bboxSize[1] = hyp.bboxSize[2];
					hyp.bboxSize[2] = fTmp;
				}
				else
					bValid = false;
			}
			if (bValid)
			{
				RVLROTX(cs, sn, RX);
				RVLCOPYMX3X3(R, Mx3x3Tmp);
				RVLMXMUL3X3(Mx3x3Tmp, RX, R);
				RVLCOPY3VECTOR(ZHyp, Z);
				RVLCOPYCOLMX3X3(R, 0, X);
				RVLCROSSPRODUCT3(Z, X, Y);
				RVLNORM3(Y, fTmp);
				RVLCROSSPRODUCT3(Y, Z, X);
				RVLCOPYMX3X3T(R__, R);
				ZClusterHyps_.Element[ZClusterHyps_.n++] = hyp;
			}
		}
		if (ZClusterHyps_.n < minClusterSize)
			continue;

		// Visualize the z-axis cluster.

		// for (i = 0; i < ZClusterHyps_.n; i++)
		//{
		//	pHyp = ZClusterHyps_.Element + i;
		//	RVLCOPYMX3X3(pHyp->pose.R, pose.R);
		//	RVLCOPY3VECTOR(pHyp->pose.t, pose.t);
		//	pVisualizationData->pVisualizer->DisplayBox(pHyp->bboxSize[0], pHyp->bboxSize[1], pHyp->bboxSize[2], &pose, 255.0, 0.0, 0.0);
		// }
		// pVisualizationData->pVisualizer->Run();
		// pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();

		// Reference frame D.

		RVLCOPY3VECTOR(ZHyp, Z);
		RVLORTHOGONAL3(Z, X, i, j, k, fTmp);
		RVLCROSSPRODUCT3(Z, X, Y);
		RVLCOPYMX3X3(R__, RCD);
		RVLCOPYMX3X3T(RCD, RDC);

		// Top and bottom face clustering.

		for (j = 0; j < 2; j++)
		{
			for (i = 0; i < ZClusterHyps_.n; i++)
			{
				pHyp = ZClusterHyps_.Element + i;
				dMem[i] = RVLDOTPRODUCT3(ZHyp, pHyp->pose.t) - 0.5 * (float)(2 * j - 1) * pHyp->bboxSize[2];
			}
			dClusterMem_ = dClusterMem + j * nHyps;
			Clustering(dMem, ZClusterHyps_.n, 1, dThr2, dClusterID, dClusters[j], dClusterMem_);
			dominantClusterSize = 0;
			for (i = 0; i < dClusters[j].n; i++)
				if (dClusters[j].Element[i].n > dominantClusterSize)
					dominantClusterSize = dClusters[j].Element[i].n;
			mindClusterSize_ = dominantClusterSize * clusterSizeThrCoeff / 100;
			mindClusterSize_ = RVLMAX(minClusterSize, mindClusterSize_);
			dHyp[j].n = 0;
			for (i = 0; i < dClusters[j].n; i++)
			{
				dCluster = dClusters[j].Element[i];
				if (dCluster.n < mindClusterSize_)
					continue;
				d = 0.0f;
				for (k = 0; k < dCluster.n; k++)
					d += dMem[dCluster.Element[k]];
				d /= (float)(dCluster.n);
				dHyp[j].Element[dHyp[j].n++] = d;
			}
		}

		// Joint axis clustering.

		XD = XDMem;
		tD = tDMem;
		for (i = 0; i < ZClusterHyps_.n; i++, XD += 2, tD += 2)
		{
			pHyp = ZClusterHyps_.Element + i;
			RVLMULMXCOL3(RCD, pHyp->pose.R, 0, V3Tmp);
			XD[0] = V3Tmp[0];
			XD[1] = V3Tmp[1];
			RVLCOPYCOLMX3X3(pHyp->pose.R, 0, X);
			dD[i] = RVLDOTPRODUCT3(X, pHyp->pose.t);
			RVLMULMX3X3VECT(RCD, pHyp->pose.t, V3Tmp);
			tD[0] = V3Tmp[0];
			tD[1] = V3Tmp[1];
		}
		XD = XDMem;
		PAHypPairs.n = 0;
		for (i = 0; i < ZClusterHyps_.n; i++, XD += 2)
		{
			XD_ = XDMem;
			for (j = 0; j < i; j++, XD_ += 2)
			{
				fTmp = XD[0] * XD_[1] - XD[1] * XD_[0];
				if (RVLABS(fTmp) < 1e-3)
					continue;
				PA = PAMem + 2 * PAHypPairs.n;
				PA[0] = (XD_[1] * dD[i] - XD[1] * dD[j]) / fTmp;
				PA[1] = (-XD_[0] * dD[i] + XD[0] * dD[j]) / fTmp;
				PAHypPairs.Element[PAHypPairs.n].iHyp[0] = i;
				PAHypPairs.Element[PAHypPairs.n].iHyp[1] = j;
				PAHypPairs.n++;
			}
		}
		PAClusterID = PAClusterMem = NULL;
		PAClusters.Element = NULL;
		Clustering(PAMem, PAHypPairs.n, 2, jointAxisClusterThr2, PAClusterID, PAClusters, PAClusterMem);

		// Visualize MPPIs.

		// float meanzD = 0.0f;
		// for (i = 0; i < ZClusterHyps_.n; i++, XD += 2)
		//{
		//	pHyp = ZClusterHyps_.Element + i;
		//	RVLMULMX3X3VECT(RCD, pHyp->pose.t, V3Tmp);
		//	meanzD += V3Tmp[2];
		// }
		// meanzD /= (float)(ZClusterHyps_.n);
		// Array<Point> MPPIs;
		// MPPIs.Element = new Point[PAHypPairs.n];
		// MPPIs.n = PAHypPairs.n;
		// pPt = MPPIs.Element;
		// PA = PAMem;
		// for (i = 0; i < PAHypPairs.n; i++, pPt++, PA += 2)
		//{
		//	RVLSET3VECTOR(V3Tmp, PA[0], PA[1], meanzD);
		//	RVLMULMX3X3VECT(RDC, V3Tmp, pPt->P);
		// }
		// pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(MPPIs, green, 6.0f);
		// pVisualizationData->pVisualizer->Run();
		// pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
		// delete[] MPPIs.Element;

		/// For every joint axis cluster.

		dominantClusterSize = 0;
		for (iJointAxis = 0; iJointAxis < PAClusters.n; iJointAxis++)
			if (PAClusters.Element[iJointAxis].n > dominantClusterSize)
				dominantClusterSize = PAClusters.Element[iJointAxis].n;
		minJointAxisClusterSize_ = dominantClusterSize * clusterSizeThrCoeff / 100;
		minJointAxisClusterSize_ = RVLMAX(minClusterSize, minJointAxisClusterSize_);
		for (iJointAxis = 0; iJointAxis < PAClusters.n; iJointAxis++)
		{
			jointAxisCluster = PAClusters.Element[iJointAxis];
			if (jointAxisCluster.n < minJointAxisClusterSize_)
				continue;
			meanPA[0] = meanPA[1] = 0.0f;
			JointAxisClusterHyps.n = 0;
			for (i = 0; i < jointAxisCluster.n; i++)
			{
				PA = PAMem + 2 * jointAxisCluster.Element[i];
				meanPA[0] += PA[0];
				meanPA[1] += PA[1];
				PAHypPair = PAHypPairs.Element[jointAxisCluster.Element[i]];
				for (j = 0; j < 2; j++)
				{
					iHyp = PAHypPair.iHyp[j];
					if (!bInJointAxisCluster[iHyp])
					{
						bInJointAxisCluster[iHyp] = true;
						JointAxisClusterHyps.Element[JointAxisClusterHyps.n++] = iHyp;
					}
				}
			}
			for (i = 0; i < JointAxisClusterHyps.n; i++)
				bInJointAxisCluster[JointAxisClusterHyps.Element[i]] = false;
			fTmp = (float)(jointAxisCluster.n);
			meanPA[0] /= fTmp;
			meanPA[1] /= fTmp;
			pHyp_ = JointAxisClusterHyps_.Element;
			JointAxisClusterHyps_.n = JointAxisClusterHyps.n;
			RVLNULLMX3X3(A);
			RVLNULL3VECTOR(b);
			for (i = 0; i < JointAxisClusterHyps.n; i++, pHyp_++)
			{
				iHyp = JointAxisClusterHyps.Element[i];
				pHyp = ZClusterHyps_.Element + iHyp;
				*pHyp_ = *pHyp;
				XD = XDMem + 2 * iHyp;
				tD = tDMem + 2 * iHyp;
				V3Tmp[0] = meanPA[0] - tD[0];
				V3Tmp[1] = meanPA[1] - tD[1];
				if (-XD[1] * V3Tmp[0] + XD[0] * V3Tmp[1] < 0.0f)
				{
					RVLMXMUL3X3(pHyp->pose.R, RZ, pHyp_->pose.R);
					XD[0] = -XD[0];
					XD[1] = -XD[1];
					dD[iHyp] = -dD[iHyp];
				}
				V3Tmp[0] = XD[0];
				V3Tmp[1] = XD[1];
				V3Tmp[2] = 1.0f;
				RVLVECTCOV3(V3Tmp, Mx3x3Tmp);
				RVLSUMMX3X3UT(A, Mx3x3Tmp, A);
				d = dD[iHyp];
				RVLSCALE3VECTOR(V3Tmp, d, V3Tmp);
				RVLSUM3VECTORS(b, V3Tmp, b);
			}
			RVLCOMPLETESIMMX3(A);
			cv::solve(cvA, cvb, cvx);
			poseAD.t[0] = x[0];
			poseAD.t[1] = x[1];
			rx = x[2];

			// Visualize joint axis cluster.

			// VisualizeSequenceHypothesis(JointAxisClusterHyps_);
			// V3Tmp[0] = tAD[0]; V3Tmp[1] = tAD[1];
			// V3Tmp[2] = dHyp[0].Element[0];
			// pPt = jointAxisEndPt;
			// RVLMULMX3X3VECT(RDC, V3Tmp, pPt->P);
			// V3Tmp[2] = dHyp[1].Element[0];
			// pPt = jointAxisEndPt + 1;
			// RVLMULMX3X3VECT(RDC, V3Tmp, pPt->P);
			// pVisualizationData->pVisualizer->DisplayLine(jointAxisEndPt, green, 5.0f);
			// pVisualizationData->pVisualizer->Run();
			// pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();

			// Inner and outer face clustering.

			for (j = 2; j < 4; j++)
			{
				if (bEstimateInnerFace || j == 2)
				{
					for (i = 0; i < JointAxisClusterHyps.n; i++)
					{
						iHyp = JointAxisClusterHyps.Element[i];
						pHyp = JointAxisClusterHyps_.Element + i;
						XD = XDMem + 2 * iHyp;
						tD = tDMem + 2 * iHyp;
						V3Tmp[0] = tD[0] - poseAD.t[0];
						V3Tmp[1] = tD[1] - poseAD.t[1];
						dMem[i] = -XD[1] * V3Tmp[0] + XD[0] * V3Tmp[1] + 0.5 * (float)(2 * j - 5) * pHyp->bboxSize[1];
					}
					dClusterMem_ = dClusterMem + j * nHyps;
					Clustering(dMem, JointAxisClusterHyps.n, 1, dThr2, dClusterID, dClusters[j], dClusterMem_);
					dominantClusterSize = 0;
					for (i = 0; i < dClusters[j].n; i++)
						if (dClusters[j].Element[i].n > dominantClusterSize)
							dominantClusterSize = dClusters[j].Element[i].n;
					mindClusterSize_ = dominantClusterSize * clusterSizeThrCoeff / 100;
					mindClusterSize_ = RVLMAX(minClusterSize, mindClusterSize_);
					dHyp[j].n = 0;
					for (i = 0; i < dClusters[j].n; i++)
					{
						dCluster = dClusters[j].Element[i];
						if (dCluster.n < mindClusterSize_)
							continue;
						d = 0.0f;
						for (k = 0; k < dCluster.n; k++)
							d += dMem[dCluster.Element[k]];
						d /= (float)(dCluster.n);
						dHyp[j].Element[dHyp[j].n++] = d;
					}
				}
				else
				{
					dHyp[j].n = 1;
					dHyp[j].Element[0] = 0.0f;
				}
			}

			// Default state.

			defaultStateImgID = -1;
			for (i = 0; i < JointAxisClusterHyps.n; i++)
			{
				iHyp = JointAxisClusterHyps.Element[i];
				pHyp = JointAxisClusterHyps_.Element + i;
				if (pHyp->imgID < defaultStateImgID || defaultStateImgID < 0)
				{
					defaultStateImgID = pHyp->imgID;
					defaultStateHyp = iHyp;
				}
			}
			XD = XDMem + 2 * defaultStateHyp;
			defaultStateXD[0] = XD[0];
			defaultStateXD[1] = XD[1];

			//

			for (i = 0; i < dHyp[3].n; i++)
				for (j = 0; j < dHyp[2].n; j++)
					for (k = 0; k < dHyp[1].n; k++)
						for (l = 0; l < dHyp[0].n; l++)
						{
							// Create door hypothesis.

							RVLROTZ(defaultStateXD[0], defaultStateXD[1], poseAD.R);
							poseAD.t[2] = 0.5f * (dHyp[0].Element[l] + dHyp[1].Element[k]);
							doorHyp.objClass = RVLDDD_MODEL_DOOR;
							RVLMXMUL3X3(RDC, poseAD.R, doorHyp.pose.R);
							RVLMULMX3X3VECT(RDC, poseAD.t, doorHyp.pose.t);
							doorHyp.s[0] = dHyp[3].Element[i] - dHyp[2].Element[j];
							doorHyp.s[1] = dHyp[0].Element[l] - dHyp[1].Element[k];
							doorHyp.r[0] = rx;
							doorHyp.r[1] = 0.5f * (dHyp[2].Element[j] + dHyp[3].Element[i]);
							doorHyp.state.n = JointAxisClusterHyps_.n;
							doorHyp.state.Element = new RECOG::DDD::AOHypothesisState[doorHyp.state.n];
							for (m = 0; m < JointAxisClusterHyps.n; m++)
							{
								iHyp = JointAxisClusterHyps.Element[m];
								pHyp = JointAxisClusterHyps_.Element + m;
								doorHyp.state.Element[m].imgID = pHyp->imgID;
								XD = XDMem + 2 * iHyp;
								doorHyp.state.Element[m].q = atan2(-defaultStateXD[1] * XD[0] + defaultStateXD[0] * XD[1],
																   defaultStateXD[0] * XD[0] + defaultStateXD[1] * XD[1]);
							}
							doorHyps.push_back(doorHyp);

							// Visualize door hypothesis.

							VisualizeDoorHypothesis(doorHyp);
							pVisualizationData->pVisualizer->Run();
							pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
						}
		}
		delete[] PAClusterID;
		delete[] PAClusters.Element;
		delete[] PAClusterMem;

		///
	}

	////

	delete[] ZClusterID;
	delete[] ZClusterMem;
	delete[] ZClusters.Element;
	delete[] bInZCluster;
	delete[] ZClusterHyps.Element;
	delete[] ZHyps.Element;
	delete[] ZHypMem;
	delete[] ZClusterHyps_.Element;
	delete[] XDMem;
	delete[] dD;
	delete[] tDMem;
	delete[] PAMem;
	delete[] PAHypPairs.Element;
	delete[] bInJointAxisCluster;
	delete[] JointAxisClusterHyps.Element;
	delete[] JointAxisClusterHyps_.Element;
	delete[] dClusterID;
	delete[] dClusterArrayMem;
	delete[] dClusterMem;
	delete[] dMem;
	delete[] dHypMem;
}

void DDDetector::GenerateDrawerHypotheses(
	std::vector<RECOG::DDD::Hypothesis> *pHyps,
	std::vector<RECOG::DDD::HypothesisDoorDrawer> &drawerHyps)
{
	// Parameters.

	float axisThr = 0.2f;
	int minClusterSize = 3;
	int clusterSizeThrCoeff = 50; //%
	float ZAngleThr = 20.0f;	  // deg
	// float ZThr = 0.1f;
	float dThr = 0.03f; // m

	// Constants.

	float csZThr = cos(ZAngleThr * DEG2RAD);
	float dThr2 = dThr * dThr;

	// Visualize moving part hypotheses.

	// VisualizeHypothesisBBoxes(pHyps);
	// pVisualizationData->pVisualizer->Run();
	// pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();

	// X-axis clustering.

	int iHyp;
	int nHyps = pHyps->size();
	int i;
	// float R__[9];
	// float* X = R__;
	// float* Y = R__ + 3;
	// float* Z = R__ + 6;
	Array<RECOG::DDD::AxisHypothesis> XHyps;
	XHyps.Element = new RECOG::DDD::AxisHypothesis[nHyps];
	XHyps.n = 0;
	RECOG::DDD::AxisHypothesis *pXHyp;
	// float X_[3];
	float *R;
	for (i = 0; i < nHyps; i++)
	{
		R = pHyps->at(i).pose.R;
		pXHyp = XHyps.Element + XHyps.n;
		RVLCOPYCOLMX3X3(R, 0, pXHyp->axis);
		pXHyp->iHyp = i;
		XHyps.n++;
	}
	int *XClusterID = NULL;
	Array<Array<int>> XClusters;
	XClusters.Element = NULL;
	int *XClusterMem = NULL;
	Vector3DClustering(XHyps, axisThr, XClusterID, XClusters, XClusterMem);

	//// For every X-axis cluster.

	int minXClusterSize = MinClusterSize(XClusters, clusterSizeThrCoeff, minClusterSize);
	int iX, iZ;
	Array<int> XCluster, ZCluster;
	Array<Vector3<float>> XHyps_;
	XHyps_.Element = new Vector3<float>[XClusters.n];
	Vector3<float> *pXHyp_ = XHyps_.Element;
	Array<RECOG::DDD::AxisHypothesis> ZHyps;
	ZHyps.Element = new RECOG::DDD::AxisHypothesis[nHyps];
	RECOG::DDD::AxisHypothesis *pZHyp;
	int *ZClusterID;
	Array<Array<int>> ZClusters;
	ZClusters.Element;
	int *ZClusterMem;
	// Array<Vector3<float>> ZHyps_;
	// Vector3<float>* pZHyp_;
	float fTmp;
	int minZClusterSize;
	float RCD[9];
	float *XDC = RCD;
	float *YDC = RCD + 3;
	float *ZDC = RCD + 6;
	int j, k, l, m;
	Array<RECOG::DDD::Hypothesis> XClusterHyps;
	XClusterHyps.Element = new RECOG::DDD::Hypothesis[nHyps];
	RECOG::DDD::Hypothesis hyp;
	RECOG::DDD::Hypothesis *pHyp;
	Array<RECOG::DDD::Hypothesis> ZClusterHyps;
	ZClusterHyps.Element = new RECOG::DDD::Hypothesis[nHyps];
	float ZRef[3], ZBC[3];
	bool bValid;
	float *dMem = new float[4 * nHyps];
	int *dClusterID = new int[nHyps];
	Array<Array<int>> dClusters[4];
	Array<int> *dClusterArrayMem = new Array<int>[4 * nHyps];
	Array<float> dHyp[4];
	float *dHypMem = new float[4 * nHyps];
	for (i = 0; i < 4; i++)
	{
		dClusters[i].Element = dClusterArrayMem + i * nHyps;
		dHyp[i].Element = dHypMem + i * nHyps;
	}
	int *dClusterMem = new int[4 * nHyps];
	int *dClusterMem_;
	Array<int> dCluster;
	float *tDMem = new float[3 * nHyps];
	float *tD;
	int iAxis;
	float dir;
	int mindClusterSize_;
	float d;
	int defaultStateHyp;
	float defaultStatetDx = 0.0f;
	RECOG::DDD::HypothesisDoorDrawer drawerHyp;
	float tAD[3];
	for (iX = 0; iX < XClusters.n; iX++)
	{
		XCluster = XClusters.Element[iX];
		if (XCluster.n < minXClusterSize)
			continue;
		RVLNULL3VECTOR(pXHyp_->Element);
		for (i = 0; i < XCluster.n; i++)
		{
			pXHyp = XHyps.Element + XCluster.Element[i];
			RVLSUM3VECTORS(pXHyp_->Element, pXHyp->axis, pXHyp_->Element);
		}
		fTmp = sqrt(RVLDOTPRODUCT3(pXHyp_->Element, pXHyp_->Element));
		RVLSCALE3VECTOR2(pXHyp_->Element, fTmp, pXHyp_->Element);
		RVLCOPY3VECTOR(pXHyp_->Element, XDC);

		// Adjust the hypotheses of XCluster so that their x-axis is identical to XDC.

		XClusterHyps.n = 0;
		for (i = 0; i < XCluster.n; i++)
		{
			pXHyp = XHyps.Element + XCluster.Element[i];
			hyp = pHyps->at(pXHyp->iHyp);
			R = hyp.pose.R;
			RVLCOPYCOLMX3X3(R, 2, ZDC);
			RVLCROSSPRODUCT3(ZDC, XDC, YDC);
			RVLNORM3(YDC, fTmp);
			RVLCROSSPRODUCT3(XDC, YDC, ZDC);
			RVLCOPYMX3X3T(RCD, hyp.pose.R);
			XClusterHyps.Element[XClusterHyps.n++] = hyp;
		}

		// Z-Axis clustering.

		pZHyp = ZHyps.Element;
		for (i = 0; i < XClusterHyps.n; i++)
		{
			hyp = XClusterHyps.Element[i];
			R = hyp.pose.R;
			RVLCOPYCOLMX3X3(R, 2, pZHyp->axis);
			pZHyp->iHyp = pXHyp->iHyp;
			pZHyp++;
		}
		ZHyps.n = pZHyp - ZHyps.Element;
		float axisThr2 = axisThr * axisThr;
		float *E = new float[ZHyps.n * ZHyps.n];
		float cs, sn, e;
		float *ERow = E;
		RECOG::DDD::AxisHypothesis *pAxisHyp, *pAxisHyp_;
		int i, j;
		for (i = 0; i < ZHyps.n; i++, ERow += ZHyps.n)
		{
			pAxisHyp = ZHyps.Element + i;
			for (j = 0; j < i; j++)
			{
				pAxisHyp_ = ZHyps.Element + j;
				cs = RVLDOTPRODUCT3(pAxisHyp->axis, pAxisHyp_->axis);
				cs = RVLABS(cs);
				sn = sqrt(1.0 - cs * cs);
				e = 1.0 - RVLMAX(cs, sn);
				ERow[j] = E[j * ZHyps.n + i] = e;
			}
			ERow[i] = 0.0f;
		}
		ZClusterID = ZClusterMem = NULL;
		ZClusters.Element = NULL;
		Clustering(NULL, ZHyps.n, 3, axisThr2, ZClusterID, ZClusters, ZClusterMem, E);
		delete[] E;
		delete[] ZClusterID;

		/// For every Z-axis cluster.

		minZClusterSize = MinClusterSize(ZClusters, clusterSizeThrCoeff, minClusterSize);
		// ZHyps_.Element = new Vector3<float>[ZClusters.n];
		// pZHyp_ = ZHyps_.Element;
		for (iZ = 0; iZ < ZClusters.n; iZ++)
		{
			ZCluster = ZClusters.Element[iZ];
			if (ZCluster.n < minZClusterSize)
				continue;

			// Prune the hypotheses of ZCluster.
			// Adjust their z-axes to be consistently oriented.
			// Compute their mean vector.

			RVLNULL3VECTOR(ZDC);
			ZClusterHyps.n = 0;
			tD = tDMem;
			for (i = 0; i < ZCluster.n; i++)
			{
				hyp = XClusterHyps.Element[ZCluster.Element[i]];
				if (i == 0)
				{
					RVLCOPYCOLMX3X3(hyp.pose.R, 2, ZRef);
				}
				if (AlignHypothesisZAxis(hyp, ZRef, csZThr))
				{
					ZClusterHyps.Element[ZClusterHyps.n++] = hyp;
					RVLCOPYCOLMX3X3(hyp.pose.R, 2, ZBC);
					RVLSUM3VECTORS(ZDC, ZBC, ZDC);
				}
			}
			RVLCROSSPRODUCT3(ZDC, XDC, YDC);
			RVLNORM3(YDC, fTmp);
			RVLCROSSPRODUCT3(XDC, YDC, ZDC);

			// Determine the default state hypothesis.
			// Compute coordinates of their centers w.r.t. RF D.

			tD = tDMem;
			defaultStateHyp = -1;
			for (i = 0; i < ZClusterHyps.n; i++, tD += 3)
			{
				pHyp = ZClusterHyps.Element + i;
				RVLMULMX3X3VECT(RCD, pHyp->pose.t, tD);
				if (tD[0] < defaultStatetDx || defaultStateHyp < 0)
				{
					defaultStatetDx = tD[0];
					defaultStateHyp = i;
				}
			}

			// Face clustering.

			for (j = 0; j < 4; j++)
			{
				iAxis = j / 2 + 1;
				dir = (float)(2 * (j % 2) - 1);
				for (i = 0; i < ZClusterHyps.n; i++)
				{
					pHyp = ZClusterHyps.Element + i;
					tD = tDMem + 3 * i;
					dMem[i] = tD[iAxis] + 0.5 * dir * pHyp->bboxSize[iAxis];
				}
				dClusterMem_ = dClusterMem + j * nHyps;
				Clustering(dMem, ZClusterHyps.n, 1, dThr2, dClusterID, dClusters[j], dClusterMem_);
				mindClusterSize_ = MinClusterSize(dClusters[j], clusterSizeThrCoeff, minClusterSize);
				dHyp[j].n = 0;
				for (i = 0; i < dClusters[j].n; i++)
				{
					dCluster = dClusters[j].Element[i];
					if (dCluster.n < mindClusterSize_)
						continue;
					d = 0.0f;
					for (k = 0; k < dCluster.n; k++)
						d += dMem[dCluster.Element[k]];
					d /= (float)(dCluster.n);
					dHyp[j].Element[dHyp[j].n++] = d;
				}
			}

			//

			for (i = 0; i < dHyp[3].n; i++)
				for (j = 0; j < dHyp[2].n; j++)
					for (k = 0; k < dHyp[1].n; k++)
						for (l = 0; l < dHyp[0].n; l++)
						{
							// Create drawer hypothesis.

							drawerHyp.objClass = RVLDDD_MODEL_DRAWER;
							RVLCOPYMX3X3T(RCD, drawerHyp.pose.R);
							tAD[0] = defaultStatetDx;
							tAD[1] = 0.5f * (dHyp[0].Element[l] + dHyp[1].Element[k]);
							tAD[2] = 0.5f * (dHyp[2].Element[j] + dHyp[3].Element[i]);
							RVLMULMX3X3TVECT(RCD, tAD, drawerHyp.pose.t);
							drawerHyp.s[0] = dHyp[1].Element[k] - dHyp[0].Element[l];
							drawerHyp.s[1] = dHyp[3].Element[i] - dHyp[2].Element[j];
							drawerHyp.state.n = ZClusterHyps.n;
							drawerHyp.state.Element = new RECOG::DDD::AOHypothesisState[drawerHyp.state.n];
							tD = tDMem;
							for (m = 0; m < drawerHyp.state.n; m++, tD += 3)
							{
								pHyp = ZClusterHyps.Element + m;
								drawerHyp.state.Element[m].imgID = pHyp->imgID;
								drawerHyp.state.Element[m].q = tD[0] - defaultStatetDx;
							}
							drawerHyps.push_back(drawerHyp);

							// Visualize drawer hypothesis.

							VisualizeDrawerHypothesis(drawerHyp);
							pVisualizationData->pVisualizer->Run();
							pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
						}

			// pZHyp_++;
		}
		delete[] ZClusterMem;
		delete[] ZClusters.Element;

		///

		pXHyp_++;
	}

	////

	delete[] XHyps.Element;
	delete[] XClusterID;
	delete[] XClusters.Element;
	delete[] XClusterMem;
	delete[] XHyps_.Element;
	delete[] ZHyps.Element;
	delete[] XClusterHyps.Element;
	delete[] ZClusterHyps.Element;
	delete[] dMem;
	delete[] dClusterID;
	delete[] dClusterArrayMem;
	delete[] dHypMem;
	delete[] dClusterMem;
}

float DDDetector::EvaluateHypothesis(
	Mesh *pMesh,
	RECOG::DDD::Model *pModel,
	int m,
	float *q,
	float *RMS,
	float maxe,
	int &nTransparentPts,
	int *SMCorrespondence,
	RECOG::SceneFittingError *errorRecord,
	float *tMS)
{
	Project(pModel, m, q, RMS, tMS);
	return RECOG::EvaluateHypothesis3(pMesh, &planarSurfaces, surfelMask, ZBuffer, ZBufferActivePtArray, subImageMap,
									  image3x3Neighborhood, maxe, transparencyDepthThr, nTransparentPts, SMCorrespondence, errorRecord);
}

void DDDetector::DetectCuboids(Mesh *pMesh)
{
	// ROI.

	AffinePose3D ROI;
	RVLSET3VECTOR(ROI.s, 2.0f, 2.0f, 2.0f);
	RVLSET3VECTOR(ROI.t, 0.0f, 0.0f, 1.3f);
	RVLUNITMX3(ROI.R);
	std::vector<AffinePose3D> ROIs;
	ROIs.push_back(ROI);

	// Generate hypotheses.

	std::vector<RECOG::DDD::Hypothesis> hyps;
	bHypGenConvex = true;
	GenerateHypotheses(pMesh, ROIs, NULL, NULL, hyps);
	if (hyps.size() == 0)
	{
		printf("No hypotheses are generated.\n");
		return;
	}

	/// Visualization.

	// Display surfels.

	// uchar green[] = { 0, 255, 0 };
	// pSurfels->NodeColors(green);
	// pSurfels->InitDisplay(pVisualizationData->pVisualizer, pMesh, pSurfelDetector);
	// pSurfels->Display(pVisualizationData->pVisualizer, pMesh);
	// pSurfels->DisplayEdgeFeatures();
	// pVisualizationData->pVisualizer->Run();

	// AICP.

	imgGrid.Create(pMesh->NodeArray, &camera, 8);
	Array<OrientedPoint> pointsS;
	pointsS.Element = new OrientedPoint[camera.w * camera.h];
	CreateOrientedPointArrayFromPointArray(pMesh->NodeArray, pointsS);
	PointAssociationData pointAssociationData;
	RECOG::DDD::Model *pModel = models.Element;
	pointAssociationData.Create(pModel->points.n, pointsS.n, true);
	SetSceneForHypothesisVisualization(pMesh);
	RECOG::DDD::Hypothesis hyp = hyps[0];
	SetBBoxForHypothesisVisualization(&hyp);
	AffinePose3D APose;
	Array<Point> pointsMS;
	pointsMS.Element = new Point[pModel->points.n];
	// pVisualizationData->bVisualizeICPSteps = true;
	// pVisualizationData->bVisualizeModelPts = true;
	// pVisualizationData->bVisualizePtAssociation = true;
	AICP(pModel->points, pointsS, hyp.pose, nICPIterations, APose, pointAssociationData, &pointsMS);
	pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
	pointAssociationData.Clear();
	delete[] pointsS.Element;
	delete[] pointsMS.Element;
}

void DDDetector::DetectStorageVolumes(Mesh *pMesh)
{
	// Parameters.

	int minSurfelSize = 100;
	int minEdgeSize = 20;
	float r = 0.6f;				  // m
	float normalMatchThr = 45.0f; // deg
	int edgeWeight = 5;
	int nRandomSamplings = 1000;
	float minHypRefSurfaceAngle = 45.0f;
	int nEquationSolvingIterations = 3;

	// Constants.

	float r2 = r * r;
	float csNormalMatchThr = cos(45.0f * DEG2RAD);
	float csMinHypRefSurfaceAngle = cos(minHypRefSurfaceAngle * DEG2RAD);
	uchar red[] = {255, 0, 0};
	uchar green[] = {0, 255, 0};

	/// Hardcoded bboxes (for development prupose!).

	Array<RECOG::DDD::Hypothesis> predictions;
	predictions.n = 1;
	predictions.Element = new RECOG::DDD::Hypothesis[predictions.n];
	// float alpha[3];
	//// BBox 1.
	// RVLSET3VECTOR(predictions.Element[0].pose.t, -0.09056377, 1.86944373, -0.44310704);
	// RVLSET3VECTOR(predictions.Element[0].bboxSize, 0.29854022, 0.13125181, 0.14948647);
	// alpha[0] = 0.69153152;
	//// BBox 2.
	// RVLSET3VECTOR(predictions.Element[1].pose.t, -0.09595419, 1.87582248, -0.73144474);
	// RVLSET3VECTOR(predictions.Element[1].bboxSize, 0.23505258, 0.12600467, 0.14260277);
	// alpha[1] = 0.65101822;
	//// BBox 3.
	// RVLSET3VECTOR(predictions.Element[2].pose.t, -0.12227286, 1.88152148, -1.02795193);
	// RVLSET3VECTOR(predictions.Element[2].bboxSize, 0.25025915, 0.15082315, 0.15474344);
	// alpha[2] = 0.6678965;
	//// Compute bbox params.
	// for (int i = 0; i < 3; i++)
	//{
	//	RVLROTZ(cos(alpha[i]), sin(alpha[i]), predictions.Element[i].pose.R);
	//	RVLSCALE3VECTOR(predictions.Element[i].bboxSize, 2.0f, predictions.Element[i].bboxSize);
	// }

	///

	// Detect surfels.

	pMem->Clear();
	pSurfels->Init(pMesh);
	pSurfelDetector->Init(pMesh, pSurfels, pMem);
	printf("Segmentation to surfels...");
	pSurfelDetector->Segment(pMesh, pSurfels);
	printf("completed.\n");
	int nSurfels = pSurfels->NodeArray.n;
	printf("No. of surfels = %d\n", nSurfels);
	pSurfels->EdgePointNormals(pMesh);

	// Display surfels.

	if (pVisualizationData->bVisualizeSurfels)
	{
		pSurfels->NodeColors(green);
		pSurfels->InitDisplay(pVisualizationData->pVisualizer, pMesh, pSurfelDetector);
		pSurfels->Display(pVisualizationData->pVisualizer, pMesh);
		pSurfels->DisplayEdgeFeatures();
		pVisualizationData->pVisualizer->Run();
		pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
	}

	/// Hardcoded initial hypothesis (for development prupose!).

	// Top shelf.

	int iRefSurfel = 168;
	int iRefSurfel_ = 140;
	float bottomCenter[3];
	RVLSET3VECTOR(bottomCenter, -0.038, -0.076, 1.863);
	float h = 0.264;

	// Middle shelf.

	// int iRefSurfel = 192;
	// int iRefSurfel_ = 237;
	// float bottomCenter[3];
	// RVLSET3VECTOR(bottomCenter, -0.060, 0.179, 1.940);
	// float h = 0.264;

	// Bottom shelf.

	// int iRefSurfel = 170;
	// int iRefSurfel_ = 160;
	// float bottomCenter[3];
	// RVLSET3VECTOR(bottomCenter, -0.085, 0.475, 2.078);
	// float h = 0.264;

	///

	// Visualization initialization.
	pVisualizationData->pVisualizer->SetMesh(pMesh);

	// Hypothesis.

	float fTmp;
	float V3Tmp[3];
	RECOG::DDD::Hypothesis *pPrediction = predictions.Element;
	float *N = pSurfels->NodeArray.Element[iRefSurfel].N;
	float *N_ = pSurfels->NodeArray.Element[iRefSurfel_].N;
	float RCM[9];
	float *XMC = RCM;
	float *YMC = RCM + 3;
	float *ZMC = RCM + 6;
	RVLCOPY3VECTOR(N, XMC);
	RVLCROSSPRODUCT3(N_, XMC, YMC);
	RVLNORM3(YMC, fTmp);
	RVLCROSSPRODUCT3(XMC, YMC, ZMC);
	RVLCOPYMX3X3T(RCM, pPrediction->pose.R);
	float hhalf = 0.5f * h;
	RVLSCALE3VECTOR(ZMC, hhalf, V3Tmp);
	RVLSUM3VECTORS(bottomCenter, V3Tmp, pPrediction->pose.t);
	RVLSET3VECTOR(pPrediction->bboxSize, 0.3f, 0.74f, h);

	///

	// Representative surfel samples.

	pSurfels->RepresentativeSurfelSamples(
		pMesh,
		minSurfelSize,
		minEdgeSize);

	// for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++, surfelRefPts += 12)
	//{
	//	//if (iSurfel == 104)
	//	//	int debug = 0;
	//	pSurfel = pSurfels->NodeArray.Element + iSurfel;
	//	nSamples = 0;
	//	if (pSurfel->bEdge)
	//	{
	//		if (pSurfel->size < minEdgeSize)
	//			continue;
	//		P_ = surfelRefPts;
	//		RVLCOPY3VECTOR(pSurfel->P, P_);
	//		iPt = FurthestPoint(pSurfel->P, pMesh, pSurfel->PtList);
	//		pPt = pMesh->NodeArray.Element + iPt;
	//		P_ += 3;
	//		RVLCOPY3VECTOR(pPt->P, P_);
	//		iPt = FurthestPoint(pPt->P, pMesh, pSurfel->PtList);
	//		pPt = pMesh->NodeArray.Element + iPt;
	//		P_ += 3;
	//		RVLCOPY3VECTOR(pPt->P, P_);
	//		nSamples = 3;
	//	}
	//	else
	//	{
	//		if (pSurfel->size < minSurfelSize)
	//			continue;
	//		P_ = surfelRefPts;
	//		RVLCOPY3VECTOR(pSurfel->P, P_);
	//		iPt = FurthestPoint(pSurfel->P, pMesh, pSurfel->BoundaryArray.Element[0]);
	//		pPt = pMesh->NodeArray.Element + iPt;
	//		P_ += 3;
	//		RVLCOPY3VECTOR(pPt->P, P_);
	//		iPt = FurthestPoint(pPt->P, pMesh, pSurfel->BoundaryArray.Element[0]);
	//		pPt = pMesh->NodeArray.Element + iPt;
	//		RVLDIF3VECTORS(pPt->P, P_, V);
	//		P_ += 3;
	//		RVLCOPY3VECTOR(pPt->P, P_);
	//		RVLNORM3(V, fTmp);
	//		iPt = FurthestPoint(pPt->P, pMesh, pSurfel->BoundaryArray.Element[0], V);
	//		pPt = pMesh->NodeArray.Element + iPt;
	//		P_ += 3;
	//		RVLCOPY3VECTOR(pPt->P, P_);
	//		nSamples = 4;
	//	}
	//	P_ = surfelRefPts;
	//	for (i = 0; i < nSamples; i++, P_ += 3)
	//	{
	//		fTmp = RVLDOTPRODUCT3(pSurfel->N, P_) - pSurfel->d;
	//		RVLSCALE3VECTOR(pSurfel->N, fTmp, V3Tmp);
	//		RVLDIF3VECTORS(P_, V3Tmp, P_);
	//	}
	// }

	//// Generate and evaluate hypotheses.

	RECOG::InitZBuffer(pMesh, sceneSamplingResolution, ZBuffer, ZBufferActivePtArray, subImageMap);
	CreateImage3x3NeighborhoodLT(ZBuffer.w, image3x3Neighborhood);

	struct SurfAssoc
	{
		int iQSurfel;
		int iMSurf;
		int w;
	};

	struct PointPlaneAssoc
	{
		float PQ[3];
		int iMSurf;
		float NM[3];
	};

	int iPt, iSurfel;
	Surfel *pSurfel;
	float V[3];
	float *P_;
	int i;
	float *surfelRefPts;
	RECOG::DDD::Model *pModel = models.Element;
	int nModelSurfaces = pModel->nSurfaces;
	int nModelEdges = pModel->edges.n;
	int *surfAssocMem = new int[nModelSurfaces * pSurfels->NodeArray.n];
	Array<int> *surfAssoc = new Array<int>[nModelSurfaces];
	float *NMSMem = new float[3 * (nModelSurfaces + nModelEdges)];
	int iMSurf;
	for (iMSurf = 0; iMSurf < nModelSurfaces; iMSurf++)
		surfAssoc[iMSurf].Element = surfAssocMem + pSurfels->NodeArray.n * iMSurf;
	int *edgeAssocMem = new int[nModelEdges * pSurfels->NodeArray.n];
	Array<int> *edgeAssoc = new Array<int>[nModelEdges];
	RECOG::DDD::ModelEdge *pEdge;
	float *NMSEdgeMem = NMSMem + 3 * nModelSurfaces;
	int iMEdge;
	for (iMEdge = 0; iMEdge < nModelEdges; iMEdge++)
		edgeAssoc[iMEdge].Element = edgeAssocMem + pSurfels->NodeArray.n * iMEdge;
	int iPrediction;
	float dP[3];
	float dist;
	float *NM, *NMS;
	float *RMS;
	float *tMS;
	float csN;
	int minSurfelSize_;
	SurfAssoc *axisSurfelDistributionMem = new SurfAssoc[6 * pSurfels->NodeArray.n];
	Array<SurfAssoc> *axisSurfelDistribution = new Array<SurfAssoc>[6];
	int iAxis;
	for (iAxis = 0; iAxis < 6; iAxis++)
		axisSurfelDistribution[iAxis].Element = axisSurfelDistributionMem + iAxis * pSurfels->NodeArray.n;
	SurfAssoc selectedSurfel[6];
	int iTmp;
	int j;
	int iMSurf_[2];
	float refSurfelScore, maxRefSurfelScore;
	float V3Tmp_[3];
	int iSampling;
	cv::Mat cvA(9, 9, CV_32FC1);
	float *A = (float *)(cvA.data);
	cv::Mat cvE(9, 1, CV_32FC1);
	float *E = (float *)(cvE.data);
	cv::Mat cvx(9, 1, CV_32FC1);
	float dFixed[9];
	float *x = (float *)(cvx.data);
	float *phi = x + 6;
	float *a, *ar;
	float RSM[9], PQR[3], dR[9];
	float *u;
	int k;
	float th;
	float M3x3Tmp[9];
	float *P;
	float bestAlignScore;
	Pair<int, int> secondarySrufelRefPts;
	Array<int> surfelRefPts_;
	int surfelRefPtsMem_[4];
	surfelRefPts_.Element = surfelRefPtsMem_;
	PointPlaneAssoc *pointPlaneAssocMem = new PointPlaneAssoc[9];
	PointPlaneAssoc *pointPlaneAssoc;
	Array<RECOG::DDD::HypothesisSV> hyps;
	hyps.Element = new RECOG::DDD::HypothesisSV[nRandomSamplings];
	RECOG::DDD::HypothesisSV *pHyp;
	float s[7];
	s[0] = storageVolumeWallThickness;
	int nTransparentPts;
	int *SMCorrespondence = new int[ZBuffer.w * ZBuffer.h];
	vtkSmartPointer<vtkActor> hypothesisActor[4];
	RVL_DELETE_ARRAY(surfelMask);
	surfelMask = new bool[pSurfels->NodeArray.n];
	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
		surfelMask[iSurfel] = true;
	float sceneFittingScore;
	float bestSceneFittingScore = 0.0f;
	int iBestHypothesis;
	for (iPrediction = 0; iPrediction < predictions.n; iPrediction++)
	{
		// Surface association.

		pPrediction = predictions.Element + iPrediction;
		RMS = pPrediction->pose.R;
		tMS = pPrediction->pose.t;
		for (iMSurf = 0; iMSurf < nModelSurfaces; iMSurf++)
		{
			surfAssoc[iMSurf].n = 0;
			NM = pModel->A + 3 * pModel->AID[iMSurf];
			NMS = NMSMem + 3 * iMSurf;
			RVLMULMX3X3VECT(RMS, NM, NMS);
		}
		for (iMEdge = 0; iMEdge < nModelEdges; iMEdge++)
		{
			edgeAssoc[iMEdge].n = 0;
			pEdge = pModel->edges.Element + iMEdge;
			NMS = NMSEdgeMem + 3 * iMEdge;
			RVLMULMX3X3VECT(RMS, pEdge->N, NMS);
		}
		for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
		{
			// if (iSurfel == 38)
			//	int debug = 0;
			pSurfel = pSurfels->NodeArray.Element + iSurfel;
			minSurfelSize_ = (pSurfel->bEdge ? minEdgeSize : minSurfelSize);
			if (pSurfel->size < minSurfelSize_)
				continue;
			RVLDIF3VECTORS(pSurfel->P, tMS, dP);
			dist = RVLDOTPRODUCT3(dP, dP);
			if (dist > r2)
				continue;
			if (pSurfel->bEdge)
			{
				for (iMEdge = 0; iMEdge < nModelEdges; iMEdge++)
				{
					pEdge = pModel->edges.Element + iMEdge;
					NMS = NMSEdgeMem + 3 * iMEdge;
					csN = RVLDOTPRODUCT3(NMS, pSurfel->N);
					if (csN < csNormalMatchThr)
						continue;
					RVLDIF3VECTORS(pSurfel->P, tMS, dP);
					NMS = NMSMem + 3 * pEdge->iFace;
					if (pModel->d[pEdge->iFace] * RVLDOTPRODUCT3(NMS, dP) < 0.0f)
						continue;
					NMS = NMSMem + 3 * pEdge->iFace_;
					if (pModel->d[pEdge->iFace_] * RVLDOTPRODUCT3(NMS, dP) < 0.0f)
						continue;
					// if (pEdge->iFace == 1 || pEdge->iFace_ == 1)
					//	int debug = 0;
					edgeAssoc[iMEdge].Element[edgeAssoc[iMEdge].n++] = iSurfel;
				}
			}
			else
			{
				for (iMSurf = 0; iMSurf < nModelSurfaces; iMSurf++)
				{
					if (iMSurf == 1)
						continue;
					NMS = NMSMem + 3 * iMSurf;
					csN = RVLDOTPRODUCT3(NMS, pSurfel->N);
					if (csN < csNormalMatchThr)
						continue;
					RVLDIF3VECTORS(pSurfel->P, tMS, dP);
					if (pModel->d[iMSurf] * RVLDOTPRODUCT3(NMS, dP) < 0.0f)
						continue;
					surfAssoc[iMSurf].Element[surfAssoc[iMSurf].n++] = iSurfel;
				}
			}
		}

		// Distribution of surfel samples over axes of Q-base.

		for (iAxis = 0; iAxis < 6; iAxis++)
			axisSurfelDistribution[iAxis].n = 0;
		for (iMSurf = 0; iMSurf < nModelSurfaces; iMSurf++)
		{
			iAxis = pModel->info[iMSurf];
			for (i = 0; i < surfAssoc[iMSurf].n; i++)
			{
				iSurfel = surfAssoc[iMSurf].Element[i];
				axisSurfelDistribution[iAxis].Element[axisSurfelDistribution[iAxis].n].iQSurfel = iSurfel;
				axisSurfelDistribution[iAxis].Element[axisSurfelDistribution[iAxis].n].iMSurf = iMSurf;
				pSurfel = pSurfels->NodeArray.Element + iSurfel;
				axisSurfelDistribution[iAxis].Element[axisSurfelDistribution[iAxis].n].w = pSurfel->size;
				axisSurfelDistribution[iAxis].n++;
			}
		}
		for (iMEdge = 0; iMEdge < nModelEdges; iMEdge++)
		{
			iMSurf_[0] = pModel->edges.Element[iMEdge].iFace;
			iMSurf_[1] = pModel->edges.Element[iMEdge].iFace_;
			for (i = 0; i < edgeAssoc[iMEdge].n; i++)
			{
				iSurfel = edgeAssoc[iMEdge].Element[i];
				for (j = 0; j < 2; j++)
				{
					iAxis = pModel->info[iMSurf_[j]];
					axisSurfelDistribution[iAxis].Element[axisSurfelDistribution[iAxis].n].iQSurfel = iSurfel;
					axisSurfelDistribution[iAxis].Element[axisSurfelDistribution[iAxis].n].iMSurf = iMSurf_[j];
					pSurfel = pSurfels->NodeArray.Element + iSurfel;
					axisSurfelDistribution[iAxis].Element[axisSurfelDistribution[iAxis].n].w = pSurfel->size * edgeWeight;
					axisSurfelDistribution[iAxis].n++;
				}
			}
		}

		/// RANSAC.

		pHyp = hyps.Element;
		for (iSampling = 0; iSampling < nRandomSamplings; iSampling++)
		{
			// Random sampling of surfels from axis distribuions.

			for (iAxis = 0; iAxis < 6; iAxis++)
			{
				iTmp = 0;
				for (i = 0; i < axisSurfelDistribution[iAxis].n; i++)
					iTmp += axisSurfelDistribution[iAxis].Element[i].w;
				j = rand() % iTmp;
				iTmp = 0;
				for (i = 0; i < axisSurfelDistribution[iAxis].n; i++)
				{
					iTmp += axisSurfelDistribution[iAxis].Element[i].w;
					if (iTmp > j)
						break;
				}
				selectedSurfel[iAxis] = axisSurfelDistribution[iAxis].Element[i];
			}

			// Only for debugging purpose!!!

			// selectedSurfel[0].iMSurf = 9;
			// selectedSurfel[0].iQSurfel = 140;
			// selectedSurfel[1].iMSurf = 6;
			// selectedSurfel[1].iQSurfel = 168;
			// selectedSurfel[2].iMSurf = 1;
			// selectedSurfel[2].iQSurfel = 74;
			// selectedSurfel[3].iMSurf = 2;
			// selectedSurfel[3].iQSurfel = 148;
			// selectedSurfel[4].iMSurf = 8;
			// selectedSurfel[4].iQSurfel = 141;
			// selectedSurfel[5].iMSurf = 5;
			// selectedSurfel[5].iQSurfel = 201;

			// Dominant surfel.

			int iDominantSurfel = -1;
			int maxSurfelSize = 0;
			for (iAxis = 0; iAxis < 6; iAxis++)
			{
				iSurfel = selectedSurfel[iAxis].iQSurfel;
				pSurfel = pSurfels->NodeArray.Element + iSurfel;
				if (!pSurfel->bEdge)
				{
					if (pSurfel->size > maxSurfelSize)
					{
						maxSurfelSize = pSurfel->size;
						iDominantSurfel = iSurfel;
					}
				}
			}
			if (iDominantSurfel < 0)
				continue;

			// Secondary surfel for the pose.

			pSurfel = pSurfels->NodeArray.Element + iDominantSurfel;
			N = pSurfel->N;
			fTmp = (float)(pSurfel->size);
			RVLSCALE3VECTOR(N, fTmp, V3Tmp_);
			maxRefSurfelScore = 0.0f;
			int iSecondarySurfel = -1;
			for (iAxis = 0; iAxis < 6; iAxis++)
			{
				iSurfel = selectedSurfel[iAxis].iQSurfel;
				if (iSurfel == iDominantSurfel)
					continue;
				pSurfel = pSurfels->NodeArray.Element + iSurfel;
				if (pSurfel->bEdge)
					continue;
				if (RVLDOTPRODUCT3(N, pSurfel->N) >= csMinHypRefSurfaceAngle)
					continue;
				RVLCROSSPRODUCT3(V3Tmp_, pSurfel->N, V3Tmp);
				refSurfelScore = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp)) * (float)(pSurfel->size);
				if (refSurfelScore > maxRefSurfelScore)
				{
					maxRefSurfelScore = refSurfelScore;
					iSecondarySurfel = iSurfel;
				}
			}
			if (iSecondarySurfel < 0)
				continue;
			pSurfel = pSurfels->NodeArray.Element + iSecondarySurfel;
			RVLCROSSPRODUCT3(N, pSurfel->N, V);
			surfelRefPts = pSurfels->surfelRefPtMem + 12 * iSecondarySurfel + 3;
			bestAlignScore = 0.0f;
			for (i = 0; i < 3; i++)
			{
				P = surfelRefPts + 3 * i;
				j = (i + 1) % 3;
				P_ = surfelRefPts + 3 * j;
				RVLDIF3VECTORS(P_, P, dP);
				fTmp = RVLDOTPRODUCT3(V, dP);
				if (fTmp < 0.0f)
					fTmp = -fTmp;
				if (fTmp > bestAlignScore)
				{
					bestAlignScore = fTmp;
					secondarySrufelRefPts.a = i;
					secondarySrufelRefPts.b = j;
				}
			}

			// QPoint-MPlane association.

			pointPlaneAssoc = pointPlaneAssocMem;
			for (iAxis = 0; iAxis < 6; iAxis++)
			{
				iSurfel = selectedSurfel[iAxis].iQSurfel;
				if (iSurfel == iDominantSurfel)
				{
					surfelRefPts_.n = 3;
					RVLSET3VECTOR(surfelRefPts_.Element, 1, 2, 3);
				}
				else if (iSurfel == iSecondarySurfel)
				{
					surfelRefPts_.n = 2;
					surfelRefPts_.Element[0] = secondarySrufelRefPts.a + 1;
					surfelRefPts_.Element[1] = secondarySrufelRefPts.b + 1;
				}
				else
				{
					surfelRefPts_.n = 1;
					surfelRefPts_.Element[0] = 0;
				}
				surfelRefPts = pSurfels->surfelRefPtMem + 4 * 3 * iSurfel;
				iMSurf = selectedSurfel[iAxis].iMSurf;
				pSurfel = pSurfels->NodeArray.Element + iSurfel;
				for (i = 0; i < surfelRefPts_.n; i++)
				{
					pointPlaneAssoc->iMSurf = iMSurf;
					P_ = surfelRefPts + 3 * surfelRefPts_.Element[i];
					RVLCOPY3VECTOR(P_, pointPlaneAssoc->PQ);
					N = pModel->A + 3 * pModel->AID[iMSurf];
					RVLCOPY3VECTOR(N, pointPlaneAssoc->NM);
					pointPlaneAssoc++;
				}
			}

			// Generate hypothesis.

			a = A;
			pointPlaneAssoc = pointPlaneAssocMem;
			for (i = 0; i < 9; i++, pointPlaneAssoc++, a += 9)
			{
				u = pModel->M + 7 * pointPlaneAssoc->iMSurf;
				dFixed[i] = u[0] * storageVolumeWallThickness;
				memcpy(a, u + 1, 6 * sizeof(float));
			}
			RVLCOPYMX3X3T(pPrediction->pose.R, RSM);
			for (k = 0; k < nEquationSolvingIterations; k++)
			{
				ar = A + 6;
				pointPlaneAssoc = pointPlaneAssocMem;
				for (i = 0; i < 9; i++, ar += 9, pointPlaneAssoc++)
				{
					RVLMULMX3X3VECT(RSM, pointPlaneAssoc->PQ, PQR);
					RVLCROSSPRODUCT3(pointPlaneAssoc->NM, PQR, ar);
					E[i] = RVLDOTPRODUCT3(pointPlaneAssoc->NM, PQR) - dFixed[i];
				}
				cv::solve(cvA, cvE, cvx);
				th = sqrt(RVLDOTPRODUCT3(phi, phi));
				RVLSCALE3VECTOR2(phi, th, V);
				AngleAxisToRot<float>(V, th, dR);
				RVLMXMUL3X3(dR, RSM, M3x3Tmp);
				RVLCOPYMX3X3(M3x3Tmp, RSM);
			}
			// Only for debugging purpose!!!
			// pointPlaneAssoc = pointPlaneAssocMem;
			// for (i = 0; i < 9; i++, pointPlaneAssoc++)
			//{

			//	RVLMULMX3X3VECT(RSM, pointPlaneAssoc->PQ, PQR);
			//}
			//
			memcpy(pHyp->s, x, 6 * sizeof(float));
			RVLCOPYMX3X3T(RSM, pHyp->RMS);

			// Evaluate hypothesis.

			memcpy(s + 1, pHyp->s, 6 * sizeof(float));
			sceneFittingScore = EvaluateHypothesis(pMesh, pModel, 7, s, pHyp->RMS, chamferDistThr, nTransparentPts, SMCorrespondence);
			sceneFittingScore -= (float)nTransparentPts;
			if (sceneFittingScore > bestSceneFittingScore)
			{
				iBestHypothesis = pHyp - hyps.Element;
				bestSceneFittingScore = sceneFittingScore;
				printf("sampling %d, hypothesis %d, score %f\n", iSampling, pHyp - hyps.Element, sceneFittingScore);
				pHyp++;
				// RECOG::DisplayHypothesisEvaluation(pVisualizationData->pVisualizer, pMesh,
				//	ZBuffer, ZBufferActivePtArray, subImageMap, SMCorrespondence, nTransparentPts, hypothesisActor);
				RECOG::DisplayHypothesisEvaluation2(pVisualizationData->pVisualizer, pMesh,
													ZBuffer, ZBufferActivePtArray, subImageMap, SMCorrespondence, nTransparentPts, hypothesisActor);
				pVisualizationData->pVisualizer->Run();
				for (i = 0; i < 4; i++)
					pVisualizationData->pVisualizer->renderer->RemoveViewProp(hypothesisActor[i]);
			}

			//

			// break;	// Only for debugging purpose!!!
		}
		hyps.n = pHyp - hyps.Element;
		///
	}

	////

	/// Visualization.

	Pose3D poseMC;
	for (i = 0; i < predictions.n; i++)
	{
		RVLCOPYMX3X3(predictions.Element[i].pose.R, poseMC.R);
		RVLCOPY3VECTOR(predictions.Element[i].pose.t, poseMC.t);
		pVisualizationData->pVisualizer->DisplayBox(predictions.Element[i].bboxSize[0], predictions.Element[i].bboxSize[1],
													predictions.Element[i].bboxSize[2], &poseMC, 255, 0, 0, true);
	}
	Array<Point> refPts;
	refPts.Element = new Point[9];
	// refPts.n = 4;
	// for (i = 0; i < 4; i++)
	//{
	//	P_ = surfelRefPtMem + 3 * (4 * 104 + i);
	//	RVLCOPY3VECTOR(P_, refPts.Element[i].P);
	// }
	refPts.n = 9;
	pointPlaneAssoc = pointPlaneAssocMem;
	for (i = 0; i < 9; i++, pointPlaneAssoc++)
	{
		RVLCOPY3VECTOR(pointPlaneAssoc->PQ, refPts.Element[i].P);
	}
	pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(refPts, red, 10);
	Array<Point> samplePts;
	VisualizeStorageVolumeModel(pModel, hyps.Element[iBestHypothesis], samplePts);
	pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(samplePts, green, 6);
	pVisualizationData->pVisualizer->Run();
	delete[] refPts.Element;
	delete[] samplePts.Element;

	///

	// Free memory.

	delete[] predictions.Element;
	delete[] surfAssocMem;
	delete[] surfAssoc;
	delete[] edgeAssoc;
	delete[] NMSMem;
	delete[] axisSurfelDistributionMem;
	delete[] axisSurfelDistribution;
	delete[] pointPlaneAssocMem;
	delete[] hyps.Element;
	delete[] SMCorrespondence;
}

void DDDetector::DDOrthogonalView(
	RECOG::DDD::RectStruct *pRectStruct,
	Array<RECOG::DDD::EdgeLineSegment> edgeLineSegments,
	float *verticalAxis,
	int nOrthogonalViews,
	Pose3D *&poseFCOut,
	std::vector<cv::Mat> &orthogonalViews,
	Array<Array2D<uchar>> &masks,
	Array<RECOG::DDD::Line2D> *&orthogonalViewLines,
	bool bVisualize,
	Mesh *pMesh)
{
	// Parameters.

	float planeDistTol = 0.040f;   // m
	float normalOrientTol = 30.0f; // deg
	int dilationSize = 6;		   // pix

	// Constants.

	float pixSize = orthogonalViewPixSize;
	float mincsN = cos(normalOrientTol * DEG2RAD);
	float csEdgeLineAngleTol = cos(orthogonalViewEdgeLineAngleTol * DEG2RAD);

	// Determine vertical axis.

	int iAxis;
	float cs, abscs;
	float maxcs = 0.0f;
	int iVerticalAxis;
	float verticalAxisDirection;
	for (iAxis = 0; iAxis < 3; iAxis++)
	{
		cs = RVLMULVECTORCOL3(verticalAxis, pRectStruct->RRS, iAxis);
		abscs = RVLABS(cs);
		if (abscs > maxcs)
		{
			maxcs = abscs;
			iVerticalAxis = iAxis;
			verticalAxisDirection = (cs > 0.0f ? 1.0f : -1.0f);
		}
	}

	// Sort vertical rectangles.

	RECOG::DDD::Rect3D *pRect;
	int nVerticalRects = 0;
	for (int iRect = 0; iRect < pRectStruct->rects.n; iRect++)
	{
		pRect = pRectStruct->rects.Element + iRect;
		if (pRect->iAxis != iVerticalAxis)
			nVerticalRects++;
	}
	std::vector<SortIndex<float>> sortedVerticalRects(nVerticalRects);
	int iVerticalRect = 0;
	for (int iRect = 0; iRect < pRectStruct->rects.n; iRect++)
	{
		pRect = pRectStruct->rects.Element + iRect;
		if (pRect->iAxis == iVerticalAxis)
			continue;
		sortedVerticalRects[iVerticalRect].idx = iRect;
		sortedVerticalRects[iVerticalRect].cost = -pRect->s[0] * pRect->s[1];
		iVerticalRect++;
	}
	if (iVerticalRect == 0)
		return;
	std::sort(sortedVerticalRects.begin(), sortedVerticalRects.end(), RECOG::DDD::SortCompare);

	/// Create orthogonal views.

	orthogonalViewLines = new Array<RECOG::DDD::Line2D>[nOrthogonalViews];
	Array<RECOG::DDD::Line2D> *pOrthogonalViewLines = orthogonalViewLines;
	int i;
	cv::Mat orthogonalView;
	masks.Element = new Array2D<uchar>[nOrthogonalViews];
	Array2D<uchar> *pMask;
	int iPix, nPix;
	Point *pPt;
	int *piSrcPt;
	int iSrcPt, iTgtPt;
	float e;
	float dP[3];
	uchar *pMaskPix;
	poseFCOut = new Pose3D[nOrthogonalViews];
	for (int iOrthogonalView = 0; iOrthogonalView < nOrthogonalViews; iOrthogonalView++, pOrthogonalViewLines++)
	{
		// Pose of the face reference frame (F) with respect to the camera reference frame (C).

		pRect = pRectStruct->rects.Element + sortedVerticalRects[iOrthogonalView].idx;
		;
		Pose3D poseFC;
		Pose3D poseCF;
		float *RCF = poseCF.R;
		float *XFC = RCF;
		float *YFC = RCF + 3;
		float *ZFC = RCF + 6;
		RVLCOPYCOLMX3X3(pRectStruct->RRS, pRect->iAxis, ZFC);
		if (ZFC[2] < 0.0f)
		{
			RVLNEGVECT3(ZFC, ZFC);
		}
		RVLCOPYCOLMX3X3(pRectStruct->RRS, iVerticalAxis, YFC);
		if (verticalAxisDirection > 0.0f)
		{
			RVLNEGVECT3(YFC, YFC);
		}
		RVLCROSSPRODUCT3(YFC, ZFC, XFC);
		RVLCOPYMX3X3T(RCF, poseFC.R);
		int iAxis1 = (pRect->iAxis + 1) % 3;
		int iAxis2 = (pRect->iAxis + 2) % 3;
		float fw, fh;
		if (iAxis1 == iVerticalAxis)
		{
			fw = pRect->s[1];
			fh = pRect->s[0];
		}
		else
		{
			fw = pRect->s[0];
			fh = pRect->s[1];
		}
		float half_fw = 0.5f * fw;
		float half_fh = 0.5f * fh;
		float PcC[3];
		RVLMULMX3X3VECT(pRectStruct->RRS, pRect->c, PcC);
		float V3Tmp[3];
		RVLSCALE3VECTOR(XFC, -half_fw, V3Tmp);
		RVLSUM3VECTORS(PcC, V3Tmp, poseFC.t);
		RVLSCALE3VECTOR(YFC, -half_fh, V3Tmp);
		RVLSUM3VECTORS(poseFC.t, V3Tmp, poseFC.t);
		poseFCOut[iOrthogonalView] = poseFC;

		// Mapping between the original image and the orthogonal view.

		int w = (int)floor(fw / pixSize);
		int h = (int)floor(fh / pixSize);
		nPix = w * h;
		orthogonalView.create(h, w, CV_32SC1);
		orthogonalView.setTo(-1);
		int *tgtPix = (int *)(orthogonalView.data);
		int x, y, u, v;
		float PF[3], PC[3];
		PF[2] = 0.0f;
		float delta;
		Array<Point> visPts;
		Point *pVisPt;
		bool bVisualize3D = (pVisualizationData->b3DVisualization && pMesh != NULL);
		if (bVisualize3D)
		{
			visPts.Element = new Point[nPix];
			visPts.n = nPix;
			pVisPt = visPts.Element;
		}
		for (y = 0; y < h; y++)
			for (x = 0; x < w; x++, tgtPix++)
			{
				PF[0] = (float)x * pixSize;
				PF[1] = (float)y * pixSize;
				RVLTRANSF3(PF, poseFC.R, poseFC.t, PC);
				if (bVisualize3D)
				{
					RVLCOPY3VECTOR(PC, pVisPt->P);
					pVisPt++;
				}
				u = (int)round(camera.fu * PC[0] / PC[2] + camera.uc);
				v = (int)round(camera.fv * PC[1] / PC[2] + camera.vc);
				if (u < 0 || u >= camera.w || v < 0 || v >= camera.h)
					continue;
				iPix = u + v * camera.w;
				*tgtPix = iPix;
			}
		orthogonalViews.push_back(orthogonalView);

		// Mask the points which don't lie on the face.

		pMask = masks.Element + iOrthogonalView;
		pMask->Element = new uchar[nPix];
		pMask->w = w;
		pMask->h = h;
		memset(pMask->Element, 0, nPix * sizeof(uchar));
		pMaskPix = pMask->Element;
		piSrcPt = (int *)(orthogonalView.data);
		for (iTgtPt = 0; iTgtPt < nPix; iTgtPt++, piSrcPt++, pMaskPix++)
		{
			iSrcPt = *piSrcPt;
			pPt = pMesh->NodeArray.Element + iSrcPt;
			RVLDIF3VECTORS(pPt->P, poseFC.t, dP);
			e = RVLDOTPRODUCT3(ZFC, dP);
			if (RVLABS(e) > planeDistTol)
				continue;
			// if (-RVLDOTPRODUCT3(pPt->N, ZFC) < mincsN)
			//	continue;
			*pMaskPix = 1;
		}

		// Orthogonal projection of the edge line segments.

		RVLINVTRANSL(poseFC.R, poseFC.t, poseCF.t);
		Rect<float> ROI;
		ROI.minx = 0.0f;
		ROI.maxx = (float)w;
		ROI.miny = 0.0f;
		ROI.maxy = (float)h;
		int iEdgeLine;
		RECOG::DDD::EdgeLineSegment *pEdgeLine;
		float rayC[3];
		rayC[2] = 1.0f;
		float rayF[3];

		pOrthogonalViewLines->Element = new RECOG::DDD::Line2D[edgeLineSegments.n];
		float s;
		RECOG::DDD::Line2D *pEdgeLineF = pOrthogonalViewLines->Element;
		bool bInROI;
		float edgeLineLen;
		float V[2];
		float vx, vy;
		for (iEdgeLine = 0; iEdgeLine < edgeLineSegments.n; iEdgeLine++)
		{
			pEdgeLine = edgeLineSegments.Element + iEdgeLine;
			bInROI = false;
			for (i = 0; i < 2; i++)
			{
				rayC[0] = (pEdgeLine->P[i][0] - camera.uc) / camera.fu;
				rayC[1] = (pEdgeLine->P[i][1] - camera.vc) / camera.fv;
				RVLMULMX3X3VECT(poseCF.R, rayC, rayF);
				s = -poseCF.t[2] / rayF[2];
				RVLSCALE3VECTOR(rayF, s, V3Tmp);
				pEdgeLineF->P[i][0] = (poseCF.t[0] + V3Tmp[0]) / pixSize;
				pEdgeLineF->P[i][1] = (poseCF.t[1] + V3Tmp[1]) / pixSize;
				if (IsInRect<float>(pEdgeLineF->P[i][0], pEdgeLineF->P[i][1], ROI))
					bInROI = true;
			}
			if (!bInROI)
				continue;
			dP[0] = pEdgeLineF->P[1][0] - pEdgeLineF->P[0][0];
			dP[1] = pEdgeLineF->P[1][1] - pEdgeLineF->P[0][1];
			edgeLineLen = sqrt(dP[0] * dP[0] + dP[1] * dP[1]);
			V[0] = dP[0] / edgeLineLen;
			V[1] = dP[1] / edgeLineLen;
			vx = (V[0] >= 0.0f ? V[0] : -V[0]);
			vy = (V[1] >= 0.0f ? V[1] : -V[1]);
			if (RVLMAX(vx, vy) < csEdgeLineAngleTol)
				continue;
			pEdgeLineF->N[0] = -V[1];
			pEdgeLineF->N[1] = V[0];
			pEdgeLineF++;
		}
		pOrthogonalViewLines->n = pEdgeLineF - pOrthogonalViewLines->Element;

		// Visualization.

		if (bVisualize)
		{
			if (bVisualize3D)
			{
				pVisualizationData->pVisualizer->SetMesh(pMesh);
				uchar green[] = {0, 255, 0};
				pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(visPts, green, 6);
				RunVisualizer();
				ClearVisualization();
				delete[] visPts.Element;
			}
			cv::Mat displayImg;
			RECOG::DDD::MapMeshRGB(pMesh, orthogonalView, displayImg);
			uchar *displayPix = (uchar *)(displayImg.data);
			cv::Mat cvMask(h, w, CV_8UC1, pMask->Element);
			cv::Mat dilatedMask;
			int dilationStructElementSize = 2 * dilationSize + 1;
			cv::Mat dilationStructElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilationStructElementSize, dilationStructElementSize), cv::Point(dilationSize, dilationSize));
			cv::dilate(cvMask, dilatedMask, dilationStructElement);
			// pMaskPix = (uchar*)(dilatedMask.data);
			pMaskPix = pMask->Element;
			for (iPix = 0; iPix < nPix; iPix++, displayPix += 3, pMaskPix++)
				if (*pMaskPix == 0)
					RVLSET3VECTOR(displayPix, 255, 0, 0);
			std::string displayName = "Orthogonal view " + std::to_string(iOrthogonalView);
			pEdgeLineF = pOrthogonalViewLines->Element;
			for (i = 0; i < pOrthogonalViewLines->n; i++, pEdgeLineF++)
				cv::line(displayImg, cv::Point(pEdgeLineF->P[0][0], pEdgeLineF->P[0][1]), cv::Point(pEdgeLineF->P[1][0], pEdgeLineF->P[1][1]), cv::Scalar(0, 255, 0));
			cv::imshow(displayName, displayImg);
			cv::waitKey();
		}
	}
}

void DDDetector::SampleImage(Mesh *pMesh)
{
	Grid SSubsamplingGrid;
	SSubsamplingGrid.Create(pMesh->NodeArray, &camera, 4);
	Array<int> SPointSubset;
	SSubsamplingGrid.SubSample(pMesh->NodeArray, normalSimilarity, SPointSubset);
	RVL_DELETE_ARRAY(sampleMask);
	sampleMask = new bool[pMesh->NodeArray.n];
	memset(sampleMask, 0, pMesh->NodeArray.n * sizeof(bool));
	for (int i = 0; i < SPointSubset.n; i++)
		sampleMask[SPointSubset.Element[i]] = true;
	delete[] SPointSubset.Element;
}

void DDDetector::PointAssociation(
	Array<OrientedPoint> pointsM,
	AffinePose3D *pPose,
	Array<OrientedPoint> pointsQ,
	PointAssociationData &pointAssociationData,
	Array<int> ptBuff,
	bool bVisualize,
	Array<Point> *pPointsMQ)
{
	// Constants.

	float csNThr = cos(normalSimilarity * DEG2RAD);
	float csNVThr = sqrt(1.0f - csNThr * csNThr);
	float beta2 = beta * beta;

	//

	int *MNN = pointAssociationData.MNN;
	int *QNN = pointAssociationData.QNN;
	float *MNNDist = pointAssociationData.MNNDist;
	float *QNNDist = pointAssociationData.QNNDist;
	float *PMQ = pointAssociationData.PMQ;
	if (bVisualize)
		pPointsMQ->n = 0;
	pointAssociationData.MPts.n = 0;
	int i, j, iMPt, iQPt;
	float *PMQ_, *NMQ_;
	float dP[3], V3Tmp[3];
	OrientedPoint *pMPt, *pQPt;
	Point *pPt;
	for (iMPt = 0; iMPt < pointsM.n; iMPt++)
	{
		pointAssociationData.MPts.Element[pointAssociationData.MPts.n++] = iMPt;
		pMPt = pointsM.Element + iMPt;
		PMQ_ = PMQ + 6 * iMPt;
		NMQ_ = PMQ_ + 3;
		RVLATRANSF3(pMPt->P, pPose->s, pPose->R, pPose->t, PMQ_, V3Tmp);
		RVLMULMX3X3VECT(pPose->R, pMPt->N, NMQ_);
		if (bVisualize)
		{
			pPt = pPointsMQ->Element + pPointsMQ->n;
			RVLCOPY3VECTOR(PMQ_, pPt->P);
			pPointsMQ->n++;
		}
	}

	// Point association.

	memset(MNN, 0xff, pointsQ.n * sizeof(int));
	memset(QNN, 0xff, pointsM.n * sizeof(int));
	pointAssociationData.explainedQPts.n = 0;
	pointAssociationData.associatedMPts.n = 0;
	float maxMQPtDist = 10000.0f;
	float eP, csN;
	for (i = 0; i < pointAssociationData.MPts.n; i++)
	{
		iMPt = pointAssociationData.MPts.Element[i];
		pMPt = pointsM.Element + iMPt;
		PMQ_ = PMQ + 6 * iMPt;
		NMQ_ = PMQ_ + 3;
		if (RVLDOTPRODUCT3(NMQ_, PMQ_) >= 0.0f)
			continue;
		imgGrid.GetNeighbors(PMQ_, ptBuff);
		QNNDist[iMPt] = maxMQPtDist;
		for (j = 0; j < ptBuff.n; j++)
		{
			iQPt = ptBuff.Element[j];
			pQPt = pointsQ.Element + iQPt;
			RVLDIF3VECTORS(pQPt->P, PMQ_, dP);
			eP = RVLDOTPRODUCT3(dP, dP);
			if (eP > beta2)
				continue;
			csN = RVLDOTPRODUCT3(pQPt->N, NMQ_);
			if (csN < csNThr)
				continue;
			if (QNN[iMPt] < 0)
			{
				QNN[iMPt] = iQPt;
				QNNDist[iMPt] = eP;
				pointAssociationData.associatedMPts.Element[pointAssociationData.associatedMPts.n++] = iMPt;
			}
			else if (eP < QNNDist[iMPt])
			{
				QNN[iMPt] = iQPt;
				QNNDist[iMPt] = eP;
			}
			if (sampleMask[iQPt])
			{
				if (MNN[iQPt] < 0)
				{
					MNN[iQPt] = iMPt;
					MNNDist[iQPt] = eP;
					pointAssociationData.explainedQPts.Element[pointAssociationData.explainedQPts.n++] = iQPt;
				}
				else if (eP < MNNDist[iQPt])
				{
					MNN[iQPt] = iMPt;
					MNNDist[iQPt] = eP;
				}
			}
		}
	}
}

void DDDetector::PointToPlaneAssociation(
	RECOG::DDD::Model *pModel,
	float *q,
	Pose3D *pPose,
	Array<OrientedPoint> pointsQ,
	bool *bRejected,
	PointAssociationData &pointAssociationData)
{
	// Constants.

	float csNThr = cos(normalSimilarity * DEG2RAD);

	//

	int *MNN = pointAssociationData.MNN;

	// Plane offsets.

	float *d = new float[pModel->nSurfaces];
	int i_, j_;
	float *mxRow;
	RVLMULMXVECT(pModel->M, q, pModel->nSurfaces, 7, d, i_, j_, mxRow);

	// Point-to-plane association.

	pointAssociationData.explainedQPts.n = 0;
	pointAssociationData.associatedMPts.n = 0;
	int iQPt;
	OrientedPoint *pQPt;
	float PQM[3];
	float NQM[3];
	int iSurf;
	float *NM;
	float csN;
	float V3Tmp[3];
	float e;
	for (iQPt = 0; iQPt < pointsQ.n; iQPt++)
	{
		if (bRejected[iQPt])
			continue;
		// if (iQPt == 351)
		//	int debug = 0;
		pQPt = pointsQ.Element + iQPt;
		RVLINVTRANSF3(pQPt->P, pPose->R, pPose->t, PQM, V3Tmp);
		RVLMULMX3X3TVECT(pPose->R, pQPt->N, NQM);
		for (iSurf = 0; iSurf < pModel->nSurfaces; iSurf++)
		{
			NM = pModel->A + 3 * pModel->AID[iSurf];
			csN = RVLDOTPRODUCT3(NM, NQM);
			if (csN > csNThr)
				break;
		}
		if (iSurf >= pModel->nSurfaces)
		{
			MNN[iQPt] = -1;
			continue;
		}
		e = RVLDOTPRODUCT3(NM, PQM) - d[iSurf];
		if (RVLABS(e) > beta)
		{
			MNN[iQPt] = -1;
			continue;
		}
		MNN[iQPt] = iSurf;
		pointAssociationData.explainedQPts.Element[pointAssociationData.explainedQPts.n++] = iQPt;
	}

	delete[] d;
}

void DDDetector::VisualizePointToPlaneAssociations(
	RECOG::DDD::Model *pModel,
	float *q,
	Pose3D *pPose,
	Array<OrientedPoint> pointsQ,
	PointAssociationData &pointAssociationData,
	Array<Point> *pPointsMQ)
{
	float *d = new float[pModel->nSurfaces];
	int i_, j_;
	float *mxRow;
	RVLMULMXVECT(pModel->M, q, pModel->nSurfaces, 7, d, i_, j_, mxRow);

	int *MNN = pointAssociationData.MNN;

	int iQPt;
	OrientedPoint *pQPt;
	float PQM[3];
	int iSurf;
	float *NM;
	float V3Tmp[3];
	float e;
	Point *pPt;
	float NMQ[3];
	pPointsMQ->n = 0;
	for (int i = 0; i < pointAssociationData.explainedQPts.n; i++)
	{
		iQPt = pointAssociationData.explainedQPts.Element[i];
		pQPt = pointsQ.Element + iQPt;
		RVLINVTRANSF3(pQPt->P, pPose->R, pPose->t, PQM, V3Tmp);
		iSurf = MNN[iQPt];
		NM = pModel->A + 3 * pModel->AID[iSurf];
		e = RVLDOTPRODUCT3(NM, PQM) - d[iSurf];
		pPt = pPointsMQ->Element + pPointsMQ->n;
		RVLMULMX3X3VECT(pPose->R, NM, NMQ);
		RVLSCALE3VECTOR(NMQ, e, V3Tmp);
		RVLDIF3VECTORS(pQPt->P, V3Tmp, pPt->P);
		// float dP[3];
		// RVLDIF3VECTORS(pPt->P, pPose->t, dP);
		// float fTmp = RVLDOTPRODUCT3(NMQ, dP);
		pPointsMQ->n++;
	}

	delete[] d;
}

// #define RVLDDDAICP_DEBUG

void DDDetector::AICP(
	Array<OrientedPoint> pointsM,
	Array<OrientedPoint> pointsQ,
	AffinePose3D APoseInit,
	int nIterations,
	AffinePose3D &APose,
	PointAssociationData &pointAssociationData,
	Array<Point> *pPointsMQ)
{
	// Constants.

	double beta2 = beta * beta;
	double ks = alphas * beta2;
	double kR = alphaR * beta2;
	double kt = alphat * beta2;

	// Compute N * N for all model points and scale the model with the initial scale.

	Array<OrientedPoint> sPointsM;
	sPointsM.Element = new OrientedPoint[pointsM.n];
	sPointsM.n = pointsM.n;
	int iMPt;
	float *WMem = new float[9 * pointsM.n];
	float *W = WMem;
	OrientedPoint *pMPt = pointsM.Element;
	OrientedPoint *psMPt = sPointsM.Element;
	for (iMPt = 0; iMPt < pointsM.n; iMPt++, W += 9, pMPt++, psMPt++)
	{
		RVLVECTCOV3(pMPt->N, W);
		RVLCOMPLETESIMMX3(W);
		// W[0] += 0.1f; W[4] += 0.1f; W[8] += 0.1f;
		RVLSCALE3VECTOR3(pMPt->P, APoseInit.s, psMPt->P);
	}

	// Visualization.

	Visualizer *pVisualizer;
	if (bVisualize)
		pVisualizer = pVisualizationData->pVisualizer;

	/// ICP loop.

	APose = APoseInit;
	float RQM[9], tQM0[3];
	RVLINVTRANSF3D(APoseInit.R, APoseInit.t, RQM, tQM0);
	float tQM[3];
	RVLCOPY3VECTOR(tQM0, tQM);
	int *MNN = pointAssociationData.MNN;
	int *QNN = pointAssociationData.QNN;
	float *PMQ = pointAssociationData.PMQ;
	cv::Mat cvQ;
	cvQ.create(9, 9, CV_64FC1);
	double *Q = (double *)(cvQ.data);
	cv::Mat cvr;
	cvr.create(9, 1, CV_64FC1);
	double *r = (double *)(cvr.data);
	double *rs = r;
	double *rR = r + 3;
	double *rt = r + 6;
	cv::Mat cvx;
	cvx.create(9, 1, CV_64FC1);
	double *x = (double *)(cvx.data);
	memset(x, 0, 9 * sizeof(double));
	double *s = x;
	double *q_ = x + 3;
	double *t = x + 6;
	Array<int> ptBuff;
	ptBuff.Element = new int[pointsQ.n];
	Array<OrientedPoint> points[2];
	points[0] = pointsM;
	points[1] = pointsQ;
	Array<int> ptIdx[2];
	int *NN[2];
	NN[0] = QNN;
	NN[1] = MNN;
	float nrm[2];
	nrm[0] = 1.0f / (float)(pointsM.n);
	nrm[1] = 1.0f / (float)(pointsQ.n);
	int i, iPt, iPt_, iSet, it, iQPt;
	float PQM[3], PQM_[3], eP[3];
	float *W1, *W2, *W3;
	cv::Mat cva;
	cva.create(9, 3, CV_64FC1);
	double *a = (double *)(cva.data);
	double *as = a;
	RVLNULLMX3X3(as);
	double *aR = a + 9;
	double *at = a + 2 * 9;
	RVLUNITMX3(at);
	cv::Mat cvaW;
	cvaW.create(9, 3, CV_64FC1);
	double *aW = (double *)(cvaW.data);
	double *asW = aW;
	double *asW1 = asW;
	double *asW2 = asW1 + 3;
	double *asW3 = asW2 + 3;
	double *aRW = aW + 9;
	double *atW = aW + 2 * 9;
	cv::Mat cvaWa;
	cvaWa.create(9, 9, CV_64FC1);
	float aWe[9];
	float *asWe = aWe;
	float *aRWe = aWe + 3;
	float *atWe = aWe + 6;
	OrientedPoint *pQPt, *pPt, *pPt_;
	int nIn[2];
	double wp, ws, wR, wt;
	double q[3], b[3];
	RVLNULL3VECTOR(q);
	float th;
	float u[3], dR[9], newR[9];
#ifdef RVLDDDAICP_DEBUG
	float E, E0, E_, E__, eWe;
	float eW[3];
	// AffinePose3D APosePrev;
	float RQMPrev[9];
#endif

	for (it = 0; it <= nIterations; it++)
	{
		// Point association.

		PointAssociation(pointsM, &APose, pointsQ, pointAssociationData, ptBuff, bVisualize, pPointsMQ);

		// Visualization.

		if (bVisualize && (pVisualizationData->bVisualizeICPSteps || it == nIterations) &&
			(pVisualizationData->bVisualizeModelPts || pVisualizationData->bVisualizePtAssociation))
		{
			VisualizeHypothesis(APose, &pointAssociationData, *pPointsMQ);
			pVisualizer->Run();
			RemoveHypothesisFromVisualization();
		}

		// If the specified number of iterations is executed, then go out of the loop.

		if (it == nIterations)
			break;

		// Optimal pose.

		ptIdx[0] = pointAssociationData.associatedMPts;
		ptIdx[1] = pointAssociationData.explainedQPts;
		memset(Q, 0, 9 * 9 * sizeof(double));
		memset(r, 0, 9 * sizeof(double));

#ifdef RVLDDDAICP_DEBUG
		E = 0.0f;
#endif

		for (iSet = 0; iSet < 2; iSet++)
		{
			nIn[iSet] = 0;
			for (i = 0; i < ptIdx[iSet].n; i++)
			{
				iPt = ptIdx[iSet].Element[i];
				iPt_ = NN[iSet][iPt];
				if (iPt_ < 0)
					continue;
				nIn[iSet]++;
				pPt = points[iSet].Element + iPt;
				pPt_ = points[1 - iSet].Element + iPt_;
				if (iSet == 0)
				{
					iMPt = iPt;
					iQPt = iPt_;
				}
				else
				{
					iMPt = iPt_;
					iQPt = iPt;
				}
				psMPt = sPointsM.Element + iMPt;
				pQPt = pointsQ.Element + iQPt;
				W = WMem + 9 * iMPt;
				W1 = W;
				W2 = W1 + 3;
				W3 = W2 + 3;
				RVLMULMX3X3VECT(RQM, pQPt->P, PQM_);
				RVLSUM3VECTORS(PQM_, tQM0, PQM);
				RVLDIF3VECTORS(psMPt->P, PQM, eP);
				as[0] = -psMPt->P[0];
				as[4] = -psMPt->P[1];
				as[8] = -psMPt->P[2];
				RVLSKEW(PQM_, aR);
				RVLSCALE3VECTOR(W1, -psMPt->P[0], asW1);
				RVLSCALE3VECTOR(W2, -psMPt->P[1], asW2);
				RVLSCALE3VECTOR(W3, -psMPt->P[2], asW3);
				RVLMXMUL3X3(aR, W, aRW);
				RVLCOPYMX3X3(W, atW);
				cvaWa = cvaW * cva.t();
				cvQ = cvQ + cvaWa;
				RVLMULMX3X3VECT(asW, eP, asWe);
				RVLMULMX3X3VECT(aRW, eP, aRWe);
				RVLMULMX3X3VECT(atW, eP, atWe);
				RVLSUM3VECTORS(rs, asWe, rs);
				RVLSUM3VECTORS(rR, aRWe, rR);
				RVLSUM3VECTORS(rt, atWe, rt);
#ifdef RVLDDDAICP_DEBUG
				RVLMULMX3X3TVECT(W, eP, eW);
				eWe = RVLDOTPRODUCT3(eP, eW);
				E += eWe;
#endif
			}
		}

		wp = 1.0 / (double)(nIn[0] + nIn[1]);
		ws = ks * wp;
		wR = kR * wp;
		wt = kt * wp;
#ifdef RVLDDDAICP_DEBUG
		float es = ws * RVLDOTPRODUCT3(s, s);
		float q__[3];
		RVLSUM3VECTORS(q, q_, q__);
		float eR = wR * RVLDOTPRODUCT3(q__, q__);
		float et = wt * RVLDOTPRODUCT3(t, t);
		printf("Ep=%f ", E);
		E += (es + eR + et);
		printf("es=%f eR=%f et=%f : ", es, eR, et);
		printf("E=%f\n", E);
		RVLCOPYMX3X3(RQM, RQMPrev);
#endif
		Q[0] += ws;
		Q[10] += ws;
		Q[20] += ws;
		Q[30] += wR;
		Q[40] += wR;
		Q[50] += wR;
		Q[60] += wt;
		Q[70] += wt;
		Q[80] += wt;
		RVLSCALE3VECTOR(q, wR, b);
		RVLDIF3VECTORS(rR, b, rR);
		cv::solve(cvQ, cvr, cvx);
		th = sqrt(RVLDOTPRODUCT3(q_, q_));
		RVLSCALE3VECTOR2(q_, th, u);
		AngleAxisToRot<float>(u, th, dR);
		RVLMXMUL3X3(dR, RQM, newR);
		RVLCOPYMX3X3(newR, RQM);
		GetAngleAxis(RQM, u, th);
		RVLSCALE3VECTOR(u, th, q);
		RVLSUM3VECTORS(tQM0, t, tQM);
		RVLINVTRANSF3D(RQM, tQM, APose.R, APose.t);
		APose.s[0] = (1.0 + s[0]) * APoseInit.s[0];
		APose.s[1] = (1.0 + s[1]) * APoseInit.s[1];
		APose.s[2] = (1.0 + s[2]) * APoseInit.s[2];
#ifdef RVLDDDAICP_DEBUG
		E = 0.0f;
		E0 = 0.0f;
		E_ = 0.0f;
		E__ = 0.0f;
		for (iSet = 0; iSet < 2; iSet++)
		{
			for (i = 0; i < ptIdx[iSet].n; i++)
			{
				iPt = ptIdx[iSet].Element[i];
				iPt_ = NN[iSet][iPt];
				if (iPt_ < 0)
					continue;
				pPt = points[iSet].Element + iPt;
				pPt_ = points[1 - iSet].Element + iPt_;
				if (iSet == 0)
				{
					iMPt = iPt;
					iQPt = iPt_;
				}
				else
				{
					iMPt = iPt_;
					iQPt = iPt;
				}
				pMPt = pointsM.Element + iMPt;
				pQPt = pointsQ.Element + iQPt;
				W = WMem + 9 * iMPt;
				float sMP[3];
				RVLSCALE3VECTOR3(pMPt->P, APose.s, sMP);
				RVLTRANSF3(pQPt->P, RQM, tQM, PQM);
				RVLDIF3VECTORS(sMP, PQM, eP);
				// RVLATRANSF3(pMPt->P, APose.s, APose.R, APose.t, PMQ__, V3Tmp);
				// RVLDIF3VECTORS(pQPt->P, PMQ__, eP);
				RVLMULMX3X3TVECT(W, eP, eW);
				eWe = RVLDOTPRODUCT3(eP, eW);
				E += eWe;
				float PQM__[3];
				RVLCOPY3VECTOR(PQM, PQM__);
				// RVLMULMX3X3VECT(RQM, pQPt->P, PQM_);
				// RVLSUM3VECTORS(PQM_, tQM, PQM);
				// RVLDIF3VECTORS(psMPt->P, PQM, eP);
				psMPt = sPointsM.Element + iMPt;
				// RVLINVTRANSF3(psMPt->P, RQMPrev, tQM0, PMQ__, V3Tmp);
				// RVLDIF3VECTORS(pQPt->P, PMQ__, eP);
				RVLTRANSF3(pQPt->P, RQMPrev, tQM0, PQM);
				float eP0[3];
				RVLDIF3VECTORS(psMPt->P, PQM, eP0);
				RVLMULMX3X3TVECT(W, eP0, eW);
				eWe = RVLDOTPRODUCT3(eP0, eW);
				E0 += eWe;
				float deP[3], eP_[3], ass[3], aRq[3];
				RVLSCALE3VECTOR3(psMPt->P, s, ass);
				RVLMULMX3X3VECT(RQMPrev, pQPt->P, PQM_);
				RVLCROSSPRODUCT3(PQM_, q_, aRq);
				RVLSUM3VECTORS(ass, aRq, deP);
				RVLDIF3VECTORS(deP, t, deP);
				RVLSUM3VECTORS(eP0, deP, eP_);
				cv::Mat cvdeP;
				cvdeP.create(3, 1, CV_64FC1);
				double *deP_ = (double *)(cvdeP.data);
				float ass_[3], aRq_[3];
				as[0] = -psMPt->P[0];
				as[4] = -psMPt->P[1];
				as[8] = -psMPt->P[2];
				// RVLNEGVECT3(PQM_, PQM_);
				RVLSKEW(PQM_, aR);
				cvdeP = cva.t() * cvx;
				RVLMULMX3X3VECT(as, s, ass_);
				RVLMULMX3X3VECT(aR, q_, aRq_);
				float eP__[3];
				RVLDIF3VECTORS(eP0, deP_, eP__);
				RVLMULMX3X3TVECT(W, eP__, eW);
				eWe = RVLDOTPRODUCT3(eP__, eW);
				E__ += eWe;
				// float sMP_[3];
				// RVLSUM3VECTORS(psMPt->P, V3Tmp, sMP_);
				// RVLMULMX3X3VECT(RQMPrev, pQPt->P, PQM_);
				// RVLCROSSPRODUCT3(PQM_, q_, V3Tmp);
				// RVLDIF3VECTORS(PQM, V3Tmp, PQM);
				// RVLSUM3VECTORS(PQM, t, PQM);
				// RVLDIF3VECTORS(sMP_, PQM, eP);
				RVLMULMX3X3TVECT(W, eP_, eW);
				eWe = RVLDOTPRODUCT3(eP_, eW);
				E_ += eWe;
			}
		}
		cv::Mat cvdE = -2 * cvr.t() * cvx + cvx.t() * cvQ * cvx;
		float E___ = E0 + *(double *)(cvdE.data);
		printf("Ep0=%f Ep=%f Elin=%f Elin2=%f Elin3=%f\n", E0, E, E_, E__, E___);
#endif
	}

	///

	delete[] ptBuff.Element;
	delete[] WMem;
	delete[] sPointsM.Element;
}

// #define RVLDDDRICP_DEBUG

void DDDetector::RICP(
	RECOG::DDD::Model *pModel,
	Array<OrientedPoint> pointsQ,
	AffinePose3D APoseInit,
	int nIterations,
	AffinePose3D &APose,
	PointAssociationData &pointAssociationData,
	Array<Point> *pPointsMQ)
{
	// Constants.

	double beta2 = beta * beta;
	double ks = alphas * beta2;
	double kR = alphaR * beta2;
	double kt = alphat * beta2;

	// Compute initial model plane offsets.

	float z0[7];
	memset(z0, 0, 7 * sizeof(float));
	// float* s0 = z0 + 1;
	// RVLSCALE3VECTOR(pModel->bboxSize, 0.5f, s0);
	z0[1] = 0.5f * pModel->bboxSize[0] * APoseInit.s[0];
	z0[2] = 0.5f * pModel->bboxSize[1] * APoseInit.s[1];
	z0[3] = 0.5f * pModel->bboxSize[2] * APoseInit.s[2];
	float *d = new float[pModel->nSurfaces];
	int i_, j_;
	float *mxRow;
	RVLMULMXVECT(pModel->M, z0, pModel->nSurfaces, 7, d, i_, j_, mxRow);

	// Visualization.

	Visualizer *pVisualizer;
	if (bVisualize)
		pVisualizer = pVisualizationData->pVisualizer;

	/// ICP loop.

	float *A = new float[9 * pointsQ.n];
	float *a_;
	float *c = new float[pointsQ.n];
	bool *bRejected = new bool[pointsQ.n];
	memset(bRejected, 0, pointsQ.n * sizeof(bool));
	float *NM;
	float z[7];
	memset(z, 0, 7 * sizeof(float));
	APose = APoseInit;
	float RQM[9], tQM0[3];
	RVLINVTRANSF3D(APoseInit.R, APoseInit.t, RQM, tQM0);
	float tQM[3];
	RVLCOPY3VECTOR(tQM0, tQM);
	int *MNN = pointAssociationData.MNN;
	float *PMQ = pointAssociationData.PMQ;
	cv::Mat cvQ;
	cvQ.create(9, 9, CV_64FC1);
	double *Q = (double *)(cvQ.data);
	cv::Mat cvr;
	cvr.create(9, 1, CV_64FC1);
	double *r = (double *)(cvr.data);
	double *rs = r;
	double *rR = r + 3;
	double *rt = r + 6;
	cv::Mat cvx;
	cvx.create(9, 1, CV_64FC1);
	double *x = (double *)(cvx.data);
	memset(x, 0, 9 * sizeof(double));
	double *s = x;
	double *q_ = x + 3;
	double *t = x + 6;
	Array<int> ptBuff;
	ptBuff.Element = new int[pointsQ.n];
	float nrm = 1.0f / (float)(pointsQ.n);
	int i, iMSurf, it, iQPt;
	float PQM[3], PQM_[3];
	float e;
	cv::Mat cva;
	cva.create(9, 1, CV_64FC1);
	double *a = (double *)(cva.data);
	double *as = a;
	double *aR = a + 3;
	double *at = a + 2 * 3;
	cv::Mat cvaa;
	cvaa.create(9, 9, CV_64FC1);
	float ae[9];
	float *ase = ae;
	float *aRe = ae + 3;
	float *ate = ae + 6;
	OrientedPoint *pQPt, *pPt;
	double wp, ws, wR, wt;
	double q[3], b[3];
	RVLNULL3VECTOR(q);
	float th;
	float u[3], dR[9], newR[9];
	Pose3D pose;
#ifdef RVLDDDRICP_DEBUG
	float E, E0, E_, E__, eWe;
	float e2;
	// AffinePose3D APosePrev;
	float RQMPrev[9];
	float *dPrev = new float[pModel->nSurfaces];
	float *dNew = new float[pModel->nSurfaces];
	float zNew[7];
	memset(zNew, 0, 7 * sizeof(float));
#endif
	int nAssoc;
	float e0, fTmp;

	for (it = 0; it <= nIterations; it++)
	{
		// Point association.

		z[1] = 0.5f * pModel->bboxSize[0] * APose.s[0];
		z[2] = 0.5f * pModel->bboxSize[1] * APose.s[1];
		z[3] = 0.5f * pModel->bboxSize[2] * APose.s[2];
		RVLCOPYMX3X3(APose.R, pose.R);
		RVLCOPY3VECTOR(APose.t, pose.t);
		// if(it == 0)
		PointToPlaneAssociation(pModel, z, &pose, pointsQ, bRejected, pointAssociationData);

		// Visualization.

		if (bVisualize && (pVisualizationData->bVisualizeICPSteps || it == nIterations) &&
			(pVisualizationData->bVisualizeModelPts || pVisualizationData->bVisualizePtAssociation))
		{
			if (pVisualizationData->bVisualizePtAssociation)
				VisualizePointToPlaneAssociations(pModel, z, &pose, pointsQ, pointAssociationData, pPointsMQ);
			VisualizeHypothesis(APose, &pointAssociationData, *pPointsMQ, &pointsQ);
			pVisualizer->Run();
			if (it < nIterations)
				RemoveHypothesisFromVisualization();
		}

		// If the specified number of iterations is executed, then go out of the loop.

		if (it == nIterations)
			break;

		// Optimal pose.

		memset(Q, 0, 9 * 9 * sizeof(double));
		memset(r, 0, 9 * sizeof(double));

#ifdef RVLDDDRICP_DEBUG
		E = 0.0f;
		RVLMULMXVECT(pModel->M, z, pModel->nSurfaces, 7, dPrev, i_, j_, mxRow);
#endif

		nAssoc = 0;
		for (i = 0; i < pointAssociationData.explainedQPts.n; i++)
		{
			iQPt = pointAssociationData.explainedQPts.Element[i];
			iMSurf = MNN[iQPt];
			if (iMSurf < 0)
				continue;
			// if (iMSurf == 5)
			//	int debug = 0;
			pPt = pointsQ.Element + iQPt;
			NM = pModel->A + 3 * pModel->AID[iMSurf];
			pQPt = pointsQ.Element + iQPt;
			RVLMULMX3X3VECT(RQM, pQPt->P, PQM_);
			RVLSUM3VECTORS(PQM_, tQM0, PQM);
			e = d[iMSurf] - RVLDOTPRODUCT3(NM, PQM);
			as[0] = -d[iMSurf] * RVLABS(NM[0]);
			as[1] = -d[iMSurf] * RVLABS(NM[1]);
			as[2] = -d[iMSurf] * RVLABS(NM[2]);
			RVLCROSSPRODUCT3(PQM_, NM, aR);
			RVLCOPY3VECTOR(NM, at);
			a_ = A + 9 * i;
			RVLCOPY3VECTOR(as, a_);
			a_ += 3;
			RVLCOPY3VECTOR(aR, a_);
			a_ += 3;
			RVLCOPY3VECTOR(at, a_);
			c[i] = e;
			nAssoc++;
			cvaa = cva * cva.t();
			cvQ = cvQ + cvaa;
			RVLSCALE3VECTOR(as, e, ase);
			RVLSCALE3VECTOR(aR, e, aRe);
			RVLSCALE3VECTOR(at, e, ate);
			RVLSUM3VECTORS(rs, ase, rs);
			RVLSUM3VECTORS(rR, aRe, rR);
			RVLSUM3VECTORS(rt, ate, rt);
#ifdef RVLDDDRICP_DEBUG
			RVLSUM3VECTORS(PQM_, tQM, PQM)
			e = dPrev[iMSurf] - RVLDOTPRODUCT3(NM, PQM);
			e2 = e * e;
			E += e2;
#endif
		}

		wp = 1.0 / (double)nAssoc;
		ws = ks * wp;
		wR = kR * wp;
		wt = kt * wp;
#ifdef RVLDDDRICP_DEBUG
		float es = ws * RVLDOTPRODUCT3(s, s);
		float q__[3];
		RVLSUM3VECTORS(q, q_, q__);
		float eR = wR * RVLDOTPRODUCT3(q__, q__);
		float et = wt * RVLDOTPRODUCT3(t, t);
		printf("Ep=%f ", E);
		E += (es + eR + et);
		printf("es=%f eR=%f et=%f : ", es, eR, et);
		printf("E=%f\n", E);
		RVLCOPYMX3X3(RQM, RQMPrev);
		float tQMPrev[3];
		RVLCOPY3VECTOR(tQM, tQMPrev);
#endif
		Q[0] += ws;
		Q[10] += ws;
		Q[20] += ws;
		Q[30] += wR;
		Q[40] += wR;
		Q[50] += wR;
		Q[60] += wt;
		Q[70] += wt;
		Q[80] += wt;
		RVLSCALE3VECTOR(q, wR, b);
		RVLDIF3VECTORS(rR, b, rR);
		cv::solve(cvQ, cvr, cvx);
		memset(Q, 0, 9 * 9 * sizeof(double));
		memset(r, 0, 9 * sizeof(double));
		nAssoc = 0;
		memset(bRejected, 0, pointsQ.n * sizeof(bool));
		FILE *fp = fopen((std::string(resultsFolder) + "\\assoc_clustering.txt").data(), "w");
		for (i = 0; i < pointAssociationData.explainedQPts.n; i++)
		{
			iQPt = pointAssociationData.explainedQPts.Element[i];
			iMSurf = MNN[iQPt];
			if (iMSurf < 0)
				continue;
			a_ = A + 9 * i;
			RVLDOTPRODUCT(a_, x, 9, fTmp, i_);
			e0 = c[i];
			e = e0 - fTmp;
			if (RVLABS(e) <= 0.02f)
			{
				RVLCOPY3VECTOR(a_, as);
				a_ += 3;
				RVLCOPY3VECTOR(a_, aR);
				a_ += 3;
				RVLCOPY3VECTOR(a_, at);
				nAssoc++;
				cvaa = cva * cva.t();
				cvQ = cvQ + cvaa;
				RVLSCALE3VECTOR(as, e0, ase);
				RVLSCALE3VECTOR(aR, e0, aRe);
				RVLSCALE3VECTOR(at, e0, ate);
				RVLSUM3VECTORS(rs, ase, rs);
				RVLSUM3VECTORS(rR, aRe, rR);
				RVLSUM3VECTORS(rt, ate, rt);
			}
			else
				bRejected[iQPt] = true;
			fprintf(fp, "%f\n", e);
		}
		fclose(fp);
		wp = 1.0 / (double)nAssoc;
		ws = ks * wp;
		wR = kR * wp;
		wt = kt * wp;
		Q[0] += ws;
		Q[10] += ws;
		Q[20] += ws;
		Q[30] += wR;
		Q[40] += wR;
		Q[50] += wR;
		Q[60] += wt;
		Q[70] += wt;
		Q[80] += wt;
		RVLSCALE3VECTOR(q, wR, b);
		RVLDIF3VECTORS(rR, b, rR);
		cv::solve(cvQ, cvr, cvx);
		// RVLNULL3VECTOR(q_);
		th = sqrt(RVLDOTPRODUCT3(q_, q_));
		printf("th=%f\n", th);
		if (th > 1e-10)
		{
			RVLSCALE3VECTOR2(q_, th, u);
			AngleAxisToRot<float>(u, th, dR);
		}
		else
		{
			RVLUNITMX3(dR);
		}
		RVLMXMUL3X3(dR, RQM, newR);
		RVLCOPYMX3X3(newR, RQM);
		GetAngleAxis(RQM, u, th);
		RVLSCALE3VECTOR(u, th, q);
		RVLSUM3VECTORS(tQM0, t, tQM);
		RVLINVTRANSF3D(RQM, tQM, APose.R, APose.t);
		APose.s[0] = (1.0 + s[0]) * APoseInit.s[0];
		APose.s[1] = (1.0 + s[1]) * APoseInit.s[1];
		APose.s[2] = (1.0 + s[2]) * APoseInit.s[2];
		// APose.s[2] = 1.0;
#ifdef RVLDDDRICP_DEBUG
		zNew[1] = 0.5f * pModel->bboxSize[0] * APose.s[0];
		zNew[2] = 0.5f * pModel->bboxSize[1] * APose.s[1];
		zNew[3] = 0.5f * pModel->bboxSize[2] * APose.s[2];
		RVLMULMXVECT(pModel->M, zNew, pModel->nSurfaces, 7, dNew, i_, j_, mxRow);
		float dRlin[9];
		RVLSKEW(q_, dRlin);
		dRlin[0] = dRlin[4] = dRlin[8] = 1.0f;
		float RQMlin[9];
		RVLMXMUL3X3(dRlin, RQMPrev, RQMlin);
		E = 0.0f;
		E0 = 0.0f;
		E_ = 0.0f;
		E__ = 0.0f;
		for (i = 0; i < pointAssociationData.explainedQPts.n; i++)
		{
			iQPt = pointAssociationData.explainedQPts.Element[i];
			iMSurf = MNN[iQPt];
			if (iMSurf < 0)
				continue;
			pPt = pointsQ.Element + iQPt;
			NM = pModel->A + 3 * pModel->AID[iMSurf];
			pQPt = pointsQ.Element + iQPt;

			// Non-linear error.

			RVLTRANSF3(pQPt->P, RQM, tQM, PQM);
			e = dNew[iMSurf] - RVLDOTPRODUCT3(NM, PQM);
			e2 = e * e;
			E += e2;

			// Linear error for x = 0.

			float PQM__[3];
			RVLMULMX3X3VECT(RQMPrev, pQPt->P, PQM_);
			RVLSUM3VECTORS(PQM_, tQM0, PQM__);
			e0 = d[iMSurf] - RVLDOTPRODUCT3(NM, PQM__);
			e2 = e0 * e0;
			E0 += e2;

			// Linear error.

			as[0] = -d[iMSurf] * RVLABS(NM[0]);
			as[1] = -d[iMSurf] * RVLABS(NM[1]);
			as[2] = -d[iMSurf] * RVLABS(NM[2]);
			RVLCROSSPRODUCT3(PQM_, NM, aR);
			RVLCOPY3VECTOR(NM, at);
			float ass = RVLDOTPRODUCT3(as, s);
			float aRq = RVLDOTPRODUCT3(aR, q_);
			float att = RVLDOTPRODUCT3(at, t);
			float eLin = e0 - (ass + aRq + att);
			e2 = eLin * eLin;
			E_ += e2;
			if (iMSurf == 5)
				int debug = 0;
		}
		cv::Mat cvdE = -2 * cvr.t() * cvx + cvx.t() * cvQ * cvx;
		float E___ = E0 + *(double *)(cvdE.data);
		printf("Ep0=%f Ep=%f Elin=%f Elin2=%f Elin3=%f\n", E0, E, E_, E__, E___);
#endif
	}

	///

	delete[] ptBuff.Element;
	delete[] d;
	delete[] A;
	delete[] c;
	delete[] bRejected;
#ifdef RVLDDDRICP_DEBUG
	delete[] dNew;
	delete[] dPrev;
#endif
}

void DDDetector::TICP(
	Array<OrientedPoint> pointsM,
	Array<OrientedPoint> pointsQ,
	AffinePose3D APoseInit,
	int nIterations,
	AffinePose3D &APose,
	PointAssociationData &pointAssociationData,
	Array<Point> *pPointsMQ)
{
	// Constants.

	double beta2 = beta * beta;
	double kt = alphat * beta2;

	// Compute N * N for all model points and scale the model with the initial scale.

	Array<OrientedPoint> sPointsM;
	sPointsM.Element = new OrientedPoint[pointsM.n];
	sPointsM.n = pointsM.n;
	int iMPt;
	float *WMem = new float[9 * pointsM.n];
	float *W = WMem;
	OrientedPoint *pMPt = pointsM.Element;
	OrientedPoint *psMPt = sPointsM.Element;
	for (iMPt = 0; iMPt < pointsM.n; iMPt++, W += 9, pMPt++, psMPt++)
	{
		RVLVECTCOV3(pMPt->N, W);
		RVLCOMPLETESIMMX3(W);
		// W[0] += 0.1f; W[4] += 0.1f; W[8] += 0.1f;
		RVLSCALE3VECTOR3(pMPt->P, APoseInit.s, psMPt->P);
	}

	// Visualization.

	Visualizer *pVisualizer;
	if (bVisualize)
		pVisualizer = pVisualizationData->pVisualizer;

	/// ICP loop.

	APose = APoseInit;
	float RQM[9], tQM0[3];
	RVLINVTRANSF3D(APoseInit.R, APoseInit.t, RQM, tQM0);
	float tQM[3];
	RVLCOPY3VECTOR(tQM0, tQM);
	int *MNN = pointAssociationData.MNN;
	int *QNN = pointAssociationData.QNN;
	float *PMQ = pointAssociationData.PMQ;
	cv::Mat cvQ;
	cvQ.create(3, 3, CV_64FC1);
	double *Q = (double *)(cvQ.data);
	cv::Mat cvr;
	cvr.create(3, 1, CV_64FC1);
	double *r = (double *)(cvr.data);
	cv::Mat cvx;
	cvx.create(3, 1, CV_64FC1);
	double *x = (double *)(cvx.data);
	memset(x, 0, 3 * sizeof(double));
	Array<int> ptBuff;
	ptBuff.Element = new int[pointsQ.n];
	Array<OrientedPoint> points[2];
	points[0] = pointsM;
	points[1] = pointsQ;
	Array<int> ptIdx[2];
	int *NN[2];
	NN[0] = QNN;
	NN[1] = MNN;
	float nrm[2];
	nrm[0] = 1.0f / (float)(pointsM.n);
	nrm[1] = 1.0f / (float)(pointsQ.n);
	int i, iPt, iPt_, iSet, it, iQPt;
	float PQM[3], PQM_[3], eP[3], We[3];
	float *W1, *W2, *W3;
	OrientedPoint *pQPt, *pPt, *pPt_;
	int nIn[2];
	double wp, wt;

	for (it = 0; it <= nIterations; it++)
	{
		// Point association.

		PointAssociation(pointsM, &APose, pointsQ, pointAssociationData, ptBuff, bVisualize, pPointsMQ);

		// Visualization.

		if (bVisualize && (pVisualizationData->bVisualizeICPSteps || it == nIterations))
		{
			VisualizeHypothesis(APose, &pointAssociationData, *pPointsMQ);
			pVisualizer->Run();
			RemoveHypothesisFromVisualization();
		}

		// If the specified number of iterations is executed, then go out of the loop.

		if (it == nIterations)
			break;

		// Optimal pose.

		ptIdx[0] = pointAssociationData.associatedMPts;
		ptIdx[1] = pointAssociationData.explainedQPts;
		memset(Q, 0, 3 * 3 * sizeof(double));
		memset(r, 0, 3 * sizeof(double));

		for (iSet = 0; iSet < 2; iSet++)
		{
			nIn[iSet] = 0;
			for (i = 0; i < ptIdx[iSet].n; i++)
			{
				iPt = ptIdx[iSet].Element[i];
				iPt_ = NN[iSet][iPt];
				if (iPt_ < 0)
					continue;
				nIn[iSet]++;
				pPt = points[iSet].Element + iPt;
				pPt_ = points[1 - iSet].Element + iPt_;
				if (iSet == 0)
				{
					iMPt = iPt;
					iQPt = iPt_;
				}
				else
				{
					iMPt = iPt_;
					iQPt = iPt;
				}
				psMPt = sPointsM.Element + iMPt;
				pQPt = pointsQ.Element + iQPt;
				W = WMem + 9 * iMPt;
				W1 = W;
				W2 = W1 + 3;
				W3 = W2 + 3;
				RVLMULMX3X3VECT(RQM, pQPt->P, PQM_);
				RVLSUM3VECTORS(PQM_, tQM0, PQM);
				RVLDIF3VECTORS(psMPt->P, PQM, eP);
				RVLSUMMX3X3(Q, W, Q);
				RVLMULMX3X3VECT(W, eP, We);
				RVLSUM3VECTORS(r, We, r);
			}
		}
		wp = 1.0 / (double)(nIn[0] + nIn[1]);
		wt = kt * wp;
		Q[0] += wt;
		Q[4] += wt;
		Q[8] += wt;
		cv::solve(cvQ, cvr, cvx);
		x[0] = x[1] = 0.0f;
		RVLSUM3VECTORS(tQM0, x, tQM);
		RVLINVTRANSF3D(RQM, tQM, APose.R, APose.t);
	}

	///

	delete[] ptBuff.Element;
	delete[] WMem;
	delete[] sPointsM.Element;
}

void DDDetector::FitRectangle(
	Mesh *pMesh,
	SurfelGraph *pSurfels_,
	int iSurfel,
	float *RBS,
	Box<float> bboxInit,
	float thr,
	Rect<float> *pRect)
{
	Surfel *pSurfel = pSurfels_->NodeArray.Element + iSurfel;
	float *boundaryMem = new float[4 * pSurfel->size];
	Array<float> boundaryPts[4];
	int i;
	for (i = 0; i < 4; i++)
	{
		boundaryPts[i].Element = boundaryMem + i * pSurfel->size;
		boundaryPts[i].n = 0;
	}
	// Array<Point> ptArray;
	// ptArray.Element = new Point[pSurfel->size];
	// ptArray.n = 0;
	// Point* pPt;
	float *PC;
	float PC_[3], PB[3];
	QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;
	MeshEdgePtr *pEdgePtr;
	float d[4];
	float absd, s;
	int iBoundary;
	while (pPtIdx)
	{
		if (pMesh->IsBoundaryPoint(pPtIdx->Idx, pSurfels_->surfelMap, iSurfel, pEdgePtr))
		{
			// pPt = ptArray.Element + ptArray.n++;
			//*pPt = pMesh->NodeArray.Element[pPtIdx->Idx];
			PC = pMesh->NodeArray.Element[pPtIdx->Idx].P;
			s = pSurfel->d / RVLDOTPRODUCT3(pSurfel->N, PC);
			RVLSCALE3VECTOR(PC, s, PC_);
			RVLMULMX3X3TVECT(RBS, PC_, PB);
			d[0] = PB[1] - bboxInit.miny;
			d[1] = PB[1] - bboxInit.maxy;
			d[2] = PB[2] - bboxInit.minz;
			d[3] = PB[2] - bboxInit.maxz;
			for (iBoundary = 0; iBoundary < 4; iBoundary++)
			{
				absd = RVLABS(d[iBoundary]);
				if (absd <= thr)
					boundaryPts[iBoundary].Element[boundaryPts[iBoundary].n++] = d[iBoundary];
			}
		}
		pPtIdx = pPtIdx->pNext;
	}
	int j;
	for (i = 0; i < 4; i++)
	{
		d[i] = 0.0f;
		for (j = 0; j < boundaryPts[i].n; j++)
			d[i] += boundaryPts[i].Element[j];
		if (boundaryPts[i].n > 0)
			d[i] /= (float)(boundaryPts[i].n);
	}
	delete[] boundaryMem;
	pRect->minx = bboxInit.miny + d[0];
	pRect->maxx = bboxInit.maxy + d[1];
	pRect->miny = bboxInit.minz + d[2];
	pRect->maxy = bboxInit.maxz + d[3];
	// uchar green[] = {0, 255, 0};
	// pVisualizationData->pVisualizer->SetMesh(pMesh);
	// pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(ptArray, green, 3.0);
	// RunVisualizer();
	// ClearVisualization();
	// delete[] ptArray.Element;
}

// #define RVLDDD_RECTSTRUCT_DEBUG

float DDDetector::RectangularStructure(
	int iSurfel1,
	int iSurfel2,
	bool bEdge,
	float beta,
	RECOG::DDD::RectStruct *pRectStruct,
	bool bComputeRSS,
	bool bOptimize,
	Mesh *pMesh)
{
	// Parameters.

	float edgeUncertCoeff = 0.2f;

	// Constants.

	float maxEdgeParentDist = 2.0f / pSurfelDetector->kPlane;

	// Initial orientation from referent surfels.

	float *RRS = pRectStruct->RRS;
	float RSR[9];
	float fTmp;
	if (bComputeRSS)
	{
		Surfel *pSurfel1 = planarSurfaces.NodeArray.Element + iSurfel1;
		Surfel *pSurfel2 = (bEdge ? pSurfels->NodeArray.Element : planarSurfaces.NodeArray.Element) + iSurfel2;
		float *XRS = RSR;
		float *YRS = RSR + 3;
		float *ZRS = RSR + 6;
		RVLCOPY3VECTOR(pSurfel1->N, XRS);
		RVLCROSSPRODUCT3(XRS, pSurfel2->N, ZRS);
		RVLNORM3(ZRS, fTmp);
		RVLCROSSPRODUCT3(ZRS, XRS, YRS);
		RVLCOPYMX3X3T(RSR, RRS);
	}
	else
		RVLCOPYMX3X3T(RRS, RSR);

	// Point-to-plane association.

	struct PtAssoc2
	{
		float PQ[3];
		float NM[3];
		uchar i;
		uchar n;
		int iSurfel;
		int iParent;
	};
	Array<PtAssoc2> ptAssocs;
	ptAssocs.Element = new PtAssoc2[3 * planarSurfaces.NodeArray.n + 2 * surfelEdges.n];
	PtAssoc2 *pPtAssoc = ptAssocs.Element;
	int i, j;
	float *surfelRefPts;
	float NQS[3], NQR[3];
	int iAxis;
	float *PQ;
	float NM[3];
	float V3Tmp[3];
	int iSurfel, iSurfel_;
	Surfel *pSurfel, *pSurfel_;
	RECOG::DDD::SurfelEdge *pSurfelEdge;
	for (iSurfel = 0; iSurfel < planarSurfaces.NodeArray.n; iSurfel++)
	{
		pSurfel = planarSurfaces.NodeArray.Element + iSurfel;
		if (pSurfel->bEdge)
			continue;
		if (pSurfel->size < minSurfelSize)
			continue;
		// if (iSurfel == 2410)
		//	int debug = 0;
		RVLMULMX3X3VECT(RSR, pSurfel->N, NQR);
		iAxis = (RVLABS(NQR[0]) >= RVLABS(NQR[1]) ? iAxis = 0 : iAxis = 1);
		if (RVLABS(NQR[2]) >= RVLABS(NQR[iAxis]))
			iAxis = 2;
		RVLNULL3VECTOR(NM);
		NM[iAxis] = (NQR[iAxis] > 0.0f ? 1.0f : -1.0f);
		surfelRefPts = pSurfel->representativePts;
		for (i = 1; i < 4; i++)
		{
			PQ = surfelRefPts + 3 * i;
			RVLCOPY3VECTOR(PQ, pPtAssoc->PQ);
			RVLCOPY3VECTOR(NM, pPtAssoc->NM);
			pPtAssoc->iSurfel = iSurfel;
			pPtAssoc->iParent = -1;
			pPtAssoc->i = i - 1;
			pPtAssoc->n = 3;
			pPtAssoc++;
		}
		for (i = 0; i < planarSurfeceEdges[iSurfel].n; i++)
		{
			pSurfelEdge = planarSurfeceEdges[iSurfel].Element[i];
			RVLMULMX3X3VECT(RSR, pSurfelEdge->N, NQR);
			iAxis = (RVLABS(NQR[0]) >= RVLABS(NQR[1]) ? iAxis = 0 : iAxis = 1);
			if (RVLABS(NQR[2]) >= RVLABS(NQR[iAxis]))
				iAxis = 2;
			RVLNULL3VECTOR(NM);
			NM[iAxis] = (NQR[iAxis] > 0.0f ? 1.0f : -1.0f);
			for (j = 0; j < 2; j++)
			{
				PQ = pSurfelEdge->P[j];
				fTmp = pSurfel->d / RVLDOTPRODUCT3(pSurfel->N, PQ);
				RVLSCALE3VECTOR(PQ, fTmp, pPtAssoc->PQ);
				RVLCOPY3VECTOR(NM, pPtAssoc->NM);
				pPtAssoc->iSurfel = pSurfelEdge->iEdgeSurfel;
				pPtAssoc->iParent = iSurfel;
				pPtAssoc->i = j;
				pPtAssoc->n = 2;
				pPtAssoc++;
			}
		}
	}
	ptAssocs.n = pPtAssoc - ptAssocs.Element;

	// Determine the consensus set.
	// If bOptimize is true, then compute the optimal RRS.

	Array<RECOG::DDD::Rect3D> rects = pRectStruct->rects;
	RECOG::DDD::Rect3D *pRect = rects.Element;
	int *iSurfelRect = new int[planarSurfaces.NodeArray.n];
	memset(iSurfelRect, 0xff, planarSurfaces.NodeArray.n * sizeof(int));
	float e[3], de[3], ec, absde, score_;
	float score;
	float a_[9], da[3], ac[3];
	float *a;
	float fn;
	float PQR_[9];
	float *PQR;
	cv::Mat cvQ;
	cvQ.create(3, 3, CV_64FC1);
	double *Q = (double *)(cvQ.data);
	cv::Mat cvr;
	cvr.create(3, 1, CV_64FC1);
	double *r = (double *)(cvr.data);
	cv::Mat cvx;
	cvx.create(3, 1, CV_64FC1);
	double *x = (double *)(cvx.data);
	double dMx3x3Tmp[9];
	float w;
	int nIterations = (bOptimize ? 3 : 1);
	float th;
	float u[3];
	float dR[9], newR[9];
	bool *bOutlier = new bool[ptAssocs.n];
	memset(bOutlier, 0, ptAssocs.n * sizeof(bool));
	int iRect;
	float uncertCoeff;
#ifdef RVLDDD_RECTSTRUCT_DEBUG
	float E, dE, EPrev, ELin2, dELin2;
	double dELin;
#endif
	for (int iIteration = 0; iIteration < nIterations; iIteration++)
	{
		score = 0.0f;
#ifdef RVLDDD_RECTSTRUCT_DEBUG
		if (iIteration > 0)
			EPrev = E;
		E = 0.0f;
#endif
		if (bOptimize)
		{
			memset(Q, 0, 9 * sizeof(double));
			memset(r, 0, 3 * sizeof(double));
		}
		for (i = 0; i < ptAssocs.n; i++)
		{
			if (bOutlier[i])
				continue;
			pPtAssoc = ptAssocs.Element + i;
			if (pPtAssoc->i == 0)
				ec = 0.0f;
			PQR = PQR_ + 3 * pPtAssoc->i;
			RVLMULMX3X3VECT(RSR, pPtAssoc->PQ, PQR);
			e[pPtAssoc->i] = RVLDOTPRODUCT3(pPtAssoc->NM, PQR);
			ec += e[pPtAssoc->i];
			if (pPtAssoc->i == pPtAssoc->n - 1)
			{
				uncertCoeff = (pPtAssoc->n == 3 ? 0.0f : edgeUncertCoeff);
				fTmp = 1.0f + uncertCoeff;
				fn = (float)(pPtAssoc->n);
				ec /= fn;
				score_ = 0.0f;
				for (j = 0; j < pPtAssoc->n; j++)
				{
					de[j] = e[j] - ec;
					absde = RVLABS(de[j]);
					if (absde > fTmp * beta && iIteration == 0)
						break;
					score_ += absde;
				}
				if (j >= pPtAssoc->n)
				{
					pSurfel = (pPtAssoc->iParent >= 0 ? pSurfels->NodeArray.Element : planarSurfaces.NodeArray.Element) + pPtAssoc->iSurfel;
					w = (float)(pSurfel->size) / fn;
					if (bOptimize)
					{
						PQR = PQR_;
						a = a_;
						RVLNULL3VECTOR(ac);
						for (j = 0; j < pPtAssoc->n; j++, PQR += 3, a += 3)
						{
							RVLCROSSPRODUCT3(pPtAssoc->NM, PQR, a);
							RVLSUM3VECTORS(ac, a, ac);
#ifdef RVLDDD_RECTSTRUCT_DEBUG
							E += (w * de[j] * de[j]);
#endif
						}
						RVLSCALE3VECTOR2(ac, fn, ac);
						a = a_;
						for (j = 0; j < pPtAssoc->n; j++, a += 3)
						{
							RVLDIF3VECTORS(a, ac, da);
							RVLVECTCOV3(da, dMx3x3Tmp);
							RVLSCALEMX3X3UT(dMx3x3Tmp, w, dMx3x3Tmp);
							RVLSUMMX3X3UT(Q, dMx3x3Tmp, Q);
							fTmp = w * de[j];
							RVLSCALE3VECTOR(da, fTmp, V3Tmp);
							RVLSUM3VECTORS(r, V3Tmp, r);
						}
					}
					score_ = fn - score_ / beta;
					score += (w * score_);
					if (iIteration == nIterations - 1)
					{
						// if (pPtAssoc->iSurfel == 2410)
						//	int debug = 0;
						pRect->iAxis = (RVLABS(pPtAssoc->NM[0]) > 0.5f ? 0 : (RVLABS(pPtAssoc->NM[1]) > 0.5f ? 1 : 2));
						pRect->direction = (pPtAssoc->NM[pRect->iAxis] > 0.0f ? 1.0 : -1.0);
						pRect->c[pRect->iAxis] = pRect->direction * ec;
						pRect->iSurfel = pPtAssoc->iSurfel;
						pRect->iParent = pPtAssoc->iParent;
						if (pPtAssoc->iParent < 0)
							iSurfelRect[pRect->iSurfel] = pRect - rects.Element;
						pRect++;
					}
				}
				else if (bOptimize)
					for (j = 0; j < pPtAssoc->n; j++)
						bOutlier[i - j] = true;
			}
		}
		if (iIteration == nIterations - 1)
		{
			rects.n = pRect - rects.Element;
			int *compressedRectID = new int[rects.n];
			int compressedRectID_ = 0;
			for (iRect = 0; iRect < rects.n; iRect++)
			{
				pRect = rects.Element + iRect;
				if (pRect->iParent >= 0)
				{
					if (iSurfelRect[pRect->iParent] < 0)
					{
						compressedRectID[iRect] = -1;
						continue;
					}
					pRect->iParent = iSurfelRect[pRect->iParent];
				}
				compressedRectID[iRect] = compressedRectID_;
				compressedRectID_++;
			}
			for (iRect = 0; iRect < rects.n; iRect++)
			{
				if (compressedRectID[iRect] < 0)
					continue;
				pRect = rects.Element + compressedRectID[iRect];
				*pRect = rects.Element[iRect];
				if (pRect->iParent >= 0)
					pRect->iParent = compressedRectID[pRect->iParent];
			}
			delete[] compressedRectID;
			pRectStruct->rects.n = rects.n = compressedRectID_;
		}
		if (bOptimize)
		{
			RVLCOMPLETESIMMX3(Q);
			cv::solve(cvQ, cvr, cvx);
#ifdef RVLDDD_RECTSTRUCT_DEBUG
			if (iIteration > 0)
			{
				dE = E - EPrev;
				dELin2 = ELin2 - EPrev;
			}
			cv::Mat cvdELin = -2.0 * cvr.t() * cvx + cvx.t() * cvQ * cvx;
			dELin = *(double *)(cvdELin.data);
			ELin2 = 0.0f;
			for (i = 0; i < ptAssocs.n; i++)
			{
				if (bOutlier[i])
					continue;
				pPtAssoc = ptAssocs.Element + i;
				if (pPtAssoc->i == 0)
					ec = 0.0f;
				PQR = PQR_ + 3 * pPtAssoc->i;
				RVLMULMX3X3VECT(RSR, pPtAssoc->PQ, PQR);
				e[pPtAssoc->i] = RVLDOTPRODUCT3(pPtAssoc->NM, PQR);
				ec += e[pPtAssoc->i];
				if (pPtAssoc->i == pPtAssoc->n - 1)
				{
					fn = (float)(pPtAssoc->n);
					ec /= fn;
					for (j = 0; j < pPtAssoc->n; j++)
						de[j] = e[j] - ec;
					w = (float)(pSurfels->NodeArray.Element[pPtAssoc->iSurfel].size) / fn;
					PQR = PQR_;
					a = a_;
					RVLNULL3VECTOR(ac);
					for (j = 0; j < pPtAssoc->n; j++, PQR += 3, a += 3)
					{
						RVLCROSSPRODUCT3(pPtAssoc->NM, PQR, a);
						RVLSUM3VECTORS(ac, a, ac);
					}
					RVLSCALE3VECTOR2(ac, fn, ac);
					a = a_;
					for (j = 0; j < pPtAssoc->n; j++, a += 3)
					{
						RVLDIF3VECTORS(a, ac, da);
						fTmp = de[j] - RVLDOTPRODUCT3(da, x);
						ELin2 += (w * fTmp * fTmp);
						// RVLVECTCOV3(da, dMx3x3Tmp);
						// RVLSCALEMX3X3UT(dMx3x3Tmp, w, dMx3x3Tmp);
						// RVLSUMMX3X3UT(Q, dMx3x3Tmp, Q);
						// fTmp = w * de[j];
						// RVLSCALE3VECTOR(da, fTmp, dV3Tmp);
						// RVLSUM3VECTORS(r, dV3Tmp, r);
					}
				}
			}
#endif
			th = sqrt(RVLDOTPRODUCT3(x, x));
			RVLSCALE3VECTOR2(x, th, u);
			AngleAxisToRot<float>(u, th, dR);
			RVLMXMUL3X3(dR, RSR, newR);
			RVLMXMUL3X3T1(newR, newR, dMx3x3Tmp);
			GetAngleAxis(newR, u, th);
			AngleAxisToRot<float>(u, th, RSR);
		}
	}
	if (bOptimize)
	{
		RVLCOPYMX3X3T(RSR, RRS);
		if (pMesh)
		{
			QLIST::Index2 *pPtIdx;
			float *PQS;
			Rect<float> rect_;
			float PQR2D[2];
			bool bInitRect;
			int iAxis1, iAxis2;
			Surfel *pParent;
			float edgeParentDist;
			bool bValid;
			// Array<Point> ptArray;
			// ptArray.Element = new Point[pMesh->NodeArray.n];
			// ptArray.n = 0;
			for (iRect = 0; iRect < rects.n; iRect++)
			{
				// if (iRect == 81)
				//	int debug = 0;
				pRect = rects.Element + iRect;
				if (pRect->iParent >= 0)
				{
					pSurfel = pSurfels->NodeArray.Element + pRect->iSurfel;
					pParent = planarSurfaces.NodeArray.Element + rects.Element[pRect->iParent].iSurfel;
				}
				else
					pSurfel = planarSurfaces.NodeArray.Element + pRect->iSurfel;
				iAxis1 = (pRect->iAxis + 1) % 3;
				iAxis2 = (pRect->iAxis + 2) % 3;
				pPtIdx = pSurfel->PtList.pFirst;
				// int size = 0;
				bInitRect = true;
				while (pPtIdx)
				{
					PQS = pMesh->NodeArray.Element[pPtIdx->Idx].P;
					// if (iRect == 1)
					//{
					//	RVLCOPY3VECTOR(PQS, ptArray.Element[ptArray.n].P);
					//	ptArray.n++;
					// }
					if (pRect->iParent >= 0)
					{
						edgeParentDist = RVLDOTPRODUCT3(pParent->N, PQS) - pParent->d;
						bValid = (RVLABS(edgeParentDist) <= maxEdgeParentDist);
					}
					else
						bValid = true;
					if (bValid)
					{
						// size++;
						RVLMULMX3X3VECT(RSR, PQS, PQR_);
						PQR2D[0] = PQR_[iAxis1];
						PQR2D[1] = PQR_[iAxis2];
						if (bInitRect)
						{
							InitRect<float>(&rect_, PQR2D);
							bInitRect = false;
						}
						else
							UpdateRect<float>(&rect_, PQR2D);
					}
					pPtIdx = pPtIdx->pNext;
				}
				// if (size < minEdgeSize)
				//	int debug = 0;
				pRect->s[0] = rect_.maxx - rect_.minx;
				pRect->s[1] = rect_.maxy - rect_.miny;
				pRect->c[iAxis1] = 0.5f * (rect_.maxx + rect_.minx);
				pRect->c[iAxis2] = 0.5f * (rect_.maxy + rect_.miny);
			}
			// pVisualizationData->pVisualizer->SetMesh(pMesh);
			// uchar green[] = {0, 255, 0};
			// pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(ptArray, green, 6);
			// pVisualizationData->pVisualizer->Run();
			// delete[] ptArray.Element;
		}
	}

	// Free memory.

	delete[] ptAssocs.Element;
	delete[] bOutlier;
	delete[] iSurfelRect;

	//

	return score;
}

bool DDDetector::RectangularStructures(
	Mesh *pMesh,
	RECOG::DDD::RectStruct *pRectStruct)
{
	// Parameters.

	int maxnClusters = 1000;
	float csOrthogonalSurfelAngleTol = 0.1f;
	float beta = 0.030f;
	int minEdgeSize = 20;
	float maxCoPlanarSurfelNormalAngle = 30.0f; // deg
	bool bRectEdgeRects = false;
	float rectEdgeRectWidth = 2.0f / pSurfelDetector->kPlane; // m
	float minRectEdgeRectLen = 0.5f;						  // m

	// Constants.

	float halfRectEdgeRectWidth = 0.5f * rectEdgeRectWidth;

	// Detect surfels.

	pMem->Clear();
	pSurfels->Init(pMesh);
	pSurfelDetector->Init(pMesh, pSurfels, pMem);
	printf("Segmentation to surfels...");
	pSurfelDetector->Segment(pMesh, pSurfels);
	printf("completed.\n");
	int nSurfels = pSurfels->NodeArray.n;
	printf("No. of surfels = %d\n", nSurfels);
	pSurfels->EdgePointNormals(pMesh);

	// Display surfels.

	uchar green[] = {0, 255, 0};

	// if (pVisualizationData->bVisualizeSurfels)
	//{
	//	pSurfels->NodeColors(green);
	//	pSurfels->InitDisplay(pVisualizationData->pVisualizer, pMesh, pSurfelDetector);
	//	pSurfels->Display(pVisualizationData->pVisualizer, pMesh);
	//	pSurfels->DisplayEdgeFeatures();
	//	pVisualizationData->pVisualizer->Run();
	// }

	// Merge approximatelly coplanar adjacent surfels into planar surfaces.

	pSurfelDetector->MergeSurfels(pMesh, pSurfels, minSurfelSize, minEdgeSize, maxCoPlanarSurfelNormalAngle, maxCoPlanarSurfelRefPtDist, &planarSurfaces);

	// Only for testing purpose! Remove later!

	// AccuratePlaneFitting(pMesh);

	// Only for measuring the camera error for RO-MAN23! Remove later!
	Surfel *pSurfel__ = planarSurfaces.NodeArray.Element;
	float e;
	float std = 0.0f;
	int nPts = 0;
	Point *pPt;
	QLIST::Index2 *pPtIdx = pSurfel__->PtList.pFirst;
	while (pPtIdx)
	{
		pPt = pMesh->NodeArray.Element + pPtIdx->Idx;
		e = RVLDOTPRODUCT3(pSurfel__->N, pPt->P) - pSurfel__->d;
		std += (e * e);
		nPts++;
		pPtIdx = pPtIdx->pNext;
	}
	std = sqrt(std / (float)nPts);

	// Assign edges to surfels.

	SurfelEdges(pMesh, pSurfels->surfelRefPtMem, minSurfelSize, minEdgeSize);

	// Array<Point> refPtsDisp;
	// refPtsDisp.Element = new Point[2];
	// refPtsDisp.n = 2;
	// float* PDisp = pSurfels->NodeArray.Element[200].representativePts;
	// RVLCOPY3VECTOR(PDisp, refPtsDisp.Element[0].P);
	// PDisp += 3;
	// RVLCOPY3VECTOR(PDisp, refPtsDisp.Element[1].P);
	// pVisualizationData->pVisualizer->SetMesh(pMesh);
	// pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(refPtsDisp, green, 6);
	// pVisualizationData->pVisualizer->Run();
	// delete[] refPtsDisp.Element;

	// Assign edges to planar surfaces.

	PlanarSurfaceEdges(pMesh);

	// Visualize planar surfaces.

	if (pVisualizationData->bVisualizeSurfels)
	{
		planarSurfaces.NodeColors(green);
		planarSurfaces.InitDisplay(pVisualizationData->pVisualizer, pMesh, pSurfelDetector);
		planarSurfaces.Display(pVisualizationData->pVisualizer, pMesh);
		pVisualizationData->pVisualizer->Run();
	}

	// Sort planar surfaces according to their size.

	Array<SortIndex<int>> sortedPlanarSurfaces;
	sortedPlanarSurfaces.Element = new SortIndex<int>[planarSurfaces.NodeArray.n];
	SortIndex<int> *pSurfelIdx = sortedPlanarSurfaces.Element;
	int iSurfel;
	Surfel *pSurfel;
	bool *bRelevant = new bool[planarSurfaces.NodeArray.n];
	for (iSurfel = 0; iSurfel < planarSurfaces.NodeArray.n; iSurfel++)
	{
		pSurfel = planarSurfaces.NodeArray.Element + iSurfel;
		if (pSurfel->bEdge)
			continue;
		if (pSurfel->size >= minRefSurfelSize)
		{
			bRelevant[iSurfel] = true;
			pSurfelIdx->idx = iSurfel;
			pSurfelIdx->cost = pSurfel->size;
			pSurfelIdx++;
		}
		else
			bRelevant[iSurfel] = false;
	}
	sortedPlanarSurfaces.n = pSurfelIdx - sortedPlanarSurfaces.Element;
	BubbleSort<SortIndex<int>>(sortedPlanarSurfaces, true);

	// Search for the largest cluster.

	struct Cluster
	{
		int iSurfel1;
		int iSurfel2;
	};
	Array<Cluster> clusters;
	clusters.Element = new Cluster[maxnClusters];
	clusters.n = 0;
	int i, j;
	int iSurfel_;
	Surfel *pSurfel_;
	float bestRRS[9];
	float score;
	float bestScore = 0.0f;
	RECOG::DDD::SurfelEdge *pSurfelEdge;
	int maxnRects = planarSurfaces.NodeArray.n + surfelEdges.n;
	if (bRectEdgeRects)
		maxnRects += 4 * planarSurfaces.NodeArray.n;
	pRectStruct->rects.Element = new RECOG::DDD::Rect3D[maxnRects];
	bool bRectStructDetected;
	for (i = 0; i < sortedPlanarSurfaces.n; i++)
	{
		iSurfel = sortedPlanarSurfaces.Element[i].idx;
		pSurfel = planarSurfaces.NodeArray.Element + iSurfel;
		for (j = 0; j < i; j++)
		{
			iSurfel_ = sortedPlanarSurfaces.Element[j].idx;
			// if (iSurfel == 3 && iSurfel_ == 0)
			//	int debug = 0;
			pSurfel_ = planarSurfaces.NodeArray.Element + iSurfel_;
			if (RVLDOTPRODUCT3(pSurfel->N, pSurfel_->N) > csOrthogonalSurfelAngleTol)
				continue;
			score = RectangularStructure(iSurfel, iSurfel_, false, beta, pRectStruct);
			if (score > bestScore)
			{
				bestScore = score;
				RVLCOPYMX3X3(pRectStruct->RRS, bestRRS);
			}
		}
		for (j = 0; j < planarSurfeceEdges[iSurfel].n; j++)
		{
			pSurfelEdge = planarSurfeceEdges[iSurfel].Element[j];
			score = RectangularStructure(iSurfel, pSurfelEdge->iEdgeSurfel, true, beta, pRectStruct);
			if (score > bestScore)
			{
				bestScore = score;
				RVLCOPYMX3X3(pRectStruct->RRS, bestRRS);
			}
		}
	}
	if (bRectStructDetected = (bestScore > 0.0f))
	{
		// Optimization.

		RVLCOPYMX3X3(bestRRS, pRectStruct->RRS);
		RectangularStructure(-1, -1, false, beta, pRectStruct, false, true, pMesh);

		// Rectangle edge rectangles.

		if (bRectEdgeRects)
		{
			int iRect;
			RECOG::DDD::Rect3D *pRect;
			RECOG::DDD::Rect3D *pRectEdgeRect = pRectStruct->rects.Element + pRectStruct->rects.n;
			int iAxis0, iAxis1, iAxis2;
			float direction;
			float rectEdgeRectLen;
			for (iRect = 0; iRect < pRectStruct->rects.n; iRect++)
			{
				pRect = pRectStruct->rects.Element + iRect;
				if (pRect->iParent >= 0)
					continue;
				for (i = 0; i < 2; i++)
				{
					rectEdgeRectLen = pRect->s[1 - i];
					if (rectEdgeRectLen < minRectEdgeRectLen)
						continue;
					iAxis0 = (pRect->iAxis + i + 1) % 3;
					iAxis1 = (iAxis0 + 1) % 3;
					iAxis2 = (iAxis0 + 2) % 3;
					direction = -1.0f;
					for (j = 0; j < 2; j++, direction = -direction)
					{
						pRectEdgeRect->iAxis = iAxis0;
						pRectEdgeRect->c[iAxis0] = pRect->c[iAxis0] + 0.5f * direction * pRect->s[i];
						pRectEdgeRect->c[iAxis1] = pRect->c[iAxis1];
						pRectEdgeRect->c[iAxis2] = pRect->c[iAxis2];
						pRectEdgeRect->s[1 - i] = halfRectEdgeRectWidth;
						pRectEdgeRect->s[i] = rectEdgeRectLen;
						pRectEdgeRect->direction = direction;
						pRectEdgeRect->iParent = iRect;
						pRectEdgeRect->iSurfel = -1;
						pRectEdgeRect++;
					}
				}
			}
			pRectStruct->rects.n = pRectEdgeRect - pRectStruct->rects.Element;
		}

		// Visualization.

		if (pVisualizationData->bVisualizeRectangularStructure)
		{
			// RectangularStructure(-1, -1, false, surfelRefPtMem, beta, pRectStruct, false);
			pVisualizationData->pVisualizer->SetMesh(pMesh);
			VisualizeRectangularStructure(pMesh, pRectStruct);
			pVisualizationData->pVisualizer->Run();
			pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
		}
		bRectStructDetected = true;
	}
	else
		printf("No rectangular structure detected!\n");

	delete[] sortedPlanarSurfaces.Element;
	delete[] bRelevant;
	delete[] clusters.Element;

	return bRectStructDetected;
}

//#define RVLDDD_MATCHRECTSTRUCT_TRANSLATION_FROM_SURFEL_ASSOC(pSurfAssoc)\
//{\
//	pMRect = MRects.Element + pSurfAssoc->iMRect;\
//	pQRect = pQRectStruct->rects.Element + pSurfAssoc->iQRect;\
//	poseMQ.t[pMRect->iAxis] = pMRect->c[pMRect->iAxis] - pQRect->c[pMRect->iAxis];\
//	bAxis[pMRect->iAxis] = true;\
//	if (pMRect->iParent >= 0)\
//	{\
//		pMRect = MRects.Element + pMRect->iParent;\
//		pQRect = pQRectStruct->rects.Element + pQRect->iParent;\
//		poseMQ.t[pMRect->iAxis] = pMRect->c[pMRect->iAxis] - pQRect->c[pMRect->iAxis];\
//		bAxis[pMRect->iAxis] = true;\
//	}\
//}

#define RVLDDD_MATCHRECTSTRUCT_METHOD1

bool DDDetector::MatchRectangularStructures(
	RECOG::DDD::RectStruct *pMRectStruct,
	RECOG::DDD::RectStruct *pQRectStruct,
	Pose3D &poseMQ,
	Pose3D &poseMC,
	Pose3D *pPoseCmCqInit,
	float *verticalMAxis)
{
	// Parameters.

	float maxe = 0.02f;
	int maxnHyps = 1000;
	// float maxRectStructAlignmentCorrection = 0.2f;
	float maxRectStructAlignmentCorrectionXY = 0.2f;
	float maxRectStructAlignmentCorrectionZ = 0.2f;

	/// Determine relative orientation.

	int iMAxis, iQAxis, iDir;
	float dir;
	float cs;
	float maxcs;
	int iMAxisMap[3];	  // iMAxisMap[i] is the index of the query axis corresponding to the i-th model axis.
	int iQAxisMap[3];	  // iQAxisMap[i] is the index of the model axis corresponding to the i-th query axis.
	float dirMAxisMap[3]; // dirMAxisMap[i] is the direction (same: 1.0; opposite: -1.0) of the query axis corresponding to the i-th model axis.
	struct RectStructRelativeOrientation
	{
		int iMAxisMap[3];
		float dirMAxisMap[3];
	};
	Array<RectStructRelativeOrientation> MQOrients;
	RectStructRelativeOrientation *pMQOrient;

	// Method 1: Closest axes.

	// for (iMAxis = 0; iMAxis < 3; iMAxis++)
	//{
	//	maxcs = 0.0f;
	//	for (iQAxis = 0; iQAxis < 3; iQAxis++)
	//	{
	//		cs = RVLMULCOLCOL3(pMRectStruct->RRS, pQRectStruct->RRS, iMAxis, iQAxis);
	//		if (cs < 0)
	//		{
	//			cs = -cs;
	//			dir = -1.0f;
	//		}
	//		else
	//			dir = 1.0f;
	//		if (cs > maxcs)
	//		{
	//			maxcs = cs;
	//			iMAxisMap[iMAxis] = iQAxis;
	//			dirMAxisMap[iMAxis] = dir;
	//		}
	//	}
	// }
	// if ((iMAxisMap[0] - iMAxisMap[1]) * (iMAxisMap[1] - iMAxisMap[2]) * (iMAxisMap[2] - iMAxisMap[0]) == 0)
	//{
	//	printf("ERROR: Orientations are too different!\n");
	//	return;
	// }
	// MQOrients.n = 1;
	// MQOrients.Element = new RectStructRelativeOrientation[1];
	// pMQOrient = MQOrients.Element;
	// RVLCOPY3VECTOR(iMAxisMap, pMQOrient->iMAxisMap);
	// RVLCOPY3VECTOR(dirMAxisMap, pMQOrient->dirMAxisMap);
	//  Only for debugging purpose!!!
	// RVLSET3VECTOR(iMAxisMap, 2, 1, 0);
	// RVLSET3VECTOR(dirMAxisMap, 1.0f, 1.0f, -1.0f);
	//

	// Method 2: Four possible orientations about the vertical axis.

	int iVerticalMAxis;
	float maxcsVerticalMAxis = 0.0f;
	float abscs;
	for (iMAxis = 0; iMAxis < 3; iMAxis++)
	{
		cs = (verticalMAxis ? RVLMULVECTORCOL3(verticalMAxis, pMRectStruct->RRS, iMAxis) : pMRectStruct->RRS[3 * iMAxis + 1]);
		abscs = RVLABS(cs);
		if (abscs > maxcsVerticalMAxis)
		{
			maxcsVerticalMAxis = abscs;
			iVerticalMAxis = iMAxis;
		}
	}
	maxcs = 0.0f;
	for (iQAxis = 0; iQAxis < 3; iQAxis++)
	{
		cs = RVLMULCOLCOL3(pMRectStruct->RRS, pQRectStruct->RRS, iVerticalMAxis, iQAxis);
		if (cs < 0)
		{
			cs = -cs;
			dir = -1.0f;
		}
		else
			dir = 1.0f;
		if (cs > maxcs)
		{
			maxcs = cs;
			iMAxisMap[iVerticalMAxis] = iQAxis;
			dirMAxisMap[iVerticalMAxis] = dir;
		}
	}
	int iVerticalQAxis = iMAxisMap[iVerticalMAxis];
	int iMAxis1 = (iVerticalMAxis + 1) % 3;
	int iQAxis1 = (iVerticalQAxis + 1) % 3;
	float dirQAxis1 = 1.0f;
	int iMAxis2 = (iVerticalMAxis + 2) % 3;
	int iQAxis2 = (iVerticalQAxis + 2) % 3;
	float dirQAxis2 = dirMAxisMap[iVerticalMAxis];
	MQOrients.n = 4;
	MQOrients.Element = new RectStructRelativeOrientation[4];
	pMQOrient = MQOrients.Element;
	int iMQOrient;
	int iTmp;
	float fTmp;
	for (iMQOrient = 0; iMQOrient < 4; iMQOrient++, pMQOrient++)
	{
		iMAxisMap[iMAxis1] = iQAxis1;
		dirMAxisMap[iMAxis1] = dirQAxis1;
		iMAxisMap[iMAxis2] = iQAxis2;
		dirMAxisMap[iMAxis2] = dirQAxis2;
		RVLCOPY3VECTOR(iMAxisMap, pMQOrient->iMAxisMap);
		RVLCOPY3VECTOR(dirMAxisMap, pMQOrient->dirMAxisMap);
		iTmp = iQAxis1;
		fTmp = dirQAxis1;
		iQAxis1 = iQAxis2;
		dirQAxis1 = dirQAxis2;
		iQAxis2 = iTmp;
		dirQAxis2 = -fTmp;
	}

	///

	// Initial position of M w.r.t. Q

	Pose3D poseMQInit;
	RVLUNITMX3(poseMQInit.R);
	RVLNULL3VECTOR(poseMQInit.t);
	if (pPoseCmCqInit)
	{
		RVLMULMX3X3TVECT(pQRectStruct->RRS, pPoseCmCqInit->t, poseMQInit.t);
	}
	//

	int i;
	float bestScore = 0.0f;
	int iBestMQOrient = -1;
	Array<RECOG::DDD::Rect3D> MRects;
	MRects.n = pMRectStruct->rects.n;
	MRects.Element = new RECOG::DDD::Rect3D[pMRectStruct->rects.n];
	Array<RECOG::DDD::Rect3D> QRects = pQRectStruct->rects;
	bool bSuccess;
	for (iMQOrient = 0; iMQOrient < MQOrients.n; iMQOrient++)
	{
		// Take orientation iMQOrient from MQOrients.

		pMQOrient = MQOrients.Element + iMQOrient;
		RVLCOPY3VECTOR(pMQOrient->iMAxisMap, iMAxisMap);
		RVLCOPY3VECTOR(pMQOrient->dirMAxisMap, dirMAxisMap);

		// Fill iQAxisMap.

		for (iMAxis = 0; iMAxis < 3; iMAxis++)
		{
			iQAxis = iMAxisMap[iMAxis];
			iQAxisMap[iQAxis] = iMAxis;
		}

		// Rotate the model.

		RotateRectStruct(pMRectStruct, iMAxisMap, dirMAxisMap, iQAxisMap, MRects);

		// Translate the model.

		RECOG::DDD::Rect3D *pMRect;
		if (pPoseCmCqInit)
		{
			pMRect = MRects.Element;
			for (int iMRect = 0; iMRect < MRects.n; iMRect++, pMRect++)
			{
				RVLSUM3VECTORS(pMRect->c, poseMQInit.t, pMRect->c);
			}
		}

		/// Model-Query structure registration.

		// Sort surfel rectangles according to their size.

		Array<SortIndex<float>> sortedSurfels[2];
		SortIndex<float> *pSurfelIdx;
		RECOG::DDD::Rect3D *pRect, *pParent;
		int iRect;
		Array<RECOG::DDD::Rect3D> rects[2];
		rects[0] = MRects;
		rects[1] = pQRectStruct->rects;
		int iStruct;
		for (iStruct = 0; iStruct < 2; iStruct++)
		{
			sortedSurfels[iStruct].Element = new SortIndex<float>[rects[iStruct].n];
			pSurfelIdx = sortedSurfels[iStruct].Element;
			for (iRect = 0; iRect < rects[iStruct].n; iRect++)
			{
				pRect = rects[iStruct].Element + iRect;
				pSurfelIdx->idx = iRect;
				pSurfelIdx->cost = pRect->s[0] * pRect->s[1];
				if (pRect->iParent >= 0)
				{
					pParent = rects[iStruct].Element + pRect->iParent;
					pSurfelIdx->cost += pParent->s[0] * pParent->s[1];
				}
				pSurfelIdx++;
			}
			sortedSurfels[iStruct].n = pSurfelIdx - sortedSurfels[iStruct].Element;
			BubbleSort<SortIndex<float>>(sortedSurfels[iStruct], true);
		}

		// Surfel association.

		Array<RECOG::DDD::SurfAssoc> surfAssocs;
		surfAssocs.Element = new RECOG::DDD::SurfAssoc[MRects.n * QRects.n];
		RECOG::DDD::SurfAssoc *pSurfAssoc = surfAssocs.Element;
		int iSmallerRect[2];
		int iLargerRect;
		iSmallerRect[0] = iSmallerRect[1] = 0;
		SortIndex<float> *pSurfelIdx_, *pSmallerSurfelIdx, *pLargerSurfelIdx;
		int iStruct_;
		RECOG::DDD::Rect3D *pSmallerRect, *pLargerRect, *pSmallerRectParent, *pLargerRectParent;
		float smallerRectSize, largerRectSize, smallerRectParentSize, largerRectParentSize;
		bool bEndReached[2];
		bEndReached[0] = bEndReached[1] = false;
		float eMQ;
		float maxeMQ;
		while (true)
		{
			pSurfelIdx = sortedSurfels[0].Element + iSmallerRect[0];
			pSurfelIdx_ = sortedSurfels[1].Element + iSmallerRect[1];
			iStruct = (bEndReached[0] ? 1 : (bEndReached[1] ? 0 : (pSurfelIdx->cost >= pSurfelIdx_->cost ? 0 : 1)));
			iStruct_ = 1 - iStruct;
			pSmallerSurfelIdx = sortedSurfels[iStruct].Element + iSmallerRect[iStruct];
			pSmallerRect = rects[iStruct].Element + pSmallerSurfelIdx->idx;
			for (iLargerRect = 0; iLargerRect < sortedSurfels[iStruct_].n; iLargerRect++)
			{
				pLargerSurfelIdx = sortedSurfels[iStruct_].Element + iLargerRect;
				if (pLargerSurfelIdx->cost < pSmallerSurfelIdx->cost)
					break;
				pLargerRect = rects[iStruct_].Element + pLargerSurfelIdx->idx;
				if (pSmallerRect->iAxis != pLargerRect->iAxis || pSmallerRect->direction != pLargerRect->direction)
					continue;
				if (pSmallerRect->iParent >= 0 && pLargerRect->iParent >= 0)
				{
					pSmallerRectParent = rects[iStruct].Element + pSmallerRect->iParent;
					pLargerRectParent = rects[iStruct_].Element + pLargerRect->iParent;
					if (pSmallerRectParent->iAxis != pLargerRectParent->iAxis || pSmallerRectParent->direction != pLargerRectParent->direction)
						continue;
				}
				else if (pSmallerRect->iParent >= 0 || pLargerRect->iParent >= 0)
					continue;
				if (pPoseCmCqInit)
				{
					maxeMQ = (pSmallerRect->iAxis == iVerticalQAxis ? maxRectStructAlignmentCorrectionZ : maxRectStructAlignmentCorrectionXY);
					eMQ = pLargerRect->c[pSmallerRect->iAxis] - pSmallerRect->c[pSmallerRect->iAxis];
					if (RVLABS(eMQ) > maxeMQ)
						continue;
					if (pSmallerRect->iParent >= 0)
					{
						maxeMQ = (pSmallerRectParent->iAxis == iVerticalQAxis ? maxRectStructAlignmentCorrectionZ : maxRectStructAlignmentCorrectionXY);
						eMQ = pLargerRectParent->c[pSmallerRectParent->iAxis] - pSmallerRectParent->c[pSmallerRectParent->iAxis];
						if (RVLABS(eMQ) > maxeMQ)
							continue;
					}
				}
				if (iStruct == 0)
				{
					pSurfAssoc->iMRect = pSmallerSurfelIdx->idx;
					pSurfAssoc->iQRect = pLargerSurfelIdx->idx;
				}
				else
				{
					pSurfAssoc->iMRect = pLargerSurfelIdx->idx;
					pSurfAssoc->iQRect = pSmallerSurfelIdx->idx;
				}
				if (pSmallerRect->iParent >= 0)
				{
					smallerRectSize = pSmallerRect->s[0] * pSmallerRect->s[1];
					largerRectSize = pLargerRect->s[0] * pLargerRect->s[1];
					pSurfAssoc->cost = RVLMIN(smallerRectSize, largerRectSize);
					smallerRectParentSize = pSmallerRectParent->s[0] * pSmallerRectParent->s[1];
					largerRectParentSize = pLargerRectParent->s[0] * pLargerRectParent->s[1];
					pSurfAssoc->cost += RVLMIN(smallerRectParentSize, largerRectParentSize);
				}
				else
					pSurfAssoc->cost = pSmallerSurfelIdx->cost;
				pSurfAssoc++;
			}
			if (iSmallerRect[iStruct] < sortedSurfels[iStruct].n - 1)
				iSmallerRect[iStruct]++;
			else
			{
				bEndReached[iStruct] = true;
				if (iSmallerRect[iStruct_] >= sortedSurfels[iStruct_].n - 1)
					break;
			}
		}
		surfAssocs.n = pSurfAssoc - surfAssocs.Element;
		BubbleSort<RECOG::DDD::SurfAssoc>(surfAssocs, true);
		delete[] sortedSurfels[0].Element;
		delete[] sortedSurfels[1].Element;

		// FILE* fpDebug = fopen((std::string(resultsFolder) + "\\surfassoc.txt").data(), "w");
		// for (i = 0; i < surfAssocs.n; i++)
		//{
		//	pSurfAssoc = surfAssocs.Element + i;
		//	fprintf(fpDebug, "%d\t%d\n", MRects.Element[pSurfAssoc->iMRect].iSurfel, QRects.Element[pSurfAssoc->iQRect].iSurfel);
		// }
		// fclose(fpDebug);

		// Find the optimal match.

		int j, k;
		bool bAxis[3];
		RECOG::DDD::Rect3D *pQRect, *pMParentRect, *pQParentRect;
		int nAxesDefined = 0;
		float e;
		float tMQ[3];
		float tMQ_;
		float tMQ__[2][3];
		int nHyps_, iHyp;
		int iiAxis[2], ijAxis[2];
		float score;
		int nHyps = 0;
		for (i = 0; i < surfAssocs.n; i++)
		{
			pSurfAssoc = surfAssocs.Element + i;
			pMRect = MRects.Element + pSurfAssoc->iMRect;
			pQRect = QRects.Element + pSurfAssoc->iQRect;
			tMQ[pMRect->iAxis] = pQRect->c[pMRect->iAxis] - pMRect->c[pMRect->iAxis];
			bAxis[pMRect->iAxis] = true;
			nAxesDefined = 1;
			iiAxis[0] = iiAxis[1] = pMRect->iAxis;
			if (pMRect->iParent >= 0)
			{
				pMRect = MRects.Element + pMRect->iParent;
				pQRect = QRects.Element + pQRect->iParent;
				tMQ[pMRect->iAxis] = pQRect->c[pMRect->iAxis] - pMRect->c[pMRect->iAxis];
				bAxis[pMRect->iAxis] = true;
				nAxesDefined++;
				iiAxis[1] = pMRect->iAxis;
			}
			for (j = 0; j < i; j++)
			{
				// if (i == 38 && j == 0)
				//	int debug = 0;
				pSurfAssoc = surfAssocs.Element + j;
				pMRect = MRects.Element + pSurfAssoc->iMRect;
				if (bAxis[pMRect->iAxis])
					continue;
				pQRect = QRects.Element + pSurfAssoc->iQRect;
				nHyps_ = 1;
#ifdef RVLDDD_MATCHRECTSTRUCT_METHOD1
				if (pMRect->iParent >= 0)
				{
					pMParentRect = MRects.Element + pMRect->iParent;
					if (bAxis[pMParentRect->iAxis])
						continue;
					pQParentRect = QRects.Element + pQRect->iParent;
					if (nAxesDefined > 1)
					{
						tMQ_ = pQParentRect->c[pMParentRect->iAxis] - pMParentRect->c[pMParentRect->iAxis];
						e = tMQ_ - tMQ[pMParentRect->iAxis];
						if (RVLABS(e) > maxe)
							continue;
						nHyps_ = 2;
					}
				}
				tMQ[pMRect->iAxis] = pQRect->c[pMRect->iAxis] - pMRect->c[pMRect->iAxis];
				RVLCOPY3VECTOR(tMQ, tMQ__[0]);
				bAxis[pMRect->iAxis] = true;
				nAxesDefined++;
				ijAxis[0] = ijAxis[1] = pMRect->iAxis;
				if (nHyps_ == 2)
					tMQ__[1][pMParentRect->iAxis] = tMQ_;
				else if (pMRect->iParent >= 0)
				{
					tMQ__[0][pMParentRect->iAxis] = pQRect->c[pMParentRect->iAxis] - pMRect->c[pMParentRect->iAxis];
					bAxis[pMParentRect->iAxis] = true;
					nAxesDefined++;
					ijAxis[1] = pMParentRect->iAxis;
				}
#else
				ijAxis[0] = ijAxis[1] = pMRect->iAxis;
				if (pMRect->iParent >= 0)
				{
					pMParentRect = MRects.Element + pMRect->iParent;
					pQParentRect = QRects.Element + pQRect->iParent;
					tMQ_ = pQParentRect->c[pMParentRect->iAxis] - pMParentRect->c[pMParentRect->iAxis];
					if (bAxis[pMParentRect->iAxis])
					{
						e = tMQ_ - tMQ[pMParentRect->iAxis];
						if (RVLABS(e) > maxe)
							continue;
					}
					else
					{
						tMQ[pMParentRect->iAxis] = tMQ_;
						bAxis[pMParentRect->iAxis] = true;
						nAxesDefined++;
					}
					ijAxis[1] = pMParentRect->iAxis;
				}
				tMQ[pMRect->iAxis] = pQRect->c[pMRect->iAxis] - pMRect->c[pMRect->iAxis];
				bAxis[pMRect->iAxis] = true;
				nAxesDefined++;
				RVLCOPY3VECTOR(tMQ, tMQ__[0]);
#endif
				if (nAxesDefined < 3)
				{
					for (iHyp = 0; iHyp < nHyps_; iHyp++)
					{
						for (k = 0; k < j; k++)
						{
							pSurfAssoc = surfAssocs.Element + k;
							pMRect = MRects.Element + pSurfAssoc->iMRect;
							if (pMRect->iParent >= 0)
								continue;
							if (bAxis[pMRect->iAxis])
								continue;
							pQRect = QRects.Element + pSurfAssoc->iQRect;
							tMQ__[iHyp][pMRect->iAxis] = pQRect->c[pMRect->iAxis] - pMRect->c[pMRect->iAxis];
							score = MatchRectangularStructures(MRects, QRects, surfAssocs, tMQ__[iHyp], maxe);
							if (score > bestScore)
							{
								bestScore = score;
								RVLCOPY3VECTOR(tMQ__[iHyp], poseMQ.t);
								iBestMQOrient = iMQOrient;
							}
							nHyps++;
							if (nHyps >= maxnHyps)
								break;
						}
						if (k < j)
							break;
					}
					if (iHyp < nHyps_)
						break;
				}
				else
				{
					score = MatchRectangularStructures(MRects, QRects, surfAssocs, tMQ__[0], maxe);
					if (score > bestScore)
					{
						bestScore = score;
						RVLCOPY3VECTOR(tMQ__[0], poseMQ.t);
						iBestMQOrient = iMQOrient;
					}
					nHyps++;
					if (nHyps >= maxnHyps)
						break;
				}
				bAxis[ijAxis[0]] = bAxis[ijAxis[1]] = false;
				nAxesDefined -= (ijAxis[0] == ijAxis[1] ? 1 : 2);
			}
			if (j < i)
				break;
			bAxis[iiAxis[0]] = bAxis[iiAxis[1]] = false;
		}
		delete[] surfAssocs.Element;
	}
	if (iBestMQOrient >= 0)
	{
		pMQOrient = MQOrients.Element + iBestMQOrient;
		RVLNULLMX3X3(poseMQ.R);
		for (iMAxis = 0; iMAxis < 3; iMAxis++)
		{
			iQAxis = pMQOrient->iMAxisMap[iMAxis];
			iQAxisMap[iQAxis] = iMAxis;
			poseMQ.R[iMAxis + 3 * iQAxis] = pMQOrient->dirMAxisMap[iMAxis];
		}
		if (pPoseCmCqInit)
		{
			RVLSUM3VECTORS(poseMQ.t, poseMQInit.t, poseMQ.t);
			poseMQ.t[iVerticalQAxis] = poseMQInit.t[iVerticalQAxis];
		}
		Pose3D poseQC;
		RVLCOPYMX3X3(pQRectStruct->RRS, poseQC.R);
		RVLNULL3VECTOR(poseQC.t);
		RVLCOMPTRANSF3D(poseQC.R, poseQC.t, poseMQ.R, poseMQ.t, poseMC.R, poseMC.t);

		///

		// Visualization.

		if (pVisualizationData->bVisualizeRectStructMatching)
		{
			RECOG::DDD::RectStruct QRectStruct;
			RVLUNITMX3(QRectStruct.RRS);
			QRectStruct.rects = QRects;
			VisualizeRectangularStructure(NULL, &QRectStruct);
			uchar red[] = {255, 0, 0};
			RECOG::DDD::RectStruct MRectStruct;
			RVLUNITMX3(MRectStruct.RRS);
			RotateRectStruct(pMRectStruct, pMQOrient->iMAxisMap, pMQOrient->dirMAxisMap, iQAxisMap, MRects);
			MRectStruct.rects = MRects;
			VisualizeRectangularStructure(NULL, &MRectStruct, red, poseMQ.t);
			pVisualizationData->pVisualizer->Run();
			pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
		}

		bSuccess = true;
	}
	else
	{
		printf("Matching of rectangular structures failed!\n");
		bSuccess = false;
	}

	//

	delete[] MRects.Element;
	delete[] MQOrients.Element;
	return bSuccess;
}

float DDDetector::MatchRectangularStructures(
	Array<RECOG::DDD::Rect3D> MRects,
	Array<RECOG::DDD::Rect3D> QRects,
	Array<RECOG::DDD::SurfAssoc> surfAssocs,
	float *tMQ,
	float maxe)
{
	int iSurfAssoc;
	float score = 0.0f;
	RECOG::DDD::SurfAssoc *pSurfAssoc;
	RECOG::DDD::Rect3D *pMRect, *pQRect;
	float e;
	int i;
	float minIS, maxIS, minMEdge, maxMEdge, minQEdge, maxQEdge;
	int iAxis;
	float cM, IS, IS_;
	float ISPerAxis[3];
	RVLNULL3VECTOR(ISPerAxis);
	// float score_;
	for (iSurfAssoc = 0; iSurfAssoc < surfAssocs.n; iSurfAssoc++)
	{
		pSurfAssoc = surfAssocs.Element + iSurfAssoc;
		pMRect = MRects.Element + pSurfAssoc->iMRect;
		if (pMRect->iParent >= 0)
			continue;
		pQRect = QRects.Element + pSurfAssoc->iQRect;
		e = pMRect->c[pMRect->iAxis] + tMQ[pMRect->iAxis] - pQRect->c[pMRect->iAxis];
		e = RVLABS(e);
		if (e > maxe)
			continue;
		iAxis = pMRect->iAxis;
		IS = 1.0f;
		// IS = 0.0f;
		for (i = 0; i < 2; i++)
		{
			iAxis = (iAxis + 1) % 3;
			cM = pMRect->c[iAxis] + tMQ[iAxis];
			minMEdge = cM - 0.5f * pMRect->s[i];
			maxMEdge = cM + 0.5f * pMRect->s[i];
			minQEdge = pQRect->c[iAxis] - 0.5f * pQRect->s[i];
			maxQEdge = pQRect->c[iAxis] + 0.5f * pQRect->s[i];
			minIS = RVLMAX(minMEdge, minQEdge);
			maxIS = RVLMIN(maxMEdge, maxQEdge);
			IS_ = maxIS - minIS;
			if (IS_ < 0.0f)
			{
				IS = 0.0f;
				break;
			}
			else
				IS *= IS_;
			// IS += (IS_ * IS_);
		}
		// IS = sqrt(IS);
		ISPerAxis[pMRect->iAxis] += IS;
		// score_ = (1.0f - e / maxe) * IS;
		// score += score_;
	}
	for (iAxis = 0; iAxis < 3; iAxis++)
	{
		// if (ISPerAxis[iAxis] > 0)
		score += ISPerAxis[iAxis];
		// else
		//{
		//	score = 0.0;
		//	break;
		// }
	}
	return score;
}

bool DDDetector::RecognizeArticulatedObject(
	Mesh *pMesh,
	RECOG::DDD::ArticulatedObject AObj,
	Pose3D &poseOC,
	Pose3D *pPoseOCInit,
	float *verticalMAxis,
	cv::Mat *pRGBImg)
{
	if (pPoseOCInit)
		poseOC = *pPoseOCInit;

	RECOG::DDD::RectStruct QRS;

	// Detect rectangular structure.

	bool bRectStructDetected;
	if (bRectStructDetected = RectangularStructures(pMesh, &QRS))
	{
		// Compare the detected rectangular structure to the model.

		Pose3D poseMQ, poseMC;
		RECOG::DDD::RectStruct *pMRS = &(AObj.MRS);
		if (MatchRectangularStructures(pMRS, &QRS, poseMQ, poseMC, pPoseOCInit, verticalMAxis))
		{
			// Compute articulated object pose with respect to the camera.

			Pose3D poseMO;
			RVLCOPYMX3X3(pMRS->RRS, poseMO.R);
			RVLNULL3VECTOR(poseMO.t);
			Pose3D poseOM;
			RVLINVTRANSF3D(poseMO.R, poseMO.t, poseOM.R, poseOM.t);
			RVLCOMPTRANSF3D(poseMC.R, poseMC.t, poseOM.R, poseOM.t, poseOC.R, poseOC.t);
		}
		// RVLCOPY3VECTOR(poseOCInit.t, poseOC.t);

		// Visualize AO.

		if (pVisualizationData->bVisualizeAOInitState)
		{
			RECOG::DDD::AOHypothesisState *states;
			AOZeroState(AObj, states);
			if (GetRGBImageVisualization())
			{
				if (pRGBImg)
				{
					cv::Mat display;
					pRGBImg->copyTo(display);
					VisualizeArticulatedObject(AObj, poseOC, false, &display);
					cv::imshow("Articulated object", display);
					cv::waitKey();
				}
			}
			GetVisualizer()->SetMesh(pMesh);
			VisualizeArticulatedObject(AObj, poseOC, true);
			RunVisualizer();
			ClearVisualization();
			delete[] states;
		}
	}

	delete[] QRS.rects.Element;

	return bRectStructDetected;
}

bool DDDetector::OrientedBoundaryPoints(
	Mesh *pMesh,
	int iSurfel,
	Array<OrientedPoint> &boundary)
{
	Surfel *pSurfel = planarSurfaces.NodeArray.Element + iSurfel;

	// Parameters.

	int bndWinHalfSize = 4;

	// Identify the largest boundary.

	Array<MeshEdgePtr *> *pBoundary;
	int iBoundary;
	int maxBoundarySize = 0;
	int iMaxBoundary;
	for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
	{
		pBoundary = pSurfel->BoundaryArray.Element + iBoundary;
		if (pBoundary->n > maxBoundarySize)
		{
			maxBoundarySize = pBoundary->n;
			iMaxBoundary = iBoundary;
		}
	}
	if (maxBoundarySize < 2 * bndWinHalfSize + 1)
		return 0.0;

	// Boundary point normals.

	pBoundary = pSurfel->BoundaryArray.Element + iMaxBoundary;
	boundary.n = pBoundary->n;
	boundary.Element = new OrientedPoint[pBoundary->n];
	int iPt;
	Point *pPt, *pPt1, *pPt2;
	float V[3];
	int i, j;
	MeshEdgePtr *pEdgePtr;
	float fTmp;
	OrientedPoint *pPt_ = boundary.Element;
	for (i = 0; i < pBoundary->n; i++, pPt_++)
	{
		pEdgePtr = pBoundary->Element[i];
		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);
		pPt = pMesh->NodeArray.Element + iPt;
		RVLCOPY3VECTOR(pPt->P, pPt_->P);
		j = (i + bndWinHalfSize) % pBoundary->n;
		pEdgePtr = pBoundary->Element[j];
		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);
		pPt1 = pMesh->NodeArray.Element + iPt;
		j = (i + pBoundary->n - bndWinHalfSize) % pBoundary->n;
		pEdgePtr = pBoundary->Element[j];
		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);
		pPt2 = pMesh->NodeArray.Element + iPt;
		RVLDIF3VECTORS(pPt2->P, pPt1->P, V);
		RVLCROSSPRODUCT3(V, pSurfel->N, pPt_->N);
		RVLNORM3(pPt_->N, fTmp);
	}
}

float DDDetector::MatchToRectangle(
	Array<OrientedPoint> boundary,
	float *RRS,
	Rect<float> *pRect)
{
	// Parameters.

	float uncert = 0.01f;

	//

	float RSR[9];
	float *XRS = RSR;
	float *YRS = RSR + 3;
	float *ZRS = RSR + 6;
	RVLCOPYMX3X3T(RRS, RSR);

	std::vector<SortIndex<float>> ptBuff[4];
	int iPt;
	OrientedPoint *pPt;
	int iBuff;
	float nx, ny;
	float s;
	SortIndex<float> ptIntervalIdx;
	for (iPt = 0; iPt < boundary.n; iPt++)
	{
		pPt = boundary.Element + iPt;
		nx = RVLDOTPRODUCT3(XRS, pPt->N);
		ny = RVLDOTPRODUCT3(YRS, pPt->N);
		if (RVLABS(nx) >= RVLABS(ny))
		{
			iBuff = (nx >= 0 ? 0 : 1);
			s = pPt->P[0];
		}
		else
		{
			iBuff = (ny >= 0 ? 2 : 3);
			s = pPt->P[1];
		}
		ptIntervalIdx.cost = s - uncert;
		ptIntervalIdx.idx = 2 * iPt;
		ptBuff[iBuff].push_back(ptIntervalIdx);
		ptIntervalIdx.cost = s + uncert;
		ptIntervalIdx.idx = 2 * iPt + 1;
		ptBuff[iBuff].push_back(ptIntervalIdx);
	}
	for (iBuff = 0; iBuff < 4; iBuff++)
		std::sort(ptBuff[iBuff].begin(), ptBuff[iBuff].end(), RECOG::DDD::SortCompare);

	//

	return 0.0f;
}

void DDDetector::DetectDominantShiftPoints2(
	Mesh *pMeshM,
	Mesh *pMeshQ,
	Array<int> *dominantShiftPointsIdx,
	Vector3<float> &dominantShift,
	bool bVisualize)
{
	// Debug values
	// this fcn should calculate ROI params
	/*ROI.minx = -0.300;
	ROI.maxx = 0.200;
	ROI.miny = 0.150;
	ROI.maxy = 0.400;
	ROI.minz = 0.900;
	ROI.maxz = 1.600;*/
	// END Debug values

	// Mesh* pMeshM, *pMeshQ;
	PointAssociationData pointAssociationData;
	Array<int> ptBuff;
	ptBuff.Element = new int[camera.w * camera.h];
	Array<Point> pointsQM;
	pointsQM.Element = new Point[camera.w * camera.h];
	// Array<OrientedPoint> pointsM, pointsQ;
	Array<OrientedPoint> pointsM;
	Array<OrientedPoint> pointsQSubsampled, pointsQValidShift, dominantShiftPoints;
	pointsQSubsampled.Element = NULL;
	pointsQValidShift.Element = NULL;
	dominantShiftPoints.Element = NULL;
	Array<int> pointsQSubsampledIdx;
	Array<Pair<int, int>> pointQSurfaceMAssotiations;
	Array<Pair<int, int>> pointQSurfaceMAssotiationsInShiftDirection;
	// pointsM.Element = new OrientedPoint[camera.w * camera.h];
	pointsM.Element = new OrientedPoint[camera.w * camera.h];
	AffinePose3D pose;
	float s[3] = {1.0, 1.0, 1.0};
	float t[3] = {0.0, 0.0, 0.0};
	float R[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	RVLCOPY3VECTOR(s, pose.s);
	RVLCOPY3VECTOR(t, pose.t);
	RVLCOPYMX3X3(R, pose.R);

	// pMeshM = meshSeq.Element + meshSeq.n - 1;

#ifdef RVLLINUX
	// CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
	MeshSubsample(pMeshQ, &pointsQSubsampledIdx);
	CreateOrientedPointArrayFromPointArray(pMeshQ->NodeArray, pointsQSubsampledIdx, pointsQSubsampled);
#else
	// MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
	MeshSubsample(pMeshQ, &pointsQSubsampledIdx);
	MESH::CreateOrientedPointArrayFromPointArray(pMeshQ->NodeArray, pointsQSubsampledIdx, pointsQSubsampled);
	// pMeshQ = meshSeq.Element + meshSeq.n - 1 - ROICalculationStep;
	MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
#endif

	// pMeshQ = meshSeq.Element + meshSeq.n - 1 - ROICalculationStep;
	CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
	pointAssociationData.Create(pointsQSubsampled.n, pointsM.n, true);
	imgGrid.Create(pMeshM->NodeArray, &camera, ROIPointAssociationGridCellSize);
	// imgGrid.Create(pMeshQ->NodeArray, &camera, pointAssociationGridCellSize);
	SampleImage(pMeshM);
	// PointAssociation(pointsM, &pose, pointsQ, pointAssociationData, ptBuff, true, &pointsMQ);
	PointAssociation(pointsQSubsampled, &pose, pointsM, pointAssociationData, ptBuff, true, &pointsQM);

	Array<Vector3<float>> pointsShift;
	Voter3D voter;
	float cellSize[3] = {votingCellSizeX, votingCellSizeY, votingCellSizeZ};
	CalculatePointsShiftVector2(pMeshQ->NodeArray, pointsQSubsampledIdx, pointsM, &pointAssociationData, &pointsShift, minPointShift, pointQSurfaceMAssotiations);

	// Visualization.

	voter.Vote(pointsShift, cellSize);
	dominantShift = voter.GetMax();

	float shiftThresh[3] = {shiftThreshX, shiftThreshY, shiftThreshZ};
	SurfaceShift(pointQSurfaceMAssotiations, pointsShift, dominantShift, shiftThresh, pointQSurfaceMAssotiationsInShiftDirection);

	if (bVisualize && pVisualizationData->bVisualizeROIDetection)
	{
		uchar green[] = {0, 255, 0};
		uchar blue[] = {0, 0, 255};
		uchar red[] = {255, 0, 0};
		uchar cyan[] = {0, 255, 255};
		uchar white[] = {255, 255, 255};

		Array<Point> pointsQ;
		pointsQ.n = pointQSurfaceMAssotiations.n;
		pointsQ.Element = new Point[pointsQ.n];
		Array<Point> pointAssociationLineEndPts;
		pointAssociationLineEndPts.n = 2 * pointQSurfaceMAssotiations.n;
		pointAssociationLineEndPts.Element = new Point[pointAssociationLineEndPts.n];
		Array<Pair<int, int>> pointAssociationLines;
		pointAssociationLines.n = pointQSurfaceMAssotiations.n;
		pointAssociationLines.Element = new Pair<int, int>[pointAssociationLines.n];
		Point *pPtM, *pPtQ;
		Pair<int, int> *pPointQSurfaceM;
		float *shift;
		for (int i = 0; i < pointQSurfaceMAssotiations.n; i++)
		{
			pPointQSurfaceM = pointQSurfaceMAssotiations.Element + i;
			pPtQ = pointAssociationLineEndPts.Element + 2 * i;
			*pPtQ = pointsQ.Element[i] = pMeshQ->NodeArray.Element[pPointQSurfaceM->a];
			pPtM = pointAssociationLineEndPts.Element + 2 * i + 1;
			shift = pointsShift.Element[i].Element;
			RVLDIF3VECTORS(pPtQ->P, shift, pPtM->P);
			pointAssociationLines.Element[i].a = 2 * i;
			pointAssociationLines.Element[i].b = 2 * i + 1;
		}
		pVisualizationData->pVisualizer->DisplayLines(pointAssociationLineEndPts, pointAssociationLines, white);
		pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pointsQ, blue, 2.0);
		Array<Point> pointsQInShiftDirection;
		pointsQInShiftDirection.n = pointQSurfaceMAssotiationsInShiftDirection.n;
		pointsQInShiftDirection.Element = new Point[pointsQInShiftDirection.n];
		for (int i = 0; i < pointQSurfaceMAssotiationsInShiftDirection.n; i++)
		{
			pPointQSurfaceM = pointQSurfaceMAssotiationsInShiftDirection.Element + i;
			pointsQInShiftDirection.Element[i] = pMeshQ->NodeArray.Element[pPointQSurfaceM->a];
		}
		pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pointsQInShiftDirection, green, 6.0);

		delete[] pointAssociationLineEndPts.Element;
		delete[] pointAssociationLines.Element;
		delete[] pointsQ.Element;
		delete[] pointsQInShiftDirection.Element;

		pVisualizationData->pVisualizer->Run();
		pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
	}

	RVL_DELETE_ARRAY(ptBuff.Element);
	RVL_DELETE_ARRAY(pointsQM.Element);
	// RVL_DELETE_ARRAY(pointsM.Element);
	RVL_DELETE_ARRAY(pointsM.Element);
	RVL_DELETE_ARRAY(pointsQSubsampled.Element);
	RVL_DELETE_ARRAY(pointsQSubsampledIdx.Element);

	RVL_DELETE_ARRAY(pointsShift.Element);
	RVL_DELETE_ARRAY(pointsQValidShift.Element);
	RVL_DELETE_ARRAY(pointQSurfaceMAssotiations.Element);
	RVL_DELETE_ARRAY(pointQSurfaceMAssotiationsInShiftDirection.Element);
	RVL_DELETE_ARRAY(dominantShiftPoints.Element);
}

void DDDetector::CalculatePointsShiftVector2(
	Array<Point> pointsQ,
	Array<int> subsampledPointsQIdx,
	Array<OrientedPoint> pointsM,
	PointAssociationData *pPointAssociationData,
	Array<RVL::Vector3<float>> *pointsShift,
	float shiftThresh,
	Array<Pair<int, int>> &pointQSurfaceMAssotiations)
{
	int *MNN = pPointAssociationData->QNN;

	if (pointsShift->Element)
		RVL_DELETE_ARRAY(pointsShift->Element);
	pointsShift->Element = new Vector3<float>[pointsQ.n];
	if (pointQSurfaceMAssotiations.Element)
		RVL_DELETE_ARRAY(pointQSurfaceMAssotiations.Element);
	pointQSurfaceMAssotiations.Element = new Pair<int, int>[pointsQ.n];

	int iMPt, iQPt;
	float *pQPt, *pPt;
	int nPoints = 0;
	float diff[3];
	int iSurfel;
	float *N;
	float s;
	Surfel *pSurfel;
	Pair<int, int> *pPointQSurfaceMAssotiation;
	for (int i = 0; i < pointsQ.n; i++)
	{
		iQPt = subsampledPointsQIdx.Element[i];
		iMPt = MNN[i];
		if (iMPt >= 0)
		{
			iSurfel = planarSurfaces.surfelMap[iMPt];
			if (iSurfel >= 0 && iSurfel < planarSurfaces.NodeArray.n)
			{
				pSurfel = planarSurfaces.NodeArray.Element + iSurfel;
				pQPt = pointsQ.Element[i].P;
				RVLDIF3VECTORS(pQPt, pSurfel->P, diff);
				N = pSurfel->N;
				s = RVLDOTPRODUCT3(N, diff);
				if (RVLABS(s) < shiftThresh)
					continue;
				else
				{
					pPt = pointsShift->Element[nPoints].Element;
					RVLSCALE3VECTOR(N, s, pPt);
					pPointQSurfaceMAssotiation = pointQSurfaceMAssotiations.Element + nPoints;
					pPointQSurfaceMAssotiation->a = iQPt;
					pPointQSurfaceMAssotiation->b = iSurfel;
					nPoints++;
				}
			}
		}
	}

	pointsShift->n = nPoints;
	pointQSurfaceMAssotiations.n = nPoints;
}

void DDDetector::SurfaceShift(
	Array<Pair<int, int>> pointQSurfaceMAssotiations,
	Array<RVL::Vector3<float>> shifts,
	Vector3<float> pointsShift,
	float *shiftThresh,
	Array<Pair<int, int>> &pointQSurfaceMAssotiationsInShiftDirection)
{
	int iMPt, iQPt;
	float *pMPt, *pQPt;
	float *shift;
	int nValid = 0;
	OrientedPoint *pPt;
	Pair<int, int> *pPointQSurfaceM;

	if (pointQSurfaceMAssotiationsInShiftDirection.Element)
		RVL_DELETE_ARRAY(pointQSurfaceMAssotiationsInShiftDirection.Element);
	pointQSurfaceMAssotiationsInShiftDirection.Element = new Pair<int, int>[pointQSurfaceMAssotiations.n];
	pointQSurfaceMAssotiationsInShiftDirection.n = 0;
	RVLSCALE3VECTOR(shiftThresh, 0.5, shiftThresh);
	float lowerThresh[3] = {pointsShift.Element[0] - shiftThresh[0], pointsShift.Element[1] - shiftThresh[1], pointsShift.Element[2] - shiftThresh[2]};
	float upperThresh[3] = {pointsShift.Element[0] + shiftThresh[0], pointsShift.Element[1] + shiftThresh[1], pointsShift.Element[2] + shiftThresh[2]};

	for (int i = 0; i < pointQSurfaceMAssotiations.n; i++)
	{
		shift = shifts.Element[i].Element;
		if ((lowerThresh[0] < shift[0]) && (shift[0] < upperThresh[0]))
			if ((lowerThresh[1] < shift[1]) && (shift[1] < upperThresh[1]))
				if ((lowerThresh[2] < shift[2]) && (shift[2] < upperThresh[2]))
					pointQSurfaceMAssotiationsInShiftDirection.Element[pointQSurfaceMAssotiationsInShiftDirection.n++] = pointQSurfaceMAssotiations.Element[i];
	}
}

void DDDetector::SaveRectangularStructure(
	std::string fileName,
	RECOG::DDD::RectStruct *pRectStruct)
{
	FILE *fp = fopen(fileName.data(), "wb");
	fwrite(pRectStruct->RRS, sizeof(float), 9, fp);
	fwrite(&(pRectStruct->rects.n), sizeof(int), 1, fp);
	fwrite(pRectStruct->rects.Element, sizeof(RECOG::DDD::Rect3D), pRectStruct->rects.n, fp);
	fclose(fp);
}

void DDDetector::LoadRectangularStructure(
	std::string fileName,
	RECOG::DDD::RectStruct *pRectStruct)
{
	FILE *fp = fopen(fileName.data(), "rb");
	fread(pRectStruct->RRS, sizeof(float), 9, fp);
	fread(&(pRectStruct->rects.n), sizeof(int), 1, fp);
	pRectStruct->rects.Element = new RECOG::DDD::Rect3D[pRectStruct->rects.n];
	fread(pRectStruct->rects.Element, sizeof(RECOG::DDD::Rect3D), pRectStruct->rects.n, fp);
	fclose(fp);
}

void DDDetector::DDBox(
	RECOG::DDD::HypothesisDoorDrawer *pMovingPartHyp,
	int iState,
	AffinePose3D *pBox)
{
	float q = (pMovingPartHyp->state.n > 0 ? pMovingPartHyp->state.Element[iState].q : 0.0f);
	if (pMovingPartHyp->objClass == RVLDDD_MODEL_DOOR)
	{
		Pose3D poseBA;
		float tBB_[3];
		RVLSET3VECTOR(tBB_, pMovingPartHyp->r[0], pMovingPartHyp->r[1], 0.0f);
		RVLROTZ(cos(q), sin(q), poseBA.R);
		RVLMULMX3X3VECT(poseBA.R, tBB_, poseBA.t);
		RVLCOMPTRANSF3D(pMovingPartHyp->pose.R, pMovingPartHyp->pose.t, poseBA.R, poseBA.t, pBox->R, pBox->t);
	}
	else if (pMovingPartHyp->objClass == RVLDDD_MODEL_DRAWER)
	{
		float tBA[3];
		tBA[1] = tBA[2] = 0.0f;
		RVLCOPYMX3X3(pMovingPartHyp->pose.R, pBox->R);
		tBA[0] = q;
		RVLTRANSF3(tBA, pMovingPartHyp->pose.R, pMovingPartHyp->pose.t, pBox->t);
	}
	pBox->s[0] = frontFaceThickness;
	pBox->s[1] = pMovingPartHyp->s[0];
	pBox->s[2] = pMovingPartHyp->s[1];
}

void DDDetector::ClearModel(RECOG::DDD::Model *pModel)
{
	if (pModel->points.Element)
		RVL_DELETE_ARRAY(pModel->points.Element);
	if (pModel->points_.Element)
		RVL_DELETE_ARRAY(pModel->points_.Element);
}

void DDDetector::SaveDD(
	FILE *fp,
	std::vector<RECOG::DDD::HypothesisDoorDrawer> &movingParts)
{
	RECOG::DDD::HypothesisDoorDrawer movingPart;
	for (int iMovingPart = 0; iMovingPart < movingParts.size(); iMovingPart++)
	{
		movingPart = movingParts[iMovingPart];
		if (movingPart.objClass == RVLDDD_MODEL_DRAWER)
			movingPart.r[0] = movingPart.r[1] = 0.0f;
		fprintf(fp, "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0.0\t%f\t%f\t%f\t%f\t%f\n",
				movingPart.objClass,
				movingPart.pose.R[0], movingPart.pose.R[1], movingPart.pose.R[2],
				movingPart.pose.R[3], movingPart.pose.R[4], movingPart.pose.R[5],
				movingPart.pose.R[6], movingPart.pose.R[7], movingPart.pose.R[8],
				movingPart.pose.t[0], movingPart.pose.t[1], movingPart.pose.t[2],
				movingPart.s[0], movingPart.s[1], movingPart.r[0], movingPart.r[1], movingPart.openingDirection);
	}
}

int DDDetector::LoadDD(
	FILE *fp,
	RECOG::DDD::HypothesisDoorDrawer *pMovingPart)
{
	return fscanf(fp, "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0.0\t%f\t%f\t%f\t%f\t%f\n",
				  &(pMovingPart->objClass),
				  pMovingPart->pose.R + 0, pMovingPart->pose.R + 1, pMovingPart->pose.R + 2,
				  pMovingPart->pose.R + 3, pMovingPart->pose.R + 4, pMovingPart->pose.R + 5,
				  pMovingPart->pose.R + 6, pMovingPart->pose.R + 7, pMovingPart->pose.R + 8,
				  pMovingPart->pose.t, pMovingPart->pose.t + 1, pMovingPart->pose.t + 2,
				  pMovingPart->s, pMovingPart->s + 1, pMovingPart->r, pMovingPart->r + 1, &(pMovingPart->openingDirection));
}

void DDDetector::LoadDD(
	FILE *fp,
	std::vector<RECOG::DDD::HypothesisDoorDrawer> &movingParts)
{
	RECOG::DDD::HypothesisDoorDrawer movingPart;
	while (LoadDD(fp, &movingPart) == 18)
		movingParts.push_back(movingPart);
}

void DDDetector::SaveDDStates(
	FILE *fp,
	std::vector<RECOG::DDD::HypothesisDoorDrawer> &DDs)
{
	RECOG::DDD::HypothesisDoorDrawer DD;
	RECOG::DDD::AOHypothesisState state;
	for (int iDD = 0; iDD < DDs.size(); iDD++)
	{
		DD = DDs[iDD];
		fprintf(fp, "%d: ", DD.state.n);
		for (int m = 0; m < DD.state.n; m++)
		{
			state = DD.state.Element[m];
			fprintf(fp, "(%d, %f, %f) ", state.imgID, state.q, state.score);
		}
		fprintf(fp, "\n");
	}
}

void DDDetector::LoadDDStates(
	FILE *fp,
	std::vector<RECOG::DDD::HypothesisDoorDrawer> &DDs)
{
	RECOG::DDD::HypothesisDoorDrawer *pDD = DDs.data();
	RECOG::DDD::AOHypothesisState *pState;
	while (!feof(fp))
	{
		fscanf(fp, "%d: ", &(pDD->state.n));
		pDD->state.Element = new RECOG::DDD::AOHypothesisState[pDD->state.n];
		pState = pDD->state.Element;
		for (int m = 0; m < pDD->state.n; m++, pState++)
			fscanf(fp, "(%d, %f, %f) ", &(pState->imgID), &(pState->q), &(pState->score));
		fscanf(fp, "\n");
		pDD++;
	}
}

void DDDetector::LoadArticulatedObject(
	char *modelFilePath,
	char *AOFileName,
	RECOG::DDD::ArticulatedObject &AObj)
{
	std::string modelFilePath_ = modelFilePath;
	if (AOFileName == NULL)
		AOFileName = "AO.txt";
	std::string AOFileName_ = AOFileName;
	modelFilePath_.append("\\" + AOFileName_);
	FILE *fp = fopen(modelFilePath_.data(), "r");
	char AORSFileName[200];
	fscanf(fp, "%s\n", AORSFileName);
	std::string AORSFilePath = modelFilePath_.substr(0, modelFilePath_.rfind('\\') + 1) + AORSFileName;
	LoadRectangularStructure(AORSFilePath, &(AObj.MRS));
	int nMovingParts;
	fscanf(fp, "%d\n", &nMovingParts);
	AObj.movingParts.Element = new RECOG::DDD::HypothesisDoorDrawer[nMovingParts];
	RECOG::DDD::HypothesisDoorDrawer *pMovingPart = AObj.movingParts.Element;
	int iMovingPart;
	int movingPartType;
	for (iMovingPart = 0; iMovingPart < nMovingParts; iMovingPart++, pMovingPart++)
	{
		LoadDD(fp, pMovingPart);
		// fscanf(fp, "%d\t", &movingPartType);
		// pMovingPart->objClass = movingPartType;
		// pMovingPart->state.Element = NULL;
		// if (movingPartType == 0)
		//{
		//	fscanf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0.0\t%f\t%f\t%f\t%f\n",
		//		pMovingPart->pose.R + 0, pMovingPart->pose.R + 1, pMovingPart->pose.R + 2,
		//		pMovingPart->pose.R + 3, pMovingPart->pose.R + 4, pMovingPart->pose.R + 5,
		//		pMovingPart->pose.R + 6, pMovingPart->pose.R + 7, pMovingPart->pose.R + 8,
		//		pMovingPart->pose.t, pMovingPart->pose.t + 1, pMovingPart->pose.t + 2,
		//		pMovingPart->s, pMovingPart->s + 1, pMovingPart->r, pMovingPart->r + 1);
		//	pMovingPart++;
		// }
		// else
		//{
		//	fscanf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t0.0\t%f\t%f\t0.0\t0.0\n",
		//		pMovingPart->pose.R + 0, pMovingPart->pose.R + 1, pMovingPart->pose.R + 2,
		//		pMovingPart->pose.R + 3, pMovingPart->pose.R + 4, pMovingPart->pose.R + 5,
		//		pMovingPart->pose.R + 6, pMovingPart->pose.R + 7, pMovingPart->pose.R + 8,
		//		pMovingPart->pose.t, pMovingPart->pose.t + 1, pMovingPart->pose.t + 2,
		//		pMovingPart->s, pMovingPart->s + 1);
		//	pMovingPart++;
		// }
	}
	AObj.movingParts.n = nMovingParts;
	fclose(fp);
}

void DDDetector::SurfelEdges(
	Mesh *pMesh,
	float *surfelRefPt,
	int minSurfelSize,
	int minEdgeSize)
{
	// Count surfel edges.

	int iSurfel, iSurfel_;
	Surfel *pSurfel, *pSurfel_;
	SURFEL::EdgePtr *pAdjacentSurfelEdgePtr;
	SURFEL::Edge *pAdjacentSurfelEdge;
	int nEdges = 0;
	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;
		if (pSurfel->bEdge)
			continue;
		if (pSurfel->size < minSurfelSize)
			continue;
		pAdjacentSurfelEdgePtr = pSurfel->EdgeList.pFirst;
		while (pAdjacentSurfelEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iSurfel, pAdjacentSurfelEdgePtr, pAdjacentSurfelEdge, iSurfel_);
			pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;
			if (pSurfel_->bEdge && pSurfel_->size >= minEdgeSize)
				nEdges++;
			pAdjacentSurfelEdgePtr = pAdjacentSurfelEdgePtr->pNext;
		}
	}

	// Create surfel edges.

	float maxEdgeParentDist = 2.0f / pSurfelDetector->kPlane;
	RVL_DELETE_ARRAY(surfelEdges.Element);
	surfelEdges.Element = new RECOG::DDD::SurfelEdge[nEdges];
	RVL_DELETE_ARRAY(surfelEdges_);
	surfelEdges_ = new Array<RECOG::DDD::SurfelEdge>[pSurfels->NodeArray.n];
	RECOG::DDD::SurfelEdge *pSurfelEdge = surfelEdges.Element;
	float *P;
	float *surfelRefPts;
	int i;
	float fTmp;
	float V3Tmp[3];
	QLIST::Index2 *pPtIdx;
	float edgeParentDist;
	int edgeSize;
	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;
		if (pSurfel->bEdge)
			continue;
		if (pSurfel->size < minSurfelSize)
			continue;
		pAdjacentSurfelEdgePtr = pSurfel->EdgeList.pFirst;
		surfelEdges_[iSurfel].Element = pSurfelEdge;
		while (pAdjacentSurfelEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iSurfel, pAdjacentSurfelEdgePtr, pAdjacentSurfelEdge, iSurfel_);
			pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;
			surfelRefPts = surfelRefPt + 12 * iSurfel_;
			if (pSurfel_->bEdge && pSurfel_->size >= minEdgeSize)
			{
				edgeSize = 0;
				pPtIdx = pSurfel_->PtList.pFirst;
				while (pPtIdx)
				{
					P = pMesh->NodeArray.Element[pPtIdx->Idx].P;
					edgeParentDist = RVLDOTPRODUCT3(pSurfel->N, P) - pSurfel->d;
					if (RVLABS(edgeParentDist) <= maxEdgeParentDist)
						edgeSize++;
					pPtIdx = pPtIdx->pNext;
				}
				if (edgeSize >= minEdgeSize)
				{
					pSurfelEdge->iEdgeSurfel = iSurfel_;
					for (i = 0; i < 2; i++)
					{
						P = surfelRefPts + 3 * (i + 1);
						fTmp = pSurfel->d / RVLDOTPRODUCT3(pSurfel->N, P);
						RVLSCALE3VECTOR(P, fTmp, pSurfelEdge->P[i]);
					}
					RVLCROSSPRODUCT3(pSurfel->N, pSurfel_->N, V3Tmp);
					RVLCROSSPRODUCT3(V3Tmp, pSurfel->N, pSurfelEdge->N);
					RVLNORM3(pSurfelEdge->N, fTmp);
					pSurfelEdge++;
				}
			}
			pAdjacentSurfelEdgePtr = pAdjacentSurfelEdgePtr->pNext;
		}
		surfelEdges_[iSurfel].n = pSurfelEdge - surfelEdges_[iSurfel].Element;
	}
	surfelEdges.n = pSurfelEdge - surfelEdges.Element;
}

void DDDetector::PlanarSurfaceEdges(Mesh *pMesh)
{
	float maxEdgeParentDist = 2.0f / pSurfelDetector->kPlane;
	RVL_DELETE_ARRAY(planarSurfeceEdges);
	planarSurfeceEdges = new Array<RECOG::DDD::SurfelEdge *>[planarSurfaces.NodeArray.n];
	RVL_DELETE_ARRAY(planarSurfeceEdgeMem);
	planarSurfeceEdgeMem = new RECOG::DDD::SurfelEdge *[surfelEdges.n];
	bool *bJoined = new bool[surfelEdges.n];
	int iPlanarSurface;
	Surfel *pPlanarSurface;
	QLIST::Index *pMemberSurfelIdx;
	int i;
	Array<RECOG::DDD::SurfelEdge> memberSurfelEdges;
	RECOG::DDD::SurfelEdge *pSurfelEdge;
	int iSurfelEdge;
	RECOG::DDD::SurfelEdge **ppPlanarSurfaceEdge = planarSurfeceEdgeMem;
	int edgeSize;
	QLIST::Index2 *pPtIdx;
	Surfel *pEdgeSurfel;
	float *P;
	float edgeParentDist;
	for (iPlanarSurface = 0; iPlanarSurface < planarSurfaces.NodeArray.n; iPlanarSurface++)
	{
		pPlanarSurface = planarSurfaces.NodeArray.Element + iPlanarSurface;
		memset(bJoined, 0, surfelEdges.n * sizeof(bool));
		planarSurfeceEdges[iPlanarSurface].Element = ppPlanarSurfaceEdge;
		pMemberSurfelIdx = pPlanarSurface->children.pFirst;
		while (pMemberSurfelIdx)
		{
			memberSurfelEdges = surfelEdges_[pMemberSurfelIdx->Idx];
			for (i = 0; i < memberSurfelEdges.n; i++)
			{
				pSurfelEdge = memberSurfelEdges.Element + i;
				iSurfelEdge = pSurfelEdge - surfelEdges.Element;
				if (bJoined[iSurfelEdge])
					continue;
				bJoined[iSurfelEdge] = true;
				edgeSize = 0;
				pEdgeSurfel = pSurfels->NodeArray.Element + pSurfelEdge->iEdgeSurfel;
				pPtIdx = pEdgeSurfel->PtList.pFirst;
				while (pPtIdx)
				{
					P = pMesh->NodeArray.Element[pPtIdx->Idx].P;
					edgeParentDist = RVLDOTPRODUCT3(pPlanarSurface->N, P) - pPlanarSurface->d;
					if (RVLABS(edgeParentDist) <= maxEdgeParentDist)
						edgeSize++;
					pPtIdx = pPtIdx->pNext;
				}
				if (edgeSize >= minEdgeSize)
					*(ppPlanarSurfaceEdge++) = pSurfelEdge;
			}
			pMemberSurfelIdx = pMemberSurfelIdx->pNext;
		}
		planarSurfeceEdges[iPlanarSurface].n = ppPlanarSurfaceEdge - planarSurfeceEdges[iPlanarSurface].Element;
	}
	delete[] bJoined;
}

#define RVLMOMENTS3UPDATEWITHWEIGHTS(x, w, M, C, n) \
	{                                               \
		n += w;                                     \
		M[0] += (w * x[0]);                         \
		M[1] += (w * x[1]);                         \
		M[2] += (w * x[2]);                         \
		C[0] += (w * x[0] * x[0]);                  \
		C[1] += (w * x[0] * x[1]);                  \
		C[2] += (w * x[0] * x[2]);                  \
		C[4] += (w * x[1] * x[1]);                  \
		C[5] += (w * x[1] * x[2]);                  \
		C[8] += (w * x[2] * x[2]);                  \
	}

template <class Type>
void GetCovMatrix3W(
	Moments<Type> *pMoments,
	Type w,
	Type *C,
	Type *M)
{
	M[0] = pMoments->S[0] / w;
	M[1] = pMoments->S[1] / w;
	M[2] = pMoments->S[2] / w;
	C[0] = pMoments->S2[0] / w - M[0] * M[0];
	C[1] = pMoments->S2[1] / w - M[0] * M[1];
	C[2] = pMoments->S2[2] / w - M[0] * M[2];
	C[3] = C[1];
	C[4] = pMoments->S2[4] / w - M[1] * M[1];
	C[5] = pMoments->S2[5] / w - M[1] * M[2];
	C[6] = C[2];
	C[7] = C[5];
	C[8] = pMoments->S2[8] / w - M[2] * M[2];
}

void DDDetector::AccuratePlaneFitting(Mesh *pMesh)
{
	// Parameters.

	int nIterations = 100;
	float c = 0.010f; // m

	// Constants.

	float c2 = c * c;

	//

	Surfel *pSurfel = planarSurfaces.NodeArray.Element + 1;
	int nPts = pSurfel->size;
	Array<Point> visPts[2];
	int iPtSubset;
	for (iPtSubset = 0; iPtSubset < 2; iPtSubset++)
	{
		visPts[iPtSubset].Element = new Point[nPts];
		visPts[iPtSubset].n = 0;
	}
	float *N = pSurfel->N;
	float d = pSurfel->d;
	float r;
	Point *pPt, *pPt_;
	QLIST::Index2 *pPtIdx;
	uchar green[] = {0, 255, 0};
	uchar blue[] = {0, 0, 255};
	Moments<double> moments;
	double P[3];
	int iPt;
	double *w = new double[nPts];
	for (iPt = 0; iPt < nPts; iPt++)
		w[iPt] = 1.0f;
	float r2max = 0.0f;
	float r2;
	pPtIdx = pSurfel->PtList.pFirst;
	while (pPtIdx)
	{
		pPt = pMesh->NodeArray.Element + pPtIdx->Idx;
		r = RVLDOTPRODUCT3(N, pPt->P) - d;
		r2 = r * r;
		if (r2 > r2max)
			r2max = r2;
		pPtIdx = pPtIdx->pNext;
	}
	double mu = c2 / (2.0f * r2max - c2);
	double fTmp;
	double a;
	double b1, b2;
	float wTotal;
	double M[3];
	cv::Mat cvC(3, 3, CV_64FC1);
	double *C = (double *)(cvC.data);
	cv::Mat cvEigC, cvEigVC;
	double *eigC;
	double *eigVC;
	double *minEigVC;
	vtkSmartPointer<vtkActor> ptActor[2];
	pVisualizationData->pVisualizer->SetMesh(pMesh);
	cv::Mat displayImg(pMesh->height, pMesh->width, CV_8UC3);
	uchar *pixArray = (uchar *)(displayImg.data);
	uchar *pix;
	for (int k = 0; k < nIterations; k++)
	{
		if (k > 0)
		{
			fTmp = mu + 1.0f;
			a = c * sqrt(mu * fTmp);
			b1 = mu / fTmp * c2;
			b2 = fTmp / mu * c2;
			iPt = 0;
			pPtIdx = pSurfel->PtList.pFirst;
			while (pPtIdx)
			{
				pPt = pMesh->NodeArray.Element + pPtIdx->Idx;
				r = RVLDOTPRODUCT3(N, pPt->P) - d;
				r2 = r * r;
				if (r2 <= b1)
					w[iPt] = 1.0f;
				else if (r2 < b2)
					w[iPt] = a / RVLABS(r) - mu;
				else
					w[iPt] = 0.0f;
				iPt++;
				pPtIdx = pPtIdx->pNext;
			}
			InitMoments<double>(moments);
			iPt = 0;
			wTotal = 0.0f;
			pPtIdx = pSurfel->PtList.pFirst;
			while (pPtIdx)
			{
				pPt = pMesh->NodeArray.Element + pPtIdx->Idx;
				RVLCOPY3VECTOR(pPt->P, P);
				RVLMOMENTS3UPDATEWITHWEIGHTS(P, w[iPt], moments.S, moments.S2, wTotal);
				iPt++;
				pPtIdx = pPtIdx->pNext;
			}
			GetCovMatrix3W<double>(&moments, wTotal, C, M);
			cv::eigen(cvC, cvEigC, cvEigVC);
			eigC = (double *)(cvEigC.data);
			eigVC = (double *)(cvEigVC.data);
			minEigVC = eigVC + 6;
			if (RVLDOTPRODUCT3(minEigVC, M) <= 0.0f)
			{
				RVLCOPY3VECTOR(minEigVC, N);
			}
			else
			{
				RVLNEGVECT3(minEigVC, N);
			}
			d = RVLDOTPRODUCT3(N, M);
		}
		visPts[0].n = visPts[1].n = 0;
		displayImg.setTo(cv::Scalar(255, 0, 0));
		pPtIdx = pSurfel->PtList.pFirst;
		while (pPtIdx)
		{
			pPt = pMesh->NodeArray.Element + pPtIdx->Idx;
			r = RVLDOTPRODUCT3(N, pPt->P) - d;
			iPtSubset = (r >= 0.0f ? 0 : 1);
			pPt_ = visPts[iPtSubset].Element + visPts[iPtSubset].n;
			visPts[iPtSubset].n++;
			RVLCOPY3VECTOR(pPt->P, pPt_->P);
			pix = pixArray + 3 * pPtIdx->Idx;
			if (r >= 0.0f)
				RVLSET3VECTOR(pix, 255, 255, 255)
			else
				RVLSET3VECTOR(pix, 0, 0, 0)
			pPtIdx = pPtIdx->pNext;
		}
		// ptActor[0] = pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(visPts[0], green, 4);
		// ptActor[1] = pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(visPts[1], blue, 4);
		// RunVisualizer();
		// pVisualizationData->pVisualizer->renderer->RemoveActor(ptActor[0]);
		// pVisualizationData->pVisualizer->renderer->RemoveActor(ptActor[1]);
		cv::imshow("error map", displayImg);
		cv::waitKey();
	}
	delete[] w;
}

void DDDetector::InitVisualizer(Visualizer *pVisualizerIn)
{
	if (pVisualizationData == NULL)
		pVisualizationData = new RECOG::DDD::DisplayCallbackData;
	if (pVisualizerIn)
	{
		pVisualizationData->pVisualizer = pVisualizerIn;
		pVisualizationData->bOwnVisualizer = false;
	}
	else
	{
		pVisualizationData->pVisualizer = new Visualizer;
		pVisualizationData->bOwnVisualizer = true;
	}
	pVisualizationData->AssociatedPts.Element = NULL;
	// pVisualizer->SetMouseRButtonDownCallback(RECOG::TGRAPH::MouseRButtonDown, pData);
	pVisualizationData->bVisualizeICPSteps = false;
	pVisualizationData->bVisualizePtAssociation = false;
	pVisualizationData->bVisualizeInitialHypothesis = false;
	pVisualizationData->bVisualizeBestHypothesis = false;
	pVisualizationData->bVisualizeModelPts = false;
	pVisualizationData->bVisualizeSurfels = false;
	pVisualizationData->bVisualizeFittingScoreCalculation = false;
	pVisualizationData->bVisualizeROIDetection = false;
	pVisualizationData->bVisualizeDoorHypotheses = false;
	pVisualizationData->bVisualizeROI = false;
	pVisualizationData->bVisualizeRectangularStructure = false;
	pVisualizationData->bVisualizeRectStructMatching = false;
	pVisualizationData->bVisualizeAOInitState = false;
	pVisualizationData->bRGBImageVisualization = false;
	pVisualizationData->b3DVisualization = true;
	pVisualizationData->bPointToPlane = false;
	pVisualizationData->paramList.m_pMem = pMem0;
	RVLPARAM_DATA *pParamData;
	pVisualizationData->paramList.Init();
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeSurfels", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeSurfels));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeICPSteps", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeICPSteps));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizePtAssociation", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizePtAssociation));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeInitialHypothesis", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeInitialHypothesis));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeBestHypothesis", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeBestHypothesis));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeModelPts", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeModelPts));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeFittingScoreCalculation", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeFittingScoreCalculation));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeROIDetection", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeROIDetection));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeHoughTransform", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeHoughTransform));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeRANSACIterations", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeRANSACIterations));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeSolution", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeSolution));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeDoorHypotheses", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeDoorHypotheses));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeROI", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeROI));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeRectangularStructure", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeRectangularStructure));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeRectStructMatching", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeRectStructMatching));
	pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeAOInitState", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeAOInitState));
	pParamData = pVisualizationData->paramList.AddParam("DDD.3DVisualization", RVLPARAM_TYPE_BOOL, &(pVisualizationData->b3DVisualization));
	pParamData = pVisualizationData->paramList.AddParam("DDD.RGBImageVisualization", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bRGBImageVisualization));
	pVisualizationData->paramList.LoadParams((char *)(cfgFileName.data()));
}

void DDDetector::RunVisualizer()
{
	pVisualizationData->pVisualizer->Run();
}

void DDDetector::ClearVisualization()
{
	pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
}

Visualizer *DDDetector::GetVisualizer()
{
	return pVisualizationData->pVisualizer;
}

void DDDetector::VisualizeHypothesisBoundingBox(
	RECOG::DDD::Hypothesis *pHyp,
	Mesh *pMesh)
{
	double red[] = {1.0, 0, 0};
	// uchar green[] = { 0, 255, 0 };

	// Display surfels.

	// pSurfels->NodeColors(green);
	// pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);
	// pSurfels->Display(pVisualizer, pMesh);
	// pSurfels->DisplayEdgeFeatures();

	// Visualize ROI.

	// Pose3D ROIPose;
	// RVLUNITMX3(ROIPose.R);
	// RVLSET3VECTOR(ROIPose.t, 0.5f * (ROI.maxx + ROI.minx), 0.5f * (ROI.maxy + ROI.miny), 0.5f * (ROI.maxz + ROI.minz));
	// pVisualizer->DisplayBox(ROI.maxx - ROI.minx, ROI.maxy - ROI.miny, ROI.maxz - ROI.minz, &ROIPose, 255.0, 0.0, 0.0, true);

	// Visualize hypothesis.

	Pose3D bboxPose;
	RVLCOPYMX3X3(pHyp->pose.R, bboxPose.R);
	RVLCOPY3VECTOR(pHyp->pose.t, bboxPose.t);
	pVisualizationData->bboxActor = pVisualizationData->pVisualizer->DisplayBox(pHyp->bboxSize[0], pHyp->bboxSize[1], pHyp->bboxSize[2], &bboxPose, 255.0, 0.0, 0.0, false, 4.0f);
	if (pMesh)
	{
		Array<Point> assocQPts[6];
		Point *ptMem = new Point[6 * pHyp->assoc.n];
		int i;
		for (i = 0; i < 6; i++)
		{
			assocQPts[i].n = 0;
			assocQPts[i].Element = ptMem + i * pHyp->assoc.n;
		}
		int iMSurf;
		for (i = 0; i < pHyp->assoc.n; i++)
		{
			iMSurf = pHyp->assoc.Element[i].b;
			assocQPts[iMSurf].Element[assocQPts[iMSurf].n++] = pMesh->NodeArray.Element[pHyp->assoc.Element[i].a];
		}
		uchar color[6][3];
		RVLSET3VECTOR(color[0], 255, 0, 0);
		RVLSET3VECTOR(color[1], 0, 255, 0);
		RVLSET3VECTOR(color[2], 0, 0, 255);
		RVLSET3VECTOR(color[3], 255, 255, 0);
		RVLSET3VECTOR(color[4], 0, 255, 255);
		RVLSET3VECTOR(color[5], 255, 0, 255);
		for (i = 0; i < 6; i++)
			pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(assocQPts[i], color[i], 6.0f);

		delete[] ptMem;
	}
}

void DDDetector::VisualizeHypothesis(
	AffinePose3D pose,
	PointAssociationData *pPointAssociationData,
	Array<Point> pointsMS_,
	Array<OrientedPoint> *pPointsQ)
{
	uchar red[] = {255, 0, 0};
	uchar green[] = {0, 255, 0};
	uchar yellow[] = {255, 255, 0};
	RECOG::DDD::Model *pModel = models.Element;

	// Visualize model point cloud.

	if (pVisualizationData->bVisualizeModelPts || pVisualizationData->bVisualizePtAssociation)
	{
		Array<Point> pointsMS;
		if (pPointAssociationData->associatedMPts.n > 0)
		{
			pointsMS.n = pModel->points.n;
			pointsMS.Element = new Point[pointsMS.n];
			float *PMM, *PMS;
			float V3Tmp[3];
			for (int iMPt = 0; iMPt < pPointAssociationData->associatedMPts.n; iMPt++)
			{
				PMM = pModel->points.Element[iMPt].P;
				PMS = pointsMS.Element[iMPt].P;
				RVLATRANSF3(PMM, pose.s, pose.R, pose.t, PMS, V3Tmp);
			}
			pVisualizationData->modelPtActor = pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pointsMS, green, 6);
			delete[] pointsMS.Element;
		}

		if (pVisualizationData->bPointToPlane)
			pVisualizationData->modelPtActor = pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pointsMS_, green, 6);

		// Visualize point association.

		if (pVisualizationData->bVisualizePtAssociation)
		{
			Point *pPtSrc;
			Point *pPtTgt = pVisualizationData->AssociatedPts.Element;
			Array<Pair<int, int>> associationLines;
			associationLines.Element = new Pair<int, int>[(pVisualizationData->bPointToPlane ? 0 : pModel->points.n) + pPointAssociationData->explainedQPts.n];
			associationLines.n = 0;
			int iMPt, iQPt;
			if (!pVisualizationData->bPointToPlane)
			{
				for (iMPt = 0; iMPt < pModel->points.n; iMPt++, pPtSrc++, pPtTgt++)
				{
					pPtSrc = pointsMS_.Element + iMPt;
					RVLCOPY3VECTOR(pPtSrc->P, pPtTgt->P);
					iQPt = pPointAssociationData->QNN[iMPt];
					if (iQPt >= 0)
					{
						associationLines.Element[associationLines.n].a = iMPt;
						associationLines.Element[associationLines.n].b = iQPt + pModel->points.n;
						associationLines.n++;
					}
				}
			}
			OrientedPoint *pPtSrc_;
			for (int i = 0; i < pPointAssociationData->explainedQPts.n; i++)
			{
				iQPt = pPointAssociationData->explainedQPts.Element[i];
				pVisualizationData->AssociatedPts.n = 0;
				if (pVisualizationData->bPointToPlane)
				{
					pPtSrc = pointsMS_.Element + i;
					RVLCOPY3VECTOR(pPtSrc->P, pPtTgt->P);
					pPtTgt++;
					pPtSrc_ = pPointsQ->Element + iQPt;
					RVLCOPY3VECTOR(pPtSrc_->P, pPtTgt->P);
					pPtTgt++;
					associationLines.Element[associationLines.n].a = 2 * i;
					associationLines.Element[associationLines.n].b = 2 * i + 1;
					associationLines.n++;
				}
				else
				{
					iMPt = pPointAssociationData->MNN[iQPt];
					if (pPointAssociationData->QNN[iMPt] != iQPt)
					{
						associationLines.Element[associationLines.n].a = iMPt;
						associationLines.Element[associationLines.n].b = iQPt + pModel->points.n;
						associationLines.n++;
					}
				}
			}
			if (pVisualizationData->bPointToPlane)
				pVisualizationData->AssociatedPts.n = pPtTgt - pVisualizationData->AssociatedPts.Element;
			pVisualizationData->associationLinesActor = pVisualizationData->pVisualizer->DisplayLines(pVisualizationData->AssociatedPts, associationLines, red);
			delete[] associationLines.Element;
		}
	}
	// else
	//{
	//	Pose3D bboxPose;
	//	RVLCOPYMX3X3(pose.R, bboxPose.R);
	//	RVLTRANSF3(pVisualizationData->bboxCenter, pose.R, pose.t, bboxPose.t);
	//	double bboxColor[3];
	//	RVLCOPY3VECTOR(red, bboxColor);
	//	pVisualizationData->bboxActor = pVisualizationData->pVisualizer->DisplayBox(pVisualizationData->bboxSize[0], pVisualizationData->bboxSize[1], pVisualizationData->bboxSize[2], &bboxPose, bboxColor[0], bboxColor[1], bboxColor[2], true);
	// }
}

void DDDetector::Display(
	cv::Mat BGR,
	Pose3D *poseFC,
	int nOrthogonalViews,
	Array<Rect<int>> *DDRects)
{
	int i;
	int iView;
	int iDDRect;
	int DDRectVertex[4][2];
	float PF[3], PC[3];
	cv::Point imgP[4];
	Array<Rect<int>> *pDDRects = DDRects;
	Rect<int> *pDDRect;
	for (iView = 0; iView < nOrthogonalViews; iView++, pDDRects++)
	{
		for (iDDRect = 0; iDDRect < pDDRects->n; iDDRect++)
		{
			pDDRect = pDDRects->Element + iDDRect;
			DDRectVertex[0][0] = pDDRect->minx;
			DDRectVertex[0][1] = pDDRect->miny;
			DDRectVertex[1][0] = pDDRect->maxx;
			DDRectVertex[1][1] = pDDRect->miny;
			DDRectVertex[2][0] = pDDRect->maxx;
			DDRectVertex[2][1] = pDDRect->maxy;
			DDRectVertex[3][0] = pDDRect->minx;
			DDRectVertex[3][1] = pDDRect->maxy;
			for (i = 0; i < 4; i++)
			{
				PF[0] = (float)DDRectVertex[i][0] * orthogonalViewPixSize;
				PF[1] = (float)DDRectVertex[i][1] * orthogonalViewPixSize;
				PF[2] = 0.0f;
				RVLTRANSF3(PF, poseFC[iView].R, poseFC[iView].t, PC);
				imgP[i].x = camera.fu * PC[0] / PC[2] + camera.uc;
				imgP[i].y = camera.fv * PC[1] / PC[2] + camera.vc;
			}
			for (i = 0; i < 4; i++)
				cv::line(BGR, imgP[(i + 1) % 4], imgP[i], cv::Scalar(0, 255, 0), 2);
		}
	}
}

void DDDetector::SetSceneForHypothesisVisualization(Mesh *pMesh)
{
	// Display mesh.

	pVisualizationData->pVisualizer->SetMesh(pMesh->pPolygonData);
	// pVisualizer->normalLength = 0.005f;
	// pVisualizer->Normals(pMesh);

	// Create array pVisualizationData->AssociatedPts for storing associated points.

	RECOG::DDD::Model *pModel = models.Element;
	if (pVisualizationData->AssociatedPts.Element)
		RVL_DELETE_ARRAY(pVisualizationData->AssociatedPts.Element);
	pVisualizationData->AssociatedPts.n = pModel->points.n + 2 * pMesh->NodeArray.n;
	pVisualizationData->AssociatedPts.Element = new Point[pVisualizationData->AssociatedPts.n];

	// Copy query points to pVisualizationData->AssociatedPts.

	memcpy(pVisualizationData->AssociatedPts.Element + pModel->points.n, pMesh->NodeArray.Element, pMesh->NodeArray.n * sizeof(Point));
}

void DDDetector::SetBBoxForHypothesisVisualization(RECOG::DDD::Hypothesis *pHyp)
{
	RVLCOPY3VECTOR(pHyp->bboxCenter, pVisualizationData->bboxCenter);
	RVLCOPY3VECTOR(pHyp->bboxSize, pVisualizationData->bboxSize);
}

void DDDetector::RemoveHypothesisFromVisualization()
{
	if (pVisualizationData->bVisualizeModelPts)
	{
		pVisualizationData->pVisualizer->renderer->RemoveViewProp(pVisualizationData->modelPtActor);
		if (pVisualizationData->bVisualizePtAssociation)
			pVisualizationData->pVisualizer->renderer->RemoveViewProp(pVisualizationData->associationLinesActor);
	}
	else
		pVisualizationData->pVisualizer->renderer->RemoveViewProp(pVisualizationData->bboxActor);
}

void DDDetector::RemoveHypothesisBoundingBoxFromVisualization()
{
	pVisualizationData->pVisualizer->renderer->RemoveViewProp(pVisualizationData->bboxActor);
}

#define RVLDDDFIT3DTO2D_VERBOSE
#define RVLDDDFIT3DTO2D_VISUALIZE_ICP

void DDDetector::Fit3DTo2D(
	Mesh *pMesh,
	Array<RECOG::DDD::EdgeSample> edgeSamplePts,
	cv::Mat edges,
	float *QNMap,
	Pose3D poseMC0,
	Array<RECOG::DDD::PtAssoc> ptAssoc,
	int maxnIterations,
	float &th_,
	float &th,
	float &et,
	float *dR,
	float *dt,
	float maxdth,
	bool bDebug)
{
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
	cv::Mat displayImg;
	float *PRMem;
	float *PCMem;
	float *NCMem;
	RECOG::DDD::Edge *edge;
	if (pVisualizationData->bVisualizeICPSteps)
	{
		edge = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
		PRMem = new float[pMesh->NodeArray.n * 3];
		PCMem = new float[pMesh->NodeArray.n * 3];
		NCMem = new float[pMesh->faces.n * 3];
	}
#endif
	cv::Mat cva(6, 1, CV_64FC1);
	double *a_ = (double *)(cva.data);
	double *at_ = a_ + 3;
	cv::Mat cvM(6, 6, CV_64FC1);
	double *M_ = (double *)(cvM.data);
	cv::Mat cvx(6, 1, CV_64FC1);
	double *x_ = (double *)(cvx.data);
	double *xt_ = x_ + 3;
	cv::Mat cvc(6, 1, CV_64FC1);
	double *c_ = (double *)(cvc.data);
	int i;
	RECOG::DDD::PtAssoc *pPtAssoc;
	int iQPix;
	double e;
	RECOG::DDD::EdgeSample *pSamplePt;
	float *QN;
	float PR[3], MP[3];
	float Qu, Qv;
	float QP[3];
	RVLNULL3VECTOR(QP);
	float V3Tmp[3];
	// float maxe = 0.0f;
	RVLUNITMX3(dR);
	RVLNULL3VECTOR(dt);
	float dR0[9], dR0Prev[9];
	float dt0[3], dt0Prev[3];
	float E0;
	float EPrev = 0.0f;
	float rotAxis[3];
	float dE, meande;
	// int nStepReductions;
	for (int iIteration = 0; iIteration < maxnIterations; iIteration++)
	{
		RVLCOPYMX3X3(dR, dR0);
		RVLCOPY3VECTOR(dt, dt0);
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
		if (pVisualizationData->bVisualizeICPSteps)
		{
			Pose3D displayPose;
			displayPose = poseMC0;
			RVLMXMUL3X3(dR0, poseMC0.R, displayPose.R);
			RVLSUM3VECTORS(poseMC0.t, dt0, displayPose.t);
			Project3DModelToImage(pMesh, displayPose, edge, PRMem, PCMem, NCMem);
			cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
			Visualize3DModelImageProjection(displayImg, pMesh, edge);
		}
#endif
		memset(M_, 0, 6 * 6 * sizeof(double));
		memset(c_, 0, 6 * sizeof(double));
		E0 = 0.0f; // Only for debugging purpose!!!
		for (i = 0; i < ptAssoc.n; i++)
		{
			pPtAssoc = ptAssoc.Element + i;
			pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
			iQPix = pPtAssoc->iQPix;
			QN = QNMap + 3 * iQPix;
			RVLMULMX3X3VECT(dR0, pSamplePt->PR, PR);
			RVLCROSSPRODUCT3(PR, QN, a_);
			RVLCOPY3VECTOR(QN, at_);
			cvM += cva * cva.t();
			RVLSUM3VECTORS(PR, poseMC0.t, MP);
			RVLSUM3VECTORS(MP, dt0, MP);
			RVLDIF3VECTORS(QP, MP, V3Tmp);
			e = RVLDOTPRODUCT3(QN, V3Tmp);
			// if (RVLABS(e) > maxe)
			//	maxe = RVLABS(e);
			cvc += e * cva;
			E0 += (e * e); // Only for debugging purpose!!!
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
			if (pVisualizationData->bVisualizeICPSteps)
			{
				float fTmp = RVLDOTPRODUCT3(QN, QP);
				int Mu = (int)round(camera.fu * MP[0] / MP[2] + camera.uc);
				int Mv = (int)round(camera.fv * MP[1] / MP[2] + camera.vc);
				RVLSCALE3VECTOR(QN, e, V3Tmp);
				float QP_[3];
				RVLSUM3VECTORS(MP, V3Tmp, QP_);
				int Qu_ = (int)round(camera.fu * QP_[0] / QP_[2] + camera.uc);
				int Qv_ = (int)round(camera.fv * QP_[1] / QP_[2] + camera.vc);
				cv::line(displayImg, cv::Point(Mu, Mv), cv::Point(Qu_, Qv_), cv::Scalar(0, 0, 255));
			}
#endif
		}
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
		if (pVisualizationData->bVisualizeICPSteps)
		{
			cv::imshow("point association", displayImg);
			cv::waitKey();
		}
#endif
		dE = E0 - EPrev;
		if (dE <= 0.0f || iIteration == 0)
		{
			if (iIteration > 0)
			{
				meande = sqrt(EPrev / (float)(ptAssoc.n)) - sqrt(E0 / (float)(ptAssoc.n));
				if (meande < 1e-4)
					break;
			}
#ifdef RVLDDDFIT3DTO2D_VERBOSE
			printf("E=%f ", E0);
			if (iIteration > 0)
				printf("meande=%f", meande);
			printf("\n");
#endif
			EPrev = E0;
			RVLCOPYMX3X3(dR0, dR0Prev);
			RVLCOPY3VECTOR(dt0, dt0Prev);
			for (i = 0; i < 6; i++)
				M_[7 * i] += (i < 3 ? fit3DTo2DLambdaR : fit3DTo2DLambdat);
			cv::solve(cvM, cvc, cvx);
			// nStepReductions = 0;
		}
		else
		{
			cvx = 0.5 * cvx;
			RVLCOPYMX3X3(dR0Prev, dR0);
			RVLCOPY3VECTOR(dt0Prev, dt0);
			iIteration--;
			// nStepReductions++;
		}

		th_ = sqrt(RVLDOTPRODUCT3(x_, x_));
		RVLSCALE3VECTOR2(x_, th_, rotAxis);

		// if (th_ > 30.0 * DEG2RAD)
		//{
		//	s = 30 * DEG2RAD / th_;
		//	th_ = 30.0 * DEG2RAD;
		//	RVLSCALE3VECTOR(xt, s, xt);
		// }

		// When debugging with linear approximate rotation, comment the following 5 lines.

		if (th_ > 1e-4)
		{
			if (th_ > maxdth)
			{
				cvx = (cvx * maxdth / th_);
				th_ = maxdth;
			}
			float dR_[9];
			AngleAxisToRot<float>(rotAxis, th_, dR_);
			float M3x3Tmp[9];
			RVLMXMUL3X3(dR_, dR0, M3x3Tmp);
			GetAngleAxis(M3x3Tmp, rotAxis, th);
			AngleAxisToRot<float>(rotAxis, th, dR);
		}
		else
		{
			RVLCOPYMX3X3(dR0, dR);
			GetAngleAxis(dR, rotAxis, th);
		}

		// Only for debugging purpose: linear approximate rotation.

		// RVLSKEW(V3Tmp, dR);
		// float I[9];
		// RVLUNITMX3(I);
		// RVLSUMMX3X3(I, dR, dR);
		// RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
		// th = th_;

		RVLSUM3VECTORS(dt0, xt_, dt);
		et = sqrt(RVLDOTPRODUCT3(dt, dt));
#ifdef RVLDDDFIT3DTO2D_VERBOSE
		printf("dth=%f th=%f dt=%f\n", th_ * RAD2DEG, th * RAD2DEG, et);
#endif
		///

		if (bDebug)
		{
			/// Only for debugging purpose!!!

			// Non-linear error.

			float E = 0.0f; // Only for debugging purpose!!!
			float PR0[3];
			for (i = 0; i < ptAssoc.n; i++)
			{
				pPtAssoc = ptAssoc.Element + i;
				pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
				iQPix = pPtAssoc->iQPix;
				QN = QNMap + 3 * iQPix;
				RVLMULMX3X3VECT(dR, pSamplePt->PR, PR);
				// RVLMULMX3X3VECT(dR0, pSamplePt->PR, PR0);
				// Qu = (float)(iQPix % camera.w);
				// Qv = (float)(iQPix / camera.w);
				// QP[0] = (Qu - camera.uc) / camera.fu;
				// QP[1] = (Qv - camera.vc) / camera.fv;
				RVLDIF3VECTORS(QP, PR, V3Tmp);
				RVLDIF3VECTORS(V3Tmp, poseMC0.t, V3Tmp);
				RVLDIF3VECTORS(V3Tmp, dt, V3Tmp);
				e = RVLDOTPRODUCT3(QN, V3Tmp);
				E += (e * e); // Only for debugging purpose!!!
			}
			float Ex = fit3DTo2DLambdaR * RVLDOTPRODUCT3(x_, x_) + fit3DTo2DLambdat * RVLDOTPRODUCT3(xt_, xt_);
			E += Ex;

			// Linear error correction.

			cv::Mat cvdeLin = cvx.t() * cvM * cvx - 2.0 * cvc.t() * cvx;

			// Print errors.

			printf("E0 = %f, NonLin. error corr. = %f, Lin. error corr. = %f\n", E0, E - E0, *(double *)(cvdeLin.data));

			cv::waitKey();
		}

		///
	}
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
	if (pVisualizationData->bVisualizeICPSteps)
	{
		delete[] edge;
		delete[] PRMem;
		delete[] PCMem;
		delete[] NCMem;
	}
#endif
}

#define RVLDDDFIT3DTO2D_VISUALIZE_PTASSOC
#define RVLDDDFIT3DTO2D_VISUALIZE_INITHYPS

void DDDetector::Fit3DTo2D(
	Mesh *pMesh,
	cv::Mat RGB,
	Array<Pose3D> initPosesMC,
	Pose3D &bestPoseMC)
{
	// Convert to graycsale.

	cv::Mat I;
	cv::cvtColor(RGB, I, cv::COLOR_BGR2GRAY);

	// Blur the image for better edge detection.

	cv::Mat IBlured;
	cv::GaussianBlur(I, IBlured, cv::Size(3, 3), 0);

	// Canny edge detection.

	cv::Mat edges;
	cv::Canny(IBlured, edges, 20, 400, 3, false);

	// Display canny edge detected image.

	// cv::imshow("Canny edge detection", edges);
	// cv::waitKey(0);

	// Diference of Gausian.

	cv::GaussianBlur(I, IBlured, cv::Size(7, 7), 0);
	cv::Mat sobelx, sobely, sobelxy;
	cv::Sobel(IBlured, sobelx, CV_64F, 1, 0, 5);
	cv::Sobel(IBlured, sobely, CV_64F, 0, 1, 5);
	int iPix;
	int nPix = RGB.cols * RGB.rows;
	float *QNMap = new float[3 * nPix];
	float *QN = QNMap;
	float *QImgNMap = new float[2 * nPix];
	float *QImgN = QImgNMap;
	double *IuMap = (double *)(sobelx.data);
	double *IvMap = (double *)(sobely.data);
	double *Iu = IuMap;
	double *Iv = IvMap;
	float dI;
	float QP[3], QV[3];
	QP[2] = 1.0f;
	QV[2] = 0.0f;
	float Qu, Qv;
	float fTmp;
	for (iPix = 0; iPix < nPix; iPix++, QN += 3, QImgN += 2, Iu++, Iv++)
	{
		if (!edges.data[iPix])
			continue;
		QImgN[0] = (float)(*Iu);
		QImgN[1] = (float)(*Iv);
		dI = sqrt(QImgN[0] * QImgN[0] + QImgN[1] * QImgN[1]);
		QImgN[0] /= dI;
		QImgN[1] /= dI;
		QV[0] = QImgN[1];
		QV[1] = -QImgN[0];
		Qu = (float)(iPix % RGB.cols);
		Qv = (float)(iPix / RGB.cols);
		QP[0] = (Qu - camera.uc) / camera.fu;
		QP[1] = (Qv - camera.vc) / camera.fv;
		RVLCROSSPRODUCT3(QV, QP, QN);
		RVLNORM3(QN, fTmp);
	}
	// double minSobel, maxSobel;
	// cv::minMaxLoc(sobelx, &minSobel, &maxSobel);
	// cv::imshow("sobelx", (sobelx - minSobel) / (maxSobel - minSobel));
	// cv::minMaxLoc(sobely, &minSobel, &maxSobel);
	// cv::imshow("sobely", (sobely - minSobel) / (maxSobel - minSobel));
	// cv::waitKey(0);

	// Compute EDT.

	CRVLEDT EDT;
	EDT.m_Flags |= RVLEDT_FLAG_EDGE_CONTOURS;
	RVLEDT_PIX_ARRAY EDTImage;
	EDTImage.Width = RGB.cols;
	EDTImage.Height = RGB.rows;
	EDTImage.pPix = new RVLEDT_PIX[nPix];
	RVLEDT_BUCKET_ENTRY *EDTBucketMem = new RVLEDT_BUCKET_ENTRY[4 * nPix];
	WORD iBucket;
	for (iBucket = 0; iBucket < 4; iBucket++)
		EDT.m_BucketPtrArray[iBucket] = EDTBucketMem + iBucket * nPix;
	EDT.Border(&EDTImage);
	EDT.CreateRTSqrtLUT();
	CRVLBuffer EDTBuff;
	EDTBuff.DataBuff = new void *[2 * nPix];
	EDTBuff.m_bOwnData = FALSE;
	// EDT.m_maxd2 = imageWidth * imageWidth + imageHeight * imageHeight;
	EDT.m_maxd2 = fit3DTo2DEDTMaxDist * fit3DTo2DEDTMaxDist;
	CRVLMem mem;
	mem.Create(nPix * sizeof(RVLPTRCHAIN_ELEMENT));
	CRVLMPtrChain boundary;
	boundary.m_pMem = &mem;
	int u, v;
	for (v = 1; v < RGB.rows - 1; v++)
		for (u = 1; u < RGB.cols - 1; u++)
		{
			iPix = u + v * RGB.cols;
			if (edges.data[iPix] > 0)
			{
				EDTImage.pPix[iPix].d2 = 0;
				boundary.Add((void *)(EDTImage.pPix + iPix));
			}
		}
	EDT.Apply(&boundary, NULL, &EDTImage, &EDTBuff);
	delete[] EDTBucketMem;
	delete[] EDTBuff.DataBuff;

	// Visualize EDT.

	cv::Mat EDTDisplayImage(RGB.rows, RGB.cols, CV_8UC1);
	uchar *pPix = EDTDisplayImage.data;
	RVLEDT_PIX *pEDTPix = EDTImage.pPix;
	float k = 255.0 / sqrt((float)(EDT.m_maxd2));
	for (iPix = 0; iPix < nPix; iPix++, pPix++, pEDTPix++)
		*pPix = (pEDTPix->d2 <= EDT.m_maxd2 ? (uchar)round(k * sqrt((float)(pEDTPix->d2))) : 255);
	// cv::imshow("EDT", EDTDisplayImage);
	// cv::waitKey();

	// Create edges and compute their lengths.

	RECOG::DDD::Edge *edge = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
	RECOG::DDD::Edge *pEdge_;
	MeshEdge *pEdge;
	int iEdge;
	Point *pPt, *pPt_;
	float V3Tmp[3];
	for (iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		pEdge = pMesh->EdgeArray.Element + iEdge;
		pPt = pMesh->NodeArray.Element + pEdge->iVertex[0];
		pPt_ = pMesh->NodeArray.Element + pEdge->iVertex[1];
		pEdge_ = edge + iEdge;
		RVLDIF3VECTORS(pPt_->P, pPt->P, V3Tmp);
		pEdge_->length = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
		pEdge_->iVertex[0] = pEdge->iVertex[0];
		pEdge_->iVertex[1] = pEdge->iVertex[1];
	}

	// Initialize visualization.

	uchar yellow[] = {0, 255, 255};
	cv::Mat displayImg;

	// Project model onto the image.

	float *PRMem = new float[pMesh->NodeArray.n * 3];
	float *PCMem = new float[pMesh->NodeArray.n * 3];
	float *NCMem = new float[pMesh->faces.n * 3];
	float PRMem_[6 * 3];

	// Visualize initial pose.

	////cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
	////cv::cvtColor(EDTDisplayImage, displayImg, cv::COLOR_GRAY2RGB);
	// displayImg = RGB.clone();
	// SuperposeBinaryImage(displayImg, edges.data, yellow);
	// Visualize3DModelImageProjection(displayImg, pMesh, edge);
	// cv::imshow("model", displayImg);
	// cv::waitKey();

	// Hypothesis generation loop.

	Array<RECOG::DDD::EdgeSample> edgeSamplePts;
	int nEdgeSamples = fit3DTo2DNoEdgeSamples;
	int iSample;
	RECOG::DDD::EdgeSample *pSamplePt;
	bool bHoughLines = true;
	RECOG::DDD::Edge *edge_ = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
	Array<RECOG::DDD::EdgeSample> edgeSamplePts_;
	edgeSamplePts_.Element = new RECOG::DDD::EdgeSample[fit3DTo2DNoEdgeSamples];
	edgeSamplePts_.n = nEdgeSamples;
	float maxOrientationCorrection = fit3DTo2DMaxOrientationCorrection * DEG2RAD;
	int nInitSolutionIterations = 3;
	int nTrials = 20;
	float minDist = 20.0f;
	float eThr = 0.2f;
	float eN = 30.0f;
	float rotStD = 0.125 * PI; // rad
	float tStD = 0.05;         // m
	float chamferThr = 3.0f;   // pix
	int i;
	float csNThr = cos(eN * DEG2RAD);
	float minDist2 = minDist * minDist;
	float chamferThr2 = chamferThr * chamferThr;
	int iIteration, iIteration2, j, iSample_, iQPix, iQPix_, iTrial;
	RECOG::DDD::PtAssoc *ptAssoc = new RECOG::DDD::PtAssoc[fit3DTo2DNoEdgeSamples];
	RECOG::DDD::PtAssoc bestPtAssoc;
	RECOG::DDD::PtAssoc *pPtAssoc, *pPtAssoc_;
	RECOG::DDD::EdgeSample *pSamplePt_;
	RECOG::DDD::Edge *pEdge__;
	float *QImgN_;
	float distM, eM, distQ, eQ, csN;
	float MdImgP[2];
	float QImgNa[2];
	int QImgP[2], QImgP_[2];
	float QdImgP[2];
	float Q[36], R[36];
	memset(R, 0, 36 * sizeof(float));
	float *q, *qt, *q_, *qt_, *r, *PR;
	float x[6], y[6];
	float *xt = x + 3;
	float r_;
	float a[6], b[6];
	float *at = a + 3;
	float *bt = b + 3;
	float bestb[6], bestr[6];
	float *bestbt = bestb + 3;
	float e[6];
	float dR[9], dR0[9], dR_[9], M3x3Tmp[9];
	float th, th_, et;
	float rotAxis[3], dt[3], dt0[3];
	Pose3D poseMC_;
	bestPoseMC = initPosesMC.Element[0];
	float bestPosedR[9];
	float bestPosedt[3];
	float bestScore = 0.0f;
	int iBestHypothesis = -1;
	int nBestNumFPs = fit3DTo2DNoEdgeSamples;
	float signN;
	int nDefDoFs;
	cv::Mat cva(6, 1, CV_64FC1);
	double *a_ = (double *)(cva.data);
	double *at_ = a_ + 3;
	cv::Mat cvM(6, 6, CV_64FC1);
	double *M_ = (double *)(cvM.data);
	cv::Mat cve(6, 1, CV_64FC1);
	double *e_ = (double *)(cve.data);
	cv::Mat cvx(6, 1, CV_64FC1);
	double *x_ = (double *)(cvx.data);
	cv::Mat cvA(6, 6, CV_64FC1);
	double *A_ = (double *)(cvA.data);
	cv::Mat cvc(6, 1, CV_64FC1);
	double *c_ = (double *)(cvc.data);
	Array<RECOG::DDD::PtAssoc> ptAssocs;
	ptAssocs.Element = ptAssoc;
	std::vector<int> fifthAssocPtsHyps;
	std::vector<int> fifthMIdxs[2];
	bool bVisualizeICPSteps = pVisualizationData->bVisualizeICPSteps;
	int iHypothesis = 0;
	Camera camera_;
	for (int iInitPose = 0; iInitPose < initPosesMC.n; iInitPose++)
	{
		Pose3D poseMC = initPosesMC.Element[iInitPose];
		Project3DModelToImage(pMesh, poseMC, edge, PRMem, PCMem, NCMem);
		SampleEdges(pMesh, edge, edgeSamplePts);

		if (bHoughLines)
		{
			nDefDoFs = 5;
			// RVL::DDDetector::HoughLinesPtAssociationCylinder(edges, &ptA0ssoc[0], pMesh, edgeSamplePts, edge, bHoughLinesVis);
			// RVL::DDDetector::HoughLinesPtAssociationLHTCP(edges, &ptAssoc[0], QNMap, pMesh, edgeSamplePts, edge, fifthAssocPtsHyps, &fifthMIdxs[0], pVisualizationData->bVisualizeHoughTransform);
			RVL::DDDetector::HoughLinesPtAssociationLHTCP(
				edges,
				QNMap,
				IuMap,
				IvMap,
				pMesh,
				edge,
				camera_,
				pVisualizationData->bVisualizeHoughTransform,
				edgeSamplePts,
				fifthAssocPtsHyps,
				&ptAssoc[0],
				fifthMIdxs[0]);
			// printf("num. 5. assocs. = %d\n", fifthAssocPtsHyps.size());
		}
		// for (iIteration = 0; iIteration < nRANSACIterations; iIteration++)
		int nHypGenIterations = 2 * fifthAssocPtsHyps.size();
		for (iIteration = 0; iIteration < nHypGenIterations; iIteration++)
		{
			// if (iIteration == 15)
			//	int debug = 0;
			if (bHoughLines)
			{
				// int idx = rand() % fifthAssocPtsHyps.size();
				int idx = iIteration / 2;
				ptAssoc[4].iQPix = fifthAssocPtsHyps[idx];
				// printf("5. assoc: idx=%d, coordinates=(%d, %d)\n", idx, ptAssoc[4].iQPix % 640, ptAssoc[4].iQPix / 640);
				// ptAssoc[4].iMSample = fifthMIdxs[rand() % 2];
				ptAssoc[4].iMSample = fifthMIdxs[0][iIteration % 2];
			}
			else
			{
				for (i = 0; i < 6; i++)
				{
					// if (i == 5)
					//	int debug = 0;
					memset(bestr, 0, 6 * sizeof(float));
					for (iTrial = 0; iTrial < nTrials; iTrial++)
					{
						// if (i == 2 && iTrial == 3)
						//	int debug = 0;
						iSample = rand() % edgeSamplePts.n;
						pSamplePt = edgeSamplePts.Element + iSample;
						pEdge_ = edge + pSamplePt->edgeIdx;
						if (EDTImage.pPix[pSamplePt->iPix].d2 > EDT.m_maxd2)
							continue;
						QImgP[0] = pSamplePt->iImgP[0] - EDTImage.pPix[pSamplePt->iPix].dx;
						QImgP[1] = pSamplePt->iImgP[1] - EDTImage.pPix[pSamplePt->iPix].dz;
						iQPix = QImgP[0] + QImgP[1] * RGB.cols;
						QImgN = QImgNMap + 2 * iQPix;
						csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
						if (csN >= csNThr)
							signN = 1.0f;
						else if (-csN >= csNThr)
							signN = -1.0f;
						else
							continue;
						QImgNa[0] = signN * QImgN[0];
						QImgNa[1] = signN * QImgN[1];
						for (j = 0; j < i; j++)
						{
							pPtAssoc_ = ptAssoc + j;
							iSample_ = pPtAssoc_->iMSample;
							pSamplePt_ = edgeSamplePts.Element + iSample_;
							pEdge__ = edge + pSamplePt_->edgeIdx;
							iQPix_ = pPtAssoc_->iQPix;
							QImgP_[0] = iQPix_ % RGB.cols;
							QImgP_[1] = iQPix_ / RGB.cols;
							QImgN_ = QImgNMap + 2 * iQPix_;
							MdImgP[0] = pSamplePt->ImgP[0] - pSamplePt_->ImgP[0];
							MdImgP[1] = pSamplePt->ImgP[1] - pSamplePt_->ImgP[1];
							distM = MdImgP[0] * MdImgP[0] + MdImgP[1] * MdImgP[1];
							if (distM < minDist2)
								break;
							distM = sqrt(distM);
							MdImgP[0] /= distM;
							MdImgP[1] /= distM;
							eM = MdImgP[0] * pEdge__->ImgN[0] + MdImgP[1] * pEdge__->ImgN[0];
							QdImgP[0] = (float)(QImgP[0] - QImgP_[0]);
							QdImgP[1] = (float)(QImgP[1] - QImgP_[1]);
							distQ = QdImgP[0] * QdImgP[0] + QdImgP[1] * QdImgP[1];
							if (distQ < minDist2)
								break;
							distQ = sqrt(distQ);
							QdImgP[0] /= distQ;
							QdImgP[1] /= distQ;
							eQ = QdImgP[0] * pPtAssoc_->QImgN[0] + QdImgP[1] * pPtAssoc_->QImgN[1];
							if (RVLABS(eM) <= eThr)
							{
								if (RVLABS(eQ) > eThr)
									break;
							}
							// else if (RVLABS(eQ) <= eThr)
							//	break;
							else if (eM * eQ < 0.0f)
								break;
						}
						// if (j < i)
						//	continue;
						QN = QNMap + 3 * iQPix;
						RVLCROSSPRODUCT3(pSamplePt->PR, QN, a);
						RVLSCALE3VECTOR(a, rotStD, a);
						RVLSCALE3VECTOR(QN, tStD, at);
						RVLCOPY3VECTOR(a, b);
						RVLCOPY3VECTOR(at, bt);
						r = R + 6 * i;
						for (j = 0; j < i; j++)
						{
							q_ = Q + 6 * j;
							qt_ = q_ + 3;
							r_ = RVLDOTPRODUCT3(q_, b) + RVLDOTPRODUCT3(qt_, bt);
							RVLSCALE3VECTOR(q_, r_, V3Tmp);
							RVLDIF3VECTORS(b, V3Tmp, b);
							RVLSCALE3VECTOR(qt_, r_, V3Tmp);
							RVLDIF3VECTORS(bt, V3Tmp, bt);
							r[j] = r_;
						}
						r_ = sqrt(RVLDOTPRODUCT3(b, b) + RVLDOTPRODUCT3(bt, bt));
						if (r_ > bestr[i])
						{
							bestPtAssoc.iMSample = iSample;
							bestPtAssoc.iQPix = iQPix;
							bestPtAssoc.QImgN[0] = QImgNa[0];
							bestPtAssoc.QImgN[1] = QImgNa[1];
							RVLCOPY3VECTOR(b, bestb);
							RVLCOPY3VECTOR(bt, bestbt);
							memcpy(bestr, r, i * sizeof(float));
							bestr[i] = r_;
						}
					}
					if (bestr[i] < 0.001f)
						break;
					ptAssoc[i] = bestPtAssoc;
					q = Q + 6 * i;
					qt = q + 3;
					RVLSCALE3VECTOR2(bestb, bestr[i], q);
					RVLSCALE3VECTOR2(bestbt, bestr[i], qt);
					memcpy(r, bestr, (i + 1) * sizeof(float));
					// Only for debugging purpose!!!
					// if (iIteration == 483)
					//{
					// float a_[6];
					// float* at_ = a_ + 3;
					// RVLNULL3VECTOR(a_);
					// RVLNULL3VECTOR(at_);
					// for (j = 0; j <= i; j++)
					//{
					//	q = Q + 6 * j;
					//	RVLSCALE3VECTOR(q, r[j], V3Tmp);
					//	RVLSUM3VECTORS(a_, V3Tmp, a_);
					//	qt = q + 3;
					//	RVLSCALE3VECTOR(qt, r[j], V3Tmp);
					//	RVLSUM3VECTORS(at_, V3Tmp, at_);
					//}
					// printf("a [%d]=( ");
					// for (j = 0; j < 6; j++)
					//	printf("%f ", a[j]);
					// printf(")\n");
					// printf("a_[%d]=( ", i);
					// for (j = 0; j < 6; j++)
					//	printf("%f ", a_[j]);
					// printf(")\n");
					// pSamplePt = edgeSamplePts.Element + ptAssoc[i].iMSample;
					// QN = QNMap + 3 * ptAssoc[i].iQPix;
					// RVLCROSSPRODUCT3(pSamplePt->PR, QN, a_);
					// RVLSCALE3VECTOR(a_, rotStD, a_);
					// RVLSCALE3VECTOR(QN, tStD, at_);
					// printf("a'[%d]=( ", i);
					// for (j = 0; j < 6; j++)
					//	printf("%f ", a_[j]);
					// printf(")\n");
					//}
					//
				}
				nDefDoFs = i;
			}
			// if (i < fit3DTo2DMinDefDoFs)
			//	continue;
#ifdef RVLDDDFIT3DTO2D_VERBOSE
			printf("iteration %d\n", iIteration);
#endif

#ifdef RVLDDDFIT3DTO2D_VISUALIZE_PTASSOC
			// Visualize point association.
			// if (iIteration == 15)
			{
				// for (i = 1; i <= nDefDoFs; i++)
				//{
				cv::cvtColor(EDTDisplayImage, displayImg, cv::COLOR_GRAY2RGB);
				SuperposeBinaryImage(displayImg, edges.data, yellow);
				VisualizeEdgeSamples(displayImg, edgeSamplePts);
				ptAssocs.n = nDefDoFs;
				// ptAssocs.n = i;
				Visualize2DPointAssociation(displayImg, ptAssocs, edgeSamplePts.Element);
				cv::imshow("model", displayImg);
				cv::waitKey();
				//}
			}
#endif

			/// Correct pose.

			for (i = 0; i < nDefDoFs; i++)
			{
				pPtAssoc = ptAssoc + i;
				pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
				PR = PRMem_ + 3 * i;
				RVLCOPY3VECTOR(pSamplePt->PR, PR);
			}

			// if(false)
			// if (iIteration == 2046 || iIteration == 2444 || iIteration == 2616 || iIteration == 3113 || iIteration == 3477 || iIteration == 3658 || iIteration == 3974 || iIteration == 4183 || iIteration == 4255)
			// if (iIteration == 281 || iIteration == 673 || iIteration == 1247 || iIteration == 1663 || iIteration == 2444 || iIteration == 2616 || iIteration == 3113 || iIteration == 3477 || iIteration == 3658 || iIteration == 3974 || iIteration == 4183 || iIteration == 4255)
			{
#ifdef NEVER
				for (iIteration2 = 0; iIteration2 < nInitSolutionIterations; iIteration2++)
				{
					if (iIteration2 > 0)
					{
						/// Solving 6 equations with 6 unknowns.

						// QR decomposition of A: A = R * Q

						for (i = 0; i < nDefDoFs; i++)
						{
							pPtAssoc = ptAssoc + i;
							pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
							iQPix = pPtAssoc->iQPix;
							QN = QNMap + 3 * iQPix;
							PR = PRMem_ + 3 * i;
							RVLMULMX3X3VECT(dR, pSamplePt->PR, PR);
							RVLCROSSPRODUCT3(PR, QN, a);
							RVLSCALE3VECTOR(a, rotStD, a);
							RVLSCALE3VECTOR(QN, tStD, at);
							RVLCOPY3VECTOR(a, b);
							RVLCOPY3VECTOR(at, bt);
							r = R + 6 * i;
							for (j = 0; j < i; j++)
							{
								q_ = Q + 6 * j;
								qt_ = q_ + 3;
								r_ = RVLDOTPRODUCT3(q_, b) + RVLDOTPRODUCT3(qt_, bt);
								RVLSCALE3VECTOR(q_, r_, V3Tmp);
								RVLDIF3VECTORS(b, V3Tmp, b);
								RVLSCALE3VECTOR(qt_, r_, V3Tmp);
								RVLDIF3VECTORS(bt, V3Tmp, bt);
								r[j] = r_;
							}
							r_ = sqrt(RVLDOTPRODUCT3(b, b) + RVLDOTPRODUCT3(bt, bt));
							q = Q + 6 * i;
							qt = q + 3;
							RVLSCALE3VECTOR2(b, r_, q);
							RVLSCALE3VECTOR2(bt, r_, qt);
							r[i] = r_;
						}
					}

					// Compute e and solve A * x = e.

					memset(e, 0, 6 * sizeof(float));
					for (i = 0; i < nDefDoFs; i++)
					{
						pPtAssoc = ptAssoc + i;
						iQPix = pPtAssoc->iQPix;
						Qu = (float)(iQPix % RGB.cols);
						Qv = (float)(iQPix / RGB.cols);
						QP[0] = (Qu - camera.uc) / camera.fu;
						QP[1] = (Qv - camera.vc) / camera.fv;
						PR = PRMem_ + 3 * i;
						RVLDIF3VECTORS(QP, PR, V3Tmp);
						RVLDIF3VECTORS(V3Tmp, poseMC.t, V3Tmp);
						QN = QNMap + 3 * iQPix;
						e[i] = RVLDOTPRODUCT3(QN, V3Tmp);
						y[i] = e[i];
						r = R + 6 * i;
						for (j = 0; j < i; j++)
							y[i] -= r[j] * y[j];
						y[i] /= r[i];
					}
					RVLMULMXTVECT(Q, y, nDefDoFs, 6, x, i, j, q);

					///
				}
#endif
				ptAssocs.n = nDefDoFs;
				pVisualizationData->bVisualizeICPSteps = false;
				Fit3DTo2D(pMesh, edgeSamplePts, edges, QNMap, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
				RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
				RVLSUM3VECTORS(poseMC.t, dt, poseMC_.t);

#ifdef RVLDDDFIT3DTO2D_VISUALIZE_INITHYPS
				// Visualize corrected pose.

				// if (iIteration == 15)
				{
					Project3DModelToImage(pMesh, poseMC_, edge_, PRMem, PCMem, NCMem);
					cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
					Visualize3DModelImageProjection(displayImg, pMesh, edge_);
					cv::imshow("model", displayImg);
					cv::waitKey();
				}
#endif
				if (th > maxOrientationCorrection || et > fit3DTo2DMaxPositionCorrection)
				{
#ifdef RVLDDDFIT3DTO2D_VERBOSE
					printf("Hypothesis rejected.\n\n");
#endif
					continue;
				}

				///

				// Evaluate hypothesis.

				float score = 0.0f;
				int nFPs = 0;
				float chamferDist2;
				for (iSample = 0; iSample < edgeSamplePts.n; iSample++)
				{
					pSamplePt = edgeSamplePts.Element + iSample;
					pSamplePt_ = edgeSamplePts_.Element + iSample;
					RVLTRANSF3(pSamplePt->PR, dR, poseMC_.t, pSamplePt_->PC);
					pSamplePt_->iImgP[0] = (int)round(camera.fu * pSamplePt_->PC[0] / pSamplePt_->PC[2] + camera.uc);
					pSamplePt_->iImgP[1] = (int)round(camera.fv * pSamplePt_->PC[1] / pSamplePt_->PC[2] + camera.vc);
					if (pSamplePt_->iImgP[0] < 0 || pSamplePt_->iImgP[0] >= RGB.cols || pSamplePt_->iImgP[1] < 0 || pSamplePt_->iImgP[1] >= RGB.rows)
					{
						nFPs++;
						continue;
					}
					pSamplePt_->iPix = (int)round(pSamplePt_->iImgP[0]) + (int)round(pSamplePt_->iImgP[1]) * RGB.cols;
					chamferDist2 = (float)EDTImage.pPix[pSamplePt_->iPix].d2;
					if (chamferDist2 > chamferThr2)
					{
						nFPs++;
						continue;
					}
					// pEdge_ = edge + pSamplePt->edgeIdx;
					// QImgP[0] = pSamplePt->iImgP[0] - EDTImage.pPix[pSamplePt->iPix].dx;
					// QImgP[1] = pSamplePt->iImgP[1] - EDTImage.pPix[pSamplePt->iPix].dz;
					// iQPix = QImgP[0] + QImgP[1] * RGB.cols;
					// QImgN = QImgNMap + 2 * iQPix;
					// csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
					// if (RVLABS(csN) >= csNThr)
					score += exp(-chamferDist2 / chamferThr2);
					// else
					//	nFPs++;
				}
#ifdef RVLDDDFIT3DTO2D_VERBOSE
				printf("scene fitting score = %f no. FPs = %d\n\n", score, nFPs);
#endif
				// Record the best hypothesis.

				if (score > bestScore)
				{
					bestScore = score;
					bestPoseMC = poseMC_;
					RVLCOPYMX3X3(dR, bestPosedR);
					RVLCOPY3VECTOR(dt, bestPosedt);
					nBestNumFPs = nFPs;
					iBestHypothesis = iHypothesis;
				}

				// Visualize hypothesis.

				// cv::cvtColor(EDTDisplayImage, displayImg, cv::COLOR_GRAY2RGB);
				// SuperposeBinaryImage(displayImg, edges.data, yellow);
				// VisualizeEdgeSamples(displayImg, edgeSamplePts_);
				// cv::imshow("model", displayImg);
				// cv::waitKey();
			}
			iHypothesis++;
		}
	}

	/// ICP fitting.

	int nICPIterations = 5;
	float ICPThr = 10.0; // pix

	float ICPThr2 = ICPThr * ICPThr;
	RVLCOPYMX3X3(bestPosedR, dR);
	Pose3D poseMC = bestPoseMC;

	// Visualize initial pose.

	// Project3DModelToImage(pMesh, poseMC, edge_, PRMem, PCMem, NCMem);
	// cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
	// Visualize3DModelImageProjection(displayImg, pMesh, edge_);
	// cv::imshow("model", displayImg);
	// cv::waitKey();

	//

	float chamferDist2;
	for (iIteration = 0; iIteration < nICPIterations; iIteration++)
	{
		Project3DModelToImage(pMesh, poseMC, edge, PRMem, PCMem, NCMem);
		SampleEdges(pMesh, edge, edgeSamplePts);
		ptAssocs.n = 0;
		for (iSample = 0; iSample < edgeSamplePts.n; iSample++)
		{
			pSamplePt = edgeSamplePts.Element + iSample;
			pSamplePt_ = pSamplePt;
			// pSamplePt_ = edgeSamplePts_.Element + iSample;
			// RVLTRANSF3(pSamplePt->PR, dR, poseMC_.t, pSamplePt_->PC);
			// pSamplePt_->iImgP[0] = (int)round(camera.fu * pSamplePt_->PC[0] / pSamplePt_->PC[2] + camera.uc);
			// pSamplePt_->iImgP[1] = (int)round(camera.fv * pSamplePt_->PC[1] / pSamplePt_->PC[2] + camera.vc);
			// if (pSamplePt_->iImgP[0] < 0 || pSamplePt_->iImgP[0] >= RGB.cols || pSamplePt_->iImgP[1] < 0 || pSamplePt_->iImgP[1] >= RGB.rows)
			//	continue;
			// pSamplePt_->iPix = (int)round(pSamplePt_->iImgP[0]) + (int)round(pSamplePt_->iImgP[1]) * camera.w;
			chamferDist2 = (float)EDTImage.pPix[pSamplePt_->iPix].d2;
			if (chamferDist2 > ICPThr2)
				continue;
			QImgP[0] = pSamplePt_->iImgP[0] - EDTImage.pPix[pSamplePt_->iPix].dx;
			QImgP[1] = pSamplePt_->iImgP[1] - EDTImage.pPix[pSamplePt_->iPix].dz;
			iQPix = QImgP[0] + QImgP[1] * camera.w;
			if (edges.data[iQPix] == 0)
				continue;
			QImgN = QImgNMap + 2 * iQPix;
			pEdge_ = edge + pSamplePt->edgeIdx;
			csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
			if (RVLABS(csN) < csNThr)
				continue;
			pPtAssoc = ptAssoc + ptAssocs.n++;
			pPtAssoc->iMSample = iSample;
			pPtAssoc->iQPix = iQPix;
			pPtAssoc->QImgN[0] = QImgN[0];
			pPtAssoc->QImgN[1] = QImgN[1];
		}
		pVisualizationData->bVisualizeICPSteps = bVisualizeICPSteps;
		Fit3DTo2D(pMesh, edgeSamplePts, edges, QNMap, poseMC, ptAssocs, 20, th_, th, et, dR, dt, 0.1f);
		RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
		RVLSUM3VECTORS(poseMC.t, dt, poseMC_.t);
		poseMC = poseMC_;

		// Visualize point association.

		// cv::cvtColor(EDTDisplayImage, displayImg, cv::COLOR_GRAY2RGB);
		// SuperposeBinaryImage(displayImg, edges.data, yellow);
		////VisualizeEdgeSamples(displayImg, edgeSamplePts);
		// Visualize2DPointAssociation(displayImg, ptAssocs, edgeSamplePts.Element);
		// cv::imshow("model", displayImg);
		// cv::waitKey();

		//

		// Project3DModelToImage(pMesh, poseMC, edge_, PRMem, PCMem, NCMem);
		// cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
		// Visualize3DModelImageProjection(displayImg, pMesh, edge_);
		// cv::imshow("model", displayImg);
		// cv::waitKey();
	}

	///

	// Visualize final solution.

	uchar red[3] = {255, 0, 0};
	printf("best scene fitting score = %f no. FPs = %d iteration %d\n\n", bestScore, nBestNumFPs, iBestHypothesis);
	displayImg = RGB.clone();
	Project3DModelToImage(pMesh, poseMC, edge_, PRMem, PCMem, NCMem);
	// cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
	Visualize3DModelImageProjection(displayImg, pMesh, edge_, red);
	cv::imshow("solution", displayImg);
	cv::waitKey();

	delete[] EDTImage.pPix;
	delete[] PRMem;
	delete[] PCMem;
	delete[] NCMem;
	delete[] edge;
	delete[] edge_;
	delete[] QNMap;
	delete[] QImgNMap;
	delete[] edgeSamplePts.Element;
	delete[] edgeSamplePts_.Element;
	delete[] ptAssoc;

	cv::destroyAllWindows();
}

void DDDetector::Project3DModelToImage(
	Mesh *pMesh,
	Pose3D poseMC,
	RECOG::DDD::Edge *edge,
	float *PRMem,
	float *PCMem,
	float *NCMem,
	int cameraNum)
{
	// Parameters.

	float minEdgeFaceAngle = 45.0f; // deg

	// Constants.

	float csMinEdgeFaceAngle = cos(minEdgeFaceAngle * DEG2RAD);

	// Transform model to the scene.

	int iVertex;
	float *PR;
	float *PC;
	Point *pPt;
	for (iVertex = 0; iVertex < pMesh->NodeArray.n; iVertex++)
	{
		pPt = pMesh->NodeArray.Element + iVertex;
		PR = PRMem + iVertex * 3;
		RVLMULMX3X3VECT(poseMC.R, pPt->P, PR);
		PC = PCMem + iVertex * 3;
		RVLSUM3VECTORS(PR, poseMC.t, PC);
	}
	float *NC;
	MESH::Face *pFace;
	int iFace;
	for (iFace = 0; iFace < pMesh->faces.n; iFace++)
	{
		pFace = pMesh->faces.Element[iFace];
		NC = NCMem + iFace * 3;
		RVLMULMX3X3VECT(poseMC.R, pFace->N, NC);
	}

	// Project edges onto the image.

	int i, iEdge;
	float *NC_;
	cv::Point2f dImgP;
	float ImgEdgeLen;
	MeshEdge *pEdge;
	RECOG::DDD::Edge *pEdge_;
	for (iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		pEdge = pMesh->EdgeArray.Element + iEdge;
		pEdge_ = edge + iEdge;
		NC = NCMem + pEdge->pFace[0]->idx * 3;
		NC_ = NCMem + pEdge->pFace[1]->idx * 3;
		if (NC[2] >= 0.0f && NC_[2] >= 0.0f)
		{
			pEdge_->bVisible = false;
			continue;
		}
		if (NC[2] * NC_[2] > 0.0f)
		{
			if (RVLDOTPRODUCT3(NC, NC_) > csMinEdgeFaceAngle)
			{
				pEdge_->bVisible = false;
				continue;
			}
		}
		pEdge_->bVisible = true;
		for (i = 0; i < 2; i++)
		{
			PR = PRMem + pEdge->iVertex[i] * 3;
			RVLCOPY3VECTOR(PR, pEdge_->PR[i]);
			PC = PCMem + pEdge->iVertex[i] * 3;
			RVLCOPY3VECTOR(PC, pEdge_->PC[i]);
			pEdge_->ImgP[i].x = (int)round(cameras[cameraNum].fu * PC[0] / PC[2] + cameras[cameraNum].uc);
			pEdge_->ImgP[i].y = (int)round(cameras[cameraNum].fv * PC[1] / PC[2] + cameras[cameraNum].vc);
		}
		RVLDIF3VECTORS(pEdge_->PR[1], pEdge_->PR[0], pEdge_->VR);
		RVLSCALE3VECTOR2(pEdge_->VR, pEdge_->length, pEdge_->VR);
		RVLDIF3VECTORS(pEdge_->PC[1], pEdge_->PC[0], pEdge_->VC);
		RVLSCALE3VECTOR2(pEdge_->VC, pEdge_->length, pEdge_->VC);
		dImgP = pEdge_->ImgP[1] - pEdge_->ImgP[0];
		ImgEdgeLen = sqrt(dImgP.x * dImgP.x + dImgP.y * dImgP.y);
		pEdge_->ImgN[0] = -dImgP.y / ImgEdgeLen;
		pEdge_->ImgN[1] = dImgP.x / ImgEdgeLen;
	}
}

void DDDetector::SampleEdges(
	Mesh *pMesh,
	RECOG::DDD::Edge *edge,
	Array<RECOG::DDD::EdgeSample> &edgeSamplePts)
{
	int nEdgeSamples = fit3DTo2DNoEdgeSamples;
	edgeSamplePts.Element = new RECOG::DDD::EdgeSample[nEdgeSamples];
	RECOG::DDD::EdgeSample *pSamplePt = edgeSamplePts.Element;
	float edgeBinStart = 0.0f;
	int iEdge;
	RECOG::DDD::Edge *pEdge_;
	for (iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		pEdge_ = edge + iEdge;
		if (pEdge_->bVisible)
		{
			pEdge_->binEnd = edgeBinStart + pEdge_->length;
			edgeBinStart = pEdge_->binEnd;
		}
		else
			pEdge_->binEnd = 0.0f;
	}
	float edgeSampleInc = edgeBinStart / (float)nEdgeSamples;
	float firsEdgeSample = 0.5f * edgeSampleInc;
	int iSample;
	float sample = firsEdgeSample;
	iEdge = 0;
	float s;
	edgeBinStart = 0.0f;
	float V3Tmp[3];
	for (iSample = 0; iSample < nEdgeSamples; iSample++, sample += edgeSampleInc)
	{
		while (sample > edge[iEdge].binEnd)
		{
			if (edge[iEdge].bVisible)
				edgeBinStart = edge[iEdge].binEnd;
			iEdge++;
		}
		pEdge_ = edge + iEdge;
		s = sample - edgeBinStart;
		RVLSCALE3VECTOR(pEdge_->VR, s, V3Tmp);
		RVLSUM3VECTORS(pEdge_->PR[0], V3Tmp, pSamplePt->PR);
		RVLSCALE3VECTOR(pEdge_->VC, s, V3Tmp);
		RVLSUM3VECTORS(pEdge_->PC[0], V3Tmp, pSamplePt->PC);
		pSamplePt->ImgP[0] = camera.fu * pSamplePt->PC[0] / pSamplePt->PC[2] + camera.uc;
		pSamplePt->ImgP[1] = camera.fv * pSamplePt->PC[1] / pSamplePt->PC[2] + camera.vc;
		pSamplePt->iImgP[0] = (int)round(pSamplePt->ImgP[0]);
		pSamplePt->iImgP[1] = (int)round(pSamplePt->ImgP[1]);
		if (pSamplePt->iImgP[0] >= 0 && pSamplePt->iImgP[0] < camera.w && pSamplePt->iImgP[1] >= 0 && pSamplePt->iImgP[1] < camera.h)
		{
			pSamplePt->iPix = pSamplePt->iImgP[0] + pSamplePt->iImgP[1] * camera.w;
			pSamplePt->edgeIdx = iEdge;
			pSamplePt++;
		}
	}
	edgeSamplePts.n = pSamplePt - edgeSamplePts.Element;
}

void DDDetector::BoundingBox(
	Mesh *pMesh,
	Array<int> surfels,
	float *RSB,
	Box<float> &bbox)
{
	float *PS = pMesh->NodeArray.Element[pSurfels->NodeArray.Element[surfels.Element[0]].PtList.pFirst->Idx].P;
	float PB[3];
	RVLMULMX3X3VECT(RSB, PS, PB);
	InitBoundingBox<float>(&bbox, PB);
	Surfel *pSurfel;
	QList<QLIST::Index2> *pPtList;
	QLIST::Index2 *pPtIdx;
	for (int k = 0; k < surfels.n; k++)
	{
		pSurfel = pSurfels->NodeArray.Element + surfels.Element[k];
		pPtList = &(pSurfel->PtList);
		pPtIdx = pPtList->pFirst;
		while (pPtIdx)
		{
			PS = pMesh->NodeArray.Element[pPtIdx->Idx].P;
			RVLMULMX3X3VECT(RSB, PS, PB);
			UpdateBoundingBox<float>(&bbox, PB);
			pPtIdx = pPtIdx->pNext;
		}
	}
}

void DDDetector::GenerateHypothesis(
	Box<float> bbox,
	RECOG::DDD::Model *pModel,
	float *RMB,
	int *bboxAxesIdxM,
	float *bboxAxesSignM,
	float *RBS,
	RECOG::DDD::Hypothesis &hyp)
{
	float bboxSizeB[3];
	BoxSize<float>(&bbox, bboxSizeB[0], bboxSizeB[1], bboxSizeB[2]);
	float tMB[3];
	for (int k = 0; k < 3; k++)
	{
		hyp.bboxSize[k] = bboxSizeB[bboxAxesIdxM[k]];
		hyp.pose.s[k] = hyp.bboxSize[k] / pModel->bboxSize[k];
		tMB[bboxAxesIdxM[k]] = -bboxAxesSignM[k] * hyp.pose.s[k] * pModel->bboxCenter[k];
	}
	float bboxCenterB0[3];
	BoxCenter<float>(&bbox, bboxCenterB0);
	float tBS[3];
	RVLMULMX3X3VECT(RBS, bboxCenterB0, tBS);
	RVLCOMPTRANSF3D(RBS, tBS, RMB, tMB, hyp.pose.R, hyp.pose.t);
	RVLINVTRANSL(RMB, tMB, hyp.bboxCenter);
}

void DDDetector::Project(
	RECOG::DDD::Model *pModel,
	int m,
	float *q,
	float *RMS,
	float *tMS)
{
	int i;
	for (i = 0; i < ZBufferActivePtArray.n; i++)
		ZBuffer.Element[ZBufferActivePtArray.Element[i]].bValid = false;
	ZBufferActivePtArray.n = 0;

	int w = ZBuffer.w;
	int h = ZBuffer.h;
	float fSceneSamplingResolution = (float)sceneSamplingResolution;
	float fu = camera.fu / fSceneSamplingResolution;
	float fv = camera.fv / fSceneSamplingResolution;
	float uc = camera.uc / fSceneSamplingResolution;
	float vc = camera.vc / fSceneSamplingResolution;
	float P[3], N[3], PM[3];
	RECOG::DDD::ModelPoint *pPtSrc;
	Point *pPtTgt;
	int u, v, iPix;
	float *mxRow;
	int i_, j_;
	for (i = 0; i < pModel->points_.n; i++)
	{
		pPtSrc = pModel->points_.Element + i;
		RVLMULMXVECT(pPtSrc->S, q, 3, m, PM, i_, j_, mxRow);
		RVLMULMX3X3VECT(RMS, PM, P);
		if (tMS)
		{
			RVLSUM3VECTORS(P, tMS, P);
		}
		RVLMULMX3X3VECT(RMS, pPtSrc->N, N);
		if (RVLDOTPRODUCT3(N, P) >= 0)
			continue;
		u = (int)(fu * P[0] / P[2] + uc + 0.5f);
		if (u < 0)
			continue;
		else if (u >= w)
			continue;
		v = (int)(fv * P[1] / P[2] + vc + 0.5f);
		if (v < 0)
			continue;
		else if (v >= h)
			continue;
		iPix = u + v * w;
		pPtTgt = ZBuffer.Element + iPix;
		if (pPtTgt->bValid)
		{
			if (P[2] < pPtTgt->P[2])
			{
				RVLCOPY3VECTOR(P, pPtTgt->P);
				RVLCOPY3VECTOR(N, pPtTgt->N);
			}
		}
		else
		{
			RVLCOPY3VECTOR(P, pPtTgt->P);
			RVLCOPY3VECTOR(N, pPtTgt->N);
			pPtTgt->bValid = true;
			ZBufferActivePtArray.Element[ZBufferActivePtArray.n++] = iPix;
		}
	}
}

void DDDetector::Clustering(
	float *data,
	int nData,
	int nDims,
	float thr,
	int *&clusterID,
	Array<Array<int>> &clusters,
	int *&clusterMem,
	float *EIn)
{
	// Compute Euclidean distance table.

	float *E;
	float *ERow;
	int i, j;
	if (EIn)
		E = EIn;
	else
	{
		E = new float[nData * nData];
		int i_;
		float *data1 = data;
		float *data2;
		float *diff = new float[nDims];
		float e;
		ERow = E;
		for (i = 0; i < nData; i++, ERow += nData, data1 += nDims)
		{
			data2 = data;
			for (j = 0; j < i; j++, data2 += nDims)
			{
				RVLDIFVECTORS(data1, data2, nDims, diff, i_);
				RVLDOTPRODUCT(diff, diff, nDims, e, i_);
				ERow[j] = E[j * nData + i] = e;
			}
			ERow[i] = 0.0f;
		}
		delete[] diff;
	}

	// Sort data pairs.

	int nDataPairs = nData * (nData - 1) / 2;
	std::vector<SortIndex<float>> sortedDataPairs(nDataPairs);
	Pair<int, int> *dataPair = new Pair<int, int>[nDataPairs];
	int iDataPair = 0;
	ERow = E;
	for (i = 0; i < nData; i++, ERow += nData)
	{
		for (j = 0; j < i; j++, iDataPair++)
		{
			dataPair[iDataPair].a = i;
			dataPair[iDataPair].b = j;
			sortedDataPairs[iDataPair].idx = iDataPair;
			sortedDataPairs[iDataPair].cost = ERow[j];
		}
	}
	std::sort(sortedDataPairs.begin(), sortedDataPairs.end(), RECOG::DDD::SortCompare);

	// Allocate cluster memory.

	if (clusterID == NULL)
		clusterID = new int[nData];
	if (clusterMem == NULL)
		clusterMem = new int[nData];
	QList<QLIST::Index> *elementListMem = new QList<QLIST::Index>[nData];
	QLIST::Index *elementMem = new QLIST::Index[nData];

	// Clustering.

	QList<QLIST::Index> *elementList;
	QLIST::Index *elementIdx = elementMem;
	for (i = 0; i < nData; i++, elementIdx++)
	{
		clusterID[i] = i;
		elementList = elementListMem + i;
		RVLQLIST_INIT(elementList);
		RVLQLIST_ADD_ENTRY(elementList, elementIdx);
		elementIdx->Idx = i;
	}
	int iCluster, iCluster_;
	Array<int> cluster;
	Pair<int, int> dataPair_;
	QList<QLIST::Index> *elementList_;
	QLIST::Index *elementIdx_;
	for (iDataPair = 0; iDataPair < nDataPairs; iDataPair++)
	{
		dataPair_ = dataPair[sortedDataPairs[iDataPair].idx];
		if (E[dataPair_.a * nData + dataPair_.b] > thr)
			break;
		iCluster = clusterID[dataPair_.a];
		iCluster_ = clusterID[dataPair_.b];
		if (iCluster == iCluster_)
			continue;
		elementList = elementListMem + iCluster;
		elementList_ = elementListMem + iCluster_;
		elementIdx = elementList->pFirst;
		while (elementIdx)
		{
			elementIdx_ = elementList_->pFirst;
			while (elementIdx_)
			{
				if (E[elementIdx->Idx * nData + elementIdx_->Idx] > thr)
					break;
				elementIdx_ = elementIdx_->pNext;
			}
			if (elementIdx_)
				break;
			elementIdx = elementIdx->pNext;
		}
		if (elementIdx)
			continue;
		elementIdx_ = elementList_->pFirst;
		while (elementIdx_)
		{
			clusterID[elementIdx_->Idx] = iCluster;
			elementIdx_ = elementIdx_->pNext;
		}
		RVLQLIST_APPEND(elementList, elementList_);
		RVLQLIST_INIT(elementList_);
	}
	if (EIn == NULL)
		delete[] E;
	delete[] dataPair;

	// Create clusters.

	clusters.n = 0;
	for (i = 0; i < nData; i++)
		if (elementListMem[i].pFirst)
			clusters.n++;
	if (clusters.Element == NULL)
		clusters.Element = new Array<int>[clusters.n];
	int *clusterMemPtr = clusterMem;
	iCluster = 0;
	for (i = 0; i < nData; i++)
	{
		elementList = elementListMem + i;
		if (elementList->pFirst)
		{
			clusters.Element[iCluster].Element = clusterMemPtr;
			QLIST::CopyToArray(elementList, clusters.Element + iCluster);
			for (j = 0; j < clusters.Element[iCluster].n; j++)
				clusterID[clusters.Element[iCluster].Element[j]] = iCluster;
			clusterMemPtr += clusters.Element[iCluster].n;
			iCluster++;
		}
	}
	delete[] elementListMem;
	delete[] elementMem;
}

void DDDetector::Vector3DClustering(
	Array<RECOG::DDD::AxisHypothesis> axisHyps,
	float thr,
	int *&clusterID,
	Array<Array<int>> &clusters,
	int *&clusterMem)
{
	float thr2 = thr * thr;
	float *E = new float[axisHyps.n * axisHyps.n];
	float cs, e;
	float *ERow = E;
	RECOG::DDD::AxisHypothesis *pHyp, *pHyp_;
	int i, j;
	for (i = 0; i < axisHyps.n; i++, ERow += axisHyps.n)
	{
		pHyp = axisHyps.Element + i;
		for (j = 0; j < i; j++)
		{
			pHyp_ = axisHyps.Element + j;
			cs = RVLDOTPRODUCT3(pHyp->axis, pHyp_->axis);
			e = 1.0 - RVLABS(cs);
			ERow[j] = E[j * axisHyps.n + i] = e;
		}
		ERow[i] = 0.0f;
	}
	Clustering(NULL, axisHyps.n, 3, thr2, clusterID, clusters, clusterMem, E);
	delete[] E;
}

int DDDetector::MinClusterSize(
	Array<Array<int>> clusters,
	int clusterSizeThrCoeff,
	int minClusterSize)
{
	int dominantClusterSize = 0;
	int i;
	for (i = 0; i < clusters.n; i++)
		if (clusters.Element[i].n > dominantClusterSize)
			dominantClusterSize = clusters.Element[i].n;
	int minZClusterSize_ = dominantClusterSize * clusterSizeThrCoeff / 100;
	return RVLMAX(minClusterSize, minZClusterSize_);
}

bool DDDetector::AlignHypothesisZAxis(
	RECOG::DDD::Hypothesis &hyp,
	float *ZRef,
	float csZThr)
{
	float *R = hyp.pose.R;
	float ZBC[3];
	RVLCOPYCOLMX3X3(R, 2, ZBC);
	float cs = RVLDOTPRODUCT3(ZBC, ZRef);
	float sn;
	bool bValid;
	float fTmp;
	float RX[9], Mx3x3Tmp[9];
	if (cs >= csZThr)
	{
		cs = 1.0f;
		sn = 0.0f;
		bValid = true;
	}
	else if (-cs >= csZThr)
	{
		cs = -1.0f;
		sn = 0.0f;
		bValid = true;
	}
	else
	{
		RVLCOPYCOLMX3X3(R, 1, ZBC);
		cs = RVLDOTPRODUCT3(ZBC, ZRef);
		if (cs >= csZThr)
		{
			cs = 0.0f;
			sn = -1.0f;
			bValid = true;
			fTmp = hyp.bboxSize[1];
			hyp.bboxSize[1] = hyp.bboxSize[2];
			hyp.bboxSize[2] = fTmp;
		}
		else if (-cs >= csZThr)
		{
			cs = 0.0f;
			sn = 1.0f;
			bValid = true;
			fTmp = hyp.bboxSize[1];
			hyp.bboxSize[1] = hyp.bboxSize[2];
			hyp.bboxSize[2] = fTmp;
		}
		else
			bValid = false;
	}
	if (bValid)
	{
		RVLROTX(cs, sn, RX);
		RVLCOPYMX3X3(R, Mx3x3Tmp);
		RVLMXMUL3X3(Mx3x3Tmp, RX, R);
	}
	return bValid;
}

void DDDetector::AOZeroState(
	RECOG::DDD::ArticulatedObject AObj,
	RECOG::DDD::AOHypothesisState *&stateMem)
{
	stateMem = new RECOG::DDD::AOHypothesisState[AObj.movingParts.n];
	for (int iMovingPart = 0; iMovingPart < AObj.movingParts.n; iMovingPart++)
	{
		RECOG::DDD::HypothesisDoorDrawer *pMovingPart = AObj.movingParts.Element + iMovingPart;
		pMovingPart->state.n = 1;
		pMovingPart->state.Element = stateMem + iMovingPart;
		pMovingPart->state.Element[0].q = 0.0f;
	}
}

void DDDetector::Visualize3DModelImageProjection(
	cv::Mat displayImg,
	Mesh *pModelMesh,
	RECOG::DDD::Edge *edge,
	uchar *colorIn)
{
	uchar colorDefault[3] = {0, 255, 0};
	uchar *color = (colorIn ? colorIn : colorDefault);
	for (int iEdge = 0; iEdge < pModelMesh->EdgeArray.n; iEdge++)
		if (edge[iEdge].bVisible)
			cv::line(displayImg, edge[iEdge].ImgP[0], edge[iEdge].ImgP[1], cv::Scalar(color[2], color[1], color[0]));
}

void DDDetector::SuperposeBinaryImage(
	cv::Mat displayImg,
	uchar *binImg,
	uchar *color)
{
	int nPix = displayImg.cols * displayImg.rows;
	uchar *pPix;
	for (int iPix = 0; iPix < nPix; iPix++)
		if (binImg[iPix])
		{
			pPix = displayImg.data + 3 * iPix;
			RVLCOPY3VECTOR(color, pPix);
		}
}

void DDDetector::VisualizeEdgeSamples(
	cv::Mat displayImg,
	Array<RECOG::DDD::EdgeSample> edgeSamplePt)
{
	RECOG::DDD::EdgeSample *pSamplePt;
	for (int iSample = 0; iSample < edgeSamplePt.n; iSample++)
	{
		pSamplePt = edgeSamplePt.Element + iSample;
		cv::circle(displayImg, cv::Point(pSamplePt->iImgP[0], pSamplePt->iImgP[1]), 3, cv::Scalar(0, 255, 0));
	}
}

void DDDetector::Visualize2DPointAssociation(
	cv::Mat displayImg,
	Array<RECOG::DDD::PtAssoc> ptAssoc,
	RECOG::DDD::EdgeSample *edgeSamplePt)
{
	int i, iSample, iQPix;
	int QImgP[2];
	RECOG::DDD::EdgeSample *pSamplePt;
	for (i = 0; i < ptAssoc.n; i++)
	{
		iSample = ptAssoc.Element[i].iMSample;
		pSamplePt = edgeSamplePt + iSample;
		iQPix = ptAssoc.Element[i].iQPix;
		QImgP[0] = iQPix % displayImg.cols;
		QImgP[1] = iQPix / displayImg.cols;
		cv::line(displayImg, cv::Point(pSamplePt->iImgP[0], pSamplePt->iImgP[1]), cv::Point(QImgP[0], QImgP[1]), cv::Scalar(0, 0, 255));
		cv::circle(displayImg, cv::Point(pSamplePt->ImgP[0], pSamplePt->ImgP[1]), 3, cv::Scalar(0, 0, 255));
	}
}

void DDDetector::VisualizeStorageVolumeModel(
	RECOG::DDD::Model *pModel,
	RECOG::DDD::HypothesisSV hyp,
	Array<Point> &samplePts,
	Array<Pair<int, int>> *pNormals)
{
	samplePts.Element = new Point[2 * pModel->points_.n];
	samplePts.n = 0;
	if (pNormals)
	{
		pNormals->Element = new Pair<int, int>[pModel->points_.n];
		pNormals->n = 0;
	}
	RECOG::DDD::ModelPoint *pMPt = pModel->points_.Element;
	int iMPt;
	Point *pPt;
	int i, j;
	float q[7];
	q[0] = storageVolumeWallThickness;
	memcpy(q + 1, hyp.s, 6 * sizeof(float));
	float *mxRow;
	float V3Tmp[3];
	float P[3], N[3];
	for (iMPt = 0; iMPt < pModel->points_.n; iMPt++, pMPt++)
	{
		pPt = samplePts.Element + samplePts.n;
		RVLMULMXVECT(pMPt->S, q, 3, 7, P, i, j, mxRow);
		RVLMULMX3X3VECT(hyp.RMS, P, pPt->P);
		if (pNormals)
		{
			RVLMULMX3X3VECT(hyp.RMS, pMPt->N, N);
			RVLSCALE3VECTOR(N, 0.01f, V3Tmp);
			RVLSUM3VECTORS(pPt->P, V3Tmp, V3Tmp);
			pPt = samplePts.Element + pModel->points_.n + samplePts.n;
			RVLCOPY3VECTOR(V3Tmp, pPt->P);
			pNormals->Element[pNormals->n].a = samplePts.n;
			pNormals->Element[pNormals->n].b = pModel->points_.n + samplePts.n;
			pNormals->n++;
		}
		samplePts.n++;
	}
}

void DDDetector::VisualizeSequenceHypothesis(Array<RECOG::DDD::Hypothesis> hyps)
{
	Array<Point> XAxisEndPts;
	XAxisEndPts.n = 2 * hyps.n;
	XAxisEndPts.Element = new Point[XAxisEndPts.n];
	Point *pPt = XAxisEndPts.Element;
	Array<Pair<int, int>> XAxes;
	XAxes.n = hyps.n;
	XAxes.Element = new Pair<int, int>[XAxes.n];
	int i;
	RECOG::DDD::Hypothesis *pHyp;
	Pose3D pose;
	float X[3], V3Tmp[3];
	for (i = 0; i < hyps.n; i++)
	{
		pHyp = hyps.Element + i;
		RVLCOPYMX3X3(pHyp->pose.R, pose.R);
		RVLCOPY3VECTOR(pHyp->pose.t, pose.t);
		pVisualizationData->pVisualizer->DisplayBox(pHyp->bboxSize[0], pHyp->bboxSize[1], pHyp->bboxSize[2], &pose, 255.0, 0.0, 0.0);
		RVLCOPY3VECTOR(pHyp->pose.t, pPt->P);
		pPt++;
		RVLCOPYCOLMX3X3(pHyp->pose.R, 0, X);
		RVLSCALE3VECTOR(X, 0.1f, V3Tmp);
		RVLSUM3VECTORS(pHyp->pose.t, V3Tmp, pPt->P);
		pPt++;
		XAxes.Element[i].a = 2 * i;
		XAxes.Element[i].b = 2 * i + 1;
	}
	uchar blue[] = {0, 0, 255};
	pVisualizationData->pVisualizer->DisplayLines(XAxisEndPts, XAxes, blue);
	delete[] XAxisEndPts.Element;
	delete[] XAxes.Element;
}

void DDDetector::VisualizeRectangularStructure(
	Mesh *pMesh,
	RECOG::DDD::RectStruct *pRectStruct,
	uchar *boxColor,
	float *tRS)
{
	uchar color[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {0, 255, 255}, {255, 0, 255}, {255, 255, 0}};
	Array<Point> pts[6];
	Point *ptMem;
	int iN;
	if (pMesh)
	{
		ptMem = new Point[6 * pMesh->NodeArray.n];
		for (iN = 0; iN < 6; iN++)
		{
			pts[iN].Element = ptMem + pMesh->NodeArray.n * iN;
			pts[iN].n = 0;
		}
	}
	uchar boxColor_[] = {0, 128, 0};
	if (boxColor)
	{
		RVLCOPY3VECTOR(boxColor, boxColor_);
	}
	int iRect;
	Surfel *pSurfel;
	QLIST::Index2 *pPtIdx;
	Point *pPtSrc, *pPtTgt;
	RECOG::DDD::Rect3D *pRect;
	int iAxis;
	float NM[3], NS[3];
	float d;
	float fTmp;
	float V3Tmp[3];
	Pose3D posePS;
	RVLCOPYMX3X3(pRectStruct->RRS, posePS.R);
	float s[3];
	for (iRect = 0; iRect < pRectStruct->rects.n; iRect++)
	{
		pRect = pRectStruct->rects.Element + iRect;
		if (pRect->iSurfel >= 0)
		{
			// if (iRect == 81)
			//	int debug = 0;
			if (pMesh)
			{
				RVLNULL3VECTOR(NM);
				NM[pRect->iAxis] = pRect->direction;
				RVLMULMX3X3VECT(pRectStruct->RRS, NM, NS);
				iN = pRect->iAxis + (pRect->direction > 0.0f ? 0 : 3);
				pSurfel = (pRect->iParent < 0 ? planarSurfaces.NodeArray.Element : pSurfels->NodeArray.Element) + pRect->iSurfel;
				pPtIdx = pSurfel->PtList.pFirst;
				while (pPtIdx)
				{
					pPtSrc = pMesh->NodeArray.Element + pPtIdx->Idx;
					fTmp = pRect->direction * pRect->c[pRect->iAxis] - RVLDOTPRODUCT3(NS, pPtSrc->P);
					RVLSCALE3VECTOR(NS, fTmp, V3Tmp);
					pPtTgt = pts[iN].Element + pts[iN].n++;
					RVLSUM3VECTORS(pPtSrc->P, V3Tmp, pPtTgt->P);
					pPtIdx = pPtIdx->pNext;
				}
			}
		}
		s[pRect->iAxis] = 0.01f;
		s[(pRect->iAxis + 1) % 3] = pRect->s[0];
		s[(pRect->iAxis + 2) % 3] = pRect->s[1];
		RVLMULMX3X3VECT(pRectStruct->RRS, pRect->c, posePS.t);
		if (tRS)
		{
			RVLSUM3VECTORS(posePS.t, tRS, posePS.t);
		}
		pVisualizationData->pVisualizer->DisplayBox(s[0], s[1], s[2], &posePS, boxColor_[0], boxColor_[1], boxColor_[2], true);
	}
	if (pMesh)
	{
		for (iN = 0; iN < 6; iN++)
			pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pts[iN], color[iN], 6);
		delete[] ptMem;
	}
}

void DDDetector::VisualizeHypothesisBBoxes(std::vector<RECOG::DDD::Hypothesis> *pHyps)
{
	for (int i = 0; i < pHyps->size(); i++)
	{
		RECOG::DDD::Hypothesis hyp = pHyps->at(i);
		Pose3D pose;
		RVLCOPYMX3X3(hyp.pose.R, pose.R);
		RVLCOPY3VECTOR(hyp.pose.t, pose.t);
		pVisualizationData->pVisualizer->DisplayBox(hyp.bboxSize[0], hyp.bboxSize[1], hyp.bboxSize[2], &pose, 255.0, 0.0, 0.0);
	}
}

void DDDetector::VisualizeBoxFrontFaceRGB(
	Pose3D pose,
	float *s,
	cv::Mat *pRGBDisplay,
	uchar *color)
{
	int i;
	float PB[3], PC[3];
	float U[2];
	float fTmp;
	U[0] = U[1] = 1.0f;
	PB[0] = (pose.R[6] < 0.0f ? 0.5f : -0.5f) * s[0];
	cv::Point pt[4];
	uchar defaultColor[] = {0, 0, 255};
	uchar *color_ = (color ? color : defaultColor);
	for (i = 0; i < 4; i++)
	{
		PB[1] = 0.5f * U[0] * s[1];
		PB[2] = 0.5f * U[1] * s[2];
		RVLTRANSF3(PB, pose.R, pose.t, PC);
		pt[i] = cv::Point(camera.fu * PC[0] / PC[2] + camera.uc, camera.fv * PC[1] / PC[2] + camera.vc);
		fTmp = U[0];
		U[0] = -U[1];
		U[1] = fTmp;
	}
	for (i = 0; i < 4; i++)
		cv::line(*pRGBDisplay, pt[i], pt[(i + 1) % 4], cv::Scalar(color_[0], color_[1], color_[2]), 2);
}

void DDDetector::VisualizeArticulatedObject(
	RECOG::DDD::ArticulatedObject AObj,
	Pose3D poseOC,
	bool bDisplayRS,
	cv::Mat *pRGBDisplay)
{
	if (bDisplayRS && pRGBDisplay == NULL)
	{
		RECOG::DDD::RectStruct MRS;
		MRS.rects = AObj.MRS.rects;
		Pose3D poseMO;
		RVLCOPYMX3X3(AObj.MRS.RRS, poseMO.R);
		RVLNULL3VECTOR(poseMO.t);
		Pose3D poseMC;
		RVLCOMPTRANSF3D(poseOC.R, poseOC.t, poseMO.R, poseMO.t, poseMC.R, poseMC.t);
		uchar blue[] = {0, 0, 255};
		RVLCOPYMX3X3(poseMC.R, MRS.RRS);
		VisualizeRectangularStructure(NULL, &MRS, blue, poseMC.t);
	}
	int iMovingPart;
	RECOG::DDD::HypothesisDoorDrawer *pMovingPart;
	Pose3D poseBO, poseBC;
	float tBA[3];
	tBA[2] = 0.0f;
	Pose3D poseAC;
	AffinePose3D box;
	for (iMovingPart = 0; iMovingPart < AObj.movingParts.n; iMovingPart++)
	{
		pMovingPart = AObj.movingParts.Element + iMovingPart;
		DDBox(pMovingPart, 0, &box);
		RVLCOMPTRANSF3D(poseOC.R, poseOC.t, box.R, box.t, poseBC.R, poseBC.t);
		pVisualizationData->pVisualizer->DisplayBox(frontFaceThickness, pMovingPart->s[0], pMovingPart->s[1], &poseBC, 255.0, 0.0, 0.0, false, 5.0f);
		if (pRGBDisplay)
		{
			float s[3];
			RVLSET3VECTOR(s, frontFaceThickness, pMovingPart->s[0], pMovingPart->s[1]);
			uchar green[] = {0, 255, 0};
			uchar red[] = {0, 0, 255};
			VisualizeBoxFrontFaceRGB(poseBC, s, pRGBDisplay);
		}
	}
}

void DDDetector::Get2DObjectPoints(
	RECOG::DDD::HypothesisDoorDrawer movingPart,
	PSD::Point2D *&pts)
{
	int iMovingPart;
	Pose3D poseBC;
	float PB[3], PC[3];
	float U[2];
	U[0] = U[1] = 1.0f;
	float fTmp;
	AffinePose3D box;

	DDBox(&movingPart, 0, &box);
	RVLCOPYMX3X3(box.R, poseBC.R);
	RVLCOPY3VECTOR(box.t, poseBC.t);

	PB[0] = (poseBC.R[6] < 0.0f ? 0.5f : -0.5f) * frontFaceThickness;
	for (int i = 0; i < 4; i++)
	{
		PB[1] = 0.5f * U[0] * movingPart.s[0];
		PB[2] = 0.5f * U[1] * movingPart.s[1];
		RVLTRANSF3(PB, poseBC.R, poseBC.t, PC);
		pts[i].P[0] = camera.fu * PC[0] / PC[2] + camera.uc;
		pts[i].P[1] = camera.fv * PC[1] / PC[2] + camera.vc;
		fTmp = U[0];
		U[0] = -U[1];
		U[1] = fTmp;
	}
}

float DDDetector::CalculateArea(std::vector<PSD::Point2D> ptsIn)
{
	float area = 0;
	int ptsSize = ptsIn.size();

	for (int i = 0; i < ptsSize; i++)
	{
		area += ptsIn[i].P[0] * ptsIn[(i + 1) % ptsSize].P[1] - ptsIn[i].P[1] * ptsIn[(i + 1) % ptsSize].P[0];
	}

	return area;
}

void DDDetector::IoUHypothesisEvaluation(
	RECOG::DDD::HypothesisDoorDrawer movingPart,
	std::vector<std::vector<PSD::Point2D>> gtPoints,
	float *&iouResults,
	cv::Mat *pRGBDisplay)
{
	float detectedArea, gtArea, overlapArea, unionArea, iou;

	PSD::Point2D *detPtsArr = new PSD::Point2D[4];
	Get2DObjectPoints(movingPart, detPtsArr);

	std::vector<PSD::Point2D> detectedPts, detectedPtsOut;
	detectedPts = std::vector<PSD::Point2D>(detPtsArr, detPtsArr + 4);
	GetConvexPoints(detectedPts, detectedPtsOut);
	detectedArea = pSurfelDetector->Area(detectedPtsOut);
	cout << "detectedArea: " << detectedArea << endl;
	// cout << "detectedArea2: " << CalculateArea(detectedPts) << endl;

	std::vector<PSD::Point2D> gtPointsOut;
	std::vector<PSD::Point2D> allPts, allPtsOut;
	for (int iGT = 0; iGT < gtPoints.size(); iGT++)
	{
		gtPointsOut.clear();
		GetConvexPoints(gtPoints[iGT], gtPointsOut);
		gtArea = pSurfelDetector->Area(gtPointsOut);
		cout << "gtArea: " << gtArea << endl;

		allPts.clear();
		allPts.insert(allPts.end(), detectedPts.begin(), detectedPts.end());
		allPts.insert(allPts.end(), gtPoints[iGT].begin(), gtPoints[iGT].end());
		overlapArea = GetIntersectionConvexSet(detectedPts, gtPoints[iGT], &allPtsOut);
		cout << "overlapArea: " << overlapArea << endl;
		// allPtsOut.clear();
		// GetConvexPoints(allPts, allPtsOut);
		// overlapArea = pSurfelDetector->Area(allPtsOut);
		// cout << "overlapArea: " << overlapArea << endl;
		// cout << endl;

		unionArea = detectedArea + gtArea;
		iou = overlapArea / (unionArea - overlapArea);

		iouResults[iGT] = iou;

		VisualizeHypothesisGT(*pRGBDisplay, detectedPts, gtPoints[iGT]);
	}

	delete[] detPtsArr;
}

void DDDetector::cvIoUHypothesisEvaluation(
	RECOG::DDD::HypothesisDoorDrawer movingPart,
	std::vector<std::vector<PSD::Point2D>> gtPoints,
	float *&iouResults,
	cv::Mat *pRGBDisplay)
{
	float detectedArea, gtArea, cvIntersectionArea, unionArea, iou;

	PSD::Point2D *detPtsArr = new PSD::Point2D[4];
	std::vector<PSD::Point2D> detectedPts;
	std::vector<cv::Point> cvDetectedPts, cvGTPts, cvAllPts;

	Get2DObjectPoints(movingPart, detPtsArr);
	detectedPts = std::vector<PSD::Point2D>(detPtsArr, detPtsArr + 4);
	VectorPSDPoin2DToVectorCvPoint(&detectedPts, cvDetectedPts);
	detectedArea = cv::contourArea(cvDetectedPts);

	for (int iGT = 0; iGT < gtPoints.size(); iGT++)
	{
		cvGTPts.clear();
		cvAllPts.clear();
		VectorPSDPoin2DToVectorCvPoint(&gtPoints[iGT], cvGTPts);
		gtArea = cv::contourArea(cvGTPts);
		cvIntersectionArea = cv::intersectConvexConvex(cvDetectedPts, cvGTPts, cvAllPts);

		cout << "gtArea: " << gtArea << endl;
		cout << "intersectionArea: " << cvIntersectionArea << endl;

		unionArea = detectedArea + gtArea;
		iou = cvIntersectionArea / (unionArea - cvIntersectionArea);
		iouResults[iGT] = iou;

		VisualizeHypothesisGT(*pRGBDisplay, detectedPts, gtPoints[iGT]);
	}

	delete[] detPtsArr;
}

void DDDetector::VectorPSDPoin2DToVectorCvPoint(std::vector<PSD::Point2D> *ptsIn, std::vector<cv::Point> &ptsOut)
{
	for (int i = 0; i < ptsIn->size(); i++)
		ptsOut.push_back(cv::Point(ptsIn->at(i).P[0], ptsIn->at(i).P[1]));
}

float DDDetector::GetIntersectionConvexSet(std::vector<PSD::Point2D> poly1, std::vector<PSD::Point2D> poly2, std::vector<PSD::Point2D> *outPoly)
{
	std::vector<cv::Point> cvPoly1;
	std::vector<cv::Point> cvPoly2;
	std::vector<cv::Point> cvOutPoly;
	float cvIntersectionArea, cvDetectedArea, cvGTArea;

	for (int i = 0; i < poly1.size(); i++)
		cvPoly1.push_back(cv::Point(poly1[i].P[0], poly1[i].P[1]));
	for (int i = 0; i < poly2.size(); i++)
		cvPoly2.push_back(cv::Point(poly2[i].P[0], poly2[i].P[1]));
	cvIntersectionArea = cv::intersectConvexConvex(cvPoly1, cvPoly2, cvOutPoly);

	cvDetectedArea = cv::contourArea(cvPoly1);
	cvGTArea = cv::contourArea(cvPoly2);

	PSD::Point2D pt;
	for (int i = 0; i < cvOutPoly.size(); i++)
	{
		pt.P[0] = cvOutPoly[i].x;
		pt.P[1] = cvOutPoly[i].y;
		outPoly->push_back(pt);
	}

	cout << "cvIOU: " << cvIntersectionArea / (cvDetectedArea + cvGTArea - cvIntersectionArea) << endl;

	return cvIntersectionArea;
}

void DDDetector::VisualizeHypothesisGT(
	cv::Mat pRGBDisplay,
	std::vector<PSD::Point2D> detectedPts,
	std::vector<PSD::Point2D> gtPts)
{
	for (int iPt = 0; iPt < detectedPts.size(); iPt++)
	{
		cv::line(pRGBDisplay,
				 cv::Point(detectedPts[iPt].P[0], detectedPts[iPt].P[1]),
				 cv::Point(detectedPts[(iPt + 1) % detectedPts.size()].P[0], detectedPts[(iPt + 1) % detectedPts.size()].P[1]),
				 cv::Scalar(0, 0, 255), 2);
	}
	// for (int iPt = 0; iPt < gtPts.size(); iPt++)
	// {
	// 	cv::line(pRGBDisplay,
	// 			 cv::Point(gtPts[iPt].P[0], gtPts[iPt].P[1]),
	// 			 cv::Point(gtPts[(iPt + 1) % gtPts.size()].P[0], gtPts[(iPt + 1) % gtPts.size()].P[1]),
	// 			 cv::Scalar(0, 255, 0), 2);
	// }

	cv::imshow("VisualizeGTHypothesis", pRGBDisplay);
	cv::waitKey(0);
}

void DDDetector::GetConvexPoints(std::vector<PSD::Point2D> ptsIn, std::vector<PSD::Point2D> &ptsOut)
{
	float N[2];
	float d;
	float dx, dy;
	ptsOut = ptsIn;
	int ptsSize = ptsIn.size();
	int nextPtIdx;
	// Get normals and offsets from pts
	for (int iPt = 0; iPt < ptsSize; iPt++)
	{
		nextPtIdx = (iPt + 1) % ptsSize;
		dx = ptsIn[nextPtIdx].P[0] - ptsIn[iPt].P[0];
		dy = ptsIn[nextPtIdx].P[1] - ptsIn[iPt].P[1];
		// N[0] = dy;
		// N[1] = -dx;
		N[0] = -dy;
		N[1] = dx;
		d = N[0] * ptsIn[iPt].P[0] + N[1] * ptsIn[iPt].P[1];
		pSurfelDetector->UpdateConvexSet(ptsOut, N, d, ptsOut);
	}
}

void DDDetector::GetGTPointsFromCSV(std::vector<std::vector<std::string>> csvContent, std::vector<std::vector<PSD::Point2D>> &allGTPoints)
{
	std::vector<PSD::Point2D> gtPoints;
	PSD::Point2D gtPoint;
	allGTPoints.clear();
	for (int iGT = 0; iGT < csvContent.size(); iGT++)
	{
		gtPoints.clear();
		for (int iPt = 0; iPt < 4; iPt++)
		{
			gtPoint.P[0] = std::stof(csvContent[iGT][2 * iPt]);
			gtPoint.P[1] = std::stof(csvContent[iGT][2 * iPt + 1]);
			gtPoints.push_back(gtPoint);
		}
		allGTPoints.push_back(gtPoints);
	}
}

bool DDDetector::CreateMeshFromPolyData(Mesh *pMesh)
{
	// Copy vertices.

	pMesh->NodeArray.n = pMesh->pPolygonData->GetNumberOfPoints();
	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pMesh->pPolygonData->GetPoints()->GetData());
	if (pointData == NULL)
		return false;
	int iPt;
	Point *pPt;
	RVL_DELETE_ARRAY(pMesh->NodeMem);
	pMesh->NodeMem = new Point[pMesh->NodeArray.n];
	pMesh->NodeArray.Element = pMesh->NodeMem;
	QList<MeshEdgePtr> *pEdgeList;
	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		pPt = pMesh->NodeArray.Element + iPt;
		pointData->GetTypedTuple(iPt, pPt->P);
		pEdgeList = &(pPt->EdgeList);
		RVLQLIST_INIT(pEdgeList);
	}

	// Allocate faces and determine the total number of edges.

	pMesh->faces.n = pMesh->pPolygonData->GetPolys()->GetNumberOfCells();
	RVL_DELETE_ARRAY(pMesh->faces.Element);
	pMesh->faces.Element = new MESH::Face *[pMesh->faces.n];
	RVL_DELETE_ARRAY(pMesh->faceMem);
	pMesh->faceMem = new MESH::Face[pMesh->faces.n];
	vtkSmartPointer<vtkCellArray> Polys = pMesh->pPolygonData->GetPolys();
	pMesh->EdgeArray.n = 0;
	Polys->InitTraversal();
	vtkSmartPointer<vtkIdList> l = vtkSmartPointer<vtkIdList>::New();
	int nPts;
	int iFace;
	MESH::Face *pFace;
	for (iFace = 0; iFace < pMesh->faces.n; iFace++)
	{
		pFace = pMesh->faceMem + iFace;
		pMesh->faces.Element[iFace] = pFace;
		pFace->pFirstEdgePtr = NULL;
		if (Polys->GetNextCell(l))
		{
			nPts = l->GetNumberOfIds();
			pMesh->EdgeArray.n += nPts;
		}
	}
	pMesh->EdgeArray.n /= 2;

	// Create faces and edges.

	RVL_DELETE_ARRAY(pMesh->EdgeMem);
	pMesh->EdgeMem = new MeshEdge[pMesh->EdgeArray.n];
	pMesh->EdgeArray.Element = pMesh->EdgeMem;
	RVL_DELETE_ARRAY(pMesh->EdgePtrMem);
	pMesh->EdgePtrMem = new MeshEdgePtr[2 * pMesh->EdgeArray.n];
	int iEdge;
	MeshEdge *pEdge;
	for (iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		pEdge = pMesh->EdgeArray.Element + iEdge;
		pEdge->pFace[0] = pEdge->pFace[1] = NULL;
	}
	int i, iPt_, iPt__;
	MeshEdgePtr *pEdgePtr;
	MeshEdgePtr *pNewEdgePtr = pMesh->EdgePtrMem;
	MeshEdge *pNewEdge = pMesh->EdgeArray.Element;
	int iVertex[3];
	float V1[3], V2[3];
	Point *pPt_;
	float fTmp;
	Polys->InitTraversal();
	for (iFace = 0; iFace < pMesh->faces.n; iFace++)
	{
		pFace = pMesh->faces.Element[iFace];
		pFace->idx = iFace;
		if (Polys->GetNextCell(l))
		{
			nPts = l->GetNumberOfIds();
			iPt = l->GetId(nPts - 1);
			for (i = 0; i < nPts; i++)
			{
				if (i < 3)
					iVertex[i] = iPt;
				pPt = pMesh->NodeArray.Element + iPt;
				iPt_ = l->GetId(i);
				pEdgePtr = pPt->EdgeList.pFirst;
				while (pEdgePtr)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt__);
					if (iPt__ == iPt_)
						break;
					pEdgePtr = pEdgePtr->pNext;
				}
				if (pEdgePtr)
				{
					if (pFace->pFirstEdgePtr == NULL)
						pFace->pFirstEdgePtr = pEdgePtr;
					pEdge->pFace[1] = pFace;
				}
				else
				{
					if (pFace->pFirstEdgePtr == NULL)
						pFace->pFirstEdgePtr = pNewEdgePtr;
					ConnectNodes<Point, MeshEdge, MeshEdgePtr>(iPt, iPt_, pMesh->NodeArray, pNewEdge, pNewEdgePtr);
					pNewEdge->pFace[0] = pFace;
					pNewEdge++;
					pNewEdgePtr += 2;
				}
				iPt = iPt_;
			}
		}
		pPt = pMesh->NodeArray.Element + iVertex[1];
		pPt_ = pMesh->NodeArray.Element + iVertex[2];
		RVLDIF3VECTORS(pPt_->P, pPt->P, V1);
		pPt_ = pMesh->NodeArray.Element + iVertex[0];
		RVLDIF3VECTORS(pPt_->P, pPt->P, V2);
		RVLCROSSPRODUCT3(V1, V2, pFace->N);
		RVLNORM3(pFace->N, fTmp);
		pFace->d = RVLDOTPRODUCT3(pFace->N, pPt->P);
	}

	return true;
}

RECOG::DDD::Model::Model()
{
	points.Element = NULL;
	points_.Element = NULL;
	d = NULL;
	A = NULL;
	AID = NULL;
	M = NULL;
	Q = NULL;
	R = NULL;
	edges.Element = NULL;
	info = NULL;
}

RECOG::DDD::Model::~Model()
{
	RVL_DELETE_ARRAY(points.Element);
	RVL_DELETE_ARRAY(points_.Element);
	RVL_DELETE_ARRAY(d);
	RVL_DELETE_ARRAY(A);
	RVL_DELETE_ARRAY(AID);
	RVL_DELETE_ARRAY(M);
	RVL_DELETE_ARRAY(edges.Element);
	RVL_DELETE_ARRAY(Q);
	RVL_DELETE_ARRAY(R);
	RVL_DELETE_ARRAY(info);
}

bool RECOG::DDD::ProjectToBase(
	float *a,
	int n,
	float *Q,
	int m,
	float *r,
	float *b,
	float &c)
{
	memcpy(b, a, n * sizeof(float));
	float *q;
	int i, i_;
	float r_;
	for (i = 0; i < m; i++)
	{
		q = Q + n * i;
		RVLDOTPRODUCT(q, a, n, r_, i_);
		r[i] = r_;
		for (i_ = 0; i_ < n; i_++)
			b[i_] -= (q[i_] * r_);
	}
	RVLDOTPRODUCT(b, b, n, c, i_);
	if (c < 1e-12)
		return false;
	c = sqrt(c);
	int fTmp = 1.0f / c;
	RVLSCALEVECTOR(b, fTmp, b, n, i_);
	return true;
}

bool RECOG::DDD::SortCompare(SortIndex<float> x1, SortIndex<float> x2)
{
	return (x1.cost < x2.cost);
}

// The following function could be moved to some general purpose utility cpp file, e.g. Util.cpp.

void RECOG::DDD::MapImageC1(
	cv::Mat srcImg,
	cv::Mat mapping,
	cv::Mat &tgtImg)
{
	int w = mapping.cols;
	int h = mapping.rows;
	tgtImg.create(mapping.size(), CV_8UC1);
	int nTgtPix = w * h;
	int iSrcPt, iTgtPix;
	int *piSrcPt = (int *)(mapping.data);
	for (iTgtPix = 0; iTgtPix < nTgtPix; iTgtPix++, piSrcPt++)
	{
		iSrcPt = *piSrcPt;
		tgtImg.data[iTgtPix] = (iSrcPt >= 0 ? srcImg.data[iSrcPt] : 0);
	}
}

// The following function could be moved to Mesh.cpp.

void RECOG::DDD::MapMeshRGB(
	Mesh *pMesh,
	cv::Mat mapping,
	cv::Mat &tgtImg)
{
	int w = mapping.cols;
	int h = mapping.rows;
	tgtImg.create(mapping.size(), CV_8UC3);
	int nTgtPix = w * h;
	int iSrcPt, iTgtPix;
	int *piSrcPt = (int *)(mapping.data);
	uchar *tgtPix = tgtImg.data;
	uchar *srcPix;
	for (iTgtPix = 0; iTgtPix < nTgtPix; iTgtPix++, piSrcPt++, tgtPix += 3)
	{
		iSrcPt = *piSrcPt;
		if (iSrcPt >= 0)
		{
			srcPix = pMesh->NodeArray.Element[iSrcPt].RGB;
			RVLSET3VECTOR(tgtPix, srcPix[2], srcPix[1], srcPix[0]);
		}
		else
			RVLSET3VECTOR(tgtPix, 255, 0, 0);
	}
}

void RECOG::DDD::Detect3CallBackFunc(int event, int x, int y, int flags, void *userdata)
{
}

// The following three functions could be moved to some general purpose utility cpp file, e.g. Util.cpp.

void RVL::EpipolarGeometryCallback(int event, int x, int y, int flags, void *userdata)
{
	EpipolarGeometryDisplayData *data = (EpipolarGeometryDisplayData *)userdata;

	if (event == cv::EVENT_LBUTTONDOWN)
	{
		int w1 = data->imgSize[0][1];
		// int iSrcImg = (x < w1 ? 0 : 1);
		// int iTgtImg = 1 - iSrcImg;
		float m[3];
		float l[3];
		int offset, offset_;
		float u;
		float v = (float)y;
		float w, h;
		if (x < w1)
		{
			offset = 0;
			u = (float)x;
			RVLSET3VECTOR(m, u, v, 1.0f);
			RVLMULMX3X3TVECT(data->F21, m, l);
			w = (float)w1;
			h = (float)(data->imgSize[0][0]);
		}
		else
		{
			offset = w1;
			u = (float)(x - offset);
			RVLSET3VECTOR(m, u, v, 1.0f);
			RVLMULMX3X3VECT(data->F21, m, l);
			w = (float)(data->imgSize[1][1]);
			h = (float)(data->imgSize[1][0]);
		}
		offset_ = w1 - offset;
		Array<Vector3<float>> imgRect;
		Vector3<float> imgRectMem[4];
		imgRect.Element = imgRectMem;
		imgRect.n = 4;
		float *imgRectSide;
		imgRectSide = imgRect.Element[0].Element;
		imgRectSide[0] = -1.0f;
		imgRectSide[1] = 0.0f;
		imgRectSide[2] = 0.0f;
		imgRectSide = imgRect.Element[1].Element;
		imgRectSide[0] = 0.0f;
		imgRectSide[1] = -1.0f;
		imgRectSide[2] = 0.0f;
		imgRectSide = imgRect.Element[2].Element;
		imgRectSide[0] = 1.0f;
		imgRectSide[1] = 0.0f;
		imgRectSide[2] = w;
		imgRectSide = imgRect.Element[3].Element;
		imgRectSide[0] = 0.0f;
		imgRectSide[1] = 1.0f;
		imgRectSide[2] = h;
		float line[4];
		LineConvexSetIntersection2D(l, imgRect, line);
		data->intiDisplay.copyTo(data->display);
		cv::drawMarker(data->display, cv::Point(x, y), cv::Scalar(0, 255, 0));
		cv::line(data->display, cv::Point(line[0] + offset_, line[1]), cv::Point(line[2] + offset_, line[3]), cv::Scalar(0, 255, 0));

		// cv::imwrite("/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP-221216/results-20230117-dist/epipolar_geometry" + std::to_string(data->counter) + ".png", data->display);
		data->counter++;
		cv::imshow("epipolar geometry", data->display);
	}
}

void RVL::VisualizeEpipolarGeometry(
	cv::Mat img1,
	cv::Mat img2,
	cv::Mat cvP1,
	cv::Mat cvP2,
	Pose3D pose21)
{
	float V3Tmp[9];
	RVLSKEW(pose21.t, V3Tmp);
	float E21[9];
	RVLMXMUL3X3(V3Tmp, pose21.R, E21);
	cv::Mat cvInvP1 = cvP1.inv();
	double *invP1 = (double *)(cvInvP1.data);
	cv::Mat cvInvP2 = cvP2.inv();
	double *invP2 = (double *)(cvInvP2.data);
	RVLMXMUL3X3(E21, invP2, V3Tmp);
	EpipolarGeometryDisplayData data;
	RVLMXMUL3X3T1(invP1, V3Tmp, data.F21);
	int h = RVLMAX(img1.rows, img2.rows);
	int w = img1.cols + img2.cols;
	cv::Mat display(h, w, CV_8UC3);
	display.setTo(cv::Scalar(255, 0, 0));
	cv::Mat ROI1(display, cv::Rect(0, 0, img1.cols, img1.rows));
	img1.copyTo(ROI1);
	cv::Mat ROI2(display, cv::Rect(img1.cols, 0, img2.cols, img2.rows));
	img2.copyTo(ROI2);
	cv::namedWindow("epipolar geometry");
	data.display = display;
	data.intiDisplay = display.clone();
	data.imgSize[0][0] = img1.size.p[0];
	data.imgSize[0][1] = img1.size.p[1];
	data.imgSize[1][0] = img2.size.p[0];
	data.imgSize[1][1] = img2.size.p[1];
	cv::setMouseCallback("epipolar geometry", EpipolarGeometryCallback, &data);
	cv::imshow("epipolar geometry", display);
	cv::waitKey();
}

// Input:
//    line - line defined by equation line[0] * x + line[1] * y + line[2] = 0
//    convexSet - array of lines defining a convex set, where each line is represented by a 3D vector
//                whose first two elements represent line normal n and the third element represents the line offset d
//                (line equation: n[0] * x + n[1] * y = d)
//    intersectionLineSegment - the line segment representing the intersection defined by a 4D vector,
//                whose first two elements represent the first endpoint and the other two elements the second endpoint

bool RVL::LineConvexSetIntersection2D(
	float *line,
	Array<Vector3<float>> convexSet,
	float *intersectionLineSegment)
{
	float c[2];
	float fTmp = -line[2] / (line[0] * line[0] + line[1] * line[1]);
	c[0] = fTmp * line[0];
	c[1] = fTmp * line[1];
	float v[2];
	v[0] = -line[1];
	v[1] = line[0];

	float a;
	float *b;
	float s0;
	float smin = 0.0f;
	float smax = 0.0f;
	bool bsmin = false;
	bool bsmax = false;
	for (int i = 0; i < convexSet.n; i++)
	{
		b = convexSet.Element[i].Element;
		a = b[0] * v[0] + b[1] * v[1];
		if (RVLABS(a) > 1e-10)
		{
			s0 = (b[2] - (b[0] * c[0] + b[1] * c[1])) / a;
			if (a > 0.0f)
			{
				if (s0 < smax || !bsmax)
				{
					smax = s0;
					bsmax = true;
				}
			}
			else
			{
				if (s0 > smin || !bsmin)
				{
					smin = s0;
					bsmin = true;
				}
			}
		}
	}

	if (smax <= smin)
		return false;
	else
	{
		intersectionLineSegment[0] = c[0] + smin * v[0];
		intersectionLineSegment[1] = c[1] + smin * v[1];
		intersectionLineSegment[2] = c[0] + smax * v[0];
		intersectionLineSegment[3] = c[1] + smax * v[1];

		return true;
	}
}

bool DDDetector::GetVisualizeDoorHypotheses()
{
	return pVisualizationData->bVisualizeDoorHypotheses;
}

bool DDDetector::GetRGBImageVisualization()
{
	return pVisualizationData->bRGBImageVisualization;
}

void DDDetector::VisualizeDDHypothesis(
	RECOG::DDD::HypothesisDoorDrawer hyp,
	int imgID)
{
	int m;
	AffinePose3D box;
	Pose3D poseBC;
	float lineWidth = (imgID >= 0 ? 5.0f : 1.0f);
	float scale;
	std::string unit;
	if (hyp.objClass == RVLDDD_MODEL_DOOR)
	{
		scale = RAD2DEG;
		unit = "deg";
	}
	else if (hyp.objClass == RVLDDD_MODEL_DRAWER)
	{
		scale = 1.0f;
		unit = "m";
	}
	for (m = 0; m < hyp.state.n; m++)
		if ((imgID < 0 || hyp.state.Element[m].imgID == imgID))
		{
			printf("State=%f %s\n", hyp.state.Element[m].q * scale, unit.data());
			DDBox(&hyp, m, &box);
			RVLCOPY3DPOSE(box.R, box.t, poseBC.R, poseBC.t);
			pVisualizationData->pVisualizer->DisplayBox(box.s[0], box.s[1], box.s[2], &poseBC, 255.0, 0.0, 0.0, false, lineWidth);
		}
	if (hyp.objClass == RVLDDD_MODEL_DOOR)
	{
		float Z[3];
		RVLCOPYCOLMX3X3(hyp.pose.R, 2, Z);
		float fTmp = hyp.s[1];
		float V3Tmp[3];
		RVLSCALE3VECTOR(Z, fTmp, V3Tmp);
		Point jointAxisEndPt[2];
		Point *pPt = jointAxisEndPt;
		RVLSUM3VECTORS(hyp.pose.t, V3Tmp, pPt->P);
		pPt++;
		RVLDIF3VECTORS(hyp.pose.t, V3Tmp, pPt->P);
		uchar green[] = {0, 255, 0};
		pVisualizationData->pVisualizer->DisplayLine(jointAxisEndPt, green, 5.0f);
	}
}

void DDDetector::VisualizeDoorHypothesis(
	RECOG::DDD::HypothesisDoorDrawer doorHyp,
	int imgID)
{
	Pose3D poseBC;
	Pose3D poseBA;
	float tBB_[3];
	RVLSET3VECTOR(tBB_, doorHyp.r[0], doorHyp.r[1], 0.0f);
	float q;
	int m;
	float lineWidth = (imgID >= 0 ? 5.0f : 1.0f);
	for (m = 0; m < doorHyp.state.n; m++)
		if (imgID < 0 || doorHyp.state.Element[m].imgID == imgID)
		{
			q = doorHyp.state.Element[m].q;
			RVLROTZ(cos(q), sin(q), poseBA.R);
			RVLMULMX3X3VECT(poseBA.R, tBB_, poseBA.t);
			RVLCOMPTRANSF3D(doorHyp.pose.R, doorHyp.pose.t, poseBA.R, poseBA.t, poseBC.R, poseBC.t);
			pVisualizationData->pVisualizer->DisplayBox(models.Element[0].bboxSize[2], doorHyp.s[0], doorHyp.s[1], &poseBC, 255.0, 0.0, 0.0, false, lineWidth);
		}
	float Z[3];
	RVLCOPYCOLMX3X3(doorHyp.pose.R, 2, Z);
	float fTmp = doorHyp.s[1];
	float V3Tmp[3];
	RVLSCALE3VECTOR(Z, fTmp, V3Tmp);
	Point jointAxisEndPt[2];
	Point *pPt = jointAxisEndPt;
	RVLSUM3VECTORS(doorHyp.pose.t, V3Tmp, pPt->P);
	pPt++;
	RVLDIF3VECTORS(doorHyp.pose.t, V3Tmp, pPt->P);
	uchar green[] = {0, 255, 0};
	pVisualizationData->pVisualizer->DisplayLine(jointAxisEndPt, green, 5.0f);
}

void DDDetector::VisualizeDrawerHypothesis(
	RECOG::DDD::HypothesisDoorDrawer drawerHyp,
	int imgID)
{
	int m;
	float lineWidth = (imgID >= 0 ? 5.0f : 1.0f);
	Pose3D poseBC;
	float tBA[3];
	tBA[1] = tBA[2] = 0.0f;
	RVLCOPYMX3X3(drawerHyp.pose.R, poseBC.R);
	for (m = 0; m < drawerHyp.state.n; m++)
		if (imgID < 0 || drawerHyp.state.Element[m].imgID == imgID)
		{
			tBA[0] = drawerHyp.state.Element[m].q;
			RVLTRANSF3(tBA, drawerHyp.pose.R, drawerHyp.pose.t, poseBC.t);
			pVisualizationData->pVisualizer->DisplayBox(models.Element[0].bboxSize[2], drawerHyp.s[0], drawerHyp.s[1], &poseBC, 255.0, 0.0, 0.0, false, lineWidth);
		}
}

// I. Vidovic
void DDDetector::Detect2(Array<Mesh> meshSeq)
{
	// ROI.

	// DEBUG
	/*Box<float> ROI;
	ROI.minx = -0.300;
	ROI.maxx = 0.200;
	ROI.miny = 0.150;
	ROI.maxy = 0.400;
	ROI.minz = 0.900;
	ROI.maxz = 1.600;
	Array<int> SOI;
	int SOIMem;
	SOI.Element = &SOIMem;
	SOI.n = 1;
	//SOI.Element[0] = 762;
	SOI.Element[0] = 2820;
	Array<int>* pSOI = (test == RVLDDD_TEST_DDD2 ? &SOI : NULL)*/
	;

	// Direction.

	float U[3];
	// RVLSET3VECTOR(U, 0.0f, 0.0f, -1.0f);
	RVLSET3VECTOR(U, 0.0f, COS45, -COS45);

	//

	Mesh *pMesh, *pMesh_;
	RECOG::DDD::Model *pModel = models.Element;
	PointAssociationData pointAssociationData;
	Array<int> ptBuff;
	ptBuff.Element = new int[camera.w * camera.h];
	Array<Point> pointsMS;
	pointsMS.Element = new Point[pModel->points.n];
	Array<OrientedPoint> pointsS;
	pointsS.Element = new OrientedPoint[camera.w * camera.h];
	AffinePose3D APoseRef, APose;
	float *t = new float[3 * meshSeq.n];
	float *t_ = t + 3 * (meshSeq.n - 1);
	RVLNULL3VECTOR(t_);
	std::vector<RECOG::DDD::Hypothesis> hyps;
	RECOG::DDD::Hypothesis hyp;

	// Array<std::vector<RECOG::DDD::Hypothesis>> seqHyps;
	// seqHyps.Element = new std::vector<RECOG::DDD::Hypothesis>[meshSeq.n];
	// seqHyps.n = meshSeq.n;

	Array<int> dominantShiftPointsIdx;
	dominantShiftPointsIdx.Element = new int[0];

	int ROIStep;
	std::vector<AffinePose3D> ROIs;
	AffinePose3D ROI_;
	Vector3<float> dominantShift;

	for (int iMesh = meshSeq.n - 1; iMesh > 0; iMesh--)
	{
		pMesh = meshSeq.Element + iMesh;
		ROIStep = ROICalculationStep;
		dominantShiftPointsIdx.n = 0;
		while (dominantShiftPointsIdx.n < minPointsWithDominantShift)
		{
			if (ROIStep > iMesh)
				break;

			pMesh_ = pMesh - ROIStep++;
			DetectDominantShiftPoints(pMesh, pMesh_, &dominantShiftPointsIdx, dominantShift, true);
		}
		if (dominantShiftPointsIdx.n < minPointsWithDominantShift)
		{
			printf("There is no enough dominant shift points. No hypotheses are generated.\n");
			continue;
		}
		Box<float> ROI;
		CalculateROI(pMesh, dominantShiftPointsIdx, &ROI);
		RVLSET3VECTOR(ROI_.s, ROI.maxx - ROI.minx, ROI.maxy - ROI.miny, ROI.maxz - ROI.minz);
		RVLSET3VECTOR(ROI_.t, 0.5f * (ROI.maxx + ROI.minx), 0.5f * (ROI.maxy + ROI.miny), 0.5f * (ROI.maxz + ROI.minz));
		RVLUNITMX3(ROI_.R);
		ROIs.push_back(ROI_);
		GenerateHypotheses(pMesh, ROIs, &dominantShiftPointsIdx, &dominantShift, hyps, U, NULL);
		if (hyps.size() == 0)
		{
			printf("No hypotheses are generated.\n");
			continue;
		}

		// Hypothesis bounding box visualization.
		hyp = hyps[0];
		if (pVisualizationData->bVisualizeInitialHypothesis)
		{
			pVisualizationData->pVisualizer->SetMesh(pMesh);
			VisualizeHypothesisBoundingBox(&hyp);
			pVisualizationData->pVisualizer->Run();
			pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
		}

		// Perform AICP on generated hypothesis
		imgGrid.Create(pMesh->NodeArray, &camera, pointAssociationGridCellSize);
		#ifdef RVLLINUX
		CreateOrientedPointArrayFromPointArray(pMesh->NodeArray, pointsS);
		#else
		MESH::CreateOrientedPointArrayFromPointArray(pMesh->NodeArray, pointsS);
		#endif
		pointAssociationData.Create(pModel->points.n, pointsS.n, true);
		// PointAssociation(pModel->points, &(hyp.pose), pointsS, pointAssociationData, ptBuff, true, &pointsMS);
		SetSceneForHypothesisVisualization(pMesh);
		SetBBoxForHypothesisVisualization(&hyp);

		AICP(pModel->points, pointsS, hyp.pose, nICPIterations, APoseRef, pointAssociationData, &pointsMS);
		APose = APoseRef;

		pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
		pointAssociationData.Clear();

		// Add generated hypothesis to seqHyps Array
		hyp.pose = APose;
		// seqHyps.Element[iMesh].push_back(hyp);

		// Evaluate hypothesis
		EvaluateHypothesis(pMesh, hyp.pose);
	}

	// CalculateMeanHypothesis(seqHyps);

	delete[] ptBuff.Element;
	delete[] pointsMS.Element;
	delete[] pointsS.Element;
	delete[] t;
}

void DDDetector::CalculateMeanHypothesis(Array<std::vector<RECOG::DDD::Hypothesis>> seqHypotheses)
{
	/////////////////////////////////
	// This function is NOT DEBUGED!!!
	/////////////////////////////////
	AffinePose3D meanPose;

	memset(meanPose.s, 0.0f, 3 * sizeof(float));
	memset(meanPose.R, 0.0f, 9 * sizeof(float));
	memset(meanPose.t, 0.0f, 3 * sizeof(float));

	RECOG::DDD::Hypothesis hyp;
	int nSeq = seqHypotheses.n;
	int nHyps = 0;

	for (int iMesh = 0; iMesh < nSeq; iMesh++)
	{
		if (seqHypotheses.Element[iMesh].size() > 0)
		{
			hyp = seqHypotheses.Element[iMesh][0];

			meanPose.s[0] += hyp.pose.s[0];
			meanPose.s[1] += hyp.pose.s[1];
			meanPose.s[2] += hyp.pose.s[2];

			meanPose.R[0] += hyp.pose.R[0];
			meanPose.R[1] += hyp.pose.R[1];
			meanPose.R[2] += hyp.pose.R[2];
			meanPose.R[3] += hyp.pose.R[3];
			meanPose.R[4] += hyp.pose.R[4];
			meanPose.R[5] += hyp.pose.R[5];
			meanPose.R[6] += hyp.pose.R[6];
			meanPose.R[7] += hyp.pose.R[7];
			meanPose.R[8] += hyp.pose.R[8];

			meanPose.t[0] += hyp.pose.t[0];
			meanPose.t[1] += hyp.pose.t[1];
			meanPose.t[2] += hyp.pose.t[2];

			nHyps++;
		}
	}

	meanPose.s[0] /= nHyps;
	meanPose.s[1] /= nHyps;
	meanPose.s[2] /= nHyps;

	meanPose.R[0] /= nHyps;
	meanPose.R[1] /= nHyps;
	meanPose.R[2] /= nHyps;
	meanPose.R[3] /= nHyps;
	meanPose.R[4] /= nHyps;
	meanPose.R[5] /= nHyps;
	meanPose.R[6] /= nHyps;
	meanPose.R[7] /= nHyps;
	meanPose.R[8] /= nHyps;

	meanPose.t[0] /= nHyps;
	meanPose.t[1] /= nHyps;
	meanPose.t[2] /= nHyps;

	for (int iMesh = 0; iMesh < nSeq; iMesh++)
	{
		if (seqHypotheses.Element[iMesh].size() > 0)
		{
			hyp = seqHypotheses.Element[iMesh][0];
			meanPose.t[2] = hyp.pose.t[2];
			hyp.pose = meanPose;
			seqHypotheses.Element[iMesh].push_back(hyp);
		}
	}
}

void DDDetector::EvaluateHypothesis(Mesh *pMesh, AffinePose3D pose)
{
	RECOG::DDD::Model *pModel = models.Element;
	RECOG::InitZBuffer(pMesh, sceneSamplingResolution, ZBuffer, ZBufferActivePtArray, subImageMap);
	CreateImage3x3NeighborhoodLT(ZBuffer.w, image3x3Neighborhood);

	float s[7];
	float sceneFittingScore;
	float bestSceneFittingScore = 0.0f;
	int nTransparentPts;
	int *SMCorrespondence = new int[ZBuffer.w * ZBuffer.h];
	RVL_DELETE_ARRAY(surfelMask);
	surfelMask = new bool[pSurfels->NodeArray.n];
	for (int iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
		surfelMask[iSurfel] = true;

	s[0] = 0.0f;
	s[1] = pModel->bboxSize[0] / 2 * pose.s[0];
	s[2] = pModel->bboxSize[1] / 2 * pose.s[1];
	s[3] = pModel->bboxSize[2] / 2 * pose.s[2];
	memset(s + 4, 0, 3 * sizeof(float));

	sceneFittingScore = EvaluateHypothesis(pMesh, pModel, 7, s, pose.R, chamferDistThr, nTransparentPts, SMCorrespondence, NULL, pose.t);
	sceneFittingScore -= (float)nTransparentPts;
	printf("Scene fitting score: %f\t(nTransparentPts = %d)\n", sceneFittingScore, nTransparentPts);

	if (pVisualizationData->bVisualizeFittingScoreCalculation)
	{
		VisualizeFittingScoreCalculation(pMesh, nTransparentPts, SMCorrespondence);
	}

	delete[] SMCorrespondence;
}

float DDDetector::EvaluateHypothesis(Mesh *pMesh, RECOG::DDD::Model *pModel, AffinePose3D pose, bool bPrintDebug)
{
	RECOG::InitZBuffer(pMesh, sceneSamplingResolution, ZBuffer, ZBufferActivePtArray, subImageMap);
	CreateImage3x3NeighborhoodLT(ZBuffer.w, image3x3Neighborhood);

	float s[7];
	float sceneFittingScore;
	float bestSceneFittingScore = 0.0f;
	int nTransparentPts;
	int *SMCorrespondence = new int[ZBuffer.w * ZBuffer.h];
	RVL_DELETE_ARRAY(surfelMask);
	surfelMask = new bool[pSurfels->NodeArray.n];
	for (int iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
		surfelMask[iSurfel] = true;

	/*s[0] = 0.0f;
	s[1] = pModel->bboxSize[0] / 2 * pose.s[0];
	s[2] = pModel->bboxSize[1] / 2 * pose.s[1];
	s[3] = pModel->bboxSize[2] / 2 * pose.s[2];
	memset(s + 4, 0, 3 * sizeof(float));*/

	memset(s, 0, 7 * sizeof(float));
	float *s_ = s + 1;
	RVLSCALE3VECTOR(pose.s, 0.5f, s_);

	sceneFittingScore = EvaluateHypothesis(pMesh, pModel, 7, s, pose.R, chamferDistThr, nTransparentPts, SMCorrespondence, NULL, pose.t);
	sceneFittingScore -= (float)nTransparentPts;
	if (bPrintDebug)
		printf("  Scene fitting score: %f.\tNumber of transparent points: %d.\n", sceneFittingScore, nTransparentPts);

	if (pVisualizationData->bVisualizeFittingScoreCalculation)
	{
		VisualizeFittingScoreCalculation(pMesh, nTransparentPts, SMCorrespondence);
	}

	delete[] SMCorrespondence;

	return sceneFittingScore;
}

// TODO
// Needs to be debugged - hyp->bboxCenter is [0.0, 0.0, 0.0]?
void DDDetector::FitBBoxToPlanarSurface(RECOG::DDD::Hypothesis *hyp, Surfel *pPlanarSurface)
{
	RECOG::DDD::Model *pModel = models.Element;
	float *R = hyp->pose.R;
	float xAxis[3];
	float correction;

	// RVLCOPY3VECTOR(R, xAxis);
	// xAxis[0] = R[0];
	// xAxis[1] = R[3];
	// xAxis[2] = R[6];

	// xAxis[0] = R[1];
	// xAxis[1] = R[4];
	// xAxis[2] = R[7];

	// xAxis[0] = R[2];
	// xAxis[1] = R[5];
	// xAxis[2] = R[8];

	xAxis[0] = pPlanarSurface->N[0];
	xAxis[1] = pPlanarSurface->N[1];
	xAxis[2] = pPlanarSurface->N[2];

	hyp->bboxSize[2] = pModel->bboxSize[2];
	float diff[3];
	RVLDIF3VECTORS(pPlanarSurface->P, hyp->bboxCenter, diff);
	correction = -RVLDOTPRODUCT3(xAxis, diff);

	float correctionVector[3];
	RVLCOPY3VECTOR(pPlanarSurface->N, correctionVector);
	RVLSCALE3VECTOR(correctionVector, correction, correctionVector);
	RVLSUM3VECTORS(hyp->pose.t, correctionVector, hyp->pose.t)
}

// void DDDetector::DetectROI(Array<Mesh> meshSeq, bool bVisualize)
//{
//	//Debug values
//	//this fcn should calculate ROI params
//	/*ROI.minx = -0.300;
//	ROI.maxx = 0.200;
//	ROI.miny = 0.150;
//	ROI.maxy = 0.400;
//	ROI.minz = 0.900;
//	ROI.maxz = 1.600;*/
//	//END Debug values
//
//	Mesh* pMeshM, * pMeshQ;
//	PointAssociationData pointAssociationData;
//	Array<int> ptBuff;
//	ptBuff.Element = new int[camera.w * camera.h];
//	Array<Point> pointsMQ;
//	pointsMQ.Element = new Point[camera.w * camera.h];
//	Array<OrientedPoint> pointsM, pointsQ;
//	Array<OrientedPoint> pointsMSubsampled, pointsMValid;
//	Array<bool> bPointsMValid;
//	pointsM.Element = new OrientedPoint[camera.w * camera.h];
//	pointsQ.Element = new OrientedPoint[camera.w * camera.h];
//	AffinePose3D pose;
//	float s[3] = { 1.0, 1.0, 1.0 };
//	float t[3] = { 0.0, 0.0, 0.0 };
//	float R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
//	RVLCOPY3VECTOR(s, pose.s);
//	RVLCOPY3VECTOR(t, pose.t);
//	RVLCOPYMX3X3(R, pose.R);
//
//	pMeshM = meshSeq.Element + meshSeq.n - 1;
//	CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
//	MeshSubsample(pMeshM, &pointsMSubsampled);
//	pMeshQ = meshSeq.Element + meshSeq.n - 1 - ROICalculationStep;
//	CreateOrientedPointArrayFromPointArray(pMeshQ->NodeArray, pointsQ);
//	pointAssociationData.Create(pointsMSubsampled.n, pointsQ.n, true);
//	imgGrid.Create(pMeshQ->NodeArray, &camera, ROIScene2SubsampleCellSize);
//	SampleImage(pMeshQ);
//	//PointAssociation(pointsM, &pose, pointsQ, pointAssociationData, ptBuff, true, &pointsMQ);
//	PointAssociation(pointsMSubsampled, &pose, pointsQ, pointAssociationData, ptBuff, true, &pointsMQ);
//
//	Array<Vector3<float>> pointsShift;
//	Voter3D voter;
//	float cellSize[3] = { votingCellSizeX, votingCellSizeY, votingCellSizeZ };
//	CalculatePointsShiftVector(pointsMSubsampled, pointsQ, &pointAssociationData, &pointsShift, minPointShift, &pointsMValid, &bPointsMValid);
//	voter.Vote(pointsShift, cellSize);
//	Vector3<float> dominantShift = voter.GetMax();
//
//	Array<OrientedPoint> selectedPoints;
//	float shiftThresh[3] = { shiftThreshX, shiftThreshY, shiftThreshZ };
//	FindPointsWithShift(pointsMSubsampled, &bPointsMValid, pointsQ, &pointAssociationData, &dominantShift, shiftThresh, &selectedPoints);
//
//	InitBoundingBox<float>(&ROI, selectedPoints.Element[0].P);
//	for (int iPt = 1; iPt < selectedPoints.n; iPt++)
//		UpdateBoundingBox<float>(&ROI, selectedPoints.Element[iPt].P);
//
//	if (bVisualize)
//	{
//		Visualizer* pVisualizer;
//		pVisualizer = pVisualizationData->pVisualizer;
//
//		uchar green[] = { 0.0, 255, 0.0 };
//		uchar blue[] = { 0.0, 0.0, 255.0 };
//		uchar red[] = { 255.0, 0.0, 0.0 };
//		uchar white[] = { 255.0, 255.0, 255.0 };
//
//		float meanPoint[3];
//		FindMeanPoint(&pointsMValid, meanPoint);
//
//		SetPointsForPointToPointAssociationVisualization(pointsMSubsampled, pMeshQ, &pointAssociationData);
//		SetShiftVectorForVisualization(meanPoint, dominantShift, blue);
//		//SetPointsForVisualization(pointsMSubsampled, blue, 1.0);
//		//SetPointsForVisualization(pointsQ, red, 1.0);
//		SetPointsForVisualization(pointsMValid, green, 2.0);
//		SetPointsForVisualization(selectedPoints, red, 2.0);
//
//		pVisualizer->Run();
//		pVisualizer->renderer->RemoveAllViewProps();
//	}
//
//	RVL_DELETE_ARRAY(ptBuff.Element);
//	RVL_DELETE_ARRAY(pointsMQ.Element);
//	RVL_DELETE_ARRAY(pointsM.Element);
//	RVL_DELETE_ARRAY(pointsQ.Element);
//	RVL_DELETE_ARRAY(pointsMSubsampled.Element);
//
//	RVL_DELETE_ARRAY(pointsShift.Element);
//	RVL_DELETE_ARRAY(pointsMValid.Element);
//	RVL_DELETE_ARRAY(bPointsMValid.Element);
//	RVL_DELETE_ARRAY(selectedPoints.Element);
// }

void DDDetector::DetectDominantShiftPoints(Array<Mesh> meshSeq, Array<int> *dominantShiftPointsIdx, bool bVisualize)
{
	// Debug values
	// this fcn should calculate ROI params
	/*ROI.minx = -0.300;
	ROI.maxx = 0.200;
	ROI.miny = 0.150;
	ROI.maxy = 0.400;
	ROI.minz = 0.900;
	ROI.maxz = 1.600;*/
	// END Debug values

	Mesh *pMeshM, *pMeshQ;
	PointAssociationData pointAssociationData;
	Array<int> ptBuff;
	ptBuff.Element = new int[camera.w * camera.h];
	Array<Point> pointsMQ;
	pointsMQ.Element = new Point[camera.w * camera.h];
	// Array<OrientedPoint> pointsM, pointsQ;
	Array<OrientedPoint> pointsQ;
	Array<OrientedPoint> pointsMSubsampled, pointsMValidShift, dominantShiftPoints;
	pointsMSubsampled.Element = NULL;
	pointsMValidShift.Element = NULL;
	dominantShiftPoints.Element = NULL;
	Array<int> pointsMSubsampledIdx, pointsMValidShiftIdx;
	// pointsM.Element = new OrientedPoint[camera.w * camera.h];
	pointsQ.Element = new OrientedPoint[camera.w * camera.h];
	AffinePose3D pose;
	float s[3] = {1.0, 1.0, 1.0};
	float t[3] = {0.0, 0.0, 0.0};
	float R[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	RVLCOPY3VECTOR(s, pose.s);
	RVLCOPY3VECTOR(t, pose.t);
	RVLCOPYMX3X3(R, pose.R);

	pMeshM = meshSeq.Element + meshSeq.n - 1;
#ifdef RVLLINUX
	// CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
	MeshSubsample(pMeshM, &pointsMSubsampledIdx);
	CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsMSubsampledIdx, pointsMSubsampled);
	pMeshQ = meshSeq.Element + meshSeq.n - 1 - ROICalculationStep;
	CreateOrientedPointArrayFromPointArray(pMeshQ->NodeArray, pointsQ);
#else
	// MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
	MeshSubsample(pMeshM, &pointsMSubsampledIdx);
	MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsMSubsampledIdx, pointsMSubsampled);
	pMeshQ = meshSeq.Element + meshSeq.n - 1 - ROICalculationStep;
	MESH::CreateOrientedPointArrayFromPointArray(pMeshQ->NodeArray, pointsQ);
#endif
	pointAssociationData.Create(pointsMSubsampled.n, pointsQ.n, true);
	imgGrid.Create(pMeshQ->NodeArray, &camera, ROIPointAssociationGridCellSize);
	// imgGrid.Create(pMeshQ->NodeArray, &camera, pointAssociationGridCellSize);
	SampleImage(pMeshQ);
	// PointAssociation(pointsM, &pose, pointsQ, pointAssociationData, ptBuff, true, &pointsMQ);
	PointAssociation(pointsMSubsampled, &pose, pointsQ, pointAssociationData, ptBuff, true, &pointsMQ);

	Array<Vector3<float>> pointsShift;
	Voter3D voter;
	float cellSize[3] = {votingCellSizeX, votingCellSizeY, votingCellSizeZ};
	CalculatePointsShiftVector(pMeshM->NodeArray, pointsMSubsampledIdx, pointsQ, &pointAssociationData, &pointsShift, minPointShift, &pointsMValidShiftIdx);
#ifdef RVLLINUX
	CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsMValidShiftIdx, pointsMValidShift);
#else
	MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsMValidShiftIdx, pointsMValidShift);
#endif
	voter.Vote(pointsShift, cellSize);
	Vector3<float> dominantShift = voter.GetMax();

	float shiftThresh[3] = {shiftThreshX, shiftThreshY, shiftThreshZ};
	FindPointsWithShift(pMeshM->NodeArray, pointsMSubsampledIdx, pointsQ, &pointAssociationData, &dominantShift, shiftThresh, dominantShiftPointsIdx);
#ifdef RVLLINUX
	CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, *dominantShiftPointsIdx, dominantShiftPoints);
#else
	MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, *dominantShiftPointsIdx, dominantShiftPoints);
#endif

	if (bVisualize && pVisualizationData->bVisualizeROIDetection)
	{
		Visualizer *pVisualizer;
		pVisualizer = pVisualizationData->pVisualizer;

		uchar green[] = {0, 255, 0};
		uchar blue[] = {0, 0, 255};
		uchar red[] = {255, 0, 0};
		uchar cyan[] = {0, 255, 255};
		uchar white[] = {255, 255, 255};

		float meanPoint[3];
		FindMeanPoint(&pointsMValidShift, meanPoint);

		SetPointsForPointToPointAssociationVisualization(pointsMSubsampled, pMeshQ, &pointAssociationData);
		SetShiftVectorForVisualization(meanPoint, dominantShift, blue);
		// SetPointsForVisualization(pointsMSubsampled, blue, 1.0);
		// SetPointsForVisualization(pointsQ, red, 1.0);
		SetPointsForVisualization(pointsMValidShift, blue, 2.0);
		SetPointsForVisualization(dominantShiftPoints, green, 2.0);

		pVisualizer->Run();
		pVisualizer->renderer->RemoveAllViewProps();
	}

	RVL_DELETE_ARRAY(ptBuff.Element);
	RVL_DELETE_ARRAY(pointsMQ.Element);
	// RVL_DELETE_ARRAY(pointsM.Element);
	RVL_DELETE_ARRAY(pointsQ.Element);
	RVL_DELETE_ARRAY(pointsMSubsampled.Element);
	RVL_DELETE_ARRAY(pointsMSubsampledIdx.Element);

	RVL_DELETE_ARRAY(pointsShift.Element);
	RVL_DELETE_ARRAY(pointsMValidShift.Element);
	RVL_DELETE_ARRAY(pointsMValidShiftIdx.Element);
	RVL_DELETE_ARRAY(dominantShiftPoints.Element);
}

void DDDetector::DetectDominantShiftPoints(Mesh *pMeshM, Mesh *pMeshQ, Array<int> *dominantShiftPointsIdx, Vector3<float> &dominantShift, bool bVisualize)
{
	// Debug values
	// this fcn should calculate ROI params
	/*ROI.minx = -0.300;
	ROI.maxx = 0.200;
	ROI.miny = 0.150;
	ROI.maxy = 0.400;
	ROI.minz = 0.900;
	ROI.maxz = 1.600;*/
	// END Debug values

	// Mesh* pMeshM, *pMeshQ;
	PointAssociationData pointAssociationData;
	Array<int> ptBuff;
	ptBuff.Element = new int[camera.w * camera.h];
	Array<Point> pointsMQ;
	pointsMQ.Element = new Point[camera.w * camera.h];
	// Array<OrientedPoint> pointsM, pointsQ;
	Array<OrientedPoint> pointsQ;
	Array<OrientedPoint> pointsMSubsampled, pointsMValidShift, dominantShiftPoints;
	pointsMSubsampled.Element = NULL;
	pointsMValidShift.Element = NULL;
	dominantShiftPoints.Element = NULL;
	Array<int> pointsMSubsampledIdx, pointsMValidShiftIdx;
	// pointsM.Element = new OrientedPoint[camera.w * camera.h];
	pointsQ.Element = new OrientedPoint[camera.w * camera.h];
	AffinePose3D pose;
	float s[3] = {1.0, 1.0, 1.0};
	float t[3] = {0.0, 0.0, 0.0};
	float R[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	RVLCOPY3VECTOR(s, pose.s);
	RVLCOPY3VECTOR(t, pose.t);
	RVLCOPYMX3X3(R, pose.R);

// pMeshM = meshSeq.Element + meshSeq.n - 1;
#ifdef RVLLINUX
	// CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
	MeshSubsample(pMeshM, &pointsMSubsampledIdx);
	CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsMSubsampledIdx, pointsMSubsampled);
	// pMeshQ = meshSeq.Element + meshSeq.n - 1 - ROICalculationStep;
	CreateOrientedPointArrayFromPointArray(pMeshQ->NodeArray, pointsQ);
#else
	// MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsM);
	MeshSubsample(pMeshM, &pointsMSubsampledIdx);
	MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsMSubsampledIdx, pointsMSubsampled);
	// pMeshQ = meshSeq.Element + meshSeq.n - 1 - ROICalculationStep;
	MESH::CreateOrientedPointArrayFromPointArray(pMeshQ->NodeArray, pointsQ);
#endif
	pointAssociationData.Create(pointsMSubsampled.n, pointsQ.n, true);
	imgGrid.Create(pMeshQ->NodeArray, &camera, ROIPointAssociationGridCellSize);
	// imgGrid.Create(pMeshQ->NodeArray, &camera, pointAssociationGridCellSize);
	SampleImage(pMeshQ);
	// PointAssociation(pointsM, &pose, pointsQ, pointAssociationData, ptBuff, true, &pointsMQ);
	PointAssociation(pointsMSubsampled, &pose, pointsQ, pointAssociationData, ptBuff, true, &pointsMQ);

	Array<Vector3<float>> pointsShift;
	Voter3D voter;
	float cellSize[3] = {votingCellSizeX, votingCellSizeY, votingCellSizeZ};
	CalculatePointsShiftVector(pMeshM->NodeArray, pointsMSubsampledIdx, pointsQ, &pointAssociationData, &pointsShift, minPointShift, &pointsMValidShiftIdx);
#ifdef RVLLINUX
	CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsMValidShiftIdx, pointsMValidShift);
#else
	MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, pointsMValidShiftIdx, pointsMValidShift);
#endif

	if (voter.Vote(pointsShift, cellSize))
	{
		dominantShift = voter.GetMax();

		float shiftThresh[3] = {shiftThreshX, shiftThreshY, shiftThreshZ};
		FindPointsWithShift(pMeshM->NodeArray, pointsMSubsampledIdx, pointsQ, &pointAssociationData, &dominantShift, shiftThresh, dominantShiftPointsIdx);

#ifdef RVLLINUX
		CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, *dominantShiftPointsIdx, dominantShiftPoints);
#else
		MESH::CreateOrientedPointArrayFromPointArray(pMeshM->NodeArray, *dominantShiftPointsIdx, dominantShiftPoints);
#endif
		// Visualization.
		
		if (bVisualize && pVisualizationData->bVisualizeROIDetection)
		{
			Visualizer *pVisualizer;
			pVisualizer = pVisualizationData->pVisualizer;

			uchar green[] = {0, 255, 0};
			uchar blue[] = {0, 0, 255};
			uchar red[] = {255, 0, 0};
			uchar cyan[] = {0, 255, 255};
			uchar white[] = {255, 255, 255};

			float meanPoint[3];
			FindMeanPoint(&pointsMValidShift, meanPoint);

			SetPointsForPointToPointAssociationVisualization(pointsMSubsampled, pMeshQ, &pointAssociationData);
			SetShiftVectorForVisualization(meanPoint, dominantShift, blue);
			// SetPointsForVisualization(pointsMSubsampled, blue, 1.0);
			// SetPointsForVisualization(pointsQ, red, 1.0);
			SetPointsForVisualization(pointsMValidShift, blue, 2.0);
			SetPointsForVisualization(dominantShiftPoints, green, 6.0);

			pVisualizer->Run();
			pVisualizer->renderer->RemoveAllViewProps();
		}
	}

	RVL_DELETE_ARRAY(ptBuff.Element);
	RVL_DELETE_ARRAY(pointsMQ.Element);
	// RVL_DELETE_ARRAY(pointsM.Element);
	RVL_DELETE_ARRAY(pointsQ.Element);
	RVL_DELETE_ARRAY(pointsMSubsampled.Element);
	RVL_DELETE_ARRAY(pointsMSubsampledIdx.Element);

	RVL_DELETE_ARRAY(pointsShift.Element);
	RVL_DELETE_ARRAY(pointsMValidShift.Element);
	RVL_DELETE_ARRAY(pointsMValidShiftIdx.Element);
	RVL_DELETE_ARRAY(dominantShiftPoints.Element);
}

void DDDetector::MeshSubsample(Mesh *pMesh, Array<int> *pSubsampledPointsIdx)
{
	Grid SubsamplingGrid;
	SubsamplingGrid.Create(pMesh->NodeArray, &camera, ROISceneSubsampleCellSize);
	SubsamplingGrid.SubSample(pMesh->NodeArray, ROISceneSubsampleNormalSimilarity, *pSubsampledPointsIdx);
}

void DDDetector::CalculatePointsShiftVector(
	Array<Point> pointsM,
	Array<int> subsampledPointsMIdx,
	Array<OrientedPoint> pointsQ,
	PointAssociationData *pPointAssociationData,
	Array<RVL::Vector3<float>> *pointsShift,
	float shiftThresh,
	Array<int> *validShiftedPointsMIdx)
{
	pointsShift->Element = new Vector3<float>[pointsM.n];
	validShiftedPointsMIdx->Element = new int[pointsM.n];

	int iMPt, iQPt;
	float *pMPt, *pQPt, *pPt;
	int nPoints = 0;
	float diff[3];
	float len;

	for (int i = 0; i < subsampledPointsMIdx.n; i++)
	{
		iMPt = subsampledPointsMIdx.Element[i];

		pMPt = (pointsM.Element + iMPt)->P;
		iQPt = pPointAssociationData->QNN[i];
		if (iQPt >= 0)
		{
			pQPt = (pointsQ.Element + iQPt)->P;

			diff[0] = pQPt[0] - pMPt[0];
			diff[1] = pQPt[1] - pMPt[1];
			diff[2] = pQPt[2] - pMPt[2];

			len = sqrt(RVLDOTPRODUCT3(diff, diff));

			if (len < shiftThresh)
			{
				continue;
			}
			else
			{
				pointsShift->Element[nPoints].Element[0] = diff[0];
				pointsShift->Element[nPoints].Element[1] = diff[1];
				pointsShift->Element[nPoints].Element[2] = diff[2];

				validShiftedPointsMIdx->Element[nPoints] = iMPt;

				nPoints++;
			}
		}
	}

	pointsShift->n = nPoints;
	validShiftedPointsMIdx->n = nPoints;
}

void DDDetector::FindPointsWithShift(
	Array<Point> pointsM,
	Array<int> validPointsMIdx,
	Array<OrientedPoint> pointsQ,
	PointAssociationData *pPointAssociationData,
	Vector3<float> *pointsShift,
	float *shiftThresh,
	Array<int> *pointsIdx)
{
	int iMPt, iQPt;
	float *pMPt, *pQPt;
	float shift[3];
	int nValid = 0;
	OrientedPoint *pPt;

	if (pointsIdx->Element)
		RVL_DELETE_ARRAY(pointsIdx->Element);

	pointsIdx->Element = new int[pointsM.n];
	pointsIdx->n = 0;

	RVLSCALE3VECTOR(shiftThresh, 0.5, shiftThresh);
	float lowerThresh[3] = {pointsShift->Element[0] - shiftThresh[0], pointsShift->Element[1] - shiftThresh[1], pointsShift->Element[2] - shiftThresh[2]};
	float upperThresh[3] = {pointsShift->Element[0] + shiftThresh[0], pointsShift->Element[1] + shiftThresh[1], pointsShift->Element[2] + shiftThresh[2]};

	for (int i = 0; i < validPointsMIdx.n; i++)
	{
		iMPt = validPointsMIdx.Element[i];

		pMPt = (pointsM.Element + iMPt)->P;
		iQPt = pPointAssociationData->QNN[i];

		if (iQPt >= 0)
		{
			pQPt = (pointsQ.Element + iQPt)->P;
			shift[0] = (pQPt[0] - pMPt[0]);
			shift[1] = (pQPt[1] - pMPt[1]);
			shift[2] = (pQPt[2] - pMPt[2]);

			if ((lowerThresh[0] < shift[0]) && (shift[0] < upperThresh[0]))
				if ((lowerThresh[1] < shift[1]) && (shift[1] < upperThresh[1]))
					if ((lowerThresh[2] < shift[2]) && (shift[2] < upperThresh[2]))
					{
						pointsIdx->Element[nValid] = iMPt;
						nValid++;
					}
		}
	}

	pointsIdx->n = nValid;
}

void ::DDDetector::CalculateROI(Mesh *pMesh, Array<int> pointsIdx, Box<float> *ROI)
{
	int iPt = pointsIdx.Element[0];
	Point *pPt = pMesh->NodeArray.Element + iPt;

	InitBoundingBox<float>(ROI, pPt->P);
	for (int iPt = 1; iPt < pointsIdx.n; iPt++)
	{
		pPt = pMesh->NodeArray.Element + iPt;
		UpdateBoundingBox<float>(ROI, pPt->P);
	}
}

void DDDetector::FindMeanPoint(Array<OrientedPoint> *points, float *meanPoint)
{
	float minValue[3];
	float maxValue[3];
	float diff[3];

	minValue[0] = FLT_MAX;
	minValue[1] = FLT_MAX;
	minValue[2] = FLT_MAX;
	maxValue[0] = FLT_MIN;
	maxValue[1] = FLT_MIN;
	maxValue[2] = FLT_MIN;

	OrientedPoint *pPt;

	for (int iPt = 0; iPt < points->n; iPt++)
	{
		pPt = points->Element + iPt;

		for (int i = 0; i < 3; i++)
		{
			if (pPt->P[i] < minValue[i])
				minValue[i] = pPt->P[i];
			if (pPt->P[i] > maxValue[i])
				maxValue[i] = pPt->P[i];
		}
	}

	RVLDIF3VECTORS(maxValue, minValue, diff);
	RVLSCALE3VECTOR(diff, 0.5, diff);
	RVLSUM3VECTORS(minValue, diff, meanPoint);
}

//***************************************************//
// Mapping to notations on Fig. 3 in COSPER.TR4.5-B.pdf
// M=A; Q=B
//***************************************************//
void DDDetector::RecognizeArticulatedObjectState(
	Mesh *pMesh,
	RECOG::DDD::ArticulatedObject AObj,
	Pose3D &poseOC,
	cv::Mat *pRGBImg,
	bool bPrintDebug)
{
	// Paremeters.

	int minSurfelSize = 1000;
	int minSurfelPtsInROI = 1000;
	float ROILength = 1.1;
	float ROIClosedLength = 0.10;
	float openingDirectionTolRad = openingDirectionTol * DEG2RAD;
	bool bAlignWithSurfel = true;
	bool bDebug = false;
	float surfOrientTol = 30.0;   // deg
	float surfPositionTol = 0.05; // m

	// Constants.

	float surfOrientTolRad = surfOrientTol * DEG2RAD;
	float snJointAxisSurfNormalAngleTol = sin(0.5f * PI - surfOrientTolRad);
	float csMaxSurfelAngle = cos(surfOrientTolRad);

	//

	bool bInROI;
	bool bInROIClosed;
	AffinePose3D ROI_;
	AffinePose3D ROIClosed_;
	Box<float> ROI;
	Box<float> ROIClosed;
	Pose3D poseAOC;
	Pose3D ROIPose;
	Pose3D ROIClosedPose;
	float PROI[3];
	float PROIClosed[3];
	float V3Tmp[3];
	RECOG::DDD::HypothesisDoorDrawer *pAObj;
	RVL::Surfel *pSurfel;
	float diff[3];
	float normalM[3];
	float tmp[3];
	float tBA[3];
	tBA[2] = 0.0f;

	Pose3D poseQC, poseMO, poseQM, poseQO, poseMC, poseMC_, poseOC_;
	RVLUNITMX3(poseQM.R);
	RVLNULL3VECTOR(poseQM.t);
	AffinePose3D APoseQC;
	float score;
	float bestScore;
	float q;
	bool bPotentialSurfelFound;
	bool bAObjFound;
	bool bBack;
	float snJointAxisSurfNormalAngle;

	int *pSurfelPointsInROI = new int[planarSurfaces.NodeArray.n];

	RECOG::DDD::Model cuboidModel;
	float cuboidSize[] = {0.018f, 0.4f, 0.4f};
	CreateCuboidModel2(cuboidSize, 1.0f / 0.02f, &cuboidModel);
	RECOG::DDD::AOHypothesisState *states = new RECOG::DDD::AOHypothesisState[AObj.movingParts.n];
	float RCM_[9];
	float *XMC_ = RCM_;
	float *YMC_ = RCM_ + 3;
	float *ZMC_ = RCM_ + 6;
	float fTmp;

	for (int iAObj = 0; iAObj < AObj.movingParts.n; iAObj++)
	{
		printf("\nHypothesis evaluation for AO %d...\n", iAObj);
		pAObj = AObj.movingParts.Element + iAObj;
		pAObj->state.n = 1;
		pAObj->state.Element = states + iAObj;
		bestScore = 0.0;
		bAObjFound = false;

		CreateAOROI(pMesh, pAObj, poseOC, &ROI, ROI_, ROIPose);
		memset(pSurfelPointsInROI, 0, planarSurfaces.NodeArray.n * sizeof(int));
		CalculateSurfelPointsInsideROI(pMesh, &ROI, ROIPose, pSurfelPointsInROI);

		if (pVisualizationData->bVisualizeROI)
			VisualizeAOROI(pMesh, ROI_, ROIPose);

		// Hypothesis for closed door/drawer
		if (bPrintDebug)
			printf("  Evaluating hypothesis for closed AO...\n");
		q = 0.0f;
		score = CreateAndEvaluateAOHypothesis(pMesh, pAObj, poseOC, q, bPrintDebug);

		if (score > bestScore)
		{
			bAObjFound = true;
			bestScore = score;
			pAObj->state.Element[0].q = q;
			pAObj->state.Element[0].score = score;
			pAObj->state.Element[0].imgID = 0;
		}

		RVLCOPYMX3X3(pAObj->pose.R, poseMO.R);
		RVLCOPY3VECTOR(pAObj->pose.t, poseMO.t);
		Pose3D poseOM;
		RVLINVTRANSF3D(poseMO.R, poseMO.t, poseOM.R, poseOM.t);
		RVLCOMPTRANSF3D(poseOC.R, poseOC.t, poseMO.R, poseMO.t, poseMC.R, poseMC.t);

		for (int iSurfel = 0; iSurfel < planarSurfaces.NodeArray.n; iSurfel++)
		{
			if (pSurfelPointsInROI[iSurfel] < minSurfelPtsInROI)
				continue;

			bPotentialSurfelFound = false;
			pSurfel = planarSurfaces.NodeArray.Element + iSurfel;

			if (pSurfel->bEdge)
				continue;

			if (bPrintDebug)
				printf("  Evaluating hypothesis generated from surfel %d...\n", iSurfel);

			if (pAObj->objClass == RVLDDD_MODEL_DOOR)
			{
				RVLCOPYCOLMX3X3(poseMC.R, 2, ZMC_);
				RVLCROSSPRODUCT3(ZMC_, pSurfel->N, V3Tmp);
				RVLNORM3(V3Tmp, snJointAxisSurfNormalAngle);
				if (snJointAxisSurfNormalAngle < snJointAxisSurfNormalAngleTol)
				{
					if (bPrintDebug)
						printf("  Surfel normal is not perpendicular to the joint axis. Hypothesis is rejected.\n");
					continue;
				}
				if (bAlignWithSurfel)
				{
					RVLCROSSPRODUCT3(pSurfel->N, V3Tmp, ZMC_);
					RVLCOPYCOLMX3X3(poseMC.R, 0, XMC_);
					RVLCROSSPRODUCT3(ZMC_, XMC_, YMC_);
					RVLNORM3(YMC_, fTmp);
					RVLCROSSPRODUCT3(YMC_, ZMC_, XMC_);
					RVLCOPYMX3X3T(RCM_, poseMC_.R);
				}
				else
					poseMC_ = poseMC;
				RVLMULMX3X3TVECT(poseMC_.R, pSurfel->N, normalM);
				q = atan2(normalM[1], normalM[0]);
				if (bBack = (pAObj->openingDirection * q < -openingDirectionTolRad))
					q += (q >= 0 ? -PI : PI);
				if (bPrintDebug)
					printf("  Hypothesis angle is: %f RAD\t%f DEG\n", q, q * 180 / PI);
				if (bAlignWithSurfel)
				{
					float XBC[3];
					float d;
					if (bBack)
					{
						RVLNEGVECT3(pSurfel->N, XBC);
						d = -pSurfel->d + 0.5f * frontFaceThickness;
					}
					else
					{
						RVLCOPY3VECTOR(pSurfel->N, XBC);
						d = pSurfel->d - 0.5f * frontFaceThickness;
					}
					float e = RVLDOTPRODUCT3(XBC, poseMC.t) - d + pAObj->r[0];
					if (RVLABS(e) > surfPositionTol)
					{
						if (bPrintDebug)
							printf("  Surfel is too distant from the joint axis. Hypothesis is rejected.\n");
						continue;
					}
					RVLSCALE3VECTOR(XBC, -e, V3Tmp);
					RVLSUM3VECTORS(poseMC.t, V3Tmp, poseMC_.t);
					RVLCOMPTRANSF3D(poseMC_.R, poseMC_.t, poseOM.R, poseOM.t, poseOC_.R, poseOC_.t);
				}
				else
					poseOC_ = poseOC;
				bPotentialSurfelFound = true;
			}
			else if (pAObj->objClass == RVLDDD_MODEL_DRAWER)
			{
				RVLCOPYCOLMX3X3(poseMC.R, 0, XMC_);
				if (RVLDOTPRODUCT3(XMC_, pSurfel->N) > csMaxSurfelAngle)
				{
					if (bPrintDebug)
						printf("  Evaluating hypothesis generated from surfel %d...\n", iSurfel);
					RVLUNITMX3(poseQM.R);
					RVLNULL3VECTOR(poseQM.t);

					RVLCOPYMX3X3(pAObj->pose.R, poseMO.R);
					RVLCOPY3VECTOR(pAObj->pose.t, poseMO.t);

					RVLCOMPTRANSF3D(poseOC.R, poseOC.t, poseMO.R, poseMO.t, poseMC.R, poseMC.t);
					RVLINVTRANSF3(pSurfel->P, poseMC.R, poseMC.t, diff, tmp);

					q = diff[0];
					if (bPrintDebug)
						printf("  Hypothesis distance is: %f m\n", q);
					if (q >= -surfPositionTol)
						bPotentialSurfelFound = true;
					else if (bPrintDebug)
						printf("  Hypothesis rejected.\n");
					poseOC_ = poseOC;
				}
			}
			if (bPotentialSurfelFound)
			{
				score = CreateAndEvaluateAOHypothesis(pMesh, pAObj, poseOC_, q, bPrintDebug);

				if (score > bestScore)
				{
					bAObjFound = true;
					bestScore = score;
					pAObj->state.Element[0].q = q;
					pAObj->state.Element[0].imgID = 0;
					pAObj->state.Element[0].score = score;
				}
			}
		}

		if (!bAObjFound)
		{
			score = CreateAndEvaluateDefaultAOHypothesis(pMesh, pAObj, poseOC, 0);
		}

		if (pAObj->objClass == RVLDDD_MODEL_DOOR)
			// printf("State of the AO %d is: %f RAD\t%f DEG\n", iAObj + 1, pAObj->state.Element[0].q, pAObj->state.Element[0].q * 180 / PI);
			printf("State of the AO %d is: %f DEG\n", iAObj + 1, pAObj->state.Element[0].q * RAD2DEG);
		else if (pAObj->objClass == RVLDDD_MODEL_DRAWER)
			// printf("State of the AO %d is: %f m\n", iAObj + 1, pAObj->state.Element[0].q);
			printf("State of the AO %d is: %f cm\n", iAObj + 1, 100.0f * pAObj->state.Element[0].q);
	}
	delete[] pSurfelPointsInROI;
	printf("\nVisualization of AOs in detected state...\n");
	if (GetRGBImageVisualization() && pRGBImg)
	{
		cv::Mat display;
		pRGBImg->copyTo(display);
		VisualizeArticulatedObject(AObj, poseOC, false, &display);
		cv::imshow("Articulated object", display);
		cv::waitKey();
	}
	if (pVisualizationData->b3DVisualization)
	{
		GetVisualizer()->SetMesh(pMesh);
		VisualizeArticulatedObject(AObj, poseOC);
		RunVisualizer();
		ClearVisualization();
	}

	delete[] states;
}

void DDDetector::CreateAOROI(
	Mesh *pMesh,
	RECOG::DDD::HypothesisDoorDrawer *pAObj,
	Pose3D &poseOC,
	Box<float> *ROI,
	AffinePose3D &ROI_,
	Pose3D &ROIPose)
{
	float ROILength = 1.1;

	float xAxis[3];
	float tBA[3];
	tBA[2] = 0.0f;
	float moveROI[3];

	Pose3D poseQC, poseMO, poseQM, poseQO, poseMC, poseMC_;
	RVLUNITMX3(poseQM.R);
	RVLNULL3VECTOR(poseQM.t);

	Pose3D poseAOC;

	RVLCOPYMX3X3(pAObj->pose.R, poseMO.R);
	RVLCOPY3VECTOR(pAObj->pose.t, poseMO.t);
	RVLCOMPTRANSF3D(poseOC.R, poseOC.t, poseMO.R, poseMO.t, poseMC.R, poseMC.t);

	if (pAObj->objClass == RVLDDD_MODEL_DOOR)
	{
		tBA[0] = pAObj->r[0];
		tBA[1] = pAObj->r[1];

		RVLCOPYMX3X3(poseMC.R, poseMC_.R);
		RVLTRANSF3(tBA, poseMC.R, poseMC.t, poseMC_.t);
	}
	else if (pAObj->objClass == RVLDDD_MODEL_DRAWER)
	{
		xAxis[0] = pAObj->pose.R[0];
		xAxis[1] = pAObj->pose.R[3];
		xAxis[2] = pAObj->pose.R[6];

		RVLCOPYMX3X3(poseMC.R, poseMC_.R);
		RVLCOPY3VECTOR(poseMC.t, poseMC_.t);
	}

	RVLCOPYMX3X3(poseMC_.R, ROI_.R);
	RVLCOPYMX3X3(poseMC_.R, ROIPose.R);
	RVLCOPY3VECTOR(poseMC_.t, ROI_.t);
	RVLCOPY3VECTOR(ROI_.t, ROIPose.t);

	ROI_.s[0] = ROILength;
	ROI_.s[1] = pAObj->s[0] * 1.1;
	ROI_.s[2] = pAObj->s[1] * 1.1;

	ROI->minx = -0.5f * ROI_.s[0];
	ROI->maxx = 0.5f * ROI_.s[0];
	ROI->miny = -0.5f * ROI_.s[1];
	ROI->maxy = 0.5f * ROI_.s[1];
	ROI->minz = -0.5f * ROI_.s[2];
	ROI->maxz = 0.5f * ROI_.s[2];
}

void DDDetector::VisualizeAOROI(
	Mesh *pMesh,
	AffinePose3D &ROI_,
	Pose3D &ROIPose)
{
	{
		pVisualizationData->pVisualizer->SetMesh(pMesh);
		pVisualizationData->pVisualizer->DisplayBox(ROI_.s[0], ROI_.s[1], ROI_.s[2], &ROIPose, 0.0, 255.0, 0.0);

		pVisualizationData->pVisualizer->Run();
		pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
	}
}

float DDDetector::CreateAndEvaluateAOHypothesis(
	Mesh *pMesh,
	RECOG::DDD::HypothesisDoorDrawer *pAObj,
	Pose3D &poseOC,
	float q,
	bool bPrintDebug)
{
	// Parameters.

	float stdZeroStateDoor = 5.0f;   // deg
	float stdZeroStateDrawer = 0.05; // m

	// Constants.

	float varZeroStateDoor = stdZeroStateDoor * stdZeroStateDoor * DEG2RAD * DEG2RAD;
	float varZeroStateDrawer = stdZeroStateDrawer * stdZeroStateDrawer;

	//

	Pose3D poseQM, poseMO, poseQO, poseQC;
	RVLUNITMX3(poseQM.R);
	RVLNULL3VECTOR(poseQM.t);

	AffinePose3D APoseQC;
	float score;

	RVLCOPYMX3X3(pAObj->pose.R, poseMO.R);
	RVLCOPY3VECTOR(pAObj->pose.t, poseMO.t);

	RECOG::DDD::Model cuboidModel;
	float cuboidModelThickness = 0.018f;
	float cuboidSize[] = {cuboidModelThickness, 0.4f, 0.4f};
	CreateCuboidModel2(cuboidSize, 1.0f / 0.02f, &cuboidModel);

	if (pAObj->objClass == RVLDDD_MODEL_DOOR)
	{
		float angle = q;
		float cs = cos(angle);
		float sn = sin(angle);
		RVLROTZ(cs, sn, poseQM.R);

		float tBB_[3];
		RVLSET3VECTOR(tBB_, pAObj->r[0], pAObj->r[1], 0.0f);
		RVLMULMX3X3VECT(poseQM.R, tBB_, poseQM.t);
	}
	else if (pAObj->objClass == RVLDDD_MODEL_DRAWER)
	{
		poseQM.t[0] = q;
	}

	RVLCOMPTRANSF3D(poseMO.R, poseMO.t, poseQM.R, poseQM.t, poseQO.R, poseQO.t);
	RVLCOMPTRANSF3D(poseOC.R, poseOC.t, poseQO.R, poseQO.t, poseQC.R, poseQC.t);

	RVLSET3VECTOR(APoseQC.s, cuboidSize[0], pAObj->s[0], pAObj->s[1]);

	RVLCOPYMX3X3(poseQC.R, APoseQC.R);
	RVLCOPY3VECTOR(poseQC.t, APoseQC.t);

	score = EvaluateHypothesis(pMesh, &cuboidModel, APoseQC, bPrintDebug);
	score *= (1.0f + exp(-q * q / (pAObj->objClass == RVLDDD_MODEL_DOOR ? varZeroStateDoor : varZeroStateDrawer)));
	return score;
}

// DOOR - hypothesis which lies on camera optical axis
// DRAWER - closed drawer
float DDDetector::CreateAndEvaluateDefaultAOHypothesis(
	Mesh *pMesh,
	RECOG::DDD::HypothesisDoorDrawer *pAObj,
	Pose3D &poseOC,
	bool bPrintDebug)
{
	float q;
	float score;
	float normalM[3];
	Pose3D poseMO, poseMC;

	if (pAObj->objClass == RVLDDD_MODEL_DOOR)
	{
		float openingDirectionTolRad = openingDirectionTol * DEG2RAD;

		RVLCOPYMX3X3(pAObj->pose.R, poseMO.R);
		RVLCOPY3VECTOR(pAObj->pose.t, poseMO.t);

		RVLCOMPTRANSF3D(poseOC.R, poseOC.t, poseMO.R, poseMO.t, poseMC.R, poseMC.t);

		float zAxis[3];
		zAxis[0] = pAObj->pose.R[2];
		zAxis[1] = pAObj->pose.R[5];
		zAxis[2] = pAObj->pose.R[8];

		float normal[3];
		RVLCROSSPRODUCT3(poseMC.t, zAxis, normal);
		RVLMULMX3X3TVECT(poseMC.R, normal, normalM);

		float angle_ = atan2(normalM[1], normalM[0]);
		if (pAObj->openingDirection * angle_ < -openingDirectionTolRad)
			angle_ += (angle_ >= 0 ? -PI : PI);
		q = angle_;
		if (bPrintDebug)
			if (bPrintDebug)
				printf("  Hypothesis angle is: %f RAD\t%f DEG\n", q, q * 180 / PI);
	}
	else if (pAObj->objClass == RVLDDD_MODEL_DRAWER)
	{
		q = 0.0f;
		if (bPrintDebug)
			printf("  Hypothesis distance is: %f m\n", q);
	}

	score = CreateAndEvaluateAOHypothesis(pMesh, pAObj, poseOC, q, bPrintDebug);

	pAObj->state.Element[0].q = q;
	pAObj->state.Element[0].score = score;
	pAObj->state.Element[0].imgID = 0;
	return score;
}

void DDDetector::CalculateSurfelPointsInsideROI(
	Mesh *pMesh,
	Box<float> *ROI,
	Pose3D &ROIPose,
	int *pSurfelPointsInROI)
{
	int iPoint;
	Point *pPoint;
	float PROI[3];
	float V3Tmp[3];
	bool bInROI;
	int iSurfel;

	for (iPoint = 0; iPoint < pMesh->NodeArray.n; iPoint++)
	{
		pPoint = pMesh->NodeArray.Element + iPoint;

		RVLINVTRANSF3(pPoint->P, ROIPose.R, ROIPose.t, PROI, V3Tmp);
		bInROI = InBoundingBox<float>(ROI, PROI);

		if (bInROI)
		{
			iSurfel = planarSurfaces.surfelMap[iPoint];
			if (iSurfel != -1)
			{
				pSurfelPointsInROI[iSurfel]++;
			}
		}
	}
}

void DDDetector::LoadIRITransformationsFromYAML(
	std::string path,
	double *R,
	double *t)
{
	cv::Mat position, orientation;
	cv::FileStorage fs;

	fs.open(path, cv::FileStorage::READ);
	fs["position"] >> position;
	fs["orientation"] >> orientation;

	double quaternion_[4];
	double quaternion[4];
	std::memcpy(t, position.ptr(0), position.total() * sizeof(double));
	std::memcpy(quaternion_, orientation.ptr(0), orientation.total() * sizeof(double));

	// quaternion[0] = quaternion_[3];
	// quaternion[1] = quaternion_[0];
	// quaternion[2] = quaternion_[1];
	// quaternion[3] = quaternion_[2];

	quaternion[0] = quaternion_[0];
	quaternion[1] = quaternion_[1];
	quaternion[2] = quaternion_[2];
	quaternion[3] = quaternion_[3];

	RVLQUATERNIONTOROT(quaternion, R);

	// if (	if(std::string(path).find("models") != std::string::npos) || std::string(path).find("fridge") != std::string::npos)
	if (std::string(path).find("models") != std::string::npos && (std::string(path).find("kitchen") != std::string::npos || std::string(path).find("night") != std::string::npos || std::string(path).find("door") != std::string::npos))
	{
		float tCorr[] = {-2.819f, 0.092f, -0.001f};
		RVLSUM3VECTORS(t, tCorr, t);
	}
}

void DDDetector::LoadGTTransformations(
	char *modelFilePath,
	RECOG::DDD::Transformation &transformation)
{
	cv::Mat position, orientation;
	std::string transformationsFilePath = modelFilePath;
	// transformationsFilePath.append("\\pose_cameralink_map.yaml");
	transformationsFilePath.append("\\pose_camera_depth_optical_frame_to_map.yaml");

	LoadIRITransformationsFromYAML(transformationsFilePath, transformation.R, transformation.t);

#ifdef NEVER
	cv::FileStorage fs;
	fs.open(transformationsFilePath, cv::FileStorage::READ);

	fs["position"] >> position;
	fs["orientation"] >> orientation;

	double t[3];
	double quaternion_[4];
	double quaternion[4];
	double R[9];

	std::memcpy(t, position.ptr(0), position.total() * sizeof(double));
	std::memcpy(quaternion_, orientation.ptr(0), orientation.total() * sizeof(double));

	// currently quaternion is stored as [x, y, z, w]
	// this code transforms it to [w, x, y, z]
	quaternion[0] = quaternion_[3];
	quaternion[1] = quaternion_[0];
	quaternion[2] = quaternion_[1];
	quaternion[3] = quaternion_[2];

	RVLQUATERNIONTOROT(quaternion, R);

	fs.release();

	double t2[3];
	double R2[9];

	LoadIRITransformationsFromYAML(transformationsFilePath, R2, t2);
#endif
}

void DDDetector::SelectAOModel(
	RECOG::DDD::Transformation *pSceneTransformation,
	Array<RECOG::DDD::Transformation> modelTransformations,
	Array<RECOG::DDD::ArticulatedObject> modelAOs,
	RECOG::DDD::ArticulatedObject &AObj,
	Array<int> &models)
{
	// Parameters.

	float FOVTol = 100.0f;

	//

	float AOPt_Camera[3];
	float AOPt_Map[3];
	float scenePt_Camera[3];
	float temp[3];
	float u, v;
	float minDistance = 1e38;
	float dist;
	int iBestAOModel = -1;
	RECOG::DDD::Transformation *pModelTransformation;
	RECOG::DDD::ArticulatedObject *pModelAO;
	models.n = 0;

	for (int iModel = 0; iModel < modelAOs.n; iModel++)
	{
		pModelTransformation = modelTransformations.Element + iModel;
		pModelAO = modelAOs.Element + iModel;

		for (int iMovingPart = 0; iMovingPart < pModelAO->movingParts.n; iMovingPart++)
		{
			RVLCOPY3VECTOR(pModelAO->movingParts.Element[iMovingPart].pose.t, AOPt_Camera);

			RVLTRANSF3(AOPt_Camera, pModelTransformation->R, pModelTransformation->t, AOPt_Map);
			RVLINVTRANSF3(AOPt_Map, pSceneTransformation->R, pSceneTransformation->t, scenePt_Camera, temp);

			if (scenePt_Camera[2] > 0.0f)
			{
				// u = camera.fu * scenePt_Camera[0] / scenePt_Camera[2] + camera.uc;
				// v = camera.fv * scenePt_Camera[1] / scenePt_Camera[2] + camera.vc;
				u = camera.fu * scenePt_Camera[0] / scenePt_Camera[2];
				v = camera.fv * scenePt_Camera[1] / scenePt_Camera[2];
				dist = u * u + v * v;
				// dist = RVLABS(u);

				// if ((u >= -FOVTol && u < camera.w + FOVTol) && (v >= -FOVTol && v < camera.h + FOVTol))
				//{
				//	if (scenePt_Camera[2] < minDistance)
				//	{
				//		AObj = *pModelAO;
				//		minDistance = scenePt_Camera[2];
				//		iBestAOModel = iModel;
				//	}
				// }

				// dist = RVLDOTPRODUCT3(scenePt_Camera, scenePt_Camera);
				// dist = scenePt_Camera[0] * scenePt_Camera[0] + scenePt_Camera[1] * scenePt_Camera[1];
				if (dist < minDistance)
				{
					AObj = *pModelAO;
					minDistance = dist;
					iBestAOModel = iModel;
				}
			}
		}
	}

	if (iBestAOModel >= 0)
		models.Element[models.n++] = iBestAOModel;
}

void DDDetector::SetPointsForPointToPointAssociationVisualization(Mesh *pMeshM, Mesh *pMeshQ, PointAssociationData *pPointAssociationData)
{
	uchar green[] = {0, 255, 0};
	uchar blue[] = {0, 0, 255};
	uchar red[] = {255, 0, 0};

	pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pMeshM->NodeArray, green, 0.2);
	pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pMeshQ->NodeArray, blue, 0.2);

	if (pVisualizationData->AssociatedPts.Element)
		RVL_DELETE_ARRAY(pVisualizationData->AssociatedPts.Element);
	pVisualizationData->AssociatedPts.n = pMeshM->NodeArray.n + pMeshQ->NodeArray.n;
	pVisualizationData->AssociatedPts.Element = new Point[pVisualizationData->AssociatedPts.n];

	memcpy(pVisualizationData->AssociatedPts.Element, pMeshM->NodeArray.Element, pMeshM->NodeArray.n * sizeof(Point));
	memcpy(pVisualizationData->AssociatedPts.Element + pMeshM->NodeArray.n, pMeshQ->NodeArray.Element, pMeshQ->NodeArray.n * sizeof(Point));

	Point *pPtSrc;
	Point *pPtTgt = pVisualizationData->AssociatedPts.Element;
	Array<Pair<int, int>> associationLines;
	associationLines.Element = new Pair<int, int>[pPointAssociationData->associatedMPts.n + pPointAssociationData->explainedQPts.n];
	associationLines.n = 0;
	int iMPt, iQPt;

	for (iMPt = 0; iMPt < pMeshM->NodeArray.n; iMPt++)
	{
		iQPt = pPointAssociationData->QNN[iMPt];
		if (iQPt >= 0)
		{
			associationLines.Element[associationLines.n].a = iMPt;
			associationLines.Element[associationLines.n].b = iQPt + pMeshM->NodeArray.n;
			associationLines.n++;
		}
	}
	for (int i = 0; i < pPointAssociationData->explainedQPts.n; i++)
	{
		iQPt = pPointAssociationData->explainedQPts.Element[i];
		iMPt = pPointAssociationData->MNN[iQPt];
		if (pPointAssociationData->QNN[iMPt] != iQPt)
		{
			associationLines.Element[associationLines.n].a = iMPt;
			associationLines.Element[associationLines.n].b = iQPt + pMeshM->NodeArray.n;
			associationLines.n++;
		}
	}
	pVisualizationData->associationLinesActor = pVisualizationData->pVisualizer->DisplayLines(pVisualizationData->AssociatedPts, associationLines, red);
	delete[] associationLines.Element;
}

void DDDetector::SetPointsForPointToPointAssociationVisualization(Array<OrientedPoint> pointsM, Mesh *pMeshQ, PointAssociationData *pPointAssociationData)
{
	uchar green[] = {0, 255, 0};
	uchar blue[] = {0, 0, 255};
	uchar red[] = {255, 0, 0};
	uchar white[] = {255, 255, 255};

	// pVisualizationData->pVisualizer->DisplayPointSet<float, OrientedPoint>(pointsM, green, 0.2);
	// pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pMeshQ->NodeArray, blue, 0.2);

	if (pVisualizationData->AssociatedPts.Element)
		RVL_DELETE_ARRAY(pVisualizationData->AssociatedPts.Element);
	pVisualizationData->AssociatedPts.n = pointsM.n + pMeshQ->NodeArray.n;
	pVisualizationData->AssociatedPts.Element = new Point[pVisualizationData->AssociatedPts.n];

	memcpy(pVisualizationData->AssociatedPts.Element + pointsM.n, pMeshQ->NodeArray.Element, pMeshQ->NodeArray.n * sizeof(Point));

	Point *pPtTgt = pVisualizationData->AssociatedPts.Element;
	Array<Pair<int, int>> associationLines;
	associationLines.Element = new Pair<int, int>[pPointAssociationData->associatedMPts.n];
	associationLines.n = 0;

	// test
	/*Array<Pair<int, int>> associationLines2;
	associationLines2.Element = new Pair<int, int>[pPointAssociationData->explainedQPts.n];
	associationLines2.n = 0;*/
	// end test

	int iMPt, iQPt;

	for (iMPt = 0; iMPt < pointsM.n; iMPt++, pPtTgt++)
	{
		RVLCOPY3VECTOR(((pointsM.Element + iMPt)->P), pPtTgt->P);
		iQPt = pPointAssociationData->QNN[iMPt];
		if (iQPt >= 0)
		{
			associationLines.Element[associationLines.n].a = iMPt;
			associationLines.Element[associationLines.n].b = iQPt + pointsM.n;
			associationLines.n++;
		}
	}

	/*for (int i = 0; i < pPointAssociationData->explainedQPts.n; i++)
	{
		iQPt = pPointAssociationData->explainedQPts.Element[i];
		iMPt = pPointAssociationData->MNN[iQPt];
		if (pPointAssociationData->QNN[iMPt] != iQPt)
		{
			associationLines2.Element[associationLines2.n].a = iMPt;
			associationLines2.Element[associationLines2.n].b = iQPt + pointsM.n;
			associationLines2.n++;
		}
	}*/

	pVisualizationData->associationLinesActor = pVisualizationData->pVisualizer->DisplayLines(pVisualizationData->AssociatedPts, associationLines, white);
	// pVisualizationData->associationLinesActor = pVisualizationData->pVisualizer->DisplayLines(pVisualizationData->AssociatedPts, associationLines2, blue);
	delete[] associationLines.Element;
	// delete[] associationLines2.Element;
}

void DDDetector::SetShiftVectorForVisualization(float *firstPoint, Vector3<float> shift, uchar *color)
{
	int scale = -10;

	Array<Pair<int, int>> lines;
	lines.Element = new Pair<int, int>[1];
	lines.n = 1;

	Array<Point> points;
	points.Element = new Point[2];
	points.n = 2;

	Point *pPt;
	float *P0 = firstPoint;
	float P[3];
	float shift_[3];

	pPt = &points.Element[0];
	RVLCOPY3VECTOR(P0, pPt->P);

	RVLSET3VECTOR(shift_, shift.Element[0], shift.Element[1], shift.Element[2]);
	RVLSCALE3VECTOR(shift_, scale, shift_);
	RVLSUM3VECTORS(P0, shift_, P);
	pPt = &points.Element[1];
	RVLCOPY3VECTOR(P, pPt->P);

	lines.Element[0].a = 0;
	lines.Element[0].b = 1;

	pVisualizationData->associationLinesActor = pVisualizationData->pVisualizer->DisplayLines(points, lines, color);
}

void DDDetector::SetShiftVectorsForVisualization(float *firstPoint, Array<Vector3<float>> shiftArray, uchar *color)
{
	int scale = -10;

	Array<Pair<int, int>> lines;
	lines.Element = new Pair<int, int>[shiftArray.n];
	lines.n = shiftArray.n;

	Array<Point> points;
	points.Element = new Point[shiftArray.n + 1];
	points.n = shiftArray.n + 1;

	Point startPt, endPt;
	Point *pPt;
	float *P0 = firstPoint;
	float P[3];
	float shift_[3];

	pPt = &points.Element[0];
	RVLCOPY3VECTOR(P0, pPt->P);

	for (int iShift = 0; iShift < shiftArray.n; iShift++)
	{
		RVLSET3VECTOR(shift_, shiftArray.Element[iShift].Element[0], shiftArray.Element[iShift].Element[1], shiftArray.Element[iShift].Element[2]);
		RVLSCALE3VECTOR(shift_, scale, shift_);
		RVLSUM3VECTORS(P0, shift_, P);
		pPt = &points.Element[iShift + 1];
		RVLCOPY3VECTOR(P, pPt->P);

		// all lines starts from same point
		lines.Element[iShift].a = 0;
		lines.Element[iShift].b = iShift + 1;
	}

	pVisualizationData->associationLinesActor = pVisualizationData->pVisualizer->DisplayLines(points, lines, color);
}

void DDDetector::SetPointsForVisualization(Array<OrientedPoint> points, uchar *color, float size)
{
	pVisualizationData->pVisualizer->DisplayPointSet<float, OrientedPoint>(points, color, size);
}

void DDDetector::VisualizeFittingScoreCalculation(Mesh *pMesh, int nTransparentPts, int *SMCorrespondence, bool bVisualizeMesh)
{
	vtkSmartPointer<vtkActor> hypothesisActor[4];
	if (bVisualizeMesh)
		pVisualizationData->pVisualizer->SetMesh(pMesh->pPolygonData);
	RECOG::DisplayHypothesisEvaluation2(pVisualizationData->pVisualizer, pMesh,
										ZBuffer, ZBufferActivePtArray, subImageMap, SMCorrespondence, nTransparentPts, hypothesisActor);
	pVisualizationData->pVisualizer->Run();
	for (int i = 0; i < 4; i++)
		if (hypothesisActor[i])
			pVisualizationData->pVisualizer->renderer->RemoveViewProp(hypothesisActor[i]);
	// pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
}

void DDDetector::Voter1DTest()
{
	// VOTE1D test
	Voter1D voter;
	Array<float> data;
	data.Element = new float[20];
	data.n = 20;

	data.Element[0] = 0.9;
	data.Element[1] = 0.8;
	data.Element[2] = 0.1;
	data.Element[3] = 1.9;
	data.Element[4] = 0.0;
	data.Element[5] = 2.0;
	data.Element[6] = 3.1;
	data.Element[7] = 0.5;
	data.Element[8] = 0.1;
	data.Element[9] = 0.2;
	data.Element[10] = 4.5;
	data.Element[11] = 2.6;
	data.Element[12] = 1.1;
	data.Element[13] = -0.1;
	data.Element[14] = -0.8;
	data.Element[15] = 1.4;
	data.Element[16] = 1.7;
	data.Element[17] = 1.8;
	data.Element[18] = 1.3;
	data.Element[19] = 1.5;

	voter.Vote(data, 1);
	float value;
	value = voter.GetCellValue(0);
	value = voter.GetCellValue(1);
	value = voter.GetCellValue(2);
	value = voter.GetCellValue(3);
	value = voter.GetCellValue(4);
	value = voter.GetCellValue(5);

	int votes;
	votes = voter.GetCellVotes(0);
	votes = voter.GetCellVotes(1);
	votes = voter.GetCellVotes(2);
	votes = voter.GetCellVotes(3);
	votes = voter.GetCellVotes(4);
	votes = voter.GetCellVotes(5);

	float maxValue = voter.GetMax();
}

void DDDetector::Voter3DTest()
{
	// VOTE3D test
	Voter3D voter;
	Array<Vector3<float>> data;
	int sizeX = 2;
	int sizeY = 2;
	int sizeZ = 2;
	data.Element = new Vector3<float>[sizeX * sizeY * sizeZ];
	data.n = sizeX * sizeY * sizeZ;

	data.Element[0].Element[0] = 0.1;
	data.Element[0].Element[1] = 1.2;
	data.Element[0].Element[2] = 2.1;

	data.Element[1].Element[0] = 0.3;
	data.Element[1].Element[1] = 1.3;
	data.Element[1].Element[2] = 2.2;

	data.Element[2].Element[0] = 0.25;
	data.Element[2].Element[1] = 1.1;
	data.Element[2].Element[2] = 2.15;

	data.Element[3].Element[0] = 0.2;
	data.Element[3].Element[1] = 1.25;
	data.Element[3].Element[2] = 2.15;

	data.Element[4].Element[0] = 3.0;
	data.Element[4].Element[1] = 1.0;
	data.Element[4].Element[2] = 2.0;

	data.Element[5].Element[0] = 2.0;
	data.Element[5].Element[1] = 0.1;
	data.Element[5].Element[2] = 1.0;

	data.Element[6].Element[0] = 1.1;
	data.Element[6].Element[1] = 3.1;
	data.Element[6].Element[2] = 0.1;

	data.Element[7].Element[0] = 0.5;
	data.Element[7].Element[1] = 2.5;
	data.Element[7].Element[2] = 1.5;

	float cellSizeIn[3] = {0.5, 0.5, 0.5};
	voter.Vote(data, cellSizeIn);
	Vector3<float> value;
	value = voter.GetCellValue(0);
	value = voter.GetCellValue(1);
	value = voter.GetCellValue(2);
	value = voter.GetCellValue(3);
	value = voter.GetCellValue(4);
	value = voter.GetCellValue(5);

	int votes;
	votes = voter.GetCellVotes(0);
	votes = voter.GetCellVotes(1);
	votes = voter.GetCellVotes(2);
	votes = voter.GetCellVotes(3);
	votes = voter.GetCellVotes(4);
	votes = voter.GetCellVotes(5);

	Vector3<float> maxValue = voter.GetMax();
}

// END I. Vidovic

// V. Simundic

void DDDetector::getLineParamFromPoints(float *paramsLine, cv::Point pt1, cv::Point pt2)
{
	paramsLine[0] = (float)-(pt1.y - pt2.y) / (pt2.x - pt1.x + std::numeric_limits<float>::epsilon());
	paramsLine[1] = (float)(pt1.y - paramsLine[0] * pt1.x);
}

void DDDetector::getPolarLineFromCartPoints(float *params, cv::Point pt1, cv::Point pt2)
{
	float m = (float)(pt2.y - pt1.y) / (pt2.x - pt1.x + std::numeric_limits<float>::epsilon());
	float b_ = (float)(pt1.y - m * pt1.x);

	// float a = 1;
	// float b = -m;
	// float c = -b_;

	// printf("pt1.x = %d\n", pt1.x);
	// printf("pt1.y = %d\n", pt1.y);
	// printf("pt2.x = %d\n", pt2.x);
	// printf("pt2.y = %d\n", pt2.y);

	// printf("m = %.2f\n", m);
	// printf("b = %.2f\n", b_);

	float theta, rho;

	theta = atan2((float)pt2.x - pt1.x, (float)pt2.y - pt1.y + std::numeric_limits<float>::epsilon()); // theta
	rho = b_ / (sin(params[0]) - m * cos(params[0]));                                                  // rho

	rho = params[0];
	theta = params[1];

	// printf("theta = %.2f\n", params[0]);
	// printf("thetaCel = %.2f\n", params[0]*180.0/CV_PI);
	// printf("r = %.2f\n\n", params[1]);
}

bool DDDetector::checkPointOnLine(cv::Mat *edges, cv::Point pt, float m, float b)
{
	// m & b are line params

	if (pt.y == pt.x * m + b && edges->at<uchar>(pt) > 0)
		return true;

	return false;
}

void DDDetector::HoughLinesPtAssociationCylinder(
	cv::Mat &edges,
	RVL::RECOG::DDD::PtAssoc *ptAssoc,
	RVL::Mesh *pMesh,
	Array<RECOG::DDD::EdgeSample> &edgeSamplePts,
	RVL::RECOG::DDD::Edge *edge,
	bool bHoughLinesVis)
{
	cv::Mat edgesClr;
	cv::cvtColor(edges, edgesClr, cv::COLOR_GRAY2BGR);

	std::vector<cv::Vec2f> vecHoughLines;
	cv::HoughLines(edges, vecHoughLines, 1, CV_PI / 180, 50, 0, 0);

	float r1, theta1, r2, theta2;
	float rho_v, theta_v; // v for visualization

	// Left line will be considered first (r1, th1)
	if (abs(vecHoughLines[0][0]) < abs(vecHoughLines[1][0]))
	{
		r1 = vecHoughLines[0][0];
		theta1 = vecHoughLines[0][1];
		r2 = vecHoughLines[1][0];
		theta2 = vecHoughLines[1][1];
	}
	else
	{
		r2 = vecHoughLines[0][0];
		theta2 = vecHoughLines[0][1];
		r1 = vecHoughLines[1][0];
		theta1 = vecHoughLines[1][1];
	}

	if (bHoughLinesVis) // Visualize only 2 longest lines
	{
		for (size_t i = 0; i < 2; i++)
		{
			float rho_v = vecHoughLines[i][0], theta_v = vecHoughLines[i][1];
			cv::Point pt1, pt2;
			double a = cos(theta_v), b = sin(theta_v);
			double x0 = a * rho_v, y0 = b * rho_v;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));

			cv::line(edgesClr, pt1, pt2, cv::Scalar(0, 0, 255), 1, 8);
		}

		cv::imshow("Hough 5 Pts", edgesClr);
		cv::waitKey(0);
	}

	int offsetFromTop = 100;    // Offset pixels from top of the image
	int offsetFromBottom = 450; // Offset pixels from bottom of the image

	// Set 4 points on the two longest lines
	int y1 = 0 + offsetFromTop, y2 = 0 + offsetFromTop, y3 = (int)edges.rows - 1 - offsetFromBottom, y4 = (int)edges.rows - 1 - offsetFromBottom;

	std::vector<cv::Point> ptLineEdges;
	ptLineEdges.push_back(cv::Point((int)((r1 - y1 * sin(theta1)) / cos(theta1)), y1));
	ptLineEdges.push_back(cv::Point((int)((r2 - y2 * sin(theta2)) / cos(theta2)), y2));
	ptLineEdges.push_back(cv::Point((int)((r1 - y3 * sin(theta1)) / cos(theta1)), y3));
	ptLineEdges.push_back(cv::Point((int)((r2 - y4 * sin(theta2)) / cos(theta2)), y4));

	if (bHoughLinesVis) // Visualize the edge ROI points
	{
		for (size_t i = 0; i < 4; i++)
		{
			cv::circle(edgesClr, cv::Point(ptLineEdges.at(i)), 5, cv::Scalar(255, 255, 255), 3);
		}

		cv::imshow("Hough 5 Pts", edgesClr.clone());
		cv::waitKey(0);
	}

	// Select random points on the lines and check if the pixels at the coordinates are "white"
	int x_temp, y_temp; // random point coordinates on a line
	int y_temps[4];
	int iiQPix;                    // Index of the point on the image
	for (size_t i = 0; i < 2; i++) // take 2 points on the first detected line
	{
		while (1)
		{
			y_temp = rand() % (edges.rows - 1 - offsetFromTop) + offsetFromTop;
			x_temp = round((r1 - y_temp * sin(theta1)) / cos(theta1));
			iiQPix = y_temp * edges.cols + x_temp;

			if (i == 1 && iiQPix == ptAssoc[i - 1].iQPix)
				continue;
			else if (edges.at<uchar>(y_temp, x_temp) > 0)
			{
				ptAssoc[i].iQPix = iiQPix;
				cv::circle(edgesClr, cv::Point(x_temp, y_temp), 5, cv::Scalar(255, 0, 0), 3);
				y_temps[i] = y_temp;
				break;
			}
		}
	}

	if (y_temps[0] > y_temps[1])
	{
		int tempIQPix;

		tempIQPix = ptAssoc[0].iQPix;
		ptAssoc[0].iQPix = ptAssoc[1].iQPix;
		ptAssoc[1].iQPix = tempIQPix;
	}

	for (size_t i = 2; i < 4; i++) // take another 2 points on the second detected line
	{
		while (1)
		{
			y_temp = rand() % (edges.rows - 1 - offsetFromTop) + offsetFromTop;
			x_temp = round((r2 - y_temp * sin(theta2)) / cos(theta2));
			iiQPix = y_temp * edges.cols + x_temp;

			if (i == 3 && iiQPix == ptAssoc[i - 1].iQPix)
				continue;
			else if (x_temp < edges.cols - 1 && x_temp > -1 && edges.at<uchar>(y_temp, x_temp) == 255)
			{
				ptAssoc[i].iQPix = iiQPix;
				cv::circle(edgesClr, cv::Point(x_temp, y_temp), 5, cv::Scalar(255, 0, 0), 3);
				y_temps[i] = y_temp;
				break;
			}
		}
	}
	printf("opety2: %d, %d\n", y_temps[2], y_temps[3]);
	if (y_temps[2] > y_temps[3])
	{
		int tempIQPix;

		tempIQPix = ptAssoc[2].iQPix;
		ptAssoc[2].iQPix = ptAssoc[3].iQPix;
		ptAssoc[3].iQPix = tempIQPix;
	}

	if (bHoughLinesVis) // Show the 4 points
	{
		cv::imshow("Hough 5 Pts", edgesClr);
		cv::waitKey(0);
	}

	// Set top-left and bottom-right images to reduce searching points
	cv::Point ptTopLeft, ptBottomRight;
	int leftMostX = edges.cols - 1, rightMostX = 0;

	for (size_t i = 0; i < 4; i++) // find the leftmost and the rightmost point
	{
		if (ptLineEdges.at(i).x < leftMostX)
		{
			leftMostX = ptLineEdges.at(i).x;
		}
		if (ptLineEdges.at(i).x > rightMostX)
		{
			rightMostX = ptLineEdges.at(i).x;
		}
	}

	// Set topleft and bottomright points
	ptTopLeft.x = leftMostX;
	ptTopLeft.y = 0 + offsetFromTop;

	ptBottomRight.x = rightMostX;
	ptBottomRight.y = edges.rows - 1 - offsetFromBottom;

	// ROI for searching the 5th point between the two lines
	cv::Mat edgesTipROI(edges, cv::Rect(ptTopLeft, ptBottomRight));

	if (bHoughLinesVis) // Show the tip ROI and all 5 points
	{
		cv::circle(edgesClr, ptTopLeft, 5, cv::Scalar(0, 255, 0), 3);
		cv::circle(edgesClr, ptBottomRight, 5, cv::Scalar(0, 255, 0), 3);
		cv::imshow("Hough 5 Pts", edgesClr);

		cv::imshow("Tip ROI", edgesTipROI);
		cv::waitKey(0);
	}

	int x_fixed = (int)edgesTipROI.cols / 2;
	int y_var;
	cv::Point fifthAssocPt, fifthAssocPtROI;

	for (int i = 0; i < edgesTipROI.rows; i++) // Take the center x-coordinate and iterate over y to find the "white" pixel
	{
		if (edgesTipROI.at<uchar>(cv::Point(x_fixed, i)) > 0)
		{
			fifthAssocPt = cv::Point(x_fixed, i) + ptTopLeft;
			fifthAssocPtROI = cv::Point(x_fixed, i);
			ptAssoc[4].iQPix = fifthAssocPt.y * edges.cols + fifthAssocPt.x;
			// cv::circle(edgesClr, fifthAssocPt, 5, cv::Scalar(255, 0, 0), 3);

			break;
		}
	}

	cv::Point sixthAssocPt;
	bool bIsPtFound = false;
	for (int i = fifthAssocPtROI.y + 1; i < edgesTipROI.rows; i++) // Take the center x-coordinate and iterate over y to find the "white" pixel
	{
		if (edgesTipROI.at<uchar>(cv::Point(x_fixed, i)) > 0)
		{
			fifthAssocPt = cv::Point(x_fixed, i) + ptTopLeft;
			ptAssoc[4].iQPix = fifthAssocPt.y * edges.cols + fifthAssocPt.x;
			bIsPtFound = true;
			break;
		}
	}
	cv::circle(edgesClr, fifthAssocPt, 5, cv::Scalar(255, 0, 0), 3);

	if (!bIsPtFound)
	{
		x_fixed += 5;
		for (int i = 0; i < edgesTipROI.rows; i++) // Take the center x-coordinate and iterate over y to find the "white" pixel
		{
			if (edgesTipROI.at<uchar>(cv::Point(x_fixed, i)) > 0)
			{
				sixthAssocPt = cv::Point(x_fixed, i) + ptTopLeft;
				ptAssoc[5].iQPix = sixthAssocPt.y * edges.cols + sixthAssocPt.x;
				cv::circle(edgesClr, sixthAssocPt, 5, cv::Scalar(255, 0, 0), 3);
				break;
			}
		}
	}

	if (bHoughLinesVis) // Show all points
	{
		cv::imshow("Hough 5 Pts", edgesClr);
		cv::waitKey(0);
	}

	// FIND POINTS ON Model
	RVL::RECOG::DDD::Edge *MpEdge_;
	int longestMEdgeIdx = 0, secondLongestMEdgeIdx = 0;
	float lengthScore = 0.0f, secondLengthScore = 0.0f;

	// Find two longest edges in model

	for (int iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		MpEdge_ = edge + iEdge;

		if (MpEdge_->bVisible && MpEdge_->length > lengthScore)
		{
			secondLongestMEdgeIdx = longestMEdgeIdx;
			longestMEdgeIdx = iEdge;

			secondLengthScore = lengthScore;
			lengthScore = MpEdge_->length;
		}
		else if (MpEdge_->bVisible && MpEdge_->length > secondLengthScore && iEdge != longestMEdgeIdx)
		{
			secondLongestMEdgeIdx = iEdge;
			secondLengthScore = MpEdge_->length;
		}
	}

	// Sort edge points based on the line idxs
	RVL::RECOG::DDD::EdgeSample *edgeSample;
	std::vector<int> longestEdgePtsM, secondLongestEdgePtsM, otherEdgesPtsM;

	for (int iEdgeSample = 0; iEdgeSample < edgeSamplePts.n; iEdgeSample++)
	{
		edgeSample = edgeSamplePts.Element + iEdgeSample;

		if (edgeSample->edgeIdx == longestMEdgeIdx)
			longestEdgePtsM.push_back(iEdgeSample);
		else if (edgeSample->edgeIdx == secondLongestMEdgeIdx)
			secondLongestEdgePtsM.push_back(iEdgeSample);
		else
			otherEdgesPtsM.push_back(iEdgeSample);
	}

	float polarParams1[2], polarParams2[2];
	int x_t1 = (edgeSamplePts.Element + longestEdgePtsM.at(0))->iImgP[0];
	int y_t1 = (edgeSamplePts.Element + longestEdgePtsM.at(0))->iImgP[1];
	int x_t2 = (edgeSamplePts.Element + longestEdgePtsM.back())->iImgP[0];
	int y_t2 = (edgeSamplePts.Element + longestEdgePtsM.back())->iImgP[1];
	RVL::DDDetector::getPolarLineFromCartPoints(&polarParams1[0], cv::Point(x_t1, y_t1), cv::Point(x_t2, y_t2));

	x_t1 = (edgeSamplePts.Element + secondLongestEdgePtsM.at(0))->iImgP[0];
	y_t1 = (edgeSamplePts.Element + secondLongestEdgePtsM.at(0))->iImgP[1];
	x_t2 = (edgeSamplePts.Element + secondLongestEdgePtsM.back())->iImgP[0];
	y_t2 = (edgeSamplePts.Element + secondLongestEdgePtsM.back())->iImgP[1];
	RVL::DDDetector::getPolarLineFromCartPoints(&polarParams2[0], cv::Point(x_t1, y_t1), cv::Point(x_t2, y_t2));

	// Find which edge is left and which is right
	if (abs(polarParams1[1]) < abs(polarParams2[1]))
	{

		ptAssoc[0].iMSample = longestEdgePtsM.at(3);
		ptAssoc[1].iMSample = longestEdgePtsM.end()[-3];

		ptAssoc[3].iMSample = secondLongestEdgePtsM.at(3);
		ptAssoc[2].iMSample = secondLongestEdgePtsM.end()[-3];
	}
	else
	{
		ptAssoc[0].iMSample = secondLongestEdgePtsM.at(3);
		ptAssoc[1].iMSample = secondLongestEdgePtsM.end()[-3];

		ptAssoc[3].iMSample = longestEdgePtsM.at(3);
		ptAssoc[2].iMSample = longestEdgePtsM.end()[-3];
	}
	if ((edgeSamplePts.Element + ptAssoc[0].iMSample)->iImgP[1] > (edgeSamplePts.Element + ptAssoc[1].iMSample)->iImgP[1])
	{
		int tempiMSample;

		tempiMSample = ptAssoc[0].iMSample;
		ptAssoc[0].iMSample = ptAssoc[1].iMSample;
		ptAssoc[1].iMSample = tempiMSample;
	}
	printf("1y: %d, %d\n", (edgeSamplePts.Element + ptAssoc[0].iMSample)->iImgP[1], (edgeSamplePts.Element + ptAssoc[1].iMSample)->iImgP[1]);

	if ((edgeSamplePts.Element + ptAssoc[2].iMSample)->iImgP[1] > (edgeSamplePts.Element + ptAssoc[3].iMSample)->iImgP[1])
	{
		int tempiMSample;

		tempiMSample = ptAssoc[2].iMSample;
		ptAssoc[2].iMSample = ptAssoc[3].iMSample;
		ptAssoc[3].iMSample = tempiMSample;
	}

	printf("2y: %d, %d\n", (edgeSamplePts.Element + ptAssoc[2].iMSample)->iImgP[1], (edgeSamplePts.Element + ptAssoc[3].iMSample)->iImgP[1]);

	// Find fifth assoc point with lowest y value (highest on the image)
	std::vector<int>::iterator itrOtherEdgesPtsM;
	int y_lowest = edges.rows - 1;

	for (itrOtherEdgesPtsM = otherEdgesPtsM.begin(); itrOtherEdgesPtsM < otherEdgesPtsM.end(); itrOtherEdgesPtsM++)
	{
		if ((edgeSamplePts.Element + *itrOtherEdgesPtsM)->iImgP[1] < y_lowest)
		{
			ptAssoc[4].iMSample = *itrOtherEdgesPtsM;
			y_lowest = (edgeSamplePts.Element + *itrOtherEdgesPtsM)->iImgP[1];
		}
	}

	ptAssoc[5].iMSample = ptAssoc[4].iMSample + 1;
}

void DDDetector::HoughLinesPtAssociationLHTCP(
	cv::Mat &edges,
	float *QNMap,
	double *IuMap,
	double *IvMap,
	RVL::Mesh *pMesh,
	RVL::RECOG::DDD::Edge *edge,
	Camera &camera_,
	bool bHoughLinesVis,
	Array<RECOG::DDD::EdgeSample> &edgeSamplePts,
	std::vector<int> &fifthAssocPtHypsIdxs,
	RVL::RECOG::DDD::PtAssoc *ptAssoc,
	std::vector<int> &fifthMIdxs)
{
	// // ********************* LHTCP *********************
	ModelPtAssociationLHTCP(pMesh, edge, edgeSamplePts, ptAssoc, fifthMIdxs);
	cv::Mat linesImg, randPtImg;
	cv::cvtColor(edges, linesImg, cv::COLOR_GRAY2BGR);
	cv::cvtColor(edges, randPtImg, cv::COLOR_GRAY2BGR);

	pVisualizationData->displayImg = &linesImg;

	std::vector<cv::Vec2f> vecHoughLines;
	cv::HoughLines(edges, vecHoughLines, 1, CV_PI / 180, 70, 0, 0);

	// Pop out all lines that deviate in angle from the most dominant line
	float thetaMostDominant = vecHoughLines[0][1];
	float delta, distance;

	for (int i = vecHoughLines.size() - 1; i >= 0; i--) // Pop lines
	{
		delta = abs(thetaMostDominant - vecHoughLines[i][1]);
		distance = delta > CV_PI / 2.0f ? CV_PI - delta : delta;
		if (distance > (float)4.0f * CV_PI / 180.0f)
		{
			vecHoughLines.erase(vecHoughLines.begin() + i);
			continue;
		}
	}

	// Find leftmost and rightmost outer lines
	int houghLinesIdxs[2];
	RVL::DDDetector::setupHoughLineEdges(vecHoughLines, edges, houghLinesIdxs);

	if (bHoughLinesVis)
	{
		cv::Scalar color(0, 0, 255);
		// Visualize the dominant line
		drawLine(*pVisualizationData->displayImg, vecHoughLines[0], color);
		cv::imshow("Hough 5 Pts", *pVisualizationData->displayImg);
		cv::waitKey(0);

		// Visualize all lines
		for (int i = 0; i < vecHoughLines.size(); i++)
		{
			drawLine(*pVisualizationData->displayImg, vecHoughLines[i], color);
		}
		cv::imshow("Hough 5 Pts", *pVisualizationData->displayImg);
		cv::waitKey(0);

		// Visualize 4 longest lines
		cv::cvtColor(edges, *pVisualizationData->displayImg, cv::COLOR_GRAY2BGR);
		for (int i = 0; i < 4; i++)
		{
			drawLine(*pVisualizationData->displayImg, vecHoughLines[i], color);
		}
		cv::imshow("Hough 5 Pts", *pVisualizationData->displayImg);
		cv::waitKey(0);
	}

	// Get 4 assoc pts
	// cv::Point outerPts[2];
	cv::Point pt;
	pVisualizationData->displayImg = &randPtImg;
	float rho_t, theta_t;
	int y_t, x_t, iiQPix;
	float *QN;
	float QV[3], QP[3], sn, cs, Qu, Qv, fTmp;
	bool bPointIsOnLine;
	QP[2] = 1.0f;
	iRnd = 0;
	for (int i = 0; i < 2; i++) // Loop over 2 edges params
	{
		for (int j = 0; j < 2; j++) // Loop over two points on each edge
		{
			rho_t = vecHoughLines[houghLinesIdxs[i]][0];
			theta_t = vecHoughLines[houghLinesIdxs[i]][1];
			while (true)
			{
				cs = cos(theta_t);
				sn = sin(theta_t);
				// y_t = (int)rand() % edges.rows;
				y_t = pseudornd.Element[iRnd++] % edges.rows;
				x_t = (int)(rho_t - y_t * sn) / cs;

				// pt.x = x_t;
				// pt.y = y_t;
				// if (bHoughLinesVis) // Show rand pts
				// {
				// 	cv::circle(*pVisualizationData->displayImg, pt, 3, cv::Scalar(0, 255, 0), 2);
				// 	cv::imshow("Random pt", *pVisualizationData->displayImg);
				// 	cv::waitKey(10);
				// }

				iiQPix = y_t * edges.cols + x_t;
				bPointIsOnLine = edges.at<uchar>(y_t, x_t) > 0;
				if (bPointIsOnLine)
				{
					// Assign PtAssoc
					ptAssoc[2 * i + j].iQPix = iiQPix;
					QN = QNMap + 3 * iiQPix;
					RVLSET3VECTOR(QV, -sn, cs, 0.0f);
					Qu = (float)(iiQPix % edges.cols);
					Qv = (float)(iiQPix / edges.cols);
					QP[0] = (Qu - camera_.uc) / camera_.fu;
					QP[1] = (Qv - camera_.vc) / camera_.fv;
					RVLCROSSPRODUCT3(QV, QP, QN);
					RVLNORM3(QN, fTmp);

					// cv::circle(pVisualizationData->displayImg, cv::Point(x_t, y_t), 5, cv::Scalar(255, 0, 0), 3);

					break;
				}
			}
		}
	}

	// 5th Assoc Pt
	float rhoMid;	// = (vecHoughLines[houghLinesIdxs[0]][0] + vecHoughLines[houghLinesIdxs[1]][0]) / 2.0f;
	float thetaMid; // = vecHoughLines[houghLinesIdxs[0]][1];

	if (abs(vecHoughLines[houghLinesIdxs[0]][1] - vecHoughLines[houghLinesIdxs[1]][1]) > 90 * DEG2RAD)
	{
		rhoMid = (abs(vecHoughLines[houghLinesIdxs[0]][0]) + abs(vecHoughLines[houghLinesIdxs[1]][0])) / 2.0f;
		thetaMid = 0.0f;
	}
	else
	{
		rhoMid = (vecHoughLines[houghLinesIdxs[0]][0] + vecHoughLines[houghLinesIdxs[1]][0]) / 2.0f;
		thetaMid = vecHoughLines[houghLinesIdxs[0]][1];
	}

	// if(vecHoughLines[houghLinesIdxs[0]][0] < 0.0f && vecHoughLines[houghLinesIdxs[1]][0] < 0.0f)
	// {
	// 	rhoMid = (vecHoughLines[houghLinesIdxs[0]][0] + vecHoughLines[houghLinesIdxs[1]][0]) / 2.0f;
	// }
	// else
	// {
	// 	rhoMid = (abs(vecHoughLines[houghLinesIdxs[0]][0]) + abs(vecHoughLines[houghLinesIdxs[1]][0])) / 2.0f;
	// }
	// float thetaMid = rhoMid < 0.0f ? 180 * DEG2RAD + vecHoughLines[houghLinesIdxs[0]][1] : vecHoughLines[houghLinesIdxs[0]][1];

	if (bHoughLinesVis) // Show midline
	{
		cv::Vec2f midLine(rhoMid, thetaMid);
		drawLine(*pVisualizationData->displayImg, midLine, cv::Scalar(255, 0, 0));
		cv::imshow("Hough 5 Pts", *pVisualizationData->displayImg);
		cv::waitKey(0);
	}

	int offsetFromTop = 0;	  // Offset pixels from top of the image
	int offsetFromBottom = 0; // Offset pixels from bottom of the image
	fifthAssocPtHypsIdxs.clear();
	for (y_t = offsetFromTop; y_t < edges.rows - 1 - offsetFromBottom; y_t++)
	{
		cs = cos(thetaMid);
		sn = sin(thetaMid);
		x_t = (int)(rhoMid - y_t * sn) / cs;
		bPointIsOnLine = edges.at<uchar>(y_t, x_t) > 0;
		if (bPointIsOnLine)
		{
			iiQPix = y_t * edges.cols + x_t;
			fifthAssocPtHypsIdxs.push_back(iiQPix);
			// if (fifthAssocPtHypsIdxs.size() >= 2)
			// 	break;
		}
	}

	if (bHoughLinesVis) // Show all points
	{

		cv::cvtColor(edges, *pVisualizationData->displayImg, cv::COLOR_GRAY2BGR);
		cv::Point ptVis;
		int i;
		for (i = 0; i < 4; i++)
		{
			ptVis.x = ptAssoc[i].iQPix % edges.cols;
			ptVis.y = ptAssoc[i].iQPix / edges.cols;
			cv::circle(*pVisualizationData->displayImg, ptVis, 3, cv::Scalar(0, 0, 255), 2);
		}
		for (i = 0; i < fifthAssocPtHypsIdxs.size(); i++)
		{
			ptVis.x = fifthAssocPtHypsIdxs[i] % edges.cols;
			ptVis.y = fifthAssocPtHypsIdxs[i] / edges.cols;
			cv::circle(*pVisualizationData->displayImg, ptVis, 3, cv::Scalar(255, 255, 0), 2);
		}
		cv::imshow("Hough 5 Pts", *pVisualizationData->displayImg);
		cv::waitKey(0);
	}
}

void DDDetector::setupHoughLineEdges(
	std::vector<cv::Vec2f> &vecHoughLines,
	cv::Mat &edges,
	int *houghLinesIdxs)
{
	float leftMostScore = 10000.0f, rightMostScore = 0.0f;
	float absScore;
	for (int i = 0; i < vecHoughLines.size(); i++)
	{
		absScore = abs(vecHoughLines[i][0]);
		if (absScore < leftMostScore)
		{
			houghLinesIdxs[0] = i;
			leftMostScore = absScore;
		}
		if (absScore > rightMostScore)
		{
			houghLinesIdxs[1] = i;
			rightMostScore = absScore;
		}
	}

	// // Find leftmost and rightmost inner lines
	// float difOuterLines = vecHoughLines[houghLinesIdxs[1]][0] - vecHoughLines[houghLinesIdxs[0]][0];
	// float dLine = difOuterLines * (1 - (1.8f / 2.9f)) / 2.0f;
	// leftMostScore = 500.0f, rightMostScore = 500.0f;
	// for (int i = 0; i < vecHoughLines.size(); i++)
	// {
	// 	if (abs(vecHoughLines[houghLinesIdxs[0]][0] + dLine - vecHoughLines[i][0]) < leftMostScore)
	// 	{
	// 		leftMostScore = abs(vecHoughLines[houghLinesIdxs[0]][0] + dLine - vecHoughLines[i][0]);
	// 		houghLinesIdxs[2] = i;
	// 	}
	// 	if (abs(vecHoughLines[houghLinesIdxs[1]][0] - dLine - vecHoughLines[i][0]) < rightMostScore)
	// 	{
	// 		rightMostScore = abs(vecHoughLines[houghLinesIdxs[1]][0] - dLine - vecHoughLines[i][0]);
	// 		houghLinesIdxs[3] = i;
	// 	}
	// }
}

bool RVL::DDDetector::compareEdgeSampleY(cv::Point &edgeSample1, cv::Point &edgeSample2)
{
	return (edgeSample1.y < edgeSample2.y);
}

void RVL::DDDetector::CamerasIntrinsicCalibration(std::string imagesPath, cv::Size chessBSize)
{
	std::vector<cv::String> images;
	cv::Mat iImage, iGray;
	bool bFoundCorners;
	std::vector<cv::Point2f> cornerPts;

	std::vector<std::vector<cv::Point3f>> objPts; // Creating vector to store vectors of 3D points for each checkerboard image
	std::vector<std::vector<cv::Point2f>> imgPts; // Creating vector to store vectors of 2D points for each checkerboard image
	std::vector<cv::Point3f> objp;				  // Defining the world coordinates for 3D points

	for (int i = 0; i < chessBSize.width; i++)
		for (int j = 0; j < chessBSize.height; j++)
			objp.push_back(cv::Point3f(j, i, 0));

	// List of all images for calibration
	cv::glob(imagesPath, images, false);

	cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

	for (int iFN = 0; iFN < images.size(); iFN++)
	{
		// cout << images[iFN] << endl;
		iImage = cv::imread(images[iFN]);
		cv::cvtColor(iImage, iGray, cv::COLOR_BGR2GRAY);

		bFoundCorners = cv::findChessboardCorners(iImage, chessBSize, cornerPts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (bFoundCorners)
		{
			cv::cornerSubPix(iGray, cornerPts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
			cv::drawChessboardCorners(iImage, chessBSize, cornerPts, bFoundCorners);

			objPts.push_back(objp);
			imgPts.push_back(cornerPts);
		}

		cv::imshow("Image", iImage);
		cv::waitKey(0);
	}

	cv::destroyAllWindows();

	cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;

	cv::calibrateCamera(objPts, imgPts, cv::Size(iGray.rows, iGray.cols), cameraMatrix, distCoeffs, rvecs, tvecs);

	std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
	std::cout << "distCoeffs : " << distCoeffs << std::endl;
	std::cout << "Rotation vector : " << rvecs << std::endl;
	std::cout << "Translation vector : " << tvecs << std::endl;

	cv::FileStorage fs("/home/valentin/FERIT/DANIELI/LHTCP/experiments/KinectCameraCalib.yml", cv::FileStorage::WRITE);
	fs << "cameraMatrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;

	printf("Done Calibration\n");

	// std::vector<cv::Point2f> projectedPoints;

	// cv::projectPoints(objPts, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints)
}

void RVL::DDDetector::CamerasExtrinsicCalibration()
// cv::Mat chessB1,
// cv::Mat chessB2)

{
	cv::Mat chessB1 = cv::imread("/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP-220618/chessB1.png");
	cv::Mat chessB2 = cv::imread("/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP-220618/chessB2.png");

	std::vector<cv::Mat> chessBoardImages;
	chessBoardImages.push_back(chessB1);
	chessBoardImages.push_back(chessB2);

	cv::Size chessBoardSize(8, 6);
	float chessBmm = 35;

	// cv::Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2;
	cv::Mat cameraMatrix[2], distCoeffs[2];

	// std::string cameraParamsPath = "/home/valentin/FERIT/DANIELI/LHTCP/experiments/Exp-LHTCP-220618/cameraParams.yml";
	std::string expDir = "/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP-221212";

	std::string camera1ParamsPath = expDir + "/basler_cam1_params.yaml";
	std::string camera2ParamsPath = expDir + "/basler_cam2_params.yaml";
	std::string savePath = "/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP-221212/extrinsicParams.yml";
	cv::FileStorage fs1, fs2, fsSave;
	fs1.open(camera1ParamsPath, cv::FileStorage::READ);
	fs2.open(camera2ParamsPath, cv::FileStorage::READ);

	// fs["K"] >> cameraMatrix;
	// fs["D"] >> distCoeffs;
	fs1["camera_matrix"] >> cameraMatrix[0];
	fs1["distortion_coefficients"] >> distCoeffs[0];
	fs2["camera_matrix"] >> cameraMatrix[1];
	fs2["distortion_coefficients"] >> distCoeffs[1];
	fs1.release();
	fs2.release();

	std::vector<cv::Point3d> axis;
	axis.push_back(cv::Point3d(3 * chessBmm, 0.0, 0.0));
	axis.push_back(cv::Point3d(0.0, 3 * chessBmm, 0.0));
	axis.push_back(cv::Point3d(0.0, 0.0, -3 * chessBmm));

	std::vector<cv::Mat> RotMats, tvecs;

	for (int iChessImage = 0; iChessImage < 2; iChessImage++)
	{
		cv::Mat image, imageGray;
		image = chessBoardImages[iChessImage];
		cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

		std::vector<cv::Point2f> cornerPts;
		std::vector<cv::Point3f> boardPts;
		std::vector<cv::Point2d> imgPts;
		bool bfoundCorners;
		cv::Mat R_;
		cv::Mat rvec, tvec;

		bfoundCorners = cv::findChessboardCorners(
			imageGray,
			chessBoardSize,
			cornerPts,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (bfoundCorners)
		{
			cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
			cv::cornerSubPix(imageGray, cornerPts, cv::Size(5, 5), cv::Size(-1, -1), criteria);
			// cv::drawChessboardCorners(image, chessBoardSize, cornerPts, bfoundCorners);

			cv::circle(image, cv::Point((int)cornerPts[0].x, (int)cornerPts[0].y), 2, cv::Scalar(0, 255, 255), 2);
		}

		cv::imshow("Chessboard1 corners", image);
		cv::waitKey(0);

		// Points of corners wrt object
		for (int i = 0; i < chessBoardSize.height; i++)
			for (int j = 0; j < chessBoardSize.width; j++)
				boardPts.push_back(cv::Point3f(chessBmm * j - 2.5 * chessBmm, i * chessBmm - 2.5 * chessBmm, 0.0f));

		// Find rotation and translation of markers wrt camera
		cv::solvePnP(cv::Mat(boardPts), cv::Mat(cornerPts), cameraMatrix[iChessImage], distCoeffs[iChessImage], rvec, tvec, false);
		cv::Rodrigues(rvec, R_);

		// Draw Axes
		// cv::projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, imgPts, cv::noArray(), 0);
		cv::arrowedLine(image, cornerPts[0], imgPts[0], cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0);
		cv::arrowedLine(image, cornerPts[0], imgPts[1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
		cv::arrowedLine(image, cornerPts[0], imgPts[2], cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0);
		cv::imshow("Chessboard1 corners", image);
		cv::waitKey(0);

		cout << "rvec = " << rvec << endl;
		cout << "tvec = " << tvec << endl;

		RotMats.push_back(R_);
		tvecs.push_back(tvec);
	}

	cv::Mat TMC1(4, 4, RotMats[0].type());
	TMC1(cv::Range(0, 3), cv::Range(0, 3)) = RotMats[0] * 1;
	TMC1(cv::Range(0, 3), cv::Range(3, 4)) = tvecs[0] * 1;
	double *p = TMC1.ptr<double>(3);
	p[0] = p[1] = p[2] = 0;
	p[3] = 1;

	cv::Mat TMC2(4, 4, RotMats[1].type());
	TMC2(cv::Range(0, 3), cv::Range(0, 3)) = RotMats[1] * 1;
	TMC2(cv::Range(0, 3), cv::Range(3, 4)) = tvecs[1] * 1;
	p = TMC2.ptr<double>(3);
	p[0] = p[1] = p[2] = 0;
	p[3] = 1;

	cout << "TMC1 = \n"
		 << TMC1 << endl;
	cout << "TMC2 = \n"
		 << TMC2 << endl;

	cv::Mat TC2M = TMC2.inv();
	cv::Mat TC2C1 = TMC1 * TC2M;

	cout << "TC2C1 = \n"
		 << TC2C1 << endl;

	// fsSave.open(savePath, cv::FileStorage::WRITE);

	// fsSave << "TC2C1" << TC2C1;

	// fsSave.release();
}

void DDDetector::StereoCalibrationFromImages(
	std::string imagesPath,
	cv::Mat cameraMatrix1,
	cv::Mat distCoeffs1,
	cv::Mat cameraMatrix2,
	cv::Mat distCoeffs2,
	RVL::Pose3D *poseC_C0)
{

#ifdef RVLLINUX
	char slash = '/';
#else
	char slash = '\\';
#endif

	cv::Mat newCameraMatrix1, newCameraMatrix2;
	newCameraMatrix1 = cv::getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, cv::Size(1280, 1024), 1, cv::Size(1280, 1024), 0);
	newCameraMatrix2 = cv::getOptimalNewCameraMatrix(cameraMatrix2, distCoeffs2, cv::Size(1280, 1024), 1, cv::Size(1280, 1024), 0);

	std::vector<cv::String> imagesL;
	std::vector<cv::String> imagesR;
	cv::glob(imagesPath + slash + "*-0.png", imagesL, false);
	cv::glob(imagesPath + slash + "*-1.png", imagesR, false);

	if (imagesL.size() != imagesR.size())
	{
		cout << "Number of images of both cameras should be the same. Aborting." << endl;
		std::abort();
	}

	cv::Mat frameLDist, frameRDist;
	cv::Mat frameL, frameR;
	bool bFoundL, bFoundR;

	int CHECKERBOARD[2]{6, 8};

	cv::Size chessBoardCorners(CHECKERBOARD[0], CHECKERBOARD[1]);
	float squareSize = 0.0105;
	std::vector<cv::Point2f> cornerPtsL, cornerPtsR;
	std::vector<std::vector<cv::Point2f>> imagePtsL, imagePtsR;
	std::vector<cv::Point3f> objPt;
	std::vector<std::vector<cv::Point3f>> objPts;

	for (int i = 0; i < CHECKERBOARD[1]; i++)
		for (int j = 0; j < CHECKERBOARD[0]; j++)
			objPt.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));

	for (int i = 0; i < imagesL.size(); i++)
	{
		frameLDist = cv::imread(imagesL[i], cv::IMREAD_GRAYSCALE);
		frameRDist = cv::imread(imagesR[i], cv::IMREAD_GRAYSCALE);
		// frameL = cv::imread(imagesL[i], cv::IMREAD_GRAYSCALE);
		// frameR = cv::imread(imagesR[i], cv::IMREAD_GRAYSCALE);

		cv::undistort(frameLDist, frameL, cameraMatrix1, distCoeffs1, newCameraMatrix1);
		cv::undistort(frameRDist, frameR, cameraMatrix2, distCoeffs2, newCameraMatrix2);

		bFoundL = cv::findChessboardCorners(frameL, chessBoardCorners, cornerPtsL);
		bFoundR = cv::findChessboardCorners(frameR, chessBoardCorners, cornerPtsR);

		if (bFoundL && bFoundR)
		{
			cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

			cv::cornerSubPix(frameL, cornerPtsL, cv::Size(5, 5), cv::Size(-1, -1), criteria);
			cv::drawChessboardCorners(frameL, chessBoardCorners, cornerPtsL, bFoundL);

			cv::cornerSubPix(frameR, cornerPtsR, cv::Size(5, 5), cv::Size(-1, -1), criteria);
			cv::drawChessboardCorners(frameR, chessBoardCorners, cornerPtsR, bFoundR);

			// cv::imshow("Image left", frameL);
			// cv::imshow("Image right", frameR);
			// cv::waitKey(0);

			objPts.push_back(objPt);
			imagePtsL.push_back(cornerPtsL);
			imagePtsR.push_back(cornerPtsR);
		}
	}

	cv::Mat R, T, essentialMtx, fundamentalMtx;
	int flag = 0;
	flag |= cv::CALIB_FIX_INTRINSIC;
	cv::stereoCalibrate(
		objPts,
		imagePtsL,
		imagePtsR,
		newCameraMatrix1,
		distCoeffs1,
		newCameraMatrix2,
		distCoeffs2,
		frameL.size(),
		R,
		T,
		essentialMtx,
		fundamentalMtx,
		flag);

	cv::Mat Rfloat, Tfloat;
	R.convertTo(Rfloat, CV_32FC1);
	T.convertTo(Tfloat, CV_32FC1);

	RVL::Pose3D poseC0C1;
	std::memcpy(poseC0C1.R, Rfloat.ptr(0), Rfloat.total() * sizeof(float));
	std::memcpy(poseC0C1.t, Tfloat.ptr(0), Tfloat.total() * sizeof(float));
	std::fill(poseC_C0[0].R, poseC_C0[0].R + sizeof(poseC_C0[0].R), 0.0f); // fill array with 0's
	poseC_C0[0].R[0] = poseC_C0[0].R[4] = poseC_C0[0].R[8] = 1.0f;		   // 1's for identity matrix (camera does not have rotation wrt itself)
	std::fill(poseC_C0[0].t, poseC_C0[0].t + sizeof(poseC_C0[0].t), 0.0f); // fill array with 0's (no translation)
	RVLINVTRANSF3D(poseC0C1.R, poseC0C1.t, poseC_C0[1].R, poseC_C0[1].t);

	// Convert to Mat for saving
	cv::Mat TC0C1(4, 4, R.type());
	TC0C1(cv::Range(0, 3), cv::Range(0, 3)) = R * 1;
	TC0C1(cv::Range(0, 3), cv::Range(3, 4)) = T * 1;
	double *p = TC0C1.ptr<double>(3);
	p[0] = p[1] = p[2] = 0.0f;
	p[3] = 1.0f;

	cv::Mat TC1C0 = TC0C1.inv();
	cout << "TC1C0 = \n"
		 << TC1C0 << endl;

	// cv::FileStorage fsSave;
	// fsSave.open(imagesPath + slash + "extrinsicCalibration.yaml", cv::FileStorage::WRITE);
	// fsSave << "TC1C0" << TC1C0;
	// fsSave.release();
}

void DDDetector::Fit3DTo2D2(
	Mesh *pMesh,
	Array<RECOG::DDD::EdgeSample> edgeSamplePts,
	cv::Mat *edges,
	Pose3D *poseC_C0,
	Pose3D poseMC0,
	Array<RECOG::DDD::PtAssoc> ptAssoc,
	int maxnIterations,
	float &th_,
	float &th,
	float &et,
	float *dR,
	float *dt,
	float maxdth,
	bool bDebug)
{
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
	cv::Mat displayImg;
	float *PRMem;
	float *PCMem;
	float *NCMem;
	RECOG::DDD::Edge *edge;
	if (pVisualizationData->bVisualizeICPSteps)
	{
		edge = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
		PRMem = new float[pMesh->NodeArray.n * 3];
		PCMem = new float[pMesh->NodeArray.n * 3];
		NCMem = new float[pMesh->faces.n * 3];
	}
#endif
	cv::Mat cva(6, 1, CV_64FC1);
	double *a_ = (double *)(cva.data);
	double *at_ = a_ + 3;
	cv::Mat cvM(6, 6, CV_64FC1);
	double *M_ = (double *)(cvM.data);
	cv::Mat cvx(6, 1, CV_64FC1);
	double *x_ = (double *)(cvx.data);
	double *xt_ = x_ + 3;
	cv::Mat cvc(6, 1, CV_64FC1);
	double *c_ = (double *)(cvc.data);
	int i;
	RECOG::DDD::PtAssoc *pPtAssoc;
	int iQPix;
	double e;
	RECOG::DDD::EdgeSample *pSamplePt;
	// float* QN;
	float PR[3], MP[3];
	float Qu, Qv;
	float QP[3];
	// QP[2] = 1.0f;
	// RVLCOPY3VECTOR(poseC_C0.t, QP);
	float V3Tmp[3];
	// float maxe = 0.0f;
	RVLUNITMX3(dR);
	RVLNULL3VECTOR(dt);
	float dR0[9], dR0Prev[9];
	float dt0[3], dt0Prev[3];
	float E0;
	float EPrev = 0.0f;
	float rotAxis[3];
	float dE, meande;
	// int nStepReductions;

	// RECOG::DDD::PtAssoc* firstElementPtAssoc;
	// int nDefDoFs = 10;
	// int numImg = 2;
	// Pose3D poseMCc;
	// float tmp3x1[3];

	// Add edges map to visualization data
	// pVisualizationData->edges1 = &edges;

	for (int iIteration = 0; iIteration < maxnIterations; iIteration++)
	{
		RVLCOPYMX3X3(dR, dR0);
		RVLCOPY3VECTOR(dt, dt0);
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
		if (pVisualizationData->bVisualizeICPSteps)
		{
			Pose3D displayPose;
			displayPose = poseMC0;
			RVLMXMUL3X3(dR0, poseMC0.R, displayPose.R);
			RVLSUM3VECTORS(poseMC0.t, dt0, displayPose.t);
			Project3DModelToImage(pMesh, displayPose, edge, PRMem, PCMem, NCMem);
			// cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
			cv::cvtColor(edges[0], displayImg, cv::COLOR_GRAY2RGB); // here only for error avoidance, needs changing
			Visualize3DModelImageProjection(displayImg, pMesh, edge);
		}
#endif
		memset(M_, 0, 6 * 6 * sizeof(double));
		memset(c_, 0, 6 * sizeof(double));
		E0 = 0.0f; // Only for debugging purpose!!!

		// ptAssoc.n = nDefDoFs/2;
		// firstElementPtAssoc = ptAssoc.Element;
		// for(int iImg = 0; iImg < numImg; iImg++)
		// {
		// 	RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
		// 	Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem);
		// 	SampleEdges(pMesh, edge, edgeSamplePts);

		// }

		for (i = 0; i < ptAssoc.n; i++)
		{
			pPtAssoc = ptAssoc.Element + i;
			RVLCOPY3VECTOR(poseC_C0[pPtAssoc->iImg].t, QP);
			pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
			iQPix = pPtAssoc->iQPix;
			RVLMULMX3X3VECT(dR0, pSamplePt->PR, PR);
			RVLCROSSPRODUCT3(PR, pPtAssoc->QN, a_);
			RVLCOPY3VECTOR(pPtAssoc->QN, at_);
			cvM += cva * cva.t();
			// Qu = (float)(iQPix % camera.w);
			// Qv = (float)(iQPix / camera.w);
			// QP[0] = (Qu - camera.uc) / camera.fu;
			// QP[1] = (Qv - camera.vc) / camera.fv;
			RVLSUM3VECTORS(PR, poseMC0.t, MP);
			RVLSUM3VECTORS(MP, dt0, MP);
			RVLDIF3VECTORS(QP, MP, V3Tmp);
			e = RVLDOTPRODUCT3(pPtAssoc->QN, V3Tmp);
			// if (RVLABS(e) > maxe)
			//	maxe = RVLABS(e);
			cvc += e * cva;
			E0 += (e * e); // Only for debugging purpose!!!
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
			if (pVisualizationData->bVisualizeICPSteps)
			{
				float fTmp = RVLDOTPRODUCT3(pPtAssoc->QN, QP);
				int Mu = (int)round(camera.fu * MP[0] / MP[2] + camera.uc);
				int Mv = (int)round(camera.fv * MP[1] / MP[2] + camera.vc);
				RVLSCALE3VECTOR(pPtAssoc->QN, e, V3Tmp);
				float QP_[3];
				RVLSUM3VECTORS(MP, V3Tmp, QP_);
				int Qu_ = (int)round(camera.fu * QP_[0] / QP_[2] + camera.uc);
				int Qv_ = (int)round(camera.fv * QP_[1] / QP_[2] + camera.vc);
				cv::line(displayImg, cv::Point(Mu, Mv), cv::Point(Qu_, Qv_), cv::Scalar(0, 0, 255));
			}
#endif
		}
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
		if (pVisualizationData->bVisualizeICPSteps)
		{
			cv::imshow("point association", displayImg);
			cv::waitKey(0);
		}
#endif
		dE = E0 - EPrev;
		if (dE <= 0.0f || iIteration == 0)
		{
			if (iIteration > 0)
			{
				meande = sqrt(EPrev / (float)(ptAssoc.n)) - sqrt(E0 / (float)(ptAssoc.n));
				if (meande < 1e-4)
					break;
			}
#ifdef RVLDDDFIT3DTO2D_VERBOSE
			printf("E=%f ", E0);
			if (iIteration > 0)
				printf("meande=%f", meande);
			printf("\n");
#endif
			EPrev = E0;
			RVLCOPYMX3X3(dR0, dR0Prev);
			RVLCOPY3VECTOR(dt0, dt0Prev);
			for (i = 0; i < 6; i++)
				M_[7 * i] += (i < 3 ? fit3DTo2DLambdaR : fit3DTo2DLambdat);
			cv::solve(cvM, cvc, cvx);
			// nStepReductions = 0;
		}
		else
		{
			cvx = 0.5 * cvx;
			RVLCOPYMX3X3(dR0Prev, dR0);
			RVLCOPY3VECTOR(dt0Prev, dt0);
			iIteration--;
			// nStepReductions++;
		}

		th_ = sqrt(RVLDOTPRODUCT3(x_, x_));
		RVLSCALE3VECTOR2(x_, th_, rotAxis);

		// if (th_ > 30.0 * DEG2RAD)
		//{
		//	s = 30 * DEG2RAD / th_;
		//	th_ = 30.0 * DEG2RAD;
		//	RVLSCALE3VECTOR(xt, s, xt);
		// }

		// When debugging with linear approximate rotation, comment the following 5 lines.

		if (th_ > 1e-4)
		{
			if (th_ > maxdth)
			{
				cvx = (cvx * maxdth / th_);
				th_ = maxdth;
			}
			float dR_[9];
			AngleAxisToRot<float>(rotAxis, th_, dR_);
			float M3x3Tmp[9];
			RVLMXMUL3X3(dR_, dR0, M3x3Tmp);
			GetAngleAxis(M3x3Tmp, rotAxis, th);
			AngleAxisToRot<float>(rotAxis, th, dR);
		}
		else
		{
			RVLCOPYMX3X3(dR0, dR);
			GetAngleAxis(dR, rotAxis, th);
		}

		// Only for debugging purpose: linear approximate rotation.

		// RVLSKEW(V3Tmp, dR);
		// float I[9];
		// RVLUNITMX3(I);
		// RVLSUMMX3X3(I, dR, dR);
		// RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
		// th = th_;

		RVLSUM3VECTORS(dt0, xt_, dt);
		et = sqrt(RVLDOTPRODUCT3(dt, dt));
#ifdef RVLDDDFIT3DTO2D_VERBOSE
		printf("dth=%f th=%f dt=%f\n", th_ * RAD2DEG, th * RAD2DEG, et);
#endif
		///

		if (bDebug)
		{
			/// Only for debugging purpose!!!

			// Non-linear error.

			float E = 0.0f; // Only for debugging purpose!!!
			float PR0[3];
			for (i = 0; i < ptAssoc.n; i++)
			{
				pPtAssoc = ptAssoc.Element + i;
				pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
				iQPix = pPtAssoc->iQPix;

				// pPtAssoc->QN[0] = (QNMap[iImg] + 3 * iQPix)[0];
				// pPtAssoc->QN[1] = (QNMap[iImg] + 3 * iQPix)[1];
				// pPtAssoc->QN[2] = (QNMap[iImg] + 3 * iQPix)[2];

				// QN = QNMap[iImg] + 3 * iQPix;
				RVLMULMX3X3VECT(dR, pSamplePt->PR, PR);
				// RVLMULMX3X3VECT(dR0, pSamplePt->PR, PR0);
				//  Qu = (float)(iQPix % camera.w);
				//  Qv = (float)(iQPix / camera.w);
				//  QP[0] = (Qu - camera.uc) / camera.fu;
				//  QP[1] = (Qv - camera.vc) / camera.fv;
				RVLDIF3VECTORS(QP, PR, V3Tmp);
				RVLDIF3VECTORS(V3Tmp, poseMC0.t, V3Tmp);
				RVLDIF3VECTORS(V3Tmp, dt, V3Tmp);
				e = RVLDOTPRODUCT3(pPtAssoc->QN, V3Tmp);
				E += (e * e); // Only for debugging purpose!!!
			}
			float Ex = fit3DTo2DLambdaR * RVLDOTPRODUCT3(x_, x_) + fit3DTo2DLambdat * RVLDOTPRODUCT3(xt_, xt_);
			E += Ex;

			// Linear error correction.

			cv::Mat cvdeLin = cvx.t() * cvM * cvx - 2.0 * cvc.t() * cvx;

			// Print errors.

			printf("E0 = %f, NonLin. error corr. = %f, Lin. error corr. = %f\n", E0, E - E0, *(double *)(cvdeLin.data));

			cv::waitKey(0);
		}

		///
	}
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
	if (pVisualizationData->bVisualizeICPSteps)
	{
		delete[] edge;
		delete[] PRMem;
		delete[] PCMem;
		delete[] NCMem;
	}
#endif
}

void DDDetector::Fit3DTo2D2S(
	Mesh *pMesh,
	Array<RECOG::DDD::EdgeSample> edgeSamplePts,
	cv::Mat *edges,
	RVL::RECOG::DDD::Edge *edge,
	Pose3D *poseC_C0,
	Pose3D poseMC0,
	Array<RECOG::DDD::PtAssoc> ptAssoc,
	float *PRMem,
	float *PCMem,
	float *NCMem,
	int maxnIterations,
	float &th_,
	float &th,
	float &et,
	float *dR,
	float *dt,
	float maxdth,
	bool bDebug)
{

	// #ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
	// #ifdef NEVER
	// cv::Mat displayImg;
	// float* PRMem;
	// float* PCMem;
	// float* NCMem;
	// RECOG::DDD::Edge* edge;
	// Array<RECOG::DDD::EdgeSample> edgeSamplePts;

	// if (pVisualizationData->bVisualizeICPSteps)
	if (false)
	{
		edge = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
		PRMem = new float[pMesh->NodeArray.n * 3]; // Number of vertex in mesh
		PCMem = new float[pMesh->NodeArray.n * 3]; // Model points
		NCMem = new float[pMesh->faces.n * 3];	   // Face normals
	}
	// #endif
	cv::Mat cva(6, 1, CV_64FC1);
	double *a_ = (double *)(cva.data);
	double *at_ = a_ + 3;
	cv::Mat cvM(6, 6, CV_64FC1);
	double *M_ = (double *)(cvM.data);
	cv::Mat cvx(6, 1, CV_64FC1);
	double *x_ = (double *)(cvx.data);
	double *xt_ = x_ + 3;
	cv::Mat cvc(6, 1, CV_64FC1);
	double *c_ = (double *)(cvc.data);
	int i;
	RECOG::DDD::PtAssoc *pPtAssoc;
	int iQPix;
	double e;
	RECOG::DDD::EdgeSample *pSamplePt;
	// float* QN;
	float PR[3], MP[3];
	float Qu, Qv;
	float QP[3];
	// QP[2] = 1.0f;
	// RVLCOPY3VECTOR(poseC_C0.t, QP);
	float V3Tmp[3];
	// float maxe = 0.0f;
	RVLUNITMX3(dR);
	RVLNULL3VECTOR(dt);
	float dR0[9], dR0Prev[9];
	float dt0[3], dt0Prev[3];
	float E0;
	float EPrev = 0.0f;
	float rotAxis[3];
	float dE, meande;
	// int nStepReductions;

	RECOG::DDD::PtAssoc *firstElementPtAssoc;
	firstElementPtAssoc = ptAssoc.Element;
	int nDefDoFs = 10;
	int numImg = 2;
	Pose3D poseMCc;
	float tmp3x1[3];
	pVisualizationData->ICPDisplayImage[0] = edges[0].clone();
	pVisualizationData->ICPDisplayImage[1] = edges[1].clone();

	for (int iIteration = 0; iIteration < maxnIterations; iIteration++)
	{
		RVLCOPYMX3X3(dR, dR0);
		RVLCOPY3VECTOR(dt, dt0);
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
		if (pVisualizationData->bVisualizeICPSteps)
		{
			ptAssoc.Element = firstElementPtAssoc;
			ptAssoc.n = nDefDoFs / 2;

			cv::Mat visualizeICPStepsImg;

			for (int iImg = 0; iImg < numImg; iImg++)
			{
				ptAssoc.Element = ptAssoc.Element + 5 * iImg;
				ptAssoc.n = nDefDoFs / 2;

				RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC0.R, poseMC0.t, poseMCc.R, poseMCc.t, tmp3x1);
				Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem);
				SampleEdges(pMesh, edge, edgeSamplePts);

				Pose3D displayPose;
				displayPose = poseMCc;
				RVLMXMUL3X3(dR0, poseMCc.R, poseMCc.R);
				RVLSUM3VECTORS(poseMCc.t, dt0, poseMCc.t);

				// Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem);
				// cv::cvtColor(edges[iImg], pVisualizationData->displayImg, cv::COLOR_GRAY2RGB);
				// Visualize3DModelImageProjection(pVisualizationData->displayImg, pMesh, edge);
				// cv::imshow("VisualizeICPSteps", pVisualizationData->displayImg);
				// cv::waitKey(0);
				// cv::destroyWindow("VisualizeICPSteps");

				// RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC0.R, poseMC0.t, poseMCc.R, poseMCc.t, tmp3x1);
				// Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem);
				// SampleEdges(pMesh, edge, edgeSamplePts);

				cv::cvtColor(edges[iImg], visualizeICPStepsImg, cv::COLOR_GRAY2RGB);
				pVisualizationData->displayImg = &visualizeICPStepsImg;
				// SuperposeBinaryImage(pVisualizationData->displayImg, edges[iImg].data, yellow);
				VisualizeEdgeSamples(*pVisualizationData->displayImg, edgeSamplePts);
				// Visualize3DModelImageProjection(pVisualizationData->displayImg, pMesh, edge);
				Visualize2DPointAssociation(*pVisualizationData->displayImg, ptAssoc, edgeSamplePts.Element);
				cv::imshow("ICPAssocs", *pVisualizationData->displayImg);
				cv::waitKey(0);
				cv::destroyWindow("ICPAssocs");
			}
			ptAssoc.Element = firstElementPtAssoc;
			ptAssoc.n = nDefDoFs;
		}
#endif
		memset(M_, 0, 6 * 6 * sizeof(double));
		memset(c_, 0, 6 * sizeof(double));
		E0 = 0.0f; // Only for debugging purpose!!!

		ptAssoc.Element = firstElementPtAssoc;
		ptAssoc.n = nDefDoFs / 2;
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			ptAssoc.Element = ptAssoc.Element + 5 * iImg;
			RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC0.R, poseMC0.t, poseMCc.R, poseMCc.t, tmp3x1);
			Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem);
			SampleEdges(pMesh, edge, edgeSamplePts);
			for (i = 0; i < ptAssoc.n; i++)
			{
				pPtAssoc = ptAssoc.Element + i;
				RVLCOPY3VECTOR(poseC_C0[iImg].t, QP);
				pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
				iQPix = pPtAssoc->iQPix;
				RVLMULMX3X3VECT(dR0, pSamplePt->PR, PR);
				RVLCROSSPRODUCT3(PR, pPtAssoc->QN, a_);
				RVLCOPY3VECTOR(pPtAssoc->QN, at_);
				cvM += cva * cva.t();
				// Qu = (float)(iQPix % camera.w);
				// Qv = (float)(iQPix / camera.w);
				// QP[0] = (Qu - camera.uc) / camera.fu;
				// QP[1] = (Qv - camera.vc) / camera.fv;
				RVLSUM3VECTORS(PR, poseMCc.t, MP);
				RVLSUM3VECTORS(MP, dt0, MP);
				RVLDIF3VECTORS(QP, MP, V3Tmp);
				e = RVLDOTPRODUCT3(pPtAssoc->QN, V3Tmp);
				// if (RVLABS(e) > maxe)
				//	maxe = RVLABS(e);
				cvc += e * cva;
				E0 += (e * e); // Only for debugging purpose!!!
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
				if (pVisualizationData->bVisualizeICPSteps)
				{
					float fTmp = RVLDOTPRODUCT3(pPtAssoc->QN, QP);
					int Mu = (int)round(camera.fu * MP[0] / MP[2] + camera.uc);
					int Mv = (int)round(camera.fv * MP[1] / MP[2] + camera.vc);
					RVLSCALE3VECTOR(pPtAssoc->QN, e, V3Tmp);
					float QP_[3];
					RVLSUM3VECTORS(MP, V3Tmp, QP_);
					int Qu_ = (int)round(camera.fu * QP_[0] / QP_[2] + camera.uc);
					int Qv_ = (int)round(camera.fv * QP_[1] / QP_[2] + camera.vc);
					cv::line(pVisualizationData->ICPDisplayImage[iImg], cv::Point(Mu, Mv), cv::Point(Qu_, Qv_), cv::Scalar(0, 0, 255));
				}
#endif
			}
		}
		ptAssoc.Element = firstElementPtAssoc;
		ptAssoc.n = nDefDoFs;

#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			if (pVisualizationData->bVisualizeICPSteps)
			{
				cv::imshow("point association", pVisualizationData->ICPDisplayImage[iImg]);
				cv::waitKey(0);
			}
		}
#endif
		dE = E0 - EPrev;
		if (dE <= 0.0f || iIteration == 0)
		{
			if (iIteration > 0)
			{
				meande = sqrt(EPrev / (float)(ptAssoc.n)) - sqrt(E0 / (float)(ptAssoc.n));
				if (meande < 1e-4)
					break;
			}
#ifdef RVLDDDFIT3DTO2D_VERBOSE
			printf("E=%f ", E0);
			if (iIteration > 0)
				printf("meande=%f", meande);
			printf("\n");
#endif
			EPrev = E0;
			RVLCOPYMX3X3(dR0, dR0Prev);
			RVLCOPY3VECTOR(dt0, dt0Prev);
			for (i = 0; i < 6; i++)
				M_[7 * i] += (i < 3 ? fit3DTo2DLambdaR : fit3DTo2DLambdat);
			cv::solve(cvM, cvc, cvx);
			// nStepReductions = 0;
		}
		else
		{
			cvx = 0.5 * cvx;
			RVLCOPYMX3X3(dR0Prev, dR0);
			RVLCOPY3VECTOR(dt0Prev, dt0);
			iIteration--;
			// nStepReductions++;
		}

		th_ = sqrt(RVLDOTPRODUCT3(x_, x_));
		RVLSCALE3VECTOR2(x_, th_, rotAxis);

		// if (th_ > 30.0 * DEG2RAD)
		//{
		//	s = 30 * DEG2RAD / th_;
		//	th_ = 30.0 * DEG2RAD;
		//	RVLSCALE3VECTOR(xt, s, xt);
		// }

		// When debugging with linear approximate rotation, comment the following 5 lines.

		if (th_ > 1e-4)
		{
			if (th_ > maxdth)
			{
				cvx = (cvx * maxdth / th_);
				th_ = maxdth;
			}
			float dR_[9];
			AngleAxisToRot<float>(rotAxis, th_, dR_);
			float M3x3Tmp[9];
			RVLMXMUL3X3(dR_, dR0, M3x3Tmp);
			GetAngleAxis(M3x3Tmp, rotAxis, th);
			AngleAxisToRot<float>(rotAxis, th, dR);
		}
		else
		{
			RVLCOPYMX3X3(dR0, dR);
			GetAngleAxis(dR, rotAxis, th);
		}

		// Only for debugging purpose: linear approximate rotation.

		// RVLSKEW(V3Tmp, dR);
		// float I[9];
		// RVLUNITMX3(I);
		// RVLSUMMX3X3(I, dR, dR);
		// RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
		// th = th_;

		RVLSUM3VECTORS(dt0, xt_, dt);
		et = sqrt(RVLDOTPRODUCT3(dt, dt));
#ifdef RVLDDDFIT3DTO2D_VERBOSE
		printf("dth=%f th=%f dt=%f\n", th_ * RAD2DEG, th * RAD2DEG, et);
#endif
		///

		if (bDebug)
		{
			/// Only for debugging purpose!!!

			// Non-linear error.

			float E = 0.0f; // Only for debugging purpose!!!
			float PR0[3];
			for (i = 0; i < ptAssoc.n; i++)
			{
				pPtAssoc = ptAssoc.Element + i;
				pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
				iQPix = pPtAssoc->iQPix;

				// pPtAssoc->QN[0] = (QNMap[iImg] + 3 * iQPix)[0];
				// pPtAssoc->QN[1] = (QNMap[iImg] + 3 * iQPix)[1];
				// pPtAssoc->QN[2] = (QNMap[iImg] + 3 * iQPix)[2];

				// QN = QNMap[iImg] + 3 * iQPix;
				RVLMULMX3X3VECT(dR, pSamplePt->PR, PR);
				// RVLMULMX3X3VECT(dR0, pSamplePt->PR, PR0);
				//  Qu = (float)(iQPix % camera.w);
				//  Qv = (float)(iQPix / camera.w);
				//  QP[0] = (Qu - camera.uc) / camera.fu;
				//  QP[1] = (Qv - camera.vc) / camera.fv;
				RVLDIF3VECTORS(QP, PR, V3Tmp);
				RVLDIF3VECTORS(V3Tmp, poseMC0.t, V3Tmp);
				RVLDIF3VECTORS(V3Tmp, dt, V3Tmp);
				e = RVLDOTPRODUCT3(pPtAssoc->QN, V3Tmp);
				E += (e * e); // Only for debugging purpose!!!
			}
			float Ex = fit3DTo2DLambdaR * RVLDOTPRODUCT3(x_, x_) + fit3DTo2DLambdat * RVLDOTPRODUCT3(xt_, xt_);
			E += Ex;

			// Linear error correction.

			cv::Mat cvdeLin = cvx.t() * cvM * cvx - 2.0 * cvc.t() * cvx;

			// Print errors.

			printf("E0 = %f, NonLin. error corr. = %f, Lin. error corr. = %f\n", E0, E - E0, *(double *)(cvdeLin.data));

			cv::waitKey(0);
		}

		///
	}
	// #ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
#ifdef NEVER
	if (pVisualizationData->bVisualizeICPSteps)
	{
		delete[] edge;
		delete[] PRMem;
		delete[] PCMem;
		delete[] NCMem;
	}
#endif
}

// #define RVLDDDFIT3DTO2D_ENGINE_VERBOSE

void DDDetector::Fit3DTo2D2SS(
	Mesh *pMesh,
	// Array<RECOG::DDD::EdgeSample> edgeSamplePts,
	cv::Mat *edges,
	RVL::RECOG::DDD::Edge *edge,
	Pose3D *poseC_C0,
	Pose3D poseMC0In,
	Array<RECOG::DDD::PtAssoc> ptAssoc,
	// float* PRMem,
	// float* PCMem,
	// float* NCMem,
	int maxnIterations,
	float &th_,
	float &th,
	float &et,
	float *dR,
	float *dt,
	float maxdth,
	bool bDebug)
{
	Pose3D poseMC0 = poseMC0In;
	// #ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
	// #ifdef NEVER
	cv::Mat displayImg;
	float *PRMem;
	float *PCMem;
	float *NCMem;
	// RECOG::DDD::Edge* edge;
	Array<RECOG::DDD::EdgeSample> edgeSamplePts[2];

	// if (pVisualizationData->bVisualizeICPSteps)
	if (true)
	{
		// edge = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
		PRMem = new float[pMesh->NodeArray.n * 3]; // Number of vertex in mesh
		PCMem = new float[pMesh->NodeArray.n * 3]; // Model points
		NCMem = new float[pMesh->faces.n * 3];	   // Face normals
	}
	// #endif
	cv::Mat cva(6, 1, CV_64FC1);
	double *a_ = (double *)(cva.data);
	double *at_ = a_ + 3;
	cv::Mat cvM(6, 6, CV_64FC1);
	double *M_ = (double *)(cvM.data);
	cv::Mat cvx(6, 1, CV_64FC1);
	double *x_ = (double *)(cvx.data);
	double *xt_ = x_ + 3;
	cv::Mat cvc(6, 1, CV_64FC1);
	double *c_ = (double *)(cvc.data);
	int i;
	RECOG::DDD::PtAssoc *pPtAssoc;
	int iQPix;
	double e;
	RECOG::DDD::EdgeSample *pSamplePt;
	// float* QN;
	float PR[3], MP[3];
	float Qu, Qv;
	float QP[3];
	// QP[2] = 1.0f;
	// RVLCOPY3VECTOR(poseC_C0.t, QP);
	float V3Tmp[3];
	// float maxe = 0.0f;
	RVLUNITMX3(dR);
	RVLNULL3VECTOR(dt);
	float dR0[9], dR0Prev[9];
	float dt0[3], dt0Prev[3];
	float E0;
	float EPrev = 0.0f;
	float rotAxis[3];
	float dE, meande;
	// int nStepReductions;

	RECOG::DDD::PtAssoc *firstElementPtAssoc;
	firstElementPtAssoc = ptAssoc.Element;
	int nDefDoFs = 10;
	int numImg = 2;
	Pose3D poseMCc;
	float tmp3x1[3];
	// pVisualizationData->ICPDisplayImage[0] = edges[0].clone();
	// pVisualizationData->ICPDisplayImage[1] = edges[1].clone();

	cv::cvtColor(edges[0], pVisualizationData->ICPDisplayImage[0], cv::COLOR_GRAY2RGB);
	cv::cvtColor(edges[1], pVisualizationData->ICPDisplayImage[1], cv::COLOR_GRAY2RGB);

	for (int iImg = 0; iImg < numImg; iImg++)
	{
		RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC0.R, poseMC0.t, poseMCc.R, poseMCc.t, tmp3x1);
		Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
		SampleEdges2(pMesh, edge, edgeSamplePts[iImg], iImg);
	}

	for (int iIteration = 0; iIteration < maxnIterations; iIteration++)
	{
		RVLCOPYMX3X3(dR, dR0);
		RVLCOPY3VECTOR(dt, dt0);

		RVLMXMUL3X3(dR0, poseMC0In.R, poseMC0.R);
		RVLSUM3VECTORS(poseMC0In.t, dt0, poseMC0.t);
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
		if (pVisualizationData->bVisualizeICPSteps)
		{
			// ptAssoc.Element = firstElementPtAssoc;
			// ptAssoc.n = nDefDoFs/2;

			for (int iImg = 0; iImg < numImg; iImg++)
			{
				// ptAssoc.Element = ptAssoc.Element + 5*iImg;
				// ptAssoc.n = nDefDoFs/2;
				// Pose3D displayPose;
				// displayPose = poseMC0;

				RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC0.R, poseMC0.t, poseMCc.R, poseMCc.t, tmp3x1);
				// Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
				// SampleEdges(pMesh, edge, edgeSamplePts, iImg);

				Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
				cv::cvtColor(edges[iImg], pVisualizationData->ICPDisplayImage[iImg], cv::COLOR_GRAY2RGB);
				Visualize3DModelImageProjection(pVisualizationData->ICPDisplayImage[iImg], pMesh, edge);
				cv::imshow("ICPAssocs", pVisualizationData->ICPDisplayImage[iImg]);
				cv::waitKey(0);
				cv::destroyWindow("ICPAssocs");
			}
			// ptAssoc.Element = firstElementPtAssoc;
			// ptAssoc.n = nDefDoFs;
		}
#endif
		memset(M_, 0, 6 * 6 * sizeof(double));
		memset(c_, 0, 6 * sizeof(double));
		E0 = 0.0f; // Only for debugging purpose!!!

		float QN[3];
		float MPR[3];
		float MPVis[3];

		for (i = 0; i < ptAssoc.n; i++)
		{
			pPtAssoc = ptAssoc.Element + i;

			// RVLCOMPTRANSF3DWITHINV(poseC_C0[pPtAssoc->iImg].R, poseC_C0[pPtAssoc->iImg].t, poseMC0.R, poseMC0.t, poseMCc.R, poseMCc.t, tmp3x1);
			// Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
			// SampleEdges(pMesh, edge, edgeSamplePts, iImg);

			pSamplePt = edgeSamplePts[pPtAssoc->iImg].Element + pPtAssoc->iMSample;
			RVLMULMX3X3VECT(poseC_C0[pPtAssoc->iImg].R, pPtAssoc->QN, QN);
			RVLMULMX3X3VECT(poseC_C0[pPtAssoc->iImg].R, pSamplePt->PR, MPR);

			RVLCOPY3VECTOR(poseC_C0[pPtAssoc->iImg].t, QP);
			iQPix = pPtAssoc->iQPix;
			RVLMULMX3X3VECT(dR0, MPR, PR);
			// RVLCOPY3VECTOR(MPR, PR);
			RVLCROSSPRODUCT3(PR, QN, a_);
			RVLCOPY3VECTOR(QN, at_);
			cvM += cva * cva.t();
			// Qu = (float)(iQPix % cameras[pPtAssoc->iImg].w);
			// Qv = (float)(iQPix / cameras[pPtAssoc->iImg].w);
			// QP[0] = (Qu - cameras[pPtAssoc->iImg].uc) / cameras[pPtAssoc->iImg].fu;
			// QP[1] = (Qv - cameras[pPtAssoc->iImg].vc) / cameras[pPtAssoc->iImg].fv;
			RVLSUM3VECTORS(PR, poseMC0In.t, MP);
			RVLSUM3VECTORS(MP, dt0, MP);
			RVLDIF3VECTORS(QP, MP, V3Tmp);
			e = RVLDOTPRODUCT3(QN, V3Tmp);
			// if (RVLABS(e) > maxe)
			//	maxe = RVLABS(e);
			cvc += (e * cva);
			E0 += (e * e); // Only for debugging purpose!!!
#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
			if (pVisualizationData->bVisualizeICPSteps)
			{
				RVLINVTRANSF3(MP, poseC_C0[pPtAssoc->iImg].R, poseC_C0[pPtAssoc->iImg].t, MPVis, V3Tmp);
				// float fTmp = RVLDOTPRODUCT3(QN, QP);
				int Mu = (int)round(cameras[pPtAssoc->iImg].fu * MPVis[0] / MPVis[2] + cameras[pPtAssoc->iImg].uc);
				int Mv = (int)round(cameras[pPtAssoc->iImg].fv * MPVis[1] / MPVis[2] + cameras[pPtAssoc->iImg].vc);
				RVLSCALE3VECTOR(pPtAssoc->QN, e, V3Tmp);
				float QP_[3];
				RVLSUM3VECTORS(MPVis, V3Tmp, QP_);
				int Qu_ = (int)round(cameras[pPtAssoc->iImg].fu * QP_[0] / QP_[2] + cameras[pPtAssoc->iImg].uc);
				int Qv_ = (int)round(cameras[pPtAssoc->iImg].fv * QP_[1] / QP_[2] + cameras[pPtAssoc->iImg].vc);
				cv::line(pVisualizationData->ICPDisplayImage[pPtAssoc->iImg], cv::Point(Mu, Mv), cv::Point(Qu_, Qv_), cv::Scalar(0, 0, 255));
				cv::circle(pVisualizationData->ICPDisplayImage[pPtAssoc->iImg], cv::Point(Mu, Mv), 3, cv::Scalar(0, 255, 0), 2);
				cv::circle(pVisualizationData->ICPDisplayImage[pPtAssoc->iImg], cv::Point(Qu_, Qv_), 3, cv::Scalar(0, 255, 255), 2);
				// cout << "Mu, Mv: " << Mu << ", " << Mv << endl;
				// cout << "Qu, Qv: " << Qu_ << ", " << Qv_ << endl;
				// cv::imshow("point association", pVisualizationData->ICPDisplayImage[pPtAssoc->iImg]);
				// cv::waitKey(0);
				// if(i == 0 || i == 5)
				// 	VisualizeEdgeSamples(pVisualizationData->ICPDisplayImage[pPtAssoc->iImg], edgeSamplePts);
			}
#endif
		}

		// ptAssoc.Element = firstElementPtAssoc;
		// ptAssoc.n = nDefDoFs;

#ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			if (pVisualizationData->bVisualizeICPSteps)
			{
				cv::imshow("point association", pVisualizationData->ICPDisplayImage[iImg]);

				cv::waitKey(0);
			}
		}
#endif
		dE = E0 - EPrev;
		if (dE <= 0.0f || iIteration == 0)
		{
			if (iIteration > 0)
			{
				meande = sqrt(EPrev / (float)(ptAssoc.n)) - sqrt(E0 / (float)(ptAssoc.n));
				if (meande < 1e-4)
					break;
			}
#ifdef RVLDDDFIT3DTO2D_ENGINE_VERBOSE
			printf("E=%f ", E0);
			if (iIteration > 0)
				printf("meande=%f", meande);
			printf("\n");
#endif
			EPrev = E0;
			RVLCOPYMX3X3(dR0, dR0Prev);
			RVLCOPY3VECTOR(dt0, dt0Prev);
			for (i = 0; i < 6; i++)
				M_[7 * i] += (i < 3 ? fit3DTo2DLambdaR : fit3DTo2DLambdat);
			cv::solve(cvM, cvc, cvx);
			// nStepReductions = 0;
		}
		else
		{
			cvx = 0.5 * cvx;
			RVLCOPYMX3X3(dR0Prev, dR0);
			RVLCOPY3VECTOR(dt0Prev, dt0);
			iIteration--;
			// nStepReductions++;
		}

		th_ = sqrt(RVLDOTPRODUCT3(x_, x_));
		RVLSCALE3VECTOR2(x_, th_, rotAxis);

		// if (th_ > 30.0 * DEG2RAD)
		//{
		//	s = 30 * DEG2RAD / th_;
		//	th_ = 30.0 * DEG2RAD;
		//	RVLSCALE3VECTOR(xt, s, xt);
		// }

		// When debugging with linear approximate rotation, comment the following 5 lines.

		if (th_ > 1e-4)
		{
			if (th_ > maxdth)
			{
				cvx = (cvx * maxdth / th_);
				th_ = maxdth;
			}
			float dR_[9];
			AngleAxisToRot<float>(rotAxis, th_, dR_);
			float M3x3Tmp[9];
			RVLMXMUL3X3(dR_, dR0, M3x3Tmp);
			GetAngleAxis(M3x3Tmp, rotAxis, th);
			AngleAxisToRot<float>(rotAxis, th, dR);
		}
		else
		{
			RVLCOPYMX3X3(dR0, dR);
			GetAngleAxis(dR, rotAxis, th);
		}

		// Only for debugging purpose: linear approximate rotation.

		// RVLSKEW(V3Tmp, dR);
		// float I[9];
		// RVLUNITMX3(I);
		// RVLSUMMX3X3(I, dR, dR);
		// RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
		// th = th_;

		RVLSUM3VECTORS(dt0, xt_, dt);
		et = sqrt(RVLDOTPRODUCT3(dt, dt));
#ifdef RVLDDDFIT3DTO2D_ENGINE_VERBOSE
		printf("dth=%f th=%f dt=%f\n", th_ * RAD2DEG, th * RAD2DEG, et);
#endif
		///

		if (bDebug)
		{
			/// Only for debugging purpose!!!

			// Non-linear error.

			float E = 0.0f; // Only for debugging purpose!!!
			float PR0[3];
			for (i = 0; i < ptAssoc.n; i++)
			{
				pPtAssoc = ptAssoc.Element + i;
				pSamplePt = edgeSamplePts[pPtAssoc->iImg].Element + pPtAssoc->iMSample;
				iQPix = pPtAssoc->iQPix;

				// pPtAssoc->QN[0] = (QNMap[iImg] + 3 * iQPix)[0];
				// pPtAssoc->QN[1] = (QNMap[iImg] + 3 * iQPix)[1];
				// pPtAssoc->QN[2] = (QNMap[iImg] + 3 * iQPix)[2];

				// QN = QNMap[iImg] + 3 * iQPix;
				RVLMULMX3X3VECT(dR, MPR, PR);
				// RVLMULMX3X3VECT(dR0, MPR, PR0);
				//  Qu = (float)(iQPix % cameras[pPtAssoc->iImg].w);
				//  Qv = (float)(iQPix / cameras[pPtAssoc->iImg].w);
				//  QP[0] = (Qu - cameras[pPtAssoc->iImg].uc) / cameras[pPtAssoc->iImg].fu;
				//  QP[1] = (Qv - cameras[pPtAssoc->iImg].vc) / cameras[pPtAssoc->iImg].fv;
				RVLDIF3VECTORS(QP, PR, V3Tmp);
				RVLDIF3VECTORS(V3Tmp, poseMC0.t, V3Tmp);
				RVLDIF3VECTORS(V3Tmp, dt, V3Tmp);
				e = RVLDOTPRODUCT3(pPtAssoc->QN, V3Tmp);
				E += (e * e); // Only for debugging purpose!!!
			}
			float Ex = fit3DTo2DLambdaR * RVLDOTPRODUCT3(x_, x_) + fit3DTo2DLambdat * RVLDOTPRODUCT3(xt_, xt_);
			E += Ex;

			// Linear error correction.

			cv::Mat cvdeLin = cvx.t() * cvM * cvx - 2.0 * cvc.t() * cvx;

			// Print errors.

			printf("E0 = %f, NonLin. error corr. = %f, Lin. error corr. = %f\n", E0, E - E0, *(double *)(cvdeLin.data));

			cv::waitKey(0);
		}

		///
	}
	// #ifdef RVLDDDFIT3DTO2D_VISUALIZE_ICP
	// #ifdef NEVER
	// if (pVisualizationData->bVisualizeICPSteps)
	{
		// delete[] edge;
		delete[] PRMem;
		delete[] PCMem;
		delete[] NCMem;
		delete[] edgeSamplePts[0].Element;
		delete[] edgeSamplePts[1].Element;
	}
	// #endif
}

void DDDetector::Fit3DTo2DStereo(
	Mesh *pMesh,
	cv::Mat *RGBs,
	int numImg,
	Array<Pose3D> initPosesMC,
	Pose3D *poseC_C0,
	Pose3D &bestPoseMC,
	cv::Mat *&solutions,
	int counter)
{
	// Create edges and compute their lengths.

	RECOG::DDD::Edge *edge = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
	RECOG::DDD::Edge *pEdge_;
	MeshEdge *pEdge;
	int iEdge;
	Point *pPt, *pPt_;
	float V3Tmp[3];
	for (iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		pEdge = pMesh->EdgeArray.Element + iEdge;
		pPt = pMesh->NodeArray.Element + pEdge->iVertex[0];
		pPt_ = pMesh->NodeArray.Element + pEdge->iVertex[1];
		pEdge_ = edge + iEdge;
		RVLDIF3VECTORS(pPt_->P, pPt->P, V3Tmp);
		pEdge_->length = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
		pEdge_->iVertex[0] = pEdge->iVertex[0];
		pEdge_->iVertex[1] = pEdge->iVertex[1];
	}

	// Visualization stuff
	uchar yellow[] = {0, 255, 255};

	cv::Mat edges[2];
	double **IuMap = new double *[numImg];
	double **IvMap = new double *[numImg];
	float **QNMap; // = new float*[numImg];

	float *QN;
	int nPix = RGBs[0].cols * RGBs[0].rows;
	float *QImgNMap = new float[2 * nPix];
	float *QImgN = QImgNMap;

	for (int iImg = 0; iImg < numImg; iImg++)
	{
		IuMap[iImg] = new double[nPix];
		IvMap[iImg] = new double[nPix];

		// QNMap[iImg] = new float[3*nPix];
	}

	setupFit3DTo2D(numImg, RGBs, edges, QImgNMap, IuMap, IvMap, QNMap);

	// Project model onto the image. -- allocation
	float *PRMem = new float[pMesh->NodeArray.n * 3];
	float *PCMem = new float[pMesh->NodeArray.n * 3];
	float *NCMem = new float[pMesh->faces.n * 3];
	float PRMem_[6 * 3];

	// // Initialize visualization.
	// // Visualize initial pose.
	// cv::Mat displayImg1, displayImg2;
	// // cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
	// // cv::cvtColor(EDTDisplayImage, displayImg, cv::COLOR_GRAY2RGB);
	// displayImg1 = RGBs[0].clone();
	// displayImg2 = RGBs[1].clone();

	// for (int iImg = 0; iImg < numImg; iImg++)
	// {
	// 	for (int iPose = 0; iPose < 4; iPose++)
	// 	{
	// 		pVisualizationData->displayImg = RGBs[iImg].clone();
	// 		SuperposeBinaryImage(pVisualizationData->displayImg, edges[iImg].data, yellow);
	// 		Visualize3DModelImageProjection(pVisualizationData->displayImg, pMesh, edge);
	// 		cv::imshow("model" + std::to_string(iPose), pVisualizationData->displayImg);
	// 		cv::waitKey(0);
	// 	}
	// }

	// pVisualizationData->displayImg = RGBs[1].clone();
	// SuperposeBinaryImage(pVisualizationData->displayImg, edges[1].data, yellow);
	// Visualize3DModelImageProjection(pVisualizationData->displayImg, pMesh, edge);
	// cv::imshow("model2", pVisualizationData->displayImg);

	// Hypothesis generation loop.

	pseudornd.n = 100000;
	pseudornd.Element = new int[pseudornd.n];
	pSurfelDetector->RandomIndices(pseudornd);
	iRnd = 0;

	Array<RECOG::DDD::EdgeSample> edgeSamplePts;
	int nEdgeSamples = fit3DTo2DNoEdgeSamples;
	int iSample;
	RECOG::DDD::EdgeSample *pSamplePt;
	bool bHoughLines = true;
	RECOG::DDD::Edge *edge_ = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
	Array<RECOG::DDD::EdgeSample> edgeSamplePts_;
	edgeSamplePts_.Element = new RECOG::DDD::EdgeSample[fit3DTo2DNoEdgeSamples];
	edgeSamplePts_.n = nEdgeSamples;
	float maxOrientationCorrection = fit3DTo2DMaxOrientationCorrection * DEG2RAD;
	int nInitSolutionIterations = 3;
	int nTrials = 20;
	float minDist = 20.0f;
	float eThr = 0.2f;
	float eN = 30.0f;
	float rotStD = 0.125 * PI; // rad
	float tStD = 0.05;		   // m
	float chamferThr = 3.0f;   // pix
	int i;
	float csNThr = cos(eN * DEG2RAD);
	float minDist2 = minDist * minDist;
	float chamferThr2 = chamferThr * chamferThr;
	int iIteration, iIteration2, j, iSample_, iQPix, iQPix_, iTrial;
	RECOG::DDD::PtAssoc *ptAssoc = new RECOG::DDD::PtAssoc[2 * fit3DTo2DNoEdgeSamples];
	RECOG::DDD::PtAssoc bestPtAssoc;
	RECOG::DDD::PtAssoc *pPtAssoc, *pPtAssoc_;
	RECOG::DDD::EdgeSample *pSamplePt_;
	RECOG::DDD::Edge *pEdge__;
	float *QImgN_;
	float distM, eM, distQ, eQ, csN;
	float MdImgP[2];
	float QImgNa[2];
	int QImgP[2], QImgP_[2];
	float QdImgP[2];
	float Q[36], R[36];
	memset(R, 0, 36 * sizeof(float));
	float *q, *qt, *q_, *qt_, *r, *PR;
	float x[6], y[6];
	float *xt = x + 3;
	float r_;
	float a[6], b[6];
	float *at = a + 3;
	float *bt = b + 3;
	float bestb[6], bestr[6];
	float *bestbt = bestb + 3;
	float e[6];
	float dR[9], dR0[9], dR_[9], M3x3Tmp[9];
	float th, th_, et;
	float rotAxis[3], dt[3], dt0[3];
	Pose3D poseMC_;
	bestPoseMC = initPosesMC.Element[0];
	float bestPosedR[9];
	float bestPosedt[3];
	float bestScore = 0.0f;
	int iBestHypothesis = -1;
	int nBestNumFPs = fit3DTo2DNoEdgeSamples;
	float signN;
	int nDefDoFs;
	cv::Mat cva(6, 1, CV_64FC1);
	double *a_ = (double *)(cva.data);
	double *at_ = a_ + 3;
	cv::Mat cvM(6, 6, CV_64FC1);
	double *M_ = (double *)(cvM.data);
	cv::Mat cve(6, 1, CV_64FC1);
	double *e_ = (double *)(cve.data);
	cv::Mat cvx(6, 1, CV_64FC1);
	double *x_ = (double *)(cvx.data);
	cv::Mat cvA(6, 6, CV_64FC1);
	double *A_ = (double *)(cvA.data);
	cv::Mat cvc(6, 1, CV_64FC1);
	double *c_ = (double *)(cvc.data);
	Array<RECOG::DDD::PtAssoc> ptAssocs;
	ptAssocs.Element = ptAssoc;
	std::vector<int> fifthAssocPtsHyps[2];
	std::vector<int> fifthMIdxs[2];
	// int fifthMIdxs[2][2];
	bool bVisualizeICPSteps = pVisualizationData->bVisualizeICPSteps;
	// pVisualizationData->bVisualizeICPSteps = false;
	int iHypothesis = 0;

	Pose3D poseMCc;
	float tmp3x1[3];
	RVL::Array<RVL::RECOG::DDD::PtAssoc> ptAssocsVis;
	ptAssocsVis.Element = ptAssoc;
	ptAssocsVis.n = 5;

	for (int iImg = 0; iImg < numImg; iImg++)
		for (int iAssoc = 0; iAssoc < 5; iAssoc++)
			ptAssocs.Element[5 * iImg + iAssoc].iImg = iImg;

	for (int iInitPose = 0; iInitPose < initPosesMC.n; iInitPose++)
	{
		Pose3D poseMC = initPosesMC.Element[iInitPose];
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			// pVisualizationData->displayImg = &RGBs[iImg].clone();

			RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
			Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
			SampleEdges2(pMesh, edge, edgeSamplePts, iImg);

			if (bHoughLines)
			{
				// cv::Mat edgesT;
				// cv::cvtColor(edges[iImg], edgesT, cv::COLOR_GRAY2RGB);

				// edgesT = edges[iImg].clone();
				// VisualizeEdgeSamples(edgesT, edgeSamplePts);
				// cv::imshow("Img model", edgesT);
				// cv::waitKey(0);

				// pVisualizationData->displayImg = &edges[iImg].clone();

				nDefDoFs = 10;
				RVL::DDDetector::HoughLinesPtAssociationLHTCP(
					edges[iImg],
					QNMap[iImg],
					IuMap[iImg],
					IvMap[iImg],
					pMesh,
					edge,
					cameras[iImg],
					pVisualizationData->bVisualizeHoughTransform,
					edgeSamplePts,
					fifthAssocPtsHyps[iImg],
					&ptAssoc[5 * iImg],
					fifthMIdxs[iImg]);
			}

			delete[] edgeSamplePts.Element;
		}

		int nHypGenIterations[2];
		nHypGenIterations[0] = 2 * fifthAssocPtsHyps[0].size();
		nHypGenIterations[1] = 2 * fifthAssocPtsHyps[1].size();
		int iFifthAssoc[2];
		for (iFifthAssoc[0] = 0; iFifthAssoc[0] < nHypGenIterations[0]; iFifthAssoc[0]++)
		{
			for (iFifthAssoc[1] = 0; iFifthAssoc[1] < nHypGenIterations[1]; iFifthAssoc[1]++)
			{
#ifdef RVLDDDFIT3DTO2D_VERBOSE
				printf("pose %d\/%d 5th_assoc. (%d\/%d, %d\/%d)\n", iInitPose, initPosesMC.n, iFifthAssoc[0], nHypGenIterations[0], iFifthAssoc[1], nHypGenIterations[1]);
#endif
				for (int iImg = 0; iImg < numImg; iImg++)
				{
					// if (iFifthAssoc == 15)
					//	int debug = 0;
					if (bHoughLines)
					{
						int idx = iFifthAssoc[iImg] / 2;
						ptAssoc[5 * (iImg + 1) - 1].iQPix = fifthAssocPtsHyps[iImg][idx];
						ptAssoc[5 * (iImg + 1) - 1].iMSample = fifthMIdxs[iImg][iFifthAssoc[iImg] % 2];
						// Set QN normals
						int iPtAssoc, iQN, idxAssoc;
						for (iPtAssoc = 0; iPtAssoc < nDefDoFs / 2; iPtAssoc++)
						{
							for (iQN = 0; iQN < 3; iQN++)
							{
								idxAssoc = 5 * iImg + iPtAssoc;
								ptAssoc[idxAssoc].QN[iQN] = (QNMap[iImg] + 3 * ptAssoc[idxAssoc].iQPix)[iQN];
							}
						}
					}
					else
					{
						for (i = 0; i < 6; i++)
						{
							// if (i == 5)
							//	int debug = 0;
							memset(bestr, 0, 6 * sizeof(float));
							for (iTrial = 0; iTrial < nTrials; iTrial++)
							{
								// if (i == 2 && iTrial == 3)
								//	int debug = 0;
								iSample = rand() % edgeSamplePts.n;
								pSamplePt = edgeSamplePts.Element + iSample;
								pEdge_ = edge + pSamplePt->edgeIdx;
								if (pVisualizationData->EDTImage[iImg].pPix[pSamplePt->iPix].d2 > pVisualizationData->EDT[iImg].m_maxd2)
									continue;
								QImgP[0] = pSamplePt->iImgP[0] - pVisualizationData->EDTImage[iImg].pPix[pSamplePt->iPix].dx;
								QImgP[1] = pSamplePt->iImgP[1] - pVisualizationData->EDTImage[iImg].pPix[pSamplePt->iPix].dz;
								iQPix = QImgP[0] + QImgP[1] * RGBs[iImg].cols;
								QImgN = QImgNMap + 2 * iQPix;
								csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
								if (csN >= csNThr)
									signN = 1.0f;
								else if (-csN >= csNThr)
									signN = -1.0f;
								else
									continue;
								QImgNa[0] = signN * QImgN[0];
								QImgNa[1] = signN * QImgN[1];
								for (j = 0; j < i; j++)
								{
									pPtAssoc_ = ptAssoc + j;
									iSample_ = pPtAssoc_->iMSample;
									pSamplePt_ = edgeSamplePts.Element + iSample_;
									pEdge__ = edge + pSamplePt_->edgeIdx;
									iQPix_ = pPtAssoc_->iQPix;
									QImgP_[0] = iQPix_ % RGBs[iImg].cols;
									QImgP_[1] = iQPix_ / RGBs[iImg].cols;
									QImgN_ = QImgNMap + 2 * iQPix_;
									MdImgP[0] = pSamplePt->ImgP[0] - pSamplePt_->ImgP[0];
									MdImgP[1] = pSamplePt->ImgP[1] - pSamplePt_->ImgP[1];
									distM = MdImgP[0] * MdImgP[0] + MdImgP[1] * MdImgP[1];
									if (distM < minDist2)
										break;
									distM = sqrt(distM);
									MdImgP[0] /= distM;
									MdImgP[1] /= distM;
									eM = MdImgP[0] * pEdge__->ImgN[0] + MdImgP[1] * pEdge__->ImgN[0];
									QdImgP[0] = (float)(QImgP[0] - QImgP_[0]);
									QdImgP[1] = (float)(QImgP[1] - QImgP_[1]);
									distQ = QdImgP[0] * QdImgP[0] + QdImgP[1] * QdImgP[1];
									if (distQ < minDist2)
										break;
									distQ = sqrt(distQ);
									QdImgP[0] /= distQ;
									QdImgP[1] /= distQ;
									eQ = QdImgP[0] * pPtAssoc_->QImgN[0] + QdImgP[1] * pPtAssoc_->QImgN[1];
									if (RVLABS(eM) <= eThr)
									{
										if (RVLABS(eQ) > eThr)
											break;
									}
									// else if (RVLABS(eQ) <= eThr)
									//	break;
									else if (eM * eQ < 0.0f)
										break;
								}
								// if (j < i)
								//	continue;
								QN = QNMap[iImg] + 3 * iQPix;
								RVLCROSSPRODUCT3(pSamplePt->PR, QN, a);
								RVLSCALE3VECTOR(a, rotStD, a);
								RVLSCALE3VECTOR(QN, tStD, at);
								RVLCOPY3VECTOR(a, b);
								RVLCOPY3VECTOR(at, bt);
								r = R + 6 * i;
								for (j = 0; j < i; j++)
								{
									q_ = Q + 6 * j;
									qt_ = q_ + 3;
									r_ = RVLDOTPRODUCT3(q_, b) + RVLDOTPRODUCT3(qt_, bt);
									RVLSCALE3VECTOR(q_, r_, V3Tmp);
									RVLDIF3VECTORS(b, V3Tmp, b);
									RVLSCALE3VECTOR(qt_, r_, V3Tmp);
									RVLDIF3VECTORS(bt, V3Tmp, bt);
									r[j] = r_;
								}
								r_ = sqrt(RVLDOTPRODUCT3(b, b) + RVLDOTPRODUCT3(bt, bt));
								if (r_ > bestr[i])
								{
									bestPtAssoc.iMSample = iSample;
									bestPtAssoc.iQPix = iQPix;
									bestPtAssoc.QImgN[0] = QImgNa[0];
									bestPtAssoc.QImgN[1] = QImgNa[1];
									RVLCOPY3VECTOR(b, bestb);
									RVLCOPY3VECTOR(bt, bestbt);
									memcpy(bestr, r, i * sizeof(float));
									bestr[i] = r_;
								}
							}
							if (bestr[i] < 0.001f)
								break;
							ptAssoc[i] = bestPtAssoc;
							q = Q + 6 * i;
							qt = q + 3;
							RVLSCALE3VECTOR2(bestb, bestr[i], q);
							RVLSCALE3VECTOR2(bestbt, bestr[i], qt);
							memcpy(r, bestr, (i + 1) * sizeof(float));
							// Only for debugging purpose!!!
							// if (iFifthAssoc == 483)
							//{
							// float a_[6];
							// float* at_ = a_ + 3;
							// RVLNULL3VECTOR(a_);
							// RVLNULL3VECTOR(at_);
							// for (j = 0; j <= i; j++)
							//{

							//	q = Q + 6 * j;
							//	RVLSCALE3VECTOR(q, r[j], V3Tmp);
							//	RVLSUM3VECTORS(a_, V3Tmp, a_);
							//	qt = q + 3;
							//	RVLSCALE3VECTOR(qt, r[j], V3Tmp);
							//	RVLSUM3VECTORS(at_, V3Tmp, at_);
							//}
							// printf("a [%d]=( ");
							// for (j = 0; j < 6; j++)
							//	printf("%f ", a[j]);
							// printf(")\n");
							// printf("a_[%d]=( ", i);
							// for (j = 0; j < 6; j++)
							//	printf("%f ", a_[j]);
							// printf(")\n");
							// pSamplePt = edgeSamplePts.Element + ptAssoc[i].iMSample;
							// QN = QNMap + 3 * ptAssoc[i].iQPix;
							// RVLCROSSPRODUCT3(pSamplePt->PR, QN, a_);
							// RVLSCALE3VECTOR(a_, rotStD, a_);
							// RVLSCALE3VECTOR(QN, tStD, at_);
							// printf("a'[%d]=( ", i);
							// for (j = 0; j < 6; j++)
							//	printf("%f ", a_[j]);
							// printf(")\n");
							//}
							//
						}
						nDefDoFs = i;
					}
					// if (i < fit3DTo2DMinDefDoFs)
					//	continue;

#ifdef RVLDDDFIT3DTO2D_VISUALIZE_PTASSOC
					// #ifdef NEVER
					// Visualize point association.
					// if (iIteration == 15)
					if (pVisualizationData->bVisualizePtAssociation)
					{
						cv::Mat visualizePtAssocImg;
						// for (int iImg = 0; iImg < numImg; iImg++)
						{
							ptAssocsVis.Element = ptAssoc + 5 * iImg;

							RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
							Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
							SampleEdges2(pMesh, edge, edgeSamplePts, iImg);

							cv::cvtColor(pVisualizationData->EDTDisplayImage[iImg], visualizePtAssocImg, cv::COLOR_GRAY2RGB);
							pVisualizationData->displayImg = &visualizePtAssocImg;
							SuperposeBinaryImage(*pVisualizationData->displayImg, edges[iImg].data, yellow);
							VisualizeEdgeSamples(*pVisualizationData->displayImg, edgeSamplePts);
							Visualize2DPointAssociation(*pVisualizationData->displayImg, ptAssocsVis, edgeSamplePts.Element);
							cv::imshow("modelPtAssoc", *pVisualizationData->displayImg);
							cv::waitKey(0);
							cv::destroyWindow("modelPtAssoc");

							delete[] edgeSamplePts.Element;
						}
					}
				}
#endif
				// Correct pose.
				RECOG::DDD::PtAssoc *firstElement;
				ptAssocs.n = nDefDoFs / 2;
				firstElement = ptAssocs.Element;

				for (int iImg = 0; iImg < numImg; iImg++)
				{
					ptAssocsVis.Element = ptAssoc + 5 * iImg;
					RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
					Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
					SampleEdges2(pMesh, edge, edgeSamplePts, iImg);

					ptAssocs.Element = &ptAssocs.Element[5 * iImg];
					for (i = 0; i < nDefDoFs / 2; i++)
					{
						pPtAssoc = &ptAssoc[5 * iImg + i];
						pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
						PR = PRMem_ + 3 * i;
						RVLCOPY3VECTOR(pSamplePt->PR, PR);
					}

					delete[] edgeSamplePts.Element;

					// Project3DModelToImage(pMesh, poseMCc, edge_, PRMem, PCMem, NCMem, iImg);
					// // Project3DModelToImage(pMesh, poseMC_, edge_, PRMem, PCMem, NCMem, iImg);
					// cv::cvtColor(edges[iImg], pVisualizationData->displayImg, cv::COLOR_GRAY2RGB);
					// Visualize3DModelImageProjection(pVisualizationData->displayImg, pMesh, edge_);
					// cv::imshow("ICPInitHyps", pVisualizationData->displayImg);
					// cv::waitKey(0);
					// cv::destroyWindow("ICPInitHyps");
				}
				ptAssocs.Element = firstElement;
				ptAssocs.n = nDefDoFs;

				// if(false)
				// if (iIteration == 2046 || iIteration == 2444 || iIteration == 2616 || iIteration == 3113 || iIteration == 3477 || iIteration == 3658 || iIteration == 3974 || iIteration == 4183 || iIteration == 4255)
				// if (iIteration == 281 || iIteration == 673 || iIteration == 1247 || iIteration == 1663 || iIteration == 2444 || iIteration == 2616 || iIteration == 3113 || iIteration == 3477 || iIteration == 3658 || iIteration == 3974 || iIteration == 4183 || iIteration == 4255)
				{
#ifdef NEVER
					for (iIteration2 = 0; iIteration2 < nInitSolutionIterations; iIteration2++)
					{
						if (iIteration2 > 0)
						{
							/// Solving 6 equations with 6 unknowns.

							// QR decomposition of A: A = R * Q

							for (i = 0; i < nDefDoFs; i++)
							{
								pPtAssoc = ptAssoc + i;
								pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
								iQPix = pPtAssoc->iQPix;
								QN = QNMap + 3 * iQPix;
								PR = PRMem_ + 3 * i;
								RVLMULMX3X3VECT(dR, pSamplePt->PR, PR);
								RVLCROSSPRODUCT3(PR, QN, a);
								RVLSCALE3VECTOR(a, rotStD, a);
								RVLSCALE3VECTOR(QN, tStD, at);
								RVLCOPY3VECTOR(a, b);
								RVLCOPY3VECTOR(at, bt);
								r = R + 6 * i;
								for (j = 0; j < i; j++)
								{
									q_ = Q + 6 * j;
									qt_ = q_ + 3;
									r_ = RVLDOTPRODUCT3(q_, b) + RVLDOTPRODUCT3(qt_, bt);
									RVLSCALE3VECTOR(q_, r_, V3Tmp);
									RVLDIF3VECTORS(b, V3Tmp, b);
									RVLSCALE3VECTOR(qt_, r_, V3Tmp);
									RVLDIF3VECTORS(bt, V3Tmp, bt);
									r[j] = r_;
								}
								r_ = sqrt(RVLDOTPRODUCT3(b, b) + RVLDOTPRODUCT3(bt, bt));
								q = Q + 6 * i;
								qt = q + 3;
								RVLSCALE3VECTOR2(b, r_, q);
								RVLSCALE3VECTOR2(bt, r_, qt);
								r[i] = r_;
							}
						}

						// Compute e and solve A * x = e.

						memset(e, 0, 6 * sizeof(float));
						for (i = 0; i < nDefDoFs; i++)
						{
							pPtAssoc = ptAssoc + i;
							iQPix = pPtAssoc->iQPix;
							Qu = (float)(iQPix % RGB.cols);
							Qv = (float)(iQPix / RGB.cols);
							QP[0] = (Qu - cameras[pPtAssoc->iImg].uc) / cameras[pPtAssoc->iImg].fu;
							QP[1] = (Qv - cameras[pPtAssoc->iImg].vc) / cameras[pPtAssoc->iImg].fv;
							PR = PRMem_ + 3 * i;
							RVLDIF3VECTORS(QP, PR, V3Tmp);
							RVLDIF3VECTORS(V3Tmp, poseMC.t, V3Tmp);
							QN = QNMap + 3 * iQPix;
							e[i] = RVLDOTPRODUCT3(QN, V3Tmp);
							y[i] = e[i];
							r = R + 6 * i;
							for (j = 0; j < i; j++)
								y[i] -= r[j] * y[j];
							y[i] /= r[i];
						}
						RVLMULMXTVECT(Q, y, nDefDoFs, 6, x, i, j, q);

						///
					}
#endif

					// ptAssocs.n = nDefDoFs;
					// pVisualizationData->bVisualizeICPSteps = false;
					// Fit3DTo2D2(pMesh, edgeSamplePts, edges, &QNMap[0], poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2(pMesh, edgeSamplePts, edges, &QNMap[0], poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2(pMesh, edgeSamplePts, edges, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2(pMesh, edgeSamplePts, edges, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2S(pMesh, edgeSamplePts, edges, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2S(pMesh, edgeSamplePts, edges, edge, poseC_C0, poseMC, ptAssocs, PRMem, PCMem, NCMem, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					Fit3DTo2D2SS(pMesh, edges, edge, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
					RVLSUM3VECTORS(poseMC.t, dt, poseMC_.t);

#ifdef RVLDDDFIT3DTO2D_VISUALIZE_INITHYPS
					// Visualize corrected pose.

					// if (iIteration == 15)
					if (pVisualizationData->bVisualizeInitialHypothesis)
					{
						cv::Mat visualizeInitialHypothesisImg;
						// if (iIteration == 15)
						// if(false)
						for (int iImg = 0; iImg < numImg; iImg++)
						{
							// RVLCOMPTRANSF3D(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC_.R, poseMC_.t, poseMCc.R, poseMCc.t);
							// RVLCOMPTRANSF3D(poseMC_.R, poseMC_.t, poseC_C0[iImg].R, poseC_C0[iImg].t, poseMCc.R, poseMCc.t);
							RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC_.R, poseMC_.t, poseMCc.R, poseMCc.t, tmp3x1);
							// RVLMXMUL3X3(poseC_C0[iImg].R, poseMC_.R, poseMCc.R);
							// RVLSUM3VECTORS(poseC_C0[iImg].t, poseMC_.t, poseMCc.t);
							Project3DModelToImage(pMesh, poseMCc, edge_, PRMem, PCMem, NCMem, iImg);
							// Project3DModelToImage(pMesh, poseMC_, edge_, PRMem, PCMem, NCMem, iImg);
							cv::cvtColor(edges[iImg], visualizeInitialHypothesisImg, cv::COLOR_GRAY2RGB);
							pVisualizationData->displayImg = &visualizeInitialHypothesisImg;
							Visualize3DModelImageProjection(*pVisualizationData->displayImg, pMesh, edge_);
							cv::imshow("modelInitHyps", *pVisualizationData->displayImg);
							cv::waitKey(0);
							cv::destroyWindow("modelInitHyps");
						}
					}
#endif
					if (th > maxOrientationCorrection || et > fit3DTo2DMaxPositionCorrection)
					{
#ifdef RVLDDDFIT3DTO2D_VERBOSE
						printf("Hypothesis rejected.\n\n");
#endif
						continue;
					}

					///

					// Evaluate hypothesis.

					float score = 0.0f;
					int nFPs = 0;
					float chamferDist2;
					for (int iImg = 0; iImg < numImg; iImg++)
					{
						RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC_.R, poseMC_.t, poseMCc.R, poseMCc.t, tmp3x1);
						Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
						SampleEdges2(pMesh, edge, edgeSamplePts, iImg);
						for (iSample = 0; iSample < edgeSamplePts.n; iSample++)
						{
							pSamplePt = edgeSamplePts.Element + iSample;
							pSamplePt_ = edgeSamplePts_.Element + iSample;
							// RVLTRANSF3(pSamplePt->PR, dR, poseMC_.t, pSamplePt_->PC);
							RVLCOPY3VECTOR(pSamplePt->PC, pSamplePt_->PC);
							pSamplePt_->iImgP[0] = (int)round(cameras[iImg].fu * pSamplePt_->PC[0] / pSamplePt_->PC[2] + cameras[iImg].uc);
							pSamplePt_->iImgP[1] = (int)round(cameras[iImg].fv * pSamplePt_->PC[1] / pSamplePt_->PC[2] + cameras[iImg].vc);
							if (pSamplePt_->iImgP[0] < 0 || pSamplePt_->iImgP[0] >= RGBs[0].cols || pSamplePt_->iImgP[1] < 0 || pSamplePt_->iImgP[1] >= RGBs[0].rows)
							{
								nFPs++;
								continue;
							}
							pSamplePt_->iPix = (int)round(pSamplePt_->iImgP[0]) + (int)round(pSamplePt_->iImgP[1]) * RGBs[0].cols;
							chamferDist2 = (float)pVisualizationData->EDTImage[iImg].pPix[pSamplePt_->iPix].d2;
							if (chamferDist2 > chamferThr2)
							{
								nFPs++;
								continue;
							}
							// pEdge_ = edge + pSamplePt->edgeIdx;
							// QImgP[0] = pSamplePt->iImgP[0] - EDTImage.pPix[pSamplePt->iPix].dx;
							// QImgP[1] = pSamplePt->iImgP[1] - EDTImage.pPix[pSamplePt->iPix].dz;
							// iQPix = QImgP[0] + QImgP[1] * RGB.cols;
							// QImgN = QImgNMap + 2 * iQPix;
							// csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
							// if (RVLABS(csN) >= csNThr)
							score += exp(-chamferDist2 / chamferThr2);
							// else
							//	nFPs++;
						}
						delete[] edgeSamplePts.Element;
					}
#ifdef RVLDDDFIT3DTO2D_VERBOSE
					printf("scene fitting score = %f no. FPs = %d\n\n", score, nFPs);
#endif
					// Record the best hypothesis.

					if (score > bestScore)
					{
						bestScore = score;
						bestPoseMC = poseMC_;
						RVLCOPYMX3X3(dR, bestPosedR);
						RVLCOPY3VECTOR(dt, bestPosedt);
						nBestNumFPs = nFPs;
						iBestHypothesis = iHypothesis;
					}

					// Visualize hypothesis.

					// cv::cvtColor(EDTDisplayImage, displayImg, cv::COLOR_GRAY2RGB);
					// SuperposeBinaryImage(displayImg, edges.data, yellow);
					// VisualizeEdgeSamples(displayImg, edgeSamplePts_);
					// cv::imshow("model", displayImg);
					// cv::waitKey(0);
				}
				iHypothesis++;
			}
		}
	}
	delete pseudornd.Element;

	/// ICP fitting.

	int nICPIterations = 5;
	float ICPThr = 10.0; // pix

	float ICPThr2 = ICPThr * ICPThr;
	RVLCOPYMX3X3(bestPosedR, dR);
	Pose3D poseMC = bestPoseMC;

	// Visualize initial pose.
	if (pVisualizationData->bVisualizeBestHypothesis)
	{
		cv::Mat visualizeBestHypothesisImg;
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
			Project3DModelToImage(pMesh, poseMCc, edge_, PRMem, PCMem, NCMem, iImg);
			cv::cvtColor(edges[iImg], visualizeBestHypothesisImg, cv::COLOR_GRAY2RGB);
			pVisualizationData->displayImg = &visualizeBestHypothesisImg;
			Visualize3DModelImageProjection(*pVisualizationData->displayImg, pMesh, edge_);
			cv::imshow("modelBeforeICP", *pVisualizationData->displayImg);
			cv::waitKey(0);
		}
	}

	//

	float chamferDist2;
	for (iIteration = 0; iIteration < nICPIterations; iIteration++)
	{
		ptAssocs.n = 0;
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
			Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
			SampleEdges2(pMesh, edge, edgeSamplePts, iImg);
			for (iSample = 0; iSample < edgeSamplePts.n; iSample++)
			{
				pSamplePt = edgeSamplePts.Element + iSample;
				pSamplePt_ = pSamplePt;
				// pSamplePt_ = edgeSamplePts_.Element + iSample;
				// RVLTRANSF3(pSamplePt->PR, dR, poseMC_.t, pSamplePt_->PC);
				// pSamplePt_->iImgP[0] = (int)round(cameras[iImg].fu * pSamplePt_->PC[0] / pSamplePt_->PC[2] + cameras[iImg].uc);
				// pSamplePt_->iImgP[1] = (int)round(cameras[iImg].fv * pSamplePt_->PC[1] / pSamplePt_->PC[2] + cameras[iImg].vc);
				// if (pSamplePt_->iImgP[0] < 0 || pSamplePt_->iImgP[0] >= RGB.cols || pSamplePt_->iImgP[1] < 0 || pSamplePt_->iImgP[1] >= RGB.rows)
				//	continue;
				// pSamplePt_->iPix = (int)round(pSamplePt_->iImgP[0]) + (int)round(pSamplePt_->iImgP[1]) * cameras[iImg].w;
				chamferDist2 = (float)pVisualizationData->EDTImage[iImg].pPix[pSamplePt_->iPix].d2;
				if (chamferDist2 > ICPThr2)
					continue;
				QImgP[0] = pSamplePt_->iImgP[0] - pVisualizationData->EDTImage[iImg].pPix[pSamplePt_->iPix].dx;
				QImgP[1] = pSamplePt_->iImgP[1] - pVisualizationData->EDTImage[iImg].pPix[pSamplePt_->iPix].dz;
				iQPix = QImgP[0] + QImgP[1] * cameras[iImg].w;
				if (edges[iImg].data[iQPix] == 0)
					continue;
				QImgN = QImgNMap + 2 * iQPix;
				pEdge_ = edge + pSamplePt->edgeIdx;
				csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
				if (RVLABS(csN) < csNThr)
					continue;
				pPtAssoc = ptAssoc + ptAssocs.n++;
				pPtAssoc->iMSample = iSample;
				pPtAssoc->iQPix = iQPix;
				pPtAssoc->iImg = iImg;
				pPtAssoc->QImgN[0] = QImgN[0];
				pPtAssoc->QImgN[1] = QImgN[1];
				QN = QNMap[iImg] + 3 * iQPix;
				RVLCOPY3VECTOR(QN, pPtAssoc->QN);
			}
			delete[] edgeSamplePts.Element;
		}
		// Fit3DTo2D2(pMesh, edgeSamplePts, edges, &QNMap[0], poseC_C0, poseMC, ptAssocs, 20, th_, th, et, dR, dt, 0.1f);
		// Fit3DTo2D2(pMesh, edgeSamplePts, edges, &QNMap[0], poseC_C0, poseMC, ptAssocs, 20, th_, th, et, dR, dt, 0.1f);
		// Fit3DTo2D2(pMesh, edgeSamplePts, edges, poseC_C0, poseMC, ptAssocs, 20, th_, th, et, dR, dt, 0.1f);
		// Fit3DTo2D2(pMesh, edgeSamplePts, edges, edge, poseC_C0, poseMC, ptAssocs, PRMem, PCMem, NCMem, 20, th_, th, et, dR, dt, 0.1f);
		Fit3DTo2D2SS(pMesh, edges, edge, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.1f);
		RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
		RVLSUM3VECTORS(poseMC.t, dt, poseMC_.t);
		poseMC = poseMC_;

		if (bVisualizeICPSteps)
		{
			// Visualize point association.

			// cv::cvtColor(pVisualizationData->EDTDisplayImage[iImg], pVisualizationData->displayImg, cv::COLOR_GRAY2RGB);
			// SuperposeBinaryImage(pVisualizationData->displayImg, edges[iImg].data, yellow);
			// //VisualizeEdgeSamples(displayImg, edgeSamplePts);
			// Visualize2DPointAssociation(pVisualizationData->displayImg, ptAssocs, edgeSamplePts.Element);
			// cv::imshow("model", pVisualizationData->displayImg);
			// cv::waitKey(0);

			//

			cv::Mat visualizeICPStepsImg;

			for (int iImg = 0; iImg < numImg; iImg++)
			{
				RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
				Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
				cv::cvtColor(edges[iImg], visualizeICPStepsImg, cv::COLOR_GRAY2RGB);
				pVisualizationData->displayImg = &visualizeICPStepsImg;
				Visualize3DModelImageProjection(*pVisualizationData->displayImg, pMesh, edge);
				std::string displayImgName = "ICP iteration " + std::to_string(iIteration) + " image " + std::to_string(iImg);
				cv::imshow(displayImgName, *pVisualizationData->displayImg);
				cv::waitKey(0);
				cv::destroyWindow(displayImgName);
			}
		}
	}
	pVisualizationData->bVisualizeICPSteps = bVisualizeICPSteps;

	///

	// Visualize final solution.

	uchar red[3] = {255, 0, 0};
	printf("best scene fitting score = %f no. FPs = %d iteration %d\n\n", bestScore, nBestNumFPs, iBestHypothesis);

	Pose3D tempPose;
	cv::Mat visualizeFinalSolutionImg;
	;
	for (int iImg = 0; iImg < numImg; iImg++)
	{
		visualizeFinalSolutionImg = RGBs[iImg].clone();
		// RVLTRANSF3(poseMC, poseC_C0[iImg].R, poseC_C0[iImg].t, tempPose);
		RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);

		pVisualizationData->displayImg = &visualizeFinalSolutionImg;
		Project3DModelToImage(pMesh, poseMCc, edge_, PRMem, PCMem, NCMem, iImg);
		// cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
		Visualize3DModelImageProjection(*pVisualizationData->displayImg, pMesh, edge_, red);

		solutions[iImg] = *pVisualizationData->displayImg;
		std::string filename = "/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP_measurements-230201/results/" + std::to_string(counter) + "-" + std::to_string(iImg) + ".png";
		// cout << filename << endl;
		cv::imwrite(filename, *pVisualizationData->displayImg);
		// cv::imshow("solution" + std::to_string(iImg), *pVisualizationData->displayImg);
		// cv::waitKey(0);
	}

	delete[] PRMem;
	delete[] PCMem;
	delete[] NCMem;
	delete[] edge;
	delete[] edge_;
	delete[] QImgNMap;
	// delete[] edgeSamplePts.Element;
	delete[] edgeSamplePts_.Element;
	delete[] ptAssoc;

	for (int iSize = 0; iSize < numImg; iSize++)
	{
		delete[] IuMap[iSize];
		delete[] IvMap[iSize];
		delete[] QNMap[iSize];
		delete[] pVisualizationData->EDTImage[iSize].pPix;
	}
	delete[] IuMap;
	delete[] IvMap;
	delete[] QNMap;

	cv::destroyAllWindows();
}

RVL::Pose3D DDDetector::Fit3DTo2DStereo2(
	Mesh *pMesh,
	cv::Mat *RGBs,
	int numImg,
	Array<Pose3D> initPosesMC,
	Pose3D *poseC_C0,
	Pose3D &bestPoseMC,
	cv::Mat *&solutions,
	int counter)
{
	// Create edges and compute their lengths.

	RECOG::DDD::Edge *edge = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
	RECOG::DDD::Edge *pEdge_;
	MeshEdge *pEdge;
	int iEdge;
	Point *pPt, *pPt_;
	float V3Tmp[3];
	for (iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		pEdge = pMesh->EdgeArray.Element + iEdge;
		pPt = pMesh->NodeArray.Element + pEdge->iVertex[0];
		pPt_ = pMesh->NodeArray.Element + pEdge->iVertex[1];
		pEdge_ = edge + iEdge;
		RVLDIF3VECTORS(pPt_->P, pPt->P, V3Tmp);
		pEdge_->length = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
		pEdge_->iVertex[0] = pEdge->iVertex[0];
		pEdge_->iVertex[1] = pEdge->iVertex[1];
	}

	// Visualization stuff
	uchar yellow[] = {0, 255, 255};

	cv::Mat edges[2];
	double **IuMap = new double *[numImg];
	double **IvMap = new double *[numImg];
	float **QNMap; // = new float*[numImg];

	float *QN;
	int nPix = RGBs[0].cols * RGBs[0].rows;
	float *QImgNMap = new float[2 * nPix];
	float *QImgN = QImgNMap;

	for (int iImg = 0; iImg < numImg; iImg++)
	{
		IuMap[iImg] = new double[nPix];
		IvMap[iImg] = new double[nPix];

		// QNMap[iImg] = new float[3*nPix];
	}

	setupFit3DTo2D(numImg, RGBs, edges, QImgNMap, IuMap, IvMap, QNMap);

	// Project model onto the image. -- allocation
	float *PRMem = new float[pMesh->NodeArray.n * 3];
	float *PCMem = new float[pMesh->NodeArray.n * 3];
	float *NCMem = new float[pMesh->faces.n * 3];
	float PRMem_[6 * 3];

	// // Initialize visualization.
	// // Visualize initial pose.
	// cv::Mat displayImg1, displayImg2;
	// // cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
	// // cv::cvtColor(EDTDisplayImage, displayImg, cv::COLOR_GRAY2RGB);
	// displayImg1 = RGBs[0].clone();
	// displayImg2 = RGBs[1].clone();

	// for (int iImg = 0; iImg < numImg; iImg++)
	// {
	// 	for (int iPose = 0; iPose < 4; iPose++)
	// 	{
	// 		pVisualizationData->displayImg = RGBs[iImg].clone();
	// 		SuperposeBinaryImage(pVisualizationData->displayImg, edges[iImg].data, yellow);
	// 		Visualize3DModelImageProjection(pVisualizationData->displayImg, pMesh, edge);
	// 		cv::imshow("model" + std::to_string(iPose), pVisualizationData->displayImg);
	// 		cv::waitKey(0);
	// 	}
	// }

	// pVisualizationData->displayImg = RGBs[1].clone();
	// SuperposeBinaryImage(pVisualizationData->displayImg, edges[1].data, yellow);
	// Visualize3DModelImageProjection(pVisualizationData->displayImg, pMesh, edge);
	// cv::imshow("model2", pVisualizationData->displayImg);

	// Hypothesis generation loop.

	pseudornd.n = 100000;
	pseudornd.Element = new int[pseudornd.n];
	pSurfelDetector->RandomIndices(pseudornd);
	iRnd = 0;

	Array<RECOG::DDD::EdgeSample> edgeSamplePts;
	int nEdgeSamples = fit3DTo2DNoEdgeSamples;
	int iSample;
	RECOG::DDD::EdgeSample *pSamplePt;
	bool bHoughLines = true;
	RECOG::DDD::Edge *edge_ = new RECOG::DDD::Edge[pMesh->EdgeArray.n];
	Array<RECOG::DDD::EdgeSample> edgeSamplePts_;
	edgeSamplePts_.Element = new RECOG::DDD::EdgeSample[fit3DTo2DNoEdgeSamples];
	edgeSamplePts_.n = nEdgeSamples;
	float maxOrientationCorrection = fit3DTo2DMaxOrientationCorrection * DEG2RAD;
	int nInitSolutionIterations = 3;
	int nTrials = 20;
	float minDist = 20.0f;
	float eThr = 0.2f;
	float eN = 30.0f;
	float rotStD = 0.125 * PI; // rad
	float tStD = 0.05;		   // m
	float chamferThr = 3.0f;   // pix
	int i;
	float csNThr = cos(eN * DEG2RAD);
	float minDist2 = minDist * minDist;
	float chamferThr2 = chamferThr * chamferThr;
	int iIteration, iIteration2, j, iSample_, iQPix, iQPix_, iTrial;
	RECOG::DDD::PtAssoc *ptAssoc = new RECOG::DDD::PtAssoc[2 * fit3DTo2DNoEdgeSamples];
	RECOG::DDD::PtAssoc bestPtAssoc;
	RECOG::DDD::PtAssoc *pPtAssoc, *pPtAssoc_;
	RECOG::DDD::EdgeSample *pSamplePt_;
	RECOG::DDD::Edge *pEdge__;
	float *QImgN_;
	float distM, eM, distQ, eQ, csN;
	float MdImgP[2];
	float QImgNa[2];
	int QImgP[2], QImgP_[2];
	float QdImgP[2];
	float Q[36], R[36];
	memset(R, 0, 36 * sizeof(float));
	float *q, *qt, *q_, *qt_, *r, *PR;
	float x[6], y[6];
	float *xt = x + 3;
	float r_;
	float a[6], b[6];
	float *at = a + 3;
	float *bt = b + 3;
	float bestb[6], bestr[6];
	float *bestbt = bestb + 3;
	float e[6];
	float dR[9], dR0[9], dR_[9], M3x3Tmp[9];
	float th, th_, et;
	float rotAxis[3], dt[3], dt0[3];
	Pose3D poseMC_;
	bestPoseMC = initPosesMC.Element[0];
	float bestPosedR[9];
	float bestPosedt[3];
	float bestScore = 0.0f;
	int iBestHypothesis = -1;
	int nBestNumFPs = fit3DTo2DNoEdgeSamples;
	float signN;
	int nDefDoFs;
	cv::Mat cva(6, 1, CV_64FC1);
	double *a_ = (double *)(cva.data);
	double *at_ = a_ + 3;
	cv::Mat cvM(6, 6, CV_64FC1);
	double *M_ = (double *)(cvM.data);
	cv::Mat cve(6, 1, CV_64FC1);
	double *e_ = (double *)(cve.data);
	cv::Mat cvx(6, 1, CV_64FC1);
	double *x_ = (double *)(cvx.data);
	cv::Mat cvA(6, 6, CV_64FC1);
	double *A_ = (double *)(cvA.data);
	cv::Mat cvc(6, 1, CV_64FC1);
	double *c_ = (double *)(cvc.data);
	Array<RECOG::DDD::PtAssoc> ptAssocs;
	ptAssocs.Element = ptAssoc;
	std::vector<int> fifthAssocPtsHyps[2];
	std::vector<int> fifthMIdxs[2];
	// int fifthMIdxs[2][2];
	bool bVisualizeICPSteps = pVisualizationData->bVisualizeICPSteps;
	// pVisualizationData->bVisualizeICPSteps = false;
	int iHypothesis = 0;

	Pose3D poseMCc;
	float tmp3x1[3];
	RVL::Array<RVL::RECOG::DDD::PtAssoc> ptAssocsVis;
	ptAssocsVis.Element = ptAssoc;
	ptAssocsVis.n = 5;

	for (int iImg = 0; iImg < numImg; iImg++)
		for (int iAssoc = 0; iAssoc < 5; iAssoc++)
			ptAssocs.Element[5 * iImg + iAssoc].iImg = iImg;

	for (int iInitPose = 0; iInitPose < initPosesMC.n; iInitPose++)
	{
		Pose3D poseMC = initPosesMC.Element[iInitPose];
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			// pVisualizationData->displayImg = &RGBs[iImg].clone();

			RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
			Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
			SampleEdges2(pMesh, edge, edgeSamplePts, iImg);

			if (bHoughLines)
			{
				// cv::Mat edgesT;
				// cv::cvtColor(edges[iImg], edgesT, cv::COLOR_GRAY2RGB);

				// edgesT = edges[iImg].clone();
				// VisualizeEdgeSamples(edgesT, edgeSamplePts);
				// cv::imshow("Img model", edgesT);
				// cv::waitKey(0);

				// pVisualizationData->displayImg = &edges[iImg].clone();

				nDefDoFs = 10;
				RVL::DDDetector::HoughLinesPtAssociationLHTCP(
					edges[iImg],
					QNMap[iImg],
					IuMap[iImg],
					IvMap[iImg],
					pMesh,
					edge,
					cameras[iImg],
					pVisualizationData->bVisualizeHoughTransform,
					edgeSamplePts,
					fifthAssocPtsHyps[iImg],
					&ptAssoc[5 * iImg],
					fifthMIdxs[iImg]);
			}

			delete[] edgeSamplePts.Element;
		}

		int nHypGenIterations[2];
		nHypGenIterations[0] = 2 * fifthAssocPtsHyps[0].size();
		nHypGenIterations[1] = 2 * fifthAssocPtsHyps[1].size();
		int iFifthAssoc[2];
		for (iFifthAssoc[0] = 0; iFifthAssoc[0] < nHypGenIterations[0]; iFifthAssoc[0]++)
		{
			for (iFifthAssoc[1] = 0; iFifthAssoc[1] < nHypGenIterations[1]; iFifthAssoc[1]++)
			{
#ifdef RVLDDDFIT3DTO2D_VERBOSE
				printf("pose %d\/%d 5th_assoc. (%d\/%d, %d\/%d)\n", iInitPose, initPosesMC.n, iFifthAssoc[0], nHypGenIterations[0], iFifthAssoc[1], nHypGenIterations[1]);
#endif
				for (int iImg = 0; iImg < numImg; iImg++)
				{
					// if (iFifthAssoc == 15)
					//	int debug = 0;
					if (bHoughLines)
					{
						int idx = iFifthAssoc[iImg] / 2;
						ptAssoc[5 * (iImg + 1) - 1].iQPix = fifthAssocPtsHyps[iImg][idx];
						ptAssoc[5 * (iImg + 1) - 1].iMSample = fifthMIdxs[iImg][iFifthAssoc[iImg] % 2];
						// Set QN normals
						int iPtAssoc, iQN, idxAssoc;
						for (iPtAssoc = 0; iPtAssoc < nDefDoFs / 2; iPtAssoc++)
						{
							for (iQN = 0; iQN < 3; iQN++)
							{
								idxAssoc = 5 * iImg + iPtAssoc;
								ptAssoc[idxAssoc].QN[iQN] = (QNMap[iImg] + 3 * ptAssoc[idxAssoc].iQPix)[iQN];
							}
						}
					}
					else
					{
						for (i = 0; i < 6; i++)
						{
							// if (i == 5)
							//	int debug = 0;
							memset(bestr, 0, 6 * sizeof(float));
							for (iTrial = 0; iTrial < nTrials; iTrial++)
							{
								// if (i == 2 && iTrial == 3)
								//	int debug = 0;
								iSample = rand() % edgeSamplePts.n;
								pSamplePt = edgeSamplePts.Element + iSample;
								pEdge_ = edge + pSamplePt->edgeIdx;
								if (pVisualizationData->EDTImage[iImg].pPix[pSamplePt->iPix].d2 > pVisualizationData->EDT[iImg].m_maxd2)
									continue;
								QImgP[0] = pSamplePt->iImgP[0] - pVisualizationData->EDTImage[iImg].pPix[pSamplePt->iPix].dx;
								QImgP[1] = pSamplePt->iImgP[1] - pVisualizationData->EDTImage[iImg].pPix[pSamplePt->iPix].dz;
								iQPix = QImgP[0] + QImgP[1] * RGBs[iImg].cols;
								QImgN = QImgNMap + 2 * iQPix;
								csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
								if (csN >= csNThr)
									signN = 1.0f;
								else if (-csN >= csNThr)
									signN = -1.0f;
								else
									continue;
								QImgNa[0] = signN * QImgN[0];
								QImgNa[1] = signN * QImgN[1];
								for (j = 0; j < i; j++)
								{
									pPtAssoc_ = ptAssoc + j;
									iSample_ = pPtAssoc_->iMSample;
									pSamplePt_ = edgeSamplePts.Element + iSample_;
									pEdge__ = edge + pSamplePt_->edgeIdx;
									iQPix_ = pPtAssoc_->iQPix;
									QImgP_[0] = iQPix_ % RGBs[iImg].cols;
									QImgP_[1] = iQPix_ / RGBs[iImg].cols;
									QImgN_ = QImgNMap + 2 * iQPix_;
									MdImgP[0] = pSamplePt->ImgP[0] - pSamplePt_->ImgP[0];
									MdImgP[1] = pSamplePt->ImgP[1] - pSamplePt_->ImgP[1];
									distM = MdImgP[0] * MdImgP[0] + MdImgP[1] * MdImgP[1];
									if (distM < minDist2)
										break;
									distM = sqrt(distM);
									MdImgP[0] /= distM;
									MdImgP[1] /= distM;
									eM = MdImgP[0] * pEdge__->ImgN[0] + MdImgP[1] * pEdge__->ImgN[0];
									QdImgP[0] = (float)(QImgP[0] - QImgP_[0]);
									QdImgP[1] = (float)(QImgP[1] - QImgP_[1]);
									distQ = QdImgP[0] * QdImgP[0] + QdImgP[1] * QdImgP[1];
									if (distQ < minDist2)
										break;
									distQ = sqrt(distQ);
									QdImgP[0] /= distQ;
									QdImgP[1] /= distQ;
									eQ = QdImgP[0] * pPtAssoc_->QImgN[0] + QdImgP[1] * pPtAssoc_->QImgN[1];
									if (RVLABS(eM) <= eThr)
									{
										if (RVLABS(eQ) > eThr)
											break;
									}
									// else if (RVLABS(eQ) <= eThr)
									//	break;
									else if (eM * eQ < 0.0f)
										break;
								}
								// if (j < i)
								//	continue;
								QN = QNMap[iImg] + 3 * iQPix;
								RVLCROSSPRODUCT3(pSamplePt->PR, QN, a);
								RVLSCALE3VECTOR(a, rotStD, a);
								RVLSCALE3VECTOR(QN, tStD, at);
								RVLCOPY3VECTOR(a, b);
								RVLCOPY3VECTOR(at, bt);
								r = R + 6 * i;
								for (j = 0; j < i; j++)
								{
									q_ = Q + 6 * j;
									qt_ = q_ + 3;
									r_ = RVLDOTPRODUCT3(q_, b) + RVLDOTPRODUCT3(qt_, bt);
									RVLSCALE3VECTOR(q_, r_, V3Tmp);
									RVLDIF3VECTORS(b, V3Tmp, b);
									RVLSCALE3VECTOR(qt_, r_, V3Tmp);
									RVLDIF3VECTORS(bt, V3Tmp, bt);
									r[j] = r_;
								}
								r_ = sqrt(RVLDOTPRODUCT3(b, b) + RVLDOTPRODUCT3(bt, bt));
								if (r_ > bestr[i])
								{
									bestPtAssoc.iMSample = iSample;
									bestPtAssoc.iQPix = iQPix;
									bestPtAssoc.QImgN[0] = QImgNa[0];
									bestPtAssoc.QImgN[1] = QImgNa[1];
									RVLCOPY3VECTOR(b, bestb);
									RVLCOPY3VECTOR(bt, bestbt);
									memcpy(bestr, r, i * sizeof(float));
									bestr[i] = r_;
								}
							}
							if (bestr[i] < 0.001f)
								break;
							ptAssoc[i] = bestPtAssoc;
							q = Q + 6 * i;
							qt = q + 3;
							RVLSCALE3VECTOR2(bestb, bestr[i], q);
							RVLSCALE3VECTOR2(bestbt, bestr[i], qt);
							memcpy(r, bestr, (i + 1) * sizeof(float));
							// Only for debugging purpose!!!
							// if (iFifthAssoc == 483)
							//{
							// float a_[6];
							// float* at_ = a_ + 3;
							// RVLNULL3VECTOR(a_);
							// RVLNULL3VECTOR(at_);
							// for (j = 0; j <= i; j++)
							//{

							//	q = Q + 6 * j;
							//	RVLSCALE3VECTOR(q, r[j], V3Tmp);
							//	RVLSUM3VECTORS(a_, V3Tmp, a_);
							//	qt = q + 3;
							//	RVLSCALE3VECTOR(qt, r[j], V3Tmp);
							//	RVLSUM3VECTORS(at_, V3Tmp, at_);
							//}
							// printf("a [%d]=( ");
							// for (j = 0; j < 6; j++)
							//	printf("%f ", a[j]);
							// printf(")\n");
							// printf("a_[%d]=( ", i);
							// for (j = 0; j < 6; j++)
							//	printf("%f ", a_[j]);
							// printf(")\n");
							// pSamplePt = edgeSamplePts.Element + ptAssoc[i].iMSample;
							// QN = QNMap + 3 * ptAssoc[i].iQPix;
							// RVLCROSSPRODUCT3(pSamplePt->PR, QN, a_);
							// RVLSCALE3VECTOR(a_, rotStD, a_);
							// RVLSCALE3VECTOR(QN, tStD, at_);
							// printf("a'[%d]=( ", i);
							// for (j = 0; j < 6; j++)
							//	printf("%f ", a_[j]);
							// printf(")\n");
							//}
							//
						}
						nDefDoFs = i;
					}
					// if (i < fit3DTo2DMinDefDoFs)
					//	continue;

#ifdef RVLDDDFIT3DTO2D_VISUALIZE_PTASSOC
					// #ifdef NEVER
					// Visualize point association.
					// if (iIteration == 15)
					if (pVisualizationData->bVisualizePtAssociation)
					{
						cv::Mat visualizePtAssocImg;
						// for (int iImg = 0; iImg < numImg; iImg++)
						{
							ptAssocsVis.Element = ptAssoc + 5 * iImg;

							RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
							Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
							SampleEdges2(pMesh, edge, edgeSamplePts, iImg);

							cv::cvtColor(pVisualizationData->EDTDisplayImage[iImg], visualizePtAssocImg, cv::COLOR_GRAY2RGB);
							pVisualizationData->displayImg = &visualizePtAssocImg;
							SuperposeBinaryImage(*pVisualizationData->displayImg, edges[iImg].data, yellow);
							VisualizeEdgeSamples(*pVisualizationData->displayImg, edgeSamplePts);
							Visualize2DPointAssociation(*pVisualizationData->displayImg, ptAssocsVis, edgeSamplePts.Element);
							cv::imshow("modelPtAssoc", *pVisualizationData->displayImg);
							cv::waitKey(0);
							cv::destroyWindow("modelPtAssoc");

							delete[] edgeSamplePts.Element;
						}
					}
				}
#endif
				// Correct pose.
				RECOG::DDD::PtAssoc *firstElement;
				ptAssocs.n = nDefDoFs / 2;
				firstElement = ptAssocs.Element;

				for (int iImg = 0; iImg < numImg; iImg++)
				{
					ptAssocsVis.Element = ptAssoc + 5 * iImg;
					RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
					Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
					SampleEdges2(pMesh, edge, edgeSamplePts, iImg);

					ptAssocs.Element = &ptAssocs.Element[5 * iImg];
					for (i = 0; i < nDefDoFs / 2; i++)
					{
						pPtAssoc = &ptAssoc[5 * iImg + i];
						pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
						PR = PRMem_ + 3 * i;
						RVLCOPY3VECTOR(pSamplePt->PR, PR);
					}

					delete[] edgeSamplePts.Element;

					// Project3DModelToImage(pMesh, poseMCc, edge_, PRMem, PCMem, NCMem, iImg);
					// // Project3DModelToImage(pMesh, poseMC_, edge_, PRMem, PCMem, NCMem, iImg);
					// cv::cvtColor(edges[iImg], pVisualizationData->displayImg, cv::COLOR_GRAY2RGB);
					// Visualize3DModelImageProjection(pVisualizationData->displayImg, pMesh, edge_);
					// cv::imshow("ICPInitHyps", pVisualizationData->displayImg);
					// cv::waitKey(0);
					// cv::destroyWindow("ICPInitHyps");
				}
				ptAssocs.Element = firstElement;
				ptAssocs.n = nDefDoFs;

				// if(false)
				// if (iIteration == 2046 || iIteration == 2444 || iIteration == 2616 || iIteration == 3113 || iIteration == 3477 || iIteration == 3658 || iIteration == 3974 || iIteration == 4183 || iIteration == 4255)
				// if (iIteration == 281 || iIteration == 673 || iIteration == 1247 || iIteration == 1663 || iIteration == 2444 || iIteration == 2616 || iIteration == 3113 || iIteration == 3477 || iIteration == 3658 || iIteration == 3974 || iIteration == 4183 || iIteration == 4255)
				{
#ifdef NEVER
					for (iIteration2 = 0; iIteration2 < nInitSolutionIterations; iIteration2++)
					{
						if (iIteration2 > 0)
						{
							/// Solving 6 equations with 6 unknowns.

							// QR decomposition of A: A = R * Q

							for (i = 0; i < nDefDoFs; i++)
							{
								pPtAssoc = ptAssoc + i;
								pSamplePt = edgeSamplePts.Element + pPtAssoc->iMSample;
								iQPix = pPtAssoc->iQPix;
								QN = QNMap + 3 * iQPix;
								PR = PRMem_ + 3 * i;
								RVLMULMX3X3VECT(dR, pSamplePt->PR, PR);
								RVLCROSSPRODUCT3(PR, QN, a);
								RVLSCALE3VECTOR(a, rotStD, a);
								RVLSCALE3VECTOR(QN, tStD, at);
								RVLCOPY3VECTOR(a, b);
								RVLCOPY3VECTOR(at, bt);
								r = R + 6 * i;
								for (j = 0; j < i; j++)
								{
									q_ = Q + 6 * j;
									qt_ = q_ + 3;
									r_ = RVLDOTPRODUCT3(q_, b) + RVLDOTPRODUCT3(qt_, bt);
									RVLSCALE3VECTOR(q_, r_, V3Tmp);
									RVLDIF3VECTORS(b, V3Tmp, b);
									RVLSCALE3VECTOR(qt_, r_, V3Tmp);
									RVLDIF3VECTORS(bt, V3Tmp, bt);
									r[j] = r_;
								}
								r_ = sqrt(RVLDOTPRODUCT3(b, b) + RVLDOTPRODUCT3(bt, bt));
								q = Q + 6 * i;
								qt = q + 3;
								RVLSCALE3VECTOR2(b, r_, q);
								RVLSCALE3VECTOR2(bt, r_, qt);
								r[i] = r_;
							}
						}

						// Compute e and solve A * x = e.

						memset(e, 0, 6 * sizeof(float));
						for (i = 0; i < nDefDoFs; i++)
						{
							pPtAssoc = ptAssoc + i;
							iQPix = pPtAssoc->iQPix;
							Qu = (float)(iQPix % RGB.cols);
							Qv = (float)(iQPix / RGB.cols);
							QP[0] = (Qu - cameras[pPtAssoc->iImg].uc) / cameras[pPtAssoc->iImg].fu;
							QP[1] = (Qv - cameras[pPtAssoc->iImg].vc) / cameras[pPtAssoc->iImg].fv;
							PR = PRMem_ + 3 * i;
							RVLDIF3VECTORS(QP, PR, V3Tmp);
							RVLDIF3VECTORS(V3Tmp, poseMC.t, V3Tmp);
							QN = QNMap + 3 * iQPix;
							e[i] = RVLDOTPRODUCT3(QN, V3Tmp);
							y[i] = e[i];
							r = R + 6 * i;
							for (j = 0; j < i; j++)
								y[i] -= r[j] * y[j];
							y[i] /= r[i];
						}
						RVLMULMXTVECT(Q, y, nDefDoFs, 6, x, i, j, q);

						///
					}
#endif

					// ptAssocs.n = nDefDoFs;
					// pVisualizationData->bVisualizeICPSteps = false;
					// Fit3DTo2D2(pMesh, edgeSamplePts, edges, &QNMap[0], poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2(pMesh, edgeSamplePts, edges, &QNMap[0], poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2(pMesh, edgeSamplePts, edges, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2(pMesh, edgeSamplePts, edges, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2S(pMesh, edgeSamplePts, edges, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					// Fit3DTo2D2S(pMesh, edgeSamplePts, edges, edge, poseC_C0, poseMC, ptAssocs, PRMem, PCMem, NCMem, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					Fit3DTo2D2SS(pMesh, edges, edge, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.4f);
					RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
					RVLSUM3VECTORS(poseMC.t, dt, poseMC_.t);

#ifdef RVLDDDFIT3DTO2D_VISUALIZE_INITHYPS
					// Visualize corrected pose.

					// if (iIteration == 15)
					if (pVisualizationData->bVisualizeInitialHypothesis)
					{
						cv::Mat visualizeInitialHypothesisImg;
						// if (iIteration == 15)
						// if(false)
						for (int iImg = 0; iImg < numImg; iImg++)
						{
							// RVLCOMPTRANSF3D(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC_.R, poseMC_.t, poseMCc.R, poseMCc.t);
							// RVLCOMPTRANSF3D(poseMC_.R, poseMC_.t, poseC_C0[iImg].R, poseC_C0[iImg].t, poseMCc.R, poseMCc.t);
							RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC_.R, poseMC_.t, poseMCc.R, poseMCc.t, tmp3x1);
							// RVLMXMUL3X3(poseC_C0[iImg].R, poseMC_.R, poseMCc.R);
							// RVLSUM3VECTORS(poseC_C0[iImg].t, poseMC_.t, poseMCc.t);
							Project3DModelToImage(pMesh, poseMCc, edge_, PRMem, PCMem, NCMem, iImg);
							// Project3DModelToImage(pMesh, poseMC_, edge_, PRMem, PCMem, NCMem, iImg);
							cv::cvtColor(edges[iImg], visualizeInitialHypothesisImg, cv::COLOR_GRAY2RGB);
							pVisualizationData->displayImg = &visualizeInitialHypothesisImg;
							Visualize3DModelImageProjection(*pVisualizationData->displayImg, pMesh, edge_);
							cv::imshow("modelInitHyps", *pVisualizationData->displayImg);
							cv::waitKey(0);
							cv::destroyWindow("modelInitHyps");
						}
					}
#endif
					if (th > maxOrientationCorrection || et > fit3DTo2DMaxPositionCorrection)
					{
#ifdef RVLDDDFIT3DTO2D_VERBOSE
						printf("Hypothesis rejected.\n\n");
#endif
						continue;
					}

					///

					// Evaluate hypothesis.

					float score = 0.0f;
					int nFPs = 0;
					float chamferDist2;
					for (int iImg = 0; iImg < numImg; iImg++)
					{
						RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC_.R, poseMC_.t, poseMCc.R, poseMCc.t, tmp3x1);
						Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
						SampleEdges2(pMesh, edge, edgeSamplePts, iImg);
						for (iSample = 0; iSample < edgeSamplePts.n; iSample++)
						{
							pSamplePt = edgeSamplePts.Element + iSample;
							pSamplePt_ = edgeSamplePts_.Element + iSample;
							// RVLTRANSF3(pSamplePt->PR, dR, poseMC_.t, pSamplePt_->PC);
							RVLCOPY3VECTOR(pSamplePt->PC, pSamplePt_->PC);
							pSamplePt_->iImgP[0] = (int)round(cameras[iImg].fu * pSamplePt_->PC[0] / pSamplePt_->PC[2] + cameras[iImg].uc);
							pSamplePt_->iImgP[1] = (int)round(cameras[iImg].fv * pSamplePt_->PC[1] / pSamplePt_->PC[2] + cameras[iImg].vc);
							if (pSamplePt_->iImgP[0] < 0 || pSamplePt_->iImgP[0] >= RGBs[0].cols || pSamplePt_->iImgP[1] < 0 || pSamplePt_->iImgP[1] >= RGBs[0].rows)
							{
								nFPs++;
								continue;
							}
							pSamplePt_->iPix = (int)round(pSamplePt_->iImgP[0]) + (int)round(pSamplePt_->iImgP[1]) * RGBs[0].cols;
							chamferDist2 = (float)pVisualizationData->EDTImage[iImg].pPix[pSamplePt_->iPix].d2;
							if (chamferDist2 > chamferThr2)
							{
								nFPs++;
								continue;
							}
							// pEdge_ = edge + pSamplePt->edgeIdx;
							// QImgP[0] = pSamplePt->iImgP[0] - EDTImage.pPix[pSamplePt->iPix].dx;
							// QImgP[1] = pSamplePt->iImgP[1] - EDTImage.pPix[pSamplePt->iPix].dz;
							// iQPix = QImgP[0] + QImgP[1] * RGB.cols;
							// QImgN = QImgNMap + 2 * iQPix;
							// csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
							// if (RVLABS(csN) >= csNThr)
							score += exp(-chamferDist2 / chamferThr2);
							// else
							//	nFPs++;
						}
						delete[] edgeSamplePts.Element;
					}
#ifdef RVLDDDFIT3DTO2D_VERBOSE
					printf("scene fitting score = %f no. FPs = %d\n\n", score, nFPs);
#endif
					// Record the best hypothesis.

					if (score > bestScore)
					{
						bestScore = score;
						bestPoseMC = poseMC_;
						RVLCOPYMX3X3(dR, bestPosedR);
						RVLCOPY3VECTOR(dt, bestPosedt);
						nBestNumFPs = nFPs;
						iBestHypothesis = iHypothesis;
					}

					// Visualize hypothesis.

					// cv::cvtColor(EDTDisplayImage, displayImg, cv::COLOR_GRAY2RGB);
					// SuperposeBinaryImage(displayImg, edges.data, yellow);
					// VisualizeEdgeSamples(displayImg, edgeSamplePts_);
					// cv::imshow("model", displayImg);
					// cv::waitKey(0);
				}
				iHypothesis++;
			}
		}
	}
	delete pseudornd.Element;

	/// ICP fitting.

	int nICPIterations = 5;
	float ICPThr = 10.0; // pix

	float ICPThr2 = ICPThr * ICPThr;
	RVLCOPYMX3X3(bestPosedR, dR);
	Pose3D poseMC = bestPoseMC;

	// Visualize initial pose.
	if (pVisualizationData->bVisualizeBestHypothesis)
	{
		cv::Mat visualizeBestHypothesisImg;
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
			Project3DModelToImage(pMesh, poseMCc, edge_, PRMem, PCMem, NCMem, iImg);
			cv::cvtColor(edges[iImg], visualizeBestHypothesisImg, cv::COLOR_GRAY2RGB);
			pVisualizationData->displayImg = &visualizeBestHypothesisImg;
			Visualize3DModelImageProjection(*pVisualizationData->displayImg, pMesh, edge_);
			cv::imshow("modelBeforeICP", *pVisualizationData->displayImg);
			cv::waitKey(0);
		}
	}

	//

	float chamferDist2;
	for (iIteration = 0; iIteration < nICPIterations; iIteration++)
	{
		ptAssocs.n = 0;
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
			Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
			SampleEdges2(pMesh, edge, edgeSamplePts, iImg);
			for (iSample = 0; iSample < edgeSamplePts.n; iSample++)
			{
				pSamplePt = edgeSamplePts.Element + iSample;
				pSamplePt_ = pSamplePt;
				// pSamplePt_ = edgeSamplePts_.Element + iSample;
				// RVLTRANSF3(pSamplePt->PR, dR, poseMC_.t, pSamplePt_->PC);
				// pSamplePt_->iImgP[0] = (int)round(cameras[iImg].fu * pSamplePt_->PC[0] / pSamplePt_->PC[2] + cameras[iImg].uc);
				// pSamplePt_->iImgP[1] = (int)round(cameras[iImg].fv * pSamplePt_->PC[1] / pSamplePt_->PC[2] + cameras[iImg].vc);
				// if (pSamplePt_->iImgP[0] < 0 || pSamplePt_->iImgP[0] >= RGB.cols || pSamplePt_->iImgP[1] < 0 || pSamplePt_->iImgP[1] >= RGB.rows)
				//	continue;
				// pSamplePt_->iPix = (int)round(pSamplePt_->iImgP[0]) + (int)round(pSamplePt_->iImgP[1]) * cameras[iImg].w;
				chamferDist2 = (float)pVisualizationData->EDTImage[iImg].pPix[pSamplePt_->iPix].d2;
				if (chamferDist2 > ICPThr2)
					continue;
				QImgP[0] = pSamplePt_->iImgP[0] - pVisualizationData->EDTImage[iImg].pPix[pSamplePt_->iPix].dx;
				QImgP[1] = pSamplePt_->iImgP[1] - pVisualizationData->EDTImage[iImg].pPix[pSamplePt_->iPix].dz;
				iQPix = QImgP[0] + QImgP[1] * cameras[iImg].w;
				if (edges[iImg].data[iQPix] == 0)
					continue;
				QImgN = QImgNMap + 2 * iQPix;
				pEdge_ = edge + pSamplePt->edgeIdx;
				csN = QImgN[0] * pEdge_->ImgN[0] + QImgN[1] * pEdge_->ImgN[1];
				if (RVLABS(csN) < csNThr)
					continue;
				pPtAssoc = ptAssoc + ptAssocs.n++;
				pPtAssoc->iMSample = iSample;
				pPtAssoc->iQPix = iQPix;
				pPtAssoc->iImg = iImg;
				pPtAssoc->QImgN[0] = QImgN[0];
				pPtAssoc->QImgN[1] = QImgN[1];
				QN = QNMap[iImg] + 3 * iQPix;
				RVLCOPY3VECTOR(QN, pPtAssoc->QN);
			}
			delete[] edgeSamplePts.Element;
		}
		// Fit3DTo2D2(pMesh, edgeSamplePts, edges, &QNMap[0], poseC_C0, poseMC, ptAssocs, 20, th_, th, et, dR, dt, 0.1f);
		// Fit3DTo2D2(pMesh, edgeSamplePts, edges, &QNMap[0], poseC_C0, poseMC, ptAssocs, 20, th_, th, et, dR, dt, 0.1f);
		// Fit3DTo2D2(pMesh, edgeSamplePts, edges, poseC_C0, poseMC, ptAssocs, 20, th_, th, et, dR, dt, 0.1f);
		// Fit3DTo2D2(pMesh, edgeSamplePts, edges, edge, poseC_C0, poseMC, ptAssocs, PRMem, PCMem, NCMem, 20, th_, th, et, dR, dt, 0.1f);
		Fit3DTo2D2SS(pMesh, edges, edge, poseC_C0, poseMC, ptAssocs, nInitSolutionIterations, th_, th, et, dR, dt, 0.1f);
		RVLMXMUL3X3(dR, poseMC.R, poseMC_.R);
		RVLSUM3VECTORS(poseMC.t, dt, poseMC_.t);
		poseMC = poseMC_;

		if (bVisualizeICPSteps)
		{
			// Visualize point association.

			// cv::cvtColor(pVisualizationData->EDTDisplayImage[iImg], pVisualizationData->displayImg, cv::COLOR_GRAY2RGB);
			// SuperposeBinaryImage(pVisualizationData->displayImg, edges[iImg].data, yellow);
			// //VisualizeEdgeSamples(displayImg, edgeSamplePts);
			// Visualize2DPointAssociation(pVisualizationData->displayImg, ptAssocs, edgeSamplePts.Element);
			// cv::imshow("model", pVisualizationData->displayImg);
			// cv::waitKey(0);

			//

			cv::Mat visualizeICPStepsImg;

			for (int iImg = 0; iImg < numImg; iImg++)
			{
				RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);
				Project3DModelToImage(pMesh, poseMCc, edge, PRMem, PCMem, NCMem, iImg);
				cv::cvtColor(edges[iImg], visualizeICPStepsImg, cv::COLOR_GRAY2RGB);
				pVisualizationData->displayImg = &visualizeICPStepsImg;
				Visualize3DModelImageProjection(*pVisualizationData->displayImg, pMesh, edge);
				std::string displayImgName = "ICP iteration " + std::to_string(iIteration) + " image " + std::to_string(iImg);
				cv::imshow(displayImgName, *pVisualizationData->displayImg);
				cv::waitKey(0);
				cv::destroyWindow(displayImgName);
			}
		}
	}
	pVisualizationData->bVisualizeICPSteps = bVisualizeICPSteps;

	///

	// Visualize final solution.

	uchar red[3] = {255, 0, 0};
	printf("best scene fitting score = %f no. FPs = %d iteration %d\n\n", bestScore, nBestNumFPs, iBestHypothesis);

	Pose3D tempPose;
	cv::Mat visualizeFinalSolutionImg;
	if (pVisualizationData->bVisualizeSolution)
	{
		for (int iImg = 0; iImg < numImg; iImg++)
		{
			visualizeFinalSolutionImg = RGBs[iImg].clone();
			// RVLTRANSF3(poseMC, poseC_C0[iImg].R, poseC_C0[iImg].t, tempPose);
			RVLCOMPTRANSF3DWITHINV(poseC_C0[iImg].R, poseC_C0[iImg].t, poseMC.R, poseMC.t, poseMCc.R, poseMCc.t, tmp3x1);

			pVisualizationData->displayImg = &visualizeFinalSolutionImg;
			Project3DModelToImage(pMesh, poseMCc, edge_, PRMem, PCMem, NCMem, iImg);
			// cv::cvtColor(edges, displayImg, cv::COLOR_GRAY2RGB);
			Visualize3DModelImageProjection(*pVisualizationData->displayImg, pMesh, edge_, red);

			solutions[iImg] = *pVisualizationData->displayImg;
			std::string filename = "/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP-221216/results-20230117-dist/" + std::to_string(counter) + "-" + std::to_string(iImg) + ".png";
			// cout << filename << endl;
			// cv::imwrite(filename, *pVisualizationData->displayImg);
			cv::imshow("solution" + std::to_string(iImg), *pVisualizationData->displayImg);
			cv::waitKey(0);
		}
	}
	delete[] PRMem;
	delete[] PCMem;
	delete[] NCMem;
	delete[] edge;
	delete[] edge_;
	delete[] QImgNMap;
	// delete[] edgeSamplePts.Element;
	delete[] edgeSamplePts_.Element;
	delete[] ptAssoc;

	for (int iSize = 0; iSize < numImg; iSize++)
	{
		delete[] IuMap[iSize];
		delete[] IvMap[iSize];
		delete[] QNMap[iSize];
		delete[] pVisualizationData->EDTImage[iSize].pPix;
	}
	delete[] IuMap;
	delete[] IvMap;
	delete[] QNMap;

	cv::destroyAllWindows();

	return poseMC;
}

void DDDetector::setupFit3DTo2D(
	int numImg,
	cv::Mat *RGBs,
	cv::Mat *edges,
	float *&QImgNMap,
	double **&IuMap,
	double **&IvMap,
	float **&QNMap)
{
	cv::Mat I;
	cv::Mat IBlured;
	QNMap = new float *[numImg];
	// IuMap = new double*[numImg];
	// IvMap = new double*[numImg];

	// for (int iImg = 0; iImg < numImg; iImg++)
	// {
	// 	IuMap[iImg] = new double[RGBs[iImg].rows*RGBs[iImg].cols];
	// 	IvMap[iImg] = new double[RGBs[iImg].rows*RGBs[iImg].cols];
	// }

	// RVL::DDDetector::VisualizeCannyTrackbar(RGBs);

	for (int iImg = 0; iImg < numImg; iImg++)
	{
		pVisualizationData->edgesVis = &RGBs[iImg];

		// Convert to graycsale.
		cv::cvtColor(RGBs[iImg], I, cv::COLOR_BGR2GRAY);

		// Blur the image for better edge detection.
		cv::GaussianBlur(I, IBlured, cv::Size(5, 5), 0);
		// cv::imshow("rgb blurred", IBlured);
		// cv::waitKey(0);

		// Canny edge detection.
		// cv::Mat edges;
		// cv::Canny(IBlured, edges[iImg], 20, 400, 3, false);
		cv::Canny(IBlured, edges[iImg], 660, 1320, 5, false);

		// Display canny edge detected image.
		// cv::imshow("Canny edge detection", edges[iImg]);
		// cv::waitKey(0);
		// cv::destroyWindow("Canny edge detection");

		// Diference of Gaussian.
		cv::GaussianBlur(I, IBlured, cv::Size(7, 7), 0);
		cv::Mat sobelx, sobely, sobelxy;
		cv::Sobel(IBlured, sobelx, CV_64F, 1, 0, 5);
		cv::Sobel(IBlured, sobely, CV_64F, 0, 1, 5);

		int iPix;
		int nPix = RGBs[0].cols * RGBs[0].rows;
		// float* QImgNMap = new float[2 * nPix];
		float *QImgN = QImgNMap;
		QNMap[iImg] = new float[3 * nPix];
		float *QN = QNMap[iImg];
		double *sobelDataX = (double *)(sobelx.data);
		double *sobelDataY = (double *)(sobely.data);
		memcpy(*(IuMap + iImg), sobelDataX, sizeof(double) * sobelx.rows * sobelx.cols);
		memcpy(*(IvMap + iImg), sobelDataY, sizeof(double) * sobely.rows * sobely.cols);
		double *Iu = *(IuMap + iImg);
		double *Iv = *(IvMap + iImg);

		float dI;
		float QP[3], QV[3];
		QP[2] = 1.0f;
		QV[2] = 0.0f;
		float Qu, Qv;
		float fTmp;
		for (iPix = 0; iPix < nPix; iPix++, QN += 3, QImgN += 2, Iu++, Iv++)
		{
			if (!edges[iImg].data[iPix])
				continue;
			QImgN[0] = (float)(*Iu);
			QImgN[1] = (float)(*Iv);
			dI = sqrt(QImgN[0] * QImgN[0] + QImgN[1] * QImgN[1]);
			QImgN[0] /= dI;
			QImgN[1] /= dI;
			QV[0] = QImgN[1];
			QV[1] = -QImgN[0];
			Qu = (float)(iPix % pVisualizationData->edgesVis->cols);
			Qv = (float)(iPix / pVisualizationData->edgesVis->cols);
			QP[0] = (Qu - cameras[iImg].uc) / cameras[iImg].fu;
			QP[1] = (Qv - cameras[iImg].vc) / cameras[iImg].fv;
			RVLCROSSPRODUCT3(QV, QP, QN);
			RVLNORM3(QN, fTmp);
		}
		// double minSobel, maxSobel;
		// cv::minMaxLoc(sobelx, &minSobel, &maxSobel);
		// cv::imshow("sobelx", (sobelx - minSobel) / (maxSobel - minSobel));
		// cv::minMaxLoc(sobely, &minSobel, &maxSobel);
		// cv::imshow("sobely", (sobely - minSobel) / (maxSobel - minSobel));
		// cv::waitKey(0);

		// Compute EDT.

		// CRVLEDT EDT;
		// RVLEDT_PIX_ARRAY EDTImage;
		// cv::Mat EDTDisplayImage;

		// pVisualizationData->EDT = &EDT;
		// pVisualizationData->EDTImage = &EDTImage;
		// pVisualizationData->EDTDisplayImage = &EDTDisplayImage;

		// CRVLEDT EDT;
		pVisualizationData->EDT[iImg].m_Flags |= RVLEDT_FLAG_EDGE_CONTOURS;
		// RVLEDT_PIX_ARRAY EDTImage;
		pVisualizationData->EDTImage[iImg].Width = pVisualizationData->edgesVis->cols;
		pVisualizationData->EDTImage[iImg].Height = pVisualizationData->edgesVis->rows;
		pVisualizationData->EDTImage[iImg].pPix = new RVLEDT_PIX[nPix];
		RVLEDT_BUCKET_ENTRY *EDTBucketMem = new RVLEDT_BUCKET_ENTRY[4 * nPix];
		WORD iBucket;
		for (iBucket = 0; iBucket < 4; iBucket++)
			pVisualizationData->EDT[iImg].m_BucketPtrArray[iBucket] = EDTBucketMem + iBucket * nPix;
		pVisualizationData->EDT[iImg].Border(&pVisualizationData->EDTImage[iImg]);
		pVisualizationData->EDT[iImg].CreateRTSqrtLUT();
		CRVLBuffer EDTBuff;
		EDTBuff.DataBuff = new void *[2 * nPix];
		EDTBuff.m_bOwnData = FALSE;
		// EDT->m_maxd2 = imageWidth * imageWidth + imageHeight * imageHeight;
		pVisualizationData->EDT[iImg].m_maxd2 = fit3DTo2DEDTMaxDist * fit3DTo2DEDTMaxDist;
		CRVLMem mem;
		mem.Create(nPix * sizeof(RVLPTRCHAIN_ELEMENT));
		CRVLMPtrChain boundary;
		boundary.m_pMem = &mem;
		int u, v;
		for (v = 1; v < pVisualizationData->edgesVis->rows - 1; v++)
			for (u = 1; u < pVisualizationData->edgesVis->cols - 1; u++)
			{
				iPix = u + v * pVisualizationData->edgesVis->cols;
				if (edges[iImg].data[iPix] > 0)
				{
					pVisualizationData->EDTImage[iImg].pPix[iPix].d2 = 0;
					boundary.Add((void *)(pVisualizationData->EDTImage[iImg].pPix + iPix));
				}
			}
		pVisualizationData->EDT[iImg].Apply(&boundary, NULL, &pVisualizationData->EDTImage[iImg], &EDTBuff);
		delete[] EDTBucketMem;
		delete[] EDTBuff.DataBuff;

		// Visualize EDT.

		pVisualizationData->EDTDisplayImage[iImg] = cv::Mat(pVisualizationData->edgesVis->rows, pVisualizationData->edgesVis->cols, CV_8UC1);
		uchar *pPix = pVisualizationData->EDTDisplayImage[iImg].data;
		RVLEDT_PIX *pEDTPix = pVisualizationData->EDTImage[iImg].pPix;
		float k = 255.0 / sqrt((float)(pVisualizationData->EDT[iImg].m_maxd2));
		for (iPix = 0; iPix < nPix; iPix++, pPix++, pEDTPix++)
			*pPix = (pEDTPix->d2 <= pVisualizationData->EDT[iImg].m_maxd2 ? (uchar)round(k * sqrt((float)(pEDTPix->d2))) : 255);
		// cv::imshow("EDT", pVisualizationData->EDTDisplayImage[iImg]);
		// cv::waitKey(0);
	}

	// cv::imshow("EDT1", pVisualizationData->EDTDisplayImage[0]);
	// cv::imshow("EDT2", pVisualizationData->EDTDisplayImage[1]);
	// cv::waitKey(0);
}

float DDDetector::distanceDifOf2Pts(cv::Point p, cv::Point L, cv::Point R)
{
	float distanceToL = sqrt(pow(p.x - L.x, 2) + pow(p.y - L.y, 2));
	float distanceToR = sqrt(pow(p.x - R.x, 2) + pow(p.y - R.y, 2));
	return abs(distanceToL - distanceToR);
}

int DDDetector::findNextEdge(std::vector<int> topFaceEdges, RVL::RECOG::DDD::Edge *edge, int iVertexBack)
{
	int iEdge;
	RVL::RECOG::DDD::Edge *MEdge_;
	for (int i = 0; i < topFaceEdges.size(); i++)
	{
		iEdge = topFaceEdges[i];
		MEdge_ = edge + iEdge;
		if (MEdge_->iVertex[0] == iVertexBack)
			break;
	}
	return iEdge;
}

void DDDetector::ModelPtAssociationLHTCP(
	RVL::Mesh *pMesh,
	RVL::RECOG::DDD::Edge *edge,
	Array<RECOG::DDD::EdgeSample> &edgeSamplePts,
	RVL::RECOG::DDD::PtAssoc *ptAssoc,
	std::vector<int> &fifthMIdxs)

{
	RVL::RECOG::DDD::Edge *MpEdge_;
	int longestMEdgeIdxs[4] = {}; // init with zeros
	std::vector<int> topFaceEdges;
	float lengthScores[4] = {}; // init with zeros
	int longestMEdgeIdx = 0, secondLongestMEdgeIdx = 0;
	float lengthScore = 0.0f, secondLengthScore = 0.0f;
	float leftMostScore, rightMostScore;
	float leftMostXIdx, rightMostXIdx;
	leftMostScore = 10000.0f, rightMostScore = 0.0f;
	cv::Point rightMostPt, leftMostPt;

	// Find 4 longest edge points in the model - longest has index 0
	for (int iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		MpEdge_ = edge + iEdge;
		if (MpEdge_->bVisible && MpEdge_->length > lengthScores[0])
		{
			longestMEdgeIdxs[3] = longestMEdgeIdxs[2];
			longestMEdgeIdxs[2] = longestMEdgeIdxs[1];
			longestMEdgeIdxs[1] = longestMEdgeIdxs[0];
			longestMEdgeIdxs[0] = iEdge;

			lengthScores[3] = lengthScores[2];
			lengthScores[2] = lengthScores[1];
			lengthScores[1] = lengthScores[0];
			lengthScores[0] = MpEdge_->length;
		}
		else if (MpEdge_->bVisible && MpEdge_->length > lengthScores[1])
		{
			longestMEdgeIdxs[3] = longestMEdgeIdxs[2];
			longestMEdgeIdxs[2] = longestMEdgeIdxs[1];
			longestMEdgeIdxs[1] = iEdge;

			lengthScores[3] = lengthScores[2];
			lengthScores[2] = lengthScores[1];
			lengthScores[1] = MpEdge_->length;
		}
		else if (MpEdge_->bVisible && MpEdge_->length > lengthScores[2])
		{
			longestMEdgeIdxs[3] = longestMEdgeIdxs[2];
			longestMEdgeIdxs[2] = iEdge;

			lengthScores[3] = lengthScores[2];
			lengthScores[2] = MpEdge_->length;
		}
		else if (MpEdge_->bVisible && MpEdge_->length > lengthScores[3])
		{
			longestMEdgeIdxs[3] = iEdge;

			lengthScores[3] = MpEdge_->length;
		}
		// Check for edges on the top face of the LHTCP model
		// else if (MpEdge_->bVisible && MpEdge_->iVertex[0] < 20 && MpEdge_->iVertex[1] < 20)
		if (MpEdge_->iVertex[0] < 20 && MpEdge_->iVertex[1] < 20)
		{
			// cout << "iEdge: " << iEdge << ", Visibility: "<< MpEdge_->bVisible << ", Pt[0]: " << MpEdge_->ImgP[0] << endl;
			topFaceEdges.push_back(iEdge);
			cv::Point tempP(MpEdge_->ImgP[0]); // Check only the first point of the edge
			if (MpEdge_->bVisible && tempP.x < leftMostScore)
			{
				leftMostScore = tempP.x;
				leftMostXIdx = iEdge;
				leftMostPt = tempP;
			}
			if (MpEdge_->bVisible && tempP.x > rightMostScore)
			{
				rightMostScore = tempP.x;
				rightMostXIdx = iEdge;
				rightMostPt = tempP;
			}
		}
	}

	std::vector<int> topFaceEdgesUp, topFaceEdgesDown;
	int iEdge = rightMostXIdx;
	RVL::RECOG::DDD::Edge *MEdge_ = edge + iEdge;
	RVL::RECOG::DDD::Edge *MEdgeF_;
	RVL::RECOG::DDD::EdgeSample *MEdgeSample_;
	int iVertexBack = MEdge_->iVertex[1];
	while (iEdge != leftMostXIdx)
	{
		MEdge_ = edge + iEdge;
		iVertexBack = MEdge_->iVertex[1];
		if (MEdge_->bVisible)
			topFaceEdgesUp.push_back(iEdge);
		iEdge = RVL::DDDetector::findNextEdge(topFaceEdges, edge, iVertexBack);
	}
	iEdge = leftMostXIdx;
	MEdge_ = edge + iEdge;
	iVertexBack = MEdge_->iVertex[1];
	while (iEdge != rightMostXIdx)
	{
		MEdge_ = edge + iEdge;
		iVertexBack = MEdge_->iVertex[1];
		if (MEdge_->bVisible)
			topFaceEdgesDown.push_back(iEdge);
		iEdge = RVL::DDDetector::findNextEdge(topFaceEdges, edge, iVertexBack);
	}

	// Sort model edge points based on the line idxs
	RVL::RECOG::DDD::EdgeSample *edgeSample;
	std::vector<int> longestEdgePtsM, secondLongestEdgePtsM;
	std::vector<cv::Point> longestEdgesPtsM[4], topFaceEdgePtsM;
	std::vector<int> upperCylinderEdgePtsUp, upperCylinderEdgePtsDown;

	// cv::Mat imgTemp; // = cv::Mat::zeros(cv::Size(1280, 1024), CV_8UC1);
	// cv::cvtColor(pVisualizationData->displayImg, imgTemp, cv::COLOR_GRAY2RGB);
	// VisualizeEdgeSamples(imgTemp, edgeSamplePts);
	// cv::imshow("edge sample pts", imgTemp);
	// cv::waitKey(0);

	// Single out model edge points of the upper cylinder
	for (int iEdgeSample = 0; iEdgeSample < edgeSamplePts.n; iEdgeSample++)
	{
		edgeSample = edgeSamplePts.Element + iEdgeSample;
		for (int i = 0; i < 5; i++)
		{
			if (i < 4)
			{
				if (edgeSample->edgeIdx == longestMEdgeIdxs[i])
					longestEdgesPtsM[i].push_back(cv::Point(iEdgeSample, edgeSample->iImgP[1]));
			}
			// Check if the edge sample belongs to the top face of the upper cylinder
			else if (std::count(topFaceEdges.begin(), topFaceEdges.end(), edgeSample->edgeIdx))
			{
				// cout << edgeSample->edgeIdx << endl;
				topFaceEdgePtsM.push_back(cv::Point(iEdgeSample, edgeSample->iImgP[1]));
				// cv::circle(imgTemp, cv::Point(edgeSample->iImgP[0], edgeSample->iImgP[1]), 3, cv::Scalar(0, 0, 255));
				if (std::count(topFaceEdgesUp.begin(), topFaceEdgesUp.end(), edgeSample->edgeIdx))
					upperCylinderEdgePtsUp.push_back(iEdgeSample);
				else
					upperCylinderEdgePtsDown.push_back(iEdgeSample);
			}
		}
	}
	// cv::imshow("temp", imgTemp);
	// cv::waitKey(0);

	int iEdgeSample, fifthMIdxUp, fifthMIdxDown;
	float distance, distanceScore;
	distanceScore = 10000.0f;
	for (int i = 0; i < upperCylinderEdgePtsUp.size(); i++)
	{
		iEdgeSample = upperCylinderEdgePtsUp[i];
		edgeSample = edgeSamplePts.Element + iEdgeSample;
		distance = RVL::DDDetector::distanceDifOf2Pts(cv::Point(edgeSample->iImgP[0], edgeSample->iImgP[1]), leftMostPt, rightMostPt);
		if (distance < distanceScore)
		{
			distanceScore = distance;
			fifthMIdxUp = iEdgeSample;
		}
	}
	distanceScore = 10000.0f;
	for (int i = 0; i < upperCylinderEdgePtsDown.size(); i++)
	{
		iEdgeSample = upperCylinderEdgePtsDown[i];
		edgeSample = edgeSamplePts.Element + iEdgeSample;
		distance = RVL::DDDetector::distanceDifOf2Pts(cv::Point(edgeSample->iImgP[0], edgeSample->iImgP[1]), leftMostPt, rightMostPt);
		if (distance < distanceScore)
		{
			distanceScore = distance;
			fifthMIdxDown = iEdgeSample;
		}
	}

	for (int i = 0; i < 5; i++)
	{
		if (i < 4)
			std::sort(longestEdgesPtsM[i].begin(), longestEdgesPtsM[i].end(), RVL::DDDetector::compareEdgeSampleY);
		else // Sort topface model edge points by y-value on image (high to low)
			std::sort(topFaceEdgePtsM.begin(), topFaceEdgePtsM.end(), RVL::DDDetector::compareEdgeSampleY);
	}

	// Sort edges from left to right (0-1 lower cylinder, 2-3 upper cylinder)
	RVL::RECOG::DDD::EdgeSample *edgeSample1, *edgeSample2;
	// 0-1
	edgeSample1 = edgeSamplePts.Element + longestEdgesPtsM[0][0].x;
	edgeSample2 = edgeSamplePts.Element + longestEdgesPtsM[1][0].x;
	if (edgeSample2->iImgP[0] < edgeSample1->iImgP[0])
	{
		std::swap(longestEdgesPtsM[1], longestEdgesPtsM[0]);
		std::swap(longestMEdgeIdxs[1], longestMEdgeIdxs[0]);
	}
	// 2-3
	edgeSample1 = edgeSamplePts.Element + longestEdgesPtsM[2][0].x;
	edgeSample2 = edgeSamplePts.Element + longestEdgesPtsM[3][0].x;
	if (edgeSample2->iImgP[0] < edgeSample1->iImgP[0])
	{
		std::swap(longestEdgesPtsM[3], longestEdgesPtsM[2]);
		std::swap(longestMEdgeIdxs[3], longestMEdgeIdxs[2]);
	}

	ptAssoc[0].iMSample = longestEdgesPtsM[2][(int)longestEdgesPtsM[2].size() / 4].x;
	ptAssoc[1].iMSample = longestEdgesPtsM[2][(int)longestEdgesPtsM[2].size() * 3 / 4].x;
	ptAssoc[2].iMSample = longestEdgesPtsM[3][(int)longestEdgesPtsM[3].size() / 4].x;
	ptAssoc[3].iMSample = longestEdgesPtsM[3][(int)longestEdgesPtsM[3].size() * 3 / 4].x;

	// fifthMIdxs.push_back(topFaceEdgePtsM.back().x);
	// fifthMIdxs.push_back(topFaceEdgePtsM[0].x);

	fifthMIdxs.clear();
	if (upperCylinderEdgePtsUp.size() > 0)
		fifthMIdxs.push_back(fifthMIdxUp);
	if (upperCylinderEdgePtsDown.size() > 0)
		fifthMIdxs.push_back(fifthMIdxDown);

	if (fifthMIdxs.size() == 1)
		fifthMIdxs.push_back(fifthMIdxs[0]);

	// for (int i = 0; i < fifthMIdxs.size(); i++)
	// {
	// 	cout << "fifthMIdxs: " << fifthMIdxs[i] << endl;
	// }
	// cout << endl;

	// cv::Mat imtemp;
	// cv::cvtColor(edges, imtemp, CV_GRAY2RGB);
	// VisualizeEdgeSamples(imtemp, edgeSamplePts);
	// for(int yy = 0; yy < fifthMIdxs.size(); yy++)
	// {
	// 	edgeSample = edgeSamplePts.Element + fifthMIdxs[yy];
	// 	// cout << "x: " << edgeSample->iImgP[0] << ", y: " << edgeSample->iImgP[1] << endl;
	// 	cv::circle(imtemp, cv::Point(edgeSample->iImgP[0], edgeSample->iImgP[1]), 3, cv::Scalar(0, 0, 255));
	// }
	// cv::imshow("Temp", imtemp);
	// cv::waitKey(0);
}

void DDDetector::loadBaslerCamerasParams(std::string loadPath, cv::Mat *&camsMatrix, cv::Mat *&camsDist)
{
	cv::FileStorage fs;
	cv::Mat cameraMatrix;
#ifdef RVLLINUX
	char slash = '/';
#else
			char slash = '\\';
#endif

	for (int i = 0; i < 2; i++)
	{
		std::string cameraParamsFilePath = loadPath + slash + "basler_cam" + std::to_string(i) + "_params.yaml";
		fs.open(cameraParamsFilePath, cv::FileStorage::READ);
		fs["camera_matrix"] >> cameraMatrix;
		cameras[i].fu = (float)cameraMatrix.at<double>(cv::Point(0, 0));
		cameras[i].fv = (float)cameraMatrix.at<double>(cv::Point(1, 1));
		cameras[i].uc = (float)cameraMatrix.at<double>(cv::Point(2, 0));
		cameras[i].vc = (float)cameraMatrix.at<double>(cv::Point(2, 1));

		// fs["projection_matrix"] >> cameraMatrix;
		// cameras[i].fu = (float)cameraMatrix.at<double>(cv::Point(0, 0));
		// cameras[i].fv = (float)cameraMatrix.at<double>(cv::Point(1, 1));
		// cameras[i].uc = (float)cameraMatrix.at<double>(cv::Point(2, 0));
		// cameras[i].vc = (float)cameraMatrix.at<double>(cv::Point(2, 1));

		fs["image_width"] >> cameras[i].w;
		fs["image_height"] >> cameras[i].h;

		camsMatrix[i] = cameraMatrix.clone();
		fs["distortion_coefficients"] >> camsDist[i];

		fs.release();
	}
}

void DDDetector::loadBaslerExtrinsicParams(std::string loadPath, RVL::Pose3D *poseC_C0)
{
#ifdef RVLLINUX
	char slash = '/';
#else
			char slash = '\\';
#endif
	std::string cameraParamsFilePath = loadPath + slash + "extrinsicCalibration.yaml";
	cv::Mat R, t, TC1C0;
	cv::FileStorage fs;
	fs.open(cameraParamsFilePath, cv::FileStorage::READ);
	fs["TC1C0"] >> TC1C0;
	fs.release();

	std::fill(poseC_C0[0].R, poseC_C0[0].R + sizeof(poseC_C0[0].R), 0.0f); // fill array with 0's
	poseC_C0[0].R[0] = poseC_C0[0].R[4] = poseC_C0[0].R[8] = 1.0f;		   // 1's for identity matrix (camera does not have rotation wrt itself)
	std::fill(poseC_C0[0].t, poseC_C0[0].t + sizeof(poseC_C0[0].t), 0.0f); // fill array with 0's (no translation)
	cv::Rect Rr(0, 0, 3, 3);
	cv::Rect Tt(3, 0, 1, 3);
	R = TC1C0(Rr);
	t = TC1C0(Tt);
	cv::Mat Rfloat, tfloat;
	R.convertTo(Rfloat, CV_32FC1);
	t.convertTo(tfloat, CV_32FC1);

	// cout << Rfloat << endl;
	std::memcpy(poseC_C0[1].R, Rfloat.ptr(0), Rfloat.total() * sizeof(float));
	std::memcpy(poseC_C0[1].t, tfloat.ptr(0), tfloat.total() * sizeof(float));

	// for (int i = 0; i < 3; i++)
	// {
	// 	const double* Rt_i = TC1C0.ptr<double>(i); // rows
	// 	for (int j = 0; j < 3; j++)
	// 		poseC_C0[1].R[3 * i + j] = (float)Rt_i[j];
	// 	poseC_C0[1].t[i] = (float)Rt_i[3];
	// }
	// Pose3D poseTmp;
	// poseTmp = poseC_C0[1];
	// RVLINVTRANSF3D(poseTmp.R, poseTmp.t, poseC_C0[1].R, poseC_C0[1].t);
}

void DDDetector::SampleEdges2(
	Mesh *pMesh,
	RECOG::DDD::Edge *edge,
	Array<RECOG::DDD::EdgeSample> &edgeSamplePts,
	int cameraNum)
{
	int nEdgeSamples = fit3DTo2DNoEdgeSamples;
	edgeSamplePts.Element = new RECOG::DDD::EdgeSample[nEdgeSamples];
	RECOG::DDD::EdgeSample *pSamplePt = edgeSamplePts.Element;
	float edgeBinStart = 0.0f;
	int iEdge;
	RECOG::DDD::Edge *pEdge_;
	for (iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		pEdge_ = edge + iEdge;
		if (pEdge_->bVisible)
		{
			pEdge_->binEnd = edgeBinStart + pEdge_->length;
			edgeBinStart = pEdge_->binEnd;
		}
		else
			pEdge_->binEnd = 0.0f;
	}
	float edgeSampleInc = edgeBinStart / (float)nEdgeSamples;
	float firsEdgeSample = 0.5f * edgeSampleInc;
	int iSample;
	float sample = firsEdgeSample;
	iEdge = 0;
	float s;
	edgeBinStart = 0.0f;
	float V3Tmp[3];
	for (iSample = 0; iSample < nEdgeSamples; iSample++, sample += edgeSampleInc)
	{
		while (sample > edge[iEdge].binEnd)
		{
			if (edge[iEdge].bVisible)
				edgeBinStart = edge[iEdge].binEnd;
			iEdge++;
		}
		pEdge_ = edge + iEdge;
		s = sample - edgeBinStart;
		RVLSCALE3VECTOR(pEdge_->VR, s, V3Tmp);
		RVLSUM3VECTORS(pEdge_->PR[0], V3Tmp, pSamplePt->PR);
		RVLSCALE3VECTOR(pEdge_->VC, s, V3Tmp);
		RVLSUM3VECTORS(pEdge_->PC[0], V3Tmp, pSamplePt->PC);
		pSamplePt->ImgP[0] = cameras[cameraNum].fu * pSamplePt->PC[0] / pSamplePt->PC[2] + cameras[cameraNum].uc;
		pSamplePt->ImgP[1] = cameras[cameraNum].fv * pSamplePt->PC[1] / pSamplePt->PC[2] + cameras[cameraNum].vc;
		pSamplePt->iImgP[0] = (int)round(pSamplePt->ImgP[0]);
		pSamplePt->iImgP[1] = (int)round(pSamplePt->ImgP[1]);
		if (pSamplePt->iImgP[0] >= 0 && pSamplePt->iImgP[0] < cameras[cameraNum].w && pSamplePt->iImgP[1] >= 0 && pSamplePt->iImgP[1] < cameras[cameraNum].h)
		{
			pSamplePt->iPix = pSamplePt->iImgP[0] + pSamplePt->iImgP[1] * cameras[cameraNum].w;
			pSamplePt->edgeIdx = iEdge;
			pSamplePt++;
		}
	}
	edgeSamplePts.n = pSamplePt - edgeSamplePts.Element;
}

void DDDetector::VisualizeCannyTrackbar(cv::Mat *images)
{
	cv::Mat IGray, IBlured;
	cv::Mat edges;
	std::string windowName = "Imgs";
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

	int lowThreshold = 600;
	int maxLowThreshold = 1500;
	int highTreshold = 1500;
	int maxHightThreshold = 2000;

	CannyDisplayData userdata;

	for (int i = 0; i < 2; i++)
	{
		cv::cvtColor(images[i], IGray, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(IGray, IBlured, cv::Size(3, 3), 0);

		userdata.src = IBlured.clone();
		userdata.lowThreshold = lowThreshold;
		userdata.maxLowThreshold = maxLowThreshold;
		userdata.windowName = windowName;
		userdata.highTreshold = highTreshold;
		userdata.maxHighThreshold = maxHightThreshold;

		cv::createTrackbar("Max threshold: ", windowName, &userdata.highTreshold, maxHightThreshold, RVL::CannyThresholdCallback, &userdata);
		cv::createTrackbar("Min threshold: ", windowName, &userdata.lowThreshold, maxLowThreshold, RVL::CannyThresholdCallback, &userdata);
		RVL::CannyThresholdCallback(0, &userdata);
		cv::waitKey(0);
	}
}

void RVL::CannyThresholdCallback(int, void *userdata)
{

	CannyDisplayData *data = (CannyDisplayData *)userdata;

	if (data->lowThreshold > data->highTreshold)
	{
		data->highTreshold = data->lowThreshold;
	}

	cv::Canny(data->src, data->dst, data->lowThreshold, data->highTreshold, 5, false);

	cv::imshow(data->windowName, data->dst);
}

void DDDetector::drawLine(cv::Mat &image, cv::Vec2f inLine, cv::Scalar color)
{
	float rho, theta;
	rho = inLine[0];
	theta = inLine[1];

	cv::Point pt1, pt2;
	double a = cos(theta), b = sin(theta);
	double x0 = a * rho, y0 = b * rho;

	pt1.x = cvRound(x0 + 1000 * (-b));
	pt1.y = cvRound(y0 + 1000 * (a));
	pt2.x = cvRound(x0 - 1000 * (-b));
	pt2.y = cvRound(y0 - 1000 * (a));

	cv::line(image, pt1, pt2, color, 1, 8);
}

bool DDDetector::ParseCSV(std::string path, bool bHasHeader, std::vector<std::vector<std::string>> &content)
{
	std::vector<std::string> row;
	std::string line, wrd;

	fstream file(path, std::ios::in);
	if (file.is_open())
	{
		while (getline(file, line))
		{
			row.clear();

			std::stringstream str(line);
			while (getline(str, wrd, ','))
				row.push_back(wrd);
			content.push_back(row);
		}
	}
	else
	{
		cout << "Could not open the file with the path: " << path << endl;
		return false;
	}

	if (bHasHeader)
		content.erase(content.begin());

	file.close();

	return true;
}

// END V. Simundic