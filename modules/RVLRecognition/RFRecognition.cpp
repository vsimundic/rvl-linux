//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
//#include <vtkCubeSource.h>
#include <vtkAxesActor.h>
#include "Util.h"
#include "Graph.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLMeshNoiser.h" //VIDOVIC
#include "RVLRecognition.h"
#include "RFRecognition.h"

using namespace RVL;

RFRecognition::RFRecognition()
{
	mode = RVLRECOGNITION_MODE_RECOGNITION;
	pSurfels = NULL;
	pSurfelDetector = NULL;
	modelMeshFileName = NULL;
	featureFileName = NULL;
	markMap = NULL;
	iSurfBuff = NULL;
	iSurfBuff2 = NULL;
	lineMem = NULL;
	voxelMem = NULL;
	voxel8Pack.Element = NULL;
	voxel.Element = NULL;
	
	meshSize = 0;
	surfelGraphSize = 0;

	iRefSurfel2 = 0;
	iRefSurfel2 = 1;
	featureDetectionParams.dp = 10.0f;
	featureDetectionParams.dl = 10.0f;
	featureDetectionParams.r = 200.0f;
	kLineLength = 0.2f;
	maxGap = 10.0f;
	mincnx = COS45;
	descriptorSize = 20;
	voxelSize = 10.0f;
	minRefSurfelSize = 100;
	minRefLineSize = 20.0f;
	eNThr = 0.866f;
	ePThr = 10.0f;
	matchThr = 0.9f;
}


RFRecognition::~RFRecognition()
{
	RVL_DELETE_ARRAY(modelMeshFileName);
	RVL_DELETE_ARRAY(featureFileName);
	RVL_DELETE_ARRAY(markMap);
	RVL_DELETE_ARRAY(iSurfBuff);
	RVL_DELETE_ARRAY(iSurfBuff2);
	RVL_DELETE_ARRAY(lineMem);
	RVL_DELETE_ARRAY(voxelMem);
	RVL_DELETE_ARRAY(voxel8Pack.Element);
	RVL_DELETE_ARRAY(voxel.Element);
}

void RFRecognition::Init(
	Mesh *pMesh,
	SurfelGraph *pSurfels)
{
	int newMeshSize = pMesh->NodeArray.n;

	if (newMeshSize > meshSize)
	{
		RVL_DELETE_ARRAY(lineMem);

		meshSize = newMeshSize;

		lineMem = new RECOG::Line3D[meshSize];
	}
	
	int newSurfelGraphSize = pSurfels->NodeArray.n;

	if (newSurfelGraphSize > surfelGraphSize)
	{
		RVL_DELETE_ARRAY(markMap);
		RVL_DELETE_ARRAY(iSurfBuff);
		RVL_DELETE_ARRAY(iSurfBuff2);

		surfelGraphSize = newSurfelGraphSize;

		markMap = new unsigned char[surfelGraphSize];
		memset(markMap, 0, sizeof(unsigned char) * surfelGraphSize);
		iSurfBuff = new int[surfelGraphSize];
		iSurfBuff2 = new int[surfelGraphSize];
	}	
}

void RFRecognition::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("Recognition.mode", RVLPARAM_TYPE_ID, &mode);
	ParamList.AddID(pParamData, "TRAINING", RVLRECOGNITION_MODE_TRAINING);
	pParamData = ParamList.AddParam("RFRecognition.iRefSurfel", RVLPARAM_TYPE_INT, &iRefSurfel);
	pParamData = ParamList.AddParam("RFRecognition.iRefSurfel2", RVLPARAM_TYPE_INT, &iRefSurfel2);
	pParamData = ParamList.AddParam("RFRecognition.ModelMeshFileName", RVLPARAM_TYPE_STRING, &modelMeshFileName);
	pParamData = ParamList.AddParam("RFRecognition.FeatureFileName", RVLPARAM_TYPE_STRING, &featureFileName);
	pParamData = ParamList.AddParam("RFRecognition.Feature.dp", RVLPARAM_TYPE_FLOAT, &(featureDetectionParams.dp));
	pParamData = ParamList.AddParam("RFRecognition.Feature.dl", RVLPARAM_TYPE_FLOAT, &(featureDetectionParams.dl));
	pParamData = ParamList.AddParam("RFRecognition.Feature.r", RVLPARAM_TYPE_FLOAT, &(featureDetectionParams.r));
	pParamData = ParamList.AddParam("RFRecognition.maxGap", RVLPARAM_TYPE_FLOAT, &maxGap);
	pParamData = ParamList.AddParam("RFRecognition.descriptorSize", RVLPARAM_TYPE_INT, &descriptorSize);
	pParamData = ParamList.AddParam("RFRecognition.voxelSize", RVLPARAM_TYPE_FLOAT, &voxelSize);
	pParamData = ParamList.AddParam("RFRecognition.minRefSurfelSize", RVLPARAM_TYPE_INT, &minRefSurfelSize);
	pParamData = ParamList.AddParam("RFRecognition.minRefLineSize", RVLPARAM_TYPE_FLOAT, &minRefLineSize);
	pParamData = ParamList.AddParam("RFRecognition.eNThr", RVLPARAM_TYPE_FLOAT, &eNThr);
	pParamData = ParamList.AddParam("RFRecognition.ePThr", RVLPARAM_TYPE_FLOAT, &ePThr);
	pParamData = ParamList.AddParam("RFRecognition.matchThr", RVLPARAM_TYPE_FLOAT, &matchThr);	
}

void RFRecognition::CreateModelDatabase()
{
	// Load model mesh from file.

	Mesh mesh;

	mesh.LoadPolyDataFromPLY(modelMeshFileName);

	mesh.CreateOrderedMeshFromPolyData();

	// Segment model mesh to surfels.

	pSurfels->Init(&mesh);

	pSurfelDetector->Init(&mesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(&mesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	// Initialize recognition.

	Init(&mesh, pSurfels);

	// Detect feature base.

	printf("Detect feature...");

	int objectID = 0;

	RECOG::RFFeatureBase featureBase;

	DetectFeatureBase(&mesh, pSurfels, iRefSurfel, &featureBase);

	// Detect features.

	int i;

	for (i = 0; i < featureBase.lineArray[0].n; i++)
		if (featureBase.lineArray[0].Element[i].iSurfel == iRefSurfel2)
			break;

	QList<RECOG::RFFeature> *pFeatureList = &modelFeatureList;

	RVLQLIST_INIT(pFeatureList);

	DetectFeatures(&mesh, pSurfels, &featureBase, 0, i, pFeatureList, pMem0);

	// Create descriptor.

	CreateDescriptor(&mesh, pSurfels, modelFeatureList.pFirst, pMem);

#ifdef RVLRFRECOGNITION_DEBUG
	FILE *fp = fopen("C:\\RVL\\Debug\\Lines0.txt", "a");

	RECOG::DebugWriteDescriptor(fp, modelFeatureList.pFirst, 10.0f);

	fclose(fp);
#endif

	// Assign objectID and frameID to the feature.

	modelFeatureList.pFirst->objectID = 0;
	modelFeatureList.pFirst->frameID = 0;

	printf("completed.\n");

	// Save feature.

	printf("Save feature to %s...", featureFileName);

	FILE *fpFeature = fopen(featureFileName, "wb");

	SaveFeature(fpFeature, modelFeatureList.pFirst);

	fclose(fpFeature);

	printf("completed.\n");

#ifdef RVLRFRECOGNITION_FEATURE_BASE_VISUALIZATION
	// Visualization

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	pSurfels->NodeColors(SelectionColor);

	Visualizer visualizer;

	visualizer.Create();

	InitDisplay(&visualizer, &mesh);
	
	pSurfels->Display(&visualizer, &mesh);

	RECOG::RFFeature *pFeature = modelFeatureList.pFirst;

	while (pFeature)
	{
		vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

		axes->SetTotalLength(10.0, 10.0, 10.0);

		// Create a mapper and actor.
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		double T[16];
		T[0] = (double)(pFeature->R[0]);
		T[1] = (double)(pFeature->R[1]);
		T[2] = (double)(pFeature->R[2]);
		T[4] = (double)(pFeature->R[3]);
		T[5] = (double)(pFeature->R[4]);
		T[6] = (double)(pFeature->R[5]);
		T[8] = (double)(pFeature->R[6]);
		T[9] = (double)(pFeature->R[7]);
		T[10] = (double)(pFeature->R[8]);
		T[3] = (double)(pFeature->t[0]);
		T[7] = (double)(pFeature->t[1]);
		T[11] = (double)(pFeature->t[2]);
		T[12] = T[13] = T[14] = 0.0;
		T[15] = 1.0;

		transform->SetMatrix(T);

		axes->SetUserTransform(transform);

		vtkMatrix4x4 *T_ = axes->GetMatrix();

		visualizer.renderer->AddActor(axes);

		pFeature = pFeature->pNext;
	}

	//RECOG::Hypothesis *pHypothesis = sceneInterpretation.pFirst;

	//while (pHypothesis)
	//{
	//	vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

	//	axes->SetTotalLength(10.0, 10.0, 10.0);

	//	// Create a mapper and actor.
	//	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	//	double T[16];
	//	T[0] = (double)(pHypothesis->RF[0]);
	//	T[1] = (double)(pHypothesis->RF[1]);
	//	T[2] = (double)(pHypothesis->RF[2]);
	//	T[4] = (double)(pHypothesis->RF[3]);
	//	T[5] = (double)(pHypothesis->RF[4]);
	//	T[6] = (double)(pHypothesis->RF[5]);
	//	T[8] = (double)(pHypothesis->RF[6]);
	//	T[9] = (double)(pHypothesis->RF[7]);
	//	T[10] = (double)(pHypothesis->RF[8]);
	//	T[3] = (double)(pHypothesis->tF[0]);
	//	T[7] = (double)(pHypothesis->tF[1]);
	//	T[11] = (double)(pHypothesis->tF[2]);
	//	T[12] = T[13] = T[14] = 0.0;
	//	T[15] = 1.0;

	//	transform->SetMatrix(T);

	//	axes->SetUserTransform(transform);

	//	vtkMatrix4x4 *T_ = axes->GetMatrix();

	//	pVisualizer->renderer->AddActor(axes);

	//	pHypothesis = pHypothesis->pNext;
	//}

	visualizer.Run();
#endif
}

bool RFRecognition::LoadModelDatabase()
{
	RECOG::RFFeature *pFeature;

	RVLMEM_ALLOC_STRUCT(pMem0, RECOG::RFFeature, pFeature);

	FILE *fp = fopen(featureFileName, "rb");

	if (fp == NULL)
		return false;	

	LoadFeature(fp, pFeature, pMem0);

	fclose(fp);

	QList<RECOG::RFFeature> *pModelFeatureList = &(modelFeatureList);

	RVLQLIST_INIT(pModelFeatureList);

	RVLQLIST_ADD_ENTRY(pModelFeatureList, pFeature);

	return true;
}

void RFRecognition::DetectFeatureBase(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iRefSurfel,
	RECOG::RFFeatureBase *pFeatureBase)
{
	// Generate feature base.

	pFeatureBase->iSurfel = iRefSurfel;

#ifdef RVLRFRECOGNITION_DEBUG
	FILE *fp = fopen("C:\\RVL\\Debug\\Lines0.txt", "w");

	fclose(fp);

	fp = fopen("C:\\RVL\\Debug\\Lines1.txt", "w");

	fclose(fp);
#endif

	printf("Generating feature.\n");

	Surfel *pRefSurfel = pSurfels->NodeArray.Element + iRefSurfel;

	float *N = pRefSurfel->N;

	int nBoundaryPts = 0;

	int i;

	for (i = 0; i < pRefSurfel->BoundaryArray.n; i++)
		nBoundaryPts += pRefSurfel->BoundaryArray.Element[i].n;

	Array<float> sBuff;
	sBuff.Element = new float[2 * nBoundaryPts];
	sBuff.n = nBoundaryPts;

	int *piPut = iSurfBuff;

	*(piPut++) = iRefSurfel;

	int *piFetch = iSurfBuff;

	markMap[iRefSurfel] = 1;

	pFeatureBase->lineArray[0].Element = lineMem;
	pFeatureBase->lineArray[0].n = 0;
	pFeatureBase->lineArray[1].Element = lineMem + meshSize / 2;
	pFeatureBase->lineArray[1].n = 0;

	RECOG::RFRegionGrowingData RGData;

	RGData.pMesh = pMesh;
	RGData.pSurfels = pSurfels;
	RGData.N = N;
	RGData.d0 = pRefSurfel->d;
	RGData.dp = featureDetectionParams.dp;
	RGData.lineArray = pFeatureBase->lineArray;
	RGData.psBuff = &sBuff;
	RGData.markMap = markMap;
	RGData.piVisited = iSurfBuff2;

	*(RGData.piVisited++) = iRefSurfel;

	int *piSurfBuffEnd = RegionGrowing<SurfelGraph, Surfel, SURFEL::Edge, SURFEL::EdgePtr, RECOG::RFRegionGrowingData, RECOG::RegionGrowingOperation>(pSurfels, &RGData, piFetch, piPut);

	int *piSurf;

	for (piSurf = iSurfBuff2; piSurf < RGData.piVisited; piSurf++)
		markMap[*piSurf] = 0;

//#ifdef RVLRFRECOGNITION_DEBUG
//	int iMaxLine;
//	float maxLineLength;
//	int iLineArray, iLine;
//	RECOG::Line3D *pLine;
//	Array<RECOG::Line3D> *pLineArray;
//	float *P1, *P2;
//
//	for (iLineArray = 0; iLineArray < 2; iLineArray++)
//	{
//		// Identify the longest line.
//
//		maxLineLength = 0.0;
//
//		pLineArray = pFeatureBase->lineArray + iLineArray;
//
//		for (iLine = 0; iLine < pLineArray->n; iLine++)
//		{
//			pLine = pLineArray->Element + iLine;
//
//			if (pLine->length > maxLineLength)
//			{
//				maxLineLength = pLine->length;
//
//				iMaxLine = iLine;
//			}
//		}
//
//		RECOG::Line3D *pRefLine;
//
//		pRefLine = pLineArray->Element + iMaxLine;
//
//		// Create feature reference frame.
//
//		P1 = pRefLine->P[0];
//		P2 = pRefLine->P[1];
//
//		float R[9];
//		//float t[3];
//
//		float *X = R;
//		float *Y = R + 3;
//		float *Z = R + 6;
//
//		RVLDIF3VECTORS(P1, P2, X);
//		RVLSCALE3VECTOR2(X, pRefLine->length, X);
//
//		RVLCOPY3VECTOR(N, Z);
//
//		RVLCROSSPRODUCT3(Z, X, Y);
//
//		//// Save lines to a file.
//
//		//char lineFileName[] = "C:\\RVL\\Debug\\Lines0.txt";
//
//		//sprintf(RVLGETFILEEXTENSION(lineFileName) - 2, "%d.txt", iLineArray);
//
//		//FILE *fp = fopen(lineFileName, "w");
//
//		//float P_[3];
//		//float VTmp[3];
//
//		//for (iLine = 0; iLine < pLineArray->n; iLine++)
//		//{
//		//	pLine = pLineArray->Element + iLine;
//
//		//	for (i = 0; i < 2; i++)
//		//	{
//		//		RVLDIF3VECTORS(pLine->P[i], pRefSurfel->P, VTmp);
//		//		RVLMULMX3X3VECT(R, VTmp, P_);
//
//		//		fprintf(fp, "%lf\t%lf\t", P_[0], P_[1]);
//		//	}
//
//		//	fprintf(fp, "\n");
//		//}
//
//		//fclose(fp);
//
//	}	// for each reference plane.
//#endif

//VIDOVIC - zakomentirano
/*
#ifdef RVLRFRECOGNITION_FEATURE_BASE_VISUALIZATION
	// Visualization

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	pSurfels->NodeColors(SelectionColor);

	visualizer.Create();
	pSurfels->InitDisplay(&visualizer, pMesh, pSurfelDetector);
	pSurfels->Display(&visualizer, pMesh);
	//pSurfelDetector->DisplaySoftEdges(&visualizer, pMesh, pSurfels, SelectionColor);
	visualizer.Run();
#endif
*/
//END VIDOVIC

	// Free memory.

	delete[] sBuff.Element;
}

void RFRecognition::DetectFeatures(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	RECOG::RFFeatureBase *pFeatureBase,
	int iRefLineArray,
	int iRefLine,
	QList<RECOG::RFFeature> *pFeatureList,
	CRVLMem *pMem_)
{
	float R[9];

	float *X = R;
	float *Y = R + 3;
	float *Z = R + 6;

	int iRefSurfel_ = pFeatureBase->iSurfel;

	Surfel *pRefSurfel = pSurfels->NodeArray.Element + iRefSurfel_;

	float *N = pRefSurfel->N;

	RVLCOPY3VECTOR(N, Z);

	Array<RECOG::Line3D> *pLineArray = pFeatureBase->lineArray + iRefLineArray;

	RECOG::Line3D *pRefLine = pLineArray->Element + iRefLine;

	Surfel *pRefSurfel2 = pSurfels->NodeArray.Element + pRefLine->iSurfel;

	float cq = RVLDOTPRODUCT3(N, pRefSurfel2->N);

	float *P1 = pRefLine->P[0];

	float dP[3];

	RVLDIF3VECTORS(pRefLine->P[1], P1, dP);
	RVLSCALE3VECTOR2(dP, pRefLine->length, X);

	RVLCROSSPRODUCT3(Z, X, Y);

	float P0[3];

	RVLSCALE3VECTOR(Y, featureDetectionParams.dl, P0);
	RVLSUM3VECTORS(P1, P0, P0);

	Array<RECOG::RFLinePoint> closeLinePointArray;

	closeLinePointArray.Element = new RECOG::RFLinePoint[2 * pLineArray->n];
	closeLinePointArray.n = 0;

#ifdef RVLRFRECOGNITION_DEBUG
	char lineFileName[] = "C:\\RVL\\Debug\\Lines0.txt";

	sprintf(RVLGETFILEEXTENSION(lineFileName) - 2, "%d.txt", iRefLineArray);

	FILE *fp = fopen(lineFileName, "a");
#endif

	int iLine;
	float *P2;
	float P1_[3], P2_[3], VTmp[3];
	RECOG::Line3D *pLine;
	Surfel *pSurfel;
	float cnx;
	RECOG::RFLinePoint *pLinePt;
	RECOG::RFFeature *pFeature;

	for (iLine = 0; iLine < pLineArray->n; iLine++)
	{
		pLine = pLineArray->Element + iLine;

		if (pLine->iSurfel == pRefLine->iSurfel)
			continue;

#ifdef RVLRFRECOGNITION_DEBUG
		if (pLine->iSurfel == 16)
			int debug = 0;
#endif

		P1 = pLine->P[0];

		RVLDIF3VECTORS(P1, P0, VTmp);
		RVLMULMX3X3VECT(R, VTmp, P1_);

		P2 = pLine->P[1];

		RVLDIF3VECTORS(P2, P0, VTmp);
		RVLMULMX3X3VECT(R, VTmp, P2_);

		if (P1_[1] * P2_[1] < 0.0f)
		{		
			pSurfel = pSurfels->NodeArray.Element + pLine->iSurfel;

			cnx = RVLDOTPRODUCT3(pSurfel->N, X);

			if (RVLABS(cnx) >= mincnx)
			{
				pFeature = RECOG::CreateFeature(pSurfel->N, pSurfel->d, R, P0, featureDetectionParams.dl, cq, pFeatureList, pMem_);				

#ifdef RVLRFRECOGNITION_DEBUG
				RECOG::DebugWriteFeature(fp, pFeature, 20.0f);
#endif
			}				
		}
		else
		{
			if (RVLABS(P1_[1]) <= maxGap)
			{
				pLinePt = closeLinePointArray.Element + closeLinePointArray.n;

				pLinePt->iLine = iLine;
				pLinePt->iPt = 0;

				closeLinePointArray.n++;
			}

			if (RVLABS(P2_[1]) <= maxGap)
			{
				pLinePt = closeLinePointArray.Element + closeLinePointArray.n;

				pLinePt->iLine = iLine;
				pLinePt->iPt = 1;

				closeLinePointArray.n++;
			}
		}	// if(!(P1_[1] * P2_[1] < 0.0f))
	}	// for each line

	float maxGap2 = maxGap * maxGap;

	int iLinePoint, iLinePoint_;
	RECOG::RFLinePoint *pLinePt_;
	RECOG::Line3D *pLine_;
	float Pm[3], Nm[3];
	float dm;
	Surfel *pSurfel_;
	float fTmp;

	for (iLinePoint = 0; iLinePoint < closeLinePointArray.n - 1; iLinePoint++)
	{
		pLinePt = closeLinePointArray.Element + iLinePoint;

		pLine = pLineArray->Element + pLinePt->iLine;

		P1 = pLine->P[pLinePt->iPt];

		for (iLinePoint_ = iLinePoint + 1; iLinePoint_ < closeLinePointArray.n; iLinePoint_++)
		{
			pLinePt_ = closeLinePointArray.Element + iLinePoint_;

			pLine_ = pLineArray->Element + pLinePt_->iLine;

			P2 = pLine_->P[pLinePt_->iPt];

			RVLDIF3VECTORS(P2, P1, dP);

			if (RVLDOTPRODUCT3(dP, dP) <= maxGap2)
			{
				pSurfel = pSurfels->NodeArray.Element + pLine->iSurfel;
				pSurfel_ = pSurfels->NodeArray.Element + pLine_->iSurfel;
				RVLSUM3VECTORS(pSurfel->N, pSurfel_->N, Nm);
				RVLNORM3(Nm, fTmp);

				cnx = RVLDOTPRODUCT3(Nm, X);

				if (cnx >= mincnx || -cnx >= mincnx)
				{
					RVLSUM3VECTORS(P1, P2, Pm);
					RVLSCALE3VECTOR(Pm, 0.5f, Pm);
					dm = RVLDOTPRODUCT3(Nm, Pm);
					pFeature = RECOG::CreateFeature(Nm, dm, R, P0, featureDetectionParams.dl, cq, pFeatureList, pMem_);

#ifdef RVLRFRECOGNITION_DEBUG
					RECOG::DebugWriteFeature(fp, pFeature, 20.0f);
#endif
				}
			}
		}
	}

#ifdef RVLRFRECOGNITION_DEBUG
	fclose(fp);
#endif

	delete[] closeLinePointArray.Element;
}

void RFRecognition::CreateDescriptor(
	Mesh *pMesh,
	SurfelGraph *pSurfels, 
	RECOG::RFFeature *pFeature,
	CRVLMem *pMem)
{
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, OrientedPoint, descriptorSize * sizeof(OrientedPoint), pFeature->descriptor.PtArray.Element);

	pFeature->descriptor.PtArray.n = descriptorSize;

	float *R = pFeature->R;
	float *t = pFeature->t;

	int nMeshPts = pMesh->NodeArray.n;

	float maxr2 = featureDetectionParams.r * featureDetectionParams.r;

	int i;
	int iPt;
	Point *pPt;
	float dP[3];
	float r2;
	float *P, *N;
	OrientedPoint *pPt_;

	for (i = 0; i < descriptorSize; i++)
	{
		do
		{
			iPt = rand() % nMeshPts; 

			pPt = pMesh->NodeArray.Element + iPt;

			RVLDIF3VECTORS(pPt->P, pFeature->t, dP);

			r2 = RVLDOTPRODUCT3(dP, dP);
		} while (r2 > maxr2);

		pPt_ = pFeature->descriptor.PtArray.Element + i;

		P = pPt_->P;
		N = pPt_->N;

		RVLMULMX3X3TVECT(R, dP, P);
		RVLMULMX3X3TVECT(R, pPt->N, N);
	}
}

void RFRecognition::FindObjects(Mesh *pMesh)
{
	pMesh->CreateOrderedMeshFromPolyData();

	// Create voxel grid.

	Voxels(pMesh);

	// Segment model mesh to surfels.

	pSurfels->Init(pMesh);

	pSurfelDetector->Init(pMesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(pMesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	// Initialize recognition.

	Init(pMesh, pSurfels);

	//VIDOVIC
#ifdef RVLRFRECOGNITION_FEATURE_BASE_VISUALIZATION
	// Visualization

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	pSurfels->NodeColors(SelectionColor);

	visualizer.Create();
	pSurfels->InitDisplay(&visualizer, pMesh, pSurfelDetector); //VIDOVIC
#ifdef RVLRFRECOGNITION_DEBUG
	RECOG::RFRecognitionCallbackData displayData;
	displayData.pRecognition = this;
	pSurfels->DisplayData.vpUserFunctionData = &displayData;
	pSurfels->DisplayData.mouseRButtonDownUserFunction = &RECOG::DetectAndWriteFeatures;
#endif
	//pSurfels->DisplayData.vpRecognition = this;//VIDOVIC
	pSurfels->Display(&visualizer, pMesh);
	//pSurfelDetector->DisplaySoftEdges(&visualizer, pMesh, pSurfels, SelectionColor);
	visualizer.Run();
#endif
	//END VIDOVIC

	QList<RECOG::Hypothesis> *pHypothesisList = &sceneInterpretation;

	RVLQLIST_INIT(pHypothesisList);

	QList<RECOG::RFFeature> featureList;
	QList<RECOG::RFFeature> *pFeatureList = &featureList;

	int nFeatures = 0;

	RECOG::RFFeature *pMFeature = modelFeatureList.pFirst;

	int matchThr_ = (int)round(matchThr * (float)descriptorSize);

	// For every sufficiently large surfel...

	int matchScore;
	RECOG::RFFeatureBase featureBase;
	int i;
	int iSurfel;
	Surfel *pSurfel;
	RECOG::Line3D *pLine;
	RECOG::RFFeature *pSFeature;
	RECOG::Hypothesis *pHypothesis;
	float *RFsS, *tFsS, *RFmM, *tFmM, *RMS, *tMS, *RFsS_, *tFsS_;
	float RMFm[9], tMFm[3];

	for (iSurfel = 0; iSurfel < nSurfels; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->size < minRefSurfelSize)
			continue;

		// Detect feature base.

		DetectFeatureBase(pMesh, pSurfels, iSurfel, &featureBase);

		// For each sufficiently large surfel participating in the feature base...

		for (i = 0; i < featureBase.lineArray[0].n; i++)
		{
			pLine = featureBase.lineArray[0].Element + i;

			if (pLine->length < minRefLineSize)
				continue;
	
			// Detect features.

			RVLQLIST_INIT(pFeatureList);

			DetectFeatures(pMesh, pSurfels, &featureBase, 0, i, pFeatureList, pMem);

			// Match descriptors.

			pSFeature = pFeatureList->pFirst;

			while (pSFeature)
			{
				nFeatures++;

				MatchDescriptors(pMesh, pSFeature, pMFeature, matchScore);

				if (matchScore >= matchThr_)
				{
					RVLMEM_ALLOC_STRUCT(pMem, RECOG::Hypothesis, pHypothesis);

					RVLQLIST_ADD_ENTRY(pHypothesisList, pHypothesis);

					pHypothesis->objectID = pMFeature->objectID;
					pHypothesis->frameID = pMFeature->frameID;
					pHypothesis->probability = matchScore; //VIDOVIC

					RFsS = pSFeature->R;
					tFsS = pSFeature->t;					
					RFmM = pMFeature->R;
					tFmM = pMFeature->t;
					RMS = pHypothesis->R;
					tMS = pHypothesis->t;
					RFsS_ = pHypothesis->RF;
					tFsS_ = pHypothesis->tF;
					RVLCOPYMX3X3(RFsS, RFsS_);
					RVLCOPY3VECTOR(tFsS, tFsS_);
					RVLINVTRANSF3D(RFmM, tFmM, RMFm, tMFm);
					RVLCOMPTRANSF3D(RFsS, tFsS, RMFm, tMFm, RMS, tMS);
				}

				pSFeature = pSFeature->pNext;
			}
		}
	}

	///
}

void RFRecognition::MatchDescriptors(
	Mesh *pSMesh,
	RECOG::RFFeature *pSFeature,
	RECOG::RFFeature *pMFeature,
	int &matchScore)
{
	float *R = pSFeature->R;
	float *t = pSFeature->t;

	float ePThr2 = ePThr * ePThr;

	matchScore = 0;

	float *NS = pSFeature->N;
	float *NM = pMFeature->N;

	float eN = RVLDOTPRODUCT3(NS, NM);

	if (eN >= eNThr)
		matchScore++;

	int i, j;
	float *PMF;
	float PMS[3];
	float *NMF;
	float NMS[3];
	OrientedPoint *pMPt;
	float *NSS;

	for (i = 0; i < pMFeature->descriptor.PtArray.n; i++)
	{
		pMPt = pMFeature->descriptor.PtArray.Element + i;

		PMF = pMPt->P;

		RVLTRANSF3(PMF, R, t, PMS);

		NMF = pMPt->N;

		RVLMULMX3X3VECT(R, NMF, NMS);

		RadiusSearch(pSMesh, PMS, ePThr2);

		for (j = 0; j < voxel8Pack.n; j++)
		{
			NSS = pSMesh->NodeArray.Element[voxel8Pack.Element[j]].N;

			eN = RVLDOTPRODUCT3(NSS, NMS);

			if (eN >= eNThr)
			{
				matchScore++;

				break;
			}
		}
	}
}

void RFRecognition::Voxels(Mesh *pMesh)
{
	// Find bounding box.

	Box<float> voxelBox_;

	pMesh->BoundingBox(&voxelBox_);

	voxel.a = RECOG::AdjustVoxelBoxSide(voxelBox_.minx, voxelBox_.maxx, voxelSize, voxelBox.minx, voxelBox.maxx);
	voxel.b = RECOG::AdjustVoxelBoxSide(voxelBox_.miny, voxelBox_.maxy, voxelSize, voxelBox.miny, voxelBox.maxy);
	voxel.c = RECOG::AdjustVoxelBoxSide(voxelBox_.minz, voxelBox_.maxz, voxelSize, voxelBox.minz, voxelBox.maxz);

	// Assing mesh points to voxels.

	RVL_DELETE_ARRAY(voxelMem);

	voxelMem = new QLIST::Index [pMesh->NodeArray.n];

	int nVoxels = voxel.a * voxel.b * voxel.c;

	RVL_DELETE_ARRAY(voxel.Element);

	voxel.Element = new QList<QLIST::Index>[nVoxels];

	Array<QList<QLIST::Index>> voxelArray;
	voxelArray.Element = voxel.Element;
	voxelArray.n = nVoxels;
	
	QList<QLIST::Index> *pPtList;

	for (int i = 0; i < voxelArray.n; i++)
	{
		pPtList = voxelArray.Element + i;
		RVLQLIST_INIT(pPtList);
	}

	int *nPts = new int[nVoxels];

	memset(nPts, 0, nVoxels * sizeof(int));

	maxnPtsPerVoxel = 0;

	int iPt;
	float *P;
	QLIST::Index *pPtIdx;
	int iVoxel;
	int nPts_;
	int ix, iy, iz;

	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		P = pMesh->NodeArray.Element[iPt].P;

		GetVoxel(P, ix, iy, iz);

		pPtList = RVL3DARRAY_ELEMENT(voxel, ix, iy, iz);

		iVoxel = pPtList - voxel.Element;

		nPts_ = nPts[iVoxel] + 1;

		if (nPts_ > maxnPtsPerVoxel)
			maxnPtsPerVoxel = nPts_;

		nPts[iVoxel] = nPts_;

		pPtIdx = voxelMem + iPt;

		pPtIdx->Idx = iPt;

		RVLQLIST_ADD_ENTRY(pPtList, pPtIdx);
	}

	delete[] nPts;

	RVL_DELETE_ARRAY(voxel8Pack.Element);

	voxel8Pack.Element = new int[8 * maxnPtsPerVoxel];
}

void RFRecognition::RadiusSearch(
	Mesh *pMesh,
	float *P,
	float r2)
{
	float P_[3];

	float halfVoxelSize = 0.5f * voxelSize;

	RVLSET3VECTOR(P_, halfVoxelSize, halfVoxelSize, halfVoxelSize);
	RVLDIF3VECTORS(P, P_, P_);

	int ix, iy, iz;

	GetVoxel(P_, ix, iy, iz);

	voxel8Pack.n = 0;

	QList<QLIST::Index> *pPtList;
	int dix, diy, diz;
	int ix_, iy_, iz_;
	QLIST::Index *pPtIdx;
	float *P__;
	float dP[3];

	for (dix = 0; dix < 2; dix++)
	{
		ix_ = ix + dix;

		if (ix_ >= 0 && ix_ < voxel.a)
		{
			for (diy = 0; diy < 2; diy++)
			{
				iy_ = iy + diy;

				if (iy_ >= 0 && iy_ < voxel.b)
				{
					for (diz = 0; diz < 2; diz++)
					{
						iz_ = iz + diz;

						if (iz_ >= 0 && iz_ < voxel.c)
						{
							pPtList = RVL3DARRAY_ELEMENT(voxel, ix_, iy_, iz_);

							pPtIdx = pPtList->pFirst;

							while (pPtIdx)
							{
								P__ = pMesh->NodeArray.Element[pPtIdx->Idx].P;

								RVLDIF3VECTORS(P__, P, dP);

								if (RVLDOTPRODUCT3(dP, dP) <= r2)
									voxel8Pack.Element[voxel8Pack.n++] = pPtIdx->Idx;

								pPtIdx = pPtIdx->pNext;
							}
						}
					}
				}
			}
		}
	}
}

void RFRecognition::SaveFeature(
	FILE *fp,
	RECOG::RFFeature *pFeature)
{
	fwrite(&(featureDetectionParams.dp), sizeof(float), 1, fp);
	fwrite(&(featureDetectionParams.dl), sizeof(float), 1, fp);
	fwrite(&(pFeature->objectID), sizeof(int), 1, fp);
	fwrite(&(pFeature->frameID), sizeof(int), 1, fp);
	fwrite(pFeature->R, sizeof(float), 9, fp);
	fwrite(pFeature->t, sizeof(float), 3, fp);
	fwrite(&(pFeature->cq), sizeof(float), 1, fp);
	fwrite(pFeature->N, sizeof(float), 3, fp);
	fwrite(&(pFeature->descriptor.PtArray.n), sizeof(int), 1, fp);
	fwrite(pFeature->descriptor.PtArray.Element, sizeof(OrientedPoint), pFeature->descriptor.PtArray.n, fp);
}

void RFRecognition::LoadFeature(
	FILE *fp,
	RECOG::RFFeature *pFeature,
	CRVLMem *pMem_)
{
	fread(&(featureDetectionParams.dp), sizeof(float), 1, fp);
	fread(&(featureDetectionParams.dl), sizeof(float), 1, fp);
	fread(&(pFeature->objectID), sizeof(int), 1, fp);
	fread(&(pFeature->frameID), sizeof(int), 1, fp);
	fread(pFeature->R, sizeof(float), 9, fp);
	fread(pFeature->t, sizeof(float), 3, fp);
	fread(&(pFeature->cq), sizeof(float), 1, fp);
	fread(pFeature->N, sizeof(float), 3, fp);
	fread(&(pFeature->descriptor.PtArray.n), sizeof(int), 1, fp);
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem_, OrientedPoint, pFeature->descriptor.PtArray.n * sizeof(OrientedPoint), pFeature->descriptor.PtArray.Element);
	fread(pFeature->descriptor.PtArray.Element, sizeof(OrientedPoint), pFeature->descriptor.PtArray.n, fp);
}

//VIDOVIC
void RFRecognition::FindBestHypothesis(RECOG::Hypothesis **pBestHypothesis)
{
	RECOG::Hypothesis *pHypothesis = sceneInterpretation.pFirst;

	*pBestHypothesis = pHypothesis;
	float highestProbability = pHypothesis->probability;

	pHypothesis = pHypothesis->pNext;

	while (pHypothesis)
	{
		if (pHypothesis->probability > highestProbability)
		{
			*pBestHypothesis = pHypothesis;
			highestProbability = pHypothesis->probability;
		}

		pHypothesis = pHypothesis->pNext;
	}

}

void RFRecognition::LoadSceneModels(char *sceneSequenceFileName)
{
	FILE *fp = fopen(sceneSequenceFileName, "r");

	char line[100];
	int lineCnt = 0;
	int sequenceFileNameLength;

	while (TRUE)
	{
		fgets(line, 100, fp);

		//int linelen = strlen(line);

		if (line[0] == '\n')
			continue;

		if (strstr(line, "end") == line)
			break;

		lineCnt++;

		sceneModelNames.resize(lineCnt, std::vector<char>(0));

		sceneModelNames[lineCnt - 1].assign(line, line + strlen(line) - 1);

		sequenceFileNameLength = strlen(strrchr(sceneSequenceFileName, '\\')) - 1;

		sceneModelNames[lineCnt - 1].insert(sceneModelNames[lineCnt - 1].begin(), sceneSequenceFileName, sceneSequenceFileName + strlen(sceneSequenceFileName) - sequenceFileNameLength);
		sceneModelNames[lineCnt - 1].insert(sceneModelNames[lineCnt - 1].end(), 1, '\0');
	}

	fclose(fp);
}

void RFRecognition::GetSceneModelFileName(int modelIndex, char *fileName)
{
	char *pSceneFileName;
	int fileNameLength;

	pSceneFileName = sceneModelNames[modelIndex].data();

	fileNameLength = sceneModelNames[modelIndex].size();

	memcpy(fileName, pSceneFileName, fileNameLength);

	//DELETE ROW
	//sceneModelNames.erase(recognition.sceneModelNames.begin() + modelIndex);

}

//Moved to Util.cpp
/*
bool RFRecognition::GetAngleAxis(float *R, float *V, float &theta)
{
	float k = 0.5 * (R[0 * 3 + 0] + R[1 * 3 + 1] + R[2 * 3 + 2] - 1.0);

	if (k > 1.0)
	{
		theta = 0.0;

		return FALSE;
	}
	else if (k < -1.0)
	{
		theta = PI;

		return FALSE;
	}

	theta = acos(k);

	k = 0.5 / sin(theta);

	V[0] = k * (R[2 * 3 + 1] - R[1 * 3 + 2]);
	V[1] = k * (R[0 * 3 + 2] - R[2 * 3 + 0]);
	V[2] = k * (R[1 * 3 + 0] - R[0 * 3 + 1]);

	return TRUE;
}

void RFRecognition::GetDistance(float *t, float &distance)
{
	distance = sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
}
*/

void RFRecognition::CoordinateSystemNoiseStabilityTest(char *sceneMeshFileName, MeshNoiser noiser, bool debug)
{
	int i;
	RECOG::Hypothesis *pBestHypothesis = NULL;
	FILE *fpHypothesisError = fopen("C:\\RVL\\Debug\\hypothesisError.txt", "w");
	float V[3], theta;
	float distance;
	Mesh mesh;
	//PCLMeshBuilder meshBuilder; //KOMENTIRANO DA SE NE BI MORAO INCLUDATI PCL

	for (i = 0; i < 100; i++)
	{
		//DEBUG
		printf("ITER: %d\n", i);

		mesh.Clear();
		mesh.pPolygonData = NULL;
		mesh.LoadPolyDataFromPLY(sceneMeshFileName);

		//add white noise to scene object
		noiser.AddWhiteNoise(&mesh);

		//save noised object
		//mesh.SaveNoisedPolyDataToPLY(sceneMeshFileName, &mesh);

		FindObjects(&mesh);

		if (debug)
		{
			char errorFileName[50];
			char meshFileName[50];

			FILE *fpHypothesisErrorDebug = NULL;
			RECOG::Hypothesis *pHypothesis = NULL;
			//pcl::PolygonMesh polygonMesh; //KOMENTIRANO DA SE NE BI MORAO INCLUDATI PCL

			//find best hypothesis
			FindBestHypothesis(&pBestHypothesis);

			GetAngleAxis(pBestHypothesis->R, V, theta);
			GetDistance(pBestHypothesis->t, distance);

			RECOG::WriteHypothesisError(fpHypothesisError, pBestHypothesis, distance, theta * 180 / PI);

			if (distance >= 10 || theta >= 0.1)
			{
				sprintf(errorFileName, "C:\\RVL\\Debug\\hypothesisError_%d.txt", i);
				sprintf(meshFileName, "C:\\RVL\\Debug\\all_noised_%d.ply", i);

				fpHypothesisErrorDebug = fopen(errorFileName, "w");

				pHypothesis = sceneInterpretation.pFirst;

				while (pHypothesis)
				{
					GetAngleAxis(pHypothesis->R, V, theta);
					GetDistance(pHypothesis->t, distance);

					RECOG::WriteHypothesisError(fpHypothesisErrorDebug, pHypothesis, distance, theta * 180 / PI);

					pHypothesis = pHypothesis->pNext;
				}


				fclose(fpHypothesisErrorDebug);

				//meshBuilder.CreateMesh(mesh.pPolygonData, polygonMesh); //KOMENTIRANO DA SE NE BI MORAO INCLUDATI PCL
				//PCLSavePLY(meshFileName, polygonMesh); //KOMENTIRANO DA SE NE BI MORAO INCLUDATI PCL

				//mesh.pPolygonData->Delete();
			}
		}
		else
		{
			//find best hypothesis
			FindBestHypothesis(&pBestHypothesis);

			GetAngleAxis(pBestHypothesis->R, V, theta);
			GetDistance(pBestHypothesis->t, distance);

			RECOG::WriteHypothesisError(fpHypothesisError, pBestHypothesis, distance, theta * 180 / PI);
		}
	}

	fclose(fpHypothesisError);
}

void RFRecognition::FeatureTestSceneSequence(char *sceneSequenceFileName, MeshNoiser noiser)
{
	int iSceneModel, nSceneModels;

	RECOG::Hypothesis *pHypothesis = NULL;
	RECOG::Hypothesis *pBestHypothesis = NULL;
	FILE *fpSceneModelsHypothesis = NULL;

	float V[3], theta;
	float distance;

	Mesh mesh;
	char *sceneMeshFileName = new char[100];

	FileSequenceLoader fileLoader;

	fileLoader.Init(sceneSequenceFileName);

	fpSceneModelsHypothesis = fopen("C:\\RVL\\Debug\\sceneModelsHypothesisError.txt", "w");

	nSceneModels = fileLoader.nFileNames;

	for (iSceneModel = 0; iSceneModel < nSceneModels; iSceneModel++)
	{
		mesh.Clear();
		mesh.pPolygonData = NULL;

		fileLoader.GetFilePath(iSceneModel, sceneMeshFileName);

		printf("*****************************************************************\n");
		printf("Scene Model %d: %s\n", iSceneModel, strrchr(sceneMeshFileName, '\\') + 1);
		printf("*****************************************************************\n");

		mesh.LoadPolyDataFromPLY(sceneMeshFileName);

		//add white noise to scene object
		noiser.AddWhiteNoise(&mesh);

		FindObjects(&mesh);

		//find best hypothesis
		FindBestHypothesis(&pBestHypothesis);

		fprintf(fpSceneModelsHypothesis, "%s %d: %s\n", "Scene Model", iSceneModel, strrchr(sceneMeshFileName, '\\') + 1);

		if (pBestHypothesis)
		{
			GetAngleAxis(pBestHypothesis->R, V, theta);
			GetDistance(pBestHypothesis->t, distance);

			fprintf(fpSceneModelsHypothesis, "%s\t%s\t%s\n", "dist_ERR", "theta_ERR", "score");
			fprintf(fpSceneModelsHypothesis, "%s\n", "Best hypothesis:");
			fprintf(fpSceneModelsHypothesis, "%f\t%f\t%f\n", distance, theta * 180 / PI, pBestHypothesis->probability);
			fprintf(fpSceneModelsHypothesis, "%s\n", "All hypothesis:");

			pHypothesis = sceneInterpretation.pFirst;

			while (pHypothesis)
			{
				GetAngleAxis(pHypothesis->R, V, theta);
				GetDistance(pHypothesis->t, distance);

				RECOG::WriteHypothesisError(fpSceneModelsHypothesis, pHypothesis, distance, theta * 180 / PI);

				pHypothesis = pHypothesis->pNext;
			}
		}
		else{
			fprintf(fpSceneModelsHypothesis, "%s\n", "NO Hypothesis!!");
		}

		fprintf(fpSceneModelsHypothesis, "\n");
	}

	fclose(fpSceneModelsHypothesis);
}

void RFRecognition::FeatureTestPrecisionRecallGraph(char *sceneMeshFileName, char *sceneSequenceFileName, MeshNoiser noiser)
{

	RECOG::Hypothesis *pBestHypothesis = NULL;
	FILE *fpTrue = NULL;
	FILE *fpFalse = NULL;
	FILE *fpBestHypothesisError = NULL;

	Mesh mesh;

	float V[3], theta;
	float distance;

	int iRefModelIter, refModelIter = 110;
	int iSceneModel, nSceneModels;
	int iSceneModelIter, sceneModelIter = 5;

	FileSequenceLoader fileLoader;

	fileLoader.Init(sceneSequenceFileName);

	fpTrue = fopen("C:\\RVL\\Debug\\True.txt", "w");
	fpBestHypothesisError = fopen("C:\\RVL\\Debug\\bestHypothesisError.txt", "w");

	printf("*****************************************************************\n");
	printf("Ref Model: %s\n", strrchr(sceneMeshFileName, '\\') + 1);
	printf("*****************************************************************\n");

	for (iRefModelIter = 0; iRefModelIter < refModelIter; iRefModelIter++)
	{
		printf("Iteration: %d\n", iRefModelIter + 1);

		mesh.Clear();
		mesh.pPolygonData = NULL;

		mesh.LoadPolyDataFromPLY(sceneMeshFileName);

		//add white noise to scene object
		noiser.AddWhiteNoise(&mesh);

		FindObjects(&mesh);

		//find best hypothesis
		FindBestHypothesis(&pBestHypothesis);

		if (pBestHypothesis)
		{
			fprintf(fpTrue, "%f\n", pBestHypothesis->probability);

			GetAngleAxis(pBestHypothesis->R, V, theta);
			GetDistance(pBestHypothesis->t, distance);

			RECOG::WriteHypothesisError(fpBestHypothesisError, pBestHypothesis, distance, theta * 180 / PI);
		}
		else
			fprintf(fpTrue, "%s\n", "NO HYPOTHESIS");
	}

	fclose(fpTrue);
	fclose(fpBestHypothesisError);

	fpFalse = fopen("C:\\RVL\\Debug\\False.txt", "w");

	nSceneModels = fileLoader.nFileNames;

	for (iSceneModel = 0; iSceneModel < nSceneModels; iSceneModel++)
	{
		fileLoader.GetFilePath(iSceneModel, sceneMeshFileName);

		//without Reference model
		if (!strcmp(strrchr(sceneMeshFileName, '\\') + 1, "all.ply"))
			continue;

		printf("*****************************************************************\n");
		printf("Scene Model %d: %s\n", iSceneModel, strrchr(sceneMeshFileName, '\\') + 1);
		printf("*****************************************************************\n");

		for (iSceneModelIter = 0; iSceneModelIter < sceneModelIter; iSceneModelIter++)
		{
			printf("Iteration: %d\n", iSceneModelIter + 1);

			mesh.Clear();
			mesh.pPolygonData = NULL;

			mesh.LoadPolyDataFromPLY(sceneMeshFileName);

			//add white noise to scene object
			noiser.AddWhiteNoise(&mesh);

			FindObjects(&mesh);

			//find best hypothesis
			FindBestHypothesis(&pBestHypothesis);

			if (pBestHypothesis)
				fprintf(fpFalse, "%f\n", pBestHypothesis->probability);
			else
				fprintf(fpFalse, "%s\n", "NO HYPOTHESIS");
		}
	}

	fclose(fpFalse);
}
//END VIDOVIC

bool RECOG::SurfelCylinderIntersection(
	Mesh *pMesh,
	SurfelGraph *pSurfels, 
	int iSurfel,
	float *N,
	float d0,
	float dp,
	float r,
	Array<RECOG::Line3D> *lineArray,
	Array<float> *psBuff)
{
#ifdef RVLRFRECOGNITION_DEBUG
	if (iSurfel == 3)
		int debug = 0;
#endif

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	float *N_ = pSurfel->N;

	float d_ = pSurfel->d;

	// V <- vector perpendicular to N and N_.

	float V[3];

	RVLCROSSPRODUCT3(N, N_, V);

	float fTmp;

	RVLNORM3(V, fTmp);

	// Compute intersection lines.

	int ns[2];

	ns[0] = ns[1] = 0;

	bool bIn = false;

	int volumeID1, volumeID2, k_;
	int iBoundary, i, j, k;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr;
	float *P1, *P2;
	int iLineArray;
	float d;
	float PIS[3];
	float s;
	float *sBuff_;

	for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
	{
		pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

		pEdgePtr = pBoundary->Element[pBoundary->n - 1];

		P1 = pMesh->NodeArray.Element[RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr)].P;

		volumeID1 = IdentifyVolume(P1, N, d0, dp);

		for (i = 0; i < pBoundary->n; i++)
		{
			pEdgePtr = pBoundary->Element[i];

			P2 = pMesh->NodeArray.Element[RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr)].P;

			volumeID2 = IdentifyVolume(P2, N, d0, dp);

			if (volumeID2 == 0)
				bIn = true;

			k_ = volumeID1 + volumeID2;

			if (k_ == 1 || k_ == -1)
			{
				iLineArray = ((k_ + 1) >> 1);

				d = d0 + (float)k_ * dp;

				LinePlaneIntersection<float>(P1, P2, N, d, PIS);

				s = RVLDOTPRODUCT3(V, PIS);

				sBuff_ = psBuff->Element + iLineArray * psBuff->n;

				for (j = 0; j < ns[iLineArray]; j++)
					if (s < sBuff_[j])
						break;

				for (k = ns[iLineArray] - 1; k >= j; k--)
					sBuff_[k + 1] = sBuff_[k];

				sBuff_[j] = s;

				ns[iLineArray]++;
			}

			P1 = P2;
			volumeID1 = volumeID2;
		}
	}

	/// Update lineArray.

	float P0[3];
	Eigen::Matrix3f M;
	Eigen::Vector3f B, P0_;
	RECOG::Line3D *pLine;
	float dP[3];
	float s1, s2;

	for (iLineArray = 0; iLineArray < 2; iLineArray++)
	{
#ifdef RVLRFRECOGNITION_DEBUG
		char lineFileName[] = "C:\\RVL\\Debug\\Lines0.txt";

		sprintf(RVLGETFILEEXTENSION(lineFileName) - 2, "%d.txt", iLineArray);

		FILE *fp = fopen(lineFileName, "a");
#endif
		d = d0 + (float)(2 * iLineArray - 1) * dp;

		// P0 <- intersection of planes (N, d), (N_, d_) and (V, 0)

		M << N[0], N[1], N[2], N_[0], N_[1], N_[2], V[0], V[1], V[2];
		B << d, d_, 0.0f;

		P0_ = M.colPivHouseholderQr().solve(B);

		P0[0] = P0_[0];
		P0[1] = P0_[1];
		P0[2] = P0_[2];

		// Compute lines and add them to lineArray[iLineArray].

		sBuff_ = psBuff->Element + iLineArray * psBuff->n;

		pLine = lineArray[iLineArray].Element + lineArray[iLineArray].n;

		for (i = 0; i < ns[iLineArray]; i += 2, pLine++)
		{
			s1 = sBuff_[i];

			RVLSCALE3VECTOR(V, s1, dP);

			P1 = pLine->P[0];

			RVLSUM3VECTORS(P0, dP, P1);

			s2 = sBuff_[i + 1];

			RVLSCALE3VECTOR(V, s2, dP);

			P2 = pLine->P[1];

			RVLSUM3VECTORS(P0, dP, P2);			

			pLine->length = s2 - s1;

			pLine->iSurfel = iSurfel;

#ifdef RVLRFRECOGNITION_DEBUG
			fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t0\n", P1[0], P1[1], P1[2], P2[0], P2[1], P2[2]);

			fflush(fp);
#endif
		}

#ifdef RVLRFRECOGNITION_DEBUG
		fclose(fp);
#endif

		lineArray[iLineArray].n += (ns[iLineArray] / 2);		
	}

	return bIn;
}

int RECOG::RegionGrowingOperation(
	int iNode,
	int iNode_,
	SURFEL::Edge *pEdge,
	SurfelGraph *pSurfels,
	RECOG::RFRegionGrowingData *pData
	)
{
	if (pData->markMap[iNode])
		return 0;

	pData->markMap[iNode] = 1;

	*(pData->piVisited++) = iNode;

	float *N = pData->N;
	float *N_ = pData->pSurfels->NodeArray.Element[iNode].N;

	if (RVLDOTPRODUCT3(N, N_) < -0.5f)
		return 0;

	return (RECOG::SurfelCylinderIntersection(pData->pMesh, pData->pSurfels, iNode, pData->N, pData->d0, pData->dp, 0.0f, pData->lineArray, pData->psBuff) ? 1 : 0);
}

int RECOG::AdjustVoxelBoxSide(
	float minSrc,
	float maxSrc,
	float voxelSize,	
	float &minTgt,
	float &maxTgt)
{
	float a_ = maxSrc - minSrc;
	float a = floor(a_ / voxelSize) + 1.0f;
	float offset = 0.5f * (a * voxelSize - a_);
	minTgt = minSrc - offset;
	maxTgt = maxSrc + offset;
	
	return (int)a;
}

void RECOG::WriteHypothesis(
	FILE *fp,
	RECOG::Hypothesis *pHypothesis)
{
	fprintf(fp, "ObjectID=%d\n", pHypothesis->objectID);
	fprintf(fp, "FrameID=%d\n", pHypothesis->frameID);
	fprintf(fp, "Rot:\n");
	PrintMatrix<float>(fp, pHypothesis->R, 3, 3);
	fprintf(fp, "t: ");
	PrintMatrix<float>(fp, pHypothesis->t, 1, 3);
	fprintf(fp, "\n");
}

void RFRecognition::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh)
{
	pVisualizer->SetMesh(pMesh);
	
	displayData.pMesh = pMesh;
	displayData.pSurfels = pSurfels;
	displayData.pRecognition = this;
	displayData.pVisualizer = pVisualizer;

	pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);

	//pVisualizer->SetMouseRButtonDownCallback(SURFEL::MouseRButtonDown, &DisplayData);
	//pVisualizer->SetKeyPressCallback(SURFEL::KeyPressCallback, &DisplayData);
}

void RFRecognition::Display()
{	
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	pSurfels->Display(pVisualizer, pMesh);

	//double t[3];

	RECOG::Hypothesis *pHypothesis = sceneInterpretation.pFirst;

	while (pHypothesis)
	{
		vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

		axes->SetTotalLength(10.0, 10.0, 10.0);

		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		double T[16];
		RVLCREATE3DTRANSF(pHypothesis->RF, pHypothesis->tF, T);

		transform->SetMatrix(T);

		axes->SetUserTransform(transform);

		vtkMatrix4x4 *T_ = axes->GetMatrix();

		pVisualizer->renderer->AddActor(axes);

		pHypothesis = pHypothesis->pNext;
	}
}
	

//VIDOVIC
void RECOG::WriteHypothesisError(
	FILE *fp,
	RECOG::Hypothesis *pHypothesis,
	float positionError,
	float angleError)
{
	fprintf(fp, "ObjectID=%d\n", pHypothesis->objectID);
	fprintf(fp, "FrameID=%d\n", pHypothesis->frameID);
	fprintf(fp, "distance\ttheta\n");
	fprintf(fp, "%f\t%f\n", positionError, angleError);
}
//END VIDOVIC

#ifdef RVLRFRECOGNITION_DEBUG
void RECOG::DebugWriteFeature(
	FILE *fp,
	RECOG::RFFeature *pFeature,
	float axisLength)
{
	float *X = pFeature->R;
	float *Y = pFeature->R + 3;
	float *Z = pFeature->R + 6;

	float *P1 = pFeature->t;

	float P2[3], V[3];

	RVLSCALE3VECTOR(X, axisLength, V);
	RVLSUM3VECTORS(P1, V, P2);

	fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t2\n", P1[0], P1[1], P1[2], P2[0], P2[1], P2[2]);

	RVLSCALE3VECTOR(Y, axisLength, V);
	RVLSUM3VECTORS(P1, V, P2);

	fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t2\n", P1[0], P1[1], P1[2], P2[0], P2[1], P2[2]);

	RVLSCALE3VECTOR(Z, axisLength, V);
	RVLSUM3VECTORS(P1, V, P2);

	fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t2\n", P1[0], P1[1], P1[2], P2[0], P2[1], P2[2]);
}

void RECOG::DebugWriteDescriptor(
	FILE *fp,
	RECOG::RFFeature *pFeature,
	float normalLength)
{
	float P1[3], P2[3];

	int i;
	OrientedPoint *pPt;
	float N[3];
	
	for (i = 0; i < pFeature->descriptor.PtArray.n; i++)
	{
		pPt = pFeature->descriptor.PtArray.Element + i;

		RVLTRANSF3(pPt->P, pFeature->R, pFeature->t, P1);

		RVLMULMX3X3TVECT(pFeature->R, pPt->N, N);
		RVLSCALE3VECTOR(N, normalLength, N);
		RVLSUM3VECTORS(P1, N, P2);

		fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t1\n", P1[0], P1[1], P1[2], P2[0], P2[1], P2[2]);
	}
}

bool RECOG::DetectAndWriteFeatures(
	Mesh *pMesh, 
	SurfelGraph *pSurfels, 
	int iSelectedPt, 
	int iSelectedSurfel, 
	void *vpData)
{
	RECOG::RFRecognitionCallbackData *pData = (RECOG::RFRecognitionCallbackData *)vpData;

	RECOG::RFFeatureBase featureBase;
	QList<RECOG::RFFeature> featureList;
	QList<RECOG::RFFeature> *pFeatureList = &featureList;
	CRVLMem Mem;
	Mem.Create(100000);
	RECOG::Line3D *pLine;
	int i;

	pData->pRecognition->DetectFeatureBase(pMesh, pSurfels, iSelectedSurfel, &featureBase);

	for (i = 0; i < featureBase.lineArray[0].n; i++)
	{
		pLine = featureBase.lineArray[0].Element + i;

		if (pLine->length < pData->pRecognition->minRefLineSize)
			continue;

		// Detect features.

		RVLQLIST_INIT(pFeatureList);

		pData->pRecognition->DetectFeatures(pMesh, pSurfels, &featureBase, 0, i, pFeatureList, &Mem);
	}

	return false;
}
#endif