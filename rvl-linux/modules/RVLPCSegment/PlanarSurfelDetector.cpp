//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Graph.h"
//#include <Eigen\Eigenvalues>
//#include <pcl/common/common.h>
//#include <pcl/PolygonMesh.h>
//#include "PCLTools.h"
//#include "PCLMeshBuilder.h"
//#include "RGBDCamera.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"

#define RVLPLANARSURFELDETECTOR_OCCLUSION_EDGES
#ifndef RVLVERSION_171125
#define RVLPLANARSURFELDETECTOR_JOIN_SMALL_SURFELS_ONLY_TO_LARGE_NEIGHBORS		// 170601: OFF
#endif

using namespace RVL;

PlanarSurfelDetector::PlanarSurfelDetector()
{
	
	k = 24.0f;
	kRGB = 0.0f;			// ECCV dataset
	//kNormal = 8.0f;		// ECCV dataset
	//kPlane = 400.0f;		// ECCV dataset
	kNormal = 8.0f;
	kPlane = 400.0f;
	surfelDistThr = 2.0f;
	minSurfelSize = 20;
	minEdgeFeatureSize = 5;
	maxEdgeFeatureConcavity = 0.010f;
	maxRange = 5000.0f;
	maxAttackSize = 1000;
	maxUnconstrainedNormalDepth = 7;
	bJoinSmallSurfelsToClosestNeighbors = false;
	bNormalConstraintInSecondInitRG = false;
	bLimitedDepthUnconstrainedNormalRG = false;
	bBottom = false;
	bEdges = true;
	bGnd = false;
	bEdgeClassForegroundIsDefault = false;
	edgeClassHalfWinSize = 5;
	edgeClassDepthDiscontinuityThr = 0.01f;
	edgeClassImageBoundary.minx = 5;
	edgeClassImageBoundary.maxx = 634;
	edgeClassImageBoundary.miny = 5;
	edgeClassImageBoundary.maxy = 474;
	zGndThr = 0.005f;

	pMem = NULL;
	//iPtBuff = NULL;
	map = NULL;
	distanceMap = NULL;
	unconstrainedNormalDepthMap = NULL;
	PointEdgeBuff = NULL;
	BoundaryMem = NULL;
	cutCostMap = NULL;
	edgeFlags = NULL;
	cutPropagationBuffMem = NULL;
	mProcessed = NULL;
	processedBuff.Element = NULL;
	GSeedMem = NULL;
	neighborMem = NULL;
	GSeedListArray.Element = NULL;
	pTimer = NULL;
	edgeClassDepthOccupancy = NULL;
	iEdgeClassDepthOccupancyBin.Element = NULL;


#ifdef RVLPLANARSURFELDETECTOR_DEBUG
	iPtBuffDebug = NULL;
#endif

#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
	debugDefineBoundaryiSurfel = 82;
	debugDefineBoundaryiSurfel_ = 191;
#endif
}


PlanarSurfelDetector::~PlanarSurfelDetector()
{
#ifdef RVLPLANARSURFELDETECTOR_DEBUG
	if (iPtBuffDebug)
		delete[] iPtBuffDebug;
#endif

	DeallocateMemory();
}

void PlanarSurfelDetector::Init(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	CRVLMem *pMem_)
{
	pMem = pMem_;

	DeallocateMemory();

	int nPts = pMesh->NodeArray.n;

	int nEdges = pMesh->EdgeArray.n;

	Mem2A.Create(2 * nPts * sizeof(int) + sizeof(BYTE *) + 1);
	Mem2B.Create((nPts + nEdges) * sizeof(int) + sizeof(BYTE *) + 1);

	map = new int[nPts];

	memset(map, 0xff, nPts * sizeof(int));

	distanceMap = new unsigned int[nPts];

	memset(distanceMap, 0xff, nPts * sizeof(unsigned int));

	unconstrainedNormalDepthMap = new unsigned int[nPts];

	memset(unconstrainedNormalDepthMap, 0, nPts * sizeof(unsigned int));

	//memset(unconstrainedNormalDepthMap, 0xff, nPts * sizeof(unsigned int));

	//iPtBuff = new int[2 * nPts];

	regionGrowingData.distThr = surfelDistThr * surfelDistThr;
	regionGrowingData.kRGB2 = kRGB * kRGB;
	//regionGrowingData.kNormal2 = 0.0f;
	regionGrowingData.kNormal2 = kNormal * kNormal;
	regionGrowingData.kPlane2 = kPlane * kPlane;
	regionGrowingData.surfelMap = pSurfels->surfelMap;
	regionGrowingData.buffer = map;
	regionGrowingData.costMap = regionGrowingData.costBuffer = NULL;
	regionGrowingData.GID = nPts;
	regionGrowingData.bLimitedDepthUnconstrainedRG = false;
	regionGrowingData.unconstrainedNormalDepthMap = unconstrainedNormalDepthMap;
	regionGrowingData.maxUnconstrainedNormalDepth = maxUnconstrainedNormalDepth;

	BoundaryMem = new QLIST::Index[nPts];

	mProcessed = new unsigned char[nPts];

	memset(mProcessed, 0, nPts * sizeof(unsigned char));
		
	processedBuff.Element = new int[nPts];

	processedBuff.n = 0;

	GSeedMem = new QLIST::Index[nPts];

	GSeedListArray.n = nPts;

	GSeedListArray.Element = new QList<QLIST::Index>[GSeedListArray.n];

	QLIST::InitListArray(GSeedListArray, GSeedListArray);

	neighborMem = new QLIST::Index[nPts];

	PointEdgeBuff = new MESH::PointEdge[4 * nEdges];

	cutCostMap = new unsigned int[nEdges];

	memset(cutCostMap, 0xff, nEdges * sizeof(unsigned int));

	edgeFlags = new unsigned char[nEdges];

	memset(edgeFlags, 0, nEdges * sizeof(unsigned char));

	cutPropagationBuffMem = new QLIST::Index[2 * nEdges];

	//int nEdgeClassDepthOccupancyBins = (int)floor(maxRange / (0.5f * edgeClassDepthDiscontinuityThr)) + 1;

	//edgeClassDepthOccupancy = new PSD::Interval[nEdgeClassDepthOccupancyBins];

	//memset(edgeClassDepthOccupancy, 0, nEdgeClassDepthOccupancyBins * sizeof(PSD::Interval));

	//iEdgeClassDepthOccupancyBin.Element = new int[nEdgeClassDepthOccupancyBins];
}

void PlanarSurfelDetector::DeallocateMemory()
{
	//RVL_DELETE_ARRAY(iPtBuff);
	RVL_DELETE_ARRAY(map);
	RVL_DELETE_ARRAY(distanceMap);
	RVL_DELETE_ARRAY(unconstrainedNormalDepthMap);
	RVL_DELETE_ARRAY(PointEdgeBuff);
	RVL_DELETE_ARRAY(BoundaryMem);
	RVL_DELETE_ARRAY(cutCostMap);
	RVL_DELETE_ARRAY(edgeFlags);
	RVL_DELETE_ARRAY(cutPropagationBuffMem);
	RVL_DELETE_ARRAY(mProcessed);
	RVL_DELETE_ARRAY(processedBuff.Element);
	RVL_DELETE_ARRAY(GSeedMem);
	RVL_DELETE_ARRAY(GSeedListArray.Element);
	RVL_DELETE_ARRAY(neighborMem);
	RVL_DELETE_ARRAY(edgeClassDepthOccupancy);
	RVL_DELETE_ARRAY(iEdgeClassDepthOccupancyBin.Element);

	Mem2A.Free();
	Mem2B.Free();
}

void PlanarSurfelDetector::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	//pParamData = ParamList.AddParam("PSD.SegmentationType", RVLPARAM_TYPE_FLAG, &m_Flags);
	//ParamList.AddID(pParamData, "3D", RVLPSD_SEGMENT_3D);
	pParamData = ParamList.AddParam("SurfelDetector.kPlane", RVLPARAM_TYPE_FLOAT, &kPlane);
	pParamData = ParamList.AddParam("SurfelDetector.kNormal", RVLPARAM_TYPE_FLOAT, &kNormal);
	pParamData = ParamList.AddParam("SurfelDetector.kRGB", RVLPARAM_TYPE_FLOAT, &kRGB);
	pParamData = ParamList.AddParam("SurfelDetector.maxRange", RVLPARAM_TYPE_FLOAT, &maxRange);
	pParamData = ParamList.AddParam("SurfelDetector.minSurfelSize", RVLPARAM_TYPE_INT, &minSurfelSize);
	pParamData = ParamList.AddParam("SurfelDetector.minEdgeFeatureSize", RVLPARAM_TYPE_INT, &minEdgeFeatureSize);
	pParamData = ParamList.AddParam("SurfelDetector.maxEdgeFeatureConcavity", RVLPARAM_TYPE_FLOAT, &maxEdgeFeatureConcavity);
	pParamData = ParamList.AddParam("SurfelDetector.maxAttackSize", RVLPARAM_TYPE_INT, &maxAttackSize);
	pParamData = ParamList.AddParam("SurfelDetector.bJoinSmallSurfelsToClosestNeighbors", RVLPARAM_TYPE_BOOL, &bJoinSmallSurfelsToClosestNeighbors);
	pParamData = ParamList.AddParam("SurfelDetector.edgeClassHalfWinSize", RVLPARAM_TYPE_INT, &edgeClassHalfWinSize);
	pParamData = ParamList.AddParam("SurfelDetector.edgeClassDepthDiscontinuityThr", RVLPARAM_TYPE_FLOAT, &edgeClassDepthDiscontinuityThr);
	pParamData = ParamList.AddParam("SurfelDetector.edgeClassImageBoundary.left", RVLPARAM_TYPE_INT, &(edgeClassImageBoundary.minx));
	pParamData = ParamList.AddParam("SurfelDetector.edgeClassImageBoundary.right", RVLPARAM_TYPE_INT, &(edgeClassImageBoundary.maxx));
	pParamData = ParamList.AddParam("SurfelDetector.edgeClassImageBoundary.top", RVLPARAM_TYPE_INT, &(edgeClassImageBoundary.miny));
	pParamData = ParamList.AddParam("SurfelDetector.edgeClassImageBoundary.bottom", RVLPARAM_TYPE_INT, &(edgeClassImageBoundary.maxy));
	pParamData = ParamList.AddParam("SurfelDetector.normalConstraintInSecondInitRG", RVLPARAM_TYPE_BOOL, &bNormalConstraintInSecondInitRG);
	pParamData = ParamList.AddParam("SurfelDetector.LimitedDepthUnconstrainedNormalRG", RVLPARAM_TYPE_BOOL, &bLimitedDepthUnconstrainedNormalRG);
	pParamData = ParamList.AddParam("SurfelDetector.bottom", RVLPARAM_TYPE_BOOL, &bBottom);
	pParamData = ParamList.AddParam("SurfelDetector.edges", RVLPARAM_TYPE_BOOL, &bEdges);
	pParamData = ParamList.AddParam("SurfelDetector.edgeClassForegroundIsDefault", RVLPARAM_TYPE_BOOL, &bEdgeClassForegroundIsDefault);
	pParamData = ParamList.AddParam("SurfelDetector.maxUnconstrainedNormalDepth", RVLPARAM_TYPE_INT, &maxUnconstrainedNormalDepth);
	pParamData = ParamList.AddParam("SurfelDetector.zGndThr", RVLPARAM_TYPE_FLOAT, &zGndThr);
}

void PlanarSurfelDetector::RandomIndices(Array<int> &A)
{
#ifdef RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
#ifdef RVLLINUX
	//FILE *fp = fopen((std::string(precomputesFolder) + "/pseudorandom1000000.dat").data(), "rb");
	FILE *fp = fopen("/home/RVLuser/rvl-linux/pseudorandom1000000.dat", "rb");
#else
	FILE *fp = fopen("..\\pseudorandom1000000.dat", "rb");
#endif

	int *iRnd = new int[A.n];

	//for (int i = 0; i < 1000000; i++)
	//	iRnd[i] = (rand() % 0x100) + (rand() % 0x100) * 0x100 + (rand() % 0x100) * 0x10000 + (rand() % 0x80) * 0x1000000;

	//fwrite(iRnd, sizeof(int), 1000000, fp);

	fread(iRnd, sizeof(int), A.n, fp);

	fclose(fp);

	int *piRnd = iRnd;
#endif

	A.Element = new int[A.n];

	int iPt;

	for (iPt = 0; iPt < A.n; iPt++)
		A.Element[iPt] = iPt;

	int iPt_;
	int iTmp;

	//srand(time(NULL)); //VIDOVIC RANDOM TEST

	for (iPt = 0; iPt < A.n; iPt++)
	{
#ifdef RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
		iPt_ = (*(piRnd++)) % A.n;
#else
		iPt_ = rand() % A.n;
#endif

		iTmp = A.Element[iPt];
		A.Element[iPt] = A.Element[iPt_];
		A.Element[iPt_] = iTmp;
	}

#ifdef RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
	delete[] iRnd;
#endif
}

int PSD::RegionGrowingOperation(
	int iNode,
	int iNode_,
	MeshEdge *pEdge,
	Mesh *pMesh,
	PlanarSurfelDetectorRegionGrowingData *pData
	)
{
	//if (iNode == 170558)
	//	int debug = 0;

	if(pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER)
	{
		if (pData->iPtSeed >= 0)
			return 0;
	}

	if (pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION)
	{
#ifdef RVLPLANARSURFELDETECTOR_CONNECTED
		if (pData->surfelMap[iNode] >= 0 || pData->buffer[iNode] >= 0)
#else
		if (pData->buffer[iNode] >= 0)
#endif
			return 0;
	}
	else // if (pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK)
	{
		if (pData->buffer[iNode] >= 0)
			return 0;	// iNode already processed.

		if (pData->surfelMap[iNode] != pData->iAttackedSurfel)
			return -1;	// iNode does not belong to G.

		if (pData->size > pData->maxSize)
			return -1;	// Maximum size is exceeded.
	}

	Point *pPt_ = pMesh->NodeArray.Element + iNode;

	float eRGB, eN, eP;
		
#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
	float cost;
#endif
#endif
#endif
	if (!pMesh->bLabels || (pPt_->label == pData->pTemplate->ObjectID))
	{
	if (RVLDOTPRODUCT3(pPt_->N, pPt_->N) > 0.5f)
	{
		PSD::VertexDist<Surfel, Point>(pData->pTemplate, pPt_, eRGB, eN, eP);

		float costRGB, costN, costP;
	
		if ((costRGB = pData->kRGB2 * eRGB) <= pData->distThr)
		{
			if ((costP = pData->kPlane2 * eP) <= pData->distThr)
			{				
				bool bNormalOK = ((costN = pData->kNormal2 * eN) <= pData->distThr);

				if (pData->bLimitedDepthUnconstrainedRG)
				{
					unsigned int unconstrainedNormalDepthParent = pData->unconstrainedNormalDepthMap[iNode_];

					if (unconstrainedNormalDepthParent >= pData->maxUnconstrainedNormalDepth)
						bNormalOK = false;
					else if (unconstrainedNormalDepthParent > 0)
					{
						bNormalOK = true;

						pData->unconstrainedNormalDepthMap[iNode] = unconstrainedNormalDepthParent + 1;
					}
					else
					{
						pData->unconstrainedNormalDepthMap[iNode] = (bNormalOK ? 0 : 1);

						bNormalOK = true;
					}
				}

				if (bNormalOK)
				{
					if (pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER)
					{
						pData->iPtSeed = iNode;

						return 0;
					}
					else // if (pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION || pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK)
					{
#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
#ifdef RVLPLANARSURFELDETECTOR_DIST_COST
						RVLDIF3VECTORS(pPt_->P, pPtTemplate->P, dPt);

						cost = RVLDOTPRODUCT3(dPt, dPt);
#else
						cost = costRGB + costN + costP;
#endif
						if (pData->surfelMap[iNode] < 0 || pData->costMap[iNode] > cost)
#endif
#endif
						{
							//pData->surfelMap[iNode] = pData->iSurfel;
							pData->buffer[iNode] = pData->iSurfel;

#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
							pData->costBuffer[iNode] = cost;
#endif
#endif
#endif
							pData->size += pData->dSize;

							return 1;	// iNode belongs to G.
						}
					} // if (pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION || pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK)
				}	// if(bNormalOK)
			}	// if ((costP = pData->kPlane2 * eP) <= pData->distThr)
		}	// if ((costRGB = pData->kRGB2 * eRGB) <= pData->distThr)

		if (pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER)
		{
			pData->buffer[iNode] = pData->iSurfel;

			return 1;
		}			
	}	// if (RVLDOTPRODUCT3(pPt_->N, pPt_->N) > 0.5f)
	}	// if (!pData->bLabels || (pPt_->label == pData->pTemplate->ObjectID))

	return -1;	// iNode does not belong to G.
}

// Segments pMesh into surfels. The resulting surfels are stored as nodes in pSurfels.
// Each point of pMesh corresponds to an element of pSurfel->surfelMap.
// The element of pSurfel->surfelMap corresponding to a particular point of pMesh has the value of the index of the surfel which the point belongs to.

void PlanarSurfelDetector::Segment(
	Mesh *pMesh,
	SurfelGraph *pSurfels)
{
	int nPts = pMesh->NodeArray.n;
	printf("nPts=%d\n", nPts);

	memset(pSurfels->surfelMap, 0xff, nPts * sizeof(int));

	memset(pSurfels->edgeMap, 0xff, nPts * sizeof(int));

	pSurfels->NodeArray.n = 0;

	int nSEdges = 0;

	QList<SURFEL::Edge> SEdgeList;

	QList<SURFEL::Edge> *pSEdgeList_ = &SEdgeList;

#ifndef RVLVERSION_171125
	Array<Pair<int, int>> iOcclusionEdgePtArray;

	if ((pMesh->bOrganizedPC || bBottom) && bEdges)
	{
		// Detect boundaries.

		Boundaries(pMesh, pSurfels);

		// Detect edge features.

		EdgeFetures(pMesh, pSurfels, &SEdgeList, nSEdges);

		// Memorize occlusion edge points and set their ID in surfelMap to 0 in order to exclude them from surfel detection.

		iOcclusionEdgePtArray.Element = new Pair<int, int>[pMesh->NodeArray.n];
		iOcclusionEdgePtArray.n = 0;

		int iPointEdge;
		int iPt;
		MeshEdgePtr *pEdgePtr;

		QLIST::Entry<Array<MeshEdgePtr *>> *pBoundary = pSurfels->BoundaryList.pFirst;

		while (pBoundary)
		{
			for (iPointEdge = 0; iPointEdge < pBoundary->data.n; iPointEdge++)
			{
				pEdgePtr = pBoundary->data.Element[iPointEdge];

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				iOcclusionEdgePtArray.Element[iOcclusionEdgePtArray.n].a = iPt;
				iOcclusionEdgePtArray.Element[iOcclusionEdgePtArray.n].b = pSurfels->surfelMap[iPt];
				iOcclusionEdgePtArray.n++;
				pSurfels->surfelMap[iPt] = 0;
			}

			pBoundary = pBoundary->pNext;
		}

		//for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
		//	if (pSurfels->surfelMap[iPt] < -1)
		//	{
		//		iOcclusionEdgePtArray.Element[iOcclusionEdgePtArray.n].a = iPt;
		//		iOcclusionEdgePtArray.Element[iOcclusionEdgePtArray.n].b = pSurfels->surfelMap[iPt];
		//		iOcclusionEdgePtArray.n++;
		//		pSurfels->surfelMap[iPt] = 0;
		//	}

		// Exclude ground points from surfel detection.

		if (bGnd)
		{
			printf("Exclude ground points from surfel detection.\n");

			Point *pPt;
			float zGnd;

			for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
			{
				pPt = pMesh->NodeArray.Element + iPt;

				if (!pPt->bValid)
					continue;

				if (RVLDOTPRODUCT3(pPt->N, pPt->N) < 0.5f)
					continue;

				zGnd = RVLDOTPRODUCT3(NGnd, pPt->P) - dGnd;

				if (zGnd <= zGndThr)
					RVLNULL3VECTOR(pPt->N);
			}
		}
	}
#endif

	Point *Pt = pMesh->NodeArray.Element;		

	// Randomize vertex indices

	Array<int> RandPtIdxArray;

	RandPtIdxArray.n = nPts;

	RandomIndices(RandPtIdxArray);

	// Allocate memory and initialize arrays.

	int *regionGrowingBuffer = new int[nPts];

	regionGrowingData.buffer = regionGrowingBuffer;

	InitPlanarRegionGrowing(pMesh, pSurfels);

	int iSurfel = pSurfels->NodeArray.n;

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	int i;
	int iPtSeed;
	Point *pPt;
	QList<QLIST::Index2> *pPtList;

	for (i = 0; i < nPts; i++)
	{
		iPtSeed = RandPtIdxArray.Element[i];

		//if(i == 18947)
		//	int debug = 0;

		if (pSurfels->surfelMap[iPtSeed] >= 0)
			continue;

		pPt = pMesh->NodeArray.Element + iPtSeed;

		if (pPt->P[2] > maxRange)
			continue;

		if (!pPt->bValid)
			continue;

		if (RVLDOTPRODUCT3(pPt->N, pPt->N) < 0.5f)
			continue;

		pSurfel->bEdge = false;

		pSurfel->flags = 0x00;

		regionGrowingData.buffer = regionGrowingBuffer;

		PlanarRegionGrowing(pMesh, pSurfels, iPtSeed, iSurfel);

		regionGrowingData.buffer = map;

#ifdef RVLPLANARSURFELDETECTOR_POLYGONS
		// Define Polygon.

		if (pSurfel->size >= minSurfelSize)
			DefinePolygon(pMesh, pSurfels, iSurfel);
#endif

		// Next surfel index

		//printf("surfel %d\t%d\n", iSurfel, pSurfel->size);	// debug

		iSurfel++;

		pSurfel++;

		//if (iSurfel == 283)	// debug
		//	int debug = 0;

		//if (iSurfel == 24)
		//	break;

		//	break;

		//Surfel *pSurfelDebug = pSurfels->NodeArray.Element + 0;

		//QLIST::Index2 *pPtIdxDebug = pSurfelDebug->PtList.pFirst;

		//while (pPtIdxDebug)
		//{
		//	if (pPtIdxDebug->Idx == 171041)
		//		int debug = 0;

		//	pPtIdxDebug = pPtIdxDebug->pNext;
		//}
	}	// for every vertex

	pSurfels->NodeArray.n = iSurfel;

	//// Assign points to surfels

	//QLIST::Index *pPtIdx = pSurfels->PtMem;

	//for (iPt = 0; iPt < nPts; iPt++)
	//{
	//	iSurfel = pSurfels->surfelMap[iPt];

	//	if (iSurfel < 0)
	//		continue;

	//	pSurfel = pSurfels->NodeArray.Element + iSurfel;

	//	pPtList = &(pSurfel->PtList);

	//	RVLQLIST_ADD_ENTRY(pPtList, pPtIdx);

	//	pPtIdx->Idx = iPt;

	//	pPtIdx++;
	//}

#ifndef RVLVERSION_171125
	if (pMesh->bOrganizedPC || bBottom)
	{
		// Restore occlusion edges.

		for (i = 0; i < iOcclusionEdgePtArray.n; i++)
			pSurfels->surfelMap[iOcclusionEdgePtArray.Element[i].a] = iOcclusionEdgePtArray.Element[i].b;
	}
#endif
	
	// Initialize buffers for determining of surfel neighbors, boundaries and sizes.

	pSurfels->InitGetNeighborsBoundaryAndSize();

	// Determine surfel size. 

	pSurfel = pSurfels->NodeArray.Element;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++, pSurfel++)
		pSurfel->size = QLIST::Size(pSurfel->PtList);

	// Identify small surfels and join them to closest neighbors.

	if (bJoinSmallSurfelsToClosestNeighbors)
		JoinSmallSurfelsToClosestNeighbors(pMesh, pSurfels);

	// Identify neighbors of large surfels and create edges between neighboring surfels. 

	MeshEdgePtr **pNewBoundaryElement = pSurfels->surfelBndMem;
	Array<MeshEdgePtr *> *pNewBoundary = pSurfels->surfelBndMem2;

	RVLQLIST_INIT(pSEdgeList_);

	pNewBoundaryElement = pSurfels->surfelBndMem;

	pSurfel = pSurfels->NodeArray.Element;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++, pSurfel++)
	{
		//if (pSurfel->size >= minSurfelSize)
		if (pSurfel->size > 1)
		{
#ifndef RVLVERSION_171125
			//if (!pSurfel->bEdge)
#endif
			{
				pPtList = &(pSurfel->PtList);

				pSurfel->BoundaryArray.Element = pNewBoundary;

				pMesh->Boundary(pPtList, pSurfels->surfelMap, pSurfel->BoundaryArray, pNewBoundaryElement, pSurfels->edgeMarkMap);

				pNewBoundary += pSurfel->BoundaryArray.n;
			}

			GetNeighbors(pMesh, pSurfels, iSurfel, &SEdgeList, nSEdges);
		}
	}

#ifdef RVLVERSION_171125
	// Detect boundaries.

	Boundaries(pMesh, pSurfels);

	// Detect edge features.

	EdgeFetures(pMesh, pSurfels, &SEdgeList, nSEdges);
#endif

	// Create surfel edge array.

	if (pSurfels->EdgeArray.Element)
		RVL_DELETE_ARRAY(pSurfels->EdgeArray.Element);

	pSurfels->EdgeArray.Element = new SURFEL::Edge *[nSEdges];

	pSurfels->EdgeArray.n = 0;

	SURFEL::Edge *pSEdge = SEdgeList.pFirst;

	while (pSEdge)
	{
		pSEdge->idx = pSurfels->EdgeArray.n;

		pSurfels->EdgeArray.Element[pSurfels->EdgeArray.n++] = pSEdge;

		pSEdge = pSEdge->pNext;
	}

	// Free memory

	pSurfels->FreeGetNeighborsBoundaryAndSize();

	FreePlanarRegionGrowingMem();

	delete[] regionGrowingBuffer;
	delete[] RandPtIdxArray.Element;
}

void PlanarSurfelDetector::InitPlanarRegionGrowing(
	Mesh *pMesh,
	SurfelGraph *pSurfels)
{
	int nPts = pMesh->NodeArray.n;

	regionGrowingData.iPtBuff = new int[nPts];

	regionGrowingData.iPtBuff2 = new int[nPts];

	regionGrowingData.iBoundaryPtBuff = new int[nPts];

	regionGrowingData.surfelPtArray.Element = regionGrowingData.iPtBuff;

	//regionGrowingData.buffer = new int[nPts];

	memset(regionGrowingData.buffer, 0xff, nPts * sizeof(int));

	regionGrowingData.iSurfelSeed = new int[nPts];

	//regionGrowingData.distThr = surfelDistThr * surfelDistThr;
	//regionGrowingData.kRGB2 = kRGB * kRGB;
	//regionGrowingData.kPlane2 = kPlane * kPlane;
	//regionGrowingData.surfelMap = pSurfels->surfelMap;
	//regionGrowingData.costMap = regionGrowingData.costBuffer = NULL;
#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
	data.costMap = new float[nPts];
	data.costBuffer = new float[nPts];
#endif
#endif
#endif
}

void PlanarSurfelDetector::PlanarRegionGrowing(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iPtSeed,
	int iSurfel)
{
	Point *Pt = pMesh->NodeArray.Element;

	int *piPtFetch = regionGrowingData.iPtBuff;
	int *piPtPut = piPtFetch;

	int *piBoundaryPt = regionGrowingData.iBoundaryPtBuff;

	*(piPtPut++) = iPtSeed;

	regionGrowingData.buffer[iPtSeed] = iSurfel;

#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
	data.costMap[iPtSeed] = 0.0f;
#endif
#endif
#endif

	regionGrowingData.iSurfelSeed[iSurfel] = iPtSeed;

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	SURFEL::CreateFromPoint(pSurfel, Pt + iPtSeed);

	if (pMesh->bLabels)
		pSurfel->ObjectID = Pt[iPtSeed].label;

	regionGrowingData.pTemplate = pSurfel;
	regionGrowingData.iSurfel = iSurfel;
	regionGrowingData.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION;
	regionGrowingData.kNormal2 = kNormal * kNormal;
	regionGrowingData.bLimitedDepthUnconstrainedRG = false;

	int *piPtBuffEnd = RegionGrowing<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &regionGrowingData, piPtFetch, piPtPut);

	regionGrowingData.surfelPtArray.n = piPtBuffEnd - regionGrowingData.surfelPtArray.Element;

	// Create surfel lists.

	QList<SURFEL::EdgePtr> *pSEdgeList = &(pSurfel->EdgeList);

	RVLQLIST_INIT(pSEdgeList);

	QList<QLIST::Index2> *pPtList = &(pSurfel->PtList);

	RVLQLIST_INIT(pPtList);

	// Compute surfel parameters

	Point *pPt = Pt + iPtSeed;

	if (regionGrowingData.surfelPtArray.n >= RVLMAX(20, minSurfelSize))
	{
		MESH::Distribution distribution;

		pMesh->ComputeDistribution(regionGrowingData.surfelPtArray, distribution);

		SURFEL::ComputeParameters(pSurfel, distribution, pPt);
	}
	else
		SURFEL::CreateFromPoint(pSurfel, pPt);

	// iPtSeed <- the closest point to pSurfel->P which is consistent with pSurfel

	regionGrowingData.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION;

	int iPtSeed_ = -1;
	float minDist = 0.0;

	float dist;
	float dP[3];
	int *piPt;
	int iPt;

	for (piPt = regionGrowingData.iPtBuff; piPt < piPtBuffEnd; piPt++)
	{
		iPt = *piPt;

		pPt = pMesh->NodeArray.Element + iPt;

		regionGrowingData.buffer[iPt] = -1;

		if (PSD::RegionGrowingOperation(iPt, 0, NULL, pMesh, &regionGrowingData))
		{
			RVLDIF3VECTORS(pPt->P, pSurfel->P, dP);

			dist = RVLDOTPRODUCT3(dP, dP);

			if (iPtSeed_ < 0 || dist < minDist)
			{
				iPtSeed_ = iPt;
				minDist = dist;
			}
		}

		regionGrowingData.buffer[iPt] = -1;
		//pSurfels->surfelMap[iPt] = iSurfel;
	}

	if (iPtSeed_ >= 0)
		iPtSeed = iPtSeed_;

	//// iPtSeed <- the closest point to iPtSeed which is consistent with ptTemplate

	//piPtFetch = piPtPut = iPtBuff;

	//*(piPtPut++) = iPtSeed;

	//data.iPtSeed = -1;
	//data.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER;

	//piPtBuffEnd = RegionGrowing2<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &data, piPtFetch, piPtPut);

	//for (piPt = iPtBuff; piPt < piPtBuffEnd; piPt++)
	//	regionGrowingBuffer[*piPt] = -1;

	//if (data.iPtSeed < 0)
	//	data.iPtSeed = iPtSeed;

	// Final region growing

	//data.kNormal2 = 0.0f;
	//data.kNormal2 = 4.0f;
	regionGrowingData.kNormal2 = (bNormalConstraintInSecondInitRG ? kNormal * kNormal : 0.0f);

	regionGrowingData.unconstrainedNormalDepthMap[iPtSeed] = 0;

	piPtFetch = piPtPut = regionGrowingData.iPtBuff2;

	*(piPtPut++) = iPtSeed;

	regionGrowingData.buffer[iPtSeed] = iSurfel;

	regionGrowingData.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION;
	//data.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK;
	regionGrowingData.size = 1;
	regionGrowingData.dSize = 1;
	regionGrowingData.maxSize = pMesh->NodeArray.n;
	regionGrowingData.iAttackedSurfel = iSurfel;
	regionGrowingData.bLimitedDepthUnconstrainedRG = bLimitedDepthUnconstrainedNormalRG;

	int *piPtBuff2End = RegionGrowing<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &regionGrowingData, piPtFetch, piPtPut);

	pSurfel->size = regionGrowingData.size;

	//for (piPt = iPtBuff; piPt < piPtBuffEnd; piPt++)
	//	pSurfels->surfelMap[*piPt] = -1;

	// Form the final surfel point set

	QLIST::Index2 *pPtIdx;

	for (piPt = regionGrowingData.iPtBuff2; piPt < piPtBuff2End; piPt++)
	{
		iPt = *piPt;

		//pPt = Pt + iPt;

		//iSurfel_ = pSurfels->surfelMap[iPt];

		//if (iSurfel_ >= 0)
		//{
		//	pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

		//	r0_ = pSurfel_->d / RVLDOTPRODUCT3(pSurfel_->N, pSurfel->P0);
		//	r = pSurfel->d / RVLDOTPRODUCT3(pSurfel->N, pPt->P);
		//	r_ = pSurfel_->d / RVLDOTPRODUCT3(pSurfel_->N, pPt->P);

		//	bAddToNewSurfel = ((r0_ - pSurfel->r0) * (r_ - r) >= 0.0);
		//}
		//else
		//	bAddToNewSurfel = true;

		//if (bAddToNewSurfel)
		{
			pSurfels->surfelMap[iPt] = iSurfel;

#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
			data.costMap[iPt] = data.costBuffer[iPt];
#endif
#endif
#endif
			pPtIdx = pSurfels->PtMem + iPt;

			RVLQLIST_ADD_ENTRY2(pPtList, pPtIdx);

			pPtIdx->Idx = iPt;
		}

		regionGrowingData.buffer[iPt] = -1;
	}	// for (piPt = iPtBuff; piPt < piPtBuffEnd; piPt++)

	pSurfel->size = piPtBuff2End - regionGrowingData.iPtBuff2;
}

void PlanarSurfelDetector::FreePlanarRegionGrowingMem()
{
	delete[] regionGrowingData.iSurfelSeed;
	//delete[] regionGrowingData.buffer;
	RVL_DELETE_ARRAY(regionGrowingData.costMap);
	RVL_DELETE_ARRAY(regionGrowingData.costBuffer);
	delete[] regionGrowingData.iPtBuff;
	delete[] regionGrowingData.iPtBuff2;
	delete[] regionGrowingData.iBoundaryPtBuff;
}

void PlanarSurfelDetector::DefineBoundaryTest(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int &iSurfel,
	int &iSurfel_,
	QList<QLIST::Index> &G)
{
	int nPts = pMesh->NodeArray.n;

#ifdef RVLPLANARSURFELDETECTOR_DEBUG
	if (iPtBuffDebug)
		delete[] iPtBuffDebug;

	iPtBuffDebug = new int[2 * nPts];
#endif

	if (iSurfel_ < iSurfel)
	{
		int iTmp = iSurfel;
		iSurfel = iSurfel_;
		iSurfel_ = iTmp;
	}

	PlanarSurfelDetectorRegionGrowingData data = regionGrowingData;

	GetNeighbors(pMesh, pSurfels, iSurfel);

	DefineBoundary(pMesh, pSurfels, data, iSurfel, iSurfel_);

	ClearProcessed();

	QList<QLIST::Index> *pGSeedPtList;

	QLIST::Index *pNeighbor = neighborList.pFirst;

	while (pNeighbor)
	{
		pGSeedPtList = GSeedListArray.Element + pNeighbor->Idx;

		RVLQLIST_INIT(pGSeedPtList);

		pNeighbor = pNeighbor->pNext;
	}

	//int *iPtBuff;
	//int nBBoundaryPts;

	//BBoundary(pMesh, pSurfels, iSurfel_, iPtBuff, nBBoundaryPts);	// iPtBuff <- array of indices of boundary points of iSurfel_
	//																// nBoundaryPts <- total no. of boundary points of iSurfel_

	//DefineBoundary(pMesh, pSurfels, data, iSurfel, iSurfel_, iPtBuff, nBBoundaryPts, G);

	// debugging

	//for (int i = 0; i < nPts; i++)
	//	if (map[i] != -1 || distanceMap[i] != 0xffffffff)
	//		int debug = 0;

	/////
}

// Input:  mesh pMesh,
//         surfel graph pSurfels,
//         initial W-surfel idx. iSurfel,
//         initial B-surfel idx. iSurfel_,
//         array of indices of boundary points of the initial B-region (B-surfel) iPtBuff,
//         total no. of boundary points of the initial B-region nBBndPts
//
// For a given mesh pMesh segmented to surfels pSurfels, this function determines the boundary between two neighboring surfels identifed by indices iSurfel and iSurfel_
// by reassigning some of the points of the surfel iSurfel to the surfel iSurfel_.
//
// The function uses the following temporary maps (member variables of PlanarSurfelDetector):
//
// map: at the beginning of the execution of DefineBoundary(), all elements must be set to -1.
//      at the end of the execution of DefineBoundary(), all elements are set to -1.
// distanceMap: at the beginning of the execution of DefineBoundary(), all elements must be set to 0xffffffff.
//              at the end of the execution of DefineBoundary(), all elements are set to 0xffffffff.
//
// iPtBuff must be allocated in Mem2A.

void PlanarSurfelDetector::DefineBoundary(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	PlanarSurfelDetectorRegionGrowingData &data,
	int iSurfel,
	int iSurfel_)
{
	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	QList<QLIST::Index2> *pW = &(pSurfel->PtList);
	QList<QLIST::Index2> *pB = &(pSurfel_->PtList);

	CRVLMem *pMem2A = &(Mem2A);

	pMem2A->Clear();

	CRVLMem *pMem2B = &Mem2B;

#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
	// Compute intersection plane.

	IntersectionPlane(pSurfels, iSurfel, iSurfel_);
#endif

	int nSurfels = pMesh->NodeArray.n;

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	//bool bDebug = (iSurfel == 0);
	bool bDebug = (iSurfel == debugDefineBoundaryiSurfel && iSurfel_ == debugDefineBoundaryiSurfel_);

	if (bDebug)
		SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);
#endif

	//if (iSurfel == 8 && iSurfel_ == 44)
	//	int debug = 0;

	// Allocate memory storage for G-region.

	int nMeshPts = pMesh->NodeArray.n;

	int *GRegionBuff;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, 2 * nMeshPts, GRegionBuff);

	/// Expand W-surfel into B-surfel.

	// G <- indices of points on the boundary of W-surfel, which have at least one neighbor in B-surfel and lie on the B-surfel plane.

	Array<int> G;

	G.Element = GRegionBuff;

	QLIST::CopyToArray(GSeedListArray.Element + iSurfel, &G);

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	if (bDebug)
	{
		FILE *fpDebugIdxArray = fopen("C:\\RVL\\Debug\\PSDIdxArray.txt", "w");

		SaveIdxArray(fpDebugIdxArray, G);

		fclose(fpDebugIdxArray);
	}
#endif

	// G_ <- indices of points on the boundary of B-surfel, which have at least one neighbor in W-surfel and lie on the W-surfel plane.

	Array<int> G_;

	G_.Element = G.Element + G.n;

	data.size = 0;
	data.maxSize = nMeshPts;
	data.dSize = 0;

	GetAttackSeed(pMesh, pSurfels, iSurfel_, nSurfels, iSurfel, G, G_);

	int i;

	for (i = 0; i < G_.n; i++)
		map[G_.Element[i]] = -1;

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	if (bDebug)
	{
		FILE *fpDebugIdxArray = fopen("C:\\RVL\\Debug\\PSDIdxArray.txt", "w");

		SaveIdxArray(fpDebugIdxArray, G_);

		fclose(fpDebugIdxArray);
	}
#endif

	// Attack iSurfel_ by iSurfel.

	data.size = 0;
	//data.maxSize = 4 * pSurfel_->size;
	data.maxSize = maxAttackSize;
	data.dSize = 1;

	Array<int> GBnd, WBnd;
	int iPt;
	int *iGPt, *iGPt0;
	bool *bPrevW;
	QLIST::Index2 *pPtIdx;

	if (G_.n > 0)
		//if (false)
	{
		// Attack B-surfel by W-surfel using G_ as the seed. The resulting G-region is stored in G_.

		data.bLimitedDepthUnconstrainedRG = false;

		GRegion(pMesh, pSurfels, data, iSurfel_, iSurfel, G_, GBnd, WBnd, bPrevW, false);

		// Reset map and distanceMap elements on the boundary of new B-surfel (after the attack).

		for (i = 0; i < WBnd.n; i++)
		{
			iPt = WBnd.Element[i];
			map[iPt] = -1;
			distanceMap[iPt] = 0xffffffff;
		}

		// G__ <- G

		RVLMEM_SET_FREE(pMem2A, G_.Element + G_.n);

		Array<int> G__;

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, nMeshPts, G__.Element);

		memcpy(G__.Element, G.Element, G.n * sizeof(int));

		G__.n = G.n;

		// G__ <- G__ + G_ - indices of points assigned to B-surfel in order to keep B-surfel connectivity during the attack of W-surfel.
		// Reset map and distanceMap.
		// Move points assigned to W-surfel by the attack to B-surfel to the PtList of W-surfel.

		iGPt0 = iGPt = G__.Element + G.n;

		for (i = 0; i < G_.n; i++)
		{
			iPt = G_.Element[i];
			map[iPt] = -1;
			distanceMap[iPt] = 0xffffffff;

			if (pSurfels->surfelMap[iPt] != iSurfel_)
			{
				pPtIdx = pSurfels->PtMem + iPt;

				RVLQLIST_MOVE_ENTRY2(pB, pW, pPtIdx, QLIST::Index2);

				*(iGPt++) = iPt;
			}
		}

		G__.n += (iGPt - iGPt0);

		// G <- G__

		G.Element = G__.Element;
		G.n = G__.n;

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
		if (bDebug)
		{
			SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);

			FILE *fpDebugIdxArray = fopen("C:\\RVL\\Debug\\PSDIdxArray.txt", "w");

			SaveIdxArray(fpDebugIdxArray, G);

			fclose(fpDebugIdxArray);
		}
#endif
	}
	else
		G.n = 0;

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	if (bDebug)
	{
		for (i = 0; i < nMeshPts; i++)
		{
			if (map[i] != -1)
				int debug = 0;

			if (distanceMap[i] != 0xffffffff)
				int debug = 0;
		}
	}
#endif

	///

	// Detect G-region.
	
	//QLIST::CopyToArray(GSeedListArray.Element + iSurfel, &G);

	int nSeed = G.n;

	data.bLimitedDepthUnconstrainedRG = bLimitedDepthUnconstrainedNormalRG;

	bool bW_ = GRegion(pMesh, pSurfels, data, iSurfel, iSurfel_, G, GBnd, WBnd, bPrevW);

	data.maxSize = nMeshPts;
	data.dSize = 0;
	data.size = 0;

	for (i = 0; i < G.n; i++)
	{
		iPt = G.Element[i];

		if (mProcessed[iPt] == 0x00)
		{
			mProcessed[iPt] = RVLPLANARSURFELDETECTOR_PROCESSED_G;

			processedBuff.Element[processedBuff.n++] = iPt;
		}
	}

	//

#ifdef RVLPLANARSURFELDETECTOR_DEFINE_BOUNDARY_OLD
	Array<int> seed;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, nMeshPts, seed.Element);

	QLIST::CopyToArray(GSeedListArray.Element + iSurfel, &seed);

	int *iPtBuff = seed.Element;

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	if (bDebug)
	{
		FILE *fpDebugIdxArray = fopen("C:\\RVL\\Debug\\PSDIdxArray.txt", "w");

		SaveIdxArray(fpDebugIdxArray, seed);

		fclose(fpDebugIdxArray);
	}
#endif

	/// B attacks iSurfel. G <- the regions of iSurfel conquered by B. W <- iSurfel \ G.

	int *iGPt = iPtBuff;

	int *piPtFetch = iGPt;

	int *piPtPut = iPtBuff + seed.n;

	int *piPt;

	for (piPt = iGPt; piPt < piPtPut; piPt++)
		map[*piPt] = 0;

	//int *iBBndPt = iPtBuff;

	//int *iGPt = iPtBuff + nBBndPts;

	//int *piPtFetch = iBBndPt;

	//int *piPtPut = iGPt;	

	CRVLMem *pMem2B = &Mem2B;

	int *iGBndPt;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2B, int, nMeshPts, iGBndPt);

	int *piGBndPtArrayEnd = iGBndPt;

	data.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK;
	data.iAttackedSurfel = iSurfel;
	data.iSurfel = 0;

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	Point PtTemplate;

	SURFEL::GetPoint(pSurfel_, &PtTemplate);	// PtTemplate <- reference point for B-region created from pSurfel_

	data.pPtTemplate = &PtTemplate;

	// iGPt <- array of indices of points in G-region
	// iGBndPt <- array of indices of boundary points of G-region
		
	int *piGPtArrayEnd = RegionGrowing3<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &data, piPtFetch, piPtPut,
		piGBndPtArrayEnd);

	int nG = piGPtArrayEnd - iGPt;	// nG <- total no. of points in G-region

	RVLMEM_SET_FREE(pMem2A, piGPtArrayEnd)

	RVLMEM_SET_FREE(pMem2B, piGBndPtArrayEnd)
	
	// iBoundaryGEnd = piBoundaryGEnd -  iBoundaryPtBuff;	// documentation

//#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
//	debugPtArray.Element = iPtBuff;	
//
//	int *piPtDebug = iPtBuff;
//
//	for (int *piPt = iBoundaryPtBuff; piPt < piBoundaryGEnd; piPt++)
//		if (pSurfels->surfelMap[*piPt] == iSurfel)
//			*(piPtDebug++) = *piPt;
//
//	debugPtArray.n = piPtDebug - debugPtArray.Element;
//#endif

	/// Detect connected components of W.
	/// nWCC <- total no. of connected components.
	/// For each boundary point i of every connected component j of W-region, where j = 0, 1, ..., nWCC-1
	///     map[i] <- j
	///     distanceMap[i] <- 0
	/// end for.

	// For all vertices i in G-region pSurfels->surfelMap[i] <- nSurfels 

	int nSurfels = pMesh->NodeArray.n;

	int iPt, iPt_;

	for (piPt = iGPt; piPt < piGPtArrayEnd; piPt++)
	{
		iPt = *piPt;

		pSurfels->surfelMap[iPt] = nSurfels;

		if (mProcessed[iPt] == 0x00)
		{
			mProcessed[iPt] = RVLPLANARSURFELDETECTOR_PROCESSED_G;

			processedBuff.Element[processedBuff.n++] = iPt;
		}
	}

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);

	if (bDebug)
	{
		FILE *fpSurfels = fopen("C:\\RVL\\Debug\\PSDSurfels.txt", "w");

		fprintf(fpSurfels, "%f\t%f\t%f\t%f\t%f\t%f\n", pSurfel->P[0], pSurfel->P[1], pSurfel->P[2], pSurfel->N[0], pSurfel->N[1], pSurfel->N[2]);
		fprintf(fpSurfels, "%f\t%f\t%f\t%f\t%f\t%f\n", pSurfel_->P[0], pSurfel_->P[1], pSurfel_->P[2], pSurfel_->N[0], pSurfel_->N[1], pSurfel_->N[2]);

		fclose(fpSurfels);
	}
#endif

	// 

	int nWCC = 0;

	//int *iWCCPtArray = piGEnd;
	int *iWCCPt;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, nMeshPts, iWCCPt);

	int *iWCCPtArrayEnd = iWCCPt;

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
	debugState = 0;
#endif

	MeshEdgePtr *pEdgePtr;
	MeshEdge *pEdge;

	for (piPt = iGBndPt; piPt < piGBndPtArrayEnd; piPt++)
	{
		iPt = *piPt;

		if (pSurfels->surfelMap[iPt] == nSurfels)	// Is this condition necessary ?
		{
			// Find a point iPt_, which is a neighbor of iPt, belongs to W-region and doesn't belong to any already detected connected component of W-region.
			// This point is used as the seed for new connected component of W-region.

			pEdgePtr = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

				if (pSurfels->surfelMap[iPt_] == iSurfel)
					if (map[iPt_] < 0)
					{
						// iPt_ is the seed for new connected component of W-region.

						nWCC++;

						ConnectedComponent(pMesh, pSurfels, iPt_, iSurfel, nWCC, iWCCPtArrayEnd, map, distanceMap);
					}

				pEdgePtr = pEdgePtr->pNext;
			}
		}
	}

	///

	int *iGEdgeBuff;
	int *piGEdgeBuffEnd;

	if (nWCC > 0)	// Check if there are connected components of W-region. If nWCC = 0, then W-region is completely covered by G-region.
	{
		// iWCC = iWCCPtArray - iPtBuff;		// documentation
		//int iWCCPtArrayEnd = iWCCPtArrayEnd - iPtBuff;

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
		debugPtArray.Element = iWCCPtArray;
		debugPtArray.n = iWCCPtArrayEnd - iWCCPtArray;

		nWCC = 1;
#endif

		/// If there are multiple connected components of W-region, then connect them into a single connected component.

		int nDistanceMatrixElements = nWCC * nWCC;

		unsigned int *distanceMatrix = new unsigned int[nDistanceMatrixElements];

		memset(distanceMatrix, 0xff, nDistanceMatrixElements * sizeof(unsigned int));

		MeshEdge **edgeMatrix = new MeshEdge *[nDistanceMatrixElements];

		PSD::DistanceComputationData distCompData;

		distCompData.distanceMap = distanceMap;
		distCompData.distanceMatrix = distanceMatrix;
		distCompData.edgeMatrix = edgeMatrix;
		distCompData.map = map;
		distCompData.nRegions = nWCC;

		// Compute distances between the connected components of W-region.
		// For each point of G-region, the corresponding element of map is set to the index of the closest connected component of W-region 
		// and the corresponding element of distanceMap is set to the distance to this connected component.

		piPtFetch = iWCCPt;

		piPtPut = iWCCPtArrayEnd;

		int *piPtDistanceBuffEnd = RegionGrowing<Mesh, Point, MeshEdge, MeshEdgePtr, PSD::DistanceComputationData, PSD::DistanceOperation>(pMesh, &distCompData, piPtFetch, piPtPut);

		RVLMEM_SET_FREE(pMem2A, piPtDistanceBuffEnd)

			if (nWCC > 1)
			{
			// Compute the minimum spanning tree of the connected components of W.

			int *tree = new int[nWCC];

			MinimumSpanningTree(distanceMatrix, nWCC, tree);

			// Connect W-region into a single connected component.

			MeshEdge **edgeArray = edgeMatrix + nWCC;
			int iWCC;

			for (iWCC = 1; iWCC < nWCC; iWCC++, edgeArray += nWCC)
			{
				pEdge = edgeArray[tree[iWCC]];

				Connect(pMesh, pEdge, iSurfel, map, distanceMap, pSurfels->surfelMap);
			}

			delete[] tree;

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
			SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);
#endif
			}

		delete[] distanceMatrix;
		delete[] edgeMatrix;
	}	// if (nWCC > 0)
#endif	

	iGPt = G.Element;
	int *piGPtArrayEnd = G.Element + G.n;
	int nG = G.n;

	int *piPt;

	int *iGEdgeBuff;
	int *piGEdgeBuffEnd;

	if (bW_)
	{		
		int *iGBndPt = GBnd.Element;
		int *piGBndPtArrayEnd = GBnd.Element + GBnd.n;
		int *iWCCPt = WBnd.Element;
		int *iWCCPtArrayEnd = WBnd.Element + WBnd.n;

//#ifdef NEVER	// switch off cut propagation
		/// Determine the boundary between surfels iSurfel and iSurfel_ by cut propagation.		

		// Allocate memory for a buffer used by CutPropagation().

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2B, int, pMesh->EdgeArray.n, iGEdgeBuff);

		piGEdgeBuffEnd = iGEdgeBuff;

		// Allocate memory for buffers used by BWConnect().

		RVLMEM_ALLOC_LOCAL_INIT(pMem2A);

		int *iGBBndPt;

		//RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, PointEdgeArray.n + nG, iGBBndPt);
		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, nG, iGBBndPt);

		RVLMEM_ALLOC_LOCAL_UPDATE(pMem2A);

		int *piGBBndPtArrayEnd = iGBBndPt;

		//int *iBWConnectionPt = iGBBndPt + PointEdgeArray.n;
		int *iBWConnectionPt = iGBBndPt;

		int *piBWConnectionPtArrayEnd = iBWConnectionPt;

		// Find a point iPt, which is a boundary point of G-region and use it as the seed for boundary detection.

		bool bBWConnected = false;

		piPt = iGBndPt;

		while(piPt < piGBndPtArrayEnd)
		{
			iPt = *piPt;

			if (pSurfels->surfelMap[iPt] != nSurfels)	// This test is required because some boundary points of G-region can be assigned to W-region in the process of
														// connecting W-region into a single connected component.
			{
				piPt++;

				continue;
			}

			if (map[iPt] < 0)
			{
				piPt++;

				continue;
			}

			// iPt is the seed for boundary detection.

			Array<MESH::PointEdge> PointEdgeArray;

			PointEdgeArray.Element = PointEdgeBuff;

			Array<MESH::PointEdge> cut;

			cut.Element = PointEdgeBuff + 2 * pMesh->EdgeArray.n;

			int iSourceStart;
			int iSourceEnd;
			int iSinkStart;
			int iSinkEnd;
			bool bB;
			bool bW;

			EdgeBoundary(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, iPt, PointEdgeArray, iSourceStart, iSourceEnd, iSinkStart, iSinkEnd, bB, bW, map, -1);

			if (bB)
			{
				//int *iBuffExt = NULL;

				//int *iBWBoundary;

				//int BWBuffSize = PointEdgeArray.n + nG;

				//if (iWCCEnd + BWBuffSize <= 2 * pMesh->NodeArray.n)
				//	iBWBoundary = iWCCPtArrayEnd;
				//else
				//{
				//	iBuffExt = new int[PointEdgeArray.n + nG];

				//	iBWBoundary = iBuffExt;
				//}

				//int *iBWConnection = iBWBoundary + PointEdgeArray.n;
				//int *piBWConnectionEnd = iBWConnection;

				// iBWBoundary_ = iBWBoundary - iPtBuff;		// documentation
				// iBWConnection_ = iBWConnection - iPtBuff;	// documentation

				if (bW)
				{
					int nPtEdges;

					CutPropagation(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, PointEdgeArray, iSourceStart, iSourceEnd, iSinkStart, iSinkEnd, map, -1, piGEdgeBuffEnd, nPtEdges);

					MinimumCut(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_, PointEdgeArray, iSinkStart, iSinkEnd, nPtEdges);

					piPt++;
				}
				else if (!bBWConnected)
				{
					if (BWConnect(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, PointEdgeArray, piGBBndPtArrayEnd, piBWConnectionPtArrayEnd))
					{
						bBWConnected = true;

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
						SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);
#endif

						//if (iPt_ >= 0)
						//	EdgeBoundary(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, iPt_, PointEdgeArray, iSourceStart, iSourceEnd, iSinkStart, iSinkEnd, bB, bW, map, -1);

						for (piPt = iGBndPt; piPt < piGBndPtArrayEnd; piPt++)
							map[*piPt] = 0;

						piPt = iGBndPt;
					}
				}

				//RVL_DELETE_ARRAY(iBuffExt);

				//break;
			}	// if (bB)
			else
				piPt++;
		}	// while(piPt < piGBndPtArrayEnd)

		//int *piPt_;

		//for (piPt_ = iBWConnectionPt; piPt_ < piBWConnectionPtArrayEnd; piPt_++)
		//	pSurfels->surfelMap[*piPt_] = nSurfels;

		RVLMEM_ALLOC_LOCAL_FREE(pMem2A);
//#endif		// switch off cut propagation

		// Reset map and distanceMap.

		for (piPt = iWCCPt; piPt < iWCCPtArrayEnd; piPt++)
		{
			map[*piPt] = -1;
			distanceMap[*piPt] = 0xffffffff;
		}
	}	// if(nWCC > 0)

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);
#endif

	// Reset map and distanceMap.

	for (piPt = iGPt; piPt < piGPtArrayEnd; piPt++)
	{
		map[*piPt] = -1;
		distanceMap[*piPt] = 0xffffffff;
	}

//#ifdef RVLPLANARSURFELDETECTOR_DEBUG
//	// Only for debugging purpose!
//	// Check if whole map is set to -1 and whole distanceMap to 0xffffffff
//
//	for (int i = 0; i < pMesh->NodeArray.n; i++)
//		if (map[i] != -1 || distanceMap[i] != 0xffffffff)
//			int debug = 0;
//#endif

//#ifdef NEVER		// switch off cut propagation
	/// Grow B-region util reaching the cut.

	int *iExpBPtBuff;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, nG + 1, iExpBPtBuff);

	PSD::ReassignToBData reassignToBData;

	reassignToBData.BID = iSurfel_;
	reassignToBData.GID = nSurfels;
	reassignToBData.edgeFlags = edgeFlags;
	reassignToBData.map = pSurfels->surfelMap;
	reassignToBData.pPSD = this;

	//int *pSeedEnd = G.Element + nSeed;
	int *pSeedEnd = G.Element + G.n;

	MeshEdgePtr *pEdgePtr;
	MeshEdge *pEdge;
	int iPt_;
	int *piPtFetch, *piPtPut;	

	for (piPt = G.Element; piPt < pSeedEnd; piPt++)
	{
		iPt = *piPt;

		if (pSurfels->surfelMap[iPt] == nSurfels)
		{
			pEdgePtr = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

				if (pSurfels->surfelMap[iPt_] == iSurfel_)
					if (!(edgeFlags[pEdge->idx] & RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT))
					{
						iExpBPtBuff[0] = iPt_;

						piPtFetch = iExpBPtBuff;

						piPtPut = iExpBPtBuff + 1;

						int *piBPtArrayEnd = RegionGrowing<Mesh, Point, MeshEdge, MeshEdgePtr, PSD::ReassignToBData, PSD::ReassignToB>(pMesh, &reassignToBData, piPtFetch, piPtPut);

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
						SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);
#endif

						break;
					}

				pEdgePtr = pEdgePtr->pNext;
			}
		}
	}

	///

	if (bW_)
	{
		// Reset edgeFlags and cutCostMap

		int *piEdge;

		for (piEdge = iGEdgeBuff; piEdge < piGEdgeBuffEnd; piEdge++)
		{
			edgeFlags[*piEdge] = 0x00;
			cutCostMap[*piEdge] = 0xffffffff;
		}
	}

//#endif	// switch off cut propagation

//#ifdef RVLPLANARSURFELDETECTOR_DEBUG
//	// Only for debugging purpose!
//	// Check if all edgeFlags are set to 0x00 and whole cutCostMap to 0xffffffff
//
//	for (int i = 0; i < pMesh->EdgeArray.n; i++)
//		if (edgeFlags[i] != 0x00 || cutCostMap[i] != 0xffffffff)
//			int debug = 0;
//#endif

	// Reassign points to surfels.

	int ID;


	for (i = 0; i < nG; i++)
	{
		iPt = G.Element[i];

		ID = pSurfels->surfelMap[iPt];

		if (ID == iSurfel_)
		{
			if (bPrevW[i])
			{
				pPtIdx = pSurfels->PtMem + iPt;

				RVLQLIST_MOVE_ENTRY2(pW, pB, pPtIdx, QLIST::Index2);
			}
		}
		else if (ID == nSurfels)
		{
			if (!bPrevW[i])
			{
				pPtIdx = pSurfels->PtMem + iPt;

				RVLQLIST_MOVE_ENTRY2(pB, pW, pPtIdx, QLIST::Index2);
			}

			pSurfels->surfelMap[iPt] = iSurfel;
		}
	}

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);
#endif

	//QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;
	//QLIST::Index2 **ppPtIdx = &(pSurfel->PtList.pFirst);

	//while (pPtIdx)
	//{
	//	//if (map[pPtIdx->Idx] >= 0 && distanceMap[pPtIdx->Idx] > 0)
	//	ID = pSurfels->surfelMap[pPtIdx->Idx];
	//	
	//	if (ID == iSurfel_)
	//	{
	//		RVLQLIST_REMOVE_ENTRY(pW, pPtIdx, ppPtIdx);
	//		RVLQLIST_ADD_ENTRY(pB, pPtIdx);		
	//		pPtIdx = (*ppPtIdx);
	//	}
	//	else
	//	{
	//		if (ID == nSurfels)
	//			pSurfels->surfelMap[pPtIdx->Idx] = iSurfel;

	//		ppPtIdx = &(pPtIdx->pNext);
	//		pPtIdx = pPtIdx->pNext;
	//	}
	//}

	RVLMEM_SET_FREE(pMem2A, GRegionBuff)

	pMem2B->Clear();
}

int PSD::DistanceOperation(
	int iNode,
	int iNode_,
	MeshEdge *pEdge,
	Mesh *pMesh,
	PSD::DistanceComputationData *pData)
{
	//if (iNode == 142837)
	//	int debug = 0;

	int iRegion = pData->map[iNode];

	if (iRegion < 0)
		return 0;

	if (iRegion > 0)
	{
		int iRegion_ = pData->map[iNode_];

		if (iRegion != iRegion_)
		{
			unsigned int distance = pData->distanceMap[iNode_] + pData->distanceMap[iNode];

			int iDistanceMatrixElement = (iRegion - 1) * pData->nRegions + iRegion_ - 1;

			if (pData->distanceMatrix[iDistanceMatrixElement] > distance)
			{			
				int iDistanceMatrixElement_ = (iRegion_ - 1) * pData->nRegions + iRegion - 1;

				pData->distanceMatrix[iDistanceMatrixElement] = pData->distanceMatrix[iDistanceMatrixElement_] = distance;
				pData->edgeMatrix[iDistanceMatrixElement] = pData->edgeMatrix[iDistanceMatrixElement_] = pEdge;
			}				
		}

		return 0;
	}

	pData->map[iNode] = pData->map[iNode_];

	pData->distanceMap[iNode] = pData->distanceMap[iNode_] + 1;

	return 1;
}

// For a given mesh pMesh, surfels pSurfels, surfel index regionIdx and the point index iPt, 
// this function determines the connected component of the surfel regionIdx which contains iPt.
// For each boundary point of the detected connected component, function ConnectedComponent() sets the corresponding element of map to the value componentIdx.
// It is assumed that all elements of map corresponding to the connected component are initially set to -1.
// For each boundary point of the detected connected component, function ConnectedComponent() sets the corresponding element of distanceMap to 0.
// The function stores the indices of all boundary points of the detected connected component to a buffer,
// whose starting pointer is given by the input value of iPtArray and the ending pointer is returned as the final value of the same variable.
 

void PlanarSurfelDetector::ConnectedComponent(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iPt,
	int regionIdx,
	int componentIdx,
	int *&iPtArray,
	int *map,
	unsigned int *distanceMap)
{
#ifdef RVLMESH_BOUNDARY_DEBUG
	if (debugState >= pMesh->debugState)
		return;

	FILE *fpDebug = fopen("C:\\RVL\\Debug\\PSDConnectedComponentDebug.txt", "w");
#endif

	//QList<QLIST::Index> PtList;

	//QList<QLIST::Index> *pPtList = &PtList;

	//RVLQLIST_INIT(pPtList);

	MeshEdgePtr *pEdgePtr;

	if (!pMesh->IsBoundaryPoint(iPt, pSurfels->surfelMap, regionIdx, pEdgePtr))
	{
		// iPt represents a single point component.

		if (map[iPt] < 0)
		{
			map[iPt] = componentIdx;

			distanceMap[iPt] = 0;

			*(iPtArray++) = iPt;
		}

#ifdef RVLMESH_BOUNDARY_DEBUG
		fclose(fpDebug);
#endif
		return;
	}

#ifdef RVLMESH_BOUNDARY_DEBUG
	fprintf(fpDebug, "P %d (%d) E %d\n", iPt, map[iPt], pEdgePtr - pMesh->EdgePtrMem);
#endif

	// Follow boundary

	MeshEdge *pEdge = pEdgePtr->pEdge;

	int side = (pEdge->iVertex[0] == iPt ? 0 : 1);

	MeshEdgePtr *pEdgePtr0 = pEdgePtr;

	int iNeighborPt;
	QList<MeshEdgePtr> *pEdgeList;
	Point *pPt;

	do
	{
		if (map[iPt] < 0)
		{
			map[iPt] = componentIdx;

			distanceMap[iPt] = 0;

			*(iPtArray++) = iPt;
		}

#ifdef RVLMESH_BOUNDARY_DEBUG
		debugState++;

		if (debugState >= pMesh->debugState)
			break;
#endif

		RVLMESH_GET_POINT(pEdge, 1 - side, iPt, pEdgePtr);
		RVLMESH_GET_NEXT_IN_REGION(pMesh, iPt, pEdgePtr, side, pSurfels->surfelMap, iNeighborPt, pPt, pEdgeList, pEdge);

#ifdef RVLMESH_BOUNDARY_DEBUG
		fprintf(fpDebug, "\nP %d (%d) E %d\n", iPt, map[iPt], pEdgePtr - pMesh->EdgePtrMem);
#endif
	} while (pEdgePtr != pEdgePtr0);

#ifdef RVLMESH_BOUNDARY_DEBUG
	fclose(fpDebug);
#endif
}

bool PlanarSurfelDetector::MinimumSpanningTree(
	unsigned int *connection,
	int n,
	int *tree)
{
	tree[0] = -1;

	bool *bInTree = new bool[n];

	memset(bInTree, 0, n * sizeof(bool));

	bInTree[0] = true;

	int *iTree = new int[n];

	iTree[0] = 0;	

	int i, j, k, j_;
	unsigned int minCost;
	int iMinCost;
	int iParent;
	unsigned int *connection_;

	for (i = 1; i < n; i++)
	{
		minCost = 0xffffffff;

		for (j = 0; j < i; j++)
		{
			j_ = iTree[j];

			connection_ = connection + n * j_;

			for (k = 0; k < n; k++)
			{
				if (bInTree[k])
					continue;

				if (connection_[k] < minCost)
				{
					minCost = connection_[k];
					iMinCost = k;
					iParent = j_;
				}
			}
		}

		if (minCost == 0xffffffff)
			return false;

		tree[iMinCost] = iParent;
		bInTree[iMinCost] = true;
		iTree[i] = iMinCost;
	}

	delete[] bInTree;
	delete[] iTree;

	return true;
}

void PlanarSurfelDetector::Connect(
	Mesh *pMesh,
	MeshEdge *pEdge,
	int idx,
	int *map,
	unsigned int *distanceMap,
	int *tgtMap)
{
	int i;
	int iPt, iPt_, iPt__;
	int iRegion;
	MeshEdge *pEdge_;
	MeshEdgePtr *pEdgePtr;
	unsigned int minDistance;

	for (i = 0; i < 2; i++)
	{
		iPt = pEdge->iVertex[i];

		iRegion = map[iPt];

		while (distanceMap[iPt] > 0)
		{
			tgtMap[iPt] = idx;

			minDistance = 0xffffffff;

			pEdgePtr = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge_, iPt_);

				if (map[iPt_] == iRegion)
					if (distanceMap[iPt_] < minDistance)
					{
						minDistance = distanceMap[iPt_];

						iPt__ = iPt_;
					}

				pEdgePtr = pEdgePtr->pNext;
			}

			iPt = iPt__;
		}
	}
}

// Function EdgeBoundary detects G-boundary (See ARP3D.TR3).
//
// Input:  pMesh - mesh,
//         map - surfel map,
//         WID - W-surfel index,
//         GID - G-region index,
//         BID - B-region index,
//         iPt0 - idx. of the initial vertex of the G-boundary,
//
// Output: PointEdgeArray - array of point-edge pairs representing the detected G-boundary,
//         iSourceStart - the index of the first point-edge of the WB-segment
//         iSourceEnd - the index of the last point-edge of the WB-segment
//         iSinkStart - the index of the first point-edge of the BW-segment
//         iSinkEnd - the index of the last point-edge of the BW-segment
//         bB - G-boundary contains a B-segment
//         bW - G-boundary contains a W-segment
//         markMap - array whose each element corresponds to a mesh vertex; the values of all elements corresponding to the G points of G-boundary are set to mark
//         mark - see description of markMap

void PlanarSurfelDetector::EdgeBoundary(
	Mesh *pMesh,
	int *map,
	int WID,
	int GID,
	int BID,
	int iPt0,
	Array<MESH::PointEdge> &PointEdgeArray,
	int &iSourceStart,
	int &iSourceEnd,
	int &iSinkStart,
	int &iSinkEnd,
	bool &bB,
	bool &bW,
	int *markMap,
	int mark)
{
#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
	bool bDebug = (WID == debugDefineBoundaryiSurfel && BID == debugDefineBoundaryiSurfel_);
	
	Point *pPtDebug;
	int debugCounter;

	if (bDebug)
	{
		fpDebugPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "w");
		fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "w");

		pPtDebug = pMesh->NodeArray.Element + iPt0;

		fprintf(fpDebugPts, "%d\t%f\t%f\t%f\t0\n", iPt0, pPtDebug->P[0], pPtDebug->P[1], pPtDebug->P[2]);

		SaveNeighborhood(pMesh, map, WID, GID, BID, iPt0, fpDebugPts, fpDebugEdges);

		debugCounter = 0;
	}
#endif

	// Find first boundary edge

	markMap[iPt0] = mark;

	Point *pPt = pMesh->NodeArray.Element + iPt0;

	QList<MeshEdgePtr> *pEdgeList = &(pPt->EdgeList);

	MeshEdgePtr *pEdgePtr = pEdgeList->pFirst;

	bool bNotG = false;

	int side;
	int iOppPt;
	MeshEdge *pEdge;
	int OppID;

	while (pEdgePtr)
	{
		RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt0, pEdgePtr, pEdge, iOppPt, side);

		OppID = map[iOppPt];

		if (OppID == WID || OppID == BID)
			break;

		if (OppID == GID)
		{
			if (bNotG)
				break;
		}
		else
			bNotG = true;

		pEdgePtr = pEdgePtr->pNext;
	}

	if (pEdgePtr == NULL)
	{
		pEdgePtr = pEdgeList->pFirst;

		RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt0, pEdgePtr, pEdge, iOppPt, side);

		OppID = map[iOppPt];

		if (OppID != GID)
		{
			PointEdgeArray.n = 0;

#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
			if (bDebug)
			{
				fclose(fpDebugPts);
				fclose(fpDebugEdges);
			}
#endif	
		}
	}

	MeshEdgePtr *pEdgePtr0 = NULL;

	int iPt = iPt0;

	MESH::PointEdge *pPointEdge = PointEdgeArray.Element;

	bB = bW = false;

	int state = 0;	// 0 - undefined
					// 1 - white region
					// 2 - black region

	// Follow boundary

	int prevState = 0;

	bool bFirst = true;

	MESH::PointEdge *pPointEdgeBW;

	do
	{
		if (OppID != GID)	// if (OppID == WID || OppID == BID)
		{
			prevState = state;

			if (OppID == WID)
			{
				bW = true;
				state = 1;
			}
			else
			{
				bB = true;
				state = 2;
			}
		}

		if (state > 0)
		{
			pPointEdge->iPt = iPt;
			pPointEdge->pEdgePtr = pEdgePtr;
			pPointEdge->side = side;

			if (OppID != GID)
			{
				if (prevState == 1 && state == 2)
				{
					iSourceStart = pPointEdgeBW - PointEdgeArray.Element;
					iSourceEnd = pPointEdge - PointEdgeArray.Element;
				}
				else if (prevState == 2 && state == 1)
				{
					iSinkStart= pPointEdgeBW - PointEdgeArray.Element;
					iSinkEnd= pPointEdge - PointEdgeArray.Element;
				}

				pPointEdgeBW = pPointEdge;
			}

			pPointEdge++;
		}

#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
		if (bDebug)
		{
			Point *pPtDebug = pMesh->NodeArray.Element + iOppPt;

			fprintf(fpDebugPts, "%d\t%f\t%f\t%f\t%d\n", iOppPt, pPtDebug->P[0], pPtDebug->P[1], pPtDebug->P[2], (OppID == GID ? 0 : (OppID == WID ? 1 : 2)));

			fprintf(fpDebugEdges, "%d\t%d\t%d\t1\n", pEdge->idx, iPt, iOppPt);

			fflush(fpDebugPts);
			fflush(fpDebugEdges);

			debugCounter++;

			if (debugCounter % 10 == 0)
				int debug = 0;
		}
#endif

		if (OppID == GID)
		{
			iPt = iOppPt;
			pPt = pMesh->NodeArray.Element + iPt;
			pEdgeList = &(pPt->EdgeList);
			pEdgePtr = pEdge->pVertexEdgePtr[1 - side];

			markMap[iPt] = mark;

#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
			if (bDebug)
				SaveNeighborhood(pMesh, map, WID, GID, BID, iPt, fpDebugPts, fpDebugEdges);
#endif
		}

		if (pEdgePtr == pEdgePtr0)
			break;

		if (bFirst || (prevState == 0 && state != 0))
		{
			pEdgePtr0 = pEdgePtr;

			bFirst = false;
		}

		RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE(pEdgeList, iPt, pEdgePtr, side, map, iOppPt, pEdge, OppID, WID, GID, BID);
	} while (true);

	PointEdgeArray.n = pPointEdge - PointEdgeArray.Element;

#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
	if (bDebug)
	{
		if (bW && bB)
		{
			for (int iPointEdge = iSourceStart; iPointEdge <= iSourceEnd; iPointEdge++)
			{
				pPointEdge = PointEdgeArray.Element + iPointEdge;

				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(pPointEdge->iPt, pPointEdge->pEdgePtr, pEdge, iOppPt);

				if (iPointEdge == iSourceStart)
					fprintf(fpDebugEdges, "%d\t%d\t%d\t2\n", pEdge->idx, iOppPt, pPointEdge->iPt);
				else
					fprintf(fpDebugEdges, "%d\t%d\t%d\t2\n", pEdge->idx, pPointEdge->iPt, iOppPt);
			}

			for (int iPointEdge = iSinkStart; iPointEdge <= iSinkEnd; iPointEdge++)
			{
				pPointEdge = PointEdgeArray.Element + iPointEdge;

				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(pPointEdge->iPt, pPointEdge->pEdgePtr, pEdge, iOppPt);

				if (iPointEdge == iSinkStart)
					fprintf(fpDebugEdges, "%d\t%d\t%d\t3\n", pEdge->idx, iOppPt, pPointEdge->iPt);
				else
					fprintf(fpDebugEdges, "%d\t%d\t%d\t3\n", pEdge->idx, pPointEdge->iPt, iOppPt);
			}
		}

		fclose(fpDebugPts);
		fclose(fpDebugEdges);
	}
#endif	
}

#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
void PlanarSurfelDetector::SaveNeighborhood(
	Mesh *pMesh,
	int *map,
	int WID,
	int GID,
	int BID,
	int iPt,
	FILE *fpPts,
	FILE *fpEdges)
{
	Point *pPt = pMesh->NodeArray.Element + iPt;

	QList<MeshEdgePtr> *pEdgeList = &(pPt->EdgeList);

	MeshEdgePtr *pEdgePtr = pEdgeList->pFirst;

	MeshEdge *pEdge;
	int iOppPt;
	int side;
	int OppID;

	while (pEdgePtr)
	{
		RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iOppPt, side);

		OppID = map[iOppPt];

		if (OppID == WID || OppID == GID || OppID == BID)
		{
			pPt = pMesh->NodeArray.Element + iOppPt;

			fprintf(fpPts, "%d\t%f\t%f\t%f\t%d\n", iOppPt, pPt->P[0], pPt->P[1], pPt->P[2], (OppID == GID ? 0 : (OppID == WID ? 1 : 2)));

			fprintf(fpEdges, "%d\t%d\t%d\t0\n", pEdge->idx, iPt, iOppPt);
		}

		pEdgePtr = pEdgePtr->pNext;
	}

	fflush(fpPts);
	fflush(fpEdges);
}
#endif

// Function propagation is described in ARP3D.TR3
//
// Input:  pMesh - mesh,
//         map - surfel map,
//         WID - W-surfel index,
//         GID - G-region index,
//         BID - B-region index,
//         BoundaryPointEdgeArray - G-boundary,
//         iSourceStart - the index of the first point-edge of the WB-segment
//         iSourceEnd - the index of the last point-edge of the WB-segment
//         iSinkStart - the index of the first point-edge of the BW-segment
//         iSinkEnd - the index of the last point-edge of the BW-segment

void PlanarSurfelDetector::CutPropagation(
	Mesh *pMesh,
	int *map,
	int WID,
	int GID,
	int BID,
	Array<MESH::PointEdge> &BoundaryPointEdgeArray,
	int iSourceStart,
	int iSourceEnd,
	int iSinkStart,
	int iSinkEnd,
	int *markMap,
	int mark,
	int *&piEdgeBuffEnd,
	int &nPtEdges)
{
	// Set flag RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK of all sink point-edges. 
	// Put the indices of all sink edges to iEdgeBuff.

	int i;
	MESH::PointEdge *pPtEdge;
	int iEdge;

	for (i = iSinkStart; i <= iSinkEnd; i++)
	{
		pPtEdge = BoundaryPointEdgeArray.Element + i;

		iEdge = pPtEdge->pEdgePtr->pEdge->idx;

		if (edgeFlags[iEdge] == 0)
			*(piEdgeBuffEnd++) = iEdge;

		// the first sink point-edge is the opposite of the first point-edge of BW-segment.
		edgeFlags[iEdge] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (i == iSinkStart ? 1 - pPtEdge->side : pPtEdge->side));
	}

	// Set the COST of all source edges to 1. COST in ARP3D.TR3 corresponds to cutCostMap.
	// Put indices of all source point-edges to cutPropagationBuff.
	// Set flag RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED of all source point-edges. 
	// The set of all point-edges with flag RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED set is denoted in the following comments by CLOSED.
	// This set is denoted in ARP3D.TR3 by C.

	int iPtEdge = BoundaryPointEdgeArray.n;

	QList<QLIST::Index> *pCutPropagationBuff = &cutPropagationBuff;		// cutPropagationBuff is denoted in ARP3D.TR3 by Z.

	RVLQLIST_INIT(pCutPropagationBuff);

	QLIST::Index *pCutPropagationBuffEntry = cutPropagationBuffMem;

#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
	QList<QLIST::Index> *pLineCutBuff = &lineCutBuff;

	RVLQLIST_INIT(pLineCutBuff);
#endif

	MeshEdge *pEdge;
	MESH::PointEdge *pPtEdge_;
	unsigned char side;
	//bool bNewPtEdge;

	for (i = iSourceStart; i <= iSourceEnd; i++)
	{
		pPtEdge_ = BoundaryPointEdgeArray.Element + i;
		pEdge = pPtEdge_->pEdgePtr->pEdge;
		iEdge = pEdge->idx;

		if (edgeFlags[iEdge] == 0)
			*(piEdgeBuffEnd++) = iEdge;

		if (i == iSourceStart)
		{
			side = 1 - pPtEdge_->side;

			//PushToCutPropagationBuffer(pMesh->NodeArray.Element, pEdge, side, &(cutPropagationBuff.pFirst), true, 0, pCutPropagationBuffEntry, BoundaryPointEdgeArray, iPtEdge);
			PushToCutPropagationBuffer(pMesh->NodeArray.Element, pEdge, side, true, 0, pCutPropagationBuffEntry, BoundaryPointEdgeArray, iPtEdge);
		}
		else
		{
			side = pPtEdge_->side;

			//PushToCutPropagationBuffer(pMesh->NodeArray.Element, pEdge, side, &(cutPropagationBuff.pFirst), false, 0, pCutPropagationBuffEntry, BoundaryPointEdgeArray, i);
			PushToCutPropagationBuffer(pMesh->NodeArray.Element, pEdge, side, false, 0, pCutPropagationBuffEntry, BoundaryPointEdgeArray, i);
		}			

//		iEdge = pEdge->idx;
//
//		cutCostMap[iEdge] = 1;
//
//		if (i == iSourceStart)	// the first source point-edge is the opposite of the first point-edge of the first point-edge of WB-segment.
//		{
//			side = 1 - pPtEdge_->side;
//
//			if (!(edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (1 - side))))
//			{				
//				pPtEdge->side = side;
//				pPtEdge->iPt = pEdge->iVertex[side];
//				pPtEdge->pEdgePtr = pEdge->pVertexEdgePtr[side];	
//
//				iPtEdge = pPtEdge - BoundaryPointEdgeArray.Element;
//
//#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
//				RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_PUSH(pEdge, iPtEdge, P1, P2, NLineCut, dLineCut, 0, piPtEdge, pCutPropagationBuff, pCutPropagationBuffEntry, cutCostMap);
//#else
//				RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_PUSH(iPtEdge, pCutPropagationBuff, pCutPropagationBuffEntry);
//#endif
//
//				pPtEdge++;
//			}
//		}
//		else
//		{
//			side = pPtEdge_->side;
//
//			if (!(edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (1 - side))))
//			{
//				RVLQLIST_ADD_ENTRY(pCutPropagationBuff, pCutPropagationBuffEntry);
//
//				pCutPropagationBuffEntry->Idx = i;
//
//				pCutPropagationBuffEntry++;
//			}
//		}

		edgeFlags[iEdge] |= ((RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << (1 - side)) | RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SOURCE);
	}

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	bool *bDebugMap;
	unsigned int debugDepth;
	int debugLoopCounter;

	bool bDebug = (WID == debugDefineBoundaryiSurfel && BID == debugDefineBoundaryiSurfel_);

	if (bDebug)
	{
		fpDebugPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "a");
		fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "a");

		bDebugMap = new bool[pMesh->NodeArray.n];

		memset(bDebugMap, 0, pMesh->NodeArray.n * sizeof(bool));

		debugDepth = 0;
	}
#endif

	//// Wave propagation starting from the source point-edges, which are stored in iPointEdgeBuff.

	//QLIST::Index **piFetch = &(cutPropagationBuff.pFirst);
	QLIST::Index **ppiFetch = &(cutPropagationBuff.pFirst);
	QLIST::Index **ppiFetchLine = &(lineCutBuff.pFirst);

	MeshEdgePtr *pEdgePtr, *pEdgePtr0;
	QList<MeshEdgePtr> *pEdgeList;
	int iPt;
	int iNextPt, iPrevPt;
	int ID;
	int iEdge_;

	while (true)	// wave propagation loop
	{
		// pPtEdge_ <- Pull(Z)

		//pPtEdge_ = BoundaryPointEdgeArray.Element + (*piFetch)->Idx;

		//piFetch = &((*piFetch)->pNext);

#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
		if (*ppiFetchLine)
		{
			pPtEdge_ = BoundaryPointEdgeArray.Element + (*ppiFetchLine)->Idx;

			ppiFetchLine = &((*ppiFetchLine)->pNext);
		}			
		else
#endif
		{
			if (*ppiFetch)
			{
				pPtEdge_ = BoundaryPointEdgeArray.Element + (*ppiFetch)->Idx;

				ppiFetch = &((*ppiFetch)->pNext);
			}				
			else
				break;
		}

		// iEdge <- edge of pPtEdge_

		pEdgePtr0 = pPtEdge_->pEdgePtr;

		iEdge = pEdgePtr0->pEdge->idx;

		// If iEdge is in CLOSED

		if (!(edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << pPtEdge_->side)))
		{
			// Put iEdge in iEdgeBuff if it is not already there. 

			if (edgeFlags[iEdge] == 0)
				*(piEdgeBuffEnd++) = iEdge;

			// Put pPtEdge_ in CLOSED.

			edgeFlags[iEdge] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << pPtEdge_->side);

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
			if (bDebug)
			{
				if (cutCostMap[iEdge] > debugDepth)
				{
					debugDepth = cutCostMap[iEdge];

					fflush(fpDebugPts);
					fflush(fpDebugEdges);
				}

				SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdgePtr0->pEdge, pPtEdge_->side, 5, bDebugMap);

				fflush(fpDebugPts);
				fflush(fpDebugEdges);

				debugLoopCounter = 0;
			}

			//if (iEdge == 37367)
			//	int debug = 0;
#endif
			/// For every point-edge in the Loop(pPtEdge_)

			pEdgePtr = pEdgePtr0;
			iPt = pPtEdge_->iPt;

			markMap[iPt] = mark;

			// (pEdge, iNextPt) <- next point-edge in the loop

			RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr, side, map, iNextPt, pEdge, ID, WID, GID, BID);

			// for all point-edges in the Loop(pPtEdge_)

			while (pEdgePtr != pEdgePtr0)
			{
				iPrevPt = iPt;
				iPt = iNextPt;

				// (pEdge, iPt) <- next point-edge in the loop

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
				if (bDebug)
					debugLoopCounter++;
#endif

				if (map[iPrevPt] == GID || map[iPt] == GID)	// if pEdge is in E^G (See ARP3D.TR3)
				{
					iEdge_ = pEdge->idx;

					// Put iEdge in iEdgeBuff if it is not already there. 

					if (edgeFlags[iEdge_] == 0)
						*(piEdgeBuffEnd++) = iEdge_;

					// Put (pEdge, iPt) to CLOSED.

					edgeFlags[iEdge_] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << side);

					// 

					if (map[iPt] == GID)
						markMap[iPt] = mark;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
					if (bDebug)
					{
						SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdge, side, 5, bDebugMap);

						fflush(fpDebugPts);
						fflush(fpDebugEdges);
					}

					int debugiPtEdge = iPtEdge;
#endif
					//PushToCutPropagationBuffer(pMesh->NodeArray.Element, pEdge, 1 - side, piFetch, true, cutCostMap[iEdge], pCutPropagationBuffEntry, BoundaryPointEdgeArray, iPtEdge);
					PushToCutPropagationBuffer(pMesh->NodeArray.Element, pEdge, 1 - side, true, cutCostMap[iEdge], pCutPropagationBuffEntry, BoundaryPointEdgeArray, iPtEdge);

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
					if (bDebug)
					{
						if (iPtEdge != debugiPtEdge)
						{
							SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdge, 1 - side, 4, bDebugMap);

							fflush(fpDebugPts);
							fflush(fpDebugEdges);

							int debug = 0;
						}
					}
#endif

					//				if (!(edgeFlags[iEdge_] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << (1 - side))))	// If Opp(pEdge, iPt) is not in CLOSED.
					//				{					
					//					if (!(edgeFlags[iEdge_] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << side)))	// If (pEdge, iPt) is not a sink point-edge.
					//					{
					//						// Push (pEdge, Opp(pEdge, iPt)) to Z.
					//
					//						pPtEdge->side = 1 - side;
					//						pPtEdge->iPt = pEdge->iVertex[pPtEdge->side];						
					//						pPtEdge->pEdgePtr = pEdge->pVertexEdgePtr[pPtEdge->side];
					//
					//						RVLQLIST_ADD_ENTRY(pCutPropagationBuff, pCutPropagationBuffEntry);
					//
					//						pCutPropagationBuffEntry->Idx = pPtEdge - BoundaryPointEdgeArray.Element;
					//
					//						pCutPropagationBuffEntry++;
					//
					//#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
					//						if (bDebug)
					//						{
					//							SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdge, pPtEdge->side, 4, bDebugMap);
					//
					//							fflush(fpDebugPts);
					//							fflush(fpDebugEdges);
					//						}
					//#endif
					//						pPtEdge++;
					//					}
					//
					//					// COST(pEdge) <- COST(pEdge) + 1
					//						
					//					cutCostMap[iEdge_] = cutCostMap[iEdge] + 1;					
					//				}
				}	// if (map[iPrevPt] == GID || map[iPt] == GID)

				RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr, side, map, iNextPt, pEdge, ID, WID, GID, BID);
			}	// for all point-edges in the Loop(pPtEdge_)
		}	// If iEdge is in CLOSED

		///

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
		if (bDebug)
			if (debugLoopCounter >= 10)
				int debug = 0;
#endif
	}	// wave propagation loop

	nPtEdges = iPtEdge;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	if (bDebug)
	{
		fclose(fpDebugPts);
		fclose(fpDebugEdges);

		delete[] bDebugMap;
	}
#endif
}

#ifdef RVLPLANARSURFELDETECTOR_DEBUG
void PlanarSurfelDetector::SaveEdge(
	FILE *fpPts,
	FILE *fpEdges,
	Mesh *pMesh,
	int *map,
	int WID,
	int GID,
	int BID,
	MeshEdge *pEdge,
	int side,
	int type,
	bool *bMap)
{
	fprintf(fpEdges, "%d\t", pEdge->idx);

	int iPt;
	float *P;

	for (int i = 0; i < 2; i++)
	{
		iPt = pEdge->iVertex[(side + i) % 2];

		if (!bMap[iPt])
		{
			bMap[iPt] = true;

			P = pMesh->NodeArray.Element[iPt].P;

			fprintf(fpPts, "%d\t%f\t%f\t%f\t%d\n", iPt, P[0], P[1], P[2], (map[iPt] == GID ? 0 : (map[iPt] == WID ? 1 : 2)));
		}

		fprintf(fpEdges, "%d\t", iPt);
	}

	fprintf(fpEdges, "%d\n", type);
}
#endif

//void PlanarSurfelDetector::LineCut(
//	Mesh *pMesh,
//	int *map,
//	int WID,
//	int GID,
//	int BID,
//	MESH::PointEdge *pPtEdge,
//	int *markMap,
//	int mark,
//	int *&piEdgeBuffEnd)
//{
//	MeshEdgePtr *pEdgePtr0 = pPtEdge->pEdgePtr;
//
//	bool bLineCut = true;
//
//	MeshEdgePtr *pEdgePtr, *pNextEdgePtr0;
//	QList<MeshEdgePtr> *pEdgeList;
//	int iPt;
//	int iNextPt, iPrevPt;
//	int ID;
//	int iEdge, iEdge_;
//	MeshEdge *pEdge;
//	int side;
//	float *P1, *P2;
//
//	while (bLineCut)
//	{
//		iEdge = pEdgePtr0->pEdge->idx;
//
//		// If iEdge is in CLOSED, then stop the procedure.
//
//		if (edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << pPtEdge->side))
//			return;
//
//		// Put iEdge in iEdgeBuff if it is not already there. 
//
//		if (edgeFlags[iEdge] == 0)
//			*(piEdgeBuffEnd++) = iEdge;
//
//		// Put pPtEdge_ in CLOSED.
//
//		edgeFlags[iEdge] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << pPtEdge->side);
//
//#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
//		if (bDebug)
//		{
//			if (cutCostMap[iEdge] > debugDepth)
//			{
//				debugDepth = cutCostMap[iEdge];
//
//				fflush(fpDebugPts);
//				fflush(fpDebugEdges);
//			}
//
//			SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdgePtr0->pEdge, pPtEdge_->side, 5, bDebugMap);
//
//			fflush(fpDebugPts);
//			fflush(fpDebugEdges);
//
//			debugLoopCounter = 0;
//		}
//#endif
//		/// For every point-edge in the Loop(pPtEdge_)
//
//		pEdgePtr = pEdgePtr0;
//		iPt = pPtEdge->iPt;
//
//		markMap[iPt] = mark;
//
//		// (pEdge, iNextPt) <- next point-edge in the loop
//
//		RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr, side, map, iNextPt, pEdge, ID, WID, GID, BID);
//
//		// for all point-edges in the Loop(pPtEdge_)
//
//		bLineCut = false;
//
//		while (pEdgePtr != pEdgePtr0)
//		{
//			iPrevPt = iPt;
//			iPt = iNextPt;
//
//			// (pEdge, iPt) <- next point-edge in the loop
//
//#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
//			if (bDebug)
//				debugLoopCounter++;
//#endif
//
//			if (map[iPrevPt] == GID || map[iPt] == GID)	// if pEdge is in E^G (See ARP3D.TR3)
//			{
//				iEdge_ = pEdge->idx;
//
//				// Put iEdge in iEdgeBuff if it is not already there. 
//
//				if (edgeFlags[iEdge_] == 0)
//					*(piEdgeBuffEnd++) = iEdge_;
//
//				// Put (pEdge, iPt) to CLOSED.
//
//				edgeFlags[iEdge_] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << side);
//
//				// 
//
//				if (map[iPt] == GID)
//					markMap[iPt] = mark;
//
//#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
//				if (bDebug)
//				{
//					SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdge, side, 5, bDebugMap);
//
//					fflush(fpDebugPts);
//					fflush(fpDebugEdges);
//				}
//#endif
//
//				if (!(edgeFlags[iEdge_] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << (1 - side))))	// If Opp(pEdge, iPt) is not in CLOSED.
//				{
//					if (!(edgeFlags[iEdge_] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << side)))	// If (pEdge, iPt) is not a sink point-edge.
//					{
//						P1 = pMesh->NodeArray.Element[iPrevPt].P;
//						P2 = pMesh->NodeArray.Element[iPt].P;
//
//						if (RVLPLANARSURFELDETECTOR_ON_LINE_CUT(NLineCut, dLineCut, P1, P2) && !bLineCut)
//						{
//							bLineCut = true;
//
//							pNextEdgePtr0 = pEdgePtr;
//						}
//						else
//						{
//
//							// Push (pEdge, Opp(pEdge, iPt)) to Z.
//
//							pPtEdge->side = 1 - side;
//							pPtEdge->iPt = pEdge->iVertex[pPtEdge->side];
//							pPtEdge->pEdgePtr = pEdge->pVertexEdgePtr[pPtEdge->side];
//
//							*(piPut++) = pPtEdge - BoundaryPointEdgeArray.Element;
//
//#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
//							if (bDebug)
//							{
//								SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdge, pPtEdge->side, 4, bDebugMap);
//
//								fflush(fpDebugPts);
//								fflush(fpDebugEdges);
//							}
//#endif
//							pPtEdge++;
//						}
//					}
//
//					// COST(pEdge) <- COST(pEdge) + 1
//
//					cutCostMap[iEdge_] = cutCostMap[iEdge] + 1;
//				}
//			}	// if (map[iPrevPt] == GID || map[iPt] == GID)
//
//			RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr, side, map, iNextPt, pEdge, ID, WID, GID, BID);
//		}	// for all point-edges in the Loop(pPtEdge_)
//
//		///
//
//		if (bLineCut)
//			pEdgePtr0 = pNextEdgePtr0;
//	}
//}

bool PlanarSurfelDetector::MinimumCut(
	Mesh *pMesh,
	SurfelGraph *pSurfels, 
	int WID,
	int GID,
	int BID,
	Array<MESH::PointEdge> &BoundaryPointEdgeArray,
	int iSinkStart,
	int iSinkEnd,
	int nPtEdges)
{
	// pEdgePtr0 <- sink point-edge with the minimum cut cost.

	unsigned int minCutCost = 0xffffffff;

	MeshEdgePtr *pEdgePtr0 = NULL;

	int i;
	MESH::PointEdge *pPtEdge;
	int iEdge;
	int side0;

	for (i = iSinkStart; i <= iSinkEnd; i++)
	{
		pPtEdge = BoundaryPointEdgeArray.Element + i;

		iEdge = pPtEdge->pEdgePtr->pEdge->idx;

		if (cutCostMap[iEdge] < minCutCost)
		{
			minCutCost = cutCostMap[iEdge];

			side0 = (i == iSinkStart ? 1 - pPtEdge->side : pPtEdge->side);

			pEdgePtr0 = pPtEdge->pEdgePtr->pEdge->pVertexEdgePtr[side0];
		}
	}

	if (pEdgePtr0 == NULL)
		return false;

	MeshEdgePtr *pEdgePtr = pEdgePtr0;

	MeshEdge *pEdge = pEdgePtr->pEdge;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	bool bDebug = (WID == debugDefineBoundaryiSurfel && BID == debugDefineBoundaryiSurfel_);

	if (bDebug)
	{
		fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "a");

		fprintf(fpDebugEdges, "%d\t%d\t%d\t6\n", pEdge->idx, pEdge->iVertex[0], pEdge->iVertex[1]);
		fprintf(fpDebugEdges, "%d\t%d\t%d\t6\n", pEdge->idx, pEdge->iVertex[1], pEdge->iVertex[0]);

		fflush(fpDebugEdges);
	}
#endif
	
	edgeFlags[pEdge->idx] |= RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT;

	// Follow the minimum cost cut path to the source.

#ifdef RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY
	int *cutBuffMem = new int[2 * nPtEdges];

	Array<int> minCut;
	minCut.Element = cutBuffMem;

	Array<int> lineCut;
	lineCut.Element = cutBuffMem + nPtEdges;

	int *piCutEdge = minCut.Element;

	*(piCutEdge++) = pEdge->idx;
#endif

	int side;	
	MeshEdge *pEdge_;
	MeshEdgePtr *pEdgePtr_, *pNextEdgePtr;
	int iPt, iNextPt;
	int side_;
	int ID;
	QList<MeshEdgePtr> *pEdgeList;
	unsigned int cutCost;
	int side1, side2;

	while (!(edgeFlags[pEdge->idx] & RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SOURCE))
	{
		cutCost = cutCostMap[pEdge->idx];

		pNextEdgePtr = NULL;

		if (pEdgePtr == pEdgePtr0)
			side1 = side2 = side0;
		else
		{
			side1 = 0;
			side2 = 1;
		}

		for (side = side1; side <= side2; side++)
		{
			pEdgePtr = pEdge->pVertexEdgePtr[side];

			iPt = pEdge->iVertex[side];

			pEdgePtr_ = pEdgePtr;

			RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr_, side_, pSurfels->surfelMap, iNextPt, pEdge_, ID, WID, GID, BID);

			pNextEdgePtr = NULL;

			while (pEdgePtr_ != pEdgePtr)
			{			
				if (pSurfels->surfelMap[iPt] == GID || pSurfels->surfelMap[iNextPt] == GID)
				{
					if (cutCostMap[pEdge_->idx] < cutCost)
					{
						pNextEdgePtr = pEdgePtr_;

						break;
					}
				}

				iPt = iNextPt;

				RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr_, side_, pSurfels->surfelMap, iNextPt, pEdge_, ID, WID, GID, BID);
			}

			if (pNextEdgePtr)
				break;
		}

		if (pNextEdgePtr == NULL)
			return false;

		pEdgePtr = pNextEdgePtr;

		pEdge = pEdgePtr->pEdge;

#ifdef RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY
		*(piCutEdge++) = pEdge->idx;
#else
		edgeFlags[pEdge->idx] |= RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT;
#endif

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
		if (bDebug)
		{
			fprintf(fpDebugEdges, "%d\t%d\t%d\t6\n", pEdge->idx, pEdge->iVertex[0], pEdge->iVertex[1]);
			fprintf(fpDebugEdges, "%d\t%d\t%d\t6\n", pEdge->idx, pEdge->iVertex[1], pEdge->iVertex[0]);

			fflush(fpDebugEdges);
		}
#endif
	}

#ifdef RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY
	// poly <- list of vertices of the initial polygonal boundary.

	minCut.n = piCutEdge - minCut.Element;

	QLIST::Index *polyMem = new QLIST::Index[2 * minCut.n];

	QLIST::Index *pPolyVertex = polyMem;

	QList<QLIST::Index> poly;

	QList<QLIST::Index> *pPoly = &poly;	

	RVLQLIST_INIT(pPoly);

	bool bIntersection = false;
	bool bPrevEdgeIsOnIntersection = false;

	for (i = 0; i < minCut.n; i++)
	{
		iEdge = minCut.Element[i];

		bIntersection = ((edgeFlags[iEdge] & RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_INTERSECTION) != 0);

		if (!bIntersection & bPrevEdgeIsOnIntersection)
		{
			RVLQLIST_ADD_ENTRY(pPoly, pPolyVertex);

			pPolyVertex->Idx = i - 1;

			pPolyVertex++;
		}

		if (i == 0 || (bIntersection & !bPrevEdgeIsOnIntersection) || i == minCut.n - 1)
		{			
			RVLQLIST_ADD_ENTRY(pPoly, pPolyVertex);

			pPolyVertex->Idx = i;

			pPolyVertex++;
		}

		bPrevEdgeIsOnIntersection = bIntersection;
	}

	// Compute midpoints of the initial cut edges.

	float *midPt = new float[3 * minCut.n];

	float *P = midPt;

	for (i = 0; i < minCut.n; i++, P += 3)
	{
		iEdge = minCut.Element[i];

		GetEdgeMidPoint(pMesh, iEdge, P, NLineCut, dLineCut, (edgeFlags[iEdge] & RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_INTERSECTION) != 0);
	}

	// Assure that the polygonal boundary passes completely through G-region.

	if (minCut.n > 1)
	{
		int side = side0;

		float *N = pSurfels->NodeArray.Element[WID].N;

		int iEdge1_, iEdge2_;
		float *P1, *P2, *P_;
		float dP[3], NBnd[3];
		float fTmp;
		float dBnd;
		int iMostDistantEdge;
		float dist, maxDist;

		QLIST::Index **ppPolyVertex = &(poly.pFirst);

		iEdge1_ = (*ppPolyVertex)->Idx;

		iEdge = minCut.Element[iEdge1_];

		P1 = midPt + 3 * iEdge1_;

		ppPolyVertex = &((*ppPolyVertex)->pNext);

		while (*ppPolyVertex)
		{
			iEdge2_ = (*ppPolyVertex)->Idx;

			P2 = midPt + 3 * iEdge2_;

			RVLDIF3VECTORS(P2, P1, dP);

			RVLCROSSPRODUCT3(N, dP, NBnd);

			RVLNORM3(NBnd, fTmp);

			dBnd = RVLDOTPRODUCT3(NBnd, P1);

			if (InsideGRegion(pMesh, pSurfels, WID, GID, BID, NBnd, dBnd, iEdge1_, side, iEdge2_, P2, dP, minCut, lineCut))
			{
				iEdge1_ = iEdge2_;

				P1 = P2;

				side = -1;

				ppPolyVertex = &((*ppPolyVertex)->pNext);
			}
			else
			{
				iMostDistantEdge = iEdge1_;

				maxDist = -1.0;

				P_ = midPt + 3 * iEdge1_;

				for (i = iEdge1_; i <= iEdge2_; i++, P_ += 3)
				{
					dist = RVLDOTPRODUCT3(NBnd, P_) - dBnd;

					dist = RVLABS(dist);

					if (dist > maxDist)
					{
						maxDist = dist;

						iMostDistantEdge = i;
					}
				}

				RVLQLIST_INSERT_ENTRY2(ppPolyVertex, pPolyVertex);

				pPolyVertex->Idx = iMostDistantEdge;

				pPolyVertex++;
			}			
		}
	}	// if (nCut > 1)

	// Free memory.

	delete[] cutBuffMem;
	delete[] polyMem;
	delete[] midPt;
#endif

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	if (bDebug)
		fclose(fpDebugEdges);
#endif

	return true;
}

// Function BWConnect connects B and W region.
// 
// Input:  pMesh - mesh,
//         map - surfel map,
//         WID - W-surfel index,
//         GID - G-region index,
//         BID - B-region index,
//         BoundaryPointEdgeArray - G-boundary
//
// Output: iGBBndPtArrayEnd - end of a point array in which G-boundary points are stored
//         piBWConnectionEnd - end of a point array in which the connection path between B- and W-region is stored

bool PlanarSurfelDetector::BWConnect(
	Mesh *pMesh,
	int *map,
	int WID,
	int GID,
	int BID,
	Array<MESH::PointEdge> &BoundaryPointEdgeArray,
	int *&iGBBndPtArrayEnd,
	int *&piBWConnectionEnd)
{
	// iPt0 <- the point on the boundary of G-region touching B-region closest to a W-region.

	//int *iGBBndPt = iGBBndPtArrayEnd;

	unsigned int minDistance = 0xffffffff;

	int iPt0 = -1;

	int i;
	MESH::PointEdge *pPointEdge;
	int iPt;
	Point *pPt;
	QList<MeshEdgePtr> *pEdgeList;
	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr;
	int side;
	int iNeighborPt;
	unsigned int distance;

	for (i = 0; i < BoundaryPointEdgeArray.n; i++)
	{
		pPointEdge = BoundaryPointEdgeArray.Element + i;

		iPt = pPointEdge->iPt;

		pPt = pMesh->NodeArray.Element + iPt;

		pEdgeList = &(pPt->EdgeList);

		pEdgePtr = pEdgeList->pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side);

			if (map[iNeighborPt] == BID)
				break;

			pEdgePtr = pEdgePtr->pNext;
		}

		if (pEdgePtr)
		{
			distance = distanceMap[iPt];

			if (distance < minDistance)
			{
				minDistance = distance;

				iPt0 = iPt;
			}

			//*(iGBBndPtArrayEnd++) = iPt;
		}
	}

	if (iPt0 < 0)
		return false;

	// Connect iPt with the closest W-point.

	int *iBWConnection = piBWConnectionEnd;

	int *piGBPt = iBWConnection;

	unsigned char halfDistance = minDistance / 2;

	iPt = iPt0;

	while (map[iPt] != WID)
	{
		if (distanceMap[iPt] >= halfDistance)
		{
			map[iPt] = BID;

			*(piGBPt++) = iPt;
		}
		else
			map[iPt] = WID;

		pEdgePtr = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side);

			if (distanceMap[iNeighborPt] < minDistance)
			{
				minDistance = distanceMap[iNeighborPt];

				break;
			}
				
			pEdgePtr = pEdgePtr->pNext;
		}

		iPt = iNeighborPt;
	}

	piBWConnectionEnd = piGBPt;

	//// iPtGB <- G-point whose neighbor is a B-point

	//for (piGBPt = iGBBndPt; piGBPt < iGBBndPtArrayEnd; piGBPt++)
	//	if (map[*piGBPt] == GID)
	//	{
	//		iGBPt = (*piGBPt);

	//		break;
	//	}
	//		
	//if (iGBPt < 0)
	//{
	//	for (piGBPt = iBWConnection; piGBPt < piBWConnectionEnd; piGBPt++)
	//	{
	//		iPt = (*piGBPt);

	//		pEdgePtr = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

	//		while (pEdgePtr)
	//		{
	//			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side);

	//			if (map[iNeighborPt] == GID)
	//			{
	//				iGBPt = iNeighborPt;

	//				break;
	//			}					

	//			pEdgePtr = pEdgePtr->pNext;
	//		}

	//		if (iGBPt >= 0)
	//			break;
	//	}		
	//}

	return true;
}

int PSD::ReassignToB(
	int iNode,
	int iNode_,
	MeshEdge *pEdge,
	Mesh *pMesh,
	PSD::ReassignToBData *pData)
{
	int iSurfel = pData->map[iNode];

	if (iSurfel != pData->GID)
	{
		if (iSurfel >= 0)
		{
			PlanarSurfelDetector *pPSD = pData->pPSD;

			if (pPSD->mProcessed[iNode] == 0x00)
				pPSD->AddToSeed(pMesh, iNode, iSurfel);
		}

		return 0;
	}

	if (pData->edgeFlags[pEdge->idx] & RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT)
		return 0;

	pData->map[iNode] = pData->BID;

	return 1;
}

// Input:  mesh pMesh,
//         surfel graph pSurfels,
//         surfel idx. iSurfel_
// Output: iPtBuff <- array of indices of boundary points of iSurfel_
//         nBoundaryPts <- total no. of boundary points of iSurfel_

void PlanarSurfelDetector::BBoundary(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel_,
	int *&iPtBuff,
	int &nBoundaryPts)
{
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	QList<QLIST::Index> Boundary;

	pMesh->Boundary(&(pSurfel_->PtList), pSurfels->surfelMap, &Boundary, BoundaryMem);	// Boundary <- boundary of pSurfel_

	Array<int> Boundary_;

	CRVLMem *pMem = &Mem2A;

	pMem->Clear();

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, pMesh->NodeArray.n, iPtBuff);

	Boundary_.Element = iPtBuff;

	QLIST::CopyToArray(&Boundary, &Boundary_);

	nBoundaryPts = Boundary_.n;
}

// Input:  pMesh - mesh
//         pSurfels - surfel graph
//         iSurfel - index of the surfel whose neighbors should be identified.
//
// Output: Updated GSeedListArray (member variable). Lists corresponding to the neigboring surfels of the surfel iSurfel are filled with indices
//              of points on the boundary of the neighboring surfel which have at least one neighbor in iSurfel and lie on the plane of iSurfel.
//
// The function uses mProcessed map, whose elements must be set to 0 before calling this function. 

void PlanarSurfelDetector::GetNeighbors(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel)
{
	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;

	pNewNeighbor = neighborMem;

	QList<QLIST::Index> *pNeighborList = &neighborList;

	RVLQLIST_INIT(pNeighborList);

	pNewGSeedPt = GSeedMem;

	regionGrowingData.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK;
	regionGrowingData.iSurfel = 0;
	regionGrowingData.bLimitedDepthUnconstrainedRG = bLimitedDepthUnconstrainedNormalRG;

	MeshEdgePtr *pEdgePtr;
	int iPt, iPt_;
	MeshEdge *pEdge;
	int iSurfel_;

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		processedBuff.Element[processedBuff.n++] = iPt;

		mProcessed[iPt] = RVLPLANARSURFELDETECTOR_PROCESSED_G;

		Point *pPt = pMesh->NodeArray.Element + iPt;

		pEdgePtr = pPt->EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

			if (mProcessed[iPt_] == 0x00)
			{
				iSurfel_ = pSurfels->surfelMap[iPt_];

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
				if (iSurfel == debugDefineBoundaryiSurfel_ && iSurfel_ == debugDefineBoundaryiSurfel)
					int debug = 0;
#endif

				if (iSurfel_ != iSurfel)
					if (iSurfel_ >= 0)
						AddToSeed(pMesh, iPt_, iSurfel_);
			}

			pEdgePtr = pEdgePtr->pNext;
		}

		pPtIdx = pPtIdx->pNext;
	}
}

void PlanarSurfelDetector::AddToSeed(
	Mesh *pMesh,
	int iPt_,
	int iSurfel_)
{
	processedBuff.Element[processedBuff.n++] = iPt_;

	mProcessed[iPt_] = RVLPLANARSURFELDETECTOR_PROCESSED_G;

	regionGrowingData.iAttackedSurfel = iSurfel_;	

	if (PSD::RegionGrowingOperation(iPt_, 0, NULL, pMesh, &regionGrowingData) > 0)
	{
		regionGrowingData.buffer[iPt_] = -1;

		QList<QLIST::Index> *pSeedPtList = GSeedListArray.Element + iSurfel_;

		if (pSeedPtList->pFirst == NULL)
		{
			QList<QLIST::Index> *pNeighborList = &neighborList;

			RVLQLIST_ADD_ENTRY(pNeighborList, pNewNeighbor);

			pNewNeighbor->Idx = iSurfel_;

			pNewNeighbor++;
		}

		RVLQLIST_ADD_ENTRY(pSeedPtList, pNewGSeedPt);

		pNewGSeedPt->Idx = iPt_;

		//if (iPt_ == 111409)
		//	int debug = 0;

		pNewGSeedPt++;
	}
}

// Function DefinePolygon corresponds to the procedure ExpandSegment in ARP3D.TR3

void PlanarSurfelDetector::DefinePolygon(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel_)
{
	int nPts = pMesh->NodeArray.n;

	// neighborList <- neighbors of iSurfel_.

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	if (iSurfel_ == debugDefineBoundaryiSurfel_)
		SaveWGB(pMesh, pSurfels, debugDefineBoundaryiSurfel, nPts, debugDefineBoundaryiSurfel_);
#endif

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel_;

	regionGrowingData.pTemplate = pSurfel;
	regionGrowingData.maxSize = nPts;
	regionGrowingData.size = 0;
	regionGrowingData.dSize = 0;
	regionGrowingData.buffer = map;

	GetNeighbors(pMesh, pSurfels, iSurfel_);

	// Define boundaries between iSurfel and all surfels in neighborList.

	PlanarSurfelDetectorRegionGrowingData data = regionGrowingData;

	QList<QLIST::Index> *pNeighborList = &neighborList;

	int iSurfel;
	//QList<QLIST::Index> G;
	QList<QLIST::Index> *pGSeedPtList;
	QLIST::Index *pNeighbor;
	QLIST::Index **ppNeighbor;

	while (neighborList.pFirst)
	{
		// iSurfel <- the first neighbor in neighborList

		pNeighbor = neighborList.pFirst;

		iSurfel = pNeighbor->Idx;

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (!pSurfel->bEdge)
		{
			// Define boundary between iSurfel and iSurfel_.

			DefineBoundary(pMesh, pSurfels, data, iSurfel, iSurfel_);

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
			if (iSurfel_ == debugDefineBoundaryiSurfel_)
				SaveWGB(pMesh, pSurfels, debugDefineBoundaryiSurfel, nPts, debugDefineBoundaryiSurfel_);

			//if (cutCostMap[452844] < 0xffffffff)
			//	int debug = 0;

			//if (iSurfel == 18)
			//{
			//	int iPtDebug = 74819;

			//	Point *pPtDebug = pMesh->NodeArray.Element + iPtDebug;

			//	MeshEdge *pEdgeDebug;
			//	int iPtDebug_;

			//	MeshEdgePtr *pEdgePtrDebug = pPtDebug->EdgeList.pFirst;

			//	while (pEdgePtrDebug)
			//	{
			//		RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPtDebug, pEdgePtrDebug, pEdgeDebug, iPtDebug_);

			//		if (pSurfels->surfelMap[iPtDebug_] == 18)
			//			break;

			//		pEdgePtrDebug = pEdgePtrDebug->pNext;
			//	}

			//	if (pEdgePtrDebug == NULL)
			//		int debug = 0;
			//}
#endif
		}

		// Clear the seed points from the seed point list of iSurfel.

		pGSeedPtList = GSeedListArray.Element + iSurfel;

		RVLQLIST_INIT(pGSeedPtList);

		// Remove the first neighbor from neighborList.		

		ppNeighbor = &(neighborList.pFirst);

		RVLQLIST_REMOVE_ENTRY(pNeighborList, pNeighbor, ppNeighbor);
	}

	// Clear mProcessed map and processedBuff.

	ClearProcessed();
}

void PlanarSurfelDetector::ClearProcessed()
{
	int *piProcessedBuffEnd = processedBuff.Element + processedBuff.n;

	int *piPt;

	for (piPt = processedBuff.Element; piPt < piProcessedBuffEnd; piPt++)
		mProcessed[*piPt] = 0x00;

	processedBuff.n = 0;
}

#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
void PlanarSurfelDetector::IntersectionPlane(
	SurfelGraph *pSurfels,
	int iSurfel,
	int iSurfel_)
{
	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	float *N = pSurfel->N;
	float *N_ = pSurfel_->N;

	RVLDIF3VECTORS(N_, N, NLineCut);

	float fTmp = sqrt(RVLDOTPRODUCT3(NLineCut, NLineCut));

	if (fTmp >= 0.01f)
	{
		RVLSCALE3VECTOR2(NLineCut, fTmp, NLineCut);

		dLineCut = (pSurfel_->d - pSurfel->d) / fTmp;
	}
	else
	{
		RVLNULL3VECTOR(NLineCut);

		dLineCut = 1.0f;
	}
}
#endif

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
void PlanarSurfelDetector::SaveWGB(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int WID,
	int GID,
	int BID)
{
	bool bDebug = (WID == debugDefineBoundaryiSurfel && BID == debugDefineBoundaryiSurfel_);
	//bool bDebug = (WID == 0);

	if (bDebug)
	{
		fpDebugPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "w");
		fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "w");
		fclose(fpDebugEdges);

		Surfel *pSurfel = pSurfels->NodeArray.Element + WID;

		QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;
		Point *pPt;
		int iPt;
		int type;

		while (pPtIdx)
		{
			iPt = pPtIdx->Idx;

			pPt = pMesh->NodeArray.Element + iPt;

			type = (pSurfels->surfelMap[iPt] == WID ? 1 : (pSurfels->surfelMap[iPt] == GID ? 0 : 2));

			fprintf(fpDebugPts, "%d\t%f\t%f\t%f\t%d\n", iPt, pPt->P[0], pPt->P[1], pPt->P[2], type);

			pPtIdx = pPtIdx->pNext;
		}

		pSurfel = pSurfels->NodeArray.Element + BID;

		pPtIdx = pSurfel->PtList.pFirst;

		while (pPtIdx)
		{
			iPt = pPtIdx->Idx;

			pPt = pMesh->NodeArray.Element + iPt;

			type = (pSurfels->surfelMap[iPt] == WID ? 1 : (pSurfels->surfelMap[iPt] == GID ? 0 : 2));

			fprintf(fpDebugPts, "%d\t%f\t%f\t%f\t%d\n", iPt, pPt->P[0], pPt->P[1], pPt->P[2], type);

			pPtIdx = pPtIdx->pNext;
		}

		fclose(fpDebugPts);
	}
}

void PlanarSurfelDetector::SaveIdxArray(
	FILE *fp,
	Array<int> &Array)
{
	int i;

	for (i = 0; i < Array.n; i++)
		fprintf(fp, "%d\n", Array.Element[i]);
}
#endif

#ifdef RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY
void PlanarSurfelDetector::GetEdgeMidPoint(
	Mesh *pMesh,
	int iEdge,
	float *P,
	float *N,
	float d,
	bool bPlaneIntersection)
{
	MeshEdge *pEdge = pMesh->EdgeArray.Element + iEdge;

	float *P1 = pMesh->NodeArray.Element[pEdge->iVertex[0]].P;
	float *P2 = pMesh->NodeArray.Element[pEdge->iVertex[1]].P;
	
	float dP[3];

	RVLDIF3VECTORS(P2, P1, dP);

	float s = (bPlaneIntersection ? (d - RVLDOTPRODUCT3(N, P1)) / RVLDOTPRODUCT3(N, dP) : 0.5f);
	
	RVLSCALE3VECTOR(dP, s, dP);

	RVLSUM3VECTORS(P1, dP, P);
}

bool PlanarSurfelDetector::InsideGRegion(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int WID,
	int GID,
	int BID,
	float *N,
	float d,	
	int iEdgeStart_,
	int sideStart,
	int iEdgeEnd_,
	float *PEnd,
	float *dP,
	Array<int> &initCut,
	Array<int> &lineCut)
{
#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	bool bDebug = (WID == debugDefineBoundaryiSurfel && BID == debugDefineBoundaryiSurfel_);
#endif

	int iEdgeStart = initCut.Element[iEdgeStart_];
	int iEdgeEnd = initCut.Element[iEdgeEnd_];

	MeshEdge *pEdge;

	if (iEdgeEnd_ == iEdgeStart_ + 1 || (edgeFlags[iEdgeStart] & edgeFlags[iEdgeEnd] & RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_INTERSECTION) != 0)
	{
		for (int i = iEdgeStart_ + 1; i <= iEdgeEnd_; i++)
		{
			edgeFlags[initCut.Element[i]] |= RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
			if (bDebug)
			{
				pEdge = pMesh->EdgeArray.Element + initCut.Element[i];

				fprintf(fpDebugEdges, "%d\t%d\t%d\t7\n", pEdge->idx, pEdge->iVertex[0], pEdge->iVertex[1]);
				fprintf(fpDebugEdges, "%d\t%d\t%d\t7\n", pEdge->idx, pEdge->iVertex[1], pEdge->iVertex[0]);

				fflush(fpDebugEdges);
			}
#endif
		}			
	}
	else
	{
		MeshEdgePtr *pEdgePtr0 = pMesh->EdgeArray.Element[iEdgeStart].pVertexEdgePtr[(sideStart >= 0 ? sideStart : 0)];

		MeshEdgePtr *pEdgePtr = pEdgePtr0;

		pEdge = pEdgePtr->pEdge;

		int i = 0;

#ifdef RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY_2
		float dist2Prev = RVLDOTPRODUCT3(dP, dP);

		float P[3], dP_[3];
		float dist2;
#endif
		int side;
		MeshEdge *pEdge_;
		MeshEdgePtr *pEdgePtr_, *pNextEdgePtr;
		int iPt, iNextPt;
		int side_;
		int ID;
		QList<MeshEdgePtr> *pEdgeList;
		unsigned int cutCost;
		int side1, side2;
		float *P1, *P2;

		while (pEdge->idx != iEdgeEnd)
		{
			cutCost = cutCostMap[pEdge->idx];

			pNextEdgePtr = NULL;

			if (pEdgePtr == pEdgePtr0 && sideStart >= 0)
				side1 = side2 = sideStart;
			else
			{
				side1 = 0;
				side2 = 1;
			}

			for (side = side1; side <= side2; side++)
			{
				pEdgePtr = pEdge->pVertexEdgePtr[side];

				iPt = pEdge->iVertex[side];

				pEdgePtr_ = pEdgePtr;

				RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr_, side_, pSurfels->surfelMap, iNextPt, pEdge_, ID, WID, GID, BID);

				pNextEdgePtr = NULL;

				P1 = pMesh->NodeArray.Element[iPt].P;

				while (pEdgePtr_ != pEdgePtr)
				{
					P2 = pMesh->NodeArray.Element[iNextPt].P;

					if (pSurfels->surfelMap[iPt] == GID || pSurfels->surfelMap[iNextPt] == GID)
					{
						if (RVLPLANARSURFELDETECTOR_ON_LINE_CUT(N, d, P1, P2))
						{
#ifdef RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY_2
							GetEdgeMidPoint(pMesh, pEdge_->idx, P, N, d);

							RVLDIF3VECTORS(PEnd, P, dP_);

							dist2 = RVLDOTPRODUCT3(dP_, dP_);
							
							if (dist2 < dist2Prev)
#else
							if (cutCostMap[pEdge_->idx] < cutCost)
#endif
							{
#ifdef RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY_2
								dist2Prev = dist2;
#endif

								pNextEdgePtr = pEdgePtr_;

								break;
							}
						}
					}

					iPt = iNextPt;

					P1 = P2;

					RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr_, side_, pSurfels->surfelMap, iNextPt, pEdge_, ID, WID, GID, BID);
				}

				if (pNextEdgePtr)
					break;
			}

			if (pNextEdgePtr == NULL)
				return false;

			pEdgePtr = pNextEdgePtr;

			pEdge = pEdgePtr->pEdge;

			lineCut.Element[i++] = pEdge->idx;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
			if (bDebug)
			{
				fprintf(fpDebugEdges, "%d\t%d\t%d\t7\n", pEdge->idx, pEdge->iVertex[0], pEdge->iVertex[1]);
				fprintf(fpDebugEdges, "%d\t%d\t%d\t7\n", pEdge->idx, pEdge->iVertex[1], pEdge->iVertex[0]);

				fflush(fpDebugEdges);
			}
#endif
		}

		lineCut.n = i;

		for (i = 0; i < lineCut.n; i++)
			edgeFlags[lineCut.Element[i]] |= RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT;
	}

	return true;
}
#endif

// Input:  pMesh - mesh,
//         pSurfels - surfel graph,
//         data - data structure with region growing parameters,
//         iSurfel - initial W-surfel idx.,
//         iSurfel_ - initial B-surfel idx.,
//         G - array with indices of initial (seed) points of G-region.
//
// Output: G - array with indices of G-region points,
//         GBnd - array of indices of G-region boundary points,
//         WBnd - array of indices of W-region boundary points,
//         The values of pSurfels->surfelMap corresponding to the points of G-region are set to the total number of mesh vertices.
//         map (member variable) - at the end of the execution of GRegion(), all elements corresponding to the G-region 
//                                 are set to the index of the closest connected component of W-region (starting from 1).
//         distanceMap (member variable) - at the end of the execution of GRegion(), each element corresponding to the G-region 
//                                         has the value representing the distance to the closest W-region point, 
//                                         where the points used to connect the parts of W-region into a single connected component 
//                                         are ignored by this distance transform.
//
// For a given mesh pMesh segmented to surfels pSurfels, this function determines the G-region between two neighboring surfels,
// W-surfel identifed by index iSurfel and B-surfel identified by index iSurfel_, by expanding B-surfel into W-surfel.
// The obtained G-region is stored in array G and its boundary is stored in array GBnd.
// Arrays G and WBnd are allocated in Mem2A, while GBnd is allocated in Mem2B.
//
// The function uses the following temporary maps (member variables of PlanarSurfelDetector):
//         map - at the beginning of the execution of GRegion(), all elements must be set to -1.
//         distanceMap - at the beginning of the execution of GRegion(), all elements must be set to 0xffffffff.

bool PlanarSurfelDetector::GRegion(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	PlanarSurfelDetectorRegionGrowingData &data,
	int iSurfel,
	int iSurfel_,
	Array<int> &G,
	Array<int> &GBnd,
	Array<int> &WBnd,
	bool *&bPrevW,
	bool bGtoW)
{
#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	//bool bDebug = (iSurfel == 0);
	bool bDebug = (iSurfel == debugDefineBoundaryiSurfel && iSurfel_ == debugDefineBoundaryiSurfel_);

	if (bDebug)
		int debug = 0;
#endif

	//if (iSurfel == 8 && iSurfel_ == 44)
	//	int debug = 0;

	CRVLMem *pMem2A = &(Mem2A);

	int nMeshPts = pMesh->NodeArray.n;

	int *iPtBuff = G.Element;

	/// B attacks iSurfel. G <- the regions of iSurfel conquered by B. W <- iSurfel \ G.

	int *iGPt = iPtBuff;

	int *piPtFetch = iGPt;

	int *piPtPut = iPtBuff + G.n;

	int *piPt;

	for (piPt = iGPt; piPt < piPtPut; piPt++)
		map[*piPt] = 0;

	//int *iBBndPt = iPtBuff;

	//int *iGPt = iPtBuff + nBBndPts;

	//int *piPtFetch = iBBndPt;

	//int *piPtPut = iGPt;	

	CRVLMem *pMem2B = &Mem2B;

	int *iGBndPt;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2B, int, nMeshPts, iGBndPt);

	int *piGBndPtArrayEnd = iGBndPt;

	data.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK;
	data.iAttackedSurfel = iSurfel;
	data.iSurfel = 0;

	float kNormal2 = data.kNormal2;
	data.kNormal2 = 0.0f;

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	data.pTemplate = pSurfel_;

	// iGPt <- array of indices of points in G-region
	// iGBndPt <- array of indices of boundary points of G-region

	int *piGPtArrayEnd = RegionGrowing3<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &data, piPtFetch, piPtPut,
		piGBndPtArrayEnd);

	data.kNormal2 = kNormal2;

	int nG = piGPtArrayEnd - iGPt;	// nG <- total no. of points in G-region

	RVLMEM_SET_FREE(pMem2A, piGPtArrayEnd);

	RVLMEM_SET_FREE(pMem2B, piGBndPtArrayEnd);

	// iBoundaryGEnd = piBoundaryGEnd -  iBoundaryPtBuff;	// documentation

	//#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
	//	debugPtArray.Element = iPtBuff;	
	//
	//	int *piPtDebug = iPtBuff;
	//
	//	for (int *piPt = iBoundaryPtBuff; piPt < piBoundaryGEnd; piPt++)
	//		if (pSurfels->surfelMap[*piPt] == iSurfel)
	//			*(piPtDebug++) = *piPt;
	//
	//	debugPtArray.n = piPtDebug - debugPtArray.Element;
	//#endif

	/// Detect connected components of W.
	/// nWCC <- total no. of connected components.
	/// For each boundary point i of every connected component j of W-region, where j = 0, 1, ..., nWCC-1
	///     map[i] <- j
	///     distanceMap[i] <- 0
	/// end for.

	// For all vertices i in G-region pSurfels->surfelMap[i] <- nSurfels 

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, bool, nG, bPrevW);

	bool *pbPrevW = bPrevW;

	int nSurfels = pMesh->NodeArray.n;

	int iPt, iPt_;

	if (bGtoW)
	{
		//QList<QLIST::Index2> *pW = &(pSurfel->PtList);
		//QList<QLIST::Index2> *pB = &(pSurfel_->PtList);

		//QLIST::Index2 *pPtIdx;

		for (piPt = iGPt; piPt < piGPtArrayEnd; piPt++, pbPrevW++)
		{
			iPt = *piPt;

			//if (pSurfels->surfelMap[iPt] == nSurfels)
			//{
			//	pPtIdx = pSurfels->PtMem + iPt;

			//	RVLQLIST_MOVE_ENTRY2(pB, pW, pPtIdx, QLIST::Index2);
			//}

			*pbPrevW = true;

			pSurfels->surfelMap[iPt] = nSurfels;
		}
	}
	else
	{
		for (piPt = iGPt; piPt < piGPtArrayEnd; piPt++, pbPrevW++)
		{
			iPt = *piPt;

			pSurfels->surfelMap[iPt] = nSurfels;
		}
	}

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);

	if (bDebug)
	{
		FILE *fpSurfels = fopen("C:\\RVL\\Debug\\PSDSurfels.txt", "w");

		fprintf(fpSurfels, "%f\t%f\t%f\t%f\t%f\t%f\n", pSurfel->P[0], pSurfel->P[1], pSurfel->P[2], pSurfel->N[0], pSurfel->N[1], pSurfel->N[2]);
		fprintf(fpSurfels, "%f\t%f\t%f\t%f\t%f\t%f\n", pSurfel_->P[0], pSurfel_->P[1], pSurfel_->P[2], pSurfel_->N[0], pSurfel_->N[1], pSurfel_->N[2]);

		fclose(fpSurfels);
	}
#endif

	// 

	int nWCC = 0;

	//int *iWCCPtArray = piGEnd;
	int *iWCCPt;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, nMeshPts, iWCCPt);

	int *iWCCPtArrayEnd = iWCCPt;

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
	debugState = 0;
#endif

	MeshEdgePtr *pEdgePtr;
	MeshEdge *pEdge;

	for (piPt = iGBndPt; piPt < piGBndPtArrayEnd; piPt++)
	{
		iPt = *piPt;

		if (pSurfels->surfelMap[iPt] == nSurfels)	// Is this condition necessary ?
		{
			// Find a point iPt_, which is a neighbor of iPt, belongs to W-region and doesn't belong to any already detected connected component of W-region.
			// This point is used as the seed for new connected component of W-region.

			pEdgePtr = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

				if (pSurfels->surfelMap[iPt_] == iSurfel)
					if (map[iPt_] < 0)
					{
						// iPt_ is the seed for new connected component of W-region.

						nWCC++;

						ConnectedComponent(pMesh, pSurfels, iPt_, iSurfel, nWCC, iWCCPtArrayEnd, map, distanceMap);
					}

				pEdgePtr = pEdgePtr->pNext;
			}
		}
	}

	///

	if (nWCC > 0)	// Check if there are connected components of W-region. If nWCC = 0, then W-region is completely covered by G-region.
	{
		// iWCC = iWCCPtArray - iPtBuff;		// documentation
		//int iWCCPtArrayEnd = iWCCPtArrayEnd - iPtBuff;

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
		debugPtArray.Element = iWCCPtArray;
		debugPtArray.n = iWCCPtArrayEnd - iWCCPtArray;

		nWCC = 1;
#endif

		/// If there are multiple connected components of W-region, then connect them into a single connected component.

		int nDistanceMatrixElements = nWCC * nWCC;

		unsigned int *distanceMatrix = new unsigned int[nDistanceMatrixElements];

		memset(distanceMatrix, 0xff, nDistanceMatrixElements * sizeof(unsigned int));

		MeshEdge **edgeMatrix = new MeshEdge *[nDistanceMatrixElements];

		PSD::DistanceComputationData distCompData;

		distCompData.distanceMap = distanceMap;
		distCompData.distanceMatrix = distanceMatrix;
		distCompData.edgeMatrix = edgeMatrix;
		distCompData.map = map;
		distCompData.nRegions = nWCC;

		// Compute distances between the connected components of W-region.
		// For each point of G-region, the corresponding element of map is set to the index of the closest connected component of W-region 
		// and the corresponding element of distanceMap is set to the distance to this connected component.

		piPtFetch = iWCCPt;

		piPtPut = iWCCPtArrayEnd;

		int *piPtDistanceBuffEnd = RegionGrowing<Mesh, Point, MeshEdge, MeshEdgePtr, PSD::DistanceComputationData, PSD::DistanceOperation>(pMesh, &distCompData, piPtFetch, piPtPut);

		RVLMEM_SET_FREE(pMem2A, piPtDistanceBuffEnd);

		if (nWCC > 1)
		{
			// Compute the minimum spanning tree of the connected components of W.

			int *tree = new int[nWCC];

			MinimumSpanningTree(distanceMatrix, nWCC, tree);

			// Connect W-region into a single connected component.

			MeshEdge **edgeArray = edgeMatrix + nWCC;
			int iWCC;

			for (iWCC = 1; iWCC < nWCC; iWCC++, edgeArray += nWCC)
			{
				pEdge = edgeArray[tree[iWCC]];

				Connect(pMesh, pEdge, iSurfel, map, distanceMap, pSurfels->surfelMap);
			}

			delete[] tree;

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
			SaveWGB(pMesh, pSurfels, iSurfel, nSurfels, iSurfel_);
#endif
		}	// if (nWCC > 1)

		delete[] distanceMatrix;
		delete[] edgeMatrix;
	}	// if(nWCC > 0)

	G.Element = iGPt;
	G.n = nG;
	GBnd.Element = iGBndPt;
	GBnd.n = piGBndPtArrayEnd - iGBndPt;
	WBnd.Element = iWCCPt;
	WBnd.n = iWCCPtArrayEnd - iWCCPt;

	return (nWCC > 0);
}

// Input:  pMesh - mesh,
//         pSurfels - surfel graph,
//         WID - W-surfel ID (attacked surfel),
//         GID - G-region ID,
//         BID - B-surfel ID (attacking surfel),
//         BBnd - array of indices of boundary points of B-surfel.
//
// Output: G - array of indices of points on the boundary of W-surfel, which have at least one neighbor in B-surfel and lie on the B-surfel plane.

void PlanarSurfelDetector::GetAttackSeed(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int WID,
	int GID,
	int BID,
	Array<int> &BBnd,
	Array<int> &G)
{
	PlanarSurfelDetectorRegionGrowingData data = regionGrowingData;

	Surfel *pSurfel_ = pSurfels->NodeArray.Element + BID;

	data.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK;
	data.iAttackedSurfel = WID;
	data.iSurfel = 0;
	data.pTemplate = pSurfel_;
	data.surfelMap = pSurfels->surfelMap;
	data.buffer = map;
	data.bLimitedDepthUnconstrainedRG = false;

	int *piGPt = G.Element;

	int i;
	int iBPt, iWPt;
	Point *pBPt;
	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr;

	for (i = 0; i < BBnd.n; i++)
	{
		iBPt = BBnd.Element[i];

		pBPt = pMesh->NodeArray.Element + iBPt;

		pEdgePtr = pBPt->EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iBPt, pEdgePtr, pEdge, iWPt);

			if (PSD::RegionGrowingOperation(iWPt, iBPt, pEdge, pMesh, &data) == 1)
				*(piGPt++) = iWPt;

			pEdgePtr = pEdgePtr->pNext;
		}
	}

	G.n = piGPt - G.Element;
}

void PlanarSurfelDetector::Boundaries(
	Mesh *pMesh,
	SurfelGraph *pSurfels)
{
	QList<QLIST::Entry<Array<MeshEdgePtr *>>> *pBoundaryList = &(pSurfels->BoundaryList);

	RVLQLIST_INIT(pBoundaryList);

	MeshEdgePtr **ppEdgePtr = pSurfels->BndMem;

	//int iDebug = 0;

	//float edgeClassDepthOccupancyBinSize = edgeClassDepthDiscontinuityThr;

	//int uMax = pMesh->width - 1;
	//int vMax = pMesh->height - 1;

	int k = 2 * edgeClassHalfWinSize;

	//MeshEdgePtr *pEdgePtr_;
	//bool bForeground;
	int i;
	int iPt, iPt_, iPt__;
	QLIST::Entry<Array<MeshEdgePtr *>> *pBoundary;
	Point *pPt, *pPt_, *pPt__;
	MeshEdgePtr *pEdgePtr;
	int u, v;
	int u0, v0;
	//int uMin_, uMax_, vMin_, vMax_;
	float d, d0, dd;
	//int iBin, iBin_, iBin__;

	int e, de, p, q, q2, dq;
	int s11, s12, s21, s22, s11_, s12_;

	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		pPt = pMesh->NodeArray.Element + iPt;

		if (!pPt->bValid)
			continue;

		if (pPt->bBoundary)
		{
			if (pSurfels->edgeMap[iPt] < 0)
			{
				RVLMEM_ALLOC_STRUCT(pMem, QLIST::Entry<Array<MeshEdgePtr *>>, pBoundary);

				RVLQLIST_ADD_ENTRY(pBoundaryList, pBoundary);

				pBoundary->data.Element = ppEdgePtr;

				iPt_ = iPt;

				pPt_ = pPt;

				while (pPt_->bBoundary && pSurfels->edgeMap[iPt_] < 0)
				{				
					//float PDebug[3];
					//RVLSET3VECTOR(PDebug, 0.072, -0.359, 1.737);
					//float dPDebug[3];
					//RVLDIF3VECTORS(pPt_->P, PDebug, dPDebug);
					//float debug = sqrt(RVLDOTPRODUCT3(dPDebug, dPDebug));
					//if (RVLABS(debug) < 0.05f)
					//	debug = 0.0f;

					pSurfels->edgeMap[iPt_] = 0;

					pEdgePtr = pPt_->EdgeList.pFirst;

//#ifndef RVLVERSION_171125
//					pEdgePtr_ = pEdgePtr;
//
//					while (pEdgePtr_)
//					{
//						pEdgePtr = pEdgePtr_;
//
//						pEdgePtr_ = pEdgePtr_->pNext;
//					}
//#endif

					*(ppEdgePtr++) = pEdgePtr;

					// Classify edge point to foreground/background.

					if (pMesh->bOrganizedPC)
					{
						d0 = pPt_->P[2];

						if (d0 <= maxRange)
						{
							u0 = iPt_ % pMesh->width;
							v0 = iPt_ / pMesh->width;

							if (IsInRect<int>(u0, v0, edgeClassImageBoundary))
							{
								if(bEdgeClassForegroundIsDefault)
									pPt_->flags |= RVLMESH_POINT_FLAG_FOREGROUND;

								s11 = 1;
								s12 = 0;
								s21 = 0;
								s22 = 1;

								for (i = 0; i < 4; i++)
								{
									for (q2 = -edgeClassHalfWinSize; q2 < edgeClassHalfWinSize; q2++)
									{
										q = 0;

										dq = (q2 >= 0 ? 1 : -1);

										de = 2 * dq * q2;

										e = de - edgeClassHalfWinSize;

										for (p = 1; p <= edgeClassHalfWinSize; p++)
										{
											if (e > 0)
											{
												q += dq;

												e -= k;
											}

											e += de;

											u = u0 + s11 * p + s12 * q;
											v = v0 + s21 * p + s22 * q;

											if (u < 0)
												break;

											if (u >= pMesh->width)
												break;

											if (v < 0)
												break;

											if (v >= pMesh->height)
												break;
	
											iPt__ = u + v * pMesh->width;
											pPt__ = pMesh->NodeArray.Element + iPt__;
											d = pPt__->P[2];

											if (d > 0.0f)
											{
												dd = d - d0;

												if (bEdgeClassForegroundIsDefault)
												{
													if (dd <= -edgeClassDepthDiscontinuityThr)
														pPt_->flags |= RVLMESH_POINT_FLAG_BACKGROUND;
												}
												else
												{
													if (dd >= edgeClassDepthDiscontinuityThr)
														pPt_->flags |= RVLMESH_POINT_FLAG_FOREGROUND;
													else if (dd <= -edgeClassDepthDiscontinuityThr)
														pPt_->flags |= RVLMESH_POINT_FLAG_BACKGROUND;
												}

												p = edgeClassHalfWinSize;
											}
										}	// for (p = 1; p <= edgeClassHalfWinSize; p++)
									}	// for (q2 = -edgeClassHalfWinSize; q2 < edgeClassHalfWinSize; q2++)

									s11_ = -s21;
									s12_ = -s22;
									s21 = s11;
									s22 = s12;
									s11 = s11_;
									s12 = s12_;
								}	// for(i = 0; i < 4; i++)
	#ifdef NEVER
								uMin_ = u0 - edgeClassHalfWinSize;
								uMax_ = u0 + edgeClassHalfWinSize;
								vMin_ = v0 - edgeClassHalfWinSize;
								vMax_ = v0 + edgeClassHalfWinSize;

								RVLCROPRECT(0, uMax, 0, vMax, uMin_, uMax_, vMin_, vMax_);

								iEdgeClassDepthOccupancyBin.n = 0;

								for (v = vMin_; v <= vMax_; v++)
								{
									for (u = uMin_; u <= uMax_; u++)
									{
										iPt__ = u + v * pMesh->width;
										pPt__ = pMesh->NodeArray.Element + iPt__;
										d = pPt__->P[2];

										if (d > 0.0f && d <= maxRange)
										{
											iBin = (int)(d / edgeClassDepthOccupancyBinSize);

											if (edgeClassDepthOccupancy[iBin].min < 1e-10)
											{
												edgeClassDepthOccupancy[iBin].min = edgeClassDepthOccupancy[iBin].max = d;

												iEdgeClassDepthOccupancyBin.Element[iEdgeClassDepthOccupancyBin.n++] = iBin;
											}
											else
											{
												if (d < edgeClassDepthOccupancy[iBin].min)
													edgeClassDepthOccupancy[iBin].min = d;
												else if (d > edgeClassDepthOccupancy[iBin].max)
													edgeClassDepthOccupancy[iBin].max = d;
											}
										}
									}
								}

								d = pPt__->P[2];

								bForeground = true;

								iBin = (int)(d / edgeClassDepthOccupancyBinSize);

								iBin_ = iBin;

								while (bForeground)
								{
									iBin__ = iBin_;

									iBin_--;

									if (edgeClassDepthOccupancy[iBin__].min < 1e-10)
										break;

									if (edgeClassDepthOccupancy[iBin_].max - edgeClassDepthOccupancy[iBin__].min > edgeClassDepthDiscontinuityThr)
										bForeground = false;
								}

								if (bForeground)
								{
									iBin_ = iBin;

									while (bForeground)
									{
										// ... not completed
									}
								}
	#endif
							}
						}
					}	// if (pMesh->bOrganizedPC)

					///

					iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

					pPt_ = pMesh->NodeArray.Element + iPt_;
				}	// while (pPt_->bBoundary && pSurfels->edgeMap[iPt_] < 0)

				pBoundary->data.n = ppEdgePtr - pBoundary->data.Element;

				//iDebug++;
			}	// if (pSurfels->edgeMap[iPt] < 0)
		}	// if (pPt->bBoundary)
	}	// for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)

	MeshEdgePtr **pBndMemEnd = ppEdgePtr;

	for (ppEdgePtr = pSurfels->BndMem; ppEdgePtr < pBndMemEnd; ppEdgePtr++)
	{
		pEdgePtr = *ppEdgePtr;

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

		pSurfels->edgeMap[iPt] = -1;
	}
}

void PlanarSurfelDetector::EdgeFetures(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	QList<SURFEL::Edge> *pSEdgeList,
	int &nSEdges)
{
	int iEdgeFeature = pSurfels->NodeArray.n;

	int nOcclusionEdges = 1;

	QLIST::Entry<Array<MeshEdgePtr *>> *pBoundary = pSurfels->BoundaryList.pFirst;

	while (pBoundary)
	{
		iEdgeFeature += CreateEdgeFeatures(pMesh, pSurfels, &(pBoundary->data), iEdgeFeature, nOcclusionEdges, pSEdgeList, 
			nSEdges, pMem);

		pBoundary = pBoundary->pNext;
	}

	pSurfels->NodeArray.n = iEdgeFeature;
}

int PlanarSurfelDetector::CreateEdgeFeatures(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Array<MeshEdgePtr *> *pBoundary,
	int iNewFeature,
	int &nOcclusionEdges,
	QList<SURFEL::Edge> *pSEdgeList,
	int &nSEdges,
	CRVLMem *pMem)
{
	if (pBoundary->n < minEdgeFeatureSize)
		return 0;

#ifdef NEVER
	// Determine the boundary bounding box.

	MeshEdgePtr *pEdgePtr = pBoundary->Element[0];

	int iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

	Point *pPt = pMesh->NodeArray.Element + iPt;

	float *P = pPt->P;

	float min_[3], max_[3];
	int i;

	for (i = 0; i < 3; i++)
		min_[i] = max_[i] = P[i];

	int iPointEdgeMin[3] = { 0, 0, 0 };
	int iPointEdgeMax[3] = { 0, 0, 0 };

	int iPointEdge;

	for (iPointEdge = 1; iPointEdge < pBoundary->n; iPointEdge++)
	{
		pEdgePtr = pBoundary->Element[iPointEdge];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

		pPt = pMesh->NodeArray.Element + iPt;

		P = pPt->P;

		for (i = 0; i < 3; i++)
		{
			if (P[i] < min_[i])
			{ 
				min_[i] = P[i];

				iPointEdgeMin[i] = iPointEdge;
			}
			else if(P[i] > max_[i])
			{
				max_[i] = P[i];

				iPointEdgeMax[i] = iPointEdge;
			}
		}
	}

	// Identify the largest size of the bounding box.

	float maxSize = 0.0f;

	int iMaxSize = -1;

	float size;

	for (i = 0; i < 3; i++)
	{
		size = max_[i] - min_[i];

		if (size > maxSize)
		{
			maxSize = size;

			iMaxSize = i;
		}
	}

	if (iMaxSize < 0)
		return 0;

	// Define the initial segment endpoints.

	int iPointEdge1, iPointEdge2;

	if (iPointEdgeMin[iMaxSize] < iPointEdgeMax[iMaxSize])
	{
		iPointEdge1 = iPointEdgeMin[iMaxSize];
		iPointEdge2 = iPointEdgeMax[iMaxSize];
	}
	else
	{
		iPointEdge2 = iPointEdgeMin[iMaxSize];
		iPointEdge1 = iPointEdgeMax[iMaxSize];
	}

	// bClosed <- Is the boundary a closed contour?

	//bool bClosed = false;

	//pEdgePtr = pBoundary->Element[pBoundary->n - 1];

	//iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

	//int iPt_;

	//pEdgePtr = pPt->EdgeList.pFirst;

	//while (pEdgePtr)
	//{
	//	iPt_ = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

	//	if (iPt_ == iPt)
	//	{
	//		bClosed = true;

	//		break;
	//	}

	//	pEdgePtr = pEdgePtr->pNext;
	//}

	bool bClosed = true;

	// Initialize the segmentation procedure.

	QList<QLIST::Index> segmentEndpointList;

	QList<QLIST::Index> *pSegmentEndpointList = &segmentEndpointList;

	RVLQLIST_INIT(pSegmentEndpointList);

	QLIST::Index *pSegmentEndpointMem = new QLIST::Index[pBoundary->n + 1];

	QLIST::Index *pSegmentEndpoint = pSegmentEndpointMem;

	if (!bClosed)
	{
		RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint);

		pSegmentEndpoint->Idx = 0;

		pSegmentEndpoint++;
	}

	RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint);

	pSegmentEndpoint->Idx = iPointEdge1;

	pSegmentEndpoint++;

	RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint);

	pSegmentEndpoint->Idx = iPointEdge2;

	pSegmentEndpoint++;

	if (bClosed)
	{
		RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint);

		pSegmentEndpoint->Idx = iPointEdge1;

		pSegmentEndpoint++;
	}
	else
	{
		RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint);

		pSegmentEndpoint->Idx = pBoundary->n - 1;

		pSegmentEndpoint++;
	}
#endif

	// Initialize the segmentation procedure.

	QList<QLIST::Index> segmentEndpointList;
	QList<QLIST::Index>* pSegmentEndpointList = &segmentEndpointList;
	RVLQLIST_INIT(pSegmentEndpointList);
	QLIST::Index* pSegmentEndpointMem = new QLIST::Index[pBoundary->n + 1];
	QLIST::Index* pSegmentEndpoint = pSegmentEndpointMem;
	RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint);
	pSegmentEndpoint->Idx = 0;
	pSegmentEndpoint++;
	RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint);
	pSegmentEndpoint->Idx = pBoundary->n - 1;
	pSegmentEndpoint++;
	QLIST::Index* pSegmentEndpoint1 = pSegmentEndpointList->pFirst;
	QLIST::Index* pSegmentEndpoint2 = pSegmentEndpoint1->pNext;

	/// Recursive segmentation of pBoundary into approximatelly linear clusters.

	int iNewFeature_ = iNewFeature;

	float R_[9];

	float *X = R_;
	float *Y = R_ + 3;
	float *Z = R_ + 6;

	float r2 = 0.0005f * (float)(pSurfels->edgeDepth);

	int iPointEdge3;
	Point *pPt1, *pPt2;
	float *P1, *P2, *P_, *V_;
	float dP[3], NE[3], V[3], Q[3];
	float dE, e, maxe, maxe_;
	float fTmp;
	float *N, *R;
#ifdef RVLVERSION_171125
	SURFEL::Edge *pSEdge;
	int nTmp;
	Array<MeshEdgePtr *> *pEdgePtArray;
	MeshEdgePtr **pEdgePtrPtrArray;
#endif
	Surfel *pEdgeFeature;
	float l, s;
	QList<SURFEL::EdgePtr> *pSEdgeList_;
	bool bPtProjectionOutOfLineSegment;
	//int iPointEdge_;
	int nForeground, nBackground;
	BYTE edgeClass;
	int iSurfel;
	Surfel *pSurfel;
	QList<QLIST::Index2> *pPtList, *pPtList_;
	QLIST::Index2 *pPtIdx;
	MeshEdgePtr* pEdgePtr;
	int iPt, iPointEdge;
	float* P;
	Point* pPt;

	while (pSegmentEndpoint2)
	{
		// P1 <- position vector of the point indexed by pSegmentEndpoint1->idx

		pEdgePtr = pBoundary->Element[pSegmentEndpoint1->Idx];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

		pPt1 = pMesh->NodeArray.Element + iPt;

		P1 = pPt1->P;

		// P2 <- position vector of the point indexed by pSegmentEndpoint2->idx

		pEdgePtr = pBoundary->Element[pSegmentEndpoint2->Idx];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

		pPt2 = pMesh->NodeArray.Element + iPt;

		P2 = pPt2->P;

		// dP <- P2 - P1

		RVLDIF3VECTORS(P2, P1, dP);

		if (bBottom)
			RVLSET3VECTOR(NE, 0.0f, 0.0f, -1.0f)
		else
		{
			// NE <- P1 x dP / || P1 x dP ||

			RVLCROSSPRODUCT3(P1, dP, NE);

			RVLNORM3(NE, fTmp);
		}

		// dE <- NE' * P1

		dE = RVLDOTPRODUCT3(NE, P1);

		// l <- || dP ||;

		l = sqrt(RVLDOTPRODUCT3(dP, dP));

		// V <- dP / || dP ||

		RVLSCALE3VECTOR2(dP, l, V);

		// iPointEdge3 <- index of the point from the segment between pSegmentEndpoint1->idx and pSegmentEndpoint2->idx, which is the most distant from the plane (NE, dE).
		// If all points of this segment are at distance eThr or closer, then iPointEdge3 <- -1.

		maxe = 1.0f;
		maxe_ = 0.0f;

		iPointEdge3 = -1;

		bPtProjectionOutOfLineSegment = false;

		iPointEdge = (pSegmentEndpoint1->Idx + 1) % pBoundary->n;

		while (iPointEdge != pSegmentEndpoint2->Idx)
		{
			pEdgePtr = pBoundary->Element[iPointEdge];

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

			pPt = pMesh->NodeArray.Element + iPt;

			P = pPt->P;

			RVLDIF3VECTORS(P, P1, Q);

			s = RVLDOTPRODUCT3(V, Q);

			if (s < 0.0f)
			{
				e = sqrt(RVLDOTPRODUCT3(Q, Q));

				if (e > maxe_)
				{
					maxe_ = e;

					iPointEdge3 = iPointEdge;
				}

				bPtProjectionOutOfLineSegment = true;
			}
			else if (s > l)
			{
				RVLDIF3VECTORS(P, P2, Q);

				e = sqrt(RVLDOTPRODUCT3(Q, Q));

				if (e > maxe_)
				{
					maxe_ = e;

					iPointEdge3 = iPointEdge;
				}

				bPtProjectionOutOfLineSegment = true;
			}
			else if (!bPtProjectionOutOfLineSegment)
			{
				e = RVLDOTPRODUCT3(NE, P) - dE;

				e = RVLABS(e) / maxEdgeFeatureConcavity;

				if (e > maxe)
				{
					maxe = e;

					iPointEdge3 = iPointEdge;
				}
			}

			iPointEdge = (iPointEdge + 1) % pBoundary->n;
		}	// for all boundary points between pSegmentEndpoint1->Idx and pSegmentEndpoint2->Idx

		if (iPointEdge3 >= 0)	// If there is a point in the interval [pSegmentEndpoint1->idx, pSegmentEndpoint2->idx], 
								// which is outside of the tolerance eThr from the plane (NE, dE)
		{
			// Insert an endpoint with index iPointEdge3 between pSegmentEndpoint1 and pSegmentEndpoint2.

			RVLQLIST_INSERT_ENTRY(pSegmentEndpointList, pSegmentEndpoint1, pSegmentEndpoint2, pSegmentEndpoint);

			pSegmentEndpoint->Idx = iPointEdge3;

			// pSegmentEndpoint2 <- pSegmentEndpoint

			pSegmentEndpoint2 = pSegmentEndpoint;

			pSegmentEndpoint++;
		}
		else // If all points in the interval [pSegmentEndpoint1->idx, pSegmentEndpoint2->idx] are within the tolerance eThr from the plane (NE, dE)
		{
			// Compute the number of foreground/background points.

			nForeground = nBackground = 0;

			iPointEdge = pSegmentEndpoint1->Idx;

			while (iPointEdge != pSegmentEndpoint2->Idx)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				edgeClass = (pPt->flags & RVLMESH_POINT_FLAG_EDGE_CLASS);

				if (edgeClass == RVLMESH_POINT_FLAG_FOREGROUND)
					nForeground++;
				else if (edgeClass == RVLMESH_POINT_FLAG_BACKGROUND)
					nBackground++;

				iPointEdge = (iPointEdge + 1) % pBoundary->n;
			}

			if (nForeground > 0 && nForeground > nBackground || bBottom)
			{
				// Create new edge feature.

				pEdgeFeature = pSurfels->NodeArray.Element + iNewFeature_;

				pEdgeFeature->bEdge = true;

				pEdgeFeature->flags = RVLSURFEL_FLAG_RF;

				N = pEdgeFeature->N;

				RVLCOPY3VECTOR(NE, N);

				pEdgeFeature->d = dE;

				P_ = pEdgeFeature->P;

				RVLCOPY3VECTOR(P1, P_);

				V_ = pEdgeFeature->V;

				RVLCOPY3VECTOR(V, V_);

				pEdgeFeature->physicalSize = l;

				pPtList = &(pEdgeFeature->PtList);

				RVLQLIST_INIT(pPtList);

				// Assign points to the new edge feature.
				// Connect the new edge feature to the neighboring surfels.

				pSEdgeList_ = &(pEdgeFeature->EdgeList);

				RVLQLIST_INIT(pSEdgeList_);

#ifdef RVLVERSION_171125
				iPointEdge = pSegmentEndpoint1->Idx;

				while (iPointEdge != pSegmentEndpoint2->Idx)
				{
					pEdgePtr = pBoundary->Element[iPointEdge];

					iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

					pSurfels->edgeMap[iPt] = iNewFeature_;

					iSurfel = pSurfels->surfelMap[iPt];

					if (iSurfel >= 0 && iSurfel < pMesh->NodeArray.n)
					{
						pSurfel = pSurfels->NodeArray.Element + iSurfel;

						if (pSurfel->size > 1)
						{
							if (pSurfels->neighborEdge[iSurfel] == NULL)
							{
								pSEdge = ConnectNodes<Surfel, SURFEL::Edge, SURFEL::EdgePtr>(iSurfel, iNewFeature_, pSurfels->NodeArray, pMem);

								RVLQLIST_ADD_ENTRY(pSEdgeList, pSEdge);

								nSEdges++;

								pSurfels->neighborEdge[iSurfel] = pSEdge;
							}
						}
					}

					iPointEdge = (iPointEdge + 1) % pBoundary->n;
				}

				SURFEL::EdgePtr *pSEdgePtr = pEdgeFeature->EdgeList.pFirst;

				while (pSEdgePtr)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNewFeature_, pSEdgePtr, pSEdge, iSurfel);

					pSurfels->neighborEdge[iSurfel] = NULL;

					pSEdgePtr = pSEdgePtr->pNext;
				}

				// Determine boundary.

				RVLMEM_ALLOC_STRUCT(pMem, Array<MeshEdgePtr *>, pEdgePtArray);

				pEdgeFeature->BoundaryArray.n = 1;
				pEdgeFeature->BoundaryArray.Element = pEdgePtArray;

				if (pSegmentEndpoint2->Idx >= pSegmentEndpoint1->Idx)
				{
					pEdgePtArray->Element = pBoundary->Element + pSegmentEndpoint1->Idx;
					pEdgePtArray->n = pSegmentEndpoint2->Idx - pSegmentEndpoint1->Idx;
				}
				else
				{
					pEdgePtArray->n = pSegmentEndpoint2->Idx - pSegmentEndpoint1->Idx + pBoundary->n;

					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, MeshEdgePtr *, pEdgePtArray->n, pEdgePtrPtrArray);

					pEdgePtArray->Element = pEdgePtrPtrArray;

					nTmp = pBoundary->n - pSegmentEndpoint1->Idx;

					memcpy(pEdgePtrPtrArray, pBoundary->Element + pSegmentEndpoint1->Idx, nTmp * sizeof(MeshEdgePtr *));

					pEdgePtrPtrArray += nTmp;

					memcpy(pEdgePtrPtrArray, pBoundary->Element, pSegmentEndpoint2->Idx * sizeof(MeshEdgePtr *));
				}

				pEdgeFeature->size = pEdgePtArray->n * pSurfels->edgeDepth;
#else
				pEdgeFeature->size = 0;

				iPointEdge = pSegmentEndpoint1->Idx;

				while (iPointEdge != pSegmentEndpoint2->Idx)
				{
					pEdgePtr = pBoundary->Element[iPointEdge];

					iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

					pPtIdx = pSurfels->PtMem + iPt;

					iSurfel = pSurfels->surfelMap[iPt];

					pSurfels->surfelMap[iPt] = iNewFeature_;

					if (iSurfel >= 0)
					{
						pSurfel = pSurfels->NodeArray.Element + iSurfel;

						pPtList_ = &(pSurfel->PtList);

						RVLQLIST_MOVE_ENTRY2(pPtList_, pPtList, pPtIdx, QLIST::Index2);

						pSurfel->size--;
					}
					else
					{
						pPtIdx->Idx = iPt;

						RVLQLIST_ADD_ENTRY2(pPtList, pPtIdx);
					}


					pEdgeFeature->size++;

					iPointEdge = (iPointEdge + 1) % pBoundary->n;
				}

				pEdgeFeature->size *= pSurfels->edgeDepth;
#endif

				// Compute other edge feature parameters.

				R = pEdgeFeature->R;

				RVLCOPY3VECTOR(N, Z);
				RVLCOPY3VECTOR(V, X);
				RVLCROSSPRODUCT3(Z, X, Y);
				RVLCOPYMX3X3(R_, R);

				pEdgeFeature->r1 = 0.5f * l;
				pEdgeFeature->r2 = r2;

				//

				iNewFeature_++;
			}	// if (nForeground > 0 && nForeground > nBackground)
#ifdef RVLPLANARSURFELDETECTOR_OCCLUSION_EDGES
			else
			{
				nOcclusionEdges++;

				iPointEdge = pSegmentEndpoint1->Idx;

				while (iPointEdge != pSegmentEndpoint2->Idx)
				{
					pEdgePtr = pBoundary->Element[iPointEdge];

					iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

#ifdef RVLVERSION_171125
					pSurfels->edgeMap[iPt] = -nOcclusionEdges;
#else
					pSurfels->surfelMap[iPt] = -nOcclusionEdges;
#endif

					iPointEdge = (iPointEdge + 1) % pBoundary->n;
				}
			}
#endif

			// (pSegmentEndpoint1, pSegmentEndpoint2) <- (pSegmentEndpoint2, pSegmentEndpoint2->pNext)

			pSegmentEndpoint1 = pSegmentEndpoint2;
			pSegmentEndpoint2 = pSegmentEndpoint2->pNext;
		}	// If all points in the interval [pSegmentEndpoint1->idx, pSegmentEndpoint2->idx] are within the tolerance eThr from the plane (NE, dE)
	}	// while(pSegmentEndpoint2)

	delete[] pSegmentEndpointMem;

	return iNewFeature_ - iNewFeature;
}

#ifdef NEVER		
// Old version of edge feature detection. 
// Each edge feature is assigned to a single surfel - its parent feature.
// Edge features are detecting by followng surfel boundaries.

void PlanarSurfelDetector::EdgeFetures(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	QList<SURFEL::Edge> *pSEdgeList,
	int &nSEdges)
{
	int iEdgeFeature = pSurfels->NodeArray.n;

	int iSurfel, iBoundary, iPointEdge;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr;
	int iPt;
	Surfel *pSurfel;
	Point *pPt;
	int iStart;	

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			iStart = -1;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				if (iStart >= 0)
				{
					if (!pPt->bBoundary)
					{
						iEdgeFeature += CreateEdgeFeatures(pMesh, pSurfels, iSurfel, iBoundary, iStart, iPointEdge - 1, iEdgeFeature, pSEdgeList, nSEdges, pMem);

						iStart = -1;
					}						
				}
				else if (pPt->bBoundary)
					iStart = iPointEdge;
			}	// for each point-edge on the boundary contour

			if (iStart >= 0)
				iEdgeFeature += CreateEdgeFeatures(pMesh, pSurfels, iSurfel, iBoundary, iStart, iPointEdge, iEdgeFeature, pSEdgeList, nSEdges, pMem);
		}	// for each boundary contour
	}	// for every surfel

	pSurfels->NodeArray.n = iEdgeFeature;
}

int PlanarSurfelDetector::CreateEdgeFeatures(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel,
	int iBoundary,
	int iStart,
	int iEnd,
	int iNewFeature,
	QList<SURFEL::Edge> *pSEdgeList,
	int &nSEdges,
	CRVLMem *pMem)
{
	int segmentSize = iEnd - iStart;

	if (segmentSize < minEdgeFeatureSize)
		return 0;

	/// Recursive segmentation of the boundary segment between iStart and iEnd into approximatelly linear clusters.

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	float *NParent = pSurfel->N;

	Array<MeshEdgePtr *> *pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

	QList<QLIST::Index> segmentEndpointList;

	QList<QLIST::Index> *pSegmentEndpointList = &segmentEndpointList;

	RVLQLIST_INIT(pSegmentEndpointList);

	QLIST::Index *pSegmentEndpointMem = new QLIST::Index[segmentSize];

	QLIST::Index *pSegmentEndpoint = pSegmentEndpointMem;

	// pSegmentEndpoint1 <- index of the first point of the boundary segment.

	QLIST::Index *pSegmentEndpoint1 = pSegmentEndpoint;

	pSegmentEndpoint1->Idx = iStart;

	RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint1);

	pSegmentEndpoint++;

	// pSegmentEndpoint2 <- index of the last point of the boundary segment.

	QLIST::Index *pSegmentEndpoint2 = pSegmentEndpoint;

	pSegmentEndpoint2->Idx = iEnd - 1;

	RVLQLIST_ADD_ENTRY(pSegmentEndpointList, pSegmentEndpoint2);

	pSegmentEndpoint++;

	//

	int iNewFeature_ = iNewFeature;

	float eThr = 2.0f / kPlane;

	int iPointEdge, iPt, iPointEdge3;
	MeshEdgePtr *pEdgePtr;
	Point *pPt, *pPt1, *pPt2;
	float *P, *P1, *P2, *P_;
	float dP[3], NE[3], V[3], Q[3], P1_[3], P2_[3];
	float d, dE, e, maxe, maxe_;
	float fTmp;
	QLIST::Index *pSegmentEndpoint1_, *pSegmentEndpoint2_;
	float *N;
	SURFEL::Edge *pSEdge;
	Surfel *pEdgeFeature;
	float l, s;
	Array<MeshEdgePtr *> *pEdgePtArray;
	QList<SURFEL::EdgePtr> *pSEdgeList_;
	bool bPtProjectionOutOfLineSegment;

	while (true)
	{
		// P1 <- position vector of the point indexed by pSegmentEndpoint1->idx

		pEdgePtr = pBoundary->Element[pSegmentEndpoint1->Idx];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

		pPt1 = pMesh->NodeArray.Element + iPt;

		P1 = pPt1->P;

		// P2 <- position vector of the point indexed by pSegmentEndpoint2->idx

		pEdgePtr = pBoundary->Element[pSegmentEndpoint2->Idx];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

		pPt2 = pMesh->NodeArray.Element + iPt;
		
		P2 = pPt2->P;

		// P1_ <- projection of P1 onto the supporting plane of iSurfel.

		d = RVLDOTPRODUCT3(NParent, P1) - pSurfel->d;

		RVLSCALE3VECTOR(NParent, d, V);

		RVLDIF3VECTORS(P1, V, P1_);

		// P2_ <- projection of P2 onto the supporting plane of iSurfel.

		d = RVLDOTPRODUCT3(NParent, P2) - pSurfel->d;

		RVLSCALE3VECTOR(NParent, d, V);

		RVLDIF3VECTORS(P2, V, P2_);

		// dP <- P2_ - P1_

		RVLDIF3VECTORS(P2_, P1_, dP);

		// NE <- P1_ x dP / || P1_ x dP ||

		RVLCROSSPRODUCT3(P1_, dP, NE);

		RVLNORM3(NE, fTmp);

		// dE <- NE' * P1

		dE = RVLDOTPRODUCT3(NE, P1_);

		// l <- || dP ||;

		l = sqrt(RVLDOTPRODUCT3(dP, dP));

		// V <- dP / || dP ||

		RVLSCALE3VECTOR2(dP, l, V)

		// iPointEdge3 <- index of the point from the segment between pSegmentEndpoint1->idx and pSegmentEndpoint2->idx, which is the most distant from the plane (NE, dE).
		// If all points of this segment are at distance eThr or closer, then iPointEdge3 <- -1.

		maxe = 1.0f;
		maxe_ = 0.0f;

		iPointEdge3 = -1;

		bPtProjectionOutOfLineSegment = false;

		for (iPointEdge = pSegmentEndpoint1->Idx + 1; iPointEdge < pSegmentEndpoint2->Idx; iPointEdge++)
		{
			pEdgePtr = pBoundary->Element[iPointEdge];

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

			pPt = pMesh->NodeArray.Element + iPt;

			P = pPt->P;

			RVLDIF3VECTORS(P, P1_, Q);

			s = RVLDOTPRODUCT3(V, Q);

			if (s < 0.0f)
			{
				e = sqrt(RVLDOTPRODUCT3(Q, Q));

				if (e > maxe_)
				{
					maxe_ = e;

					iPointEdge3 = iPointEdge;
				}
					
				bPtProjectionOutOfLineSegment = true;
			}
			else if (s > l)
			{
				RVLDIF3VECTORS(P, P2_, Q);

				e = sqrt(RVLDOTPRODUCT3(Q, Q));

				if (e > maxe_)
				{
					maxe_ = e;

					iPointEdge3 = iPointEdge;
				}

				bPtProjectionOutOfLineSegment = true;
			}
			else if (!bPtProjectionOutOfLineSegment)
			{
				e = RVLDOTPRODUCT3(NE, P) - dE;

				e = e / (e >= 0 ? eThr : -maxEdgeFeatureConcavity);

				if (e > maxe)
				{
					maxe = e;

					iPointEdge3 = iPointEdge;
				}
			}
		}

		if (iPointEdge3 >= 0)	// If there is a point in the interval [pSegmentEndpoint1->idx, pSegmentEndpoint2->idx], 
								// which is outside of the tolerance eThr from the plane (NE, dE)
		{
			if (iPointEdge3 - 1 > pSegmentEndpoint1->Idx)
			{
				// Insert an endpoint pSegmentEndpoint2_ with index iPointEdge3 - 1 between pSegmentEndpoint1 and pSegmentEndpoint2.

				pSegmentEndpoint2_ = pSegmentEndpoint;

				RVLQLIST_INSERT_ENTRY(pSegmentEndpointList, pSegmentEndpoint1, pSegmentEndpoint2, pSegmentEndpoint2_);

				pSegmentEndpoint2_->Idx = iPointEdge3 - 1;

				pSegmentEndpoint++;
			}
			else
				pSegmentEndpoint2_ = pSegmentEndpoint1;

			// Insert an endpoint pSegmentEndpoint1_ with index iPointEdge3 between pSegmentEndpoint2_ and pSegmentEndpoint2.

			pSegmentEndpoint1_ = pSegmentEndpoint;

			RVLQLIST_INSERT_ENTRY(pSegmentEndpointList, pSegmentEndpoint2_, pSegmentEndpoint2, pSegmentEndpoint1_);
			
			pSegmentEndpoint1_->Idx = iPointEdge3;

			pSegmentEndpoint++;

			// If pSegmentEndpoint2_ = pSegmentEndpoint1, then pSegmentEndpoint1 <- pSegmentEndpoint1_, otherwise pSegmentEndpoint2 <- pSegmentEndpoint2_

			if (pSegmentEndpoint2_ == pSegmentEndpoint1)
				pSegmentEndpoint1 = pSegmentEndpoint1_;
			else
				pSegmentEndpoint2 = pSegmentEndpoint2_;
		}
		else // If all points in the interval [pSegmentEndpoint1->idx, pSegmentEndpoint2->idx] are within the tolerance eThr from the plane (NE, dE)
		{
			segmentSize = pSegmentEndpoint2->Idx - pSegmentEndpoint1->Idx + 1;

			if (segmentSize >= minEdgeFeatureSize)
			{
				// Create new edge feature.

				pEdgeFeature = pSurfels->NodeArray.Element + iNewFeature_;

				pEdgeFeature->bEdge = true;

				N = pEdgeFeature->N;

				RVLCOPY3VECTOR(NE, N);

				pEdgeFeature->d = dE;

				P_ = pEdgeFeature->P;

				RVLCOPY3VECTOR(P1_, P_);

				pEdgeFeature->physicalSize = l;

				// Connect the new edge feature to the iSurfel.

				pSEdgeList_ = &(pEdgeFeature->EdgeList);

				RVLQLIST_INIT(pSEdgeList_);

				pSEdge = ConnectNodes<Surfel, SURFEL::Edge, SURFEL::EdgePtr>(iSurfel, iNewFeature_, pSurfels->NodeArray, pMem);

				RVLQLIST_ADD_ENTRY(pSEdgeList, pSEdge);

				nSEdges++;

				// Assign points to the new edge feature.

				for (iPointEdge = pSegmentEndpoint1->Idx; iPointEdge <= pSegmentEndpoint2->Idx; iPointEdge++)
				{
					pEdgePtr = pBoundary->Element[iPointEdge];

					iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

					pSurfels->edgeMap[iPt] = iNewFeature_;
				}

				RVLMEM_ALLOC_STRUCT(pMem, Array<MeshEdgePtr *>, pEdgePtArray);

				pEdgePtArray->Element = pBoundary->Element + pSegmentEndpoint1->Idx;
				pEdgePtArray->n = segmentSize;

				pEdgeFeature->BoundaryArray.Element = pEdgePtArray;
				pEdgeFeature->BoundaryArray.n = 1;

				//
				
				iNewFeature_++;
			}

			pSegmentEndpoint1 = pSegmentEndpoint2->pNext;

			if (pSegmentEndpoint1)
				pSegmentEndpoint2 = pSegmentEndpoint1->pNext;
			else
				break;
		}
	}	// while(true)

	delete[] pSegmentEndpointMem;

	return iNewFeature_ - iNewFeature;
}
#endif

void PlanarSurfelDetector::DisplaySoftEdges(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	unsigned char *Color)
{
	int iEdge;
	MeshEdge *pEdge;
	int iSurfel1, iSurfel2;
	int iPt1, iPt2;
	Point *pPt1, *pPt2;

	for (iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	{
		pEdge = pMesh->EdgeArray.Element + iEdge;

		iPt1 = pEdge->iVertex[0];
		iPt2 = pEdge->iVertex[1];

		iSurfel1 = pSurfels->surfelMap[iPt1];
		iSurfel2 = pSurfels->surfelMap[iPt2];

		if (iSurfel1 >= 0 && iSurfel1 < pMesh->NodeArray.n && iSurfel2 >= 0 && iSurfel2 < pMesh->NodeArray.n)
		{
			if (iSurfel1 != iSurfel2)
			{
				pPt1 = pMesh->NodeArray.Element + iPt1;
				pPt2 = pMesh->NodeArray.Element + iPt2;

				IntersectionPlane(pSurfels, iSurfel1, iSurfel2);

				if (!RVLPLANARSURFELDETECTOR_ON_LINE_CUT(NLineCut, dLineCut, pPt1->P, pPt2->P))
				{
					pVisualizer->PaintPoint(iPt1, pMesh->pPolygonData, Color);
					pVisualizer->PaintPoint(iPt2, pMesh->pPolygonData, Color);
				}
			}
		}
	}
}

// Joint surfel iSurfel to surfel iSurfel_.

void PlanarSurfelDetector::JoinSurfel(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel,
	int iSurfel_)
{
	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	QList<QLIST::Index2> *pPtList = &(pSurfel->PtList);
	QList<QLIST::Index2> *pPtList_ = &(pSurfel_->PtList);

	QLIST::Index2 *pPtIdx = pPtList->pFirst;

	QLIST::Index2 *pNextPtIdx;

	while (pPtIdx)
	{
		pNextPtIdx = pPtIdx->pNext;

		RVLQLIST_MOVE_ENTRY2(pPtList, pPtList_, pPtIdx, QLIST::Index2);

		pSurfels->surfelMap[pPtIdx->Idx] = iSurfel_;

		pPtIdx = pNextPtIdx;
	}
}

void PlanarSurfelDetector::GetNeighbors(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel,
	QList<SURFEL::Edge> *pEdgeList,
	int &nEdges,
	bool bSmall)
{
	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	CRVLMem *pMem2 = &Mem2A;

	if (bSmall)
		RVLQLIST_INIT(pEdgeList);

	int iBoundary, iPointEdge;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_;
	int iPt, iPt_;
	int iSurfel_;
	SURFEL::Edge *pSEdge;

	for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
	{
		pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

		for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
		{
			pEdgePtr = pBoundary->Element[iPointEdge];

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

			pEdgePtr_ = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

			while (pEdgePtr_)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr_, pEdge, iPt_);

				iSurfel_ = pSurfels->surfelMap[iPt_];

				if (iSurfel_ != iSurfel)
				{
					if (iSurfel_ >= 0 && iSurfel_ < pMesh->NodeArray.n)
					{
						if (pSurfels->neighborEdge[iSurfel_] == NULL)
						{
							if (bSmall)
							{
								RVLMEM_ALLOC_STRUCT(pMem2, SURFEL::Edge, pSEdge);

								pSEdge->iVertex[1] = iSurfel_;

								RVLQLIST_ADD_ENTRY(pEdgeList, pSEdge);

								pSurfels->neighborEdge[iSurfel_] = pSEdge;
							}
							else
							{
								if (iSurfel_ > iSurfel)
								{
									pSEdge = ConnectNodes<Surfel, SURFEL::Edge, SURFEL::EdgePtr>(iSurfel, iSurfel_, pSurfels->NodeArray, pMem);

									RVLQLIST_ADD_ENTRY(pEdgeList, pSEdge);

									nEdges++;

									pSurfels->neighborEdge[iSurfel_] = pSEdge;
								}
							}
						}	// if (pSurfels->neighborEdge[iSurfel_] == NULL)
					}	// if (iSurfel_ >= 0 && iSurfel_ < pMesh->NodeArray.n)
				}	// if (iSurfel_ != iSurfel)

				pEdgePtr_ = pEdgePtr_->pNext;
			}	// for each neighboring point of the point iPt
		}	// for each point-edge on the boundary contour
	}	// for each boundary contour

	if (bSmall)
	{
		pSEdge = pEdgeList->pFirst;

		while (pSEdge)
		{
			pSurfels->neighborEdge[pSEdge->iVertex[1]] = NULL;

			pSEdge = pSEdge->pNext;
		}
	}
	else
	{
		SURFEL::EdgePtr *pSEdgePtr = pSurfel->EdgeList.pFirst;

		while (pSEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iSurfel, pSEdgePtr, pSEdge, iSurfel_);

			pSurfels->neighborEdge[iSurfel_] = NULL;

			pSEdgePtr = pSEdgePtr->pNext;
		}
	}
}

int PlanarSurfelDetector::GetClosestNeighbor(
	SurfelGraph *pSurfels,
	int iSurfel,
	QList<SURFEL::Edge> &neighborList)
{
	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	int iClosestNeighbor = -1;
	float minDist = 0.0f;

	int iSurfel_;
	Surfel *pSurfel_;
	float dist;

	SURFEL::Edge *pEdge = neighborList.pFirst;

	while (pEdge)
	{
		iSurfel_ = pEdge->iVertex[1];

		pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;
		
		if (!pSurfel_->bEdge)
		{
#ifdef RVLPLANARSURFELDETECTOR_JOIN_SMALL_SURFELS_ONLY_TO_LARGE_NEIGHBORS
			if (pSurfel_->size >= minSurfelSize)
#endif
			{
				dist = RVLDOTPRODUCT3(pSurfel_->N, pSurfel->P) - pSurfel_->d;

				dist = RVLABS(dist);

				if (iClosestNeighbor < 0 || dist < minDist)
				{
					iClosestNeighbor = iSurfel_;

					minDist = dist;
				}
			}
		}

		pEdge = pEdge->pNext;
	}

	return iClosestNeighbor;
}

void PlanarSurfelDetector::JoinSmallSurfelsToClosestNeighbors(
	Mesh *pMesh,
	SurfelGraph *pSurfels)
{
	Surfel *pSurfel = pSurfels->NodeArray.Element;

	MeshEdgePtr **pNewBoundaryElement = pSurfels->surfelBndMem;
	Array<MeshEdgePtr *> *pNewBoundary = pSurfels->surfelBndMem2;

	int nSEdges = 0;

	QList<SURFEL::Edge> SEdgeList;

	QList<SURFEL::Edge> *pSEdgeList_ = &SEdgeList;

	int iSurfel;
	int iClosestNeighbor;
	MeshEdgePtr *pEdgePtr;
	QList<QLIST::Index2> *pPtList;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++, pSurfel++)
	{
		if (pSurfel->bEdge)
			continue;

		if (iSurfel == 269)
			int debug = 0;

		if (pSurfel->size > 0 && pSurfel->size < minSurfelSize)
		{
			pPtList = &(pSurfel->PtList);

			pSurfel->BoundaryArray.Element = pNewBoundary;

			if (pSurfel->size == 1)
			{
				pEdgePtr = pMesh->NodeArray.Element[pPtList->pFirst->Idx].EdgeList.pFirst;

				if (pEdgePtr)
				{
					pSurfel->BoundaryArray.n = 1;

					pNewBoundary->Element = pNewBoundaryElement;

					pNewBoundaryElement++;

					pNewBoundary->Element[0] = pEdgePtr;

					pNewBoundary->n = 1;
				}
				else
					pSurfel->BoundaryArray.n = 0;
			}
			else
				pMesh->Boundary(pPtList, pSurfels->surfelMap, pSurfel->BoundaryArray, pNewBoundaryElement, pSurfels->edgeMarkMap);

			Mem2A.Clear();

			GetNeighbors(pMesh, pSurfels, iSurfel, &SEdgeList, nSEdges, true);

			iClosestNeighbor = GetClosestNeighbor(pSurfels, iSurfel, SEdgeList);

			if (iClosestNeighbor >= 0)
			{
				JoinSurfel(pMesh, pSurfels, iSurfel, iClosestNeighbor);

				pSurfel->size = 0;
			}
		}	// if (pSurfel->size > 0 && pSurfel->size < minSurfelSize)

		//if (pSurfels->NodeArray.Element[974].PtList.pFirst->pNext != NULL)
		//	int debug = 0;
	}	// for every surfel
}

void PlanarSurfelDetector::CreatePolygons(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Mesh *pPolygonMesh)
{
	// Constants.

	float eThr = 4.0f * maxEdgeFeatureConcavity * maxEdgeFeatureConcavity;

	//

	Visualizer visualizer;
	visualizer.Create();
	//visualizer.SetMesh(pMesh);
	uchar green[] = { 0, 255, 0 };
	pSurfels->NodeColors(green);
	pSurfels->InitDisplay(&visualizer, pMesh, this);
	pSurfels->Display(&visualizer, pMesh);
	pSurfels->DisplayEdgeFeatures();

	std::vector<Point> vertices;
	std::vector<Pair<int, int>> lines;
	int i, iPt, iPt_, iSurfel, iSurfel_, iSurfel__;
	int iBoundary;
	Surfel *pSurfel, *pSurfel__;
	Array<MeshEdgePtr *> *pBoundary;
	//MeshEdgePtr *pBoundaryElement;
	int iBoundaryElement, iBoundaryElement0;
	MeshEdgePtr* pEdgePtr, * pEdgePtr_, * pEdgePtr__;
	MeshEdge* pEdge;
	Point* pPt;
	QList<MeshEdgePtr>* pNeighborPtList;
	float fTmp;
	bool bCoPlanar;
	cv::Mat cvM(3, 3, CV_32FC1);
	float* M = (float *)(cvM.data);
	float* N = M;
	float* N__ = M + 3;
	float* V = M + 6;
	cv::Mat cvc(3, 1, CV_32FC1);
	float* c = (float*)(cvc.data);
	cv::Mat cvr(3, 1, CV_32FC1);
	float* r = (float*)(cvr.data);
	float s, e;
	float dP[3], V3Tmp[3];
	float sMin, sMax;
	bool bDefined;
	Point vertex;
	Pair<int, int> line;
	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		//if (iSurfel == 170)
		//	int debug = 0;
		pSurfel = pSurfels->NodeArray.Element + iSurfel;
		if (pSurfel->size == 0 || pSurfel->bEdge)
			continue;
		RVLCOPY3VECTOR(pSurfel->N, N);
		r[0] = pSurfel->d;
		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			iSurfel__ = pSurfels->NodeArray.n;
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;
			for (i = 0; i < pBoundary->n; i++)
			{
				pEdgePtr = pBoundary->Element[i];
				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);
				pPt = pMesh->NodeArray.Element + iPt;
				pNeighborPtList = &(pPt->EdgeList);
				pEdgePtr_ = pEdgePtr;
				while (pEdgePtr)
				{
					pEdgePtr__ = pEdgePtr_;
					RVLQLIST_GET_NEXT_CIRCULAR(pNeighborPtList, pEdgePtr_);
					if (pEdgePtr_ == pEdgePtr)
						break;
				}
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr__, pEdge, iPt_);
				iSurfel_ = pSurfels->surfelMap[iPt_];
				if (iSurfel_ != iSurfel)
				{
					if (iSurfel_ == iSurfel__)
					{
						RVLDIF3VECTORS(pPt->P, c, dP);
						s = RVLDOTPRODUCT3(V, dP);
						RVLSCALE3VECTOR(V, s, V3Tmp);
						RVLDIF3VECTORS(dP, V3Tmp, dP);
						e = RVLDOTPRODUCT3(dP, dP);
						if (e <= eThr)
						{
							if (bDefined)
							{
								if (s < sMin)
									sMin = s;
								else if (s > sMax)
									sMax = s;
							}
							else
							{
								bDefined = true;
								sMin = sMax = s;
							}
						}
					}
					else 
					{
						if (iSurfel__ < pSurfels->NodeArray.n)
						{
							RVLSCALE3VECTOR(V, sMin, V3Tmp);
							RVLSUM3VECTORS(c, V3Tmp, vertex.P);
							line.a = vertices.size();
							vertices.push_back(vertex);
							RVLSCALE3VECTOR(V, sMax, V3Tmp);
							RVLSUM3VECTORS(c, V3Tmp, vertex.P);
							line.b = vertices.size();
							vertices.push_back(vertex);
							lines.push_back(line);
						}
						if (iSurfel_ >= 0 && iSurfel_ < pSurfels->NodeArray.n)
						{
							iSurfel__ = iSurfel_;
							pSurfel__ = pSurfels->NodeArray.Element + iSurfel__;
							RVLCROSSPRODUCT3(pSurfel->N, pSurfel__->N, V);
							fTmp = RVLDOTPRODUCT3(V, V);
							if (!(bCoPlanar = (fTmp < 1e-8)))
							{
								RVLCOPY3VECTOR(pSurfel__->N, N__);
								r[1] = pSurfel__->d;
								RVLSCALE3VECTOR2(V, fTmp, V);
								r[2] = RVLDOTPRODUCT3(V, pPt->P);
								cv::solve(cvM, cvr, cvc);
								RVLDIF3VECTORS(pPt->P, c, dP);
								e = RVLDOTPRODUCT3(dP, dP);
								if (bDefined = (e <= eThr))
									sMin = sMax = 0.0f;
							}
						}
					}
				}
			}
			if (iSurfel__ < pSurfels->NodeArray.n)
			{
				RVLSCALE3VECTOR(V, sMin, V3Tmp);
				RVLSUM3VECTORS(c, V3Tmp, vertex.P);
				line.a = vertices.size();
				vertices.push_back(vertex);
				RVLSCALE3VECTOR(V, sMax, V3Tmp);
				RVLSUM3VECTORS(c, V3Tmp, vertex.P);
				line.b = vertices.size();
				vertices.push_back(vertex);
				lines.push_back(line);
			}
		}	// for each boundary
	}	// for each surfel
	Array<Point> points;
	points.n = vertices.size();
	points.Element = new Point[points.n];
	for (int i = 0; i < points.n; i++)
		points.Element[i] = vertices[i];
	Array<Pair<int, int>> lines_;
	lines_.n = lines.size();
	lines_.Element = new Pair<int, int>[lines_.n];
	for (int i = 0; i < lines_.n; i++)
		lines_.Element[i] = lines[i];
	visualizer.DisplayLines(points, lines_, green);
	delete[] points.Element;
	delete[] lines_.Element;
	visualizer.Run();

	// Create ploygon mesh.
}

// Polygonalize boundary segment between two vertices.
// Input:  pSurfels - surfels
//         pBoundary - boundary
//         iContourStart - the first boundary element of the segment
//         iContourEnd - the first boundary element of the followng segment
//         PStart - position of the starting point
//         PEnd - position of the ending point
//         iSurfel - surfel to which pBoundary belongs
//         iSurfel_ - surfel which shares pBoundary with iSurfel

void PlanarSurfelDetector::Polygonalize(
	SurfelGraph *pSurfels,
	Array<MeshEdgePtr *> *pBoundary,
	int iContourStart,
	int iContourEnd,
	float *PStart,
	float *PEnd,
	int iSurfel,
	int iSurfel_)
{
	// (N, d) <- separating plane.

	// 
}

void PlanarSurfelDetector::MergeSurfels(
	Mesh* pMesh,
	SurfelGraph* pSurfels,
	int minSurfelSize,
	int minEdgeSize,
	float maxCoPlanarSurfelNormalAngle,
	float maxCoPlanarSurfelRefPtDist,
	SurfelGraph* pPlanarSurfaces,
	Visualizer* pVisualizer)
{
	// Constants.

	float csMaxCoPlanarSurfelNormalAngle = cos(maxCoPlanarSurfelNormalAngle * DEG2RAD);

	// surfels <- sorted list of suficiently large surfels

	Array<SortIndex<int>> surfels;
	surfels.Element = new SortIndex<int>[pSurfels->NodeArray.n];
	SortIndex<int>* pSurfelIdx = surfels.Element;
	int iSurfel;
	Surfel* pSurfel;
	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;
		if (pSurfel->bEdge)
			continue;
		if (pSurfel->size < minSurfelSize)
			continue;
		pSurfelIdx->idx = iSurfel;
		pSurfelIdx->cost = pSurfel->size;
		pSurfelIdx++;
	}
	surfels.n = pSurfelIdx - surfels.Element;
	BubbleSort<SortIndex<int>>(surfels, true);

	// relevantSurfelID <- assigns the index in list surfels to elements of pSurfels->NodeArray.
	// moments <- moments of surfels in list surfels.
	// Recompute parameters of relevant surfels.

	int* relevantSurfelID = new int[pSurfels->NodeArray.n];
	memset(relevantSurfelID, 0xff, pSurfels->NodeArray.n * sizeof(int));
	RVL_DELETE_ARRAY(pSurfels->momentsMem);
	pSurfels->momentsMem = new Moments<double>[surfels.n];
	Moments<double>* pMoments = pSurfels->momentsMem;
	Moments<double> moments_;
	Array<int> ptArray;
	ptArray.Element = new int[pMesh->NodeArray.n];
	Array<int> iSurfelArray;
	iSurfelArray.Element = &iSurfel;
	iSurfelArray.n = 1;
	int i;
	cv::Mat cvC(3, 3, CV_64FC1);
	double* C = (double*)(cvC.data);
	cv::Mat cvEigVC;
	double* eigVC;
	cv::Mat cvEigC;
	double* lfN;
	double lfP[3];
	for (i = 0; i < surfels.n; i++, pMoments++)
	{
		iSurfel = surfels.Element[i].idx;
		pSurfel = pSurfels->NodeArray.Element + iSurfel;
		relevantSurfelID[iSurfel] = i;
		pSurfels->GetPoints(iSurfelArray, ptArray);
		pMesh->ComputeMoments(ptArray, moments_);
		pSurfel->pMoments = pMoments;
		*pMoments = moments_;
		GetCovMatrix3<double>(pMoments, C, lfP);
		cv::eigen(cvC, cvEigC, cvEigVC);
		eigVC = (double*)(cvEigVC.data);
		lfN = eigVC + 6;
		RVLCOPY3VECTOR(lfN, pSurfel->N);
		if (RVLDOTPRODUCT3(pSurfel->N, pSurfel->P) > 0.0f)
		{
			RVLNEGVECT3(pSurfel->N, pSurfel->N);
		}
		RVLCOPY3VECTOR(lfP, pSurfel->P);
		pSurfel->d = RVLDOTPRODUCT3(pSurfel->N, pSurfel->P);
	}

	// Representative surfel points.

	pSurfels->RepresentativeSurfelSamples(pMesh, minSurfelSize, minEdgeSize);

	/// Merge approximatelly coplanar adjacent surfels into planar surfaces.
	/// sortedPlanarSurfaces <- list of sorted planar surfaces.

	RVL_DELETE_ARRAY(pPlanarSurfaces->NodeArray.Element);
	pPlanarSurfaces->NodeArray.Element = new Surfel[surfels.n];
	Surfel* pPlanarSurface = pPlanarSurfaces->NodeArray.Element;
	RVL_DELETE_ARRAY(pPlanarSurfaces->momentsMem);
	pPlanarSurfaces->momentsMem = new Moments<double>[surfels.n];
	pMoments = pPlanarSurfaces->momentsMem;
	int iPlanarSurface;
	QList<QLIST::Index>* pSurfelList;
	RVL_DELETE_ARRAY(pSurfels->planarSurfaceSurfelMem);
	pSurfels->planarSurfaceSurfelMem = new QLIST::Index[surfels.n];
	QLIST::Index* pSurfelIdx_ = pSurfels->planarSurfaceSurfelMem;
	bool* bMerged = new bool[pSurfels->NodeArray.n];
	memset(bMerged, 0, pSurfels->NodeArray.n * sizeof(bool));
	int iSurfel_;
	Surfel* pSurfel_;
	float dP[3];
	float e;
	float fTmp;
	SURFEL::EdgePtr* pAdjacentSurfelEdgePtr;
	SURFEL::Edge* pAdjacentSurfelEdge;
	maxnSurfelsPerPlanarSurface = 0;
	int nMergedSurfels;
	float V3Tmp[3];
	Moments<double> jointMoments;
	QLIST::Index* pMemberSurfelIdx;
	Surfel* pMemberSurfel;
	bool bSurfelJoined;
	int iLargestAdjacentSurfel;
	int maxAdjacentSurfelSize;
	double jointP[3];
	double* jointN;
	float* surfelRefPts;
	float* P;
	int j;
	for (i = 0; i < surfels.n; i++)
	{
		iSurfel = surfels.Element[i].idx;
		if (bMerged[iSurfel])
			continue;
		bMerged[iSurfel] = true;
		pSurfelList = &(pPlanarSurface->children);
		RVLQLIST_INIT(pSurfelList);
		RVLQLIST_ADD_ENTRY(pSurfelList, pSurfelIdx_);
		pSurfelIdx_->Idx = iSurfel;
		pSurfelIdx_++;
		pSurfel = pSurfels->NodeArray.Element + iSurfel;
		//RVLCOPY3VECTOR(pSurfel->P, pPlanarSurface->P);
		//fTmp = (float)(pSurfel->size);
		//RVLSCALE3VECTOR(pPlanarSurface->P, fTmp, pPlanarSurface->P);
		pPlanarSurface->bEdge = false;
		pPlanarSurface->pMoments = pMoments++;
		*(pPlanarSurface->pMoments) = *(pSurfel->pMoments);
		RVLCOPY3VECTOR(pSurfel->N, pPlanarSurface->N);
		RVLCOPY3VECTOR(pSurfel->P, pPlanarSurface->P);
		nMergedSurfels = 1;
		bSurfelJoined = true;
		while (bSurfelJoined)
		{
			bSurfelJoined = false;
			maxAdjacentSurfelSize = 0;
			pMemberSurfelIdx = pSurfelList->pFirst;
			while (pMemberSurfelIdx)
			{
				pMemberSurfel = pSurfels->NodeArray.Element + pMemberSurfelIdx->Idx;
				pAdjacentSurfelEdgePtr = pMemberSurfel->EdgeList.pFirst;
				while (pAdjacentSurfelEdgePtr)
					//for (int j = i + 1; j < surfels.n; j++)
				{
					//iSurfel_ = surfels.Element[j].idx;
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(pMemberSurfelIdx->Idx, pAdjacentSurfelEdgePtr, pAdjacentSurfelEdge, iSurfel_);
					if (!bMerged[iSurfel_] && relevantSurfelID[iSurfel_] >= 0)
					{
						pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;
						if (!pSurfel_->bEdge)
						{
							if (pSurfel_->size > maxAdjacentSurfelSize)
							{
								if (RVLDOTPRODUCT3(pPlanarSurface->N, pSurfel_->N) >= csMaxCoPlanarSurfelNormalAngle)
								{
									maxAdjacentSurfelSize = pSurfel_->size;
									iLargestAdjacentSurfel = iSurfel_;
								}
							}
						}
					}
					pAdjacentSurfelEdgePtr = pAdjacentSurfelEdgePtr->pNext;
				}
				pMemberSurfelIdx = pMemberSurfelIdx->pNext;
			}
			if (maxAdjacentSurfelSize > 0)
			{
				//if (iLargestAdjacentSurfel == 1400)
				//	int debug = 0;
				pSurfel_ = pSurfels->NodeArray.Element + iLargestAdjacentSurfel;
				SumMoments<double>(*(pSurfel_->pMoments), *(pPlanarSurface->pMoments), jointMoments);
				GetCovMatrix3<double>(&jointMoments, C, jointP);
				cv::eigen(cvC, cvEigC, cvEigVC);
				eigVC = (double*)(cvEigVC.data);
				jointN = eigVC + 6;

				// Visualization of planar surface hypothesis.

				if (pVisualizer)
				{
					pVisualizer->SetMesh(pMesh);
					Array<Point> ptArray_;
					ptArray_.Element = new Point[12 * surfels.n];
					Point* pPt = ptArray_.Element;
					Box<float> bbox;
					surfelRefPts = pSurfel_->representativePts;
					float PB[3];
					for (j = 0; j < 4; j++, pPt++)
					{
						P = surfelRefPts + 3 * j;
						RVLCOPY3VECTOR(P, pPt->P);
						RVLDIF3VECTORS(P, jointP, dP);
						RVLMULMX3X3VECT(eigVC, dP, PB);
						if (j == 0)
							InitBoundingBox<float>(&bbox, PB);
						else
							UpdateBoundingBox<float>(&bbox, PB);
					}
					pMemberSurfelIdx = pSurfelList->pFirst;
					while (pMemberSurfelIdx)
					{
						pMemberSurfel = pSurfels->NodeArray.Element + pMemberSurfelIdx->Idx;
						surfelRefPts = pMemberSurfel->representativePts;
						for (j = 0; j < 4; j++, pPt++)
						{
							P = surfelRefPts + 3 * j;
							RVLCOPY3VECTOR(P, pPt->P);
							RVLDIF3VECTORS(P, jointP, dP);
							RVLMULMX3X3VECT(eigVC, dP, PB);
							UpdateBoundingBox<float>(&bbox, PB);
						}
						pMemberSurfelIdx = pMemberSurfelIdx->pNext;
					}
					Pose3D poseBC;
					RVLCOPYMX3X3T(eigVC, poseBC.R);
					RVLCOPY3VECTOR(jointP, poseBC.t);
					float BBoxCenterB[3];
					RVLSET3VECTOR(BBoxCenterB, 0.5f * (bbox.maxx + bbox.minx), 0.5f * (bbox.maxy + bbox.miny), 0.0f);
					Pose3D poseB_C = poseBC;
					RVLTRANSF3(BBoxCenterB, poseBC.R, poseBC.t, poseB_C.t);
					pVisualizer->DisplayBox(bbox.maxx - bbox.minx, bbox.maxy - bbox.miny, 2.0 * maxCoPlanarSurfelRefPtDist, &poseB_C, 255, 0, 0);
					ptArray_.n = pPt - ptArray_.Element;
					uchar green[] = { 0, 255, 0 };
					pVisualizer->DisplayPointSet<float, Point>(ptArray_, green, 6);
					pVisualizer->Run();
					pVisualizer->renderer->RemoveAllViewProps();
					delete[] ptArray_.Element;
				}

				//

				//surfelRefPts = surfelRefPtMem + 12 * iLargestAdjacentSurfel;
				surfelRefPts = pSurfel_->representativePts;
				for (j = 0; j < 4; j++)
				{
					P = surfelRefPts + 3 * j;
					RVLDIF3VECTORS(P, jointP, dP);
					e = RVLDOTPRODUCT3(jointN, dP);
					if (RVLABS(e) > maxCoPlanarSurfelRefPtDist)
						break;
				}
				if (j >= 4)
				{
					pMemberSurfelIdx = pSurfelList->pFirst;
					while (pMemberSurfelIdx)
					{
						pMemberSurfel = pSurfels->NodeArray.Element + pMemberSurfelIdx->Idx;
						//surfelRefPts = surfelRefPtMem + 12 * pMemberSurfelIdx->Idx;
						surfelRefPts = pMemberSurfel->representativePts;
						for (j = 0; j < 4; j++)
						{
							P = surfelRefPts + 3 * j;
							RVLDIF3VECTORS(P, jointP, dP);
							e = RVLDOTPRODUCT3(jointN, dP);
							if (RVLABS(e) > maxCoPlanarSurfelRefPtDist)
								break;
						}
						if (j < 4)
							break;
						pMemberSurfelIdx = pMemberSurfelIdx->pNext;
					}
					if (pMemberSurfelIdx == NULL)
					{
						bMerged[iLargestAdjacentSurfel] = true;
						RVLQLIST_ADD_ENTRY(pSurfelList, pSurfelIdx_);
						pSurfelIdx_->Idx = iLargestAdjacentSurfel;
						pSurfelIdx_++;
						if (RVLDOTPRODUCT3(jointN, jointP) > 0.0f)
						{
							RVLNEGVECT3(jointN, jointN);
						}
						RVLCOPY3VECTOR(jointN, pPlanarSurface->N);
						RVLCOPY3VECTOR(jointP, pPlanarSurface->P);
						*(pPlanarSurface->pMoments) = jointMoments;
						nMergedSurfels++;
						bSurfelJoined = true;
					}
				}
			}
		}
		//fTmp = (float)(pPlanarSurfacelIdx->cost);
		//RVLSCALE3VECTOR2(pPlanarSurface->P, fTmp, pPlanarSurface->P);
		//RVLCOPY3VECTOR(pSurfel->N, pPlanarSurface->N);
		pPlanarSurface->size = pPlanarSurface->pMoments->n;
		pPlanarSurface->d = RVLDOTPRODUCT3(pPlanarSurface->N, pPlanarSurface->P);
		pPlanarSurface++;
		if (nMergedSurfels > maxnSurfelsPerPlanarSurface)
			maxnSurfelsPerPlanarSurface = nMergedSurfels;
	}
	pPlanarSurfaces->NodeArray.n = pPlanarSurface - pPlanarSurfaces->NodeArray.Element;
	delete[] bMerged;
	delete[] relevantSurfelID;
	delete[] ptArray.Element;
	delete[] surfels.Element;
	QList<QLIST::Index2>* pPtList, * pPtList_;
	QLIST::Index2* pPtIdx;
	RVL_DELETE_ARRAY(pPlanarSurfaces->surfelMap);
	pPlanarSurfaces->surfelMap = new int[pMesh->NodeArray.n];
	memset(pPlanarSurfaces->surfelMap, 0xff, pMesh->NodeArray.n * sizeof(int));
	for (iPlanarSurface = 0; iPlanarSurface < pPlanarSurfaces->NodeArray.n; iPlanarSurface++)
	{
		pPlanarSurface = pPlanarSurfaces->NodeArray.Element + iPlanarSurface;
		pPtList = &(pPlanarSurface->PtList);
		RVLQLIST_INIT(pPtList);
		pMemberSurfelIdx = pPlanarSurface->children.pFirst;
		while (pMemberSurfelIdx)
		{
			pMemberSurfel = pSurfels->NodeArray.Element + pMemberSurfelIdx->Idx;
			pPtList_ = &(pMemberSurfel->PtList);
			RVLQLIST_APPEND2(pPtList, pPtList_);
			pMemberSurfelIdx = pMemberSurfelIdx->pNext;
		}
		pPtIdx = pPtList->pFirst;
		while (pPtIdx)
		{
			pPlanarSurfaces->surfelMap[pPtIdx->Idx] = iPlanarSurface;
			pPtIdx = pPtIdx->pNext;
		}
	}

	// Representative planar surface points.

	pPlanarSurfaces->RepresentativeComplexSurfelSamples(pMesh, minSurfelSize, minEdgeSize, pSurfels);

	// Establish adjacency relations between planar surfaces.
	// (I started programming this, but I haven't finished.)

	//bool* bProcessed = new bool[pPlanarSurfaces->NodeArray.n];
	//memset(bProcessed, 0, pPlanarSurfaces->NodeArray.n * sizeof(bool));
	//Array<int> processedPSs;
	//processedPSs.Element = new int[pPlanarSurfaces->NodeArray.n];
	//bool* bMember = new bool[pSurfels->NodeArray.n];
	//memset(bMember, 0, pSurfels->NodeArray.n * sizeof(bool));
	//for (iPlanarSurface = 0; iPlanarSurface < pPlanarSurfaces->NodeArray.n; iPlanarSurface++)
	//{
	//	pPlanarSurface = pPlanarSurfaces->NodeArray.Element + iPlanarSurface;
	//	pMemberSurfelIdx = pPlanarSurface->children.pFirst;
	//	while (pMemberSurfelIdx)
	//	{
	//		//pMemberSurfel = pSurfels->NodeArray.Element + pMemberSurfelIdx->Idx;
	//		bMember[pMemberSurfelIdx->Idx] = true;
	//		pMemberSurfelIdx = pMemberSurfelIdx->pNext;
	//	}
	//}
	//delete[] bProcessed;
	//delete[] processedPSs.Element;
	//delete[] bMember;
}

void PlanarSurfelDetector::DetectPlaneNSB(
	Mesh* pMesh,
	int iSeedPt,
	Array<int>& ptIdx)
{
	// Parameters.

	float resolution = 1e-5;
	int initCircleApproxResolution = 20;
	float eThr = 0.020f;

	// Constants.

	float kQ = 1.0f / resolution;

	//

	Point* pSeedPt = pMesh->NodeArray.Element + iSeedPt;
	float RCP[9];
	float* XPC = RCP;
	float* YPC = RCP + 3;
	float* ZPC = RCP + 6;
	RVLCOPY3VECTOR(pSeedPt->N, ZPC);
	int i, j, k;
	float fTmp;
	RVLORTHOGONAL3(ZPC, XPC, i, j, k, fTmp);
	RVLCROSSPRODUCT3(ZPC, XPC, YPC);
	float* P0 = pSeedPt->P;
	struct PointP
	{
		int idx;
		float P[3];
		float rho[2];
		PointP* pNext;
		bool bJoined;
		bool bInQueue;
		bool bOutlier;
	};
	Array<QList<PointP>> Q;
	Q.n = (int)ceil(PI / resolution);
	Q.Element = new QList<PointP>[Q.n];
	QList<PointP>* pBin;
	for (i = 0; i < Q.n; i++)
	{
		pBin = Q.Element + i;
		RVLQLIST_INIT(pBin);
	}
	PointP* ptPMem = new PointP[pMesh->NodeArray.n];
	int iPt;
	PointP* pPtP;
	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		pPtP = ptPMem + iPt;
		pPtP->idx = iPt;
		pPtP->bJoined = false;
		pPtP->bInQueue = false;
		pPtP->bOutlier = false;
	}
	std::vector<PSD::Point2D> C;
	float rInitCircle = 1.0f / sqrt(2.0f);
	PSD::Point2D v;
	float phi;
	float dPhi = 2.0f * PI / (float)initCircleApproxResolution;
	for (i = 0; i < initCircleApproxResolution; i++)
	{
		phi = (float)i * dPhi;
		v.P[0] = rInitCircle * cos(phi);
		v.P[1] = rInitCircle * sin(phi);
		C.push_back(v);
	}
	// Only for debugging purpose!!!
	Rect<float> ROI;
	ROI.minx = -1.0f;
	ROI.maxx = 1.0f;
	ROI.miny = -1.0f;
	ROI.maxy = 1.0f;
	//
	//VisualizeConvexSet(C, 200, &ROI);
	float area = Area(C);
	int iBin = (int)floor(kQ * area);
	pBin = Q.Element + iBin;
	pPtP = ptPMem + iSeedPt;
	RVLQLIST_ADD_ENTRY(pBin, pPtP);
	pPtP->bJoined = true;
	pPtP->bInQueue = true;
	pPtP->P[0] = 1.0f; pPtP->P[1] = 0.0f;
	pPtP->rho[0] = 1.0f;
	pPtP->rho[1] = 1.0f;
	Point* pPt;
	float r;
	float dP[3], P_[2];
	bool bBelow1, bBelow2;
	std::vector<PSD::Point2D> C_, C__;
	ptIdx.n = 0;
	int iBin_;
	MeshEdge* pEdge;
	MeshEdgePtr* pEdgePtr;
	int iPt_;
	PointP* pPtP_;
	PointP** ppPtP;
	float refArea = area;
	for (iBin = Q.n - 1; iBin > 0; iBin--)
	{
		if (iBin == 1)
			int debug = 0;
		pBin = Q.Element + iBin;
		if (pBin->pFirst == NULL)
			continue;
		pPtP = pBin->pFirst;
		ppPtP = &(pBin->pFirst);
		while (pPtP)
		{
			C_.clear();
			if (bBelow1 = PSD::UpdateConvexSet(C, pPtP->P, pPtP->rho[0], C_))
			{
				P_[0] = -pPtP->P[0]; P_[1] = -pPtP->P[1];
				C__.clear();
				bBelow2 = PSD::UpdateConvexSet(C_, P_, pPtP->rho[1], C__);
			}
			if (bBelow1 && bBelow2)
			{
				//VisualizeConvexSet(C__, 200);
				area = Area(C__);
				iBin_ = (int)floor(kQ * area);
				if (iBin_ == iBin)
				{
					ptIdx.Element[ptIdx.n++] = pPtP->idx;
					//if (ptIdx.n >= 200)
					//	break;
					pPtP->bJoined = true;
					C = C__;
					// Only for debugging purpose!!!
					//if (area < refArea)
					//{
					//	VisualizeConvexSet(C__, 200, &ROI);
					//	refArea = area;
					//}
					//printf("%d %f\n", ptIdx.n, area);
					//
					pPt = pMesh->NodeArray.Element + pPtP->idx;
					pEdgePtr = pPt->EdgeList.pFirst;
					while (pEdgePtr)
					{
						RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(pPtP->idx, pEdgePtr, pEdge, iPt_);
						if (iPt_ == 167479)
							int debug = 0;
						pPtP_ = ptPMem + iPt_;
						if (!pPtP_->bJoined && !pPtP_->bOutlier && !pPtP_->bInQueue)
						{
							pPt = pMesh->NodeArray.Element + iPt_;
							RVLDIF3VECTORS(pPt->P, P0, dP);
							RVLMULMX3X3VECT(RCP, dP, pPtP_->P);
							r = sqrt(pPtP_->P[0] * pPtP_->P[0] + pPtP_->P[1] * pPtP_->P[1]);
							RVLSCALE3VECTOR2(pPtP_->P, r, pPtP_->P);
							fTmp = eThr / r;
							pPtP_->rho[0] = fTmp - pPtP_->P[2];
							pPtP_->rho[1] = fTmp + pPtP_->P[2];
							C_.clear();
							if (bBelow1 = PSD::UpdateConvexSet(C, pPtP_->P, pPtP_->rho[0], C_))
							{
								P_[0] = -pPtP_->P[0]; P_[1] = -pPtP_->P[1];
								C__.clear();
								bBelow2 = PSD::UpdateConvexSet(C_, P_, pPtP_->rho[1], C__);
							}
							//if (iPt_ == 163633)
							//	VisualizeConvexSet(C__, 200);
							//if (iPt_ == 167479)	// debug
							//{
							//	VisualizeConvexSet(C, 200, &ROI);
							//	float NP[3];
							//	RVLSET3VECTOR(NP, C[0].P[0], C[0].P[1], 1.0f);
							//	RVLNORM3(NP, fTmp);
							//	float e = RVLDOTPRODUCT3(NP, pPtP_->P);
							//	int debug = 0;
							//}
							if (bBelow1 && bBelow2)
							{
								area = Area(C__);
								iBin_ = (int)floor(kQ * area);
								pBin = Q.Element + iBin_;
								RVLQLIST_ADD_ENTRY(pBin, pPtP_);
								pPtP_->bInQueue = true;
							}
							else
								pPtP_->bOutlier = true;
						}
						pEdgePtr = pEdgePtr->pNext;
					}
					ppPtP = &(pPtP->pNext);
					pPtP = pPtP->pNext;
				}
				else
				{
					if (iBin_ > iBin)
						int debug = 0;
					pBin = Q.Element + iBin;
					RVLQLIST_REMOVE_ENTRY(pBin, pPtP, ppPtP);
					if (*ppPtP)
						int debug = 0;
					pBin = Q.Element + iBin_;
					RVLQLIST_ADD_ENTRY(pBin, pPtP);
					pPtP = *ppPtP;
				}
			}
			else
			{
				pPtP->bOutlier = true;
				ppPtP = &(pPtP->pNext);
				pPtP = pPtP->pNext;
			}
		}
	}
	//for (iBin = 1; iBin < Q.n; iBin++)
	//{
	//	pBin = Q.Element + iBin;
	//	if (pBin->pFirst == NULL)
	//		continue;
	//	pPtP = pBin->pFirst;
	//	while (pPtP)
	//	{
	//		if (!pPtP->bJoined && !pPtP->bOutlier)
	//			int debug = 0;
	//		pPtP = pPtP->pNext;
	//	}
	//}
	delete[] Q.Element;
	delete[] ptPMem;
}

bool PlanarSurfelDetector::UpdateConvexSet(
	std::vector<PSD::Point2D> CIn,
	float *N,
	float d,
	std::vector<PSD::Point2D> &COut)
{
	int n = CIn.size();
	PSD::Point2D v = CIn[n - 1];
	float e = v.P[0] * N[0] + v.P[1] * N[1] - d;
	float ePrev;
	PSD::Point2D vPrev, vIS, dv;
	float s, s_;
	bool bBelow = (e < 0.0f);
	bool bPrevIsBelow;
	bool bPtsBelow = false;
	for (int i = 0; i < n; i++)
	{
		vPrev = v;
		ePrev = e;
		v = CIn[i];
		e = v.P[0] * N[0] + v.P[1] * N[1] - d;
		bPrevIsBelow = bBelow;
		bBelow = (e < 0.0f);
		if (bBelow || bPrevIsBelow)
		{
			if (bBelow != bPrevIsBelow)
			{
				s = ePrev / (ePrev - e);
				s_ = 1.0f - s;
				vIS.P[0] = s_ * vPrev.P[0] + s * v.P[0];
				vIS.P[1] = s_ * vPrev.P[1] + s * v.P[1];
				COut.push_back(vIS);
			}
			if (bBelow)
				COut.push_back(v);
			bPtsBelow = true;
		}
	}
	return bPtsBelow;
}

float PlanarSurfelDetector::Area(std::vector<PSD::Point2D> C)
{
	int n = C.size();
	PSD::Point2D v = C[n - 1];
	PSD::Point2D vPrev;
	float area = 0.0f;
	for (int i = 0; i < n; i++)
	{
		vPrev = v;
		v = C[i];
		area += (vPrev.P[0] * v.P[1] - vPrev.P[1] * v.P[0]);
	}
	return area;
}

void PlanarSurfelDetector::VisualizeConvexSet(
	std::vector<PSD::Point2D> C,
	int resolution,
	Rect<float>* pROI)
{
	Rect<float> bbox;
	if (pROI)
		bbox = *pROI;
	else
	{
		InitRect<float>(&bbox, C[0].P);
		for (int i = 1; i < C.size(); i++)
			UpdateRect<float>(&bbox, C[i].P);
	}
	float sizex = bbox.maxx - bbox.minx;
	float sizey = bbox.maxy - bbox.miny;
	float size = RVLMAX(sizex, sizey);
	float s = (float)resolution / size;
	int w = (int)ceil(s * sizex);
	int h = (int)ceil(s * sizey);
	cv::Mat img;
	img.create(h, w, CV_8UC3);
	img.setTo(cv::Scalar(255, 255, 255));
	int j;
	for (int i = 0; i < C.size(); i++)
	{
		j = (i + 1) % C.size();
		cv::line(img, cv::Point((int)round(s * (C[i].P[0] - bbox.minx)), (int)round(s * (C[i].P[1] - bbox.miny))), 
			cv::Point((int)round(s * (C[j].P[0] - bbox.minx)), (int)round(s * (C[j].P[1] - bbox.miny))), cv::Scalar(255, 0, 0));
	}
	cv::imshow("Set", img);
	cv::waitKey();
}

void PlanarSurfelDetector::Save(FILE *fp)
{
	char header[] = "RVL::PlanarSurfelDetector 000";

	int headerLength = strlen(header);

	sprintf(header + headerLength - 3, "%03d", RVLPLANARSURFELDETECTOR_VERSION_0);

	fwrite(header, sizeof(char), headerLength + 1, fp);

	fwrite(&k, sizeof(float), 1, fp);
	fwrite(&kRGB, sizeof(float), 1, fp);
	fwrite(&kNormal, sizeof(float), 1, fp);
	fwrite(&kPlane, sizeof(float), 1, fp);
	fwrite(&surfelDistThr, sizeof(float), 1, fp);
	fwrite(&minSurfelSize, sizeof(int), 1, fp);
	fwrite(&maxRange, sizeof(float), 1, fp);
	fwrite(&maxAttackSize, sizeof(int), 1, fp);
	fwrite(&bJoinSmallSurfelsToClosestNeighbors, sizeof(bool), 1, fp);
}

void PSD::MouseRButtonDown(vtkObject* caller, unsigned long eid, void* clientdata, void* calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor*>(caller);
	PSD::DisplayCallbackData* pData = (PSD::DisplayCallbackData*)clientdata;

	Visualizer* pVisualizer = pData->pVisualizer;
	Mesh* pMesh = pData->pMesh;

	pData->pVisualizer->pointPicker->Pick(interactor->GetEventPosition()[0], interactor->GetEventPosition()[1], 0,
		interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	vtkIdType selectedPointIdx = pData->pVisualizer->pointPicker->GetPointId();

	uchar red[] = { 255, 0, 0 };
	uchar green[] = { 0, 255, 0 };
	uchar darkGreen[] = { 0, 128, 0 };
	uchar cyan[3] = { 0, 255, 255 };
	uchar yellow[] = {255, 255, 0};

	if (selectedPointIdx >= 0)
	{
		if (pData->selectedPtActor)
			pVisualizer->renderer->RemoveViewProp(pData->selectedPtActor);
		Point* pPt = pMesh->NodeArray.Element + selectedPointIdx;
		printf("Selected point: %d (%f %f %f)\n", selectedPointIdx, pPt->P[0], pPt->P[1], pPt->P[2]);
		Array<Point> selectedPts;
		selectedPts.n = 1;
		selectedPts.Element = pPt;
		pData->selectedPtActor = pVisualizer->DisplayPointSet<float, Point>(selectedPts, yellow, 6);
		interactor->GetRenderWindow()->Render();
	}
}

bool PSD::UpdateConvexSet(
	std::vector<PSD::Point2D> CIn,
	float* N,
	float d,
	std::vector<PSD::Point2D>& COut)
{
	int n = CIn.size();
	PSD::Point2D v = CIn[n - 1];
	float e = v.P[0] * N[0] + v.P[1] * N[1] - d;
	float ePrev;
	PSD::Point2D vPrev, vIS, dv;
	float s, s_;
	bool bBelow = (e < 0.0f);
	bool bPrevIsBelow;
	bool bPtsBelow = false;
	for (int i = 0; i < n; i++)
	{
		vPrev = v;
		ePrev = e;
		v = CIn[i];
		e = v.P[0] * N[0] + v.P[1] * N[1] - d;
		bPrevIsBelow = bBelow;
		bBelow = (e < 0.0f);
		if (bBelow || bPrevIsBelow)
		{
			if (bBelow != bPrevIsBelow)
			{
				s = ePrev / (ePrev - e);
				s_ = 1.0f - s;
				vIS.P[0] = s_ * vPrev.P[0] + s * v.P[0];
				vIS.P[1] = s_ * vPrev.P[1] + s * v.P[1];
				COut.push_back(vIS);
			}
			if (bBelow)
				COut.push_back(v);
			bPtsBelow = true;
		}
	}
	return bPtsBelow;
}