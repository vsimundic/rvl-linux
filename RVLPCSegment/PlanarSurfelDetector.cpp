//#include "stdafx.h"
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"

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
	maxRange = 5000.0f;

	pMem = NULL;
	//iPtBuff = NULL;
	map = NULL;
	distanceMap = NULL;
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

#ifdef RVLPLANARSURFELDETECTOR_DEBUG
	iPtBuffDebug = NULL;
#endif

#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
	debugDefineBoundaryiSurfel = 23;
	debugDefineBoundaryiSurfel_ = 56;
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

	Mem2A.Create(2 * nPts * sizeof(int));
	Mem2B.Create((nPts + nEdges) * sizeof(int));

	map = new int[nPts];

	memset(map, 0xff, nPts * sizeof(int));

	distanceMap = new unsigned int[nPts];

	memset(distanceMap, 0xff, nPts * sizeof(unsigned int));

	//iPtBuff = new int[2 * nPts];

	regionGrowingData.distThr = surfelDistThr * surfelDistThr;
	regionGrowingData.kRGB2 = kRGB * kRGB;
	regionGrowingData.kNormal2 = kNormal * kNormal;
	regionGrowingData.kPlane2 = kPlane * kPlane;
	regionGrowingData.surfelMap = pSurfels->surfelMap;
	regionGrowingData.buffer = map;
	regionGrowingData.costMap = regionGrowingData.costBuffer = NULL;

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

	PointEdgeBuff = new MESH::PointEdge[2 * nEdges];

	cutCostMap = new unsigned int[nEdges];

	memset(cutCostMap, 0xff, nEdges * sizeof(unsigned int));

	edgeFlags = new unsigned char[nEdges];

	memset(edgeFlags, 0, nEdges * sizeof(unsigned char));

	cutPropagationBuffMem = new QLIST::Index[2 * nEdges];
}

void PlanarSurfelDetector::DeallocateMemory()
{
	//RVL_DELETE_ARRAY(iPtBuff);
	RVL_DELETE_ARRAY(map);
	RVL_DELETE_ARRAY(distanceMap);
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
}

void PlanarSurfelDetector::RandomIndices(Array<int> &A)
{
	A.Element = new int[A.n];

	int iPt;

	for (iPt = 0; iPt < A.n; iPt++)
		A.Element[iPt] = iPt;

	int iPt_;
	int iTmp;

	for (iPt = 0; iPt < A.n; iPt++)
	{
		iPt_ = rand() % A.n;

		iTmp = A.Element[iPt];
		A.Element[iPt] = A.Element[iPt_];
		A.Element[iPt_] = iTmp;
	}
}

int PSD::RegionGrowingOperation(
	int iNode,
	int iNode_,
	MeshEdge *pEdge,
	Mesh *pMesh,
	PlanarSurfelDetectorRegionGrowingData *pData
	)
{
	if (iNode == 142837)
		int debug = 0;

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
		if (pData->surfelMap[iNode] != pData->iAttackedSurfel)
			return -1;	// iNode does not belong to G.

		if (pData->buffer[iNode] >= 0)
			return 0;	// iNode already processed.
	}

	{
		Point *pPt_ = pMesh->NodeArray.Element + iNode;

		float eRGB, eN, eP;
		
#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
		float cost;
#endif
#endif
#endif
	
		PSD::VertexDist(pData->pPtTemplate, pPt_, eRGB, eN, eP);

		float costRGB, costN, costP;
	
		if ((costRGB = pData->kRGB2 * eRGB) <= pData->distThr)
		{
			if ((costN = pData->kNormal2 * eN) <= pData->distThr)
			{
				if ((costP = pData->kPlane2 * eP) <= pData->distThr)
				{					
					if(pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER)
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
							return 1;	// iNode belongs to G.
						}
					}
				}
			}
		}

		if (pData->mode == RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER)
		{
			pData->buffer[iNode] = pData->iSurfel;

			return 1;
		}			
	}

	return -1;	// iNode does not belong to G.
}

// Segments pMesh into surfels. The resulting surfels are stored as nodes in pSurfels.
// Each point of pMesh corresponds to an element of pSurfel->surfelMap.
// The element of pSurfel->surfelMap corresponding to a particular point of pMesh has the value of the index of the surfel which the point belongs to.


void PlanarSurfelDetector::Segment(
	Mesh *pMesh,
	SurfelGraph *pSurfels)
{
	Point *Pt = pMesh->NodeArray.Element;
	 
	int nPts = pMesh->NodeArray.n;

	// Randomize vertex indices

	Array<int> RandPtIdxArray;

	RandPtIdxArray.n = nPts;

	RandomIndices(RandPtIdxArray);

	// Allocate memory and initialize arrays.

	memset(pSurfels->surfelMap, 0xff, nPts * sizeof(int));

	int *iPtBuff = new int[nPts];

	int *iBoundaryPtBuff = new int[nPts];

	Array<int> surfelPtArray;

	surfelPtArray.Element = iPtBuff;

	int *regionGrowingBuffer = new int[nPts];

	memset(regionGrowingBuffer, 0xff, nPts * sizeof(int));

	int *iSurfelSeed = new int[nPts];

	int iSurfel = 0;

	Surfel *pSurfel = pSurfels->NodeArray.Element;

	PlanarSurfelDetectorRegionGrowingData data;

	data.distThr = surfelDistThr * surfelDistThr;
	data.kRGB2 = kRGB * kRGB;
	data.kNormal2 = kNormal * kNormal;
	data.kPlane2 = kPlane * kPlane;
	data.surfelMap = pSurfels->surfelMap;	
	data.buffer = regionGrowingBuffer;
	data.costMap = data.costBuffer = NULL;
#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
	data.costMap = new float[nPts];
	data.costBuffer = new float[nPts];
#endif
#endif
#endif

	QLIST::Index *pPtIdx = pSurfels->PtMem;

	int i;
	int iPt;
	int iPtSeed;
	int *piPtFetch, *piPtPut, *piPtBuffEnd, *piPt, *piBoundaryPt;
	Point *pPt;
	Point ptTemplate;
	MESH::Distribution distribution;
	//Surfel *pSurfel_;
	QList<MeshEdgePtr> *pEdgeList;
	QList<QLIST::Index> *pPtList;
	//int iSurfel_;
	//bool bAddToNewSurfel;
	//float r0_, r, r_;

	for (i = 0; i < nPts; i++)
	{
		iPtSeed = RandPtIdxArray.Element[i];

		if (pSurfels->surfelMap[iPtSeed] >= 0)
			continue;

		pPt = pMesh->NodeArray.Element + iPtSeed;

		if (pPt->P[2] > maxRange)
			continue;

		if (RVLDOTPRODUCT3(pPt->N, pPt->N) < 0.5f)
			continue;

		// Initial region growing		

		piPtFetch = piPtPut = iPtBuff;

		piBoundaryPt = iBoundaryPtBuff;

		*(piPtPut++) = iPtSeed;

		regionGrowingBuffer[iPtSeed] = iSurfel;

#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#ifdef RVLPLANARSURFELDETECTOR_MIN_COST
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
		data.costMap[iPtSeed] = 0.0f;
#endif
#endif
#endif

		iSurfelSeed[iSurfel] = iPtSeed;

		data.pPtTemplate = Pt + iPtSeed;
		data.iSurfel = iSurfel;
		data.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION;
		
		piPtBuffEnd = RegionGrowing<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &data, piPtFetch, piPtPut);
		
		surfelPtArray.n = piPtBuffEnd - surfelPtArray.Element;

		if (surfelPtArray.n >= minSurfelSize)
		{
			// Create surfel lists.

			pEdgeList = &(pSurfel->EdgeList);

			RVLQLIST_INIT(pEdgeList);

			pPtList = &(pSurfel->PtList);

			RVLQLIST_INIT(pPtList);

			// Compute surfel parameters

			pPt = Pt + iPtSeed;

			if (surfelPtArray.n >= 20)
			{
				pMesh->ComputeDistribution(surfelPtArray, distribution);

				SURFEL::ComputeParameters(pSurfel, distribution, pPt);
			}
			else
				SURFEL::CreateFromPoint(pSurfel, pPt);

			SURFEL::GetPoint(pSurfel, &ptTemplate);

			// Reset regionGrowingBuffer

			for (piPt = iPtBuff; piPt < piPtBuffEnd; piPt++)
				regionGrowingBuffer[*piPt] = -1;

			// Todo: iPtSeed <- the closest point to iPtSeed which is consistent with ptTemplate

			piPtFetch = piPtPut = iPtBuff;

			*(piPtPut++) = iPtSeed;

			data.pPtTemplate = &ptTemplate;
			data.iPtSeed = -1;
			data.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER;

			piPtBuffEnd = RegionGrowing2<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &data, piPtFetch, piPtPut);

			for (piPt = iPtBuff; piPt < piPtBuffEnd; piPt++)
				regionGrowingBuffer[*piPt] = -1;

			if (data.iPtSeed < 0)
				data.iPtSeed = iPtSeed;

			// Final region growing

			piPtFetch = piPtPut = iPtBuff;

			*(piPtPut++) = data.iPtSeed;

			regionGrowingBuffer[iPtSeed] = iSurfel;

			data.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION;

			piPtBuffEnd = RegionGrowing<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &data, piPtFetch, piPtPut);

			// Form the final surfel point set

			for (piPt = iPtBuff; piPt < piPtBuffEnd; piPt++)
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

					RVLQLIST_ADD_ENTRY(pPtList, pPtIdx);

					pPtIdx->Idx = iPt;

					pPtIdx++;
				}

				regionGrowingBuffer[iPt] = -1;
			}	// for (piPt = iPtBuff; piPt < piPtBuffEnd; piPt++)

#ifdef RVLPLANARSURFELDETECTOR_POLYGONS
			// Define Polygon.

			DefinePolygon(pMesh, pSurfels, iSurfel);
#endif
			// Next surfel index

			iSurfel++;

			pSurfel++;

			//if (iSurfel > 0)	// debug
			//	break;
		}
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

	// Create edges between surfels.

	pSurfels->InitGetNeighbors();

	pSurfel = pSurfels->NodeArray.Element;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++, pSurfel++)
		pSurfels->GetNeighbors(iSurfel, pMesh, pMem);

	// Free memory

	pSurfels->FreeGetNeighbors();

	delete[] iSurfelSeed;
	delete[] regionGrowingBuffer;
	RVL_DELETE_ARRAY(data.costMap);
	RVL_DELETE_ARRAY(data.costBuffer);
	delete[] iPtBuff;
	delete[] iBoundaryPtBuff;
	delete[] RandPtIdxArray.Element;
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

	DefineBoundary(pMesh, pSurfels, data, iSurfel, iSurfel_, G);

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

	for (int i = 0; i < nPts; i++)
		if (map[i] != -1 || distanceMap[i] != 0xffffffff)
			int debug = 0;

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
	int iSurfel_,
	QList<QLIST::Index> &G)
{
#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
	bool bDebug = (iSurfel == debugDefineBoundaryiSurfel && iSurfel_ == debugDefineBoundaryiSurfel_);

	if (bDebug)
		int debug = 0;
#endif

#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
	// Compute intersection plane.

	IntersectionPlane(pSurfels, iSurfel, iSurfel_);
#endif

	//if (iSurfel == 8 && iSurfel_ == 44)
	//	int debug = 0;

	CRVLMem *pMem2A = &(Mem2A);

	int nMeshPts = pMesh->NodeArray.n;

	Array<int> seed;

	pMem2A->Clear();

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
					CutPropagation(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, PointEdgeArray, iSourceStart, iSourceEnd, iSinkStart, iSinkEnd, map, -1, piGEdgeBuffEnd);

					MinimumCut(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, PointEdgeArray, iSinkStart, iSinkEnd);

					piPt++;
				}
				else if (!bBWConnected)
				{
					BWConnect(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, PointEdgeArray, iPt_, piGBBndPtArrayEnd, piBWConnectionPtArrayEnd);

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

	/// Grow B-region util reaching the cut.

	int *iExpBPtBuff;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, nG + 1, iExpBPtBuff);

	PSD::ReassignToBData reassignToBData;

	reassignToBData.BID = iSurfel_;
	reassignToBData.GID = nSurfels;
	reassignToBData.edgeFlags = edgeFlags;
	reassignToBData.map = pSurfels->surfelMap;
	reassignToBData.pPSD = this;

	int *pSeedEnd = iPtBuff + seed.n;

	for (piPt = iPtBuff; piPt < pSeedEnd; piPt++)
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

	if (nWCC > 0)
	{
		// Reset edgeFlags and cutCostMap

		int *piEdge;

		for (piEdge = iGEdgeBuff; piEdge < piGEdgeBuffEnd; piEdge++)
		{
			edgeFlags[*piEdge] = 0x00;
			cutCostMap[*piEdge] = 0xffffffff;
		}
	}

//#ifdef RVLPLANARSURFELDETECTOR_DEBUG
//	// Only for debugging purpose!
//	// Check if all edgeFlags are set to 0x00 and whole cutCostMap to 0xffffffff
//
//	for (int i = 0; i < pMesh->EdgeArray.n; i++)
//		if (edgeFlags[i] != 0x00 || cutCostMap[i] != 0xffffffff)
//			int debug = 0;
//#endif

	// Reassign points to surfels.

	QList<QLIST::Index> *pW = &(pSurfel->PtList);
	//QList<QLIST::Index> *pG = &G;
	QList<QLIST::Index> *pB = &(pSurfel_->PtList);

	//RVLQLIST_INIT(pG);

	QLIST::Index *pPtIdx = pSurfel->PtList.pFirst;
	QLIST::Index **ppPtIdx = &(pSurfel->PtList.pFirst);

	int ID;

	while (pPtIdx)
	{
		//if (map[pPtIdx->Idx] >= 0 && distanceMap[pPtIdx->Idx] > 0)
		ID = pSurfels->surfelMap[pPtIdx->Idx];
		
		if (ID == iSurfel_)
		{
			RVLQLIST_REMOVE_ENTRY(pW, pPtIdx, ppPtIdx);
			RVLQLIST_ADD_ENTRY(pB, pPtIdx);		
			pPtIdx = (*ppPtIdx);
		}
		else
		{
			if (ID == nSurfels)
				pSurfels->surfelMap[pPtIdx->Idx] = iSurfel;

			ppPtIdx = &(pPtIdx->pNext);
			pPtIdx = pPtIdx->pNext;
		}
	}

	RVLMEM_SET_FREE(pMem2A, iGPt)

	pMem2B->Clear();
}

int PSD::DistanceOperation(
	int iNode,
	int iNode_,
	MeshEdge *pEdge,
	Mesh *pMesh,
	PSD::DistanceComputationData *pData)
{
	if (iNode == 142837)
		int debug = 0;

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

	QList<QLIST::Index> PtList;

	QList<QLIST::Index> *pPtList = &PtList;

	RVLQLIST_INIT(pPtList);

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
	
	FILE *fpDebugPts;
	FILE *fpDebugEdges;
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
	int *&piEdgeBuffEnd)
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

		// the first sink point-edge is the opposite of the first point-edge of the first point-edge of BW-segment.
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

		iEdge = pPtEdge_->pEdgePtr->pEdge->idx;

		if (edgeFlags[iEdge] == 0)
			*(piEdgeBuffEnd++) = iEdge;

		edgeFlags[iEdge] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << (1 - side));
	}

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	FILE *fpDebugPts;
	FILE *fpDebugEdges;
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
	int *map,
	int WID,
	int GID,
	int BID,
	Array<MESH::PointEdge> &BoundaryPointEdgeArray,
	int iSinkStart,
	int iSinkEnd)
{
	unsigned int minCutCost = 0xffffffff;

	MeshEdgePtr *pEdgePtr0 = NULL;

	int i;
	MESH::PointEdge *pPtEdge;
	int iEdge;
	int side0;

	for (i = iSinkStart; i < iSinkEnd; i++)
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

	FILE *fpDebugEdges;

	if (bDebug)
	{
		fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "a");

		fprintf(fpDebugEdges, "%d\t%d\t%d\t6\n", pEdge->idx, pEdge->iVertex[0], pEdge->iVertex[1]);
		fprintf(fpDebugEdges, "%d\t%d\t%d\t6\n", pEdge->idx, pEdge->iVertex[1], pEdge->iVertex[0]);

		fflush(fpDebugEdges);
	}
#endif

	edgeFlags[pEdge->idx] |= RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT;

	int side;	
	MeshEdge *pEdge_;
	MeshEdgePtr *pEdgePtr_, *pNextEdgePtr;
	int iPt, iNextPt;
	int side_;
	int ID;
	QList<MeshEdgePtr> *pEdgeList;
	unsigned int cutCost;
	int side1, side2;

	while (cutCostMap[pEdge->idx] > 1)
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

			RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr_, side_, map, iNextPt, pEdge_, ID, WID, GID, BID);

			pNextEdgePtr = NULL;

			while (pEdgePtr_ != pEdgePtr)
			{
				iPt = iNextPt;

				if (cutCostMap[pEdge_->idx] < cutCost)
				{
					pNextEdgePtr = pEdgePtr_;

					break;
				}

				RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr_, side_, map, iNextPt, pEdge_, ID, WID, GID, BID);
			}

			if (pNextEdgePtr)
				break;
		}

		if (pNextEdgePtr == NULL)
			return false;

		pEdgePtr = pNextEdgePtr;

		pEdge = pEdgePtr->pEdge;

		edgeFlags[pEdge->idx] |= RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
		if (bDebug)
		{
			fprintf(fpDebugEdges, "%d\t%d\t%d\t6\n", pEdge->idx, pEdge->iVertex[0], pEdge->iVertex[1]);
			fprintf(fpDebugEdges, "%d\t%d\t%d\t6\n", pEdge->idx, pEdge->iVertex[1], pEdge->iVertex[0]);

			fflush(fpDebugEdges);
		}
#endif
	}

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

void PlanarSurfelDetector::BWConnect(
	Mesh *pMesh,
	int *map,
	int WID,
	int GID,
	int BID,
	Array<MESH::PointEdge> &BoundaryPointEdgeArray,
	int &iGBPt,
	int *&iGBBndPtArrayEnd,
	int *&piBWConnectionEnd)
{
	// iPt0 <- the point on the boundary of G-region touching B-region closest to a W-region.

	iGBPt = -1;

	//int *iGBBndPt = iGBBndPtArrayEnd;

	unsigned int minDistance = 0xffffffff;

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
	int iPt0;

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

void PlanarSurfelDetector::GetNeighbors(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel)
{
	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	QLIST::Index *pPtIdx = pSurfel->PtList.pFirst;

	pNewNeighbor = neighborMem;

	QList<QLIST::Index> *pNeighborList = &neighborList;

	RVLQLIST_INIT(pNeighborList);

	pNewGSeedPt = GSeedMem;

	regionGrowingData.mode = RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK;

	Point PtTemplate;

	SURFEL::GetPoint(pSurfel, &PtTemplate);	// PtTemplate <- reference point for B-region created from pSurfel_

	regionGrowingData.pPtTemplate = &PtTemplate;

	regionGrowingData.iSurfel = 0;

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

		pNewGSeedPt++;
	}
}

void PlanarSurfelDetector::DefinePolygon(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel_)
{
	int nPts = pMesh->NodeArray.n;

	// neighborList <- neighbors of iSurfel_.

	GetNeighbors(pMesh, pSurfels, iSurfel_);

	// Define boundaries between iSurfel and all surfels in neighborList.

	PlanarSurfelDetectorRegionGrowingData data = regionGrowingData;

	QList<QLIST::Index> *pNeighborList = &neighborList;

	int iSurfel;
	QList<QLIST::Index> G;
	QList<QLIST::Index> *pGSeedPtList;
	QLIST::Index *pNeighbor;
	QLIST::Index **ppNeighbor;

	while (neighborList.pFirst)
	{
		// iSurfel <- the first neighbor in neighborList

		pNeighbor = neighborList.pFirst;

		iSurfel = pNeighbor->Idx;

		// Define boundary between iSurfel and iSurfel_.

		DefineBoundary(pMesh, pSurfels, data, iSurfel, iSurfel_, G);

		// Clear the seed points from the seed point list of iSurfel_

		pGSeedPtList = GSeedListArray.Element + iSurfel;

		// Remove the first neighbor from neighborList.

		RVLQLIST_INIT(pGSeedPtList);

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

	if (bDebug)
	{
		FILE *fpDebugPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "w");
		FILE *fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "w");
		fclose(fpDebugEdges);

		Surfel *pSurfel = pSurfels->NodeArray.Element + WID;

		QLIST::Index *pPtIdx = pSurfel->PtList.pFirst;
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

			fprintf(fpDebugPts, "%d\t%f\t%f\t%f\t2\n", iPt, pPt->P[0], pPt->P[1], pPt->P[2]);

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