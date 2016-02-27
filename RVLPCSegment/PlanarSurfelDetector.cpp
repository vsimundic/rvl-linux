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

	pMem = NULL;
	//iPtBuff = NULL;
	map = NULL;
	distanceMap = NULL;
	PointEdgeBuff = NULL;
	BoundaryMem = NULL;
	cutCostMap = NULL;
	edgeFlags = NULL;
	iPointEdgeBuff = NULL;

#ifdef RVLPLANARSURFELDETECTOR_DEBUG
	iPtBuffDebug = NULL;
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

	PointEdgeBuff = new MESH::PointEdge[2 * nEdges];

	cutCostMap = new unsigned int[nEdges];

	memset(cutCostMap, 0xff, nEdges * sizeof(unsigned int));

	edgeFlags = new unsigned char[nEdges];

	memset(edgeFlags, 0, nEdges * sizeof(unsigned char));

	iPointEdgeBuff = new int[2 * nEdges];
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
	RVL_DELETE_ARRAY(iPointEdgeBuff);

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
			return -1;

		if (pData->buffer[iNode] >= 0)
			return 0;
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
	
		PSD::VertexDist(pPt_, pData->pPtTemplate, eRGB, eN, eP);

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
							return 1;
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

	return -1;
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

		if (surfelPtArray.n >= 20)
		{
			// Create surfel lists.

			pEdgeList = &(pSurfel->EdgeList);

			RVLQLIST_INIT(pEdgeList);

			pPtList = &(pSurfel->PtList);

			RVLQLIST_INIT(pPtList);

			// Compute surfel parameters

			pMesh->ComputeDistribution(surfelPtArray, distribution);

			pPt = Pt + iPtSeed;

			SURFEL::ComputeParameters(pSurfel, distribution, pPt);

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
				}

				regionGrowingBuffer[iPt] = -1;
			}

			// Next surfel index

			iSurfel++;

			pSurfel++;
		}
	}	// for every vertex

	pSurfels->NodeArray.n = iSurfel;

	// Assign points to surfels

	QLIST::Index *pPtIdx = pSurfels->PtMem;

	for (iPt = 0; iPt < nPts; iPt++)
	{
		iSurfel = pSurfels->surfelMap[iPt];

		if (iSurfel < 0)
			continue;

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pPtList = &(pSurfel->PtList);

		RVLQLIST_ADD_ENTRY(pPtList, pPtIdx);

		pPtIdx->Idx = iPt;

		pPtIdx++;
	}

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

	int *iPtBuff;
	int nBBoundaryPts;

	BBoundary(pMesh, pSurfels, iSurfel_, iPtBuff, nBBoundaryPts);

	DefineBoundary(pMesh, pSurfels, data, iSurfel, iSurfel_, iPtBuff, nBBoundaryPts, G);

	// debugging

	for (int i = 0; i < nPts; i++)
		if (map[i] != -1 || distanceMap[i] != 0xffffffff)
			int debug = 0;

	/////
}

// For a given mesh pMesh segmented to surfels pSurfels, this function determines the boundary between two neighboring surfels identifed by indices iSurfel and iSurfel_
// by reassigning some of the points of the surfel iSurfel to the surfel iSurfel_.
// An input to the function is also the set of boundary points of the surfel iSurfel_ stored in the first nBoundaryBPts elements of iPtBuff.
// 
// The function uses the following buffers (member variables of PlanarSurfelDetector):
// 
// iBoundaryPtBuff: 0 - iBoundaryGEnd-1: boundary of G-region
// PointEdgeBuff: 
// 
// The function uses the following temporary maps (member variables of PlanarSurfelDetector):
//
// map: at the beginning of the execution of DefineBoundary(), all elements must be set to -1.
//      at the end of the execution of DefineBoundary(), all elements are set to -1.
// distanceMap: at the beginning of the execution of DefineBoundary(), all elements must be set to 0xffffffff.
//              at the end of the execution of DefineBoundary(), all elements are set to 0xffffffff.

void PlanarSurfelDetector::DefineBoundary(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	PlanarSurfelDetectorRegionGrowingData &data,
	int iSurfel,
	int iSurfel_,
	int *iPtBuff,
	int nBBndPts,
	QList<QLIST::Index> &G)
{
	CRVLMem *pMem2A = &(Mem2A);

	int nMeshPts = pMesh->NodeArray.n;

	// B attacks iSurfel. G <- the regions of iSurfel conquered by B. W <- iSurfel \ G.

	int *iBBndPt = iPtBuff;

	int *iGPt = iPtBuff + nBBndPts;

	int *piPtFetch = iBBndPt;

	int *piPtPut = iGPt;	

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

	SURFEL::GetPoint(pSurfel_, &PtTemplate);

	data.pPtTemplate = &PtTemplate;
		
	int *piGPtArrayEnd = RegionGrowing3<Mesh, Point, MeshEdge, MeshEdgePtr, PlanarSurfelDetectorRegionGrowingData, PSD::RegionGrowingOperation>(pMesh, &data, piPtFetch, piPtPut,
		piGBndPtArrayEnd);

	int nG = piGPtArrayEnd - iGPt;	

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

	// Detect connected components of W.

	int nSurfels = pSurfels->NodeArray.n;

	int *piPt;

	for (piPt = iGPt; piPt < piGPtArrayEnd; piPt++)
		pSurfels->surfelMap[*piPt] = nSurfels;

	int nWCC = 0;

	//int *iWCCPtArray = piGEnd;
	int *iWCCPt;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, nMeshPts, iWCCPt);

	int *iWCCPtArrayEnd = iWCCPt;

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
	debugState = 0;
#endif

	int iPt, iPt_;
	MeshEdgePtr *pEdgePtr;
	MeshEdge *pEdge;

	for (piPt = iGBndPt; piPt < piGBndPtArrayEnd; piPt++)
	{
		iPt = *piPt;

		if (pSurfels->surfelMap[iPt] == nSurfels)
		{
			pEdgePtr = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

				if (pSurfels->surfelMap[iPt_] == iSurfel)
					if (map[iPt_] < 0)
					{
						nWCC++;

						ConnectedComponent(pMesh, pSurfels, iPt_, iSurfel, nWCC, iWCCPtArrayEnd, map, distanceMap);
					}

				pEdgePtr = pEdgePtr->pNext;
			}
		}
	}

	int *iGEdgeBuff;
	int *piGEdgeBuffEnd;

	if (nWCC > 0)
	{
		// iWCC = iWCCPtArray - iPtBuff;		// documentation
		//int iWCCPtArrayEnd = iWCCPtArrayEnd - iPtBuff;

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
		debugPtArray.Element = iWCCPtArray;
		debugPtArray.n = iWCCPtArrayEnd - iWCCPtArray;

		nWCC = 1;
#endif

		// If there are multiple connected components of W-region, then connect them into a single connected component.

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
		}

		delete[] distanceMatrix;
		delete[] edgeMatrix;

		// Determine the boundary between surfels iSurfel and iSurfel_ using cut propagation		

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2B, int, pMesh->EdgeArray.n, iGEdgeBuff);

		piGEdgeBuffEnd = iGEdgeBuff;

		for (piPt = iGBndPt; piPt < piGBndPtArrayEnd; piPt++)
		{
			iPt = *piPt;

			if (pSurfels->surfelMap[iPt] != nSurfels)
				continue;

			if (map[iPt] < 0)
				continue;

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

				//

				RVLMEM_ALLOC_LOCAL_INIT(pMem2A);

				int *iGBBndPt;

				RVLMEM_ALLOC_STRUCT_ARRAY(pMem2A, int, PointEdgeArray.n + nG, iGBBndPt);

				int *piGBBndPtArrayEnd = iGBBndPt;

				int *iBWConnectionPt = iGBBndPt + PointEdgeArray.n;

				int *piBWConnectionPtArrayEnd = iBWConnectionPt;

				// iBWBoundary_ = iBWBoundary - iPtBuff;		// documentation
				// iBWConnection_ = iBWConnection - iPtBuff;	// documentation

				if (!bW)
				{
					BWConnect(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, PointEdgeArray, iPt_, piGBBndPtArrayEnd, piBWConnectionPtArrayEnd);

					if (iPt_ >= 0)
						EdgeBoundary(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, iPt_, PointEdgeArray, iSourceStart, iSourceEnd, iSinkStart, iSinkEnd, bB, bW, map, -1);
				}

				CutPropagation(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, PointEdgeArray, iSourceStart, iSourceEnd, iSinkStart, iSinkEnd, map, -1, piGEdgeBuffEnd);

				MinimumCut(pMesh, pSurfels->surfelMap, iSurfel, nSurfels, iSurfel_, PointEdgeArray, iSinkStart, iSinkEnd);

				int *piPt_;

				for (piPt_ = iBWConnectionPt; piPt_ < piBWConnectionPtArrayEnd; piPt_++)
					pSurfels->surfelMap[*piPt_] = nSurfels;

				RVLMEM_ALLOC_LOCAL_UPDATE(pMem2A);

				//RVL_DELETE_ARRAY(iBuffExt);

				//break;
			}
		}

		// Reset map and distanceMap.

		for (piPt = iWCCPt; piPt < iWCCPtArrayEnd; piPt++)
		{
			map[*piPt] = -1;
			distanceMap[*piPt] = 0xffffffff;
		}
	}	// if(nWCC > 0)

	// Reset map and distanceMap.

	for (piPt = iGPt; piPt < piGPtArrayEnd; piPt++)
	{
		map[*piPt] = -1;
		distanceMap[*piPt] = 0xffffffff;
	}

#ifdef RVLPLANARSURFELDETECTOR_DEBUG
	// Only for debugging purpose!
	// Check if whole map is set to -1 and whole distanceMap to 0xffffffff

	for (int i = 0; i < pMesh->NodeArray.n; i++)
		if (map[i] != -1 || distanceMap[i] != 0xffffffff)
			int debug = 0;
#endif

	// Grow B-region util reaching the cut.

	PSD::ReassignToBData reassignToBData;

	reassignToBData.BID = iSurfel_;
	reassignToBData.GID = nSurfels;
	reassignToBData.edgeFlags = edgeFlags;
	reassignToBData.map = pSurfels->surfelMap;

	piPtFetch = iBBndPt;

	piPtPut = iGPt;

	int *piBPtArrayEnd = RegionGrowing<Mesh, Point, MeshEdge, MeshEdgePtr, PSD::ReassignToBData, PSD::ReassignToB>(pMesh, &reassignToBData, piPtFetch, piPtPut);

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

#ifdef RVLPLANARSURFELDETECTOR_DEBUG
	// Only for debugging purpose!
	// Check if all edgeFlags are set to 0x00 and whole cutCostMap to 0xffffffff

	for (int i = 0; i < pMesh->EdgeArray.n; i++)
		if (edgeFlags[i] != 0x00 || cutCostMap[i] != 0xffffffff)
			int debug = 0;
#endif

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

		RVLMESH_GET_NEXT_BOUNDARY_POINT(iPt, pEdge, 1 - side, pEdgePtr);
		RVLMESH_GET_NEXT_BOUNDARY_EDGE(pMesh, iPt, pEdgePtr, side, pSurfels->surfelMap, iNeighborPt, pPt, pEdgeList, pEdge);

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
	FILE *fpDebugPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "w");
	FILE *fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "w");

	Point *pPtDebug = pMesh->NodeArray.Element + iPt0;

	fprintf(fpDebugPts, "%d\t%f\t%f\t%f\t0\n", iPt0, pPtDebug->P[0], pPtDebug->P[1], pPtDebug->P[2]);

	SaveNeighborhood(pMesh, map, WID, GID, BID, iPt0, fpDebugPts, fpDebugEdges);

	int debugCounter = 0;
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
			fclose(fpDebugPts);
			fclose(fpDebugEdges);
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
		Point *pPtDebug = pMesh->NodeArray.Element + iOppPt;

		fprintf(fpDebugPts, "%d\t%f\t%f\t%f\t%d\n", iOppPt, pPtDebug->P[0], pPtDebug->P[1], pPtDebug->P[2], (OppID == GID ? 0 : (OppID == WID ? 1 : 2)));

		fprintf(fpDebugEdges, "%d\t%d\t1\n", iPt, iOppPt);

		fflush(fpDebugPts);
		fflush(fpDebugEdges);

		debugCounter++;

		if (debugCounter % 10 == 0)
			int debug = 0;
#endif

		if (OppID == GID)
		{
			iPt = iOppPt;
			pPt = pMesh->NodeArray.Element + iPt;
			pEdgeList = &(pPt->EdgeList);
			pEdgePtr = pEdge->pVertexEdgePtr[1 - side];

			markMap[iPt] = mark;

#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
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
	if (bW && bB)
	{
		for (int iPointEdge = iSourceStart; iPointEdge <= iSourceEnd; iPointEdge++)
		{
			pPointEdge = PointEdgeArray.Element + iPointEdge;

			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(pPointEdge->iPt, pPointEdge->pEdgePtr, pEdge, iOppPt);
	
			if (iPointEdge == iSourceStart)
				fprintf(fpDebugEdges, "%d\t%d\t2\n", iOppPt, pPointEdge->iPt);
			else
				fprintf(fpDebugEdges, "%d\t%d\t2\n", pPointEdge->iPt, iOppPt);
		}
		
		for (int iPointEdge = iSinkStart; iPointEdge <= iSinkEnd; iPointEdge++)
		{
			pPointEdge = PointEdgeArray.Element + iPointEdge;

			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(pPointEdge->iPt, pPointEdge->pEdgePtr, pEdge, iOppPt);

			if (iPointEdge == iSinkStart)
				fprintf(fpDebugEdges, "%d\t%d\t3\n", iOppPt, pPointEdge->iPt);
			else
				fprintf(fpDebugEdges, "%d\t%d\t3\n", pPointEdge->iPt, iOppPt);
		}
	}

	fclose(fpDebugPts);
	fclose(fpDebugEdges);
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

			fprintf(fpEdges, "%d\t%d\t0\n", iPt, iOppPt);
		}

		pEdgePtr = pEdgePtr->pNext;
	}

	fflush(fpPts);
	fflush(fpEdges);
}
#endif

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
	int i;
	MESH::PointEdge *pPtEdge;
	int iEdge;

	for (i = iSinkStart; i <= iSinkEnd; i++)
	{
		pPtEdge = BoundaryPointEdgeArray.Element + i;

		iEdge = pPtEdge->pEdgePtr->pEdge->idx;

		if (edgeFlags[iEdge] == 0)
			*(piEdgeBuffEnd++) = iEdge;

		edgeFlags[iEdge] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (i == iSinkStart ? 1 - pPtEdge->side : pPtEdge->side));
	}

	pPtEdge = BoundaryPointEdgeArray.Element + BoundaryPointEdgeArray.n;

	int *piPut = iPointEdgeBuff;

	MeshEdge *pEdge;
	MESH::PointEdge *pPtEdge_;
	int side;

	for (i = iSourceStart; i <= iSourceEnd; i++)
	{
		pPtEdge_ = BoundaryPointEdgeArray.Element + i;

		pEdge = pPtEdge_->pEdgePtr->pEdge;
		iEdge = pEdge->idx;

		cutCostMap[iEdge] = 1;

		if (i == iSourceStart)
		{
			side = 1 - pPtEdge_->side;

			if (!(edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (1 - side))))
			{				
				pPtEdge->side = side;
				pPtEdge->iPt = pEdge->iVertex[side];
				pPtEdge->pEdgePtr = pEdge->pVertexEdgePtr[side];				

				*(piPut++) = pPtEdge - BoundaryPointEdgeArray.Element;

				pPtEdge++;
			}
		}
		else
		{
			side = pPtEdge_->side;

			if (!(edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (1 - side))))
				*(piPut++) = i;
		}

		if (edgeFlags[iEdge] == 0)
			*(piEdgeBuffEnd++) = iEdge;

		edgeFlags[iEdge] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << (1 - side));
	}

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	FILE *fpDebugPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "a");
	FILE *fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "a");

	bool *bDebugMap = new bool[pMesh->NodeArray.n];

	memset(bDebugMap, 0, pMesh->NodeArray.n * sizeof(bool));

	unsigned int debugDepth = 0;

	int debugLoopCounter;
#endif

	int *piFetch = iPointEdgeBuff;

	MeshEdgePtr *pEdgePtr, *pEdgePtr0;
	QList<MeshEdgePtr> *pEdgeList;
	int iPt;
	int iNextPt, iPrevPt;	
	int ID;
	int iEdge_;

	while (piFetch < piPut)
	{
		pPtEdge_ = BoundaryPointEdgeArray.Element + (*piFetch);

		piFetch++;

		pEdgePtr0 = pPtEdge_->pEdgePtr;

		iEdge = pEdgePtr0->pEdge->idx;

		if (edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << pPtEdge_->side))
			continue;

		if (edgeFlags[iEdge] == 0)
			*(piEdgeBuffEnd++) = iEdge;

		edgeFlags[iEdge] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << pPtEdge_->side);

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
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

		//if (iEdge == 37367)
		//	int debug = 0;
#endif

		pEdgePtr = pEdgePtr0;
		iPt = pPtEdge_->iPt;

		markMap[iPt] = mark;

		RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr, side, map, iNextPt, pEdge, ID, WID, GID, BID);

		while (pEdgePtr != pEdgePtr0)
		{
			iPrevPt = iPt;
			iPt = iNextPt;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
			debugLoopCounter++;
#endif

			if (map[iPrevPt] == GID || map[iPt] == GID)
			{
				iEdge_ = pEdge->idx;

				if (edgeFlags[iEdge_] == 0)
					*(piEdgeBuffEnd++) = iEdge_;

				edgeFlags[iEdge_] |= (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << side);

				if (map[iPt] == GID)
					markMap[iPt] = mark;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
				SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdge, side, 5, bDebugMap);

				fflush(fpDebugPts);
				fflush(fpDebugEdges);
#endif

				if (!(edgeFlags[iEdge_] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << (1 - side))))
				{
					if (!(edgeFlags[iEdge_] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << side)))
					{
						pPtEdge->side = 1 - side;
						pPtEdge->iPt = pEdge->iVertex[pPtEdge->side];						
						pPtEdge->pEdgePtr = pEdge->pVertexEdgePtr[pPtEdge->side];

						*(piPut++) = pPtEdge - BoundaryPointEdgeArray.Element;

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
						SaveEdge(fpDebugPts, fpDebugEdges, pMesh, map, WID, GID, BID, pEdge, pPtEdge->side, 4, bDebugMap);

						fflush(fpDebugPts);
						fflush(fpDebugEdges);
#endif
						pPtEdge++;
					}
						
					cutCostMap[iEdge_] = cutCostMap[iEdge] + 1;					
				}
			}

			RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr, side, map, iNextPt, pEdge, ID, WID, GID, BID);
		}

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
		if (debugLoopCounter >= 10)
			int debug = 0;
#endif
	}

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	fclose(fpDebugPts);
	fclose(fpDebugEdges);

	delete[] bDebugMap;
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
	FILE *fpDebugEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "a");

	fprintf(fpDebugEdges, "%d\t%d\t6\n", pEdge->iVertex[0], pEdge->iVertex[1]);
	fprintf(fpDebugEdges, "%d\t%d\t6\n", pEdge->iVertex[1], pEdge->iVertex[0]);

	fflush(fpDebugEdges);
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
		fprintf(fpDebugEdges, "%d\t%d\t6\n", pEdge->iVertex[0], pEdge->iVertex[1]);
		fprintf(fpDebugEdges, "%d\t%d\t6\n", pEdge->iVertex[1], pEdge->iVertex[0]);

		fflush(fpDebugEdges);
#endif
	}

#ifdef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
	fclose(fpDebugEdges);
#endif

	return true;
}

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
	// iPt <- the point on the boundary of G-region touching B-region closest to a W-region.

	iGBPt = -1;

	int *iGBBndPt = iGBBndPtArrayEnd;

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

			*(iGBBndPtArrayEnd++) = iPt;
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

	// iPtGB <- G-point whose neighbor is a B-point

	for (piGBPt = iGBBndPt; piGBPt < iGBBndPtArrayEnd; piGBPt++)
		if (map[*piGBPt] == GID)
		{
			iGBPt = (*piGBPt);

			break;
		}
			
	if (iGBPt < 0)
	{
		for (piGBPt = iBWConnection; piGBPt < piBWConnectionEnd; piGBPt++)
		{
			iPt = (*piGBPt);

			pEdgePtr = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side);

				if (map[iNeighborPt] == GID)
				{
					iGBPt = iNeighborPt;

					break;
				}					

				pEdgePtr = pEdgePtr->pNext;
			}

			if (iGBPt >= 0)
				break;
		}		
	}
}

int PSD::ReassignToB(
	int iNode,
	int iNode_,
	MeshEdge *pEdge,
	Mesh *pMesh,
	PSD::ReassignToBData *pData)
{
	if (pData->map[iNode] != pData->GID)
		return 0;

	if (pData->edgeFlags[pEdge->idx] & RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT)
		return 0;

	pData->map[iNode] = pData->BID;

	return 1;
}

void PlanarSurfelDetector::BBoundary(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSurfel_,
	int *&iPtBuff,
	int &nBoundaryPts)
{
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	QList<QLIST::Index> Boundary;

	pMesh->Boundary(&(pSurfel_->PtList), pSurfels->surfelMap, &Boundary, BoundaryMem);

	Array<int> Boundary_;

	CRVLMem *pMem = &Mem2A;

	pMem->Clear();

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, pMesh->NodeArray.n, iPtBuff);

	Boundary_.Element = iPtBuff;

	QLIST::CopyToArray(&Boundary, &Boundary_);

	nBoundaryPts = Boundary_.n;
}