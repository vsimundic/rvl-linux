#pragma once

#define RVLPLANARSURFELDETECTOR_METHOD_SURFELS 0
#define RVLPLANARSURFELDETECTOR_METHOD_NSB 1

#define RVLPLANARSURFELDETECTOR_CONNECTED
// #define RVLPLANARSURFELDETECTOR_DIST_COST
// #define RVLPLANARSURFELDETECTOR_MIN_COST
// #define RVLPLANARSURFELDETECTOR_BIDIRECTIONAL_PLANARITY
#define RVLPLANARSURFELDETECTOR_POLYGONS
#ifdef RVLPLANARSURFELDETECTOR_POLYGONS
#ifndef RVLPLANARSURFELDETECTOR_CONNECTED
#define RVLPLANARSURFELDETECTOR_CONNECTED
#endif
#endif
#define RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
#define RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY
// #define RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY_2
#define RVLPLANARSURFELDETECTOR_LIMITED_DEPTH_UNCONSTRAINED_RG

#define RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG // RANDOM DATOTEKA
// #define RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
// #define RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
// #define RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
// #define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG

#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
#ifndef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
#define RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
#endif
#ifndef RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG
#endif
#ifndef RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
#define RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
#endif
#endif
#ifdef RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
#define RVL_RANDOM_DEBUG
#endif

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
#define RVLPLANARSURFELDETECTOR_DEBUG
#else
#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
#define RVLPLANARSURFELDETECTOR_DEBUG
#endif
#endif

#define RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION 0
#define RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER 1
#define RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK 2
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT 0x01
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SOURCE 0x02
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED 0x08
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK 0x20
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_INTERSECTION 0x80
#define RVLPLANARSURFELDETECTOR_PROCESSED_G 0x01
#define RVLPLANARSURFELDETECTOR_PROCESSED_SEED 0x02

#define RVLPLANARSURFELDETECTOR_VERSION_0 0

#define RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE(pEdgeList, iPt, pEdgePtr, side, map, iNeighborPt, pEdge, OppID, WID, GID, BID) \
	{                                                                                                                        \
		do                                                                                                                   \
		{                                                                                                                    \
			RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr)                                                                  \
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side)                                        \
			OppID = map[iNeighborPt];                                                                                        \
		} while (OppID != WID && OppID != GID && OppID != BID);                                                              \
	}

#define RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr, nextSide, map, iNextPt, pEdge, nextID, WID, GID, BID) \
	{                                                                                                                                        \
		pEdgeList = &(pMesh->NodeArray.Element[iPt].EdgeList);                                                                               \
		RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE(pEdgeList, iPt, pEdgePtr, nextSide, map, iNextPt, pEdge, nextID, WID, GID, BID);               \
		nextSide = 1 - nextSide;                                                                                                             \
		pEdgePtr = pEdge->pVertexEdgePtr[nextSide];                                                                                          \
	}

#define RVLPLANARSURFELDETECTOR_ON_LINE_CUT(N, d, P1, P2) ((RVLDOTPRODUCT3(N, P1) - d) * (RVLDOTPRODUCT3(N, P2) - d) <= 0)

// #ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
// #define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_PUSH_TO_BUFFER(pEdge, side, iPtEdge, P1, P2, N, d, cost, ppPushPlace, pCutPropagationBuff, pCutPropagationBuffEntry, edgeFlags, cutCostMap)\
//{\
//	if (!(edgeFlags[pEdge->idx] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (1 - side))))\
//	{\
//		P1 = pMesh->NodeArray.Element[pEdge->iVertex[0]].P; \
//		P2 = pMesh->NodeArray.Element[pEdge->iVertex[1]].P; \
//		if (RVLPLANARSURFELDETECTOR_ON_LINE_CUT(N, d, P1, P2))\
//		{\
//			pCutPropagationBuffEntry->pNext = *ppPushPlace; \
//			*ppPushPlace = pCutPropagationBuffEntry; \
//			cutCostMap[pEdge->idx] = cost; \
//		}\
//		else\
//		{\
//			RVLQLIST_ADD_ENTRY(pCutPropagationBuff, pCutPropagationBuffEntry)\
//			cutCostMap[pEdge->idx] = cost + 1; \
//		}\
//		pCutPropagationBuffEntry->Idx = iPtEdge; \
//		pCutPropagationBuffEntry++; \
//	}\
//}
// #else
// #define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_PUSH(pEdge, side, iPtEdge, cost, pCutPropagationBuff, pCutPropagationBuffEntry, edgeFlags, cutCostMap)\
//{\
//	if (!(edgeFlags[pEdge->idx] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (1 - side))))\
//	{\
//		RVLQLIST_ADD_ENTRY(pCutPropagationBuff, pCutPropagationBuffEntry);\
//		pCutPropagationBuffEntry->Idx = iPtEdge;\
//		pCutPropagationBuffEntry++;\
//	}\
//	cutCostMap[pEdge->idx] = cost + 1; \
//}
// #endif

namespace RVL
{
	class PlanarSurfelDetector;

	struct PlanarSurfelDetectorRegionGrowingData
	{
		float kRGB2;
		float kNormal2;
		float kPlane2;
		float distThr;
		Surfel *pTemplate;
		int iSurfel;
		int *surfelMap;
		int *buffer;
		int *depthMap;
		unsigned int *unconstrainedNormalDepthMap;
#ifndef RVLPLANARSURFELDETECTOR_SURFELS_CONNECTED
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
		float *costMap;
		float *costBuffer;
		int iAttackedSurfel;
		int GID;
#endif
#endif
		unsigned char mode;
		int iPtSeed;
		int maxSize;
		int size;
		int dSize;
		int *iPtBuff;
		int *iPtBuff2;
		int *iBoundaryPtBuff;
		Array<int> surfelPtArray;
		int *iSurfelSeed;
		bool bLimitedDepthUnconstrainedRG;
		int maxUnconstrainedNormalDepth;
	};

	namespace PSD
	{
		struct DistanceComputationData
		{
			int nRegions;
			unsigned int *distanceMatrix;
			MeshEdge **edgeMatrix;
			int *map;
			unsigned int *distanceMap;
		};

		struct ReassignToBData
		{
			int GID;
			int BID;
			unsigned char *edgeFlags;
			int *map;
			PlanarSurfelDetector *pPSD;
		};

		struct Interval
		{
			float min;
			float max;
		};

		struct Point2D
		{
			float P[2];
		};

		struct DisplayCallbackData
		{
			Visualizer *pVisualizer;
			Mesh *pMesh;
			vtkSmartPointer<vtkActor> selectedPtActor;
		};

		int RegionGrowingOperation(
			int iNode,
			int iNode_,
			MeshEdge *pEdge,
			Mesh *pMesh,
			PlanarSurfelDetectorRegionGrowingData *pData);
		int DistanceOperation(
			int iNode,
			int iNode_,
			MeshEdge *pEdge,
			Mesh *pMesh,
			DistanceComputationData *pData);
		int ReassignToB(
			int iNode,
			int iNode_,
			MeshEdge *pEdge,
			Mesh *pMesh,
			ReassignToBData *pData);
		void MouseRButtonDown(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata);
		template <typename T1, typename T2>
		inline void VertexDist(
			T1 *pPt1,
			T2 *pPt2,
			float &distRGB,
			float &distN,
			float &distP)
		{
			int V3Tmp[3];
			float dN[3];
			float dP[3];

			RVLDIF3VECTORS(pPt2->RGB, pPt1->RGB, V3Tmp);

			distRGB = (float)(RVLDOTPRODUCT3(V3Tmp, V3Tmp));

			RVLDIF3VECTORS(pPt2->N, pPt1->N, dN);

			distN = RVLDOTPRODUCT3(dN, dN);

			RVLDIF3VECTORS(pPt2->P, pPt1->P, dP);

			float eP = RVLDOTPRODUCT3(dP, pPt1->N);

#ifdef RVLPLANARSURFELDETECTOR_BIDIRECTIONAL_PLANARITY
			float distP1 = eP * eP;

			eP = RVLDOTPRODUCT3(dP, pPt2->N);

			float distP2 = eP * eP;

			distP = RVLMAX(distP1, distP2);
#else
			distP = eP * eP;
#endif
		}
		bool UpdateConvexSet(
			std::vector<PSD::Point2D> CIn,
			float *N,
			float d,
			std::vector<PSD::Point2D> &COut);
	}

	class PlanarSurfelDetector
	{
	public:
		PlanarSurfelDetector();
		virtual ~PlanarSurfelDetector();
		void Init(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			CRVLMem *pMem_);
		void CreateParamList(CRVLMem *pMem);
		void Segment(
			Mesh *pMesh,
			SurfelGraph *pSurfels);
		void RandomIndices(Array<int> &A);
		void DefineBoundary(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			PlanarSurfelDetectorRegionGrowingData &data,
			int iSurfel,
			int iSurfel_);
		void DefineBoundaryTest(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int &iSurfel,
			int &iSurfel_,
			QList<QLIST::Index> &G);
		void DefinePolygon(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSurfel);
		void AddToSeed(
			Mesh *pMesh,
			int iPt_,
			int iSurfel_);
		void GetNeighbors(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSurfel,
			QList<SURFEL::Edge> *pEdgeList,
			int &nEdges,
			bool bSmall = false);
		void JoinSmallSurfelsToClosestNeighbors(
			Mesh *pMesh,
			SurfelGraph *pSurfels);
		void CreatePolygons(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Mesh *pPolygonMesh);
		void Polygonalize(
			SurfelGraph *pSurfels,
			Array<MeshEdgePtr *> *pBoundary,
			int iContourStart,
			int iContourEnd,
			float *PStart,
			float *PEnd,
			int iSurfel,
			int iSurfel_);
		void MergeSurfels(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int minSurfelSize,
			int minEdgeSize,
			float maxCoPlanarSurfelNormalAngle,
			float maxCoPlanarSurfelRefPtDist,
			SurfelGraph *pPlanarSurfaces,
			Visualizer *pVisualizer = NULL);
		void DetectPlaneNSB(
			Mesh *pMesh,
			int iSeedPt,
			Array<int> &ptIdx);
		bool UpdateConvexSet(
			std::vector<PSD::Point2D> CIn,
			float *N,
			float d,
			std::vector<PSD::Point2D> &COut);
		float Area(std::vector<PSD::Point2D> C);
		void VisualizeConvexSet(
			std::vector<PSD::Point2D> C,
			int resolution,
			Rect<float> *pROI = NULL);
		void DisplaySoftEdges(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			unsigned char *Color);
		void Save(FILE *fp);

	private:
		void InitPlanarRegionGrowing(
			Mesh *pMesh,
			SurfelGraph *pSurfels);
		void PlanarRegionGrowing(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iPtSeed,
			int iSurfel);
		void FreePlanarRegionGrowingMem();
		bool GRegion(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			PlanarSurfelDetectorRegionGrowingData &data,
			int iSurfel,
			int iSurfel_,
			Array<int> &G,
			Array<int> &GBnd,
			Array<int> &WBnd,
			bool *&bPrevW,
			bool bGtoW = true);
		void ConnectedComponent(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iPt,
			int regionIdx,
			int componentIdx,
			int *&iPtArray,
			int *map,
			unsigned int *distanceMap);
		void Connect(
			Mesh *pMesh,
			MeshEdge *pEdge,
			int idx,
			int *map,
			unsigned int *distanceMap,
			int *tgtMap);
		bool MinimumSpanningTree(
			unsigned int *connection,
			int n,
			int *tree);
		void EdgeBoundary(
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
			int mark);
		void Boundaries(
			Mesh *pMesh,
			SurfelGraph *pSurfels);
		void EdgeFetures(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			QList<SURFEL::Edge> *pSEdgeList,
			int &nSEdges);
		int CreateEdgeFeatures(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Array<MeshEdgePtr *> *pBoundary,
			int iNewFeature,
			int &nOcclusionEdges,
			QList<SURFEL::Edge> *pSEdgeList,
			int &nSEdges,
			CRVLMem *pMem);
		void DeallocateMemory();
		void CutPropagation(
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
			int *&piEdgeBuffEndd,
			int &nPtEdges);
		// void LineCut(
		//	Mesh *pMesh,
		//	int *map,
		//	int WID,
		//	int GID,
		//	int BID,
		//	MESH::PointEdge *pPtEdge,
		//	int *markMap,
		//	int mark,
		//	int *&piEdgeBuffEnd);
		bool MinimumCut(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int WID,
			int GID,
			int BID,
			Array<MESH::PointEdge> &BoundaryPointEdgeArray,
			int iSinkStart,
			int iSinkEnd,
			int nPtEdges);
#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
		void IntersectionPlane(
			SurfelGraph *pSurfels,
			int iSurfel,
			int iSurfel_);
#endif
		inline void PushToCutPropagationBuffer(
			Point *Vertex,		// mesh vertex array
			MeshEdge *pEdge,	// edge used to create the point-edge which should be stored in cutPropagationBuff
			unsigned char side, // side of the edge corresponding to the point-edge which should be stored in cutPropagationBuff
			// QLIST::Index **ppiPtEdge,					// ptr. to ptr. to the current cutPropagationBuff entry (needed to push an entry right after the current entry)
			bool bCreateNewPtEdge,					 // if true, new point-edge is created
			unsigned int cost,						 // cost of the point-edge whose opposite point-edge should be created and stored in cutPropagationBuff
			QLIST::Index *&pCutPropagationBuffEntry, // ptr. to the next free place in the cut propagation buffer memory
			Array<MESH::PointEdge> PtEdgeArray,		 // storage of point-edges
			int &iNewPtEdge							 // idx. of the new point-edge in the storage of point-edges
		)
		{
			int iEdge = pEdge->idx;

#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
			float *P1 = Vertex[pEdge->iVertex[0]].P;
			float *P2 = Vertex[pEdge->iVertex[1]].P;

			unsigned int dCost, costMask;

			if (RVLPLANARSURFELDETECTOR_ON_LINE_CUT(NLineCut, dLineCut, P1, P2))
			{
				dCost = 1;
				costMask = 0xffffffff;
			}
			else
			{
				dCost = 0x10000;
				costMask = 0xffff0000;
			}
#else
			unsigned int dCost = 1;
#endif
			if (!(edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED << side)))
			{
				if (!(edgeFlags[iEdge] & (RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK << (1 - side))))
				{
					if (bCreateNewPtEdge)
					{
						MESH::PointEdge *pNewPtEdge = PtEdgeArray.Element + iNewPtEdge;

						pNewPtEdge->side = side;
						pNewPtEdge->iPt = pEdge->iVertex[side];
						pNewPtEdge->pEdgePtr = pEdge->pVertexEdgePtr[side];
					}

					if (dCost == 1)
					{
						// pCutPropagationBuffEntry->pNext = *ppiPtEdge;
						//*ppiPtEdge = pCutPropagationBuffEntry;

						QList<QLIST::Index> *pLineCutBuff = &lineCutBuff;

						RVLQLIST_ADD_ENTRY(pLineCutBuff, pCutPropagationBuffEntry);

						edgeFlags[iEdge] |= RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_INTERSECTION;
					}
					else
					{
						QList<QLIST::Index> *pCutPropagationBuff = &cutPropagationBuff;

						RVLQLIST_ADD_ENTRY(pCutPropagationBuff, pCutPropagationBuffEntry)
					}

					pCutPropagationBuffEntry->Idx = iNewPtEdge;
					pCutPropagationBuffEntry++;

					if (bCreateNewPtEdge)
						iNewPtEdge++;
				}

#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
				cutCostMap[iEdge] = (cost & costMask) + dCost;
#else
				cutCostMap[iEdge] = cost + dCost;
#endif
			}
		}
		bool BWConnect(
			Mesh *pMesh,
			int *map,
			int WID,
			int GID,
			int BID,
			Array<MESH::PointEdge> &BoundaryPointEdgeArray,
			int *&iGBBndPtArrayEnd,
			int *&piBWConnectionEnd);
		void BBoundary(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSurfel_,
			int *&iPtBuff,
			int &nBoundaryPts);
		void GetNeighbors(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSurfel);
		void ClearProcessed();
		void GetAttackSeed(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int WID,
			int GID,
			int BID,
			Array<int> &BBnd,
			Array<int> &G);
		int GetClosestNeighbor(
			SurfelGraph *pSurfels,
			int iSurfel,
			QList<SURFEL::Edge> &neighborList);
		void JoinSurfel(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSurfel,
			int iSurfel_);
#ifdef RVLPLANARSURFELDETECTOR_POLYGONALIZE_BOUNDARY
		void GetEdgeMidPoint(
			Mesh *pMesh,
			int iEdge,
			float *P,
			float *N,
			float d,
			bool bPlaneIntersection = true);
		bool InsideGRegion(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int WID,
			int GID,
			int BID,
			float *N,
			float d,
			int iEdgeStart,
			int sideStart,
			int iEdgeEnd,
			float *PEnd,
			float *dP,
			Array<int> &initCut,
			Array<int> &lineCut);
#endif
#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
		void SaveNeighborhood(
			Mesh *pMesh,
			int *map,
			int WID,
			int GID,
			int BID,
			int iPt,
			FILE *fpPts,
			FILE *fpEdges);
#endif
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
			bool *bMap);
#endif
#ifdef RVLPLANARSURFELDETECTOR_G_REGION_DEBUG
		void SaveWGB(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int WID,
			int GID,
			int BID);
		void SaveIdxArray(
			FILE *fp,
			Array<int> &Array);
#endif

	public:
		CRVLParameterList ParamList;
		float k;
		float kRGB;
		float kNormal;
		float kPlane;
		float surfelDistThr;
		int minSurfelSize;
		int minEdgeFeatureSize;
		float maxEdgeFeatureConcavity;
		float maxRange;
		int maxAttackSize;
		int maxUnconstrainedNormalDepth;
		int edgeClassHalfWinSize;
		float edgeClassDepthDiscontinuityThr;
		Rect<int> edgeClassImageBoundary;
		bool bJoinSmallSurfelsToClosestNeighbors;
		bool bNormalConstraintInSecondInitRG;
		bool bLimitedDepthUnconstrainedNormalRG;
		bool bBottom;
		bool bEdges;
		unsigned char *mProcessed;
		bool bGnd;
		bool bEdgeClassForegroundIsDefault;
		float NGnd[3];
		float dGnd;
		float zGndThr;
		int maxnSurfelsPerPlanarSurface;
		CRVLTimer *pTimer;
#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
		int debugDefineBoundaryiSurfel;
		int debugDefineBoundaryiSurfel_;
#endif
#ifdef RVLPLANARSURFELDETECTOR_DEBUG
		int *iPtBuffDebug;
		Array<int> debugPtArray;
		int debugState;
#endif

	private:
		CRVLMem *pMem;
		CRVLMem Mem2A;
		CRVLMem Mem2B;
		int *map;
		unsigned int *distanceMap;
		unsigned int *unconstrainedNormalDepthMap;
		MESH::PointEdge *PointEdgeBuff;
		QLIST::Index *BoundaryMem;
		unsigned int *cutCostMap;
		unsigned char *edgeFlags;
		QLIST::Index *cutPropagationBuffMem;
		QList<QLIST::Index> cutPropagationBuff;
		PlanarSurfelDetectorRegionGrowingData regionGrowingData;
		Array<int> processedBuff;
		QLIST::Index *GSeedMem;
		Array<QList<QLIST::Index>> GSeedListArray;
		QLIST::Index *pNewGSeedPt;
		QLIST::Index *neighborMem;
		QList<QLIST::Index> neighborList;
		QLIST::Index *pNewNeighbor;
#ifdef RVLPLANARSURFELDETECTOR_PLANE_INTERSECTION
		float NLineCut[3];
		float dLineCut;
		QList<QLIST::Index> lineCutBuff;
#endif
		PSD::Interval *edgeClassDepthOccupancy;
		Array<int> iEdgeClassDepthOccupancyBin;
		FILE *fpDebugPts;
		FILE *fpDebugEdges;
	};
}
