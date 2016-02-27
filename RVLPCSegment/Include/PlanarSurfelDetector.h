#pragma once

#define RVLPLANARSURFELDETECTOR_CONNECTED
//#define RVLPLANARSURFELDETECTOR_DIST_COST
//#define RVLPLANARSURFELDETECTOR_MIN_COST
//#define RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
#define RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_DEBUG

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
#define RVLPLANARSURFELDETECTOR_DEBUG
#else
#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
#define RVLPLANARSURFELDETECTOR_DEBUG
#endif
#endif

#define RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_SURFEL_DETECTION		0
#define RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_FIND_CLOSEST_INLIER	1
#define RVLPLANARSURFELDETECTOR_REGIONGROWING_MODE_ATTACK				2
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CUT			0x01
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_BOUNDARY		0x02
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_CLOSED		0x08
#define RVLPLANARSURFELDETECTOR_CUT_PROPAGATION_EDGE_FLAG_SINK			0x20

#define RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE(pEdgeList, iPt, pEdgePtr, side, map, iNeighborPt, pEdge, OppID, WID, GID, BID)\
{\
	do\
	{\
		RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr)\
		RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side)\
		OppID = map[iNeighborPt];\
	} while (OppID != WID && OppID != GID && OppID != BID);\
}

#define RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE_IN_LOOP(pMesh, pEdgeList, iPt, pEdgePtr, nextSide, map, iNextPt, pEdge, nextID, WID, GID, BID)\
{\
	pEdgeList = &(pMesh->NodeArray.Element[iPt].EdgeList);\
	RVLPLANARSURFELDETECTOR_GET_NEXT_EDGE(pEdgeList, iPt, pEdgePtr, nextSide, map, iNextPt, pEdge, nextID, WID, GID, BID);\
	nextSide = 1 - nextSide;\
	pEdgePtr = pEdge->pVertexEdgePtr[nextSide]; \
}

namespace RVL
{
	struct PlanarSurfelDetectorRegionGrowingData
	{
		float kRGB2;
		float kNormal2;
		float kPlane2;
		float distThr;
		Point *pPtTemplate;
		int iSurfel;
		int *surfelMap;
		int *buffer;
#ifndef RVLPLANARSURFELDETECTOR_SURFELS_CONNECTED
#ifndef RVLPLANARSURFELDETECTOR_DIST_COST
		float *costMap;
		float *costBuffer;
		int iAttackedSurfel;
#endif
#endif
		unsigned char mode;
		int iPtSeed;
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
		inline void VertexDist(
			Point *pPt1,
			Point *pPt2,
			float &distRGB,
			float &distN,
			float &distP)
		{
			int RGB1[3], RGB2[3];
			int V3Tmp[3];
			float dN[3];
			float dP[3];

			RVLCONVTOINT3(pPt1->RGB, RGB1);
			RVLCONVTOINT3(pPt2->RGB, RGB2);
			RVLDIF3VECTORS(RGB2, RGB1, V3Tmp);

			distRGB = (float)(RVLDOTPRODUCT3(V3Tmp, V3Tmp));

			RVLDIF3VECTORS(pPt2->N, pPt1->N, dN);

			distN = RVLDOTPRODUCT3(dN, dN);

			RVLDIF3VECTORS(pPt2->P, pPt1->P, dP);

			float eP = RVLDOTPRODUCT3(dP, pPt1->N);

			float distP1 = eP * eP;

			eP = RVLDOTPRODUCT3(dP, pPt2->N);

			float distP2 = eP * eP;

			distP = RVLMAX(distP1, distP2);
		}
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
			int iSurfel_,
			int *iPtBuff,
			int nBoundaryBPts,
			QList<QLIST::Index> &G);
		void DefineBoundaryTest(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int &iSurfel,
			int &iSurfel_,
			QList<QLIST::Index> &G);
	private:
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
			int *&piEdgeBuffEnd);
		bool MinimumCut(
			Mesh *pMesh,
			int *map,
			int WID,
			int GID,
			int BID,
			Array<MESH::PointEdge> &BoundaryPointEdgeArray,
			int iSinkStart,
			int iSinkEnd);
		void BWConnect(
			Mesh *pMesh,
			int *map,
			int WID,
			int GID,
			int BID,
			Array<MESH::PointEdge> &BoundaryPointEdgeArray,
			int &iGBPt,
			int *&iGBBndPtArrayEnd,
			int *&piBWConnectionEnd);
		void BBoundary(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSurfel_,
			int *&iPtBuff,
			int &nBoundaryPts);
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

	public:
		CRVLParameterList ParamList;
		float k;
		float kRGB;
		float kNormal;
		float kPlane;
		float surfelDistThr;
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
		MESH::PointEdge *PointEdgeBuff;
		QLIST::Index *BoundaryMem;
		unsigned int *cutCostMap;
		unsigned char *edgeFlags;
		int *iPointEdgeBuff;
		PlanarSurfelDetectorRegionGrowingData regionGrowingData;
	};
}


