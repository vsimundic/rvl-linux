#pragma once

//#define RVLTG_EDGES
//#define RVLTG_MATCH_DEBUG

#define RVLRECOG_TG_VERTEX_FLAG_MARKED		0x01

namespace RVL
{
	namespace QLIST
	{
		// Structure QLIST::SortIndex2 shoule be moved to RVLQListArray.h.

		template <typename T> struct SortIndex2
		{
			int Idx;
			T cost;
			SortIndex2<T> *pNext;
			SortIndex2<T> **pPtrToThis;
		};

		// Structure QLIST::TreeIndex2 shoule be moved to RVLQListArray.h.

		template <typename T> struct TreeIndex2
		{
			int Idx;
			void *vpEdge;
			T cost;
			TreeIndex2<T> *pNext;
			TreeIndex2<T> **pPtrToThis;
		};
	}

	namespace RECOG
	{
		struct TGEdge;

		struct TGNode
		{
			QList<GRAPH::EdgePtr2<TGEdge>> EdgeList;
			float d;
			int i;
			int j;
			int iVertex;
			TGNode *pNext;
		};

		struct TGEdge
		{
			int iVertex[2];
			GRAPH::EdgePtr2<TGEdge> *pVertexEdgePtr[2];
			int idx;
			TGEdge *pNext;
		};

		struct TGCorrespondence
		{
			TGNode *pNode;
			int iVertex;
			float e;
		};
		
		struct TangentVertexCorrespondence
		{
			int iTangent;
			int iVertex;
			float e;
		};
		
		struct TGConnectNodesRGData
		{
			BYTE *mFlags;
			float *N;
			float V[3];
			float csNThr;
			float csVThr;
			int iGoalNode;
			Array<int> iOutNodeArray;
			QLIST::TreeIndex2<float> *iNodeMap;
			BYTE *nOwners;
			SurfelGraph *pSurfels;
		};

		int ConnectNodesRG(
			int iVertex,
			int iParentVertex,
			SURFEL::VertexEdge *pEdge,
			VertexGraph *pVertexGraph,
			TGConnectNodesRGData *pData);
		void RotateTemplate(
			Array2D<float> A,
			float *R,
			float *A_);
		void TangentAlignment(
			SurfelGraph *pSurfels,
			Array<int> iVertexArray,
			float scale,
			Array2D<float> A,
			float *descriptor,
			float *RIn,
			float *tIn,
			float eThr,
			float &score,
			Array<TangentVertexCorrespondence> &correspondences,
			float *R,
			float *t);

#ifdef NEVER
		float ConnectNodesRG(
			int iVertex,
			int iParentVertex,
			SURFEL::VertexEdge *pEdge,
			VertexGraph *pVertexGraph,
			TGConnectNodesRGData *pData,
			bool &bGoalReached);

		float ConnectNodesRG2(
			int iVertex,
			int iParentVertex,
			SURFEL::VertexEdge *pEdge,
			VertexGraph *pVertexGraph,
			TGConnectNodesRGData *pData,
			bool &bGoalReached);
#endif

		class TG : public Graph < TGNode, TGEdge, GRAPH::EdgePtr2<TGEdge> >
		{
		public:
			TG();
			virtual ~TG();
			void Create(
				VertexGraph *pVertexGraph,
				Array<int> iVertexArray,
				float *R,
				float *t,
				void *vpSet,
				SurfelGraph *pSurfels,
				bool bForceMaxdNodes = false);
			void Match(
				SurfelGraph *pSurfels,
				Array<int> iVertexArray,
				float scale,
				void *vpSet,
				float *RIn,
				float *tIn,
				bool bConvexHullAllignment,
				float &score,
				Array<TGCorrespondence> &correspondences,
				float *ROut,
				float *tOut
				);
			void Save(
				FILE *fp,
				bool bSaveA = false);
			bool Load(
				FILE *fp,
				void *vpSet,
				bool bLoadA = false);


		public:
			Array2D<float> A;
			float R[9];
			float t[3];
			Array<QList<QLIST::Ptr<TGNode>>> descriptor;
			int iObject;
			int iVertexGraph;
			int nEdges;
		};
	}
}


