#pragma once

#define RVLRECOG_TG_VERTEX_FLAG_MARKED		0x01

//#define RVLTG_MATCH_DEBUG

namespace RVL
{
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
		
		struct TGConnectNodesRGData
		{
			BYTE *mFlags;
			float *N;
			float csNThr;
		};

		int ConnectNodesRG(
			int iVertex,
			int iParentVertex,
			SURFEL::VertexEdge *pEdge,
			VertexGraph *pVertexGraph,
			TGConnectNodesRGData *pData);

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
			void TransformVertices(
				SurfelGraph *pSurfels,
				Array<int> iVertexArray,
				float scale,
				float *R,
				float *t, 
				float *PArray);
			void RotateTemplate(
				float *R,
				float *A_);
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


