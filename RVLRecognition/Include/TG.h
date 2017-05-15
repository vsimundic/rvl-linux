#pragma once

#define RVLRECOG_TG_VERTEX_FLAG_MARKED		0x01

namespace RVL
{
	namespace RECOG
	{
		struct TGNode
		{
			float d;
			int i;
			int j;
			int iVertex;
			TGNode *pNext;
		};

		struct TGCorrespondence
		{
			TGNode *pNode;
			int iVertex;
			float e;
		};

		class TG
		{
		public:
			TG();
			virtual ~TG();
			void Create(
				SurfelGraph *pSurfels,
				Array<int> iVertexArray,
				float *R,
				float *t,
				void *vpSet);
			void Match(
				SurfelGraph *pSurfels,
				Array<int> iVertexArray,
				float scale,
				void *vpSet,
				float *R,
				float *t,
				float &score,
				Array<TGCorrespondence> &correspondences
				);
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
			//QList<TGNode> NodeList;
			Array2D<float> A;
			float R[9];
			float t[3];
			Array<QList<TGNode>> descriptor;
			int nNodes;
			int iObject;
			int iVertexGraph;
		};
	}
}


