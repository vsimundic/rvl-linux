#pragma once

#define RVLRECOG_TSM_VERTEX_FLAG_MARKED		0x01

namespace RVL
{
	namespace RECOG
	{
		struct TSMNode
		{
			float d;
			int i;
			TSMNode *pNext;
		};

		class TSM
		{
		public:
			TSM();
			virtual ~TSM();
			void Create(
				SurfelGraph *pSurfels,
				Array<int> iSurfelArray,
				float *R,
				float *t,
				void *vpSet);

		public:
			QList<TSMNode> NodeList;
		};
	}
}



