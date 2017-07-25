#pragma once

namespace RVL
{
	namespace RECOG
	{
		class TSMSet
		{
		public:
			TSMSet();
			virtual ~TSMSet();
			void Init(SurfelGraph *pSurfels);

		public:
			CRVLMem *pMem;
			Array<int> vertexArray;
			BYTE *mVertexFlags;
			Array2D<float> convexTemplate;
		};
	}
}



