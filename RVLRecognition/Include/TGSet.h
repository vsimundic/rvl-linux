#pragma once

namespace RVL
{
	namespace RECOG
	{
		class TGSet
		{
		public:
			TGSet();
			virtual ~TGSet();
			void Init(SurfelGraph *pSurfels);

		public:
			CRVLMem *pMem;
			//Array<int> vertexArray;			
			//Array2D<float> convexTemplate;
		//private:
			//BYTE *mVertexFlags;
		};
	}
}

