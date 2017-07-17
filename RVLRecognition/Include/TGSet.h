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
			void Clear();
			void Save(char *fileName);
			void Load(char *fileName);
			VertexGraph *GetVertexGraph(TG *pTG);
			TG *GetTG(int ID);

		public:
			CRVLMem *pMem;
			std::vector<VertexGraph *> vertexGraphs;
			std::vector<TG *> TGs;
			float nodeSimilarityThr;
			float eLimit;
			Array2D<float> A;
			//Array<int> vertexArray;			
			//Array2D<float> convexTemplate;
		//private:
			//BYTE *mVertexFlags;
		};
	}
}

