#pragma once

namespace RVL
{
	class PSGM;

	namespace RECOG
	{
		namespace PSGM_
		{
			struct ModelInstanceElement
			{
				float d;
				float e;
				bool valid;
			};

			struct ModelInstance
			{
				int iModel; // VIDOVIC
				int iCluster; // VIDOVIC
				float R[9];
				float t[3];
				float tc[3]; // VIDOVIC
				Array<ModelInstanceElement> modelInstance;
				ModelInstance *pNext;
			};
		}

		class CTISet
		{
		public:
			CTISet();
			virtual ~CTISet();
			void Load(char * filePath);

		public:
			int nT;
			Array<RECOG::PSGM_::ModelInstance> CTI;
			//std::vector<std::vector<int>> SegmentCTIs;
			Array<Array<int>> SegmentCTIs;
			int *segmentCTIIdxMem;
		};
	}
}


