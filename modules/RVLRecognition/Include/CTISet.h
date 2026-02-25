#pragma once

#define RVLCTIDESCRIPTOR_SAVE_IN_OLD_FORMAT

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
				int iVertex;
			};

			struct ModelInstance
			{
				int iModel; // VIDOVIC
				int iPart;
				int iCluster; // VIDOVIC
				int type;
				float R[9];
				float t[3];
				float tc[3]; // VIDOVIC
				float varX;
				Array<ModelInstanceElement> modelInstance;
				ModelInstance *pNext;
			};
		}

		class CTISet
		{
		public:
			CTISet();
			virtual ~CTISet();
			void Load(
				char * filePath,
				float scale = 1.0f);
			void Init();
			void AddCTI(RECOG::PSGM_::ModelInstance *pCTI_);
			void CopyCTIsToArray();
			//void LoadSMCTI(char * filePath, Array<RECOG::PSGM_::Plane> *convexTemplate); //Petra

		public:
			int nT;
			QList<RECOG::PSGM_::ModelInstance> CTI;
			Array<RECOG::PSGM_::ModelInstance*> pCTI;
			//Array<RECOG::PSGM_::ModelInstance> CTIArr;
			//std::vector<std::vector<int>> SegmentCTIs;
			Array<Array<int>> SegmentCTIs;
			int *segmentCTIIdxMem;
			int maxSegmentIdx;
			int nModels;
			int iCluster; //VN
			int iModel; //VN
		};
	}
}