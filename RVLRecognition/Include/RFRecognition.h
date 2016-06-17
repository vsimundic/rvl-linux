#pragma once

#define RVLRECOGNITION_MODE_RECOGNITION			0
#define RVLRECOGNITION_MODE_TRAINING			1

#define RVLRFRECOGNITION_DEBUG
#define RVLRFRECOGNITION_FEATURE_BASE_VISUALIZATION

namespace RVL
{
	class RFRecognition;

	namespace RECOG
	{
		struct RFFeatureDetectionParams
		{
			float dp;
			float dl;
			float r;
		};

		struct RFDescriptor
		{
			Array<OrientedPoint> PtArray;
		};

		struct RFFeature
		{
			int objectID;
			int frameID;
			float cq;
			float N[3];
			float R[9];
			float t[3];
			RECOG::RFDescriptor descriptor;
			RECOG::RFFeature *pNext;
		};

		struct Line3D
		{
			float P[2][3];
			float length;
			int iSurfel;
		};

		struct RFFeatureBase
		{
			int iSurfel;
			Array<RECOG::Line3D> lineArray[2];
		};

		struct RFRegionGrowingData
		{
			Mesh *pMesh;
			SurfelGraph *pSurfels;
			unsigned char *markMap;
			float *N;
			float d0;
			float dp;
			Array<RECOG::Line3D> *lineArray;
			Array<float> *psBuff;
			int *piVisited;
		};

		struct RFLinePoint
		{
			int iLine;
			int iPt;
		};

		struct Hypothesis
		{
			int objectID;
			int frameID;
			float R[9];
			float t[3];
			float probability; //VIDOVIC
			Hypothesis *pNext;
		};

		struct RFRecognitionCallbackData
		{
			RFRecognition *pRecognition;
		};

		bool SurfelCylinderIntersection(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSurfel,
			float *N,
			float d0,
			float dp,
			float r,
			Array<RECOG::Line3D> *lineArray,
			Array<float> *psBuff);
		int RegionGrowingOperation(
			int iNode,
			int iNode_,
			SURFEL::Edge *pEdge,
			SurfelGraph *pSurfels,
			RECOG::RFRegionGrowingData *pData
			);
		int AdjustVoxelBoxSide(
			float minSrc,
			float maxSrc,
			float voxelSize,
			float &minTgt,
			float &maxTgt);
		void WriteHypothesis(
			FILE *fp,
			RECOG::Hypothesis *pHypothesis);
		//VIDOVIC
		void WriteHypothesisError(
			FILE *fp,
			RECOG::Hypothesis *pHypothesis,
			float positionError,
			float angleError);
		//END VIDOVIC

		inline int IdentifyVolume(
			float *P,
			float *N,
			float d0,
			float dp)
		{
			float d = RVLDOTPRODUCT3(N, P) - d0;

			return (d <= -dp ? -1 : (d >= dp ? 1 : 0));
		}

		inline RECOG::RFFeature * CreateFeature(
			float *N,
			float d,
			float *R,
			float *P0,
			float dl,
			float cq,
			QList<RECOG::RFFeature> *pFeatureList,
			CRVLMem *pMem)
		{
			RECOG::RFFeature *pFeature;

			RVLMEM_ALLOC_STRUCT(pMem, RECOG::RFFeature, pFeature);

			RVLQLIST_ADD_ENTRY(pFeatureList, pFeature);

			float *R_ = pFeature->R;
			float *t_ = pFeature->t;
			float *N_ = pFeature->N;

			RVLCOPYMX3X3T(R, R_);

			RVLMULMX3X3VECT(R, N, N_);

			float x = (d - RVLDOTPRODUCT3(N, P0)) / N_[0];

			float *X = R;

			RVLSCALE3VECTOR(X, x, t_);
			RVLSUM3VECTORS(P0, t_, t_);

			pFeature->cq = cq;
			pFeature->objectID = -1;
			pFeature->frameID = -1;

			return pFeature;
		}

#ifdef RVLRFRECOGNITION_DEBUG
		void DebugWriteFeature(
			FILE *fp,
			RECOG::RFFeature *pFeature,
			float axisLength);
		void DebugWriteDescriptor(
			FILE *fp,
			RECOG::RFFeature *pFeature,
			float normalLength);
		void DetectAndWriteFeatures(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSelectedPt,
			int iSelectedSurfel,
			void *vpData);
#endif
	}

	class RFRecognition
	{
	public:
		RFRecognition();
		virtual ~RFRecognition();
		void CreateParamList(CRVLMem *pMem);
		void CreateModelDatabase();
		bool LoadModelDatabase();
		void FindObjects(Mesh *pMesh);
		void Init(
			Mesh *pMesh,
			SurfelGraph *pSurfels);
		void DetectFeatureBase(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iRefSurfel,
			RECOG::RFFeatureBase *pFeatureBase);
		void DetectFeatures(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			RECOG::RFFeatureBase *pFeatureBase,
			int iRefLineArray,
			int iRefLine,
			QList<RECOG::RFFeature> *pFeatureList,
			CRVLMem *pMem_);
		void CreateDescriptor(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			RECOG::RFFeature *pFeature,
			CRVLMem *pMem);
		void SaveFeature(
			FILE *fp,
			RECOG::RFFeature *pFeature);
		void LoadFeature(
			FILE *fp,
			RECOG::RFFeature *pFeature,
			CRVLMem *pMem_);
		void Voxels(Mesh *pMesh);
		void MatchDescriptors(
			Mesh *pSMesh,
			RECOG::RFFeature *pSFeature,
			RECOG::RFFeature *pMFeature,
			int &matchScore);
		void RadiusSearch(
			Mesh *pMesh, 
			float *P,
			float r2);
		inline void GetVoxel(float *P, int &ix, int &iy, int &iz)
		{
			ix = (int)floor((P[0] - voxelBox.minx) / voxelSize);
			iy = (int)floor((P[1] - voxelBox.miny) / voxelSize);
			iz = (int)floor((P[2] - voxelBox.minz) / voxelSize);
		}
		void FindBestHypothesis(RECOG::Hypothesis **pBestHypothesis); //VIDOVIC

	public:
		CRVLParameterList ParamList;
		DWORD mode;
		int iRefSurfel;
		int iRefSurfel2;
		PlanarSurfelDetector *pSurfelDetector;
		SurfelGraph *pSurfels;
		Visualizer visualizer;
		char *modelMeshFileName;
		char *featureFileName;
		CRVLMem *pMem0;
		CRVLMem *pMem;
		RECOG::RFFeatureDetectionParams featureDetectionParams;
		float kLineLength;
		float maxGap;
		float mincnx;
		int descriptorSize;
		float voxelSize;
		int minRefSurfelSize;
		float minRefLineSize;
		float eNThr;
		float ePThr;
		float matchThr;
		QList<RECOG::RFFeature> modelFeatureList;
		QList<RECOG::Hypothesis> sceneInterpretation;
	private:
		unsigned char *markMap;
		int *iSurfBuff;
		int *iSurfBuff2;
		int meshSize;
		int surfelGraphSize;
		RECOG::Line3D *lineMem;
		Array3D<QList<QLIST::Index>> voxel;
		QLIST::Index *voxelMem;
		Box<float> voxelBox;
		int maxnPtsPerVoxel;
		Array<int> voxel8Pack;
	};

}

