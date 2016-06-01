#pragma once

#define RVLRECOGNITION_MODE_RECOGNITION			0
#define RVLRECOGNITION_MODE_TRAINING			1

#define RVLRFRECOGNITION_DEBUG
//#define RVLRFRECOGNITION_FEATURE_BASE_VISUALIZATION

namespace RVL
{
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
		};

		struct RFLinePoint
		{
			int iLine;
			int iPt;
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

			RVLCOPYMX3X3(R, R_);

			RVLMULMX3X3VECT(R, N, N_);

			float x = (d - RVLDOTPRODUCT3(N, P0)) / N_[0];

			float *X = R;

			RVLSCALE3VECTOR(X, x, t_);
			RVLSUM3VECTORS(P0, t_, t_);

			pFeature->cq = cq;

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
#endif
	}

	class RFRecognition
	{
	public:
		RFRecognition();
		virtual ~RFRecognition();
		void CreateParamList(CRVLMem *pMem);
		void CreateModelDatabase();
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
		CRVLMem *pMem;
		RECOG::RFFeatureDetectionParams featureDetectionParams;
		float kLineLength;
		float maxGap;
		float mincnx;
		int descriptorSize;
	private:
		unsigned char *markMap;
		int *iSurfBuff;
		int meshSize;
		int surfelGraphSize;
		RECOG::Line3D *lineMem;
	};

}

