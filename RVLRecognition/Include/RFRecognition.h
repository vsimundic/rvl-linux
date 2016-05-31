#pragma once

#define RVLRECOGNITION_MODE_RECOGNITION			0
#define RVLRECOGNITION_MODE_TRAINING			1

#define RVLRFRECOGNITION_DEBUG

namespace RVL
{
	namespace RECOG
	{
		struct RFFeature
		{
			float dp;
			float dl;
			float r;
			float q[3];
		};

		struct Line3D
		{
			float P[2][3];
			float length;
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
	}

	class RFRecognition
	{
	public:
		RFRecognition();
		virtual ~RFRecognition();
		void CreateParamList(CRVLMem *pMem);
		void CreateModelDatabase();

	public:
		CRVLParameterList ParamList;
		DWORD mode;
		int iRefSurfel;
		PlanarSurfelDetector *pSurfelDetector;
		SurfelGraph *pSurfels;
		Visualizer visualizer;
		char *modelMeshFileName;
		CRVLMem *pMem;
		RECOG::RFFeature feature;
		float kLineLength;
	};

}

