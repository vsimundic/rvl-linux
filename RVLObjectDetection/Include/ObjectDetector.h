#pragma once

#define RVLOBJECTDETECTION_FLAG_SAVE_PLY		0x00000001
#define RVLOBJECTDETECTION_FLAG_SAVE_SSF		0x00000002
#define RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT	0x00000004

namespace RVL
{
	class ObjectDetector
	{
	public:
		ObjectDetector();
		virtual ~ObjectDetector();
		void Init();
		void CreateParamList();
		void DetectObjects(char *MeshFilePathName);
		void Evaluate(
			FILE *fp,
			char *fileName);
		void CTIs();
		
	public:
		DWORD flags;
		CRVLParameterList ParamList;
		CRVLMem *pMem0;
		CRVLMem *pMem;
		char *SVMClassifierParamsFileName;
		float convexityThr;
		float convexityRatioThr1;
		float convexityRatioThr2;
		bool bSegmentToObjects;
		bool bObjectAggregationLevel2;
		bool bSurfelsFromSSF;
		bool bCTIBasedObjectAggregation;
		SurfelGraph *pSurfels;
		PlanarSurfelDetector *pSurfelDetector;
		SURFEL::ObjectGraph *pObjects;
		PSGM *pPSGM;
		Mesh mesh;
		char *cfgFileName;
		void *vpMeshBuilder;
		bool (*LoadMesh)(void *vpMeshBuilder,
			char *FileName,
			Mesh *pMesh,
			bool bSavePLY);
	};
}

