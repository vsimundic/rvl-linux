#pragma once

#define RVLOBJECTDETECTION_FLAG_SAVE_PLY		0x00000001
#define RVLOBJECTDETECTION_FLAG_SAVE_SSF		0x00000002
#define RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT	0x00000004

namespace RVL
{
	class ObjectDetector;

	namespace OBJECT_DETECTION
	{
		struct TrainingHMIData
		{
			ObjectDetector *pObjectDetector;
			cv::Mat RGB;
			char *imageName;
			GRAPH::HierarchyNode *pObject;
			GRAPH::HierarchyNode *pObject2;
		};

		void Symmetry(
			SURFEL::ObjectGraph *pObjects, 
			int iObject1, 
			int iObject2, 
			void *vpData);

		void TrainingHMIMouseCallback(int event, int x, int y, int flags, void* vpData);
	}

	class ObjectDetector
	{
	public:
		ObjectDetector();
		virtual ~ObjectDetector();
		void Init(PSGM *pPSGM_ = NULL);
		void CreateParamList();
		void DetectObjects(char *MeshFilePathName);
		void Evaluate(
			FILE *fp,
			char *fileName,
			char *selectedGTObjectsFileName = NULL);
		void BoundingBox(
			int iObject1,
			int iObject2,
			RECOG::PSGM_::ModelInstance *pBoundingBox);
		static bool CheckIfWithinCTIBoundingBox(void * odObj, int iObject1, int iObject2, float dimThr = 0.30);	//Filko
		void GroundTruthGroundPlane();
		void SaveBoundingBoxSizes(char *imageFileName);
		void TrainingHMI(char *meshFileName);
		void DisplaySelectedObject(
			GRAPH::HierarchyNode *pObject,
			uchar *color,
			cv::Mat RGB);
		GRAPH::HierarchyNode * GetObject(int iPix);
		
	public:
		DWORD flags;
		CRVLParameterList ParamList;
		CRVLMem *pMem0;
		CRVLMem *pMem;
		char *SVMClassifierParamsFileName;
		float convexityThr;
		float convexityRatioThr1;
		float convexityRatioThr2;
		int nMultilateralFilterIterations;
		int joinSmallObjectsToLargestNeighborSizeThr;
		float joinSmallObjectsToLargestNeighborDistThr;
		bool bSegmentToObjects;
		bool bObjectAggregationLevel2;
		bool bSurfelsFromSSF;
		bool bCTIBasedObjectAggregation;
		bool bMultilateralFilter;
		bool bJoinSmallObjectsToLargestNeighbor;
		bool bGroundTruthSegmentation;
		bool bGroundTruthSegmentationOnSurfelLevel;
		bool bGroundTruthBoundingBoxes;
		bool bOwnsSurfelDetectionTool;
		bool bOwnsPSGM;
		bool bTrainingHMI;
		bool bDisplay;
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
		RECOG::CTISet CTIs;
		RECOG::CTISet boundingBoxes;
	};
}

