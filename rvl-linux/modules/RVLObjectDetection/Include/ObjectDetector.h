#pragma once

#define RVLOBJECTDETECTION_FLAG_SAVE_PLY		0x00000001
#define RVLOBJECTDETECTION_FLAG_SAVE_SSF		0x00000002
#define RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT	0x00000004
#define RVLOBJECTDETECTION_METHOD_FUZZY_WER				0
#define RVLOBJECTDETECTION_METHOD_CONVEX_AND_CONCAVE	1

#define RVLOBJECTDETECTION_DISPLAY_ECMR19

namespace RVL
{
	class ObjectDetector;

	namespace OBJECT_DETECTION
	{
		struct VisualizationData
		{
			bool bVisualizeConvexClusters;
			bool bVisualizeConcaveClusters;
			bool bVisualizeSurfels;
			bool bVisualizeAllClusters;
			bool bVisualizeConvexHulls;
			RECOG::ClusterVisualizationData clusterVisualizationData;
		};

		struct TrainingHMIData
		{
			ObjectDetector *pObjectDetector;
			cv::Mat RGB;
			char *imageName;
			GRAPH::HierarchyNode *pObject;
			GRAPH::HierarchyNode *pObject2;
			int iPix;
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
		void Init(
			PSGM *pPSGM_ = NULL,
			void *vpVNClassifier_ = NULL);
		void CreateParamList();
		void DetectObjects(
			char *MeshFilePathName,
			Array2D<short int> *pDepthImage = NULL,
			IplImage *pRGBImage = NULL);
		void ObjectAggregationLevel2VN();
		void ObjectAggregationLevel2VN2();	//Filko
		void ObjectAggregationLevel2VN3();
		void ObjectAggregationLevel2VN4(int maxNeighbourhoodSize);	//Filko
		void Evaluate(
			FILE *fp,
			char *fileName,
			char *selectedGTObjectsFileName = NULL);
		void Evaluate3DReconstruction(
			ReconstructionEval *pReconstructionEval,
			int &TP,
			int &FP,
			int &FN,
			Array3D<uchar> *pGrid);
		void BoundingBox(
			int iObject1,
			int iObject2,
			RECOG::PSGM_::ModelInstance *pBoundingBox);
		static bool CheckIfWithinCTIBoundingBox(void * odObj, int iObject1, int iObject2, float dimThr = 0.30);	//Filko
		float CalculateIntersectionOverUnion(std::string GTfilename, Array<int> iSegmentArray);	//Filko
		void GroundTruthGroundPlane();
		void SaveBoundingBoxSizes(char *imageFileName);
		void TrainingHMI(char *meshFileName);
		void Display(Visualizer *pVisualizer);
		void DisplayObjectMap();
		void SaveObjectMapLabelImg(std::string filename);
		void DisplaySelectedObject(
			GRAPH::HierarchyNode *pObject,
			uchar *color,
			cv::Mat RGB);
		void DisplayClustersInteractive(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			uchar *selectionColor);
		GRAPH::HierarchyNode * GetObject(int iPix);
		
	public:
		DWORD flags;
		DWORD objectAggregationLevel1Method;
		CRVLParameterList ParamList;
		CRVLMem *pMem0;
		CRVLMem *pMem;
		Array2D<int> objectMap;
		char *SVMClassifierParamsFileName;
		Camera camera;
		float convexityThr;
		float convexityRatioThr1;
		float convexityRatioThr2;
		float connectedComponentMaxDist;
		int nMultilateralFilterIterations;
		int joinSmallObjectsToLargestNeighborSizeThr;
		float joinSmallObjectsToLargestNeighborDistThr;
		float appearanceCost;
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
		bool bOwnConvexConcaveClustering;
		bool bOwnsVNClassifier;
		bool bTrainingHMI;
		bool bDisplay;
		bool bHypothesisLog;
		SurfelGraph *pSurfels;
		PlanarSurfelDetector *pSurfelDetector;
		SURFEL::ObjectGraph *pObjects;
		PSGM *pPSGM;
		PSGM *pConvexClustering;
		PSGM *pConcaveClustering;
		Mesh mesh;
		Array<RECOG::SceneCluster> SClusters;
		int *clusterMap;
		char *cfgFileName;
		char *resultsFolder;
		char *meshFileName;
		void *vpMeshBuilder;
		bool (*LoadMesh)(
			void *vpMeshBuilder,
			char *FileName,
			Mesh *pMesh,
			bool bSavePLY,
			char *PLYFileName,
			char *depthFileName);
		void(*CreateMesh)(
			void *vpMeshBuilder,
			Array2D<short int> *pDepthImage,
			IplImage *pRGBImage,
			Mesh *pMesh,
			bool bSavePLY,
			char *FileName);
		RECOG::CTISet CTIs;
		RECOG::CTISet boundingBoxes;
		void *vpVNClassifier;
		std::vector<void *>hypotheses;
		OBJECT_DETECTION::VisualizationData visualizationData;
		int debug1, debug2;
	};
}

