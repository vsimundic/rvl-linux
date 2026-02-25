#pragma once

#ifndef RVLLINUX
#define RVLVN_TIME_MESUREMENT // Vidovic
#endif

#define RVLVN_GET_CORRESPONDENCE(correspMatrix, modelVNArray, iModel1, iModel2, i)	correspMatrix[iModel1].Element[modelVNArray.Element[iModel1]->nComponents * iModel2 + i]

#define RVLVN_MODELDB_NONE		0
#define RVLVN_MODELDB_3DNET		1
#define RVLVN_MODELDB_OSD		2
#define RVLVN_MODELDB_SHAPENET	3

#define RVLVN_SUPERSEGMENT_FLAG_REJECTED		0x01

namespace RVL
{
	class VNClassifier;

	namespace RECOG
	{
		namespace VN_
		{
			struct VisualizationData
			{
				float resolution;
				float SDFSurfaceValue;
				float meshSize;
				bool bVisualizeConvexClusters;
				bool bVisualizeConcaveClusters;
				bool bVisualizeSurfels;
				bool bVisualizeAllClusters;
				bool bVisualizeMergedClusters;
				bool bVisualizeCTIs;
				bool bVisualizeSceneFitting;
				bool bVisualizeZAxes;
				bool bVisualizeSceneInterpretation;
				bool bVisualizeVNInstance;
				VNClassifier *pClassifier;
				Mesh *pMesh;
				SurfelGraph *pSurfels;
				Visualizer *pVisualizer;
				bool bClusters;
				unsigned char selectionColor[3];
				int iSelectedCluster;
				unsigned char *clusterColor;
				int iSelectedSuperSegment;
				bool bHypothesisSelected;
				Array<OrientedPoint> *ZAxes;
				vtkSmartPointer<vtkActor> ZAxesActor;
				Array<vtkSmartPointer<vtkActor>> topHypothesisActors;
				vtkSmartPointer<vtkActor> selectedHypothesisActor[3];
				VNInstance *pVNSceneInstance; //Vidovic
				VN_::PartAssociation *componentAssociation; //Vidovic
				VN_::PartAssociation *cellAssociation; //Vidovic
				int iSelectedComponent;
			};

			struct HypothesisSortPtr
			{
				Hypothesis2 *ptr;
				float cost;
			};

			struct GTHypothesis
			{
				Hypothesis2 *pHypothesis;
				float BBoxCost;
				float CHCost;
				float RMSE;
				float R[9];
				float t[3];
				int rank;
				Hypothesis2 *pBestEBBTopHypothesis;
				float BBoxCost_;
			};

			struct SurfelMask
			{
				bool *mask;
				bool *mem;
				int nSurfelIDs;
			};

			void _3DNetDatabaseClasses(VNClassifier *pClassifier);
			void OSDDatabaseClasses(VNClassifier *pClassifier);
			void ShapeNetDatabaseClasses(VNClassifier *pClassifier);
			bool keyPressUserFunction(
				Mesh *pMesh,
				SurfelGraph *pSurfels,
				std::string &key,
				void *vpData);
			bool mouseRButtonDownUserFunction(
				Mesh *pMesh,
				SurfelGraph *pSurfels,
				int iSelectedPt,
				int iSelectedSurfel,
				void *vpData);
			float ComparePosesUsingConvexHull(
				float *R,
				float *t,
				Mesh *pCH,
				float *PMem = NULL);
		}	// namespace VN_
	}	// namespace RECOG

	class VNClassifier
	{
	public:
		VNClassifier();
		virtual ~VNClassifier();
		void Create(char *cfgFileName);
		void Init(Mesh *pMesh);
		void Clear();
		void CreateParamList();
		void FreeTmpMem();
		void ComputeDescriptor(
			Mesh *pMesh,
			float *RIn,
			float *tIn,
			float *&dS,
			bool *&bdS,
			Box<float> &SBoundingBox,
			int iModel = -1,
			CRVLMem *pMem0 = NULL,
			VNInstance **ppVNModel = NULL);
		void ComputeDescriptor(
			Array<int> iVertexArray,
			Array<SURFEL::NormalHullElement> NHull,
			float *R,
			float *d,
			uchar *bd,
			float *NS,
			int *iV,
			float o = 1.0f,
			bool bNormalTest = true,
			bool bVisibilityTest = true,
			float *J = NULL);
		void ComputeDescriptor(
			SURFEL::Object *pSegment,
			Array<SURFEL::NormalHullElement> NHull,
			float *R,
			float *d,
			uchar *bd,
			float *NS,
			int *iV,
			bool bNormalTest = true,
			bool bVisibilityTest = true,
			float *J = NULL);
		void ComputeDescriptor(
			Mesh *pConvexHull,
			float *R,
			float *d,
			uchar *bd,
			Array<int> *piVertexArray = NULL,
			float o = 1.0f,
			int *iV = NULL);
		void ComputeDescriptors(
			Array<int> iSurfelArray,
			Array<int> iVertexArray,
			float *d,
			uchar *bd,
			float *dU = NULL,
			uchar *bdU = NULL);
		void ComputeDescriptors(
			Array<int> iSurfelArray,
			Array<int> iVertexArray,
			float **pd,
			uchar **pbd);
		void ComputeDescriptors(
			float *dSrc,
			float *dTgt,
			uchar *bd);
		void ComputeDescriptors(
			Mesh *pConvexHull,
			float *R,
			float *d,
			uchar *bd,
			Point **pVertex);
		bool ComputeDescriptors(
			Array<int> iVertexArray,
			float *R,
			float *d,
			uchar *bd,
			bool bConvexHullVertexMemAllocated = false);
		void CTIs(
			Mesh *pMesh,
			int iModel);
		void MergeDescriptors(
			float *dSrc1,
			uchar *bdSrc1,
			float *dSrc2,
			uchar *bdSrc2,
			float o,
			float *dTgt,
			uchar *bdTgt);
		void MergeDescriptors(
			float *dSrc1,
			float *dSrc2,
			float o,
			float *dTgt);
		void Learn(
			char *modelSequenceFileName,
			int iClass = -1,
			int iMM = -1,
			Visualizer *pVisualizer = NULL); //Vidovic
		void LearnPrimitives(char *primitiveFileName);
		void Interpret(Mesh *pMesh);
		void Interpret1(Mesh *pMesh);
		void Interpret2(Mesh *pMesh);
		void Interpret3(Mesh *pMesh);
		void Interpretation();
		void GreedyInterpretation();
		bool Classify(
			Mesh *pMesh,
			SURFEL::ObjectGraph *pObjects,
			Array<int> *piConvexObjects,
			Array<int> iSegmentArray,
			RECOG::VN_::FitData *fitData,
			QList<RECOG::VN_::Hypothesis> *pHypothesisList,
			float *d,
			uchar *bd,
			float *f,
			float *R,
			float *J,
			CRVLMem *pMem);
		bool Classify2(
			Mesh *pMesh,
			SURFEL::ObjectGraph *pObjects,
			Array<int> *piConvexObjects,
			Array<int> iSegmentArray,
			RECOG::VN_::FitData *fitData,
			QList<RECOG::VN_::Hypothesis> *pHypothesisList,
			float *d,
			uchar *bd,
			float *f,
			float *R,
			CRVLMem *pMem);
		float MatchLatentVectors(
			float *q1,
			float s1,
			float *q2,
			float s2,
			int m,
			float &et,
			float &es,
			float &ea,
			float *e,
			int scale = 0);
		RECOG::VN_::SuperSegment * CreateSupersegment(
			Array<int> segments,
			float o);
		int CreateHypotheses(
			RECOG::VN_::SuperSegment *pSuperSegment,
			float o,
			QList<RECOG::VN_::Hypothesis> *pHypothesisList);
		float InterpretationCost(bool *X);
		void InitInterpretationCostComputation();
		void DeallocateInterpretationCostComputationMem();
		void SegmentToParts(Mesh *pMesh,
			int ID = -1);
		void DetectPrimitives(
			Mesh *pMesh,
			RECOG::VN_::FitData *fitData,
			QList<RECOG::VN_::Hypothesis> *pHypothesisList,
			CRVLMem *pMem);
		void ImageSegmentation(
			Mesh *pMesh,
			Array2D<int> &objectMap);
		void SuperSegmentNeighbourhood(Mesh *pMesh);
		void SampleSO3();
		float ProjectToLatentSubspace(
			float *d,
			uchar *bd,
			float *dU,
			uchar *bdU,
			RECOG::ClassData *pClass,
			float *q,
			int &iR);
		float ProjectToLatentSubspace(
			float *d,
			uchar *bd,
			RECOG::ClassData *pClass,
			RECOG::VN_::FitData *pFitData,
			float *q,
			int &iR,
			int &nb,
			bool &bAllDoFsVisible);
		void ProjectToLatentSubspace(
			float *d,
			uchar *bd,
			RECOG::ClassData *pClass,
			RECOG::VN_::FitData *pFitData,
			std::vector<std::tuple<float, float*, int, int, bool>> *allbuffer);	//Filko (tuple<E, q, iR, nb, bAllDoFsVisible>)
		float FitLS(
			float *d1,
			uchar *bd1,
			float *d2,
			uchar *bd2,
			float *t,
			int &nV,
			float *e = NULL);
		float FitLS(
			float *A,
			float *d1,
			float *d2,
			uchar *bd,
			float *t,
			float *e = NULL);
		int InitFitLS(
			float *d,
			uchar *bd,
			float *A);
		bool FitLS(
			float *d,
			uchar *bd,
			RECOG::ClassData *pClass,
			RECOG::VN_::FitData *pData,
			float &E);
		bool FitRotLS(	
			Array<int> iVertexArray,
			float o,
			float *NM,
			RECOG::ClassData *pClass,
			float *RIn,
			int nIterations,
			bool bNormalTest,
			bool bVisibilityTest,
			RECOG::VN_::FitData *pData,
			float *qOpt,
			float *ROpt,
			float &E);
		float FitCTI(
			Mesh *pMesh,
			Array<int> iSurfelArray, 
			Array<int> iVertexArray, 
			Camera camera,
			int nIterations,
			float *R,
			float *d,
			uchar *bd);
		void InitNormalAlignment(
			Array<MESH::Face *> faces,
			Array<int> iFaceArray,
			float *U);
		float FitCTI(
			Mesh *pConvexHull,
			float *U,
			float *RInit,
			bool *mask,
			int nIterations,
			float *R,
			int *faceCorrespondence,
			Array<int> *piModelNormalArray = NULL);
		float Align(
			Array<MESH::Face *> faceArray,
			Array<MESH::Face *> refFaceArray,
			float *R);
		void CanonicalOrientation(float *RCanonical);
		void ComputeRotationCostJacobian(
			Mesh *pConvexHull,
			float *RCanonical,
			float *d,
			unsigned char *bd,
			float *J,
			Array<int> *piVertexArray = NULL);
		void Primitives(
			Mesh *pMesh, 
			VNInstance *pVNInstance = NULL);
		void SetSceneFileName(char *sceneFileName_);
		void SaveDescriptor(
			FILE *fp,
			float *d,
			bool *bd,
			int iModel,
			int iMetaModel);
		void SaveDescriptor(
			FILE *fp,
			float *d,
			uchar *bd,
			float *R,
			int iModel,
			int iPart,
			int iCluster = 0,
			int type = 0);
		void SaveDescriptors(
			FILE *fp,
			float *d,
			uchar *bd,
			int iCluster,
			int type);
		void SaveDescriptors(
			FILE *fp,
			float *d,
			uchar *bd,
			int nDescriptors,
			int iCluster,
			int type);
		void LoadDescriptor(
			FILE *fp,
			float *&d,
			bool *&bd,
			int &iModel,
			int &iMetaModel);
		void SaveLatentVector(
			FILE *fp,
			float *q,
			int iClass);
		void LoadLatentVector(
			FILE *fp,
			float *&q,
			int &iClass);
		void LoadShapeSpace(char *shapeSpaceFileName);
		void InitDisplay(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			unsigned char *selectionColor);
		void DisplayClusters();
		void ResetClusterColor(int iCluster);
		void DisplaySelectedCluster(int iCluster);
		void DisplaySelectedSuperSegment(int iSuperSegment);
		void DisplayInterpretation(
			Mesh *pMesh,
			uchar *ptColor,
			cv::Mat image,
			int pointSize = 1,
			bool bBGR = false);
		void ComputeClassPropertiesForGrasping(int iClass, VN *pModel);
		//void SaveVertices(FILE *fp, RECOG::PSGM_::Cluster *pSCluster);
		float ComputeDescriptor(
			Mesh *pMesh,
			float *RIn,
			float *tIn,
			float *&dS,
			bool *&bdS,
			Box<float> &SBoundingBox,
			int iModel,
			float score);
		void SaveVNInstances(
			QList<VNInstance> *pModelVNList,
			void *vpClassifier);
		void LoadVNInstances(void *vpClassifier, char *VNInstanceListFileName, QList<VNInstance> *pVNList, Array<VNInstance *> *pVNArray);
		void LoadPCs();
		//bool ReferenceFrames(Mesh convexHull);
		void SaveVertices(FILE *fp, RECOG::PSGM_::Cluster *pSCluster, int iCluster, int clusterType);
		void MergeClusters(
			Mesh *pMesh,
			Array<RECOG::SceneCluster> *pSClusters,
			bool bMergeClusters = true,
			bool verbose = false); //Vidovic
		void CreateVNInstances(Mesh *pMesh, VNInstance **ppVNModel);
		void CreatePartModel();
		void CreateComponentAssociationGraph();
		void ComponentAssociation(float *fLabel);
		void ComponentAssociationMST();
		void CreateConvexHullsAndBoundingSpheres();
		void CreateSuperSegments(int minClusterSize = 0);
		bool HypothesisEvaluationCH(
			RECOG::VN_::Hypothesis2 *pHypothesis,
			float &eavg,
			float &emax);
		void AssignSegmentsToHypothesis(
			RECOG::VN_::Hypothesis2 *pHypothesis,
			float tolerance = 0.0f);
		void DetectGroundPlane(Mesh *pMesh);
		void CreateSurfelMask(
			Mesh *pMesh,
			RECOG::VN_::SurfelMask &surfelMask);
		void CreateForegroundObjectSurfelMask(RECOG::VN_::SurfelMask surfelMask);
		void CreateSurfelMask(
			Array<int> segments, 
			RECOG::VN_::SurfelMask surfelMask,
			bool b = true);
		void DisplaySurfelMask(
			RECOG::VN_::SurfelMask surfelMask,
			Rect<int> *pBBox = NULL);
		void GreedyInterpretation2();
		//void UpdateSegmentsColorHistograms(); //Vidovic
		float CHMatching(
			Array<int> iSSurfelArray,
			int iModel,
			int metric,
			FILE *fp = NULL); //Vidovic
		void DetectSupportingSurfaces(Mesh* pMesh);

	public:
		CRVLMem *pMem0;
		CRVLMem *pMem;
		CRVLParameterList paramList;
		DWORD mode;
		DWORD problem;
		DWORD dataSet;
		int trainingMethod;
		int recognitionMethod;
		int hypothesisEvaluationLevel3Method;
		DWORD modelDB;
		SurfelGraph *pSurfels;
		SURFEL::ObjectGraph *pObjects;
		PlanarSurfelDetector *pSurfelDetector;
		Array2D<float> sampledUnitSphere;
		PSGM alignment;
		PSGM convexClustering;
		PSGM concaveClustering;
		PSGM planeDetector;
		PSGM *pShapeInstanceDetection;
		NeighborhoodTool *pNeighborhoodTool;
		Camera camera;
		int nCT;
		std::vector<VN *> models;
		std::vector<VN *> metaModels;
		Mesh **modelCH;
		Sphere<float> *modelBoundingSphere;
		Array<int> unassignedSurfels;
		float wePosition;
		float weSize;
		float kMaxMatchCost;
		float tangentOrientationTolerance;
		float kTangentDistanceTolerance;
		float clusteringTolerance;
		int maxnSCClusters;
		int maxnSUClusters;
		int maxnSTClusters;
		int nHypothesesLevel3;
		float voxelSize;
		int sampleVoxelDistance;
		float connectedComponentMaxDist;
		int connectedComponentMinSize;
		float maxDistFromNHull;
		float maxedmax;
		float maxBoundingSphereRadius;
		float kZMin2;
		float alignmentSupportThr;
		int maxOutlierPerc;
		float maxz;
		int collisionPercThr;
		float convexityErrorCoeff;
		float invisibleComponentPenal;
		float unassignedPointCost;
		float multiplePointAssignmentCost;
		float outlierCost;
		float sigmaR;
		float sigmaRdeg;
		float lambda;
		float stdTranslation; //Vidovic
		float stdSize;
		float stdShape;
		float kComponentUncert;
		float priorProbabilityComponent;
		float beta; //Vidovic
		float gama; //Vidovic
		float dUnassignedPointsLogProbability;
		float dUnassignedPoints;
		float CTIMatchSigmad1;
		float CTIMatchSigmad2;
		float CTIMatchRedundantPoseOrientThr;
		float ICPSuperSegmentRadius;
		float hypothesisEvaluationLevel3CHTolerance;
		int simulatedAnnealingnIterationsPerHypothesis;
		int simulatedAnnealingnSamples;
		float kFaceClusterSizeThr;
		float level3MatchedPercentageThr;
		float greedyInterpretationMatchedPercentageThr;
		float greedyInterpretationColorThr;
		float edThr;
		float subsampleModelsVoxelSize;
		//int minSuperSegmentSegmentSize;
		int nIterationsComponentAssociationLocalSearch;
		int nSamplesComponentAssociationLocalSearch;
		float metamodelChangeProbability;
		float labelChangeProbability;
		float componentClusterMaxCost;
		float componentAssociationAlphat;
		float componentAssociationAlphas;
		float componentAssociationBeta;
		float componentAssociationGamma;
		float componentAssociationSig;
		float alphaCTINetPrior;
		float componentNeighborhoodMatchUnmatchPenal;
		float wModelSimilarity;
		float wNeighborhoodSimilarity;
		float componentAssociationfCompThr;
		RECOG::VN_::FitParams fitParams;
		bool b3DNetTestSet;
		bool b3DNetVOI;
		bool bBoundingBoxes;
		bool bDetectSupportingSurfaces;
		bool bGndConstraint;
		bool bGenerateModelCTIs;
		bool bCreateAlignmentTree;
		bool bPrimitiveLayer;
		bool bLoadCTIDataBase;
		bool bConcavity;
		bool bVisualization;
		bool bVisualizeBestInstance;
		bool bSingleClass;
		bool bCTIAlignment;
		bool bDepthImage;
		bool bOwnsObjectGraph;
		bool bAlignWithConvexTemplate;
		bool bHorizontalBottomAssumption;
		bool bShapeSpace;
		bool bPlusPlus;
		bool bInstanceMatchWithLabels;
		bool bLearnMetamodels;
		bool bGenerateModelVNInstances;
		bool bGenerateSceneVNInstances;
		bool bSampleModelPointClouds;
		bool bMST;
		bool bCAG;
		bool bLoadModelVNInstances;
		bool bLoadSceneVNInstances;
		bool bSaveGeneratedVNInstances;
		bool bSaveSupersegments;
		bool bUseColor;
		bool bConcaveSuperSegments;
		bool bUseLevel3MatchedPercentageThr;
		bool bVisualizePartAssociation;
		bool bLabelsAssignedToUnassignedPoints;
		bool bPartSegmentationMetamodelClusters;
		bool bVNCTIAlignment;
		char *modelDataBase; //Vidovic
		char *modelsInDataBase; //Vidovic
		char *modelSequenceFileName;
		char *VNShapeSpaceFileName;
		char *ModelVNInstanceListFileName;
		char *componentFileName;
		char *SceneVNInstanceListFileName;
		char *modelLayerFileName;
		char *ScenePointCellID;
		char *PartSuperSegmentID;
		char *scenesInDataBase;
		char *primitiveDataBase;
		char *sceneFileName; //Vidovic
		char *validFeaturesFileName;
		char *representativesFileName;
		char *resultsFolder;
		void *vpMeshBuilder;
		bool(*LoadMesh)(void *vpMeshBuilder,
			char *FileName,
			Mesh *pMesh,
			bool bSavePLY,
			char *PLYFileName,
			char *depthFileName);
		void(*ICPFunction)(vtkSmartPointer<vtkPolyData>, 
			vtkSmartPointer<vtkPolyData>, 
			float*, 
			int, 
			float, 
			int, 
			double*, 
			void*);
		int ICPVariant;
		RECOG::VN_::VisualizationData visualizationData;
		RECOG::VN_::SceneObject sceneObject;
		float NGnd[3];
		float dGnd;
		bool bGnd;
		RECOG::VN_::Instance refModel;
		Array<RECOG::ClassData> classArray;
		int *hypClass;
		int *clusterMap;
		Array<RECOG::SceneCluster> SClusters;
		Array3D<float> SO3Samples;
		CRVLMem descriptorMem;
		bool bTrainingStructure;
		FILE *fpDebug;
		float timeInterpret;
		ConvexTemplateLookUpTable convexTemplateLUT;
		RECOG::PSGM_::CTIDescriptorMapping<float> CTIDescriptorMapingData;
		Eigen::MatrixXf convexTemplate;
		int debug1, debug2;
		Mesh convexHull;
		Array<RECOG::VN_::Hypothesis *> hypotheses;
		QList<RECOG::VN_::Hypothesis2> hypothesisList;
		Array<RECOG::VN_::SuperSegment *> superSegments;
		Array<int> interpretation;
		Array<SortIndex<float>> sortedHypotheses;
		Array<RECOG::VN_::ModelTemplate> modelTemplates;
		Array<RECOG::VN_::ComponentCluster> componentClusters;
		Array<RECOG::VN_::HypothesisSortPtr> *sortedSuperSegmentHypotheses;
		Array<RECOG::VN_::HypothesisSortPtr> *sortedModelHypotheses;
		RECOG::VN_::FitData *fitData;
		float *D;
		uchar *bD;
		float *f;
		float *R;
		RECOG::VN_::PrimitiveLayerOutput primitiveLayerOutput;
		QList<VNInstance> modelVNList;
		QList<VNInstance> sceneVNList;
		int sceneID;
		Array<VNInstance*> modelVNArray;
		Array<int> modelSet;
		Array<int> firstComponentAbsIdx;
		int nMCompsTotalOU[2];
		Graph<GRAPH::Node_<GRAPH::EdgePtr<RECOG::VN_::CAGEdge>>, RECOG::VN_::CAGEdge, GRAPH::EdgePtr<RECOG::VN_::CAGEdge>> componentAssociationGraph;
		RVL::Array<VNInstance*> sceneVNArray;
		std::vector<VNClass> classes;
		Pair<int, int> *modelSegmentInterval;
		Array2D<float> esComponent;
		Array2D<float> etComponent;
		Array<int> *activeGauss;
		int *activeGaussMem;
		int *metamodelGaussAssociation;
		float *metamodelGaussAssociationCost;
		Array<int> *activeMetamodelGauss;
		int *activeMetamodelGaussMem;
		RECOG::VN_::ProbabilisticAssociationWorkData probabilisticAssociationWorkData;
		RECOG::VN_::AssociationProbabilityWorkData associationProbabilityWorkData;
		int maxnModelSegments;
		float *DM;
		uchar *bDM;
		std::map<int, vtkSmartPointer<vtkPolyData>> segmentN_PD; //neighbourhood
		std::vector<RECOG::VN_::Hypothesis2 *> interpretation2;
		int nTP;
		int nFP;
		int nFN;
		int maxGTPartLabel;


		//struct convexHullFacesPairs
		//{
		//	Pair < int, int > *faces;
		//	float cost;

		//};

		//Array<RECOG::VN_::Hypothesis *> interpretation;  //Vidovic commented on 7.2.2019 - redefinition of variable RVL::VNClassifier::interpretation
		int iConvexCluster; //Vidovic - for printing vertices to file
		int iConcaveCluster; //Vidovic - for printing vertices to file
		bool bMergeClusters; //Vidovic
		float proximityThresh; //Vidovic
		float CHDistanceThresh; //Vidovic
		float mergeThresh1; //Vidovic
		float mergeThresh2; //Vidovic
		float trainingModelScale;
		bool bSaveClusterDistanceFile; //Vidovic
		bool bDistanceToCHUsingNormals; //Vidovic
		int iSelectedClass;
		int iRefModel;
		int iRefComponent;
		CRVLTimer *pTimer;
		double surfelTime;
		double planarAndConvexSurfacesTime;
		double hypGenAndLEVEL1Time;
		double LEVEL2Time;
		double LEVEL3Time;
		double greedyInterpretationTime;
		double totalTime;
		double totalTime_;
		double surfelTime2;
		double planarAndConvexSurfacesTime2;
		double hypGenAndLEVEL1Time2;
		double LEVEL2Time2;
		double LEVEL3Time2;
		double greedyInterpretationTime2;
		double totalTime2;
		double totalTime2_;
	private:
		Array<int> iSurfelArray;
		Array<int> iVertexArray;
		//Array<int> iVertexArray2;
		bool *bSurfelInArray;
		bool *bVertexInArray;
		//bool *bVertexInArray2;
		Array<int> iConcavitySurfelArray;
		Array<int> iConcavityVertexArray;
		Array2D<float> convexHullVertices;
		Array2D<float> concavityVertices;
		CRVLMem memConvexHull;
		Mesh CTIMesh;
		CRVLMem memCTIMesh;
		int CTIMeshVertexMemSize;
		QList<VN> VNList;
		int *nSurfelCellHypotheses;
		Array<int> iObjectVertexArray;
		int maxnModelCHFaces;
		float *NBuff;
		float varTranslation;
		float varSize;
		float varShape;
	};
}