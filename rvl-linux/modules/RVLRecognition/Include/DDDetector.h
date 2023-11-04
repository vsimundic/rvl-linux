#pragma once

#define RVLDDD_TEST_DDD 0
#define RVLDDD_TEST_CUBOIDS 1
#define RVLDDD_TEST_3DTO2DFIT 2
#define RVLDDD_TEST_DDD2 3
#define RVLDDD_TEST_SVD 4
#define RVLDDD_TEST_DETECT_RECTSTRUCT 5
#define RVLDDD_TEST_RECOGNIZE_RECTSTRUCT 6
#define RVLDDD_TEST_RECOGNIZE_AO 7
#define RVLDDD_EDGE_MODEL_BOX 0
#define RVLDDD_EDGE_MODEL_CYLINDER 1
#define RVLDDD_MODEL_DOOR 0
#define RVLDDD_MODEL_DRAWER 1

// This macro should be moved to RVL3DTools.h
#define RVLCOPY3DPOSE(srcR, srct, tgtR, tgtt) \
	{                                         \
		RVLCOPYMX3X3(srcR, tgtR)              \
		RVLCOPY3VECTOR(srct, tgtt)            \
	}

namespace RVL
{
	struct AffinePose3D
	{
		float s[3];
		float R[9];
		float t[3];
	};

	struct EpipolarGeometryDisplayData
	{
		cv::Mat display;
		cv::Mat intiDisplay;
		int imgSize[2][2];
		float F21[9];
		int counter;
	};

	struct CannyDisplayData
	{
		cv::Mat src;
		cv::Mat dst;
		int lowThreshold;
		int maxLowThreshold;
		int highTreshold;
		int maxHighThreshold;
		std::string windowName;
	};

	void VisualizeEpipolarGeometry(
		cv::Mat img1,
		cv::Mat img2,
		cv::Mat cvP1,
		cv::Mat cvP2,
		Pose3D pose21);
	void EpipolarGeometryCallback(int event, int x, int y, int flags, void *userdata);
	bool LineConvexSetIntersection2D(
		float *line,
		Array<Vector3<float>> convexSet,
		float *intersectionLineSegment);

	void CannyThresholdCallback(int, void *);

	namespace RECOG
	{
		namespace DDD
		{
			// struct PlanarSurface
			//{
			//	float P[3];
			//	float N[3];
			//	QList<QLIST::Index> surfels;
			//	Moments<double> moments;
			// };

			struct ModelPoint
			{
				float S[3 * 7];
				float N[3];
			};

			struct ModelEdge
			{
				int iFace;
				int iFace_;
				float N[3];
			};

			struct Transformation
			{
				double R[9];
				double t[3];
			};

			class Model
			{
			public:
				Model();
				virtual ~Model();

			public:
				Array<OrientedPoint> points;
				float bboxSize[3];
				float bboxCenter[3];
				Array<ModelPoint> points_;
				int nSurfaces;
				float *d;
				float *A;
				int *AID;
				float *Q;
				float *R;
				float *M;
				Array<ModelEdge> edges;
				int *info;
			};

			struct Hypothesis
			{
				AffinePose3D pose;
				float bboxSize[3];
				float bboxCenter[3];
				Array<Pair<int, int>> assoc;
				int imgID;
			};

			struct HypothesisSV
			{
				float s[6];
				float RMS[9];
			};

			struct AOHypothesisState
			{
				int imgID;
				float q;
				float score;
			};

			struct AOHypothesisStateTrack
			{
				// char direction;
				// float uncertainty;
				int iHypothesis;
				std::vector<AOHypothesisState> states;
				std::vector<AOHypothesisState> nextStates;
			};

			struct HypothesisDoorDrawer
			{
				uchar objClass;
				Pose3D pose;
				float s[2];
				float r[2];
				float openingDirection;
				Array<AOHypothesisState> state;
				float score;
			};

			struct DisplayCallbackData
			{
				Visualizer *pVisualizer;
				bool bOwnVisualizer;
				CRVLParameterList paramList;
				Mesh *pMesh;
				Array<Point> AssociatedPts;
				// Array<Pair<int, int>> associations;
				// vtkSmartPointer<vtkActor> selectedPtActor[6];
				vtkSmartPointer<vtkActor> bboxActor;
				vtkSmartPointer<vtkActor> modelPtActor;
				vtkSmartPointer<vtkActor> associationLinesActor;
				// vtkSmartPointer<vtkActor2D> selectedLabelActor[4];
				// int selectionMode;
				// int matchStep;
				// Array<Point>* pPoints;
				// uchar* pointColor;
				bool bVisualizeSurfels;
				bool bVisualizeICPSteps;
				bool bVisualizePtAssociation;
				bool bVisualizeInitialHypothesis;
				bool bVisualizeBestHypothesis;
				bool bVisualizeModelPts;
				bool bVisualizeFittingScoreCalculation;
				bool bVisualizeROIDetection;
				bool bVisualizeHoughTransform;
				bool bVisualizeRANSACIterations;
				bool bVisualizeSolution;
				bool bVisualizeDoorHypotheses;
				bool bVisualizeROI;
				bool bVisualizeRectangularStructure;
				bool bVisualizeRectStructMatching;
				bool bVisualizeAOInitState;
				bool b3DVisualization;
				bool bRGBImageVisualization;
				bool bPointToPlane;
				float bboxCenter[3];
				float bboxSize[3];
				cv::Mat *edges1;
				cv::Mat *edges2;
				cv::Mat *edgesVis;
				CRVLEDT EDT[2];
				RVLEDT_PIX_ARRAY EDTImage[2];
				cv::Mat EDTDisplayImage[2];
				cv::Mat ICPDisplayImage[2];
				cv::Mat *displayImg;
			};

			struct EdgeSample
			{
				float PR[3];
				float PC[3];
				float ImgP[2];
				int iImgP[2];
				int iPix;
				int edgeIdx;
			};

			struct Edge
			{
				int iVertex[2];
				float PR[2][3];
				float PC[2][3];
				float VR[3];
				float VC[3];
				cv::Point ImgP[2];
				float ImgN[2];
				float length;
				bool bVisible;
				float binEnd;
			};

			struct Plane
			{
				float N[3];
				int iN[3];
				float d;
			};

			struct PtAssoc
			{
				int iMSample;
				int iQPix;
				int iImg;
				float QImgN[2];
				float QN[3];
			};

			struct SurfelEdge
			{
				float P[2][3];
				float N[3];
				int iEdgeSurfel;
			};

			struct Rect3D
			{
				float c[3];
				float s[2];
				char iAxis;
				float direction;
				int iSurfel;
				int iParent;
			};

			struct RectStruct
			{
				float RRS[9];
				Array<Rect3D> rects;
			};

			struct SurfAssoc
			{
				int iMRect;
				int iQRect;
				float cost;
			};

			struct AxisHypothesis
			{
				int iHyp;
				float axis[3];
			};

			struct ArticulatedObject
			{
				Array<RECOG::DDD::HypothesisDoorDrawer> movingParts;
				RECOG::DDD::RectStruct MRS;
			};

            struct EdgeLineSegmentPixel
            {
                int iPix;
                float e;
                RECOG::DDD::EdgeLineSegmentPixel *pNext;
            };

            struct EdgeLineSegment
            {
                int iCluster;
                float P[2][2];
                QList<RECOG::DDD::EdgeLineSegmentPixel> pix;
                Array<int> pix_;
                float w;
                float N[2];
                float P0[2];
            };

            struct Line2D
            {
                float P[2][2];
                float N[2];
                float d;
            };

            struct Detect3CallBackFuncData
            {
            };

			bool ProjectToBase(
				float *a,
				int n,
				float *Q,
				int m,
				float *r,
				float *b,
				float &c);
			bool SortCompare(SortIndex<float> x1, SortIndex<float> x2);
			void MapMeshRGB(
				Mesh *pMesh,
				cv::Mat mapping,
				cv::Mat &tgtImg);
            void MapImageC1(
                cv::Mat srcImg,
                cv::Mat mapping,
                cv::Mat &tgtImg);
            void Detect3CallBackFunc(int event, int x, int y, int flags, void *userdata);
		}
	}

	class DDDetector
	{
	public:
		DDDetector();
		virtual ~DDDetector();
		void Create(char *cfgFileName);
		void Clear();
		void CreateParamList();
        void DetectRGBEdgeLineSegments(
            cv::Mat RGB,
            int cannyThrL,
            int cannyThrH,
            int minHoughLineSize,
            cv::Mat &edges,
            cv::Mat &sobelx,
            cv::Mat &sobely,
            Array<RECOG::DDD::EdgeLineSegment> &lineSegments,
            int *&lineSegmentPixIdxMem,
            int *&lineSegmentMap);
        void SegmentEdgeLines(
            cv::Mat &edges,
            double *IuMap,
            double *IvMap,
            Array<RECOG::DDD::Line2D> lines,
            int linePtTolIn,
            float mincsN,
            int maxLineGap,
            Array<RECOG::DDD::EdgeLineSegment> &lineSegments,
            RECOG::DDD::EdgeLineSegmentPixel *&edgeLineSegmentMem,
            int &nClusterElements);
        void SegmentEdgeLines2(
            cv::Mat &edges,
            double *IuMap,
            double *IvMap,
            Array<RECOG::DDD::Line2D> lines,
            int linePtTolIn,
            float mincsN,
            int maxLineGap,
            Array<RECOG::DDD::EdgeLineSegment> &lineSegments,
            RECOG::DDD::EdgeLineSegmentPixel *&edgeLineSegmentMem,
            int &nClusterElements);
		void CreateModels(
			Array<Mesh> models,
			std::vector<std::string> modelFileNames);
		void CreateCuboidModel(
			float *size,
			float sampleDensity,
			RECOG::DDD::Model *pModel);
		void CreateSurfNetModel(
			float *A,
			int *AID,
			int nSurfaces,
			float *M,
			std::vector<std::vector<std::vector<int>>> SN,
			float minSamplingDensity,
			float *q,
			RECOG::DDD::Model *pModel,
			bool bVisualize = false);
        void BoxNormals(float *A);
		void CreateCuboidModel2(
			float *size,
			float minSamplingDensity,
			RECOG::DDD::Model *pModel,
			bool bVisualize = false);
		void CreateStorageVolumeModel(
			RECOG::DDD::Model *pModel,
			bool bVisualize = false);
		void BoxNormals(float *A);
		void LoadModels(std::vector<std::string> modelFileNames);
		void Detect(
			Array<Mesh> meshSeq,
			RECOG::DDD::HypothesisDoorDrawer *pFinalHyp,
			char *hypFileName = NULL,
			std::vector<cv::Mat> *pRGBSeq = NULL);
        void Detect3(
            Array2D<uchar> mask,
            Array<RECOG::DDD::Line2D> *pOrthogonalViewLines,
            Array<Rect<int>> *pDDRects);
		bool GenerateHypotheses(
			Mesh *pMesh,
			std::vector<AffinePose3D> ROIs,
			Array<int> *dominantShiftPointsIdx,
			Vector3<float> *pDominantShift,
			std::vector<RECOG::DDD::Hypothesis> &hyps,
			float *U = NULL,
			Array<int> *pSOI = NULL,
			Pose3D *pRF = NULL);
		void GenerateDoorHypotheses(
			std::vector<RECOG::DDD::Hypothesis> *pHyps,
			std::vector<RECOG::DDD::HypothesisDoorDrawer> &doorHyps);
		void GenerateDrawerHypotheses(
			std::vector<RECOG::DDD::Hypothesis> *pHyps,
			std::vector<RECOG::DDD::HypothesisDoorDrawer> &drawerHyps);
		float EvaluateHypothesis(
			Mesh *pMesh,
			RECOG::DDD::Model *pModel,
			int m,
			float *q,
			float *RMS,
			float maxe,
			int &nTransparentPts,
			int *SMCorrespondence = NULL,
			RECOG::SceneFittingError *errorRecord = NULL,
			float *tMS = NULL);
		void DetectCuboids(Mesh *pMesh);
		void DetectStorageVolumes(Mesh *pMesh);
		void DDOrthogonalView(
			RECOG::DDD::RectStruct *pRectStruct,
            Array<RECOG::DDD::EdgeLineSegment> edgeLineSegments,
			float *verticalAxis,
			int nOrthogonalViews,
            Pose3D *&poseFCOut,
			std::vector<cv::Mat> &orthogonalViews,
            Array<Array2D<uchar>> &masks,
            Array<RECOG::DDD::Line2D> *&orthogonalViewLines,
			bool bVisualize = false,
			Mesh *pMesh = NULL);
		void SampleImage(Mesh *pMesh);
		void PointAssociation(
			Array<OrientedPoint> pointsM,
			AffinePose3D *pPose,
			Array<OrientedPoint> pointsQ,
			PointAssociationData &pointAssociationData,
			Array<int> ptBuff,
			bool bVisualize = false,
			Array<Point> *pPointsMQ = NULL);
		void PointToPlaneAssociation(
			RECOG::DDD::Model *pModel,
			float *q,
			Pose3D *pPose,
			Array<OrientedPoint> pointsQ,
			bool *bRejected,
			PointAssociationData &pointAssociationData);
		void VisualizePointToPlaneAssociations(
			RECOG::DDD::Model *pModel,
			float *q,
			Pose3D *pPose,
			Array<OrientedPoint> pointsQ,
			PointAssociationData &pointAssociationData,
			Array<Point> *pPointsMQ);
		void AICP(
			Array<OrientedPoint> pointsM,
			Array<OrientedPoint> pointsQ,
			AffinePose3D APoseInit,
			int nIterations,
			AffinePose3D &APose,
			PointAssociationData &pointAssociationData,
			Array<Point> *pPointsMQ);
		void RICP(
			RECOG::DDD::Model *pModel,
			Array<OrientedPoint> pointsQ,
			AffinePose3D APoseInit,
			int nIterations,
			AffinePose3D &APose,
			PointAssociationData &pointAssociationData,
			Array<Point> *pPointsMQ);
		void TICP(
			Array<OrientedPoint> pointsM,
			Array<OrientedPoint> pointsQ,
			AffinePose3D APoseInit,
			int nIterations,
			AffinePose3D &APose,
			PointAssociationData &pointAssociationData,
			Array<Point> *pPointsMQ);
		void FitRectangle(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSurfel,
			float *RBS,
			Box<float> bboxInit,
			float thr,
			Rect<float> *pRect);
		float RectangularStructure(
			int iSurfel1,
			int iSurfel2,
			bool bEdge,
			float beta,
			RECOG::DDD::RectStruct *pRectStruct,
			bool bComputeRSS = true,
			bool bOptimize = false,
			Mesh *pMesh = NULL);
		bool RectangularStructures(
			Mesh *pMesh,
			RECOG::DDD::RectStruct *pRectStruct);
		bool MatchRectangularStructures(
			RECOG::DDD::RectStruct *pMRectStruct,
			RECOG::DDD::RectStruct *pQRectStruct,
			Pose3D &poseMQ,
			Pose3D &poseMC,
			Pose3D *pPoseCmCqInit = NULL,
			float *verticalMAxis = NULL);
		float MatchRectangularStructures(
			Array<RECOG::DDD::Rect3D> MRects,
			Array<RECOG::DDD::Rect3D> QRects,
			Array<RECOG::DDD::SurfAssoc> surfAssocs,
			float *tMQ,
			float maxe);
		bool RecognizeArticulatedObject(
			Mesh *pMesh,
			RECOG::DDD::ArticulatedObject AObj,
			Pose3D &poseOC,
			Pose3D *pPoseOCInit = NULL,
			float *verticalMAxis = NULL,
			cv::Mat *pRGBImg = NULL);
		bool OrientedBoundaryPoints(
			Mesh *pMesh,
			int iSurfel,
			Array<OrientedPoint> &boundary);
		float MatchToRectangle(
			Array<OrientedPoint> boundary,
			float *RRS,
			Rect<float> *pRect);
		void DetectDominantShiftPoints2(
			Mesh *pMeshM,
			Mesh *pMeshQ,
			Array<int> *dominantShiftPointsIdx, Vector3<float> &dominantShift,
			bool bVisualize);
		void CalculatePointsShiftVector2(
			Array<Point> pointsQ,
			Array<int> subsampledPointsQIdx,
			Array<OrientedPoint> pointsM,
			PointAssociationData *pPointAssociationData,
			Array<RVL::Vector3<float>> *pointsShift,
			float shiftThresh,
			Array<Pair<int, int>> &pointQSurfaceMAssotiations);
		void SurfaceShift(
			Array<Pair<int, int>> pointQSurfaceMAssotiations,
			Array<RVL::Vector3<float>> shifts,
			Vector3<float> pointsShift,
			float *shiftThresh,
			Array<Pair<int, int>> &pointQSurfaceMAssotiationsInShiftDirection);
		void SaveRectangularStructure(
			std::string fileName,
			RECOG::DDD::RectStruct *pRectStruct);
		void LoadRectangularStructure(
			std::string fileName,
			RECOG::DDD::RectStruct *pRectStruct);
		void DDBox(
			RECOG::DDD::HypothesisDoorDrawer *pMovingPartHyp,
			int iState,
			AffinePose3D *pBox);
		void ClearModel(RECOG::DDD::Model *pModel);
		void SaveDD(
			FILE *fp,
			std::vector<RECOG::DDD::HypothesisDoorDrawer> &movingParts);
		int LoadDD(
			FILE *fp,
			RECOG::DDD::HypothesisDoorDrawer *pMovingPart);
		void LoadDD(
			FILE *fp,
			std::vector<RECOG::DDD::HypothesisDoorDrawer> &movingParts);
		void SaveDDStates(
			FILE *fp,
			std::vector<RECOG::DDD::HypothesisDoorDrawer> &DDs);
		void LoadDDStates(
			FILE *fp,
			std::vector<RECOG::DDD::HypothesisDoorDrawer> &DDs);
		void LoadArticulatedObject(
			char *modelFilePath,
			char *AOFileName,
			RECOG::DDD::ArticulatedObject &AObj);
		void SurfelEdges(
			Mesh *pMesh,
			float *surfelRefPt,
			int minSurfelSize,
			int minEdgeSize);
		void PlanarSurfaceEdges(Mesh *pMesh);
        void AccuratePlaneFitting(Mesh *pMesh);
		void InitVisualizer(Visualizer *pVisualizer = NULL);
		void RunVisualizer();
		void ClearVisualization();
		Visualizer *GetVisualizer();
		void VisualizeHypothesisBoundingBox(
			RECOG::DDD::Hypothesis *pHyp,
			Mesh *pMesh = NULL);
		void VisualizeHypothesis(
			AffinePose3D pose,
			PointAssociationData *pPointAssociationData,
			Array<Point> pointsMS,
			Array<OrientedPoint> *pPointsQ = NULL);
        void Display(
            cv::Mat BGR,
            Pose3D *poseFC,
            int nOrthogonalViews,
            Array<Rect<int>> *DDRects);
		void SetSceneForHypothesisVisualization(Mesh *pMesh);
		void SetBBoxForHypothesisVisualization(RECOG::DDD::Hypothesis *pHyp);
		void RemoveHypothesisFromVisualization();
		void RemoveHypothesisBoundingBoxFromVisualization();
		void DetectDominantShiftPoints(Array<Mesh> meshSeq, Array<int> *selectedPointsIdx, bool bVisualize = false);
		void DetectDominantShiftPoints(Mesh *pMeshM, Mesh *pMeshQ, Array<int> *selectedPointsIdx, Vector3<float> &dominantShift, bool bVisualize = false);
		void MeshSubsample(Mesh *pMesh, Array<int> *pSubsampledPointsIdx);
		void CalculatePointsShiftVector(
			Array<Point> pointsM,
			Array<int> subsampledPointsMIdx,
			Array<OrientedPoint> pointsQ,
			PointAssociationData *pPointAssociationData,
			Array<Vector3<float>> *pointsShift,
			float shiftThresh,
			Array<int> *validShiftedPointsMIdx);
		void FindPointsWithShift(
			Array<Point> pointsM,
			Array<int> validPointsMIdx,
			Array<OrientedPoint> pointsQ,
			PointAssociationData *pPointAssociationData,
			Vector3<float> *pointsShift,
			float *shiftThresh,
			Array<int> *pointsIdx);
		void CalculateROI(Mesh *pMesh, Array<int> pointsIdx, Box<float> *ROI);
		void FindMeanPoint(Array<OrientedPoint> *points, float *meanPoint);
		void CalculateMeanHypothesis(Array<std::vector<RECOG::DDD::Hypothesis>> seqHypotheses);
		void EvaluateHypothesis(Mesh *pMesh, AffinePose3D pose);
		float EvaluateHypothesis(Mesh *pMesh, RECOG::DDD::Model *pModel, AffinePose3D pose, bool bPrintDebug = false);
		void FitBBoxToPlanarSurface(RECOG::DDD::Hypothesis *hyp, Surfel *pPlanarSurface);
		void RecognizeArticulatedObjectState(
			Mesh *pMesh,
			RECOG::DDD::ArticulatedObject AObj,
			Pose3D &poseOC,
			cv::Mat *pRGBImg,
			bool bPrintDebug = false);
		void CreateAOROI(
			Mesh *pMesh,
			RECOG::DDD::HypothesisDoorDrawer *pAObj,
			Pose3D &poseOC,
			Box<float> *ROI,
			AffinePose3D &ROI_,
			Pose3D &ROIPose);
		void VisualizeAOROI(
			Mesh *pMesh,
			AffinePose3D &ROI_,
			Pose3D &ROIPose);
		float CreateAndEvaluateAOHypothesis(
			Mesh *pMesh,
			RECOG::DDD::HypothesisDoorDrawer *pAObj,
			Pose3D &poseOC,
			float q,
			bool bPrintDebug = false);
		float CreateAndEvaluateDefaultAOHypothesis(
			Mesh *pMesh,
			RECOG::DDD::HypothesisDoorDrawer *pAObj,
			Pose3D &poseOC,
			bool bPrintDebug = false);
		void CalculateSurfelPointsInsideROI(
			Mesh *pMesh,
			Box<float> *ROI,
			Pose3D &ROIPose,
			int *pSurfelPointsInROI);
		void LoadIRITransformationsFromYAML(
			std::string path,
			double *R,
			double *t);
		void LoadGTTransformations(
			char *modelFilePath,
			RECOG::DDD::Transformation &transformation);
		void SelectAOModel(
			RECOG::DDD::Transformation *pSceneTransformation,
			Array<RECOG::DDD::Transformation> modelTransformations,
			Array<RECOG::DDD::ArticulatedObject> modelAOs,
			RECOG::DDD::ArticulatedObject &AObj,
			Array<int> &models);
		void SetPointsForPointToPointAssociationVisualization(
			Mesh *pMeshM,
			Mesh *pMeshQ,
			PointAssociationData *pPointAssociationData);
		void SetPointsForPointToPointAssociationVisualization(
			Array<OrientedPoint> pointsM,
			Mesh *pMeshQ,
			PointAssociationData *pPointAssociationData);
		void SetShiftVectorForVisualization(float *firstPoint, Vector3<float> shift, uchar *color);
		void SetShiftVectorsForVisualization(float *firstPoint, Array<Vector3<float>> shiftArray, uchar *color);
		void SetPointsForVisualization(Array<OrientedPoint> points, uchar *color, float size = 0.2);
		void VisualizeFittingScoreCalculation(Mesh *pMesh, int nTransparentPts, int *SMCorrespondence, bool bVisualizeMesh = true);
		bool GetVisualizeDoorHypotheses();
		bool GetRGBImageVisualization();
		void VisualizeDDHypothesis(
			RECOG::DDD::HypothesisDoorDrawer hyp,
			int imgID = -1);
		void VisualizeDoorHypothesis(
			RECOG::DDD::HypothesisDoorDrawer doorHyp,
			int imgID = -1);
		void VisualizeDrawerHypothesis(
			RECOG::DDD::HypothesisDoorDrawer drawerHyp,
			int imgID = -1);
		void Voter1DTest();
		void Voter3DTest();
		void Fit3DTo2D(
			Mesh *pMesh,
			Array<RECOG::DDD::EdgeSample> edgeSamplePts,
			cv::Mat edges,
			float *QNMap,
			Pose3D poseMC0,
			Array<RECOG::DDD::PtAssoc> ptAssoc,
			int maxnIterations,
			float &th_,
			float &th,
			float &et,
			float *dR,
			float *dt,
			float maxdth = 0.4f,
			bool bDebug = false);
		void Fit3DTo2D(
			Mesh *pMesh,
			cv::Mat RGB,
			Array<Pose3D> initPosesMC,
			Pose3D &poseMC);
		void Project3DModelToImage(
			Mesh *pMesh,
			Pose3D poseMC,
			RECOG::DDD::Edge *edge,
			float *PRMem,
			float *PCMem,
			float *NCMem,
			int cameraNum = 0);
		void SampleEdges(
			Mesh *pMesh,
			RECOG::DDD::Edge *edge,
			Array<RECOG::DDD::EdgeSample> &edgeSamplePts);
		void BoundingBox(
			Mesh *pMesh,
			Array<int> surfels,
			float *RSB,
			Box<float> &bbox);
		void GenerateHypothesis(
			Box<float> bbox,
			RECOG::DDD::Model *pModel,
			float *RMB,
			int *bboxAxesIdxM,
			float *bboxAxesSignM,
			float *RBS,
			RECOG::DDD::Hypothesis &hyp);
		void Project(
			RECOG::DDD::Model *pModel,
			int m,
			float *q,
			float *RMS,
			float *tMS = NULL);
		// bool ClusteringCompare(SortIndex<float> x1, SortIndex<float> x2);
		void Clustering(
			float *data,
			int nData,
			int nDims,
			float thr,
			int *&clusterID,
			Array<Array<int>> &clusters,
			int *&clusterMem,
			float *E = NULL);
		void Vector3DClustering(
			Array<RECOG::DDD::AxisHypothesis> axisHyps,
			float thr,
			int *&clusterID,
			Array<Array<int>> &clusters,
			int *&clusterMem);
		int MinClusterSize(
			Array<Array<int>> clusters,
			int clusterSizeThrCoeff,
			int minClusterSize);
		bool AlignHypothesisZAxis(
			RECOG::DDD::Hypothesis &hyp,
			float *ZRef,
			float csZThr);
		void AOZeroState(RECOG::DDD::ArticulatedObject AObj,
						 RECOG::DDD::AOHypothesisState *&stateMem);
		void Visualize3DModelImageProjection(
			cv::Mat displayImg,
			Mesh *pModelMesh,
			RECOG::DDD::Edge *edge,
			uchar *color = NULL);
		void SuperposeBinaryImage(
			cv::Mat displayImg,
			uchar *binImg,
			uchar *color);
		void VisualizeEdgeSamples(
			cv::Mat displayImg,
			Array<RECOG::DDD::EdgeSample> edgeSamplePt);
		void Visualize2DPointAssociation(
			cv::Mat displayImg,
			Array<RECOG::DDD::PtAssoc> ptAssoc,
			RECOG::DDD::EdgeSample *edgeSamplePt);
		void VisualizeStorageVolumeModel(
			RECOG::DDD::Model *pModel,
			RECOG::DDD::HypothesisSV hyp,
			Array<Point> &samplePts,
			Array<Pair<int, int>> *pNormals = NULL);
		void VisualizeSequenceHypothesis(Array<RECOG::DDD::Hypothesis> hyps);
		void VisualizeRectangularStructure(
			Mesh *pMesh,
			RECOG::DDD::RectStruct *pRectStruct,
			uchar *boxColor = NULL,
			float *tRS = NULL);
		void VisualizeHypothesisBBoxes(std::vector<RECOG::DDD::Hypothesis> *pHyps);
		void VisualizeBoxFrontFaceRGB(
			Pose3D pose,
			float *s,
			cv::Mat *pRGBDisplay,
			uchar *color = NULL);
		void VisualizeArticulatedObject(
			RECOG::DDD::ArticulatedObject AObj,
			Pose3D poseOC,
			bool bDisplayRS = false,
			cv::Mat *pRGBDisplay = NULL);
		void Get2DObjectPoints(
			RECOG::DDD::HypothesisDoorDrawer movingPart,
			// Pose3D poseOC,
			PSD::Point2D *&pts);
		float CalculateArea(std::vector<PSD::Point2D> ptsIn);
		void IoUHypothesisEvaluation(
			RECOG::DDD::HypothesisDoorDrawer movingPart,
			std::vector<std::vector<PSD::Point2D>> gtPoints,
			float *&iouResults,
			cv::Mat *pRGBDisplay);
		void cvIoUHypothesisEvaluation(
			RECOG::DDD::HypothesisDoorDrawer movingPart,
			std::vector<std::vector<PSD::Point2D>> gtPoints,
			float *&iouResults,
			cv::Mat *pRGBDisplay);

		void VectorPSDPoin2DToVectorCvPoint(std::vector<PSD::Point2D> *ptsIn, std::vector<cv::Point> &ptsOut);

		void VisualizeHypothesisGT(
			cv::Mat pRGBDisplay,
			std::vector<PSD::Point2D> detectedPts,
			std::vector<PSD::Point2D> gtPts);
		float GetIntersectionConvexSet(std::vector<PSD::Point2D> poly1, std::vector<PSD::Point2D> poly2, std::vector<PSD::Point2D> *outPoly);
		void GetConvexPoints(
			std::vector<PSD::Point2D> ptsIn,
			std::vector<PSD::Point2D> &ptsOut);
		void GetGTPointsFromCSV(
			std::vector<std::vector<std::string>> csvContent,
			std::vector<std::vector<PSD::Point2D>> &allGTPoints);

		bool CreateMeshFromPolyData(Mesh *pMesh);
		void CreateBox(
			Mesh *pMesh,
			float size[3]);
		void CreateCylinder(
			Mesh *pMesh,
			float r,
			float h,
			int resolution);
		void CreateLHTCPModel(
			Mesh *pMesh,
			float r1,
			float h1,
			float r2,
			float h2,
			int resolution);
		void RotateRectStruct(
			RECOG::DDD::RectStruct *pMRectStruct,
			int *iMAxisMap,
			float *dirMAxisMap,
			int *iQAxisMap,
			Array<RECOG::DDD::Rect3D> &MRects);
		// Simundic funcs
		void getPolarLineFromCartPoints(float *, cv::Point, cv::Point);
		void getLineParamFromPoints(float *, cv::Point, cv::Point);
		bool checkPointOnLine(cv::Mat *, cv::Point, float, float);
		void HoughLinesPtAssociationCylinder(
			cv::Mat &,
			RVL::RECOG::DDD::PtAssoc *,
			RVL::Mesh *,
			Array<RECOG::DDD::EdgeSample> &,
			RVL::RECOG::DDD::Edge *,
			bool);
		void HoughLinesPtAssociationLHTCP(
			cv::Mat &edges,
			float *QNMap,
			double *IuMap,
			double *IvMap,
			RVL::Mesh *pMesh,
			RVL::RECOG::DDD::Edge *edge,
			Camera &camera_,
			bool bHoughLinesVis,
			Array<RECOG::DDD::EdgeSample> &edgeSamplePts,
			std::vector<int> &fifthAssocPtHypsIdxs,
			RVL::RECOG::DDD::PtAssoc *ptAssoc,
			std::vector<int> &fifthMIdxs);

		void drawLine(cv::Mat &image, cv::Vec2f houghLine, cv::Scalar color);

		void setupHoughLineEdges(
			std::vector<cv::Vec2f> &vecHoughLines,
			cv::Mat &edges,
			int *houghLinesIdxs);
		// void CamerasExtrinsicCalibration(cv::Mat chessB1, cv::Mat chessB2);
		void CamerasExtrinsicCalibration();
		void CamerasIntrinsicCalibration(std::string imagesPath, cv::Size chessBSize);

		void StereoCalibrationFromImages(
			std::string imagesPath,
			cv::Mat cameraMatrix1,
			cv::Mat distCoeffs1,
			cv::Mat cameraMatrix2,
			cv::Mat distCoeffs2,
			RVL::Pose3D *poseC_C0);

		void Fit3DTo2DStereo(
			Mesh *pMesh,
			cv::Mat *RGBs,
			int imSize,
			// cv::Mat RGB2,
			Array<Pose3D> initPosesMC,
			Pose3D *poseC_C0,
			Pose3D &bestPoseMC,
			cv::Mat *&solutions,
			int counter);

		RVL::Pose3D Fit3DTo2DStereo2(
			Mesh *pMesh,
			cv::Mat *RGBs,
			int imSize,
			// cv::Mat RGB2,
			Array<Pose3D> initPosesMC,
			Pose3D *poseC_C0,
			Pose3D &bestPoseMC,
			cv::Mat *&solutions,
			int counter);

		void Fit3DTo2D2(
			Mesh *pMesh,
			Array<RECOG::DDD::EdgeSample> edgeSamplePts,
			cv::Mat *edges,
			// float** QNMap,
			Pose3D *poseC_C0,
			Pose3D poseMC0,
			Array<RECOG::DDD::PtAssoc> ptAssoc,
			int maxnIterations,
			float &th_,
			float &th,
			float &et,
			float *dR,
			float *dt,
			float maxdth = 0.4f,
			bool bDebug = false);
		void Fit3DTo2D2S(
			Mesh *pMesh,
			Array<RECOG::DDD::EdgeSample> edgeSamplePts,
			cv::Mat *edges,
			RVL::RECOG::DDD::Edge *edge,
			Pose3D *poseC_C0,
			Pose3D poseMC0,
			Array<RECOG::DDD::PtAssoc> ptAssoc,
			float *PRMem,
			float *PCMem,
			float *NCMem,
			int maxnIterations,
			float &th_,
			float &th,
			float &et,
			float *dR,
			float *dt,
			float maxdth = 0.4f,
			bool bDebug = false);
		void Fit3DTo2D2SS(
			Mesh *pMesh,
			// Array<RECOG::DDD::EdgeSample> edgeSamplePts,
			cv::Mat *edges,
			RVL::RECOG::DDD::Edge *edge,
			Pose3D *poseC_C0,
			Pose3D poseMC0,
			Array<RECOG::DDD::PtAssoc> ptAssoc,
			// float* PRMem,
			// float* PCMem,
			// float* NCMem,
			int maxnIterations,
			float &th_,
			float &th,
			float &et,
			float *dR,
			float *dt,
			float maxdth = 0.4f,
			bool bDebug = false);
		void setupFit3DTo2D(
			int numImg,
			cv::Mat *RGBs,
			cv::Mat *edges,
			float *&QImgNMap,
			double **&IuMap,
			double **&IvMap,
			float **&QNMap);
		static bool compareEdgeSampleY(
			cv::Point &edgeSample1,
			cv::Point &edgeSample2);
		float distanceDifOf2Pts(cv::Point p, cv::Point L, cv::Point R);
		int findNextEdge(std::vector<int> topFaceEdges, RVL::RECOG::DDD::Edge *edge, int iVertexBack);
		void ModelPtAssociationLHTCP(
			RVL::Mesh *pMesh,
			RVL::RECOG::DDD::Edge *edge,
			Array<RECOG::DDD::EdgeSample> &edgeSamplePts,
			RVL::RECOG::DDD::PtAssoc *ptAssoc,
			std::vector<int> &fifthMIdxs);
		void loadBaslerCamerasParams(
			std::string loadPath,
			cv::Mat *&camsMatrix,
			cv::Mat *&camsDist);
		void loadBaslerExtrinsicParams(std::string loadPath, RVL::Pose3D *poseC_C0);
		void SampleEdges2(
			Mesh *pMesh,
			RECOG::DDD::Edge *edge,
			Array<RECOG::DDD::EdgeSample> &edgeSamplePts,
			int cameraNum);

		void VisualizeCannyTrackbar(cv::Mat *images);
		bool ParseCSV(std::string path, bool bHasHeader, std::vector<std::vector<std::string>> &content);

	public:
		DWORD mode;
		DWORD model;
		CRVLMem *pMem;
		CRVLMem *pMem0;
		CRVLParameterList paramList;
		Camera camera;
		Camera cameras[2];
		SurfelGraph *pSurfels;
		PlanarSurfelDetector *pSurfelDetector;
		SurfelGraph planarSurfaces;
		float beta;
		float alphas;
		float alphaR;
		float alphat;
		int minSurfelSize;
		int minEdgeSize;
		int minRefSurfelSize;
		float normalSimilarity;
		int nICPIterations;
		bool bHypGenConvex;
		bool bHypGenFlat;
		bool bLoadMovingPartHypothesesFromFile;
		bool bLoadDDHypothesesFromFile;
		Array<RECOG::DDD::Model> models;
		bool bVisualize;
		std::string cfgFileName;
		DWORD test;
		float storageVolumeWallThickness;
		Array2D<Point> ZBuffer;
		Array<int> ZBufferActivePtArray;
		int *subImageMap;
		int sceneSamplingResolution;
		int image3x3Neighborhood[9];
		float chamferDistThr;
		float transparencyDepthThr;
		DWORD edgeModel;
		int fit3DTo2DEDTMaxDist;
		int fit3DTo2DMinDefDoFs;
		int fit3DTo2DNoEdgeSamples;
		float fit3DTo2DLambdaR;
		float fit3DTo2DLambdat;
		float fit3DTo2DMaxOrientationCorrection;
		float fit3DTo2DMaxPositionCorrection;
		bool fit3DTo2DStereo;
		int nRANSACIterations;
		int pointAssociationGridCellSize;
		float maxCoPlanarSurfelRefPtDist;
		float maxCoPlanarSurfelNormalAngle;
		float openingDirectionTol;
		char *resultsFolder;
		Array<RECOG::DDD::SurfelEdge> *surfelEdges_;
		Array<RECOG::DDD::SurfelEdge *> *planarSurfeceEdges;
		RECOG::DDD::SurfelEdge **planarSurfeceEdgeMem;
		float frontFaceThickness;
		int maxROIStep;
		float rectStructAlignmentCorrectionBounds[3];
        float orthogonalViewEdgeLineAngleTol;
        float orthogonalViewPixSize;

	private:
		RECOG::DDD::DisplayCallbackData *pVisualizationData;
		Array<RECOG::DDD::SurfelEdge> surfelEdges;
		bool *surfelMask;
		bool *sampleMask;
		Grid imgGrid;
		int ROICalculationStep;
		float minPointShift;
		int ROISceneSubsampleCellSize;
		float ROISceneSubsampleNormalSimilarity;
		int ROIPointAssociationGridCellSize;
		float votingCellSizeX;
		float votingCellSizeY;
		float votingCellSizeZ;
		float shiftThreshX;
		float shiftThreshY;
		float shiftThreshZ;
		int minPointsWithDominantShift;
		Array<int> pseudornd;
		int iRnd;
	};

}
