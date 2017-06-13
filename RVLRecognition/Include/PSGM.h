#pragma once
#include "RVLVTK.h"

//#define RVLPSGM_NORMAL_HULL
#define RVLPSGM_MATCH_SATURATION //VIDOVIC
//#define RVLPSGM_MATCH_SEGMENT_CENTROID //VIDOVIC
//#define PSGM_CALCULATE_PROBABILITY //Vidovic
#define RVLPSGM_EVALUATION_PRINT_INFO //Vidovic
#define RVLPSGM_MATCH_USING_SEGMENT_GT //Vidovic
#define RVLPSGM_SAVE_MATCHES //Vidovic
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SQUARE_DISTANCE									1
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_MAX_ABS_DISTANCE										2
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_SATURATED_SQUARE_DISTANCE_INVISIBILITY_PENAL			3
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SATURATED_SQUARE_DISTANCE							4
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_MEDIAN_ABS_DISTANCE									5
//#define RVLPSGM_RANSAC
#define RVLPSGM_ICP		// 170601: ON
#define RVLPSGM_ICP_SIMILARITY_MEASURE_COSTNN				0
#define RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE		1


#define RVLRECOGNITION_MODE_PSGM_CREATE_CTIS		2

#define RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI		0
#define RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY		1
#include "Eigen\Dense"
namespace RVL
{
	class PSGM;
	class CTISet;
	namespace RECOG
	{
		namespace PSGM_
		{
			struct Cluster
			{
				Array<int> iSurfelArray;
				Array<int> iVertexArray;
				int size;
				int boundaryDiscontinuityPerc;
				float N[3];
				float normalDistributionStd1;
				float normalDistributionStd2;
				bool bValid;
				//QList<RECOG::PSGM_::ModelInstance> modelInstanceList; //Vidovic
			};

			struct TangentRegionGrowingData
			{
				RECOG::PSGM_::Plane planeA;
				float cs;
				int iCluster;
				PSGM *pRecognition;
				Array<RECOG::PSGM_::Tangent> *pTangentArray;
				bool *bParent;
				//Array<RECOG::PSGM_::NormalHullElement> *pNormalHull;
				float baseSeparationAngle;
				bool *bBase;
			};

			struct DisplayData
			{
				PSGM *pRecognition;
				Mesh *pMesh;
				SurfelGraph *pSurfels;
				Visualizer *pVisualizer;
				bool bClusters;
				unsigned char selectionColor[3];
				int iSelectedCluster;
				vtkSmartPointer<vtkActor> referenceFrames;
				DWORD hypothesisVisualizationMode;
			};


			//Petra
			struct SegmentMatch
			{
				float Eseg;
				int iCTIs;
				int iCTIm;
				int iSS;
				int iSM;
				int iM;
				Eigen::VectorXf t;
			};


			//VIDOVIC
			struct MatchInstance
			{
				int ID;
				int iScene;
				int iSCTI;
				int iMCTI;
				int iMS;
				int iSS;
				float R[9];
				float t[3];
				float tMatch[3];
				float E;
				float score;
				float probability1;
				float probability2;
				float angleGT;
				float distanceGT;
				int nValids;
				float eSeg;
				// Petra
				double cost_ICP; 
				float T_ICP[16]; //transformation between current and ICP pose
				float RICP_[9]; //transformation between current and ICP pose
				float tICP_[3]; //transformation between current and ICP pose
				float RICP[9]; //Pose after ICP
				float tICP[3]; //Pose after ICP
				double cost_NN;
				// end Petra
				MatchInstance *pNext;
			};

			struct FPMatch
			{
				int iScene;
				int iModel;
				float t[3];
				int n;
				FPMatch *pNext;
			};
			//END Vidovic

			struct SymmetryMatch
			{
				float d;
				float w;
				bool b;
				int iCTIElement;
			};

			struct Hypothesis
			{
				float R[9];
				float P[3];
				int iMatch;
				int iCell;
				float score;
				Hypothesis *pNext;
				Hypothesis **pPtrToThis;
			};

			int ValidTangent(
				int iSurfel,
				int iSurfel_,
				SURFEL::Edge *pEdge,
				SurfelGraph *pSurfels,
				RECOG::PSGM_::TangentRegionGrowingData *pData);
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
		}	// namespace PSGM_
		}
	//class CTISet
	//{
	//public:
	//	CTISet();
	//	virtual ~CTISet();

	//	void LoadSMCTI(char * filePath, Array<RECOG::PSGM_::Plane> *convexTemplate);

	//	Array<RECOG::PSGM_::ModelInstance> CTI;
	//	//std::vector<std::vector<int>> SegmentCTIs;
	//	Array<Array<int>> SegmentCTIs;
	//	int *segmentCTIIdxMem;
	//};


	template <typename T>
	struct NanoFlannPointCloud
	{
		struct Point
		{
			T  x, y, z;
		};

		std::vector<Point>  pts;

		// Must return the number of data points
		inline size_t kdtree_get_point_count() const { return pts.size(); }

		// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
		inline T kdtree_distance(const T *p1, const size_t idx_p2, size_t /*size*/) const
		{
			const T d0 = p1[0] - pts[idx_p2].x;
			const T d1 = p1[1] - pts[idx_p2].y;
			const T d2 = p1[2] - pts[idx_p2].z;
			return d0*d0 + d1*d1 + d2*d2;
		}

		// Returns the dim'th component of the idx'th point in the class:
		// Since this is inlined and the "dim" argument is typically an immediate value, the
		//  "if/else's" are actually solved at compile time.
		inline T kdtree_get_pt(const size_t idx, int dim) const
		{
			if (dim == 0) return pts[idx].x;
			else if (dim == 1) return pts[idx].y;
			else return pts[idx].z;
		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

	};


	class PSGM
	{
	public:
		PSGM();
		virtual ~PSGM();
		//void Create();
		void CreateParamList(CRVLMem *pMem);
		void Init(Mesh *pMesh);
		void Interpret(
			Mesh *pMesh,
			int iScene = 0);
		
		//Petra
		void InterpreteCTIS(
			Mesh *pMesh);
		
		void MatchInPrimitiveSpace(
			Eigen::MatrixXf QM,
			Eigen::MatrixXf M,
			int iCTI
			);

		void CTIMatch(
			Eigen::MatrixXf dM,
			int iCTI);

		//void UpdateMatchMatrix(
		//	RECOG::PSGM_::SegmentMatch *SMatch,			
		//	int iCTI
		//	);

		void VisualizeCTIMatch( //Damir
			float *nT, 
			float *dM, 
			float *dS, 
			int *validS);
		
		Eigen::MatrixXf ConvexTemplatenT();

		void VisualizeCTIMatchidx( //for a given Scene and Model CTI index, calls visualization (prepares descriptors and visibility mask).
			int iSCTI,
			int iMCTI);

		void CalculatePose(int iMatch);

		typedef void(*ICPfunction)(vtkSmartPointer<vtkPolyData>, vtkSmartPointer<vtkPolyData>, float*, int, float, int, double*, void*);

		//For a given scene segment adds desired ranked hypotheses to visualizer:
		void AddModelsToVisualizer(Visualizer *pVisualizer, bool align, ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr = NULL);
		
		//Runs ICP for a hypotheses chosen in "AddModelsToVisualizer" and visualizes it (not recomended, rather use AddOneModelToVisualizer):
		void AddOneModelToVisualizerICP(Visualizer *pVisualizer, int iMatch, bool align, ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr = NULL);
		
		//Recomended,
		//Visualizes chosen hypotheses 0-6 for each segment on the scene, activated when pressed "c":
		void AddOneModelToVisualizer(Visualizer *pVisualizer, int iMatch, int iRank, bool align);

		//Visualizes GT models on the scene, activated when pressed "g":
		void AddGTModelsToVisualizer(Visualizer *pVisualizer);
		
		void LoadModelMeshDB(char *modelSequenceFileName, std::map<int, vtkSmartPointer<vtkPolyData>> *vtkModelDB, bool bDecimate = false, float decimatePercent = 0.4);

		vtkSmartPointer<vtkPolyData> GetSceneModelPC(int iCluster);

		void CalculateICPCost(RVL::PSGM::ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr = NULL); 

		static vtkSmartPointer<vtkPolyData> GetVisiblePart(vtkSmartPointer<vtkPolyData> PD); // Models are reduced to only the visible part (using angle between normals) which improves ICP. 

		static vtkSmartPointer<vtkPolyData> GetVisiblePart(vtkSmartPointer<vtkPolyData> PD, double *T_M_S); // Models are reduced to only the visible part (using angle between normals) which improves ICP.

		void CalculateNNCost(Visualizer *pVisualizer, RVL::PSGM::ICPfunction ICPFunction, int ICPvariant); // For each pair of scene segment and visible part of the matched model, calls NNCost.

		float NNCost(int iCluster, vtkSmartPointer<vtkPolyData> sourcePD, vtkSmartPointer<vtkPolyData> targetPD, int similarityMeasure = 0); // Calculates cost based on sum of distances between scene segment points and their nearest neighbours in visible part of the matched model.
		//end Petra

		float groundPlaneDistance(int iModel, double *MSTransform); //Vidovic

		void RMSE(FILE *fp, bool allTPHypotheses = false); //calculate RMSE for TP hypothesis on the scene (only 0-th placed TP hypotheses-> allTPHypothesis = false; all TP hypotheses -> allTPHypothesis = true) - Vidovic

		float RMSE(int iModel, double *TGT, double *T); //calculate RMSE for single model - Vidovic

		void InitDisplay(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			unsigned char *selectionColor);
		void Display();
		void DisplayModelInstance(Visualizer *pVisualizer);
		void DisplayClusters();
		void DisplayCTIs(
			Visualizer *pVisualizer,
			RECOG::CTISet *pCTISet,
			Array<int> *pCTIArray = NULL);
		void DisplayCTI(
			Visualizer *pVisualizer,
			RECOG::PSGM_::ModelInstance *pCTI);
		void PaintCluster(
			int iCluster,
			unsigned char *color);
		void PaintClusterVertices(
			int iCluster,
			unsigned char *color);
		void DisplayReferenceFrames();
		void SetSceneFileName(char *sceneFileName_);
		bool ModelExistInDB(
			char *modelFileName,
			FileSequenceLoader dbLoader); //Vidovic
		void SaveModelID(FileSequenceLoader dbLoader); //Vidovic
		void Learn(
			char *modelSequenceFileName,
			Visualizer *visualizer = NULL); //Vidovic
		void LoadModelDataBase(); //Vidovic
		void Match(); //Vidovic
		void Match(
			RECOG::PSGM_::ModelInstance *pSModelInstance,
			int startIdx,
			int endIdx); //Vidovic
		void MatchTGs();
		void AddSegmentMatches(
			int iCluster,
			Space3DGrid<RECOG::PSGM_::Hypothesis, float> &HSpace,
			Array<int> &iMergingCandidates);
		bool IsFlat(
			Array<int> SurfelArray,
			float *N,
			float &d,
			Array<int> PtArray);
		void DetectGroundPlane(SURFEL::ObjectGraph *pObjects);
		bool GravityReferenceFrame(
			QList<QLIST::Index> surfelList,
			float *RGC,
			float &varX);
		int CTIs(
			QList<QLIST::Index> surfelList,
			Array<int> iVertexArray,
			int iModel,
			int iCluster,
			RECOG::CTISet *pCTISet,
			CRVLMem *pMem);
		void CTIs(
			SURFEL::ObjectGraph *pObjects,
			RECOG::CTISet *pCTISet);
		void FitModel(
			Array<int> iVertexArray,
			RECOG::PSGM_::ModelInstance *pModelInstance,
			bool bMemAllocated = false);
		float Symmetry(
			SURFEL::ObjectGraph *pObjects,
			int iObject1,
			int iObject2,
			RECOG::CTISet *pCTIs,
			Array<RECOG::PSGM_::SymmetryMatch> &symmetryMatch);
		void PrintMatchInfo(
			FILE *fp,
			FILE *fpLog,
			int TP_,
			int FP_,
			int FN_,
			float precision,
			float recall,
			int nSSegments,
			int *firstTP,
			int *firstTPiModel,
			float *firstTPScore,
			float scoreThresh,
			float minScore,
			float maxScore,
			float scoreStep,
			int nBestSegments,
			int iBestMatches,
			int graphID); //Vidovic
		void CalculateScore(
			int similarityMeasure = 3,
			int iFirstCTI = 0,
			int iEndCTI = -1); //Vidovic
		void UpdateScoreMatchMatrix(RECOG::PSGM_::ModelInstance *pSModelInstance); //Vidovic
		void SortScoreMatchMatrix(bool descending = false); //Vidovic
		void EvaluateMatchesByScore(
			FILE *fp,
			FILE *fpLog,
			FILE *fpPoseError,
			FILE *fpnotFirstInfo,
			FILE *fpnotFirstPoseErr,
			int nBestSegments = 0,
			bool evaluateICP = false); //Vidovic
		void WriteClusterNormalDistribution(FILE *fp);
		void MSTransformation(
			RECOG::PSGM_::ModelInstance *pMModelInstance,
			RECOG::PSGM_::ModelInstance *pSModelInstance,
			float *tBestMatch,
			float *R,
			float *t); //Vidovic
		void SaveMatches(); //Vidovic
		bool PSGM::CompareMatchToGT(
			RECOG::PSGM_::MatchInstance *pMatch,
			bool poseCheck,
			float angleThresh,
			float distanceThresh); //Vidovic
		bool PSGM::CompareMatchToSegmentGT(
			RECOG::PSGM_::MatchInstance *pMatch); //Vidovic
		bool PSGM::CompareMatchToSegmentGT(
			int iScene,
			int iSSegment,
			int iMatchedModel); //Vidovic
		void PSGM::CountTPandFN(
			int &TP,
			int &FN,
			bool printMatchInfo); //Vidovic
		void PSGM::CalculatePR(
			int TP,
			int FP,
			int FN,
			float &precision,
			float &recall); //Vidovic
		void ConvexTemplateCentoidID(); //Vidovic
		void SaveSegmentGT(
			FILE*fp,
			int iScene); //Vidovic
		void LoadSegmentGT(
			FILE*fp,
			int iScene); //Vidovic
		void LoadCompleteSegmentGT(FileSequenceLoader sceneSequence); //Vidovic
		void LoadCTI(char *fileName); //Vidovic
		bool PoseCheck(
			RVL::GTInstance *pGT,
			RECOG::PSGM_::MatchInstance *pMatch,
			float distanceThresh,
			float angleThresh,
			FILE *fpLog = NULL,
			FILE *fpnotFirstPoseErr = NULL,
			bool evaluateICP = false); //Vidovic
		void FindGTInstance(
			RVL::GTInstance **pGT,
			int iScene,
			int iModel);
		//bool PSGM::CompareMatchToGT(RECOG::PSGM_::MatchInstance *pMatch, ECCVGTLoader *ECCVGT, bool poseCheck, float angleThresh, float distanceThresh); //VIDOVIC
		//void PSGM::CountTPandFN(ECCVGTLoader *ECCVGT, int &TP, int &FN, bool printMatchInfo); //VIDOVIC
		void CreateScoreMatchMatrixICP();
		void FindMinMaxInScoreMatchMatrix(
			float &min,
			float &max,
			Array<Array<SortIndex<float>>> &scoreMatchMatrix_);
		void SaveCTIs(
			FILE *fp,
			RECOG::CTISet *pCTISet,
			int iModel = -1);
		void RVLPSGInstanceMesh(Eigen::MatrixXf nI, float *dI);
		void BoundingBoxSize(
			RECOG::PSGM_::ModelInstance *pBoundingBox,
			float *size);

	private:
		void Clusters();
		void CreateTemplate66();
		void CreateTemplateBox();
		void TemplateMatrix(Array2D<float> &A);
		bool ReferenceFrames(int iCluster);
		bool ReferenceFrames(
			RECOG::PSGM_::Cluster *pCluster,
			int iCluster = -1);
		bool Inside(
			int iVertex,
			RECOG::PSGM_::Cluster *pCluster,
			int iSurfel = -1);
		bool BelowPlane(
			RECOG::PSGM_::Cluster *pCluster,
			Surfel *pSurfel,
			int iFirstVertex = 0);
		void UpdateMeanNormal(
			float *sumN,
			float &wN,
			float *N,
			float w,
			float *meanN);
		void ComputeClusterNormalDistribution(
			RECOG::PSGM_::Cluster *pCluster);
		void ComputeClusterBoundaryDiscontinuityPerc(int iCluster);
		RECOG::PSGM_::ModelInstance *AddReferenceFrame(
			//int iCluster, //Vidovic
			float *R = NULL,
			float *t = NULL);
		void SaveModelInstances(
			FILE *fp,
			int iModel = - 1);
		void PrintCTIMeshFaces(FILE *fp, Eigen::MatrixXi F, Eigen::MatrixXi Fn, int n, Eigen::MatrixXi nP);

	public:
		CRVLParameterList ParamList;
		DWORD mode;
		CRVLMem *pMem;
		CRVLMem *pMem0;
		PlanarSurfelDetector *pSurfelDetector;
		SurfelGraph *pSurfels;
		Mesh *pMesh;
		RECOG::PSGM_::DisplayData displayData;
		Array<RECOG::PSGM_::Cluster *> clusters;
		int *clusterMap;
		int nDominantClusters;
		float kNoise;
		Array<RECOG::PSGM_::Plane> convexTemplate;
		Array<RECOG::PSGM_::Plane> convexTemplate66;
		Array<RECOG::PSGM_::Plane> convexTemplateBox;
		int minInitialSurfelSize;
		int minVertexPerc;
		float kReferenceSurfelSize;
		float kReferenceTangentSize;
		float baseSeparationAngle;
		float edgeTangentAngle;
		int nModels; //Vidovic
		int nMSegments; //Vidovic
		int minClusterSize;
		int maxClusterSize;
		int minSignificantClusterSize;
		int minClusterBoundaryDiscontinuityPerc;
		float minClusterNormalDistributionStd;
		float groundPlaneTolerance;
		bool bZeroRFDescriptor;
		bool bGTRFDescriptors;
		bool bMatchRANSAC; //Vidovic
		bool bGnd;
		Array<RECOG::PSGM_::ModelInstance> modelInstanceDB; //Vidovic
		QList<RECOG::PSGM_::MatchInstance> CTImatches; //Vidovic
		Array<RECOG::PSGM_::MatchInstance*> pCTImatchesArray; //Vidovic
		//RECOG::PSGM_::MatchInstance *pCurrentSceneMatch; //Vidovic
		//QList<RECOG::PSGM_::MatchInstance> SSegmentMatches1; //Vidovic - probability1
		//QList<RECOG::PSGM_::MatchInstance> SSegmentMatches2; //Vidovic - probability2
		Array<Array<SortIndex<float>>> scoreMatchMatrix;
		Array<Array<SortIndex<float>>> scoreMatchMatrixICP;
		Array<Array<SortIndex<float>>> sceneSegmentMatches;
		Array<SortIndex<float>> sceneSegmentMatchesArray;
		//Array2D<Array<int>> matchMatrix;
		//int *matchMatrixMem;
		DWORD scoreCalculation; //Vidovic - TO DO (Implement read from cfg file)
		ECCVGTLoader *pECCVGT; //Vidovic
		Array <RVL::SegmentGTInstance> segmentGT;
		RECOG::CTISet CTISet;
		RECOG::CTISet MCTISet;
		RECOG::TGSet STGSet;
		RECOG::TGSet MTGSet;
		CRVLTimer *pTimer;
		FILE *fpTime;
		Eigen::MatrixXf nT; //Petra
		RECOG::PSGM_::SegmentMatch *SMatch; //Petra
		SortIndex<float> *sortedMatches; //Petra
		Eigen::VectorXf E;
		Eigen::MatrixXf t;
		RECOG::CTISet CTIset;
		RECOG::CTISet MCTIset;
		std::map<int, vtkSmartPointer<vtkPolyData>> vtkModelDB;
		std::map<int, vtkSmartPointer<vtkPolyData>> vtkRMSEModelDB; //Vidovic
		std::map<int, vtkSmartPointer<vtkPolyData>> segmentN_PD; //neighbourhood

				
		//Petra & Ivan
		double *icpTMatrix;

		float NGnd[3];
		float dGnd;
		int iGndObject;
		float symmetryMatchThr;

	private:		
		RECOG::PSGM_::Cluster *clusterMem;
		int *clusterSurfelMem;
		int *clusterVertexMem;
		//RECOG::PSGM_::ModelInstanceElement *modelInstanceMem;
		vtkSmartPointer<vtkPolyData> referenceFramesPolyData;
		char *sceneFileName;
		char *modelDataBase; //Vidovic
		char *modelsInDataBase; //Vidovic
		//int nSamples; //RANSAC //Vidovic
		int stdNoise; //RANSAC //Vidovic
		bool bNormalValidityTest; // Vidovic
		bool bBoundingPlanes;
		char *sceneMIMatch; //Vidovic
		int iScene; //Vidovic
		Array<QLIST::Index> centroidID; //Vidovic
		QList<QLIST::Index> *pISampleCandidateList; //Vidovic
		Array<QLIST::Index> iValidSampleCandidate; //Vidovic
		Array<QLIST::Index> iValid; //Vidovic
		Array<QLIST::Index> iRansacCandidates; //Vidovic
		Array<QLIST::Index> iConsensus; //Vidovic
		Array<QLIST::Index> iConsensusTemp; //Vidovic
		Array<Array<float>> e; //Vidovic
		Array<Array<float>> tBestMatch; //Vidovic
		Array<float> score; //Vidovic
		QList<RECOG::PSGM_::MatchInstance> *pCTImatches; //Vidovic
		RECOG::PSGM_::MatchInstance *pCTIMatch; //Vidovic
		RECOG::PSGM_::MatchInstance *pFirstSCTIMatch; //Vidovic
		int matchID; //Vidovic
		float *nTc; //Vidovic
		float *dISMc; //Vidovic
		int CTIIdx; //Vidovic
		int nBestMatches; //n best matches for each scene segment
		int debug1, debug2;
		//For InstanceMesh:
		Eigen::MatrixXf P; //points list
		Eigen::MatrixXi F; //faces list (polygones)
		Eigen::MatrixXi Edges; //Edges
	};

	

}
