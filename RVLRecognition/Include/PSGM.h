#pragma once
#include "RVLVTK.h"

//#define RVLPSGM_NORMAL_HULL
#define RVLPSGM_MATCH_SATURATION //VIDOVIC
//#define RVLPSGM_MATCH_SEGMENT_CENTROID //VIDOVIC
//#define PSGM_CALCULATE_PROBABILITY //Vidovic
#define RVLPSGM_EVALUATION_PRINT_INFO //Vidovic
#define RVLPSGM_MATCH_USING_SEGMENT_GT //Vidovic
#define RVLPSGM_SAVE_MATCHES //Vidovic
#define RVLPSGM_MATCHES_SIMILARITY_MEASURE			3
//#define RVLPSGM_RANSAC

#define RVLRECOGNITION_MODE_PSGM_CREATE_CTIS		2
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
				float cost_ICP; 
				float R_ICP[9];
				float t_ICP[3];
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

	class PSGM
	{
	public:
		PSGM();
		virtual ~PSGM();
		//void Create();
		void CreateParamList(CRVLMem *pMem);
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

		void UpdateMatchMatrix(
			RECOG::PSGM_::SegmentMatch *SMatch,			
			int iCTI
			);

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

		typedef void(*ICPfunction)(vtkSmartPointer<vtkPolyData>, vtkSmartPointer<vtkPolyData>, float *, int, float, int, double*);

		void AddBestCTIModelsToVisualizer(Visualizer *pVisualizer, bool align, ICPfunction ICPFunction, int ICPvariant);
		
		void AddCTIModelToVisualizer(Visualizer *pVisualizer, int iMatch, bool align, ICPfunction ICPFunction, int ICPvariant);
		
		void LoadModelMeshDB(char *modelSequenceFileName);

		vtkSmartPointer<vtkPolyData> GetSceneModelPC(int iCluster);

		void CalculateICPCost(RVL::PSGM::ICPfunction ICPFunction, int ICPvariant);
		//end Petra

		void InitDisplay(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			unsigned char *selectionColor);
		void Display();
		void DisplayModelInstance(Visualizer *pVisualizer);
		void DisplayClusters();
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
		bool IsFlat(
			Array<int> SurfelArray,
			float *N,
			float &d,
			Array<int> PtArray);
		bool DetectGroundPlane(SURFEL::ObjectGraph *pObjects);
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
		void CalculateScore(int similarityMeasure = 3); //Vidovic
		void UpdateScoreMatchMatrix(RECOG::PSGM_::ModelInstance *pSModelInstance); //Vidovic
		void SortScoreMatchMatrix(bool descending = false); //Vidovic
		void EvaluateMatchesByScore(
			FILE *fp,
			FILE *fpLog,
			int nBestSegments = 0); //Vidovic
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
		bool PSGM::CompareMatchToGT(RECOG::PSGM_::MatchInstance *pMatch, ECCVGTLoader *ECCVGT, bool poseCheck, float angleThresh, float distanceThresh); //VIDOVIC
		void PSGM::CountTPandFN(ECCVGTLoader *ECCVGT, int &TP, int &FN, bool printMatchInfo); //VIDOVIC

	private:
		void Clusters();
		void CreateTemplate();
		void TemplateMatrix(Array2D<float> A);
		void FitModel(
			//RECOG::PSGM_::Cluster *pCluster, //Vidovic
			RECOG::PSGM_::ModelInstance *pModelInstance);
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
		float DistanceFromNormalHull(
			Array<SURFEL::NormalHullElement> &NHull,
			float *N);
		void UpdateMeanNormal(
			float *sumN,
			float &wN,
			float *N,
			float w,
			float *meanN);
		void ComputeClusterNormalDistribution(
			RECOG::PSGM_::Cluster *pCluster);
		void ComputeClusterBoundaryDiscontinuityPerc(int iCluster);
		void AddReferenceFrame(
			//int iCluster, //Vidovic
			float *R = NULL,
			float *t = NULL);
		void SaveModelInstances(
			FILE *fp,
			int iModel = - 1);

	public:
		CRVLParameterList ParamList;
		DWORD mode;
		CRVLMem *pMem;
		PlanarSurfelDetector *pSurfelDetector;
		SurfelGraph *pSurfels;
		Mesh *pMesh;
		RECOG::PSGM_::DisplayData displayData;
		Array<RECOG::PSGM_::Cluster *> clusters;
		int *clusterMap;
		int nDominantClusters;
		float kNoise;
		Array<RECOG::PSGM_::Plane> convexTemplate;
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
		Array<RECOG::PSGM_::ModelInstance> modelInstanceDB; //Vidovic
		QList<RECOG::PSGM_::MatchInstance> CTImatches; //Vidovic
		Array<RECOG::PSGM_::MatchInstance*> pCTImatchesArray; //Vidovic
		//RECOG::PSGM_::MatchInstance *pCurrentSceneMatch; //Vidovic
		//QList<RECOG::PSGM_::MatchInstance> SSegmentMatches1; //Vidovic - probability1
		//QList<RECOG::PSGM_::MatchInstance> SSegmentMatches2; //Vidovic - probability2
		Array<Array<SortIndex<float>>> scoreMatchMatrix;
		DWORD scoreCalculation; //Vidovic - TO DO (Implement read from cfg file)
		ECCVGTLoader *pECCVGT; //Vidovic
		Array <RVL::SegmentGTInstance> segmentGT;
		RECOG::CTISet CTISet;
		RECOG::CTISet MCTISet;
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
		float NGnd[3];
		float dGnd;

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
	};

	

}
