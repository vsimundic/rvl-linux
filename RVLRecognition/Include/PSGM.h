#pragma once

//#define RVLPSGM_NORMAL_HULL
#define RVLPSGM_MATCH_SATURATION //VIDOVIC
//#define RVLPSGM_MATCH_SEGMENT_CENTROID //VIDOVIC
//#define PSGM_CALCULATE_PROBABILITY //Vidovic
//#define RVLPSGM_EVALUATION_PRINT_INFO //Vidovic
#define RVLPSGM_MATCH_USING_SEGMENT_GT //Vidovic
//#define RVLPSGM_SAVE_MATCHES //Vidovic

#define RVLRECOGNITION_MODE_PSGM_CREATE_CTIS		2

namespace RVL
{
	class PSGM;

	namespace RECOG
	{
		namespace PSGM_
		{
			struct ModelInstanceElement
			{
				float d;
				float e;
				bool valid;
			};

			struct ModelInstance
			{
				int iModel; // VIDOVIC
				int iCluster; // VIDOVIC
				float R[9];
				float t[3];
				float tc[3]; // VIDOVIC
				Array<ModelInstanceElement> modelInstance;
				ModelInstance *pNext;
			};

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
				QList<RECOG::PSGM_::ModelInstance> modelInstanceList;
			};

			struct Plane
			{
				float N[3];
				float d;
			};

			struct Tangent
			{
				float N[3];
				float V[3];
				float d;
				float len;
				int iVertex[2];
				bool bMerged;
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

			//VIDOVIC
			struct MatchInstance
			{
				int ID;
				int iScene;
				int iCluster;			//iSSegment
				int iCRF;				//iSRF		
				int iSMI;				//iCTIS
				int iModel;				
				int iMCluster;			//iMSegment
				int iMMI;
				float R[9];
				float t[3];
				float tMatch[3];
				float E;
				float score;
				float probability1;
				float probability2;
				float angle;			//angleGT
				float distance;			//distanceGT
				int nValids;
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
			//END VIDOVIC

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
		}
	}

	class PSGM
	{
	public:
		PSGM();
		virtual ~PSGM();
		void CreateParamList(CRVLMem *pMem);
		void Interpret(
			Mesh *pMesh,
			int iScene = 0);
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
		bool ModelExistInDB(char *modelFileName, FileSequenceLoader dbLoader); //VIDOVIC
		void SaveModelID(FileSequenceLoader dbLoader); //VIDOVIC
		void Learn(char *modelSequenceFileName); //VIDOVIC
		void LoadModelDataBase(); //VIDOVIC
		void Match(); //VIDOVIC
		void WriteClusterNormalDistribution(FILE *fp);
		void MSTransformation(
			RECOG::PSGM_::ModelInstance *pMModelInstance,
			RECOG::PSGM_::ModelInstance *pSModelInstance,
			float *tBestMatch,
			float *R,
			float *t); //VIDOVIC
		//void SetNumberOfScenes(int scenesNumber); //VIDOVIC
		void SaveMatches(); //VIDOVIC
		void CompareMatchesToGT(
			ECCVGTLoader *ECCVGT,
			float scoreThresh,
			float angleThresh,
			float distanceThresh,
			float &precision,
			float &recall); //VIDOVIC
		void CompareSMIMatchesToGT(
			ECCVGTLoader *ECCVGT,
			float scoreThresh,
			float angleThresh,
			float distanceThresh,
			float &precision,
			float &recall); //VIDOVIC
		void CompareProbabilityMatchesToGT(
			ECCVGTLoader *ECCVGT,
			float probabilityThresh,
			int probabilityCalculation,
			bool poseCheck,
			float angleThresh,
			float distanceThresh,
			float &precision,
			float &recall); //VIDOVIC
		bool PSGM::CompareMatchToGT(
			RECOG::PSGM_::MatchInstance *pMatch,
			bool poseCheck,
			float angleThresh,
			float distanceThresh); //VIDOVIC
		bool PSGM::CompareMatchToSegmentGT(
			RECOG::PSGM_::MatchInstance *pMatch,
			bool compareSegmentsWithoutGT = true); //Vidovic
		void PSGM::CountTPandFN(
			int &TP,
			int &FN,
			bool printMatchInfo); //VIDOVIC
		void PSGM::CalculatePR(
			int TP,
			int FP,
			int FN,
			float &precision,
			float &recall); //VIDOVIC
		void PSGM::CreateMatchMatrix(); //VIDOVIC
		void PSGM::ClearMatchMatrix(); //Vidovic
		void PSGM::UpdateMatchMatrix(RECOG::PSGM_::MatchInstance *pMatch, float cost); //Vidovic
		void PSGM::SortMatchMatrix(); //Vidovic
		void PSGM::EvaluateMatchesByScore(FILE *fp, FILE *fpLog); //Vidovic
		void ConvexTemplateCentoidID(); //VIDOVIC
		void FillMatch(
			RECOG::PSGM_::MatchInstance *pMatch,
			int ID,
			int iScene,
			int iSSegment,
			int iSRF,
			int iCTIS,
			int iModel,
			int iMSegment,
			int iCTIM,
			float *R,
			float *t,
			float *tMatch,
			float E,
			float score,
			float probability1,
			float probability2,
			float angleGT,
			float distanceGT,
			int nValids); //Vidovic
		void SaveSegmentGT(FILE*fp); //Vidovic
	private:
		void Clusters();
		void CreateTemplate();
		void FitModel(
			RECOG::PSGM_::Cluster *pCluster,
			RECOG::PSGM_::ModelInstance *pModelInstance);
		bool ReferenceFrames(int iCluster);
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
			int iCluster,
			float *R = NULL,
			float *t = NULL);
		void SaveModelInstances(
			FILE *fp,
			int iModel,
			int iCluster);

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
		Array<RECOG::PSGM_::ModelInstance> modelInstanceDB; //VIDOVIC
		Array<RECOG::PSGM_::MatchInstance> matches; //VIDOVIC
		RECOG::PSGM_::MatchInstance *pMatches; //VIDOVIC
		QList<RECOG::PSGM_::MatchInstance> SMImatches; //VIDOVIC
		RECOG::PSGM_::MatchInstance *pCurrentSceneMatch; //VIDOVIC
		QList<RECOG::PSGM_::MatchInstance> SSegmentMatches1; //VIDOVIC - probability1
		QList<RECOG::PSGM_::MatchInstance> SSegmentMatches2; //VIDOVIC - probability2
		Array<Array<RECOG::PSGM_::MatchInstance *>> matchMatrix;
		Array<Array<SortIndex<float>>> sortedMatches;
		DWORD scoreCalculation; //VIDOVIC
		ECCVGTLoader *pECCVGT; //Vidovic
		Array <RVL::SegmentGTInstance> segmentGT;

	private:		
		RECOG::PSGM_::Cluster *clusterMem;
		int *clusterSurfelMem;
		int *clusterVertexMem;
		//RECOG::PSGM_::ModelInstanceElement *modelInstanceMem;
		vtkSmartPointer<vtkPolyData> referenceFramesPolyData;
		char *sceneFileName;
		char *modelDataBase; //VIDOVIC
		char *modelsInDataBase; //VIDOVIC
		int nSModelInstances; //VIDOVIC
		int nSamples; //RANSAC //VIDOVIC
		int stdNoise; //RANSAC //VIDOVIC
		bool bNormalValidityTest; // VIDOVIC
		char *sceneMIMatch; //VIDOVIC
		int iScene; //VIDOVIC
		Array<QLIST::Index> centroidID; //VIDOVIC
		int TP;
		int FP;
		int FN;
	};
}

