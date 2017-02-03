#pragma once

//#define RVLPSGM_NORMAL_HULL

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
			struct NormalHullElement
			{
				float N[3];
				float Nh[3];
			};

			struct Vertex
			{
				float P[3];
				Array<NormalHullElement> normalHull;
				Array<int> iSurfelArray;
				Vertex *pNext;
				bool bEdge;
			};

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
				Array<ModelInstanceElement> modelInstance;
				ModelInstance *pNext;
			};

			struct Cluster
			{
				Array<int> iSurfelArray;
				Array<int> iVertexArray;
				int size;
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
				bool bVertices;
				vtkSmartPointer<vtkActor> vertices;
				vtkSmartPointer<vtkActor> referenceFrames;
				unsigned char selectionColor[3];
				int iSelectedCluster;
				float normalLen;
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
				int iScene;
				int iModel;
				int iMS;
				int iSS;
				float R[9];
				float t[3];
				float score;
				float angle;
				float distance;
				float eSeg;
				MatchInstance *pNext;
			};

			struct FPMatch
			{
				int iScene;
				int iModel;
				float t[3];
				float n;
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
	class CTISet
	{
	public:
		CTISet();
		virtual ~CTISet();

		void LoadSMCTI(char * filePath, Array<RECOG::PSGM_::Plane> *convexTemplate);

		Array<RECOG::PSGM_::ModelInstance> CTI;
		//std::vector<std::vector<int>> SegmentCTIs;
		Array<Array<int>> SegmentCTIs;
		int *segmentCTIIdxMem;
	};

	class PSGM
	{
	public:
		PSGM();
		virtual ~PSGM();
		void Create();
		void CreateParamList(CRVLMem *pMem);
		void Interpret(
			Mesh *pMesh);
		
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
			float *tM, 
			float *dS, 
			int *validS);
		
		Eigen::MatrixXf ConvexTemplatenT();
		//end Petra

		void InitDisplay(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			unsigned char *selectionColor);
		void Display();
		void DisplayModelInstance(Visualizer *pVisualizer);
		void DisplayVertices();
		void DisplayClusters();
		void PaintCluster(
			int iCluster,
			unsigned char *color);
		void PaintClusterVertices(
			int iCluster,
			unsigned char *color);
		void UpdateVertexDisplayLines();
		void DisplayReferenceFrames();
		void SetSceneFileName(char *sceneFileName_);
		void UpdateNormalHull(
			Array<RECOG::PSGM_::NormalHullElement> &NHull,
			float *N);
		bool ModelExistInDB(char *modelFileName, FileSequenceLoader dbLoader); //VIDOVIC
		void SaveModelID(FileSequenceLoader dbLoader); //VIDOVIC
		void Learn(char *modelSequenceFileName); //VIDOVIC
		void LoadModelDataBase(); //VIDOVIC
		void LoadCTI(char * filePath); //VIDOVIC //Moved to CTISet class
		void Match(); //VIDOVIC
		void MSTransformation(RECOG::PSGM_::ModelInstance *pMModelInstance, RECOG::PSGM_::ModelInstance *pSModelInstance, float *tBestMatch, float *R, float *t); //VIDOVIC
		void SetNumberOfScenes(int scenesNumber); //VIDOVIC
		void SaveMatches(); //VIDOVIC
		void CompareMatchesToGT(ECCVGTLoader *ECCVGT, float scoreThresh, float angleThresh, float distanceThresh, float &precision, float &recall); //VIDOVIC
		void PSGM::CompareCTIMatchesToGT(
			ECCVGTLoader *ECCVGT,
			float scoreThresh,
			float angleThresh,
			float distanceThresh,
			float &precision,
			float &recall); //Petra
		void CompareSMIMatchesToGT(ECCVGTLoader *ECCVGT, float scoreThresh, float angleThresh, float distanceThresh, float &precision, float &recall); //VIDOVIC
		bool PSGM::CompareMatchToGT(RECOG::PSGM_::MatchInstance *pMatch, ECCVGTLoader *ECCVGT, bool poseCheck, float angleThresh, float distanceThresh); //VIDOVIC
		void PSGM::CountTPandFN(ECCVGTLoader *ECCVGT, int &TP, int &FN, bool printMatchInfo); //VIDOVIC
		void PSGM::CalculatePR(int TP, int FP, int FN, float &precision, float &recall); //VIDOVIC

	private:
		void DetectVertices(
			Mesh *pMesh);
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
			Array<RECOG::PSGM_::NormalHullElement> &NHull,
			float *N);
		void UpdateMeanNormal(
			float *sumN,
			float &wN,
			float *N,
			float w,
			float *meanN);
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
		QList<RECOG::PSGM_::Vertex> vertexList;
		Array<RECOG::PSGM_::Vertex *> vertexArray;
		Array<QList<QLIST::Index>> surfelVertexList;
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
		Array<RECOG::PSGM_::ModelInstance> modelInstanceDB; //VIDOVIC
		Array<RECOG::PSGM_::ModelInstance> CTI; //VIDOVIC
		Array<RECOG::PSGM_::MatchInstance> matches; //VIDOVIC
		RECOG::PSGM_::MatchInstance *pMatches; //VIDOVIC
		QList<RECOG::PSGM_::MatchInstance> SMImatches; //VIDOVIC
		Eigen::MatrixXf nT; //Petra
		RECOG::PSGM_::SegmentMatch *SMatch; //Petra
		SortIndex<float> *sortedMatches; //Petra
		Eigen::VectorXf E;
		Eigen::MatrixXf t;
		CTISet CTIset;
		CTISet MCTIset;

	private:		
		QLIST::Index *surfelVertexMem;
		RECOG::PSGM_::Cluster *clusterMem;
		int *clusterSurfelMem;
		int *clusterVertexMem;
		//RECOG::PSGM_::ModelInstanceElement *modelInstanceMem;
		Array<Array<int>> vertexDisplayLineArray;
		int *vertexDisplayLineArrayMem;
		vtkSmartPointer<vtkPolyData> linesPolyData;
		vtkSmartPointer<vtkPolyData> referenceFramesPolyData;
		char *sceneFileName;
		int nVertexSurfelRelations;
		char *modelDataBase; //VIDOVIC
		char *modelsInDataBase; //VIDOVIC
		int nSModelInstances; //VIDOVIC
		int nSamples; //RANSAC //VIDOVIC
		int stdNoise; //RANSAC //VIDOVIC
		bool bNormalValidityTest; // VIDOVIC
		char *sceneMIMatch; //VIDOVIC
		int iScene; //VIDOVIC
	};

	

}

