#pragma once

//#define RVLPSGM_NORMAL_HULL

namespace RVL
{
	class PSGM;

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
			};

			struct ModelInstanceElement
			{
				float d;
				bool defined;
			};

			struct ModelInstance
			{
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
		}
	}

	class PSGM
	{
	public:
		PSGM();
		virtual ~PSGM();
		void CreateParamList(CRVLMem *pMem);
		void Interpret(
			Mesh *pMesh);
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
	private:
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
			int iCluster);

	public:
		CRVLParameterList ParamList;
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
	};
}

