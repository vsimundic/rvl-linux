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

			struct Cluster
			{
				Array<int> iSurfelArray;
				Array<int> iVertexArray;
				int size;
				Array<ModelInstanceElement> modelInstance;
			};

			struct Plane
			{
				float N[3];
				float d;
			};

			struct DisplayData
			{
				PSGM *pRecognition;
				Mesh *pMesh;
				SurfelGraph *pSurfels;
				Visualizer *pVisualizer;
			};
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
			Mesh *pMesh);
		void Display();
		void DisplayModelInstance(Visualizer *pVisualizer);
		void DisplayVertices();
		void DisplayClusters();
	private:
		void CreateTemplate();
		bool Inside(
			int iVertex,
			RECOG::PSGM_::Cluster *pCluster,
			int iSurfel = -1);
		bool BelowPlane(
			RECOG::PSGM_::Cluster *pCluster,
			Surfel *pSurfel,
			int iFirstVertex = 0);
		void UpdateNormalHull(
			Array<RECOG::PSGM_::NormalHullElement> &NHull,
			float *N);
		float DistanceFromNormalHull(
			Array<RECOG::PSGM_::NormalHullElement> &NHull,
			float *N);
		void UpdateMeanNormal(
			float *sumN,
			float &wN,
			float *N,
			float w,
			float *meanN);

	public:
		CRVLParameterList ParamList;
		CRVLMem *pMem;
		PlanarSurfelDetector *pSurfelDetector;
		SurfelGraph *pSurfels;
		QList<RECOG::PSGM_::Vertex> vertexList;
		Array<RECOG::PSGM_::Vertex *> vertexArray;
		RECOG::PSGM_::DisplayData displayData;
		Array<RECOG::PSGM_::Cluster *> clusters;
		int *clusterMap;
		int nDominantClusters;
		float kNoise;
		Array<RECOG::PSGM_::Plane> convexTemplate;
	private:
		Array<QList<QLIST::Index>> surfelVertexList;
		QLIST::Index *surfelVertexMem;
		RECOG::PSGM_::Cluster *clusterMem;
		int *clusterSurfelMem;
		int *clusterVertexMem;
		RECOG::PSGM_::ModelInstanceElement *modelInstanceMem;
	};
}

