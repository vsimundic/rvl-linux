#pragma once

//#define RVLPSGM_NORMAL_HULL

namespace RVL
{
	class PSGM;

	namespace RECOG
	{
		namespace PSGM_
		{
			struct Vertex
			{
				float P[3];
				Array<int> iSurfelArray;
				Vertex *pNext;
			};

			struct Cluster
			{
				Array<int> iSurfelArray;
				Array<int> iVertexArray;
				int size;
			};

			struct DisplayData
			{
				PSGM *pRecognition;
				Mesh *pMesh;
				SurfelGraph *pSurfels;
				Visualizer *pVisualizer;
			};

			struct NormalHullElement
			{
				float N[3];
				float Nh[3];
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
		Array<QList<QLIST::Index>> surfelVertexList;
		QLIST::Index *surfelVertexMem;
		RECOG::PSGM_::DisplayData displayData;
		Array<RECOG::PSGM_::Cluster *> clusters;
		RECOG::PSGM_::Cluster *clusterMem;
		int *clusterSurfelMem;
		int *clusterVertexMem;
		int *clusterMap;
		int maxnClusters;
		float kNoise;
	};
}

