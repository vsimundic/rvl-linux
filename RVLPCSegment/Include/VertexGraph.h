#pragma once

//#define RVLVERTEX_GRAPH_EDGES_FROM_SURFELS

namespace RVL
{
	class VertexGraph;

	namespace SURFEL
	{
		struct VertexCluster
		{
			Array<int> iVertexArray;
		};

		struct VertexClusterRGData
		{
			BYTE *nOwners;
		};

		struct VertexConnectRGData
		{
			int iRefVertex;
			float thr1;
			float thr2;
			float *X1;
			float Y1[3];
			float *X2;
			float Y2[3];
			float *Z;
			bool *bVisited;
			Array<int> visitedNodeArray;
			Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> *pVertexGraph2;
			CRVLMem *pMem;
		};

		struct VertexClusterRGData3
		{
			VertexGraph *pVertexGraph;
		};

		int ConnectNodesRG(
			int iVertex,
			int iParentVertex,
			SURFEL::VertexEdge *pEdge,
			VertexGraph *pVertexGraph,
			VertexClusterRGData *pData);

		int ConnectNodesRG2(
			int iVertex,
			int iParentVertex,
			VertexEdge *pEdge,
			VertexGraph *pVertexGraph,
			VertexConnectRGData *pData);

		int ConnectNodesRG3(
			int iVertex,
			int iParentVertex,
			GRAPH::Edge *pEdge,
			Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> *pGraph,
			VertexClusterRGData3 *pData);
	}

	class VertexGraph :
		public Graph < SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge> >
	{
	public:
		VertexGraph();
		virtual ~VertexGraph();
		void Create(SurfelGraph *pSurfels);
		void Clustering();
		void Display(Visualizer *pVisualizer);
		void Save(FILE *fp);
		bool Load(FILE *fp);
		bool BoundingBox(Box<float> *pBox);

	public:
		CRVLMem *pMem;
		SurfelGraph *pSurfels;
		int idx;
		QList<SURFEL::VertexEdge> edgeList;
		int nEdges;
		std::vector<SURFEL::VertexCluster> clusters;
		int *iVertexClusterMem;
		Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> G;
		QList<GRAPH::Edge> GEdgeList;
		int nGEdges;
		vtkSmartPointer<vtkActor> actor;
		vtkSmartPointer<vtkPolyData> polyData;
	};
}
