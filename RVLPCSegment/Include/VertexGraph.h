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

		int ConnectNodesRG(
			int iVertex,
			int iParentVertex,
			SURFEL::VertexEdge *pEdge,
			VertexGraph *pVertexGraph,
			VertexClusterRGData *pData);
	}

	class VertexGraph :
		public Graph < SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge> >
	{
	public:
		VertexGraph();
		virtual ~VertexGraph();
		void Create(SurfelGraph *pSurfels);
		void Clustering();
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
	};
}

