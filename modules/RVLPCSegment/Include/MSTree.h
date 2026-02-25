#pragma once

namespace RVL
{
	namespace GRAPH
	{
		struct UpdateMSTreeInsertData
		{
			int r;
			UpdateMSTreeInsertData *q;
			Pair<int, int> m;
			EdgePtr<Edge> *pEdgePtr;
			uchar state;
		};

		struct MSTreeEdge
		{
			int iVertex[2];
			float cost;
		};

		void MSTreeTest();
	}

	class MSTree
	{
	public:
		MSTree();
		virtual ~MSTree();
		void Init(int memSizeIn = 1000);
		void Clear();
		void Update(float *ENew);
		void Create(
			Array<GRAPH::MSTreeEdge> edges,
			Array<GRAPH::MSTreeEdge> *pSortedTreeEdges = NULL,
			float costThr = 0.0f,
			int *label = NULL);

	public:
		Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> T;
		int memSize;
		float **E;
		GRAPH::UpdateMSTreeInsertData *Q;
		bool *bProcessed;
		Pair<int, int> *T_;
		CRVLMem *pMem;
	};
}

