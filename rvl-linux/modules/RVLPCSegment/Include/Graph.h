#pragma once

#define RVLGRAPH_AGGNODE_FLAG_VALID 0x01

// #define RVLSURFEL_IMAGE_ADJACENCY //Vidovic -> exclude Filko functions; In order to get the VolumeNet results reported in ROBIO18, this should be active.
// #define RVLSURFEL_COLOR_HISTOGRAM //Vidovic -> exclude Filko functions

// #define RVLVERSION_171125

#ifdef RVLVERSION_171125
#ifndef RVLSURFEL_IMAGE_ADJACENCY
#define RVLSURFEL_IMAGE_ADJACENCY
#endif
#ifndef RVLSURFEL_COLOR_HISTOGRAM
#define RVLSURFEL_COLOR_HISTOGRAM
#endif
#endif

// #define RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
// #define RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG

// For a given node index iNode and an edge connector pEdgePtr belonging to this node, the function returns the index of the opposite node.
// pEdge_ is the output variable representing the edge corresponding to the connector pEdgePtr.

#define RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge_, iNeighbor)                  \
	{                                                                                        \
		pEdge_ = pEdgePtr->pEdge;                                                            \
		iNeighbor = (pEdge_->iVertex[0] == iNode ? pEdge_->iVertex[1] : pEdge_->iVertex[0]); \
	}

// Input: edge pEdge, node idx. iNode
// Output: side <- the side of the edge pEdge to which is connected the node iNode

#define RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdge, iNode) (pEdge->iVertex[0] == iNode ? 0 : 1)

// Input: node idx. iNode,
//        connector pEdgePtr connecting an edge to the node iNode
// Output: pEdge_ <- the edge connected to the node iNode by the connector pEdgePtr,
//         side   <- the side of the edge pEdge to which is connected the node iNode,
//         iNeighbor <- Opp(pEdge_, iNode), where Opp is defined in ARP3D.TR3

#define RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iNode, pEdgePtr, pEdge_, iNeighbor, side) \
	{                                                                              \
		pEdge_ = pEdgePtr->pEdge;                                                  \
		side = RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdge_, iNode);                    \
		iNeighbor = pEdge_->iVertex[1 - side];                                     \
	}

#define RVLPCSEGMENT_GRAPH_GET_SIDE(pEdgePtr) (pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? 0 : 1)

#define RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr) (pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->iVertex[0] : pEdgePtr->pEdge->iVertex[1])

#define RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr) (pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->iVertex[1] : pEdgePtr->pEdge->iVertex[0])

#define RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr) (pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->pVertexEdgePtr[1] : pEdgePtr->pEdge->pVertexEdgePtr[0])

#define RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(x, min, lnRes) (x > min ? (int)floor(log((float)(x / min)) / lnRes) : 0)

namespace RVL
{
	namespace GRAPH
	{
		template <typename EdgeType>
		struct EdgePtr
		{
			EdgeType *pEdge;
			EdgePtr<EdgeType> *pNext;
		};

		template <typename EdgeType>
		struct EdgePtr2
		{
			EdgeType *pEdge;
			EdgePtr2<EdgeType> *pNext;
			EdgePtr2<EdgeType> **pPtrToThis;
		};

		struct Edge
		{
			int iVertex[2];
			GRAPH::EdgePtr<GRAPH::Edge> *pVertexEdgePtr[2];
			int idx;
			GRAPH::Edge *pNext;
		};

		template <typename CostType>
		struct Edge2
		{
			int iVertex[2];
			GRAPH::EdgePtr<GRAPH::Edge2<CostType>> *pVertexEdgePtr[2];
			int idx;
			CostType cost;
			GRAPH::Edge *pNext;
		};

		struct Node
		{
			int idx;
			QList<EdgePtr<Edge>> EdgeList;
		};

		template <typename EdgePtrType>
		struct Node_
		{
			int idx;
			QList<EdgePtrType> EdgeList;
		};

		struct HierarchyNode
		{
			int iElement;
			HierarchyNode *pParent;
			HierarchyNode *pChild[2];
			HierarchyNode *pNext;
		};

		template <typename EdgeType>
		struct AggregateNode
		{
			QList<QLIST::Index> elementList;
			QList<EdgePtr2<EdgeType>> EdgeList;
			HierarchyNode *pHierarchyNode;
			int size;
			BYTE flags;
		};
	}

	template <typename NodeType, typename EdgeType, typename EdgePtrType>
	class Graph
	{
	public:
		Graph()
		{
            NodeArray.n = EdgeArray.n = 0;
			NodeMem = NULL;
			EdgeMem = NULL;
			EdgePtrMem = NULL;
		}
		virtual ~Graph()
		{
			Clear();
		}

		void Create(
			int nNodes)
		{
			NodeMem = new NodeType[nNodes];
			NodeArray.Element = NodeMem;
			NodeArray.n = nNodes;

			int iNode;
			QList<EdgePtrType> *pEdgeList;

			for (iNode = 0; iNode < nNodes; iNode++)
			{
				pEdgeList = &(NodeArray.Element[iNode].EdgeList);

				RVLQLIST_INIT(pEdgeList);
			}

			EdgePtrMem = new EdgePtrType[2 * EdgeArray.n];

			EdgePtrType *pEdgePtr = EdgePtrMem;

			int iEdge;
			EdgeType *pEdge;
			NodeType *pNode1, *pNode2;
			QList<EdgePtrType> *pEdgeList1, *pEdgeList2;
			EdgePtrType *pEdgePtr_;

			for (iEdge = 0; iEdge < EdgeArray.n; iEdge++)
			{
				pEdge = EdgeArray.Element + iEdge;

				pNode1 = NodeArray.Element + pEdge->iVertex[0];
				pNode2 = NodeArray.Element + pEdge->iVertex[1];

				pEdgeList1 = &(pNode1->EdgeList);
				pEdgeList2 = &(pNode2->EdgeList);

				pEdgePtr_ = pEdgeList1->pFirst;

				while (pEdgePtr_)
				{
					if (pEdgePtr_->pEdge->iVertex[0] == pEdge->iVertex[1])
						break;

					pEdgePtr_ = pEdgePtr_->pNext;
				}

				if (pEdgePtr_)
					continue;

				pEdgePtr->pEdge = pEdge;
				pEdge->pVertexEdgePtr[0] = pEdgePtr;

				RVLQLIST_ADD_ENTRY(pEdgeList1, pEdgePtr);

				pEdgePtr++;

				pEdgePtr->pEdge = pEdge;
				pEdge->pVertexEdgePtr[1] = pEdgePtr;

				RVLQLIST_ADD_ENTRY(pEdgeList2, pEdgePtr);

				pEdgePtr++;
			}
		}

		void Clear()
		{
			RVL_DELETE_ARRAY(NodeMem);
            NodeMem = NULL;
			RVL_DELETE_ARRAY(EdgeMem);
            EdgeMem = NULL;
			RVL_DELETE_ARRAY(EdgePtrMem);
            EdgePtrMem = NULL;
		}

		void OrientTreeEdges(int iRoot)
		{
			bool *bProcessed = new bool[NodeArray.n];

			memset(bProcessed, 0, NodeArray.n * sizeof(bool));

			int *Q = new int[NodeArray.n];

			int *pPut = Q;

			*(pPut++) = iRoot;

			int *pFetch = Q;

			int iNode, iNode_, iTmp;
			NodeType *pNode;
			EdgeType *pEdge;
			EdgePtrType *pEdgePtr, *pEdgePtr_;

			while (pFetch < pPut)
			{
				iNode = *(pFetch++);

				bProcessed[iNode] = true;

				pNode = NodeArray.Element + iNode;

				pEdgePtr = pNode->EdgeList.pFirst;

				while (pEdgePtr)
				{
					iNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

					if (!bProcessed[iNode_])
					{
						pEdge = pEdgePtr->pEdge;

						if (pEdge->iVertex[0] != iNode)
						{
							iTmp = pEdge->iVertex[0];
							pEdge->iVertex[0] = pEdge->iVertex[1];
							pEdge->iVertex[1] = iTmp;

							pEdgePtr_ = pEdge->pVertexEdgePtr[0];
							pEdge->pVertexEdgePtr[0] = pEdge->pVertexEdgePtr[1];
							pEdge->pVertexEdgePtr[1] = pEdgePtr_;
						}

						*(pPut++) = pEdge->iVertex[1];
					}

					pEdgePtr = pEdgePtr->pNext;
				}
			}

			delete[] bProcessed;
			delete[] Q;
		}

	public:
		Array<NodeType> NodeArray;
		Array<EdgeType> EdgeArray;
		NodeType *NodeMem;
		EdgeType *EdgeMem;
		EdgePtrType *EdgePtrMem;
	};

	// template<typename GraphType, typename DataType, bool(*f)(int, GraphType *, DataType *)>
	// void RegionGrowing(GraphType* pGraph, DataType *pData, int *piNodeFetch, int *piNodePut)
	//{

	//}

	// template<int>
	// void RegionGrowing(void *vpGraph, void *pData, int *piNodeFetch, int *piNodePut)
	//{

	//}

	template <typename GraphType, typename NodeType, typename EdgeType, typename EdgePtrType, typename DataType, int (*f)(int, int, EdgeType *, GraphType *, DataType *)>
	int *RegionGrowing(GraphType *pGraph, DataType *pData, int *piNodeFetch, int *piNodePut)
	{
		int iNode, iNode_;
		EdgeType *pEdge;
		EdgePtrType *pEdgePtr;
		NodeType *pNode;

		while (piNodeFetch < piNodePut)
		{
			iNode = *(piNodeFetch++);

			pNode = pGraph->NodeArray.Element + iNode;

			pEdgePtr = pNode->EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);

				if (f(iNode_, iNode, pEdge, pGraph, pData) > 0)
					*(piNodePut++) = iNode_;

				pEdgePtr = pEdgePtr->pNext;
			} // for every neighborint node
        }     // region growing loop

		return piNodeFetch;
	}

	template <typename GraphType, typename NodeType, typename EdgeType, typename EdgePtrType, typename DataType, int (*f)(int, int, EdgeType *, GraphType *, DataType *)>
	int *RegionGrowing2(GraphType *pGraph, DataType *pData, int *piNodeFetch, int *piNodePut)
	{
		int iNode, iNode_;
		EdgeType *pEdge;
		EdgePtrType *pEdgePtr;
		NodeType *pNode;

		while (piNodeFetch < piNodePut)
		{
			iNode = *(piNodeFetch++);

			if (f(iNode, 0, NULL, pGraph, pData) > 0)
			{
				pNode = pGraph->NodeArray.Element + iNode;

				pEdgePtr = pNode->EdgeList.pFirst;

				while (pEdgePtr)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);

					*(piNodePut++) = iNode_;

					pEdgePtr = pEdgePtr->pNext;
				} // for every neighborint node
			}
		} // region growing loop

		return piNodeFetch;
	}

	template <typename GraphType, typename NodeType, typename EdgeType, typename EdgePtrType, typename DataType, int (*f)(int, int, EdgeType *, GraphType *, DataType *)>
	int *RegionGrowing3(GraphType *pGraph, DataType *pData, int *piNodeFetch, int *piNodePut, int *&piBoundaryNode)
	{
		int iNode, iNode_;
		EdgeType *pEdge;
		EdgePtrType *pEdgePtr;
		NodeType *pNode;
		int nodeClass;
		bool bBoundary;

		while (piNodeFetch < piNodePut)
		{
			iNode = *(piNodeFetch++);

			pNode = pGraph->NodeArray.Element + iNode;

			bBoundary = false;

			pEdgePtr = pNode->EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);

				nodeClass = f(iNode_, iNode, pEdge, pGraph, pData);

				if (nodeClass > 0)
					*(piNodePut++) = iNode_;
				else if (nodeClass < 0)
					bBoundary = true;

				pEdgePtr = pEdgePtr->pNext;
			} // for every neighborint node

			if (bBoundary || pNode->bBoundary)
				*(piBoundaryNode++) = iNode;
		} // region growing loop

		return piNodeFetch;
	}

	template <typename NodeType, typename EdgeType, typename EdgePtrType>
	inline EdgeType *ConnectNodes(
		int iNode1,
		int iNode2,
		Array<NodeType> &NodeArray,
		CRVLMem *pMem)
	{
		NodeType *pNode1 = NodeArray.Element + iNode1;
		NodeType *pNode2 = NodeArray.Element + iNode2;

		QList<EdgePtrType> *pEdgeList1 = &(pNode1->EdgeList);
		QList<EdgePtrType> *pEdgeList2 = &(pNode2->EdgeList);

		EdgeType *pEdge;

		RVLMEM_ALLOC_STRUCT(pMem, EdgeType, pEdge);

		pEdge->iVertex[0] = iNode1;
		pEdge->iVertex[1] = iNode2;

		EdgePtrType *pEdgePtr;

		RVLMEM_ALLOC_STRUCT(pMem, EdgePtrType, pEdgePtr);

		pEdgePtr->pEdge = pEdge;
		pEdge->pVertexEdgePtr[0] = pEdgePtr;

		RVLQLIST_ADD_ENTRY(pEdgeList1, pEdgePtr);

		RVLMEM_ALLOC_STRUCT(pMem, EdgePtrType, pEdgePtr);

		pEdgePtr->pEdge = pEdge;
		pEdge->pVertexEdgePtr[1] = pEdgePtr;

		RVLQLIST_ADD_ENTRY(pEdgeList2, pEdgePtr);

		return pEdge;
	}

	template <typename NodeType, typename EdgeType, typename EdgePtrType>
	inline EdgeType *ConnectNodes(
		NodeType *pNode1,
		NodeType *pNode2,
		int iNode1,
		int iNode2,
		CRVLMem *pMem)
	{
		QList<EdgePtrType> *pEdgeList1 = &(pNode1->EdgeList);
		QList<EdgePtrType> *pEdgeList2 = &(pNode2->EdgeList);

		EdgeType *pEdge;

		RVLMEM_ALLOC_STRUCT(pMem, EdgeType, pEdge);

		pEdge->iVertex[0] = iNode1;
		pEdge->iVertex[1] = iNode2;

		EdgePtrType *pEdgePtr;

		RVLMEM_ALLOC_STRUCT(pMem, EdgePtrType, pEdgePtr);

		pEdgePtr->pEdge = pEdge;
		pEdge->pVertexEdgePtr[0] = pEdgePtr;

		RVLQLIST_ADD_ENTRY(pEdgeList1, pEdgePtr);

		RVLMEM_ALLOC_STRUCT(pMem, EdgePtrType, pEdgePtr);

		pEdgePtr->pEdge = pEdge;
		pEdge->pVertexEdgePtr[1] = pEdgePtr;

		RVLQLIST_ADD_ENTRY(pEdgeList2, pEdgePtr);

		return pEdge;
	}

	template <typename NodeType, typename EdgeType, typename EdgePtrType>
	inline void ConnectNodes(
		int iNode1,
		int iNode2,
		Array<NodeType> &NodeArray,
		EdgeType *pEdge,
		EdgePtrType *pEdgePtr)
	{
		NodeType *pNode1 = NodeArray.Element + iNode1;
		NodeType *pNode2 = NodeArray.Element + iNode2;

		QList<EdgePtrType> *pEdgeList1 = &(pNode1->EdgeList);
		QList<EdgePtrType> *pEdgeList2 = &(pNode2->EdgeList);

		pEdge->iVertex[0] = iNode1;
		pEdge->iVertex[1] = iNode2;

		pEdgePtr->pEdge = pEdge;
		pEdge->pVertexEdgePtr[0] = pEdgePtr;

		RVLQLIST_ADD_ENTRY(pEdgeList1, pEdgePtr);

		EdgePtrType *pEdgePtr_ = pEdgePtr + 1;

		pEdgePtr_->pEdge = pEdge;
		pEdge->pVertexEdgePtr[1] = pEdgePtr_;

		RVLQLIST_ADD_ENTRY(pEdgeList2, pEdgePtr_);
	}

    template <typename NodeType, typename EdgeType, typename EdgePtrType>
    inline void ConnectNodes(
        Array<NodeType> &NodeArray,
        EdgeType *pEdge,
        EdgePtrType *pEdgePtr)
    {
        NodeType *pNode1 = NodeArray.Element + pEdge->iVertex[0];
        NodeType *pNode2 = NodeArray.Element + pEdge->iVertex[1];

        QList<EdgePtrType> *pEdgeList1 = &(pNode1->EdgeList);
        QList<EdgePtrType> *pEdgeList2 = &(pNode2->EdgeList);

        pEdgePtr->pEdge = pEdge;
        pEdge->pVertexEdgePtr[0] = pEdgePtr;

        RVLQLIST_ADD_ENTRY(pEdgeList1, pEdgePtr);

        EdgePtrType *pEdgePtr_ = pEdgePtr + 1;

        pEdgePtr_->pEdge = pEdge;
        pEdge->pVertexEdgePtr[1] = pEdgePtr_;

        RVLQLIST_ADD_ENTRY(pEdgeList2, pEdgePtr_);
    }

    template <typename NodeType, typename EdgeType, typename EdgePtrType>
    inline void ConnectNodes2(
        Array<NodeType> &NodeArray,
        EdgeType *pEdge,
        EdgePtrType *pEdgePtr)
    {
        NodeType *pNode1 = NodeArray.Element + pEdge->iVertex[0];
        NodeType *pNode2 = NodeArray.Element + pEdge->iVertex[1];

        QList<EdgePtrType> *pEdgeList1 = &(pNode1->EdgeList);
        QList<EdgePtrType> *pEdgeList2 = &(pNode2->EdgeList);

        pEdgePtr->pEdge = pEdge;
        pEdge->pVertexEdgePtr[0] = pEdgePtr;

        RVLQLIST_ADD_ENTRY2(pEdgeList1, pEdgePtr);

        EdgePtrType *pEdgePtr_ = pEdgePtr + 1;

        pEdgePtr_->pEdge = pEdge;
        pEdge->pVertexEdgePtr[1] = pEdgePtr_;

        RVLQLIST_ADD_ENTRY2(pEdgeList2, pEdgePtr_);
    }

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
	template <typename NodeType, typename EdgeType, typename EdgePtrType>
	void WriteAggNodeData(
		FILE *fp,
		Graph<typename NodeType, typename EdgeType, typename EdgePtrType> &graph,
		int iNode)
	{
		fprintf(fp, "N%d: Elements: ", iNode);

		NodeType *pNode = graph.NodeArray.Element + iNode;

		QList<EdgePtrType> *pEdgeList = &(pNode->EdgeList);

		QList<QLIST::Index> *pElementList = &(pNode->elementList);

		QLIST::Index *piElement = pElementList->pFirst;

		while (piElement)
		{
			fprintf(fp, "%d ", piElement->Idx);

			piElement = piElement->pNext;
		}

		fprintf(fp, "Neighbors: ");

		int iNode_;

		EdgePtrType *pEdgePtr = pEdgeList->pFirst;

		while (pEdgePtr)
		{
			iNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			fprintf(fp, "%d ", iNode_);

			pEdgePtr = pEdgePtr->pNext;
		}

		fprintf(fp, "\n");
	}

	template <typename CostType>
	void WriteWERAggEdgeQueueBin(
		FILE *fp,
		Array<QList<QLIST::Index2>> &edgeQueue,
		int iCost,
		bool bSkipIfEmpty = false)
	{
		QList<QLIST::Index2> *pEdgeList = edgeQueue.Element + iCost;

		QLIST::Index2 *pEdgeIdx = pEdgeList->pFirst;

		if (pEdgeIdx)
		{
			fprintf(fp, "%d:\t", iCost);

			while (pEdgeIdx)
			{
				fprintf(fp, "%d ", pEdgeIdx->Idx);

				pEdgeIdx = pEdgeIdx->pNext;
			}

			fprintf(fp, "\n");
		}
		else if (!bSkipIfEmpty)
			fprintf(fp, "%d:\n", iCost);
	}
#endif

	namespace GRAPH
	{
		template <typename NodeType, typename EdgeType, typename EdgePtrType, typename CostType>
		void WERAggregation(
			Graph<NodeType, EdgeType, EdgePtrType> &graph,
			int *aggregateMap,
			QLIST::Index *elementListMem,
			CostType minCostDiff,
			CostType costResolution)
		{
#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
			FILE *fp = fopen("C:\\RVL\\Debug\\WERAgg.txt", "w");
#endif

			// Initialize elements lists of all nodes.

			QLIST::Index *pElement = elementListMem;

			NodeType *pNode;
			int iNode;
			QList<QLIST::Index> *pElementList;

			for (iNode = 0; iNode < graph.NodeArray.n; iNode++)
			{
				pNode = graph.NodeArray.Element + iNode;

				pElementList = &(pNode->elementList);

				RVLQLIST_INIT(pElementList);

				RVLQLIST_ADD_ENTRY(pElementList, pElement);

				pElement->Idx = iNode;

				pElement++;
			}

			// maxPossibleCost <- the maximum possible cost.

			CostType maxPossibleCost = 0;

			int i;
			CostType cost;

			for (i = 0; i < graph.EdgeArray.n; i++)
			{
				cost = graph.EdgeArray.Element[i].cost;

				if (cost > 0)
					maxPossibleCost += cost;
			}

			// edgeQueue <- edge queue sorted according to their cost.

			float lnCostResolution = log((float)(1 + costResolution));

			Array<QList<QLIST::Index2>> edgeQueue;

			edgeQueue.n = RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(maxPossibleCost, minCostDiff, lnCostResolution) + 1;

			edgeQueue.Element = new QList<QLIST::Index2>[edgeQueue.n];

			QLIST::Index2 *edgeQueueMem = new QLIST::Index2[graph.EdgeArray.n];

			QList<QLIST::Index2> *pEdgeList;

			for (i = 0; i < edgeQueue.n; i++)
			{
				pEdgeList = edgeQueue.Element + i;

				RVLQLIST_INIT(pEdgeList);
			}

			int iMaxCost = 0;

			int iCost;
			EdgeType *pEdge;
			int iEdge;
			QLIST::Index2 *pEdgeQueueEntry;

			for (iEdge = 0; iEdge < graph.EdgeArray.n; iEdge++)
			{
				pEdge = graph.EdgeArray.Element + iEdge;

				pEdgeQueueEntry = edgeQueueMem + iEdge;

				pEdgeQueueEntry->Idx = iEdge;

				cost = pEdge->cost;

				if (cost > 0)
				{
					iCost = RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(cost, minCostDiff, lnCostResolution);

					pEdgeList = edgeQueue.Element + iCost;

					RVLQLIST_ADD_ENTRY2(pEdgeList, pEdgeQueueEntry);

					if (iCost > iMaxCost)
						iMaxCost = iCost;
				}
			}

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
			fprintf(fp, "Sorted edge list:\n\n", iEdge, pEdge->cost);

			for (iCost = iMaxCost; iCost >= 0; iCost--)
				WriteWERAggEdgeQueueBin<CostType>(fp, edgeQueue, iCost, true);

			fprintf(fp, "\n");
#endif

			/// main loop

			int *iVisitedNodeEdge = new int[graph.NodeArray.n];

			memset(iVisitedNodeEdge, 0xff, graph.NodeArray.n * sizeof(int));

			// QLIST::Index2 **ppNextDebug = NULL;

			int iNode1, iNode2, iNode3, iEdge13, iRefEdge, iCost_;
			EdgePtrType *pEdgePtr13, *pEdgePtr31, *pEdgePtr21;
			NodeType *pNode1, *pNode2, *pNode3;
			QList<EdgePtrType> *pEdgeList1, *pEdgeList2, *pEdgeList3;
			int side3;
			QLIST::Index2 *pEdge13QueueEntry, *pRefEdgeQueueEntry;
			QList<QLIST::Index2> *pEdgeList_;
			EdgeType *pEdge12, *pEdge13, *pRefEdge;
			QList<QLIST::Index> *pElementList1, *pElementList2;

			while (iMaxCost >= 0)
			{
				pEdgeList = edgeQueue.Element + iMaxCost;

				pEdgeQueueEntry = pEdgeList->pFirst;

				while (pEdgeQueueEntry == NULL)
				{
					iMaxCost--;

					if (iMaxCost >= 0)
					{
						pEdgeList = edgeQueue.Element + iMaxCost;

						pEdgeQueueEntry = pEdgeList->pFirst;
					}
					else
						break;
				}

				if (iMaxCost < 0)
					break;

				// pEdge <- the first top edge in the edgeQueue.

				iEdge = pEdgeQueueEntry->Idx;

				pEdge = graph.EdgeArray.Element + iEdge;

				// iNode1, iNode2 <- nodes connected by pEdge

				iNode1 = pEdge->iVertex[0];

				pNode1 = graph.NodeArray.Element + iNode1;

				pEdgeList1 = &(pNode1->EdgeList);

				pElementList1 = &(pNode1->elementList);

				iNode2 = pEdge->iVertex[1];

				pNode2 = graph.NodeArray.Element + iNode2;

				pEdgeList2 = &(pNode2->EdgeList);

				pElementList2 = &(pNode2->elementList);

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
				fprintf(fp, "Removing edge %d: cost %f iCost %d\n", iEdge, pEdge->cost, iMaxCost);

				WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode1);

				WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode2);
#endif

				// iNode1 <- union of iNode1 and iNode2

				RVLQLIST_APPEND(pElementList1, pElementList2);

				// iNode2 <- empty set

				RVLQLIST_INIT(pElementList2);

				// Remove the edge connecting iNode1 and iNode2 from the edgeQueue.

				RVLQLIST_REMOVE_ENTRY2(pEdgeList, pEdgeQueueEntry, QLIST::Index2);

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
				fprintf(fp, "Remove edge %d from queue.\n", iEdge);

				WriteWERAggEdgeQueueBin<CostType>(fp, edgeQueue, iMaxCost);
#endif

				// Append the edge list of iNode2 to the edge list of iNode1.

				RVLQLIST_APPEND2(pEdgeList1, pEdgeList2);

				pEdgePtr21 = pEdgeList2->pFirst;

				while (pEdgePtr21)
				{
					pEdge12 = pEdgePtr21->pEdge;

					if (pEdge12->iVertex[0] == iNode2)
						pEdge12->iVertex[0] = iNode1;
					else if (pEdge12->iVertex[1] == iNode2)
						pEdge12->iVertex[1] = iNode1;

					pEdgePtr21 = pEdgePtr21->pNext;
				}

				// Empty the edge list of iNode2.

				RVLQLIST_INIT(pEdgeList2);

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
				WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode1);

				WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode2);
#endif

				//

				pEdgePtr13 = pEdgeList1->pFirst;

				while (pEdgePtr13) // for every edge of iNode1
				{
					// iNode3 <- node connected to iNode1 via edge pEdge13

					pEdge13 = pEdgePtr13->pEdge;

					iEdge13 = pEdge13->idx;

					side3 = 1 - RVLPCSEGMENT_GRAPH_GET_SIDE(pEdgePtr13);

					iNode3 = pEdge13->iVertex[side3];

					if (iNode3 == iNode1)
					{
						RVLQLIST_REMOVE_ENTRY2(pEdgeList1, pEdgePtr13, EdgePtrType); // Remove pEdge13 from the edge list of iNode1.

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
						fprintf(fp, "Removing edge %d(%d-%d) from the edge list of N%d.\n", iEdge13, iNode1, iNode3, iNode1);

						WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode1);
#endif
					}
					else if (iVisitedNodeEdge[iNode3] >= 0)
					{
						// Remove pEdge13 from the edge list of iNode1.

						RVLQLIST_REMOVE_ENTRY2(pEdgeList1, pEdgePtr13, EdgePtrType);

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
						fprintf(fp, "Removing edge %d(%d-%d) from the edge list of N%d.\n", iEdge13, iNode1, iNode3, iNode1);

						WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode1);
#endif

						// Remove pEdge13 from the edge list of iNode3.

						pEdgePtr31 = pEdge13->pVertexEdgePtr[side3];

						pNode3 = graph.NodeArray.Element + iNode3;

						pEdgeList3 = &(pNode3->EdgeList);

						RVLQLIST_REMOVE_ENTRY2(pEdgeList3, pEdgePtr31, EdgePtrType);

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
						fprintf(fp, "Removing edge %d(%d-%d) from the edge list of N%d.\n", iEdge13, iNode1, iNode3, iNode3);

						WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode3);
#endif

						// pRefEdge <- the first visited edge which connects iNode1 and iNode3

						iRefEdge = iVisitedNodeEdge[iNode3];

						pRefEdge = graph.EdgeArray.Element + iRefEdge;

						// Remove pEdge13 from edgeQueue.

						if (pEdge13->cost > 0)
						{
							pEdge13QueueEntry = edgeQueueMem + iEdge13;

							iCost_ = RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(pEdge13->cost, minCostDiff, lnCostResolution);

							// if (iCost_ == 576)
							//	int debug = 0;

							pEdgeList_ = edgeQueue.Element + iCost_;

							//// Debug

							// bool bDebug = false;

							// QLIST::Index2 *pEdgeQueueEntryDebug = pEdgeList_->pFirst;

							// while (pEdgeQueueEntryDebug)
							//{
							//	if (pEdgeQueueEntryDebug == pEdge13QueueEntry)
							//		bDebug = true;

							//	if (pEdgeQueueEntryDebug->pNext == NULL)
							//		if (pEdgeList_->ppNext != &(pEdgeQueueEntryDebug->pNext))
							//			int debug = 0;

							//	pEdgeQueueEntryDebug = pEdgeQueueEntryDebug->pNext;
							//}

							// if (!bDebug)
							//	int debug = 0;

							/////

							RVLQLIST_REMOVE_ENTRY2(pEdgeList_, pEdge13QueueEntry, QLIST::Index2);

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
							fprintf(fp, "Remove edge %d from queue.\n", iEdge13);

							WriteWERAggEdgeQueueBin<CostType>(fp, edgeQueue, iCost_);
#endif
						}

						// Remove pRefEdge from edgeQueue.

						pRefEdgeQueueEntry = edgeQueueMem + iRefEdge;

						if (pRefEdge->cost > 0)
						{
							iCost_ = RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(pRefEdge->cost, minCostDiff, lnCostResolution);

							// if (iCost_ == 576)
							//	int debug = 0;

							pEdgeList_ = edgeQueue.Element + iCost_;

							//// Debug

							// bool bDebug = false;

							// QLIST::Index2 *pEdgeQueueEntryDebug = pEdgeList_->pFirst;

							// while (pEdgeQueueEntryDebug)
							//{
							//	if (pEdgeQueueEntryDebug == pRefEdgeQueueEntry)
							//		bDebug = true;

							//	if (pEdgeQueueEntryDebug->pNext == NULL)
							//		if (pEdgeList_->ppNext != &(pEdgeQueueEntryDebug->pNext))
							//			int debug = 0;

							//	pEdgeQueueEntryDebug = pEdgeQueueEntryDebug->pNext;
							//}

							// if (!bDebug)
							//	int debug = 0;

							/////

							RVLQLIST_REMOVE_ENTRY2(pEdgeList_, pRefEdgeQueueEntry, QLIST::Index2);

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
							fprintf(fp, "Remove edge %d from queue.\n", iRefEdge);

							WriteWERAggEdgeQueueBin<CostType>(fp, edgeQueue, iCost_);
#endif
						}

						// pRefEdge->cost <- pRefEdge->cost + pEdge13->cost

						pRefEdge->cost += pEdge13->cost;

						// if (pEdge13->cost > pRefEdge->cost)		// Region growing method
						//	pRefEdge->cost = pEdge13->cost;

						if (pEdge13->distance < pRefEdge->distance)
							pRefEdge->distance = pEdge13->distance;

						if (pRefEdge->cost > 0)
						{
							// Add pRefEdge to edgeQueue.

							iCost = RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(pRefEdge->cost, minCostDiff, lnCostResolution);

							// if (iCost == 576)
							//	int debug = 0;

							// if (iCost == 576 && iRefEdge == 12438)
							//	int debug = 0;

							// if (iCost == 576 && iRefEdge == 7839)
							//	int debug = 0;

							pEdgeList_ = edgeQueue.Element + iCost;

							RVLQLIST_ADD_ENTRY2(pEdgeList_, pRefEdgeQueueEntry);

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
							if (iCost == 576 && iRefEdge == 7839)
								ppNextDebug = &(pRefEdgeQueueEntry->pNext);

							fprintf(fp, "Add edge %d to queue.\n", iRefEdge);

							WriteWERAggEdgeQueueBin<CostType>(fp, edgeQueue, iCost);
#endif

							// Update iMaxCost.

							if (iCost > iMaxCost)
							{
								iMaxCost = iCost;

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
								fprintf(fp, "new max cost bin index: %d\n", iMaxCost);

								// if (iMaxCost == 574)
								//	int debug = 0;
#endif
							}
						}
					}
					else
						iVisitedNodeEdge[iNode3] = iEdge13;

					// if (ppNextDebug)
					//	if (edgeQueue.Element[576].ppNext != ppNextDebug)
					//		int debug = 0;

					pEdgePtr13 = pEdgePtr13->pNext;
				} // for every edge of iNode1

				pEdgePtr13 = pEdgeList1->pFirst;

				while (pEdgePtr13) // for every edge of iNode1
				{
					iNode3 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr13);

					iVisitedNodeEdge[iNode3] = -1;

					pEdgePtr13 = pEdgePtr13->pNext;
				}

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
				fprintf(fp, "After aggregation:\n", iEdge, pEdge->cost);

				WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode1);

				WriteAggNodeData<NodeType, EdgeType, EdgePtrType>(fp, graph, iNode2);

				fprintf(fp, "\n");

				fflush(fp);
#endif
			} // while (iMaxCost >= 0)

			///

			delete[] iVisitedNodeEdge;
			delete[] edgeQueue.Element;
			delete[] edgeQueueMem;

			// Fill the elementMap.

			memset(aggregateMap, 0xff, graph.NodeArray.n * sizeof(int));

			for (iNode = 0; iNode < graph.NodeArray.n; iNode++)
			{
				pNode = graph.NodeArray.Element + iNode;

				pElementList = &(pNode->elementList);

				pElement = pElementList->pFirst;

				while (pElement)
				{
					aggregateMap[pElement->Idx] = iNode;

					pElement = pElement->pNext;
				}
			}

#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
			fclose(fp);
#endif
		} // WERSegmentation()
    }     // namespace GRAPH
} // namespace RVL
