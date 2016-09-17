#pragma once

// For a given node index iNode and an edge connector pEdgePtr belonging to this node, the function returns the index of the opposite node.
// pEdge_ is the output variable representing the edge corresponding to the connector pEdgePtr.

#define RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge_, iNeighbor)\
{\
	pEdge_ = pEdgePtr->pEdge;\
	iNeighbor = (pEdge_->iVertex[0] == iNode ? pEdge_->iVertex[1] : pEdge_->iVertex[0]);\
}

// Input: edge pEdge, node idx. iNode
// Output: side <- the side of the edge pEdge to which is connected the node iNode 

#define RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdge, iNode) (pEdge->iVertex[0] == iNode ? 0 : 1);

// Input: node idx. iNode, 
//        connector pEdgePtr connecting an edge to the node iNode
// Output: pEdge_ <- the edge connected to the node iNode by the connector pEdgePtr, 
//         side   <- the side of the edge pEdge to which is connected the node iNode,
//         iNeighbor <- Opp(pEdge_, iNode), where Opp is defined in ARP3D.TR3

#define RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iNode, pEdgePtr, pEdge_, iNeighbor, side)\
{\
	pEdge_ = pEdgePtr->pEdge;\
	side = RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdge_, iNode);\
	iNeighbor = pEdge_->iVertex[1 - side];\
}

#define RVLPCSEGMENT_GRAPH_GET_SIDE(pEdgePtr)	(pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? 0 : 1)

#define RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr)	(pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->iVertex[0] : pEdgePtr->pEdge->iVertex[1])

#define RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr)	(pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->iVertex[1] : pEdgePtr->pEdge->iVertex[0])

#define RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr)	(pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->pVertexEdgePtr[1] : pEdgePtr->pEdge->pVertexEdgePtr[0])

#define RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(x, min, lnRes) (x > min ? (int)floor(log((float)(x / min)) / lnRes) : 0)

namespace RVL
{
	namespace GRAPH
	{
		template<typename EdgeType> struct EdgePtr
		{
			EdgeType *pEdge;
			EdgePtr<EdgeType> *pNext;
		};

		struct Edge
		{
			int iNode[2];
			GRAPH::EdgePtr<GRAPH::Edge> *pEdgePtr[2];
			int idx;
		};
	}

	template<typename NodeType, typename EdgeType, typename EdgePtrType>
	class Graph
	{
	public:
		Graph()
		{
			NodeMem = NULL;
			EdgeMem = NULL;
			EdgePtrMem = NULL;
		}
		virtual ~Graph()
		{
			Clear();
		}

		void Clear()
		{
			RVL_DELETE_ARRAY(NodeMem);
			RVL_DELETE_ARRAY(EdgeMem);
			RVL_DELETE_ARRAY(EdgePtrMem);
		}

	public:
		Array<NodeType> NodeArray;
		Array<EdgeType> EdgeArray;
		NodeType *NodeMem;
		EdgeType *EdgeMem;
		EdgePtrType *EdgePtrMem;
	};

	//template<typename GraphType, typename DataType, bool(*f)(int, GraphType *, DataType *)>
	//void RegionGrowing(GraphType* pGraph, DataType *pData, int *piNodeFetch, int *piNodePut)
	//{

	//}

	//template<int>
	//void RegionGrowing(void *vpGraph, void *pData, int *piNodeFetch, int *piNodePut)
	//{

	//}

	template<typename GraphType, typename NodeType, typename EdgeType, typename EdgePtrType, typename DataType, int(*f)(int, int, EdgeType *, GraphType *, DataType *)>
	int * RegionGrowing(GraphType *pGraph, DataType *pData, int *piNodeFetch, int *piNodePut)
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
			}	// for every neighborint node
		}	// region growing loop

		return piNodeFetch;
	}

	template<typename GraphType, typename NodeType, typename EdgeType, typename EdgePtrType, typename DataType, int(*f)(int, int, EdgeType *, GraphType *, DataType *)>
	int * RegionGrowing2(GraphType *pGraph, DataType *pData, int *piNodeFetch, int *piNodePut)
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
				}	// for every neighborint node
			}
		}	// region growing loop

		return piNodeFetch;
	}

	template<typename GraphType, typename NodeType, typename EdgeType, typename EdgePtrType, typename DataType, int(*f)(int, int, EdgeType *, GraphType *, DataType *)>
	int * RegionGrowing3(GraphType *pGraph, DataType *pData, int *piNodeFetch, int *piNodePut, int *&piBoundaryNode)
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
			}	// for every neighborint node

			if (bBoundary || pNode->bBoundary)
				*(piBoundaryNode++) = iNode;
		}	// region growing loop

		return piNodeFetch;
	}

	template<typename NodeType, typename EdgeType, typename EdgePtrType>
	inline EdgeType *ConnectNodes(
		int iNode1,
		int iNode2,
		Array<NodeType> &NodeArray,
		CRVLMem *pMem
		)
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

	namespace GRAPH
	{
		template<typename NodeType, typename EdgeType, typename EdgePtrType, typename CostType>
		void WERSegmentation(
			Graph<typename NodeType, typename EdgeType, typename EdgePtrType> &graph,
			QLIST::Index *elementListMem,
			CostType minCostDiff,
			CostType costResolution)
		{
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

			for (i = 0; i < graph.EdgeArray.n; i++)
				maxPossibleCost += graph.EdgeArray.Element[i].cost;

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

			QLIST::Index2 *pEdgeQueueEntry = edgeQueueMem;

			CostType iMaxCost = 0;

			CostType cost;
			int iCost;
			EdgeType *pEdge;
			int iEdge;

			for (iEdge = 0; iEdge < graph.EdgeArray.n; iEdge++)
			{
				pEdge = graph.EdgeArray.Element + iEdge;

				cost = pEdge->cost;

				if (cost > 0)
				{
					iCost = RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(cost, minCostDiff, lnCostResolution);

					pEdgeList = edgeQueue.Element + iCost;

					RVLQLIST_ADD_ENTRY2(pEdgeList, pEdgeQueueEntry);

					pEdgeQueueEntry->Idx = iEdge;

					if (iCost > iMaxCost)
						iMaxCost = iCost;
				}

				pEdgeQueueEntry++;
			}

			/// main loop

			int *iVisitedNodeEdge = new int[graph.EdgeArray.n];

			memset(iVisitedNodeEdge, 0xff, graph.EdgeArray.n * sizeof(int));			

			int iNode1, iNode2, iNode3, iEdge13, iRefEdge;
			EdgePtrType *pEdgePtr13, *pEdgePtr31, *pEdgePtr21;
			NodeType *pNode1, *pNode2, *pNode3;
			QList<EdgePtrType> *pEdgeList1, *pEdgeList2, *pEdgeList3;
			int side3;
			QLIST::Index2 *pEdge13QueueEntry, *pRefEdgeQueueEntry;
			QList<QLIST::Index2> *pEdgeList_;
			EdgeType *pEdge12, *pEdge13, *pRefEdge;
			QList<QLIST::Index> *pElementList1, *pElementList2;
			bool bNewMaxCost;

			pEdgeList = edgeQueue.Element + iMaxCost;

			pEdgeQueueEntry = pEdgeList->pFirst;

			while (iMaxCost >= 0)
			{
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

				// iNode1 <- union of iNode1 and iNode2 

				RVLQLIST_APPEND(pElementList1, pElementList2);

				// iNode2 <- empty set

				RVLQLIST_INIT(pElementList2);

				// Remove the edge connecting iNode1 and iNode2 from the edgeQueue.

				RVLQLIST_REMOVE_ENTRY2(pEdgeList, pEdgeQueueEntry, QLIST::Index2);

				// Append the edge list of iNode2 to the edge list of iNode1.

				RVLQLIST_APPEND(pEdgeList1, pEdgeList2);

				pEdgePtr21 = pEdgeList2->pFirst;

				while (pEdgePtr21)
				{
					pEdge12 = pEdgePtr21->pEdge;

					if (pEdge12->iVertex[0] == iNode2)
						pEdge12->iVertex[0] == iNode1;
					else if (pEdge12->iVertex[1] == iNode2)
						pEdge12->iVertex[1] == iNode1;

					pEdgePtr21 = pEdgePtr21->pNext;
				}

				// 

				bNewMaxCost = false;

				pEdgePtr13 = pEdgeList1->pFirst;

				while (pEdgePtr13)	// for every edge of iNode1
				{
					// iNode3 <- node connected to iNode1 via edge pEdge13

					pEdge13 = pEdgePtr13->pEdge;

					iEdge13 = pEdge13->idx;

					side3 = 1 - RVLPCSEGMENT_GRAPH_GET_SIDE(pEdgePtr13);

					iNode3 = pEdge13->iVertex[side3];

					if (iNode3 == iNode1)
						RVLQLIST_REMOVE_ENTRY2(pEdgeList1, pEdgePtr13, EdgePtrType);	// Remove pEdge13 from the edge list of iNode1.
					else if (iVisitedNodeEdge[iNode3] >= 0)
					{
						// Remove pEdge13 from the edge list of iNode1. 

						RVLQLIST_REMOVE_ENTRY2(pEdgeList1, pEdgePtr13, EdgePtrType);	

						// Remove pEdge13 from the edge list of iNode3. 

						pEdgePtr31 = pEdge13->pVertexEdgePtr[side3];

						pNode3 = graph.NodeArray.Element + iNode3;

						pEdgeList3 = &(pNode3->EdgeList);

						RVLQLIST_REMOVE_ENTRY2(pEdgeList3, pEdgePtr31);

						// pRefEdge <- the first visited edge which connects iNode1 and iNode3

						iRefEdge = iVisitedNodeEdge[iNode3];

						pRefEdge = graph.EdgeArray.Element + iRefEdge;

						// Remove pEdge13 from edgeQueue.

						if (pEdge13->cost > 0)
						{
							pEdge13QueueEntry = edgeQueueMem + iEdge13;

							pEdgeList_ = edgeQueue.Element + RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(pEdge13->cost, minCostDiff, lnCostResolution);;

							RVLQLIST_REMOVE_ENTRY2(pEdgeList_, pEdge13QueueEntry);
						}

						// Remove pRefEdge from edgeQueue.

						pRefEdgeQueueEntry = edgeQueueMem + iRefEdge;

						if (pRefEdge->cost > 0)
						{
							pEdgeList_ = edgeQueue.Element + RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(pRefEdge->cost, minCostDiff, lnCostResolution);

							RVLQLIST_REMOVE_ENTRY2(pEdgeList_, pRefEdgeQueueEntry);
						}

						// pRefEdge->cost <- pRefEdge->cost + pEdge13->cost

						pRefEdge->cost += pEdge13->cost;

						if (pRefEdge->cost > 0)
						{
							// Add pRefEdge to edgeQueue.

							iCost = RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(pRefEdge->cost, minCostDiff, lnCostResolution);

							pEdgeList_ = edgeQueue.Element + iCost;

							RVLQLIST_ADD_ENTRY2(pEdgeList_, pRefEdgeQueueEntry);

							// Update iMaxCost.

							if (iCost > iMaxCost)
							{
								iMaxCost = iCost;

								pEdgeList = edgeQueue.Element + iMaxCost;

								pEdgeQueueEntry = pEdgeList->pFirst;

								bNewMaxCost = true;
							}
						}
					}
					else
						iVisitedNodeEdge[iNode3] = iEdge;

					pEdgePtr13 = pEdgePtr13->pNext;
				}	// for every edge of iNode1

				if (!bNewMaxCost)
				{
					pEdgeQueueEntry = pEdgeQueueEntry->pNext;

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
				}
			}	// while (iMaxCost >= 0)

			/// 

			delete[] iVisitedNodeEdge;
			delete[] edgeQueue.Element;
			delete[] edgeQueueMem;
		}	// WERSegmentation()
	}	// namespace GRAPH
}	// namespace RVL

