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

#define RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr)	(pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->iVertex[0] : pEdgePtr->pEdge->iVertex[1])

#define RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr)	(pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->iVertex[1] : pEdgePtr->pEdge->iVertex[0])

#define RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr)	(pEdgePtr->pEdge->pVertexEdgePtr[0] == pEdgePtr ? pEdgePtr->pEdge->pVertexEdgePtr[1] : pEdgePtr->pEdge->pVertexEdgePtr[0])

namespace RVL
{
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
}

