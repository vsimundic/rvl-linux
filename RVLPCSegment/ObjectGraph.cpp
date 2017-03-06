//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkPolyLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"

#include <numeric>
#include <queue>

//#define RVLPCSEGMENT_OBJECT_GRAPH_LOG
//#define RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG

/// Move to RVLQListArray.h

#define RVLQLIST_APPEND2(pList, pList2)\
{if(pList2->pFirst)\
{\
	*(pList->ppNext) = pList2->pFirst;\
	pList2->pFirst->pPtrToThis = pList->ppNext;\
	pList->ppNext = pList2->ppNext;\
}}

/// Move to Graph.h

//#define RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
//#define RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG

namespace RVL
{
	namespace GRAPH
	{
#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
		template<typename NodeType, typename EdgeType, typename EdgePtrType>
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

		template<typename CostType>
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

		template<typename NodeType, typename EdgeType, typename EdgePtrType, typename CostType>
		void WERAggregation(
			Graph<typename NodeType, typename EdgeType, typename EdgePtrType> &graph,
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

			//QLIST::Index2 **ppNextDebug = NULL;

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

				while (pEdgePtr13)	// for every edge of iNode1
				{
					// iNode3 <- node connected to iNode1 via edge pEdge13

					pEdge13 = pEdgePtr13->pEdge;

					iEdge13 = pEdge13->idx;

					side3 = 1 - RVLPCSEGMENT_GRAPH_GET_SIDE(pEdgePtr13);

					iNode3 = pEdge13->iVertex[side3];

					if (iNode3 == iNode1)
					{
						RVLQLIST_REMOVE_ENTRY2(pEdgeList1, pEdgePtr13, EdgePtrType);	// Remove pEdge13 from the edge list of iNode1.

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

							//if (iCost_ == 576)
							//	int debug = 0;

							pEdgeList_ = edgeQueue.Element + iCost_;

							//// Debug

							//bool bDebug = false;

							//QLIST::Index2 *pEdgeQueueEntryDebug = pEdgeList_->pFirst;

							//while (pEdgeQueueEntryDebug)
							//{
							//	if (pEdgeQueueEntryDebug == pEdge13QueueEntry)
							//		bDebug = true;

							//	if (pEdgeQueueEntryDebug->pNext == NULL)
							//		if (pEdgeList_->ppNext != &(pEdgeQueueEntryDebug->pNext))
							//			int debug = 0;

							//	pEdgeQueueEntryDebug = pEdgeQueueEntryDebug->pNext;
							//}

							//if (!bDebug)
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

							//if (iCost_ == 576)
							//	int debug = 0;

							pEdgeList_ = edgeQueue.Element + iCost_;

							//// Debug

							//bool bDebug = false;

							//QLIST::Index2 *pEdgeQueueEntryDebug = pEdgeList_->pFirst;

							//while (pEdgeQueueEntryDebug)
							//{
							//	if (pEdgeQueueEntryDebug == pRefEdgeQueueEntry)
							//		bDebug = true;

							//	if (pEdgeQueueEntryDebug->pNext == NULL)
							//		if (pEdgeList_->ppNext != &(pEdgeQueueEntryDebug->pNext))
							//			int debug = 0;

							//	pEdgeQueueEntryDebug = pEdgeQueueEntryDebug->pNext;
							//}

							//if (!bDebug)
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

						if (pRefEdge->cost > 0)
						{
							// Add pRefEdge to edgeQueue.

							iCost = RVLPCSEGMENT_GRAPH_LOG_BIN_INDEX(pRefEdge->cost, minCostDiff, lnCostResolution);

							//if (iCost == 576)
							//	int debug = 0;

							//if (iCost == 576 && iRefEdge == 12438)
							//	int debug = 0;

							//if (iCost == 576 && iRefEdge == 7839)
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

								//if (iMaxCost == 574)
								//	int debug = 0;
#endif
							}
						}
					}
					else
						iVisitedNodeEdge[iNode3] = iEdge13;

					//if (ppNextDebug)
					//	if (edgeQueue.Element[576].ppNext != ppNextDebug)
					//		int debug = 0;

					pEdgePtr13 = pEdgePtr13->pNext;
				}	// for every edge of iNode1

				pEdgePtr13 = pEdgeList1->pFirst;

				while (pEdgePtr13)	// for every edge of iNode1
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
			}	// while (iMaxCost >= 0)

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
		}	// WERSegmentation()
	}	// namespace GRAPH
}

///

using namespace RVL;
using namespace SURFEL;

ObjectGraph::ObjectGraph()
{
	WERSegmentationMinCostDiff = 0.1f;
	WERSegmentationCostResolution = 0.01f;
	kCoverage = 0.99f;
	alpha = 0.5f;

	elementMem = NULL;
	NodeArray.Element = NULL;
	EdgeArray.Element = NULL;
	EdgePtrMem = NULL;
	objectMap = NULL;
	objectArray.Element = NULL;
	//sortedElementIdxMem = NULL;
	//Array<int> *sortedElementIdxArray = NULL;
	relationClassifier = RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC;
}


ObjectGraph::~ObjectGraph()
{
	RVL_DELETE_ARRAY(elementMem);
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(EdgePtrMem);
	RVL_DELETE_ARRAY(objectMap);
	RVL_DELETE_ARRAY(objectArray.Element);
	//RVL_DELETE_ARRAY(sortedElementIdxMem);
	//RVL_DELETE_ARRAY(sortedElementIdxArray);
}

void ObjectGraph::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("ObjectGraph.alpha", RVLPARAM_TYPE_FLOAT, &alpha);
	pParamData = ParamList.AddParam("ObjectGraph.relationClassifier", RVLPARAM_TYPE_ID, &relationClassifier);
	ParamList.AddID(pParamData, "HEURISTIC", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_HEURISTIC);
	ParamList.AddID(pParamData, "SVM", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM);
	ParamList.AddID(pParamData, "NLMC", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC);
	ParamList.AddID(pParamData, "NLMC2", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC2);
}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
void ObjectGraph::Create(SurfelGraph *pSurfels_)
{
	pSurfels = pSurfels_;

	RVL_DELETE_ARRAY(NodeArray.Element);
	NodeArray.Element = new GRAPH::AggregateNode<AgEdge>[pSurfels->NodeArray.n];
	NodeArray.n = pSurfels->NodeArray.n;
	RVL_DELETE_ARRAY(EdgeArray.Element);
	EdgeArray.Element = new AgEdge[pSurfels->nImageAdjacencyRelations];
	EdgeArray.n = pSurfels->nImageAdjacencyRelations;
	RVL_DELETE_ARRAY(EdgePtrMem);
	EdgePtrMem = new GRAPH::EdgePtr2<AgEdge>[2 * EdgeArray.n];
	RVL_DELETE_ARRAY(elementMem);
	elementMem = new QLIST::Index[pSurfels->NodeArray.n];
	RVL_DELETE_ARRAY(objectMap);
	objectMap = new int[pSurfels->NodeArray.n];	
	//RVL_DELETE_ARRAY(sortedElementIdxMem);
	//sortedElementIdxMem = new int[pSurfels->NodeArray.n];
	//RVL_DELETE_ARRAY(sortedElementIdxArray);
	//sortedElementIdxArray = new Array<int>[pSurfels->NodeArray.n];

	QLIST::Index *piElement = elementMem;

	GRAPH::EdgePtr2<AgEdge> *pEdgePtr = EdgePtrMem;

	AgEdge *pEdge = EdgeArray.Element;

	int i;
	int iSurfel, iSurfel_;
	Surfel *pSurfel, *pSurfel_;
	GRAPH::AggregateNode<AgEdge> *pAgNode, *pAgNode_;
	QList<GRAPH::EdgePtr2<AgEdge>> *pEdgeList, *pEdgeList_;
	SurfelAdjecencyDescriptors *pDesc;
	QList<QLIST::Index> *pElementList;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pAgNode = NodeArray.Element + iSurfel;

		pElementList = &(pAgNode->elementList);

		RVLQLIST_INIT(pElementList);
		RVLQLIST_ADD_ENTRY(pElementList, piElement);
		piElement->Idx = iSurfel;

		piElement++;

		pEdgeList = &(pAgNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		//if (pSurfel->size < 0)
		//	int debug = 0;

		pAgNode->size = pSurfel->size;
	}

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pAgNode = NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

		if (pSurfel->bEdge)
			continue;

		pEdgeList = &(pAgNode->EdgeList);

		for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
		{
			pSurfel_ = pSurfel->imgAdjacency.at(i);

			if (pSurfel_->bEdge)
				continue;

			pDesc = pSurfel->imgAdjacencyDescriptors.at(i);

			iSurfel_ = pSurfel_ - pSurfels->NodeArray.Element;

			if (iSurfel < iSurfel_)
			{
				pEdge->iVertex[0] = iSurfel;
				pEdge->iVertex[1] = iSurfel_;
				pEdge->desc = *pDesc;
				pEdge->cost = 0.0f;
				pEdge->idx = pEdge - EdgeArray.Element;
				pEdgePtr->pEdge = pEdge;
				RVLQLIST_ADD_ENTRY2(pEdgeList, pEdgePtr);
				pEdge->pVertexEdgePtr[0] = pEdgePtr;
				pEdgePtr++;
				pEdgePtr->pEdge = pEdge;
				pAgNode_ = NodeArray.Element + iSurfel_;
				pEdgeList_ = &(pAgNode_->EdgeList);
				RVLQLIST_ADD_ENTRY2(pEdgeList_, pEdgePtr);
				pEdge->pVertexEdgePtr[1] = pEdgePtr;
				pEdgePtr++;
				pEdge++;
			}
		}
	}
}
#endif

//Create (initialize) ObjectGraph object from SFF file
void ObjectGraph::CreateFromSSF(std::string ssfFileName)	
{
	//Loading SSF
	//SSF vars
	this->ssf = std::make_shared<SceneSegFile::SceneSegFile>("");
	this->ssf->Load(ssfFileName);	
	std::shared_ptr<SceneSegFile::SegFileElement> currSSFElement;
	std::shared_ptr<SceneSegFile::FeatureGroup> currSSFAdjacencyFeatureGroup;
	std::shared_ptr<SceneSegFile::FeatureSet> currSSFAdjacencyLink;
	//Get number of surfels and their links
	int noSurfels = this->ssf->elements.size();
	int noLinks = 0;
	//Supplementary vars
	std::map<int, std::vector<int>> adjacencyLinks;
	std::vector<int> *currLink;
	std::vector<int> *otherLink;
	std::map<int, std::vector<int>>::iterator adjacencyLinks_iter;
	std::vector<SurfelAdjecencyDescriptors*> adjacencyDescriptors;
	SurfelAdjecencyDescriptors *pDesc;
	//std::map<int, int> adjacencyLinks2SSFElementsMap;
	for (int i = 0; i < noSurfels; i++)
	{
		currSSFElement = this->ssf->elements.at(i);
		currSSFAdjacencyFeatureGroup = currSSFElement->featureGroups.at(SceneSegFile::FeatureGroupsList::AdjacencyFeatureGroup);
		adjacencyLinks.insert(std::pair<int, std::vector<int>>(currSSFElement->id, std::vector<int>()));
		currLink = &adjacencyLinks.at(currSSFElement->id);
		for (SceneSegFile::featureSets_map_iter_type featureSetsIterator = currSSFAdjacencyFeatureGroup->featureSets.begin(); featureSetsIterator != currSSFAdjacencyFeatureGroup->featureSets.end(); featureSetsIterator++)
		{
			// iterator->first = key
			// iterator->second = value
			currSSFAdjacencyLink = featureSetsIterator->second;
			currLink->push_back(featureSetsIterator->first);
		}
		this->objID2idxMap.insert(std::pair<int, int>(currSSFElement->id, i));
	}
	//Prune links. Remove duplicates and links whose surfels are not in the list
	for (adjacencyLinks_iter = adjacencyLinks.begin(); adjacencyLinks_iter != adjacencyLinks.end(); adjacencyLinks_iter++)
	{
		// iterator->first = key
		// iterator->second = value
		currLink = &adjacencyLinks_iter->second;
		//running through surfel links
		for (int i = 0; i < currLink->size(); i++)
		{
			if (adjacencyLinks.count(currLink->at(i)) > 0)	//if it is on the list
			{
				otherLink = &adjacencyLinks.at(currLink->at(i));	//other surfel's links
				//Find the corresponding surfel and remove it
				for (int k = 0; k < otherLink->size(); k++)
				{
					if (adjacencyLinks_iter->first == otherLink->at(k))
					{
						otherLink->erase(otherLink->begin() + k);
						break;
					}
				}
			}
			else //if it doesn't exist then it was probably in the background
			{
				currLink->erase(currLink->begin() + i);
				i--; //because of size change (current stays current)
			}
		}
		noLinks += currLink->size();	//
		//Adding descriptors
		currSSFElement = this->ssf->elements.at(this->objID2idxMap.at(adjacencyLinks_iter->first));
		currSSFAdjacencyFeatureGroup = currSSFElement->featureGroups.at(SceneSegFile::FeatureGroupsList::AdjacencyFeatureGroup);
		for (int i = 0; i < currLink->size(); i++)
		{
			currSSFAdjacencyLink = currSSFAdjacencyFeatureGroup->featureSets.at(currLink->at(i));
			double *cupyF = (double*)currSSFAdjacencyLink->features.at(SceneSegFile::FeaturesList::CupysFeature)->GetDataPtr();
			pDesc = new SurfelAdjecencyDescriptors;

			pDesc->minDist = cupyF[3];
			pDesc->cupyDescriptor[0] = cupyF[0];
			pDesc->cupyDescriptor[1] = cupyF[1];
			pDesc->cupyDescriptor[2] = cupyF[2];
			pDesc->cupyDescriptor[3] = cupyF[3];
			int *cBL = (int*)currSSFAdjacencyLink->features.at(SceneSegFile::FeaturesList::CommonBoundaryLength)->GetDataPtr();
			pDesc->commonBoundaryLength = *cBL;
			adjacencyDescriptors.push_back(pDesc);	//push descriptor on the list
		}
	}
	
	
	pSurfels = NULL;//pSurfels_;

	RVL_DELETE_ARRAY(NodeArray.Element);
	NodeArray.Element = new GRAPH::AggregateNode<AgEdge>[noSurfels];//[pSurfels->NodeArray.n];
	NodeArray.n = noSurfels;//pSurfels->NodeArray.n;
	RVL_DELETE_ARRAY(EdgeArray.Element);
	EdgeArray.Element = new AgEdge[noLinks];// [pSurfels->nImageAdjacencyRelations];
	EdgeArray.n = noLinks;// pSurfels->nImageAdjacencyRelations;
	RVL_DELETE_ARRAY(EdgePtrMem);
	EdgePtrMem = new GRAPH::EdgePtr2<AgEdge>[2 * EdgeArray.n];
	RVL_DELETE_ARRAY(elementMem);
	elementMem = new QLIST::Index[noSurfels];// [pSurfels->NodeArray.n];
	RVL_DELETE_ARRAY(objectMap);
	objectMap = new int[noSurfels];// [pSurfels->NodeArray.n];

	QLIST::Index *piElement = elementMem;

	GRAPH::EdgePtr2<AgEdge> *pEdgePtr = EdgePtrMem;

	AgEdge *pEdge = EdgeArray.Element;

	int iSurfel;
	//Surfel *pSurfel, *pSurfel_;
	GRAPH::AggregateNode<AgEdge> *pAgNode, *pAgNode_;
	QList<GRAPH::EdgePtr2<AgEdge>> *pEdgeList, *pEdgeList_;
	QList<QLIST::Index> *pElementList;
	int iDesc = 0;
	//List initialization
	for (iSurfel = 0; iSurfel < noSurfels; iSurfel++)
	{
		//pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pAgNode = NodeArray.Element + iSurfel;

		pElementList = &(pAgNode->elementList);

		RVLQLIST_INIT(pElementList);
		RVLQLIST_ADD_ENTRY(pElementList, piElement);
		piElement->Idx = iSurfel;

		piElement++;

		pEdgeList = &(pAgNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		pAgNode->size = ssf->elements.at(iSurfel)->features.features.at(SceneSegFile::FeaturesList::PixelAffiliation)->size;
	}

	//iSurfel = 0;
	//Runnng through surfels
	for (iSurfel = 0, adjacencyLinks_iter = adjacencyLinks.begin(); adjacencyLinks_iter != adjacencyLinks.end(); adjacencyLinks_iter++, iSurfel++)
	{
		pAgNode = NodeArray.Element + iSurfel;

		pEdgeList = &(pAgNode->EdgeList);
		// iterator->first = key
		// iterator->second = value
		currLink = &adjacencyLinks_iter->second;
		//running through surfel links
		for (int i = 0; i < currLink->size(); i++)
		{
			//pSurfel_ = pSurfel->imgAdjacency.at(i);
			pDesc = adjacencyDescriptors.at(iDesc);// pSurfel->imgAdjacencyDescriptors.at(i);

			pEdge->iVertex[0] = this->objID2idxMap.at(adjacencyLinks_iter->first);	//surfel
			pEdge->iVertex[1] = this->objID2idxMap.at(currLink->at(i));	//other surfel
			pEdge->desc = *pDesc;
			pEdge->cost = 0.0f;
			pEdge->idx = pEdge - EdgeArray.Element;
			pEdgePtr->pEdge = pEdge;
			RVLQLIST_ADD_ENTRY2(pEdgeList, pEdgePtr);
			pEdge->pVertexEdgePtr[0] = pEdgePtr;
			pEdgePtr++;
			pEdgePtr->pEdge = pEdge;
			pAgNode_ = NodeArray.Element + this->objID2idxMap.at(currLink->at(i));//+ iSurfel_;
			pEdgeList_ = &(pAgNode_->EdgeList);
			RVLQLIST_ADD_ENTRY2(pEdgeList_, pEdgePtr);
			pEdge->pVertexEdgePtr[1] = pEdgePtr;
			pEdgePtr++;
			pEdge++;

			iDesc++;//aggr list index
		}
	}
}

//Requires that CreateFromSSF be run before this. Return 'Ntrue', 'Nfalse' and 'N' needed to calculate oversegmentation (Fos = 1 - Ntrue/N) and undersegmenation (Fus =Nfalse/N) error. The asumption is that the GT object hist bin with the highest values is the correct one!!! 
void ObjectGraph::CalculateOverAndUnderSegmentation_SSF(int *E, int &N, bool useGTNoPix, bool useBackground)
{
	std::shared_ptr<SceneSegFile::SceneSegFile> ssf = this->ssf;
	std::shared_ptr<SceneSegFile::SegFileElement> currSSFElement;
	std::shared_ptr<SceneSegFile::FeatureTypeInt> GTObjHistogram_surfel;

	//Getting GThist size and initializing GT object histogram;
	currSSFElement = ssf->elements.at(0);
	GTObjHistogram_surfel = std::dynamic_pointer_cast<SceneSegFile::FeatureTypeInt>(currSSFElement->features.features.at(SceneSegFile::FeaturesList::GTObjHistogram));
	int GTHistSize = GTObjHistogram_surfel->size;	
	int *GTObjHistogram = new int[GTHistSize * this->NodeArray.n];	//GTObject histogam per segmented object
	memset(GTObjHistogram, 0, GTHistSize * this->NodeArray.n * sizeof(int));
	int *maxObj = new int[GTHistSize];	//Idx of segmented object per maximum bin
	memset(maxObj, 0, GTHistSize * sizeof(int));
	int *g = new int[GTHistSize];	//gama
	memset(g, 0, GTHistSize * sizeof(int));
	int *maxBin = new int[this->NodeArray.n];	//maximum bin per segmented object
	memset(maxBin, 0, this->NodeArray.n * sizeof(int));

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;

	//Calculating GT object histogram
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		while (piElement)
		{
			//current data
			currSSFElement = ssf->elements.at(piElement->Idx);
			GTObjHistogram_surfel = std::dynamic_pointer_cast<SceneSegFile::FeatureTypeInt>(currSSFElement->features.features.at(SceneSegFile::FeaturesList::GTObjHistogram));
			for (int i = 0; i < GTHistSize; i++)
				GTObjHistogram[iObject * GTHistSize + i] += GTObjHistogram_surfel->data[i];
			piElement = piElement->pNext;
		}
	}
	
	E[0] = 0;	//Oversegmentation values
	E[1] = 0;	//Undersegmentation values
	int* ptrGTObjHist;
	int max = 0;

	int totVal = 0;
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;
		//check if object
		piElement = pObject->elementList.pFirst;
		if (!piElement)
			continue;
		//
		ptrGTObjHist = &(GTObjHistogram[iObject * GTHistSize]);
		
		//find max
		max = 0;
		maxBin[iObject] = -1;
		for (int i = 0; i < GTHistSize; i++)
		{
			if (ptrGTObjHist[i] > max)
			{
				maxBin[iObject] = i;
				max = ptrGTObjHist[i];
			}
		}

		//Sum false values
		for (int i = 0; i < GTHistSize; i++)
		{
			//if ((i == 0) && !useBackground)
			//	continue;

			if (i != maxBin[iObject])
				E[1] += ptrGTObjHist[i];

			totVal += ptrGTObjHist[i];
		}
		
		//Set max segmented object per max bin
		if (ptrGTObjHist[maxBin[iObject]] > g[maxBin[iObject]])
		{
			maxObj[maxBin[iObject]] = iObject;
			g[maxBin[iObject]] = ptrGTObjHist[maxBin[iObject]];
		}
	}

	//Sum positive values
	for (int i = 0; i < GTHistSize; i++)
	{
		if ((i == 0) && !useBackground)
			continue;

		if (i == maxBin[maxObj[i]])
			E[0] += GTObjHistogram[maxObj[i] * GTHistSize + i];
	}


	//Final results
	/*E[0] = 1 - E[0] / totVal;
	E[1] /= totVal;*/
	N = totVal;

	if (useGTNoPix)	//Assumption - GT files is in the same directory as the SSF file and has name in format : SSFfilename + a + .png (label image) and SSFfilename + d + .png (depth image)
	{
		std::string labelImgFileName = this->ssf->filename;
		labelImgFileName.erase(labelImgFileName.find_last_of("."));
		std::string depthImgFileName = labelImgFileName + "d.png";
		labelImgFileName += "a.png";
		//load GT files
		cv::Mat GTLabImg = cv::imread(labelImgFileName);
		cv::Mat GTDepthImg = cv::imread(depthImgFileName, cv::ImreadModes::IMREAD_ANYDEPTH);
		//Count GT object pixels
		N = 0;
		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
				//adding points that have valid label and depth value
				if ((GTLabImg.at<cv::Vec3b>(y, x)[0] > 0) && GTDepthImg.at<unsigned short>(y, x) > 0)
					N++;
			}
		}
	}

	//DeRef
	delete[] GTObjHistogram;
	delete[] maxObj;
	delete[] g;
	delete[] maxBin;
}

//Must be linked with the ground truth (AssignGroundTruthSegmentation per surfel). Return 'Ntrue', 'Nfalse' and 'N' needed to calculate oversegmentation (Fos = 1 - Ntrue/N) and undersegmenation (Fus =Nfalse/N) error. The asumption is that the GT object hist bin with the highest values is the correct one!!! 
void ObjectGraph::CalculateOverAndUnderSegmentation(
	int *E, 
	int &N, 
	bool useGTNoPix, 
	std::string imageFileName, 
	bool useBackground)
{
	//Getting GThist size and initializing GT object histogram;
	//find a surfel that has defined GTObjHist
	int GTHistSize;
	for (int i = 0; i < this->pSurfels->NodeArray.n; i++)
	{
		if (this->pSurfels->NodeArray.Element[i].GTObjHist.size() > 0)
		{
			GTHistSize = this->pSurfels->NodeArray.Element[i].GTObjHist.size();
			break;
		}
	}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	FILE *fp = fopen("C:\\RVL\\ExpRez\\SegmentationToGT.txt", "w");
#endif

	int *GTObjHistogram = new int[GTHistSize * this->NodeArray.n];	//GTObject histogam per segmented object
	memset(GTObjHistogram, 0, GTHistSize * this->NodeArray.n * sizeof(int));
	int *maxObj = new int[GTHistSize];	//Idx of segmented object per maximum bin
	memset(maxObj, 0, GTHistSize * sizeof(int));
	int *g = new int[GTHistSize];	//gama
	memset(g, 0, GTHistSize * sizeof(int));
	int *maxBin = new int[this->NodeArray.n];	//maximum bin per segmented object
	memset(maxBin, 0, this->NodeArray.n * sizeof(int));

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	Surfel *pSurfel;

	//Calculating GT object histogram
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		//check
		if (!piElement)
			continue;

		while (piElement)
		{
			pSurfel = pSurfels->NodeArray.Element + piElement->Idx;
			if (pSurfel->GTObjHist.size() != 0)
			{
				for (int i = 0; i < GTHistSize; i++)
					GTObjHistogram[iObject * GTHistSize + i] += pSurfel->GTObjHist.at(i);
			}
			piElement = piElement->pNext;
		}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
		int nPtsTotal = 0;

		for (int i = 0; i < GTHistSize; i++)
			nPtsTotal += GTObjHistogram[iObject * GTHistSize + i];

		if (nPtsTotal > 0)
		{
			fprintf(fp, "S %d: #P: %d\n", iObject, nPtsTotal);

			fprintf(fp, "-----------------------------\n");

			float fnPtsTotal = (float)nPtsTotal;

			for (int i = 0; i < GTHistSize; i++)
				fprintf(fp, "GTO %d: #IP: %d, perc: %lf\n", i, GTObjHistogram[iObject * GTHistSize + i], (float)GTObjHistogram[iObject * GTHistSize + i] / fnPtsTotal * 100.0f);

			fprintf(fp, "\n");
		}
#endif

	}

	E[0] = 0;	//Oversegmentation values
	E[1] = 0;	//Undersegmentation values
	int* ptrGTObjHist;
	int max = 0;

	int totVal = 0;
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;
		//check if object
		piElement = pObject->elementList.pFirst;
		if (!piElement)
			continue;
		//
		ptrGTObjHist = &(GTObjHistogram[iObject * GTHistSize]);

		//find max
		max = 0;
		maxBin[iObject] = -1;
		for (int i = 0; i < GTHistSize; i++)
		{
			if (ptrGTObjHist[i] > max)
			{
				maxBin[iObject] = i;
				max = ptrGTObjHist[i];
			}
		}

		if (max == 0)//invalid object
			continue;

		//Sum false values
		for (int i = 0; i < GTHistSize; i++)
		{
			//if ((i == 0) && !useBackground)
			//	continue;

			if (i != maxBin[iObject])
				E[1] += ptrGTObjHist[i];

			totVal += ptrGTObjHist[i];
		}

		//Set max segmented object per max bin
		if (ptrGTObjHist[maxBin[iObject]] > g[maxBin[iObject]])
		{
			maxObj[maxBin[iObject]] = iObject;
			g[maxBin[iObject]] = ptrGTObjHist[maxBin[iObject]];
		}
	}

	//Sum positive values
	for (int i = 0; i < GTHistSize; i++)
	{
		if ((i == 0) && !useBackground)
			continue;

		if (i == maxBin[maxObj[i]])
			E[0] += GTObjHistogram[maxObj[i] * GTHistSize + i];
	}


	//Final results
	/*E[0] = 1 - E[0] / totVal;
	E[1] /= totVal;*/
	N = totVal;

	if (useGTNoPix)	//Assumption - GT files is in the same directory as the SSF file and has name in format : SSFfilename + a + .png (label image) and SSFfilename + d + .png (depth image)
	{
		std::string imageName = imageFileName;
		imageName.erase(imageName.find_last_of("."));
		std::string depthImgFileName = imageName + "d.png";
		std::string labelImgFileName = imageName + "a.png";

		//load GT files
		cv::Mat GTLabImg = cv::imread(labelImgFileName);
		cv::Mat GTDepthImg = cv::imread(depthImgFileName, cv::ImreadModes::IMREAD_ANYDEPTH);
		//Count GT object pixels
		N = 0;
		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
				//adding points that have valid label and depth value
				if ((GTLabImg.at<cv::Vec3b>(y, x)[0] > 0) && GTDepthImg.at<unsigned short>(y, x) > 0)
					N++;
			}
		}
	}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	fclose(fp);
#endif

	//DeRef
	delete[] GTObjHistogram;
	delete[] maxObj;
	delete[] g;
	delete[] maxBin;
}

void ObjectGraph::WERSegmentation()
{
#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
	FILE *fp = fopen("C:\\RVL\\Debug\\WERAggGraph.txt", "w");

	WriteSurfelDataToFile(fp);

	fclose(fp);
#endif

	GRAPH::WERAggregation<GRAPH::AggregateNode<AgEdge>, AgEdge, GRAPH::EdgePtr2<AgEdge>, float>(*this, objectMap, elementMem, WERSegmentationMinCostDiff, WERSegmentationCostResolution);

	CreateSortedObjectArray();

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_LOG
	FILE *fpLog = fopen("C:\\RVL\\Debug\\WERAggGraph.txt", "w");

	WriteObjectDataToFile(fpLog);

	fclose(fpLog);
#endif
}

void ObjectGraph::CreateSortedObjectArray()
{
	// Compute object sizes and determine the number of objects.

	int *objectArray_ = new int[NodeArray.n];

	objectArray.n = 0;

	int nPts = 0;

	int iNode;
	GRAPH::AggregateNode<AgEdge> *pAgNode, *pElement;
	QLIST::Index *pElementIdx;
	int size;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		if (iNode == 1230 || iNode == 786 || iNode == 946)
			int debug = 0;

		pAgNode = NodeArray.Element + iNode;

		pElementIdx = pAgNode->elementList.pFirst;

		if (pElementIdx)
		{
			size = 0;

			while (pElementIdx)
			{
				pElement = NodeArray.Element + pElementIdx->Idx;

				size += pElement->size;

				pElementIdx = pElementIdx->pNext;
			}

			pAgNode->size = size;

			if (size > 0)
				nPts += size;

			if (size > 1)
				objectArray_[objectArray.n++] = iNode;
		}
	}

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pAgNode = NodeArray.Element + iNode;

		if (pAgNode->elementList.pFirst == NULL)
			pAgNode->size = 0;
	}

	RVL_DELETE_ARRAY(objectArray.Element);

	objectArray.Element = new int[objectArray.n];

	int nCoverage = 0;

	bool *bCoverage = new bool[objectArray.n];

	memset(bCoverage, 0, objectArray.n * sizeof(bool));

	float fnPts = (float)nPts;

	int iObject = 0;

	int iiNode, iiMaxObject;
	int maxObjectSize;

	while ((float)nCoverage / fnPts < kCoverage)
	{
		maxObjectSize = 0;

		for (iiNode = 0; iiNode < objectArray.n; iiNode++)
		{
			if (bCoverage[iiNode])
				continue;

			iNode = objectArray_[iiNode];

			pAgNode = NodeArray.Element + iNode;

			if (pAgNode->size > maxObjectSize)
			{
				maxObjectSize = pAgNode->size;

				iiMaxObject = iiNode;
			}
		}

		objectArray.Element[iObject++] = objectArray_[iiMaxObject];

		bCoverage[iiMaxObject] = true;

		nCoverage += maxObjectSize;
	}

	objectArray.n = iObject;

	delete[] bCoverage;
	delete[] objectArray_;
}

void ObjectGraph::ComputeRelationCosts()
{
	int iNode, iNode_;
	GRAPH::AggregateNode<AgEdge> *pAgNode;
	QList<GRAPH::EdgePtr2<AgEdge>> *pEdgeList;
	AgEdge *pEdge;
	GRAPH::EdgePtr2<AgEdge> *pEdgePtr;
	ObjectEdgeData edgeData;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pAgNode = NodeArray.Element + iNode;

		pEdgeList = &(pAgNode->EdgeList);

		pEdgePtr = pEdgeList->pFirst;

		while (pEdgePtr)
		{
			iNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			if (iNode < iNode_)
			{
				pEdge = pEdgePtr->pEdge;

				//if (iNode == 15 && iNode_ == 27)
				//	int debug = 0;

				ComputeRelationCost(pEdge, edgeData);
			}

			pEdgePtr = pEdgePtr->pNext;
		}
	}
}

void ObjectGraph::ComputeRelationCost(
	AgEdge *pEdge,
	ObjectEdgeData &data)
{
	//float scale = 1000.0f;
	float scale = 1.0f;
	float depthStepIntThr = scale * 0.005f;
	float depthStepExtThr = scale * 0.025f;
	float concaveAngleThr = 45.0f * DEG2RAD;
	float concaveMinCost = 0.3f;

	float f1 = pEdge->desc.cupyDescriptor[0];
	float f2 = pEdge->desc.cupyDescriptor[1];
	float f3 = pEdge->desc.cupyDescriptor[2];
	float f4 = pEdge->desc.cupyDescriptor[3];

	float y1, y2, y3, y4;

	switch (relationClassifier){
	case RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_HEURISTIC:
		data.PContinuous = (f4 <= depthStepIntThr ? 1.0f : (f4 <= depthStepExtThr ? (depthStepExtThr - f4) / (depthStepExtThr - depthStepIntThr) : 0.0f));

		data.PConvex = (f1 >= 0 ? 1.0f : (f1 >= -concaveAngleThr ? concaveMinCost + (1.0f - concaveMinCost) * (concaveAngleThr + f1) / concaveAngleThr : concaveMinCost));

		data.PClean = 0.5f + 0.5f * f2;

		data.P = RVLMIN(data.PContinuous, RVLMIN(data.PConvex, data.PClean));

		break;
	case RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM:
		//Nyarko - SVM Classification (courtesy of Grbiæ)
		data.PContinuous = -1.0;
		data.PConvex = -1.0;
		data.PClean = -1.0;
		data.P = this->pSVMClassifier->makeClassification(pEdge->desc.cupyDescriptor, 4);

		break;
	case RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC:
		//Nyarko - exponential functions + optimization
		data.PContinuous = -1.0;
		data.PConvex = -1.0;
		data.PClean = -1.0;
		
		y1 = 0.809918368068113 / (0.903903357035594 + exp(-(f1 - (-0.578312575550574)) / 0.369035236083353));
		y2 = 120.173561176014 / (191.419216478501 + exp(-(f2 - 0.512539837485237) / 170.634944320671));
		y3 = 0.840899503394324 / (0.744251572103782 + exp(-(f3 - 0.356242721458920) / 0.321654820477843));
		y4 = 3.28709542710274 / (0.776255228658931 + exp(-(f4 - (-0.0188359236813348)) / (-0.0166765498782912)));

		//Karlo 1
		//data.P = 0.919702757268570*y1 + 0.577863699653274*y2 + 0.0141310682609543*y3 + 0.835443189905499*y4 - 0.910458015938145;

		// Karlo 2
		data.P = 0.844317765926573*y1 + 0.778963337269011*y2 + 0.177692819332776*y3 + 0.582259630958912*y4 - 0.844696779380087;

		break;
	case RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC2:
		data.PContinuous = 3.28709542710274 / (0.776255228658931 + exp(-(f4 - (-0.0188359236813348)) / (-0.0166765498782912)));
		data.PConvex = 0.809918368068113 / (0.903903357035594 + exp(-(f1 - (-0.578312575550574)) / 0.369035236083353));
		data.PClean = 0.840899503394324 / (0.744251572103782 + exp(-(f3 - 0.356242721458920) / 0.321654820477843));

		data.P = RVLMIN(data.PContinuous, data.PConvex);
		//data.P = RVLMIN(data.PContinuous, RVLMIN(data.PConvex, data.PClean));
	}

	pEdge->cost = data.P;

	pEdge->cost -= alpha;

	pEdge->cost /= (pEdge->cost >= 0 ? 1.0f - alpha : alpha);

	pEdge->cost *= (float)(pEdge->desc.commonBoundaryLength);

	//if (pEdge->cost < -640 * 480 || pEdge->cost > 640 * 480)
	//	int debug = 0;
}

void ObjectGraph::SortElements(
	GRAPH::AggregateNode<AgEdge> *pAgNode,
	Array<SortIndex<int>> *pSortedElementIdxArray)
{
	pSortedElementIdxArray->n = 0;

	QLIST::Index *pElementIdx = pAgNode->elementList.pFirst;
	
	Surfel *pElement;
	SortIndex<int> *pSortedElementIdx;

	while (pElementIdx)
	{
		pElement = pSurfels->NodeArray.Element + pElementIdx->Idx;

		pSortedElementIdx = pSortedElementIdxArray->Element + pSortedElementIdxArray->n;

		pSortedElementIdx->cost = pElement->size;
		pSortedElementIdx->idx = pElementIdx->Idx;

		pSortedElementIdxArray->n++;

		pElementIdx = pElementIdx->pNext;
	}

	BubbleSort<SortIndex<int>>(*pSortedElementIdxArray, true);
}

//===== VISUALIZATION =====

void ObjectGraph::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	unsigned char *selectionColor)
{
	pVisualizer->normalLength = 10.0;

	pVisualizer->SetMesh(pMesh);

	displayData.pMesh = pMesh;
	displayData.pSurfels = pSurfels;
	displayData.pObjects = this;
	displayData.pVisualizer = pVisualizer;
	RVLCOPY3VECTOR(selectionColor, displayData.selectionColor);
	displayData.iSelectedObject = -1;

	pSurfels->DisplayData.keyPressUserFunction = &objectKeyPressUserFunction;
	pSurfels->DisplayData.mouseRButtonDownUserFunction = &objectMouseRButtonDownUserFunction;
	pSurfels->DisplayData.vpUserFunctionData = &displayData;

	pSurfels->InitDisplay(pVisualizer, pMesh, NULL);

	displayData.bObjects = true;
}

void ObjectGraph::Display()
{
	int iObject;
	unsigned char color[3];	

	for (iObject = 0; iObject < NodeArray.n; iObject++)
	{
		RandomColor(color);

		PaintObject(iObject, color);
	}
}

void ObjectGraph::PaintObject(
	int iObject,
	unsigned char *color)
{
	GRAPH::AggregateNode<AgEdge> *pObject = NodeArray.Element + iObject;

	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	Surfel *pSurfel;

	QLIST::Index *piElement = pObject->elementList.pFirst;

	while (piElement)
	{
		pSurfel = pSurfels->NodeArray.Element + piElement->Idx;

		if (!pSurfel->bEdge)
			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);

		piElement = piElement->pNext;
	}
}

//===== WRITE TO FILE =====

void ObjectGraph::WriteSurfelDataToFile(FILE *fp)
{
	fprintf(fp, "========== EDGES ==========\n\n");

	int iEdge;
	AgEdge *pEdge;

	for (iEdge = 0; iEdge < EdgeArray.n; iEdge++)
	{
		pEdge = EdgeArray.Element + iEdge;

		fprintf(fp, "E%d(%d-%d): cost=%f\n",
			pEdge->idx,
			pEdge->iVertex[0],
			pEdge->iVertex[1],
			pEdge->cost);
	}
}

void ObjectGraph::WriteObjectDataToFile(FILE *fp)
{

}

void ObjectGraph::Debug()
{
	int maxnGTObjects = 100;
	int minSurfelSize = 20;

	int iNode, iNode_;
	GRAPH::AggregateNode<AgEdge> *pAgNode, *pAgNode_;
	QList<GRAPH::EdgePtr2<AgEdge>> *pEdgeList;
	AgEdge *pEdge;
	GRAPH::EdgePtr2<AgEdge> *pEdgePtr;
	Surfel *pSurfel, *pSurfel_;
	ObjectEdgeData edgeData;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pAgNode = NodeArray.Element + iNode;

		if (pAgNode->size < minSurfelSize)
			continue;

		pSurfel = pSurfels->NodeArray.Element + iNode;

		if (pSurfel->ObjectID < 0 || pSurfel->ObjectID >= maxnGTObjects)
			continue;

		pEdgeList = &(pAgNode->EdgeList);

		pEdgePtr = pEdgeList->pFirst;

		while (pEdgePtr)
		{
			iNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			pAgNode_ = NodeArray.Element + iNode_;

			if (pAgNode_->size >= minSurfelSize)
			{
				if (iNode < iNode_)
				{
					pSurfel_ = pSurfels->NodeArray.Element + iNode_;

					if (pSurfel_->ObjectID >= 0 && pSurfel_->ObjectID < maxnGTObjects)
					{
						if ((pSurfel->ObjectID > 0 || pSurfel_->ObjectID > 0) && pSurfel->ObjectID != pSurfel_->ObjectID)
						{						
							pEdge = pEdgePtr->pEdge;

							ComputeRelationCost(pEdge, edgeData);

							if (edgeData.P > 0.5f)
								int debug = 0;
						}
					}
				}
			}

			pEdgePtr = pEdgePtr->pNext;
		}
	}
}

// Initialize SVM Classifier
void ObjectGraph::InitSVMClassifier(char *svmParamsFileName)
{
	this->pSVMClassifier = new SVMClassifier(svmParamsFileName);
}


//===== GLOBAL FUNCTIONS =====

bool RVL::SURFEL::objectKeyPressUserFunction(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	std::string &key,
	void *vpData)
{
	ObjectDisplayData *pData = (ObjectDisplayData *)vpData;

	ObjectGraph *pObjects = pData->pObjects;
	Visualizer *pVisualizer = pData->pVisualizer;

	if (key == "a")
	{
		if (pData->bObjects)
		{
			pData->bObjects = false;

			pSurfels->Display(pVisualizer, pMesh);

			pData->iSelectedObject = -1;

			return true;
		}
	}
	else if (key == "o")
	{
		if (!pData->bObjects)
		{
			pData->bObjects = true;

			pObjects->Display();

			return true;
		}		
	}

	return false;
}

bool RVL::SURFEL::objectMouseRButtonDownUserFunction(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSelectedPt,
	int iSelectedSurfel,
	void *vpData)
{
	ObjectDisplayData *pData = (ObjectDisplayData *)vpData;

	if (!pData->bObjects)
		return false;

	ObjectGraph *pObjects = pData->pObjects;
	Visualizer *pVisualizer = pData->pVisualizer;

	unsigned char color[3];

	if (pData->iSelectedObject >= 0)
	{
		RandomColor(color);

		pObjects->PaintObject(pData->iSelectedObject, color);
	}

	int iObject = pObjects->objectMap[iSelectedSurfel];

	if (iObject >= 0)
	{
		pObjects->PaintObject(iObject, pData->selectionColor);

		pData->iSelectedObject = iObject;

		printf("Slected object: %d\n", iObject);

		return true;
	}
	else
		return false;
}

void ObjectGraph::DetermineObjectConvexityData(float convexThr, float minDiffFlipReq, bool setflip)
{
	//Reseting convexity data
	if (this->additionalObjectData.CHVertexIndices.size())
		this->additionalObjectData.CHVertexIndices.clear();
	this->additionalObjectData.CHVertexIndices.resize(this->NodeArray.n); //allocate

	if (this->additionalObjectData.ObjectsSurfelConvexity.size())
		this->additionalObjectData.ObjectsSurfelConvexity.clear();
	this->additionalObjectData.ObjectsSurfelConvexity.resize(this->NodeArray.n); //allocate

	if (this->additionalObjectData.convexityMultipliers.size())
		this->additionalObjectData.convexityMultipliers.clear();
	this->additionalObjectData.convexityMultipliers.resize(this->NodeArray.n, 1.0); //allocate
	
	//bool *bVertexInCH = new bool[pSurfels->vertexArray.n];
	//memset(bVertexInCH, 0, pSurfels->vertexArray.n * sizeof(bool));
	
	//running through all objects
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	QList<QLIST::Index> *pSurfelVertexList;
	QList<QLIST::Index> *pSurfelVertexListSurfelIN;
	QLIST::Index *qlistelement;
	SURFEL::Vertex * rvlvertex;
	SURFEL::Vertex * rvlvertexInList;
	Surfel *pSurfel;
	Surfel *pSurfelIN;
	float addedSize = 0;
	bool fail = false;
	Array<SortIndex<int>> sortedElementIdxArray;
	sortedElementIdxArray.Element = new SortIndex < int >[this->pSurfels->NodeArray.n];
	SortIndex<int> *sortedIdx;
	std::set<int> CHVertexIndicesDefDir;
	std::map<int, bool> ObjectsSurfelConvexityDefDir;
	std::set<int> CHVertexIndicesOtherDir;
	std::map<int, bool> ObjectsSurfelConvexityOtherDir;
	float defDirRatio;
	float otherDirRatio;
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;
		
		piElement = pObject->elementList.pFirst;

		//check if object
		if (!piElement)
			continue;
		
		//we are not intrested in objects with size less than 20 points???
		if (pObject->size < 20)
			continue;
		
		// Sort surfels in objects.
		this->SortElements(pObject, &sortedElementIdxArray);

		//Reseting temp vars
		CHVertexIndicesDefDir.clear();
		ObjectsSurfelConvexityDefDir.clear();
		CHVertexIndicesOtherDir.clear();
		ObjectsSurfelConvexityOtherDir.clear();

		//Check the convexity in default direction first (normal)

		//Run through surfels
		addedSize = 0;
		for (int iS = 0; iS < sortedElementIdxArray.n; iS++)
		{
			sortedIdx = sortedElementIdxArray.Element + iS;
			//getting current surfel
			pSurfel = this->pSurfels->NodeArray.Element + sortedIdx->idx;//piElement->Idx;
			//check if edge
			if (pSurfel->bEdge)
				continue;
			//getting current surfel vertex list
			pSurfelVertexList = this->pSurfels->surfelVertexList.Element + sortedIdx->idx; //piElement->Idx;
			
			fail = false;
			//runnong through a current list of added object vertices
			for (CHVertexIndices_iterator_type iterator = CHVertexIndicesDefDir.begin(); iterator != CHVertexIndicesDefDir.end(); iterator++)
			{
				//*iterator = value
				rvlvertexInList = this->pSurfels->vertexArray.Element[*iterator];
				if ((pSurfel->N[0] * rvlvertexInList->P[0] + pSurfel->N[1] * rvlvertexInList->P[1] + pSurfel->N[2] * rvlvertexInList->P[2] - pSurfel->d) > convexThr)
				{
					fail = true;
					break;
				}
			}

			//Check the other direction (if vertex from current surfels is below surfels that were added in CH)
			for (ObjectsSurfelConvexity_iterator_type iterator = ObjectsSurfelConvexityDefDir.begin(); iterator != ObjectsSurfelConvexityDefDir.end(); iterator++)
			{
				//iterator->first = key
				//iterator->second = value
				if (iterator->second)	//if surfel was valid
				{
					//getting added surfel
					pSurfelIN = this->pSurfels->NodeArray.Element + iterator->first;
					//getting current surfel vertex list
					pSurfelVertexListSurfelIN = this->pSurfels->surfelVertexList.Element + sortedIdx->idx;	//CHECK IDX!!!!!!!!sortedIdx->idx!!!!!!!!
					//running through added surfel vertices
					qlistelement = pSurfelVertexListSurfelIN->pFirst;
					while (qlistelement)
					{
						rvlvertex = this->pSurfels->vertexArray.Element[qlistelement->Idx];
						if ((pSurfelIN->N[0] * rvlvertex->P[0] + pSurfelIN->N[1] * rvlvertex->P[1] + pSurfelIN->N[2] * rvlvertex->P[2] - pSurfelIN->d) > convexThr)
						{
							fail = true;
							break;
						}
						//Next
						qlistelement = qlistelement->pNext;
					}
				}

				if (fail)
					break;
			}

			//add fail flag for current surfel
			ObjectsSurfelConvexityDefDir.insert(std::pair<int, bool>(sortedIdx->idx, !fail));
			//if not failed add vertices to list
			if (!fail)
			{
				qlistelement = pSurfelVertexList->pFirst;
				while (qlistelement)
				{
					CHVertexIndicesDefDir.insert(qlistelement->Idx);

					//Next
					qlistelement = qlistelement->pNext;
				}
				//update size
				addedSize += pSurfel->size;
			}
		}

		//ratio
		defDirRatio = addedSize / (float)pObject->size;
		//if ((addedSize / (float)pObject->size) < ratioThr)
		//	this->additionalObjectData.CHVertexIndices.at(iObject).clear(); //if the ratio is lower than threshold, then empty it's list of vertices

		//Check convexity in the other direction (normal)
		//Run through surfels
		addedSize = 0;
		for (int iS = 0; iS < sortedElementIdxArray.n; iS++)
		{
			sortedIdx = sortedElementIdxArray.Element + iS;
			//getting current surfel
			pSurfel = this->pSurfels->NodeArray.Element + sortedIdx->idx;//piElement->Idx;
			//check if edge
			if (pSurfel->bEdge)
				continue;
			//getting current surfel vertex list
			pSurfelVertexList = this->pSurfels->surfelVertexList.Element + sortedIdx->idx; //piElement->Idx;

			fail = false;
			//runnong through a current list of added object vertices
			for (CHVertexIndices_iterator_type iterator = CHVertexIndicesOtherDir.begin(); iterator != CHVertexIndicesOtherDir.end(); iterator++)
			{
				//*iterator = value
				rvlvertexInList = this->pSurfels->vertexArray.Element[*iterator];
				if (((-1)*pSurfel->N[0] * rvlvertexInList->P[0] + (-1)*pSurfel->N[1] * rvlvertexInList->P[1] + (-1)*pSurfel->N[2] * rvlvertexInList->P[2] - (-1)*pSurfel->d) > convexThr)
				{
					fail = true;
					break;
				}
			}

			//Check the other direction (if vertex from current surfels is below surfels that were added in CH)
			for (ObjectsSurfelConvexity_iterator_type iterator = ObjectsSurfelConvexityOtherDir.begin(); iterator != ObjectsSurfelConvexityOtherDir.end(); iterator++)
			{
				//iterator->first = key
				//iterator->second = value
				if (iterator->second)	//if surfel was valid
				{
					//getting added surfel
					pSurfelIN = this->pSurfels->NodeArray.Element + iterator->first;
					//getting current surfel vertex list
					pSurfelVertexListSurfelIN = this->pSurfels->surfelVertexList.Element + sortedIdx->idx;	//CHECK IDX!!!!!!!!sortedIdx->idx!!!!!!!!
					//running through added surfel vertices
					qlistelement = pSurfelVertexListSurfelIN->pFirst;
					while (qlistelement)
					{
						rvlvertex = this->pSurfels->vertexArray.Element[qlistelement->Idx];
						if (((-1)*pSurfelIN->N[0] * rvlvertex->P[0] + (-1)*pSurfelIN->N[1] * rvlvertex->P[1] + (-1)*pSurfelIN->N[2] * rvlvertex->P[2] - (-1)*pSurfelIN->d) > convexThr)
						{
							fail = true;
							break;
						}
						//Next
						qlistelement = qlistelement->pNext;
					}
				}

				if (fail)
					break;
			}

			//add fail flag for current surfel
			ObjectsSurfelConvexityOtherDir.insert(std::pair<int, bool>(sortedIdx->idx, !fail));
			//if not failed add vertices to list
			if (!fail)
			{
				qlistelement = pSurfelVertexList->pFirst;
				while (qlistelement)
				{
					CHVertexIndicesOtherDir.insert(qlistelement->Idx);

					//Next
					qlistelement = qlistelement->pNext;
				}
				//update size
				addedSize += pSurfel->size;
			}
		}

		//ratio
		otherDirRatio = addedSize / (float)pObject->size;

		//Determine which direction to use
		if ((defDirRatio > otherDirRatio) || ((otherDirRatio - defDirRatio) < minDiffFlipReq))
		{
			this->additionalObjectData.CHVertexIndices.at(iObject) = CHVertexIndicesDefDir;
			this->additionalObjectData.ObjectsSurfelConvexity.at(iObject) = ObjectsSurfelConvexityDefDir;
		}
		else
		{
			this->additionalObjectData.CHVertexIndices.at(iObject) = CHVertexIndicesOtherDir;
			this->additionalObjectData.ObjectsSurfelConvexity.at(iObject) = ObjectsSurfelConvexityOtherDir;
			//Set multiplier to -1
			if (setflip)
				this->additionalObjectData.convexityMultipliers.at(iObject) = -1.0;
			//std::cout << "Object " << iObject << " is concave!" << std::endl;
		}
	}	// for every object
	//Deref
	delete[] sortedElementIdxArray.Element;
	//delete[] bVertexInCH;
}

void ObjectGraph::CalculateObjectsColorHistogram()
{
	//Reseting color descriptor data
	if (this->additionalObjectData.colordescriptor.size())
		this->additionalObjectData.colordescriptor.clear();
	this->additionalObjectData.colordescriptor.resize(this->NodeArray.n); //allocate

	//running through all objects
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	Surfel *pSurfel;
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		//check if object
		if (!piElement)
			continue;

		//we are not intrested in objects with size less than 20 points???
		if (pObject->size < 20)
			continue;
		//Find prototype
		while (piElement)
		{
			pSurfel = pSurfels->NodeArray.Element + piElement->Idx;
			//check 
			if (!((pSurfel->size <= 1) || pSurfel->bEdge))
			{
				RVLColorDescriptor newDesc(*pSurfel->colordescriptor); //prototype
				this->additionalObjectData.colordescriptor.at(iObject) = newDesc;
				break;
			}

			piElement = piElement->pNext;
		}
		//running through object's surfels
		piElement = pObject->elementList.pFirst;
		while (piElement)
		{
			pSurfel = pSurfels->NodeArray.Element + piElement->Idx;
			//check 
			if (!((pSurfel->size <= 1) || pSurfel->bEdge))
			{
				this->additionalObjectData.colordescriptor.at(iObject) += *(pSurfel->colordescriptor);
			}

			piElement = piElement->pNext;
		}
	}
}

void ObjectGraph::CalculateConvexityRatiosForObjectPair(int firstObject, int secondObject, float& firstRatio, float& secondRatio, float convexThr)
{
	GRAPH::AggregateNode<SURFEL::AgEdge> *pFirstObject = this->NodeArray.Element + firstObject;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pSecondObject = this->NodeArray.Element + secondObject;

	//Heuristic mumbo-jumbo
	if (((this->additionalObjectData.convexityMultipliers.at(firstObject) == -1) || (this->additionalObjectData.convexityMultipliers.at(secondObject) == -1)) && !CheckIfNeighbours(firstObject, secondObject))
	{
		firstRatio = 0.0;
		secondRatio = 0.0;
		return;
	}
	std::map<int, Surfel*> aggregateObject; //Sorted in ascending order by definition
	std::map<int, Surfel*>::reverse_iterator aggObjIt;	//Reverse iterator (Descending order)
	std::map<int, Surfel*>::reverse_iterator aggObjItSec;
	std::map<int, int> aggregateObjectIdx; //Sorted in ascending order by definition
	std::map<int, int>::reverse_iterator aggObjIdxId;
	//Aggregate object
	QLIST::Index *piElement;
	Surfel *pSurfel;
	Surfel *pSurfelIN;
	//First
	piElement = pFirstObject->elementList.pFirst;
	int firstTotal = 0;
	int keyVal;
	while (piElement)
	{
		pSurfel = pSurfels->NodeArray.Element + piElement->Idx;
		//check 
		if (!((pSurfel->size <= 1) || pSurfel->bEdge))
		{
			//Find key value (if there are two surfels with same size)
			keyVal = pSurfel->size;
			while(aggregateObject.count(keyVal))
			{
				keyVal++;
			}
			aggregateObject.insert(std::pair<int, Surfel*>(keyVal, pSurfel));
			aggregateObjectIdx.insert(std::pair<int, int>(keyVal, firstObject));
			firstTotal += pSurfel->size;
		}

		piElement = piElement->pNext;
	}
	//Second
	piElement = pSecondObject->elementList.pFirst;
	int secondTotal = 0;
	while (piElement)
	{
		pSurfel = pSurfels->NodeArray.Element + piElement->Idx;
		//check 
		if (!((pSurfel->size <= 1) || pSurfel->bEdge))
		{
			//Find key value (if there are two surfels with same size)
			keyVal = pSurfel->size;
			while (aggregateObject.count(keyVal))
			{
				keyVal++;
			}
			aggregateObject.insert(std::pair<int, Surfel*>(keyVal, pSurfel));
			aggregateObjectIdx.insert(std::pair<int, int>(keyVal, secondObject));
			secondTotal += pSurfel->size;
		}

		piElement = piElement->pNext;
	}

	//Running through added surfels in reverse order
	QList<QLIST::Index> *pSurfelVertexList;
	QList<QLIST::Index> *pSurfelVertexListIN;
	SURFEL::Vertex * rvlvertex;
	QLIST::Index *qlistelement;
	int surfelIdx;
	int surfelIdxOther;
	bool fail;
	std::set<int> chVertexIndices;
	std::set<int>::iterator chVertexIndices_iterator;
	bool *added = new bool[aggregateObject.size()];
	memset(added, 0, aggregateObject.size() * sizeof(bool));
	int currIdx = 0;
	int currIdxIN = 0;
	float currmultiplier = 0.0;
	float currmultiplierIN = 0.0;
	for (aggObjIt = aggregateObject.rbegin(); aggObjIt != aggregateObject.rend(); ++aggObjIt)
	{
		//iterator->first = key
		//iterator->second = value

		//getting current surfel
		pSurfel = aggObjIt->second;
		currmultiplier = (this->additionalObjectData.convexityMultipliers.at(aggregateObjectIdx.at(aggObjIt->first)));
		surfelIdx = pSurfel - pSurfels->NodeArray.Element;
		//std::cout << aggObjIt->first << ", " << pSurfel->size << std::endl;
		//getting current surfel vertex list
		pSurfelVertexList = this->pSurfels->surfelVertexList.Element + surfelIdx; //piElement->Idx;

		fail = false;
		//runnong through a current list of added object vertices
		for (chVertexIndices_iterator = chVertexIndices.begin(); chVertexIndices_iterator != chVertexIndices.end(); chVertexIndices_iterator++)
		{
			//*iterator = value
			rvlvertex = this->pSurfels->vertexArray.Element[*chVertexIndices_iterator];
			if ((currmultiplier * pSurfel->N[0] * rvlvertex->P[0] + currmultiplier * pSurfel->N[1] * rvlvertex->P[1] + currmultiplier * pSurfel->N[2] * rvlvertex->P[2] - currmultiplier * pSurfel->d) > convexThr)
			{
				fail = true;
				break;
			}
		}

		//Check the other direction (if vertex from current surfels is below surfels that were added in CH)
		currIdxIN = 0;
		for (aggObjItSec = aggregateObject.rbegin(); aggObjItSec != aggregateObject.rend(); aggObjItSec++)
		{
			//iterator->first = key
			//iterator->second = value
			if (added[currIdxIN])	//if surfel was valid
			{
				//getting added surfel
				pSurfelIN = aggObjItSec->second;
				currmultiplierIN = (this->additionalObjectData.convexityMultipliers.at(aggregateObjectIdx.at(aggObjItSec->first)));
				surfelIdxOther = pSurfelIN - pSurfels->NodeArray.Element;
				//getting current surfel vertex list
				pSurfelVertexListIN = this->pSurfels->surfelVertexList.Element + surfelIdx;
				//running through added surfel vertices
				qlistelement = pSurfelVertexListIN->pFirst;
				while (qlistelement)
				{
					rvlvertex = this->pSurfels->vertexArray.Element[qlistelement->Idx];
					if ((currmultiplierIN * pSurfelIN->N[0] * rvlvertex->P[0] + currmultiplierIN * pSurfelIN->N[1] * rvlvertex->P[1] + currmultiplierIN * pSurfelIN->N[2] * rvlvertex->P[2] - currmultiplierIN * pSurfelIN->d) > convexThr)
					{
						fail = true;
						break;
					}
					//Next
					qlistelement = qlistelement->pNext;
				}
			}

			if (fail)
				break;
			currIdxIN++;
		}
		//if not failed add vertices to list
		if (!fail)
		{
			qlistelement = pSurfelVertexList->pFirst;
			while (qlistelement)
			{
				chVertexIndices.insert(qlistelement->Idx);
				//Next
				qlistelement = qlistelement->pNext;
			}
			added[currIdx] = true;
		}
		//next surfel
		currIdx++;
	}

	//analyze
	currIdxIN = 0;
	int firstAdded = 0;
	int secondAdded = 0;
	for (aggObjIt = aggregateObject.rbegin(), aggObjIdxId = aggregateObjectIdx.rbegin(); aggObjIdxId != aggregateObjectIdx.rend(); aggObjIt++, aggObjIdxId++)
	{
		//iterator->first = key
		//iterator->second = value
		if (added[currIdxIN])	//if surfel was valid
		{
			if (aggObjIdxId->second == firstObject)
				firstAdded += aggObjIt->second->size;
			else
				secondAdded += aggObjIt->second->size;
		}
		currIdxIN++;
	}
	firstRatio = (float)firstAdded / firstTotal;
	secondRatio = (float)secondAdded / secondTotal;

	delete[] added;
}

//Different version of same thing?
//void ObjectGraph::ObjectAggregationLevel2_ViaObjectPairConvexity(float convexThr, float ratioThr, float ratioThr2, int objValidThr, bool verbose)
//{
//	//running through all objects (Generating a list of valid objects)
//	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
//	QLIST::Index *piElement;
//	std::vector<int> validObjects;
//	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
//	{
//		pObject = this->NodeArray.Element + iObject;
//
//		if (pObject->size < objValidThr)
//			continue;
//
//		piElement = pObject->elementList.pFirst;
//
//		//check if object
//		if (!piElement)
//			continue;
//		//
//		validObjects.push_back(iObject);
//	}
//
//	//Helper stuff
//	struct temp_pair{ int a; int b; float score; static bool sort_desc(temp_pair first, temp_pair second) { return (first.score > second.score); } };
//	//
//	//Generating a list of possible merge pairs
//	float firstRatio = 0;
//	float secondRatio = 0;
//	QList<QLIST::Index> *pSurfelVertexList;
//	QLIST::Index *qlistelement;
//	SURFEL::Vertex * rvlvertex;
//	Surfel *pSurfel;
//	double P[3];
//	int ptIdx = 0;
//	std::vector<temp_pair> merge_pairs;
//	std::map<std::string, float> min_convexity_values;
//	std::stringstream ss;
//	float minValue;
//	for (int iObject = 0; iObject < validObjects.size(); iObject++)
//	{
//		for (int iObject2 = iObject + 1; iObject2 < validObjects.size(); iObject2++)
//		{
//			this->CalculateConvexityRatiosForObjectPair(validObjects.at(iObject), validObjects.at(iObject2), firstRatio, secondRatio, convexThr);
//			if (verbose)
//				std::cout << "(" << validObjects.at(iObject) << ", " << validObjects.at(iObject2) << ")" << " = " << firstRatio << ", " << secondRatio << std::endl;
//			//Adding all connections (via min values)
//			if (firstRatio < secondRatio)
//				minValue = firstRatio;
//			else
//				minValue = secondRatio;
//			ss.clear();
//			ss.str("");
//			ss << validObjects.at(iObject) << "_" << validObjects.at(iObject2);	//one way
//			min_convexity_values.insert(std::pair<std::string, float>(ss.str(), minValue));
//			ss.clear();
//			ss.str("");
//			ss << validObjects.at(iObject2) << "_" << validObjects.at(iObject);	//other way
//			min_convexity_values.insert(std::pair<std::string, float>(ss.str(), minValue));
//			if ((firstRatio > ratioThr) && (secondRatio > ratioThr))
//			{
//				merge_pairs.push_back(temp_pair());
//				merge_pairs.at(merge_pairs.size() - 1).a = validObjects.at(iObject);
//				merge_pairs.at(merge_pairs.size() - 1).b = validObjects.at(iObject2);
//				merge_pairs.at(merge_pairs.size() - 1).score = minValue;
//			}
//		}
//	}
//	//sort links
//	std::sort(merge_pairs.begin(), merge_pairs.end(), temp_pair::sort_desc);
//
//	//Generating merge clusters (object pairs is in decreasing order)
//	std::map<int, std::set<int>> merge_clusters;
//	std::map<int, std::set<int>>::iterator clustIt;
//	std::set<int>::iterator clusterSetIt;
//	int foundSet = 0;
//	bool insertFirst;
//	bool keyfound;
//	bool passedcheck;
//	for (int i = 0; i < merge_pairs.size(); i++)
//	{
//		foundSet = -1;
//		insertFirst = false;
//		keyfound = false;
//		passedcheck = true;
//		//if both are keys then continue???
//		if (merge_clusters.count(merge_pairs.at(i).a) && merge_clusters.count(merge_pairs.at(i).b))
//		{
//			continue;
//		}
//		else if (merge_clusters.count(merge_pairs.at(i).a))//check if current pair first item is already defined as cluster leader (KEY)
//		{
//			foundSet = merge_pairs.at(i).a;
//			keyfound = true;
//		}
//		else if (merge_clusters.count(merge_pairs.at(i).b))
//		{
//			foundSet = merge_pairs.at(i).b;
//			keyfound = true;
//			insertFirst = true;
//		}
//		else //check if current pair first (or second???) item is already in some set //CHAINING!!!
//		{
//			for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
//			{
//				if (clustIt->second.count(merge_pairs.at(i).a))
//				{
//					foundSet = clustIt->first;
//					break;
//				}
//				else if (clustIt->second.count(merge_pairs.at(i).b))
//				{
//					foundSet = clustIt->first;
//					insertFirst = true;
//					break;
//				}
//			}
//		}
//		//if found then put the second element in pair in that set
//		if (foundSet >= 0)
//		{
//			//check if the insert element supports connections with other members (according to ratioThr2)
//			if (keyfound)
//			{
//				for (clusterSetIt = merge_clusters.at(foundSet).begin(); clusterSetIt != merge_clusters.at(foundSet).end(); clusterSetIt++)
//				{
//					ss.clear();
//					ss.str("");
//					if (insertFirst)
//						ss << merge_pairs.at(i).a << "_" << *clusterSetIt;
//					else
//						ss << merge_pairs.at(i).b << "_" << *clusterSetIt;
//					if (min_convexity_values.at(ss.str()) < ratioThr2)
//					{
//						passedcheck = false;
//						break;
//					}
//				}
//			}
//			else
//			{
//				//ckeck against the key
//				ss.clear();
//				ss.str("");
//				ss << merge_pairs.at(i).a << "_" << merge_pairs.at(i).b;
//				if (min_convexity_values.at(ss.str()) < ratioThr2)
//					passedcheck = false;
//				else //check against other in set
//				{
//					for (clusterSetIt = merge_clusters.at(foundSet).begin(); clusterSetIt != merge_clusters.at(foundSet).end(); clusterSetIt++)
//					{
//						if (insertFirst && (merge_pairs.at(i).a == *clusterSetIt))
//							continue;
//						else if (!insertFirst && (merge_pairs.at(i).b == *clusterSetIt))
//							continue;
//						ss.clear();
//						ss.str("");
//						if (insertFirst)
//							ss << merge_pairs.at(i).a << "_" << *clusterSetIt;
//						else
//							ss << merge_pairs.at(i).b << "_" << *clusterSetIt;
//						if (min_convexity_values.at(ss.str()) < ratioThr2)
//						{
//							passedcheck = false;
//							break;
//						}
//					}
//				}
//			}
//			//continue
//			if (passedcheck)
//			{
//				if (insertFirst)
//					merge_clusters.at(foundSet).insert(merge_pairs.at(i).a);
//				else
//					merge_clusters.at(foundSet).insert(merge_pairs.at(i).b);
//			}
//		}
//		else //if not found then create new cluster and put second pair element in it (first pair is the KEY of map pair)
//		{
//			merge_clusters.insert(std::pair<int, std::set<int>>(merge_pairs.at(i).a, std::set<int>()));
//			merge_clusters.at(merge_pairs.at(i).a).insert(merge_pairs.at(i).b);
//		}
//	}
//
//	//Running through merge clusters and combining objects
//	GRAPH::AggregateNode<SURFEL::AgEdge> *pNode1;
//	GRAPH::AggregateNode<SURFEL::AgEdge> *pNode2;
//	QList<QLIST::Index> *pElementList1;
//	QList<QLIST::Index> *pElementList2;
//	for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
//	{
//		pNode1 = this->NodeArray.Element + clustIt->first; //main object is the KEY of cluster while other objects are elements of the set
//
//		pElementList1 = &(pNode1->elementList);
//		for (clusterSetIt = clustIt->second.begin(); clusterSetIt != clustIt->second.end(); clusterSetIt++)
//		{
//			pNode2 = this->NodeArray.Element + *clusterSetIt;	//get second object
//
//			pElementList2 = &(pNode2->elementList);
//
//			// iNode1 <- union of iNode1 and iNode2 
//
//			RVLQLIST_APPEND(pElementList1, pElementList2);	//append their surfels
//
//			// iNode2 <- empty set
//
//			RVLQLIST_INIT(pElementList2);	//reset list
//			if (verbose)
//				std::cout << "Merged: " << clustIt->first << ", " << *clusterSetIt << std::endl;
//		}
//
//	}
//}

void ObjectGraph::ObjectAggregationLevel2_ViaObjectPairConvexity(float convexThr, float ratioThr, float ratioThr2, int objValidThr, bool verbose)
{
	//running through all objects (Generating a list of valid objects)
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	std::vector<int> validObjects;
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;

		if (pObject->size < objValidThr)
			continue;

		piElement = pObject->elementList.pFirst;

		//check if object
		if (!piElement)
			continue;
		//
		validObjects.push_back(iObject);
	}

	//Generating a list of possible merge pairs
	float firstRatio = 0;
	float secondRatio = 0;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *qlistelement;
	SURFEL::Vertex * rvlvertex;
	Surfel *pSurfel;
	double P[3];
	int ptIdx = 0;
	std::vector<std::pair<int, int>> merge_pairs;
	std::map<std::string, float> min_convexity_values;
	std::stringstream ss;
	float minValue;
	for (int iObject = 0; iObject < validObjects.size(); iObject++)
	{
		for (int iObject2 = iObject + 1; iObject2 < validObjects.size(); iObject2++)
		{
			this->CalculateConvexityRatiosForObjectPair(validObjects.at(iObject), validObjects.at(iObject2), firstRatio, secondRatio, convexThr);
			if (verbose)
				std::cout << "(" << validObjects.at(iObject) << ", " << validObjects.at(iObject2) << ")" << " = " << firstRatio << ", " << secondRatio << std::endl;
			if ((firstRatio > ratioThr) && (secondRatio > ratioThr))
				merge_pairs.push_back(std::make_pair(validObjects.at(iObject), validObjects.at(iObject2)));
			//Adding all connections (via min values)
			if (firstRatio < secondRatio)
				minValue = firstRatio;
			else
				minValue = secondRatio;
			ss.clear();
			ss.str("");
			ss << validObjects.at(iObject) << "_" << validObjects.at(iObject2);	//one way
			min_convexity_values.insert(std::pair<std::string, float>(ss.str(), minValue));
			ss.clear();
			ss.str("");
			ss << validObjects.at(iObject2) << "_" << validObjects.at(iObject);	//other way
			min_convexity_values.insert(std::pair<std::string, float>(ss.str(), minValue));
		}
	}

	//Generating merge clusters (object pairs is in decreasing order)
	std::map<int, std::set<int>> merge_clusters;
	std::map<int, std::set<int>>::iterator clustIt;
	std::set<int>::iterator clusterSetIt;
	int foundSet = 0;
	bool insertFirst;
	bool intersection;
	int secondSet = 0;
	for (int i = 0; i < merge_pairs.size(); i++)
	{
		foundSet = -1;
		secondSet = -1;
		insertFirst = false;
		intersection = false;
		//check if current pair first item is already defined as cluster leader (KEY)
		if (merge_clusters.count(merge_pairs.at(i).first))
		{
			foundSet = merge_pairs.at(i).first;
			//check for intersection 
			for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
			{
				if (clustIt->second.count(merge_pairs.at(i).second))
				{
					secondSet = clustIt->first;
					intersection = true;
					break;
				}
			}
		}
		else //check if current pair first (or second???) item is already in some set //CHAINING!!!
		{
			for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
			{
				if (clustIt->second.count(merge_pairs.at(i).first))
				{
					foundSet = clustIt->first;
					break;
				}
				else if (clustIt->second.count(merge_pairs.at(i).second))
				{
					foundSet = clustIt->first;
					insertFirst = true;
					break;
				}
			}
		}
		if (intersection) //If intersection then clusters need to be merged
		{
			merge_clusters.at(foundSet).insert(secondSet);
			//merge second cluster into first cluster
			for (clusterSetIt = merge_clusters.at(secondSet).begin(); clusterSetIt != merge_clusters.at(secondSet).end(); clusterSetIt++)
				merge_clusters.at(foundSet).insert(*clusterSetIt);
			//remove the second set
			merge_clusters.erase(secondSet);
		}
		else if (foundSet >= 0) //if found then put the second element in pair in that set
		{

			if (insertFirst)
				merge_clusters.at(foundSet).insert(merge_pairs.at(i).first);
			else
				merge_clusters.at(foundSet).insert(merge_pairs.at(i).second);
		}
		else //if not found then create new cluster and put second pair element in it (first pair is the KEY of map pair)
		{
			merge_clusters.insert(std::pair<int, std::set<int>>(merge_pairs.at(i).first, std::set<int>()));
			merge_clusters.at(merge_pairs.at(i).first).insert(merge_pairs.at(i).second);
		}
	}

	//Checking cluster consistincy if there is are more than two objects in cluster
	//Helper stuff
	struct temp_pair{ int a; int b; float score; static bool sort_desc(temp_pair first, temp_pair second) { return (first.score > second.score); } };
	//
	std::vector<int> inputcluster;
	std::vector<int> inputcluster_label;
	std::queue<int> fifo;
	std::vector<std::vector<temp_pair>> object_links;
	std::vector<std::vector<int>> newclusters;
	std::map<int, std::set<int>> merge_clusters_copy = merge_clusters;
	int label = 0;
	int currObj;
	bool fail;
	int startObj;
	float startObjScore;
	float score;
	for (clustIt = merge_clusters_copy.begin(); clustIt != merge_clusters_copy.end(); clustIt++)
	{
		if (clustIt->second.size() > 1) //total cluster size is clustIt->second.size() + 1 (clustIt->first is cluster leader)
		{
			//remove this cluster from the list (new cluster or clusters will be added)
			merge_clusters.erase(clustIt->first);
			//setup a cluster
			inputcluster.clear();	//reset cluster
			inputcluster.push_back(clustIt->first);
			for (clusterSetIt = clustIt->second.begin(); clusterSetIt != clustIt->second.end(); clusterSetIt++)
				inputcluster.push_back(*clusterSetIt);

			//run through cluster and find the object woth the best connection
			startObj = 0;
			startObjScore = 0.0;
			for (int iObject = 0; iObject < inputcluster.size(); iObject++)
			{
				for (int iObject2 = iObject + 1; iObject2 < inputcluster.size(); iObject2++)
				{
					ss.clear();
					ss.str("");
					ss << inputcluster.at(iObject) << "_" << inputcluster.at(iObject2);
					score = min_convexity_values.at(ss.str());
					if (score > startObjScore)
					{
						startObj = iObject;
						startObjScore = score;
					}

				}
			}

			//swap the best and whoever is the first
			std::swap(inputcluster.front(), *(std::find(inputcluster.begin(), inputcluster.end(), inputcluster.at(startObj))));

			//run through cluster and set object links
			object_links.clear();
			object_links.resize(inputcluster.size());
			for (int iObject = 0; iObject < inputcluster.size(); iObject++)
			{
				for (int iObject2 = iObject + 1; iObject2 < inputcluster.size(); iObject2++)
				{
					ss.clear();
					ss.str("");
					ss << inputcluster.at(iObject) << "_" << inputcluster.at(iObject2);
					
					object_links.at(iObject).push_back(temp_pair());
					object_links.at(iObject).at(object_links.at(iObject).size() - 1).a = iObject;
					object_links.at(iObject).at(object_links.at(iObject).size() - 1).b = iObject2;
					object_links.at(iObject).at(object_links.at(iObject).size() - 1).score = min_convexity_values.at(ss.str());
					object_links.at(iObject2).push_back(temp_pair());
					object_links.at(iObject2).at(object_links.at(iObject2).size() - 1).a = iObject2;
					object_links.at(iObject2).at(object_links.at(iObject2).size() - 1).b = iObject;
					object_links.at(iObject2).at(object_links.at(iObject2).size() - 1).score = min_convexity_values.at(ss.str());
				}
			}
			//sort generated lists
			for (int i = 0; i < object_links.size(); i++)
				std::sort(object_links.at(i).begin(), object_links.at(i).end(), temp_pair::sort_desc);

			//We are going to label objects per new clusters (greedy region growing)
			inputcluster_label.clear();
			inputcluster_label.resize(inputcluster.size(), -1);
			label = 0;
			for (int i = 0; i < inputcluster.size(); i++) //for each object
			{
				if (inputcluster_label.at(i) != -1)	//if it is already labeled, continue
					continue;
				fifo.push(i); //
				inputcluster_label.at(i) = label; //current cluster label
				while (fifo.size())	//while fifo have elements
				{
					currObj = fifo.front(); //current element
					fifo.pop();
					for (int l = 0; l < object_links.at(currObj).size(); l++)	//check links for current object
					{
						if (inputcluster_label.at(object_links.at(currObj).at(l).b) != -1)	//check if the object at the other side of link is labeled
							continue;
						//check the score for the other members of the current (labeled) cluster
						fail = false;
						for (int k = 0; k < inputcluster.size(); k++)
						{
							if (inputcluster_label.at(k) != label) //look for the same label as current cluster
								continue;
							ss.clear();
							ss.str("");
							ss << inputcluster.at(k) << "_" << inputcluster.at(object_links.at(currObj).at(l).b);
							if (min_convexity_values.at(ss.str()) < ratioThr2) //if the score is smaller against any currently labeled member then break
							{
								fail = true;
								break;
							}
						}
						if (fail)//got to the next link
							continue;
						inputcluster_label.at(object_links.at(currObj).at(l).b) = label; //if everything has passed then label the object
						fifo.push(object_links.at(i).at(l).b);	//push it to fifo so it's links can be analyzed
					}
				}
				label++; //current cluster finished, go to next
			}

			//compiling new clusters based on labels
			newclusters.clear();
			newclusters.resize(label);
			for (int i = 0; i < inputcluster_label.size(); i++)
				newclusters.at(inputcluster_label.at(i)).push_back(inputcluster.at(i));

			//Adding new clusters that have more than one member
			for (int i = 0; i < newclusters.size(); i++)
			{
				if (newclusters.at(i).size() < 2)
					continue;
				merge_clusters.insert(std::pair<int, std::set<int>>(newclusters.at(i).at(0), std::set<int>()));
				for (int j = 1; j < newclusters.at(i).size(); j++)
					merge_clusters.at(newclusters.at(i).at(0)).insert(newclusters.at(i).at(j));
			}
		}
	}



	//Running through merge clusters and combining objects
	GRAPH::AggregateNode<SURFEL::AgEdge> *pNode1;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pNode2;
	QList<QLIST::Index> *pElementList1;
	QList<QLIST::Index> *pElementList2;
	for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
	{
		pNode1 = this->NodeArray.Element + clustIt->first; //main object is the KEY of cluster while other objects are elements of the set

		pElementList1 = &(pNode1->elementList);
		for (clusterSetIt = clustIt->second.begin(); clusterSetIt != clustIt->second.end(); clusterSetIt++)
		{
			pNode2 = this->NodeArray.Element + *clusterSetIt;	//get second object

			pElementList2 = &(pNode2->elementList);

			// iNode1 <- union of iNode1 and iNode2 

			RVLQLIST_APPEND(pElementList1, pElementList2);	//append their surfels

			// iNode2 <- empty set

			RVLQLIST_INIT(pElementList2);	//reset list
			if (verbose)
				std::cout << "Merged: " << clustIt->first << ", " << *clusterSetIt << std::endl;
		}

	}
}

cv::Mat ObjectGraph::CreateSegmentationImage()
{
	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	Surfel *pSurfel;
	unsigned char labSegColor[3];
	int x = 0, y = 0;
	RVL::QLIST::Index2 *pt;
	srand(time(NULL));
	for (int iObject = 0; iObject < NodeArray.n; iObject++)
	{
		//Generate surfel color
		labSegColor[0] = rand() % 255;
		labSegColor[1] = rand() % 255;
		labSegColor[2] = rand() % 255;

		pObject = NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		while (piElement)
		{
			pSurfel = pSurfels->NodeArray.Element + piElement->Idx;
			//check 
			if (!((pSurfel->size <= 1) || pSurfel->bEdge))
			{
				pt = pSurfel->PtList.pFirst;
				//Set pixel colors
				for (int k = 0; k < pSurfel->size; k++)
				{
					y = floor(pt->Idx / 640.0);
					x = floor(pt->Idx - 640.0 * y);
					coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
					coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
					coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
					pt = pt->pNext;
				}
			}
			piElement = piElement->pNext;
		}

	}
	//return image
	return coloredSegLab;
}

cv::Mat ObjectGraph::CreateSegmentationImageFromSSF()
{
	std::shared_ptr<SceneSegFile::SegFileElement> currSSFElement;
	std::shared_ptr<SceneSegFile::FeatureTypeInt> pixAff;

	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	unsigned char labSegColor[3];
	int x = 0, y = 0;
	RVL::QLIST::Index2 *pt;
	srand(time(NULL));

	for (int iObject = 0; iObject < NodeArray.n; iObject++)
	{
		//Generate object color
		labSegColor[0] = rand() % 255;
		labSegColor[1] = rand() % 255;
		labSegColor[2] = rand() % 255;

		pObject = NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		while (piElement)
		{
			currSSFElement = ssf->elements.at(piElement->Idx);

			pixAff = std::dynamic_pointer_cast<SceneSegFile::FeatureTypeInt>(currSSFElement->features.features.at(SceneSegFile::FeaturesList::PixelAffiliation));

			for (int k = 0; k < pixAff->size; k++)
			{
				y = floor(pixAff->data[k] / 640.0);
				x = floor(pixAff->data[k] - 640.0 * y);
				coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
				coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
				coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
			}

			piElement = piElement->pNext;
		}

	}
	//return image
	return coloredSegLab;
}

void ObjectGraph::SaveSegmentationLabelImg(std::string filename)
{
	cv::Mat labelImg(480, 640, CV_8UC1, cv::Scalar::all(0));

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	Surfel *pSurfel;
	int x = 0, y = 0;
	RVL::QLIST::Index2 *pt;
	unsigned char objLabel = 1;
	bool increment = false;
	//For all objects
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{

		pObject = this->NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		//check if object
		if (!piElement)
			continue;
		//

		//for all object's surfels
		increment = false;
		while (piElement)
		{
			pSurfel = this->pSurfels->NodeArray.Element + piElement->Idx;
			//check 
			if (!((pSurfel->size <= 1) || pSurfel->bEdge))
			{
				pt = pSurfel->PtList.pFirst;
				//for all surfel's points
				for (int k = 0; k < pSurfel->size; k++)
				{
					y = floor(pt->Idx / 640.0);
					x = floor(pt->Idx - 640.0 * y);
					labelImg.at<unsigned char>(y, x) = objLabel;
					pt = pt->pNext;
				}
				increment = true; //Just in cease all elements of the object are edges
			}
			piElement = piElement->pNext;
		}
		//increment object label
		if (increment)
			objLabel++; //What if the number of objects is above 255???
	}

	//Save image (preferable as png)
	cv::imwrite(filename, labelImg);
}

bool ObjectGraph::CheckObjectUniformity(int objectIdx, int minSurfelSize, float uniThr)
{
	bool uni = false;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject = this->NodeArray.Element + objectIdx;
	QLIST::Index *piElement;
	Surfel *pSurfel;
	//Determine average surfel size for surfels above required size
	float sumSize = 0;
	int noSurfels = 0;
	piElement = pObject->elementList.pFirst;
	while (piElement)
	{
		pSurfel = this->pSurfels->NodeArray.Element + piElement->Idx;
		//check 
		if (!((pSurfel->size < minSurfelSize) || pSurfel->bEdge))
		{
			sumSize += pSurfel->size;
			noSurfels++;
		}
		piElement = piElement->pNext;
	}
	float avgSize = sumSize / noSurfels;

	//Determine the sum of differences compared to avg size
	float uniSum = 0;
	piElement = pObject->elementList.pFirst;
	while (piElement)
	{
		pSurfel = this->pSurfels->NodeArray.Element + piElement->Idx;
		//check 
		if (!((pSurfel->size < minSurfelSize) || pSurfel->bEdge))
			uniSum += abs(avgSize - pSurfel->size);
		piElement = piElement->pNext;
	}
	std::cout << "Object " << objectIdx << " uniformity: " << uniSum / sumSize << std::endl;
	return uni;
}

bool ObjectGraph::CheckIfNeighbours(int iObject1, int iObject2)
{
	//Get pointers to object
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject1 = this->NodeArray.Element + iObject1;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject2 = this->NodeArray.Element + iObject2; //It is not needed
	GRAPH::EdgePtr2<SURFEL::AgEdge> *edgeElement;
	edgeElement = pObject1->EdgeList.pFirst;
	//Running through object edges
	while (edgeElement)
	{
		if (RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(edgeElement) == iObject2) //If the index correspondes to other object then they are neighbours
			return true;
		edgeElement = edgeElement->pNext;
	}
	return false; //if the function has not finished earlier then they are not neighbours
}
