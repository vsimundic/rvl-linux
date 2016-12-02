//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkPolyLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "ObjectGraph.h"

/// Move to RVLQListArray.h

#define RVLQLIST_APPEND2(pList, pList2)\
{if(pList2->pFirst)\
{\
	*(pList->ppNext) = pList2->pFirst;\
	pList2->pFirst->pPtrToThis = pList->ppNext;\
	pList->ppNext = pList2->ppNext;\
}}

/// Move to Graph.h

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

								if (iMaxCost == 574)
									int debug = 0;
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

	elementMem = NULL;
	NodeArray.Element = NULL;
	EdgeArray.Element = NULL;
	EdgePtrMem = NULL;
	objectMap = NULL;
}


ObjectGraph::~ObjectGraph()
{
	RVL_DELETE_ARRAY(elementMem);
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(EdgePtrMem);
	RVL_DELETE_ARRAY(objectMap);
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
	}

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pAgNode = NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

		pEdgeList = &(pAgNode->EdgeList);

		for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
		{
			pSurfel_ = pSurfel->imgAdjacency.at(i);
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

//Return 'Ntrue', 'Nfalse' and 'N' needed to calculate oversegmentation (Fos = 1 - Ntrue/N) and undersegmenation (Fus =Nfalse/N) error. The asumption is that the GT object hist bin with the highest values is the correct one!!! 
void ObjectGraph::CalculateOverAndUnderSegmentation(int *E, int &N, bool useGTNoPix, bool useBackground)
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
			if ((i == 0) && !useBackground)
				continue;

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

void ObjectGraph::WERSegmentation()
{
#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
	FILE *fp = fopen("C:\\RVL\\Debug\\WERAggGraph.txt", "w");

	WriteSurfelDataToFile(fp);

	fclose(fp);
#endif

	GRAPH::WERAggregation<GRAPH::AggregateNode<AgEdge>, AgEdge, GRAPH::EdgePtr2<AgEdge>, float>(*this, objectMap, elementMem, WERSegmentationMinCostDiff, WERSegmentationCostResolution);
}

void ObjectGraph::ComputeRelationCosts()
{
	int iNode, iNode_;
	GRAPH::AggregateNode<AgEdge> *pAgNode;
	QList<GRAPH::EdgePtr2<AgEdge>> *pEdgeList;
	AgEdge *pEdge;
	GRAPH::EdgePtr2<AgEdge> *pEdgePtr;

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

				ComputeRelationCost(pEdge);
			}

			pEdgePtr = pEdgePtr->pNext;
		}
	}
}

void ObjectGraph::ComputeRelationCost(AgEdge *pEdge)
{
	//float scale = 1000.0f;
	float scale = 1.0f;
	float depthStepIntThr = scale * 0.005f;
	float depthStepExtThr = scale * 0.025f;
	float concaveAngleThr = 45.0f * DEG2RAD;
	float concaveMinCost = 0.3f;
	float alpha = 0.5f;

	float f1 = pEdge->desc.cupyDescriptor[0];
	float f2 = pEdge->desc.cupyDescriptor[1];
	float f3 = pEdge->desc.cupyDescriptor[2];
	float f4 = pEdge->desc.cupyDescriptor[3];

	float PContinuous = (f4 <= depthStepIntThr ? 1.0f : (f4 <= depthStepExtThr ? (depthStepExtThr - f4) / (depthStepExtThr - depthStepIntThr) : 0.0f));

	float PConvex = (f1 >= 0 ? 1.0f : (f1 >= -concaveAngleThr ? concaveMinCost + (1.0f - concaveMinCost) * (concaveAngleThr + f1) / concaveAngleThr : concaveMinCost));

	float PClean = 0.5f + 0.5f * f2;

	pEdge->cost = RVLMIN(PContinuous, RVLMIN(PConvex, PClean));

	pEdge->cost -= alpha;

	pEdge->cost /= (pEdge->cost >= 0 ? 1.0f - alpha : alpha);

	pEdge->cost *= (float)(pEdge->desc.commonBoundaryLength);

	//if (pEdge->cost < -640 * 480 || pEdge->cost > 640 * 480)
	//	int debug = 0;
}

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

		return true;
	}
	else
		return false;
}