#pragma once
#include "Graph.h"
#include <memory>
#include "SceneSegFile.hpp"
#include "SVMClassifier.h"
#include <set>

#define RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_HEURISTIC		0
#define RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM				1
#define RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC			2
#define RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC2			3
#define RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_FUZZY_HEURISTIC	4

#define RVLPCSEGMENT_OBJECT_FLAG_IN_VOI		0x10
#define RVLPCSEGMENT_OBJECT_FLAG_GND		0x20
#define RVLPCSEGMENT_OBJECT_FLAG_CONCAVE	0x40
#define RVLPCSEGMENT_OBJECT_FLAG_VALID		0x01 //Vidovic

#define RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_CONVEXITY		0
#define RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_SYMMETRY		1
#define RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_VN			2

namespace RVL
{
	namespace SURFEL
	{
		struct AgEdge
		{
			int iVertex[2];
			GRAPH::EdgePtr2<AgEdge> *pVertexEdgePtr[2];
			int idx;
			SurfelAdjecencyDescriptors desc;
			float cost;
			float distance;
			AgEdge *pNext;
		};

		class ObjectGraph;

		struct Object
		{
			int iNode;
			QList<QLIST::Index> surfelList;
			Array<int> iSurfelArray; //Vidovic
			Array<int> iVertexArray;
			Array<int> CTIs;
			float varGRF;
			BYTE flags;
			int size;
			float *d;
			uchar *bd;
			int label;
			Array<int> iClusterArray; //Vidovic
		};

		struct ObjectDisplayData
		{
			Mesh *pMesh;
			SurfelGraph *pSurfels;
			ObjectGraph *pObjects;
			Visualizer *pVisualizer;
			unsigned char defaultColor[3];
			unsigned char selectionColor[3];
			int iSelectedObject;
			bool bObjects;
			bool bUniformColor;
		};

		struct ObjectEdgeData
		{
			float PContinuous;
			float PConvex;
			float PClean;
			float P;
		};
		
		struct ObjectCoverage
		{
			int iObject;
			unsigned char type;
			float coverage;
		};

		struct ConnectedSetRGData
		{
			int iRefObject;
			float maxDist;
			int minSize;
		};

		struct ConnectedSetRGData2
		{
			int iRefObject;
			bool *bAssigned;
			Array<Object> nodeArray;
			int *objectMap;
		};

		struct Cell
		{
			int idx;
			QList<QLIST::Index> surfelList;
			QList<QLIST::Index> objectList;
			int size;
		};

		int ConnectedSetRG(
			int iObject,
			int iObject_,
			AgEdge *pEdge,
			ObjectGraph *pObjects,
			ConnectedSetRGData *pData);
		int ConnectedSetRG2(
			int iObject,
			int iObject_,
			GRAPH::Edge *pEdge,
			Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> *pG,
			ConnectedSetRGData2 *pData);
		bool objectKeyPressUserFunction(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			std::string &key,
			void *vpData);
		bool objectMouseRButtonDownUserFunction(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSelectedPt,
			int iSelectedSurfel,
			void *vpData);

		//Filko
		//Definition of iterator type
		typedef std::map<int, bool>::iterator ObjectsSurfelConvexity_iterator_type;
		typedef std::set<int>::iterator CHVertexIndices_iterator_type;
		//
		struct ObjectGraphObjectData
		{
			std::vector<std::set<int>> CHVertexIndices;
			std::vector<std::map<int, bool>> ObjectsSurfelConvexity;
			std::vector<RVLColorDescriptor> colordescriptor;
			std::vector<float> convexityMultipliers;
			//std::vector<std::vector<float>> bbDistances;
		};
		//

		class ObjectGraph :
			public Graph < GRAPH::AggregateNode<AgEdge>, AgEdge, GRAPH::EdgePtr2<AgEdge> >
		{
		public:
			ObjectGraph();
			virtual ~ObjectGraph();
			void CreateParamList(CRVLMem *pMem);
			void Create(SurfelGraph *pSurfels_);
#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
			void GroupAccordingToGroundTruth();
			void CreateFromGroundTruth(SurfelGraph *pSurfels_);
			void CalculateOverAndUnderSegmentation_SSF(int *E, int &N, bool useGTNoPix = true, bool useBackground = true);	//Filko
			void CalculateOverAndUnderSegmentation(
				int *E, 
				int &N, 
				bool useGTNoPix = false, 
				std::string GTlabImgFilename = "", 
				bool useBackground = true,
				std::string selectedGTObjectFileName = "",
				std::vector<ObjectCoverage> *pSelectedGTObjectCoverage = NULL);	//Filko
#endif
			void CreateFromSSF(std::string ssfFileName);	//Filko
			static void CalculateOverAndUnderSegmentation_Img(
				int *E,
				int &N,
				std::string SegLabImgFilename,
				std::string GTlabImgFilename,
				std::string DepthImgFilename,
				bool useGTNoPix = false,
				bool useBackground = true,
				std::string selectedGTObjectFileName = "",
				std::vector<ObjectCoverage> *pSelectedGTObjectCoverage = NULL);	//Filko
			void DetermineObjectConvexityData(float convexThr = 0.005, float minDiffFlipReq = 0.1, bool verbose = false);	//Filko
			void CalculateConvexityRatiosForObjectPair(int firstObject, int secondObject, float& firstRatio, float& secondRatio, float convexThr = 0.005);	//Filko
			void CalculateObjectsColorHistogram(); //Filko
			bool(*ExtFuncCheckIfWithinVolume)(void*, int, int, float);	//Filko
			void ObjectAggregationLevel2_ViaObjectPairConvexity(float convexThr, float ratioThr, float ratioThr2, int objValidThr = 300, bool verbose = false); //Filko - NOT OPTIMIZED!!!!
			void SaveSegmentationLabelImg(std::string filename); //Filko
			bool CheckObjectUniformity(int objectIdx, int minSurfelSize, float uniThr); //Filko
			bool CheckIfNeighbours(int iObject1, int iObject2);	//Filko
			void RenderConvexityPos(int iObjectSurf, int iObjectVert, Mesh *pMeshScene);	//Filko
			void FlattenVertex(const float * P, float * Pc, const float * N, float d); //Filko
			bool MergeSmallObjects(int sizeThr = 500, float maxDistThr = 0.02, bool verbose = false); //Filko
			std::vector<std::vector<int>> GetObjectBSNeighbourhood(float boundingSphereRadiusThr, bool verbose = false); //Filko //Returns bounding sphere neighbourhood
			bool CheckBoundingSphere(std::vector<int> objects, float boundingSphereRadiusThr);	//Filko	//Checks if the list of objects is within a certain bounding sphere radius
			
			void WERSegmentation();
			void ComputeRelationCosts();
			void ComputeRelationCost(
				AgEdge *pEdge,
				ObjectEdgeData &data);
			//void CreateSortedObjectArray();
			void SortElements(
				GRAPH::AggregateNode<AgEdge> *pAgNode,
				Array<SortIndex<int>> *pSortedElementIdxArray);
			void SortObjects();
			void CountValidObjects();
			void GetVertices();
#ifdef RVLSURFEL_IMAGE_ADJACENCY
			void CreateObjectsAsConnectedComponents(
				Array<int> &groundPlaneObjectArray,
				float maxDist,
				int minSize);
#endif
			void CreateObjectsAsConnectedComponents2(
				int cellSize,
				int minSize);
			void GroupObjectsAccordingToMahalanobisDistance(
				float distThr,
				float uncertainty,
				uchar mask = 0x00,
				uchar flags = 0x00);
			void ObjectsInVOI(
				float *RSG,
				float *tSG,
				float r);
			void InitDisplay(
				Visualizer *pVisualizer,
				Mesh *pMesh,
				unsigned char *selectionColor);
			void Display();
			void Display2();
			void PaintObject(
				int iObject,
				unsigned char *color);
			void PaintObject(
				SURFEL::Object *pObject,
				unsigned char *color);
			void WriteSurfelDataToFile(FILE *fp);
			void WriteObjectDataToFile(FILE *fp);
			void InitSVMClassifier(char *svmParamsFileName);	//Nyarko
			cv::Mat CreateSegmentationImage();
			cv::Mat CreateSegmentationImageFromSSF();
			void Debug();
			static void LoadSelectedGTObjects(
				char *meshFileName,
				char *selectedGTObjectFileName,
				std::vector<ObjectCoverage> &selectedGTObjectCoverage);
			void ObjectMapMask(cv::Mat *pMask);
			int GetForegroundObject();
			void UnionOfVertices(
				Array<int> objects,
				Array<int> &iVertexArray,
				bool *bVertexInArray);
			void UnionOfSurfels(
				Array<int> objects,
				Array<int> &iSurfelArray,
				bool *bSurfelInArray);
			void UnionOfObjects(
				Array<int> objects,
				Array<int> &iSurfelArray,
				Array<int> &iVertexArray,
				bool *bSurfelInArray,
				bool *bVertexInArray);
			void SurfelCells(
				std::vector<int> validObjects);

		public:
			CRVLParameterList ParamList;
			CRVLMem *pMem;
			SurfelGraph *pSurfels;
			float WERSegmentationMinCostDiff;
			float WERSegmentationCostResolution;
			ObjectDisplayData displayData;
			int *objectMap;
			std::shared_ptr<SceneSegFile::SceneSegFile> ssf;	//Filko
			std::map<int, int> objID2idxMap; //Filko
			SVMClassifier *pSVMClassifier;  //Nyarko
			//Array<int> objectArray;
			float kCoverage;
			float alpha;
			ObjectGraphObjectData additionalObjectData;	//Filko
			//Array<int> *sortedElementIdxArray;
			DWORD relationClassifier;
			DWORD objectAggregationLevel2Method;
			Array<SortIndex<int>> sortedObjectArray;
			Array<SURFEL::Object> objectArray;
			int *iObjectAssignedToNode;
			int *objectVertexIdxMem;
			int nValidObjects;
			bool bObjectAggregationLevel2Uncertainty;
			bool bObjectAggregationLevel2Edges;
			bool bFlattenVertices;
			bool bConcaveObjectAggregation;
			bool b3DNetVOI;
			void(*objectAggregationLevel2Criterion)(ObjectGraph *pObjects, int iObject1, int iObject2, void *vpData);
			void *vpObjectAggregationLevel2CriterionData;
			FILE *fpSymmetry;
			int minObjectSize;
			float continuousThr;
			float convexThr;
			float cleanThr;
			float depthStepIntThr;
			float depthStepExtThr;
			float concaveAngleIntThr;
			float concaveAngleExtThr;
			float concaveMinCost;

			Mesh *pMesh; //FIlko
			QList<GRAPH::HierarchyNode> hierarchy;
			GaussianDistribution3D<float> *distribution;
			QLIST::Index *elementMem;
			Array<SURFEL::Cell> surfelCells;
			QLIST::Index *surfelCellObjectMem;
			QLIST::Index *surfelCellSurfelMem;
			Array<int> *objectCells;
			int *objectCellMem;
			//int *sortedElementIdxMem;
		};
	}

	// The next block should be moved to Graph.h after completion.

	namespace GRAPH
	{
		template<typename NodeType, typename EdgeType, typename EdgePtrType, typename CostType>
		void WERAggregation2(
			Graph<NodeType, EdgeType, EdgePtrType> &graph,
			QList<HierarchyNode> *pHierarchy,
			int *aggregateMap,
			QLIST::Index *elementListMem,
			CostType minCostDiff,
			CostType costResolution,
			CRVLMem *pMem)
		{
#ifdef RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
			FILE *fp = fopen("C:\\RVL\\Debug\\WERAgg.txt", "w");
#endif
			// Initialize hierarchy.

			RVLQLIST_INIT(pHierarchy);

			// Initialize elements lists of all nodes. 

			QLIST::Index *pElement = elementListMem;

			NodeType *pNode;
			int iNode;
			QList<QLIST::Index> *pElementList;
			HierarchyNode *pHierarchyNode;

			for (iNode = 0; iNode < graph.NodeArray.n; iNode++)
			{
				pNode = graph.NodeArray.Element + iNode;

				pElementList = &(pNode->elementList);

				RVLQLIST_INIT(pElementList);

				RVLQLIST_ADD_ENTRY(pElementList, pElement);

				pElement->Idx = iNode;

				pElement++;

				RVLMEM_ALLOC_STRUCT(pMem, HierarchyNode, pHierarchyNode);

				RVLQLIST_ADD_ENTRY(pHierarchy, pHierarchyNode);

				pHierarchyNode->pChild[0] = pHierarchyNode->pChild[1] = pHierarchyNode->pParent = NULL;
				pHierarchyNode->iElement = iNode;

				pNode->pHierarchyNode = pHierarchyNode;
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

				// Add new node to the hierarchy.

				RVLMEM_ALLOC_STRUCT(pMem, HierarchyNode, pHierarchyNode);

				RVLQLIST_ADD_ENTRY(pHierarchy, pHierarchyNode);

				pNode1->pHierarchyNode->pParent = pHierarchyNode;

				pHierarchyNode->pChild[0] = pNode1->pHierarchyNode;

				pNode1->pHierarchyNode = pHierarchyNode;

				pNode2->pHierarchyNode->pParent = pHierarchyNode;

				pHierarchyNode->pChild[1] = pNode2->pHierarchyNode;

				pHierarchyNode->pParent = NULL;

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

						//if (pEdge13->cost > pRefEdge->cost)		// Region growing method
						//	pRefEdge->cost = pEdge13->cost;

						if (pEdge13->distance < pRefEdge->distance)
							pRefEdge->distance = pEdge13->distance;

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
