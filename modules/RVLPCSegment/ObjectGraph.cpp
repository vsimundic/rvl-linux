//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkPolyLine.h>
#include "Util.h"
//#define RVLPCSEGMENT_GRAPH_WERAGGREGATION_DEBUG
//#define RVLPCSEGMENT_GRAPH_WERAGGREGATION_DETAILED_DEBUG
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"

#include <numeric>
#include <queue>

#define RVLPCSEGMENT_OBJECT_GRAPH_IGNORE_EDGES
#define RVLPCSEGMENT_OBJECT_GRAPH_OBJECT_SIZE_WITHOUT_EDGES

//#define RVLPCSEGMENT_OBJECT_GRAPH_LOG					// Currently is not used for anything!
//#define RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
#define RVLPCSEGMENT_OBJECT_GRAPH_SYMMETRY_LOG
#define RVLPCSEGMENT_OBJECT_GRAPH_OBJECT_AGGREGATION_LEVEL2_GREEDY_GROUPING2

using namespace RVL;
using namespace SURFEL;

ObjectGraph::ObjectGraph()
{
	WERSegmentationMinCostDiff = 0.1f;
	WERSegmentationCostResolution = 0.01f;
	kCoverage = 0.99f;
	alpha = 0.5f;
	minObjectSize = 300;
	continuousThr = 0.015f;
	convexThr = -20.0f;
	cleanThr = 0.8f;
	depthStepIntThr = 0.005f;
	depthStepExtThr = 0.025f;
	concaveAngleIntThr = 0.0f;
	concaveAngleExtThr = 45.0f;
	concaveMinCost = 0.3f;

	bObjectAggregationLevel2Uncertainty = false;
	bObjectAggregationLevel2Edges = false;
	bFlattenVertices = false;
	bConcaveObjectAggregation = false;
	b3DNetVOI = false;

	nValidObjects = -1;
	sortedObjectArray.n = -1;

	elementMem = NULL;
	NodeArray.Element = NULL;
	EdgeArray.Element = NULL;
	EdgePtrMem = NULL;
	objectMap = NULL;
	//objectArray.Element = NULL;
	//sortedElementIdxMem = NULL;
	//Array<int> *sortedElementIdxArray = NULL;
	sortedObjectArray.Element = NULL;
	objectArray.Element = NULL;
	objectVertexIdxMem = NULL;
	iObjectAssignedToNode = NULL;
	surfelCells.Element = NULL;
	surfelCellObjectMem = NULL;
	surfelCellSurfelMem = NULL;
	objectCells = NULL;
	objectCellMem = NULL;

	fpSymmetry = NULL;

	relationClassifier = RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC;
	objectAggregationLevel2Method = RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_CONVEXITY;

	ExtFuncCheckIfWithinVolume = NULL;
	distribution = NULL;
			}


ObjectGraph::~ObjectGraph()
			{
	RVL_DELETE_ARRAY(elementMem);
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(EdgePtrMem);
	RVL_DELETE_ARRAY(objectMap);
	//RVL_DELETE_ARRAY(objectArray.Element);
	//RVL_DELETE_ARRAY(sortedElementIdxMem);
	//RVL_DELETE_ARRAY(sortedElementIdxArray);
	//RVL_DELETE_ARRAY(sortedObjectArray.Element);
	RVL_DELETE_ARRAY(objectArray.Element);
	RVL_DELETE_ARRAY(objectVertexIdxMem);
	RVL_DELETE_ARRAY(iObjectAssignedToNode);
	RVL_DELETE_ARRAY(surfelCells.Element);
	RVL_DELETE_ARRAY(surfelCellObjectMem);
	RVL_DELETE_ARRAY(surfelCellSurfelMem);
	RVL_DELETE_ARRAY(objectCells);
	RVL_DELETE_ARRAY(objectCellMem);
			}

void ObjectGraph::CreateParamList(CRVLMem *pMem)
		{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("ObjectGraph.alpha", RVLPARAM_TYPE_FLOAT, &alpha);
	pParamData = ParamList.AddParam("ObjectGraph.continuousThr", RVLPARAM_TYPE_FLOAT, &continuousThr);
	pParamData = ParamList.AddParam("ObjectGraph.convexThr", RVLPARAM_TYPE_FLOAT, &convexThr);
	pParamData = ParamList.AddParam("ObjectGraph.cleanThr", RVLPARAM_TYPE_FLOAT, &cleanThr);
	pParamData = ParamList.AddParam("ObjectGraph.depthStepIntThr", RVLPARAM_TYPE_FLOAT, &depthStepIntThr);
	pParamData = ParamList.AddParam("ObjectGraph.depthStepExtThr", RVLPARAM_TYPE_FLOAT, &depthStepExtThr);
	pParamData = ParamList.AddParam("ObjectGraph.concaveAngleIntThr", RVLPARAM_TYPE_FLOAT, &concaveAngleIntThr);
	pParamData = ParamList.AddParam("ObjectGraph.concaveAngleExtThr", RVLPARAM_TYPE_FLOAT, &concaveAngleExtThr);
	pParamData = ParamList.AddParam("ObjectGraph.concaveMinCost", RVLPARAM_TYPE_FLOAT, &concaveMinCost);
	pParamData = ParamList.AddParam("ObjectGraph.relationClassifier", RVLPARAM_TYPE_ID, &relationClassifier);
	ParamList.AddID(pParamData, "HEURISTIC", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_HEURISTIC);
	ParamList.AddID(pParamData, "SVM", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM);
	ParamList.AddID(pParamData, "NLMC", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC);
	ParamList.AddID(pParamData, "NLMC2", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_NLMC2);
	ParamList.AddID(pParamData, "FUZZY_HEURISTIC", RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_FUZZY_HEURISTIC);
	pParamData = ParamList.AddParam("ObjectGraph.objectAggregationLevel2.uncertainty", RVLPARAM_TYPE_BOOL, &bObjectAggregationLevel2Uncertainty);
	pParamData = ParamList.AddParam("ObjectGraph.objectAggregationLevel2.edges", RVLPARAM_TYPE_BOOL, &bObjectAggregationLevel2Edges);
	pParamData = ParamList.AddParam("ObjectGraph.objectAggregationLevel2.method", RVLPARAM_TYPE_ID, &objectAggregationLevel2Method);
	ParamList.AddID(pParamData, "CONVEXITY", RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_CONVEXITY);
	ParamList.AddID(pParamData, "SYMMETRY", RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_SYMMETRY);
	ParamList.AddID(pParamData, "VN", RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_VN);
	pParamData = ParamList.AddParam("ObjectGraph.minObjectSize", RVLPARAM_TYPE_INT, &minObjectSize);
	pParamData = ParamList.AddParam("ObjectGraph.flattenVertices", RVLPARAM_TYPE_BOOL, &bFlattenVertices);
	pParamData = ParamList.AddParam("ObjectGraph.concaveObjectAggregation", RVLPARAM_TYPE_BOOL, &bConcaveObjectAggregation);
	pParamData = ParamList.AddParam("ObjectGraph.3DNetVOI", RVLPARAM_TYPE_BOOL, &b3DNetVOI);
}

#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
void ObjectGraph::CreateFromGroundTruth(SurfelGraph *pSurfels_)
{
	pSurfels = pSurfels_;

	// Create an object for each surfel.

	RVL_DELETE_ARRAY(NodeArray.Element);
	NodeArray.Element = new GRAPH::AggregateNode<AgEdge>[pSurfels->NodeArray.n];
	NodeArray.n = pSurfels->NodeArray.n;
	RVL_DELETE_ARRAY(elementMem);
	elementMem = new QLIST::Index[pSurfels->NodeArray.n];
	RVL_DELETE_ARRAY(objectMap);
	objectMap = new int[pSurfels->NodeArray.n];
	EdgeArray.n = 0;

	QLIST::Index *piElement = elementMem;

	int i;
	int iSurfel;
	Surfel *pSurfel;
	GRAPH::AggregateNode<AgEdge> *pAgNode;
	QList<GRAPH::EdgePtr2<AgEdge>> *pEdgeList;
	QList<QLIST::Index> *pElementList;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pAgNode = NodeArray.Element + iSurfel;

		pElementList = &(pAgNode->elementList);

		RVLQLIST_INIT(pElementList);

		pEdgeList = &(pAgNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		//if (pSurfel->size < 0)
		//	int debug = 0;

		pAgNode->size = 0;
	}

	// Allocate array for storing indices of reference surfels of GT objects.

	Array<int> refSurfelArray;

	refSurfelArray.n = 0;

	for (int i = 0; i < pSurfels->NodeArray.n; i++)
	{
		if (pSurfels->NodeArray.Element[i].GTObjHist.size() > 0)
		{
			refSurfelArray.n = this->pSurfels->NodeArray.Element[i].GTObjHist.size();
			break;
		}
	}

	if (refSurfelArray.n == 0)
		return;

	refSurfelArray.Element = new int[refSurfelArray.n];

	memset(refSurfelArray.Element, 0xff, refSurfelArray.n * sizeof(int));

	// Create objects from the ground truth.

	int iRefSurfel;
	Surfel *pRefSurfel;
	GRAPH::AggregateNode<AgEdge> *pRefAgNode;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pAgNode = NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

		if (pSurfel->bEdge)
			continue;

		if (pSurfel->ObjectID >= 0 && pSurfel->ObjectID < refSurfelArray.n)
		{
			iRefSurfel = refSurfelArray.Element[pSurfel->ObjectID];

			if (iRefSurfel < 0)
				iRefSurfel = refSurfelArray.Element[pSurfel->ObjectID] = iSurfel;

			pRefAgNode = NodeArray.Element + iRefSurfel;

			pElementList = &(pRefAgNode->elementList);

			RVLQLIST_ADD_ENTRY(pElementList, piElement);
			piElement->Idx = iSurfel;

			pRefAgNode->size += pSurfel->size;

			piElement++;
		}
	}

	//int iObject;
	//QLIST::Index *pElementIdx;

	//for (iObject = 0; iObject < NodeArray.n; iObject++)
							//{
	//	pAgNode = NodeArray.Element + iObject;

	//	if (pAgNode->elementList.pFirst == NULL)
	//		continue;

	//	printf("Object %d: ", pSurfels->NodeArray.Element[iObject].ObjectID);

	//	pElementList = &(pAgNode->elementList);

	//	pElementIdx = pElementList->pFirst;

	//	while (pElementIdx)
	//	{
	//		printf("%d, ", pElementIdx->Idx);

	//		pElementIdx = pElementIdx->pNext;
	//	}

	//	printf("\n");
							//}

	// Free memory.

	delete[] refSurfelArray.Element;
}

void ObjectGraph::GroupAccordingToGroundTruth()
{
	// Allocate array for storing indices of reference AgNodes of GT objects.

	Array<int> refAgNodeArray;

	int nGTObjects = 0;

	for (int i = 0; i < pSurfels->NodeArray.n; i++)
	{
		if (pSurfels->NodeArray.Element[i].GTObjHist.size() > 0)
		{
			nGTObjects = this->pSurfels->NodeArray.Element[i].GTObjHist.size();
			break;
		}
	}

	if (nGTObjects == 0)
		return;

	refAgNodeArray.Element = new int[nGTObjects];

	memset(refAgNodeArray.Element, 0xff, nGTObjects * sizeof(int));

	refAgNodeArray.n = nGTObjects;

	// Group nodes according to the ground truth.

	int *overlap = new int[nGTObjects];

	int iAgNode, iGTObject;
	GRAPH::AggregateNode<AgEdge> *pAgNode, *pRefAgNode;
	QLIST::Index *piElement;
	Surfel *pSurfel;
	int maxOverlap;
	int iAssignedGTObject, iRefAgNode;
	QList<QLIST::Index> *pElementList, *pRefElementList;

	for (iAgNode = 0; iAgNode < NodeArray.n; iAgNode++)
	{
		pAgNode = NodeArray.Element + iAgNode;

		if (pAgNode->elementList.pFirst == NULL)
			continue;

		memset(overlap, 0, nGTObjects * sizeof(int));

		piElement = pAgNode->elementList.pFirst;

		while (piElement)
		{
			pSurfel = pSurfels->NodeArray.Element + piElement->Idx;

			for (iGTObject = 0; iGTObject < pSurfel->GTObjHist.size(); iGTObject++)
				overlap[iGTObject] += pSurfel->GTObjHist[iGTObject];

			piElement = piElement->pNext;
		}

		maxOverlap = 0;

		for (iGTObject = 0; iGTObject < nGTObjects; iGTObject++)
			if (overlap[iGTObject] > maxOverlap)
			{
				maxOverlap = overlap[iGTObject];

				iAssignedGTObject = iGTObject;
			}

		if (maxOverlap == 0)
			continue;

		iRefAgNode = refAgNodeArray.Element[iAssignedGTObject];

		if (iRefAgNode < 0)
			refAgNodeArray.Element[iAssignedGTObject] = iAgNode;
		else
		{
			pRefAgNode = NodeArray.Element + iRefAgNode;

			pElementList = &(pAgNode->elementList);

			pRefElementList = &(pRefAgNode->elementList);

			RVLQLIST_APPEND(pRefElementList, pElementList);

			RVLQLIST_INIT(pElementList);

			pRefAgNode->size += pAgNode->size;
		}
	}

	// Free memory.

	delete[] refAgNodeArray.Element;
	delete[] overlap;
}
#endif

void ObjectGraph::Create(SurfelGraph *pSurfels_)
{
	pSurfels = pSurfels_;

	RVL_DELETE_ARRAY(NodeArray.Element);
	NodeArray.Element = new GRAPH::AggregateNode<AgEdge>[pSurfels->NodeArray.n];
	NodeArray.n = pSurfels->NodeArray.n;
#ifdef RVLSURFEL_IMAGE_ADJACENCY
	RVL_DELETE_ARRAY(EdgeArray.Element);
	EdgeArray.Element = new AgEdge[pSurfels->nImageAdjacencyRelations];
	EdgeArray.n = pSurfels->nImageAdjacencyRelations;
	RVL_DELETE_ARRAY(EdgePtrMem);
	EdgePtrMem = new GRAPH::EdgePtr2<AgEdge>[2 * EdgeArray.n];
#endif
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

		pAgNode->flags = 0x00;
	}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pAgNode = NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_IGNORE_EDGES
		if (pSurfel->bEdge)
			continue;
#endif

		if (pSurfel->BoundaryArray.n == 0)
			continue;

		pEdgeList = &(pAgNode->EdgeList);

		for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
		{
			pSurfel_ = pSurfel->imgAdjacency.at(i);

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_IGNORE_EDGES
			if (pSurfel_->bEdge)
				continue;
#endif

			if (pSurfel_->BoundaryArray.n == 0)
				continue;

			pDesc = pSurfel->imgAdjacencyDescriptors.at(i);

			iSurfel_ = pSurfel_ - pSurfels->NodeArray.Element;

			if (iSurfel < iSurfel_)
			{
				pEdge->iVertex[0] = iSurfel;
				pEdge->iVertex[1] = iSurfel_;
				pEdge->desc = *pDesc;
				pEdge->cost = 0.0f;
				pEdge->distance = pDesc->minDist;
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
#endif
}

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

#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
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
	bool useBackground,
	std::string selectedGTObjectFileName,
	std::vector<ObjectCoverage> *pSelectedGTObjectCoverage)
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

	if (pSelectedGTObjectCoverage)
		LoadSelectedGTObjects((char *)(imageFileName.c_str()), (char *)(selectedGTObjectFileName.c_str()), *pSelectedGTObjectCoverage);

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	FILE *fp = fopen("C:\\RVL\\ExpRez\\SegmentationToGT.txt", "w");
#endif

	int *nObjectPts = new int[GTHistSize];

	memset(nObjectPts, 0, GTHistSize * sizeof(int));

	if (useGTNoPix)	//Assumption - GT files is in the same directory as the SSF file and has name in format : SSFfilename + a + .png (label image) and SSFfilename + d + .png (depth image)
	{
		std::string imageName = imageFileName;
		imageName.erase(imageName.find_last_of("."));
		std::string depthImgFileName = imageName + "d.png";
		std::string labelImgFileName = imageName + "a.png";

		//load GT files
		cv::Mat GTLabImg = cv::imread(labelImgFileName, cv::ImreadModes::IMREAD_ANYDEPTH);
		cv::Mat GTDepthImg = cv::imread(depthImgFileName, cv::ImreadModes::IMREAD_ANYDEPTH);
		//Count GT object pixels
		N = 0;

		int GTLabel;

		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
				//adding points that have valid label and depth value

				GTLabel = (int)GTLabImg.at<unsigned char>(y, x);

				nObjectPts[GTLabel]++;

				//if ((GTLabel > 0) && GTDepthImg.at<unsigned short>(y, x) > 0)
				if (GTLabel > 0)
					N++;
				}					
			}
#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
		fprintf(fp, "Total #GTPts: %d\n\n", N);
#endif
	}

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

//#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
//		int nPtsTotal = 0;
//
//		for (int i = 0; i < GTHistSize; i++)
//			nPtsTotal += GTObjHistogram[iObject * GTHistSize + i];
//
//		if (nPtsTotal > 0)
//		{
//			fprintf(fp, "S %d: #P: %d\n", iObject, nPtsTotal);
//
//			fprintf(fp, "-----------------------------\n");
//
//			float fnPtsTotal = (float)nPtsTotal;
//
//			for (int i = 0; i < GTHistSize; i++)
//				fprintf(fp, "GTO %d: #IP: %d, perc: %lf\n", i, GTObjHistogram[iObject * GTHistSize + i], (float)GTObjHistogram[iObject * GTHistSize + i] / fnPtsTotal * 100.0f);
//
//			fprintf(fp, "\n");
//		}
//#endif

	}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	fprintf(fp, "Undersegmentation\n");
	fprintf(fp, "-----------------------------\n");
#endif

	E[0] = 0;	//Oversegmentation values
	E[1] = 0;	//Undersegmentation values
	int* ptrGTObjHist;
	int max = 0;

	int intersection;

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
			intersection = ptrGTObjHist[i];

			if (intersection > max)
			{
				maxBin[iObject] = i;
				max = intersection;
			}
		}

		if (max == 0)//invalid object
			continue;

		//Sum false values
		for (int i = 0; i < GTHistSize; i++)
		{
			//if ((i == 0) && !useBackground)
			//	continue;

			intersection = ptrGTObjHist[i];

			if (i != maxBin[iObject])
			{				
				if (intersection > 0)
				{
					E[1] += intersection;

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
					fprintf(fp, "S %d GTO %d: #Pts: %d, perc: %lf\n", iObject, i, intersection, (float)intersection / (float)N * 100.0f);
#endif
				}
			}

			totVal += intersection;
		}

		//Set max segmented object per max bin
		if (ptrGTObjHist[maxBin[iObject]] > g[maxBin[iObject]])
		{
			maxObj[maxBin[iObject]] = iObject;
			g[maxBin[iObject]] = ptrGTObjHist[maxBin[iObject]];
		}
	}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	fprintf(fp, "\n");
	fprintf(fp, "Oversegmentation\n");
	fprintf(fp, "-----------------------------\n");
#endif

	if (pSelectedGTObjectCoverage)
		for (int j = 0; j < pSelectedGTObjectCoverage->size(); j++)
			pSelectedGTObjectCoverage->at(j).coverage = 0.0f;

	//Sum positive values
	for (int i = 0; i < GTHistSize; i++)
	{
#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
		fprintf(fp, "GTO %d (%d pts): ", i, nObjectPts[i]);
#endif

		if ((i == 0) && !useBackground)
			continue;

		if (i == maxBin[maxObj[i]])
		{
			intersection = GTObjHistogram[maxObj[i] * GTHistSize + i];

			E[0] += intersection;

			if (pSelectedGTObjectCoverage)
			{
				for (int j = 0; j < pSelectedGTObjectCoverage->size(); j++)
				{
					if (i == pSelectedGTObjectCoverage->at(j).iObject)
						pSelectedGTObjectCoverage->at(j).coverage = (float)intersection / (float)nObjectPts[i];
				}
			}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
			fprintf(fp, "S %d #Pts: %d, perc: %lf, error perc: %lf\n", maxObj[i], intersection, (float)intersection / (float)nObjectPts[i] * 100.0f,
				(float)(nObjectPts[i] - intersection) / (float)N * 100.0f);
#endif
		}
#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
		else
			fprintf(fp, "S - #Pts: %d, perc: %lf, error perc: %lf\n", 0, 0.0, (float)nObjectPts[i] / (float)N * 100.0f);
#endif
	}


	//Final results
	/*E[0] = 1 - E[0] / totVal;
	E[1] /= totVal;*/

	if (!useGTNoPix)
		N = totVal;

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	fclose(fp);
#endif

	//DeRef
	delete[] nObjectPts;
	delete[] GTObjHistogram;
	delete[] maxObj;
	delete[] g;
	delete[] maxBin;
}
#endif

void ObjectGraph::LoadSelectedGTObjects(
	char *meshFileName,
	char *selectedGTObjectFileName,
	std::vector<ObjectCoverage> &selectedGTObjectCoverage)
{
	selectedGTObjectCoverage.clear();

	FILE *fpSelectedGTObjects = fopen(selectedGTObjectFileName, "r");

	if (fpSelectedGTObjects)
	{
		char *meshName;
		char *meshFilePath = NULL;

		GetFileNameAndPath(meshFileName, meshName, meshFilePath);

		char line[200];
		char meshName_[200];
		int iObject;
		int type;
		ObjectCoverage objectCoverageData;

		while (true)
		{
			fgets(line, 200, fpSelectedGTObjects);

			if (line[0] == '\n')
				continue;

			if (strstr(line, "end") == line)
				break;

			sscanf(line, "%s\t%d\t%d\n", meshName_, &iObject, &type);

			if (strcmp(meshName, meshName_) == 0)
			{
				objectCoverageData.iObject = iObject;
				objectCoverageData.type = type;

				selectedGTObjectCoverage.push_back(objectCoverageData);
			}
		}

		fclose(fpSelectedGTObjects);
	}
}

//Must be linked with the ground truth (AssignGroundTruthSegmentation per surfel). Return 'Ntrue', 'Nfalse' and 'N' needed to calculate oversegmentation (Fos = 1 - Ntrue/N) and undersegmenation (Fus =Nfalse/N) error. The asumption is that the GT object hist bin with the highest values is the correct one!!! 
void ObjectGraph::CalculateOverAndUnderSegmentation_Img(
	int *E,
	int &N,
	std::string SegLabImgFilename,
	std::string GTlabImgFilename,
	std::string DepthImgFilename,
	bool useGTNoPix,
	bool useBackground,
	std::string selectedGTObjectFileName,
	std::vector<ObjectCoverage> *pSelectedGTObjectCoverage)
{
	//Load segmentation and GT label images
	cv::Mat GTLabImg = cv::imread(GTlabImgFilename, cv::ImreadModes::IMREAD_ANYDEPTH);
	cv::Mat SeglabImg = cv::imread(SegLabImgFilename, cv::ImreadModes::IMREAD_ANYDEPTH);
	cv::Mat GTDepthImg = cv::imread(DepthImgFilename, cv::ImreadModes::IMREAD_ANYDEPTH);
	
	//Getting GThist size and initializing GT object histogram;
	//Get GT label min/max value
	double minLab, maxLab;
	cv::minMaxLoc(GTLabImg, &minLab, &maxLab);
	int GTHistSize = maxLab + 1;
	//Get Segmenation label min/max value
	cv::minMaxLoc(SeglabImg, &minLab, &maxLab);
	int noSegObject = maxLab + 1;
	
	if (pSelectedGTObjectCoverage)
	{
		//Generate a mesh filename
		std::string meshfilename = GTlabImgFilename;
		meshfilename.erase(meshfilename.find_last_of(".") - 1);
		meshfilename += ".ply";
		LoadSelectedGTObjects((char *)(meshfilename.c_str()), (char *)(selectedGTObjectFileName.c_str()), *pSelectedGTObjectCoverage);
	}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	FILE *fp = fopen("C:\\RVL\\ExpRez\\SegmentationToGT.txt", "w");
#endif

	int *nObjectPts = new int[GTHistSize];

	memset(nObjectPts, 0, GTHistSize * sizeof(int));

	if (useGTNoPix)	
	{

		//Count GT object pixels
		N = 0;

		int GTLabel;

		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
				//adding points that have valid label and depth value
				if (GTLabImg.step[1] == 1)
					GTLabel = (int)GTLabImg.at<unsigned char>(y, x);
				else if (GTLabImg.step[1] == 2)
					GTLabel = (int)GTLabImg.at<unsigned short>(y, x);

				nObjectPts[GTLabel]++;

				//if ((GTLabel > 0) && GTDepthImg.at<unsigned short>(y, x) > 0)
				if (GTLabel > 0)
					N++;
			}
		}
#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
		fprintf(fp, "Total #GTPts: %d\n\n", N);
#endif
	}

	int *GTObjHistogram = new int[GTHistSize * noSegObject];	//GTObject histogam per segmented object
	memset(GTObjHistogram, 0, GTHistSize * noSegObject * sizeof(int));
	int *maxObj = new int[GTHistSize];	//Idx of segmented object per maximum bin
	memset(maxObj, 0, GTHistSize * sizeof(int));
	int *g = new int[GTHistSize];	//gama
	memset(g, 0, GTHistSize * sizeof(int));
	int *maxBin = new int[noSegObject];	//maximum bin per segmented object
	memset(maxBin, 0, noSegObject * sizeof(int));

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	Surfel *pSurfel;

	//Calculating GT object histogram
	for (int y = 0; y < GTLabImg.rows; y++)
	{
		for (int x = 0; x < GTLabImg.cols; x++)
		{
			if ((SeglabImg.step[1] == 1) && (GTLabImg.step[1] == 1))
				GTObjHistogram[(int)SeglabImg.at<unsigned char>(y, x) * GTHistSize + (int)GTLabImg.at<unsigned char>(y, x)]++;
			else if ((SeglabImg.step[1] == 2) && (GTLabImg.step[1] == 2))
				GTObjHistogram[(int)SeglabImg.at<unsigned short>(y, x) * GTHistSize + (int)GTLabImg.at<unsigned short>(y, x)]++;
			else if ((SeglabImg.step[1] == 1) && (GTLabImg.step[1] == 2))
				GTObjHistogram[(int)SeglabImg.at<unsigned char>(y, x) * GTHistSize + (int)GTLabImg.at<unsigned short>(y, x)]++;
			else if ((SeglabImg.step[1] == 2) && (GTLabImg.step[1] == 1))
				GTObjHistogram[(int)SeglabImg.at<unsigned short>(y, x) * GTHistSize + (int)GTLabImg.at<unsigned char>(y, x)]++;

		}
	}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	fprintf(fp, "Undersegmentation\n");
	fprintf(fp, "-----------------------------\n");
#endif

	E[0] = 0;	//Oversegmentation values
	E[1] = 0;	//Undersegmentation values
	int* ptrGTObjHist;
	int max = 0;

	int intersection;

	int totVal = 0;
	for (int iObject = 0; iObject < noSegObject; iObject++)
	{
		ptrGTObjHist = &(GTObjHistogram[iObject * GTHistSize]);

		//find max
		max = 0;
		maxBin[iObject] = -1;
		for (int i = 0; i < GTHistSize; i++)
		{
			intersection = ptrGTObjHist[i];

			if (intersection > max)
			{
				maxBin[iObject] = i;
				max = intersection;
			}
		}

		if (max == 0)//invalid object
			continue;

		//Sum false values
		for (int i = 0; i < GTHistSize; i++)
		{
			//if ((i == 0) && !useBackground)
			//	continue;

			intersection = ptrGTObjHist[i];

			if (i != maxBin[iObject])
			{
				if (intersection > 0)
				{
					E[1] += intersection;

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
					fprintf(fp, "S %d GTO %d: #Pts: %d, perc: %lf\n", iObject, i, intersection, (float)intersection / (float)N * 100.0f);
#endif
				}
			}

			totVal += intersection;
		}

		//Set max segmented object per max bin
		if (ptrGTObjHist[maxBin[iObject]] > g[maxBin[iObject]])
		{
			maxObj[maxBin[iObject]] = iObject;
			g[maxBin[iObject]] = ptrGTObjHist[maxBin[iObject]];
		}
	}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	fprintf(fp, "\n");
	fprintf(fp, "Oversegmentation\n");
	fprintf(fp, "-----------------------------\n");
#endif

	if (pSelectedGTObjectCoverage)
		for (int j = 0; j < pSelectedGTObjectCoverage->size(); j++)
			pSelectedGTObjectCoverage->at(j).coverage = 0.0f;

	//Sum positive values
	for (int i = 0; i < GTHistSize; i++)
	{
#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
		fprintf(fp, "GTO %d (%d pts): ", i, nObjectPts[i]);
#endif

		if ((i == 0) && !useBackground)
			continue;

		if (i == maxBin[maxObj[i]])
		{
			intersection = GTObjHistogram[maxObj[i] * GTHistSize + i];

			E[0] += intersection;

			if (pSelectedGTObjectCoverage)
			{
				for (int j = 0; j < pSelectedGTObjectCoverage->size(); j++)
				{
					if (i == pSelectedGTObjectCoverage->at(j).iObject)
						pSelectedGTObjectCoverage->at(j).coverage = (float)intersection / (float)nObjectPts[i];
				}
			}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
			fprintf(fp, "S %d #Pts: %d, perc: %lf, error perc: %lf\n", maxObj[i], intersection, (float)intersection / (float)nObjectPts[i] * 100.0f,
				(float)(nObjectPts[i] - intersection) / (float)N * 100.0f);
#endif
		}
#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
		else
			fprintf(fp, "S - #Pts: %d, perc: %lf, error perc: %lf\n", 0, 0.0, (float)nObjectPts[i] / (float)N * 100.0f);
#endif
	}


	//Final results
	/*E[0] = 1 - E[0] / totVal;
	E[1] /= totVal;*/

	if (!useGTNoPix)
		N = totVal;

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_EVALUATION_LOG
	fclose(fp);
#endif

	//DeRef
	delete[] nObjectPts;
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

	GRAPH::WERAggregation2<GRAPH::AggregateNode<AgEdge>, AgEdge, GRAPH::EdgePtr2<AgEdge>, float>(*this, &hierarchy, objectMap,
		elementMem, WERSegmentationMinCostDiff, WERSegmentationCostResolution, pMem);

	sortedObjectArray.n = -1;
	nValidObjects = -1;

	int size;
	int iNode;
	GRAPH::AggregateNode<AgEdge> *pAgNode, *pElement;
	QLIST::Index *pElementIdx;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		//if (iNode == 1230 || iNode == 786 || iNode == 946)
		//	int debug = 0;

		pAgNode = NodeArray.Element + iNode;

		pElementIdx = pAgNode->elementList.pFirst;

		if (pElementIdx)
		{
			size = 0;

			while (pElementIdx)
			{
				pElement = NodeArray.Element + pElementIdx->Idx;

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_OBJECT_SIZE_WITHOUT_EDGES
				if (!pSurfels->NodeArray.Element[pElementIdx->Idx].bEdge)
#endif
				size += pElement->size;

				pElementIdx = pElementIdx->pNext;
			}

			pAgNode->size = size;
		}
	}

	//CreateSortedObjectArray();

//#ifdef RVLPCSEGMENT_OBJECT_GRAPH_LOG
//	FILE *fpLog = fopen("C:\\RVL\\Debug\\WERAggGraph.txt", "w");
//
//	WriteObjectDataToFile(fpLog);
//
//	fclose(fpLog);
//#endif
	}

//void ObjectGraph::CreateSortedObjectArray()
//{
//	// Compute object sizes and determine the number of objects.
//
//	int *objectArray_ = new int[NodeArray.n];
//
//	objectArray.n = 0;
//
//	int nPts = 0;
//
//	int iNode;
//	GRAPH::AggregateNode<AgEdge> *pAgNode, *pElement;
//	QLIST::Index *pElementIdx;
//	int size;
//
//	for (iNode = 0; iNode < NodeArray.n; iNode++)
//	{
//		//if (iNode == 1230 || iNode == 786 || iNode == 946)
//		//	int debug = 0;
//
//		pAgNode = NodeArray.Element + iNode;
//
//		pElementIdx = pAgNode->elementList.pFirst;
//
//		if (pElementIdx)
//		{
//			size = 0;
//
//			while (pElementIdx)
//			{
//				pElement = NodeArray.Element + pElementIdx->Idx;
//
//				size += pElement->size;
//
//				pElementIdx = pElementIdx->pNext;
//			}
//
//			pAgNode->size = size;
//
//			if (size > 0)
//				nPts += size;
//
//			if (size > 1)
//				objectArray_[objectArray.n++] = iNode;
//		}
//	}
//
//	for (iNode = 0; iNode < NodeArray.n; iNode++)
//	{
//		pAgNode = NodeArray.Element + iNode;
//
//		if (pAgNode->elementList.pFirst == NULL)
//			pAgNode->size = 0;
//	}
//
//	RVL_DELETE_ARRAY(objectArray.Element);
//
//	objectArray.Element = new int[objectArray.n];
//
//	int nCoverage = 0;
//
//	bool *bCoverage = new bool[objectArray.n];
//
//	memset(bCoverage, 0, objectArray.n * sizeof(bool));
//
//	float fnPts = (float)nPts;
//
//	int iObject = 0;
//
//	int iiNode, iiMaxObject;
//	int maxObjectSize;
//
//	while ((float)nCoverage / fnPts < kCoverage)
//	{
//		maxObjectSize = 0;
//
//		for (iiNode = 0; iiNode < objectArray.n; iiNode++)
//		{
//			if (bCoverage[iiNode])
//				continue;
//
//			iNode = objectArray_[iiNode];
//
//			pAgNode = NodeArray.Element + iNode;
//
//			if (pAgNode->size > maxObjectSize)
//			{
//				maxObjectSize = pAgNode->size;
//
//				iiMaxObject = iiNode;
//			}
//		}
//
//		objectArray.Element[iObject++] = objectArray_[iiMaxObject];
//
//		bCoverage[iiMaxObject] = true;
//
//		nCoverage += maxObjectSize;
//	}
//
//	objectArray.n = iObject;
//
//	delete[] bCoverage;
//	delete[] objectArray_;
//}

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
	//if (pEdge->desc.commonBoundaryLength < 20)
	//	pEdge->cost = 0;

	//float scale = 1000.0f;
	float scale = 1.0f;
	float depthStepIntThr_ = depthStepIntThr * scale;
	float depthStepExtThr_ = depthStepExtThr * scale;
	float concaveAngleIntThr_ = concaveAngleIntThr * DEG2RAD;
	float concaveAngleExtThr_ = concaveAngleExtThr * DEG2RAD;

	float f1 = pEdge->desc.cupyDescriptor[0];
	float f2 = pEdge->desc.cupyDescriptor[1];
	float f3 = pEdge->desc.cupyDescriptor[2];
	float f4 = pEdge->desc.cupyDescriptor[3];

	//float PClean_ = 0.0f; 

	float y1, y2, y3, y4;

	switch (relationClassifier){
	case RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_HEURISTIC:
		data.PContinuous = (f4 <= continuousThr ? 1 : 0);
		data.PConvex = (f1 >= convexThr * DEG2RAD ? 1 : 0);
		data.PClean = (f2 >= cleanThr ? 1 : 0);

		data.P = RVLMIN(data.PContinuous, RVLMIN(data.PConvex, data.PClean));

		break;
	case RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_FUZZY_HEURISTIC:
		data.PContinuous = (f4 <= depthStepIntThr_ ? 1.0f : (f4 <= depthStepExtThr_ ? (depthStepExtThr_ - f4) / (depthStepExtThr_ - depthStepIntThr_) : 0.0f));

		data.PConvex = (f1 >= -concaveAngleIntThr_ ? 1.0f : (f1 >= -concaveAngleExtThr_ ? concaveMinCost + (1.0f - concaveMinCost) * (concaveAngleExtThr_ + f1) / (concaveAngleExtThr_ - concaveAngleIntThr_) : concaveMinCost));

		//data.PClean = 0.5f + 0.5f * f2;
		data.PClean = (RVLABS(f1) >= 10.0f * DEG2RAD ? (f3 >= 0.5 ? 2.0f * (f3 - 0.5f) : 0.0f) : 1.0f);
		//data.PClean = (RVLABS(f1) >= 20.0f * DEG2RAD ? (f2 >= 0.5 ? 2.0f * (f2 - 0.5f) : 0.0f) : 1.0f);
		//data.PClean = (f2 >= 0.5 ? 2.0f * (f2 - 0.5f) : 0.0f);

		//PClean_ = (f3 >= 0.5 ? 2.0f * (f3 - 0.5f) : 0.0f);

		//data.PClean = RVLMIN(data.PClean, PClean_);

		data.P = RVLMIN(data.PContinuous, RVLMIN(data.PConvex, data.PClean));

		break;
	case RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM:
		//Nyarko - SVM Classification (courtesy of Grbi�)
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
	RVLSET3VECTOR(displayData.defaultColor, 128, 128, 128);
	displayData.iSelectedObject = -1;
	displayData.bUniformColor = false;

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
		if (displayData.bUniformColor)
		{
			RVLCOPY3VECTOR(displayData.defaultColor, color);
		}
		else
			RandomColor(color);

		PaintObject(iObject, color);
	}
}

void ObjectGraph::Display2()
{
	int iObject;
	unsigned char color[3];

	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		if (displayData.bUniformColor)
		{
			RVLCOPY3VECTOR(displayData.defaultColor, color);
		}
		else
			RandomColor(color);

		PaintObject(objectArray.Element + iObject, color);
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

void ObjectGraph::PaintObject(
	SURFEL::Object *pObject,
	unsigned char *color)
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	Surfel *pSurfel;

	QLIST::Index *piElement = pObject->surfelList.pFirst;

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
	else if (key == "s")
	{
		printf("Select object (max. %d):", pObjects->objectArray.n - 1);
		int iSelectedObject;
		scanf("%d", &iSelectedObject);

		SURFEL::Object *pObject;

		if (iSelectedObject >= 0 && iSelectedObject < pObjects->objectArray.n)
		{
			if (pData->iSelectedObject >= 0)
			{
				pObject = pObjects->objectArray.Element + pData->iSelectedObject;

				pObjects->PaintObject(pObject, pData->defaultColor);
			}

			pData->iSelectedObject = iSelectedObject;

			pObject = pObjects->objectArray.Element + pData->iSelectedObject;

			pObjects->PaintObject(pObject, pData->selectionColor);

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

		printf("Selected object: %d (node %d)\n", pObjects->iObjectAssignedToNode[iObject], iObject);

		return true;
	}
	else
		return false;
}

void ObjectGraph::DetermineObjectConvexityData(float convexThr, float minDiffFlipReq, bool verbose)
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
	int totalSize;
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
		totalSize = 0;
		for (int iS = 0; iS < sortedElementIdxArray.n; iS++)
		{
			sortedIdx = sortedElementIdxArray.Element + iS;
			//getting current surfel
			pSurfel = this->pSurfels->NodeArray.Element + sortedIdx->idx;//piElement->Idx;
			//check if edge
			if (!bObjectAggregationLevel2Edges)
				if (pSurfel->bEdge)
					continue;

			totalSize += pSurfel->size;

			//getting current surfel vertex list
			pSurfelVertexList = this->pSurfels->surfelVertexList.Element + sortedIdx->idx; //piElement->Idx;
			
			fail = false;
			//runnong through a current list of added object vertices
			for (CHVertexIndices_iterator_type iterator = CHVertexIndicesDefDir.begin(); iterator != CHVertexIndicesDefDir.end(); iterator++)
			{
				//*iterator = value
				rvlvertexInList = this->pSurfels->vertexArray.Element[*iterator];
				//if ((pSurfel->N[0] * rvlvertexInList->P[0] + pSurfel->N[1] * rvlvertexInList->P[1] + pSurfel->N[2] * rvlvertexInList->P[2] - pSurfel->d) > convexThr)
				if (pSurfels->Distance(pSurfel, rvlvertexInList->P, bObjectAggregationLevel2Uncertainty) > convexThr)
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
						//if ((pSurfelIN->N[0] * rvlvertex->P[0] + pSurfelIN->N[1] * rvlvertex->P[1] + pSurfelIN->N[2] * rvlvertex->P[2] - pSurfelIN->d) > convexThr)
						if (pSurfels->Distance(pSurfelIN, rvlvertex->P, bObjectAggregationLevel2Uncertainty) > convexThr)
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
		defDirRatio = addedSize / (float)totalSize;
		//if ((addedSize / (float)pObject->size) < ratioThr)
		//	this->additionalObjectData.CHVertexIndices.at(iObject).clear(); //if the ratio is lower than threshold, then empty it's list of vertices

		//Check convexity in the other direction (normal)
		//Run through surfels
		addedSize = 0;
		totalSize = 0;
		for (int iS = 0; iS < sortedElementIdxArray.n; iS++)
		{
			sortedIdx = sortedElementIdxArray.Element + iS;
			//getting current surfel
			pSurfel = this->pSurfels->NodeArray.Element + sortedIdx->idx;//piElement->Idx;
			//check if edge
			if (!bObjectAggregationLevel2Edges)
				if (pSurfel->bEdge)
					continue;

			totalSize += pSurfel->size;

			//getting current surfel vertex list
			pSurfelVertexList = this->pSurfels->surfelVertexList.Element + sortedIdx->idx; //piElement->Idx;

			fail = false;
			//runnong through a current list of added object vertices
			for (CHVertexIndices_iterator_type iterator = CHVertexIndicesOtherDir.begin(); iterator != CHVertexIndicesOtherDir.end(); iterator++)
			{
				//*iterator = value
				rvlvertexInList = this->pSurfels->vertexArray.Element[*iterator];
				//if (((-1)*pSurfel->N[0] * rvlvertexInList->P[0] + (-1)*pSurfel->N[1] * rvlvertexInList->P[1] + (-1)*pSurfel->N[2] * rvlvertexInList->P[2] - (-1)*pSurfel->d) > convexThr)
				if (-pSurfels->Distance(pSurfel, rvlvertexInList->P, bObjectAggregationLevel2Uncertainty) > convexThr)
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
						//if (((-1)*pSurfelIN->N[0] * rvlvertex->P[0] + (-1)*pSurfelIN->N[1] * rvlvertex->P[1] + (-1)*pSurfelIN->N[2] * rvlvertex->P[2] - (-1)*pSurfelIN->d) > convexThr)
						if (-pSurfels->Distance(pSurfelIN, rvlvertex->P, bObjectAggregationLevel2Uncertainty) > convexThr)
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
		otherDirRatio = addedSize / (float)totalSize;

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
			if (bConcaveObjectAggregation)
				this->additionalObjectData.convexityMultipliers.at(iObject) = -1.0;
			//std::cout << "Object " << iObject << " is concave!" << std::endl;
		}
		if (verbose)
			std::cout << "Object " << iObject << "has convexity ratios (" << defDirRatio << ", " << otherDirRatio << ") therfore multiplier is: " << this->additionalObjectData.convexityMultipliers.at(iObject) << std::endl;
	}	// for every object
	//Deref
	delete[] sortedElementIdxArray.Element;
	//delete[] bVertexInCH;
}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
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
#endif

void ObjectGraph::FlattenVertex(const float * P, float * Pc, const float * N, float d)
{
	float ntpd;
	ntpd = P[0] * N[0] + P[1] * N[1] + P[2] * N[2] - d;
	Pc[0] = P[0] - ntpd * N[0];
	Pc[1] = P[1] - ntpd * N[1];
	Pc[2] = P[2] - ntpd * N[2];
}

void ObjectGraph::CalculateConvexityRatiosForObjectPair(int firstObject, int secondObject, float& firstRatio, float& secondRatio, float convexThr)
{
	//if ((firstObject == 30) && (secondObject == 32))	//60, 484 za test 57
	//	RenderConvexityPos(secondObject, firstObject, this->pMesh);
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
	Surfel *pSurfelTemp;
	//First
	piElement = pFirstObject->elementList.pFirst;
	int firstTotal = 0;
	int keyVal;
	while (piElement)
	{
		pSurfel = pSurfels->NodeArray.Element + piElement->Idx;
		//check 
		if (!((pSurfel->size <= 1) || (!bObjectAggregationLevel2Edges && pSurfel->bEdge)))
		{
			//Find key value (if there are two surfels with same size)
			keyVal = pSurfel->size;
			while (aggregateObject.count(keyVal))
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
		if (!((pSurfel->size <= 1) || (!bObjectAggregationLevel2Edges && pSurfel->bEdge)))
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
	std::vector<int> chVertexIndices;
	std::vector<int> chVertexIndicesSurfelIdx;
	//std::set<int>::iterator chVertexIndices_iterator;
	bool *added = new bool[aggregateObject.size()];
	memset(added, 0, aggregateObject.size() * sizeof(bool));
	int currIdx = 0;
	int currIdxIN = 0;
	float currmultiplier = 0.0;
	float currmultiplierIN = 0.0;
	float fP[3];
	float *fPu;
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
		for (int i = 0; i < chVertexIndices.size(); i++)
		{
			//*iterator = value
			rvlvertex = this->pSurfels->vertexArray.Element[chVertexIndices.at(i)];
			if (bFlattenVertices)
			{
				pSurfelTemp = pSurfels->NodeArray.Element + chVertexIndicesSurfelIdx.at(i);
				FlattenVertex(rvlvertex->P, fP, pSurfelTemp->N, pSurfelTemp->d);
				fPu = fP;
			}
			else
				fPu = rvlvertex->P;
			//if ((currmultiplier * pSurfel->N[0] * rvlvertex->P[0] + currmultiplier * pSurfel->N[1] * rvlvertex->P[1] + currmultiplier * pSurfel->N[2] * rvlvertex->P[2] - currmultiplier * pSurfel->d) > convexThr)
			if (currmultiplier * pSurfels->Distance(pSurfel, fPu, bObjectAggregationLevel2Uncertainty) > convexThr)
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
					if (bFlattenVertices)
					{
						FlattenVertex(rvlvertex->P, fP, pSurfel->N, pSurfel->d);
						fPu = fP;
					}
					else
						fPu = rvlvertex->P;
					//if ((currmultiplierIN * pSurfelIN->N[0] * rvlvertex->P[0] + currmultiplierIN * pSurfelIN->N[1] * rvlvertex->P[1] + currmultiplierIN * pSurfelIN->N[2] * rvlvertex->P[2] - currmultiplierIN * pSurfelIN->d) > convexThr)
					if (currmultiplierIN * pSurfels->Distance(pSurfelIN, fPu, bObjectAggregationLevel2Uncertainty) > convexThr)
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
				chVertexIndices.push_back(qlistelement->Idx);
				chVertexIndicesSurfelIdx.push_back(surfelIdx);
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
#ifdef RVLPCSEGMENT_OBJECT_GRAPH_SYMMETRY_LOG
	if (objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_SYMMETRY) 
		fpSymmetry = fopen("symmetries.txt", "w");
#endif

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
	int iObject1_, iObject2_;
	for (int iObject = 0; iObject < validObjects.size(); iObject++)
	{
		for (int iObject2 = iObject + 1; iObject2 < validObjects.size(); iObject2++)
		{
			if (objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_CONVEXITY)
			{
				//if external function for checking if objects are within volume is defined
				//Only objects within specified volume within each otherwill be considered
				if (ExtFuncCheckIfWithinVolume)
				{
					//Check if within volume
					if (!ExtFuncCheckIfWithinVolume(vpObjectAggregationLevel2CriterionData, validObjects.at(iObject), validObjects.at(iObject2), 0.30))	//HARDCODED THRESHOLD?????
						continue;
				}
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
			else if (objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_SYMMETRY)
			{
				iObject1_ = iObjectAssignedToNode[validObjects.at(iObject)];
				iObject2_ = iObjectAssignedToNode[validObjects.at(iObject2)];

				objectAggregationLevel2Criterion(this, iObject1_, iObject2_, vpObjectAggregationLevel2CriterionData);
			}	
		}
	}

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_SYMMETRY_LOG
	if (fpSymmetry)
	{
		fclose(fpSymmetry);

		fpSymmetry = NULL;
	}
#endif

	//Generating merge clusters (object pairs is in decreasing order)
	std::map<int, std::set<int>> merge_clusters;
	std::map<int, std::set<int>>::iterator clustIt;
	std::map<int, std::set<int>>::iterator clustIt2;
	std::set<int>::iterator clusterSetIt;
	//int foundSet = 0;
	//bool insertFirst;
	//bool intersection;
	//int secondSet = 0;
	//for (int i = 0; i < merge_pairs.size(); i++)
	//{
	//	foundSet = -1;
	//	secondSet = -1;
	//	insertFirst = false;
	//	intersection = false;
	//	//check if current pair first item is already defined as cluster leader (KEY)
	//	if (merge_clusters.count(merge_pairs.at(i).first))
	//	{
	//		foundSet = merge_pairs.at(i).first;
	//		//check for intersection 
	//		for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
	//		{
	//			if (clustIt->second.count(merge_pairs.at(i).second))
	//			{
	//				secondSet = clustIt->first;
	//				intersection = true;
	//				break;
	//			}
	//		}
	//	}
	//	else //check if current pair first (or second???) item is already in some set //CHAINING!!!
	//	{
	//		for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
	//		{
	//			if (clustIt->second.count(merge_pairs.at(i).first))
	//			{
	//				foundSet = clustIt->first;
	//				break;
	//			}
	//			else if (clustIt->second.count(merge_pairs.at(i).second))
	//			{
	//				foundSet = clustIt->first;
	//				insertFirst = true;
	//				break;
	//			}
	//		}
	//	}
	//	if (intersection) //If intersection then clusters need to be merged
	//	{
	//		merge_clusters.at(foundSet).insert(secondSet);
	//		//merge second cluster into first cluster
	//		for (clusterSetIt = merge_clusters.at(secondSet).begin(); clusterSetIt != merge_clusters.at(secondSet).end(); clusterSetIt++)
	//			merge_clusters.at(foundSet).insert(*clusterSetIt);
	//		//remove the second set
	//		merge_clusters.erase(secondSet);
	//	}
	//	else if (foundSet >= 0) //if found then put the second element in pair in that set
	//	{

	//		if (insertFirst)
	//			merge_clusters.at(foundSet).insert(merge_pairs.at(i).first);
	//		else
	//			merge_clusters.at(foundSet).insert(merge_pairs.at(i).second);
	//	}
	//	else //if not found then create new cluster and put second pair element in it (first pair is the KEY of map pair)
	//	{
	//		merge_clusters.insert(std::pair<int, std::set<int>>(merge_pairs.at(i).first, std::set<int>()));
	//		merge_clusters.at(merge_pairs.at(i).first).insert(merge_pairs.at(i).second);
	//	}
	//}

	bool firstIntoSecond;
	bool secondCL;
	bool secondIntoFirst;
	bool firstCL;
	bool mergeClusters;
	int mergeInto = -1;
	int mergeFrom = -1;
	int first = -1;
	int second = -1;
	for (int i = 0; i < merge_pairs.size(); i++)
	{
		firstIntoSecond = false;
		secondCL = false;
		secondIntoFirst = false;
		firstCL = false;
		mergeClusters = false;
		mergeInto = -1;
		mergeFrom = -1;
		first = merge_pairs.at(i).first;
		second = merge_pairs.at(i).second;
		//check if current pair first item is already defined as cluster leader (KEY)
		if (merge_clusters.count(first))
		{
			firstCL = true;
			//check if second is cluster leader (IT IS NOT POSSIBLE SINCE THE LIST IS ORDERED!!!)
			for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
			{
				if (merge_clusters.count(second))
				{
					secondCL = true;
					mergeInto = first;
					mergeFrom = second;
					mergeClusters = true;
					break;
				}
			}
			//If second is not cluster leader
			if (!secondCL)
			{
				//Check if second is in some other cluster in order to merge them
				for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
				{
					if (clustIt->second.count(second))
					{
						mergeInto = first;
						mergeFrom = clustIt->first;
						mergeClusters = true;
						break;
					}
				}
			}
			if (!mergeClusters) // if clusters are not to be merged than just add second to first's cluster
				secondIntoFirst = true;
		}
		else //First is not cluster leader
		{
			//Check if second is cluster leader
			for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
			{
				if (merge_clusters.count(second))
				{
					secondCL = true;
					break;
				}
			}
			if (secondCL)	//second is cluster leader
		{
				//check if first is in some cluster in order to merge with second0s cluster
			for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
			{
					if (clustIt->second.count(first))
				{
						mergeInto = second;
						mergeFrom = clustIt->first;
						mergeClusters = true;
					break;
				}
			}
				if (!mergeClusters) //first is not in any cluster therefore just add first into second's cluster
					firstIntoSecond = true;
		}
			else //Second is also not cluster leader
		{
				//check if first is already in some other cluster (SHOULD NOT BE POSSIBLE)
			for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
			{
					if (clustIt->second.count(first))
				{
						mergeInto = clustIt->first;
						secondIntoFirst = true;
					break;
				}
				}
				//check if second is already in some cluster
				for (clustIt = merge_clusters.begin(); clustIt != merge_clusters.end(); clustIt++)
				{
					if (clustIt->second.count(second))
				{
						mergeFrom = clustIt->first;	//mergeFrom will be used if first is added into second's cluster
						firstIntoSecond = true;
					break;
				}
				}
				if (firstIntoSecond && secondIntoFirst)	//If they are both in som other clusters then merge clusters
					mergeClusters = true;
			}

			
		}
		if (mergeClusters)	//if something is going to be merged
		{
			if (mergeInto == mergeFrom)	//Already part of the same cluster
				continue;
			merge_clusters.at(mergeInto).insert(mergeFrom);
			//merge second cluster into first cluster
			for (clusterSetIt = merge_clusters.at(mergeFrom).begin(); clusterSetIt != merge_clusters.at(mergeFrom).end(); clusterSetIt++)
				merge_clusters.at(mergeInto).insert(*clusterSetIt);
			//remove the second set
			merge_clusters.erase(mergeFrom);
		}
		else if (secondIntoFirst && (mergeInto >= 0)) //Add second into cluster where first is already located
		{
			merge_clusters.at(mergeInto).insert(second);
		}
		else if (firstIntoSecond && (mergeFrom >= 0))	//Add first into cluster where second is already found
		{
			merge_clusters.at(mergeFrom).insert(first);
		}
		else if (secondIntoFirst) //Add second into first's cluster
		{
			merge_clusters.at(first).insert(second);
		}
		else if (firstIntoSecond) //Add first into second's cluster
		{
			merge_clusters.at(second).insert(first);
		}
		else  //Create new cluster with first as leader (KEY)
		{
			merge_clusters.insert(std::pair<int, std::set<int>>(first, std::set<int>()));
			merge_clusters.at(first).insert(second);
		}
	}

	//Checking cluster consistincy if there are more than two objects in cluster
	//Helper stuff
	struct temp_pair{ int a; int b; float score;  static bool sort_desc(temp_pair first, temp_pair second) { return (first.score > second.score); } };
	//
#ifdef RVLPCSEGMENT_OBJECT_GRAPH_OBJECT_AGGREGATION_LEVEL2_GREEDY_GROUPING2
	int *objectLinkIdx;
	std::vector<temp_pair> objectLinkQueue;
	temp_pair link;
	int nObjects;
	int iFirstOpenLink;
	int iLink, iLink_;
	std::vector<int> aggregate;
	int iGroupedObject;
#else
	std::queue<int> fifo;
	std::vector<std::vector<temp_pair>> object_links;
#endif
	std::vector<int> inputcluster;
	std::vector<int> inputcluster_label;
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

#ifdef RVLPCSEGMENT_OBJECT_GRAPH_OBJECT_AGGREGATION_LEVEL2_GREEDY_GROUPING2
			nObjects = inputcluster.size();

			objectLinkIdx = new int[nObjects * nObjects];

			objectLinkQueue.clear();
			objectLinkQueue.resize((nObjects * (nObjects - 1)) / 2);

			iLink = 0;

			for (int iObject = 0; iObject < nObjects; iObject++)
			{
				for (int iObject2 = iObject + 1; iObject2 < nObjects; iObject2++, iLink++)
				{
					link.a = iObject;
					link.b = iObject2;
					ss.clear();
					ss.str("");
					ss << inputcluster.at(iObject) << "_" << inputcluster.at(iObject2);
					link.score = (min_convexity_values.count(ss.str()) ? min_convexity_values.at(ss.str()) : 0.0);

					objectLinkQueue.at(iLink) = link;
				}
			}

			std::sort(objectLinkQueue.begin(), objectLinkQueue.end(), temp_pair::sort_desc);

			for (iLink = 0; iLink < objectLinkQueue.size(); iLink++)
			{
				link = objectLinkQueue.at(iLink);

				objectLinkIdx[link.a + link.b * nObjects] = objectLinkIdx[link.b + link.a * nObjects] = iLink;
			}

			inputcluster_label.clear();
			inputcluster_label.resize(nObjects, -1);

			label = 0;

			iFirstOpenLink = 0;

			iLink = 0;

			while (iLink < objectLinkQueue.size())	// while there are ungrouped objects in the cluster
			{
				link = objectLinkQueue.at(iLink);

				if (inputcluster_label.at(link.a) != -1 || inputcluster_label.at(link.b) != -1)
				{
					iLink++;

					continue;
				}

				aggregate.clear();
				aggregate.push_back(link.a);

				inputcluster_label.at(link.a) = label;

				iLink_ = iFirstOpenLink = iLink;

				while (iLink_ < objectLinkQueue.size())	// while there are open links
				{
					link = objectLinkQueue.at(iLink_);

					if (inputcluster_label.at(link.a) == label && inputcluster_label.at(link.b) == -1)
						iObject1_ = link.b;
					else if (inputcluster_label.at(link.b) == label && inputcluster_label.at(link.a) == -1)
						iObject1_ = link.a;
					else if (inputcluster_label.at(link.a) == -1 && inputcluster_label.at(link.b) == -1)
					{
						iLink_++;

						continue;
					}
					else
					{
						iFirstOpenLink++;

						iLink_ = iFirstOpenLink;

						continue;
					}

					for (iGroupedObject = 0; iGroupedObject < aggregate.size(); iGroupedObject++)
					{
						iObject2_ = aggregate.at(iGroupedObject);

						if (objectLinkQueue.at(objectLinkIdx[iObject1_ + iObject2_ * nObjects]).score < ratioThr2)
							break;
					}

					if (iGroupedObject == aggregate.size())
					{
						inputcluster_label.at(iObject1_) = label;

						aggregate.push_back(iObject1_);

						iLink_ = iFirstOpenLink;
					}
					else
					{
						inputcluster_label.at(iObject1_) = -2;

						iLink_++;
					}
				}	// while there are open links

				for (iObject1_ = 0; iObject1_ < inputcluster_label.size(); iObject1_++)
					if (inputcluster_label.at(iObject1_) == -2)
						inputcluster_label.at(iObject1_) = -1;

				label++;
			}	// while there are ungrouped objects in the cluster

			delete[] objectLinkIdx;
#else

			//run through cluster and find the object with the best connection
			startObj = 0;
			startObjScore = 0.0;
			for (int iObject = 0; iObject < inputcluster.size(); iObject++)
			{
				for (int iObject2 = iObject + 1; iObject2 < inputcluster.size(); iObject2++)
				{
					ss.clear();
					ss.str("");
					ss << inputcluster.at(iObject) << "_" << inputcluster.at(iObject2);
					if (min_convexity_values.count(ss.str()))
						score = min_convexity_values.at(ss.str());
					else
						score = 0.0;
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
					if (min_convexity_values.count(ss.str()))
						object_links.at(iObject).at(object_links.at(iObject).size() - 1).score = min_convexity_values.at(ss.str());
					else
						object_links.at(iObject).at(object_links.at(iObject).size() - 1).score = 0.0;
					object_links.at(iObject2).push_back(temp_pair());
					object_links.at(iObject2).at(object_links.at(iObject2).size() - 1).a = iObject2;
					object_links.at(iObject2).at(object_links.at(iObject2).size() - 1).b = iObject;
					if (min_convexity_values.count(ss.str()))
						object_links.at(iObject2).at(object_links.at(iObject2).size() - 1).score = min_convexity_values.at(ss.str());
					else
						object_links.at(iObject2).at(object_links.at(iObject2).size() - 1).score = 0.0;
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
							if (min_convexity_values.count(ss.str()))
								score = min_convexity_values.at(ss.str());
							else
								score = 0.0;
							if (score < ratioThr2) //if the score is smaller against any currently labeled member then break
							{
								fail = true;
								break;
							}
						}
						if (fail)//got to the next link
							continue;
						inputcluster_label.at(object_links.at(currObj).at(l).b) = label; //if everything has passed then label the object
						fifo.push(object_links.at(currObj).at(l).b);	//push it to fifo so it's links can be analyzed
					}
				}
				label++; //current cluster finished, go to next
			}
#endif

			//compiling new clusters based on labels
			newclusters.clear();
			newclusters.resize(label);
			for (int i = 0; i < inputcluster_label.size(); i++)
				if (inputcluster_label.at(i) >= 0)
				newclusters.at(inputcluster_label.at(i)).push_back(inputcluster.at(i));

			//for (int i = 0; i < newclusters.size(); i++)
			//	if (newclusters.at(i).size() >= 3)
			//		printf("%d segments grouped.\n", newclusters.at(i).size());

			//Adding new clusters that have more than one member
			for (int i = 0; i < newclusters.size(); i++)
			{
				if (newclusters.at(i).size() < 2)
					continue;
				merge_clusters.insert(std::pair<int, std::set<int>>(newclusters.at(i).at(0), std::set<int>()));
				for (int j = 1; j < newclusters.at(i).size(); j++)
					merge_clusters.at(newclusters.at(i).at(0)).insert(newclusters.at(i).at(j));
			}
		}	// if cluster has three or more segments
	}	// for every cluster

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

			//Update objectMap
			piElement = pElementList2->pFirst;
			while (piElement)
			{
				this->objectMap[piElement->Idx] = clustIt->first;
				piElement = piElement->pNext;
			}

			// iNode1 <- union of iNode1 and iNode2 

			RVLQLIST_APPEND(pElementList1, pElementList2);	//append their surfels

			// iNode2 <- empty set

			RVLQLIST_INIT(pElementList2);	//reset list

			//Update object size
			pNode1->size += pNode2->size;
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

void ObjectGraph::SortObjects()
{
	if (nValidObjects < 0)
		CountValidObjects();

	sortedObjectArray.Element = new SortIndex<int>[nValidObjects];

	SortIndex<int> *pSortIndex = sortedObjectArray.Element;

	int minSize = RVLMAX(20, minObjectSize);

	//printf("valid objects: ");

	int iObject;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;

	for (iObject = 0; iObject < NodeArray.n; iObject++)
	{
		pObject = NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		//check if object
		if (!piElement)
			continue;

		//we are not intrested in objects with size less than 20 points???
		if (pObject->size < minSize)
			continue;

		//printf("%d ", iObject);

		pSortIndex->cost = pObject->size;
		pSortIndex->idx = iObject;
		pSortIndex++;
	}

	//printf("\n");

	sortedObjectArray.n = nValidObjects;

	BubbleSort<SortIndex<int>>(sortedObjectArray, true);

	RVL_DELETE_ARRAY(objectArray.Element);

	objectArray.Element = new Object[sortedObjectArray.n];
	objectArray.n = sortedObjectArray.n;

	RVL_DELETE_ARRAY(iObjectAssignedToNode);

	iObjectAssignedToNode = new int[NodeArray.n];

	memset(iObjectAssignedToNode, 0xff, NodeArray.n * sizeof(int));

	Object *pObject_;

	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		pObject_ = objectArray.Element + iObject;

		pObject_->iNode = sortedObjectArray.Element[iObject].idx;

		pObject = NodeArray.Element + pObject_->iNode;

		pObject_->surfelList = pObject->elementList;
		pObject_->flags = pObject->flags;
		pObject_->size = pObject->size;

		iObjectAssignedToNode[pObject_->iNode] = iObject;
	}

	delete[] sortedObjectArray.Element;
}

void ObjectGraph::CountValidObjects()
{
	nValidObjects = 0;

	int minSize = RVLMAX(20, minObjectSize);

	//printf("valid objects: ");

	int iObject;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;

	for (iObject = 0; iObject < NodeArray.n; iObject++)
	{
		if (iObject == 135)
			int debug = 0;

		pObject = NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		//check if object
		if (!piElement)
			continue;

		//we are not intrested in objects with size less than 20 points???
		if (pObject->size < minSize)
			continue;

		//printf("%d ", iObject);

		//Set flag RVLGRAPH_AGGNODE_FLAG_VALID.

		pObject->flags |= RVLGRAPH_AGGNODE_FLAG_VALID;

		//Increase valid object counter.

		nValidObjects++;
	}

	//printf("\n");
}

void ObjectGraph::GetVertices()
{
	if (sortedObjectArray.n < 0)
		SortObjects();

	pSurfels->bVertexAssigned = new bool[pSurfels->vertexArray.n];

	memset(pSurfels->bVertexAssigned, 0, pSurfels->vertexArray.n * sizeof(bool));

	int nVerticesTotal = 0;

	int iObject;
	Object *pObject;

	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		pObject = objectArray.Element + iObject;

		nVerticesTotal += pSurfels->GetNoVertices(NodeArray.Element[pObject->iNode].elementList);
	}

	RVL_DELETE_ARRAY(pSurfels->iVertexMem);

	pSurfels->iVertexMem = new int[nVerticesTotal];

	int *piNextVertex = pSurfels->iVertexMem;

	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		pObject = objectArray.Element + iObject;

		pSurfels->GetVertices(NodeArray.Element[pObject->iNode].elementList, &(pObject->iVertexArray), piNextVertex);
	}

	delete[] pSurfels->bVertexAssigned;

	pSurfels->bVertexAssigned = NULL;
}

void ObjectGraph::RenderConvexityPos(int iObjectSurf, int iObjectVert, Mesh *pMeshScene)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();;
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//
	//Get pointers to objects
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObjectSurf = this->NodeArray.Element + iObjectSurf;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObjectVert = this->NodeArray.Element + iObjectVert;

	//Adding planes (surfels for pObjectSurf)
	QLIST::Index *piElement;
	Surfel *pSurfel;
	QList<QLIST::Index> *pSurfelVertexList;
	SURFEL::Vertex * rvlvertex;
	QLIST::Index *qlistelement;
	//first surfel
	piElement = pObjectSurf->elementList.pFirst;
	while (piElement)
	{
		pSurfel = this->pSurfels->NodeArray.Element + piElement->Idx;
		//check 
		if (!((pSurfel->size <= 1) || pSurfel->bEdge))
		{
			//Creating plan and adding actor
			vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
			plane->SetOrigin(0.0, 0.0, 0.0);
			plane->SetPoint1(pSurfel->size / (float)pObjectSurf->size, 0.0, 0.0);
			plane->SetPoint2(0.0, pSurfel->size / (float)pObjectSurf->size, 0.0);
			plane->SetNormal(pSurfel->N[0], pSurfel->N[1], pSurfel->N[2]);
			plane->SetCenter(pSurfel->P[0], pSurfel->P[1], pSurfel->P[2]);
			//plane->Push(pSurfel->d);
			plane->SetResolution(5, 5);
			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputConnection(plane->GetOutputPort());
			vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
			act->SetMapper(mapper);
			renderer->AddActor(act);				
		}
		piElement = piElement->pNext;
	}

	//Adding points (vertices for pObjectVert)
	int ptIdx = 0;
	//first surfel
	piElement = pObjectVert->elementList.pFirst;
	while (piElement)
	{
		pSurfel = this->pSurfels->NodeArray.Element + piElement->Idx;
		//check 
		if (!((pSurfel->size <= 1) || pSurfel->bEdge))
		{
			vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
			points->SetDataTypeToFloat();
			vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();

			//getting current surfel vertex list
			pSurfelVertexList = this->pSurfels->surfelVertexList.Element + piElement->Idx;
			//running through added surfel vertices
			qlistelement = pSurfelVertexList->pFirst;
			ptIdx = 0;
			while (qlistelement)
			{
				rvlvertex = this->pSurfels->vertexArray.Element[qlistelement->Idx];
				points->InsertNextPoint(rvlvertex->P);
				verts->InsertNextCell(1, (vtkIdType*)&ptIdx);
				//verts->InsertCellPoint(ptIdx);
				ptIdx++;
				//Next
				qlistelement = qlistelement->pNext;
			}

			//Adding actor
			pd->SetPoints(points);
			pd->SetVerts(verts);
			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputData(pd);
			vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
			act->SetMapper(mapper);
			act->GetProperty()->SetPointSize(5);
			renderer->AddActor(act);
		}

		piElement = piElement->pNext;
	}

	//Add scene
	if (pMeshScene)
	{
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(pMeshScene->pPolygonData);
		vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
		act->SetMapper(mapper);
		renderer->AddActor(act);
	}

	//Start VTK
	renderer->ResetCamera();
	window->Render();
	interactor->Start();
}

bool ObjectGraph::MergeSmallObjects(int sizeThr, float maxDistThr, bool verbose)
{
#ifdef RVLSURFEL_IMAGE_ADJACENCY
	bool merged = false;
	//Determine a list of neighbours and distances for each small object
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pOtherObject;
	Surfel *pSurfel;
	int surfelIdx;
	Surfel *pOtherSurfel;
	int otherSurfelIdx;
	QLIST::Index *piElement;
	std::vector<int> validObjects;
	std::map<int, std::map<int, float>> objectNeighbourhood;
	std::map<int, std::map<int, float>>::iterator objectIt;
	std::map<int, float>::iterator objectNeighboorIt;
	//For each object
	for (int iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;

		if (pObject->size > sizeThr)
			continue;

		piElement = pObject->elementList.pFirst;

		//check if object
		if (!piElement)
			continue;
		//
		objectNeighbourhood.insert(std::make_pair(iObject, std::map<int, float>()));
		//For each object surfel look at adjacent surfel and chech if they are owned by different object
		while (piElement)
		{
			pSurfel = this->pSurfels->NodeArray.Element + piElement->Idx;
			surfelIdx = piElement->Idx;
			//check 
			if (!((pSurfel->size <= 1) || pSurfel->bEdge))
			{
				//Running through its adjacency
				for (int i = 0; i < pSurfel->imgAdjacency.size(); i++)
				{
					pOtherSurfel = pSurfel->imgAdjacency.at(i);
					otherSurfelIdx = pOtherSurfel - this->pSurfels->NodeArray.Element;
					//Check if current and other surfel is owned by different object, if yes add to the list, if it is already there update min distance
					if (this->objectMap[surfelIdx] != this->objectMap[otherSurfelIdx])
					{
						//check if the size of the found object is greater than minimum
						pOtherObject = this->NodeArray.Element + this->objectMap[otherSurfelIdx];
						if (pOtherObject->size <= sizeThr)
							continue;
						//check if it exists
						if (objectNeighbourhood.at(iObject).count(this->objectMap[otherSurfelIdx]))
						{
							//check if update of minimum distance is required
							if (objectNeighbourhood.at(iObject).at(this->objectMap[otherSurfelIdx]) > pSurfel->imgAdjacencyDescriptors.at(i)->minDist)
								objectNeighbourhood.at(iObject).at(this->objectMap[otherSurfelIdx]) = pSurfel->imgAdjacencyDescriptors.at(i)->minDist;
						}
						else //add
							objectNeighbourhood.at(iObject).insert(std::make_pair(this->objectMap[otherSurfelIdx], pSurfel->imgAdjacencyDescriptors.at(i)->minDist));
					}
				}
			}

			piElement = piElement->pNext;
		}
	}

	//Merge object to its closest and largest neighbour
	QList<QLIST::Index> *pElementList1;
	QList<QLIST::Index> *pElementList2;
	float dist = 10000000;
	int size = 0;
	int chosenOtherObject;
	for (objectIt = objectNeighbourhood.begin(); objectIt != objectNeighbourhood.end(); objectIt++)
	{
		//first = key, second = value
		pObject = this->NodeArray.Element + objectIt->first;
		pElementList1 = &pObject->elementList;
		//run through neighbourhood and find min size
		dist = 10000000;
		chosenOtherObject = -1;
		size = 0;
		for (objectNeighboorIt = objectIt->second.begin(); objectNeighboorIt != objectIt->second.end(); objectNeighboorIt++)
		{
			//find largest object whose distance is lower than specified
			pOtherObject = this->NodeArray.Element + objectNeighboorIt->first;
			if (objectNeighboorIt->second > maxDistThr)
				continue;
			if (pOtherObject->size < size)
				continue;
			if (objectNeighboorIt->second < dist)
			{
				dist = objectNeighboorIt->second;
				size = pOtherObject->size;
				chosenOtherObject = objectNeighboorIt->first;
			}
		}

		//check if merege is apropriate
		if ((chosenOtherObject >= 0) && (dist <= maxDistThr))
		{
			pOtherObject = this->NodeArray.Element + chosenOtherObject;
			pElementList2 = &pOtherObject->elementList;
			
			//Update objectMap
			piElement = pElementList1->pFirst;
			while (piElement)
			{
				this->objectMap[piElement->Idx] = chosenOtherObject;
				piElement = piElement->pNext;
			}

			// iNode1 <- union of iNode1 and iNode2 

			RVLQLIST_APPEND(pElementList2, pElementList1);	//append their surfels

			// iNode2 <- empty set

			RVLQLIST_INIT(pElementList1);	//reset list

			//Update object size
			pOtherObject->size += pObject->size;
			if (verbose)
				std::cout << "Merged: " << objectIt->first << " into " << chosenOtherObject << std::endl;
			merged = true;
		}
	}
	return merged;
#else
	return false;
#endif
}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
void ObjectGraph::CreateObjectsAsConnectedComponents(
	Array<int> &groundPlaneObjectArray,
	float maxDist,
	int minSize)
{
	// Create objects from surfels;

	Create(pSurfels);

	memset(objectMap, 0xff, NodeArray.n * sizeof(int));

	// Assign all ground plane objects to the reference ground plane object.

	int iObject;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;

	if (groundPlaneObjectArray.n > 0)
	{
		int iGndPlane = groundPlaneObjectArray.Element[0];

		pObject = NodeArray.Element + iGndPlane;

		pObject->flags |= RVLPCSEGMENT_OBJECT_FLAG_GND;

		objectMap[iGndPlane] = iGndPlane;

		QList<QLIST::Index> *pElementList = &(pObject->elementList);

		QList<QLIST::Index> *pElementList_;
		GRAPH::AggregateNode<SURFEL::AgEdge> *pObject_;
		int i;

		for (i = 1; i < groundPlaneObjectArray.n; i++)
		{
			iObject = groundPlaneObjectArray.Element[i];

			pObject_ = NodeArray.Element + iObject;

			objectMap[iObject] = iGndPlane;

			pElementList_ = &(pObject_->elementList);

			RVLQLIST_APPEND(pElementList, pElementList_);

			RVLQLIST_INIT(pElementList_);

			pObject->size += pObject_->size;

			pObject_->size = 0;
		}
	}

	// Detect objects as connected surfel sets.

	int *iObjectBuff = new int[NodeArray.n];

	ConnectedSetRGData RGData;

	RGData.maxDist = maxDist;
	RGData.minSize = minSize;

	int *piObjectPut, *piObjectFetch, *piObjectBuffEnd;

	for (iObject = 0; iObject < NodeArray.n; iObject++)
	{
		//if (iObject == 74)
		//	int debug = 0;

		if (objectMap[iObject] >= 0)
			continue;

		pObject = NodeArray.Element + iObject;

		if (pObject->size <= 1)
			continue;

		RGData.iRefObject = iObject;

		piObjectPut = piObjectFetch = iObjectBuff;

		*(piObjectPut++) = iObject;

		objectMap[iObject] = iObject;

		piObjectBuffEnd = RegionGrowing<ObjectGraph, GRAPH::AggregateNode<AgEdge>, AgEdge, GRAPH::EdgePtr2<AgEdge>, ConnectedSetRGData,
			ConnectedSetRG>(this, &RGData, piObjectFetch, piObjectPut);
	}

	delete[] iObjectBuff;
}
#endif

void ObjectGraph::GroupObjectsAccordingToMahalanobisDistance(
	float distThr,
	float uncertainty,
	uchar mask,
	uchar flags)
{
	//GaussianDistribution3D<float> *distribution = new GaussianDistribution3D<float>[objectArray.n];

	RVL_DELETE_ARRAY(distribution);

	distribution = new GaussianDistribution3D<float>[objectArray.n];
	
	Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> G;

	G.NodeArray.Element = new GRAPH::Node[objectArray.n];

	// Compute point distributions of all objects. 

	int *PtMem = new int[pMesh->NodeArray.n];

	Array<int> iSurfelArray;
	iSurfelArray.Element = new int[objectArray.n];

	int iObject;
	Object *pObject;
	float *C, *P;
	GRAPH::Node *pNode;
	QList<GRAPH::EdgePtr<GRAPH::Edge>> *pEdgeList;

	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		pObject = objectArray.Element + iObject;

		if ((pObject->flags & mask) != flags)
			continue;

		QLIST::CopyToArray(&(pObject->surfelList), &iSurfelArray);

		pSurfels->ComputeDistribution(pMesh, iSurfelArray, distribution + iObject, PtMem);

		pNode = G.NodeArray.Element + iObject;

		pNode->idx = iObject;

		pEdgeList = &(pNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	delete[] PtMem;

	// Connect objects 

	float uncertainty2 = uncertainty * uncertainty;

	float Cu[9];

	RVLNULLMX3X3(Cu);

	Cu[0] = Cu[4] = Cu[8] = uncertainty2;

	int edgeMemSize = objectArray.n * (objectArray.n - 1) / 2;

	GRAPH::Edge *edgeMem = new GRAPH::Edge[edgeMemSize];
	GRAPH::Edge *pEdge = edgeMem;

	GRAPH::EdgePtr<GRAPH::Edge> *edgePtrMem = new GRAPH::EdgePtr<GRAPH::Edge>[2 * edgeMemSize];
	GRAPH::EdgePtr<GRAPH::Edge> *pEdgePtr = edgePtrMem;

	CvMat *pC = cvCreateMatHeader(3, 3, CV_32FC1);

	float CSum[9];

	pC->data.fl = CSum;

	CvMat *pe = cvCreateMatHeader(3, 1, CV_32FC1);

	float dP[3];

	pe->data.fl = dP;

	CvMat *pV3DTmp = cvCreateMat(3, 1, CV_32FC1);

	int iObject_;
	Object *pObject_;
	float CSum_[9];
	float *C_, *P_;
	float e;
	
	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		pObject = objectArray.Element + iObject;

		if ((pObject->flags & mask) != flags)
			continue;

		C = distribution[iObject].C;
		P = distribution[iObject].P;

		RVLSUMMX3X3UT(Cu, C, CSum_);

		for (iObject_ = iObject + 1; iObject_ < objectArray.n; iObject_++)
		{
			pObject_ = objectArray.Element + iObject_;

			if ((pObject_->flags & mask) != flags)
				continue;

			C_ = distribution[iObject_].C;

			RVLSUMMX3X3UT(CSum_, C_, CSum);

			RVLCOMPLETESIMMX3(CSum);

			P_ = distribution[iObject_].P;

			RVLDIF3VECTORS(P_, P, dP);

			e = MahalanobisDistance<float>(pe, pC, pV3DTmp);

			if (e <= distThr)
			{
				ConnectNodes<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>>(iObject, iObject_, G.NodeArray, pEdge, pEdgePtr);

				pEdge++;
				pEdgePtr += 2;
			}
		}
	}

	cvReleaseMat(&pC);
	cvReleaseMat(&pe);
	cvReleaseMat(&pV3DTmp);

	//delete[] distribution;

	// Group objects.

	int *iObjectBuff = new int[objectArray.n];

	ConnectedSetRGData2 RGData;

	RGData.nodeArray = objectArray;

	RGData.bAssigned = new bool[objectArray.n];
	memset(RGData.bAssigned, 0, objectArray.n * sizeof(bool));
	RGData.objectMap = objectMap;

	int *piObjectPut, *piObjectFetch, *piObjectBuffEnd;

	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		pObject = objectArray.Element + iObject;

		if ((pObject->flags & mask) != flags)
			continue;

		if (RGData.bAssigned[iObject])
			continue;

		RGData.bAssigned[iObject] = true;

		RGData.iRefObject = iObject;

		piObjectPut = piObjectFetch = iObjectBuff;

		*(piObjectPut++) = iObject;

		objectMap[iObject] = iObject;

		piObjectBuffEnd = RegionGrowing<Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>>, GRAPH::Node, GRAPH::Edge,
			GRAPH::EdgePtr<GRAPH::Edge>, ConnectedSetRGData2, ConnectedSetRG2>(&G, &RGData, piObjectFetch, piObjectPut);
	}

	delete[] iObjectBuff;
	delete[] RGData.bAssigned;

	// Free memory.

	delete[] G.NodeArray.Element;
	delete[] edgeMem;
	delete[] edgePtrMem;	
}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
// Function CreateObjectsAsConnectedComponents2 is not completed.

void ObjectGraph::CreateObjectsAsConnectedComponents2(
	int cellSize,
	int minSize)
{
	// Create objects from surfels.

	Create(pSurfels);

	// Create object map.

	memset(objectMap, 0xff, NodeArray.n * sizeof(int));

	// Connect neighboring objects.

	int w = pMesh->width / cellSize;

	if (pMesh->width % cellSize > 0)
		w++;

	int h = pMesh->height / cellSize;

	if (pMesh->height % cellSize > 0)
		h++;

	int nCells = w * h;

	QList<QLIST::Index> *cell = new QList<QLIST::Index>[nCells];

	QList<QLIST::Index> *pCell = cell;

	int i;

	for (i = 0; i < nCells; i++, pCell++)
		RVLQLIST_INIT(pCell);

	QList<QLIST::Index> *objectCellList = new QList<QLIST::Index>[NodeArray.n];

	QList<QLIST::Index> *pCellIdx = objectCellList;

	for (i = 0; i < NodeArray.n; i++, pCellIdx++)
		RVLQLIST_INIT(pCellIdx);

	QLIST::Index *cellMem = new QLIST::Index[pMesh->NodeArray.n];

	QLIST::Index *objectCellListMem = new QLIST::Index[pMesh->NodeArray.n];

	bool *bAssigned = new bool[nCells];

	memset(bAssigned, 0, nCells * sizeof(bool));

	int iObject;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;

	for (iObject = 0; iObject < NodeArray.n; iObject++)
	{
		if (objectMap[iObject] >= 0)
			continue;

		pObject = NodeArray.Element + iObject;

		if (pObject->size <= 1)
			continue;
	}

	delete[] cell;
	delete[] objectCellList;
	delete[] cellMem;
	delete[] objectCellListMem;
	delete[] bAssigned;

	// Detect objects as connected surfel sets.
}
#endif

void ObjectGraph::ObjectsInVOI(
	float *RSG,
	float *tSG,
	float r)
{
	int iObject;
	Object *pObject;

	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		pObject = objectArray.Element + iObject;

		if(pSurfels->InVOI(pObject->iVertexArray, RSG, tSG, r))
			pObject->flags |= RVLPCSEGMENT_OBJECT_FLAG_IN_VOI;
	}
}

int ObjectGraph::GetForegroundObject()
{
	int iForegroundObject = -1;

	int foregroundObjectSize = 0;

	int iObject;
	Object *pObject;

	for (iObject = 0; iObject < objectArray.n; iObject++)
	{
		pObject = objectArray.Element + iObject;

		if (pObject->flags & RVLPCSEGMENT_OBJECT_FLAG_GND)
			continue;

		if (b3DNetVOI)
			if (!(pObject->flags & RVLPCSEGMENT_OBJECT_FLAG_IN_VOI))
				continue;

		if (pObject->size > foregroundObjectSize)
		{
			foregroundObjectSize = pObject->size;
			iForegroundObject = iObject;
		}
	}

	return iForegroundObject;
}

void ObjectGraph::UnionOfVertices(
	Array<int> objects,
	Array<int> &iVertexArray,
	bool *bVertexInArray)
{
	iVertexArray.n = 0;

	int k, l, iVertex;
	Object *pObject;

	for (int k = 0; k < objects.n; k++)
	{
		pObject = objectArray.Element + objects.Element[k];

		for (l = 0; l < pObject->iVertexArray.n; l++)
		{
			iVertex = pObject->iVertexArray.Element[l];

			if (bVertexInArray[iVertex])
				continue;

			iVertexArray.Element[iVertexArray.n++] = iVertex;

			bVertexInArray[iVertex] = true;
		}
	}

	for (l = 0; l < iVertexArray.n; l++)
		bVertexInArray[iVertexArray.Element[l]] = false;
}

void ObjectGraph::UnionOfSurfels(
	Array<int> objects,
	Array<int> &iSurfelArray,
	bool *bSurfelInArray)
{
	iSurfelArray.n = 0;

	int k, iSurfel;
	Object *pObject;
	QLIST::Index *piSurfel;

	for (int k = 0; k < objects.n; k++)
	{
		pObject = objectArray.Element + objects.Element[k];

		piSurfel = pObject->surfelList.pFirst;

		while (piSurfel)
		{
			if (!bSurfelInArray[piSurfel->Idx])
			{
				iSurfelArray.Element[iSurfelArray.n++] = piSurfel->Idx;

				bSurfelInArray[piSurfel->Idx] = true;
			}

			piSurfel = piSurfel->pNext;
		}
	}

	for (k = 0; k < iSurfelArray.n; k++)
		bSurfelInArray[iSurfelArray.Element[k]] = false;
}

void ObjectGraph::UnionOfObjects(
	Array<int> objects,
	Array<int> &iSurfelArray,
	Array<int> &iVertexArray,
	bool *bSurfelInArray,
	bool *bVertexInArray)
{
	UnionOfSurfels(objects, iSurfelArray, bSurfelInArray);

	UnionOfVertices(objects, iVertexArray, bVertexInArray);
}

void ObjectGraph::SurfelCells(
	std::vector<int> validObjects)
{
	// Determine the number of object-surfel relations.

	int nObjectSurfelRelations = 0;

	int i, iObject, iObject_;
	QLIST::Index *piSurfel;

	for (i = 0; i < validObjects.size(); i++)
	{
		iObject = validObjects[i];

		iObject_ = iObjectAssignedToNode[iObject];

		piSurfel = objectArray.Element[iObject_].surfelList.pFirst;

		while (piSurfel)
		{
			nObjectSurfelRelations++;

			piSurfel = piSurfel->pNext;
		}
	}

	// Assign objects to every surfel.

	QList<QLIST::Index> *surfelObjectRelations = new QList<QLIST::Index>[pSurfels->NodeArray.n];

	RVL_DELETE_ARRAY(surfelCellObjectMem);

	surfelCellObjectMem = new QLIST::Index[nObjectSurfelRelations];

	QLIST::Index *pSurfelObjectRelation = surfelCellObjectMem;

	int iSurfel;
	QList<QLIST::Index> *pSurfelObjectRelationList;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfelObjectRelationList = surfelObjectRelations + iSurfel;

		RVLQLIST_INIT(pSurfelObjectRelationList);
	}

	int maxObjectID = -1;

	for (i = 0; i < validObjects.size(); i++)
	{
		iObject = validObjects[i];

		iObject_ = iObjectAssignedToNode[iObject];

		if (iObject_ > maxObjectID)
			maxObjectID = iObject_;

		piSurfel = objectArray.Element[iObject_].surfelList.pFirst;

		while (piSurfel)
		{
			iSurfel = piSurfel->Idx;

			pSurfelObjectRelationList = surfelObjectRelations + iSurfel;

			RVLQLIST_ADD_ENTRY(pSurfelObjectRelationList, pSurfelObjectRelation);

			pSurfelObjectRelation->Idx = iObject_;

			pSurfelObjectRelation++;

			piSurfel = piSurfel->pNext;
		}
	}

	/// Create surfel cells.

	RVL_DELETE_ARRAY(objectCells);

	objectCells = new Array<int>[maxObjectID + 1];

	RVL_DELETE_ARRAY(objectCellMem);

	objectCellMem = new int[nObjectSurfelRelations];

	int *piObjectCell = objectCellMem;

	RVL_DELETE_ARRAY(surfelCells.Element);

	surfelCells.Element = new Cell[nObjectSurfelRelations];

	Cell *pCell = surfelCells.Element;

	surfelCells.n = 0;

	RVL_DELETE_ARRAY(surfelCellSurfelMem);

	surfelCellSurfelMem = new QLIST::Index[nObjectSurfelRelations];

	QLIST::Index *piSurfelCellSurfel = surfelCellSurfelMem;

	bool *bSurfelInCell = new bool[pSurfels->NodeArray.n];

	memset(bSurfelInCell, 0, pSurfels->NodeArray.n * sizeof(bool));

	int j, iSurfel_;
	QLIST::Index *piSurfel_, *pCellObject;
	QList<QLIST::Index> *pCellSurfelList;
	Array<int> *pObjectCells, *pObjectCells_;
	QList<QLIST::Index> *pSurfelObjectRelationList_;

	for (i = 0; i < validObjects.size(); i++)
	{
		iObject = validObjects[i];

		iObject_ = iObjectAssignedToNode[iObject];

		pObjectCells = objectCells + iObject_;

		pObjectCells->Element = piObjectCell;

		pObjectCells->n = 0;

		piSurfel = objectArray.Element[iObject_].surfelList.pFirst;

		while (piSurfel)
		{
			iSurfel = piSurfel->Idx;

			if (!bSurfelInCell[iSurfel] && surfelObjectRelations[iSurfel].pFirst)		// if surfel belongs to at least one object 
																						// and it is not already assigned to a cell
			{
				pSurfelObjectRelationList = surfelObjectRelations + iSurfel;

				pCellObject = pSurfelObjectRelationList->pFirst;

				while (pCellObject)
				{
					if (pCellObject->Idx < iObject_)
						break;

					pCellObject = pCellObject->pNext;
				}

				if (pCellObject)	// if surfel iSurfel belongs to an object with smaller index than iObject_
									// (This means that the cell containing this surfel is already created.)
				{
					// pCell <- the cell containing surfel iSurfel.

					pObjectCells_ = objectCells + pCellObject->Idx;

					for (j = 0; j < pObjectCells_->n; j++)
					{
						pCell = surfelCells.Element + pObjectCells_->Element[j];

						piSurfel_ = pCell->surfelList.pFirst;

						while (piSurfel_)
						{
							if (piSurfel_->Idx == iSurfel)
								break;

							piSurfel_ = piSurfel_->pNext;
						}

						if (piSurfel_)
							break;
					}

					// Add pCell to the list of cells of object iObject_.

					*(piObjectCell++) = pObjectCells_->Element[j];

					// Set flag bSurfelInCell for all surfels belonging to pCell.

					piSurfel_ = pCell->surfelList.pFirst;

					while (piSurfel_)
					{
						bSurfelInCell[piSurfel_->Idx] = true;

						piSurfel_ = piSurfel_->pNext;
					}
				}
				else
				{
					pCell = surfelCells.Element + surfelCells.n;

					pCell->idx = surfelCells.n;

					pCell->objectList = surfelObjectRelations[iSurfel];

					pCellSurfelList = &(pCell->surfelList);

					RVLQLIST_INIT(pCellSurfelList);

					pCell->size = 0;

					*(piObjectCell++) = surfelCells.n;

					piSurfel_ = objectArray.Element[iObject_].surfelList.pFirst;

					while (piSurfel_)
					{
						iSurfel_ = piSurfel_->Idx;

						if (!bSurfelInCell[iSurfel_])
						{
							pSurfelObjectRelationList = surfelObjectRelations + iSurfel_;

							pSurfelObjectRelation = pSurfelObjectRelationList->pFirst;

							pCellObject = pCell->objectList.pFirst;

							while (pSurfelObjectRelation && pCellObject)
							{
								if (pSurfelObjectRelation->Idx != pCellObject->Idx)
									break;

								pSurfelObjectRelation = pSurfelObjectRelation->pNext;

								pCellObject = pCellObject->pNext;
							}

							if (pSurfelObjectRelation == NULL && pCellObject == NULL)
							{
								piSurfelCellSurfel->Idx = iSurfel_;

								RVLQLIST_ADD_ENTRY(pCellSurfelList, piSurfelCellSurfel);

								piSurfelCellSurfel++;

								bSurfelInCell[iSurfel_] = true;

								pCell->size += pSurfels->NodeArray.Element[iSurfel_].size;
							}
						}

						piSurfel_ = piSurfel_->pNext;
					}

					surfelCells.n++;
				}
			}	// if surfel belongs to at least one object and it is not already assigned to a cell

			piSurfel = piSurfel->pNext;
		}	// for every surfel belonging to iObject_

		pObjectCells->n = piObjectCell - pObjectCells->Element;

		piSurfel = objectArray.Element[iObject_].surfelList.pFirst;

		while (piSurfel)
		{
			bSurfelInCell[piSurfel->Idx] = false;

			piSurfel = piSurfel->pNext;
		}
	}	// for every object in validObjects

	delete[] bSurfelInCell;
	delete[] surfelObjectRelations;

	///

	// Debug

	//FILE *fp = fopen("surfelCells.txt", "w");

	//for (i = 0; i < validObjects.size(); i++)
	//{
	//	iObject = validObjects[i];

	//	iObject_ = iObjectAssignedToNode[iObject];

	//	fprintf(fp, "Object %d:\n", iObject_);

	//	piSurfel = objectArray.Element[iObject_].surfelList.pFirst;

	//	while (piSurfel)
	//	{
	//		fprintf(fp, "%d ", piSurfel->Idx);

	//		piSurfel = piSurfel->pNext;
	//	}	// for every surfel belonging to iObject_

	//	fprintf(fp, "\n\n");
	//}

	//fprintf(fp, "\n\n");

	//QLIST::Index *piObject;

	//for (i = 0; i < surfelCells.n; i++)
	//{
	//	fprintf(fp, "Cell %d\n", i);

	//	fprintf(fp, "Surfels: ");

	//	pCell = surfelCells.Element + i;

	//	piSurfel = pCell->surfelList.pFirst;

	//	while (piSurfel)
	//	{
	//		fprintf(fp, "%d ", piSurfel->Idx);

	//		piSurfel = piSurfel->pNext;
	//	}

	//	fprintf(fp, "\n");

	//	fprintf(fp, "Objects: ");

	//	piObject = pCell->objectList.pFirst;

	//	while (piObject)
	//	{
	//		fprintf(fp, "%d ", piObject->Idx);

	//		piObject = piObject->pNext;
	//	}

	//	fprintf(fp, "\n\n");
	//}

	//fclose(fp);
}

void ObjectGraph::ObjectMapMask(cv::Mat *pMask)
{
	if (!pMesh->bOrganizedPC)
		return;

	int iForegroundObject = GetForegroundObject();

	int i;
	int iObject, iSurfel, iNode;

	for (i = 0; i < pMesh->NodeArray.n; i++)
	{
		iSurfel = pSurfels->surfelMap[i];

		if (iSurfel >= 0)
		{
			iNode = objectMap[iSurfel];

			if (iNode >= 0)
			{
				iObject = iObjectAssignedToNode[iNode];

				if (iObject == iForegroundObject)
					pMask->data[i] = 255;
				else
					pMask->data[i] = 0;
			}
			else
				pMask->data[i] = 0;
		}
		else
			pMask->data[i] = 0;
	}
}

int SURFEL::ConnectedSetRG(
	int iObject,
	int iObject_,
	AgEdge *pEdge,
	ObjectGraph *pObjects,
	ConnectedSetRGData *pData)
{
	//if (iObject == 116 && iObject_ == 114 || iObject_ == 116 && iObject == 114)
	//	int debug = 0;

	if (pObjects->objectMap[iObject] >= 0)
		return 0;

	if (pEdge->desc.minDist > pData->maxDist)
		return 0;

	GRAPH::AggregateNode<AgEdge> *pObject_ = pObjects->NodeArray.Element + iObject;

	if (pObject_->size < pData->minSize)
		return 0;

	pObjects->objectMap[iObject] = pData->iRefObject;

	GRAPH::AggregateNode<AgEdge> *pObject = pObjects->NodeArray.Element + pData->iRefObject;

	QList<QLIST::Index> *pElementList = &(pObject->elementList);	

	QList<QLIST::Index> *pElementList_ = &(pObject_->elementList);

	RVLQLIST_APPEND(pElementList, pElementList_);

	RVLQLIST_INIT(pElementList_);

	pObject->size += pObject_->size;

	pObject_->size = 0;

	return 1;
}

int SURFEL::ConnectedSetRG2(
	int iObject,
	int iObject_,
	GRAPH::Edge *pEdge,
	Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> *pG,
	ConnectedSetRGData2 *pData)
{
	if (pData->bAssigned[iObject])
		return 0;

	Object *pObject_ = pData->nodeArray.Element + iObject;

	pData->bAssigned[iObject] = true;

	Object *pObject = pData->nodeArray.Element + pData->iRefObject;

	QList<QLIST::Index> *pElementList = &(pObject->surfelList);

	QList<QLIST::Index> *pElementList_ = &(pObject_->surfelList);

	QLIST::Index *pSurfelIdx = pElementList_->pFirst;

	while (pSurfelIdx)
	{
		pData->objectMap[pSurfelIdx->Idx] = pObject->iNode;

		pSurfelIdx = pSurfelIdx->pNext;
	}

	RVLQLIST_APPEND(pElementList, pElementList_);

	RVLQLIST_INIT(pElementList_);

	pObject->size += pObject_->size;

	pObject_->size = 0;

	return 1;
}

std::vector<std::vector<int>>  ObjectGraph::GetObjectBSNeighbourhood(float boundingSphereRadiusThr, bool verbose)
{
	//number of segments
	int noObjects = this->NodeArray.n;
	//create list
	std::vector<std::vector<int>> neighbourhood(noObjects);

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject2;
	QLIST::Index *piElement;
	std::vector<int> validObjects;
	int iObject;
	int iObject2;
	for (iObject = 0; iObject < this->NodeArray.n; iObject++)
	{
		pObject = this->NodeArray.Element + iObject;

		if (pObject->size < this->minObjectSize)
			continue;

		piElement = pObject->elementList.pFirst;

		//check if object
		if (!piElement)
			continue;

		// Add object to the list of valid objects.
		validObjects.push_back(iObjectAssignedToNode[iObject]);
	}

	Array2D<float> convexHullVertices;
	convexHullVertices.w = 3;
	convexHullVertices.Element = new float[3 * this->pSurfels->vertexArray.n];
	
	Array<int> iObjectArray;
	iObjectArray.Element = new int[2];
	iObjectArray.n = 2;


	Array<int> iVertexArray;
	iVertexArray.Element = new int[this->pSurfels->vertexArray.n];

	bool *bVertexInArray = new bool[this->pSurfels->vertexArray.n];
	memset(bVertexInArray, 0,this-> pSurfels->vertexArray.n * sizeof(bool));

	//Get neighboorhood for each valid object
	int noValidObjects = validObjects.size();
	float *P, *P_;
	float boundingSphereCenter[3];
	float boundingSphereRadius;
	for (int i = 0; i < noValidObjects; i++)
	{
		//First object
		iObject = validObjects[i];
		//pObject = this->NodeArray.Element + iObject;

		if (neighbourhood[iObject].size() == 0)
			neighbourhood.at(iObject).push_back(iObject); //Put main object onto its neighbourhood list in order to verify objects validity if no other objects are in vicinity
		
		iObjectArray.Element[0] = iObject;	//add to array for vertex union
		for (int j = i + 1; j < noValidObjects; j++)
		{
			//Second object
			iObject2 = validObjects[j];
			iObjectArray.Element[1] = iObject2;

			//Get vertex union
			UnionOfVertices(iObjectArray, iVertexArray, bVertexInArray);

			//Copy vertex data to Array2D
			for (int v = 0; v < iVertexArray.n; v++)
			{
				P = pSurfels->vertexArray.Element[iVertexArray.Element[v]]->P;
				P_ = convexHullVertices.Element + 3 * v;
				RVLCOPY3VECTOR(P, P_);
			}
			convexHullVertices.h = iVertexArray.n;

			//Calculate bounding sphere
			BoundingSphere<float>(convexHullVertices, boundingSphereCenter, boundingSphereRadius);

			//Check if radius is within threshold and add to neighbourhood
			if (boundingSphereRadius < boundingSphereRadiusThr)
			{			
				//if (neighbourhood[iObject].size() == 0)
				//	neighbourhood.at(iObject).push_back(iObject); //Put main object onto its neighbourhood list in order to verify objects validity if no other objects are in vicinity
				if (neighbourhood[iObject2].size() == 0)
					neighbourhood.at(iObject2).push_back(iObject2); //If second object is not added to its list, add it
				//Add to neighbourhood
				neighbourhood.at(iObject).push_back(iObject2);
				neighbourhood.at(iObject2).push_back(iObject);

				if (verbose)
					std::cout << "Objects " << i << " and " << j << " are in the neighbourhood with bounding sphere radius of: " << boundingSphereRadius << std::endl;
			}

		}
	}

	delete[] iVertexArray.Element;
	delete[] bVertexInArray;
	delete[] iObjectArray.Element;
	delete[] convexHullVertices.Element;

	return neighbourhood; //The first index in the list for all valid objects is the index of that same valid object
}

bool  ObjectGraph::CheckBoundingSphere(std::vector<int> objects, float boundingSphereRadiusThr)
{
	Array2D<float> convexHullVertices;
	convexHullVertices.w = 3;
	convexHullVertices.Element = new float[3 * this->pSurfels->vertexArray.n];

	Array<int> iObjectArray;
	iObjectArray.Element = new int[objects.size()];
	iObjectArray.n = objects.size();

	Array<int> iVertexArray;
	iVertexArray.Element = new int[this->pSurfels->vertexArray.n];

	bool *bVertexInArray = new bool[this->pSurfels->vertexArray.n];
	memset(bVertexInArray, 0, this->pSurfels->vertexArray.n * sizeof(bool));

	//Copy object list to array
	for (int i = 0; i < objects.size(); i++)
		iObjectArray.Element[i] = objects[i];

	//Get vertex union
	UnionOfVertices(iObjectArray, iVertexArray, bVertexInArray);

	//Copy vertex data to Array2D
	float *P, *P_;
	for (int v = 0; v < iVertexArray.n; v++)
	{
		P = pSurfels->vertexArray.Element[iVertexArray.Element[v]]->P;
		P_ = convexHullVertices.Element + 3 * v;
		RVLCOPY3VECTOR(P, P_);
	}
	convexHullVertices.h = iVertexArray.n;

	//Calculate bounding sphere
	float boundingSphereCenter[3];
	float boundingSphereRadius;
	BoundingSphere<float>(convexHullVertices, boundingSphereCenter, boundingSphereRadius);

	delete[] iVertexArray.Element;
	delete[] bVertexInArray;
	delete[] iObjectArray.Element;
	delete[] convexHullVertices.Element;

	//Check if radius is within threshold and add to neighbourhood
	if (boundingSphereRadius < boundingSphereRadiusThr)
		return true;
	else
		return false;
}