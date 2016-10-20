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
				pEdge->idx = EdgeArray.n;
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
			}
		}
	}
}

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

	int i;
	int iSurfel, iSurfel_;
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

		piElement++;

		pEdgeList = &(pAgNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	//iSurfel = 0;
	//Runnng through surfels
	for (iSurfel = 0, adjacencyLinks_iter = adjacencyLinks.begin(); adjacencyLinks_iter != adjacencyLinks.end(); adjacencyLinks_iter++, iSurfel++)
	{
		pAgNode = NodeArray.Element + iSurfel;
		// iterator->first = key
		// iterator->second = value
		currLink = &adjacencyLinks_iter->second;
		//running through surfel links
		for (int i = 0; i < currLink->size(); i++)
		{
			//pSurfel_ = pSurfel->imgAdjacency.at(i);
			pDesc = adjacencyDescriptors.at(iDesc);// pSurfel->imgAdjacencyDescriptors.at(i);

			pEdge->iVertex[0] = adjacencyLinks_iter->first;	//surfel
			pEdge->iVertex[1] = currLink->at(i);	//other surfel
			pEdge->desc = *pDesc;
			pEdge->idx = EdgeArray.n;
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

			iDesc++;//aggr list index
		}
	}
}

void ObjectGraph::WERSegmentation()
{
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

				ComputeRelationCost(pEdge);
			}

			pEdgePtr = pEdgePtr->pNext;
		}
	}
}

void ObjectGraph::ComputeRelationCost(AgEdge *pEdge)
{
	float scale = 1000.0f;
	float depthStepIntThr = scale * 0.005f;
	float depthStepExtThr = scale * 0.025f;
	float concaveAngleThr = 45.0f * DEG2RAD;
	float concaveMinCost = 0.3f;
	float alpha = 0.8f;

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

		pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);

		piElement = piElement->pNext;
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
		pData->bObjects = !pData->bObjects;

		if (pData->bObjects)
			pObjects->Display();
		else
		{
			pSurfels->Display(pVisualizer, pMesh);

			pData->iSelectedObject = -1;
		}

		return true;
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