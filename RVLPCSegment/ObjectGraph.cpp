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
	iElementMem = NULL;
	NodeArray.Element = NULL;
	EdgeArray.Element = NULL;
	EdgePtrMem = NULL;
}


ObjectGraph::~ObjectGraph()
{
	RVL_DELETE_ARRAY(iElementMem);
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(EdgePtrMem);
}

void ObjectGraph::Create(SurfelGraph *pSurfels)
{
	RVL_DELETE_ARRAY(NodeArray.Element);
	NodeArray.Element = new GRAPH::AggregateNode<AgEdge>[pSurfels->NodeArray.n];
	NodeArray.n = pSurfels->NodeArray.n;
	RVL_DELETE_ARRAY(EdgeArray.Element);
	EdgeArray.Element = new AgEdge[pSurfels->nImageAdjacencyRelations];
	EdgeArray.n = pSurfels->nImageAdjacencyRelations;
	RVL_DELETE_ARRAY(EdgePtrMem);
	EdgePtrMem = new GRAPH::EdgePtr<AgEdge>[2 * EdgeArray.n];
	RVL_DELETE_ARRAY(iElementMem);
	iElementMem = new int[pSurfels->NodeArray.n];

	int *piElement = iElementMem;

	GRAPH::EdgePtr<AgEdge> *pEdgePtr = EdgePtrMem;

	AgEdge *pEdge = EdgeArray.Element;

	int i;
	int iSurfel, iSurfel_;
	Surfel *pSurfel, *pSurfel_;
	GRAPH::AggregateNode<AgEdge> *pAgNode, *pAgNode_;
	QList<GRAPH::EdgePtr<AgEdge>> *pEdgeList, *pEdgeList_;
	SurfelAdjecencyDescriptors *pDesc;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pAgNode = NodeArray.Element + iSurfel;

		pAgNode->iElementArray.Element = piElement;
		pAgNode->iElementArray.Element[0] = iSurfel;
		pAgNode->iElementArray.n = 1;
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
				RVLQLIST_ADD_ENTRY(pEdgeList, pEdgePtr);
				pEdge->pVertexEdgePtr[0] = pEdgePtr;
				pEdgePtr++;
				pEdgePtr->pEdge = pEdge;
				pAgNode_ = NodeArray.Element + iSurfel_;
				pEdgeList_ = &(pAgNode_->EdgeList);
				RVLQLIST_ADD_ENTRY(pEdgeList_, pEdgePtr);
				pEdge->pVertexEdgePtr[1] = pEdgePtr;
				pEdgePtr++;
			}
		}
	}
}
