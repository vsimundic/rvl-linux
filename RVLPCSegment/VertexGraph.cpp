//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "VertexGraph.h"

using namespace RVL;
using namespace SURFEL;

VertexGraph::VertexGraph()
{
	NodeArray.Element = NULL;
	EdgeArray.Element = NULL;
	EdgePtrMem = NULL;
}


VertexGraph::~VertexGraph()
{
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(EdgePtrMem);
}

void VertexGraph::Create(SurfelGraph *pSurfels)
{
	// Create nodes.

	NodeArray.n = pSurfels->vertexArray.n;

	NodeArray.Element = new Vertex[NodeArray.n];

	Vertex *pVertex = NodeArray.Element;

	int iVertex, iVertex_;
	QList<GRAPH::EdgePtr2<VertexEdge>> *pEdgeList;
	int i;
	int iSurfel;
	Surfel *pSurfel;
	QList<QLIST::Index> *pVertexList;
	QLIST::Index *pVertexIdx;
	Vertex *pVertex_;

	for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++, pVertex++)
	{
		pVertex_ = pSurfels->vertexArray.Element[iVertex];

		*pVertex = *pVertex_;

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, NormalHullElement, pVertex->normalHull.n, pVertex->normalHull.Element);

		memcpy(pVertex->normalHull.Element, pVertex_->normalHull.Element, pVertex->normalHull.n * sizeof(NormalHullElement));

		pEdgeList = &(pVertex->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	// Create edges.	

	bool *bAlreadyConnected = new bool[NodeArray.n];

	memset(bAlreadyConnected, 0, NodeArray.n * sizeof(bool));

	QList<SURFEL::VertexEdge> *pEdgeList_ = &edgeList;

	RVLQLIST_INIT(pEdgeList_);

	nEdges = 0;

	SURFEL::VertexEdge *pEdge;
	GRAPH::EdgePtr2<SURFEL::VertexEdge> *pEdgePtr;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		if (iVertex == 217)
			int debug = 0;

		for (i = 0; i < pVertex->iSurfelArray.n; i++)
		{
			iSurfel = pVertex->iSurfelArray.Element[i];

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			pVertexList = pSurfels->surfelVertexList.Element + iSurfel;

			pVertexIdx = pVertexList->pFirst;

			while (pVertexIdx)
			{
				if (!bAlreadyConnected[pVertexIdx->Idx])
				{
					if (iVertex < pVertexIdx->Idx)
					{
						pEdge = ConnectNodes<SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>>(iVertex, pVertexIdx->Idx,
							NodeArray, pMem);

						RVLQLIST_ADD_ENTRY(pEdgeList_, pEdge);

						RVLCOPY3VECTOR(pSurfel->N, pEdge->N);

						nEdges++;
					}

					bAlreadyConnected[pVertexIdx->Idx] = true;
				}

				pVertexIdx = pVertexIdx->pNext;
			}
		}

		pEdgePtr = pVertex->EdgeList.pFirst;

		while (pEdgePtr)
		{
			iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			bAlreadyConnected[iVertex_] = false;

			pEdgePtr = pEdgePtr->pNext;
		}
	}

	delete[] bAlreadyConnected;

}

void VertexGraph::Save(FILE *fp)
{
	fprintf(fp, "%d\t%d\t%d\t0\t0\n", idx, NodeArray.n, nEdges);

	Vertex *pVertex = NodeArray.Element;

	int i;

	for (i = 0; i < NodeArray.n; i++, pVertex++)
		fprintf(fp, "%f\t%f\t%f\t%d\t%d\n", pVertex->P[0], pVertex->P[1], pVertex->P[2], pVertex->type, pVertex->bEdge);

	SURFEL::VertexEdge *pEdge = edgeList.pFirst;

	while (pEdge)
	{
		fprintf(fp, "%d\t%d\t%f\t%f\t%f\n", pEdge->iVertex[0], pEdge->iVertex[1], pEdge->N[0], pEdge->N[1], pEdge->N[2]);

		pEdge = pEdge->pNext;
	}
}

bool VertexGraph::Load(FILE *fp)
{
	if (fscanf(fp, "%d\t%d\t%d\t0\t0\n", &idx, &NodeArray.n, &nEdges) < 3)
		return false;

	NodeArray.Element = new Vertex[NodeArray.n];

	Vertex *pVertex = NodeArray.Element;

	int iVertex;
	QList<GRAPH::EdgePtr2<VertexEdge>> *pEdgeList;
	int type, bEdge;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++, pVertex++)
	{
		fscanf(fp, "%f\t%f\t%f\t%d\t%d\n", pVertex->P, pVertex->P + 1, pVertex->P + 2, &type, &bEdge);

		pVertex->type = (unsigned char)type;
		pVertex->bEdge = (bEdge > 0);

		pVertex->normalHull.n = 0;

		pEdgeList = &(pVertex->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		pVertex->iSurfelArray.n = 0;
	}

	QList<SURFEL::VertexEdge> *pEdgeList_ = &edgeList;

	RVLQLIST_INIT(pEdgeList_);

	int iEdge, iVertex_;
	SURFEL::VertexEdge *pEdge;
	float N[3];

	for (iEdge = 0; iEdge < nEdges; iEdge++)
	{
		fscanf(fp, "%d\t%d\t%f\t%f\t%f\n", &iVertex, &iVertex_, N, N + 1, N + 2);

		pEdge = ConnectNodes<SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>>(iVertex, iVertex_, 
			NodeArray, pMem);

		RVLQLIST_ADD_ENTRY(pEdgeList_, pEdge);

		RVLCOPY3VECTOR(N, pEdge->N);
	}

	return true;
}

bool VertexGraph::BoundingBox(Box<float> *pBox)
{
	if (NodeArray.n == 0)
		return false;

	SURFEL::Vertex *pVertex = NodeArray.Element;

	InitBoundingBox<float>(pBox, pVertex->P);

	int iVertex;	

	for (iVertex = 1; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		UpdateBoundingBox<float>(pBox, pVertex->P);
	}

	return true;
}