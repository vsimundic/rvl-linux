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
	NodeArray.n = pSurfels->vertexArray.n;

	NodeArray.Element = new Vertex[NodeArray.n];

	Vertex *pVertex = NodeArray.Element;

	int iVertex;
	QList<GRAPH::EdgePtr2<VertexEdge>> *pEdgeList;

	for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++, pVertex++)
	{
		*pVertex = *(pSurfels->vertexArray.Element[iVertex]);

		pVertex->normalHull.n = 0;

		pEdgeList = &(pVertex->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		pVertex->iSurfelArray.n = 0;
	}
}

void VertexGraph::Save(FILE *fp)
{
	fprintf(fp, "%d\t%d\t0\t0\t0\n", idx, NodeArray.n);

	Vertex *pVertex = NodeArray.Element;

	int i;

	for (i = 0; i < NodeArray.n; i++, pVertex++)
		fprintf(fp, "%f\t%f\t%f\t%d\t%d\n", pVertex->P[0], pVertex->P[1], pVertex->P[2], pVertex->type, pVertex->bEdge);
}

bool VertexGraph::Load(FILE *fp)
{
	if (fscanf(fp, "%d\t%d\t0\t0\t0\n", &idx, &NodeArray.n) < 2)
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