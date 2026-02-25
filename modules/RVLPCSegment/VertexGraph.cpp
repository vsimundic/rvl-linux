//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include <vtkVertexGlyphFilter.h>
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
	iVertexClusterMem = NULL;
	G.NodeArray.Element = NULL;
	nGEdges = 0;
	QList<GRAPH::Edge> *pGEdgeList = &GEdgeList;
	RVLQLIST_INIT(pGEdgeList);
}

//


VertexGraph::~VertexGraph()
{
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(EdgePtrMem);
	RVL_DELETE_ARRAY(iVertexClusterMem);
	RVL_DELETE_ARRAY(G.NodeArray.Element);
}

void VertexGraph::Create(SurfelGraph *pSurfels_)
{
	pSurfels = pSurfels_;

	// Create nodes.

	NodeArray.n = pSurfels->vertexArray.n;

	NodeArray.Element = new Vertex[NodeArray.n];

	Vertex *pVertex = NodeArray.Element;

	int iVertex, iVertex_;
	QList<GRAPH::EdgePtr2<VertexEdge>> *pEdgeList;
	//int i;
	//int iSurfel;
	//Surfel *pSurfel;
	//QList<QLIST::Index> *pVertexList;
	//QLIST::Index *pVertexIdx;
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

	// Copy edges.

	QList<SURFEL::VertexEdge> *pEdgeList_ = &edgeList;

	RVLQLIST_INIT(pEdgeList_);

	nEdges = 0;

#ifdef NEVER
	SURFEL::VertexEdge *pEdge;
	GRAPH::EdgePtr2<SURFEL::VertexEdge> *pEdgePtr;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		pVertex_ = pSurfels->vertexArray.Element[iVertex];

		pEdgeList = &(pVertex_->EdgeList);

		pEdgePtr = pEdgeList->pFirst;

		while (pEdgePtr)
		{
			iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			if (iVertex_ > iVertex)
			{
				pEdge = ConnectNodes<SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>>(iVertex, iVertex_, NodeArray, pMem);

				RVLQLIST_ADD_ENTRY(pEdgeList_, pEdge);

				pEdge->iSurfel[0] = pEdgePtr->pEdge->iSurfel[0];
				pEdge->iSurfel[1] = pEdgePtr->pEdge->iSurfel[1];

				//RVLCOPY3VECTOR(pSurfel->N, pEdge->N);

				nEdges++;
			}

			pEdgePtr = pEdgePtr->pNext;
		}
	}
#endif

#ifdef NEVER	// Old version: each vertex is connected to all vertices which share two common surfels.
	// Remove redundant vertices

	bool *bBelongsToRefVertex = new bool[pSurfels->NodeArray.n];

	memset(bBelongsToRefVertex, 0, pSurfels->NodeArray.n * sizeof(bool));
	
	int j, iSurfel_, nCommonSurfels, nRefVertexSurfels;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		if (pVertex->type & RVLSURFELVERTEX_TYPE_REDUNDANT)
			continue;

		nRefVertexSurfels = pVertex->iSurfelArray.n;

		for (i = 0; i < nRefVertexSurfels; i++)
		{
			iSurfel = pVertex->iSurfelArray.Element[i];

			bBelongsToRefVertex[iSurfel] = true;
		}

		//if (bBelongsToRefVertex[2] && bBelongsToRefVertex[8])
		//	int debug = 0;

		for (i = 0; i < pVertex->iSurfelArray.n; i++)
		{
			iSurfel = pVertex->iSurfelArray.Element[i];

			pVertexList = pSurfels->surfelVertexList.Element + iSurfel;

			pVertexIdx = pVertexList->pFirst;

			while (pVertexIdx)
			{
				iVertex_ = pVertexIdx->Idx;

				if (iVertex_ > iVertex)
				{
					pVertex_ = NodeArray.Element + iVertex_;

					if (!(pVertex_->type & RVLSURFELVERTEX_TYPE_REDUNDANT))
					{
						nCommonSurfels = 0;

						for (j = 0; j < pVertex_->iSurfelArray.n; j++)
						{
							iSurfel_ = pVertex_->iSurfelArray.Element[j];

							if (bBelongsToRefVertex[iSurfel_])
								nCommonSurfels++;
						}

						if (nCommonSurfels == nRefVertexSurfels && nCommonSurfels == pVertex_->iSurfelArray.n)
							pVertex_->type |= RVLSURFELVERTEX_TYPE_REDUNDANT;
					}
				}

				pVertexIdx = pVertexIdx->pNext;
			}
		}

		for (i = 0; i < nRefVertexSurfels; i++)
		{
			iSurfel = pVertex->iSurfelArray.Element[i];

			bBelongsToRefVertex[iSurfel] = false;
		}
	}	// for every vertex

	// Create edges.	

	QList<SURFEL::VertexEdge> *pEdgeList_ = &edgeList;

	RVLQLIST_INIT(pEdgeList_);

	nEdges = 0;

	bool *bAlreadyConnected = new bool[NodeArray.n];

	memset(bAlreadyConnected, 0, NodeArray.n * sizeof(bool));

	SURFEL::VertexEdge *pEdge;
	GRAPH::EdgePtr2<SURFEL::VertexEdge> *pEdgePtr;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		if (pVertex->type & RVLSURFELVERTEX_TYPE_REDUNDANT)
			continue;

		nRefVertexSurfels = pVertex->iSurfelArray.n;

		for (i = 0; i < nRefVertexSurfels; i++)
		{
			iSurfel = pVertex->iSurfelArray.Element[i];

			bBelongsToRefVertex[iSurfel] = true;
		}

		if (bBelongsToRefVertex[2] && bBelongsToRefVertex[8])
			int debug = 0;

		for (i = 0; i < pVertex->iSurfelArray.n; i++)
		{
			iSurfel = pVertex->iSurfelArray.Element[i];

			pVertexList = pSurfels->surfelVertexList.Element + iSurfel;

			pVertexIdx = pVertexList->pFirst;

			while (pVertexIdx)
			{
				iVertex_ = pVertexIdx->Idx;

				if (!bAlreadyConnected[iVertex_])
				{
					pVertex_ = NodeArray.Element + iVertex_;

					if (!(pVertex_->type & RVLSURFELVERTEX_TYPE_REDUNDANT))
					{
						nCommonSurfels = 0;

						for (j = 0; j < pVertex_->iSurfelArray.n; j++)
						{
							iSurfel_ = pVertex_->iSurfelArray.Element[j];

							if (bBelongsToRefVertex[iSurfel_])
								nCommonSurfels++;
						}

						if (nCommonSurfels == 2 && iVertex < iVertex_)
						{
							pEdge = ConnectNodes<SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>>(iVertex, pVertexIdx->Idx,
								NodeArray, pMem);

							RVLQLIST_ADD_ENTRY(pEdgeList_, pEdge);

							//RVLCOPY3VECTOR(pSurfel->N, pEdge->N);

							nEdges++;

							bAlreadyConnected[iVertex_] = true;
						}
					}
				}

				pVertexIdx = pVertexIdx->pNext;
			}
		}

		for (i = 0; i < nRefVertexSurfels; i++)
		{
			iSurfel = pVertex->iSurfelArray.Element[i];

			bBelongsToRefVertex[iSurfel] = false;
		}

		pEdgePtr = pVertex->EdgeList.pFirst;

		while (pEdgePtr)
		{
			iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			bAlreadyConnected[iVertex_] = false;

			pEdgePtr = pEdgePtr->pNext;
		}
	}	// for every vertex

	delete[] bAlreadyConnected;
	delete[] bBelongsToRefVertex;
#endif		// Old version: each vertex is connected to all vertices which share two common surfels.

#ifdef NEVER		// Even older version: each vertex is connected with all vertices sharing a common surfel.

	//bool *bAlreadyConnected = new bool[NodeArray.n];

	//memset(bAlreadyConnected, 0, NodeArray.n * sizeof(bool));

	QList<SURFEL::VertexEdge> *pEdgeList_ = &edgeList;

	RVLQLIST_INIT(pEdgeList_);

	nEdges = 0;

	SURFEL::VertexEdge *pEdge;
	GRAPH::EdgePtr2<SURFEL::VertexEdge> *pEdgePtr;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		//if (iVertex == 217)
		//	int debug = 0;

		for (i = 0; i < pVertex->iSurfelArray.n; i++)
		{
			iSurfel = pVertex->iSurfelArray.Element[i];

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			pVertexList = pSurfels->surfelVertexList.Element + iSurfel;

			pVertexIdx = pVertexList->pFirst;

			while (pVertexIdx)
			{
				//if (!bAlreadyConnected[pVertexIdx->Idx])
				{
					if (iVertex < pVertexIdx->Idx)
					{
						pEdge = ConnectNodes<SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>>(iVertex, pVertexIdx->Idx,
							NodeArray, pMem);

						RVLQLIST_ADD_ENTRY(pEdgeList_, pEdge);

						RVLCOPY3VECTOR(pSurfel->N, pEdge->N);

						nEdges++;
					}

					//bAlreadyConnected[pVertexIdx->Idx] = true;
				}

				pVertexIdx = pVertexIdx->pNext;
			}
		}

		//pEdgePtr = pVertex->EdgeList.pFirst;

		//while (pEdgePtr)
		//{
		//	iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

		//	bAlreadyConnected[iVertex_] = false;

		//	pEdgePtr = pEdgePtr->pNext;
		//}
	}

	//delete[] bAlreadyConnected;
#endif
}

//#ifdef NEVER		// New version

void VertexGraph::Clustering()
{
	float normalNoiseDeg = 20.0f;	// deg

	float normalNoise = normalNoiseDeg * DEG2RAD;

	// Detect tangent vertices.

	//float normalNoise_ = 0.99 * normalNoise;

	float snqThr = sin(normalNoise);

	int iVertex;
	Vertex *pVertex;
	float minsnq;
	int i, j;
	bool bTangent;
	float *N, *Nh;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		if (pVertex->normalHull.n < 3)
			continue;

		minsnq = pVertex->normalHull.Element[0].snq;

		for (i = 1; i < pVertex->normalHull.n; i++)
			if (pVertex->normalHull.Element[i].snq < minsnq)
				minsnq = pVertex->normalHull.Element[i].snq;

		if (minsnq < snqThr)
			continue;

		bTangent = false;

		for (i = 0; i < pVertex->normalHull.n && !bTangent; i++)
		{
			Nh = pVertex->normalHull.Element[i].Nh;

			for (j = 0; j < pVertex->normalHull.n - 2; j++)
			{
				N = pVertex->normalHull.Element[(i + 2 + j) % pVertex->normalHull.n].N;

				if (RVLDOTPRODUCT3(Nh, N) < -snqThr)
				{
					bTangent = true;

					break;
				}
			}
		}

		if (bTangent)
			pVertex->type |= RVLSURFELVERTEX_TYPE_TANGENT;
	}

	// Create graph G.

	G.NodeArray.Element = new GRAPH::Node[NodeArray.n];
	G.NodeArray.n = NodeArray.n;

	GRAPH::Node *pNode2;
	QList<GRAPH::EdgePtr<GRAPH::Edge>> *pEdge2List;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pNode2 = G.NodeArray.Element + iVertex;

		pNode2->idx = iVertex;

		pEdge2List = &(pNode2->EdgeList);

		RVLQLIST_INIT(pEdge2List);
	}

	VertexConnectRGData RGData2;	

	RGData2.thr1 = sin(normalNoise);
	RGData2.thr2 = cos(normalNoise);

	RGData2.bVisited = new bool[NodeArray.n];

	memset(RGData2.bVisited, 0, NodeArray.n * sizeof(bool));

	RGData2.visitedNodeArray.Element = new int[NodeArray.n];

	RGData2.pVertexGraph2 = &G;
	RGData2.pMem = pMem;

	nGEdges = 0;

	QList<GRAPH::Edge> *pGEdgeList = &(GEdgeList);

	RVLQLIST_INIT(pGEdgeList);

	int *iNodeBuff = new int[NodeArray.n];

	NormalHullElement *pNormaHullElement;
	int *piNodeBuffEnd, *piNodeFetch, *piNodePut;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		//if (iVertex == 276)
		//	int debug = 0;

		pVertex = NodeArray.Element + iVertex;

		if (!(pVertex->type & RVLSURFELVERTEX_TYPE_TANGENT))
			continue;

		iNodeBuff[0] = iVertex;

		RGData2.iRefVertex = iVertex;
		RGData2.bVisited[iVertex] = true;

		for (i = 0; i < pVertex->normalHull.n; i++)
		{
			pNormaHullElement = pVertex->normalHull.Element + i;

			RGData2.Z = pNormaHullElement->Nh;

			RGData2.X1 = pNormaHullElement->N;

			pNormaHullElement = pVertex->normalHull.Element + (i + 1) % pVertex->normalHull.n;

			RGData2.X2 = pNormaHullElement->N;

			/// Only for debugging purpose!!!

			//float V3Tmp[3];

			//RVLCROSSPRODUCT3(RGData2.X2, RGData2.X1, V3Tmp);

			//float fTmp;

			//RVLNORM3(V3Tmp, fTmp);

			//fTmp = RVLDOTPRODUCT3(RGData2.Z, pVertex->normalHull.Element[(i + pVertex->normalHull.n - 1) % pVertex->normalHull.n].N);

			///

			RVLCROSSPRODUCT3(RGData2.X1, RGData2.Z, RGData2.Y1);
			RVLCROSSPRODUCT3(RGData2.Z, RGData2.X2, RGData2.Y2);

			RGData2.visitedNodeArray.n = 0;

			piNodeFetch = iNodeBuff;

			piNodePut = iNodeBuff + 1;

			piNodeBuffEnd = RegionGrowing<VertexGraph, SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>,
				VertexConnectRGData, ConnectNodesRG2>(this, &RGData2, piNodeFetch, piNodePut);

			for (j = 0; j < RGData2.visitedNodeArray.n; j++)
				RGData2.bVisited[RGData2.visitedNodeArray.Element[j]] = false;
		}

		RGData2.bVisited[iVertex] = false;
	}

	delete[] iNodeBuff;
	delete[] RGData2.bVisited;
	delete[] RGData2.visitedNodeArray.Element;

	//float X1[3];
	//float Y1[3];
	//float X2[3];
	//float Y2[3];
	//float Z[3];	
	//CRVLMem *pMem;

	// Create clusters by detecting connected subgraphs of G.

	clusters.clear();

	RVL_DELETE_ARRAY(iVertexClusterMem);

	iVertexClusterMem = new int[NodeArray.n];

	int *piVertex = iVertexClusterMem;

	VertexClusterRGData3 RGData;

	RGData.pVertexGraph = this;

	VertexCluster cluster;
	int *piVertexArrayEnd, *piVertexFetch, *piVertexPut;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		if (pVertex->iCluster >= 0)
			continue;

		if ((pVertex->type & (RVLSURFELVERTEX_TYPE_REDUNDANT | RVLSURFELVERTEX_TYPE_TANGENT)) != RVLSURFELVERTEX_TYPE_TANGENT)
			continue;

		pVertex->iCluster = clusters.size();

		cluster.iVertexArray.Element = piVertex;

		piVertexFetch = piVertexPut = cluster.iVertexArray.Element;

		*(piVertexPut++) = iVertex;

		piVertexArrayEnd = RegionGrowing<Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>>,
			GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>,
			VertexClusterRGData3, ConnectNodesRG3>(&G, &RGData, piVertexFetch, piVertexPut);

		cluster.iVertexArray.n = piVertexArrayEnd - cluster.iVertexArray.Element;

		clusters.push_back(cluster);
	}

	// Free memory.

	
}

//#endif

#ifdef NEVER	// Old version.

void VertexGraph::Clustering()
{
	clusters.clear();

	RVL_DELETE_ARRAY(iVertexClusterMem);

	iVertexClusterMem = new int[NodeArray.n];

	int *piVertex = iVertexClusterMem;

	VertexClusterRGData RGData;

	RGData.nOwners = new BYTE[pSurfels->NodeArray.n];

	memset(RGData.nOwners, 0, pSurfels->NodeArray.n * sizeof(BYTE));

	int iVertex;
	Vertex *pVertex;
	VertexCluster cluster;
	int *piVertexArrayEnd, *piVertexFetch, *piVertexPut;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		if (pVertex->iCluster >= 0)
			continue;

		if (pVertex->type & RVLSURFELVERTEX_TYPE_REDUNDANT)
			continue;

		pVertex->iCluster = clusters.size();

		cluster.iVertexArray.Element = piVertex;

		piVertexFetch = piVertexPut = cluster.iVertexArray.Element;

		*(piVertexPut++) = iVertex;

		piVertexArrayEnd = RegionGrowing<VertexGraph, SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>, 
			VertexClusterRGData, ConnectNodesRG>(this, &RGData, piVertexFetch, piVertexPut);

		cluster.iVertexArray.n = piVertexArrayEnd - cluster.iVertexArray.Element;

		clusters.push_back(cluster);
	}

	delete[] RGData.nOwners;
}

#endif

void VertexGraph::Save(FILE *fp)
{
	fprintf(fp, "%d\t%d\t%d\t%d\t0\n", idx, NodeArray.n, nEdges, nGEdges);

	Vertex *pVertex = NodeArray.Element;

	int i;

	for (i = 0; i < NodeArray.n; i++, pVertex++)
		fprintf(fp, "%f\t%f\t%f\t%d\t%d\n", pVertex->P[0], pVertex->P[1], pVertex->P[2], pVertex->type, pVertex->iCluster);

	SURFEL::VertexEdge *pEdge = edgeList.pFirst;

	while (pEdge)
	{
		fprintf(fp, "%d\t%d\t%f\t%f\t%f\n", pEdge->iVertex[0], pEdge->iVertex[1], pEdge->N[0], pEdge->N[1], pEdge->N[2]);

		pEdge = pEdge->pNext;
	}

	GRAPH::Edge *pGEdge = GEdgeList.pFirst;

	while (pGEdge)
	{
		fprintf(fp, "%d\t%d\t0\t0\t0\n", pGEdge->iVertex[0], pGEdge->iVertex[1]);

		pGEdge = pGEdge->pNext;
	}
}

bool VertexGraph::Load(FILE *fp)
{
	if (fscanf(fp, "%d\t%d\t%d\t%d\t0\n", &idx, &NodeArray.n, &nEdges, &nGEdges) < 4)
		return false;

	NodeArray.Element = new Vertex[NodeArray.n];

	Vertex *pVertex = NodeArray.Element;

	int iVertex;
	QList<GRAPH::EdgePtr2<VertexEdge>> *pEdgeList;
	int type, iCluster;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++, pVertex++)
	{
		fscanf(fp, "%f\t%f\t%f\t%d\t%d\n", pVertex->P, pVertex->P + 1, pVertex->P + 2, &type, &iCluster);

		pVertex->type = (unsigned char)type;
		//pVertex->bEdge = (bEdge > 0);
		pVertex->iCluster = iCluster;

		pVertex->normalHull.n = 0;

		pEdgeList = &(pVertex->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		pVertex->iSurfelArray.n = 0;
	}

	QList<VertexEdge> *pEdgeList_ = &edgeList;

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

	QList<GRAPH::Edge> *pGEdgeList = &GEdgeList;

	RVLQLIST_INIT(pGEdgeList);

	GRAPH::Edge *pGEdge;

	for (iEdge = 0; iEdge < nGEdges; iEdge++)
	{
		fscanf(fp, "%d\t%d\t0\t0\t0\n", &iVertex, &iVertex_);

		pGEdge = ConnectNodes<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>>(iVertex, iVertex_, G.NodeArray, pMem);

		RVLQLIST_ADD_ENTRY(pGEdgeList, pGEdge);
	}

	return true;
}

bool VertexGraph::BoundingBox(Box<float> *pBox)
{
	if (NodeArray.n == 0)
		return false;

	Vertex *pVertex = NodeArray.Element;

	InitBoundingBox<float>(pBox, pVertex->P);

	int iVertex;	

	for (iVertex = 1; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		UpdateBoundingBox<float>(pBox, pVertex->P);
	}

	return true;
}

void VertexGraph::Display(Visualizer *pVisualizer)
{
	polyData = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPolyData> ptsPolyData = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> points =	vtkSmartPointer<vtkPoints>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	unsigned char color[3] = {0, 128, 255};

	int iVertex;
	Vertex *pVertex;

	for (iVertex = 0; iVertex < NodeArray.n; iVertex++)
	{
		pVertex = NodeArray.Element + iVertex;

		points->InsertNextPoint(pVertex->P);

#ifdef RVLVTK9_0
		colors->InsertNextTypedTuple(color);
#else
		colors->InsertNextTupleValue(color);
#endif
	}

	ptsPolyData->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();

	vertexFilter->SetInputData(ptsPolyData);

	vertexFilter->Update();

	polyData->ShallowCopy(vertexFilter->GetOutput());

	polyData->SetPoints(points);

	polyData->GetPointData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(polyData);

	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(10);

	pVisualizer->renderer->AddActor(actor);

	//vtkActorCollection* actorCollection = pVisualizer->renderer->GetActors();
	//actorCollection->InitTraversal();

	//for (vtkIdType i = 0; i < actorCollection->GetNumberOfItems(); i++)
	//{
	//	vtkActor* pActor_ = actorCollection->GetNextActor();

	//	vtkPolyDataMapper *pMapper_ = (vtkPolyDataMapper *)(pActor_->GetMapper());

	//	pMapper_->GetInputDataObject();
	//}
}

int SURFEL::ConnectNodesRG(
	int iVertex,
	int iParentVertex,
	VertexEdge *pEdge,
	VertexGraph *pVertexGraph,
	VertexClusterRGData *pData)
{
	Vertex *pVertex = pVertexGraph->NodeArray.Element + iVertex;

	if (pVertex->iCluster >= 0)
		return 0;

	//if (iVertex == 81 && iParentVertex == 119)
	//	int debug = 0;

	//if (!(pData->mFlags[iVertex] & 0x01))
	//	return 0;

	// Check if the angle between the normal of at least one of the two common surfels of iVertex and iParentVertex 
	// and the reference normal is <= pData->csNThr.

	Vertex *pParentVertex = pVertexGraph->NodeArray.Element + iParentVertex;

	int i, iSurfel;

	for (i = 0; i < pParentVertex->iSurfelArray.n; i++)
	{
		iSurfel = pParentVertex->iSurfelArray.Element[i];

		pData->nOwners[iSurfel] = 1;
	}	

	int nCommonSurfels = 0;

	//bool bContinue = false;

	//Surfel *pSurfel;
	int iSurfel_[4];

	for (i = 0; i < pVertex->iSurfelArray.n; i++)
	{
		iSurfel = pVertex->iSurfelArray.Element[i];

		if (pData->nOwners[iSurfel] > 0)
		{
			iSurfel_[nCommonSurfels++] = iSurfel;

			pData->nOwners[iSurfel] = 2;

			//pSurfel = pData->pSurfels->NodeArray.Element + iSurfel;

			//if (!bContinue)
			//	if (RVLDOTPRODUCT3(pSurfel->N, pData->N) >= pData->csNThr)
			//		bContinue = true;
		}
		else
			iSurfel_[2] = iSurfel;
	}

	for (i = 0; i < pParentVertex->iSurfelArray.n; i++)
	{
		iSurfel = pParentVertex->iSurfelArray.Element[i];

		if (pData->nOwners[iSurfel] == 1)
			iSurfel_[3] = iSurfel;

		pData->nOwners[iSurfel] = 0;
	}

	if (nCommonSurfels > 2)
	{
		pVertex->iCluster = pParentVertex->iCluster;

		return 1;
	}

	//if (!bContinue)
	//	return 0;

	// Check if the normal of the third surfel of iVertex is on the opposite side of the plane defined by the normals of 
	// the two common surfels of iVertex and iParentVertex w.r.t. the reference normal.

	float *N0 = pVertexGraph->pSurfels->NodeArray.Element[iSurfel_[0]].N;
	float *N1 = pVertexGraph->pSurfels->NodeArray.Element[iSurfel_[1]].N;
	float *N2 = pVertexGraph->pSurfels->NodeArray.Element[iSurfel_[2]].N;
	float *N3 = pVertexGraph->pSurfels->NodeArray.Element[iSurfel_[3]].N;

	float V[3];

	RVLCROSSPRODUCT3(N0, N1, V);

	float s1 = RVLDOTPRODUCT3(N2, V);
	float s2 = RVLDOTPRODUCT3(N3, V);

	if (s1 * s2 < 0)
	{
		pVertex->iCluster = pParentVertex->iCluster;

		return 1;
	}

	return 0;
}


int SURFEL::ConnectNodesRG2(
	int iVertex,
	int iParentVertex,
	VertexEdge *pEdge,
	VertexGraph *pVertexGraph,
	VertexConnectRGData *pData)
{
	Vertex *pVertex = pVertexGraph->NodeArray.Element + iVertex;

	if (pData->bVisited[iVertex])
		return 0;

	int iSurfel1 = pEdge->iSurfel[0];

	if (iSurfel1 < 0)
		return 0;

	int iSurfel2 = pEdge->iSurfel[1];

	if (iSurfel2 < 0)
		return 0;

	float *N1 = pVertexGraph->pSurfels->NodeArray.Element[iSurfel1].N;
	float *N2 = pVertexGraph->pSurfels->NodeArray.Element[iSurfel2].N;

	if (RVLDOTPRODUCT3(N1, N2) > pData->thr2)
		return 0;

	pData->bVisited[iVertex] = true;

	pData->visitedNodeArray.Element[pData->visitedNodeArray.n++] = iVertex;

	int nCoplanar = 0;

	bool bOpposite = false;

	int i;
	float x1, x2, y1, y2, z;
	float *N;
	float Np[3];
	float V3Tmp[3];
	float fTmp;
	GRAPH::Edge *pEdge2;

	for (i = 0; i < pVertex->normalHull.n; i++)
	{
		N = pVertex->normalHull.Element[i].N;

		z = RVLDOTPRODUCT3(N, pData->Z);

		if (z < -pData->thr1)
			return 0;
		else if (z <= pData->thr1)
		{
			//RVLSCALE3VECTOR(pData->Z, z, V3Tmp);

			//RVLDIF3VECTORS(N, V3Tmp, Np);

			//RVLNORM3(Np, fTmp);

			//y1 = RVLDOTPRODUCT3(Np, pData->Y1);

			//if (y1 < 0.0f)
			//{
			//	x1 = RVLDOTPRODUCT3(Np, pData->X1);

			//	if (x1 >= pData->thr2)
			//	{
			//		nCoplanar++;

			//		continue;
			//	}
			//	else
			//		return 0;
			//}

			//y2 = RVLDOTPRODUCT3(Np, pData->Y2);

			//if (y2 < 0.0f)
			//{
			//	x2 = RVLDOTPRODUCT3(Np, pData->X2);

			//	if (x2 >= pData->thr2)
			//	{
			//		nCoplanar++;

			//		continue;
			//	}
			//	else
			//		return 0;
			//}

			nCoplanar++;
		}
		else
			bOpposite = true;
	}	// for (i = 0; i < pVertex->normalHull.n; i++)

	if (nCoplanar == 2 && bOpposite)
	{
		if (pVertex->type & RVLSURFELVERTEX_TYPE_TANGENT)
		{
			GRAPH::Node *pNode = pData->pVertexGraph2->NodeArray.Element + iVertex;

			GRAPH::EdgePtr<GRAPH::Edge > *pEdgePtr = pNode->EdgeList.pFirst;

			while (pEdgePtr)
			{
				if (RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr) == pData->iRefVertex)
					return 0;

				pEdgePtr = pEdgePtr->pNext;
			}

			pEdge2 = ConnectNodes<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>>(pData->iRefVertex, iVertex,
				pData->pVertexGraph2->NodeArray, pData->pMem);

			QList<GRAPH::Edge> *pGEdgeList = &(pVertexGraph->GEdgeList);

			RVLQLIST_ADD_ENTRY(pGEdgeList, pEdge2);

			pVertexGraph->nGEdges++;
		}

		return 0;
	}

	return 1;
}

int SURFEL::ConnectNodesRG3(
	int iVertex,
	int iParentVertex,
	GRAPH::Edge *pEdge,
	Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> *pGraph,
	VertexClusterRGData3 *pData)
{
	Vertex *pVertex = pData->pVertexGraph->NodeArray.Element + iVertex;

	if (pVertex->iCluster >= 0)
		return 0;

	Vertex *pParentVertex = pData->pVertexGraph->NodeArray.Element + iParentVertex;

	pVertex->iCluster = pParentVertex->iCluster;

	return 1;
}