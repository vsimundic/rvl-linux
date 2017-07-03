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
	iVertexClusterMem = NULL;
}


VertexGraph::~VertexGraph()
{
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(EdgePtrMem);
	RVL_DELETE_ARRAY(iVertexClusterMem);
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

	// Copy edges.

	QList<SURFEL::VertexEdge> *pEdgeList_ = &edgeList;

	RVLQLIST_INIT(pEdgeList_);

	nEdges = 0;

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

			pEdge = ConnectNodes<SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>>(iVertex, iVertex_, NodeArray, pMem);

			RVLQLIST_ADD_ENTRY(pEdgeList_, pEdge);

			//RVLCOPY3VECTOR(pSurfel->N, pEdge->N);

			nEdges++;

			pEdgePtr = pEdgePtr->pNext;
		}
	}

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

void VertexGraph::Save(FILE *fp)
{
	fprintf(fp, "%d\t%d\t%d\t0\t0\n", idx, NodeArray.n, nEdges);

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
}

bool VertexGraph::Load(FILE *fp)
{
	if (fscanf(fp, "%d\t%d\t%d\t0\t0\n", &idx, &NodeArray.n, &nEdges) < 3)
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

	if (iVertex == 81 && iParentVertex == 119)
		int debug = 0;

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

	int j = 0;

	//bool bContinue = false;

	Surfel *pSurfel;
	int iSurfel_[4];

	for (i = 0; i < pVertex->iSurfelArray.n; i++)
	{
		iSurfel = pVertex->iSurfelArray.Element[i];

		if (pData->nOwners[iSurfel] > 0)
		{
			iSurfel_[j++] = iSurfel;

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
