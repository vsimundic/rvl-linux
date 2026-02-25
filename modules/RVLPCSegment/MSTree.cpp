//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "MSTree.h"

using namespace RVL;
using namespace GRAPH;

MSTree::MSTree()
{
	memSize = 1000;

	T.NodeArray.n = 0;

	E = NULL;
	Q = NULL;
	bProcessed = NULL;
	T_ = NULL;
}


MSTree::~MSTree()
{
	Clear();
}

void MSTree::Init(int memSizeIn)
{
	memSize = memSizeIn;

	Clear();

	RVL_DELETE_ARRAY(T.NodeMem);
	RVL_DELETE_ARRAY(T.EdgeMem);
	RVL_DELETE_ARRAY(T.EdgePtrMem);

	T.NodeMem = new Node[memSize];
	T.NodeArray.Element = T.NodeMem;
	T.EdgeMem = new Edge[memSize];
	T.EdgePtrMem = new EdgePtr<Edge>[2 * memSize];
	E = new float *[memSize];
	for (int i = 0; i < memSize; i++)
		E[i] = NULL;
	Q = new UpdateMSTreeInsertData[memSize];
	bProcessed = new bool[memSize];
	T_ = new Pair<int, int>[memSize];
}

void MSTree::Clear()
{
	int i;

	if (E)
		for (i = 1; i < T.NodeArray.n; i++)
			RVL_DELETE_ARRAY(E[i]);
	RVL_DELETE_ARRAY(E);
	RVL_DELETE_ARRAY(Q);
	RVL_DELETE_ARRAY(bProcessed);
	RVL_DELETE_ARRAY(T_);
	T.NodeArray.n = 0;
}

bool MSTreeEdgeComparison(MSTreeEdge edge1, MSTreeEdge edge2)
{
	return edge1.cost < edge2.cost;
}

void MSTree::Create(
	Array<MSTreeEdge> edges,
	Array<MSTreeEdge> *pSortedTreeEdges,
	float costThrIn,
	int *label)
{
	if (edges.n == 0)
		return;

	T.NodeArray.n = -1;

	int iEdge;

	for (iEdge = 0; iEdge < edges.n; iEdge++)
	{
		if (edges.Element[iEdge].iVertex[0] > T.NodeArray.n)
			T.NodeArray.n = edges.Element[iEdge].iVertex[0];

		if (edges.Element[iEdge].iVertex[1] > T.NodeArray.n)
			T.NodeArray.n = edges.Element[iEdge].iVertex[1];
	}

	T.NodeArray.n++;

	if (T.NodeArray.n == 0)
		return;

	RVL_DELETE_ARRAY(T.NodeMem);
	T.NodeMem = new Node[T.NodeArray.n];
	T.NodeArray.Element = T.NodeMem;
	RVL_DELETE_ARRAY(T.EdgeMem);
	T.EdgeMem = new Edge[T.NodeArray.n - 1];
	RVL_DELETE_ARRAY(T.EdgePtrMem);
	T.EdgePtrMem = new EdgePtr<Edge>[2 * (T.NodeArray.n - 1)];

	Edge *pEdge = T.EdgeMem;
	EdgePtr<Edge> *pEdgePtr = T.EdgePtrMem;

	int iNode;
	Node *pNode;
	QList<EdgePtr<Edge>> *pEdgeList;

	for (iNode = 0; iNode < T.NodeArray.n; iNode++)
	{
		pNode = T.NodeArray.Element + iNode;

		pEdgeList = &(pNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	int *iCluster = new int[T.NodeArray.n];

	for (iNode = 0; iNode < T.NodeArray.n; iNode++)
		iCluster[iNode] = iNode;

	std::vector<MSTreeEdge> sortedEdges(edges.Element, edges.Element + edges.n);

	std::sort(sortedEdges.begin(), sortedEdges.end(), MSTreeEdgeComparison);

	float costThr = (costThrIn > 0.0f ? costThrIn : sortedEdges[edges.n - 1].cost);

	int *buff = new int[T.NodeArray.n];

	if (pSortedTreeEdges)
		pSortedTreeEdges->n = 0;

	int *pFetch, *pPush;
	MSTreeEdge inEdge;
	int iCluster1, iCluster2, iRefCluster, iSeedNode, iNode_, label1, label2, refLabel;
	EdgePtr<Edge> *pEdgePtr_;

	for (iEdge = 0; iEdge < edges.n; iEdge++)
	{
		inEdge = sortedEdges[iEdge];

		if (inEdge.cost > costThr)
			break;

		iCluster1 = iCluster[inEdge.iVertex[0]];
		iCluster2 = iCluster[inEdge.iVertex[1]];

		if (iCluster1 == iCluster2)
			continue;

		if (label)
		{
			label1 = label[inEdge.iVertex[0]];
			label2 = label[inEdge.iVertex[1]];

			if (label1 >= 0 && label2 >= 0)
				continue;

			if (label1 >= 0 || label2 >= 0)
			{
				if (label1 >= 0)
				{
					refLabel = label1;
					iSeedNode = inEdge.iVertex[1];
				}
				else
				{
					refLabel = label2;
					iSeedNode = inEdge.iVertex[0];
				}

				pFetch = pPush = buff;

				label[iSeedNode] = refLabel;

				*(pPush++) = iSeedNode;

				while (pFetch < pPush)
				{
					iNode = *(pFetch++);

					pNode = T.NodeArray.Element + iNode;

					pEdgePtr_ = pNode->EdgeList.pFirst;

					while (pEdgePtr_)
					{
						iNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						if (label[iNode_] < 0)
						{
							label[iNode_] = refLabel;

							*(pPush++) = iNode_;
						}

						pEdgePtr_ = pEdgePtr_->pNext;
					}
				}
			}
		}

		if (pSortedTreeEdges)
			pSortedTreeEdges->Element[pSortedTreeEdges->n++] = inEdge;

		if (iCluster1 < iCluster2)
		{
			iRefCluster = iCluster1;
			iSeedNode = inEdge.iVertex[1];
		}
		else
		{
			iRefCluster = iCluster2;
			iSeedNode = inEdge.iVertex[0];
		}

		pFetch = pPush = buff;

		iCluster[iSeedNode] = iRefCluster;

		*(pPush++) = iSeedNode;

		while (pFetch < pPush)
		{
			iNode = *(pFetch++);

			pNode = T.NodeArray.Element + iNode;

			pEdgePtr_ = pNode->EdgeList.pFirst;

			while (pEdgePtr_)
			{
				iNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

				if (iCluster[iNode_] != iRefCluster)
				{
					iCluster[iNode_] = iRefCluster;

					*(pPush++) = iNode_;
				}

				pEdgePtr_ = pEdgePtr_->pNext;
			}
		}

		ConnectNodes<Node, Edge, EdgePtr<Edge>>(inEdge.iVertex[0], inEdge.iVertex[1], T.NodeArray, pEdge, pEdgePtr);

		pEdge++;
		pEdgePtr += 2;
	}

	delete[] iCluster;
	delete[] buff;
}

// Function UpdateMSTree is an implementation of the method proposed in chin_JCSS78.

void MSTree::Update(float *ENew)
{
	Node *pNode;
	QList<EdgePtr<Edge>> *pEdgeList;

	if (T.NodeArray.n == 0)
	{
		T.NodeArray.n = 1;

		pNode = T.NodeArray.Element;

		pNode->idx = 0;

		pEdgeList = &(pNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		return;
	}

	if (ENew == NULL)
	{
		T.NodeArray.Element[T.NodeArray.n].idx = T.NodeArray.n;

		pNode = T.NodeArray.Element + T.NodeArray.n;

		pEdgeList = &(pNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		T.NodeArray.n++;

		return;
	}

	// Add ENew to E.

	bool bMemReallocation;
	Edge *newEdgeMem;
	EdgePtr<Edge> *newEdgePtrMem;

	if (bMemReallocation = (T.NodeArray.n == memSize))
	{
		int newMemSize = memSize * 2;

		float **newEBlock = new float *[newMemSize];

		memcpy(newEBlock, E, memSize * sizeof(float *));

		delete[] E;

		E = newEBlock;

		delete[] Q;

		Q = new UpdateMSTreeInsertData[newMemSize];

		delete[] bProcessed;

		bProcessed = new bool[newMemSize];

		delete[] T_;

		T_ = new Pair<int, int>[newMemSize];

		newEdgeMem = new Edge[newMemSize];

		newEdgePtrMem = new EdgePtr<Edge>[2 * newMemSize];

		memSize = newMemSize;
	}

	E[T.NodeArray.n] = ENew;

	// Compute new minimum spanning tree T_.

	memset(bProcessed, 0, (T.NodeArray.n + 1) * sizeof(bool));

	UpdateMSTreeInsertData *pPut = Q;

	pPut->q = NULL;
	pPut->r = 0;
	pPut->state = 0;

	pPut++;

	UpdateMSTreeInsertData *pProcess = Q;

	int nT_ = 0;

	int z = T.NodeArray.n;

	int r, w;
	Pair<int, int> t, k, h;
	float ct, cwr, ck, cm;
	bool bInsert;

	while (pProcess)
	{
		r = pProcess->r;

		if (pProcess->state == 0)
		{
			bProcessed[r] = true;

			pProcess->m.a = r;
			pProcess->m.b = z;

			pProcess->pEdgePtr = T.NodeArray.Element[r].EdgeList.pFirst;
		}
		else
		{
			w = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pProcess->pEdgePtr);

			ct = (t.a > t.b ? E[t.a][t.b] : E[t.b][t.a]);
			cwr = (w > r ? E[w][r] : E[r][w]);

			if (ct > cwr)
			{
				k = t;
				h.a = w;
				h.b = r;
				ck = ct;
			}
			else
			{
				k.a = w;
				k.b = r;
				h = t;
				ck = cwr;
			}

			T_[nT_++] = h;

			cm = (pProcess->m.a > pProcess->m.b ? E[pProcess->m.a][pProcess->m.b] : E[pProcess->m.b][pProcess->m.a]);

			if (ck < cm)
				pProcess->m = k;

			pProcess->pEdgePtr = pProcess->pEdgePtr->pNext;
		}

		bInsert = false;

		while (pProcess->pEdgePtr)
		{
			w = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pProcess->pEdgePtr);

			if (!bProcessed[w])
			{
				pProcess->state = 1;

				pPut->q = pProcess;
				pPut->r = w;
				pPut->state = 0;

				pProcess = pPut;

				pPut++;

				bInsert = true;

				break;
			}

			pProcess->pEdgePtr = pProcess->pEdgePtr->pNext;
		}

		if (!bInsert)
		{
			t = pProcess->m;

			pProcess = pProcess->q;
		}
	}

	T_[nT_++] = t;

	T.NodeArray.Element[T.NodeArray.n].idx = T.NodeArray.n;

	T.NodeArray.n++;

	// Copy T_ to T.

	if (bMemReallocation)
	{
		delete[] T.EdgeMem;

		T.EdgeMem = newEdgeMem;

		delete[] T.EdgePtrMem;

		T.EdgePtrMem = newEdgePtrMem;
	}

	int i;

	for (i = 0; i < T.NodeArray.n; i++)
	{
		pNode = T.NodeArray.Element + i;

		pEdgeList = &(pNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	Edge *pEdge = T.EdgeMem;
	EdgePtr<Edge> *pEdgePtr = T.EdgePtrMem;

	for (i = 0; i < nT_; i++)
	{
		ConnectNodes<Node, Edge, EdgePtr<Edge>>(T_[i].a, T_[i].b, T.NodeArray, pEdge, pEdgePtr);

		pEdge++;
		pEdgePtr += 2;
	}
}

void RVL::GRAPH::MSTreeTest()
{
	float P[8][2] = {
		{ 50, 230 },
		{ 70, 150 },
		{ 10, 260 },
		{ 35, 250 },
		{ 30, 205 },
		{ 80, 210 },
		{ 30, 160 },
		{ 60, 190 } };

	int i;

	FILE *fp = fopen("C:\\RVL\\Debug\\P.txt", "w");

	for (i = 0; i < 8; i++)
		fprintf(fp, "%f\t%f\n", P[i][0], P[i][1]);

	fclose(fp);

	MSTree MST;
	int j;
	float dP[2];

	// Test MSTree basic method.

	Array<MSTreeEdge> edges;

	edges.n = 8 * 7 / 2;

	edges.Element = new MSTreeEdge[edges.n];

	MSTreeEdge *pEdge = edges.Element;

	for (i = 0; i < 8; i++)
		for (j = 0; j < i; j++)
		{
			dP[0] = P[j][0] - P[i][0];
			dP[1] = P[j][1] - P[i][1];

			pEdge->iVertex[0] = i;
			pEdge->iVertex[1] = j;
			pEdge->cost = dP[0] * dP[0] + dP[1] * dP[1];

			pEdge++;
		}

	MST.Create(edges);

	delete[] edges.Element;

	// Test MSTree incremental method.

	MST.Init();

	float *E;

	for (i = 0; i < 8; i++)
	{
		if (i > 0)
			E = new float[i];
		else
			E = NULL;

		for (j = 0; j < i; j++)
		{
			dP[0] = P[j][0] - P[i][0];
			dP[1] = P[j][1] - P[i][1];

			E[j] = dP[0] * dP[0] + dP[1] * dP[1];
		}

		MST.Update(E);
	}
}