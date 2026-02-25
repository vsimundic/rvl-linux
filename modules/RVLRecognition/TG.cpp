//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif

#define RVLTG_DEBUG

// Macro RVLQLIST_INSERT_ENTRY2_ should be moved to RVLQListArray.h.

#define RVLQLIST_INSERT_ENTRY2_(ppEntry, pNewEntry)\
{\
	pNewEntry->pNext = (*ppEntry);\
	*ppEntry = pNewEntry;\
	pNewEntry->pPtrToThis = ppEntry;\
	if(pNewEntry->pNext)\
		pNewEntry->pNext->pPtrToThis = &(pNewEntry->pNext);\
}

namespace RVL
{
	// Function Dijkstra() should be moved to Graph.h.

	// Function Dijkstra() requires an array iNodeMap of data structures of type QLIST::TreeIndex2<CostType>,
	// which is initialized so that idx field of each element contains the index of this element in the array iNodeMap
	// and cost field is negative except for the elements contained in piNodeQueue, for which cost is 0.
	// This function is never proven to work!

	template<typename GraphType, typename NodeType, typename EdgeType, typename EdgePtrType, typename DataType, typename CostType,
		CostType(*f)(int, int, EdgeType *, GraphType *, DataType *, bool &)>
	bool Dijkstra(
		GraphType *pGraph, 
		DataType *pData, 
		QList<QLIST::TreeIndex2<CostType>> *piNodeQueue,
		QLIST::TreeIndex2<CostType> *iNodeMap)
	{
		int iNode, iNode_;
		EdgeType *pEdge;
		EdgePtrType *pEdgePtr;
		NodeType *pNode;
		CostType cost;
		QLIST::TreeIndex2<CostType> *piNode, *piNode_, *piNode__;
		QLIST::TreeIndex2<CostType> **ppiNode__;
		bool bInsertNodeInQueue, bGoalReached;

		while (piNodeQueue->pFirst)
		{
			iNode = piNodeQueue->pFirst->Idx;
			
			pNode = pGraph->NodeArray.Element + iNode;

			piNode = piNodeQueue->pFirst;

			RVLQLIST_REMOVE_ENTRY2(piNodeQueue, piNode, QLIST::TreeIndex2<CostType>);

			pEdgePtr = pNode->EdgeList.pFirst;

			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);

				cost = f(iNode_, iNode, pEdge, pGraph, pData, bGoalReached);

				if (bGoalReached)
					return true;

				if (cost >= 0)
				{
					bInsertNodeInQueue = true;

					piNode_ = iNodeMap + iNode_;

					if (piNode_->cost >= 0)
					{
						if (piNode_->cost > cost)
							RVLQLIST_REMOVE_ENTRY2(piNodeQueue, piNode_, QLIST::TreeIndex2<CostType>)
						else
							bInsertNodeInQueue = false;
					}

					if (bInsertNodeInQueue)
					{
						piNode_->cost = cost;

						piNode_->vpEdge = pEdge;

						ppiNode__ = &(piNodeQueue->pFirst);

						piNode__ = piNodeQueue->pFirst;

						while (piNode__)
						{
							if (piNode__->cost > cost)
								break;

							ppiNode__ = &(piNode__->pNext);

							piNode__ = *ppiNode__;
						}

						RVLQLIST_INSERT_ENTRY2_(ppiNode__, piNode_);
					}
				}

				pEdgePtr = pEdgePtr->pNext;
			}	// for every neighborint node
		}	// region growing loop

		return false;
	}
}

using namespace RVL;
using namespace RECOG;

TG::TG()
{
	NodeArray.Element = NULL;
}


TG::~TG()
{
	RVL_DELETE_ARRAY(NodeArray.Element);
}

void RECOG::RotateTemplate(
	Array2D<float> A,
	float *R,
	float *A_)
{
	float *A__ = A.Element;

	int nT = A.h;

	int i;
	float *a__, *a_;

	for (i = 0; i < nT; i++)
	{
		a__ = A__ + 3 * i;
		a_ = A_ + 3 * i;

		RVLMULMX3X3VECT(R, a__, a_);
	}
}

void TG::Create(
	VertexGraph *pVertexGraph,
	Array<int> iVertexArray,
	float *RIn,
	float *tIn,
	void *vpSet,
	SurfelGraph *pSurfels,
	bool bForceMaxdNodes)
{
	TGSet *pSet = (TGSet *)vpSet;

	CRVLMem *pMem = pSet->pMem;

	// Copy rotation matrix and translation vector.

	RVLCOPYMX3X3(RIn, R);
	RVLCOPY3VECTOR(tIn, t);

	// A_ <- A * R'

	float *A_ = new float[3 * A.h];

	RotateTemplate(A, R, A_);

	// Create nodes and descriptor.

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, QList<QLIST::Ptr<TGNode>>, A.h, descriptor.Element);

	float maxd = 0.0f;

	int i, j;
	SURFEL::Vertex *pVertex;
	float dist;
	float *N;
	TGNode *pNode, *pNode_;
	QList<QLIST::Ptr<TGNode>> *pDescriptorBin;
	QLIST::Ptr<TGNode> *pNodePtr, *pNodePtr_, *pNodePtr__;
	float d;
	int iVertex;
	int iMaxdVertex;
	bool bMaxdNodeAdded;

	for (i = 0; i < A.h; i++)
	{
		N = A_ + 3 * i;

		iMaxdVertex = -1;

		pDescriptorBin = descriptor.Element + i;

		RVLQLIST_INIT(pDescriptorBin);

		for (j = 0; j < iVertexArray.n; j++)
		{
			iVertex = iVertexArray.Element[j];

			//if (iVertex == 71)
			//	int debug = 0;

			pVertex = pVertexGraph->NodeArray.Element + iVertex;

			if (pVertex->type & RVLSURFELVERTEX_TYPE_REDUNDANT)
				continue;

			d = RVLDOTPRODUCT3(N, pVertex->P);

			if (bForceMaxdNodes)
			{
				if (iMaxdVertex < 0 || d > maxd)
				{
					iMaxdVertex = iVertex;
					maxd = d;
					bMaxdNodeAdded = false;
				}
			}

			if (pVertex->normalHull.n < 3)
				continue;

			dist = pSurfels->DistanceFromNormalHull(pVertex->normalHull, N);

			if (dist > 0.0f)
				continue;			

			pNodePtr_ = pDescriptorBin->pFirst;

			while (pNodePtr_)
			{
				if (d > pNodePtr_->ptr->d)
					break;

				pNodePtr__ = pNodePtr_;

				pNodePtr_ = pNodePtr_->pNext;
			}	

			RVLMEM_ALLOC_STRUCT(pMem, QLIST::Ptr<TGNode>, pNodePtr);

			RVLMEM_ALLOC_STRUCT(pMem, TGNode, pNode);		// In order to optimize memory consuption, this should be allocated in a tempmorary memory.

			pNodePtr->ptr = pNode;

			RVLQLIST_INSERT_ENTRY(pDescriptorBin, pNodePtr__, pNodePtr_, pNodePtr);

			pNode->d = d;
			pNode->i = i;
			pNode->iVertex = iVertex;

			if (iVertex == iMaxdVertex)
				bMaxdNodeAdded = true;
		}
		
		if (bForceMaxdNodes && !bMaxdNodeAdded)
		{			
			RVLMEM_ALLOC_STRUCT(pMem, QLIST::Ptr<TGNode>, pNodePtr);

			RVLMEM_ALLOC_STRUCT(pMem, TGNode, pNode);		// In order to optimize memory consuption, this should be allocated in a tempmorary memory.

			pNodePtr->ptr = pNode;

			pNodePtr_ = pDescriptorBin->pFirst;

			pDescriptorBin->pFirst = pNodePtr;

			pNodePtr->pNext = pNodePtr_;

			pNode->d = maxd;
			pNode->i = i;
			pNode->iVertex = iMaxdVertex;
		}
	}

	// Remove similar nodes.

	NodeArray.n = 0;

	QLIST::Ptr<TGNode> **ppNodePtr;

	for (i = 0; i < A.h; i++)
	{
		pDescriptorBin = descriptor.Element + i;

		j = 0;

		ppNodePtr = &(pDescriptorBin->pFirst);

		pNodePtr = pDescriptorBin->pFirst;

		if (pNodePtr)
		{
			pNode = pNodePtr->ptr;

			d = pNode->d + 2.0f * pSet->nodeSimilarityThr;

			while (pNodePtr)
			{
				pNode = pNodePtr->ptr;

				if (d - pNode->d < pSet->nodeSimilarityThr)
					RVLQLIST_REMOVE_ENTRY(pDescriptorBin, pNodePtr, ppNodePtr)
				else
				{
					d = pNode->d;

					pNode->j = j;

					j++;

					NodeArray.n++;

					ppNodePtr = &(pNodePtr->pNext);
				}

				pNodePtr = *ppNodePtr;
			}
		}
	}

	// Copy nodes to NodeArray.

	RVL_DELETE_ARRAY(NodeArray.Element);

	NodeArray.Element = new TGNode[NodeArray.n];

	NodeArray.n = 0;

	for (i = 0; i < A.h; i++)
	{
		pDescriptorBin = descriptor.Element + i;

		pNodePtr = pDescriptorBin->pFirst;

		while (pNodePtr)
		{
			pNode = pNodePtr->ptr;

			NodeArray.Element[NodeArray.n++] = *pNode;

			pNodePtr = pNodePtr->pNext;
		}
	}

	int iNode;
	QList<GRAPH::EdgePtr2<TGEdge>> *pEdgeList;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pEdgeList = &(pNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	nEdges = 0;

#ifdef RVLTG_EDGES
	// Connect neighboring nodes with edges.

	TGConnectNodesRGData RGData;

	RGData.mFlags = new BYTE[pVertexGraph->NodeArray.n];	// 0x01 - vertex in iVertexArray
															// 0x02 - vertex in vertexBuff

	memset(RGData.mFlags, 0, pVertexGraph->NodeArray.n);

	QList<QLIST::Index> *iVertexTGNodeList;		// a list which assigns TG nodes to each vertex in pVertexGraph

	iVertexTGNodeList = new QList<QLIST::Index>[pVertexGraph->NodeArray.n];

	QList<QLIST::TreeIndex2<float>> iNodeQueue;

	QList<QLIST::TreeIndex2<float>> *piNodeQueue = &iNodeQueue;

	RGData.iNodeMap = new QLIST::TreeIndex2<float>[pVertexGraph->NodeArray.n];

	QList<QLIST::Index> *piVertexTGNodeList;

	for (i = 0; i < iVertexArray.n; i++)
	{
		iVertex = iVertexArray.Element[i];

		pVertex = pVertexGraph->NodeArray.Element + iVertex;

		if (pVertex->type & RVLSURFELVERTEX_TYPE_REDUNDANT)
			continue;

		RGData.mFlags[iVertex] = 0x01;

		piVertexTGNodeList = iVertexTGNodeList + iVertex;

		RVLQLIST_INIT(piVertexTGNodeList);

		RGData.iNodeMap[iVertex].cost = -1.0f;
		RGData.iNodeMap[iVertex].Idx = iVertex;
	}
		
	QLIST::Index *iVertexTGNodeMem = new QLIST::Index[NodeArray.n];

	QLIST::Index *vertexTGNodeIdx = iVertexTGNodeMem;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		piVertexTGNodeList = iVertexTGNodeList + pNode->iVertex;

		RVLQLIST_ADD_ENTRY(piVertexTGNodeList, vertexTGNodeIdx);

		vertexTGNodeIdx->Idx = iNode;

		vertexTGNodeIdx++;
	}

	TGEdge *pEdge;

	RGData.csNThr = 0.8;

#ifdef NEVER
	Array<int> iNeighborVertices;

	iNeighborVertices.Element = new int[pVertexGraph->NodeArray.n];
	
	Array<int> iVertexBuff;

	iVertexBuff.Element = new int[pVertexGraph->NodeArray.n];

	RGData.csNThr = 0.0;
	RGData.csVThr = 0.8;
#endif

	int *vertexBuff = new int[iVertexArray.n];

	int *piVertexPut, *piVertexFetch, *vertexBuffEnd, *piVertex;

	//int iVertex_;
	float *N_;
	int iNode_;
	//QLIST::TreeIndex2<float> *piVertex;
	//bool bConnect;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		//if (iNode == 19)
		//	int debug = 0;

		pNode = NodeArray.Element + iNode;

		RGData.N = A_ + 3 * pNode->i;

#ifdef NEVER
		RGData.iOutNodeArray.Element = iNeighborVertices.Element;

		RGData.iOutNodeArray.n = 0;

		RVLQLIST_INIT(piNodeQueue);

		piVertex = RGData.iNodeMap + pNode->iVertex;

		RVLQLIST_ADD_ENTRY2(piNodeQueue, piVertex);

		piVertex->cost = 0.0f;

		Dijkstra < VertexGraph, SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>, TGConnectNodesRGData, float,
			ConnectNodesRG >(pVertexGraph, &RGData, piNodeQueue, RGData.iNodeMap);

		piVertex->cost = -1.0f;

		for (i = 0; i < RGData.iOutNodeArray.n; i++)
		{
			iVertex = RGData.iOutNodeArray.Element[i];

			RGData.mFlags[iVertex] &= ~0x02;

			RGData.iNodeMap[iVertex].cost = -1.0f;
		}

		iNeighborVertices.n = RGData.iOutNodeArray.n;

		for (i = 0; i < iNeighborVertices.n; i++)
		{
			iVertex = iNeighborVertices.Element[i];

			RGData.iOutNodeArray.Element = iVertexBuff.Element;

			piVertexTGNodeList = iVertexTGNodeList + iVertex;

			vertexTGNodeIdx = piVertexTGNodeList->pFirst;

			while (vertexTGNodeIdx)
			{
				iNode_ = vertexTGNodeIdx->Idx;

				if (iNode < iNode_)
				{
					pNode_ = NodeArray.Element + iNode_;

					N_ = A_ + 3 * pNode_->i;

					if (RVLDOTPRODUCT3(RGData.N, N_) >= RGData.csNThr)
					{
						RVLCROSSPRODUCT3(RGData.N, N_, RGData.V);

						RGData.iGoalNode = iVertex;
						RGData.iOutNodeArray.n = 0;

						RVLQLIST_INIT(piNodeQueue);

						piVertex = RGData.iNodeMap + pNode->iVertex;

						RVLQLIST_ADD_ENTRY2(piNodeQueue, piVertex);

						piVertex->cost = 0.0f;

						bConnect = Dijkstra < VertexGraph, SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>, 
							TGConnectNodesRGData, float, ConnectNodesRG2 >(pVertexGraph, &RGData, piNodeQueue, RGData.iNodeMap);

						piVertex->cost = -1.0f;

						for (j = 0; j < RGData.iOutNodeArray.n; j++)
						{
							iVertex_ = RGData.iOutNodeArray.Element[j];

							RGData.mFlags[iVertex_] &= ~0x02;

							RGData.iNodeMap[iVertex_].cost = -1.0f;
						}

						if (bConnect)
						{
							pEdge = ConnectNodes<TGNode, TGEdge, GRAPH::EdgePtr2<TGEdge>>(iNode, iNode_, NodeArray, pMem);

							nEdges++;
						}
					}
				}
						
				vertexTGNodeIdx = vertexTGNodeIdx->pNext;
			}
		}
#endif
		piVertexFetch = piVertexPut = vertexBuff;

		*(piVertexPut++) = pNode->iVertex;

		RGData.mFlags[pNode->iVertex] |= 0x02;

		vertexBuffEnd = RegionGrowing<VertexGraph, SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge>, TGConnectNodesRGData,
			ConnectNodesRG>(pVertexGraph, &RGData, piVertexFetch, piVertexPut);

		for (piVertex = vertexBuff; piVertex < vertexBuffEnd; piVertex++)
		{
			iVertex = *piVertex;

			RGData.mFlags[iVertex] &= ~0x02;

			piVertexTGNodeList = iVertexTGNodeList + iVertex;

			vertexTGNodeIdx = piVertexTGNodeList->pFirst;

			while (vertexTGNodeIdx)
			{
				iNode_ = vertexTGNodeIdx->Idx;

				if (iNode < iNode_)
				{
					pNode_ = NodeArray.Element + iNode_;

					N_ = A_ + 3 * pNode_->i;

					if (RVLDOTPRODUCT3(RGData.N, N_) >= RGData.csNThr)
					{
						pEdge = ConnectNodes<TGNode, TGEdge, GRAPH::EdgePtr2<TGEdge>>(iNode, iNode_, NodeArray, pMem);

						nEdges++;
					}
				}
					
				vertexTGNodeIdx = vertexTGNodeIdx->pNext;
			}
		}
	}	// for every TG node

	// Free memory.

	delete[] RGData.mFlags;
	delete[] vertexBuff;
	delete[] RGData.iNodeMap;
	delete[] iVertexTGNodeList;
	delete[] iVertexTGNodeMem;
#endif

	delete[] A_;
	//delete[] iNeighborVertices.Element;
	//delete[] iVertexBuff.Element;
}

int RECOG::ConnectNodesRG(
	int iVertex,
	int iParentVertex,
	SURFEL::VertexEdge *pEdge,
	VertexGraph *pVertexGraph,
	TGConnectNodesRGData *pData)
{
	if (pData->mFlags[iVertex] != 0x01)
		return 0;

	float csN = RVLDOTPRODUCT3(pEdge->N, pData->N);

	if (csN >= pData->csNThr)
	{
		pData->mFlags[iVertex] |= 0x02;

		return 1;
	}
	else
		return 0;
}

#ifdef NEVER
float RECOG::ConnectNodesRG2(
	int iVertex,
	int iParentVertex,
	SURFEL::VertexEdge *pEdge,
	VertexGraph *pVertexGraph,
	TGConnectNodesRGData *pData,
	bool &bGoalReached)
{
	if (!(pData->mFlags[iVertex] & 0x01))
		return -1.0f;

	if (bGoalReached = (iVertex == pData->iGoalNode))
		return -1.0f;

	float csN = RVLDOTPRODUCT3(pEdge->N, pData->N);

	if (csN >= pData->csNThr)
	{
		float cost = 1.0f - csN;

		if (cost > pData->iNodeMap[iParentVertex].cost)
		{
			float e = RVLDOTPRODUCT3(pEdge->N, pData->V);

			if (RVLABS(e) <= pData->csVThr)
			{
				if (!(pData->mFlags[iVertex] & 0x02))
				{
					pData->iOutNodeArray.Element[pData->iOutNodeArray.n++] = iVertex;

					pData->mFlags[iVertex] |= 0x02;
				}

				return cost;
			}
			else
				return -1.0f;
		}
		else
			return -1.0f;
	}
	else
		return -1.0f;
}

float RECOG::ConnectNodesRG(
	int iVertex,
	int iParentVertex,
	SURFEL::VertexEdge *pEdge,
	VertexGraph *pVertexGraph,
	TGConnectNodesRGData *pData,
	bool &bGoalReached)
{
	bGoalReached = false;

	if (!(pData->mFlags[iVertex] & 0x01))
		return -1.0f;

	float csN = RVLDOTPRODUCT3(pEdge->N, pData->N);

	if (csN >= pData->csNThr)
	{
		float cost = 1.0f - csN;

		if (cost > pData->iNodeMap[iParentVertex].cost)
		{
			if (!(pData->mFlags[iVertex] & 0x02))
			{
				pData->iOutNodeArray.Element[pData->iOutNodeArray.n++] = iVertex;

				pData->mFlags[iVertex] |= 0x02;
			}

			return cost;
		}
		else
			return -1.0f;
	}
	else
		return -1.0f;
}
#endif

void TG::Match(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	float scale,
	void *vpSet,
	float *RIn,
	float *tIn,
	bool bConvexHullAllignment,
	float &score,
	Array<TGCorrespondence> &correspondences,
	float *R,
	float *t
	)
{
	RVLCOPYMX3X3(RIn, R);
	RVLCOPY3VECTOR(tIn, t);

	if (iVertexArray.n == 0)
	{
		score = 0.0f;

		correspondences.n = 0;

		correspondences.Element = NULL;

#ifdef RVLTG_MATCH_DEBUG
		// Write matches to file.

		FILE *fp = fopen("TG_match_error.txt", "a");

		fprintf(fp, "%d\t0\t0\t%f\n", iObject, 0.0f);

		int i;

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t%f\t%f\t%f\n", 0.0f, 0.0f, 0.0f, 0.0f);

		fclose(fp);
#endif

		return;
	}

	// parameters

	int maxnIterations = 20;
	int maxnOptimizationIterations = 5;
	float dOrientationThr = PI / 200.0f;
	float dPositionThr = 1.0f;

	///

	TGSet *pSet = (TGSet *)vpSet;

	VertexGraph *pVertexGraph = pSet->GetVertexGraph(this);

	// Allocate arrays.

	float *A_ = new float[3 * A.h];
	float *PArray = new float[3 * iVertexArray.n];
	correspondences.Element = new TGCorrespondence[NodeArray.n];

	Array<TGCorrespondence> correspondences1, correspondences2;

	correspondences1.Element = new TGCorrespondence[NodeArray.n];
	correspondences1.n = 0;

	correspondences2.Element = new TGCorrespondence[NodeArray.n];
	correspondences2.n = 0;

	Array<TGCorrespondence> *pCorrespondences = &correspondences1;
	Array<TGCorrespondence> *pPrevCorrespondences = &correspondences2;

	Array<TGCorrespondence> *pCorrespondancesTmp;

	// Transform vertices to TG RF.

	pSurfels->TransformVertices(iVertexArray, scale, R, t, PArray);

	// main loop

	int k = 0;

	bool bCompleted = false;

	float dmax = 0.0f;

	double Mqq[3 * 3], Mqt[3 * 3], Mtt[3 * 3];
	double Mqq_[3 * 3], Mqt_[3 * 3], Mtt_[3 * 3];
	float aq[3];
	double bq[3], bt[3], bq_[3], bt_[3];
	float dq[3], dt[3], u[3];
	float q, lendt;
	int j;
	SURFEL::Vertex *pVertex;
	int iVertex;
	float V3Tmp[3];
	float *P;
	int i;
	float *N, *N_;
	QList<QLIST::Ptr<TGNode>> *pDescriptorBin;
	TGNode *pNode, *pCorrespondingNode;
	float d, eClosest, e, eAbsClosest, eAbs;
	float dist, fTmp;
	QLIST::Ptr<TGNode> *pNodePtr;
	TGCorrespondence *pCorrespondence, *pPrevCorrespondence;
	Eigen::MatrixXd M(6, 6);
	Eigen::VectorXd b(6);
	Eigen::VectorXd dw(6);
	float dR[9], RNew[9];
	int l;
	
	while (true)
	{
		// A_ <- A * R'	

		RotateTemplate(A, R, A_);

		// Identify correspondences and compute score.

		pCorrespondancesTmp = pCorrespondences;
		pCorrespondences = pPrevCorrespondences;
		pPrevCorrespondences = pCorrespondancesTmp;

		TGCorrespondence *pCorrespondence = pCorrespondences->Element;

		score = 0.0f;

		if (bConvexHullAllignment)
		{
			for (i = 0; i < A.h; i++)	// for every template normal
			{
				N = A.Element + 3 * i;
				N_ = A_ + 3 * i;

				pDescriptorBin = descriptor.Element + i;

				pCorrespondence->iVertex = -1;

				pNode = pDescriptorBin->pFirst->ptr;

				for (j = 0; j < iVertexArray.n; j++)		// for every vertex in iVertexArray
				{
					iVertex = iVertexArray.Element[j];

					pVertex = pSurfels->vertexArray.Element[iVertex];

					P = PArray + 3 * j;

					d = RVLDOTPRODUCT3(N, P);

					if (pCorrespondence->iVertex < 0 || d > dmax)
					{
						dmax = d;

						pCorrespondence->iVertex = j;
					}
				}	// for every vertex in iVertexArray

				pVertex = pSurfels->vertexArray.Element[pCorrespondence->iVertex];

				if (RVLDOTPRODUCT3(pVertex->P, N_) < 0.0f)
				{
					pCorrespondence->pNode = pNode;
					pCorrespondence->e = pNode->d - dmax;

					fTmp = pCorrespondence->e / pSet->eLimit;

					score += (1.0f - fTmp * fTmp);
					
					pCorrespondence++;
				}
			}	// for every template normal
		}	// if (bConvexHullAllignment)
		else
		{
			for (j = 0; j < iVertexArray.n; j++)	// for every vertex in iVertexArray
			{
				iVertex = iVertexArray.Element[j];

				pVertex = pSurfels->vertexArray.Element[iVertex];

				if (pVertex->normalHull.n < 3)
					continue;

				P = PArray + 3 * j;

				for (i = 0; i < A.h; i++)	// for every template normal
				{
					//if (i == 65)
					//	int debug = 0;

					N = A.Element + 3 * i;
					N_ = A_ + 3 * i;

					eAbsClosest = -1.0f;

					pDescriptorBin = descriptor.Element + i;

					pNodePtr = pDescriptorBin->pFirst;

					while (pNodePtr)	// for every node in the descriptor bin
					{
						pNode = pNodePtr->ptr;

						if (pVertex->type == pVertexGraph->NodeArray.Element[pNode->iVertex].type)
						{
							dist = pSurfels->DistanceFromNormalHull(pVertex->normalHull, N_);

							if (dist <= 0.0f)
							{
								d = RVLDOTPRODUCT3(N, P);

								e = pNode->d - d;

								eAbs = RVLABS(e);

								if (eAbsClosest < 0.0f || eAbs < eAbsClosest)
								{
									eClosest = e;

									eAbsClosest = eAbs;

									pCorrespondingNode = pNode;
								}
							}	// if the template normal is in the normal hull of the vertex 
						}	// if the vertices are of the same type

						pNodePtr = pNodePtr->pNext;
					}	// for every node in the descriptor bin

					if (eAbsClosest >= 0.0f && eAbsClosest <= pSet->eLimit)
					{
						pCorrespondence->pNode = pCorrespondingNode;
						pCorrespondence->iVertex = j;
						pCorrespondence->e = eClosest;
						pCorrespondence++;

						fTmp = eClosest / pSet->eLimit;

						score += (1.0f - fTmp * fTmp);
					}
				}	// for every template normal
			}	// for every vertex
		}

		pCorrespondences->n = pCorrespondence - pCorrespondences->Element;

//#ifdef RVLTG_MATCH_DEBUG
//		// Write matches to file.
//
//		FILE *fp = fopen("TG_match_error.txt", "w");
//
//		for (i = 0; i < 3; i++)
//			fprintf(fp, "%f\t%f\t%f\t%f\n", R[3 * i + 0], R[3 * i + 1], R[3 * i + 2], t[i]);
//
//		for (i = 0; i < pCorrespondences->n; i++)
//		{
//			pCorrespondence = pCorrespondences->Element + i;
//
//			fprintf(fp, "%d\t%d\t%d\t%f\n", pCorrespondence->pNode->i, pCorrespondence->pNode->j, iVertexArray.Element[pCorrespondence->iVertex],
//				pCorrespondence->e);
//		}
//
//		fclose(fp);
//#endif

		// If there are no changes in correspondences, then stop the procedure.

		if (k >= maxnIterations)
			bCompleted = true;
		else if (pCorrespondences->n == pPrevCorrespondences->n)
		{
			pCorrespondence = pCorrespondences->Element;
			pPrevCorrespondence = pPrevCorrespondences->Element;

			for (i = 0; i < pCorrespondences->n; i++, pCorrespondence++, pPrevCorrespondence++)
			{
				if (pCorrespondence->pNode != pCorrespondence->pNode)
					break;

				if (pCorrespondence->iVertex != pPrevCorrespondence->iVertex)
					break;
			}
				
			bCompleted = (i >= pCorrespondences->n);
		}

		if (bCompleted)
		{
#ifdef RVLTG_MATCH_DEBUG
			// Write matches to file.

			FILE *fp = fopen("TG_match_error.txt", "a");

			fprintf(fp, "%d\t%d\t%d\t%f\n", iObject, iVertexArray.n, pCorrespondences->n, score);

			for (i = 0; i < 3; i++)
				fprintf(fp, "%f\t%f\t%f\t%f\n", R[3 * i + 0], R[3 * i + 1], R[3 * i + 2], t[i]);

			int *iVertexRow = iVertexArray.Element;

			for (i = 0; i < iVertexArray.n / 4; i++, iVertexRow += 4)
				fprintf(fp, "%d\t%d\t%d\t%d\n", iVertexRow[0], iVertexRow[1], iVertexRow[2], iVertexRow[3]);

			int m = iVertexArray.n % 4;

			if (m > 0)
			{
				int iVertexLastRow[4];

				memset(iVertexLastRow, 0xff, 4 * sizeof(int));

				memcpy(iVertexLastRow, iVertexRow, m * sizeof(int));

				fprintf(fp, "%d\t%d\t%d\t%d\n", iVertexLastRow[0], iVertexLastRow[1], iVertexLastRow[2], iVertexLastRow[3]);
			}
			
			for (i = 0; i < pCorrespondences->n; i++)
			{
				pCorrespondence = pCorrespondences->Element + i;

				fprintf(fp, "%d\t%d\t%d\t%f\n", pCorrespondence->pNode->i, pCorrespondence->pNode->j, iVertexArray.Element[pCorrespondence->iVertex],
					pCorrespondence->e);
			}

			fclose(fp);
#endif

			break;
		}

		/// Compute the optimal pose.

		l = 0;

		do
		{
			RVLNULLMX3X3(Mqq);
			RVLNULLMX3X3(Mqt);
			RVLNULLMX3X3(Mtt);
			RVLNULL3VECTOR(bq);
			RVLNULL3VECTOR(bt);

			for (i = 0; i < pCorrespondences->n; i++)
			{
				pCorrespondence = pCorrespondences->Element + i;

				P = PArray + 3 * pCorrespondence->iVertex;

				N = A.Element + 3 * pCorrespondence->pNode->i;

				// aq <- P x N

				RVLCROSSPRODUCT3(P, N, aq);

				// M_ = [Mqq_  Mqt_] <- a * a',   where a = [aq', N']'
				//      [Mqt_' Mtt_]

				RVLVECTCOV3(aq, Mqq_);
				RVLMULVECT3VECT3T(aq, N, Mqt_);
				RVLVECTCOV3(N, Mtt_);

				// b_ = [bq_', bt_']' <- a * e

				e = pCorrespondence->e;

				RVLSCALE3VECTOR(aq, e, bq_);
				RVLSCALE3VECTOR(N, e, bt_);

				// M <- M + M_

				RVLSUMMX3X3UT(Mqq, Mqq_, Mqq);
				RVLSUMMX3X3(Mqt, Mqt_, Mqt);
				RVLSUMMX3X3UT(Mtt, Mtt_, Mtt);

				// b <- b + b_

				RVLSUM3VECTORS(bq, bq_, bq);
				RVLSUM3VECTORS(bt, bt_, bt);
			}

			// dw = [dq' dt']' <- solve M * dw = b

			M << RVLMXEL(Mqq, 3, 0, 0), RVLMXEL(Mqq, 3, 0, 1), RVLMXEL(Mqq, 3, 0, 2), RVLMXEL(Mqt, 3, 0, 0), RVLMXEL(Mqt, 3, 0, 1), RVLMXEL(Mqt, 3, 0, 2),
				RVLMXEL(Mqq, 3, 0, 1), RVLMXEL(Mqq, 3, 1, 1), RVLMXEL(Mqq, 3, 1, 2), RVLMXEL(Mqt, 3, 1, 0), RVLMXEL(Mqt, 3, 1, 1), RVLMXEL(Mqt, 3, 1, 2),
				RVLMXEL(Mqq, 3, 0, 2), RVLMXEL(Mqq, 3, 1, 2), RVLMXEL(Mqq, 3, 2, 2), RVLMXEL(Mqt, 3, 2, 0), RVLMXEL(Mqt, 3, 2, 1), RVLMXEL(Mqt, 3, 2, 2),
				RVLMXEL(Mqt, 3, 0, 0), RVLMXEL(Mqt, 3, 1, 0), RVLMXEL(Mqt, 3, 2, 0), RVLMXEL(Mtt, 3, 0, 0), RVLMXEL(Mtt, 3, 0, 1), RVLMXEL(Mtt, 3, 0, 2),
				RVLMXEL(Mqt, 3, 0, 1), RVLMXEL(Mqt, 3, 1, 1), RVLMXEL(Mqt, 3, 2, 1), RVLMXEL(Mtt, 3, 0, 1), RVLMXEL(Mtt, 3, 1, 1), RVLMXEL(Mtt, 3, 1, 2),
				RVLMXEL(Mqt, 3, 0, 2), RVLMXEL(Mqt, 3, 1, 2), RVLMXEL(Mqt, 3, 2, 2), RVLMXEL(Mtt, 3, 0, 2), RVLMXEL(Mtt, 3, 1, 2), RVLMXEL(Mtt, 3, 2, 2);
			b << bq[0], bq[1], bq[2], bt[0], bt[1], bt[2];
			dw = M.colPivHouseholderQr().solve(b);
			RVLCOPY3VECTOR(dw, dq);
			dt[0] = dw[3]; dt[1] = dw[4]; dt[2] = dw[5];

			// q = || dq ||

			q = sqrt(RVLDOTPRODUCT3(dq, dq));

			// u <- dq / || dq ||

			RVLSCALE3VECTOR2(dq, q, u);

			// dR <- Rot(u, q) (angle axis to rotation matrix)

			AngleAxisToRot<float>(u, q, dR);

			//RVLSKEW(dq, dR);
			//dR[0] = dR[4] = dR[8] = 1.0f;

			// R <- (dR * R')' = R * dR'

			RVLMXMUL3X3T2(R, dR, RNew);
			RVLCOPYMX3X3(RNew, R);

			// t <- t - RNew * dt

			RVLMULMX3X3VECT(RNew, dt, V3Tmp);
			RVLDIF3VECTORS(t, V3Tmp, t);

			// lendt <- || dt ||

			lendt = sqrt(RVLDOTPRODUCT3(dt, dt));

			// Transform vertices with new R and t

			pSurfels->TransformVertices(iVertexArray, scale, R, t, PArray);

			// Compute new score

			float score_ = 0.0f;
			//float E = 0.0f;
			//float E_ = 0.0f;

			float PM[3];
			float e_;

			for (i = 0; i < pCorrespondences->n; i++)
			{
				pCorrespondence = pCorrespondences->Element + i;

				P = PArray + 3 * pCorrespondence->iVertex;

				N = A.Element + 3 * pCorrespondence->pNode->i;

				d = RVLDOTPRODUCT3(N, P);

				e = pCorrespondence->pNode->d - d;

				fTmp = e / pSet->eLimit;

				score_ += (1.0f - fTmp * fTmp);

				pCorrespondence->e = e;

				//P = PArray + 3 * pCorrespondence->iVertex;

				//RVLCROSSPRODUCT3(P, N, aq);

				//e_ = pCorrespondence->e - RVLDOTPRODUCT3(aq, dq) - RVLDOTPRODUCT3(N, dt);

				//// M_ = [Mqq_  Mqt_] <- a * a',   where a = [aq', N']'
				////      [Mqt_' Mtt_]

				//RVLVECTCOV3(aq, Mqq_);
				//RVLMULVECT3VECT3T(aq, N, Mqt_);
				//RVLVECTCOV3(N, Mtt_);

				//// b_ = [bq_', bt_']' <- a * e

				//RVLSCALE3VECTOR(aq, pCorrespondence->e, bq_);
				//RVLSCALE3VECTOR(N, pCorrespondence->e, bt_);

				//RVLMULMX3X3TVECT(Mqt_, dq, V3Tmp)

				//float e__2 = RVLCOV3DTRANSFTO1D(Mqq_, dq) + 2.0 * RVLDOTPRODUCT3(V3Tmp, dt) + RVLCOV3DTRANSFTO1D(Mtt_, dt);

				//E += (pCorrespondence->e * pCorrespondence->e);
				//E_ += (e_ * e_);

				//int debug = 0;
			}

			//// E__ = dw' * M * dw - 2 * b' dw + E

			//Eigen::VectorXd g = dw.transpose() * M * dw - 2.0 * b.transpose() * dw;

			//double E__ = g[0] + E;

			l++;
		} while ((RVLABS(q) > dOrientationThr || RVLABS(lendt) > dPositionThr) && l < maxnOptimizationIterations);

		k++;
	}	// main loop

	// Copy pCorrespondences to correspondences.

	correspondences.n = pCorrespondences->n;

	memcpy(correspondences.Element, pCorrespondences->Element, pCorrespondences->n * sizeof(TGCorrespondence));

	// Free memory.

	delete[] A_;
	delete[] PArray;
	delete[] correspondences1.Element;
	delete[] correspondences2.Element;
}

void TG::Save(
	FILE *fp,
	bool bSaveA)
{
	fprintf(fp, "%d\t%d\t0\n", iObject, iVertexGraph);
	fprintf(fp, "%d\t%d\t0\n", NodeArray.n, nEdges);

	int i;

	for (i = 0; i < 3; i++)
		fprintf(fp, "%f\t%f\t%f\n", R[3 * i + 0], R[3 * i + 1], R[3 * i + 2]);

	fprintf(fp, "%f\t%f\t%f\n", t[0], t[1], t[2]);

	if (bSaveA)
	{
		fprintf(fp, "%d\t0\t0\n", A.h);

		float *N = A.Element;

		for (i = 0; i < A.h; i++, N += 3)
			fprintf(fp, "%f\t%f\t%f\n", N[0], N[1], N[2]);
	}	

	TGNode *pNode;

	for (i = 0; i < NodeArray.n; i++)
	{
		pNode = NodeArray.Element + i;

		fprintf(fp, "%d\t%f\t%d\n", pNode->i, pNode->d, pNode->iVertex);
	}

	QList<GRAPH::EdgePtr2<TGEdge>> *pEdgeList;
	GRAPH::EdgePtr2<TGEdge> *pEdgePtr;
	int iNode, iNode_;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pEdgeList = &(pNode->EdgeList);

		pEdgePtr = pEdgeList->pFirst;

		while (pEdgePtr)
		{
			iNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			if (iNode < iNode_)
				fprintf(fp, "%d\t%d\t0\n", iNode, iNode_);

			pEdgePtr = pEdgePtr->pNext;
		}
	}
}

bool TG::Load(
	FILE *fp,
	void *vpSet,
	bool bLoadA)
{
	if (fscanf(fp, "%d\t%d\t0\n", &iObject, &iVertexGraph) < 2)
		return false;

	if (fscanf(fp, "%d\t%d\t0\n", &(NodeArray.n), &nEdges) < 2)
		return false;

	TGSet *pSet = (TGSet *)vpSet;

	CRVLMem *pMem = pSet->pMem;

	int i;

	for (i = 0; i < 3; i++)
		fscanf(fp, "%f\t%f\t%f\n", R + 3 * i, R + 3 * i + 1, R + 3 * i + 2);

	fscanf(fp, "%f\t%f\t%f\n", t, t + 1, t + 2);

	if (bLoadA)
	{
		fscanf(fp, "%d\t0\t0\n", &(A.h));

		A.w = 3;

		float *N = A.Element;

		for (i = 0; i < A.h; i++, N += 3)
			fscanf(fp, "%f\t%f\t%f\n", N, N + 1, N + 2);
	}

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, QList<QLIST::Ptr<TGNode>>, A.h, descriptor.Element);

	QList<QLIST::Ptr<TGNode>> *pDescriptorBin;

	for (i = 0; i < A.h; i++)
	{
		pDescriptorBin = descriptor.Element + i;

		RVLQLIST_INIT(pDescriptorBin);
	}

	QLIST::Ptr<TGNode> *descriptorMem;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, QLIST::Ptr<TGNode>, NodeArray.n, descriptorMem);

	QLIST::Ptr<TGNode> *pNodePtr = descriptorMem;

	RVL_DELETE_ARRAY(NodeArray.Element);

	NodeArray.Element = new TGNode[NodeArray.n];

	TGNode *pNode = NodeArray.Element;

	int j = 0;
	int i_ = -1;

	QList<GRAPH::EdgePtr2<TGEdge>> *pEdgeList;

	for (i = 0; i < NodeArray.n; i++, pNode++, pNodePtr++)
	{
		fscanf(fp, "%d\t%f\t%d\n", &(pNode->i), &(pNode->d), &(pNode->iVertex));

		pDescriptorBin = descriptor.Element + pNode->i;

		pNodePtr->ptr = pNode;

		RVLQLIST_ADD_ENTRY(pDescriptorBin, pNodePtr);
		
		if (pNode->i == i_)
			j++;
		else
		{
			j = 0;
			i_ = pNode->i;
		}

		pNode->j = j;

		pEdgeList = &(pNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	int iNode, iNode_;
	TGEdge *pEdge;

	for (i = 0; i < nEdges; i++)
	{
		fscanf(fp, "%d\t%d\t0\n", &iNode, &iNode_);

		pEdge = ConnectNodes<TGNode, TGEdge, GRAPH::EdgePtr2<TGEdge>>(iNode, iNode_, NodeArray, pMem);
	}

	return true;
}

// This function is explained in ARP3D.TR12, Section "Tangent Alignment".

void RECOG::TangentAlignment(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	float scale,
	Array2D<float> A,
	float *descriptor,
	float *RIn,
	float *tIn,
	float eThr,
	float &score,
	Array<TangentVertexCorrespondence> &correspondences,
	float *R,
	float *t
	)
{
	RVLCOPYMX3X3(RIn, R);
	RVLCOPY3VECTOR(tIn, t);

	if (iVertexArray.n == 0)
	{
		score = 0.0f;

		correspondences.n = 0;

		correspondences.Element = NULL;

#ifdef RVLTG_MATCH_DEBUG
		// Write matches to file.

		FILE *fp = fopen("TG_match_error.txt", "a");

		fprintf(fp, "%d\t0\t0\t%f\n", iObject, 0.0f);

		int i;

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t%f\t%f\t%f\n", 0.0f, 0.0f, 0.0f, 0.0f);

		fclose(fp);
#endif

		return;
	}

	// parameters

	int maxnIterations = 1;
	int maxnOptimizationIterations = 5;
	float dOrientationThr = PI / 200.0f;
	float dPositionThr = 1.0f;

	// Allocate arrays.

	float *A_ = new float[3 * A.h];
	float *PArray = new float[3 * iVertexArray.n];
	correspondences.Element = new TangentVertexCorrespondence[A.h];

	Array<TangentVertexCorrespondence> correspondences1, correspondences2;

	correspondences1.Element = new TangentVertexCorrespondence[A.h];
	correspondences1.n = 0;

	correspondences2.Element = new TangentVertexCorrespondence[A.h];
	correspondences2.n = 0;

	Array<TangentVertexCorrespondence> *pCorrespondences = &correspondences1;
	Array<TangentVertexCorrespondence> *pPrevCorrespondences = &correspondences2;

	Array<TangentVertexCorrespondence> *pCorrespondancesTmp;

	// Transform vertices to TG RF.

	pSurfels->TransformVertices(iVertexArray, scale, R, t, PArray);

	// main loop

	int k = 0;

	bool bCompleted = false;

	float dmax = 0.0f;

	double Mqq[3 * 3], Mqt[3 * 3], Mtt[3 * 3];
	double Mqq_[3 * 3], Mqt_[3 * 3], Mtt_[3 * 3];
	float aq[3];
	double bq[3], bt[3], bq_[3], bt_[3];
	float dq[3], dt[3], u[3];
	float q, lendt;
	int j;
	SURFEL::Vertex *pVertex;
	int iVertex;
	float V3Tmp[3];
	float *P;
	int i;
	float *N, *N_;
	float d, e;
	float fTmp;
	TangentVertexCorrespondence *pCorrespondence, *pPrevCorrespondence;
	Eigen::MatrixXd M(6, 6);
	Eigen::VectorXd b(6);
	Eigen::VectorXd dw(6);
	float dR[9], RNew[9];
	int l;
	//float eNormalHull;

	while (true)
	{
		// A_ <- A * R'	

		RotateTemplate(A, R, A_);

		// Identify correspondences and compute score.

		pCorrespondancesTmp = pCorrespondences;
		pCorrespondences = pPrevCorrespondences;
		pPrevCorrespondences = pCorrespondancesTmp;

		pCorrespondence = pCorrespondences->Element;

		score = 0.0f;

		for (i = 0; i < A.h; i++)	// for every template normal
		{
			N = A.Element + 3 * i;
			N_ = A_ + 3 * i;

			pCorrespondence->iVertex = -1;

			for (j = 0; j < iVertexArray.n; j++)		// for every vertex in iVertexArray
			{
				iVertex = iVertexArray.Element[j];

				pVertex = pSurfels->vertexArray.Element[iVertex];

				P = PArray + 3 * j;

				d = RVLDOTPRODUCT3(N, P);

				if (pCorrespondence->iVertex < 0 || d > dmax)
				{
					dmax = d;

					pCorrespondence->iVertex = j;
				}
			}	// for every vertex in iVertexArray

			pVertex = pSurfels->vertexArray.Element[pCorrespondence->iVertex];

			if (RVLDOTPRODUCT3(pVertex->P, N_) < 0.0f)
			{
				//eNormalHull = pSurfels->DistanceFromNormalHull(pVertex->normalHull, N_);

				//if (eNormalHull <= 0.0f)
				//if (pVertex->bForeground)

				e = descriptor[i] - dmax;

				if (RVLABS(e) <= eThr)
				{
					pCorrespondence->iTangent = i;
					pCorrespondence->e = e;

					fTmp = e / eThr;

					score += (1.0f - fTmp * fTmp);

					pCorrespondence++;
				}
			}
		}	// for every template normal

		pCorrespondences->n = pCorrespondence - pCorrespondences->Element;

		//#ifdef RVLTG_MATCH_DEBUG
		//		// Write matches to file.
		//
		//		FILE *fp = fopen("TG_match_error.txt", "w");
		//
		//		for (i = 0; i < 3; i++)
		//			fprintf(fp, "%f\t%f\t%f\t%f\n", R[3 * i + 0], R[3 * i + 1], R[3 * i + 2], t[i]);
		//
		//		for (i = 0; i < pCorrespondences->n; i++)
		//		{
		//			pCorrespondence = pCorrespondences->Element + i;
		//
		//			fprintf(fp, "%d\t%d\t%d\t%f\n", pCorrespondence->pNode->i, pCorrespondence->pNode->j, iVertexArray.Element[pCorrespondence->iVertex],
		//				pCorrespondence->e);
		//		}
		//
		//		fclose(fp);
		//#endif

		// If there are no changes in correspondences, then stop the procedure.

		if (k >= maxnIterations)
			bCompleted = true;
		else if (pCorrespondences->n == pPrevCorrespondences->n)
		{
			pCorrespondence = pCorrespondences->Element;
			pPrevCorrespondence = pPrevCorrespondences->Element;

			for (i = 0; i < pCorrespondences->n; i++, pCorrespondence++, pPrevCorrespondence++)
			{
				if (pCorrespondence->iTangent != pCorrespondence->iTangent)
					break;

				if (pCorrespondence->iVertex != pPrevCorrespondence->iVertex)
					break;
			}

			bCompleted = (i >= pCorrespondences->n);
		}

		if (bCompleted)
		{
#ifdef RVLTG_MATCH_DEBUG
			// Write matches to file.

			FILE *fp = fopen("TG_match_error.txt", "a");

			fprintf(fp, "%d\t%d\t%d\t%f\n", iObject, iVertexArray.n, pCorrespondences->n, score);

			for (i = 0; i < 3; i++)
				fprintf(fp, "%f\t%f\t%f\t%f\n", R[3 * i + 0], R[3 * i + 1], R[3 * i + 2], t[i]);

			int *iVertexRow = iVertexArray.Element;

			for (i = 0; i < iVertexArray.n / 4; i++, iVertexRow += 4)
				fprintf(fp, "%d\t%d\t%d\t%d\n", iVertexRow[0], iVertexRow[1], iVertexRow[2], iVertexRow[3]);

			int m = iVertexArray.n % 4;

			if (m > 0)
			{
				int iVertexLastRow[4];

				memset(iVertexLastRow, 0xff, 4 * sizeof(int));

				memcpy(iVertexLastRow, iVertexRow, m * sizeof(int));

				fprintf(fp, "%d\t%d\t%d\t%d\n", iVertexLastRow[0], iVertexLastRow[1], iVertexLastRow[2], iVertexLastRow[3]);
			}

			for (i = 0; i < pCorrespondences->n; i++)
			{
				pCorrespondence = pCorrespondences->Element + i;

				fprintf(fp, "%d\t%d\t%d\t%f\n", pCorrespondence->pNode->i, pCorrespondence->pNode->j, iVertexArray.Element[pCorrespondence->iVertex],
					pCorrespondence->e);
			}

			fclose(fp);
#endif

			break;
		}

		/// Compute the optimal pose.

		l = 0;

		do
		{
			RVLNULLMX3X3(Mqq);
			RVLNULLMX3X3(Mqt);
			RVLNULLMX3X3(Mtt);
			RVLNULL3VECTOR(bq);
			RVLNULL3VECTOR(bt);

			for (i = 0; i < pCorrespondences->n; i++)
			{
				pCorrespondence = pCorrespondences->Element + i;

				P = PArray + 3 * pCorrespondence->iVertex;

				N = A.Element + 3 * pCorrespondence->iTangent;

				// aq <- P x N

				RVLCROSSPRODUCT3(P, N, aq);

				// M_ = [Mqq_  Mqt_] <- a * a',   where a = [aq', N']'
				//      [Mqt_' Mtt_]

				RVLVECTCOV3(aq, Mqq_);
				RVLMULVECT3VECT3T(aq, N, Mqt_);
				RVLVECTCOV3(N, Mtt_);

				// b_ = [bq_', bt_']' <- a * e

				e = pCorrespondence->e;

				RVLSCALE3VECTOR(aq, e, bq_);
				RVLSCALE3VECTOR(N, e, bt_);

				// M <- M + M_

				RVLSUMMX3X3UT(Mqq, Mqq_, Mqq);
				RVLSUMMX3X3(Mqt, Mqt_, Mqt);
				RVLSUMMX3X3UT(Mtt, Mtt_, Mtt);

				// b <- b + b_

				RVLSUM3VECTORS(bq, bq_, bq);
				RVLSUM3VECTORS(bt, bt_, bt);
			}

			// dw = [dq' dt']' <- solve M * dw = b

			M << RVLMXEL(Mqq, 3, 0, 0), RVLMXEL(Mqq, 3, 0, 1), RVLMXEL(Mqq, 3, 0, 2), RVLMXEL(Mqt, 3, 0, 0), RVLMXEL(Mqt, 3, 0, 1), RVLMXEL(Mqt, 3, 0, 2),
				RVLMXEL(Mqq, 3, 0, 1), RVLMXEL(Mqq, 3, 1, 1), RVLMXEL(Mqq, 3, 1, 2), RVLMXEL(Mqt, 3, 1, 0), RVLMXEL(Mqt, 3, 1, 1), RVLMXEL(Mqt, 3, 1, 2),
				RVLMXEL(Mqq, 3, 0, 2), RVLMXEL(Mqq, 3, 1, 2), RVLMXEL(Mqq, 3, 2, 2), RVLMXEL(Mqt, 3, 2, 0), RVLMXEL(Mqt, 3, 2, 1), RVLMXEL(Mqt, 3, 2, 2),
				RVLMXEL(Mqt, 3, 0, 0), RVLMXEL(Mqt, 3, 1, 0), RVLMXEL(Mqt, 3, 2, 0), RVLMXEL(Mtt, 3, 0, 0), RVLMXEL(Mtt, 3, 0, 1), RVLMXEL(Mtt, 3, 0, 2),
				RVLMXEL(Mqt, 3, 0, 1), RVLMXEL(Mqt, 3, 1, 1), RVLMXEL(Mqt, 3, 2, 1), RVLMXEL(Mtt, 3, 0, 1), RVLMXEL(Mtt, 3, 1, 1), RVLMXEL(Mtt, 3, 1, 2),
				RVLMXEL(Mqt, 3, 0, 2), RVLMXEL(Mqt, 3, 1, 2), RVLMXEL(Mqt, 3, 2, 2), RVLMXEL(Mtt, 3, 0, 2), RVLMXEL(Mtt, 3, 1, 2), RVLMXEL(Mtt, 3, 2, 2);
			b << bq[0], bq[1], bq[2], bt[0], bt[1], bt[2];
			dw = M.colPivHouseholderQr().solve(b);
			RVLCOPY3VECTOR(dw, dq);
			dt[0] = dw[3]; dt[1] = dw[4]; dt[2] = dw[5];

			// q = || dq ||

			q = sqrt(RVLDOTPRODUCT3(dq, dq));

			// u <- dq / || dq ||

			RVLSCALE3VECTOR2(dq, q, u);

			// dR <- Rot(u, q) (angle axis to rotation matrix)

			AngleAxisToRot<float>(u, q, dR);

			//RVLSKEW(dq, dR);
			//dR[0] = dR[4] = dR[8] = 1.0f;

			// R <- (dR * R')' = R * dR'

			RVLMXMUL3X3T2(R, dR, RNew);
			RVLCOPYMX3X3(RNew, R);

			// t <- t - RNew * dt

			RVLMULMX3X3VECT(RNew, dt, V3Tmp);
			RVLDIF3VECTORS(t, V3Tmp, t);

			// lendt <- || dt ||

			lendt = sqrt(RVLDOTPRODUCT3(dt, dt));

			// Transform vertices with new R and t

			pSurfels->TransformVertices(iVertexArray, scale, R, t, PArray);

			// Compute new score

			float score_ = 0.0f;
			//float E = 0.0f;
			//float E_ = 0.0f;

			for (i = 0; i < pCorrespondences->n; i++)
			{
				pCorrespondence = pCorrespondences->Element + i;

				P = PArray + 3 * pCorrespondence->iVertex;

				N = A.Element + 3 * pCorrespondence->iTangent;

				d = RVLDOTPRODUCT3(N, P);

				e = descriptor[pCorrespondence->iTangent] - d;

				fTmp = e / eThr;

				score_ += (1.0f - fTmp * fTmp);

				pCorrespondence->e = e;

				//P = PArray + 3 * pCorrespondence->iVertex;

				//RVLCROSSPRODUCT3(P, N, aq);

				//e_ = pCorrespondence->e - RVLDOTPRODUCT3(aq, dq) - RVLDOTPRODUCT3(N, dt);

				//// M_ = [Mqq_  Mqt_] <- a * a',   where a = [aq', N']'
				////      [Mqt_' Mtt_]

				//RVLVECTCOV3(aq, Mqq_);
				//RVLMULVECT3VECT3T(aq, N, Mqt_);
				//RVLVECTCOV3(N, Mtt_);

				//// b_ = [bq_', bt_']' <- a * e

				//RVLSCALE3VECTOR(aq, pCorrespondence->e, bq_);
				//RVLSCALE3VECTOR(N, pCorrespondence->e, bt_);

				//RVLMULMX3X3TVECT(Mqt_, dq, V3Tmp)

				//float e__2 = RVLCOV3DTRANSFTO1D(Mqq_, dq) + 2.0 * RVLDOTPRODUCT3(V3Tmp, dt) + RVLCOV3DTRANSFTO1D(Mtt_, dt);

				//E += (pCorrespondence->e * pCorrespondence->e);
				//E_ += (e_ * e_);

				//int debug = 0;
			}

			//// E__ = dw' * M * dw - 2 * b' dw + E

			//Eigen::VectorXd g = dw.transpose() * M * dw - 2.0 * b.transpose() * dw;

			//double E__ = g[0] + E;

			l++;
		} while ((RVLABS(q) > dOrientationThr || RVLABS(lendt) > dPositionThr) && l < maxnOptimizationIterations);

		k++;
	}	// main loop

	// Copy pCorrespondences to correspondences.

	correspondences.n = pCorrespondences->n;

	memcpy(correspondences.Element, pCorrespondences->Element, pCorrespondences->n * sizeof(TGCorrespondence));

	// Free memory.

	delete[] A_;
	delete[] PArray;
	delete[] correspondences1.Element;
	delete[] correspondences2.Element;
}