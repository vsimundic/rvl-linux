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
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include <Eigen\Eigenvalues>

#define RVLTG_MATCH_DEBUG

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

void TG::RotateTemplate(
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

	RotateTemplate(R, A_);

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

	// Connect neighboring nodes with edges.

	TGConnectNodesRGData RGData;

	RGData.mFlags = new BYTE[pVertexGraph->NodeArray.n];

	memset(RGData.mFlags, 0, pVertexGraph->NodeArray.n);

	QList<QLIST::Index> *iVertexTGNodeList;

	iVertexTGNodeList = new QList<QLIST::Index>[pVertexGraph->NodeArray.n];

	QList<QLIST::Index> *piVertexTGNodeList;

	for (i = 0; i < iVertexArray.n; i++)
	{
		iVertex = iVertexArray.Element[i];

		RGData.mFlags[iVertex] = 0x01;

		piVertexTGNodeList = iVertexTGNodeList + iVertex;

		RVLQLIST_INIT(piVertexTGNodeList);
	}
		
	QLIST::Index *iVertexTGNodeMem = new QLIST::Index[NodeArray.n];

	QLIST::Index *vertexTGNodeIdx = iVertexTGNodeMem;

	int iNode;
	
	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		piVertexTGNodeList = iVertexTGNodeList + pNode->iVertex;

		RVLQLIST_ADD_ENTRY(piVertexTGNodeList, vertexTGNodeIdx);

		vertexTGNodeIdx->Idx = iNode;

		vertexTGNodeIdx++;
	}

	QList<GRAPH::EdgePtr2<TGEdge>> *pEdgeList;
	TGEdge *pEdge;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pEdgeList = &(pNode->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	RGData.csNThr = 0.8;

	int *vertexBuff = new int[iVertexArray.n];

	nEdges = 0;

	int *piVertexPut, *piVertexFetch, *vertexBuffEnd, *piVertex;

	float *N_;
	int iNode_;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		//if (iNode == 92)
		//	int debug = 0;

		pNode = NodeArray.Element + iNode;

		RGData.N = A_ + 3 * pNode->i;

		piVertexFetch = piVertexPut = vertexBuff;

		*(piVertexPut++) = pNode->iVertex;

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

	delete[] iVertexTGNodeMem;
	delete[] iVertexTGNodeList;
	delete[] RGData.mFlags;
	delete[] vertexBuff;
	delete[] A_;
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

void TG::Match(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	float scale,
	void *vpSet,
	float *RIn,
	float *tIn,
	float &score,
	Array<TGCorrespondence> &correspondences
	)
{
	// parameters

	int maxnIterations = 20;
	int maxnOptimizationIterations = 5;
	float dOrientationThr = PI / 200.0f;
	float dPositionThr = 1.0f;

	///

	TGSet *pSet = (TGSet *)vpSet;

	VertexGraph *pVertexGraph = pSet->GetVertexGraph(this);

	float R[9], t[3];

	RVLCOPYMX3X3(RIn, R);
	RVLCOPY3VECTOR(tIn, t);

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

	TransformVertices(pSurfels, iVertexArray, scale, R, t, PArray);

	// main loop

	int k = 0;

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
	bool bCompleted;
	int l;
	
	while (true)
	{
		// A_ <- A * R'	

		RotateTemplate(R, A_);

		// Identify correspondences and compute score.

		pCorrespondancesTmp = pCorrespondences;
		pCorrespondences = pPrevCorrespondences;
		pPrevCorrespondences = pCorrespondancesTmp;

		TGCorrespondence *pCorrespondence = pCorrespondences->Element;

		score = 0.0f;

		for (j = 0; j < iVertexArray.n; j++)
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

			FILE *fp = fopen("TG_match_error.txt", "w");

			for (i = 0; i < 3; i++)
				fprintf(fp, "%f\t%f\t%f\t%f\n", R[3 * i + 0], R[3 * i + 1], R[3 * i + 2], t[i]);

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

			TransformVertices(pSurfels, iVertexArray, scale, R, t, PArray);

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

	// Free memory.

	delete[] A_;
	delete[] PArray;
	delete[] correspondences1.Element;
	delete[] correspondences2.Element;
}

void TG::TransformVertices(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	float scale,
	float *R,
	float *t,
	float *PArray)
{
	float sR[9];
	float st[3];

	// sR <- scale * R

	RVLSCALEMX3X3(R, scale, sR);
	RVLSCALE3VECTOR2(t, scale, st);

	// Transform vertices to TG RF.

	int j, iVertex;
	SURFEL::Vertex *pVertex;
	float *P;
	float V3Tmp[3];

	for (j = 0; j < iVertexArray.n; j++)
	{
		iVertex = iVertexArray.Element[j];

		pVertex = pSurfels->vertexArray.Element[iVertex];

		if (pVertex->normalHull.n < 3)
			continue;

		P = PArray + 3 * j;

		RVLINVTRANSF3(pVertex->P, sR, st, P, V3Tmp);
	}
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