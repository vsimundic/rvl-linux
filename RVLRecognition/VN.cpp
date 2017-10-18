//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "ObjectDetector.h"
#include <Eigen\Eigenvalues>
#include "VN.h"
#include "VNClassifier.h"

#define RVLVN_MATCH_DEBUG

using namespace RVL;
using namespace RECOG;

VN::VN()
{
	NodeArray.Element = NULL;
	featureArray.Element = NULL;
	projectionIntervals.Element = NULL;
	projectionIntervalMem = NULL;
	projectionIntervals.Element = NULL;
	projectionIntervalMem = NULL;
	projectionIntervalBuff.Element = NULL;
	dc = NULL;

	QList<RECOG::VN_::ModelCluster> *pModelClusterList = &modelClusterList;

	RVLQLIST_INIT(pModelClusterList);
}


VN::~VN()
{
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(featureArray.Element);
	RVL_DELETE_ARRAY(projectionIntervals.Element);
	RVL_DELETE_ARRAY(projectionIntervalMem);
	RVL_DELETE_ARRAY(projectionIntervalBuff.Element);
	RVL_DELETE_ARRAY(dc);
}

void VN::CreateParamList(
	CRVLParameterList *pParamList,
	RECOG::VN_::Parameters &params,
	CRVLMem *pMem)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("VN.kMaxMatchCost", RVLPARAM_TYPE_FLOAT, &(params.kMaxMatchCost));
	pParamData = pParamList->AddParam("VN.maxnSClusters", RVLPARAM_TYPE_INT, &(params.maxnSClusters));
}

void VN::AddModelCluster(
	int ID,
	BYTE type,
	float *R,
	float *t,
	float r,
	Array<float> alphaArray,
	Array<float> betaArray,
	CRVLMem *pMem,
	float rT)
{
	VN_::ModelCluster *pMCluster;

	RVLMEM_ALLOC_STRUCT(pMem, VN_::ModelCluster, pMCluster);

	QList<VN_::ModelCluster> *pModelClusterList = &modelClusterList;

	RVLQLIST_ADD_ENTRY(pModelClusterList, pMCluster);

	pMCluster->ID = ID;
	pMCluster->type = type;
	RVLCOPYMX3X3(R, pMCluster->R);
	RVLCOPY3VECTOR(t, pMCluster->t);
	pMCluster->r = r;
	pMCluster->rT = rT;
	pMCluster->alphaArray = alphaArray;
	pMCluster->betaArray = betaArray;
}

void VN::AddModelCluster(
	int ID,
	BYTE type,
	float *R,
	float *t,
	float r,
	int nAlphasPer2PI,
	int nBetasPerPI,
	Pair<int, int> iBetaInterval,
	CRVLMem *pMem,
	float rT,
	Pair<int, int> iAlphaIntervalIn)
{
	Pair<int, int> iAlphaInterval = iAlphaIntervalIn;

	if (iAlphaInterval.b <= iAlphaInterval.a)
	{
		iAlphaInterval.a = 0;
		iAlphaInterval.b = nAlphasPer2PI - 1;
	}

	Array<float> alphaArray;

	alphaArray.n = iAlphaInterval.b - iAlphaInterval.a + 1;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, alphaArray.n, alphaArray.Element);

	float dAlpha = 2 * PI / (float)nAlphasPer2PI;

	int iAlpha_ = 0;

	int iAlpha;

	for (iAlpha = iAlphaInterval.a; iAlpha <= iAlphaInterval.b; iAlpha++, iAlpha_++)
		alphaArray.Element[iAlpha_] = dAlpha * (float)iAlpha;

	Array<float> betaArray;

	betaArray.n = iBetaInterval.b - iBetaInterval.a + 1;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, betaArray.n, betaArray.Element);

	float dBeta = PI / (float)nBetasPerPI;

	int iBeta_ = 0;

	int iBeta;

	for (iBeta = iBetaInterval.a; iBeta <= iBetaInterval.b; iBeta++, iBeta_++)
		betaArray.Element[iBeta_] = dBeta * (float)iBeta;

	AddModelCluster(ID, type, R, t, r, alphaArray, betaArray, pMem, rT);
}

void VN::AddOperation(
	int ID,
	int operation,
	int operand1,
	int operand2,
	CRVLMem *pMem)
{
	VN_::Operation *pOperation;

	RVLMEM_ALLOC_STRUCT(pMem, VN_::Operation, pOperation);

	QList<VN_::Operation> *pOperationList = &operationList;

	RVLQLIST_ADD_ENTRY(pOperationList, pOperation);

	pOperation->ID = ID;
	pOperation->operation = operation;
	pOperation->operand[0] = operand1;
	pOperation->operand[1] = operand2;
}

void VN::AddLimit(
	int sourceClusterID,
	int iAlpha,
	int iBeta,
	int targetClusterID,
	CRVLMem *pMem)
{
	VN_::Limit *pLimit;

	RVLMEM_ALLOC_STRUCT(pMem, VN_::Limit, pLimit);

	QList<VN_::Limit> *pLimitList = &limitList;

	RVLQLIST_ADD_ENTRY(pLimitList, pLimit);

	pLimit->sourceClusterID = sourceClusterID;
	pLimit->iAlpha = iAlpha;
	pLimit->iBeta = iBeta;
	pLimit->targetClusterID = targetClusterID;
}

void VN::SetOutput(int outputIDIn)
{
	outputID = outputIDIn;
}

void VN::CreateEmpty()
{
	QList<VN_::ModelCluster> *pModelClusterList = &modelClusterList;

	RVLQLIST_INIT(pModelClusterList);

	QList<VN_::Operation> *pOperationList = &operationList;

	RVLQLIST_INIT(pOperationList);

	QList<VN_::Limit> *pLimitList = &limitList;

	RVLQLIST_INIT(pLimitList);
}

void VN::Create(CRVLMem *pMem)
{
	featureArray.n = 0;

	NodeArray.n = 0;

	int iBeta;

	VN_::ModelCluster *pMCluster = modelClusterList.pFirst;

	while (pMCluster)
	{
		for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
			featureArray.n += (pMCluster->betaArray.Element[iBeta] < 0.001f || pMCluster->betaArray.Element[iBeta] > PI - 0.001f ?
			1 : pMCluster->alphaArray.n);

		if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pMCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
			NodeArray.n += 1;
		else if (pMCluster->type == RVLVN_CLUSTER_TYPE_XTORUS || pMCluster->type == RVLVN_CLUSTER_TYPE_ITORUS)
			NodeArray.n += (pMCluster->betaArray.n + 1);

		pMCluster = pMCluster->pNext;
	}

	NodeArray.n += featureArray.n;

	VN_::Operation *pOperation = operationList.pFirst;

	while (pOperation)
	{
		NodeArray.n++;

		pOperation = pOperation->pNext;
	}

	RVL_DELETE_ARRAY(featureArray.Element);

	featureArray.Element = new VN_::Feature[featureArray.n];

	VN_::Feature *pFeature = featureArray.Element;

	int iAlpha;
	float N[3];
	float ca, sa, cb, sb;
	float *R, *t;
	VN_::Node *pNode;
	int iFeature;
	VN_::Feature *pFeature_;
	float alpha, beta;
	float P[3], P_[3];
	float a;
	float fOperation;

	pMCluster = modelClusterList.pFirst;

	while (pMCluster)
	{
		fOperation = (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX ? 1.0f : -1.0f);

		float *R = pMCluster->R;

		pMCluster->iFeatureInterval.a = pFeature - featureArray.Element;

		for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
		{
			if (pMCluster->betaArray.Element[iBeta] < 0.001f)
			{
				RVLCOPYCOLMX3X3(R, 2, pFeature->N);

				pFeature->iAlpha = 0;
				pFeature->iBeta = iBeta;
			
				pFeature++;
			}
			else if (pMCluster->betaArray.Element[iBeta] > PI - 0.001f)
			{
				RVLCOPYCOLMX3X3(R, 2, N);

				RVLNEGVECT3(N, pFeature->N);
				
				pFeature->iAlpha = 0;
				pFeature->iBeta = iBeta;

				pFeature++;
			}
			else
			{
				cb = cos(pMCluster->betaArray.Element[iBeta]);
				sb = sin(pMCluster->betaArray.Element[iBeta]);

				for (iAlpha = 0; iAlpha < pMCluster->alphaArray.n; iAlpha++, pFeature++)
				{
					ca = cos(pMCluster->alphaArray.Element[iAlpha]);
					sa = sin(pMCluster->alphaArray.Element[iAlpha]);

					N[0] = ca * sb;
					N[1] = sa * sb;
					N[2] = cb;

					RVLMULMX3X3VECT(R, N, pFeature->N);

					pFeature->iAlpha = iAlpha;
					pFeature->iBeta = iBeta;
				}
			}
		}

		pMCluster->iFeatureInterval.b = pFeature - featureArray.Element - 1;

		t = pMCluster->t;

		for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
		{
			pFeature_ = featureArray.Element + iFeature;

			if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pMCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
				pFeature_->d = RVLDOTPRODUCT3(pFeature_->N, pMCluster->t) + fOperation * pMCluster->r;
			else if (pMCluster->type == RVLVN_CLUSTER_TYPE_XTORUS || pMCluster->type == RVLVN_CLUSTER_TYPE_ITORUS)
			{
				alpha = pMCluster->alphaArray.Element[pFeature_->iAlpha];
				beta = pMCluster->betaArray.Element[pFeature_->iBeta];
				ca = cos(alpha);
				sa = sin(alpha);
				cb = cos(beta);
				sb = sin(beta);
				P[2] = pMCluster->r * cb;
				a = pMCluster->rT + pMCluster->r * (1.0f - sb);
				P[0] = -a * ca;
				P[1] = -a * sa;
				RVLTRANSF3(P, R, t, P_);
				pFeature_->d = RVLDOTPRODUCT3(pFeature_->N, P_);
			}
		}

		pMCluster = pMCluster->pNext;
	}

	RVL_DELETE_ARRAY(NodeArray.Element);

	NodeArray.Element = new VN_::Node[NodeArray.n];

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		pNode = NodeArray.Element + iFeature;

		pNode->operation = 0;
		pNode->fOperation = 0.0f;
		pNode->iFeature = iFeature;
		pNode->pFeature = featureArray.Element + iFeature;
	}

	QList<VN_::Edge> *pEdgeList = &EdgeList;

	RVLQLIST_INIT(pEdgeList);

	int iNode = featureArray.n;

	iFeature = 0;

	VN_::Edge *pEdge;
	bool bTorus;
	int iRingNode;

	pMCluster = modelClusterList.pFirst;

	while (pMCluster)
	{
		bTorus = (pMCluster->type == RVLVN_CLUSTER_TYPE_XTORUS || pMCluster->type == RVLVN_CLUSTER_TYPE_ITORUS);

		if (!bTorus)
		{
			pNode = NodeArray.Element + iNode;

			pNode->operation = (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX ? 1 : -1);
			pNode->fOperation = (float)(pNode->operation);
			pNode->iFeature = -1;
		}

		for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
		{
			if (bTorus)
			{
				pNode = NodeArray.Element + iNode;

				pNode->operation = (pMCluster->type == RVLVN_CLUSTER_TYPE_ITORUS ? 1 : -1);
				pNode->fOperation = (float)(pNode->operation);
				pNode->iFeature = -1;
			}

			if (pMCluster->betaArray.Element[iBeta] < 0.001f)
			{
				RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

				RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);

				pEdge->data.a = iFeature;
				pEdge->data.b = iNode;
				pEdge->bPrimary = true;

				iFeature++;
			}
			else if (pMCluster->betaArray.Element[iBeta] > PI - 0.001f)
			{
				RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

				RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);

				pEdge->data.a = iFeature;
				pEdge->data.b = iNode;
				pEdge->bPrimary = true;

				iFeature++;
			}
			else
			{
				for (iAlpha = 0; iAlpha < pMCluster->alphaArray.n; iAlpha++)
				{
					RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

					RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);

					pEdge->data.a = iFeature;
					pEdge->data.b = iNode;
					pEdge->bPrimary = true;

					iFeature++;
				}
			}

			if (bTorus)
				iNode++;
		}

		if (bTorus)
		{
			pNode = NodeArray.Element + iNode;

			pNode->operation = (pMCluster->type == RVLVN_CLUSTER_TYPE_XTORUS ? 1 : -1);
			pNode->fOperation = (float)(pNode->operation);
			pNode->iFeature = -1;

			iRingNode = iNode - pMCluster->betaArray.n;

			for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++, iRingNode++)
			{
				RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

				RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);

				pEdge->data.a = iRingNode;
				pEdge->data.b = iNode;
				pEdge->bPrimary = true;
			}
		}

		pMCluster->iNode = iNode;

		iNode++;

		pMCluster = pMCluster->pNext;
	}

	VN_::Limit *pLimit = limitList.pFirst;

	VN_::Edge *pEdge_;
	VN_::ModelCluster *pMCluster_;

	while (pLimit)
	{
		if (((pMCluster = GetModelCluster(pLimit->sourceClusterID)) != NULL) &&
			((pMCluster_ = GetModelCluster(pLimit->targetClusterID)) != NULL))
		{
			for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
			{
				pFeature = featureArray.Element + iFeature;

				if (pFeature->iAlpha == pLimit->iAlpha && pFeature->iBeta == pLimit->iBeta)
				{
					RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

					RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);

					pEdge->data.a = iFeature;
					pEdge->data.b = pMCluster_->iNode;
					pEdge->bPrimary = false;

					break;
				}
			}
		}

		pLimit = pLimit->pNext;
	}

	pOperation = operationList.pFirst;

	while (pOperation)
	{
		pNode = NodeArray.Element + iNode;

		pNode->operation = pOperation->operation;
		pNode->fOperation = (float)(pNode->operation);
		pNode->iFeature = -1;

		pOperation->iNode = iNode;

		iNode++;

		pOperation = pOperation->pNext;
	}

	int iOperand;
	VN_::Operation *pOperation_;

	pOperation = operationList.pFirst;

	while (pOperation)
	{
		for (iOperand = 0; iOperand < 2; iOperand++)
		{
			RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

			RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);

			pEdge->data.a = -1;
			pEdge->data.b = pOperation->iNode;
			pEdge->bPrimary = true;

			if(pMCluster = GetModelCluster(pOperation->operand[iOperand]))
				pEdge->data.a = pMCluster->iNode;
			
			if (pEdge->data.a < 0)
			{
				if (pOperation_ = GetOperation(pOperation->operand[iOperand]))
					pEdge->data.a = pOperation_->iNode;
			}
		}

		pOperation = pOperation->pNext;
	}

	pMCluster = GetModelCluster(outputID);

	if (pMCluster)
		iy = pMCluster->iNode;
	else
	{
		pOperation = GetOperation(outputID);

		if (pOperation)
			iy = pOperation->iNode;
		else
			iy = -1;
	}

	projectionIntervals.n = NodeArray.n;

	RVL_DELETE_ARRAY(projectionIntervals.Element);

	projectionIntervals.Element = new Array<Pair<float, float>>[projectionIntervals.n];

	RVL_DELETE_ARRAY(projectionIntervalMem);

	for (iNode = 0; iNode < featureArray.n; iNode++)
		projectionIntervals.Element[iNode].n = 1;

	for (; iNode < NodeArray.n; iNode++)
		projectionIntervals.Element[iNode].n = 0;

	pEdge = EdgeList.pFirst;

	while (pEdge)
	{
		projectionIntervals.Element[pEdge->data.b].n += projectionIntervals.Element[pEdge->data.a].n;

		pEdge = pEdge->pNext;
	}

	int maxnIntervals = 0;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
		maxnIntervals += projectionIntervals.Element[iNode].n;

	projectionIntervalMem = new Pair<float, float>[maxnIntervals];

	Pair<float, float> *pInterval = projectionIntervalMem;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		projectionIntervals.Element[iNode].Element = pInterval;

		pInterval += projectionIntervals.Element[iNode].n;
	}

	RVL_DELETE_ARRAY(projectionIntervalBuff.Element);

	projectionIntervalBuff.Element = new Pair<float, float >[featureArray.n];

	RVL_DELETE_ARRAY(dc);
	
	dc = new float[featureArray.n];
}

VN_::ModelCluster * VN::GetModelCluster(int ID)
{
	VN_::ModelCluster *pMCluster = modelClusterList.pFirst;

	while (pMCluster)
	{
		if (pMCluster->ID == ID)
			return pMCluster;

		pMCluster = pMCluster->pNext;
	}

	return NULL;
}

VN_::Operation * VN::GetOperation(int ID)
{
	VN_::Operation *pOperation_ = operationList.pFirst;

	while (pOperation_)
	{
		if (pOperation_->ID == ID)
			return pOperation_;

		pOperation_ = pOperation_->pNext;
	}

	return NULL;
}

void VN::Create(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	CRVLMem *pMem,
	float voxelSize_,
	int sampleVoxelDistance,
	float eps,
	Visualizer *pVisualizer)
{
	printf("Creating VN...");

	voxelSize = voxelSize_;

	Array<RECOG::VN_::Sample> sampleArray;
	float P0[3];

	SampleMeshDistanceFunction(pMesh, pSurfels, voxelSize, sampleVoxelDistance, volume, P0, sampleArray, boundingBox);

	Array<QList<QLIST::Index>> surfelSampleAssignmentArray;

	surfelSampleAssignmentArray.Element = new QList<QLIST::Index>[pSurfels->NodeArray.n];

	surfelSampleAssignmentArray.n = pSurfels->NodeArray.n;

	QList<QLIST::Index> *pSurfelSampleList;
	int i;

	for (i = 0; i < surfelSampleAssignmentArray.n; i++) 
	{
		pSurfelSampleList = surfelSampleAssignmentArray.Element + i; 
		RVLQLIST_INIT(pSurfelSampleList);
	}

	QLIST::Index *surfelSampleAssignmentMem = new QLIST::Index[sampleArray.n];

	QLIST::Index *pSampleIdx = surfelSampleAssignmentMem;

	int iSample, iFeature;

	for (iSample = 0; iSample < sampleArray.n; iSample++)
	{
		iFeature = sampleArray.Element[iSample].iFeature;

		if (iFeature < 0)
			continue;

		pSurfelSampleList = surfelSampleAssignmentArray.Element + iFeature;

		RVLQLIST_ADD_ENTRY(pSurfelSampleList, pSampleIdx);

		pSampleIdx->Idx = iSample;

		pSampleIdx++;
	}

	Array<SortIndex<int>> sortedFeaturesArray;

	sortedFeaturesArray.Element = new SortIndex<int>[pSurfels->NodeArray.n];

	sortedFeaturesArray.n = pSurfels->NodeArray.n;

	for (iFeature = 0; iFeature < sortedFeaturesArray.n; iFeature++)
	{
		sortedFeaturesArray.Element[iFeature].idx = iFeature;
		sortedFeaturesArray.Element[iFeature].cost = pSurfels->NodeArray.Element[iFeature].size;
	}

	BubbleSort<SortIndex<int>>(sortedFeaturesArray, true);

	for (i = sortedFeaturesArray.n - 1; i >= 0; i--)
		if (sortedFeaturesArray.Element[i].cost > 1)
			break;

	sortedFeaturesArray.n = i;

	Array<int> sortedSampleArray;

	sortedSampleArray.Element = new int[sampleArray.n];

	sortedSampleArray.n = 0;

	for (i = 0; i < sortedFeaturesArray.n; i++)
	{
		iFeature = sortedFeaturesArray.Element[i].idx;

		pSurfelSampleList = surfelSampleAssignmentArray.Element + iFeature;

		pSampleIdx = pSurfelSampleList->pFirst;

		while (pSampleIdx)
		{
			sortedSampleArray.Element[sortedSampleArray.n++] = pSampleIdx->Idx;

			pSampleIdx = pSampleIdx->pNext;
		}
	}

	pFeatures = pSurfels;

	RVL_DELETE_ARRAY(NodeArray.Element);

	NodeArray.Element = new VN_::Node[sortedSampleArray.n];

	int *featureNodeMap = new int[pSurfels->NodeArray.n];

	memset(featureNodeMap, 0xff, pSurfels->NodeArray.n * sizeof(int));

	featureArray.n = 0;

	VN_::Node *pNode;

	for (i = 0; i < sortedFeaturesArray.n; i++)
	{
		iFeature = sortedFeaturesArray.Element[i].idx;

		pSurfelSampleList = surfelSampleAssignmentArray.Element + iFeature;

		if (pSurfelSampleList->pFirst)
		{
			pNode = NodeArray.Element + featureArray.n;

			pNode->operation = 0;
			pNode->iFeature = iFeature;

			featureNodeMap[iFeature] = featureArray.n;

			featureArray.n++;
		}
	}

	NodeArray.n = featureArray.n;

	RVL_DELETE_ARRAY(featureArray.Element);

	featureArray.Element = new RECOG::VN_::Feature[featureArray.n];

	RECOG::VN_::Feature *pFeature_ = featureArray.Element;
	float *N;
	Surfel *pFeature;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++, pFeature_++)
	{
		pNode = NodeArray.Element + iFeature;

		pFeature = pSurfels->NodeArray.Element + pNode->iFeature;

		N = pFeature->N;

		RVLCOPY3VECTOR(N, pFeature_->N);

		pFeature_->d = pFeature->d;

		pNode->pFeature = pFeature_;
	}

	QList<RECOG::VN_::Edge> *pEdgeList = &EdgeList;

	RVLQLIST_INIT(pEdgeList);

	float *SDF_ = new float[featureArray.n * sortedSampleArray.n];

	float *P;

	for (i = 0; i < sortedSampleArray.n; i++)
	{
		P = sampleArray.Element[sortedSampleArray.Element[i]].P;

		ComputeFeatureSDFs(P, SDF_ + i * featureArray.n);
	}

	iSample = sortedSampleArray.Element[0];

	VN_::Sample *pSample = sampleArray.Element + iSample;

	Array<int> cluster;

	cluster.Element = new int[featureArray.n];

	bool *bInCluster = new bool[featureArray.n];

	memset(bInCluster, 0, featureArray.n * sizeof(bool));

	iy = pSample->iFeature;

	int operation;
	float fOperation;
	bool bAllSamplesPreserved;
	int iSample_;
	VN_::Sample *pSample_;
	int iBestCandidate;
	float eSDF_, eSDFBestCandidate;
	int iyPrev;
	RECOG::VN_::Edge *pEdge;
	int iClusterNode;
	int iActiveFeature;
	float SDF, eSDF;

	for (i = 1; i < sortedSampleArray.n; i++)
	{
		//if (i == 3639)
		//	int debug = 0;

		iSample = sortedSampleArray.Element[i];

		pSample = sampleArray.Element + iSample;

		SDF = Evaluate(NULL, SDF_ + i * featureArray.n, iActiveFeature, false);

		if (iActiveFeature == pSample->iFeature)
			continue;

		eSDF = SDF - pSample->SDF;

		if (eSDF < -eps)
			operation = 1;
		else if (eSDF < eps)
			continue;
		else
			operation = -1;

		fOperation = (float)operation;
			
		bAllSamplesPreserved = true;
		cluster.Element[0] = featureNodeMap[pSample->iFeature];
		cluster.n = 1;
		bInCluster[cluster.Element[0]] = true;

		int j, k, l;

		for (j = 0; j < i; j++)
		{
			SDF = SDF_[cluster.Element[0] + j * featureArray.n];

			if (operation > 0)
			{
				for (k = 1; k < cluster.n; k++)
				{
					l = cluster.Element[k] + j * featureArray.n;

					if (SDF_[l] < SDF)
						SDF = SDF_[l];
				}
			}
			else
			{
				for (k = 1; k < cluster.n; k++)
				{
					l = cluster.Element[k] + j * featureArray.n;

					if (SDF_[l] > SDF)
						SDF = SDF_[l];
				}
			}

			iSample_ = sortedSampleArray.Element[j];

			pSample_ = sampleArray.Element + iSample_;

			eSDF = SDF - pSample_->SDF;

			if (fOperation * eSDF > eps)
			{
				iBestCandidate = -1;

				for (k = 0; k < featureArray.n; k++)
				{
					eSDF = fOperation * (SDF_[k + i * featureArray.n] - pSample->SDF);
					eSDF_ = fOperation * (SDF_[k + j * featureArray.n] - pSample_->SDF);

					if (eSDF > 0.0f && (eSDF_ < 0.0f || NodeArray.Element[k].iFeature == pSample_->iFeature))
					{
						if (iBestCandidate >= 0)
						{
							if (eSDF < eSDFBestCandidate)
							{
								eSDFBestCandidate = eSDF;
								iBestCandidate = k;
							}					
						}
						else
						{
							eSDFBestCandidate = eSDF;
							iBestCandidate = k;
						}
					}
				}

				if (iBestCandidate >= 0)
				{
					if (!bInCluster[iBestCandidate])
					{
						cluster.Element[cluster.n++] = iBestCandidate;

						bInCluster[iBestCandidate] = true;

						//if (cluster.n > nFeatures)
						//	int debug = 0;
					}
				}
				else
				{
					bAllSamplesPreserved = false;

					break;
				}
			}	// if (fOperation * eSDF > eps)
		}	// for (j = 0; j < i; j++)

		for (j = 0; j < cluster.n; j++)
			bInCluster[cluster.Element[j]] = false;

		if (bAllSamplesPreserved)
		{
			if (operation != NodeArray.Element[iy].operation)
			{
				iyPrev = iy;

				iy = NodeArray.n;

				NodeArray.Element[iy].operation = operation;

				NodeArray.n++;

				RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

				pEdge->data.a = iyPrev;
				pEdge->data.b = iy;

				RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);
			}

			if (cluster.n > 1)
			{
				iClusterNode = NodeArray.n;

				NodeArray.Element[iClusterNode].operation = -operation;

				NodeArray.n++;
			}
			else
				iClusterNode = iy;

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, VN_::Edge, cluster.n, pEdge);

			for (j = 0; j < cluster.n; j++, pEdge++)
			{
				pEdge->data.a = cluster.Element[j];
				pEdge->data.b = iClusterNode;

				RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);
			}

			if (iClusterNode != iy)
			{
				RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

				pEdge->data.a = iClusterNode;
				pEdge->data.b = iy;

				RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);
			}
		}
	}	// for (i = 1; i < sortedSampleArray.n; i++)

	printf("completed.\n");

	printf("no. of nodes=%d (no. of features=%d)", NodeArray.n, featureArray.n);

	FILE *fp = fopen("VNError.txt", "w");

	for (i = 0; i < sortedSampleArray.n; i++)
	{
		SDF = Evaluate(NULL, SDF_ + i * featureArray.n, iActiveFeature, false);

		fprintf(fp, "%f\n", SDF);
	}

	fclose(fp);

	//if (pVisualizer)
	//{
	//	DisplaySampledMesh(pVisualizer, volume, P0, voxelSize);

	//	unsigned char color[] = { 0, 128, 255 };

	//	pVisualizer->DisplayPointSet<float, VN_::Sample>(sampleArray, color, 6.0f);
	//}

	delete[] volume.Element;
	delete[] sampleArray.Element;
	delete[] surfelSampleAssignmentArray.Element;
	delete[] surfelSampleAssignmentMem;
	delete[] sortedFeaturesArray.Element;
	delete[] sortedSampleArray.Element;
	delete[] SDF_;
	delete[] cluster.Element;
	delete[] featureNodeMap;
	delete[] bInCluster;
}

void VN::ComputeFeatureSDFs(
	float *P,
	float *SDF,
	float *d)
{
	int iNode;
	VN_::Node *pNode;
	float *N;

	for (iNode = 0; iNode < featureArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		N = pNode->pFeature->N;

		SDF[iNode] = RVLDOTPRODUCT3(N, P) - (d ? d[iNode] : pNode->pFeature->d);
	}
}

float VN::Evaluate(
	float *P,
	float *SDF,
	int &iActiveFeature,
	bool bComputeSDFs,
	float *d,
	bool *bd
	)
{
	if (bComputeSDFs)
		ComputeFeatureSDFs(P, SDF, d);
	
	int iNode;
	VN_::Node *pNode;

	for (iNode = 0; iNode < featureArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pNode->output = SDF[iNode];
		pNode->iActiveFeature = pNode->iFeature;

		pNode->bOutput = (bd ? bd[iNode] : true);
	}

	for (; iNode < NodeArray.n; iNode++)
		NodeArray.Element[iNode].bOutput = false;

	RECOG::VN_::Edge *pEdge = EdgeList.pFirst;

	iActiveFeature = iy;

	VN_::Node *pChildNode, *pParentNode;

	while (pEdge)
	{
		pChildNode = NodeArray.Element + pEdge->data.a;

		if (pChildNode->bOutput)
		{
			pParentNode = NodeArray.Element + pEdge->data.b;

			if (pParentNode->bOutput)
			{
				if (pParentNode->operation < 0)
				{
					if (pChildNode->output < pParentNode->output)
					{
						pParentNode->output = pChildNode->output;
						pParentNode->iActiveFeature = pChildNode->iActiveFeature;
					}
				}
				else
				{
					if (pChildNode->output > pParentNode->output)
					{
						pParentNode->output = pChildNode->output;
						pParentNode->iActiveFeature = pChildNode->iActiveFeature;
					}
				}
			}
			else
			{
				pParentNode->output = pChildNode->output;
				pParentNode->iActiveFeature = pChildNode->iActiveFeature;
				pParentNode->bOutput = true;
			}
		}

		pEdge = pEdge->pNext;
	}

	iActiveFeature = NodeArray.Element[iy].iActiveFeature;

	return NodeArray.Element[iy].output;
}

void VN::Match(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Box<float> boundingBox,
	RECOG::VN_::Parameters params,
	float *dS,
	bool *bdS)
{
	float kNodeCorrespClusteringThr = 0.01f;
	float kMeanShiftTol = 0.01f;
	float kMaxMatchCost = params.kMaxMatchCost;
	float kMinMatchCostDiff = 0.001f;
	int maxnGITNodes = 10000;

#ifdef RVLVN_MATCH_DEBUG
	FILE *fp = fopen("VNMatch.log", "w");
#endif

	float size = GetMeshSize(boundingBox);

	float nodeCorrespClusteringThr = kNodeCorrespClusteringThr * size;
	float meanShiftTol = kMeanShiftTol * nodeCorrespClusteringThr;
	float minMatchCostDiff = kMinMatchCostDiff * size;

	CRVLMem mem;

	mem.Create(10000000);

	CRVLMem *pMem2 = &mem;

	Array<VN_::Correspondence> nodeCorrespArray;

	nodeCorrespArray.Element = new VN_::Correspondence[pSurfels->vertexArray.n];

	bool *bOwnedByRefVertex = new bool[pSurfels->NodeArray.n];

	memset(bOwnedByRefVertex, 0, pSurfels->NodeArray.n * sizeof(bool));

	Array<Array<VN_::SceneFeature>> correspondenceArray;

	correspondenceArray.Element = new Array<VN_::SceneFeature>[featureArray.n];

	VN_::Correspondence *pCorresp, *pCorresp_;
	int iNode, iVertex;
	RECOG::VN_::Node *pNode;
	float *N;
	SURFEL::Vertex *pVertex, *pVertex_;
	float distFromNormalHull;
	float d, dist;
	int i, j, k, nd;
	float meand, sumd;
	bool bChange;
	Array<VN_::SceneFeature> *pCorrespondenceArray;
	QList<QLIST::Index> *pVertexList;
	QLIST::Index *vertexListMem, *pVertexListEntry;

	for (iNode = 0; iNode < featureArray.n; iNode++)
	{
#ifdef RVLVN_MATCH_DEBUG
		fprintf(fp, "%d:", iNode);
#endif
		pNode = NodeArray.Element + iNode;

		N = pNode->pFeature->N;

		pCorresp = nodeCorrespArray.Element;

		for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
		{
			pVertex = pSurfels->vertexArray.Element[iVertex];

			if (pVertex->normalHull.n < 3)
				continue;

			distFromNormalHull = pSurfels->DistanceFromNormalHull(pVertex->normalHull, N);
			
			if (distFromNormalHull > 0.0f)
				continue;

			d = RVLDOTPRODUCT3(N, pVertex->P);

			pCorresp->iVertex = iVertex;
			pCorresp->d = d;
			pCorresp->bMerged = pCorresp->bPrevMerged = false;
			pCorresp->iParent = -1;

			pCorresp++;
		}

		nodeCorrespArray.n = pCorresp - nodeCorrespArray.Element;

		for (i = 0; i < nodeCorrespArray.n; i++)
		{
			meand = nodeCorrespArray.Element[i].d;

			nodeCorrespArray.Element[i].bPrevMerged = true;

			do{
				sumd = 0.0f;
				nd = 0;

				for (j = 0; j < nodeCorrespArray.n; j++)
				{
					dist = nodeCorrespArray.Element[j].d - meand;

					if (RVLABS(dist) <= nodeCorrespClusteringThr)
					{
						sumd += nodeCorrespArray.Element[j].d;

						nd++;

						nodeCorrespArray.Element[j].bMerged = true;
					}
				}

				meand = sumd / (float)nd;

				bChange = false;

				for (j = 0; j < nodeCorrespArray.n; j++)
				{
					if (nodeCorrespArray.Element[j].bMerged != nodeCorrespArray.Element[j].bPrevMerged)
						bChange = true;

					nodeCorrespArray.Element[j].bPrevMerged = nodeCorrespArray.Element[j].bMerged;
				}
			} while (bChange);

			nodeCorrespArray.Element[i].dCluster = meand;

			for (j = 0; j < nodeCorrespArray.n; j++)
				nodeCorrespArray.Element[j].bPrevMerged = nodeCorrespArray.Element[j].bMerged = false;
		}

		for (i = 0; i < nodeCorrespArray.n; i++)
		{
			pCorresp = nodeCorrespArray.Element + i;

			if (pCorresp->bMerged || pCorresp->iParent >= 0)
				continue;

			pVertex = pSurfels->vertexArray.Element[pCorresp->iVertex];

			for (k = 0; k < pVertex->iSurfelArray.n; k++)
				bOwnedByRefVertex[pVertex->iSurfelArray.Element[k]] = true;

			for (j = i + 1; j < nodeCorrespArray.n; j++)
			{
				pCorresp_ = nodeCorrespArray.Element + j;

				if (pCorresp_->bMerged || pCorresp_->iParent >= 0)
					continue;

				dist = pCorresp->dCluster - pCorresp_->dCluster;

				if (RVLABS(dist) <= meanShiftTol)
				{
					pVertex_ = pSurfels->vertexArray.Element[pCorresp_->iVertex];

					for (k = 0; k < pVertex_->iSurfelArray.n; k++)
						if (bOwnedByRefVertex[pVertex_->iSurfelArray.Element[k]])
							break;

					if (k < pVertex_->iSurfelArray.n)
						pCorresp_->bMerged = true;
					else
						pCorresp_->iParent = i;
				}
			}

			for (k = 0; k < pVertex->iSurfelArray.n; k++)
				bOwnedByRefVertex[pVertex->iSurfelArray.Element[k]] = false;
		}

		pCorrespondenceArray = correspondenceArray.Element + iNode;

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, VN_::SceneFeature, nodeCorrespArray.n, pCorrespondenceArray->Element);

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, QLIST::Index, nodeCorrespArray.n, vertexListMem);

		pVertexListEntry = vertexListMem;

		pCorrespondenceArray->n = 0;

		for (i = 0; i < nodeCorrespArray.n; i++)
		{
			pCorresp = nodeCorrespArray.Element + i;

			pCorresp->iSceneFeature = -1;

			if (pCorresp->bMerged)
				continue;

			if (pCorresp->iParent < 0)
			{
				pVertexList = &(pCorrespondenceArray->Element[pCorrespondenceArray->n].iVertexList);

				RVLQLIST_INIT(pVertexList);

				pCorrespondenceArray->Element[pCorrespondenceArray->n].d = pCorresp->dCluster;

				pCorresp->iSceneFeature = pCorrespondenceArray->n;

				pCorrespondenceArray->n++;
			}
			else
				pVertexList = &(pCorrespondenceArray->Element[nodeCorrespArray.Element[pCorresp->iParent].iSceneFeature].iVertexList);

			RVLQLIST_ADD_ENTRY(pVertexList, pVertexListEntry);

			pVertexListEntry->Idx = pCorresp->iVertex;

			pVertexListEntry++;
		}

#ifdef RVLVN_MATCH_DEBUG
		for (i = 0; i < pCorrespondenceArray->n; i++)
		{
			fprintf(fp, "\t%f(", pCorrespondenceArray->Element[i].d);

			pVertexListEntry = pCorrespondenceArray->Element[i].iVertexList.pFirst;

			while (pVertexListEntry)
			{
				fprintf(fp, "%d ", pVertexListEntry->Idx);

				pVertexListEntry = pVertexListEntry->pNext;
			}

			fprintf(fp, ")\t");
		}

		fprintf(fp, "\n");
#endif
	}	// for every feature node

#ifdef RVLVN_MATCH_DEBUG
	fclose(fp);
#endif

	delete[] nodeCorrespArray.Element;
	delete[] bOwnedByRefVertex;

	///

	//float maxMatchCost = kMaxMatchCost * size;

	//MatchByBuildingInterpretationTree(pSurfels, correspondenceArray, maxMatchCost, minMatchCostDiff, maxnGITNodes, dS, bdS, pMem2);

	GeneticAlg(pMesh, pSurfels, correspondenceArray, dS, bdS);

#ifdef NEVER
	// Ground truth.

	VN_::Edge *pEdge = EdgeList.pFirst;

	int iFeature;
	float mind, maxd;

	while (pEdge)
	{
		iFeature = pEdge->data.a;

		if (iFeature == 82)
			int debug = 0;

		if (iFeature < featureArray.n)
		{
			if (pEdge->data.b == featureArray.n)
			{
				pCorrespondenceArray = correspondenceArray.Element + iFeature;

				maxd = pCorrespondenceArray->Element[0].d;

				for (i = 1; i < pCorrespondenceArray->n; i++)
					if (pCorrespondenceArray->Element[i].d > maxd)
						maxd = pCorrespondenceArray->Element[i].d;

				dS[iFeature] = maxd;
			}
			else
			{
				pCorrespondenceArray = correspondenceArray.Element + iFeature;

				mind = pCorrespondenceArray->Element[0].d;

				for (i = 1; i < pCorrespondenceArray->n; i++)
					if (pCorrespondenceArray->Element[i].d < mind)
						mind = pCorrespondenceArray->Element[i].d;

				dS[iFeature] = mind;
			}
		}

		pEdge = pEdge->pNext;
	}	
#endif

	delete[] correspondenceArray.Element;
}

void VN::Descriptor(
	VN_::ITPtr *ITPtr_,
	Array<Array<VN_::SceneFeature>> correspondenceArray,
	float *dS,
	bool *bdS)
{
	memset(dS, 0, featureArray.n * sizeof(float));
	memset(bdS, 0, featureArray.n * sizeof(bool));

	VN_::SceneFeature *pSFeature;
	int iCluster;
	VN_::ITNode *pITNode;

	for (iCluster = 0; iCluster < clusters.size(); iCluster++)
	{
		pITNode = ITPtr_[iCluster].pLITNode;

		while (pITNode)
		{
			if (pITNode->iLevel >= 0)
			{
				pSFeature = correspondenceArray.Element[pITNode->iNode].Element + pITNode->iCorrespondence;

				dS[pITNode->iNode] = pSFeature->d;

				bdS[pITNode->iNode] = true;
			}

			pITNode = pITNode->pParent;
		}
	}
}

float VN::Distance(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	float *dS,
	bool *bdS,
	int &iMaxErrVertex,
	float *SDF)
{
	float maxe = 0.0f;

	iMaxErrVertex = -1;

	int i;
	float *P;
	float e;
	int iActiveFeature;
	int iVertex;

	for (i = 0; i < iVertexArray.n; i++)
	{
		iVertex = iVertexArray.Element[i];

		P = pSurfels->vertexArray.Element[iVertex]->P;

		e = Evaluate(P, SDF, iActiveFeature, true, dS, bdS);

		if (e < 0.0f)
			e = -e;

		if (e > maxe)
		{
			maxe = e;

			iMaxErrVertex = iVertex;
		}
	}

	return maxe;
}

void VN::InitMatchCluster(RECOG::VN_::Queue &Q)
{
	QList<VN_::ITNode> **&queue = Q.queue;
	CRVLMem *pMem2 = Q.pMem;

	Q.nMatchCostLevels = (int)ceil(Q.maxMatchCost / Q.minMatchCostDiff);

	queue = new QList<VN_::ITNode> *[Q.nMatchCostLevels];

	memset(queue, 0, Q.nMatchCostLevels * sizeof(QList<VN_::ITNode> *));

	QList<VN_::ITNode> *pQueueBin;

	RVLMEM_ALLOC_STRUCT(pMem2, QList<VN_::ITNode>, pQueueBin);

	queue[0] = pQueueBin;

	RVLQLIST_INIT(pQueueBin);

	VN_::ITNode *pITNode;

	RVLMEM_ALLOC_STRUCT(pMem2, VN_::ITNode, pITNode);

	pITNode->pParent = NULL;
	pITNode->iLevel = -1;
	pITNode->e = 0.0f;
	pITNode->bExpanded = false;

	RVLQLIST_ADD_ENTRY(pQueueBin, pITNode);

	Q.iTopQueueBin = 0;
	Q.iBottomQueueBin = 0;
}

bool VN::MatchCluster(
	VN_::Cluster *pCluster,
	Array<Array<VN_::SceneFeature>> correspondenceArray,
	SurfelGraph *pSurfels,
	RECOG::VN_::ITNode *pITNodeIn,
	VN_::Queue &Q,
	VN_::ITNode *&pITNodeOut
	)
{
#ifdef RVLVN_MATCH_DEBUG
	FILE *fp = fopen("VNMatchCluster.txt", "w");
#endif

	QList<VN_::ITNode> **&queue = Q.queue;
	CRVLMem *pMem2 = Q.pMem;

	float fOperation = NodeArray.Element[pCluster->iParent].fOperation;

	bool bCompleted = false;

	int nITNodes = 0;

	VN_::ITNode *pITNode = pITNodeIn;

	int i;
	int iNode, iLevel;
	float mine, maxe;
	Array<VN_::SceneFeature> *pCorrespondenceArray__;
	float d, e;
	float *P;
	int iNode__;
	VN_::Node *pNode, *pNode__;
	int iBin;
	float *N, *N__;
	bool bFirstInMin;
	VN_::SceneFeature *pSFeature;
	Array<VN_::SceneFeature> *pCorrespondenceArray;
	QLIST::Index *pVertexListEntry;
	VN_::ITNode *pITNode_, *pITNode__;
	QList<VN_::ITNode> *pQueueBin;

	while (true)
	{
		RVLVN_GET_NEXT_QUEUE_ENTRY(queue, pITNode, pITNode, Q.iTopQueueBin, Q.iBottomQueueBin, bCompleted, pQueueBin);

		if (bCompleted)
			break;

#ifdef RVLVN_MATCH_DEBUG
		fprintf(fp, "Expanding node %d instance %d:\n", pITNode->iNode, pITNode->iCorrespondence);
#endif

		//pQueueBin->pFirst = pITNode->pNext;

		//if (pQueueBin->pFirst == NULL)
		//	pQueueBin->ppNext = &(pQueueBin->pFirst);

		iLevel = pITNode->iLevel + 1;

		//printf("%d\n", Q.iTopQueueBin);

		//if (iLevel == 66)
		//	int debug = 0;

		if (iLevel == pCluster->iChild.size())
		{
			pITNodeOut = pITNode;

			break;
		}

		if (!pITNode->bExpanded)
		{
			pITNode->bExpanded = true;

			iNode = pCluster->iChild.at(iLevel);

			pNode = NodeArray.Element + iNode;

			N = pNode->pFeature->N;

			pCorrespondenceArray = correspondenceArray.Element + iNode;

			for (i = 0; i < pCorrespondenceArray->n; i++)
			{
				// Only for debugging purpose!!!

				//if (pCluster->iParent = clusters.at(0).iParent)
				//{
				//	if (iNode >= 65 && iNode <= 80)
				//		if (pCorrespondenceArray->Element[i].d > 0.1f)
				//			continue;
				//}

				//if (iNode == 65)
				//	int debug = 0;

				//

				RVLMEM_ALLOC_STRUCT(pMem2, VN_::ITNode, pITNode_);

				nITNodes++;

				pITNode_->iNode = iNode;
				pITNode_->iCorrespondence = i;
				pITNode_->pParent = pITNode;
				pITNode_->iLevel = iLevel;
				pITNode_->bExpanded = false;

				pSFeature = pCorrespondenceArray->Element + i;

				d = pSFeature->d;

				bFirstInMin = true;

				mine = 0.0f;

				pVertexListEntry = pSFeature->iVertexList.pFirst;

				while (pVertexListEntry)
				{
					P = pSurfels->vertexArray.Element[pVertexListEntry->Idx]->P;

					maxe = 0.0f;

					pITNode__ = pITNode_;

					while (pITNode__)
					{
						if (pITNode__->iLevel >= 0)
						{
							iNode__ = pITNode__->iNode;

							pNode__ = NodeArray.Element + iNode__;

							N__ = pNode__->pFeature->N;

							pCorrespondenceArray__ = correspondenceArray.Element + iNode__;

							pSFeature = pCorrespondenceArray__->Element + pITNode__->iCorrespondence;

							e = fOperation * (RVLDOTPRODUCT3(N__, P) - pSFeature->d);

							if (e > maxe)
								maxe = e;
						}

						pITNode__ = pITNode__->pParent;
					}

					if (maxe < mine || bFirstInMin)
						mine = maxe;

					bFirstInMin = false;

					pVertexListEntry = pVertexListEntry->pNext;
				}

				pITNode_->e = RVLMAX(pITNode->e, mine);

				pITNode__ = pITNode_->pParent;

				while (pITNode__)
				{
					if (pITNode__->iLevel >= 0)
					{
						iNode__ = pITNode__->iNode;

						pCorrespondenceArray__ = correspondenceArray.Element + iNode__;

						pSFeature = pCorrespondenceArray__->Element + pITNode__->iCorrespondence;

						bFirstInMin = true;

						mine = 0.0f;

						pVertexListEntry = pSFeature->iVertexList.pFirst;

						while (pVertexListEntry)
						{
							P = pSurfels->vertexArray.Element[pVertexListEntry->Idx]->P;

							e = fOperation * (RVLDOTPRODUCT3(N, P) - d);

							if (e >= 0.0f)
							{
								if (bFirstInMin || e < mine)
								{
									mine = e;

									bFirstInMin = false;
								}
							}

							pVertexListEntry = pVertexListEntry->pNext;
						}

						if (mine > pITNode_->e)
							pITNode_->e = mine;
					}

					pITNode__ = pITNode__->pParent;
				}

#ifdef RVLVN_MATCH_DEBUG
				fprintf(fp, "%d,%d: e=%f\n", iNode, i, pITNode_->e);
#endif

				RVLVN_ADD_QUEUE_ENTRY(pITNode_, VN_::ITNode, pITNode_->e, queue, Q.minMatchCostDiff, Q.nMatchCostLevels, Q.iTopQueueBin, Q.iBottomQueueBin, pMem2, iBin, pQueueBin);
			}	// for each node instance
		}	// if(!pITNode->bExpanded)

#ifdef RVLVN_MATCH_DEBUG
		fprintf(fp, "\n");
#endif
	}	// main loop

#ifdef RVLVN_MATCH_DEBUG
	fclose(fp);
#endif

	return !bCompleted;
}

void VN::MatchByBuildingInterpretationTree(
	SurfelGraph *pSurfels,
	Array<Array<VN_::SceneFeature>> correspondenceArray,
	float maxMatchCost,
	float minMatchCostDiff,
	int maxnGITNodes,
	float *dS,
	bool *bdS,
	CRVLMem *pMem2)
{
	ModelClusters();

	int nClusters = clusters.size();

	int maxClusterSize = clusters.at(0).iChild.size();

	int iLargestCluster = 0;

	int iCluster;
	int clusterSize;

	for (iCluster = 1; iCluster < nClusters; iCluster++)
	{
		clusterSize = clusters.at(iCluster).iChild.size();

		if (clusterSize > maxClusterSize)
		{
			maxClusterSize = clusterSize;
			iLargestCluster = iCluster;
		}
	}

	VN_::Queue *Q = new VN_::Queue[nClusters];

	bool bAllClustersMatched = true;

	float *eCluster = new float[nClusters];
	VN_::ITPtr *pITPtrCluster = new VN_::ITPtr[nClusters];

	VN_::Cluster cluster;
	bool bClusterMatched;
	VN_::ITNode *pITNode;
	VN_::ITNode initITNode;

	//for (i = 0; i < 2; i++)
	for (iCluster = 0; iCluster < nClusters; iCluster++)
	{
		//iCluster = (i == 0 ? iLargestCluster : 0);

		cluster = clusters.at(iCluster);

		//Q[iCluster].maxMatchCost = size * (float)(pSurfels->vertexArray.n);
		Q[iCluster].maxMatchCost = maxMatchCost;
		Q[iCluster].minMatchCostDiff = minMatchCostDiff;
		Q[iCluster].pMem = pMem2;

		InitMatchCluster(Q[iCluster]);

		initITNode.pNext = Q[iCluster].queue[0]->pFirst;

		bClusterMatched = MatchCluster(&cluster, correspondenceArray, pSurfels, &initITNode, Q[iCluster], pITNode);

		pITPtrCluster[iCluster].pLITNode = pITNode;
		pITPtrCluster[iCluster].iQueueBin = Q[iCluster].iTopQueueBin;

		if (bClusterMatched)
			eCluster[iCluster] = pITNode->e;
		else
			bAllClustersMatched = false;
	}

	int nMatchCostLevels = Q[0].nMatchCostLevels;

	float *SDF = new float[featureArray.n];

	Array<int> iVertexArray;

	iVertexArray.n = pSurfels->vertexArray.n;

	iVertexArray.Element = new int[iVertexArray.n];

	int i;

	for (i = 0; i < iVertexArray.n; i++)
		iVertexArray.Element[i] = i;

	Array<int> iCriticalVertexArray;

	iCriticalVertexArray.Element = new int[pSurfels->vertexArray.n];
	iCriticalVertexArray.n = 0;

	int iMaxErrVertex;

	QList<VN_::GITNode> **GQueue = new QList<VN_::GITNode> *[nMatchCostLevels];

	memset(GQueue, 0, nMatchCostLevels * sizeof(QList<VN_::GITNode> *));

	QList<VN_::GITNode> *pQueueBin;

	RVLMEM_ALLOC_STRUCT(pMem2, QList<VN_::GITNode>, pQueueBin);

	GQueue[0] = pQueueBin;

	RVLQLIST_INIT(pQueueBin);

	VN_::GITNode *pGITNode;

	RVLMEM_ALLOC_STRUCT(pMem2, VN_::GITNode, pGITNode);

	pGITNode->pParent = NULL;
	pGITNode->e = 0.0f;
	pGITNode->ITPtr_ = pITPtrCluster;

	RVLQLIST_ADD_ENTRY(pQueueBin, pGITNode);

	int iTopGQueueBin = 0;
	int iBottomGQueueBin = 0;

	VN_::GITNode *pGITNodeBest = pGITNode;

	Descriptor(pGITNode->ITPtr_, correspondenceArray, dS, bdS);

	float minmaxe = -1.0f;
	float maxe = 0.0f;

	int nGITNodes = 0;

	bool bCompleted;
	int iCluster_;
	VN_::ITPtr *pITPtr_;
	int iBin;
	VN_::GITNode *pGITNode_;
	float minmaxe_;
	VN_::GITNode *pGITNodeSuboptimal;

	do
	{
		minmaxe_ = -1.0f;
		//int maxDebugCounter = 0;

		while (maxe > maxMatchCost)
		{
			// Only for debugging purpose!

			//int debugCounter = 0;

			//VN_::ITNode *pDebugITNode = pGITNode->ITPtr_[0].pLITNode;

			//while (pDebugITNode)
			//{
			//	if (pDebugITNode->iNode >= 65 && pDebugITNode->iNode <= 80)
			//	{
			//		if (correspondenceArray.Element[pDebugITNode->iNode].Element[pDebugITNode->iCorrespondence].d < 0.1f)
			//			debugCounter++;

			//		printf("%5.3f ", correspondenceArray.Element[pDebugITNode->iNode].Element[pDebugITNode->iCorrespondence].d);
			//	}

			//	pDebugITNode = pDebugITNode->pParent;
			//}

			//printf("\n");

			//if (debugCounter > maxDebugCounter)
			//{
			//	maxDebugCounter = debugCounter;

			//	printf("maxDebugCounter=%d\n", maxDebugCounter);
			//}

			///

			for (iCluster = 0; iCluster < nClusters; iCluster++)
			{
				cluster = clusters.at(iCluster);

				pITPtr_ = pGITNode->ITPtr_ + iCluster;

				pITNode = pITPtr_->pLITNode;
				Q[iCluster].iTopQueueBin = pITPtr_->iQueueBin;

				bClusterMatched = MatchCluster(&cluster, correspondenceArray, pSurfels, pITNode, Q[iCluster], pITNode);

				RVLMEM_ALLOC_STRUCT(pMem2, VN_::GITNode, pGITNode_);

				nGITNodes++;

				if (nGITNodes % 1000 == 0)
					printf(".");

				pGITNode_->pParent = pGITNode;
				pGITNode_->e = 0.0f;

				RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, VN_::ITPtr, nClusters, pGITNode_->ITPtr_);

				for (iCluster_ = 0; iCluster_ < nClusters; iCluster_++)
				{
					if (iCluster_ == iCluster)
					{
						pGITNode_->ITPtr_[iCluster_].pLITNode = pITNode;
						pGITNode_->ITPtr_[iCluster_].iQueueBin = Q[iCluster].iTopQueueBin;

						pGITNode_->e += (pITNode->e - eCluster[iCluster]);
					}
					else
					{
						pGITNode_->ITPtr_[iCluster_] = pGITNode->ITPtr_[iCluster_];

						pGITNode_->e += (pGITNode_->ITPtr_[iCluster_].pLITNode->e - eCluster[iCluster_]);
					}
				}

				RVLVN_ADD_QUEUE_ENTRY(pGITNode_, VN_::GITNode, pGITNode_->e, GQueue, minMatchCostDiff, nMatchCostLevels, iTopGQueueBin, iBottomGQueueBin, pMem2, iBin, pQueueBin)\
			}

			RVLVN_GET_NEXT_QUEUE_ENTRY(GQueue, pGITNode, pGITNode, iTopGQueueBin, iBottomGQueueBin, bCompleted, pQueueBin);

			Descriptor(pGITNode->ITPtr_, correspondenceArray, dS, bdS);

			maxe = Distance(pSurfels, iCriticalVertexArray, dS, bdS, iMaxErrVertex, SDF);

			if (minmaxe_ < 0.0f || maxe < minmaxe_)
			{
				minmaxe_ = maxe;

				pGITNodeSuboptimal = pGITNode;

				//printf("e=%f\n", minmaxe_);
			}

			if (nGITNodes > maxnGITNodes)
				break;
		}

		if (nGITNodes > maxnGITNodes)
			Descriptor(pGITNodeSuboptimal->ITPtr_, correspondenceArray, dS, bdS);

		maxe = Distance(pSurfels, iVertexArray, dS, bdS, iMaxErrVertex, SDF);

		if (maxe > maxMatchCost)
		{
			iCriticalVertexArray.Element[iCriticalVertexArray.n++] = iMaxErrVertex;

			if (minmaxe < 0.0f || maxe < minmaxe)
			{
				minmaxe = maxe;

				pGITNodeBest = pGITNode;
			}
		}
	} while (maxe > maxMatchCost && nGITNodes <= maxnGITNodes);

	delete[] SDF;
	delete[] iVertexArray.Element;
	delete[] iCriticalVertexArray.Element;
	for (iCluster = 0; iCluster < clusters.size(); iCluster++)
		delete[] Q[iCluster].queue;
	delete[] Q;
	delete[] GQueue;
	delete[] eCluster;
	delete[] pITPtrCluster;
}

void VN::ModelClusters()
{
	clusters.clear();

	VN_::Cluster cluster;

	VN_::Edge *pEdge = EdgeList.pFirst;
	
	int iCluster;

	while (pEdge)
	{
		if (pEdge->data.a < featureArray.n && pEdge->bPrimary)
		{
			for (iCluster = 0; iCluster < clusters.size(); iCluster++)
			{
				if (clusters.at(iCluster).iParent == pEdge->data.b)
					break;
			}

			if (iCluster < clusters.size())
				clusters.at(iCluster).iChild.push_back(pEdge->data.a);
			else
			{
				cluster.iParent = pEdge->data.b;
				cluster.iChild.clear();
				cluster.iChild.push_back(pEdge->data.a);
				clusters.push_back(cluster);
			}
		}

		pEdge = pEdge->pNext;
	}
}

void VN::GeneticAlg(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Array<Array<RECOG::VN_::SceneFeature>> correspondenceArray,
	float *dS,
	bool *bdS)
{
	int nGenerations = 1000;
	int nGeneration = 1000;
	int nSelection = 20;
	int nSamples = 100;
	int mutationRate = 50;

	Array<int> iPtArray;

	iPtArray.Element = new int[pMesh->NodeArray.n];

	iPtArray.n = 0;

	bool *bVisited = new bool[pMesh->NodeArray.n];

	memset(bVisited, 0, pMesh->NodeArray.n * sizeof(bool));

	int iSurfel;	
	Surfel *pSurfel;
	QLIST::Index2 *pPtIdx;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		pPtIdx = pSurfel->PtList.pFirst;

		while (pPtIdx)
		{
			if (!bVisited[pPtIdx->Idx])
			{
				iPtArray.Element[iPtArray.n++] = pPtIdx->Idx;

				bVisited[pPtIdx->Idx] = true;
			}

			pPtIdx = pPtIdx->pNext;
		}
	}

	Array<float *>PArray;
	
	PArray.Element = new float *[nSamples];
	PArray.n = nSamples;

	int diPt = iPtArray.n / nSamples;
	
	int iSample;

	for (iSample = 0; iSample < nSamples; iSample++)
		PArray.Element[iSample] = pMesh->NodeArray.Element[iPtArray.Element[iSample * diPt]].P;

	int nGenes = featureArray.n * nGeneration;

	int *solution = new int[nGenes];

	FILE *fp = fopen("..\\pseudorandom1000000.dat", "rb");

	int nRnd = 1000000;

	int *iRnd = new int[nRnd];

	fread(iRnd, sizeof(int), nRnd, fp);

	fclose(fp);

	int iiRnd = 0;

	Array<SortIndex<float>> E;

	E.Element = new SortIndex<float>[nGeneration];
	E.n = nGeneration;

	int maxCross = featureArray.n - 1;

	float *SDF = new float[featureArray.n];
	float *d = new float[featureArray.n];
	bool *bSelected = new bool[nGeneration];
	memset(bSelected, 0, nGeneration * sizeof(bool));

	int iSolution, iFeature, iCross, iCorrespondence;
	int *child, *parent1, *parent2;

	for (iSolution = 0; iSolution < nGeneration; iSolution++)
		for (iFeature = 0; iFeature < featureArray.n; iFeature++)
		{
			RVLRND(correspondenceArray.Element[iFeature].n, iRnd, nRnd, iiRnd, iCorrespondence);
			RVLMXEL(solution, featureArray.n, iSolution, iFeature) = iCorrespondence;
		}

	int iGeneration, iSelection, iParent1, iParent2, rnd100;

	for (iGeneration = 0; iGeneration < nGenerations; iGeneration++)
	{
		for (iSolution = 0; iSolution < nGeneration; iSolution++)
		{
			E.Element[iSolution].cost = Evaluate(PArray, correspondenceArray, solution + iSolution * featureArray.n, SDF, d);
			E.Element[iSolution].idx = iSolution;
		}

		BubbleSort<SortIndex<float>>(E);

		if (iGeneration == nGenerations - 1)
			break;

		//if (iGeneration % 10 == 9)
		//	printf(".");
		printf("%d\t%f\n", iGeneration, E.Element[0].cost);

		for (iSelection = 0; iSelection < nSelection; iSelection++)
			bSelected[E.Element[iSelection].idx] = true;

		for (iSolution = 0; iSolution < nGeneration; iSolution++)
		{
			if (bSelected[iSolution])
				continue;

			RVLRND(nSelection, iRnd, nRnd, iiRnd, iParent1);

			parent1 = solution + featureArray.n * E.Element[iParent1].idx;

			do RVLRND(nSelection, iRnd, nRnd, iiRnd, iParent2) while (iParent2 == iParent1);

			parent2 = solution + featureArray.n * E.Element[iParent2].idx;

			RVLRND(maxCross, iRnd, nRnd, iiRnd, iCross);

			child = solution + iSolution * featureArray.n;

			for (iFeature = 0; iFeature <= iCross; iFeature++)
				child[iFeature] = parent1[iFeature];

			for (iFeature = iCross + 1; iFeature < featureArray.n; iFeature++)
				child[iFeature] = parent2[iFeature];

			RVLRND(100, iRnd, nRnd, iiRnd, rnd100);

			if (rnd100 < mutationRate)
			{
				RVLRND(featureArray.n, iRnd, nRnd, iiRnd, iFeature);
				RVLRND(correspondenceArray.Element[iFeature].n, iRnd, nRnd, iiRnd, iCorrespondence);
				child[iFeature] = iCorrespondence;
			}
		}
	}

	int *finalSolution = solution + featureArray.n * E.Element[0].idx;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		dS[iFeature] = correspondenceArray.Element[iFeature].Element[finalSolution[iFeature]].d;
		bdS[iFeature] = true;
	}

	delete[] iPtArray.Element;
	delete[] bSelected;
	delete[] PArray.Element;
	delete[] solution;
	delete[] iRnd;
	delete[] E.Element;
	delete[] SDF;
	delete[] d;
	delete[] bVisited;
}

void VN::Fit(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	float *dS,
	bool *bdS)
{

}

float VN::Evaluate(
	Array<float *>PArray,
	Array<Array<RECOG::VN_::SceneFeature>> correspondenceArray,
	int *solution,
	float *SDF,
	float *d,
	bool *bd,
	float maxe)
{
	int iFeature;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
		d[iFeature] = correspondenceArray.Element[iFeature].Element[solution[iFeature]].d;

	float e = 0.0f;

	int iSample;
	int iActiveFeature;
	float e_;

	for (iSample = 0; iSample < PArray.n; iSample++)
	{
		e_ = Evaluate(PArray.Element[iSample], SDF, iActiveFeature, true, d);

		// sum of absolute distances

		//if (e_ < 0.0f)
		//	e_ = -e_;
		//e += e_;

		// maximum absolute distance

		if (e_ < 0.0f)
			e_ = -e_;
		if (e_ > e)
			e = e_;
	}

	return e;
}

float VN::Evaluate(
	Mesh *pMesh,
	Array<int> iPtArray,
	float *SDF,
	float *d,
	bool *bd,
	float maxe)
{
	float e = 0.0f;

	int iSample;
	int iActiveFeature;
	float e_;
	float *P;

	for (iSample = 0; iSample < iPtArray.n; iSample++)
	{
		P = pMesh->NodeArray.Element[iPtArray.Element[iSample]].P;

		e_ = Evaluate(P, SDF, iActiveFeature, true, d, bd);

		// sum of absolute distances

		//if (e_ < 0.0f)
		//	e_ = -e_;
		//e += e_;

		// maximum absolute distance

		//if (e_ < 0.0f)
		//	e_ = -e_;
		//if (e_ > e)
		//	e = e_;

		// sum of saturated absolute distances

		if (e_ < 0.0f)
			e_ = -e_;
		if (e_ > maxe)
			e_ = maxe;
		e += (e_ / maxe);
	}

	return e;
}

float VN::Evaluate(
	float *PArray,
	int nP,
	float *SDF,
	float *d,
	bool *bd,
	float maxe)
{
	float *P = PArray;

	float e = 0.0f;

	int iSample;
	int iActiveFeature;
	float e_;

	for (iSample = 0; iSample < nP; iSample++, P += 3)
	{
		e_ = Evaluate(P, SDF, iActiveFeature, true, d, bd);

		// sum of absolute distances

		//if (e_ < 0.0f)
		//	e_ = -e_;
		//e += e_;

		// maximum absolute distance

		//if (e_ < 0.0f)
		//	e_ = -e_;
		//if (e_ > e)
		//	e = e_;

		// sum of saturated absolute distances

		if (e_ < 0.0f)
			e_ = -e_;
		if (e_ > maxe)
			e_ = maxe;
		e += (e_ / maxe);
	}

	return e;
}

float VN::Evaluate(
	Array<RECOG::VN_::Sample> sampleArray,
	float *SDF,
	float *d,
	bool *bd,
	float maxe)
{
	float e = 0.0f;

	int iSample;
	int iActiveFeature;
	float e_;
	RECOG::VN_::Sample *pSample;

	for (iSample = 0; iSample < sampleArray.n; iSample++)
	{
		pSample = sampleArray.Element + iSample;

		e_ = Evaluate(pSample->P, SDF, iActiveFeature, true, d, bd) - pSample->SDF;

		// sum of absolute distances

		//if (e_ < 0.0f)
		//	e_ = -e_;
		//e += e_;

		// maximum absolute distance

		//if (e_ < 0.0f)
		//	e_ = -e_;
		//if (e_ > e)
		//	e = e_;

		// sum of saturated absolute distances

		if (e_ < 0.0f)
			e_ = -e_;
		if (e_ > maxe)
			e_ = maxe;
		e += (e_ / maxe);
	}

	return e;
}

float VN::GetMeshSize(Box<float> boundingBox)
{
	float a = boundingBox.maxx - boundingBox.minx;
	float b = boundingBox.maxy - boundingBox.miny;
	float c = boundingBox.maxz - boundingBox.minz;

	float size = RVLMAX(a, b);
	if (c > size)
		size = c;

	return size;
}

void VN::Match2(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Box<float> boundingBox,
	RECOG::VN_::Parameters params,
	float *dS,
	bool *bdS)
{
	int minFeatureSize = 100;
	float kMaxFeatureDist = 0.1f;
	float mincsN = cos(30.0f * DEG2RAD);

	float size = GetMeshSize(boundingBox);

	float maxFeatureDist = kMaxFeatureDist * size;
	float maxFeatureDist2 = maxFeatureDist * maxFeatureDist;
	float maxminFeatureDist2 = 4.0 * maxFeatureDist2;
	float maxDeviation = params.kMaxMatchCost * size;

	CRVLMem mem;

	mem.Create(10000000);

	CRVLMem *pMem2 = &mem;

	Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> featureGraph;

	featureGraph.NodeArray.Element = new GRAPH::Node[pSurfels->NodeArray.n];
	
	featureGraph.NodeArray.n = 0;

	int nPts = 0;

	int iFeature;
	GRAPH::Node *pFNode;
	Surfel *pFeature;
	QList<GRAPH::EdgePtr<GRAPH::Edge>> *pFEdgeList_;

	for (iFeature = 0; iFeature < pSurfels->NodeArray.n; iFeature++)
	{
		pFeature = pSurfels->NodeArray.Element + iFeature;

		if (!pFeature->bEdge && pFeature->size >= minFeatureSize)
		{
			pFNode = featureGraph.NodeArray.Element + featureGraph.NodeArray.n;

			pFEdgeList_ = &(pFNode->EdgeList);

			RVLQLIST_INIT(pFEdgeList_);

			pFNode->idx = iFeature;

			featureGraph.NodeArray.n++;

			nPts += pFeature->size;
		}
	}

	QList<GRAPH::Edge> FEdgeList;

	QList<GRAPH::Edge> *pFEdgeList = &FEdgeList;

	RVLQLIST_INIT(pFEdgeList);

	QList<QLIST::Entry<float>> FEdgeCostList;

	QList<QLIST::Entry<float>> *pFEdgeCostList = &FEdgeCostList;
	
	RVLQLIST_INIT(pFEdgeCostList);

	int nFEdges = 0;

	int iFNode, iFNode_;
	GRAPH::Node *pFNode_;
	Surfel *pFeature_;
	Array<MeshEdgePtr *> boundary;
	MeshEdgePtr *piBndPt;
	int i;
	QLIST::Index *pVertexIdx;
	float *P, *P_;
	float dP[3];
	float dist, minDist;
	GRAPH::Edge *pFEdge;
	QLIST::Entry<float> *pFEdgeCost;

	for (iFNode = 0; iFNode < featureGraph.NodeArray.n; iFNode++)
	{
		pFNode = featureGraph.NodeArray.Element + iFNode;

		pFeature = pSurfels->NodeArray.Element + pFNode->idx;

		boundary = pFeature->BoundaryArray.Element[0];

		for (iFNode_ = iFNode + 1; iFNode_ < featureGraph.NodeArray.n; iFNode_++)
		{
			pFNode_ = featureGraph.NodeArray.Element + iFNode_;

			pFeature_ = pSurfels->NodeArray.Element + pFNode_->idx;

			minDist = maxminFeatureDist2;

			for (i = 0; i < boundary.n; i++)
			{
				piBndPt = boundary.Element[i];

				P = pMesh->NodeArray.Element[RVLPCSEGMENT_GRAPH_GET_NODE(piBndPt)].P;

				pVertexIdx = pSurfels->surfelVertexList.Element[pFNode_->idx].pFirst;

				while (pVertexIdx)
				{
					P_ = pSurfels->vertexArray.Element[pVertexIdx->Idx]->P;

					RVLDIF3VECTORS(P_, P, dP);

					dist = RVLDOTPRODUCT3(dP, dP);

					if (dist < minDist)
						minDist = dist;

					pVertexIdx = pVertexIdx->pNext;
				}
			}

			if (minDist <= maxFeatureDist2)
			{
				pFEdge = ConnectNodes<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>>(pFNode, pFNode_, iFNode, iFNode_, pMem2);

				RVLQLIST_ADD_ENTRY(pFEdgeList, pFEdge);

				RVLMEM_ALLOC_STRUCT(pMem2, QLIST::Entry<float>, pFEdgeCost);

				RVLQLIST_ADD_ENTRY(pFEdgeCostList, pFEdgeCost);

				pFEdgeCost->data = sqrt(minDist);

				nFEdges++;
			}
		}
	}

	Array<float> FEdgeCostarray;
	
	FEdgeCostarray.Element = new float[nFEdges];

	QLIST::CopyToArray<float>(pFEdgeCostList, &FEdgeCostarray);

	ModelClusters();

	int nClusters = clusters.size();

	float featureSize;

	QLIST::Entry<Pair<int, int>> *queueMem = new QLIST::Entry<Pair<int, int>>[featureGraph.NodeArray.n];

	QList<QLIST::Entry<Pair<int, int>>> queue;

	QList<QLIST::Entry<Pair<int, int>>> *pQueue = &queue;

	bool *bVisited = new bool[featureGraph.NodeArray.n];

	memset(bVisited, 0, featureGraph.NodeArray.n * sizeof(bool));

	Array<int> iVisitedFNodeArray;

	iVisitedFNodeArray.Element = new int[featureGraph.NodeArray.n];

	Array<int> iClusterVertexArray;

	iClusterVertexArray.Element = new int[pSurfels->vertexArray.n];

	bool *bVertexInCluster = new bool[pSurfels->vertexArray.n];

	memset(bVertexInCluster, 0, pSurfels->vertexArray.n * sizeof(bool));

	VN_::FeatureNodeData *FNodeData = new VN_::FeatureNodeData[nClusters * featureGraph.NodeArray.n];

	Array<int> iClusterFeatureArray;

	iClusterFeatureArray.Element = new int[featureGraph.NodeArray.n];

	Array<Array<VN_::Correspondence2>> correspondences;

	correspondences.n = nClusters;
	correspondences.Element = new Array<VN_::Correspondence2>[correspondences.n];
	
	memset(bdS, 0, featureArray.n * sizeof(bool));

	int *clusterMap = new int[featureGraph.NodeArray.n];

	QList<QLIST::Entry<VN_::Correspondence2>> correspList;

	QList<QLIST::Entry<VN_::Correspondence2>> *pCorrespList = &correspList;

	QLIST::Entry<Pair<int, int>> **ppPutCandidate;
	QLIST::Entry<Pair<int, int>> *pPutCandidate, *pNewCandidate;
	int iCluster, iSCluster;
	GRAPH::EdgePtr<GRAPH::Edge> *pFEdgePtr;
	float *N, *N__;
	VN_::Cluster cluster;
	VN_::FeatureNodeData *pFNodeData, *FNodeData_;
	int nClusterFeatures;
	int iNode;
	VN_::Feature *pMFeature;
	float cs, maxcsN;
	int iCorrespondingNode;
	float d, e, d__;
	int maxFeatureSize, iLargestFeature;
	int iFNode__;
	bool bUpdateFNode;
	int nRemainingPts;
	QLIST::Entry<VN_::Correspondence2> *pCorresp;
	VN_::Correspondence2 *pCorresp_;
	Array<VN_::Correspondence2> *pCorrespArray;
	float maxd;

	for (iCluster = 0; iCluster < correspondences.n; iCluster++)
	{
		cluster = clusters.at(iCluster);

		nClusterFeatures = cluster.iChild.size();

		FNodeData_ = FNodeData + featureGraph.NodeArray.n * iCluster;

		for (iFNode = 0; iFNode < featureGraph.NodeArray.n; iFNode++)
		{
			iFeature = featureGraph.NodeArray.Element[iFNode].idx;

			pFeature = pSurfels->NodeArray.Element + iFeature;

			N = pFeature->N;

			maxcsN = mincsN;

			iCorrespondingNode = -1;

			for (i = 0; i < nClusterFeatures; i++)
			{
				iNode = cluster.iChild.at(i);

				pMFeature = NodeArray.Element[iNode].pFeature;

				cs = RVLDOTPRODUCT3(N, pMFeature->N);

				if (cs > maxcsN)
				{
					maxcsN = cs;

					iCorrespondingNode = iNode;
				}
			}

			FNodeData_[iFNode].iNode = iCorrespondingNode;

			if (iCorrespondingNode >= 0)
			{
				N = NodeArray.Element[iCorrespondingNode].pFeature->N;

				pVertexIdx = pSurfels->surfelVertexList.Element[iFeature].pFirst;

				if (pVertexIdx)
				{
					P = pSurfels->vertexArray.Element[pVertexIdx->Idx]->P;

					maxd = RVLDOTPRODUCT3(N, P);

					pVertexIdx = pVertexIdx->pNext;

					while (pVertexIdx)
					{
						P = pSurfels->vertexArray.Element[pVertexIdx->Idx]->P;

						d = RVLDOTPRODUCT3(N, P);

						if (d > maxd)
							maxd = d;

						pVertexIdx = pVertexIdx->pNext;
					}

					FNodeData_[iFNode].d = maxd;
				}
				else
					FNodeData_[iFNode].d = RVLDOTPRODUCT3(N, pFeature->P);
			}
		}

		RVLQLIST_INIT(pCorrespList);

		memset(clusterMap, 0xff, featureGraph.NodeArray.n * sizeof(int));

		iSCluster = 0;

		nRemainingPts = nPts;

		while (100 * nRemainingPts / nPts > 5)
		{
			maxFeatureSize = 0;
			iLargestFeature = -1;

			for (iFNode = 0; iFNode < featureGraph.NodeArray.n; iFNode++)
			{
				if (FNodeData_[iFNode].iNode < 0)
					continue;

				if (clusterMap[iFNode] >= 0)
					continue;

				pFNode = featureGraph.NodeArray.Element + iFNode;

				featureSize = pSurfels->NodeArray.Element[pFNode->idx].size;

				if (featureSize > maxFeatureSize)
				{
					maxFeatureSize = featureSize;

					iLargestFeature = iFNode;
				}
			}

			if (iLargestFeature < 0)
				break;

			RVLQLIST_INIT(pQueue);

			pNewCandidate = queueMem;

			pNewCandidate->data.a = iLargestFeature;
			pNewCandidate->data.b = maxFeatureSize;

			RVLQLIST_ADD_ENTRY(pQueue, pNewCandidate);

			pNewCandidate++;

			bVisited[iLargestFeature] = true;

			iVisitedFNodeArray.n = 0;

			iVisitedFNodeArray.Element[iVisitedFNodeArray.n++] = iLargestFeature;

			iClusterVertexArray.n = iClusterFeatureArray.n = 0;

			while (pQueue->pFirst)
			{
				iFNode = pQueue->pFirst->data.a;

				if (iFNode == 15)
					int debug = 0;

				pQueue->pFirst = pQueue->pFirst->pNext;

				pFNodeData = FNodeData_ + iFNode;

				pMFeature = NodeArray.Element[pFNodeData->iNode].pFeature;

				N = pMFeature->N;
				d = pFNodeData->d;

				for (i = 0; i < iClusterVertexArray.n; i++)
				{
					P = pSurfels->vertexArray.Element[iClusterVertexArray.Element[i]]->P;

					e = RVLDOTPRODUCT3(N, P) - d;

					if (e > maxDeviation)
						break;
				}

				if (i >= iClusterVertexArray.n)
				{
					pVertexIdx = pSurfels->surfelVertexList.Element[featureGraph.NodeArray.Element[iFNode].idx].pFirst;

					while (pVertexIdx)
					{
						P = pSurfels->vertexArray.Element[pVertexIdx->Idx]->P;

						for (i = 0; i < iClusterFeatureArray.n; i++)
						{
							iFNode__ = iClusterFeatureArray.Element[i];

							pFNodeData = FNodeData_ + iFNode__;

							N__ = NodeArray.Element[pFNodeData->iNode].pFeature->N;
							d__ = pFNodeData->d;

							e = RVLDOTPRODUCT3(N__, P) - d__;

							if (e > maxDeviation)
								break;
						}

						if (i < iClusterFeatureArray.n)
							break;

						pVertexIdx = pVertexIdx->pNext;
					}

					if (pVertexIdx == NULL)
					{
						iClusterFeatureArray.Element[iClusterFeatureArray.n++] = iFNode;

						clusterMap[iFNode] = iSCluster;

						pVertexIdx = pSurfels->surfelVertexList.Element[iFNode].pFirst;

						while (pVertexIdx)
						{
							if (!bVertexInCluster[pVertexIdx->Idx])
							{
								iClusterVertexArray.Element[iClusterVertexArray.n++] = pVertexIdx->Idx;

								bVertexInCluster[pVertexIdx->Idx] = true;
							}

							pVertexIdx = pVertexIdx->pNext;
						}

						pFNode = featureGraph.NodeArray.Element + iFNode;

						pFEdgePtr = pFNode->EdgeList.pFirst;

						while (pFEdgePtr)	// For every neighbor of iFNode
						{
							iFNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pFEdgePtr);

							//if (iFNode_ == 15)
							//	int debug = 0;

							pFNodeData = FNodeData_ + iFNode_;

							if (pFNodeData->iNode >= 0 && clusterMap[iFNode_] < 0)
							{
								if (!bVisited[iFNode_])
								{
									bVisited[iFNode_] = true;

									iVisitedFNodeArray.Element[iVisitedFNodeArray.n++] = iFNode_;

									size = pSurfels->NodeArray.Element[featureGraph.NodeArray.Element[iFNode_].idx].size;

									ppPutCandidate = &(pQueue->pFirst);

									pPutCandidate = *ppPutCandidate;

									while (pPutCandidate)
									{
										if (size > pPutCandidate->data.b)
											break;

										ppPutCandidate = &(pPutCandidate->pNext);

										pPutCandidate = *ppPutCandidate;
									}

									RVLQLIST_INSERT_ENTRY2(ppPutCandidate, pNewCandidate);

									pNewCandidate->data.a = iFNode_;
									pNewCandidate->data.b = size;

									pNewCandidate++;
								}
							}

							pFEdgePtr = pFEdgePtr->pNext;
						}	// for every neighbor of iFNode
					}
				}
			}	// while queue is not empty

			for (i = 0; i < iVisitedFNodeArray.n; i++)
				bVisited[iVisitedFNodeArray.Element[i]] = false;

			for (i = 0; i < iClusterVertexArray.n; i++)
				bVertexInCluster[iClusterVertexArray.Element[i]] = false;

			RVLMEM_ALLOC_STRUCT(pMem2, QLIST::Entry<VN_::Correspondence2>, pCorresp);

			RVLQLIST_ADD_ENTRY(pCorrespList, pCorresp);

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, int, featureArray.n, pCorresp->data.iFNode);

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, int, iClusterFeatureArray.n, pCorresp->data.iFeatureArray.Element);

			pCorresp->data.iFeatureArray.n = iClusterFeatureArray.n;

			memset(pCorresp->data.iFNode, 0xff, featureArray.n * sizeof(int));

			pCorresp->data.cost = 0;

			for (i = 0; i < iClusterFeatureArray.n; i++)
			{
				iFNode = iClusterFeatureArray.Element[i];

				pCorresp->data.iFeatureArray.Element[i] = iFNode;

				pFNode = featureGraph.NodeArray.Element + iFNode;

				size = pSurfels->NodeArray.Element[pFNode->idx].size;

				pCorresp->data.cost += size;

				iNode = FNodeData_[iFNode].iNode;

				iFNode_ = pCorresp->data.iFNode[iNode];				

				if (iFNode_ >= 0)
					bUpdateFNode = (size > pSurfels->NodeArray.Element[featureGraph.NodeArray.Element[iFNode_].idx].size);
				else
					bUpdateFNode = true;

				if (bUpdateFNode)
					pCorresp->data.iFNode[iNode] = iFNode;
			}

			nRemainingPts -= pCorresp->data.cost;

			iSCluster++;
		}	// while (100 * nRemainingPts / nPts > 5)

		pCorrespArray = correspondences.Element + iCluster;

		pCorrespArray->n = iSCluster;
		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, VN_::Correspondence2, pCorrespArray->n, pCorrespArray->Element);

		QLIST::CopyToArray<VN_::Correspondence2>(pCorrespList, pCorrespArray);

		BubbleSort<VN_::Correspondence2>(correspondences.Element[iCluster], true);
	}	// for each cluster

	int maxGQueueSize = 1;

	for (iCluster = 0; iCluster < correspondences.n; iCluster++)
		maxGQueueSize += correspondences.Element[iCluster].Element[0].cost;

	QList<VN_::Correspondence3> **GQueue;

	GQueue = new QList<VN_::Correspondence3> *[maxGQueueSize];

	memset(GQueue, 0, maxGQueueSize * sizeof(QList<VN_::Correspondence3> *));

	int nmFnodes = (featureGraph.NodeArray.n >> 6);

	if ((nmFnodes << 6) < featureGraph.NodeArray.n)
		nmFnodes++;

	VN_::Correspondence3 *pCCorresp;

	RVLMEM_ALLOC_STRUCT(pMem2, VN_::Correspondence3, pCCorresp);

	pCCorresp->iMCluster = pCCorresp->iSCluster = -1;
	pCCorresp->pParent = NULL;
	pCCorresp->cost = 0;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, unsigned long long int, nmFnodes, pCCorresp->mFNodes);

	for (i = 0; i < nmFnodes; i++)
		pCCorresp->mFNodes[i] = 0x0000000000000000;

	int iTopQueueBin = 0;
	int iBottomQueueBin = 0;

	int iBin;
	QList<VN_::Correspondence3> *pGQueueBin;

	RVLVN_ADD_QUEUE_ENTRY(pCCorresp, VN_::Correspondence3, 0, GQueue, 1, maxGQueueSize, iTopQueueBin, iBottomQueueBin, pMem2, iBin, pGQueueBin);

	VN_::Correspondence3 initCorresp;

	initCorresp.pNext = pCCorresp;

	pCCorresp = &initCorresp;

	bool bFail = false;

	int j;
	VN_::Correspondence3 *pCCorresp_;
	unsigned long long int bitFNode;
	int iBitFNode, iWordFNode;
	int SClusterSize;

	while (true)
	{
		RVLVN_GET_NEXT_QUEUE_ENTRY(GQueue, pCCorresp, pCCorresp, iTopQueueBin, iBottomQueueBin, bFail, pGQueueBin);

		if (bFail)
			break;

		iCluster = pCCorresp->iMCluster + 1;

		if (iCluster >= correspondences.n)
			break;

		for (i = 0; i < correspondences.Element[iCluster].n; i++)
		{
			RVLMEM_ALLOC_STRUCT(pMem2, VN_::Correspondence3, pCCorresp_);

			pCCorresp_->iMCluster = iCluster;
			pCCorresp_->iSCluster = i;
			pCCorresp_->pParent = pCCorresp;

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, unsigned long long int, nmFnodes, pCCorresp_->mFNodes);

			for (j = 0; j < nmFnodes; j++)
				pCCorresp_->mFNodes[j] = pCCorresp->mFNodes[j];

			SClusterSize = 0;

			pCorresp_ = correspondences.Element[iCluster].Element + i;

			for (j = 0; j < pCorresp_->iFeatureArray.n; j++)
			{
				iFNode = pCorresp_->iFeatureArray.Element[j];

				iBitFNode = (iFNode & 0x0000003f);
				iWordFNode = (iFNode >> 6);
				bitFNode = (1ULL << iBitFNode);

				if ((pCCorresp_->mFNodes[iWordFNode] & bitFNode) == 0)
				{
					pCCorresp_->mFNodes[iWordFNode] |= bitFNode;

					SClusterSize += pSurfels->NodeArray.Element[featureGraph.NodeArray.Element[iFNode].idx].size;
				}
				else
					int debug = 0;
			}

			pCCorresp_->cost = pCCorresp->cost + correspondences.Element[iCluster].Element[0].cost - SClusterSize;

			RVLVN_ADD_QUEUE_ENTRY(pCCorresp_, VN_::Correspondence3, pCCorresp_->cost, GQueue, 1, maxGQueueSize, iTopQueueBin, iBottomQueueBin, pMem2, iBin, pGQueueBin);
		}
	}

	while (pCCorresp)
	{
		if (pCCorresp->iMCluster >= 0)
		{
			pCorresp_ = correspondences.Element[pCCorresp->iMCluster].Element + pCCorresp->iSCluster;

			cluster = clusters.at(pCCorresp->iMCluster);

			FNodeData_ = FNodeData + featureGraph.NodeArray.n * pCCorresp->iMCluster;

			for (i = 0; i < cluster.iChild.size(); i++)
			{
				iNode = cluster.iChild.at(i);

				iFNode = pCorresp_->iFNode[iNode];

				if (iFNode < 0)
					continue;

				dS[iNode] = FNodeData_[iFNode].d;
				bdS[iNode] = true;
			}
		}

		pCCorresp = pCCorresp->pParent;
	}

	delete[] featureGraph.NodeArray.Element;
	delete[] FEdgeCostarray.Element;
	delete[] queueMem;
	delete[] bVisited;
	delete[] iVisitedFNodeArray.Element;
	delete[] iClusterVertexArray.Element;
	delete[] bVertexInCluster;
	delete[] FNodeData;
	delete[] iClusterFeatureArray.Element;
	delete[] clusterMap;
	delete[] correspondences.Element;
	delete[] GQueue;
}

void VN::Match3(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Array<RECOG::PSGM_::Cluster *> SClusters,
	Box<float> boundingBox,
	RECOG::VN_::Parameters params,
	float *dSOut,
	bool *bdSOut)
{
	float size = GetMeshSize(boundingBox);

	float maxDeviation = params.kMaxMatchCost * size;

	CRVLMem mem;

	mem.Create(10000000);

	CRVLMem *pMem2 = &mem;

	Array<int> iPtArray;

	iPtArray.n = pMesh->NodeArray.n;

	RandomIndices(iPtArray);

	iPtArray.n = 300;

	int i;
	float *P;

	//FILE *fp = fopen("sampledMesh.txt", "w");

	//for (i = 0; i < iPtArray.n; i++)
	//{
	//	P = pMesh->NodeArray.Element[iPtArray.Element[i]].P;

	//	fprintf(fp, "%f\t%f\t%f\n", P[0], P[1], P[2]);
	//}

	//fclose(fp);

	ModelClusters();

	int nMClusters = clusters.size();

	float *dS = new float[featureArray.n * SClusters.n];

	Array<Array<float *>> descriptors;

	descriptors.Element = new Array<float *>[nMClusters];
	descriptors.n = nMClusters;

	float *dS_ = dS;

	float *D = new float[featureArray.n];
	bool *bD = new bool[featureArray.n];

	int j;
	int iMCluster, iSCluster, nMClusterFeatures;
	PSGM_::Cluster *pSCluster;
	int iFeature;
	float *N;
	VN_::Cluster MCluster;
	float d, maxd;

	for (iMCluster = 0; iMCluster < nMClusters; iMCluster++)
	{
		MCluster = clusters.at(iMCluster);

		nMClusterFeatures = MCluster.iChild.size();

		descriptors.Element[iMCluster].Element = new float *[SClusters.n];
		descriptors.Element[iMCluster].n = SClusters.n;

		for (iSCluster = 0; iSCluster < SClusters.n; iSCluster++)
		{
			descriptors.Element[iMCluster].Element[iSCluster] = dS_;

			pSCluster = SClusters.Element[iSCluster];

			for (i = 0; i < nMClusterFeatures; i++)
			{
				iFeature = MCluster.iChild.at(i);

				N = NodeArray.Element[iFeature].pFeature->N;

				P = pSurfels->vertexArray.Element[pSCluster->iVertexArray.Element[0]]->P;

				maxd = RVLDOTPRODUCT3(N, P);

				for (j = 1; j < pSCluster->iVertexArray.n; j++)
				{
					P = pSurfels->vertexArray.Element[pSCluster->iVertexArray.Element[j]]->P;

					d = RVLDOTPRODUCT3(N, P);

					if (d > maxd)
						maxd = d;
				}

				dS_[i] = maxd;
			}

			dS_ += nMClusterFeatures;
		}
	}

	int maxQueueSize = 100 * iPtArray.n + 1;

	QList<VN_::Correspondence4> **queue;

	queue = new QList<VN_::Correspondence4> *[maxQueueSize];

	memset(queue, 0, maxQueueSize * sizeof(QList<VN_::Correspondence4> *));

	VN_::Correspondence4 *pCorresp;

	RVLMEM_ALLOC_STRUCT(pMem2, VN_::Correspondence4, pCorresp);

	pCorresp->iMCluster = pCorresp->iSCluster = -1;
	pCorresp->iLevel = -1;
	pCorresp->pParent = NULL;
	pCorresp->cost = 0;

	int iTopQueueBin = 0;
	int iBottomQueueBin = 0;

	int iBin;
	QList<VN_::Correspondence4> *pQueueBin;

	RVLVN_ADD_QUEUE_ENTRY(pCorresp, VN_::Correspondence4, 0, queue, 1, maxQueueSize, iTopQueueBin, iBottomQueueBin, pMem2, iBin, pQueueBin);

	VN_::Correspondence4 initCorresp;

	initCorresp.pNext = pCorresp;

	pCorresp = &initCorresp;

	bool bFail = false;

	int maxLevel = 2;

	float *SDF = new float[featureArray.n];

	VN_::Correspondence4 *pCorresp_, *pCorresp__;
	float e;

	while (true)
	{
		RVLVN_GET_NEXT_QUEUE_ENTRY(queue, pCorresp, pCorresp, iTopQueueBin, iBottomQueueBin, bFail, pQueueBin);

		if (bFail)
			break;

		if (pCorresp->iLevel < maxLevel)
		{
			if (pCorresp->iLevel == 0)
			{
				RVLMEM_ALLOC_STRUCT(pMem2, VN_::Correspondence4, pCorresp_);

				pCorresp_->iLevel = pCorresp->iLevel + 1;
				pCorresp_->iMCluster = pCorresp->iMCluster;
				pCorresp_->iSCluster = pCorresp->iSCluster;
				pCorresp_->pParent = pCorresp->pParent;
				pCorresp_->cost = 0;

				RVLVN_ADD_QUEUE_ENTRY(pCorresp_, VN_::Correspondence4, pCorresp_->cost, queue, 1, maxQueueSize, iTopQueueBin, iBottomQueueBin, pMem2, iBin, pQueueBin);
			}

			for (iSCluster = 0; iSCluster < SClusters.n; iSCluster++)
			{
				pCorresp__ = pCorresp;

				while (pCorresp__)
				{
					if (pCorresp__->iSCluster == iSCluster)
						break;

					pCorresp__ = pCorresp__->pParent;
				}

				if (pCorresp__)
					continue;

				RVLMEM_ALLOC_STRUCT(pMem2, VN_::Correspondence4, pCorresp_);

				pCorresp_->iLevel = pCorresp->iLevel + 1;
				pCorresp_->iMCluster = pCorresp->iMCluster + (pCorresp_->iLevel == 1 ? 0 : 1);
				pCorresp_->iSCluster = iSCluster;
				pCorresp_->pParent = pCorresp;

				if (pCorresp_->iMCluster == nMClusters - 1)
				{
					memset(bD, 0, featureArray.n * sizeof(bool));

					pCorresp__ = pCorresp_;

					while (pCorresp__)
					{
						if (pCorresp__->iMCluster >= 0)
						{
							MCluster = clusters.at(pCorresp__->iMCluster);

							nMClusterFeatures = MCluster.iChild.size();

							for (i = 0; i < nMClusterFeatures; i++)
							{
								iFeature = MCluster.iChild.at(i);

								d = descriptors.Element[pCorresp__->iMCluster].Element[pCorresp__->iSCluster][i];

								if (bD[iFeature])
								{
									if (d > D[iFeature])
										D[iFeature] = d;
								}
								else
								{
									bD[iFeature] = true;
									D[iFeature] = d;
								}
							}
						}

						pCorresp__ = pCorresp__->pParent;
					}

					e = Evaluate(pMesh, iPtArray, SDF, D, bD, maxDeviation);

					pCorresp_->cost = (int)(100.0f * e);
				}
				else
					pCorresp_->cost = 0;

				RVLVN_ADD_QUEUE_ENTRY(pCorresp_, VN_::Correspondence4, pCorresp_->cost, queue, 1, maxQueueSize, iTopQueueBin, iBottomQueueBin, pMem2, iBin, pQueueBin);
			}	// for (iSCluster = 0; iSCluster < SClusters.n; iSCluster++)
		}	// if (pCorresp->iLevel < maxLevel)
	}

	for (i = 1; i < iBottomQueueBin; i++)
		if (queue[i])
			break;

	pCorresp = queue[i]->pFirst;

	memset(bdSOut, 0, featureArray.n * sizeof(bool));

	pCorresp__ = pCorresp;

	while (pCorresp__)
	{
		if (pCorresp__->iMCluster >= 0)
		{
			MCluster = clusters.at(pCorresp__->iMCluster);

			nMClusterFeatures = MCluster.iChild.size();

			for (i = 0; i < nMClusterFeatures; i++)
			{
				iFeature = MCluster.iChild.at(i);

				d = descriptors.Element[pCorresp__->iMCluster].Element[pCorresp__->iSCluster][i];

				if (bdSOut[iFeature])
				{
					if (d > dSOut[iFeature])
						dSOut[iFeature] = d;
				}
				else
				{
					bdSOut[iFeature] = true;
					dSOut[iFeature] = d;
				}
			}
		}

		pCorresp__ = pCorresp__->pParent;
	}

	delete[] iPtArray.Element;
	for (iMCluster = 0; iMCluster < nMClusters; iMCluster++)
		delete[] descriptors.Element[iMCluster].Element;
	delete[] descriptors.Element;
	delete[] D;
	delete[] bD;
	delete[] SDF;
}

void VN::Match4(
	Mesh *pMesh,
	RECOG::VN_::SceneObject sceneObject,
	void *vpClassifier,	
	Box<float> boundingBox,
	float *dS,
	bool *bdS)
	//Mesh *pMesh,
	//SurfelGraph *pSurfels,
	//Array<RECOG::PSGM_::Cluster *> SCClusters,
	//Array<RECOG::PSGM_::Cluster *> SUClusters,
	//Box<float> boundingBox,
	//RECOG::VN_::Parameters params,
	//CRVLMem *pMem,
	//float *dS,
	//bool *bdS)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	SurfelGraph *pSurfels = pClassifier->pSurfels;

	float *PArray = sceneObject.vertexArray;
	float *NArray = sceneObject.NArray;
	float *R = sceneObject.R;
	float *t = sceneObject.t;

	bool bTorus = false;
	bool bConcavity = false;

	int nMClusters = 0;

	RECOG::VN_::ModelCluster *pMCluster = modelClusterList.pFirst;

	while (pMCluster)
	{
		if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
			bConcavity = true;
		else if (pMCluster->type == RVLVN_CLUSTER_TYPE_XTORUS)
			bTorus = true;

		nMClusters++;

		pMCluster = pMCluster->pNext;
	}

	if (nMClusters == 0)
		return;

	float size = GetMeshSize(boundingBox);

	float maxDeviation = pClassifier->kMaxMatchCost * size;

	float *P;

	//// Sample the mesh surface and transform the sampled points using R and t.

	//int nSamplePts = 300;

	//Array<int> iPtArray;

	//iPtArray.n = pMesh->NodeArray.n;

	//RandomIndices(iPtArray);

	//float *PSampleArray = new float[3 * nSamplePts];

	//P = PSampleArray;

	//int i;
	//float *P_;

	//for (i = 0; i < nSamplePts; i++, PSampleArray += 3)
	//{
	//	P_ = pMesh->NodeArray.Element[iPtArray.Element[i]].P;

	//	RVLTRANSF3(P_, R, t, P);
	//}

	// Detect toroidal clusters.

	Array<RECOG::VN_::Torus *> STClusters;

	STClusters.Element = NULL;
	STClusters.n = 0;

	if (bTorus)
	{
		float axis[] = { 0.0f, 0.0f, 1.0f };

		pMCluster = modelClusterList.pFirst;

		while (pMCluster)
		{
			if (pMCluster->type == RVLVN_CLUSTER_TYPE_XTORUS)
				break;

			pMCluster = pMCluster->pNext;
		}

		ToroidalClusters(pMesh, PArray, NArray, pSurfels, axis, pMCluster->alphaArray, pMCluster->betaArray, 
			pClassifier->clusteringTolerance, STClusters, pClassifier->pMem);
	}
	
	// Join all clusters into array SClusters_.

	Array<RECOG::PSGM_::Cluster *> SCClusters = pClassifier->convexClustering.clusters;
	Array<RECOG::PSGM_::Cluster *> SUClusters = pClassifier->concaveClustering.clusters;

	Array<RECOG::VN_::SceneCluster> SClusters_;

	int nSCClusters = RVLMIN(SCClusters.n, pClassifier->maxnSCClusters);

	int nSUClusters = (bConcavity ? RVLMIN(SUClusters.n, pClassifier->maxnSUClusters) : 0);

	int nSTClusters = (bTorus ? RVLMIN(STClusters.n, pClassifier->maxnSTClusters) : 0);

	SClusters_.n = nSCClusters + nSUClusters + nSTClusters;

	SClusters_.Element = new RECOG::VN_::SceneCluster[SClusters_.n];

	RECOG::VN_::SceneCluster *pSCluster = SClusters_.Element;

	bool *bCovered = new bool[pSurfels->NodeArray.n];

	memset(bCovered, 0, pSurfels->NodeArray.n * sizeof(bool));

	bool *bCopied[2];

	bCopied[0] = new bool[SCClusters.n];

	memset(bCopied[0], 0, SCClusters.n * sizeof(bool));

	if (SUClusters.n > 0)
	{
		bCopied[1] = new bool[SUClusters.n];

		memset(bCopied[1], 0, SUClusters.n * sizeof(bool));
	}
	else
		bCopied[1] = NULL;

	int iSCluster__[2];

	iSCluster__[0] = iSCluster__[1] = 0;

	Array<RECOG::PSGM_::Cluster *> SClusterArray[2];

	SClusterArray[0] = SCClusters;
	SClusterArray[1] = SUClusters;

	int iLargestCluster[2];

	int iFirstArray = 0;
	int iLastArray = (nSUClusters ? 1 : 0);

	int *clusterMap[2];

	clusterMap[0] = pClassifier->convexClustering.clusterMap;
	clusterMap[1] = pClassifier->concaveClustering.clusterMap;

	int nSClusters[2];

	nSClusters[0] = nSCClusters;
	nSClusters[1] = nSUClusters;

	RECOG::PSGM_::Cluster *pCopiedCluster;
	int i, j, iFirstClusterToUpdate;
	int iSurfel, iCluster;
	int iClusterArray;
	int largestClusterOrig;

	while (iSCluster__[0] < nSCClusters || iSCluster__[1] < nSUClusters)
	{
		for (i = iFirstArray; i <= iLastArray; i++)
		{
			for (iCluster = 0; iCluster < SClusterArray[i].n; iCluster++)
				if (!bCopied[i][iCluster])
					break;

			iLargestCluster[i] = iCluster;
			
			largestClusterOrig = SClusterArray[i].Element[iCluster]->orig;

			iCluster++;

			for (; iCluster < SClusterArray[i].n; iCluster++)
				if (!bCopied[i][iCluster])
					if (SClusterArray[i].Element[iCluster]->orig > largestClusterOrig)
					{
						iLargestCluster[i] = iCluster; 
						
						largestClusterOrig = SClusterArray[i].Element[iCluster]->orig;
					}
		}

		if (iFirstArray < iLastArray)
			iClusterArray = (SClusterArray[0].Element[iLargestCluster[0]]->orig >= SClusterArray[1].Element[iLargestCluster[1]]->orig ? 0 : 1);
		else
			iClusterArray = iFirstArray;

		pCopiedCluster = SClusterArray[iClusterArray].Element[iLargestCluster[iClusterArray]];

		pSCluster = SClusters_.Element + iClusterArray * nSCClusters + iSCluster__[iClusterArray];

		pSCluster->type = (iClusterArray == 0 ? RVLVN_CLUSTER_TYPE_CONVEX : RVLVN_CLUSTER_TYPE_CONCAVE);
		pSCluster->vpCluster = pCopiedCluster;

		for (i = 0; i < pCopiedCluster->iSurfelArray.n; i++)
		{
			iSurfel = pCopiedCluster->iSurfelArray.Element[i];
			
			if (!bCovered[iSurfel])
			{
				for (j = iFirstArray; j <= iLastArray; j++)
				{
					iCluster = clusterMap[j][iSurfel];

					if (!bCopied[j][iCluster])
						SClusterArray[j].Element[iCluster]->orig -= pSurfels->NodeArray.Element[iSurfel].size;
				}

				bCovered[iSurfel] = true;
			}
		}

		bCopied[iClusterArray][iLargestCluster[iClusterArray]] = true;
				
		iSCluster__[iClusterArray]++;

		if (iSCluster__[iClusterArray] >= nSClusters[iClusterArray])
			iFirstArray = iLastArray = 1 - iClusterArray;
	}

	delete[] bCovered;
	delete[] bCopied[0];
	RVL_DELETE_ARRAY(bCopied[1]);

	pSCluster = SClusters_.Element + nSCClusters + nSUClusters;

	//for (iCluster = 0; iCluster < nSCClusters; iCluster++, pSCluster++)
	//{
	//	pSCluster->type = RVLVN_CLUSTER_TYPE_CONVEX;
	//	pSCluster->vpCluster = SCClusters.Element + iCluster;
	//}

	//for (iCluster = 0; iCluster < nSUClusters; iCluster++, pSCluster++)
	//{
	//	pSCluster->type = RVLVN_CLUSTER_TYPE_CONCAVE;
	//	pSCluster->vpCluster = SUClusters.Element + iCluster;
	//}

	if (bTorus)
	{
		for (iCluster = 0; iCluster < nSTClusters; iCluster++, pSCluster++)
		{
			pSCluster->type = RVLVN_CLUSTER_TYPE_XTORUS;
			pSCluster->vpCluster = STClusters.Element[iCluster];
		}
	}

	// Compute descriptor valuses for all correspondences (scene cluster, model cluster).

	float *dS_ = new float[SClusters_.n * featureArray.n];

	bool *bdS_ = new bool[SClusters_.n * featureArray.n];

	memset(bdS_, 0, SClusters_.n * featureArray.n * sizeof(bool));

	Array<float *> descriptors;

	descriptors.Element = new float *[SClusters_.n];
	descriptors.n = SClusters_.n;

	int iSCluster, iMCluster, iFeature, iVertex;
	float *N;
	PSGM_::Cluster *pSCCluster, *pSUCluster;
	VN_::Torus *pTCluster;
	float d, maxd, mind;
	float *dS__;
	bool *bdS__;
	VN_::Feature *pFeature;
	VN_::TorusRing *pTorusRing;

	for (iSCluster = 0; iSCluster < SClusters_.n; iSCluster++)
	{
		pSCluster = SClusters_.Element + iSCluster;

		dS__ = dS_ + iSCluster * featureArray.n;

		bdS__ = bdS_ + iSCluster * featureArray.n;

		descriptors.Element[iSCluster] = dS__;

		pMCluster = modelClusterList.pFirst;

		while (pMCluster)
		{
			if (pMCluster->type == pSCluster->type)
			{
				switch (pSCluster->type){
				case RVLVN_CLUSTER_TYPE_CONVEX:
					pSCCluster = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

					for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
					{
						N = featureArray.Element[iFeature].N;

						P = PArray + 3 * pSCCluster->iVertexArray.Element[0];

						maxd = RVLDOTPRODUCT3(N, P);

						for (j = 1; j < pSCCluster->iVertexArray.n; j++)
						{
							P = PArray + 3 * pSCCluster->iVertexArray.Element[j];

							d = RVLDOTPRODUCT3(N, P);

							if (d > maxd)
								maxd = d;
						}

						dS__[iFeature] = maxd;
						bdS__[iFeature] = true;
					}

					break;
				case RVLVN_CLUSTER_TYPE_CONCAVE:
					pSUCluster = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

					for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
					{
						N = featureArray.Element[iFeature].N;

						P = PArray + 3 * pSUCluster->iVertexArray.Element[0];

						mind = RVLDOTPRODUCT3(N, P);

						for (j = 1; j < pSUCluster->iVertexArray.n; j++)
						{
							P = PArray + 3 * pSUCluster->iVertexArray.Element[j];

							d = RVLDOTPRODUCT3(N, P);

							if (d < mind)
								mind = d;
						}

						dS__[iFeature] = mind;
						bdS__[iFeature] = true;
					}

					break;
				case RVLVN_CLUSTER_TYPE_XTORUS:
					pTCluster = (RECOG::VN_::Torus *)(pSCluster->vpCluster);

					for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
					{
						pFeature = featureArray.Element + iFeature;

						pTorusRing = pTCluster->ringArray.Element[pFeature->iBeta];

						if (pTorusRing)
						{
							dS__[iFeature] = pTorusRing->d[pFeature->iAlpha];
							bdS__[iFeature] = true;
						}
					}
				}
			}

			pMCluster = pMCluster->pNext;
		}
	}

	// Try all combinations of correspondences (scene cluster, model cluster) and find the best one.

	CRVLMem mem;

	mem.Create(10000000);

	CRVLMem *pMem2 = &mem;

	VN_::Correspondence4 *pCorresp;

	RVLMEM_ALLOC_STRUCT(pMem2, VN_::Correspondence4, pCorresp);

	pCorresp->iMCluster = pCorresp->iSCluster = -1;
	pCorresp->pParent = NULL;

	QList<VN_::Correspondence4> queue;

	QList<VN_::Correspondence4> *pQueue = &queue;

	RVLQLIST_INIT(pQueue);

	RVLQLIST_ADD_ENTRY(pQueue, pCorresp);

	float *dSEval = new float[featureArray.n];
	bool *bdSEval = new bool[featureArray.n];
	float *SDF = new float[featureArray.n];

	float mine = -1.0f;

	VN_::Correspondence4 *pCorresp_;
	int iSCluster_;
	RECOG::VN_::SceneCluster *pSCluster_;
	float e;
	float operation;

	while (pCorresp)
	{
		iSCluster_ = pCorresp->iSCluster + 1;

		if (iSCluster_ == SClusters_.n)
		{
			memset(bdSEval, 0, featureArray.n * sizeof(bool));

			pCorresp_ = pCorresp;

			while (pCorresp_)
			{
				pMCluster = modelClusterList.pFirst;

				while (pMCluster)
				{
					if (pMCluster->ID == pCorresp_->iMCluster)
						break;

					pMCluster = pMCluster->pNext;
				}

				if (pMCluster)
				{
					dS__ = descriptors.Element[pCorresp_->iSCluster];

					bdS__ = bdS_ + pCorresp_->iSCluster * featureArray.n;

					operation = (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX ? 1.0f : -1.0f);

					for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
						if (bdS__[iFeature])
						{
							if (bdSEval[iFeature])
							{
								if (operation * dS__[iFeature] > operation * dSEval[iFeature])
									dSEval[iFeature] = dS__[iFeature];
							}
							else
							{
								dSEval[iFeature] = dS__[iFeature];
								bdSEval[iFeature] = true;
							}
						}
				}

				pCorresp_ = pCorresp_->pParent;
			}

			//if (pCorresp->iMCluster == 1 && pCorresp->pParent->iMCluster == -1 && pCorresp->pParent->pParent->iMCluster == 0)
			//	int debug = 0;

			pMCluster = modelClusterList.pFirst;

			while (pMCluster)
			{
				for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
					if (bdSEval[iFeature])
						break;

				if (iFeature > pMCluster->iFeatureInterval.b)
					break;

				pMCluster = pMCluster->pNext;
			}

			if (pMCluster == NULL)
			{
				e = Evaluate(sceneObject.sampleArray, SDF, dSEval, bdSEval, maxDeviation);

				if (mine < 0.0f || e < mine)
				{
					mine = e;

					memcpy(dS, dSEval, featureArray.n * sizeof(float));
					memcpy(bdS, bdSEval, featureArray.n * sizeof(bool));
				}
			}
		}
		else
		{
			pSCluster_ = SClusters_.Element + iSCluster_;

			pMCluster = modelClusterList.pFirst;

			while (pMCluster)
			{
				if (pMCluster->type == pSCluster_->type)
				{
					RVLMEM_ALLOC_STRUCT(pMem2, VN_::Correspondence4, pCorresp_);

					RVLQLIST_ADD_ENTRY(pQueue, pCorresp_);

					pCorresp_->iSCluster = iSCluster_;
					pCorresp_->iMCluster = pMCluster->ID;
					pCorresp_->pParent = pCorresp;
				}

				pMCluster = pMCluster->pNext;
			}

			RVLMEM_ALLOC_STRUCT(pMem2, VN_::Correspondence4, pCorresp_);

			RVLQLIST_ADD_ENTRY(pQueue, pCorresp_);

			pCorresp_->iSCluster = iSCluster_;
			pCorresp_->iMCluster = -1;
			pCorresp_->pParent = pCorresp;
		}

		pCorresp = pCorresp->pNext;
	}

	// Deallocate memory.

	RVL_DELETE_ARRAY(STClusters.Element);
	delete[] SClusters_.Element;
	delete[] dS_;
	delete[] bdS_;
	delete[] dSEval;
	delete[] bdSEval;
	delete[] descriptors.Element;
	delete[] SDF;
}

void VN::ToroidalClusters(
	Mesh *pMesh,
	float *PArray,
	float *NArray,
	SurfelGraph *pSurfels,
	float *axis,
	Array<float> alphaArray,
	Array<float> betaArray,
	float maxErr,
	Array<RECOG::VN_::Torus *> &SClusters,
	CRVLMem *pMem)
{
	CRVLMem mem;

	mem.Create(10000000);

	CRVLMem *pMem2 = &mem;

	float *cb = new float[betaArray.n];
	float *sb = new float[betaArray.n];
	float *cb2 = new float[betaArray.n];

	int iBeta;

	for (iBeta = 0; iBeta < betaArray.n; iBeta++)
	{
		cb[iBeta] = cos(betaArray.Element[iBeta]);
		sb[iBeta] = sin(betaArray.Element[iBeta]);
		cb2[iBeta] = cb[iBeta] * cb[iBeta];
	}	

	float sign[] = {1.0f, -1.0f};

	VN_::EdgeTangent **tangentArray;

	tangentArray = new VN_::EdgeTangent *[betaArray.n];

	VN_::EdgeTangentSet *tangentSet = new VN_::EdgeTangentSet[pSurfels->vertexEdgeArray.n];

	int i, j;
	int iVertex, iVertex_;
	SURFEL::Vertex *pVertex;
	SURFEL::VertexEdge *pVEdge, *pVEdge_;
	GRAPH::EdgePtr2<SURFEL::VertexEdge> *pVEdgePtr;
	float dN[3];
	float *N1, *N2;
	float an1, adn, n1dn, dndn, a, b, c, k, g;
	float s[2];
	bool bRing;
	bool b90deg;
	VN_::EdgeTangentSet *pTangentSet;
	VN_::EdgeTangent *pTangent, *pTangent_;
	int nTangents;
	float fTmp;
	float *P, *P_;
	float dP[3];
	float d_, s_;

	for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
	{
		pVertex = pSurfels->vertexArray.Element[iVertex];

		P = PArray + 3 * iVertex;

		//if (iVertex == 110)
		//	int debug_ = 0;

		//int debug = 0;

		//int debug__;

		pVEdgePtr = pVertex->EdgeList.pFirst;

		while (pVEdgePtr)
		{
			iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pVEdgePtr);

			if (iVertex_ > iVertex)
			{
				pVEdge = pVEdgePtr->pEdge;

				//if (pVEdge->idx == 75)
				//	int debug = 0;

				P_ = PArray + 3 * iVertex_;

				pTangentSet = tangentSet + pVEdge->idx;

				RVLDIF3VECTORS(P, P_, dP);

				pTangentSet->edgeLength = sqrt(RVLDOTPRODUCT3(dP, dP));

				pTangentSet->miniBeta = pTangentSet->maxiBeta = -1;

				N1 = NArray + 3 * pVEdge->iSurfel[0];
				N2 = NArray + 3 * pVEdge->iSurfel[1];

				RVLDIF3VECTORS(N2, N1, dN);

				an1 = RVLDOTPRODUCT3(axis, N1);
				adn = RVLDOTPRODUCT3(axis, dN);
				dndn = RVLDOTPRODUCT3(dN, dN);
				n1dn = RVLDOTPRODUCT3(N1, dN);

				for (iBeta = 0; iBeta < betaArray.n; iBeta++)
				{
					bRing = false;

					nTangents = 0;

					b90deg = (cb2[iBeta] < 1e-6);

					if (b90deg)
					{
						s[0] = -an1 / adn;

						bRing = (s[0] >= 0.0f && s[0] <= 1.0f);

						if (bRing)
							nTangents = 1;
					}
					else
					{
						a = adn*adn - cb2[iBeta] * dndn;
						b = 2.0f * (adn*an1 - cb2[iBeta] * n1dn);
						c = an1*an1 - cb2[iBeta];

						k = b * b - 4 * a * c;

						if (k >= 0.0f)
						{
							k = sqrt(k);
							g = 2 * a;

							for (i = 0; i < 2; i++)
							{
								s_ = (-b + sign[i] * k) / g;

								if (s_ >= 0.0f && s_ <= 1.0f)
								{
									if ((an1 + s_ * adn) * cb[iBeta] > 0.0f)
									{
										bRing = true;

										s[nTangents] = s_;

										nTangents++;
									}
								}
							}
						}
					}

					if (bRing)
					{
						if (pTangentSet->miniBeta < 0)
							pTangentSet->miniBeta = pTangentSet->maxiBeta = iBeta;
						else
						{
							if (iBeta < pTangentSet->miniBeta)
								pTangentSet->miniBeta = iBeta;
							else if (iBeta > pTangentSet->maxiBeta)
								pTangentSet->maxiBeta = iBeta;
						}

						pTangent_ = NULL;

						for (i = 0; i < nTangents; i++)
						{
							RVLMEM_ALLOC_STRUCT(pMem2, VN_::EdgeTangent, pTangent);

							tangentArray[iBeta] = pTangent;

							RVLSCALE3VECTOR(dN, s[i], pTangent->N);
							RVLSUM3VECTORS(N1, pTangent->N, pTangent->N);
							RVLNORM3(pTangent->N, fTmp);

							pTangent->d = RVLDOTPRODUCT3(pTangent->N, P);

							d_ = RVLDOTPRODUCT3(pTangent->N, P_);

							if (d_ < pTangent->d)
								pTangent->d = d_;

							if (i == nTangents - 1)
							{
								if (pTangent_)
									pTangent_->pNext = pTangent;
								pTangent->pNext = NULL;
							}
							else
								pTangent_ = pTangent;
						}
					}
				}	// for (iBeta = 0; iBeta < betaArray.n; iBeta++)

				pTangentSet->tangentArray.n = pTangentSet->maxiBeta - pTangentSet->miniBeta + 1;

				RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, VN_::EdgeTangent *, pTangentSet->tangentArray.n, pTangentSet->tangentArray.Element);

				j = 0;

				for (i = pTangentSet->miniBeta; i <= pTangentSet->maxiBeta; i++, j++)
					pTangentSet->tangentArray.Element[j] = tangentArray[i];				
			}	// if (iVertex_ > iVertex)

			pVEdgePtr = pVEdgePtr->pNext;
		}	// while (pVEdgePtr)

		//if (debug == 1)
		//	int debug_ = 0;
	}	// for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)

	delete[] cb2;
	delete[] tangentArray;

	//FILE *fp = fopen("tangentRing.txt", "w");

	//iBeta = 2;

	//SURFEL::Vertex *pVertex_;

	//for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
	//{
	//	pVertex = pSurfels->vertexArray.Element[iVertex];

	//	pVEdgePtr = pVertex->EdgeList.pFirst;

	//	while (pVEdgePtr)
	//	{
	//		iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pVEdgePtr);

	//		if (iVertex_ > iVertex)
	//		{
	//			pVEdge = pVEdgePtr->pEdge;

	//			pVertex_ = pSurfels->vertexArray.Element[iVertex_];

	//			if (iBeta >= tangentSet[pVEdge->idx].miniBeta && iBeta <= tangentSet[pVEdge->idx].maxiBeta)
	//				fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\n", pVertex->P[0], pVertex->P[1], pVertex->P[2], pVertex_->P[0], pVertex_->P[1], pVertex_->P[2]);
	//		}

	//		pVEdgePtr = pVEdgePtr->pNext;
	//	}
	//}

	//fclose(fp);

	bool *bEdgeJoined = new bool[pSurfels->vertexEdgeArray.n];

	bool *bVertexJoined = new bool[pSurfels->vertexArray.n];

	memset(bVertexJoined, 0, pSurfels->vertexArray.n * sizeof(bool));

	Array<QList<RECOG::VN_::TorusRing>> ringListArray;

	ringListArray.Element = new QList<RECOG::VN_::TorusRing>[betaArray.n];
	ringListArray.n = betaArray.n;

	QList<RECOG::VN_::TorusRing> *pRingList;

	SURFEL::VertexEdge **VEdgeBuff = new SURFEL::VertexEdge *[pSurfels->vertexEdgeArray.n];

	for (iBeta = 0; iBeta < betaArray.n; iBeta++)
	{
		pRingList = ringListArray.Element + iBeta;

		RVLQLIST_INIT(pRingList);

		memset(bEdgeJoined, 0, pSurfels->vertexEdgeArray.n * sizeof(bool));

		for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
		{
			pVertex = pSurfels->vertexArray.Element[iVertex];

			pVEdgePtr = pVertex->EdgeList.pFirst;

			while (pVEdgePtr)
			{
				iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pVEdgePtr);

				if (iVertex_ > iVertex)
				{
					pVEdge = pVEdgePtr->pEdge;

					DetectTorusRings(pMesh, PArray, pSurfels, pVEdge, axis, iBeta, tangentSet, maxErr, pRingList, pMem, VEdgeBuff,
						bEdgeJoined, bVertexJoined);				
				}

				pVEdgePtr = pVEdgePtr->pNext;
			}
		}
	}

	int iRing;

	//FILE *fp = fopen("tangentRing.txt", "w");

	//iRing = 0;

	//pRingList = ringListArray.Element + 0;

	//RECOG::VN_::TorusRing *pRingDebug = pRingList->pFirst;

	//while (pRingDebug)
	//{
	//	for (i = 0; i < pRingDebug->iEdgeArray.n; i++)
	//	{
	//		pVEdge = pSurfels->vertexEdgeArray.Element[pRingDebug->iEdgeArray.Element[i]];

	//		P = pSurfels->vertexArray.Element[pVEdge->iVertex[0]]->P;
	//		P_ = pSurfels->vertexArray.Element[pVEdge->iVertex[1]]->P;

	//		fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\n", P[0], P[1], P[2], P_[0], P_[1], P_[2], pRingDebug->iBeta, iRing);
	//	}

	//	iRing++;

	//	pRingDebug = pRingDebug->pNext;
	//}

	//fclose(fp);

	delete[] tangentSet;
	delete[] bEdgeJoined;
	delete[] bVertexJoined;
	delete[] VEdgeBuff;

	float *ca = new float[alphaArray.n];
	float *sa = new float[alphaArray.n];

	int iAlpha;

	for (iAlpha = 0; iAlpha < alphaArray.n; iAlpha++)
	{
		ca[iAlpha] = cos(alphaArray.Element[iAlpha]);
		sa[iAlpha] = sin(alphaArray.Element[iAlpha]);
	}

	iRing = 0;

	VN_::TorusRing *pRing;
	float N[3];
	float d, dmin;
	int iKeyVertex;
	QList<QLIST::Ptr<RECOG::VN_::TorusRing>> *pCompList;

	for (iBeta = 0; iBeta < betaArray.n; iBeta++)
	{
		pRingList = ringListArray.Element + iBeta;

		pRing = pRingList->pFirst;

		while (pRing)
		{
			pRing->idx = iRing++;

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, alphaArray.n, pRing->d);
			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, alphaArray.n, pRing->iKeyVertex);

			pCompList = &(pRing->compList);

			RVLQLIST_INIT(pCompList);

			pRing->bLast = true;

			for (iAlpha = 0; iAlpha < alphaArray.n; iAlpha++)
			{
				N[0] = ca[iAlpha] * sb[iBeta];
				N[1] = sa[iAlpha] * sb[iBeta];
				N[2] = cb[iBeta];

				iKeyVertex = pRing->iVertexArray.Element[0];

				P = PArray + 3 * iKeyVertex;

				dmin = RVLDOTPRODUCT3(N, P);

				for (i = 1; i < pRing->iVertexArray.n; i++)
				{
					iVertex = pRing->iVertexArray.Element[i];

					P = PArray + 3 * iVertex;
			
					d = RVLDOTPRODUCT3(N, P);

					if (d < dmin)
					{
						dmin = d;

						iKeyVertex = iVertex;
					}
				}

				pRing->d[iAlpha] = dmin;
				pRing->iKeyVertex[iAlpha] = iKeyVertex;
			}

			pRing = pRing->pNext;
		}
	}

	int iPrevBeta = 0;

	VN_::TorusRing *pPrevRing;
	float e;
	QList<RECOG::VN_::TorusRing> *pPrevRingList;	
	QLIST::Ptr<RECOG::VN_::TorusRing> *pCompListEntry;

	for (iBeta = 1; iBeta < betaArray.n; iBeta++, iPrevBeta++)
	{
		pRingList = ringListArray.Element + iBeta;

		pRing = pRingList->pFirst;
		
		while (pRing)
		{
			pCompList = &(pRing->compList);

			pPrevRingList = ringListArray.Element + iPrevBeta;

			pPrevRing = pPrevRingList->pFirst;

			while (pPrevRing)
			{
				for (iAlpha = 0; iAlpha < alphaArray.n; iAlpha++)
				{
					N[0] = ca[iAlpha] * sb[iBeta];
					N[1] = sa[iAlpha] * sb[iBeta];
					N[2] = cb[iBeta];

					P = PArray + 3 * pPrevRing->iKeyVertex[iAlpha];

					e = RVLDOTPRODUCT3(N, P) - pRing->d[iAlpha];

					if (e > maxErr)
						break;
				}

				if (iAlpha >= alphaArray.n)
				{
					for (iAlpha = 0; iAlpha < alphaArray.n; iAlpha++)
					{
						N[0] = ca[iAlpha] * sb[iPrevBeta];
						N[1] = sa[iAlpha] * sb[iPrevBeta];
						N[2] = cb[iPrevBeta];

						P = PArray + 3 * pRing->iKeyVertex[iAlpha];

						e = RVLDOTPRODUCT3(N, P) - pPrevRing->d[iAlpha];

						if (e > maxErr)
							break;
					}

					if (iAlpha >= alphaArray.n)
					{
						RVLMEM_ALLOC_STRUCT(pMem, QLIST::Ptr<RECOG::VN_::TorusRing>, pCompListEntry);

						RVLQLIST_ADD_ENTRY(pCompList, pCompListEntry);

						pCompListEntry->ptr = pPrevRing;

						pPrevRing->bLast = false;
					}
				}

				pPrevRing = pPrevRing->pNext;
			}

			pRing = pRing->pNext;
		}
	}

	QList<VN_::TorusTreeNode> torusQueue;

	QList<VN_::TorusTreeNode> *pTorusQueue = &torusQueue;

	RVLQLIST_INIT(pTorusQueue);

	VN_::TorusTreeNode *pTorusTreeNode;

	for (iBeta = 0; iBeta < betaArray.n; iBeta++)
	{
		pRingList = ringListArray.Element + iBeta;

		pRing = pRingList->pFirst;

		while (pRing)
		{
			if (pRing->bLast)
			{
				RVLMEM_ALLOC_STRUCT(pMem2, VN_::TorusTreeNode, pTorusTreeNode);

				RVLQLIST_ADD_ENTRY(pTorusQueue, pTorusTreeNode);

				pTorusTreeNode->pRing = pRing;
				pTorusTreeNode->pParent = NULL;
			}

			pRing = pRing->pNext;
		}
	}

	QList<VN_::Torus> torusList;

	QList<VN_::Torus> *pTorusList = &torusList;

	RVLQLIST_INIT(pTorusList);

	SClusters.n = 0;

	VN_::TorusTreeNode *pTorusTreeNode_;
	VN_::Torus *pTorus;

	pTorusTreeNode = pTorusQueue->pFirst;

	while (pTorusTreeNode)
	{
		pCompListEntry = pTorusTreeNode->pRing->compList.pFirst;

		if (pCompListEntry)
		{
			while (pCompListEntry)
			{
				RVLMEM_ALLOC_STRUCT(pMem2, VN_::TorusTreeNode, pTorusTreeNode_);

				RVLQLIST_ADD_ENTRY(pTorusQueue, pTorusTreeNode_);

				pTorusTreeNode_->pRing = pCompListEntry->ptr;

				pTorusTreeNode_->pParent = pTorusTreeNode;

				pCompListEntry = pCompListEntry->pNext;
			}
		}
		else
		{
			RVLMEM_ALLOC_STRUCT(pMem, VN_::Torus, pTorus);

			RVLQLIST_ADD_ENTRY(pTorusList, pTorus);

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, VN_::TorusRing *, betaArray.n, pTorus->ringArray.Element);

			memset(pTorus->ringArray.Element, 0, betaArray.n * sizeof(VN_::TorusRing *));

			pTorus->ringArray.n = betaArray.n;

			pTorusTreeNode_ = pTorusTreeNode;

			while (pTorusTreeNode_)
			{
				pTorus->ringArray.Element[pTorusTreeNode_->pRing->iBeta] = pTorusTreeNode_->pRing;

				pTorusTreeNode_ = pTorusTreeNode_->pParent;
			}

			SClusters.n++;
		}

		pTorusTreeNode = pTorusTreeNode->pNext;
	}

	delete[] ringListArray.Element;
	delete[] ca;
	delete[] sa;
	delete[] cb;
	delete[] sb;

	SClusters.Element = new VN_::Torus *[SClusters.n];

	QLIST::CreatePtrArray<VN_::Torus>(pTorusList, &SClusters);
}

void VN::DetectTorusRings(
	Mesh *pMesh,
	float *PArray,
	SurfelGraph *pSurfels,
	SURFEL::VertexEdge *pVEdge0,
	float *axis,
	int iBeta,
	VN_::EdgeTangentSet *tangentSet,
	float maxError,
	QList<RECOG::VN_::TorusRing> *pRingList,
	CRVLMem *pMem,
	SURFEL::VertexEdge **VEdgeBuff,
	bool *bEdgeJoined,
	bool *bVertexJoined)
{
	if (iBeta < tangentSet[pVEdge0->idx].miniBeta || iBeta > tangentSet[pVEdge0->idx].maxiBeta || bEdgeJoined[pVEdge0->idx])
		return;

	int minnEdges = 5;

	SURFEL::VertexEdge **pVEdgePush = VEdgeBuff;

	bEdgeJoined[pVEdge0->idx] = true;

	*(pVEdgePush++) = pVEdge0;

	SURFEL::VertexEdge **pVEdgeFetch = VEdgeBuff;

	int i, j;
	int iVertex, iVertex_;
	SURFEL::Vertex *pVertex, *pVertex_;
	SURFEL::VertexEdge *pVEdge, *pVEdge_;
	GRAPH::EdgePtr2<SURFEL::VertexEdge> *pVEdgePtr;
	float dN[3];
	float *N1, *N2;
	float an1, adn, n1dn, dndn, a, b, c, k, g;
	float s;
	bool bRing;

	while (pVEdgePush > pVEdgeFetch)
	{
		pVEdge = *(pVEdgeFetch++);

		for (j = 0; j < 2; j++)
		{
			pVEdgePtr = pSurfels->vertexArray.Element[pVEdge->iVertex[j]]->EdgeList.pFirst;

			while (pVEdgePtr)
			{
				pVEdge_ = pVEdgePtr->pEdge;

				if (iBeta >= tangentSet[pVEdge_->idx].miniBeta && iBeta <= tangentSet[pVEdge_->idx].maxiBeta && !bEdgeJoined[pVEdge_->idx])
				{
					bEdgeJoined[pVEdge_->idx] = true;

					*(pVEdgePush++) = pVEdge_;
				}	// if (!bJoined[pVEdge_->idx])

				pVEdgePtr = pVEdgePtr->pNext;
			}	// for every neighbor edge
		}	// for (j = 0; j < 2; j++)
	}	// main loop

	SURFEL::VertexEdge **pVEdgeBuffEnd = pVEdgeFetch;

	int nEdges = pVEdgeBuffEnd - VEdgeBuff;

	if (nEdges < minnEdges)
		return;

	int nEdges2 = nEdges * nEdges;

	bool *bComp = new bool[nEdges2];

	float *EMem = new float[2 * nEdges2];
	
	float *E[2];

	E[0] = EMem;
	E[1] = EMem + nEdges2;

	int l;
	float *P;
	float e;
	VN_::EdgeTangent *pTangent, *pTangent_;
	int iComp, iComp_;

	for (i = 0; i < nEdges; i++)
	{
		pVEdge = VEdgeBuff[i];

		pTangent = tangentSet[pVEdge->idx].tangentArray.Element[iBeta - tangentSet[pVEdge->idx].miniBeta];

		for (j = i + 1; j < nEdges; j++)
		{
			pVEdge_ = VEdgeBuff[j];

			iComp = i * nEdges + j;
			iComp_ = j * nEdges + i;

			bComp[iComp] = false;

			for (l = 0; l < 2; l++)
			{
				P = PArray + 3 * pVEdge_->iVertex[l];

				e = RVLDOTPRODUCT3(pTangent->N, P) - pTangent->d;

				if (e < -maxError)
					break;

				E[l][iComp] = e;
			}

			if (l >= 2)
			{
				pTangent_ = tangentSet[pVEdge_->idx].tangentArray.Element[iBeta - tangentSet[pVEdge_->idx].miniBeta];

				for (l = 0; l < 2; l++)
				{
					P = PArray + 3 * pVEdge->iVertex[l];

					e = RVLDOTPRODUCT3(pTangent_->N, P) - pTangent_->d;

					if (e < -maxError)
						break;

					E[l][iComp_] = e;
				}

				if (l >= 2)
					bComp[iComp] = true;
			}

			bComp[iComp_] = bComp[iComp];
		}

		iComp = i * nEdges + i;
		bComp[iComp] = true;
		E[0][iComp] = E[1][iComp] = 0.0f;
	}
	
	Array<SortIndex<float>> sortedTangentArray;
	
	sortedTangentArray.Element = new SortIndex<float>[nEdges];
	sortedTangentArray.n = nEdges;

	VN_::EdgeTangentSet *pTangentSet;
	float score;

	for (i = 0; i < nEdges; i++)
	{
		pVEdge = VEdgeBuff[i];

		pTangentSet = tangentSet + pVEdge->idx;

		sortedTangentArray.Element[i].idx = i;

		score = pTangentSet->edgeLength;

		for (j = 0; j < nEdges; j++)
		{
			pVEdge_ = VEdgeBuff[j];

			iComp = i * nEdges + j;

			if (!bComp[iComp])
				score -= tangentSet[pVEdge_->idx].edgeLength;
		}

		sortedTangentArray.Element[i].cost = score;
	}

	BubbleSort<SortIndex<float>>(sortedTangentArray, true);

	Array<int> ringEdgeArray;

	ringEdgeArray.Element = new int[nEdges];

	ringEdgeArray.n = 0;

	bool *bComp_;
	int i_;

	for (i = 0; i < nEdges; i++)
	{
		i_ = sortedTangentArray.Element[i].idx;

		bComp_ = bComp + i_ * nEdges;

		for (j = 0; j < ringEdgeArray.n; j++)
			if (!bComp_[ringEdgeArray.Element[j]])
				break;

		if (j >= ringEdgeArray.n)
			ringEdgeArray.Element[ringEdgeArray.n++] = i_;
	}

	//if (ringEdgeArray.n >= minnEdges && ringEdgeArray.n > nEdges / 2)
	if (ringEdgeArray.n >= minnEdges)
	{
		for (i = 0; i < ringEdgeArray.n; i++)
		{
			for (j = 0; j < ringEdgeArray.n; j++)
			{
				iComp = ringEdgeArray.Element[i] * nEdges + ringEdgeArray.Element[j];

				for (l = 0; l < 2; l++)
					if (E[l][iComp] > maxError)
						break;

				if (l < 2)
					break;
			}

			if (j >= ringEdgeArray.n)
				break;
		}

		if (i >= ringEdgeArray.n)
		{
			RECOG::VN_::TorusRing *pRing;

			RVLMEM_ALLOC_STRUCT(pMem, RECOG::VN_::TorusRing, pRing);

			RVLQLIST_ADD_ENTRY(pRingList, pRing);

			pRing->iBeta = iBeta;

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, 2 * ringEdgeArray.n, pRing->iVertexArray.Element);

			pRing->iVertexArray.n = 0;

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, ringEdgeArray.n, pRing->iEdgeArray.Element);

			pRing->iEdgeArray.n = 0;

			for (i = 0; i < ringEdgeArray.n; i++)
			{
				pVEdge = VEdgeBuff[ringEdgeArray.Element[i]];

				pRing->iEdgeArray.Element[pRing->iEdgeArray.n++] = pVEdge->idx;

				for (j = 0; j < 2; j++)
				{
					iVertex = pVEdge->iVertex[j];

					//if (iVertex >= pSurfels->vertexArray.n)
					//	int debug = 0;

					if (!bVertexJoined[iVertex])
					{
						bVertexJoined[iVertex] = true;

						pRing->iVertexArray.Element[pRing->iVertexArray.n++] = iVertex;
					}
				}
			}

			for (i = 0; i < pRing->iVertexArray.n; i++)
				bVertexJoined[pRing->iVertexArray.Element[i]] = false;
		}
	}

	//if (iBeta == 2)
	//{
	//	FILE *fp = fopen("tangentRing.txt", "w");

	//	bool *bRingMember = new bool[nEdges];

	//	memset(bRingMember, 0, nEdges * sizeof(bool));

	//	for (i = 0; i < ringEdgeArray.n; i++)
	//		bRingMember[ringEdgeArray.Element[i]] = true;

	//	for (i = 0; i < nEdges; i++)
	//	{
	//		pVertex = pSurfels->vertexArray.Element[VEdgeBuff[i]->iVertex[0]];
	//		pVertex_ = pSurfels->vertexArray.Element[VEdgeBuff[i]->iVertex[1]];

	//		fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%d\n", 
	//			pVertex->P[0], pVertex->P[1], pVertex->P[2], pVertex_->P[0], pVertex_->P[1], pVertex_->P[2], 
	//			(bRingMember[i] ? 1 : 0));
	//	}

	//	delete[] bRingMember;

	//	fclose(fp);
	//}

	delete[] bComp;
	delete[] EMem;
	delete[] sortedTangentArray.Element;
	delete[] ringEdgeArray.Element;
}

void VN::Project(
	float *d,
	float *R,
	float *t,
	Camera camera,
	Array2D<float> imgPtArray,
	Array2D<float> PtArray)
{
	float s_ = RVLDOTPRODUCT3(R, R);

	float PcM[3];

	RVLMULMX3X3TVECT(R, t, PcM);
	RVLSCALE3VECTOR2(PcM, -s_, PcM);

	s_ = sqrt(s_);

	int iNode;
	VN_::Node *pNode;
	Pair<float, float> *pProjectionInterval;
	float *N;

	for (iNode = 0; iNode < featureArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		N = pNode->pFeature->N;

		dc[iNode] = d[iNode] - RVLDOTPRODUCT3(N, PcM);
	}

	int iPt;
	float *m, *P;
	float r[3], rM[3];
	float fTmp;
	float sM, s;

	for (iPt = 0; iPt < imgPtArray.h; iPt++)
	{
		if (iPt == 90)
			int debug = 0;

		m = imgPtArray.Element + imgPtArray.w * iPt;

		r[0] = (m[0] - camera.uc) / camera.fu;
		r[1] = (m[1] - camera.vc) / camera.fv;
		r[2] = 1.0f;

		RVLNORM3(r, fTmp);

		RVLMULMX3X3TVECT(R, r, rM);

		RVLNORM3(rM, fTmp);

		sM = Project(dc, rM);

		//if (s < 1000.0f)
		//	int debug = 0;

		P = PtArray.Element + PtArray.w * iPt;

		s = sM * s_;

		RVLSCALE3VECTOR(r, s, P);
	}
}

float VN::Project(
	float *d,
	float *r)
{
	int iNode;
	VN_::Node *pNode;
	Array<Pair<float, float>> *pProjectionInterval;
	float *N;
	float d_;
	float c;

	for (iNode = 0; iNode < featureArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pProjectionInterval = projectionIntervals.Element + iNode;

		N = pNode->pFeature->N;

		c = RVLDOTPRODUCT3(N, r);

		if (c < -1e-6)
		{
			pProjectionInterval->Element[0].a = d[iNode] / c;
			pProjectionInterval->Element[0].b = 1e6;
			pProjectionInterval->n = 1;
		}
		else if (c < 1e-6)
		{
			if (d[iNode] >= 0.0f)
			{
				pProjectionInterval->Element[0].a = -1e6;
				pProjectionInterval->Element[0].b = 1e6;
				pProjectionInterval->n = 1;
			}
			else
				pProjectionInterval->n = 0;
		}
		else
		{
			pProjectionInterval->Element[0].a = -1e6;
			pProjectionInterval->Element[0].b = d[iNode] / c;
			pProjectionInterval->n = 1;
		}
	}

	for (; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pProjectionInterval = projectionIntervals.Element + iNode;

		if (pNode->operation > 0)
		{
			pProjectionInterval->Element[0].a = -1e6;
			pProjectionInterval->Element[0].b = 1e6;
			pProjectionInterval->n = 1;
		}
		else
			pProjectionInterval->n = 0;
	}

	RECOG::VN_::Edge *pEdge = EdgeList.pFirst;

	VN_::Node *pParentNode;
	Array<Pair<float, float>> *pChildInterval, *pParentInterval;
	int i, iPrev, j;
	int bOpen[2];
	int iInterval[2];
	int iNewInterval;
	float nextPt[2];
	int iNext;
	bool b1, b2;
	int nb, nbPrev;
	int nOpen, nClosed;
	
	while (pEdge)
	{
		//if (pEdge->data.b == 181)
		//	int debug = 0;

		pChildInterval = projectionIntervals.Element + pEdge->data.a;

		pParentInterval = projectionIntervals.Element + pEdge->data.b;

		pParentNode = NodeArray.Element + pEdge->data.b;

		nOpen = (pParentNode->operation > 0 ? 2 : 1);

		nClosed = nOpen - 1;

		projectionIntervalBuff.n = pParentInterval->n;

		for (i = 0; i < pParentInterval->n; i++)
			projectionIntervalBuff.Element[i] = pParentInterval->Element[i];

		bOpen[0] = bOpen[1] = 0;

		iInterval[0] = iInterval[1] = 0;

		nb = 0;

		pParentInterval->n = 0;

		while (true)
		{
			if (b1 = (iInterval[0] < pChildInterval->n))
				nextPt[0] = (bOpen[0] > 0 ? pChildInterval->Element[iInterval[0]].b : pChildInterval->Element[iInterval[0]].a);
			if (b2 = (iInterval[1] < projectionIntervalBuff.n))
				nextPt[1] = (bOpen[1] > 0 ? projectionIntervalBuff.Element[iInterval[1]].b : projectionIntervalBuff.Element[iInterval[1]].a);
			if (b1 && b2)
				iNext = (nextPt[0] <= nextPt[1] ? 0 : 1);
			else if (b1)
				iNext = 0;
			else if (b2)
				iNext = 1;
			else
				break;
			bOpen[iNext] = 1 - bOpen[iNext];
			nbPrev = nb;
			nb = bOpen[0] + bOpen[1];
			if (nb == nOpen && nbPrev == nClosed)
				pParentInterval->Element[pParentInterval->n].a = nextPt[iNext];
			else if (nb == nClosed && nbPrev == nOpen)
			{
				pParentInterval->Element[pParentInterval->n].b = nextPt[iNext];
				pParentInterval->n++;
			}
			if (bOpen[iNext] == 0)
				iInterval[iNext]++;
		}

		pEdge = pEdge->pNext;
	}	// while (pEdge)

	return (projectionIntervals.Element[iy].n > 0 ? projectionIntervals.Element[iy].Element[0].a : 1e6);
}

void VN::Load(
	char *fileName,
	CRVLMem *pMem)
{
	FILE *fp = fopen(fileName, "r");	

	int nEdges;

	fscanf(fp, "%d\t%d\t%d\t%d\n", &(featureArray.n), &(NodeArray.n), &nEdges, &iy);

	RVL_DELETE_ARRAY(featureArray.Element);

	featureArray.Element = new RECOG::VN_::Feature[featureArray.n];

	RECOG::VN_::Feature *pFeature = featureArray.Element;

	RVL_DELETE_ARRAY(NodeArray.Element);

	NodeArray.Element = new RECOG::VN_::Node[NodeArray.n];

	RECOG::VN_::Node *pNode = NodeArray.Element;

	int iFeature;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++, pFeature++, pNode++)
	{
		fscanf(fp, "%f\t%f\t%f\t%f\n", pFeature->N, pFeature->N + 1, pFeature->N + 2, &(pFeature->d));

		pNode->operation = 0;
		pNode->fOperation = 0.0f;
		pNode->iFeature = iFeature;
		pNode->pFeature = pFeature;
	}

	int iNode;
	
	for (iNode = featureArray.n; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		fscanf(fp, "%d\n", &(pNode->operation));

		pNode->fOperation = (float)(pNode->operation);
		pNode->iFeature = -1;
		pNode->pFeature = NULL;
	}

	RECOG::VN_::Edge *pEdge;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RECOG::VN_::Edge, nEdges, pEdge);

	QList<RECOG::VN_::Edge> *pEdgeList = &EdgeList;

	RVLQLIST_INIT(pEdgeList);

	int iEdge, bPrimary;

	for (iEdge = 0; iEdge < nEdges; iEdge++, pEdge++)
	{
		fscanf(fp, "%d\t%d\t%d\n", &(pEdge->data.a), &(pEdge->data.b), &bPrimary);

		pEdge->bPrimary = (bPrimary > 0);

		RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);
	}

	fclose(fp);
}

void VN::Display(
	Visualizer *pVisualizer,
	Box<float> box,
	float resolution,
	float *d,
	bool *bd,
	float SDFSurfaceValue)
{
	printf("Computing SDF...");

	Array3D<float> f;

	float a = box.maxx - box.minx;
	float b = box.maxy - box.miny;
	float c = box.maxz - box.minz;

	f.a = (int)floor(a / resolution) + 1;
	f.b = (int)floor(b / resolution) + 1;
	f.c = (int)floor(c / resolution) + 1;

	int nSamples = f.a * f.b * f.c;

	f.Element = new float[nSamples];

	float *SDF = new float[featureArray.n];

	int i, j, k;
	float P[3];
	int iActiveFeature;

	for (k = 0; k < f.c; k++)
		for (j = 0; j < f.b; j++)
			for (i = 0; i < f.a; i++)
	{
		P[0] = (float)i * resolution + box.minx;
		P[1] = (float)j * resolution + box.miny;
		P[2] = (float)k * resolution + box.minz;

		f.Element[f.a * (f.b * k + j) + i] = Evaluate(P, SDF, iActiveFeature, true, d, bd);
	}
		
	delete[] SDF;

	printf("completed.\n");

	float P0_[3];

	P0_[0] = box.minx;
	P0_[1] = box.miny;
	P0_[2] = box.minz;

	vtkSmartPointer<vtkPolyData> polyData = DisplayIsoSurface(f, P0_, resolution, SDFSurfaceValue);

	// Create a mapper and actor.
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	pVisualizer->renderer->AddActor(actor);

	delete[] f.Element;
}

void VN::PrintTorus(
	FILE *fp,
	SurfelGraph *pSurfels,
	VN_::Torus *pTorus,
	int iTorus)
{
	int i;
	int iRing;
	float *P, *P_;
	RECOG::VN_::TorusRing *pRing;
	SURFEL::VertexEdge *pVEdge;

	for (iRing = 0; iRing < pTorus->ringArray.n; iRing++)
	{
		pRing = pTorus->ringArray.Element[iRing];

		if (pRing)
		{
			for (i = 0; i < pRing->iEdgeArray.n; i++)
			{
				pVEdge = pSurfels->vertexEdgeArray.Element[pRing->iEdgeArray.Element[i]];

				P = pSurfels->vertexArray.Element[pVEdge->iVertex[0]]->P;
				P_ = pSurfels->vertexArray.Element[pVEdge->iVertex[1]]->P;

				fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%d\n", P[0], P[1], P[2], P_[0], P_[1], P_[2], pRing->iBeta, pRing->idx, iTorus);
			}
		}
	}
}

void VN::PrintTori(
	FILE *fp,
	SurfelGraph *pSurfels,
	Array<RECOG::VN_::Torus *> torusArray)
{
	int iTorus;

	for (iTorus = 0; iTorus < torusArray.n; iTorus++)
		PrintTorus(fp, pSurfels, torusArray.Element[iTorus], iTorus);
}

void VN_::CreateConvex(
	VN *pVN,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 0;
	iBetaInterval.b = 8;

	pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, 16, 8, iBetaInterval, pMem);

	pVN->SetOutput(0);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateTorus(
	VN *pVN,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 0;
	iBetaInterval.b = 8;

	pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, 16, 8, iBetaInterval, pMem);

	iBetaInterval.a = 1;
	iBetaInterval.b = 7;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_XTORUS, R, t, 0.2f, 16, 8, iBetaInterval, pMem, 0.1f);

	pVN->AddOperation(2, 1, 0, 1, pMem);

	pVN->SetOutput(2);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateBanana(
	VN *pVN,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 0;
	iBetaInterval.b = 8;

	pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, 16, 8, iBetaInterval, pMem);

	Pair<int, int> iAlphaInterval;

	iAlphaInterval.a = 9;
	iAlphaInterval.b = 13;

	iBetaInterval.a = 1;
	iBetaInterval.b = 7;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_XTORUS, R, t, 0.2f, 16, 8, iBetaInterval, pMem, 0.1f, iAlphaInterval);

	pVN->AddOperation(2, 1, 0, 1, pMem);

	pVN->SetOutput(2);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateBottle(
	VN *pVN,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, -0.25f);

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 1;
	iBetaInterval.b = 8;

	pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.25f, 16, 8, iBetaInterval, pMem);

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.4f);

	iBetaInterval.a = 0;
	iBetaInterval.b = 4;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.1f, 16, 8, iBetaInterval, pMem);

	pVN->AddLimit(1, 0, 0, 0, pMem);
	
	int iAlpha, iBeta;

	for (iBeta = 1; iBeta <= 2; iBeta++)
		for (iAlpha = 0; iAlpha < 16; iAlpha++)
			pVN->AddLimit(1, iAlpha, iBeta, 0, pMem);	

	for (iBeta = 5; iBeta <= 6; iBeta++)
		for (iAlpha = 0; iAlpha <= 16; iAlpha++)
			pVN->AddLimit(0, iAlpha, iBeta, 1, pMem);

	pVN->AddLimit(0, 0, 7, 1, pMem);

	pVN->AddOperation(2, -1, 0, 1, pMem);

	pVN->SetOutput(2);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateHammer(
	VN *pVN,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, -0.25f);

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 0;
	iBetaInterval.b = 8;

	pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.25f, 16, 8, iBetaInterval, pMem);

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.4f);

	iBetaInterval.a = 0;
	iBetaInterval.b = 4;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.1f, 16, 8, iBetaInterval, pMem);

	int iAlpha, iBeta;

	for (iBeta = 6; iBeta <= 7; iBeta++)
		for (iAlpha = 0; iAlpha <= 16; iAlpha++)
			pVN->AddLimit(0, iAlpha, iBeta, 1, pMem);

	pVN->AddLimit(0, 0, 8, 1, pMem);

	pVN->AddOperation(2, -1, 0, 1, pMem);

	pVN->SetOutput(2);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateBowl(
	VN *pVN,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 0;
	iBetaInterval.b = 8;

	pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, 16, 8, iBetaInterval, pMem);

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	iBetaInterval.a = 0;
	iBetaInterval.b = 4;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.3f, 16, 8, iBetaInterval, pMem);

	pVN->AddOperation(2, 1, 0, 1, pMem);

	pVN->SetOutput(2);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateMug(
	VN *pVN,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 0;
	iBetaInterval.b = 8;

	pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, 16, 8, iBetaInterval, pMem);

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	iBetaInterval.a = 0;
	iBetaInterval.b = 4;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.3f, 16, 8, iBetaInterval, pMem);

	iBetaInterval.a = 0;
	iBetaInterval.b = 8;

	pVN->AddModelCluster(2, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, 16, 8, iBetaInterval, pMem);

	pVN->AddOperation(3, -1, 0, 2, pMem);

	pVN->AddOperation(4, 1, 3, 1, pMem);

	pVN->SetOutput(4);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void RVL::SampleMesh(
	Mesh *pMesh,
	float *R,
	float *t,
	Array<VN_::Sample> &sampleArray)
{
	int nSamplePts = sampleArray.n;

	Array<int> iPtArray;

	iPtArray.n = pMesh->NodeArray.n;

	RandomIndices(iPtArray);

	int i;
	float *P_;
	VN_::Sample *pSample;

	for (i = 0; i < nSamplePts; i++)
	{
		pSample = sampleArray.Element + i;

		P_ = pMesh->NodeArray.Element[iPtArray.Element[i]].P;

		RVLTRANSF3(P_, R, t, pSample->P);

		pSample->SDF = 0.0f;
	}
}

void RVL::SampleMeshDistanceFunction(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	float voxelSize,
	int sampleVoxelDistance,
	Array3D<RECOG::VN_::Voxel> &volume,
	float *P0,
	Array<RECOG::VN_::Sample> &sampleArray,
	Box<float> &boundingBox)
{
	// volume <- empty 3D voxel array with voxel size specified by voxelSize.
	// It is larger than the bounding box of pMesh for sampleVoxelDistance + 1 on each side.

	pMesh->BoundingBox(&boundingBox);

	int border = 2 * sampleVoxelDistance + 1;

	int nx = (int)ceil(0.5f * (boundingBox.maxx - boundingBox.minx) / voxelSize) + border;
	int ny = (int)ceil(0.5f * (boundingBox.maxy - boundingBox.miny) / voxelSize) + border;
	int nz = (int)ceil(0.5f * (boundingBox.maxz - boundingBox.minz) / voxelSize) + border;

	float a = voxelSize * (float)nx;
	float b = voxelSize * (float)ny;
	float c = voxelSize * (float)nz;

	float center[3];

	BoxCenter<float>(&boundingBox, center);

	Box<float> box;

	box.minx = center[0] - a;
	box.miny = center[1] - b;
	box.minz = center[2] - c;
	box.maxx = center[0] + a;
	box.maxy = center[1] + b;
	box.maxz = center[2] + c;

	volume.a = 2 * nx;
	volume.b = 2 * ny;
	volume.c = 2 * nz;

	int nVoxels = volume.a * volume.b * volume.c;

	volume.Element = new RECOG::VN_::Voxel[nVoxels];

	// Assign mesh points to volume voxels.
	// Set voxelDistance field of all voxels to -1.

	int i;
	QList<QLIST::Index> *pPtList;
	RECOG::VN_::Voxel *pVoxel;

	for (i = 0; i < nVoxels; i++)
	{
		pVoxel = volume.Element + i;

		pPtList = &(pVoxel->PtList);

		RVLQLIST_INIT(pPtList);

		pVoxel->voxelDistance = -1;
	}

	QLIST::Index *PtMem = new QLIST::Index[pMesh->NodeArray.n];

	QLIST::Index *pPtIdx = PtMem;

	float *P;
	int iPt;
	int j, k;

	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		P = pMesh->NodeArray.Element[iPt].P;

		i = (int)floor((P[0] - box.minx) / voxelSize);
		j = (int)floor((P[1] - box.miny) / voxelSize);
		k = (int)floor((P[2] - box.minz) / voxelSize);

		pVoxel = RVL3DARRAY_ELEMENT(volume, i, j, k);

		pPtList = &(pVoxel->PtList);

		pPtIdx->Idx = iPt;

		RVLQLIST_ADD_ENTRY(pPtList, pPtIdx);

		pPtIdx++;
	}

	// Assign distance function value to all voxels outside pMesh.

	int *RGBuff = new int[nVoxels];

	int *pPut = RGBuff;
	int *pFetch = RGBuff;

	Array<int> zeroDistanceVoxelArray;

	zeroDistanceVoxelArray.Element = new int[nVoxels];

	zeroDistanceVoxelArray.n = 0;

	*(pPut++) = 0;

	int dijk[][3] = {
		{ -1, 0, 0 },
		{ 1, 0, 0 },
		{ 0, -1, 0 },
		{ 0, 1, 0 },
		{ 0, 0, -1 },
		{ 0, 0, 1 } };

	//int maxVoxelDistance = volume.a + volume.b + volume.c;

	int maxVoxelDistance = sampleVoxelDistance + 1;

	int iVoxel, iVoxel_;
	int i_, j_, k_, l;

	while (pPut > pFetch)
	{
		iVoxel = (*pFetch++);

		RVL3DARRAY_INDICES(volume, iVoxel, i, j, k);

		for (l = 0; l < 6; l++)
		{
			i_ = i + dijk[l][0];
			j_ = j + dijk[l][1];
			k_ = k + dijk[l][2];

			if (i_ >= 0 && i_ < volume.a && j_ >= 0 && j_ < volume.b && k_ >= 0 && k_ < volume.c)
			{
				iVoxel_ = RVL3DARRAY_INDEX(volume, i_, j_, k_);

				pVoxel = volume.Element + iVoxel_;

				if (pVoxel->voxelDistance >= 0)
					continue;

				if (pVoxel->PtList.pFirst)
				{
					pVoxel->voxelDistance = 0;

					zeroDistanceVoxelArray.Element[zeroDistanceVoxelArray.n++] = iVoxel_;
				}
				else
				{
					pVoxel->voxelDistance = maxVoxelDistance;

					*(pPut++) = iVoxel_;
				}
			}
		}
	}

	// sampleArray <- array of sample points at distance approximatelly equal to sampleVoxelDistance.
	// Field SDF of every sample represents the distance function value.

	sampleArray.Element = new RECOG::VN_::Sample[nVoxels];

	sampleArray.n = 0;

	float halfVoxelSize = 0.5f * voxelSize;

	P0[0] = box.minx + halfVoxelSize;
	P0[1] = box.miny + halfVoxelSize;
	P0[2] = box.minz + halfVoxelSize;

	pPut = RGBuff + zeroDistanceVoxelArray.n;

	pFetch = RGBuff;

	memcpy(RGBuff, zeroDistanceVoxelArray.Element, zeroDistanceVoxelArray.n * sizeof(int));

	float maxeSDF = voxelSize * (sampleVoxelDistance + 2);

	float maxDist = maxeSDF * maxeSDF;

	int voxelDistance;
	int i__, j__, k__;
	int p, q, r;
	float dist, minDist;
	int iVoxel__;
	float *P_;
	float dP[3];
	int iClosestPt;
	VN_::Sample *pSample;
	float SDF, eSDF, mineSDF;
	Surfel *pFeature;
	int iFeature;

	while (pPut > pFetch)
	{
		iVoxel = (*pFetch++);

		pVoxel = volume.Element + iVoxel;

		voxelDistance = pVoxel->voxelDistance + 1;

		RVL3DARRAY_INDICES(volume, iVoxel, i, j, k);

		for (l = 0; l < 6; l++)
		{
			i_ = i + dijk[l][0];
			j_ = j + dijk[l][1];
			k_ = k + dijk[l][2];

			if (i_ >= 0 && i_ < volume.a && j_ >= 0 && j_ < volume.b && k_ >= 0 && k_ < volume.c)
			{
				iVoxel_ = RVL3DARRAY_INDEX(volume, i_, j_, k_);

				pVoxel = volume.Element + iVoxel_;

				if (pVoxel->voxelDistance > voxelDistance)
				{
					pVoxel->voxelDistance = voxelDistance;

					*(pPut++) = iVoxel_;

					if (voxelDistance == sampleVoxelDistance)
					{
						pSample = sampleArray.Element + sampleArray.n;

						P = pSample->P;

						P[0] = (float)i_ * voxelSize;
						P[1] = (float)j_ * voxelSize;
						P[2] = (float)k_ * voxelSize;

						RVLSUM3VECTORS(P, P0, P);

						minDist = maxDist;

						iClosestPt = -1;

						for (k__ = k_ - sampleVoxelDistance; k__ <= k_ + sampleVoxelDistance; k__++)
							for (j__ = j_ - sampleVoxelDistance; j__ <= j_ + sampleVoxelDistance; j__++)
								for (i__ = i_ - sampleVoxelDistance; i__ <= i_ + sampleVoxelDistance; i__++)
								{
									if (i__ == i_ && j__ == j_ && k__ == k_)
										continue;

									iVoxel__ = RVL3DARRAY_INDEX(volume, i__, j__, k__);

									pPtIdx = volume.Element[iVoxel__].PtList.pFirst;

									while (pPtIdx)
									{
										P_ = pMesh->NodeArray.Element[pPtIdx->Idx].P;

										RVLDIF3VECTORS(P_, P, dP);

										dist = RVLDOTPRODUCT3(dP, dP);

										if (dist < minDist)
										{
											minDist = dist;

											iClosestPt = pPtIdx->Idx;

											p = i__;
											q = j__;
											r = k__;
										}

										pPtIdx = pPtIdx->pNext;
									}
								}

						pSample->iFeature = -1;

						mineSDF = maxeSDF;

						if (iClosestPt >= 0)
						{
							pSample->SDF = sqrt(minDist);

							for (k__ = r - 1; k__ <= r + 1; k__++)
								for (j__ = q - 1; j__ <= q + 1; j__++)
									for (i__ = p - 1; i__ <= p + 1; i__++)
									{
										iVoxel__ = RVL3DARRAY_INDEX(volume, i__, j__, k__);

										pPtIdx = volume.Element[iVoxel__].PtList.pFirst;

										while (pPtIdx)
										{
											iFeature = pSurfels->surfelMap[pPtIdx->Idx];

											if (iFeature >= 0)
											{
												pFeature = pSurfels->NodeArray.Element + iFeature;

												if (pFeature->size > 1)
												{
													SDF = RVLDOTPRODUCT3(pFeature->N, P) - pFeature->d;

													eSDF = pSample->SDF - SDF;

													eSDF = RVLABS(eSDF);

													if (eSDF < mineSDF)
													{
														mineSDF = eSDF;

														pSample->iFeature = iFeature;
													}
												}
											}

											pPtIdx = pPtIdx->pNext;
										}
									}

							sampleArray.n++;
						}	// if (iClosestPt >= 0)
					}	// if (voxelDistance == sampleVoxelDistance)
				}	// if (pVoxel->voxelDistance > voxelDistance)
			}	// if (i_ >= 0 && i_ < volume.a && j_ >= 0 && j_ < volume.b && k_ >= 0 && k_ < volume.c)
		}	// for (l = 0; l < 6; l++)
	}	// while (pPut > pFetch)

	delete[] PtMem;
	delete[] RGBuff;
	delete[] zeroDistanceVoxelArray.Element;
}

void RVL::DisplaySampledMesh(
	Visualizer *pVisualizer,
	Array3D<RECOG::VN_::Voxel> volume,
	float *P0,
	float voxelSize)
{
	Array3D<float> f;

	f.a = volume.a;
	f.b = volume.b;
	f.c = volume.c;

	int nVoxels = volume.a * volume.b * volume.c;

	f.Element = new float[nVoxels];

	int iVoxel;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
		f.Element[iVoxel] = (volume.Element[iVoxel].voxelDistance > 0 ? 1.0f : -1.0f);

	vtkSmartPointer<vtkPolyData> polyData = DisplayIsoSurface(f, P0, voxelSize, 0.0f);

	// Create a mapper and actor.
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	pVisualizer->renderer->AddActor(actor);

	delete[] f.Element;
}
