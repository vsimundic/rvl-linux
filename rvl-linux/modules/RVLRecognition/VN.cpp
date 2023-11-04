// #include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkLine.h>
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "ReconstructionEval.h"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "ObjectDetector.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif 
#include "NeighborhoodTool.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"

#define RVLVN_MATCH_DEBUG

using namespace RVL;
using namespace RECOG;

VN::VN()
{
	NodeArray.n = 0;
	NodeArray.Element = NULL;
	featureArray.Element = NULL;
	projectionIntervals.Element = NULL;
	projectionIntervalMem = NULL;
	projectionIntervals.Element = NULL;
	projectionIntervalMem = NULL;
	projectionIntervalBuff.Element = NULL;
	dc = NULL;
	parallelDescriptorComponentArray.Element = NULL;
	parallelDescriptorComponentIdxMem = NULL;
	fpDebug = NULL;
	localConstraints = NULL;
	localConstraintsMem = NULL;

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
	RVL_DELETE_ARRAY(parallelDescriptorComponentArray.Element);
	RVL_DELETE_ARRAY(parallelDescriptorComponentIdxMem);
	RVL_DELETE_ARRAY(localConstraints);
	RVL_DELETE_ARRAY(localConstraintsMem);
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

VN_::ModelCluster *VN::AddModelCluster(
	int ID,
	BYTE type,
	float *R,
	float *t,
	float r,
	Array<float> alphaArray,
	Array<float> betaArray,
	Array2D<float> NArray,
	CRVLMem *pMem,
	float rT,
	int *iN)
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
	pMCluster->NArray = NArray;
	pMCluster->iN = iN;

	return pMCluster;
}

VN_::ModelCluster *VN::AddModelCluster(
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

	Array2D<float> NArray;

	NArray.h = 0;

	return AddModelCluster(ID, type, R, t, r, alphaArray, betaArray, NArray, pMem, rT);
}

VN_::ModelCluster *VN::AddModelCluster(
	int ID,
	BYTE type,
	float *R,
	float *t,
	float r,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	Pair<float, float> betaInterval,
	Array2D<float> NArrayIn,
	CRVLMem *pMem)
{
	float mincs = cos(betaInterval.a) + 1e-3;
	float maxcs = cos(betaInterval.b) - 1e-3;

	Array2D<float> NArray;
	NArray.w = 3;
	NArray.h = convexTemplate.n + NArrayIn.h;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, NArray.w *NArray.h, NArray.Element);

	int *iN;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, NArray.h, iN);

	NArray.h = 0;

	int i;
	float *NSrc, *NTgt;

	for (i = 0; i < convexTemplate.n; i++)
	{
		NSrc = convexTemplate.Element[i].N;

		if (NSrc[2] <= mincs && NSrc[2] >= maxcs)
		{
			NTgt = NArray.Element + NArray.w * NArray.h;

			RVLCOPY3VECTOR(NSrc, NTgt);

			iN[NArray.h] = i;

			NArray.h++;
		}
	}

	for (i = 0; i < NArrayIn.h; i++)
	{
		NSrc = NArrayIn.Element + NArrayIn.w * i;

		NTgt = NArray.Element + NArray.w * NArray.h;

		RVLCOPY3VECTOR(NSrc, NTgt);

		iN[NArray.h] = i;

		NArray.h++;
	}

	Array<float> alphaArray;

	alphaArray.n = 0;

	Array<float> betaArray;

	betaArray.n = 0;

	float rT = 0.0f;

	return AddModelCluster(ID, type, R, t, r, alphaArray, betaArray, NArray, pMem, rT, iN);
}

VN_::ModelCluster *VN::AddModelCluster(
	int ID,
	BYTE type,
	float *R,
	float *t,
	float r,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	float *axis,
	Pair<float, float> betaInterval,
	Array2D<float> NArrayIn,
	CRVLMem *pMem)
{
	float mincs = cos(betaInterval.a) + 1e-3;
	float maxcs = cos(betaInterval.b) - 1e-3;

	Array2D<float> NArray;
	NArray.w = 3;
	NArray.h = convexTemplate.n + NArrayIn.h;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, NArray.w *NArray.h, NArray.Element);

	NArray.h = 0;

	int i;
	float *NSrc, *NTgt;
	float cs;

	for (i = 0; i < convexTemplate.n; i++)
	{
		NSrc = convexTemplate.Element[i].N;

		cs = RVLDOTPRODUCT3(axis, NSrc);

		if (cs <= mincs && cs >= maxcs)
		{
			NTgt = NArray.Element + NArray.w * NArray.h;

			RVLCOPY3VECTOR(NSrc, NTgt);

			NArray.h++;
		}
	}

	for (i = 0; i < NArrayIn.h; i++)
	{
		NSrc = NArrayIn.Element + NArrayIn.w * i;

		NTgt = NArray.Element + NArray.w * NArray.h;

		RVLCOPY3VECTOR(NSrc, NTgt);

		NArray.h++;
	}

	Array<float> alphaArray;

	alphaArray.n = 0;

	Array<float> betaArray;

	betaArray.n = 0;

	float rT = 0.0f;

	return AddModelCluster(ID, type, R, t, r, alphaArray, betaArray, NArray, pMem, rT);
}

VN_::ModelCluster *VN::AddModelCluster(
	VN_::ModelCluster *pMCluster,
	int *valid,
	float *R,
	float *t,
	CRVLMem *pMem)
{
	Array2D<float> NArray;
	NArray.w = 3;
	NArray.h = 0;

	int *iN = NULL;

	Array<float> alphaArray;
	alphaArray.n = 0;

	Array<float> betaArray;
	betaArray.n = 0;

	if (pMCluster->NArray.h > 0)
	{
		int nFeatures = pMCluster->NArray.h;

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, NArray.w *nFeatures, NArray.Element);

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nFeatures, iN);

		float *R_ = pMCluster->R;

		int i;
		float *NSrc, *NTgt;

		for (i = 0; i < pMCluster->NArray.h; i++)
		{
			NSrc = pMCluster->NArray.Element + NArray.w * i;

			if (valid[i] == 1)
			{
				NTgt = NArray.Element + NArray.w * NArray.h;

				RVLCOPY3VECTOR(NSrc, NTgt);

				iN[NArray.h] = i;

				NArray.h++;
			}
		}
	}
	else
	{
		bool bValid = false;

		int i = 0;

		int iAlpha, iBeta, iminAlpha, imaxAlpha, iminBeta, imaxBeta;

		for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
			for (iAlpha = 0; iAlpha < pMCluster->alphaArray.n; iAlpha++)
			{
				if (valid[i] == 1)
				{
					if (bValid)
					{
						if (iAlpha < iminAlpha)
							iminAlpha = iAlpha;

						if (iAlpha > imaxAlpha)
							imaxAlpha = iAlpha;

						if (iBeta > imaxBeta)
							imaxBeta = iBeta;
					}
					else
					{
						bValid = true;

						iminAlpha = imaxAlpha = iAlpha;
						iminBeta = imaxBeta = iBeta;
					}
				}

				i++;
			}

		alphaArray.n = imaxAlpha - iminAlpha + 1;

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, alphaArray.n, alphaArray.Element);

		alphaArray.n = 0;

		for (iAlpha = iminAlpha; iAlpha <= imaxAlpha; iAlpha++)
			alphaArray.Element[alphaArray.n++] = pMCluster->alphaArray.Element[iAlpha];

		betaArray.n = imaxBeta - iminBeta + 1;

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, betaArray.n, betaArray.Element);

		betaArray.n = 0;

		for (iBeta = iminBeta; iBeta <= imaxBeta; iBeta++)
			betaArray.Element[betaArray.n++] = pMCluster->betaArray.Element[iBeta];

		// nFeatures = 0;

		// for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
		//	nFeatures += (pMCluster->betaArray.Element[iBeta] < 0.001f || pMCluster->betaArray.Element[iBeta] > PI - 0.001f ?
		//	1 : pMCluster->alphaArray.n);

		// int i = 0;

		// for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
		//{
		//	if (pMCluster->betaArray.Element[iBeta] < 0.001f)
		//	{
		//		if (valid[i] == 1)
		//		{
		//			NTgt = NArray.Element + NArray.w * NArray.h;

		//			RVLCOPYCOLMX3X3(R_, 2, NTgt);

		//			iN[NArray.h++] = i;
		//		}

		//		i++;
		//	}
		//	else if (pMCluster->betaArray.Element[iBeta] > PI - 0.001f)
		//	{
		//		if (valid[i] == 1)
		//		{
		//			NTgt = NArray.Element + NArray.w * NArray.h;

		//			RVLCOPYCOLMX3X3(R_, 2, NTgt);

		//			RVLNEGVECT3(NTgt, NTgt);

		//			iN[NArray.h++] = i;
		//		}

		//		i++;
		//	}
		//	else
		//	{
		//		cb = cos(pMCluster->betaArray.Element[iBeta]);
		//		sb = sin(pMCluster->betaArray.Element[iBeta]);

		//		for (iAlpha = 0; iAlpha < pMCluster->alphaArray.n; iAlpha++)
		//		{
		//			if (valid[i] == 1)
		//			{
		//				ca = cos(pMCluster->alphaArray.Element[iAlpha]);
		//				sa = sin(pMCluster->alphaArray.Element[iAlpha]);

		//				N[0] = ca * sb;
		//				N[1] = sa * sb;
		//				N[2] = cb;

		//				NTgt = NArray.Element + NArray.w * NArray.h;

		//				RVLMULMX3X3VECT(R_, N, NTgt);

		//				iN[NArray.h++] = i;
		//			}

		//			i++;
		//		}
		//	}
		//}	// for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
	} // if (pMCluster->NArray.h == 0)

	float rT = 0.0f;

	return AddModelCluster(pMCluster->ID, pMCluster->type, R, t, 0.0f, alphaArray, betaArray, NArray, pMem, rT, iN);
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
	int iN,
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
	pLimit->iN = iN;
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

	int nConvexClusters = 0;
	int maxnCovexClusterFeatures = 0;

	VN_::ModelCluster *pMCluster = modelClusterList.pFirst;

	int nFeatures;

	while (pMCluster)
	{
		if (pMCluster->NArray.h > 0)
			nFeatures = pMCluster->NArray.h;
		else
		{
			nFeatures = 0;

			for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
				nFeatures += (pMCluster->betaArray.Element[iBeta] < 0.001f || pMCluster->betaArray.Element[iBeta] > PI - 0.001f ? 1 : pMCluster->alphaArray.n);
		}

		featureArray.n += nFeatures;

		if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX)
		{
			nConvexClusters++;

			if (nFeatures > maxnCovexClusterFeatures)
				maxnCovexClusterFeatures = nFeatures;
		}

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

	int iAlpha, iN;
	float N[3];
	float ca, sa, cb, sb;
	float *t;
	VN_::Node *pNode;
	int iFeature;
	VN_::Feature *pFeature_;
	float alpha, beta;
	float P[3], P_[3];
	float a;
	float fOperation;
	float *N_;

	pMCluster = modelClusterList.pFirst;

	while (pMCluster)
	{
		fOperation = (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX ? 1.0f : -1.0f);

		float *R = pMCluster->R;

		pMCluster->iFeatureInterval.a = pFeature - featureArray.Element;

		if (pMCluster->NArray.h > 0)
		{
			for (iN = 0; iN < pMCluster->NArray.h; iN++, pFeature++)
			{
				N_ = pMCluster->NArray.Element + pMCluster->NArray.w * iN;

				RVLMULMX3X3VECT(R, N_, pFeature->N);

				pFeature->iAlpha = -1;
				pFeature->iBeta = -1;
				pFeature->iN = iN;
			}
		}
		else
		{
			pMCluster->NArray.w = 3;
			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pMCluster->alphaArray.n * pMCluster->betaArray.n * pMCluster->NArray.w, pMCluster->NArray.Element);

			for (iBeta = 0; iBeta < pMCluster->betaArray.n; iBeta++)
			{
				if (pMCluster->betaArray.Element[iBeta] < 0.001f)
				{
					RVLCOPYCOLMX3X3(R, 2, pFeature->N);

					pFeature->iAlpha = 0;
					pFeature->iBeta = iBeta;
					pFeature->iN = -1;

					pFeature++;

					N_ = pMCluster->NArray.Element + pMCluster->NArray.w * pMCluster->NArray.h;
					RVLSET3VECTOR(N_, 0.0f, 0.0f, 1.0f);
					pMCluster->NArray.h++;
				}
				else if (pMCluster->betaArray.Element[iBeta] > PI - 0.001f)
				{
					RVLCOPYCOLMX3X3(R, 2, N);

					RVLNEGVECT3(N, pFeature->N);

					pFeature->iAlpha = 0;
					pFeature->iBeta = iBeta;
					pFeature->iN = -1;

					pFeature++;

					N_ = pMCluster->NArray.Element + pMCluster->NArray.w * pMCluster->NArray.h;
					RVLSET3VECTOR(N_, 0.0f, 0.0f, -1.0f);
					pMCluster->NArray.h++;
				}
				else
				{
					cb = cos(pMCluster->betaArray.Element[iBeta]);
					sb = sin(pMCluster->betaArray.Element[iBeta]);

					for (iAlpha = 0; iAlpha < pMCluster->alphaArray.n; iAlpha++, pFeature++, pMCluster->NArray.h++)
					{
						ca = cos(pMCluster->alphaArray.Element[iAlpha]);
						sa = sin(pMCluster->alphaArray.Element[iAlpha]);

						N[0] = ca * sb;
						N[1] = sa * sb;
						N[2] = cb;

						RVLMULMX3X3VECT(R, N, pFeature->N);

						pFeature->iAlpha = iAlpha;
						pFeature->iBeta = iBeta;
						pFeature->iN = -1;

						N_ = pMCluster->NArray.Element + pMCluster->NArray.w * pMCluster->NArray.h;
						RVLCOPY3VECTOR(N, N_);
					}
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
				if (pFeature_->iAlpha >= 0)
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
				else
					pFeature_->d = 0.0f;
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

			if (pMCluster->NArray.h > 0)
			{
				for (iN = 0; iN < pMCluster->NArray.h; iN++)
				{
					RVLMEM_ALLOC_STRUCT(pMem, VN_::Edge, pEdge);

					RVLQLIST_ADD_ENTRY(pEdgeList, pEdge);

					pEdge->data.a = iFeature;
					pEdge->data.b = iNode;
					pEdge->bPrimary = true;

					iFeature++;
				}
			}
		}

		if (bTorus || pMCluster->NArray.h == 0)
		{
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

				if (pFeature->iAlpha == pLimit->iAlpha && pFeature->iBeta == pLimit->iBeta || pFeature->iN == pLimit->iN)
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

			if (pMCluster = GetModelCluster(pOperation->operand[iOperand]))
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

	projectionIntervals.Element = new Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>[projectionIntervals.n];

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

	projectionIntervalMem = new Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>[maxnIntervals];

	Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection> *pInterval = projectionIntervalMem;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		projectionIntervals.Element[iNode].Element = pInterval;

		pInterval += projectionIntervals.Element[iNode].n;
	}

	RVL_DELETE_ARRAY(projectionIntervalBuff.Element);

	projectionIntervalBuff.Element = new Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>[featureArray.n];

	RVL_DELETE_ARRAY(dc);

	dc = new float[featureArray.n];

	// Parallel descriptor components.

	parallelDescriptorComponentArray.n = 0;

	if (nConvexClusters >= 1)
	{
		RVL_DELETE_ARRAY(parallelDescriptorComponentIdxMem);

		parallelDescriptorComponentIdxMem = new int[nConvexClusters * maxnCovexClusterFeatures];

		RVL_DELETE_ARRAY(parallelDescriptorComponentArray.Element);

		parallelDescriptorComponentArray.Element = new VN_::ParallelDescriptorComponent[maxnCovexClusterFeatures];

		pMCluster = modelClusterList.pFirst;

		while (pMCluster)
		{
			if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX)
				break;

			pMCluster = pMCluster->pNext;
		}

		int *piParallelComponent = parallelDescriptorComponentIdxMem;

		parallelDescriptorComponentArray.Element[0].idxArray.n = nConvexClusters;

		parallelDescriptorComponentArray.Element[0].idxArray.Element = piParallelComponent;

		float *N__;
		int iFeature_;
		float e;
		int iParallelDescriptorComponent;

		pEdge = EdgeList.pFirst;

		while (pEdge)
		{
			if (pEdge->data.b == pMCluster->iNode)
			{
				iFeature = pEdge->data.a;

				N_ = featureArray.Element[iFeature].N;

				iParallelDescriptorComponent = 0;

				parallelDescriptorComponentArray.Element[parallelDescriptorComponentArray.n].idxArray.Element[iParallelDescriptorComponent++] = iFeature;

				pMCluster_ = pMCluster->pNext;

				while (pMCluster_)
				{
					if (pMCluster_->type == RVLVN_CLUSTER_TYPE_CONVEX)
					{
						pEdge_ = EdgeList.pFirst;

						while (pEdge_)
						{
							if (pEdge_->data.b == pMCluster_->iNode)
							{
								iFeature_ = pEdge_->data.a;

								N__ = featureArray.Element[iFeature_].N;

								e = 1.0f - RVLDOTPRODUCT3(N_, N__);

								if (e <= 1e-6)
								{
									parallelDescriptorComponentArray.Element[parallelDescriptorComponentArray.n].idxArray.Element[iParallelDescriptorComponent++] = iFeature_;

									break;
								}
							}

							pEdge_ = pEdge_->pNext;
						}

						if (pEdge_ == NULL)
							break;
					} // if (pMCluster_->type == RVLVN_CLUSTER_TYPE_CONVEX)

					pMCluster_ = pMCluster_->pNext;
				} // while (pMCluster_)

				if (pMCluster_ == NULL)
				{
					N__ = parallelDescriptorComponentArray.Element[parallelDescriptorComponentArray.n].N;

					RVLCOPY3VECTOR(N_, N__);

					parallelDescriptorComponentArray.n++;

					if (parallelDescriptorComponentArray.n < maxnCovexClusterFeatures)
					{
						piParallelComponent += nConvexClusters;

						parallelDescriptorComponentArray.Element[parallelDescriptorComponentArray.n].idxArray.n = nConvexClusters;

						parallelDescriptorComponentArray.Element[parallelDescriptorComponentArray.n].idxArray.Element = piParallelComponent;
					}
					else
						break;
				}
			} // if (pEdge->data.b == pMCluster->iNode)

			pEdge = pEdge->pNext;
		} // while (pEdge)
	}	  // if (nConvexClusters >= 1)

	// Allocate memory for local constraints.

	RVL_DELETE_ARRAY(localConstraints);
	localConstraints = new Array<int>[NodeArray.n];
	RVL_DELETE_ARRAY(localConstraintsMem);
	localConstraintsMem = new int[NodeArray.n * featureArray.n];
	for (iNode = 0; iNode < NodeArray.n; iNode++)
		localConstraints[iNode].Element = localConstraintsMem + iNode * featureArray.n;
}

VN_::ModelCluster *VN::GetModelCluster(int ID)
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

VN_::Operation *VN::GetOperation(int ID)
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

	Array<MESH::Sample> sampleArray;
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

	MESH::Sample *pSample = sampleArray.Element + iSample;

	Array<int> cluster;

	cluster.Element = new int[featureArray.n];

	bool *bInCluster = new bool[featureArray.n];

	memset(bInCluster, 0, featureArray.n * sizeof(bool));

	iy = pSample->iFeature;

	int operation;
	float fOperation;
	bool bAllSamplesPreserved;
	int iSample_;
	MESH::Sample *pSample_;
	int iBestCandidate;
	float eSDF_, eSDFBestCandidate;
	int iyPrev;
	RECOG::VN_::Edge *pEdge;
	int iClusterNode;
	int iActiveFeature;
	float SDF, eSDF;

	for (i = 1; i < sortedSampleArray.n; i++)
	{
		// if (i == 3639)
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

						// if (cluster.n > nFeatures)
						//	int debug = 0;
					}
				}
				else
				{
					bAllSamplesPreserved = false;

					break;
				}
			} // if (fOperation * eSDF > eps)
		}	  // for (j = 0; j < i; j++)

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
	} // for (i = 1; i < sortedSampleArray.n; i++)

	printf("completed.\n");

	printf("no. of nodes=%d (no. of features=%d)", NodeArray.n, featureArray.n);

	FILE *fp = fopen("VNError.txt", "w");

	for (i = 0; i < sortedSampleArray.n; i++)
	{
		SDF = Evaluate(NULL, SDF_ + i * featureArray.n, iActiveFeature, false);

		fprintf(fp, "%f\n", SDF);
	}

	fclose(fp);

	// if (pVisualizer)
	//{
	//	DisplaySampledMesh(pVisualizer, volume, P0, voxelSize);

	//	unsigned char color[] = { 0, 128, 255 };

	//	pVisualizer->DisplayPointSet<float, MESH::Sample>(sampleArray, color, 6.0f);
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

void VN::UpdateClusterOrientations()
{
	VN_::ModelCluster *pMCluster = modelClusterList.pFirst;
	int iFeature;
	VN_::Feature *pFeature;
	float *N;
	int i;
	while (pMCluster)
	{
		N = pMCluster->NArray.Element;
		for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++, N += 3)
		{
			pFeature = featureArray.Element + iFeature;
			RVLMULMX3X3VECT(pMCluster->R, N, pFeature->N);
		}
		pMCluster = pMCluster->pNext;
	}
}

void VN::SetFeatureOffsets(float *d)
{
	int iFeature;
	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
		featureArray.Element[iFeature].d = d[iFeature];
}

void VN::GetEdges(Array<VN_::Edge> &edges)
{
	if (edges.Element == NULL)
		edges.Element = new VN_::Edge[NodeArray.n - 1];
	RECOG::VN_::Edge *pEdge = EdgeList.pFirst;
	QLIST::CopyToArray<RECOG::VN_::Edge>(&(EdgeList), &edges);
}

void VN::Descriptor(float *d)
{
	UpdateClusterOrientations();
	int iFeature;
	VN_::Feature *pFeature;
	VN_::ModelCluster *pMCluster = modelClusterList.pFirst;
	while (pMCluster)
	{
		for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
		{
			pFeature = featureArray.Element + iFeature;
			d[iFeature] = pFeature->d + RVLDOTPRODUCT3(pFeature->N, pMCluster->t);
		}
		pMCluster = pMCluster->pNext;
	}
}

void VN::Descriptor(
	Array<Vector3<float>> points,
	Array<VN_::Correspondence5> assoc,
	float *d)
{
	// int nMClusters = 0;
	// VN_::ModelCluster* pMCluster = modelClusterList.pFirst;
	// while (pMCluster)
	//{
	//	nMClusters++;
	//	pMCluster = pMCluster->pNext;
	// }
	bool *bd = new bool[featureArray.n];
	memset(bd, 0, featureArray.n * sizeof(bool));
	int iAssoc;
	VN_::Correspondence5 *pAssoc;
	VN_::ModelCluster *pMCluster;
	int iFeature;
	VN_::Feature *pFeature;
	int iFirstFeature, iLastFeature;
	float d_;
	float *PS;
	for (iAssoc = 0; iAssoc < assoc.n; iAssoc++)
	{
		if (iAssoc == 24)
			int debug = 0;
		pAssoc = assoc.Element + iAssoc;
		pMCluster = modelClusterList.pFirst;
		while (pMCluster)
		{
			if (pMCluster->ID == pAssoc->iMCluster)
				break;
			pMCluster = pMCluster->pNext;
		}
		if (pMCluster == NULL)
			continue;
		if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pMCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
		{
			iFirstFeature = pMCluster->iFeatureInterval.a;
			iLastFeature = pMCluster->iFeatureInterval.b;
		}
		else if (pMCluster->type == RVLVN_CLUSTER_TYPE_XTORUS || pMCluster->type == RVLVN_CLUSTER_TYPE_ITORUS)
		{
			iFirstFeature = pMCluster->iFeatureInterval.a + pAssoc->iBeta * pMCluster->alphaArray.n;
			iLastFeature = iFirstFeature + pMCluster->alphaArray.n - 1;
		}
		PS = points.Element[pAssoc->iSPoint].Element;
		for (iFeature = iFirstFeature; iFeature <= iLastFeature; iFeature++)
		{
			pFeature = featureArray.Element + iFeature;
			d_ = RVLDOTPRODUCT3(pFeature->N, PS);
			if (bd[iFeature])
			{
				if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pMCluster->type == RVLVN_CLUSTER_TYPE_ITORUS)
				{
					if (d_ > d[iFeature])
						d[iFeature] = d_;
				}
				else if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE || pMCluster->type == RVLVN_CLUSTER_TYPE_XTORUS)
				{
					if (d_ < d[iFeature])
						d[iFeature] = d_;
				}
			}
			else
			{
				d[iFeature] = d_;
				bd[iFeature] = true;
			}
		}
	}

	delete[] bd;
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
	bool *bd)
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

			do
			{
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
	} // for every feature node

#ifdef RVLVN_MATCH_DEBUG
	fclose(fp);
#endif

	delete[] nodeCorrespArray.Element;
	delete[] bOwnedByRefVertex;

	///

	// float maxMatchCost = kMaxMatchCost * size;

	// MatchByBuildingInterpretationTree(pSurfels, correspondenceArray, maxMatchCost, minMatchCostDiff, maxnGITNodes, dS, bdS, pMem2);

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
	VN_::ITNode *&pITNodeOut)
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

		// pQueueBin->pFirst = pITNode->pNext;

		// if (pQueueBin->pFirst == NULL)
		//	pQueueBin->ppNext = &(pQueueBin->pFirst);

		iLevel = pITNode->iLevel + 1;

		// printf("%d\n", Q.iTopQueueBin);

		// if (iLevel == 66)
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

				// if (pCluster->iParent = clusters.at(0).iParent)
				//{
				//	if (iNode >= 65 && iNode <= 80)
				//		if (pCorrespondenceArray->Element[i].d > 0.1f)
				//			continue;
				// }

				// if (iNode == 65)
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
			} // for each node instance
		}	  // if(!pITNode->bExpanded)

#ifdef RVLVN_MATCH_DEBUG
		fprintf(fp, "\n");
#endif
	} // main loop

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

	// for (i = 0; i < 2; i++)
	for (iCluster = 0; iCluster < nClusters; iCluster++)
	{
		// iCluster = (i == 0 ? iLargestCluster : 0);

		cluster = clusters.at(iCluster);

		// Q[iCluster].maxMatchCost = size * (float)(pSurfels->vertexArray.n);
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
		// int maxDebugCounter = 0;

		while (maxe > maxMatchCost)
		{
			// Only for debugging purpose!

			// int debugCounter = 0;

			// VN_::ITNode *pDebugITNode = pGITNode->ITPtr_[0].pLITNode;

			// while (pDebugITNode)
			//{
			//	if (pDebugITNode->iNode >= 65 && pDebugITNode->iNode <= 80)
			//	{
			//		if (correspondenceArray.Element[pDebugITNode->iNode].Element[pDebugITNode->iCorrespondence].d < 0.1f)
			//			debugCounter++;

			//		printf("%5.3f ", correspondenceArray.Element[pDebugITNode->iNode].Element[pDebugITNode->iCorrespondence].d);
			//	}

			//	pDebugITNode = pDebugITNode->pParent;
			//}

			// printf("\n");

			// if (debugCounter > maxDebugCounter)
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

				RVLVN_ADD_QUEUE_ENTRY(pGITNode_, VN_::GITNode, pGITNode_->e, GQueue, minMatchCostDiff, nMatchCostLevels, iTopGQueueBin, iBottomGQueueBin, pMem2, iBin, pQueueBin)
			}

			RVLVN_GET_NEXT_QUEUE_ENTRY(GQueue, pGITNode, pGITNode, iTopGQueueBin, iBottomGQueueBin, bCompleted, pQueueBin);

			Descriptor(pGITNode->ITPtr_, correspondenceArray, dS, bdS);

			maxe = Distance(pSurfels, iCriticalVertexArray, dS, bdS, iMaxErrVertex, SDF);

			if (minmaxe_ < 0.0f || maxe < minmaxe_)
			{
				minmaxe_ = maxe;

				pGITNodeSuboptimal = pGITNode;

				// printf("e=%f\n", minmaxe_);
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

int VN::ClusterTypes(
	bool &bConcavity,
	bool &bTorus)
{
	bTorus = false;
	bConcavity = false;

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

	return nMClusters;
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

	Array<float *> PArray;

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

		// if (iGeneration % 10 == 9)
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

			do
				RVLRND(nSelection, iRnd, nRnd, iiRnd, iParent2)
				while (iParent2 == iParent1);

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

float VN::Evaluate(
	Array<float *> PArray,
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

		// if (e_ < 0.0f)
		//	e_ = -e_;
		// e += e_;

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

		// if (e_ < 0.0f)
		//	e_ = -e_;
		// e += e_;

		// maximum absolute distance

		// if (e_ < 0.0f)
		//	e_ = -e_;
		// if (e_ > e)
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

		// if (e_ < 0.0f)
		//	e_ = -e_;
		// e += e_;

		// maximum absolute distance

		// if (e_ < 0.0f)
		//	e_ = -e_;
		// if (e_ > e)
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
	Array<MESH::Sample> sampleArray,
	float *SDF,
	float *d,
	bool *bd,
	float maxe)
{
	float e = 0.0f;

	int iSample;
	int iActiveFeature;
	float e_;
	MESH::Sample *pSample;

	for (iSample = 0; iSample < sampleArray.n; iSample++)
	{
		pSample = sampleArray.Element + iSample;

		e_ = Evaluate(pSample->P, SDF, iActiveFeature, true, d, bd) - pSample->SDF;

		// sum of absolute distances

		// if (e_ < 0.0f)
		//	e_ = -e_;
		// e += e_;

		// maximum absolute distance

		// if (e_ < 0.0f)
		//	e_ = -e_;
		// if (e_ > e)
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

float VN::LocalConstraints(
	float *P,
	float *SDF,
	int &iActiveFeature,
	bool bComputeSDFs,
	float *d)
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
		localConstraints[iNode].n = 1;
		localConstraints[iNode].Element[0] = iNode;
		pNode->bOutput = true;
	}

	for (; iNode < NodeArray.n; iNode++)
	{
		NodeArray.Element[iNode].bOutput = false;
		localConstraints[iNode].n = 0;
	}

	RECOG::VN_::Edge *pEdge = EdgeList.pFirst;

	VN_::Node *pChildNode, *pParentNode;

	int iParentNode, iChildNode, iConstraint;
	Array<int> *pChildNodeLocalConstraints, *pParentNodeLocalConstraints;
	while (pEdge)
	{
		iChildNode = pEdge->data.a;
		pChildNode = NodeArray.Element + iChildNode;

		if (pChildNode->bOutput)
		{
			iParentNode = pEdge->data.b;
			pParentNode = NodeArray.Element + iParentNode;
			pChildNodeLocalConstraints = localConstraints + iChildNode;
			pParentNodeLocalConstraints = localConstraints + iParentNode;

			if (pParentNode->bOutput)
			{
				if (pParentNode->operation < 0)
				{
					if (pChildNode->output < pParentNode->output)
					{
						pParentNode->output = pChildNode->output;
						pParentNode->iActiveFeature = pChildNode->iActiveFeature;
					}
					if (pChildNodeLocalConstraints->n > 0)
						for (iConstraint = 0; iConstraint < pChildNodeLocalConstraints->n; iConstraint++)
							pParentNodeLocalConstraints->Element[pParentNodeLocalConstraints->n++] = pChildNodeLocalConstraints->Element[iConstraint];
				}
				else
				{
					if (pChildNode->output > pParentNode->output)
					{
						pParentNode->output = pChildNode->output;
						pParentNode->iActiveFeature = pChildNode->iActiveFeature;
						pParentNodeLocalConstraints->n = 0;
						if (pChildNodeLocalConstraints->n > 0)
							for (iConstraint = 0; iConstraint < pChildNodeLocalConstraints->n; iConstraint++)
								pParentNodeLocalConstraints->Element[pParentNodeLocalConstraints->n++] = pChildNodeLocalConstraints->Element[iConstraint];
					}
				}
			}
			else
			{
				pParentNode->output = pChildNode->output;
				pParentNode->iActiveFeature = pChildNode->iActiveFeature;
				pParentNode->bOutput = true;
				if (pChildNodeLocalConstraints->n > 0)
					for (iConstraint = 0; iConstraint < pChildNodeLocalConstraints->n; iConstraint++)
						pParentNodeLocalConstraints->Element[pParentNodeLocalConstraints->n++] = pChildNodeLocalConstraints->Element[iConstraint];
			}
		}

		pEdge = pEdge->pNext;
	}

	iActiveFeature = NodeArray.Element[iy].iActiveFeature;

	return NodeArray.Element[iy].output;
}

// The method implemened by function Fit is described in ARP3D.TR4.11, Section "Classification Criterion".

void VN::Fit(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	QList<QLIST::Index> surfelList,
	Array<int> iVertexArray,
	Camera camera,
	Array2D<float> M,
	float *R,
	float *d0,
	float *q,
	bool bVisualize)
{
	// Constants

	int maxnIterations = 0;
	float gamma = 0.2f;
	float kMinAbsJq = 0.1f;
	int imageNeighborhood = 7;

	// Determine image ROI containing the object.

	Rect<float> ROI;

	pSurfels->GetDepthImageROI(iVertexArray, camera, ROI);

	// Sample points in the ROI.

	Array2D<float> imagePtArray;

	Rect<float> cameraWin;

	cameraWin.minx = 0.0f;
	cameraWin.maxx = (float)(camera.w - 1);
	cameraWin.miny = 0.0f;
	cameraWin.maxy = (float)(camera.h - 1);

	SampleRect<float>(&ROI, 10.0f, cameraWin, 32, imagePtArray);

	// PtArray <- all sample points close to the object surface

	int halfImageNeighborhood = (imageNeighborhood - 1) / 2;

	Rect<int> cropWin;
	cropWin.minx = 0;
	cropWin.maxx = camera.w - 1;
	cropWin.miny = 0;
	cropWin.maxy = camera.h - 1;

	bool *bBelongsToObject = new bool[pSurfels->NodeArray.n];

	memset(bBelongsToObject, 0, pSurfels->NodeArray.n * sizeof(bool));

	QLIST::Index *pSurfelIdx = surfelList.pFirst;

	while (pSurfelIdx)
	{
		bBelongsToObject[pSurfelIdx->Idx] = true;

		pSurfelIdx = pSurfelIdx->pNext;
	}

	Array<float *> PtArray;

	PtArray.Element = new float *[imagePtArray.h];

	PtArray.n = 0;

	float *g = new float[imagePtArray.h];

	float *PExternal_ = new float[3 * imagePtArray.h];

	float *PExternal = PExternal_;

	int i;
	float *P;
	float *m;
	int u, v;
	int iPt;
	int iSurfel;
	int u_, v_;
	Rect<int> neighborhood;
	float Ray[3], dP[3], P_[3];
	float s, fTmp, sClosest;
	float dist, minDist;
	bool bFirst;

	for (i = 0; i < imagePtArray.h; i++)
	{
		m = imagePtArray.Element + imagePtArray.w * i;

		u = (int)round(m[0]);
		v = (int)round(m[1]);

		iPt = u + v * pMesh->width;

		iSurfel = pSurfels->surfelMap[iPt];

		if (iSurfel < 0 || iSurfel >= pSurfels->NodeArray.n)
			continue;

		if (bBelongsToObject[iSurfel])
		{
			g[PtArray.n] = 0.0f;

			PtArray.Element[PtArray.n++] = pMesh->NodeArray.Element[iPt].P;
		}
		else
		{
			RVLSET3VECTOR(Ray, ((m[0] - camera.uc) / camera.fu), ((m[1] - camera.vc) / camera.fv), 1.0f);

			fTmp = RVLDOTPRODUCT3(Ray, Ray);

			neighborhood.minx = u - halfImageNeighborhood;
			neighborhood.maxx = u + halfImageNeighborhood;
			neighborhood.miny = v - halfImageNeighborhood;
			neighborhood.maxy = v + halfImageNeighborhood;

			CropRect<int>(neighborhood, cropWin);

			bFirst = true;

			for (v_ = neighborhood.miny; v_ <= neighborhood.maxy; v_++)
				for (u_ = neighborhood.minx; u_ <= neighborhood.maxx; u_++)
				{
					iPt = u_ + v_ * pMesh->width;

					iSurfel = pSurfels->surfelMap[iPt];

					if (iSurfel < 0 || iSurfel >= pSurfels->NodeArray.n)
						continue;

					if (!bBelongsToObject[iSurfel])
						continue;

					P = pMesh->NodeArray.Element[iPt].P;

					s = RVLDOTPRODUCT3(Ray, P) / fTmp;

					RVLSCALE3VECTOR(Ray, s, P_);

					RVLDIF3VECTORS(P, P_, dP);

					dist = RVLDOTPRODUCT3(dP, dP);

					if (bFirst)
					{
						minDist = dist;

						sClosest = s;

						bFirst = false;
					}
					else if (dist < minDist)
					{
						minDist = dist;

						sClosest = s;
					}
				}

			if (!bFirst)
			{
				RVLSCALE3VECTOR(Ray, sClosest, PExternal);

				g[PtArray.n] = sqrt(minDist);

				PtArray.Element[PtArray.n++] = PExternal;

				PExternal += 3;
			}
		}
	}

	// q <- projection of d0 to the latent space defined by M

	int j;
	float *a;

	RVLMULMXVECT(M.Element, d0, M.h, M.w, q, i, j, a);

	// Initialize visualization.

	Visualizer *pVisualizer;
	bool *bd;

	if (bVisualize)
	{
		pVisualizer = new Visualizer;

		pVisualizer->Create();

		bd = new bool[featureArray.n];

		for (i = 0; i < featureArray.n; i++)
			bd[i] = true;
	}

	/// Minimization of the cost function.

	float minAbsJq2 = kMinAbsJq * kMinAbsJq * (float)(PtArray.n);

	float *d = new float[featureArray.n];

	float *NS_ = new float[3 * featureArray.n];

	int iFeature;
	float *NM, *NS;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		NM = featureArray.Element[iFeature].N;
		NS = NS_ + 3 * iFeature;

		RVLMULMX3X3VECT(R, NM, NS);
	}

	float *SDF0 = new float[featureArray.n * PtArray.n];

	for (i = 0; i < PtArray.n; i++)
	{
		P = PtArray.Element[i];

		for (iFeature = 0; iFeature < featureArray.n; iFeature++)
		{
			NS = NS_ + 3 * iFeature;

			SDF0[featureArray.n * i + iFeature] = RVLDOTPRODUCT3(NS, P);
		}
	}

	float *SDF = new float[featureArray.n];

	float *Jd = new float[featureArray.n];

	float *Jq = new float[M.h];

	float E_ = -1.0f;

	int k;
	float e, E;
	float *SDF0_;
	int iActiveFeature;
	float absJq2;

	for (k = 0; k < maxnIterations; k++)
	{
		// d <- M * q

		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		// Visualization

		// visualizer.renderer->RemoveAllViewProps();

		// Display(&visualizer, 0.01f, d, bd, 0.0f);

		// visualizer.Run();

		// Compute the cost function and Jd = df/dd.

		E = 0.0f;

		memset(Jd, 0, featureArray.n * sizeof(float));

		for (i = 0; i < PtArray.n; i++)
		{
			// e <- f(p; d)

			SDF0_ = SDF0 + featureArray.n * i;

			RVLDIFVECTORS(SDF0_, d, M.w, SDF, j);

			P = PtArray.Element[i];

			e = g[i] - Evaluate(P, SDF, iActiveFeature, false);

			Jd[iActiveFeature] += e;

			E = E + e * e;
		}

		if (E_ > 0.0f && E > E_)
		{
			for (i = 0; i < M.h; i++)
				q[i] += s * Jq[i];

			break;
		}

		E_ = E;

		// Jq = df/dq <- Jd * M

		RVLMULMXVECT(M.Element, Jd, M.h, M.w, Jq, i, j, a);

		// absJq2 <- Jq' * Jq

		RVLDOTPRODUCT(Jq, Jq, M.h, absJq2, i);

		// If the magnitude of gradient Jq is too low, stop the procedure.

		// if (absJq2 < minAbsJq2)
		//	break;

		// s <- gamma * E / absJq2

		s = gamma * E / absJq2;

		// q <- q - s * Jq

		for (i = 0; i < M.h; i++)
			q[i] -= s * Jq[i];
	}

	// Visualization

	if (bVisualize)
	{
		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		pVisualizer->renderer->RemoveAllViewProps();

		Display(pVisualizer, 0.01f, d, bd, 0.0f);

		pVisualizer->Run();

		delete pVisualizer;

		delete[] bd;
	}

	// Free memory.

	delete[] imagePtArray.Element;
	delete[] PtArray.Element;
	delete[] bBelongsToObject;
	delete[] d;
	delete[] SDF;
	delete[] SDF0;
	delete[] NS_;
	// delete[] J;
	delete[] Jd;
	delete[] Jq;
	delete[] g;
	delete[] PExternal_;
}

void VN::FitLM(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	QList<QLIST::Index> surfelList,
	Array<int> iVertexArray,
	Camera camera,
	Array2D<float> M,
	float *R,
	float *d0,
	float *q,
	bool bVisualize)
{
	// Constants

	int maxnIterations = 10;
	float lambdaUp = 2.0f;
	float lambdaDown = 1.0f / lambdaUp;
	// float kMinAbsJq = 0.1f;
	int imageNeighborhood = 7;

	// Determine image ROI containing the object.

	Rect<float> ROI;

	pSurfels->GetDepthImageROI(iVertexArray, camera, ROI);

	// Sample points in the ROI.

	Array2D<float> imagePtArray;

	Rect<float> cameraWin;

	cameraWin.minx = 0.0f;
	cameraWin.maxx = (float)(camera.w - 1);
	cameraWin.miny = 0.0f;
	cameraWin.maxy = (float)(camera.h - 1);

	SampleRect<float>(&ROI, 10.0f, cameraWin, 32, imagePtArray);

	// PtArray <- all sample points close to the object surface

	int halfImageNeighborhood = (imageNeighborhood - 1) / 2;

	Rect<int> cropWin;
	cropWin.minx = 0;
	cropWin.maxx = camera.w - 1;
	cropWin.miny = 0;
	cropWin.maxy = camera.h - 1;

	bool *bBelongsToObject = new bool[pSurfels->NodeArray.n];

	memset(bBelongsToObject, 0, pSurfels->NodeArray.n * sizeof(bool));

	QLIST::Index *pSurfelIdx = surfelList.pFirst;

	while (pSurfelIdx)
	{
		bBelongsToObject[pSurfelIdx->Idx] = true;

		pSurfelIdx = pSurfelIdx->pNext;
	}

	Array<float *> PtArray;

	PtArray.Element = new float *[imagePtArray.h];

	PtArray.n = 0;

	float *g = new float[imagePtArray.h];

	float *PMem = new float[3 * imagePtArray.h];

	float *P__ = PMem;

	int i;
	float *P;
	float *m;
	int u, v;
	int iPt;
	int iSurfel;
	int u_, v_;
	Rect<int> neighborhood;
	float Ray[3], dP[3], P_[3];
	float s, fTmp, sClosest;
	float dist, minDist;
	bool bFirst;

	for (i = 0; i < imagePtArray.h; i++)
	{
		m = imagePtArray.Element + imagePtArray.w * i;

		u = (int)round(m[0]);
		v = (int)round(m[1]);

		iPt = u + v * pMesh->width;

		iSurfel = pSurfels->surfelMap[iPt];

		if (iSurfel < 0 || iSurfel >= pSurfels->NodeArray.n)
			continue;

		if (bBelongsToObject[iSurfel])
		{
			g[PtArray.n] = 0.0f;

			P = pMesh->NodeArray.Element[iPt].P;

			RVLCOPY3VECTOR(P, P__);

			PtArray.Element[PtArray.n++] = P__;

			P__ += 3;
		}
		else
		{
			RVLSET3VECTOR(Ray, ((m[0] - camera.uc) / camera.fu), ((m[1] - camera.vc) / camera.fv), 1.0f);

			fTmp = RVLDOTPRODUCT3(Ray, Ray);

			neighborhood.minx = u - halfImageNeighborhood;
			neighborhood.maxx = u + halfImageNeighborhood;
			neighborhood.miny = v - halfImageNeighborhood;
			neighborhood.maxy = v + halfImageNeighborhood;

			CropRect<int>(neighborhood, cropWin);

			bFirst = true;

			for (v_ = neighborhood.miny; v_ <= neighborhood.maxy; v_++)
				for (u_ = neighborhood.minx; u_ <= neighborhood.maxx; u_++)
				{
					iPt = u_ + v_ * pMesh->width;

					iSurfel = pSurfels->surfelMap[iPt];

					if (iSurfel < 0 || iSurfel >= pSurfels->NodeArray.n)
						continue;

					if (!bBelongsToObject[iSurfel])
						continue;

					P = pMesh->NodeArray.Element[iPt].P;

					s = RVLDOTPRODUCT3(Ray, P) / fTmp;

					RVLSCALE3VECTOR(Ray, s, P_);

					RVLDIF3VECTORS(P, P_, dP);

					dist = RVLDOTPRODUCT3(dP, dP);

					if (bFirst)
					{
						minDist = dist;

						sClosest = s;

						bFirst = false;
					}
					else if (dist < minDist)
					{
						minDist = dist;

						sClosest = s;
					}
				}

			if (!bFirst)
			{
				RVLSCALE3VECTOR(Ray, sClosest, P__);

				g[PtArray.n] = sqrt(minDist);

				PtArray.Element[PtArray.n++] = P__;

				P__ += 3;
			}
		}
	}

	// Compute centroid of scene points.

	float Pc[3];

	RVLNULL3VECTOR(Pc);

	for (i = 0; i < PtArray.n; i++)
	{
		P = PtArray.Element[i];

		RVLSUM3VECTORS(Pc, P, Pc);
	}

	fTmp = (float)(PtArray.n);

	RVLSCALE3VECTOR2(Pc, fTmp, Pc);

	// Center PtArray in Pc.

	for (i = 0; i < PtArray.n; i++)
	{
		P = PtArray.Element[i];

		RVLDIF3VECTORS(P, Pc, P);
	}

	// Transform model normals to the scene RF and center the descriptor in Pc.

	float *d = new float[featureArray.n];

	float *NS_ = new float[3 * featureArray.n];

	int iFeature;
	float *NM, *NS;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		NM = featureArray.Element[iFeature].N;
		NS = NS_ + 3 * iFeature;

		RVLMULMX3X3VECT(R, NM, NS);

		d[iFeature] = d0[iFeature] - RVLDOTPRODUCT3(NS, Pc);
	}

	// q <- M' * d

	int j;
	float *a;

	RVLMULMXVECT(M.Element, d, M.h, M.w, q, i, j, a);

	// Initialize visualization.

	Visualizer *pVisualizer;
	bool *bd;

	if (bVisualize)
	{
		pVisualizer = new Visualizer;

		pVisualizer->Create();

		bd = new bool[featureArray.n];

		for (i = 0; i < featureArray.n; i++)
			bd[i] = true;
	}

	/// Minimization of the cost function.

	float *SDF0 = new float[featureArray.n * PtArray.n];

	for (i = 0; i < PtArray.n; i++)
	{
		P = PtArray.Element[i];

		for (iFeature = 0; iFeature < featureArray.n; iFeature++)
		{
			NS = NS_ + 3 * iFeature;

			SDF0[featureArray.n * i + iFeature] = RVLDOTPRODUCT3(NS, P);
		}
	}

	float *SDF = new float[featureArray.n];

	float *Jd = new float[featureArray.n];

	float *Jq = new float[M.h];

	cv::Mat A;

	A.create(M.h, M.h, CV_32FC1);

	float *A_ = (float *)(A.data);

	cv::Mat b;

	b.create(M.h, 1.0f, CV_32FC1);

	float *b_ = (float *)(b.data);

	cv::Mat dq;

	dq.create(M.h, 1.0f, CV_32FC1);

	float *dq_ = (float *)(dq.data);

	int m2 = M.h * M.h;

	float E_ = -1.0f;

	int k, l;
	float e, E;
	float *SDF0_;
	int iActiveFeature;
	float *M_;
	float lambda;
	float maxTrA;

	for (k = 0; k < maxnIterations; k++)
	{
		// d <- M * q

		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		// Visualization

		// visualizer.renderer->RemoveAllViewProps();

		// Display(&visualizer, 0.01f, d, bd, 0.0f);

		// visualizer.Run();

		// Compute the cost function and Jd = df/dd.

		memset(A_, 0, m2 * sizeof(float));

		memset(b_, 0, M.h * sizeof(float));

		E = 0.0f;

		memset(Jd, 0, featureArray.n * sizeof(float));

		for (i = 0; i < PtArray.n; i++)
		{
			// e <- f(p; d)

			SDF0_ = SDF0 + featureArray.n * i;

			RVLDIFVECTORS(SDF0_, d, M.w, SDF, j);

			P = PtArray.Element[i];

			e = g[i] - Evaluate(P, SDF, iActiveFeature, false);

			// Jd[iActiveFeature] += e;

			M_ = M.Element + iActiveFeature;

			for (l = 0; l < M.h; l++)
				for (j = l; j < M.h; j++)
					A_[l * M.h + j] += (M_[l * M.w] * M_[j * M.w]);

			for (j = 0; j < M.h; j++)
				b_[j] += (M_[j * M.w] * e);

			E = E + e * e;
		}

		if (E_ > 0.0f && E > E_)
		{
			for (i = 0; i < M.h; i++)
				q[i] += dq_[i];

			lambda *= lambdaUp;
		}
		else if (E_ < 0.0f)
		{
			maxTrA = 0.0;

			for (i = 0; i < M.h; i++)
			{
				fTmp = A_[i * M.h + i];

				if (fTmp > maxTrA)
					maxTrA = fTmp;
			}

			lambda = 0.1f * maxTrA;

			E_ = E;
		}
		else
		{
			lambda *= lambdaDown;

			E_ = E;
		}

		for (i = 1; i < M.h; i++)
			for (j = 0; j < i; j++)
				A_[i * M.h + j] = A_[j * M.h + i];

		for (i = 0; i < M.h; i++)
			A_[i * M.h + i] += lambda;

		cv::solve(A, b, dq);

		for (i = 0; i < M.h; i++)
			q[i] -= dq_[i];
	}

	// Translate q to the scene RF.

	RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		NS = NS_ + 3 * iFeature;

		d[iFeature] += RVLDOTPRODUCT3(NS, Pc);
	}

	RVLMULMXVECT(M.Element, d, M.h, M.w, q, i, j, a);

	// Visualization

	if (bVisualize)
	{
		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		pVisualizer->renderer->RemoveAllViewProps();

		Display(pVisualizer, 0.01f, d, bd, 0.0f);

		pVisualizer->Run();

		delete pVisualizer;

		delete[] bd;
	}

	// Free memory.

	delete[] imagePtArray.Element;
	delete[] PtArray.Element;
	delete[] bBelongsToObject;
	delete[] d;
	delete[] SDF;
	delete[] SDF0;
	delete[] NS_;
	// delete[] J;
	delete[] Jd;
	delete[] Jq;
	delete[] g;
	delete[] PMem;
}

float VN::FitRotLM(
	void *vpClassifier,
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	Camera camera,
	SURFEL::SceneSamples sceneSamples,
	RECOG::ClassData *pClass,
	float *d0,
	RECOG::VN_::FitParams params,
	float *qOpt,
	float *ROpt,
	bool bVisualize,
	bool bVisualizeBestInstance)
{
	// Parameters.

	// uchar level1EvaluationFitting = 0;	// RotLM (best, slowest)
	uchar level1EvaluationFitting = 1; // LS translation and scale

	// uchar level1EvaluationCriterion = 0;	// scene fitting score (best, slowest)
	uchar level1EvaluationCriterion = 1; // CTI descriptor distance

	int nBestProposals = 50;

	///

	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	Array2D<float> M = pClass->M;

	// float *q = new float[M.h];

	float *d = new float[M.w];

	float t[3];

	RVLNULL3VECTOR(t);

	Array<OrientedPoint> PtArray;

	PtArray.n = sceneSamples.imagePtArray.h;

	PtArray.Element = new OrientedPoint[PtArray.n];

	float EOpt = -1.0f;

	float scoreOpt = -params.kOutliers * (float)(sceneSamples.imagePtArray.h) - 1.0f;

	float Pc[3];

	RVLNULL3VECTOR(Pc);

	// Sample SO(3).

	int nRotSamples = pClassifier->SO3Samples.c;

	// Generate alignment proposals, one for every SO(3) sample.

	printf("Generating alignment proposals");

	Array<SortIndex<float>> proposals;

	proposals.Element = new SortIndex<float>[nRotSamples * pClass->nq0];

	proposals.n = nRotSamples * pClass->nq0;

	SortIndex<float> *pProposal = proposals.Element;

	int nqs = M.h - 4;

	float *q_ = new float[M.h * nRotSamples * pClass->nq0];

	float *q = q_;

	int iProposal = 0;

	float *PGnd = (params.bGnd ? sceneSamples.PGnd : NULL);

	int i_, k;
	float R_[9];
	float EHull, scoreSceneFit, score, E;
	float *a;
	int nOutliers;
	int iq0;
	int iRotSample;
	float *R__;

	for (iq0 = 0; iq0 < pClass->nq0; iq0++)
	{
		for (iRotSample = 0; iRotSample < nRotSamples; iRotSample++, q += M.h, iProposal++)
		{
			R__ = pClassifier->SO3Samples.Element + 9 * iRotSample;

			pProposal->idx = iProposal;

			if (level1EvaluationFitting == 0)
				FitRotLM(pSurfels, iVertexArray, sceneSamples, pClass, d0, pClass->q0 + iq0 * nqs, R__, params, q, R_, EHull);
			else // if (level1EvaluationFitting == 1)
			{
				RVLCOPYMX3X3(R__, R_);
				EHull = FitConvexHull(pSurfels, iVertexArray, Pc, sceneSamples.PGnd, M, R_, q, pClass->q0 + iq0 * nqs);
			}

			if (level1EvaluationCriterion == 0)
			{
				RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i_, k, a);

				Project(d, R_, t, camera, sceneSamples.imagePtArray, PtArray);

				scoreSceneFit = VN_::SceneFittingScore(sceneSamples, PtArray, 0.020f, 2.0f, params.cosSurfaceRayAngleThr, nOutliers);

				score = scoreSceneFit - params.kOutliers * (float)nOutliers;

				pProposal->cost = -score;

				// if (score > scoreOpt)
				//{
				//	scoreOpt = score;

				//	RVLCOPYMX3X3(R_, ROpt);

				//	memcpy(qOpt, q, M.h * sizeof(float));
				//}
			}
			else // if (level1EvaluationCriterion == 1)
			{
				E = EHull;

				// if (EOpt < 0.0f || E < EOpt)
				//{
				//	EOpt = E;

				//	RVLCOPYMX3X3(R_, ROpt);

				//	memcpy(qOpt, q, M.h * sizeof(float));
				//}

				pProposal->cost = E;
			}

			pProposal++;

			if (iRotSample % 100 == 99)
				printf(".");
		}
	}

	printf("\n");

	printf("Evaluating alignment proposals...");

	Array<SortIndex<float>> bestProposals;

	bestProposals.Element = new SortIndex<float>[nBestProposals];

	Min<SortIndex<float>, float>(proposals, nBestProposals, bestProposals);

	float *SDFOpt;

	if (bVisualize)
		SDFOpt = new float[sceneSamples.imagePtArray.h];

	int i;
	float sceneSupport;
	float sceneSupportOpt;
	int nOutliersOpt;
	float EHullOpt;
	SortIndex<float> *pBestProposal;
	float maxnIterations;
	bool bGnd;
	float zMin, zMinOpt;

	for (i = 0; i < nBestProposals; i++)
	{
		pProposal = bestProposals.Element + i;

		iRotSample = pProposal->idx % nRotSamples;
		iq0 = pProposal->idx / nRotSamples;

		R__ = pClassifier->SO3Samples.Element + 9 * iRotSample;

		q = q_ + M.h * pProposal->idx;

		FitRotLM(pSurfels, iVertexArray, sceneSamples, pClass, d0, pClass->q0 + nqs * iq0, R__, params, q, R_, EHull);

		maxnIterations = params.maxnIterations;
		bGnd = params.bGnd;

		params.maxnIterations = 0;
		params.bInit = false;
		params.bGnd = params.bGnd2;

		// if (i == 12)
		//	int debug = 0;

		FitRotLMCC(pSurfels, iVertexArray, sceneSamples, pClass, d0, NULL, R_, params, q, R_, EHullOpt, false, &camera);

		params.maxnIterations = maxnIterations;
		params.bGnd = bGnd;
		params.bInit = true;

		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i_, k, a);

		Project(d, R_, t, camera, sceneSamples.imagePtArray, PtArray, pClassifier->NGnd, &zMin);

		zMin -= pClassifier->dGnd;

		sceneSupport = VN_::SceneFittingScore(sceneSamples, PtArray, params.maxe, 2.0f, params.cosSurfaceRayAngleThr, nOutliers, false);

		// score = params.kSceneSupport * sceneSupport - params.kOutliers * (float)nOutliers - params.kHull * EHull;
		score = params.kSceneSupport * sceneSupport - params.kOutliers * (float)nOutliers - params.kZMin * RVLABS(zMin);

		if (score > scoreOpt)
		{
			scoreOpt = score;

			sceneSupportOpt = sceneSupport;

			nOutliersOpt = nOutliers;

			EHullOpt = EHull;

			zMinOpt = zMin;

			RVLCOPYMX3X3(R_, ROpt);

			memcpy(qOpt, q, M.h * sizeof(float));

			pBestProposal = pProposal;

			if (bVisualize)
				memcpy(SDFOpt, sceneSamples.SDF, sceneSamples.imagePtArray.h * sizeof(float));
		}
	}

	iRotSample = pProposal->idx % nRotSamples;

	R__ = pClassifier->SO3Samples.Element + 9 * iRotSample;

	q = q_ + M.h * pProposal->idx;

	bGnd = params.bGnd;

	params.bInit = false;
	params.bGnd = params.bGnd2;

	memcpy(q, qOpt, M.h * sizeof(float));

	RVLCOPYMX3X3(ROpt, R_);

	// FitRotLMCC(pSurfels, iVertexArray, sceneSamples, pClass, d0, R__, params, qOpt, ROpt, EHullOpt, false, &camera);
	FitRotLMCC(pSurfels, iVertexArray, sceneSamples, pClass, d0, NULL, R_, params, q, R_, EHull, false, &camera);

	params.bInit = true;
	params.bGnd = bGnd;

	RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i_, k, a);

	Project(d, R_, t, camera, sceneSamples.imagePtArray, PtArray, pClassifier->NGnd, &zMin);

	zMin -= pClassifier->dGnd;

	// memcpy(sceneSamples.SDF, SDFOpt, sceneSamples.imagePtArray.h * sizeof(float));	// If FitRotLMCC is used, this line shoule be commented!

	sceneSupport = VN_::SceneFittingScore(sceneSamples, PtArray, params.maxe, 2.0f, params.cosSurfaceRayAngleThr, nOutliers, false);

	// score = params.kSceneSupport * sceneSupport - params.kOutliers * (float)nOutliers - params.kHull * EHull;
	score = params.kSceneSupport * sceneSupport - params.kOutliers * (float)nOutliers - params.kZMin * RVLABS(zMin);

	if (score > scoreOpt)
	{
		scoreOpt = score;

		sceneSupportOpt = sceneSupport;

		nOutliersOpt = nOutliers;

		EHullOpt = EHull;

		zMinOpt = zMin;

		RVLCOPYMX3X3(R_, ROpt);

		memcpy(qOpt, q, M.h * sizeof(float));

		if (bVisualize)
			memcpy(SDFOpt, sceneSamples.SDF, sceneSamples.imagePtArray.h * sizeof(float));
	}

	delete[] q_;
	delete[] proposals.Element;
	delete[] bestProposals.Element;

	fprintf(params.fpScore, "%f\t%d\t%f\t", sceneSupportOpt, nOutliersOpt, zMinOpt);

	printf("finished\n");

	printf("score=%f (scene support=%f nOutliers=%d EHull=%f minz=%f)\n", scoreOpt, sceneSupportOpt, nOutliersOpt, EHullOpt, zMinOpt);

	// Visualization

	if (bVisualize)
	{
		int j;

		RVLMULMXTVECT(M.Element, qOpt, M.h, M.w, d, i, j, a);

		Project(d, ROpt, t, camera, sceneSamples.imagePtArray, PtArray);

		memcpy(sceneSamples.SDF, SDFOpt, sceneSamples.imagePtArray.h * sizeof(float));

		VN_::SceneFittingScore(sceneSamples, PtArray, params.maxe, 2.0f, params.cosSurfaceRayAngleThr, nOutliers, true);

		delete[] SDFOpt;

		if (bVisualizeBestInstance)
		{
			// Visualize VN model instance defined by qOpt.

			Visualizer visualizer;

			visualizer.Create();

			bool *bd;

			bd = new bool[featureArray.n];

			for (i = 0; i < featureArray.n; i++)
				bd[i] = true;

			visualizer.renderer->RemoveAllViewProps();

			Display(&visualizer, 0.01f, d, bd, 0.0f);

			visualizer.Run();

			delete[] bd;
		}
	}

	delete[] d;
	delete[] PtArray.Element;

	return scoreOpt;
}

float VN::FitRotLM(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	SURFEL::SceneSamples sceneSamples,
	RECOG::ClassData *pClass,
	float *d0,
	float *qs0,
	float *RIn,
	RECOG::VN_::FitParams params,
	float *q,
	float *R,
	float &EHull,
	bool bVisualize)
{
	Array2D<float> M = pClass->M;

	// Constants

	int maxnIterations = 10;
	float lambdaUp = 2.0f;
	float lambdaDown = 1.0f / lambdaUp;
	// float kMinAbsJq = 0.1f;
	int imageNeighborhood = 7;
	bool bExternalSamples = false;

	Array<float *> PtArray = sceneSamples.PtArray;
	float *g = sceneSamples.g;
	uchar *status = sceneSamples.status;

	// int halfImageNeighborhood = (imageNeighborhood - 1) / 2;

	// Rect<int> cropWin;
	// cropWin.minx = 0;
	// cropWin.maxx = camera.w - 1;
	// cropWin.miny = 0;
	// cropWin.maxy = camera.h - 1;

	// bool *bBelongsToObject = new bool[pSurfels->NodeArray.n];

	// memset(bBelongsToObject, 0, pSurfels->NodeArray.n * sizeof(bool));

	// QLIST::Index *pSurfelIdx = surfelList.pFirst;

	// while (pSurfelIdx)
	//{
	//	bBelongsToObject[pSurfelIdx->Idx] = true;

	//	pSurfelIdx = pSurfelIdx->pNext;
	//}

	// Array<float *> PtArray;

	// PtArray.Element = new float *[imagePtArray.h];

	// PtArray.n = 0;

	// float *g = new float[imagePtArray.h];

	// float *PMem = new float[3 * imagePtArray.h];

	// float *P__ = PMem;

	// int i;
	// float *P;
	// float *m;
	// int u, v;
	// int iPt;
	// int iSurfel;
	// int u_, v_;
	// Rect<int> neighborhood;
	// float Ray[3], dP[3], P_[3];
	// float s, fTmp, sClosest;
	// float dist, minDist;
	// bool bFirst;

	// for (i = 0; i < imagePtArray.h; i++)
	//{
	//	m = imagePtArray.Element + imagePtArray.w * i;

	//	u = (int)round(m[0]);
	//	v = (int)round(m[1]);

	//	iPt = u + v * pMesh->width;

	//	iSurfel = pSurfels->surfelMap[iPt];

	//	if (iSurfel < 0 || iSurfel >= pSurfels->NodeArray.n)
	//		continue;

	//	if (bBelongsToObject[iSurfel])
	//	{
	//		g[PtArray.n] = 0.0f;

	//		P = pMesh->NodeArray.Element[iPt].P;

	//		RVLCOPY3VECTOR(P, P__);

	//		PtArray.Element[PtArray.n++] = P__;

	//		P__ += 3;
	//	}
	//	else if (bExternalSamples)
	//	{
	//		RVLSET3VECTOR(Ray, ((m[0] - camera.uc) / camera.fu), ((m[1] - camera.vc) / camera.fv), 1.0f);

	//		fTmp = RVLDOTPRODUCT3(Ray, Ray);

	//		neighborhood.minx = u - halfImageNeighborhood;
	//		neighborhood.maxx = u + halfImageNeighborhood;
	//		neighborhood.miny = v - halfImageNeighborhood;
	//		neighborhood.maxy = v + halfImageNeighborhood;

	//		CropRect<int>(neighborhood, cropWin);

	//		bFirst = true;

	//		for (v_ = neighborhood.miny; v_ <= neighborhood.maxy; v_++)
	//			for (u_ = neighborhood.minx; u_ <= neighborhood.maxx; u_++)
	//			{
	//				iPt = u_ + v_ * pMesh->width;

	//				iSurfel = pSurfels->surfelMap[iPt];

	//				if (iSurfel < 0 || iSurfel >= pSurfels->NodeArray.n)
	//					continue;

	//				if (!bBelongsToObject[iSurfel])
	//					continue;

	//				P = pMesh->NodeArray.Element[iPt].P;

	//				s = RVLDOTPRODUCT3(Ray, P) / fTmp;

	//				RVLSCALE3VECTOR(Ray, s, P_);

	//				RVLDIF3VECTORS(P, P_, dP);

	//				dist = RVLDOTPRODUCT3(dP, dP);

	//				if (bFirst)
	//				{
	//					minDist = dist;

	//					sClosest = s;

	//					bFirst = false;
	//				}
	//				else if (dist < minDist)
	//				{
	//					minDist = dist;

	//					sClosest = s;
	//				}
	//			}

	//		if (!bFirst)
	//		{
	//			RVLSCALE3VECTOR(Ray, sClosest, P__);

	//			g[PtArray.n] = sqrt(minDist);

	//			PtArray.Element[PtArray.n++] = P__;

	//			P__ += 3;
	//		}
	//	}
	//}

	//// Compute centroid of scene points.

	// float Pc[3];

	// RVLNULL3VECTOR(Pc);

	// int i;
	// float *P;

	// for (i = 0; i < PtArray.n; i++)
	//{
	//	P = PtArray.Element[i];

	//	RVLSUM3VECTORS(Pc, P, Pc);
	//}

	// float fTmp = (float)(PtArray.n);

	// RVLSCALE3VECTOR2(Pc, fTmp, Pc);

	//// Center PtArray in Pc.

	// for (i = 0; i < PtArray.n; i++)
	//{
	//	P = PtArray.Element[i];

	//	RVLDIF3VECTORS(P, Pc, P);
	//}

	// R <- RIn

	RVLCOPYMX3X3(RIn, R);

	// Allocate VN descriptor.

	float *d = new float[featureArray.n];

	// Transform model normals to the scene RF and center the descriptor in Pc.

	int nx = M.h + 3;

	float *J = new float[nx];

	int iFeature;
	float *NM;
	float NS[3];
	int i, j;
	float *a;

	if (params.bInit)
		// LS fitting of the reference model to the scene data.

		FitConvexHull(pSurfels, iVertexArray, sceneSamples.Pc, sceneSamples.PGnd, M, R, q, qs0);
	else
	{
		for (iFeature = 0; iFeature < featureArray.n; iFeature++)
		{
			NM = featureArray.Element[iFeature].N;

			RVLMULMX3X3VECT(RIn, NM, NS);

			d[iFeature] = d0[iFeature] - RVLDOTPRODUCT3(NS, sceneSamples.Pc);
		}

		// q <- M' * d

		RVLMULMXVECT(M.Element, d, M.h, M.w, q, i, j, a);
	}

	// Initialize visualization.

	Visualizer *pVisualizer;
	bool *bd;

	if (bVisualize)
	{
		pVisualizer = new Visualizer;

		pVisualizer->Create();

		bd = new bool[featureArray.n];

		for (i = 0; i < featureArray.n; i++)
			bd[i] = true;
	}

	/// Minimization of the cost function.

	float *SDF = new float[featureArray.n];

	cv::Mat A;

	A.create(nx, nx, CV_32FC1);

	float *A_ = (float *)(A.data);

	cv::Mat b;

	b.create(nx, 1.0f, CV_32FC1);

	float *b_ = (float *)(b.data);

	cv::Mat dx;

	dx.create(nx, 1.0f, CV_32FC1);

	float *dx_ = (float *)(dx.data);

	float *dphi = dx_;

	int nx2 = nx * nx;

	float *Jq = J + 3;

	float E;

	float E_ = -1.0f;

	Array<int> iFreeArray;

	iFreeArray.Element = new int[nx];

	for (i = 0; i < 7; i++)
		iFreeArray.Element[i] = i;

	int nqs = M.h - 4;

	float *qs = q + 4;

	float *dqs = dx_ + 7;

	float *bs = b_ + 7;

	float *qMin_ = new float[nqs];

	float *qMax_ = new float[nqs];

	for (i = 0; i < nqs; i++)
	{
		qMin_[i] = pClass->qMin[i] * q[3];
		qMax_[i] = pClass->qMax[i] * q[3];
	}

	int *limit = new int[nqs];

	memset(limit, 0, nqs * sizeof(int));

	float *Ar_ = new float[nx * nx];
	float *br_ = new float[nx];
	float *dxr_ = new float[nx];

	float *PGnd = (params.bGnd ? sceneSamples.PGnd : NULL);

	int k, l;
	float e;
	int iActiveFeature;
	float *M_;
	float lambda;
	float maxTrA;
	float R_[9], R__[9], dR[9];
	float z[3];
	float th, s;
	float d_, dSmax, dMmax;
	VN_::ParallelDescriptorComponent *pParallelDescriptorComponent;
	SURFEL::Vertex *pVertex, *pVertex_;
	float mins, qs_;
	bool bRepeat;
	int iLimit, sgnLimit;
	float *P;
	float fTmp;
	float SDF_;

	for (k = 0; k <= maxnIterations; k++)
	{
		// d <- M * q

		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		// Visualization

		// visualizer.renderer->RemoveAllViewProps();

		// Display(&visualizer, 0.01f, d, bd, 0.0f);

		// visualizer.Run();

		// Compute the cost function and Jd = df/dd.

		memset(A_, 0, nx2 * sizeof(float));

		memset(b_, 0, nx * sizeof(float));

		E = 0.0f;

		for (i = 0; i < PtArray.n; i++)
		{
			if (status[i] != 1)
				continue;

			// e <- f(p; d)

			P = PtArray.Element[i];

			for (iFeature = 0; iFeature < featureArray.n; iFeature++)
			{
				NM = featureArray.Element[iFeature].N;

				RVLMULMX3X3VECT(R, NM, NS);

				SDF[iFeature] = RVLDOTPRODUCT3(NS, P) - d[iFeature];
			}

			sceneSamples.SDF[i] = SDF_ = Evaluate(P, SDF, iActiveFeature, false);

			e = g[i] - SDF_;

			// Jd[iActiveFeature] += e;

			M_ = M.Element + iActiveFeature;

			NM = featureArray.Element[iActiveFeature].N;

			RVLMULMX3X3VECT(R, NM, NS);

			RVLCROSSPRODUCT3(NS, P, J);

			for (j = 0; j < M.h; j++)
				Jq[j] = -M_[j * M.w];

			for (l = 0; l < nx; l++)
				for (j = l; j < nx; j++)
					A_[l * nx + j] += (J[l] * J[j]);

			for (j = 0; j < nx; j++)
				b_[j] += (J[j] * e);

			E = E + e * e;
		} // for every sample point

		EHull = FitConvexHullIteration(pSurfels, iVertexArray, sceneSamples.Pc, PGnd, d, M, M.h, true, params.alpha, R, A_, b_, J);

		E += EHull;

		if (E_ > 0.0f && E > E_)
		{
			for (i = 0; i < M.h; i++)
				q[i] -= dx_[i + 3];

			RVLCOPYMX3X3(R_, R);

			if (k >= maxnIterations - 1)
				break;
			else
				lambda *= lambdaUp;
		}
		else if (E_ < 0.0f)
		{
			maxTrA = 0.0;

			for (i = 0; i < nx; i++)
			{
				fTmp = A_[i * nx + i];

				if (fTmp > maxTrA)
					maxTrA = fTmp;
			}

			lambda = 0.1f * maxTrA;

			E_ = E;

			RVLCOPYMX3X3(R, R_);
		}
		else
		{
			lambda *= lambdaDown;

			E_ = E;

			RVLCOPYMX3X3(R, R_);
		}

		if (k >= maxnIterations)
			break;

		for (i = 1; i < nx; i++)
			for (j = 0; j < i; j++)
				A_[i * nx + j] = A_[j * nx + i];

		for (i = 0; i < nx; i++)
			A_[i * nx + i] += lambda;

		bRepeat = true;

		while (bRepeat)
		{
			iFreeArray.n = 7;

			for (i = 0; i < nqs; i++)
				if (limit[i] == 0 || limit[i] == 1 && bs[i] < 0 || limit[i] == -1 && bs[i] > 0)
					iFreeArray.Element[iFreeArray.n++] = i + 7;

			if (iFreeArray.n == nx)
				cv::solve(A, b, dx);
			else
			{
				memset(dx_, 0, nx * sizeof(float));

				if (iFreeArray.n > 0)
				{
					cv::Mat Ar(iFreeArray.n, iFreeArray.n, CV_32FC1, Ar_);
					cv::Mat br(iFreeArray.n, 1, CV_32FC1, br_);
					cv::Mat dxr(iFreeArray.n, 1, CV_32FC1, dxr_);

					for (i = 0; i < iFreeArray.n; i++)
					{
						for (j = i; j < iFreeArray.n; j++)
							Ar_[i * iFreeArray.n + j] = A_[iFreeArray.Element[i] * nx + iFreeArray.Element[j]];

						br_[i] = b_[iFreeArray.Element[i]];
					}

					for (i = 1; i < iFreeArray.n; i++)
						for (j = 0; j < i; j++)
							Ar_[i * iFreeArray.n + j] = Ar_[j * iFreeArray.n + i];

					cv::solve(Ar, br, dxr);

					for (i = 0; i < iFreeArray.n; i++)
						dx_[iFreeArray.Element[i]] = dxr_[i];
				}
			}

			mins = 1.0f;

			iLimit = -1;

			bRepeat = false;

			for (i = 0; i < nqs; i++)
			{
				if (limit[i] == 1 && dqs[i] < -1e-10)
					limit[i] = 0;
				else if (limit[i] == -1 && dqs[i] > 1e-10)
					limit[i] = 0;

				if (limit[i] == 0)
				{
					qs_ = qs[i] + dqs[i];

					if (qs_ < qMin_[i])
					{
						s = (qMin_[i] - qs[i]) / dqs[i];

						if (s < mins)
						{
							mins = s;

							iLimit = i;

							sgnLimit = -1;
						}
					}
					else if (qs_ > qMax_[i])
					{
						s = (qMax_[i] - qs[i]) / dqs[i];

						if (s < mins)
						{
							mins = s;

							iLimit = i;

							sgnLimit = 1;
						}
					}
				}
				else if (bs[i] * dqs[i] < -1e-8)
				{
					limit[i] *= 2;

					bRepeat = true;
				}
			}

			if (iLimit >= 0)
				limit[iLimit] = sgnLimit;

			if (!bRepeat)
				RVLSCALEVECTOR(dx_, mins, dx_, nx, i);
		}

		for (i = 0; i < nqs; i++)
			if (limit[i] == 2)
				limit[i] = 1;
			else if (limit[i] == -2)
				limit[i] = -1;

		for (i = 0; i < M.h; i++)
			q[i] += dx_[i + 3];

		// Only for debugging purpose!

		// for (i = 0; i < nqs; i++)
		//	if (qs[i] < qMin_[i] - 1e-8 || qs[i] > qMax_[i] + 1e-8)
		//		printf("latent vector component %d out of limits!\n", i);

		// dphi = || dphi ||

		th = sqrt(RVLDOTPRODUCT3(dphi, dphi));

		if (RVLABS(th) >= 1e-8)
		{
			// z <- dphi / || dphi ||

			RVLSCALE3VECTOR2(dphi, th, z);

			// dR <- Rot(z, th) (angle axis to rotation matrix)

			AngleAxisToRot<float>(z, th, dR);

			// R <- dR * R

			RVLMXMUL3X3(dR, R, R__);

			RVLCOPYMX3X3(R__, R);
		}
	}

	// If q is outside the class limits, then q <- the closest point within the class limits.

	// float qn;

	// for (i = 0; i < nqs; i++)
	//{
	//	qn = qs[i] / q[3];

	//	if (qn > pClass->qMax[i])
	//		q[i] = pClass->qMax[i] * q[3];
	//	else if (qn < pClass->qMin[i])
	//		q[i] = pClass->qMin[i] * q[3];
	//}

	// float mins = 1.0f;

	// for (i = 0; i < nqs; i++)
	//{
	//	qn = qs[i] / q[3];

	//	if (qn > pClass->qMax[i])
	//		s = pClass->qMax[i] / qn;
	//	else if (qn < pClass->qMin[i])
	//		s = pClass->qMin[i] / qn;

	//	if (s < mins)
	//		mins = s;
	//}

	// for (i = 0; i < nqs; i++)
	//	qs[i] *= s;

	// Translate q to the scene RF.

	RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		NM = featureArray.Element[iFeature].N;

		RVLMULMX3X3VECT(R, NM, NS);

		d[iFeature] += RVLDOTPRODUCT3(NS, sceneSamples.Pc);
	}

	RVLMULMXVECT(M.Element, d, M.h, M.w, q, i, j, a);

	// Visualization

	if (bVisualize)
	{
		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		pVisualizer->renderer->RemoveAllViewProps();

		Display(pVisualizer, 0.01f, d, bd, 0.0f);

		pVisualizer->Run();

		delete pVisualizer;

		delete[] bd;
	}

	// Free memory.

	delete[] d;
	delete[] SDF;
	delete[] J;
	delete[] iFreeArray.Element;
	delete[] qMin_;
	delete[] qMax_;
	delete[] limit;
	delete[] Ar_;
	delete[] br_;
	delete[] dxr_;

	// Output

	return E;
}

float VN::FitRotLMCC(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	SURFEL::SceneSamples sceneSamples,
	RECOG::ClassData *pClass,
	float *d0,
	float *qs0,
	float *RIn,
	RECOG::VN_::FitParams params,
	float *q,
	float *R,
	float &EHull,
	bool bVisualize,
	Camera *pCamera)
{
	Array2D<float> M = pClass->M;

	// Constants

	int maxnStepReductionIterations = 10;
	float lambdaUp = 2.0f;
	float lambdaDown = 1.0f / lambdaUp;
	// float kMinAbsJq = 0.1f;
	int imageNeighborhood = 7;
	bool bExternalSamples = false;
	float alpha0 = 0.5f;

	Array<float *> PtArray = sceneSamples.PtArray;
	float *g = sceneSamples.g;
	uchar *status = sceneSamples.status;

	// R <- RIn

	RVLCOPYMX3X3(RIn, R);

	// Allocate VN descriptor.

	float *d = new float[featureArray.n];

	// Transform model normals to the scene RF and center the descriptor in Pc.

	int nx = M.h + 3;

	float *J = new float[nx];

	float *qsGoal = NULL;

	float *qs = q + 4;

	int nqs = M.h - 4;

	float *dx_ = new float[nx];

	float *dphi = dx_;

	float *dq = dx_ + 3;

	float *dqs = dq + 4;

	float *NS_ = new float[3 * featureArray.n];

	int iFeature;
	float *NM;
	float NS[3];
	int i, j;
	float *a;
	bool bInit;

	if (params.bInit)
	{
		// LS fitting of the reference model to the scene data.

		FitConvexHull(pSurfels, iVertexArray, sceneSamples.Pc, sceneSamples.PGnd, M, R, q, qs0);

		bInit = false;
	}
	else
	{
		// RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		// float t[3];

		// RVLNULL3VECTOR(t);

		// Array<OrientedPoint> PtArray_;

		// PtArray_.n = sceneSamples.imagePtArray.h;

		// PtArray_.Element = new OrientedPoint[PtArray_.n];

		// Project(d, R, t, *pCamera, sceneSamples.imagePtArray, PtArray_);

		// int nOutliersOpt;

		// VN_::SceneFittingScore(sceneSamples, PtArray_, params.maxe, 2.0f, params.cosSurfaceRayAngleThr, nOutliersOpt, true);

		// delete[] PtArray_.Element;

		// Translate q to the scene RF.

		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		for (iFeature = 0; iFeature < featureArray.n; iFeature++)
		{
			NM = featureArray.Element[iFeature].N;

			RVLMULMX3X3VECT(R, NM, NS);

			d[iFeature] -= RVLDOTPRODUCT3(NS, sceneSamples.Pc);
		}

		RVLMULMXVECT(M.Element, d, M.h, M.w, q, i, j, a);

		qsGoal = new float[nqs];

		memcpy(qsGoal, qs, nqs * sizeof(float));

		memset(qs, 0, nqs * sizeof(float));

		bInit = true;

		RVLNULL3VECTOR(dq);

		dq[3] = 0.0f;

		// for (iFeature = 0; iFeature < featureArray.n; iFeature++)
		//{
		//	NM = featureArray.Element[iFeature].N;

		//	RVLMULMX3X3VECT(RIn, NM, NS);

		//	d[iFeature] = d0[iFeature] - RVLDOTPRODUCT3(NS, sceneSamples.Pc);
		//}

		//// q <- M' * d

		// RVLMULMXVECT(M.Element, d, M.h, M.w, q, i, j, a);
	}

	// Initialize visualization.

	Visualizer *pVisualizer;
	bool *bd;

	if (bVisualize)
	{
		pVisualizer = new Visualizer;

		pVisualizer->Create();

		bd = new bool[featureArray.n];

		for (i = 0; i < featureArray.n; i++)
			bd[i] = true;
	}

	/// Minimization of the cost function.

	float *SDF = new float[featureArray.n];

	cv::Mat A;

	// A.create(nx, nx, CV_32FC1);
	A.create(7, 7, CV_32FC1);

	float *A_ = (float *)(A.data);

	cv::Mat b;

	// b.create(nx, 1.0f, CV_32FC1);
	b.create(7, 1, CV_32FC1);

	float *b_ = new float[nx];

	cv::Mat dw(7, 1, CV_32FC1);

	float *dw_ = (float *)(dw.data);

	int nx2 = nx * nx;

	float *Jq = J + 3;

	float E;

	float E_ = -1.0f;

	// Array<int> iFreeArray;

	// iFreeArray.Element = new int[nx];

	// for (i = 0; i < 7; i++)
	//	iFreeArray.Element[i] = i;

	float *bq = b_ + 3;

	float *bs = bq + 4;

	float s0 = q[3];

	// float *qMin_ = new float[nqs];

	// float *qMax_ = new float[nqs];

	// for (i = 0; i < nqs; i++)
	//{
	//	qMin_[i] = pClass->qMin[i] * s0;
	//	qMax_[i] = pClass->qMax[i] * s0;
	// }

	int *limit = new int[nqs];

	memset(limit, 0, nqs * sizeof(int));

	// float *Ar_ = new float[nx * nx];
	// float *br_ = new float[nx];
	// float *dxr_ = new float[nx];

	float *qs_ = new float[nqs];

	bool *bLimit = new bool[pClass->nHull];

	memset(bLimit, 0, pClass->nHull * sizeof(bool));

	Array<int> iLimitArray;

	iLimitArray.Element = new int[pClass->nHull];

	iLimitArray.n = 0;

	float *a_ = new float[pClass->nHull];

	bool bRot = false;

	int kStepReduction = 0;

	int nLimited_ = -1;

	float *PGnd = (params.bGnd ? sceneSamples.PGnd : NULL);

	int k, l;
	float e;
	int iActiveFeature;
	float *M_;
	float lambda;
	float maxTrA;
	float R_[9], R__[9], dR[9];
	float z[3];
	float th, s;
	float d_, dSmax, dMmax;
	VN_::ParallelDescriptorComponent *pParallelDescriptorComponent;
	SURFEL::Vertex *pVertex, *pVertex_;
	float mins;
	bool bRepeat;
	int iLimit, sgnLimit;
	float *P;
	float fTmp;
	float SDF_;
	float *AH, *AH_;
	float gamma;
	float eqs, deqs;
	float bH;
	float alpha;
	int nLimited;
	float *NS__;

	for (k = 0; k <= params.maxnIterations; k++)
	{
		if (!bInit)
		{
			NS__ = NS_;

			for (iFeature = 0; iFeature < featureArray.n; iFeature++, NS__ += 3)
			{
				NM = featureArray.Element[iFeature].N;

				RVLMULMX3X3VECT(R, NM, NS__);
			}

			// d <- M * q

			RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

			// Visualization

			// visualizer.renderer->RemoveAllViewProps();

			// Display(&visualizer, 0.01f, d, bd, 0.0f);

			// visualizer.Run();

			// Compute the cost function and Jd = df/dd.

			memset(A_, 0, 7 * 7 * sizeof(float));

			memset(b_, 0, nx * sizeof(float));

			E = 0.0f;

			for (i = 0; i < PtArray.n; i++)
			{
				if (status[i] != 1)
					continue;

				// e <- f(p; d)

				P = PtArray.Element[i];

				NS__ = NS_;

				for (iFeature = 0; iFeature < featureArray.n; iFeature++, NS__ += 3)
					SDF[iFeature] = RVLDOTPRODUCT3(NS__, P) - d[iFeature];

				sceneSamples.SDF[i] = SDF_ = Evaluate(P, SDF, iActiveFeature, false);

				e = g[i] - SDF_;

				// Jd[iActiveFeature] += e;

				M_ = M.Element + iActiveFeature;

				NM = featureArray.Element[iActiveFeature].N;

				NS__ = NS_ + 3 * iActiveFeature;

				RVLCROSSPRODUCT3(NS__, P, J);

				for (j = 0; j < M.h; j++)
					Jq[j] = -M_[j * M.w];

				for (l = 0; l < 7; l++)
					for (j = l; j < 7; j++)
						A_[l * 7 + j] += (J[l] * J[j]);

				for (j = 0; j < nx; j++)
					b_[j] += (J[j] * e);

				E = E + e * e;
			} // for every sample point

			EHull = FitConvexHullIteration(pSurfels, iVertexArray, sceneSamples.Pc, PGnd, d, M, 4, true, params.alpha, R, A_, b_, J);

			E += EHull;

			if (E_ > 0.0f && E > E_) // If E increased, then return q and R to their previous values.
			{
				RVLDIFVECTORS(q, dq, M.h, q, i);

				RVLCOPYMX3X3(R_, R);

				kStepReduction++;

				if (kStepReduction >= maxnStepReductionIterations)
				{
					kStepReduction = 0;
					bRot = !bRot;
				}
				else
				{
					if (bRot)
						lambda *= lambdaUp;
					else
						alpha *= 0.5f;

					k--;
				}

				if (k >= params.maxnIterations - 1)
					break;
			}
			else if (E_ < 0.0f) // First iteration
			{
				maxTrA = 0.0;

				for (i = 0; i < 7; i++)
				{
					fTmp = A_[i * 7 + i];

					if (fTmp > maxTrA)
						maxTrA = fTmp;
				}

				lambda = 0.1f * maxTrA;

				alpha = alpha0;

				E_ = E;

				RVLCOPYMX3X3(R, R_);
			}
			else // If E decreased.
			{
				// Only for debugging purpose!

				// if (pCamera)
				//{
				//	RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

				//	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
				//	{
				//		NM = featureArray.Element[iFeature].N;

				//		RVLMULMX3X3VECT(R, NM, NS);

				//		d[iFeature] += RVLDOTPRODUCT3(NS, sceneSamples.Pc);
				//	}

				//	float t[3];

				//	RVLNULL3VECTOR(t);

				//	Array<OrientedPoint> PtArray_;

				//	PtArray_.n = sceneSamples.imagePtArray.h;

				//	PtArray_.Element = new OrientedPoint[PtArray_.n];

				//	Project(d, R, t, *pCamera, sceneSamples.imagePtArray, PtArray_);

				//	int nOutliersOpt;

				//	VN_::SceneFittingScore(sceneSamples, PtArray_, params.maxe, 2.0f, params.cosSurfaceRayAngleThr, nOutliersOpt, true);

				//	delete[] PtArray_.Element;
				//}

				/////

				kStepReduction = 0;

				if (bRot)
				{
					lambda *= lambdaDown;

					alpha = alpha0;

					bRot = false;
				}
				else
					bRot = true;

				E_ = E;

				RVLCOPYMX3X3(R, R_);
			}
		} // if (!bInit)

		if (bInit)
			k = -1;
		else if (k >= params.maxnIterations)
			break;

		if (bRot)
		{
			for (i = 1; i < 7; i++)
				for (j = 0; j < i; j++)
					A_[i * 7 + j] = A_[j * 7 + i];

			for (i = 0; i < 7; i++)
				A_[i * 7 + i] += lambda;

			memcpy(b.data, b_, 7 * sizeof(float));

			cv::solve(A, b, dw);

			memcpy(dx_, dw_, 7 * sizeof(float));

			memset(dqs, 0, nqs * sizeof(float));
		}
		else
		{
			if (bInit)
			{
				RVLDIFVECTORS(qsGoal, qs, nqs, dqs, i);

				// Only for debugging purpose!

				// RVLDOTPRODUCT(dqs, dqs, nqs, fTmp, i);

				// float eq = sqrt(fTmp);

				// int debug = 0;
			}
			else
			{
				RVLDOTPRODUCT(bq, bq, M.h, fTmp, i);

				gamma = 0.5f * alpha * E / fTmp;

				RVLSCALEVECTOR(bq, gamma, dq, M.h, i);
			}

			RVLNULL3VECTOR(dphi);

			iLimitArray.n = 0;

			for (i = 0; i < pClass->nHull; i++)
			{
				if (bLimit[i])
				{
					AH = pClass->A + nqs * i;

					RVLDOTPRODUCT(AH, dqs, nqs, deqs, j);

					if (deqs < 0.0f)
					{
						if (deqs < -1e-4 * s0)
							bLimit[i] = false;
					}
					else
					{
						memcpy(a_, AH, pClass->nHull * sizeof(float));

						for (j = 0; j < iLimitArray.n; j++)
						{
							AH_ = pClass->A + nqs * iLimitArray.Element[j];

							RVLDOTPRODUCT(AH_, a_, nqs, s, l);

							for (l = 0; l < nqs; l++)
								a_[l] -= (s * AH_[l]);
						}

						RVLDOTPRODUCT(a_, a_, nqs, fTmp, j);

						if (fTmp > 0.0f)
						{
							s = 1.0f / sqrt(fTmp);

							RVLSCALEVECTOR(a_, s, a_, nqs, j);

							RVLDOTPRODUCT(a_, dqs, nqs, s, j);

							for (j = 0; j < nqs; j++)
								dqs[j] -= (s * a_[j]);
						}

						iLimitArray.Element[iLimitArray.n++] = i;
					}
				}
			}

			mins = 1.0f;

			for (i = 0; i < pClass->nHull; i++)
			{
				if (!bLimit[i])
				{
					AH = pClass->A + nqs * i;

					RVLDOTPRODUCT(AH, dqs, nqs, deqs, j);

					if (deqs != 0.0f)
					{
						bH = s0 * pClass->b[i];

						RVLDOTPRODUCT(AH, qs, nqs, eqs, j);

						eqs -= bH;

						if (eqs + deqs > 0.0f)
						{
							s = -eqs / deqs;

							if (s < mins)
							{
								mins = s;

								iLimit = i;
							}
						}
					}
				}
			}

			if (mins < 1.0f)
				bLimit[iLimit] = true;

			nLimited = 0;

			for (i = 0; i < pClass->nHull; i++)
				if (bLimit[i])
					nLimited++;

			RVLSCALEVECTOR(dq, mins, dq, M.h, i);

			RVLDOTPRODUCT(dq, dq, M.h, fTmp, i);

			if (bInit)
			{
				if (nLimited >= nqs || nLimited <= nLimited_)
					bInit = false;

				nLimited_ = nLimited;
			}
		}

		RVLSUMVECTORS(q, dq, M.h, q, i);

		// Only for debugging purpose!

		// float *eqs_ = new float[pClass->nHull];

		// for (i = 0; i < pClass->nHull; i++)
		//{
		//	AH = pClass->A + nqs * i;

		//	bH = s0 * pClass->b[i];

		//	RVLDOTPRODUCT(AH, qs, nqs, eqs, j);

		//	eqs -= bH;

		//	if (eqs > 1e-4f)
		//		int debug = 0;

		//	eqs_[i] = eqs;
		//}

		// delete[] eqs_;

		/////

		// Only for debugging purpose!

		// for (i = 0; i < nqs; i++)
		//	if (qs[i] < qMin_[i] - 1e-8 || qs[i] > qMax_[i] + 1e-8)
		//		printf("latent vector component %d out of limits!\n", i);

		/////

		if (bRot)
		{
			// dphi = || dphi ||

			th = sqrt(RVLDOTPRODUCT3(dphi, dphi));

			if (RVLABS(th) >= 1e-8)
			{
				// z <- dphi / || dphi ||

				RVLSCALE3VECTOR2(dphi, th, z);

				// dR <- Rot(z, th) (angle axis to rotation matrix)

				AngleAxisToRot<float>(z, th, dR);

				// R <- dR * R

				RVLMXMUL3X3(dR, R, R__);

				RVLCOPYMX3X3(R__, R);
			}
		}
	} // for (k = 0; k <= params.maxnIterations; k++)

	// Only for debugging purpose!

	// float *eqs_ = new float[pClass->nHull];

	// for (i = 0; i < pClass->nHull; i++)
	//{
	//	AH = pClass->A + nqs * i;

	//	bH = s0 * pClass->b[i];

	//	RVLDOTPRODUCT(AH, qs, nqs, eqs, j);

	//	eqs -= bH;

	//	if (eqs > 1e-5f)
	//		int debug = 0;

	//	eqs_[i] = eqs;
	//}

	// delete[] eqs_;
	// delete[] qsGoal;

	/////

	// If q is outside the class limits, then q <- the closest point within the class limits.

	// float qn;

	// for (i = 0; i < nqs; i++)
	//{
	//	qn = qs[i] / q[3];

	//	if (qn > pClass->qMax[i])
	//		q[i] = pClass->qMax[i] * q[3];
	//	else if (qn < pClass->qMin[i])
	//		q[i] = pClass->qMin[i] * q[3];
	//}

	// float mins = 1.0f;

	// for (i = 0; i < nqs; i++)
	//{
	//	qn = qs[i] / q[3];

	//	if (qn > pClass->qMax[i])
	//		s = pClass->qMax[i] / qn;
	//	else if (qn < pClass->qMin[i])
	//		s = pClass->qMin[i] / qn;

	//	if (s < mins)
	//		mins = s;
	//}

	// for (i = 0; i < nqs; i++)
	//	qs[i] *= s;

	// Compute SDF.

	NS__ = NS_;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++, NS__ += 3)
	{
		NM = featureArray.Element[iFeature].N;

		RVLMULMX3X3VECT(R, NM, NS__);
	}

	RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

	for (i = 0; i < PtArray.n; i++)
	{
		if (status[i] != 1)
			continue;

		P = PtArray.Element[i];

		NS__ = NS_;

		for (iFeature = 0; iFeature < featureArray.n; iFeature++, NS__ += 3)
			SDF[iFeature] = RVLDOTPRODUCT3(NS__, P) - d[iFeature];

		sceneSamples.SDF[i] = Evaluate(P, SDF, iActiveFeature, false);
	}

	// Translate q to the scene RF.

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		NM = featureArray.Element[iFeature].N;

		RVLMULMX3X3VECT(R, NM, NS);

		d[iFeature] += RVLDOTPRODUCT3(NS, sceneSamples.Pc);
	}

	RVLMULMXVECT(M.Element, d, M.h, M.w, q, i, j, a);

	// Visualization

	if (bVisualize)
	{
		RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

		pVisualizer->renderer->RemoveAllViewProps();

		Display(pVisualizer, 0.01f, d, bd, 0.0f);

		pVisualizer->Run();

		delete pVisualizer;

		delete[] bd;
	}

	// Free memory.

	delete[] d;
	delete[] SDF;
	delete[] J;
	// delete[] iFreeArray.Element;
	// delete[] qMin_;
	// delete[] qMax_;
	delete[] limit;
	// delete[] Ar_;
	// delete[] br_;
	// delete[] dxr_;
	delete[] qs_;
	delete[] bLimit;
	delete[] iLimitArray.Element;
	delete[] a_;
	delete[] dx_;
	delete[] NS_;

	// Output

	return E;
}

float VN::FitConvexHull(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	float *Pc,
	float *PGnd,
	Array2D<float> M,
	float *R,
	float *q,
	float *qs0)
{
	RVLNULL3VECTOR(q);

	q[3] = 1.0f;

	int nqs = M.h - 4;

	memcpy(q + 4, qs0, nqs * sizeof(float));

	// q[4] = -0.0633420970093849;
	// q[5] = -0.0790493006858217;
	// q[6] = -0.0329341087239083;
	// q[7] = 0.0237768445331545;
	// q[8] = 5.11683007640849e-005;
	// q[9] = 0.0090215808494636;

	float *d = new float[M.w];

	int i, j;
	float *a;

	RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

	cv::Mat A0;

	A0.create(4, 4, CV_32FC1);

	float *A0_ = (float *)(A0.data);

	cv::Mat b0;

	b0.create(4, 1, CV_32FC1);

	float *b0_ = (float *)(b0.data);

	memset(A0_, 0, 16 * sizeof(float));

	memset(b0_, 0, 4 * sizeof(float));

	float *J = new float[4];

	float E = FitConvexHullIteration(pSurfels, iVertexArray, Pc, PGnd, d, M, 4, false, 1.0f, R, A0_, b0_, J);

	for (i = 1; i < 4; i++)
		for (j = 0; j < i; j++)
			A0_[i * 4 + j] = A0_[j * 4 + i];

	cv::Mat dq0;

	dq0.create(4, 1, CV_32FC1);

	float *dq0_ = (float *)(dq0.data);

	cv::solve(A0, b0, dq0);

	for (i = 0; i < 4; i++)
		q[i] += dq0_[i];

	RVLMULMXTVECT(M.Element, q, M.h, M.w, d, i, j, a);

	E = FitConvexHullIteration(pSurfels, iVertexArray, Pc, PGnd, d, M, 4, false, 1.0f, R);

	delete[] d;
	delete[] J;

	return E;
}

float VN::FitConvexHullIteration(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	float *Pc,
	float *PGnd,
	float *d,
	Array2D<float> M,
	int nq,
	bool bRot,
	float alpha,
	float *R,
	float *A,
	float *b,
	float *J)
{
	float gndCTIThr = 0.015f;

	float *Jq;
	int nx;

	if (bRot)
	{
		Jq = J + 3;
		nx = nq + 3;
	}
	else
	{
		Jq = J;
		nx = nq;
	}

	float E = 0.0f;

	int i, j, l;
	VN_::ParallelDescriptorComponent *pParallelDescriptorComponent;
	float *NM;
	float NS[3];
	SURFEL::Vertex *pVertex, *pVertex_;
	float *P;
	float dSmax, dMmax, d_;
	float P_[3];
	int iActiveFeature;
	float e;
	float *M_;
	float *PGnd_;
	float eGnd;

	for (i = 0; i < parallelDescriptorComponentArray.n; i++)
	{
		pParallelDescriptorComponent = parallelDescriptorComponentArray.Element + i;

		NM = pParallelDescriptorComponent->N;

		RVLMULMX3X3VECT(R, NM, NS);

		pVertex = pSurfels->vertexArray.Element[iVertexArray.Element[0]];

		P = pVertex->P;

		dSmax = RVLDOTPRODUCT3(NS, P);

		for (j = 1; j < iVertexArray.n; j++)
		{
			pVertex_ = pSurfels->vertexArray.Element[iVertexArray.Element[j]];

			P = pVertex_->P;

			d_ = RVLDOTPRODUCT3(NS, P);

			if (d_ > dSmax)
			{
				dSmax = d_;

				pVertex = pVertex_;
			}
		}

		P = pVertex->P;

		if (PGnd)
		{
			for (j = 0; j < iVertexArray.n; j++)
			{
				PGnd_ = PGnd + 3 * j;

				eGnd = RVLDOTPRODUCT3(NS, PGnd_) - dSmax;

				if (eGnd > gndCTIThr)
					break;
			}

			if (j < iVertexArray.n)
				continue;
		}
		else if (RVLDOTPRODUCT3(P, NS) >= 0.0f)
			continue;

		RVLDIF3VECTORS(P, Pc, P_);

		dSmax = RVLDOTPRODUCT3(NS, P_);

		if (bRot)
		{
			RVLCROSSPRODUCT3(P_, NS, J);
		}

		iActiveFeature = pParallelDescriptorComponent->idxArray.Element[0];

		dMmax = d[iActiveFeature];

		for (j = 1; j < pParallelDescriptorComponent->idxArray.n; j++)
		{
			l = pParallelDescriptorComponent->idxArray.Element[j];

			if (d[l] > dMmax)
			{
				dMmax = d[l];

				iActiveFeature = l;
			}
		}

		e = dSmax - dMmax;

		if (A)
		{
			M_ = M.Element + iActiveFeature;

			for (j = 0; j < nq; j++)
				Jq[j] = M_[j * M.w];

			for (l = 0; l < nx; l++)
				for (j = l; j < nx; j++)
					A[l * nx + j] += (alpha * J[l] * J[j]);

			for (j = 0; j < nx; j++)
				b[j] += (alpha * J[j] * e);
		}

		E = E + alpha * e * e;
	} // for every parallel descriptor component

	return E;
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

				// if (iFNode == 15)
				//	int debug = 0;

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

						while (pFEdgePtr) // For every neighbor of iFNode
						{
							iFNode_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pFEdgePtr);

							// if (iFNode_ == 15)
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
						} // for every neighbor of iFNode
					}
				}
			} // while queue is not empty

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
		} // while (100 * nRemainingPts / nPts > 5)

		pCorrespArray = correspondences.Element + iCluster;

		pCorrespArray->n = iSCluster;
		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, VN_::Correspondence2, pCorrespArray->n, pCorrespArray->Element);

		QLIST::CopyToArray<VN_::Correspondence2>(pCorrespList, pCorrespArray);

		BubbleSort<VN_::Correspondence2>(correspondences.Element[iCluster], true);
	} // for each cluster

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
				// else
				//	int debug = 0;
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

	// FILE *fp = fopen("sampledMesh.txt", "w");

	// for (i = 0; i < iPtArray.n; i++)
	//{
	//	P = pMesh->NodeArray.Element[iPtArray.Element[i]].P;

	//	fprintf(fp, "%f\t%f\t%f\n", P[0], P[1], P[2]);
	//}

	// fclose(fp);

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
			} // for (iSCluster = 0; iSCluster < SClusters.n; iSCluster++)
		}	  // if (pCorresp->iLevel < maxLevel)
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
	Array<RECOG::SceneCluster> &SClusters_,
	Box<float> boundingBox,
	float *dS,
	bool *bdS,
	bool bValidateDescriptorComponents)
// Mesh *pMesh,
// SurfelGraph *pSurfels,
// Array<RECOG::PSGM_::Cluster *> SCClusters,
// Array<RECOG::PSGM_::Cluster *> SUClusters,
// Box<float> boundingBox,
// RECOG::VN_::Parameters params,
// CRVLMem *pMem,
// float *dS,
// bool *bdS)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	SurfelGraph *pSurfels = pClassifier->pSurfels;

	float *PArray = sceneObject.vertexArray;
	float *NArray = sceneObject.NArray;
	float *R = sceneObject.R;
	float *t = sceneObject.t;

	// nMClusters <- number of clusters in the VN model

	bool bTorus, bConcavity;

	int nMClusters = ClusterTypes(bConcavity, bTorus);

	// If no clusters, then exit.

	if (nMClusters == 0)
		return;

	// size <- mesh size

	float size = GetMeshSize(boundingBox);

	float maxDeviation = pClassifier->kMaxMatchCost * size;

	float tangentDistanceTolerance = pClassifier->kTangentDistanceTolerance * size;

	float *P;

	//// Sample the mesh surface and transform the sampled points using R and t.

	// int nSamplePts = 300;

	// Array<int> iPtArray;

	// iPtArray.n = pMesh->NodeArray.n;

	// RandomIndices(iPtArray);

	// float *PSampleArray = new float[3 * nSamplePts];

	// P = PSampleArray;

	// int i;
	// float *P_;

	// for (i = 0; i < nSamplePts; i++, PSampleArray += 3)
	//{
	//	P_ = pMesh->NodeArray.Element[iPtArray.Element[i]].P;

	//	RVLTRANSF3(P_, R, t, P);
	//}

	// Detect toroidal clusters.

	Array<RECOG::VN_::Torus *> STClusters;

	STClusters.Element = NULL;
	STClusters.n = 0;

	RECOG::VN_::ModelCluster *pMCluster;

	if (bTorus)
	{
		float axis[] = {0.0f, 0.0f, 1.0f};

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

	int nSTClusters = (bTorus ? RVLMIN(STClusters.n, pClassifier->maxnSTClusters) : 0);

	// int nSCClusters = RVLMIN(SCClusters.n, pClassifier->maxnSCClusters);

	// int nSUClusters = (bConcavity ? RVLMIN(SUClusters.n, pClassifier->maxnSUClusters) : 0);

	// SClusters_.n = nSCClusters + nSUClusters + nSTClusters;

	// SClusters_.Element = new RECOG::SceneCluster[SClusters_.n];

	// RECOG::SceneCluster *pSCluster = SClusters_.Element;

	// bool *bCovered = new bool[pSurfels->NodeArray.n];

	// memset(bCovered, 0, pSurfels->NodeArray.n * sizeof(bool));

	// bool *bCopied[2];

	// bCopied[0] = new bool[SCClusters.n];

	// memset(bCopied[0], 0, SCClusters.n * sizeof(bool));

	// if (SUClusters.n > 0)
	//{
	//	bCopied[1] = new bool[SUClusters.n];

	//	memset(bCopied[1], 0, SUClusters.n * sizeof(bool));
	//}
	// else
	//	bCopied[1] = NULL;

	// int iSCluster__[2];

	// iSCluster__[0] = iSCluster__[1] = 0;

	// Array<RECOG::PSGM_::Cluster *> SClusterArray[2];

	// SClusterArray[0] = SCClusters;
	// SClusterArray[1] = SUClusters;

	// int iLargestCluster[2];

	// int iFirstArray = 0;
	// int iLastArray = (nSUClusters ? 1 : 0);

	// int *clusterMap[2];

	// clusterMap[0] = pClassifier->convexClustering.clusterMap;
	// clusterMap[1] = pClassifier->concaveClustering.clusterMap;

	// int nSClusters[2];

	// nSClusters[0] = nSCClusters;
	// nSClusters[1] = nSUClusters;

	// RECOG::PSGM_::Cluster *pCopiedCluster;
	// int i, j;
	// int iSurfel, iCluster;
	// int iClusterArray;
	// int largestClusterOrig;

	// while (iSCluster__[0] < nSCClusters || iSCluster__[1] < nSUClusters)
	//{
	//	for (i = iFirstArray; i <= iLastArray; i++)
	//	{
	//		for (iCluster = 0; iCluster < SClusterArray[i].n; iCluster++)
	//			if (!bCopied[i][iCluster])
	//				break;

	//		iLargestCluster[i] = iCluster;
	//
	//		largestClusterOrig = SClusterArray[i].Element[iCluster]->orig;

	//		iCluster++;

	//		for (; iCluster < SClusterArray[i].n; iCluster++)
	//			if (!bCopied[i][iCluster])
	//				if (SClusterArray[i].Element[iCluster]->orig > largestClusterOrig)
	//				{
	//					iLargestCluster[i] = iCluster;
	//
	//					largestClusterOrig = SClusterArray[i].Element[iCluster]->orig;
	//				}
	//	}

	//	if (iFirstArray < iLastArray)
	//		iClusterArray = (SClusterArray[0].Element[iLargestCluster[0]]->orig >= SClusterArray[1].Element[iLargestCluster[1]]->orig ? 0 : 1);
	//	else
	//		iClusterArray = iFirstArray;

	//	pCopiedCluster = SClusterArray[iClusterArray].Element[iLargestCluster[iClusterArray]];

	//	pSCluster = SClusters_.Element + iClusterArray * nSCClusters + iSCluster__[iClusterArray];

	//	pSCluster->type = (iClusterArray == 0 ? RVLVN_CLUSTER_TYPE_CONVEX : RVLVN_CLUSTER_TYPE_CONCAVE);
	//	pSCluster->vpCluster = pCopiedCluster;

	//	for (i = 0; i < pCopiedCluster->iSurfelArray.n; i++)
	//	{
	//		iSurfel = pCopiedCluster->iSurfelArray.Element[i];
	//
	//		if (!bCovered[iSurfel])
	//		{
	//			for (j = iFirstArray; j <= iLastArray; j++)
	//			{
	//				iCluster = clusterMap[j][iSurfel];

	//				if (!bCopied[j][iCluster])
	//					SClusterArray[j].Element[iCluster]->orig -= pSurfels->NodeArray.Element[iSurfel].size;
	//			}

	//			bCovered[iSurfel] = true;
	//		}
	//	}

	//	bCopied[iClusterArray][iLargestCluster[iClusterArray]] = true;
	//
	//	iSCluster__[iClusterArray]++;

	//	if (iSCluster__[iClusterArray] >= nSClusters[iClusterArray])
	//		iFirstArray = iLastArray = 1 - iClusterArray;
	//}

	// delete[] bCovered;
	// delete[] bCopied[0];
	// RVL_DELETE_ARRAY(bCopied[1]);

	RECOG::SceneCluster *pSCluster = SClusters_.Element + SClusters_.n;

	// for (iCluster = 0; iCluster < nSCClusters; iCluster++, pSCluster++)
	//{
	//	pSCluster->type = RVLVN_CLUSTER_TYPE_CONVEX;
	//	pSCluster->vpCluster = SCClusters.Element + iCluster;
	// }

	// for (iCluster = 0; iCluster < nSUClusters; iCluster++, pSCluster++)
	//{
	//	pSCluster->type = RVLVN_CLUSTER_TYPE_CONCAVE;
	//	pSCluster->vpCluster = SUClusters.Element + iCluster;
	// }

	int iCluster;

	if (bTorus)
	{
		for (iCluster = 0; iCluster < nSTClusters; iCluster++, pSCluster++)
		{
			pSCluster->type = RVLVN_CLUSTER_TYPE_XTORUS;
			pSCluster->vpCluster = STClusters.Element[iCluster];
		}
	}

	SClusters_.n += nSTClusters;

	// Compute descriptor valuses for all correspondences (scene cluster, model cluster).

	float *dS_ = new float[SClusters_.n * featureArray.n];

	bool *bdS_ = new bool[SClusters_.n * featureArray.n];

	memset(bdS_, 0, SClusters_.n * featureArray.n * sizeof(bool));

	Array<float *> descriptors;

	descriptors.Element = new float *[SClusters_.n];
	descriptors.n = SClusters_.n;

	int j;
	int iSCluster, iFeature, iVertex;
	float *N;
	PSGM_::Cluster *pSCCluster, *pSUCluster;
	VN_::Torus *pTCluster;
	float d, maxd, mind;
	float *dS__;
	bool *bdS__;
	VN_::Feature *pFeature;
	VN_::TorusRing *pTorusRing;
	bool bd, bdmax, bdmaxValid;

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
				switch (pSCluster->type)
				{
				case RVLVN_CLUSTER_TYPE_CONVEX:
					pSCCluster = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

					for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
					{
						N = featureArray.Element[iFeature].N;

						pSurfels->ComputeTangent(pSCCluster->iVertexArray, PArray, N, 1.0f, false, false, pClassifier->tangentOrientationTolerance, tangentDistanceTolerance,
												 dS__[iFeature], bd, iVertex);

						bdS__[iFeature] = true;
					}

					break;
				case RVLVN_CLUSTER_TYPE_CONCAVE:
					pSUCluster = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

					for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
					{
						N = featureArray.Element[iFeature].N;

						pSurfels->ComputeTangent(pSUCluster->iVertexArray, PArray, N, -1.0f, false, false, pClassifier->tangentOrientationTolerance, tangentDistanceTolerance,
												 dS__[iFeature], bd, iVertex);

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

	VN_::Correspondence4 *pBestCorresp = NULL;

	VN_::Correspondence4 *pCorresp_;
	int iSCluster_;
	RECOG::SceneCluster *pSCluster_;
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

			// int debugMClusterID[] = {-1, -1, -1, -1, -1, -1, 0, -1};

			// int iDebug = 0;

			// VN_::Correspondence4 *pDebugCorresp = pCorresp;

			// while (pDebugCorresp)
			//{
			//	if (pDebugCorresp->iMCluster != debugMClusterID[iDebug++])
			//		break;

			//	pDebugCorresp = pDebugCorresp->pParent;
			//}

			// if (pDebugCorresp == NULL)
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

				if (fpDebug)
				{
					VN_::Correspondence4 *pDebugCorresp = pCorresp;

					while (pDebugCorresp)
					{
						fprintf(fpDebug, "(%d %d) ", pDebugCorresp->iMCluster, pDebugCorresp->iSCluster);

						pDebugCorresp = pDebugCorresp->pParent;
					}

					fprintf(fpDebug, "%f\n", e);
				}

				if (mine < 0.0f || e < mine)
				{
					mine = e;

					memcpy(dS, dSEval, featureArray.n * sizeof(float));
					memcpy(bdS, bdSEval, featureArray.n * sizeof(bool));
					pBestCorresp = pCorresp;
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

	score = mine;
	/// Validate descriptor components.

	// nMClusters <- number of model clusters

	nMClusters = 0;

	pMCluster = modelClusterList.pFirst;

	while (pMCluster)
	{
		if (pMCluster->ID > nMClusters)
			nMClusters = pMCluster->ID;

		pMCluster = pMCluster->pNext;
	}

	nMClusters++;

	// Create vertex and surfel sets for model clusters.

	int *vertexIdxMem = new int[nMClusters * pSurfels->vertexArray.n];
	bool *vertexMarkMem = new bool[nMClusters * pSurfels->vertexArray.n];
	int *surfelIdxMem = new int[nMClusters * pSurfels->NodeArray.n];
	bool *surfelMarkMem = new bool[nMClusters * pSurfels->NodeArray.n];

	memset(vertexMarkMem, 0, nMClusters * pSurfels->vertexArray.n * sizeof(bool));
	memset(surfelMarkMem, 0, nMClusters * pSurfels->NodeArray.n * sizeof(bool));

	Array<int> *iVertexArrayArray = new Array<int>[nMClusters];
	bool **bVertexAlreadyConsidered = new bool *[nMClusters];
	Array<int> *iSurfelArrayArray = new Array<int>[nMClusters];
	bool **bSurfelAlreadyConsidered = new bool *[nMClusters];

	for (iCluster = 0; iCluster < nMClusters; iCluster++)
	{
		iVertexArrayArray[iCluster].n = 0;
		iVertexArrayArray[iCluster].Element = vertexIdxMem + iCluster * pSurfels->vertexArray.n;
		bVertexAlreadyConsidered[iCluster] = vertexMarkMem + iCluster * pSurfels->vertexArray.n;
		iSurfelArrayArray[iCluster].n = 0;
		iSurfelArrayArray[iCluster].Element = surfelIdxMem + iCluster * pSurfels->NodeArray.n;
		bSurfelAlreadyConsidered[iCluster] = surfelMarkMem + iCluster * pSurfels->NodeArray.n;
	}

	pCorresp = pBestCorresp;

	int i;
	PSGM_::Cluster *pSCluster__;
	int iSurfel;

	while (pCorresp)
	{
		if (pCorresp->iSCluster >= 0)
		{
			pSCluster_ = SClusters_.Element + pCorresp->iSCluster;

			pMCluster = modelClusterList.pFirst;

			while (pMCluster)
			{
				if (pMCluster->ID == pCorresp->iMCluster)
					break;

				pMCluster = pMCluster->pNext;
			}

			if (pMCluster)
			{
				if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pMCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
				{
					pSCluster__ = (PSGM_::Cluster *)(pSCluster_->vpCluster);

					for (i = 0; i < pSCluster__->iVertexArray.n; i++)
					{
						iVertex = pSCluster__->iVertexArray.Element[i];

						if (!bVertexAlreadyConsidered[pMCluster->ID][iVertex])
						{
							iVertexArrayArray[pMCluster->ID].Element[iVertexArrayArray[pMCluster->ID].n++] = iVertex;
							bVertexAlreadyConsidered[pMCluster->ID][iVertex] = true;
						}
					}

					for (i = 0; i < pSCluster__->iSurfelArray.n; i++)
					{
						iSurfel = pSCluster__->iSurfelArray.Element[i];

						if (!bSurfelAlreadyConsidered[pMCluster->ID][iSurfel])
						{
							iSurfelArrayArray[pMCluster->ID].Element[iSurfelArrayArray[pMCluster->ID].n++] = iSurfel;
							bSurfelAlreadyConsidered[pMCluster->ID][iSurfel] = true;
						}
					}
				}
			}
		}

		pCorresp = pCorresp->pParent;
	}

	// NHull <- convex hull of normals of all surfels grouped in a particular model cluster.

	// Array<SURFEL::NormalHullElement> *NHull = new Array<SURFEL::NormalHullElement>[nMClusters];

	// SURFEL::NormalHullElement *NHullMem = new SURFEL::NormalHullElement[nMClusters * pSurfels->NodeArray.n];

	// Surfel *pSurfel;

	// pMCluster = modelClusterList.pFirst;

	// while (pMCluster)
	//{
	//	NHull[pMCluster->ID].Element = NHullMem + pMCluster->ID * pSurfels->NodeArray.n;
	//	NHull[pMCluster->ID].n = 0;

	//	for (i = 0; i < iSurfelArrayArray[pMCluster->ID].n; i++)
	//	{
	//		iSurfel = iSurfelArrayArray[pMCluster->ID].Element[i];

	//		pSurfel = pSurfels->NodeArray.Element + iSurfel;

	//		pSurfels->UpdateNormalHull(NHull[pMCluster->ID], pSurfel->N);
	//	}

	//	pMCluster = pMCluster->pNext;
	//}

	// Validate descriptor components using normal hulls of surfels.

	// memset(bdS, 0, featureArray.n * sizeof(bool));

	// float distFromNHull;

	// pMCluster = modelClusterList.pFirst;

	// while (pMCluster)
	//{
	//	for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
	//	{
	//		N = featureArray.Element[iFeature].N;

	//		distFromNHull = pSurfels->DistanceFromNormalHull(NHull[pMCluster->ID], N);

	//		bdS[iFeature] = (distFromNHull <= pClassifier->tangentOrientationTolerance);
	//	}

	//	pMCluster = pMCluster->pNext;
	//}

	// Validate descriptor components using normal hulls of vertices.

	Array<SURFEL::NormalHullElement> *NHull = new Array<SURFEL::NormalHullElement>[pSurfels->vertexArray.n];

	SURFEL::NormalHullElement *NHullMem = new SURFEL::NormalHullElement[3 * pSurfels->vertexArray.n];

	if (bValidateDescriptorComponents)
	{
		memset(bdS, 0, featureArray.n * sizeof(bool));

		float fTmp;
		SURFEL::Vertex *pVertex;
		float NS[3];

		pMCluster = modelClusterList.pFirst;

		while (pMCluster)
		{
			if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pMCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
			{
				for (i = 0; i < iVertexArrayArray[pMCluster->ID].n; i++)
				{
					iVertex = iVertexArrayArray[pMCluster->ID].Element[i];

					pVertex = pSurfels->vertexArray.Element[iVertex];

					NHull[iVertex].n = 0;

					NHull[iVertex].Element = NHullMem + 3 * iVertex;

					for (j = 0; j < pVertex->iSurfelArray.n; j++)
					{
						iSurfel = pVertex->iSurfelArray.Element[j];

						if (bSurfelAlreadyConsidered[pMCluster->ID][iSurfel])
							pSurfels->UpdateNormalHull(NHull[iVertex], NArray + 3 * iSurfel);
					}
				}
			}

			switch (pMCluster->type)
			{
			case RVLVN_CLUSTER_TYPE_CONVEX:
				for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
				{
					N = featureArray.Element[iFeature].N;

					pSurfels->ComputeTangent(iVertexArrayArray[pMCluster->ID], PArray, NHull, N, 1.0f, true, false, pClassifier->tangentOrientationTolerance,
											 tangentDistanceTolerance, fTmp, bdS[iFeature], iVertex);
				}

				break;
			case RVLVN_CLUSTER_TYPE_CONCAVE:
				for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
				{
					N = featureArray.Element[iFeature].N;

					// if (iFeature == 66+33)
					//	int debug = 0;

					pSurfels->ComputeTangent(iVertexArrayArray[pMCluster->ID], PArray, NHull, N, -1.0f, true, false, pClassifier->tangentOrientationTolerance,
											 tangentDistanceTolerance, fTmp, bdS[iFeature], iVertex);
				}

				break;
			case RVLVN_CLUSTER_TYPE_XTORUS:
				pTCluster = (RECOG::VN_::Torus *)(pSCluster->vpCluster);

				for (iFeature = pMCluster->iFeatureInterval.a; iFeature <= pMCluster->iFeatureInterval.b; iFeature++)
				{
					N = featureArray.Element[iFeature].N;

					RVLMULMX3X3TVECT(R, N, NS);

					pFeature = featureArray.Element + iFeature;

					pTorusRing = pTCluster->ringArray.Element[pFeature->iBeta];

					if (pTorusRing)
						pSurfels->ComputeTangent(pTorusRing->iVertexArray, PArray, NS, -1.0f, true, false, pClassifier->tangentOrientationTolerance,
												 tangentDistanceTolerance, fTmp, bdS[iFeature], iVertex);
				}
			}

			pMCluster = pMCluster->pNext;
		}
	}

	// for (int iF = 0; iF < featureArray.n; iF++)
	//{
	//	bdS[iF] = 1;
	// }

	///

	// Deallocate memory.

	RVL_DELETE_ARRAY(STClusters.Element);
	delete[] dS_;
	delete[] bdS_;
	delete[] dSEval;
	delete[] bdSEval;
	delete[] descriptors.Element;
	delete[] SDF;
	delete[] vertexIdxMem;
	delete[] vertexMarkMem;
	delete[] surfelIdxMem;
	delete[] surfelMarkMem;
	delete[] iVertexArrayArray;
	delete[] bVertexAlreadyConsidered;
	delete[] iSurfelArrayArray;
	delete[] bSurfelAlreadyConsidered;
	delete[] NHull;
	delete[] NHullMem;
}

// The following function detects toroidal clusters in pMesh.
// The algorithm which this function implements is described in APR3D.TR11-B.

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
	// Assign tangents to all vertex edges. (line 1)

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
	SURFEL::VertexEdge *pVEdge;
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

		// if (iVertex == 110)
		//	int debug_ = 0;

		// int debug = 0;

		// int debug__;

		pVEdgePtr = pVertex->EdgeList.pFirst;

		while (pVEdgePtr)
		{
			iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pVEdgePtr);

			if (iVertex_ > iVertex)
			{
				pVEdge = pVEdgePtr->pEdge;

				// if (pVEdge->idx == 75)
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
						a = adn * adn - cb2[iBeta] * dndn;
						b = 2.0f * (adn * an1 - cb2[iBeta] * n1dn);
						c = an1 * an1 - cb2[iBeta];

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
				} // for (iBeta = 0; iBeta < betaArray.n; iBeta++)

				pTangentSet->tangentArray.n = pTangentSet->maxiBeta - pTangentSet->miniBeta + 1;

				RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, VN_::EdgeTangent *, pTangentSet->tangentArray.n, pTangentSet->tangentArray.Element);

				j = 0;

				for (i = pTangentSet->miniBeta; i <= pTangentSet->maxiBeta; i++, j++)
					pTangentSet->tangentArray.Element[j] = tangentArray[i];
			} // if (iVertex_ > iVertex)

			pVEdgePtr = pVEdgePtr->pNext;
		} // while (pVEdgePtr)

		// if (debug == 1)
		//	int debug_ = 0;
	} // for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)

	delete[] cb2;
	delete[] tangentArray;

	// FILE *fp = fopen("tangentRing.txt", "w");

	// iBeta = 2;

	// SURFEL::Vertex *pVertex_;

	// for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
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

	// fclose(fp);

	// Detect torus rings for each beta. The detected rings are stored in ringListArray.

	bool *bEdgeJoined = new bool[pSurfels->vertexEdgeArray.n];

	bool *bVertexJoined = new bool[pSurfels->vertexArray.n];

	memset(bVertexJoined, 0, pSurfels->vertexArray.n * sizeof(bool));

	Array<QList<RECOG::VN_::TorusRing>> ringListArray;

	ringListArray.Element = new QList<RECOG::VN_::TorusRing>[betaArray.n];
	ringListArray.n = betaArray.n;

	QList<RECOG::VN_::TorusRing> *pRingList;

	SURFEL::VertexEdge **VEdgeBuff = new SURFEL::VertexEdge *[pSurfels->vertexEdgeArray.n];

	for (iBeta = 0; iBeta < betaArray.n; iBeta++) // line 4
	{
		pRingList = ringListArray.Element + iBeta;

		RVLQLIST_INIT(pRingList);

		memset(bEdgeJoined, 0, pSurfels->vertexEdgeArray.n * sizeof(bool)); // line 7

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

	// FILE *fp = fopen("tangentRing.txt", "w");

	// iRing = 0;

	// pRingList = ringListArray.Element + 0;

	// RECOG::VN_::TorusRing *pRingDebug = pRingList->pFirst;

	// while (pRingDebug)
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

	// fclose(fp);

	delete[] tangentSet;
	delete[] bEdgeJoined;
	delete[] bVertexJoined;
	delete[] VEdgeBuff;

	// Compute CTI descriptors for all rings. (lines 15-19)
	// For each alpha, the index of the kex vertex, i.e. the vertex defining the tangent corresponding to this alpha
	// is stored in iKeyVertex array of each ring.

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

	// For each pair of rings (pPrevRing, pRing), where pRing is a ring form the ring list of a particular beta
	// and pPrevRing is a ring form the ring list of the previous beta in betaArray, a convexity test is applied.
	// This test checks whether for each alpha, the key vertex of pPrevRing assigned to this alpha is below the tangent
	// of pRing corresponding to the same alpha and opposite.
	// If the pair (pPrevRing, pRing) passes the convexity test, then pPrevRing is added to pRing->compList. (lines 20-24)

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

	/// The following code implements lines 30-35.

	// A queue is created and every ring which is not contained in compList of any other ring is inserted in this queue.

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

	// For each sequence of rings, where each ring in a sequence is contained in compList of any the previous ring in the sequence,
	// create a torus and add it to torusList.

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

	// Copy all toruses from pTorusList to SClusters.

	SClusters.Element = new VN_::Torus *[SClusters.n];

	QLIST::CreatePtrArray<VN_::Torus>(pTorusList, &SClusters);
}

#ifdef NEVER
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
	if (iBeta < tangentSet[pVEdge0->idx].miniBeta || iBeta > tangentSet[pVEdge0->idx].maxiBeta || bEdgeJoined[pVEdge0->idx]) // line 6
		return;

	// Identify the connected set of edges containing pVEdge0 such that all edges in this set have a tangent in the tangent set iBeta.
	// This edge set is stored in VEdgeBuff. (line 10)

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

		for (j = 0; j < 2; j++) // for every endpoint of pVEdge
		{
			pVEdgePtr = pSurfels->vertexArray.Element[pVEdge->iVertex[j]]->EdgeList.pFirst;

			while (pVEdgePtr)
			{
				pVEdge_ = pVEdgePtr->pEdge;

				if (iBeta >= tangentSet[pVEdge_->idx].miniBeta && iBeta <= tangentSet[pVEdge_->idx].maxiBeta && !bEdgeJoined[pVEdge_->idx])
				{
					bEdgeJoined[pVEdge_->idx] = true; // line 11

					*(pVEdgePush++) = pVEdge_;
				} // if (!bJoined[pVEdge_->idx])

				pVEdgePtr = pVEdgePtr->pNext;
			} // for every neighbor edge
		}	  // for (j = 0; j < 2; j++)
	}		  // main loop

	SURFEL::VertexEdge **pVEdgeBuffEnd = pVEdgeFetch;

	int nEdges = pVEdgeBuffEnd - VEdgeBuff;

	if (nEdges < minnEdges) // line 12
		return;

	/// The following code implements line 13.

	// Create the compatibility table bComp indicating compatibility between all edge pairs in VEdgeBuff.
	// bComp(i, j) = true if the both endpoints of the i-th edge in VEdgeBuff are above the tangent of the j-th edge,
	// otherwise bComp(i, j) = false.

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

	// Sort edges according to the compatibility score in descending order.
	// The compatibility score of an edge is computed by summing the lengths of all edges, which are not compatible with this edge
	// and subtracting this sum from the length of the considered edge.
	// The sorted edges are stored in sortedTangentArray.

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

	// Create ringEdgeArray iteratively by considering edges contained in sortedTangentArray one by one, in the order in which they are stored,
	// and adding to ringEdgeArray every edge, which is compatible with all edges already contained in ringEdgeArray.

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

	///

	// If ringEdgeArray.n < minnEdges, then no ring is created.

	// if (ringEdgeArray.n >= minnEdges && ringEdgeArray.n > nEdges / 2)
	if (ringEdgeArray.n >= minnEdges) // line 14
	{
		// If there is an edge in ringEdgeArray such that all other edges lie on its tangent within tolerance maxError,
		// then no ring is created.

		int iTmp;

		for (i = 0; i < ringEdgeArray.n; i++)
		{
			iTmp = ringEdgeArray.Element[i] * nEdges;

			for (j = 0; j < ringEdgeArray.n; j++)
			{
				iComp = iTmp + ringEdgeArray.Element[j];

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
			// Create torus ring and add it to pRingList.

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

					// if (iVertex >= pSurfels->vertexArray.n)
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
		} // if (i >= ringEdgeArray.n)
	}	  // if (ringEdgeArray.n >= minnEdges)

	// if (iBeta == 2)
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
#endif

void VN::Project(
	float *d,
	float *R,
	float *t,
	Camera camera,
	Array2D<float> imgPtArray,
	Array<OrientedPoint> PtArray,
	float *Z,
	float *pMinZ)
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

	float minz = 1e6;

	VN_::SurfaceRayIntersection surfaceRayIntersectionEnd;

	VN_::SurfaceRayIntersection *pSurfaceRayIntersectionEnd = (Z ? &surfaceRayIntersectionEnd : NULL);

	int iPt;
	float *m;
	OrientedPoint *pPt;
	float r[3], rM[3];
	float fTmp;
	float s;
	VN_::SurfaceRayIntersection surfaceRayIntersection;
	float z;
	float P[3];

	for (iPt = 0; iPt < imgPtArray.h; iPt++)
	{
		// if (iPt == 325)
		//	int debug = 0;

		m = imgPtArray.Element + imgPtArray.w * iPt;

		r[0] = (m[0] - camera.uc) / camera.fu;
		r[1] = (m[1] - camera.vc) / camera.fv;
		r[2] = 1.0f;

		RVLNORM3(r, fTmp);

		RVLMULMX3X3TVECT(R, r, rM);

		RVLNORM3(rM, fTmp);

		surfaceRayIntersection = Project(dc, rM, pSurfaceRayIntersectionEnd);

		pPt = PtArray.Element + iPt;

		s = surfaceRayIntersection.s * s_;

		// if (s > 0.0f && s < 2000.0f)
		//	int debug = 0;

		RVLSCALE3VECTOR(r, s, pPt->P);

		N = featureArray.Element[surfaceRayIntersection.iFeature].N;

		RVLMULMX3X3VECT(R, N, pPt->N);

		if (Z)
		{
			if (surfaceRayIntersectionEnd.s > -1e6 && surfaceRayIntersectionEnd.s < 1e6)
			{
				s = surfaceRayIntersectionEnd.s * s_;

				RVLSCALE3VECTOR(r, s, P);

				z = RVLDOTPRODUCT3(Z, P);

				if (z < minz)
					minz = z;
			}
		}
	}

	if (pMinZ)
		*pMinZ = minz;
}

VN_::SurfaceRayIntersection VN::Project(
	float *d,
	float *r,
	RECOG::VN_::SurfaceRayIntersection *pSurfaceRayIntersectionEnd,
	bool *bd)
{
	int iNode;
	VN_::Node *pNode;
	Array<Pair<VN_::SurfaceRayIntersection, VN_::SurfaceRayIntersection>> *pProjectionInterval;
	float *N;
	float d_;
	float c;
	Pair<VN_::SurfaceRayIntersection, VN_::SurfaceRayIntersection> *pInterval;

	for (iNode = 0; iNode < featureArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pProjectionInterval = projectionIntervals.Element + iNode;

		pInterval = pProjectionInterval->Element;

		N = pNode->pFeature->N;

		c = RVLDOTPRODUCT3(N, r);

		if (c < -1e-6)
		{
			pInterval->a.s = d[iNode] / c;
			pInterval->a.iFeature = iNode;
			pInterval->b.s = 1e6;
			pInterval->b.iFeature = -1;
			pProjectionInterval->n = 1;
		}
		else if (c < 1e-6)
		{
			if (d[iNode] >= 0.0f)
			{
				pInterval->a.s = -1e6;
				pInterval->a.iFeature = -1;
				pInterval->b.s = 1e6;
				pInterval->b.iFeature = -1;
				pProjectionInterval->n = 1;
			}
			else
				pProjectionInterval->n = 0;
		}
		else
		{
			pInterval->a.s = -1e6;
			pInterval->a.iFeature = -1;
			pInterval->b.s = d[iNode] / c;
			pInterval->b.iFeature = iNode;
			pProjectionInterval->n = 1;
		}
	}

	for (; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pProjectionInterval = projectionIntervals.Element + iNode;

		if (pNode->operation > 0)
		{
			pInterval = pProjectionInterval->Element;
			pInterval->a.s = -1e6;
			pInterval->a.iFeature = -1;
			pInterval->b.s = 1e6;
			pInterval->b.iFeature = -1;
			pProjectionInterval->n = 1;
		}
		else
			pProjectionInterval->n = 0;
	}

	VN_::Edge *pEdge = EdgeList.pFirst;

	VN_::Node *pParentNode;
	Array<Pair<VN_::SurfaceRayIntersection, VN_::SurfaceRayIntersection>> *pChildInterval, *pParentInterval;
	int i, iPrev, j;
	int bOpen[2];
	int iInterval[2];
	int iNewInterval;
	VN_::SurfaceRayIntersection nextPt[2];
	int iNext;
	bool b1, b2;
	int nb, nbPrev;
	int nOpen, nClosed;

	while (pEdge)
	{
		// if (pEdge->data.b == 181)
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
				iNext = (nextPt[0].s <= nextPt[1].s ? 0 : 1);
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
	} // while (pEdge)

	if (projectionIntervals.Element[iy].n > 0)
	{
		if (pSurfaceRayIntersectionEnd)
			*pSurfaceRayIntersectionEnd = projectionIntervals.Element[iy].Element[projectionIntervals.Element[iy].n - 1].b;

		return projectionIntervals.Element[iy].Element[0].a;
	}
	else
	{
		VN_::SurfaceRayIntersection surfaceRayIntersection;

		surfaceRayIntersection.s = 1e6;
		surfaceRayIntersection.iFeature = -1;

		if (pSurfaceRayIntersectionEnd)
			*pSurfaceRayIntersectionEnd = surfaceRayIntersection;

		return surfaceRayIntersection;
	}
}

Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>> *VN::VolumeCylinderIntersection(
	float *d,
	float *P1,
	float *P2,
	float r)
{
	float V[3];
	RVLDIF3VECTORS(P2, P1, V);
	float l = sqrt(RVLDOTPRODUCT3(V, V));
	RVLSCALE3VECTOR2(V, l, V);

	int iNode;
	VN_::Node *pNode;
	Array<Pair<VN_::SurfaceRayIntersection, VN_::SurfaceRayIntersection>> *pProjectionInterval;
	float *N;
	float d_;
	float c;
	Pair<VN_::SurfaceRayIntersection, VN_::SurfaceRayIntersection> *pInterval;
	float s0;

	for (iNode = 0; iNode < featureArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pProjectionInterval = projectionIntervals.Element + iNode;

		pInterval = pProjectionInterval->Element;

		N = pNode->pFeature->N;

		c = RVLDOTPRODUCT3(N, V);

		if (c < -1e-6)
		{
			s0 = -(RVLDOTPRODUCT3(N, P1) - d[iNode] - r) / c;
			if (s0 <= l)
			{
				pInterval->a.s = s0;
				pInterval->a.iFeature = iNode;
				pInterval->b.s = 1e6;
				pInterval->b.iFeature = -1;
				pProjectionInterval->n = 1;
			}
			else
				pProjectionInterval->n = 0;
		}
		else if (c < 1e-6)
		{
			if (d[iNode] >= 0.0f)
			{
				pInterval->a.s = -1e6;
				pInterval->a.iFeature = -1;
				pInterval->b.s = 1e6;
				pInterval->b.iFeature = -1;
				pProjectionInterval->n = 1;
			}
			else
				pProjectionInterval->n = 0;
		}
		else
		{
			s0 = -(RVLDOTPRODUCT3(N, P1) - d[iNode] - r) / c;
			if (s0 >= 0.0f)
			{
				pInterval->a.s = -1e6;
				pInterval->a.iFeature = -1;
				pInterval->b.s = s0;
				pInterval->b.iFeature = iNode;
				pProjectionInterval->n = 1;
			}
			else
				pProjectionInterval->n = 0;
		}
	}

	for (; iNode < NodeArray.n; iNode++)
	{
		pNode = NodeArray.Element + iNode;

		pProjectionInterval = projectionIntervals.Element + iNode;

		if (pNode->operation > 0)
		{
			pInterval = pProjectionInterval->Element;
			pInterval->a.s = -1e6;
			pInterval->a.iFeature = -1;
			pInterval->b.s = 1e6;
			pInterval->b.iFeature = -1;
			pProjectionInterval->n = 1;
		}
		else
			pProjectionInterval->n = 0;
	}

	VN_::Edge *pEdge = EdgeList.pFirst;

	VN_::Node *pParentNode;
	Array<Pair<VN_::SurfaceRayIntersection, VN_::SurfaceRayIntersection>> *pChildInterval, *pParentInterval;
	int i, iPrev, j;
	int bOpen[2];
	int iInterval[2];
	int iNewInterval;
	VN_::SurfaceRayIntersection nextPt[2];
	int iNext;
	bool b1, b2;
	int nb, nbPrev;
	int nOpen, nClosed;

	while (pEdge)
	{
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
				iNext = (nextPt[0].s <= nextPt[1].s ? 0 : 1);
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
	} // while (pEdge)

	return projectionIntervals.Element + iy;
}

void VN::Transform(
	float *dSrc,
	float *R,
	float *t,
	float *dTgt)
{
	float s;
	float R_[9];

	RVLEXTRACTSCALEFROMROT(R, s, R_);

	int i;
	float *NM;
	float NS[3];

	for (i = 0; i < featureArray.n; i++)
	{
		NM = featureArray.Element[i].N;

		RVLMULMX3X3VECT(R_, NM, NS);

		dTgt[i] = s * dSrc[i] + RVLDOTPRODUCT3(NS, t);
	}
}

void VN::BoundingBox(
	float *d,
	Box<float> &box)
{
	Box<bool> bbox;

	bbox.minx = bbox.maxx = bbox.miny = bbox.maxy = bbox.minz = bbox.maxz = false;

	int iFeature;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		if (featureArray.Element[iFeature].N[0] < -1.0f + 1e-3)
		{
			if (bbox.minx)
			{
				if (-d[iFeature] < box.minx)
					box.minx = -d[iFeature];
			}
			else
			{
				box.minx = -d[iFeature];
				bbox.minx = true;
			}
		}
		else if (featureArray.Element[iFeature].N[0] > 1.0f - 1e-3)
		{
			if (bbox.maxx)
			{
				if (d[iFeature] > box.maxx)
					box.maxx = d[iFeature];
			}
			else
			{
				box.maxx = d[iFeature];
				bbox.maxx = true;
			}
		}
		else if (featureArray.Element[iFeature].N[1] < -1.0f + 1e-3)
		{
			if (bbox.miny)
			{
				if (-d[iFeature] < box.miny)
					box.miny = -d[iFeature];
			}
			else
			{
				box.miny = -d[iFeature];
				bbox.miny = true;
			}
		}
		else if (featureArray.Element[iFeature].N[1] > 1.0f - 1e-3)
		{
			if (bbox.maxy)
			{
				if (d[iFeature] > box.maxy)
					box.maxy = d[iFeature];
			}
			else
			{
				box.maxy = d[iFeature];
				bbox.maxy = true;
			}
		}
		else if (featureArray.Element[iFeature].N[2] < -1.0f + 1e-3)
		{
			if (bbox.minz)
			{
				if (-d[iFeature] < box.minz)
					box.minz = -d[iFeature];
			}
			else
			{
				box.minz = -d[iFeature];
				bbox.minz = true;
			}
		}
		else if (featureArray.Element[iFeature].N[2] > 1.0f - 1e-3)
		{
			if (bbox.maxz)
			{
				if (d[iFeature] > box.maxz)
					box.maxz = d[iFeature];
			}
			else
			{
				box.maxz = d[iFeature];
				bbox.maxz = true;
			}
		}
	}
}

void VN::CreateFromConvexMesh(
	Mesh *pMesh,
	float *&d,
	CRVLMem *pMem)
{
	Array<PSGM_::Plane> convexTemplate;

	convexTemplate.n = pMesh->faces.n;
	convexTemplate.Element = new PSGM_::Plane[convexTemplate.n];

	d = new float[convexTemplate.n];

	PSGM_::Plane *pPlane = convexTemplate.Element;

	int i;
	MESH::Face *pFace;

	for (i = 0; i < pMesh->faces.n; i++, pPlane++)
	{
		pFace = pMesh->faces.Element[i];

		RVLCOPY3VECTOR(pFace->N, pPlane->N);

		d[i] = pFace->d;
	}

	VN_::CreateConvex(this, convexTemplate, pMem);

	delete[] convexTemplate.Element;
}

float VN::SceneFittingScore(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Camera camera,
	Array<int> iSurfelArray,
	Array<int> iVertexArray,
	float *d,
	float maxe,
	float maxz,
	float cThr,
	int &nSamples,
	float &chamferDist,
	int &nOutliers,
	bool bVisualize)
{
	SURFEL::SceneSamples sceneSamples;

	sceneSamples.PGnd = NULL;

	pSurfels->SampleSurfelSet(pMesh, iSurfelArray, iVertexArray, camera, sceneSamples, false, false);

	Array<OrientedPoint> PtArray;

	PtArray.n = sceneSamples.imagePtArray.h;

	PtArray.Element = new OrientedPoint[PtArray.n];

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	CRVLTimer timer;

	timer.Start();

	Project(d, R, t, camera, sceneSamples.imagePtArray, PtArray);

	timer.Stop();

	double tProjectVN = timer.GetTime();

	float *SDF = new float[featureArray.n];

	int i, iActiveFeature;
	float *P;

	for (i = 0; i < sceneSamples.PtArray.n; i++)
	{
		if (sceneSamples.status[i] != 1)
			continue;

		P = sceneSamples.PtArray.Element[i];

		sceneSamples.SDF[i] = Evaluate(P, SDF, iActiveFeature, true, d);
	}

	delete[] SDF;

	timer.Start();

	float sceneSupport = RECOG::VN_::SceneFittingScore2(sceneSamples, PtArray, maxe, maxz,
														cThr, nSamples, chamferDist, nOutliers, bVisualize);

	delete[] PtArray.Element;

	SURFEL::DeleteSceneSamples(sceneSamples);

	timer.Stop();

	double tComputeScore = timer.GetTime();

	return sceneSupport;
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
	float kResolution,
	float *dIn,
	bool *bd,
	float SDFSurfaceValue)
{
	float *d = new float[featureArray.n];

	int i;

	if (dIn)
		memcpy(d, dIn, featureArray.n * sizeof(float));
	else
		for (i = 0; i < featureArray.n; i++)
			d[i] = featureArray.Element[i].d;

	Box<float> box;

	BoundingBox(d, box);

	float resolution = kResolution * BoxSize(&box);

	ExpandBox<float>(&box, 10.0f * resolution);

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

	int j, k;
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
	delete[] d;
}

void VN::Display(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	Array<int> iSurfelArray,
	Array<int> iVertexArray,
	float *d,
	float *R,
	uchar *color,
	Camera camera)
{
	SURFEL::SceneSamples sceneSamples;

	pSurfels->SampleSurfelSet(pMesh, iSurfelArray, iVertexArray, camera, sceneSamples);

	Array<OrientedPoint> PtArray;

	PtArray.n = sceneSamples.imagePtArray.h;

	PtArray.Element = new OrientedPoint[PtArray.n];

	float t[3];

	RVLNULL3VECTOR(t);

	Project(d, R, t, camera, sceneSamples.imagePtArray, PtArray);

	Array<Point> PtArray_;

	PtArray_.Element = new Point[PtArray.n];

	MESH::CreatePointArrayFromOrientedPointArray(PtArray, PtArray_, 2.0f);

	pVisualizer->DisplayPointSet<float, Point>(PtArray_, color, 6.0f);

	delete[] PtArray.Element;
	delete[] PtArray_.Element;
	SURFEL::DeleteSceneSamples(sceneSamples);
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

void VN::SaveFeatures(FILE *fp)
{
	int iFeature;
	float *N;

	for (iFeature = 0; iFeature < featureArray.n; iFeature++)
	{
		N = featureArray.Element[iFeature].N;

		fprintf(fp, "%f\t%f\t%f\n", N[0], N[1], N[2]);
	}
}

void VN_::CreateConvex(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

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
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	Pair<int, int> iBetaInterval;

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
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	Pair<int, int> iAlphaInterval;

	iAlphaInterval.a = 9;
	iAlphaInterval.b = 13;

	Pair<int, int> iBetaInterval;

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
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	betaInterval.a = 0.0f;
	betaInterval.b = 0.5f * PI;

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.5f);

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.1f, convexTemplate, betaInterval, NArray, pMem);

	int iN;
	float *N;

	for (iN = 0; iN < pMCluster0->NArray.h; iN++)
	{
		N = pMCluster0->NArray.Element + pMCluster0->NArray.w * iN;

		if (N[2] < -1e-3)
			pVN->AddLimit(0, 0, 0, iN, 1, pMem);
	}

	pVN->AddOperation(2, -1, 0, 1, pMem);

	pVN->SetOutput(2);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.6f;
}

void VN_::CreateHammer(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	betaInterval.a = 0.5f * PI;
	betaInterval.b = PI;

	RVLSET3VECTOR(t, 0.0f, 0.0f, -0.5f);

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.1f, convexTemplate, betaInterval, NArray, pMem);

	int iN;
	float *N;

	for (iN = 0; iN < pMCluster0->NArray.h; iN++)
	{
		N = pMCluster0->NArray.Element + pMCluster0->NArray.w * iN;

		if (N[2] > 1e-3)
			pVN->AddLimit(0, 0, 0, iN, 1, pMem);
	}

	pVN->AddOperation(2, -1, 0, 1, pMem);

	pVN->SetOutput(2);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.6f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateBowl(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	betaInterval.a = 0.0f;
	betaInterval.b = 0.5f * PI + 1e-3;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.3f, convexTemplate, betaInterval, NArray, pMem);

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

void VN_::CreateBowl2(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.3f, convexTemplate, betaInterval, NArray, pMem);

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

void VN_::CreateTwoConvex(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

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

void VN_::CreateMug2(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.5f * PI - 1e-3;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 1;

	float N0[3];

	RVLSET3VECTOR(N0, 0.0f, 0.0f, 1.0f);

	NArray.Element = N0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	betaInterval.a = 0.0f;
	betaInterval.b = 0.5f * PI + 1e-3;

	NArray.h = 0;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.3f, convexTemplate, betaInterval, NArray, pMem);

	RVLMXEL(R, 3, 0, 0) = 0.0f;
	RVLMXEL(R, 3, 1, 0) = 0.0f;
	RVLMXEL(R, 3, 2, 0) = 1.0f;

	RVLMXEL(R, 3, 0, 1) = -1.0f;
	RVLMXEL(R, 3, 1, 1) = 0.0f;
	RVLMXEL(R, 3, 2, 1) = 0.0f;

	RVLMXEL(R, 3, 0, 2) = 0.0f;
	RVLMXEL(R, 3, 1, 2) = -1.0f;
	RVLMXEL(R, 3, 2, 2) = 0.0f;

	Pair<int, int> iAlphaInterval;

	iAlphaInterval.a = 8;
	iAlphaInterval.b = 16;

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 0;
	iBetaInterval.b = 2;

	RVLSET3VECTOR(t, 0.6f, 0.0f, 0.2f);

	pVN->AddModelCluster(2, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.1f, 16, 2, iBetaInterval, pMem, 0.0f, iAlphaInterval);

	int iN;
	float *N;

	for (iN = 0; iN < pMCluster0->NArray.h; iN++)
	{
		N = pMCluster0->NArray.Element + pMCluster0->NArray.w * iN;

		if (N[0] < -1e-3)
			pVN->AddLimit(0, 0, 0, iN, 2, pMem);
	}

	pVN->AddOperation(3, -1, 0, 2, pMem);

	pVN->AddOperation(4, 1, 3, 1, pMem);

	pVN->SetOutput(4);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.7f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateMug3(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	betaInterval.a = 0.0f;
	betaInterval.b = 0.5f * PI + 1e-3;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.3f, convexTemplate, betaInterval, NArray, pMem);

	betaInterval.a = 0.5f * PI;
	betaInterval.b = PI;

	float axis[3];

	RVLSET3VECTOR(axis, -1.0f, 0.0f, 0.0f);

	RVLSET3VECTOR(t, 0.6f, 0.0f, 0.0f);

	pVN->AddModelCluster(2, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.1f, convexTemplate, axis, betaInterval, NArray, pMem);

	int iN;
	float *N;

	for (iN = 0; iN < pMCluster0->NArray.h; iN++)
	{
		N = pMCluster0->NArray.Element + pMCluster0->NArray.w * iN;

		if (N[0] < -1e-3)
			pVN->AddLimit(0, 0, 0, iN, 2, pMem);
	}

	pVN->AddOperation(3, -1, 0, 2, pMem);

	pVN->AddOperation(4, 1, 3, 1, pMem);

	pVN->SetOutput(4);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.7f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateMug4(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	pVN->AddModelCluster(2, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	pVN->AddOperation(3, -1, 0, 2, pMem);

	// pVN->SetOutput(3);

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

void VN_::CreateMug5(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.0f);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	pVN->AddModelCluster(2, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	pVN->AddOperation(3, 1, 0, 1, pMem);

	// pVN->SetOutput(3);

	pVN->AddOperation(4, -1, 3, 2, pMem);

	pVN->SetOutput(4);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreatePipe(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	betaInterval.a = 0.5f * PI - 0.01;
	betaInterval.b = 0.5f * PI + 0.01;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.1f, convexTemplate, betaInterval, NArray, pMem);

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

void VN_::CreateCar(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	float cs = 0.0f;
	float sn = -1.0f;

	RVLROTX(cs, sn, R);

	RVLSET3VECTOR(t, 0.4f, 0.0f, 0.4f);

	Pair<int, int> iAlphaInterval;

	iAlphaInterval.a = 12;
	iAlphaInterval.b = 14;

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 1;
	iBetaInterval.b = 1;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.3f, 16, 2, iBetaInterval, pMem, 0.0f, iAlphaInterval);

	RVLSET3VECTOR(t, -0.5f, 0.0f, 0.5f);

	iAlphaInterval.a = 8;
	iAlphaInterval.b = 12;

	pVN->AddModelCluster(2, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.4f, 16, 2, iBetaInterval, pMem, 0.0f, iAlphaInterval);

	pVN->AddOperation(3, 1, 0, 1, pMem);

	pVN->AddOperation(4, 1, 2, 3, pMem);

	pVN->SetOutput(4);

	pVN->Create(pMem);

	pVN->boundingBox.minx = -0.5f;
	pVN->boundingBox.maxx = 0.5f;
	pVN->boundingBox.miny = -0.5f;
	pVN->boundingBox.maxy = 0.5f;
	pVN->boundingBox.minz = -0.5f;
	pVN->boundingBox.maxz = 0.5f;
}

void VN_::CreateApple(
	VN *pVN,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	Pair<float, float> betaInterval;

	betaInterval.a = 0.0f;
	betaInterval.b = PI;

	Array2D<float> NArray;

	NArray.w = 3;
	NArray.h = 0;

	VN_::ModelCluster *pMCluster0 = pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, convexTemplate, betaInterval, NArray, pMem);

	RVLSET3VECTOR(t, 0.0f, 0.0f, 0.4f);

	Pair<int, int> iBetaInterval;

	iBetaInterval.a = 1;
	iBetaInterval.b = 3;

	pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_XTORUS, R, t, 0.1f, 16, 8, iBetaInterval, pMem, 0.02f);

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

float VN_::SceneFittingScore(
	SURFEL::SceneSamples sceneSamples,
	Array<OrientedPoint> PtArray,
	float maxe,
	float maxz,
	float cosSurfaceRayAngleThr,
	int &nOutliers,
	// float transparencyDepthThr,
	bool bVisualize)
{
	int neighborhoodSize = 5;

	int halfNeighborhoodSize = (neighborhoodSize - 1) / 2;

	// int neighbor[9];

	// int w = ZBuffer.w;

	// neighbor[0] = -1 - w;
	// neighbor[1] = -w;
	// neighbor[2] = 1 - w;
	// neighbor[3] = -1;
	// neighbor[4] = 0;
	// neighbor[5] = 1;
	// neighbor[6] = -1 + w;
	// neighbor[7] = w;
	// neighbor[8] = 1 + w;

	int maxmaxu = sceneSamples.w - 1;
	int maxmaxv = sceneSamples.h - 1;

	float *PS_ = new float[3 * sceneSamples.imagePtArray.h];

	float *PS__ = PS_;

	bool *bVisible = new bool[sceneSamples.imagePtArray.h];

	int i;
	float *PS;
	OrientedPoint *pPM;
	float c, r;

	for (i = 0; i < sceneSamples.imagePtArray.h; i++, PS__ += 3)
	{
		bVisible[i] = false;

		if (sceneSamples.status[i] == 0)
			continue;

		PS = sceneSamples.PtArray.Element[i];

		RVLSUM3VECTORS(PS, sceneSamples.Pc, PS__);

		pPM = PtArray.Element + i;

		if (pPM->P[2] <= maxz)
		{
			r = sqrt(RVLDOTPRODUCT3(pPM->P, pPM->P));

			c = RVLDOTPRODUCT3(pPM->N, pPM->P) / r;

			if (c <= -cosSurfaceRayAngleThr)
				bVisible[i] = true;
		}
	}

	bool *bMInlier;
	bool *bSInlier;

	if (bVisualize)
	{
		bMInlier = new bool[sceneSamples.imagePtArray.h];
		bSInlier = new bool[sceneSamples.imagePtArray.h];
	}

	float maxe2 = maxe * maxe;

	float scoreS = 0.0f;
	float scoreM = 0.0f;
	int nS = 0;
	int nM = 0;

	nOutliers = 0;

	int nMOutliers = 0;
	int nSOutliers = 0;

	int u, v, umin, umax, vmin, vmax;
	float dP[3];
	float e, e2;
	Pair<int, int> PtIdx;
	int iPt;
	float mine2;

	for (i = 0; i < sceneSamples.imagePtArray.h; i++)
	{
		if (sceneSamples.status[i] == 1)
		{
			nS++;

			e = sceneSamples.SDF[i];

			if (e < 0.0f)
				e = -e;

			if (e <= maxe)
			{
				scoreS += (1.0f - RVLABS(e) / maxe);

				if (bVisualize)
					bSInlier[i] = true;
			}
			else
			{
				nOutliers++;

				if (bVisualize)
				{
					bSInlier[i] = false;

					nSOutliers++;
				}
			}
		}

		if (bVisible[i])
		{
			pPM = PtArray.Element + i;

			PtIdx = sceneSamples.PtIdxArray[i];

			RVLNEIGHBORHOOD(PtIdx.a, PtIdx.b, halfNeighborhoodSize, 0, maxmaxu, 0, maxmaxv, umin, umax, vmin, vmax);

			mine2 = 2.0f * maxe2;

			for (v = vmin; v <= vmax; v++)
				for (u = umin; u <= umax; u++)
				{
					iPt = u + v * sceneSamples.w;

					PS = PS_ + 3 * iPt;

					RVLDIF3VECTORS(PS, pPM->P, dP);

					e2 = RVLDOTPRODUCT3(dP, dP);

					if (e2 < mine2)
						mine2 = e2;
				}

			e2 = mine2;

			if (e2 <= maxe2)
			{
				scoreM += (1.0f - sqrt(e2) / maxe);

				nM++;

				if (bVisualize)
					bMInlier[i] = true;
			}
			else
			{
				nOutliers++;

				if (bVisualize)
				{
					bMInlier[i] = false;

					nMOutliers++;
				}
			}

			// if (PM[2] > maxz)
			//{
			//	nOutliers++;

			//	if (bVisualize)
			//	{
			//		bSInlier[i] = false;

			//		nSOutliers++;
			//	}
			//}
			// else
			//{
			//	RVLDIF3VECTORS(PS_, PM, dP);

			//	e2 = RVLDOTPRODUCT3(dP, dP);

			//	if (e2 <= maxe2)
			//	{
			//		score += (1.0f - sqrt(e2) / maxe);

			//		if (bVisualize)
			//			bSInlier[i] = bMInlier[i] = true;
			//	}
			//	//else if (pSPt->P[2] > PM[2] + transparencyDepthThr)
			//	else
			//	{
			//		nOutliers++;

			//		if (bVisualize)
			//		{
			//			bSInlier[i] = bMInlier[i] = false;

			//			nSOutliers++;

			//			nMOutliers++;
			//		}
			//	}
			//}
		}
		// else if (sceneSamples.status[i] == 2)
		//{
		//	if (PM[2] <= maxz)
		//	{
		//		nOutliers++;

		//		if (bVisualize)
		//		{
		//			bSInlier[i] = bMInlier[i] = false;

		//			nSOutliers++;

		//			nMOutliers++;
		//		}
		//	}
		//	else if (bVisualize)
		//		bSInlier[i] = true;
		//}
	}

	float score = scoreS + scoreM * (float)nS / (float)nM;

	if (bVisualize)
	{
		// PSGM_::ModelInstance *pSCTI = CTISet.pCTI.Element[pHypothesis->iSCTI];

		// printf("segment %d model %d match %d score %f transparency %d ground distance %f\n",
		//	pSCTI->iCluster, iModel, iHypothesis, score, nTransparentPts, pHypothesis->gndDistance);

		int nPts = sceneSamples.imagePtArray.h;

		Point *PC = new Point[2 * nPts];

		Array<Point> MatchedPC;

		MatchedPC.Element = PC;
		MatchedPC.n = 0;

		Array<Point> OutlierPC;

		OutlierPC.Element = PC + nPts - nMOutliers;
		OutlierPC.n = 0;

		Array<Point> SPC;

		SPC.Element = PC + nPts;
		SPC.n = 0;

		Array<Point> SOutlierPC;

		SOutlierPC.Element = PC + 2 * nPts - nSOutliers;
		SOutlierPC.n = 0;

		Point *pSPt, *pMPt;

		for (i = 0; i < nPts; i++)
		{
			if (sceneSamples.status[i] == 1)
			{
				PS = PS_ + 3 * i;

				if (bSInlier[i])
				{
					pSPt = SPC.Element + SPC.n;

					SPC.n++;

					RVLCOPY3VECTOR(PS, pSPt->P);
				}
				else
				{
					pSPt = SOutlierPC.Element + SOutlierPC.n;

					SOutlierPC.n++;

					RVLCOPY3VECTOR(PS, pSPt->P);
				}
			}

			if (bVisible[i])
			{
				pPM = PtArray.Element + i;

				if (bMInlier[i])
				{
					pMPt = MatchedPC.Element + MatchedPC.n;

					MatchedPC.n++;

					RVLCOPY3VECTOR(pPM->P, pMPt->P);
				}
				else
				{
					pMPt = OutlierPC.Element + OutlierPC.n;

					OutlierPC.n++;

					RVLCOPY3VECTOR(pPM->P, pMPt->P);
				}
			}
		}

		Visualizer visualizer;

		visualizer.Create();

		unsigned char color[3];

		RVLSET3VECTOR(color, 0, 255, 0);

		visualizer.DisplayPointSet<float, Point>(MatchedPC, color, 4.0f);

		RVLSET3VECTOR(color, 0, 0, 255);

		visualizer.DisplayPointSet<float, Point>(SPC, color, 4.0f);

		RVLSET3VECTOR(color, 255, 0, 0);

		visualizer.DisplayPointSet<float, Point>(OutlierPC, color, 4.0f);

		RVLSET3VECTOR(color, 255, 0, 255);

		visualizer.DisplayPointSet<float, Point>(SOutlierPC, color, 4.0f);

		// visualizer.Run();

		delete[] PC;
		delete[] bMInlier;
		delete[] bSInlier;
	} // if (bVisualize)

	delete[] PS_;
	delete[] bVisible;

	return score;
}

float VN_::SceneFittingScore2(
	SURFEL::SceneSamples sceneSamples,
	Array<OrientedPoint> PtArray,
	float maxe,
	float maxz,
	float cosSurfaceRayAngleThr,
	int &nSamples,
	float &chamferDist,
	int &nOutliers,
	// float transparencyDepthThr,
	bool bVisualize)
{
	int neighborhoodSize = 5;

	int halfNeighborhoodSize = (neighborhoodSize - 1) / 2;

	// int neighbor[9];

	// int w = ZBuffer.w;

	// neighbor[0] = -1 - w;
	// neighbor[1] = -w;
	// neighbor[2] = 1 - w;
	// neighbor[3] = -1;
	// neighbor[4] = 0;
	// neighbor[5] = 1;
	// neighbor[6] = -1 + w;
	// neighbor[7] = w;
	// neighbor[8] = 1 + w;

	int maxmaxu = sceneSamples.w - 1;
	int maxmaxv = sceneSamples.h - 1;

	float *PS_ = new float[3 * sceneSamples.imagePtArray.h];

	float *PS__ = PS_;

	bool *bVisible = new bool[sceneSamples.imagePtArray.h];

	int i;
	float *PS;
	OrientedPoint *pPM;
	float c, r;

	for (i = 0; i < sceneSamples.imagePtArray.h; i++, PS__ += 3)
	{
		// if (i == 100)
		//	int debug = 0;

		bVisible[i] = false;

		if (sceneSamples.status[i] == 0)
			continue;

		PS = sceneSamples.PtArray.Element[i];

		if (sceneSamples.bCentered)
		{
			RVLSUM3VECTORS(PS, sceneSamples.Pc, PS__);
		}
		else
		{
			RVLCOPY3VECTOR(PS, PS__);
		}

		pPM = PtArray.Element + i;

		if (pPM->P[2] <= maxz)
		{
			if (pPM->P[2] < PS__[2] + maxe)
			{
				r = sqrt(RVLDOTPRODUCT3(pPM->P, pPM->P));

				c = RVLDOTPRODUCT3(pPM->N, pPM->P) / r;

				if (c <= -cosSurfaceRayAngleThr)
					bVisible[i] = true;
			}
		}
	}

	bool *bMInlier;
	bool *bSInlier;

	if (bVisualize)
	{
		bMInlier = new bool[sceneSamples.imagePtArray.h];
		bSInlier = new bool[sceneSamples.imagePtArray.h];
	}

	float maxe2 = maxe * maxe;

	float scoreS = 0.0f;
	float scoreM = 0.0f;
	int nS = 0;
	int nM = 0;

	chamferDist = 0.0f;

	nSamples = 0;

	nOutliers = 0;

	int nMOutliers = 0;
	int nSOutliers = 0;

	int u, v, umin, umax, vmin, vmax;
	float dP[3];
	float e, e2;
	Pair<int, int> PtIdx;
	int iPt;
	float mine2;

	for (i = 0; i < sceneSamples.imagePtArray.h; i++)
	{
		if (sceneSamples.status[i] == 1)
		{
			nSamples++;

			nS++;

			e = sceneSamples.SDF[i];

			if (e < 0.0f)
				e = -e;

			if (e <= maxe)
			{
				chamferDist += (e * e);

				scoreS += (1.0f - RVLABS(e) / maxe);

				if (bVisualize)
					bSInlier[i] = true;
			}
			else
			{
				chamferDist += maxe2;

				nOutliers++;

				if (bVisualize)
				{
					bSInlier[i] = false;

					nSOutliers++;
				}
			}
		}

		if (bVisible[i])
		{
			nSamples++;

			pPM = PtArray.Element + i;

			PtIdx = sceneSamples.PtIdxArray[i];

			RVLNEIGHBORHOOD(PtIdx.a, PtIdx.b, halfNeighborhoodSize, 0, maxmaxu, 0, maxmaxv, umin, umax, vmin, vmax);

			mine2 = 2.0f * maxe2;

			for (v = vmin; v <= vmax; v++)
				for (u = umin; u <= umax; u++)
				{
					iPt = u + v * sceneSamples.w;

					PS = PS_ + 3 * iPt;

					RVLDIF3VECTORS(PS, pPM->P, dP);

					e2 = RVLDOTPRODUCT3(dP, dP);

					if (e2 < mine2)
						mine2 = e2;
				}

			e2 = mine2;

			if (e2 <= maxe2)
			{
				scoreM += (1.0f - sqrt(e2) / maxe);

				chamferDist += e2;

				nM++;

				if (bVisualize)
					bMInlier[i] = true;
			}
			else
			{
				chamferDist += maxe2;

				nOutliers++;

				if (bVisualize)
				{
					bMInlier[i] = false;

					nMOutliers++;
				}
			}

			// if (PM[2] > maxz)
			//{
			//	nOutliers++;

			//	if (bVisualize)
			//	{
			//		bSInlier[i] = false;

			//		nSOutliers++;
			//	}
			//}
			// else
			//{
			//	RVLDIF3VECTORS(PS_, PM, dP);

			//	e2 = RVLDOTPRODUCT3(dP, dP);

			//	if (e2 <= maxe2)
			//	{
			//		score += (1.0f - sqrt(e2) / maxe);

			//		if (bVisualize)
			//			bSInlier[i] = bMInlier[i] = true;
			//	}
			//	//else if (pSPt->P[2] > PM[2] + transparencyDepthThr)
			//	else
			//	{
			//		nOutliers++;

			//		if (bVisualize)
			//		{
			//			bSInlier[i] = bMInlier[i] = false;

			//			nSOutliers++;

			//			nMOutliers++;
			//		}
			//	}
			//}
		}
		// else if (sceneSamples.status[i] == 2)
		//{
		//	if (PM[2] <= maxz)
		//	{
		//		nOutliers++;

		//		if (bVisualize)
		//		{
		//			bSInlier[i] = bMInlier[i] = false;

		//			nSOutliers++;

		//			nMOutliers++;
		//		}
		//	}
		//	else if (bVisualize)
		//		bSInlier[i] = true;
		//}
	}

	float score = scoreS + scoreM * (float)nS / (float)nM;

	if (bVisualize)
	{
		// PSGM_::ModelInstance *pSCTI = CTISet.pCTI.Element[pHypothesis->iSCTI];

		// printf("segment %d model %d match %d score %f transparency %d ground distance %f\n",
		//	pSCTI->iCluster, iModel, iHypothesis, score, nTransparentPts, pHypothesis->gndDistance);

		int nPts = sceneSamples.imagePtArray.h;

		Point *PC = new Point[2 * nPts];

		Array<Point> MatchedPC;

		MatchedPC.Element = PC;
		MatchedPC.n = 0;

		Array<Point> OutlierPC;

		OutlierPC.Element = PC + nPts - nMOutliers;
		OutlierPC.n = 0;

		Array<Point> SPC;

		SPC.Element = PC + nPts;
		SPC.n = 0;

		Array<Point> SOutlierPC;

		SOutlierPC.Element = PC + 2 * nPts - nSOutliers;
		SOutlierPC.n = 0;

		Point *pSPt, *pMPt;

		for (i = 0; i < nPts; i++)
		{
			if (sceneSamples.status[i] == 1)
			{
				PS = PS_ + 3 * i;

				if (bSInlier[i])
				{
					pSPt = SPC.Element + SPC.n;

					SPC.n++;

					RVLCOPY3VECTOR(PS, pSPt->P);
				}
				else
				{
					pSPt = SOutlierPC.Element + SOutlierPC.n;

					SOutlierPC.n++;

					RVLCOPY3VECTOR(PS, pSPt->P);
				}
			}

			if (bVisible[i])
			{
				pPM = PtArray.Element + i;

				if (bMInlier[i])
				{
					pMPt = MatchedPC.Element + MatchedPC.n;

					MatchedPC.n++;

					RVLCOPY3VECTOR(pPM->P, pMPt->P);
				}
				else
				{
					pMPt = OutlierPC.Element + OutlierPC.n;

					OutlierPC.n++;

					RVLCOPY3VECTOR(pPM->P, pMPt->P);
				}
			}
		}

		Visualizer visualizer;

		visualizer.Create();

		unsigned char color[3];

		RVLSET3VECTOR(color, 0, 255, 0);

		visualizer.DisplayPointSet<float, Point>(MatchedPC, color, 4.0f);

		RVLSET3VECTOR(color, 0, 0, 255);

		visualizer.DisplayPointSet<float, Point>(SPC, color, 4.0f);

		RVLSET3VECTOR(color, 255, 0, 0);

		visualizer.DisplayPointSet<float, Point>(OutlierPC, color, 4.0f);

		RVLSET3VECTOR(color, 255, 0, 255);

		visualizer.DisplayPointSet<float, Point>(SOutlierPC, color, 4.0f);

		visualizer.Run();

		delete[] PC;
		delete[] bMInlier;
		delete[] bSInlier;
	} // if (bVisualize)

	delete[] PS_;
	delete[] bVisible;

	return score;
}

void VN_::Init(
	RECOG::VN_::FitData &fitData,
	Array2D<float> M,
	float beta,
	float lambda0,
	float lambda1,
	bool bRot,
	int nPts)
{
	fitData.M = M;

	fitData.iV.Element = new int[M.w];

	int m = (bRot ? M.h + 3 : M.h);

	fitData.A.create(m, m, CV_32FC1);

	fitData.A_ = (float *)(fitData.A.data);

	fitData.b.create(m, 1, CV_32FC1);

	fitData.b_ = (float *)(fitData.b.data);

	fitData.a = new float[m];

	fitData.q.create(m, 1, CV_32FC1);

	fitData.q_ = (float *)(fitData.q.data);

	fitData.dSV = new float[M.w];

	fitData.lambda0 = lambda0;
	fitData.lambda1 = lambda1 / (float)(M.w);

	fitData.sqrtbeta = sqrt(beta);

	fitData.P = new float[3 * nPts];
}

void VN_::Delete(RECOG::VN_::FitData &fitData)
{
	delete[] fitData.a;
	delete[] fitData.iV.Element;
	delete[] fitData.dSV;
	delete[] fitData.P;
}

void VN_::Create(
	VN *pMetaModel,
	int *valid,
	Array<RECOG::PSGM_::Plane> convexTemplate,
	CRVLMem *pMem,
	VN *pVN)
{
	pVN->CreateEmpty();

	float R[9];

	RVLUNITMX3(R);

	float t[3];

	RVLNULL3VECTOR(t);

	VN_::ModelCluster *pMCluster = pMetaModel->modelClusterList.pFirst;

	int nModelClusters = 0;
	int nConvex = 0;
	while (pMCluster)
	{
		nModelClusters++;
		if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX)
			nConvex++;
		pMCluster = pMCluster->pNext;
	}

	// if (nConvex >= 2)
	//	int debug = 1;
	VN_::ModelCluster **convexCluster = new VN_::ModelCluster *[nConvex];
	int *valid_ = valid;
	int **valid__ = new int *[nModelClusters];

	int iCluster = 0;
	int iConvex = 0;
	int nFeatures;
	VN_::ModelCluster *pMCluster_;
	pMCluster = pMetaModel->modelClusterList.pFirst;
	while (pMCluster)
	{
		pMCluster_ = pVN->AddModelCluster(pMCluster, valid_, R, t, pMem);
		valid__[iCluster] = valid_;
		nFeatures = (pMCluster->NArray.h > 0 ? pMCluster->NArray.h : pMCluster->alphaArray.n * pMCluster->betaArray.n);
		valid_ += nFeatures;

		if (pMCluster->type == RVLVN_CLUSTER_TYPE_CONVEX)
			convexCluster[iConvex++] = pMCluster_;

		pMCluster = pMCluster->pNext;
	}

	VN_::Operation *pMOperation = pMetaModel->operationList.pFirst;

	while (pMOperation)
	{
		pVN->AddOperation(pMOperation->ID, pMOperation->operation, pMOperation->operand[0], pMOperation->operand[1], pMem);

		pMOperation = pMOperation->pNext;
	}

	VN_::ModelCluster *pMCluster2;
	bool bN;
	// if 2 or more convex
	if (nConvex >= 2)
	{
		pMCluster = convexCluster[0];
		for (int i = 1; i < nConvex; i++)
		{
			pMCluster2 = convexCluster[i];
			for (int n = 0; n < pMCluster->NArray.h; n++)
			{
				// if (pMCluster->NArray.Element[n]==1 && pMCluster2->NArray.Element[n]==0)
				//	pVN->AddLimit(convexCluster[0]->ID, 0, 0, pMCluster->NArray.Element[n], convexCluster[i]->ID, pMem); //[i]

				bN = false;
				for (int n2 = 0; n2 < pMCluster2->NArray.h; n2++)
				{
					// if (pMCluster->NArray.Element[n] == pMCluster2->NArray.Element[n2])
					if (pMCluster->iN[n] == pMCluster2->iN[n2])
					{
						bN = true;
						break;
					}
				}

				if (!bN)
					pVN->AddLimit(convexCluster[0]->ID, 0, 0, n, convexCluster[i]->ID, pMem); //[i]
			}
		}
	}

	delete[] valid__;
	delete[] convexCluster;

	// pVN->AddOperation(2, 1, 0, 1, pMem);

	pVN->SetOutput(pMetaModel->outputID);

	pVN->Create(pMem);

	// pVN->boundingBox.minx = -0.5f;
	// pVN->boundingBox.maxx = 0.5f;
	// pVN->boundingBox.miny = -0.5f;
	// pVN->boundingBox.maxy = 0.5f;
	// pVN->boundingBox.minz = -0.5f;
	// pVN->boundingBox.maxz = 0.5f;
}

// Created by Robert Cupec and modified by Siniï¿½a Staniï¿½.

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
	bool *bVertexJoined,
	bool *vertexCH)
{
	if (iBeta < tangentSet[pVEdge0->idx].miniBeta || iBeta > tangentSet[pVEdge0->idx].maxiBeta || bEdgeJoined[pVEdge0->idx]) // line 6
		return;

	// Identify the connected set of edges containing pVEdge0 such that all edges in this set have a tangent in the tangent set iBeta.
	// This edge set is stored in VEdgeBuff. (line 10)

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

		for (j = 0; j < 2; j++) // for every endpoint of pVEdge
		{
			pVEdgePtr = pSurfels->vertexArray.Element[pVEdge->iVertex[j]]->EdgeList.pFirst;

			while (pVEdgePtr)
			{
				pVEdge_ = pVEdgePtr->pEdge;

				// Stanic: provjeriti jesu li oba verteksa pVEdge_ unutar CH. (Y)

				if (vertexCH)
					if (!*(vertexCH + pVEdge_->iVertex[0]) && !*(vertexCH + pVEdge_->iVertex[1]))
					{
						pVEdgePtr = pVEdgePtr->pNext;
						continue;
					}

				if (iBeta >= tangentSet[pVEdge_->idx].miniBeta && iBeta <= tangentSet[pVEdge_->idx].maxiBeta && !bEdgeJoined[pVEdge_->idx])
				{
					bEdgeJoined[pVEdge_->idx] = true; // line 11

					*(pVEdgePush++) = pVEdge_;
				} // if (!bJoined[pVEdge_->idx])

				pVEdgePtr = pVEdgePtr->pNext;

			} // for every neighbor edge
		}	  // for (j = 0; j < 2; j++)
	}		  // main loop

	SURFEL::VertexEdge **pVEdgeBuffEnd = pVEdgeFetch;

	int nEdges = pVEdgeBuffEnd - VEdgeBuff;

	if (nEdges < minnEdges) // line 12
		return;

	/// The following code implements line 13.

	// Create the compatibility table bComp indicating compatibility between all edge pairs in VEdgeBuff.
	// bComp(i, j) = true if the both endpoints of the i-th edge in VEdgeBuff are above the tangent of the j-th edge,
	// otherwise bComp(i, j) = false.

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

	// Sort edges according to the compatibility score in descending order.
	// The compatibility score of an edge is computed by summing the lengths of all edges, which are not compatible with this edge
	// and subtracting this sum from the length of the considered edge.
	// The sorted edges are stored in sortedTangentArray.

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

	// Create ringEdgeArray iteratively by considering edges contained in sortedTangentArray one by one, in the order in which they are stored,
	// and adding to ringEdgeArray every edge, which is compatible with all edges already contained in ringEdgeArray.

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

	///

	// If ringEdgeArray.n < minnEdges, then no ring is created.

	// if (ringEdgeArray.n >= minnEdges && ringEdgeArray.n > nEdges / 2)
	if (ringEdgeArray.n >= minnEdges) // line 14
	{
		// If there is an edge in ringEdgeArray such that all other edges lie on its tangent within tolerance maxError,
		// then no ring is created.

		int iTmp;

		for (i = 0; i < ringEdgeArray.n; i++)
		{
			iTmp = ringEdgeArray.Element[i] * nEdges;

			for (j = 0; j < ringEdgeArray.n; j++)
			{
				iComp = iTmp + ringEdgeArray.Element[j];

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
			// Create torus ring and add it to pRingList.

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

					// if (iVertex >= pSurfels->vertexArray.n)
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
		} // if (i >= ringEdgeArray.n)
	}	  // if (ringEdgeArray.n >= minnEdges)

	// if (iBeta == 2)
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

// Created by Robert Cupec and modified by Siniï¿½a Staniï¿½.

void VN::ToroidalClusters2(
	Mesh *pMesh,
	float *PArray,
	float *NArray,
	SurfelGraph *pSurfels,
	PlanarSurfelDetector *pSurfelDetector,
	float *axis,
	Array<float> alphaArray,
	Array<float> betaArray,
	float maxErr,
	QList<RECOG::VN_::Torus> *pTorusList,
	// Array<RECOG::VN_::Torus *> &SClusters,
	CRVLMem *pMem,
	bool *vertexCH,
	int &nToruses,
	int iSuperSegment)
{

	// Assign tangents to all vertex edges. (line 1)

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
	SURFEL::VertexEdge *pVEdge;
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
		// provjeriti je li iVertex unutar CH. (Y)

		if (!*(vertexCH + iVertex))
		{
			continue;
		}

		pVertex = pSurfels->vertexArray.Element[iVertex];

		P = PArray + 3 * iVertex;

		// if (iVertex == 110)
		//	int debug_ = 0;

		// int debug = 0;

		// int debug__;

		pVEdgePtr = pVertex->EdgeList.pFirst;

		while (pVEdgePtr)
		{
			iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pVEdgePtr);

			// provjeriti je li iVertex_ unutar CH. (Y)
			if (!*(vertexCH + iVertex_))
			{
				pVEdgePtr = pVEdgePtr->pNext;
				continue;
			}

			if (iVertex_ > iVertex)
			{
				pVEdge = pVEdgePtr->pEdge;

				// if (pVEdge->idx == 75)
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
						a = adn * adn - cb2[iBeta] * dndn;
						b = 2.0f * (adn * an1 - cb2[iBeta] * n1dn);
						c = an1 * an1 - cb2[iBeta];

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
				} // for (iBeta = 0; iBeta < betaArray.n; iBeta++)

				pTangentSet->tangentArray.n = pTangentSet->maxiBeta - pTangentSet->miniBeta + 1;

				RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, VN_::EdgeTangent *, pTangentSet->tangentArray.n, pTangentSet->tangentArray.Element);

				j = 0;

				for (i = pTangentSet->miniBeta; i <= pTangentSet->maxiBeta; i++, j++)
					pTangentSet->tangentArray.Element[j] = tangentArray[i];
			} // if (iVertex_ > iVertex)

			pVEdgePtr = pVEdgePtr->pNext;
		} // while (pVEdgePtr)

		// if (debug == 1)
		//	int debug_ = 0;
	} // for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)

	delete[] cb2;
	delete[] tangentArray;

	// FILE *fp = fopen("tangentRing.txt", "w");

	// iBeta = 2;

	// SURFEL::Vertex *pVertex_;

	// for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
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

	// fclose(fp);

	// Detect torus rings for each beta. The detected rings are stored in ringListArray.

	bool *bEdgeJoined = new bool[pSurfels->vertexEdgeArray.n];

	bool *bVertexJoined = new bool[pSurfels->vertexArray.n];

	memset(bVertexJoined, 0, pSurfels->vertexArray.n * sizeof(bool));

	Array<QList<RECOG::VN_::TorusRing>> ringListArray;

	ringListArray.Element = new QList<RECOG::VN_::TorusRing>[betaArray.n];
	ringListArray.n = betaArray.n;

	QList<RECOG::VN_::TorusRing> *pRingList;

	SURFEL::VertexEdge **VEdgeBuff = new SURFEL::VertexEdge *[pSurfels->vertexEdgeArray.n];

	for (iBeta = 0; iBeta < betaArray.n; iBeta++) // line 4
	{
		pRingList = ringListArray.Element + iBeta;

		RVLQLIST_INIT(pRingList);

		memset(bEdgeJoined, 0, pSurfels->vertexEdgeArray.n * sizeof(bool)); // line 7

		for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
		{

			// provjeriti je li iVertex unutar CH. (Y)

			if (!*(vertexCH + iVertex))
				continue;

			pVertex = pSurfels->vertexArray.Element[iVertex];

			// if ()

			pVEdgePtr = pVertex->EdgeList.pFirst;

			while (pVEdgePtr)
			{
				iVertex_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pVEdgePtr);

				// provjeriti je li iVertex_ unutar CH. (Y)

				if (!*(vertexCH + iVertex_))
				{
					pVEdgePtr = pVEdgePtr->pNext;
					continue;
				}

				if (iVertex_ > iVertex)
				{
					pVEdge = pVEdgePtr->pEdge;

					DetectTorusRings(pMesh, PArray, pSurfels, pVEdge, axis, iBeta, tangentSet, maxErr, pRingList, pMem, VEdgeBuff,
									 bEdgeJoined, bVertexJoined, vertexCH);
				}

				pVEdgePtr = pVEdgePtr->pNext;
			}
		}
	}

	int iRing;

	// FILE *fp = fopen("tangentRing.txt", "w");

	// iRing = 0;

	// pRingList = ringListArray.Element + 0;

	// RECOG::VN_::TorusRing *pRingDebug = pRingList->pFirst;

	// while (pRingDebug)
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

	// fclose(fp);

	delete[] tangentSet;
	delete[] bEdgeJoined;
	delete[] bVertexJoined;
	delete[] VEdgeBuff;

	// Compute CTI descriptors for all rings. (lines 15-19)
	// For each alpha, the index of the kex vertex, i.e. the vertex defining the tangent corresponding to this alpha
	// is stored in iKeyVertex array of each ring.

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

	// For each pair of rings (pPrevRing, pRing), where pRing is a ring form the ring list of a particular beta
	// and pPrevRing is a ring form the ring list of the previous beta in betaArray, a convexity test is applied.
	// This test checks whether for each alpha, the key vertex of pPrevRing assigned to this alpha is below the tangent
	// of pRing corresponding to the same alpha and opposite.
	// If the pair (pPrevRing, pRing) passes the convexity test, then pPrevRing is added to pRing->compList. (lines 20-24)

	int iPrevBeta = 0;
#ifdef RVLVN_TOROIDALCLUSTERS_VERBOSE
	FILE *pFileTorusInfo = NULL;
	pFileTorusInfo = fopen("D:\\Dipl\\torusinfo.txt", "a+");
#endif
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
					{

#ifdef RVLVN_TOROIDALCLUSTERS_VERBOSE
						fprintf(pFileTorusInfo, "SuperSegment: no.%d\nStopped at pPrevRing condition!\nCurrent e:%f\nMaxErr:%f\nRingID:%d\nPrevious RingID:%d\nCurrent beta:%d\n\n",
								iSuperSegment, e, maxErr, pRing->idx, pPrevRing->idx, iBeta);
#endif
						break;
					}
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
						{

#ifdef RVLVN_TOROIDALCLUSTERS_VERBOSE
							fprintf(pFileTorusInfo, "SuperSegment: no.%d\nStopped at pRing condition!\nCurrent e:%f\nMaxErr:%f\nRingID:%d\nPrevious RingID:%d\nCurrent beta:%d\n\n",
									iSuperSegment, e, maxErr, pRing->idx, pPrevRing->idx, iBeta);
#endif
							break;
						}
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
#ifdef RVLVN_TOROIDALCLUSTERS_VERBOSE
	fclose(pFileTorusInfo);
#endif

	/// The following code implements lines 30-35.

	// A queue is created and every ring which is not contained in compList of any other ring is inserted in this queue.

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

	// For each sequence of rings, where each ring in a sequence is contained in compList of any the previous ring in the sequence,
	// create a torus and add it to torusList.

	// zakomentiran ptoruslist
	/*
	QList<VN_::Torus> torusList;

	QList<VN_::Torus> *pTorusList = &torusList;

	RVLQLIST_INIT(pTorusList);

	SClusters.n = 0;
	*/

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
			nToruses++;
			// ovdje promijeniti

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, VN_::TorusRing *, betaArray.n, pTorus->ringArray.Element);

			memset(pTorus->ringArray.Element, 0, betaArray.n * sizeof(VN_::TorusRing *));

			pTorus->ringArray.n = betaArray.n;

			pTorusTreeNode_ = pTorusTreeNode;

			while (pTorusTreeNode_)
			{
				pTorus->ringArray.Element[pTorusTreeNode_->pRing->iBeta] = pTorusTreeNode_->pRing;

				pTorusTreeNode_ = pTorusTreeNode_->pParent;
			}

			// SClusters.n++;
		}

		pTorusTreeNode = pTorusTreeNode->pNext;
	}

	delete[] ringListArray.Element;
	delete[] ca;
	delete[] sa;
	delete[] cb;
	delete[] sb;

	// Copy all toruses from pTorusList to SClusters.

	// SClusters.Element = new VN_::Torus *[SClusters.n];

	// QLIST::CreatePtrArray<VN_::Torus>(pTorusList, &SClusters);
}

// Created Siniï¿½a Staniï¿½.

bool RECOG::VN_::TorusDetectionKeyPressCallback(
	// vtkObject* caller, unsigned long eid, void* clientdata, void *calldata)
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	std::string &keySym,
	void *vpData)
{
	VN_::TorusDetectionDisplayData *pData = (VN_::TorusDetectionDisplayData *)vpData;

	// vtkSmartPointer<vtkRenderWindowInteractor> interactor =
	//	reinterpret_cast<vtkRenderWindowInteractor*>(caller);

	// std::string keySym = "";
	// keySym = interactor->GetKeySym();

	if (keySym == "s")
	{
		std::cout << "Enter torus index: ";

		std::string line;

		std::getline(std::cin, line);

		int iSelectedTorus;

		sscanf(line.data(), "%d", &iSelectedTorus);

		// VNClassifier *pClassifier = (VNClassifier *)clientdata;
		Visualizer *pVisualizer = pData->pVisualizer;

		vtkPropCollection *props = pVisualizer->renderer->GetViewProps();
		props->InitTraversal();
		for (int i = 0; i < props->GetNumberOfItems(); i++)
		{

			if (i == (pData->nProps + iSelectedTorus))
			{
				props->GetNextProp()->VisibilityOn();
			}
			else if (i >= pData->nProps)
			{
				props->GetNextProp()->VisibilityOff();
			}
			else
				props->GetNextProp();
		}

		// interactor->GetRenderWindow()->Render();

		return true;
	}

	return false;
}

// Created Siniï¿½a Staniï¿½.

void RECOG::VN_::TorusVisualization(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	PlanarSurfelDetector *pSurfelDetector,
	QList<RECOG::VN_::Torus> torusList,
	std::vector<RVL::Point> MidPointCH)
{

	RECOG::VN_::Torus *pTorus;
	Visualizer vTorus;

	vTorus.Create();
	// vTorus.SetMesh(pMesh);

	VN_::TorusDetectionDisplayData displayData;

	displayData.pVisualizer = &vTorus;

	pSurfels->DisplayData.keyPressUserFunction = &(VN_::TorusDetectionKeyPressCallback);
	pSurfels->DisplayData.vpUserFunctionData = &displayData;
	pSurfels->DisplayData.bCallbackFunctionsDefined = false;

	uchar SelectionColor[] = {0, 255, 0};
	pSurfels->NodeColors(SelectionColor);

	// RandomColors(SelectionColor, pSurfels-visualizationData.clusterColor, pSurfels->NodeArray.n);

	pSurfels->InitDisplay(&vTorus, pMesh, pSurfelDetector);

	// pSurfels->Display(&vTorus, pMesh);

	// pSurfels->DisplayEdgeFeatures();

	pTorus = torusList.pFirst;

	// vTorus.SetKeyPressCallback(TorusDetectionKeyPressCallback(),this);

	// vtkSmartPointer<vtkPolyData> ringPolyData = vtkSmartPointer<vtkPolyData>::New();
	// vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	// vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

	// vtkSmartPointer<vtkPoints> ringPts = vtkSmartPointer<vtkPoints>::New();
	// vtkSmartPointer<vtkCellArray> ringEdges = vtkSmartPointer<vtkCellArray>::New();
	// vtkSmartPointer<vtkUnsignedCharArray> ringColors = vtkSmartPointer<vtkUnsignedCharArray>::New();

	SURFEL::VertexEdge *pVEdgeS;
	vtkSmartPointer<vtkLine> edge_helper = vtkSmartPointer<vtkLine>::New();

	std::vector<vtkSmartPointer<vtkActor>> actorList;
	std::vector<vtkSmartPointer<vtkPolyDataMapper>> mapperList;
	std::vector<vtkSmartPointer<vtkPolyData>> ringPolyDataList;
	std::vector<vtkSmartPointer<vtkPoints>> ringPtsList;
	std::vector<vtkSmartPointer<vtkCellArray>> ringEdgesList;
	std::vector<vtkSmartPointer<vtkUnsignedCharArray>> ringColorsList;

	// ringColors->SetNumberOfComponents(3);

	unsigned char one_red[3] = {255, 0, 0};
	unsigned char two_green[3] = {0, 255, 0};
	unsigned char three_blue[3] = {0, 0, 255};
	unsigned char four_yellow[3] = {255, 255, 0};
	unsigned char five_magenta[3] = {255, 0, 255};
	unsigned char six_orange[3] = {255, 165, 0};
	unsigned char axis_white[3] = {255, 255, 255};
	unsigned char seven_cyan[3] = {0, 255, 255};

	RECOG::VN_::TorusRing *pRingS;

	vtkPropCollection *props = vTorus.renderer->GetViewProps();

	displayData.nProps = props->GetNumberOfItems();

	int torusID = 0, CurrentAxisID = 0;
	while (pTorus)
	{

		ringPtsList.push_back(vtkSmartPointer<vtkPoints>::New());
		ringEdgesList.push_back(vtkSmartPointer<vtkCellArray>::New());
		ringColorsList.push_back(vtkSmartPointer<vtkUnsignedCharArray>::New());
		ringPolyDataList.push_back(vtkSmartPointer<vtkPolyData>::New());
		mapperList.push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
		actorList.push_back(vtkSmartPointer<vtkActor>::New());

		ringColorsList[torusID]->SetNumberOfComponents(3);

		for (int iRing = 0; iRing < pTorus->ringArray.n; iRing++)
		{ // prodi kroz sve prstene torusa
			pRingS = pTorus->ringArray.Element[iRing];

			if (pRingS != NULL)
			{
				for (int iEdge = 0; iEdge < pRingS->iEdgeArray.n; iEdge++)
				{ // prodi kroz sve edgeve prstena

					pVEdgeS = pSurfels->vertexEdgeArray.Element[pRingS->iEdgeArray.Element[iEdge]]; // dohvati edge ciji indeks odgovara i-tom edgeu prstena

					int iVertex0 = pVEdgeS->iVertex[0];
					int iVertex1 = pVEdgeS->iVertex[1];

					ringPtsList[torusID]->InsertNextPoint(pSurfels->vertexArray.Element[iVertex0]->P);
					ringPtsList[torusID]->InsertNextPoint(pSurfels->vertexArray.Element[iVertex1]->P);

					edge_helper->GetPointIds()->SetId(0, ringPtsList[torusID]->GetNumberOfPoints() - 2); // jer sam vec dodao 2 tocke, pa mi za nulti index num of pts bude 2
					edge_helper->GetPointIds()->SetId(1, ringPtsList[torusID]->GetNumberOfPoints() - 1); // uvecano za 1

					ringEdgesList[torusID]->InsertNextCell(edge_helper);

					switch (pRingS->iBeta)
					{
					case 0:
						ringColorsList[torusID]->InsertNextTypedTuple(one_red);
						break;
					case 1:
						ringColorsList[torusID]->InsertNextTypedTuple(two_green);
						break;
					case 2:
						ringColorsList[torusID]->InsertNextTypedTuple(three_blue);
						break;
					case 3:
						ringColorsList[torusID]->InsertNextTypedTuple(four_yellow);
						break;
					case 4:
						ringColorsList[torusID]->InsertNextTypedTuple(five_magenta);
						break;
					case 5:
						ringColorsList[torusID]->InsertNextTypedTuple(six_orange);
						break;
					case 6:
						ringColorsList[torusID]->InsertNextTypedTuple(seven_cyan);
						break;
					}
				}
			}
		}
		// crtanje axis_s

		if (torusID == MidPointCH[CurrentAxisID].label)
			CurrentAxisID++;

		ringPtsList[torusID]->InsertNextPoint(MidPointCH[CurrentAxisID].P);
		ringPtsList[torusID]->InsertNextPoint(MidPointCH[CurrentAxisID].N);

		edge_helper->GetPointIds()->SetId(0, ringPtsList[torusID]->GetNumberOfPoints() - 2);
		edge_helper->GetPointIds()->SetId(1, ringPtsList[torusID]->GetNumberOfPoints() - 1);

		ringEdgesList[torusID]->InsertNextCell(edge_helper);
		ringColorsList[torusID]->InsertNextTypedTuple(axis_white);

		ringPolyDataList[torusID]->SetPoints(ringPtsList[torusID]);
		ringPolyDataList[torusID]->SetLines(ringEdgesList[torusID]);
		ringPolyDataList[torusID]->GetCellData()->SetScalars(ringColorsList[torusID]);

		mapperList[torusID]->SetInputData(ringPolyDataList[torusID]);
		actorList[torusID]->SetMapper(mapperList[torusID]);
		actorList[torusID]->GetProperty()->SetLineWidth(5);

		vTorus.renderer->AddActor(actorList[torusID]);

		torusID++;
		pTorus = pTorus->pNext;
	}

	props = vTorus.renderer->GetViewProps();
	props->InitTraversal();
	for (int i = 0; i < props->GetNumberOfItems(); i++)
	{
		// za i = 2 props je mesh, nakon kojeg slijede prstenovi,
		if (i >= displayData.nProps)
		{
			props->GetNextProp()->VisibilityOff();
		}
		else
		{
			props->GetNextProp();
		}
	}

	// vTorus.SetKeyPressCallback(VN_::TorusDetectionKeyPressCallback,	&vTorus);

	vTorus.Run();
}