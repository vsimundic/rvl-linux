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
#include "TG.h"
#include "TGSet.h"

using namespace RVL;
using namespace RECOG;

TG::TG()
{
}


TG::~TG()
{
}

void TG::Create(
	SurfelGraph *pSurfels,
	Array<int> iVertexArray,
	float *RIn,
	float *tIn,
	void *vpSet)
{
	TGSet *pSet = (TGSet *)vpSet;

	CRVLMem *pMem = pSet->pMem;

	// Copy rotation matrix and translation vector.

	RVLCOPYMX3X3(RIn, R);
	RVLCOPY3VECTOR(tIn, t);

	// A_ <- A * R'

	float *A__ = A.Element;

	int nT = A.h;

	float *A_ = new float[3 * nT];

	int i;
	float *a__, *a_;

	for (i = 0; i < nT; i++)
	{
		a__ = A__ + 3 * i;
		a_ = A_ + 3 * i;

		RVLMULMX3X3VECT(R, a__, a_);
	}

	// Create nodes and descriptor.

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, QList<TGNode>, A.h, descriptor.Element);

	nNodes = 0;

	int j;
	SURFEL::Vertex *pVertex;
	float dist;
	float *N;
	TGNode *pNode, *pNode_, *pNode__;
	QList<TGNode> *pDescriptorBin;
	float d;
	int iVertex;

	for (i = 0; i < A.h; i++)
	{
		N = A_ + 3 * i;

		pDescriptorBin = descriptor.Element + i;

		RVLQLIST_INIT(pDescriptorBin);

		for (j = 0; j < iVertexArray.n; j++)
		{
			iVertex = iVertexArray.Element[j];

			pVertex = pSurfels->vertexArray.Element[iVertex];

			dist = pSurfels->DistanceFromNormalHull(pVertex->normalHull, N);

			if (dist < 0.0f)
				continue;

			d = RVLDOTPRODUCT3(N, pVertex->P);

			pNode_ = pDescriptorBin->pFirst;

			while (pNode_)
			{
				if (d > pNode_->d)
					break;

				pNode__ = pNode_;

				pNode_ = pNode_->pNext;
			}

			RVLMEM_ALLOC_STRUCT(pMem, TGNode, pNode);

			RVLQLIST_INSERT_ENTRY(pDescriptorBin, pNode__, pNode_, pNode);

			pNode->d = d;
			pNode->i = i;
			pNode->iVertex = iVertex;

			nNodes++;
		}
	}

	// Free memory.

	delete[] A_;
}

void TG::Save(
	FILE *fp,
	bool bSaveA)
{
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

	fprintf(fp, "%d\t0\t0\n", nNodes);

	QList<TGNode> *pDescriptorBin;
	TGNode *pNode;

	for (i = 0; i < A.h; i++)
	{
		pDescriptorBin = descriptor.Element + i;

		pNode = pDescriptorBin->pFirst;

		while (pNode)
		{
			fprintf(fp, "%d\t%f\t%d\n", pNode->i, pNode->d, pNode->iVertex);

			pNode = pNode->pNext;
		}
	}
}

void TG::Load(
	FILE *fp,
	void *vpSet,
	bool bLoadA)
{
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

	fscanf(fp, "%d\t0\t0\n", &nNodes);

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, QList<TGNode>, A.h, descriptor.Element);

	QList<TGNode> *pDescriptorBin;

	for (i = 0; i < A.h; i++)
	{
		pDescriptorBin = descriptor.Element + i;

		RVLQLIST_INIT(pDescriptorBin);
	}

	TGNode *NodeMem;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, TGNode, nNodes, NodeMem);

	TGNode *pNode = NodeMem;	

	for (i = 0; i < nNodes; i++, pNode++)
	{
		fscanf(fp, "%d\t%f\t%d\n", &(pNode->i), &(pNode->d), &(pNode->iVertex));

		pDescriptorBin = descriptor.Element + pNode->i;

		RVLQLIST_ADD_ENTRY(pDescriptorBin, pNode);
	}
}