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
#include "TSM.h"
#include "TSMSet.h"

using namespace RVL;
using namespace RECOG;

TSM::TSM()
{
}


TSM::~TSM()
{
}

void TSM::Create(
	SurfelGraph *pSurfels,
	Array<int> iSurfelArray,
	float *R,
	float *t,
	void *vpSet)
{
	TSMSet *pSet = (TSMSet *)vpSet;

	Array<int> vertexArray = pSet->vertexArray;

	BYTE *mVertexFlags = pSet->mVertexFlags;

	CRVLMem *pMem = pSet->pMem;

	// vertexArray <- indices of vertices belonging to surfels in iSurfelArray

	vertexArray.n = 0;

	int iVertex, iSurfel;
	//Surfel *pSurfel;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *pSurfelVertexIdx;
	int iiSurfel;

	for (iiSurfel = 0; iiSurfel < iSurfelArray.n; iiSurfel++)
	{
		iSurfel = iSurfelArray.Element[iiSurfel];

		//pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

		pSurfelVertexIdx = pSurfelVertexList->pFirst;

		while (pSurfelVertexIdx)
		{
			iVertex = pSurfelVertexIdx->Idx;

			if (!(mVertexFlags[iVertex] & RVLRECOG_TSM_VERTEX_FLAG_MARKED))
			{
				vertexArray.Element[vertexArray.n++] = iVertex;

				mVertexFlags[iVertex] |= RVLRECOG_TSM_VERTEX_FLAG_MARKED;
			}

			pSurfelVertexIdx = pSurfelVertexIdx->pNext;
		}
	}

	// A_ <- A * R'

	float *A = pSet->convexTemplate.Element;

	int nT = pSet->convexTemplate.h;

	float *A_ = new float[3 * nT];

	int i;
	float *a, *a_;

	for (i = 0; i < nT; i++)
	{
		a = A + 3 * i;
		a_ = A_ + 3 * i;

		RVLMULMX3X3VECT(R, a, a_);
	}



	delete[] A_;

	// Clear RVLRECOG_TSM_VERTEX_FLAG_MARKED flag.

	int iiVertex;

	for (iiVertex = 0; iiVertex < vertexArray.n; iiVertex++)
		mVertexFlags[vertexArray.Element[iiVertex]] &= ~RVLRECOG_TSM_VERTEX_FLAG_MARKED;
}