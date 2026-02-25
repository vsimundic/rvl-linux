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

using namespace RVL;
using namespace RECOG;

TGSet::TGSet()
{
	//vertexArray.Element = NULL;
	//mVertexFlags = NULL;
	//convexTemplate.Element = NULL;
	//convexTemplate.w = 3;
	A.h = 0;
	A.Element = NULL;

	nodeSimilarityThr = 1.0f;
	eLimit = 1.0f;
}


TGSet::~TGSet()
{
	//RVL_DELETE_ARRAY(vertexArray.Element);
	//RVL_DELETE_ARRAY(mVertexFlags);
	//RVL_DELETE_ARRAY(convexTemplate.Element);
	RVL_DELETE_ARRAY(A.Element);

	Clear();
}

void TGSet::Init(SurfelGraph *pSurfels)
{
	//vertexArray.Element = new int[pSurfels->vertexArray.n];
	//mVertexFlags = new BYTE[pSurfels->vertexArray.n];
	//memset(mVertexFlags, 0, pSurfels->vertexArray.n * sizeof(BYTE));
}

void TGSet::Clear()
{
	int i;
	TG *pTG;

	for (i = 0; i < TGs.size(); i++)
	{
		pTG = TGs.at(i);

		delete pTG;
	}

	TGs.clear();

	VertexGraph *pVertexGraph;

	for (i = 0; i < vertexGraphs.size(); i++)
	{
		pVertexGraph = vertexGraphs.at(i);

		delete pVertexGraph;
	}

	vertexGraphs.clear();
}

void TGSet::Save(char *fileName)
{
	if (TGs.size() == 0)
		return;

	// Open file.

	FILE *fp = fopen(fileName, "w");

	if (fp == NULL)
		return;

	// Save template.

	fprintf(fp, "%d\t0\t0\n", A.h);

	int i;
	float *N;

	for (i = 0; i < A.h; i++)
	{
		N = A.Element + 3 * i;

		fprintf(fp, "%f\t%f\t%f\n", N[0], N[1], N[2]);
	}
		
	// Save TGs.	

	TG *pTG;

	for (i = 0; i < TGs.size(); i++)
	{
		pTG = TGs.at(i);

		pTG->Save(fp);
	}

	fclose(fp);

	// Save vertices.

	char *vertexGraphFileName = RVLCreateFileName(fileName, ".tgr", -1, ".vgr");

	fp = fopen(vertexGraphFileName, "w");

	delete[] vertexGraphFileName;

	VertexGraph *pVertexGraph;

	for (i = 0; i < vertexGraphs.size(); i++)
	{
		pVertexGraph = vertexGraphs.at(i);

		pVertexGraph->Save(fp);
	}
	
	fclose(fp);
}

void TGSet::Load(char *fileName)
{
	// Open file.

	FILE *fp = fopen(fileName, "r");

	if (fp == NULL)
		return;

	// Load template.

	int nA;

	fscanf(fp, "%d\t0\t0\n", &nA);

	if (nA > A.h)
	{
		RVL_DELETE_ARRAY(A.Element);

		A.Element = new float[nA];
	}

	A.h = nA;

	int i;
	float *N;

	for (i = 0; i < A.h; i++)
	{
		N = A.Element + 3 * i;

		fscanf(fp, "%f\t%f\t%f\n", N, N + 1, N + 2);
	}

	// Load TGs.

	TG *pTG;

	while (true)
	{
		pTG = new TG;

		pTG->A = A;

		if (!pTG->Load(fp, this))
		{
			delete pTG;

			break;
		}

		TGs.push_back(pTG);
	}

	fclose(fp);

	// Load vertices.

	char *vertexGraphFileName = RVLCreateFileName(fileName, ".tgr", -1, ".vgr");

	fp = fopen(vertexGraphFileName, "r");

	delete[] vertexGraphFileName;

	if (fp == NULL)
		return;

	VertexGraph *pVertexGraph;	

	while (true)
	{
		pVertexGraph = new VertexGraph;

		pVertexGraph->pMem = pMem;

		if (!pVertexGraph->Load(fp))
		{
			delete pVertexGraph;

			break;
		}

		vertexGraphs.push_back(pVertexGraph);
	}

	fclose(fp);
}

VertexGraph *TGSet::GetVertexGraph(TG *pTG)
{
	int i;
	VertexGraph *pVG;

	for (i = 0; i < vertexGraphs.size(); i++)
	{
		pVG = vertexGraphs.at(i);

		if (pVG->idx == pTG->iVertexGraph)
			return pVG;
	}

	return NULL;
}

TG *TGSet::GetTG(int ID)
{
	int i;
	TG *pTG;

	for (i = 0; i < TGs.size(); i++)
	{
		pTG = TGs.at(i);

		if (pTG->iObject == ID)
			return pTG;
	}

	return NULL;
}
