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
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "CTISet.h"
#include <Eigen\Eigenvalues>

using namespace RVL;
using namespace RECOG;

CTISet::CTISet()
{
	SegmentCTIs.Element = NULL;
	segmentCTIIdxMem = NULL;
	CTI.n = 0;
}

CTISet::~CTISet()
{
	RVL_DELETE_ARRAY(SegmentCTIs.Element);
	RVL_DELETE_ARRAY(segmentCTIIdxMem);
}

void CTISet::Load(char *filePath)
{
	FILE *fp = fopen(filePath, "r");

	char line[1600] = { 0 };

	int iModelInstance, iModelInstanceElement, i;

	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;
	RECOG::PSGM_::ModelInstance *pModelInstance;
	CTI.n = 0;
	if (fp)
	{
		//count number of lines in CTI file
		while (!feof(fp))
		{
			line[0] = '\0';

			fgets(line, 1600, fp);

			if (line[0] == '\0' || line[0] == '\n')
				continue;

			CTI.n++;
		}

		rewind(fp);

		CTI.Element = new RECOG::PSGM_::ModelInstance[CTI.n];

		pModelInstance = CTI.Element;

		for (iModelInstance = 0; iModelInstance < CTI.n; iModelInstance++)
		{
			pModelInstance->modelInstance.Element = new RECOG::PSGM_::ModelInstanceElement[nT];

			pModelInstance->modelInstance.n = nT;

			fscanf(fp, "%d\t%d\t", &pModelInstance->iModel, &pModelInstance->iCluster);

			for (i = 0; i < 9; i++)
				fscanf(fp, "%f\t", &pModelInstance->R[i]);

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &pModelInstance->t[i]);

			for (iModelInstanceElement = 0; iModelInstanceElement < nT; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->d);
			}

			for (iModelInstanceElement = 0; iModelInstanceElement < nT; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%d\t", &pModelInstanceElement->valid);
			}

			for (iModelInstanceElement = 0; iModelInstanceElement < nT; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->e);
			}

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &pModelInstance->tc[i]);

			if (iModelInstance == CTI.n - 1)
				pModelInstance->pNext = NULL;
			else
			{
				pModelInstance->pNext = pModelInstance + 1;
				pModelInstance++;
			}
		}

		fclose(fp);
	}

	// Calculate number of scene/model segments
	RECOG::PSGM_::ModelInstance *pCTI;
	RECOG::PSGM_::ModelInstance *pCTINext;
	pCTI = CTI.Element;
	pCTINext = pCTI++;

	int nS = 0; //number of scene/model segments
	int br;
	for (br = 0; br < CTI.n; br++)
	{
		if (pCTI->iCluster != pCTINext->iCluster && br != CTI.n - 1)
			nS++;
		pCTI++;
		pCTINext++;
	}
	nS += 1;


	// nCTI(i) represents number of CTI-s in i-th segment	
	Eigen::VectorXi nCTI(nS);
	int brojac = 0;
	int iC, iM;
	pCTI = CTI.Element;
	RECOG::PSGM_::ModelInstance *pCTIEnd = CTI.Element + CTI.n;

	for (int i = 0; i < nS; i++)
	{
		iM = pCTI->iModel;
		iC = pCTI->iCluster;
		nCTI(i) = 0;
		while (iM == pCTI->iModel && iC == pCTI->iCluster)
		{
			nCTI(i)++;
			pCTI++;
			if (pCTI >= pCTIEnd)
				break;
		}
	}

	// Creates Array of segments, each segment contains CTI indices in that segment
	RVL_DELETE_ARRAY(SegmentCTIs.Element);
	RVL_DELETE_ARRAY(segmentCTIIdxMem);
	SegmentCTIs.Element = new Array<int>[nS];
	SegmentCTIs.n = nS;
	segmentCTIIdxMem = new int[CTI.n];

	int *iSegmentCTIIdx = segmentCTIIdxMem;
	int iCTI = 0;

	for (int i = 0; i < nS; i++)
	{
		SegmentCTIs.Element[i].Element = iSegmentCTIIdx;

		for (int j = 0; j < nCTI(i); j++, iCTI++)
			*(iSegmentCTIIdx++) = iCTI;

		SegmentCTIs.Element[i].n = iSegmentCTIIdx - SegmentCTIs.Element[i].Element;
	}
}
