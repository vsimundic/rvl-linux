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
#include "PSGMCommon.h"
#include "CTISet.h"
//#include <Eigen\Eigenvalues>

using namespace RVL;
using namespace RECOG;

CTISet::CTISet()
{
	SegmentCTIs.Element = NULL;
	segmentCTIIdxMem = NULL;
	pCTI.n = 0;
	pCTI.Element = NULL;
}

CTISet::~CTISet()
{
	RVL_DELETE_ARRAY(SegmentCTIs.Element);
	RVL_DELETE_ARRAY(segmentCTIIdxMem);
	RVL_DELETE_ARRAY(pCTI.Element);
}

void CTISet::Init()
{
	RVLQLIST_INIT((&CTI));

	pCTI.n = 0;
}

void CTISet::Load(char *filePath)
{
	FILE *fp = fopen(filePath, "r");

	char line[3000] = { 0 };

	int iModelInstance, iModelInstanceElement, i;

	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;
	RECOG::PSGM_::ModelInstance *pModelInstance;
	pCTI.n = 0;
	if (fp)
	{
		//count number of lines in CTI file
		while (!feof(fp))
		{
			line[0] = '\0';

			fgets(line, 3000, fp);

			if (line[0] == '\0' || line[0] == '\n')
				continue;

			pCTI.n++;
		}

		rewind(fp);

		//QList<RECOG::PSGM_::ModelInstance> *pCTIQlist = &CTI;

		//RVLQLIST_INIT(pCTIQlist);
		RVLQLIST_INIT((&CTI));

		RECOG::PSGM_::ModelInstance *pQlistEntry;

		//Use Qlist to save CTIs
		for (iModelInstance = 0; iModelInstance < pCTI.n; iModelInstance++)
		{
			pQlistEntry = new RECOG::PSGM_::ModelInstance;

			//RVLQLIST_ADD_ENTRY(pCTIQlist, pQlistEntry);
			RVLQLIST_ADD_ENTRY((&CTI), pQlistEntry);

			pQlistEntry->modelInstance.Element = new RECOG::PSGM_::ModelInstanceElement[nT];

			pQlistEntry->modelInstance.n = nT;

			fscanf(fp, "%d\t%d\t", &pQlistEntry->iModel, &pQlistEntry->iCluster);

			for (i = 0; i < 9; i++)
				fscanf(fp, "%f\t", &pQlistEntry->R[i]);

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &pQlistEntry->t[i]);

			for (iModelInstanceElement = 0; iModelInstanceElement < nT; iModelInstanceElement++)
			{
				pModelInstanceElement = pQlistEntry->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->d);
			}

			for (iModelInstanceElement = 0; iModelInstanceElement < nT; iModelInstanceElement++)
			{
				pModelInstanceElement = pQlistEntry->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%d\t", &pModelInstanceElement->valid);
			}

			for (iModelInstanceElement = 0; iModelInstanceElement < nT; iModelInstanceElement++)
			{
				pModelInstanceElement = pQlistEntry->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->e);
			}

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &pQlistEntry->tc[i]);
		}

		CopyCTIsToArray();

		fclose(fp);
	}
}


void CTISet::AddCTI(RECOG::PSGM_::ModelInstance *pCTI_)
{
	RVLQLIST_ADD_ENTRY((&CTI), pCTI_);

	pCTI.n++;
}

void CTISet::CopyCTIsToArray()
{
	//Copy Qlist to Array
	if (pCTI.n > 0)
	{
		RVL_DELETE_ARRAY(pCTI.Element);

		pCTI.Element = new RECOG::PSGM_::ModelInstance*[pCTI.n];

		QLIST::CreatePtrArray<RECOG::PSGM_::ModelInstance>(&CTI, &pCTI);

		// Calculate number of scene/model segments
		RECOG::PSGM_::ModelInstance *pCTI_;
		RECOG::PSGM_::ModelInstance *pCTINext;

		pCTI_ = CTI.pFirst;
		pCTINext = pCTI_->pNext;

		int nS = 0; //number of scene/model segments

		maxSegmentIdx = 0;

		for (int i = 0; i < pCTI.n - 1; i++)
		{
			if (pCTI_->iCluster != pCTINext->iCluster || pCTI_->iModel != pCTINext->iModel)
				nS++;

			pCTI_ = pCTINext;
			pCTINext = pCTI_->pNext;

			if (pCTI_->iCluster > maxSegmentIdx)
				maxSegmentIdx = pCTI_->iCluster;
		}

		nS += 1;

		nModels = pCTI.Element[pCTI.n - 1]->iModel;

		// nCTI(i) represents number of CTI-s in i-th segment	
		int *nCTI = new int[nS];
		int iC, iM;

		pCTI_ = CTI.pFirst;

		for (int i = 0; i < nS; i++)
		{
			iM = pCTI_->iModel;
			iC = pCTI_->iCluster;

			nCTI[i] = 0;

			while (pCTI_ && iM == pCTI_->iModel && iC == pCTI_->iCluster)
			{
				nCTI[i]++;

				pCTI_ = pCTI_->pNext;
			}
		}

		// Creates Array of segments, each segment contains CTI indices in that segment
		RVL_DELETE_ARRAY(SegmentCTIs.Element);
		RVL_DELETE_ARRAY(segmentCTIIdxMem);

		SegmentCTIs.Element = new Array<int>[nS];
		SegmentCTIs.n = nS;

		segmentCTIIdxMem = new int[pCTI.n];

		int *iSegmentCTIIdx = segmentCTIIdxMem;
		int iCTI = 0;

		for (int i = 0; i < nS; i++)
		{
			SegmentCTIs.Element[i].Element = iSegmentCTIIdx;

			for (int j = 0; j < nCTI[i]; j++, iCTI++)
				*(iSegmentCTIIdx++) = iCTI;

			SegmentCTIs.Element[i].n = nCTI[i];
		}

		delete[] nCTI;
	}
}
