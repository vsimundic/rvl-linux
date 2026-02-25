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
	//CTI.n = 0;
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

void CTISet::Load(
	char *filePath,
	float scale)
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

#ifdef RVLVERSION_171125
			fscanf(fp, "%d\t%d\t", &pQlistEntry->iModel, &pQlistEntry->iCluster);
#else
#ifdef RVLCTIDESCRIPTOR_SAVE_IN_OLD_FORMAT
			fscanf(fp, "%d\t%d\t", &pQlistEntry->iModel, &pQlistEntry->iCluster);
#else
			fscanf(fp, "%d\t%d\t%d\t%d\t", &pQlistEntry->iModel, &pQlistEntry->iPart, &pQlistEntry->iCluster, &pQlistEntry->type);
#endif
#endif

			for (i = 0; i < 9; i++)
				fscanf(fp, "%f\t", &pQlistEntry->R[i]);

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &pQlistEntry->t[i]);

			RVLSCALE3VECTOR(pQlistEntry->t, scale, pQlistEntry->t);

			for (iModelInstanceElement = 0; iModelInstanceElement < nT; iModelInstanceElement++)
			{
				pModelInstanceElement = pQlistEntry->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->d);

				pModelInstanceElement->d *= scale;
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

			RVLSCALE3VECTOR(pQlistEntry->tc, scale, pQlistEntry->tc);
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

		int segmentDiff;

		for (int i = 0; i < pCTI.n - 1; i++)
		{
			if (pCTI_->iCluster != pCTINext->iCluster || pCTI_->iModel != pCTINext->iModel)
			{
				nS++;
				segmentDiff = pCTINext->iCluster - pCTI_->iCluster;
								
				if (segmentDiff > 1)
					nS += segmentDiff - 1;
			}
				

			pCTI_ = pCTINext;
			pCTINext = pCTI_->pNext;

			if (pCTI_->iCluster > maxSegmentIdx)
				maxSegmentIdx = pCTI_->iCluster;
		}

		nS += 1;

		nModels = pCTI.Element[pCTI.n - 1]->iModel + 1;

		if (nModels != 0)
			nS = (nModels) * (maxSegmentIdx + 1);

		// nCTI(i) represents number of CTI-s in i-th segment	
		int *nCTI = new int[nS];
		int iC, iM;

		pCTI_ = CTI.pFirst;

		for (int i = 0; i < nS; i++)
			nCTI[i] = 0;

		pCTI_ = CTI.pFirst;

		for (int i = 0; i < pCTI.n; i++)
		{
			if (pCTI_->iModel == -1)
				nCTI[pCTI_->iCluster]++;
			else
				nCTI[pCTI_->iModel * (maxSegmentIdx + 1) + pCTI_->iCluster]++;

			pCTI_ = pCTI_->pNext;
		}

		/*
		for (int i = 0; i < nS; i++)
		{
			nCTI[i] = 0;

			if (pCTI_)
			{
				iM = pCTI_->iModel;
				iC = pCTI_->iCluster;

				if (iM == -1)
				{
					if (i != iC)
						continue;
				}
				else
					if ((iM * (maxSegmentIdx + 1) + iC) != i)
						continue;

				//if ((iM == -1 && i != iC) || ((iM * (maxSegmentIdx + 1) + iC) != i))
				//	continue;

				while (pCTI_ && iM == pCTI_->iModel && iC == pCTI_->iCluster)
				{
					nCTI[i]++;

					pCTI_ = pCTI_->pNext;
				}
			}
		}*/

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
			if (nCTI[i] == 0)
				SegmentCTIs.Element[i].Element = NULL;
			else
			{
				SegmentCTIs.Element[i].Element = iSegmentCTIIdx;

				for (int j = 0; j < nCTI[i]; j++, iCTI++)
					*(iSegmentCTIIdx++) = iCTI;
			}

			SegmentCTIs.Element[i].n = nCTI[i];
		}

		delete[] nCTI;
	}
}

/*void CTISet::LoadSMCTI(char *filePath, Array<RECOG::PSGM_::Plane> *convexTemplate)
{
	FILE *fp = fopen(filePath, "r");

	char line[1600] = { 0 };

	int iModelInstance, iModelInstanceElement, i;
	float fTmp;

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
			pModelInstance->modelInstance.Element = new RECOG::PSGM_::ModelInstanceElement[convexTemplate->n];

			pModelInstance->modelInstance.n = convexTemplate->n;

			fscanf(fp, "%d\t%d\t", &pModelInstance->iModel, &pModelInstance->iCluster);

			for (i = 0; i < 9; i++)
				fscanf(fp, "%f\t", &pModelInstance->R[i]);

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &pModelInstance->t[i]);

			for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate->n; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->d);
			}

			for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate->n; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%d\t", &pModelInstanceElement->valid);
			}

			for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate->n; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->e);
			}

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &fTmp);

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
	pCTI = CTI.Element;

	int nS = -1; //number of scene/model segments
	int br;
	for (br = 0; br < CTI.n; br++)
	{
		if (pCTI->iCluster > nS)
			nS = pCTI->iCluster;

		pCTI++;
	}
	nS++;

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

}*/