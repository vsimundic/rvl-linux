//#include "stdafx.h"		// remove after moving CRVLPSuLM to RVL2
//#include <highgui.h>
#include "RVLCore.h"
#include "RVL2DLine2.h"		// move to RVLCoreObjectLib.h
#include "RVLPCS.h"
#include "RVLRLM.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include "RVLPSuLMBuilder.h"
//#include "RVLCommon2.h"
//#include "RVLObjectLib.h"
//#include "RVLAImage.h"
//#include "RVLEDT.h"
//#include "RVLDelaunay.h"
//#include "RVLCamera.h"
//#include "RVLStereoVision.h"
//#include "RVLSegmentationEB.h"
//#include "RVLPlanarSurfaceDetector.h"
//#include "RVLStereoVision.h"
//#include "RVLCluster.h"
//#include "RVLRLM.h"
//#include "RVLPSuLMBuilder.h"
//#include <string>
//#include <sstream>
//#include <iostream>
//#include <fstream>
//#include "Include\RVLPSuLM.h"
using namespace std;


CRVLPSuLM::CRVLPSuLM(void)
{
	m_Flags = 0x00000000;
	m_CellArray = NULL;
	m_pRootMeshObject = NULL;//new CRVL3DMeshObject();
	//m_pRootMeshObject->InitParent();
	m_FileName = NULL;
	m_NeighbourList = NULL;
	m_PathPlanningNeighbourArray = NULL;
	m_Name[0] = 0;
}

CRVLPSuLM::~CRVLPSuLM(void)
{
	if(m_pRootMeshObject)
		delete m_pRootMeshObject;

	if(m_FileName)
		delete[] m_FileName;

	if(m_PathPlanningNeighbourArray)
		delete[] m_PathPlanningNeighbourArray;
}


void CRVLPSuLM::Project(CRVL3DPose *pPoseC0,
						BOOL bCells,
						int iSelectedPix,
						CRVL3DSurface2 **ppSelectedSurf,
						CRVL3DLine2 **ppSelectedLine)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	int u, v, w;

	if(iSelectedPix >= 0)
	{
		w = (m_Flags & RVLPSULM_FLAG_COMPLEX ? pBuilder->m_pCamera->m_wSpherical : pBuilder->m_pCamera->Width);
		u = ((iSelectedPix % w) << 1) + 1;
		v = ((iSelectedPix / w) << 1) + 1;
	}

	//if(pBuilder->m_Flags & RVLPSULM_FLAG_SURFACES)
	//{
		CRVLClass *p2DContourSet = &(pBuilder->m_2DContourSet);

		CRVLMPtrChain *p2DContourList = &(p2DContourSet->m_ObjectList);

		CvPoint *PtBuff = (CvPoint *)(pBuilder->m_PtBuff);	

		int *iSampleCellArray = pBuilder->m_iSampleCellArray;

		CRVLStereoVision *pStereoVision = pBuilder->m_pStereoVision;

		BOOL bKinect = ((pStereoVision->m_Flags & RVLSTEREO_FLAGS_METHOD) == RVLSTEREO_FLAGS_METHOD_KINECT);

		CRVLCamera *pCamera = pBuilder->m_pCamera;

		double *RC0 = pPoseC0->m_Rot;
		double *tC0 = pPoseC0->m_X;

		double P[3 * 3];

		pBuilder->GetProjectionMatrix(P);

		double A[3 * 3];

		RVLMXMUL3X3T2(P, RC0, A);

		double R0C[9], t0C[3];

		if(m_Flags & RVLPSULM_FLAG_COMPLEX)
			RVLINVTRANSF3D(RC0, tC0, R0C, t0C)

		// clear cell array

		RVLPSULM_CELL *CellArray = pBuilder->m_CellArray;
		RVLPSULM_CELL_CONST *CellConstArray = pBuilder->m_CellConstArray;

		if(bCells)
			memcpy(CellArray, pBuilder->m_EmptyCellArray, pBuilder->m_nCells * sizeof(RVLPSULM_CELL));

		// project lines

		if(iSelectedPix >= 0 && ppSelectedLine != NULL)
		{
			*ppSelectedLine = NULL;

			int iUSelected[2];

			iUSelected[0] = u;
			iUSelected[1] = v;

			CRVL3DLine2 *pLine;
			CvPoint Pt1, Pt2;
			BYTE CropSide;
			CRVL2DLine2 Line2D;
			int dist;
			BYTE bOut;
			int iU1[2], iU2[2];

			m_3DLineList.Start();

			while(m_3DLineList.m_pNext)
			{
				pLine = (CRVL3DLine2 *)(m_3DLineList.GetNext());

				if(m_Flags & RVLPSULM_FLAG_COMPLEX)
					bOut = RVLCrop3DLineSpherical(pLine->m_X[0], pLine->m_X[1], R0C, t0C, pBuilder->m_pCamera,
						&(pBuilder->m_ROI), pBuilder->m_CropLTs.minz, pBuilder->m_CropLTs.bOutLT, &Pt1, &Pt2, CropSide);
				else
					bOut = RVLCrop3DLine(pLine->m_X[0], pLine->m_X[1], A, tC0, &(pBuilder->m_ROI),
						pBuilder->m_CropLTs.minz, pBuilder->m_CropLTs.minr, pBuilder->m_CropLTs.bOutLT, 
						iU1, iU2, &Pt1, &Pt2, CropSide);

				Line2D.m_iU[0][0] = Pt1.x;
				Line2D.m_iU[0][1] = Pt1.y;
				Line2D.m_iU[1][0] = Pt2.x;
				Line2D.m_iU[1][1] = Pt2.y;

				Line2D.SetdiU();

				if(Line2D.m_leniU > 0)
				{
					dist = Line2D.Distance(iUSelected);

					if(dist < 4)
					{
						*ppSelectedLine = pLine;

						break;
					}
				}
			}
		}

		// project surfaces

		p2DContourList->RemoveAll();

		int abcNrm = 2 * pCamera->Width + pCamera->Height;
		int abcNrm2 = 2 * abcNrm;
		double fabcNrm = (double)(bKinect ? abcNrm : 16 * abcNrm);  

		int iSurface = 0;		// only for debugging purpose!!!

		if(ppSelectedSurf)
			*ppSelectedSurf = NULL;

		if(m_SurfaceList.m_nElements == 0)
			return;

		m_SurfaceList.Start();

		CRVL3DSurface2 *pSurf = (CRVL3DSurface2 *)(m_SurfaceList.GetNext());

		CRVLClass *p3DSurfaceSet = pSurf->m_pClass;

		int HistRGBBaseLog2 = p3DSurfaceSet->m_HistRGBBaseLog2;
		int HistRGBBaseSquareLog2 = (HistRGBBaseLog2 << 1);
		ushort mask = ((1 << HistRGBBaseLog2) - 1);

		CRVL3DSurface2 **ppSurfArrayEnd = m_3DSurfaceArray + m_n3DSurfacesTotal;

		CRVL3DSurface2 *pConvexSegment;
		CRVL3DSurface2 **ppConvexSegment, **pConvexSegmentArrayEnd;
		RVLPTRCHAIN_ELEMENT *pLast;
		//int nBoundaries;
		RVLARRAY *pRelList;
		int *piSampleCellArrayEnd;
		RVLPSULM_CELL *pCell;
		RVLPSULM_CELL_CONST *pCellConst;
		int *piCell;
		double N[3];
		double d;
		double fa, fb, fc;
		int a, b, c;
		short disparity;
		int nContours, nNewContours;
		RVLCOLOR Color;
		RVLQLIST_HIST_ENTRY_SHORT *pHistRGBEntry;
		ushort HistRGBBin;
		CRVL3DSurface2 **ppSurf;

		for(ppSurf = m_3DSurfaceArray; ppSurf < ppSurfArrayEnd; ppSurf++)
		{
			pSurf = *ppSurf;

			//if(pSurf->m_Index == 116)
			//	int debug = 0;

			//pHistRGBEntry = (RVLQLIST_HIST_ENTRY_SHORT *)(pSurf->m_histRGB->pFirst);

			//if(pHistRGBEntry)
			//{
			//	HistRGBBin = pHistRGBEntry->adr;

			//	Color.r = (BYTE)(HistRGBBin >> HistRGBBaseSquareLog2);
			//	Color.g = (BYTE)((HistRGBBin >> HistRGBBaseLog2) & mask);
			//	Color.b = (BYTE)(HistRGBBin & mask);
			//}
		
			//if((int)pSurf == 0x03dac990)
			//	int debug = 0;

			//if(iSurface == 86)
			//	int tmp1 = 0;

			//iSurface++;		// only for debugging purpose!!!

			RVL3DSurfaceInvTransf(pSurf->m_N, pSurf->m_d, pPoseC0, N, d);

			if(d * d < pBuilder->m_CropLTs.minr)
				continue;

			pStereoVision->GetUVDPlane(N, d, fa, fb, fc);

			a = DOUBLE2INT(fabcNrm * fa);
			b = DOUBLE2INT(fabcNrm * fb);
			c = DOUBLE2INT(fabcNrm * (2.0 * fc - fa - fb) + abcNrm);

#ifdef NEVER	// old version
			pRelList = pSurf->m_RelList + pSurf->m_pClass->m_iRelListContours;

			nBoundaries = (CRVL2DContour **)(pRelList->pEnd) - (CRVL2DContour **)(pRelList->pFirst);
		
			pLast = (p2DContourList->m_pFirst ? p2DContourList->m_pLast : NULL);
#endif

#ifdef RVLPSULM_CONVEX_SEGMENTS
			pRelList = pSurf->m_RelList + pSurf->m_pClass->m_iRelListComponents;

			pConvexSegmentArrayEnd = (CRVL3DSurface2 **)(pRelList->pEnd);

			for(ppConvexSegment = (CRVL3DSurface2 **)(pRelList->pFirst); ppConvexSegment < pConvexSegmentArrayEnd;
				ppConvexSegment++)
			{
				pConvexSegment = *ppConvexSegment;

				if(pConvexSegment->m_Flags & RVLOBJ2_FLAG_REJECTED)
					continue;

				pLast = (p2DContourList->m_pFirst ? p2DContourList->m_pLast : NULL);

				nContours = p2DContourList->m_nElements;
				
				pConvexSegment->Crop(pPoseC0, pCamera, &(pBuilder->m_ROI), &(pBuilder->m_CropLTs), 
					&PtBuff, p2DContourSet);

				p2DContourList->m_pNext = (pLast ? pLast->pNext : p2DContourList->m_pFirst);

				nNewContours = p2DContourList->m_nElements - nContours;

				if(iSelectedPix >= 0)
					if(ppSelectedSurf)
						if(RVLIsInsideContour(p2DContourList, nNewContours, u, v))
							*ppSelectedSurf = pSurf;

				p2DContourList->m_pNext = (pLast ? pLast->pNext : p2DContourList->m_pFirst);

				if(bCells)
				{
					pBuilder->RegionSampling(p2DContourList, nNewContours);

					piSampleCellArrayEnd = iSampleCellArray + pBuilder->m_nSampleCells;

					for(piCell = iSampleCellArray; piCell < piSampleCellArrayEnd; piCell++)
					{
						pCell = CellArray + (*piCell);

						pCellConst = CellConstArray + (*piCell);

						disparity = (short)((a * pCellConst->u + b * pCellConst->v + c) / abcNrm2);

						if(disparity < pCell->disparity)
						{
							pCell->disparity = disparity;

							if(pHistRGBEntry)
							{
								pCell->color = Color;
								pCell->Flags |= RVLPSULM_CELL_FLAG_COLOR;
							}
						}
					}
				}
			}	// for each convex segment
#else
			if(iSelectedPix >= 0 && ppSelectedSurf != NULL && *ppSelectedLine == NULL)
			{
				int nPts;

				CRVL3DPose PoseMC;

				InverseTransform3D(PoseMC.m_Rot, PoseMC.m_X, pPoseC0->m_Rot, pPoseC0->m_X);

				CRVL3DPose PoseFC;

				RVLCombineTransform3D(PoseMC.m_Rot, PoseMC.m_X, pSurf->m_Pose.m_Rot, pSurf->m_Pose.m_X, PoseFC.m_Rot, PoseFC.m_X);

				CvPoint *PtArray;

				RVLDisplay3DEllipse(pSurf->m_EigenValues[0], pSurf->m_EigenValues[1], 1.0, &PoseFC, 
					pBuilder->m_pCamera, &PtArray, nPts);
				
				if(nPts > 0)
				{
					if(RVLIsInsideContour(PtArray, nPts, u, v))
						*ppSelectedSurf = pSurf;

					delete[] PtArray;
				}
			}

#endif
		}	// for each surface
	//}
}


// match pSurface to all surfaces in PSuLM by surface sampling

int CRVLPSuLM::Match(	CRVL3DSurface2 *pSurface, 
						CRVL3DPose *pPoseC0)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLClass *p2DContourSet = &(pBuilder->m_2DContourSet);

	CRVLMPtrChain *p2DContourList = &(p2DContourSet->m_ObjectList);

	CvPoint *PtBuff = (CvPoint *)(pBuilder->m_PtBuff);	

	int *iSampleCellArray = pBuilder->m_iSampleCellArray;

	CRVLStereoVision *pStereoVision = pBuilder->m_pStereoVision;

	BOOL bKinect = ((pStereoVision->m_Flags & RVLSTEREO_FLAGS_METHOD) == RVLSTEREO_FLAGS_METHOD_KINECT);

	CRVLCamera *pCamera = pBuilder->m_pCamera;

	RVLPSULM_CELL *CellArray = pBuilder->m_CellArray;

	RVLPSULM_CELL_CONST *CellConstArray = pBuilder->m_CellConstArray;

	int thr = pBuilder->m_ProjDisparityErr; 

	//memcpy(CellArray, pBuilder->m_EmptyCellArray, pBuilder->m_nCells * sizeof(RVLPSULM_CELL));

	// project surfaces
	p2DContourList->RemoveAll();

	int abcNrm = 2 * pCamera->Width + pCamera->Height;
	int abcNrm2 = 2 * abcNrm;
	double fabcNrm = (double)(bKinect ? abcNrm : 16 * abcNrm);  


	CRVL3DSurface2 *pConvexSegment;
	CRVL3DSurface2 **ppConvexSegment, **pConvexSegmentArrayEnd;
	RVLPTRCHAIN_ELEMENT *pLast;

	RVLARRAY *pRelList;
	int *piSampleCellArrayEnd;
	RVLPSULM_CELL *pCell;
	RVLPSULM_CELL_CONST *pCellConst;
	int *piCell;
	double N[3];
	double d;
	double fa, fb, fc;
	int a, b, c;
	short disparity;
	int nContours, nNewContours;

	
	RVL3DSurfaceInvTransf(pSurface->m_N, pSurface->m_d, pPoseC0, N, d);

	if(d * d < pBuilder->m_CropLTs.minr)
		return 0;

	pStereoVision->GetUVDPlane(N, d, fa, fb, fc);

	a = DOUBLE2INT(fabcNrm * fa);
	b = DOUBLE2INT(fabcNrm * fb);
	c = DOUBLE2INT(fabcNrm * (2.0 * fc - fa - fb) + abcNrm);

	int cost = 0;

	pRelList = pSurface->m_RelList + pSurface->m_pClass->m_iRelListComponents;

	pConvexSegmentArrayEnd = (CRVL3DSurface2 **)(pRelList->pEnd);

	int iCell;
	int abse;

	for(ppConvexSegment = (CRVL3DSurface2 **)(pRelList->pFirst); ppConvexSegment < pConvexSegmentArrayEnd;
		ppConvexSegment++)
	{
		pConvexSegment = *ppConvexSegment;

		if(pConvexSegment->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pLast = (p2DContourList->m_pFirst ? p2DContourList->m_pLast : NULL);

		nContours = p2DContourList->m_nElements;
		
		pConvexSegment->Crop(pPoseC0, pCamera, &(pBuilder->m_ROI), &(pBuilder->m_CropLTs), 
			&PtBuff, p2DContourSet);

		p2DContourList->m_pNext = (pLast ? pLast->pNext : p2DContourList->m_pFirst);

		nNewContours = p2DContourList->m_nElements - nContours;

		p2DContourList->m_pNext = (pLast ? pLast->pNext : p2DContourList->m_pFirst);

		pBuilder->RegionSampling(p2DContourList, nNewContours);

		piSampleCellArrayEnd = iSampleCellArray + pBuilder->m_nSampleCells;

		for(piCell = iSampleCellArray; piCell < piSampleCellArrayEnd; piCell++)
		{
			iCell = *piCell;

			pCell = m_CellArray + iCell;

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
			if(pCell->disparity < 0)
			{
				//cost++;

				continue;
			}
#endif

			pCellConst = CellConstArray + iCell;

			disparity = (short)((a * pCellConst->u + b * pCellConst->v + c) / abcNrm2);

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
			CellArray[iCell].disparity = disparity;

			if(pCell->disparity < 0)
			{
				//cost++;

				continue;
			}
#endif

			if((abse = abs(disparity - pCell->disparity)) > thr)
			{
				cost--;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG			
				CellArray[iCell].Flags |= RVLPSULM_CELL_FLAG_ERROR;
#endif
			}
			else	
			{
				cost += (thr - abse);
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
				CellArray[iCell].Flags |= RVLPSULM_CELL_FLAG_MATCHED;
#endif
			}
		}
	}

	return cost;
}


// match pSurface to all surfaces in PSuLM using Mahalanobis distance

int CRVLPSuLM::Match2(	CRVL3DSurface2 *pSurface, 
						CRVL3DPose *pPoseSM)
{
	int score = 0;

	//CRVL3DSurface2 **pMSurfArrayEnd = m_3DSurfaceArray + m_n3DSurfaces;
	CRVL3DSurface2 **pMSurfArrayEnd = m_3DSurfaceArray + (m_n3DSurfaces <= 20 ? m_n3DSurfaces : 20);

	//double MinMatchCost = 12.0;

	int bestScore = 0;

	CRVL3DSurface2 **ppMSurf;
	CRVL3DSurface2 *pMSurface;
	double MatchCost;
	double detQ;
	RVLSURFACE_MATCH_ARRAY MatchData;
	CRVL3DSurface2 *pBestMatch;

	for(ppMSurf = m_3DSurfaceArray; ppMSurf < pMSurfArrayEnd; ppMSurf++)
	{
		pMSurface = *ppMSurf;

		//if(pMSurface->m_Flags & RVLOBJ2_FLAG_MATCHED)
		//	continue;

		if(pSurface->Match2(pMSurface, pPoseSM, MatchCost, detQ, &MatchData))
		{
			score = chi2_LookUpTable[DOUBLE2INT(100.0 * MatchCost)] * 
				(pSurface->m_nSupport <= pMSurface->m_nSupport ? pSurface->m_nSupport : pMSurface->m_nSupport);
			//score = (pSurface->m_nSupport <= pMSurface->m_nSupport ? pSurface->m_nSupport : pMSurface->m_nSupport);

			if(score > bestScore)
			{
				bestScore = score;

				pBestMatch = pMSurface;
			}

			//if(MatchCost < MinMatchCost)
			//	MinMatchCost = MatchCost;

			//score += (chi2_LookUpTable[DOUBLE2INT(100.0 * MatchCost)] * 
			//	(pSurface->m_nSupport <= pMSurface->m_nSupport ? pSurface->m_nSupport : pMSurface->m_nSupport));
		}
	}

	//if(bestScore > 0)
	//	pBestMatch->m_Flags |= RVLOBJ2_FLAG_MATCHED;

	//return score;
	return bestScore;
	
	//return (MinMatchCost < 11.9 ? chi2_LookUpTable[DOUBLE2INT(100.0 * MinMatchCost)] * pSurface->m_nSupport : 0);

}

void CRVLPSuLM::Project3DSurface(CRVL3DSurface2 *pSurf, 
								 CRVL3DPose *pPoseC0)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLClass *p2DContourSet = &(pBuilder->m_2DContourSet);

	CRVLMPtrChain *p2DContourList = &(p2DContourSet->m_ObjectList);

	CvPoint *PtBuff = (CvPoint *)(pBuilder->m_PtBuff);	

	CRVLCamera *pCamera = pBuilder->m_pCamera;

	p2DContourList->RemoveAll();

	CRVL3DSurface2 *pConvexSegment;
	CRVL3DSurface2 **ppConvexSegment, **pConvexSegmentArrayEnd;
	RVLARRAY *pRelList;
	double N[3];
	double d;

	RVL3DSurfaceInvTransf(pSurf->m_N, pSurf->m_d, pPoseC0, N, d);

	if(d * d < pBuilder->m_CropLTs.minr)
		return;

	pRelList = pSurf->m_RelList + pSurf->m_pClass->m_iRelListComponents;

	pConvexSegmentArrayEnd = (CRVL3DSurface2 **)(pRelList->pEnd);

	for(ppConvexSegment = (CRVL3DSurface2 **)(pRelList->pFirst); ppConvexSegment < pConvexSegmentArrayEnd;
		ppConvexSegment++)
	{
		pConvexSegment = *ppConvexSegment;

		if(pConvexSegment->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pConvexSegment->Crop(pPoseC0, pCamera, &(pBuilder->m_ROI), &(pBuilder->m_CropLTs), 
			&PtBuff, p2DContourSet);
	}
}

void CRVLPSuLM::GetCenter(double *XCenter)
{	
	RVLNULL3VECTOR(XCenter);

	int nPts = 0;

	double *X;

	CRVL3DSurface2 *pSurf;
	CRVL3DSurface2 *pConvexSegment;
	CRVL3DSurface2 **ppConvexSegment;
	RVLARRAY *pRelList;
	CRVL3DSurface2 **pSegmentArrayEnd;
	double w;

	//CRVL3DContour *pBoundary;
	//CRVL3DContour **ppBoundary;
	//double *pBoundaryEnd;

	m_SurfaceList.Start();

	while(m_SurfaceList.m_pNext)
	{
		pSurf = (CRVL3DSurface2 *)(m_SurfaceList.GetNext());

#ifdef RVLPSULM_CONVEX_SEGMENTS
		pRelList = pSurf->m_RelList + pSurf->m_pClass->m_iRelListComponents;

		pSegmentArrayEnd = (CRVL3DSurface2 **)(pRelList->pEnd);

		for(ppConvexSegment = (CRVL3DSurface2 **)(pRelList->pFirst); ppConvexSegment < pSegmentArrayEnd; ppConvexSegment++)
		{
			pConvexSegment = *ppConvexSegment;

			if(pConvexSegment->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;
					
			w = (double)(pConvexSegment->m_nSupport);

			XCenter[0] += (w * pConvexSegment->m_Pose.m_X[0]);
			XCenter[1] += (w * pConvexSegment->m_Pose.m_X[1]);
			XCenter[2] += (w * pConvexSegment->m_Pose.m_X[2]);

			nPts += pConvexSegment->m_nSupport;
		}
#else
		w = (double)(pSurf->m_nSupport);

		XCenter[0] += (w * pSurf->m_Pose.m_X[0]);
		XCenter[1] += (w * pSurf->m_Pose.m_X[1]);
		XCenter[2] += (w * pSurf->m_Pose.m_X[2]);

		nPts += pSurf->m_nSupport;
#endif

	//  // old method

	//	pRelList = pSurf->m_RelList + pSurf->m_pClass->m_iRelListContours;

	//	CRVL3DContour **pBoundaryPtrArrayEnd = (CRVL3DContour **)(pRelList->pEnd);				

	//	for(ppBoundary = (CRVL3DContour **)(pRelList->pFirst); ppBoundary < pBoundaryPtrArrayEnd; ppBoundary++)
	//	{
	//		pBoundary = *ppBoundary;

	//		X = (double *)(pBoundary->m_PtArray);

	//		pBoundaryEnd = X + 3 * pBoundary->m_nPts;

	//		for(; X < pBoundaryEnd; X += 3)
	//		{
	//			XCenter[0] += X[0];
	//			XCenter[1] += X[1];
	//			XCenter[2] += X[2];

	//			nPts++;
	//		}
	//	}
	}	

	CRVL3DLine2 *pLine;

	m_3DLineList.Start();

	while(m_3DLineList.m_pNext)
	{
		pLine = (CRVL3DLine2 *)(m_3DLineList.GetNext());

		X = pLine->m_X[0];

		XCenter[0] += X[0];
		XCenter[1] += X[1];
		XCenter[2] += X[2];

		X = pLine->m_X[1];

		XCenter[0] += X[0];
		XCenter[1] += X[1];
		XCenter[2] += X[2];
		
		nPts += 2;
	}

	double fn = (double)nPts;

	XCenter[0] /= fn;
	XCenter[1] /= fn;
	XCenter[2] /= fn;
}

void CRVLPSuLM::Display(CRVLFigure * pFig,
						CRVL3DPose *pPoseM0,
						CvScalar Color,
						DWORD Flags,
						int iSelectedPix,
						CRVL3DSurface2 **ppSelectedSurf,
						CRVL3DLine2 **ppSelectedLine)
{
	if(Flags & RVLPSULM_DISPLAY_CLEAR)
		cvSet(pFig->m_pImage, cvScalar(0, 0, 0));

	unsigned char *ImageData;
	int wStep;

	CRVLDisplayVector Vector;

	RVLCOLOR ColorMarked;
	
	if(Flags & RVLPSULM_DISPLAY_VECTORS)
	{
		Vector.m_rP = (BYTE)DOUBLE2INT(Color.val[0]);
		Vector.m_gP = (BYTE)DOUBLE2INT(Color.val[1]);
		Vector.m_bP = (BYTE)DOUBLE2INT(Color.val[2]);
		Vector.m_rL = (BYTE)DOUBLE2INT(Color.val[0]);
		Vector.m_gL = (BYTE)DOUBLE2INT(Color.val[1]);
		Vector.m_bL = (BYTE)DOUBLE2INT(Color.val[2]);

		Vector.m_PointArray.Create(pFig->m_pMem, sizeof(RVLGUI_POINT));

		ColorMarked.r = (BYTE)(0.25 * Color.val[0] + 0.75 * 255.0);
		ColorMarked.g = (BYTE)(0.25 * Color.val[1] + 0.75 * 255.0);
		ColorMarked.b = (BYTE)(0.25 * Color.val[2] + 0.75 * 255.0);
	}
	else
	{
		ImageData = (unsigned char *)(pFig->m_pImage->imageData);

		wStep = pFig->m_pImage->widthStep;
	}

	CRVLDisplayVector *pVector;

	CRVLPSuLMBuilder *pPSuLMBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	int w = pPSuLMBuilder->m_pPSD->m_Width;
	int h = pPSuLMBuilder->m_pPSD->m_Height;

	CRVL3DPose Pose0M;

	InverseTransform3D(Pose0M.m_Rot, Pose0M.m_X, pPoseM0->m_Rot, pPoseM0->m_X);

	CRVL3DPose PoseCM;

	RVLCombineTransform3D(Pose0M.m_Rot, Pose0M.m_X, pFig->m_PoseC0.m_Rot, pFig->m_PoseC0.m_X, PoseCM.m_Rot, PoseCM.m_X);

	double *RCM = PoseCM.m_Rot;
	double *tCM = PoseCM.m_X;

	double P[3 * 3];

	pPSuLMBuilder->GetProjectionMatrix(P);

	double A[3 * 3];

	RVLMXMUL3X3T2(P, RCM, A);

	double RMC[9], tMC[3];

	if(m_Flags & RVLPSULM_FLAG_COMPLEX)
		RVLINVTRANSF3D(RCM, tCM, RMC, tMC)

	// display surfaces

	if(Flags & RVLPSULM_DISPLAY_SURFACES)
	{
		CRVLMPtrChain *p2DContourList = &(pPSuLMBuilder->m_2DContourSet.m_ObjectList);

		pPSuLMBuilder->m_2DContourSet.m_pMem0 = pFig->m_pMem;

		BYTE *pMem = pFig->m_pMem->m_pFreeMem;

		CRVL2DContour *p2DContour;
		CvPoint *pPt;
		CvPoint *pContourEnd;
		CvPoint *PtArray;

		double StartTime = pPSuLMBuilder->m_pTimer->GetTime();

		Project(&PoseCM, FALSE, iSelectedPix, ppSelectedSurf, ppSelectedLine);

		double ExecutionTime = pPSuLMBuilder->m_pTimer->GetTime() - StartTime;

#ifdef RVLPSULM_CONVEX_SEGMENTS
		p2DContourList->Start();

		while(p2DContourList->m_pNext)
		{
			p2DContour = (CRVL2DContour *)(p2DContourList->GetNext());

			PtArray = (CvPoint *)(p2DContour->m_ContourIPArray);
			pContourEnd = PtArray + p2DContour->m_nContourIPs;

			if(Flags & RVLPSULM_DISPLAY_VECTORS)
			{
				pVector = pFig->AddVector(&Vector);

				for(pPt = PtArray; pPt < pContourEnd; pPt++)
					pVector->Point(pPt->x, pPt->y);
			}
			else
			{
				for(pPt = PtArray; pPt < pContourEnd; pPt++)
				{
					pPt->x /= 2;
					pPt->y /= 2;
				}

				cvPolyLine(pFig->m_pImage, &PtArray, &(p2DContour->m_nContourIPs), 1, 1, Color);
			}
		}

		pFig->m_pMem->m_pFreeMem = pMem;

		if(Flags & RVLPSULM_DISPLAY_CELLS)
		{
			pPSuLMBuilder->DisplayCellArray(pFig, m_CellArray, &PoseCM, Color); 

			if(Flags & RVLPSULM_DISPLAY_PROJ_CELLS)
				pPSuLMBuilder->DisplayCellArray(pFig, pPSuLMBuilder->m_CellArray, &PoseCM, 
					cvScalar(255-Color.val[0], 255-Color.val[1], 255-Color.val[2])); 
		}
#else
		CRVL3DSurface2 **ppSurfArrayEnd = m_3DSurfaceArray + m_n3DSurfacesTotal;

		CRVL3DSurface2 **ppSurf;
		CRVL3DSurface2 *pSurf;
		CRVL3DPose PoseMC, PoseFC;
		int nPts;
		RVL3DSURFACE_SAMPLE *pSample;
		double *RFC, *tFC;

		for(ppSurf = m_3DSurfaceArray; ppSurf < ppSurfArrayEnd; ppSurf++)
		{
			pSurf = *ppSurf;

			//if(ppSurf - m_3DSurfaceArray < m_n3DSurfaces)
			if(Flags & RVLPSULM_DISPLAY_ELLIPSES)
			{
				InverseTransform3D(PoseMC.m_Rot, PoseMC.m_X, PoseCM.m_Rot, PoseCM.m_X);

				RFC = PoseFC.m_Rot;
				tFC = PoseFC.m_X;

				RVLCombineTransform3D(PoseMC.m_Rot, PoseMC.m_X, pSurf->m_Pose.m_Rot, pSurf->m_Pose.m_X, RFC, tFC);

				RVLDisplay3DEllipse(pSurf->m_EigenValues[0], pSurf->m_EigenValues[1], 1.0, &PoseFC, pPSuLMBuilder->m_pCamera,
					&PtArray, nPts);

				if(nPts > 0)
				{
					pContourEnd = PtArray + nPts;

					if(Flags & RVLPSULM_DISPLAY_VECTORS)
					{
						pVector = pFig->AddVector(&Vector);

						pVector->m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
						pVector->m_bClosed = TRUE;

						if(pSurf->m_Flags & RVLOBJ2_FLAG_MARKED)
						{
							pVector->m_rL = ColorMarked.r;
							pVector->m_gL = ColorMarked.g;
							pVector->m_bL = ColorMarked.b;
						}

						for(pPt = PtArray; pPt < pContourEnd; pPt++)
							pVector->Point(pPt->x, pPt->y);
					}
					else
					{
						for(pPt = PtArray; pPt < pContourEnd; pPt++)
						{
							pPt->x /= 2;
							pPt->y /= 2;
						}

						cvPolyLine(pFig->m_pImage, &PtArray, &nPts, 1, 1, Color);
					}

					delete[] PtArray;
				}
			}

			if(Flags & RVLPSULM_DISPLAY_SAMPLES)
				Display3DSurfaceSamples(pFig, pSurf, A, tCM, Flags | 
					RVLPSULM_DISPLAY_SAMPLE_REGIONS | 
					RVLPSULM_DISPLAY_SAMPLE_TYPES | 
					RVLPSULM_DISPLAY_COLOR
					);
		}
#endif
	}

	// display lines

	if(Flags & RVLPSULM_DISPLAY_LINES)
	{
		CRVL3DLine2 *pLine;
		BYTE bOut;
		int iU1[2], iU2[2];
		CvPoint Pt1, Pt2;
		BYTE CropSide;
		//double X1[3], X2[3];

		m_3DLineList.Start();

		while(m_3DLineList.m_pNext)
		{
			pLine = (CRVL3DLine2 *)(m_3DLineList.GetNext());

			if(m_Flags & RVLPSULM_FLAG_COMPLEX)
				bOut = RVLCrop3DLineSpherical(pLine->m_X[0], pLine->m_X[1], RMC, tMC, pPSuLMBuilder->m_pCamera,
					&(pPSuLMBuilder->m_ROI), pPSuLMBuilder->m_CropLTs.minz, pPSuLMBuilder->m_CropLTs.bOutLT, &Pt1, &Pt2,
					CropSide);
			else
				bOut = RVLCrop3DLine(pLine->m_X[0], pLine->m_X[1], A, tCM, &(pPSuLMBuilder->m_ROI),
					pPSuLMBuilder->m_CropLTs.minz, pPSuLMBuilder->m_CropLTs.minr, pPSuLMBuilder->m_CropLTs.bOutLT, 
					iU1, iU2, &Pt1, &Pt2, CropSide);

			if(bOut & 0x04)
				continue;

			if(Flags & RVLPSULM_DISPLAY_VECTORS)
			{
				pVector = pFig->AddVector(&Vector);

				pVector->m_PointType = RVLGUI_POINT_DISPLAY_TYPE_SQUARE;
				pVector->m_bClosed = FALSE;
				pVector->m_LineWidth = 2;

				if(pLine->m_Flags & RVLOBJ2_FLAG_MARKED)
				{
					pVector->m_rP = 255;
					pVector->m_gP = 192;
					pVector->m_bP = 255;
					pVector->m_rL = 255;
					pVector->m_gL = 192;
					pVector->m_bL = 255;
				}
				else
				{
					pVector->m_rP = 255;
					pVector->m_gP = 0;
					pVector->m_bP = 255;
					pVector->m_rL = 255;
					pVector->m_gL = 0;
					pVector->m_bL = 255;
				}

				pVector->Line(Pt1.x, Pt1.y, Pt2.x, Pt2.y);
			}
			else
			{
				Pt1.x /= 2;
				Pt1.y /= 2;
				Pt2.x /= 2;
				Pt2.y /= 2;

				cvLine(pFig->m_pImage, Pt1, Pt2, cvScalar(255, 0, 255), 2);
			}
		}
	}
}

void CRVLPSuLM::Display3DSurfaceSamples(CRVLFigure * pFig,
										CRVL3DSurface2 *pSurf,
										double *A, 
										double *tCM,
										DWORD Flags)
{
	CRVLPSuLMBuilder *pPSuLMBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	int w = pPSuLMBuilder->m_pPSD->m_Width;
	int h = pPSuLMBuilder->m_pPSD->m_Height;

	CRVLDisplayVector Vector;
	unsigned char *ImageData;
	int wStep;

	if(Flags & RVLPSULM_DISPLAY_VECTORS)
		Vector.m_PointArray.Create(pFig->m_pMem, sizeof(RVLGUI_POINT));	
	else
	{
		ImageData = (unsigned char *)(pFig->m_pImage->imageData);

		wStep = pFig->m_pImage->widthStep;
	}

	RVL3DSURFACE_SAMPLE *pSample = (RVL3DSURFACE_SAMPLE *)(pSurf->m_Samples.pFirst);

	double *XM;
	double XC[3], tmp3x1[3];
	int u, v;
	CRVLDisplayVector *pVector;
	unsigned char *pPix;
	CvScalar Color;
	int wSampleRegion;
	CvRect SampleRegion;

	while(pSample)
	{
		XM = pSample->X;

		RVLDIF3VECTORS(XM, tCM, tmp3x1)
		RVLMULMX3X3VECT(A, tmp3x1, XC)

		u = DOUBLE2INT(XC[0] / XC[2]);
		v = DOUBLE2INT(XC[1] / XC[2]);

		if(Flags & RVLPSULM_DISPLAY_COLOR)
		{
			if(Flags & RVLPSULM_DISPLAY_SAMPLE_TYPES)
			{
				switch(pSample->Flags){
				case RVL3DSURFACE_SAMPLE_FLAG_MATCHED:
					Color = cvScalar(0, 255, 0);

					break;
				case RVL3DSURFACE_SAMPLE_FLAG_OCLUDED:
					Color = cvScalar(255, 255, 0);

					break;
				case RVL3DSURFACE_SAMPLE_FLAG_REMOVED:
					Color = cvScalar(255, 0, 0);

					break;
				default:
					Color = cvScalar(0, 0, 255);
				}
			}
			else
				Color = cvScalar(0, 255, 0);
		}
		else
			Color = cvScalar(255, 255, 255);

		if(u >= 0 && u < w && v >= 0 && v < h)
		{
			if(Flags & RVLPSULM_DISPLAY_VECTORS)
			{
				if(Flags & RVLPSULM_DISPLAY_SAMPLE_CENTERS)
				{
					pVector = pFig->AddVector(&Vector);

					pVector->m_PointType = RVLGUI_POINT_DISPLAY_TYPE_SQUARE;

					pVector->m_rP = (unsigned char)DOUBLE2INT(Color.val[0]);
					pVector->m_gP = (unsigned char)DOUBLE2INT(Color.val[1]);
					pVector->m_bP = (unsigned char)DOUBLE2INT(Color.val[2]);
					
					pVector->m_bClosed = FALSE;

					pVector->Point(u << 1, v << 1);
				}

				if(Flags & RVLPSULM_DISPLAY_SAMPLE_REGIONS)
				{
					wSampleRegion = DOUBLE2INT(pSample->wz / XM[2]);

					SampleRegion.x = ((u - wSampleRegion) << 1);
					SampleRegion.y = ((v - wSampleRegion) << 1);
					SampleRegion.width = SampleRegion.height = (wSampleRegion << 2);

					pVector = pFig->AddVector(&Vector);

					pVector->m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
					pVector->Rect(&SampleRegion);
					pVector->m_bClosed = TRUE;
					pVector->m_rL = (unsigned char)DOUBLE2INT(Color.val[0]);
					pVector->m_gL = (unsigned char)DOUBLE2INT(Color.val[1]);
					pVector->m_bL = (unsigned char)DOUBLE2INT(Color.val[2]);
				}
			}
			else
			{
				pPix = ImageData + 3 * u + v * wStep;

				*(pPix++) = Color.val[0];
				*(pPix++) = Color.val[1];
				*pPix = Color.val[2];
			}
		}

		pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
	}
}

void CRVLPSuLM::Display3DSurface(	CRVLFigure * pFig,
									CRVL3DSurface2 *pSurf,
									CRVL3DPose *pPoseM0,
									CvScalar Color,
									int LineWidth,
									DWORD Flags)
{
	CRVLPSuLMBuilder *pPSuLMBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLDisplayVector Vector;
	
	if(Flags & RVLPSULM_DISPLAY_VECTORS)
	{
		Vector.m_bClosed = TRUE;
		Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
		Vector.m_rL = (BYTE)DOUBLE2INT(Color.val[0]);
		Vector.m_gL = (BYTE)DOUBLE2INT(Color.val[1]);
		Vector.m_bL = (BYTE)DOUBLE2INT(Color.val[2]);
		Vector.m_LineWidth = LineWidth;

		Vector.m_PointArray.Create(pFig->m_pMem, sizeof(RVLGUI_POINT));
	}

	CRVLDisplayVector *pVector;

	CRVL3DPose Pose0M;

	InverseTransform3D(Pose0M.m_Rot, Pose0M.m_X, pPoseM0->m_Rot, pPoseM0->m_X);

	CRVL3DPose PoseCM;

	RVLCombineTransform3D(Pose0M.m_Rot, Pose0M.m_X, pFig->m_PoseC0.m_Rot, pFig->m_PoseC0.m_X, PoseCM.m_Rot, PoseCM.m_X);

	double *RCM = PoseCM.m_Rot;
	double *tCM = PoseCM.m_X;

	double P[3 * 3];

	pPSuLMBuilder->GetProjectionMatrix(P);

	double A[3 * 3];

	RVLMXMUL3X3T2(P, RCM, A);

	CvPoint *PtArray;
	CvPoint *pContourEnd;
	CvPoint *pPt;
	int nPts;

#ifdef RVLPSULM_CONVEX_SEGMENTS
	CRVLMPtrChain *p2DContourList = &(pPSuLMBuilder->m_2DContourSet.m_ObjectList);

	CRVLMem Mem2;
	
	Mem2.Create(1000000);

	pPSuLMBuilder->m_2DContourSet.m_pMem0 = &Mem2;

	CRVL2DContour *p2DContour;

	Project3DSurface(pSurf, &PoseCM);

	p2DContourList->Start();

	while(p2DContourList->m_pNext)
	{
		p2DContour = (CRVL2DContour *)(p2DContourList->GetNext());

		PtArray = (CvPoint *)(p2DContour->m_ContourIPArray);

		nPts = p2DContour->m_nContourIPs;
#else
	CRVL3DPose PoseMC;

	InverseTransform3D(PoseMC.m_Rot, PoseMC.m_X, PoseCM.m_Rot, PoseCM.m_X);

	CRVL3DPose PoseFC;

	RVLCombineTransform3D(PoseMC.m_Rot, PoseMC.m_X, pSurf->m_Pose.m_Rot, pSurf->m_Pose.m_X, PoseFC.m_Rot, PoseFC.m_X);

	RVLDisplay3DEllipse(pSurf->m_EigenValues[0], pSurf->m_EigenValues[1], 1.0, &PoseFC, pPSuLMBuilder->m_pCamera,
		&PtArray, nPts);
#endif
	if(nPts > 0)
	{
		pContourEnd = PtArray + nPts;

		if(Flags & RVLPSULM_DISPLAY_VECTORS)
		{
			pVector = pFig->AddVector(&Vector);

			for(pPt = PtArray; pPt < pContourEnd; pPt++)
				pVector->Point(pPt->x, pPt->y);
		}
		else
		{
			for(pPt = PtArray; pPt < pContourEnd; pPt++)
			{
				pPt->x /= 2;
				pPt->y /= 2;
			}

			cvPolyLine(pFig->m_pImage, &PtArray, &nPts, 1, 1, Color);
		}

		Display3DSurfaceSamples(pFig, pSurf, A, tCM, Flags);

#ifdef RVLPSULM_CONVEX_SEGMENTS
	}
	}
#else
		delete[] PtArray;
	}
#endif
}

void CRVLPSuLM::Display3DLine(	CRVLFigure * pFig,
								CRVL3DLine2 *pLine,
								CRVL3DPose *pPoseM0,
								CvScalar Color,
								int LineWidth,
								DWORD Flags)
{
	CRVLPSuLMBuilder *pPSuLMBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLDisplayVector Vector;
	
	if(Flags & RVLPSULM_DISPLAY_VECTORS)
	{
		Vector.m_bClosed = FALSE;
		Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_SQUARE;
		Vector.m_rL = (BYTE)DOUBLE2INT(Color.val[0]);
		Vector.m_gL = (BYTE)DOUBLE2INT(Color.val[1]);
		Vector.m_bL = (BYTE)DOUBLE2INT(Color.val[2]);
		Vector.m_rP = (BYTE)DOUBLE2INT(Color.val[0]);
		Vector.m_gP = (BYTE)DOUBLE2INT(Color.val[1]);
		Vector.m_bP = (BYTE)DOUBLE2INT(Color.val[2]);
		Vector.m_LineWidth = LineWidth;

		Vector.m_PointArray.Create(pFig->m_pMem, sizeof(RVLGUI_POINT));
	}

	CRVLDisplayVector *pVector;

	CRVL3DPose Pose0M;

	InverseTransform3D(Pose0M.m_Rot, Pose0M.m_X, pPoseM0->m_Rot, pPoseM0->m_X);

	CRVL3DPose PoseCM;

	RVLCombineTransform3D(Pose0M.m_Rot, Pose0M.m_X, pFig->m_PoseC0.m_Rot, pFig->m_PoseC0.m_X, PoseCM.m_Rot, PoseCM.m_X);

	double *RCM = PoseCM.m_Rot;
	double *tCM = PoseCM.m_X;

	double P[3 * 3];

	pPSuLMBuilder->GetProjectionMatrix(P);

	double A[3 * 3];

	RVLMXMUL3X3T2(P, RCM, A);

	double RMC[9], tMC[3];

	if(m_Flags & RVLPSULM_FLAG_COMPLEX)
		RVLINVTRANSF3D(RCM, tCM, RMC, tMC)

	BYTE bOut;
	int iU1[2], iU2[2];
	CvPoint Pt1, Pt2;
	BYTE CropSide;
	//double X1[3], X2[3];

	if(m_Flags & RVLPSULM_FLAG_COMPLEX)
		bOut = RVLCrop3DLineSpherical(pLine->m_X[0], pLine->m_X[1], RMC, tMC, pPSuLMBuilder->m_pCamera,
			&(pPSuLMBuilder->m_ROI), pPSuLMBuilder->m_CropLTs.minz, pPSuLMBuilder->m_CropLTs.bOutLT, &Pt1, &Pt2, CropSide);
	else
		bOut = RVLCrop3DLine(pLine->m_X[0], pLine->m_X[1], A, tCM, &(pPSuLMBuilder->m_ROI),
			pPSuLMBuilder->m_CropLTs.minz, pPSuLMBuilder->m_CropLTs.minr, pPSuLMBuilder->m_CropLTs.bOutLT, 
			iU1, iU2, &Pt1, &Pt2, CropSide);

	if(bOut & 0x04)
		return;

	if(Flags & RVLPSULM_DISPLAY_VECTORS)
	{
		pVector = pFig->AddVector(&Vector);

		pVector->Line(Pt1.x, Pt1.y, Pt2.x, Pt2.y);
	}
	else
	{
		Pt1.x /= 2;
		Pt1.y /= 2;
		Pt2.x /= 2;
		Pt2.y /= 2;

		cvLine(pFig->m_pImage, Pt1, Pt2, Color, 2);
	}	
}

void CRVLPSuLM::Display(CRVLGUI * pGUI,
						CRVL3DPose *pPoseS0,
						char *FigName,
						DWORD Flags)
{
	CRVLPSuLMBuilder *pPSuLMBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLFigure *pFig;

	if(FigName)
		pFig = pGUI->OpenFigure(FigName);
	else
		pFig = pGUI->OpenFigure("PSuLM");

	pGUI->m_PoseC0ChangeScale.m_Alpha = 1.0 / pPSuLMBuilder->m_pCamera->fNrm;

	pFig->EmptyBitmap(cvSize(pPSuLMBuilder->m_pCamera->Width, pPSuLMBuilder->m_pCamera->Height), cvScalar(0, 0, 0));

	if(pFig->m_vpMouseCallbackData)
		delete pFig->m_vpMouseCallbackData;

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = new RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA;

	pFig->m_vpMouseCallbackData = pMouseCallbackData;
	
	pMouseCallbackData->pSPSuLM = this;
	pMouseCallbackData->pMPSuLM = NULL;
	pMouseCallbackData->pSSurf = pMouseCallbackData->pMSurf = NULL;
	RVLUnitMatrix(pMouseCallbackData->PoseM0.m_Rot, 3);
	RVLNULL3VECTOR(pMouseCallbackData->PoseM0.m_X);
	memcpy(pMouseCallbackData->PoseS0.m_Rot, pPoseS0->m_Rot, 3 * 3 * sizeof(double));
	memcpy(pMouseCallbackData->PoseS0.m_X, pPoseS0->m_X, 3 * sizeof(double));
	pMouseCallbackData->Flags = Flags;

	pFig->m_PoseC0.Reset();

	GetCenter(pFig->m_XFocus);

	// display PSuLM

	Display(pFig, &(pMouseCallbackData->PoseS0), cvScalar(0, 255, 0), Flags | RVLPSULM_DISPLAY_CLEAR);

	pFig->DisplayFocus(pPSuLMBuilder->m_pCamera);

	pGUI->ShowFigure(pFig);

	// set mouse callback

	cvSetMouseCallback(pFig->m_ImageName, RVLPSuLMDisplayMouseCallback, pFig);

}

void CRVLPSuLM::AddToFigure(CRVLGUI * pGUI)
{
	CRVLPSuLMBuilder *pPSuLMBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLFigure *pFig = pGUI->OpenFigure("PSuLM");

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	pMouseCallbackData->pMPSuLM = this;

	// display PSuLM

	Display(pFig, &(pMouseCallbackData->PoseM0), cvScalar(255, 0, 255), pMouseCallbackData->Flags & ~RVLPSULM_DISPLAY_MCELLS);

	pFig->DisplayFocus(pPSuLMBuilder->m_pCamera);

	pGUI->ShowFigure(pFig);
}

void CRVLPSuLM::Display2DLines(CRVLFigure *pFig)
{
	CRVLGUI *pGUI = (CRVLGUI *)(pFig->m_vpGUI);

	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLCamera *pCamera = pBuilder->m_pCamera;

	double f = pCamera->fNrm;
	double uc = pCamera->CenterXNrm;
	double vc = pCamera->CenterYNrm;

	double X0[3];

	X0[0] = X0[1] = X0[2] = 0.0;

	CRVLDisplayVector Vector(pFig->m_pMem);

	CRVLDisplayVector *pVector;

	Vector.m_bClosed = FALSE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector.m_rL = 255;
	Vector.m_gL = 0;
	Vector.m_bL = 255;

	int vGroundLeft = (DOUBLE2INT(-m_cGroundPlane / m_bGroundPlane) << 1);
	int vGroundRight = (DOUBLE2INT(-(m_aGroundPlane * (double)(pCamera->Width - 1) + m_cGroundPlane) / m_bGroundPlane) << 1);
	
	pVector = pFig->AddVector(&Vector);

	pVector->m_rL = 0;
	pVector->m_gL = 0;
	pVector->m_bL = 255;

	pVector->Line(0, vGroundLeft, ((pCamera->Width - 1) << 1), vGroundRight);

	double ca = cos(m_Alpha);
	double sa = sin(m_Alpha);

	double RotLG[3 * 3];

	pBuilder->CreateRotLG(m_NGroundPlane, RotLG);

	double *XGL = RotLG;

	double *ZGL = RotLG + 3 * 2;

	double V[3];

	V[0] = ca * XGL[0] + sa * ZGL[0];
	V[1] = ca * XGL[1] + sa * ZGL[1];
	V[2] = ca * XGL[2] + sa * ZGL[2];

	CRVL2DLine2 *pLine;
	double U1[3];
	double U2[3];
	double Um[3];
	double X[3];
	double XTmp[3];
	int iU1[2], iU2[2];
	double U[3];
	CRVL3DLine2 *p3DLine;
	int iData3DObjectPtr;

	m_2DLineList.Start();

	while(m_2DLineList.m_pNext)
	{
		pLine = (CRVL2DLine2 *)(m_2DLineList.GetNext());

		pVector = pFig->AddVector(&Vector);

		iData3DObjectPtr = pLine->m_pClass->m_iData3DObjectPtr;

		if(iData3DObjectPtr >= 0)
		{
			p3DLine = *((CRVL3DLine2 **)(pLine->m_pData + pLine->m_pClass->m_iData3DObjectPtr));

			if(p3DLine->m_Flags & RVL3DLINE_PARAM_FLAG_VERTICAL)
			{
				if(pLine->m_Flags & RVLOBJ2_FLAG_SELECTED)
				{
					pVector->m_rL = 255;
					pVector->m_gL = 255;
					pVector->m_bL = 0;
				}
				else
				{
					pVector->m_rL = 64;
					pVector->m_gL = 64;
					pVector->m_bL = 255;
				}
			}
		}
		else
			p3DLine = NULL;

		pVector->Line(pLine->m_iU[0][0], pLine->m_iU[0][1], pLine->m_iU[1][0], pLine->m_iU[1][1]);

		if(p3DLine)
		{
			if(p3DLine->m_Flags & RVL3DLINE_PARAM_FLAG_HORIZONTAL)
			{
				RVLiU2U(pLine->m_iU[0], uc, vc, U1);

				U1[2] = f;

				RVLiU2U(pLine->m_iU[1], uc, vc, U2);

				U2[2] = f;

				Um[0] = 0.5 * (U2[0] + U1[0]);
				Um[1] = 0.5 * (U2[1] + U1[1]);
				Um[2] = f;

				RVL3DLineClosestPoints(Um, V, X0, U1, X, XTmp);

				pCamera->Project3DPoint(X, U, iU1); 

				RVL3DLineClosestPoints(Um, V, X0, U2, X, XTmp);

				pCamera->Project3DPoint(X, U, iU2); 

				pVector = pFig->AddVector(&Vector);

				pVector->m_rL = 0;
				pVector->m_gL = 255;
				pVector->m_bL = 255;

				pVector->Line(iU1[0], iU1[1], iU2[0], iU2[1]);
			}
		}
	}

	if(pGUI->m_pSelectedObject)
	{
		pVector = pFig->AddVector(&Vector);

		int w = pBuilder->m_pCamera->Width;

		int iPix1 = (RVLAPIX *)(pGUI->m_pSelectedObject) - pBuilder->m_pAImage->m_pPix;

		CvPoint Pt1 = cvPoint(((iPix1 % w) << 1) + 1, ((iPix1 / w) << 1) + 1);

		if(pGUI->m_pSelectedObject2)
		{
			int iPix2 = (RVLAPIX *)(pGUI->m_pSelectedObject2) - pBuilder->m_pAImage->m_pPix;

			CvPoint Pt2 = cvPoint(((iPix2 % w) << 1) + 1, ((iPix2 / w) << 1) + 1);

			pVector->m_rL = 0;
			pVector->m_gL = 255;
			pVector->m_bL = 0;

			pVector->Line(Pt1.x, Pt1.y, Pt2.x, Pt2.y);

			double StartTime = pBuilder->m_pTimer->GetTime();

			int cost;
			int visible;

			//int i;

			//Pt1 = cvPoint(101, 15);
			//Pt2 = cvPoint(101, 415);

			//for(i = 0; i < 200; i++)
				cost = pBuilder->MatchLine(&Pt1, &Pt2, this, visible, pFig->m_pMem);

			double ExecutionTime = pBuilder->m_pTimer->GetTime() - StartTime; 

			RVLPSULM_LINE_COVERAGE_INTERVAL *pInterval = pBuilder->m_LineCoverage;

			if(pInterval)
			{
				Vector.m_rL = 255;
				Vector.m_gL = 128;
				Vector.m_bL = 128;
				Vector.m_LineWidth = 2;

				int du = Pt2.x - Pt1.x;
				int dv = Pt2.y - Pt1.y;
				int len2 = du * du + dv * dv;
				double flen2 = (double)len2;
				double au = (double)du / flen2;
				double av = (double)dv / flen2;

				while(pInterval)
				{
					pVector = pFig->AddVector(&Vector);

					pVector->Line(
						DOUBLE2INT((double)(Pt1.x) + au * (double)(pInterval->p1)),
						DOUBLE2INT((double)(Pt1.y) + av * (double)(pInterval->p1)),
						DOUBLE2INT((double)(Pt1.x) + au * (double)(pInterval->p2)),
						DOUBLE2INT((double)(Pt1.y) + av * (double)(pInterval->p2)));

					pInterval = pInterval->pNext;
				}
			}
		}
		else
		{
			pVector->m_rP = 0;
			pVector->m_gP = 255;
			pVector->m_bP = 0;

			pVector->m_PointType = RVLGUI_POINT_DISPLAY_TYPE_CROSS;

			pVector->Point(Pt1.x, Pt1.y);
		}
	}
}


void CRVLPSuLM::Save(FILE * fp, DWORD Flags)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	//Save each 3D surfaces, with corresponding 3D convex segments(surfaces) and 3D contours

	CRVL3DSurface2 *p3DSurface, *p3DConvexSegment;
	CRVL3DSurface2 **pp3DConvexSegment, **pp3DConvexSegmentEnd;

	CRVL3DContour *p3DContour;
	CRVL3DContour **pp3DContour;

	RVLARRAY *pRelListSurface;

	int n3DConvexSegments;
	int i;
	
	//save if min Plane exists
	fwrite(&m_minPlaneExists, sizeof(BOOL), 1, fp);

	//save min Info content
	fwrite(&m_minInfo, sizeof(double), 1, fp);

	//save map index
	fwrite(&m_iSubMap, sizeof(WORD), 1, fp);

	//save number of 3D surface
	fwrite(&m_n3DSurfaces, sizeof(int), 1, fp);

	//save total number of 3D surface
	fwrite(&m_n3DSurfacesTotal, sizeof(int), 1, fp);

	//Number of 3D convex surfaces (the number of good  3D convex surfaces can be obtained by counting the number of 3D contours generated )
	n3DConvexSegments = pBuilder->m_S3DContourSet.m_ObjectList.m_nElements;

	//save number of 3D convex surfaces
	fwrite(&n3DConvexSegments, sizeof(int), 1, fp);

	//for each 3D surface
	for(i=0;i<m_n3DSurfacesTotal;i++)
	{
		p3DSurface = m_3DSurfaceArray[i];

		//Save 3D surface (m_N and m_d)
		p3DSurface->Save(fp);

		//get corresponding convex segments
		pRelListSurface = p3DSurface->m_RelList + p3DSurface->m_pClass->m_iRelListComponents;
		pp3DConvexSegment = (CRVL3DSurface2 **)(pRelListSurface->pFirst);
		pp3DConvexSegmentEnd = (CRVL3DSurface2 **)(pRelListSurface->pEnd);

		//calculate number of 3D convex segments that are not rejected (this is neccessary since in the later stages
		// the rejected flag is set if no 3D contour can be created)
		n3DConvexSegments = 0;
		for(; pp3DConvexSegment < pp3DConvexSegmentEnd; pp3DConvexSegment++)
		{
			p3DConvexSegment = *pp3DConvexSegment;
			if((p3DConvexSegment->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
				n3DConvexSegments++;
		}

		//save number of 3D convex segments
		fwrite(&n3DConvexSegments, sizeof(int), 1, fp);

		//reset pointer
		pp3DConvexSegment = (CRVL3DSurface2 **)(pRelListSurface->pFirst);

		//for each 3D convex segment
		for(; pp3DConvexSegment < pp3DConvexSegmentEnd; pp3DConvexSegment++)
		{
			p3DConvexSegment = *pp3DConvexSegment;
			
			if(p3DConvexSegment->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			//Save 3D convex segment (m_Rot, m_X, m_EigenValues)
			p3DConvexSegment->Save(fp, RVL3DSURFACE_FLAG_CONVEX);

			//Get corresponding 3D contour
			pp3DContour = (CRVL3DContour **)(p3DConvexSegment->m_pData + pBuilder->m_S3DConvexSegmentSet.m_iDataContourPtr);

			p3DContour = *pp3DContour;

			//save 3D contour m_nPts, m_nPtArray
			p3DContour->Save(fp);
		
		}
	
	
	}
	
	//Save CellArray

	RVLPSULM_CELL *pCell;

	for(i=0;i<pBuilder->m_nCells;i++)
	{
		pCell = m_CellArray + i;

		fwrite(&(pCell->disparity) , sizeof(short), 1, fp);
		//fwrite(&(pCell->) , sizeof(short), 1, fp);

	}
	
	// save lines

	fwrite(&m_n3DLinesTotal, sizeof(int), 1, fp);

	if(m_n3DLinesTotal > 0)
	{
		CRVL3DLine2 *p3DLine = m_3DLineArray[0];

		CRVLClass *p3DLineSet = p3DLine->m_pClass;

		//CRVL2DLine2 *p2DLine = *((CRVL2DLine2 **)(p3DLine->m_pData + p3DLineSet->m_iDataProjectPtr));

		//CRVLClass *p2DLineSet = p2DLine->m_pClass;		

		CRVL3DLine2 **p3DLineArrayEnd = m_3DLineArray + m_n3DLines;

		CRVL3DLine2 **pp3DLine;

		for(pp3DLine = m_3DLineArray; pp3DLine < p3DLineArrayEnd; pp3DLine++)
		{
			p3DLine = *pp3DLine;

			//p2DLine = *((CRVL2DLine2 **)(p3DLine->m_pData + p3DLineSet->m_iDataProjectPtr));

			//p2DLine->Save(fp, 0x00000000);

			p3DLine->Save(fp, 0x00000000);

			fwrite(p3DLine->m_pData, sizeof(RVL3DLINE_EXTENDED_DATA), 1, fp);
		}
	}

#ifdef NEVER
	// save ref. view params.

	fwrite(&m_CameraHeight, sizeof(double), 1, fp);

	fwrite(m_NGroundPlane, sizeof(double), 3, fp);

	fwrite(&m_Alpha, sizeof(double), 1, fp);

	// save cell array

	WORD NullIndex = 0xffff;

	RVLPSULM_CELL *pCellArrayEnd = m_CellArray + pBuilder->m_nCells;

	int nLinePtrs = 0;

	RVLPSULM_CELL *pCell;
	RVLPSULM_CELL_LINEPTR *pLinePtr;

	for(pCell = m_CellArray; pCell < pCellArrayEnd; pCell++)
	{
		pLinePtr = pCell->pLinePtr;

		while(pLinePtr)
		{
			nLinePtrs++;

			pLinePtr = pLinePtr->pNext;
		}
	}

	fwrite(&nLinePtrs, sizeof(int), 1, fp);

	int iData3DObjectPtr = p2DLineSet->m_iData3DObjectPtr;

	for(pCell = m_CellArray; pCell < pCellArrayEnd; pCell++)
	{
		pLinePtr = pCell->pLinePtr;

		while(pLinePtr)
		{
			p2DLine = pLinePtr->pLine;

			p3DLine = *((CRVL3DLine2 **)(p2DLine->m_pData + iData3DObjectPtr));

			fwrite(&(p3DLine->m_Index), sizeof(WORD), 1, fp);

			pLinePtr = pLinePtr->pNext;
		}

		fwrite(&NullIndex, sizeof(WORD), 1, fp);
	}

	// save landmarks

	fwrite(&m_nLandmarks, sizeof(int), 1, fp);

	RVLPSULM_LANDMARK *pLandmark = (RVLPSULM_LANDMARK *)(m_LandmarkList.pFirst);
	
	while(pLandmark)
	{
		fwrite(pLandmark, sizeof(RVLPSULM_LANDMARK), 1, fp);

		pLandmark = (RVLPSULM_LANDMARK *)(pLandmark->pNext);
	}
#endif
}

void CRVLPSuLM::Load(FILE * fp, DWORD Flags)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	BYTE *pMem2	= pBuilder->m_pMem2->m_pFreeMem;;

	int i,j;
	
	//int k;

	//Load surfaces 

	CRVL3DSurface2 *p3DSurface, *p3DConvexSegment;

	CRVL3DContour *p3DContour;
	CRVL3DContour **pp3DContour;

	RVLARRAY *pRelListSurface;


	int n3DConvexSegments;


	CRVLClass  *p3DSurfaceSet = &(pBuilder->m_M3DSurfaceSet);
	CRVLClass  *p3DConvexSegmentSet = &(pBuilder->m_M3DConvexSegmentSet);
	CRVLClass  *p3DContourSet = &(pBuilder->m_M3DContourSet);

	//get if min Plane exists
	fread(&m_minPlaneExists, sizeof(BOOL), 1, fp);

	//get min Info content
	fread(&m_minInfo, sizeof(double), 1, fp);

	//get map index
	fread(&m_iSubMap, sizeof(WORD), 1, fp);
	
	//get and store number of 3D surface
	fread(&m_n3DSurfaces, sizeof(int), 1, fp);

	//get and store total number of 3D surface
	fread(&m_n3DSurfacesTotal, sizeof(int), 1, fp);

	//allocate SurfaceArray
	m_3DSurfaceArray = (CRVL3DSurface2 **)(pBuilder->m_pMem0->Alloc(m_n3DSurfacesTotal * sizeof(CRVL3DSurface2 *)));

	//get number of 3D convex surfaces
	fread(&n3DConvexSegments, sizeof(int), 1, fp);

	//Allocate space for 3D convex segments of LEVEL2 regions
	CRVL3DSurface2 **ConvexSegment3DArray = (CRVL3DSurface2 **)(pBuilder->m_pMem0->Alloc(n3DConvexSegments * sizeof(CRVL3DSurface2 *))); 
	memset(ConvexSegment3DArray, 0x00, n3DConvexSegments * sizeof(CRVL3DSurface2 *));
	CRVL3DSurface2 **pp3DConvexSegmentArray, **pp3DConvexSegmentArrayStart;

	//Allocate space for 3D contours of LEVEL2 regions
	CRVL3DContour **Contour3DArray = (CRVL3DContour **)(pBuilder->m_pMem0->Alloc(n3DConvexSegments * sizeof(CRVL3DContour *))); 
	memset(Contour3DArray, 0x00, n3DConvexSegments * sizeof(CRVL3DContour *));
	CRVL3DContour **pp3DContourArray;

	//Set initial pointers
	pp3DConvexSegmentArray = ConvexSegment3DArray;
	pp3DContourArray = Contour3DArray;

	m_nSurfaceSamples = 0;

	RVLQLIST *pSamples;
	RVL3DSURFACE_SAMPLE *pSample;

	//get all 3D surfaces
	for(i=0;i<m_n3DSurfacesTotal;i++)
	{
		//Create convex surface and store params
		p3DSurface = (CRVL3DSurface2 *)(RVL3DSurfaceTemplate.Create3(p3DSurfaceSet));

		pSamples = &(p3DSurface->m_Samples);
		RVLQLIST_INIT(pSamples)

		//Initialize as 3D Mesh object
		p3DSurface->Init();

		p3DSurface->m_Index = i;
	
		
		//Load 3D surface (m_N and m_d)
		p3DSurface->Load(fp);

		//add to chain and array in PSuLM
		m_SurfaceList.Add(p3DSurface);
		m_3DSurfaceArray[i] = p3DSurface;
		
		//store start
		pp3DConvexSegmentArrayStart = pp3DConvexSegmentArray;
		

		//load number of 3D convex segments
		fread(&n3DConvexSegments, sizeof(int), 1, fp);

		for(j=0;j<n3DConvexSegments;j++)
		{
			p3DConvexSegment = (CRVL3DSurface2 *)(RVL3DSurfaceTemplate.Create3(p3DConvexSegmentSet));
			
			//Load 3D convex segment (m_Rot, m_X, m_EigenValues)
			p3DConvexSegment->Init();
			p3DConvexSegment->Load(fp, RVL3DSURFACE_FLAG_CONVEX);
			
			//add to Convex surface array
			*(pp3DConvexSegmentArray++) = p3DConvexSegment;
			
			//Set corresponding 3D contour
			p3DContour = (CRVL3DContour *)(RVL3DContourTemplate.Create3(p3DContourSet));
			
			//load 3D contour m_nPts, m_nPtArray
			p3DContour->Load(fp);

			//relate to convex segment
			pp3DContour = (CRVL3DContour **)(p3DConvexSegment->m_pData + p3DConvexSegmentSet->m_iDataContourPtr);
			*pp3DContour = p3DContour;

			
		}

		//Define relation list
		pRelListSurface = p3DSurface->m_RelList + p3DSurface->m_pClass->m_iRelListComponents;
		pRelListSurface->pFirst = (unsigned char *)pp3DConvexSegmentArrayStart;
		pRelListSurface->pEnd = (unsigned char *)pp3DConvexSegmentArray;

		//set indexes to surface samples

		pSample = (RVL3DSURFACE_SAMPLE *)(p3DSurface->m_Samples.pFirst);

		while(pSample)
		{
			pSample->Index = (m_nSurfaceSamples++);

			pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		}
	}

	//Load CellArray
	for(i=0;i<pBuilder->m_nCells;i++)
	{
		fread(&(m_CellArray[i].disparity) , sizeof(short), 1, fp);
		m_CellArray[i].Flags = 0x00;
	}

#ifdef RVLPSULM_LINES
	// load lines

	//CRVLClass *p2DLineSet = &(pBuilder->m_M2DLineSet);
	CRVLClass *p3DLineSet = &(pBuilder->m_M3DLineSet);
	
	fread(&m_n3DLinesTotal, sizeof(int), 1, fp);

	m_n3DLines = (m_n3DLinesTotal > pBuilder->m_maxnDominant3DLines ? pBuilder->m_maxnDominant3DLines : m_n3DLinesTotal);

	m_3DLineArray = (CRVL3DLine2 **)(pBuilder->m_pMem0->Alloc(m_n3DLinesTotal * sizeof(CRVL3DLine2 *)));

	CRVL3DLine2 **pp3DLine = m_3DLineArray;

	//int iDataProjectPtr = p3DLineSet->m_iDataProjectPtr;

	//m_Evidence = 0;

	//CRVL2DLine2 *p2DLine;
	CRVL3DLine2 *p3DLine;
	RVL3DLINE_EXTENDED_DATA *p3DLineData;

	for(i = 0; i < m_n3DLinesTotal; i++)
	{
		//p2DLine = (CRVL2DLine2 *)(RVL2DLine2Template.Create3(p2DLineSet));

		//m_2DLineList.Add(p2DLine);

		//p2DLine->Load(fp, 0x00000000);

		//m_Evidence += p2DLine->m_leniU;

		//p2DLine->m_Index = i;

		p3DLine = (CRVL3DLine2 *)(RVL3DLine2Template.Create3(p3DLineSet));

		p3DLine->m_Index = i;

		*(pp3DLine++) = p3DLine;

		m_3DLineList.Add(p3DLine);

		p3DLine->Load(fp, 0x00000000);

		RVLMEM_ALLOC_STRUCT(pBuilder->m_pMem0, RVL3DLINE_EXTENDED_DATA, p3DLineData)

		fread(p3DLineData, sizeof(RVL3DLINE_EXTENDED_DATA), 1, fp);

		p3DLine->m_pData = (BYTE *)p3DLineData;

		//*((CRVL2DLine2 **)(p3DLine->m_pData + iDataProjectPtr)) = p2DLine;

		//*((CRVL3DLine2 **)(p2DLine->m_pData + p2DLineSet->m_iData3DObjectPtr)) = p3DLine;
	}
#endif

#ifdef NEVER
	// load ref. view params.

	fread(&m_CameraHeight, sizeof(double), 1, fp);

	fread(m_NGroundPlane, sizeof(double), 3, fp);

	fread(&m_Alpha, sizeof(double), 1, fp);

	// load cell array

	int nLinePtrs;

	fread(&nLinePtrs, sizeof(int), 1, fp);

	RVLPSULM_CELL_LINEPTR *LinePtrMem = 
		(RVLPSULM_CELL_LINEPTR *)(pBuilder->m_pMem0->Alloc(nLinePtrs * sizeof(RVLPSULM_CELL_LINEPTR)));

	RVLPSULM_CELL_LINEPTR *pLinePtr = LinePtrMem;

	RVLPSULM_CELL *pCellArrayEnd = m_CellArray + pBuilder->m_nCells;

	RVLPSULM_CELL *pCell;
	RVLPSULM_CELL_LINEPTR **ppNextLinePtr;
	WORD LineIndex;

	for(pCell = m_CellArray; pCell < pCellArrayEnd; pCell++)
	{
		ppNextLinePtr = &(pCell->pLinePtr);

		while(TRUE)
		{
			fread(&LineIndex, sizeof(WORD), 1, fp);

			if(LineIndex == 0xffff)
				break;

			*ppNextLinePtr = pLinePtr;

			p3DLine = m_3DLineArray[LineIndex];

			pLinePtr->pLine = *((CRVL2DLine2 **)(p3DLine->m_pData + iDataProjectPtr));

			ppNextLinePtr = &(pLinePtr->pNext);

			pLinePtr++;
		}

		*ppNextLinePtr = NULL;
	}

	// load landmarks

	fread(&m_nLandmarks, sizeof(int), 1, fp);

	RVLPSULM_LANDMARK *LandmarkArray = (RVLPSULM_LANDMARK *)(pBuilder->m_pMem0->Alloc(m_nLandmarks * sizeof(RVLPSULM_LANDMARK)));

	fread(LandmarkArray, sizeof(RVLPSULM_LANDMARK), m_nLandmarks, fp);	

	RVLPSULM_LANDMARK *pLandmarkArrayEnd = LandmarkArray + m_nLandmarks - 1;

	RVLPSULM_LANDMARK *pLandmark;

	for(pLandmark = LandmarkArray; pLandmark < pLandmarkArrayEnd; pLandmark++)
		pLandmark->pNext = pLandmark + 1;

	pLandmark->pNext = NULL;

	pBuilder->m_pMem2->m_pFreeMem = pMem2;

#endif
}




void CRVLPSuLM::Save()
{
	FILE *fp;
			
	fp = fopen(m_ModelFilePath, "wb");

	if(fp)
	{
		Save(fp);
		fclose(fp);
	}
}

void CRVLPSuLM::Load(DWORD Flags)
{
	FILE *fp;
			
	fp = fopen(m_ModelFilePath, "rb");

	if(fp)
	{
		Load(fp, Flags);
		fclose(fp);
	}
}

//void CRVLPSuLM::Clone(CRVLPSuLM *pPSulMOriginal)
//{
//
//	
//
//
//	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;
//
//	CRVLPSuLMBuilder *pBuilderOriginal = (CRVLPSuLMBuilder *)(pPSulMOriginal->m_vpBuilder);
//
//	BYTE *pMem2	= pBuilder->m_pMem2->m_pFreeMem;;
//
//	int i,j,k;
//	
//	RVLARRAY *pRelListSurface;
//
//	//Load surfaces 
//	CRVL3DSurface2 *p3DSurface, *p3DConvexSegment;
//
//	CRVL3DContour *p3DContour, *p3DContourOriginal;
//	CRVL3DContour **pp3DContour, **pp3DContourOriginal;
//
//	RVLARRAY *pRelListSurfaceOriginal;
//	CRVL3DSurface2 *p3DSurfaceOriginal, *p3DConvexSegmentOriginal;
//	CRVL3DSurface2 **pp3DConvexSegmentOriginal, **pp3DConvexSegmentEndOriginal;
//
//	int n3DConvexSegments;
//
//
//	CRVLClass  *p3DSurfaceSet = &(pBuilder->m_M3DSurfaceSet);
//	CRVLClass  *p3DConvexSegmentSet = &(pBuilder->m_M3DConvexSegmentSet);
//	CRVLClass  *p3DContourSet = &(pBuilder->m_M3DContourSet);
//	
//	//get and store number of 3D surface
//	m_n3DSurfaces = pPSulMOriginal->m_n3DSurfaces;
//
//	//allocate SurfaceArray
//	m_3DSurfaceArray = (CRVL3DSurface2 **)(pBuilder->m_pMem0->Alloc(m_n3DSurfaces * sizeof(CRVL3DSurface2 *)));
//
//	//get number of 3D convex surfaces
//	n3DConvexSegments = pBuilderOriginal->m_S3DContourSet.m_ObjectList.m_nElements;
//
//	//Allocate space for 3D convex segments of LEVEL2 regions
//	CRVL3DSurface2 **ConvexSegment3DArray = (CRVL3DSurface2 **)(pBuilder->m_pMem0->Alloc(n3DConvexSegments * sizeof(CRVL3DSurface2 *))); 
//	memset(ConvexSegment3DArray, 0x00, n3DConvexSegments * sizeof(CRVL3DSurface2 *));
//	CRVL3DSurface2 **pp3DConvexSegmentArray, **pp3DConvexSegmentArrayStart;
//
//	//Allocate space for 3D contours of LEVEL2 regions
//	CRVL3DContour **Contour3DArray = (CRVL3DContour **)(pBuilder->m_pMem0->Alloc(n3DConvexSegments * sizeof(CRVL3DContour *))); 
//	memset(Contour3DArray, 0x00, n3DConvexSegments * sizeof(CRVL3DContour *));
//	CRVL3DContour **pp3DContourArray;
//
//	//Set initial pointers
//	pp3DConvexSegmentArray = ConvexSegment3DArray;
//	pp3DContourArray = Contour3DArray;
//
//	//int noBinsUsed = 0;
//
//	//get all 3D surfaces
//	for(i=0;i<m_n3DSurfaces;i++)
//	{
//		p3DSurfaceOriginal = pPSulMOriginal->m_3DSurfaceArray[i];
//
//		//Create surface and store params
//		p3DSurface = (CRVL3DSurface2 *)(RVL3DSurfaceTemplate.Create3(p3DSurfaceSet));
//
//		//Initialize as 3D Mesh object
//		p3DSurface->Init();
//
//		p3DSurface->m_Index = i;
//	
//		
//		//Load 3D surface (m_N and m_d)
//		memcpy(p3DSurface->m_N, p3DSurfaceOriginal->m_N, 3 * sizeof(double));
//		
//		p3DSurface->m_d = p3DSurfaceOriginal->m_d;
//		p3DSurface->m_Area = p3DSurfaceOriginal->m_Area;
//		p3DSurface->m_nSupport = p3DSurfaceOriginal->m_nSupport;
//
//
//		//CRVL3DMeshObject Load
//		//Find no of bins used
//		//noBinsUsed = 0;		
//		RVLQLIST_HIST_ENTRY_SHORT *pHistEntry, *pHistEntryOriginal;
//
//		RVLMEM_ALLOC_STRUCT(p3DSurface->m_pClass->m_pMem0, RVLQLIST, p3DSurface->m_histRGB);
//		RVLQLIST_INIT(p3DSurface->m_histRGB);
//
//		pHistEntryOriginal = (RVLQLIST_HIST_ENTRY_SHORT*)(p3DSurfaceOriginal->m_histRGB->pFirst);
//		
//		while(pHistEntryOriginal)
//		{
//			//noBinsUsed++;
//			RVLMEM_ALLOC_STRUCT(p3DSurface->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);  
//			
//			pHistEntry->value = pHistEntryOriginal->value;
//
//			RVLQLIST_ADD_ENTRY(p3DSurface->m_histRGB, pHistEntry);
//
//			pHistEntryOriginal = (RVLQLIST_HIST_ENTRY_SHORT*)pHistEntryOriginal->pNext;
//		}
//
//		p3DSurface->m_noUsedColorPts = p3DSurfaceOriginal->m_noUsedColorPts;
//		p3DSurface->m_histRGB_base = p3DSurfaceOriginal->m_histRGB_base;
//		
//		p3DSurface->m_ShapeLabel = p3DSurfaceOriginal->m_ShapeLabel;
//		
//		//add to chain and array in PSuLM
//		m_SurfaceList.Add(p3DSurface);
//		m_3DSurfaceArray[i] = p3DSurface;
//		
//
//		//store start
//		pp3DConvexSegmentArrayStart = pp3DConvexSegmentArray;
//
//		//get corresponding convex segments
//		pRelListSurfaceOriginal = p3DSurfaceOriginal->m_RelList + p3DSurfaceOriginal->m_pClass->m_iRelListComponents;
//		pp3DConvexSegmentOriginal = (CRVL3DSurface2 **)(pRelListSurfaceOriginal->pFirst);
//		pp3DConvexSegmentEndOriginal = (CRVL3DSurface2 **)(pRelListSurfaceOriginal->pEnd);
//
//		for(; pp3DConvexSegmentOriginal < pp3DConvexSegmentEndOriginal; pp3DConvexSegmentOriginal++)
//		{
//			p3DConvexSegmentOriginal = *pp3DConvexSegmentOriginal;
//			if((p3DConvexSegmentOriginal->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
//			{
//				//CONVEX SEGEMENTS
//				p3DConvexSegment = (CRVL3DSurface2 *)(RVL3DSurfaceTemplate.Create3(p3DConvexSegmentSet));
//			
//				//Load 3D convex segment (m_Rot, m_X, m_EigenValues)
//				memcpy(p3DConvexSegment->m_Pose.m_Rot, p3DConvexSegmentOriginal->m_Pose.m_Rot, 3 * 3 * sizeof(double));
//				memcpy(p3DConvexSegment->m_Pose.m_X, p3DConvexSegmentOriginal->m_Pose.m_X, 3 * sizeof(double));
//				memcpy(p3DConvexSegment->m_EigenValues, p3DConvexSegmentOriginal->m_EigenValues, 2 * sizeof(double));
//				p3DConvexSegment->m_sigmaR = p3DConvexSegmentOriginal->m_sigmaR;
//				
//		
//
//				p3DConvexSegment->m_varq[0] =  p3DConvexSegment->m_sigmaR / (p3DConvexSegment->m_EigenValues[0] * p3DConvexSegment->m_EigenValues[0] + p3DConvexSegment->m_sigmaR);
//				p3DConvexSegment->m_varq[1] =  p3DConvexSegment->m_sigmaR / (p3DConvexSegment->m_EigenValues[1] * p3DConvexSegment->m_EigenValues[1] + p3DConvexSegment->m_sigmaR);
//				p3DConvexSegment->m_varq[2] =  p3DConvexSegment->m_sigmaR;
//				p3DConvexSegment->m_N[0] = p3DConvexSegment->m_Pose.m_Rot[2];
//				p3DConvexSegment->m_N[1] = p3DConvexSegment->m_Pose.m_Rot[5];
//				p3DConvexSegment->m_N[2] = p3DConvexSegment->m_Pose.m_Rot[8];
//				p3DConvexSegment->m_d = RVLDOTPRODUCT3(p3DConvexSegment->m_Pose.m_X, p3DConvexSegment->m_N);
//
//				//add to Convex surface array
//				*(pp3DConvexSegmentArray++) = p3DConvexSegment;
//				
//
//				//CONTOURS
//				//Set corresponding 3D contour
//				p3DContour = (CRVL3DContour *)(RVL3DContourTemplate.Create3(p3DContourSet));
//				
//				//load 3D contour m_nPts, m_nPtArray
//
//				//relate to convex segment
//				pp3DContour = (CRVL3DContour **)(p3DConvexSegment->m_pData + p3DConvexSegmentSet->m_iDataContourPtr);
//				*pp3DContour = p3DContour;
//
//				//Get corresponding Original 3D contour
//				p3DContourOriginal = *(CRVL3DContour **)(p3DConvexSegmentOriginal->m_pData + pBuilderOriginal->m_S3DConvexSegmentSet.m_iDataContourPtr);
//
//				p3DContour->m_nPts = p3DContourOriginal->m_nPts;
//
//				p3DContour->m_PtArray = (double *)(p3DContour->m_pClass->m_pMem0->Alloc(3 * p3DContour->m_nPts * sizeof(double)));
//				
//
//				memcpy(p3DContour->m_PtArray, p3DContourOriginal->m_PtArray, 3 * p3DContour->m_nPts * sizeof(double));
//
//			}
//			
//		}
//
//
//
//		//Define relation list
//		pRelListSurface = p3DSurface->m_RelList + p3DSurface->m_pClass->m_iRelListComponents;
//		pRelListSurface->pFirst = (unsigned char *)pp3DConvexSegmentArrayStart;
//		pRelListSurface->pEnd = (unsigned char *)pp3DConvexSegmentArray;
//		
//		
//
//		
//
//		
//
//	
//	}
//
//	//Load CellArray
//	for(i=0;i<pBuilder->m_nCells;i++)
//	{
//		m_CellArray[i].disparity = pPSulMOriginal->m_CellArray[i].disparity;
//		m_CellArray[i].Flags = 0x00;
//	}
//
//}
//
//

void CRVLPSuLM::Display3DLines(CRVLFigure *pFig,
							   CRVL3DPose *pPoseC0,
							   BOOL bInvTransf,
							   CRVLPSuLM *pPSuLM)
{
	CRVLGUI *pGUI = (CRVLGUI *)(pFig->m_vpGUI);

	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLCamera *pCamera = pBuilder->m_pCamera;

	CRVLDisplayVector Vector(pFig->m_pMem);

	Vector.m_bClosed = FALSE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;

	CRVLDisplayVector *pVector;
	CRVL3DLine2 *pLine;
	BYTE bOut;
	int iU1[2], iU2[2];
	CvPoint Pt1, Pt2;
	BYTE CropSide;
	double X1[3], X2[3];
	RVLPSULM_LINE_COVERAGE_INTERVAL *pInterval;
	int pPrev;
	int du, dv;
	double flen2;
	double au, av;
	int len2;
	int visible;

	m_3DLineList.Start();

	while(m_3DLineList.m_pNext)
	{
		pLine = (CRVL3DLine2 *)(m_3DLineList.GetNext());

		if(bInvTransf)
		{
			pPoseC0->InvTransf(pLine->m_X[0], X1);
			pPoseC0->InvTransf(pLine->m_X[1], X2);
		}
		else
		{
			pPoseC0->Transf(pLine->m_X[0], X1);
			pPoseC0->Transf(pLine->m_X[1], X2);
		}

		bOut = RVLCrop3DLine(X1, X2, pBuilder->m_pCamera, &(pBuilder->m_ROI),
			pBuilder->m_CropLTs.minz, pBuilder->m_CropLTs.minr, pBuilder->m_CropLTs.bOutLT, 
			iU1, iU2, &Pt1, &Pt2, CropSide);

		if(bOut & 0x04)
			continue;
	
		pBuilder->MatchLine(&Pt1, &Pt2, pPSuLM, visible, pFig->m_pMem);
	
		pInterval = pBuilder->m_LineCoverage;

		pPrev = 0;

		du = Pt2.x - Pt1.x;
		dv = Pt2.y - Pt1.y;
		len2 = du * du + dv * dv;
		flen2 = (double)len2;
		au = (double)du / flen2;
		av = (double)dv / flen2;

		while(pInterval)
		{
			if(pInterval->p1 > pPrev)
			{
				pVector = pFig->AddVector(&Vector);

				pVector->m_rL = 255;
				pVector->m_gL = 0;
				pVector->m_bL = 0;
				pVector->m_LineWidth = 2;
		
				pVector->Line(
					DOUBLE2INT((double)(Pt1.x) + au * (double)pPrev),
					DOUBLE2INT((double)(Pt1.y) + av * (double)pPrev),
					DOUBLE2INT((double)(Pt1.x) + au * (double)(pInterval->p1)),
					DOUBLE2INT((double)(Pt1.y) + av * (double)(pInterval->p1)));
			}

			if(pInterval->p2 > pInterval->p1)
			{
				pVector = pFig->AddVector(&Vector);

				pVector->m_rL = 0;
				pVector->m_gL = 255;
				pVector->m_bL = 0;
				pVector->m_LineWidth = 1;
		
				pVector->Line(
					DOUBLE2INT((double)(Pt1.x) + au * (double)(pInterval->p1)),
					DOUBLE2INT((double)(Pt1.y) + av * (double)(pInterval->p1)),
					DOUBLE2INT((double)(Pt1.x) + au * (double)(pInterval->p2)),
					DOUBLE2INT((double)(Pt1.y) + av * (double)(pInterval->p2)));
			}

			pPrev = pInterval->p2;

			pInterval = pInterval->pNext;
		}

		if(len2 > pPrev)
		{
			pVector = pFig->AddVector(&Vector);

			pVector->m_rL = 255;
			pVector->m_gL = 0;
			pVector->m_bL = 0;
			pVector->m_LineWidth = 2;
	
			pVector->Line(
				DOUBLE2INT((double)(Pt1.x) + au * (double)pPrev),
				DOUBLE2INT((double)(Pt1.y) + av * (double)pPrev),
				DOUBLE2INT((double)(Pt1.x) + au * flen2),
				DOUBLE2INT((double)(Pt1.y) + av * flen2));
		}
	}	
}

RVLPSULM_LANDMARK * CRVLPSuLM::SelectLandmarkFromDisplay(int * iU)
{
	RVLPSULM_LANDMARK *pClosestLandmark = NULL;

	int iDataProjectPtr = m_3DLineArray[0]->m_pClass->m_iDataProjectPtr;

	RVLPSULM_LANDMARK *pLandmark = (RVLPSULM_LANDMARK *)(m_LandmarkList.pFirst);

	int mindist;
	CRVL3DLine2 *p3DLine;
	CRVL2DLine2 *p2DLine;
	CRVL2DLine2 *pSelected2DLine;
	int dist;

	while(pLandmark)
	{
		p3DLine = m_3DLineArray[pLandmark->i3DLine];

		p2DLine = *((CRVL2DLine2 **)(p3DLine->m_pData + iDataProjectPtr));

		p2DLine->m_Flags &= ~RVLOBJ2_FLAG_SELECTED;

		if(iU)
		{
			dist = p2DLine->Distance(iU);

			if(pClosestLandmark)
			{
				if(dist < mindist)
				{
					mindist = dist;

					pClosestLandmark = pLandmark;

					pSelected2DLine = p2DLine;
				}
			}
			else
			{
				mindist = dist;

				pClosestLandmark = pLandmark;

				pSelected2DLine = p2DLine;
			}
		}

		pLandmark = (RVLPSULM_LANDMARK *)(pLandmark->pNext);
	}

	if(pClosestLandmark)
		pSelected2DLine->m_Flags |= RVLOBJ2_FLAG_SELECTED;

	return pClosestLandmark;
}

void CRVLPSuLM::CreateCellArray()
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVL2DRegion2 **RegionMap = pBuilder->m_pPSD->m_2DRegionMap;

	int iDataGrandParentPtr = pBuilder->m_pAImage->m_C2DRegion.m_iDataGrandParentPtr;

	int w = pBuilder->m_pCamera->Width;

	int nRows = pBuilder->m_nRows;
	int nCols = pBuilder->m_nCols;

	RVLPSULM_CELL *pCell = m_CellArray + nCols;

	RVLPSULM_CELL *pCellArrayEnd = m_CellArray + pBuilder->m_nCells;

	RVLPSULM_CELL_CONST *pCellConst = pBuilder->m_CellConstArray + nCols;

	nRows--;
	nCols--;

	int u, v, iPix;
	CRVL2DRegion2 *pTriangle, *p2DRegion;
	int iRow, iCol;
	
	for(iRow = 1; iRow < nRows; iRow++)
	{
		pCell++;

		pCellConst++;

		for(iCol = 1; iCol < nCols; iCol++, pCell++, pCellConst++)
		{
			u = pCellConst->u;

			v = pCellConst->v;

			iPix = (u  >> 1) + (v  >> 1) * w;

			pTriangle = RegionMap[iPix];

			if(pTriangle == NULL)
			{
				pCell->disparity = -1;

				continue;
			}

			if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			{
				pCell->disparity = -1;

				continue;
			}

			p2DRegion = *((CRVL2DRegion2 **)(pTriangle->m_pData + iDataGrandParentPtr));

			if(p2DRegion)
				pCell->disparity = (short)(0.5 * (p2DRegion->m_a * (double)u + p2DRegion->m_b * (double)v + 2.0 * p2DRegion->m_c));			
		}

		pCell++;

		pCellConst++;
	}
}

void CRVLPSuLM::Get3DSurfaceSamplesFrom2DRegionSamples(int nSamples)
{
	m_nSurfaceSamples = nSamples;

	if(nSamples == 0)
		return;

	if(m_n3DSurfaces == 0)
		return;

	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	CRVLCamera *pCamera = pBuilder->m_pCamera;

	CRVLPlanarSurfaceDetector *pPSD = pBuilder->m_pPSD;

	CRVLStereoVision *pStereoVision = pBuilder->m_pStereoVision;

	//double f = RVLMAX(pStereoVision->m_KinectParams.depthFu, pStereoVision->m_KinectParams.depthFv);

	//CRVLC3D *p3DSurfaceSet = (CRVLC3D *)(m_3DSurfaceArray[0]->m_pClass);
	CRVLClass *p3DSurfaceSet = m_3DSurfaceArray[0]->m_pClass;

	RVLAPIX *APixArray = pPSD->m_pAImage->m_pPix;

	CRVLMem *pMem = p3DSurfaceSet->m_pMem0;

	RVL3DSURFACE_SAMPLE *SampleMem;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RVL3DSURFACE_SAMPLE, nSamples, SampleMem);

	RVL3DSURFACE_SAMPLE *p3DSurfaceSample = SampleMem;

	CRVL3DSurface2 **p3DSurfaceArrayEnd = m_3DSurfaceArray + m_n3DSurfacesTotal;  //m_n3DSurfaces

	double RuvTol2 = pPSD->m_RuvTol*pPSD->m_RuvTol;
	double RuvdTol2 = pPSD->m_RuvdTol*pPSD->m_RuvdTol;

	CRVL3DSurface2 **pp3DSurface;
	CRVL3DSurface2 *p3DSurface;
	CRVL2DRegion2 *p2DRegion;
	RVLPSD_2DREGION_SAMPLE *p2DRegionSample;
	int U[2];
	RVLAPIX *pAPix;
	RVLQLIST *p3DSurfaceSamples;
	double C[9];
	double d;
	double *X, *N, *V;
	double M[3];
	int iPix;
	double eig[3];
	BOOL bReal[3];
	double q, wz, stdX;

	for(pp3DSurface = m_3DSurfaceArray; pp3DSurface < p3DSurfaceArrayEnd; pp3DSurface++)
	{
		p3DSurface = *pp3DSurface;

		N = p3DSurface->m_N;

		p3DSurfaceSamples = &(p3DSurface->m_Samples);

		p2DRegion = (CRVL2DRegion2 *)(p3DSurface->m_vp2DRegion);

		p2DRegionSample = (RVLPSD_2DREGION_SAMPLE *)(p2DRegion->m_Samples.pFirst);

		while(p2DRegionSample)
		{
			p3DSurfaceSample->Flags = 0x00;

			p3DSurfaceSample->Index = p3DSurfaceSample - SampleMem;

			p3DSurfaceSample->pSurface = p3DSurface;

			pAPix = (RVLAPIX *)(p2DRegionSample->pPt);

			iPix = pAPix - APixArray;

			U[0] = (pAPix->u >> 1);
			U[1] = (pAPix->v >> 1);

			X = p3DSurfaceSample->X;

			RVLProject2DPointTo3DPlane(U, X, p3DSurface,
									   pStereoVision->m_KinectParams.depthUc,
									   pStereoVision->m_KinectParams.depthVc,
									   pStereoVision->m_KinectParams.depthFu,
									   pStereoVision->m_KinectParams.depthFv);

			d = RVLKINECTZTODEPTH(X[2]);

			pCamera->KinectReconWithUncert(	(double)(U[0]), (double)(U[1]), d,
											pStereoVision->m_KinectParams.d0,
											pStereoVision->m_KinectParams.k,
											pStereoVision->m_KinectParams.depthUc,
											pStereoVision->m_KinectParams.depthVc,
											pStereoVision->m_KinectParams.depthFu,
											pStereoVision->m_KinectParams.depthFv, 
											RuvTol2, RuvdTol2, C);	

			V = p3DSurfaceSample->V;

			RVLGetMaxEigVector3(C, eig, bReal, V);

			stdX = sqrt(eig[2]);

			p3DSurfaceSample->stdX = stdX;

			wz = X[2] * (double)(p2DRegionSample->w);

			p3DSurfaceSample->wz = wz;

			q = fabs(RVLDOTPRODUCT3(N, V));

			M[0] = V[0] - q * N[0];
			M[1] = V[1] - q * N[1];
			M[2] = V[2] - q * N[2];

			p3DSurfaceSample->stdN[0] = sqrt(RVLDOTPRODUCT3(M, M));
			p3DSurfaceSample->stdN[1] = q;

			RVLQLIST_ADD_ENTRY(p3DSurfaceSamples, p3DSurfaceSample)

			p3DSurfaceSample++;

			p2DRegionSample = (RVLPSD_2DREGION_SAMPLE *)(p2DRegionSample->pNext);
		}
	}
}

int CRVLPSuLM::MatchGeometryAndColor(	CRVL3DSurface2 *pSurface, 
						CRVL3DPose *pPoseSM,
						int noBins,
						float *helperArray)
{
	int score = 0;

	//CRVL3DSurface2 **pMSurfArrayEnd = m_3DSurfaceArray + m_n3DSurfaces;
	CRVL3DSurface2 **pMSurfArrayEnd = m_3DSurfaceArray + (m_n3DSurfaces <= 20 ? m_n3DSurfaces : 20);

	//double MinMatchCost = 12.0;

	int bestScore = 0;

	CRVL3DSurface2 **ppMSurf;
	CRVL3DSurface2 *pMSurface;
	double MatchCost;
	double detQ;
	RVLSURFACE_MATCH_ARRAY MatchData;
	CRVL3DSurface2 *pBestMatch;

	for(ppMSurf = m_3DSurfaceArray; ppMSurf < pMSurfArrayEnd; ppMSurf++)
	{
		pMSurface = *ppMSurf;

		//if(pMSurface->m_Flags & RVLOBJ2_FLAG_MATCHED)
		//	continue;

		if(pSurface->Match2(pMSurface, pPoseSM, MatchCost, detQ, &MatchData))
		{
			score = chi2_LookUpTable[DOUBLE2INT(100.0 * MatchCost)] * 
				//(pMSurface->m_nSupport + pSurface->m_nSupport);
				(pSurface->m_nSupport <= pMSurface->m_nSupport ? pSurface->m_nSupport : pMSurface->m_nSupport);
			score *= (1.0 + pMSurface->RVLIntersectColorHistogram(pSurface, noBins, true, helperArray));
			///////score *= (1.0 + CRVL3DMeshObject::RVLIntersectHistograms(pMSurface->m_LBP_RIU_VAR, pSurface->m_LBP_RIU_VAR, pMSurface->m_noUsedLbpPts, pSurface->m_noUsedLbpPts, 17*26, NULL, -1, helperArray));
			if(score > bestScore)
			{
				bestScore = score;

				pBestMatch = pMSurface;
			}

			//if(MatchCost < MinMatchCost)
			//	MinMatchCost = MatchCost;

			//score += (chi2_LookUpTable[DOUBLE2INT(100.0 * MatchCost)] * 
			//	(pSurface->m_nSupport <= pMSurface->m_nSupport ? pSurface->m_nSupport : pMSurface->m_nSupport));
		}
	}

	//if(bestScore > 0)
	//	pBestMatch->m_Flags |= RVLOBJ2_FLAG_MATCHED;

	//return score;
	return bestScore;
	
	//return (MinMatchCost < 11.9 ? chi2_LookUpTable[DOUBLE2INT(100.0 * MinMatchCost)] * pSurface->m_nSupport : 0);

}

int* CRVLPSuLM::GetCorrectHypothesisIdxFromGT(CRVLMPtrChain* hypothesislist,
											 CRVL3DPose* pPoseAC,
											 double toleranceXYZ,
											 double toleranceAng,
											 char* gtFile,
											 double *errData,
											 CRVL3DPose *pPoseAsAm,
											 double *gtAsAmData,
											 bool descending,
											 bool search)
{
	pPoseAsAm->m_X[0] = pPoseAsAm->m_X[1] = pPoseAsAm->m_X[2] = 0.0;
	pPoseAsAm->m_Alpha = pPoseAsAm->m_Beta = pPoseAsAm->m_Theta = 0.0;

	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	//creating hypothesis array for sorting purposes
	void** hypArray = new void*[hypothesislist->m_nElements];
	RVLPSULM_HYPOTHESIS* hypElement;
	hypothesislist->Start();
	for(int i = 0; i < hypothesislist->m_nElements; i++)
	{
		hypElement = (RVLPSULM_HYPOTHESIS*)hypothesislist->GetNext();
		hypArray[i] = hypElement;
	}
	//Sortiranje - bubble sort
	void* tempVoid;
	int tempInt[2];
	bool chg = true;
	while(chg)
	{
		chg = false;
		for(int i = 0; i < hypothesislist->m_nElements - 1; i++)
		{
			if (descending)
			{
				if (((RVLPSULM_HYPOTHESIS*)hypArray[i + 1])->cost > ((RVLPSULM_HYPOTHESIS*)hypArray[i])->cost)
				{
					tempVoid = hypArray[i];
					hypArray[i] = hypArray[i + 1];
					hypArray[i + 1] = tempVoid;

					chg = true;
				}
			}
			else
			{
				if (((RVLPSULM_HYPOTHESIS*)hypArray[i + 1])->cost < ((RVLPSULM_HYPOTHESIS*)hypArray[i])->cost)
				{
					tempVoid = hypArray[i];
					hypArray[i] = hypArray[i + 1];
					hypArray[i + 1] = tempVoid;

					chg = true;
				}
			}
		}
	}

	int *position = new int[3];
	position[0] = -1;
	position[1] = -1;
	position[2] = -1;
	errData[0] = -1.0;
	errData[1] = -1.0;
	gtAsAmData[0] = gtAsAmData[1] = gtAsAmData[2] = -1.0;
	//Opening and parsing ground thruth file
	ifstream gtFileStream;
	stringstream ss (stringstream::in | stringstream::out);
	gtFileStream.open(gtFile, ifstream::in);
	if (search)
	{
		string tempString(m_FileName);
		int fileIdx = -1;
		int tempFileIdx = -1;
		//file index
		ss.clear();
		ss.str("");
		ss << tempString.substr(tempString.length() - 12, 5);
		ss >> fileIdx;
		string tempString2 = tempString.replace(tempString.length() - 12, 5, "00000");
		while(!gtFileStream.eof())
		{
			tempString.clear();
			getline(gtFileStream, tempString);
			//finding necessary part
			if (!(tempString.find(tempString2)==string::npos))
			{
				while(!gtFileStream.eof())
				{
					tempString.clear();
					getline(gtFileStream, tempString);
					ss.clear();
					ss.str("");
					ss << tempString;
					ss >> tempFileIdx;
					if (tempFileIdx == fileIdx)
						break;
				}
				break;
			}
		}
		//line is found and stored in tempString and ss
		double tempDouble[9];
		ss >> tempInt[0];	//Submap No
		ss >> tempInt[1];	//Model No
		if (tempInt[0] == -1)	//if no GT exists for this scene, return
		{
			delete [] hypArray;
			return position;
		}

		ss >> tempDouble[0];	//X
		ss >> tempDouble[1];	//Y
		ss >> tempDouble[2];	//Z
		ss >> tempDouble[3];	//Alpha
		ss >> tempDouble[4];	//Beta
		ss >> tempDouble[5];	//Theta
		ss >> tempDouble[6];	//X0
		ss >> tempDouble[7];	//Y0
		ss >> tempDouble[8];	//Alpha0
		//ss >> tempDouble[9];	//XGT
		//ss >> tempDouble[10];	//YGT
		//ss >> tempDouble[11];	//AlphaGT

		double euclidDist = 0.0;
		double angleErr = 0.0;
		CRVL3DPose PoseAs0;

		//Check if ground truth exists
		double X,Y, alpha;
		//if((tempDouble[9]+tempDouble[10]+tempDouble[11])!=0.0)
		//{
		//	X = tempDouble[9];
		//	Y = tempDouble[10];
		//	alpha = tempDouble[11];
		//}
		//else
		//{
			X = tempDouble[6];
			Y = tempDouble[7];
			alpha = tempDouble[8];
		//}

		hypothesislist->Start();
		//finding hypothesis that match tolerance and returning its position in sorted array
		for(int i = 0; i < hypothesislist->m_nElements; i++)
		{
			hypElement = (RVLPSULM_HYPOTHESIS*)hypArray[i];
			//RVLPSuLMHypothesisGetAbsPose(hypElement, pPoseAC, &PoseCs0);
			//euclidDist = sqrt(pow(PoseCs0.m_X[0] - tempDouble[6], 2) + pow(PoseCs0.m_X[1] - tempDouble[7], 2) + pow(PoseCs0.m_X[2] - tempDouble[8], 2));
			//if ((euclidDist <= toleranceXYZ) && (abs(PoseCs0.m_Alpha * RAD2DEG - tempDouble[9]) <= toleranceAng) && (abs(PoseCs0.m_Beta * RAD2DEG  - tempDouble[10]) <= toleranceAng) && (abs(PoseCs0.m_Theta * RAD2DEG  - tempDouble[11]) <= toleranceAng))
			//{
			//	position = i;
			//	break;
			//}
			if (hypElement->pMPSuLM->m_iSubMap != tempInt[0])
				continue;

			RVLPSuLMHypothesisGetAbsPose3DOF(hypElement, pPoseAC, &PoseAs0);
			
			euclidDist = sqrt(pow(PoseAs0.m_X[0] - X, 2) + pow(PoseAs0.m_X[1] - Y, 2));
			angleErr = abs(PoseAs0.m_Alpha * RAD2DEG - alpha);
			if ((euclidDist <= toleranceXYZ) && (angleErr <= toleranceAng))
			{
				errData[0] = euclidDist;
				errData[1] = angleErr;
				position[0] = i;
				position[1] = hypElement->pMPSuLM->m_iSubMap;
				position[2] = hypElement->pMPSuLM->m_Index;
				//Convert selected poseSM to 3DOF
				RVLDeterminePose6DOFTo3DOF(pPoseAsAm, &(hypElement->PoseSM),  pPoseAC);
				//Get GT data for model
				if(pBuilder->m_Flags & RVLPSULMBUILDER_FLAG_GROUNDTRUTH_EXISTS)
					pBuilder->GetGroundTruth(hypElement->pMPSuLM,gtAsAmData);

				break;
			}

		}
	}
	else
	{
		//This is obsolete  since the ExpRezPSuLM file already contains the correct absolute pose info

		//float gt[] = {0.0, 0.0, 0.0};
		//string tempString;
		//tempString.clear();
		//getline(gtFileStream, tempString);
		//ss << tempString;
		//ss >> gt[0]; //X
		//ss >> gt[1]; //Y
		//ss >> gt[2]; //Theta

		//double euclidDist = 0.0;	
		//CRVL3DPose PoseCs0;
		//hypothesislist->Start();
		////finding hypothesis that match tolerance and returning its position in sorted array
		//for(int i = 0; i < hypothesislist->m_nElements; i++)
		//{
		//	hypElement = (RVLPSULM_HYPOTHESIS*)hypArray[i];
		//	RVLPSuLMHypothesisGetAbsPose(hypElement, pPoseAC, &PoseCs0);
		//	euclidDist = sqrt(pow(PoseCs0.m_X[0] - gt[0], 2) + pow(PoseCs0.m_X[1] - gt[1], 2));
		//	if ((euclidDist <= toleranceXYZ) && (abs(PoseCs0.m_Theta * RAD2DEG  - gt[2]) <= toleranceAng))
		//	{
		//		position = i;
		//		break;
		//	}

		//}

	}
	delete [] hypArray;
	return position;
}


RVLQLIST* CRVLPSuLM::GetCorrectHypothesesViaGT_EvalBench(CRVLMPtrChain* hypothesislist,
											 CRVL3DPose* pPoseAC,
											 double toleranceXYZ,
											 double toleranceAng,
											 char* gtFile,
											 CRVLMem *pMem,
											 bool descending)
{
	//creating hypothesis array for sorting purposes
	void** hypArray = new void*[hypothesislist->m_nElements];
	RVLPSULM_HYPOTHESIS* hypElement;
	hypothesislist->Start();
	for(int i = 0; i < hypothesislist->m_nElements; i++)
	{
		hypElement = (RVLPSULM_HYPOTHESIS*)hypothesislist->GetNext();
		hypArray[i] = hypElement;
	}
	//Sortiranje - bubble sort
	void* tempVoid;
	int tempInt[2];
	bool chg = true;
	while(chg)
	{
		chg = false;
		for(int i = 0; i < hypothesislist->m_nElements - 1; i++)
		{
			if (descending)
			{
				if (((RVLPSULM_HYPOTHESIS*)hypArray[i + 1])->cost > ((RVLPSULM_HYPOTHESIS*)hypArray[i])->cost)
				{
					tempVoid = hypArray[i];
					hypArray[i] = hypArray[i + 1];
					hypArray[i + 1] = tempVoid;

					chg = true;
				}
			}
			else
			{
				if (((RVLPSULM_HYPOTHESIS*)hypArray[i + 1])->cost < ((RVLPSULM_HYPOTHESIS*)hypArray[i])->cost)
				{
					tempVoid = hypArray[i];
					hypArray[i] = hypArray[i + 1];
					hypArray[i + 1] = tempVoid;

					chg = true;
				}
			}
		}
	}

	//Opening and parsing ground thruth file
	ifstream gtFileStream;
	stringstream ss (stringstream::in | stringstream::out);
	gtFileStream.open(gtFile, ifstream::in);

	string tempString(m_FileName);
	int fileIdx = -1;
	int tempFileIdx = -1;
	//file index
	ss.clear();
	ss.str("");
	ss << tempString.substr(tempString.length() - 12, 5);
	ss >> fileIdx;
	string tempString2 = tempString.replace(tempString.length() - 12, 5, "00000");
	while(!gtFileStream.eof())
	{
		tempString.clear();
		getline(gtFileStream, tempString);
		//finding necessary part
		if (!(tempString.find(tempString2)==string::npos))
		{
			while(!gtFileStream.eof())
			{
				tempString.clear();
				getline(gtFileStream, tempString);
				ss.clear();
				ss.str("");
				ss << tempString;
				ss >> tempFileIdx;
				if (tempFileIdx == fileIdx)
					break;
			}
			break;
		}
	}
	//line is found and stored in tempString and ss
	double tempDouble[9];
	ss >> tempInt[0];	//Submap No
	ss >> tempInt[1];	//Model No
	if (tempInt[0] == -1)	//if no GT exists for this scene, return
	{
		delete [] hypArray;
		return NULL;
	}

	ss >> tempDouble[0];	//X
	ss >> tempDouble[1];	//Y
	ss >> tempDouble[2];	//Z
	ss >> tempDouble[3];	//Alpha
	ss >> tempDouble[4];	//Beta
	ss >> tempDouble[5];	//Theta
	ss >> tempDouble[6];	//X0
	ss >> tempDouble[7];	//Y0
	ss >> tempDouble[8];	//Alpha0
	//ss >> tempDouble[9];	//XGT
	//ss >> tempDouble[10];	//YGT
	//ss >> tempDouble[11];	//AlphaGT

	double euclidDist = 0.0;
	double angleErr = 0.0;
	CRVL3DPose PoseAs0;

	//Check if ground truth exists
	double X,Y, alpha;
	X = tempDouble[6];
	Y = tempDouble[7];
	alpha = tempDouble[8];

	hypothesislist->Start();	//Cemu ovo kada se koristi array a ne lista?

	//Stvaramo listu
	RVLQLIST *correctHypList;
	RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST, correctHypList);
	RVLQLIST_INIT(correctHypList);
	RVLQLIST_PTR_ENTRY* tempEntry;
	bool brk = false;

	//finding hypothesis that match tolerance and returning its position in sorted array
	for(int i = 0; i < hypothesislist->m_nElements; i++)
	{
		hypElement = (RVLPSULM_HYPOTHESIS*)hypArray[i];

		//check if it is on the same submap
		if (hypElement->pMPSuLM->m_iSubMap != tempInt[0])
			continue;

		RVLPSuLMHypothesisGetAbsPose3DOF(hypElement, pPoseAC, &PoseAs0);
		
		euclidDist = sqrt(pow(PoseAs0.m_X[0] - X, 2) + pow(PoseAs0.m_X[1] - Y, 2));
		angleErr = abs(PoseAs0.m_Alpha * RAD2DEG - alpha);

		if ((euclidDist <= toleranceXYZ) && (angleErr <= toleranceAng))
		{
			//obilazimo postojece hipoteze u listi i provjeravamo da li je za taj model vec stavljena hipoteza, ako je prelazi se na iducu hipotezu
			if (correctHypList->pFirst != NULL)
			{
				brk = false;
				tempEntry = (RVLQLIST_PTR_ENTRY*)correctHypList->pFirst;
				while(tempEntry)
				{
					if (((RVLPSULM_HYPOTHESIS*)tempEntry->Ptr)->pMPSuLM == hypElement->pMPSuLM)
					{
						brk = true;
						break;
					}
					tempEntry = (RVLQLIST_PTR_ENTRY*)tempEntry->pNext;
				}
				if (brk)
					continue;
			}
			//ako je popis prazan ili nije zakljucio da se na popisu nalazi hipoteza za taj model, dodaje se ta hipoteza na popis 
			RVLQLIST_PTR_ENTRY *pNewEntry;
			RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pNewEntry);
			pNewEntry->Ptr = hypElement;
			RVLQLIST_ADD_ENTRY(correctHypList, pNewEntry);
		}

	}

	delete [] hypArray;
	return correctHypList;
}


void CRVLPSuLM::AddToMeshFile(	char *MeshName,
								FILE *fp,
								int &iVertex0,
								CRVL3DPose *pPose,
								double scale,
								RVLPSULM_MESH_FILE_GROUP_DATA *GroupData,
								int nGroups)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	double fu = pBuilder->m_pStereoVision->m_KinectParams.depthFu;
	double fv = pBuilder->m_pStereoVision->m_KinectParams.depthFu;
	double uc = pBuilder->m_pStereoVision->m_KinectParams.depthUc;
	double vc = pBuilder->m_pStereoVision->m_KinectParams.depthVc;

	int nSamples = m_nSurfaceSamples;

	int i;
	CRVL3DSurface2 *p3DSurface; 
	RVL3DSURFACE_SAMPLE *pSample;

	double *R = pPose->m_Rot;
	double *t = pPose->m_X;

	m_nSurfaceSamples = 0;

	for(i=0;i<m_n3DSurfacesTotal;i++)
	{
		p3DSurface = m_3DSurfaceArray[i];

		pSample = (RVL3DSURFACE_SAMPLE *)(p3DSurface->m_Samples.pFirst);

		while(pSample)
		{
			m_nSurfaceSamples++;

			pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		}
	}

	double *VertexMem = new double[3 * 5 * m_nSurfaceSamples * nGroups];	//$

	//$double *VertexArray = new double[3 * 5 * m_nSurfaceSamples];

	double **VertexArray = new double *[nGroups];	//$

	//$double *pVertex = VertexArray;

	double **pVertex = new double *[nGroups];	//$

	double *NArray = new double[3 * 5 * m_nSurfaceSamples];

	double *pN = NArray;

	//$int *IdxMem = new int[3 * 4 * m_nSurfaceSamples * nGroups];

	//$int **IdxArray = new int *[nGroups];
	//$int **pIdx = new int *[nGroups];

	int iGroup;

	for(iGroup = 0; iGroup < nGroups; iGroup++)
	{
		//$IdxArray[iGroup] = IdxMem + iGroup * 3 * 4 * m_nSurfaceSamples;
		//$pIdx[iGroup] = IdxArray[iGroup];
		VertexArray[iGroup] = VertexMem + iGroup * 3 * 5 * m_nSurfaceSamples;	//$
		pVertex[iGroup] = VertexArray[iGroup];									//$
	}

	// create mesh 

	int j;
	int U[2];
	double U0[2], dU[2];
	double fTmp;
	double *X, *N;
	double X0[3], N0[3], X2[3];
	double *pVertex2;	//$

	for(i=0;i<m_n3DSurfacesTotal;i++)
	{
		p3DSurface = m_3DSurfaceArray[i];

		N = p3DSurface->m_N;

		RVLMULMX3X3TVECT(R, N, N0)

		pSample = (RVL3DSURFACE_SAMPLE *)(p3DSurface->m_Samples.pFirst);

		while(pSample)
		{
			for(iGroup = 0; iGroup < nGroups; iGroup++)
				if((pSample->Flags & GroupData[iGroup].Mask) == GroupData[iGroup].Value)
					break;

			if(iGroup == nGroups)
				iGroup = 0;

			pVertex2 = pVertex[iGroup];

			X = pSample->X;

			RVLMULMX3X3VECT(R, X, X0)
			RVLSUM3VECTORS(X0, t, X0)
			RVLCOPY3VECTOR(X0, pVertex2)
			
			pVertex2 += 3;

			RVLCOPY3VECTOR(N0, pN);

			pN += 3;

			U0[0] = fu * X[0] / X[2] + uc;
			U0[1] = fv * X[1] / X[2] + vc;

			dU[0] = dU[1] = scale * (pSample->wz / X[2] + 1.0);

			for(j = 0; j < 4; j++, pVertex2 += 3, pN += 3)
			{

				U[0] = DOUBLE2INT(U0[0] + dU[0]);
				U[1] = DOUBLE2INT(U0[1] + dU[1]);

				RVLProject2DPointTo3DPlane(U, X2, p3DSurface, uc, vc, fu, fv);

				RVLMULMX3X3VECT(R, X2, pVertex2)
				RVLSUM3VECTORS(pVertex2, t, pVertex2)

				RVLCOPY3VECTOR(N0, pN);

				//$*(pIdx[iGroup]++) = iVertex0;
				//$*(pIdx[iGroup]++) = iVertex0 + (j + 1) % 4 + 1;
				//$*(pIdx[iGroup]++) = iVertex0 + j + 1;

				fTmp = dU[0];
				dU[0] = -dU[1];
				dU[1] = fTmp;
			}

			pVertex[iGroup] = pVertex2;

			//$iVertex0 += 5;

			pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		}
	}

	fprintf(fp, "o %s\n", MeshName);

	//$BEGIN

	for(iGroup = 0; iGroup < nGroups; iGroup++)
	{
		fprintf(fp, "g %s_material_%d\n", MeshName, GroupData[iGroup].MaterialID);

		fprintf(fp, "usemtl material_%d\n", GroupData[iGroup].MaterialID);

		fprintf(fp, "s 1\n");

		double *pVertexArrayEnd = pVertex[iGroup];

		for(pVertex2 = VertexArray[iGroup]; pVertex2 < pVertexArrayEnd; pVertex2 += 3)
			fprintf(fp, "v %lf %lf %lf\n", pVertex2[0], pVertex2[1], pVertex2[2]);
	}

	//&END

	//$double *pVertexArrayEnd = pVertex;

	//$for(pVertex = VertexArray; pVertex < pVertexArrayEnd; pVertex += 3)
	//$	fprintf(fp, "v %lf %lf %lf\n", pVertex[0], pVertex[1], pVertex[2]);

	double *pNArrayEnd = pN;

	for(pN = NArray; pN < pNArrayEnd; pN += 3)
		fprintf(fp, "vn %lf %lf %lf\n", pN[0], pN[1], pN[2]);

	//$BEGIN

	int idx1, idx2, idx3;

	for(i = 0; i < m_nSurfaceSamples; i++)
	{
		for(j = 0; j < 4; j++)
		{
			idx1 = iVertex0;
			idx2 = iVertex0 + (j + 1) % 4 + 1;
			idx3 = iVertex0 + j + 1;

			fprintf(fp, "f %d//%d %d//%d %d//%d\n", idx1, idx1, idx2, idx2, idx3, idx3);
		}

		iVertex0 += 5;
	}

	//$END

	//$int *Face;

	//$for(iGroup = 0; iGroup < nGroups; iGroup++)
	//${
	//$	fprintf(fp, "g %s_material_%d\n", MeshName, GroupData[iGroup].MaterialID);

	//$	fprintf(fp, "usemtl material_%d\n", GroupData[iGroup].MaterialID);

	//$	fprintf(fp, "s 1\n");

	//$	for(Face = IdxArray[iGroup]; Face < pIdx[iGroup]; Face += 3)
	//$		fprintf(fp, "f %d//%d %d//%d %d//%d\n", Face[0], Face[0], Face[1], Face[1], Face[2], Face[2]);
	//$}

	delete[] VertexMem;	//$
	delete[] pVertex; //$
	//$delete[] IdxArray;
	//$delete[] pIdx;
	//$delete[] IdxMem;
	delete[] NArray;
	delete[] VertexArray;
}

#ifdef NEVER

void CRVLPSuLM::CreateMeshFile(char *MeshFileName,
							   RVLPSULM_HYPOTHESIS *pHypothesis)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)m_vpBuilder;

	double fu = pBuilder->m_pStereoVision->m_KinectParams.depthFu;
	double fv = pBuilder->m_pStereoVision->m_KinectParams.depthFu;
	double uc = pBuilder->m_pStereoVision->m_KinectParams.depthUc;
	double vc = pBuilder->m_pStereoVision->m_KinectParams.depthVc;

	int nSamples = m_nSurfaceSamples;

	CRVLPSuLM *pMPSuLM;
	double *RSM;
	double *tSM;
	int i;
	CRVL3DSurface2 *pM3DSurface; 
	RVL3DSURFACE_SAMPLE *pSampleM;

	if(pHypothesis)
	{
		pMPSuLM = pHypothesis->pMPSuLM;

		RSM = pHypothesis->PoseSM.m_Rot;
		tSM = pHypothesis->PoseSM.m_X;

		pMPSuLM->m_nSurfaceSamples = 0;

		for(i=0;i<pMPSuLM->m_n3DSurfacesTotal;i++)
		{
			pM3DSurface = pMPSuLM->m_3DSurfaceArray[i];

			pSampleM = (RVL3DSURFACE_SAMPLE *)(pM3DSurface->m_Samples.pFirst);

			while(pSampleM)
			{
				pMPSuLM->m_nSurfaceSamples++;

				pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);
			}
		}

		if(pMPSuLM->m_nSurfaceSamples > nSamples)
			nSamples = pMPSuLM->m_nSurfaceSamples;
	}

	double *VertexArray = new double[3 * 5 * nSamples];

	double *pVertex = VertexArray;

	double *NArray = new double[3 * 5 * nSamples];

	double *pN = NArray;

	int *IdxArray = new int[3 * 4 * nSamples];

	int *pIdx = IdxArray;

	// create mesh from this

	int iVertex0 = 1;

	int j;
	RVL3DSURFACE_SAMPLE *pSampleS;
	CRVL3DSurface2 *pS3DSurface; 
	int U[2];
	double U0[2], dU[2];
	double fTmp;
	double *X0, *N;

	for(i=0;i<m_n3DSurfacesTotal;i++)
	{
		pS3DSurface = m_3DSurfaceArray[i];

		N = pS3DSurface->m_N;

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

		while(pSampleS)
		{
			X0 = pSampleS->X;

			RVLCOPY3VECTOR(X0, pVertex);

			pVertex += 3;

			RVLCOPY3VECTOR(N, pN);

			pN += 3;

			U0[0] = fu * X0[0] / X0[2] + uc;
			U0[1] = fv * X0[1] / X0[2] + vc;

			dU[0] = dU[1] = pSampleS->wz / X0[2] + 1.0;

			for(j = 0; j < 4; j++, pVertex += 3, pN += 3)
			{
				U[0] = DOUBLE2INT(U0[0] + dU[0]);
				U[1] = DOUBLE2INT(U0[1] + dU[1]);

				RVLProject2DPointTo3DPlane(U, pVertex, pS3DSurface, uc, vc, fu, fv);

				RVLCOPY3VECTOR(N, pN);

				*(pIdx++) = iVertex0;
				*(pIdx++) = iVertex0 + (j + 1) % 4 + 1;
				*(pIdx++) = iVertex0 + j + 1;

				fTmp = dU[0];
				dU[0] = -dU[1];
				dU[1] = fTmp;
			}

			iVertex0 += 5;

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}
	}

	FILE *fp = fopen(MeshFileName, "w");

	fprintf(fp, "mtllib kinect_example.mtl\n");

	fprintf(fp, "o scene\n");

	double *pVertexArrayEnd = pVertex;

	for(pVertex = VertexArray; pVertex < pVertexArrayEnd; pVertex += 3)
		fprintf(fp, "v %lf %lf %lf\n", pVertex[0], pVertex[1], pVertex[2]);

	double *pNArrayEnd = pN;

	for(pN = NArray; pN < pNArrayEnd; pN += 3)
		fprintf(fp, "vn %lf %lf %lf\n", pN[0], pN[1], pN[2]);

	fprintf(fp, "g scene_material_30\n");

	fprintf(fp, "usemtl material_30\n");

	fprintf(fp, "s 1\n");

	int *pIdxArrayEnd = pIdx;

	for(pIdx = IdxArray; pIdx < pIdxArrayEnd; pIdx += 3)
		fprintf(fp, "f %d//%d %d//%d %d//%d\n", pIdx[0], pIdx[0], pIdx[1], pIdx[1], pIdx[2], pIdx[2]);

	// create mesh from pMPSuLM

	pVertex = VertexArray;

	pN = NArray;

	pIdx = IdxArray;
	
	double NMS[3], tmp3x1[3];
	double XMS0[3], P[3], Q[3], XMS[3];

	for(i=0;i<pMPSuLM->m_n3DSurfacesTotal;i++)
	{
		pM3DSurface = pMPSuLM->m_3DSurfaceArray[i];

		N = pM3DSurface->m_N;

		RVLMULMX3X3TVECT(RSM, N, NMS)

		j = 0;

		if(fabs(NMS[1]) < fabs(NMS[0]))
			j = 1;

		if(fabs(NMS[2]) < fabs(NMS[j]))
			j = 2;

		RVLNULL3VECTOR(P)

		P[j] = 1.0;

		RVLCROSSPRODUCT3(NMS, P, Q);
		fTmp = sqrt(RVLDOTPRODUCT3(Q, Q));
		RVLSCALE3VECTOR2(Q, fTmp, Q);
		RVLCROSSPRODUCT3(Q, NMS, P);

		pSampleM = (RVL3DSURFACE_SAMPLE *)(pM3DSurface->m_Samples.pFirst);

		while(pSampleM)
		{
			X0 = pSampleM->X;

			RVLDIF3VECTORS(X0, tSM, tmp3x1)
			RVLMULMX3X3TVECT(RSM, tmp3x1, XMS0)

			RVLCOPY3VECTOR(XMS0, pVertex)

			pVertex += 3;

			RVLCOPY3VECTOR(NMS, pN);

			pN += 3;

			dU[0] = dU[1] = 20.0;

			for(j = 0; j < 4; j++, pVertex += 3, pN += 3)
			{
				XMS[0] = XMS0[0] + dU[0] * P[0] + dU[1] * Q[0];
				XMS[1] = XMS0[1] + dU[0] * P[1] + dU[1] * Q[1];
				XMS[2] = XMS0[2] + dU[0] * P[2] + dU[1] * Q[2];

				RVLCOPY3VECTOR(XMS, pVertex)

				RVLCOPY3VECTOR(NMS, pN);

				*(pIdx++) = iVertex0;
				*(pIdx++) = iVertex0 + (j + 1) % 4 + 1;
				*(pIdx++) = iVertex0 + j + 1;

				fTmp = dU[0];
				dU[0] = -dU[1];
				dU[1] = fTmp;
			}

			iVertex0 += 5;

			pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);
		}
	}

	fprintf(fp, "o model\n");

	pVertexArrayEnd = pVertex;

	for(pVertex = VertexArray; pVertex < pVertexArrayEnd; pVertex += 3)
		fprintf(fp, "v %lf %lf %lf\n", pVertex[0], pVertex[1], pVertex[2]);

	pNArrayEnd = pN;

	for(pN = NArray; pN < pNArrayEnd; pN += 3)
		fprintf(fp, "vn %lf %lf %lf\n", pN[0], pN[1], pN[2]);

	fprintf(fp, "g model_material_0\n");

	fprintf(fp, "usemtl material_0\n");

	fprintf(fp, "s 1\n");

	pIdxArrayEnd = pIdx;

	for(pIdx = IdxArray; pIdx < pIdxArrayEnd; pIdx += 3)
		fprintf(fp, "f %d//%d %d//%d %d//%d\n", pIdx[0], pIdx[0], pIdx[1], pIdx[1], pIdx[2], pIdx[2]);

	fclose(fp);

	delete[] IdxArray;
	delete[] NArray;
	delete[] VertexArray;
}

#endif

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 

void RVLPSuLMDisplayMouseCallback(int event, int x, int y, int flags, void* vpFig)
{
	CRVLFigure *pFig = (CRVLFigure *)vpFig;	

	BOOL bDraw = FALSE;

	if(pFig->MouseCallback3DDisplay(event, x, y, flags))
		bDraw = TRUE;

	switch( event )
	{
		case CV_EVENT_LBUTTONDOWN:
			pFig->m_iSelectedPix = -1;

			break;
		case CV_EVENT_RBUTTONDOWN:
			pFig->m_iSelectedPix = x + y * pFig->m_pImage->width;

			bDraw = TRUE;
	}

	if(bDraw)
		RVLPSuLMDisplay(pFig);
}

void RVLPSuLMDisplay(CRVLFigure *pFig)
{
	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	CRVLPSuLM *pSPSuLM = pMouseCallbackData->pSPSuLM;
	CRVLPSuLM *pMPSuLM = pMouseCallbackData->pMPSuLM;

	BYTE *MatchMatrix = pMouseCallbackData->MatrixSceneModel;

	int nMSurfs = pMouseCallbackData->nMSurfs;

	CRVLPSuLMBuilder *pPSuLMBuilder = (CRVLPSuLMBuilder *)(pSPSuLM->m_vpBuilder);

	int iSelectedPix = pFig->m_iSelectedPix;

	CRVL3DSurface2 *pSelectedSurf;
	CRVL3DLine2 *pSelectedLine;

	pSPSuLM->Display(pFig, &(pMouseCallbackData->PoseS0), cvScalar(0, 255, 0), 
		(pMouseCallbackData->Flags & ~(RVLPSULM_DISPLAY_MCELLS | RVLPSULM_DISPLAY_PROJ_MCELLS)) | RVLPSULM_DISPLAY_CLEAR,
		iSelectedPix, &pSelectedSurf, &pSelectedLine);

	if(iSelectedPix >= 0)
	{
		pMouseCallbackData->pSSurf = pSelectedSurf;

		pMouseCallbackData->pMSurf = NULL;

		pMouseCallbackData->Flags |= RVLPSULM_DISPLAY_SELECTION;
	}

	//double eig[3];
	//double VNrm[3 * 3], V[3 * 3];
	//double lV[3];

	char str[6];

	if(pMouseCallbackData->pSSurf)
	{
		pSPSuLM->Display3DSurface(pFig, pMouseCallbackData->pSSurf, &(pMouseCallbackData->PoseS0), cvScalar(0, 255, 255));

		sprintf(str, "S%d", pMouseCallbackData->pSSurf->m_Index);

		pFig->PutText(str, cvPoint(8, pFig->m_FontSize), cvScalar(0, 255, 255));

		//RVLGetAxesOfCov3D(pMouseCallbackData->pSSurf->m_PoseContribution, eig, VNrm, V, lV);

		//int debug = 0;
	}

	//RVLPSULM_MSMATCH_DATA *MatchArray;
	int iMatch;

	if(pMPSuLM)
	{
		pMPSuLM->Display(pFig, &(pMouseCallbackData->PoseM0), cvScalar(255, 0, 255), 
			pMouseCallbackData->Flags & ~(RVLPSULM_DISPLAY_SCELLS | RVLPSULM_DISPLAY_PROJ_SCELLS), 
			iSelectedPix, &pSelectedSurf, &pSelectedLine);

		if(pMouseCallbackData->Flags & RVLPSULM_DISPLAY_SELECTION)
		{
			if(pMouseCallbackData->pSSurf)
			{
				// display all geometrically consistent model surfaces

				if(MatchMatrix)
				{
					for(iMatch = 0; iMatch < pMPSuLM->m_n3DSurfaces; iMatch++)
						if(MatchMatrix[pMouseCallbackData->pSSurf->m_Index * pMPSuLM->m_n3DSurfaces + iMatch] == 2)
						{
							pSelectedSurf = pMPSuLM->m_3DSurfaceArray[iMatch];

							pMPSuLM->Display3DSurface(pFig, pSelectedSurf, &(pMouseCallbackData->PoseM0), cvScalar(204, 204, 255));
						}
				}

				pMouseCallbackData->pMSurf = NULL;
			}
			else if(pSelectedSurf)
				pMouseCallbackData->pMSurf = pSelectedSurf;
		}

		if(pMouseCallbackData->pMSurf)
		{
			pMPSuLM->Display3DSurface(pFig, pMouseCallbackData->pMSurf, &(pMouseCallbackData->PoseM0), cvScalar(204, 204, 255));

			sprintf(str, "M%d", pMouseCallbackData->pMSurf->m_Index);

			pFig->PutText(str, cvPoint(40, pFig->m_FontSize), cvScalar(0, 255, 255));

			//RVLGetAxesOfCov3D(pMouseCallbackData->pMSurf->m_PoseContribution, eig, VNrm, V, lV);

			//int debug = 0;
		}
	}

	iSelectedPix = -1;

	pFig->DisplayFocus(pPSuLMBuilder->m_pCamera);

	CRVLGUI *pGUI = (CRVLGUI *)(pFig->m_vpGUI);

	pGUI->ShowFigure(pFig);	
}

void RVLDisplayColoredDepthMap(CRVL3DMeshObject *pSceneMesh,
							   int w,
							   int h,
							   PIX_ARRAY *pPixArray)
{
	pPixArray->Width = w;
	pPixArray->Height = h;

	int ImageSize = w * h;

	unsigned char *pPix0 = pPixArray->pPix;

	memset(pPix0, 0, 3 * w * h * sizeof(unsigned char));

	int HistRGBBaseLog2 = pSceneMesh->m_pClass->m_HistRGBBaseLog2;
	int HistRGBBaseSquareLog2 = (HistRGBBaseLog2 << 1);
	ushort mask = ((1 << HistRGBBaseLog2) - 1);
	int HistRGBBinSizeLog2 = 8 - HistRGBBaseLog2;

	RVLQLIST_PTR_ENTRY *pSurfacePtr = (RVLQLIST_PTR_ENTRY *)(pSceneMesh->m_ChildMeshObjects->pFirst);
	
	CRVL3DSurface2 *p3DSurface;
	CRVL2DRegion2 *pTriangle;
	RVLQLIST_PTR_ENTRY *pTrianglePtr;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	RVLCOLOR Color;
	RVLQLIST_HIST_ENTRY_SHORT *pHistRGBEntry;
	ushort HistRGBBin;
	unsigned char *pPix;

	while(pSurfacePtr)
	{
		p3DSurface = (CRVL3DSurface2 *)(pSurfacePtr->Ptr);

		pHistRGBEntry = (RVLQLIST_HIST_ENTRY_SHORT *)(p3DSurface->m_histRGB->pFirst);

		if(pHistRGBEntry)
		{
			HistRGBBin = pHistRGBEntry->adr;

			Color.r = (BYTE)((HistRGBBin >> HistRGBBaseSquareLog2) << HistRGBBinSizeLog2);
			Color.g = (BYTE)(((HistRGBBin >> HistRGBBaseLog2) & mask) << HistRGBBinSizeLog2);
			Color.b = (BYTE)((HistRGBBin & mask) << HistRGBBinSizeLog2);

			pTrianglePtr = (RVLQLIST_PTR_ENTRY *)(p3DSurface->m_FaceList->pFirst);

			while(pTrianglePtr)
			{
				pTriangle = (CRVL2DRegion2 *)(pTrianglePtr->Ptr);

				pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

				for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
				{
					pPt = *ppPt;

					pPix = pPix0 + 3 * pPt->iPix;

					*(pPix++) = Color.r;
					*(pPix++) = Color.g;
					*pPix = Color.b;
				}
				
				pTrianglePtr = (RVLQLIST_PTR_ENTRY *)(pTrianglePtr->pNext);
			}
		}

		pSurfacePtr = (RVLQLIST_PTR_ENTRY *)(pSurfacePtr->pNext);
	}
}

