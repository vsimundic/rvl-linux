// RVLSegmentation.cpp: implementation of the CRVLSegmentation class.
//
//////////////////////////////////////////////////////////////////////

#include "RVLCore.h"
#include "RVLPCS.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLSegmentation::CRVLSegmentation()
{
	m_Flags = 0x00000000;

	m_pROI = NULL;
}

CRVLSegmentation::~CRVLSegmentation()
{
}

void CRVLSegmentation::Init()
{
}

void CRVLSegmentation::Clear()
{

}

void CRVLSegmentation::Segment(PIX_ARRAY *pPixArray,
							   CRVLAImage *pAImage,
							   CRVLC2D *p2DRegionSet)
{

}

void CRVLSegmentation::CreateParamList(CRVLMem * pMem)
{

}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 

void RVLSaveSegmentation(FILE *fp,
						 CRVLClass *p2DRegionSet,
						 int ImageWidth)
{
	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	CRVL2DRegion2 *pPolygon;

	RVLMESH_LINK *pLink0, *pLink, *pLink2;
	int iPix, iPix2;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);

		do
		{
			pLink2 = pLink->pOpposite;

			//if(((pLink->Flags & RVLMESH_LINK_FLAG_BOUNDARY) !=0)  && ((pLink2->Flags & RVLMESH_LINK_FLAG_BOUNDARY) !=0) ) // && ((pLink->Flags & RVLMESH_LINK_FLAG_BACKGROUND) ==0)
			//if((pLink->Flags & RVLMESH_LINK_FLAG_FOREGROUND) !=0)
			//{

			iPix = pLink->iPix0;
			iPix2 = pLink2->iPix0;

			pPolygon = (CRVL2DRegion2 *)(pLink2->vp2DRegion);

			if(pPolygon)
				if((pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
					if(iPix > iPix2)
					{
						pLink = pLink->pNext->pOpposite;

						continue;
					}

			fprintf(fp, "%d\t%d\t%d\t%d\n", iPix % ImageWidth, iPix / ImageWidth,
				iPix2 % ImageWidth, iPix2 / ImageWidth);

			//}

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);
	}
}

//#define RVLUPDATECONVEXHULL_DEBUG

BOOL RVLUpdateConvexHull(CRVLMPtrChain *pTriangleList,
						 CRVLClass *pTriangleSet,
						 int ImageWidth,
						 RVL3DPOINT2 **Point3DMap,
						 RVLMESH_LINK *pLinkNewPt,
						 int maxDist,
						 CRVLMem *pMem,
						 bool bmm)
{
	int iPixNew = pLinkNewPt->iPix0;

	//if(iPixNew == 31670)
	//	int debug = 0;

	//if((int)pLinkNewPt == 0x023f913c)
	//	int debug = 0;

	RVL3DPOINT2 *pPixNew = Point3DMap[iPixNew];

	int uNew = pPixNew->u;
	int vNew = pPixNew->v;

	//if(vNew < 120)
	//	int tmp1 = 0;

#ifdef RVLUPDATECONVEXHULL_DEBUG
	FILE *fpLog = fopen("Debug\\RVLUpdateConvexHull.log", "w");
#endif

	// Classify all triangles and links to those which belong to the new convex hull
	// and those which don't.

	RVLMESH_LINK *pFirstBoundaryLink = NULL;

	int nPtsToReassign = 0;
	int nNewTriangles = 0;

	CRVL2DRegion2 **RejectedTriangleArray = new CRVL2DRegion2 *[pTriangleList->m_nElements];

	CRVL2DRegion2 **pRejectedTriangle = RejectedTriangleArray;

	int minDist = maxDist;

	int64 minDist64 = (int64)maxDist;

	double minfDist = (double)maxDist;

	CRVL2DRegion2 *pClosestTriangle = NULL;
	
	CRVL2DRegion2 *pTriangle, *pAdjacentTriangle;
	RVLMESH_LINK *pLink0, *pLink, *pLinkPrev, *pLinkNext;
	int *N;
	//double *fN;
	int dist;
	int64 dist64;
	//double fdist;
	//double *XNew;
	int dX[3];
	int *XNew;
	int *X0;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if(bmm)
		{
//			fN = pTriangle->m_fN;
//
//			XNew = pPixNew->XYZ;
//
//			fdist = pTriangle->m_rho - RVLDOTPRODUCT3(fN, XNew);
//
//			if(fdist < 0.0)
//			{
//				pLink = pLink0 = (RVLMESH_LINK *)(pTriangle->m_PtArray);
//
//				do
//				{
//					pAdjacentTriangle = (CRVL2DRegion2 *)(pLink->pOpposite->vp2DRegion);
//
//					fN = pAdjacentTriangle->m_fN;
//
//					if(RVLDOTPRODUCT3(fN, XNew) <= pAdjacentTriangle->m_rho)
//					{
//						if(pFirstBoundaryLink == NULL)
//							pFirstBoundaryLink = pLink;
//
//						nNewTriangles++;
//
//						pLink->Flags |= RVLMESH_LINK_FLAG_BOUNDARY;
//
//#ifdef RVLUPDATECONVEXHULL_DEBUG 
//						RVL3DPOINT2 *p3DPtDebug1 = Point3DMap[pLink->iPix0];
//						RVL3DPOINT2 *p3DPtDebug2 = Point3DMap[pLink->pOpposite->iPix0];
//
//						fprintf(fpLog, "%d\t%d\t%d\t%d\n", p3DPtDebug1->u, p3DPtDebug1->v, p3DPtDebug2->u, p3DPtDebug2->v);
//#endif
//					}
//
//					pLink = pLink->pNext->pOpposite;
//				}
//				while(pLink != pLink0);
//
//				*(pRejectedTriangle++) = pTriangle;
//
//				nPtsToReassign += pTriangle->m_n3DPts;
//			}
//			else if(pTriangle->m_rho < 0.0) 
//			{
//				if(fdist <= minfDist)
//				{
//					minfDist = fdist;
//
//					pClosestTriangle = pTriangle;
//				}
//			}

			N = pTriangle->m_N;

			X0 = pTriangle->m_X0;

			XNew = pPixNew->iX;

			RVLDIF3VECTORS(X0, XNew, dX);

			dist64 = RVLDOTPRODUCT3_64(N, dX);

			if(dist64 < 0)
			{
				pLink = pLink0 = (RVLMESH_LINK *)(pTriangle->m_PtArray);

				do
				{
					pAdjacentTriangle = (CRVL2DRegion2 *)(pLink->pOpposite->vp2DRegion);

					N = pAdjacentTriangle->m_N;

					X0 = pAdjacentTriangle->m_X0;

					RVLDIF3VECTORS(XNew, X0, dX);

					if(RVLDOTPRODUCT3_64(N, dX) <= 0)
					{
						if(pFirstBoundaryLink == NULL)
							pFirstBoundaryLink = pLink;

						nNewTriangles++;

						pLink->Flags |= RVLMESH_LINK_FLAG_BOUNDARY;
					}

					pLink = pLink->pNext->pOpposite;
				}
				while(pLink != pLink0);

				*(pRejectedTriangle++) = pTriangle;

				nPtsToReassign += pTriangle->m_n3DPts;
			}
			else if((pTriangle->m_Flags & RVL2DREGION_FLAG_INVISIBLE) == 0) 
			{
				dist64 /= (int64)(pTriangle->m_lenN);

				if(dist64 <= minDist64)
				{
					minDist64 = dist64;

					pClosestTriangle = pTriangle;
				}
			}
		}	// if(bmm)
		else
		{
			N = pTriangle->m_N;

			dist = pTriangle->m_d - (N[0] * pPixNew->u + N[1] * pPixNew->v + N[2] * pPixNew->d);

			if(dist < 0)
			{
				pLink = pLink0 = (RVLMESH_LINK *)(pTriangle->m_PtArray);

				do
				{
					pAdjacentTriangle = (CRVL2DRegion2 *)(pLink->pOpposite->vp2DRegion);

					N = pAdjacentTriangle->m_N;

					if(N[0] * pPixNew->u + N[1] * pPixNew->v + N[2] * pPixNew->d <= pAdjacentTriangle->m_d)
					{
						if(pFirstBoundaryLink == NULL)
							pFirstBoundaryLink = pLink;

						nNewTriangles++;

						pLink->Flags |= RVLMESH_LINK_FLAG_BOUNDARY;
					}

					pLink = pLink->pNext->pOpposite;
				}
				while(pLink != pLink0);

				*(pRejectedTriangle++) = pTriangle;

				nPtsToReassign += pTriangle->m_n3DPts;
			}
			else if(N[2] < 0) 
			{
				dist /= pTriangle->m_lenN;

				if(dist <= minDist)
				{
					minDist = dist;

					pClosestTriangle = pTriangle;
				}
			}
		}	// if(!bmm)
	}	// for every triangle in pTriangleList

#ifdef RVLUPDATECONVEXHULL_DEBUG
	fclose(fpLog);
#endif

	RVLMESH_LINK **PtMem;

	if(pFirstBoundaryLink == NULL)		// the new point is inside the convex hull
	{
		if(pClosestTriangle)
		{
			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RVLMESH_LINK *, (pClosestTriangle->m_n3DPts + 1), PtMem);

			memcpy(PtMem + 1, pClosestTriangle->m_pPoint3DArray, pClosestTriangle->m_n3DPts * sizeof(RVLMESH_LINK *));

			PtMem[0] = pLinkNewPt;

			pClosestTriangle->m_n3DPts++;

			pClosestTriangle->m_pPoint3DArray = (RVL3DPOINT2 **)PtMem;

			delete[] RejectedTriangleArray;

			return TRUE;
		}
		else
		{
			delete[] RejectedTriangleArray;

			return FALSE;
		}
	}

	CRVL2DRegion2 **pRejectedTriangleArrayEnd = pRejectedTriangle; 

	// Determine the boundary between the old and the new convex hull.
	// Compute the new triangle plane parameters

	CRVL2DRegion2 *NewTriangleArray = new CRVL2DRegion2[nNewTriangles];

	CRVL2DRegion2 *pNewTriangleArrayEnd = NewTriangleArray + nNewTriangles;

	pTriangle = NewTriangleArray;

	pLink = pFirstBoundaryLink;

	int dd1, dd2;
	RVL3DPOINT2 *p3DPt0, *p3DPt1, *p3DPt2;
	int u, v, du, dv;
	double fN_[3];
	int *X1, *X2;
	int dX1[3], dX2[3];
	double fTmp;
	int *X0_;

	do
	{
		pTriangle->m_PtArray = pLink;

		p3DPt0 = Point3DMap[pLink->iPix0];

		p3DPt1 = pPixNew;

		p3DPt2 = Point3DMap[pLink->pOpposite->iPix0];

		if(bmm)
		{
			//X0 = p3DPt0->XYZ;
			//X1 = p3DPt1->XYZ;
			//X2 = p3DPt2->XYZ;

			//RVLDIF3VECTORS(X1, X0, dX1)
			//RVLDIF3VECTORS(X2, X0, dX2)

			//fN = pTriangle->m_fN;

			//RVLCROSSPRODUCT3(dX1, dX2, fN)

			//RVLNORM3(fN, fTmp)

			//pTriangle->m_rho = RVLDOTPRODUCT3(X0, fN);

			X0 = p3DPt0->iX;
			X1 = p3DPt1->iX;
			X2 = p3DPt2->iX;

			RVLDIF3VECTORS(X1, X0, dX1)
			RVLDIF3VECTORS(X2, X0, dX2)

			N = pTriangle->m_N;

			RVLCROSSPRODUCT3(dX1, dX2, N)

			pTriangle->m_lenN = DOUBLE2INT(sqrt((double)(N[0]) * (double)(N[0]) + (double)(N[1]) * (double)(N[1]) + (double)(N[2]) * (double)(N[2])));

			//if(pTriangle->m_lenN == 0)
			//	int debug = 0;

			X0_ = pTriangle->m_X0;

			RVLCOPY3VECTOR(X0, X0_)

			pTriangle->m_Flags = (RVLDOTPRODUCT3_64(N, X0) < 0 ? 0x00000000 : RVL2DREGION_FLAG_INVISIBLE);
		}
		else
		{
			u = p3DPt0->u;
			v = p3DPt0->v;

			du = uNew - u;
			dv = vNew - v;

			dd1 = p3DPt1->d - p3DPt0->d;
			dd2 = p3DPt2->d - p3DPt0->d;

			N = pTriangle->m_N;

			N[0] = dv * dd2 - dd1 * pLink->dv;
			N[1] = dd1 * pLink->du - du * dd2;
			N[2] = -dv * pLink->du + du * pLink->dv;

			//if(N[0] == -144 && N[1] == 0 && N[2] == 0)
			//	int debug = 0;

			fN_[0] = (double)N[0];
			fN_[1] = (double)N[1];
			fN_[2] = (double)N[2];

			pTriangle->m_lenN = DOUBLE2INT(sqrt(fN_[0] * fN_[0] + fN_[1] * fN_[1] + fN_[2] * fN_[2]));

			pTriangle->m_d = N[0] * u + N[1] * v + N[2] * p3DPt0->d;
		}

		pTriangle->m_n3DPts = 0;

		pTriangle++;

		pLinkNext = pLink->pNext;

		while((pLinkNext->pOpposite->Flags & RVLMESH_LINK_FLAG_BOUNDARY) == 0)
			pLinkNext = pLinkNext->pNext;

		pLink = pLinkNext->pOpposite;

		pLink->Flags &= ~RVLMESH_LINK_FLAG_BOUNDARY;
	}
	while(pLink != pFirstBoundaryLink);

	//if(pTriangle - NewTriangleArray != nNewTriangles)
	//	int debug = 0;

	// determine if the new surface is approximatelly convex 

	CRVL2DRegion2 *pNewVisibleTriangle;

	for(pNewVisibleTriangle = NewTriangleArray; pNewVisibleTriangle < pNewTriangleArrayEnd; pNewVisibleTriangle++)
		if(bmm)
		{
			if((pNewVisibleTriangle->m_Flags & RVL2DREGION_FLAG_INVISIBLE) == 0)
				break;
		}
		else
		{
			if(pNewVisibleTriangle->m_N[2] < 0)
				break;
		}

	int OneMorePt;

	if(pNewVisibleTriangle == pNewTriangleArrayEnd)
	{
		if(pClosestTriangle)
		{
			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RVLMESH_LINK *, (pClosestTriangle->m_n3DPts + 1), PtMem);

			memcpy(PtMem + 1, pClosestTriangle->m_pPoint3DArray, pClosestTriangle->m_n3DPts * sizeof(RVLMESH_LINK *));

			PtMem[0] = pLinkNewPt;

			pClosestTriangle->m_n3DPts++;

			pClosestTriangle->m_pPoint3DArray = (RVL3DPOINT2 **)PtMem;

			OneMorePt = 0;
		}
		else
		{
			delete[] RejectedTriangleArray;
			delete[] NewTriangleArray;	

			return FALSE;
		}
	}
	else
	{
		pNewVisibleTriangle->m_n3DPts = 1;

		OneMorePt = 1;
	}

	CRVL2DRegion2 **PtTriangleArray = new CRVL2DRegion2 *[nPtsToReassign];

	CRVL2DRegion2 **pPtTriangle = PtTriangleArray;

	RVL3DPOINT2 **PtToReasignArray = new RVL3DPOINT2 *[nPtsToReassign];

	RVLMESH_LINK **ppPt2 = (RVLMESH_LINK **)PtToReasignArray;

	bool bVisibleTrianglesAreRejected = false;

	CRVL2DRegion2 *pNewTriangle;

	RVLMESH_LINK **ppPt;
	RVLMESH_LINK **pPtArrayEnd;

	for(pRejectedTriangle = RejectedTriangleArray; pRejectedTriangle < pRejectedTriangleArrayEnd; pRejectedTriangle++)
	{
		pTriangle = *pRejectedTriangle;

		if(pTriangle->m_n3DPts == 0)
			continue;

		bVisibleTrianglesAreRejected = true;

		pPtArrayEnd = (RVLMESH_LINK **)(pTriangle->m_pPoint3DArray) + pTriangle->m_n3DPts;

		for(ppPt = (RVLMESH_LINK **)(pTriangle->m_pPoint3DArray); ppPt < pPtArrayEnd; ppPt++)
		{
			pLink = *ppPt;

			p3DPt0 = Point3DMap[pLink->iPix0];

			*(ppPt2++) = pLink;

			minDist = maxDist;

			//minfDist = (double)maxDist;

			minDist64 = (int64)maxDist;

			pClosestTriangle = NULL;

			for(pNewTriangle = NewTriangleArray; pNewTriangle < pNewTriangleArrayEnd; pNewTriangle++)
			{
				if(bmm)
				{
					//fN = pNewTriangle->m_fN;

					//if(pNewTriangle->m_rho >= 0)
					//	continue;

					//X0 = p3DPt0->XYZ;

					//fdist = pNewTriangle->m_rho - RVLDOTPRODUCT3(fN, X0);

					//if(fdist <= minfDist)
					//{
					//	minfDist = fdist;

					//	pClosestTriangle = pNewTriangle;
					//}

					if(pNewTriangle->m_Flags & RVL2DREGION_FLAG_INVISIBLE)
						continue;

					N = pNewTriangle->m_N;

					X0 = p3DPt0->iX;

					X0_ = pNewTriangle->m_X0;

					RVLDIF3VECTORS(X0_, X0, dX);

					dist64 = RVLDOTPRODUCT3_64(N, dX) / (int64)(pNewTriangle->m_lenN);

					if(dist64 <= minDist64)
					{
						minDist64 = dist64;

						pClosestTriangle = pNewTriangle;
					}
				}
				else
				{
					N = pNewTriangle->m_N;

					if(N[2] >= 0)
						continue;

					dist = (pNewTriangle->m_d - (N[0] * p3DPt0->u + N[1] * p3DPt0->v + N[2] * p3DPt0->d)) / pNewTriangle->m_lenN;

					if(dist <= minDist)
					{
						minDist = dist;

						pClosestTriangle = pNewTriangle;
					}
				}
			}	// for each new triangle in the new convex hull

			if(pClosestTriangle == NULL)
				break;

			*(pPtTriangle++) = pClosestTriangle;

			pClosestTriangle->m_n3DPts++;
		}	// for every point in pTriangle

		if(pClosestTriangle == NULL)
			break;
	}

	if(bVisibleTrianglesAreRejected)
		if(pClosestTriangle == NULL)
		{
			delete[] RejectedTriangleArray;
			delete[] NewTriangleArray;	
			delete[] PtToReasignArray;
			delete[] PtTriangleArray;

			return FALSE;
		}

	// mark rejected triangles

	for(pRejectedTriangle = RejectedTriangleArray; pRejectedTriangle < pRejectedTriangleArrayEnd; pRejectedTriangle++)
	{
		pTriangle = *pRejectedTriangle;

		pTriangle->m_Flags |= RVLOBJ2_FLAG_REJECTED;
	}

	delete[] RejectedTriangleArray;

	// assign pts. to the new triangles

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RVLMESH_LINK *, (nPtsToReassign + OneMorePt), PtMem);

	ppPt = PtMem;

	for(pNewTriangle = NewTriangleArray; pNewTriangle < pNewTriangleArrayEnd; pNewTriangle++)
	{
		pNewTriangle->m_pPoint3DArray = (RVL3DPOINT2 **)ppPt;

		ppPt += pNewTriangle->m_n3DPts;

		pNewTriangle->m_n3DPts = 0;
	}

	if(OneMorePt)
	{
		pNewVisibleTriangle->m_pPoint3DArray[0] = (RVL3DPOINT2 *)pLinkNewPt;

		pNewVisibleTriangle->m_n3DPts = 1;
	}

	int iPt;

	for(iPt = 0; iPt < nPtsToReassign; iPt++)
	{
		pTriangle = PtTriangleArray[iPt];

		pTriangle->m_pPoint3DArray[pTriangle->m_n3DPts] = PtToReasignArray[iPt];

		pTriangle->m_n3DPts++;
	}

	delete[] PtToReasignArray;
	delete[] PtTriangleArray;

	// for each boundary link create a new triangle

	RVLMESH_LINK *NewLinkArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RVLMESH_LINK, 2 * nNewTriangles, NewLinkArray);

	RVLMESH_LINK *pNewLink = NewLinkArray;

	pNewTriangle = NewTriangleArray;

	RVLMESH_LINK *pLinkNew1, *pLinkNew2;
	int iLink;

	for(iLink = 0; iLink < nNewTriangles; iLink++)
	{
		pLink = (RVLMESH_LINK *)(pNewTriangle->m_PtArray);

		pTriangle = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(pTriangleSet));

		//if((int)pTriangle == 0x03852988)
		//	int debug = 0;

		if(bmm)
		{
			//RVLCOPY3VECTOR(pNewTriangle->m_fN, pTriangle->m_fN);

			//pTriangle->m_rho = pNewTriangle->m_rho;

			RVLCOPY3VECTOR(pNewTriangle->m_N, pTriangle->m_N);

			RVLCOPY3VECTOR(pNewTriangle->m_X0, pTriangle->m_X0);

			pTriangle->m_lenN = pNewTriangle->m_lenN;

			pTriangle->m_Flags = pNewTriangle->m_Flags;
		}
		else
		{
			RVLCOPY3VECTOR(pNewTriangle->m_N, pTriangle->m_N);

			pTriangle->m_lenN = pNewTriangle->m_lenN;

			pTriangle->m_d = pNewTriangle->m_d;
		}

		pTriangle->m_n3DPts = pNewTriangle->m_n3DPts;

		pTriangle->m_pPoint3DArray = pNewTriangle->m_pPoint3DArray;

		pTriangle->m_PtArray = pLink;

		pNewTriangle++;

		pLink->vp2DRegion = pTriangle;

		pLinkNext = (RVLMESH_LINK *)((NewTriangleArray + (iLink + 1) % nNewTriangles)->m_PtArray);

		pLinkNew1 = pNewLink;

		pNewLink++;

		pLinkNew1->Flags = 0x00;
		pLinkNew1->iPix0 = pLink->iPix0;
		pLinkNew1->pPrev = pLink;
		pLinkNew1->pNext = pLinkNext->pOpposite;
		pLink->pNext = pLinkNew1;
		pLinkNext->pOpposite->pPrev = pLinkNew1;

		pLinkNew2 = pNewLink;

		pNewLink++;

		pLinkNew1->pOpposite = pLinkNew2;

		pLinkNew2->Flags = 0x00;
		pLinkNew2->iPix0 = iPixNew;
		pLinkNew2->pOpposite = pLinkNew1;
		pLinkNew2->vp2DRegion = pTriangle;
	}

	delete[] NewTriangleArray;	

	// complete link data

	pLinkPrev = pFirstBoundaryLink->pOpposite->pPrev->pOpposite;

	pLink = pFirstBoundaryLink;

	do
	{
		pLinkNew1 = pLink->pNext;

		pLinkNew2 = pLinkNew1->pOpposite;

		pLinkNew2->pNext = pLinkPrev;

		pLinkPrev->pPrev = pLinkNew2;

		pLinkPrev = pLinkNew2;

		p3DPt0 = Point3DMap[pLink->iPix0];

		u = p3DPt0->u;
		v = p3DPt0->v;

		du = uNew - u;
		dv = vNew - v;

		pLinkNew1->du = du;
		pLinkNew1->dv = dv;
		pLinkNew2->du = -du;
		pLinkNew2->dv = -dv;

		pLink = pLinkNew1->pNext->pOpposite;

		pLinkNew1->vp2DRegion = pLink->vp2DRegion;
	}
	while(pLink != pFirstBoundaryLink);

	return TRUE;
}

void RVLInitConvexHull(	CRVL2DRegion2 *pTriangleSrc,
						CRVLClass *pTriangleSet,
						int ImageWidth,
						RVL3DPOINT2 **Point3DMap,
						CRVLMem *pMem,
						bool bmm)
{
	CRVL2DRegion2 *pTriangle = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(pTriangleSet));
	CRVL2DRegion2 *pTriangle2 = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(pTriangleSet));

	pTriangle->m_n3DPts = pTriangle2->m_n3DPts = 0;

	RVLMESH_LINK *pLink, *pLink0, *pLinkSrc, *pLink2;

	pLinkSrc = pLink0 = (RVLMESH_LINK *)(pTriangleSrc->m_PtArray);

	RVLMESH_LINK *LinkArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RVLMESH_LINK, 6, LinkArray);

	pTriangle->m_PtArray = LinkArray;
	pTriangle2->m_PtArray = LinkArray + 3;

	int iPt = 0;
	
	do
	{
		pLink = LinkArray + iPt;		

		pLink->Flags = 0x00;
		pLink->iPix0 = pLinkSrc->iPix0;
		pLink->du = pLinkSrc->du;
		pLink->dv = pLinkSrc->dv;

		pLink2 = LinkArray + 5 - iPt;

		pLink->pNext = pLink->pPrev = pLink2;
		pLink2->pNext = pLink2->pPrev = pLink;

		pLink2 = LinkArray + 3 + (3 - iPt) % 3;

		pLink->pOpposite = pLink2;
		pLink2->pOpposite = pLink;

		pLink->vp2DRegion = pTriangle;

		iPt++;

		pLinkSrc = pLinkSrc->pNext->pOpposite;
	}
	while(pLinkSrc != pLink0);

	pLinkSrc = pLink0;

	do
	{
		pLink = LinkArray + iPt;

		pLink->Flags = 0x00;
		pLink->iPix0 = pLinkSrc->pOpposite->iPix0;
		pLink->du = -pLinkSrc->du;
		pLink->dv = -pLinkSrc->dv;
		pLink->vp2DRegion = pTriangle2;

		iPt++;

		pLinkSrc = pLinkSrc->pOpposite->pPrev;
	}
	while(pLinkSrc != pLink0);

	RVL3DPOINT2 *p3DPt0, *p3DPt1, *p3DPt2;
	//double dX1[3], dX2[3];
	int dX1[3], dX2[3];

	if(bmm)
	{		
		//double *fN = pTriangle->m_fN;

		//pLink = LinkArray;
		//pLink2 = pLink->pNext;

		//p3DPt0 = Point3DMap[pLink->iPix0];

		//p3DPt1 = Point3DMap[pLink2->pOpposite->iPix0];

		//p3DPt2 = Point3DMap[pLink->pOpposite->iPix0];

		//double *X0 = p3DPt0->XYZ;
		//double *X1 = p3DPt1->XYZ;
		//double *X2 = p3DPt2->XYZ;

		//RVLDIF3VECTORS(X1, X0, dX1)
		//RVLDIF3VECTORS(X2, X0, dX2)

		//RVLCROSSPRODUCT3(dX1, dX2, fN)

		//double fTmp;

		//RVLNORM3(fN, fTmp)

		//pTriangle->m_rho = RVLDOTPRODUCT3(X0, fN);

		//double *fN2 = pTriangle2->m_fN;

		//fN2[0] = -fN[0];
		//fN2[1] = -fN[1];
		//fN2[2] = -fN[2];

		//pTriangle2->m_rho = -pTriangle->m_rho;

		//if(pTriangle2->m_rho < 0.0)
		//	pTriangle = pTriangle2;

		int *N = pTriangle->m_N;

		pLink = LinkArray;
		pLink2 = pLink->pNext;

		p3DPt0 = Point3DMap[pLink->iPix0];

		p3DPt1 = Point3DMap[pLink2->pOpposite->iPix0];

		p3DPt2 = Point3DMap[pLink->pOpposite->iPix0];

		int *X0 = p3DPt0->iX;
		int *X1 = p3DPt1->iX;
		int *X2 = p3DPt2->iX;

		RVLDIF3VECTORS(X1, X0, dX1)
		RVLDIF3VECTORS(X2, X0, dX2)

		RVLCROSSPRODUCT3(dX1, dX2, N)

		pTriangle->m_lenN = DOUBLE2INT(sqrt((double)(N[0]) * (double)(N[0]) + (double)(N[1]) * (double)(N[1]) + (double)(N[2]) * (double)(N[2])));

		int *X0_ = pTriangle->m_X0;

		RVLCOPY3VECTOR(X0, X0_)

		int *N2 = pTriangle2->m_N;

		N2[0] = -N[0];
		N2[1] = -N[1];
		N2[2] = -N[2];

		int *X0_2 = pTriangle2->m_X0;

		pTriangle2->m_lenN = pTriangle->m_lenN;

		RVLCOPY3VECTOR(X0_, X0_2)

		if(RVLDOTPRODUCT3_64(N, X0) < 0)
		{
			pTriangle->m_Flags = 0x00000000;

			pTriangle2->m_Flags = RVL2DREGION_FLAG_INVISIBLE;
		}
		else
		{
			pTriangle2->m_Flags = 0x00000000;

			pTriangle->m_Flags = RVL2DREGION_FLAG_INVISIBLE;

			pTriangle = pTriangle2;
		}
	}
	else
	{
		int dd1, dd2;

		int *N = pTriangle->m_N;

		pLink = LinkArray;
		pLink2 = pLink->pNext;

		p3DPt0 = Point3DMap[pLink->iPix0];

		p3DPt1 = Point3DMap[pLink2->pOpposite->iPix0];

		p3DPt2 = Point3DMap[pLink->pOpposite->iPix0];

		dd1 = p3DPt1->d - p3DPt0->d;
		dd2 = p3DPt2->d - p3DPt0->d;

		N[0] = pLink2->dv * dd2 - dd1 * pLink->dv;
		N[1] = dd1 * pLink->du - pLink2->du * dd2;
		N[2] = -pLink2->dv * pLink->du + pLink2->du * pLink->dv;

		pTriangle->m_d = N[0] * p3DPt0->u + N[1] * p3DPt0->v + N[2] * p3DPt0->d;

		double fN[3];

		fN[0] = (double)N[0];
		fN[1] = (double)N[1];
		fN[2] = (double)N[2];

		pTriangle->m_lenN = DOUBLE2INT(sqrt(fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2]));

		int *N2 = pTriangle2->m_N;

		N2[0] = -N[0];
		N2[1] = -N[1];
		N2[2] = -N[2];

		pTriangle2->m_d = -pTriangle->m_d;

		pTriangle2->m_lenN = pTriangle->m_lenN;

		if(N2[2] < 0)
			pTriangle = pTriangle2;
	}

	pTriangle->m_n3DPts = 3;

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RVL3DPOINT2 *, 3, pTriangle->m_pPoint3DArray);

	pTriangle->m_pPoint3DArray[0] = (RVL3DPOINT2 *)pLinkSrc;
	pTriangle->m_pPoint3DArray[1] = (RVL3DPOINT2 *)(pLinkSrc->pNext->pOpposite);
	pTriangle->m_pPoint3DArray[2] = (RVL3DPOINT2 *)(pLinkSrc->pOpposite->pPrev);

	//if((int)pLinkSrc == 0x023f913c || (int)(pLinkSrc->pOpposite) == 0x023f913c || (int)(pLinkSrc->pOpposite->pPrev) == 0x023f913c)
	//	int debug = 0;
}

BOOL RVLIsVertex(	int iPix,
					CRVLMPtrChain *pTriangleList)
{
	CRVL2DRegion2 *pTriangle;
	RVLMESH_LINK *pLink0, *pLink;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		do
		{
			if(pLink->iPix0 == iPix)
				return TRUE;

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);
	}

	return FALSE;
}

int RVLGetConvexHull(	CRVL2DRegion2 *pTriangleSrc,
						DWORD Label,
						CRVLMPtrChain *pTriangleList,
						CRVLClass *pTriangleSet,
						int maxDist,
						int ImageWidth,
						RVL3DPOINT2 **Point3DMap,
						CRVLMem *pMem,
						RVLQLIST_PTR_ENTRY *LinkQueueEntryMem,
						CRVL3DMeshObject *childMO,
						bool bmm)
						//CRVLPlanarSurfaceDetector *pPSD)
{
	// Form a initial (flat) convex hull consisting of two triangles: pTriangleSrc and the opposite side of the same triangle

	RVLInitConvexHull(pTriangleSrc, pTriangleSet, ImageWidth, Point3DMap, pMem, bmm);

	//if(!RVL3DMeshIsConvex(&(pTriangleSet->m_ObjectList), Point3DMap))
	//	int debug = 0;

	// Form LinkQueue containing all links of pTriangleSrc
	
	RVLMESH_LINK *pLink = (RVLMESH_LINK *)(pTriangleSrc->m_PtArray);

	RVLQLIST LinkQueue;

	RVLQLIST *pLinkQueue = &LinkQueue;

	RVLQLIST_INIT(pLinkQueue);

	RVLQLIST_PTR_ENTRY *pLinkPtr = LinkQueueEntryMem;

	pTriangleSrc->m_Label = Label;

	int Size = pTriangleSrc->m_Size;

	RVLMESH_LINK *pLink0 = pLink;

	do
	{
		RVLQLIST_ADD_ENTRY(pLinkQueue, pLinkPtr);

		pLinkPtr->Ptr = pLink;

		pLinkPtr++;

		pLink = pLink->pNext->pOpposite;
	}
	while(pLink != pLink0);

	//int debugConunter = 0;

	// Determine the new approximatelly convex set (ACS) by applying an incremental region growing procedure starting from pTriangleSrc.

	CRVL2DRegion2 *pTriangle, *pTriangle2, *pTriangle3;
	int *N, *N2;
	double cosAngle, maxCosAngle;
	void **ppLinkPtr, **ppBestLinkPtr;
	//double fN[3];
	RVLQLIST_PTR_ENTRY *pLinkPtr2;
	BOOL bConvex;
	double *fN, *fN2;

	while(TRUE)
	{
		// Among all links in LinkQueue which are on the boundary of the ACS 
		// find the one which connects two triangles with the smallest angle between their normals.
		// This link is referred to in the following as expansion link.
		// The triangle outside the ACS connected to the ACS by the expansion link
		// is the candidate for growing of the ACS.
		// This triangle is referred to in the following as an expansion candidate.
		// The ptr. to the ptr. to the expansion link is stored in ppBestLinkPtr.

		maxCosAngle = -2.0;

		ppLinkPtr = &(LinkQueue.pFirst);

		pLinkPtr2 = (RVLQLIST_PTR_ENTRY *)(LinkQueue.pFirst);

		ppBestLinkPtr = NULL;
	
		while(pLinkPtr2)
		{
			pLink = (RVLMESH_LINK *)(pLinkPtr2->Ptr);

			pTriangle = (CRVL2DRegion2 *)(pLink->vp2DRegion);

			pTriangle2 = (CRVL2DRegion2 *)(pLink->pOpposite->vp2DRegion);

			if(bmm)
			{
				fN = pTriangle->m_fN;
				
				if(pTriangle2)
					if(pTriangle2->m_Label == 0xffffffff)
					{
						fN2 = pTriangle2->m_fN;

						cosAngle = RVLDOTPRODUCT3(fN, fN2);

						if(cosAngle > maxCosAngle)
						{
							maxCosAngle = cosAngle;

							ppBestLinkPtr = ppLinkPtr;
						}
					}

				//N = pTriangle->m_N;
				//
				//if(pTriangle2)
				//	if(pTriangle2->m_Label == 0xffffffff)
				//	{
				//		N2 = pTriangle2->m_N;

				//		cosAngle = (double)(RVLDOTPRODUCT3_64(N, N2)) / ((double)(pTriangle->m_lenN) * (double)(pTriangle2->m_lenN));

				//		if(cosAngle > maxCosAngle)
				//		{
				//			maxCosAngle = cosAngle;

				//			ppBestLinkPtr = ppLinkPtr;
				//		}
				//	}
			}
			else
			{
				N = pTriangle->m_N;

				//if(pTriangle->m_lenN == 0)
				//{
				//	fN[0] = (double)N[0];
				//	fN[1] = (double)N[1];
				//	fN[2] = (double)N[2];

				//	pTriangle->m_lenN = DOUBLE2INT(sqrt(fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2]));					
				//}

				if(pTriangle2)
					if(pTriangle2->m_Label == 0xffffffff)
					{
						N2 = pTriangle2->m_N;

						//if(pTriangle2->m_lenN == 0)
						//{
						//	fN[0] = (double)N2[0];
						//	fN[1] = (double)N2[1];
						//	fN[2] = (double)N2[2];

						//	pTriangle2->m_lenN = DOUBLE2INT(sqrt(fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2]));					
						//}

						cosAngle = ((double)(N[0]) * (double)(N2[0]) + (double)(N[1]) * (double)(N2[1]) + (double)(N[2]) * (double)(N2[2])) /
							((double)(pTriangle->m_lenN) * (double)(pTriangle2->m_lenN));

						if(cosAngle > maxCosAngle)
						{
							maxCosAngle = cosAngle;

							ppBestLinkPtr = ppLinkPtr;
						}
					}
			}

			ppLinkPtr = &(pLinkPtr2->pNext);

			pLinkPtr2 = (RVLQLIST_PTR_ENTRY *)(pLinkPtr2->pNext);
		}		

		if(ppBestLinkPtr == NULL)
			break;

		// pLinkPtr2 <- ptr. to the expansion link

		pLinkPtr2 = (RVLQLIST_PTR_ENTRY *)(*ppBestLinkPtr);

		// remove the expansion link from LinkQueue

		if(pLinkPtr2->pNext == NULL)
			LinkQueue.ppNext = ppBestLinkPtr;

		*ppBestLinkPtr = pLinkPtr2->pNext;

		// pLink <- expansion link

		pLink = (RVLMESH_LINK *)(pLinkPtr2->Ptr);

		// pTriangle <- expansion candidate

		pTriangle = (CRVL2DRegion2 *)(pLink->pPrev->vp2DRegion);

		// If two or all three triangles adjacent to the expansion candidate are already in the ACS,
		// then this expansion candidate is automatically appended to the ACS without updating its convex hull.
		// Otherwise, RVLUpdateConvexHull() function is applied to determine whether the union of the ACS 
		// and the expansion candidate is an approximatelly convex set or not. 
		// If this is the case, then the expansion candidate is appended to the ACS and its convex hull is updated.

		bConvex = FALSE;

		if(pTriangle)
		{
			//if((pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
			{
				pTriangle2 = (CRVL2DRegion2 *)(pLink->pPrev->pOpposite->vp2DRegion);

				if(pTriangle2 != NULL)
					if((pTriangle2->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
						if(pTriangle2->m_Label == Label)
							bConvex = TRUE;

				pTriangle3 = (CRVL2DRegion2 *)(pLink->pOpposite->pNext->vp2DRegion);

				if(pTriangle3 != NULL)
					if((pTriangle3->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
						if(pTriangle3->m_Label == Label)
							bConvex = TRUE;				

				//if(!bConvex)
				if(bConvex)
					pTriangle->m_Flags &= ~RVLOBJ2_FLAG_REJECTED;
				else if((pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
					bConvex = RVLUpdateConvexHull(pTriangleList, pTriangleSet, ImageWidth, Point3DMap, 
						pLink->pPrev->pOpposite->pPrev, maxDist, pMem, bmm);

				// only for debugging purpose !!!

				//if(!RVL3DMeshIsConvex(pTriangleList, Point3DMap))
				//	int debug = 0;

				//if(pTriangleList->m_nElements == 137)
				//	int debug = 0;

				/////
			}
		}

		// If the expansion candidate is appended to the ACS, then it is assigned the label of the ACS 
		// and its links which are at the boundary of the new ACS are inserted into LinkQueue.

		if(bConvex)
		{
			//If childMO is assigned we add new triangle to it
			if(childMO)
			{
				RVLQLIST_PTR_ENTRY *pElement = new RVLQLIST_PTR_ENTRY[1];
				pElement->Ptr = pTriangle;
				RVLQLIST_ADD_ENTRY(childMO->m_FaceList, pElement);
				childMO->m_noFaces++;
				childMO->m_noVertices = childMO->m_noFaces + 2;
			}

			pTriangle->m_Label = Label;

			Size += pTriangleSrc->m_Size;

			if(pTriangle2)
				if(pTriangle2->m_Label == 0xffffffff)
				{
					RVLQLIST_ADD_ENTRY(pLinkQueue, pLinkPtr);

					pLinkPtr->Ptr = pLink->pPrev;

					pLinkPtr++;
				}

			if(pTriangle3)
				if(pTriangle3->m_Label == 0xffffffff)
				{
					RVLQLIST_ADD_ENTRY(pLinkQueue, pLinkPtr);

					pLinkPtr->Ptr = pLink->pOpposite->pNext->pOpposite;

					pLinkPtr++;
				}
		}

		//debugConunter++;

		//if((debugConunter % 10) == 0)
		//	int tmp1 = 0;
	}

	//FILE *fpPts, *fpIdx;

	//fopen_s(&fpPts, "C:\\RVL\\ExpRez\\mesh2Pts.dat", "w");

	//fopen_s(&fpIdx, "C:\\RVL\\ExpRez\\mesh2Idx.dat", "w");

	//pPSD->SaveMesh(fpPts, fpIdx, pTriangleSet);

	//fclose(fpPts);

	//fclose(fpIdx);

	// assign convex hull triangles to pts. (currently only for display purposes)

	RVLMESH_LINK **ppLink, **ppLinkArrayEnd;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		ppLink = (RVLMESH_LINK **)(pTriangle->m_pPoint3DArray);

		ppLinkArrayEnd = ppLink + pTriangle->m_n3DPts;

		for(;ppLink < ppLinkArrayEnd; ppLink++)
		{
			pLink = *ppLink;

			//if((int)(*ppLink) == 0x02369fc4)
			//	int debug = 0;

			pLink0 = pLink;

			do
			{
				pTriangle2 = (CRVL2DRegion2 *)(pLink->vp2DRegion);

				if(pTriangle2)
					if(pTriangle2->m_Label == Label)
						pLink->pData = pTriangle;

				pLink = pLink->pNext;
			}
			while(pLink != pLink0);
		}
	}

	return Size;
}	

int RVLSegmentToConvex( CRVLClass *pTriangleSetSrc,
						CRVL2DRegion2 *pSelectedTriangle,
						CRVLClass *pTriangleSetTgt,
						int maxDist,
						int ImageWidth,
						int ImageHeight,
						RVL3DPOINT2 **Point3DMap,
						CRVLMem *pMem,
						//CRVLPlanarSurfaceDetector *pPSD,
						int *SizeArray,
						CRVL3DMeshObject *rootMO,
						bool bmm)
{
	//Stvaraju se nove prazne datoteke u koje ce su upisivati podaci o generiranom convex hull-u
	/*FILE *fpPts, *fpIdx, *fpSizes, *fpCurvs, *fpDirs;
	fopen_s(&fpPts, "C:\\RVL\\ExpRez\\mesh2PtsCH.dat", "w");
	fopen_s(&fpIdx, "C:\\RVL\\ExpRez\\mesh2IdxCH.dat", "w");
	fopen_s(&fpSizes, "C:\\RVL\\ExpRez\\mesh2SizesCH.dat", "w");	
	fclose(fpPts);
	fclose(fpIdx);
	fclose(fpSizes);
	//za ispis principijalinh zakrivljenosti
	fopen_s(&fpCurvs, "C:\\RVL\\ExpRez\\mesh2CurvsCH.dat", "w");
	fopen_s(&fpDirs, "C:\\RVL\\ExpRez\\mesh2DirsCH.dat", "w");
	fclose(fpCurvs);
	fclose(fpDirs);*/

	CRVLMPtrChain *pTriangleList = &(pTriangleSetSrc->m_ObjectList);

	int maxSize = 0;

	CRVL2DRegion2 *pTriangle;
	RVLMESH_LINK *pLink;
	RVL3DPOINT2 *p3DPt0, *p3DPt1, *p3DPt2;
	int du2, dv2, du1, dv1;
	int size;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		pTriangle->m_Label = 0xffffffff;

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pLink = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		p3DPt0 = Point3DMap[pLink->iPix0];

		p3DPt1 = Point3DMap[pLink->pNext->pOpposite->iPix0];

		p3DPt2 = Point3DMap[pLink->pOpposite->iPix0];

		du1 = p3DPt1->u - p3DPt0->u;
		du2 = p3DPt2->u - p3DPt0->u;

		dv1 = p3DPt1->v - p3DPt0->v;
		dv2 = p3DPt2->v - p3DPt0->v;

		size = -dv1 * du2 + du1 * dv2;

		if(size < 0)
			size = -size;

		pTriangle->m_Size = size;

		if(size > maxSize) 
			maxSize = size;		
	}	

	RVLQLIST_PTR_ENTRY *LinkQueueEntryMem = new RVLQLIST_PTR_ENTRY[6 * ImageWidth * ImageHeight];

	CRVLQListArray TriangleQueue;

	TriangleQueue.m_Size = maxSize + 1;

	TriangleQueue.m_ListArray = new RVLQLIST[TriangleQueue.m_Size];

	TriangleQueue.InitListArray(TriangleQueue.m_ListArray, TriangleQueue.m_ListArray);

	RVLQLIST *TriangleQueueListArray = TriangleQueue.m_ListArray;

	RVLQLIST_PTR_ENTRY *TriangleQueueEntryMem = new RVLQLIST_PTR_ENTRY[pTriangleList->m_nElements];

	RVLQLIST_PTR_ENTRY *pTriangleQueueEntry = TriangleQueueEntryMem;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		//if(((RVLMESH_LINK *)(pTriangle->m_PtArray))->iPix0 == 114 + 135*320 ||
		//	((RVLMESH_LINK *)(pTriangle->m_PtArray))->pNext->pOpposite->iPix0 == 114 + 135*320 ||
		//	((RVLMESH_LINK *)(pTriangle->m_PtArray))->pNext->pOpposite->pNext->pOpposite->iPix0 == 114 + 135*320)
		//	int debug = 0;

		RVLQLISTARRAY_ADD_ENTRY(TriangleQueueListArray, pTriangle->m_Size, pTriangleQueueEntry);

		pTriangleQueueEntry->Ptr = pTriangle;

		pTriangleQueueEntry++;
	}	

	DWORD Label = 0;

	int iTriangleQueueBin;
	
	//Index map koja je potrebana kod zapisivanja mesha convex hulla
	//int *IdxMap = new int[ImageWidth * ImageHeight];
	//memset(IdxMap, 0, ImageWidth * ImageHeight * sizeof(int));

	for(iTriangleQueueBin = maxSize; iTriangleQueueBin >= 0; iTriangleQueueBin--)
	{
		pTriangleQueueEntry = (RVLQLIST_PTR_ENTRY *)(TriangleQueueListArray[iTriangleQueueBin].pFirst);

		//if(iTriangleQueueBin == 179)
		//	int debug = 0;

		while(pTriangleQueueEntry)
		{
			pTriangle = (CRVL2DRegion2 *)(pTriangleQueueEntry->Ptr);

			if(pTriangle->m_Label == 0xffffffff)
			{
				//if(((RVLMESH_LINK *)(pTriangle->m_PtArray))->iPix0 == 114 + 135*320 ||
				//	((RVLMESH_LINK *)(pTriangle->m_PtArray))->pNext->pOpposite->iPix0 == 114 + 135*320 ||
				//	((RVLMESH_LINK *)(pTriangle->m_PtArray))->pNext->pOpposite->pNext->pOpposite->iPix0 == 114 + 135*320)
				//	int debug = 0;

				//if(Label == 91)
				//	int debug = 0;
				
				pTriangleSetTgt->Clear();
				
				if (rootMO)
				{
					//A child Mesh3DObject is created, its properties set and linked to parent 3DMeshObject
					CRVL3DMeshObject *childMO = new CRVL3DMeshObject();
					childMO->m_pClass = rootMO->m_pClass;
					childMO->InitChild();
					childMO->parentMeshObject = rootMO;
					childMO->rootMeshObject = rootMO;
					
					RVLQLIST_PTR_ENTRY *pElement = new RVLQLIST_PTR_ENTRY[1];
					pElement->Ptr = childMO;
					RVLQLIST_ADD_ENTRY(rootMO->m_ChildMeshObjects, pElement);
					//Add first element
					pElement = new RVLQLIST_PTR_ENTRY[1];
					pElement->Ptr = pTriangle;
					RVLQLIST_ADD_ENTRY(childMO->m_FaceList, pElement);
					childMO->m_noFaces++;
					childMO->m_noVertices = childMO->m_noFaces + 2;

					SizeArray[Label] = RVLGetConvexHull(pTriangle, Label, &(pTriangleSetTgt->m_ObjectList), pTriangleSetTgt, 
						maxDist, ImageWidth, Point3DMap, pMem, LinkQueueEntryMem, childMO, bmm);

					//We update rootMO values
					rootMO->m_noFaces += childMO->m_noFaces;
					rootMO->m_noVertices += childMO->m_noVertices;
				}
				else if(SizeArray)
					SizeArray[Label] = RVLGetConvexHull(pTriangle, Label, &(pTriangleSetTgt->m_ObjectList), pTriangleSetTgt, 
						maxDist, ImageWidth, Point3DMap, pMem, LinkQueueEntryMem, NULL, bmm);
				else
					RVLGetConvexHull(pTriangle, Label, &(pTriangleSetTgt->m_ObjectList), pTriangleSetTgt, 
						maxDist, ImageWidth, Point3DMap, pMem, LinkQueueEntryMem, NULL, bmm);			

				//SaveMeshCH(pTriangleSetTgt, Point3DMap, IdxMap, ImageWidth * ImageHeight);
				Label++;

				//if(Label == 90)
				//{
				//	int debug = 0;

				//	break;
				//}
			}

			pTriangleQueueEntry = (RVLQLIST_PTR_ENTRY *)(pTriangleQueueEntry->pNext);
		}

		//if(Label == 90)
		//{
		//	int debug = 0;

		//	break;
		//}
	}

	//delete[] IdxMap;
	//FILE *fpPts, *fpIdx;

	//fopen_s(&fpPts, "C:\\RVL\\ExpRez\\mesh2Pts.dat", "w");

	//fopen_s(&fpIdx, "C:\\RVL\\ExpRez\\mesh2Idx.dat", "w");

	//pPSD->SaveMesh(fpPts, fpIdx, pTriangleSetSrc);

	//fclose(fpPts);

	//fclose(fpIdx);

	delete[] LinkQueueEntryMem;

	delete[] TriangleQueueEntryMem;

	return (int)Label;
}

// some details about the algorithm implemented by function RVLSegmentToConvex
// are given in SEG12

void RVLSegmentToConvex(CRVLDelaunay *pDelaunay,
						double maxLineSegmentErr,
						CRVLQListArray *pDelaunayLinkQueue,
						CRVLMPtrChain *pTriangleList,
						DWORD Flags)
{
	RVLMESH_LINK *pDelaunayLink;
	int maxlen2 = 0;
	int len2;
	RVLQLIST *pList;

	pDelaunayLinkQueue->Reset();

	RVLQLIST *ListArray = pDelaunayLinkQueue->m_ListArray;

	RVLQLIST_PTR_ENTRY *LinkPtrMem;
	RVLQLIST_PTR_ENTRY *pLinkPtr;

	if(pTriangleList)
	{
		LinkPtrMem = new RVLQLIST_PTR_ENTRY[3 * pTriangleList->m_nElements];

		pLinkPtr = LinkPtrMem;

		CRVL2DRegion2 *pTriangle;
		RVLMESH_LINK *pDelaunayLink0, *pOppositeDelaunayLink;

		pTriangleList->Start();

		while(pTriangleList->m_pNext)
		{
			pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

			if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			pDelaunayLink0 = pDelaunayLink = (RVLMESH_LINK *)(pTriangle->m_PtArray);

			do
			{
				if(pDelaunayLink->Flags & RVLMESH_LINK_FLAG_EDGE)
				{
					pDelaunayLink = pDelaunayLink->pNext->pOpposite;

					continue;
				}

				if(Flags & RVLSEGMENT_TO_CONVEX_FLAG_ROI)
					if((pDelaunayLink->Flags & RVLMESH_LINK_FLAG_ROI) == 0)
						continue;

				pOppositeDelaunayLink = pDelaunayLink->pOpposite;

				if(pDelaunayLink->iPix0 > pOppositeDelaunayLink->iPix0)
				{
					pDelaunayLink = pDelaunayLink->pNext->pOpposite;

					continue;
				}

				pDelaunayLink->Flags |= RVLMESH_LINK_FLAG_EDGE;

				pOppositeDelaunayLink->Flags |= RVLMESH_LINK_FLAG_EDGE;

				len2 = pDelaunayLink->du * pDelaunayLink->du + pDelaunayLink->dv * pDelaunayLink->dv;

				RVLQLISTARRAY_ADD_ENTRY(ListArray, len2, pLinkPtr);

				pLinkPtr->Ptr = pDelaunayLink;

				pLinkPtr++;

				if(len2 > maxlen2)
					maxlen2 = len2;

				pDelaunayLink = pDelaunayLink->pNext->pOpposite;
			}
			while(pDelaunayLink != pDelaunayLink0);
		}		
	}
	else
	{
		LinkPtrMem = new RVLQLIST_PTR_ENTRY[pDelaunay->m_nLinks];

		pLinkPtr = LinkPtrMem;

		int nDelaunayLinks;
		RVLMESH_LINK *DelaunayLink;
		RVLMESH_LINK *pDelaunayLinkArrayEnd;
		BYTE *pDelaunayData;

		//FILE *fp;
		//
		//fopen_s(&fp, "C:\\RVL\\Debug\\segmentationEB.log", "w");

		//int iTmp = 0;

		// sort non-edge Delaunay links according to their length in descending order

		pDelaunayData = pDelaunay->m_DelaunayData;

		int iPix, iPix0;

		while(TRUE)
		{
			nDelaunayLinks = *((short *)pDelaunayData);

			if(nDelaunayLinks <= 0)
				break;

			//if(iTmp >= m_Width * m_Height)
			//	break;

			//iTmp++;

			pDelaunayData += sizeof(short);

			DelaunayLink = (RVLMESH_LINK *)pDelaunayData;

			iPix0 = DelaunayLink->iPix0;

			//fprintf(fp, "%d\t%d\n", iPix0, nDelaunayLinks);

			//fflush(fp);

			pDelaunayLinkArrayEnd = DelaunayLink + nDelaunayLinks;

			for(pDelaunayLink = DelaunayLink; pDelaunayLink < pDelaunayLinkArrayEnd; pDelaunayLink++)
			{
				if(pDelaunayLink->Flags & RVLMESH_LINK_FLAG_EDGE)
					continue;

				iPix = pDelaunayLink->pOpposite->iPix0;

				if(iPix < iPix0)
					continue;

				pDelaunayLink->Flags |= RVLMESH_LINK_FLAG_EDGE;

				pDelaunayLink->pOpposite->Flags |= RVLMESH_LINK_FLAG_EDGE;

				len2 = pDelaunayLink->du * pDelaunayLink->du + pDelaunayLink->dv * pDelaunayLink->dv;

				RVLQLISTARRAY_ADD_ENTRY(ListArray, len2, pLinkPtr);

				//if(len2 >= m_DelaunayLinkQueue.m_Size)
				//	int tmp1 = 0;

				pLinkPtr->Ptr = pDelaunayLink;

				pLinkPtr++;

				if(len2 > maxlen2)
					maxlen2 = len2;
			}

			pDelaunayData += (nDelaunayLinks * sizeof(RVLMESH_LINK));
		}
	}

	//fclose(fp);

	// Remove every non-edge delaunay link whose removal does not produce non-convex regions
	// starting from the longest link. The remaining links are denoted as new edge links.

	RVLMESH_LINK *pA, *pB, *pC, *pD, *pE;
	RVLMESH_LINK *pNextDelaunayLink;
	int duAB, dvAB, duBC, dvBC, duCD, dvCD, duBE, dvBE, duBF, dvBF;
	int duA0, dvA0, du0D, dv0D, du0F, dv0F;
	BOOL bMoveA, bMoveD;
	BOOL bAB;	// |AB| > 0
	BOOL bBC;	// |BC| > 0
	BOOL bCD;	// |CD| > 0
	int eThr;
	BOOL bConvex;
	int iSide;
	RVLMESH_LINK *pB2[2], *pC2[2];
	//BOOL bConcavity;

	for(pList = ListArray + maxlen2; pList >= ListArray; pList--)
	{
		pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pList->pFirst);

		while(pLinkPtr)
		{
			pDelaunayLink = (RVLMESH_LINK *)(pLinkPtr->Ptr);

			//if(pDelaunayLink->iPix0 == 23463 && pDelaunayLink->du == -75 && pDelaunayLink->dv == 17)
			//	int debug = 0;
			//if(pDelaunayLink->iPix0 == 27504 || pDelaunayLink->pOpposite->iPix0 == 27504)
			//{
			//	FILE *fp;

			//	fp = fopen("C:\\RVL\\ExpRez\\delaunay_edges.dat", "w");

			//	CRVLFigure Fig;

			//	Fig.Create(1000000);

			//	RVLDisplay2DRegions(&Fig, pTriangleList, pDelaunay->m_Width, RVLColor(0, 0, 0), 1,
			//		RVLMESH_LINK_FLAG_EDGE, RVLMESH_LINK_FLAG_EDGE);

			//	RVLSaveDisplayVectors(&Fig, fp);

			//	fclose(fp);

			//	Fig.Clear();

			//	int debug = 0;
			//}

			// concavity test

			bConvex = TRUE;

			for(iSide = 0; iSide < 2; iSide++)
			{
				//du0F = pDelaunayLink->du;
				//dv0F = pDelaunayLink->dv;

				bAB = bBC = bCD = FALSE;

				pNextDelaunayLink = pDelaunayLink->pPrev;

				while((pNextDelaunayLink->Flags & RVLMESH_LINK_FLAG_EDGE) == 0)
					pNextDelaunayLink = pNextDelaunayLink->pPrev;

				pA = pB = pC = pD = pNextDelaunayLink;

				pDelaunayLink->Flags &= ~RVLMESH_LINK_FLAG_EDGE;

				//duA0 = dvA0 = du0D = dv0D = 0;
				
				duBC = dvBC = 0;

				while(bConvex)
				{
					if(bBC)
					{
						if(bMoveD = (-dvBC * duCD + duBC * dvCD > 0))
						{
							pC = pD;

							duBC += duCD;
							dvBC += dvCD;
						}
						else if(bMoveA = (-dvAB * duBC + duAB * dvBC > 0))
						{
							pB = pA;

							duBC += duAB;
							dvBC += dvAB;
						}
					}
					else
					{
						if(bCD)
						{
							if(bAB)
							{
								if(-dvAB * duCD + duAB * dvCD > 0)
								{
									pC = pD;

									duBC += duCD;
									dvBC += dvCD;

									bMoveD = TRUE;

									bBC = TRUE;
								}
								//else
								//{	
								//	duBF = pDelaunayLink->du;
								//	dvBF = pDelaunayLink->dv;

								//	if(-dvAB * duBF + duAB * dvBF >= 0)
								//		bConvex = FALSE;
								//	else if(dvBF * duCD - duBF * dvCD >= 0)
								//		bConvex = FALSE;
								//}

								bMoveA = FALSE;
							}
							else
							{
								bMoveD = FALSE;

								bMoveA = TRUE;

								bAB = TRUE;
							}
						}
						else
						{
							bMoveD = TRUE;

							bCD = TRUE;
						}
					}

					if(bMoveD)
					{
						duCD = dvCD = 0;

						do
						{
							pNextDelaunayLink = pD->pNext;

							while((pNextDelaunayLink->Flags & RVLMESH_LINK_FLAG_EDGE) == 0)
								pNextDelaunayLink = pNextDelaunayLink->pNext;

							duCD += pNextDelaunayLink->du;
							dvCD += pNextDelaunayLink->dv;

							//du0D += pNextDelaunayLink->du;
							//dv0D += pNextDelaunayLink->dv;

							//if(-dv0F * du0D + du0F * dv0D <= 0)
							//{
							//	if(pNextDelaunayLink->pOpposite->iPix0 != pDelaunayLink->pOpposite->iPix0)
							//	{
							//		bConvex = FALSE;

							//		break;
							//	}
							//}

							//bConcavity = (-pD->dv * pNextDelaunayLink->du + pD->du * pNextDelaunayLink->dv < 0);

							pD = pNextDelaunayLink->pOpposite;

							if(pD == pA)
							{
								bConvex = FALSE;

								break;
							}
						}
						while(pD->Flags & RVLMESH_LINK_CONCAVITY);	
						//while(bConcavity);
					}
					else if(bMoveA)
					{
						duAB = dvAB = 0;

						do
						{
							pNextDelaunayLink = pA->pOpposite;

							duAB += pNextDelaunayLink->du;
							dvAB += pNextDelaunayLink->dv;

							//duA0 += pNextDelaunayLink->du;
							//dvA0 += pNextDelaunayLink->dv;

							//if(-dv0F * duA0 + du0F * dvA0 <= 0)
							//{
							//	if(pNextDelaunayLink->iPix0 != pDelaunayLink->pOpposite->iPix0)
							//	{
							//		bConvex = FALSE;

							//		break;
							//	}
							//}

							pA = pNextDelaunayLink;

							do
								pA = pA->pPrev;
							while((pA->Flags & RVLMESH_LINK_FLAG_EDGE) == 0);

							if(pD == pA)
							{
								bConvex = FALSE;

								break;
							}
						}
						while(pA->Flags & RVLMESH_LINK_CONCAVITY);	
						//while(-pA->dv * pNextDelaunayLink->du + pA->du * pNextDelaunayLink->dv < 0);
					}
					else
						break;
				}	// while(TRUE)

				if(!bConvex)
					break;

				// check whether any point E on the region boundary between B and C is further than
				// maxLineSegmentErr from the line BC

				if(bBC)
				{
					eThr = -DOUBLE2INT(maxLineSegmentErr * sqrt((double)(duBC * duBC + dvBC * dvBC)));

					pE = pB;

					duBE = dvBE = 0;

					do
					{
						if(-dvBC * duBE + duBC * dvBE < eThr)
						{
							bConvex = FALSE;

							break;
						}

						pNextDelaunayLink = pE->pNext;

						while((pNextDelaunayLink->Flags & RVLMESH_LINK_FLAG_EDGE) == 0)
							pNextDelaunayLink = pNextDelaunayLink->pNext;

						duBE += pNextDelaunayLink->du;
						dvBE += pNextDelaunayLink->dv;

						pE = pNextDelaunayLink->pOpposite;
					}
					while(pE != pC);	
				}	// if(bBC)

				pB2[iSide] = pB;
				pC2[iSide] = pC;

				pDelaunayLink = pDelaunayLink->pOpposite;
			}	// for every side of the link

			if(bConvex)
			{
				// set RVLMESH_LINK_CONCAVITY flags
				
				for(iSide = 0; iSide < 2; iSide++)
				{
					pB = pB2[iSide];
					pC = pC2[iSide];

					if(pB == pC)
						continue;

					pE = pB;

					pNextDelaunayLink = pE->pNext;

					while((pNextDelaunayLink->Flags & RVLMESH_LINK_FLAG_EDGE) == 0)
						pNextDelaunayLink = pNextDelaunayLink->pNext;

					pE = pNextDelaunayLink->pOpposite;

					while(pE != pC)
					{
						pE->Flags |= RVLMESH_LINK_CONCAVITY;

						pNextDelaunayLink = pE->pNext;

						while((pNextDelaunayLink->Flags & RVLMESH_LINK_FLAG_EDGE) == 0)
							pNextDelaunayLink = pNextDelaunayLink->pNext;

						pE = pNextDelaunayLink->pOpposite;
					}
					
				}
			}
			else
			{
				pDelaunayLink->Flags |= RVLMESH_LINK_FLAG_EDGE;

				pDelaunayLink->pOpposite->Flags |= RVLMESH_LINK_FLAG_EDGE;
			}

			pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pLinkPtr->pNext);
		}
	}

	delete[] LinkPtrMem;
}


//Funkcija za spremanje mesha od Convex Hulla
void SaveMeshCH(CRVLClass *p2DRegionSet, RVL3DPOINT2 **Point3DMap, int *IdxMap, int imgSize)	
{
	FILE *fpPts, *fpIdx, *fpSizes;

	fpPts = fopen("C:\\RVL\\ExpRez\\mesh2PtsCH.dat", "a");

	fpIdx = fopen("C:\\RVL\\ExpRez\\mesh2IdxCH.dat", "a");

	fpSizes = fopen("C:\\RVL\\ExpRez\\mesh2SizesCH.dat", "a+");
	//int off = fseek(fpSizes, 0, SEEK_CUR);
	rewind(fpSizes);

	int noPoints = 1, noTriangles = 0, tempP = 0, tempT = 0, label = 1, ptsOff = 0, triOff = 0;
	while (!feof(fpSizes))
	{
		fscanf(fpSizes, "%d\t%d\n", &tempP, &tempT);
		ptsOff += tempP;
		triOff += tempT;	//Ne treba???
		if ((tempP == 0) && (tempT == 0))
			continue;
		label++;
	}

	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	CRVL2DRegion2 *pPolygon;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;

	//Izracunavamo zakrivljenosti prvo
	int *Idx = new int[p2DRegionList->m_nElements * 3];
	memset(Idx, 0, noTriangles * 3 * sizeof(int));
	float *Curvatures = new float[imgSize * 2];
	memset(Curvatures, 0, imgSize * 2 * sizeof(float));
	float *minD = new float[imgSize * 3];
	memset(minD, 0, imgSize * 3 * sizeof(float));
	float *maxD = new float[imgSize * 3];
	memset(maxD, 0, imgSize * 3 * sizeof(float));
	CalculatePrincipialCurvature2(imgSize, Point3DMap, p2DRegionSet, Curvatures, minD, maxD);
	
	FILE *fpCurvs, *fpDirs;
	fpCurvs = fopen("C:\\RVL\\ExpRez\\mesh2CurvsCH.dat", "a");
	fpDirs = fopen("C:\\RVL\\ExpRez\\mesh2DirsCH.dat", "a");

	p2DRegionList->Start();	

	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);

		do
		{
			iPix = pLink->iPix0;
			if(IdxMap[iPix] == 0)
			{
				pPt = Point3DMap[iPix];
				//zapisuje tocku
				fprintf(fpPts, "%d\t%d\t%d\n", pPt->u, pPt->v, pPt->d);			
				//i njezin indeks u trokutic
				fprintf(fpIdx, "%d\t", noPoints + ptsOff);
				IdxMap[iPix] = noPoints + ptsOff;
				noPoints++;	
				//zapisivanje zakrivljenosti i smjerova
				fprintf(fpCurvs, "%.5f\t%.5f\n", Curvatures[iPix * 2], Curvatures[iPix * 2 + 1]);
				fprintf(fpDirs, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", minD[iPix * 3], minD[iPix * 3 + 1], minD[iPix * 3 + 2], maxD[iPix * 3], maxD[iPix * 3 + 1], maxD[iPix * 3 + 2]);
			}
			else
			{
				fprintf(fpIdx, "%d\t", IdxMap[iPix]);
			}
			
			pLink = pLink->pNext->pOpposite;			
		}while(pLink != pLink0);

		fprintf(fpIdx, "%d\n", label);
		noTriangles++;
	}
	noPoints--;
	fprintf(fpSizes, "%d\t%d\n", noPoints, noTriangles);

	fclose(fpSizes);
	fclose(fpPts);
	fclose(fpIdx);

	fclose(fpCurvs);
	fclose(fpDirs);

	delete [] Idx;
	delete [] Curvatures;
	delete [] minD;
	delete [] maxD;

}

//funkcija za proracunavanje principijalne zakrivljenosti mesha
void CalculatePrincipialCurvature(int noVertices,  RVL3DPOINT2 **Point3DMap, CRVLClass *p2DRegionSet, int *Idx, float *Curvatures, float *minD, float *maxD)
{
	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);
	CRVL2DRegion2 *pPolygon, *pPolygonNext;
	RVLMESH_LINK *pLink0, *pLink, *pLinkNext;
	int iPix;
	RVL3DPOINT2 *pPt;
	p2DRegionList->Start();

	int noTriangles = p2DRegionList->m_nElements;
	double wLen = 0;
	float *vNormals = new float[noVertices * 3];
	memset(vNormals, 0, noVertices * 3 * sizeof(float));
	
	//Ponovo obilazimo sve trokute svih regija, indeksiramo trokute i pronalazimo normale u svakom vertex-u
	int nIdx = 0, pIdx = 0;
	p2DRegionList->Start();
	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());
		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;
		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		do
		{
			iPix = pLink->iPix0;

			pPolygonNext = (CRVL2DRegion2 *)(pLink->vp2DRegion);
			/*vNormals[iPix * 3] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[0];
			vNormals[iPix * 3 + 1] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[1];
			vNormals[iPix * 3 + 2] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[2];*/
			vNormals[iPix * 3] += pPolygonNext->m_N[0];
			vNormals[iPix * 3 + 1] += pPolygonNext->m_N[1];
			vNormals[iPix * 3 + 2] += pPolygonNext->m_N[2];
			pLinkNext = pLink->pNext;
			do
			{
				if (pLinkNext != pLink)
				{
					pPolygonNext = (CRVL2DRegion2 *)(pLinkNext->vp2DRegion);
					/*vNormals[iPix * 3] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[0];
					vNormals[iPix * 3 + 1] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[1];
					vNormals[iPix * 3 + 2] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[2];*/
					vNormals[iPix * 3] += pPolygonNext->m_N[0];
					vNormals[iPix * 3 + 1] += pPolygonNext->m_N[1];
					vNormals[iPix * 3 + 2] += pPolygonNext->m_N[2];
				}
				pLinkNext = pLinkNext->pNext;
			}while(pLink != pLinkNext);
			wLen = sqrtf(vNormals[iPix * 3] * vNormals[iPix * 3] + vNormals[iPix * 3 + 1] * vNormals[iPix * 3 + 1] + vNormals[iPix * 3 + 2] * vNormals[iPix * 3 + 2]);
			vNormals[iPix * 3] = vNormals[iPix * 3] / wLen;
			vNormals[iPix * 3 + 1] = vNormals[iPix * 3 + 1] / wLen;
			vNormals[iPix * 3 + 2] = vNormals[iPix * 3 + 2] / wLen;
			Idx[3 * nIdx + pIdx] = iPix;
			pIdx++;
			pLink = pLink->pNext->pOpposite;			
		}while(pLink != pLink0);
		nIdx++;
		pIdx = 0;
	}
	// Compute the matrix of normal derivatives.
	//Prvo definiramo varijable za svaki vertex
	float*** DNormal = new float**[noVertices];
    float*** WWTrn = new float**[noVertices];
    float*** DWTrn = new float**[noVertices];
	for (int i = 0; i < noVertices; i++)
	{
		DNormal[i] = new float*[3];
		WWTrn[i] = new float*[3];
		DWTrn[i] = new float*[3];
		for (int j = 0; j < 3; j++)
		{
			DNormal[i][j] = new float[3];
			WWTrn[i][j] = new float[3];
			DWTrn[i][j] = new float[3];
			for (int k = 0; k < 3; k++)
			{
				DNormal[i][j][k] = 0;
				WWTrn[i][j][k] = 0;
				DWTrn[i][j][k] = 0;
			}
		}	
	}  
    bool* DWTrnZero = new bool[noVertices];
    memset(DWTrnZero, 0, noVertices*sizeof(bool));
	
	int Vi[3], v0, v1, v2;
	double* temp2 = new double[3];
	double temp;
	double* E = new double[3];
	double* W = new double[3];
	double* D = new double[3];
    for (int i = 0; i < noTriangles; i++)
    {
		if ((Idx[i * 3] < 0) || (Idx[i * 3 + 1] < 0) || (Idx[i * 3 + 2] < 0))
			continue;
        // Get vertex indices.      
        Vi[0] = Idx[i * 3];
        Vi[1] = Idx[i * 3 + 1];
        Vi[2] = Idx[i * 3 + 2];

        for (int j = 0; j < 3; j++)
        {
			//odredujemo redoslijed
            v0 = Vi[j];
            v1 = Vi[(j+1)%3];
            v2 = Vi[(j+2)%3];

            // Compute edge from V0 to V1, project to tangent plane of vertex,
            // and compute difference of adjacent normals.
            
			E[0] = Point3DMap[v1]->u - Point3DMap[v0]->u;
			E[1] = Point3DMap[v1]->v - Point3DMap[v0]->v;
			E[2] = Point3DMap[v1]->d - Point3DMap[v0]->d;
           
			temp = E[0] * vNormals[v0 * 3] + E[1] * vNormals[v0 * 3 + 1] + E[2] * vNormals[v0 * 3 + 2];
			temp2[0] = temp * vNormals[v0 * 3];
			temp2[1] = temp * vNormals[v0 * 3 + 1];
			temp2[2] = temp * vNormals[v0 * 3 + 2];
			RVLDif3D(E, temp2, W);
            
			D[0] = vNormals[v1 * 3] - vNormals[v0 * 3];
			D[1] = vNormals[v1 * 3 + 1] - vNormals[v0 * 3 + 1];
			D[2] = vNormals[v1 * 3 + 2] - vNormals[v0 * 3 + 2];
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 3; col++)
                {
                    WWTrn[v0][row][col] += W[row] * W[col];
                    DWTrn[v0][row][col] += D[row] * W[col];
                }
            }

            // Compute edge from V0 to V2, project to tangent plane of vertex,
            // and compute difference of adjacent normals.
			E[0] = Point3DMap[v2]->u - Point3DMap[v0]->u;
			E[1] = Point3DMap[v2]->v - Point3DMap[v0]->v;
			E[2] = Point3DMap[v2]->d - Point3DMap[v0]->d;
			temp = E[0] * vNormals[v0 * 3] + E[1] * vNormals[v0 * 3 + 1] + E[2] * vNormals[v0 * 3 + 2];
			temp2[0] = temp * vNormals[v0 * 3];
			temp2[1] = temp * vNormals[v0 * 3 + 1];
			temp2[2] = temp * vNormals[v0 * 3 + 2];
			RVLDif3D(E, temp2, W);
			D[0] = vNormals[v2 * 3] - vNormals[v0 * 3];
			D[1] = vNormals[v2 * 3 + 1] - vNormals[v0 * 3 + 1];
			D[2] = vNormals[v2 * 3 + 2] - vNormals[v0 * 3 + 2];
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 3; col++)
                {
                    WWTrn[v0][row][col] += W[row]*W[col];
                    DWTrn[v0][row][col] += D[row]*W[col];
                }
            }
        }
    }
	delete [] E;
	delete [] W;
	delete [] D;
	// Add in N*N^T to W*W^T for numerical stability.  In theory 0*0^T gets
    // added to D*W^T, but of course no update is needed in the
    // implementation.  Compute the matrix of normal derivatives.
	double maxAbs, absEntry;
	//temp varijable potrebene kod mnozenja matrica
	double* temp3 = new double[9];
	double* temp4 = new double[9];
	double* temp5 = new double[9];
	double* temp6 = new double[9];
	int br = 0;
    for (int i = 0; i < noVertices; i++)
    {
		if ((vNormals[i * 3] == 0) && (vNormals[i * 3 + 1] == 0) && (vNormals[i * 3 + 2] == 0))
			continue;

        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                WWTrn[i][row][col] = ((float)0.5)*WWTrn[i][row][col] +
                    vNormals[i * 3 + row] * vNormals[i * 3 + col];
                DWTrn[i][row][col] *= 0.5;
            }
        }

        // Compute the max-abs entry of D*W^T.  If this entry is (nearly)
        // zero, flag the DNormal matrix as singular.
        maxAbs = 0;
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
				absEntry =  fabs(DWTrn[i][row][col]);
                if (absEntry > maxAbs)
                {
                    maxAbs = absEntry;
                }
            }
        }
        if (maxAbs < 1e-07f)
        {
            DWTrnZero[i] = true;
        }
		br = 0;
		for (int r = 0; r < 3; r++)
		{
			for (int s = 0; s < 3; s++)
			{
				temp3[br] = DWTrn[i][r][s];
				temp4[br] = WWTrn[i][r][s];
				br++;
			}
		}
		InverseMatrix3(temp5, temp4);
		MatrixMultiplication(temp3, temp5, temp6, 3, 3);
		br = 0;
		for (int r = 0; r < 3; r++)
		{
			for (int s = 0; s < 3; s++)
			{
				DNormal[i][r][s] = temp6[br];
				br++;
			}
		}
        //DNormal[i] = DWTrn[i]*WWTrn[i].Inverse();
    }
	
	//Brisemo privremene varijable
	for (int i = 0; i < noVertices; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			delete [] WWTrn[i][j];
			delete [] DWTrn[i][j];
		}
		delete [] WWTrn[i];
		delete [] DWTrn[i];
	}
	delete [] WWTrn;
	delete [] DWTrn;

	delete [] temp2;
	delete [] temp3;
	delete [] temp4;
	delete [] temp5;
	delete [] temp6;

	// If N is a unit-length normal at a vertex, let U and V be unit-length
    // tangents so that {U, V, N} is an orthonormal set.  Define the matrix
    // J = [U | V], a 3-by-2 matrix whose columns are U and V.  Define J^T
    // to be the transpose of J, a 2-by-3 matrix.  Let dN/dX denote the
    // matrix of first-order derivatives of the normal vector field.  The
    // shape matrix is
    //   S = (J^T * J)^{-1} * J^T * dN/dX * J = J^T * dN/dX * J
    // where the superscript of -1 denotes the inverse.  (The formula allows
    // for J built from non-perpendicular vectors.) The matrix S is 2-by-2.
    // The principal curvatures are the eigenvalues of S.  If k is a principal
    // curvature and W is the 2-by-1 eigenvector corresponding to it, then
    // S*W = k*W (by definition).  The corresponding 3-by-1 tangent vector at
    // the vertex is called the principal direction for k, and is J*W.
	
	/*Curvatures = new float[noVertices * 2];
	minD = new float[noVertices * 3];
	maxD = new float[noVertices * 3];*/
	double* U = new double[3];
	double* V = new double[3];
	float S[2][2];
	float s01, s10, sAvr, trace, det, discr, rootDiscr;
	double* temp1 = new double[3];
	float W0[2], slenW0;
	float W1[2], slenW1;
    for (int i = 0; i < noVertices; ++i)
    {
		if ((vNormals[i * 3] == 0) && (vNormals[i * 3 + 1] == 0) && (vNormals[i * 3 + 2] == 0))
			continue;

        // Compute U and V given N.
		temp1[0] = vNormals[i * 3];
		temp1[1] = vNormals[i * 3 + 1];
		temp1[2] = vNormals[i * 3 + 2];
		GenerateComplementBasis(U, V, temp1);

        if (DWTrnZero[i])
        {
            // At a locally planar point.
			Curvatures[i * 2] = 0;
			Curvatures[i * 2 + 1] = 0;
			minD[i * 3] = U[0];
			minD[i * 3 + 1] = U[1];
			minD[i * 3 + 2] = U[2];
			maxD[i * 3] = V[0];
			maxD[i * 3 + 1] = V[1];
			maxD[i * 3 + 2] = V[2];
            continue;
        }

        // Compute S = J^T * dN/dX * J.  In theory S is symmetric, but
        // because we have estimated dN/dX, we must slightly adjust our
        // calculations to make sure S is symmetric.
		temp1[0] = DNormal[i][0][0] * V[0] + DNormal[i][0][1] * V[1] + DNormal[i][0][2] * V[2];
		temp1[1] = DNormal[i][1][0] * V[0] + DNormal[i][1][1] * V[1] + DNormal[i][1][2] * V[2];
		temp1[2] = DNormal[i][2][0] * V[0] + DNormal[i][2][1] * V[1] + DNormal[i][2][2] * V[2];
		S[1][1] = RVLDotProduct(V, temp1);
		s01 = U[0] * temp1[0] + U[1] * temp1[1] + U[2] * temp1[2];
		temp1[0] = DNormal[i][0][0] * U[0] + DNormal[i][0][1] * U[1] + DNormal[i][0][2] * U[2];
		temp1[1] = DNormal[i][1][0] * U[0] + DNormal[i][1][1] * U[1] + DNormal[i][1][2] * U[2];
		temp1[2] = DNormal[i][2][0] * U[0] + DNormal[i][2][1] * U[1] + DNormal[i][2][2] * U[2];
		S[0][0] = RVLDotProduct(U, temp1);
		s10 = V[0] * temp1[0] + V[1] * temp1[1] + V[2] * temp1[2];
        sAvr = (s01 + s10) / 2;
		S[0][1] = S[1][0] = sAvr;
		
        // Compute the eigenvalues of S (min and max curvatures).
        trace = S[0][0] + S[1][1];
        det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        discr = trace * trace - 4.0 * det;
        rootDiscr = sqrtf(fabs(discr));
		Curvatures[i * 2] = (trace - rootDiscr) / 2;
		Curvatures[i * 2 + 1] = (trace + rootDiscr) / 2;

        // Compute the eigenvectors of S.
		W0[0] = S[0][1];
		W0[1] = Curvatures[i * 2] - S[0][0];
		slenW0 = W0[0] * W0[0] + W0[1] * W0[1];
		W1[0] = Curvatures[i * 2] - S[1][1];
		W1[1] = S[1][0];
		slenW1 = W1[0] * W1[0] + W1[1] * W1[1];
        if (slenW0 >= slenW1)
        {
            W0[0] /= sqrtf(slenW0);
			W0[1] /= sqrtf(slenW0);
			minD[i * 3] = W0[0] * U[0] + W0[1] * V[0];
			minD[i * 3 + 1] = W0[0] * U[1] + W0[1] * V[1];
			minD[i * 3 + 2] = W0[0] * U[2] + W0[1] * V[2];
        }
        else
        {
            W1[0] /= sqrtf(slenW1);
			W1[1] /= sqrtf(slenW1);
			minD[i * 3] = W1[0] * U[0] + W1[1] * V[0];
			minD[i * 3 + 1] = W1[0] * U[1] + W1[1] * V[1];
			minD[i * 3 + 2] = W1[0] * U[2] + W1[1] * V[2];
        }
		
		W0[0] = S[0][1];
		W0[1] = Curvatures[i * 2 + 1] - S[0][0];
		slenW0 = W0[0] * W0[0] + W0[1] * W0[1];
		W1[0] = Curvatures[i * 2 + 1] - S[1][1];
		W1[1] = S[1][0];
		slenW1 = W1[0] * W1[0] + W1[1] * W1[1];
        if (slenW0 >= slenW1)
        {
            W0[0] /= sqrtf(slenW0);
			W0[1] /= sqrtf(slenW0);
			maxD[i * 3] = W0[0] * U[0] + W0[1] * V[0];
			maxD[i * 3 + 1] = W0[0] * U[1] + W0[1] * V[1];
			maxD[i * 3 + 2] = W0[0] * U[2] + W0[1] * V[2];
        }
        else
        {
            W1[0] /= sqrtf(slenW1);
			W1[1] /= sqrtf(slenW1);
			maxD[i * 3] = W1[0] * U[0] + W1[1] * V[0];
			maxD[i * 3 + 1] = W1[0] * U[1] + W1[1] * V[1];
			maxD[i * 3 + 2] = W1[0] * U[2] + W1[1] * V[2];
        }
    }
	//brisemo varijable
	for (int i = 0; i < noVertices; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			delete [] DNormal[i][j];
		}
		delete [] DNormal[i];
	}
	delete [] DNormal;

	delete [] DWTrnZero;
	delete [] vNormals;
	delete [] U;
	delete [] V;
	delete [] temp1;
}

//funkcija za brze(???) proracunavanje principijalne zakrivljenosti mesha
void CalculatePrincipialCurvature2(int noVertices,  RVL3DPOINT2 **Point3DMap, CRVLClass *p2DRegionSet, float *Curvatures, float *minD, float *maxD)
{
	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);
	CRVL2DRegion2 *pPolygon, *pPolygonNext;
	RVLMESH_LINK *pLink0, *pLink, *pLinkNext;
	int iPix;
	RVL3DPOINT2 *pPt;
	p2DRegionList->Start();

	int noTriangles = p2DRegionList->m_nElements;
	double wLen = 0;
	double *vNormals = new double[noVertices * 3];
	memset(vNormals, 0, noVertices * 3 * sizeof(double));
	int *noNeighbors = new int[noTriangles * 2 + 1];
	memset(noNeighbors, 0, (noTriangles * 2 + 1) * sizeof(int));
	int **vNeighborhood = new int*[noTriangles * 2 + 1];
	for (int i = 0; i < noTriangles * 2 + 1; i++)
	{
		vNeighborhood[i] = new int[noTriangles * 2 + 1];
		//memset(&vNeighborhood[i], -1, noTriangles * 3 * sizeof(int));
	}
	//stvaramo povezani popis koji ce sadrzavati popis tocaka u okolini neke tocke
	/*RVLQLIST *pList = new RVLQLIST[noVertices];
	RVLQLIST *pListTemp;
	for (int i = 0; i < noVertices; i++)
	{
		pListTemp = &pList[i];
		RVLQLIST_INIT(pListTemp);
	}*/	
	//Ponovo obilazimo sve trokute svih regija, indeksiramo trokute i pronalazimo normale u svakom vertex-u
	//int nIdx = 0, pIdx = 0;
	int br = 0;
	p2DRegionList->Start();
	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());
		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;
		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		do
		{
			iPix = pLink->iPix0;
			if ((vNormals[iPix * 3] != 0) && (vNormals[iPix * 3 + 1] != 0) && (vNormals[iPix * 3 + 2] != 0))
			{
				pLink = pLink->pNext->pOpposite;
				continue;
			}
			pPolygonNext = (CRVL2DRegion2 *)(pLink->vp2DRegion);
			/*vNormals[iPix * 3] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[0];
			vNormals[iPix * 3 + 1] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[1];
			vNormals[iPix * 3 + 2] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[2];*/
			vNormals[iPix * 3] += pPolygonNext->m_N[0];
			vNormals[iPix * 3 + 1] += pPolygonNext->m_N[1];
			vNormals[iPix * 3 + 2] += pPolygonNext->m_N[2];
			pLink = pLink->pOpposite;
			vNeighborhood[br][noNeighbors[br]] = pLink->iPix0;
			noNeighbors[br]++;
			pLink = pLink->pOpposite;
			pLinkNext = pLink->pNext;
			//pListTemp = &pList[iPix];
			do
			{
				if (pLinkNext != pLink)
				{
					pPolygonNext = (CRVL2DRegion2 *)(pLinkNext->vp2DRegion);
					/*vNormals[iPix * 3] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[0];
					vNormals[iPix * 3 + 1] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[1];
					vNormals[iPix * 3 + 2] += 0.5 * pPolygonNext->m_lenN * pPolygonNext->m_N[2];*/
					vNormals[iPix * 3] += pPolygonNext->m_N[0];
					vNormals[iPix * 3 + 1] += pPolygonNext->m_N[1];
					vNormals[iPix * 3 + 2] += pPolygonNext->m_N[2];
					pLinkNext = pLinkNext->pOpposite;
					/*RVLQLIST_INT_ENTRY *plistElement = new RVLQLIST_INT_ENTRY[1];
					plistElement->i = pLinkNext->iPix0;
					RVLQLIST_ADD_ENTRY(pListTemp, plistElement);*/
					vNeighborhood[br][noNeighbors[br]] = pLinkNext->iPix0;
					noNeighbors[br]++;
					pLinkNext = pLinkNext->pOpposite;
				}
				pLinkNext = pLinkNext->pNext;
			}while(pLink != pLinkNext);
			wLen = sqrtf(vNormals[iPix * 3] * vNormals[iPix * 3] + vNormals[iPix * 3 + 1] * vNormals[iPix * 3 + 1] + vNormals[iPix * 3 + 2] * vNormals[iPix * 3 + 2]);
			vNormals[iPix * 3] = vNormals[iPix * 3] / wLen;
			vNormals[iPix * 3 + 1] = vNormals[iPix * 3 + 1] / wLen;
			vNormals[iPix * 3 + 2] = vNormals[iPix * 3 + 2] / wLen;
			//Idx[3 * nIdx + pIdx] = iPix;
			//pIdx++;
			br++;
			pLink = pLink->pNext->pOpposite;			
		}while(pLink != pLink0);
		//nIdx++;
		//pIdx = 0;
	}
	//obilazimo sve tocke i racnamo zakrivljenost
	double* I = new double[9];
	memset(I, 0, 9 * sizeof(double));
	I[0] = 1;
	I[3] = 1;
	I[8] = 1;
	double* M = new double[9];
	double xyz_centroid[3];
	//double* projN = new double[3];
	double demean[3], demean_xy, demean_xz, demean_yz;
	double* C = new double[9];
	double* temp3x3 = new double[9];
	double* temp3 = new double[3];
	double* temp3_2 = new double[3];
	/*RVLQLIST *pL;
	RVLQLIST_INT_ENTRY *pLTemp;
	RVLQLIST projN;
	RVLQLIST *pprojN = &projN;
	RVLQLIST_INIT(pprojN);
	RVLQLIST_PTR_ENTRY *pprojNTemp;*/
	int t, j;
	double *eig = new double[3];
	int *bReal = new int[3];
	double *pNormals = new double[(noTriangles * 2 + 1) * 3];
	memset(pNormals, 0, (noTriangles * 2 + 1) * 3 * sizeof(double));
	p2DRegionList->Start();
	br = 0;
	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());
		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;
		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		do
		{
			iPix = pLink->iPix0;			
			pPolygonNext = (CRVL2DRegion2 *)(pLink->vp2DRegion);
			
			if (((vNormals[iPix * 3] == 0) && (vNormals[iPix * 3 + 1] == 0) && (vNormals[iPix * 3 + 2] == 0)) || ((Curvatures[iPix * 2] != 0) && (Curvatures[iPix * 2 + 1] != 0)))
			{
				pLink = pLink->pNext->pOpposite;
				continue;
			}

			for (t = 0; t < 3; t++)
				temp3[t] = vNormals[iPix * 3 + t];
			RVLVector2Matrix(temp3, 3, temp3x3);
			for (t = 0; t < 9; t++)
				M[t] = I[t] - temp3x3[t];
			memset(xyz_centroid, 0, 3 * sizeof(double));
			for (j = 0; j < noNeighbors[br]; j++)
			{
				for (t = 0; t < 3; t++)
					temp3[t] = vNormals[vNeighborhood[br][j] * 3 + t];
				
				MatrixMultiplicationT(M, temp3, &pNormals[j * 3], 3, 3, 1);
				xyz_centroid[0] += pNormals[j * 3];
				xyz_centroid[1] += pNormals[j * 3 + 1];
				xyz_centroid[2] += pNormals[j * 3 + 2];
			}
			
			xyz_centroid[0] /= noNeighbors[br];
			xyz_centroid[1] /= noNeighbors[br];
			xyz_centroid[2] /= noNeighbors[br];
			memset(C, 0, 9 * sizeof(double));
			for (j = 0; j < noNeighbors[br]; j++)
			{
				demean[0] = pNormals[j * 3] - xyz_centroid[0];
				demean[1] = pNormals[j * 3 + 1] - xyz_centroid[1];
				demean[2] = pNormals[j * 3 + 2] - xyz_centroid[2];
				demean_xy = demean[0] * demean[1];
				demean_xz = demean[0] * demean[2];
				demean_yz = demean[1] * demean[2];
				C[0] += demean[0] * demean[0];
				C[1] += demean_xy;
				C[2] += demean_xz;
				C[3] += demean_xy;
				C[4] += demean[1] * demean[1];
				C[5] += demean_yz;
				C[6] += demean_xz;
				C[7] += demean_yz;
				C[8] += demean[2] * demean[2];
			}
			
			RVLEig3(C, eig, bReal);		
			Sort3(eig);
			Curvatures[iPix * 2] = eig[0];
			Curvatures[iPix * 2 + 1] = eig[2];

			pLink = pLink->pNext->pOpposite;
			br++;
		}while(pLink != pLink0);
		//nIdx++;
		//pIdx = 0;
	}

	/*for (int i = 0; i < noVertices; i++)
	{
		if ((vNormals[i * 3] == 0) && (vNormals[i * 3 + 1] == 0) && (vNormals[i * 3 + 2] == 0))
			continue;
		/*pL = &pList[i];
		if (pL->pFirst == NULL)
			continue;
		for (t = 0; t < 3; t++)
			temp3[t] = vNormals[i * 3 + t];
		RVLVector2Matrix(temp3, 3, temp3x3);
		for (t = 0; t < 9; t++)
			M[t] = I[t] - temp3x3[t];
		//pLTemp = (RVLQLIST_INT_ENTRY *)(pL->pFirst);
		memset(xyz_centroid, 0, 3 * sizeof(double));
		for (j = 0; j < noNeighbors[i]; j++)
		{
			for (t = 0; t < 3; t++)
				temp3[t] = vNormals[vNeighborhood[i][j] * 3 + t];
			
			MatrixMultiplicationT(M, temp3, &pNormals[j * 3], 3, 3, 1);
			xyz_centroid[0] += pNormals[j * 3];
			xyz_centroid[1] += pNormals[j * 3 + 1];
			xyz_centroid[2] += pNormals[j * 3 + 2];
		}
		/*while(pLTemp)
		{
			for (t = 0; t < 3; t++)
				temp3[t] = vNormals[((RVLQLIST_INT_ENTRY *)pLTemp)->i * 3 + t];
			double *pNorm = new double[3];
			MatrixMultiplicationT(M, temp3, pNorm, 3, 3, 1);
			RVLQLIST_PTR_ENTRY *plistElement = new RVLQLIST_PTR_ENTRY[1];
			plistElement->Ptr = pNorm;
			RVLQLIST_ADD_ENTRY(pprojN, plistElement);
			xyz_centroid[0] += pNorm[0];
			xyz_centroid[1] += pNorm[1];
			xyz_centroid[2] += pNorm[2];
			noV++;
			pLTemp = (RVLQLIST_INT_ENTRY *)(pLTemp->pNext);
		}
		xyz_centroid[0] /= noNeighbors[i];
		xyz_centroid[1] /= noNeighbors[i];
		xyz_centroid[2] /= noNeighbors[i];
		//pLTemp = (RVLQLIST_INT_ENTRY *)(pL->pFirst);
		//pprojNTemp = (RVLQLIST_PTR_ENTRY *)(pprojN->pFirst);
		memset(C, 0, 9 * sizeof(double));
		for (j = 0; j < noNeighbors[i]; j++)
		{
			demean[0] = pNormals[j * 3] - xyz_centroid[0];
			demean[1] = pNormals[j * 3 + 1] - xyz_centroid[1];
			demean[2] = pNormals[j * 3 + 2] - xyz_centroid[2];
			demean_xy = demean[0] * demean[1];
			demean_xz = demean[0] * demean[2];
			demean_yz = demean[1] * demean[2];
			C[0] += demean[0] * demean[0];
			C[1] += demean_xy;
			C[2] += demean_xz;
			C[3] += demean_xy;
			C[4] += demean[1] * demean[1];
			C[5] += demean_yz;
			C[6] += demean_xz;
			C[7] += demean_yz;
			C[8] += demean[2] * demean[2];
		}
		/*while(pLTemp)
		{
			temp3 = (double*)(pprojNTemp->Ptr);
			demean[0] = temp3[0] - xyz_centroid[0];
			demean[1] = temp3[1] - xyz_centroid[1];
			demean[2] = temp3[2] - xyz_centroid[2];
			demean_xy = demean[0] * demean[1];
			demean_xz = demean[0] * demean[2];
			demean_yz = demean[1] * demean[2];
			C[0] += demean[0] * demean[0];
			C[1] += demean_xy;
			C[2] += demean_xz;
			C[3] += demean_xy;
			C[4] += demean[1] * demean[1];
			C[5] += demean_yz;
			C[6] += demean_xz;
			C[7] += demean_yz;
			C[8] += demean[2] * demean[2];

			pLTemp = (RVLQLIST_INT_ENTRY *)(pLTemp->pNext);
			pprojNTemp = (RVLQLIST_PTR_ENTRY *)(pprojNTemp->pNext);
		}

		RVLEig3(C, eig, bReal);		
		Sort3(eig);
		Curvatures[iPix * 2] = eig[0];
		Curvatures[iPix * 2 + 1] = eig[2];		
	}*/
	//brisemo varijable
	for (int i = 0; i < noTriangles * 2 + 1; i++)
	{
		delete [] vNeighborhood[i];
	}
	delete [] vNeighborhood;
	delete [] noNeighbors;
	delete [] vNormals;
	delete [] pNormals;	
	delete [] I;
	delete [] M;
	delete [] C;
	delete [] temp3x3; 
	delete [] temp3; 
	delete [] temp3_2;	
	delete [] eig;
	delete [] bReal;
}

//funkcija koja generira komplementarnu bazu
void GenerateComplementBasis(double *v0, double *v1, double *v2)
{
	float invLength;

    if (abs(v2[0]) >= abs(v2[1]))
    {
        // v2.x or v2.z is the largest magnitude component, swap them
        invLength = 1.0f/sqrtf(v2[0]*v2[0] + v2[2]*v2[2]);
        v0[0] = (-1) * v2[2]*invLength;
        v0[1] = 0.0f;
        v0[2] = v2[0]*invLength;
        v1[0] = v2[1]*v0[2];
        v1[1] = v2[2]*v0[0] - v2[0]*v0[2];
        v1[2] = (-1) * v2[1]*v0[0];
    }
    else
    {
        // v2.y or v2.z is the largest magnitude component, swap them
        invLength = 1.0f/sqrtf(v2[1]*v2[1] + v2[2]*v2[2]);
        v0[0] = 0.0f;
        v0[1] = v2[2]*invLength;
        v0[2] = (-1) * v2[1]*invLength;
        v1[0] = v2[1]*v0[2] - v2[2]*v0[1];
        v1[1] = (-1) * v2[0]*v0[2];
        v1[2] = v2[0]*v0[1];
    }

}

void RVLRemoveInternalVertices(CRVLDelaunay *m_pDelaunay)
{
	RVLQLIST *pVertexList = &(m_pDelaunay->m_VertexList);

	BYTE **DelaunayMap = m_pDelaunay->m_DelaunayMap;

	void **ppVertex = &(pVertexList->pFirst);

	//if(!m_pDelaunay->Test3())
	//	int debug = 0;

	RVLQLIST_PTR_ENTRY *pVertex = (RVLQLIST_PTR_ENTRY *)(pVertexList->pFirst);

	RVLMESH_LINK *pLink0, *pLink, *pNextLink, *pLink2, *pLink20, *pLink3;
	BOOL bConcavity;
	CRVL2DRegion2 *pTriangle, *pTriangle2;
	RVLQLIST_PTR_ENTRY *pVertex2;

	while(pVertex)
	{
		pLink0 = pLink = (RVLMESH_LINK *)(pVertex->Ptr);

		//if(pLink0->iPix0 != 48+48*320)
		//{
		//	pVertex = (RVLQLIST_PTR_ENTRY *)(pVertex->pNext);

		//	continue;
		//}	

		//if(pLink0->iPix0 == 54312)
			//break;
			//int debug = 0;

		do
		{
			if(pLink->Flags & RVLMESH_LINK_FLAG_EDGE)
				break;

			pLink = pLink->pNext;
		}
		while(pLink != pLink0);

		if((pLink->Flags & (RVLMESH_LINK_FLAG_EDGE | RVLMESH_LINK_FLAG_ROI)) == 
			RVLMESH_LINK_FLAG_ROI)
		{
			// retriangulation

			while(TRUE)
			{
				// determine if pLink splits a convex quadrangle

				pNextLink = pLink->pNext;

				//if(pNextLink->iPix0 == 52404 && pNextLink->du == -11 && pNextLink->dv == -1)
				//	int debug = 0;

				pLink20 = pLink2 = pNextLink->pOpposite;

				do
				{
					//if(pLink2->iPix0 == 1002)
					//	int debug = 0;

					pLink3 = pLink2->pNext;
				
					if(pLink3 == pLink)
						pLink3 = pLink3->pNext;
					else if(pLink3 == pLink->pOpposite)
						pLink3 = pLink3->pNext;

					pLink3 = pLink3->pOpposite;

					if(bConcavity = (-pLink2->dv * pLink3->du + pLink2->du * pLink3->dv > 0))
						break;
				
					pLink2 = pLink3;
				}
				while(pLink2 != pLink20);

				if(bConcavity)
				{
					pLink = pNextLink;

					if(pLink == pLink0)
						break;
				}
				else
				{
					if(pLink0->pNext->pNext == pLink0)	// degenerate triangle
						break;
					else
					{
						// flip diagonal 

						//if(!m_pDelaunay->Test3())
						//	int debug = 0;

						pLink2 = pLink->pOpposite;

						pVertex2 = (RVLQLIST_PTR_ENTRY *)(DelaunayMap[pLink2->iPix0]);

						if(pLink2 == pVertex2->Ptr)
							pVertex2->Ptr = pLink2->pNext;

						RVLFlipDiagonal(pLink);

						pLink0 = pLink = pNextLink;

						//if(!m_pDelaunay->Test3())
						//	int debug = 0;
					}
				}
			}

			// merge remaining triangles

			pTriangle = (CRVL2DRegion2 *)(pLink->vp2DRegion);

			//int debug = 0;

			do
			{
				pLink2 = pLink->pOpposite;

				pVertex2 = (RVLQLIST_PTR_ENTRY *)(DelaunayMap[pLink2->iPix0]);

				pLink3 = pLink2->pNext;

				if(pLink2 == pVertex2->Ptr)
					pVertex2->Ptr = pLink3;
				
				pLink2 = pLink2->pPrev;
				
				pLink2->pNext = pLink3;
				pLink3->pPrev = pLink2;

				pTriangle2 = (CRVL2DRegion2 *)(pLink->vp2DRegion);

				pTriangle2->m_Flags |= RVLOBJ2_FLAG_REJECTED;

				pLink2->vp2DRegion = pTriangle;

				pLink = pLink->pNext;

				//debug++;
			}
			while(pLink != pLink0);

			//if(debug != 3)
			//	debug = debug + 0;

			//if(pLink0->iPix0 == 6600)
			//	int debug = 0;

			if(pLink0->pNext->pNext == pLink0)	// degenerate triangle
			{
				pVertex2 = (RVLQLIST_PTR_ENTRY *)(DelaunayMap[pLink2->iPix0]);

				pLink3 = pLink2->pNext;
		
				if(pLink2 == pVertex2->Ptr)
					pVertex2->Ptr = pLink3;

				pLink2->pPrev->pNext = pLink3;
				pLink3->pPrev = pLink2->pPrev;

				pLink2 = pLink2->pOpposite;

				pVertex2 = (RVLQLIST_PTR_ENTRY *)(DelaunayMap[pLink2->iPix0]);

				pLink3 = pLink2->pPrev;

				if(pLink2 == pVertex2->Ptr)
					pVertex2->Ptr = pLink3;

				pLink3->pNext = pLink2->pNext;
				pLink2->pNext->pPrev = pLink3;

				pLink3->vp2DRegion = pLink2->vp2DRegion;

				pTriangle = (CRVL2DRegion2 *)(pLink2->vp2DRegion);

				pTriangle->m_PtArray = pLink3;
			}
			else
			{
				pTriangle->m_PtArray = pLink2;

				pTriangle->m_Flags &= ~RVLOBJ2_FLAG_REJECTED;
			}

			// remove vertex from pVertexList

			//if(!m_pDelaunay->Test3())
			//	int debug = 0;

			RVLQLIST_REMOVE_ENTRY(pVertexList, pVertex, ppVertex);

			//if(!m_pDelaunay->Test3())
			//	int debug = 0;

			//ppVertex = &(pVertex->pNext);		// remove this line after decommenting the code above!!!
		}
		else
			ppVertex = &(pVertex->pNext);

		//if(((RVLMESH_LINK *)(((RVLQLIST_PTR_ENTRY *)(DelaunayMap[1005]))->Ptr))->pNext != (RVLMESH_LINK *)0x02332d74)
		//	int debug = 0;

		pVertex = (RVLQLIST_PTR_ENTRY *)(pVertex->pNext);
	}
}

void RVLSegmentationEdgesFromLabels(CRVLClass *pTriangleSet, DWORD Mask, DWORD refMask)
{
	CRVLMPtrChain *pTriangleList = &(pTriangleSet->m_ObjectList);

	CRVL2DRegion2 *pTriangle, *pTriangle2;
	RVLMESH_LINK *pLink, *pLink0, *pLink2;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pTriangle->m_Flags & Mask) != refMask)
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		do
		{	
			pLink2 = pLink->pOpposite;

			pTriangle2 = (CRVL2DRegion2 *)(pLink2->vp2DRegion);

			if(pTriangle2)
			{
				if(pTriangle2->m_Flags & RVLOBJ2_FLAG_REJECTED)
				{
					pLink->Flags |= RVLMESH_LINK_FLAG_EDGE;
					pLink2->Flags |= RVLMESH_LINK_FLAG_EDGE;
				}
				else if((pTriangle2->m_Flags & Mask) != refMask)
				{
					pLink->Flags |= RVLMESH_LINK_FLAG_EDGE;
					pLink2->Flags |= RVLMESH_LINK_FLAG_EDGE;
				}
				else if(pTriangle->m_Label != pTriangle2->m_Label)
				{
					pLink->Flags |= RVLMESH_LINK_FLAG_EDGE;
					pLink2->Flags |= RVLMESH_LINK_FLAG_EDGE;
				}
			}
			else
			{
				pLink->Flags |= RVLMESH_LINK_FLAG_EDGE;
				pLink2->Flags |= RVLMESH_LINK_FLAG_EDGE;
			}

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);
	}
}

void RVLSegmentationGetBoundary(
	CRVL2DRegion2 *pSegment,
	int w,
	RVLQLIST *pContourList,
	CRVLMem *pMem)
{
	RVLARRAY *pRelList = pSegment->m_RelList + pSegment->m_pClass->m_iRelList[RVLRELLIST_ELEMENTS];

	RVLQLIST_INIT(pContourList);

	RVL3DCONTOUR *pContour;
	CRVL2DRegion2 *pTriangle;
	CRVL2DRegion2 **ppTriangle;
	RVLMESH_LINK *pLink, *pLink0, *pLink_, *pLink0_;
	RVL3DPOINT3 *pVertex;
	RVLQLIST *pPtList;
	int Area, u, v;

	for (ppTriangle = (CRVL2DRegion2 **)(pRelList->pFirst); ppTriangle < (CRVL2DRegion2 **)(pRelList->pEnd); ppTriangle++)
	{
		pTriangle = *ppTriangle;

		if (pTriangle->m_Flags & RVLOBJ2_FLAG_VISITED)
			continue;

		pTriangle->m_Flags |= RVLOBJ2_FLAG_VISITED;

		pLink = pLink0 = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		do
		{
			if (pLink->Flags & RVLMESH_LINK_FLAG_EDGE)
			{
				Area = 0;

				RVLMEM_ALLOC_STRUCT(pMem, RVL3DCONTOUR, pContour);

				pContour->iView = 0;

				RVLQLIST_ADD_ENTRY(pContourList, pContour);

				pPtList = &(pContour->PtList);

				RVLQLIST_INIT(pPtList);

				pLink0_ = pLink_ = pLink;

				do
				{
					RVLMEM_ALLOC_STRUCT(pMem, RVL3DPOINT3, pVertex);

					RVLQLIST_ADD_ENTRY(pPtList, pVertex);

					u = pLink_->iPix0 % w;
					v = pLink_->iPix0 / w;

					pVertex->P2D[0] = u;
					pVertex->P2D[1] = v;

					Area += (pLink_->dv*(2 * u + pLink_->du));

					do
						pLink_ = pLink_->pNext;
					while (!(pLink_->Flags & RVLMESH_LINK_FLAG_EDGE));

					pLink_ = pLink_->pOpposite;

					((CRVL2DRegion2 *)(pLink_->vp2DRegion))->m_Flags |= RVLOBJ2_FLAG_VISITED;
				} while (pLink_ != pLink0_);

				pContour->bHole = (Area <= 0);

				break;
			}

			pLink = pLink->pNext->pOpposite;
		} while (pLink != pLink0);
	}

	for (ppTriangle = (CRVL2DRegion2 **)(pRelList->pFirst); ppTriangle < (CRVL2DRegion2 **)(pRelList->pEnd); ppTriangle++)
	{
		pTriangle = *ppTriangle;

		pTriangle->m_Flags &= ~RVLOBJ2_FLAG_VISITED;
	}
}

void RVLSegmentationDisplayBoundary(CRVLFigure *pFig,
									CRVL2DRegion2 *pSegment,
									int ImageWidth,
									CRVLMem *pMem,
									CvScalar Color,
									int LineWidth,
									int uOffset)
{
	CRVLDisplayVector Vector(pFig->m_pMem);

	Vector.m_bClosed = TRUE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector.m_rL = (BYTE)DOUBLE2INT(Color.val[0]);
	Vector.m_gL = (BYTE)DOUBLE2INT(Color.val[1]);
	Vector.m_bL = (BYTE)DOUBLE2INT(Color.val[2]);
	Vector.m_LineWidth = LineWidth;

	CRVLDisplayVector *pVector;

	RVLQLIST ContourList;

	RVLSegmentationGetBoundary(pSegment, ImageWidth, &ContourList, pMem);

	RVL3DPOINT3 *pVertex;
	int u1, v1, u2, v2;

	RVL3DCONTOUR *pContour = (RVL3DCONTOUR *)(ContourList.pFirst);

	while(pContour)
	{
		pVertex = (RVL3DPOINT3 *)(pContour->PtList.pFirst);

		u1 = pVertex->P2D[0];
		v1 = pVertex->P2D[1];

		pVertex = (RVL3DPOINT3 *)(pVertex->pNext);

		while(pVertex)
		{
			u2 = pVertex->P2D[0];
			v2 = pVertex->P2D[1];

			pVector = pFig->AddVector(&Vector);
			
			pVector->Line(((u1 + uOffset) << 1), (v1 << 1), ((u2 + uOffset) << 1), (v2 << 1));

			u1 = u2;
			v1 = v2;

			pVertex = (RVL3DPOINT3 *)(pVertex->pNext);
		}

		pContour = (RVL3DCONTOUR *)(pContour->pNext);
	}
}

void RVLSegmentationMarkSelectedRegion(	CRVLClass *pTriangleSet, 
										int Label,
										DWORD mMask)
{
	CRVLMPtrChain *pTriangleList = &(pTriangleSet->m_ObjectList);

	CRVL2DRegion2 *pTriangle, *pTriangle2;
	RVLMESH_LINK *pLink, *pLink0, *pLink2;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if(pTriangle->m_Label != Label)
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		do
		{	
			pLink2 = pLink->pOpposite;

			pTriangle2 = (CRVL2DRegion2 *)(pLink2->vp2DRegion);

			if(pTriangle2)
			{
				if(pTriangle2->m_Flags & RVLOBJ2_FLAG_REJECTED)
				{
					pLink->Flags |= mMask;
					pLink2->Flags |= mMask;
				}
				else if(pTriangle2->m_Label != Label)
				{
					pLink->Flags |= mMask;
					pLink2->Flags |= mMask;
				}
			}
			else
			{
				pLink->Flags |= mMask;
				pLink2->Flags |= mMask;
			}

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);
	}
}

#ifdef NEVER

RVLMESH_LINK *RVLGetTangentPlane(	RVLMESH_LINK *pLink0,
											RVL3DPOINT2 **Point3DMap,
											int *N,
											RVLMESH_LINK *pLinkA, 
											int *a)
{
	RVL3DPOINT2 *pPtA;

	if(pLinkA)
		pPtA = Point3DMap[pLinkA->iPix0];

	RVLMESH_LINK *pLinkC = pLink0;

	RVL3DPOINT2 *pPtC, *pPtC2;
	int c[3];
	RVLMESH_LINK *pLinkC2, pLinkD;
	BOOL bContinue;

	while(TRUE)
	{
		pPtC = Point3DMap[pLinkC->iPix0];

		if(pLinkA)
		{
			c[0] = pPtC->u;
			c[1] = pPtC->v;
			c[2] = pPtC->d;

			//CrossProduct(c, a, N);		// create CrossProduct for integer vectors as a macro !!!
		}

		bContinue = FALSE;

		pLinkD = pLinkC;

		do
		{
			pLinkC2 = pLinkD->pOpposite

			pPtC2 = Point3DMap[pLinkC2->iPix0];

			if(N[0] * (pPtC2->u - pPtA->u) + N[1] * (pPtC2->v - pPtA->v) + N[2] * (pPtC2->d - pPtA->d) > 0)
			{
				bContinue = TRUE;

				break;
			}
					
			pLinkD = pLinkC2->pNext;
		}
		while(pLinkD != pLinkC);

		if(bContinue)
			pLinkC = pLinkC2;
		else
			return pLinkC;
	}
}

BOOL RVLMergeConvexHulls(	RVLMESH_LINK *pLink1,
							RVLMESH_LINK *pLink2,
							CRVLClass *pTriangleSet,
							int ImageWidth,
							RVL3DPOINT2 **Point3DMap,
							int maxDist,
							CRVLMem *pMem,
							BYTE **DelaunayMap,
							RVLQLIST *pNewTriangleList)
{
	CRVL2DRegion2 *pTriangle = (CRVL2DRegion2 *)(pLink1->vp2DRegion);

	int *N0 = pTriangle->m_N;

	RVLMESH_LINK *pLinkA2 = RVLGetTangentPlane(pLink2, Point3DMap, N0);

	RVL3DPOINT2 *pPtA = Point3DMap[pLink1->iPix0];

	RVL3DPOINT2 *pPtA2 = Point3DMap[pLinkA2->iPix0];

	RVLMESH_LINK *pLinkB = pLink1->pOpposite;

	RVLMESH_LINK *pLinkA, *pLinkB2, *pLinkC, *pLinkD;
	RVL3DPOINT2 *pPtB, *pPtB2, *pPtC, *pPtD;
	int a[3];
	int N[3], N2[3];
	RVLCONVEX_HULL_EDGE_DATA EdgeAB, EdgeBA, EdgeCB, EdgeAC;
	RVLCONVEX_HULL_TRIANGLE *pNewTriangle;

	if(N0[0] * (pPtA->u - pPtA2->u) + N0[1] * (pPtA->v - pPtA2->v) + N0[2] * (pPtA->d - pPtA2->d) >= 0)
	{
		pLinkA = pLink1;

		EdgeAB.pLinkA = pLinkA;
		EdgeAB.pLinkB = pLinkB;
		EdgeAB.iHullA = EdgeAB.iHullB = 0;
	}
	else
	{
		pLinkA = pLinkA2;

		pPtB = pLinkB->iPix0;

		a[0] = pPtB->u - pPtA->u;
		a[1] = pPtB->v - pPtA->v;
		a[2] = pPtB->d - pPtA->d;

		pLinkB = RVLGetTangentPlane(pLinkA->pOpposite, Point3DMap, N, pLinkA, a);

		pLinkB2 = RVLGetTangentPlane(pLink1, Point3DMap, N2, pLinkA, a);

		pPtB = Point3DMap[pLinkB->iPix0];

		pPtB2 = Point3DMap[pLinkB2->iPix0];

		EdgeAB.pLinkA = pLinkA;
		EdgeAB.iHullA = 1;

		if(N[0] * (pPtB2->u - pPtB->u) + N[1] * (pPtB2->v - pPtB->v) + N[2] * (pPtB2->d - pPtB->d) > 0)
		{
			EdgeAB.pLinkB = pLinkB2;
			EdgeAB.iHullB = 0;
		}
		else
		{
			EdgeAB.pLinkB = pLinkB;
			EdgeAB.iHullB = 1;
		}
	}

	RVLQLIST_INIT(pNewTriangleList);

	// create empty FIFO queue Q

	// PUSH(Q, EdgeAB)

	RVLCONVEX_HULL_EDGE_DATA *pEdge;
	BYTE iHullA, iHullB, iHullC;

	//while(!EMPTY(Q))
	{
		// pEdge = POP(Q);

		// if(!VISITED(pEdge->pLinkA, pEdge->pLinkB))
		{
			pLinkA = pEdge->pLinkA;
			pLinkB = pEdge->pLinkB;
			iHullA = pEdge->iHullA;
			iHullB = pEdge->iHullB;

			EdgeBA.pLinkA = pLinkB;
			EdgeBA.pLinkB = pLinkA;
			EdgeBA.iHullA = iHullB;
			EdgeBA.iHullB = iHullA;

			EdgeCB.pLinkB = pLinkB;
			EdgeCB.iHullB = iHullB;

			EdgeAC.pLinkA = pLinkA;
			EdgeAC.iHullA = iHullA;

			pPtA = Point3DMap[pLinkA->iPix0];
			pPtB = Point3DMap[pLinkB->iPix0];

			a[0] = pPtB->u - pPtA->u;
			a[1] = pPtB->v - pPtA->v;
			a[2] = pPtB->d - pPtA->d;

			if(iHullA != iHullB)
			{
				RVLMEM_ALLOC_STRUCT(pMem, RVLCONVEX_HULL_TRIANGLE, pNewTriangle);

				pLinkC = RVLGetTangentPlane(pLinkA->pNext->pOpposite, Point3DMap, N, pLinkA, a);

				pLinkD = RVLGetTangentPlane(pLinkB->pNext->pOpposite, Point3DMap, N2, pLinkA, a);

				pPtD = Point3DMap[pLinkD->iPix0];

				if(N[0] * (pPtD->u - pPtA->u) + N[1] * (pPtD->v - pPtA->v) + N[2] * (pPtD->d - pPtA->d) > 0)
				{
					EdgeCB.pLinkA = pLinkD;
					EdgeCB.iHullA = iHullB;

					EdgeAC.pLinkB = pLinkD;
					EdgeAC.iHullB = iHullB;

					pNewTriangle->pLink = pLinkC;
					pNewTriangle->iPix = pLinkA->iPix0;
				}
				else
				{
					EdgeCB.pLinkA = pLinkC;
					EdgeCB.iHullA = iHullA;

					EdgeAC.pLinkB = pLinkC;
					Edge.iHullB = iHullA;				

					NewTriangle->pLink = pLinkA;
					NewTriangle->iPix = pLinkB->iPix0;
				}

				RVLQLIST_ADD_ENTRY(pNewTriangleList, pNewTriangle);
			}
			else	// if(iHullA == iHullB)
			{
				pLinkC = RVLGetTangentPlane((iHullA ? pLink1 : pLink2), Point3DMap, N, pLinkA, a);

				pTriangle = (CRVL2DRegion2 *)(pLinkA->vp2DRegion);

				pPtC = Point3DMap[pLinkC->iPix0];

				N0 = pTriangle->m_N;

				if(N0[0] * pPtC->u + N0[1] * pPtC->v + N0[2] * pPtC->d < pTriangle->m_d)
				{
					pTriangle->m_Flags |= RVLOBJ2_FLAG_MARKED;

					pLinkC = pLinkA->pOpposite;

					Edge.pLinkA = pLinkC;
					Edge.iHullA = iHullA;

					Edge.pLinkB = pLinkC;
					Edge.iHullB = iHullA;
				}
				else
				{
					iHullC = 1 - iHullA;

					Edge.pLinkA = pLinkC;
					Edge.iHullA = iHullC;

					Edge.pLinkB = pLinkC;
					Edge.iHullB = iHullC;

					RVLMEM_ALLOC_STRUCT(pMem, RVLCONVEX_HULL_TRIANGLE, pNewTriangle);

					pNewTriangle->pLink = pLinkB;
					pNewTriangle->iPix = pLinkC->iPix0;		

					RVLQLIST_ADD_ENTRY(pNewTriangleList, pNewTriangle);
				}
			}

			// MARK_VISITED(EdgeBA.pLinkB, EdgeBA.pLinkA)
			// MARK_VISITED(EdgeCB.pLinkB, EdgeCB.pLinkA)
			// MARK_VISITED(EdgeAC.pLinkB, EdgeAC.pLinkA)
			// PUSH(Q, EdgeBA)
			// PUSH(Q, EdgeCB)
			// PUSH(Q, EdgeAC)
		}
	}

	return TRUE;
}

#endif 

void RVLSegmentationGTFromMesh(	FILE *fpSrc,
								CRVLDelaunay *pDelaunay,
								FILE *fpTgt)
{
	int w = pDelaunay->m_Width;
	int h = pDelaunay->m_Height;

	int ImageSize = w * h;

	BYTE **DelaunayMap = pDelaunay->m_DelaunayMap;

	CRVLMem Mem;

	Mem.Create(1000000);

	CRVLMem *pMem = &Mem;

	RVLQLIST VertexList;

	RVLQLIST *pVertexList = &VertexList;

	RVLQLIST_INIT(pVertexList);

	int nVertices = 0;

	int *VertexArray = NULL;

	int *Segmentation;
	
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, ImageSize, Segmentation);

	memset(Segmentation, 0xff, ImageSize * sizeof(int));

	int labelFalse = -1;

	RVLQLIST_INT_ENTRY *piVertex;
	double fu, fv, fd;
	int u, v, d;
	int *piPix;
	int label;
	int i1, i2, i3;
	int iTmp;
	int iPix1, iPix2, iPix3, iPix4;
	RVLMESH_LINK *pLink, *pLink0, *pLink2;
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	char cTmp;
	RVLQLIST_PTR_ENTRY *pPtrVertex;

	while(!feof(fpSrc))
	{
		char line[200];

		fgets(line, 200, fpSrc);

		switch(line[0]){
		case '#':
			cTmp = line[6];

			line[6] = 0;

			if(strcmp("false", line + 1) == 0)
			{
				line[6] = cTmp;

				sscanf(line, "#false %d", &labelFalse);
			}
			else
				line[6] = cTmp;

			break;
		case 'v':
			if(line[1] == ' ')
			{
				sscanf(line, "v %lf %lf %lf", &fu, &fv, &fd);	

				u = (int)fu;
				v = (int)fv;
				d = (int)fd;

				RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_INT_ENTRY, piVertex);

				piVertex->i = u + v * w;

				RVLQLIST_ADD_ENTRY(pVertexList, piVertex);

				nVertices++;
			}
			else if(line[1] == 'n' && VertexArray == NULL)
			{
				RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nVertices, VertexArray);

				piPix = VertexArray;

				piVertex = (RVLQLIST_INT_ENTRY *)(VertexList.pFirst);

				while(piVertex)
				{
					*(piPix++) = piVertex->i;

					piVertex = (RVLQLIST_INT_ENTRY *)(piVertex->pNext);
				}
			}

			break;
		case 'u':
			sscanf(line, "usemtl material_%d", &label);	

			break;
		case 'f':
			if(label != labelFalse)
			{
				sscanf(line, "f %d//%d %d//%d %d//%d", &i1, &iTmp, &i2, &iTmp, &i3, &iTmp);	

				iPix1 = VertexArray[i1 - 1];
				iPix2 = VertexArray[i2 - 1];
				iPix3 = VertexArray[i3 - 1];

				pPtrVertex = (RVLQLIST_PTR_ENTRY *)(DelaunayMap[iPix1]);

				if(pPtrVertex)
				{
					pLink0 = pLink = (RVLMESH_LINK *)(pPtrVertex->Ptr);

					pLink2 = NULL;

					do
					{
						if(pLink->pOpposite->iPix0 == iPix2)
						{
							if(pLink->pNext->pOpposite->iPix0 == iPix3)
							{
								pLink2 = pLink;
					
								break;
							}
							else if(pLink->pPrev->pOpposite->iPix0 == iPix3)
							{
								pLink2 = pLink->pPrev;

								break;
							}
						}

						pLink = pLink->pNext;
					}
					while(pLink != pLink0);

					if(pLink2)
					{
						pTriangle = (CRVL2DRegion2 *)(pLink2->vp2DRegion);

						if((pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
						{
							pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

							for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
							{
								pPt = *ppPt;

								iPix1 = pPt->iPix;

								if(Segmentation[iPix1] == -1)
									Segmentation[iPix1] = label;
								else if(Segmentation[iPix1] != label)
									Segmentation[iPix1] = -1;
							}
						}
					}
				}
			}
		}
	}

	int *pSegmentation = Segmentation;

	fprintf(fpTgt, "width %d\n", w);
	fprintf(fpTgt, "height %d\n\n", h);

	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pSegmentation++)
			fprintf(fpTgt, "%d ", *pSegmentation);

		fprintf(fpTgt, "\n");
	}
}

BOOL RVLImportSegmentation(	FILE *fp,
						    int *Segmentation)
{
	int Width, Height;
	int Size;

	char line[200];

	fgets(line, 200, fp);

	while(strstr(line, "width") == NULL)
		fgets(line, 200, fp);

	sscanf(line, "width %d", &Width);

	fscanf(fp, "height %d\n", &Height);

	int *pSegmentation = Segmentation;

	BOOL bOK = TRUE;

	int u, v;

	for(v = 0; v < Height && bOK; v++)
	{
		for(u = 0; u < Width; u++, pSegmentation++)
			if(!(bOK = (fscanf(fp, "%d ", pSegmentation) == 1)))
				break;
	}

	return bOK;
}


void RVLCompareToGT(int *Segmentation,
					int *GT,
					int w, int h,
					RVLSEGMENT_VMEASURE_DATA *pVMeasureData)
{
	int *pSegmentationEnd = Segmentation + w * h;
	int *pGT = GT;

	int *pSegmentation;

	int maxLabel = 0;
	int maxLabelGT = 0; 

	for(pSegmentation = Segmentation; pSegmentation < pSegmentationEnd; pSegmentation++, pGT++)
	{
		if(*pSegmentation > maxLabel)
			maxLabel = *pSegmentation;

		if(*pGT > maxLabelGT)
			maxLabelGT = *pGT;
	}

	int n = maxLabelGT + 1;
	int m = maxLabel + 1;

	int *A = (int *)calloc(n * m, sizeof(int));

	pGT = GT;

	int k, c;

	for(pSegmentation = Segmentation; pSegmentation < pSegmentationEnd; pSegmentation++, pGT++)
	{
		k = *pSegmentation;

		if(k < 0)
			continue;

		c = *pGT;

		if(c < 0)
			continue;

		A[c * m + k]++;
	}

	int *aC = new int[n];
	int *aK = new int[m];

	memset(aK, 0, m * sizeof(int));

	int *pA = A;

	int N = 0;

	int a, aC2;

	for(c = 0; c < n; c++)
	{
		aC2 = 0;

		for(k = 0; k < m; k++, pA++)
		{
			a = *pA;

			aC2 += a;
			aK[k] += a;
			N += a;
		}

		aC[c] = aC2;
	}

	pVMeasureData->N = (double)N;

	pVMeasureData->HCK = 0.0;
	pVMeasureData->HKC = 0.0;

	pA = A;

	double fa;

	for(c = 0; c < n; c++)
		for(k = 0; k < m; k++, pA++)
		{
			a = *pA;

			if(a == 0)
				continue;

			fa = (double)a;

			pVMeasureData->HCK -= fa * log(fa / (double)aK[k]);

			pVMeasureData->HKC -= fa * log(fa / (double)aC[c]);
		}

	pVMeasureData->HCK /= pVMeasureData->N;
	pVMeasureData->HKC /= pVMeasureData->N;

	pVMeasureData->HC = 0.0;

	for(c = 0; c < n; c++)
	{
		a = aC[c];

		if(a == 0)
			continue;

		fa = (double)a;

		pVMeasureData->HC -= fa * log(fa / pVMeasureData->N);
	}

	pVMeasureData->HC /= pVMeasureData->N;

	pVMeasureData->HK = 0.0;

	for(k = 0; k < m; k++, pA++)
	{
		a = aK[k];

		if(a == 0)
			continue;

		fa = (double)a;

		pVMeasureData->HK -= fa * log(fa / pVMeasureData->N);
	}

	pVMeasureData->HK /= pVMeasureData->N;

	delete[] aC;
	delete[] aK;

	free(A);

	pVMeasureData->hom = 1.0 - pVMeasureData->HCK / pVMeasureData->HC;
	pVMeasureData->com = 1.0 - pVMeasureData->HKC / pVMeasureData->HK;

	pVMeasureData->V = 2.0 * pVMeasureData->hom * pVMeasureData->com / (pVMeasureData->hom + pVMeasureData->com);
}

void RVLGetSegmentation( CRVLMPtrChain *pTriangleList,
						 int *Segmentation,
						 int w, int h)
{
	memset(Segmentation, 0xff, w * h * sizeof(int));

	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;

			if(Segmentation[iPix] == -1)
				Segmentation[iPix] = label;
			else if(Segmentation[iPix] != label)
				Segmentation[iPix] = -1;
		}
	}
}


void RVLGetSegmentation( CRVL2DRegion2 **RegionMap,
						 int *Segmentation,
						 int w, int h)
{
	int *piSeg;

	int *pSegmentationEnd = Segmentation + w * h;

	CRVL2DRegion2 **ppRegion = RegionMap;

	CRVL2DRegion2 *pRegion;
	
	for(piSeg = Segmentation; piSeg < pSegmentationEnd; piSeg++, ppRegion++)
	{
		pRegion = *ppRegion;

		*piSeg = (pRegion ? pRegion->m_Index : -1);
	}
}

void RVLSaveSegmentation(FILE *fp,
						 int *Segmentation,
						 int w, int h)
{
	fprintf(fp, "width %d\n", w);
	fprintf(fp, "height %d\n\n", h);

	int *pSegmentation = Segmentation;

	int u, v;

	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pSegmentation++)
			fprintf(fp, "%d ", *pSegmentation);

		fprintf(fp, "\n");
	}
}

void RVLSWERUpdateQueue(RVLSWER_LINK *pLink,
						RVLSWER_LINK *pLink0,
						int maxCost,
						double k,
						void RVLSWERUpdateLink(	RVLSWER_NODE *pNode1,
												RVLSWER_NODE *pNode2,
												RVLSWER_LINK *pLink),
						int RVLSWERGetCost(	BYTE *pData,
											int maxCost,
											double k),
						RVLQLIST *QueueListArray,
						int &Cost)
{
	RVLSWER_NODE *pNode1 = pLink->pNode[0];
	RVLSWER_NODE *pNode2 = pLink->pNode[1];

	RVLSWERUpdateLink(pNode1, pNode2, pLink);

	int NewCost = RVLSWERGetCost(pLink->pData, maxCost, k);

	if(pLink->Cost >= 0)
	{
		if(NewCost == pLink->Cost)
			return;

		if(pLink == pLink0->pNext)
			pLink0->pNext = pLink->pNext;

		RVLQLIST *pQueueList = QueueListArray + pLink->Cost;

		RVLQLIST_REMOVE_ENTRY2(pQueueList, pLink, RVLSWER_LINK);

		//if(pLink->pNext == pLink)
		//	int debug = 0;
	}

	pLink->Cost = NewCost;

	RVLQLISTARRAY_ADD_ENTRY2(QueueListArray, NewCost, pLink);

	//if(pLink->pNext == pLink)
	//	int debug = 0;

	if(NewCost <= Cost)
		Cost = NewCost - 1;
}


int RVLSegmentationWER(	void RVLSWEROnCreateNewNode(RVLSWER_NODE *pNode,
													RVLSWER_LINK *pLink),
						void RVLSWERUpdateLink(	RVLSWER_NODE *pNode1,
												RVLSWER_NODE *pNode2,
												RVLSWER_LINK *pLink),
						int RVLSWERGetCost(	BYTE *pData,
											int maxCost,
											double k),
						RVLSWER_NODE *pNewNode,
						BYTE *pNodeData,
						int NodeDataSize,
						RVLQLIST *QueueListArray,
						int maxnNodes,
						int minCost,
						int maxCost,
						double k)
{
	// build tree by a WER procedure

	RVLSWER_NODE *pFirstNewNode = pNewNode;

	RVLSWER_NODE **NodeBuff = new RVLSWER_NODE *[maxnNodes];

	RVLQLIST *pQueueList, *pQueueList2;
	RVLSWER_NODE *pNode, *pNode2, *pNode3;
	RVLSWER_LINK *pLink, *pLink2;
	void **ppNext;
	RVLSWER_NODE **ppNode, **pNodeBuffEnd;
#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
	int iTmp;
#endif
	int Cost;
	RVLQLIST *pLinkPtrList, *pLinkPtrList2;
	RVLQLIST_PTR_ENTRY *pLinkPtr;

	for(Cost = minCost; Cost <= maxCost; Cost++)
	{
		//if(QueueListArray[17].pFirst != NULL && Cost > 17)
		//	int debug = 0;

		pQueueList = QueueListArray + Cost;

		pLink = (RVLSWER_LINK *)(pQueueList->pFirst);

		while(pLink)
		{
#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
			fopen_s(&fpLog, "C:\\RVL\\ExpRez\\PSDMeshSegmentWER.log", "a");

			if(!(bOK = (fpLog != NULL)))
				break;
#endif

			//if((int)pLink == 0x0ee78040)
			//	int debug = 0;

			//if((int)pLink == 0x0ee3a540)
			//	int debug = 0;

			RVLQLIST_REMOVE_ENTRY2(pQueueList, pLink, RVLSWER_LINK);

			pNode = pLink->pNode[0];

			pNode2 = pLink->pNode[1];
	
			// create new node

			pNewNode->Flags = 0x00000000;

			//pNewNode->Moments = pLink->Moments;
			//memcpy(&(pNewNode->Moments), &(pLink->Moments), sizeof(RVL3DMOMENTS));

			pNewNode->pData = pNodeData;

			RVLSWEROnCreateNewNode(pNewNode, pLink);

			pNewNode->pChild[0] = pNode;
			pNewNode->pChild[1] = pNode2;

			pNewNode->pParent = NULL;

			pNewNode->Cost = Cost;

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
			if(!(bOK = (pNode != NULL)))
			{
				fprintf(fpLog, "ERROR: rejected link LINK%0x fetched from the Queue!\n", pLink);

				break;
			}
#endif

			pNode->pParent = pNewNode;
			pNode2->pParent = pNewNode;

			pNewNode->LinkPtrList = pNode->LinkPtrList;

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
			fprintf(fpLog, "\nRemoving LINK%0x:\n", pLink);

			if(!(bOK = MeshSegmentWERNodeWriteToFile(fpLog, pNode, maxnLinks)))
				break;

			if(!(bOK = MeshSegmentWERNodeWriteToFile(fpLog, pNode2, maxnLinks)))
				break;

			fprintf(fpLog, "Updated Queue%d: ", pQueueList - QueueListArray);

			RVLQLIST_WRITE_TO_FILE(pQueueList, RVLSWER_LINK, fpLog, nEntriesInQueueList, maxnLinks);			

			fprintf(fpLog, "Total %d Entries\n", nEntriesInQueueList);
#endif	

			// process all links of the pNode

			//if(pNewNode - NodeMem == 2987)
			//	int debug = 0;

			ppNode = NodeBuff;

			pLinkPtrList = &(pNewNode->LinkPtrList);

			pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pLinkPtrList->pFirst);

			ppNext = &(pLinkPtrList->pFirst);

			while(pLinkPtr)
			{
				pLink2 = (RVLSWER_LINK *)(pLinkPtr->Ptr);

				if(pLink2 == pLink)
				{
					RVLQLIST_REMOVE_ENTRY(pLinkPtrList, pLinkPtr, ppNext);

//#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
//					fprintf(fpLog, "Updated ");
//
//					MeshSegmentWERNodeWriteToFile(fpLog, pNode, maxnLinks);
//#endif
				}
				else 
				{
					pNode3 = pLink2->pNode[0];

					if(pNode3)
					{
						if(pNode3 == pNode)
						{
							pNode3 = pLink2->pNode[1];
							pLink2->pNode[0] = pNewNode;
						}
						else
							pLink2->pNode[1] = pNewNode;

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
						iTmp = pLink2->Cost;
#endif
						RVLSWERUpdateQueue(pLink2, pLink, maxCost, k,
							RVLSWERUpdateLink, RVLSWERGetCost, QueueListArray, Cost);

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
						if(iTmp != pLink2->Cost)
						{
							fprintf(fpLog, "LINK%0x: cost changed\n", pLink2);

							fprintf(fpLog, "Updated Queue%d: ", iTmp);

							RVLQLIST_WRITE_TO_FILE((QueueListArray + iTmp), RVLSWER_LINK, fpLog, nEntriesInQueueList, maxnLinks);			

							fprintf(fpLog, "Total %d Entries\n", nEntriesInQueueList);

							fprintf(fpLog, "Updated Queue%d: ", pLink2->Cost);

							RVLQLIST_WRITE_TO_FILE((QueueListArray + pLink2->Cost), RVLSWER_LINK, fpLog, nEntriesInQueueList, maxnLinks);			

							fprintf(fpLog, "Total %d Entries\n", nEntriesInQueueList);
						}
#endif	

						pNode3->Flags |= RVLSWER_NODE_FLAG_VISITED;

						*(ppNode++) = pNode3;

						ppNext = &(pLinkPtr->pNext);
					}
					else	// if(pNode3 == NULL)
					{
						RVLQLIST_REMOVE_ENTRY(pLinkPtrList, pLinkPtr, ppNext);

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
						fprintf(fpLog, "LINK%0x disconnedted from NODE%0x\n", pLinkPtr->Ptr, pNode);
#endif
					}
				}	// if(pLink2 != pLink)		

				pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pLinkPtr->pNext);
			}	// for all links of pNode

			// process all links of the pNode2
			
			pLinkPtrList2 = &(pNode2->LinkPtrList);

			pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pLinkPtrList2->pFirst);

			ppNext = &(pLinkPtrList2->pFirst);

			while(pLinkPtr)
			{
				pLink2 = (RVLSWER_LINK *)(pLinkPtr->Ptr);

				if(pLink2 == pLink)
				{
					RVLQLIST_REMOVE_ENTRY(pLinkPtrList2, pLinkPtr, ppNext);

//#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
//					fprintf(fpLog, "Updated ");
//
//					MeshSegmentWERNodeWriteToFile(fpLog, pNode2, maxnLinks);
//#endif
				}
				else 
				{
					pNode3 = pLink2->pNode[0];

					if(pNode3)
					{
						if(pNode3 == pNode2)
						{
							pNode3 = pLink2->pNode[1];
							pLink2->pNode[0] = pNewNode;
						}
						else
							pLink2->pNode[1] = pNewNode;

						if(pNode3->Flags & RVLSWER_NODE_FLAG_VISITED)
						{
							RVLQLIST_REMOVE_ENTRY(pLinkPtrList2, pLinkPtr, ppNext);

							pQueueList2 = QueueListArray + pLink2->Cost;

							RVLQLIST_REMOVE_ENTRY2(pQueueList2, pLink2, RVLSWER_LINK);

							pLink2->pNode[0] = NULL;	// this means that pLink2 shoul be ignored in further processing

							if(pLink2 == pLink->pNext)
								pLink->pNext = pLink2->pNext;

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
							fprintf(fpLog, "LINK%0x removed as a duplicate\n", pLink2);

							fprintf(fpLog, "Updated Queue%d: ", pLink2->Cost);

							RVLQLIST_WRITE_TO_FILE((QueueListArray + pLink2->Cost), RVLSWER_LINK, fpLog, nEntriesInQueueList, maxnLinks);			

							fprintf(fpLog, "Total %d Entries\n", nEntriesInQueueList);
#endif								
						}
						else	// if pNode3 not VISITED 
						{
#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
							iTmp = pLink2->Cost;
#endif
							RVLSWERUpdateQueue(pLink2, pLink, maxCost, k,
								RVLSWERUpdateLink, RVLSWERGetCost, QueueListArray, Cost);

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
							if(iTmp != pLink2->Cost)
							{
								fprintf(fpLog, "LINK%0x: cost changed\n", pLink2);

								fprintf(fpLog, "Updated Queue%d: ", iTmp);

								RVLQLIST_WRITE_TO_FILE((QueueListArray + iTmp), RVLSWER_LINK, fpLog, nEntriesInQueueList, maxnLinks);			

								fprintf(fpLog, "Total %d Entries\n", nEntriesInQueueList);

								fprintf(fpLog, "Updated Queue%d: ", pLink2->Cost);

								RVLQLIST_WRITE_TO_FILE((QueueListArray + pLink2->Cost), RVLSWER_LINK, fpLog, nEntriesInQueueList, maxnLinks);			

								fprintf(fpLog, "Total %d Entries\n", nEntriesInQueueList);
							}
#endif	
							ppNext = &(pLinkPtr->pNext);
						}	// if pNode3 not VISITED 
					}
					else	// if(pNode3 == NULL)
					{
						RVLQLIST_REMOVE_ENTRY(pLinkPtrList2, pLinkPtr, ppNext);

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
						fprintf(fpLog, "LINK%0x disconnedted from NODE%0x\n", pLinkPtr->Ptr, pNode2);
#endif
					}
				}	// if(pLink2 != pLink)				

				pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pLinkPtr->pNext);
			}	// for all links of pNode2

			pNodeBuffEnd = ppNode;

			for(ppNode = NodeBuff; ppNode < pNodeBuffEnd; ppNode++)
			{
				pNode = *ppNode;

				pNode->Flags &= ~RVLSWER_NODE_FLAG_VISITED;
			}

			//if((int)pLinkPtrList == 0x0370530c)
			//	int debug = 0;

//#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
//			MeshSegmentWERNodeWriteToFile(fpLog, pNewNode, maxnLinks);
//
//			MeshSegmentWERNodeWriteToFile(fpLog, pNode2, maxnLinks);
//#endif

			//if((int)pNewNode == 0x0370e97c)
			//	int debug = 0;

			RVLQLIST_APPEND(pLinkPtrList, pLinkPtrList2);

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
			fprintf(fpLog, "new ");

			if(!(bOK = MeshSegmentWERNodeWriteToFile(fpLog, pNewNode, maxnLinks)))
				break;

			fclose(fpLog);
#endif
			
			pNewNode++;

			pNodeData += NodeDataSize;

			pLink = (RVLSWER_LINK *)(pLink->pNext);
		}	// for all links in pQueueList

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
		if(!bOK)
			break;
#endif
	}	// for all costs

	// only for debugging purposes !!!

	//for(Cost = 0; Cost < maxCost; Cost++)
	//	if(QueueListArray[Cost].pFirst != NULL)
	//		int debug = 0;

	// deallocate memory

	delete[] NodeBuff;

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
	fclose(fpLog);
#endif

	return pNewNode - pFirstNewNode;
}

/*DEPRECATED Cvoid RVLCalculateRGBHist(CRVLMPtrChain *pTriangleList,
						 IplImage* rgbImg,
						 int *pProjected,
						 int w, int h)
{
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;
	int r = 0, g = 0, b = 0, rgb_x = 0, rgb_y = 0;
	ushort *histRGB16 = new ushort[4096];
	uchar *ptr;
	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		//ushort *histRGB16 = new ushort[4096];
		//pTriangle->m_histRGB16 = new ushort[4096];	
		memset(histRGB16, 0, 4096 * sizeof(ushort));

		pTriangle->m_histRGB16 = new RVLQLIST[1];
		RVLQLIST_INIT(pTriangle->m_histRGB16);

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			//Provjeriti
			rgb_y = pProjected[iPix] / w;
			rgb_x = pProjected[iPix] - rgb_y * w;
			if (rgb_y != 0)
				rgb_x --;
			ptr = (uchar*)(rgbImg->imageData + rgb_y * rgbImg->widthStep);
			if (rgbImg->channelSeq == "BGR")
			{
				b = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				r = ptr[rgb_x * 3 + 2];
			}
			else
			{
				r = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				b = ptr[rgb_x * 3 + 2];
			}

			histRGB16[(int)(floor(r / 16.0) * 16 * 16 + floor(g / 16.0) * 16 + floor(b / 16.0))]++;	// i = r/16 * 16^2 + g/16 * 16 + b/16
			
		}

		for(int i = 0; i < 4096; i++)
		{
			if (histRGB16[i] != 0)
			{
				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
				pHistEntry->adr = i;
				pHistEntry->value = histRGB16[i];
				RVLQLIST_ADD_ENTRY(pTriangle->m_histRGB16, pHistEntry);
			}
		}
	}
}*/

void RVLCalculateKinectProjections(int *depthMap,
						 int *pProjected,
						 int w, int h)
{
	//KINECT constants (devided by 2 because of subsampled resolution of 320x240)
	double depthI11 = 582.26972991070841/2; //fNrm
	double depthI13 = 333.78575299556587/2;  //CenterXNrm
	double depthI22 = 562.98284310455313/2;  //fvNrm
	double depthI23 = 238.07133397745864/2;	//CenterYNrm

	double rgbI11 = 521.14532308239393/2;
	double rgbI13 = 311.66743557620077/2;
	double rgbI22 = 516.65646258458503/2;
	double rgbI23 = 259.25911293440481/2;

	double rotMat[9];
	rotMat[0] = 0.99899332935846452;
	rotMat[1] = -0.0015284464595360591;
	rotMat[2] = 0.044832931520376568;
	rotMat[3] = 0.0027173126649378660;
	rotMat[4] = 0.99964594791272010;
	rotMat[5] = -0.026468755799252012;
	rotMat[6] = -0.044776602251303213;
	rotMat[7] = 0.026563935572497543;
	rotMat[8] = 0.99864378695194855;

	CvMat *dMatRot = cvCreateMatHeader(3, 3, CV_64FC1);
	dMatRot->data.db = rotMat;

	double transMat[3]; 
	transMat[0] =  0.026709940276080747;
    transMat[0] = -0.0056095917183017910; 
    transMat[0] = -0.0070003194334035236; 

	double D3D[3], temp3D[3];
	
	CvMat *dMatTemp3D = cvCreateMatHeader(3, 1, CV_64FC1);
	dMatTemp3D->data.db = temp3D;
	
	double C3D[3];
	CvMat *dMat3DRGBData = cvCreateMatHeader(3, 1, CV_64FC1);
	dMat3DRGBData->data.db = C3D;
	int rgbPoint[2];

	for (int j = 0; j < h; j++)
	{
		for (int i = 0; i < w; i++)
		{
			if (depthMap[j * w + i] == 2047)
				continue;
			//D3D[2] = 0.1236 * tan((depthMap[j * w + i]/2842.5) + 1.1863);
			D3D[2] = 123.6 * tan((depthMap[j * w + i]/2842.5) + 1.1863);
			D3D[0] = (i - depthI13) * (D3D[2]/depthI11);
			D3D[1] = (j - depthI23) * (D3D[2]/depthI22);

			RVLDif3D(D3D,transMat,temp3D);
			cvMatMul(dMatRot,dMatTemp3D,dMat3DRGBData);
			rgbPoint[0] = (int)((C3D[0] * rgbI11) / C3D[2] + rgbI13);
			rgbPoint[1] = (int)((C3D[1] * rgbI22) / C3D[2] + rgbI23);
			if ((rgbPoint[0] > 0) && (rgbPoint[1] > 0) && (rgbPoint[0] < w) && (rgbPoint[1] < h))
				pProjected[j * w + i] = rgbPoint[1] * w + rgbPoint[0];

		}
	}
}

void RVLCalculateRGBHOG(CRVLMPtrChain *pTriangleList,
						 IplImage* rgbImg,
						 int *pProjected)
{
	//Pronalazak gradienta i smjerova
	CvMat *r = cvCreateMat(rgbImg->height, rgbImg->width, CV_8UC1);
	CvMat *g = cvCreateMat(rgbImg->height, rgbImg->width, CV_8UC1);
	CvMat *b = cvCreateMat(rgbImg->height, rgbImg->width, CV_8UC1);
	cvSplit(rgbImg, r, g, b, NULL);

	//CvMat *r_x = cvCreateMat(rgbImg->height, rgbImg->width, CV_32FC1);
	//CvMat *r_y = cvCreateMat(rgbImg->height, rgbImg->width, CV_32FC1);
	//CvMat *g_x = cvCreateMat(rgbImg->height, rgbImg->width, CV_32FC1);
	//CvMat *g_y = cvCreateMat(rgbImg->height, rgbImg->width, CV_32FC1);
	//CvMat *b_x = cvCreateMat(rgbImg->height, rgbImg->width, CV_32FC1);
	//CvMat *b_y = cvCreateMat(rgbImg->height, rgbImg->width, CV_32FC1);

	float *r_xData = new float[rgbImg->height * rgbImg->width];
	memset(r_xData, 0.0, rgbImg->height * rgbImg->width * sizeof(float));
	float *r_yData = new float[rgbImg->height * rgbImg->width];
	memset(r_yData, 0.0, rgbImg->height * rgbImg->width * sizeof(float));
	float *g_xData = new float[rgbImg->height * rgbImg->width];
	memset(g_xData, 0.0, rgbImg->height * rgbImg->width * sizeof(float));
	float *g_yData = new float[rgbImg->height * rgbImg->width];
	memset(g_yData, 0.0, rgbImg->height * rgbImg->width * sizeof(float));
	float *b_xData = new float[rgbImg->height * rgbImg->width];
	memset(b_xData, 0.0, rgbImg->height * rgbImg->width * sizeof(float));
	float *b_yData = new float[rgbImg->height * rgbImg->width];
	memset(b_yData, 0.0, rgbImg->height * rgbImg->width * sizeof(float));
	CvMat r_x, r_y, g_x, g_y, b_x, b_y;
	cvInitMatHeader(&r_x, rgbImg->height, rgbImg->width, CV_32FC1, r_xData);
	cvInitMatHeader(&r_y, rgbImg->height, rgbImg->width, CV_32FC1, r_yData);
	cvInitMatHeader(&g_x, rgbImg->height, rgbImg->width, CV_32FC1, g_xData);
	cvInitMatHeader(&g_y, rgbImg->height, rgbImg->width, CV_32FC1, g_yData);
	cvInitMatHeader(&b_x, rgbImg->height, rgbImg->width, CV_32FC1, b_xData);
	cvInitMatHeader(&b_y, rgbImg->height, rgbImg->width, CV_32FC1, b_yData);

	cvSobel(r, &r_x, 1, 0, 1);
	cvSobel(r, &r_y, 0, 1, 1);
	cvSobel(g, &g_x, 1, 0, 1);
	cvSobel(g, &g_y, 0, 1, 1);
	cvSobel(b, &b_x, 1, 0, 1);
	cvSobel(b, &b_y, 0, 1, 1);
	//pomocne varijable
	float *angle_r = new float[rgbImg->height * rgbImg->width];
	memset(angle_r, 0, rgbImg->height * rgbImg->width * sizeof(float));
	float *magnitude_r = new float[rgbImg->height * rgbImg->width];
	memset(magnitude_r, 0, rgbImg->height * rgbImg->width * sizeof(float));
	float *angle_g = new float[rgbImg->height * rgbImg->width];
	memset(angle_g, 0, rgbImg->height * rgbImg->width * sizeof(float));
	float *magnitude_g = new float[rgbImg->height * rgbImg->width];
	memset(magnitude_g, 0, rgbImg->height * rgbImg->width * sizeof(float));
	float *angle_b = new float[rgbImg->height * rgbImg->width];
	memset(angle_b, 0, rgbImg->height * rgbImg->width * sizeof(float));
	float *magnitude_b = new float[rgbImg->height * rgbImg->width];
	memset(magnitude_b, 0, rgbImg->height * rgbImg->width * sizeof(float));
	bool *map = new bool[rgbImg->height * rgbImg->width];
	memset(map, 0, rgbImg->height * rgbImg->width * sizeof(bool));
	float ang3x3_r[9];
	float mag3x3_r[9];
	float ang3x3_g[9];
	float mag3x3_g[9];
	float ang3x3_b[9];
	float mag3x3_b[9];
	float pixHOG_r[9];
	float pixHOG_g[9];
	float pixHOG_b[9];
	int adr = 0;
	int rgbOrient[3];
	float binSize = PI / 9;
	float pos;

	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;
	ushort *rgbHOG9 = new ushort[729];

	uchar *ptr;
	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		//pTriangle->m_rgbHOG9 = new ushort[729];	
		memset(rgbHOG9, 0, 729 * sizeof(ushort));
		pTriangle->m_rgbHOG9 = new RVLQLIST[1];
		RVLQLIST_INIT(pTriangle->m_rgbHOG9);

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			memset(pixHOG_r, 0.0, 9 * sizeof(float));
			memset(pixHOG_g, 0.0, 9 * sizeof(float));
			memset(pixHOG_b, 0.0, 9 * sizeof(float));
			//racunamo okolinu
			for (int i = 0; i < 9; i++)
			{
				//za koji piksel racunamo
				if ((i == 0) && (iPix != 0))
					adr = iPix - 320 - 1; //gornji lijevi
				else if ((i == 1) && (iPix > rgbImg->width))
					adr = iPix - 320; //gornji srednji
				else if ((i == 2) && (iPix > rgbImg->width) && ((iPix + 1) % rgbImg->width != 0))
					adr = iPix - 320 + 1; //gornji desni
				else if ((i == 3) && (iPix != rgbImg->width - 1))
					adr = iPix - 1; //srednji lijevi
				else if (i == 4)
					adr = iPix; //srednji(promatrani)
				else if ((i == 5) && ((iPix + 1) % rgbImg->width != 0))
					adr = iPix + 1; //srednji desni
				else if ((i == 6) && (iPix % rgbImg->width != 0) && ((iPix + 320) < rgbImg->width * rgbImg->height))
					adr = iPix + 320 - 1; //donji lijevi
				else if ((i == 7) && ((iPix + 320) < rgbImg->width * rgbImg->height))
					adr = iPix + 320; //donji srednji
				else if ((i == 8) && ((iPix + 320) < rgbImg->width * rgbImg->height) && (iPix != (rgbImg->width * rgbImg->height - 1)))
					adr = iPix + 320 + 1; //donji desni
				else
				{
					ang3x3_r[i] = 0.0;
					mag3x3_r[i] = 0.0;
					ang3x3_g[i] = 0.0;
					mag3x3_g[i] = 0.0;
					ang3x3_b[i] = 0.0;
					mag3x3_b[i] = 0.0;
					continue;
				}

				if (map[adr])
				{
					ang3x3_r[i] = angle_r[adr];
					mag3x3_r[i] = magnitude_r[adr];
					ang3x3_g[i] = angle_g[adr];
					mag3x3_g[i] = magnitude_g[adr];
					ang3x3_b[i] = angle_b[adr];
					mag3x3_b[i] = magnitude_b[adr];
					continue;
				}

				angle_r[adr] = atan2(r_yData[adr], r_xData[adr]);
				magnitude_r[adr] = sqrt(r_xData[adr] * r_xData[adr] + r_yData[adr] * r_yData[adr]);
				angle_g[adr] = atan2(g_yData[adr], g_xData[adr]);
				magnitude_g[adr] = sqrt(g_xData[adr] * g_xData[adr] + g_yData[adr] * g_yData[adr]);
				angle_b[adr] = atan2(b_yData[adr], b_xData[adr]);
				magnitude_b[adr] = sqrt(b_xData[adr] * b_xData[adr] + b_yData[adr] * b_yData[adr]);
				map[adr] = TRUE;
				ang3x3_r[i] = angle_r[adr];
				mag3x3_r[i] = magnitude_r[adr];
				ang3x3_g[i] = angle_g[adr];
				mag3x3_g[i] = magnitude_g[adr];
				ang3x3_b[i] = angle_b[adr];
				mag3x3_b[i] = magnitude_b[adr];
			}

			//Odredujemo kojem binu pripada orjentacija i stvaramo histogram
			for (int i = 0; i < 9; i++)
			{
				if (ang3x3_r[i] < 0)
					ang3x3_r[i] += PI;
				pos = ang3x3_r[i] / binSize;				
				pixHOG_r[(int)pos == 9 ? (int)(pos - 1) : (int)pos] += mag3x3_r[i];

				if (ang3x3_g[i] < 0)
					ang3x3_g[i] += PI;
				pos = ang3x3_g[i] / binSize;
				pixHOG_g[(int)pos == 9 ? (int)(pos - 1) : (int)pos] += mag3x3_g[i];

				if (ang3x3_b[i] < 0)
					ang3x3_b[i] += PI;
				pos = ang3x3_b[i] / binSize;
				pixHOG_b[(int)pos == 9 ? (int)(pos - 1) : (int)pos] += mag3x3_b[i];
			}
			Max(pixHOG_r, 9, rgbOrient[0]);
			Max(pixHOG_g, 9, rgbOrient[1]);
			Max(pixHOG_b, 9, rgbOrient[2]);
			//povecavamo odgovarajuci bin histograma
			rgbHOG9[(int)(rgbOrient[0] * 9 * 9 + rgbOrient[1] * 9 + rgbOrient[2])]++;	// i = r_orientation/16 * 16^2 + g_orientation/16 * 16 + b_orientation/16
		}

		for(int i = 0; i < 729; i++)
		{
			if (rgbHOG9[i] != 0)
			{
				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
				pHistEntry->adr = i;
				pHistEntry->value = rgbHOG9[i];
				RVLQLIST_ADD_ENTRY(pTriangle->m_rgbHOG9, pHistEntry);
			}
		}
	}
	delete [] angle_r;
	delete [] magnitude_r;
	delete [] angle_g;
	delete [] magnitude_g;
	delete [] angle_b;
	delete [] magnitude_b;
	delete [] map;
	cvReleaseMat(&r);
	cvReleaseMat(&g);
	cvReleaseMat(&b);
	delete [] r_xData;
	delete [] r_yData;
	delete [] g_xData;
	delete [] g_yData;
	delete [] b_xData;
	delete [] b_yData;
}

void RVLCalculateCentroidDescriptor(RVL3DPOINT2 **Point3DMap, CRVLClass *p2DRegionSet, float *eigs, float *minD, float *maxD)
{
	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);
	CRVL2DRegion2 *pPolygon, *pPolygonNext;
	RVLMESH_LINK *pLink0, *pLink, *pLinkNext;
	int iPix;
	RVL3DPOINT2 *pPt;
	p2DRegionList->Start();

	int noTriangles = p2DRegionList->m_nElements;
	double xyz_centroid[3];

	//KINECT constants (devided by 2 because of subsampled resolution of 320x240)
	double depthI11 = 582.26972991070841/2;
	double depthI13 = 333.78575299556587/2;
	double depthI22 = 562.98284310455313/2;
	double depthI23 = 238.07133397745864/2;
	double *vertXYZ = new double[noTriangles + 2];
	memset(vertXYZ, 0, (noTriangles + 2) * sizeof(double));
	int br = 0;
	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());
		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;
		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		do
		{
			iPix = pLink->iPix0;			
			pPolygonNext = (CRVL2DRegion2 *)(pLink->vp2DRegion);
			//racunamo X, Y, Z
			//vertXYZ[br * 3 + 2] = 0.1236 * tan((Point3DMap[iPix]->d/2842.5) + 1.1863);
			vertXYZ[br * 3 + 2] = 123.6 * tan((Point3DMap[iPix]->d/2842.5) + 1.1863);
			vertXYZ[br * 3] = (Point3DMap[iPix]->u - depthI13) * (vertXYZ[br * 3 + 2]/depthI11);
			vertXYZ[br * 3 + 1] = (Point3DMap[iPix]->v - depthI23) * (vertXYZ[br * 3 + 2]/depthI22);
			xyz_centroid[0] += vertXYZ[br * 3];
			xyz_centroid[1] += vertXYZ[br * 3 + 1];
			xyz_centroid[2] += vertXYZ[br * 3 + 2];
			br++;
			pLink = pLink->pNext->pOpposite;
		}while(pLink != pLink0);
	}
	xyz_centroid[0] /= (noTriangles + 2);
	xyz_centroid[1] /= (noTriangles + 2);
	xyz_centroid[2] /= (noTriangles + 2);


	double demean[3], demean_xy, demean_xz, demean_yz;
	double* C = new double[9];
	memset(C, 0, 9 * sizeof(double));
	double *eig = new double[3];
	memset(eig, 0, 3 * sizeof(double));
	double *eigV = new double[9];
	memset(eigV, 0, 9 * sizeof(double));
	p2DRegionList->Start();
	for (int i = 0; i < noTriangles + 2; i++)
	{
		demean[0] = vertXYZ[i * 3] - xyz_centroid[0];
		demean[1] = vertXYZ[i * 3 + 1] - xyz_centroid[1];
		demean[2] = vertXYZ[i * 3 + 2] - xyz_centroid[2];
		demean_xy = demean[0] * demean[1];
		demean_xz = demean[0] * demean[2];
		demean_yz = demean[1] * demean[2];
		C[0] += demean[0] * demean[0];
		C[1] += demean_xy;
		C[2] += demean_xz;
		C[3] += demean_xy;
		C[4] += demean[1] * demean[1];
		C[5] += demean_yz;
		C[6] += demean_xz;
		C[7] += demean_yz;
		C[8] += demean[2] * demean[2];
	}
	
	RVLGetEigVects3(C, eig, eigV);
	double maxEig = eig[0], minEig=eig[0];
	int iM = 0, im = 0;
	if (maxEig < eig[1])
	{
		maxEig = eig[1];
		iM = 1;
	}
	else
	{
		minEig = eig[1];
		im = 1;
	}
	if (maxEig < eig[2])
	{
		maxEig = eig[2];
		iM = 2;
	}
	else if(minEig > eig[2])
	{
		minEig = eig[2];
		im = 2;
	}
	eigs[0] = eig[iM];
	eigs[1] = eig[im];
	maxD[0] = eigV[3*iM];
	maxD[1] = eigV[3*iM + 1];
	maxD[2] = eigV[3*iM + 2];
	minD[0] = eigV[3*im];
	minD[1] = eigV[3*im + 1];
	minD[2] = eigV[3*im + 2];
	//brisemo varijable
	delete [] C;
	delete [] eig;
	delete [] eigV;
	delete [] vertXYZ;
}

void RVLCalculateD2ShapeDistribution(RVL3DPOINT2 **Point3DMap, CRVLClass *p2DRegionSet, RVLQLIST *distribution)
{
	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);
	CRVL2DRegion2 *pPolygon, *pPolygonNext;
	RVLMESH_LINK *pLink0, *pLink, *pLinkNext;
	int iPix;
	RVL3DPOINT2 *pPt;
	p2DRegionList->Start();

	int noTriangles = p2DRegionList->m_nElements;

	//KINECT constants (devided by 2 because of subsampled resolution of 320x240)
	double depthI11 = 582.26972991070841/2;
	double depthI13 = 333.78575299556587/2;
	double depthI22 = 562.98284310455313/2;
	double depthI23 = 238.07133397745864/2;
	double *vertXYZ = new double[noTriangles + 2];
	memset(vertXYZ, 0, (noTriangles + 2) * sizeof(double));
	int noLen = ((noTriangles + 2) * (noTriangles + 2)) / 2 - (noTriangles + 2);
	double *dist = new double[noLen];
	memset(dist, 0, noLen * sizeof(double));
	int br = 0;
	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());
		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;
		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		do
		{
			iPix = pLink->iPix0;			
			pPolygonNext = (CRVL2DRegion2 *)(pLink->vp2DRegion);
			//racunamo X, Y, Z
			//vertXYZ[br * 3 + 2] = 0.1236 * tan((Point3DMap[iPix]->d/2842.5) + 1.1863);
			vertXYZ[br * 3 + 2] = 123.6 * tan((Point3DMap[iPix]->d/2842.5) + 1.1863);
			vertXYZ[br * 3] = (Point3DMap[iPix]->u - depthI13) * (vertXYZ[br * 3 + 2]/depthI11);
			vertXYZ[br * 3 + 1] = (Point3DMap[iPix]->v - depthI23) * (vertXYZ[br * 3 + 2]/depthI22);
			br++;
			pLink = pLink->pNext->pOpposite;
		}while(pLink != pLink0);
	}
	//racunamo udaljenosti
	double temp[3];
	br = 0;
	for (int i = 0; i < (noTriangles + 2); i++)
	{
		for (int j = 0; j < i; j++)
		{
			RVLDif3D(&vertXYZ[i * 3], &vertXYZ[j * 3], temp);
			dist[br] = RVLGet3DVectorLen(temp);
			br++;
		}
	}
	double maxLen;
	maxLen = Max(dist, noLen);
	double binSize = maxLen / 512;
	//pronalazimo distribuciju/histogram
	int hist[512];
	memset(hist, 0, 512 * sizeof(ushort));
	int pos = 0;
	for (int i = 0; i < noLen; i++)
	{
		pos = dist[i] / binSize;
		hist[(int)pos == 512 ? (int)(pos - 1) : (int)pos]++;
	}
	RVLQLIST_DISTRIBUTION_ENTRY *pHistNULLEntry = new RVLQLIST_DISTRIBUTION_ENTRY[1];
	pHistNULLEntry->adr = -1;
	pHistNULLEntry->value = maxLen;
	RVLQLIST_ADD_ENTRY(distribution, pHistNULLEntry);
	maxLen = (double)Max(hist, 512);
	for (int i = 0; i < 512; i++)
	{
		if (hist[i] != 0)
		{
			RVLQLIST_DISTRIBUTION_ENTRY *pHistEntry = new RVLQLIST_DISTRIBUTION_ENTRY[1];
			pHistEntry->adr = i;
			pHistEntry->value = hist[i] / maxLen;
			RVLQLIST_ADD_ENTRY(distribution, pHistEntry);
		}
	}
	
	//brisemo varijable
}

void RVLSetUsefulPixMask(CRVLClass *p2DRegionSet,
						RVL3DPOINT2 **Point3DMap,
						int w, int h,
						CRVLMem *pMem,
						int border,
						BYTE *uPixMask,
						BYTE label,
						DWORD Flags,
						DWORD Mask)
{
	IplImage *mask = cvCreateImage(cvSize(w, h), 8, 1);
	memset(mask->imageData, 0, w*h*sizeof(uchar));
	//BYTE *uPixMask;// = new bool[w * h];	//Preko Mem-a?!?!?!
	if (!uPixMask)
	{
		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, BYTE, w * h, uPixMask)
		memset(uPixMask, 0, w * h * sizeof(BYTE));
	}

	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);
	CRVL2DRegion2 *pPolygon0, *pPolygon1, *pPolygon2, *pPolygon3;
	RVLMESH_LINK *pLink0, *pLink1, *pLink2, *pLink3;
	CvPoint P1, P2;
	P1 = cvPoint(0, 0);
	P2 = cvPoint(0, 0);

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pPolygon0 = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(pPolygon0->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pPolygon0->m_Flags & Mask) != Mask)
			continue;
		//Postavljamo varijablu
		pPolygon0->m_uPixMask = uPixMask;
		//Dohvacamo sve susjedne trokute a time i tocke
		pLink0 = (RVLMESH_LINK *)(pPolygon0->m_PtArray);
		/*DELAUNAY TRIANGULATIONpLink1 = pLink0;
		do
		{
			//obilazimo trokut
			P1.x = Point3DMap[pLink1->iPix0]->u;
			P1.y = Point3DMap[pLink1->iPix0]->v;
			P2.x = Point3DMap[pLink1->pOpposite->iPix0]->u;
			P2.y = Point3DMap[pLink1->pOpposite->iPix0]->v;
			cvLine(mask, P1, P2, CV_RGB(label,label,label), border);
		 	pLink1 = pLink1->pNext->pOpposite;
		}
		while(pLink1 != pLink0);*/
		pLink1 = pLink0->pNext;
		pPolygon1 = (CRVL2DRegion2 *)pLink1->vp2DRegion;
		pLink2 = pLink1->pOpposite->pNext;
		pPolygon2 = (CRVL2DRegion2 *)pLink2->vp2DRegion;
		pLink3 = pLink0->pOpposite;
		pPolygon3 = (CRVL2DRegion2 *)pLink3->vp2DRegion;
		//Provjeravamo trokute odnosno granice
		//1,2
		if ((!pPolygon1) || (pPolygon1->m_Label != pPolygon0->m_Label) || (pPolygon1->m_Flags & RVLOBJ2_FLAG_REJECTED) || ((pPolygon1->m_Flags & Mask) != Mask))
		{
			P1.x = Point3DMap[pLink1->iPix0]->u;
			P1.y = Point3DMap[pLink1->iPix0]->v;
			P2.x = Point3DMap[pLink2->iPix0]->u;
			P2.y = Point3DMap[pLink2->iPix0]->v;
			//cvLine(mask, P1, P2, CV_RGB(4,4,4), border + 4);	//Zbog LBP R = 3
			//cvLine(mask, P1, P2, CV_RGB(3,3,3), border + 3);	//Zbog LBP R = 2
			//cvLine(mask, P1, P2, CV_RGB(2,2,2), border + 2);	//Zbog LBP R = 1
			cvLine(mask, P1, P2, CV_RGB(label,label,label), border);
		}
		//2,3
		if ((!pPolygon2) || (pPolygon2->m_Label != pPolygon0->m_Label) || (pPolygon2->m_Flags & RVLOBJ2_FLAG_REJECTED) || ((pPolygon2->m_Flags & Mask) != Mask))
		{
			P1.x = Point3DMap[pLink2->iPix0]->u;
			P1.y = Point3DMap[pLink2->iPix0]->v;
			P2.x = Point3DMap[pLink3->iPix0]->u;
			P2.y = Point3DMap[pLink3->iPix0]->v;
			//cvLine(mask, P1, P2, CV_RGB(4,4,4), border + 4);	//Zbog LBP R = 3
			//cvLine(mask, P1, P2, CV_RGB(3,3,3), border + 3);	//Zbog LBP R = 2
			//cvLine(mask, P1, P2, CV_RGB(2,2,2), border + 2);	//Zbog LBP R = 1
			cvLine(mask, P1, P2, CV_RGB(label,label,label), border);
		}
		//3,0
		if ((!pPolygon3) || (pPolygon3->m_Label != pPolygon0->m_Label) || (pPolygon3->m_Flags & RVLOBJ2_FLAG_REJECTED) || ((pPolygon3->m_Flags & Mask) != Mask))
		{
			P1.x = Point3DMap[pLink3->iPix0]->u;
			P1.y = Point3DMap[pLink3->iPix0]->v;
			P2.x = Point3DMap[pLink0->iPix0]->u;
			P2.y = Point3DMap[pLink0->iPix0]->v;
			//cvLine(mask, P1, P2, CV_RGB(4,4,4), border + 4);	//Zbog LBP R = 3
			//cvLine(mask, P1, P2, CV_RGB(3,3,3), border + 3);	//Zbog LBP R = 2
			//cvLine(mask, P1, P2, CV_RGB(2,2,2), border + 2);	//Zbog LBP R = 1
			cvLine(mask, P1, P2, CV_RGB(label,label,label), border);
		}	
	}
	//Zapisujemo binarnu sliku kao scanline buffer
	uchar *ptr;
	int s = 0;
	for (int j = 0; j < mask->height; j++)
	{
		ptr = (uchar*)(mask->imageData + j * mask->widthStep);
		for (int i = 0; i < mask->width; i++)
		{
			/*if (ptr[i] == 1)
				uPixMask[s] = 1;
			else if (ptr[i] == 2)
				uPixMask[s] = 2;
			else if (ptr[i] == 3)
				uPixMask[s] = 3;
			else if (ptr[i] == 4)
				uPixMask[s] = 4;*/
			if (ptr[i] > 0)
				uPixMask[s] = ptr[i];
			s++;
		}
	}
	//cvSaveImage("C:\\Users\\Damir\\Documents\\border.bmp", mask);
	cvReleaseImage(&mask);
}

void RVLGetAvgNeighborhoodIntensity(CRVLMPtrChain *p2DRegionList,
									RVLAPIX_2DREGION_PTR *Region2DMap, 
									int Width, int Height)
{
	// reset edge no. and sum of neighboring intensities

	CRVL2DRegion2 *p2DRegion;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		p2DRegion->m_nEdges = 0;

		p2DRegion->m_SINeighborhood = 0;
	}

	// compute average intensity of neighboring regions along the edge of every region

	int u, v;
	RVLAPIX_2DREGION_PTR *pRegionPtr;
	CRVL2DRegion2 *p2DRegion1, *p2DRegion2;
	int I1, I2;

	for(v = 0; v < Height; v++)
		for(u = 1; u < Width; u++)
		{
			pRegionPtr = Region2DMap + u + v*Width;

			p2DRegion1 = (pRegionPtr - 1)->p2DRegion;

			p2DRegion2 = pRegionPtr->p2DRegion;

			if(p2DRegion2 == p2DRegion1)
				continue;

			I1 = p2DRegion1->m_SI / p2DRegion1->m_nPts;
			I2 = p2DRegion2->m_SI / p2DRegion2->m_nPts;

			p2DRegion1->m_nEdges++;
			p2DRegion2->m_nEdges++;
			p2DRegion1->m_SINeighborhood += I2;
			p2DRegion2->m_SINeighborhood += I1;
		}

	for(v = 1; v < Height; v++)
		for(u = 0; u < Width; u++)
		{
			pRegionPtr = Region2DMap + u + v*Width;

			p2DRegion1 = (pRegionPtr - Width)->p2DRegion;

			p2DRegion2 = pRegionPtr->p2DRegion;

			if(p2DRegion2 == p2DRegion1)
				continue;

			I1 = p2DRegion1->m_SI / p2DRegion1->m_nPts;
			I2 = p2DRegion2->m_SI / p2DRegion2->m_nPts;

			p2DRegion1->m_nEdges++;
			p2DRegion2->m_nEdges++;
			p2DRegion1->m_SINeighborhood += I2;
			p2DRegion2->m_SINeighborhood += I1;
		}
}

void RVLGetAvgColorOf2DRegions(	CRVLMPtrChain *p2DRegionList,
								RVLAPIX_2DREGION_PTR *Region2DMap, 
								unsigned char *PixArray,
								int ImageSize,
								BOOL bColor)
{
	// reset pt. no. and intensity sum of regions

	CRVL2DRegion2 *p2DRegion;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		p2DRegion->m_nPts = 0;

		p2DRegion->m_SI = 0;

		p2DRegion->m_Sr = p2DRegion->m_Sg = p2DRegion->m_Sb = 0;
	}

	// compute pt. no. and intensity sum of regions

	unsigned char *pPix = PixArray;

	RVLAPIX_2DREGION_PTR *pRegion2DMapEnd = Region2DMap + ImageSize;

	unsigned char I, r, g, b;
	RVLAPIX_2DREGION_PTR *pRegionPtrList, *pRegionPtr;

	for(pRegionPtrList = Region2DMap; pRegionPtrList < pRegion2DMapEnd; pRegionPtrList++)
	{
		pRegionPtr = pRegionPtrList;

		if(bColor)
		{
			b = *(pPix++);
			g = *(pPix++);
			r = *(pPix++);
		}
		else
			I = *(pPix++);
		
		while(pRegionPtr)
		{
			p2DRegion = pRegionPtr->p2DRegion;

			p2DRegion->m_nPts++;

			if(bColor)
			{
				p2DRegion->m_Sr += r;
				p2DRegion->m_Sg += g;
				p2DRegion->m_Sb += b;
			}
			else
				p2DRegion->m_SI += (int)I;

			pRegionPtr = pRegionPtr->pNext;
		}
	}
}

void RVLDisplaySegmentationToConvex(CRVLMPtrChain *p2DRegionList,
									RVLAPIX_2DREGION_PTR *Region2DMap, 
									int ImageSize,
									BOOL bColor,
									PIX_ARRAY *pOutPixArray)
{
	unsigned char *pPix = pOutPixArray->pPix;

	RVLAPIX_2DREGION_PTR *pRegion2DMapEnd = Region2DMap + ImageSize;

	int nPts;
	CRVL2DRegion2 *pLargest2DRegion;
	RVLAPIX_2DREGION_PTR *pRegionPtrList, *pRegionPtr;
	unsigned char I;
	CRVL2DRegion2 *p2DRegion;

	for(pRegionPtrList = Region2DMap; pRegionPtrList < pRegion2DMapEnd; pRegionPtrList++)
	{
		nPts = 0;

		pRegionPtr = pRegionPtrList;

		while(pRegionPtr)
		{
			p2DRegion = pRegionPtr->p2DRegion;

			if(p2DRegion->m_nPts > nPts)
			{
				nPts = p2DRegion->m_nPts;

				pLargest2DRegion = p2DRegion;
			}

			pRegionPtr = pRegionPtr->pNext;
		}

		if(bColor)
		{
			*(pPix++) = (unsigned char)((2 * pLargest2DRegion->m_Sr + nPts) / (2 * nPts));
			*(pPix++) = (unsigned char)((2 * pLargest2DRegion->m_Sg + nPts) / (2 * nPts));
			*(pPix++) = (unsigned char)((2 * pLargest2DRegion->m_Sb + nPts) / (2 * nPts));			
		}
		else
		{
			I = (unsigned char)((2 * pLargest2DRegion->m_SI + nPts) / (2 * nPts));

			*(pPix++) = I;
			*(pPix++) = I;
			*(pPix++) = I;
		}
	}

}

void RVLGetMomentsOf2DRegions(	CRVLMPtrChain *p2DRegionList,
								RVLAPIX_2DREGION_PTR *Region2DMap,
								int ImageWidth, int ImageHeight)
{
	// reset moments

	CRVL2DRegion2 *p2DRegion;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		p2DRegion->m_Moments.n = 0;
		p2DRegion->m_Moments.S[0] = p2DRegion->m_Moments.S[1] = 0;
		p2DRegion->m_Moments.S2[0] = p2DRegion->m_Moments.S2[1] = p2DRegion->m_Moments.S2[3] = 0;
	}

	// compute moments

	RVLAPIX_2DREGION_PTR *pRegionPtrList = Region2DMap;

	RVLAPIX_2DREGION_PTR *pRegionPtr;
	int u, v;
	RVL2DMOMENTS *pMoments;

	for(v = 0; v < ImageHeight; v++)
		for(u = 0; u < ImageWidth; u++, pRegionPtrList++)
		{
			pRegionPtr = pRegionPtrList;

			while(pRegionPtr)
			{
				p2DRegion = pRegionPtr->p2DRegion;

				pMoments = &(p2DRegion->m_Moments);

				pMoments->n++;

				pMoments->S[0] += u;
				pMoments->S[1] += v;
				pMoments->S2[0] += u * u;
				pMoments->S2[1] += u * v;
				pMoments->S2[3] += v * v;

				pRegionPtr = pRegionPtr->pNext;
			}
		}

	double M[2];
	double C[2 * 2];

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		pMoments = &(p2DRegion->m_Moments);

		RVLGetCovMatrix2(pMoments, C, M);

		pMoments->S[0] = M[0];
		pMoments->S[1] = M[1];
		pMoments->S2[0] = C[0];
		pMoments->S2[1] = C[1];
		pMoments->S2[3] = C[3];
	}
}

bool RVL3DMeshIsConvex(CRVLMPtrChain *pTriangleList,
					   RVL3DPOINT2 **Point3DMap)
{
	RVLPTRCHAIN_ELEMENT *pNext;	
	CRVL2DRegion2 *pTriangle2;
	RVLMESH_LINK *pLink, *pLink0;
	//double *X, *N;
	//double rho, e;
	int *X, *N, *X0;
	int dX[3];
	int64 e;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		CRVL2DRegion2 *pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pNext = pTriangleList->m_pNext;

		//N = pTriangle->m_fN;

		N = pTriangle->m_N;

		//rho = pTriangle->m_rho;

		X0 = pTriangle->m_X0;

		pTriangleList->Start();

		while(pTriangleList->m_pNext)
		{
			CRVL2DRegion2 *pTriangle2 = (CRVL2DRegion2 *)(pTriangleList->GetNext());

			if(pTriangle2->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			pLink = pLink0 = (RVLMESH_LINK *)(pTriangle2->m_PtArray);

			do
			{
				//X = Point3DMap[pLink->iPix0]->XYZ;
				X = Point3DMap[pLink->iPix0]->iX;

				//e = RVLDOTPRODUCT3(X, N) - rho;

				RVLDIF3VECTORS(X, X0, dX)

				e = RVLDOTPRODUCT3_64(N, dX);

				if(e > 0)
					return false;

				pLink = pLink->pNext->pOpposite;
			}
			while(pLink != pLink0);
		}

		pTriangleList->m_pNext = pNext;
	}

	return true;
}

// Segmentation of RGB images using the graph segmentation method proposed in felzenszwalb_IJCV04

int RVLRGBGraphSegmentationQSortCompare(const void * a, const void * b)
{
	float diff = (*((RVLSWER_LINK2 **)a))->cost - (*((RVLSWER_LINK2 **)b))->cost;

	return (diff > 0.0f ? 1 : (diff < 0.0f ? -1 : 0));
}

void RVLRGBGraphSegmentation(
	unsigned char *RGB,
	int w,
	int h,
	int NeighborhoodSize,
	float k,
	RVLSWER_NODE2 *Node,
	RVLARRAY_<RVLSWER_SEGMENT<RVLSWER_NODE2>> &SegmentArray,
	int *PtMem)
{
	// create a graph from the input RGB image

	int nPts = w * h;

	RVLSWER_NODE2 *pNode = Node;

	RVLSWER_LINK2 *Link = new RVLSWER_LINK2[nPts * NeighborhoodSize * NeighborhoodSize];

	RVLSWER_LINK2 *pLink = Link;

	RVLSWER_LINK2 **SortedLinkList = new RVLSWER_LINK2 *[nPts * NeighborhoodSize * NeighborhoodSize];

	RVLSWER_LINK2 **ppLink = SortedLinkList;

	int halfNeighborhoodSize = (NeighborhoodSize - 1) / 2;

	unsigned char *pPix0 = RGB;

	int u0, v0, u, v;
	int minu, maxu, minv, maxv;
	unsigned char *pPix;
	int RGB0[3], RGB_[3], V3Tmp[3];
	int iPix0, iPix;

	for (v0 = 0; v0 < h; v0++)
	{
		minv = v0 - halfNeighborhoodSize;

		if (minv < 0)
			minv = 0;

		maxv = v0 + halfNeighborhoodSize;

		if (maxv >= h)
			maxv = h - 1;

		for (u0 = 0; u0 < w; u0++)
		{
			iPix0 = u0 + v0 * w;

			pNode->pChild[0] = NULL;
			pNode->pChild[1] = NULL;
			pNode->pParent = NULL;
			pNode->Size = 1;
			pNode->cost = 0;

			RGB0[0] = (int)(*(pPix0++));
			RGB0[1] = (int)(*(pPix0++));
			RGB0[2] = (int)(*(pPix0++));

			minu = u0 - halfNeighborhoodSize;

			if (minu < 0)
				minu = 0;

			maxu = u0 + halfNeighborhoodSize;

			if (maxu >= w)
				maxu = w - 1;

			for (v = minv; v <= maxv; v++)
			{
				for (u = minu; u <= maxu; u++)
				{
					if (u == u0 && v == v0)
						continue;

					iPix = u + v * w;

					pPix = RGB + 3 * iPix;

					RGB_[0] = (int)(*(pPix++));
					RGB_[1] = (int)(*(pPix++));
					RGB_[2] = (int)(*pPix);

					RVLDIF3VECTORS(RGB_, RGB0, V3Tmp);

					pLink->cost = sqrt((float)(RVLDOTPRODUCT3(V3Tmp, V3Tmp)));
					pLink->pNode[0] = Node + iPix0;
					pLink->pNode[1] = Node + iPix;

					*(ppLink++) = pLink;

					pLink++;
				}
			}

			pNode++;
		}
	}

	// Create sorted link list

	int nLinks = pLink - Link;

	//RVLBubbleSort<RVLSWER_LINK2>(SortedLinkList, nLinks);

	//short *Key = new short[nLinks];

	//int i;

	//for (i = 0; i < nLinks; i++)
	//	Key[i] = (short)(10.0 * Link[i].cost);

	//int *Index = new int[nLinks];

	//RVLQuickSort(Key, Index, nLinks);

	//delete[] Key;

	//for (i = 0; i < nLinks; i++)
	//	SortedLinkList[i] = Link + Index[i];

	//delete[] Index;

	int i;

	for (i = 0; i < nLinks; i++)
		SortedLinkList[i] = Link + i;

	qsort(SortedLinkList, nLinks, sizeof(RVLSWER_LINK2 *), RVLRGBGraphSegmentationQSortCompare);

	// Hierarchical segmentation

	int nNodes;

	RVLGraphSegmentationFH(Node, nPts, SortedLinkList, nLinks, k, nNodes, SegmentArray, PtMem);

	// deallocate memory

	delete[] Link;
	delete[] SortedLinkList;
}

void RVLCreateHSHistograms(
	CRVLMPtrChain *pTriangleList,
	int nObjects,
	IplImage* pRGBImg,
	int satThr,
	int *&HSVHist,
	int *&nSamples,
	int *&nPts)
{
	IplImage *pHSVImg = cvCreateImage(cvSize(pRGBImg->width, pRGBImg->height), IPL_DEPTH_8U, 3);

	cvCvtColor(pRGBImg, pHSVImg, CV_RGB2HSV);

	uchar *HSV = (uchar *)(pHSVImg->imageData);

	HSVHist = new int[(2 * 181 + 1) * nObjects];

	memset(HSVHist, 0, (2 * 181 + 1) * nObjects * sizeof(int));

	nSamples = HSVHist + 181 * nObjects;

	nPts = nSamples + 181 * nObjects;

	int i;
	CRVL2DRegion2 *pTriangle;
	int iPix;
	uchar *HSV_;
	int *HSVHist_, *nSamples_;
	//int sat;

	pTriangleList->Start();

	while (pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if (pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if (pTriangle->m_Label == 16)
			int debug = 0;

		nPts[pTriangle->m_Label] += pTriangle->m_n3DPts;

		for (i = 0; i < pTriangle->m_n3DPts; i++)
		{
			iPix = pTriangle->m_pPoint3DArray[i]->iPix;

			HSV_ = HSV + 3 * iPix;

			HSVHist_ = HSVHist + 181 * pTriangle->m_Label;
			nSamples_ = nSamples + 181 * pTriangle->m_Label;

			if (HSV_[1] < satThr)
				continue;

			//sat = HSV_[1] - 100;

			//if (sat < 0)
			//	sat = 0;

			HSVHist_[HSV_[0]] += (int)(HSV_[1]);
			nSamples_[HSV_[0]]++;
		}
	}

	cvReleaseImage(&pHSVImg);
}

void RVLDominantSegmentColor(
	CRVLMPtrChain *pTriangleList,
	int nObjects,
	IplImage* pRGBImg,
	uchar *&color)
{
	int colorWin = 10;

	int *HSVHist, *nSamples, *nPts;

	RVLCreateHSHistograms(pTriangleList, nObjects, pRGBImg, 100, HSVHist, nSamples, nPts);

	color = new uchar[2 * nObjects];

	int i, j, k;
	int iObject;
	int w, maxw, imaxw, iColor;
	int n;
	int *HSVHist_, *nSamples_;

	for (iObject = 0; iObject < nObjects; iObject++)
	{
		maxw = -1;

		HSVHist_ = HSVHist + 181 * iObject;
		nSamples_ = nSamples + 181 * iObject;

		for (i = 0; i <= 180; i++)
		{
			w = 0;

			for (j = -colorWin; j <= colorWin; j++)
			{			
				k = ((i + j + 181) % 181);

				w += HSVHist_[k];
			}

			if (w > maxw)
			{
				maxw = w;

				imaxw = i;
			}
		}

		w = 0;
		n = 0;
		iColor = 0;

		for (j = -colorWin; j <= colorWin; j++)
		{
			k = ((imaxw + j + 181) % 181);

			iColor += (j * HSVHist_[k]);

			w += HSVHist_[k];
			n += nSamples_[k];
		}
		
		color[2 * iObject] = (imaxw + iColor / (w == 0 ? 1 : w) + 181) % 181;
		color[2 * iObject + 1] = w / (n == 0 ? 1 : n);
	}

	delete[] HSVHist;
}

void RVLDominantSegmentColor2(
	CRVLMPtrChain *pTriangleList,
	int nObjects,
	IplImage* pRGBImg,
	uchar *&color)
{
	int colorWin = 10;

	int *HSVHist, *nSamples, *nPts;

	RVLCreateHSHistograms(pTriangleList, nObjects, pRGBImg, 50, HSVHist, nSamples, nPts);

	color = new uchar[2 * nObjects];
	int *dColor_ = new int[181];
	int *w_ = new int[181];
	int *nCluster = new int[181];
	int *wCluster = new int[181];

	int i, j, k;
	int iObject;
	int w, dColor;
	int n, maxn, sign, iDominantPeek;
	int *HSVHist_, *nSamples_;

	for (iObject = 0; iObject < nObjects; iObject++)
	{
		//if (iObject == 9)
		//	int debug = 0;

		nSamples_ = nSamples + 181 * iObject;

		for (i = 0; i <= 180; i++)
			if (nSamples_[i] > 0)
				break;

		if (i >= 181)
		{
			color[2 * iObject] = 0;
			color[2 * iObject + 1] = 0;

			continue;
		}

		HSVHist_ = HSVHist + 181 * iObject;

		maxn = 0;

		for (i = 0; i <= 180; i++)
		{
			w = 0;
			dColor = 0;

			for (j = -colorWin; j <= colorWin; j++)
			{
				k = ((i + j + 181) % 181);

				dColor += ((j + colorWin) * HSVHist_[k]);

				w += HSVHist_[k];
			}

			dColor = (w > 0 ? (2 * dColor + 1) / (2 * w) - colorWin : 0);

			dColor_[i] = dColor;

			w_[i] = w;
		}
	
		memset(nCluster, 0, 181 * sizeof(int));
		memset(wCluster, 0, 181 * sizeof(int));

		for (i = 0; i <= 180; i++)
		{
			if (w_[i] == 0)
				continue;

			sign = (dColor_[i] >= 0 ? 1 : -1);

			j = i;

			while (sign * dColor_[j] > 0)
				j = (j + dColor_[j] + 181) % 181;

			if (dColor_[j] < 0)
				j = (j + dColor_[j] + 181) % 181;

			nCluster[j] += nSamples_[i];
			wCluster[j] += HSVHist_[i];
		}

		maxn = nCluster[0];
		iDominantPeek = 0;

		for (i = 1; i <= 180; i++)
			if (nCluster[i] > maxn)
			{
				maxn = nCluster[i];
				iDominantPeek = i;
			}

		if (100 * maxn / nPts[iObject] >= 50)
		{
			color[2 * iObject] = iDominantPeek;
			color[2 * iObject + 1] = wCluster[iDominantPeek] / maxn;
		}
		else
		{
			color[2 * iObject] = 0;
			color[2 * iObject + 1] = 0;
		}
	}

	delete[] HSVHist;
	delete[] dColor_;
	delete[] w_;
	delete[] nCluster;
	delete[] wCluster;
}

// This function implements the graph segmentation method proposed in felzenszwalb_IJCV04
// Input:
//     Node      - array of graph nodes filled with input nodes (points)
//                 this array must be allocated for 2 * nInNodes elements
//     nInNodes  - number of input graph nodes
//     SortedLinkList - list of graph edges sorted according to their cost in ascending order
//     nLinks    - number of graph edges
//     k         - user defined parameter (see felzenszwalb_IJCV04)
// Output:
//     Node      - the function adds additional nodes into this array
//     nNodes    - the total number of nodes after completion of the segmentatio process
//     SegmentArray - array of detected segments
//                 this array must be allocated before calling this function for nInNodes elements
//     PtListMem - memory storage for points in point lists
//                 this array must be allocated before calling this function for nInNodes elements
// The function returns the number of segments.

void RVLGraphSegmentationFH(
	RVLSWER_NODE2 *Node,
	int nInNodes,
	RVLSWER_LINK2 **SortedLinkList,
	int nLinks,
	float k,
	int &nNodes,
	RVLARRAY_<RVLSWER_SEGMENT<RVLSWER_NODE2>> &SegmentArray,
	int *PtListMem
	)
{
	RVLSWER_NODE2 *pNode = Node + nInNodes;

	RVLSWER_LINK2 **pLinkListEnd = SortedLinkList + nLinks;

	RVLSWER_LINK2 **ppLink;
	RVLSWER_LINK2 *pLink;
	RVLSWER_NODE2 *pSegment1, *pSegment2;
	float MInt, Int1, Int2;

	for (ppLink = SortedLinkList; ppLink < pLinkListEnd; ppLink++)
	{
		pLink = *ppLink;

		pSegment1 = pLink->pNode[0];

		while (pSegment1->pParent)
			pSegment1 = pSegment1->pParent;

		pSegment2 = pLink->pNode[1];

		while (pSegment2->pParent)
			pSegment2 = pSegment2->pParent;

		if (pSegment1 == pSegment2)
			continue;

		Int1 = pSegment1->cost + k / (float)(pSegment1->Size);
		Int2 = pSegment2->cost + k / (float)(pSegment2->Size);

		MInt = RVLMIN(Int1, Int2);

		if (pLink->cost > MInt)
			continue;

		//if (pLink->cost > 10.0)
		//	continue;

		pNode->pChild[0] = pSegment1;
		pNode->pChild[1] = pSegment2;
		pNode->pParent = NULL;
		pNode->cost = RVLMAX(pSegment1->cost, pSegment2->cost);
		if (pLink->cost > pNode->cost)
			pNode->cost = pLink->cost;
		pNode->Size = pSegment1->Size + pSegment2->Size;

		pSegment1->pParent = pNode;
		pSegment2->pParent = pNode;

		pNode++;
	}

	RVLSWER_NODE2 *pNodeListEnd = pNode;

	nNodes = pNode - Node;

	// create segments

	RVLSWER_SEGMENT<RVLSWER_NODE2> *pSegment = SegmentArray.Element;

	int *piPt = PtListMem;

	RVLSWER_NODE2 **NodeBuff = new RVLSWER_NODE2 *[2 * nInNodes];

	RVLSWER_NODE2 **ppNodePut, **ppNodeFetch;
	RVLSWER_NODE2 *pNode_;

	for (pNode = Node; pNode < pNodeListEnd; pNode++)
		if (pNode->pParent == NULL)
		{
			pSegment->pNode = pNode;
			pSegment->PtList.Element = piPt;

			ppNodePut = ppNodeFetch = NodeBuff;

			*(ppNodePut++) = pNode;

			while (ppNodeFetch < ppNodePut)
			{
				pNode_ = *(ppNodeFetch++);

				if (pNode_->pChild[0])
				{
					*(ppNodePut++) = pNode_->pChild[0];
					*(ppNodePut++) = pNode_->pChild[1];
				}
				else
					*(piPt++) = pNode_ - Node;
			}

			pSegment->PtList.n = piPt - pSegment->PtList.Element;

			pSegment++;
		}

	delete[] NodeBuff;

	SegmentArray.n = pSegment - SegmentArray.Element;
}

void RVLDisplayRGBSegmentation(
	IplImage *pInImage,
	RVLARRAY_<RVLSWER_SEGMENT<RVLSWER_NODE2>> &SegmentArray, 
	IplImage *pOutImage)
{
	unsigned char *InPixArray = (unsigned char *)(pInImage->imageData);

	cvSet(pOutImage, cvScalar(0, 0, 0));

	unsigned char *OutPixArray = (unsigned char *)(pOutImage->imageData);

	int i, j;
	RVLSWER_SEGMENT<RVLSWER_NODE2> *pSegment;
	RVLARRAY_<int> *pPtList;
	int RGB[3], sumRGB[3];
	unsigned char *RGB_;
	unsigned char avgRGB_[3];

	for (i = 0; i < SegmentArray.n; i++)
	{
		pSegment = SegmentArray.Element + i;

		pPtList = &(pSegment->PtList);

		// compute average segment color

		RVLNULL3VECTOR(sumRGB);

		for (j = 0; j < pPtList->n; j++)
		{
			RGB_ = InPixArray + 3 * pPtList->Element[j];

			RGB[0] = (int)(RGB_[0]);
			RGB[1] = (int)(RGB_[1]);
			RGB[2] = (int)(RGB_[2]);

			RVLSUM3VECTORS(RGB, sumRGB, sumRGB);
		}
		
		avgRGB_[0] = (unsigned char)(sumRGB[0] / pPtList->n);
		avgRGB_[1] = (unsigned char)(sumRGB[1] / pPtList->n);
		avgRGB_[2] = (unsigned char)(sumRGB[2] / pPtList->n);

		// draw segment

		for (j = 0; j < pPtList->n; j++)
		{
			RGB_ = OutPixArray + 3 * pPtList->Element[j];

			RVLCOPY3VECTOR(avgRGB_, RGB_);
		}
	}
}

void RVLSegmentationSaveSelection(
	CRVLMPtrChain *pTriangleList,
	int nSegments,
	DWORD Flags,
	char *FileName)
{
	bool *bSegmentSelected = new bool[nSegments];

	memset(bSegmentSelected, 0, nSegments * sizeof(bool));

	bool bSelectedSegments = false;

	CRVL2DRegion2 *pTriangle;

	pTriangleList->Start();

	while (pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if (pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if (pTriangle->m_Label < 0 || pTriangle->m_Label >= nSegments)
			continue;

		if (pTriangle->m_Flags & Flags)
		{
			bSelectedSegments = true;

			bSegmentSelected[pTriangle->m_Label] = true;
		}
	}

	if (bSelectedSegments)
	{
		FILE *fp = fopen(FileName, "w");

		for (int iSegment = 0; iSegment < nSegments; iSegment++)
			fprintf(fp, "%d\t%d\n", iSegment, bSegmentSelected[iSegment]);

		fclose(fp);
	}

	delete[] bSegmentSelected;
}

void RVLSegmentationLoadSelection(
	CRVLMPtrChain *pTriangleList,
	int nSegments,
	DWORD Flags,
	char *FileName)
{
	bool *bSegmentSelected = new bool[nSegments];

	memset(bSegmentSelected, 0, nSegments * sizeof(bool));

	FILE *fp = fopen(FileName, "r");

	if (fp)
	{
		int iTmp1, iTmp2;

		for (int iSegment = 0; iSegment < nSegments; iSegment++)
		{
			fscanf(fp, "%d\t%d\n", &iTmp1, &iTmp2);

			bSegmentSelected[iSegment] = (iTmp2 > 0);
		}

		fclose(fp);
	}

	CRVL2DRegion2 *pTriangle;

	pTriangleList->Start();

	while (pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if (pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if (pTriangle->m_Label < 0 || pTriangle->m_Label >= nSegments)
			continue;

		if (bSegmentSelected[pTriangle->m_Label])
			pTriangle->m_Flags |= Flags;
	}

	delete[] bSegmentSelected;
}


#ifdef RVLVTK
void RVLDisplaySegmentedMesh3D(CRVLVTKRenderer *pRenderer,
                                CRVLMPtrChain *pTriangleList,
                                int nObjects,
                                int w, 
                                int h,
								int nFOVExtensions,
                                int *pointmap_,
                                RVL3DPOINT2 **Point3DMap_,
                                int colortype,
                                IplImage* pTexImg)
{
	uchar *dominantColor = NULL;	

	  RVL3DPOINT2 **Point3DMap;
	  int *pointmap;

      //generating random color (colortype = 0)
      unsigned char **color;
      if (colortype == 0)
      {
		  RVLDominantSegmentColor2(pTriangleList, nObjects, pTexImg, dominantColor);

		  cv::Mat HSVColors(1, nObjects, CV_8UC3);

		 int iObject;
		  uchar *HSVColor;

		  for (iObject = 0; iObject < nObjects; iObject++)
		  {
			  HSVColor = HSVColors.data + 3 * iObject;

			  HSVColor[0] = dominantColor[2 * iObject];
			  HSVColor[1] = dominantColor[2 * iObject + 1];
			  HSVColor[2] = 255;
		  }

		  cv::Mat RGBColors(1, nObjects, CV_8UC3);

		  cv::cvtColor(HSVColors, RGBColors, cv::COLOR_HSV2RGB);

		  uchar *RGBColor;

            color = new unsigned char*[nObjects];
			for (iObject = 0; iObject < nObjects; iObject++)
            {
				color[iObject] = new unsigned char[3];
                  //color[iObject][0] = (unsigned char)(rand() * 255);
                  //color[iObject][1] = (unsigned char)(rand() * 255);
                  //color[iObject][2] = (unsigned char)(rand() * 255);
				  RGBColor = RGBColors.data + 3 * iObject;
				  color[iObject][0] = RGBColor[0];
				  color[iObject][1] = RGBColor[1];
				  color[iObject][2] = RGBColor[2];
            }
      }
      //points map
	  memset(pointmap_, 255, (2 * nFOVExtensions + 1) * w * h * sizeof(int));
      //Setting texture (colortype = 1)
      vtkSmartPointer<vtkFloatArray> texCoords;
      vtkSmartPointer<vtkImageData> texImg;
      vtkSmartPointer<vtkTexture> texObj;
      if (colortype == 1)
      {
            //Texture coordinates & texture
            texCoords = vtkSmartPointer<vtkFloatArray>::New();
            texCoords->SetNumberOfComponents(2);
            //texCoords->SetNumberOfTuples(w * h);
            texImg = vtkSmartPointer<vtkImageData>::New();
            texImg->SetExtent(0, w - 1, 0, h - 1, 0, 0);
            texImg->SetOrigin(0, 0, 0);
            //texImg->SetNumberOfScalarComponents(3, texImg->GetInformation());
            //texImg->SetScalarTypeToUnsignedChar();
			//texImg->SetScalarType(VTK_CHAR, texImg->GetInformation());
			texImg->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
            char *scalarData = (char *)texImg->GetScalarPointer();
            memcpy(scalarData, pTexImg->imageData, w * h * 3 * sizeof(char));
            texObj = vtkSmartPointer<vtkTexture>::New();
			texObj->SetInputData(texImg);// ->SetInputConnection(texImg->GetProducerPort());
            texObj->InterpolateOn();
      }
      //color based on histogram (colortype = 2)
      int r, g, b;
      float matR, matG, matB;
      RVLQLIST_HIST_ENTRY *pHistEntry;

 
	  //Monochrome with selection (colortype = 3)


 

      //Setting up VTK scene

 

 

      vtkRenderer *ren1 = pRenderer->m_pRenderer;
      vtkRenderWindow *renWin = pRenderer->m_pWindow;
      vtkRenderWindowInteractor *iren = pRenderer->m_pInteractor;
      ren1->RemoveAllViewProps();
      ren1->Clear();
      renWin->Render();

      //points variable
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      points->SetDataTypeToFloat();
      points->Reset();
      //triangles variable
      vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
      //indices (for triangles)
      vtkSmartPointer<vtkIdList> indices = vtkSmartPointer<vtkIdList>::New();
      //colors (for triangles)
      vtkSmartPointer<vtkUnsignedCharArray> rgbs = vtkSmartPointer<vtkUnsignedCharArray>::New();
      rgbs->Reset();
      rgbs->SetName("RGB");
      rgbs->SetNumberOfComponents(3);
      //number of inserted points
      int noPts = 0;
      //vertices
      int vert[3];
      //
      CRVL2DRegion2 *pTriangle;
      pTriangleList->Start();
      while(pTriangleList->m_pNext)
      {
            pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

			// pTriangle is a pointer to an instance of the class
            // CRVL2DRegion2 representing a mesh triangle. 
            // Now you can do whatever you want with the triangle. 

            if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
                  continue;

            RVLMESH_LINK *pLink = (RVLMESH_LINK *)(pTriangle->m_PtArray);

			Point3DMap = pTriangle->m_pPoint3DMap;

			pointmap = pointmap_ + (Point3DMap - Point3DMap_);
 
            RVL3DPOINT2 *point = Point3DMap[pLink->iPix0];

			//inserting points/vertices

            if (pointmap[pLink->iPix0] < 0)
            {
                  points->InsertNextPoint(point->XYZ[0], point->XYZ[1],point->XYZ[2]);

				  pointmap[pLink->iPix0] = noPts;

                  vert[0] = noPts;

                  if (colortype == 1)
                        texCoords->InsertNextTuple2((float)(pLink->iPix0 - (int)(pLink->iPix0 / w) * w) / w, (float)((int)(pLink->iPix0 / w))/ h);

                  noPts++;
            }
            else
                  vert[0] = pointmap[pLink->iPix0];

            pLink = pLink->pNext->pOpposite;

            point = Point3DMap[pLink->iPix0];

            if (pointmap[pLink->iPix0] < 0)
            {
                  points->InsertNextPoint(point->XYZ[0], point->XYZ[1], point->XYZ[2]);

                  pointmap[pLink->iPix0] = noPts;

                  vert[1] = noPts;

                  if (colortype == 1)
                        texCoords->InsertNextTuple2((float)(pLink->iPix0 - (int)(pLink->iPix0 / w) * w) / w, (float)((int)(pLink->iPix0 / w))/ h);

                  noPts++;
            }
            else
                  vert[1] = pointmap[pLink->iPix0];

            pLink = pLink->pNext->pOpposite;

            point = Point3DMap[pLink->iPix0];

            if (pointmap[pLink->iPix0] < 0)
            {
                  points->InsertNextPoint(point->XYZ[0], point->XYZ[1], point->XYZ[2]);

                  pointmap[pLink->iPix0] = noPts;

                  vert[2] = noPts;

                  if (colortype == 1)
                        texCoords->InsertNextTuple2((float)(pLink->iPix0 - (int)(pLink->iPix0 / w) * w) / w, (float)((int)(pLink->iPix0 / w))/ h);

                  noPts++;
            }
            else
                  vert[2] = pointmap[pLink->iPix0];

            //inserting triangle

            indices->Reset();
            indices->InsertNextId(vert[0]);
            indices->InsertNextId(vert[1]);
            indices->InsertNextId(vert[2]);
            triangles->InsertNextCell(indices);

            if (colortype == 0)
                  rgbs->InsertNextTupleValue(color[pTriangle->m_Label]);
            else if (colortype == 2)
            {
                  pHistEntry = (RVLQLIST_HIST_ENTRY*)(pTriangle->m_histRGB->pFirst);

                  r = floor(pHistEntry->adr / (pTriangle->m_histRGB_base[1] * pTriangle->m_histRGB_base[2]));

                  matR = (float)((r * (256.0 / pTriangle->m_histRGB_base[0])) + (pTriangle->m_histRGB_base[0]/2));

                  g = floor((pHistEntry->adr / (pTriangle->m_histRGB_base[1] * pTriangle->m_histRGB_base[2]) - r) * pTriangle->m_histRGB_base[1]);

                  matG = (float)((g * (256.0 / pTriangle->m_histRGB_base[1])) + (pTriangle->m_histRGB_base[1]/2));

                  b = floor(((pHistEntry->adr / (pTriangle->m_histRGB_base[1] * pTriangle->m_histRGB_base[2]) - r) * pTriangle->m_histRGB_base[1] - g) * pTriangle->m_histRGB_base[2]);

                  matB = (float)((b * (256.0 / pTriangle->m_histRGB_base[2])) + (pTriangle->m_histRGB_base[2]/2));

                  rgbs->InsertNextTuple3(matR, matG, matB);
            }
            else if (colortype == 3)
            {
                if(pTriangle->m_Flags & RVLOBJ2_FLAG_MARKED)
                     rgbs->InsertNextTuple3(255, 0, 0);
                else
                     rgbs->InsertNextTuple3(0, 255, 0);
            }
      }

      //generating VTK objects
      //polydata
      vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
      output->SetPoints(points);
      output->SetPolys(triangles);
      if ((colortype == 0) || (colortype == 2) || (colortype == 3))
		  output->GetCellData()->SetScalars(rgbs);
      else if (colortype == 1)
          output->GetPointData()->SetTCoords(texCoords);

      /*////BEZ NORMALA////
      //mapper
      vtkSmartPointer<vtkPolyDataMapper> map = vtkSmartPointer<vtkPolyDataMapper>::New();
      map->SetInputConnection(output->GetProducerPort());
     ////BEZ NORMALA////*/

     ////SA NORMALAMA////
      //generate normals filter
      vtkSmartPointer<vtkPolyDataNormals> nor = vtkSmartPointer<vtkPolyDataNormals>::New();
	  nor->SetInputData(output);// ->SetInputConnection(output->GetProducerPort());
      nor->SplittingOff();
      nor->Update();
      //mapper
      vtkSmartPointer<vtkPolyDataMapper> map = vtkSmartPointer<vtkPolyDataMapper>::New();
      map->SetInputConnection(nor->GetOutputPort());
      ////SA NORMALAMA////

      //actor
      vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New(); 
      act->SetMapper(map);
      if (colortype == 1)
		  act->SetTexture(texObj);
      ren1->AddActor(act);                                       
      ren1->ResetCamera();
      ren1->SetBackground(0.5294, 0.8078, 0.9803);
      renWin->Render();
      //releasing
      if (colortype == 0)
      {
		  for (int i = 0; i < nObjects; ++i)
			  delete [] color[i];
          delete [] color;
      }

	  if (dominantColor)
		delete[] dominantColor;
}

#endif