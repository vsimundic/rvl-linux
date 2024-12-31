// RVL2DContour.cpp: implementation of the CRVL2DContour class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

CRVL2DContour RVL2DContourTemplate;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL2DContour::CRVL2DContour()
{

}

CRVL2DContour::~CRVL2DContour()
{

}

void CRVL2DContour::Segment2Lines(CRVLMPtrChain *pContourIPBuffer,
							      double LineSegmentThr)
{
	pContourIPBuffer->RemoveAll();

	RVL2DEdgeContourSegment2Lines((RVLEDGE_ELEMENT *)m_ContourIPArray,
		NULL, 0, pContourIPBuffer, LineSegmentThr);

	RVLARRAY *pSegmentArray = (RVLARRAY *)(m_pData + m_pClass->m_iDataSegmentArray);

	int Size = pContourIPBuffer->m_nElements * sizeof(RVL2DCONTOUR_SEGMENT);

	RVL2DCONTOUR_SEGMENT *pSeg = (RVL2DCONTOUR_SEGMENT *)(m_pClass->m_pMem->Alloc(Size));

	pSegmentArray->pFirst = (BYTE *)pSeg;

	pSegmentArray->pEnd = pSegmentArray->pFirst + Size;

	CRVLC2DContour *pClass = (CRVLC2DContour *)m_pClass;

	RVL2DCONTOUR_SEGMENT *pPrevSeg = NULL;

	pContourIPBuffer->Start();
	
	while(pContourIPBuffer->m_pNext)
	{
		pSeg->p2DContour = this;
		pSeg->pLastEdgeElement = (RVLEDGE_ELEMENT *)(pContourIPBuffer->GetNext());
		pSeg->Index = pClass->m_ContourSegmentList.m_nElements;
		pSeg->NeighborArray.pFirst = pSeg->NeighborArray.pEnd = NULL;
		pSeg->pPrev = pPrevSeg;
		pClass->m_ContourSegmentList.Add(pSeg);
		pPrevSeg = pSeg;
		pSeg++;
		pPrevSeg->pNext = pSeg;
	}

	pPrevSeg->pNext = NULL;
	
	pSegmentArray = (RVLARRAY *)(m_pData + m_pClass->m_iDataSegmentArray);

	pSeg = (RVL2DCONTOUR_SEGMENT *)(pSegmentArray->pFirst);

	RVLEDGE_ELEMENT *pEdgeElement = (RVLEDGE_ELEMENT *)m_ContourIPArray;

	RVLEDGE_ELEMENT *pFirstEdgeElement = pEdgeElement->pContourNeighbor[0];

	pFirstEdgeElement->p2DContourSeg = pSeg;

	do
	{
		if(pEdgeElement == pFirstEdgeElement)
			break;

		pEdgeElement->p2DContourSeg = pSeg;

		if(pEdgeElement == pSeg->pLastEdgeElement)
			pSeg++;

		pEdgeElement = pEdgeElement->pContourNeighbor[1];
	}
	while(pEdgeElement);
}

void CRVL2DContour::Display(CRVLDisplayVector *pVector)
{
	RVLARRAY *pSegmentArray = (RVLARRAY *)(m_pData + m_pClass->m_iDataSegmentArray);

	RVL2DCONTOUR_SEGMENT *pSeg = (RVL2DCONTOUR_SEGMENT *)(pSegmentArray->pFirst);

	RVL2DCONTOUR_SEGMENT *pSegArrayEnd = (RVL2DCONTOUR_SEGMENT *)(pSegmentArray->pEnd);

	RVLEDGE_ELEMENT *pIP = (RVLEDGE_ELEMENT *)m_ContourIPArray;

	pIP = pIP->pContourNeighbor[0];

	if(pIP->nContourNeighbors == 2)
	{
		pVector->m_bClosed = TRUE;

		pSegArrayEnd--;
	}
	else
		pVector->m_bClosed = FALSE;

	pVector->Point(pIP->u, pIP->v);

	for(; pSeg < pSegArrayEnd; pSeg++)
		pVector->Point(pSeg->pLastEdgeElement->u, pSeg->pLastEdgeElement->v);	
}

CRVLObject2 * CRVL2DContour::Create2(CRVLClass *pClass)
{
	CRVLObject2 *pObject = (CRVLObject2 *)(pClass->m_pMem0->Alloc(sizeof(CRVL2DContour)));

	memcpy(pObject, this, sizeof(CRVL2DContour));

	pObject->Create(pClass);

	pClass->Add(pObject);

	return pObject;
}




///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 


void RVLCreateC2DContour(CRVLClass *pClass)
{
	CRVLC2DContour *pC2DContour = (CRVLC2DContour *)pClass;

	pC2DContour->m_nRelLists = 4;

	pC2DContour->m_RelListDesc = new RVLRELLIST_DESC[pC2DContour->m_nRelLists];

	pC2DContour->m_RelListDesc[0].index = RVLRELLIST_INDEX_NEIGHBORS;
	pC2DContour->m_RelListDesc[0].type = RVLRELLIST_TYPE_CHAIN;

	pC2DContour->m_RelListDesc[1].index = RVLRELLIST_INDEX_COMPONENTS;
	pC2DContour->m_RelListDesc[1].type = RVLRELLIST_TYPE_CHAIN;

	pC2DContour->m_RelListDesc[2].index = RVLRELLIST_INDEX_SUPEROBJECTS;
	pC2DContour->m_RelListDesc[2].type = RVLRELLIST_TYPE_CHAIN;

	pC2DContour->m_RelListDesc[3].index = RVLRELLIST_INDEX_2DCONTOUR_CONTOUR_SEGMENTS;
	pC2DContour->m_RelListDesc[3].type = RVLRELLIST_TYPE_CHAIN;	

	CRVLC2D *pC2D = (CRVLC2D *)pClass;

	pClass->Init();

	pC2D->Init();
}

BYTE RVL2DEdgeContourElementDetect(CRVLC2DContour *pClass,
								   void *vpContourDetectData, 
								   BYTE *vpElement, 
								   int Direction)
{
	DWORD Edge = (((RVLAPIX *)(vpElement + pClass->m_dvpEdge2[Direction]))->Edge >>
		pClass->m_EdgeShift[Direction]);

	BYTE mOut = 0x00;

	if(Edge & RVLEDGE_FLAG_RIGHT)
		mOut |= RVL2DCONTOUR_DETECT_RES_CONTOUR;

	if(Edge & RVLEDGE_FLAG_MARK_RIGHT)
		mOut |= RVL2DCONTOUR_DETECT_RES_MARKED;

	return mOut;
}


void RVL2DEdgeContourMarkElement(CRVLC2DContour *pClass,
								 void *vpContourDetectData, 
								 BYTE *vpElement, 
								 int Direction)
{
	RVLAPIX *pAPix = (RVLAPIX *)(vpElement + pClass->m_dvpEdge2[Direction]);

	pAPix->Edge |= (RVLEDGE_FLAG_MARK_RIGHT << pClass->m_EdgeShift[Direction]);
}




void RVL2DContourSegment2Lines(RVLIPOINT *pIP1, RVLIPOINT *pIP2,
							   CRVLMPtrChain *pContourIPBuffer,
							   double LineSegmentThr)
{
	// if the contour has only 2 points then it represents a line segment

	if(pIP2 - pIP1 <= 2)
	{
		pContourIPBuffer->Add(pIP2);

		return;
	}

	// if the contour is closed (the first and the last point have
	// the identical position), then split the contour in two, 
	// where the splitting point is the most distat point relative to
	// the first point of the contour

	RVLIPOINT *pIP, *pMostDistantIP;
	int maxDist2;
	int du, dv, dist2;

	if(pIP1->vp == pIP2->vp)
	{
		// find the most distant contour point relative to pIP1

		maxDist2 = 0;
		pMostDistantIP = pIP1;
		
		for(pIP = pIP1 + 1; pIP < pIP2; pIP++)
		{
			du = pIP->u - pIP1->u;
			dv = pIP->v - pIP1->v;
			dist2 = du * du + dv * dv;

			if(dist2 > maxDist2)
			{
				maxDist2 = dist2;
				pMostDistantIP = pIP;
			}
		}

		// segment the contour

		RVL2DContourSegment2Lines(pIP1, pMostDistantIP, pContourIPBuffer, 
			LineSegmentThr);
		RVL2DContourSegment2Lines(pMostDistantIP, pIP2, pContourIPBuffer, 
			LineSegmentThr);

		return;
	}

	// if the contour is not closed, split it recursively to approximatelly 
	// collinear set of points 

	// generate a line hypothesis: line with endpoints identical to the 
	// endpoints of the contour

	int du12 = pIP1->u - pIP2->u;
	int dv12 = pIP2->v - pIP1->v;
	int d = dv12 * pIP1->u + du12 * pIP1->v;
	double len = sqrt((double)(du12 * du12 + dv12 * dv12));

	// find the most distant contour point from the hypothetical line segment

	int maxDist = 0;

	int dist;

	for(pIP = pIP1 + 1; pIP < pIP2; pIP++)
	{
		dist = abs(dv12 * pIP->u + du12 * pIP->v - d);

		if(dist > maxDist)
		{
			maxDist = dist;
			pMostDistantIP = pIP;
		}
	}

	// if the distance between the most distant contour point from the hypothetical 
	// line segment is greater then SegmentThr, then segment the contour so that
	// the most distant point is the breaking point

	if((double)maxDist / len > LineSegmentThr)
	{
		RVL2DContourSegment2Lines(pIP1, pMostDistantIP, pContourIPBuffer, 
			LineSegmentThr);
		RVL2DContourSegment2Lines(pMostDistantIP, pIP2, pContourIPBuffer, 
			LineSegmentThr);
		return;
	}

	// if the distances from all contour points to the hypothetical line segment are 
	// <= SegmentThr, then add iIP2 to the pContourIPBuffer

	pContourIPBuffer->Add(pIP2);
}


void RVL2DEdgeContourSegment2Lines(RVLEDGE_ELEMENT *pIP0, RVLEDGE_ELEMENT *pIP2, 								   
								   int nIn,
							       CRVLMPtrChain *pContourIPBuffer,
							       double LineSegmentThr)
{
	RVLEDGE_ELEMENT *pIP1 = pIP0->pContourNeighbor[0];

	//if(pIP1)
	//	if(pIP1->u == 74 * 2 && pIP1->v == 203 * 2)
	//		int tmp1 = 0;

	// get the contour length

	BOOL bClosed = FALSE;
	RVLEDGE_ELEMENT *pIP;
	
	int n;

	if(pIP2)
		n = nIn;
	else
	{
		n = 1;

		pIP = pIP0;

		while(pIP)
		{
			pIP2 = pIP;

			n++;

			pIP = pIP->pContourNeighbor[1];

			if(pIP == pIP1)
			{
				n++;

				pIP2 = pIP;

				bClosed = TRUE;

				break;
			}
		}
	}
	

	// if the contour has 3 or less points, then it represents a line segment

	if(n <= 3)
	{
		pContourIPBuffer->Add(pIP2);

		return;
	}

	// if the contour is closed (the first and the last point have
	// the identical position), then split the contour in two, 
	// where the splitting point is the most distat point relative to
	// the first point of the contour

	int i = 1;

	RVLEDGE_ELEMENT *pMostDistantIP;
	int maxDist2;
	int du, dv, dist2;
	int n1;

	if(bClosed)
	{
		// find the most distant contour point relative to pIP1

		maxDist2 = 0;
		pMostDistantIP = pIP1;
		pIP = pIP0;
		
		while(pIP != pIP2)
		{
			i++;
			du = pIP->u - pIP1->u;
			dv = pIP->v - pIP1->v;
			dist2 = du * du + dv * dv;

			if(dist2 > maxDist2)
			{
				n1 = i;
				maxDist2 = dist2;
				pMostDistantIP = pIP;
			}

			pIP = pIP->pContourNeighbor[1];
		}

		// segment the contour

		RVL2DEdgeContourSegment2Lines(pIP0, pMostDistantIP, n1, pContourIPBuffer, 
			LineSegmentThr);
		RVL2DEdgeContourSegment2Lines(pMostDistantIP->pContourNeighbor[1], pIP2, n - n1 + 1, pContourIPBuffer, 
			LineSegmentThr);

		return;
	}

	// if the contour is not closed, split it recursively to approximatelly 
	// collinear set of points 

	// generate a line hypothesis: line with endpoints identical to the 
	// endpoints of the contour

	int du12 = pIP1->u - pIP2->u;
	int dv12 = pIP2->v - pIP1->v;
	int d = dv12 * pIP1->u + du12 * pIP1->v;
	double len = sqrt((double)(du12 * du12 + dv12 * dv12));

	// find the most distant contour point from the hypothetical line segment

	int maxDist = 0;

	i = 1;

	pIP = pIP0;

	int dist;
		
	while(pIP != pIP2)
	{
		i++;
		//dist = abs(dv12 * pIP->u + du12 * pIP->v - d) - pIP->w;
		dist = (abs(dv12 * pIP->u + du12 * pIP->v - d) >> pIP->w);

		if(dist > maxDist)
		{
			n1 = i;
			maxDist = dist;
			pMostDistantIP = pIP;
		}

		pIP = pIP->pContourNeighbor[1];
	}

	// if the distance between the most distant contour point from the hypothetical 
	// line segment is greater then LineSegmentThr, then segment the contour so that
	// the most distant point is the breaking point

	if((double)maxDist / len > LineSegmentThr)
	{
		RVL2DEdgeContourSegment2Lines(pIP0, pMostDistantIP, n1, pContourIPBuffer, 
			LineSegmentThr);
		RVL2DEdgeContourSegment2Lines(pMostDistantIP->pContourNeighbor[1], pIP2, n - n1 + 1, pContourIPBuffer, 
			LineSegmentThr);
		return;
	}

	// if the distances from all contour points to the hypothetical line segment are 
	// <= SegmentThr, then add iIP2 to the pContourIPBuffer

	pContourIPBuffer->Add(pIP2);
}

void RVL2DEdgeContourOnCreate(CRVLC2DContour *pClass,
							  CRVL2DContour *p2DContour,
							  void *vpContourDetectData)
{
	int i;
	RVLIPOINT *pIP = p2DContour->m_ContourIPArray;
	RVLAPIX *pAPix;
	int nContours;
	RVLOBJ2_CONNECTION *pcEdgeObject;

	for(i = 0; i < 2; i++)
	{
		pAPix = (RVLAPIX *)(pIP->vp);

		nContours = (int)(pAPix->Flags & RVLAPIX_FLAG_N_CONTOURS);

		if(nContours == 0)
		{
			pcEdgeObject = (RVLOBJ2_CONNECTION *)
				(pClass->m_pMem->Alloc(4 * sizeof(RVLOBJ2_CONNECTION)));
			pAPix->vpcEdgeObject = pcEdgeObject;
		}
		else 
			pcEdgeObject = (RVLOBJ2_CONNECTION *)(pAPix->vpcEdgeObject)
				+ nContours;

		pcEdgeObject->vpObject = p2DContour;
		pcEdgeObject->jConnector = i;

		pAPix->Flags++;

		pIP += (p2DContour->m_nContourIPs - 1);
	}
}


// Assigns the closest contour point to all image points

void RVL2DContourEDT(CRVLC2DContour *pClass,
					 CRVLMPtrChain *p2DContourArray,
					 CRVLEDT *pEDT,
					 RVLEDT_PIX_ARRAY *pEDTImage,
					 CRVLMChain *pIPBuffer)
{
	int Width = pClass->m_Array.m_Width;
	int Height = pClass->m_Array.m_Height;

	CRVLMPtrChain BorderIn;

	BorderIn.m_pMem = pClass->m_pMem2;

	RVLIPOINT *pEndIP;

	CRVL2DContour *p2DContour;
	RVLIPOINT IP;
	RVLIPOINT *pIP1, *pIP2;
	int du, dv, u1, v1;

	p2DContourArray->Start();

	while(p2DContourArray->m_pNext)
	{
		p2DContour = (CRVL2DContour *)(p2DContourArray->GetNext());

		pEndIP = p2DContour->m_ContourIPArray + p2DContour->m_nContourIPs;

		pIP1 = p2DContour->m_ContourIPArray;

		for(pIP2 = p2DContour->m_ContourIPArray + 1; pIP2 < pEndIP; pIP2++)
		{
			du = pIP2->u - pIP1->u;
			dv = pIP2->v - pIP1->v;

			u1 = pIP2->u + pIP1->u - 1;
			v1 = pIP2->v + pIP1->v - 1;

			IP.u = ((u1 - dv) >> 1);
			IP.v = ((v1 + du) >> 1);
			IP.vp = p2DContour;

			BorderIn.Add((void *)(pEDTImage->pPix + (IP.u  + IP.v * Width)));

			pIPBuffer->Add(&IP);

			IP.u = ((u1 + dv) >> 1);
			IP.v = ((v1 - du) >> 1);
			IP.vp = p2DContour;

			BorderIn.Add((void *)(pEDTImage->pPix + (IP.u  + IP.v * Width)));

			pIPBuffer->Add(&IP);

			pIP1 = pIP2;
		}
	}

	/*
	int u, v;

	for(u = pClass->m_ROI.left; u <= pClass->m_ROI.right; u++)
	{
		IP.u = u;
		IP.v = pClass->m_ROI.top;
		IP.vp = NULL;

		BorderIn.Add((void *)(pEDTImage->pPix + (IP.u  + IP.v * Width)));

		pIPBuffer->Add(&IP);

		IP.u = u;
		IP.v = pClass->m_ROI.bottom;
		IP.vp = NULL;

		BorderIn.Add((void *)(pEDTImage->pPix + (IP.u  + IP.v * Width)));

		pIPBuffer->Add(&IP);
	}

	for(v = pClass->m_ROI.top + 1; v < pClass->m_ROI.bottom; v++)
	{
		IP.u = pClass->m_ROI.left;
		IP.v = v;
		IP.vp = NULL;

		BorderIn.Add((void *)(pEDTImage->pPix + (IP.u  + IP.v * Width)));

		pIPBuffer->Add(&IP);

		IP.u = pClass->m_ROI.right;
		IP.v = v;
		IP.vp = NULL;

		BorderIn.Add((void *)(pEDTImage->pPix + (IP.u  + IP.v * Width)));

		pIPBuffer->Add(&IP);
	}
	*/

	int ImageSize = pClass->m_Array.m_Width * pClass->m_Array.m_Height;

	WORD iBucket;

	for(iBucket = 0; iBucket < 4; iBucket++)
		pEDT->m_BucketPtrArray[iBucket] = (RVLEDT_BUCKET_ENTRY *)
			(pClass->m_pMem2->Alloc(ImageSize * sizeof(RVLEDT_BUCKET_ENTRY)));

	memset(pEDTImage->pPix, 0x00, pClass->m_ROI.top * Width	* sizeof(RVLEDT_PIX));	

	memset(pEDTImage->pPix + (pClass->m_ROI.bottom + 1) * Width, 0x00, 
		(Height - pClass->m_ROI.bottom - 1) * Width * sizeof(RVLEDT_PIX));

	RVLEDT_PIX *pEDTPix = pEDTImage->pPix + pClass->m_ROI.top * Width;

	int LeftBorderWidth = pClass->m_ROI.left;
	int LeftBorderSize = LeftBorderWidth * sizeof(RVLEDT_PIX);
	int RightBorderWidth = (Width - pClass->m_ROI.right - 1);
	int RightBorderSize = RightBorderWidth * sizeof(RVLEDT_PIX);
	int ROIWidth = (pClass->m_ROI.right - pClass->m_ROI.left + 1);
	int ROISize = ROIWidth * sizeof(RVLEDT_PIX);
	int RightBorderOffset = LeftBorderWidth + ROIWidth;

	int v;

	for(v = pClass->m_ROI.top; v <= pClass->m_ROI.bottom; v++, pEDTPix += Width)
	{
		memset(pEDTPix, 0x00, LeftBorderSize);
		memset(pEDTPix + LeftBorderWidth, 0xff, ROISize);
		memset(pEDTPix + RightBorderOffset, 0x00, RightBorderSize);
	}	
	
	//pEDT->InitEDTPixArray(pEDTImage);

	//int ROIWidth = pClass->m_ROI.right - pClass->m_ROI.left + 1;

	//memset(pEDTImage->pPix + pClass->m_ROI.left + pClass->m_ROI.top * Width, 0x00, 
	//	ROIWidth * sizeof(RVLEDT_PIX));	

	//memset(pEDTImage->pPix + pClass->m_ROI.left + pClass->m_ROI.bottom * Width, 0x00, 
	//	ROIWidth * sizeof(RVLEDT_PIX));

	//RVLEDT_PIX *pEDTPix = pEDTImage->pPix + pClass->m_ROI.left + pClass->m_ROI.top * Width;

	//int dpPix3 = ROIWidth - 1;

	//int v;
	
	//for(v = pClass->m_ROI.top; v <= pClass->m_ROI.bottom; v++, pEDTPix += Width)
	//{
	//	memset(pEDTPix, 0x00, sizeof(RVLEDT_PIX));
	//	memset(pEDTPix + dpPix3, 0x00, sizeof(RVLEDT_PIX));
	//}

	CRVLBuffer EDTBuff;

	EDTBuff.DataBuff = (void **)(pClass->m_pMem2->Alloc(ImageSize * sizeof(void *)));
	EDTBuff.m_bOwnData = FALSE;

	pEDT->Apply(&BorderIn, NULL, pEDTImage, &EDTBuff);

	pClass->m_pMem2->Clear();
}

void RVL2DEdgeContourDetect(CRVLMPtrChain *pContourEndList, 
							CRVLC2DContour *pClass,
							CRVLMem *pMem,
							CRVLMem *pMem2,
							double LineSegmentThr)
{
#ifdef RVL_DEBUG
	fprintf(fpDebug, "=== EDGE CONTOUR DETECTION ============================\n\n\n");
#endif

	CRVLMPtrChain *p2DContourList = &(pClass->m_ObjectList);
	CRVLMPtrChain *pBranchList = &(pClass->m_BranchList);

	pClass->m_ContourSegmentList.RemoveAll();
	pBranchList->RemoveAll();

	CRVLMPtrChain ContourIPBuffer(pMem2);

	CRVL2DContour Contour2DTemplate;
	CRVL2DContour *p2DContour;
	RVLEDGE_ELEMENT *pEdgeElement, *pNextEdgeElement;
	RVLEDGE_ELEMENT **ppNextEdgeElement;
	RVLEDGE_ELEMENT **NeighborPtrArrayEnd;
	
	pContourEndList->Start();
	
	while(pContourEndList->m_pNext)
	{
		pEdgeElement = (RVLEDGE_ELEMENT *)(pContourEndList->GetNext());

		// segment contour into line segments

		p2DContour = NULL;

		if(pEdgeElement->nContourNeighbors == 2)	// closed contour
		{
			p2DContour = (CRVL2DContour *)(Contour2DTemplate.Create3(pClass));

			p2DContour->m_Index = pClass->m_ObjectList.m_nElements - 1;

			p2DContour->m_ContourIPArray = (RVLIPOINT *)(pEdgeElement);	
			
			p2DContour->Segment2Lines(&ContourIPBuffer, LineSegmentThr);

		}
		else	// contour end or branch
		{
			if(pEdgeElement->nContourNeighbors >= 3)
			{
				pBranchList->Add(pEdgeElement);

				pEdgeElement->Flags |= RVLEDGE_ELEMENT_FLAG_BRANCH;
			}
			//else if(pEdgeElement->nContourNeighbors == 1)
			//	pEdgeElement->Flags |= RVLEDGE_ELEMENT_FLAG_TERMINATING;

			//pEdgeElement->pContourNeighbor[0] = pEdgeElement->pContourNeighbor[1] = NULL;

			NeighborPtrArrayEnd = pEdgeElement->NeighborPtrArray + pEdgeElement->nNeighbors;

			for(ppNextEdgeElement = pEdgeElement->NeighborPtrArray; 
				ppNextEdgeElement < NeighborPtrArrayEnd; ppNextEdgeElement++)
			{
				pNextEdgeElement = *ppNextEdgeElement;

				if(pNextEdgeElement == NULL)
					continue;

				if((pNextEdgeElement->Flags & RVLEDGE_ELEMENT_FLAG_CONTOUR) == 0)
					continue;
				
				if(pNextEdgeElement->pContourNeighbor[0] == pEdgeElement)
				{
					p2DContour = (CRVL2DContour *)(Contour2DTemplate.Create3(pClass));

					p2DContour->m_Index = pClass->m_ObjectList.m_nElements - 1;

					p2DContour->m_ContourIPArray = (RVLIPOINT *)(pNextEdgeElement);

					p2DContour->Segment2Lines(&ContourIPBuffer, LineSegmentThr);

#ifdef RVL_DEBUG
					fprintf(fpDebug, "%5d:\n", p2DContour->m_Index);

					fprintf(fpDebug, "(%5.1lf,%5.1lf)\n", 0.5 * (double)pEdgeElement->u, 0.5 * (double)pEdgeElement->v);

					RVLEDGE_ELEMENT *pEdgeElement2 = pNextEdgeElement;

					do
						fprintf(fpDebug, "(%5.1lf,%5.1lf)\n", 0.5 * (double)pEdgeElement2->u, 0.5 * (double)pEdgeElement2->v);
					while(pEdgeElement2 = pEdgeElement2->pContourNeighbor[1]);

					fprintf(fpDebug, "\n");
#endif
				}
			}
		}
	}

#ifdef RVL_DEBUG
	fprintf(fpDebug, "\n\n");
#endif

	pMem2->Clear();
}


void RVL2DEdgeContourDisplay(CRVLMPtrChain *p2DContourList,
							 PIX_ARRAY *pIIn,
							 PIX_ARRAY *pIOut)
{
	// copy the input image to the output image

	unsigned char *pPixIn = pIIn->pPix;

	int w = pIIn->Width;
	int h = pIIn->Height;

	unsigned char *pPixOut = pIOut->pPix;

	int u, v;
	unsigned char I;

	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pPixIn++)
		{
			I = *pPixIn;

			*(pPixOut++) = I;
			*(pPixOut++) = I;
			*(pPixOut++) = I;
		}
	}

	// draw contours

	CRVL2DContour *p2DContour;
	RVLEDGE_ELEMENT *pEdgeElement, *pEdgeElement0;
	int uPrev, vPrev;
	//int du, dv, s, t;

	p2DContourList->Start();

	while(p2DContourList->m_pNext)
	{
		p2DContour = (CRVL2DContour *)(p2DContourList->GetNext());

		pEdgeElement = (RVLEDGE_ELEMENT *)(p2DContour->m_ContourIPArray);

		pEdgeElement0 = pEdgeElement->pContourNeighbor[0];

		u = pEdgeElement0->u / 2;
		v = pEdgeElement0->v / 2;

		pPixOut = pIOut->pPix + 3 * (u + v * w);

		if(pEdgeElement0->nContourNeighbors == 2)
		{
			*(pPixOut++) = 255;
			*(pPixOut++) = 0;
			*pPixOut     = 255;	
		}
		else
		{
			*(pPixOut++) = 0;
			*(pPixOut++) = 255;
			*pPixOut     = 0;	
		}

		while(pEdgeElement)
		{			
			uPrev = u;
			vPrev = v;

			u = pEdgeElement->u / 2;
			v = pEdgeElement->v / 2;

			pEdgeElement = pEdgeElement->pContourNeighbor[1];

			// under construction !!!

			//du = u - uPrev;
			//dv = v - vPrev;

			//if(abs(du) >= abs(dv))
			//{
			//	for(s = uPrev; s <= u; s++)
			//		t = vPrev + ((s - uPrev) * 2 / du + 1) / 2;
			//}
			//else
			//{
			//}

			pPixOut = pIOut->pPix + 3 * (u + v * w);

			if(pEdgeElement)
			{
				*(pPixOut++) = 255;
				*(pPixOut++) = 0;
				*pPixOut     = 255;
			}
			else
			{
				*(pPixOut++) = 0;
				*(pPixOut++) = 255;
				*pPixOut     = 0;	
			}

			if(pEdgeElement == pEdgeElement0)
				break;
		}
	}
}

RVLEDGE_ELEMENT *RVL2DContourGetNextBranch(RVLEDGE_ELEMENT *pBranching,
										   RVLEDGE_ELEMENT *pEdgeElementIn)
{
	RVLEDGE_ELEMENT **ppBranch, **ppNextBranch;
	RVLEDGE_ELEMENT *pEdgeElement;

	RVLEDGE_ELEMENT **ppFirstBranch = (RVLEDGE_ELEMENT **)(pBranching->pBranchPtrArray->pFirst);
	RVLEDGE_ELEMENT **BranchPtrArrayEnd = (RVLEDGE_ELEMENT **)(pBranching->pBranchPtrArray->pEnd);

	for(ppBranch = ppFirstBranch; ppBranch < BranchPtrArrayEnd; ppBranch++)
	{
		pEdgeElement = *ppBranch;

		if(pEdgeElement == pEdgeElementIn)
		{
			ppNextBranch = ppBranch + 1;

			return (ppNextBranch == BranchPtrArrayEnd ? *ppFirstBranch : *ppNextBranch);
		}
	}

	return NULL;
}

BOOL RVLIsInsideContour(CRVLMPtrChain *pContourList,
						int nContours,
						int u, int v)
{
	CvPoint *PtArray;
	int iContour;
	int nBoundaryPts;
	CRVL2DContour *pContour;

	for(iContour = 0; iContour < nContours; iContour++)
	{
		pContour = (CRVL2DContour *)(pContourList->GetNext());

		nBoundaryPts = pContour->m_nContourIPs;

		PtArray = (CvPoint *)(pContour->m_ContourIPArray);

		if(RVLIsInsideContour(PtArray, nBoundaryPts, u, v))
			return TRUE;
	}

	return FALSE;
}


BOOL RVLIsInsideContour(CvPoint *PtArray,
						int nPts,
						int u, int v)
{
	if(nPts == 0)
		return FALSE;

	int iPt;
	CvPoint *pPt2;
	CvPoint *pPt1;	
	int sign0, sign, signPrev;
	int u1, v1, du, dv;

	int nCrosses = 0;

	pPt1 = PtArray;

	if(pPt1->y < v)
		sign0 = -1;
	else if(pPt1->y > v)
		sign0 = 1;
	else
		sign0 = 0;

	signPrev = sign0;

	for(iPt = 0; iPt < nPts; iPt++)
	{
		pPt2 = PtArray + ((iPt + 1) % nPts);

		if(pPt2->y < v)
			sign = -1;
		else if(pPt2->y > v)
			sign = 1;
		else
			sign = 0;

		if(sign0 == 0)
			if(sign != 0)
				sign0 = sign;

		if(sign * signPrev < 0)
		{
			dv = pPt2->y - pPt1->y;

			if(dv > 0)
			{
				u1 = pPt1->x;
				v1 = pPt1->y;
				du = pPt2->x - u1;
			}
			else
			{
				u1 = pPt2->x;
				v1 = pPt2->y;
				du = pPt1->x - u1;
				dv = -dv;
			}
		
			if(dv * u < du * (v - v1) + dv * u1)
				nCrosses++;
		}

		if(sign != 0)
			signPrev = sign;
		
		pPt1 = pPt2;
	}

	if(pPt2->x > u)
		if(signPrev * sign0 < 0)
			nCrosses++;
	
	if(nCrosses & 1)
		return TRUE;

	return FALSE;
}

void RVLDetectDepthDiscontinuityContours(short *Depth,
										 int w, int h,
										 unsigned int Format,
										 short DepthDiscontinuityThr,
										 int minSize,
										 CRVLMem *pMem,
										 void *vpAImage,
										 RVLQLIST *pContourList,
										 BYTE *ContourMap)
{
	short InvalidDepth = (Format == RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY ? 2047 : 0);

	int ImageSize = w * h;

	CRVLAImage *pAImage = (CRVLAImage *)vpAImage;

	int *dpNeighbor4 = pAImage->m_dpNeighbor4;
	int *NeighborLimit = pAImage->m_NeighborLimit;
	
	if(ContourMap == NULL)
		ContourMap = new BYTE[4 * ImageSize];

	memset(ContourMap, 0, 4 * ImageSize);

	short *pDepthMapEnd = Depth + ImageSize;

	// detect depth discontinuities

	short *pDepth;
	int iPix;
	int u, v, iPix2;
	int iNeighbor;	
	short d, d2;

	for (v = NeighborLimit[3]; v <= NeighborLimit[1]; v++)
	{
		iPix = w * v + NeighborLimit[2];
		pDepth = Depth + iPix;

		for (u = NeighborLimit[2]; u <= NeighborLimit[0]; u++, pDepth++, iPix++)
		{
			d = *pDepth;

			if (d == InvalidDepth)
				continue;

			for (iNeighbor = 0; iNeighbor <= 3; iNeighbor++)
			{
				if (iNeighbor & 1)
				{
					if (v == NeighborLimit[iNeighbor])
					{
						ContourMap[4 * iPix + iNeighbor] |= RVL2DCONTOUR_STOP;

						continue;
					}
				}
				else
				{
					if (u == NeighborLimit[iNeighbor])
					{
						ContourMap[4 * iPix + iNeighbor] |= RVL2DCONTOUR_STOP;

						continue;
					}
				}

				iPix2 = iPix + dpNeighbor4[iNeighbor];

				d2 = Depth[iPix2];

				if (d2 == InvalidDepth)
					ContourMap[4 * iPix + iNeighbor] |= (RVL2DCONTOUR_EDGE | RVL2DCONTOUR_VOID);
				else if (d2 - d >= DepthDiscontinuityThr)
					ContourMap[4 * iPix + iNeighbor] |= RVL2DCONTOUR_EDGE;
				else if (d - d2 >= DepthDiscontinuityThr)
					ContourMap[4 * iPix + iNeighbor] |= RVL2DCONTOUR_STOP;
			}
		}
	}

	// connect contour elements

	int i, j;
	BYTE *pContourElement, *pNextContourElement, *pFirstContourElement;

	for(iPix = 0; iPix < ImageSize; iPix++)
	{
		for(iNeighbor = 0; iNeighbor <= 3; iNeighbor++)
		{
			pContourElement = ContourMap + 4 * iPix + iNeighbor;

			if((*pContourElement) & RVL2DCONTOUR_EDGE)
			{
				j = ((iNeighbor + 2) & 3);

				for(i = 1; i <= 3; i++)
				{
					j = ((j + 1) & 3);

					pNextContourElement = pContourElement + pAImage->m_dpContourFollow[iNeighbor][j];

					if((*pNextContourElement) & RVL2DCONTOUR_STOP)
						break;

					if((*pNextContourElement) & RVL2DCONTOUR_EDGE)
					{
						(*pNextContourElement) |= RVL2DCONTOUR_PREV;
						(*pContourElement) |= (RVL2DCONTOUR_NEXT | j);
						break;
					}
				}
			}
		}
	}

	//* put the ptrs. to the starting contour elements into pContourList

	RVLQLIST_INIT(pContourList)

	int iNeighbor0, iNextNeighbor;
	RVL2DCONTOUR_DATA2 *pContour;
	int nElements;

	// get open contours

	for(iPix = 0; iPix < ImageSize; iPix++)
	{
		//if(iPix == 319 + 239 * 320)
		//	int debug = 0;

		for(iNeighbor0 = 0; iNeighbor0 <= 3; iNeighbor0++)
		{
			pContourElement = ContourMap + 4 * iPix + iNeighbor0;

			iNeighbor = iNeighbor0;

			if(((*pContourElement) & (RVL2DCONTOUR_EDGE | RVL2DCONTOUR_PREV | RVL2DCONTOUR_VISITED)) == RVL2DCONTOUR_EDGE)
			{
				nElements = 0;

				pFirstContourElement = pContourElement;

				while(TRUE)
				{
					nElements++;

					(*pContourElement) |= RVL2DCONTOUR_VISITED;

					if((*pContourElement) & RVL2DCONTOUR_NEXT)
					{
						iNextNeighbor = RVL2DCONTOUR_GET_NEXT_EDGEID(pContourElement);
						pContourElement = RVL2DCONTOUR_GET_NEXT(pContourElement, pAImage, iNeighbor, iNextNeighbor);
						iNeighbor = iNextNeighbor;
					}
					else
						break;
				}
			
				if(nElements >= minSize)
				{
					RVLMEM_ALLOC_STRUCT(pMem, RVL2DCONTOUR_DATA2, pContour)
					RVLQLIST_ADD_ENTRY(pContourList, pContour)
					pContour->pFirst = pFirstContourElement;
					pContour->pLast = pContourElement;
				}
			}
		}		
	}

	// get closed contours

	for(iPix = 0; iPix < ImageSize; iPix++)
	{
		for(iNeighbor0 = 0; iNeighbor0 <= 3; iNeighbor0++)
		{
			pContourElement = ContourMap + 4 * iPix + iNeighbor0;

			iNeighbor = iNeighbor0;

			if(((*pContourElement) & (RVL2DCONTOUR_EDGE | RVL2DCONTOUR_VISITED)) == RVL2DCONTOUR_EDGE)
			{
				nElements = 0;

				pFirstContourElement = pContourElement;

				while(TRUE)
				{
					nElements++;

					(*pContourElement) |= RVL2DCONTOUR_VISITED;

					iNextNeighbor = RVL2DCONTOUR_GET_NEXT_EDGEID(pContourElement);
					pContourElement = RVL2DCONTOUR_GET_NEXT(pContourElement, pAImage, iNeighbor, iNextNeighbor);
					iNeighbor = iNextNeighbor;

					if(pContourElement == pFirstContourElement)
						break;
				}

				if(nElements >= minSize)
				{
					RVLMEM_ALLOC_STRUCT(pMem, RVL2DCONTOUR_DATA2, pContour)
					RVLQLIST_ADD_ENTRY(pContourList, pContour)
					pContour->pFirst = pFirstContourElement;
					pContour->pLast = pContourElement;
				}
			}
		}		
	}
}

void RVLDepthContourSegmentToLines(	RVL2DCONTOUR_DATA2 *pContour,
									BYTE *ContourMap,
									RVL3DPOINT2 **Pt3DMap,
									double Tol,
									CRVLMem *pMem,
									void *vpAImage,
									RVLQLIST *pVertexList,
									int &nVertices)
{
	double k = 0.25;

	RVLQLIST_INIT(pVertexList);
	
	nVertices = 0;

	BYTE *pContourElement1 = pContour->pFirst;

	if((*pContourElement1 | RVL2DCONTOUR_NEXT) == 0)
		return;

	CRVLAImage *pAImage = (CRVLAImage *)vpAImage;

	double Tol2 = Tol * Tol;

	BYTE *pContourElement2 = pContour->pLast;

	RVLQLIST_PTR_ENTRY *pVertex1;

	RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pVertex1)

	pVertex1->Ptr = pContourElement1;

	RVLQLIST_ADD_ENTRY(pVertexList, pVertex1)

	RVLQLIST_PTR_ENTRY *pVertex2;

	RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pVertex2)

	pVertex2->Ptr = pContourElement2;

	RVLQLIST_ADD_ENTRY(pVertexList, pVertex2)

	nVertices = 2;

	int iNeighbor, iNextNeighbor;
	BYTE *pContourElement;
	int iPix, iPix1, iPix2, iPixPrev;
	RVL3DPOINT2 *pPt, *pPt1, *pPt2;
	double e, emax;
	RVLQLIST_PTR_ENTRY *pVertex;
	BYTE *pContourElementMaxDev;
	double U[3], U1[3], U2[3], V[3], X[3], P[3], Q[3];
	double l;
	double p;

	while(TRUE)
	{
		iPix1 = RVL2DCONTOUR_GET_IPIX(pContourElement1, ContourMap);
		iPix2 = RVL2DCONTOUR_GET_IPIX(pContourElement2, ContourMap);

		//if(iPix1 == 320*240-1)
		//	int debug = 0;

		pPt1 = Pt3DMap[iPix1];
		pPt2 = Pt3DMap[iPix2];

		if(iPix1 != iPix2)
		{
			U1[0] = pPt1->x;
			U1[1] = pPt1->y;
			U1[2] = k * pPt1->z;

			U2[0] = pPt2->x;
			U2[1] = pPt2->y;
			U2[2] = k * pPt2->z;

			RVLDIF3VECTORS(U2, U1, V)
			l = sqrt(RVLDOTPRODUCT3(V, V));
			RVLSCALE3VECTOR2(V, l, V);
		}

		pContourElement = pContourElement1;

		iNeighbor = RVL2DCONTOUR_GET_EDGEID(pContourElement, ContourMap);

		iPixPrev = iPix1;

		emax = 0.0;

		while(TRUE)		// search all points along the segment between pPt1 and pPt2 
						// for the point with the maximum deviation from the line pPt1 - pPt2
		{			
			if((*pContourElement) & RVL2DCONTOUR_NEXT)
			{
				iNextNeighbor = RVL2DCONTOUR_GET_NEXT_EDGEID(pContourElement);
				pContourElement = RVL2DCONTOUR_GET_NEXT(pContourElement, pAImage, iNeighbor, iNextNeighbor);
				iNeighbor = iNextNeighbor;

				if(pContourElement == pContourElement2)
					break;

				iPix = RVL2DCONTOUR_GET_IPIX(pContourElement, ContourMap);

				if(iPix != iPixPrev && iPix != iPix2)
				{
					// compute the distance from the line segment

					pPt = Pt3DMap[iPix];

					U[0] = pPt->x;
					U[1] = pPt->y;
					U[2] = k * pPt->z;

					RVLDIF3VECTORS(U, U1, X)

					if(iPix1 != iPix2)
					{
						p = RVLDOTPRODUCT3(X, V);

						if(p < 0.0)
							e = RVLDOTPRODUCT3(X, X);
						else if(p > l)
						{
							RVLDIF3VECTORS(U, U2, X);

							e = RVLDOTPRODUCT3(X, X);
						}
						else
						{
							RVLSCALE3VECTOR(V, p, P)
							RVLDIF3VECTORS(X, P, Q)
							e = RVLDOTPRODUCT3(Q, Q);
						}
					}
					else
						e = RVLDOTPRODUCT3(X, X);

					/////

					if(e > emax)
					{
						pContourElementMaxDev = pContourElement;

						emax = e;
					}
				}
			}
			else
				break;
		}	// search all points along the segment between pPt1 and pPt2 
			// for the point with the maximum deviation from the line pPt1 - pPt2

		if(emax > Tol2)
		{
			RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pVertex)

			pVertex->Ptr = pContourElementMaxDev;

			RVLQLIST_INSERT_ENTRY(pVertexList, pVertex1, pVertex2, pVertex)

			pVertex2 = pVertex;

			pContourElement2 = pContourElementMaxDev;

			nVertices++;
		}
		else
		{
			if(pVertex2->pNext == NULL)
				break;

			pVertex1 = (RVLQLIST_PTR_ENTRY *)(pVertex1->pNext);
		
			pVertex2 = (RVLQLIST_PTR_ENTRY *)(pVertex2->pNext);

			pContourElement1 = (BYTE *)(pVertex1->Ptr);

			pContourElement2 = (BYTE *)(pVertex2->Ptr);
		}
	}
}

void RVL2DContourDisplay(IplImage *pDisplay,
						 RVLQLIST *pContourList,
						 BYTE *ContourMap,
						 void *vpAImage)
{
	CRVLAImage *pAImage = (CRVLAImage *)vpAImage;

	RVL2DCONTOUR_DATA2 *pContour = (RVL2DCONTOUR_DATA2 *)(pContourList->pFirst);

	BYTE *pContourElement, *pContourElement0;
	unsigned char *PixArray = (unsigned char *)(pDisplay->imageData);
	unsigned char *pPix;
	int iNeighbor, iNextNeighbor;
	int iPix;

	while(pContour)
	{
		pContourElement0 = pContour->pFirst;

		pContourElement = pContourElement0;

		iNeighbor = RVL2DCONTOUR_GET_EDGEID(pContourElement, ContourMap);

		do
		{
			iNextNeighbor = RVL2DCONTOUR_GET_NEXT_EDGEID(pContourElement);
			pContourElement = RVL2DCONTOUR_GET_NEXT(pContourElement, pAImage, iNeighbor, iNextNeighbor);
			iNeighbor = iNextNeighbor;

			iPix = RVL2DCONTOUR_GET_IPIX(pContourElement, ContourMap);

			pPix = PixArray + 3 * iPix;

			pPix[0] = 255;
			pPix[1] = 0;
			pPix[2] = 255;
		}
		while(((*pContourElement) & RVL2DCONTOUR_NEXT) != 0 && pContourElement != pContourElement0);
		
		pContour = (RVL2DCONTOUR_DATA2 *)(pContour->pNext);
	}
}