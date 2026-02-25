// RVL2DRegion2.cpp: implementation of the CRVL2DRegion2 class.
//
//////////////////////////////////////////////////////////////////////

#include "RVLCore.h"

CRVL2DRegion2 RVL2DRegionTemplate;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL2DRegion2::CRVL2DRegion2()
{

}

CRVL2DRegion2::~CRVL2DRegion2()
{

}


CRVLObject2 * CRVL2DRegion2::Create2(CRVLClass *pClass)
{
	CRVL2DRegion2 *pObject = (CRVL2DRegion2 *)(pClass->m_pMem0->Alloc(sizeof(CRVL2DRegion2)));

	memcpy(pObject, this, sizeof(CRVL2DRegion2));

	pObject->CRVLObject2::Create(pClass);

	pClass->Add(pObject);

	return pObject;		
}

void CRVL2DRegion2::Display(PIX_ARRAY *pIOut,
							void *vpAImage,	
							BYTE Flags)
{
	int *pPtArrayEnd = (int *)(m_PtArray) + m_nPts;

	int *piPix;
	int iPix;

	for(piPix = (int *)(m_PtArray); piPix < pPtArrayEnd; piPix++)
	{
		iPix = *piPix;

		DisplayPt(iPix, pIOut, vpAImage, Flags);
	}	
}


void CRVL2DRegion2::Display2(PIX_ARRAY *pIOut,
							void *vpAImage,	
							BYTE Flags)
{
	RVL2DREGION_PIXIDX *pPixPtr = (RVL2DREGION_PIXIDX *)m_PtArray;

	while(pPixPtr)
	{
		DisplayPt(pPixPtr->iPix, pIOut, vpAImage, Flags);

		pPixPtr = pPixPtr->pNext;
	}
}

void CRVL2DRegion2::DisplayPt(	int iPix,
								PIX_ARRAY *pIOut,
								void *vpAImage,	
								BYTE Flags)
{
	CRVLAImage *pAImage = (CRVLAImage *)vpAImage;

	RVLAPIX *pAPix = pAImage->m_pPix + iPix;

	unsigned char *pPixOut = pIOut->pPix + 3 * iPix;

	unsigned char I;
	BYTE mColor;

	if((pAPix->Flags & RVLAPIX_FLAG_EDGE_REJECTED) == 0 &&
		(pAPix->Flags & (RVLAPIX_FLAG_EDGE | RVLAPIX_FLAG_VIRTUAL_EDGE)) != 0 &&
		(Flags & RVL2DREGION_DISPLAY_FLAG_BOUNDARY) != 0)
	{
		*(pPixOut++) = 255;
		*(pPixOut++) = 255;
		*(pPixOut++) = 0;				
	}
	else
	{
		I = 64 + (unsigned char)((((int)pAPix->I) * 3) >> 2);

		for(mColor = 0x01; mColor <= 0x04; mColor = (mColor << 1))
			*(pPixOut++) = (Flags & mColor ? I : 0);
	}
}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 


void RVLEdgeBasedImageSegmentation(void *vpAImage,
								   int DepthThr,
								   int *RegionGrowingMap,
								   int *Segment,
								   int *RegionGrowingBuff,
								   int *RegionGrowingBuff2,
								   int *EmptyRegionGrowingMap
								   )
{
	CRVLAImage *pAImage = (CRVLAImage *)vpAImage;

	int w = pAImage->m_Width;
	int h = pAImage->m_Height;

	int ImageSize = w * h;

	int *pRGBuff = RegionGrowingBuff;

	int iPix;
	RVLAPIX *pAPix;

	// copy edge points to RegionGrowingBuff

	for(iPix = 0; iPix < ImageSize; iPix++)
	{
		pAPix = pAImage->m_pPix + iPix;

		if(pAPix->Flags & RVLAPIX_FLAG_EDGE)
			*(pRGBuff++) = iPix;
	}

	int n = pRGBuff - RegionGrowingBuff;

	memcpy(RegionGrowingMap, EmptyRegionGrowingMap, ImageSize * sizeof(int));

	// dilate edge map

	int *dpNeighbor4 = pAImage->m_dpNeighbor4;

	int iRG1 = n;
	int iRG2 = 0;
	int iRG3 = n;

	int depth = 1;

	int j;
	int iPix1, iPix2;

	while(iRG2 < iRG1)
	{
		iPix2 = RegionGrowingBuff[iRG2];

		for(j = 0; j < 4; j++)	// for each point in 4-neighborhood 
		{
			iPix1 = iPix2 + dpNeighbor4[j];

			if(RegionGrowingMap[iPix1] == -1)
			{
				RegionGrowingMap[iPix1] = depth;

				RegionGrowingBuff[iRG1++] = iPix1;
			}
			
		}	// for each point in 4-neighborhood of pPt2
		
		iRG2++;

		if(iRG2 == iRG3)
		{
			depth++;

			if(depth > DepthThr)
				break;

			iRG3 = iRG1;
		}
	}	// region growing loop	

	// dilating high-depth regions

	pRGBuff = RegionGrowingBuff;

	for(iPix = 0; iPix < ImageSize; iPix++)
	{
		if(RegionGrowingMap[iPix] == -1)
		{
			RegionGrowingMap[iPix] = 0;

			*(pRGBuff++) = iPix;
		}
		else if(RegionGrowingMap[iPix] != 0xfefefefe)
			RegionGrowingMap[iPix] = -1;
	}

	n = pRGBuff - RegionGrowingBuff;

	iRG1 = n;
	iRG2 = 0;
	iRG3 = n;

	depth = 1;

	int kDilation = 2;

	while(iRG2 < iRG1)
	{
		iPix2 = RegionGrowingBuff[iRG2];

		for(j = 0; j < 4; j++)	// for each point in 4-neighborhood 
		{
			iPix1 = iPix2 + dpNeighbor4[j];

			if(RegionGrowingMap[iPix1] == -1)
			{
				RegionGrowingMap[iPix1] = depth;

				RegionGrowingBuff[iRG1++] = iPix1;
			}
			
		}	// for each point in 4-neighborhood of pPt2
		
		iRG2++;

		if(iRG2 == iRG3)
		{
			depth++;

			if(depth > kDilation)
				break;

			iRG3 = iRG1;
		}
	}	// region growing loop	
	
	// segment detection

	memcpy(Segment, EmptyRegionGrowingMap, ImageSize * sizeof(int));

	WORD SegmentID = 0;

	pRGBuff = RegionGrowingBuff2;

	BOOL bSegmentBorder;

	for(iPix = 0; iPix < ImageSize; iPix++)
	{
		if(RegionGrowingMap[iPix] == -1)
			continue;

		if(Segment[iPix] != -1)
			continue;

		iPix1 = iPix;

		RegionGrowingBuff[0] = iPix1;

		iRG1 = 1;

		iRG2 = 0;

		Segment[iPix1] = SegmentID;

		while(iRG2 < iRG1)
		{
			iPix2 = RegionGrowingBuff[iRG2];

			bSegmentBorder = FALSE;

			for(j = 0; j < 4; j++)	// for each point in 4-neighborhood 
			{
				iPix1 = iPix2 + dpNeighbor4[j];

				if(RegionGrowingMap[iPix1] == -1)
				{
					bSegmentBorder = TRUE;

					continue;
				}

				if(Segment[iPix1] != -1)
					continue;

				Segment[iPix1] = SegmentID;

				RegionGrowingBuff[iRG1++] = iPix1;				
			}	// for each point in 4-neighborhood of pPt2

			if(bSegmentBorder)
				*(pRGBuff++) = iPix2;
			
			iRG2++;
		}	// region growing loop		

		SegmentID++;
	}

	n = pRGBuff - RegionGrowingBuff2;

	// segment growing

	iRG1 = n;
	iRG2 = 0;
	iRG3 = n;

	depth = 1;

	//int DepthThr2 = DepthThr - kDilation;

	pAPix = pAImage->m_pPix;

	while(iRG2 < iRG1)
	{
		iPix2 = RegionGrowingBuff2[iRG2];

		SegmentID = Segment[iPix2];

		for(j = 0; j < 4; j++)	// for each point in 4-neighborhood 
		{
			iPix1 = iPix2 + dpNeighbor4[j];

			if(pAPix[iPix1].Flags & RVLAPIX_FLAG_EDGE)
				continue;

			if(Segment[iPix1] == -1)
			{
				Segment[iPix1] = SegmentID;

				RegionGrowingBuff2[iRG1++] = iPix1;
			}
			
		}	// for each point in 4-neighborhood of pPt2
		
		iRG2++;

		if(iRG2 == iRG3)
		{
			depth++;

			//if(depth > DepthThr2)
			if(depth > DepthThr)
				break;

			iRG3 = iRG1;
		}
	}	// region growing loop		
}

void RVLEdgeBasedSegmentationDisplay(	CRVLGUI *pGUI,
										CRVLFigure *pFig,
										//int iFirstTextLine,
										WORD *Segment,
										void *vpAImage,
										RVLAPIX *pSelectedAPix,
										PIX_ARRAY *pIOut
										)
{
	CRVLAImage *pAImage = (CRVLAImage *)vpAImage;

	RVLAPIX *pAPix = pAImage->m_pPix;

	int w = pAImage->m_Width;
	int h = pAImage->m_Height;

	unsigned char *pPixOut = pIOut->pPix;

	int SlectedSegmentID;
	
	if(pSelectedAPix)
	{
		SlectedSegmentID = Segment[pSelectedAPix - pAImage->m_pPix];

		if(SlectedSegmentID == -1)
			SlectedSegmentID = 0xfffffffe;
	}
	else
		SlectedSegmentID = 0xfffffffe;

	int u, v;
	unsigned char I;
	int iPix;

	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pAPix++)
		{
			iPix = u + v * w;

			I = pAPix->I;

			if(pAPix->Flags & RVLAPIX_FLAG_EDGE)
			{
				if(Segment[iPix] == -1)
				{
					*(pPixOut++) = 255;
					*(pPixOut++) = 0;
					*(pPixOut++) = 255;
				}
				else
				{
					*(pPixOut++) = 0;
					*(pPixOut++) = 255;
					*(pPixOut++) = 255;
				}
			}
			else if(Segment[iPix] == SlectedSegmentID)
			{
				*(pPixOut++) = 64 + (unsigned char)((((int)I) * 3) >> 2);
				*(pPixOut++) = 64 + (unsigned char)((((int)I) * 3) >> 2);
				*(pPixOut++) = 0;
			}
			else if(Segment[iPix] == -1)
			{
				*(pPixOut++) = 0;
				*(pPixOut++) = 64 + (unsigned char)((((int)I) * 3) >> 2);
				*(pPixOut++) = 0;
			}
			else if(Segment[iPix] == 0xfefefefe)
			{
				*(pPixOut++) = 0;
				*(pPixOut++) = 0;
				*(pPixOut++) = 255;
			}
			else
			{
				*(pPixOut++) = I;
				*(pPixOut++) = I;
				*(pPixOut++) = I;
			}
		}
	}
}


void RVLDisplay2DRegions(CRVLFigure *pFig,
						 CRVLMPtrChain *p2DRegionSet,
						 int ImageWidth,
						 RVLCOLOR Color,
						 int LineWidth,
						 DWORD mMask1, DWORD mMask2)
{	
	CRVL2DRegion2 *pRegion;

	p2DRegionSet->Start();

	while(p2DRegionSet->m_pNext)
	{
		pRegion = (CRVL2DRegion2 *)(p2DRegionSet->GetNext());

		if(pRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		RVLDisplay2DRegion(pFig, (RVLMESH_LINK *)(pRegion->m_PtArray), ImageWidth, Color,
			LineWidth, mMask1, mMask2);
	}
}

void RVLDisplayConvex(	 CRVLFigure *pFig,
						 CRVLMPtrChain *p2DRegionSet,
						 int ImageWidth,
						 RVLCOLOR ForegroundColor,
						 RVLCOLOR BackgroundColor)
{	
	CRVL2DRegion2 *pRegion;

	p2DRegionSet->Start();

	while(p2DRegionSet->m_pNext)
	{
		pRegion = (CRVL2DRegion2 *)(p2DRegionSet->GetNext());

		if(pRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if(pRegion->m_N[2] > 0)
			RVLDisplay2DRegion(pFig, (RVLMESH_LINK *)(pRegion->m_PtArray), ImageWidth, BackgroundColor);
	}

	p2DRegionSet->Start();

	while(p2DRegionSet->m_pNext)
	{
		pRegion = (CRVL2DRegion2 *)(p2DRegionSet->GetNext());

		if(pRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if(pRegion->m_N[2] <= 0)
			RVLDisplay2DRegion(pFig, (RVLMESH_LINK *)(pRegion->m_PtArray), ImageWidth, ForegroundColor);
	}
}


CRVL2DRegion2 *RVLSelectTriangle(int iPix,
								 int ImageWidth,
								 CRVLMPtrChain *p2DRegionSet)
{
	double u0 = (double)(iPix % ImageWidth);
	double v0 = (double)(iPix / ImageWidth);

	CRVL2DRegion2 *pRegion;
	double u[3], v[3];
	int iPt;
	RVLMESH_LINK *pLink;

	p2DRegionSet->Start();

	while(p2DRegionSet->m_pNext)
	{
		pRegion = (CRVL2DRegion2 *)(p2DRegionSet->GetNext());

		if(pRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pLink = (RVLMESH_LINK *)(pRegion->m_PtArray);

		for(iPt = 0; iPt < 3; iPt++)
		{
			u[iPt] = (double)(pLink->iPix0 % ImageWidth);
			v[iPt] = (double)(pLink->iPix0 / ImageWidth);

			pLink = pLink->pNext->pOpposite;
		}

		if(RVLInsideTriangle(u[0], v[0], u[2], v[2], u[1], v[1], u0, v0))
			return pRegion;
	}

	return NULL;
}

void RVL2DRegionResetLabels(CRVLMPtrChain *pTriangleList)
{
	CRVL2DRegion2 *pTriangle;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		pTriangle->m_Label = -1;
	}
}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 



int RVLGetSignature(RVLAPIX_2DREGION_PTR *TextonMap,
					int Width,
					RVLRECT *pROI,
					RVLSIGNATURE_ELEMENT *Signature,
					RVLSIGNATURE_ELEMENT **TextonSignatureMap)
{
	RVLSIGNATURE_ELEMENT *pNewSignatureElement = Signature;

	int u, v;
	CRVL2DRegion2 *pTexton;
	RVLSIGNATURE_ELEMENT *pSignatureElement;

	for(v = pROI->top; v <= pROI->bottom; v++)
		for(u = pROI->left; u <= pROI->right; u++)
		{
			pTexton = TextonMap[u + v * Width].p2DRegion;

			pSignatureElement = TextonSignatureMap[pTexton->m_Index];

			if(pSignatureElement == NULL)
			{
				pSignatureElement = pNewSignatureElement;
				pSignatureElement->pTexton = pTexton;
				pSignatureElement->n = 0;
				pNewSignatureElement++;
			}

			pSignatureElement->n++;
		}

	for(pSignatureElement = Signature; pSignatureElement < pNewSignatureElement; pSignatureElement++)
		TextonSignatureMap[pSignatureElement->pTexton->m_Index] = NULL;

	return pNewSignatureElement - Signature;
}

void RVLDisplayTextons(RVLQLIST *pTextonList,
					   double res,
					   double minuTxt,
					   double minvTxt,
					   double hTxt,
					   IplImage *pOutImage,
					   CRVLMem *pMem2)
{
	BYTE *pFreeMem = pMem2->m_pFreeMem;

	BYTE *pStartBlock = pMem2->m_pStartBlock;

	CRVLMChain IPArray(pMem2, sizeof(RVLIPOINT));

	CvPoint *PtArray;
	int nEllipsePts;
	CvPoint *pPt;
	RVLIPOINT *pIP;

	/*

	RVL2DMOMENTS *pMoments;
	double fn;
	double HRGBF[3 * 3];

	double J[2 * 2], CF[2 * 2];
	double *CRGB;
	double detCF;
	double r1, r2, phi;
	double a[3];

	InverseMatrix3(HRGBF, HFRGB);

	CRVL2DRegion2 **pTextonListEnd = ppTexton;

	for(ppTexton = TextonList; ppTexton < pTextonListEnd; ppTexton++)	// for every texton on p3DSurface
	{
		pTexton = *ppTexton;

		pTexton->m_Flags &= ~RVLOBJ2_FLAG_MARKED;

		// compute texton center PF

		pMoments = &(pTexton->m_Moments);

		if(pMoments->n == 0)
			continue;

		fn = (double)(pMoments->n);

		URGB[0] = pMoments->S[0];
		URGB[1] = pMoments->S[1];
		URGB[2] = 1.0;

		RVLMULMX3X3VECT(HRGBF, URGB, a)

		PF[0] = a[0] / a[2];
		PF[1] = a[1] / a[2];

		// jacobian for texton point distribution transformation 
		// (The mathematics for this computation is given in RVLMath.doc, 
		// chapter "Transformation of the Uncertainty of a 2D Point Mapped by a Homography")
		
		RVLMXEL(J, 2, 0, 0) = (RVLMXEL(HRGBF, 3, 0, 0) - a[0] * RVLMXEL(HRGBF, 3, 2, 0) / a[2]) / a[2];
		RVLMXEL(J, 2, 0, 1) = (RVLMXEL(HRGBF, 3, 0, 1) - a[0] * RVLMXEL(HRGBF, 3, 2, 1) / a[2]) / a[2];
		RVLMXEL(J, 2, 1, 0) = (RVLMXEL(HRGBF, 3, 1, 0) - a[1] * RVLMXEL(HRGBF, 3, 2, 0) / a[2]) / a[2];
		RVLMXEL(J, 2, 1, 1) = (RVLMXEL(HRGBF, 3, 1, 1) - a[1] * RVLMXEL(HRGBF, 3, 2, 1) / a[2]) / a[2];

		// transform the texton point distribution

		CRGB = pMoments->S2;

		RVLCOV2DTRANSF(CRGB, J, CF)

		// draw texton ellipse

		detCF = RVLDET2(CF);

		GetEllipseParams(CF[3]/detCF, -CF[1]/detCF, CF[0]/detCF, r1, r2, phi);

		IPArray.RemoveAll();

		RVLDrawEllipse(PF[0]/res-(double)minuTxt, PF[1]/res-(double)minvTxt,
			1.0*r1/res, 1.0*r2/res, phi, 1.0, &IPArray);

		nEllipsePts = IPArray.m_nElements;

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, CvPoint, nEllipsePts, PtArray)

		if(pMem2->m_pStartBlock != pStartBlock)
		{
			pStartBlock = pMem2->m_pStartBlock;

			pFreeMem = pStartBlock + sizeof(BYTE *);
		}

		pPt = PtArray;

		IPArray.Start();

		while(IPArray.m_pNext)
		{
			pIP = (RVLIPOINT *)(IPArray.GetNext());

			pPt->x = pIP->u;
			pPt->y = hTxt - 1 - pIP->v;

			pPt++;
		}

		cvPolyLine(pOutImage, &PtArray, &nEllipsePts, 1, 1, cvScalar(0, 0, 255));

		pMem2->m_pFreeMem = pFreeMem;
	}
	*/	

	RVLTEXTON *pTexton = (RVLTEXTON *)(pTextonList->pFirst);

	while(pTexton)
	{
		IPArray.RemoveAll();

		RVLDrawEllipse(pTexton->x/res-(double)minuTxt, pTexton->y/res-(double)minvTxt,
			1.0*pTexton->Elongation*pTexton->Size/res, 1.0*pTexton->Size/res, 
			pTexton->Orientation, 1.0, &IPArray);

		nEllipsePts = IPArray.m_nElements;

		RVLMEM_ALLOC_STRUCT_ARRAY(pMem2, CvPoint, nEllipsePts, PtArray)

		if(pMem2->m_pStartBlock != pStartBlock)
		{
			pStartBlock = pMem2->m_pStartBlock;

			pFreeMem = pStartBlock + sizeof(BYTE *);
		}

		pPt = PtArray;

		IPArray.Start();

		while(IPArray.m_pNext)
		{
			pIP = (RVLIPOINT *)(IPArray.GetNext());

			pPt->x = pIP->u;
			pPt->y = hTxt - 1 - pIP->v;

			pPt++;
		}

		cvPolyLine(pOutImage, &PtArray, &nEllipsePts, 1, 1, cvScalar(0, 0, 255));

		pMem2->m_pFreeMem = pFreeMem;			

		pTexton = (RVLTEXTON *)(pTexton->pNext);
	}
}

void RVLDrawEllipse(double u0, double v0, 
			     	double r1, double r2, 
					double phi, 
					double maxErr,
					CRVLMChain *pIPArray)
{
	int i;
	RVLIPOINT IP;
	double gamma, p, q, fu, fv;

	int n = DOUBLE2INT(PI / (2 * acos(1.0 - maxErr / r1))) + 1;

	double dgamma = PI / (double)n;

	double cs = cos(phi);
	double sn = sin(phi);

	for(i = 0; i < 2 * n; i++)
	{
		gamma = (double)i*dgamma;
		p = r1 * cos(gamma);
		q = r2 * sin(gamma);
		LinearTransform2D2(cs, sn, u0, v0, p, q, fu, fv);

		IP.u = DOUBLE2INT(fu);
		IP.v = DOUBLE2INT(fv);

		pIPArray->Add(&IP);
	}
}
