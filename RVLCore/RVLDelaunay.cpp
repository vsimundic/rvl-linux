#include "RVLCore.h"
#include "RVLEDT.h"
#include "RVLDelaunay.h"
#include "RVLClass.h"
#include "RVL2DRegion2.h"

CRVLDelaunay::CRVLDelaunay(void)
{
	m_DelaunayMap = NULL;
	m_DelaunayData = NULL;
	m_NeighborAngleArray = NULL;
	m_pROI = NULL;
	m_VertexListData = NULL;
}

CRVLDelaunay::~CRVLDelaunay(void)
{
	Clear();
}


void CRVLDelaunay::Init()
{
	int nPts = m_Width * m_Height;

	m_DelaunayMap = new BYTE *[nPts];

	m_DelaunayData = new BYTE[nPts * sizeof(short) + 8 * nPts * sizeof(RVLMESH_LINK)];

	m_NeighborAngleArray = new RVLDELAUNAY_NEIGHBOR_ANGLE[2 * nPts];
}


void CRVLDelaunay::Clear(void)
{
	if(m_DelaunayMap)
		delete[] m_DelaunayMap;

	if(m_DelaunayData)
		delete[] m_DelaunayData;

	if(m_NeighborAngleArray)
		delete[] m_NeighborAngleArray;
}

void CRVLDelaunay::Apply(RVLEDT_PIX_ARRAY *pEDTImage)
{
	int w = pEDTImage->Width;
	int h = pEDTImage->Height;

	int nPix = w * h;

	RVLRECT ROI;
	RVLRECT *pROI;

	if(m_pROI)
		pROI = m_pROI;
	else
	{
		ROI.left = 0;
		ROI.right = w - 1;
		ROI.top = 0;
		ROI.bottom = h - 1;

		pROI = &ROI;
	}	

	RVLEDT_PIX *EDTPixArray = pEDTImage->pPix;

	int *iPixBuff2 = new int[nPix * 6 + 1];

	int *piPixBuff2 = iPixBuff2;

	BYTE *Flags = (BYTE *)calloc(nPix, sizeof(BYTE));

	m_nLinks = 0;

	// detect boundaries of Voronoi cells

	int iPixVC, iPixVC2, iPixVC3, iPix, iPix2, iPix0;
	RVLEDT_PIX *pEDTPix, *pEDTPix2, *pEDTPix3;
	BYTE iDirection, iDirection0;
	//BYTE iDirection2;
	short *pnNeighbors;
	int *pnAdjacentPix;
	int u0, v0, du, dv, du0, dv0;
	int dot1, dot2, dot3;
	signed char quadrant, dquadrant;
	short iNeighbor, iNeighbor2, diNeighbor, iNeighbor3;
	short nNeighbors3;
	RVLMESH_LINK *NeighborArray;
	BYTE *pNeighborData;
	RVLMESH_LINK *pConnection, *pConnection3;
	RVLDELAUNAY_NEIGHBOR_ANGLE *pNeighborAngle;
	BOOL bImageBorder;
	BOOL bInit;
	int *piPixBuff, *piPixBuffEnd;
	int *pFirstiPix;
	int u, v;

	for(v = pROI->top; v <= pROI->bottom; v++)	
	{
		iPixVC = v * w + pROI->left;

		for(u = pROI->left; u <= pROI->right; u++, iPixVC++)	
		{

			//if(iPixVC == 22829)
			//	int tmp1 = 0;

			pEDTPix = EDTPixArray + iPixVC;

			if(pEDTPix->d2 > 0)
				continue;

			*(piPixBuff2++) = iPixVC;

			pnAdjacentPix = piPixBuff2;

			piPixBuff2++;

			pFirstiPix = piPixBuff2;

			// go to the boundary of the Voronoi cell

			iPix2 = iPixVC + 1;

			if(iPix2 % w)
			{
				while(TRUE)
				{
					pEDTPix2 = EDTPixArray + iPix2;

					if(pEDTPix2->d2 == 0xffffffff)
					{
						iPixVC3 = -1;

						break;
					}

					iPixVC3 = iPix2 - ((int)(pEDTPix2->dx) + (int)(pEDTPix2->dz) * w);

					if(iPixVC3 != iPixVC)
						break;

					iPix2++;
				}
			}
			else
				iPixVC3 = -1;

			iDirection0 = iDirection = 0;
		
			iPix0 = iPix = iPix2 - 1;		

			bInit = FALSE;

			// follow the boundary of the Voronoi cell

			while(TRUE)
			{
				// get next boundary element

				if(iPix == iPix0)
					if(iDirection == iDirection0)
						if(bInit)
							break;

				Flags[iPix] = 0x02;

				bInit = TRUE;

				iDirection = ((iDirection + 3) & 3);

				while(TRUE)
				{
					switch(iDirection){
					case 0:
						iPix2 = iPix + 1;

						bImageBorder = (iPix2 % w == 0);

						break;
					case 1:
						iPix2 = iPix - w;

						bImageBorder = (iPix2 < 0);

						break;
					case 2:
						iPix2 = iPix - 1;

						bImageBorder = (iPix % w == 0);

						break;
					case 3:
						iPix2 = iPix + w;

						bImageBorder = (iPix2 >= nPix);
					}

					if(bImageBorder)
					{
						iPixVC3 = -1;

						break;
					}

					pEDTPix3 = EDTPixArray + iPix2;

					if(pEDTPix3->d2 == 0xffffffff)
					{
						iPixVC3 = -1;

						break;
					}

					iPixVC3 = iPix2 - ((int)(pEDTPix3->dx) + (int)(pEDTPix3->dz) * w);
					
					if(iPixVC3 != iPixVC)
						break;

					iPix = iPix2;

					iDirection = ((iDirection + 1) & 3);
				}	// while(TRUE)

				*(piPixBuff2++) = (iPixVC3 >= 0 ? iPix2 : -1);
			}	// while(TRUE)

			*pnAdjacentPix = piPixBuff2 - pFirstiPix;
		}	
	}// for every pixel in ROI

	*piPixBuff2 = -1;

	// create links between neighboring points

	BYTE *pDelaunayData = m_DelaunayData;

	memset(m_DelaunayMap, 0, nPix * sizeof(BYTE *));

	memset(m_DelaunayData, 0, nPix * sizeof(short) + 8 * nPix * sizeof(RVLMESH_LINK));

	int *iPixBuff = new int[nPix];

	piPixBuff2 = iPixBuff2;

	BOOL bTmp = FALSE;

	int nAdjacentPix;
	int *piPixBuff2End;
	RVLMESH_LINK *pConnectionPrev;

	while((iPixVC = (*(piPixBuff2++))) >= 0)	
	{
		//if(iPixVC == 21227)
		//	int tmp1 = 0;

		nAdjacentPix = (*(piPixBuff2++));

		piPixBuff = iPixBuff;

		m_DelaunayMap[iPixVC] = pDelaunayData;

		pnNeighbors = (short *)pDelaunayData;

		pDelaunayData += sizeof(short);

		NeighborArray = pConnection = (RVLMESH_LINK *)pDelaunayData;

		u0 = iPixVC % w;
		v0 = iPixVC / w;

		iPix2 = *(piPixBuff2 + nAdjacentPix - 1);

		if(iPix2 >= 0)
		{
			if(Flags[iPix2] == 0x02)
			{
				pEDTPix3 = EDTPixArray + iPix2;

				iPixVC2 = iPix2 - ((int)(pEDTPix3->dx) + (int)(pEDTPix3->dz) * w);
			}
			else
				iPixVC2 = -1;
		}
		else
			iPixVC2 = -1;

		iNeighbor = 0;

		piPixBuff2End = piPixBuff2 + nAdjacentPix;

		pConnectionPrev = NULL;

		for(; piPixBuff2 < piPixBuff2End; piPixBuff2++)	// for every adjacent pixel of the Voronoi cell
		{
			iPix2 = *piPixBuff2;

			if(iPix2 >= 0)
			{
				if(Flags[iPix2] == 0x02)
				{
					pEDTPix3 = EDTPixArray + iPix2;

					iPixVC3 = iPix2 - ((int)(pEDTPix3->dx) + (int)(pEDTPix3->dz) * w);

					if((Flags[iPixVC3] & 0x01) == 0)
					{
						// create new neighbor connection

						if(iPixVC3 != iPixVC2)
						{
							du = iPixVC3 % w - u0;
							dv = iPixVC3 / w - v0;

							if(iNeighbor)
							{
								dot1 = du * du0 + dv * dv0;
								dot2 = -du * dv0 + dv * du0;

								for(iNeighbor2 = iNeighbor - 1; iNeighbor2 >= 0; iNeighbor2--)
								{
									pNeighborAngle = m_NeighborAngleArray + iNeighbor2;

									if(dot1 >= 0)
										quadrant = (dot2 >= 0 ? 0 : 3);
									else
										quadrant = (dot2 >= 0 ? 1 : 2);

									dquadrant = quadrant - pNeighborAngle->quadrant;

									if(dquadrant > 0)
										break;
									else if(dquadrant == 0)
									{
										dot3 = -du * pNeighborAngle->dv + dv * pNeighborAngle->du;

										if(dot3 > 0)
											break;
										else if(dot3 == 0)
											int tmp1 = 0;
									}
								}

								iNeighbor2++;

								pConnection = NeighborArray + iNeighbor2;

								pNeighborAngle++;

								diNeighbor = iNeighbor - iNeighbor2;
						
								if(diNeighbor)
								{
									memmove(pConnection + 1, pConnection, 
										diNeighbor * sizeof(RVLMESH_LINK));

									memmove(pNeighborAngle + 1, pNeighborAngle, 
										diNeighbor * sizeof(RVLDELAUNAY_NEIGHBOR_ANGLE));
								}
							}	// if(iNeighbor)
							else
							{
								du0 = du;
								dv0 = dv;
								quadrant = 0;
								pNeighborAngle = m_NeighborAngleArray;
								iNeighbor2 = 0;
							}

							pConnection->iPix0 = iPixVC;

							pConnection->iPix = iPixVC3;

							pConnection->du = du;
							pConnection->dv = dv;

							if(pConnectionPrev)
							{
								pConnection->pPrev = pConnectionPrev;
	
								pConnectionPrev->pNext = pConnection;
							}

							pConnectionPrev = pConnection;

							if(iPixVC3 < iPixVC)
							{
								pNeighborData = m_DelaunayMap[iPixVC3];

								nNeighbors3 = *((short *)pNeighborData);

								pConnection3 = (RVLMESH_LINK *)(pNeighborData + sizeof(short));

								for(iNeighbor3 = 0; iNeighbor3 < nNeighbors3; iNeighbor3++)
								{
									if(pConnection3->iPix == iPixVC)
										break;

									pConnection3++;
								}

								if(iNeighbor3 >= nNeighbors3)
									int tmp1 = 0;

								pConnection3->iConnection = iNeighbor2;

								pConnection3->pOpposite = pConnection;

								pConnection->iConnection = iNeighbor3;

								pConnection->pOpposite = pConnection3;
							}

							pNeighborAngle->du = du;
							pNeighborAngle->dv = dv;
							pNeighborAngle->quadrant = quadrant;

							m_nLinks++;

							iNeighbor++;

							iPixVC2 = iPixVC3;

							Flags[iPixVC3] |= 0x01;

							*(piPixBuff++) = iPixVC3;
						}	// if(iPixVC3 != iPixVC2)
					}	// if((Flags[iPixVC3] & 0x01) == 0)
				}	// if(Flags[iPix2] == 0x02)
			}	// if(iPix2 >= 0)
		}	// for every adjacent pixel of the Voronoi cell

		NeighborArray->pPrev = pConnection;

		pConnection->pNext = NeighborArray;

		piPixBuffEnd = piPixBuff;

		for(piPixBuff = iPixBuff; piPixBuff < piPixBuffEnd; piPixBuff++)
			Flags[*piPixBuff] &= ~0x01;

		*pnNeighbors = iNeighbor;

		pDelaunayData += (iNeighbor * sizeof(RVLMESH_LINK));

		//if(iPixVC == 21281)
		//{
		//	int tmp1 = 0;
		//	bTmp = TRUE;
		//}

		//if(bTmp && ((RVLMESH_LINK *)(m_DelaunayData + 46130))->iPix > w * h)
		//	int tmp1 = 0;
	}	// while(*piPixBuff2 >= 0)

	delete[] iPixBuff2;
	free(Flags);
	delete[] iPixBuff;
}	// CRVLDelaunay::Apply()



void CRVLDelaunay::Save(FILE * fp, BYTE mMask)
{
	int nPix = m_Width * m_Height;

	BYTE **ppDelaunayData = m_DelaunayMap;

	BYTE *pDelaunayData;
	short iNeighbor, nNeighbors;
	int iPix, iPix2;
	RVLMESH_LINK *pConnection;
	int u1, v1, u2, v2;

	for(iPix = 0; iPix < nPix; iPix++, ppDelaunayData++)
	{
		pDelaunayData = *ppDelaunayData;

		if(pDelaunayData == NULL)
			continue;

		nNeighbors = *((short *)pDelaunayData);

		pConnection = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		for(iNeighbor = 0; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
		{
			if((pConnection->Flags & mMask) != 0 || mMask == 0xff)
			{
				u1 = iPix % m_Width;
				v1 = iPix / m_Width;
				u2 = u1 + pConnection->du;
				v2 = v1 + pConnection->dv;
				iPix2 = u2 + v2 * m_Width;

				if(iPix2 > iPix)
					fprintf(fp, "%d\t%d\t%d\t%d\n", u1, v1,	u2, v2);
			}
		}
	}
}

BOOL CRVLDelaunay::Test(void)
{
	int nPix = m_Width * m_Height;

	BYTE **ppDelaunayData = m_DelaunayMap;

	BYTE *pDelaunayData;
	short iNeighbor, nNeighbors;
	RVLMESH_LINK *pConnection, *pConnection2;
	RVLMESH_LINK *ConnectionArray, *ConnectionArray2;
	int iPix, iPix2;
	int u, v, u0, v0, du, dv;
	double l, cs0, sn0, fdu, fdv;
	double phi, phiPrev;
	double p, q;

	for(iPix = 0; iPix < nPix; iPix++, ppDelaunayData++)
	{
		pDelaunayData = *ppDelaunayData;

		if(pDelaunayData == NULL)
			continue;

		u0 = iPix % m_Width;
		v0 = iPix / m_Width;

		nNeighbors = *((short *)pDelaunayData);

		ConnectionArray = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		pConnection = ConnectionArray;

		iPix2 = pConnection->iPix;

		pDelaunayData = m_DelaunayMap[iPix2];

		ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		pConnection2 = ConnectionArray2 + pConnection->iConnection;

		if(pConnection2->iPix != iPix)
			return FALSE;

		if(pConnection2->iConnection != 0)
			return FALSE;

		pConnection++;

		u = iPix2 % m_Width;
		v = iPix2 / m_Width;		

		du = u - u0;
		dv = v - v0;
		l = sqrt((double)(du * du + dv * dv));
		
		cs0 = (double)du / l;
		sn0 = (double)dv / l;

		phiPrev = 0.0;

		for(iNeighbor = 1; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
		{
			iPix2 = pConnection->iPix;

			u = iPix2 % m_Width;
			v = iPix2 / m_Width;		

			du = u - u0;
			dv = v - v0;

			fdu = (double)du;
			fdv = (double)dv;
			
			p = fdu * cs0 + fdv * sn0;
			q = -fdu * sn0 + fdv * cs0;

			phi = atan2(q, p);
	
			if(phi < 0.0)
				phi += 2 * PI;

			if(phi <= phiPrev)
				return FALSE;

			phiPrev = phi;

			pDelaunayData = m_DelaunayMap[iPix2];

			ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

			pConnection2 = ConnectionArray2 + pConnection->iConnection;

			if(pConnection2->iPix != iPix)
				//return FALSE;
				int tmp1 = 0;

			if(pConnection2->iConnection != iNeighbor)
				//return FALSE;
				int tmp1 = 0;
		}
	}

	return TRUE;
}

BOOL CRVLDelaunay::Test(RVLMESH_LINK *pLink)
{
	RVLMESH_LINK *pLink0 = pLink;

	do
	{
		if(pLink->vp2DRegion)
		{
			if(-pLink->dv * pLink->pNext->du + pLink->du * pLink->pNext->dv < 0)
				return FALSE;
		}

		pLink = pLink->pNext;
	}
	while(pLink != pLink0);

	return TRUE;
}

BOOL CRVLDelaunay::Test3()
{
	RVLQLIST_PTR_ENTRY *pVertex = (RVLQLIST_PTR_ENTRY *)(m_VertexList.pFirst);

	int WDCounter;

	RVLMESH_LINK *pLink0, *pLink;

	while(pVertex)
	{
		WDCounter = 0;

		pLink0 = pLink = (RVLMESH_LINK *)(pVertex->Ptr);

		do
		{
			pLink = pLink->pNext;

			WDCounter++;
		}
		while(pLink != pLink0 && WDCounter < m_nVertices);

		if(WDCounter == m_nVertices)
			return FALSE;

		pVertex = (RVLQLIST_PTR_ENTRY *)(pVertex->pNext);
	}
	
	return TRUE;
}


int * CRVLDelaunay::SelectRegion(	int iSelectedPix,
									BYTE BoundaryFlags,								
									int &nPts,
									CRVLMem *pMem)
{
	int iPix = iSelectedPix;

	int v0 = iPix / m_Width;

	while(m_DelaunayMap[iPix] == NULL)
		iPix++;

	BYTE *pDelaunayData = m_DelaunayMap[iPix];

	int nDelaunayLinks = *((short *)pDelaunayData);

	RVLMESH_LINK *DelaunayLink = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

	RVLMESH_LINK *pDelaunayLink = DelaunayLink;

	RVLMESH_LINK *pDelaunayLink0 = NULL;

	int iConnection, iConnection0;

	for(iConnection = 0; iConnection < nDelaunayLinks; iConnection++, pDelaunayLink++)
		if(pDelaunayLink->Flags & BoundaryFlags)
		{
			if(pDelaunayLink0)
				if(pDelaunayLink0->iPix / m_Width > v0 && pDelaunayLink->iPix / m_Width < v0)
					break;

			iConnection0 = iConnection;

			pDelaunayLink0 = pDelaunayLink;
		}

	nPts = 0;

	if(pDelaunayLink0 == NULL)
		return NULL;

	if(pDelaunayLink0->iPix / m_Width <= v0)
		return NULL;

	if(iConnection == nDelaunayLinks)
	{
		pDelaunayLink = DelaunayLink;

		for(iConnection = 0; iConnection < nDelaunayLinks; iConnection++, pDelaunayLink++)
			if(pDelaunayLink->Flags & BoundaryFlags)
				break;
	}

	if(pDelaunayLink->iPix / m_Width >= v0)
		return NULL;

	int *iPixArray = (int *)(pMem->Alloc(m_Width * m_Height * sizeof(int)));

	int *piPix = iPixArray;

	RVLMESH_LINK *pDelaunayLinkIn = pDelaunayLink0;

	RVLMESH_LINK *pDelaunayLinkOut;

	do
	{
		pDelaunayLinkOut = pDelaunayLinkIn;

		do
			pDelaunayLinkOut = pDelaunayLinkOut->pNext;
		while((pDelaunayLinkOut->Flags & BoundaryFlags)==0);

		*(piPix++) = pDelaunayLinkOut->iPix;

		pDelaunayLinkIn = pDelaunayLinkOut->pOpposite;
	}	
	while(pDelaunayLinkIn != pDelaunayLink0);

	pMem->m_pFreeMem = (BYTE *)piPix;

	nPts = piPix - iPixArray;

	return iPixArray;
}


void CRVLDelaunay::GetPtsInConvexPolygon(	RVLMESH_LINK *pLink0,
											int * iScanLineStart, 
											int * iScanLineEnd, 
											int &iFirstScanLine,
											int &iLastScanLine)
{
	iFirstScanLine = m_Height - 1;
	iLastScanLine = 0;

	RVLMESH_LINK *pLink = pLink0;

	int iPix, u, v, du, dv;
	int vLast;
	int *piScanLine;
	int iPixCorr;
	int iPixNrm, diPixNrm;

	do
	{
		iPix = pLink->iPix0;

		u = iPix % m_Width;
		v = iPix / m_Width;

		if(v < iFirstScanLine)
			iFirstScanLine = v;

		if(v > iLastScanLine)
			iLastScanLine = v;

		dv = pLink->dv;

		if(dv != 0)
		{
			du = pLink->du;

			if(dv > 0)
			{
				vLast = v + dv;

				piScanLine = iScanLineEnd + v;

				iPixCorr = 0;
			}
			else
			{
				vLast = v;

				v += dv;

				du = -du;
				dv = -dv;

				piScanLine = iScanLineStart + v;

				iPixCorr = 1;

				iPix = pLink->pOpposite->iPix0;
			}

			diPixNrm = du + dv * m_Width;

			iPixNrm = iPix * dv;

			*(piScanLine++) = iPix;

			do
			{
				v++;

				iPixNrm += diPixNrm;
				iPix = iPixNrm / dv;

				if(iPixNrm % dv)
					iPix += iPixCorr;

				*(piScanLine++) = iPix;
			}
			while(v < vLast);
		}

		pLink = pLink->pNext->pOpposite;
	}
	while(pLink != pLink0);
}

void CRVLDelaunay::GetPtsInConvexHull(		RVLMESH_LINK *pLink0,
											int * iScanLineStart, 
											int * iScanLineEnd, 
											int &iFirstScanLine,
											int &iLastScanLine)
{
	iFirstScanLine = m_Height - 1;
	iLastScanLine = 0;

	RVLMESH_LINK *pLink = pLink0;

	while(pLink->Flags & RVLMESH_LINK_CONCAVITY)
	{
		do
			pLink = pLink->pNext;
		while((pLink->Flags & RVLMESH_LINK_FLAG_EDGE) == 0);
			
		pLink = pLink->pOpposite;
	}

	if(pLink == pLink0)
		return;

	pLink0 = pLink;

	int iPix, u, v, du, dv;
	int vLast;
	int *piScanLine;
	int iPixCorr;
	int iPixNrm, diPixNrm;

	do
	{
		iPix = pLink->iPix0;

		u = iPix % m_Width;
		v = iPix / m_Width;

		if(v < iFirstScanLine)
			iFirstScanLine = v;

		if(v > iLastScanLine)
			iLastScanLine = v;

		du = dv = 0;

		do
		{
			du += pLink->du;
			dv += pLink->dv;

			do
				pLink = pLink->pNext;
			while((pLink->Flags & RVLMESH_LINK_FLAG_EDGE) == 0);

			pLink = pLink->pOpposite;
		}
		while(pLink->Flags & RVLMESH_LINK_CONCAVITY);

		if(dv != 0)
		{
			if(dv > 0)
			{
				vLast = v + dv;

				piScanLine = iScanLineEnd + v;

				iPixCorr = 0;
			}
			else
			{
				vLast = v;

				v += dv;

				du = -du;
				dv = -dv;

				piScanLine = iScanLineStart + v;

				iPixCorr = 1;

				iPix = pLink->iPix0;
			}

			diPixNrm = du + dv * m_Width;

			iPixNrm = iPix * dv;

			*(piScanLine++) = iPix;

			do
			{
				v++;

				iPixNrm += diPixNrm;
				iPix = iPixNrm / dv;

				if(iPixNrm % dv)
					iPix += iPixCorr;

				*(piScanLine++) = iPix;
			}
			while(v < vLast);
		}
	}
	while(pLink != pLink0);
}

BOOL CRVLDelaunay::Test2()
{
	BYTE *bVisited = new BYTE[m_Width * m_Height];

	memset(bVisited, 0, sizeof(BYTE) * m_Width * m_Height);

	//int WDCounter;

	RVLMESH_LINK *pLink0, *pLink;

	RVLQLIST_PTR_ENTRY *pLinkPtr = (RVLQLIST_PTR_ENTRY *)(m_LinkList.pFirst);

	while(pLinkPtr)
	{
		pLink0 = (RVLMESH_LINK *)(pLinkPtr->Ptr);

		if((pLink0->Flags & RVLMESH_LINK_FLAG_REJECTED) == 0)
		{
			if(!bVisited[pLink0->iPix0])
			{
				bVisited[pLink0->iPix0] = TRUE;

				pLink = pLink0;

				//WDCounter = 0;

				do
				{
					if(-pLink->dv * pLink->pNext->du + pLink->du * pLink->pNext->dv < 0)
						if(pLink->vp2DRegion)
						{
							delete[] bVisited;

							return FALSE;
						}

					pLink = pLink->pNext;

					//WDCounter++;

					//if(WDCounter > 100)
					//	int tmp1 = 0;
				}
				while(pLink != pLink0);
			}

			if((pLink0->Flags & RVLMESH_LINK_FLAG_MARKED2) == 0)
			{
				if(pLink0->vp2DRegion)
				{
					pLink0->Flags |= RVLMESH_LINK_FLAG_MARKED2;

					if(!RVLPolygonConvexityTest(pLink0))
						return FALSE;
				}
			}
		}

		pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pLinkPtr->pNext);
	}

	delete[] bVisited;

	pLinkPtr = (RVLQLIST_PTR_ENTRY *)(m_LinkList.pFirst);

	while(pLinkPtr)
	{
		pLink = (RVLMESH_LINK *)(pLinkPtr->Ptr);

		pLink->Flags &= ~RVLMESH_LINK_FLAG_MARKED2;

		pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pLinkPtr->pNext);
	}

	return TRUE;
}

void CRVLDelaunay::DisplayRegion(	CRVLFigure *pFig,	
									int iSelectedPix,
									BYTE BoundaryFlags)
{
	int nPts;

	CRVLMem Mem;

	Mem.Create(1000000);

	int *iPixArray = SelectRegion(iSelectedPix, BoundaryFlags, nPts, &Mem);

	CRVLDisplayVector Vector(pFig->m_pMem);

	Vector.m_bClosed = FALSE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector.m_rL = 255;
	Vector.m_gL = 255;
	Vector.m_bL = 0;
	Vector.m_bClosed = TRUE;

	CRVLDisplayVector *pVector = pFig->AddVector(&Vector);

	int i;
	int u, v;

	for(i = 0; i < nPts; i++)
	{
		u = iPixArray[i] % m_Width;
		v = iPixArray[i] / m_Width;

		pVector->Point((u << 1), (v << 1));
	}	
}


void CRVLDelaunay::ResetFlags(DWORD mMask)
{
	RVLQLIST *pVertexList = &m_VertexList;

	void **ppVertex = &(pVertexList->pFirst);

	RVLQLIST_PTR_ENTRY *pVertex = (RVLQLIST_PTR_ENTRY *)(pVertexList->pFirst);

	RVLMESH_LINK *pLink0, *pLink;

	while(pVertex)
	{
		pLink0 = pLink = (RVLMESH_LINK *)(pVertex->Ptr);

		do
		{
			pLink->Flags &= ~mMask;

			pLink = pLink->pNext;
		}
		while(pLink != pLink0);

		pVertex = (RVLQLIST_PTR_ENTRY *)(pVertex->pNext);
	}
}

void CRVLDelaunay::Display(	CRVLFigure *pFig,
							int ImageWidth,
							RVLCOLOR Color,
							DWORD mMask1, 
							DWORD mMask2)
{
	CRVLDisplayVector Vector(pFig->m_pMem);

	Vector.m_bClosed = FALSE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector.m_rL = Color.r;
	Vector.m_gL = Color.g;
	Vector.m_bL = Color.b;

	CRVLDisplayVector *pVector;

	RVLQLIST *pVertexList = &m_VertexList;

	void **ppVertex = &(pVertexList->pFirst);

	RVLQLIST_PTR_ENTRY *pVertex = (RVLQLIST_PTR_ENTRY *)(pVertexList->pFirst);

	RVLMESH_LINK *pLink0, *pLink, *pLinkOpposite;

	while(pVertex)
	{
		pLink0 = pLink = (RVLMESH_LINK *)(pVertex->Ptr);

		do
		{
			pLinkOpposite = pLink->pOpposite;

			if(pLink->iPix0 < pLinkOpposite->iPix0 && (pLink->Flags & mMask1) == mMask2)
			{
				pVector = pFig->AddVector(&Vector);
				
				pVector->Line(((pLink->iPix0 % ImageWidth) << 1),
					((pLink->iPix0 / ImageWidth) << 1),
					((pLinkOpposite->iPix0 % ImageWidth) << 1),
					((pLinkOpposite->iPix0 / ImageWidth) << 1));				
			}

			pLink = pLink->pNext;
		}
		while(pLink != pLink0);

		pVertex = (RVLQLIST_PTR_ENTRY *)(pVertex->pNext);
	}
}


void CRVLDelaunay::Display2(CRVLFigure *pFig,
							int ImageWidth,
							RVLCOLOR Color,
							DWORD mMask1, 
							DWORD mMask2)
{
	CRVLDisplayVector Vector(pFig->m_pMem);

	Vector.m_bClosed = FALSE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector.m_rL = Color.r;
	Vector.m_gL = Color.g;
	Vector.m_bL = Color.b;

	CRVLDisplayVector *pVector;

	BYTE *pDelaunayData0 = m_DelaunayData;

	RVLMESH_LINK *DelaunayLink0, *pDelaunayLinkArrayEnd, *pLink, *pLinkOpposite;

	int nDelaunayLinks;

	while(TRUE)
	{
		nDelaunayLinks = *((short *)pDelaunayData0);

		if(nDelaunayLinks < 1)
			break;

		DelaunayLink0 = (RVLMESH_LINK *)(pDelaunayData0 + sizeof(short));

		pDelaunayLinkArrayEnd = DelaunayLink0 + nDelaunayLinks;

		for(pLink = DelaunayLink0; pLink < pDelaunayLinkArrayEnd; pLink++)
		{
			pLinkOpposite = pLink->pOpposite;

			if(pLink->iPix0 < pLinkOpposite->iPix0 && (pLink->Flags & mMask1) == mMask2)
			{
				pVector = pFig->AddVector(&Vector);
				
				pVector->Line(((pLink->iPix0 % ImageWidth) << 1),
					((pLink->iPix0 / ImageWidth) << 1),
					((pLinkOpposite->iPix0 % ImageWidth) << 1),
					((pLinkOpposite->iPix0 / ImageWidth) << 1));				
			}			
		}

		pDelaunayData0 = (BYTE *)pLink;
	}
}

///////////////////////////////////// 

//
//     Global Functions
//
///////////////////////////////////// 


BOOL RVLPolygonConvexityTest(RVLMESH_LINK *pLink0)
{
	RVLMESH_LINK *pLink = pLink0;

	RVLMESH_LINK *pNextLink;

	do
	{
		pNextLink = pLink->pNext;

		if(-pLink->dv * pNextLink->du + pLink->du * pNextLink->dv < 0)
			return FALSE;

		pLink = pNextLink->pOpposite;
	}
	while(pLink != pLink0);

	return TRUE;
}


void RVLSetFlags(RVLMESH_LINK *pDelaunayLink0,
				 BYTE Flags)
{
	RVLMESH_LINK *pDelaunayLink = pDelaunayLink0;

	do
	{
		pDelaunayLink->Flags |= Flags;

		pDelaunayLink = pDelaunayLink->pNext->pOpposite;
	}	
	while(pDelaunayLink != pDelaunayLink0);
	
}

void RVLResetFlags(	RVLMESH_LINK *pDelaunayLink0,
					BYTE Flags)
{
	RVLMESH_LINK *pDelaunayLink = pDelaunayLink0;

	do
	{
		pDelaunayLink->Flags &= ~Flags;

		pDelaunayLink = pDelaunayLink->pNext->pOpposite;
	}	
	while(pDelaunayLink != pDelaunayLink0);
	
}

void RVLDisplay2DRegion(	CRVLFigure *pFig,
							RVLMESH_LINK *pDelaunayLink0,
							int ImageWidth,
							RVLCOLOR Color,
							int LineWidth,
							DWORD mMask1, DWORD mMask2, 
							BOOL bBoundary,
							BYTE BoundaryFlags,
							bool bAvoidDoubleEdges)
{

	CRVLDisplayVector Vector(pFig->m_pMem);

	Vector.m_bClosed = FALSE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector.m_rL = Color.r;
	Vector.m_gL = Color.g;
	Vector.m_bL = Color.b;
	Vector.m_LineWidth = LineWidth;

	CRVLDisplayVector *pVector;

	RVLMESH_LINK *pDelaunayLinkIn = pDelaunayLink0;

	RVLMESH_LINK *pDelaunayLinkOut;

	CRVL2DRegion2 *p2DRegion;

	do
	{
		if(bBoundary)
		{
			pDelaunayLinkOut = pDelaunayLinkIn;

			do
				pDelaunayLinkOut = pDelaunayLinkOut->pNext;
			while((pDelaunayLinkOut->Flags & BoundaryFlags)==0 && pDelaunayLinkOut != pDelaunayLinkIn);

			if(pDelaunayLinkOut == pDelaunayLinkIn)
			{
				pDelaunayLinkIn = pDelaunayLinkOut->pOpposite;

				continue;
			}
		}
		else
			pDelaunayLinkOut = pDelaunayLinkIn->pNext;

		pDelaunayLinkIn = pDelaunayLinkOut->pOpposite;

		p2DRegion = (CRVL2DRegion2 *)(pDelaunayLinkOut->vp2DRegion);

		if(bAvoidDoubleEdges)
			if(p2DRegion)
				if((p2DRegion->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
					if(pDelaunayLinkIn->iPix0 > pDelaunayLinkOut->iPix0)
						continue;

		if((pDelaunayLinkIn->Flags & mMask1) == mMask2)
		{
		//added this/these line(s) for LEVEL3 edge display
		//if(((pDelaunayLinkIn->Flags & RVLMESH_LINK_FLAG_BOUNDARY) !=0)  && ((pDelaunayLinkOut->Flags & RVLMESH_LINK_FLAG_BOUNDARY) !=0) )
		//if(((pDelaunayLinkIn->Flags & RVLMESH_LINK_FLAG_EDGE) !=0)  && ((pDelaunayLinkOut->Flags & RVLMESH_LINK_FLAG_EDGE) !=0) )
		//{
			
			pVector = pFig->AddVector(&Vector);
			
			pVector->Line(((pDelaunayLinkOut->iPix0 % ImageWidth) << 1),
				((pDelaunayLinkOut->iPix0 / ImageWidth) << 1),
				((pDelaunayLinkIn->iPix0 % ImageWidth) << 1),
				((pDelaunayLinkIn->iPix0 / ImageWidth) << 1));
			
		//}
		}
	}	
	while(pDelaunayLinkIn != pDelaunayLink0);
}

BOOL RVLFlipDiagonal(RVLMESH_LINK *pLink,
					 RVLMESH_LINK **LinkArray)
{
	RVLMESH_LINK *pLink0 = pLink->pNext->pOpposite;
	RVLMESH_LINK *pLink1 = pLink0->pNext;
	RVLMESH_LINK *pLink3 = pLink->pPrev->pOpposite;
	RVLMESH_LINK *pLink2 = pLink3->pPrev;

	if(LinkArray)
	{
		double c01 = (double)(pLink0->du * pLink1->du + pLink0->dv * pLink1->dv);
		double s01 = (double)(-pLink0->dv * pLink1->du + pLink0->du * pLink1->dv);

		double c23 = (double)(pLink2->du * pLink3->du + pLink2->dv * pLink3->dv);
		double s23 = (double)(-pLink2->dv * pLink3->du + pLink2->du * pLink3->dv);

		if(s01 * c23 + c01 * s23 >= 0.0)
			return FALSE;
	}

	RVLMESH_LINK *pLinkOpposite = pLink->pOpposite;

	pLink0->pNext = pLink;

	pLink->pPrev = pLink0;
	
	pLink1->pPrev = pLink;

	pLink->pNext = pLink1;

	pLink2->pNext = pLinkOpposite;

	pLinkOpposite->pPrev = pLink2;

	pLink3->pPrev = pLinkOpposite;

	pLinkOpposite->pNext = pLink3;

	RVLMESH_LINK *pLink4 = pLink0->pOpposite;
	RVLMESH_LINK *pLink5 = pLink1->pOpposite;
	RVLMESH_LINK *pLink6 = pLink2->pOpposite;
	RVLMESH_LINK *pLink7 = pLink3->pOpposite;

	pLink7->pNext = pLink4;

	pLink4->pPrev = pLink7;

	pLink6->pPrev = pLink5;

	pLink5->pNext = pLink6;

	CRVL2DRegion2 *pTriangle = (CRVL2DRegion2 *)(pLink->vp2DRegion);

	CRVL2DRegion2 *pTriangle2 = (CRVL2DRegion2 *)(pLinkOpposite->vp2DRegion);

	pLink0->vp2DRegion = pLinkOpposite->vp2DRegion = pLink7->vp2DRegion = pTriangle;

	pTriangle->m_PtArray = pLink0;

	pLink->vp2DRegion = pLink5->vp2DRegion = pLink2->vp2DRegion = pTriangle2;

	pTriangle2->m_PtArray = pLink;

	pLink->iPix0 = pLink0->iPix0;
	int du = pLink0->du + pLink7->du;
	int dv = pLink0->dv + pLink7->dv;
	pLink->du = du;
	pLink->dv = dv;
	pLink->len = DOUBLE2INT(sqrt((double)(du * du + dv * dv)));

	pLinkOpposite->iPix0 = pLink2->iPix0;
	pLinkOpposite->du = -du;
	pLinkOpposite->dv = -dv;
	pLinkOpposite->len = pLink->len;

	if(LinkArray)
	{
		LinkArray[0] = pLink0;
		LinkArray[1] = pLink5;
		LinkArray[2] = pLink2;
		LinkArray[3] = pLink7;
		LinkArray[4] = pLink4;
		LinkArray[5] = pLink1;
		LinkArray[6] = pLink6;
		LinkArray[7] = pLink3;
	}
	
	return TRUE;
}