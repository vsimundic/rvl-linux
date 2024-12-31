// RVLStereoVision.cpp: implementation of the CRVLStereoVision class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "Platform.h"

#ifdef SVS
#include "svs.h"
#include "svsclass.h"
#endif

//#include <highgui.h>
#include "RVLCore.h"

#ifdef RVLSTEREO_CORRESPONDENCE_SAD_ARRAY
CV_IMPL void cvFindStereoCorrespondenceBM_RVL( const CvArr* leftarr, const CvArr* rightarr,
                              CvArr* disparr, CvStereoBMState* state, unsigned short *SADArray );
#endif

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLStereoVision::CRVLStereoVision()
{
	m_Flags = RVLSTEREO_FLAGS_METHOD_KINECT;
	m_TolDisparity = 16;		// pix
	m_varIP = 0.333 * 0.333;	// pix^2
	m_TriangulationLookupTable = NULL;
	m_u1Shift = 0;
	m_EdgeMagnitudeThr = 255 - 4;
	m_minDisparity = 0;			// pix
	m_UncertCoeff = 1.0;
	m_WDisparity = m_UncertCoeff * 16.0;
	m_maxz = 5000.0;			// mm
	m_3DMap = NULL;
	m_nDisp = 128;				// pix
	m_nSubPixDisps = 8;
	m_WinSize = 11;				// pix
	m_ConfThr = 16;
	m_UniquenessThr = 4;
	m_vpVideoObject = NULL;	
	m_vpEdgeDetectR = NULL;
	m_SADArray = NULL;
	m_KinectParams.depthFu0 = 584.70194597700402;
	m_KinectParams.depthUc0 = 318.55964537649561;
	m_KinectParams.depthFv0 = 585.70332900816618;
	m_KinectParams.depthVc0 = 256.14501544470505;
	
#ifdef SVS
	m_vpImageAcquisition = (void *)(new svsStoredImages);
	m_vpStereoImage = (void *)(new svsStereoImage);
#else
	m_vpImageAcquisition = NULL;
	m_vpStereoImage = NULL;
#endif
	m_DisparityOffset = 0;
	m_DisparityOffsetNrm = (m_DisparityOffset << 4);

	m_DisparityFileName = NULL;
	m_ParamFileName = NULL;

	m_DisparityMap.Width = m_DisparityMap.Height = 0;
	m_DisparityMap.Disparity = NULL;

	m_KinectParams.pZProjLT = NULL;
	m_KinectParams.pPose = NULL;
	m_KinectScale = 2;
}

CRVLStereoVision::~CRVLStereoVision()
{
	if(m_TriangulationLookupTable != NULL)
		delete[] m_TriangulationLookupTable;

	if(m_3DMap != NULL)
		delete[] m_3DMap;

#ifdef SVS
	delete (svsStoredImages *)m_vpImageAcquisition;

	if(m_vpVideoObject)
		CameraStop();
	else
		delete (svsStereoImage *)m_vpStereoImage;
#else
	if(m_DisparityMap.Disparity)
		delete[] m_DisparityMap.Disparity;
#endif

	if(m_ParamFileName)
		delete[] m_ParamFileName;

	if(m_SADArray)
		delete[] m_SADArray;

	if(m_KinectParams.pZProjLT)
		delete[] m_KinectParams.pZProjLT;

	if(m_KinectParams.pPose)
		delete m_KinectParams.pPose;

}


/*
void CRVLStereoVision::Get3DLines(CRVLMPtrChain *p2DLineArray, 
								  CRVLC3D *pC3DLine)
{	
	int Size = (m_pCameraL->Width > m_pCameraL->Height ? m_pCameraL->Width :
		m_pCameraL->Height);

#ifdef STEREOVISION_RECORD
	double *P3D = new double[3 * m_pCameraL.Width * m_pCameraL.Height];

	//m_pP3D = P3D;
#endif

	// get close-to-vertical 3D edges

	RVLSTEREOPOINT *StereoPointBuffer = new RVLSTEREOPOINT[Size];

	RVLSTEREOPOINT *StereoPoint = new RVLSTEREOPOINT[Size];

	int n2DLines = p2DLineArray->m_nElements;

	//CRVL2DLine2 **Hor2DLine = new CRVL2DLine2 *[n2DLines];
	//CRVL2DLine2 **ppHor2DLine = Hor2DLine;
		
	CRVL2DLine2 *p2DLine;

	p2DLineArray->Start();

	while(p2DLineArray->m_pNext)
	{
		p2DLine = (CRVL2DLine2 *)(p2DLineArray->GetNext());

		//if(p2DLine->m_Flags & RVL2DLINE_FLAG_FRAME)
		//	continue;

		if(abs(p2DLine->m_diU[1]) >= abs(p2DLine->m_diU[0]))
			Get3DLine(p2DLine, StereoPoint, StereoPointBuffer, pC3DLine);

		//else
		//	*(ppHor2DLine++) = p2DLine;
	}

	// get close-to-horizontal 3D edges

	//CRVL2DLine **ppLastHor2DLine = ppHor2DLine;

	//for(ppHor2DLine = Hor2DLine; ppHor2DLine < ppLastHor2DLine; ppHor2DLine++)
	//{
	//	p2DLine = *ppHor2DLine;
			
	//	p3DEdge = GetHor3DEdge(p2DLine, vp2DLineArray, RotCW);

	//	if(p3DEdge == NULL)
	//		p3DEdge = Get3DEdge(p2DLine, vp2DLineArray, RotCW, StereoPoint, StereoPointBuffer);

	//	if(p3DEdge != NULL)
	//		Add3DEdge(p3DEdge, p2DLine);
	//}

	//delete[] Hor2DLine;

	delete[] StereoPointBuffer;

	delete[] StereoPoint;

	// transform to world coordinates

	//RVL3DOBJ_POSE Transf;

	//memcpy(Transf.Rot, RotCW, 9 * sizeof(double));

	//LinearTransform3D(RotCW, m_CameraParams[0].P3DC0Nrm, Transf.Position);

	//for(i3DEdge = 0; i3DEdge < m_3DEdgeArray.GetSize(); i3DEdge++)
	//{
	//	p3DEdge = m_3DEdgeArray.GetAt(i3DEdge);

	//	p3DEdge->m_ParamFlags = (RVL3DEDGE_PARAM_FLAG_P2D | RVL3DEDGE_PARAM_FLAG_P3D | RVL3DEDGE_PARAM_FLAG_C3D);

	//	p3DEdge->Transform(&Transf);

	//	p3DEdge->m_Wq = 0;

	//	memset(p3DEdge->W3DLine, 0, 3 * sizeof(double));
	//}

	// record 3D edge data

//#ifdef STEREOVISION_RECORD
	//FILE *fp = fopen("D:\\Cupec\\Computer_Vision\\RVL\\ExpRez\\stereo3D.dat", "wb");

	//fwrite(P3D, (int)(m_pP3D - P3D), sizeof(double), fp);

	//fclose(fp);

	//delete[] P3D;

	//fp = fopen("D:\\Cupec\\Computer_Vision\\RVL\\ExpRez\\edge3D.dat", "w");

	//int i;

	//for(i = 0; i < m_3DEdgeArray.GetSize(); i++)
	//{
	//	p3DEdge = m_3DEdgeArray.GetAt(i);

	//	if(p3DEdge->P3D1[2] > 5000.0 && p3DEdge->P3D2[2] > 5000.0)
	//		continue;

	//	p3DEdge->Save(fp);
	//}

	//fclose(fp);
//#endif	

}
*/

void CRVLStereoVision::SetDisparityMap(RVLDISPARITYMAP *pDisparityMap)
{
	m_DisparityMap = *pDisparityMap;

	m_DisparityOffsetNrm = (m_DisparityMap.DisparityOffset << 4);
}

/*
BOOL CRVLStereoVision::Get3DLine(CRVL2DLine2 *p2DLine, 
								 RVLSTEREOPOINT *StereoPoint, 
								 RVLSTEREOPOINT *StereoPointBuffer,
								 CRVLC3D *pC3DLine)
{
	//CArray<CRVL2DLine *, CRVL2DLine *> *p2DLineArray = 
	//	(CArray<CRVL2DLine *, CRVL2DLine *> *)vp2DLineArray;
	
	// get number of points supporting the line

	//int nPoints = 0;

	//WORD jSeg;
	//CRVL *pSeg;

	//for(jSeg = 0; jSeg < p2DLine->m_pLine->pContourSegmentBuffer->GetSize(); jSeg++)
	//{
	//	pSeg = p2DLine->m_pLine->pContourSegmentBuffer->GetAt(jSeg);		

		//nPoints += (pSeg->iP2 - pSeg->iP1 + 1);
	//}

	//if(p2DLine->m_Index == 58)
	//	int debug = 1;

	// get stereo point set

	int absdiU[2];
	absdiU[0] = abs(p2DLine->m_diU[0]);
	absdiU[1] = abs(p2DLine->m_diU[1]);

	int iDomCoord = (absdiU[1] > absdiU[0] ? 1 : 0);
		
	int nBuffer, BufferOffset, BufferSize;

	nBuffer = absdiU[iDomCoord] + 1;
	BufferOffset = (p2DLine->m_diU[iDomCoord] > 0 ? p2DLine->m_iU[0][iDomCoord] : 
		p2DLine->m_iU[1][iDomCoord]);
	BufferSize = (iDomCoord ? m_pCameraL->Height : m_pCameraL->Width);

	memset(StereoPointBuffer, 0xff, BufferSize * sizeof(RVLSTEREOPOINT));

	RVLSTEREOPOINT *pStereoPoint1 = NULL;
	RVLSTEREOPOINT *pStereoPoint2 = NULL;

	WORD kPoint1, kPoint2;
	kPoint1 = kPoint2 = RVL_NONE;

	RVLSTEREOPOINT *pStereoPoint;
	WORD kPoint;
	int iPix, s;
	short disparity;

	CRVLMPtrChain2 RelList;

	RelList.Create(p2DLine->m_RelList + RVLRELLIST_INDEX_2DLINE_CONTOUR_SEGMENTS,
		p2DLine->m_pClass->m_pMem);

	CRVLContourSegmentRelation *pContourSegmentRel;
	RVLIPOINT *pIP, *pEndIP;
	int *iU;

	RelList.Start();

	while(RelList.m_pNext)
	{
		pContourSegmentRel = (CRVLContourSegmentRelation *)(RelList.GetNext());

		pEndIP = pContourSegmentRel->pIP + pContourSegmentRel->nIP;

		for(pIP = pContourSegmentRel->pIP; pIP < pEndIP; pIP++)
		{
			iU = (int *)pIP;

			iPix = (iU[1] - m_DisparityMap.v0) * m_DisparityMap.Width + 
				(iU[0] - m_DisparityMap.u0);

			disparity = m_DisparityMap.Disparity[iPix];

			if(disparity < 0)
				continue;

			s = iU[iDomCoord];

			pStereoPoint = StereoPointBuffer + s;

			pStereoPoint->s = s;
			pStereoPoint->u = iU[0];
			pStereoPoint->v = iU[1];
			pStereoPoint->disparity = disparity;

			if(kPoint1 == RVL_NONE)
				kPoint1 = kPoint2 = s;
			else
			{
				if(pStereoPoint->s < kPoint1)
					kPoint1 = pStereoPoint->s;
				else if(pStereoPoint->s > kPoint2)
					kPoint2 = pStereoPoint->s;
			}
		}	// for(pIP = pFirstIP; pIP < pEndIP; pIP++)
	}	// while(RelList.m_pNext)

	if(kPoint1 == RVL_NONE)
		return FALSE;

	int nPoints = kPoint2 - kPoint1 + 1;
	
	//WORD kPoint8;
	int nPoints3;
	int ddisparity;
	int PredictedDisparityNrm;
	int nMergedPoints;
	int DisparityErr;

	BYTE mPointMatch = 0x00;

	if(iDomCoord == 0) // edge is close-to-horizontal
	{
		return FALSE;
#ifdef RVL_NEVER	
		pStereoPoint = StereoPointBuffer + kPoint1;

		int du, dv;

		for(kPoint = 0; kPoint < 2; kPoint++, pStereoPoint = StereoPointBuffer + kPoint2)
		{
			if(pStereoPoint->disparity < 0)
				return NULL;

			P = p2DLine->m_pLine->P1;

			for(kPoint8 = 0; kPoint8 < 2; kPoint8++, P = p2DLine->m_pLine->P2)
			{
				du = pStereoPoint->u - P.x;
				dv = pStereoPoint->v - P.y;

				if(du * du + dv * dv > 4)
					continue;

				mPointMatch |= (1 << kPoint8);
			}
		}

		if(mPointMatch != 0x03)
			return NULL;

		// check the reliability of the endpoints

		WORD jConnection;
		EDGE_VECTOR_CONNECTION *pConnection;
		CArray<EDGE_VECTOR_CONNECTION *, EDGE_VECTOR_CONNECTION *> *pConnectionArray;
		CRVL2DLine *pConnected2DLine;
		BOOL bConnection;

		pConnectionArray = p2DLine->m_pLine->pConnectionArray1;

		if(pConnectionArray == NULL)
			return NULL;

		CPoint P = p2DLine->m_pLine->P1;
		
		int absdu, absdv;
		int LinkLen21, LinkLen22, LinkLen2;
		CRVL3DEdge *pConnected3DEdge;

		for(kPoint = 0; kPoint < 2; kPoint++, P = p2DLine->m_pLine->P2)
		{
			bConnection = FALSE;

			for(jConnection = 0; jConnection < pConnectionArray->GetSize(); jConnection++)
			{
				pConnection = pConnectionArray->GetAt(jConnection);

				if(pConnection->LinkLen > m_maxLinkLen)
					continue;

				if(pConnection->Type != EDGE_VECTOR_CONNECTION_COTERMINATION)
					continue;

				pConnected2DLine = p2DLineArray->GetAt(pConnection->iVector & EDGE_VECTOR_INDEX);

				if((pConnected2DLine->m_Flags & RVL2DLINE_FLAG_3D) == 0)
					continue;

				absdv = abs(pConnected2DLine->m_pLine->dv);

				if(absdv < m_minConnectLenL)
					continue;

				absdu = abs(pConnected2DLine->m_pLine->du);

				if(absdu > absdv)
					continue;

				pConnected3DEdge = (CRVL3DEdge *)(pConnected2DLine->m_cObjectArray.GetAt(0)->vpObject);

				du = P.x - pConnected3DEdge->P1.x;
				dv = P.y - pConnected3DEdge->P1.y;

				LinkLen21 = du * du + dv * dv;

				du = P.x - pConnected3DEdge->P2.x;
				dv = P.y - pConnected3DEdge->P2.y;

				LinkLen22 = du * du + dv * dv;

				LinkLen2 = (LinkLen21 < LinkLen22 ? LinkLen21 : LinkLen22);

				if(LinkLen2 > m_maxLinkLen * m_maxLinkLen)
					continue;

				bConnection = TRUE;

				break;

				
				//if(pBestConnection[kPoint] != NULL)
				//{
				//	if(pConnection->LinkLen < pBestConnection[kPoint]->LinkLen)
				//		pBestConnection[kPoint] = pConnection;
				//}
				//else
				//	pBestConnection[kPoint] = pConnection;
				
			}

			if(!bConnection)
				return NULL;

			pConnectionArray = p2DLine->m_pLine->pConnectionArray2;
		}

		// get points consistent wiht the endpoints

		nPoints3 = kPoint2 - kPoint1;

		if(nPoints3 <= 0)
			return NULL;

		ddisparity = (int)(StereoPointBuffer[kPoint2].disparity) - 
			(int)(StereoPointBuffer[kPoint1].disparity);

		PredictedDisparityNrm = StereoPointBuffer[kPoint1].disparity * nPoints3;

		pStereoPoint = StereoPointBuffer + kPoint1;

		pStereoPoint2 = StereoPoint;

		nMergedPoints = 0;

		for(kPoint = kPoint1; kPoint <= kPoint2; kPoint++, pStereoPoint++, 
			PredictedDisparityNrm += ddisparity)
			if(pStereoPoint->disparity >= 0)
			{
				DisparityErr = abs(PredictedDisparityNrm / nPoints3 - pStereoPoint->disparity);
			
				if(DisparityErr <= m_TolDisparity)
				{
					*(pStereoPoint2++) = *pStereoPoint;

					nMergedPoints++;

					//if(kPoint5 == RVL_NONE)
					//	kPoint5 = kPoint6 = kPoint;
					//else
					//{
					//	if(kPoint < kPoint5)
					//		kPoint5 = kPoint;
					//	else if(kPoint > kPoint6)
					//		kPoint6 = kPoint;
					//}
				}
			}

		//if(bConfirmed = (nMergedPoints > minnPoints))
		//	break;
#endif // RVL_NEVER
	}
	else	// edge is close-to-vertical
	{
		//if(pStereoPoint1 == NULL)
		//	return;

		// split stereo point set to linear intervals

		CRVLWChain IntervalArray;

		IntervalArray.Add(kPoint1);
		IntervalArray.Add(kPoint2);

		IntervalArray.Start();

		WORD kMaxErrPoint;
		RVLWCHAIN_ELEMENT *pPrevious;
		int maxDisparityErr;

		while(IntervalArray.m_pNext)
		{
			kPoint1 = IntervalArray.GetNext();

			pPrevious = IntervalArray.m_pCurrent;

			kPoint2 = IntervalArray.GetNext();

			nPoints3 = kPoint2 - kPoint1;

			if(nPoints3 == 0)
				continue;

			ddisparity = (int)(StereoPointBuffer[kPoint2].disparity) - 
				(int)(StereoPointBuffer[kPoint1].disparity);

			PredictedDisparityNrm = StereoPointBuffer[kPoint1].disparity * nPoints3 + ddisparity;

			pStereoPoint = StereoPointBuffer + kPoint1 + 1;

			maxDisparityErr = 0;

			for(kPoint = kPoint1 + 1; kPoint < kPoint2; kPoint++, pStereoPoint++, 
				PredictedDisparityNrm += ddisparity)
				if(pStereoPoint->disparity >= 0)
				{
					DisparityErr = abs(PredictedDisparityNrm / nPoints3 - pStereoPoint->disparity);
				
					if(DisparityErr > maxDisparityErr)
					{
						maxDisparityErr = DisparityErr;
						kMaxErrPoint = kPoint;
					}
				}

			if(maxDisparityErr > m_TolDisparity)
			{
				IntervalArray.InsertAt(kMaxErrPoint, pPrevious);
				IntervalArray.InsertAt(kMaxErrPoint - 1, pPrevious);
				IntervalArray.m_pNext = pPrevious;
			}
		}

		// merge intervals of the stereo point set to linear subsets

		WORD *kPoint3 = new WORD[IntervalArray.m_nElements];
		WORD *pkPoint = kPoint3;

		int nIntervals = IntervalArray.m_nElements / 2;
		
		IntervalArray.Start();

		while(IntervalArray.m_pNext)
			*(pkPoint++) = IntervalArray.GetNext();

		IntervalArray.RemoveAll();

		BOOL *bMerged = new BOOL[nIntervals];

		memset(bMerged, 0, nIntervals * sizeof(BOOL));

		BOOL bConfirmed = FALSE;

		int minnPoints = nPoints / 2 + 1;

		//int kPoint4, nPoints4;
		WORD kPoint5, kPoint6;
		kPoint5 = kPoint6 = RVL_NONE;

		int kInterval, kInterval2;

		for(kInterval = 0; kInterval < nIntervals && !bConfirmed; kInterval++)
		{	
			if(bMerged[kInterval])
				continue;

			kPoint1 = kPoint3[kInterval << 1];

			//kPoint4 = kPoint3[(kInterval << 1) + 1];

			//nPoints4 = 0;

			//pStereoPoint = StereoPointBuffer + kPoint1;

			//for(kPoint = kPoint1; kPoint <= kPoint4; kPoint++, pStereoPoint++)
			//	if(pStereoPoint->disparity >= 0)
			//		nPoints4++;
				
			//if(nPoints4 == 0)
			//	continue;

			for(kInterval2 = nIntervals - 1; kInterval2 >= kInterval; kInterval2--)
			{
				kPoint2 = kPoint3[(kInterval2 << 1) + 1];

				nPoints3 = kPoint2 - kPoint1;

				if(nPoints3 <= 0)
					continue;

				ddisparity = (int)(StereoPointBuffer[kPoint2].disparity) - 
					(int)(StereoPointBuffer[kPoint1].disparity);

				PredictedDisparityNrm = StereoPointBuffer[kPoint1].disparity * nPoints3;

				pStereoPoint = StereoPointBuffer + kPoint1;

				pStereoPoint2 = StereoPoint;

				nMergedPoints = 0;

				for(kPoint = kPoint1; kPoint <= kPoint2; kPoint++, pStereoPoint++, 
					PredictedDisparityNrm += ddisparity)
					if(pStereoPoint->disparity >= 0)
					{
						DisparityErr = abs(PredictedDisparityNrm / nPoints3 - pStereoPoint->disparity);
					
						if(DisparityErr <= m_TolDisparity)
						{
							*(pStereoPoint2++) = *pStereoPoint;

							nMergedPoints++;

							if(kPoint5 == RVL_NONE)
								kPoint5 = kPoint6 = kPoint;
							else
							{
								if(kPoint < kPoint5)
									kPoint5 = kPoint;
								else if(kPoint > kPoint6)
									kPoint6 = kPoint;
							}
						}
					}

				if(bConfirmed = (nMergedPoints > minnPoints))
					break;
			}
		}

		delete[] kPoint3;

		delete[] bMerged;

		if(!bConfirmed)
			return FALSE;
	}

	// check the reliability of the edge

	//ddisparity = (int)(pStereoPoint2->disparity) - (int)(pStereoPoint1->disparity);

	//int NStereoPoints = 0;	

	//pStereoPoint = StereoPointBuffer + pStereoPoint1->s + 1;

	//nPoints3 = abs(pStereoPoint2->s - pStereoPoint1->s);

	//PredictedDisparityNrm = pStereoPoint1->disparity * nPoints3 + ddisparity;

	//for(kPoint = 1; kPoint < nPoints3; kPoint++, pStereoPoint++, PredictedDisparityNrm += ddisparity)
	//	if(pStereoPoint->disparity >= 0)
	//		if(abs(PredictedDisparityNrm / nPoints3 - pStereoPoint->disparity) < m_TolDisparity)
	//			NStereoPoints++;

	//if(NStereoPoints < nPoints / 2)
	//	return;

	// perform triangulation and fit 3D line to reconstructed 3D points

	double S[3], S2[9];

	S[0] = S[1] = S[2] = 0.0;
	
	memset(S2, 0, 9 * sizeof(double));

	//if(p2DLine->m_Index == 45)
	//	int tmp1 = 0;

	//FILE *fp;

	//if(p2DLine->m_Index == 6)
	//	fp = fopen("D:\\Cupec\\Computer_Vision\\RVL\\ExpRez\\edge3D.dat", "wb");

#ifdef STEREOVISION_RECORD
	double P3DTmp[3];
#endif

	double *P3DBuff = new double[3 * nMergedPoints];

	double *pP3D = P3DBuff;

	int u2_16;
	double den;
	RVLTRIANGULATION_PARAMS *pTriangulationParams;

	for(pStereoPoint1 = StereoPoint; pStereoPoint1 < pStereoPoint2; pStereoPoint1++, pP3D += 3)
	{
		Triangulation4(pStereoPoint1->u, pStereoPoint1->v, pStereoPoint1->disparity, 
			pP3D, u2_16, den, &pTriangulationParams);

#ifdef STEREOVISION_RECORD
		memcpy(P3DTmp, pP3D, 3 * sizeof(double));

		Transform2W(P3DTmp, RotCW, m_CameraParams[0].P3DC0Nrm, m_pP3D);

		if(m_pP3D[2] > 0.0 && m_pP3D[2] < 5000.0)
			m_pP3D += 3;
#endif

		S[0] += pP3D[0];
		S[1] += pP3D[1];
		S[2] += pP3D[2];
		S2[0] += pP3D[0] * pP3D[0];
		S2[1] += pP3D[0] * pP3D[1];
		S2[2] += pP3D[0] * pP3D[2];
		S2[3] += pP3D[1] * pP3D[0];
		S2[4] += pP3D[1] * pP3D[1];
		S2[5] += pP3D[1] * pP3D[2];
		S2[6] += pP3D[2] * pP3D[0];
		S2[7] += pP3D[2] * pP3D[1];
		S2[8] += pP3D[2] * pP3D[2];
	}

	double V[9];

	V[0] = p2DLine->m_cs;
	V[1] = -p2DLine->m_sn;
	V[2] = 0.0;

	double M[3], Ray[3];

	double u0 = 0.5 * (double)(p2DLine->m_iU[0][0] + p2DLine->m_iU[1][0]);
	double v0 = 0.5 * (double)(p2DLine->m_iU[0][1] + p2DLine->m_iU[1][1]);
	double d = p2DLine->m_sn * u0 + p2DLine->m_cs * v0 - p2DLine->m_d;

	M[0] = u0 - p2DLine->m_sn * d;
	M[1] = v0 - p2DLine->m_cs * d;
	M[2] = 1.0;

	LinearTransform3D(m_pCameraL->InvPRNrm, M, Ray);

	CrossProduct(Ray, V, V + 6);

	double fTmp1 = sqrt(V[6] * V[6] + V[7] * V[7] + V[8] * V[8]);

	V[6] /= fTmp1;
	V[7] /= fTmp1;
	V[8] /= fTmp1;	

	CrossProduct(V, V + 6, V + 3);

	//if(p2DLine->m_Index == 6)
	//{
	//	fp = fopen("D:\\Cupec\\Computer_Vision\\RVL\\ExpRez\\V.dat", "wb");

	//	fwrite(V, 9, sizeof(double), fp);

	//	fclose(fp);
	//}

	double W[6];

	MatrixMultiplication(V, S2, W, 2, 3, 3);

	RVL2DMOMENTS StatData;

	StatData.n = (int)(pStereoPoint2 - StereoPoint);
	
	StatData.S[0] = V[0] * S[0] + V[1] * S[1] + V[2] * S[2];
	StatData.S[1] = V[3] * S[0] + V[4] * S[1] + V[5] * S[2];

	MatrixMultiplicationT(W, V, StatData.S2, 2, 3, 2);

	double err;
	double cs, sn, d2;

	RVLFit2DLine(&StatData, cs, sn, d2, err);

	double cscs = cs * cs;
	double cssn = cs * sn;
	double snsn = sn * sn;
	double csd = cs * d2;
	double snd = sn * d2;

	pP3D = P3DBuff;

	double p, q, p0, q0, pmin, pmax, qmin, qmax;

	for(kPoint = 0; kPoint < nMergedPoints; kPoint++, pP3D += 3)
	{
		p = RVLDotProduct(V, pP3D);
		q = RVLDotProduct(V + 3, pP3D);

		p0 = p - (snsn * p + cssn * q - snd);
		q0 = q - (cssn * p + cscs * q - csd);

		if(kPoint)
		{
			if(p0 < pmin)
			{
				pmin = p0;
				qmin = q0;
			}
			else if(p0 > pmax)
			{
				pmax = p0;
				qmax = q0;
			}
		}
		else
		{
			pmin = pmax = p0;
			qmin = qmax = q0;
		}
	}

	double P3DC1[3], P3DC2[3];

	P3DC1[0] = pmin * V[0] + qmin * V[3];
	P3DC1[1] = pmin * V[1] + qmin * V[4];
	P3DC1[2] = pmin * V[2] + qmin * V[5];

	P3DC2[0] = pmax * V[0] + qmax * V[3];
	P3DC2[1] = pmax * V[1] + qmax * V[4];
	P3DC2[2] = pmax * V[2] + qmax * V[5];

	int u1, v1, u2, v2;

	Projection(P3DC1, m_pCameraL->PRNrm, u1, v1);
	Projection(P3DC2, m_pCameraL->PRNrm, u2, v2);

	int du = u2 - u1;
	int dv = v2 - v1;

	double *pP3DC1, *pP3DC2;

	RVLIPOINT P1, P2;

	if(abs(du) >= abs(dv))
	{
		if(p2DLine->m_diU[0] * du > 0)
		{
			pP3DC1 = P3DC1;
			pP3DC2 = P3DC2;
			P1.u = u1;
			P1.v = v1;
			P2.u = u2;
			P2.v = v2;
		}
		else
		{
			pP3DC1 = P3DC2;
			pP3DC2 = P3DC1;
			P1.u = u2;
			P1.v = v2;
			P2.u = u1;
			P2.v = v1;
		}
	}
	else
	{
		if(p2DLine->m_diU[1] * dv > 0)
		{
			pP3DC1 = P3DC1;
			pP3DC2 = P3DC2;
			P1.u = u1;
			P1.v = v1;
			P2.u = u2;
			P2.v = v2;
		}
		else
		{
			pP3DC1 = P3DC2;
			pP3DC2 = P3DC1;
			P1.u = u2;
			P1.v = v2;
			P2.u = u1;
			P2.v = v1;
		}
	}

	//double k1, k2;

	//M[0] = (double)(p2DLine->m_pLine->P1.x);
	//M[1] = (double)(p2DLine->m_pLine->P1.y);
	//M[2] = 1.0;

	//LinearTransform3D(m_CameraParams[0].InvPRNrm, M, p2DLine->m_Ray1);

	//k1 = V[0] * p2DLine->m_Ray1[0] + V[1] * p2DLine->m_Ray1[1] + V[2] * p2DLine->m_Ray1[2];
	//k2 = V[3] * p2DLine->m_Ray1[0] + V[4] * p2DLine->m_Ray1[1] + V[5] * p2DLine->m_Ray1[2];

	//TwoEqsTwoVars(CSD.sn, CSD.cs, -k2, k1, CSD.d, 0.0, p, q);

	//P3DC1[0] = p * V[0] + q * V[3];
	//P3DC1[1] = p * V[1] + q * V[4];
	//P3DC1[2] = p * V[2] + q * V[5];

	//M[0] = (double)(p2DLine->m_pLine->P2.x);
	//M[1] = (double)(p2DLine->m_pLine->P2.y);
	//M[2] = 1.0;

	//LinearTransform3D(m_CameraParams[0].InvPRNrm, M, p2DLine->m_Ray2);

	//k1 = V[0] * p2DLine->m_Ray2[0] + V[1] * p2DLine->m_Ray2[1] + V[2] * p2DLine->m_Ray2[2];
	//k2 = V[3] * p2DLine->m_Ray2[0] + V[4] * p2DLine->m_Ray2[1] + V[5] * p2DLine->m_Ray2[2];

	//TwoEqsTwoVars(CSD.sn, CSD.cs, -k2, k1, CSD.d, 0.0, p, q);

	//P3DC2[0] = p * V[0] + q * V[3];
	//P3DC2[1] = p * V[1] + q * V[4];
	//P3DC2[2] = p * V[2] + q * V[5];

	// create 3D edge

	//p2DLine->m_pLine->bMarked = TRUE;

	CRVL3DLine2 Line3D;

	Line3D.Create(pC3DLine);	

	//pStereoPoint = StereoPointBuffer;

	//double *TL = m_pCameraL->NrmTransf;
	//double *TR = m_pCameraL->NrmTransf;

	//double P3D12[3], P3D22[3];
	//double W3D1[3], W3D2[3];
	//double C[9];

	//pStereoPoint1 = StereoPointBuffer + kPoint1;
	//pStereoPoint2 = StereoPointBuffer + kPoint2;

	// get 3D point 1

	memcpy(Line3D.m_XL[0], pP3DC1, 3 * sizeof(double));

	// get 3D point 2

	memcpy(Line3D.m_XL[1], pP3DC2, 3 * sizeof(double));

	// get triangulation uncertainty

	//RVLDif3D(p3DEdge->P3DC2, p3DEdge->P3DC1, p3DEdge->dP3D);

	//double len2 = RVLDotProduct(p3DEdge->dP3D, p3DEdge->dP3D);
	
	//WORD kPoint7 = kPoint5;

	//double C3D[18], lambda[2];

	//double *pC3D = C3D;

	//double lenRay2, dr, t, s2;
	//double P3D[3];

	//for(kPoint = 0; kPoint < 2; kPoint++, pC3D += 9)
	//{
	//	M[0] = (double)(StereoPointBuffer[kPoint7].u);
	//	M[1] = (double)(StereoPointBuffer[kPoint7].v);
	//	M[2] = 1.0;

	//	LinearTransform3D(m_CameraParams[0].InvPRNrm, M, Ray);

	//	lenRay2 = RVLDotProduct(Ray, Ray);

	//	GetClosestPoints(p3DEdge->P3DC1, p3DEdge->dP3D, len2, Ray, lenRay2, t, s2);

	//	P3D[0] = p3DEdge->P3DC1[0] + t * p3DEdge->dP3D[0];
	//	P3D[1] = p3DEdge->P3DC1[1] + t * p3DEdge->dP3D[1];
	//	P3D[2] = p3DEdge->P3DC1[2] + t * p3DEdge->dP3D[2];

	//	lambda[kPoint] = t;

	//	GetTriangulationUncert(P3D, pC3D);

	//	kPoint7 = kPoint6;
	//}

	//pC3D = p3DEdge->m_C3DC;

	//for(kPoint = 0; kPoint < 2; kPoint++, pC3D += 9)
	//	p3DEdge->GetCov3D(C3D, C3D + 9, ((double)kPoint - lambda[0]) / (lambda[1] - lambda[0]), pC3D);

	GetTriangulationUncert(Line3D.m_XL[0], Line3D.m_CXL[0]);
	GetTriangulationUncert(Line3D.m_XL[1], Line3D.m_CXL[1]);

	Line3D.m_ParamFlags = (RVL3DLINE_PARAM_FLAG_XL | RVL3DLINE_PARAM_FLAG_CXL);

	Line3D.UpdateParams();

	CRVL3DLine2 *p3DLine = (CRVL3DLine2 *)(pC3DLine->m_pMem->Alloc(sizeof(CRVL3DLine2)));
	*p3DLine = Line3D;

	RVLAddObject(p3DLine, pC3DLine);

	CRVL3D2DLineRelation Relation;

	Relation.m_pObject[0] = p3DLine;
	Relation.m_pObject[1] = p2DLine;

	CRVL3D2DLineRelation *pRelation = (CRVL3D2DLineRelation *)
		(pC3DLine->m_pMem->Alloc(sizeof(CRVL3D2DLineRelation)));

	*pRelation = Relation;

	pC3DLine->m_pRelList->Add(pRelation);

	pRelation->ConnectObjects(p3DLine, p2DLine, 
		RVLRELLIST_INDEX_3DLINE_3D2DLINE, RVLRELLIST_INDEX_2DLINE_3D2DLINE,
		&RelList);
	
	delete[] P3DBuff;

	return TRUE;
}
*/


void CRVLStereoVision::GetTriangulationUncert(double *X, double *CX)
{
	double u1, v1, u2, v2;

	Projection(X, m_pCameraL->PRNrm, u1, v1);
	Projection(X, m_pCameraR->PRNrm, u2, v2);

	u1 -= m_pCameraL->CenterXNrm;
	v1 -= m_pCameraL->CenterYNrm;
	u2 -= m_pCameraR->CenterXNrm;

	// u1 / f = x / z;   u2 / f = (x - b) / z  =>
	// =>   z = f * b / (u1 - u2);   x = u1 * z / f;   y = v1 * z / f
	// J = jacobian([x y z]', [u1 v1 u2 v2]')
	// P = J'* J

	double d = u1 - u2;
	double d2 = d * d;
	double k = m_BaseLenNrm / d2;
	k *= k;
	double sumu = u1 + u2;
	v2 = 2.0 * v1;
	double ev = 1.0;

	CX[0] = k * (u1 * u1 + u2 * u2) * m_varIP;
	CX[1] = CX[3] = k * v1 * sumu * m_varIP;
	CX[2] = CX[6] = k * m_pCameraL->fNrm * sumu * m_varIP;
	CX[4] = k * (v2 * v1 + d2 * ev * ev) * m_varIP;
	CX[5] = CX[7] = k * v2 * m_pCameraL->fNrm * m_varIP;
	CX[8] = k * 2.0 * m_pCameraL->fNrm * m_pCameraL->fNrm * m_varIP;
}

void CRVLStereoVision::Init()
{
	if(m_Flags & RVLSTEREO_FLAGS_CALIB_METHOD_SVS)
		m_BaseLenNrm = m_pCameraR->InvPRNrm[9] - m_pCameraL->InvPRNrm[9];
	
	else
	{
		int i;

		m_BaseLenNrm = 0.0;

		double dP3DC0Nrm;

		for(i = 0; i < 3; i++)
		{
			dP3DC0Nrm = m_pCameraL->P3DC0Nrm[i] - m_pCameraL->P3DC0Nrm[i];

			m_BaseLenNrm += (dP3DC0Nrm * dP3DC0Nrm);
		}

		m_BaseLenNrm = sqrt(m_BaseLenNrm);
	}

	m_pCameraL->m_nrmImage.Width = m_pCameraL->m_Image.Width = m_pCameraL->Width;
	m_pCameraL->m_nrmImage.Height = m_pCameraL->m_Image.Height = m_pCameraL->Height;

	if(m_Flags & RVLSTEREO_FLAGS_METHOD_KINECT)
	{
		//Initialize KINECT params
		//KINECT constants (divided by 2 because of subsampled resolution of 320x240)
		
		//OLD KINECT DATA!
		//m_KinectParams.depthFu = 582.26972991070841/2;
		//m_KinectParams.depthUc = 333.78575299556587/2;
		//m_KinectParams.depthFv = 562.98284310455313/2;
		//m_KinectParams.depthVc = 238.07133397745864/2;

		//m_KinectParams.rgbFu = 521.14532308239393/2;
		//m_KinectParams.rgbUc = 311.66743557620077/2;
		//m_KinectParams.rgbFv = 516.65646258458503/2;
		//m_KinectParams.rgbVc = 259.25911293440481/2;
		
		//m_KinectParams.pPose = new CRVL3DPose;
		//m_KinectParams.pPose->m_Rot[0] = 0.99899332935846452;
		//m_KinectParams.pPose->m_Rot[1] = -0.0015284464595360591;
		//m_KinectParams.pPose->m_Rot[2] = 0.044832931520376568;
		//m_KinectParams.pPose->m_Rot[3] = 0.0027173126649378660;
		//m_KinectParams.pPose->m_Rot[4] = 0.99964594791272010;
		//m_KinectParams.pPose->m_Rot[5] = -0.026468755799252012;
		//m_KinectParams.pPose->m_Rot[6] = -0.044776602251303213;
		//m_KinectParams.pPose->m_Rot[7] = 0.026563935572497543;
		//m_KinectParams.pPose->m_Rot[8] = 0.99864378695194855;
		
		//m_KinectParams.pPose->m_X[0] =  0.026709940276080747;
		//m_KinectParams.pPose->m_X[1] = -0.0056095917183017910; 
		//m_KinectParams.pPose->m_X[2] = -0.0070003194334035236;

		/////////// KORISTENI PARAMETRI ///////////

		m_KinectParams.depthFu = m_KinectParams.depthFu0 / m_KinectScale;
		m_KinectParams.depthUc = m_KinectParams.depthUc0 / m_KinectScale;
		m_KinectParams.depthFv = m_KinectParams.depthFv0 / m_KinectScale;
		m_KinectParams.depthVc = m_KinectParams.depthVc0 / m_KinectScale;
		
		m_KinectParams.rgbFu = 521.50932707979371/m_KinectScale;
		m_KinectParams.rgbUc = 322.17660535484873/m_KinectScale;
		m_KinectParams.rgbFv = 522.56240592356971/m_KinectScale;
		m_KinectParams.rgbVc = 263.39316821976075/m_KinectScale;
		
		m_KinectParams.pPose = new CRVL3DPose;
		m_KinectParams.pPose->m_Rot[0] = 0.99998448339672230;
		m_KinectParams.pPose->m_Rot[1] = -0.0026635069214754063;
		m_KinectParams.pPose->m_Rot[2] = -0.0048927187400962229;
		m_KinectParams.pPose->m_Rot[3] = 0.0027115637569482538;
		m_KinectParams.pPose->m_Rot[4] = 0.99994789104620374;
		m_KinectParams.pPose->m_Rot[5] = 0.0098418806252346565;
		m_KinectParams.pPose->m_Rot[6] = 0.0048662498684758099;
		m_KinectParams.pPose->m_Rot[7] = -0.0098549948314860733;
		m_KinectParams.pPose->m_Rot[8] = 0.99993959752031469;
		
		m_KinectParams.pPose->m_X[0] =  0.026546242279625421;
		m_KinectParams.pPose->m_X[1] = -0.000048908757700053104; 
		m_KinectParams.pPose->m_X[2] = -0.0028166523363169250; 

		////Parameters without distort?
		/*m_KinectParams.depthFu = 585.56866224223631/m_KinectScale;
		m_KinectParams.depthUc = 316.26792237007004/m_KinectScale;
		m_KinectParams.depthFv = 586.60786495871628/m_KinectScale;
		m_KinectParams.depthVc = 246.81163664338146/m_KinectScale;

		m_KinectParams.rgbFu = 521.18355697315383/m_KinectScale;
		m_KinectParams.rgbUc = 319.28799894543192/m_KinectScale;
		m_KinectParams.rgbFv = 522.29636059128597/m_KinectScale;
		m_KinectParams.rgbVc = 259.22861939763811/m_KinectScale;
		
		m_KinectParams.pPose = new CRVL3DPose;
		m_KinectParams.pPose->m_Rot[0] = 0.99999224805435027;
		m_KinectParams.pPose->m_Rot[1] = 0.0024377872532775320;
		m_KinectParams.pPose->m_Rot[2] = 0.0030920906381707190;
		m_KinectParams.pPose->m_Rot[3] = -0.0024318691501572466;
		m_KinectParams.pPose->m_Rot[4] = 0.99999520695564181;
		m_KinectParams.pPose->m_Rot[5] = -0.0019162667297652290;
		m_KinectParams.pPose->m_Rot[6] = -0.0030967472682508152;
		m_KinectParams.pPose->m_Rot[7] = 0.0019087323151372594;
		m_KinectParams.pPose->m_Rot[8] = 0.99999338342676336;
		
		m_KinectParams.pPose->m_X[0] =  0.026667275689053013;
		m_KinectParams.pPose->m_X[1] = 0.000044294836008717494; 
		m_KinectParams.pPose->m_X[2] = 0.0087891702168891236;*/ 	

		////Parameters 3.
		//m_KinectParams.depthFu = 584.70194597700402/m_KinectScale;
		//m_KinectParams.depthUc = 318.55964537649561/m_KinectScale;
		//m_KinectParams.depthFv = 585.70332900816618/m_KinectScale;
		//m_KinectParams.depthVc = 256.14501544470505/m_KinectScale;
		//
		//m_KinectParams.rgbFu = 521.50932707979371/m_KinectScale;
		//m_KinectParams.rgbUc = 322.17660535484873/m_KinectScale;
		//m_KinectParams.rgbFv = 522.56240592356971/m_KinectScale;
		//m_KinectParams.rgbVc = 263.39316821976075/m_KinectScale;
		//
		//m_KinectParams.pPose = new CRVL3DPose;
		//m_KinectParams.pPose->m_Rot[0] = 0.99998448342439450;
		//m_KinectParams.pPose->m_Rot[1] = 0.0027115625740029006;
		//m_KinectParams.pPose->m_Rot[2] = 0.0048662448411574575;
		//m_KinectParams.pPose->m_Rot[3] = -0.0026635059018564940;
		//m_KinectParams.pPose->m_Rot[4] = 0.99994789127814310;
		//m_KinectParams.pPose->m_Rot[5] = -0.0098549715730513066;
		//m_KinectParams.pPose->m_Rot[6] = -0.0048927136394439066;
		//m_KinectParams.pPose->m_Rot[7] = 0.0098418573857854946;
		//m_KinectParams.pPose->m_Rot[8] = 0.99993959777400565;
		//
		//m_KinectParams.pPose->m_X[0] =  0.026546242584187585;
		//m_KinectParams.pPose->m_X[1] = -0.000048908881012111233; 
		//m_KinectParams.pPose->m_X[2] = -0.0028166496710898136;	 
		
		
		//KINECT Params (Cupec)
		m_KinectParams.k = 351218.540681877;
		m_KinectParams.d0 = 1092.93080891449;

		//Create KINECT lookup table for Z coordinate
		m_KinectParams.pZProjLT = new double[2047];
		int i;
		for(i = 0; i < 2047; i++)
			//m_KinectParams.pZProjLT[i] = 0.1236 * tan((i/2842.5) + 1.1863);
			m_KinectParams.pZProjLT[i] = 123.6 * tan((i/2842.5) + 1.1863);

		//Copy KINECT parameters to camera 

		m_pCameraL->fNrm = m_KinectParams.depthFu;
		m_pCameraL->fvNrm = m_KinectParams.depthFv;
		m_pCameraL->CenterXNrm = m_KinectParams.depthUc;
		m_pCameraL->CenterYNrm = m_KinectParams.depthVc;
		m_pCameraL->m_Flags |= RVLCAMERA_FLAG_FV;
	}

	m_ImageWidth = m_pCameraL->Width;
	m_ImageHeight = m_pCameraL->Height;

	m_DisparityMap.Disparity = new short int[m_ImageWidth * m_ImageHeight];
	m_DisparityMap.Width = m_ImageWidth;
	m_DisparityMap.Height = m_ImageHeight;

	m_3DMap = (RVL3DPOINT *)(malloc(m_ImageWidth * m_ImageHeight * 
		sizeof(RVL3DPOINT)));

	m_SADArray = new unsigned short[m_nDisp * m_ImageWidth * m_ImageHeight];

	memset(m_SADArray, 0, m_nDisp * m_ImageWidth * m_ImageHeight * sizeof(short));
}

void CRVLStereoVision::CreateTriangulationLookupTable()
{
	int ImageSize = m_pCameraL->Width * m_pCameraL->Height;

	m_TriangulationLookupTable = new RVLTRIANGULATION_PARAMS[ImageSize];

	RVLTRIANGULATION_PARAMS *pTriangulationParams = m_TriangulationLookupTable;

	double *A1 = m_pCameraL->PRNrm;
	double *c1 = m_pCameraL->InvPRNrm + 9;
	double *A2 = m_pCameraR->PRNrm;
	double *b2 = m_pCameraR->PRNrm + 9;

	m_k1 = (b2[0] - (A2[3 * 0 + 0] * c1[0] + A2[3 * 0 + 1] * c1[1] + A2[3 * 0 + 2] * c1[2])) * 16.0;
	m_k3 = b2[2] - (A2[3 * 2 + 0] * c1[0] + A2[3 * 2 + 1] * c1[1] + A2[3 * 2 + 2] * c1[2]);

	int u, v;
	double fu, fv;
	double P[4], q[2], invP[4];

	for(v = 0; v < m_pCameraL->Height; v++)
	{
		fv = (double)v;

		P[2] = A1[3 * 2 + 0] * fv - A1[3 * 1 + 0];
		P[3] = A1[3 * 2 + 1] * fv - A1[3 * 1 + 1];

		q[1] = A1[3 * 2 + 2] * fv - A1[3 * 1 + 2]; 

		//r[1] = b1[2] * fv - b1[1];

		for(u = 0; u < m_pCameraL->Width; u++)
		{
			fu = (double)(u - m_u1Shift);

			P[0] = A1[3 * 2 + 0] * fu - A1[3 * 0 + 0];
			P[1] = A1[3 * 2 + 1] * fu - A1[3 * 0 + 1];

			q[0] = A1[3 * 2 + 2] * fu - A1[3 * 0 + 2]; 

			//r[0] = b1[2] * fu - b1[0];

			InverseMatrix2(invP, P, APPROX_ZERO);

			pTriangulationParams->f1 = -invP[0] * q[0] - invP[1] * q[1];
			pTriangulationParams->f2 = -invP[2] * q[0] - invP[3] * q[1];

			/*
			pTriangulationParams->g1 = -invP[0] * r[0] - invP[1] * r[1];
			pTriangulationParams->g2 = -invP[2] * r[0] - invP[3] * r[1];

			h1 = b23 + A2[3 * 2 + 0] * pTriangulationParams->g1 + 
				A2[3 * 2 + 1] * pTriangulationParams->g2;
			h2 = b21 + A2[3 * 0 + 0] * pTriangulationParams->g1 + 
				A2[3 * 0 + 1] * pTriangulationParams->g2;
			h3 = A2[3 * 2 + 0] * pTriangulationParams->f1 + 
				A2[3 * 2 + 1] * pTriangulationParams->f2 + A2[3 * 2 + 2];
			h4 = A2[3 * 0 + 0] * pTriangulationParams->f1 + 
				A2[3 * 0 + 1] * pTriangulationParams->f2 + A2[3 * 0 + 2];

			pTriangulationParams->w1 = -h1 / h3;
			pTriangulationParams->w2 = (h1 * fu - h2) / h3 * 16.0;
			pTriangulationParams->w3 = -(fu - h4 / h3) * 16.0;
			*/

			pTriangulationParams->w1 = (A2[3 * 0 + 0] * pTriangulationParams->f1 + 
				A2[3 * 0 + 1] * pTriangulationParams->f2 + A2[3 * 0 + 2]) * 16.0;
			pTriangulationParams->w3 = A2[3 * 2 + 0] * pTriangulationParams->f1 + 
				A2[3 * 2 + 1] * pTriangulationParams->f2 + A2[3 * 2 + 2];

			pTriangulationParams++;
		}
	}
}

void CRVLStereoVision::Get3DWithUncert(unsigned char *EdgeMagnitude, 
									   CRVL3DPose *pPoseLA)
{
	short *pDisparity = m_DisparityMap.Disparity;
	RVL3DPOINT *pP3D = m_3DMap;
	int ImageWidth = m_pCameraL->Width;
	RVLTRIANGULATION_PARAMS *pTriangulationParams = m_TriangulationLookupTable + 
		ImageWidth * m_DisparityMap.v0 + m_DisparityMap.u0;
	unsigned char *pEdgeMagnitude = EdgeMagnitude;

	int u, v, u1, v1, d, u2_16;
	RVLTRIANGULATION_PARAMS *pTriangulationParams2;
	unsigned char *pEdgeMagnitude2;
	double X[3], W3D[3];
	double den;

	for(v = 0; v < m_DisparityMap.Height; v++)
	{
		pTriangulationParams2 = pTriangulationParams;
		pEdgeMagnitude2 = pEdgeMagnitude;

		for(u = 0; u < m_DisparityMap.Width; u++, pDisparity++, pP3D++, pTriangulationParams2++, 
			pEdgeMagnitude2++)
		{
			d = *pDisparity;

			if(d >= m_minDisparity)
			{
				u1 = u + m_DisparityMap.u0;
				v1 = v + m_DisparityMap.v0;

				Triangulation4(u1, v1, d, X, u2_16, den, pTriangulationParams2);

				pPoseLA->Transf(X, pP3D->X);

				GetTriangulationUncert4(den, pTriangulationParams2, W3D);

				pP3D->Wy = pPoseLA->m_Rot[0 + 1 * 3] * W3D[0] + 
					pPoseLA->m_Rot[1 + 1 * 3] * W3D[1] + 
					pPoseLA->m_Rot[2 + 1 * 3] * W3D[2];

				pP3D->u = u1;
				pP3D->v = v1;
				pP3D->d = d + m_DisparityOffsetNrm;

				pP3D->Flags = (RVL3DPOINT_FLAG_3D | RVL3DPOINT_FLAG_W);

				if(*pEdgeMagnitude2 > m_EdgeMagnitudeThr)
					pP3D->Flags |= RVL3DPOINT_FLAG_MASK;
			}
			else
				pP3D->Flags = 0x00;
		}

		pTriangulationParams += ImageWidth;
		pEdgeMagnitude += ImageWidth;
	}
}

// Computes 3D coordinates of image points with respect to the 
// coordinate system S_G

void CRVLStereoVision::Create3DMap(CRVL3DPose *pPoseLA)
{
	short *pDisparity = m_DisparityMap.Disparity;
	RVL3DPOINT *pP3D = m_3DMap;
	int ImageWidth = m_pCameraL->Width;
	RVLTRIANGULATION_PARAMS *pTriangulationParams = m_TriangulationLookupTable + 
		ImageWidth * m_DisparityMap.v0 + m_DisparityMap.u0;

	int u, v, u1, v1, d, u2_16;
	RVLTRIANGULATION_PARAMS *pTriangulationParams2;
	double X[3], W3D[3];
	double den;

	for(v = 0; v < m_DisparityMap.Height; v++)
	{
		pTriangulationParams2 = pTriangulationParams;

		for(u = 0; u < m_DisparityMap.Width; u++, pDisparity++, pP3D++, 
			pTriangulationParams2++)
		{
			d = *pDisparity;

			if(d >= m_minDisparity)
			{
				u1 = u + m_DisparityMap.u0;
				v1 = v + m_DisparityMap.v0;

				Triangulation4(u1, v1, d, X, u2_16, den, pTriangulationParams2);

				pPoseLA->Rot(X, pP3D->X);

				GetTriangulationUncert4(den, pTriangulationParams2, W3D);

				pP3D->Wy = pPoseLA->m_Rot[0 + 1 * 3] * W3D[0] + 
					pPoseLA->m_Rot[1 + 1 * 3] * W3D[1] + 
					pPoseLA->m_Rot[2 + 1 * 3] * W3D[2];

				pP3D->u = u1;
				pP3D->v = v1;
				pP3D->d = d + m_DisparityOffsetNrm;

				pP3D->Flags = (RVL3DPOINT_FLAG_3D | RVL3DPOINT_FLAG_W);
			}
			else
				pP3D->Flags = 0x00;
		}

		pTriangulationParams += ImageWidth;
	}
}

void CRVLStereoVision::GetMinDisparity()
{
	double X[3];

	X[0] = X[1] = 0.0;
	X[2] = m_maxz;

	double fu1, fv1, fu2, fv2;

	Projection(X, m_pCameraL->PRNrm, fu1, fv1);

	Projection(X, m_pCameraR->PRNrm, fu2, fv2);

	m_minDisparity = (short)(DOUBLE2INT((fu1 - fu2) * 16.0) - m_DisparityOffsetNrm);	

	// only for debugging purpose

	/*
	int u1 = DOUBLE2INT(fu1);
	int v1 = DOUBLE2INT(fv1);

	RVLTRIANGULATION_PARAMS *pTriangulationParams = m_TriangulationLookupTable + 
		u1 + v1 * m_BmpWidth;	

	double P3D2[3];
	int u2_16;
	double den;

	Triangulation4(u1, v1, m_minDisparity, P3D2, u2_16, den, pTriangulationParams);

	double W3D[3];

	GetTriangulationUncert4(den, pTriangulationParams, W3D);
	*/
}

void CRVLStereoVision::DisplayHeightMap(PIX_ARRAY &HeightPixArray, 
										double y0)
{
	int n = HeightPixArray.Width * HeightPixArray.Height;
	int maxh = 15;
	int k1 = 200 / (2 * maxh);

	int iPix;
	unsigned char *pPix = HeightPixArray.pPix;
	RVL3DPOINT *pP3D = m_3DMap;
	int h;
	double yCorr;

	for(iPix = 0; iPix < n; iPix++)
	{
	    if(pP3D->Flags)
		{
			if(m_3DMapRotCorr != NULL)
				yCorr = m_3DMapRotCorr[3] * pP3D->X[0] + 
						m_3DMapRotCorr[4] * pP3D->X[1] + 
						m_3DMapRotCorr[5] * pP3D->X[2];
			else
				yCorr = pP3D->X[1];

			h = DOUBLE2INT(yCorr - y0);

			/*
			if(h >= -maxh && h <= maxh)
				*pPix = (unsigned char)(255 - (h + maxh) * k1);
			//else if(h > 50)
			//	*pPix = (unsigned char)(255 - (50 + 50) * k1);
			else
				*pPix = 0;
			*/

			if(h < -maxh)
				//*pPix = 55;
				*pPix = 0;
			else if(h <= maxh)
				*pPix = 55 + (maxh - h) * 100 / maxh;
				//*pPix = 192;
			else
				//*pPix = 255;
				*pPix = 0;
		}
	    else
			*pPix = 0;

	    pPix++;
	    pP3D++;
	}
}


void CRVLStereoVision::CreateDisparityMap()
{

#ifdef SVS
	if(m_Flags & RVLSTEREO_FLAGS_METHOD_SVS)
	{
		svsStereoImage *pStereoImage = (svsStereoImage *)m_vpStereoImage;

		pStereoImage->haveImages = TRUE;
		pStereoImage->haveColor = FALSE;
		pStereoImage->haveColorRight = FALSE;
		pStereoImage->isRectified = TRUE;
		pStereoImage->haveRect = FALSE;
		pStereoImage->haveDisparity = FALSE;  
		pStereoImage->ip.vergence = 0;
		pStereoImage->dp.lr = TRUE;	

		pStereoImage->SetImages(m_pCameraL->m_nrmImage.pPix, 
			m_pCameraR->m_nrmImage.pPix, NULL, NULL);
		
		GetDisparityOffset();
		
		pStereoImage->ip.linelen = m_pCameraL->m_nrmImage.Width;
		pStereoImage->ip.lines = m_pCameraL->m_nrmImage.Height;
		pStereoImage->ip.ix = 0;
		pStereoImage->ip.iy = 0;
		pStereoImage->ip.width = m_pCameraL->m_nrmImage.Width;
		pStereoImage->ip.height = m_pCameraL->m_nrmImage.Height;
		pStereoImage->ip.vergence = 0;
		pStereoImage->rp.left.pwidth = m_pCameraL->m_nrmImage.Width;
		pStereoImage->rp.left.pheight = m_pCameraL->m_nrmImage.Height;
		pStereoImage->rp.right.pwidth = m_pCameraR->m_nrmImage.Width;
		pStereoImage->rp.right.pheight = m_pCameraR->m_nrmImage.Height;
		pStereoImage->dp.corrsize = m_WinSize;
		pStereoImage->dp.thresh = m_ConfThr;
		//pStereoImage->dp.unique = m_UniquenessThr;
		pStereoImage->dp.ndisp = m_nDisp;
		pStereoImage->dp.dpp = m_nSubPixDisps;
		pStereoImage->dp.offx = -m_DisparityOffset;
		pStereoImage->dp.dleft = 0;
		pStereoImage->dp.dtop = 0;
		pStereoImage->dp.dwidth = m_pCameraL->m_nrmImage.Width;
		pStereoImage->dp.dheight = m_pCameraL->m_nrmImage.Height;

		svsStereoProcess StereoProcess;

		StereoProcess.CalcStereo(pStereoImage);

		m_DisparityMap.Disparity = pStereoImage->Disparity();
		m_DisparityMap.Width = m_pCameraL->m_nrmImage.Width;
		m_DisparityMap.Height = m_pCameraL->m_nrmImage.Height;
		m_DisparityMap.u0 = 0;
		m_DisparityMap.v0 = 0;	

		if(m_DisparityFileName)
		{
			pStereoImage->SaveDisparity(m_DisparityFileName);
			
			pStereoImage->SaveParams(m_ParamFileName);
		}
	}
	else
#endif

	if(m_Flags & RVLSTEREO_FLAGS_METHOD_OPENCV)
	{
		IplImage *pDisparityImage = cvCreateImageHeader(cvSize(m_pCameraL->m_nrmImage.Width, m_pCameraL->m_nrmImage.Height) ,IPL_DEPTH_16S,1);

		pDisparityImage->imageData = (char *)(m_DisparityMap.Disparity);

		m_DisparityMap.Width = m_pCameraL->m_nrmImage.Width;
		m_DisparityMap.Height = m_pCameraL->m_nrmImage.Height;
		m_DisparityMap.u0 = 0;
		m_DisparityMap.v0 = 0;	


		CvStereoBMState* state = cvCreateStereoBMState(CV_STEREO_BM_BASIC);

		//Set params
		state->SADWindowSize = m_WinSize;		//5
		state->preFilterSize = 5;
		state->numberOfDisparities = m_nDisp;	//64
		state->textureThreshold = m_ConfThr;	//10
		state->uniquenessRatio = 15;

		IplImage* pLeftImage = cvCreateImageHeader( cvSize(m_pCameraL->m_nrmImage.Width, m_pCameraL->m_nrmImage.Height), IPL_DEPTH_8U, 1  );
		IplImage* pRightImage = cvCreateImageHeader( cvSize(m_pCameraR->m_nrmImage.Width, m_pCameraR->m_nrmImage.Height), IPL_DEPTH_8U, 1  );

		pLeftImage->imageData = (char *)m_pCameraL->m_nrmImage.pPix;

		pRightImage->imageData = (char *)m_pCameraR->m_nrmImage.pPix;

		//pDisparityImage = cvCreateImage(cvSize(m_pCameraL->m_nrmImage.Width, m_pCameraL->m_nrmImage.Height),IPL_DEPTH_16S,1);

		//pDisparityImage->imageData = (char *)malloc(450*375*2);

#ifdef RVLSTEREO_CORRESPONDENCE_SAD_ARRAY
		cvFindStereoCorrespondenceBM_RVL(pLeftImage, pRightImage, pDisparityImage,state, m_SADArray);
#else
		cvFindStereoCorrespondenceBM(pLeftImage, pRightImage, pDisparityImage,state);
#endif

		cvReleaseStereoBMState(&state);
		cvReleaseImageHeader(&pDisparityImage);
		cvReleaseImageHeader(&pLeftImage);
		cvReleaseImageHeader(&pRightImage);
	
	}
}


#ifdef SVS
void CRVLStereoVision::NormalizeStereoImage(PIX_ARRAY *pImageL, 
											PIX_ARRAY *pImageR, 
											PIX_ARRAY *pNrmImageL, 
											PIX_ARRAY *pNrmImageR)
{
	svsStoredImages *pImageAcquisition = 
		(svsStoredImages *)m_vpImageAcquisition;

	pImageAcquisition->Load(pImageL->Width, pImageL->Height,
		pImageL->pPix, pImageR->pPix, NULL, NULL, FALSE, TRUE);

	svsStereoImage *pStereoImage = pImageAcquisition->GetImage(0);

	pNrmImageL->pPix = pStereoImage->Left();
	pNrmImageL->bOwnData = FALSE;
	pNrmImageR->pPix = pStereoImage->Right();
	pNrmImageR->bOwnData = FALSE;	
}

#endif

void CRVLStereoVision::GetDisparityOffset()
{
	DWORD Method = (m_Flags & RVLSTEREO_FLAGS_DISPARITY_OFFSET);

	if(Method == RVLSTEREO_FLAGS_DISPARITY_OFFSET_AUTO)
	{
		double BaseLen = m_pCameraR->InvPRNrm[9] - m_pCameraL->InvPRNrm[9];

		m_DisparityOffset = DOUBLE2INT(m_pCameraL->fNrm * BaseLen / m_minDist -
			m_pCameraR->CenterXNrm +
			m_pCameraL->CenterXNrm - m_nDisp); 

		
	}
	else if(Method == RVLSTEREO_FLAGS_DISPARITY_OFFSET_FILE)
	{
		// add code here when you find time!
	}
}

void CRVLStereoVision::SetDisparityOffset(int DisparityOffset)
{
	m_DisparityOffset = DisparityOffset;

	m_DisparityOffsetNrm = (m_DisparityOffset << 4);
}


void CRVLStereoVision::InitCamerasSVS(char *ParamFileName)
{
	CRVLCamera *pCamera[2];

	pCamera[0] = m_pCameraL;
	pCamera[1] = m_pCameraR;

	CRVLCamera *pCamera2;

	int iCamera, i, j;
	double *pPR;

#ifdef SVS
	svsStoredImages *pImageAcquisition = (svsStoredImages *)m_vpImageAcquisition;

	pImageAcquisition->ReadParams(ParamFileName);

	svsIntrinsicParams *pCameraParams[2];

	pCameraParams[0] = &(pImageAcquisition->GetRP()->left);
	pCameraParams[1] = &(pImageAcquisition->GetRP()->right);

	svsIntrinsicParams *pCameraParams2;

	for(iCamera = 0; iCamera < 2; iCamera++)
	{
		pCamera2 = pCamera[iCamera];
		pCameraParams2 = pCameraParams[iCamera];

		pPR = pCamera2->PRNrm;

		for(i = 0; i < 3; i++)
			for(j = 0; j < 3; j++, pPR++)
				*pPR = (double)(pCameraParams2->proj[i][j]);

		for(i = 0; i < 3; i++, pPR++)
			*pPR = (double)(pCameraParams2->proj[i][3]);

		pCamera2->Width = pCamera2->m_Image.Width = 
			pCamera2->m_nrmImage.Width = pCameraParams2->pwidth;
		pCamera2->Height = pCamera2->m_Image.Height = 
			pCamera2->m_nrmImage.Height = pCameraParams2->pheight;

		pCamera2->m_Image.pPix = pCamera2->m_nrmImage.pPix = NULL;
		pCamera2->m_Image.bOwnData = pCamera2->m_nrmImage.bOwnData = FALSE;
		pCamera2->m_Image.bColor = pCamera2->m_nrmImage.bColor = FALSE;		

		pCamera2->fNrm = pCamera2->PRNrm[0];
		pCamera2->CenterXNrm = pCamera2->PRNrm[2];
		pCamera2->CenterYNrm = pCamera2->PRNrm[5];

		InverseMatrix3(pCamera2->InvPRNrm, pCamera2->PRNrm);

		LinearTransform3D(pCamera2->InvPRNrm, pCamera2->PRNrm + 9, 
			pCamera2->InvPRNrm + 9);

		pCamera2->InvPRNrm[9]  = -pCamera2->InvPRNrm[9];
		pCamera2->InvPRNrm[10] = -pCamera2->InvPRNrm[10];
		pCamera2->InvPRNrm[11] = -pCamera2->InvPRNrm[11];
	}
#else
	FILE *fp = fopen(ParamFileName, "r");

	char line[200];

	iCamera = -1;
	BOOL bReadWidthAndHeight = FALSE;

	while(!feof(fp))
	{
		fgets(line, 200, fp);

		if(strcmp(line, "[left camera]\xa") == 0)
		{
			iCamera = 0;
			bReadWidthAndHeight = TRUE;
		}
		else if(strcmp(line, "[right camera]\xa") == 0)
		{
			iCamera = 1;
			bReadWidthAndHeight = TRUE;
		}
		else if(strcmp(line, "proj \xa") == 0)
		{			
			pPR = pCamera2->PRNrm;

			for(i = 0; i < 3; i++)
			{
				fscanf(fp, " ");

				for(j = 0; j < 3; j++, pPR++)
					fscanf(fp, " %lf", pPR);

				fscanf(fp, " %lf", pCamera2->PRNrm + 9 + i);

				fscanf(fp, "\n");
			}

			pCamera2->fNrm = pCamera2->fvNrm = pCamera2->PRNrm[0];
			pCamera2->CenterXNrm = pCamera2->PRNrm[2];
			pCamera2->CenterYNrm = pCamera2->PRNrm[5];

			InverseMatrix3(pCamera2->InvPRNrm, pCamera2->PRNrm);

			LinearTransform3D(pCamera2->InvPRNrm, pCamera2->PRNrm + 9, pCamera2->InvPRNrm + 9);

			pCamera2->InvPRNrm[9]  = -pCamera2->InvPRNrm[9];
			pCamera2->InvPRNrm[10] = -pCamera2->InvPRNrm[10];
			pCamera2->InvPRNrm[11] = -pCamera2->InvPRNrm[11];
		
			if(iCamera == 1)
				break;
		}

		if(bReadWidthAndHeight)
		{
			bReadWidthAndHeight = FALSE;

			pCamera2 = pCamera[iCamera];

			fscanf(fp, "pwidth %d \n", &(pCamera2->Width));
			fscanf(fp, "pheight %d \n", &(pCamera2->Height));

			pCamera2->m_nrmImage.Width = pCamera2->m_Image.Width = pCamera2->Width;
			pCamera2->m_nrmImage.Height = pCamera2->m_Image.Height = pCamera2->Height;
			pCamera2->m_nrmImage.bColor = pCamera2->m_Image.bColor = FALSE;
		}
	}

	fclose(fp);
#endif
}

void CRVLStereoVision::CreateStereoPointArray()
{
	if(m_Flags & RVLSTEREO_STEREO_POINT_ARRAY)
		return;

	short int *pDisparity = m_DisparityMap.Disparity;
	RVLSTEREOPOINT *pStereoPoint = m_StereoPointArray;

	int u, v;

	for(v = 0; v < m_DisparityMap.Height; v++)
		for(u = 0; u < m_DisparityMap.Width; u++)
			if(*pDisparity >= 0)
			{
				pStereoPoint->disparity = *pDisparity;
				pStereoPoint->u = u;
				pStereoPoint->v = v;
				pStereoPoint++;
			}

	m_nStereoPoints = pStereoPoint - m_StereoPointArray;

	m_Flags |= RVLSTEREO_STEREO_POINT_ARRAY;
}

void CRVLStereoVision::Statistics(float *dGuI, 
								  float *dGvI)
{
	int w = m_pCameraL->m_nrmImage.Width;
	int h = m_pCameraL->m_nrmImage.Height;
	int imageSize = w * h;

	CRVLHistogram histeI, histeIu, histeIv;
	
	histeI.Init(2.0, m_pMem2, "Intensity");
	histeIu.Init(2.0, m_pMem2, "Intensity Gradient u");
	histeIv.Init(2.0, m_pMem2, "Intensity Gradient v");

	short int *D = m_DisparityMap.Disparity;
	unsigned char *I1 = m_pCameraL->m_nrmImage.pPix;
	unsigned char *I2 = m_pCameraR->m_nrmImage.pPix;

	int u1, v1;
	int iPix1, iPix2;
	short int d;
	double eI;
	double eIu;
	//double eIv;

	for(v1 = 3; v1 <= 235; v1++)
		for(u1 = 69; u1 <= 244; u1++)
		{
			iPix1 = u1 + v1 * w;

			d = D[iPix1];

			if(d < 0)
				continue;

			d = ((d + 8) >> 4);

			iPix2 = iPix1 - d;

			eI = (double)((int)I1[iPix1] - (int)I2[iPix2]);

			histeI.m_Data.Add(&eI);

			eIu = (double)(dGuI[iPix1]);

			histeIu.m_Data.Add(&eIu);

			//if(fabs(eI) > 10.0)
			//	D[iPix1] = -1;

			//if(dGuI[iPix1] * dGuI[iPix1] + dGvI[iPix1] * dGvI[iPix1] < 10.0)
			//	D[iPix1] = -1;
		}

	histeI.Create();
	histeIu.Create();

	FILE *fp = fopen("C:\\RVL\\ExpRez\\StereoVisionStatistics.txt", "w");

	histeI.Save(fp);
	histeIu.Save(fp);

	fclose(fp);

	m_pMem2->Clear();
}


// Mathematics behind this function is given in from_disparity_space_to_3D.doc

void CRVLStereoVision::GetUVDPlane(	double *N,
									double d,
									double &a,
									double &b,
									double &c)
{
	if((m_Flags & RVLSTEREO_FLAGS_METHOD) == RVLSTEREO_FLAGS_METHOD_KINECT)
	{
		double k1 = -m_KinectParams.k / d;

		a = k1 * N[0] / m_KinectParams.depthFu;
		b = k1 * N[1] / m_KinectParams.depthFv;
		c = k1 * N[2] - a * m_KinectParams.depthUc - b * m_KinectParams.depthVc + m_KinectParams.d0;
	}
	else
	{
		double k1 = m_BaseLenNrm / d;

		a = k1 * N[0];
		b = k1 * N[1];
		c = k1 * (-N[0] * m_pCameraL->CenterXNrm - N[1] * m_pCameraL->CenterYNrm + N[2] * m_pCameraL->fNrm);
	}
}

void CRVLStereoVision::SADStatistics()
{
	short *pDisparityMapEnd = m_DisparityMap.Disparity + m_ImageWidth * m_ImageHeight;

	short *pDisp;
	short d0;

	// find SAD range

	unsigned short maxSAD = 0;

	unsigned short *SAD = m_SADArray;

	int iminSAD;
	unsigned short minSAD;

	for(pDisp = m_DisparityMap.Disparity; pDisp < pDisparityMapEnd; pDisp++, SAD += m_nDisp)
	{
		d0 = *pDisp;

		if(d0 < 0)
			continue;

		iminSAD = m_nDisp - 1 - ((d0 >> 4) + ((d0 & 15) > 8));

		minSAD = SAD[iminSAD];

		//if(iminSAD > 0)
		//	if(SAD[iminSAD - 1] < minSAD)
		//		minSAD = SAD[iminSAD - 1];

		//if(iminSAD < m_nDisp - 1)
		//	if(SAD[iminSAD + 1] < minSAD)
		//		minSAD = SAD[iminSAD + 1];

		//for(d = 0; d < m_nDisp; d++)
		//	if(SAD[d] < minSAD)
		//		int tmp1 = 0;

		if(minSAD > maxSAD)
			maxSAD = minSAD;
	}

	int *Hist = new int[maxSAD + 1];

	memset(Hist, 0, (maxSAD + 1) * sizeof(int));

	int n = 0;

	SAD = m_SADArray;

	for(pDisp = m_DisparityMap.Disparity; pDisp < pDisparityMapEnd; pDisp++, SAD += m_nDisp)
	{
		d0 = *pDisp;

		if(d0 < 0)
			continue;

		iminSAD = m_nDisp - 1 - (d0 >> 4);

		minSAD = SAD[iminSAD];

		if(iminSAD > 0)
			if(SAD[iminSAD - 1] < minSAD)
				minSAD = SAD[iminSAD - 1];

		if(iminSAD < m_nDisp - 1)
			if(SAD[iminSAD + 1] < minSAD)
				minSAD = SAD[iminSAD + 1];

		Hist[minSAD]++;

		n++;
	}

	FILE *fp;

	fp = fopen("C:\\RVL\\ExpRez\\SASStatistics.dat", "w");

	int SAD2;

	int cumsum = 0;

	for(SAD2 = 0; SAD2 <= maxSAD; SAD2++)
	{
		cumsum += Hist[SAD2];

		fprintf(fp, "%d\t%lf\n", SAD2, (double)(100 * cumsum)/(double)n);
	}

	fclose(fp);

	delete[] Hist;
}


void CRVLStereoVision::CreateParamList(CRVLMem *pMem)
{
	m_ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	m_ParamList.Init();

	pParamData = m_ParamList.AddParam("StereoVision.CameraFileName", RVLPARAM_TYPE_STRING, 
		&m_ParamFileName);

	pParamData = m_ParamList.AddParam("StereoVision.Method", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "SVS", RVLSTEREO_FLAGS_METHOD_SVS);
	m_ParamList.AddID(pParamData, "OPENCV", RVLSTEREO_FLAGS_METHOD_OPENCV);
	m_ParamList.AddID(pParamData, "KINECT", RVLSTEREO_FLAGS_METHOD_KINECT);
	m_ParamList.AddID(pParamData, "SVS_WARP", RVLSTEREO_FLAGS_CALIB_METHOD_SVS);

	pParamData = m_ParamList.AddParam("StereoVision.DisparityRange", RVLPARAM_TYPE_INT, &m_nDisp);

	pParamData = m_ParamList.AddParam("StereoVision.WindowSize", RVLPARAM_TYPE_INT, &m_WinSize);

	pParamData = m_ParamList.AddParam("StereoVision.ConfidenceThr", RVLPARAM_TYPE_INT, &m_ConfThr);

	pParamData = m_ParamList.AddParam("StereoVision.Kinect.Scale", RVLPARAM_TYPE_INT, &m_KinectScale);

	pParamData = m_ParamList.AddParam("StereoVision.Width", RVLPARAM_TYPE_INT, &(m_pCameraL->Width));
	pParamData = m_ParamList.AddParam("StereoVision.Height", RVLPARAM_TYPE_INT, &(m_pCameraL->Height));
	pParamData = m_ParamList.AddParam("StereoVision.Kinect.fu", RVLPARAM_TYPE_DOUBLE, &(m_KinectParams.depthFu0));
	pParamData = m_ParamList.AddParam("StereoVision.Kinect.fv", RVLPARAM_TYPE_DOUBLE, &(m_KinectParams.depthFv0));
	pParamData = m_ParamList.AddParam("StereoVision.Kinect.uc", RVLPARAM_TYPE_DOUBLE, &(m_KinectParams.depthUc0));
	pParamData = m_ParamList.AddParam("StereoVision.Kinect.vc", RVLPARAM_TYPE_DOUBLE, &(m_KinectParams.depthVc0));

	pParamData = m_ParamList.AddParam("StereoVision.maxz", RVLPARAM_TYPE_DOUBLE, &m_maxz);

}

void CRVLStereoVision::Get3DKinect(int u, int v, int d, double *X)
{
	X[2] = m_KinectParams.pZProjLT[d];
	X[0] = (u - m_KinectParams.depthUc) * (X[2]/m_KinectParams.depthFu);
	X[1] = (v - m_KinectParams.depthVc) * (X[2]/m_KinectParams.depthFv);
}

void CRVLStereoVision::GetKinectProjectionMatrix(double *P)
{
	P[0] = m_KinectParams.depthFu;
	P[1] = 0.0;
	P[2] = m_KinectParams.depthUc;
	P[3] = 0.0;
	P[4] = m_KinectParams.depthFv;
	P[5] = m_KinectParams.depthVc;
	P[6] = 0.0;
	P[7] = 0.0;
	P[8] = 1.0;
}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 


BOOL RVLImportDisparityImage(char *FileName, 
						     RVLDISPARITYMAP *pDisparityImage,
							 unsigned int &Format,
							 short *zToDepthLookupTable)
{
	FILE *fp;
	
	fp = fopen(FileName, "r");

	int Width, Height;
	int Size;

	if(fp)
	{
		char line[200];

		fgets(line, 200, fp);

		while(strstr(line, "width") == NULL)
			fgets(line, 200, fp);

		sscanf(line, "width %d", &Width);

		fscanf(fp, "height %d\n", &Height);

		fgets(line, 200, fp);

		if(strcmp(line, "1mm\n") == 0)
			Format = RVLKINECT_DEPTH_IMAGE_FORMAT_1MM;
		else if(strcmp(line, "100um\n") == 0)
			Format = RVLKINECT_DEPTH_IMAGE_FORMAT_100UM;
		else
		{
			Format = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY;

			fclose(fp);

			fp = fopen(FileName, "r");

			fgets(line, 200, fp);
			fgets(line, 200, fp);
		}

		Size = Width * Height;

		if(pDisparityImage->Width * pDisparityImage->Height != Size)
		{
			if(pDisparityImage->Disparity)
				delete[] pDisparityImage->Disparity;

			pDisparityImage->Disparity = new short int[Size];
		}

		pDisparityImage->Width = Width;
		pDisparityImage->Height = Height;
		pDisparityImage->Format = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY;

		BOOL bOK = TRUE;

		short int *pDisparity = pDisparityImage->Disparity;

		int u, v, d;

		for(v = 0; v < Height && bOK; v++)
		{
			for(u = 0; u < Width; u++, pDisparity++)
			{
				if(!(bOK = (fscanf(fp, "%d ", &d) == 1)))
					break;

				if(Format == RVLKINECT_DEPTH_IMAGE_FORMAT_1MM)
					*pDisparity = zToDepthLookupTable[d];
				else
					*pDisparity = (short int)d;
			}
		}

		fclose(fp);

		return bOK;
	}
	else
	{
		char *FileName2 = RVLCreateString(FileName);

		sprintf(FileName2 + strlen(FileName2) - 3, "bmp");

		IplImage *pDisparityBmp = cvLoadImage(FileName2, false);

		if(pDisparityBmp)
		{
			Width = pDisparityBmp->width;
			Height = pDisparityBmp->height;

			Size = Width * Height;

			if(pDisparityImage->Width * pDisparityImage->Height != Size)
			{
				if(pDisparityImage->Disparity)
					delete[] pDisparityImage->Disparity;

				pDisparityImage->Disparity = new short int[Size];
			}

			pDisparityImage->Width = Width;
			pDisparityImage->Height = Height;

			IplImage *pDisparityMap = cvCreateImageHeader(cvSize(pDisparityBmp->width, pDisparityBmp->height), IPL_DEPTH_16S, 1);

			pDisparityMap->imageData = (char *)(pDisparityImage->Disparity);

			cvConvertScale(pDisparityBmp, pDisparityMap, 4);

			cvReleaseImage(&pDisparityBmp);

			cvReleaseImageHeader(&pDisparityMap);

			delete[] FileName2;

			return TRUE;
		}
		else
			return FALSE;
	}
}

void RVLDisplayDisparityMap(RVLDISPARITYMAP *pDisparityMap, 
							BOOL bInverse, 
							PIX_ARRAY &DisparityPixArray)
{
	DisparityPixArray.Width = pDisparityMap->Width;
	DisparityPixArray.Height = pDisparityMap->Height;

	int n = pDisparityMap->Width * pDisparityMap->Height;

	int iPix;
	unsigned char *pPix = DisparityPixArray.pPix;
	short *pDisparity = pDisparityMap->Disparity;

	if(bInverse)
	{
		for(iPix = 0; iPix < n; iPix++)
		{
		    if(*pDisparity >= 0)
				*pPix = 255 - (unsigned char)((*pDisparity) / 4);
		    else
				*pPix = 255;

		    pPix++;
		    pDisparity++;
		}
	}
	else
	{
		for(iPix = 0; iPix < n; iPix++)
		{
		    if(*pDisparity >= 0)
				*pPix = (unsigned char)((*pDisparity) / 4);
		    else
				*pPix = 0;

		    pPix++;
		    pDisparity++;
		}
	}
}

void RVLDisplayDisparityMapColor(RVLDISPARITYMAP *pDisparityMap,
								 short int nDisp,
								 BOOL bInverse, 
								 IplImage *pDisparityImage,
								 unsigned int Format)
{
	PIX_ARRAY DisparityPixArray;

	DisparityPixArray.bOwnData = FALSE;
	DisparityPixArray.bColor = TRUE;
	DisparityPixArray.pPix = (unsigned char *)(pDisparityImage->imageData);

	RVLDisplayDisparityMapColor(pDisparityMap, nDisp, bInverse, &DisparityPixArray, Format);
}


void RVLDisplayDisparityMapColor(RVLDISPARITYMAP *pDisparityMap,
								 short int nDisp,
								 BOOL bInverse, 
								 PIX_ARRAY *pDisparityPixArray,
								 unsigned int Format)
{
	pDisparityPixArray->Width = pDisparityMap->Width;
	pDisparityPixArray->Height = pDisparityMap->Height;

	int n = pDisparityMap->Width * pDisparityMap->Height;

	int maxDisparity = 0;
	int minDisparity = 2048;

	unsigned char *pPix = pDisparityPixArray->pPix;
	short *pDisparity = pDisparityMap->Disparity;

	int iPix;
	unsigned char I;
	int Disparity;

	for(iPix = 0; iPix < n; iPix++, pDisparity++)
	{
		Disparity = (int)(*pDisparity);

		if((Format == RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY && Disparity >= 0 && Disparity < 2047) ||
			((Format == RVLKINECT_DEPTH_IMAGE_FORMAT_1MM || Format == RVLKINECT_DEPTH_IMAGE_FORMAT_100UM) 
			&& Disparity > 0))
		{
			if(Disparity > maxDisparity)
				maxDisparity = Disparity;

			if(Disparity < minDisparity)
				minDisparity = Disparity;
		}
	}

	int DisparityRange = maxDisparity - minDisparity;

	pDisparity = pDisparityMap->Disparity;	

	if(bInverse)
	{
		for(iPix = 0; iPix < n; iPix++)
		{
		    if(*pDisparity >= 0)
			{
				I = 255 - (unsigned char)((int)(*pDisparity) * 255 / maxDisparity);

				*(pPix++) = I;
				*(pPix++) = I;
				*(pPix++) = I;
			}
		    else
			{
				*(pPix++) = 255;
				*(pPix++) = 255;
				*(pPix++) = 255;
			}

		    pDisparity++;
		}
	}
	else
	{
		for(iPix = 0; iPix < n; iPix++)
		{
			Disparity = (int)(*pDisparity);

		    if((Format == RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY && Disparity >= 0 && Disparity < 2047) ||
				((Format == RVLKINECT_DEPTH_IMAGE_FORMAT_1MM || Format == RVLKINECT_DEPTH_IMAGE_FORMAT_100UM) 
				&& Disparity > 0))
			{
				I = 64 + (unsigned char)((Disparity - minDisparity) * (255 - 64) / DisparityRange);

				*(pPix++) = I;
				*(pPix++) = I;
				*(pPix++) = I;
			}
		    else
			{
				*(pPix++) = 0;
				*(pPix++) = 0;
				*(pPix++) = 0;
			}

		    pDisparity++;
		}
	}
}


#ifdef SVS
WORD CRVLStereoVision::CameraInit()
{
	m_vpVideoObject = (void *)getVideoObject();

	svsVideoImages *pVideoObject = (svsVideoImages *)m_vpVideoObject;

	bool ret = pVideoObject->Open();

	if (!ret)
		return RVLSTEREO_RES_CANT_OPEN_CAMERA;

	if(pVideoObject->ReadParams("C:\\RVL\\Calibration\\Calib_081124\\megad-75.ini"))  
		pVideoObject->SaveParams("C:\\RVL\\Calibration\\Calib_081124\\megad-75.dat"); 
	else
		return RVLSTEREO_RES_CANT_LOAD_PARAMS;

	if(!pVideoObject->SetRate(3)) //3   15
		return RVLSTEREO_RES_CANT_SET_FRAME_RATE;

	int width=320, height=240;
	//svsWindow *win1 = new svsWindow(width,height);
	//svsWindow *win2 = new svsWindow(width,height);
	pVideoObject->SetSize(width,height);

	pVideoObject->SetExposure(20, 15, TRUE, TRUE);

	if(!pVideoObject->Start())
		return RVLSTEREO_RES_CANT_START_ACQUISITION;

	return RVL_RES_OK;
}

WORD CRVLStereoVision::ImageAcquisition(int n)
{
	svsVideoImages *pVideoObject = (svsVideoImages *)m_vpVideoObject;

	svsStereoImage *imageObject;

	int i;

	for(i = 0; i < n; i++)
		imageObject = pVideoObject->GetImage(100);  //100 200
	
	m_vpStereoImage = (void *)imageObject;
		
	if(!imageObject)   //imageObject == NULL
		return RVLSTEREO_RES_IMAGE_ACQUISITION_FAILED;

	if(m_pCameraL->m_Image.bOwnData)
		if(m_pCameraL->m_Image.pPix)
			delete[] m_pCameraL->m_Image.pPix;

	m_pCameraL->m_Image.bOwnData = FALSE;
	m_pCameraL->m_Image.pPix = imageObject->Left();
	m_pCameraL->m_Image.Width = 320;
	m_pCameraL->m_Image.Height = 240;

	if(m_pCameraR->m_Image.bOwnData)
		if(m_pCameraR->m_Image.pPix)
			delete[] m_pCameraR->m_Image.pPix;

	m_pCameraR->m_Image.bOwnData = FALSE;
	m_pCameraR->m_Image.pPix = imageObject->Right();
	m_pCameraR->m_Image.Width = 320;
	m_pCameraR->m_Image.Height = 240;		

	return RVL_RES_OK;
}

void CRVLStereoVision::CameraStop()
{
	if(m_vpVideoObject)
	{
		svsVideoImages *pVideoObject = (svsVideoImages *)m_vpVideoObject;

		pVideoObject->Stop();
	}
}
#endif	// SVS

void RVLSaveStereoPts(FILE *fp,
					  RVL3DPOINT2 *PtArray,
					  int nPts)
{
	int i;
	RVL3DPOINT2 *pPt;

	for(i = 0; i < nPts; i++)
	{
		pPt = PtArray + i;

		fprintf(fp, "%lf\t%lf\t%lf\n", pPt->x, pPt->y, pPt->z);
	}
}


//Get xyz coordinates from uvd values
void RVLGetKinect3DData(int *pUVDPt, double *pXYZPt, RVLKINECT_PARAMS kinectParams)
{
	//new method (Cupec)
	// pXYZPt[2] = kinectParams.k / (kinectParams.d0 - pUVDPt[2]);
	//z
	pXYZPt[2] = kinectParams.pZProjLT[pUVDPt[2]];
	//x
	pXYZPt[0] = (pUVDPt[0] - kinectParams.depthUc) * (pXYZPt[2]/kinectParams.depthFu);
	//y
	pXYZPt[1] = (pUVDPt[1] - kinectParams.depthVc) * (pXYZPt[2]/kinectParams.depthFv);

}

//Get uvd values from xyz coordinates
void RVLGetKinect2DData(int *pUVDPt, double *pXYZPt, RVLKINECT_PARAMS kinectParams)
{
	//d
	//pUVDPt[2] = (int)(2842.5 * (atan(pXYZPt[2]/0.1236) - 1.1863));
	pUVDPt[2] = (int)RVLKINECTZTODEPTH(pXYZPt[2]);
	//new method (Cupec)
	// pUVDPt[2] = kinectParams.d0 - (kinectParams.k/pXYZPt[2]);

	//u
	pUVDPt[0] = (int)(kinectParams.depthUc + (pXYZPt[0] * kinectParams.depthFu/pXYZPt[2]));

	//v
	pUVDPt[1] = (int)(kinectParams.depthVc + (pXYZPt[1] * kinectParams.depthFv/pXYZPt[2]));

}

//Get RGB uv coordinates from uvd values
void RVLGetKinectRGBProjection(int *pUVDPt, int *pRGBPt, RVLKINECT_PARAMS *kinectParams)
{
	double pXYZPt[3];
	
	//z
	pXYZPt[2] = kinectParams->pZProjLT[pUVDPt[2]];
	//x
	pXYZPt[0] = (pUVDPt[0] - kinectParams->depthUc) * (pXYZPt[2]/kinectParams->depthFu);
	//y
	pXYZPt[1] = (pUVDPt[1] - kinectParams->depthVc) * (pXYZPt[2]/kinectParams->depthFv);

	RVLGetKinectRGBProjection(pXYZPt,pRGBPt, kinectParams);
}


//Get RGB uv coordinates from xyz coordinates
void RVLGetKinectRGBProjection(double *pXYZPt, int *pRGBPt, RVLKINECT_PARAMS *kinectParams)
{
	double temp3D[3], C3D[3];

	RVLDif3D(pXYZPt,kinectParams->pPose->m_X,temp3D);
	MatrixMultiplication(kinectParams->pPose->m_Rot,temp3D,C3D,3,3,1);

	pRGBPt[0] = (int)((C3D[0] * kinectParams->rgbFu) / C3D[2] + kinectParams->rgbUc);
	pRGBPt[1] = (int)((C3D[1] * kinectParams->rgbFv) / C3D[2] + kinectParams->rgbVc);

	int w = 320-1, h = 240-1;

	pRGBPt[0] = (pRGBPt[0]<0 ? 0 : pRGBPt[0]);
	pRGBPt[0] = (pRGBPt[0]>w ? w : pRGBPt[0]);

	pRGBPt[1] = (pRGBPt[1]<0 ? 0 : pRGBPt[1]);
	pRGBPt[1] = (pRGBPt[1]>h ? h : pRGBPt[1]);

			

}


void RVLSaveDepthImage(short *iDepth, int w, int h, char *DepthFileName, DWORD SrcFormat, DWORD TgtFormat, short *zToDepthLT)
{
	FILE *fpDepth = fopen(DepthFileName,"w");

	fprintf(fpDepth, "width %d\n", w);
	fprintf(fpDepth, "height %d\n", h);

	if(TgtFormat == RVLKINECT_DEPTH_IMAGE_FORMAT_1MM)
		fprintf(fpDepth, "1mm\n");
	else if(TgtFormat == RVLKINECT_DEPTH_IMAGE_FORMAT_100UM)
		fprintf(fpDepth, "100um\n");
	else
		fprintf(fpDepth, "\n");

	short d;

	for(int r=0;r<h;r++)
	{
		for(int c=0;c<w;c++)
		{
			d = iDepth[r*w + c];

			if(SrcFormat != TgtFormat)
			{
				if(SrcFormat == RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY && TgtFormat == RVLKINECT_DEPTH_IMAGE_FORMAT_1MM)
				{
					if(d < 2047)
						fprintf(fpDepth, "%d ", (int)RVLKINECTDEPTHTOZ(d));
					else
						fprintf(fpDepth, "0 ");
				}
				else if(SrcFormat == RVLKINECT_DEPTH_IMAGE_FORMAT_1MM && TgtFormat == RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY)
				{
					if(zToDepthLT)
						fprintf(fpDepth, "%d ", zToDepthLT[d]);
					else
						fprintf(fpDepth, "%d ", (d > 0 ? (int)RVLKINECTZTODEPTH((double)d) : 2047));
				}
			}
			else
				fprintf(fpDepth,"%d ",d);
		}
		fprintf(fpDepth,"\n");
	}
	fclose(fpDepth);
}