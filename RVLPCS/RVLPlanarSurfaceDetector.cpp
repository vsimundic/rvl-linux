// RVLPlanarSurfaceDetector.cpp: implementation of the CRVLPlanarSurfaceDetector class.
//
// Functions 
//     
//     LADPlaneThreePoints()
//     ThreePointsModified_searchb()
//     ThreePointsAbSearch_afor()
//     ThreePointsAbSearch_bfor()
//     WeightedMedian()
//
// written by R. Grbic and adapted for RVL by R. Cupec
//
// Osijek, 2008
//
//////////////////////////////////////////////////////////////////////

//#include "Platform.h"

//#include "highgui.h"
#include "RVLCore.h"
#include "RVLPCS.h"
#include "RVLPlanarSurfaceDetector.h"


#ifdef RVLPSDLAD_GRBIC
#include "Timer.h"
#endif

#ifdef RVLPSD_SEGMENT_STRM_PC_DEBUG
#include <list>

bool bDebugTriangle;
bool bDebug;
bool *bDebugMap;
double x0Debug = 3301.0;
double y0Debug = -357.6; 
double z0Debug = 27010.0;
double rDebug = 5000.0;
std::list<CRVL2DRegion2 *> debugTriangles;
FILE *fpDebugTriangles;
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLPlanarSurfaceDetector::CRVLPlanarSurfaceDetector()
{	
	
	//m_Segmentation = NULL;
	m_Point3DArray = NULL;
	m_Point3DMap = NULL;
	m_Point3DMapMem = NULL;
	m_Point3DBuff = NULL;
	m_RegionGrowingMap = NULL;
	m_EmptyRegionGrowingMap = NULL;
	m_RegionGrowingBuff = NULL;
	m_CCBuff = NULL;
	m_WallArray = NULL;	
	m_ResidualHistogram = NULL;
	m_QueueMem = NULL;
	m_iScanLineStart = NULL;
	m_iScanLineEnd = NULL;
	m_pGround = NULL;
	m_bCorner = NULL;
	m_DTMap = NULL;
	m_DTQueueEntryMem = NULL;
	m_2DRegionMap = NULL;
	m_3DSurfaceMap = NULL;
	m_APixBuff = NULL;
	m_nFOVExtensions = 0;
	m_FOVExtension = 2.0 * PI / 3.0;
	m_nWalls = 0;
	m_Flags = 0x00000000;
	m_Flags2 = 0x00000000;
	m_Tol = 16.0;
	m_ResidualThr = 80.0;
	m_MEstThr = 2.0;
	m_ITol = 15;
	m_ScoreScale = m_Tol * 0.5;
	m_alpha = 1.3;
	m_BinSize = 4.0;
	m_nBins = 4 * 32;
	m_rho = 0.12;
	m_WinSize = 31;
	//m_WinSize = 11;
	m_RelTol = 2.5;
	m_kminnCBin = 0.044;
	m_nRANSACIterations = 200;
	m_kminnInitPts = 0.5;
	m_minnLADRANSACInit3DPoints = 20;
	m_kLADRANSACInit3DPoints = 0.5 * 0.5 * (double)m_WinSize;
	m_enableMinDisparity = false;
	m_minDisparity = 0;
	m_maxDisparity = 2046;
	m_enableTimeLimit = false;
	m_maxSegmentTime = 0.2;
	m_GroundBeta = 0.0;
	m_GroundBetaTol = 10.0 * DEG2RAD;
	m_GroundThetaTol = 10.0 * DEG2RAD;
	//m_maxdist = 5;
	m_nCCDilationIterations = 5;
	m_uNrm = 8;
	m_uvdTol = 4; //4
	m_MeshTol = 2;
	m_uvTol = 2;  //2
	m_RuvdTol = 1.0;
	m_RuvTol = 1.0;
	m_fillPerc = 50;
	//m_maxDis = 4;
	m_MinConvexSegmentSize = 5;
	m_maxSampleSize = 7;
	m_minSampleSize = 4;
	m_MeshPlanarSegWERThr1 = 3;
	m_MeshPlanarSegWERThr2 = 10;
	m_MeshDiscontinuityThr = 0.3;
	m_PointMeasurementUncertStD = 10.0;

	// the default values of the follwing parameters are adjusted according to wang_TPAMI04

	m_SubSample = 5;					// default: 8
	m_minSegmentSize = 1000;			// default: 80
	m_NoiseThr = m_minSegmentSize;		// default: 5
	m_minnInliers = 100;
	m_DiscontinuitiyThr = 32;			// default: 9
	m_minnValids = 10;
	m_AngleThr = 45.0 * DEG2RAD;

	m_MeshSegmentWERMaxCost = 1000;
	m_MeshSegmentWERk = 10.0;

	/////
	
	int i;

	for(i = 0; i < 10001; i++)
		m_expLT[i] = exp(-0.01 * (double)i);


	//*********used in Get3DSurfaceAndContours***********
	dMatRot = cvCreateMatHeader(3, 3, CV_64FC1);
	dMatRot->data.db = rotMat;

	dMatPL = cvCreateMatHeader(3, 1, CV_64FC1);
	dMatPL->data.db = PL;

	dMatPL1 = cvCreateMatHeader(3, 1, CV_64FC1);
	dMatPL1->data.db = PL1;

	dMatSqr = cvCreateMatHeader(2, 2, CV_64FC1);
	dMatSqr->data.db = sqrMat;

	dMatEig = cvCreateMatHeader(2, 2, CV_64FC1);
	dMatEig->data.db = eigMat;

	dMatEigVal = cvCreateMatHeader(2, 1, CV_64FC1);
	dMatEigVal->data.db = eigVal;

	dMatRotFinal = cvCreateMatHeader(3, 3, CV_64FC1);
	dMatRotFinal->data.db = rotMatFinal;
	
	dMatRotF = cvCreateMatHeader(3, 3, CV_64FC1);
	dMatRotF->data.db = rotMatF;
	//*******************************************************
}

CRVLPlanarSurfaceDetector::~CRVLPlanarSurfaceDetector()
{
	//if(m_Segmentation)
	//	delete[] m_Segmentation;

	if(m_Point3DArray)
		delete[] m_Point3DArray;

	if(m_Point3DMapMem)
		delete[] m_Point3DMapMem;

	if(m_Point3DBuff)
		delete[] m_Point3DBuff;

	if(m_RegionGrowingMap)
		delete[] m_RegionGrowingMap;

	if(m_EmptyRegionGrowingMap)
		delete[] m_EmptyRegionGrowingMap;

	if(m_RegionGrowingBuff)
		delete[] m_RegionGrowingBuff;

	if(m_CCBuff)
		delete[] m_CCBuff;

	if(m_WallArray)
		delete[] m_WallArray;

	if(m_ResidualHistogram)
		delete[] m_ResidualHistogram;

	if(m_QueueMem)
		delete[] m_QueueMem;

	if(m_iScanLineStart)
		delete[] m_iScanLineStart;

	if(m_iScanLineEnd)
		delete[] m_iScanLineEnd;

	if(m_bCorner)
		delete[] m_bCorner;

	if(m_DTMap)
		delete[] m_DTMap;

	if(m_DTQueueEntryMem)
		delete[] m_DTQueueEntryMem;

	if(m_2DRegionMap)
		delete[] m_2DRegionMap;

	if(m_APixBuff)
		delete[] m_APixBuff;

	if(m_3DSurfaceMap)
		delete[] m_3DSurfaceMap;

	//*********used in Get3DSurfaceAndContours***********
	cvReleaseMat(&dMatRot);
	cvReleaseMat(&dMatPL);
	cvReleaseMat(&dMatPL1);
	cvReleaseMat(&dMatSqr);
	cvReleaseMat(&dMatEig);
	cvReleaseMat(&dMatEigVal);
	cvReleaseMat(&dMatRotFinal);
	cvReleaseMat(&dMatRotF);
	//*******************************************************
}

#ifndef RVLPSDLAD_GRBIC
void CRVLPlanarSurfaceDetector::Apply(CRVLStereoVision *pStereoVision)
{
	short int *DisparityMap = pStereoVision->m_DisparityMap.Disparity;
	RVLSTEREOPOINT *StereoPointArray = pStereoVision->m_StereoPointArray;

	pStereoVision->CreateStereoPointArray();

	int w = pStereoVision->m_DisparityMap.Width;

	int minnInitPoints = (int)ceil(m_kminnInitPts * (double)(m_WinSize * m_WinSize));

	double *StereoPointPtrBuff = 
		(double *)(m_pMem->Alloc(m_WinSize * m_WinSize * 3 * sizeof(double)));
	
	int iIteration;
	RVLSTEREOPOINT *pStereoPt0;
	double *pStereoPointPtrBuff;
	int u, v;
	short int *pd, *pd0;
	short int d;
	int n;

	for(iIteration = 0; iIteration < m_nRANSACIterations; iIteration++)
	{
		// select an initial point

		pStereoPt0 = pStereoVision->m_StereoPointArray + 
			RVLRandom(0, pStereoVision->m_nStereoPoints - 1);

		if(pStereoPt0->u + m_WinSize > w)
			continue;

		if(pStereoPt0->v + m_WinSize > pStereoVision->m_DisparityMap.Height)
			continue;

		// store all points from the selected window into StereoPointPtrBuff

		pd0 = DisparityMap + pStereoPt0->u + pStereoPt0->v * w;

		pStereoPointPtrBuff = StereoPointPtrBuff;

		for(v = pStereoPt0->v; v < pStereoPt0->v + m_WinSize; v++, pd0 += w)
		{
			pd = pd0;

			for(u = pStereoPt0->u; u < pStereoPt0->u + m_WinSize; u++, pd++)
			{
				d = *pd;

				if(d < 0)
					continue;

				*(pStereoPointPtrBuff++) = (double)u;
				*(pStereoPointPtrBuff++) = (double)v;
				*(pStereoPointPtrBuff++) = (double)d;
			}
		}

		n = (pStereoPointPtrBuff - StereoPointPtrBuff) / 3;

		if(n < minnInitPoints)
			continue;

		// compute the initial LAD-plane


	}	// for(iIteration = 0; iIteration < m_nRANSACIterations; iIteration++)

	m_pMem2->Clear();
}
#endif // RVLPSDLAD_GRBIC


#ifdef RVLPSDLAD_GRBIC
BOOL CRVLPlanarSurfaceDetector::ThreePtsPlane(RVL3DPOINT2 **Point3DPtrArray,
											        int n,
													double &a, double &b, double &c)

#else
BOOL CRVLPlanarSurfaceDetector::ThreePtsPlane(RVL3DPOINT2 **Point3DPtrArray,
											        int n,
													CRVL2DRegion2 *pPlane)
#endif
{
	RVL3DPOINT2 *pPoint3D0, *pPoint3D1, *pPoint3D2;
	bool samePoint = false;
	
	pPoint3D0 = *(Point3DPtrArray + RVLRandom(0, n - 1));

	pPoint3D1 = *(Point3DPtrArray + RVLRandom(0, n - 1));

	pPoint3D2 = *(Point3DPtrArray + RVLRandom(0, n - 1));

#ifdef RVLPSDLAD_GRBIC
	 return (Plane(pPoint3D0, pPoint3D1, pPoint3D2, a, b, c));
#else
	return (Plane(pPoint3D0, pPoint3D1, pPoint3D2, pPlane));
#endif


}




#ifdef RVLPSDLAD_GRBIC
BOOL CRVLPlanarSurfaceDetector::LADPlaneThreePoints(RVL3DPOINT2 **Point3DPtrArray,
											        int n,
													double &a, double &b, double &c)
#else
BOOL CRVLPlanarSurfaceDetector::LADPlaneThreePoints(RVL3DPOINT2 **Point3DPtrArray,
											        int n,
													CRVL2DRegion2 *pPlane)
#endif
{
	//predlozeni ThreePoints za clanak
	bool bRezult = false;

	double Gp = 0, Gpp = 0;
	double tol = 1e-6;
	int up, down;
	RVL3DPOINT2 tocka_p, tocka_mi, tocka_ni;
	RVLPSDLAD_THREE_POINTS_ITER_AB search_b, search_third;
	m_Point3DPtrArray = new RVL3DPOINT2 *[n];
	RVL3DPOINT2 *pPoint3D;

	//m_nIterations = 2;

	//pocetna tocka je centroid

	tocka_p.x = tocka_p.y = tocka_p.z = 0;

	double fn = (double)n;
	
	int i;
	
	for(i = 0; i < n; i++)
	{
		pPoint3D = Point3DPtrArray[i];

		tocka_p.x += pPoint3D->x;
		tocka_p.y += pPoint3D->y;
		tocka_p.z += pPoint3D->z;
	}

	tocka_p.x /= fn;
	tocka_p.y /= fn;
	tocka_p.z /= fn;

	//if true then do what is asked for otherwise
	//trazenje najboljeg b i druge tocke
	//search_b = ThreePointsModified_searchb(Point3DPtrArray, n, tocka_p);
	if(ThreePointsModified_searchb(Point3DPtrArray, n, tocka_p,search_b))
	{
		bRezult = true;
		tocka_mi = search_b.tocka;
		Gpp = search_b.G;		

		//trazenje trece tocke
		if ((fabs(tocka_p.x-tocka_mi.x))>tol)
			search_third = ThreePointsAbSearch_bfor(Point3DPtrArray, n, tocka_mi, tocka_p);
		else
			search_third = ThreePointsAbSearch_afor(Point3DPtrArray, n, tocka_mi, tocka_p);
		tocka_ni = search_third.tocka;
		Gp = search_third.G;

		//priprema za novi krug
		tocka_p = tocka_mi;
		tocka_mi = tocka_ni;
		double eps = 1e-5;
		
		while (1)
		{
			up = 0;
			down = 0;
			//m_nIterations++;
			
	#ifdef RVLPSDLAD_GRBIC
			a = search_third.a;
			b = search_third.b;
			c = search_third.c;
	#else
			pPlane->m_a = search_third.a;
			pPlane->m_b = search_third.b;
			pPlane->m_c = search_third.c;
	#endif
			//m_TotalDistance = search_third.G;

			/*for (int i=0; i<DataLength; i++)
			{
				if (Z[i] < (m_Plane.a*X[i] + m_Plane.b*Y[i] + m_Plane.c))
		//			if (Z[i] < (m_Plane.a*X[i] + m_Plane.b*Y[i] + m_Plane.c - eps))
						down++;

				if (Z[i] > (m_Plane.a*X[i] + m_Plane.b*Y[i] + m_Plane.c))
		//			if (Z[i] > (m_Plane.a*X[i] + m_Plane.b*Y[i] + m_Plane.c + eps))
						up++;
			}
			if ((up <= DataLength/2) && (down <= DataLength/2))
				return m_Plane;*/
		

			if ((fabs(tocka_mi.x-tocka_p.x))>tol)
				search_third = ThreePointsAbSearch_bfor(Point3DPtrArray, n, tocka_mi, tocka_p);
			else
				search_third = ThreePointsAbSearch_afor(Point3DPtrArray, n, tocka_mi, tocka_p);

			if ((search_third.G-Gp)>tol || ((fabs(search_third.G-Gp)<tol) && (fabs(Gp-Gpp)<tol)))
				break;

			tocka_ni = search_third.tocka;
			
			//priprema za novi krug
			tocka_p = tocka_mi;
			tocka_mi = tocka_ni;

			Gpp = Gp;
			Gp = search_third.G;
		}
	}
	delete[] m_Point3DPtrArray;

	return bRezult;
}



BOOL CRVLPlanarSurfaceDetector::ThreePointsModified_searchb(
	 RVL3DPOINT2 **Point3DPtrArray, 
	 int ArraySize,
	 RVL3DPOINT2 &tocka_p,
	 RVLPSDLAD_THREE_POINTS_ITER_AB &search_b)
{
	//search for best b and the second point

	RVLPSDLAD_THREE_POINTS_ITER_AB temp;
	//RVLPSDLAD_KIP_TWO_POINTS temp1;
	RVL3DPOINT2 **Point3DPtrArray2 = m_Point3DPtrArray;
	RVL3DPOINT2 *pPoint3D;
	//double *pod = new double [ArraySize];
	//double *weights = new double [ArraySize];
	//int *indeksi = new int [ArraySize];
	double temp2=0;

	int i = 0;
	int p = 0;
	
	for(i = 0; i < ArraySize; i++)	
	{
		pPoint3D = Point3DPtrArray[i];

		if (pPoint3D->y - tocka_p.y)
		{
			/*
			weights[p] = Point3DArray[i].y - tocka_p.y;
			pod[p] = (Point3DArray[i].z - tocka_p.z)/weights[p];
			
			if (weights[p]<0)
				weights[p] *= -1;
			indeksi[p]=i;
			p++;
			*/
			pPoint3D->weight = pPoint3D->y - tocka_p.y;
			pPoint3D->pod = (pPoint3D->z - tocka_p.z)/pPoint3D->weight;
			
			if (pPoint3D->weight<0)
				pPoint3D->weight *= -1;
			pPoint3D->index=i;

			Point3DPtrArray2[p++] = pPoint3D;
		}
	}


	if(p>=2)
	{
		//temp1 = WeightedMedian(pod, weights, indeksi, 0, p-1);
		temp.tocka = (*(WeightedMedian(m_Point3DPtrArray, 0, p-1, (p - 1)/2)));

		temp.a = 0;
		temp.b = temp.tocka.pod;
		temp.c = tocka_p.z - temp.a*tocka_p.x - temp.b*tocka_p.y;
		
		temp.G = 0;
		
		for(i = 0; i < ArraySize; i++)
		{
			pPoint3D = Point3DPtrArray[i];

			temp2 = temp.a*pPoint3D->x + temp.b*pPoint3D->y + temp.c - pPoint3D->z;
			if (temp2<0)
				temp.G -= temp2;
			else
				temp.G += temp2;
		}

		//delete[] m_Point3DPtrArray;
		//delete [] pod;
		//delete [] weights;
		//delete [] indeksi;
		
		//return temp;
		search_b = temp;
		return true;
	}
	else
		return false;

}


RVLPSDLAD_THREE_POINTS_ITER_AB CRVLPlanarSurfaceDetector::ThreePointsAbSearch_afor(
	 RVL3DPOINT2 **Point3DPtrArray, 
	 int ArraySize,
	 RVL3DPOINT2 &tocka_mi,
	 RVL3DPOINT2 &tocka_p)
{
	/*a formule*/
	//CTimer *m_pTimer_1;
	//m_pTimer_1 = new CTimer;
	//DWORD vrijeme;

	RVLPSDLAD_THREE_POINTS_ITER_AB temp;
	//RVLPSDLAD_KIP_TWO_POINTS temp1;
	RVL3DPOINT2 *pPoint3D;
	RVL3DPOINT2 **Point3DPtrArray2 = m_Point3DPtrArray;
	//double *pod = new double [ArraySize];
	//double *weights = new double [ArraySize];
	//int *indeksi = new int [ArraySize];
	double temp2, con1, con2;
	double tol = 1e-5;
	double dx,dy,cond; 

	int i = 0;
	int p = 0;
	con1 = (tocka_mi.x - tocka_p.x)/(tocka_mi.y - tocka_p.y);
	con2 = (tocka_mi.z - tocka_p.z)/(tocka_mi.y - tocka_p.y);

	//m_pTimer_1->Reset();
	//m_pTimer_1->Start();

	for (i = 0; i < ArraySize; i++)
	{
		pPoint3D = Point3DPtrArray[i];

		dx = pPoint3D->x-tocka_p.x;
		dy = pPoint3D->y-tocka_p.y;
		cond = dx - dy*con1;

		//if (!(((fabs(cond))<tol) || (((fabs(dx))<tol) && ((fabs(dy))<tol)) || (((fabs(X[i]-tocka_mi.x))<tol) && ((fabs(Y[i]-tocka_mi.y))<tol))))

		if (cond<tol)
			if(cond > -tol)
				continue;
		{
			/*
			weights[p] = cond;
			pod[p] = (Point3DArray[i].z - tocka_p.z - dy*con2)/weights[p];
			if (weights[p]<0)
				weights[p] *= -1;
			indeksi[p]=i;
			*/
			pPoint3D->weight = cond;
			pPoint3D->pod = (pPoint3D->z - tocka_p.z - dy*con2)/pPoint3D->weight;
			if (pPoint3D->weight<0)
				pPoint3D->weight *= -1;
			pPoint3D->index=i;

			Point3DPtrArray2[p++] = pPoint3D;
		}
	}
	//m_pTimer_1->End();
	//vrijeme = m_pTimer_1->GetEllapsedMS();

	//temp1 = WeightedMedian(pod, weights, indeksi, 0, p-1);
	temp.tocka = (*(WeightedMedian(m_Point3DPtrArray, 0, p-1, (p - 1)/2)));
	temp.a = temp.tocka.pod;
	temp.b = con2 - temp.a*con1;
	temp.c = tocka_p.z - temp.a*tocka_p.x - temp.b*tocka_p.y;

	temp.G = 0;	
	for (i = 0; i < ArraySize; i++)
	{
		pPoint3D = Point3DPtrArray[i];

		temp2 = temp.a*pPoint3D->x + temp.b*pPoint3D->y + temp.c - pPoint3D->z;
		if (temp2<0)
			temp.G -= temp2;
		else
			temp.G += temp2;
	}

	//delete[] Point3DPtrArray;
	//delete [] pod;
	//delete [] weights;
	//delete [] indeksi;
	//delete m_pTimer_1;

	return temp;
}


RVLPSDLAD_THREE_POINTS_ITER_AB CRVLPlanarSurfaceDetector::ThreePointsAbSearch_bfor(
	 RVL3DPOINT2 **Point3DPtrArray, 
	 int ArraySize,
	 RVL3DPOINT2 &tocka_mi,
	 RVL3DPOINT2 &tocka_p)
{
	/*b formule*/

	//CTimer *m_pTimer_1;
	//m_pTimer_1 = new CTimer;
	//DWORD vrijeme;

	RVLPSDLAD_THREE_POINTS_ITER_AB temp;
	//RVLPSDLAD_KIP_TWO_POINTS temp1;
	RVL3DPOINT2 *pPoint3D;
	RVL3DPOINT2 **Point3DPtrArray2 = m_Point3DPtrArray;
	//double *pod = new double [ArraySize];
	//double *weights = new double [ArraySize];
	//int *indeksi = new int [ArraySize];
	double temp2, con1, con2;
	double tol = 1e-5;
	double dx,dy,cond;

	int p = 0;
	int i = 0;
	con1 = (tocka_mi.y - tocka_p.y)/(tocka_mi.x - tocka_p.x);
	con2 = (tocka_mi.z - tocka_p.z)/(tocka_mi.x - tocka_p.x);
	
	//m_pTimer_1->Reset();
	//m_pTimer_1->Start();
	for (i = 0; i < ArraySize; i++)
	{
		pPoint3D = Point3DPtrArray[i];

		dx = pPoint3D->x-tocka_p.x;
		dy = pPoint3D->y-tocka_p.y;
		cond = dy - dx*con1;

		//if (!(((fabs(cond))<tol) || (((fabs(dx))<tol) && ((fabs(dy))<tol)) || (((fabs(X[i]-tocka_mi.x))<tol) && ((fabs(Y[i]-tocka_mi.y))<tol))))
		if (cond<tol)
			if(cond > -tol)
				continue;

		{
			/*
			weights[p] = cond;
			pod[p] = (Point3DArray[i].z - tocka_p.z - dx*con2)/weights[p];
			if (weights[p]<0)
				weights[p] *= -1;
			indeksi[p]=i;
			p++;
			*/
			pPoint3D->weight = cond;
			pPoint3D->pod = (pPoint3D->z - tocka_p.z - dx*con2)/pPoint3D->weight;
			if (pPoint3D->weight<0)
				pPoint3D->weight *= -1;
			pPoint3D->index=i;

			Point3DPtrArray2[p++] = pPoint3D;
		}
	}
	//m_pTimer_1->End();
	//vrijeme = m_pTimer_1->GetEllapsedMS();

	//temp1 = WeightedMedian(pod, weights, indeksi, 0, p-1);
	temp.tocka = (*(WeightedMedian(m_Point3DPtrArray, 0, p-1, (p - 1)/2)));

	temp.b = temp.tocka.pod;
	temp.a = con2 - temp.b*con1;
	temp.c = tocka_p.z - temp.a*tocka_p.x - temp.b*tocka_p.y;

	temp.G = 0;	

	for (i = 0; i < ArraySize; i++)
	{
		pPoint3D = Point3DPtrArray[i];

		temp2 = temp.a*pPoint3D->x + temp.b*pPoint3D->y + temp.c - pPoint3D->z;
		if (temp2<0)
			temp.G -= temp2;
		else
			temp.G += temp2;
	}	

	//delete[] Point3DPtrArray;
	//delete [] pod;
	//delete [] weights;
	//delete [] indeksi;
	//delete  m_pTimer_1;

	return temp;

}

/*
RVLPSDLAD_KIP_TWO_POINTS CRVLPlanarSurfaceDetector::WeightedMedian(
	double *DataSort, 
	double *DataWeights, 
	int *Indeksi, 
	int left, int right)
{
	//solve weighted median problem without complete sort of array X 
	// R.Grbic

	RVLPSDLAD_KIP_TWO_POINTS result;
	int left_old = left;
	int right_old = right;
	int median;
	double W0, Wp, WL;
	
	W0 = 0;
	WL = 0;
	Wp = 0;

	for (int i=0; i<=right; i++)
		W0 += DataWeights[i];
	
	W0 /=2;

	while(1)
	{
		int mid = (left + right)/2;
		double center = DataSort[mid];

		//first swap pivot into starting position
		swap(DataSort, DataWeights, Indeksi, left, mid);
		
		WL = DataWeights[left];

		int i = left+1;
		int j = right;
		

 		while (i<=j)
		{
			if (DataSort[i] < DataSort[left])
			{
				WL += DataWeights[i];
				i++;
			}
			else
			{
				if (DataSort[j] < DataSort[left])
				{
					swap (DataSort, DataWeights, Indeksi, i, j);
					WL +=DataWeights[i];
					i++;
				}
				j--;
			}
		}
		
		//edit this part
		if ((j==right) || (i==(left+1)))		//if pivot is the largest element or the smallest element
		{
			if (i==(left+1))					//if pivot is the smallest element
			{
				if ((WL+Wp)>W0)					//check if pivot is weighted median of data
				{
					median = left;
					break;
				}					
				Wp = Wp + DataWeights[left];
			}				

			if (j==right)						//if pivot is the largest element
			{
				if ((Wp+WL-DataWeights[left])<=W0)					//check if pivot is weighted median of data
				{
					median = left;			
					break;
				}
			}
			
			left += 1;							//just remove that element if it's not weighted median
			left_old = left;
		}

		else
		{
			if ( (WL+Wp)>W0 )			//choose left part
			{
				left = left_old;
				right = i-1;
			}
			else						//choose right part
			{			
				left = i;
				right = right_old;
				Wp = WL + Wp;
			}
			left_old = left;
			right_old = right;
		}
		
		if (((right-left)==1) )	//if there is two data left
		{
			if (DataSort[left]>DataSort[right])
				swap (DataSort, DataWeights, Indeksi, left, right);

			if ((Wp+DataWeights[left])>W0)
				median = left;
			else
				median = right;

			break;
		}

	}

	result.ak = DataSort[median];
	result.indeks = Indeksi[median];

	return result;
}
*/

RVL3DPOINT2 *CRVLPlanarSurfaceDetector::WeightedMedian(
	RVL3DPOINT2 **Point3DPtrArray, 
	int left, int right, 
	int pivot)
{
	//solve weighted median problem without complete sort of array X 
	// R.Grbic

	int left_old = left;
	int right_old = right;
	int median;
	double W0, Wp, WL;
	RVL3DPOINT2 *tmp;
	
	W0 = 0;
	WL = 0;
	Wp = 0;

	for (int i=0; i<=right; i++)
		W0 += Point3DPtrArray[i]->weight;
	
	W0 /=2;	

	//int mid = (left + right)/2;

	int mid = pivot;

	while(1)
	{			
		double center = Point3DPtrArray[mid]->pod;

		//first swap pivot into starting position
		//swap(DataSort, DataWeights, Indeksi, left, mid);
		tmp = Point3DPtrArray[left];
		Point3DPtrArray[left] = Point3DPtrArray[mid];
		Point3DPtrArray[mid] = tmp;
		
		WL = Point3DPtrArray[left]->weight;

		int i = left+1;
		int j = right;
		
		/*
 		while (i<=j)
		{
			if (Point3DPtrArray[i]->pod < Point3DPtrArray[left]->pod)
			{
				WL += Point3DPtrArray[i]->weight;
				i++;
			}
			else
			{
				if (Point3DPtrArray[j]->pod < Point3DPtrArray[left]->pod)
				{
					//swap (DataSort, DataWeights, Indeksi, i, j);
					tmp = Point3DPtrArray[i];
					Point3DPtrArray[i] = Point3DPtrArray[j];
					Point3DPtrArray[j] = tmp;
					WL +=Point3DPtrArray[i]->weight;
					i++;
				}
				j--;
			}
		}
		*/

		while(i <= j)
		{
			while(i <= j)
			{
				if(Point3DPtrArray[i]->pod >= Point3DPtrArray[left]->pod)
					break;

				WL += Point3DPtrArray[i]->weight;
				i++;
			}

			while(i <= j)
			{
				if(Point3DPtrArray[j]->pod < Point3DPtrArray[left]->pod)
				{
					tmp = Point3DPtrArray[i];
					Point3DPtrArray[i] = Point3DPtrArray[j];
					Point3DPtrArray[j] = tmp;	
					WL +=Point3DPtrArray[i]->weight;
					i++;
					j--;
					break;
				}

				j--;
			}
		}

		
		//edit this part
		if ((j==right) || (i==(left+1)))		//if pivot is the largest element or the smallest element
		{
			if (i==(left+1))					//if pivot is the smallest element
			{
				if ((WL+Wp)>W0)					//check if pivot is weighted median of data
				{
					median = left;
					break;
				}					
				Wp = Wp + Point3DPtrArray[left]->weight;
			}				

			if (j==right)						//if pivot is the largest element
			{
				if ((Wp+WL-Point3DPtrArray[left]->weight)<=W0)					//check if pivot is weighted median of data
				{
					median = left;			
					break;
				}
			}
			
			left += 1;							//just remove that element if it's not weighted median
			left_old = left;
		}

		else
		{
			if ( (WL+Wp)>W0 )			//choose left part
			{
				left = left_old;
				right = i-1;
			}
			else						//choose right part
			{			
				left = i;
				right = right_old;
				Wp = WL + Wp;
			}
			left_old = left;
			right_old = right;
		}
		
		if (((right-left)==1) )	//if there is two data left
		{
			if (Point3DPtrArray[left]->pod > Point3DPtrArray[right]->pod)
			{
				//swap (DataSort, DataWeights, Indeksi, left, right);
				tmp = Point3DPtrArray[left];
				Point3DPtrArray[left] = Point3DPtrArray[right];
				Point3DPtrArray[right] = tmp;
			}

			if ((Wp+Point3DPtrArray[left]->weight)>W0)
				median = left;
			else
				median = right;

			break;
		}

		mid = (left + right)/2;
	}

	return Point3DPtrArray[median];
}


//	LS-fitting of a plane to a set A of 3D points.
//	Points are represented by structures RVL3DPOINT2,
//	where fields x, y and z represent the point coordinates
//	The output are the values of the parameters m_a, m_b and m_c
//	of pPlane.
//	Details are given in RVMath.doc, Section 2


#ifdef RVLPSDLAD_GRBIC
void CRVLPlanarSurfaceDetector::LSPlane(RVL3DPOINT2 **Point3DPtrArray,
										int n,
										double &a, double &b, double &c)
#else
// When moving to the new version of OpenCV function cvReleaseMatHeader became unknown.
// Hence, I commented it out in this function.
// If this function is ever to be used, cvReleaseMatHeader must be substituted by some other appropriate function.
void CRVLPlanarSurfaceDetector::LSPlane(RVL3DPOINT2 **Point3DPtrArray,		// Input: set A of 3D points
										int n,								// Input: num. of pts. in A
										CRVL2DRegion2 *pPlane,				// Output: Best LS-plane
										DWORD Flags)
#endif
{
	RVL3DPOINT2 *pPoint3D;

	double A[3 * 3];
	memset(A, 0, 3 * 3 * sizeof(double));

	double v[3];
	v[0] = v[1] = v[2] = 0.0;

	double a, b, c;

	if(Flags & RVLPSDLSPLANE_FLAG_M_ESTIMATOR)
	{
		a = pPlane->m_a;
		b = pPlane->m_b;
		c = pPlane->m_c;
	}

	int i;
	double w;
	double wx, wy;
	double res;
	
	for(i = 0; i < n; i++)
	{
		pPoint3D = Point3DPtrArray[i];

		if(Flags & RVLPSDLSPLANE_FLAG_M_ESTIMATOR)		
		{
			res = fabs(a * pPoint3D->x + b * pPoint3D->y + c - pPoint3D->z);

			if(res <= m_MEstThr)
				w = 1.0;
			else
				w = m_MEstThr / res;
		}
		else
			w = 1.0;

		wx = w * pPoint3D->x;
		wy = w * pPoint3D->y;

		A[0*3+0] += (wx * pPoint3D->x);
		A[0*3+1] += (wx * pPoint3D->y);
		A[0*3+2] += wx;
		A[1*3+1] += (wy * pPoint3D->y);
		A[1*3+2] += wy;
		A[2*3+2] += w;
		v[0] += (wx * pPoint3D->z);
		v[1] += (wy * pPoint3D->z);
		v[2] += w * pPoint3D->z;
	}

	A[1*3+0] = A[0*3+1];
	A[2*3+0] = A[0*3+2];
	A[2*3+1] = A[1*3+2];

	CvMat *A_ = cvCreateMatHeader(3, 3, CV_64FC1);
	A_->data.db = A;
	CvMat *v_ = cvCreateMatHeader(3, 1, CV_64FC1);
	v_->data.db = v;

	double p[3];
	CvMat *p_ = cvCreateMatHeader(3, 1, CV_64FC1);
	p_->data.db = p;

	cvSolve(A_, v_, p_);

#ifdef RVLPSDLAD_GRBIC
	a = p[0];
	b = p[1];
	c = p[2];
#else
	pPlane->m_a = p[0];
	pPlane->m_b = p[1];
	pPlane->m_c = p[2];
#endif

	//cvReleaseMatHeader(&A_);
	//cvReleaseMatHeader(&v_);
	//cvReleaseMatHeader(&p_);
}

//	RANSAC-based detection of dominant planar surface in a set A of stereo points
//	
//	Relevant member variables:
//
//		Parameters:
//			m_enableTimeLimit		-	time-limited
//			m_maxt					-	max. duration of RANSAC loop
//			m_nRANSACIterations		-	num. of RANSAC iterations
//
//			the follwoing parameters are used only if at least one of flags
//				RVLPSDRANSAC_FLAG_LADINIT
//				RVLPSDRANSAC_FLAG_LSINIT
//				RVLPSDRANSAC_FLAG_3PTSINIT
//			is set:
//
//			m_WinSize				-	size of the hypothesis generation window;
//										used only if Flags ? and ? are set
//			m_kLADRANSACInit3DPoints	-	used only if Flags ? and ? are set

//
//		Inputs:
//			m_Point3DArray			-	set A of stereo points; used if ppPoint3DArray == NULL;
//										stereo points are represented by structures RVL3DPOINT2;
//										fields x, y and z of these structures are u, v and d
//										components of stereo points
//			m_n3DPoints				-	num. of pts. in set A
//			m_Point3DMap			-	array of the size of the input image; 
//										each element is a ptr. to a stereo point;
//										if an element is NULL, then this image point is not
//										reconstructed by stereo vision;
//
//	Flags:
//		RVLPSDRANSAC_FLAG_LADINIT
//		RVLPSDRANSAC_FLAG_LSINIT
//		RVLPSDRANSAC_FLAG_3PTSINIT
//		RVLPSDRANSAC_FLAG_PT_DISTRIBUTION
//		RVLPSD_FLAG_GROUND
//		RVLPSDRANSAC_FLAG_LAD						-	final plane fitting by searching for best LAD-plane
//		RVLPSDRANSAC_FLAG_LS						-	final plane fitting by searching for best LS-plane
//		RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET
//		RVLPSDRANSAC_FLAG_RESC	
//		RVLPSDRANSAC_FLAG_WEIGHTED_SCORE		

#ifdef RVLPSDLAD_GRBIC
int CRVLPlanarSurfaceDetector::RANSAC(double &a, double &b, double &c,
									  bool bSegment,
									  RVL3DPOINT2 **ppPoint3DArray,
									  int n3DPointsIn,
									  RVLPSDLAD_SEGMENT_DETAILS *segmentDetails,
									  DWORD Flags)
#else
int CRVLPlanarSurfaceDetector::RANSAC(CRVL2DRegion2 *pPlane,						//	Output:	dominant planar surface
									  bool bSegment,								//	Param:	Karlo ?;	default: false
									  RVL3DPOINT2 **ppPoint3DArray,					//	Input:	set A of stereo points
									  int n3DPointsIn,								//	Input:	num. of pts. in A
									  RVLPSDLAD_SEGMENT_DETAILS *segmentDetails,	
									  DWORD Flags)
#endif
{
	double t0,t; //,tt0,tt1,tt2=0,tt3=0;
	
	if(bSegment || m_enableTimeLimit)
		t0 = m_pTimer->GetTime();

#ifndef RVLPSDLAD_GRBIC
	double a = pPlane->m_a;
	double b = pPlane->m_b;
	double c = pPlane->m_c;
#endif

	RVL3DPOINT2 *pPoint3D;

	RVL3DPOINT2 **Point3DArray;
	RVL3DPOINT2 **Point3DOriginalArray;
	int n3DPoints;

	if (!ppPoint3DArray)
	{
		//GetPointsWithDisparity(pDisparityMap);
		
		Point3DOriginalArray = new RVL3DPOINT2 *[m_n3DPoints];

		RVL3DPOINT2 **ppPoint3DArr = Point3DOriginalArray;
		
		pPoint3D = m_Point3DArray;
		
		for(int iP=0; iP<m_n3DPoints; iP++, ppPoint3DArr++)
		{
			*ppPoint3DArr = pPoint3D;
			pPoint3D++;
		}

		Point3DArray = Point3DOriginalArray;

		n3DPoints = m_n3DPoints;
	}
	else
	{
		Point3DArray = ppPoint3DArray;

		n3DPoints = n3DPointsIn;
	}

	// from now on, Point3DArray represents the input point set A
	
	RVL3DPOINT2 **Point3DMap = m_Point3DMap;

	RVL3DPOINT2 **Point3DPtrBuff1 = new RVL3DPOINT2 *[n3DPoints];
	RVL3DPOINT2 **Point3DPtrBuff2 = new RVL3DPOINT2 *[n3DPoints];

	RVL3DPOINT2 **ConsensusSet = Point3DPtrBuff1;
	RVL3DPOINT2 **BestConsensusSet = Point3DPtrBuff2;

	RVL3DPOINT2 **InitSet = new RVL3DPOINT2 *[m_WinSize * m_WinSize];
	RVL3DPOINT2 **tmp;

	double k1 = 2.0 * m_kLADRANSACInit3DPoints * m_kLADRANSACInit3DPoints;

	int nBest = 0;
	double BestScore = 0.0;

	RVLRECT Win;
	int nInit;
	double aBest, bBest, cBest;
	int iIteration;
	int nConsensus;
	RVL3DPOINT2 *pPoint3D0, *pPoint3D1, *pPoint3D2;
	int Su, Sv, Suu, Suv, Svv;
	double avgu, avgv, Suunrm, Suvnrm, Svvnrm;
	double k2, k3;
	int i;
	int u, v;
	double fnInit;
	double Score;
	CRVL3DSurface2 Plane3D;

	for(iIteration = 0; iIteration < m_nRANSACIterations; iIteration++)  //m_nRANSACIterations
	{	
		if(bSegment || m_enableTimeLimit)
		{
			t = m_pTimer->GetTime() - t0;

			if(!bSegment && m_enableTimeLimit && t > m_maxt)
				break;

			if(bSegment && t > m_maxSegmentTime)
				break;
		}
		
		//tt0 = m_pTimer->GetTime();

		// initial hypothesis

		if(Flags & (RVLPSDRANSAC_FLAG_LADINIT | RVLPSDRANSAC_FLAG_LSINIT | RVLPSDRANSAC_FLAG_3PTSINIT))
		{
			while(TRUE)
			{
				Win.left = RVLRandom(0, m_Width - m_WinSize);
				Win.top = RVLRandom(0, m_Height - m_WinSize);
				Win.right = Win.left + m_WinSize - 1;
				Win.bottom = Win.top + m_WinSize - 1;

				nInit = GetPointsWithDisparity(&Win, InitSet);

				if(nInit < m_minnLADRANSACInit3DPoints)
					continue;

				if(Flags & RVLPSDRANSAC_FLAG_PT_DISTRIBUTION)
				{
					Suu = 0;
					Suv = 0;
					Svv = 0;
					Su = 0;
					Sv = 0;

					for(i = 0; i < nInit; i++)
					{
						pPoint3D = InitSet[i];

						u = pPoint3D->u;
						v = pPoint3D->v;
						Su += u;
						Sv += v;
						Suu += u * u;
						Suv += u * v;
						Svv += v * v;
					}

					fnInit = (double)nInit;

					avgu = (double)Su / fnInit;
					avgv = (double)Sv / fnInit;

					Suunrm = (double)Suu / fnInit - avgu * avgu;
					Suvnrm = (double)Suv / fnInit - avgu * avgv;
					Svvnrm = (double)Svv / fnInit - avgv * avgv;

					k2 = Suunrm + Svvnrm - k1;

					if(k2 < 0.0)
						continue;

					k3 = Suunrm - Svvnrm;

					// only for debugging purpose !!!

					//double eig2 = 0.5 * ((k2 + k1) - sqrt(k3 * k3 + 4.0 * Suvnrm * Suvnrm));

					/////

					if(k3 * k3 + 4.0 * Suvnrm * Suvnrm <= k2 * k2)
						break;
				}
				else
					break;
			}	// while(TRUE)

			if(Flags & RVLPSDRANSAC_FLAG_LADINIT)
			{
#ifdef RVLPSDLAD_GRBIC
				if(!LADPlaneThreePoints(InitSet, nInit, a, b, c))
					continue;
				//LADPlaneThreePoints(InitSet, nInit, a, b, c);
#else
				if(!LADPlaneThreePoints(InitSet, nInit, pPlane))
					continue;
				//LADPlaneThreePoints(InitSet, nInit, pPlane);
#endif
			}
			else if(Flags & RVLPSDRANSAC_FLAG_LSINIT)
			{
#ifdef RVLPSDLAD_GRBIC
				LSPlane(InitSet, nInit, a, b, c);
#else
				LSPlane(InitSet, nInit, pPlane);
#endif
			}
			else
			{
#ifdef RVLPSDLAD_GRBIC
			if(!ThreePtsPlane(InitSet, nInit, a, b, c))
				continue;
#else
			if(!ThreePtsPlane(InitSet, nInit, pPlane))
				continue;
#endif
			}
			if(Flags & RVLPSD_FLAG_GROUND)
			{
				Get3DPlane(pPlane, &Plane3D);

				if(fabs(Plane3D.m_N[0]) > sin(m_GroundThetaTol))
					continue;

				if(fabs(-asin(Plane3D.m_N[2]) - m_GroundBeta) > m_GroundBetaTol)
					continue;
			}
		}	// RANSAC initialization by fitting all data inside a window around a randomly 
			// selected point
		else	// 'classic' RANSAC
		{
			// randomly select 3 points from set A

			pPoint3D0 = *(Point3DArray + RVLRandom(0, n3DPoints - 1));

			pPoint3D1 = *(Point3DArray + RVLRandom(0, n3DPoints - 1));

			pPoint3D2 = *(Point3DArray + RVLRandom(0, n3DPoints - 1));

			// instantiate a model

#ifdef RVLPSDLAD_GRBIC
			if(!Plane(pPoint3D0, pPoint3D1, pPoint3D2, a, b, c))
				continue;
#else
			if(!Plane(pPoint3D0, pPoint3D1, pPoint3D2, pPlane))
				continue;
#endif
		}

		//tt1 = m_pTimer->GetTime();

		//tt2 += tt1-tt0;

#ifdef RVLPSDLAD_GRBIC
		Score = Consensus(Point3DArray, n3DPoints, a, b, c, ConsensusSet, nConsensus);
#else
		Score = Consensus(Point3DArray, n3DPoints, pPlane, ConsensusSet, nConsensus, Flags, BestScore);
#endif

		//tt3 += m_pTimer->GetTime() - tt1;

		// update the best plane

		//if(nConsensus > nBest)
		if(Score > BestScore)
		{
			BestScore = Score;
			nBest = nConsensus;

#ifdef RVLPSDLAD_GRBIC
			aBest = a;
			bBest = b;
			cBest = c;
#else
			aBest = pPlane->m_a;
			bBest = pPlane->m_b;
			cBest = pPlane->m_c;
#endif

			tmp = ConsensusSet;
			ConsensusSet = BestConsensusSet;
			BestConsensusSet = tmp;
		}		
	}	// for(iIteration = 0; iIteration < m_nRANSACIterations; iIteration++)	

#ifdef RVLPSDLAD_GRBIC
	a = aBest;
	b = bBest;
	c = cBest;
#endif

	//for(iIteration = 0; iIteration < m_nRANSACIterations; iIteration++)

	// final fitting

	if(nBest)
	{
		if(Flags & RVLPSDRANSAC_FLAG_LAD)
#ifdef RVLPSDLAD_GRBIC
			LADPlaneThreePoints(BestConsensusSet, nBest, a, b, c);
#else
			LADPlaneThreePoints(BestConsensusSet, nBest, pPlane);
#endif
		else if(Flags & RVLPSDRANSAC_FLAG_LS)
#ifdef RVLPSDLAD_GRBIC
			LSPlane(BestConsensusSet, nBest, a, b, c);
#else
			LSPlane(BestConsensusSet, nBest, pPlane);
#endif
		else
		{
			pPlane->m_a = aBest;
			pPlane->m_b = bBest;
			pPlane->m_c = cBest;
		}
	}

	//final consensus set

	if((Flags & RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET) == 0)
	{
		Score = Consensus(Point3DArray, n3DPoints, pPlane, ConsensusSet, nConsensus);

		if(bSegment)
		{					
			//if(Score > BestScore)
			//{
			BestScore = Score;
			nBest = nConsensus;
				
			//}
			
			
			segmentDetails->a = pPlane->m_a;
			segmentDetails->b = pPlane->m_b;
			segmentDetails->c = pPlane->m_c;
			segmentDetails->numberOfIterations = iIteration;
			
			tmp = ConsensusSet;

			int _cnt=0,cnt=0;
			for(i=0; i<nBest; i++,tmp++)
			{
				pPoint3D= *tmp;
				if(pPoint3D->segmentNumber == -1)
				{
					pPoint3D->segmentNumber = segmentDetails->segmentNumber;
					cnt++;
				}
				else
				{
					_cnt++;
				}
			}

			segmentDetails->numberOfPoints = cnt;
			
		}
	}
	
	delete[] Point3DPtrBuff1;
	delete[] Point3DPtrBuff2;
	delete[] InitSet;
	if(!ppPoint3DArray)
		delete[] Point3DOriginalArray;

	return nBest;
}


//	Computes plane parameters from three 3D points.
//	Points are represented by structures RVL3DPOINT2,
//	where fields x, y and z represent the point coordinates
//	The output are the values of the parameters m_a, m_b and m_c
//	of pPlane.
//	Details are given in RVMath.doc, Section 1
	
// NOTE: When moving to the new version of OpenCV function cvReleaseMatHeader became unknown.
// Hence, I commented it out in this function.
// If this function is ever to be used, cvReleaseMatHeader must be substituted by some other appropriate function.

#ifdef RVLPSDLAD_GRBIC
BOOL CRVLPlanarSurfaceDetector::Plane(RVL3DPOINT2 *pPoint3D0, 
									  RVL3DPOINT2 *pPoint3D1, 
									  RVL3DPOINT2 *pPoint3D2,
									  double &a, double &b, double &c)
#else
BOOL CRVLPlanarSurfaceDetector::Plane(RVL3DPOINT2 *pPoint3D0, 
									  RVL3DPOINT2 *pPoint3D1, 
									  RVL3DPOINT2 *pPoint3D2,
									  CRVL2DRegion2 *pPlane)
#endif
{
	double A[3 * 3];

	A[0*3+0] = pPoint3D0->x;
	A[0*3+1] = pPoint3D0->y;
	A[0*3+2] = 1.0;

	A[1*3+0] = pPoint3D1->x;
	A[1*3+1] = pPoint3D1->y;
	A[1*3+2] = 1.0;

	A[2*3+0] = pPoint3D2->x;
	A[2*3+1] = pPoint3D2->y;
	A[2*3+2] = 1.0;

	double Z[3];

	Z[0] = pPoint3D0->z;
	Z[1] = pPoint3D1->z;
	Z[2] = pPoint3D2->z;

	CvMat *A_ = cvCreateMatHeader(3, 3, CV_64FC1);
	A_->data.db = A;
	CvMat *Z_ = cvCreateMatHeader(3, 1, CV_64FC1);
	Z_->data.db = Z;

	double p[3];
	CvMat *p_ = cvCreateMatHeader(3, 1, CV_64FC1);
	p_->data.db = p;
	
	if(cvSolve(A_, Z_, p_))
	{
#ifdef RVLPSDLAD_GRBIC
		a = p[0];
		b = p[1];
		c = p[2];	
#else
		pPlane->m_a = p[0];
		pPlane->m_b = p[1];
		pPlane->m_c = p[2];	
#endif
		//cvReleaseMatHeader(&A_);
		//cvReleaseMatHeader(&Z_);
		//cvReleaseMatHeader(&p_);

		return TRUE;
	}
	else
	{
		//cvReleaseMatHeader(&A_);
		//cvReleaseMatHeader(&Z_);
		//cvReleaseMatHeader(&p_);

		return FALSE;
	}
}

#ifdef RVLPSDLAD_GRBIC
int CRVLPlanarSurfaceDetector::GetNumberOfInliers(RVL3DPOINT2 *Point3DArray,
											      int n,
											      double a, double b, double c)
#else
int CRVLPlanarSurfaceDetector::GetNumberOfInliers(RVL3DPOINT2 *Point3DArray,
											      int n,
											      CRVL2DRegion2 *pPlane)
#endif

{
	RVL3DPOINT2 *pPoint3D = Point3DArray;

#ifndef RVLPSDLAD_GRBIC
	double a = pPlane->m_a;
	double b = pPlane->m_b;
	double c = pPlane->m_c;
#endif

	int nInliers = 0;

	int i;
	double e;

	for(i = 0; i < n; i++, pPoint3D++)
	{
		e = pPoint3D->z - (a * pPoint3D->x + b * pPoint3D->y + c);

		if(e > m_Tol)
			continue;

		if(e >= -m_Tol)
			nInliers++;
	}
	
	return nInliers;
}

//	Determines the set containing all points from a stereo point set Point3DArray 
//	which lie approximatelly on a plane pPlane
//
//	Parameters:
//		m_Tol			-	
//		m_ScoreScale	-	relevant only if flag RVLPSDRANSAC_FLAG_WEIGHTED_SCORE is set
//
//	RESC-method is described in SUR101

#ifdef RVLPSDLAD_GRBIC
double CRVLPlanarSurfaceDetector::Consensus(RVL3DPOINT2 **Point3DArray,
											int n,
											double a, double b, double c,
											RVL3DPOINT2 **ConsensusSet,
											int &nConsensus)
#else
double CRVLPlanarSurfaceDetector::Consensus(RVL3DPOINT2 **Point3DArray,		//	Input:	stereo point set
											int n,							//	Input:	num. of pts. in Point3DArray
											CRVL2DRegion2 *pPlane,			//	Output:	plane
											RVL3DPOINT2 **ConsensusSet,		//	Output:	set of points lying 
																			//			approximatelly on plane pPlane
											int &nConsensus,				//	Output:	num. of pts. in ConsensusSet
											DWORD Flags,
											double BestScore)				//	Input:	relevant only if flag 
																			//			RVLPSDRANSAC_FLAG_RESC is set
#endif
{
	RVL3DPOINT2 **ppPoint3D = Point3DArray;
	RVL3DPOINT2 *pPoint3D;

#ifndef RVLPSDLAD_GRBIC
	double a = pPlane->m_a;
	double b = pPlane->m_b;
	double c = pPlane->m_c;
#endif

	RVL3DPOINT2 **pConsensusSet = ConsensusSet;	
	double Tol = m_Tol;
	double var = m_ScoreScale * m_ScoreScale * 0.01;
	double Score = 0.0;
	
	int i;
	double e;

	if(Flags & RVLPSDRANSAC_FLAG_RESC)
	{	
		// RESC

		RVL3DPOINT2 **ppPt3D = m_Point3DBuff;

		int nBins = DOUBLE2INT(m_ResidualThr);

		memset(m_ResidualHistogram, 0, nBins * sizeof(int));

		for(i = 0; i < n; i++, ppPoint3D++)
		{
			pPoint3D = *ppPoint3D;

			e = pPoint3D->z - (a * pPoint3D->x + b * pPoint3D->y + c);

			pPoint3D->e = e;

			if(e < 0.0) 
				e = -e;

			if(e >= m_ResidualThr)
				continue;

			*(ppPt3D++) = pPoint3D;

			m_ResidualHistogram[(int)e]++;
		}

		// histogram compression

		int nH = 0;

		for(i = 0; i < nBins; i++)
			nH += m_ResidualHistogram[i];

		int *CHist = new int[nBins];

		int hs = 12 * nH / 100;

		int h = 0;

		for(i = 0; i < nBins; i++)
		{
			h += m_ResidualHistogram[i];

			if(h >= hs)
				break;
		}

		i++;

		int CBinSize = i;

		int k = 0;

		if(i == nBins)
			CHist[0] = h;
		else
		{
			int hmin = 44 * h / 1000;

			for(; i < nBins; i++)
			{
				if(i % CBinSize == 0)
				{
					CHist[k] = h;

					if(h < hmin)
						break;

					h = 0;

					k++;
				}

				h += m_ResidualHistogram[i];
			}

			if(i == nBins)
				CHist[k] = h;
		}

		int nCBins = k + 1;

		// 

		for(k = 0; k < nCBins; k++)
			Score += (pow((double)CHist[k], 1.3) / (double)(k + 1));

		Score /= (double)CBinSize;

		delete[] CHist;	

		if(Score <= BestScore)
			return Score;

		// determine the scale of the inliers  

		Tol = (double)(nCBins * CBinSize);

		double Se = 0.0;
		double See = 0.0;

		int nCH = 0;

		ppPoint3D = m_Point3DBuff;

		for(i = 0; i < nH; i++, ppPoint3D++)
		{
			pPoint3D = *ppPoint3D;

			e = pPoint3D->e;

			if(e >= Tol)
				continue;

			if(e <= -Tol)
				continue;

			Se += e;

			See += (e * e);

			nCH++;
		}

		double fnCH = (double)nCH;

		m_Tol = 2.5 * sqrt((See - Se * Se / fnCH) / (fnCH - 1.0));

		// determine the consensus set

		ppPoint3D = m_Point3DBuff;

		for(i = 0; i < nH; i++, ppPoint3D++)
		{
			pPoint3D = *ppPoint3D;

			e = pPoint3D->e;

			if(e >= m_Tol)
				continue;

			if(e <= -m_Tol)
				continue;

			*(pConsensusSet++) = pPoint3D;
		}

		nConsensus = pConsensusSet - ConsensusSet;	
	}	// RESC
	else if(Flags & RVLPSDRANSAC_FLAG_WEIGHTED_SCORE)
	{
		for(i = 0; i < n; i++, ppPoint3D++)
		{
			pPoint3D = *ppPoint3D;
			e = pPoint3D->z - (a * pPoint3D->x + b * pPoint3D->y + c);

			if(e > Tol)
				continue;

			if(e >= -Tol)
			{
				Score += m_expLT[(int)(e * e / var)];
				*(pConsensusSet++) = pPoint3D;
				//if(Flags & RVLPSDCONSENSUS_FLAG_ASSIGN)
				//	pPoint3D->p2DRegion = pPlane;
			}
		}

		nConsensus = pConsensusSet - ConsensusSet;	
	}
	else	// basic method
	{
		for(i = 0; i < n; i++, ppPoint3D++)
		{
			pPoint3D = *ppPoint3D;
			e = pPoint3D->z - (a * pPoint3D->x + b * pPoint3D->y + c);

			if(e > Tol)
				continue;

			if(e >= -Tol)
				*(pConsensusSet++) = pPoint3D;
		}

		nConsensus = pConsensusSet - ConsensusSet;	

		Score = (double)nConsensus;
	}

	return Score;
}



void CRVLPlanarSurfaceDetector::CreateDisparityMap(double *X, double *Y, double *Z,
												   int n,
												   RVLDISPARITYMAP *pDisparityMap)
{
	int w = pDisparityMap->Width;
	short int *d = pDisparityMap->Disparity;

	memset(d, 0xff, w * pDisparityMap->Height * sizeof(short int));

	int i;

	for(i = 0; i < n; i++)
		d[(int)X[i] + (int)Y[i] * w] = (short int)(Z[i]);
}


void CRVLPlanarSurfaceDetector::InitilizePointWithDisparity(RVL3DPOINT2 *pPoint3D, 
															int u, int v, int d, int iPix,
															RVL3DPOINT2 **ppP3DMap,
															CRVLMPtrChain *pBorderIn,
															RVLEDT_PIX_ARRAY *peDTImage,
															bool bDelaunay)
{
	pPoint3D->u = u;
	pPoint3D->v = v;
	pPoint3D->d = d;
	pPoint3D->iPix = iPix;
	pPoint3D->x = (double)u;
	pPoint3D->y = (double)v;
	pPoint3D->z = (double)d;
	pPoint3D->segmentNumber = -1;
	pPoint3D->refSegmentNumber = -1;
	pPoint3D->iCell = -1;
	pPoint3D->regionList = NULL;

	*ppP3DMap = pPoint3D;
							
	if(bDelaunay)
		pBorderIn->Add((void *)(peDTImage->pPix + iPix));
}


void CRVLPlanarSurfaceDetector::GetPointsWithDisparity(RVLDISPARITYMAP *pDisparityMap, 
													   CRVLAImage *pAImage, 
													   CRVLMem *pMem,
													   bool bDelaunay)
{
	
	memset(m_Point3DArray, 0, m_Width * m_Height * sizeof(RVL3DPOINT2));
	memset(m_Point3DMap, 0, m_Width * m_Height * sizeof(RVL3DPOINT2 *));

	RVL3DPOINT2 **Point3DMap = m_Point3DMap;
	RVL3DPOINT2 **ppP3DMap;
		
	RVL3DPOINT2 *pPoint3D = m_Point3DArray;

	short int *DispArray = pDisparityMap->Disparity;
	short int *pd;

	int u, v, d,i;
	
	short int minDisparity = m_enableMinDisparity ? m_minDisparity : 0;

	RVLAPIX *pAPix;
	RVLAPIX *APixArray = pAImage->m_pPix;

	int minu, maxu;

	int iPix = 0;

	double currentdIu;
	bool bIsMax = true;
	bool bIsEdge;
	
	int ImageSize = m_Width * m_Height;


	/***************** KINECT ****************************/
	double D3D[3], temp3D[3];
					
	CvMat *dMatTemp3D = cvCreateMatHeader(3, 1, CV_64FC1);
	dMatTemp3D->data.db = temp3D;
	
	double C3D[3];
	CvMat *dMat3DRGBData = cvCreateMatHeader(3, 1, CV_64FC1);
	dMat3DRGBData->data.db = C3D;
	int rgbPoint[2];

	CvMat *dMatRot = cvCreateMatHeader(3, 3, CV_64FC1);
	dMatRot->data.db = m_pStereoVision->m_KinectParams.pPose->m_Rot;
	/*****************       ****************************/

	bool bAddPt = false;

	CRVLEDT  eDT = pAImage->m_EDT;
	RVLEDT_PIX_ARRAY eDTImage = pAImage->m_EDTDisparityImage;

	CRVLMPtrChain BorderIn;

	BorderIn.m_pMem = pMem;


	if(m_Flags & RVLPSD_FLAG_RELIABLE_DISPARITY)
	{
		minu = m_RowLength;
		maxu = m_Width - m_RowLength;
	}
	else
	{
		minu = 0;
		maxu = m_Width;
	}
	
	
	
	for(v = 0; v < m_Height; v++)
	{
		for(u = minu; u < maxu; u++)
		{
			iPix = (v * m_Width) + u;
			pAPix = APixArray + iPix;

			pd = DispArray + iPix;
			ppP3DMap = Point3DMap + iPix;
		
			*ppP3DMap = NULL;

			//if(u==0 && v == 239)
			//	int gg = 0;

			//if disparity exists
			if(*pd >= minDisparity && *pd <= m_maxDisparity)
			{
				
				d = (int)(*pd);

				bAddPt = false;

				/*Get RELIABLE disparity points*/
				//All edge points have advantage
				if(m_Flags & RVLPSD_FLAG_RELIABLE_DISPARITY)
				{
					currentdIu = fabs(pAPix->dIu);
				
					if(currentdIu > 0)
					{
						bIsMax = true;
						bIsEdge = (pAPix->Flags & RVLAPIX_FLAG_EDGE) ? true : false;

						//check left to right
						for(i = -m_RowLength; i <= m_RowLength; i++)
						{
							if(i != 0)  //do not check for current pApix
							{
								if(!bIsEdge)
								{
									//check for edge or max dIu
									if((pAPix[i].Flags & RVLAPIX_FLAG_EDGE)  || (fabs(pAPix[i].dIu) > currentdIu))
									{
										bIsMax = false;
										break;
									}
								}
							}
						}

						if (bIsMax)
						{
							InitilizePointWithDisparity(pPoint3D,u,v,d,iPix,ppP3DMap,&BorderIn,&eDTImage,bDelaunay);
							bAddPt = true;
						}
					
					}
				}
				/*Get ALL disparity points*/
				else
				{
					InitilizePointWithDisparity(pPoint3D,u,v,d,iPix,ppP3DMap,&BorderIn,&eDTImage,bDelaunay);
					bAddPt = true;
				}


				if(bAddPt==true)
				{
					//GET xyz as well as RGB index if KINECT  (if lookup table has been initialized/exists this implies we are using KINECT data )
					if((m_pStereoVision->m_KinectParams.pZProjLT) && (d<2047))
					{
						//z
						D3D[2] = m_pStereoVision->m_KinectParams.pZProjLT[d];
						//x
						D3D[0] = (u - m_pStereoVision->m_KinectParams.depthUc) * (D3D[2]/m_pStereoVision->m_KinectParams.depthFu);
						//y
						D3D[1] = (v - m_pStereoVision->m_KinectParams.depthVc) * (D3D[2]/m_pStereoVision->m_KinectParams.depthFv);

						RVLCOPY3VECTOR(D3D, pPoint3D->XYZ)

						RVLDif3D(D3D,m_pStereoVision->m_KinectParams.pPose->m_X,temp3D);
						cvMatMul(dMatRot,dMatTemp3D,dMat3DRGBData);
						rgbPoint[0] = (int)((C3D[0] * m_pStereoVision->m_KinectParams.rgbFu) / C3D[2] + m_pStereoVision->m_KinectParams.rgbUc);
						rgbPoint[1] = (int)((C3D[1] * m_pStereoVision->m_KinectParams.rgbFv) / C3D[2] + m_pStereoVision->m_KinectParams.rgbVc);
						if ((rgbPoint[0] > 0) && (rgbPoint[1] > 0) && (rgbPoint[0] < m_Width) && (rgbPoint[1] < m_Height))
							pPoint3D->iPixRGB = rgbPoint[1] * m_Width + rgbPoint[0];

					}


					pPoint3D++;
				
				}
			}

			
			
			


		
		}
		
	}
	
	
	m_n3DPoints = pPoint3D - m_Point3DArray;



	/*Delaunay*/
	if(bDelaunay)
	{
		WORD iBucket;

		for(iBucket = 0; iBucket < 4; iBucket++)
			eDT.m_BucketPtrArray[iBucket] = (RVLEDT_BUCKET_ENTRY *)
				(pMem->Alloc(ImageSize * sizeof(RVLEDT_BUCKET_ENTRY)));

		eDT.Border(&eDTImage);
		
		CRVLBuffer EDTBuff;

		EDTBuff.DataBuff = (void **)(pMem->Alloc(ImageSize * sizeof(void *)));
		EDTBuff.m_bOwnData = FALSE;

		eDT.m_maxd2 = m_Width*m_Width  +  m_Height*m_Height;

		eDT.Apply(&BorderIn, NULL, &eDTImage, &EDTBuff);

		m_pDelaunay->Apply(&eDTImage);

		FILE *fp = fopen("C:\\RVL\\ExpRez\\delaunay.dat", "w");

		m_pDelaunay->Save(fp);

		fclose(fp);

		m_pDelaunay->Test();

		pMem->Clear();
	}

}

void CRVLPlanarSurfaceDetector::GetPointsWithDisparity(RVLDISPARITYMAP *pDisparityMap)
{
	short int *pd = pDisparityMap->Disparity;

	RVL3DPOINT2 **ppP3DMap = m_Point3DMap;

	RVL3DPOINT2 *pPoint3D = m_Point3DArray;
	double *X;
	int *iX;

	int iPix = 0;

	short maxDepth = (short)(m_pStereoVision->m_maxz);

	//double k = (m_Flags & RVLPSD_FLAG_100UM ? 0.1 : 1.0);

	int u, v;
	short int d;

	for(v = 0; v < m_Height; v++)
		for(u = 0; u < m_Width; u++, pd++, ppP3DMap++, iPix++)
		{
			X = pPoint3D->XYZ;

			d = *pd;

			if(m_Flags & RVLPSD_FLAG_MM)
			{
				if(d == 0 || d > maxDepth)
				{
					*ppP3DMap = NULL;

					continue;
				}

				X[2] = (double)d;
			}
			else
			{
				if(d == 2047)
				{
					*ppP3DMap = NULL;

					continue;
				}

				X[2] = m_pStereoVision->m_KinectParams.pZProjLT[d];
			}

			pPoint3D->u = u;
			pPoint3D->v = v;
			pPoint3D->d = d;
			pPoint3D->iPix = iPix;
			pPoint3D->segmentNumber = -1;
			pPoint3D->refSegmentNumber = -1;
			pPoint3D->iCell = -1;
			pPoint3D->regionList = NULL;
			
			X[0] = (u - m_pStereoVision->m_KinectParams.depthUc) * (X[2]/m_pStereoVision->m_KinectParams.depthFu);
			X[1] = (v - m_pStereoVision->m_KinectParams.depthVc) * (X[2]/m_pStereoVision->m_KinectParams.depthFv);

			iX = pPoint3D->iX;

			iX[0] = DOUBLE2INT(X[0]);
			iX[1] = DOUBLE2INT(X[1]);
			iX[2] = DOUBLE2INT(X[2]);

			if(m_Flags & RVLPSD_FLAG_MM)
			{
				pPoint3D->x = X[0];
				pPoint3D->y = X[1];
				pPoint3D->z = X[2];
			}
			else
			{
				pPoint3D->x = (double)u;
				pPoint3D->y = (double)v;
				pPoint3D->z = (double)d;
			}

			*ppP3DMap = pPoint3D;

			pPoint3D++;
		}

	m_n3DPoints = pPoint3D - m_Point3DArray;
}

void CRVLPlanarSurfaceDetector::GetReliablePointsWithDisparity(RVLDISPARITYMAP *pDisparityMap, 
															   CRVLAImage *pAImage, 
															   CRVLMem *pMem)
{
	
	memset(m_Point3DArray, 0, m_Width * m_Height * sizeof(RVL3DPOINT2));
	memset(m_Point3DMap, 0, m_Width * m_Height * sizeof(RVL3DPOINT2 *));

	RVL3DPOINT2 **Point3DMap = m_Point3DMap;
	RVL3DPOINT2 **ppP3DMap;
		
	RVL3DPOINT2 *pPoint3D = m_Point3DArray;

	short int *DispArray = pDisparityMap->Disparity;
	short int *pd;


	int u, v, d,i;
	

	short int minDisparity = m_enableMinDisparity ? m_minDisparity : 0;


	RVLAPIX *pAPix;
	RVLAPIX *APixArray = pAImage->m_pPix;

	int minu = m_RowLength;
	int maxu = m_Width - m_RowLength - 1;
	
	int iPix = minu;
	double currentdIu;
	bool bIsMax = true;
	bool bIsEdge;


	//
	int ImageSize = m_Width * m_Height;

	CRVLEDT  eDT = pAImage->m_EDT;
	RVLEDT_PIX_ARRAY eDTImage = pAImage->m_EDTDisparityImage;

	CRVLMPtrChain BorderIn;

	BorderIn.m_pMem = pMem;


	/*******************************/
	/*Get reliable disparity points*/
	/*******************************/

	//Edge points have advantage
	for(v = 0; v < m_Height; v++)
	{
		for(u = minu; u <= maxu; u++)
		{
			iPix = (v * m_Width) + u;
			pAPix = APixArray + iPix;

			pd = DispArray + iPix;
			ppP3DMap = Point3DMap + iPix;
		
			*ppP3DMap = NULL;

			//if disparity exists
			if(*pd >= minDisparity)
			{
				currentdIu = fabs(pAPix->dIu);
			
				if(currentdIu > 0)
				{
					bIsMax = true;
					bIsEdge = (pAPix->Flags & RVLAPIX_FLAG_EDGE) ? true : false;


					//check left to right
					for(i = -m_RowLength; i <= m_RowLength; i++)
					{
						if(i != 0)  //do not check for current pApix
						{
							if(bIsEdge)
							{
								////check for edge
								//if((pAPix[i].Flags & RVLAPIX_FLAG_EDGE) && (fabs(pAPix[i].dIu) > currentdIu))
								//{
								//	bIsMax = false;
								//	break;
								//}
							}
							else
							{
								//check for edge or max dIu
								if((pAPix[i].Flags & RVLAPIX_FLAG_EDGE)  || (fabs(pAPix[i].dIu) > currentdIu))
								{
									bIsMax = false;
									break;
								}
							}
							
						}
					
					}


					if (bIsMax)
					{
						d = (int)(*pd);
				
						pPoint3D->u = u;
						pPoint3D->v = v;
						pPoint3D->d = d;
						pPoint3D->iPix = iPix;
						pPoint3D->x = (double)u;
						pPoint3D->y = (double)v;
						pPoint3D->z = (double)d;
						pPoint3D->segmentNumber = -1;
						pPoint3D->refSegmentNumber = -1;
						pPoint3D->iCell = -1;
						pPoint3D->regionList = NULL;

						*ppP3DMap = pPoint3D;
						
						pPoint3D++;


						BorderIn.Add((void *)(eDTImage.pPix + iPix));
					
					}
				
				}
				
			}
		
		}
		
	}

	m_n3DPoints = pPoint3D - m_Point3DArray;



	/*******************************/
	/**********Delaunay*************/
	/*******************************/
	WORD iBucket;

	for(iBucket = 0; iBucket < 4; iBucket++)
		eDT.m_BucketPtrArray[iBucket] = (RVLEDT_BUCKET_ENTRY *)
			(pMem->Alloc(ImageSize * sizeof(RVLEDT_BUCKET_ENTRY)));

	eDT.Border(&eDTImage);
	
	CRVLBuffer EDTBuff;

	EDTBuff.DataBuff = (void **)(pMem->Alloc(ImageSize * sizeof(void *)));
	EDTBuff.m_bOwnData = FALSE;

	eDT.m_maxd2 = m_Width*m_Width  +  m_Height*m_Height;

	eDT.Apply(&BorderIn, NULL, &eDTImage, &EDTBuff);

	m_pDelaunay->Apply(&eDTImage);

	FILE *fp = fopen("C:\\RVL\\ExpRez\\delaunay.dat", "w");

	m_pDelaunay->Save(fp);

	fclose(fp);

	m_pDelaunay->Test();

	pMem->Clear();

}

int CRVLPlanarSurfaceDetector::GetPointsWithDisparity(RVLRECT *pROI,
													  RVL3DPOINT2 **Point3DPtrArray)
{
	int w = m_Width;
	int maxv = pROI->bottom;
	RVL3DPOINT2 **ppPoint3DSrc = m_Point3DMap + pROI->left + pROI->top * w;
	int wROI = pROI->right - pROI->left + 1;
	int dpNextRow = w - wROI;
	RVL3DPOINT2 **ppPoint3DTgt = Point3DPtrArray;

	int i;
	int v;

	for(v = pROI->top; v < maxv; v++)
	{
		for(i = 0; i < wROI; i++, ppPoint3DSrc++)
			if(*ppPoint3DSrc)
				*(ppPoint3DTgt++) = (*ppPoint3DSrc);

		ppPoint3DSrc += dpNextRow;
	}

	return ppPoint3DTgt - Point3DPtrArray;
}



void CRVLPlanarSurfaceDetector::GetPointsWithDisparity(RVLDISPARITYMAP *pDisparityMap,
													   double *X, double *Y, double *Z,
													   int &nPoints)
{
	int w = pDisparityMap->Width;
	int h = pDisparityMap->Height;

	short int *pd = pDisparityMap->Disparity;

	int i = 0;

	int u, v;

	short int minDisparity = m_enableMinDisparity ? m_minDisparity : 0;

	for(v = 0; v < h; v++)
		for(u = 0; u < w; u++, pd++)
			if(*pd >= minDisparity)
			{
				X[i] = (double)u;
				Y[i] = (double)v;
				Z[i] = (double)(*pd);
				i++;
			}

	nPoints = i;
}

void CRVLPlanarSurfaceDetector::Get3DPlane(CRVL2DRegion2 *p2DRegion, 
										   CRVL3DSurface2 *p3DSurface)
{
	CRVLCamera *pCamera = m_pStereoVision->m_pCameraL;

	double uc = pCamera->CenterXNrm;

	double a = p2DRegion->m_a;
	double b = p2DRegion->m_b;
	double c = p2DRegion->m_c;

	double k1 = (a * uc + b * pCamera->CenterYNrm + c) / pCamera->fNrm;
	double k2 = sqrt(a * a + b * b + k1 * k1);

	p3DSurface->m_N[0] = a / k2;
	p3DSurface->m_N[1] = b / k2;
	p3DSurface->m_N[2] = k1 / k2;

	p3DSurface->m_d = m_pStereoVision->m_BaseLenNrm / k2 * 16.0;
}


void CRVLPlanarSurfaceDetector::GetGroundPoint(int *iU, 
											   CRVL3DSurface2 *pGroundPlane, 
											   double *X)
{
	double V[3];

	CRVLCamera *pCamera = m_pStereoVision->m_pCameraL;

	RVLiU2U(iU, pCamera->CenterXNrm, pCamera->CenterYNrm, V);

	double f = pCamera->fNrm;

	V[2] = f;

	double z = f * pGroundPlane->m_d / RVLDotProduct(pGroundPlane->m_N, V);

	X[0] = z * V[0] / f;
	X[1] = z * V[1] / f;
	X[2] = z;
}



void CRVLPlanarSurfaceDetector::InitStatistics(CRVLMem *pHistMem)
{
	
	m_Histogram[0].Init(10.0,pHistMem,"NoInSeg1");
	m_Histogram[1].Init(10.0,pHistMem,"NoInSeg2");
	m_Histogram[2].Init(1.0,pHistMem,"PercSeg1");
	m_Histogram[3].Init(1.0,pHistMem,"PercSeg2");
	m_Histogram[4].Init(10,pHistMem,"NoInSegs");
	m_Histogram[5].Init(5.0,pHistMem,"PercSegs");

}




void CRVLPlanarSurfaceDetector::CreateStatistics()
{
	// ENABLE THIS IN ORDER TO SAVE HISTOGRAM TO FILE
	char HistFile[] = "C:\\RVL\\ExpRez\\Histogram.dat";
	FILE *fp;
	fp = fopen(HistFile,"w");

	for(int i = 0; i < 6; i++)
	{
		m_Histogram[i].Create();
		m_Histogram[i].Save(fp, "%7.2lf\t");
	}
	
	fclose(fp);

	
}


void CRVLPlanarSurfaceDetector::RANSACStatistics(CRVL2DRegion2 *pPlane,
												CRVLMem *pHistMem)
{
	
	int i,j,k,nBest;

	DWORD Flags;

	double a = pPlane->m_a;
	double b = pPlane->m_b;
	double c = pPlane->m_c;

	m_nRANSACIterations = 10000;
	int maxLoop = 10; //1000

	//Get initial points wih disparity
	
	
	int n3DOriginalPoints = m_n3DPoints;

	RVL3DPOINT2 **Point3DPtrBuff1 = new RVL3DPOINT2 *[m_n3DPoints];
	RVL3DPOINT2 **Point3DPtrBuff2 = new RVL3DPOINT2 *[m_n3DPoints];

	RVL3DPOINT2 **ppPoint3DPtrSrc = Point3DPtrBuff1;
	RVL3DPOINT2 **ppPoint3DPtrTgt = Point3DPtrBuff2;
	RVL3DPOINT2 **tmp;

	RVL3DPOINT2 *pPoint3D;
	RVL3DPOINT2 **ppPoint3DSrcHelper;
	RVL3DPOINT2 **ppPoint3DTgtHelper;
	
	
	//*************Initial Segmentation - get the main planes***********************//
	
	Flags = (RVLPSDRANSAC_FLAG_LS | RVLPSDRANSAC_FLAG_WEIGHTED_SCORE | RVLPSDRANSAC_FLAG_PT_DISTRIBUTION);
	m_maxSegmentTime = 2;
	
	
	pPoint3D =	m_Point3DArray;
	ppPoint3DSrcHelper = ppPoint3DPtrSrc;	

	//copy initial source
	for(i=0; i<m_n3DPoints; i++, ppPoint3DSrcHelper++)
	{
		*ppPoint3DSrcHelper = pPoint3D;
		pPoint3D++;
	}

	FILE *fMain;
	char MainSegmentFile[] = "C:\\RVL\\ExpRez\\MainSegments.dat";
	fMain = fopen(MainSegmentFile,"w");

	fprintf(fMain, "SegmentNo\ta\tb\tc\tNoPtsInSeg\tNoInitPoints");

	
	RVLPSDLAD_SEGMENT_DETAILS mainSegments[RVLPSDRANSAC_NUMBER_OF_MAIN_SEGMENTS];

	
	for(i=0; i<RVLPSDRANSAC_NUMBER_OF_MAIN_SEGMENTS; i++) 
	{
		//Initialize values
		pPlane->m_a = a;
		pPlane->m_b = b;
		pPlane->m_c = c;
		
		mainSegments[i].segmentNumber = i;

		nBest = RANSAC(pPlane,true, ppPoint3DPtrSrc, m_n3DPoints, &mainSegments[i],Flags);

		fprintf(fMain,"\n%2d\t%11.6lf\t%11.6lf\t%11.6lf\t%8d\t%8d",
		mainSegments[i].segmentNumber,
		mainSegments[i].a,
		mainSegments[i].b,
		mainSegments[i].c,
		mainSegments[i].numberOfPoints,
		m_n3DPoints);

		
		ppPoint3DSrcHelper = ppPoint3DPtrSrc;
		ppPoint3DTgtHelper = ppPoint3DPtrTgt;
		
		
		for(k=0; k<m_n3DPoints; k++, ppPoint3DSrcHelper++)
		{
			pPoint3D = *ppPoint3DSrcHelper;
			if(pPoint3D->segmentNumber == -1)
			{
				*ppPoint3DTgtHelper = pPoint3D;
				*ppPoint3DTgtHelper++;
			}
		
		}


		m_n3DPoints = ppPoint3DTgtHelper - ppPoint3DPtrTgt;;

		tmp = ppPoint3DPtrTgt;
		ppPoint3DPtrTgt = ppPoint3DPtrSrc;
		ppPoint3DPtrSrc = tmp;
			
	
	}

	fclose(fMain);



	//Initial segmentation done
	//transfer segmentNumber to RefSegmentNumber and reset Point 3D segmentNumber, reset source
	m_n3DPoints = n3DOriginalPoints;
	pPoint3D =	m_Point3DArray;
	ppPoint3DSrcHelper = ppPoint3DPtrSrc;	
	for (i=0; i<n3DOriginalPoints; i++, ppPoint3DSrcHelper++)
	{
		pPoint3D->refSegmentNumber = pPoint3D->segmentNumber;
		pPoint3D->segmentNumber = -1;

		*ppPoint3DSrcHelper = pPoint3D;
		pPoint3D++;
	}



	//**************************Analysis*************************//

	//Flags = (Flags | RVLPSDRANSAC_FLAG_3PTSINIT);  //RVLPSDRANSAC_FLAG_3PTSINIT
	m_maxSegmentTime = 0.03;

	//double dmaxTime = 0.02;
	//double dminTime = 0.01;
	//double dMaxSegmentTimeConstant = (dmaxTime - dminTime) / n3DOriginalPoints;

	RVLPSDLAD_SEGMENT_DETAILS allSegments[RVLPSDRANSAC_MAX_NUMBER_OF_SEGMENTS];

	int minPtsRemaining = (int)(0.2 * n3DOriginalPoints);

	  
	int segmentNo = 0;
	int nPtsSeg1 = 0;
	int nPtsSeg2 = 0;
	
	double fTmp;
	int maxnOfPts = mainSegments[0].numberOfPoints + mainSegments[1].numberOfPoints;
	

	int segmentMatrix[RVLPSDRANSAC_MAX_NUMBER_OF_SEGMENTS][RVLPSDRANSAC_NUMBER_OF_MAIN_SEGMENTS + 1];


	FILE *fSummary;
	FILE *fDetails;
	FILE *fMatrix;
	

	char AllSegmentsSummary[] = "C:\\RVL\\ExpRez\\AllSegmentsSummary.dat";
	char AllSegmentsDetails[] = "C:\\RVL\\ExpRez\\AllSegmentsDetails.dat";
	char AllSegmentsMatrix[] = "C:\\RVL\\ExpRez\\AllSegmentsMatrix.dat";
	

	
	//Start Statistics
	InitStatistics(pHistMem);

	fSummary = fopen(AllSegmentsSummary,"w");
	fDetails = fopen(AllSegmentsDetails,"w");
	fMatrix = fopen(AllSegmentsMatrix,"w");

	fprintf(fSummary, "maxLoop\tNumberInSeg1\tNumberInSeg2\tNIterationsSeg1\tNIterationsSeg2");
	fprintf(fDetails, "SegmentNo\ta\tb\tc\tNoPtsInSeg\tNoInitPoints\tNIterations");
	fprintf(fMatrix, "SegmentNo\tNumberInSeg1\tNumberInSeg2\tNumberWithNoRef");


	for(i=0; i<maxLoop; i++)
	{

		//Initialize values

		pPlane->m_a = a;
		pPlane->m_b = b;
		pPlane->m_c = c;
		
		
		for(segmentNo = 0; segmentNo < RVLPSDRANSAC_MAX_NUMBER_OF_SEGMENTS; segmentNo++)
		{
			allSegments[segmentNo].segmentNumber = segmentNo;

			// set time limit proportionally to the total number of points
			//m_maxSegmentTime = dminTime + (dMaxSegmentTimeConstant * m_n3DPoints);

			nBest = RANSAC(pPlane,true, ppPoint3DPtrSrc, m_n3DPoints, &allSegments[segmentNo],Flags);

			fprintf(fDetails,"\n%2d\t%11.6lf\t%11.6lf\t%11.6lf\t%8d\t%8d\t%8d",
			allSegments[segmentNo].segmentNumber,
			allSegments[segmentNo].a,
			allSegments[segmentNo].b,
			allSegments[segmentNo].c,
			allSegments[segmentNo].numberOfPoints,
			m_n3DPoints,
			allSegments[segmentNo].numberOfIterations);

			
			ppPoint3DSrcHelper = ppPoint3DPtrSrc;
			ppPoint3DTgtHelper = ppPoint3DPtrTgt;
			
			
			for(k=0; k<m_n3DPoints; k++, ppPoint3DSrcHelper++)
			{
				pPoint3D = *ppPoint3DSrcHelper;
				if(pPoint3D->segmentNumber == -1)
				{
					*ppPoint3DTgtHelper = pPoint3D;
					*ppPoint3DTgtHelper++;
				}
			
			}
			

			m_n3DPoints = ppPoint3DTgtHelper - ppPoint3DPtrTgt;;

			tmp = ppPoint3DPtrTgt;
			ppPoint3DPtrTgt = ppPoint3DPtrSrc;
			ppPoint3DPtrSrc = tmp;
			

			//if the number of points left for segmentation 
			//is less than the min defined, exit loop
			if(m_n3DPoints < minPtsRemaining)
				break;

	
		} //for loop


		//for some strange reason it is possible that segmentNo == RVLPSDRANSAC_MAX_NUMBER_OF_SEGMENTS
		if(segmentNo == RVLPSDRANSAC_MAX_NUMBER_OF_SEGMENTS)
		{
			segmentNo -=1;
		}

		//fill segmentMatrix, reset point3D segmentNumber, reset source 
		int cnt=0, _cnt=0;
	
		//reset matrix
		memset(&segmentMatrix,0,RVLPSDRANSAC_MAX_NUMBER_OF_SEGMENTS * (RVLPSDRANSAC_NUMBER_OF_MAIN_SEGMENTS + 1) * sizeof(int));


		m_n3DPoints = n3DOriginalPoints;
		ppPoint3DSrcHelper = ppPoint3DPtrSrc;	

		pPoint3D =	m_Point3DArray;
		for (j=0; j<n3DOriginalPoints; j++, ppPoint3DSrcHelper++)
		{
			
			//if(pPoint3D->segmentNumber > -1)
			//{
				if(pPoint3D->segmentNumber != -1 && pPoint3D->refSegmentNumber != -1)
				{
					segmentMatrix[pPoint3D->segmentNumber][pPoint3D->refSegmentNumber]++;
					cnt++;
				}
				else if (pPoint3D->segmentNumber != -1 && pPoint3D->refSegmentNumber == -1)
				{
					segmentMatrix[pPoint3D->segmentNumber][RVLPSDRANSAC_NUMBER_OF_MAIN_SEGMENTS]++;
					_cnt++;
	
				}

				
			//}
			
			pPoint3D->segmentNumber = -1;
			
			*ppPoint3DSrcHelper = pPoint3D;
			pPoint3D++;
		}


		
		//Write matrix
		//fprintf(fMatrix,"\nmaxLoop=%4d",maxLoop);


		//get max number of points from matrix for each main segment
		nPtsSeg1=0;
		nPtsSeg2=0;
		int nIterationsSeg1 = 0;
		int nIterationsSeg2 = 0;
		int val;
		
		for(int row=0; row<=segmentNo; row++)
		{
			for(int col=0; col<RVLPSDRANSAC_NUMBER_OF_MAIN_SEGMENTS; col++)
			{
				val = (int)segmentMatrix[row][col];
				if(col==0)
				{
					if(val>nPtsSeg1)
					{
						nPtsSeg1=val;
						nIterationsSeg1 = allSegments[row].numberOfIterations;
					}
				}
				if(col==1)
				{
					if(val>nPtsSeg2)
					{
						nPtsSeg2=val;
						nIterationsSeg2 = allSegments[row].numberOfIterations;
					}
				}

				
				
			}

			//Write matrix
			fprintf(fMatrix,"\n%4d\t%8d\t%8d\t%8d",
							row,
							(int)segmentMatrix[row][0],
							(int)segmentMatrix[row][1],
							(int)segmentMatrix[row][2]);
		}


		


		//Write segments summary
		fprintf(fSummary,"\n%4d\t%8d\t%8d\t%8d\t%8d",
		i,
		nPtsSeg1,
		nPtsSeg2,
		nIterationsSeg1,
		nIterationsSeg2);




		// Add to histogram	
		//NoInSeg1
		fTmp = (double)nPtsSeg1;
		m_Histogram[0].m_Data.Add(&fTmp); 
		//NoInSeg2
		fTmp = (double)nPtsSeg2;
		m_Histogram[1].m_Data.Add(&fTmp); 
		//PercSeg1
		fTmp = nPtsSeg1 * 100/(double)mainSegments[0].numberOfPoints;
		m_Histogram[2].m_Data.Add(&fTmp); 
		//PercSeg2
		fTmp = nPtsSeg2 * 100/(double)mainSegments[1].numberOfPoints;
		m_Histogram[3].m_Data.Add(&fTmp); 
		//NoInSegs
		fTmp = (double)(nPtsSeg1 + nPtsSeg2);
		m_Histogram[4].m_Data.Add(&fTmp); 
		//PercSegs
		fTmp = (nPtsSeg1 + nPtsSeg2) * 100 / (double)maxnOfPts;
		m_Histogram[5].m_Data.Add(&fTmp); 



		


	}// main maxLoop loop

	fclose(fDetails);
	fclose(fSummary);
	fclose(fMatrix);

	CreateStatistics();


	
	delete[] Point3DPtrBuff1;
	delete[] Point3DPtrBuff2;
		
}

int CRVLPlanarSurfaceDetector::GetSurface(int uIn, int vIn,
										   int *PtBuff,
										   DWORD Flags)
{
	unsigned int *RegionGrowingMap = (unsigned int *)m_RegionGrowingMap;

	int u = (uIn >> 1);
	int v = (vIn >> 1);

	RVLRECT Win;

	int halfWinSize = (m_WinSize - 1) / 2;

	Win.top = v - halfWinSize;
	
	if(Win.top < 0)
		return 0;

	Win.bottom = v + halfWinSize;

	if(Win.bottom >= m_Height)
		return 0;

	Win.left = u - halfWinSize;

	if(Win.left < 0)
		return 0;

	Win.right = u + halfWinSize;

	if(Win.right >= m_Width)
		return 0;

	RVL3DPOINT2 **InitSet = new RVL3DPOINT2 *[m_WinSize * m_WinSize];

	int nInit = GetPointsWithDisparity(&Win, InitSet);
	
	if(nInit < m_minnLADRANSACInit3DPoints)
	{
		delete[] InitSet;

		return 0;
	}


	if(Flags & RVLPSDRANSAC_FLAG_PT_DISTRIBUTION)
	{
		double k1 = 2.0 * m_kLADRANSACInit3DPoints * m_kLADRANSACInit3DPoints;

		int Su, Sv, Suu, Suv, Svv;
		double avgu, avgv, Suunrm, Suvnrm, Svvnrm;
		double k2, k3;

		Suu = 0;
		Suv = 0;
		Svv = 0;
		Su = 0;
		Sv = 0;

		int i;
		RVL3DPOINT2 *pPoint3D;

		for(i = 0; i < nInit; i++)
		{
			pPoint3D = InitSet[i];

			u = pPoint3D->u;
			v = pPoint3D->v;
			Su += u;
			Sv += v;
			Suu += u * u;
			Suv += u * v;
			Svv += v * v;
		}

		double fnInit = (double)nInit;

		avgu = (double)Su / fnInit;
		avgv = (double)Sv / fnInit;

		Suunrm = (double)Suu / fnInit - avgu * avgu;
		Suvnrm = (double)Suv / fnInit - avgu * avgv;
		Svvnrm = (double)Svv / fnInit - avgv * avgv;

		k2 = Suunrm + Svvnrm - k1;

		if(k2 < 0.0)
		{
			delete[] InitSet;

			return 0;
		}

		k3 = Suunrm - Svvnrm;

		if(k3 * k3 + 4.0 * Suvnrm * Suvnrm > k2 * k2)
		{
			delete[] InitSet;

			return 0;
		}
	}

	CRVL2DRegion2 Plane;

	LADPlaneThreePoints(InitSet, nInit, &Plane);

	delete[] InitSet;

	short int *d = m_pStereoVision->m_DisparityMap.Disparity;

	int iPt1, iPt2;
	//int iPt3;

	int ImageSize = m_Width * m_Height;

	InitSet = new RVL3DPOINT2 *[ImageSize];

	int u0 = (uIn >> 1);
	int v0 = (vIn >> 1);

	int iPix0 = u0 + v0 * m_Width;

	short d0 = d[iPix0];

	double a, b, c;

	a = Plane.m_a;
	b = Plane.m_b;
	c = Plane.m_c;

	short Tol = (short)DOUBLE2INT(m_Tol);



	RVL3DPOINT2 **p3DPt;
	int iPix2, iPix1;
	int i, j;
	int u1, v1;
	short d1, d2;
	unsigned int dist2;
	short int dPlane;

	for(i = 0; i < 5; i++)
	{
		iPt1 = 0;

		p3DPt = InitSet;

		dPlane = (short int)DOUBLE2INT(a * (double)u0 + b * (double)v0 + c);

		if(d0 >= 0)
		{
			if(abs(dPlane - d0) <= Tol)
			{
				RegionGrowingMap[iPix0] = 0;

				PtBuff[iPt1++] = iPix0;

				*(p3DPt++) = m_Point3DMap[iPix0];
			}
		}

		// region growing

		iPt2 = 0;

		while(iPt2 < iPt1)
		{
			iPix2 = PtBuff[iPt2];
			d2 = d[iPix2];
			dist2 = RegionGrowingMap[iPix2];

			for(j = 0; j < 4; j++)
			{
				iPix1 = iPix2 + m_dpNeighbor4[j];

				if(RegionGrowingMap[iPix1] <= dist2)
					continue;

				d1 = d[iPix1];

				if(d1 >= 0)
				{
					u1 = iPix1 % m_Width;
					v1 = iPix1 / m_Width;
	
					dPlane = (short int)DOUBLE2INT(a * (double)u1 + b * (double)v1 + c);

					if(abs(dPlane - d1) <= Tol)
					{
						RegionGrowingMap[iPix1] = 0;

						PtBuff[iPt1++] = iPix1;

						*(p3DPt++) = m_Point3DMap[iPix1];
					}
				}
				//else if(dist2 < m_maxdist)
				//{
				//	RegionGrowingMap[iPix1] = dist2 + 1;

				//	PtBuff[iPt1++] = iPix1;
				//}
			}
			
			iPt2++;
		}	// region growing loop

		ClearRegionGrowingMap();

		nInit = p3DPt - InitSet;

		if(nInit < m_minnLADRANSACInit3DPoints)
			break;

		LSPlane(InitSet, nInit, &Plane
			//, RVLPSDLSPLANE_FLAG_M_ESTIMATOR
			);
		//LADPlaneThreePoints(InitSet, nInit, &Plane);

		a = Plane.m_a;
		b = Plane.m_b;
		c = Plane.m_c;
	}	// for(i = 0; i < 5; i++)

	delete[] InitSet;

	return iPt1;
}

BOOL CRVLPlanarSurfaceDetector::Consistent(CRVL2DRegion2 *pPlane, 
										   RVLSTEREOPOINT *pPt)
{
	int u = pPt->u;
	int v = pPt->v;
	int d = pPt->disparity;

	short int dPlane = (short int)DOUBLE2INT(pPlane->m_a * (double)u + 
		pPlane->m_b * (double)v + pPlane->m_c);

	if(dPlane < 0)
		return FALSE;

	int iPix = u + v * m_Width; 

	if(d >= 0)	
		return abs(dPlane - d) <= (int)m_Tol;
	else
		return FALSE;
	/*
	{
		dPlane += m_pStereoVision->m_DisparityOffsetNrm;

		int u2 = u - ((dPlane + 8) >> 4);

		if(u2 < 0)
			return FALSE;

		int I  = (int)(m_pStereoVision->m_pCameraL->m_nrmImage.pPix[iPix]);
		int I2 = (int)(m_pStereoVision->m_pCameraR->m_nrmImage.pPix[u2 + v * m_Width]);

		return abs(I - I2) <= m_ITol;
	}
	*/
}

void CRVLPlanarSurfaceDetector::Segment3DDisplay(	CRVLGUI *pGUI,
													CRVLFigure *pFig,
													int iFirstTextLine,
													CRVLMPtrChain *p2DRegionList,
													PIX_ARRAY *pIIn,
													RVLAPIX *pSelectedAPix,
													PIX_ARRAY *pIOut,
													DWORD Flags)
{
	unsigned char *pPixIn = pIIn->pPix;

	int w = pIIn->Width;
	int h = pIIn->Height;

	unsigned char *pPixOut = pIOut->pPix;

	short int *d = m_pStereoVision->m_DisparityMap.Disparity;


	int u, v;
	unsigned char I;
	int iPix;

	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pPixIn++,pPixOut += 3)
		{
			iPix = u + v * m_Width;

			I = (*pPixIn);

			pPixOut[0] = I;
			pPixOut[1] = I;
			pPixOut[2] = I;


			if(Flags & RVLPSDDISPLAY_DISPARITY_OVER_IMAGE)
			{
				if(d[iPix] >= 0)
				{
				
					pPixOut[0] = 0;
					pPixOut[1] = 64 + (unsigned char)((((int)I) * 3) >> 2);
					pPixOut[2] = 0;

				}
				
			}
		}
	}

	// display segmented image

	CRVL2DRegion2 *pSegment;

	int nSegments = p2DRegionList->m_nElements;

	CRVL2DRegion2 **SortedSurfaceArray = new CRVL2DRegion2 *[nSegments];

	CRVL2DRegion2 **ppSegment = SortedSurfaceArray;

	if(m_pGround)
		*(ppSegment++) = m_pGround;

	int i;

	for(i = 0; i < m_nWalls; i++)
		*(ppSegment++) = m_WallArray[i];

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if((pSegment->m_Flags & (RVL2DREGION_FLAG_GROUND | RVL2DREGION_FLAG_WALL)) == 0)
			*(ppSegment++) = pSegment;
	}

	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;

	if(Flags & RVLPSDDISPLAY_SEGMENTATION)
	{
		double log2 = log(2.0);

		double h;
		int s;
		RVLCOLOR Color;

		p2DRegionList->Start();

		for(i = 1; i <= nSegments; i++)
		{
			s = (1 << (int)floor(log((double)i)/log2));

			h = PI * ((double)(2 * (i - s) + 1) / (double)s - 1.0);

			pSegment = SortedSurfaceArray[i - 1];

			ppPt = (RVL3DPOINT2 **)pSegment->m_PtArray;

			pPtArrayEnd = ppPt + pSegment->m_nPts;

			for(; ppPt < pPtArrayEnd; ppPt++)
			{
				pPt = *ppPt;

				pPixOut = pIOut->pPix + 3 * pPt->iPix;

				I = pIIn->pPix[pPt->iPix];

				Color = RVLHSL2RGB(h, 1.0, (double)I / 255.0);

				*(pPixOut++) = Color.r;
				*(pPixOut++) = Color.g;
				*pPixOut     = Color.b;
			}
		}
	}

	delete[] SortedSurfaceArray;

	// selection

	if(Flags & RVLPSDDISPLAY_GET_SURFACE)
	{
		if(pSelectedAPix == NULL)
			return;

		int *PtBuff = new int[4 * m_Width * m_Height];

		double StartTime = m_pTimer->GetTime();

		m_nSupport = GetSurface(pSelectedAPix->u, pSelectedAPix->v, PtBuff);

		double ExecTime = m_pTimer->GetTime() - StartTime;

		int i;

		for(i = 0; i < m_nSupport; i++)
		{
			pPixOut = pIOut->pPix + 3 * PtBuff[i];

			I = pIIn->pPix[PtBuff[i]];

			*(pPixOut++) = 64 + (unsigned char)((((int)I) * 3) >> 2);
			*(pPixOut++) = 0;
			*pPixOut = 0;
		}

		delete[] PtBuff;

		sprintf(pGUI->m_Text[iFirstTextLine], "Plane Support=%d", m_nSupport);

		if(pGUI->m_nTextLines < iFirstTextLine + 1)
			pGUI->m_nTextLines = iFirstTextLine + 1;
	}
	else
	{
		double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
			(m_pStereoVision->m_pCameraL->CenterXNrm - 
			m_pStereoVision->m_pCameraR->CenterXNrm);

		if(pSelectedAPix == NULL)
			return;
		
		//Karlo_TG
		/*RVLPSDDisplaySegments(&(m_pStereoVision->m_DisparityMap),
							  pIIn,
							  pIOut,
							  m_pGround,
							  m_WallArray,
							  RVLPSDDISPLAY_TWOSEGMENTS,
							  false);
		*/

		int uSelected = (pSelectedAPix->u >> 1);
		int vSelected = (pSelectedAPix->v >> 1);

		int mind, maxd;

		BOOL bSegment = FALSE;
	
		p2DRegionList->Start();

		while(p2DRegionList->m_pNext)
		{
			pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

			mind = 100000;
			maxd = 0;

			ppPt = (RVL3DPOINT2 **)pSegment->m_PtArray;

			pPtArrayEnd = ppPt + pSegment->m_nPts;

			for(; ppPt < pPtArrayEnd; ppPt++)
			{
				pPt = *ppPt;

				if(pPt->u == uSelected && pPt->v == vSelected)
					bSegment = TRUE;

				if(pPt->d > maxd)
					maxd = pPt->d;

				if(pPt->d < mind)
					mind = pPt->d;
			}

			if(bSegment)
				break;
		}	
		
		if(!bSegment)
			return;

		sprintf(pGUI->m_Text[iFirstTextLine], "Plane Support=%d tol=%3.0lf", 
			pSegment->m_nPts,
			pSegment->m_Tol);

		if(pGUI->m_nTextLines < 6)
			pGUI->m_nTextLines = 6;		

		if(pSegment->m_Flags & RVL2DREGION_FLAG_GROUND)
		{
			sprintf(pGUI->m_Text[iFirstTextLine + 1], "");

			return;
		}

		if(m_pGround == NULL)
		{
			sprintf(pGUI->m_Text[iFirstTextLine + 1], "");

			return;
		}

		sprintf(pGUI->m_Text[iFirstTextLine + 1], "Inclination=%4.0lf[deg]", 
			acos(RVLDotProduct(((CRVL3DSurface2 *)(pSegment->m_vp3DSurface))->m_N,
			((CRVL3DSurface2 *)(m_pGround->m_vp3DSurface))->m_N)) * RAD2DEG);

		double a1 = m_pGround->m_a;
		double b1 = m_pGround->m_b;
		double c1 = m_pGround->m_c;

		double a2 = pSegment->m_a;
		double b2 = pSegment->m_b;
		double c2 = pSegment->m_c;

		double fmaxd = (double)maxd / 16.0 + d0;
		double fmind = (double)mind / 16.0 + d0;

		int u1 = (DOUBLE2INT((b2*fmind-c1*b2-b1*fmind+b1*c2)/(a1*b2-b1*a2)) << 1) + 1;
		int v1 = (DOUBLE2INT(-(a2*fmind-c1*a2-a1*fmind+a1*c2)/(a1*b2-b1*a2)) << 1) + 1;

		int u2 = (DOUBLE2INT((b2*fmaxd-c1*b2-b1*fmaxd+b1*c2)/(a1*b2-b1*a2)) << 1) + 1;
		int v2 = (DOUBLE2INT(-(a2*fmaxd-c1*a2-a1*fmaxd+a1*c2)/(a1*b2-b1*a2)) << 1) + 1;
			
		CRVLDisplayVector Vector(pFig->m_pMem);
		
		Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
		
		Vector.m_rL = 255;
		Vector.m_gL = 255;
		Vector.m_bL = 0;

		Vector.m_LineWidth = 2;

		CRVLDisplayVector *pVector = pFig->AddVector(&Vector);

		pVector->Line(u1, v1, u2, v2);


		//Draw line from reference file if exists
		//open uv file and get the first 2 points
		FILE *uvFile = fopen(pGUI->m_InputImageUVFileName, "r");
		
		if(uvFile != NULL)
		{
			int n = 0;
			int u10,u20,v10,v20 = 0;
			char chUV[10];
			
			//get first 4 numbers
			while(!feof(uvFile))
			{
				fgets(chUV,10,uvFile);
				n++;
				
				if(n==1)
					u10 = atoi(chUV);
				else if(n==2)
					v10 = atoi(chUV);
				else if(n==3)
					u20 = atoi(chUV);
				else if(n==4)
					v20 = atoi(chUV);
				else
					break;

			}

			fclose(uvFile);

			CRVLDisplayVector VectorR(pFig->m_pMem);

			VectorR.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
 
			VectorR.m_rL = 255;
			VectorR.m_gL = 0;
			VectorR.m_bL = 255;
			
			VectorR.m_rL = 255;
			VectorR.m_gL = 255;
			VectorR.m_bL = 255;

			VectorR.m_LineWidth = 2;

			pVector = pFig->AddVector(&VectorR);

			pVector->Line(u10*2, v10*2, u20*2, v20*2);

			//Karlo_TG
			//RVLPSDDrawLine(u1/2,v1/2,u2/2,v2/2,pIOut,false, u10,u20);
            //RVLPSDDrawLine(u10,v10,u20,v20,pIOut,true,u10,u20);


			GetMinAngles(pGUI->m_InputImageUVFileName);

			sprintf(pGUI->m_Text[iFirstTextLine + 2], "Ref. line coordinates (%d,%d) (%d,%d)", u10,v10,u20,v20);
			sprintf(pGUI->m_Text[iFirstTextLine + 3], "AngleBtwnLines=%11.6lf[deg]", m_minAngleBtwnLines * RAD2DEG);
			sprintf(pGUI->m_Text[iFirstTextLine + 4], "AngleBtwnLinesIn3D=%11.6lf[deg]", m_minAngleBtwnProjectedLines * RAD2DEG);

		}

	}	// selection: if((Flags & RVLPSDDISPLAY_GET_SURFACE) == 0)
}







void CRVLPlanarSurfaceDetector::SegmentRBDisplay(	CRVLGUI *pGUI,
													CRVLFigure *pFig,
													int iFirstTextLine,
													CRVLAImage *pAImage,
													PIX_ARRAY *pIIn,
													RVLAPIX *pSelectedAPix,
													PIX_ARRAY *pIOut,
													DWORD Flags)
{
	unsigned char *pPixIn = pIIn->pPix;

	int w = pIIn->Width;
	int h = pIIn->Height;

	unsigned char *pPixOut = pIOut->pPix;

	short int *d = m_pStereoVision->m_DisparityMap.Disparity;

	CRVLMPtrChain *p2DRegionList;

	bool bSuperSegments = (pAImage->m_C2DRegion3.m_ObjectList.m_nElements>0);
	CRVLC2D *p2DRegionSet2, *p2DRegionSet;

	//if there are super segments display them else standard segments
	if(bSuperSegments)
	{
		p2DRegionList = &(pAImage->m_C2DRegion3.m_ObjectList);
		p2DRegionSet2 = &(pAImage->m_C2DRegion3);
	}
	else
	{
		p2DRegionList = &(pAImage->m_C2DRegion.m_ObjectList);
	}
	
	p2DRegionSet = &(pAImage->m_C2DRegion);


	int u, v;
	unsigned char I;
	int iPix;

	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pPixIn++,pPixOut += 3)
		{
			iPix = u + v * m_Width;

			I = (*pPixIn);

			pPixOut[0] = I;
			pPixOut[1] = I;
			pPixOut[2] = I;


			if(Flags & RVLPSDDISPLAY_DISPARITY_OVER_IMAGE)
			{
				if(d[iPix] >= 0)
				{
				
					pPixOut[0] = 0;
					pPixOut[1] = 64 + (unsigned char)((((int)I) * 3) >> 2);
					pPixOut[2] = 0;

				}
				
			}
		}
	}

	// display segmented image

	CRVL2DRegion2 *pSegment;

	int nSegments = p2DRegionList->m_nElements;

	CRVL2DRegion2 **SortedSurfaceArray = new CRVL2DRegion2 *[nSegments];

	CRVL2DRegion2 **ppSegment = SortedSurfaceArray;

	if(m_pGround)
		*(ppSegment++) = m_pGround;

	int i,j;

	for(i = 0; i < m_nWalls; i++)
		*(ppSegment++) = m_WallArray[i];

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if((pSegment->m_Flags & (RVL2DREGION_FLAG_GROUND | RVL2DREGION_FLAG_WALL)) == 0)
			*(ppSegment++) = pSegment;
	}

	int *ppPt, *pPtArrayEnd;
	int index;


	CRVL2DRegion2 **ppSubRegionStart, **ppSubRegionEnd, *p2DSubRegion;
	RVLARRAY *pRelList;
	int nSubSegments;
	

	
	

	if(Flags & RVLPSDDISPLAY_SEGMENTATION)
	{
		double log2 = log(2.0);

		double h;
		int s;
		RVLCOLOR Color;

		p2DRegionList->Start();

		for(i = 1; i <= nSegments; i++)
		{
			s = (1 << (int)floor(log((double)i)/log2));

			h = PI * ((double)(2 * (i - s) + 1) / (double)s - 1.0);

			pSegment = SortedSurfaceArray[i - 1];

			if(bSuperSegments)
			{
				pRelList = pSegment->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_COMPONENTS];
				ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
				ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
				nSubSegments = ppSubRegionEnd - ppSubRegionStart;

				for(j=0;j<nSubSegments;j++,ppSubRegionStart++)
				{
					p2DSubRegion = *ppSubRegionStart;
					
					ppPt = (int *)p2DSubRegion->m_PtArray;

					pPtArrayEnd = ppPt + p2DSubRegion->m_nPts;

					for(; ppPt < pPtArrayEnd; ppPt++)
					{
						index = *ppPt;

						pPixOut = pIOut->pPix + 3 * index;

						I = pAImage->m_pPix[index].I;

						Color = RVLHSL2RGB(h, 1.0, (double)I / 255.0);

						*(pPixOut++) = Color.r;
						*(pPixOut++) = Color.g;
						*pPixOut     = Color.b;
					}
				}
			
			}//if (bSuperSegments)
			else
			{

				ppPt = (int *)pSegment->m_PtArray;

				pPtArrayEnd = ppPt + pSegment->m_nPts;

				for(; ppPt < pPtArrayEnd; ppPt++)
				{
					index = *ppPt;

					pPixOut = pIOut->pPix + 3 * index;

					I = pAImage->m_pPix[index].I;

					Color = RVLHSL2RGB(h, 1.0, (double)I / 255.0);

					*(pPixOut++) = Color.r;
					*(pPixOut++) = Color.g;
					*pPixOut     = Color.b;
				}
			}//else (!bSuperSegments) 

		}
	}

	delete[] SortedSurfaceArray;

	

	// display selection
	if(pSelectedAPix == NULL)
			return;


	//CRVLRelation *pRelation;
	CRVLMPtrChain2 RelList;
	//CRVL2DRegion2 *pRegion2;
	CRVL2DRegion2 *p2DSuperSegment;

	iPix = pSelectedAPix - pAImage->m_pPix;

	//get segment region from selected pixel
	pSegment = pAImage->m_2DRegionMap[iPix];

	if(pSegment)
	{
		if(bSuperSegments)
		{
			p2DSuperSegment = *((CRVL2DRegion2 **)(pSegment->m_pData + p2DRegionSet->m_iDataParentPtr));

			if(p2DSuperSegment)
			{
				pRelList = p2DSuperSegment->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_COMPONENTS];
				ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
				ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
				nSubSegments = ppSubRegionEnd - ppSubRegionStart;

				for(j=0;j<nSubSegments;j++,ppSubRegionStart++)
				{
					p2DSubRegion = *ppSubRegionStart;
					
					ppPt = (int *)p2DSubRegion->m_PtArray;

					pPtArrayEnd = ppPt + p2DSubRegion->m_nPts;

					for(; ppPt < pPtArrayEnd; ppPt++)
					{
						index = *ppPt;

						pPixOut = pIOut->pPix + 3 * index;
						*(pPixOut++) = 0; 
						*(pPixOut++) = 0; 
						*(pPixOut++) = 0; 
					}
				}

			}//if(p2DSuperSegment)
			else
			{
				ppPt = (int *)pSegment->m_PtArray;

				pPtArrayEnd = ppPt + pSegment->m_nPts;

				for(; ppPt < pPtArrayEnd; ppPt++)
				{
					index = *ppPt;

					pPixOut = pIOut->pPix + 3 * index;
					*(pPixOut++) = 0; 
					*(pPixOut++) = 0; 
					*(pPixOut++) = 0; 
				}
			}
		}
		else
		{
			ppPt = (int *)pSegment->m_PtArray;

			pPtArrayEnd = ppPt + pSegment->m_nPts;

			for(; ppPt < pPtArrayEnd; ppPt++)
			{
				index = *ppPt;

				pPixOut = pIOut->pPix + 3 * index;
				*(pPixOut++) = 0; 
				*(pPixOut++) = 0; 
				*(pPixOut++) = 0; 
			}
		}

	}
	




	/*
	// selection

	if(Flags & RVLPSDDISPLAY_GET_SURFACE)
	{
		if(pSelectedAPix == NULL)
			return;

		int *PtBuff = new int[4 * m_Width * m_Height];

		double StartTime = m_pTimer->GetTime();

		m_nSupport = GetSurface(pSelectedAPix->u, pSelectedAPix->v, PtBuff);

		double ExecTime = m_pTimer->GetTime() - StartTime;

		int i;

		for(i = 0; i < m_nSupport; i++)
		{
			pPixOut = pIOut->pPix + 3 * PtBuff[i];

			I = pIIn->pPix[PtBuff[i]];

			*(pPixOut++) = 64 + (unsigned char)((((int)I) * 3) >> 2);
			*(pPixOut++) = 0;
			*pPixOut = 0;
		}

		delete[] PtBuff;

		sprintf(pGUI->m_Text[iFirstTextLine], "Plane Support=%d", m_nSupport);

		if(pGUI->m_nTextLines < iFirstTextLine + 1)
			pGUI->m_nTextLines = iFirstTextLine + 1;
	}
	else
	{
		double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
			(m_pStereoVision->m_pCameraL->CenterXNrm - 
			m_pStereoVision->m_pCameraR->CenterXNrm);

		if(pSelectedAPix == NULL)
			return;
		
		//Karlo_TG
		//RVLPSDDisplaySegments(&(m_pStereoVision->m_DisparityMap),
		//					  pIIn,
		//					  pIOut,
		//					  m_pGround,
		//					  m_WallArray,
		//					  RVLPSDDISPLAY_TWOSEGMENTS,
		//					  false);
		//

		int uSelected = (pSelectedAPix->u >> 1);
		int vSelected = (pSelectedAPix->v >> 1);

		int mind, maxd, disp;
		int iSelectedPix = pSelectedAPix - pAImage->m_pPix;

		BOOL bSegment = FALSE;
	
		p2DRegionList->Start();

		while(p2DRegionList->m_pNext)
		{
			pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

			mind = 100000;
			maxd = 0;

			if(bSuperSegments)
			{
				pRelList = pSegment->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_COMPONENTS];
				ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
				ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
				nSubSegments = ppSubRegionEnd - ppSubRegionStart;

				for(j=0;j<nSubSegments;j++,ppSubRegionStart++)
				{
					p2DSubRegion = *ppSubRegionStart;

					ppPt = (int *)p2DSubRegion->m_PtArray;

					pPtArrayEnd = ppPt + p2DSubRegion->m_nPts;

					for(; ppPt < pPtArrayEnd; ppPt++)
					{
						index = *ppPt;

						if(index == iSelectedPix)
							bSegment = TRUE;

						v = index / m_Width;
						u = index % m_Width;

						disp = (int)(pSegment->m_a*u + pSegment->m_b*v + pSegment->m_c);

						if(disp > maxd)
							maxd = disp;

						if(disp < mind)
							mind = disp;
					}

					if(bSegment)
						break;
				}
			}//if(bSuperSegments)
			else
			{
				ppPt = (int *)pSegment->m_PtArray;

				pPtArrayEnd = ppPt + pSegment->m_nPts;

				for(; ppPt < pPtArrayEnd; ppPt++)
				{
					index = *ppPt;

					if(index == iSelectedPix)
						bSegment = TRUE;

					v = index / m_Width;
					u = index % m_Width;

					disp = (int)(pSegment->m_a*u + pSegment->m_b*v + pSegment->m_c);

					if(disp > maxd)
						maxd = disp;

					if(disp < mind)
						mind = disp;
				}

				if(bSegment)
					break;
			
			}//else(!bSuperSegments)

			
		}	
		
		if(!bSegment)
			return;

		sprintf(pGUI->m_Text[iFirstTextLine], "Plane Support=%d tol=%3.0lf", 
			pSegment->m_nPts,
			pSegment->m_Tol);

		if(pGUI->m_nTextLines < 6)
			pGUI->m_nTextLines = 6;		

		if(pSegment->m_Flags & RVL2DREGION_FLAG_GROUND)
		{
			sprintf(pGUI->m_Text[iFirstTextLine + 1], "");

			return;
		}

		if(m_pGround == NULL)
		{
			sprintf(pGUI->m_Text[iFirstTextLine + 1], "");

			return;
		}

		sprintf(pGUI->m_Text[iFirstTextLine + 1], "Inclination=%4.0lf[deg]", 
			acos(RVLDotProduct(((CRVL3DSurface2 *)(pSegment->m_vp3DSurface))->m_N,
			((CRVL3DSurface2 *)(m_pGround->m_vp3DSurface))->m_N)) * RAD2DEG);

		double a1 = m_pGround->m_a;
		double b1 = m_pGround->m_b;
		double c1 = m_pGround->m_c;

		double a2 = pSegment->m_a;
		double b2 = pSegment->m_b;
		double c2 = pSegment->m_c;

		double fmaxd = (double)maxd / 16.0 + d0;
		double fmind = (double)mind / 16.0 + d0;

		int u1 = (DOUBLE2INT((b2*fmind-c1*b2-b1*fmind+b1*c2)/(a1*b2-b1*a2)) << 1) + 1;
		int v1 = (DOUBLE2INT(-(a2*fmind-c1*a2-a1*fmind+a1*c2)/(a1*b2-b1*a2)) << 1) + 1;

		int u2 = (DOUBLE2INT((b2*fmaxd-c1*b2-b1*fmaxd+b1*c2)/(a1*b2-b1*a2)) << 1) + 1;
		int v2 = (DOUBLE2INT(-(a2*fmaxd-c1*a2-a1*fmaxd+a1*c2)/(a1*b2-b1*a2)) << 1) + 1;
			
		CRVLDisplayVector Vector(pFig->m_pMem);
		
		Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
		
		//Vector.m_rL = 255;
		//Vector.m_gL = 255;
		//Vector.m_bL = 0;
		Vector.m_rL = 0;
		Vector.m_gL = 0;
		Vector.m_bL = 0;

		Vector.m_LineWidth = 2;

		CRVLDisplayVector *pVector = pFig->AddVector(&Vector);

		pVector->Line(u1, v1, u2, v2);


		//Draw line from reference file if exists
		//open uv file and get the first 2 points
		FILE *uvFile = fopen(pGUI->m_InputImageUVFileName, "r");
		
		if(uvFile != NULL)
		{
			int n = 0;
			int u10,u20,v10,v20 = 0;
			char chUV[10];
			
			//get first 4 numbers
			while(!feof(uvFile))
			{
				fgets(chUV,10,uvFile);
				n++;
				
				if(n==1)
					u10 = atoi(chUV);
				else if(n==2)
					v10 = atoi(chUV);
				else if(n==3)
					u20 = atoi(chUV);
				else if(n==4)
					v20 = atoi(chUV);
				else
					break;

			}

			fclose(uvFile);

			CRVLDisplayVector VectorR(pFig->m_pMem);

			VectorR.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
 
			//VectorR.m_rL = 255;
			//VectorR.m_gL = 0;
			//VectorR.m_bL = 255;
			
			VectorR.m_rL = 255;
			VectorR.m_gL = 255;
			VectorR.m_bL = 255;

			VectorR.m_LineWidth = 2;

			pVector = pFig->AddVector(&VectorR);

			pVector->Line(u10*2, v10*2, u20*2, v20*2);

			//Karlo_TG
			//RVLPSDDrawLine(u1/2,v1/2,u2/2,v2/2,pIOut,false, u10,u20);
            //RVLPSDDrawLine(u10,v10,u20,v20,pIOut,true,u10,u20);


			GetMinAngles(pGUI->m_InputImageUVFileName);

			sprintf(pGUI->m_Text[iFirstTextLine + 2], "Ref. line coordinates (%d,%d) (%d,%d)", u10,v10,u20,v20);
			sprintf(pGUI->m_Text[iFirstTextLine + 3], "AngleBtwnLines=%11.6lf[deg]", m_minAngleBtwnLines * RAD2DEG);
			sprintf(pGUI->m_Text[iFirstTextLine + 4], "AngleBtwnLinesIn3D=%11.6lf[deg]", m_minAngleBtwnProjectedLines * RAD2DEG);

		}


	}	// selection: if((Flags & RVLPSDDISPLAY_GET_SURFACE) == 0)
	*/
}







//used for SegmentRB and SegmentRB2
void CRVLPlanarSurfaceDetector::SegmentRBDisplay2(	CRVLGUI *pGUI,
													CRVLFigure *pFig,
													int iFirstTextLine,
													CRVLAImage *pAImage,
													PIX_ARRAY *pIIn,
													RVLAPIX *pSelectedAPix,
													PIX_ARRAY *pIOut,
													DWORD Flags)
{
	unsigned char *pPixIn = pIIn->pPix;

	int w = pIIn->Width;
	int h = pIIn->Height;

	unsigned char *pPixOut = pIOut->pPix;

	short int *d = m_pStereoVision->m_DisparityMap.Disparity;

	CRVLMPtrChain *p2DRegionList;

	bool bSuperSegments = (pAImage->m_C2DRegion3.m_ObjectList.m_nElements>0);
	CRVLC2D *p2DRegionSet2, *p2DRegionSet;

	//if there are super segments display them else standard segments
	if(bSuperSegments)
	{
		p2DRegionList = &(pAImage->m_C2DRegion3.m_ObjectList);
		p2DRegionSet2 = &(pAImage->m_C2DRegion3);
	}
	else
	{
		p2DRegionList = &(pAImage->m_C2DRegion.m_ObjectList);
	}
	
	p2DRegionSet = &(pAImage->m_C2DRegion);


	int u, v;
	unsigned char I;
	int iPix;

	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pPixIn++,pPixOut += 3)
		{
			iPix = u + v * m_Width;

			I = (*pPixIn);

			pPixOut[0] = I;
			pPixOut[1] = I;
			pPixOut[2] = I;


			if(Flags & RVLPSDDISPLAY_DISPARITY_OVER_IMAGE)
			{
				if(d[iPix] >= 0)
				{
				
					pPixOut[0] = 0;
					pPixOut[1] = 64 + (unsigned char)((((int)I) * 3) >> 2);
					pPixOut[2] = 0;

				}
				
			}
		}
	}

	// display segmented image

	CRVL2DRegion2 *pSegment;

	int nSegments = p2DRegionList->m_nElements;

	CRVL2DRegion2 **SortedSurfaceArray = new CRVL2DRegion2 *[nSegments];

	CRVL2DRegion2 **ppSegment = SortedSurfaceArray;

	if(m_pGround)
		*(ppSegment++) = m_pGround;

	int i,j;

	for(i = 0; i < m_nWalls; i++)
		*(ppSegment++) = m_WallArray[i];

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if((pSegment->m_Flags & (RVL2DREGION_FLAG_GROUND | RVL2DREGION_FLAG_WALL)) == 0)
			*(ppSegment++) = pSegment;
	}

	int *ppPt, *pPtArrayEnd;
	int index;


	CRVL2DRegion2 **ppSubRegionStart, **ppSubRegionEnd, *p2DSubRegion;
	RVLARRAY *pRelList;
	int nSubSegments;
	

	
	

	if(Flags & RVLPSDDISPLAY_SEGMENTATION)
	{
		double log2 = log(2.0);

		double h;
		int s;
		RVLCOLOR Color;

		p2DRegionList->Start();

		for(i = 1; i <= nSegments; i++)
		{
			s = (1 << (int)floor(log((double)i)/log2));

			h = PI * ((double)(2 * (i - s) + 1) / (double)s - 1.0);

			pSegment = SortedSurfaceArray[i - 1];

			if(bSuperSegments)
			{
				pRelList = pSegment->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_COMPONENTS];
				ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
				ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
				nSubSegments = ppSubRegionEnd - ppSubRegionStart;

				for(j=0;j<nSubSegments;j++,ppSubRegionStart++)
				{
					p2DSubRegion = *ppSubRegionStart;
					
					ppPt = (int *)p2DSubRegion->m_PtArray;

					pPtArrayEnd = ppPt + p2DSubRegion->m_nPts;

					for(; ppPt < pPtArrayEnd; ppPt++)
					{
						index = *ppPt;

						pPixOut = pIOut->pPix + 3 * index;

						I = pAImage->m_pPix[index].I;

						Color = RVLHSL2RGB(h, 1.0, (double)I / 255.0);

						*(pPixOut++) = Color.r;
						*(pPixOut++) = Color.g;
						*pPixOut     = Color.b;
					}
				}
			
			}//if (bSuperSegments)
			else
			{

				ppPt = (int *)pSegment->m_PtArray;

				pPtArrayEnd = ppPt + pSegment->m_nPts;

				for(; ppPt < pPtArrayEnd; ppPt++)
				{
					index = *ppPt;

					pPixOut = pIOut->pPix + 3 * index;

					I = pAImage->m_pPix[index].I;

					Color = RVLHSL2RGB(h, 1.0, (double)I / 255.0);

					*(pPixOut++) = Color.r;
					*(pPixOut++) = Color.g;
					*pPixOut     = Color.b;
				}
			}//else (!bSuperSegments) 

		}
	}

	delete[] SortedSurfaceArray;

	

	// display selection
	if(pSelectedAPix == NULL)
			return;



	if(Flags & RVLPSDDISPLAY_GET_SURFACE)
	{
		int *PtBuff = new int[4 * m_Width * m_Height];

		m_nSupport = GetSurface(pSelectedAPix->u, pSelectedAPix->v, PtBuff);

		for(i = 0; i < m_nSupport; i++)
		{
			pPixOut = pIOut->pPix + 3 * PtBuff[i];

			I = pIIn->pPix[PtBuff[i]];

			*(pPixOut++) = 64 + (unsigned char)((((int)I) * 3) >> 2);
			*(pPixOut++) = 0;
			*pPixOut = 0;
		}

		delete[] PtBuff;

		sprintf(pGUI->m_Text[iFirstTextLine], "Plane Support=%d", m_nSupport);

		if(pGUI->m_nTextLines < iFirstTextLine + 1)
			pGUI->m_nTextLines = iFirstTextLine + 1;
	}
	else
	{
		double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
			(m_pStereoVision->m_pCameraL->CenterXNrm - 
			m_pStereoVision->m_pCameraR->CenterXNrm);

		
		//Karlo_TG
		/*RVLPSDDisplaySegments2(pAImage,
							  &(m_pStereoVision->m_DisparityMap),
							  pIIn,
							  pIOut,
							  m_pGround,
							  m_WallArray,
							  RVLPSDDISPLAY_TWOSEGMENTS,
							  false);*/
		

		//int uSelected = (pSelectedAPix->u >> 1);
		//int vSelected = (pSelectedAPix->v >> 1);

		int mind, maxd, disp;
		int iSelectedPix = pSelectedAPix - pAImage->m_pPix;

		BOOL bSegment = FALSE;
	
		p2DRegionList->Start();

		while(p2DRegionList->m_pNext)
		{
			mind = 100000;
			maxd = 0;

			pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

			if(bSuperSegments)
			{
				pRelList = pSegment->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_COMPONENTS];
				ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
				ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
				nSubSegments = ppSubRegionEnd - ppSubRegionStart;

				for(j=0;j<nSubSegments;j++,ppSubRegionStart++)
				{
					p2DSubRegion = *ppSubRegionStart;
					
					ppPt = (int *)p2DSubRegion->m_PtArray;

					pPtArrayEnd = ppPt + p2DSubRegion->m_nPts;
				
					for(; ppPt < pPtArrayEnd; ppPt++)
					{
						index = *ppPt;

						if(index == iSelectedPix)
							bSegment = TRUE;

						v = index / m_Width;
						u = index % m_Width;

						disp = (int)(pSegment->m_a*u + pSegment->m_b*v + pSegment->m_c);

						if(disp > maxd)
							maxd = disp;

						if(disp < mind)
							mind = disp;
					}

				}

				if(bSegment)
					break;
			}


		else
			{
					
					ppPt = (int *)pSegment->m_PtArray;

					pPtArrayEnd = ppPt + pSegment->m_nPts;

					for(; ppPt < pPtArrayEnd; ppPt++)
					{
						index = *ppPt;

						if(index == iSelectedPix)
							bSegment = TRUE;

						v = index / m_Width;
						u = index % m_Width;

						disp = (int)(pSegment->m_a*u + pSegment->m_b*v + pSegment->m_c);

						if(disp > maxd)
							maxd = disp;

						if(disp < mind)
							mind = disp;
					}

					if(bSegment)
						break;
			}
		}	
		
		if(!bSegment)
			return;

		if((Flags & RVLPSDDISPLAY_SEGMENTATION) == 0)
		{
			ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;

			for(j=0;j<nSubSegments;j++,ppSubRegionStart++)
			{
				p2DSubRegion = *ppSubRegionStart;

				p2DSubRegion->Display(pIOut, pAImage);
			}
		}

		sprintf(pGUI->m_Text[iFirstTextLine], "Plane Support=%d tol=%3.0lf", 
			pSegment->m_nPts,
			pSegment->m_Tol);

		if(pGUI->m_nTextLines - iFirstTextLine < 2)
			pGUI->m_nTextLines = iFirstTextLine + 2;	

		

		if(m_pGround == NULL)
		{
			sprintf(pGUI->m_Text[iFirstTextLine + 1], "");

			return;
		}

		CRVL3DSurface2 *p3DGround = (CRVL3DSurface2 *)(m_pGround->m_vp3DSurface);

		if(pSegment->m_Flags & RVL2DREGION_FLAG_GROUND)
		{
			sprintf(pGUI->m_Text[iFirstTextLine + 1], "Distance=%5.2lf[m]", 
				0.001 * p3DGround->m_d);

			return;
		}

		CRVL3DSurface2 *p3DSegment = (CRVL3DSurface2 *)(pSegment->m_vp3DSurface);

		double V[3];

		CrossProduct(p3DSegment->m_N, p3DGround->m_N, V);

		sprintf(pGUI->m_Text[iFirstTextLine + 1], "Inclination=%4.0lf[deg]  Orientation=%4.0lf[deg]  Distance=%5.2lf[m]", 
			acos(RVLDotProduct(p3DSegment->m_N, p3DGround->m_N)) * RAD2DEG,
			atan2(-V[0],V[2]) * RAD2DEG,
			0.001 * p3DSegment->m_d
			);

		double a1 = m_pGround->m_a;
		double b1 = m_pGround->m_b;
		double c1 = m_pGround->m_c;

		double a2 = pSegment->m_a;
		double b2 = pSegment->m_b;
		double c2 = pSegment->m_c;

		double fmaxd = (double)maxd;
		double fmind = (double)mind;

		int u1 = (DOUBLE2INT((b2*fmind-c1*b2-b1*fmind+b1*c2)/(a1*b2-b1*a2)) << 1) + 1;
		int v1 = (DOUBLE2INT(-(a2*fmind-c1*a2-a1*fmind+a1*c2)/(a1*b2-b1*a2)) << 1) + 1;

		int u2 = (DOUBLE2INT((b2*fmaxd-c1*b2-b1*fmaxd+b1*c2)/(a1*b2-b1*a2)) << 1) + 1;
		int v2 = (DOUBLE2INT(-(a2*fmaxd-c1*a2-a1*fmaxd+a1*c2)/(a1*b2-b1*a2)) << 1) + 1;
			
		CRVLDisplayVector Vector(pFig->m_pMem);
		
		Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
		
		Vector.m_rL = 0;  //255;
		Vector.m_gL = 0;	//255;
		Vector.m_bL = 0;	//0;

		Vector.m_LineWidth = 2;

		CRVLDisplayVector *pVector = pFig->AddVector(&Vector);

		pVector->Line(u1, v1, u2, v2);


		//Draw line from reference file if exists
		//open uv file and get the first 2 points
		FILE *uvFile = fopen(pGUI->m_InputImageUVFileName, "r");
		
		if(uvFile != NULL)
		{
			if(pGUI->m_nTextLines - iFirstTextLine < 5)
				pGUI->m_nTextLines = iFirstTextLine + 5;	

			int n = 0;
			int u10,u20,v10,v20 = 0;
			char chUV[10];
			
			//get first 4 numbers
			while(!feof(uvFile))
			{
				fgets(chUV,10,uvFile);
				n++;
				
				if(n==1)
					u10 = atoi(chUV);
				else if(n==2)
					v10 = atoi(chUV);
				else if(n==3)
					u20 = atoi(chUV);
				else if(n==4)
					v20 = atoi(chUV);
				else
					break;

			}

			fclose(uvFile);

			CRVLDisplayVector VectorR(pFig->m_pMem);

			VectorR.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
 
						
			VectorR.m_rL = 255;
			VectorR.m_gL = 255;
			VectorR.m_bL = 255;

			VectorR.m_LineWidth = 2;

			pVector = pFig->AddVector(&VectorR);

			pVector->Line(u10*2, v10*2, u20*2, v20*2);

			//Karlo_TG
			//RVLPSDDrawLine(u1/2,v1/2,u2/2,v2/2,pIOut,false, u10,u20);
            //RVLPSDDrawLine(u10,v10,u20,v20,pIOut,true,u10,u20);


			GetMinAngles(pGUI->m_InputImageUVFileName);

			sprintf(pGUI->m_Text[iFirstTextLine + 2], "Ref. line coordinates (%d,%d) (%d,%d)", u10,v10,u20,v20);
			sprintf(pGUI->m_Text[iFirstTextLine + 3], "AngleBtwnLines=%11.6lf[deg]", m_minAngleBtwnLines * RAD2DEG);
			sprintf(pGUI->m_Text[iFirstTextLine + 4], "AngleBtwnLinesIn3D=%11.6lf[deg]", m_minAngleBtwnProjectedLines * RAD2DEG);

		}

	}	

	
}



//used for SegmentRB3
void CRVLPlanarSurfaceDetector::SegmentRBDisplay3(	CRVLGUI *pGUI,
													CRVLFigure *pFig,
													int iFirstTextLine,
													CRVLAImage *pAImage,
													RVLAPIX *pSelectedAPix,
													PIX_ARRAY *pIOut,
													DWORD Flags)
{
	RVL3DPOINT2 *pPoint3D, *pPoint3D2;
	RVL3DPOINT2 *pPoint3DArrayEnd;

	CRVLMPtrChain *pRegionList;

	CRVL2DRegion2  **ppSuperRegion, *pSuperRegion, *pRegion, *pRegionText;  
	CRVL2DRegion2 **ppSubRegionStart, **ppSubRegionEnd, *p2DSubRegion;
	RVLARRAY *pRelList;
	int nSubSegments;

	CRVLMPtrChain *p2DRegionList;
	CRVLC2D *p2DRegionSet;

	double e;

	int iPix, index;
	int *ppPt, *pPtArrayEnd;
	RVL2DREGION_PIXIDX *pPixPtr;

	
	unsigned char *pPixOut;
	
	if ((m_Flags & RVLPSD_FLAG_SUBSEGMENT) != 0)
	{
		//Mark all Reliable 3D points
		pPoint3DArrayEnd = m_Point3DArray + m_n3DPoints;
		for(pPoint3D = m_Point3DArray; pPoint3D < pPoint3DArrayEnd; pPoint3D++)
		{
			iPix = pPoint3D->u + pPoint3D->v * m_Width;
			pPixOut = pIOut->pPix + 3 * iPix;

			pPixOut[0] = 0;
			pPixOut[1] = 0;
			pPixOut[2] = 255;

		}

		RVL3DPOINT2 **ppP3D,**ppP3DEnd;

		//Check if a point has been clicked
		if(pGUI->m_pSelectedObject)
		{
			pPoint3D = (RVL3DPOINT2 *)(pGUI->m_pSelectedObject);

			pRegionList = &(pPoint3D->regionList);

			int i = 0;
			bool bRegionFound = false;

			pRegionList->Start();

			while(pRegionList->m_pNext)
			{
				pRegion = (CRVL2DRegion2 *)pRegionList->GetNext();
				pRegionText = pRegion;

				if(i== pGUI->m_iStep)
				{
					bRegionFound = true;
					break;
				}
				i++;
			}
			
    		if(bRegionFound)
			{
				//use super segments
				if((Flags & RVLPSDDISPLAY_SEGMENTATION) != 0)
				{
					p2DRegionSet = &(pAImage->m_C2DRegion3);

					ppSuperRegion = (CRVL2DRegion2 **)(pRegion->m_pData + pRegion->m_pClass->m_iDataGrandParentPtr);
					pSuperRegion = *ppSuperRegion;

					if(pSuperRegion != NULL)
					{
						pRegionText = pSuperRegion;

						pRelList = pSuperRegion->m_RelList + p2DRegionSet->m_iRelList[RVLRELLIST_ELEMENTS];
						ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
						ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
						nSubSegments = ppSubRegionEnd - ppSubRegionStart;

						for(int j=0;j<nSubSegments;j++,ppSubRegionStart++)
						{
							p2DSubRegion = *ppSubRegionStart;
							
							ppP3D = p2DSubRegion->m_pPoint3DArray;
							ppP3DEnd = ppP3D + p2DSubRegion->m_n3DPts;

							for(; ppP3D < ppP3DEnd; ppP3D++)
							{
								pPoint3D2 = *ppP3D;

								e = pPoint3D2->z - (pSuperRegion->m_a * pPoint3D2->x + pSuperRegion->m_b * pPoint3D2->y + pSuperRegion->m_c);
								
								if(e > m_Tol)
									continue;

								if(e >= -m_Tol)
								{
									iPix = pPoint3D2->u + pPoint3D2->v * m_Width;
									pPixOut = pIOut->pPix + 3 * iPix;

									pPixOut[0] = 255;
									pPixOut[1] = 255;
									pPixOut[2] = 0;
								}
							}
						}

					}
					else  //display only the selected region
					{
						ppP3D = pRegion->m_pPoint3DArray;
						ppP3DEnd = ppP3D + pRegion->m_n3DPts;

						for(; ppP3D < ppP3DEnd; ppP3D++)
						{
							pPoint3D2 = *ppP3D;
							
							e = pPoint3D2->z - (pRegion->m_a * pPoint3D2->x + pRegion->m_b * pPoint3D2->y + pRegion->m_c);
								
							if(e > m_Tol)
								continue;

							if(e >= -m_Tol)
							{
								iPix = pPoint3D2->u + pPoint3D2->v * m_Width;
								pPixOut = pIOut->pPix + 3 * iPix;

								pPixOut[0] = 255;
								pPixOut[1] = 255;
								pPixOut[2] = 0;
							}
						}

					}
		
				}
				else
				{
					ppP3D = pRegion->m_pPoint3DArray;
					ppP3DEnd = ppP3D + pRegion->m_n3DPts;

					for(; ppP3D < ppP3DEnd; ppP3D++)
					{
						pPoint3D2 = *ppP3D;
						e = pPoint3D2->z - (pRegion->m_a * pPoint3D2->x + pRegion->m_b * pPoint3D2->y + pRegion->m_c);
								
						if(e > m_Tol)
							continue;

						if(e >= -m_Tol)
						{
							iPix = pPoint3D2->u + pPoint3D2->v * m_Width;
							pPixOut = pIOut->pPix + 3 * iPix;

							pPixOut[0] = 255;
							pPixOut[1] = 255;
							pPixOut[2] = 0;
						}
					}

				}




				sprintf(pGUI->m_Text[iFirstTextLine], "Plane params a=%7.2lf b=%7.2lf c=%7.2lf", pRegionText->m_a, pRegionText->m_b, pRegionText->m_c);
				if(pGUI->m_nTextLines < iFirstTextLine + 1)
					pGUI->m_nTextLines = iFirstTextLine + 1;
				
			}
		
		
		}
	
	}
	else
	{
		//int mind = 100000;
		//int	maxd = 0;
		int u, v;
		//unsigned char I;
		
		double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
							(m_pStereoVision->m_pCameraL->CenterXNrm - 
							m_pStereoVision->m_pCameraR->CenterXNrm);

		double dispScale = 16.0;
		int iScale = 4; //4

		//over write all pixels
		for(v = 0; v < m_Height; v++)
		{
			for(u = 0; u < m_Width; u++)
			{
				iPix = u + v * m_Width;
				pPixOut = pIOut->pPix + 3 * iPix;


				pPixOut[0] = 0;
				pPixOut[1] = 0;
				pPixOut[2] = 0;
			}
		}

		
		

		bool bSuperSegments = (pAImage->m_C2DRegion3.m_ObjectList.m_nElements>0);

		p2DRegionSet =  bSuperSegments ? &(pAImage->m_C2DRegion3) : &(pAImage->m_C2DRegion);
		p2DRegionList = bSuperSegments ? &(pAImage->m_C2DRegion3.m_ObjectList) : &(pAImage->m_C2DRegion.m_ObjectList);

			
		p2DRegionList->Start();

		while(p2DRegionList->m_pNext)
		{
			pRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());
		
			if(bSuperSegments)
			{
				pRelList = pRegion->m_RelList + p2DRegionSet->m_iRelList[RVLRELLIST_ELEMENTS];   //if pAImage->m_C2DRegion3 then RVLRELLIST_ELEMENTS elseif pAImage->m_C2DRegion2 then RVLRELLIST_COMPONENTS 
				ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
				ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
				nSubSegments = ppSubRegionEnd - ppSubRegionStart;
				
				for(int j=0;j<nSubSegments;j++,ppSubRegionStart++)
				{
					p2DSubRegion = *ppSubRegionStart;

					if(m_bEdgeBasedSegments)
					{
						pPixPtr = (RVL2DREGION_PIXIDX *)(p2DSubRegion->m_PtArray);

						while(pPixPtr)
						{
							index = pPixPtr->iPix;

							SegmentDisplayHelper(index,pRegion,pIOut,d0,dispScale,iScale);

							pPixPtr = pPixPtr->pNext;
						}
					}
					else
					{

						ppPt = (int *)p2DSubRegion->m_PtArray;

						pPtArrayEnd = ppPt + p2DSubRegion->m_nPts;

						for(; ppPt < pPtArrayEnd; ppPt++)
						{
							index = *ppPt;

							SegmentDisplayHelper(index,pRegion,pIOut,d0,dispScale,iScale);
							
						}	
					}
				}

			}
			else
			{
				if(m_bEdgeBasedSegments)
				{
					pPixPtr = (RVL2DREGION_PIXIDX *)(pRegion->m_PtArray);

					while(pPixPtr)
					{
						index = pPixPtr->iPix;

						SegmentDisplayHelper(index,pRegion,pIOut,d0,dispScale,iScale);

						pPixPtr = pPixPtr->pNext;
					}
				
					
				}
				else
				{
					ppPt = (int *)pRegion->m_PtArray;

					pPtArrayEnd = ppPt + pRegion->m_nPts;

					for(; ppPt < pPtArrayEnd; ppPt++)
					{
						index = *ppPt;

						SegmentDisplayHelper(index,pRegion,pIOut,d0,dispScale,iScale);

					}	
				}
			
			}

		}

		
		
}


}


//used for SegmentRB3
void CRVLPlanarSurfaceDetector::SegmentRBDisplayContour(CRVLGUI *pGUI,
														CRVLFigure *pFig,
														CRVLAImage *pAImage,
														PIX_ARRAY *pIIn,
														PIX_ARRAY *pIOut)
{


	RVLARRAY *pRelList;

	CRVL2DRegion2 *pRegion;

	CRVLMPtrChain *p2DRegionList;
	CRVLC2D *p2DRegionSet;

	CRVL2DContour **ppContourStart,**ppContourEnd, *p2DContour;
	int nContours;

	
	unsigned char *pPixOut = pIOut->pPix;
	unsigned char *pPixIn = pIIn->pPix;
	
	int u1, v1;
	//int u, v, u2, v2;
	//int iPix;
	
	CRVLDisplayVector Vector(pFig->m_pMem);
		
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	
	Vector.m_rL = 255;  //255;
	Vector.m_gL = 255;	//255;
	Vector.m_bL = 0;	//0;

	Vector.m_bClosed = TRUE;

	Vector.m_LineWidth = 1;

	
	//unsigned char I;

	//over write all pixels
	//for(v = 0; v < m_Height; v++)
	//{
	//	for(u = 0; u < m_Width; u++,pPixIn++)
	//	{
	//		iPix = u + v * m_Width;
	//		pPixOut = pIOut->pPix + 3 * iPix;
	//		
	//		I = (*pPixIn);

	//		pPixOut[0] = I;
	//		pPixOut[1] = I;
	//		pPixOut[2] = I;

	//		/*pPixOut[0] = 0;
	//		pPixOut[1] = 0;
	//		pPixOut[2] = 0;*/
	//	}
	//}

	
	p2DRegionSet =  &(pAImage->m_C2DRegion2);
	p2DRegionList = &(pAImage->m_C2DRegion2.m_ObjectList);

		
	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());
			
		pRelList = pRegion->m_RelList + p2DRegionSet->m_iRelList[RVLRELLIST_CONTOURS];   
		ppContourStart = (CRVL2DContour **)pRelList->pFirst;
		ppContourEnd = (CRVL2DContour **)pRelList->pEnd;
		nContours = ppContourEnd - ppContourStart;
		
		for(int j=0;j<nContours;j++,ppContourStart++)
		{
			p2DContour = *ppContourStart;

			CRVLDisplayVector *pVector = pFig->AddVector(&Vector);

			for (int i=0;i<p2DContour->m_nContourIPs;i++)
			{
				u1 = p2DContour->m_ContourIPArray[i].u;
				v1 = p2DContour->m_ContourIPArray[i].v;

				//u2 = p2DContour->m_ContourIPArray[i+1].u;
				//v2 = p2DContour->m_ContourIPArray[i+1].v;				

				pVector->Point((u1 << 1) + 1, (v1 << 1) + 1);

			/*	u = p2DContour->m_ContourIPArray[i].u;
				v = p2DContour->m_ContourIPArray[i].v;

				iPix = u + v * m_Width;

				pPixOut = pIOut->pPix + 3 * iPix;

				pPixOut[0] = 255;
				pPixOut[1] = 0;
				pPixOut[2] = 0;*/

				
			
			}
			
			

			
		}
	
		
	}


}


void CRVLPlanarSurfaceDetector::SegmentDisplayHelper(int index, CRVL2DRegion2 *pRegion, PIX_ARRAY *pIOut, double d0, double dispScale, int iScale )
{
	int u,v;
	unsigned char* pPixOut;
	unsigned char I;

	pPixOut = pIOut->pPix + 3 * index;

	v = index / m_Width;
	u = index % m_Width;

	I = (unsigned char)((pRegion->m_a*u + pRegion->m_b*v + pRegion->m_c + d0*dispScale)*iScale/dispScale);

	pPixOut[0] = I;
	pPixOut[1] = I;
	pPixOut[2] = I;

	
}


//To display all points in 3D (if segmentRB is used)
void CRVLPlanarSurfaceDetector::Display2(CRVLAImage *pAImage,
										 PIX_ARRAY *pIOut,
										 DWORD Flags)
{
	//unsigned char *pPixIn = pIIn->pPix;

	int w = pAImage->m_Width;
	int h = pAImage->m_Height;

	unsigned char *pPixOut;// = pIOut->pPix;

	//short int *d = m_pStereoVision->m_DisparityMap.Disparity;


	int u, v;
	unsigned char I;
	int iPix;

	CRVL2DRegion2 *pRegion;

	iPix = m_Width + 1;

	//TO DO: put zeros in first/last row/column
	
	for(v = 1; v < h-1; v++)
	{
		for(u = 1; u < w-1; u++, iPix++ ) //= 3   pPixIn++, pPixOut +=3
		{
			//iPix = u + v * m_Width;
			pPixOut = pIOut->pPix + 3 * iPix;

			pPixOut[0] = 0;
			pPixOut[1] = 0;
			pPixOut[2] = 0;

			//get segment region from selected pixel
			pRegion = pAImage->m_2DRegionMap[iPix];
			if(pRegion)
			{
				CRVL2DRegion2 **ppSuperRegion = (CRVL2DRegion2 **)(pRegion->m_pData + pRegion->m_pClass->m_iDataParentPtr);

				CRVL2DRegion2 *pSuperRegion = *ppSuperRegion;

				if(pSuperRegion)
				{
					//I = (unsigned char)((pRegion->m_a*u + pRegion->m_b*v + pRegion->m_c)/4)  ;

					I = (unsigned char)((pSuperRegion->m_a*u + pSuperRegion->m_b*v + pSuperRegion->m_c)*4)  ;
					
					pPixOut[0] = I;
					pPixOut[1] = I;
					pPixOut[2] = I;
				}
			}		
			

		}
		iPix +=2;
	}

	
}






void CRVLPlanarSurfaceDetector::Init()
{
	/////

	m_Width = m_pStereoVision->m_pCameraL->m_nrmImage.Width;
	m_Height = m_pStereoVision->m_pCameraL->m_nrmImage.Height;

	int ImageSize = m_Width * m_Height;

	//if(m_Segmentation)
	//	delete[] m_Segmentation;

	//m_Segmentation = new WORD[ImageSize];

	// initialize region growing tools

	m_dpNeighbor4[0] = 1;
	m_dpNeighbor4[1] = m_Width;
	m_dpNeighbor4[2] = -1;
	m_dpNeighbor4[3] = -m_Width;
	m_dpNeighbor4[4] = m_Width+1;
	m_dpNeighbor4[5] = m_Width-1;
	m_dpNeighbor4[6] = -m_Width-1;
	m_dpNeighbor4[7] = -m_Width+1;

	m_NeighborLimit[0] = m_Width - 1;
	m_NeighborLimit[1] = m_Height - 1;
	m_NeighborLimit[2] = 0;
	m_NeighborLimit[3] = 0;

	m_diPixContourFollow[0][0] = -m_Width;
	m_diPixContourFollow[0][1] = -m_Width + 1;
	m_diPixContourFollow[0][3] = 0;
	m_diPixContourFollow[1][0] = 0;
	m_diPixContourFollow[1][1] = 1;
	m_diPixContourFollow[1][2] = m_Width + 1;
	m_diPixContourFollow[2][1] = 0;
	m_diPixContourFollow[2][2] = m_Width;
	m_diPixContourFollow[2][3] = m_Width-1;
	m_diPixContourFollow[3][0] = -m_Width - 1;
	m_diPixContourFollow[3][2] = 0;
	m_diPixContourFollow[3][3] = -1;

	//half the row length used in searching for reliable disparity points
	m_RowLength = 3;

	// allocate arrays

	int maxn3DPts = (2 * m_nFOVExtensions + 1) * ImageSize;

	if(m_Point3DArray)
		delete[] m_Point3DArray;

	m_Point3DArray = new RVL3DPOINT2[maxn3DPts];

	if(m_Point3DMapMem)
		delete[] m_Point3DMapMem;

	m_Point3DMapMem = new RVL3DPOINT2 *[maxn3DPts];

	m_Point3DMap = m_Point3DMapMem;

	if(m_Point3DBuff)
		delete[] m_Point3DBuff;

	m_Point3DBuff = new RVL3DPOINT2 *[ImageSize];

	if(m_EmptyRegionGrowingMap)
		delete[] m_EmptyRegionGrowingMap;

	m_EmptyRegionGrowingMap = new int[ImageSize];

	int RowSize = m_Width * sizeof(int);

	memset(m_EmptyRegionGrowingMap, 0xfe, RowSize);

	memset(m_EmptyRegionGrowingMap + m_Width, 0xff, (m_Height - 2) * RowSize);

	memset(m_EmptyRegionGrowingMap + (m_Height - 1) * m_Width, 0xfe, RowSize);

	int v;

	for(v = 1; v < m_Height - 1; v++)
		m_EmptyRegionGrowingMap[v * m_Width] = m_EmptyRegionGrowingMap[(v + 1) * m_Width - 1] = 0xfefefefe;

	if(m_RegionGrowingMap)
		delete[] m_RegionGrowingMap;

	m_RegionGrowingMap = new int[ImageSize];

	if(m_RegionGrowingBuff)
		delete[] m_RegionGrowingBuff;

	m_RegionGrowingBuff = new int[ImageSize];

	if(m_CCBuff)
		delete[] m_CCBuff;

	m_CCBuff = new RVL3DPOINT2 *[ImageSize];

	if(m_ResidualHistogram)
		delete[] m_ResidualHistogram;

	m_ResidualHistogram = new int[DOUBLE2INT(m_ResidualThr)];

	m_bEdgeBasedSegments = false;

	m_QueueMem = new RVLPSD_STRM_QUEUE_ENTRY[2 * ImageSize];

	if(m_Flags & RVLPSD_FLAG_MM)
		m_Queue.m_Size = 800;
	else
		m_Queue.m_Size = m_pStereoVision->m_nDisp;

	m_Queue.Init();

	m_iScanLineStart = new int[m_Height];
	m_iScanLineEnd = new int[m_Height];

	m_bCorner = new BYTE[ImageSize];

	if(m_DTMap)
		delete[] m_DTMap;

	m_DTMap = new int[ImageSize];

	if(m_DTQueueEntryMem)
		delete[] m_DTQueueEntryMem;

	m_DTQueueEntryMem = new RVLPSD_DT_QUEUE_ENTRY[ImageSize];

	m_DelaunayLinkQueue.m_Size = m_Width*m_Width + m_Height*m_Height + 1;

	if(m_DelaunayLinkQueue.m_ListArray)
		delete[] m_DelaunayLinkQueue.m_ListArray;

	m_DelaunayLinkQueue.Init();

	if(m_2DRegionMap)
		delete[] m_2DRegionMap;

	m_2DRegionMap = new CRVL2DRegion2 *[ImageSize];

	if(m_APixBuff)
		delete[] m_APixBuff;

	m_APixBuff = new RVLAPIX *[ImageSize];

	if(m_3DSurfaceMap)
		delete[] m_3DSurfaceMap;

	m_3DSurfaceMap = new CRVL3DSurface2 *[ImageSize];
}


//	Main segmentation function
//
//	Relevant member variables:
//
//		m_Flags
//
//	Calls Segment3D, SegmentGB or SegmentEB
void CRVLPlanarSurfaceDetector::Segment(CRVLC2D *p2DRegionLevel1,	// Output: Set of 2D (level1) regions  corresponding initial segments
										CRVLC2D *p2DRegionLevel2,	// Output: Set of 2D (level2) regions corresponding to the dominant planar surfaces
										CRVLC2D *p2DRegionLevel3,	// Output: Set of 2D (level3) regions corresponding to the dominant planar surfaces
										CRVLMem *pMem,
										DWORD FlagsExt,
										CRVLC2DContour *p2DContourSet)
{
	DWORD Flags;

	if((m_Flags & RVLPSD_FLAG_INTERNAL) != 0)
	{
		Flags = m_Flags;
	}
	else
	{
		Flags = FlagsExt;
	}

	if(Flags & RVLPSD_SEGMENT_3D)
		Segment3D(p2DRegionLevel1,Flags);
	else if(Flags & RVLPSD_SEGMENT_REGIONBASED)
	{
		if(p2DRegionLevel3 == NULL)
			SegmentRB(p2DRegionLevel1,Flags);
		else
			SegmentRB4(p2DRegionLevel1,p2DRegionLevel2,p2DRegionLevel3,p2DContourSet,Flags);
			//SegmentRB2(p2DRegionSet,p2DRegionSuperSet,Flags);
	}
	//else if(Flags & RVLPSD_SEGMENT_EDGEBASED)
	//	SegmentEB(p2DRegionSet,Flags);
	else if(Flags & RVLPSD_SEGMENT_STRM)
		SegmentSTRM(p2DRegionLevel1,p2DRegionLevel2,p2DRegionLevel3,pMem,Flags);
}


//	Segmentation of disparity image
//
//	Relevant member variables:
//
//		m_Flags	
//
//		Parameters:
//			m_minSegmentSize		-	min. num. of pts. in a connected component
//
//		Inputs:
//			m_Width, m_Height		-	disparity image width and height
//			m_Point3DArray			-	set A of points in uvd-space
//			m_n3DPoints				-	num. of pts. in A
//
//		Internal:
//			m_RegionGrowingMap
//
//	Flags:
//		RVLPSD_FLAG_INTERNAL		-	if set, m_Flags is used, otherwise FlagsExt is considered

void CRVLPlanarSurfaceDetector::Segment3D(CRVLC2D *p2DRegionSet,		// Output: Set of 2D regions corresponding 
																		// to the dominant planar surfaces
										  DWORD Flags)
{	
	unsigned int *RegionGrowingMap = (unsigned int *)m_RegionGrowingMap;

	p2DRegionSet->Clear();

	int ImageSize = m_Width * m_Height;

	RVL3DPOINT2 **Point3DPtrArray = new RVL3DPOINT2 *[m_n3DPoints];

	RVL3DPOINT2 **ppPt = Point3DPtrArray;

	RVL3DPOINT2 *Point3DArrayEnd = m_Point3DArray + m_n3DPoints;

	RVL3DPOINT2 *pPt;

	// copy A to Point3DPtrArray

	for(pPt = m_Point3DArray; pPt < Point3DArrayEnd; pPt++, ppPt++)
		*ppPt = pPt;

	CRVLMChain CCList(m_pMem2, sizeof(RVLARRAY));

	RVLARRAY CC;

	if(Flags & RVLPSD_FLAG_CONNECTED_COMPONENTS)
	{
		ClearRegionGrowingMap();

		// for every point in A set the corresponding element of RegionGrowingMap to 0

		for(pPt = m_Point3DArray; pPt < Point3DArrayEnd; pPt++, ppPt++)
			if(RegionGrowingMap[pPt->iPix] != 0xfefefefe)
				RegionGrowingMap[pPt->iPix] = 0;

		// get connected components with at least m_minSegmentSize points

		GetConnectedComponents(Point3DPtrArray, m_n3DPoints, &CCList, 
			Flags & (RVLPSDGCC_FLAG_DISPARITY_CONTINUITY | RVLPSDGCC_FLAG_DILATE));

		if(CCList.m_nElements == 0)
		{
			m_pMem2->Clear();

			delete[] Point3DPtrArray;

			return;
		}
	}
	else
	{
		memcpy(m_CCBuff, Point3DPtrArray, m_n3DPoints * sizeof(RVL3DPOINT *));

		CC.pFirst = (BYTE *)m_CCBuff;
		CC.pEnd = (BYTE *)(m_CCBuff + m_n3DPoints);

		CCList.Add(&CC);
	}

	// main loop

	double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
			(m_pStereoVision->m_pCameraL->CenterXNrm - 
			m_pStereoVision->m_pCameraR->CenterXNrm);

	int iSegment = 0;

	CRVLMChain CCList2(m_pMem2, sizeof(RVLARRAY));

	RVLARRAY *pCC;
	int CCSize, CCmaxSize;
	RVLARRAY *pCCmax, *pCCmax2;
	RVL3DPOINT2 **pCCEnd;
	RVL3DPOINT2 **ppPt2;
	CRVL2DRegion2 Plane;
	RVLPSDLAD_SEGMENT_DETAILS segmentDetails;
	int nCCmax, nCCmax2;
	CRVL2DRegion2 *pSegment;
	double a, b, c, ePlane;
	int iPix, n;

	while(TRUE)
	{
		// determine the greatest connected component

		CCmaxSize = 0;

		CCList.Start();

		while(CCList.m_pNext)
		{
			pCC = (RVLARRAY *)(CCList.GetNext());

			CCSize = pCC->pEnd - pCC->pFirst;

			if(CCSize > CCmaxSize)
			{
				CCmaxSize = CCSize;

				pCCmax = pCC;
			}
		}

		nCCmax = CCmaxSize / sizeof(RVL3DPOINT2 *);

		if(nCCmax < m_minSegmentSize)
			break;

		// subsample the greatest connected component

		if(Flags & RVLPSD_FLAG_SUBSAMPLE)
		{
			ppPt = (RVL3DPOINT2 **)(pCCmax->pFirst);

			pCCEnd = (RVL3DPOINT2 **)(pCCmax->pEnd);

			ppPt2 = Point3DPtrArray;

			for(; ppPt < pCCEnd; ppPt++)
			{
				pPt = *ppPt;

				if(pPt->u % m_SubSample)
					continue;

				if(pPt->v % m_SubSample)
					continue;

				*(ppPt2++) = pPt;
			}

			n = ppPt2 - Point3DPtrArray;

			// find dominant planar surface

			RANSAC(&Plane, false, Point3DPtrArray, n, &segmentDetails, 
				Flags | RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET);
		}
		else
			RANSAC(&Plane, false, (RVL3DPOINT2 **)(pCCmax->pFirst), nCCmax, &segmentDetails, 
				Flags | RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET);
	
		// find the connected components in pCCmax consistent with Plane

		if(Flags & RVLPSD_FLAG_CONNECTED_COMPONENTS)
			ClearRegionGrowingMap();

		ppPt = (RVL3DPOINT2 **)(pCCmax->pFirst);

		pCCEnd = (RVL3DPOINT2 **)(pCCmax->pEnd);

		ppPt2 = Point3DPtrArray;

		a = Plane.m_a;
		b = Plane.m_b;
		c = Plane.m_c;

		for(; ppPt < pCCEnd; ppPt++)
		{
			pPt = *ppPt;

			ePlane = a * pPt->x + b * pPt->y + c - pPt->z;

			if(ePlane > m_Tol) 
				continue;

			if(ePlane < -m_Tol) 
				continue;			

			*(ppPt2++) = pPt;

			if(Flags & RVLPSD_FLAG_CONNECTED_COMPONENTS)
			{
				iPix = pPt->iPix;

				RegionGrowingMap[iPix] = 0;
			}
			else
				pPt->segmentNumber = iSegment;
		}	
		
		n = ppPt2 - Point3DPtrArray;

		if(n < m_minSegmentSize)
		{
			// remove pCCmax from CCList

			pCCmax->pEnd = pCCmax->pFirst;

			continue;
		}

		if(Flags & RVLPSD_FLAG_CONNECTED_COMPONENTS)
		{
			GetConnectedComponents(Point3DPtrArray, n, &CCList2, 
					(Flags & RVLPSDGCC_FLAG_DILATE));
					//| RVLPSDGCC_FLAG_PLANE
					//| RVLPSDGCC_FLAG_MASK
					//, 1);

			if(CCList2.m_nElements == 0)
			{
				// remove pCCmax from CCList

				pCCmax->pEnd = pCCmax->pFirst;

				continue;
			}
		}
		else
		{
			CC.pFirst = (BYTE *)Point3DPtrArray;
			CC.pEnd = (BYTE *)Point3DPtrArray + n * sizeof(RVL3DPOINT *);

			CCList2.Add(&CC);			
		}

		// determine the greatest connected component in pCCmax consistent with Plane

		CCmaxSize = 0;

		CCList2.Start();

		while(CCList2.m_pNext)
		{
			pCC = (RVLARRAY *)(CCList2.GetNext());

			CCSize = pCC->pEnd - pCC->pFirst;

			if(CCSize > CCmaxSize)
			{
				CCmaxSize = CCSize;

				pCCmax2 = pCC;
			}
		}

		nCCmax2 = CCmaxSize / sizeof(RVL3DPOINT2 *);

		CCList2.RemoveAll();

		// create new region and add it to the region set

		pSegment = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet));

		pSegment->m_PtArray = (void **)(m_pMem->Alloc(nCCmax2 * sizeof(RVL3DPOINT2 *)));

		memcpy(pSegment->m_PtArray, pCCmax2->pFirst, nCCmax2 * sizeof(RVL3DPOINT2 *));

		pSegment->m_n3DPts = nCCmax2;

		if(Flags & RVLPSDRANSAC_FLAG_LAD)
			LADPlaneThreePoints((RVL3DPOINT2 **)(pCCmax2->pFirst), nCCmax2, &Plane);
		else
			LSPlane((RVL3DPOINT2 **)(pCCmax2->pFirst), nCCmax2, &Plane);

		pSegment->m_a = Plane.m_a / 16.0;
		pSegment->m_b = Plane.m_b / 16.0;
		pSegment->m_c = Plane.m_c / 16.0 + d0;

		pSegment->m_Tol = m_Tol;

		iSegment++;

		if(Flags & RVLPSD_FLAG_CONNECTED_COMPONENTS)
		{ 
			// get connected components with at least m_minSegmentSize points in the remaining of pCCmax

			ClearRegionGrowingMap();

			Mask((RVL3DPOINT2 **)(pCCmax2->pFirst), nCCmax2, 1);

			ppPt = (RVL3DPOINT2 **)(pCCmax->pFirst);

			ppPt2 = Point3DPtrArray;

			for(; ppPt < pCCEnd; ppPt++)
			{
				pPt = *ppPt;

				iPix = pPt->iPix;

				if(RegionGrowingMap[iPix] == 1)
					RegionGrowingMap[iPix] = 0xffffffff;
				else
				{				
					*(ppPt2++) = pPt;

					RegionGrowingMap[iPix] = 0;
				}
			}		

			n = ppPt2 - Point3DPtrArray;

			if(n < m_minSegmentSize)
			{
				// remove pCCmax from CCList

				pCCmax->pEnd = pCCmax->pFirst;

				continue;
			}

			GetConnectedComponents(Point3DPtrArray, n, &CCList, 
				(Flags & RVLPSDGCC_FLAG_DILATE));
				//| RVLPSDGCC_FLAG_MASK, 1);

			// remove pCCmax from CCList

			pCCmax->pEnd = pCCmax->pFirst;
		}	// if(Flags & RVLPSD_FLAG_CONNECTED_COMPONENTS)
		else
		{
			ppPt = (RVL3DPOINT2 **)(pCCmax->pFirst);

			ppPt2 = m_CCBuff;

			for(; ppPt < pCCEnd; ppPt++)
			{
				pPt = *ppPt;

				if(pPt->segmentNumber < 0)
					*(ppPt2++) = pPt;
			}		

			n = ppPt2 - m_CCBuff;

			if(n < m_minSegmentSize)
			{
				// remove pCCmax from CCList

				pCCmax->pEnd = pCCmax->pFirst;

				continue;
			}
			else
				pCCmax->pEnd = (BYTE *)ppPt2;
		}
	}	// main loop

	m_pMem2->Clear();

	delete[] Point3DPtrArray;
}


//Planar surface segmentation using 2D image segments as initial region
void CRVLPlanarSurfaceDetector::SegmentRB(CRVLC2D *p2DRegionSet,		// Input/Output: input image 2D segments   
																		// corresponding dominant planar surfaces
										 DWORD Flags)
{
	double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
		(m_pStereoVision->m_pCameraL->CenterXNrm - 
		m_pStereoVision->m_pCameraR->CenterXNrm);
	
	RVL3DPOINT2 **Point3DPtrArray = new RVL3DPOINT2 *[m_n3DPoints];

	RVL3DPOINT2 **ppPt = Point3DPtrArray;

	//RVL3DPOINT2 *pPt;
	RVL3DPOINT2 *pPoint3D;

	int index;
	int *ptr;
	
	//RVL3DPOINT2 *Point3DArrayEnd = m_Point3DArray + m_n3DPoints;

	CRVL2DRegion2 Plane;
	//double a, b, c, ePlane;
	RVLPSDLAD_SEGMENT_DETAILS segmentDetails;
	int n;

	CRVL2DRegion2 *p2DRegion;
	//Get segment list
	CRVLMPtrChain regionList = p2DRegionSet->m_ObjectList;
	

	//For each segment  (main loop)
	regionList.Start();
	while(regionList.m_pNext)
	{
		//reset pointer
		ppPt = Point3DPtrArray;

		//get segment
		p2DRegion = (CRVL2DRegion2 *)(regionList.GetNext());
		ptr = (int *)p2DRegion->m_PtArray;


		//Get 3D points in segment
		for (int i=0; i< p2DRegion->m_nPts; i++,ptr++)
		{
			index = (*ptr);

			//get 3D point
			pPoint3D = m_Point3DMap[index];
			
			if(pPoint3D != NULL)
			{
				if(Flags & RVLPSD_FLAG_SUBSAMPLE)
				{
					if(pPoint3D->u % m_SubSample)
					continue;

					if(pPoint3D->v % m_SubSample)
						continue;

					*(ppPt++) = pPoint3D;
				}
				else
				{
					*(ppPt++) = pPoint3D;
				}
			}
		
		}//for (int i=0; i< p2DRegion->m_nPts;

		n = ppPt - Point3DPtrArray;

		p2DRegion->m_n3DPts = n;

		p2DRegion->m_Tol = m_Tol;

		// find dominant planar surface
		if(n>0)
		{
			RANSAC(p2DRegion, false, Point3DPtrArray, n, &segmentDetails, 
				   Flags | RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET);

			p2DRegion->m_a /= 16.0;
			p2DRegion->m_b /= 16.0;
			p2DRegion->m_c = p2DRegion->m_c/16.0 + d0;
		}
		
	} // while(p2DRegionList->m_pNext)



	//m_pMem2->Clear();

	delete[] Point3DPtrArray;
}

//Planar surface segmentation using 2D image segments as initial region
//Implements RANSAC on bigger regions and then tries to add neighbouriing regions to create super segments/regions
//Read SIP paper for details

void CRVLPlanarSurfaceDetector::SegmentRB2(CRVLC2D *p2DRegionSet,		// Input/Output: input image 2D segments  
										   CRVLC2D *p2DRegionSet2,		// Input/Output: input image 2D super segments  
																		// corresponding dominant planar surfaces
										 DWORD Flags)
{
	p2DRegionSet2->m_ObjectList.RemoveAll();

	double minPerc = 0.5;  //determines the min. percentage of points that a neighbour segment 
						   //must have in order to be added to the main segment

	double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
		(m_pStereoVision->m_pCameraL->CenterXNrm - 
		m_pStereoVision->m_pCameraR->CenterXNrm);

	RVL3DPOINT2 **Point3DPtrArray = new RVL3DPOINT2 *[2 * m_n3DPoints];  //contains all the 3D points for all the segments
	RVL3DPOINT2 **ppPt;
	RVL3DPOINT2 **ppPt0;	//ppPt0 -> points to the initial position of the buffer for a particular region
	
	RVL3DPOINT2 *pPoint3D;

	int index;
	int *ptr;
	int i,j;
	
	int n3DPoints = 0, nPoints = 0;;
	int nMax = 0;
	
	CRVLMPtrChain **SortedSegmentBuffer = new CRVLMPtrChain *[m_n3DPoints];
	memset(SortedSegmentBuffer, 0x00, m_n3DPoints * sizeof(CRVLMPtrChain *));
	CRVLMPtrChain *pSortedBin;

	CRVLMPtrChain SortedBinTemplate;
	SortedBinTemplate.m_pMem = m_pMem2;

	CRVL2DRegion2 *p2DRegion, *p2DRegionHelper;

	CRVL2DRegion2 **pp2DSuperSegment;

	//Get segment list
	CRVLMPtrChain regionList = p2DRegionSet->m_ObjectList;
	
	/*********************************************************/
	/* 1. Find number of 3D points for each segment and sort */
	/*********************************************************/
	
	ppPt = Point3DPtrArray;

	
	//For each segment  (main loop)
	regionList.Start();

	while(regionList.m_pNext)
	{
		//point to initial position
		ppPt0 = ppPt;

		//get segment
		p2DRegion = (CRVL2DRegion2 *)(regionList.GetNext());
		ptr = (int *)p2DRegion->m_PtArray;


		//Get 3D points in segment
		for (i=0; i< p2DRegion->m_nPts; i++,ptr++)
		{
			index = (*ptr);

			//get 3D point
			pPoint3D = m_Point3DMap[index];
			
			if(pPoint3D != NULL)
			{
				if(Flags & RVLPSD_FLAG_SUBSAMPLE)
				{
					if(pPoint3D->u % m_SubSample)
					continue;

					if(pPoint3D->v % m_SubSample)
						continue;

					*(ppPt++) = pPoint3D;
				}
				else
				{
					*(ppPt++) = pPoint3D;
				}
			}
		
		}//for (int i=0; i< p2DRegion->m_nPts;
		
		n3DPoints = ppPt - ppPt0;

		if(n3DPoints >nMax)
			nMax = n3DPoints;

		//set number of points
		p2DRegion->m_n3DPts = n3DPoints;
		
		//set region pointer
		p2DRegion->m_pPoint3DArray = ppPt0;

		//reset flag (RANSAC not yet performed)
		p2DRegion->m_Flags &= ~RVL2DREGION_FLAG_MARKED;
		//reset flag (Final fitting not yet performed)
		p2DRegion->m_Flags &= ~RVL2DREGION_FLAG_MARKED2;

		//Set pointer to super segment to NULL
		pp2DSuperSegment = (CRVL2DRegion2 **)(p2DRegion->m_pData + p2DRegionSet->m_iDataParentPtr);
		*pp2DSuperSegment = NULL;


		//store region in SortedSegmentBuffer depending on n3DPoints
		//1. get position 
		pSortedBin = SortedSegmentBuffer[n3DPoints];

		if(pSortedBin == NULL)
		{
			pSortedBin = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
			*pSortedBin = SortedBinTemplate;

			SortedSegmentBuffer[n3DPoints] = pSortedBin;
		}
		

		//2. Add region
		pSortedBin->Add(p2DRegion);
		
	}

	/*************************************************************/
	/* 2. Iterate through SortedSegmentBuffer and perform RANSAC */
	/*************************************************************/

	RVL3DPOINT2 **Point3DArray = new RVL3DPOINT2 *[5*m_n3DPoints];   //contains 3D points of segment + neighbours
	RVL3DPOINT2 **Point3DArrayNeighbour = new RVL3DPOINT2 *[m_n3DPoints];   //contains 3D points of neighbour

	//contains set of grouped (sub) segments for RANSAC
	CRVL2DRegion2 **SubRegionList = (CRVL2DRegion2 **)(m_pMem->Alloc(p2DRegionSet->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	memset(SubRegionList, 0x00, p2DRegionSet->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));
	
	CRVL2DRegion2 **ppSubRegionList,**ppSubRegionListHelper, **ppSubRegionStart,  **ppSubRegion;

	int nList;

	//RVL3DPOINT2 **ppPtMain;
	//RVL3DPOINT2 **ppPtNeighbour;
	RVL3DPOINT2 **pConsensusSet;
	
	//RVL3DPOINT2 **ConsensusSet;
	int nConsensus;
	double Score, ScorePerc;
	double BestScore = 0.0;

	CRVLMPtrChain *pSortedBinNew;
	
	CRVLRelation *pRelation;
	CRVLMPtrChain2 RelList;
	CRVL2DRegion2 *p2DRegionNeighbor;
	CRVL2DRegion2 *p2DSegment, *p2DSuperSegment;
	RVLARRAY *pRelList;

	RVLPSDLAD_SEGMENT_DETAILS segmentDetails;
	int nBest = 0;

	//RVL2DREGION_FLAG_MARKED -> indicates RANSAC+sorting has been performed
	//RVL2DREGION_FLAG_MARKED2 -> indicates final fitting has been performed

	ppSubRegionList = SubRegionList;
	ppSubRegionListHelper = SubRegionList;
	
	for(i = nMax; i >= 3; i--)  //No need to go below 3 points, cannot determine plane
	{
		pSortedBin = SortedSegmentBuffer[i];
		
		if(pSortedBin == NULL)
			continue;
		
		pSortedBin->Start();
		while(pSortedBin->m_pNext)
		{
			//get segment
			p2DRegion = (CRVL2DRegion2 *)(pSortedBin->GetNext());

			//a. Make sure it has not been marked (final fitting has not been performed)
			//b. Make sure it has 3D points before performing RANSAC  (This is an unnecessary check if i>=3 otherwise it is!!)
			if(((p2DRegion->m_Flags & RVL2DREGION_FLAG_MARKED2)==0) && (p2DRegion->m_n3DPts > 0)) 
			{
				//RANSAC and 2nd sorting has not been performed
				if((p2DRegion->m_Flags & RVL2DREGION_FLAG_MARKED)==0) 
				{
					
						nBest = RANSAC(p2DRegion, false, p2DRegion->m_pPoint3DArray, p2DRegion->m_n3DPts, &segmentDetails, 
								Flags | RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET);


						//Set flag
						p2DRegion->m_Flags |= RVL2DREGION_FLAG_MARKED;

						//move p2DRegion lower if nBest < p2DRegion->m_n3DPts
						if(nBest < p2DRegion->m_n3DPts)
						{
							pSortedBinNew = SortedSegmentBuffer[nBest];

							if(pSortedBinNew == NULL)
							{
								pSortedBinNew = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
								*pSortedBinNew = SortedBinTemplate;

								SortedSegmentBuffer[nBest] = pSortedBinNew;
							}

							pSortedBinNew->Add(p2DRegion);
						}
					
				} 
				//Expand marked regions (RANSAC and 2nd sorting has been performed)
				else 
				{
					//check neighbouring regions and add if minPerc of the number of points in the neighbor is within the error
					//mark neighbour if it is included
					//perform LS fitting on all the points

					

					//1. Reset temp buffer and fill with current 3D points
					/*ppPtMain = Point3DArray;

					for(j = 0; j < p2DRegion->m_n3DPts; j++)
					{
						*(ppPtMain++) = p2DRegion->m_pPoint3DArray[j];
					}*/
					
					// reset ConsensusSet pointer
					//pConsensusSet = ppPtMain;

					// reset ConsensusSet pointer
					pConsensusSet = Point3DArray;

					Consensus(p2DRegion->m_pPoint3DArray, p2DRegion->m_n3DPts, p2DRegion, pConsensusSet, nConsensus, Flags, BestScore);

					pConsensusSet += nConsensus;

					//Create super segment
					p2DSuperSegment = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet2));

					p2DSuperSegment->m_a = p2DRegion->m_a;
					p2DSuperSegment->m_b = p2DRegion->m_b;
					p2DSuperSegment->m_c = p2DRegion->m_c;

					p2DSuperSegment->m_Tol = m_Tol;


					//2. Reset beginning of sub region and add main segment
					ppSubRegionStart = ppSubRegionList;  

					ppSubRegionListHelper = ppSubRegionList;

					p2DRegion->m_Flags |= RVL2DREGION_FLAG_MARKED2;
					*(ppSubRegionList++) = p2DRegion;
					
					while(ppSubRegionListHelper < ppSubRegionList)
					{
						p2DRegionHelper = *ppSubRegionListHelper;
					
						//3. Get neighbours
						RelList.Create(p2DRegionHelper->m_RelList + p2DRegionHelper->m_pClass->m_iRelListNeighbors, NULL);

						RelList.Start();

						while(RelList.m_pNext)
						{
							pRelation = (CRVLRelation *)(RelList.GetNext());
						
							p2DRegionNeighbor = (CRVL2DRegion2 *)(pRelation->m_pObject[0]);

							if(p2DRegionNeighbor == p2DRegionHelper)
								p2DRegionNeighbor = (CRVL2DRegion2 *)(pRelation->m_pObject[1]);


							//Make sure neighbour has not been marked (ie final fitting has not already been performed)
							//Neighbour must have 3D points
							if(((p2DRegionNeighbor->m_Flags & RVL2DREGION_FLAG_MARKED2)!=0) ||  (p2DRegionNeighbor->m_n3DPts == 0))// 
								continue;

							//Check consensus for neighbour using plane parameters of main segment
								
							/*ppPtNeighbour = Point3DArrayNeighbour;

							for(j = 0; j < p2DRegionNeighbor->m_n3DPts; j++)
							{
								*(ppPtNeighbour++) = p2DRegionNeighbor->m_pPoint3DArray[j];
							}*/

							//Get Score  
							Score = Consensus(p2DRegionNeighbor->m_pPoint3DArray, p2DRegionNeighbor->m_n3DPts, p2DRegion, pConsensusSet, nConsensus, Flags, BestScore);

							ScorePerc = Score/((double)(p2DRegionNeighbor->m_n3DPts));

							//Add neighbour 3D points from COnsenus set to Point3DArray and add neighbour to RegionList
							if(ScorePerc > minPerc)
							{
								//set/move consensus set pointer
								pConsensusSet += nConsensus;

								p2DRegionNeighbor->m_Flags |= RVL2DREGION_FLAG_MARKED2;

								*(ppSubRegionList++) = p2DRegionNeighbor;

							}
							

						}//while RelList.m_pNext


						ppSubRegionListHelper++;
					}
					
					n3DPoints = pConsensusSet - Point3DArray;

					pRelList = p2DSuperSegment->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_COMPONENTS];

					//updating segment params
					pRelList->pFirst = (unsigned char *)ppSubRegionStart;
					pRelList->pEnd = (unsigned char *)ppSubRegionList;

					nList = ppSubRegionList - ppSubRegionStart;

					//perform final fitting 
					if(nList > 1)
						LSPlane(Point3DArray, n3DPoints, p2DSuperSegment);

					nPoints = 0;

					ppSubRegion = ppSubRegionStart;
					//updating sub segment params
					for(j = 0; j < nList; j++,ppSubRegion++)
					{
						p2DSegment = *ppSubRegion; //SubRegionList[j];

						nPoints += p2DSegment->m_nPts;

						pp2DSuperSegment = (CRVL2DRegion2 **)(p2DSegment->m_pData + p2DRegionSet->m_iDataParentPtr);

						*pp2DSuperSegment = p2DSuperSegment;
					}
					

					p2DSuperSegment->m_n3DPts = n3DPoints;
					p2DSuperSegment->m_nPts = nPoints;

					p2DSuperSegment->m_a /= 16.0;
					p2DSuperSegment->m_b /= 16.0;
					p2DSuperSegment->m_c = p2DSuperSegment->m_c/16.0 + d0;
				
				} //expand marked regions
				
			}//if(p2DRegion->m_n3DPts > 0) 
		
		}//while(pSortedBin->m_pNext)

	}//for(i = nMax; i >= 3; i--) 

	m_pMem2->Clear();

	delete[] Point3DPtrArray;
	delete[] SortedSegmentBuffer;
	delete[] Point3DArrayNeighbour;
	delete[] Point3DArray;
	
}

//returns 3Dpoints that have !(pPoint3D->Flags & RVLOBJ2_FLAG_MARKED)
int GetUnassigned3DPoints(RVL3DPOINT2 **CurrentSegment3DPtrArray, RVL3DPOINT2 **Temp3DPtrArray, int n3DPoints)
{
	int nUnassigned3DPts = 0;
	RVL3DPOINT2 *pPoint3D;
	RVL3DPOINT2 **ppTempPt = Temp3DPtrArray;
	

	for(int i=0; i<n3DPoints; i++ )
	{
		pPoint3D = CurrentSegment3DPtrArray[i];
		if(!(pPoint3D->Flags & RVLOBJ2_FLAG_MARKED))
		{					
			*(ppTempPt++) = pPoint3D;
			nUnassigned3DPts++;
		}
	}

	return nUnassigned3DPts;

}


void CopyPlanarSegmentDetails(CRVL2DRegion2 *p2DRegion, RVLPSD_SEGMENT_DETAILS *pPlanarSegmentDetails, RVL3DPOINT2 **ppPtrArrayStart)
{
	RVL3DPOINT2 *pPoint3D;
	RVL3DPOINT2 **ppPt;

	p2DRegion->m_a = pPlanarSegmentDetails->a;
	p2DRegion->m_b = pPlanarSegmentDetails->b;
	p2DRegion->m_c = pPlanarSegmentDetails->c;
	
	//Set pointer
	p2DRegion->m_pPoint3DArray = ppPtrArrayStart;
	
	ppPt = p2DRegion->m_pPoint3DArray;
	
	pPlanarSegmentDetails->m_ObjectList.Start();

	while(pPlanarSegmentDetails->m_ObjectList.m_pNext)
	{
		pPoint3D = (RVL3DPOINT2 *)(pPlanarSegmentDetails->m_ObjectList.GetNext());

		//add point if it has not already  been assigned to a sub-segment
		if (!(pPoint3D->Flags & RVLOBJ2_FLAG_MARKED3))
		{
			//mark point as having been finally added to a plane
			pPoint3D->Flags |= RVLOBJ2_FLAG_MARKED3;

			//add segment to  the points region list
			pPoint3D->regionList.Add(p2DRegion);
			
			*(ppPt++) = pPoint3D;
		}

	}
	
	p2DRegion->m_n3DPts = ppPt - p2DRegion->m_pPoint3DArray;

	
}

//Planar surface segmentation using 2D image segments as initial region
//Divides image segments into planar surfaces/regions

void CRVLPlanarSurfaceDetector::SegmentRB3(CRVLC2D *p2DRegionSet,		// Input/Output: input image 2D segments  
										   CRVLC2D *p2DRegionSet2,
										   DWORD Flags)
{
	//delete all super segments
	p2DRegionSet2->m_ObjectList.RemoveAll();

	RVL3DPOINT2 **Point3DPtrArray = (RVL3DPOINT2 **)(m_pMem->Alloc(10 * m_n3DPoints * sizeof(RVL3DPOINT2))); //new RVL3DPOINT2 *[10 * m_n3DPoints];  //contains all the 3D points for all the regions
	RVL3DPOINT2 **ppPtrArrayStart;	//points to the initial position of the buffer for the 3D of a given region

	RVL3DPOINT2 **ppPt;
	
	RVL3DPOINT2 *pPoint3D;

	int i,j,index;
	int *ptr;
		
	int n3DPoints = 0, nPoints = 0;;
	int nMax = 0;
	
	CRVLMPtrChain2 RelList;

	int nOriginalRegions = p2DRegionSet->m_ObjectList.m_nElements;

	CRVL2DRegion2 **RegionBuffer = new CRVL2DRegion2 *[nOriginalRegions];
	memset(RegionBuffer, 0x00, nOriginalRegions * sizeof(CRVL2DRegion2 *));
	CRVL2DRegion2 **ppRegionBuffer;

	ppRegionBuffer = RegionBuffer;

	CRVL2DRegion2 *p2DRegion;  
	CRVL2DRegion2 **pp2DSuperSegment;


	//Get region list
	CRVLMPtrChain regionList = p2DRegionSet->m_ObjectList;
	
	/*****************************************************************/
	/* 1. Find number of 3D points for each segment and add to buffer*/
	/*****************************************************************/
	ppPt = Point3DPtrArray;

	//For each region  (main loop)
	regionList.Start();

	while(regionList.m_pNext)
	{
		//point to initial position
		ppPtrArrayStart = ppPt;

		//get segment
		p2DRegion = (CRVL2DRegion2 *)(regionList.GetNext());
		ptr = (int *)p2DRegion->m_PtArray;

		//Get 3D points in segment
		for (i=0; i< p2DRegion->m_nPts; i++,ptr++)
		{
			index = (*ptr);

			//get 3D point
			pPoint3D = m_Point3DMap[index];
			
			if(pPoint3D != NULL)
			{
				if(Flags & RVLPSD_FLAG_SUBSAMPLE)
				{
					if(pPoint3D->u % m_SubSample)
					continue;

					if(pPoint3D->v % m_SubSample)
						continue;

					pPoint3D->regionList.m_pMem =  m_pMem2;
					//reset values
					pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED;
					pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED2;
					pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED3;
					*(ppPt++) = pPoint3D;
				}
				else
				{
					pPoint3D->regionList.m_pMem =  m_pMem2;
					//reset values
					pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED;
					pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED2;
					pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED3;
					*(ppPt++) = pPoint3D;
				}
			}
		
		}//for (int i=0; i< p2DRegion->m_nPts;
		
		n3DPoints = ppPt - ppPtrArrayStart;

		//set number of points
		p2DRegion->m_n3DPts = n3DPoints;
		
		//set region pointer
		p2DRegion->m_pPoint3DArray = ppPtrArrayStart;

		//Set pointer to super segment to NULL
		pp2DSuperSegment = (CRVL2DRegion2 **)(p2DRegion->m_pData + p2DRegionSet->m_iDataParentPtr);
		*pp2DSuperSegment = NULL;


		//RESET Relation List !!!!!!!!!!!!!!!!!!!!
		RelList.Create(p2DRegion->m_RelList + p2DRegion->m_pClass->m_iRelListNeighbors, m_pMem);
		
		//Clear relation List
		RelList.RemoveAll();



		//store region in RegionBuffer
		*(ppRegionBuffer++) = p2DRegion;
		
	}

	//point to final position so that new segments can be added
	ppPtrArrayStart = ppPt;


	/***********************************************************************/
	/* 2. Iterate through RegionBuffer and generate planar surfaces */
	/***********************************************************************/

	BYTE *pDelaunayData;
	short nNeighbors,iNeighbor;
	RVLMESH_LINK *pConnection;
	int iPix;

	RVL3DPOINT2 **CurrentSegment3DPtrArray = new RVL3DPOINT2 *[m_n3DPoints]; //contains all the 3D points for a given segment
	RVL3DPOINT2 **Unassigned3DPtrArray = new RVL3DPOINT2 *[m_n3DPoints];	 //contains all the 3D points for a given segment that have NOT been assigned to a plane
	RVL3DPOINT2 **Assigned3DPtrArray = new RVL3DPOINT2 *[2 * m_n3DPoints];	 //contains all the 3D points for a given segment that have been assigned to a plane
	RVL3DPOINT2 **ppTempPt, **ppTempPtHelper;
	
	CRVL2DRegion2 planarRegion;

	RVL3DPOINT2 *pPoint3D1;
	RVL3DPOINT2 *pPoint3D2;
	RVL3DPOINT2 *pPoint3D3;
	int nUnassigned3DPts = 0;
	int nCurrentSegment3DPoints = 0;
			
	bool bSecondPt = false;
	bool bThirdPt = false;
	bool bAll3PointsInPlane = false;

	bool bInitial = false;

	double e;
	
	CRVLMPtrChain **TempSortedSegmentBuffer = new CRVLMPtrChain *[m_n3DPoints];
	memset(TempSortedSegmentBuffer, 0x00, m_n3DPoints * sizeof(CRVLMPtrChain *));
	
	CRVLMPtrChain *pTempSortedBin, *pTempSortedBinNew;

	CRVLMPtrChain TempSortedBinTemplate;
	TempSortedBinTemplate.m_pMem = m_pMem2;
	int nMax2 = 0;

	RVLPSD_SEGMENT_DETAILS *PlanarSegmentDetails = new RVLPSD_SEGMENT_DETAILS[m_n3DPoints];
	memset(PlanarSegmentDetails, 0x00, m_n3DPoints * sizeof(RVLPSD_SEGMENT_DETAILS *));
	RVLPSD_SEGMENT_DETAILS *pPlanarSegmentDetails;

	pPlanarSegmentDetails = PlanarSegmentDetails;
	
	
	RVLPSD_SEGMENT_DETAILS *pSegmentDetails;
	CRVL2DRegion2 *pNewSegment;
	
	int nTrials = 0;
	int nUnassigned3DPts0 = m_n3DPoints;
	
	//RVL3DPOINT2->Flags = RVLOBJ2_FLAG_MARKED  implies that the point cannot be used as a FIRST/STARTING point
	//RVL3DPOINT2->Flags = RVLOBJ2_FLAG_MARKED2 implies that a point has been initially assigned to a plane when using Delaunay links
	//RVL3DPOINT2->Flags = RVLOBJ2_FLAG_MARKED3 implies that a point has been finally assigned to a plane

	//reset pointer to RegionBuffer
	ppRegionBuffer = RegionBuffer;

	for(i = 0; i < nOriginalRegions; i++,ppRegionBuffer++)   //Go through all original regions
	{
		//get region
		p2DRegion =  *ppRegionBuffer;

		if(p2DRegion->m_n3DPts > 20) //No need to go below 20 points
		{

			nCurrentSegment3DPoints = p2DRegion->m_n3DPts;

			//copy all its 3d points to buffer
			memcpy(CurrentSegment3DPtrArray,p2DRegion->m_pPoint3DArray,nCurrentSegment3DPoints * sizeof(RVL3DPOINT2 *));

			//mark all 3D points belonging to segment (default is -1)
			ppPt = CurrentSegment3DPtrArray;
			for(j=0; j<nCurrentSegment3DPoints; j++,ppPt++)
				(*ppPt)->refSegmentNumber = 0;
				

			//initialize temp array
			nUnassigned3DPts = GetUnassigned3DPoints(CurrentSegment3DPtrArray,Unassigned3DPtrArray,nCurrentSegment3DPoints);

			//reset sort buffer
			memset(TempSortedSegmentBuffer, 0x00, m_n3DPoints * sizeof(CRVLMPtrChain *));
			
			nMax2 = 0;

			nTrials = 0;
			nUnassigned3DPts0 = m_n3DPoints;


			//Get segment planes
			while(nUnassigned3DPts>0)
			{
				//Get random 3 points
				//1st point from unassigned points
				pPoint3D1 = *(Unassigned3DPtrArray + RVLRandom(0, nUnassigned3DPts - 1));

				//2nd point from the whole set
				do
				{
					pPoint3D2 = *(CurrentSegment3DPtrArray + RVLRandom(0, nCurrentSegment3DPoints - 1));
				}
				while (pPoint3D2==pPoint3D1);
				
				//3rd point from the whole set
				do
				{
					pPoint3D3 = *(CurrentSegment3DPtrArray + RVLRandom(0, nCurrentSegment3DPoints - 1));
				}
				while ((pPoint3D3==pPoint3D1) || (pPoint3D3==pPoint3D2));


				//check to see that nUnassigned3DPts is changing;
				if(nUnassigned3DPts < nUnassigned3DPts0)
					nUnassigned3DPts0 = nUnassigned3DPts;
				else
					nTrials++;

				//exit loop if nUnassigned3DPts does not change in 10 trials
				if (nTrials > 10)
					break;

				//Get Plane
				if(!Plane(pPoint3D1, pPoint3D2, pPoint3D3, &planarRegion))
					continue;
				
				
				//Define planar segment
				
				pPlanarSegmentDetails->a = planarRegion.m_a;
				pPlanarSegmentDetails->b = planarRegion.m_b;
				pPlanarSegmentDetails->c = planarRegion.m_c;
				pPlanarSegmentDetails->m_ObjectList.m_pMem =  m_pMem2;

				//reset values
				bSecondPt = false;
				bThirdPt = false;

				//reset pointers
				ppTempPt = Assigned3DPtrArray;
				ppTempPtHelper = ppTempPt;


				//add first point
				pPoint3D1->Flags |= RVLOBJ2_FLAG_MARKED;
				pPoint3D1->Flags |= RVLOBJ2_FLAG_MARKED2;
				pPlanarSegmentDetails->m_ObjectList.Add(pPoint3D1);
				*(ppTempPt++) = pPoint3D1;
				


				//Get all neighbouring points using Delaunay links
				while(ppTempPtHelper<ppTempPt)
				{
					pPoint3D = *(ppTempPtHelper);
										
					pDelaunayData = m_pDelaunay->m_DelaunayMap[pPoint3D->iPix];

					nNeighbors = *((short *)pDelaunayData);

					pConnection = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

					for(iNeighbor = 0; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
					{
						iPix = pConnection->iPix;
					
						//get 3D point
						pPoint3D = m_Point3DMap[iPix];
						
						//make sure it is in the same segment and that it has not already been added
						if((pPoint3D != NULL) && (pPoint3D->refSegmentNumber == 0) && !(pPoint3D->Flags & RVLOBJ2_FLAG_MARKED2))
						{
							
							//determine tolerance and add to TempArray
							e = pPoint3D->z - (pPlanarSegmentDetails->a * pPoint3D->x + pPlanarSegmentDetails->b * pPoint3D->y + pPlanarSegmentDetails->c);

							if(e >= -m_Tol)
							{
								pPoint3D->Flags |= RVLOBJ2_FLAG_MARKED;
								pPoint3D->Flags |= RVLOBJ2_FLAG_MARKED2;
								
								pPlanarSegmentDetails->m_ObjectList.Add(pPoint3D);

								*(ppTempPt++) = pPoint3D;

								if(pPoint3D == pPoint3D2)
									bSecondPt = true;

								if(pPoint3D == pPoint3D3)
									bThirdPt = true;
							}
						}

						
					}
					ppTempPtHelper++;
				}

				//bAll3PointsInPlane = (bSecondPt && bThirdPt);
				bAll3PointsInPlane = TRUE;
				
				//Second and Third point belong to consensus set -> Save the plane properties and add into SortBuffer
				if(bAll3PointsInPlane)
				{
					n3DPoints = ppTempPt - Assigned3DPtrArray;

					//store max value
					if(n3DPoints > nMax2)
						nMax2 = n3DPoints;

					pPlanarSegmentDetails->numberOf3DPoints = n3DPoints;

					//store planarSegmentDetails in TempSortedSegmentBuffer depending on n3DPoints
					//1. get position 
					pTempSortedBin = TempSortedSegmentBuffer[n3DPoints];

					if(pTempSortedBin == NULL)
					{
						pTempSortedBin = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
						*pTempSortedBin = TempSortedBinTemplate;

						TempSortedSegmentBuffer[n3DPoints] = pTempSortedBin;
					}
					
					//2. Add planarSegmentDetails
					pTempSortedBin->Add(pPlanarSegmentDetails);

					//Fill temp array
					nUnassigned3DPts = GetUnassigned3DPoints(CurrentSegment3DPtrArray,Unassigned3DPtrArray,nCurrentSegment3DPoints);
					 
				}
				
				//reset point3D flags
				pPlanarSegmentDetails->m_ObjectList.Start();
				while(pPlanarSegmentDetails->m_ObjectList.m_pNext)
				{
					pPoint3D = (RVL3DPOINT2 *)(pPlanarSegmentDetails->m_ObjectList.GetNext());
					pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED2; //reset

					if(!bAll3PointsInPlane)
						pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED; // point can be used as a FIRST/STARTING point in the future

					
				}
				
						

				//set new SegmentDetail
				if(bAll3PointsInPlane)
					pPlanarSegmentDetails++;
				else
					pPlanarSegmentDetails->m_ObjectList.RemoveAll();
			
			}
			

			bInitial = false;
			
			//sort planes and create new sub segments
			for(j = nMax2; j > 0; j--)  
			{
				pTempSortedBin = TempSortedSegmentBuffer[j];
				
				if(pTempSortedBin == NULL)
					continue;

				pTempSortedBin->Start();
				while(pTempSortedBin->m_pNext)
				{
					//get segment
					pSegmentDetails = (RVLPSD_SEGMENT_DETAILS *)(pTempSortedBin->GetNext());
					
					if((j == nMax2) && (bInitial == false)) //copy biggest plane to current p2DRegion (if there is more than one copy the first one)
					{
						bInitial = true;
						
						CopyPlanarSegmentDetails(p2DRegion,pSegmentDetails,ppPtrArrayStart);

						//adjust pointer for next sub segment
						ppPtrArrayStart += p2DRegion->m_n3DPts;
						
					}
					else //check number of free points and re-sort 
					{
						//check current count
						n3DPoints = 0;
						
						pSegmentDetails->m_ObjectList.Start();
						while(pSegmentDetails->m_ObjectList.m_pNext)
						{
							pPoint3D = (RVL3DPOINT2 *)(pSegmentDetails->m_ObjectList.GetNext());
							if(pPoint3D->regionList.m_nElements == 0)
								n3DPoints++;
						}

						//move p2DRegion  if n3DPoints != pPlanarSegmentDetails2->m_n3DPts
						if(n3DPoints != pSegmentDetails->numberOf3DPoints)
						{
							pSegmentDetails->numberOf3DPoints = n3DPoints;
							
							pTempSortedBinNew = TempSortedSegmentBuffer[n3DPoints];

							if(pTempSortedBinNew == NULL)
							{
								pTempSortedBinNew = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
								*pTempSortedBinNew = TempSortedBinTemplate;

								TempSortedSegmentBuffer[n3DPoints] = pTempSortedBinNew;
							}

							pTempSortedBinNew->Add(pSegmentDetails);
						}
						else //create new 2DRegion and add to current
						{	
							//add segment to RegionSet
							pNewSegment = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet));

							//Set pointer to super segment to NULL
							pp2DSuperSegment = (CRVL2DRegion2 **)(pNewSegment->m_pData + p2DRegionSet->m_iDataParentPtr);
							*pp2DSuperSegment = NULL;

							CopyPlanarSegmentDetails(pNewSegment,pSegmentDetails,ppPtrArrayStart);

							//adjust pointer for next sub segment
							ppPtrArrayStart += pNewSegment->m_n3DPts;
							
						}
						
						}
						
					}
					
				}
			}


			//reset 3D points belonging to segment (default is -1)
			ppPt = CurrentSegment3DPtrArray;
			for(j=0; j<nCurrentSegment3DPoints; j++,ppPt++)
			{
				(*ppPt)->refSegmentNumber = -1;
				(*ppPt)->Flags &= ~RVLOBJ2_FLAG_MARKED;
				(*ppPt)->Flags &= ~RVLOBJ2_FLAG_MARKED2;
				(*ppPt)->Flags &= ~RVLOBJ2_FLAG_MARKED3;
			}

	}

	

	
	delete[] RegionBuffer;
	
	delete[] TempSortedSegmentBuffer;
	delete[] Unassigned3DPtrArray;
	delete[] Assigned3DPtrArray;
	delete[] CurrentSegment3DPtrArray;
	delete[] PlanarSegmentDetails;


#ifdef NEVER
	/*********************************************************/
	/* 3. Assign region ptrs. to Delaunay links              */
	/*********************************************************/
	
	BYTE **DelaunayMap = m_pDelaunay->m_DelaunayMap;

	// set vpRegion of every border connection to 0xffffffff

	pDelaunayData = DelaunayMap[0];

	nNeighbors = *((short *)pDelaunayData);

	RVLMESH_LINK *ConnectionArray = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

	pConnection = ConnectionArray;

	while(pConnection->iPix != m_Width)
		pConnection++;
	
	short iConnection2 = pConnection - ConnectionArray;

	RVLMESH_LINK *pConnection2 = pConnection;

	do
	{
		pConnection2->vpRegion = (void *)(0xffffffff);

		pConnection2 = ConnectionArray + ((iConnection2 + 1) % nNeighbors);

		pDelaunayData = DelaunayMap[pConnection2->iPix];

		nNeighbors = *((short *)pDelaunayData);

		ConnectionArray = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		iConnection2 = pConnection2->iConnection;

		pConnection2 = ConnectionArray + iConnection2;
	}
	while(pConnection2 != pConnection);

	// set vpRegion pointers of other connections

	void **RegionPtrBuff = new void *[4 * (m_Width * m_Height)];

	void **RegionPtrBuff1 = RegionPtrBuff;
	void **RegionPtrBuff2 = RegionPtrBuff + 2 * (m_Width * m_Height);

	RVL3DPOINT2 *pPoint3DArrayEnd = m_Point3DArray + m_n3DPoints;
	
	void **ppTmp;
	void *vpRegion;
	void **ppRegion, **ppRegion2;
	void **pRegionPtrBuff1End;
	RVLMESH_LINK *ConnectionArray2;
	short nNeighbors2;
	int nRegions;

	CRVLMPtrChain *pRegionList;

	// for every point in m_Point3DArray

	for(pPoint3D = m_Point3DArray; pPoint3D < pPoint3DArrayEnd; pPoint3D++)
	{
		//if(pPoint3D->iPix == 250 + 132 * 320)
		//	int tmp1 = 0;

		pDelaunayData = DelaunayMap[pPoint3D->iPix];

		ConnectionArray = pConnection = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		nNeighbors = *((short *)pDelaunayData);

		// for every connection of pPoint3D

		for(iNeighbor = 0; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
		{
			if(pConnection->vpRegion)
				continue;

			pRegionList = &(pPoint3D->regionList);

			nRegions = pRegionList->m_nElements;

			ppRegion = RegionPtrBuff1;

			pRegionList->Start();

			while(pRegionList->m_pNext)
				*(ppRegion)++ = pRegionList->GetNext();

			//iConnection2 = iNeighbor;

			//ConnectionArray2 = ConnectionArray;

			//nNeighbors2 = nNeighbors;

			pConnection2 = ConnectionArray + ((iNeighbor + 1) % nNeighbors);

			iPix = pConnection2->iPix;

			pDelaunayData = DelaunayMap[iPix];

			nNeighbors2 = *((short *)pDelaunayData);

			ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

			iConnection2 = pConnection2->iConnection;

			pConnection2 = ConnectionArray2 + iConnection2;

			do
			{
				pPoint3D2 = m_Point3DMap[iPix];

				if(pPoint3D2 == NULL)
				{
					nRegions = 0;

					break;
				}

				pRegionList = &(pPoint3D2->regionList);	

				pRegionPtrBuff1End = RegionPtrBuff1 + nRegions;

				ppRegion2 = RegionPtrBuff2;

				pRegionList->Start();

				while(pRegionList->m_pNext)
				{
					vpRegion = pRegionList->GetNext();

					for(ppRegion = RegionPtrBuff1; ppRegion < pRegionPtrBuff1End; ppRegion++)
						if(*ppRegion == vpRegion)
						{
							*(ppRegion2++) = vpRegion;

							break;
						}
				}

				nRegions = ppRegion2 - RegionPtrBuff2;

				if(nRegions == 0)
					break;

				ppTmp = RegionPtrBuff1;
				RegionPtrBuff1 = RegionPtrBuff2;
				RegionPtrBuff2 = ppTmp;

				pConnection2 = ConnectionArray2 + ((iConnection2 + 1) % nNeighbors2);

				iPix = pConnection2->iPix;

				pDelaunayData = DelaunayMap[iPix];

				nNeighbors2 = *((short *)pDelaunayData);

				ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

				iConnection2 = pConnection2->iConnection;

				pConnection2 = ConnectionArray2 + iConnection2;
			}
			while(pConnection2 != pConnection);

			vpRegion = (nRegions == 1 ? RegionPtrBuff1[0] : (void *)(0xffffffff));

			pConnection2 = pConnection;

			iConnection2 = iNeighbor;

			nNeighbors2 = nNeighbors;

			ConnectionArray2 = ConnectionArray;

			do
			{
				pConnection2->vpRegion = vpRegion;

				pConnection2 = ConnectionArray2 + ((iConnection2 + 1) % nNeighbors2);

				pDelaunayData = DelaunayMap[pConnection2->iPix];

				nNeighbors2 = *((short *)pDelaunayData);

				ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

				iConnection2 = pConnection2->iConnection;

				pConnection2 = ConnectionArray2 + iConnection2;
			}
			while(pConnection2 != pConnection);
		}
	}

	delete[] RegionPtrBuff;
#endif

	/*********************************************************/
	/* 3. Detect region boundaries				             */
	/*********************************************************/
	
	BYTE **DelaunayMap = m_pDelaunay->m_DelaunayMap;

	// set RVLMESH_LINK_FLAG_BOUNDARY flag of every border connection

	pDelaunayData = DelaunayMap[0];

	nNeighbors = *((short *)pDelaunayData);

	RVLMESH_LINK *ConnectionArray = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

	pConnection = ConnectionArray;

	while(pConnection->iPix != m_Width)
		pConnection++;
	
	short iConnection2 = pConnection - ConnectionArray;

	RVLMESH_LINK *pConnection2 = pConnection;

	do
	{
		pConnection2->Flags |= RVLMESH_LINK_FLAG_BOUNDARY;

		pConnection2 = ConnectionArray + ((iConnection2 + 1) % nNeighbors);

		pDelaunayData = DelaunayMap[pConnection2->iPix];

		nNeighbors = *((short *)pDelaunayData);

		ConnectionArray = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		iConnection2 = pConnection2->iConnection;

		pConnection2 = ConnectionArray + iConnection2;
	}
	while(pConnection2 != pConnection);

	// set RVLMESH_LINK_FLAG_BOUNDARY flag of other connections

	void **RegionPtrBuff = new void *[4 * (m_Width * m_Height)];

	void **RegionPtrBuff1 = RegionPtrBuff;
	void **RegionPtrBuff2 = RegionPtrBuff + 2 * (m_Width * m_Height);

	RVL3DPOINT2 *pPoint3DArrayEnd = m_Point3DArray + m_n3DPoints;
	
	//void **ppTmp;
	void *vpRegion;
	void **ppRegion, **ppRegion2;
	void **pRegionPtrBuff1End, **pRegionPtrBuff2End;
	RVLMESH_LINK *ConnectionArray2, *ConnectionArray3;
	short nNeighbors2, nNeighbors3;
	int nRegions, nRegions2;
	CRVLMPtrChain *pRegionList, *pRegionList2, *pRegionList3;
	int iPix2;
	RVLMESH_LINK *pConnection3;
	int iConnection3;
	BOOL bMatch;

	// for every point in m_Point3DArray

	for(pPoint3D = m_Point3DArray; pPoint3D < pPoint3DArrayEnd; pPoint3D++)
	{
		//if(pPoint3D->iPix == 250 + 132 * 320)
		//	int tmp1 = 0;

		pDelaunayData = DelaunayMap[pPoint3D->iPix];

		ConnectionArray = pConnection = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		nNeighbors = *((short *)pDelaunayData);

		pRegionList = &(pPoint3D->regionList);

		nRegions = pRegionList->m_nElements;

		ppRegion = RegionPtrBuff1;

		pRegionList->Start();

		while(pRegionList->m_pNext)
			*(ppRegion)++ = pRegionList->GetNext();

		pRegionPtrBuff1End = ppRegion;

		// for every connection of pPoint3D

		for(iNeighbor = 0; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
		{
			iPix2 = pConnection->iPix;

			pPoint3D2 = m_Point3DMap[iPix2];

			if(pPoint3D2 == NULL)
				continue;

			pRegionList2 = &(pPoint3D2->regionList);

			nRegions2 = pRegionList2->m_nElements;

			ppRegion2 = RegionPtrBuff2;

			pRegionList2->Start();

			while(pRegionList2->m_pNext)
			{
				vpRegion = pRegionList2->GetNext();

				for(ppRegion = RegionPtrBuff1; ppRegion < pRegionPtrBuff1End; ppRegion++)
					if(*ppRegion == vpRegion)
					{
						*(ppRegion2++) = vpRegion;

						break;
					}
			}

			pRegionPtrBuff2End = ppRegion2;

			nRegions2 = pRegionPtrBuff2End - RegionPtrBuff2;

			if(nRegions2 == 0)
				continue;

			pDelaunayData = DelaunayMap[iPix2];

			ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

			nNeighbors2 = *((short *)pDelaunayData);

			pConnection2 = ConnectionArray2 + ((pConnection->iConnection + nNeighbors2 - 1) % nNeighbors2);

			iConnection3 = ((iNeighbor + 1) % nNeighbors);

			pConnection3 = ConnectionArray + iConnection3;

			pPoint3D3 = m_Point3DMap[pConnection3->iPix];

			pDelaunayData = DelaunayMap[pConnection3->iPix];

			ConnectionArray3 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

			iConnection3 = pConnection3->iConnection;

			pConnection3 = ConnectionArray3 + iConnection3;

			nNeighbors3 = *((short *)pDelaunayData);

			do
			{				
				if(pPoint3D3 == NULL)
				{
					pConnection->Flags |= RVLMESH_LINK_FLAG_BOUNDARY;

					break;
				}

				pRegionList3 = &(pPoint3D3->regionList);	

				for(ppRegion2 = RegionPtrBuff2; ppRegion2 < pRegionPtrBuff2End; ppRegion2++)
				{
					vpRegion = *ppRegion2;

					pRegionList3->Start();

					bMatch = FALSE;

					while(pRegionList3->m_pNext)
						if(bMatch = (pRegionList3->GetNext() == vpRegion))
							break;

					if(!bMatch)
						break;
				}

				if(!bMatch)
				{
					pConnection->Flags |= RVLMESH_LINK_FLAG_BOUNDARY;

					break;
				}

				pConnection3 = ConnectionArray3 + ((iConnection3 + 1) % nNeighbors3);

				pPoint3D3 = m_Point3DMap[pConnection3->iPix];

				iConnection3 = pConnection3->iConnection;

				pDelaunayData = DelaunayMap[pConnection3->iPix];

				ConnectionArray3 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

				nNeighbors3 = *((short *)pDelaunayData);

				pConnection3 = ConnectionArray3 + iConnection3;
			}
			while(pConnection3 != pConnection2);
		}	// for every neighbor of pPoint3D
	}

	delete[] RegionPtrBuff;

	/***************************************************************************************/
	/* 4. Create relation between neighbouring segments and add to SortedSegmentBuffer     */
	/***************************************************************************************/

	CRVLMPtrChain **SortedSegmentBuffer = new CRVLMPtrChain *[m_n3DPoints];
	memset(SortedSegmentBuffer, 0x00, m_n3DPoints * sizeof(CRVLMPtrChain *));
	CRVLMPtrChain *pSortedBin;

	CRVLMPtrChain SortedBinTemplate;
	SortedBinTemplate.m_pMem = m_pMem2;

	nMax = 0;


	CRVLRelation *pRelation, *pRelation2;
	CRVLMPtrChain2 RelList2;
	
	
	CRVLMPtrChain *pSegmentList;
	CRVL2DRegion2 *p2DRegion2,*p2DRegionNeighbor;  

	bool bRelExists = false;

	//Go through all regions (original + new)
	regionList = p2DRegionSet->m_ObjectList;

	//For each region  (main loop)
	regionList.Start();

	while(regionList.m_pNext)
	{
		//get segment
		p2DRegion = (CRVL2DRegion2 *)(regionList.GetNext());

		//reset flag (RANSAC not yet performed)
		p2DRegion->m_Flags &= ~RVL2DREGION_FLAG_MARKED;
		//reset flag (Final fitting not yet performed)
		p2DRegion->m_Flags &= ~RVL2DREGION_FLAG_MARKED2;
		
		//get nMax
		n3DPoints = p2DRegion->m_n3DPts;

		if(n3DPoints > nMax)
			nMax = n3DPoints;

		RelList.Create(p2DRegion->m_RelList + p2DRegion->m_pClass->m_iRelListNeighbors, m_pMem);

		//Re-create relation list
		//Go through all 3D points
		ppPt = p2DRegion->m_pPoint3DArray;

		for(j=0; j<n3DPoints; j++,ppPt++)
		{
			pPoint3D = *ppPt;

			pSegmentList = &(pPoint3D->regionList);

			//check if we have a region list
			if(pSegmentList->m_nElements > 1)
			{
				//Go through region List
				pSegmentList->Start();
				while(pSegmentList->m_pNext)
				{
					p2DRegion2 = (CRVL2DRegion2 *)(pSegmentList->GetNext());

					if(p2DRegion == p2DRegion2)
						continue;

					if(p2DRegion2->m_Flags & RVL2DREGION_FLAG_MARKED)
						continue;

					//check to see that a relation does not already exist
					bRelExists = false;

					RelList2.Create(p2DRegion2->m_RelList + p2DRegion2->m_pClass->m_iRelListNeighbors, m_pMem);

					RelList2.Start();

					while(RelList2.m_pNext)
					{
						pRelation2 = (CRVLRelation *)(RelList2.GetNext());
					
						p2DRegionNeighbor = (CRVL2DRegion2 *)(pRelation2->m_pObject[0]);

						if(p2DRegionNeighbor == p2DRegion)
						{
							bRelExists = true;
							break;
						}
							
					}
					
					if(bRelExists)
						continue;

					//create relation
					pRelation=(CRVLRelation *)(m_pMem->Alloc(sizeof(CRVLRelation)));

					p2DRegion2->m_Flags |= RVL2DREGION_FLAG_MARKED;

					pRelation->m_pObject[0] = p2DRegion;
					pRelation->m_pObject[1] = p2DRegion2;

					RelList2.Add(pRelation);

					RelList.Add(pRelation);
				}

			
			}
			
		}

		//reset flags 
		RelList.Start();
		while(RelList.m_pNext)
		{
			pRelation = (CRVLRelation *)(RelList.GetNext());

			p2DRegion2 = (CRVL2DRegion2 *)(pRelation->m_pObject[1]);

			p2DRegion2->m_Flags &= ~RVL2DREGION_FLAG_MARKED;
			
		}
			
	
		//store region in SortedSegmentBuffer depending on n3DPoints
		//1. get position 
		pSortedBin = SortedSegmentBuffer[n3DPoints];

		if(pSortedBin == NULL)
		{
			pSortedBin = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
			*pSortedBin = SortedBinTemplate;

			SortedSegmentBuffer[n3DPoints] = pSortedBin;
		}

		//2. Add region
		pSortedBin->Add(p2DRegion);

	}
	




	/******************************************************************************************/
	/* 5. Iterate through SortedSegmentBuffer and perform region/segment growing with RANSAC */
	/******************************************************************************************/

	double minPerc = 0.5;  //determines the min. percentage of points that a neighbour segment 
						   //must have in order to be added to the main segment
	double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
						(m_pStereoVision->m_pCameraL->CenterXNrm - 
						m_pStereoVision->m_pCameraR->CenterXNrm);

	RVL3DPOINT2 **Point3DArray = new RVL3DPOINT2 *[m_n3DPoints];   //contains 3D points of segment + neighbours
	RVL3DPOINT2 **Point3DArrayNeighbour = new RVL3DPOINT2 *[m_n3DPoints];   //contains 3D points of neighbour

	//contains set of grouped (sub) segments for RANSAC
	CRVL2DRegion2 **SubRegionList = (CRVL2DRegion2 **)(m_pMem->Alloc(p2DRegionSet->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	memset(SubRegionList, 0x00, p2DRegionSet->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));
	
	CRVL2DRegion2 **ppSubRegionList,**ppSubRegionListHelper, **ppSubRegionStart,  **ppSubRegion;

	int nList;

	
	RVL3DPOINT2 **pConsensusSet;
	
	int nConsensus;
	double Score, ScorePerc;
	double BestScore = 0.0;

	CRVLMPtrChain *pSortedBinNew;
	
	CRVL2DRegion2 *p2DRegionHelper;
	CRVL2DRegion2 *p2DSegment, *p2DSuperSegment;
	

	RVLARRAY *pRelList;

	RVLPSDLAD_SEGMENT_DETAILS segmentDetails;
	int nBest = 0;

	//RVL2DREGION_FLAG_MARKED -> indicates RANSAC+sorting has been performed
	//RVL2DREGION_FLAG_MARKED2 -> indicates final fitting has been performed

	ppSubRegionList = SubRegionList;
	ppSubRegionListHelper = SubRegionList;
	
	for(i = nMax; i >= 3; i--)  //No need to go below 3 points, cannot determine plane
	{
		pSortedBin = SortedSegmentBuffer[i];
		
		if(pSortedBin == NULL)
			continue;
		
		pSortedBin->Start();
		while(pSortedBin->m_pNext)
		{
			//get segment
			p2DRegion = (CRVL2DRegion2 *)(pSortedBin->GetNext());

			//a. Make sure it has not been marked (final fitting has not been performed)
			//b. Make sure it has 3D points before performing RANSAC  (This is an unnecessary check if i>=3 otherwise it is!!)
			if(((p2DRegion->m_Flags & RVL2DREGION_FLAG_MARKED2)==0) && (p2DRegion->m_n3DPts > 0)) 
			{
				//RANSAC and 2nd sorting has not been performed
				if((p2DRegion->m_Flags & RVL2DREGION_FLAG_MARKED)==0) 
				{
					
						nBest = RANSAC(p2DRegion, false, p2DRegion->m_pPoint3DArray, p2DRegion->m_n3DPts, &segmentDetails, 
								Flags | RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET);


						//Set flag
						p2DRegion->m_Flags |= RVL2DREGION_FLAG_MARKED;

						//move p2DRegion lower if nBest < p2DRegion->m_n3DPts
						if(nBest < p2DRegion->m_n3DPts)
						{
							pSortedBinNew = SortedSegmentBuffer[nBest];

							if(pSortedBinNew == NULL)
							{
								pSortedBinNew = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
								*pSortedBinNew = SortedBinTemplate;

								SortedSegmentBuffer[nBest] = pSortedBinNew;
							}

							pSortedBinNew->Add(p2DRegion);
						}
					
				} 
				//Expand marked regions (RANSAC and 2nd sorting has been performed)
				else 
				{
					//check neighbouring regions and add if minPerc of the number of points in the neighbor is within the error
					//mark neighbour if it is included
					//perform LS fitting on all the points

					//1. Reset temp buffer and fill with current 3D points
					
					// reset ConsensusSet pointer
					pConsensusSet = Point3DArray;

					Consensus(p2DRegion->m_pPoint3DArray, p2DRegion->m_n3DPts, p2DRegion, pConsensusSet, nConsensus, Flags, BestScore);

					pConsensusSet += nConsensus;

					//Create super segment
					p2DSuperSegment = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet2));

					p2DSuperSegment->m_a = p2DRegion->m_a;
					p2DSuperSegment->m_b = p2DRegion->m_b;
					p2DSuperSegment->m_c = p2DRegion->m_c;

					p2DSuperSegment->m_Tol = m_Tol;


					//2. Reset beginning of sub region and add main segment
					ppSubRegionStart = ppSubRegionList;  

					ppSubRegionListHelper = ppSubRegionList;

					p2DRegion->m_Flags |= RVL2DREGION_FLAG_MARKED2;
					*(ppSubRegionList++) = p2DRegion;
					
					while(ppSubRegionListHelper < ppSubRegionList)
					{
						p2DRegionHelper = *ppSubRegionListHelper;
					
						//3. Get neighbours
						RelList.Create(p2DRegionHelper->m_RelList + p2DRegionHelper->m_pClass->m_iRelListNeighbors, NULL);

						RelList.Start();

						while(RelList.m_pNext)
						{
							pRelation = (CRVLRelation *)(RelList.GetNext());
						
							p2DRegionNeighbor = (CRVL2DRegion2 *)(pRelation->m_pObject[0]);

							if(p2DRegionNeighbor == p2DRegionHelper)
								p2DRegionNeighbor = (CRVL2DRegion2 *)(pRelation->m_pObject[1]);


							//Make sure neighbour has not been marked (ie final fitting has not already been performed)
							//Neighbour must have 3D points
							if(((p2DRegionNeighbor->m_Flags & RVL2DREGION_FLAG_MARKED2)!=0) ||  (p2DRegionNeighbor->m_n3DPts == 0))// 
								continue;

							//Check consensus for neighbour using plane parameters of main segment
							

							//Get Score  
							Score = Consensus(p2DRegionNeighbor->m_pPoint3DArray, p2DRegionNeighbor->m_n3DPts, p2DRegion, pConsensusSet, nConsensus, Flags, BestScore);

							ScorePerc = Score/((double)(p2DRegionNeighbor->m_n3DPts));

							//Add neighbour 3D points from COnsenus set to Point3DArray and add neighbour to RegionList
							if(ScorePerc > minPerc)
							{
								//set/move consensus set pointer
								pConsensusSet += nConsensus;

								p2DRegionNeighbor->m_Flags |= RVL2DREGION_FLAG_MARKED2;

								*(ppSubRegionList++) = p2DRegionNeighbor;

							}
							

						}//while RelList.m_pNext


						ppSubRegionListHelper++;
					}
					
					n3DPoints = pConsensusSet - Point3DArray;

					pRelList = p2DSuperSegment->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_COMPONENTS];

					//updating segment params
					pRelList->pFirst = (unsigned char *)ppSubRegionStart;
					pRelList->pEnd = (unsigned char *)ppSubRegionList;

					nList = ppSubRegionList - ppSubRegionStart;

					//perform final fitting 
					if(nList > 1)
						LSPlane(Point3DArray, n3DPoints, p2DSuperSegment);

					nPoints = 0;

					ppSubRegion = ppSubRegionStart;
					//updating sub segment params
					for(j = 0; j < nList; j++,ppSubRegion++)
					{
						p2DSegment = *ppSubRegion; //SubRegionList[j];

						nPoints += p2DSegment->m_nPts;

						pp2DSuperSegment = (CRVL2DRegion2 **)(p2DSegment->m_pData + p2DRegionSet->m_iDataParentPtr);

						*pp2DSuperSegment = p2DSuperSegment;
					}
					

					p2DSuperSegment->m_n3DPts = n3DPoints;
					p2DSuperSegment->m_nPts = nPoints;

					p2DSuperSegment->m_a /= 16.0;
					p2DSuperSegment->m_b /= 16.0;
					p2DSuperSegment->m_c = p2DSuperSegment->m_c/16.0 + d0;
				
				} //expand marked regions
				
			}//if(p2DRegion->m_n3DPts > 0) 
		
		}//while(pSortedBin->m_pNext)

	}//for(i = nMax; i >= 3; i--) 



	
	m_pMem2->Clear();

	delete[] Point3DArray;
	delete[] Point3DArrayNeighbour;
	
	delete[] SortedSegmentBuffer;
	//delete[] Point3DPtrArray;


	
}



	




// Finds connected components in a set A of points in uvd-space
//
//	Relevant member variables:
//
//		Parameters:
//			m_minSegmentSize		-	min. num. of pts. in a connected component
//
//	Flags:
//		RVLPSDGCC_FLAG_DILATE		-	the projection of A onto the uv-plane is dilated 
//										by region growing before detection of connected components 

void CRVLPlanarSurfaceDetector::GetConnectedComponents(RVL3DPOINT2 **Point3DSet,	// Input: set A of uvd-points
													   int n,						// Input: num. of pts. in A
													   CRVLMChain *pCCList,			// Output: list of connected components
													   DWORD Flags
													   //unsigned short Mask
													   //CRVL2DRegion2 *pPlane
													   )
{
	//double a, b, c;

	//if(Flags & RVLPSDGCC_FLAG_PLANE)
	//{
	//	a = pPlane->m_a;
	//	b = pPlane->m_b;
	//	c = pPlane->m_c;
	//}

	unsigned int *RegionGrowingMap = (unsigned int *)m_RegionGrowingMap;

	if(Flags & RVLPSDGCC_FLAG_DILATE)
		Dilate(Point3DSet, n);

	RVL3DPOINT2 **pPoint3DSetEnd = Point3DSet + n;

	RVL3DPOINT2 **pCCBuff;
	RVL3DPOINT2 **ppPt;
	RVL3DPOINT2 *pPt0, *pPt1, *pPt2;
	int iPix1, iPix2;
	int d2;
	int ed;
	BYTE *CCData;
	RVLARRAY CC;
	int CCDataSize;
	//double ePlane;
	int j;

	for(ppPt = Point3DSet; ppPt < pPoint3DSetEnd; ppPt++)
	{
		pPt0 = *ppPt;

		iPix1 = pPt0->iPix;

		if(RegionGrowingMap[iPix1] > (unsigned int)m_nCCDilationIterations)
			continue;

		//if(Flags & RVLPSDGCC_FLAG_MASK)
		//	if(m_RegionGrowingMap[iPix1] != Mask)
		//		continue;

		//if(Flags & RVLPSDGCC_FLAG_PLANE)
		//{
		//	ePlane = DOUBLE2INT(a * pPt0->x + b * pPt0->y + c - pPt0->z);

		//	if(ePlane > m_Tol) 
		//		continue;

		//	if(ePlane < -m_Tol) 
		//		continue;
		//}

		// region growing 

		pCCBuff = m_CCBuff;

		m_RegionGrowingBuff[0] = iPix1;

		pPt1 = m_Point3DMap[iPix1];

		if(pPt1)
			if(RegionGrowingMap[iPix1] == 0)
				*(pCCBuff++) = pPt1;

		RegionGrowingMap[iPix1] = 0xffffffff;

		m_iRG1 = 1;

		m_iRG2 = 0;

		while(m_iRG2 < m_iRG1)
		{
			iPix2 = m_RegionGrowingBuff[m_iRG2];

			pPt2 = m_Point3DMap[iPix2];

			if(pPt2)
				d2 = pPt2->d;

			for(j = 0; j < 4; j++)	// for each point in 4-neighborhood of pPt2
			{
				iPix1 = iPix2 + m_dpNeighbor4[j];

				pPt1 = m_Point3DMap[iPix1];

				if(RegionGrowingMap[iPix1] > (unsigned int)m_nCCDilationIterations)
					continue;

				//if(Flags & RVLPSDGCC_FLAG_MASK)
				//	if(m_RegionGrowingMap[iPix1] != Mask)
				//		continue;

				if(pPt1)
				{
					//if(Flags & RVLPSDGCC_FLAG_PLANE)
					//{
					//	ePlane = DOUBLE2INT(a * pPt1->x + b * pPt1->y + c - pPt1->z);

					//	if(ePlane > m_Tol) 
					//		continue;

					//	if(ePlane < -m_Tol) 
					//		continue;
					//}

					if(Flags & RVLPSDGCC_FLAG_DISPARITY_CONTINUITY)
					{
						ed = pPt1->d - d2;

						if(ed > m_DiscontinuitiyThr)
							continue;

						if(ed < -m_DiscontinuitiyThr)
							continue;
					}

					if(RegionGrowingMap[iPix1] == 0)
						*(pCCBuff++) = pPt1;
				}

				RegionGrowingMap[iPix1] = 0xffffffff;

				m_RegionGrowingBuff[m_iRG1++] = iPix1;				
			}	// for each point in 4-neighborhood of pPt2
			
			m_iRG2++;
		}	// region growing loop		

		if(m_iRG1 < m_NoiseThr)
			continue;

		CCDataSize = (pCCBuff - m_CCBuff) * sizeof(RVL3DPOINT *);

		CCData = (BYTE *)(m_pMem2->Alloc(CCDataSize));

		memcpy(CCData, m_CCBuff, CCDataSize);

		CC.pFirst = CCData;
		CC.pEnd = CCData + CCDataSize;

		pCCList->Add(&CC);
	}	// for every point in m_Point3DArray
}

//	Clears m_RegionGrowingMap

void CRVLPlanarSurfaceDetector::ClearRegionGrowingMap()
{
	memcpy(m_RegionGrowingMap, m_EmptyRegionGrowingMap, 
		m_Width * m_Height * sizeof(int));
}

void CRVLPlanarSurfaceDetector::Mask(RVL3DPOINT2 **PtSet, 
									 int n,
									 int mask)
{
	RVL3DPOINT2 **PtSetEnd = PtSet + n;

	RVL3DPOINT2 **ppPt;

	for(ppPt = PtSet; ppPt < PtSetEnd; ppPt++)
		m_RegionGrowingMap[(*ppPt)->iPix] = mask;
}

// Dilation of a set A of uvd-points by region growing
//
//

void CRVLPlanarSurfaceDetector::Dilate(	RVL3DPOINT2 **PtSet,	// Input: set A of uvd-points
										int n)					// Input: num. of pts. in A
{
	// copy PtSet to m_RegionGrowingBuff

	RVL3DPOINT2 **pPtSetEnd = PtSet + n;

	int *pRGBuff = m_RegionGrowingBuff;

	RVL3DPOINT2 **ppPt;

	for(ppPt = PtSet; ppPt < pPtSetEnd; ppPt++, pRGBuff++)
		*pRGBuff = (*ppPt)->iPix;

	// dilation

	m_iRG1 = n;
	m_iRG2 = 0;

	int iRG3 = n;

	int depth = 1;

	int j;
	int iPix1, iPix2;

	while(m_iRG2 < m_iRG1)
	{
		iPix2 = m_RegionGrowingBuff[m_iRG2];

		for(j = 0; j < 4; j++)	// for each point in 4-neighborhood of pPt2
		{
			iPix1 = iPix2 + m_dpNeighbor4[j];

			if(m_RegionGrowingMap[iPix1] == 0xffffffff)
			{
				m_RegionGrowingMap[iPix1] = depth;

				m_RegionGrowingBuff[m_iRG1++] = iPix1;
			}
			
		}	// for each point in 4-neighborhood of pPt2
		
		m_iRG2++;

		if(m_iRG2 == iRG3)
		{
			depth++;

			if(depth > m_nCCDilationIterations)
				break;

			iRG3 = m_iRG1;
		}
	}	// region growing loop	
}

void CRVLPlanarSurfaceDetector::GetRefSurfaces(CRVLMPtrChain *p2DRegionList, 
											   double &y0, double &z0,
											   double **pHArray)
{
	// svn

	m_pGround = NULL;

	m_nWalls = 0;

	if(p2DRegionList->m_nElements == 0)
		return;

	CRVL2DRegion2 *pSegment;

	int maxnGroundPts = 0;

	int n = p2DRegionList->m_nElements;

	int *Index = new int[n];

	int *Key = new int[n];

	CRVL2DRegion2 **UnsortedWallArray = new CRVL2DRegion2 *[n];

	int iWall = 0;

	double absb;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(pSegment->m_n3DPts < 1000)
			continue;

		absb = fabs(pSegment->m_b);

		if(fabs(pSegment->m_a) < absb && absb > 0.08)
		{
			if(pSegment->m_n3DPts > maxnGroundPts)
			{
				maxnGroundPts = pSegment->m_n3DPts;

				m_pGround = pSegment;
			}
		}
		else
		{
			pSegment->m_Flags |= RVL2DREGION_FLAG_WALL;

			UnsortedWallArray[iWall] = pSegment;

			Index[iWall] = iWall;

			Key[iWall] = pSegment->m_n3DPts;

			iWall++;
			
		}
	}

	if(m_pGround)
		m_pGround->m_Flags |= RVL2DREGION_FLAG_GROUND;

	if(m_WallArray)
		delete[] m_WallArray;

	m_nWalls = iWall;

	m_WallArray = new CRVL2DRegion2 *[m_nWalls];

	Sort(Key, Index, m_nWalls, TRUE);

	for(iWall = 0; iWall < m_nWalls; iWall++)
		m_WallArray[m_nWalls - iWall - 1] = UnsortedWallArray[Index[iWall]];

	delete[] Index; 

	delete[] Key;

	delete[] UnsortedWallArray;

	if(m_pGround != NULL && m_nWalls > 0)
	{
		CRVL3DSurface2 *pGround3DSurface = (CRVL3DSurface2 *)(m_pGround->m_vp3DSurface);

		// compute Homographies

		CRVL3DSurface2 *pWall3DSurface;
		
		if(pHArray)
		{
			double *HArray = new double[3 * 3 * m_nWalls];

			*pHArray = HArray;
	
			double X[3];
			double fTmp;
	
			for(iWall = 0; iWall < m_nWalls; iWall++)
			{
				pWall3DSurface = (CRVL3DSurface2 *)(m_WallArray[iWall]->m_vp3DSurface);
	
				CrossProduct(pWall3DSurface->m_N, pGround3DSurface->m_N, X);
	
				fTmp = sqrt(RVLDotProduct(X, X));
	
				X[0] /= fTmp;
				X[1] /= fTmp;
				X[2] /= fTmp;
	
				m_pStereoVision->m_pCameraL->Homography(pWall3DSurface->m_N, pWall3DSurface->m_d, 
					X, HArray + 3 * 3 * iWall);
			}

			delete[] HArray;
		}

		// compute RotLG

		m_PoseLG.m_Rot[2 * 3 + 0] = -pGround3DSurface->m_N[0];
		m_PoseLG.m_Rot[2 * 3 + 1] = -pGround3DSurface->m_N[1];
		m_PoseLG.m_Rot[2 * 3 + 2] = -pGround3DSurface->m_N[2];

		double XGL[3];

		pWall3DSurface = (CRVL3DSurface2 *)(m_WallArray[0]->m_vp3DSurface);

		CrossProduct(pWall3DSurface->m_N, pGround3DSurface->m_N, XGL);

		double ftmp = sqrt(RVLDotProduct(XGL, XGL));

		m_PoseLG.m_Rot[0 * 3 + 0] = XGL[0] / ftmp;
		m_PoseLG.m_Rot[0 * 3 + 1] = XGL[1] / ftmp;
		m_PoseLG.m_Rot[0 * 3 + 2] = XGL[2] / ftmp;

		CrossProduct(m_PoseLG.m_Rot + 2 * 3, m_PoseLG.m_Rot, m_PoseLG.m_Rot + 1 * 3);
	}
}


void CRVLPlanarSurfaceDetector::Get3DPlanes(CRVLMPtrChain *p2DRegionList, 
											CRVLClass *p3DSurfaceSet)
{
	CRVL2DRegion2 *p2DRegion;
	CRVL2DRegion2 **pp2DRegion;
	CRVL3DSurface2 *p3DSurface;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		p3DSurface = (CRVL3DSurface2 *)RVL3DSurfaceTemplate.Create3(p3DSurfaceSet);

		Get3DPlane(p2DRegion, p3DSurface);

		p2DRegion->m_vp3DSurface = p3DSurface;

		pp2DRegion = (CRVL2DRegion2 **)(p3DSurface->GetData(RVLOBJ2_DATA_PROJECT_PTR));

		*pp2DRegion = p2DRegion;
	}
}


void CRVLPlanarSurfaceDetector::DisplayProjectedIntensityGradient(	CRVLMPtrChain *p2DRegionList,
																	PIX_ARRAY *pIIn,
																	PIX_ARRAY *pIOut)
{
	// copy original image to output image

	unsigned char *pPixIn = pIIn->pPix;

	int w = pIIn->Width;
	int h = pIIn->Height;

	unsigned char *pPixOut = pIOut->pPix;

	int u, v;
	unsigned char I;
	int iPix;

	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pPixIn++)
		{
			iPix = u + v * m_Width;

			I = (*pPixIn);

			*(pPixOut++) = I;
			*(pPixOut++) = I;
			*(pPixOut++) = I;
		}
	}

	// display projected intensity gradients with HSL color

	CRVL2DRegion2 *pSegment;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	double phi;
	double s;
	RVLCOLOR Color;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if((pSegment->m_Flags & RVL2DREGION_FLAG_WALL) == 0)
			continue;

		ppPt = (RVL3DPOINT2 **)pSegment->m_PtArray;

		pPtArrayEnd = ppPt + pSegment->m_nPts;

		for(; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			if(pPt->dI < 5)
				continue;

			pPixOut = pIOut->pPix + 3 * pPt->iPix;

			I = *pPixOut;

			phi = 0.5 * ((double)(pPt->dIProjAngle) * NRM2RAD - PI);

			s = (double)(pPt->dI) / 30.0;

			if(s > 1.0)
				s = 1.0;

			//Color = RVLHSL2RGB(2.0 * phi, s, (double)(I / 255.0) * (1.0 - 0.5 * s) + 0.5 * s);
			Color = RVLHSL2RGB(2.0 * phi, 1.0, 0.5);

			*(pPixOut++) = Color.r;
			*(pPixOut++) = Color.g;
			*pPixOut     = Color.b;
		}		
	}
}

void CRVLPlanarSurfaceDetector::ProjectIntensityGradientToPlanarSurfaces(
									CRVLAImage *pAImage,
									CRVLMPtrChain *p2DRegionList
									//float *dIx, float *dIy
									)
{
	if(m_pGround == NULL)
		return;

	double f  = m_pStereoVision->m_pCameraL->fNrm;
	double uc = m_pStereoVision->m_pCameraL->CenterXNrm;
	double vc = m_pStereoVision->m_pCameraL->CenterYNrm;
	double l  = m_pStereoVision->m_BaseLenNrm;

	double a0 = m_pGround->m_a;
	double b0 = m_pGround->m_b;
	double c0 = m_pGround->m_c;

	//int ImageSize = m_Width * m_Height;

	//memset(dIx, 0, ImageSize * sizeof(float));
	//memset(dIy, 0, ImageSize * sizeof(float));

	CRVL2DRegion2 *pSegment;

	double v1, v2;
	double a, b, c;
	double u, v;
	//double dIu, dIv;
	double det, lv;
	double X[3], Y[3], Z[3];
	double ftmp;
	double H[3 * 2];
	RVL3DPOINT2 **ppPt, **pEndPtArray;
	RVL3DPOINT2 *pPt;
	RVLAPIX *pAPix;
	double a13, a23;
	//double dux, duy, dvx, dvy;
	double dxu, dxv, dyu, dyv;
	double dus, dvs;
	double dxs, dys;
	//double dIx, dIy;
	double nrm;
	int dI;
	RVLEDGE_ELEMENT *pEdgeElement, *pEdgeElement1, *pEdgeElement2;
	RVL2DCONTOUR_SEGMENT *p2DContourSeg;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if((pSegment->m_Flags & RVL2DREGION_FLAG_WALL) == 0)
			continue;

		a = pSegment->m_a;
		b = pSegment->m_b;
		c = pSegment->m_c;

		a13 = a*uc/f+b*vc/f+c/f;
		a23 = a0*uc/f+b0*vc/f+c0/f;

		det = a * b0 - b * a0;

		v1 = (-b0 * a13 + b * a23) / det;
		v2 = ( a0 * a13 - a * a23) / det;

		lv = sqrt(v1 * v1 + v2 * v2 + 1.0);

		X[0] = v1 / lv;
		X[1] = v2 / lv;
		X[2] = 1.0 / lv;

		Z[0] = f * a;
		Z[1] = f * b;
		Z[2] = a * uc + b * vc + c;

		ftmp = sqrt(RVLDotProduct(Z, Z));

		Z[0] /= ftmp;
		Z[1] /= ftmp;
		Z[2] /= ftmp;

		CrossProduct(Z, X, Y);

		H[0 * 3 + 0] = (f * X[0] + uc * X[2]);
		H[1 * 3 + 0] = (f * X[1] + vc * X[2]);
		//H[2 * 3 + 0] = X[2];

		H[0 * 3 + 1] = (f * Y[0] + uc * Y[2]);
		H[1 * 3 + 1] = (f * Y[1] + vc * Y[2]);
		//H[2 * 3 + 1] = Y[2];

		ppPt = (RVL3DPOINT2 **)pSegment->m_PtArray;

		pEndPtArray = ppPt + pSegment->m_nPts;

		for(; ppPt < pEndPtArray; ppPt++)
		{
			pPt = *ppPt;

			u = pPt->x;
			v = pPt->y;

			//if(pPt->u == 161 && pPt->v == 145)
			//	int tmp1 = 0;

			pAPix = pAImage->m_pPix + pPt->iPix;

			if((pAPix->Flags & RVLAPIX_FLAG_EDGE) == 0)
				continue;

			//dI = DOUBLE2INT(pAPix->fdI);

			dI = pAPix->dI;

			if(dI == 0)
				continue;

			//dIu = pAPix->dIu;
			//dIv = pAPix->dIv;

			pEdgeElement = (RVLEDGE_ELEMENT *)(pAPix->vpEdgeElement);

			if(pEdgeElement == NULL)
				continue;

			p2DContourSeg = pEdgeElement->p2DContourSeg;

			if(p2DContourSeg == NULL)
				continue;

			pEdgeElement1 = (p2DContourSeg->pPrev ? p2DContourSeg->pPrev->pLastEdgeElement : 
				((RVLEDGE_ELEMENT *)(p2DContourSeg->p2DContour->m_ContourIPArray))->pContourNeighbor[0]);

			pEdgeElement2 = p2DContourSeg->pLastEdgeElement;

			//dIu = (double)(pEdgeElement1->v - pEdgeElement2->v);
			//dIv = (double)(pEdgeElement2->u - pEdgeElement1->u);

			//dux = H[0 * 3 + 0] - H[2 * 3 + 0] * u;
			//duy = H[0 * 3 + 1] - H[2 * 3 + 1] * u;
			//dvx = H[1 * 3 + 0] - H[2 * 3 + 0] * v;
			//dvy = H[1 * 3 + 1] - H[2 * 3 + 1] * v;

			//dIx = dIu * dux + dIv * dvx;

			//if(dIx >= 0.0)
			//	dIy = dIu * duy + dIv * dvy;
			//else
			//{
			//	dIx = -dIx;

			//	dIy = -(dIu * duy + dIv * dvy);
			//}

			//nrm = dIx + (dIy >= 0 ? dIy : -dIy);			

			dus = (double)(pEdgeElement2->u - pEdgeElement1->u);
			dvs = (double)(pEdgeElement2->v - pEdgeElement1->v);

			dxu = H[1 * 3 + 1] * (b * v + c) + H[0 * 3 + 1] * a * v;
			dxv = -H[1 * 3 + 1] * b * u - H[0 * 3 + 1] * (a * u + c);
			dyu = -H[1 * 3 + 0] * (b * v + c) - H[0 * 3 + 0] * a * v;
			dyv = H[1 * 3 + 0] * b * u + H[0 * 3 + 0] * (a * u + c);

			dxs = dxu * dus + dxv * dvs;			

			if(dxs >= 0.0)
				dys = dyu * dus + dyv * dvs;
			else
			{
				dxs = -dxs;

				dys = -(dyu * dus + dyv * dvs);
			}

			nrm = dxs + (dys >= 0 ? dys : -dys);

			if(nrm > APPROX_ZERO)
			{
				nrm = 1000.0 / nrm;

				//pPt->dIProjAngle = rtAngle180_2(DOUBLE2INT(nrm * dIx), DOUBLE2INT(nrm * dIy));

				pPt->dIProjAngle = rtAngle180_2(DOUBLE2INT(nrm * dxs), DOUBLE2INT(nrm * dys));

				pPt->dI = dI;
			}
			else
				pPt->dI = 0;
		}
	}
}

double GetAngleDiff(double ground_a, double ground_b, double wall_a, double wall_b, double cs_0, double sn_0)
{
	double a_diff,b_diff, l_diff;
	double cs_diff, sn_diff;

	a_diff = ground_a -  wall_a;
	b_diff = ground_b -  wall_b;
	
	l_diff = sqrt(a_diff*a_diff + b_diff*b_diff);
	cs_diff = a_diff/l_diff;
	sn_diff = b_diff/l_diff;

	return  acos( fabs(cs_diff*cs_0 + sn_diff*sn_0)  );
	
}

void CRVLPlanarSurfaceDetector::GetMinAngles(const char *UVPointsFileName)
{
	
	m_minAngleBtwnLines = PI; 
	m_minAngleBtwnPlanes = PI; 
	m_minAngleBtwnProjectedLines = PI;


	if ((m_pGround == NULL) || (m_nWalls == 0))
		return;


	//Get reference line from file

	//open uv file and get the first 2 points
	FILE *uvFile = fopen(UVPointsFileName, "r");
	
	if(uvFile != NULL)
	{
		int n = 0;
		int u10,u20,v10,v20 = 0;
		char chUV[10];
		
		//get first 4 numbers
		while(!feof(uvFile))
		{
			fgets(chUV,10,uvFile);
			n++;
			
			if(n==1)
				u10 = atoi(chUV);
			else if(n==2)
				v10 = atoi(chUV);
			else if(n==3)
				u20 = atoi(chUV);
			else if(n==4)
				v20 = atoi(chUV);
			else
				break;

		}

		fclose(uvFile);
	
		int dU0 = u20-u10;
		int dV0 = v20-v10;

		double l_Ref = sqrt((double)(dU0 * dU0  +  dV0 * dV0));
		double cs_Ref = -dV0 / l_Ref;
		double sn_Ref = dU0 / l_Ref;
		double rho_Ref = cs_Ref*u10 + sn_Ref*v10;


		double a_diff,b_diff,l_Lin, rho_Lin;
		double cs_Lin, sn_Lin;

		double phi_dif,phi_Projdif;
		
		double cs_3DLin, sn_3DLin, rho_3DLin;
		double cs_3DRef, sn_3DRef, rho_3DRef;

		
		CRVL2DRegion2 *pSegment;

		double H[3 * 3];

	

		int iWall = -1;

		CRVLCamera *pCamera = m_pStereoVision->m_pCameraL;
		CRVL3DSurface2 *p3DGround = (CRVL3DSurface2 *)(m_pGround->m_vp3DSurface);

		double X[3];
		double fTmp = sqrt(p3DGround->m_N[0] * p3DGround->m_N[0] + p3DGround->m_N[1] * p3DGround->m_N[1]);
		X[0] = -p3DGround->m_N[1] / fTmp;
		X[1] = p3DGround->m_N[0] / fTmp;
		X[2] = 0;

		pCamera->Homography(p3DGround->m_N,p3DGround->m_d,X,H);
		
		/* Calculate angle between line */
		//Test for first (and second wall if it exists)

		for (int i=0; i<m_nWalls; i++)
		{
			if(i==4)
				break;

			pSegment = m_WallArray[i];

			//parameters of line obtained by intersecting ground plane and wall plane
			a_diff = m_pGround->m_a -  pSegment->m_a;
			b_diff = m_pGround->m_b -  pSegment->m_b;
	
			l_Lin = sqrt(a_diff*a_diff + b_diff*b_diff);
			cs_Lin = a_diff/l_Lin;
			sn_Lin = b_diff/l_Lin;
			rho_Lin = -(m_pGround->m_c - pSegment->m_c)/l_Lin;

			//angle difference between referent line and line obtained above
			phi_dif=acos( fabs(cs_Lin*cs_Ref + sn_Lin*sn_Ref)  );

			if(phi_dif < m_minAngleBtwnLines)
			{
				m_minAngleBtwnLines = phi_dif;

				RVLProject2DLineTo3DPlane(H,cs_Lin, sn_Lin, rho_Lin,cs_3DLin,sn_3DLin,rho_3DLin);
				RVLProject2DLineTo3DPlane(H,cs_Ref, sn_Ref, rho_Ref,cs_3DRef,sn_3DRef,rho_3DRef);
				
				//angle difference between referent line and line obtained above in 3D
				phi_Projdif=acos( fabs(cs_3DLin*cs_3DRef + sn_3DLin*sn_3DRef)  );
				
				m_minAngleBtwnProjectedLines = phi_Projdif;
				
				iWall=i;
			}
		
		}
		


		if(iWall>-1)
		{
			/* Calculate angle between planes */
			pSegment = m_WallArray[iWall];

			m_minAngleBtwnPlanes = fabs(	acos(RVLDotProduct(((CRVL3DSurface2 *)(pSegment->m_vp3DSurface))->m_N,
										 p3DGround->m_N)));
		}

	}	
}


void CRVLPlanarSurfaceDetector::DisplayInvisibleBoundaries(	CRVLGUI *pGUI,
															CRVLFigure *pFig)
{
	BYTE **DelaunayMap = m_pDelaunay->m_DelaunayMap;

	CRVLDisplayVector Vector(pFig->m_pMem);
	
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	
	Vector.m_rL = 0;
	Vector.m_gL = 255;
	Vector.m_bL = 0;	
	Vector.m_LineWidth = 1;

	RVL3DPOINT2 *pPoint3DArrayEnd = m_Point3DArray + m_n3DPoints;
	
	//void **ppTmp;
	//void *vpRegion;
	//void **ppRegion, **ppRegion2;
	//void **pRegionPtrBuff1End;
	RVLMESH_LINK *ConnectionArray;
	//RVLMESH_LINK *ConnectionArray2;
	RVLMESH_LINK *pConnection;
	//RVLMESH_LINK *pConnection2;
	short iNeighbor;
	short nNeighbors;
	//int nRegions;

	//CRVLMPtrChain *pRegionList;

	// for every point in m_Point3DArray

	RVL3DPOINT2 *pPoint3D;
	BYTE *pDelaunayData;
	int iPix, iPix2;
	int u, v, u2, v2;
	//int du, dv;
	CRVLDisplayVector *pVector;

//#ifdef NEVER
	for(pPoint3D = m_Point3DArray; pPoint3D < pPoint3DArrayEnd; pPoint3D++)
	{
		iPix = pPoint3D->iPix;

		u = iPix % m_Width;
		v = iPix / m_Width;

		pDelaunayData = DelaunayMap[iPix];

		ConnectionArray = pConnection = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		nNeighbors = *((short *)pDelaunayData);

		// for every connection of pPoint3D

		for(iNeighbor = 0; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
		{
			iPix2 = pConnection->iPix;

			//if(iPix2 < iPix)
			//	continue;

			u2 = iPix2 % m_Width;
			v2 = iPix2 / m_Width;

			//du = u2 - u;
			//dv = v2 - v;

			//if(du * du + dv * dv <= 2)
			//	continue;

			//pDelaunayData = DelaunayMap[iPix2];

			//ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

			//pConnection2 = ConnectionArray2 + pConnection->iConnection;			

			if((pConnection->Flags & RVLMESH_LINK_FLAG_BOUNDARY) == 0)
			{
				pVector = pFig->AddVector(&Vector);

				pVector->m_rL = 255;
				pVector->m_gL = 0;
				pVector->m_bL = 0;

				pVector->Line((u << 1) + 1, (v << 1) + 1, (u2 << 1) + 1, (v2 << 1) + 1);		
			}
		}
	}
//#endif NEVER

	for(pPoint3D = m_Point3DArray; pPoint3D < pPoint3DArrayEnd; pPoint3D++)
	{
		iPix = pPoint3D->iPix;

		u = iPix % m_Width;
		v = iPix / m_Width;

		pDelaunayData = DelaunayMap[iPix];

		ConnectionArray = pConnection = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		nNeighbors = *((short *)pDelaunayData);

		// for every connection of pPoint3D

		for(iNeighbor = 0; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
		{
			iPix2 = pConnection->iPix;

			u2 = iPix2 % m_Width;
			v2 = iPix2 / m_Width;

			if(pConnection->Flags & RVLMESH_LINK_FLAG_BOUNDARY)
			{
				pVector = pFig->AddVector(&Vector);

				pVector->m_rL = 0;
				pVector->m_gL = 255;
				pVector->m_bL = 0;

				pVector->Line((u << 1) + 1, (v << 1) + 1, (u2 << 1) + 1, (v2 << 1) + 1);		
			}
		}
	}
}


RVL3DPOINT2* CRVLPlanarSurfaceDetector::FindNearest3DPoint(int u, int v, CRVLGUI *pGUI,CRVLC2D *p2DRegionSet)
{
	double minDist = m_Height*m_Width;
	double dist;

	int uDif,vDif;
	
	RVL3DPOINT2 *pNearestPoint3D = NULL;

	RVL3DPOINT2 *pPoint3D;
	
	RVL3DPOINT2 *pPoint3DArrayEnd = m_Point3DArray + m_n3DPoints;

	for(pPoint3D = m_Point3DArray; pPoint3D < pPoint3DArrayEnd; pPoint3D++)
	{
		uDif = u - pPoint3D->u;
		vDif = v - pPoint3D->v;
		dist = sqrt(uDif*uDif*1.0 + vDif*vDif*1.0);

		if(dist<minDist)
		{
			minDist = dist;
			pNearestPoint3D = pPoint3D;
		}
	}


	if((m_Flags & RVLPSD_FLAG_SUBSEGMENT) ==0 ) //we have to create relation between the selected 3D point and its corresponding segments
	{
		CRVL2DRegion2 *p2DSegment;
		RVL3DPOINT2 **ppP3D, **ppP3DEnd;
		

		CRVLMPtrChain segmentList = p2DRegionSet->m_ObjectList;

		//For each segment  (main loop)
		segmentList.Start();

		while(segmentList.m_pNext)
		{
			//get segment
			p2DSegment = (CRVL2DRegion2 *)(segmentList.GetNext());

			ppP3D = p2DSegment->m_pPoint3DArray;
			ppP3DEnd = ppP3D + p2DSegment->m_n3DPts;

			//for each 3D point
			for(; ppP3D < ppP3DEnd; ppP3D++)
			{
				pPoint3D = *ppP3D;

				if(pPoint3D == pNearestPoint3D)
				{
					pNearestPoint3D->regionList.Add(p2DSegment);
				}

			}
		}
	}
	
			
	
	//set values
	pGUI->m_nSteps = pNearestPoint3D->regionList.m_nElements;
	pGUI->m_iStep = 0;


	return pNearestPoint3D;

}


//1. Iterate through SegmentBuffer and generate planar surfaces ie create subsegments
//2. Assign region ptrs. to Delaunay links   
//3. Create relation between neighbouring segments and add to SortedSegmentBuffer   

void CRVLPlanarSurfaceDetector::SubSegment(CRVLC2D *p2DRegionSet,
										   CRVL2DRegion2 **ppSegmentBuffer,
										   RVL3DPOINT2 **ppPtrArrayStart,
										   CRVLMPtrChain **SortedSegmentBuffer,
										   int &nMax)
{
	//RVLOBJ2_FLAG_MARKED		-> indicates that the point cannot be used as a FIRST/STARTING point
	//RVLOBJ2_FLAG_MARKED2		-> indicates that a point has been initially assigned to a plane when using Delaunay links
	//RVLOBJ2_FLAG_MARKED3		-> indicates that a point has been finally assigned to a plane


	int i,j;
	int nOriginalRegions = p2DRegionSet->m_ObjectList.m_nElements;



	/***********************************************************************/
	/* 1. Iterate through SegmentBuffer and generate planar surfaces */
	/***********************************************************************/

	BYTE *pDelaunayData;
	short nNeighbors,iNeighbor;
	RVLMESH_LINK *pConnection;
	int iPix;

	RVL3DPOINT2 **CurrentSegment3DPtrArray = new RVL3DPOINT2 *[m_n3DPoints]; //contains all the 3D points for a given segment
	RVL3DPOINT2 **Unassigned3DPtrArray = new RVL3DPOINT2 *[m_n3DPoints];	 //contains all the 3D points for a given segment that have NOT been assigned to a plane
	RVL3DPOINT2 **Assigned3DPtrArray = new RVL3DPOINT2 *[2 * m_n3DPoints];	 //contains all the 3D points for a given segment that have been assigned to a plane
	RVL3DPOINT2 **ppTempPt, **ppTempPtHelper;
	
	CRVL2DRegion2 planarRegion;
	CRVL2DRegion2 *p2DSegment;
	CRVL2DRegion2 **pp2DSuperSegment;

	RVL3DPOINT2 *pPoint3D;
	RVL3DPOINT2 *pPoint3D1;
	RVL3DPOINT2 *pPoint3D2;
	RVL3DPOINT2 *pPoint3D3;

	RVL3DPOINT2 **ppPoint3D;

	int nUnassigned3DPts = 0;
	int nCurrentSegment3DPoints = 0;
			
	bool bSecondPt = false;
	bool bThirdPt = false;
	bool bAll3PointsInPlane = false;

	bool bInitial = false;

	double e;
	
	CRVLMPtrChain **TempSortedSegmentBuffer = new CRVLMPtrChain *[m_n3DPoints];
	memset(TempSortedSegmentBuffer, 0x00, m_n3DPoints * sizeof(CRVLMPtrChain *));
	
	CRVLMPtrChain *pTempSortedBin, *pTempSortedBinNew;

	CRVLMPtrChain TempSortedBinTemplate;
	TempSortedBinTemplate.m_pMem = m_pMem2;

	int nMax2 = 0;
	int n3DPoints = 0;

	RVLPSD_SEGMENT_DETAILS *PlanarSegmentDetails = new RVLPSD_SEGMENT_DETAILS[m_n3DPoints];
	memset(PlanarSegmentDetails, 0x00, m_n3DPoints * sizeof(RVLPSD_SEGMENT_DETAILS *));
	RVLPSD_SEGMENT_DETAILS *pPlanarSegmentDetails;

	pPlanarSegmentDetails = PlanarSegmentDetails;
	
	RVLPSD_SEGMENT_DETAILS *pSegmentDetails;
	CRVL2DRegion2 *pNewSegment;

	int nTrials = 0;
	int nUnassigned3DPts0 = m_n3DPoints;
	
	for(i = 0; i < nOriginalRegions; i++,ppSegmentBuffer++)   //Go through all original regions
	{
		//get region
		p2DSegment =  *ppSegmentBuffer;

		if(p2DSegment->m_n3DPts > 20) //No need to go below 20 points
		{

			nCurrentSegment3DPoints = p2DSegment->m_n3DPts;

			//copy all its 3d points to buffer
			memcpy(CurrentSegment3DPtrArray,p2DSegment->m_pPoint3DArray,nCurrentSegment3DPoints * sizeof(RVL3DPOINT2 *));

			//mark all 3D points belonging to segment (default is -1)
			ppPoint3D = CurrentSegment3DPtrArray;
			for(j=0; j<nCurrentSegment3DPoints; j++,ppPoint3D++)
				(*ppPoint3D)->refSegmentNumber = 0;
				

			//initialize temp array
			nUnassigned3DPts = GetUnassigned3DPoints(CurrentSegment3DPtrArray,Unassigned3DPtrArray,nCurrentSegment3DPoints);

			//reset sort buffer
			memset(TempSortedSegmentBuffer, 0x00, m_n3DPoints * sizeof(CRVLMPtrChain *));
			
			nMax2 = 0;

			nTrials = 0;
			nUnassigned3DPts0 = m_n3DPoints;


			//Get segment planes
			while(nUnassigned3DPts>0)
			{
				//Get random 3 points
				//1st point from unassigned points
				pPoint3D1 = *(Unassigned3DPtrArray + RVLRandom(0, nUnassigned3DPts - 1));

				//2nd point from the whole set
				do
				{
					pPoint3D2 = *(CurrentSegment3DPtrArray + RVLRandom(0, nCurrentSegment3DPoints - 1));
				}
				while (pPoint3D2==pPoint3D1);
				
				//3rd point from the whole set
				do
				{
					pPoint3D3 = *(CurrentSegment3DPtrArray + RVLRandom(0, nCurrentSegment3DPoints - 1));
				}
				while ((pPoint3D3==pPoint3D1) || (pPoint3D3==pPoint3D2));


				//check to see that nUnassigned3DPts is changing;
				if(nUnassigned3DPts < nUnassigned3DPts0)
					nUnassigned3DPts0 = nUnassigned3DPts;
				else
					nTrials++;

				//exit loop if nUnassigned3DPts does not change in 10 trials
				if (nTrials > 10)
					break;

				//Get Plane
				if(!Plane(pPoint3D1, pPoint3D2, pPoint3D3, &planarRegion))
					continue;
				
				
				//Define planar segment
				
				pPlanarSegmentDetails->a = planarRegion.m_a;
				pPlanarSegmentDetails->b = planarRegion.m_b;
				pPlanarSegmentDetails->c = planarRegion.m_c;
				pPlanarSegmentDetails->m_ObjectList.m_pMem =  m_pMem2;

				//reset values
				bSecondPt = false;
				bThirdPt = false;

				//reset pointers
				ppTempPt = Assigned3DPtrArray;
				ppTempPtHelper = ppTempPt;


				//add first point
				pPoint3D1->Flags |= RVLOBJ2_FLAG_MARKED;
				pPoint3D1->Flags |= RVLOBJ2_FLAG_MARKED2;
				pPlanarSegmentDetails->m_ObjectList.Add(pPoint3D1);
				*(ppTempPt++) = pPoint3D1;
				


				//Get all neighbouring points using Delaunay links
				while(ppTempPtHelper<ppTempPt)
				{
					pPoint3D = *(ppTempPtHelper);
										
					pDelaunayData = m_pDelaunay->m_DelaunayMap[pPoint3D->iPix];

					nNeighbors = *((short *)pDelaunayData);

					pConnection = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

					for(iNeighbor = 0; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
					{
						iPix = pConnection->iPix;
					
						//get 3D point
						pPoint3D = m_Point3DMap[iPix];
						
						//make sure it is in the same segment and that it has not already been added
						if((pPoint3D != NULL) && (pPoint3D->refSegmentNumber == 0) && !(pPoint3D->Flags & RVLOBJ2_FLAG_MARKED2))
						{
							
							//determine tolerance and add to TempArray
							e = pPoint3D->z - (pPlanarSegmentDetails->a * pPoint3D->x + pPlanarSegmentDetails->b * pPoint3D->y + pPlanarSegmentDetails->c);

							if(e > m_Tol)
								continue;

							if(e >= -m_Tol)
							{
								pPoint3D->Flags |= RVLOBJ2_FLAG_MARKED;
								pPoint3D->Flags |= RVLOBJ2_FLAG_MARKED2;
								
								pPlanarSegmentDetails->m_ObjectList.Add(pPoint3D);

								*(ppTempPt++) = pPoint3D;

								if(pPoint3D == pPoint3D2)
									bSecondPt = true;

								if(pPoint3D == pPoint3D3)
									bThirdPt = true;
							}
						}

						
					}
					ppTempPtHelper++;
				}

				//bAll3PointsInPlane = (bSecondPt && bThirdPt);
				bAll3PointsInPlane = TRUE;
				
				//Second and Third point belong to consensus set -> Save the plane properties and add into SortBuffer
				if(bAll3PointsInPlane)
				{
					n3DPoints = ppTempPt - Assigned3DPtrArray;

					//store max value
					if(n3DPoints > nMax2)
						nMax2 = n3DPoints;

					pPlanarSegmentDetails->numberOf3DPoints = n3DPoints;

					//store planarSegmentDetails in TempSortedSegmentBuffer depending on n3DPoints
					//1. get position 
					pTempSortedBin = TempSortedSegmentBuffer[n3DPoints];

					if(pTempSortedBin == NULL)
					{
						pTempSortedBin = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
						*pTempSortedBin = TempSortedBinTemplate;

						TempSortedSegmentBuffer[n3DPoints] = pTempSortedBin;
					}
					
					//2. Add planarSegmentDetails
					pTempSortedBin->Add(pPlanarSegmentDetails);

					//Fill temp array
					nUnassigned3DPts = GetUnassigned3DPoints(CurrentSegment3DPtrArray,Unassigned3DPtrArray,nCurrentSegment3DPoints);
					 
				}
				
				//reset point3D flags
				pPlanarSegmentDetails->m_ObjectList.Start();
				while(pPlanarSegmentDetails->m_ObjectList.m_pNext)
				{
					pPoint3D = (RVL3DPOINT2 *)(pPlanarSegmentDetails->m_ObjectList.GetNext());
					pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED2; //reset

					if(!bAll3PointsInPlane)
						pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED; // point can be used as a FIRST/STARTING point in the future

					
				}
				
						

				//set new SegmentDetail
				if(bAll3PointsInPlane)
					pPlanarSegmentDetails++;
				else
					pPlanarSegmentDetails->m_ObjectList.RemoveAll();
			
			}
			

			bInitial = false;
			
			//sort planes and create new sub segments
			for(j = nMax2; j > 0; j--)  
			{
				pTempSortedBin = TempSortedSegmentBuffer[j];
				
				if(pTempSortedBin == NULL)
					continue;

				pTempSortedBin->Start();
				while(pTempSortedBin->m_pNext)
				{
					//get segment
					pSegmentDetails = (RVLPSD_SEGMENT_DETAILS *)(pTempSortedBin->GetNext());
					
					if((j == nMax2) && (bInitial == false)) //copy biggest plane to current p2DSegment (if there is more than one copy the first one)
					{
						bInitial = true;
						
						CopyPlanarSegmentDetails(p2DSegment,pSegmentDetails,ppPtrArrayStart);

						//adjust pointer for next sub segment
						ppPtrArrayStart += p2DSegment->m_n3DPts;
						
					}
					else //check number of free points and re-sort 
					{
						//check current count
						n3DPoints = 0;
						
						pSegmentDetails->m_ObjectList.Start();
						while(pSegmentDetails->m_ObjectList.m_pNext)
						{
							pPoint3D = (RVL3DPOINT2 *)(pSegmentDetails->m_ObjectList.GetNext());
							if(pPoint3D->regionList.m_nElements == 0)
								n3DPoints++;
						}

						//move p2DSegment  if n3DPoints != pPlanarSegmentDetails2->m_n3DPts
						if(n3DPoints != pSegmentDetails->numberOf3DPoints)
						{
							pSegmentDetails->numberOf3DPoints = n3DPoints;
							
							pTempSortedBinNew = TempSortedSegmentBuffer[n3DPoints];

							if(pTempSortedBinNew == NULL)
							{
								pTempSortedBinNew = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
								*pTempSortedBinNew = TempSortedBinTemplate;

								TempSortedSegmentBuffer[n3DPoints] = pTempSortedBinNew;
							}

							pTempSortedBinNew->Add(pSegmentDetails);
						}
						else //create new 2DRegion and add to current
						{	
							//add segment to RegionSet
							pNewSegment = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet));

							//Set pointer to super segment to NULL
							pp2DSuperSegment = (CRVL2DRegion2 **)(pNewSegment->m_pData + p2DRegionSet->m_iDataGrandParentPtr);
							*pp2DSuperSegment = NULL;

							CopyPlanarSegmentDetails(pNewSegment,pSegmentDetails,ppPtrArrayStart);

							//adjust pointer for next sub segment
							ppPtrArrayStart += pNewSegment->m_n3DPts;
							
						}
						
						}
						
					}
					
				}
			}


			//reset 3D points belonging to segment (default is -1)
			ppPoint3D = CurrentSegment3DPtrArray;
			for(j=0; j<nCurrentSegment3DPoints; j++,ppPoint3D++)
			{
				(*ppPoint3D)->refSegmentNumber = -1;
				(*ppPoint3D)->Flags &= ~RVLOBJ2_FLAG_MARKED;
				(*ppPoint3D)->Flags &= ~RVLOBJ2_FLAG_MARKED2;
				(*ppPoint3D)->Flags &= ~RVLOBJ2_FLAG_MARKED3;
			}

	}

	

	
	
	
	delete[] TempSortedSegmentBuffer;
	delete[] Unassigned3DPtrArray;
	delete[] Assigned3DPtrArray;
	delete[] CurrentSegment3DPtrArray;
	delete[] PlanarSegmentDetails;




	/*********************************************************/
	/* 2. Assign region ptrs. to Delaunay links              */
	/*********************************************************/
	
#ifdef NEVER

	BYTE **DelaunayMap = m_pDelaunay->m_DelaunayMap;

	// set vpRegion of every border connection to 0xffffffff

	pDelaunayData = DelaunayMap[0];

	nNeighbors = *((short *)pDelaunayData);

	RVLMESH_LINK *ConnectionArray = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

	pConnection = ConnectionArray;

	while(pConnection->iPix != m_Width)
		pConnection++;
	
	short iConnection2 = pConnection - ConnectionArray;

	RVLMESH_LINK *pConnection2 = pConnection;

	do
	{
		pConnection2->vpRegion = (void *)(0xffffffff);

		pConnection2 = ConnectionArray + ((iConnection2 + 1) % nNeighbors);

		pDelaunayData = DelaunayMap[pConnection2->iPix];

		nNeighbors = *((short *)pDelaunayData);

		ConnectionArray = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		iConnection2 = pConnection2->iConnection;

		pConnection2 = ConnectionArray + iConnection2;
	}
	while(pConnection2 != pConnection);

	// set vpRegion pointers of other connections

	void **RegionPtrBuff = new void *[4 * (m_Width * m_Height)];

	void **RegionPtrBuff1 = RegionPtrBuff;
	void **RegionPtrBuff2 = RegionPtrBuff + 2 * (m_Width * m_Height);

	RVL3DPOINT2 *pPoint3DArrayEnd = m_Point3DArray + m_n3DPoints;
	
	void **ppTmp;
	void *vpRegion;
	void **ppRegion, **ppRegion2;
	void **pRegionPtrBuff1End;
	RVLMESH_LINK *ConnectionArray2;
	short nNeighbors2;
	int nRegions;

	CRVLMPtrChain *pRegionList;

	// for every point in m_Point3DArray

	for(pPoint3D = m_Point3DArray; pPoint3D < pPoint3DArrayEnd; pPoint3D++)
	{
		pDelaunayData = DelaunayMap[pPoint3D->iPix];

		ConnectionArray = pConnection = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

		nNeighbors = *((short *)pDelaunayData);

		// for every connection of pPoint3D

		for(iNeighbor = 0; iNeighbor < nNeighbors; iNeighbor++, pConnection++)
		{
			if(pConnection->vpRegion)
				continue;

			pRegionList = &(pPoint3D->regionList);

			nRegions = pRegionList->m_nElements;

			ppRegion = RegionPtrBuff1;

			pRegionList->Start();

			while(pRegionList->m_pNext)
				*(ppRegion)++ = pRegionList->GetNext();

			//iConnection2 = iNeighbor;

			//ConnectionArray2 = ConnectionArray;

			//nNeighbors2 = nNeighbors;

			pConnection2 = ConnectionArray + ((iNeighbor + 1) % nNeighbors);

			iPix = pConnection2->iPix;

			pDelaunayData = DelaunayMap[iPix];

			nNeighbors2 = *((short *)pDelaunayData);

			ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

			iConnection2 = pConnection2->iConnection;

			pConnection2 = ConnectionArray2 + iConnection2;

			do
			{
				pPoint3D2 = m_Point3DMap[iPix];

				if(pPoint3D2 == NULL)
				{
					nRegions = 0;

					break;
				}

				pRegionList = &(pPoint3D2->regionList);	

				pRegionPtrBuff1End = RegionPtrBuff1 + nRegions;

				ppRegion2 = RegionPtrBuff2;

				pRegionList->Start();

				while(pRegionList->m_pNext)
				{
					vpRegion = pRegionList->GetNext();

					for(ppRegion = RegionPtrBuff1; ppRegion < pRegionPtrBuff1End; ppRegion++)
						if(*ppRegion == vpRegion)
						{
							*(ppRegion2++) = vpRegion;

							break;
						}
				}

				nRegions = ppRegion2 - RegionPtrBuff2;

				if(nRegions == 0)
					break;

				ppTmp = RegionPtrBuff1;
				RegionPtrBuff1 = RegionPtrBuff2;
				RegionPtrBuff2 = ppTmp;

				pConnection2 = ConnectionArray2 + ((iConnection2 + 1) % nNeighbors2);

				iPix = pConnection2->iPix;

				pDelaunayData = DelaunayMap[iPix];

				nNeighbors2 = *((short *)pDelaunayData);

				ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

				iConnection2 = pConnection2->iConnection;

				pConnection2 = ConnectionArray2 + iConnection2;
			}
			while(pConnection2 != pConnection);

			vpRegion = (nRegions == 1 ? RegionPtrBuff1[0] : (void *)(0xffffffff));

			pConnection2 = pConnection;

			iConnection2 = iNeighbor;

			nNeighbors2 = nNeighbors;

			ConnectionArray2 = ConnectionArray;

			do
			{
				pConnection2->vpRegion = vpRegion;

				pConnection2 = ConnectionArray2 + ((iConnection2 + 1) % nNeighbors2);

				pDelaunayData = DelaunayMap[pConnection2->iPix];

				nNeighbors2 = *((short *)pDelaunayData);

				ConnectionArray2 = (RVLMESH_LINK *)(pDelaunayData + sizeof(short));

				iConnection2 = pConnection2->iConnection;

				pConnection2 = ConnectionArray2 + iConnection2;
			}
			while(pConnection2 != pConnection);
		}
	}

	delete[] RegionPtrBuff;
#endif NEVER
	

	/***************************************************************************************/
	/* 3. Create relation between neighbouring segments and add to SortedSegmentBuffer     */
	/***************************************************************************************/

	CRVLMPtrChain *pSortedBin;

	CRVLMPtrChain SortedBinTemplate;
	SortedBinTemplate.m_pMem = m_pMem2;

	


	CRVLRelation *pRelation, *pRelation2;
	CRVLMPtrChain2 RelList, RelList2;
	
	
	
	
	CRVLMPtrChain *pSegmentList;
	CRVL2DRegion2 *p2DSegment2,*p2DSegmentNeighbor;  

	bool bRelExists = false;

	//Get region list
	CRVLMPtrChain segmentList = p2DRegionSet->m_ObjectList;

	//Go through all regions (original + new)
	segmentList = p2DRegionSet->m_ObjectList;

	//For each region  (main loop)
	segmentList.Start();

	while(segmentList.m_pNext)
	{
		//get segment
		p2DSegment = (CRVL2DRegion2 *)(segmentList.GetNext());

		//reset flags
		p2DSegment->m_Flags &= ~RVL2DREGION_FLAG_MARKED;
		p2DSegment->m_Flags &= ~RVL2DREGION_FLAG_MARKED2;
		
		//get nMax
		n3DPoints = p2DSegment->m_n3DPts;

		if(n3DPoints > nMax)
			nMax = n3DPoints;

		RelList.Create(p2DSegment->m_RelList + p2DSegment->m_pClass->m_iRelListNeighbors, m_pMem);

		//Re-create relation list
		//Go through all 3D points and create relation list for those points related to more than one plane/region/segment
		ppPoint3D = p2DSegment->m_pPoint3DArray;

		for(j=0; j<n3DPoints; j++,ppPoint3D++)
		{
			pPoint3D = *ppPoint3D;

			pSegmentList = &(pPoint3D->regionList);

			//check if we have a region list
			if(pSegmentList->m_nElements > 1)
			{
				//Go through region List
				pSegmentList->Start();
				while(pSegmentList->m_pNext)
				{
					p2DSegment2 = (CRVL2DRegion2 *)(pSegmentList->GetNext());

					if(p2DSegment == p2DSegment2)
						continue;

					if(p2DSegment2->m_Flags & RVL2DREGION_FLAG_MARKED)
						continue;

					//check to see that a relation does not already exist
					bRelExists = false;

					RelList2.Create(p2DSegment2->m_RelList + p2DSegment2->m_pClass->m_iRelListNeighbors, m_pMem);

					RelList2.Start();

					while(RelList2.m_pNext)
					{
						pRelation2 = (CRVLRelation *)(RelList2.GetNext());
					
						p2DSegmentNeighbor = (CRVL2DRegion2 *)(pRelation2->m_pObject[0]);

						if(p2DSegmentNeighbor == p2DSegment)
						{
							bRelExists = true;
							break;
						}
							
					}
					
					if(bRelExists)
						continue;

					//create relation
					pRelation=(CRVLRelation *)(m_pMem->Alloc(sizeof(CRVLRelation)));

					p2DSegment2->m_Flags |= RVL2DREGION_FLAG_MARKED;

					pRelation->m_pObject[0] = p2DSegment;
					pRelation->m_pObject[1] = p2DSegment2;

					RelList2.Add(pRelation);

					RelList.Add(pRelation);
				}
			
			}
		}

		//reset flags 
		RelList.Start();
		while(RelList.m_pNext)
		{
			pRelation = (CRVLRelation *)(RelList.GetNext());

			p2DSegment2 = (CRVL2DRegion2 *)(pRelation->m_pObject[1]);

			p2DSegment2->m_Flags &= ~RVL2DREGION_FLAG_MARKED;
			
		}
			
	
		//store region in SortedSegmentBuffer depending on n3DPoints
		//1. get position 
		pSortedBin = SortedSegmentBuffer[n3DPoints];

		if(pSortedBin == NULL)
		{
			pSortedBin = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
			*pSortedBin = SortedBinTemplate;

			SortedSegmentBuffer[n3DPoints] = pSortedBin;
		}

		
		//2. Set flag
		p2DSegment->m_Flags |= RVL2DREGION_FLAG_MARKED;
		//3. Add region
		pSortedBin->Add(p2DSegment);

	}
	



}




void CRVLPlanarSurfaceDetector::SegmentGrowing(CRVL2DRegion2 **GroupedRegionList,
											   CRVL2DRegion2 **pp2DSegment,
											   CRVL2DRegion2 *p2DSegmentLevel3,
											   RVL3DPOINT2 **ConsensusSet,
											   int &nList,
											   DWORD Flags)
{
	double minPerc = 0.5;  //determines the min. percentage of points that a neighbour segment 
						   //must have in order to be added to the main segment

	CRVL2DRegion2 *p2DSegment, *p2DRegionNeighbor, *p2DRegionHelper;

	CRVL2DRegion2 **ppGroupedRegionList, **ppGroupedRegionListHelper;

	CRVLRelation *pRelation;
	CRVLMPtrChain2 RelList;

	RVL3DPOINT2 **ppConsensusSet;
	
	int nConsensus, nConsenusMax;
	int n3DPoints;

	double Score, ScorePerc;
	double BestScore = 0.0;


	//perform initial fitting 
	if(nList > 1)
		LSPlane(ConsensusSet, p2DSegmentLevel3->m_n3DPts, p2DSegmentLevel3);


	ppConsensusSet = ConsensusSet;

	//get best segment (from previous session)
	p2DSegment = *pp2DSegment;

	ppGroupedRegionList = GroupedRegionList;
	ppGroupedRegionListHelper = GroupedRegionList;

	Consensus(p2DSegment->m_pPoint3DArray, p2DSegment->m_n3DPts, p2DSegment, ppConsensusSet, nConsensus, Flags, BestScore);

	ppConsensusSet += nConsensus;

	nConsenusMax = nConsensus;

	
	//add first segment
	p2DSegment->m_Flags |= RVL2DREGION_FLAG_MARKED2;
	*(ppGroupedRegionList++) = p2DSegment;
	

	
	while(ppGroupedRegionListHelper < ppGroupedRegionList)
	{
		p2DRegionHelper = *ppGroupedRegionListHelper;
	
		//1. Get neighbours
		RelList.Create(p2DRegionHelper->m_RelList + p2DRegionHelper->m_pClass->m_iRelListNeighbors, NULL);

		RelList.Start();

		while(RelList.m_pNext)
		{
			pRelation = (CRVLRelation *)(RelList.GetNext());
		
			p2DRegionNeighbor = (CRVL2DRegion2 *)(pRelation->m_pObject[0]);

			if(p2DRegionNeighbor == p2DRegionHelper)
				p2DRegionNeighbor = (CRVL2DRegion2 *)(pRelation->m_pObject[1]);


			//Make sure neighbour has not been marked (ie final fitting has not already been performed)
			//Neighbour must have 3D points
			if(((p2DRegionNeighbor->m_Flags & RVL2DREGION_FLAG_MARKED2)!=0) ||  (p2DRegionNeighbor->m_n3DPts == 0))// 
				continue;

			//Check consensus for neighbour using plane parameters of main segment
			

			//Get Score  
			Score = Consensus(p2DRegionNeighbor->m_pPoint3DArray, p2DRegionNeighbor->m_n3DPts, p2DSegment, ppConsensusSet, nConsensus, Flags, BestScore);

			ScorePerc = Score/((double)(p2DRegionNeighbor->m_n3DPts));

			//Add neighbour 3D points from Consenus set to Point3DArray and add neighbour to RegionList
			if(ScorePerc > minPerc)
			{
				//set/move consensus set pointer
				ppConsensusSet += nConsensus;

				p2DRegionNeighbor->m_Flags |= RVL2DREGION_FLAG_MARKED2;

				*(ppGroupedRegionList++) = p2DRegionNeighbor;
				

				//return segment with max consensus to serve as starting point in the next iteration
				if(nConsensus > nConsenusMax)
				{
					nConsenusMax = nConsensus;
					*pp2DSegment = p2DRegionNeighbor;
				}
			}

		}//while RelList.m_pNext

		ppGroupedRegionListHelper++;
	}
	
	n3DPoints = ppConsensusSet - ConsensusSet;
	nList = ppGroupedRegionList - GroupedRegionList;

	p2DSegmentLevel3->m_n3DPts = n3DPoints;
	
}


//void CRVLPlanarSurfaceDetector::Copy3DPointsToSegment(CRVL2DRegion2 *p2DSegment,RVL3DPOINT2 **ppPtrArrayStart)
//{
//	RVLPSD_SEGMENT_DETAILS PlanarSegmentDetails;
//	PlanarSegmentDetails.m_ObjectList.m_pMem =  m_pMem2;
//	PlanarSegmentDetails.m_ObjectList.RemoveAll();
//
//	PlanarSegmentDetails.a = p2DSegment->m_a;
//	PlanarSegmentDetails.b = p2DSegment->m_b;
//	PlanarSegmentDetails.c = p2DSegment->m_c;
//	
//	double e;
//
//	RVL3DPOINT2 *pPoint3D;
//	RVL3DPOINT2 **ppP3D,**ppP3DEnd;
//
//	ppP3D = p2DSegment->m_pPoint3DArray;
//	ppP3DEnd = ppP3D + p2DSegment->m_n3DPts;
//
//
//	for(; ppP3D < ppP3DEnd; ppP3D++)
//	{
//		pPoint3D = *ppP3D;
//
//		//determine tolerance and add to TempArray
//		e = pPoint3D->z - (PlanarSegmentDetails.a * pPoint3D->x + PlanarSegmentDetails.b * pPoint3D->y + PlanarSegmentDetails.c);
//
//		if(e > m_Tol)
//			continue;
//
//		if(e >= -m_Tol)
//			PlanarSegmentDetails.m_ObjectList.Add(pPoint3D);
//
//	}
//
//	CopyPlanarSegmentDetails(p2DSegment,&PlanarSegmentDetails,ppPtrArrayStart);
//
//}




bool CRVLPlanarSurfaceDetector::GetSegment3DPoints(RVL3DPOINT2 **ppPoint3D, int iPix, DWORD Flags)
{
	bool b3DPointFound = false;


	//get 3D point
	RVL3DPOINT2 *pPoint3D;

	pPoint3D = m_Point3DMap[iPix];
	
	if(pPoint3D != NULL)
	{
		if(Flags & RVLPSD_FLAG_SUBSAMPLE)
		{
			if(pPoint3D->u % m_SubSample)
				return b3DPointFound;

			if(pPoint3D->v % m_SubSample)
				return b3DPointFound;

			pPoint3D->regionList.m_pMem =  m_pMem2;
			
			//reset values
			pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED;
			pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED2;
			pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED3;
			
			//add 3D pt
			*ppPoint3D = pPoint3D;
			b3DPointFound = true;
		}
		else
		{
			pPoint3D->regionList.m_pMem =  m_pMem2;
			
			//reset values
			pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED;
			pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED2;
			pPoint3D->Flags &= ~RVLOBJ2_FLAG_MARKED3;
			
			//add 3D pt
			*ppPoint3D = pPoint3D;
			b3DPointFound = true;
		}
	}

	return b3DPointFound;

}

//Planar surface segmentation using 2D image segments as initial region
//Divides image segments into planar surfaces/regions





void CRVLPlanarSurfaceDetector::SegmentRB4(CRVLC2D *p2DRegionSet1,			// Input/Output: input image 2D segments (level1)
										   CRVLC2D *p2DRegionSet2,			// Input/Output: output image of 2D supersegments (level2)
										   CRVLC2D *p2DRegionSet3,			// Input/Output: output image of 2D supersegments (level3)
										   CRVLC2DContour *p2DContourSet,	// Contour set
										   DWORD Flags)
{
	//Standard algorithm
	//1. Get 3D points for each (LEVEL1) segment
	//2. Add to Sort Buffer using number of 3D points
	//3. Starting with the biggest segment perform RANSAC and determine consensus set
	//4. Resort using size of consensus set
	//5. Perform region growing and form LEVEL3 segments/planes
	//6. Iterate through region list and and form LEVEL2 segments
	//7. Determine edges (contours) of LEVEL2 segments using Delaunay connections


	//RVLPSD_FLAG_SUBSEGMENT -> indicates that Delaunay and Reliable disparities are used 


	//RVL2DREGION_FLAG_MARKED	-> indicates RANSAC+sorting has been performed
	//RVL2DREGION_FLAG_MARKED2	-> indicates final fitting has been performed
	//RVL2DREGION_FLAG_MARKED3	-> indicates a LEVEL1 segment has been added to a LEVEL2 segment
	


	//Delete all LEVEL3/LEVEL2 segments
	p2DRegionSet3->m_ObjectList.RemoveAll();
	p2DRegionSet2->m_ObjectList.RemoveAll();




	RVL3DPOINT2 **Point3DPtrArray = (RVL3DPOINT2 **)(m_pMem->Alloc(6 * m_n3DPoints * sizeof(RVL3DPOINT2)));  //contains all the 3D points for all the regions
	RVL3DPOINT2 **ppPtrArrayStart;	//points to the initial position of the buffer for the 3D of a given region

	RVL3DPOINT2 **ppPoint3D;
	
	//RVL3DPOINT2 *pPoint3D;

	int i,j,index;
	int *ptr;
		
	int n3DPoints = 0;
	int nPoints = 0;
	int nMax = 0;
	
	CRVLMPtrChain2 RelList;

	int nOriginalRegions = p2DRegionSet1->m_ObjectList.m_nElements;

	
	CRVL2DRegion2 **SegmentBufferLevel1 = new CRVL2DRegion2 *[nOriginalRegions];
	memset(SegmentBufferLevel1, 0x00, nOriginalRegions * sizeof(CRVL2DRegion2 *));
	CRVL2DRegion2 **ppSegmentBufferLevel1;
	
	ppSegmentBufferLevel1 = SegmentBufferLevel1;


	CRVLMPtrChain **SortedSegmentBuffer = new CRVLMPtrChain *[m_n3DPoints];
	memset(SortedSegmentBuffer, 0x00, m_n3DPoints * sizeof(CRVLMPtrChain *));
	CRVLMPtrChain SortedBinTemplate;
	SortedBinTemplate.m_pMem = m_pMem2;

	CRVLMPtrChain *pSortedBin;

	CRVL2DRegion2 *p2DSegment;
	CRVL2DRegion2 *p2DSegmentLevel1;  
	CRVL2DRegion2 *p2DSegmentLevel3, **pp2DSegmentLevel3;

	CRVL2DRegion2 *p2DSegmentLevel2, **pp2DSegmentLevel2;  
	
	//Get segment list
	CRVLMPtrChain segmentList = p2DRegionSet1->m_ObjectList;

	/*********************************************************************************************/
	/* 1. Find number of 3D points for each segment and add to SegmentBuffer/SortedSegmentBuffer */
	/*********************************************************************************************/
	ppPoint3D = Point3DPtrArray;

	//For each segment  (main loop)
	segmentList.Start();

	while(segmentList.m_pNext)
	{
		ppPtrArrayStart = ppPoint3D;

		//get segment
		p2DSegmentLevel1 = (CRVL2DRegion2 *)(segmentList.GetNext());
		

		//Get 3D points in segment
		if(m_bEdgeBasedSegments)
		{
			RVL2DREGION_PIXIDX *pPixPtr = (RVL2DREGION_PIXIDX *)(p2DSegmentLevel1->m_PtArray);

			while(pPixPtr)
			{
				index = pPixPtr->iPix;
				if(GetSegment3DPoints(ppPoint3D, index, Flags))
					ppPoint3D++;

				pPixPtr = pPixPtr->pNext;
			}
		}
		else
		{
			ptr = (int *)p2DSegmentLevel1->m_PtArray;
			for (i=0; i< p2DSegmentLevel1->m_nPts; i++,ptr++)
			{
				index = (*ptr);
				if(GetSegment3DPoints(ppPoint3D, index, Flags))
					ppPoint3D++;
			}
		}

		n3DPoints = ppPoint3D - ppPtrArrayStart;

		if(n3DPoints >nMax)
			nMax = n3DPoints;

		//set number of points
		p2DSegmentLevel1->m_n3DPts = n3DPoints;
		
		//set region pointer
		p2DSegmentLevel1->m_pPoint3DArray = ppPtrArrayStart;

		//reset values
		p2DSegmentLevel1->m_Flags &= ~RVL2DREGION_FLAG_MARKED;
		p2DSegmentLevel1->m_Flags &= ~RVL2DREGION_FLAG_MARKED2;
		p2DSegmentLevel1->m_Flags &= ~RVL2DREGION_FLAG_MARKED3;

		//Set pointer to LEVEL3 segment to NULL
		pp2DSegmentLevel3 = (CRVL2DRegion2 **)(p2DSegmentLevel1->m_pData + p2DRegionSet1->m_iDataGrandParentPtr);
		*pp2DSegmentLevel3 = NULL;


		//Set pointer to LEVEL2 segment to NULL
		pp2DSegmentLevel2 = (CRVL2DRegion2 **)(p2DSegmentLevel1->m_pData + p2DRegionSet1->m_iDataParentPtr);
		*pp2DSegmentLevel2 = NULL;

		
		if((m_Flags & RVLPSD_FLAG_SUBSEGMENT) !=0 ) //Clear relation List and add to SegmentBuffer
		{			
			RelList.Create(p2DSegmentLevel1->m_RelList + p2DSegmentLevel1->m_pClass->m_iRelListNeighbors, m_pMem);
			RelList.RemoveAll();

			//store region in RegionBuffer
			*(ppSegmentBufferLevel1++) = p2DSegmentLevel1;
		}
		else // add to SortedSegmentBuffer depending on n3DPoints
		{
			//1. get position 
			pSortedBin = SortedSegmentBuffer[n3DPoints];

			if(pSortedBin == NULL)
			{
				pSortedBin = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
				*pSortedBin = SortedBinTemplate;

				SortedSegmentBuffer[n3DPoints] = pSortedBin;
			}
			

			//2. Add region
			pSortedBin->Add(p2DSegmentLevel1);
		
		}

	}



	//point to final position so that new segments can be added
	ppPtrArrayStart = ppPoint3D;

	//reset pointer to SegmentBuffer
	ppSegmentBufferLevel1 = SegmentBufferLevel1;

	/*********************************************************************************************/
	/* 2. Subsegment image if the flag has been set and add to SortedSegmentBuffer				 */
	/*********************************************************************************************/
	if((m_Flags & RVLPSD_FLAG_SUBSEGMENT) !=0 )
		SubSegment(p2DRegionSet1,ppSegmentBufferLevel1,ppPtrArrayStart, SortedSegmentBuffer,nMax);
	

	double StartTime = m_pTimer->GetTime();


	/*******************************************************************************************************************/
	/* 3. Iterate through SortedSegmentBuffer and perform region/segment growing with RANSAC  to create LEVEL3 segments*/
	/*******************************************************************************************************************/

	//RVL2DREGION_FLAG_MARKED -> indicates RANSAC+sorting has been performed
	//RVL2DREGION_FLAG_MARKED2 -> indicates final fitting has been performed

	// mark border for not to be considered in region groupping

	int iPix0 = (m_pDelaunay->m_pROI ? m_pDelaunay->m_pROI->left + m_pDelaunay->m_pROI->top * m_Width : 0);

	RVLMESH_LINK *pDelaunayLink = (RVLMESH_LINK *)(m_pDelaunay->m_DelaunayMap[iPix0] + sizeof(short));

	int iPix1 = iPix0 + m_Width;

	while(pDelaunayLink->iPix != iPix1)
		pDelaunayLink++;

	p2DSegment = (CRVL2DRegion2 *)(pDelaunayLink->vp2DRegion);

	p2DSegment->m_Flags |= (RVL2DREGION_FLAG_MARKED | RVL2DREGION_FLAG_MARKED2);

	//double d0 = (double)(m_pStereoVision->m_DisparityOffset) - 
	//					(m_pStereoVision->m_pCameraL->CenterXNrm - 
	//					m_pStereoVision->m_pCameraR->CenterXNrm);

	

	//contains LEVEL1 segments grouped by LEVEL3 segments for RANSAC
	CRVL2DRegion2 **GroupedRegionList31 = (CRVL2DRegion2 **)(m_pMem->Alloc(p2DRegionSet1->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	memset(GroupedRegionList31, 0x00, p2DRegionSet1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

	//contains set of grouped (sub) segments for RANSAC
	CRVL2DRegion2 **GroupedRegionListHelper1 = new CRVL2DRegion2 *[p2DRegionSet1->m_ObjectList.m_nElements]; 
	memset(GroupedRegionListHelper1, 0x00, p2DRegionSet1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

	//contains set of grouped (sub) segments for RANSAC
	CRVL2DRegion2 **GroupedRegionListHelper2 = new CRVL2DRegion2 *[p2DRegionSet1->m_ObjectList.m_nElements]; 
	memset(GroupedRegionListHelper2, 0x00, p2DRegionSet1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

	RVL3DPOINT2 **ConsensusSet = new RVL3DPOINT2 *[m_n3DPoints];   //contains 3D points of segment + neighbours
	
	CRVL2DRegion2 **ppGroupedRegionList, **ppGroupedRegionStart, **ppGroupedRegion;
	CRVL2DRegion2 **ppGroupedRegionListHelper, **ppGroupedRegionListHelper1, **ppGroupedRegionListHelper2, **ppGroupedRegionListBest;
	

	int nSegmentLevel3Consensus;
	

	CRVLMPtrChain *pSortedBinNew;
	
	CRVL2DRegion2 **Segment2DStart = new CRVL2DRegion2 *[1]; 
	CRVL2DRegion2 **pp2DSegmentStart;
	pp2DSegmentStart = Segment2DStart;

	RVLARRAY *pRelList;

	RVLPSDLAD_SEGMENT_DETAILS segmentDetails;
	int nBest = 0;
	int nList = 0;
	int nListBest = 0;

	double aBest = 0.0;
	double bBest = 0.0;
	double cBest = 0.0;

	int n3DPoints_Old = 0;


	ppGroupedRegionList = GroupedRegionList31;
	ppGroupedRegionListHelper1 = GroupedRegionListHelper1;
	ppGroupedRegionListHelper2 = GroupedRegionListHelper2;
	
	
	for(i = nMax; i >= 3; i--)  //No need to go below 3 points, cannot determine plane
	{
		pSortedBin = SortedSegmentBuffer[i];
		
		if(pSortedBin == NULL)
			continue;
		
		pSortedBin->Start();
		while(pSortedBin->m_pNext)
		{
			//get segment
			p2DSegmentLevel1 = (CRVL2DRegion2 *)(pSortedBin->GetNext());

			//a. Make sure it has not been marked (final fitting has not been performed)
			//b. Make sure it has 3D points before performing RANSAC  (This is an unnecessary check if i>=3 otherwise it is!!)
			if(((p2DSegmentLevel1->m_Flags & RVL2DREGION_FLAG_MARKED2)==0) && (p2DSegmentLevel1->m_n3DPts > 0)) 
			{
				//RANSAC and 2nd sorting has not been performed
				if((p2DSegmentLevel1->m_Flags & RVL2DREGION_FLAG_MARKED)==0) 
				{
					
						nBest = RANSAC(p2DSegmentLevel1, false, p2DSegmentLevel1->m_pPoint3DArray, p2DSegmentLevel1->m_n3DPts, &segmentDetails, 
									   Flags | RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET);

						//n3DPoints_Old = p2DSegment->m_n3DPts;

						////Relate 3D points and their segments if sub-segmentation has not been performed
						//if((m_Flags & RVLPSD_FLAG_SUBSEGMENT) ==0 )
						//{
						//	Copy3DPointsToSegment(p2DSegment,ppPtrArrayStart);

						//	//adjust pointer for next sub segment
						//	ppPtrArrayStart += p2DSegment->m_n3DPts;
						//	
						//}

						//Set flag
						p2DSegmentLevel1->m_Flags |= RVL2DREGION_FLAG_MARKED;

						//move p2DRegion lower if nBest < p2DRegion->m_n3DPts
						if(nBest < p2DSegmentLevel1->m_n3DPts)
						{
							pSortedBinNew = SortedSegmentBuffer[nBest];

							if(pSortedBinNew == NULL)
							{
								pSortedBinNew = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
								*pSortedBinNew = SortedBinTemplate;

								SortedSegmentBuffer[nBest] = pSortedBinNew;
							}

							pSortedBinNew->Add(p2DSegmentLevel1);
						}
					
				} 
				//Expand marked regions (RANSAC and 2nd sorting has been performed)
				else 
				{
					//1. Reset temp buffer and fill with current 3D points
					
					//Create LEVEL3 segment
					p2DSegmentLevel3 = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet3));

					aBest = p2DSegmentLevel1->m_a;
					bBest = p2DSegmentLevel1->m_b;
					cBest = p2DSegmentLevel1->m_c;

					//initialize SuperSegment
					p2DSegmentLevel3->m_a = aBest;
					p2DSegmentLevel3->m_b = bBest;
					p2DSegmentLevel3->m_c = cBest;
					p2DSegmentLevel3->m_Tol = m_Tol;

					//start segment is the first segment
					*pp2DSegmentStart = p2DSegmentLevel1;

					nSegmentLevel3Consensus = 0;

					//expand segments until p2DSuperSegment->m_n3DPts remains the same or decreases
					
					ppGroupedRegionStart = ppGroupedRegionList;  
					
					//reset helper arrays
					nListBest = 0;
					memset(GroupedRegionListHelper1, 0x00, p2DRegionSet1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));
					memset(GroupedRegionListHelper2, 0x00, p2DRegionSet1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));
					ppGroupedRegionListHelper = ppGroupedRegionListHelper1;
					ppGroupedRegionListBest = NULL;

					do
					{
						nList = 0;

						//Perform segment growing
						SegmentGrowing(ppGroupedRegionListHelper,pp2DSegmentStart,p2DSegmentLevel3,ConsensusSet,nList,Flags);

						//reset flags
						ppGroupedRegion = ppGroupedRegionListHelper;
						for(j = 0; j < nList; j++, ppGroupedRegion++)
						{
							p2DSegment = *ppGroupedRegion; 
							p2DSegment->m_Flags &= ~RVL2DREGION_FLAG_MARKED2;
						}


						if(p2DSegmentLevel3->m_n3DPts > nSegmentLevel3Consensus)
						{
							nSegmentLevel3Consensus = p2DSegmentLevel3->m_n3DPts;
							
							//store result 
							ppGroupedRegionListBest = ppGroupedRegionListHelper;
							//save number in list
							nListBest = nList;

							//Save params
							 aBest = p2DSegmentLevel3->m_a;
							 bBest = p2DSegmentLevel3->m_b;
							 cBest = p2DSegmentLevel3->m_c;

							//switch helper pointer to new pointer
							ppGroupedRegionListHelper = (ppGroupedRegionListHelper==ppGroupedRegionListHelper1) ? ppGroupedRegionListHelper2 : ppGroupedRegionListHelper1;
							

						}
						else
							break;
						
					}
					while(m_Flags & RVLPSD_FLAG_MAXIMUMREGIONGROWING);

					nPoints = 0;

					if(ppGroupedRegionListBest != NULL)
					{
						//set flags and copy over to SubRegionList
						//updating sub segment params
						ppGroupedRegion = ppGroupedRegionListBest;
						for(j = 0; j < nListBest; j++, ppGroupedRegion++)
						{
							p2DSegment = *ppGroupedRegion; 
							
							p2DSegment->m_Flags |= RVL2DREGION_FLAG_MARKED2;
							*(ppGroupedRegionList++) = p2DSegment;

							nPoints += p2DSegment->m_nPts;

							pp2DSegmentLevel3 = (CRVL2DRegion2 **)(p2DSegment->m_pData + p2DRegionSet1->m_iDataGrandParentPtr);

							*pp2DSegmentLevel3 = p2DSegmentLevel3;
						}

						//LSPlane(ConsensusSet, n3DPoints, p2DSuperSegment);
						
						p2DSegmentLevel3->m_nPts = nPoints;

						p2DSegmentLevel3->m_a = aBest;
						p2DSegmentLevel3->m_b = bBest;
						p2DSegmentLevel3->m_c = cBest;

						pRelList = p2DSegmentLevel3->m_RelList + p2DRegionSet3->m_iRelList[RVLRELLIST_ELEMENTS];

						//updating segment params
						pRelList->pFirst = (unsigned char *)ppGroupedRegionStart;
						pRelList->pEnd = (unsigned char *)ppGroupedRegionList;

					}
				
				} //expand marked regions
				
			}//if(p2DRegion->m_n3DPts > 0) 
		
		}//while(pSortedBin->m_pNext)

	}//for(i = nMax; i >= 3; i--) 


	double ExecutionTime = m_pTimer->GetTime() - StartTime;

	
	
	/****************************/
	/* 4. Create LEVEL2 segments*/
	/****************************/

	//RVL2DREGION_FLAG_MARKED3	-> indicates a LEVEL1 segment has been added to a LEVEL2 segment

	CRVL2DRegion2 *p2DSegmentLevel1Neighbor, *p2DSegmentLevel1Helper;
	CRVL2DRegion2 *p2DCurrentSegmentLevel3;  

	CRVLRelation *pRelation;
	

	//contains LEVEL1 segments grouped by LEVEL2 segments for RANSAC
	CRVL2DRegion2 **GroupedRegionList21 = (CRVL2DRegion2 **)(m_pMem->Alloc(p2DRegionSet1->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	memset(GroupedRegionList21, 0x00, p2DRegionSet1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

	ppGroupedRegionList = GroupedRegionList21;



	////contains LEVEL1 segments grouped by LEVEL2 segments for RANSAC
	//CRVL2DRegion2 **GroupedRegionList32 = (CRVL2DRegion2 **)(m_pMem->Alloc(p2DRegionSet1->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	//memset(GroupedRegionList32, 0x00, p2DRegionSet1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

	//CRVL2DRegion2 **ppGroupedRegionList2, **ppGroupedRegionStart2;
	//ppGroupedRegionList2 = GroupedRegionList32;

	//Delete all LEVEL2 segments
	//p2DRegionSet2->m_ObjectList.RemoveAll();

	segmentList = p2DRegionSet1->m_ObjectList;
	segmentList.Start();
	
	while(segmentList.m_pNext)
	{
		//get segment
		p2DSegmentLevel1 = (CRVL2DRegion2 *)(segmentList.GetNext());

		//get current LEVEL3 segment
		p2DCurrentSegmentLevel3 = *((CRVL2DRegion2 **)(p2DSegmentLevel1->m_pData + p2DRegionSet1->m_iDataGrandParentPtr));

		//if 
		//1. LEVEL3 segment exists 
		//2. LEVEL1 segment has not been grouped into LEVEL2
		//3. LEVEL1 segment is THICK
		//-> start a new LEVEL2 segment group

		if((p2DCurrentSegmentLevel3) && 
		   ((p2DSegmentLevel1->m_Flags & (RVL2DREGION_FLAG_THICK | RVL2DREGION_FLAG_MARKED3))==0))
		{
			//reset helper array
			memset(GroupedRegionListHelper1, 0x00, p2DRegionSet1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

			ppGroupedRegionListHelper1 = GroupedRegionListHelper1;
			ppGroupedRegionListHelper2 = GroupedRegionListHelper1;


			ppGroupedRegionStart = ppGroupedRegionList;
			//ppGroupedRegionStart2 = ppGroupedRegionList2;


			nPoints = 0;

			//Create LEVEL2 segment
			p2DSegmentLevel2 = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet2));

			p2DSegmentLevel2->m_Index = p2DRegionSet2->m_ObjectList.m_nElements - 1;


			//Copy LEVEL3 params to LEVEL2
			p2DSegmentLevel2->m_a = p2DCurrentSegmentLevel3->m_a;
			p2DSegmentLevel2->m_b = p2DCurrentSegmentLevel3->m_b;
			p2DSegmentLevel2->m_c = p2DCurrentSegmentLevel3->m_c;
			


			//mark LEVEL1 segment and add to LEVEL2 group
			p2DSegmentLevel1->m_Flags |= RVL2DREGION_FLAG_MARKED3;
			*(ppGroupedRegionListHelper1++) = p2DSegmentLevel1;
			*(ppGroupedRegionList++) = p2DSegmentLevel1;
			
			pp2DSegmentLevel2 = (CRVL2DRegion2 **)(p2DSegmentLevel1->m_pData + p2DRegionSet1->m_iDataParentPtr);
			*pp2DSegmentLevel2 = p2DSegmentLevel2;

			nPoints += p2DSegmentLevel1->m_nPts;

			//add LEVEL2 segment to LEVEL3 group
			//*(ppGroupedRegionLis2t++) = p2DSegmentLevel2;

			
			while(ppGroupedRegionListHelper2 < ppGroupedRegionListHelper1)
			{
				p2DSegmentLevel1Helper = *ppGroupedRegionListHelper2;

				p2DSegmentLevel3 = *((CRVL2DRegion2 **)(p2DSegmentLevel1Helper->m_pData + p2DRegionSet1->m_iDataGrandParentPtr));

				if(p2DSegmentLevel3 == p2DCurrentSegmentLevel3)
				{
					//1. Get neighbours
					RelList.Create(p2DSegmentLevel1Helper->m_RelList + p2DSegmentLevel1Helper->m_pClass->m_iRelListNeighbors, NULL);

					RelList.Start();

					while(RelList.m_pNext)
					{
						pRelation = (CRVLRelation *)(RelList.GetNext());
					
						p2DSegmentLevel1Neighbor = (CRVL2DRegion2 *)(pRelation->m_pObject[0]);

						if(p2DSegmentLevel1Neighbor == p2DSegmentLevel1Helper)
							p2DSegmentLevel1Neighbor = (CRVL2DRegion2 *)(pRelation->m_pObject[1]);


						p2DSegmentLevel3 = *((CRVL2DRegion2 **)(p2DSegmentLevel1Neighbor->m_pData + p2DRegionSet1->m_iDataGrandParentPtr));
						
						//make sure the LEVEL3 segments are the same and that the segment has not been marked
						if((p2DSegmentLevel3 != p2DCurrentSegmentLevel3) || (p2DSegmentLevel1Neighbor->m_Flags & (RVL2DREGION_FLAG_THICK | RVL2DREGION_FLAG_MARKED3)) != 0)
							continue;

						p2DSegmentLevel1Neighbor->m_Flags |= RVL2DREGION_FLAG_MARKED3;

						*(ppGroupedRegionListHelper1++) = p2DSegmentLevel1Neighbor;

						*(ppGroupedRegionList++) = p2DSegmentLevel1Neighbor;

						pp2DSegmentLevel2 = (CRVL2DRegion2 **)(p2DSegmentLevel1Neighbor->m_pData + p2DRegionSet1->m_iDataParentPtr);
						*pp2DSegmentLevel2 = p2DSegmentLevel2;

						nPoints += p2DSegmentLevel1Neighbor->m_nPts;

					}
				}
				
				ppGroupedRegionListHelper2++;

			}

			
			//updating LEVEL2 segment params (relate LEVEL1 segment list to LEVEL2)			
			pRelList = p2DSegmentLevel2->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_COMPONENTS];
			
			pRelList->pFirst = (unsigned char *)ppGroupedRegionStart;
			pRelList->pEnd = (unsigned char *)ppGroupedRegionList;

			p2DSegmentLevel2->m_nPts = nPoints;
			
			//relate LEVEL2 segment and LEVEL3 segment
			pp2DSegmentLevel3 = (CRVL2DRegion2 **)(p2DSegmentLevel2->m_pData + p2DRegionSet2->m_iDataParentPtr);
			*pp2DSegmentLevel3 = p2DCurrentSegmentLevel3;
			
		}
		
	}
		


	/****************************************************/
	/* 5. Get LEVEL2 (and LEVEL1) Segment Edges using Delaunay links */
	/****************************************************/
	
	//RVLMESH_LINK_FLAG_MARKED -> indicates delaunay edge has been added as edge (contour) of LEVEL2


	//Create temp contour buffer 
	int nLevel2Segments = p2DRegionSet2->m_ObjectList.m_nElements;

	CRVLMPtrChain **ContourBuffer = new CRVLMPtrChain *[nLevel2Segments];
	memset(ContourBuffer, 0x00, nLevel2Segments * sizeof(CRVLMPtrChain *));

	CRVLMPtrChain ContourBinTemplate;
	ContourBinTemplate.m_pMem = m_pMem2;

	CRVLMPtrChain *pContourBin;

	CRVL2DContour *p2DContour;


	//Allocate space in m_pMem to store all the LEVEL1 Delaunay edges
	//RVLMESH_LINK **DelaunayEdgesLevel1Array = (RVLMESH_LINK **)(m_pMem->Alloc(6 * m_Width * m_Height * sizeof(RVLMESH_LINK *))); 
	//memset(DelaunayEdgesLevel1Array, 0x00, 6 * m_Width * m_Height * sizeof(RVLMESH_LINK *));

	//RVLMESH_LINK **ppDelaunayEdgeLevel1;
	
	//int nDelaunayEdgesLevel1 = 0;

	//Allocate space in m_pMem to store all the LEVEL2 contour vertices 
	RVLIPOINT *PointIPtrArrayLevel2 = (RVLIPOINT *)(m_pMem->Alloc(6 * m_Width * m_Height * sizeof(RVLIPOINT)));  //contains all the LEVEL2 contour vertices
	RVLIPOINT *pIPtrArrayStartLevel2;	//points to the initial position of the buffer for the 3D of a given region

	RVLIPOINT *pIPointLevel2;

	int nContours = 0;
	
	
	CRVL2DRegion2 *p2DSegmentLevel11, *p2DSegmentLevel22;
	
	
	int nDelaunayLinks0;
	RVLMESH_LINK *pDelaunayLink0, *DelaunayLink0;
	RVLMESH_LINK *pDelaunayLinkArrayEnd;
	BYTE *pDelaunayData0;
	

	int iConnection1, iConnection2;


	int nDelaunayLinks1;
	RVLMESH_LINK *pDelaunayLink1, *DelaunayLink1;
	BYTE *pDelaunayData1;

	RVLMESH_LINK *pDelaunayLinkIn, *pDelaunayLinkOut;


	pIPointLevel2 = PointIPtrArrayLevel2;
	pDelaunayData0 = m_pDelaunay->m_DelaunayData;
	

	//ppDelaunayEdgeLevel1 = DelaunayEdgesLevel1Array;

	// Go through all delaunay edges
	while(TRUE)
	{
		nDelaunayLinks0 = *((short *)pDelaunayData0);

		if(nDelaunayLinks0 < 1)
			break;

		DelaunayLink0 = (RVLMESH_LINK *)(pDelaunayData0 + sizeof(short));

		pDelaunayLinkArrayEnd = DelaunayLink0 + nDelaunayLinks0;

		for(pDelaunayLink0 = DelaunayLink0; pDelaunayLink0 < pDelaunayLinkArrayEnd; pDelaunayLink0++)
		{
			
			//make sure Delaunay link is an edge and has not been marked
			if((pDelaunayLink0->Flags & RVLMESH_LINK_FLAG_EDGE) && 
			   ((pDelaunayLink0->Flags & RVLMESH_LINK_FLAG_MARKED)==0))
			{
				//store level1 edge
				//*(ppDelaunayEdgeLevel1++) = pDelaunayLink0;
				//nDelaunayEdgesLevel1++;
				
				//make sure edge is between 2 different level2 segments
				
				//Get level2 segment (pDelaunayLink0)
				p2DSegmentLevel11 = (CRVL2DRegion2 *)(pDelaunayLink0->vp2DRegion);
				p2DSegmentLevel22 = *((CRVL2DRegion2 **)(p2DSegmentLevel11->m_pData + p2DRegionSet1->m_iDataParentPtr));

				//if(p2DSegmentLevel22 != NULL)		// cupec
				{
					iPix1 = pDelaunayLink0->iPix;
					iConnection1 = pDelaunayLink0->iConnection;

					pDelaunayData1 = m_pDelaunay->m_DelaunayMap[iPix1];
					DelaunayLink1 = (RVLMESH_LINK *)(pDelaunayData1 + sizeof(short));
					pDelaunayLink1 = DelaunayLink1 + iConnection1;

					//Get level2 segment (pDelaunayLink1)
					p2DSegmentLevel1 = (CRVL2DRegion2 *)(pDelaunayLink1->vp2DRegion);
					p2DSegmentLevel2 = *((CRVL2DRegion2 **)(p2DSegmentLevel1->m_pData + p2DRegionSet1->m_iDataParentPtr));

					
					if((p2DSegmentLevel2 != NULL) && (p2DSegmentLevel22 != p2DSegmentLevel2))
					{
						pIPtrArrayStartLevel2 = pIPointLevel2;

						nDelaunayLinks1 = *((short *)pDelaunayData1);
						iConnection2 = iConnection1;
						pDelaunayLinkIn = pDelaunayLink1;
					
						//mark link
						pDelaunayLinkIn->Flags |= RVLMESH_LINK_FLAG_MARKED;

						do
						{
							do
							{
								
								iConnection2 = (iConnection2 + 1) % nDelaunayLinks1;

								pDelaunayLinkOut = DelaunayLink1 + iConnection2;

								//Get level2 segment (pDelaunayLinkOut)
								p2DSegmentLevel11 = (CRVL2DRegion2 *)(pDelaunayLinkOut->vp2DRegion);
								p2DSegmentLevel22 = *((CRVL2DRegion2 **)(p2DSegmentLevel11->m_pData + p2DRegionSet1->m_iDataParentPtr));

								
							}
							while(!((pDelaunayLinkOut->Flags & RVLMESH_LINK_FLAG_EDGE) && 
								    (p2DSegmentLevel2 != p2DSegmentLevel22)));// && (p2DSegmentLevel22 != NULL) &&
								   //((pDelaunayLinkOut->Flags & RVLMESH_LINK_FLAG_MARKED)==0)));

							//mark link
							pDelaunayLinkOut->Flags |= RVLMESH_LINK_FLAG_MARKED;

							pIPointLevel2->u = pDelaunayLinkIn->iPix % m_Width;
							pIPointLevel2->v = pDelaunayLinkIn->iPix / m_Width;
							pIPointLevel2++;


							pDelaunayData1 = m_pDelaunay->m_DelaunayMap[pDelaunayLinkOut->iPix];

							nDelaunayLinks1 = *((short *)pDelaunayData1);

							DelaunayLink1 = (RVLMESH_LINK *)(pDelaunayData1 + sizeof(short));

							iConnection2 = pDelaunayLinkOut->iConnection;

							pDelaunayLinkIn = DelaunayLink1 + iConnection2;
						}	
						while(pDelaunayLinkIn != pDelaunayLink1);
					

						//create and store contour
						p2DContour = (CRVL2DContour *)(RVL2DContourTemplate.Create3(p2DContourSet));
						p2DContour->m_ContourIPArray = pIPtrArrayStartLevel2;
						p2DContour->m_nContourIPs = pIPointLevel2 - pIPtrArrayStartLevel2;
						nContours++;

						
						//get contour chain
		
						pContourBin = ContourBuffer[p2DSegmentLevel2->m_Index];

						if(pContourBin == NULL)
						{
							pContourBin = (CRVLMPtrChain *)(m_pMem2->Alloc(sizeof(CRVLMPtrChain)));
							*pContourBin = ContourBinTemplate;

							ContourBuffer[p2DSegmentLevel2->m_Index] = pContourBin;
						}
						

						//2. Add contour
						pContourBin->Add(p2DContour);
					}
				}
			
			}
			
		}

		pDelaunayData0 = (BYTE *)pDelaunayLink0;
	}
	

	//resize m_pMem ie get rid of empty space
	m_pMem->m_pFreeMem = (unsigned char *)pIPointLevel2;


	//Save m_DelaunayEdgesLevel1
	//m_DelaunayEdgesLevel1 = DelaunayEdgesLevel1Array;
	//m_nDelaunayEdgesLevel1 = nDelaunayEdgesLevel1;



	//Allocate space in m_pMem to store all the contours 
	CRVL2DContour **ContourArray = (CRVL2DContour **)(m_pMem->Alloc(nContours * sizeof(CRVL2DContour)));  //contains all the contours
	memset(ContourArray, 0x00, nContours * sizeof(CRVL2DContour *));
	CRVL2DContour **ppContourArrayStart;	//points to the initial position

	CRVL2DContour **ppContour;

	ppContour = ContourArray;

	int nContoursCnt = 0;
	
	//copy all contours from ContourBuffer to ContourArray
	segmentList = p2DRegionSet2->m_ObjectList;
	segmentList.Start();
	
	while(segmentList.m_pNext)
	{
		//get segment
		p2DSegmentLevel2 = (CRVL2DRegion2 *)(segmentList.GetNext());

		pContourBin = ContourBuffer[p2DSegmentLevel2->m_Index];
		
		if(pContourBin == NULL)
			continue;

		ppContourArrayStart = ppContour;

		pContourBin->Start();
		while(pContourBin->m_pNext)
		{
			//get contour
			p2DContour = (CRVL2DContour *)(pContourBin->GetNext());
			
			//store contour
			*(ppContour++) = p2DContour;

			nContoursCnt++;
		}


		//updating LEVEL2 segment contour list				
		pRelList = p2DSegmentLevel2->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_CONTOURS];
		
		pRelList->pFirst = (unsigned char *)ppContourArrayStart;
		pRelList->pEnd = (unsigned char *)ppContour;

		
	}
	

	

	/*********************/
	/* 7. Exit elegantly */
	/*********************/
	m_pMem2->Clear();

	delete[] SegmentBufferLevel1;
	
	delete[] SortedSegmentBuffer;
	
	delete[] GroupedRegionListHelper1;
	delete[] GroupedRegionListHelper2;

	delete[] Segment2DStart;

	delete[] ConsensusSet;

	delete[] ContourBuffer;

	

}


void CRVLPlanarSurfaceDetector::SegmentSTRM(CRVLC2D *p2DRegionSet,
											CRVLC2D *p2DRegionSet2, 
											CRVLC2D *p2DRegionSet3, 
											CRVLMem *pMem, 
											DWORD Flags)
{
	double StartTime = m_pTimer->GetTime();

	int ImageSize = m_Width * m_Height;

	memset(m_2DRegionMap, 0, ImageSize * sizeof(CRVL2DRegion2 *));

	if(m_n3DPoints == 0)
		return;

	int err = 0;

	CRVL2DRegion2 *pTriangle2;
	int iLink;
	BOOL bErrCorrection;

#pragma region Convex Hull of the Points with Disparity

	RVL3DPOINT2 **p3DPtMapEnd = m_Point3DMap + ImageSize;

	int u, v;
	RVL3DPOINT2 **pp3DPt;

	for(pp3DPt = m_Point3DMap; pp3DPt < p3DPtMapEnd; pp3DPt++)
		if(*pp3DPt)
			break;

	if (!(*pp3DPt))
		return;

	int Direction = 1;

	int vTop = (pp3DPt - m_Point3DMap) / m_Width;

	if(vTop == m_Height - 1)
		return;

	v = vTop + 1;

	RVLPSD_CHVERTEX *ConvexHull = new RVLPSD_CHVERTEX[2 * m_Height];

	RVLPSD_CHVERTEX *pLastCHVertex = ConvexHull;

	pLastCHVertex->u = (*pp3DPt)->u;
	pLastCHVertex->v = vTop;
	pLastCHVertex->du = -1;
	pLastCHVertex->dv = 0;

	int uStart = 0;
	int uEnd = m_Width;

	int vBottom = vTop;

	int du, dv, len;

	while(Direction == 1 || v >= vTop)
	{
		pp3DPt = m_Point3DMap + v * m_Width + uStart;

		for(u = uStart; u != uEnd; u += Direction, pp3DPt += Direction)
			if(*pp3DPt)
				break;

		if(u != uEnd)
		{
			vBottom = v;

			while(true)
			{
				du = u - pLastCHVertex->u;
				dv = v - pLastCHVertex->v;

				if(-pLastCHVertex->dv * du + pLastCHVertex->du * dv < 0)
				{
					pLastCHVertex++;
					pLastCHVertex->u = u;
					pLastCHVertex->v = v;
					pLastCHVertex->du = du;
					pLastCHVertex->dv = dv;
					break;
				}

				pLastCHVertex--;
			}
		}

		v += Direction;

		if(v == m_Height)
			if(Direction == 1)
			{
				Direction = -1;
				uStart = m_Width - 1;
				uEnd = -1;

				if(vBottom == vTop)
				{
					delete[] ConvexHull;

					return;
				}

				v = vBottom;
			}
	}

	du = ConvexHull[0].u - pLastCHVertex->u;

	if(du == 0)
	{
		ConvexHull[0].du = pLastCHVertex->du;
		ConvexHull[0].dv = pLastCHVertex->dv;

		pLastCHVertex--;
	}
	else
	{
		ConvexHull[0].du = du;
		ConvexHull[0].dv = 0;
	}

	int nCHVertices = pLastCHVertex - ConvexHull + 1;

	if(nCHVertices < 3)
	{
		delete[] ConvexHull;

		return;
	}

	m_pDelaunay->m_nVertices = nCHVertices;
#pragma endregion

	//*** initialize LinkList

	RVLQLIST *LinkList = &(m_pDelaunay->m_LinkList);

	RVLQLIST_INIT(LinkList);

	//*** initialize queue

	RVLPSD_STRM_QUEUE_ENTRY *pNewEntry = m_QueueMem;

	m_Queue.Reset();

	RVLQLIST *ListArray = m_Queue.m_ListArray;

	//*** remember first triangle

	RVLPTRCHAIN_ELEMENT **ppFirstTriangle = 
		(p2DRegionSet->m_ObjectList.m_nElements == 0 ? &(p2DRegionSet->m_ObjectList.m_pFirst) : &(p2DRegionSet->m_ObjectList.m_pLast->pNext));
	
#pragma region Initial Triangulation
	memset(m_bCorner, 0, ImageSize * sizeof(BYTE));

	CRVL2DRegion2 **TriangleBuff = new CRVL2DRegion2 *[2 * ImageSize];

	CRVL2DRegion2 **ppTriangle = TriangleBuff;

	int nInitialLinks = 2 * (2 * nCHVertices - 3);

	RVLMESH_LINK *NewLinkArray;
	RVLQLIST_PTR_ENTRY *pLinkPtr;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, RVLMESH_LINK, nInitialLinks, NewLinkArray);

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, RVLQLIST_PTR_ENTRY, nInitialLinks, pLinkPtr);

	RVLMESH_LINK *pLinkArrayEnd = NewLinkArray + nInitialLinks;

	RVLMESH_LINK *pLink, *pLink2;

	for(pLink = NewLinkArray; pLink < pLinkArrayEnd; pLink++, pLinkPtr++)
	{
		RVLQLIST_ADD_ENTRY(LinkList, pLinkPtr);

		pLinkPtr->Ptr = pLink;
	}

	// create convex hull links

	pLink = NewLinkArray;

	RVLPSD_CHVERTEX *pCHVertex = ConvexHull;
	RVLPSD_CHVERTEX *pNextCHVertex;

	int iCHVertex, iNextCHVertex, iPrevCHVertex;
	//int iTmp;

	for(iCHVertex = 0; iCHVertex < nCHVertices; iCHVertex++, pCHVertex++)
	{
		iPrevCHVertex = (iCHVertex + nCHVertices - 1) % nCHVertices;

		iNextCHVertex = (iCHVertex + 1) % nCHVertices;

		//iTmp = 2 * (nCHVertices + iCHVertex);

		pNextCHVertex = ConvexHull + iNextCHVertex;

		pLink->Flags = RVLMESH_LINK_FLAG_VISITED;
		pLink->iPix0 = pCHVertex->u + pCHVertex->v * m_Width;
		m_bCorner[pLink->iPix0] = 1;
		du = pNextCHVertex->du;
		dv = pNextCHVertex->dv;
		len = DOUBLE2INT(sqrt((double)(du * du + dv * dv)));
		pLink->du = du;
		pLink->dv = dv;
		pLink->len = len;
		pLink->pOpposite = NewLinkArray + 2 * iCHVertex + 1;
		//pLink->pNext = NewLinkArray + 2 * iPrevCHVertex + 1;
		//pLink->pPrev = NewLinkArray + iTmp - 3;
		pLink->pNext = pLink->pPrev = NewLinkArray + 2 * iPrevCHVertex + 1;
		pLink->vp2DRegion = NULL;

		pLink++;

		pLink->Flags = 0x00;
		pLink->iPix0 = pNextCHVertex->u + pNextCHVertex->v * m_Width;
		pLink->du = -du;
		pLink->dv = -dv;
		pLink->len = len;
		pLink->pOpposite = NewLinkArray + 2 * iCHVertex;
		//pLink->pNext = NewLinkArray + iTmp - 1;
		//pLink->pPrev = NewLinkArray + 2 * iNextCHVertex;
		pLink->pNext = pLink->pPrev = NewLinkArray + 2 * iNextCHVertex;

		pLink++;
	}	

	// Search for a polygon with more than 3 vertices within the convex hull and split it.
	// Repeat this procedure until all polygons within the convex hull are triangles.

	RVLMESH_LINK *pNewLink = pLink;

	RVLMESH_LINK **LinkBuff = new RVLMESH_LINK *[nInitialLinks];

	RVLMESH_LINK **ppLink1 = LinkBuff;

	LinkBuff[0] = NewLinkArray + 2 * nCHVertices - 1;

	RVLMESH_LINK **ppLink2 = LinkBuff + 1;

	RVLMESH_LINK *pLink3, *pLink_, *pLink2_, *pLink3_;
	CRVL2DRegion2 *pTriangle;	

	while(ppLink2 > ppLink1)
	{
		pLink = *ppLink1;

		if(pLink->Flags & RVLMESH_LINK_FLAG_VISITED)
		{
			ppLink1++;

			continue;
		}

		pTriangle = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet));

		*(ppTriangle++) = pTriangle;

		pTriangle->m_vpQueueEntry = NULL;	

		pLink2_ = pLink->pNext;
		if((pLink2_->Flags & (RVLMESH_LINK_FLAG_VISITED | RVLMESH_LINK_FLAG_MARKED)) == 0)
		{
			*(ppLink2++) = pLink2_;
			pLink2_->Flags |= RVLMESH_LINK_FLAG_MARKED;
		}
		
		pLink2 = pLink2_->pOpposite;

		pLink3_ = pLink2->pNext;

		if((pLink3_->Flags & (RVLMESH_LINK_FLAG_VISITED | RVLMESH_LINK_FLAG_MARKED)) == 0)
		{
			*(ppLink2++) = pLink3_;
			pLink3_->Flags |= RVLMESH_LINK_FLAG_MARKED;
		}

		pLink3 = pLink3_->pOpposite;

		pLink_ = pLink3->pNext;

		pTriangle->m_PtArray = pLink2;

		pLink2->vp2DRegion = pTriangle;
		pLink3->vp2DRegion = pTriangle;

		pLink2->Flags |= RVLMESH_LINK_FLAG_VISITED;
		pLink3->Flags |= RVLMESH_LINK_FLAG_VISITED;			

		if(pLink_->pOpposite->iPix0 == pLink->iPix0)
		{
			pLink->vp2DRegion = pTriangle;

			if((pLink_->Flags & (RVLMESH_LINK_FLAG_VISITED | RVLMESH_LINK_FLAG_MARKED)) == 0)
			{
				*(ppLink2++) = pLink_;
				pLink_->Flags |= RVLMESH_LINK_FLAG_MARKED;
			}

			pLink->Flags |= RVLMESH_LINK_FLAG_VISITED;

			ppLink1++;
		}
		else
		{
			pNewLink->Flags = RVLMESH_LINK_FLAG_VISITED;
			pNewLink->iPix0 = pLink->iPix0;
			du = m_Point3DMap[pLink3->iPix0]->u - m_Point3DMap[pLink->iPix0]->u;
			dv = m_Point3DMap[pLink3->iPix0]->v - m_Point3DMap[pLink->iPix0]->v;
			len = DOUBLE2INT(sqrt((double)(du * du + dv * dv)));
			pNewLink->du = du;
			pNewLink->dv = dv;
			pNewLink->len = len;
			pNewLink->pOpposite = pNewLink + 1;
			pNewLink->pNext = pLink2_;
			pNewLink->pPrev = pLink;
			pNewLink->vp2DRegion = pTriangle;

			pLink->pNext = pNewLink;
			pLink2_->pPrev = pNewLink;

			pNewLink++;

			pNewLink->Flags = RVLMESH_LINK_FLAG_MARKED;
			*(ppLink2++) = pNewLink;
			pNewLink->iPix0 = pLink3->iPix0;
			pNewLink->du = -du;
			pNewLink->dv = -dv;
			pNewLink->len = len;
			pNewLink->pOpposite = pNewLink - 1;
			pNewLink->pNext = pLink_;
			pNewLink->pPrev = pLink3;

			pLink3->pNext = pNewLink;
			pLink_->pPrev = pNewLink;

			pNewLink++;
		}
	}

	delete[] LinkBuff;

	// reset all link flags

	RVLMESH_LINK *pNewLinkArrayEnd = NewLinkArray + nInitialLinks;

	for(pLink = NewLinkArray; pLink < pNewLinkArrayEnd; pLink++)
		pLink->Flags = 0x00;

	//int u0 = ConvexHull[0].u;
	//int v0 = ConvexHull[0].v;
	//int iPix0 = u0 + v0 * m_Width;

	//pCHVertex = ConvexHull + 2;

	//for(iCHVertex = 2; iCHVertex <= nCHVertices - 2; iCHVertex++)
	//{
	//	pTriangle = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet));

	//	*(ppTriangle++) = pTriangle;

	//	pTriangle->m_PtArray = NewLinkArray + 2 * (nCHVertices + iCHVertex) - 4;
	//	pTriangle->m_vpQueueEntry = NULL;	
	//}

	//pTriangle = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet));

	//*(ppTriangle++) = pTriangle;

	//pTriangle->m_PtArray = NewLinkArray + 2 * nCHVertices - 1;
	//pTriangle->m_vpQueueEntry = NULL;		

	//for(iCHVertex = 2; iCHVertex <= nCHVertices - 2; iCHVertex++, pCHVertex++)
	//{
	//	iTmp = 2 * (nCHVertices + iCHVertex);

	//	pLink->Flags = 0x00;
	//	pLink->iPix0 = iPix0;
	//	du = pCHVertex->u - u0;
	//	dv = pCHVertex->v - v0;
	//	len = DOUBLE2INT(sqrt((double)(du * du + dv * dv)));
	//	pLink->du = du;
	//	pLink->dv = dv;
	//	pLink->len = len;
	//	pLink->pOpposite = NewLinkArray + iTmp - 3;
	//	pLink->pNext = NewLinkArray + iTmp - 6;
	//	pLink->pPrev = NewLinkArray + iTmp - 2;
	//	pLink->vp2DRegion = TriangleBuff[iCHVertex - 2];

	//	pLink++;

	//	pLink->Flags = 0x00;
	//	pLink->iPix0 = pCHVertex->u + pCHVertex->v * m_Width;
	//	pLink->du = -du;
	//	pLink->dv = -dv;
	//	pLink->len = len;
	//	pLink->pOpposite = NewLinkArray + iTmp - 4;
	//	pLink->pNext = NewLinkArray + 2 * iCHVertex;
	//	pLink->pPrev = NewLinkArray + 2 * iCHVertex - 1;
	//	pLink->vp2DRegion = TriangleBuff[iCHVertex - 1];

	//	pLink++;
	//}

	//iTmp = 2 * nCHVertices;

	//if(nCHVertices == 3)
	//{
	//	NewLinkArray[0].pPrev = NewLinkArray + 5;
	//	NewLinkArray[5].pPrev = NewLinkArray;
	//}
	//else
	//{
	//	NewLinkArray[0].pPrev = NewLinkArray + iTmp;
	//	NewLinkArray[iTmp].pNext = NewLinkArray;
	//	NewLinkArray[iTmp - 1].pNext = NewLinkArray + 4 * (nCHVertices - 2);
	//	NewLinkArray[4 * (nCHVertices - 2)].pPrev = NewLinkArray + iTmp - 1;
	//}
	//NewLinkArray[1].pNext = NewLinkArray + 2;
	//NewLinkArray[1].vp2DRegion = TriangleBuff[0];
	//NewLinkArray[2].pPrev = NewLinkArray + 1;
	//NewLinkArray[iTmp - 1].vp2DRegion = TriangleBuff[nCHVertices - 3];
	//NewLinkArray[iTmp - 2].pPrev = NewLinkArray + iTmp - 3;
	//NewLinkArray[iTmp - 3].pNext = NewLinkArray + iTmp - 2;
	//NewLinkArray[iTmp - 3].vp2DRegion = TriangleBuff[nCHVertices - 3];

	delete[] ConvexHull;

#ifdef RVLPSD_SEGMENT_STRM_DEBUG
	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("STRM");

	pFig->EmptyBitmap(cvSize(m_Width, m_Height), cvScalar(0, 0, 0));

	pFig->m_VectorArray.RemoveAll();

	pFig->m_pMem->Clear();

	RVLDisplay2DRegions(pFig, &(p2DRegionSet->m_ObjectList), m_Width, RVLColor(255, 255, 0));

	m_DebugData.pGUI->DisplayVectors(pFig, 0, 0, 1.0);

	m_DebugData.pGUI->ShowFigure(pFig);

	cvWaitKey();
#endif

#ifdef RVLPSD_SEGMENT_STRM_PC_DEBUG
	int iFOVExtensionDebug = 0;

	int iFOVExtension = (m_Point3DMap - m_Point3DMapMem) / ImageSize - m_nFOVExtensions;

	bDebug = (iFOVExtension == iFOVExtensionDebug);

	if (bDebug)
	{
		bDebugMap = new bool[ImageSize];

		memset(bDebugMap, 0, ImageSize * sizeof(bool));

		double P0[3];

		P0[0] = x0Debug; P0[1] = y0Debug; P0[2] = z0Debug;

		double dP[3];
		RVL3DPOINT2 *p3DPt;

		for (int iPix = 0; iPix < ImageSize; iPix++)
		{
			p3DPt = m_Point3DMap[iPix];

			if (p3DPt == NULL)
				continue;

			RVLDIF3VECTORS(p3DPt->XYZ, P0, dP);

			if (RVLABS(dP[0]) > rDebug)
				continue;

			if (RVLABS(dP[1]) > rDebug)
				continue;

			if (RVLABS(dP[2]) > rDebug)
				continue;

			bDebugMap[iPix] = true;
		}

		debugTriangles.clear();
	}
#endif

	UpdateSTRMQueue(TriangleBuff, ppTriangle, err, bErrCorrection, &pNewEntry);

#pragma endregion

#pragma region Succesive Triangulation Refinement	

	int LinkBuffSize = 3 * ImageSize;

	LinkBuff = new RVLMESH_LINK *[LinkBuffSize];

	//int DebugCounter = 0;

	int maxDist = m_Width + m_Height;

	RVLPSD_STRM_QUEUE_ENTRY *pEntry;
	RVLMESH_LINK *pLink0, *pLink4, *pLinkOpposite;
	int uNew, vNew;
	int iPixNew;
	BOOL bNew[4];
	int nTriangles;
	int nTrianglesMinus1;
	int iTriangle, iNextTriangle, iPrevTriangle;
	BOOL bOnEdge;
	RVLMESH_LINK *LinkArray[8];
#ifdef RVLPSD_SEGMENT_STRM_OLD
	int duOpposite, dvOpposite;
	int h1, h2, len2;
	int duArray[3], dvArray[3];
	int iTmp;
	int dist, minDist;
	int *pdu, *pdv;
#else
	int iLinkPtr1, iLinkPtr2;
#endif

	do
	{
		pEntry = (RVLPSD_STRM_QUEUE_ENTRY *)(ListArray[err].pFirst);

		while(pEntry)
		{
			//if(pEntry - m_QueueMem == 171)
			//	int tmp1 = 0;

			//if(fabs(m_Point3DMap[127 + 227 * 320]->XYZ[2]) < 100.0)
			//	int debug = 0;

			pTriangle = pEntry->pRegion;

			if(pTriangle)
			{
				pLink0 = pLink = (RVLMESH_LINK *)(pTriangle->m_PtArray);

				iPixNew = pEntry->iPix;

				m_pDelaunay->m_nVertices++;

				m_bCorner[iPixNew] = 1;

				uNew = iPixNew % m_Width;
				vNew = iPixNew / m_Width;

				//if(uNew == 309 && vNew == 194)
				//	int debug = 0;

				pLink0 = pLink;

				do
				{
					u = pLink->iPix0 % m_Width;
					v = pLink->iPix0 / m_Width;

					du = uNew - u;
					dv = vNew - v;

					if(bOnEdge = (-pLink->dv * du + pLink->du * dv == 0))
						break;

					pLink = pLink->pNext->pOpposite;
				}
				while(pLink != pLink0);

				if(bOnEdge)
				{
					if(pLink->pPrev->vp2DRegion)
					{
						// split two triangles into four triangles

						LinkArray[0] = pLink->pPrev;
						LinkArray[1] = pLink->pNext->pOpposite;
						LinkArray[2] = LinkArray[1]->pNext->pOpposite;
						LinkArray[3] = LinkArray[2]->pNext->pNext->pOpposite;

						bNew[0] = bNew[1] = FALSE;
						bNew[2] = bNew[3] = TRUE;

						nTriangles = 4;
					}
					else
					{
						// split triangle into two triangles

						LinkArray[0] = pLink->pNext->pOpposite;
						LinkArray[1] = LinkArray[0]->pNext->pOpposite;

						bNew[0] = FALSE;
						bNew[1] = TRUE;

						nTriangles = 2;
					}
				}	
				else
				{
					// split triangle into three triangles

					LinkArray[0] = pLink;
					LinkArray[1] = pLink->pNext->pOpposite;
					LinkArray[2] = LinkArray[1]->pNext->pOpposite;

					bNew[0] = FALSE;
					bNew[1] = bNew[2] = TRUE;

					nTriangles = 3;
				}

				RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, RVLMESH_LINK, 2 * nTriangles, NewLinkArray);

				RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, RVLQLIST_PTR_ENTRY, 2 * nTriangles, pLinkPtr);

				nTrianglesMinus1 = nTriangles - 1;

				for(iTriangle = 0; iTriangle < nTriangles; iTriangle++)
				{
					pLink = LinkArray[iTriangle];

					pLinkOpposite = pLink->pOpposite;

					if(bNew[iTriangle])
					{
						pTriangle = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(p2DRegionSet));

						pLink->vp2DRegion = pTriangle;

						//if(p2DRegionSet->m_ObjectList.m_nElements == 89)
						//	int tmp1 = 0;
					}
					else
						pTriangle = (CRVL2DRegion2 *)(pLink->vp2DRegion);

					pTriangle->m_PtArray = pLink;

					u = pLink->iPix0 % m_Width;
					v = pLink->iPix0 / m_Width;

					du = u - uNew;
					dv = v - vNew;

					iNextTriangle = (iTriangle + 1) % nTriangles;

					iPrevTriangle = (iTriangle + nTrianglesMinus1) % nTriangles;

					pLink2 = NewLinkArray + iTriangle;

					pLink2->Flags = 0x00;
					pLink2->iPix0 = iPixNew;
					pLink2->pOpposite = NewLinkArray + nTriangles + iNextTriangle;
					pLink2->pNext = NewLinkArray + iPrevTriangle;
					pLink2->pPrev = NewLinkArray + iNextTriangle;
					pLink2->du = du;
					pLink2->dv = dv;
					pLink2->len = DOUBLE2INT(sqrt((double)(du * du + dv * dv)));
					pLink2->vp2DRegion = pTriangle;

					RVLQLIST_ADD_ENTRY(LinkList, pLinkPtr);

					pLinkPtr->Ptr = pLink2;

					pLinkPtr++;

					pLink3 = NewLinkArray + nTriangles + iTriangle;

					pLink3->Flags = 0x00;
					pLink3->iPix0 = pLinkOpposite->iPix0;
					pLink3->pOpposite = pLink2->pNext;
					pLink3->pNext = pLinkOpposite;
					pLink3->pPrev = LinkArray[iPrevTriangle];
					pLink3->vp2DRegion = pTriangle;

					RVLQLIST_ADD_ENTRY(LinkList, pLinkPtr);

					pLinkPtr->Ptr = pLink3;

					pLinkPtr++;

					pLink4 = NewLinkArray + nTriangles + iNextTriangle;

					pLink4->du = -du;
					pLink4->dv = -dv;
					pLink4->len = pLink2->len;
				}

				pLink0 = LinkArray[0]->pOpposite->pPrev;
				pLink2 = LinkArray[1]->pNext;

				for(iTriangle = 0; iTriangle < nTriangles; iTriangle++)
				{
					pLink = LinkArray[iTriangle];

					pLink->pNext = NewLinkArray + nTriangles + (iTriangle + 1) % nTriangles;

					pLink->pOpposite->pPrev = NewLinkArray + nTriangles + iTriangle;
				}

				if(nTriangles == 2)
				{
					LinkArray[0]->pOpposite->pPrev = pLink0;

					u = pLink0->iPix0 % m_Width;
					v = pLink0->iPix0 / m_Width;

					du = u - uNew;
					dv = v - vNew;

					pLink0->du = -du;
					pLink0->dv = -dv;
					pLink0->len = DOUBLE2INT(sqrt((double)(du * du + dv * dv)));
		
					NewLinkArray[0].pNext = pLink2;
					NewLinkArray[1].pPrev = pLink2;
					NewLinkArray[2].iPix0 = LinkArray[1]->iPix0;
					NewLinkArray[2].pNext = pLink2->pNext;
					NewLinkArray[2].vp2DRegion = NULL;

					pLink2->pPrev = NewLinkArray;
					pLink2->pNext = NewLinkArray + 1;
					pLink2->iPix0 = iPixNew;
					pLink2->du = du;
					pLink2->dv = dv;
					pLink2->len = pLink0->len;
				}

				// for debugging purpose only !!!

				//for(iTriangle = 0; iTriangle < nTriangles; iTriangle++)
				//{
				//	pLink = LinkArray[iTriangle];

				//	if(!m_pDelaunay->Test(pLink))
				//		int debug = 0;
				//}

				//if(!m_pDelaunay->Test(NewLinkArray))
				//	int debug = 0;

				/////

				//*** Update Delaunay triangulation

				// for each link from LinkArray which separates two triangles,
				// insert 

				ppTriangle = TriangleBuff;

				ppLink1 = LinkBuff;

				for(iTriangle = 0; iTriangle < nTriangles; iTriangle++)
				{
					pLink = LinkArray[iTriangle];

					pTriangle = (CRVL2DRegion2 *)(pLink->vp2DRegion);

					*(ppTriangle++) = pTriangle;

					pTriangle->m_Flags |= RVLOBJ2_FLAG_MARKED;

					pLinkOpposite = pLink->pOpposite;

					pTriangle2 = (CRVL2DRegion2 *)(pLinkOpposite->vp2DRegion);

					if(pTriangle2)
					{						
						*(ppLink1++) = (pLink->iPix0 < pLinkOpposite->iPix0 ? pLink : pLinkOpposite);

						pLink->Flags |= RVLMESH_LINK_FLAG_VISITED;
					}
				}

				iLinkPtr1 = ppLink1 - LinkBuff;

				iLinkPtr2 = 0;

				while(iLinkPtr2 != iLinkPtr1)
				{
					pLink = LinkBuff[iLinkPtr2];

					pLink->Flags &= ~RVLMESH_LINK_FLAG_VISITED;

					if(RVLFlipDiagonal(pLink, LinkArray))
					{
						pTriangle = (CRVL2DRegion2 *)(pLink->vp2DRegion);

						pTriangle2 = (CRVL2DRegion2 *)(pLink->pOpposite->vp2DRegion);

						if((pTriangle->m_Flags & RVLOBJ2_FLAG_MARKED) == 0)
						{
							pTriangle->m_Flags |= RVLOBJ2_FLAG_MARKED;

							*(ppTriangle++) = pTriangle;
						}

						if((pTriangle2->m_Flags & RVLOBJ2_FLAG_MARKED) == 0)
						{
							pTriangle2->m_Flags |= RVLOBJ2_FLAG_MARKED;

							*(ppTriangle++) = pTriangle2;
						}						

						for(iLink = 0; iLink < 4; iLink++)
						{
							pLink = LinkArray[iLink];

							pLinkOpposite = pLink->pOpposite;

							if(pLinkOpposite->vp2DRegion)
							{
								if(pLinkOpposite->iPix0 < pLink->iPix0)
									pLink = pLinkOpposite;

								if((pLink->Flags & RVLMESH_LINK_FLAG_VISITED) == 0)
								{
									pLink->Flags |= RVLMESH_LINK_FLAG_VISITED;

									LinkBuff[iLinkPtr1] = pLink;

									iLinkPtr1 = (iLinkPtr1 + 1) % LinkBuffSize;
								}
							}

							//if(!m_pDelaunay->Test(pLink))
							//	int debug = 0;
						}

					}	// if(RVLFlipDiagonal(pLink, LinkArray))

					iLinkPtr2 = (iLinkPtr2 + 1) % LinkBuffSize;
				}	// while(iLinkPtr2 != iLinkPtr1)

				// only for debugging purpose!!!

				//RVLMESH_LINK *pLinkDebug = (RVLMESH_LINK *)(pTriangle->m_PtArray);

				//for(iTriangle = 0; iTriangle < nTriangles; iTriangle++)
				//{
				//	pLink = LinkArray[iTriangle];

				//	if(pLink->iPix0 == 7172 && 
				//		pLink->pNext->pOpposite->iPix0 == 6574 &&
				//		pLink->pNext->pOpposite->pNext->pOpposite->iPix0 == 319)
				//		int tmp1 = 0;
				//}
				
				//m_pDelaunay->Test2();

				/////

#ifdef RVLPSD_SEGMENT_STRM_DEBUG
				//FILE *fp;

				//fopen_s(&fp, "C:\\RVL\\ExpRez\\delaunay.dat", "w");

				//RVLSaveSegmentation(fp, p2DRegionSet, m_Width);

				//fclose(fp);

				CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("STRM");

				pFig->EmptyBitmap(cvSize(m_Width, m_Height), cvScalar(0, 0, 0));

				pFig->m_VectorArray.RemoveAll();

				pFig->m_pMem->Clear();

				RVLDisplay2DRegions(pFig, &(p2DRegionSet->m_ObjectList), m_Width, RVLColor(255, 255, 0));

				m_DebugData.pGUI->DisplayVectors(pFig, 0, 0, 1.0);

				m_DebugData.pGUI->ShowFigure(pFig);

				cvWaitKey();
#endif

//#ifdef RVLPSD_SEGMENT_STRM_LOG_FILE
//				if(p2DRegionSet->m_ObjectList.m_nElements / 50 > DebugCounter)
//				{
//					FILE *fp;
//
//					fopen_s(&fp, "C:\\RVL\\ExpRez\\delaunay.dat", "w");
//
//					RVLSaveSegmentation(fp, p2DRegionSet, m_Width);
//
//					fclose(fp);
//
//					DebugCounter++;
//				}
//#endif

				// test new triangles and update queue

				UpdateSTRMQueue(TriangleBuff, ppTriangle, err, bErrCorrection, &pNewEntry);

				// for debugging purpose only !!!

				//p2DRegionSet->m_ObjectList.Start();

				//while(p2DRegionSet->m_ObjectList.m_pNext)
				//{
				//	pTriangle = (CRVL2DRegion2 *)(p2DRegionSet->m_ObjectList.GetNext());

				//	RVLMESH_LINK *pLinkDebug = (RVLMESH_LINK *)(pTriangle->m_PtArray);

				//	if(pLinkDebug->iPix0 == 7172 && 
				//		pLinkDebug->pNext->pOpposite->iPix0 == 6574 &&
				//		pLinkDebug->pNext->pOpposite->pNext->pOpposite->iPix0 == 319)
				//		int tmp1 = 0;
				//}

				/////

				if(bErrCorrection)
					break;
			}	// if(pTriangle)

			pEntry = (RVLPSD_STRM_QUEUE_ENTRY *)(pEntry->pNext);

			ListArray[err].pFirst = pEntry;

			if(pEntry == NULL)
				ListArray[err].ppNext = &(ListArray[err].pFirst);
		}	// for every entry in the current bin

		if(bErrCorrection)
			bErrCorrection = FALSE;
		else
			err--;
	}
	while(err >= 0);
#pragma endregion 

	double ExecutionTime = m_pTimer->GetTime() - StartTime;

#ifdef RVLPSD_SEGMENT_STRM_PC_DEBUG
	if (bDebug)
	{
		delete[] bDebugMap;
	}
#endif

#ifdef NEVER
	////KARLO
	////Determine dominant/reliable edges by determining the uncertainty of the intersection of 2 planes

	//CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	//CRVL2DRegion2 *p2DRegion, *p2DRegionNeighbour;

	//RVLMESH_LINK *pLinkNeighbour;

	//RVL3DPOINT2 *p3DPt1, *p3DPt2, *p3DPt3, *p3DPt4;

	//double pt3D_1[3], pt3D_2[3], pt3D_3[3], pt3D_4[3];

	//double pt3D_c[3];
	//
	//int pt2D_1[2], pt2D_2[2];

	//double xL[3], yL[3], zL[3];
	//
	//double P01[3][3],P02[3][3];
	//double PL1[3][3],PL2[3][3];

	//double N1Temp[3][3], N2Temp[3][3];
	//double N1[3], N2[3];
	//double n1xn2[3], c, c2;

	//double J[6];

	//CvMat *dMatPL = cvCreateMatHeader(3, 1, CV_64FC1);

	//double  n1xz[3], n2xz[3];
	//CvMat *dMatn1xz = cvCreateMatHeader(3, 1, CV_64FC1);
	//CvMat *dMatn2xz = cvCreateMatHeader(3, 1, CV_64FC1);
	//dMatn1xz->data.db = n1xz;
	//dMatn2xz->data.db = n2xz;

	//double drho1dP[3], drho2dP[3];
	//CvMat *dMatdrho1dP = cvCreateMatHeader(1, 3, CV_64FC1);
	//CvMat *dMatdrho2dP = cvCreateMatHeader(1, 3, CV_64FC1);
	//dMatdrho1dP->data.db = drho1dP;
	//dMatdrho2dP->data.db = drho2dP;
	//
	//double rot0L[9];
	//CvMat *dMatRot = cvCreateMatHeader(3, 3, CV_64FC1);
	//dMatRot->data.db = rot0L;

	//double dLpdL1[9], dLpdL2[9];
	//CvMat *dMatdLpdL1 = cvCreateMatHeader(3, 3, CV_64FC1);
	//CvMat *dMatdLpdL2 = cvCreateMatHeader(3, 3, CV_64FC1);
	//dMatdLpdL1->data.db = dLpdL1;
	//dMatdLpdL2->data.db = dLpdL2;

	//double tempArr[3];
	//CvMat *dMatTemp = cvCreateMatHeader(3, 1, CV_64FC1);
	//dMatTemp->data.db = tempArr;
	//
	//double diffD[5];
	//double A1[2], B[2], A2[2];
	//double bL;
	//double h1, h2;

	//double *pPoz, *pNeg;

	//double d;

	//int i,j;

	//double currUnc, maxUnc;

	//double z[3] = {0, 0, 1};

	//maxUnc = 100;

	//double fillPerc = 0.5;

	//int minHeight = 3;
	//int thresh1 = 10;
	//int thresh2 = 10;


	//p2DRegionList->Start();

	//while(p2DRegionList->m_pNext)
	//{
	//	p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

	//	pLink = pLink0 = (RVLMESH_LINK *)(p2DRegion->m_PtArray);

	//	do
	//	{
	//		
	//		pLinkNeighbour = pLink->pNext;

	//		p2DRegionNeighbour = (CRVL2DRegion2 *)(pLinkNeighbour->vp2DRegion);


	//		if((p2DRegion != NULL) && 
	//		   (p2DRegionNeighbour != NULL) && 
	//		   (p2DRegion->m_nPts > 0) &&
	//		   (p2DRegionNeighbour->m_nPts > 0) &&
	//		   (p2DRegion->m_n3DPts/(p2DRegion->m_nPts * 1.0) > fillPerc) &&
	//		   (p2DRegionNeighbour->m_n3DPts/(p2DRegionNeighbour->m_nPts * 1.0) > fillPerc))
	//		   
	//		{
	//			
	//			//Define points
	//			pLink2 = pLinkNeighbour->pOpposite;
	//			pLink3 = pLink->pOpposite;
	//			pLink4 = pLinkNeighbour->pNext->pOpposite;

	//			p3DPt1 = m_Point3DMap[pLink->iPix0];
	//			p3DPt2 = m_Point3DMap[pLink2->iPix0];

	//			p3DPt3 = m_Point3DMap[pLink3->iPix0];
	//			p3DPt4 = m_Point3DMap[pLink4->iPix0];


	//			pt3D_1[0] = p3DPt1->u;
	//			pt3D_1[1] = p3DPt1->v;
	//			pt3D_1[2] = p3DPt1->d;

	//			pt3D_2[0] = p3DPt2->u;
	//			pt3D_2[1] = p3DPt2->v;
	//			pt3D_2[2] = p3DPt2->d;

	//			pt3D_3[0] = p3DPt3->u;
	//			pt3D_3[1] = p3DPt3->v;
	//			pt3D_3[2] = p3DPt3->d;

	//			pt3D_4[0] = p3DPt4->u;
	//			pt3D_4[1] = p3DPt4->v;
	//			pt3D_4[2] = p3DPt4->d;


	//			pt3D_c[0] = (pt3D_1[0] + pt3D_2[0])/2.0;
	//			pt3D_c[1] = (pt3D_1[1] + pt3D_2[1])/2.0;
	//			pt3D_c[2] = (pt3D_1[2] + pt3D_2[2])/2.0;

	//			for (i=0;i<3;i++)
	//			{
	//				P01[0][i] = pt3D_1[i];
	//				P01[1][i] = pt3D_2[i];
	//				P01[2][i] = pt3D_3[i];

	//				P02[0][i] = pt3D_1[i];
	//				P02[1][i] = pt3D_2[i];
	//				P02[2][i] = pt3D_4[i];

	//				
	//			}

	//			
	//			



	//			//Define zL
	//			zL[0] = p3DPt1->u - p3DPt2->u;
	//			zL[1] = p3DPt1->v - p3DPt2->v;
	//			zL[2] = p3DPt1->d - p3DPt2->d;

	//			d = sqrt((double)(zL[0]*zL[0] + zL[1]*zL[1] + zL[2]*zL[2])); 

	//			for (i=0;i<3;i++)
	//				zL[i] = zL[i]/d;
	//			

	//			//Define xL
	//			//pt2D_1[0] = pLink->iPix0 % m_Width;
	//			//pt2D_1[1] = pLink->iPix0 / m_Width;

	//			//pt2D_2[0] = pLink2->iPix0 % m_Width;
	//			//pt2D_2[1] = pLink2->iPix0 / m_Width;

	//			for (i=0;i<2;i++)
	//			{
	//				pt2D_1[i] = (int)pt3D_1[i];
	//				pt2D_2[i] = (int)pt3D_2[i];
	//			}
	//			
	//			xL[0] = -(pt2D_1[1] - pt2D_2[1]);
	//			xL[1] = pt2D_1[0] - pt2D_2[0];
	//			xL[2] = 0;

	//			d = sqrt((double)(xL[0]*xL[0] + xL[1]*xL[1])); 

	//			for (i=0;i<2;i++)
	//				xL[i] = xL[i]/d;


	//			//Define yL
	//			CrossProduct(zL,xL,yL);


	//			//Define rotation matrix R_0L
	//			rot0L[0 * 3 + 0] = xL[0];
	//			rot0L[1 * 3 + 0] = xL[1];
	//			rot0L[2 * 3 + 0] = xL[2];

	//			rot0L[0 * 3 + 1] = yL[0];
	//			rot0L[1 * 3 + 1] = yL[1];
	//			rot0L[2 * 3 + 1] = yL[2];

	//			rot0L[0 * 3 + 2] = zL[0];
	//			rot0L[1 * 3 + 2] = zL[1];
	//			rot0L[2 * 3 + 2] = zL[2];

	//			
	//			//Create PL1 and PL2 matrices
	//			for (i=0;i<3;i++)
	//			{
	//				//For Plane 1
	//				dMatPL->data.db = PL1[i];
	//				RVLDif3D(P01[i],pt3D_c,tempArr);
	//				cvGEMM(dMatRot,dMatTemp,1,NULL,1,dMatPL,CV_GEMM_A_T);

	//				//For Plane 2
	//				dMatPL->data.db = PL2[i];
	//				RVLDif3D(P02[i],pt3D_c,tempArr);
	//				cvGEMM(dMatRot,dMatTemp,1,NULL,1,dMatPL,CV_GEMM_A_T);
	//			
	//			}

	//			//define constants
	//			CrossProduct(PL1[0],PL1[1],N1Temp[0]);
	//			CrossProduct(PL1[1],PL1[2],N1Temp[1]);
	//			CrossProduct(PL1[2],PL1[0],N1Temp[2]);

	//			CrossProduct(PL2[0],PL2[1],N2Temp[0]);
	//			CrossProduct(PL2[1],PL2[2],N2Temp[1]);
	//			CrossProduct(PL2[2],PL2[0],N2Temp[2]);

	//			RVLSum3D(N1Temp[0],N1Temp[1],N1);
	//			RVLSum3D(N1Temp[2],N1,N1);

	//			RVLSum3D(N2Temp[0],N2Temp[1],N2);
	//			RVLSum3D(N2Temp[2],N2,N2);


	//			CrossProduct(N1,N2,n1xn2);
	//			CrossProduct(N1,z,n1xz);
	//			CrossProduct(N2,z,n2xz);

	//			c = RVLDotProduct(z,n1xn2);
	//			c2 = 1/(c*c);

	//			

	//			

	//			if(c != 0.0)
	//			{

	//				for (i=0;i<3;i++)
	//				{
	//					//define drho1dP/drho2dP
	//					pPoz = PL1[(i+1)%3];	//i+
	//					pNeg = PL1[(i+2)%3];	//i-
	//					CrossProduct(pPoz, pNeg, drho1dP);
	//					
	//					pPoz = PL2[(i+1)%3];	//i+
	//					pNeg = PL2[(i+2)%3];	//i-
	//					CrossProduct(pPoz, pNeg, drho2dP);

	//					cvGEMM(dMatn2xz,dMatdrho1dP,1,NULL,1,dMatdLpdL1);
	//					cvGEMM(dMatn1xz,dMatdrho2dP,-1,NULL,1,dMatdLpdL2);

	//					// reset values
	//					J[0 * 3 + i] = 0;
	//					J[1 * 3 + i] = 0;

	//					//J = first row of dLpdL1/dLpdL2 * last row of rot0L
	//					//modify this for faster algorithm - no need to calculate 3x!!
	//					for(j=0;j<3;j++)
	//					{
	//						J[0 * 3 + i] += dLpdL1[0 * 3 + j] * rot0L[6 + j];
	//						J[1 * 3 + i] += dLpdL2[0 * 3 + j] * rot0L[6 + j];
	//					}
	//					
	//				}	
	//				
	//				currUnc = 0.0;
	//				for (i=0;i<6;i++)
	//					currUnc += J[i]*J[i];
	//				//OR
	//				//currUnc = (J[0]+J[1])*(J[0]+J[1]) + (J[3]+J[4])*(J[3]+J[4]);
	//				
	//				currUnc = currUnc * m_uvdTol * m_uvdTol * c2;

	//				if (currUnc < maxUnc)
	//				{
	//					pLinkNeighbour->Flags |= RVLMESH_LINK_FLAG_BOUNDARY;
	//					pLinkNeighbour->pOpposite->Flags |= RVLMESH_LINK_FLAG_BOUNDARY;

	//					//detect occluding edges and mark them as such
	//					for (i=0;i<2;i++)
	//					{
	//						A1[i] = pt3D_3[i] - pt3D_1[i];
	//						A2[i] = pt3D_4[i] - pt3D_1[i];
	//						B[i] = pt3D_2[i] - pt3D_1[i];
	//					}
	//					
	//					if((pt3D_2[1]==95.0 || pt3D_1[1]==95.0) && (pt3D_2[0]==212.0 ||  pt3D_1[0]==212.0))
	//						int ttt = 0;

	//					diffD[0] = (pt3D_2[2] - pt3D_1[2]);
	//					diffD[1] = (pt3D_3[2] - pt3D_1[2]);
	//					diffD[2] = (pt3D_3[2] - pt3D_2[2]);
	//					diffD[3] = (pt3D_4[2] - pt3D_1[2]);
	//					diffD[4] = (pt3D_4[2] - pt3D_2[2]);
	//			
	//					bL = sqrt((B[0]*B[0] + B[1]*B[1]));
	//					h1 = abs(A1[0]*B[1] - A1[1]*B[0])/(bL);
	//					h2 = abs(A2[0]*B[1] - A2[1]*B[0])/(bL);

	//					//0.Find occluding edge
	//					//1.make sure height of triangle is max 2 pixels and that diff in depth of the points on the edge is below thresh1
	//					//2.make sure that depths of corresponding are greater than thresh2 and have the same sign
	//					if(h1<=minHeight)
	//					{
	//						if (abs(diffD[0])<=thresh1)
	//						{
	//							if(((abs(diffD[1])>thresh2) && (abs(diffD[2])>thresh2) && (diffD[1]*diffD[2]>0)))
	//							{
	//								if(diffD[1]>0)
	//								//edge is near
	//								{
	//									pLinkNeighbour->Flags |= RVLMESH_LINK_FLAG_FOREGROUND;
	//									pLinkNeighbour->pOpposite->Flags |= RVLMESH_LINK_FLAG_FOREGROUND;
	//								}
	//								else
	//								//edge is far
	//								{
	//									pLinkNeighbour->Flags |= RVLMESH_LINK_FLAG_BACKGROUND;
	//									pLinkNeighbour->pOpposite->Flags |= RVLMESH_LINK_FLAG_BACKGROUND;
	//								}
	//								
	//							}
	//							
	//						}
	//						//else if (abs(diffD[0])>thresh2)
	//						//{
	//						//	//edge is not important
	//						//	pLinkNeighbour->Flags &= ~RVLMESH_LINK_FLAG_BOUNDARY;
	//						//	pLinkNeighbour->pOpposite->Flags &= ~RVLMESH_LINK_FLAG_BOUNDARY;
	//						//}
	//						
	//					}
	//					if(h2<=minHeight)
	//					{
	//						if (abs(diffD[0])<=thresh1)
	//						{

	//							if(((abs(diffD[3])>thresh2) && (abs(diffD[4])>thresh2) && (diffD[3]*diffD[4]>0)))
	//							{
	//								if(diffD[3]>0)
	//								//edge is near
	//								{
	//									pLinkNeighbour->Flags |= RVLMESH_LINK_FLAG_FOREGROUND;
	//									pLinkNeighbour->pOpposite->Flags |= RVLMESH_LINK_FLAG_FOREGROUND;
	//								}
	//								else
	//								//edge is far
	//								{
	//									pLinkNeighbour->Flags |= RVLMESH_LINK_FLAG_BACKGROUND;
	//									pLinkNeighbour->pOpposite->Flags |= RVLMESH_LINK_FLAG_BACKGROUND;
	//								}

	//							}
	//						}
	//						//else if (abs(diffD[0])>thresh2)
	//						//{
	//						//	//edge is not important
	//						//	pLinkNeighbour->Flags &= ~RVLMESH_LINK_FLAG_BOUNDARY;
	//						//	pLinkNeighbour->pOpposite->Flags &= ~RVLMESH_LINK_FLAG_BOUNDARY;
	//						//}
	//					
	//					}
	//					
	//				}
	//			}
	//		}


	//		pLink = pLinkNeighbour->pOpposite;
	//	}
	//	while(pLink != pLink0);
	//}


#endif


#ifdef RVLPSD_SEGMENT_STRM_LOG_FILE
	FILE *fp;

	fopen_s(&fp, "C:\\RVL\\ExpRez\\delaunay.dat", "w");

	RVLSaveSegmentation(fp, p2DRegionSet, m_Width);

	fclose(fp);

	FILE *fpPts, *fpIdx;

	fopen_s(&fpPts, "C:\\RVL\\ExpRez\\meshPts.dat", "w");

	fopen_s(&fpIdx, "C:\\RVL\\ExpRez\\meshIdx.dat", "w");

	SaveMesh(fpPts, fpIdx, p2DRegionSet);

	fclose(fpPts);

	fclose(fpIdx);
#endif

	
	delete[] LinkBuff;
	delete[] TriangleBuff;	

#pragma region Add properties to the triangles	
	//Reject discontinuity triangles as well as empty triangles
	//and relate 3D points to triangles and create vertex list 

	BYTE **DelaunayMap = m_pDelaunay->m_DelaunayMap;

	memset(DelaunayMap, 0, ImageSize * sizeof(BYTE *));

	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	CRVL2DRegion2 *p2DRegion;

	RVL3DPOINT2 **Point3DPtrArray = (RVL3DPOINT2 **)(m_pMem->Alloc(2 * m_Width*m_Height * sizeof(RVL3DPOINT2)));  //contains all the 3D points for all the regions
	RVL3DPOINT2 **ppPtrArrayStart;

	ppPtrArrayStart = Point3DPtrArray;

	RVLQLIST *pVertexList = &(m_pDelaunay->m_VertexList);

	RVLQLIST_INIT(pVertexList);

	RVLQLIST_PTR_ENTRY *pVertex = m_pDelaunay->m_VertexListData = 
		(RVLQLIST_PTR_ENTRY *)(m_pMem->Alloc(m_pDelaunay->m_nVertices * sizeof(RVLQLIST_PTR_ENTRY)));

	int *N;

	iTriangle = 0;

	p2DRegionList->m_pNext = *ppFirstTriangle;

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		//if((int)p2DRegion == 0x023a660c)
		//	int debug = 0;

		p2DRegion->m_pPoint3DArray = ppPtrArrayStart;

		p2DRegion->m_pPoint3DMap = m_Point3DMap;

		GetPointsWithDisparity(p2DRegion,ppPtrArrayStart);

		ppPtrArrayStart += p2DRegion->m_n3DPts;

		ppPtrArrayStart++;

		pLink0 = pLink = (RVLMESH_LINK *)(p2DRegion->m_PtArray);

		do
		{
			if((m_bCorner[pLink->iPix0] & 0x02) == 0)
			{	
				RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

				pVertex->Ptr = pLink;

				DelaunayMap[pLink->iPix0] = (BYTE *)pVertex;

				pVertex++;

				m_bCorner[pLink->iPix0] |= 0x02;
			}

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);		

		p2DRegion->m_Index = (iTriangle++);

		if(p2DRegion->m_nPts == 0)
			p2DRegion->m_Flags = RVLOBJ2_FLAG_REJECTED;
		else if(100 * p2DRegion->m_n3DPts / p2DRegion->m_nPts < m_fillPerc)
			p2DRegion->m_Flags = RVLOBJ2_FLAG_REJECTED;
		else
		{
			if(m_Flags & RVLPSD_FLAG_MM)
			{
				if(p2DRegion->m_Flags & RVL2DREGION_FLAG_LINE)
					p2DRegion->m_Flags = RVLOBJ2_FLAG_REJECTED;
				else
				{
					double *fN = p2DRegion->m_fN;

					double *X0 = m_Point3DMap[pLink->iPix0]->XYZ;

					double fTmp = sqrt(RVLDOTPRODUCT3(X0, X0));

					double V3Tmp[3];

					RVLSCALE3VECTOR2(X0, fTmp, V3Tmp);

					if(RVLDOTPRODUCT3(fN, V3Tmp) < m_MeshDiscontinuityThr)
						p2DRegion->m_Flags = RVLOBJ2_FLAG_REJECTED;
				}
			}
			else
			{
				N = p2DRegion->m_N;

				double fN[3];

				fN[0] = (double)N[0];
				fN[1] = (double)N[1];
				fN[2] = (double)N[2];

				p2DRegion->m_lenN = DOUBLE2INT(sqrt(fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2]));					
				
				if(p2DRegion->m_lenN == 0)
					p2DRegion->m_Flags = RVLOBJ2_FLAG_REJECTED;
				else if(1000 * p2DRegion->m_N[2] / p2DRegion->m_lenN < 100)
					p2DRegion->m_Flags = RVLOBJ2_FLAG_REJECTED;
				else
					p2DRegion->m_Flags = 0x00000000;
			}
		}
	}

#pragma endregion

	if((Flags & RVLPSD_MESH_SEGMENT_PLANAR) == 0)
		return;
	
	if(Flags & RVLPSD_MESH_SEGMENT_WER)
	{
		StartTime = m_pTimer->GetTime();

		if(m_Flags & RVLPSD_FLAG_MM)
			MeshSegmentWER(	p2DRegionSet,
							RVLMeshSegmentSWEROnCreateNewNode,
							RVLMeshSegmentSWERUpdateLink,
							RVLMeshSegmentWERGetCostLog);
		else
			MeshSegmentWER(	p2DRegionSet,
							RVLMeshSegmentSWEROnCreateNewNode,
							RVLMeshSegmentSWERUpdateLink,
							RVLMeshSegmentWERGetCost);			

		ExecutionTime = m_pTimer->GetTime() - StartTime;

		int nLevel3Segs = GenRelListFromWER(p2DRegionSet, p2DRegionSet3);
	}
	else
	{
		//FILKO COMMENTED OUT 30.06.2011   START

		//*** merging
		//Create LEVEL3 segments

		StartTime = m_pTimer->GetTime();

		int nLevel3Segs = SegmentSTRMLevel3(p2DRegionSet, p2DRegionSet3, pMem);

		return;  // REMOVE THIS AFTER SYROCO12


		ExecutionTime = m_pTimer->GetTime() - StartTime;

		
		//mark LEVEL3/LEVEL2 edges
		CRVL2DRegion2 *p2DRegionNeighbour;
		RVLMESH_LINK *pLinkNeighbour;

		//MARK LEVEL3 edges using LEVEL3 diff and set ROI flag
		CRVL2DRegion2 *p2DRegionLevel3, *p2DRegionNeghbourLevel3;
		p2DRegionList->Start();
		while(p2DRegionList->m_pNext)
		{
			p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

			if(p2DRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			pLink = pLink0 = (RVLMESH_LINK *)(p2DRegion->m_PtArray);
			
				do
				{
					pLink->Flags &= ~RVLMESH_LINK_FLAG_EDGE;

					pLinkNeighbour = pLink->pOpposite;
					p2DRegionNeighbour = (CRVL2DRegion2 *)(pLinkNeighbour->vp2DRegion);

					if(p2DRegion)
					{
						p2DRegionLevel3 = *(CRVL2DRegion2 **)(p2DRegion->m_pData + p2DRegionSet->m_iDataGrandParentPtr);

						if(p2DRegionLevel3 == NULL)
							p2DRegionLevel3 = p2DRegion;
						else
							pLink->Flags |= RVLMESH_LINK_FLAG_ROI;
					}
					else
						p2DRegionLevel3 = NULL;

					if(p2DRegionNeighbour)
					{
						p2DRegionNeghbourLevel3 = *(CRVL2DRegion2 **)(p2DRegionNeighbour->m_pData + p2DRegionSet->m_iDataGrandParentPtr);

						if(p2DRegionNeghbourLevel3 == NULL)
							p2DRegionNeghbourLevel3 = p2DRegionNeighbour;
					}
					else
						p2DRegionNeghbourLevel3 = NULL;

					if(p2DRegionLevel3 != p2DRegionNeghbourLevel3)
					{
						pLink->Flags |= RVLMESH_LINK_FLAG_EDGE;
						pLinkNeighbour->Flags |= RVLMESH_LINK_FLAG_EDGE;
					}

					pLink = pLink->pNext->pOpposite;

				}
				while(pLink != pLink0);
		}	
	}	

	// assign 2D regions obtained by triangle mesh segmentation to image pixels and opposite

	p2DRegionList =&(p2DRegionSet3->m_ObjectList);

	RVLQLIST *PtListMem;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, RVLQLIST, p2DRegionList->m_nElements, PtListMem);
	
	RVLQLIST *pPtList = PtListMem;

	RVLARRAY *pRelList;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		pRelList = p2DRegion->m_RelList + p2DRegionSet3->m_iRelListElements;

		pTriangle = *((CRVL2DRegion2 **)(pRelList->pFirst));

		p2DRegion->m_pPoint3DMap = pTriangle->m_pPoint3DMap;

		RVLQLIST_INIT(pPtList);

		p2DRegion->m_PtArray = pPtList++;

		p2DRegion->m_nPts = 0;
	}

	RVLQLIST_INT_ENTRY *PtListEntryMem;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, RVLQLIST_INT_ENTRY, ImageSize, PtListEntryMem)

	RVLQLIST_INT_ENTRY *PtListEntry = PtListEntryMem;

	CRVL2DRegion2 **p2DRegionMapEnd = m_2DRegionMap + ImageSize;

	CRVL2DRegion2 **p2DRegionPtr;
	
	for(p2DRegionPtr = m_2DRegionMap; p2DRegionPtr < p2DRegionMapEnd; p2DRegionPtr++)
	{
		pTriangle = *p2DRegionPtr;

		if(pTriangle == NULL)
			continue;

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
		{
			*p2DRegionPtr = NULL;

			continue;
		}

		p2DRegion = *((CRVL2DRegion2 **)(pTriangle->m_pData + pTriangle->m_pClass->m_iDataGrandParentPtr));

		*p2DRegionPtr = p2DRegion;

		if(p2DRegion == NULL)
			continue;

		PtListEntry->i = p2DRegionPtr - m_2DRegionMap;

		pPtList = (RVLQLIST *)(p2DRegion->m_PtArray);

		RVLQLIST_ADD_ENTRY(pPtList, PtListEntry);

		p2DRegion->m_nPts++;
	
		PtListEntry++;
	}

	if(Flags & RVLPSD_MESH_CONVEX)
	{
		StartTime = m_pTimer->GetTime();

		RVLRemoveInternalVertices(m_pDelaunay);

		StartTime = m_pTimer->GetTime();

		// update 3D points to triangles relation after removing internal vertices

		ppPtrArrayStart = Point3DPtrArray;

		p2DRegionList->Start();

		while(p2DRegionList->m_pNext)
		{
			p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

			if(p2DRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			p2DRegion->m_pPoint3DArray = ppPtrArrayStart;

			GetPointsWithDisparity(p2DRegion,ppPtrArrayStart);

			ppPtrArrayStart += p2DRegion->m_n3DPts;

			ppPtrArrayStart++;
		}

	#ifdef RVLPSD_SEGMENT_STRM_LOG_FILE
		fopen_s(&fp, "C:\\RVL\\ExpRez\\delaunay.dat", "w");

		RVLSaveSegmentation(fp, p2DRegionSet, m_Width);

		fclose(fp);
	#endif

		//StartTime = m_pTimer->GetTime();

		RVLSegmentToConvex(m_pDelaunay, 4, &m_DelaunayLinkQueue, p2DRegionList, RVLSEGMENT_TO_CONVEX_FLAG_ROI);

		//ExecutionTime = m_pTimer->GetTime() - StartTime;


		//Create LEVEL2 regions
		int nLevel2Segs = SegmentSTRMLevel2(p2DRegionSet, p2DRegionSet2, p2DRegionSet3, pMem);

		//Determine centroid and moments of LEVEL3 regions
		

	   //FILKO COMMENTED OUT 30.06.2011   END
	}
}

void CRVLPlanarSurfaceDetector::UpdateSTRMQueue(CRVL2DRegion2 **TriangleArray, 
												CRVL2DRegion2 **pTriangleArrayEnd,
												int &err,
												BOOL &bErrCorrection,
												RVLPSD_STRM_QUEUE_ENTRY **ppNewEntry)
{
	RVLPSD_STRM_QUEUE_ENTRY *pNewEntry = *ppNewEntry;

	bErrCorrection = FALSE;

	RVLQLIST *ListArray = m_Queue.m_ListArray;

	double uNrm = (double)(10000 * m_uNrm);

	RVLMESH_LINK *pLink, *pLink2;
	RVL3DPOINT2 *p3DPt0, *p3DPt1, *p3DPt2;
	int eMax, eThr, iPixeMax;
	CRVL2DRegion2 *pTriangle;
	CRVL2DRegion2 **ppTriangle;
	int dd1, dd2;
	int *N;
	double *fN;
	double dX1[3], dX2[3];
	//int nPts, n3DPts;
	//double fTmp;
	//double fN[3];

	for(ppTriangle = TriangleArray; ppTriangle < pTriangleArrayEnd; ppTriangle++)
	{
		pTriangle = *ppTriangle;

		pLink = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		pTriangle->m_Flags &= ~RVLOBJ2_FLAG_MARKED;

		if(pTriangle->m_vpQueueEntry)
			((RVLPSD_STRM_QUEUE_ENTRY *)(pTriangle->m_vpQueueEntry))->pRegion = NULL;

		// compute the parameters of the plane in which the triangle lies

		pLink2 = pLink->pNext;

		p3DPt0 = m_Point3DMap[pLink->iPix0];

		p3DPt1 = m_Point3DMap[pLink->pOpposite->iPix0];

		p3DPt2 = m_Point3DMap[pLink2->pOpposite->iPix0];

		if(m_Flags & RVLPSD_FLAG_MM)
		{
			double *X0 = p3DPt0->XYZ;
			double *X1 = p3DPt1->XYZ;
			double *X2 = p3DPt2->XYZ;

			RVLDIF3VECTORS(X1, X0, dX1)
			RVLDIF3VECTORS(X2, X0, dX2)

			fN = pTriangle->m_fN;

			RVLCROSSPRODUCT3(dX1, dX2, fN)

			double fTmp = sqrt(RVLDOTPRODUCT3(fN, fN)); 
			
			if(fTmp <= APPROX_ZERO && fTmp >= -APPROX_ZERO)
			{
				pTriangle->m_Flags |= RVL2DREGION_FLAG_LINE;

				eMax = 0;
			}
			else
			{
				RVLSCALE3VECTOR2(fN, fTmp, fN)

				pTriangle->m_rho = RVLDOTPRODUCT3(X0, fN);

				if(pTriangle->m_rho < 0.0)
				{
					fN[0] = -fN[0];
					fN[1] = -fN[1];
					fN[2] = -fN[2];

					pTriangle->m_rho = -pTriangle->m_rho;
				}

#ifdef RVLPSD_SEGMENT_STRM_PC_DEBUG
				if (bDebug)
					bDebugTriangle = false;
#endif

				GetMaxDeviation(pTriangle, eMax, iPixeMax, true);

#ifdef RVLPSD_SEGMENT_STRM_PC_DEBUG
				if (bDebug)
				{
					if (bDebugTriangle)
					{
						fpDebugTriangles = fopen("C:\\RVL\\Debug\\DebugTriangle.txt", "w");

						fprintf(fpDebugTriangles, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
							X0[0], X0[1], X0[2], X1[0], X1[1], X1[2], X2[0], X2[1], X2[2]);

						CRVL2DRegion2 *pDebugTriangle;

						bool bRecorded = false;

						std::list<CRVL2DRegion2 *>::iterator triangle = debugTriangles.begin();

						while (triangle != debugTriangles.end())
						{
							pDebugTriangle = *triangle;

							if (pDebugTriangle == pTriangle)
								bRecorded = true;

							RVLMESH_LINK *pLink = (RVLMESH_LINK *)(pDebugTriangle->m_PtArray);

							RVLMESH_LINK *pLink_ = pLink;

							bool bDebugTriangle_ = false;

							int iVertex = 0;
								
							int iPix;
							RVL3DPOINT2 *Vertex[3];
								
							do
							{
								iPix = pLink_->iPix0;
								
								if (bDebugMap[iPix])
									bDebugTriangle_ = true;

								Vertex[iVertex++] = m_Point3DMap[iPix];
								
								pLink_ = pLink_->pNext->pOpposite;
							}
							while (pLink_ != pLink);

							if (bDebugTriangle_)
							{
								fprintf(fpDebugTriangles, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
									Vertex[0]->XYZ[0], Vertex[0]->XYZ[1], Vertex[0]->XYZ[2], 
									Vertex[1]->XYZ[0], Vertex[1]->XYZ[1], Vertex[1]->XYZ[2], 
									Vertex[2]->XYZ[0], Vertex[2]->XYZ[1], Vertex[2]->XYZ[2]);

								triangle++;
							}
							else
								triangle = debugTriangles.erase(triangle);
						}

						if (!bRecorded)
							debugTriangles.push_back(pTriangle);						

						fclose(fpDebugTriangles);

						int debug = 0;

						if (debugTriangles.size() > 1)
							int debug_ = 0;
					}
				}
#endif
			}

			eThr = (m_Flags2 & RVLPSD_FLAG2_VELODYNE_SENSOR_UNCERT_MODEL ? 20.0 + 0.01 * m_Point3DMap[((RVLMESH_LINK *)(pTriangle->m_PtArray))->iPix0]->r : m_MeshTol);
		}	// if(m_Flags & RVLPSD_FLAG_MM)
		else
		{
			dd1 = p3DPt1->d - p3DPt0->d;
			dd2 = p3DPt2->d - p3DPt0->d;

			N = pTriangle->m_N;		

#ifdef NEVER
		fN[0] = (double)(pLink->dv * dd2 - dd1 * pLink2->dv);
		fN[1] = (double)(dd1 * pLink2->du - pLink->du * dd2);
		fN[2] = (double)(m_uNrm * (pLink->du * pLink2->dv - pLink->dv * pLink2->du));

		fTmp = sqrt(fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2]);

		N[0] = (int)((uNrm * fN[0] + 0.5) / fTmp);
		N[1] = (int)((uNrm * fN[1] + 0.5) / fTmp);
		N[2] = (int)((10000.0 * fN[2] + 0.5) / fTmp);

		pTriangle->m_d = N[0] * p3DPt0->u + N[1] * p3DPt0->v + N[2] * p3DPt0->d;
#endif

			N[0] = pLink->dv * dd2 - dd1 * pLink2->dv;
			N[1] = dd1 * pLink2->du - pLink->du * dd2;
			N[2] = -pLink->dv * pLink2->du + pLink->du * pLink2->dv;

			//pTriangle->m_d = N[0] * p3DPt0->u + N[1] * p3DPt0->v + N[2] * p3DPt0->d;

			if(N[2] < 0)
			{
				N[0] = -N[0];
				N[1] = -N[1];
				N[2] = -N[2];
			}

			if(N[2] > 0)
			{
				// for each point inside the triangle determine whether it lies in the triangle plane within a given tolerance or not
				// for the greatest deviation, create a queue entry

				GetMaxDeviation(pTriangle, eMax, iPixeMax);			
			}
			else
				eMax = 0;

			eThr = N[2] * m_uvdTol;
		}	// if(!(m_Flags & RVLPSD_FLAG_MM))

		//if(eMax <= 10000 * m_uvdTol)

		if(eMax == 0)
			continue;
		else if(eMax <= eThr)
		{
		//	pTriangle->m_Flags |= RVLOBJ2_FLAG_MARKED;

			if(pTriangle->m_nPts < 20)
				continue;

			if(100 * pTriangle->m_n3DPts / pTriangle->m_nPts >= m_fillPerc)
				continue;
		}

		if(m_Flags & RVLPSD_FLAG_MM)
		{
			if(eMax > 100)
				eMax = DOUBLE2INT(log((double)eMax / 100.0)/log(1.01))+100;

			if(eMax > 799)
				eMax = 799;
		}
		else
			eMax /= (16 * N[2]);

#ifdef RVLPSD_SEGMENT_STRM_PC_DEBUG
		FILE *fpNewVertex = fopen("C:\\RVL\\Debug\\NewVertex.txt", "w");

		double *newPt = m_Point3DMap[iPixeMax]->XYZ;

		fprintf(fpNewVertex, "%lf\t%lf\t%lf\n", newPt[0], newPt[1], newPt[2]);

		fclose(fpNewVertex);
#endif

		//eMax /= 160000;
		
		pTriangle->m_vpQueueEntry = pNewEntry;

		//if(pNewEntry - m_QueueMem == 98)
		//	int tmp1 = 0;

		RVLQLISTARRAY_ADD_ENTRY(ListArray, eMax, pNewEntry);

		pNewEntry->iPix = iPixeMax;
		pNewEntry->pRegion = pTriangle;

		pNewEntry++;

		//if(iPixeMax == 307 + 186 * 320)
		//	int debug = 0;

		if(eMax > err)
		{
			err = eMax;

			bErrCorrection = TRUE;
		}
	}	// for every triangle

	*ppNewEntry = pNewEntry;
}

void CRVLPlanarSurfaceDetector::GetMaxDeviation(CRVL2DRegion2 *pPolygon,
												int &eMax,
												int &iPixeMax,
												bool bFloat)
{
	RVLMESH_LINK *pLink = (RVLMESH_LINK *)(pPolygon->m_PtArray);

	int iFirstScanLine, iLastScanLine;		

	m_pDelaunay->GetPtsInConvexPolygon(pLink, m_iScanLineStart, m_iScanLineEnd, iFirstScanLine, iLastScanLine);

//#ifdef RVLPSD_SEGMENT_STRM_PC_DEBUG
//	if (bDebug)
//	{
//		RVLMESH_LINK *pLink_ = pLink;
//
//		int iPix;
//
//		do
//		{
//			iPix = pLink_->iPix0;
//
//			if (bDebugMap[iPix])
//				bDebugTriangle = true;
//
//			pLink_ = pLink_->pNext->pOpposite;
//		}
//		while (pLink_ != pLink);
//	}
//#endif

	int iPix0 = pLink->iPix0;

	int u0 = iPix0 % m_Width;

	int v0 = iPix0 / m_Width;

	int iPix = m_iScanLineStart[iFirstScanLine];

	int u = iPix % m_Width;

	int v = iPix / m_Width;

	//int k0 = N[1] * v - pPolygon->m_d;

	//int k0 = DOUBLE2INT(10000.0 * (pPolygon->m_a * (double)u0 + pPolygon->m_b * (double)v0 + pPolygon->m_c));

	int *N = pPolygon->m_N;

	double feMax;
	int k0, d0;
	double rho;
	double *fN;

	if(bFloat)
	{
		feMax = 0.0;
		fN = pPolygon->m_fN;
		rho = pPolygon->m_rho;
		N[0] = N[1] = N[2] = 0;
	}
	else
	{
		eMax = 0;	
		d0 = m_Point3DMap[iPix0]->d;
		N = pPolygon->m_N;
		k0 = N[1] * (v - v0) - N[2] * d0;
	}

	int nPts = 0;

	int n3DPts = 0;

	int k;
	int iScanLine;	
	int e;
	int d;
	double fe;
	double *X;

	for(iScanLine = iFirstScanLine; iScanLine <= iLastScanLine; iScanLine++, k0 += N[1])
	{
		iPix = m_iScanLineStart[iScanLine];

		u = iPix % m_Width;

		k = k0 + N[0] * (u - u0);

		for(; iPix <= m_iScanLineEnd[iScanLine]; iPix++, k += N[0])
		{
			nPts++;

			if(m_Point3DMap[iPix] == NULL)
				continue;

			n3DPts++;

			if(m_bCorner[iPix])
				continue;

			if(bFloat)
			{
#ifdef RVLPSD_SEGMENT_STRM_PC_DEBUG
				if (bDebug)
					if (bDebugMap[iPix])
						bDebugTriangle = true;
#endif
				X = m_Point3DMap[iPix]->XYZ;

				if (m_Flags & RVLPSD_FLAG_PC_BEAM_DISTANCE)
					fe = (rho / RVLDOTPRODUCT3(fN, X) - 1.0) * m_Point3DMap[iPix]->r;		// distance along the beam
				else
					fe = RVLDOTPRODUCT3(fN, X) - rho;	// distance to the plane
				
				if(fe < 0.0)
					fe = -fe;

				if(fe > feMax)
				{
					feMax = fe;

					iPixeMax = iPix;
				}
			}
			else
			{				
				d = m_Point3DMap[iPix]->d;

				e = k + N[2] * d;

				//if(e != (N[0]*(iPix % m_Width)+N[1]*(iPix/m_Width)+N[2]*m_Point3DMap[iPix]->d-pTriangle->m_d))
				//	int tmp1 = 0;

				if(e < 0)
					e = -e;

				if(e > eMax)
				{
					eMax = e;

					iPixeMax = iPix;
				}
			}
		}	// for each pixel within a scan line
	}	// for each scan line

	pPolygon->m_nPts = nPts;

	pPolygon->m_n3DPts = n3DPts;

	if(bFloat)
		eMax = DOUBLE2INT(feMax);
}

void CRVLPlanarSurfaceDetector::GetMaxDeviation(CRVLC2D * p2DRegionSet,
												int &eMax,
												int &iPixeMax)
{
	CRVLMPtrChain *pRegionList = (CRVLMPtrChain *)&(p2DRegionSet->m_ObjectList);

	CRVL2DRegion2 *pRegion;

	eMax = 0;

	int nPts = 0;

	int n3DPts = 0;

	int eMaxRegion;
	int iPixeMaxRegion;

	pRegionList->Start();

	while(pRegionList->m_pNext)
	{
		pRegion = (CRVL2DRegion2 *)(pRegionList->GetNext());

		GetMaxDeviation(pRegion, eMaxRegion, iPixeMaxRegion);

		if(eMaxRegion > eMax)
		{
			eMax = eMaxRegion;

			iPixeMax = iPixeMaxRegion;
		}
	}
}


void CRVLPlanarSurfaceDetector::SaveMesh(	FILE *fpPts,
											FILE *fpIdx,
											CRVLClass *p2DRegionSet,
											DWORD Flags,
											int min3DPtsPerc,
											DWORD Mask)	
{
	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	int ImageSize = m_Width * m_Height;

	int *IdxMap = new int[ImageSize];

	memset(IdxMap, 0, ImageSize * sizeof(int));

	int iPt = 1;

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pPolygon->m_Flags & Mask) != Mask)
			continue;

		if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);

		do
		{
			iPix = pLink->iPix0;

			if(IdxMap[iPix] == 0)
			{
				pPt = m_Point3DMap[iPix];

				if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
				{
					pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);

					if(pHullTriangle->m_lenN > 0)
					{
						N = pHullTriangle->m_N;

						fN[0] = (double)N[0];
						fN[1] = (double)N[1];
						fN[2] = (double)N[2];

						lenN2 = fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2];					

						k = (double)(pHullTriangle->m_d - (N[0] * pPt->u + N[1] * pPt->v + N[2] * pPt->d)) / lenN2;

						fprintf(fpPts, "%lf\t%lf\t%lf\n", 
							k * (double)N[0] + (double)(pPt->u),
							k * (double)N[1] + (double)(pPt->v),
							k * (double)N[2] + (double)(pPt->d));
					}
					else
						fprintf(fpPts, "%d\t%d\t%d\n", pPt->u, pPt->v, pPt->d);
				}
				else
					fprintf(fpPts, "%d\t%d\t%d\n", pPt->u, pPt->v, pPt->d);

				IdxMap[iPix] = iPt;

				iPt++;
			}

			fprintf(fpIdx, "%d\t", IdxMap[iPix]);

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);

		fprintf(fpIdx, "%d\n", pPolygon->m_Label);
	}

	delete[] IdxMap;
}

void CRVLPlanarSurfaceDetector::SaveMesh2Ply(	FILE *fpPly,
											CRVLClass *p2DRegionSet,
											DWORD Flags,
											int min3DPtsPerc,
											DWORD Mask)	
{
	//fopen_s(&fpPly, "C:\\RVL\\ExpRez\\mesh2PLY.ply", "w");
	//Elementi potrebni za stvaranje PLY datoteke
	char* vertices = new char[500000];
	memset(vertices, 0, 500000 * sizeof(char));
	char* indices = new char[500000];
	memset(indices, 0, 500000 * sizeof(char));
	//char* normals = new char[500000];
	//memset(normals, 0, 500000 * sizeof(char));
	char* temp = new char[200];
	memset(temp, 0, 200 * sizeof(char));
	int noV = 0;
	int noI = 0;

	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	int ImageSize = m_Width * m_Height;

	int *IdxMap = new int[ImageSize];

	memset(IdxMap, 0, ImageSize * sizeof(int));

	int iPt = 0;

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pPolygon->m_Flags & Mask) != Mask)
			continue;

		if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		
		sprintf(temp, "3\t");
		strcat(indices, temp);
		do
		{
			iPix = pLink->iPix0;

			if(IdxMap[iPix] == 0)
			{
				pPt = m_Point3DMap[iPix];

				if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
				{
					pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);

					if(pHullTriangle->m_lenN > 0)
					{
						N = pHullTriangle->m_N;

						fN[0] = (double)N[0];
						fN[1] = (double)N[1];
						fN[2] = (double)N[2];

						lenN2 = fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2];					

						k = (double)(pHullTriangle->m_d - (N[0] * pPt->u + N[1] * pPt->v + N[2] * pPt->d)) / lenN2;

						sprintf(temp, "%lf\t%lf\t%lf\n", 
							k * (double)N[0] + (double)(pPt->u),
							k * (double)N[1] + (double)(pPt->v),
							k * (double)N[2] + (double)(pPt->d));
						strcat(vertices, temp);
						noV++;
					}
					else
					{
						sprintf(temp, "%d\t%d\t%d\n", pPt->u, pPt->v, pPt->d);
						strcat(vertices, temp);
						noV++;
					}
				}
				else
				{
					sprintf(temp, "%d\t%d\t%d\n", pPt->u, pPt->v, pPt->d);
					strcat(vertices, temp);
					noV++;
				}

				IdxMap[iPix] = iPt;

				iPt++;
			}

			sprintf(temp, "%d\t", IdxMap[iPix]);
			strcat(indices, temp);

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);

		sprintf(temp, "\n");
		strcat(indices, temp);
		noI++;
		/*pHullTriangle = (CRVL2DRegion2 *)(pLink0->pData);
		if ( pHullTriangle->m_lenN != 0)
			sprintf(temp, "%.4f\t%.4f\t%.4f\n", pHullTriangle->m_N[0] / pHullTriangle->m_lenN, pHullTriangle->m_N[1] / pHullTriangle->m_lenN, pHullTriangle->m_N[2] / pHullTriangle->m_lenN);
		else
			sprintf(temp, "%0\t0\t0\n");
		strcat(normals, temp);*/
	}
	fprintf(fpPly, "ply\nformat ascii 1.0\n");
	fprintf(fpPly, "element vertex %d\n", noV);
	fprintf(fpPly, "property float x\nproperty float y\nproperty float z\n");
	fprintf(fpPly, "element face %d\n", noI);
	fprintf(fpPly, "property list uchar int vertex_indices\n");
	//fprintf(fpPly, "element normal %d\n", noI);
	//fprintf(fpPly, "property float x\nproperty float y\nproperty float z\n");
	fprintf(fpPly, "end_header\n");
	fprintf(fpPly, "%s", vertices);
	fprintf(fpPly, "%s", indices);
	//fprintf(fpPly, "%s", normals);


	delete[] IdxMap;
	delete[] vertices;
	delete[] indices;
	//delete[] normals;
	delete[] temp;
}

void CRVLPlanarSurfaceDetector::SaveMesh2Ply2(	FILE *fpPly2,
											CRVLClass *p2DRegionSet,
											DWORD Flags,
											int min3DPtsPerc,
											DWORD Mask)	
{
	//fopen_s(&fpPly2, "C:\\RVL\\ExpRez\\mesh2PLY.ply2", "w");
	//Elementi potrebni za stvaranje PLY2 datoteke
	char* vertices = new char[500000];
	memset(vertices, 0, 500000 * sizeof(char));
	char* indices = new char[500000];
	memset(indices, 0, 500000 * sizeof(char));
	//char* normals = new char[500000];
	//memset(normals, 0, 500000 * sizeof(char));
	char* temp = new char[200];
	memset(temp, 0, 200 * sizeof(char));
	int noV = 0;
	int noI = 0;

	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	int ImageSize = m_Width * m_Height;

	int *IdxMap = new int[ImageSize];

	memset(IdxMap, 0, ImageSize * sizeof(int));

	int iPt = 0;

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pPolygon->m_Flags & Mask) != Mask)
			continue;

		if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		
		sprintf(temp, "3\t");
		strcat(indices, temp);
		do
		{
			iPix = pLink->iPix0;

			if(IdxMap[iPix] == 0)
			{
				pPt = m_Point3DMap[iPix];

				if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
				{
					pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);

					if(pHullTriangle->m_lenN > 0)
					{
						N = pHullTriangle->m_N;

						fN[0] = (double)N[0];
						fN[1] = (double)N[1];
						fN[2] = (double)N[2];

						lenN2 = fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2];					

						k = (double)(pHullTriangle->m_d - (N[0] * pPt->u + N[1] * pPt->v + N[2] * pPt->d)) / lenN2;

						sprintf(temp, "%lf\t%lf\t%lf\n", 
							k * (double)N[0] + (double)(pPt->u),
							k * (double)N[1] + (double)(pPt->v),
							k * (double)N[2] + (double)(pPt->d));
						strcat(vertices, temp);
						noV++;
					}
					else
					{
						sprintf(temp, "%d\t%d\t%d\n", pPt->u, pPt->v, pPt->d);
						strcat(vertices, temp);
						noV++;
					}
				}
				else
				{
					sprintf(temp, "%d\t%d\t%d\n", pPt->u, pPt->v, pPt->d);
					strcat(vertices, temp);
					noV++;
				}

				IdxMap[iPix] = iPt;

				iPt++;
			}

			sprintf(temp, "%d\t", IdxMap[iPix]);
			strcat(indices, temp);

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);

		sprintf(temp, "\n");
		strcat(indices, temp);
		noI++;
		//pHullTriangle = (CRVL2DRegion2 *)(pLink0->pData);
		//sprintf(temp, "%.4f\t%.4f\t%.4f\n", pHullTriangle->m_N[0], pHullTriangle->m_N[1], pHullTriangle->m_N[2]);
		//strcat(normals, temp);
	}
	fprintf(fpPly2, "%d\n", noV);
	fprintf(fpPly2, "%d\n", noI);
	fprintf(fpPly2, "%s", vertices);
	fprintf(fpPly2, "%s", indices);

	delete[] IdxMap;
	delete[] vertices;
	delete[] indices;
	delete[] temp;
}

void CRVLPlanarSurfaceDetector::SaveMesh2OFF(	FILE *fpOFF,
											CRVLClass *p2DRegionSet,
											DWORD Flags,
											int min3DPtsPerc,
											DWORD Mask)	
{
	//fopen_s(&fpOFF, "C:\\RVL\\ExpRez\\mesh2OFF.off", "w");
	//Elementi potrebni za stvaranje PLY2 datoteke
	char* vertices = new char[500000];
	memset(vertices, 0, 500000 * sizeof(char));
	char* indices = new char[500000];
	memset(indices, 0, 500000 * sizeof(char));
	//char* normals = new char[500000];
	//memset(normals, 0, 500000 * sizeof(char));
	char* temp = new char[200];
	memset(temp, 0, 200 * sizeof(char));
	int noV = 0;
	int noI = 0;

	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	int ImageSize = m_Width * m_Height;

	int *IdxMap = new int[ImageSize];

	memset(IdxMap, 0, ImageSize * sizeof(int));

	int iPt = 0;

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pPolygon = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pPolygon->m_Flags & Mask) != Mask)
			continue;

		if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		
		sprintf(temp, "3 ");
		strcat(indices, temp);
		do
		{
			iPix = pLink->iPix0;

			if(IdxMap[iPix] == 0)
			{
				pPt = m_Point3DMap[iPix];

				if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
				{
					pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);

					if(pHullTriangle->m_lenN > 0)
					{
						N = pHullTriangle->m_N;

						fN[0] = (double)N[0];
						fN[1] = (double)N[1];
						fN[2] = (double)N[2];

						lenN2 = fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2];					

						k = (double)(pHullTriangle->m_d - (N[0] * pPt->u + N[1] * pPt->v + N[2] * pPt->d)) / lenN2;

						sprintf(temp, "%lf %lf %lf\n", 
							k * (double)N[0] + (double)(pPt->u),
							k * (double)N[1] + (double)(pPt->v),
							k * (double)N[2] + (double)(pPt->d));
						strcat(vertices, temp);
						noV++;
					}
					else
					{
						sprintf(temp, "%d %d %d\n", pPt->u, pPt->v, pPt->d);
						strcat(vertices, temp);
						noV++;
					}
				}
				else
				{
					sprintf(temp, "%d %d %d\n", pPt->u, pPt->v, pPt->d);
					strcat(vertices, temp);
					noV++;
				}

				IdxMap[iPix] = iPt;

				iPt++;
			}

			sprintf(temp, "%d ", IdxMap[iPix]);
			strcat(indices, temp);

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);

		sprintf(temp, "\n");
		strcat(indices, temp);
		noI++;
		//pHullTriangle = (CRVL2DRegion2 *)(pLink0->pData);
		//sprintf(temp, "%.4f\t%.4f\t%.4f\n", pHullTriangle->m_N[0], pHullTriangle->m_N[1], pHullTriangle->m_N[2]);
		//strcat(normals, temp);
	}
	fprintf(fpOFF, "OFF\n", noV);
	fprintf(fpOFF, "%d %d %d\n", noV, noI, noV + noI - 2);
	fprintf(fpOFF, "%s", vertices);
	fprintf(fpOFF, "%s", indices);

	delete[] IdxMap;
	delete[] vertices;
	delete[] indices;
	delete[] temp;
}


int CRVLPlanarSurfaceDetector::SegmentSTRMLevel3(CRVLC2D *pTriangleSetLevel1,
												 CRVLC2D *pTriangleSetLevel3,
												 CRVLMem *pMem)
{
	int maxSize = 0;

	//double fN[3];
	//int *N;
	CRVL2DRegion2 *pTriangle;
	RVLMESH_LINK *pLink;
	RVL3DPOINT2 *p3DPt0, *p3DPt1, *p3DPt2;
	int du2, dv2, du1, dv1;
	int size;

	CRVL2DRegion2 *p2DSegmentLevel3, **pp2DSegmentLevel3, **pp2DSegmentLevel2;


	//Delete all LEVEL3 segments
	pTriangleSetLevel3->m_ObjectList.RemoveAll();

	int *SizeArray = new int[pTriangleSetLevel1->m_ObjectList.m_nElements];

	//RVLClearFlags(pTriangleSetLevel1);

	CRVLMPtrChain *pTriangleList = &(pTriangleSetLevel1->m_ObjectList);

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		//Set pointer to LEVEL3 segment to NULL
		pp2DSegmentLevel3 = (CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr);
		*pp2DSegmentLevel3 = NULL;


		//Set pointer to LEVEL2 segment to NULL
		pp2DSegmentLevel2 = (CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataParentPtr);
		*pp2DSegmentLevel2 = NULL;

		//reset flags
		pTriangle->m_Flags &= ~RVL2DREGION_FLAG_MARKED3;

		pTriangle->m_Label = 0xffffffff;

		//N = pTriangle->m_N;

		//fN[0] = (double)N[0];
		//fN[1] = (double)N[1];
		//fN[2] = (double)N[2];

		//pTriangle->m_lenN = DOUBLE2INT(sqrt(fN[0] * fN[0] + fN[1] * fN[1] + fN[2] * fN[2]));					

		//if(pTriangle->m_lenN > 0)
		//{
		//	if((1000 * pTriangle->m_N[2] / pTriangle->m_lenN < 100) ||  //<------------ !!!  (80)
		//	  (100 * pTriangle->m_n3DPts/(pTriangle->m_nPts * 1.0) < m_fillPerc))
		//	{
		//		pTriangle->m_Flags |= RVLOBJ2_FLAG_REJECTED;

		//		continue;
		//	}
		//	
		//}
		pLink = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		p3DPt0 = m_Point3DMap[pLink->iPix0];

		p3DPt1 = m_Point3DMap[pLink->pNext->pOpposite->iPix0];

		p3DPt2 = m_Point3DMap[pLink->pOpposite->iPix0];

		pTriangle->m_d = pTriangle->m_N[0]*p3DPt0->u + pTriangle->m_N[1]*p3DPt0->v + pTriangle->m_N[2]*p3DPt0->d;

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

	RVLQLIST_PTR_ENTRY *LinkQueueEntryMem = new RVLQLIST_PTR_ENTRY[6 * m_Width * m_Height];

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

		RVLQLISTARRAY_ADD_ENTRY(TriangleQueueListArray, pTriangle->m_Size, pTriangleQueueEntry);

		pTriangleQueueEntry->Ptr = pTriangle;

		pTriangleQueueEntry++;
	}	



	CRVL2DRegion2 **ppGroupedTriangleList, **ppGroupedTriangleStart;
	RVLARRAY *pRelList;


	//contains LEVEL1 segments grouped by LEVEL3 segments
	CRVL2DRegion2 **GroupedTriangleList31 = (CRVL2DRegion2 **)(m_pMem->Alloc(pTriangleSetLevel1->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	memset(GroupedTriangleList31, 0x00, pTriangleSetLevel1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

	ppGroupedTriangleList = GroupedTriangleList31;

	DWORD Label = 0;

	int nTriangles = 0;

	int iTriangleQueueBin;

	for(iTriangleQueueBin = maxSize; iTriangleQueueBin >= 0; iTriangleQueueBin--)
	{
		pTriangleQueueEntry = (RVLQLIST_PTR_ENTRY *)(TriangleQueueListArray[iTriangleQueueBin].pFirst);

		while(pTriangleQueueEntry)
		{
			pTriangle = (CRVL2DRegion2 *)(pTriangleQueueEntry->Ptr);

			if((pTriangle->m_Label == 0xffffffff) && ((pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED) ==0) && (pTriangle->m_n3DPts>10))
			{
				nTriangles = 0;

				//if((int)Label==92)
				//	int gg=0;

				//Create LEVEL3 segment
				p2DSegmentLevel3 = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(pTriangleSetLevel3));
				RVLQLIST *pSamples = &(p2DSegmentLevel3->m_Samples);
				RVLQLIST_INIT(pSamples)

				//initialize LEVEL3 segment properties
				p2DSegmentLevel3->m_N[0] = pTriangle->m_N[0];
				p2DSegmentLevel3->m_N[1] = pTriangle->m_N[1];
				p2DSegmentLevel3->m_N[2] = pTriangle->m_N[2];
				p2DSegmentLevel3->m_lenN = pTriangle->m_lenN;

				p2DSegmentLevel3->m_d = pTriangle->m_d;
				p2DSegmentLevel3->m_nPts = pTriangle->m_nPts;
				p2DSegmentLevel3->m_n3DPts = pTriangle->m_n3DPts;

				//move pointer to first empty location
				ppGroupedTriangleStart = ppGroupedTriangleList;

				//add first triangle
				//pTriangle->m_Label = Label;
				*(ppGroupedTriangleList++) = pTriangle;

				//relate LEVEL3 segment to LEVEL1 triangle
				pp2DSegmentLevel3 = (CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr);
				*pp2DSegmentLevel3 = p2DSegmentLevel3;
				

				SizeArray[Label] = SegmentSTRMGrowing(ppGroupedTriangleList, pTriangle, p2DSegmentLevel3, 
													  LinkQueueEntryMem, pMem, Label, nTriangles,pTriangleSetLevel1);

				ppGroupedTriangleList += nTriangles;

				
				//update LEVEL3 segment properties
				p2DSegmentLevel3->m_Size = SizeArray[Label];
				p2DSegmentLevel3->m_a = pTriangle->m_a;
				p2DSegmentLevel3->m_b = pTriangle->m_b;
				p2DSegmentLevel3->m_c = pTriangle->m_c;


				pRelList = p2DSegmentLevel3->m_RelList + pTriangleSetLevel3->m_iRelList[RVLRELLIST_ELEMENTS];

				//updating segment params
				pRelList->pFirst = (unsigned char *)ppGroupedTriangleStart;
				pRelList->pEnd = (unsigned char *)ppGroupedTriangleList;

				Label++;
			}

			pTriangleQueueEntry = (RVLQLIST_PTR_ENTRY *)(pTriangleQueueEntry->pNext);
		}
	}




	//Sort LEVEL3 segments by m_n3DPts???
	
	

#ifdef RVLPSD_SEGMENT_STRM_LOG_FILE

	FILE *fpPts, *fpIdx, *fpSizes;

	fopen_s(&fpPts, "C:\\RVL\\ExpRez\\mesh2Pts.dat", "w");

	fopen_s(&fpIdx, "C:\\RVL\\ExpRez\\mesh2Idx.dat", "w");

	fopen_s(&fpSizes, "C:\\RVL\\ExpRez\\mesh2Sizes.dat", "w");

	SaveMesh(fpPts, fpIdx, pTriangleSetLevel1);

	int i;
	for(i = 0; i < (int)Label; i++)
			fprintf(fpSizes, "%d\n", SizeArray[i]);

	fclose(fpSizes);

	fclose(fpPts);

	fclose(fpIdx);
#endif

	delete[] LinkQueueEntryMem;

	delete[] TriangleQueueEntryMem;

	delete[] SizeArray;

	return (int)Label;
}



int CRVLPlanarSurfaceDetector::SegmentSTRMGrowing(CRVL2DRegion2 **ppGroupedTriangleList,
													CRVL2DRegion2 *pTriangleSrc,
													CRVL2DRegion2 *p2DSegmentLevel3,
													RVLQLIST_PTR_ENTRY *LinkQueueEntryMem,
													CRVLMem *pMem,
													DWORD Label,
													int &nTriangles,
													CRVLC2D *p2DTriangleSet)
{
	RVL3DPOINT2 *p3DPt;   //, *p3DPtSrc;

	CRVL2DRegion2 **pp2DSegmentLevel3;

	RVLMESH_LINK *pLink = (RVLMESH_LINK *)(pTriangleSrc->m_PtArray);

	//p3DPtSrc = m_Point3DMap[pLink->iPix0];


	//Perform LS fitting on plane
	LSPlane(pTriangleSrc->m_pPoint3DArray,pTriangleSrc->m_n3DPts,pTriangleSrc);

	////**************TEST start************** comment out
	//double eN=0.0, eA=0.0;
	//double dN, dA;
	//for(int i = 0; i<pTriangleSrc->m_n3DPts;i++)
	//{
	//	p3DPt = pTriangleSrc->m_pPoint3DArray[i];
	//	
	//	dA = pTriangleSrc->m_a*p3DPt->u + pTriangleSrc->m_b*p3DPt->v + pTriangleSrc->m_c - p3DPt->d;
	//	eA += dA*dA;

	//	dN = (pTriangleSrc->m_N[0]*p3DPt->u + pTriangleSrc->m_N[1]*p3DPt->v + pTriangleSrc->m_N[2]*p3DPt->d - pTriangleSrc->m_d)/(pTriangleSrc->m_N[2]*1.0);
	//	eN += dN * dN;

	//}

	//if(eA>eN)
	//	int ggg = 67;
	////**************TEST end************** comment out

	RVLQLIST LinkQueue;

	RVLQLIST *pLinkQueue = &LinkQueue;

	RVLQLIST_INIT(pLinkQueue);

	RVLQLIST_PTR_ENTRY *pLinkPtr = LinkQueueEntryMem;

	pTriangleSrc->m_Label = Label;

	//add original plane size
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


	CRVL2DRegion2 *pTriangle, *pTriangle2, *pTriangle3;
	int *N, *N2;
	double cosAngle, maxCosAngle;
	void **ppLinkPtr, **ppBestLinkPtr;
	//double fN[3];
	RVLQLIST_PTR_ENTRY *pLinkPtr2;
	BOOL bAddToPlane;

	int dist;

	int cntdebug = 0;
	while(TRUE)
	{
		maxCosAngle = -2.0;

		ppLinkPtr = &(LinkQueue.pFirst);

		pLinkPtr2 = (RVLQLIST_PTR_ENTRY *)(LinkQueue.pFirst);

		ppBestLinkPtr = NULL;
	
		cntdebug = 0;

		while(pLinkPtr2)
		{
			pLink = (RVLMESH_LINK *)(pLinkPtr2->Ptr);

			pTriangle = (CRVL2DRegion2 *)(pLink->vp2DRegion);

			N = pTriangle->m_N;

			pTriangle2 = (CRVL2DRegion2 *)(pLink->pOpposite->vp2DRegion);

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

			ppLinkPtr = &(pLinkPtr2->pNext);

			pLinkPtr2 = (RVLQLIST_PTR_ENTRY *)(pLinkPtr2->pNext);
			cntdebug++;
		}		

		if(ppBestLinkPtr == NULL)
			break;

		pLinkPtr2 = (RVLQLIST_PTR_ENTRY *)(*ppBestLinkPtr);

		if(pLinkPtr2->pNext == NULL)
			LinkQueue.ppNext = ppBestLinkPtr;

		*ppBestLinkPtr = pLinkPtr2->pNext;

		pLink = (RVLMESH_LINK *)(pLinkPtr2->Ptr);

		pTriangle = (CRVL2DRegion2 *)(pLink->pPrev->vp2DRegion);

		//debug
		//p3DPt =  m_Point3DMap[pLink->iPix0];

		//
		//if(((p3DPt->u == 248) || (p3DPt->u ==249)) && (p3DPt->v==211))
		//	int ghu = 89;
		//end debug
		

		bAddToPlane = FALSE;

		if(pTriangle)
		{
			if((pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
			{
				pTriangle2 = (CRVL2DRegion2 *)(pLink->pPrev->pOpposite->vp2DRegion);

				if((pTriangle2 != NULL) && ((pTriangle2->m_Flags & RVLOBJ2_FLAG_REJECTED) ==0))
					if(pTriangle2->m_Label == Label)
						bAddToPlane = TRUE;

				pTriangle3 = (CRVL2DRegion2 *)(pLink->pOpposite->pNext->vp2DRegion);

				if((pTriangle3 != NULL) && ((pTriangle3->m_Flags & RVLOBJ2_FLAG_REJECTED) ==0))
					if(pTriangle3->m_Label == Label)
						bAddToPlane = TRUE;

				//check if new pt lies in the original plane
				if(!bAddToPlane)
				{
					p3DPt =  m_Point3DMap[pLink->pPrev->pOpposite->iPix0];

					////debug
					//if(((p3DPt->u == 248) || (p3DPt->u ==249)) && (p3DPt->v==211))
					//	int ghu = 89;
					////end debug
					
					//use original plane params
					dist = pTriangleSrc->m_N[0]*p3DPt->u + pTriangleSrc->m_N[1]*p3DPt->v + pTriangleSrc->m_N[2]*p3DPt->d;
					if(abs(dist - pTriangleSrc->m_d ) < m_uvdTol*pTriangleSrc->m_lenN)  //m_uvdTol    6*pTriangleSrc->m_lenN
						bAddToPlane = TRUE;

					//use LS plane params
					//dist = abs(DOUBLE2INT((p3DPt->d - (pTriangleSrc->m_a*p3DPt->u + pTriangleSrc->m_b*p3DPt->v + pTriangleSrc->m_c))));
					//if(dist < m_uvdTol)
					//	bAddToPlane = TRUE;

				}
					
			}
		}

		if(bAddToPlane)
		{
			pTriangle->m_Label = Label;

			//add triangle to list
			*(ppGroupedTriangleList++) = pTriangle;

			Size += pTriangle->m_Size;

			nTriangles++;


			//update p2DSegmentLevel3 properties
			p2DSegmentLevel3->m_nPts += pTriangle->m_nPts;
			p2DSegmentLevel3->m_n3DPts += pTriangle->m_n3DPts;

			//relate LEVEL3 segment to LEVEL1
			pp2DSegmentLevel3 = (CRVL2DRegion2 **)(pTriangle->m_pData + p2DTriangleSet->m_iDataGrandParentPtr);
			*pp2DSegmentLevel3 = p2DSegmentLevel3;

			

			if(pTriangle2)
				if((pTriangle2->m_Label == 0xffffffff) && ((pTriangle2->m_Flags & RVLOBJ2_FLAG_REJECTED) ==0))
				{
					RVLQLIST_ADD_ENTRY(pLinkQueue, pLinkPtr);

					pLinkPtr->Ptr = pLink->pPrev;

					pLinkPtr++;
				}

			if(pTriangle3)
				if((pTriangle3->m_Label == 0xffffffff) && ((pTriangle3->m_Flags & RVLOBJ2_FLAG_REJECTED) ==0))
				{
					RVLQLIST_ADD_ENTRY(pLinkQueue, pLinkPtr);

					pLinkPtr->Ptr = pLink->pOpposite->pNext->pOpposite;

					pLinkPtr++;
				}
		}

		
	}

	
	return Size;
}	


void CRVLPlanarSurfaceDetector::GetPointsWithDisparity(CRVL2DRegion2 *pPolygon, RVL3DPOINT2 **ppPtrArray)
{
	

	int iFirstScanLine, iLastScanLine, iScanLine;		
	int iPix;
	int nPts = 0;
	int n3DPts = 0;
	
	RVL3DPOINT2 *pPoint3D;

	RVLMESH_LINK *pLink = (RVLMESH_LINK *)(pPolygon->m_PtArray);
	m_pDelaunay->GetPtsInConvexPolygon(pLink, m_iScanLineStart, m_iScanLineEnd, iFirstScanLine, iLastScanLine);


	for(iScanLine = iFirstScanLine; iScanLine <= iLastScanLine; iScanLine++)
	{
		iPix = m_iScanLineStart[iScanLine];

		for(; iPix <= m_iScanLineEnd[iScanLine]; iPix++)
		{
			m_2DRegionMap[iPix] = pPolygon;

			nPts++;

			pPoint3D = m_Point3DMap[iPix];
			
			if( pPoint3D == NULL)
				continue;

			n3DPts++;
			*ppPtrArray = pPoint3D;
			ppPtrArray++;
			pPoint3D->vp2DRegion = pPolygon;
			
		}
	}	

	pPolygon->m_nPts = nPts;

	pPolygon->m_n3DPts = n3DPts;
}





int CRVLPlanarSurfaceDetector::SegmentSTRMLevel2(CRVLC2D *pTriangleSetLevel1, 
												 CRVLC2D *pTriangleSetLevel2, 
												 CRVLC2D *pTriangleSetLevel3,
												 CRVLMem *pMem)
{

	//1. CREATE LEVEL2 regions using convex regions

	//RVL2DREGION_FLAG_MARKED3	-> indicates a LEVEL1 segment has been added to a LEVEL2 segment

	int nPts, n3DPts;

	int nLevel2Segments = 0;

	RVLARRAY *pRelList, *pRelList3;

	CRVL2DRegion2 *pTriangle, *pTriangleNeighbour, *p2DRegion;//, *pTriangleNeighbourLevel3

	CRVL2DRegion2 **ppTriangle, **ppTriangleEnd;

	CRVL2DRegion2 *p2DSegmentLevel2, **pp2DSegmentLevel2;

	CRVL2DRegion2 *p2DCurrentSegmentLevel3, **pp2DSegmentLevel3;

	CRVL2DRegion2 **ppGroupedTriangleList, **ppGroupedTriangleStart, **ppGroupedTriangle;

	CRVL2DRegion2 **ppGroupedTriangleListHelper, **ppGroupedTriangleListHelper1;

	CRVL2DRegion2 **ppGroupedLEVEL2List, **ppGroupedLEVEL2Start;

	CRVL2DRegion2 *p2DSegmentLevel1;//, *p2DSegmentLevel3;

	RVLMESH_LINK *pLink, *pLink0, *pLinkNeighbour;

	BOOL bEdgeFound = 0;

	//contains LEVEL2 segments grouped by LEVEL3 segments
	CRVL2DRegion2 **GroupedLEVEL2List32 = (CRVL2DRegion2 **)(m_pMem->Alloc(pTriangleSetLevel1->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	memset(GroupedLEVEL2List32, 0x00, pTriangleSetLevel1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

	ppGroupedLEVEL2List = GroupedLEVEL2List32;


	//contains LEVEL1 segments grouped by LEVEL2 segments
	CRVL2DRegion2 **GroupedTriangleList21 = (CRVL2DRegion2 **)(m_pMem->Alloc(pTriangleSetLevel1->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	memset(GroupedTriangleList21, 0x00, pTriangleSetLevel1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

	ppGroupedTriangleList = GroupedTriangleList21;

	//contains set of grouped (sub) segments 
	CRVL2DRegion2 **GroupedTriangleListHelper = new CRVL2DRegion2 *[pTriangleSetLevel1->m_ObjectList.m_nElements]; 
	memset(GroupedTriangleListHelper, 0x00, pTriangleSetLevel1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));


	//Delete all LEVEL2 segments
	pTriangleSetLevel2->m_ObjectList.RemoveAll();


	//Iterate through all triangles of each LEVEL3 and create LEVEL2 regions
	//CRVLMPtrChain *pTriangleList;
	CRVLMPtrChain *pTriangleList3 = &(pTriangleSetLevel3->m_ObjectList);

	//for each LEVEL3
	pTriangleList3->Start();

	while(pTriangleList3->m_pNext)
	{
		//get current LEVEL3 segment
		p2DCurrentSegmentLevel3 = (CRVL2DRegion2 *)(pTriangleList3->GetNext());

		//move pointer to first empty location for LEVEl2 list
		ppGroupedLEVEL2Start = ppGroupedLEVEL2List;


		//get LEVEL1 elements from rellist
		pRelList3 = p2DCurrentSegmentLevel3->m_RelList + pTriangleSetLevel3->m_iRelList[RVLRELLIST_ELEMENTS];
		ppTriangle = (CRVL2DRegion2 **)pRelList3->pFirst;
		ppTriangleEnd = (CRVL2DRegion2 **)pRelList3->pEnd;

		
		//For each LEVEL1
		for(; ppTriangle < ppTriangleEnd; ppTriangle++)
		{
			pTriangle = *ppTriangle;


			//if rejected continue
			if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			//if triangle already added to LEVEL2 continue
			if(pTriangle->m_Flags & RVL2DREGION_FLAG_MARKED3)
				continue;
			
			
			//reset counters
			nPts = 0;
			n3DPts = 0;

			//reset helper array
			memset(GroupedTriangleListHelper, 0x00, pTriangleSetLevel1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));

			ppGroupedTriangleListHelper = ppGroupedTriangleListHelper1 = GroupedTriangleListHelper;
			
			//move pointer to first empty location for LEVEl1 list
			ppGroupedTriangleStart = ppGroupedTriangleList;
			

			//Create LEVEL2 segment
			p2DSegmentLevel2 = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(pTriangleSetLevel2));
			nLevel2Segments++;

			//add LEVEL2 segment
			*(ppGroupedLEVEL2List++) = p2DSegmentLevel2;

			//initialize using LEVEL3 segment properties
			p2DSegmentLevel2->m_N[0] = p2DCurrentSegmentLevel3->m_N[0];
			p2DSegmentLevel2->m_N[1] = p2DCurrentSegmentLevel3->m_N[1];
			p2DSegmentLevel2->m_N[2] = p2DCurrentSegmentLevel3->m_N[2];
			p2DSegmentLevel2->m_lenN = p2DCurrentSegmentLevel3->m_lenN;
			
			p2DSegmentLevel2->m_a = p2DCurrentSegmentLevel3->m_a;
			p2DSegmentLevel2->m_b = p2DCurrentSegmentLevel3->m_b;
			p2DSegmentLevel2->m_c = p2DCurrentSegmentLevel3->m_c;
			p2DSegmentLevel2->m_d = p2DCurrentSegmentLevel3->m_d;
				

			//add first triangle
			pTriangle->m_Flags |= RVL2DREGION_FLAG_MARKED3;
			*(ppGroupedTriangleList++) = pTriangle;
			*(ppGroupedTriangleListHelper++) = pTriangle;

			//relate LEVEL2 segment to LEVEL1 triangle
			pp2DSegmentLevel2 = (CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataParentPtr);
			*pp2DSegmentLevel2 = p2DSegmentLevel2;
		
			nPts += pTriangle->m_nPts;
			n3DPts += pTriangle->m_n3DPts;

			

			while(ppGroupedTriangleListHelper1 < ppGroupedTriangleListHelper)
			{
				p2DSegmentLevel1 = *ppGroupedTriangleListHelper1;

				//p2DSegmentLevel3 = *((CRVL2DRegion2 **)(p2DSegmentLevel1->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr));

				//if(p2DSegmentLevel3 == p2DCurrentSegmentLevel3)
				//{
					//Get neighbours and expand
					pLink = pLink0 = (RVLMESH_LINK *)(p2DSegmentLevel1->m_PtArray);
					do
					{
						pLinkNeighbour = pLink->pOpposite;
						pTriangleNeighbour = (CRVL2DRegion2 *)(pLinkNeighbour->vp2DRegion);

						if((pTriangleNeighbour) && ((pTriangleNeighbour->m_Flags & RVLOBJ2_FLAG_REJECTED)==0) && ((pTriangleNeighbour->m_Flags & RVL2DREGION_FLAG_MARKED3)==0))
						{
							//pTriangleNeighbourLevel3 = *((CRVL2DRegion2 **)(pTriangleNeighbour->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr));
							
							//check neighbour triangle
							//neighbour triangle should have the same LEVEL3 and the link between them should not be a boundary/edge
							//if((pTriangleNeighbourLevel3 == p2DCurrentSegmentLevel3) && ((pLinkNeighbour->Flags & RVLMESH_LINK_FLAG_EDGE)==0))
							if((pLinkNeighbour->Flags & RVLMESH_LINK_FLAG_EDGE)==0)
							{
							
								pTriangleNeighbour->m_Flags |= RVL2DREGION_FLAG_MARKED3;

								*(ppGroupedTriangleList++) = pTriangleNeighbour;
								*(ppGroupedTriangleListHelper++) = pTriangleNeighbour;
							

								pp2DSegmentLevel2 = (CRVL2DRegion2 **)(pTriangleNeighbour->m_pData + pTriangleSetLevel1->m_iDataParentPtr);
								*pp2DSegmentLevel2 = p2DSegmentLevel2;

								
								nPts += pTriangleNeighbour->m_nPts;
								n3DPts += pTriangleNeighbour->m_n3DPts;
							}
						}
						
						pLink = pLink->pNext->pOpposite;
					}
					while(pLink != pLink0);
				//}



				ppGroupedTriangleListHelper1++;

			}



			//finally update LEVEL2 number segment params
			p2DSegmentLevel2->m_nPts = nPts;
			p2DSegmentLevel2->m_n3DPts = n3DPts;

			//Create LEVEL1 element list for LEVEL2
			pRelList = p2DSegmentLevel2->m_RelList + pTriangleSetLevel2->m_iRelList[RVLRELLIST_COMPONENTS];
			
			pRelList->pFirst = (unsigned char *)ppGroupedTriangleStart;
			pRelList->pEnd = (unsigned char *)ppGroupedTriangleList;

			//relate LEVEL2 segment and LEVEL3 segment
			pp2DSegmentLevel3 = (CRVL2DRegion2 **)(p2DSegmentLevel2->m_pData + pTriangleSetLevel2->m_iDataParentPtr);
			*pp2DSegmentLevel3 = p2DCurrentSegmentLevel3;

			
			//Find the first edge link for each LEVEL2 region
			ppGroupedTriangle = ppGroupedTriangleStart;


			bEdgeFound = 0;

			while(ppGroupedTriangle<ppGroupedTriangleList)
			{
				p2DRegion = *ppGroupedTriangle;

				pLink = pLink0 = (RVLMESH_LINK *)(p2DRegion->m_PtArray);

				bEdgeFound = 0;
			
				do
				{
					if((pLink->Flags & RVLMESH_LINK_FLAG_EDGE)!= 0) //edge found
					{
						//mark the first edge and exit.			
						bEdgeFound = 1;
						p2DSegmentLevel2->m_PtArray = pLink;
						break;
					
					}

					pLink = pLink->pNext->pOpposite;

				}
				while(pLink != pLink0);

				if (bEdgeFound == 1)
					break;
				//{
					//mark the first edge and exit.			
					//p2DSegmentLevel2->m_PtArray = pLink;
					//break;
				//}
				
				ppGroupedTriangle++;
			}
			
			
		}
		
		
		pRelList = p2DCurrentSegmentLevel3->m_RelList + pTriangleSetLevel3->m_iRelList[RVLRELLIST_COMPONENTS];
		pRelList->pFirst = (unsigned char *)ppGroupedLEVEL2Start;
		pRelList->pEnd = (unsigned char *)ppGroupedLEVEL2List;

	}






	
	//exit elegantly
	delete[] GroupedTriangleListHelper;

	return nLevel2Segments;
}




void CRVLPlanarSurfaceDetector::CreateParamList(CRVLMem *pMem)
{
	m_ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	m_ParamList.Init();

	pParamData = m_ParamList.AddParam("PSD.SegmentationType", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "3D", RVLPSD_SEGMENT_3D);
	m_ParamList.AddID(pParamData, "REGION", RVLPSD_SEGMENT_REGIONBASED);
	m_ParamList.AddID(pParamData, "EDGE", RVLPSD_SEGMENT_REGIONBASED);
	m_ParamList.AddID(pParamData, "STRM", RVLPSD_SEGMENT_STRM);
	m_ParamList.AddID(pParamData, "WER", RVLPSD_MESH_SEGMENT_WER);
	m_ParamList.AddID(pParamData, "PLANAR", RVLPSD_MESH_SEGMENT_PLANAR);

	pParamData = m_ParamList.AddParam("PSD.Configuration", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "INTERNAL_FLAGS", RVLPSD_FLAG_INTERNAL);

	pParamData = m_ParamList.AddParam("PSD.Space", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "MM", RVLPSD_FLAG_MM);

	pParamData = m_ParamList.AddParam("PSD.GroundPlaneDetection", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "1", RVLPSD_FLAG_GROUND);

	pParamData = m_ParamList.AddParam("PSD.STRM.uvdTol", RVLPARAM_TYPE_INT, &m_uvdTol);

	pParamData = m_ParamList.AddParam("PSD.STRM.MeshTol", RVLPARAM_TYPE_INT, &m_MeshTol);

	pParamData = m_ParamList.AddParam("PSD.STRM.MeshDiscontinuityThr", RVLPARAM_TYPE_DOUBLE, &m_MeshDiscontinuityThr);

	pParamData = m_ParamList.AddParam("PSD.STRM.MinTriangleFillPerc", RVLPARAM_TYPE_INT, &m_fillPerc);

	pParamData = m_ParamList.AddParam("PSD.MinConvexSegmentSize", RVLPARAM_TYPE_INT, &m_MinConvexSegmentSize);

	pParamData = m_ParamList.AddParam("PSD.Sample.minSampleSize", RVLPARAM_TYPE_INT, &m_minSampleSize);

	pParamData = m_ParamList.AddParam("PSD.Sample.maxSampleSize", RVLPARAM_TYPE_INT, &m_maxSampleSize);

	pParamData = m_ParamList.AddParam("PSD.MeshPlanarSegWERThr1", RVLPARAM_TYPE_INT, &m_MeshPlanarSegWERThr1);

	pParamData = m_ParamList.AddParam("PSD.MeshPlanarSegWERThr2", RVLPARAM_TYPE_INT, &m_MeshPlanarSegWERThr2);

	pParamData = m_ParamList.AddParam("PSD.PointMeasurementUncertStD", RVLPARAM_TYPE_DOUBLE, &m_PointMeasurementUncertStD);

	pParamData = m_ParamList.AddParam("PSD.ConvexMesh", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSD_MESH_CONVEX);
}



BOOL CRVLPlanarSurfaceDetector::MeshSegmentWERNodeWriteToFile(FILE *fp,
															  RVLSWER_NODE *pNode,
															  int maxnLinks)
{
	fprintf(fp, "NODE%0x:\n", pNode);

	fprintf(fp, "Children: %0x, %0x\n", pNode->pChild[0], pNode->pChild[1]);

	RVLQLIST_PTR_ENTRY *pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pNode->LinkPtrList.pFirst);

	int WDCounter = 0;

	RVLSWER_LINK *pLink;
	RVLSWER_NODE *pNode2;	

	while(pLinkPtr)
	{
		pLink = (RVLSWER_LINK *)(pLinkPtr->Ptr);

		pNode2 = pLink->pNode[(pLink->pNode[0] == pNode ? 1 : 0)];

		fprintf(fp, "LINK%0x to NODE%0x\n", pLink, pNode2);

		if(pLinkPtr->pNext == NULL)
			if(&(pLinkPtr->pNext) != pNode->LinkPtrList.ppNext)
			{
				fprintf(fp, "ERROR: False LinkPtrList.ppNext!\n\n");

				return FALSE;
			}

		pLinkPtr = (RVLQLIST_PTR_ENTRY *)(pLinkPtr->pNext);

		WDCounter++;

		if(WDCounter > maxnLinks)
		{
			fprintf(fp, "ERROR: WD STOP!\n\n");

			return FALSE;
		}
	}

	fprintf(fp, "\n");

	return TRUE;
}

// Data structures used in MeshSegmentWER() are explained in SEG_11_2_data_structure.vsd

//#define RVLPSD_MESH_SEGMENT_WER_LOG_FILE

void CRVLPlanarSurfaceDetector::MeshSegmentWER(	CRVLC2D *p2DRegionSet,
												void RVLSWEROnCreateNewNode(RVLSWER_NODE *pNode,
																			RVLSWER_LINK *pLink),
												void RVLSWERUpdateLink(	RVLSWER_NODE *pNode1,
																		RVLSWER_NODE *pNode2,
																		RVLSWER_LINK *pLink),
												int RVLSWERGetCost(	BYTE *pData,
																	int maxCost,
																	double k)
)
{
#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
	FILE *fpLog;

	fopen_s(&fpLog, "C:\\RVL\\ExpRez\\PSDMeshSegmentWER.log", "w");
#endif

	CRVLMPtrChain *pTriangleList = &(p2DRegionSet->m_ObjectList);

	double CostNrm = (double)m_MeshSegmentWERMaxCost;

	// allocate memory and initialize arrays

	int nTriangles = pTriangleList->m_nElements;

	int maxnLinks = 3 * nTriangles / 2 + 1;

	RVLSWER_LINK *LinkMem = new RVLSWER_LINK[maxnLinks];

	RVLSWER_LINK *pLink = LinkMem;

	RVL3DMOMENTS *LinkDataMem = new RVL3DMOMENTS[maxnLinks];

	RVL3DMOMENTS *pLinkData = LinkDataMem;

	RVLSWER_NODE *NodeMem = (RVLSWER_NODE *)(m_pMem->Alloc(
		(2 * nTriangles - 1) * sizeof(RVLSWER_NODE)));

	m_MeshSegmentWERNodeArray = NodeMem;

	RVLSWER_NODE *pNode = NodeMem;

	RVL3DMOMENTS *NodeDataMem = (RVL3DMOMENTS *)(m_pMem->Alloc(
		(2 * nTriangles - 1) * sizeof(RVL3DMOMENTS)));

	RVL3DMOMENTS *pNodeData = NodeDataMem;

	RVLQLIST_PTR_ENTRY *LinkPtrMem = new RVLQLIST_PTR_ENTRY[3 * nTriangles];

	RVLQLIST_PTR_ENTRY *pLinkPtr = LinkPtrMem;

	CRVLQListArray Queue;

	Queue.m_Size = m_MeshSegmentWERMaxCost + 1;

	Queue.m_ListArray = new RVLQLIST[Queue.m_Size];

	Queue.InitListArray(Queue.m_ListArray, Queue.m_ListArray);

	RVLQLIST *QueueListArray = Queue.m_ListArray;

	// create nodes from triangles

	int iDataNodePtr = p2DRegionSet->m_iDataNodePtr;

	CRVL2DRegion2 *pTriangle;
	RVLMESH_LINK *pMeshLink, *pMeshLink0;
	RVLQLIST *pLinkPtrList;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	double fu, fv, fd;
	double Su, Sv, Sd, Suu, Suv, Sud, Svv, Svd, Sdd;
	double *S, *S2;
	double *X;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		*((CRVL2DRegion2 **)(pTriangle->m_pData + p2DRegionSet->m_iDataGrandParentPtr)) = NULL;

		*((RVLSWER_NODE **)(pTriangle->m_pData + iDataNodePtr)) = pNode;

		pNode->Flags = 0x00000000;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		pNode->pData = (BYTE *)pNodeData;

		pNodeData->n = pTriangle->m_n3DPts;

		if(m_Flags & RVLPSD_FLAG_MM)
		{
			S = pNodeData->S;

			RVLNULL3VECTOR(S)

			S2 = pNodeData->S2;

			RVLNULLMX3X3(S2)

			for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
			{
				pPt = *ppPt;

				X = pPt->XYZ;

				RVLSUM3VECTORS(S, X, S)

				RVLMXEL(S2, 3, 0, 0) += (X[0] * X[0]);
				RVLMXEL(S2, 3, 0, 1) += (X[0] * X[1]);
				RVLMXEL(S2, 3, 0, 2) += (X[0] * X[2]);
				RVLMXEL(S2, 3, 1, 1) += (X[1] * X[1]);
				RVLMXEL(S2, 3, 1, 2) += (X[1] * X[2]);
				RVLMXEL(S2, 3, 2, 2) += (X[2] * X[2]);
			}

			RVLCOMPLETESIMMX3(S2)
		}
		else
		{
			Su = Sv = Sd = Suu = Suv = Sud = Svv = Svd = Sdd = 0.0;

			for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
			{
				pPt = *ppPt;

				fu = (double)(pPt->u);
				fv = (double)(pPt->v);
				fd = (double)(pPt->d);

				Su += fu;
				Sv += fv;
				Sd += fd;
				Suu += (fu * fu);
				Suv += (fu * fv);
				Sud += (fu * fd);
				Svv += (fv * fv);
				Svd += (fv * fd);
				Sdd += (fd * fd);
			}

			S = pNodeData->S;

			S[0] = Su;
			S[1] = Sv;
			S[2] = Sd;

			S = pNodeData->S2;

			S[0] = Suu;
			S[1] = Suv;
			S[2] = Sud;
			S[3] = Suv;
			S[4] = Svv;
			S[5] = Svd;
			S[6] = Sud;
			S[7] = Svd;
			S[8] = Sdd;

			//if(S[8] - pNode->Moments.S[2] * pNode->Moments.S[2] / (double)(pNode->Moments.n) < 0.0)
			//	int debug = 0;

			//if((int)pNode == 0x036a2954)
			//	int debug = 0;
		}

		pNode->pChild[0] = (RVLSWER_NODE *)pTriangle;

		pNode->pChild[1] = NULL;

		pNode->pParent = NULL;

		pNode->Cost = RVLSWERGetCost(pNode->pData, m_MeshSegmentWERMaxCost, m_MeshSegmentWERk);

		pLinkPtrList = &(pNode->LinkPtrList);

		RVLQLIST_INIT(pLinkPtrList);

		pNode++;

		pNodeData++;
	}
	
	RVLSWER_NODE *pNewNode = pNode;

	int nLeaveNodes = pNewNode - NodeMem;

	int Cost = m_MeshSegmentWERMaxCost;

	// create links between nodes and put them into Queue

	double StartTime = m_pTimer->GetTime();

	CRVL2DRegion2 *pTriangle2;
	RVLSWER_NODE *pNode2;
	RVLQLIST *pLinkPtrList2;

	for(pNode = NodeMem; pNode < pNewNode; pNode++)
	{
		pLinkPtrList = &(pNode->LinkPtrList);

		pTriangle = (CRVL2DRegion2 *)(pNode->pChild[0]);

		pMeshLink0 = pMeshLink = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		do
		{
			pTriangle2 = (CRVL2DRegion2 *)(pMeshLink->pOpposite->vp2DRegion);

			if(pTriangle2)
				if((pTriangle2->m_Flags & RVLOBJ2_FLAG_REJECTED) == 0)
				{
					pNode2 = *((RVLSWER_NODE **)(pTriangle2->m_pData + iDataNodePtr));

					if(pNode2 > pNode)
					{
						RVLQLIST_ADD_ENTRY(pLinkPtrList, pLinkPtr);

						pLinkPtr->Ptr = pLink;

						pLinkPtr++;

						pLinkPtrList2 = &(pNode2->LinkPtrList);

						RVLQLIST_ADD_ENTRY(pLinkPtrList2, pLinkPtr);

						pLinkPtr->Ptr = pLink;

						pLinkPtr++;

						pLink->pNode[0] = pNode;
						pLink->pNode[1] = pNode2;

						pLink->Cost = -1;

						pLink->pData = (BYTE *)pLinkData;

						RVLSWERUpdateQueue(pLink, NULL, m_MeshSegmentWERMaxCost, m_MeshSegmentWERk,
							RVLMeshSegmentSWERUpdateLink, RVLSWERGetCost, QueueListArray, Cost);

						pLink++;

						pLinkData++;
					}
				}
			
			pMeshLink = pMeshLink->pNext->pOpposite;
		}
		while(pMeshLink != pMeshLink0);
	}

	int minCost = Cost + 1;

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
	fprintf(fpLog, "Nodes:\n");

	for(pNode = NodeMem; pNode < pNewNode; pNode++)
		MeshSegmentWERNodeWriteToFile(fpLog, pNode, maxnLinks);

	fprintf(fpLog, "\nQueue:\n");

	int nEntriesInQueueList;

	for(Cost = minCost; Cost <= maxCost; Cost++)
	{
		fprintf(fpLog, "Queue%d: ", Cost);

		RVLQLIST_WRITE_TO_FILE((QueueListArray + Cost), RVLSWER_LINK, fpLog, nEntriesInQueueList, maxnLinks);

		fprintf(fpLog, "Total %d Entries\n", nEntriesInQueueList);
	}

	fprintf(fpLog, "\n");

	fprintf(fpLog, "WER:\n");

	fclose(fpLog);

	BOOL bOK = TRUE;
#endif

	// build tree by a WER procedure

	m_nMeshSegmentWERLevels = RVLSegmentationWER(RVLSWEROnCreateNewNode,
		RVLSWERUpdateLink, 
		RVLSWERGetCost,
		pNewNode,
		(BYTE *)pNodeData,
		sizeof(RVL3DMOMENTS),
		QueueListArray,
		maxnLinks,
		minCost,
		m_MeshSegmentWERMaxCost,
		m_MeshSegmentWERk);

	// only for debugging purposes !!!

	//for(Cost = 0; Cost < maxCost; Cost++)
	//	if(QueueListArray[Cost].pFirst != NULL)
	//		int debug = 0;

	m_nMeshSegmentWERNodes = m_nMeshSegmentWERLevels + nLeaveNodes;

	// deallocate memory

	delete[] LinkMem;	
	delete[] LinkDataMem;
	delete[] LinkPtrMem;

	double ExecutionTime = m_pTimer->GetTime() - StartTime;

#ifdef RVLPSD_MESH_SEGMENT_WER_LOG_FILE
	fclose(fpLog);
#endif

	// detect objects

	int nObjects = 0;

	RVLSWER_NODE *pNodeArrayEnd = m_MeshSegmentWERNodeArray + 
		m_nMeshSegmentWERNodes;

	for(pNode = m_MeshSegmentWERNodeArray; pNode < pNodeArrayEnd; pNode++)
		if(pNode->pParent)
		{
			if(pNode->pParent->Cost - pNode->Cost >= m_MeshPlanarSegWERThr1 && pNode->pParent->Cost > m_MeshPlanarSegWERThr2)
			{
				pNode->Flags |= RVLSWER_NODE_FLAG_OBJECT;

				nObjects++;
			}
		}
		else
		{
			pNode->Flags |= RVLSWER_NODE_FLAG_OBJECT;

			nObjects++;
		}
}




RVLSWER_NODE *CRVLPlanarSurfaceDetector::MeshSegmentWERSelectObject(	
	CRVL2DRegion2 *pTriangle,
	int &Level,
	DWORD Flags)
{
	if(pTriangle == NULL)
		return NULL;

	RVLSWER_NODE *pNode;
	RVLSWER_NODE *pNodeArrayEnd = 
		m_MeshSegmentWERNodeArray + m_nMeshSegmentWERNodes - m_nMeshSegmentWERLevels;

	for(pNode = m_MeshSegmentWERNodeArray; pNode < pNodeArrayEnd; pNode++)
		if(pNode->pChild[0] == (RVLSWER_NODE *)pTriangle)
			break;

	if(pNode == pNodeArrayEnd)
		return NULL;

	int i;

	for(i = 1; i <= Level; i++)
	{
		if(pNode->pParent == NULL)
			return pNode;

		pNode = pNode->pParent;
	}

	if(Flags & RVLMESHSEGMENT_SELECT_OBJECT_FLAG_OBJECT)
	{
		while((pNode->Flags & RVLSWER_NODE_FLAG_OBJECT) == 0)
		{
			if(pNode->pParent == NULL)
				return pNode;

			pNode = pNode->pParent;

			Level++;
		}
	}

	return pNode;
}

void CRVLPlanarSurfaceDetector::MeshSegmentWERSetLabels(CRVLMPtrChain *pTriangleList,
														int Level)
{
	RVL2DRegionResetLabels(pTriangleList);

	RVLSWER_NODE *pTopNode = m_MeshSegmentWERNodeArray + m_nMeshSegmentWERNodes - 1;

	RVLSWER_NODE *pNode = pTopNode;

	int Label = 0;

	int i, iChild;

	for(i = 0; i <= Level; i++, pNode--)
	{
		for(iChild = 0; iChild < 2; iChild++)
			if(pTopNode - pNode->pChild[iChild] > Level)
			{
				MeshSegmentWERLabelObject(pTriangleList, pNode->pChild[iChild], Label);
		
				Label++;
			}
	}
}



void CRVLPlanarSurfaceDetector::MeshSegmentWERLabelObject(	CRVLMPtrChain *pTriangleList,
															RVLSWER_NODE *pSelectedNode,
															int Label)
{
	if(pSelectedNode == NULL)
		return;

	RVLSWER_NODE **NodeBuff = new RVLSWER_NODE *[m_nMeshSegmentWERLevels];

	RVLSWER_NODE **ppNode;
	CRVL2DRegion2 *pTriangle;
	RVLSWER_NODE *pNode;

	NodeBuff[0] = pSelectedNode;

	ppNode = NodeBuff;

	do
	{
		pNode = *ppNode;

		if(pNode->pChild[1])
		{
			*ppNode = pNode->pChild[0];
			ppNode++;
			*ppNode = pNode->pChild[1];
		}
		else
		{
			pTriangle = (CRVL2DRegion2 *)(pNode->pChild[0]);

			pTriangle->m_Label = Label;

			ppNode--;
		}
	}
	while(ppNode >= NodeBuff);

	delete[] NodeBuff;
}



//SOON TO BE OBSOLETE
//Get 3D Surface Characteristics/Properties ie centroid, eigen values, Rot matrix, normal, contours
void CRVLPlanarSurfaceDetector::Get3DSurfaceAndContours(CRVL2DRegion2 *p2DRegionLevel3,
				 										CRVL3DSurface2 *p3DSurfaceLevel3,
														CRVLClass *p3DConvexSegmentSet,
														CRVL3DSurface2 **pp3DConvexSegmentArray,
														CRVLC2DContour *p2DContourSet,
														CRVL2DContour **pp2DContourArray,
														RVLIPOINT *pIPointArray,
														RVL3DPOINT2 **ppVertexPtArray,
														int &nConvexSegments,
														int &nTotalVertexPts,
														double *TransformedVertexArray,
														int &nSupportPts)
{

	//A. Determine LEVEL3 surface (normal and rho ) for each LEVEL3 region

	//B. For each LEVEL2 regions of a given LEVEL3 region determine centroid and moments of  create LEVEL2 3D surface
	//0. Find new uvd coordinates of the vertices of polygon by projecting the vertices onto the LEVEL3 plane
	//1. Find 3D points (xyz) coordinates of (uvd) using transformation for kinect for only corners of triangles/regions within LEVEL2.
	//2. For each plane create a coordinate system z' in direction of N of the plane, x' is determined by using any 2 points in the plane
	// and y' is obtained using crossproduct. We then have the rotation matrice R(O wrt L) R[x';y';z']  L_R_O
	//3. Using this rot Mat transform all the edge points of each LEVEL2 polygon to this coordinate system (x'y'z')
	//4. Using only the x' and y' coordinates calulate the Area, centroid and second moment of area formulas in RVLMath.doc
	//5. Using second moments (Ix,Iy,Ixy) create a matrix and then calulate the eigen values and eigen vectors.
	//6. Transform the eigen vectors and centroid back to (xyz/camera) coordinate system using R transpose.
	//7. Determine contours for each LEVEL2 surface
	
	//Iterate through all segment in LEVEL2

	CRVL3DSurface2 **ConvexSegmentArray = pp3DConvexSegmentArray;

	RVLARRAY *pRelList;

	CRVL2DRegion2  **pp2DRegionLevel2, **pp2DRegionLevel2End;  //*p2DRegionLevel2,

	CRVL2DRegion2 *p2DRegion;

	RVLMESH_LINK *pDelaunayEdge, *pDelaunayEdge0, *pDelaunayEdge1;

	RVL3DPOINT2 *p3DPt;
	
	double  zL[3], xL[3], yL[3], xLTemp1[3], xLTemp2[3];
	double xLen, yLen;

	double zF[3],xF[3],yF[3];
	
	double transL[3], p3DxyzPt[3],p3DxyzPt1[3],p3DxyzPt2[3];
	

	double xi, xip, yi, yip, x1, y1, Xc,Yc, A, Ix, Iy, Ixy;
	
	int i,j;

	CRVL3DSurface2 *p3DConvexSurface;

	int nVertices = 0;
	int nRegions = 0;
	//int maxSize = 0;
	int iNewPosition;
	bool bInsert;

	double k = 4.0/PI;
	k *= k;

	double r1, s1, s2;

	
	RVL3DPOINT2 **ppVertexPt, **ppVertexPt0;
	
	
	RVLIPOINT *pIPoint, *pIPointStart;


	CRVL2DContour *p2DContour, **pp2DContour, **pp2DContourPtr;

	double *pTransformedVertexArray;

	int pUVDCenterPt[3], pUVDPt[3], iPlaneWidth, dU, dV;
	double pXYZPt[3];

	double C[9], Ctemp[3];
	

	iPlaneWidth = 64; //width^2

	/*********************************************************************/
	//A. Determine LEVEL3 surface (normal and rho ) for each LEVEL3 region
	/*********************************************************************/
	Get3DPlaneKinect(p2DRegionLevel3,p3DSurfaceLevel3);
	
	//Get xyz (LEVEL3)normal from uvd normal and store as z dir
	for(i=0;i<3;i++)
		zL[i] = p3DSurfaceLevel3->m_N[i];

	//Set initial postion of helper pointers
	ppVertexPt = ppVertexPtArray;
	pIPoint = pIPointArray;
	pp2DContour = pp2DContourArray;

	/****************************************************************************************************************/
	//B. For each LEVEL2 region of the given LEVEL3 region determine centroid and second moments of area and create LEVEL2 3D surface
	/****************************************************************************************************************/
	

	//get LEVEL2 segments from rellist
	pRelList = p2DRegionLevel3->m_RelList + p2DRegionLevel3->m_pClass->m_iRelListComponents;
	pp2DRegionLevel2 = (CRVL2DRegion2 **)pRelList->pFirst;
	pp2DRegionLevel2End = (CRVL2DRegion2 **)pRelList->pEnd;

	nTotalVertexPts = 0;
	nConvexSegments = 0;
	nSupportPts = 0;

	p3DSurfaceLevel3->m_Area = 0.0;

	//for each LEVEL2
	for(; pp2DRegionLevel2 < pp2DRegionLevel2End; pp2DRegionLevel2++)
	{
		p2DRegion = *pp2DRegionLevel2;


		//Get first edge
		pDelaunayEdge = pDelaunayEdge0 = (RVLMESH_LINK *)(p2DRegion->m_PtArray);

		//reset counter		
		nVertices = 0;

		//point to initial position
		ppVertexPt0 = ppVertexPt;
		pIPointStart = pIPoint;

		pTransformedVertexArray = TransformedVertexArray;

		Xc=0;
		Yc=0; 
		A=0; 
		Ix=0; 
		Iy=0; 
		Ixy=0;

		//Go through all edges
		do
		{
			p3DPt = m_Point3DMap[pDelaunayEdge->iPix0];  //!!!!

			//pUVDPt[0] = p3DPt->u;
			//pUVDPt[1] = p3DPt->v;
			//pUVDPt[2] = p3DPt->d;

			for(i=0;i<3;i++)
				p3DxyzPt[i] = p3DPt->XYZ[i];
			

			//Store vertex point
			*(ppVertexPt++) = p3DPt;

			//Store Ipoint 
			pIPoint->u = p3DPt->u;
			pIPoint->v = p3DPt->v;
			pIPoint++;

			nVertices++;
			nTotalVertexPts++;
			
			if(nVertices==1) //Get 1st point
			{
				for(i=0;i<3;i++)
				{
					p3DxyzPt1[i] = p3DxyzPt[i];
					transL[i] = p3DxyzPt[i];
				}
			}
			else if(nVertices==2) //Get 2nd point
			{
				//create xLtemp
				for(i=0;i<3;i++)
				{
					p3DxyzPt2[i] = p3DxyzPt[i];
					xLTemp1[i] = (p3DxyzPt2[i] - p3DxyzPt1[i]);
				}

				//Find the projection of xLTemp1 on the plane
				CrossProduct(zL,xLTemp1,xLTemp2);
				CrossProduct(xLTemp2,zL,xL);

				xLen = sqrt(xL[0]*xL[0] + xL[1]*xL[1] + xL[2]*xL[2]);

				//create unit vector xL
				for(i=0;i<3;i++)
					xL[i] = xL[i]/xLen;

				//create yL
				CrossProduct(zL,xL,yL);

				yLen = sqrt(yL[0]*yL[0] + yL[1]*yL[1] + yL[2]*yL[2]);

				for(i=0;i<3;i++)
					yL[i] = yL[i]/yLen;

				
				//create rotation matrix  (L_R_O or trans(O_R_L))
				for(i=0;i<3;i++)
				{
					rotMat[0 * 3 + i] = xL[i];
					rotMat[1 * 3 + i] = yL[i];
					rotMat[2 * 3 + i] = zL[i];
				}

				//transform 1st point and store
				RVLDif3D(p3DxyzPt1,transL,PL);
				cvGEMM(dMatRot,dMatPL,1,NULL,1,dMatPL1);
				
				pTransformedVertexArray[0] = PL1[0];
				pTransformedVertexArray[1] = PL1[1];
				pTransformedVertexArray += 2;
				
				xi = PL1[0];
				yi = PL1[1];

				x1 = xi;
				y1 = yi;


				//transform 2nd point and store
				RVLDif3D(p3DxyzPt2,transL,PL);
				cvGEMM(dMatRot,dMatPL,1,NULL,1,dMatPL1);
				
				pTransformedVertexArray[0] = PL1[0];
				pTransformedVertexArray[1] = PL1[1];
				pTransformedVertexArray += 2;

				//update polygon 
				xip = PL1[0];
				yip = PL1[1];

				RVLUpdate2DPolygonAreaAndCentroid(xi,yi,xip,yip,A,Xc,Yc);

				xi = xip;
				yi = yip;
				
			}
			else if(nVertices>2)
			{
				//transform point and store
				RVLDif3D(p3DxyzPt,transL,PL);
				cvGEMM(dMatRot,dMatPL,1,NULL,1,dMatPL1);
				
				pTransformedVertexArray[0] = PL1[0];
				pTransformedVertexArray[1] = PL1[1];
				pTransformedVertexArray += 2;

				//update polygon
				xip = PL1[0];
				yip = PL1[1];

				RVLUpdate2DPolygonAreaAndCentroid(xi,yi,xip,yip,A,Xc,Yc);

				xi = xip;
				yi = yip;
			}

			//find next edge
			pDelaunayEdge1 = pDelaunayEdge;
			do
			{
				pDelaunayEdge = pDelaunayEdge->pNext;
			}
			while(!(((pDelaunayEdge->Flags & RVLMESH_LINK_FLAG_EDGE) != 0) && (pDelaunayEdge != pDelaunayEdge1)));

			pDelaunayEdge = pDelaunayEdge->pOpposite;

		}
		while(pDelaunayEdge != pDelaunayEdge0);

		p2DRegion->m_nVertices = nVertices; 
		p2DRegion->m_pVertexArray = ppVertexPt0;


		//final update polygon
		xip = x1;
		yip = y1;
		RVLUpdate2DPolygonAreaAndCentroid(xi,yi,xip,yip,A,Xc,Yc);

		//Area
		A = A/2;

		if(A == 0.0)
		{
			p2DRegion->m_vp3DSurface = NULL;

			continue;
		}

		

		//Centroid
		Xc = Xc/(6*A);
		Yc = Yc/(6*A);


		//Go through all saved points to determine moments
		for(i=0;i<nVertices;i++)
		{
			xi = TransformedVertexArray[i*2 + 0] - Xc;
			yi = TransformedVertexArray[i*2 + 1] - Yc;
		
			if(i==nVertices-1)
			{
				xip = TransformedVertexArray[0] - Xc;
				yip = TransformedVertexArray[1] - Yc;
			}
			else
			{
				xip = TransformedVertexArray[(i+1)*2 + 0] - Xc;
				yip = TransformedVertexArray[(i+1)*2 + 1] - Yc;
			}

			RVLUpdate2DPolygonMoments(xi,yi,xip,yip, Ix, Iy, Ixy);
		}

	

		//Second moment of Area
		Ix = Ix/12;
		Iy = Iy/12;
		Ixy = Ixy/24;

		//Get eigen values of second moment of Area 
		sqrMat[0] = Iy;
		sqrMat[1] = Ixy;
		sqrMat[2] = Ixy;
		sqrMat[3] = Ix;

		//The eigenvectors are perpendicular because dMatSqr = transpose(dMatSqr)
		cvEigenVV(dMatSqr, dMatEig, dMatEigVal);
		
		//Largest eigen value corresponds to first eigen vector (eigen values sorted in descending order)(eigen vectors stored in subsequent rows)
		xF[0] = eigMat[0];
		xF[1] = eigMat[1];
		xF[2] = 0;

		yF[0] = eigMat[2];
		yF[1] = eigMat[3];
		yF[2] = 0;

		zF[0] = 0;
		zF[1] = 0;
		zF[2] = 1;
		
		
		//transform centroid
		PL[0] = Xc;
		PL[1] = Yc;
		PL[2] = 0;

		cvGEMM(dMatRot,dMatPL,1,NULL,1,dMatPL1,CV_GEMM_A_T);

		//add transformed centroid to initial translation vector to get final translation vector PL
		RVLSum3D(PL1,transL,PL);
		
		for(i=0;i<3;i++)
		{
			//rotation matrix L_R_F
			rotMatF[i * 3 + 0] = xF[i];
			rotMatF[i * 3 + 1] = yF[i];
			rotMatF[i * 3 + 2] = zF[i];
		}

		
		cvGEMM(dMatRot,dMatRotF,1,NULL,1,dMatRotFinal,CV_GEMM_A_T);

		//store eigen values
		s1 = eigVal[0];
        s2 = eigVal[1];
		//if(s1*s2<=0)
		//	//int gg = 0;  //debug for one eigen value positive and one negative!!!
		//{
		//	
		//	p2DRegion->m_vp3DSurface = NULL;

		//	continue;
		//}
			

		s1 = abs(s1);
		s2 = abs(s2);
        r1 = pow(k * s1 * s1 * s1 / s2, 0.125);		
		eigVal[0] = r1;
		eigVal[1] = r1 * sqrt(s2 / s1);

		

		//*********check whether the plane is wide enough****************//
		RVLGetKinect2DData(pUVDCenterPt,PL, m_pStereoVision->m_KinectParams);

		for(i=0;i<3;i++)
			pXYZPt[i] = PL[i] + (eigVal[1] * rotMatFinal[3*i + 1]);

		RVLGetKinect2DData(pUVDPt,pXYZPt, m_pStereoVision->m_KinectParams);

		dU = pUVDPt[0] - pUVDCenterPt[0];
		dV = pUVDPt[1] - pUVDCenterPt[1];
		if((dU*dU + dV*dV) <= iPlaneWidth)
		{
			p2DRegion->m_vp3DSurface = NULL;

			continue;
		}

		//Create convex surface and store params
		p3DConvexSurface = (CRVL3DSurface2 *)(RVL3DSurfaceTemplate.Create3(p3DConvexSegmentSet));
		//add to Convex surface array
		//*(pp3DConvexSegmentArray++) = p3DConvexSurface;

		for(i=0;i<3;i++)
		{
			//Store normal 
			p3DConvexSurface->m_N[i] = p3DSurfaceLevel3->m_N[i];
			//Store eigen vectors
			//p3DConvexSurface->m_EigenVectors[i] =  xF[i];
			//p3DConvexSurface->m_EigenVectors[3 + i] =  yF[i];

			//Final translation vector O->L->F
			p3DConvexSurface->m_Pose.m_X[i] =  PL[i];
			
		}

		for(i=0;i<9;i++)
		{
			//Final rotation matrix (O->L->F) O_R_L * L_R_F
			p3DConvexSurface->m_Pose.m_Rot[i] =  rotMatFinal[i];
		}

		//store rho ie m_d
		p3DConvexSurface->m_d = p3DSurfaceLevel3->m_d;
		double md = RVLDOTPRODUCT3(p3DConvexSurface->m_Pose.m_X, p3DConvexSurface->m_N);

		//store eigen values
		p3DConvexSurface->m_EigenValues[0] = eigVal[0];
		p3DConvexSurface->m_EigenValues[1] = eigVal[1];

		
		//copy support and check max size
		p3DConvexSurface->m_nSupport = p2DRegion->m_n3DPts;
		nSupportPts += p2DRegion->m_n3DPts;
		//if (p3DConvexSurface->m_nSupport>maxSize)
		//	maxSize = p3DConvexSurface->m_nSupport;
		
		p3DSurfaceLevel3->m_Area += A;


		//Create 2D contour
		p2DContour = (CRVL2DContour *)(RVL2DContourTemplate.Create3(p2DContourSet));
		p2DContour->m_ContourIPArray = pIPointStart;
		p2DContour->m_nContourIPs = pIPoint - pIPointStart;
		//Add to contour array
		*(pp2DContour++) = p2DContour;

		//Relate 2D contour to 3D surface
		pp2DContourPtr = (CRVL2DContour **)(p3DConvexSurface->m_pData + p3DConvexSegmentSet->m_iDataContourPtr);
		*pp2DContourPtr = p2DContour;


		//Calculate uncertainty of xyz centroid using uvd centroid  ???? (Using uvd centroid determined from 3d centroid??!!)
		m_pStereoVision->m_pCameraL->KinectReconWithUncert((double)pUVDCenterPt[0],(double)pUVDCenterPt[1],(double)pUVDCenterPt[2],
															m_pStereoVision->m_KinectParams.d0,
															m_pStereoVision->m_KinectParams.k,
															m_pStereoVision->m_KinectParams.depthUc,
															m_pStereoVision->m_KinectParams.depthVc,
															m_pStereoVision->m_KinectParams.depthFu,
															m_pStereoVision->m_KinectParams.depthFv, 
															m_RuvTol*m_RuvTol, m_RuvdTol*m_RuvdTol, C);

		MatrixMultiplication(C,p3DConvexSurface->m_N,Ctemp,3,3,1);
		p3DConvexSurface->m_sigmaR = RVLDotProduct(p3DConvexSurface->m_N,Ctemp);
		
		//Relate 3D convex surface to LEVEL2 region 
		p2DRegion->m_vp3DSurface = p3DConvexSurface;

		

		////add to Convex surface array ie Insert into appropriate position (Sort procedure)
		if(nRegions == 0)
		{
			//store new value
			ConvexSegmentArray[0] = p3DConvexSurface;
			//SortArray[0].iIndex = nRegions;
		}
		else
		{
			bInsert = false;
			for(i=0;i<nRegions;i++)
			{
				if(p3DConvexSurface->m_nSupport > ConvexSegmentArray[i]->m_nSupport)
				{
					iNewPosition = i;
					//Shift all values down first
					for(j=nRegions;j>i;j--)
						ConvexSegmentArray[j] = ConvexSegmentArray[j-1];

					bInsert = true;

					break;
				}
				
			}

			if(bInsert==false)
				iNewPosition = nRegions;

			//store new value
			ConvexSegmentArray[iNewPosition] = p3DConvexSurface;
			//SortArray[iNewPosition].iIndex = nRegions;
		}

		nRegions++;

		

		
		//*(pp3DConvexSegmentArray++) = p3DConvexSurface;
		
	} 

	//nConvexSegments = pp3DConvexSegmentArray - ConvexSegmentArray;
	nConvexSegments = nRegions;
	
}


//Get 3D Surface Characteristics/Properties ie centroid, eigen values, Rot matrix, normal, contours
void CRVLPlanarSurfaceDetector::Get3DSurfaceAndContours2(CRVL2DRegion2 *p2DRegionLevel3,
				 										CRVL3DSurface2 *p3DSurfaceLevel3,
														CRVLClass *p3DConvexSegmentSet,
														CRVL3DSurface2 **pp3DConvexSegmentArray,
														CRVLC2DContour *p2DContourSet,
														CRVL2DContour **pp2DContourArray,
														RVLIPOINT *pIPointArray,
														RVL3DPOINT2 **ppVertexPtArray,
														int &nConvexSegments,
														int &nTotalVertexPts,
														double *TransformedVertexArray,
														int &nSupportPts)
{
	double fu = m_pStereoVision->m_KinectParams.depthFu;
	double fv = m_pStereoVision->m_KinectParams.depthFv;
	double uc = m_pStereoVision->m_KinectParams.depthUc;
	double vc = m_pStereoVision->m_KinectParams.depthVc;
	double k_ = m_pStereoVision->m_KinectParams.k;
	double d0 = m_pStereoVision->m_KinectParams.d0;

	//A. Determine LEVEL3 surface (normal and rho ) for each LEVEL3 region

	//B. For each LEVEL2 regions of a given LEVEL3 region determine centroid and moments of  create LEVEL2 3D surface
	//0. Find new uvd coordinates of the vertices of polygon by projecting the vertices onto the LEVEL3 plane
	//1. Find 3D points (xyz) coordinates of (uvd) using transformation for kinect for only corners of triangles/regions within LEVEL2.
	//2. For each plane create a coordinate system z' in direction of N of the plane, x' is determined by using any 2 points in the plane
	// and y' is obtained using crossproduct. We then have the rotation matrice R(O wrt L) R[x';y';z']  L_R_O
	//3. Using this rot Mat transform all the edge points of each LEVEL2 polygon to this coordinate system (x'y'z')
	//4. Using only the x' and y' coordinates calulate the Area, centroid and second moment of area formulas in RVLMath.doc
	//5. Using second moments (Ix,Iy,Ixy) create a matrix and then calulate the eigen values and eigen vectors.
	//6. Transform the eigen vectors and centroid back to (xyz/camera) coordinate system using R transpose.
	//7. Determine contours for each LEVEL2 surface
	
	//Iterate through all segment in LEVEL2

	CRVL3DSurface2 **ConvexSegmentArray = pp3DConvexSegmentArray;

	RVLARRAY *pRelList;

	CRVL2DRegion2  **pp2DRegionLevel2, **pp2DRegionLevel2End;  //*p2DRegionLevel2,

	CRVL2DRegion2 *p2DRegion;

	RVLMESH_LINK *pDelaunayEdge, *pDelaunayEdge0, *pDelaunayEdge1;

	RVL3DPOINT2 *p3DPt;
	
	double  zL[3], xL[3], yL[3], xLTemp1[3];//, xLTemp2[3];
	double xLen, yLen;

	double zF[3],xF[3],yF[3];
	
	double  p3DxyzPt[3]; //transL[3],,p3DxyzPt1[3],p3DxyzPt2[3]
	

	double xi, xip, yi, yip, x1, y1, Xc,Yc, A, Ix, Iy, Ixy;
	
	int i,j;

	CRVL3DSurface2 *p3DConvexSurface;

	int nVertices = 0;
	int nRegions = 0;
	//int maxSize = 0;
	int iNewPosition;
	bool bInsert;

	double k = 4.0/PI;
	k *= k;

	double r1, r2, s1, s2;

	
	RVL3DPOINT2 **ppVertexPt, **ppVertexPt0;
	
	
	RVLIPOINT *pIPoint, *pIPointStart;


	CRVL2DContour *p2DContour, **pp2DContour, **pp2DContourPtr;

	double *pTransformedVertexArray;

	int pUVDCenterPt[3], pUVDPt[3], iPlaneWidth, dU, dV;
	double pXYZPt[3];

	double Ctemp[3];
	

	iPlaneWidth = m_MinConvexSegmentSize * m_MinConvexSegmentSize; //width^2

	/*********************************************************************/
	//A. Determine LEVEL3 surface (normal and rho ) for each LEVEL3 region
	/*********************************************************************/

	pRelList = p2DRegionLevel3->m_RelList + p2DRegionLevel3->m_pClass->m_iRelListElements;

	CRVL2DRegion2 **ppTriangleArrayEnd = (CRVL2DRegion2 **)(pRelList->pEnd);

	int n3DPts = 0;

	RVL3DMOMENTS Moments;

	double *S = Moments.S;
	double *S2 = Moments.S2;

	RVLNULL3VECTOR(S)
	RVLNULLMX3X3(S2)

	CRVL2DRegion2 **ppTriangle;
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **pp3DPt;
	RVL3DPOINT2 **p3DPtArrayEnd;
	double *X;
	double x, y, z;

	for(ppTriangle = (CRVL2DRegion2 **)(pRelList->pFirst); ppTriangle < ppTriangleArrayEnd; ppTriangle++)
	{
		pTriangle = *ppTriangle;

		pp3DPt = (RVL3DPOINT2 **)(pTriangle->m_pPoint3DArray);
		
		p3DPtArrayEnd = pp3DPt + pTriangle->m_n3DPts;

		for(; pp3DPt < p3DPtArrayEnd; pp3DPt++)
		{
			p3DPt = *pp3DPt;

#ifdef RVLPSD_GET3DSURFACES_UVD
			x = p3DPt->x;
			y = p3DPt->y;
			z = p3DPt->z;
#else
			X = p3DPt->XYZ;

			x = X[0];
			y = X[1];
			z = X[2];
#endif
			
			S[0] += x;
			S[1] += y;
			S[2] += z;
			S2[0] += x * x;
			S2[1] += x * y;
			S2[2] += x * z;
			S2[4] += y * y;
			S2[5] += y * z;
			S2[8] += z * z;
		}

		n3DPts += pTriangle->m_n3DPts;
	}

	Moments.n = n3DPts;

	double *R = p3DSurfaceLevel3->m_Pose.m_Rot;
	double *t = p3DSurfaceLevel3->m_Pose.m_X;
	double *N = p3DSurfaceLevel3->m_N;
	//double *C = p3DSurfaceLevel3->m_Cp;

	double eig[3];
	double VNrm[3 * 3];
	double V[3 * 3];
	double lV[3];
	double fTmp;
	double rho;
	double *pV;
	double Y[3];
	double C[9];

	if(m_Flags & RVLPSD_FLAG_MM)
	{
		RVLGetCovMatrix3(&Moments, C, t);

		RVLGetAxesOfCov3D(C, eig, VNrm, V, lV);

		fTmp = sqrt(RVLDOTPRODUCT3(VNrm, VNrm));
		RVLSCALE3VECTOR2(VNrm, fTmp, N)

		rho = RVLDOTPRODUCT3(t, N);

		if(rho < 0.0)
		{
			N[0] = -N[0];
			N[1] = -N[1];
			N[2] = -N[2];

			rho = -rho;
		}

		p3DSurfaceLevel3->m_d = rho;

		RVLCOPYTOCOL3(N, 2, R);

		X = VNrm + 6;

		RVLNORM3(X, fTmp)		

		RVLCOPYTOCOL3(X, 0, R);

		RVLCROSSPRODUCT3(N, X, Y);

		RVLCOPYTOCOL3(Y, 1, R);
	}
	else
	{		
		double q[3];

		RVLGetCovMatrix3(&Moments, C, q);

		RVLGetAxesOfCov3D(C, eig, VNrm, V, lV);

		if(VNrm[2] < 0.0)
		{
			VNrm[0] = -VNrm[0];
			VNrm[1] = -VNrm[1];
			VNrm[2] = -VNrm[2];
		}

		N[0] = VNrm[0] * fu;
		N[1] = VNrm[1] * fv;
		N[2] = VNrm[0] * (uc - q[0]) + VNrm[1] * (vc - q[1]) + VNrm[2] * (d0 - q[2]);
		fTmp = sqrt(RVLDOTPRODUCT3(N, N));
		RVLSCALE3VECTOR2(N, fTmp, N)

		rho = p3DSurfaceLevel3->m_d = VNrm[2] * k_ / fTmp;
		
		RVLNULL3VECTOR(S)
		RVLNULLMX3X3(S2)

		for(ppTriangle = (CRVL2DRegion2 **)(pRelList->pFirst); ppTriangle < ppTriangleArrayEnd; ppTriangle++)
		{
			pTriangle = *ppTriangle;

			pp3DPt = (RVL3DPOINT2 **)(pTriangle->m_pPoint3DArray);
			
			p3DPtArrayEnd = pp3DPt + pTriangle->m_n3DPts;

			for(; pp3DPt < p3DPtArrayEnd; pp3DPt++)
			{
				p3DPt = *pp3DPt;

				X = p3DPt->XYZ;

				fTmp = RVLDOTPRODUCT3(N, X);

				x = X[0] - fTmp * N[0];
				y = X[1] - fTmp * N[1];
				z = X[2] - fTmp * N[2];
				
				S[0] += x;
				S[1] += y;
				S[2] += z;
				S2[0] += x * x;
				S2[1] += x * y;
				S2[2] += x * z;
				S2[4] += y * y;
				S2[5] += y * z;
				S2[8] += z * z;
			}
		}	

		RVLGetCovMatrix3(&Moments, C, t);

		t[0] += (rho * N[0]);
		t[1] += (rho * N[1]);
		t[2] += (rho * N[2]);

		RVLGetAxesOfCov3D(C, eig, VNrm, V, lV);

		pV = VNrm;

		RVLCOPYTOCOL3(pV, 2, R)

		pV += 3;

		RVLCOPYTOCOL3(pV, 1, R)

		pV += 3;

		RVLCOPYTOCOL3(pV, 0, R)

		if(RVLDOTPRODUCT3(VNrm,N) < 0.0)
		{
			R[0] = -R[0];
			R[2] = -R[2];
			R[3] = -R[3];
			R[5] = -R[5];
			R[6] = -R[6];
			R[8] = -R[8];
		}
	}
	
	r1 = sqrt(eig[2]);
	r2 = sqrt(eig[1]);

	p3DSurfaceLevel3->m_EigenValues[0] = r1;
	p3DSurfaceLevel3->m_EigenValues[1] = r2;

	double sigmaR;

	if (m_Flags & RVLPSD_FLAG_MM)
		//sigmaR = p2DRegionLevel3->m_std * p2DRegionLevel3->m_std;
		sigmaR = m_PointMeasurementUncertStD * m_PointMeasurementUncertStD;
	else
	{
		CRVLCamera *pCamera = m_pStereoVision->m_pCameraL;

		int U[3];

		RVLGetKinect2DData(U, t, m_pStereoVision->m_KinectParams);

		double Cp[9];

		pCamera->KinectReconWithUncert(	(double)(U[0]), (double)(U[1]), (double)(U[2]), d0, k_, uc, vc, fu, fv, 
			m_RuvTol*m_RuvTol, m_RuvdTol*m_RuvdTol, Cp);	

		sigmaR = RVLCOV3DTRANSFTO1D(Cp, N);

		//RVLMULVECT3VECT3T(N, N, Cp)

		//RVLSCALEMX3X3(Cp, sigmaR, Cp)

		//RVLSUMMX3X3(C, Cp, C)
	}

	p3DSurfaceLevel3->m_sigmaR = sigmaR;

	sigmaR *= (p3DSurfaceLevel3->m_pClass->m_UncertCoeff * p3DSurfaceLevel3->m_pClass->m_UncertCoeff);

	p3DSurfaceLevel3->m_varq[0] =  sigmaR / (r1 * r1 + sigmaR);
	p3DSurfaceLevel3->m_varq[1] =  sigmaR / (r2 * r2 + sigmaR);
	p3DSurfaceLevel3->m_varq[2] =  sigmaR;

	nSupportPts = n3DPts;

#ifdef NEVER
	//if(Flags & RVLPSD_MESH_CONVEX) //MODIFY THE FOLLOWING USING THIS FLAG
	//{
	//}

	// Old version

	//Get3DPlaneKinect(p2DRegionLevel3,p3DSurfaceLevel3);
	//
	////Get xyz (LEVEL3)normal from uvd normal and store as z dir
	//RVLCOPY3VECTOR(p3DSurfaceLevel3->m_N,zL);
	

	/****************************************************************************************************************/
	//B. For each LEVEL2 region of the given LEVEL3 region determine centroid and second moments of area and create LEVEL2 3D surface
	/****************************************************************************************************************/

	//Set initial postion of helper pointers
	ppVertexPt = ppVertexPtArray;
	pIPoint = pIPointArray;
	pp2DContour = pp2DContourArray;

	//get LEVEL2 segments from rellist
	pRelList = p2DRegionLevel3->m_RelList + p2DRegionLevel3->m_pClass->m_iRelListComponents;
	pp2DRegionLevel2 = (CRVL2DRegion2 **)pRelList->pFirst;
	pp2DRegionLevel2End = (CRVL2DRegion2 **)pRelList->pEnd;

	nTotalVertexPts = 0;
	nConvexSegments = 0;
	nSupportPts = 0;

	p3DSurfaceLevel3->m_Area = 0.0;

	bool bTransfMatCreated = false;


	//create new cs with z = normal of level 3 plane and x,y chosen arbitrarily
	xLTemp1[0] = 0;
	xLTemp1[1] = 0;
	xLTemp1[2] = 0;

	//find min component of zL
	int iMin;
	double minVal = 1.0;
	for(i=0;i<3;i++)
	{
		if(abs(zL[i])<minVal)
		{
			minVal = abs(zL[i]);
			iMin = i;
		}
	}

	//set xLTemp1
	xLTemp1[iMin] = 1;

	//Get yL 
	//Find the projection of xLTemp1 on the plane ie yL
	CrossProduct(zL,xLTemp1,yL);
	
	//create unit vector yL
	yLen = sqrt(yL[0]*yL[0] + yL[1]*yL[1] + yL[2]*yL[2]);
	for(i=0;i<3;i++)
		yL[i] = yL[i]/yLen;

	//Get xL
	CrossProduct(yL,zL,xL);

	//create unit vector xL
	xLen = sqrt(xL[0]*xL[0] + xL[1]*xL[1] + xL[2]*xL[2]);
	for(i=0;i<3;i++)
		xL[i] = xL[i]/xLen;


	//create rotation matrix  (L_R_O or trans(O_R_L))
	for(i=0;i<3;i++)
	{
		rotMat[0 * 3 + i] = xL[i];
		rotMat[1 * 3 + i] = yL[i];
		rotMat[2 * 3 + i] = zL[i];
	}


	//For each LEVEL2
	for(; pp2DRegionLevel2 < pp2DRegionLevel2End; pp2DRegionLevel2++)
	{
		p2DRegion = *pp2DRegionLevel2;

		//Get first edge
		pDelaunayEdge = pDelaunayEdge0 = (RVLMESH_LINK *)(p2DRegion->m_PtArray);

		//reset counter		
		nVertices = 0;

		//point to initial position
		ppVertexPt0 = ppVertexPt;
		pIPointStart = pIPoint;

		pTransformedVertexArray = TransformedVertexArray;

		Xc=0;
		Yc=0; 
		A=0; 
		Ix=0; 
		Iy=0; 
		Ixy=0;

		//Go through all edges
		do
		{
			p3DPt = m_Point3DMap[pDelaunayEdge->iPix0];  //!!!!
			
			RVLCOPY3VECTOR(p3DPt->XYZ,p3DxyzPt);
			
			//Store vertex point
			*(ppVertexPt++) = p3DPt;

			//Store Ipoint 
			pIPoint->u = p3DPt->u;
			pIPoint->v = p3DPt->v;
			pIPoint++;

			nVertices++;
			
			
			if(nVertices==1) //Get 1st point
			{
				//RVLCOPY3VECTOR(p3DxyzPt,p3DxyzPt1);
				//transform point and store
				RVLCOPY3VECTOR(p3DxyzPt,PL);
				cvGEMM(dMatRot,dMatPL,1,NULL,1,dMatPL1);
				
				pTransformedVertexArray[0] = PL1[0];
				pTransformedVertexArray[1] = PL1[1];
				pTransformedVertexArray += 2;

				xi = PL1[0];
				yi = PL1[1];

				x1 = xi;
				y1 = yi;

			}
			//else if(nVertices==2) //Get 2nd point
			//{
			//	RVLCOPY3VECTOR(p3DxyzPt,p3DxyzPt2);

			//	//transform 1st point and store
			//	RVLCOPY3VECTOR(p3DxyzPt1,PL);
			//	cvGEMM(dMatRot,dMatPL,1,NULL,1,dMatPL1);

			//	pTransformedVertexArray[0] = PL1[0];
			//	pTransformedVertexArray[1] = PL1[1];
			//	pTransformedVertexArray += 2;
			//	
			//	xi = PL1[0];
			//	yi = PL1[1];

			//	x1 = xi;
			//	y1 = yi;

			//	//transform 2nd point and store
			//	RVLCOPY3VECTOR(p3DxyzPt2,PL);
			//	cvGEMM(dMatRot,dMatPL,1,NULL,1,dMatPL1);
			//	
			//	pTransformedVertexArray[0] = PL1[0];
			//	pTransformedVertexArray[1] = PL1[1];
			//	pTransformedVertexArray += 2;

			//	//update polygon 
			//	xip = PL1[0];
			//	yip = PL1[1];

			//	RVLUpdate2DPolygonAreaAndCentroid(xi,yi,xip,yip,A,Xc,Yc);

			//	xi = xip;
			//	yi = yip;
			//	
			//}
			else //if(nVertices>2)
			{
				//transform point and store
				RVLCOPY3VECTOR(p3DxyzPt,PL);
				cvGEMM(dMatRot,dMatPL,1,NULL,1,dMatPL1);
				
				pTransformedVertexArray[0] = PL1[0];
				pTransformedVertexArray[1] = PL1[1];
				pTransformedVertexArray += 2;

				//update polygon
				xip = PL1[0];
				yip = PL1[1];

				RVLUpdate2DPolygonAreaAndCentroid(xi,yi,xip,yip,A,Xc,Yc);

				xi = xip;
				yi = yip;
			}

			//find next edge
			pDelaunayEdge1 = pDelaunayEdge;
			do
			{
				pDelaunayEdge = pDelaunayEdge->pNext;
			}
			while(!(((pDelaunayEdge->Flags & RVLMESH_LINK_FLAG_EDGE) != 0) && (pDelaunayEdge != pDelaunayEdge1)));

			pDelaunayEdge = pDelaunayEdge->pOpposite;

		}
		while(pDelaunayEdge != pDelaunayEdge0);

		


		//final update polygon
		xip = x1;
		yip = y1;
		RVLUpdate2DPolygonAreaAndCentroid(xi,yi,xip,yip,A,Xc,Yc);

		//Area
		A = A/2;

		if(A == 0.0)
		{
			p2DRegion->m_vp3DSurface = NULL;

			//reset initial pointers
			ppVertexPt = ppVertexPt0;
			pIPoint = pIPointStart;

			continue;
		}

		

		//Centroid
		Xc = Xc/(6*A);
		Yc = Yc/(6*A);


		//Go through all saved points to determine moments
		for(i=0;i<nVertices;i++)
		{
			xi = TransformedVertexArray[i*2 + 0] - Xc;
			yi = TransformedVertexArray[i*2 + 1] - Yc;
		
			if(i==nVertices-1)
			{
				xip = TransformedVertexArray[0] - Xc;
				yip = TransformedVertexArray[1] - Yc;
			}
			else
			{
				xip = TransformedVertexArray[(i+1)*2 + 0] - Xc;
				yip = TransformedVertexArray[(i+1)*2 + 1] - Yc;
			}

			RVLUpdate2DPolygonMoments(xi,yi,xip,yip, Ix, Iy, Ixy);
		}

	

		//Second moment of Area
		Ix = Ix/12;
		Iy = Iy/12;
		Ixy = Ixy/24;

		//Get eigen values of second moment of Area 
		sqrMat[0] = Iy;
		sqrMat[1] = Ixy;
		sqrMat[2] = Ixy;
		sqrMat[3] = Ix;

		//The eigenvectors are perpendicular because dMatSqr = transpose(dMatSqr)
		cvEigenVV(dMatSqr, dMatEig, dMatEigVal);
		
		//Largest eigen value corresponds to first eigen vector (eigen values sorted in descending order)(eigen vectors stored in subsequent rows)
		xF[0] = eigMat[0];
		xF[1] = eigMat[1];
		xF[2] = 0;

		yF[0] = eigMat[2];
		yF[1] = eigMat[3];
		yF[2] = 0;

		zF[0] = 0;
		zF[1] = 0;
		zF[2] = 1;
		
		
		//transform centroid
		PL1[0] = Xc;
		PL1[1] = Yc;
		PL1[2] = p3DSurfaceLevel3->m_d;

		//Get final translation vector PL
		cvGEMM(dMatRot,dMatPL1,1,NULL,1,dMatPL,CV_GEMM_A_T);

				
		for(i=0;i<3;i++)
		{
			//rotation matrix L_R_F
			rotMatF[i * 3 + 0] = xF[i];
			rotMatF[i * 3 + 1] = yF[i];
			rotMatF[i * 3 + 2] = zF[i];
		}

		
		cvGEMM(dMatRot,dMatRotF,1,NULL,1,dMatRotFinal,CV_GEMM_A_T);

		//store eigen values
		s1 = eigVal[0];
        s2 = eigVal[1];
		s1 = abs(s1);
		s2 = abs(s2);
        r1 = pow(k * s1 * s1 * s1 / s2, 0.125);		
		eigVal[0] = r1;
		eigVal[1] = r1 * sqrt(s2 / s1);

		

		//*********check whether the plane is wide enough****************//
		RVLGetKinect2DData(pUVDCenterPt,PL, m_pStereoVision->m_KinectParams);

		for(i=0;i<3;i++)
			pXYZPt[i] = PL[i] + (eigVal[1] * rotMatFinal[3*i + 1]);

		RVLGetKinect2DData(pUVDPt,pXYZPt, m_pStereoVision->m_KinectParams);

		dU = pUVDPt[0] - pUVDCenterPt[0];
		dV = pUVDPt[1] - pUVDCenterPt[1];
		if((dU*dU + dV*dV) <= iPlaneWidth)
		{
			p2DRegion->m_vp3DSurface = NULL;

			//reset initial pointers
			ppVertexPt = ppVertexPt0;
			pIPoint = pIPointStart;

			continue;
		}

		nTotalVertexPts += nVertices;
		p2DRegion->m_nVertices = nVertices; 
		p2DRegion->m_pVertexArray = ppVertexPt0;

		//Create convex surface and store params
		p3DConvexSurface = (CRVL3DSurface2 *)(RVL3DSurfaceTemplate.Create3(p3DConvexSegmentSet));
		
		//Store normal
		RVLCOPY3VECTOR(p3DSurfaceLevel3->m_N,p3DConvexSurface->m_N);

		//Final translation vector O->L->F
		RVLCOPY3VECTOR(PL,p3DConvexSurface->m_Pose.m_X);

		//Final rotation matrix (O->L->F) O_R_L * L_R_F
		RVLCOPYMX3X3(rotMatFinal,p3DConvexSurface->m_Pose.m_Rot);
		

		//store rho ie m_d
		double md = RVLDOTPRODUCT3(p3DConvexSurface->m_Pose.m_X, p3DConvexSurface->m_N);
		p3DConvexSurface->m_d = p3DSurfaceLevel3->m_d;

		//store eigen values
		p3DConvexSurface->m_EigenValues[0] = eigVal[0];
		p3DConvexSurface->m_EigenValues[1] = eigVal[1];

		
		//copy support and check max size
		p3DConvexSurface->m_nSupport = p2DRegion->m_n3DPts;
		nSupportPts += p2DRegion->m_n3DPts;

		p3DSurfaceLevel3->m_Area += A;

		//if (p3DConvexSurface->m_nSupport>maxSize)
		//	maxSize = p3DConvexSurface->m_nSupport;

		//Create 2D contour
		p2DContour = (CRVL2DContour *)(RVL2DContourTemplate.Create3(p2DContourSet));
		p2DContour->m_ContourIPArray = pIPointStart;
		p2DContour->m_nContourIPs = pIPoint - pIPointStart;
		//Add to contour array
		*(pp2DContour++) = p2DContour;

		//Relate 2D contour to 3D surface
		pp2DContourPtr = (CRVL2DContour **)(p3DConvexSurface->m_pData + p3DConvexSegmentSet->m_iDataContourPtr);
		*pp2DContourPtr = p2DContour;


		//Calculate uncertainty of xyz centroid using uvd centroid  ???? (Using uvd centroid determined from 3d centroid??!!)
		m_pStereoVision->m_pCameraL->KinectReconWithUncert((double)pUVDCenterPt[0],(double)pUVDCenterPt[1],(double)pUVDCenterPt[2],
															m_pStereoVision->m_KinectParams.d0,
															m_pStereoVision->m_KinectParams.k,
															m_pStereoVision->m_KinectParams.depthUc,
															m_pStereoVision->m_KinectParams.depthVc,
															m_pStereoVision->m_KinectParams.depthFu,
															m_pStereoVision->m_KinectParams.depthFv, 
															m_RuvTol*m_RuvTol, m_RuvdTol*m_RuvdTol, C);

		MatrixMultiplication(C,p3DConvexSurface->m_N,Ctemp,3,3,1);

		double sigmaR = RVLDotProduct(p3DConvexSurface->m_N,Ctemp);

		p3DConvexSurface->m_sigmaR = sigmaR;

		sigmaR *= (p3DSurfaceLevel3->m_pClass->m_UncertCoeff * p3DSurfaceLevel3->m_pClass->m_UncertCoeff);

		p3DConvexSurface->m_varq[0] =  sigmaR / (p3DConvexSurface->m_EigenValues[0] * p3DConvexSurface->m_EigenValues[0] + sigmaR);
		p3DConvexSurface->m_varq[1] =  sigmaR / (p3DConvexSurface->m_EigenValues[1] * p3DConvexSurface->m_EigenValues[1] + sigmaR);
		p3DConvexSurface->m_varq[2] =  sigmaR;
		
		//Relate 3D convex surface to LEVEL2 region and vice versa 
		p2DRegion->m_vp3DSurface = p3DConvexSurface;
		p3DConvexSurface->m_vp2DRegion = p2DRegion;

		////add to Convex surface array ie Insert into appropriate position (Sort procedure)
		if(nRegions == 0)
		{
			//store new value
			ConvexSegmentArray[0] = p3DConvexSurface;
			ConvexSegmentArray[0]->m_Index = 0;
		}
		else
		{
			bInsert = false;
			for(i=0;i<nRegions;i++)
			{
				if(p3DConvexSurface->m_nSupport > ConvexSegmentArray[i]->m_nSupport)
				{
					iNewPosition = i;
					//Shift all values down first
					for(j=nRegions;j>i;j--)
					{
						ConvexSegmentArray[j] = ConvexSegmentArray[j-1];
						ConvexSegmentArray[j]->m_Index = j;
					}
					bInsert = true;

					break;
				}
				
			}

			if(bInsert==false)
				iNewPosition = nRegions;

			//store new value
			ConvexSegmentArray[iNewPosition] = p3DConvexSurface;
			ConvexSegmentArray[iNewPosition]->m_Index = iNewPosition;
		}

		nRegions++;

		
	} 

	nConvexSegments = nRegions;
#endif	
}


//Get normal and rho of xyz plane from uvd plane
void CRVLPlanarSurfaceDetector::Get3DPlaneKinect(CRVL2DRegion2 *p2DRegion, CRVL3DSurface2 *p3DSurface)
{
	double aa, bb, cc, temp;

	RVLKINECT_PARAMS kinectParams = m_pStereoVision->m_KinectParams;

	cc = p2DRegion->m_a*kinectParams.depthUc + p2DRegion->m_b*kinectParams.depthVc + p2DRegion->m_c - kinectParams.d0;
	aa = p2DRegion->m_a*kinectParams.depthFu;
	bb = p2DRegion->m_b*kinectParams.depthFv;

	temp = sqrt(aa*aa + bb*bb + cc*cc);

	p3DSurface->m_N[0] = aa/temp;
	p3DSurface->m_N[1] = bb/temp;
	p3DSurface->m_N[2] = cc/temp;

	p3DSurface->m_d = -kinectParams.k/temp;

	
	
}

//Get a,b,c params of a plane in uvd-space from normal and rho of a plane in Euclidean space

void CRVLPlanarSurfaceDetector::Get2DPlaneKinect(double *N,
												 double d,
												 double &a,
												 double &b,
												 double &c)
{
	RVLKINECT_PARAMS kinectParams = m_pStereoVision->m_KinectParams;

	double tmp = -kinectParams.k/d;

	a = N[0] * tmp / kinectParams.depthFu;
	b = N[1] * tmp / kinectParams.depthFv;
	c = N[1] * tmp - a*kinectParams.depthUc - b*kinectParams.depthVc + kinectParams.d0;
}

int CRVLPlanarSurfaceDetector::Sample2DRegions()
{
	int ImageSize = m_Width * m_Height;

	// Initialize m_DTMap.

	memset(m_DTMap, 0x7f, ImageSize * sizeof(int));

	// Initialize SampleMap

	RVLPSD_2DREGION_SAMPLE **SampleMap = new RVLPSD_2DREGION_SAMPLE *[ImageSize];

	memset(SampleMap, 0, ImageSize * sizeof(RVLPSD_2DREGION_SAMPLE *));

	//int OutDist = m_SamplingResolution / 2;
	int OutDist = 0;

	// Set each element of m_DTMap corresponding to a pixel which does not belong to a dominant region 
	// or is on the boundary of a dominant region to OutDist.
	// Store ptrs. to all pixels on the boundary of the dominant regions to the buffer m_APixBuff
	RVLAPIX *APixArray = m_pAImage->m_pPix;

	RVLAPIX **ppPt1 = m_APixBuff;

	CRVL2DRegion2 **p2DRegionMapEnd = m_2DRegionMap + ImageSize;

	int iPix = 0;

	int umaxNrm = 2 * m_Width - 1;
	int vmaxNrm = 2 * m_Height - 1;

	int iNeighbor;
	int iPix2, iPix3;
	RVLAPIX *pPt, *pPt2;
	CRVL2DRegion2 **pp2DRegion;
	CRVL2DRegion2 *p2DRegion, *p2DRegion2;
	BOOL bEdge;

	for(pp2DRegion = m_2DRegionMap; pp2DRegion < p2DRegionMapEnd; pp2DRegion++, iPix++)
	{
		pPt = APixArray + iPix;

		p2DRegion = *pp2DRegion;

		if(p2DRegion == NULL)
		{
			m_DTMap[iPix] = OutDist;

			continue;
		}

		//if((p2DRegion->m_Flags & RVLOBJ2_FLAG_DOMINANT) == 0)
		//{
		//	m_DTMap[iPix] = OutDist;

		//	continue;
		//}

		for(iNeighbor = 0; iNeighbor < 4; iNeighbor++)
		{
			if(iNeighbor & 1)
			{
				if(bEdge = ((pPt->v >> 1) == m_NeighborLimit[iNeighbor]))
					break;
			}
			else
			{
				if(bEdge = ((pPt->u >> 1) == m_NeighborLimit[iNeighbor]))
					break;
			}

			iPix2 = iPix + m_dpNeighbor4[iNeighbor];

			p2DRegion2 = m_2DRegionMap[iPix2];

			if(bEdge = (p2DRegion2 == NULL))
				break;

			if(bEdge = (p2DRegion2 != p2DRegion))
				break;
		}

		if(bEdge)
		{
			m_DTMap[iPix] = OutDist;

			*(ppPt1++) = pPt;
		}
	}

	// Compute distance transformation. The result is stored in m_DTMap. 

	RVLAPIX **ppPt2 = ppPt1;

	ppPt1 = m_APixBuff;

	int maxDist = 0;

	BYTE mBlockDirections[] = {0x01, 0x02, 0x04, 0x08, 0x03, 0x06, 0x0c, 0x09};

	int dist, dist2, dist3;
	BYTE mBlock;
	BYTE mBlockDirection;
	BYTE *pmBlockDirections;

	while(ppPt1 < ppPt2)
	{
		pPt2 = *ppPt1;

		iPix2 = pPt2 - APixArray;

		dist2 = m_DTMap[iPix2];

		if(dist2 > maxDist)
			maxDist = dist2;

		dist3 = dist2 + 1;

		mBlock = 0x00;
		mBlockDirection = 0x01;

		for(iNeighbor = 0; iNeighbor < 4; iNeighbor++, mBlockDirection = (mBlockDirection << 1))
		{
			if(iNeighbor & 1)
			{
				if((pPt2->v >> 1) == m_NeighborLimit[iNeighbor])
					mBlock |= mBlockDirection;
			}
			else
			{
				if((pPt2->u >> 1) == m_NeighborLimit[iNeighbor])
					mBlock |= mBlockDirection;
			}
		}

		pmBlockDirections = mBlockDirections;

		for(iNeighbor = 0; iNeighbor < 8; iNeighbor++, pmBlockDirections++)
		{
			if(mBlock & (*pmBlockDirections))
				continue;

			iPix3 = iPix2 + m_dpNeighbor4[iNeighbor];

			if(m_DTMap[iPix3] > dist3)
			{
				m_DTMap[iPix3] = dist3;

				*(ppPt2++) = APixArray + iPix3;
			}
		}

		ppPt1++;
	}

#ifdef RVLPSD_SAMPLE_SURF_DEBUG
	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("DT");

	pFig->m_pImage = cvCreateImage(cvSize(m_Width, m_Height), IPL_DEPTH_8U, 3);

	unsigned char *PixArray = (unsigned char *)(pFig->m_pImage->imageData);

	RVLDisplayDistanceTransformMap(m_DTMap, SampleMap, ImageSize, OutDist, PixArray);

	m_DebugData.pGUI->ShowFigure(pFig);

	cvWaitKey();
#endif

	// Initialize DTQueue

	CRVLQListArray DTQueue;

	DTQueue.m_Size = maxDist + 1;

	DTQueue.m_ListArray = new RVLQLIST[DTQueue.m_Size];

	DTQueue.InitListArray(DTQueue.m_ListArray, DTQueue.m_ListArray);

	RVLQLIST *DTQueueListArray = DTQueue.m_ListArray;

	RVLQLIST *pDTQueueList;

	// For every pixel with distance >= m_minSampleSize an entry is inserted into DTQueue

	RVLPSD_DT_QUEUE_ENTRY *pDTQueueEntry = m_DTQueueEntryMem;

	for(iPix = 0; iPix < ImageSize; iPix++)
	{
		dist = m_DTMap[iPix];

		if(dist >= m_minSampleSize)
		{
			pDTQueueList = DTQueueListArray + dist;

			pDTQueueEntry->dist = dist;
			pDTQueueEntry->iPix = iPix;

			RVLQLIST_ADD_ENTRY(pDTQueueList, pDTQueueEntry)

			pDTQueueEntry++;
		}
	}

	//***** Sampling

	int nSamples = 0;

	pDTQueueList = DTQueueListArray + maxDist;

	void **ppDTQueueEntry;
	RVLPSD_DT_QUEUE_ENTRY *pNextDTQueueEntry;
	CRVLMem *pMem;
	RVLPSD_2DREGION_SAMPLE *pSample;
	RVLQLIST *pSamples;
	int SampleSize;

	while(pDTQueueList > DTQueueListArray)
	{
		pDTQueueEntry = (RVLPSD_DT_QUEUE_ENTRY *)(pDTQueueList->pFirst);

		ppDTQueueEntry = &(pDTQueueList->pFirst);

		while(pDTQueueEntry)
		{
			pNextDTQueueEntry = (RVLPSD_DT_QUEUE_ENTRY *)(pDTQueueEntry->pNext);

			iPix = pDTQueueEntry->iPix;

			dist = m_DTMap[iPix];

			if(dist < m_minSampleSize)
			{
				ppDTQueueEntry = &(pDTQueueEntry->pNext);

				pDTQueueEntry = pNextDTQueueEntry;

				continue;
			}
			//else if(dist < m_SamplingResolution)
			//{
			//	if(SampleMap[iPix])
			//	{
			//		ppDTQueueEntry = &(pDTQueueEntry->pNext);

			//		pDTQueueEntry = pNextDTQueueEntry;

			//		continue;
			//	}
			//}

			if(dist == pDTQueueEntry->dist)
			{
				// p2DRegion <- the region containing pPt

  				p2DRegion = m_2DRegionMap[iPix];

				// add new sample to p2DRegion->m_Samples

				pMem = p2DRegion->m_pClass->m_pMem0;

				RVLMEM_ALLOC_STRUCT(pMem, RVLPSD_2DREGION_SAMPLE, pSample)

				pPt = APixArray + iPix;

				pSample->pPt = pPt;
				pSample->w = 0;

				pSamples = &(p2DRegion->m_Samples);

				RVLQLIST_ADD_ENTRY(pSamples, pSample)

				nSamples++;

				// update m_DTMap by adding the new sample pixel

				SampleSize = m_DTMap[iPix];

				if(SampleSize > m_maxSampleSize)
					SampleSize = m_maxSampleSize;

				m_DTMap[iPix] = -SampleSize;

#ifdef RVLPSD_SAMPLE_SURF_DEBUG
				SampleMap[iPix] = pPt;
#endif

				ppPt1 = m_APixBuff;

				*ppPt1 = pPt;

				ppPt2 = ppPt1 + 1;

				while(ppPt1 < ppPt2)
				{
					pPt2 = *ppPt1;

					iPix2 = pPt2 - APixArray;

					dist3 = m_DTMap[iPix2] + 1;

					for(iNeighbor = 0; iNeighbor < 8; iNeighbor++)
					{
						//if(iNeighbor & 1)
						//{
						//	if((pPt2->v >> 1) == m_NeighborLimit[iNeighbor])
						//		continue;
						//}
						//else
						//{
						//	if((pPt2->u >> 1) == m_NeighborLimit[iNeighbor])
						//		continue;
						//}

						iPix3 = iPix2 + m_dpNeighbor4[iNeighbor];

						if(m_DTMap[iPix3] > dist3)
						{
							m_DTMap[iPix3] = dist3;

							SampleMap[iPix3] = pSample;

							*(ppPt2++) = APixArray + iPix3;
						}
					}

					ppPt1++;
				}

#ifdef RVLPSD_SAMPLE_SURF_DEBUG
				RVLDisplayDistanceTransformMap(m_DTMap, SampleMap, ImageSize, OutDist, PixArray);

				m_DebugData.pGUI->ShowFigure(pFig);

				cvWaitKey();
#endif

				ppDTQueueEntry = &(pDTQueueEntry->pNext);
			}
			else
			{
				pDTQueueEntry->dist = dist;

				// update the position of pDTQueueEntry in DTQueue according to dist

				RVLQLIST_REMOVE_ENTRY(pDTQueueList, pDTQueueEntry, ppDTQueueEntry)

				RVLQLISTARRAY_ADD_ENTRY(DTQueueListArray, dist, pDTQueueEntry)
			}

			pDTQueueEntry = pNextDTQueueEntry;
		}

		pDTQueueList--;
	}

#ifdef RVLPSD_SAMPLE_SURF_DEBUG
	delete[] SampleMap;

	m_DebugData.pGUI->ShowFigure(pFig);

 	cvWaitKey();	

   	cvDestroyWindow("DT");
#endif

	// assign a region of influence to each sample

	int w;

	for(iPix2 = 0; iPix2 < ImageSize; iPix2++)
	{
		pSample = SampleMap[iPix2];

		if(pSample)
		{
			pPt = pSample->pPt;

			iPix = pPt - APixArray;

			w = m_DTMap[iPix2] - m_DTMap[iPix];

			if(w > pSample->w)
				pSample->w = w;			
		}
	}

	delete[] SampleMap;

	//// only for debugging purpose!

	//int minw = m_Width;
	//int maxw = 0;

	//for(pp2DRegion = m_2DRegionMap; pp2DRegion < p2DRegionMapEnd; pp2DRegion++)
	//	if(*pp2DRegion != NULL)
	//		break;

	//p2DRegion = *pp2DRegion;

	//CRVLClass *p2DRegionSet = p2DRegion->m_pClass;

	//CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	//p2DRegionList->Start();

	//while(p2DRegionList->m_pNext)
	//{
	//	p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

	//	pSample = (RVLPSD_2DREGION_SAMPLE *)(p2DRegion->m_Samples.pFirst);

	//	while(pSample)
	//	{
	//		if(pSample->w > maxw)
	//			maxw = pSample->w;

	//		if(pSample->w < minw)
	//			minw = pSample->w;

	//		pSample = (RVLPSD_2DREGION_SAMPLE *)(pSample->pNext);
	//	}
	//}

	/////

  	return nSamples;
}

void CRVLPlanarSurfaceDetector::Display2DRegionMap(PIX_ARRAY *pOutPixArray)
{
	int ImageSize = m_Width * m_Height;

	BYTE *pOutPix = pOutPixArray->pPix;

	RVLAPIX *APixArray = m_pAImage->m_pPix;

	CRVL2DRegion2 **p2DRegionMapEnd = m_2DRegionMap + ImageSize;

	int iPix = 0;

	int umaxNrm = 2 * m_Width - 1;
	int vmaxNrm = 2 * m_Height - 1;

	int iNeighbor;
	int iPix2;
	RVLAPIX *pPt;
	CRVL2DRegion2 **pp2DRegion;
	CRVL2DRegion2 *p2DRegion, *p2DRegion2;
	BOOL bEdge;

	for(pp2DRegion = m_2DRegionMap; pp2DRegion < p2DRegionMapEnd; pp2DRegion++, iPix++)
	{
		pPt = APixArray + iPix;

		p2DRegion = *pp2DRegion;

		if(p2DRegion == NULL)
		{
			*(pOutPix++) = 0;
			*(pOutPix++) = 0;
			*(pOutPix++) = 0;

			continue;
		}

		for(iNeighbor = 0; iNeighbor < 4; iNeighbor++)
		{
			if(iNeighbor & 1)
			{
				if(bEdge = ((pPt->v >> 1) == m_NeighborLimit[iNeighbor]))
					break;
			}
			else
			{
				if(bEdge = ((pPt->u >> 1) == m_NeighborLimit[iNeighbor]))
					break;
			}

			iPix2 = iPix + m_dpNeighbor4[iNeighbor];

			p2DRegion2 = m_2DRegionMap[iPix2];

			if(bEdge = (p2DRegion2 == NULL))
				break;

			if(bEdge = (p2DRegion2 != p2DRegion))
				break;
		}

		if(bEdge)
		{
			*(pOutPix++) = 0;
			*(pOutPix++) = 0;
			*(pOutPix++) = 0;
		}
		else
		{
			*(pOutPix++) = 128;
			*(pOutPix++) = 128;
			*(pOutPix++) = 128;
		}
	}	
}

// uses m_pMem2

void CRVLPlanarSurfaceDetector::GetTexture(CRVL3DSurface2 * p3DSurface,
										   DWORD Flags)
{
	CRVLMem *pMem2 = m_pMem2;

	//RVL3DPOINT2 **Point3DMap = m_Point3DMap;

	CRVL3DSurface2 **SurfaceMap = m_3DSurfaceMap;

	RVLAPIX_2DREGION_PTR *TextureMap = m_pAImage->m_2DRegionMap2;
	
	CRVLStereoVision *pStereoVision = m_pStereoVision;

	double fuD = pStereoVision->m_KinectParams.depthFu;
	double fvD = pStereoVision->m_KinectParams.depthFv;
	double ucD = pStereoVision->m_KinectParams.depthUc;
	double vcD = pStereoVision->m_KinectParams.depthVc;
	double *tRGBD = pStereoVision->m_KinectParams.pPose->m_X;
	double *RDRGB = pStereoVision->m_KinectParams.pPose->m_Rot;
	double fuRGB = pStereoVision->m_KinectParams.rgbFu;
	double fvRGB = pStereoVision->m_KinectParams.rgbFv;
	double ucRGB = pStereoVision->m_KinectParams.rgbUc;
	double vcRGB = pStereoVision->m_KinectParams.rgbVc;

	int ImageSize = m_Width * m_Height;

	char *RGBPixArray = pStereoVision->m_pCameraL->m_pRGBImage->imageData;

	CRVLMPtrChain *pTextonList = &(m_pAImage->m_C2DRegion2.m_ObjectList);

	// homography between the depth camera and the p3DSurface

	double *RFD = p3DSurface->m_Pose.m_Rot;
	double *tFD = p3DSurface->m_Pose.m_X;

	double HFD[3 * 3];

	RVLHomography(RFD, tFD, fuD, fvD, ucD, vcD, HFD);

	double HDF[3 * 3];

	InverseMatrix3(HDF, HFD);

	// homography between the surface and the RGB image

	double HFRGB[3 * 3];
	double RFRGB[3 * 3];
	double tFRGB[3];
	double tmp3x1[3];

	RVLMXMUL3X3(RDRGB, RFD, RFRGB)
	RVLDIF3VECTORS(tFD, tRGBD, tmp3x1)
	RVLMULMX3X3VECT(RDRGB, tmp3x1, tFRGB)

	RVLHomography(RFRGB, tFRGB, fuRGB, fvRGB, ucRGB, vcRGB, HFRGB);

	// create output image

	CRVL3DSurface2 **pp3DSurface = SurfaceMap;

	int maxTxtSize = 600;		// pix

	double fUD[3];

	fUD[2] = 1.0;

	BOOL bFirstPoint = TRUE;

	double minX, maxX, minY, maxY;
	int uD, vD;
	double PF[3];

	for(vD = 0; vD < m_Height; vD++)
	{
		for(uD = 0; uD < m_Width; uD++, pp3DSurface++)
			if((*pp3DSurface) == p3DSurface)
			{
				fUD[0] = (double)uD;
				fUD[1] = (double)vD;

				RVLMULMX3X3VECT(HDF, fUD, PF)

				PF[0] /= PF[2];
				PF[1] /= PF[2];

				if(bFirstPoint)
				{
					minX = maxX = PF[0];
					minY = maxY = PF[1];

					bFirstPoint = FALSE;
				}
				else
				{
					if(PF[0] < minX)
						minX = PF[0];

					if(PF[0] > maxX)
						maxX = PF[0];

					if(PF[1] < minY)
						minY = PF[1];

					if(PF[1] > maxY)
						maxY = PF[1];
				}
			}
	}

	double rangeX = maxX - minX;
	double rangeY = maxY - minY;

	int minuTxt, maxuTxt, minvTxt, maxvTxt;
	int wTxt, hTxt;
	double res;

	if(rangeX > rangeY)
	{
		wTxt = maxTxtSize;
		res = rangeX / (double)wTxt;
		hTxt = DOUBLE2INT(rangeY / res);		
	}
	else
	{
		hTxt = maxTxtSize;
		res = rangeY / (double)hTxt;
		wTxt = DOUBLE2INT(rangeX / res);		
	}

	minuTxt = DOUBLE2INT(minX / res);
	maxuTxt = DOUBLE2INT(maxX / res);
	minvTxt = DOUBLE2INT(minY / res);
	maxvTxt = DOUBLE2INT(maxY / res);

	int TxtImageSize = wTxt * hTxt;
	
	IplImage *pOutImage = cvCreateImage(cvSize(wTxt, hTxt), IPL_DEPTH_8U, 3);

	cvSet(pOutImage, cvScalar(255, 0, 0));

	char *OutPixArray = pOutImage->imageData;

	// create mask and texton list

	CRVL2DRegion2 **TextonList;
	CRVL2DRegion2 **ppTexton;
	BYTE *Mask;
	
	if((Flags & RVLPSDGETTEXTURE_FLAG_ORTOGONAL_MAPPING) == 0)
	{
		Mask = new BYTE[TxtImageSize];

		memset(Mask, 0, TxtImageSize * sizeof(BYTE));
	}

	if(((Flags & RVLPSDGETTEXTURE_FLAG_ORTOGONAL_MAPPING) == 0) ||
		((Flags & RVLPSDGETTEXTURE_FLAG_TEXTON_ELLIPSES) != 0))
	{
		TextonList = new CRVL2DRegion2 *[pTextonList->m_nElements];

		ppTexton = TextonList;
	}

	PF[2] = 1.0;

	double URGB[3];
	int uRGB, vRGB;
	int uTxt, vTxt;
	double UD[3];
	char *pPix;
	//RVL3DPOINT2 *p3DPt;
	int iPixD;
	char *pPixRGB;
	CRVL2DRegion2 *pTexton;
	CRVL3DSurface2 *p3DSurface2;
	
	for(vTxt = 0; vTxt < hTxt; vTxt++)
	{
		PF[1] = res * (double)(hTxt - vTxt - 1 + minvTxt);

		for(uTxt = 0; uTxt < wTxt; uTxt++)
		{
			// compute UD - the projection of PF to the depth image

			PF[0] = res * (double)(uTxt + minuTxt);

			RVLMULMX3X3VECT(HFD, PF, UD)

			uD = DOUBLE2INT(UD[0] / UD[2]);
			vD = DOUBLE2INT(UD[1] / UD[2]);

			// retrieve the surface corresponding to UD

			if(uD < 0)
				continue;

			if(uD >= m_Width)
				continue;

			if(vD < 0)
				continue;

			if(vD >= m_Height)
				continue;

			iPixD = uD + vD * m_Width;

			p3DSurface2 = SurfaceMap[iPixD];

			// if the surface corresponding to UD is not p3DSurface, then go to the next PF

			if(p3DSurface2 != p3DSurface)
				continue;

			// project PF to RGB image using homography HFRGB

			//p3DPt = Point3DMap[iPixD];

			pPix = OutPixArray + 3 * (uTxt + vTxt * wTxt);

			RVLMULMX3X3VECT(HFRGB, PF, URGB)

			uRGB = DOUBLE2INT(URGB[0] / URGB[2]);
			vRGB = DOUBLE2INT(URGB[1] / URGB[2]);

			if(Flags & RVLPSDGETTEXTURE_FLAG_ORTOGONAL_MAPPING)
			{
				// copy the color of the projection of PF to the texture image

				pPixRGB = RGBPixArray + 3 * (uRGB + vRGB * m_Width);

				*(pPix++) = *(pPixRGB++);
				*(pPix++) = *(pPixRGB++);
				*(pPix++) = *(pPixRGB++);
			}
			else
			{
				// update Mask

				Mask[uTxt + vTxt * wTxt] = 1;
			}

			if(((Flags & RVLPSDGETTEXTURE_FLAG_ORTOGONAL_MAPPING) == 0) ||
				((Flags & RVLPSDGETTEXTURE_FLAG_TEXTON_ELLIPSES) != 0))
			{			
				// update TextonList

				pTexton = (CRVL2DRegion2 *)(TextureMap[uRGB + vRGB * m_Width].p2DRegion);

				if((pTexton->m_Flags & RVLOBJ2_FLAG_MARKED) == 0)
				{
					pTexton->m_Flags |= RVLOBJ2_FLAG_MARKED;

					*(ppTexton++) = pTexton;
				}
			}
		}
	}

	if(((Flags & RVLPSDGETTEXTURE_FLAG_ORTOGONAL_MAPPING) == 0) ||
		((Flags & RVLPSDGETTEXTURE_FLAG_TEXTON_ELLIPSES) != 0))
	{
		//*** reconstruct texture from textons

		RVLQLIST *pTextonList = &(p3DSurface->m_TextonList);

		RVLDisplayTextons(pTextonList, res, minuTxt, minvTxt, hTxt, pOutImage, pMem2);
	}
	// display image

	cvShowImage("Texture", pOutImage);

	// deallocate memory

	if((Flags & RVLPSDGETTEXTURE_FLAG_ORTOGONAL_MAPPING) == 0)
		delete[] Mask;

	if(((Flags & RVLPSDGETTEXTURE_FLAG_ORTOGONAL_MAPPING) == 0) ||
		((Flags & RVLPSDGETTEXTURE_FLAG_TEXTON_ELLIPSES) != 0))
		delete[] TextonList;
	
	cvReleaseImage(&pOutImage);
}

void CRVLPlanarSurfaceDetector::Get3DTextons(CRVLMPtrChain *p3DSurfaceList)
{
	//CRVLMem *pMem2 = m_pMem2;

	//CRVL3DSurface2 **SurfaceMap = m_3DSurfaceMap;

	RVLAPIX_2DREGION_PTR *TextureMap = m_pAImage->m_2DRegionMap2;
	
	CRVLStereoVision *pStereoVision = m_pStereoVision;

	double fuD = pStereoVision->m_KinectParams.depthFu;
	double fvD = pStereoVision->m_KinectParams.depthFv;
	double ucD = pStereoVision->m_KinectParams.depthUc;
	double vcD = pStereoVision->m_KinectParams.depthVc;
	double *tRGBD = pStereoVision->m_KinectParams.pPose->m_X;
	double *RDRGB = pStereoVision->m_KinectParams.pPose->m_Rot;
	double fuRGB = pStereoVision->m_KinectParams.rgbFu;
	double fvRGB = pStereoVision->m_KinectParams.rgbFv;
	double ucRGB = pStereoVision->m_KinectParams.rgbUc;
	double vcRGB = pStereoVision->m_KinectParams.rgbVc;

	//int ImageSize = m_Width * m_Height;

	//char *RGBPixArray = pStereoVision->m_pCameraL->m_pRGBImage->imageData;

	int nTextons = m_pAImage->m_C2DRegion2.m_ObjectList.m_nElements;

	RVLTEXTON **TextonRegionArray = new RVLTEXTON *[nTextons];

	CRVL2DRegion2 **TextonRegionBuff = new CRVL2DRegion2 *[nTextons];

	double UD[3];

	UD[2] = 1.0;

	CRVL3DSurface2 *p3DSurface;
	CRVL2DRegion2 *p2DRegion;
	RVLQLIST *pPtList;
	RVLQLIST_INT_ENTRY *pPtIdx;
	double *RFD, *tFD;
	double HFD[3 * 3], HDF[3 * 3], HDRGB[3 * 3], HFRGB[3 * 3], HRGBF[3 * 3];
	double RFRGB[3 * 3];
	double tFRGB[3];
	double tmp3x1[3];
	double URGB[3];
	int uD, vD, uRGB, vRGB;
	int iPixRGB;
	RVLTEXTON *pTexton;
	RVLQLIST *pTextonList;
	CRVL2DRegion2 *pTextonRegion;
	CRVL2DRegion2 **ppTextonRegion, **pTextonRegionBuffEnd;
	RVL2DMOMENTS *pMoments;
	double fn;
	double a[3];
	double J[2 * 2], CF[2 * 2];
	double detCF;
	double k1, k2, k3, k4, c11, c12, c22;

	p3DSurfaceList->Start();

	while(p3DSurfaceList->m_pNext)
	{
		p3DSurface = (CRVL3DSurface2 *)(p3DSurfaceList->GetNext());

		pTextonList = &(p3DSurface->m_TextonList);

		RVLQLIST_INIT(pTextonList);

		// homography between the depth camera and the p3DSurface

		RFD = p3DSurface->m_Pose.m_Rot;
		tFD = p3DSurface->m_Pose.m_X;

		RVLHomography(RFD, tFD, fuD, fvD, ucD, vcD, HFD);

		InverseMatrix3(HDF, HFD);

		// homography between the surface and the RGB image

		RVLMXMUL3X3(RDRGB, RFD, RFRGB)
		RVLDIF3VECTORS(tFD, tRGBD, tmp3x1)
		RVLMULMX3X3VECT(RDRGB, tmp3x1, tFRGB)

		RVLHomography(RFRGB, tFRGB, fuRGB, fvRGB, ucRGB, vcRGB, HFRGB);

		InverseMatrix3(HRGBF, HFRGB);

		// homography between p3DSurface in uvd-space and the RGB image

		RVLMXMUL3X3(HFRGB, HDF, HDRGB)

		// assign 3D textons to p3DSurface

		p2DRegion = (CRVL2DRegion2 *)(p3DSurface->m_vp2DRegion);

		ppTextonRegion = TextonRegionBuff;

		pPtList = (RVLQLIST *)(p2DRegion->m_PtArray);

		pPtIdx = (RVLQLIST_INT_ENTRY *)(pPtList->pFirst);

		while(pPtIdx)
		{
			uD = pPtIdx->i % m_Width;
			vD = pPtIdx->i / m_Width;

			UD[0] = (double)uD;
			UD[1] = (double)vD;

			RVLMULMX3X3VECT(HDRGB, UD, URGB);

			uRGB = DOUBLE2INT(URGB[0] / URGB[2]);

			if(uRGB < 0)
			{
				pPtIdx = (RVLQLIST_INT_ENTRY *)(pPtIdx->pNext);

				continue;
			}

			if(uRGB >= m_Width)
			{
				pPtIdx = (RVLQLIST_INT_ENTRY *)(pPtIdx->pNext);

				continue;
			}

			vRGB = DOUBLE2INT(URGB[1] / URGB[2]);

			if(vRGB < 0)
			{
				pPtIdx = (RVLQLIST_INT_ENTRY *)(pPtIdx->pNext);

				continue;
			}

			if(vRGB >= m_Height)
			{
				pPtIdx = (RVLQLIST_INT_ENTRY *)(pPtIdx->pNext);

				continue;
			}

			iPixRGB = uRGB + vRGB * m_Width;

			pTextonRegion = (CRVL2DRegion2 *)(TextureMap[uRGB + vRGB * m_Width].p2DRegion);

			if(pTextonRegion->m_Flags & RVLOBJ2_FLAG_MARKED)
			{
				pTexton = TextonRegionArray[pTextonRegion->m_Index];

				pTexton->nPts++;

				pPtIdx = (RVLQLIST_INT_ENTRY *)(pPtIdx->pNext);

				continue;
			}

			pTextonRegion->m_Flags |= RVLOBJ2_FLAG_MARKED;

			RVLMEM_ALLOC_STRUCT(m_pMem, RVLTEXTON, pTexton)

			RVLQLIST_ADD_ENTRY(pTextonList, pTexton)

			TextonRegionArray[pTextonRegion->m_Index] = pTexton;

			// compute texton center PF

			pMoments = &(pTextonRegion->m_Moments);

			if(pMoments->n == 0)
			{
				pTexton->nPts = 0;

				pPtIdx = (RVLQLIST_INT_ENTRY *)(pPtIdx->pNext);

				continue;
			}

			*(ppTextonRegion++) = pTextonRegion;

			pTexton->nPts = 1;

			fn = (double)(pMoments->n);

			URGB[0] = pMoments->S[0];
			URGB[1] = pMoments->S[1];
			URGB[2] = 1.0;

			RVLMULMX3X3VECT(HRGBF, URGB, a)

			pTexton->x = a[0] / a[2];
			pTexton->y = a[1] / a[2];

			// jacobian for texton point distribution transformation 
			// (The mathematics for this computation is given in RVLMath.doc, 
			// chapter "Transformation of the Uncertainty of a 2D Point Mapped by a Homography")
			
			RVLMXEL(J, 2, 0, 0) = (RVLMXEL(HRGBF, 3, 0, 0) - a[0] * RVLMXEL(HRGBF, 3, 2, 0) / a[2]) / a[2];
			RVLMXEL(J, 2, 0, 1) = (RVLMXEL(HRGBF, 3, 0, 1) - a[0] * RVLMXEL(HRGBF, 3, 2, 1) / a[2]) / a[2];
			RVLMXEL(J, 2, 1, 0) = (RVLMXEL(HRGBF, 3, 1, 0) - a[1] * RVLMXEL(HRGBF, 3, 2, 0) / a[2]) / a[2];
			RVLMXEL(J, 2, 1, 1) = (RVLMXEL(HRGBF, 3, 1, 1) - a[1] * RVLMXEL(HRGBF, 3, 2, 1) / a[2]) / a[2];

			// transform the texton point distribution

			RVLCOV2DTRANSF(pMoments->S2, J, CF)

			detCF = RVLDET2(CF);

			c11 = CF[3]/detCF;
			c12 = -CF[1]/detCF;
			c22 = CF[0]/detCF;

			k1 = c11 + c22;
			k2 = c22 - c11;
			k3 = 2 * c12;
			k4 = sqrt(k2 * k2 + k3 * k3);
			
			pTexton->Size = sqrt(2.0 / (k1 + k4));			
			pTexton->Elongation = sqrt((k1 + k4) / (k1 - k4));
			pTexton->Orientation = atan2(-k3, k2) * 0.5;
			pTexton->AvgRelI = (pTextonRegion->m_nEdges * pTextonRegion->m_SI - pTextonRegion->m_SINeighborhood) / pTextonRegion->m_nEdges;

			/////

			pPtIdx = (RVLQLIST_INT_ENTRY *)(pPtIdx->pNext);
		}	// for each pixel

		pTextonRegionBuffEnd = ppTextonRegion;

		for(ppTextonRegion = TextonRegionBuff; ppTextonRegion < pTextonRegionBuffEnd; ppTextonRegion++)
			(*ppTextonRegion)->m_Flags &= ~RVLOBJ2_FLAG_MARKED;
	}	// for each surface
	
	delete[] TextonRegionArray;
	delete[] TextonRegionBuff;
}

void CRVLPlanarSurfaceDetector::GetRegionBoundaries()
{
	// get boundary contours

	int ImageSize = m_Width * m_Height;	

	BYTE *ContourData;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, BYTE, (sizeof(RVL2DCONTOUR_DATA) + 4) * ImageSize, ContourData); 

	BYTE *pContourData = ContourData;

	BYTE *bVisited = new BYTE[ImageSize];

	memset(bVisited, 0, ImageSize);

	CRVL2DRegion2 **p2DRegionMapEnd = m_2DRegionMap + ImageSize;

	int i, u, v;
	int iPix0, iPix, iPix2;
	CRVL2DRegion2 *p2DRegion, *p2DRegion2;
	CRVL2DRegion2 **p2DRegionPtr;
	int iNeighbor;
	BOOL bEdge;
	RVL2DCONTOUR_DATA *pContour;
	RVLQLIST *pBoundaryContourList;
	BYTE edge0;

	for(p2DRegionPtr = m_2DRegionMap; p2DRegionPtr < p2DRegionMapEnd; p2DRegionPtr++)
	{
		p2DRegion = *p2DRegionPtr;

		if(p2DRegion == NULL)
			continue;

		if(p2DRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		iPix = p2DRegionPtr - m_2DRegionMap;

		if(bVisited[iPix])	// it is assumed that a pixel cannot be the first pixel in a contour if it is 
			continue;		// already contained in a boundary

		u = iPix % m_Width;
		v = iPix / m_Width;

		// check if the pixel iPix is on a boundary

		for(iNeighbor = 1; iNeighbor <= 3; iNeighbor += 2)	// it is assumed that each contour must start
															// either with an edge directed to the left (outer boundary)
															// or with an edge directed to the right (inner boundary, hole)
		{
			if(iNeighbor & 1)
			{
				if(bEdge = (v == m_NeighborLimit[iNeighbor]))
					break;
			}
			else
			{
				if(bEdge = (u == m_NeighborLimit[iNeighbor]))
					break;
			}

			iPix2 = iPix + m_dpNeighbor4[iNeighbor];

			p2DRegion2 = m_2DRegionMap[iPix2];

			if(bEdge = (p2DRegion2 == NULL))
				break;

			if(bEdge = (p2DRegion2 != p2DRegion))
				break;
		}

		if(!bEdge)
			continue;

		iPix0 = iPix;
		edge0 = (BYTE)iNeighbor;

		// initialize a boundary contour and add it to the m_BoundaryContourList of p2DRegion

		pContour = (RVL2DCONTOUR_DATA *)(pContourData);

		pContourData += sizeof(RVL2DCONTOUR_DATA);

		pContour->iPix0 = iPix0;
		pContour->Data = pContourData;			

		pBoundaryContourList = &(p2DRegion->m_BoundaryContourList);

		if((p2DRegion->m_Flags & RVL2DREGION_FLAG_BOUNDARY) == 0)
		{
			RVLQLIST_INIT(pBoundaryContourList)

			p2DRegion->m_Flags |= RVL2DREGION_FLAG_BOUNDARY;
		}

		RVLQLIST_ADD_ENTRY(pBoundaryContourList, pContour)

		*(pContourData++) = edge0;

		//* follow the contour and write it to the boundary contour data

		while(TRUE)
		{
			iNeighbor = ((iNeighbor + 3) & 3);

			for(i = 0; i < 3; i++)
			{
				if(iNeighbor & 1)
				{
					if(bEdge = (v == m_NeighborLimit[iNeighbor]))
						break;
				}
				else
				{
					if(bEdge = (u == m_NeighborLimit[iNeighbor]))
						break;
				}

				iPix2 = iPix + m_dpNeighbor4[iNeighbor];

				p2DRegion2 = m_2DRegionMap[iPix2];

				if(bEdge = (p2DRegion2 == NULL))
					break;

				if(bEdge = (p2DRegion2 != p2DRegion))
					break;

				iPix = iPix + m_dpNeighbor4[iNeighbor];

				bVisited[iPix] = 1;

				u = iPix % m_Width;
				v = iPix / m_Width;

				iNeighbor = ((iNeighbor + 1) & 3);
			}

			if(iPix == iPix0)
				if(iNeighbor == edge0)
					break;

			*(pContourData++) = iNeighbor;
		}

		pContour->n = pContourData - pContour->Data;
	}

	m_pMem->m_pFreeMem = pContourData;

	delete[] bVisited;
}

void CRVLPlanarSurfaceDetector::GetOrgPCProjectionParams(double &fu, double &fv, double &uc, double &vc)
{
	//uc = 0.5 * (double)m_Width; 
	//vc = (double)m_Height / 3.0; 
	//f = uc / sqrt(3.0);
	uc = 0.5 * (double)(m_pLidarParams->nPanIn120deg);
	fu = uc / sqrt(3.0);
	double dv_ = m_pLidarParams->dTilt * DEG2RAD;
	double minv_ = 2.0 * tan(m_pLidarParams->minTilt * DEG2RAD);
	fv = 1.0 / dv_;
	vc = -minv_ * fv;
}

bool CRVLPlanarSurfaceDetector::ProjectToFOVExtension(	double *X,
														double *P,
														double *R,
														int &iFOVExtension_)
{
	if (m_pStereoVision->m_pCameraL->m_Flags & RVLCAMERA_FLAG_SPHERICAL)
	{
		iFOVExtension_ = 0;

		RVLUNITMX3(R);

		return true;
	}

	int iFOVExtension;
	double q, cs, sn;
	double X_[3];
	int u, v;

	for(iFOVExtension = -m_nFOVExtensions; iFOVExtension <= m_nFOVExtensions; iFOVExtension++)
	{
		q = (double)iFOVExtension * m_FOVExtension;
		cs = cos(q);
		sn = sin(q);

		X_[2] = cs * X[2] + sn * X[0];
		X_[0] = -sn * X[2] + cs * X[0];

		if(X_[2] < 1.0)
			continue;

		u = DOUBLE2INT(P[0] * X_[0] / X_[2] + P[2]);
		v = DOUBLE2INT(P[4] * X[1]  / X_[2] + P[5]);

		if(u < 0)
			continue;

		if(u >= m_Width)
			continue;

		if(v < 0)
			continue;

		if(v >= m_Height)
			continue;

		iFOVExtension_ = iFOVExtension;

		sn = -sn;

		RVLROTY(cs, sn, R)

		return true;
	}

	return false;
}

void CRVLPlanarSurfaceDetector::GetOrgPC(double * PC, int n)
{
	int ImageSize = m_Width * m_Height;

	int maxn3DPts =  (2 * m_nFOVExtensions + 1) * ImageSize;

	memset(m_Point3DMapMem, 0, maxn3DPts * sizeof(RVL3DPOINT2 *));

	m_Point3DMap = m_Point3DMapMem + m_nFOVExtensions * ImageSize;

	RVL3DPOINT2 *pPoint3D = m_Point3DArray;
	
	double fu, fv, uc, vc;

	GetOrgPCProjectionParams(fu, fv, uc, vc);

	double cs[RVLPSD_MAXN_FOV_EXTENSIONS]; 
	double sn[RVLPSD_MAXN_FOV_EXTENSIONS];

	int iFOVExtension;
	double q;

	for(iFOVExtension = 0; iFOVExtension <= m_nFOVExtensions; iFOVExtension++)
	{
		q = (double)iFOVExtension * m_FOVExtension;
		cs[iFOVExtension] = cos(q);
		sn[iFOVExtension] = sin(q);
	}

	double *pPCEnd = PC + 3 * n;

	int iPix;
	double *X;
	int *iX;
	double *X_;
	double X__[3];
	int u, v;
	double r;
	RVL3DPOINT2 *p3DPt;	
	RVL3DPOINT2 **Point3DMap;
	int i;
	double cs_, sn_;

	for(X_ = PC; X_ < pPCEnd; X_ += 3)
	{
		for(iFOVExtension = -m_nFOVExtensions; iFOVExtension <= m_nFOVExtensions; iFOVExtension++)
		{
			i = RVLABS(iFOVExtension);

			cs_ = cs[i];
			sn_ = (iFOVExtension >= 0 ? sn[i] : -sn[i]);

			X__[2] = cs_ * X_[2] + sn_ * X_[0];
			X__[0] = -sn_ * X_[2] + cs_ * X_[0];

			if(X__[2] < 1.0)
				continue;

			u = DOUBLE2INT(fu * X__[0] / X__[2] + uc);
			v = DOUBLE2INT(fv * X_[1] / X__[2] + vc);

			if(u < 0)
				continue;

			if(u >= m_Width)
				continue;

			if(v < 0)
				continue;

			if(v >= m_Height)
				continue;

			r = sqrt(RVLDOTPRODUCT3(X_, X_));

			//if (r < 500.0)
			//	continue;

			iPix = u + v * m_Width;

			Point3DMap = m_Point3DMap + iFOVExtension * ImageSize;

			p3DPt = Point3DMap[iPix];

			if(p3DPt)
			{
				if(r >= p3DPt->r)
					continue;
			}
			else
			{
				p3DPt = pPoint3D;

				p3DPt->u = u;
				p3DPt->v = v;
				p3DPt->iPix = iPix;
				p3DPt->segmentNumber = -1;
				p3DPt->iCell = -1;
				p3DPt->regionList = NULL;

				Point3DMap[iPix] = pPoint3D;

				pPoint3D++;
			}

			X = p3DPt->XYZ;
			RVLCOPY3VECTOR(X_, X)
			p3DPt->r = r;
			iX = p3DPt->iX;
			iX[0] = DOUBLE2INT(X[0]);
			iX[1] = DOUBLE2INT(X[1]);
			iX[2] = DOUBLE2INT(X[2]);
			p3DPt->x = X[0];
			p3DPt->y = X[1];
			p3DPt->z = X[2];

			break;
		}
	}

	m_n3DPoints = pPoint3D - m_Point3DArray;

#ifdef	RVLPSD_SEGMENT_STRM_PC_DEBUG
	FILE *fpPC = fopen("C:\\RVL\\Debug\\PC.txt", "w");

	double P0[3];

	P0[0] = x0Debug; P0[1] = y0Debug; P0[2] = z0Debug;

	double dP[3];

	for (iFOVExtension = -m_nFOVExtensions; iFOVExtension <= m_nFOVExtensions; iFOVExtension++)
	{
		Point3DMap = m_Point3DMap + iFOVExtension * ImageSize;

		for (iPix = 0; iPix < ImageSize; iPix++)
		{
			p3DPt = Point3DMap[iPix];

			if (p3DPt == NULL)
				continue;

			RVLDIF3VECTORS(p3DPt->XYZ, P0, dP);

			if (RVLABS(dP[0]) > rDebug)
				continue;

			if (RVLABS(dP[1]) > rDebug)
				continue;

			if (RVLABS(dP[2]) > rDebug)
				continue;

			fprintf(fpPC, "%lf\t%lf\t%lf\n", p3DPt->XYZ[0], p3DPt->XYZ[1], p3DPt->XYZ[2]);
		}
	}

	fclose(fpPC);
#endif
}

void CRVLPlanarSurfaceDetector::DisplayPC(IplImage *pDisplay)
{
	unsigned char *pPix = (unsigned char *)(pDisplay->imageData);

	int ImageSize = m_Width * m_Height;

	// determine min and max range

	double minr = 10000.0;
	double maxr = 0.0;

	RVL3DPOINT2 *p3DPtListEnd = m_Point3DArray + m_n3DPoints;

	RVL3DPOINT2 *p3DPt;

	for(p3DPt = m_Point3DArray; p3DPt < p3DPtListEnd; p3DPt++)
	{
		if(p3DPt->r < minr)
			minr = p3DPt->r;

		if(p3DPt->r > maxr)
			maxr = p3DPt->r;
	}

	if(minr >= maxr)
	{
		memset(pPix, 0, 3 * (2 * m_nFOVExtensions + 1) * ImageSize);

		return;
	}

	// display projected 3D points with range represented by pixel intensity in logaritmic scale

	unsigned char offset = 64;

	double k = (255.0 - (double)offset) / log(maxr / minr);

	int dpNextRow = (2 * m_nFOVExtensions * m_Width);

	m_Point3DMap = m_Point3DMapMem;

	unsigned char *pPix0 = pPix;

	RVL3DPOINT2 **pp3DPt;
	unsigned char I;
	int iFOVExtension;
	int u, v;
	
	for(iFOVExtension = -m_nFOVExtensions; iFOVExtension <= m_nFOVExtensions; iFOVExtension++, m_Point3DMap += ImageSize, pPix0 += (3 * m_Width))
	{
		pp3DPt = m_Point3DMap;

		pPix = pPix0;

		for(v = 0; v < m_Height; v++)
		{
			for(u = 0; u < m_Width; u++, pp3DPt++)
			{
				p3DPt = *pp3DPt;

				if(p3DPt)
				{
					I = offset + DOUBLE2INT(k * log(p3DPt->r / minr));

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
			}

			u += dpNextRow;
			pPix += (3 * dpNextRow);
		}
	}
}


///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 




void RVLMeshSegmentSWEROnCreateNewNode(	RVLSWER_NODE *pNode,
										RVLSWER_LINK *pLink)
{
	memcpy(pNode->pData, pLink->pData, sizeof(RVL3DMOMENTS));
}

int RVLMeshSegmentWERGetCost(	BYTE *pData,
								int maxCost,
								double k)
{
	RVL3DMOMENTS *pMoments = (RVL3DMOMENTS *)pData;

	BOOL bReal[3];
	double eig[3];
	double C[3 * 3];
	double M[3];

	RVLGetCovMatrix3(pMoments, C, M);

	RVLEig3(C, eig, bReal);
	
	//if(!bReal[0] || !bReal[1] || !bReal[2])
	//	return FALSE;

	Sort3(eig);

	if(eig[0] < 0.0)
		eig[0] = 0.0;

	//return DOUBLE2INT((double)maxCost * sqrt(eig[0] / eig[2]));

	int Cost = DOUBLE2INT(k * sqrt(eig[0]));

	if(Cost > maxCost)
		Cost = maxCost;

	return Cost;
}

int RVLMeshSegmentWERGetCostLog(	BYTE *pData,
									int maxCost,
									double k)
{
	RVL3DMOMENTS *pMoments = (RVL3DMOMENTS *)pData;

	BOOL bReal[3];
	double eig[3];
	double C[3 * 3];
	double M[3];

	RVLGetCovMatrix3(pMoments, C, M);

	RVLEig3(C, eig, bReal);
	
	Sort3(eig);

	if(eig[0] < 0.0)
		return 0;
	
	double std = sqrt(eig[0]);

	if(std <= 100.0)
		return DOUBLE2INT(std);

	int Cost = DOUBLE2INT(log(std / 100.0)/RVLLN1p01)+100;

	if(Cost > maxCost)
		Cost = maxCost;

	return Cost;
}


void RVLMeshSegmentSWERUpdateLink(	RVLSWER_NODE *pNode1,
									RVLSWER_NODE *pNode2,
									RVLSWER_LINK *pLink)
{
	RVL3DMOMENTS *pMomentsNode1 = (RVL3DMOMENTS *)(pNode1->pData);
	RVL3DMOMENTS *pMomentsNode2 = (RVL3DMOMENTS *)(pNode2->pData);

	RVL3DMOMENTS *pMomentsLink = (RVL3DMOMENTS *)(pLink->pData);

	RVLMomentsSum(pMomentsNode1, pMomentsNode2, pMomentsLink);
}


void RVLPSDDisplay(RVLDISPARITYMAP *pDisparityMap, 
				   CRVL2DRegion2 *pPlane,
				   double Tol,
				   PIX_ARRAY *pInPixArray,
				   PIX_ARRAY *pDisplayPixArray)
{
	 
	

	double a = pPlane->m_a;
	double b = pPlane->m_b;
	double c = pPlane->m_c;

	int w = pDisparityMap->Width;
	int h = pDisparityMap->Height;

	int dpPix = (pInPixArray->bColor ? 3 : 1);

	short int *pd = pDisparityMap->Disparity;
	unsigned char *pInPix = pInPixArray->pPix;
	unsigned char *pOutPix = pDisplayPixArray->pPix;

	int u, v;
	double fv;
	short int d;
	double e;
	unsigned char I;

	for(v = 0; v < h; v++)
	{
		fv = (double)v;

		for(u = 0; u < w; u++, pd++, pInPix += dpPix, pOutPix += 3)
		{
			d = *pd;

			I = *pInPix;

			if(d >= 0)
			{
				e = (double)d - (a * (double)u + b * fv + c);
				//m_Histogram.m_Data.Add(&e);

				if(e <= Tol && e >= -Tol)
				{
					*pOutPix = 64 + (unsigned char)((((int)I) * 3) >> 2);
					pOutPix[1] = 0;
					pOutPix[2] = 0;
				}
				else
				{
					*pOutPix = I;
					pOutPix[1] = I;
					pOutPix[2] = I;
				}
			}
			else
			{
				*pOutPix = I;
				pOutPix[1] = I;
				pOutPix[2] = I;
			}
		}
	}

	
		
}


// Display the first 2 segments on the disparity
void RVLPSDDisplayDisparitySegments(RVLDISPARITYMAP *pDisparityMap,
									 RVL3DPOINT2 **ppPoint3DMap,
									 PIX_ARRAY *pDisplayPixArray)
{
	 
	

	
	int w = pDisparityMap->Width;
	int h = pDisparityMap->Height;

	

	short int *pd = pDisparityMap->Disparity;
	
	
	RVL3DPOINT2 **pp3DMap = ppPoint3DMap;
	unsigned char *pOutPix = pDisplayPixArray->pPix;

	RVL3DPOINT2 *pPoint3D;

	int u, v;
	


	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pd++, pp3DMap++, pOutPix +=3)
		{
			*pOutPix = 0;
			pOutPix[1] = 0;
			pOutPix[2] = 0;

			if(*pp3DMap != NULL)
			{
				
					
				pPoint3D = *pp3DMap;
				
			
				if(pPoint3D->refSegmentNumber == 0)
				{
					*pOutPix = 255;
					pOutPix[1] = 255;
					pOutPix[2] = 255;
					
				}
				if(pPoint3D->refSegmentNumber == 1)
				{
					*pOutPix = 64;
					pOutPix[1] = 64;
					pOutPix[2] = 64;
					
				}
			
				
			}
			
		}
	}

	

	
		
}

// Display the first 2 segments on the image
void RVLPSDDisplayImageSegments(RVLDISPARITYMAP *pDisparityMap,
						         RVL3DPOINT2 **ppPoint3DMap,
						         PIX_ARRAY *pInPixArray,
						         PIX_ARRAY *pDisplayPixArray)
{
	 
	

	
	int w = pDisparityMap->Width;
	int h = pDisparityMap->Height;

	short int *pd = pDisparityMap->Disparity;
	
	
	RVL3DPOINT2 **pp3DMap = ppPoint3DMap;
	unsigned char *pInPix = pInPixArray->pPix;
	unsigned char *pOutPix = pDisplayPixArray->pPix;
	
	int dpPix = (pInPixArray->bColor ? 3 : 1);

	RVL3DPOINT2 *pPoint3D;

	int u, v;
	unsigned char I;


	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pd++, pp3DMap++, pInPix += dpPix, pOutPix +=3)
		{
			I = *pInPix;
			
			*pOutPix = I;
			pOutPix[1] = I;
			pOutPix[2] = I;
			
			if(*pp3DMap != NULL)
			{
				
					
				pPoint3D = *pp3DMap;
				
			
				if(pPoint3D->refSegmentNumber == 0)
				{
					*pOutPix = 64 + (unsigned char)((((int)I) * 3) >> 2);
					pOutPix[1] = 0;
					pOutPix[2] = 0;
					//*pOutPix = 255;
					//pOutPix[1] = 255;
					//pOutPix[2] = 255;
				
					
				}
				if(pPoint3D->refSegmentNumber == 1)
				{
					*pOutPix =  0;
					pOutPix[1] = 64 + (unsigned char)((((int)I) * 3) >> 2);
					pOutPix[2] = 0;
					
					//*pOutPix = 64;
					//pOutPix[1] = 64;
					//pOutPix[2] = 64;
					
				}
			
			/*	if(pPoint3D->refSegmentNumber == 1)
				{
					*pOutPix = 0;
					pOutPix[1] = 64 + (unsigned char)((((int)I) * 3) >> 2);
					pOutPix[2] = 0;
					//*pOutPix = 255;
					//pOutPix[1] = 255;
					//pOutPix[2] = 255;
				
					
				}
			*/
			}
			
		}
	}

	

	
		
}


//display floor and main wall segment
void RVLPSDDisplaySegments(RVLDISPARITYMAP *pDisparityMap,
					  PIX_ARRAY *pIIn,
					  PIX_ARRAY *pIOut,
					  CRVL2DRegion2 *pGround,
					  CRVL2DRegion2 **ppWalls,
					  DWORD Flags,
					  bool bMarkSegments)
{

	unsigned char *pPixIn = pIIn->pPix;

	int w = pIIn->Width;
	int h = pIIn->Height;


	unsigned char *pPixOut = pIOut->pPix;

	short int *pd = pDisparityMap->Disparity;


	int u, v;
	unsigned char I;
	int iPix;

	



	//first fill the output image with default values
	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pPixIn++,pPixOut += 3)
		{
			iPix = u + v * w;

		
			//if(Flags & RVLPSDDISPLAY_TWOSEGMENTS)
			//	I = (*pPixIn);
			if(Flags & RVLPSDDISPLAY_DISPARITY_TWOSEGMENTS)
				I = 0; //pd[iPix] < 0 ? 0 : (unsigned char)(pd[iPix]/4);
			else
				I = 128 + (*pPixIn)/2 ;
		
			pPixOut[0] = I;
			pPixOut[1] = I;
			pPixOut[2] = I;
			
		}
	}


	if(bMarkSegments)
	{
		//reset pointer
		pPixOut = pIOut->pPix;


		//fill with values required
		CRVL2DRegion2 *pSegment;

		RVL3DPOINT2 **ppPt, **pPtArrayEnd;
		RVL3DPOINT2 *pPt;

		//define ground
		if(pGround != NULL)
		{
			pSegment = pGround;

			ppPt = (RVL3DPOINT2 **)pSegment->m_PtArray;

			pPtArrayEnd = ppPt + pSegment->m_nPts;


			for(; ppPt < pPtArrayEnd; ppPt++)
			{
				pPt = *ppPt;

				pPixOut = pIOut->pPix + 3 * pPt->iPix;

				I = 255;  //white			pIIn->pPix[pPt->iPix]

				pPixOut[0] = I; //		64 + (unsigned char)((((int)I) * 3) >> 2);
				pPixOut[1] = I;
				pPixOut[2] = I;
				
			}
		}			
		
		//define dominant wall
		if(ppWalls[0]!=NULL)
		{
			pSegment = ppWalls[0];

			ppPt = (RVL3DPOINT2 **)pSegment->m_PtArray;

			pPtArrayEnd = ppPt + pSegment->m_nPts;


			for(; ppPt < pPtArrayEnd; ppPt++)
			{
				pPt = *ppPt;

				pPixOut = pIOut->pPix + 3 * pPt->iPix;

				I = 0; //black   192 - grey

				pPixOut[0] = I;
				pPixOut[1] = I;
				pPixOut[2] = I;
				
			}
		
		}
	}
	
}



//display floor and main wall segment (SIP)
void RVLPSDDisplaySegments2(CRVLAImage *pAImage,
							RVLDISPARITYMAP *pDisparityMap,
						    PIX_ARRAY *pIIn,
						    PIX_ARRAY *pIOut,
						    CRVL2DRegion2 *pGround,
						    CRVL2DRegion2 **ppWalls,
						    DWORD Flags,
							bool bEdgeBasedSegments,
						    bool bMarkSegments)
{

	unsigned char *pPixIn = pIIn->pPix;

	int w = pIIn->Width;
	int h = pIIn->Height;


	unsigned char *pPixOut = pIOut->pPix;

	short int *pd = pDisparityMap->Disparity;


	int u, v;
	unsigned char I;
	int iPix;

	RVL2DREGION_PIXIDX *pPixPtr;



	//first fill the output image with default values
	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pPixIn++,pPixOut += 3)
		{
			iPix = u + v * w;

		
			//if(Flags & RVLPSDDISPLAY_TWOSEGMENTS)
			//	I = (*pPixIn);
			if(Flags & RVLPSDDISPLAY_DISPARITY_TWOSEGMENTS)
				I = 0; //pd[iPix] < 0 ? 0 : (unsigned char)(pd[iPix]/4);
			else
				I = 128 + (*pPixIn)/2 ;
		
			pPixOut[0] = I;
			pPixOut[1] = I;
			pPixOut[2] = I;
			
		}
	}



	

	if(bMarkSegments)
	{
		CRVLMPtrChain *p2DRegionList;

		bool bSuperSegments = (pAImage->m_C2DRegion3.m_ObjectList.m_nElements>0);
		CRVLC2D *p2DRegionSet3, *p2DRegionSet;

		//if there are super segments display them else standard segments
		if(bSuperSegments)
		{
			p2DRegionList = &(pAImage->m_C2DRegion3.m_ObjectList);
			p2DRegionSet3 = &(pAImage->m_C2DRegion3);
		}
		else
		{
			p2DRegionList = &(pAImage->m_C2DRegion.m_ObjectList);
		}
		
		p2DRegionSet = &(pAImage->m_C2DRegion);


		CRVL2DRegion2 **ppSubRegionStart, **ppSubRegionEnd, *p2DSubRegion;
		RVLARRAY *pRelList;
		int nSubSegments;
	
		//fill with values required
		CRVL2DRegion2 *pSegment;


		//reset pointer
		pPixOut = pIOut->pPix;

		int *ppPt, *pPtArrayEnd;
		//int *pPt;

		int j, index;


		if(bSuperSegments)
		{
			//define ground
			if(pGround != NULL)
			{
				pSegment = pGround;

				pRelList = pSegment->m_RelList + p2DRegionSet3->m_iRelList[RVLRELLIST_COMPONENTS];
				ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
				ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
				nSubSegments = ppSubRegionEnd - ppSubRegionStart;

				
					for(j=0;j<nSubSegments;j++,ppSubRegionStart++)
					{
						p2DSubRegion = *ppSubRegionStart;
						
						if(bEdgeBasedSegments)
						{
							pPixPtr = (RVL2DREGION_PIXIDX *)(p2DSubRegion->m_PtArray);

							while(pPixPtr)
							{
								index = pPixPtr->iPix;

								pPixOut = pIOut->pPix + 3 * index;

								I = 255;  //white

								pPixOut[0] = I;
								pPixOut[1] = I;
								pPixOut[2] = I;

								pPixPtr = pPixPtr->pNext;
							}
						}
						else
						{
							ppPt = (int *)p2DSubRegion->m_PtArray;

							pPtArrayEnd = ppPt + p2DSubRegion->m_nPts;

							for(; ppPt < pPtArrayEnd; ppPt++)
							{
								index = *ppPt;

								pPixOut = pIOut->pPix + 3 * index;

								I = 255;  //white

								pPixOut[0] = I;
								pPixOut[1] = I;
								pPixOut[2] = I;

							}
						}
					}
				
				

				

				
			}			
			
			//define dominant wall
			if(ppWalls[0]!=NULL)
			{
				pSegment = ppWalls[0];

				pRelList = pSegment->m_RelList + p2DRegionSet3->m_iRelList[RVLRELLIST_COMPONENTS];
				ppSubRegionStart = (CRVL2DRegion2 **)pRelList->pFirst;
				ppSubRegionEnd = (CRVL2DRegion2 **)pRelList->pEnd;
				nSubSegments = ppSubRegionEnd - ppSubRegionStart;



				for(j=0;j<nSubSegments;j++,ppSubRegionStart++)
				{
					p2DSubRegion = *ppSubRegionStart;
					
					if(bEdgeBasedSegments)
					{
						pPixPtr = (RVL2DREGION_PIXIDX *)(p2DSubRegion->m_PtArray);

						while(pPixPtr)
						{
							index = pPixPtr->iPix;

							pPixOut = pIOut->pPix + 3 * index;

							I = 0;  //black

							pPixOut[0] = I;
							pPixOut[1] = I;
							pPixOut[2] = I;

							pPixPtr = pPixPtr->pNext;
						}
					}
					else
					{
						ppPt = (int *)p2DSubRegion->m_PtArray;

						pPtArrayEnd = ppPt + p2DSubRegion->m_nPts;

						for(; ppPt < pPtArrayEnd; ppPt++)
						{
							index = *ppPt;

							pPixOut = pIOut->pPix + 3 * index;

							I = 0;  //black

							pPixOut[0] = I;
							pPixOut[1] = I;
							pPixOut[2] = I;

						}
					}
				}
			
			}
		}
		else  //bsupersegments
		{
			//define ground
			if(pGround != NULL)
			{
				pSegment = pGround;

				if(bEdgeBasedSegments)
				{
					pPixPtr = (RVL2DREGION_PIXIDX *)(pSegment->m_PtArray);

					while(pPixPtr)
					{
						index = pPixPtr->iPix;

						pPixOut = pIOut->pPix + 3 * index;

						I = 255;  //white

						pPixOut[0] = I;
						pPixOut[1] = I;
						pPixOut[2] = I;

						pPixPtr = pPixPtr->pNext;
					}
				}
				else
					{
						ppPt = (int *)pSegment->m_PtArray;

						pPtArrayEnd = ppPt + pSegment->m_nPts;

						for(; ppPt < pPtArrayEnd; ppPt++)
						{
							index = *ppPt;

							pPixOut = pIOut->pPix + 3 * index;

							I = 255;  //white			pIIn->pPix[pPt->iPix]

							pPixOut[0] = I; //		64 + (unsigned char)((((int)I) * 3) >> 2);
							pPixOut[1] = I;
							pPixOut[2] = I;
							
						}
					}
				
			}			
			
			//define dominant wall
			if(ppWalls[0]!=NULL)
			{
				pSegment = ppWalls[0];

				if(bEdgeBasedSegments)
				{
					pPixPtr = (RVL2DREGION_PIXIDX *)(pSegment->m_PtArray);

					while(pPixPtr)
					{
						index = pPixPtr->iPix;

						pPixOut = pIOut->pPix + 3 * index;

						I = 0;  //black	

						pPixOut[0] = I;
						pPixOut[1] = I;
						pPixOut[2] = I;

						pPixPtr = pPixPtr->pNext;
					}
				}
				else
				{
					ppPt = (int *)pSegment->m_PtArray;

					pPtArrayEnd = ppPt + pSegment->m_nPts;

					for(; ppPt < pPtArrayEnd; ppPt++)
					{
						index = *ppPt;

						pPixOut = pIOut->pPix + 3 * index;

						I = 0;  //black			pIIn->pPix[pPt->iPix]

						pPixOut[0] = I; //		64 + (unsigned char)((((int)I) * 3) >> 2);
						pPixOut[1] = I;
						pPixOut[2] = I;
						
					}
				}
			
			}
			
		}
	}
}

//display the first two floor segments if they exist
void RVLPSDDisplayFloorSegments(RVLDISPARITYMAP *pDisparityMap,
					  PIX_ARRAY *pIIn,
					  PIX_ARRAY *pIOut,
					  CRVL2DRegion2 *pGround,
					  CRVLMPtrChain *p2DRegionList)
{

	unsigned char *pPixIn = pIIn->pPix;

	int w = pIIn->Width;
	int h = pIIn->Height;


	unsigned char *pPixOut = pIOut->pPix;

	short int *pd = pDisparityMap->Disparity;


	int u, v;
	unsigned char I;

	//first fill the output image with default values
	for(v = 0; v < h; v++)
	{
		for(u = 0; u < w; u++, pPixIn++,pPixOut += 3)
		{
			I = 128 + (*pPixIn)/2 ;
		
			pPixOut[0] = I;
			pPixOut[1] = I;
			pPixOut[2] = I;
			
		}
	}

	int maxnGroundPts = 0;

	CRVL2DRegion2 *pSegment;
	CRVL2DRegion2 *pGround2 = NULL;

	double absb;

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		absb = fabs(pSegment->m_b);

		if(fabs(pSegment->m_a) < absb && absb > 0.05)
		{
			if((pSegment->m_nPts > maxnGroundPts) && (pSegment != pGround))
			{
				maxnGroundPts = pSegment->m_nPts;

				pGround2 = pSegment;
			}
		}
		
	}

	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;

	//define main ground
	if(pGround != NULL)
	{
		pSegment = pGround;

		ppPt = (RVL3DPOINT2 **)pSegment->m_PtArray;

		pPtArrayEnd = ppPt + pSegment->m_nPts;


		for(; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			pPixOut = pIOut->pPix + 3 * pPt->iPix;

			I = 255;  //white			pIIn->pPix[pPt->iPix]

			pPixOut[0] = I; //		64 + (unsigned char)((((int)I) * 3) >> 2);
			pPixOut[1] = I;
			pPixOut[2] = I;
			
		}
	}		

	//define second ground
	if(pGround2 != NULL)
	{
		pSegment = pGround2;

		ppPt = (RVL3DPOINT2 **)pSegment->m_PtArray;

		pPtArrayEnd = ppPt + pSegment->m_nPts;


		for(; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			pPixOut = pIOut->pPix + 3 * pPt->iPix;

			I = 0;  //white			pIIn->pPix[pPt->iPix]

			pPixOut[0] = I; //		64 + (unsigned char)((((int)I) * 3) >> 2);
			pPixOut[1] = I;
			pPixOut[2] = I;
			
		}
	}		


}



void RVLPSDDrawLine(int u0, int v0, int u1, int v1,PIX_ARRAY *pIPic, bool bIsRef, int u0Ref, int u1Ref)
{
	int u, v, b, iPix,iPix1;
	unsigned char I;
	double a;

	int w = pIPic->Width;
	int h = pIPic->Height;


	//thickness *2
	unsigned char *pPix;
	unsigned char *pPix1;

	int uHelper;

	//check
	if (u0Ref > u1Ref)
	{
		uHelper = u1Ref;
		u1Ref = u0Ref;
		u0Ref = uHelper;
	}

	if(u0Ref == 0 && u1Ref == 0)
		u1Ref = w-1;



	a = (v1 - v0)*1.0 / ((u1 - u0)*1.0);
	
	b = DOUBLE2INT(v1 - (a * u1));

	I = bIsRef ? 255 : 0;

	for(u = u0Ref; u <= u1Ref; u++)
	{
		v = DOUBLE2INT((a * u) + b);

		iPix = u + v * w;
		iPix1 = u + (v + 1) * w;

		pPix = pIPic->pPix + 3 * iPix;
		pPix1 = pIPic->pPix + 3 * iPix1;
	
		pPix[0] = I;
		pPix[1] = I;
		pPix[2] = I;

		pPix1[0] = I;
		pPix1[1] = I;
		pPix1[2] = I;
	}
}



void RVLUpdate2DPolygonAreaAndCentroid(double xi, double yi,
									   double xip, double yip,
									   double &Area,
									   double &Xcentroid, double &Ycentroid)
{
	double ai = xi*yip - xip*yi;;

	Area += ai;

	Xcentroid +=((xi + xip)*ai);
	Ycentroid +=((yi + yip)*ai);
}

void RVLUpdate2DPolygonMoments(double xi, double yi,
							   double xip, double yip,
							   double &Xmoment, double &Ymoment,
							   double &XYmoment)
{
	double ai = xi*yip - xip*yi;;

	Xmoment += ((yi*yi + yi*yip + yip*yip)*ai);
	Ymoment += ((xi*xi + xi*xip + xip*xip)*ai);
	XYmoment += ((xi*yip + 2*xi*yi + 2*xip*yip + xip*yi)*ai);
}


void RVLPSDDisplayLEVEL2Regions(CRVLFigure *pFig,
								CRVLC2D *p2DRegionSet,
								int ImageWidth,
								RVLCOLOR Color)
{	
	CRVL2DRegion2 *p2DRegion, *p2DRegionNeighbour;

	CRVL2DRegion2 *p2DRegionLevel2, *p2DRegionNeghbourLevel2;
	CRVL2DRegion2 *p2DRegionLevel3, *p2DRegionNeghbourLevel3;

	RVLMESH_LINK *pLink, *pLink0, *pLinkNeighbour;

	CRVLDisplayVector Vector(pFig->m_pMem);

	Vector.m_bClosed = FALSE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector.m_rL = Color.r;
	Vector.m_gL = Color.g;
	Vector.m_bL = Color.b;

	CRVLDisplayVector *pVector;

	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(p2DRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(p2DRegion->m_PtArray);

		do
		{

			pLinkNeighbour = pLink->pOpposite;
			p2DRegionNeighbour = (CRVL2DRegion2 *)(pLinkNeighbour->vp2DRegion);

			//if ((p2DRegionNeighbour->m_Flags & RVLOBJ2_FLAG_REJECTED)==0) && ((p2DRegionNeighbour->m_Flags & RVL2DREGION_FLAG_MARKED3)==0))

			if(p2DRegion)
			{
				p2DRegionLevel2 = *(CRVL2DRegion2 **)(p2DRegion->m_pData + p2DRegionSet->m_iDataParentPtr);
				p2DRegionLevel3 = *(CRVL2DRegion2 **)(p2DRegion->m_pData + p2DRegionSet->m_iDataGrandParentPtr);
				
				if(p2DRegionLevel2 == NULL)
					p2DRegionLevel2 = p2DRegion;
				
				if(p2DRegionLevel3 == NULL)
					p2DRegionLevel3 = p2DRegion;
				
			}
			

			if(p2DRegionNeighbour)
			{
				p2DRegionNeghbourLevel2 = *(CRVL2DRegion2 **)(p2DRegionNeighbour->m_pData + p2DRegionSet->m_iDataParentPtr);
				p2DRegionNeghbourLevel3 = *(CRVL2DRegion2 **)(p2DRegionNeighbour->m_pData + p2DRegionSet->m_iDataGrandParentPtr);

				if(p2DRegionNeghbourLevel2 == NULL)
					p2DRegionNeghbourLevel2 = p2DRegionNeighbour;

				if(p2DRegionNeghbourLevel3 == NULL)
					p2DRegionNeghbourLevel3 = p2DRegionNeighbour;
			}
			else
			{
				//	continue;
				p2DRegionNeghbourLevel2 = NULL;
				p2DRegionNeghbourLevel3 = NULL;
			}

			if((p2DRegionLevel2 != p2DRegionNeghbourLevel2) && (p2DRegionLevel3 == p2DRegionNeghbourLevel3))
			{
				pVector = pFig->AddVector(&Vector);
			
				pVector->Line(((pLinkNeighbour->iPix0 % ImageWidth) << 1),
							 ((pLinkNeighbour->iPix0 / ImageWidth) << 1),
							 ((pLink->iPix0 % ImageWidth) << 1),
							 ((pLink->iPix0 / ImageWidth) << 1));

			}
			

			pLink = pLink->pNext->pOpposite;

		}
		while(pLink != pLink0);


		
	
			

		
	}
}


//OBSOLETE!!! Will be deleted in the future
void RVLPSDDisplayLEVEL2Desc(CRVLFigure *pFig,
								   CRVLC2D *p2DRegionSet,
								   int ImageWidth,
								   RVLCOLOR Color,
								   RVLKINECT_PARAMS kinectParams)
{	
	CRVL2DRegion2 *p2DRegion; //, *p2DRegionNeighbour;

	//CRVL2DRegion2 *p2DRegionLevel2, *p2DRegionNeghbourLevel2;
	//CRVL2DRegion2 *p2DRegionLevel3, *p2DRegionNeghbourLevel3;

	CRVL3DSurface2 *p3DSurface;

	CRVLDisplayVector *pVector;

    double mainAxis[3], minorAxis[3];

	//double ptC[2][3];
	double ptRect[4][3];//, ptRectTrans[4][3];

	int ptUVD[4][3];

	
	CvMat *dMatRot = cvCreateMatHeader(3, 3, CV_64FC1);
	
	int uvd[3];
	
	double P[3];
	CvMat *dMatP = cvCreateMatHeader(3, 1, CV_64FC1);
	dMatP->data.db = P;

	CvMat *dMatRect = cvCreateMatHeader(3, 1, CV_64FC1);

	int i;

	CRVLDisplayVector Vector(pFig->m_pMem);

	Vector.m_bClosed = FALSE;
	Vector.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_SQUARE;
	Vector.m_rL = Color.r;
	Vector.m_gL = Color.g;
	Vector.m_bL = Color.b;


	CRVLDisplayVector Vector1(pFig->m_pMem);

	Vector1.m_bClosed = FALSE;
	Vector1.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector1.m_rL = 255;
	Vector1.m_gL = 255;
	Vector1.m_bL = 0;

	CRVLDisplayVector Vector2(pFig->m_pMem);

	Vector2.m_bClosed = FALSE;
	Vector2.m_PointType = RVLGUI_POINT_DISPLAY_TYPE_NONE;
	Vector2.m_rL = 0;
	Vector2.m_gL = 255;
	Vector2.m_bL = 0;


	int nRegions = 0;
	int nFalsePoints = 0;

	CRVLMPtrChain *p2DRegionList = &(p2DRegionSet->m_ObjectList);

	p2DRegionList->Start();

	while(p2DRegionList->m_pNext)
	{
		p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

		if(p2DRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;


		nRegions++;

		p3DSurface = (CRVL3DSurface2 *)(p2DRegion->m_vp3DSurface);

		if(p3DSurface == NULL)
			continue;
		
		for(i=0;i<3;i++)
		{
			/*ptC[0][i] = p3DSurface->m_Pose.m_X[i] + p3DSurface->m_EigenValues[0] * p3DSurface->m_Pose.m_Rot[i*3]; 
			ptC[1][i] = p3DSurface->m_Pose.m_X[i] - p3DSurface->m_EigenValues[0] * p3DSurface->m_Pose.m_Rot[i*3];*/

			mainAxis[i] = p3DSurface->m_EigenValues[0] * p3DSurface->m_Pose.m_Rot[3*i];
			minorAxis[i] = p3DSurface->m_EigenValues[1] * p3DSurface->m_Pose.m_Rot[3*i + 1];

			ptRect[0][i] = p3DSurface->m_Pose.m_X[i] + mainAxis[i];
			ptRect[1][i] = p3DSurface->m_Pose.m_X[i] - mainAxis[i];
			ptRect[2][i] = p3DSurface->m_Pose.m_X[i] + minorAxis[i];
			ptRect[3][i] = p3DSurface->m_Pose.m_X[i] - minorAxis[i];
		}

		//dMatRot->data.db = p3DSurface->m_Pose.m_Rot;

		for(i=0;i<4;i++)
		{
			//dMatRect->data.db = ptRect[i];
			//cvGEMM(dMatRot,dMatRect,1,NULL,1,dMatP);
			//RVLSum3D(P,p3DSurface->m_Pose.m_X,ptRectTrans[i]);
			
			
			RVLGetKinect2DData(ptUVD[i],ptRect[i],kinectParams);
		}
		

		//pVector->Line(u1,v1,u2,v2);

		//for(i=1;i<3;i++)
		//{
			pVector = pFig->AddVector(&Vector1);
			
			//main axis
			pVector->Line((ptUVD[0][0] << 1), 
   						  (ptUVD[0][1] << 1),
						  (ptUVD[1][0] << 1), 
						  (ptUVD[1][1] << 1));

			
			pVector = pFig->AddVector(&Vector2);
			
			
			//minor axis
			pVector->Line((ptUVD[2][0] << 1), 
   						  (ptUVD[2][1] << 1),
						  (ptUVD[3][0] << 1), 
						  (ptUVD[3][1] << 1));

			
		//}

		RVLGetKinect2DData(uvd,p3DSurface->m_Pose.m_X,kinectParams);

		pVector = pFig->AddVector(&Vector);
		pVector->Point(uvd[0] << 1,uvd[1] << 1);

		if(uvd[0]<0 || uvd[1]<0)
			nFalsePoints++;
		

		
	}

}

void CRVLPlanarSurfaceDetector::Gen3DMeshObjectHierarchy(CRVL3DMeshObject *pRootMO)
{
	if (!m_MeshSegmentWERNodeArray)
		return;

	//buffers
	RVLSWER_NODE **objBuff;
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSWER_NODE*, m_nMeshSegmentWERNodes, objBuff);
	memset(objBuff, 0, m_nMeshSegmentWERNodes * sizeof(RVLSWER_NODE*));
	RVLSWER_NODE **subObjBuff;
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSWER_NODE*, m_nMeshSegmentWERNodes, subObjBuff);
	memset(subObjBuff, 0, m_nMeshSegmentWERNodes * sizeof(RVLSWER_NODE*));
	//pointers
	int objP[2] = {0, 0};
	int subObjP[2] = {0, 0};
	//first object
	//objBuff[0] = &m_MeshSegmentWERNodeArray[m_nMeshSegmentWERNodes - 1];//m_MeshSegmentWERNodeArray;
	//objP[1]++;
	for (int i = m_nMeshSegmentWERNodes - m_nMeshSegmentWERLevels; i < m_nMeshSegmentWERNodes; i++)
	{
		if (!m_MeshSegmentWERNodeArray[i].pParent)
		{
			objBuff[objP[1]] = &m_MeshSegmentWERNodeArray[i];
			objP[1]++;
		}
	}

	//Creating first mesh object and adding it to root object list
	CRVL3DMeshObject *pChildMO;
	RVLMEM_ALLOC_STRUCT(m_pMem, CRVL3DMeshObject, pChildMO);
	pChildMO->m_pClass = pRootMO->m_pClass;
	pChildMO->Init();
	pChildMO->InitChild();
	pChildMO->parentMeshObject = pRootMO;
	pChildMO->rootMeshObject = pRootMO;					
	RVLQLIST_PTR_ENTRY *pElement;
	//RVLMEM_ALLOC_STRUCT(m_pMem, RVLQLIST_PTR_ENTRY, pElement);
	//pElement->Ptr = pChildMO;
	//RVLQLIST_ADD_ENTRY(pRootMO->m_ChildMeshObjects, pElement);

	RVLSWER_NODE *pWerNode, *pWerSubNode;
	pWerNode = objBuff[0];//&m_MeshSegmentWERNodeArray[m_nMeshSegmentWERNodes - 1];//m_MeshSegmentWERNodeArray;
	CRVL2DRegion2* pTriangle;
	//outer loop per object(segment)
	while(pWerNode)
	{
		if (!pWerNode->pChild[0]) //Next node(left) doesn't exists???
		{
		}
		else if (pWerNode->pChild[0]->Flags & RVLSWER_NODE_FLAG_OBJECT)		//Next node(left) is object???
		{
			objBuff[objP[1]] = pWerNode->pChild[0];
			objP[1]++;
		}
		else if (pWerNode->pChild[0]->pChild[1] == NULL)	//Next node(left) is triangle
		{
			if (pWerNode->pChild[0]->pChild[0])
			{
				pTriangle = (CRVL2DRegion2*)pWerNode->pChild[0]->pChild[0];
				pTriangle->m_Label = objP[0];

				RVLMEM_ALLOC_STRUCT(m_pMem, RVLQLIST_PTR_ENTRY, pElement);
				pElement->Ptr = (CRVL2DRegion2*)pWerNode->pChild[0]->pChild[0];
				RVLQLIST_ADD_ENTRY(pChildMO->m_FaceList, pElement);
				pChildMO->m_noFaces++;
			}
		}
		else	//Next node(left) is subobject!
		{
			subObjBuff[subObjP[1]] = pWerNode->pChild[0];
			subObjP[1]++;
		}

		if (!pWerNode->pChild[1]) //Next node(right) doesn't exists???
		{
		}
		else if (pWerNode->pChild[1]->Flags & RVLSWER_NODE_FLAG_OBJECT)		//Next node(right) is object???
		{
			objBuff[objP[1]] = pWerNode->pChild[1];
			objP[1]++;
		}
		else if (pWerNode->pChild[1]->pChild[1] == NULL)	//Next node(right) is triangle???
		{
			if (pWerNode->pChild[1]->pChild[0])
			{
				pTriangle = (CRVL2DRegion2*)pWerNode->pChild[1]->pChild[0];
				
				pTriangle->m_Label = objP[0];
				RVLMEM_ALLOC_STRUCT(m_pMem, RVLQLIST_PTR_ENTRY, pElement);
				pElement->Ptr = (CRVL2DRegion2*)pWerNode->pChild[1]->pChild[0];
				RVLQLIST_ADD_ENTRY(pChildMO->m_FaceList, pElement);
				pChildMO->m_noFaces++;
			}
		}
		else	//Next node(right) is subobject!
		{
			subObjBuff[subObjP[1]] = pWerNode->pChild[1];
			subObjP[1]++;
		}

		//inner loop per subobject
		pWerSubNode = subObjBuff[subObjP[0]];
		while(pWerSubNode)
		{
			if (subObjP[1] == 0)
				break;
			if (!pWerSubNode->pChild[0]) //Next node(left) doesn't exists???
			{
			}
			else if (pWerSubNode->pChild[0]->Flags & RVLSWER_NODE_FLAG_OBJECT)		//Next node(left) is object???
			{
				objBuff[objP[1]] = pWerSubNode->pChild[0];
				objP[1]++;
			}
			else if (pWerSubNode->pChild[0]->pChild[1] == NULL)	//Next node(left) is triangle
			{
				if (pWerSubNode->pChild[0]->pChild[0])
				{
					pTriangle = (CRVL2DRegion2*)pWerSubNode->pChild[0]->pChild[0];
					
					pTriangle->m_Label = objP[0];
					RVLMEM_ALLOC_STRUCT(m_pMem, RVLQLIST_PTR_ENTRY, pElement);
					pElement->Ptr = (CRVL2DRegion2*)pWerSubNode->pChild[0]->pChild[0];
					RVLQLIST_ADD_ENTRY(pChildMO->m_FaceList, pElement);
					pChildMO->m_noFaces++;
				}
			}
			else	//Next node(left) is subobject!
			{
				subObjBuff[subObjP[1]] = pWerSubNode->pChild[0];
				subObjP[1]++;
			}

			if (!pWerSubNode->pChild[1]) //Next node(right) doesn't exists???
			{
			}
			else if (pWerSubNode->pChild[1]->Flags & RVLSWER_NODE_FLAG_OBJECT)		//Next node(right) is object???
			{
				objBuff[objP[1]] = pWerSubNode->pChild[1];
				objP[1]++;
			}
			else if (pWerSubNode->pChild[1]->pChild[1] == NULL)	//Next node(right) is triangle???
			{
				if (pWerSubNode->pChild[1]->pChild[0])
				{
					pTriangle = (CRVL2DRegion2*)pWerSubNode->pChild[1]->pChild[0];
					pTriangle->m_Label = objP[0];

					RVLMEM_ALLOC_STRUCT(m_pMem, RVLQLIST_PTR_ENTRY, pElement);
					pElement->Ptr = (CRVL2DRegion2*)pWerSubNode->pChild[1]->pChild[0];
					RVLQLIST_ADD_ENTRY(pChildMO->m_FaceList, pElement);
					pChildMO->m_noFaces++;
				}
			}
			else	//Next node(right) is subobject!
			{
				subObjBuff[subObjP[1]] = pWerSubNode->pChild[1];
				subObjP[1]++;
			}
			
			//going to next subnode in buffer
			subObjP[0]++;
			if (subObjP[0] == subObjP[1])
				break;
			else
				pWerSubNode = subObjBuff[subObjP[0]];
		}

		//if there are faces in object
		if (pChildMO->m_noFaces > 0)
		{
			RVLMEM_ALLOC_STRUCT(m_pMem, RVLQLIST_PTR_ENTRY, pElement);
			pElement->Ptr = pChildMO;
			RVLQLIST_ADD_ENTRY(pRootMO->m_ChildMeshObjects, pElement);
		}
		//going to next object/segment in buffer
		objP[0]++;
		if (objP[0] == objP[1])
			break;
		else
		{
			if (pChildMO->m_noFaces > 0)
			{
				RVLMEM_ALLOC_STRUCT(m_pMem, CRVL3DMeshObject, pChildMO);
				pChildMO->m_pClass = pRootMO->m_pClass;
				pChildMO->Init();
				pChildMO->InitChild();
				pChildMO->parentMeshObject = pRootMO;
				pChildMO->rootMeshObject = pRootMO;					
				//RVLMEM_ALLOC_STRUCT(m_pMem, RVLQLIST_PTR_ENTRY, pElement);
				//pElement->Ptr = pChildMO;
				//RVLQLIST_ADD_ENTRY(pRootMO->m_ChildMeshObjects, pElement);
			}

			subObjP[0] = 0;
			subObjP[1] = 0;

			pWerNode = objBuff[objP[0]];
		}
	}
}

void CRVLPlanarSurfaceDetector::AssignLabels(CRVLC2D *pTriangleSetLevel1, CRVLC2D *pTriangleSetLevel3)
{
	CRVLMPtrChain *pSegmentList = &(pTriangleSetLevel3->m_ObjectList);

	CRVL2DRegion2 *pSegment;

	int Label = 0;

	pSegmentList->Start();

	while(pSegmentList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(pSegmentList->GetNext());

		pSegment->m_Label = (Label++);
	}

	int nSegments = Label;

	CRVLMPtrChain *pTriangleList = &(pTriangleSetLevel1->m_ObjectList);

	CRVL2DRegion2 *pTriangle;

	pTriangleList->Start();

	while(pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pSegment = *((CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr));

		if(pSegment)
			pTriangle->m_Label = pSegment->m_Label;
		else
			pTriangle->m_Label = nSegments;
	}
}

void CRVLPlanarSurfaceDetector::GetNeighbors(CRVLC2D *pSegmentSet)
{
	CRVLMPtrChain *pSegmentList = &(pSegmentSet->m_ObjectList);

	CRVL2DRegion2 *pSegment;

	pSegmentList->Start();

	while(pSegmentList->m_pNext)
	{
		pSegment = (CRVL2DRegion2 *)(pSegmentList->GetNext());


	}	// for each segment
}

int CRVLPlanarSurfaceDetector::GenRelListFromWER(CRVLC2D *pTriangleSetLevel1, CRVLC2D *pTriangleSetLevel3)
{
	if (!m_MeshSegmentWERNodeArray)
		return 0;

	//buffers
	RVLSWER_NODE **objBuff;
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSWER_NODE*, m_nMeshSegmentWERNodes, objBuff);
	memset(objBuff, 0, m_nMeshSegmentWERNodes * sizeof(RVLSWER_NODE*));
	RVLSWER_NODE **subObjBuff;
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSWER_NODE*, m_nMeshSegmentWERNodes, subObjBuff);
	memset(subObjBuff, 0, m_nMeshSegmentWERNodes * sizeof(RVLSWER_NODE*));
	//pointers
	int objP[2] = {0, 0};
	int subObjP[2] = {0, 0};
	//first object
	//objBuff[0] = &m_MeshSegmentWERNodeArray[m_nMeshSegmentWERNodes - 1];//m_MeshSegmentWERNodeArray;
	//objP[1]++;
	for (int i = m_nMeshSegmentWERNodes - m_nMeshSegmentWERLevels; i < m_nMeshSegmentWERNodes; i++)
	{
		if (!m_MeshSegmentWERNodeArray[i].pParent)
		{
			objBuff[objP[1]] = &m_MeshSegmentWERNodeArray[i];
			objP[1]++;
		}
	}

	//Bufferi
	CRVL2DRegion2 **GroupedTriangleList31 = (CRVL2DRegion2 **)(m_pMem->Alloc(pTriangleSetLevel1->m_ObjectList.m_nElements*sizeof(CRVL2DRegion2 *))); 
	memset(GroupedTriangleList31, 0x00, pTriangleSetLevel1->m_ObjectList.m_nElements * sizeof(CRVL2DRegion2 *));
	CRVL2DRegion2 **ppGroupedTriangleList = GroupedTriangleList31;

	pTriangleSetLevel3->m_ObjectList.RemoveAll();
	//Creating first segment
	CRVL2DRegion2 *p2DSegmentLevel3 = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(pTriangleSetLevel3));
	RVLQLIST *pSamples = &(p2DSegmentLevel3->m_Samples);
	RVLQLIST_INIT(pSamples)
	RVLARRAY *pRelList = p2DSegmentLevel3->m_RelList + pTriangleSetLevel3->m_iRelList[RVLRELLIST_ELEMENTS];
	p2DSegmentLevel3->m_n3DPts = 0;
	CRVL2DRegion2 **ppGroupedTriangleStart = ppGroupedTriangleList;

	CRVL2DRegion2 *pTriangle;
	int noTriangles = 0;
	int noObjects = 0;

	RVLPTRCHAIN_ELEMENT *pPrevObjPtr = NULL;

	RVLSWER_NODE *pWerNode, *pWerSubNode;
	pWerNode = objBuff[0];//&m_MeshSegmentWERNodeArray[m_nMeshSegmentWERNodes - 1];//m_MeshSegmentWERNodeArray;
	//outer loop per object(segment)
	while(pWerNode)
	{
		if (!pWerNode->pChild[0]) //Next node(left) doesn't exists???
		{
		}
		else if (pWerNode->pChild[0]->Flags & RVLSWER_NODE_FLAG_OBJECT)		//Next node(left) is object???
		{
			objBuff[objP[1]] = pWerNode->pChild[0];
			objP[1]++;
		}
		else if (pWerNode->pChild[0]->pChild[1] == NULL)	//Next node(left) is triangle
		{
			if (pWerNode->pChild[0]->pChild[0])
			{
				pTriangle = (CRVL2DRegion2*)pWerNode->pChild[0]->pChild[0];
				pTriangle->m_Label = objP[0];

				p2DSegmentLevel3->m_n3DPts += pTriangle->m_n3DPts;

				//relate LEVEL3 segment to LEVEL1 triangle
				CRVL2DRegion2 **pp2DSegmentLevel3 = (CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr);
				*pp2DSegmentLevel3 = p2DSegmentLevel3;

				*(ppGroupedTriangleList++) = pTriangle; 

				noTriangles++;
			}
		}
		else	//Next node(left) is subobject!
		{
			subObjBuff[subObjP[1]] = pWerNode->pChild[0];
			subObjP[1]++;
		}

		if (!pWerNode->pChild[1]) //Next node(right) doesn't exists???
		{
		}
		else if (pWerNode->pChild[1]->Flags & RVLSWER_NODE_FLAG_OBJECT)		//Next node(right) is object???
		{
			objBuff[objP[1]] = pWerNode->pChild[1];
			objP[1]++;
		}
		else if (pWerNode->pChild[1]->pChild[1] == NULL)	//Next node(right) is triangle???
		{
			if (pWerNode->pChild[1]->pChild[0])
			{
				pTriangle = (CRVL2DRegion2*)pWerNode->pChild[1]->pChild[0];
				pTriangle->m_Label = objP[0];

				p2DSegmentLevel3->m_n3DPts += pTriangle->m_n3DPts;

				//relate LEVEL3 segment to LEVEL1 triangle
				CRVL2DRegion2 **pp2DSegmentLevel3 = (CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr);
				*pp2DSegmentLevel3 = p2DSegmentLevel3;

				*(ppGroupedTriangleList++) = pTriangle; 

				noTriangles++;
			}
		}
		else	//Next node(right) is subobject!
		{
			subObjBuff[subObjP[1]] = pWerNode->pChild[1];
			subObjP[1]++;
		}

		//inner loop per subobject
		pWerSubNode = subObjBuff[subObjP[0]];
		while(pWerSubNode)
		{
			if (subObjP[1] == 0)
				break;
			if (!pWerSubNode->pChild[0]) //Next node(left) doesn't exists???
			{
			}
			else if (pWerSubNode->pChild[0]->Flags & RVLSWER_NODE_FLAG_OBJECT)		//Next node(left) is object???
			{
				objBuff[objP[1]] = pWerSubNode->pChild[0];
				objP[1]++;
			}
			else if (pWerSubNode->pChild[0]->pChild[1] == NULL)	//Next node(left) is triangle
			{
				if (pWerSubNode->pChild[0]->pChild[0])
				{
					pTriangle = (CRVL2DRegion2*)pWerSubNode->pChild[0]->pChild[0];
					pTriangle->m_Label = objP[0];

					p2DSegmentLevel3->m_n3DPts += pTriangle->m_n3DPts;

					//relate LEVEL3 segment to LEVEL1 triangle
					CRVL2DRegion2 **pp2DSegmentLevel3 = (CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr);
					*pp2DSegmentLevel3 = p2DSegmentLevel3;

					*(ppGroupedTriangleList++) = pTriangle; 

					noTriangles++;
				}
			}
			else	//Next node(left) is subobject!
			{
				subObjBuff[subObjP[1]] = pWerSubNode->pChild[0];
				subObjP[1]++;
			}

			if (!pWerSubNode->pChild[1]) //Next node(right) doesn't exists???
			{
			}
			else if (pWerSubNode->pChild[1]->Flags & RVLSWER_NODE_FLAG_OBJECT)		//Next node(right) is object???
			{
				objBuff[objP[1]] = pWerSubNode->pChild[1];
				objP[1]++;
			}
			else if (pWerSubNode->pChild[1]->pChild[1] == NULL)	//Next node(right) is triangle???
			{
				if (pWerSubNode->pChild[1]->pChild[0])
				{
					pTriangle = (CRVL2DRegion2*)pWerSubNode->pChild[1]->pChild[0];
					pTriangle->m_Label = objP[0];

					p2DSegmentLevel3->m_n3DPts += pTriangle->m_n3DPts;

					//relate LEVEL3 segment to LEVEL1 triangle
					CRVL2DRegion2 **pp2DSegmentLevel3 = (CRVL2DRegion2 **)(pTriangle->m_pData + pTriangleSetLevel1->m_iDataGrandParentPtr);
					*pp2DSegmentLevel3 = p2DSegmentLevel3;

					*(ppGroupedTriangleList++) = pTriangle; 

					noTriangles++;
				}
			}
			else	//Next node(right) is subobject!
			{
				subObjBuff[subObjP[1]] = pWerSubNode->pChild[1];
				subObjP[1]++;
			}
			
			//going to next subnode in buffer
			subObjP[0]++;
			if (subObjP[0] == subObjP[1])
				break;
			else
				pWerSubNode = subObjBuff[subObjP[0]];
		}

		//if there are faces in object
		if (noTriangles > 0)
		{			
				pRelList->pFirst = (unsigned char *)ppGroupedTriangleStart;
				pRelList->pEnd = (unsigned char *)ppGroupedTriangleList;
				noObjects++;
		}
		//going to next object/segment in buffer
		objP[0]++;
		if (objP[0] == objP[1])
			break;
		else
		{
			if (noTriangles > 0)
			{
				if(m_Flags & RVLPSD_FLAG_MM)
					p2DSegmentLevel3->m_std =(pWerNode->Cost <= 100 ? (double)(pWerNode->Cost) : 
						100.0 * exp(RVLLN1p01 * (double)(pWerNode->Cost - 100)));
				pPrevObjPtr = pTriangleSetLevel3->m_ObjectList.m_pLast;
				p2DSegmentLevel3 = (CRVL2DRegion2 *)(RVL2DRegionTemplate.Create3(pTriangleSetLevel3));
				RVLQLIST *pSamples = &(p2DSegmentLevel3->m_Samples);
				RVLQLIST_INIT(pSamples)
				pRelList = p2DSegmentLevel3->m_RelList + pTriangleSetLevel3->m_iRelList[RVLRELLIST_ELEMENTS];
				p2DSegmentLevel3->m_n3DPts = 0;
				ppGroupedTriangleStart = ppGroupedTriangleList;
				noTriangles = 0;
			}

			subObjP[0] = 0;
			subObjP[1] = 0;

			pWerNode = objBuff[objP[0]];
		}
	}

	if(p2DSegmentLevel3->m_n3DPts == 0)
		pTriangleSetLevel3->m_ObjectList.RemoveAt(pPrevObjPtr);
	else if(m_Flags & RVLPSD_FLAG_MM)
		p2DSegmentLevel3->m_std =(pWerNode->Cost <= 100 ? (double)(pWerNode->Cost) : 
			100.0 * exp(RVLLN1p01 * (double)(pWerNode->Cost - 100)));

	return noObjects;
}

void RVLDisplayDistanceTransformMap(int *DTMap,
									RVLAPIX **SampleMap,
									int ImageSize,
									int minDist,
									unsigned char *PixArray)
{
	unsigned char *pPixArrayEnd = PixArray + 3 * ImageSize;

	RVLAPIX **pSamplePtr = SampleMap;
	int *pDist = DTMap;

	unsigned char *pPix;
	unsigned char *pColor;
	int dist, NrmDist;

	for(pPix = PixArray; pPix < pPixArrayEnd; pDist++, pSamplePtr++)
	{
		dist = *pDist;

		//if(dist == 0)
		//{
		//	*(pPix++) = 255;
		//	*(pPix++) = 255;
		//	*(pPix++) = 255;
		//}
		//else 
		if(*pSamplePtr)
		{
			*(pPix++) = 255;
			*(pPix++) = 255;
			*(pPix++) = 255;
		}
		else if(dist <= minDist)
		{
			*(pPix++) = 0;
			*(pPix++) = 0;
			*(pPix++) = 0;
		}
		else 
		{
			NrmDist = dist - minDist - 1;

			pColor = RVLColorMap + 3 * (NrmDist < 64 ? NrmDist : 63);
			*(pPix++) = *(pColor++);
			*(pPix++) = *(pColor++);
			*(pPix++) = *(pColor++);
		}
	}	
}

void RVLResetFlags(CRVLMPtrChain *pObjectList, BYTE Flags)
{
	CRVL2DRegion2 *pObject;

	pObjectList->Start();

	while(pObjectList->m_pNext)
	{
		pObject = (CRVL2DRegion2 *)(pObjectList->GetNext());

		RVLResetFlags((RVLMESH_LINK *)(pObject->m_PtArray), Flags);
	}
}

void CRVLPlanarSurfaceDetector::Get3DPlanarSurfaceBoundary(
	CRVL3DSurface2 * pSurf,
	bool bPC)
{
	RVL3DPOINT3 *pPt;

	RVL3DCONTOUR *pContour = (RVL3DCONTOUR *)(pSurf->m_BoundaryContourList.pFirst);

	RVL3DPOINT2 **Point3DMap = ((CRVL2DRegion2 *)(pSurf->m_vp2DRegion))->m_pPoint3DMap;
	
	int iPix;
	double *X;
	double s;
	double V3Tmp[3];

	while (pContour)
	{
		pPt = (RVL3DPOINT3 *)(pContour->PtList.pFirst);

		while (pPt)
		{
			if (bPC)
			{
				iPix = pPt->P2D[0] + pPt->P2D[1] * m_Width;

				X = Point3DMap[iPix]->XYZ;

				//RVLProject3DPointTo3DPlane(X, pSurf, pPt->P3D);

				s = RVLDOTPRODUCT3(pSurf->m_N, X) - pSurf->m_d;

				RVLSCALE3VECTOR(pSurf->m_N, s, V3Tmp);

				RVLDIF3VECTORS(X, V3Tmp, pPt->P3D);
			}				
			else
				RVLProject2DPointTo3DPlane(pPt->P2D, pPt->P3D, pSurf,
					m_pStereoVision->m_KinectParams.depthUc,
					m_pStereoVision->m_KinectParams.depthVc,
					m_pStereoVision->m_KinectParams.depthFu,
					m_pStereoVision->m_KinectParams.depthFv);

			pPt = (RVL3DPOINT3 *)(pPt->pNext);
		}

		pContour = (RVL3DCONTOUR *)(pContour->pNext);
	}
}


