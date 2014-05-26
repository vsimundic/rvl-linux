// RVL3DLine2.cpp: implementation of the CRVL3DLine2 class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVLC2D.h"
#include "RVLClass.h"
#include "RVL2DLine3.h"
#include "RVL3DLine2.h"

CRVL3DLine2 RVL3DLine2Template;

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL3DLine2::CRVL3DLine2()
{

}

CRVL3DLine2::~CRVL3DLine2()
{

}


CRVLObject2 * CRVL3DLine2::Create2(CRVLClass * pClass)
{
	CRVL3DLine2 *pObject = (CRVL3DLine2 *)(pClass->m_pMem0->Alloc(sizeof(CRVL3DLine2)));

	memcpy(pObject, this, sizeof(CRVL3DLine2));

	pObject->CRVLObject2::Create(pClass);

	pClass->Add(pObject);

	return pObject;		
}

void RVLCreateC3DLine(CRVLClass *pClass)
{
	pClass->m_nRelLists = 4;

	pClass->m_RelListDesc = new RVLRELLIST_DESC[pClass->m_nRelLists];

	pClass->m_RelListDesc[0].index = RVLRELLIST_INDEX_NEIGHBORS;
	pClass->m_RelListDesc[0].type = RVLRELLIST_TYPE_CHAIN;

	pClass->m_RelListDesc[1].index = RVLRELLIST_INDEX_COMPONENTS;
	pClass->m_RelListDesc[1].type = RVLRELLIST_TYPE_CHAIN;

	pClass->m_RelListDesc[2].index = RVLRELLIST_INDEX_SUPEROBJECTS;
	pClass->m_RelListDesc[2].type = RVLRELLIST_TYPE_CHAIN;

	pClass->m_RelListDesc[3].index = RVLRELLIST_INDEX_3DLINE_3D2DLINE;
	pClass->m_RelListDesc[3].type = RVLRELLIST_TYPE_CHAIN;

	pClass->Init();
}

void CRVL3DLine2::UpdateParams()
{
	CRVL3DObject::UpdateParams();	
}

//void CRVL3DLine2::Save(FILE *fp)
//{
//	fprintf(fp, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
//		m_Index,
//		m_X[0][0], m_X[0][1], m_X[0][2],
//		//m_P[0], m_P[1], m_P[3],
//		//W3D1[0], W3D1[1], W3D1[2],
//		1.0, 0.0, 1.0, 0.0, 0.0, 0.0,
//		m_X[1][0], m_X[1][1], m_X[1][2],
//		//m_P[4 + 0], m_P[4 + 1], m_P[4 + 3],
//		//W3D2[0], W3D2[1], W3D2[2]
//		1.0, 0.0, 1.0, 0.0, 0.0, 0.0
//		);	
//}


void CRVL3DLine2::Save(	FILE *fp,
						DWORD Flags)
{
	fwrite(m_X[0], sizeof(double), 3, fp);
	fwrite(m_CX[0], sizeof(double), 3 * 3, fp);
	fwrite(m_X[1], sizeof(double), 3, fp);
	fwrite(m_CX[1], sizeof(double), 3 * 3, fp);
	fwrite(&m_nSupport, sizeof(int), 1, fp);
}

void CRVL3DLine2::Load(	FILE *fp,
						DWORD Flags)
{
	fread(m_X[0], sizeof(double), 3, fp);
	fread(m_CX[0], sizeof(double), 3 * 3, fp);
	fread(m_X[1], sizeof(double), 3, fp);
	fread(m_CX[1], sizeof(double), 3 * 3, fp);
	fread(&m_nSupport, sizeof(int), 1, fp);
}

//void CRVL3DLine2::TransfLA(CRVL3DPose *pPoseLA)
//{
//	if(m_ParamFlags & RVL3DLINE_PARAM_FLAG_XL)
//	{
//		pPoseLA->Transf(m_XL[0], m_X[0]);
//		pPoseLA->Transf(m_XL[1], m_X[1]);
//
//		m_ParamFlags |= RVL3DLINE_PARAM_FLAG_X;
//	}
//
//	if(m_ParamFlags & RVL3DLINE_PARAM_FLAG_CXL)
//	{
//		pPoseLA->RotCov(m_CXL[0], m_CX[0]);
//		pPoseLA->RotCov(m_CXL[1], m_CX[1]);
//
//		m_ParamFlags |= RVL3DLINE_PARAM_FLAG_CX;
//	}
//}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 

//void RVL3DLinesTransfLA(CRVLMPtrChain *p3DLineList, 
//						  CRVL3DPose *pPoseLA)
//{
//	CRVL3DLine2 *p3DLine;
//
//	p3DLineList->Start();
//
//	while(p3DLineList->m_pNext)
//	{
//		p3DLine = (CRVL3DLine2 *)(p3DLineList->GetNext());
//
//		p3DLine->TransfLA(pPoseLA);
//	}
//}

// Given 2 line segments S1 = {X | X = X1 + s * U1, -l1 <= s <= l1} and
// S2 = {X | X = X2 + s * U2, -l2 <= s <= l2} the function computes
// the closest points Xc1 and Xc2 of these two line segments

void RVL3DLineClosestPoint(double *X1,
						   double *U1,
						   double l1,
						   double *X2, 
						   double *U2, 
						   double l2,
						   double *Xc1,
						   double *Xc2)
{		
	// dX = X2 - X1

	double dX[3];

	RVLDif3D(X2, X1, dX);

	// a_i = dX'* U_i, i=1,2

	RVL3DLINE_CLOSEST_POINTS_DATA Data[2];

	Data[0].l = l1;
	Data[1].l = l2;

	Data[0].a = RVLDotProduct(dX, U1);
	Data[1].a = RVLDotProduct(dX, U2);

	// c = U1' * U2

	double c = RVLDotProduct(U1, U2);

	if(1.0 - fabs(c) < 1e-12)		// if the L1 and L2 are parallel
	{
		double a, a1, a2, s1, s2;
		int i;

		for(i = 0; i < 2; i++)
		{
			a = (i == 0 ? Data[i].a : -Data[i].a);

			a1 = a - l2;
			a2 = a + l2;

			if(a1 > l1)
				Data[i].Sc = l1;
			else if(a2 < -l1)
				Data[i].Sc = -l1;
			else
			{
				if(a2 < l1)
					s1 = a2;
				else
					s1 = l1;

				if(a1 > -l1)
					s2 = a1;
				else
					s2 = -l1;

				Data[i].Sc = 0.5 * (s1 + s2);
			}
		}
	}
	else	// if the L1 and L2 are not parallel
	{
		// compute closest points on the infinite 3D lines 
		// containg S1 and S2

		double k = 1 / (1 - c * c);

		Data[0].SIS =  k * (Data[0].a - Data[1].a * c);
		Data[1].SIS = -k * (Data[1].a - Data[0].a * c);

		// compute closest points of each line segment to the
		// infinite line containing the other segment

		RVL3DLINE_CLOSEST_POINTS_DATA *pDataEnd = Data + 1;

		RVL3DLINE_CLOSEST_POINTS_DATA *pData;

		for(pData = Data; pData <= pDataEnd; pData++)
			if(pData->SIS < -pData->l)
				pData->Sc = -pData->l;
			else if(pData->SIS > pData->l)
				pData->Sc = pData->l;
			else
				pData->Sc = pData->SIS;

		// compute the closest points L1 and L2

		if(fabs(Data[0].Sc - Data[0].SIS) <= fabs(Data[1].Sc - Data[1].SIS))
		{
			Data[0].Sc = Data[0].a + c * Data[1].Sc;
			pData = Data;
		}
		else
		{
			Data[1].Sc = -Data[1].a + c * Data[0].Sc;
			pData = Data + 1;
		}	

		if(pData->Sc < -pData->l)
			pData->Sc = -pData->l;
		else if(pData->Sc > pData->l)
			pData->Sc = pData->l; 
	}

	double s = Data[0].Sc;

	Xc1[0] = X1[0] + s * U1[0];
	Xc1[1] = X1[1] + s * U1[1];
	Xc1[2] = X1[2] + s * U1[2];

	s = Data[1].Sc;

	Xc2[0] = X2[0] + s * U2[0];
	Xc2[1] = X2[1] + s * U2[1];
	Xc2[2] = X2[2] + s * U2[2];
}


BYTE RVLCrop3DLine(	double *X1Src, double *X2Src,
					CRVLCamera *pCamera,
					RVLRECT *pROI,
					double minz,
					double minr,
					BYTE *bOutLT,					
					int *iU1, int *iU2,
					CvPoint *pTgtPt1,
					CvPoint *pTgtPt2,
					BYTE &CropSide)
{
	BYTE bOut = ((BYTE)(X1Src[2] < minz) << 3) | ((BYTE)(X2Src[2] < minz) << 4);

	double XBuff[3];
	double *X1, *X2;

	if(bOut)
	{
		if(bOut == 0x18)
			return 0x1c;

		double s = (minz - X1Src[2]) / (X2Src[2] - X1Src[2]);

		XBuff[0] = X1Src[0] + s * (X2Src[0] - X1Src[0]);
		XBuff[1] = X1Src[1] + s * (X2Src[1] - X1Src[1]);
		XBuff[2] = minz;

		if(bOut & 0x08)
		{
			X1 = XBuff;
			X2 = X2Src;
		}
		else
		{
			X1 = X1Src;
			X2 = XBuff;
		}
	}
	else
	{
		X1 = X1Src;
		X2 = X2Src;
	}

	double U[2];

	pCamera->Project3DPoint(X1, U, iU1);
	pCamera->Project3DPoint(X2, U, iU2);

	return RVLCrop2DLine(iU1[0], iU1[1], iU2[0], iU2[1], pROI, bOutLT, pTgtPt1, pTgtPt2, CropSide) | bOut; 
}

BYTE RVLCrop3DLine(	double *X1Src, double *X2Src,
					double *A,
					double *tCM,
					RVLRECT *pROI,
					double minz,
					double minr,
					BYTE *bOutLT,					
					int *iU1, int *iU2,
					CvPoint *pTgtPt1,
					CvPoint *pTgtPt2,
					BYTE &CropSide)
{
	BYTE bOut = ((BYTE)(X1Src[2] < minz) << 3) | ((BYTE)(X2Src[2] < minz) << 4);

	double XBuff[3];
	double *X1, *X2;

	if(bOut)
	{
		if(bOut == 0x18)
			return 0x1c;

		double s = (minz - X1Src[2]) / (X2Src[2] - X1Src[2]);

		XBuff[0] = X1Src[0] + s * (X2Src[0] - X1Src[0]);
		XBuff[1] = X1Src[1] + s * (X2Src[1] - X1Src[1]);
		XBuff[2] = minz;

		if(bOut & 0x08)
		{
			X1 = XBuff;
			X2 = X2Src;
		}
		else
		{
			X1 = X1Src;
			X2 = XBuff;
		}
	}
	else
	{
		X1 = X1Src;
		X2 = X2Src;
	}

	int U1[2], U2[2];
	double tmp3x1[3];
	double XC[3];

	RVLDIF3VECTORS(X1, tCM, tmp3x1)
	RVLMULMX3X3VECT(A, tmp3x1, XC)

	U1[0] = (DOUBLE2INT(XC[0] / XC[2]) << 1) + 1;
	U1[1] = (DOUBLE2INT(XC[1] / XC[2]) << 1) + 1;

	RVLDIF3VECTORS(X2, tCM, tmp3x1)
	RVLMULMX3X3VECT(A, tmp3x1, XC)

	U2[0] = (DOUBLE2INT(XC[0] / XC[2]) << 1) + 1;
	U2[1] = (DOUBLE2INT(XC[1] / XC[2]) << 1) + 1;

	return RVLCrop2DLine(U1[0], U1[1], U2[0], U2[1], pROI, bOutLT, pTgtPt1, pTgtPt2, CropSide) | bOut; 
}


// The mathematics for the following function is given in RVMath.doc

BOOL RVL3DLineClosestPoints(double *X01,
							double *V1,
							double *X02,
							double *V2,
							double *X1,
							double *X2)
{
	double a11 = RVLDotProduct(V1, V1);
	double a12 = -RVLDotProduct(V1, V2);
	double a22 = RVLDotProduct(V2, V2);

	double det = a11 * a22 - a12 * a12;

	if(det > -APPROX_ZERO && det < APPROX_ZERO)
		return FALSE;

	double dX0[3];

	dX0[0] = X02[0] - X01[0];
	dX0[1] = X02[1] - X01[1];
	dX0[2] = X02[2] - X01[2];

	double b1 = RVLDotProduct(dX0, V1);
	double b2 = -RVLDotProduct(dX0, V2);

	double s1 = (a22 * b1 - a12 * b2) / det;
	double s2 = (-a12 * b1 + a11 * b2) / det;

	X1[0] = X01[0] + s1 * V1[0];
	X1[1] = X01[1] + s1 * V1[1];
	X1[2] = X01[2] + s1 * V1[2];
			 
	X2[0] = X02[0] + s2 * V2[0];
	X2[1] = X02[1] + s2 * V2[1];
	X2[2] = X02[2] + s2 * V2[2];

	return TRUE;
}


void RVL3DLineEKFUpdate(	double *XSA,
							double varxST,
							double *XMB,
							double varxMT,
							CRVL3DPose *pInitPose,
							double *xTB,
							CRVL3DPose *pFinalPose)
{
	double *R0 = pInitPose->m_Rot;
	double *t0 = pInitPose->m_X;
	double q0[3];
	q0[0] = pInitPose->m_Alpha;
	q0[1] = pInitPose->m_Beta;
	q0[2] = pInitPose->m_Theta;
	double cs = pInitPose->m_ca;
	double sn = pInitPose->m_sa;
	double *Pqq0 = pInitPose->m_C;
	double *Pqt0 = pInitPose->m_C + 3 * 3;
	double *Ptt0 = pInitPose->m_C + 2 * 3 * 3;

	double *R = pFinalPose->m_Rot;
	double *t = pFinalPose->m_X;
	double *Pqq = pFinalPose->m_C;
	double *Pqt = pFinalPose->m_C + 3 * 3;
	double *Ptt = pFinalPose->m_C + 2 * 3 * 3;	

	// e = xTB' * (R0 * XSA + t0 - XM)

	double XSB[3];
	double V3x1tmp[3];

	RVLMULMX3X3VECT(R0, XSA, XSB)
	RVLSUM3VECTORS(XSB, t0, XSB)
	RVLDIF3VECTORS(XSB, XMB, V3x1tmp)

	double e = RVLDOTPRODUCT3(xTB, V3x1tmp);

	// J = d(R0([alpha, beta, theta]) * XSA) / d([alpha, beta, theta])

	double J[3 * 3];

	RVLJACOBIANPTRTOROTX(XSA, R0, cs, sn, XSB, J)

	// C = xTB' * J

	double C[3];

	RVLMULMX3X3TVECT(J, xTB, C)

	// P * C' = [PC1; PC2] = P0 * [C'; xTB]

	double PC1[3];

	RVLMULMX3X3VECT(Pqq0, C, PC1)
	RVLMULMX3X3VECT(Pqt0, xTB, V3x1tmp)
	RVLSUM3VECTORS(PC1, V3x1tmp, PC1)

	double PC2[3];

	RVLMULMX3X3TVECT(Pqt0, C, PC2)
	RVLMULMX3X3VECT(Ptt0, xTB, V3x1tmp)
	RVLSUM3VECTORS(PC2, V3x1tmp, PC2)

	// Q = varxST + varxMT + [C xTB'] * PC'

	double Q = varxST + varxMT + RVLDOTPRODUCT3(C, PC1) + RVLDOTPRODUCT3(xTB, PC2);

	// K = [K1; K2] = 1/Q * PC'

	double K1[3];

	RVLSCALE3VECTOR2(PC1, Q, K1)

	double K2[3];

	RVLSCALE3VECTOR2(PC2, Q, K2)

	// w = [q; t] = w0 - K * e

	double q[3];

	RVLSCALE3VECTOR(K1, e, V3x1tmp)

	RVLDIF3VECTORS(q0, V3x1tmp, q)	

	RVLSCALE3VECTOR(K2, e, V3x1tmp)

	RVLDIF3VECTORS(t0, V3x1tmp, t)

	pFinalPose->m_Alpha = q[0];
	pFinalPose->m_Beta = q[1];
	pFinalPose->m_Theta = q[2];
	pFinalPose->UpdateRotLL();
	pFinalPose->m_sa = sin(pFinalPose->m_Alpha);
	pFinalPose->m_ca = cos(pFinalPose->m_Alpha);

	double *invt = (double *)(pFinalPose->m_pData);
	RVLMULMX3X3TVECT(R, t, invt);

	// P = P0 - K * C * P0 = P0 - K * (P0 * C')'

	double M3x3tmp[3 * 3];

	RVLMULVECT3VECT3T(K1, PC1, M3x3tmp)
	RVLDIFMX3X3(Pqq0, M3x3tmp, Pqq)
	RVLMULVECT3VECT3T(K1, PC2, M3x3tmp)
	RVLDIFMX3X3(Pqt0, M3x3tmp, Pqt)
	RVLMULVECT3VECT3T(K2, PC2, M3x3tmp)
	RVLDIFMX3X3(Ptt0, M3x3tmp, Ptt)
}