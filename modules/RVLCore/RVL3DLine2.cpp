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

void CRVL3DLine2::Transform(CRVL3DLine2 *pLineSrc,
							CRVL3DPose *pPose)
{
	double *R = pPose->m_Rot;
	double *t = pPose->m_X;

	RVL3DLINE_EXTENDED_DATA *pData = (RVL3DLINE_EXTENDED_DATA *)m_pData;

	double *PSrc, *PTgt1, *PTgt2;

	PTgt1 = m_X[0];
	PSrc = pLineSrc->m_X[0];
	RVLTRANSF3(PSrc, R, t, PTgt1)
	PTgt2 = m_X[1];
	PSrc = pLineSrc->m_X[1];
	RVLTRANSF3(PSrc, R, t, PTgt2)

	double *dX = pData->dX;
	double *V = pData->V;

	RVLDIF3VECTORS(PTgt2, PTgt1, dX)
	double len = sqrt(RVLDOTPRODUCT3(dX, dX));
	RVLSCALE3VECTOR2(dX, len, V)
	pData->len = len;

	double *CSrc, *CTgt;
	double M3x3Tmp[9];
	
	CTgt = m_CX[0];
	CSrc = pLineSrc->m_CX[0];
	RVLCOV3DTRANSF(CSrc, R, CTgt, M3x3Tmp)
	CTgt = m_CX[1];
	CSrc = pLineSrc->m_CX[1];
	RVLCOV3DTRANSF(CSrc, R, CTgt, M3x3Tmp)
}

void CRVL3DLine2::ComputeMatchParams(double w1,
									 double dw,
									 double wo,
									 double *C1,
									 double *C2,
									 double *Co,
									 double &s,
									 double &varzo)
{
	s = (wo - w1) / dw;
		
	double fTmp = 1.0 - s;
	fTmp *= fTmp;
	double fTmp2 = s * s;
	Co[0] = fTmp * C1[0] + fTmp2 * C2[0];
	Co[1] = fTmp * C1[1] + fTmp2 * C2[1];
	Co[3] = fTmp * C1[4] + fTmp2 * C2[4];
	varzo = fTmp * C1[8] + fTmp2 * C2[8];
}

bool CRVL3DLine2::ComputeOrientUncert(double *C1o,
									  double *C2o,
									  double varz1o,
									  double varz2o,
									  double dwo,
									  double *Cu)
{
	double fTmp = dwo - 3.0 * sqrt(varz1o);

	if(fTmp / dwo < 0.1)
		return false;

	//fTmp *= fTmp;		// Version before ECMR 2019

	double fTmp2 = dwo - 3.0 * sqrt(varz2o);

	if(fTmp2 / dwo < 0.1)
		return false;

	//fTmp2 *= fTmp2;	// Version before ECMR 2019

	// Version before ECMR 2019

	//Cu[0] = C1o[0] / fTmp + C2o[0] / fTmp2;
	//Cu[1] = C1o[1] / fTmp + C2o[1] / fTmp2;
	//Cu[3] = C1o[3] / fTmp + C2o[3] / fTmp2;

	// Version used in ECMR 2019

	fTmp = dwo * dwo;

	Cu[0] = C1o[0] / fTmp + C2o[0] / fTmp;
	Cu[1] = C1o[1] / fTmp + C2o[1] / fTmp;
	Cu[3] = C1o[3] / fTmp + C2o[3] / fTmp;

	return true;
}

bool CRVL3DLine2::Match(	CRVL3DObject *pObject_, 
							RVL3DLINE2_MATCH_DATA *pMatchData,
							double *tMS,
							double &MatchQuality,
							DWORD Flags)
{
	double minrOverlap = 0.4;
	//double minrOverlap = -10.0;

	RVL3DLINE_EXTENDED_DATA *pData = (RVL3DLINE_EXTENDED_DATA *)m_pData;

	// coarse orientation matching

	CRVL3DLine2 *pLine_ = (CRVL3DLine2 *)pObject_;

	RVL3DLINE_EXTENDED_DATA *pData_ = (RVL3DLINE_EXTENDED_DATA *)(pLine_->m_pData);

	double *V = pData->V;
	double *V_ = pData_->V;

	if(RVLDOTPRODUCT3(V, V_) < COS45)
		return false;

	// endpoints

	double *P1C = m_X[0];
	double *P2C = m_X[1];

	double *P1C_ = pLine_->m_X[0];
	double *P2C_ = pLine_->m_X[1];

	// z-axis of the match reference frame

	double *dP = pData->dX;
	double *dP_ = pData_->dX;

	double fTmp;

	double RCL[3*3];
	double tCL[3];
	double tLC[3];

	double *XLC = RCL;
	double *YLC = RCL + 3;
	double *ZLC = RCL + 6;

	RVLSUM3VECTORS(dP, dP_, ZLC)
	RVLNORM3(ZLC, fTmp)

	// overlapping

	double w1 = RVLDOTPRODUCT3(P1C, ZLC);
	double w2 = RVLDOTPRODUCT3(P2C, ZLC);
	double w1_ = RVLDOTPRODUCT3(P1C_, ZLC);
	double w2_ = RVLDOTPRODUCT3(P2C_, ZLC);

	double w0;

	if (Flags & RVL3DLINE_MATCH_FLAG_OVERLAP)
	{
		double w1o = RVLMAX(w1, w1_);
		double w2o = RVLMIN(w2, w2_);

		double dwo = w2o - w1o;

		double rOverlap = dwo / pData->len;

		if (rOverlap < minrOverlap)
			return false;

		double rOverlap_ = dwo / pData_->len;

		if (rOverlap_ < minrOverlap)
			return false;

		// central point of overlapping segment

		w0 = 0.5 * (w1o + w2o);
	}

	// x and y-axis of the match reference frame

	RVLCROSSPRODUCT3(V, V_, XLC)

	fTmp = RVLDOTPRODUCT3(XLC, XLC);

	int i, j, k;

	if (fTmp <= APPROX_ZERO)
	{
		//double absZ[3];		
		//RVLORTHOGONAL3(ZLC, XLC, i, j, k, absZ, fTmp)
		RVLORTHOGONAL3(ZLC, XLC, i, j, k, fTmp)
	}
	else
	{
		fTmp = sqrt(fTmp);

		RVLSCALE3VECTOR2(XLC, fTmp, XLC)
	}

	RVLCROSSPRODUCT3(ZLC, XLC, YLC);

	if (Flags & RVL3DLINE_MATCH_FLAG_OVERLAP)
	{
		// origin of the match reference frame

		RVLSCALE3VECTOR(ZLC, w0, tLC)	

		RVLMULMX3X3VECT(RCL, tLC, tCL)

		RVLNEGVECT3(tCL, tCL)
	}

	// transform lines into the match reference frame

	double P1[3], P2[3], P1_[3], P2_[3];

	if(Flags & RVL3DLINE_MATCH_FLAG_OVERLAP)
	{
		RVLTRANSF3(P1C, RCL, tCL, P1)
		RVLTRANSF3(P2C, RCL, tCL, P2)
		RVLTRANSF3(P1C_, RCL, tCL, P1_)
		RVLTRANSF3(P2C_, RCL, tCL, P2_)
	}
	else
	{
		RVLMULMX3X3VECT(RCL, P1C, P1)
		RVLMULMX3X3VECT(RCL, P2C, P2)
		RVLMULMX3X3VECT(RCL, P1C_, P1_)
		RVLMULMX3X3VECT(RCL, P2C_, P2_)
	}

	double C1[3*3], C2[3*3], C1_[3*3], C2_[3*3];
	double *CC;
	double Mx3x3Tmp[3*3];	

	CC = m_CX[0];
	RVLCOV3DTRANSF(CC, RCL, C1, Mx3x3Tmp)

	CC = m_CX[1];
	RVLCOV3DTRANSF(CC, RCL, C2, Mx3x3Tmp)

	CC = pLine_->m_CX[0];
	RVLCOV3DTRANSF(CC, RCL, C1_, Mx3x3Tmp)

	CC = pLine_->m_CX[1];
	RVLCOV3DTRANSF(CC, RCL, C2_, Mx3x3Tmp)

	// computing the central points

	double dw = w2 - w1;
	double dw_ = w2_ - w1_;

	double s, s_;
	double C0[2*2], C0_[2*2];

	if(Flags & RVL3DLINE_MATCH_FLAG_OVERLAP)
	{
		// central points of the overlapping segments

		ComputeMatchParams(w1, dw, w0, C1, C2, C0, s, fTmp);
		ComputeMatchParams(w1_, dw_, w0, C1_, C2_, C0_, s_, fTmp);	
	}
	else
	{
		// central points of the matched lines

		ComputeMatchParams(w1, dw, 0.5 * (w1 + w2), C1, C2, C0, s, fTmp);
		ComputeMatchParams(w1_, dw_, 0.5 * (w1_ + w2_), C1_, C2_, C0_, s_, fTmp);			
	}

	double P0[2], P0_[2];

	P0[0] = P1[0] + s * (P2[0] - P1[0]);
	P0[1] = P1[1] + s * (P2[1] - P1[1]);
	P0_[0] = P1_[0] + s_ * (P2_[0] - P1_[0]);
	P0_[1] = P1_[1] + s_ * (P2_[1] - P1_[1]);

	// computing the uncertainty of the relative position of the central points:
	//     P0C <- P1C + s * (P2C - P1C)
	//     ZRC <- P0C / ||P0C||
	//     XRC <- unit vector orthogonal to ZRC
	//     YRC <- ZRC x XRC
	//     RCR <- [XRC'; YRC'; ZRC']
	//     RRL <- RCL * RCR'
	//     J <- [1 0 0] * RRL
	//          [0 1 0]
	//     varOrientUncert = ||P0C||^2 * pMatchData->varOrientationUncert
	//     COrientUncert <- varOrientUncert * J * [1 0 0] * J'
	//                                            [0 1 0]
	//                                            [0 0 0]
	//     It can be shown that  J * [1 0 0] * J' = J_ * J_, where J_ = [XLC'; YLC'] * [XRC YRC]
	//                               [0 1 0]
	//                               [0 0 0]
	//     CPositionUncert <- eye(2) * pMatchData->varPositionUncert
	//     C <- C0 + C0_ + COrientUncert + CPositionUncert

	// // Version before ECMR 2019

	//double P0C[3];

	//P0C[0] = P1C[0] + s * (P2C[0] - P1C[0]);
	//P0C[1] = P1C[1] + s * (P2C[1] - P1C[1]);
	//P0C[2] = P1C[2] + s * (P2C[2] - P1C[2]);

	//double r_ = sqrt(RVLDOTPRODUCT3(P0C, P0C));

	//double RCR[3 * 3];

	//double *XRC = RCR;
	//double *YRC = RCR + 3;
	//double *ZRC = RCR + 6;

	//RVLSCALE3VECTOR2(P0C, r_, ZRC);

	//RVLORTHOGONAL3(ZRC, XRC, i, j, k, fTmp);

	//RVLCROSSPRODUCT3(ZRC, XRC, YRC);

	//double J_[4];

	//J_[0] = RVLDOTPRODUCT3(XLC, XRC);
	//J_[1] = RVLDOTPRODUCT3(XLC, YRC);
	//J_[2] = RVLDOTPRODUCT3(YLC, XRC);
	//J_[3] = RVLDOTPRODUCT3(YLC, YRC);

	//double varOrientUncert_ = r_ * r_ * pMatchData->varOrientationUncert;

	//double COrientUncert_[2 * 2];

	//COrientUncert_[0] = varOrientUncert_ * (J_[0] * J_[0] + J_[1] * J_[1]);
	//COrientUncert_[1] = varOrientUncert_ * (J_[0] * J_[2] + J_[1] * J_[3]);
	//COrientUncert_[3] = varOrientUncert_ * (J_[2] * J_[2] + J_[3] * J_[3]);

	// Version for ECMR 2019

	double P0C_[3];

	P0C_[0] = P1C_[0] + s_ * (P2C_[0] - P1C_[0]);
	P0C_[1] = P1C_[1] + s_ * (P2C_[1] - P1C_[1]);
	P0C_[2] = P1C_[2] + s_ * (P2C_[2] - P1C_[2]);

	double V3Tmp[3];

	RVLDIF3VECTORS(P0C_, tMS, V3Tmp);
	
	double r[3];

	RVLMULMX3X3VECT(RCL, V3Tmp, r);

	double COrientUncert[2 * 2];

	COrientUncert[0] = pMatchData->varOrientationUncert * (r[1] * r[1] + r[2] * r[2]);
	COrientUncert[1] = -pMatchData->varOrientationUncert * r[0] * r[1];
	COrientUncert[3] = pMatchData->varOrientationUncert * (r[0] * r[0] + r[2] * r[2]);

	double E[2];

	E[0] = P0_[0] - P0[0];
	E[1] = P0_[1] - P0[1];

	double C[2 * 2];

	C[0] = C0[0] + C0_[0] + COrientUncert[0] + pMatchData->varPositionUncert; 
	C[1] = C0[1] + C0_[1] + COrientUncert[1]; 
	C[3] = C0[3] + C0_[3] + COrientUncert[3] + pMatchData->varPositionUncert; 

	// computing the Mahalanobis distance of the central points

	double detC = RVLDET2(C);

	if (detC < 0.0)
		int debug = 0;

	double ep = RVLMAHDIST2(E, C, detC);

	if(Flags & RVL3DLINE_MATCH_FLAG_POSITION)
		if(ep > 9.21)
			return false;

	// position probability

	double Pp = pMatchData->PPriorPosition - 0.5*(log(detC)+ep) - RVLLN2PI;	

	if(Pp < 0.0)
		Pp = 0.0;

	// orientation probability

	double C1o[2*2], C2o[2*2], C1o_[2*2], C2o_[2*2];
	double varz1o, varz2o, varz1o_, varz2o_;

	//ComputeMatchParams(w1, dw, w1o, C1, C2, C1o, s, varz1o);
	//ComputeMatchParams(w1, dw, w20, C1, C2, C2o, s, varz2o);
	//ComputeMatchParams(w1_, dw_, w1o, C1_, C2_, C1o_, s, varz1o_);
	//ComputeMatchParams(w1_, dw_, w2o, C1_, C2_, C2o_, s, varz2o_);

	//ComputeMatchParams(w1, dw, w1, C1, C2, C1o, s, varz1o);
	//ComputeMatchParams(w1, dw, w2, C1, C2, C2o, s, varz2o);
	//ComputeMatchParams(w1_, dw_, w1_, C1_, C2_, C1o_, s, varz1o_);
	//ComputeMatchParams(w1_, dw_, w2_, C1_, C2_, C2o_, s, varz2o_);

	C1o[0] = C1[0]; C1o[1] = C1[1];	C1o[3] = C1[4]; varz1o = C1[8];
	C2o[0] = C2[0]; C2o[1] = C2[1];	C2o[3] = C2[4]; varz2o = C2[8];
	C1o_[0] = C1_[0]; C1o_[1] = C1_[1];	C1o_[3] = C1_[4]; varz1o_ = C1_[8];
	C2o_[0] = C2_[0]; C2o_[1] = C2_[1];	C2o_[3] = C2_[4]; varz2o_ = C2_[8];

	double Cu[2*2], Cu_[2*2];
	double eu, Pu;

	//if(!ComputeOrientUncert(C1o, C2o, varz1o, varz2o, dwo, Cu))
	if(!ComputeOrientUncert(C1o, C2o, varz1o, varz2o, pData->len, Cu))
		Pu = 0.0;
	//else if(!ComputeOrientUncert(C1o_, C2o_, varz1o_, varz2o_, dwo, Cu_))
	else if(!ComputeOrientUncert(C1o_, C2o_, varz1o_, varz2o_, pData_->len, Cu_))
		Pu = 0.0;
	else
	{
		double CuS[2*2];

		CuS[0] = Cu[0] + Cu_[0] + pMatchData->varOrientationUncert;
		CuS[1] = Cu[1] + Cu_[1];
		CuS[3] = Cu[3] + Cu_[3] + pMatchData->varOrientationUncert;

		double detCuS = RVLDET2(CuS);

		if(detCuS > 4.0)
			Pu = 0.0;
		else
		{	
			double uy = RVLDOTPRODUCT3(V, YLC);

			eu = CuS[0] * 4.0 * uy * uy / detCuS;

			Pu = RVLLN4PI - 0.5*(log(detCuS)+eu) - RVLLN2PI;

			if(Pu < 0.0)
				Pu = 0.0;
		}
	}

	pMatchData->POrientMatch = Pu;

	// total probability

	MatchQuality = Pu + Pp;

	return true;
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

BYTE RVLCrop3DLine(	double *X1Src, 
					double *X2Src,
					double minz,
					double *X1Tgt,
					double *X2Tgt)
{
	BYTE bOut = ((BYTE)(X1Src[2] < minz) << 3) | ((BYTE)(X2Src[2] < minz) << 4);

	double XBuff[3];	

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
			RVLCOPY3VECTOR(XBuff, X1Tgt)
			RVLCOPY3VECTOR(X2Src, X2Tgt)
		}
		else
		{
			RVLCOPY3VECTOR(X1Src, X1Tgt)
			RVLCOPY3VECTOR(XBuff, X2Tgt)
		}
	}
	else
	{
		RVLCOPY3VECTOR(X1Src, X1Tgt)
		RVLCOPY3VECTOR(X2Src, X2Tgt)
	}

	return bOut;
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
	double X1[3], X2[3];

	BYTE bOut = RVLCrop3DLine(X1Src, X2Src, minz, X1, X2);

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

void RVLCrop3DLineSpherical(double *X1Src, 
							double *X2Src,
							double *R,
							double *t,
							CRVLCamera *pCamera,
							RVLRECT *pROI,
							double minz,
							BYTE *bOutLT,
							CvPoint **pPtArray,
							int &n,
							BYTE &CropSide)
{
	double imageSegmentLen = 20.0;

	double X1C[3], X2C[3];

	RVLTRANSF3(X1Src, R, t, X1C)

	RVLTRANSF3(X2Src, R, t, X2C)

	//double X1C_[3], X2C_[3];

	//BYTE bOut = RVLCrop3DLine(X1C, X2C, minz, X1C_, X2C_);

	BYTE bOut = 0x00;

	double U[2];
	int U1[2], U2[2], dU[2];

	//pCamera->Project3DPointToSphere(X1C_, U, U1);

	//pCamera->Project3DPointToSphere(X2C_, U, U2);

	pCamera->Project3DPointToSphere(X1C, U, U1);

	pCamera->Project3DPointToSphere(X2C, U, U2);

	dU[0] = U2[0] - U1[0];
	dU[1] = U2[1] - U1[1];

	n = (int)(floor(sqrt((double)(dU[0] * dU[0] + dU[1] * dU[1])) / imageSegmentLen)) + 2;

	double dXC[3];

	RVLDIF3VECTORS(X2C, X1C, dXC);

	double nSegments = (double)(n - 1);

	RVLSCALE3VECTOR2(dXC, nSegments, dXC);

	double XCm1[3], XCm2[3];

	RVLCOPY3VECTOR(X1C, XCm1);

	pCamera->Project3DPointToSphere(XCm1, U, U1);

	bool bFirstPt = false;

	CvPoint *PtArray = new CvPoint[n];

	CvPoint *pPt = PtArray;

	BYTE bOut_;
	CvPoint TgtPt1, TgtPt2;

	for(int i = 1; i < n; i++)
	{
		RVLSUM3VECTORS(XCm1, dXC, XCm2);

		pCamera->Project3DPointToSphere(XCm2, U, U2);

		bOut_ = RVLCrop2DLine(U1[0], U1[1], U2[0], U2[1], pROI, bOutLT, &TgtPt1, &TgtPt2, CropSide) | bOut; 	

		if(bOut_ & 0x04)
			continue;

		if(!bFirstPt)
		{
			bFirstPt = true;

			*(pPt++) = TgtPt1;
		}

		*(pPt++) = TgtPt2;

		RVLCOPY3VECTOR(XCm2, XCm1)

		U1[0] = U2[0];
		U1[1] = U2[1];
	}

	n = pPt - PtArray;

	*pPtArray = PtArray;
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