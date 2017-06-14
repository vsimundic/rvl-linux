// RVL3DPose.cpp: implementation of the CRVL3DPose class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVL3DPose.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL3DPose::CRVL3DPose()
{
	m_ParamFlags = 0x00000000;
	m_q = NULL;
	m_C = NULL;
	m_OrientNrm = 1; //DEG2RAD * 0.1;
}

CRVL3DPose::~CRVL3DPose()
{

}

// RotLA = [ 0  0  1]
//         [-1  0  0] * Roty(alpha) * Rotx(beta) * Rotz(theta) = 
//         [ 0 -1  0]
//
//       = [ -sa*cq+ca*sb*sq,  sa*sq+ca*sb*cq,           ca*cb]
//         [ -ca*cq-sa*sb*sq,  ca*sq-sa*sb*cq,          -sa*cb]
//         [          -cb*sq,          -cb*cq,              sb]

void CRVL3DPose::UpdateRotLA()
{
	double ca = cos(m_Alpha);
	double sa = sin(m_Alpha);
	double cb = cos(m_Beta);
	double sb = sin(m_Beta);
	double cq = cos(m_Theta);
	double sq = sin(m_Theta);

	m_Rot[0*3+0] = -sa*cq+ca*sb*sq;
	m_Rot[0*3+1] = sa*sq+ca*sb*cq;
	m_Rot[0*3+2] = ca*cb;

	m_Rot[1*3+0] = -ca*cq-sa*sb*sq;
	m_Rot[1*3+1] = ca*sq-sa*sb*cq;
	m_Rot[1*3+2] = -sa*cb;

	m_Rot[2*3+0] = -cb*sq;
	m_Rot[2*3+1] = -cb*cq;
	m_Rot[2*3+2] = sb;	

	m_ParamFlags &= ~RVL3DPOSE_PARAM_FLAGS_ROT;
	m_ParamFlags |= RVL3DPOSE_PARAM_FLAGS_ROTLA;
}



void CRVL3DPose::UpdatePTRLA()
{
	m_Alpha = atan(-m_Rot[1*3+2] / m_Rot[0*3+2]);
	m_Beta = asin(m_Rot[2*3+2]);
	m_Theta = atan(m_Rot[2*3+0] / m_Rot[2*3+1]);
	
	m_ParamFlags |= RVL3DPOSE_PARAM_FLAGS_PTR;
}



// RotA0 = [ ca,  -sa,   0]
//         [ sa,  ca,   0]
//         [  0,    0,   1]

void CRVL3DPose::UpdateRotA0()
{
	double ca = cos(m_Alpha);
	double sa = sin(m_Alpha);

	m_Rot[0*3+0] = ca;
	m_Rot[0*3+1] = -sa;
	m_Rot[0*3+2] = 0.0;

	m_Rot[1*3+0] = sa;
	m_Rot[1*3+1] = ca;
	m_Rot[1*3+2] = 0.0;

	m_Rot[2*3+0] = 0.0;
	m_Rot[2*3+1] = 0.0;
	m_Rot[2*3+2] = 1.0;

	m_ParamFlags &= ~RVL3DPOSE_PARAM_FLAGS_ROT;
	m_ParamFlags |= RVL3DPOSE_PARAM_FLAGS_ROTA0;
}

// obsolete !

void CRVL3DPose::UpdatePTR()
{
	PanTiltRoll(m_Rot, m_Alpha, m_Beta, m_Theta);

	m_ParamFlags |= RVL3DPOSE_PARAM_FLAGS_PTR;
}

void CRVL3DPose::Copy(CRVL3DPose *pPoseSrc, 
					  CRVLMem *pMem)
{
	memcpy(this, pPoseSrc, sizeof(CRVL3DPose));

	m_C = (double *)(pMem->Alloc(9 * sizeof(double)));

	memcpy(m_C, pPoseSrc->m_C, 9 * sizeof(double));
}

void CRVL3DPose::Copy(CRVL3DPose *pPoseSrc)
{
	double *C = m_C;
	double *q = m_q;
	//unsigned int paramFlags = m_ParamFlags;

	memcpy(this, pPoseSrc, sizeof(CRVL3DPose));

	//m_ParamFlags = paramFlags;

	m_C = C;
	m_q = q;

	DWORD CovType = (m_ParamFlags & RVL3DPOSE_PARAM_FLAGS_COV);

	if(CovType)
	{		
		switch(CovType){
		case RVL3DPOSE_PARAM_FLAGS_COV_3D:
			memcpy(m_C, pPoseSrc->m_C, 3 * 3 * sizeof(double));

			break;
		case RVL3DPOSE_PARAM_FLAGS_COV_6D:
			memcpy(m_C, pPoseSrc->m_C, 3 * 3 * 3 * sizeof(double));

			break;
		case RVL3DPOSE_PARAM_FLAGS_COV_LINE_3D:
			memcpy(m_C, pPoseSrc->m_C, (4 + 3 * 3) * sizeof(double));
		}
	}

	if(m_ParamFlags & RVL3DPOSE_PARAM_FLAGS_QUATERNION)
		memcpy(m_q, pPoseSrc->m_q, 4 * sizeof(double));
}

// RotLL'= Roty(alpha) * Rotx(beta) * Rotz(theta) = 
//       = [  ca*cq+sa*sb*sq, -ca*sq+sa*sb*cq,           sa*cb]
//         [           cb*sq,           cb*cq,             -sb]
//         [ -sa*cq+ca*sb*sq,  sa*sq+ca*sb*cq,           ca*cb]
 
void CRVL3DPose::UpdateRotLL()
{
	double ca = cos(m_Alpha);
	double sa = sin(m_Alpha);
	double cb = cos(m_Beta);
	double sb = sin(m_Beta);
	double cq = cos(m_Theta);
	double sq = sin(m_Theta);

	m_Rot[0*3+0] = ca*cq+sa*sb*sq;
	m_Rot[0*3+1] = -ca*sq+sa*sb*cq;
	m_Rot[0*3+2] = sa*cb;

	m_Rot[1*3+0] = cb*sq;
	m_Rot[1*3+1] = cb*cq;
	m_Rot[1*3+2] = -sb;

	m_Rot[2*3+0] = -sa*cq+ca*sb*sq;
	m_Rot[2*3+1] = sa*sq+ca*sb*cq;
	m_Rot[2*3+2] = ca*cb;	

	m_ParamFlags &= ~RVL3DPOSE_PARAM_FLAGS_ROT;
	m_ParamFlags |= RVL3DPOSE_PARAM_FLAGS_ROTLL;	
}


void CRVL3DPose::UpdatedRotLL()
{
	double ca = cos(m_Alpha);
	double sa = sin(m_Alpha);
	double cb = cos(m_Beta);
	double sb = sin(m_Beta);
	double cq = cos(m_Theta);
	double sq = sin(m_Theta);

	double *dRotAlpha = m_dRot;
	double *dRotBeta  = m_dRot + 3 * 3;
	double *dRotTheta = m_dRot + 2 * 3 * 3;

	dRotAlpha[0*3+0] = -sa*cq+ca*sb*sq;
	dRotAlpha[0*3+1] = sa*sq+ca*sb*cq;
	dRotAlpha[0*3+2] = ca*cb;

	dRotAlpha[1*3+0] = 0.0;
	dRotAlpha[1*3+1] = 0.0;
	dRotAlpha[1*3+2] = 0.0;

	dRotAlpha[2*3+0] = -ca*cq-sa*sb*sq;
	dRotAlpha[2*3+1] = ca*sq-sa*sb*cq;
	dRotAlpha[2*3+2] = -sa*cb;
	
	dRotBeta[0*3+0] = sa*cb*sq;
	dRotBeta[0*3+1] = sa*cb*cq;
	dRotBeta[0*3+2] = -sa*sb;

	dRotBeta[1*3+0] = -sb*sq;
	dRotBeta[1*3+1] = -sb*cq;
	dRotBeta[1*3+2] = -cb;

	dRotBeta[2*3+0] = ca*cb*sq;
	dRotBeta[2*3+1] = ca*cb*cq;
	dRotBeta[2*3+2] = -ca*sb;	

	dRotTheta[0*3+0] = -ca*sq+sa*sb*cq;
	dRotTheta[0*3+1] = -ca*cq-sa*sb*sq;
	dRotTheta[0*3+2] = 0.0;

	dRotTheta[1*3+0] = cb*cq;
	dRotTheta[1*3+1] = -cb*sq;
	dRotTheta[1*3+2] = -sb;

	dRotTheta[2*3+0] = sa*sq+ca*sb*cq;
	dRotTheta[2*3+1] = sa*cq-ca*sb*sq;
	dRotTheta[2*3+2] = 0.0;	

	m_ParamFlags |= RVL3DPOSE_PARAM_FLAGS_DROT;
}


// Let: XL' = RotLL' * X + t;
//      Phi = [m_Alpha m_Beta m_Theta]'.
// Then J = dXL'/dPhi.

void CRVL3DPose::JacobianRotLL(double *X,
							   double *J,
							   int n)
{
	double *dRotAlpha = m_dRot;
	double *dRotBeta  = m_dRot + 3 * 3;
	double *dRotTheta = m_dRot + 2 * 3 * 3;

	if((m_ParamFlags & RVL3DPOSE_PARAM_FLAGS_DROT) == 0)
		UpdatedRotLL();

	double a[3];

	LinearTransform3D(dRotAlpha, X, a);

	J[0*n+0] = a[0];
	J[1*n+0] = a[1];
	J[2*n+0] = a[2];

	LinearTransform3D(dRotBeta, X, a);

	J[0*n+1] = a[0];
	J[1*n+1] = a[1];
	J[2*n+1] = a[2];

	LinearTransform3D(dRotTheta, X, a);

	J[0*n+2] = a[0];
	J[1*n+2] = a[1];
	J[2*n+2] = a[2];	
}



void CRVL3DPose::UpdatePTRLL()
{
	m_Alpha = atan2(m_Rot[0*3+2], m_Rot[2*3+2]);
	m_Beta = asin(-m_Rot[1*3+2]);
	m_Theta = atan2(m_Rot[1*3+0], m_Rot[1*3+1]);
	
	m_ParamFlags |= RVL3DPOSE_PARAM_FLAGS_PTR;
}

void CRVL3DPose::TransfWithUncert(double *XSrc, 
								  double *XTgt, 
								  double *CTgt)
{
	Transf(XSrc, XTgt);

	if((m_ParamFlags & 
		(RVL3DPOSE_PARAM_FLAGS_COV_2X3D | 
		RVL3DPOSE_PARAM_FLAGS_ROTLL)) ==
		(RVL3DPOSE_PARAM_FLAGS_COV_2X3D | 
		RVL3DPOSE_PARAM_FLAGS_ROTLL))
	{
		double J[3 * 3];

		JacobianRotLL(XSrc, J);

		RVLCovTransf(m_C + 3 * 3, J, 3, 3, CTgt);

		double *pCTrans = m_C;

		double *pCEnd = CTgt + 3 * 3;

		double *pC;

		for(pC = CTgt; pC < pCEnd; pC++, pCTrans++)
			*pC += (*pCTrans);
	}
}

void CRVL3DPose::UpdateRotFromQuat()
{
	m_Rot[0 * 3 + 0] = m_q[0]*m_q[0]+m_q[1]*m_q[1]-m_q[2]*m_q[2]-m_q[3]*m_q[3];
	m_Rot[1 * 3 + 0] = 2.0*m_q[1]*m_q[2]+2.0*m_q[0]*m_q[3];
	m_Rot[2 * 3 + 0] = 2.0*m_q[1]*m_q[3]-2.0*m_q[0]*m_q[2];
	m_Rot[0 * 3 + 1] = 2.0*m_q[1]*m_q[2]-2.0*m_q[0]*m_q[3];
	m_Rot[1 * 3 + 1] = m_q[0]*m_q[0]-m_q[1]*m_q[1]+m_q[2]*m_q[2]-m_q[3]*m_q[3];
	m_Rot[2 * 3 + 1] = 2.0*m_q[2]*m_q[3]+2.0*m_q[0]*m_q[1];
	m_Rot[0 * 3 + 2] = 2.0*m_q[1]*m_q[3]+2.0*m_q[0]*m_q[2];
	m_Rot[1 * 3 + 2] = 2.0*m_q[2]*m_q[3]-2.0*m_q[0]*m_q[1];
	m_Rot[2 * 3 + 2] = m_q[0]*m_q[0]-m_q[1]*m_q[1]-m_q[2]*m_q[2]+m_q[3]*m_q[3];
}


void CRVL3DPose::UpdateQuatFromRot()
{
	double a21 = m_Rot[1*3+0] - m_Rot[0*3+1];
	double a32 = m_Rot[2*3+1] - m_Rot[1*3+2];
	double a13 = m_Rot[0*3+2] - m_Rot[2*3+0];

	double p[3];

	p[0] = -(a21*a21 + a32*a32 + a13*a13);
	p[1] = -(m_Rot[0*3+0] + m_Rot[1*3+1] + m_Rot[2*3+2]);
	p[2] = 0.1875;

	double b[2];

	Roots2(p, b);

	double c = sqrt(b[0] > APPROX_ZERO ? b[0] : b[1]);

	m_q[0] = 0.25 * c;
	m_q[1] = a32 / c;
	m_q[2] = a13 / c;
	m_q[3] = a21 / c;	

	m_ParamFlags |= RVL3DPOSE_PARAM_FLAGS_QUATERNION;
}

// A detailed explanation of UpdateX() is given in RVL-documentation 

void CRVL3DPose::JacobianPTR2RotX(double *X, 
								  double *XRot, 
								  double *J,
								  DWORD Flags)
{
	if((Flags & RVL3DPOSE_FLAGS_XROT) == 0)
		Rot(X, XRot);

	//                        [ 0  0  1]
	// d(Rot * X) / dAlpha := [ 0  0  0] * Rot * X
	//                        [-1  0  0]

	J[0*3+0] = XRot[2] * m_OrientNrm;
	J[1*3+0] = 0.0;
	J[2*3+0] = -XRot[0] * m_OrientNrm;

	//                       [  0  sa   0]
	// d(Rot * X) / dBeta := [-sa   0 -ca] * Rot * X
	//                       [  0  ca   0]

	J[0*3+1] = m_sa * XRot[1] * m_OrientNrm;
	J[1*3+1] = (-m_sa * XRot[0] - m_ca * XRot[2]) * m_OrientNrm;
	J[2*3+1] = m_ca * XRot[1] * m_OrientNrm;

	//                              [ 0 -1  0]
	// d(Rot * X) / dTheta := Rot * [ 1  0  0] * X
	//                              [ 0  0  0]

	J[0*3+2] = (-m_Rot[0*3+0] * X[1] + m_Rot[0*3+1] * X[0]) * m_OrientNrm;
	J[1*3+2] = (-m_Rot[1*3+0] * X[1] + m_Rot[1*3+1] * X[0]) * m_OrientNrm;
	J[2*3+2] = (-m_Rot[2*3+0] * X[1] + m_Rot[2*3+1] * X[0]) * m_OrientNrm;
}


// Reference: Wikipedia - Axis angle

BOOL CRVL3DPose::GetAngleAxis(double *V, double &theta)
{
	double k = 0.5 * (m_Rot[0*3+0] + m_Rot[1*3+1] + m_Rot[2*3+2] - 1.0);

	if(k > 1.0)
	{
		theta = 0.0;

		return FALSE;
	}
	else if(k < -1.0)
	{
		theta = PI;

		return FALSE;
	}

	theta = acos(k);

	k = 0.5 / sin(theta);

	V[0] = k * (m_Rot[2 * 3 + 1] - m_Rot[1 * 3 + 2]);
	V[1] = k * (m_Rot[0 * 3 + 2] - m_Rot[2 * 3 + 0]);
	V[2] = k * (m_Rot[1 * 3 + 0] - m_Rot[0 * 3 + 1]);

	return TRUE;
}

// Reference: Osnove robotike (consistent with Wikipedia - Axis angle)

void CRVL3DPose::UpdateRotFromAngleAxis(double *V, double theta)
{
	double cq = cos(theta);
	double sq = sin(theta);
	double k = 1.0 - cq;

	m_Rot[0 * 3 + 0] = V[0] * V[0] * k + cq;
	m_Rot[0 * 3 + 1] = V[1] * V[0] * k - V[2] * sq;
	m_Rot[0 * 3 + 2] = V[2] * V[0] * k + V[1] * sq;
	m_Rot[1 * 3 + 0] = V[0] * V[1] * k + V[2] * sq;
	m_Rot[1 * 3 + 1] = V[1] * V[1] * k + cq;
	m_Rot[1 * 3 + 2] = V[2] * V[1] * k - V[0] * sq;
	m_Rot[2 * 3 + 0] = V[0] * V[2] * k - V[1] * sq;
	m_Rot[2 * 3 + 1] = V[1] * V[2] * k + V[0] * sq;
	m_Rot[2 * 3 + 2] = V[2] * V[2] * k + cq;
}

void CRVL3DPose::Save(FILE *fp)
{
	fwrite(m_X, sizeof(double), 3, fp);

	fwrite(&m_Alpha, sizeof(double), 1, fp);

	fwrite(&m_Beta, sizeof(double), 1, fp);

	fwrite(&m_Theta, sizeof(double), 1, fp);

	fwrite(m_Rot, sizeof(double), 3 * 3, fp);
}

void CRVL3DPose::Load(FILE *fp)
{
	fread(m_X, sizeof(double), 3, fp);

	fread(&m_Alpha, sizeof(double), 1, fp);

	fread(&m_Beta, sizeof(double), 1, fp);

	fread(&m_Theta, sizeof(double), 1, fp);

	fread(m_Rot, sizeof(double), 3 * 3, fp);
}

#ifdef RVL_DEBUG

void CRVL3DPose::DebugPrint()
{
	fprintf(fpDebug, "Pose:\n");

	if(m_ParamFlags & RVL3DPOSE_PARAM_FLAGS_PTR)
		fprintf(fpDebug, "Orient=(%lf,%lf,%lf)\n", 
			m_Alpha * RAD2DEG, m_Beta * RAD2DEG, m_Theta * RAD2DEG);

	fprintf(fpDebug, "Transl=(%lf,%lf,%lf)\n", 
		m_X[0], m_X[1], m_X[2]);

	DWORD CovType = (m_ParamFlags & RVL3DPOSE_PARAM_FLAGS_COV);

	if(CovType)
	{
		fprintf(fpDebug, "Cov:\n");

		switch(CovType){
		case RVL3DPOSE_PARAM_FLAGS_COV_LINE_3D:
			RVLPrintCov(fpDebug, m_C + 4, 3);

			break;
		}
	}

	fprintf(fpDebug, "ENDPose\n");
}

#endif


void CRVL3DPose::EKFCorrect(CvKalman *pKalman,
							double *XM, 
							double *CXM, 
							double *XSM, 
							double *CXSM,
							CvMat *Z,
							double *J)
{
	// skip prediction 

	memcpy(pKalman->state_pre->data.fl, pKalman->state_post->data.fl, 6 * sizeof(float));

	memcpy(pKalman->error_cov_pre->data.fl, pKalman->error_cov_post->data.fl, 6 * 6 * sizeof(float));

	// H = [J I(3x3)]

	float *H = pKalman->measurement_matrix->data.fl;

	H[0*6+0] = (float)J[0*3+0]; H[0*6+1] = (float)J[0*3+1]; H[0*6+2] = (float)J[0*3+2];
	H[1*6+0] = (float)J[1*3+0]; H[1*6+1] = (float)J[1*3+1]; H[1*6+2] = (float)J[1*3+2];
	H[2*6+0] = (float)J[2*3+0]; H[2*6+1] = (float)J[2*3+1]; H[2*6+2] = (float)J[2*3+2];
	H[0*6+3] = 1.0; H[0*6+4] = 0.0; H[0*6+5] = 0.0;
	H[1*6+3] = 0.0; H[1*6+4] = 1.0; H[1*6+5] = 0.0;
	H[2*6+3] = 0.0; H[2*6+4] = 0.0; H[2*6+5] = 1.0;

	// R = CXM + CXSM

	float *R = pKalman->measurement_noise_cov->data.fl;

	R[0*3+0] = (float)(CXM[0*3+0] + CXSM[0*3+0]);
	R[0*3+1] = (float)(CXM[0*3+1] + CXSM[0*3+1]);
	R[0*3+2] = (float)(CXM[0*3+2] + CXSM[0*3+2]);

	R[1*3+0] = (float)(CXM[1*3+0] + CXSM[1*3+0]);
	R[1*3+1] = (float)(CXM[1*3+1] + CXSM[1*3+1]);
	R[1*3+2] = (float)(CXM[1*3+2] + CXSM[1*3+2]);

	R[2*3+0] = (float)(CXM[2*3+0] + CXSM[2*3+0]);
	R[2*3+1] = (float)(CXM[2*3+1] + CXSM[2*3+1]);
	R[2*3+2] = (float)(CXM[2*3+2] + CXSM[2*3+2]);

	// z = XM - XSM

	float *z = Z->data.fl;

	z[0] = (float)(XM[0] - XSM[0]);
	z[1] = (float)(XM[1] - XSM[1]);
	z[2] = (float)(XM[2] - XSM[2]);

	cvMatMulAdd(pKalman->measurement_matrix, pKalman->state_pre, Z, Z);

	// Kalman Filter Update

	cvKalmanCorrect(pKalman, Z);

	// m_X = W

	float *W = pKalman->state_post->data.fl;

	m_Alpha = (double)W[0] * m_OrientNrm;
	m_Beta  = (double)W[1] * m_OrientNrm;
	m_Theta = (double)W[2] * m_OrientNrm;
	m_X[0] = (double)W[3];
	m_X[1] = (double)W[4];
	m_X[2] = (double)W[5];

	// CW = m_C

	float *CW = pKalman->error_cov_post->data.fl;

	m_C[0*9+0*3+0] = (double)CW[0*6+0];
	m_C[0*9+0*3+1] = (double)CW[0*6+1];
	m_C[0*9+0*3+2] = (double)CW[0*6+2];

	m_C[1*9+0*3+0] = (double)CW[0*6+3];
	m_C[1*9+0*3+1] = (double)CW[0*6+4];
	m_C[1*9+0*3+2] = (double)CW[0*6+5];

	m_C[0*9+1*3+0] = (double)CW[1*6+0];
	m_C[0*9+1*3+1] = (double)CW[1*6+1];
	m_C[0*9+1*3+2] = (double)CW[1*6+2];

	m_C[1*9+1*3+0] = (double)CW[1*6+3];
	m_C[1*9+1*3+1] = (double)CW[1*6+4];
	m_C[1*9+1*3+2] = (double)CW[1*6+5];

	m_C[0*9+2*3+0] = (double)CW[2*6+0];
	m_C[0*9+2*3+1] = (double)CW[2*6+1];
	m_C[0*9+2*3+2] = (double)CW[2*6+2];

	m_C[1*9+2*3+0] = (double)CW[2*6+3];
	m_C[1*9+2*3+1] = (double)CW[2*6+4];
	m_C[1*9+2*3+2] = (double)CW[2*6+5];

	m_C[2*9+0*3+0] = (double)CW[3*6+3];
	m_C[2*9+0*3+1] = (double)CW[3*6+4];
	m_C[2*9+0*3+2] = (double)CW[3*6+5];

	m_C[2*9+1*3+0] = (double)CW[4*6+3];
	m_C[2*9+1*3+1] = (double)CW[4*6+4];
	m_C[2*9+1*3+2] = (double)CW[4*6+5];

	m_C[2*9+2*3+0] = (double)CW[5*6+3];
	m_C[2*9+2*3+1] = (double)CW[5*6+4];
	m_C[2*9+2*3+2] = (double)CW[5*6+5];	

	UpdateRotLL();
}

void CRVL3DPose::EKFSetState(CvKalman *pKalman)
{
	// W = m_X

	float *W = pKalman->state_post->data.fl;

	W[0] = (float)(m_Alpha / m_OrientNrm);
	W[1] = (float)(m_Beta / m_OrientNrm);
	W[2] = (float)(m_Theta / m_OrientNrm);
	W[3] = (float)m_X[0];
	W[4] = (float)m_X[1];
	W[5] = (float)m_X[2];

	// CW = m_C

	float *CW = pKalman->error_cov_post->data.fl;

	CW[0*6+0] = (float)m_C[0*9+0*3+0];
	CW[0*6+1] = (float)m_C[0*9+0*3+1];
	CW[0*6+2] = (float)m_C[0*9+0*3+2];
	CW[0*6+3] = (float)m_C[1*9+0*3+0];
	CW[0*6+4] = (float)m_C[1*9+0*3+1];
	CW[0*6+5] = (float)m_C[1*9+0*3+2];

	CW[1*6+0] = (float)m_C[0*9+1*3+0];
	CW[1*6+1] = (float)m_C[0*9+1*3+1];
	CW[1*6+2] = (float)m_C[0*9+1*3+2];
	CW[1*6+3] = (float)m_C[1*9+1*3+0];
	CW[1*6+4] = (float)m_C[1*9+1*3+1];
	CW[1*6+5] = (float)m_C[1*9+1*3+2];

	CW[2*6+0] = (float)m_C[0*9+2*3+0];
	CW[2*6+1] = (float)m_C[0*9+2*3+1];
	CW[2*6+2] = (float)m_C[0*9+2*3+2];
	CW[2*6+3] = (float)m_C[1*9+2*3+0];
	CW[2*6+4] = (float)m_C[1*9+2*3+1];
	CW[2*6+5] = (float)m_C[1*9+2*3+2];

	CW[3*6+0] = (float)m_C[1*9+0*3+0];
	CW[3*6+1] = (float)m_C[1*9+1*3+0];
	CW[3*6+2] = (float)m_C[1*9+2*3+0];
	CW[3*6+3] = (float)m_C[2*9+0*3+0];
	CW[3*6+4] = (float)m_C[2*9+0*3+1];
	CW[3*6+5] = (float)m_C[2*9+0*3+2];

	CW[4*6+0] = (float)m_C[1*9+0*3+1];
	CW[4*6+1] = (float)m_C[1*9+1*3+1];
	CW[4*6+2] = (float)m_C[1*9+2*3+1];
	CW[4*6+3] = (float)m_C[2*9+1*3+0];
	CW[4*6+4] = (float)m_C[2*9+1*3+1];
	CW[4*6+5] = (float)m_C[2*9+1*3+2];

	CW[5*6+0] = (float)m_C[1*9+0*3+2];
	CW[5*6+1] = (float)m_C[1*9+1*3+2];
	CW[5*6+2] = (float)m_C[1*9+2*3+2];
	CW[5*6+3] = (float)m_C[2*9+2*3+0];
	CW[5*6+4] = (float)m_C[2*9+2*3+1];
	CW[5*6+5] = (float)m_C[2*9+2*3+2];
}


void CRVL3DPose::EKFCorrect(double *XM, 
							double *CXM, 
							double *XSM, 
							double *CXSM,
							double *J,
							RVL3DPOSE_CORRECT_EKF_DATA *pEKF)
{
	pEKF->J->data.db = J;
	pEKF->CXM->data.db = CXM;
	pEKF->CXSM->data.db = CXSM;

	// Tmp33_1 = CXM + CXSM

	cvAdd(pEKF->CXSM, pEKF->CXM, pEKF->Tmp33_1);

	// Tmp33_2 = Cqq * J' + Cqt
		
	cvGEMM(pEKF->Cqq, pEKF->J, 1.0, pEKF->Cqt, 1.0, pEKF->Tmp33_2, CV_GEMM_B_T);

	// Tmp33_3 = Cqt' * J' + Ctt

	cvGEMM(pEKF->Cqt, pEKF->J, 1.0, pEKF->Ctt, 1.0, pEKF->Tmp33_3, CV_GEMM_A_T + CV_GEMM_B_T);

	// Tmp33_4 = inv(Tmp33_1)

	cvInvert(pEKF->Tmp33_1, pEKF->Tmp33_4, CV_SVD_SYM);

	// Kq = Tmp33_2 * Tmp33_4

	cvMatMul(pEKF->Tmp33_2, pEKF->Tmp33_4, pEKF->Kq);

	// Kt = Tmp33_3 * Tmp33_4

	cvMatMul(pEKF->Tmp33_3, pEKF->Tmp33_4, pEKF->Kt);

	// e = XM - XSM

	RVLDif3D(XM, XSM, pEKF->e);

	// q = Kq * e + q

	LinearTransform3D(pEKF->Kq->data.db, pEKF->q, pEKF->e, pEKF->q);

	// m_X = Kt * e + m_X

	LinearTransform3D(pEKF->Kt->data.db, m_X, pEKF->e, m_X);

	// Tmp33_2 = J * Cqq + Cqt'
		
	cvGEMM(pEKF->J, pEKF->Cqq, 1.0, pEKF->Cqt, 1.0, pEKF->Tmp33_2, CV_GEMM_C_T);

	// Tmp33_3 = J * Cqt + Ctt

	cvGEMM(pEKF->J, pEKF->Cqt, 1.0, pEKF->Ctt, 1.0, pEKF->Tmp33_3);

	// Cqq = Cqq - Kq * Tmp33_2
	
	cvGEMM(pEKF->Kq, pEKF->Tmp33_2, -1.0, pEKF->Cqq, 1.0, pEKF->Cqq);

	// Cqt = Cqt - Kq * Tmp33_3
	
	cvGEMM(pEKF->Kq, pEKF->Tmp33_3, -1.0, pEKF->Cqt, 1.0, pEKF->Cqt);

	// Ctt = Ctt - Kt * Tmp33_3
	
	cvGEMM(pEKF->Kt, pEKF->Tmp33_3, -1.0, pEKF->Ctt, 1.0, pEKF->Ctt);

	// update this

	m_Alpha = pEKF->q[0] * m_OrientNrm;
	m_Beta  = pEKF->q[1] * m_OrientNrm;
	m_Theta = pEKF->q[2] * m_OrientNrm;

	UpdateRotLL();
}


void CRVL3DPose::EKFInit(RVL3DPOSE_CORRECT_EKF_DATA *pEKF)
{
	pEKF->J				= cvCreateMatHeader(3, 3, CV_64FC1);
	pEKF->CXM			= cvCreateMatHeader(3, 3, CV_64FC1);
	pEKF->CXSM			= cvCreateMatHeader(3, 3, CV_64FC1);

	pEKF->Cqq			= cvCreateMatHeader(3, 3, CV_64FC1);
	pEKF->Cqt			= cvCreateMatHeader(3, 3, CV_64FC1);
	pEKF->Ctt			= cvCreateMatHeader(3, 3, CV_64FC1);

	pEKF->Cqq->data.db = m_C;
	pEKF->Cqt->data.db = m_C + 3 * 3;
	pEKF->Ctt->data.db = m_C + 2 * 3 * 3;

	pEKF->Kq			= cvCreateMat(3, 3, CV_64FC1);
	pEKF->Kt			= cvCreateMat(3, 3, CV_64FC1);
	pEKF->Tmp33_1		= cvCreateMat(3, 3, CV_64FC1);
	pEKF->Tmp33_2		= cvCreateMat(3, 3, CV_64FC1);
	pEKF->Tmp33_3		= cvCreateMat(3, 3, CV_64FC1);
	pEKF->Tmp33_4		= cvCreateMat(3, 3, CV_64FC1);
}

void CRVL3DPose::EKFSetState(RVL3DPOSE_CORRECT_EKF_DATA *pEKF)
{
	pEKF->q[0] = m_Alpha / m_OrientNrm;
	pEKF->q[1] = m_Beta  / m_OrientNrm;
	pEKF->q[2] = m_Theta / m_OrientNrm;	
}

void CRVL3DPose::SetUncert(	CRVL3DPose *pstddPose)
{
	RVLCreatePoseUncertMx(pstddPose, m_C, m_OrientNrm);
}


void CRVL3DPose::Reset(void)
{
	RVLUNITMX3(m_Rot);

	m_Alpha = m_Beta = m_Theta = 0.0;

	RVLNULL3VECTOR(m_X);
}

//Updates pose paramaters
//This EKF update is used for estimation of Pose from corresponding pairs of planar surfaces (read REG11)
void CRVL3DPose::PlanarSurfaceEKFUpdate(double *C,double *Q, double *e)
{
	int i;

	double P[36], Ptemp[36], Pnew[36];		// 6 x 6
	double K[18], Ktemp[18];				// 6 x 3
	double w[6], wtemp[6], wnew[6];			// 6 x 1
	double I[36];							// 6 x 6  identity matrix


	//Define I (identity matrix)
	memset(I,0,36*sizeof(double));
	for(i=0;i<6;i++)
		I[i*7] = 1;

	//Get P matrix
	RVL3x3x3BlockMxTo6x6(m_C, P);

	
	//Get w
	w[0] = m_Alpha;
	w[1] = m_Beta;
	w[2] = m_Theta;
	w[3] = m_X[0];
	w[4] = m_X[1];
	w[5] = m_X[2];
	


	//K = P*C'*Q
	MatrixMultiplicationT(P,C,Ktemp,6,6,3);
	MatrixMultiplication(Ktemp,Q,K,6,3,3);

	//Update P
	MatrixMultiplication(K,C,Ptemp,6,3,6);
	for(i=0;i<36;i++)
		Ptemp[i] = I[i] - Ptemp[i];

	MatrixMultiplication(Ptemp,P,Pnew,6,6,6);

	//Update w
	MatrixMultiplication(K,e,wtemp,6,3,1);
	for(i=0;i<6;i++)
		wnew[i] = w[i] - wtemp[i];



	//Store w ie wnew
	m_Alpha = wnew[0];
	m_Beta = wnew[1];
	m_Theta = wnew[2];
	m_X[0] = wnew[3];
	m_X[1] = wnew[4];
	m_X[2] = wnew[5];

	UpdateRotLL();
	m_sa = sin(m_Alpha);
	m_ca = cos(m_Alpha);


	//Store P ie Pnew
	SetPMatrix(Pnew);	

}

//Updates pose paramaters
//This EKF update is used for estimation of Pose from corresponding pairs of planar surfaces (read REG11)
void CRVL3DPose::PlanarSurfaceEKFUpdate2(double *C,double *Q, double *e, CRVL3DPose *pPose0)
{
	int i;

	double P[36], Ptemp[36], Pnew[36];		// 6 x 6
	double K[18], Ktemp[18];				// 6 x 3
	double w[6], wtemp[6], wnew[6];			// 6 x 1
	//double I[36];							// 6 x 6  identity matrix


	//Define I (identity matrix)
	//memset(I,0,36*sizeof(double));
	//for(i=0;i<6;i++)
	//	I[i*7] = 1;

	if(pPose0)
	{
		//Get P matrix
		RVL3x3x3BlockMxTo6x6(pPose0->m_C, P);
		
		//Get w
		w[0] = pPose0->m_Alpha;
		w[1] = pPose0->m_Beta;
		w[2] = pPose0->m_Theta;
		w[3] = pPose0->m_X[0];
		w[4] = pPose0->m_X[1];
		w[5] = pPose0->m_X[2];
	}
	else
	{
		//Get P matrix
		RVL3x3x3BlockMxTo6x6(m_C, P);

		
		//Get w
		w[0] = m_Alpha;
		w[1] = m_Beta;
		w[2] = m_Theta;
		w[3] = m_X[0];
		w[4] = m_X[1];
		w[5] = m_X[2];
	}
	
	// debug

	//FILE *fp;

	//fopen_s(&fp, "C:\\RVL\\Debug\\P.dat", "w");

	//RVLPrintMatrix(fp, P, 6, 6);
	//
	//fclose(fp);

	//fopen_s(&fp, "C:\\RVL\\Debug\\C.dat", "w");

	//RVLPrintMatrix(fp, C, 3, 6);
	//
	//fclose(fp);

	//fopen_s(&fp, "C:\\RVL\\Debug\\Q.dat", "w");

	//RVLPrintMatrix(fp, Q, 3, 3);
	//
	//fclose(fp);

	/////

	//K = P*C'*Q
	MatrixMultiplicationT(P,C,Ktemp,6,6,3);
	MatrixMultiplication(Ktemp,Q,K,6,3,3);

	//Update P
	MatrixMultiplication(K,C,Ptemp,6,3,6);
	//for(i=0;i<36;i++)
	//	Ptemp[i] = I[i] - Ptemp[i];

	MatrixMultiplication(Ptemp,P,Pnew,6,6,6);

	for(i=0;i<36;i++)
		Pnew[i] = P[i] - Pnew[i];

	//Update w
	MatrixMultiplication(K,e,wtemp,6,3,1);
	for(i=0;i<6;i++)
		wnew[i] = w[i] - wtemp[i];



	//Store w ie wnew
	m_Alpha = wnew[0];
	m_Beta = wnew[1];
	m_Theta = wnew[2];
	m_X[0] = wnew[3];
	m_X[1] = wnew[4];
	m_X[2] = wnew[5];

	UpdateRotLL();
	m_sa = sin(m_Alpha);
	m_ca = cos(m_Alpha);


	//Store P ie Pnew
	SetPMatrix(Pnew);	

}

//Store the P matrix to m_C
void CRVL3DPose::SetPMatrix(double *P)
{

	m_C[0*9+0*3+0] = P[0*6+0];
	m_C[0*9+0*3+1] = P[0*6+1];
	m_C[0*9+0*3+2] = P[0*6+2];

	m_C[1*9+0*3+0] = P[0*6+3];
	m_C[1*9+0*3+1] = P[0*6+4];
	m_C[1*9+0*3+2] = P[0*6+5];

	m_C[0*9+1*3+0] = P[1*6+0];
	m_C[0*9+1*3+1] = P[1*6+1];
	m_C[0*9+1*3+2] = P[1*6+2];

	m_C[1*9+1*3+0] = P[1*6+3];
	m_C[1*9+1*3+1] = P[1*6+4];
	m_C[1*9+1*3+2] = P[1*6+5];

	m_C[0*9+2*3+0] = P[2*6+0];
	m_C[0*9+2*3+1] = P[2*6+1];
	m_C[0*9+2*3+2] = P[2*6+2];

	m_C[1*9+2*3+0] = P[2*6+3];
	m_C[1*9+2*3+1] = P[2*6+4];
	m_C[1*9+2*3+2] = P[2*6+5];

	m_C[2*9+0*3+0] = P[3*6+3];
	m_C[2*9+0*3+1] = P[3*6+4];
	m_C[2*9+0*3+2] = P[3*6+5];

	m_C[2*9+1*3+0] = P[4*6+3];
	m_C[2*9+1*3+1] = P[4*6+4];
	m_C[2*9+1*3+2] = P[4*6+5];

	m_C[2*9+2*3+0] = P[5*6+3];
	m_C[2*9+2*3+1] = P[5*6+4];
	m_C[2*9+2*3+2] = P[5*6+5];	

}


//void RVLCreateRotZ(double q, 
//				   double *Rot)
//{
//	cs = cos(q);
//	sn = sin(q);
//
//	Rot[3 * 0 + 0] = cs;
//	Rot[3 * 0 + 1] = -sn;
//	Rot[3 * 0 + 2] = 0.0;
//
//	Rot[3 * 1 + 0] = sn;
//	Rot[3 * 1 + 1] = cs;
//	Rot[3 * 1 + 2] = 0.0;
//
//	Rot[3 * 2 + 0] = 0.0;
//	Rot[3 * 2 + 1] = 0.0;
//	Rot[3 * 2 + 2] = 0.1;
//}

void CRVL3DPose::Diff(CRVL3DPose * pPose, double & dist, double & angle)
{
	double *R = pPose->m_Rot;
	double *t = pPose->m_X;

	CRVL3DPose dPose;

	double *dR = dPose.m_Rot;
	double *dt = dPose.m_X;

	double V3x1Tmp[3];

	RVLCOMPTRANSF3DWITHINV(m_Rot, m_X, R, t, dR, dt, V3x1Tmp)

	dist = sqrt(RVLDOTPRODUCT3(dt, dt));

	double V[3];

	dPose.GetAngleAxis(V, angle);
}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 

void RVL3DTransfLA(double *XSrc, double *XTgt)
{
	XTgt[0] = XSrc[2];
	XTgt[1] = -XSrc[0];
	XTgt[2] = -XSrc[1];
}

// T(Rot, Trans) = T(dRot, dTrans) * T(Rot, Trans)
	
void RVLUpdateTransform3D(double *Rot, double *Trans, 
					      double *dRot, double *dTrans)
{
	double RotTmp[3 * 3];
	double TransTmp[3];

	MatrixMultiplication(dRot, Rot, RotTmp, 3, 3);
	memcpy(Rot, RotTmp, 3 * 3 * sizeof(double));
	LinearTransform3D(dRot, Trans, TransTmp);
	Trans[0] = TransTmp[0] + dTrans[0];
	Trans[1] = TransTmp[1] + dTrans[1];
	Trans[2] = TransTmp[2] + dTrans[2];
}

// Let X = (I + dRot) * X' + t
//     dRot = [   0 -rz  ry;
//               rz   0 -rx;
//              -ry  rx   0]
// The function returns: J = dX / r
// where r = [rx ry rz]'

void RVLJacobiandRot(double *X, 
					 double *J)
{
	J[0*3+0] = 0.0;
	J[0*3+1] = X[2];
	J[0*3+2] = -X[1];

	J[1*3+0] = -X[2];
	J[1*3+1] = 0.0;
	J[1*3+2] = X[0];

	J[2*3+0] = X[1];
	J[2*3+1] = -X[0];
	J[2*3+2] = 0.0;
}


void RVL3DPoseCovConvertToBlocks(CvMat *CSrc, 
						         double *CTgt)
{
	double *pC = CSrc->data.db;
	double *pC2 = CTgt;

	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);pC += 4;
	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);pC += 4;
	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);

	pC = CSrc->data.db + 3;

	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);pC += 4;
	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);pC += 4;
	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);

	pC = CSrc->data.db + 2 * 3 * 3 + 3;

	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);pC += 4;
	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);pC += 4;
	*(pC2++) = (*(pC++));
	*(pC2++) = (*(pC++));
	*(pC2++) = (*pC);
}

void RVL3DPoseCov6DFromBlocks(double *CSrc, 
						      CvMat *CTgt)
{
	double *pC = CTgt->data.db;
	double *pC2 = CSrc;

	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));pC += 4;
	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));pC += 4;	
	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));

	pC = CTgt->data.db + 3;

	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));pC += 4;	
	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));pC += 4;
	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));

	pC = CTgt->data.db + 3 * 6 + 3;

	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));pC += 4;
	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));pC += 4;
	*(pC++)		= (*(pC2++));
	*(pC++)		= (*(pC2++));
	*pC			= (*(pC2++));

	pC2 = CSrc + 3 * 3;

	double *pC0 = CTgt->data.db + 3 * 6;

	pC = pC0;
		
	*pC			= (*(pC2++));pC += 3;
	*pC			= (*(pC2++));pC += 3;
	*pC			= (*(pC2++));

	pC0++;
	pC = pC0;

	*pC			= (*(pC2++));pC += 3;
	*pC			= (*(pC2++));pC += 3;
	*pC			= (*(pC2++));

	pC0++;
	pC = pC0;

	*pC			= (*(pC2++));pC += 3;
	*pC			= (*(pC2++));pC += 3;
	*pC			= (*(pC2++));	
}

void RVLCreatePoseUncertMx(CRVL3DPose *pstddPose,
						   double *P,
						   double kNrm)
{
	memset(P, 0, 3 * 3 * 3 * sizeof(double));	
	double kq = 1.0 / (kNrm * kNrm);
	
	P[0*3*3+0*3+0] = pstddPose->m_Alpha * pstddPose->m_Alpha * DEG2RAD * DEG2RAD * kq;
	P[0*3*3+1*3+1] = pstddPose->m_Beta * pstddPose->m_Beta * DEG2RAD * DEG2RAD * kq;
	P[0*3*3+2*3+2] = pstddPose->m_Theta * pstddPose->m_Theta * DEG2RAD * DEG2RAD * kq;
	P[2*3*3+0*3+0] = pstddPose->m_X[0] * pstddPose->m_X[0];
	P[2*3*3+1*3+1] = pstddPose->m_X[1] * pstddPose->m_X[1];
	P[2*3*3+2*3+2] = pstddPose->m_X[2] * pstddPose->m_X[2];	
}