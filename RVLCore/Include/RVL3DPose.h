// RVL3DPose.h: interface for the CRVL3DPose class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL3DPOSE_H__43E7FA49_799B_4BFA_8AE5_9D354E2FCB75__INCLUDED_)
#define AFX_RVL3DPOSE_H__43E7FA49_799B_4BFA_8AE5_9D354E2FCB75__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVL3DPOSE_PARAM_FLAGS_X				0x00000001
#define RVL3DPOSE_PARAM_FLAGS_PTR			0x00000002
#define RVL3DPOSE_PARAM_FLAGS_ROT			0x0000000c
#define RVL3DPOSE_PARAM_FLAGS_ROTA0			0x00000004
#define RVL3DPOSE_PARAM_FLAGS_ROTLA			0x00000008
#define RVL3DPOSE_PARAM_FLAGS_ROTLL			0x0000000c
#define RVL3DPOSE_PARAM_FLAGS_COV_6x1D		0x00000010
#define RVL3DPOSE_PARAM_FLAGS_COV_6D		0x00000020
#define RVL3DPOSE_PARAM_FLAGS_COV_2X3D		0x00000030
#define RVL3DPOSE_PARAM_FLAGS_COV_XYALPHA	0x00000040
#define RVL3DPOSE_PARAM_FLAGS_COV_3D		0x00000050
#define RVL3DPOSE_PARAM_FLAGS_COV_LINE_3D	0x00000060
#define RVL3DPOSE_PARAM_FLAGS_COV			0x00000070
#define RVL3DPOSE_PARAM_FLAGS_DROT			0x00000080
#define RVL3DPOSE_PARAM_FLAGS_QUATERNION	0x00000100

#define RVL3DPOSE_FLAGS_XROT				0x00000001

#define RVLJACOBIANPTRTOROTX(X, R, c, s, XRot, J)\
{\
	J[0*3+0] = XRot[2];\
	J[1*3+0] = 0.0;\
	J[2*3+0] = -XRot[0];\
	J[0*3+1] = s * XRot[1];\
	J[1*3+1] = -s * XRot[0] - c * XRot[2];\
	J[2*3+1] = c * XRot[1];\
	J[0*3+2] = -R[0*3+0] * X[1] + R[0*3+1] * X[0];\
	J[1*3+2] = -R[1*3+0] * X[1] + R[1*3+1] * X[0];\
	J[2*3+2] = -R[2*3+0] * X[1] + R[2*3+1] * X[0];\
}

class CRVL3DPose;

struct RVL3DPOSE_CORRECT_EKF_DATA
{
	CvMat *J;
	CvMat *CXM;
	CvMat *CXSM;
	CvMat *Cqq;
	CvMat *Cqt;
	CvMat *Ctt;

	CvMat *Kq;
	CvMat *Kt;
	CvMat *Tmp33_1;
	CvMat *Tmp33_2;
	CvMat *Tmp33_3;
	CvMat *Tmp33_4;

	double q[3];
	double e[3];
};

void RVL3DPoseCovConvertToBlocks(CvMat *CSrc, 
								 double *CTgt);
void RVL3DPoseCov6DFromBlocks(double *CSrc, 
						      CvMat *CTgt);
void RVLUpdateTransform3D(double *Rot, double *Trans, 
					      double *dRot, double *dTrans);
void RVLJacobiandRot(double *X, 
					 double *J);
void RVLCreatePoseUncertMx(	CRVL3DPose *pstddPose,
							double *P,
							double kNrm);


class CRVL3DPose  
{
public:
	DWORD m_ParamFlags;
	double m_X[3];						// [VAR 00]
	double m_Alpha;						// [VAR 01]
	double m_Beta;						// [VAR 02]
	double m_Theta;						// [VAR 03]
	double m_Rot[9];					// [VAR 04]
	double *m_C;						// [VAR 05]
	double *m_q;						// [VAR 06]
	double *m_dRot;
	double m_OrientNrm;
	double m_ca, m_sa;
	void *m_pData;

public:
	void SetUncert(	CRVL3DPose *pstddPose);
	void EKFInit(RVL3DPOSE_CORRECT_EKF_DATA *pEKF);
	void EKFSetState(RVL3DPOSE_CORRECT_EKF_DATA *pEKF);
	void EKFCorrect(double *XM, 
					double *CXM, 
					double *XSM, 
					double *CXSM,
					double *J,
					RVL3DPOSE_CORRECT_EKF_DATA *pEKF);
	void EKFSetState(CvKalman *pKalman);
	void EKFCorrect(CvKalman *pKalman,
					double *XM, 
					double *CXM, 
					double *XSM, 
					double *CXSM,
					CvMat *Z,
					double *J);
	BOOL GetAngleAxis(double *V, double &theta);
	void UpdateRotFromAngleAxis(double *V, double theta);
	void PlanarSurfaceEKFUpdate(double *C, 
							    double *Q,
							    double *e);
	void PlanarSurfaceEKFUpdate2(	double *C, 
									double *Q,
									double *e,
									CRVL3DPose *pPose0 = NULL);
	void SetPMatrix(double *P); //P[36] 6x6

#ifdef RVL_DEBUG
	void DebugPrint();
#endif
	void Load(FILE *fp);
	void Save(FILE *fp);
	void JacobianPTR2RotX(double *X, 
						  double *XRot, 
						  double *J,
						  DWORD Flags = 0x00000000);
	void UpdateQuatFromRot();
	void UpdateRotFromQuat();
	void TransfWithUncert(double *XSrc, double *XTgt, double *CTgt);
	void UpdatedRotLL();
	void JacobianRotLL(double *X,
					   double *J,
					   int n = 3);
	void UpdatePTRLL();
	void UpdateRotLL();
	void Copy(CRVL3DPose *pPoseSrc);
	void Copy(CRVL3DPose *pPoseSrc, 
		CRVLMem *pMem);
	void UpdatePTRLA();
	void UpdateRotA0();
	void UpdatePTR();
	void UpdateRotLA();
	CRVL3DPose();
	virtual ~CRVL3DPose();

	inline void Transf(double *XSrc, 
					   double *XTgt)
	{
		LinearTransform3D(m_Rot, m_X, XSrc, XTgt);
	}

	inline void Rot(double *XSrc, double *XTgt)
	{
		LinearTransform3D(m_Rot, XSrc, XTgt);
	}

	inline void RotCov(double *CSrc, double *CTgt)
	{
		RotCov3D(CSrc, m_Rot, CTgt);
	}

	inline void InvTransf(double *XSrc, 
					      double *XTgt)
	{
		InvLinearTransform3D(m_Rot, m_X, XSrc, XTgt);
	}
	void Reset(void);
	void Diff(CRVL3DPose * pPose, double & dist, double & angle);
};

#endif // !defined(AFX_RVL3DPOSE_H__43E7FA49_799B_4BFA_8AE5_9D354E2FCB75__INCLUDED_)
