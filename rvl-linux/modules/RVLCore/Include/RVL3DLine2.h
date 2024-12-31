// RVL3DLine2.h: interface for the CRVL3DLine2 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL3DLINE2_H__71911163_7170_41C4_B8FD_AED80956A47D__INCLUDED_)
#define AFX_RVL3DLINE2_H__71911163_7170_41C4_B8FD_AED80956A47D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVL3DObject.h"

#define RVL3DLINE_PARAM_FLAG_XL				0x00010000
#define RVL3DLINE_PARAM_FLAG_X				0x00020000
#define RVL3DLINE_PARAM_FLAG_CXL			0x00040000
#define RVL3DLINE_PARAM_FLAG_CX				0x00080000
#define RVL3DLINE_PARAM_FLAG_CS				0x00100000
#define RVL3DLINE_PARAM_FLAG_DX				0x00200000
#define RVL3DLINE_PARAM_FLAG_ROT			0x00400000
#define RVL3DLINE_PARAM_FLAG_HORIZONTAL		0x01000000
#define RVL3DLINE_PARAM_FLAG_VERTICAL		0x02000000

#define RVL3DLINE_MATCH_FLAG_OVERLAP		0x00000001
#define RVL3DLINE_MATCH_FLAG_POSITION		0x00000002

#define RVLRELLIST_INDEX_3DLINE_3D2DLINE	3

struct RVL3DLINE_CLOSEST_POINTS_DATA
{
	double l;
	double a;
	double SIS;
	double Sc;
};

struct RVL3DLINE_EXTENDED_DATA
{
	double dX[3];
	double V[3];
	double len;
};

struct RVL3DLINE2_MATCH_DATA
{
	double varPositionUncert;
	double varOrientationUncert;
	double PPriorPosition;
	double PPriorPosition1DOF;
	double POrientMatch;
};

void RVLCreateC3DLine(CRVLClass *pClass);
//void RVL3DLinesTransfLA(CRVLMPtrChain *p3DLineList, 
//						  CRVL3DPose *pPoseLA);
void RVL3DLineClosestPoint(double *X1,
						   double *U1,
						   double l1,
						   double *X2, 
						   double *U2, 
						   double l2,
						   double *Xc1,
						   double *Xc2);
BYTE RVLCrop3DLine(double *X1Src, double *X2Src,
				   CRVLCamera *pCamera,
				   RVLRECT *pROI,
				   double minz,
				   double minr,
				   BYTE *bOutLT,
				   int *iU1, int *iU2,
				   CvPoint *pTgtPt1,
				   CvPoint *pTgtPt2,
				   BYTE &CropSide);
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
					BYTE &CropSide);
BYTE RVLCrop3DLine(	double *X1Src, 
					double *X2Src,
					double minz,
					double *X1Tgt,
					double *X2Tgt);
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
							BYTE &CropSide);
BOOL RVL3DLineClosestPoints(double *X01,
							double *V1,
							double *X02,
							double *V2,
							double *X1,
							double *X2);
void RVL3DLineEKFUpdate(	double *XSA,
							double varxST,
							double *XMB,
							double varxMT,
							CRVL3DPose *pInitPose,
							double *xTB,
							CRVL3DPose *pFinalPose);

class CRVL3DLine2 : public CRVL3DObject  
{
public:
	double m_X[2][3];
	double m_CX[2][9];
	int m_nSupport;
	//double m_XL[2][3];
	//double m_CXL[2][9];
	//double m_Cs[2][4];
	//double m_dX[3];
	//double m_RotAE[9];
	//double m_len;

public:
	bool ComputeOrientUncert(	double *C1o,
								double *C2o,
								double varz1o,
								double varz2o,
								double dwo,
								double *Cu);
	bool Match(	CRVL3DObject *pObject_, 
				RVL3DLINE2_MATCH_DATA *pData,
				double *tMS,
				double &MatchQuality,
				DWORD Flags = (RVL3DLINE_MATCH_FLAG_OVERLAP | RVL3DLINE_MATCH_FLAG_POSITION));
	//void TransfLA(CRVL3DPose *pPoseLA);
	void Transform(	CRVL3DLine2 *pLineSrc,
					CRVL3DPose *pPose);
	void Save(	FILE *fp,
				DWORD Flags);
	void Load(	FILE *fp,
				DWORD Flags);
	void UpdateParams();
	CRVLObject2 * Create2(CRVLClass * pClass);
	CRVL3DLine2();
	virtual ~CRVL3DLine2();

private:
	void ComputeMatchParams(double w1,
							double dw,
							double wo,
							double *C1,
							double *C2,
							double *Co,
							double &s,
							double &varzo);
};

extern CRVL3DLine2 RVL3DLine2Template;

#endif // !defined(AFX_RVL3DLINE2_H__71911163_7170_41C4_B8FD_AED80956A47D__INCLUDED_)
