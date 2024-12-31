// RVL3DSurface2.h: interface for the CRVL3DSurface2 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL3DSURFACE2_H__2C1A94DD_51D5_4ECF_8DDC_1304987BAA4E__INCLUDED_)
#define AFX_RVL3DSURFACE2_H__2C1A94DD_51D5_4ECF_8DDC_1304987BAA4E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVL3DMeshObject.h" //"RVL3DObject2.h"


#define RVL3DSURFACE_FLAG_CONVEX						0x00010000
#define RVL3DSURFACE_FLAG_HYPOTHESIS_EVALUATION			0x00020000
#define RVL3DSURFACE_FLAG_MATERIAL						0x00040000
#define RVL3DSURFACE_FLAG_MARKED						RVLOBJ2_FLAG_MARKED
#define RVL3DSURFACE_FLAG_CLOSE							0x00080000
#define RVL3DSURFACE_FLAG_REMOVED						0x00100000
#define RVL3DSURFACE_FLAG_GROUND						0x00200000
#define RVL3DSURFACE_FLAG_SAMPLES						0x00400000
#define RVL3DSURFACE_FLAG_BOUNDARY						0x00800000

#define RVL3DSURFACE_SAMPLE_FLAG_REMOVED				0x01
#define RVL3DSURFACE_SAMPLE_FLAG_OCLUDED				0x02
#define RVL3DSURFACE_SAMPLE_FLAG_MATCHED				0x03

#define RVL3DSURFACE_MATCH4_FLAG_OVERLAP				0x00000001
#define RVL3DSURFACE_MATCH4_FLAG_POSITION				0x00000002

//#define RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
//#define RVL3DSURFACE_MODEL_2

class CRVL3DSurface2;

struct RVLSURFACE_MATCH
{
	void *vpSObject;
	CRVL3DObject *pMObject;
};

struct RVLSURFACE_MATCH_ARRAY
{
	CRVL3DObject *pSObject;
	RVLSURFACE_MATCH *pFirst;
	RVLSURFACE_MATCH *pLast;
	
	//used to store temp values in EKF of surfaces
	double m_e[3];
	double m_C[18];		//3 * 6
	double m_Q[9];		//3 * 3
};

struct RVL3DSURFACE_SAMPLE
{
	int Index;
	CRVL3DSurface2 *pSurface;
	double X[3];
	double wz;
	double V[3];
	double stdX;
	double stdN[2];
	RVLSIGNATURE_ELEMENT *Signature;
	int SignatureSize;
	void *pNext;
	BYTE Flags;
};

struct RVL3DSURFACE2_MATCH_DATA
{
	double *Cp;
	double *Cp_;
	double *invCp;
	double *invCp_;
	double varPositionUncert;
	double varOrientationUncert;
	double PPriorPosition;
	double POrientMatch;
};

void RVL3DSurfacesRotate(CRVLMPtrChain *p3DSurfaceList,
						 double *Rot);

void RVLProject2DLineTo3DPlane(double *H,									
							   double csIn, double snIn, double rhoIn,		
							   double &csOut, double &snOut, double &rhoOut);
void RVL3DSurfaceInvTransf(double *NSrc,
						   double dSrc,
						   CRVL3DPose *pPose,
						   double *NTgt,
						   double &dTgt);

void RVLProject2DPointTo3DPlane(int *i2DPoint, 
								double *d3DPoint,
								CRVL3DSurface2 *p3DPlane,
								CRVLCamera *pCamera);

void RVLProject2DPointTo3DPlane(int *i2DPoint, 
								double *d3DPoint,
								CRVL3DSurface2 *p3DPlane,
								double Uc, double Vc,
								double Fu, double Fv);
void RVLProject3DPointTo3DPlane(
	double *XSrc,
	CRVL3DSurface2 *p3DPlane,
	double *XTgt);
BOOL RVL3DPlanarSurfaceEKFUpdate(	CRVL3DSurface2 *pSSurf,
									CRVL3DSurface2 *pMSurf,
									CRVL3DPose *pInitPose,
									CRVL3DPose *pFinalPose,
									RVLSURFACE_MATCH_ARRAY *pMatchData,
									bool bCheckConsistency = true);
																		

class CRVL3DSurface2 : public CRVL3DMeshObject //CRVL3DObject2  
{
public:
	CRVL3DPose m_Pose;
	double m_N[3];  //plane/surface normal
	double m_d;		//plane/surface rho
	int m_nSupport;
	double m_EigenValues[2];
	double m_varq[3];
	//double m_EigenVectors[6];
	double m_sigmaR;
	double m_PoseContribution[2 * 3 * 3];
	double m_epsilon1;
	double m_epsilon2;
	double m_Area;
	void *m_vp2DRegion;
	double m_PoseInformation;
	RVLQLIST m_Samples;
	RVLQLIST m_BoundaryContourList;
	//double m_Cp[9];

	//double m_Centroid[3];
	//double m_CN[9];
	//double m_vard;
	//CRVL2DRegion2 *m_p2DRegion;

public:
	CRVL3DSurface2();
	virtual ~CRVL3DSurface2();
	CRVLObject2 * Create2(CRVLClass * pClass);
	void Rotate(double *Rot);
	void Crop(	CRVL3DPose *pPoseC0,
				CRVLCamera *pCamera,
				RVLRECT *pROI,
				RVL3DCONTOUR_CROP_PARAMS *pCropLTs,
				CvPoint **pPt2DArray,
				CRVLClass *p2DContourSet);
	void Save(	FILE *fp,
				DWORD Flags = 0x00000000);
	void Load(	FILE *fp,
				DWORD Flags = 0x00000000);
	BOOL Match(	CRVL3DObject *pMObject, 
				CRVL3DPose *pPose,
				double &MatchQuality, void *vpMatchData = NULL);
	BOOL Match2(CRVL3DObject *pMObject, 
				CRVL3DPose *pPose,
				double &MatchQuality, double &detQ, void *vpMatchData = NULL, DWORD Flags = 0x00000000);
	BOOL Match3(CRVL3DObject *pMObject, 
				CRVL3DPose *pPose,
				double &MatchQuality, void *vpMatchData = NULL);
	bool Match4(CRVL3DObject *pObject_, 
				RVL3DSURFACE2_MATCH_DATA *pData,
				double &MatchQuality,
				DWORD Flags = (RVL3DSURFACE_MATCH4_FLAG_OVERLAP | RVL3DSURFACE_MATCH4_FLAG_POSITION));
	void GetPoseContribution(void);
	void TransfToMatchRefFrame(	double *RFT,
								double *RPT,
								double *CP);
	void Transf(
		CRVL3DPose * pPose,
		int iView = 0);
};

extern CRVL3DSurface2 RVL3DSurfaceTemplate;

#endif // !defined(AFX_RVL3DSURFACE2_H__2C1A94DD_51D5_4ECF_8DDC_1304987BAA4E__INCLUDED_)
