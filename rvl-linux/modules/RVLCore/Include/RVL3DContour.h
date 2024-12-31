#pragma once
#include "RVL3DObject.h"


struct RVL3DCONTOUR_CROP_PARAMS
{
	BYTE bOutLT[256];
	CvPoint CornerLT[8];
	int auLT[4];
	int avLT[4];
	int iNextSideLT[8];
	double minz;
	double minr;
};

void RVLCrop3DContour(double *Pt3DArray,
					  int n3DPts,
					  CRVL3DPose *pPoseC0,
					  CRVLCamera *pCamera,
					  RVLRECT *pROI,
					  RVL3DCONTOUR_CROP_PARAMS *pCropLTs,
					  CvPoint *Pt2DArray,
					  int *pn2DPts);
void RVLCrop3DContourCreateLTs(RVL3DCONTOUR_CROP_PARAMS *pLTs,
							   RVLRECT *pROI);
void RVLCrop3DContourCreateLTs(RVL3DCONTOUR_CROP_PARAMS *pLTs,
							   CRVLCamera *pCamera);
void RVLCrop3DContourAddPts(BYTE CropSide,
							BYTE CropSide2,
							int dir,
							RVL3DCONTOUR_CROP_PARAMS *pCropLTs,
							int au, int av,
							CvPoint *pPt1,
							CvPoint *&pPt,
							CvPoint *&pPtPrev);
int RVLCrop3DContourGetCropSide(	int u,
									RVLRECT *pROI,
									BYTE CropSide);
void RVLCrop3DContourUpdateDirection(CvPoint *pPtOut1, CvPoint *pPtOut2, 
									 int u0, int v0,
									 int au, int av,
									 int &dir);


class CRVL3DContour :
	public CRVL3DObject
{
public:
	CRVL3DContour(void);
	virtual ~CRVL3DContour(void);
	CRVLObject2 * Create2(CRVLClass * pClass);
	void Save(	FILE * fp,
				DWORD Flags = 0x00000000);
	void Load(	FILE * fp,
				DWORD Flags = 0x00000000);

public:
	void *m_PtArray;
	int m_nPts;
};

extern CRVL3DContour RVL3DContourTemplate;
