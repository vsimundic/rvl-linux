#pragma once
#include "RVL2DObject.h"

void RVL2DLine3Save(FILE *fp,
					CRVLC2D *p2DLineSet,
					int ImageWidth);
void RVLCreateCrop2DLineLT(BYTE *Crop2DLineLT);
BYTE RVLCrop2DLine(int u1, int v1,
				   int u2, int v2,
				   RVLRECT *pROI,
				   BYTE *bOutLT,
				   CvPoint *pTgtPt1,
				   CvPoint *pTgtPt2,
				   BYTE &CropSide);

class CRVL2DLine3 : public CRVL2DObject
{
public:
	int m_iPix[2];
	void *m_vpLink[2];

public:
	CRVL2DLine3(void);
	virtual ~CRVL2DLine3(void);
	CRVLObject2 * Create2(CRVLClass * pClass);
};

extern CRVL2DLine3 RVL2DLine3Template;
