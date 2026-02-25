#include "RVLCore.h"
#include "RVLC2D.h"
#include "RVL2DLine3.h"

CRVL2DLine3 RVL2DLine3Template;

CRVL2DLine3::CRVL2DLine3(void)
{
}

CRVL2DLine3::~CRVL2DLine3(void)
{
}

void RVL2DLine3Save(FILE *fp,
					CRVLC2D *p2DLineSet,
					int ImageWidth)
{
	CRVL2DLine3 *p2DLine;

	p2DLineSet->m_ObjectList.Start();

	while(p2DLineSet->m_ObjectList.m_pNext)
	{
		p2DLine = (CRVL2DLine3 *)(p2DLineSet->m_ObjectList.GetNext());

		fprintf(fp, "%d\t%d\t%d\t%d\n", 
			p2DLine->m_iPix[0] % ImageWidth,
			p2DLine->m_iPix[0] / ImageWidth,
			p2DLine->m_iPix[1] % ImageWidth,
			p2DLine->m_iPix[1] / ImageWidth);
	}
}

CRVLObject2 * CRVL2DLine3::Create2(CRVLClass * pClass)
{
	CRVL2DLine3 *pObject = (CRVL2DLine3 *)(pClass->m_pMem0->Alloc(sizeof(CRVL2DLine3)));

	memcpy(pObject, this, sizeof(CRVL2DLine3));

	pObject->CRVLObject2::Create(pClass);

	pClass->Add(pObject);

	return pObject;		
}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 


void RVLCreateCrop2DLineLT(BYTE *Crop2DLineLT)
{
	int i;
	BYTE mOut;

	for(i = 0; i < 256; i++)
	{
		mOut = (BYTE)i;

		Crop2DLineLT[i] = 0x00;

		if(	(mOut & 0x11) == 0x11 || 
			(mOut & 0x22) == 0x22 ||
			(mOut & 0x44) == 0x44 ||
			(mOut & 0x88) == 0x88)
			Crop2DLineLT[i] = 0x04;
		else
		{
			if(mOut & 0x0f)
				Crop2DLineLT[i] |= 0x01;

			if(mOut & 0xf0)
				Crop2DLineLT[i] |= 0x02;
		}				
	}
}

BYTE RVLCrop2DLine(int u1, int v1,
				   int u2, int v2,
				   RVLRECT *pROI,
				   BYTE *bOutLT,
				   CvPoint *pTgtPt1,
				   CvPoint *pTgtPt2,
				   BYTE &CropSide)
{
	BYTE mOut = ((BYTE)(u1 < pROI->left) 
		| ((BYTE)(u1 > pROI->right) << 1)
		| ((BYTE)(v1 < pROI->top) << 2)
		| ((BYTE)(v1 > pROI->bottom) << 3)
		| ((BYTE)(u2 < pROI->left) << 4)
		| ((BYTE)(u2 > pROI->right) << 5)
		| ((BYTE)(v2 < pROI->top) << 6)
		| ((BYTE)(v2 > pROI->bottom) << 7));

	BYTE bOut = bOutLT[mOut];
	
	if(bOut == 0x04)
		return bOut;

	pTgtPt1->x = u1;
	pTgtPt1->y = v1;
	pTgtPt2->x = u2;
	pTgtPt2->y = v2;

	if(mOut == 0)
		return 0x00;

	int du = u2 - u1;
	int dv = v2 - v1;

	int p1 = u1;
	int p2 = u2;
	int q1 = v1;
	int q2 = v2;
	int dp = du;
	int dq = dv;
	int Margin1 = pROI->top;
	int Margin2 = pROI->bottom;

	int *pp1 = &(pTgtPt1->x);
	int *pq1 = &(pTgtPt1->y);
	int *pp2 = &(pTgtPt2->x);
	int *pq2 = &(pTgtPt2->y);

	int *pMargin = (int *)pROI;

	BYTE mOut2 = 0x11;

	int dir = 1;

	BYTE bCut = 0x00;

	CropSide = 0x00;

	int iSide;
	int pCut, qCut;

	for(iSide = 0; iSide < 4; iSide++, pMargin++, mOut2 = (mOut2 << 1), dir = -dir)
	{
		if(mOut & mOut2)
		{
			pCut = *pMargin;
			qCut = (((dq * (*pMargin - p1) + dp * q1) / (dp << 1)) << 1) + 1;

			if(qCut >= Margin1 && qCut <= Margin2)
			{
				if(dir * dp > 0)
				{
					*pp1 = pCut;
					*pq1 = qCut;	

					CropSide &= 0xfc;
					CropSide |= (BYTE)iSide;

					bCut |= 0x01;
				}
				else
				{
					*pp2 = pCut;
					*pq2 = qCut;

					CropSide &= 0xf3;
					CropSide |= ((BYTE)iSide << 2);

					bCut |= 0x02;
				}
			}
		}

		if(iSide == 1)
		{
			p1 = v1;
			p2 = v2;
			q1 = u1;
			q2 = u2;
			dp = dv;
			dq = du;
			Margin1 = pROI->left;
			Margin2 = pROI->right;
			pp1 = &(pTgtPt1->y);
			pq1 = &(pTgtPt1->x);
			pp2 = &(pTgtPt2->y);
			pq2 = &(pTgtPt2->x);
		}
	}

	return (bCut == bOut ? bOut : 0x04);
}