#include "RVLCore.h"
#include "RVLClass.h"
#include "RVLC2D.h"
//#include "RVL2DObject3.h"
#include "RVL2DLine3.h"
#include "RVL3DLine2.h"
#include "RVL3DContour.h"
#include "RVLRelation.h"

CRVL3DContour RVL3DContourTemplate;

CRVL3DContour::CRVL3DContour(void)
{
}

CRVL3DContour::~CRVL3DContour(void)
{
}

CRVLObject2 * CRVL3DContour::Create2(CRVLClass * pClass)
{
	CRVL3DContour *pObject = (CRVL3DContour *)(pClass->m_pMem0->Alloc(sizeof(CRVL3DContour)));

	memcpy(pObject, this, sizeof(CRVL3DContour));

	pObject->CRVLObject2::Create(pClass);

	pClass->Add(pObject);

	return pObject;	
}

void CRVL3DContour::Save(	FILE * fp,
							DWORD Flags)
{
	//fprintf(fp, "%d\n", m_nPts);

	//double *X = (double *)m_PtArray;

	//double *pPtArrayEnd = X + 3 * m_nPts;

	//for(; X < pPtArrayEnd; X += 3)
	//	fprintf(fp, "%lf\t%lf\t%lf\n", X[0], X[1], X[2]);

	fwrite(&m_nPts, sizeof(int), 1, fp);
	fwrite(m_PtArray, sizeof(double), 3 * m_nPts, fp);
}

void CRVL3DContour::Load(	FILE * fp,
							DWORD Flags)
{
	//fscanf(fp, "%d\n", &m_nPts);

	//double *X = (double *)(m_pClass->m_pMem0->Alloc(3 * m_nPts * sizeof(double)));
	//	
	//m_PtArray = X;

	//double *pPtArrayEnd = X + 3 * m_nPts;

	//for(; X < pPtArrayEnd; X += 3)
	//	fscanf(fp, "%lf\t%lf\t%lf\n", X, X + 1, X + 2);

	fread(&m_nPts, sizeof(int), 1, fp);

	m_PtArray = (double *)(m_pClass->m_pMem0->Alloc(3 * m_nPts * sizeof(double)));
	//m_PtArray = X;
	fread(m_PtArray, sizeof(double), 3 * m_nPts, fp);

}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 


void RVLCrop3DContour(double *Pt3DArray,
					  int n3DPts,
					  CRVL3DPose *pPoseC0,
					  CRVLCamera *pCamera,
					  RVLRECT *pROI,
					  RVL3DCONTOUR_CROP_PARAMS *pCropLTs,
					  CvPoint *Pt2DArray,
					  int *pn2DPts)
{
	BYTE *bOutLT = pCropLTs->bOutLT;
	CvPoint *CornerLT = pCropLTs->CornerLT;
	int *auLT = pCropLTs->auLT;
	int *avLT = pCropLTs->avLT;
	int *iNextSideLT = pCropLTs->iNextSideLT;
	double minz = pCropLTs->minz;
	double minr = pCropLTs->minr;

	double *X1 = Pt3DArray;

	double XBuff[6];
	double *X1C = XBuff;
	double *X2C = XBuff + 3;

	pPoseC0->InvTransf(X1, X1C);

	CvPoint *pPt = Pt2DArray;

	BYTE CropSidePrev = 0x00;

	BYTE CropSideFirst = 0x80;

	BYTE Flags = 0x00;

	int iPt = 0;
	int iPt0 = 0;

	int u0 = 1;
	int v0 = 2;
	int au = -1;
	int av = 0;
	int dir = 1;

	CvPoint Pt1, Pt2;
	CvPoint PtOut0, PtOut1, PtOut2;
	BYTE bOut;
	BYTE CropSide, CropSide2;
	double *X2;
	int iU1[2], iU2[2];
	BOOL bContinue;
	CvPoint *pPtPrev;
	double *pTmp;
	BYTE bOut2;	

	while(TRUE)
	{
		X2 = Pt3DArray + 3 * ((iPt + 1) % n3DPts);

		pPoseC0->InvTransf(X2, X2C);

		bOut = RVLCrop3DLine(X1C, X2C, pCamera, pROI, minz, minr, bOutLT, iU1, iU2, &Pt1, &Pt2, CropSide);

		if((Flags & 0x01) == 0)
			if((bOut & 0x04) == 0)
			{
				Flags = 0x01;

				pPt->x = Pt1.x;
				pPt->y = Pt1.y;

				pPtPrev = pPt;

				pPt++;

				iPt0 = iPt;
			}

		if(Flags & 0x01)
		{
			if(bOut)
			{
				if((bOut & 0x18) != 0x18)
				{
					bContinue = TRUE;

					while(bContinue)
					{
						if((bOut & 0x18) == 0x08)
						{
							PtOut2.x = iU1[0];
							PtOut2.y = iU1[1];

							bOut &= ~0x08;

							if(Flags & 0x02)
								RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut2, u0, v0, au, av, dir);
						}
						else if(bOut & 0x04)
						{
							PtOut1.x = iU1[0];
							PtOut1.y = iU1[1];
							PtOut2.x = iU2[0];
							PtOut2.y = iU2[1];

							if(Flags & 0x02)
								RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut2, u0, v0, au, av, dir);

							bContinue = FALSE;
						}
						else if(bOut & 0x01)
						{
							PtOut1.x = iU1[0];
							PtOut1.y = iU1[1];
							PtOut2.x = Pt1.x;
							PtOut2.y = Pt1.y;	

							bOut &= ~0x01;

							if(Flags & 0x02)
							{
								RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut2, u0, v0, au, av, dir);

								RVLCrop3DContourAddPts(CropSide, CropSide2, dir, pCropLTs, au, av, &Pt1, pPt, pPtPrev);

								Flags &= 0xfd;

								if(pPtPrev->x == Pt2DArray->x)
									if(pPtPrev->y == Pt2DArray->y)
										break;
							}
						}
						else
						{
							if(Pt2.x != pPtPrev->x || Pt2.y != pPtPrev->y)
							{
								pPt->x = Pt2.x;
								pPt->y = Pt2.y;

								pPtPrev = pPt;

								pPt++;

								if(pPtPrev->x == Pt2DArray->x)
									if(pPtPrev->y == Pt2DArray->y)
										break;
							}

							if(bOut & 0x02)
							{
								CropSide2 = RVLCrop3DContourGetCropSide(Pt2.x, pROI, CropSide);

								au = auLT[CropSide2];
								av = avLT[CropSide2];

								u0 = Pt2.x - av;
								v0 = Pt2.y + au;	

								PtOut1.x = Pt2.x;
								PtOut1.y = Pt2.y;
								PtOut2.x = iU2[0];
								PtOut2.y = iU2[1];

								dir = 1;

								Flags |= 0x02;

								RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut2, u0, v0, au, av, dir);
							}

							bContinue = FALSE;
						}
					}

					if((bOut & 0x18) == 0x10)
					{
						PtOut1.x = iU2[0];
						PtOut1.y = iU2[1];
					}	
				}
			}
			else
			{
				if(Pt2.x != pPtPrev->x || Pt2.y != pPtPrev->y)
				{
					pPt->x = Pt2.x;
					pPt->y = Pt2.y;

					pPtPrev = pPt;

					pPt++;
				}
			}
		}
		else	// (Flags & 0x01) == 0
		{
			bOut2 = bOut & 0x18;

			if(bOut2 != 0x18)
			{
				Flags |= 0x08;

				if(bOut2 == 0x08)
				{
					if(Flags & 0x04)
					{
						PtOut2.x = iU1[0];
						PtOut2.y = iU1[1];

						RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut2, u0, v0, au, av, dir);
					}
					else
					{
						PtOut0.x = iU1[0];
						PtOut0.y = iU1[1];
					}

					PtOut1.x = iU1[0];
					PtOut1.y = iU1[1];
					PtOut2.x = iU2[0];
					PtOut2.y = iU2[1];

					RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut2, u0, v0, au, av, dir);

					Flags &= 0xfb;
				}
				else if(bOut2 == 0x10)
				{
					PtOut1.x = iU1[0];
					PtOut1.y = iU1[1];
					PtOut2.x = iU2[0];
					PtOut2.y = iU2[1];

					RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut2, u0, v0, au, av, dir);

					PtOut1.x = iU2[0];
					PtOut1.y = iU2[1];

					Flags |= 0x04;
				}
				else
				{
					PtOut1.x = iU1[0];
					PtOut1.y = iU1[1];
					PtOut2.x = iU2[0];
					PtOut2.y = iU2[1];

					RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut2, u0, v0, au, av, dir);

					Flags &= 0xfb;
				}
			}
		}

		X1 = X2;

		pTmp = X1C;
		X1C = X2C;
		X2C = pTmp;

		iPt = ((iPt + 1) % n3DPts);

		if(Flags & 0x01)
		{
			if(pPtPrev->x == Pt2DArray->x && pPtPrev->y == Pt2DArray->y)
			{
				if(Flags & 0x10)
					break;
			}
			else
				Flags |= 0x10;
		}

		if((Flags & 0x10) == 0)
			if(iPt == iPt0)
				break;
	}

	if((Flags & 0x01) == 0)
	{
		if(Flags & 0x08)
		{
			if(Flags & 0x04)
				RVLCrop3DContourUpdateDirection(&PtOut1, &PtOut0, u0, v0, au, av, dir);

			if(dir < 0)
			{
				CvPoint *pCorner;

				int iSide = 0;

				do
				{
					pCorner = pCropLTs->CornerLT + iSide;			

					pPt->x = pCorner->x;
					pPt->y = pCorner->y;

					pPt++;

					iSide = pCropLTs->iNextSideLT[iSide];

				}
				while(iSide);
			}
		}

		pPt++;
	}

	*pn2DPts = pPt - Pt2DArray - 1;
}

void RVLCrop3DContourCreateLTs(RVL3DCONTOUR_CROP_PARAMS *pLTs,
							   RVLRECT *pROI)
{
	pLTs->auLT[0] = -1;
	pLTs->auLT[1] = 1;
	pLTs->auLT[2] = 0;
	pLTs->auLT[3] = 0;

	pLTs->avLT[0] = 0;
	pLTs->avLT[1] = 0;
	pLTs->avLT[2] = -1;
	pLTs->avLT[3] = 1;

	pLTs->iNextSideLT[0] = 3;
	pLTs->iNextSideLT[1] = 2;
	pLTs->iNextSideLT[2] = 0;
	pLTs->iNextSideLT[3] = 1;
	pLTs->iNextSideLT[4+0] = 2;
	pLTs->iNextSideLT[4+1] = 3;
	pLTs->iNextSideLT[4+2] = 1;
	pLTs->iNextSideLT[4+3] = 0;

	RVLCreateCrop2DLineLT(pLTs->bOutLT);	

	pLTs->CornerLT[0] = cvPoint(pROI->left, pROI->top);
	pLTs->CornerLT[1] = cvPoint(pROI->right, pROI->bottom);
	pLTs->CornerLT[2] = cvPoint(pROI->right, pROI->top);
	pLTs->CornerLT[3] = cvPoint(pROI->left, pROI->bottom);
	pLTs->CornerLT[4+0] = cvPoint(pROI->left, pROI->bottom);
	pLTs->CornerLT[4+1] = cvPoint(pROI->right, pROI->top);
	pLTs->CornerLT[4+2] = cvPoint(pROI->left, pROI->top);
	pLTs->CornerLT[4+3] = cvPoint(pROI->right, pROI->bottom);
}

void RVLCrop3DContourCreateLTs(RVL3DCONTOUR_CROP_PARAMS *pLTs,
							   CRVLCamera *pCamera)
{
	RVLRECT ROI;

	ROI.left = 1;
	ROI.right = 2 * (pCamera->Width - 1) + 1;
	ROI.top = 1;
	ROI.bottom = 2 * (pCamera->Height - 1) + 1;

	RVLCrop3DContourCreateLTs(pLTs, &ROI);

	double a = (pCamera->CenterXNrm > 0.5 * pCamera->Width ? pCamera->CenterXNrm : pCamera->Width - pCamera->CenterXNrm);
	double b = (pCamera->CenterYNrm > 0.5 * pCamera->Height ? pCamera->CenterYNrm : pCamera->Height - pCamera->CenterYNrm);

	pLTs->minr = 1.1 * pLTs->minz * pLTs->minz * ((a * a + b * b) / (pCamera->fNrm * pCamera->fNrm) + 1.0);
}

void RVLCrop3DContourAddPts(BYTE CropSide,
							BYTE CropSide2,
							int dir,
							RVL3DCONTOUR_CROP_PARAMS *pCropLTs,
							int au, int av,
							CvPoint *pPt1,
							CvPoint *&pPt,
							CvPoint *&pPtPrev)
{
	BYTE CropSide1 = (CropSide & 0x03);

	int iSide = CropSide2;

	int LTIndexOffset = ((1 - dir) << 1);

	CvPoint *Corner = pCropLTs->CornerLT + LTIndexOffset;

	int *iNextSide = pCropLTs->iNextSideLT + LTIndexOffset;

	int dist;
	CvPoint *pCorner;

	while(TRUE)
	{
		if(CropSide1 == iSide)
		{
			dist = dir * (-(pPt1->x - pPtPrev->x) * pCropLTs->avLT[iSide]  + (pPt1->y - pPtPrev->y) * pCropLTs->auLT[iSide]);

			if(dist < 0)
			{
				pPt->x = pPt1->x;
				pPt->y = pPt1->y;

				pPtPrev = pPt;

				pPt++;

				break;
			}
			else if(dist == 0)
				break;
		}

		iSide = iNextSide[iSide];

		pCorner = Corner + iSide;

		if(pCorner->x != pPtPrev->x || pCorner->y != pPtPrev->y)
		{
			pPt->x = pCorner->x;
			pPt->y = pCorner->y;

			pPtPrev = pPt;

			pPt++;
		}
	}	
}

int RVLCrop3DContourGetCropSide(	int u,
									RVLRECT *pROI,
									BYTE CropSide)
{
	if(u == pROI->left)
	{
		if((CropSide >> 2) == 3)
			return 0;
	}
	else if(u == pROI->right)
	{
		if((CropSide >> 2) == 2)
			return 1;
	}
		
	return (CropSide >> 2);
}

void RVLCrop3DContourUpdateDirection(CvPoint *pPtOut1, CvPoint *pPtOut2, 
									 int u0, int v0,
									 int au, int av,
									 int &dir)
{
	int du = pPtOut2->x - pPtOut1->x;
	int dv = pPtOut2->y - pPtOut1->y;

	int dens = dv * au - du * av;

	if(dens)
	{
		int nums = dv * (pPtOut1->x - u0) - du * (pPtOut1->y - v0);

		if((nums > 0 && dens > 0) || (nums < 0 && dens < 0))
		{
			int k1 = -av * (pPtOut1->x - u0) + au * (pPtOut1->y - v0);
			int k2 = -av * (pPtOut2->x - u0) + au * (pPtOut2->y - v0);

			if((k1 > 0 && k2 < 0) || (k1 < 0 && k2 > 0))
				dir = -dir;
		}
	}	
}
