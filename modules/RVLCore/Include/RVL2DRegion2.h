// RVL2DRegion2.h: interface for the CRVL2DRegion2 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL2DREGION2_H__82CC44F0_8CD1_446D_B107_69795EBC8481__INCLUDED_)
#define AFX_RVL2DREGION2_H__82CC44F0_8CD1_446D_B107_69795EBC8481__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVL2DObject.h"

#define RVL2DREGION_FLAG_GROUND			0x00010000
#define RVL2DREGION_FLAG_WALL			0x00020000
#define RVL2DREGION_FLAG_BOUNDARY		0x00040000 
#define RVL2DREGION_FLAG_MARKED			0x00100000
#define RVL2DREGION_FLAG_MARKED2		0x00200000
#define RVL2DREGION_FLAG_MARKED3		0x00400000
#define RVL2DREGION_FLAG_THICK			0x00800000
#define RVL2DREGION_FLAG_LINE			0x01000000
#define RVL2DREGION_FLAG_INVISIBLE		0x02000000

#define RVL2DREGION_DISPLAY_FLAG_BOUNDARY	0x08

struct RVL2DREGION_PIXIDX
{
	int iPix;
	RVL2DREGION_PIXIDX *pNext;
};

class CRVL2DRegion2;

struct RVLTEXTON
{
	double x, y;
	double Size;
	double Elongation;
	double Orientation;
	int AvgRelI;
	int nPts;
	void *pNext;
};

struct RVLSIGNATURE_ELEMENT
{
	CRVL2DRegion2 *pTexton;
	int n;
};

struct RVLAPIX_2DREGION_PTR
{
	CRVL2DRegion2 *p2DRegion;
	RVLAPIX_2DREGION_PTR *pNext;
};

int RVLGetSignature(RVLAPIX_2DREGION_PTR *TextonMap,
					int Width,
					RVLRECT *pROI,
					RVLSIGNATURE_ELEMENT *Signature,
					RVLSIGNATURE_ELEMENT **TextonSignatureMap);
void RVLDisplayTextons(RVLQLIST *pTextonList,
					   double res,
					   double minuTxt,
					   double minvTxt,
					   double hTxt,
					   IplImage *pOutImage,
					   CRVLMem *pMem2);
void RVLDrawEllipse(double u0, double v0, 
			     	double r1, double r2, 
					double phi, 
					double maxErr,
					CRVLMChain *pIPArray);

void RVLEdgeBasedImageSegmentation(void *vpAImage,
								   int DepthThr,
								   int *RegionGrowingMap,
								   int *Segment,
								   int *RegionGrowingBuff,
								   int *RegionGrowingBuff2,
								   int *EmptyRegionGrowingMap
								   );
void RVLEdgeBasedSegmentationDisplay(	CRVLGUI *pGUI,
										CRVLFigure *pFig,
										//int iFirstTextLine,
										WORD *Segment,
										void *vpAImage,
										RVLAPIX *pSelectedAPix,
										PIX_ARRAY *pIOut
										);
void RVLDisplay2DRegions(CRVLFigure *pFig,
						 CRVLMPtrChain *p2DRegionSet,
						 int ImageWidth,
						 RVLCOLOR Color,
						 int LineWidth = 1,
						 DWORD mMask1 = 0x00000000, DWORD mMask2 = 0x00000000);
void RVLDisplayConvex(	 CRVLFigure *pFig,
						 CRVLMPtrChain *p2DRegionSet,
						 int ImageWidth,
						 RVLCOLOR ForegroundColor,
						 RVLCOLOR BackgroundColor);
CRVL2DRegion2 *RVLSelectTriangle(int iPix,
								 int ImageWidth,
								 CRVLMPtrChain *p2DRegionSet);
void RVL2DRegionResetLabels(CRVLMPtrChain *pTriangleList);

class CRVL2DRegion2 : public CRVL2DObject
{
public:
	double m_a, m_b, m_c;
	void *m_PtArray;
	int m_nPts;
	int m_n3DPts;
	RVL3DPOINT2 **m_pPoint3DArray;
	void *m_vp3DSurface;
	double m_Tol;
	double m_std;
	int m_N[3];
	int m_lenN;
	int m_d;
	double m_fN[3];
	double m_rho;
	int m_X0[3];
	void *m_vpQueueEntry;
	int m_Size;
	DWORD m_Label;
	RVLQLIST m_BoundaryContourList;
	int m_SI;
	int m_Sr, m_Sg, m_Sb;
	int m_nEdges;
	int m_SINeighborhood;
	RVLQLIST *m_histRGB;
	float m_histRGB_base[3];
	int m_ColorSystem;
	int m_noHistDimensions;
	RVLQLIST *m_rgbHOG9;
	RVLQLIST *m_LBP;
	RVLQLIST *m_LBP_RIU;
	RVLQLIST *m_LBP_RIU_VAR;
	int m_nVertices;
	RVL3DPOINT2 **m_pVertexArray;
	BYTE *m_uPixMask;
	RVLQLIST m_Samples;
	RVL2DMOMENTS m_Moments;
	RVL3DPOINT2 **m_pPoint3DMap;

public:
	void DisplayPt(	int iPix,
					PIX_ARRAY *pIOut,
					void *vpAImage,	
					BYTE Flags);
	void Display(	PIX_ARRAY *pIOut,
					void *vpAImage,
					BYTE Flags = 0x02);
	void Display2(	PIX_ARRAY *pIOut,
					void *vpAImage,	
					BYTE Flags);
	CRVLObject2 * Create2(CRVLClass *pClass);
	CRVL2DRegion2();
	virtual ~CRVL2DRegion2();

};

extern CRVL2DRegion2 RVL2DRegionTemplate;

#endif // !defined(AFX_RVL2DREGION2_H__82CC44F0_8CD1_446D_B107_69795EBC8481__INCLUDED_)
