// RVL2DContour.h: interface for the CRVL2DContour class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL2DCONTOUR_H__206A48F0_7BBF_4634_B5CD_D018378BADCE__INCLUDED_)
#define AFX_RVL2DCONTOUR_H__206A48F0_7BBF_4634_B5CD_D018378BADCE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVL2DObject.h"

#define RVL2DCONTOUR_PARAM_FLAG_IU	0x00010000
#define RVL2DCONTOUR_PARAM_FLAG_DIU	0x00020000
#define RVL2DCONTOUR_DETECT_FLAG_MIN_LEN	0x01
#define RVL2DCONTOUR_FOLLOW_FLAG_BUFFER		0x01
#define RVL2DCONTOUR_FOLLOW_FLAG_LEN_LIMIT	0x02
#define RVLRELLIST_INDEX_2DCONTOUR_CONTOUR_SEGMENTS	3
#define RVL2DCONTOUR_DETECT_RES_CONTOUR		0x01
#define RVL2DCONTOUR_DETECT_RES_MARKED		0x02

#define RVLEDGE_ELEMENT_FLAG_BRANCH			0x01
#define RVLEDGE_ELEMENT_FLAG_BORDER			0x02
//#define RVLEDGE_ELEMENT_FLAG_TERMINATING	0x02
//#define RVLEDGE_ELEMENT_FLAG_MULTIBRANCH	0x02
#define RVLEDGE_ELEMENT_FLAG_CONTOUR		0x04
#define RVL2DCONTOUR_UP						0x00
#define RVL2DCONTOUR_RIGHT					0x01
#define RVL2DCONTOUR_DOWN					0x02
#define RVL2DCONTOUR_LEFT					0x03
#define RVL2DCONTOUR_DIRECTION				0x03
#define RVL2DCONTOUR_PREV					0x04
#define RVL2DCONTOUR_NEXT					0x08
#define RVL2DCONTOUR_VISITED				0x10
#define RVL2DCONTOUR_STOP					0x20
#define RVL2DCONTOUR_VOID					0x40
#define RVL2DCONTOUR_EDGE					0x80

#define RVL2DCONTOUR_GET_NEXT_EDGEID(pContourElement)	((*pContourElement) & RVL2DCONTOUR_DIRECTION)
#define RVL2DCONTOUR_GET_NEXT(pContourElement, pAImage, iEdge, iNextEdge)\
	pContourElement + pAImage->m_dpContourFollow[iEdge][iNextEdge]
#define RVL2DCONTOUR_GET_IPIX(pContourElement, ContourMap)	((pContourElement - ContourMap) / 4)
#define RVL2DCONTOUR_GET_EDGEID(pContourElement, ContourMap) ((pContourElement - ContourMap) & 3)

class CRVL2DContour;

struct RVL2DCONTOUR_SEGMENT;

struct RVLEDGE_ELEMENT
{
	BYTE Flags;
	int u,v;
	double dIu, dIv;
	//double std;
	int w;
	int n;
	BYTE *vpPix;
	BYTE idIDirection;
	int nNeighbors;
	RVLEDGE_ELEMENT **NeighborPtrArray;
	int nContourNeighbors;
	RVLEDGE_ELEMENT *pContourNeighbor[2];
	CRVLMChain *pTriangleSideArray;
	RVL2DCONTOUR_SEGMENT *p2DContourSeg;
	RVLARRAY *pBranchPtrArray;
};

struct RVL2DCONTOUR_SEGMENT
{
	BYTE Flags;
	WORD Index;
	RVLEDGE_ELEMENT *pLastEdgeElement;
	CRVL2DContour *p2DContour;
	RVLARRAY NeighborArray;
	RVL2DCONTOUR_SEGMENT *pPrev, *pNext;
	double cs, sn, len;
};

struct RVL2DCONTOUR_DATA
{
	int iPix0;
	BYTE *Data;
	int n;
	void *pNext;
};

struct RVL2DCONTOUR_DATA2
{
	BYTE *pFirst;
	BYTE *pLast;
	void *pNext;
};

void RVLCreateC2DContour(CRVLClass *pClass);
BYTE RVL2DEdgeContourElementDetect(CRVLC2DContour *pClass,
								   void *vpContourDetectData, 
								   BYTE *vpElement, 
								   int Direction);
void RVL2DEdgeContourMarkElement(CRVLC2DContour *pClass,
								 void *vpContourDetectData, 
								 BYTE *vpElement, 
								 int Direction);
void RVL2DEdgeContourOnCreate(CRVLC2DContour *pClass,
							  CRVL2DContour *p2DContour,
							  void *vpContourDetectData);
void RVL2DContourSegment2Lines(RVLIPOINT *pIP1, RVLIPOINT *pIP2,
							   CRVLMPtrChain *pContourIPBuffer,
							   double LineSegmentThr);
void RVL2DEdgeContourSegment2Lines(RVLEDGE_ELEMENT *pIP1, RVLEDGE_ELEMENT *pIP2, 								   
								   int nIn,
							       CRVLMPtrChain *pContourIPBuffer,
							       double LineSegmentThr);
void RVL2DContourEDT(CRVLC2DContour *pClass,
					 CRVLMPtrChain *p2DContourArray,
					 CRVLEDT *pEDT,
					 RVLEDT_PIX_ARRAY *pEDTImage,
					 CRVLMChain *pIPBuffer);
void RVL2DEdgeContourDetect(CRVLMPtrChain *pContourEndList, 
							CRVLC2DContour *pClass,
							CRVLMem *pMem,
							CRVLMem *pMem2,
							double LineSegmentThr);
void RVL2DEdgeContourDisplay(CRVLMPtrChain *p2DContourList,
							 PIX_ARRAY *pIIn,
							 PIX_ARRAY *pIOut);
void RVL2DEdgeContourSegDisplay(CRVLFigure *pFig,
								CRVLMPtrChain *p2DContourList,
								RVL2DCONTOUR_SEGMENT *pSelectedSeg);
void RVL2DEdgeContourSegDisplay(CRVLDisplayVector *pVector,
								RVL2DCONTOUR_SEGMENT *pSelectedSeg);
void RVL2DContourNeighbors(void *vpAImage);
void RVLConnect2DContours(CRVLMPtrChain *pContourEndList,
						  CRVLC2DContour *pC2D,
						  CRVLMem *pMem,
						  CRVLMem *pMem2);
RVLEDGE_ELEMENT *RVL2DContourGetNextBranch(RVLEDGE_ELEMENT *pBranching,
										   RVLEDGE_ELEMENT *pEdgeElementIn);
BOOL RVLIsInsideContour(CRVLMPtrChain *pContourList,
						int nContours,
						int u, int v);
BOOL RVLIsInsideContour(CvPoint *PtArray,
						int nPts,
						int u, int v);
void RVLDetectDepthDiscontinuityContours(short *Depth,
										 int w, int h,
										 unsigned int Format,
										 short DepthDiscontinuityThr,
										 int minSize,
										 CRVLMem *pMem,
										 void *vpAImage,
										 RVLQLIST *pContourList,
										 BYTE *ContourMap = NULL);
void RVLDepthContourSegmentToLines(	RVL2DCONTOUR_DATA2 *pContour,
									BYTE *ContourMap,
									RVL3DPOINT2 **Pt3DMap,
									double Tol,
									CRVLMem *pMem,
									void *vpAImage,
									RVLQLIST *pVertexList,
									int &nVertices);
void RVL2DContourDisplay(IplImage *pDisplay,
						 RVLQLIST *pContourList,
						 BYTE *ContourMap,
						 void *vpAImage);

class CRVL2DContour : public CRVL2DObject
{
public:
	//int m_iU[2][2];
	//int m_diU[2];
	//int m_leniU;
	RVLIPOINT *m_ContourIPArray;
	int m_nContourIPs;

public:
	CRVLObject2 * Create2(CRVLClass *pClass);
	void Display(CRVLDisplayVector *pVector);
	void Segment2Lines(CRVLMPtrChain *pContourIPBuffer,
					   double LineSegmentThr);
	//void UpdateVoronoiPointList(CRVLMChain *VoronoiPointList, 
	//	int &maxq);
	//void SetdiU();
	//void UpdateParams();
	//void VoronoiSignificant();
	CRVL2DContour();
	virtual ~CRVL2DContour();

};

extern CRVL2DContour RVL2DContourTemplate;

#endif // !defined(AFX_RVL2DCONTOUR_H__206A48F0_7BBF_4634_B5CD_D018378BADCE__INCLUDED_)
