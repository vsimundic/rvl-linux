#pragma once

#define RVLMESH_LINK_FLAG_BOUNDARY		0x00000001
#define RVLMESH_LINK_FLAG_EDGE			0x00000002
#define RVLMESH_LINK_FLAG_END				0x00000004
#define RVLMESH_LINK_FLAG_VISITED			0x00000008
#define RVLMESH_LINK_FLAG_MARKED			0x00000010
#define RVLMESH_LINK_FLAG_MARKED2			0x00000020
#define RVLMESH_LINK_CONCAVITY			0x00000040
#define RVLMESH_LINK_FLAG_REJECTED		0x00000080
#define RVLMESH_LINK_FLAG_ROI				0x00000100
#define RVLMESH_LINK_FLAG_FOREGROUND		0x00010000
#define RVLMESH_LINK_FLAG_BACKGROUND		0x00020000
#define RVLMESH_LINK_FLAG_SELECTED		0x00040000
//#define RVLMESH_LINK_FLAG_INTERNAL		0x40

#define RVLDELAUNAY_GET_LINK(DelaunayMap, iPix) ((RVLMESH_LINK *)(((RVLQLIST_PTR_ENTRY *)(DelaunayMap[iPix]))->Ptr))

struct RVLDELAUNAY_NEIGHBOR_ANGLE
{
	int du, dv;
	char quadrant;
};

struct RVLMESH_LINK
{
	int iPix;
	int iPix0;
	short iConnection;
	RVLMESH_LINK *pOpposite;
	RVLMESH_LINK *pNext;
	RVLMESH_LINK *pPrev;
	void *vp2DLine;
	void *vp2DRegion;
	DWORD Flags;
	int du, dv;
	int len;
	void *pData;
};

BOOL RVLPolygonConvexityTest(RVLMESH_LINK *pLink0);
void RVLSetFlags(RVLMESH_LINK *pDelaunayLink0,
				 BYTE Flags);
void RVLResetFlags(	RVLMESH_LINK *pDelaunayLink0,
					BYTE Flags);
void RVLDisplay2DRegion(CRVLFigure *pFig,
						RVLMESH_LINK *pDelaunayLink0,
						int ImageWidth,
						RVLCOLOR Color,
						int LineWidth = 1,
						DWORD mMask1 = 0x00000000, DWORD mMask2 = 0x00000000, 
						BOOL bMask = FALSE,
						BYTE BoundaryFlags = 0x00,
						bool bAvoidDoubleEdges = true);
BOOL RVLFlipDiagonal(RVLMESH_LINK *pLink,
					 RVLMESH_LINK **LinkArray = NULL);


class CRVLDelaunay
{
public:
	int m_Width, m_Height;
	BYTE **m_DelaunayMap;
	BYTE *m_DelaunayData;
	RVLRECT *m_pROI;
	int m_nLinks;
	RVLQLIST m_LinkList;
	RVLQLIST m_VertexList;
	RVLQLIST_PTR_ENTRY *m_VertexListData;
	int m_nVertices;

private:
	RVLDELAUNAY_NEIGHBOR_ANGLE *m_NeighborAngleArray;

public:
	CRVLDelaunay(void);
	~CRVLDelaunay(void);
	void Apply(RVLEDT_PIX_ARRAY *pEDTImage);
	void Init();
	void Clear(void);
	void Save(FILE * fp, BYTE mMask = 0xff);
	BOOL Test();
	BOOL Test2();
	BOOL Test3();
	int *SelectRegion(	int iSelectedPix,
						BYTE BoundaryFlags,								
						int &nPts,
						CRVLMem *pMem);
	void DisplayRegion(	CRVLFigure *pFig,	
						int iSelectedPix,
						BYTE BoundaryFlags);
	void GetPtsInConvexPolygon(	RVLMESH_LINK *pLink0,
								int * iScanLineStart, 
								int * iScanLineEnd, 
								int &iFirstScanLine,
								int &iLastScanLine);
	void GetPtsInConvexHull(	RVLMESH_LINK *pLink0,
								int * iScanLineStart, 
								int * iScanLineEnd, 
								int &iFirstScanLine,
								int &iLastScanLine);
	BOOL Test(RVLMESH_LINK *pLink);
	void ResetFlags(DWORD mMask);
	void Display(	CRVLFigure *pFig,
					int ImageWidth,
					RVLCOLOR Color,
					DWORD mMask1, 
					DWORD mMask2);
	void Display2(	CRVLFigure *pFig,
					int ImageWidth,
					RVLCOLOR Color,
					DWORD mMask1, 
					DWORD mMask2);
};
