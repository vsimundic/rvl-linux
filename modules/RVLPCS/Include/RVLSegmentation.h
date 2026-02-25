// RVLSegmentation.h: interface for the CRVLSegmentation class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLSEGMENTATION_H__9938901F_2DAC_478B_BA31_E568663CF2AD__INCLUDED_)
#define AFX_RVLSEGMENTATION_H__9938901F_2DAC_478B_BA31_E568663CF2AD__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include "RVLEdgeDetect.h"


#define RVLSEGMENT_TO_CONVEX_FLAG_ROI	0x00000001
#define RVLSEGMENT_TO_CONVEX_FLAG_ROI	0x00000001

#define RVLSWER_NODE_FLAG_VISITED					0x00000001
#define RVLSWER_NODE_FLAG_OBJECT					0x00000002

struct RVLCONVEX_HULL_TRIANGLE
{
	RVLMESH_LINK *pLink;
	int iPix;
	RVLCONVEX_HULL_TRIANGLE *pNext;
};

struct RVLCONVEX_HULL_EDGE_DATA
{
	RVLMESH_LINK *pLinkA;
	RVLMESH_LINK *pLinkB;
	BYTE iHullA, iHullB;
};

struct RVLSEGMENT_VMEASURE_DATA
{
	double HCK;
	double HKC;
	double HC;
	double HK;
	double N;
	double A;
	double hom;
	double com;
	double V;
};

struct RVLSWER_NODE
{
	DWORD Flags;
	//RVL3DMOMENTS Moments;
	BYTE *pData;
	RVLQLIST LinkPtrList;
	RVLSWER_NODE *pChild[2];
	RVLSWER_NODE *pParent;
	int Cost;
};

struct RVLSWER_LINK
{
	//DWORD Flags;
	RVLSWER_NODE *pNode[2];
	//RVL3DMOMENTS Moments;
	BYTE *pData;
	void *pNext;
	void **pPtrToThis;
	int Cost;
};

struct RVLSWER_NODE2
{
	DWORD Flags;
	BYTE *pData;
	RVLSWER_NODE2 *pChild[2];
	RVLSWER_NODE2 *pParent;
	float cost;
	int Size;
};

template <class Type> struct RVLSWER_SEGMENT
{
	Type *pNode;
	RVLARRAY_<int> PtList;
};

struct RVLSWER_LINK2
{
	//DWORD Flags;
	RVLSWER_NODE2 *pNode[2];
	//RVL3DMOMENTS Moments;
	BYTE *pData;
	void *pNext;
	void **pPtrToThis;
	float cost;
};

void RVLSaveSegmentation(FILE *fp,
						 CRVLClass *p2DRegionSet,
						 int ImageWidth);
BOOL RVLUpdateConvexHull(CRVLMPtrChain *pTriangleList,
						 CRVLClass *pTriangleSet,
						 int ImageWidth,
						 RVL3DPOINT2 **Point3DMap,
						 RVLMESH_LINK *pLinkNewPt,
						 int maxDist,
						 CRVLMem *pMem,
						 bool bmm = false);
void RVLInitConvexHull(	CRVL2DRegion2 *pTriangleSrc,
						CRVLClass *pTriangleSet,
						int ImageWidth,
						RVL3DPOINT2 **Point3DMap,
						CRVLMem *pMem,
						bool bmm = false);
int RVLGetConvexHull(	CRVL2DRegion2 *pTriangleSrc,
						DWORD Label,
						CRVLMPtrChain *pTriangleList,
						CRVLClass *pTriangleSet,
						int maxDist,
						int ImageWidth,
						RVL3DPOINT2 **Point3DMap,
						CRVLMem *pMem,
						RVLQLIST_PTR_ENTRY *LinkQueueEntryMem,
						//CRVLPlanarSurfaceDetector *pPSD
						CRVL3DMeshObject *childMO = NULL,
						bool bmm = false
						);
int RVLSegmentToConvex( CRVLClass *pTriangleSetSrc,
						CRVL2DRegion2 *pSelectedTriangle,
						CRVLClass *pTriangleSetTgt,
						int maxDist,
						int ImageWidth,
						int ImageHeight,
						RVL3DPOINT2 **Point3DMap,
						CRVLMem *pMem,						
						//CRVLPlanarSurfaceDetector *pPSD,
						int *SizeArray = NULL,
						CRVL3DMeshObject *rootMO = NULL,
						bool bmm = false
						);
BOOL RVLIsVertex(	int iPix,
					CRVLMPtrChain *pTriangleList);
void RVLSegmentToConvex(CRVLDelaunay *pDelaunay,
						double maxLineSegmentErr,
						CRVLQListArray *pDelaunayLinkQueue,
						CRVLMPtrChain *pTriangleList = NULL,
						DWORD Flags = 0x00000000);
//funkcija za snimanje generiranog Convex Hulla
void SaveMeshCH(CRVLClass *p2DRegionSet, RVL3DPOINT2 **Point3DMap, int *IdxMap, int imgSize);
//funkcija za proracunavanje principijalne zakrivljenosti mesha
void CalculatePrincipialCurvature(int noVertices, 
								  RVL3DPOINT2 **Point3DMap, 
								  CRVLClass *p2DRegionSet, 
								  int *Idx,
								  float *Curvatures,
								  float *minD,
								  float *maxD);
//funkcija za brze(???) proracunavanje principijalne zakrivljenosti mesha
void CalculatePrincipialCurvature2(int noVertices, 
								  RVL3DPOINT2 **Point3DMap, 
								  CRVLClass *p2DRegionSet, 
								  float *Curvatures,
								  float *minD,
								  float *maxD);
//funkcija koja generira komplementarnu bazu
void GenerateComplementBasis(double *v1, double *v2, double *v3);
void RVLRemoveInternalVertices(CRVLDelaunay *m_pDelaunay);
void RVLSegmentationEdgesFromLabels(CRVLClass *pTriangleSet, DWORD Mask = 0x00000000, DWORD refMask = 0x00000000);
void RVLSegmentationGetBoundary(
	CRVL2DRegion2 *pSegment,
	int w,
	RVLQLIST *pContourList,
	CRVLMem *pMem);
void RVLSegmentationGTFromMesh(	FILE *fpSrc,
								CRVLDelaunay *pDelaunay,
								FILE *fpTgt);
BOOL RVLImportSegmentation(	FILE *fp,
						    int *Segmentation);
void RVLCompareToGT(int *Segmentation,
					int *GT,
					int w, int h,
					RVLSEGMENT_VMEASURE_DATA *pVMeasureData);
void RVLGetSegmentation( CRVLMPtrChain *pTriangleList,
						 int *Segmentation,
						 int w, int h);
void RVLGetSegmentation( CRVL2DRegion2 **RegionMap,
						 int *Segmentation,
						 int w, int h);
void RVLSaveSegmentation(FILE *fp,
						 int *Segmentation,
						 int w, int h);
void RVLSegmentationMarkSelectedRegion(	CRVLClass *pTriangleSet, 
										int Label,
										DWORD mMask);
void RVLSWERUpdateQueue(RVLSWER_LINK *pLink,
								  RVLSWER_LINK *pLink0,
								  int maxCost,
								  double k,
								  void RVLSWERUpdateLink(	RVLSWER_NODE *pNode1,
															RVLSWER_NODE *pNode2,
															RVLSWER_LINK *pLink),
								  int RVLSWERGetCost(	BYTE *pData,
														int maxCost,
														double k),
								  RVLQLIST *QueueListArray,
								  int &Cost);
int RVLSegmentationWER(	void RVLSWEROnCreateNewNode(RVLSWER_NODE *pNode,
													RVLSWER_LINK *pLink),
						void RVLSWERUpdateLink(	RVLSWER_NODE *pNode1,
												RVLSWER_NODE *pNode2,
												RVLSWER_LINK *pLink),
						int RVLSWERGetCost(	BYTE *pData,
											int maxCost,
											double k),
						RVLSWER_NODE *pNewNode,
						BYTE *pNodeData,
						int NodeDataSize,
						RVLQLIST *QueueListArray,
						int maxnNodes,
						int minCost,
						int maxCost,
						double k);
/*DEPRECATED void RVLCalculateRGBHist(CRVLMPtrChain *pTriangleList,
						 IplImage* rgbImg,
						 int *pProjected,
						 int w, int h);*/
void RVLCalculateKinectProjections(int *depthMap,
						 int *pProjected,
						 int w, int h);
void RVLCalculateRGBHOG(CRVLMPtrChain *pTriangleList,
						 IplImage* rgbImg,
						 int *pProjected);
void RVLCalculateCentroidDescriptor(RVL3DPOINT2 **Point3DMap, 
								  CRVLClass *p2DRegionSet, 
								  float *eigs,
								  float *minD,
								  float *maxD);
void RVLCalculateD2ShapeDistribution(RVL3DPOINT2 **Point3DMap, 
								  CRVLClass *p2DRegionSet, 
								  RVLQLIST *distribution);
void RVLSetUsefulPixMask(CRVLClass *p2DRegionSet,
						RVL3DPOINT2 **Point3DMap,
						int w, int h,
						CRVLMem *pMem,
						int border = 3,
						BYTE *uPixMask = NULL,
						BYTE label = 1,
						DWORD Flags = 0,
						DWORD Mask = 0);
void RVLGetAvgColorOf2DRegions(	CRVLMPtrChain *p2DRegionList,
								RVLAPIX_2DREGION_PTR *Region2DMap, 
								unsigned char *PixArray,
								int ImageSize,
								BOOL bColor);
void RVLGetAvgNeighborhoodIntensity(CRVLMPtrChain *p2DRegionList,
									RVLAPIX_2DREGION_PTR *Region2DMap, 
									int Width, int Height);
void RVLDisplaySegmentationToConvex(CRVLMPtrChain *p2DRegionList,
									RVLAPIX_2DREGION_PTR *Region2DMap, 
									int ImageSize,
									BOOL bColor,
									PIX_ARRAY *pOutPixArray);
void RVLGetMomentsOf2DRegions(	CRVLMPtrChain *p2DRegionList,
								RVLAPIX_2DREGION_PTR *Region2DMap,
								int ImageWidth, int ImageHeight);
bool RVL3DMeshIsConvex(CRVLMPtrChain *pTriangleList,
					   RVL3DPOINT2 **Point3DMap);
void RVLSegmentationDisplayBoundary(CRVLFigure *pFig,
									CRVL2DRegion2 *pSegment,
									int ImageWidth,
									CRVLMem *pMem,
									CvScalar Color,
									int LineWidth = 1,
									int uOffset = 0);
void RVLGraphSegmentationFH(
	RVLSWER_NODE2 *Node,
	int nInNodes,
	RVLSWER_LINK2 **SortedLinkList,
	int nLinks,
	float k,
	int &nNodes,
	RVLARRAY_<RVLSWER_SEGMENT<RVLSWER_NODE2>> &SegmentArray,
	int *PtListMem);
void RVLRGBGraphSegmentation(
	unsigned char *RGB,
	int w,
	int h,
	int NeighborhoodSize,
	float k,
	RVLSWER_NODE2 *Node,
	RVLARRAY_<RVLSWER_SEGMENT<RVLSWER_NODE2>> &SegmentArray,
	int *PtMem);
void RVLDominantSegmentColor(
	CRVLMPtrChain *pTriangleList, 
	int nObjects,
	IplImage* pRGBImg,
	uchar *&color);
void RVLDominantSegmentColor2(
	CRVLMPtrChain *pTriangleList,
	int nObjects,
	IplImage* pRGBImg,
	uchar *&color);
void RVLCreateHSHistograms(
	CRVLMPtrChain *pTriangleList,
	int nObjects,
	IplImage* pRGBImg,
	int satThr,
	int *&HSVHist,
	int *&nSamples,
	int *&nPts);
void RVLDisplayRGBSegmentation(
	IplImage *pInImage,
	RVLARRAY_<RVLSWER_SEGMENT<RVLSWER_NODE2>> &SegmentArray,
	IplImage *pOutImage);
void RVLSegmentationSaveSelection(
	CRVLMPtrChain *pTriangleList,
	int nSegments,
	DWORD Flags,
	char *FileName);
void RVLSegmentationLoadSelection(
	CRVLMPtrChain *pTriangleList,
	int nSegments,
	DWORD Flags,
	char *FileName);
#ifdef RVLVTK
void RVLDisplaySegmentedMesh3D(CRVLVTKRenderer *pRenderer,
                                CRVLMPtrChain *pTriangleList,
                                int nObjects,
                                int w, 
                                int h,
								int nFOVExtensions,
                                int *pointmap,
                                RVL3DPOINT2 **Point3DMap,
                                int colortype = 0,
                                IplImage* pTexImg = NULL);
#endif

class CRVLSegmentation  
{
public:
	DWORD m_Flags;
	int m_Width, m_Height;
	int *m_EmptyRegionGrowingMap;
	int *dpNeighbor4;
	CRVLMem *m_pMem; 
	CRVLMem *m_pMem2;
	CRVLTimer *m_pTimer;
	RVLRECT *m_pROI;
	CRVLParameterList m_ParamList;
	CRVL2DRegion2 **m_p2DRegionMap;
	
public:
	virtual void Clear();
	virtual void Init();
	
	//GB Segment
	virtual void Segment(PIX_ARRAY *pPixArray,
						 CRVLAImage *pAImage,
						 CRVLC2D *p2DRegionSet);
	
	CRVLSegmentation();
	virtual ~CRVLSegmentation();

	virtual void CreateParamList(CRVLMem * pMem);
};

#endif // !defined(AFX_RVLSEGMENTATION_H__9938901F_2DAC_478B_BA31_E568663CF2AD__INCLUDED_)
