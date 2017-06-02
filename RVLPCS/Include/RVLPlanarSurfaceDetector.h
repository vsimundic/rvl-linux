// RVLPlanarSurfaceDetector.h: interface for the CRVLPlanarSurfaceDetector class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLPLANARSURFACEDETECTOR_H__1DCCB796_4F94_4476_9777_2D5D6B292431__INCLUDED_)
#define AFX_RVLPLANARSURFACEDETECTOR_H__1DCCB796_4F94_4476_9777_2D5D6B292431__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVLDelaunay.h"

#define RVLPSD_FLAG_INTERNAL						0x10000000

#define RVLPSD_FLAG_GROUND							0x00000001
#define RVLPSD_FLAG_SUBSAMPLE						0x00000002
#define RVLPSD_FLAG_CONNECTED_COMPONENTS			0x00000004
#define RVLPSDLAD_3DPOINT_FLAG_REJECTED				0x00000100
//#define RVLPSDLAD_GRBIC
#define RVLPSDRANSAC_FLAG_LS						0x00000100
#define RVLPSDRANSAC_FLAG_LAD						0x00000200
#define RVLPSDRANSAC_FLAG_LSINIT					0x00000400
#define RVLPSDRANSAC_FLAG_LADINIT					0x00000800
#define RVLPSDRANSAC_FLAG_3PTSINIT					0x00001000
#define RVLPSDRANSAC_FLAG_WEIGHTED_SCORE			0x00002000
#define RVLPSDRANSAC_FLAG_PT_DISTRIBUTION			0x00004000
#define RVLPSDRANSAC_FLAG_SKIP_FINAL_CONSENSUS_SET	0x00008000
#define RVLPSDRANSAC_FLAG_RESC						0x00010000
#define RVLPSDGCC_FLAG_DISPARITY_CONTINUITY			0x00020000		// must be compatible with RVLPSDRANSAC_FLAGs
//#define RVLPSDGCC_FLAG_PLANE						0x00000002
//#define RVLPSDGCC_FLAG_MASK						0x00000001
#define RVLPSDGCC_FLAG_DILATE						0x00040000		// must be compatible with RVLPSDRANSAC_FLAGs
#define RVLPSD_SEGMENT_3D							0x00100000		//Original segmentation (performed on 3D image)
#define RVLPSD_SEGMENT_REGIONBASED					0x00200000		//Region based segmentation (eg Graph based by Pedro F. Felzenszwalb or Edge Based by Robert Cupec)
//#define RVLPSD_SEGMENT_EDGEBASED					0x00400000		//Edge based segmentation
#define RVLPSD_SEGMENT_STRM							0x00400000		//Succesive triangulation refinement + merging
#define RVLPSD_FLAG_PC_BEAM_DISTANCE				0x00800000		//
#define RVLPSD_MESH_SEGMENT_PLANAR					0x01000000		//Segmentation into planar surface segments
#define RVLPSD_MESH_SEGMENT_WER						0x02000000		//Segmentation into planar surface segments using WER approach
#define RVLPSD_MESH_CONVEX							0x04000000
#define RVLPSD_FLAG_MM								0x08000000		// segmentation is performed in mm-space (instead of uvd-space)
#define RVLPSD_FLAG_100UM							0x10000000		// segmentation is performed in mm-space with 100um precision
#define RVLPSD_FLAG_RELIABLE_DISPARITY				0x20000000		//Determine Reliable disparities are used with Delaunay triangulation.
#define RVLPSD_FLAG_SUBSEGMENT						0x40000000		//Subsegmentation to be performed. Reliable disparities are used with Delaunay triangulation.
#define RVLPSD_FLAG_MAXIMUMREGIONGROWING			0x80000000		//Maximum region growing performed 
#define RVLPSD_FLAG2_VELODYNE_SENSOR_UNCERT_MODEL	0x00000001

#define RVLPSD_SAVE_MESH_FLAG_HULL					0x00000001


//#define RVLPSDCONSENSUS_FLAG_ASSIGN					0x01

#define RVLPSDRANSAC_FLAG_TIME_LIMIT				0x00000100		// limited execution time

#define RVLPSDRANSAC_NUMBER_OF_MAIN_SEGMENTS		2
#define RVLPSDRANSAC_MAX_NUMBER_OF_SEGMENTS			10

#define RVLPSDLSPLANE_FLAG_M_ESTIMATOR				0x00000001


#define RVLPSDDISPLAY_DISPARITY_OVER_IMAGE			0x00000001
#define RVLPSDDISPLAY_GET_SURFACE					0x00000002
#define RVLPSDDISPLAY_SEGMENTATION					0x00000004

#define RVLPSDDISPLAY_TWOSEGMENTS					0x00000008
#define RVLPSDDISPLAY_DISPARITY_TWOSEGMENTS			0x00000010

#define RVLMESHSEGMENT_SELECT_OBJECT_FLAG_OBJECT	0x00000001

#define RVLPSDGETTEXTURE_FLAG_ORTOGONAL_MAPPING		0x00000001
#define RVLPSDGETTEXTURE_FLAG_TEXTON_ELLIPSES		0x00000002

#define RVLPSD_MAXN_FOV_EXTENSIONS					5

//#define RVLPSD_PROJECT_DI_TO_PLANES_FLAG_PROJECT	
//#define RVLPSD_SEGMENT_STRM_LOG_FILE
//#define RVLPSD_SEGMENT_STRM_DEBUG
//#define RVLPSD_SAMPLE_SURF_DEBUG
//#define RVLPSD_SEGMENT_STRM_PC_DEBUG
#define RVLPSD_GET3DSURFACES_UVD

struct RVLPSDLAD_THREE_POINTS_ITER_AB
{
	double a;
	double b;
	double c;
	double G;
	RVL3DPOINT2 tocka;
};

struct RVLPSDLAD_KIP_TWO_POINTS
{
	double ak;
	double x;
	double y;
	int indeks;
};

struct RVLPSDLAD_SEGMENT_DETAILS
{
	double a;
	double b;
	double c;
	int segmentNumber;
	int numberOfPoints;
	int numberOfIterations;
};

struct RVLPSD_SEGMENT_DETAILS
{
	double a;
	double b;
	double c;
	int numberOf3DPoints;
	CRVLMPtrChain m_ObjectList;
};

struct RVLPSD_STRM_QUEUE_ENTRY
{
	CRVL2DRegion2 *pRegion;
	int iPix;
	void *pNext;
};

struct RVLPSD_DEBUG_DATA
{
	CRVLGUI *pGUI;
};

struct RVLPSD_DT_QUEUE_ENTRY
{
	DWORD dist;
	int iPix;
	void *pNext;
};

struct RVLPSD_2DREGION_SAMPLE
{
	RVLAPIX *pPt;
	int w;
	void *pNext;
};

struct RVLPSD_CHVERTEX
{
	int u;
	int v;
	int du;
	int dv;
};

void RVLPSDDisplay(RVLDISPARITYMAP *pDisparityMap, 
				   CRVL2DRegion2 *pPlane,
				   double Tol,
				   PIX_ARRAY *pInPixArray,
				   PIX_ARRAY *pDisplayPixArray);

void RVLPSDDisplayImageSegments(RVLDISPARITYMAP *pDisparityMap,
								 RVL3DPOINT2 **ppPoint3DMap,
								 PIX_ARRAY *pInPixArray,
								 PIX_ARRAY *pDisplayPixArray);

void RVLPSDDisplayDisparitySegments(RVLDISPARITYMAP *pDisparityMap,
									 RVL3DPOINT2 **ppPoint3DMap,
									 PIX_ARRAY *pDisplayPixArray);
void RVLGet3DPlane( CRVL2DRegion2 *p2DRegion, 
					CRVLStereoVision *pStereoVision,
					CRVL3DSurface2 *p3DSurface);


void RVLPSDDisplaySegments(RVLDISPARITYMAP *pDisparityMap,
						   PIX_ARRAY *pIIn,
					  	   PIX_ARRAY *pIOut,
						   CRVL2DRegion2 *pGround,
						   CRVL2DRegion2 **ppWalls,
						   DWORD Flags,
						   bool bMarkSegments = true);

void RVLPSDDisplaySegments2(CRVLAImage *pAImage,
							RVLDISPARITYMAP *pDisparityMap,
					  PIX_ARRAY *pIIn,
					  PIX_ARRAY *pIOut,
					  CRVL2DRegion2 *pGround,
					  CRVL2DRegion2 **ppWalls,
					  DWORD Flags,
					  bool bEdgeBasedSegments = false,
					  bool bMarkSegments = true);

void RVLPSDDisplayFloorSegments(RVLDISPARITYMAP *pDisparityMap,
						   PIX_ARRAY *pIIn,
					  	   PIX_ARRAY *pIOut,
						   CRVL2DRegion2 *pGround,
						   CRVLMPtrChain *p2DRegionList);

void RVLPSDDrawLine(int u0, int v0, int u1, int v1,PIX_ARRAY *pIPic, bool bIsRef, int u0Ref, int u1Ref);



void RVLUpdate2DPolygonAreaAndCentroid(double xi, 
									   double yi,
									   double xip, 
									   double yip,
									   double &Area,
									   double &Xcentroid,
									   double &Ycentroid);

void RVLUpdate2DPolygonMoments(double xi, 
							   double yi,
							   double xip, 
							   double yip,
							   double &Xmoment,
							   double &Ymoment,
							   double &XYmoment);

int RVLMeshSegmentWERGetCost(	BYTE *pData,
								int maxCost,
								double k);

int RVLMeshSegmentWERGetCostLog(	BYTE *pData,
									int maxCost,
									double k);

void RVLMeshSegmentSWERUpdateLink(	RVLSWER_NODE *pNode1,
									RVLSWER_NODE *pNode2,
									RVLSWER_LINK *pLink);

void RVLMeshSegmentSWEROnCreateNewNode(	RVLSWER_NODE *pNode,
										RVLSWER_LINK *pLink);

void RVLPSDDisplayLEVEL2Regions(CRVLFigure *pFig,
								CRVLC2D *p2DRegionSet,
								int ImageWidth,
								RVLCOLOR Color);
//OBSOLETE!!! Will be deleted in the future
void RVLPSDDisplayLEVEL2Desc(CRVLFigure *pFig,
							   CRVLC2D *p2DRegionSet,
							   int ImageWidth,
							   RVLCOLOR Color,
							   RVLKINECT_PARAMS kinectParams);
void RVLDisplayDistanceTransformMap(int *DTMap,
									RVLAPIX **SampleMap,
									int ImageSize,
									int minDist,
									unsigned char *PixArray);
void RVLResetFlags(CRVLMPtrChain *pObjectList, BYTE Flags);


class CRVLPlanarSurfaceDetector  
{
public:
	DWORD m_Flags;
	DWORD m_Flags2;
	double m_ScoreScale;
	double m_kLADRANSACInit3DPoints;
	int m_minnLADRANSACInit3DPoints;
	double m_alpha;
	double m_kminnCBin;
	double m_rho;
	double m_BinSize;
	int m_nBins;
	int m_WinSize;
	int m_nRANSACIterations;
	double m_Tol;
	int m_MeshTol;
	double m_ResidualThr;
	double m_RelTol;
	double m_kminnInitPts;
	RVL3DPOINT2 *m_Point3DArray;
	//RVL3DPOINT2 **m_Point3DOriginalArray;
	//RVL3DPOINT2 **m_Point3DBuff1;
	//RVL3DPOINT2 **m_Point3DBuff2;
	RVL3DPOINT2 **m_Point3DMap;
	RVL3DPOINT2 **m_Point3DMapMem;
	RVL3DPOINT2 **m_Point3DBuff;
	int m_n3DPoints;
	//int m_n3DOriginalPoints;
	int m_Width, m_Height;
	short int m_minDisparity;
	short int m_maxDisparity;
	bool m_enableMinDisparity;
	CRVLTimer *m_pTimer;
	bool m_enableTimeLimit;
	double m_maxt;
	CRVLStereoVision *m_pStereoVision;
	double m_maxSegmentTime;
	double m_GroundBeta;
	double m_GroundBetaTol;
	double m_GroundThetaTol;
	int m_ITol;
	int m_nSupport;
	//WORD *m_Segmentation;
	int m_dpNeighbor4[8];
	int m_NeighborLimit[4];
	int m_diPixContourFollow[4][4];
	//unsigned short m_maxdist;
	double m_MEstThr;
	int m_SubSample;
	int m_NoiseThr;
	int m_minSegmentSize;
	int m_minnInliers;
	int m_DiscontinuitiyThr;
	double m_MeshDiscontinuityThr;
	int m_minnValids;
	double m_AngleThr;
	//int m_maxdepth;
	int m_nCCDilationIterations;
	CRVL2DRegion2 **m_WallArray;
	int m_nWalls;
	CRVL2DRegion2 *m_pGround;
	CRVL3DPose m_PoseLG;
	int *m_ResidualHistogram;
	double m_minAngleBtwnLines;
	double m_minAngleBtwnProjectedLines;
	double m_minAngleBtwnPlanes;
	int *m_RegionGrowingMap;
	int *m_EmptyRegionGrowingMap;
	int *m_RegionGrowingBuff;
	CRVLDelaunay *m_pDelaunay;
	int m_RowLength;  //half the row length used in searching for reliable disparity points
	bool m_bEdgeBasedSegments;   //indicates that the segments were obtained using edge based segmentation
	int m_uNrm;
	int m_uvdTol;
	int m_uvTol;
	double m_RuvdTol;
	double m_RuvTol;
	int m_fillPerc;
	int m_MeshPlanarSegWERThr1;
	int m_MeshPlanarSegWERThr2;
	//int m_maxDis;
	CRVLParameterList m_ParamList;
	RVL3DPOINT2 m_CornerPtArray[4];
	RVLSWER_NODE *m_MeshSegmentWERNodeArray;
	int m_nMeshSegmentWERNodes;
	int m_nMeshSegmentWERLevels;
	int m_MeshSegmentWERMaxCost;
	double m_MeshSegmentWERk;
	CRVL2DRegion2 **m_2DRegionMap;
	CRVL3DSurface2 **m_3DSurfaceMap;
	RVLPSD_DEBUG_DATA m_DebugData;
	int m_MinConvexSegmentSize;
	int m_minSampleSize, m_maxSampleSize;
	RVLAPIX **m_APixBuff;
	CRVLAImage *m_pAImage;
	int *m_DTMap;
	int m_nFOVExtensions;
	double m_FOVExtension;
	double m_PointMeasurementUncertStD;
	RVLLIDAR_PARAMS *m_pLidarParams;
		
#ifndef RVLPSDLAD_GRBIC
	CRVLMem *m_pMem, *m_pMem2;
#endif
private:
	double m_expLT[10001];
	
	RVL3DPOINT2 **m_Point3DPtrArray;
	//int *m_Histogram;
	//int *m_CHistogram;
	//double *m_Se;
	CRVLHistogram m_Histogram[6];
	RVL3DPOINT2 **m_CCBuff;
	int m_iRG1, m_iRG2;
	int *m_iScanLineStart;
	int *m_iScanLineEnd;
	CRVLQListArray m_Queue;
	RVLPSD_STRM_QUEUE_ENTRY *m_QueueMem;
	BYTE *m_bCorner;
	CRVLQListArray m_DelaunayLinkQueue;
	RVLPSD_DT_QUEUE_ENTRY *m_DTQueueEntryMem;	


	//*********used in Get3DSurfaceAndContours***********
	double rotMat[9];
	CvMat *dMatRot;

	double PL[3];
	CvMat *dMatPL;

	double PL1[3];
	CvMat *dMatPL1;

	double sqrMat[4];
	CvMat *dMatSqr;

	double eigMat[4];
	CvMat *dMatEig;

	double eigVal[2];
	CvMat *dMatEigVal;

	double rotMatFinal[9];
	CvMat *dMatRotFinal;
	
	double rotMatF[9];
	CvMat *dMatRotF;
	//*******************************************************

public:
	void GetTexture(CRVL3DSurface2 * p3DSurface,
					DWORD Flags = 0x00000000);
	void Get3DTextons(CRVLMPtrChain *p3DSurfaceList);
	int Sample2DRegions();
	RVL3DPOINT2 *FindNearest3DPoint(int u, int v, CRVLGUI *pGUI,CRVLC2D *p2DRegionSet);

	void DisplayInvisibleBoundaries(	CRVLGUI *pGUI,
										CRVLFigure *pFig);
	void DisplayProjectedIntensityGradient(	CRVLMPtrChain *p2DRegionList,
											PIX_ARRAY *pIIn,
											PIX_ARRAY *pIOut);
	void GetMinAngles(const char *UVPointsFileName);
	void Get3DPlanes(CRVLMPtrChain *p2DRegionList, CRVLClass *p3DSurfaceSet);
	void ProjectIntensityGradientToPlanarSurfaces(	CRVLAImage *pAImage,
													CRVLMPtrChain *p2DRegionList);
	void GetRefSurfaces(CRVLMPtrChain *p2DRegionList, 
						double &y0, double &z0,
						double **pHArray = NULL);
	void Dilate(RVL3DPOINT2 **PtSet, 
				int n);
	void Mask(	RVL3DPOINT2 **PtSet, 
				int n,
				int mask);
	void ClearRegionGrowingMap();
	void GetConnectedComponents(RVL3DPOINT2 **Point3DSet,
								int n,
								CRVLMChain *pCCList,
								DWORD Flags
								//unsigned short Mask = 0
								//CRVL2DRegion2 *pPlane = NULL
								);
	void Segment(	CRVLC2D *p2DRegionLevel1,
					CRVLC2D *p2DRegionLevel2 = NULL,
					CRVLC2D *p2DRegionLevel3 = NULL,
					CRVLMem *pMem = NULL,
					DWORD FlagsExt = RVLPSD_SEGMENT_3D	  
									| RVLPSD_FLAG_SUBSAMPLE  
									| RVLPSD_FLAG_CONNECTED_COMPONENTS
									//| RVLPSDRANSAC_FLAG_LADINIT
									//| RVLPSDRANSAC_FLAG_LSINIT
									| RVLPSDRANSAC_FLAG_LS
									//| RVLPSDRANSAC_FLAG_LAD
									| RVLPSDRANSAC_FLAG_PT_DISTRIBUTION
									| RVLPSDRANSAC_FLAG_WEIGHTED_SCORE
									//| RVLPSDGCC_FLAG_DISPARITY_CONTINUITY
									| RVLPSDGCC_FLAG_DILATE
									| RVLPSDRANSAC_FLAG_RESC,
					CRVLC2DContour *p2DContourSet = NULL
									);

	void Init();
	void Display2(	CRVLAImage *pAImage,
					PIX_ARRAY *pIOut,
					DWORD Flags);

	

	BOOL Consistent(CRVL2DRegion2 *pPlane, 
					RVLSTEREOPOINT *pPt);
	int GetSurface( int u, int v,
					int *PtBuff,
					DWORD Flags = 0x00000000);
	void GetGroundPoint(int *iU, CRVL3DSurface2 *pGroundPlane, double *X);
	void Get3DPlane(CRVL2DRegion2 *pRegion, CRVL3DSurface2 *p3DSurface);
	void GetPointsWithDisparity(RVLDISPARITYMAP *pDisparityMap,
							   double *X, double *Y, double *Z,
							   int &nPoints);
	int GetPointsWithDisparity(RVLRECT *pROI,
							   RVL3DPOINT2 **Point3DPtrArray);
	void GetPointsWithDisparity(RVLDISPARITYMAP *pDisparityMap);
	void GetReliablePointsWithDisparity(RVLDISPARITYMAP *pDisparityMap,
										CRVLAImage *pAImage,
										CRVLMem *pMem);
	void GetPointsWithDisparity(CRVL2DRegion2 *pPolygon,RVL3DPOINT2 **ppPtrArray);

	void GetPointsWithDisparity(RVLDISPARITYMAP *pDisparityMap,
								CRVLAImage *pAImage,
								CRVLMem *pMem,
								bool bDelaunay);  // to replace GetPointsWithDisparity(RVLDISPARITYMAP *pDisparityMap) & GetReliablePointsWithDisparity

	

	void CreateDisparityMap(double *X, double *Y, double *Z,
							int n,
							RVLDISPARITYMAP *pDisparityMap);

	
	//SOON TO BE OBSOLETE!!
	void Get3DSurfaceAndContours(CRVL2DRegion2 *p2DRegionLevel3,
								 CRVL3DSurface2 *p3DSurfaceLevel3,
								 CRVLClass *p3DConvexSegmentSet,
								 CRVL3DSurface2 **pp3DConvexSegmentArray,
								 CRVLC2DContour *p2DContourSet,
								 CRVL2DContour **pp2DContourArray,
								 RVLIPOINT *pIPointArray,
								 RVL3DPOINT2 **ppVertexPtArray,
								 int &nConvexSegments,
								 int &nTotalVertexPts,
								 double *TransformedVertexArray,
								 int &nSupportPts);

	void Get3DSurfaceAndContours2(CRVL2DRegion2 *p2DRegionLevel3,
								 CRVL3DSurface2 *p3DSurfaceLevel3,
								 CRVLClass *p3DConvexSegmentSet,
								 CRVL3DSurface2 **pp3DConvexSegmentArray,
								 CRVLC2DContour *p2DContourSet,
								 CRVL2DContour **pp2DContourArray,
								 RVLIPOINT *pIPointArray,
								 RVL3DPOINT2 **ppVertexPtArray,
								 int &nConvexSegments,
								 int &nTotalVertexPts,
								 double *TransformedVertexArray,
								 int &nSupportPts);

	
	void Get3DPlaneKinect(CRVL2DRegion2 *p2DRegion, CRVL3DSurface2 *p3DSurface);
	void Get2DPlaneKinect(	double *N,
							double d,
							double &a,
							double &b,
							double &c);
	
	//double RESCInlierSTD();
#ifdef RVLPSDLAD_GRBIC
	double Consensus(RVL3DPOINT2 *Point3DArray,
				  int n,
				  double a, double b, double c,
				  RVL3DPOINT2 **ConsensusSet,
				  int &nConsensus);
	int GetNumberOfInliers(RVL3DPOINT2 *Point3DArray,
							int n,
							double a, double b, double c);
	BOOL Plane(RVL3DPOINT2 *pPoint3D0, 
			   RVL3DPOINT2 *pPoint3D1, 
			   RVL3DPOINT2 *pPoint3D2,
			   double &a, double &b, double &c);
	int RANSAC(double &a, double &b, double &c,
				bool bSegment,
				RVL3DPOINT2 **pPoint3DArray,
				int n3DPoints,
				RVLPSDLAD_SEGMENT_DETAILS *segmentDetails,
				DWORD Flags = 0x00000000);
	BOOL LADPlaneThreePoints(RVL3DPOINT2 **Point3DPtrArray,
							 int n,
							 double &a, double &b, double &c);
	void LSPlane(RVL3DPOINT2 **Point3DPtrArray,
				 int n,
				 double &a, double &b, double &c);
	BOOL ThreePtsPlane(RVL3DPOINT2 **Point3DPtrArray,
					   int n,
					   double &a, double &b, double &c);


#else
	double Consensus(RVL3DPOINT2 **Point3DArray,
				  int n,
				  CRVL2DRegion2 *pPlane,
				  RVL3DPOINT2 **ConsensusSet,
				  int &nConsensus,
				  DWORD Flags = 0x00000000,
				  double BestScore = -1.0);
	int GetNumberOfInliers(RVL3DPOINT2 *Point3DArray,
							int n,
							CRVL2DRegion2 *pPlane);
	BOOL Plane(RVL3DPOINT2 *pPoint3D0, 
			   RVL3DPOINT2 *pPoint3D1, 
			   RVL3DPOINT2 *pPoint3D2,
			   CRVL2DRegion2 *pPlane);
	int RANSAC(	CRVL2DRegion2 *pPlane,
				bool bSegment,
				RVL3DPOINT2 **pPoint3DArray,
				int n3DPoints,
				RVLPSDLAD_SEGMENT_DETAILS *segmentDetails,
				DWORD Flags = 0x00000000);		
	BOOL LADPlaneThreePoints(RVL3DPOINT2 **Point3DArray,
							 int n,
							 CRVL2DRegion2 *pPlane);
	void LSPlane(RVL3DPOINT2 **Point3DArray,
				 int n,
				 CRVL2DRegion2 *pPlane,
				 DWORD Flags = 0x00000000);
	BOOL ThreePtsPlane(RVL3DPOINT2 **Point3DPtrArray,
					   int n,
					   CRVL2DRegion2 *pPlane);
	void Apply(CRVLStereoVision *pStereoVision);

	void InitStatistics(CRVLMem *pHistMem);
	void CreateStatistics();
	void RANSACStatistics(CRVL2DRegion2 *pPlane,
						 CRVLMem *pHistMem);	
	void CreateParamList(CRVLMem *pMem);
	
#endif
	RVL3DPOINT2 *WeightedMedian(
		//double *DataSort, 
		//double *DataWeights, 
		//int *Indeksi, 
		RVL3DPOINT2 **Point3DPtrArray, 
		int left, int right, 
		int pivot);
	void SaveMesh(	FILE *fpPts,
					FILE *fpIdx,
					CRVLClass *p2DRegionSet,
					DWORD Flags = 0x00000000,
					int min3DPtsPerc = 0,
					DWORD Mask = 0x00000000);
	void SaveMesh2Ply(	FILE *fpPly,
					CRVLClass *p2DRegionSet,
					DWORD Flags = 0x00000000,
					int min3DPtsPerc = 0,
					DWORD Mask = 0x00000000);
	void SaveMesh2Ply2(	FILE *fpPly,
					CRVLClass *p2DRegionSet,
					DWORD Flags = 0x00000000,
					int min3DPtsPerc = 0,
					DWORD Mask = 0x00000000);
	void SaveMesh2OFF(	FILE *fpOFF,
					CRVLClass *p2DRegionSet,
					DWORD Flags = 0x00000000,
					int min3DPtsPerc = 0,
					DWORD Mask = 0x00000000);
	void MeshSegmentWER(	CRVLC2D *p2DRegionSet,
							void RVLSWEROnCreateNewNode(RVLSWER_NODE *pNode,
														RVLSWER_LINK *pLink),
							void RVLSWERUpdateLink(	RVLSWER_NODE *pNode1,
													RVLSWER_NODE *pNode2,
													RVLSWER_LINK *pLink),
							int RVLSWERGetCost(	BYTE *pData,
												int maxCost,
												double k)
							);
	void MeshSegmentWERSetLabels(CRVLMPtrChain *pTriangleList,
									int Level);
	BOOL MeshSegmentWERNodeWriteToFile(	FILE *fp,
										RVLSWER_NODE *pNode,
										int maxnLinks);
	void MeshSegmentWERLabelObject(CRVLMPtrChain *pTriangleList,
									RVLSWER_NODE *pSelectedNode,
									int Label = 0);
	RVLSWER_NODE *MeshSegmentWERSelectObject(CRVL2DRegion2 *pTriangle,
														int &Level,
														DWORD Flags = 0x00000000);
	void Gen3DMeshObjectHierarchy(CRVL3DMeshObject *pRootMO);
	int GenRelListFromWER(CRVLC2D *pTriangleSetLevel1,CRVLC2D *pTriangleSetLevel3);
	void GetNeighbors(CRVLC2D *pSegmentSet);
	void Display2DRegionMap(PIX_ARRAY *pOutPixArray);
	CRVLPlanarSurfaceDetector();
	virtual ~CRVLPlanarSurfaceDetector();
	void GetRegionBoundaries(void);
	void GetOrgPC(double * PC, int n);
	void GetOrgPCProjectionParams(double &fu, double &fv, double &uc, double &vc);
	void DisplayPC(IplImage *pDisplay);
	void AssignLabels(CRVLC2D *pTriangleSetLevel1, CRVLC2D *pTriangleSetLevel3);
	bool ProjectToFOVExtension(	double *X,
								double *P,
								double *R,
								int &iFOVExtension_);

private:
/*	RVLPSDLAD_THREE_POINTS_ITER_AB ThreePointsModified_searchb(
		RVL3DPOINT2 **Point3DPtrArray, 
		int ArraySize,
		RVL3DPOINT2 &tocka_p);*/
	RVLPSDLAD_THREE_POINTS_ITER_AB ThreePointsAbSearch_afor(
		RVL3DPOINT2 **Point3DPtrArray, 
		int ArraySize,
		RVL3DPOINT2 &tocka_mi,
		RVL3DPOINT2 &tocka_p);

	RVLPSDLAD_THREE_POINTS_ITER_AB ThreePointsAbSearch_bfor(
		RVL3DPOINT2 **Point3DPtrArray, 
		int ArraySize,
		RVL3DPOINT2 &tocka_mi,
		RVL3DPOINT2 &tocka_p);

	BOOL ThreePointsModified_searchb(
		RVL3DPOINT2 **Point3DPtrArray, 
		int ArraySize,
		RVL3DPOINT2 &tocka_p,
		RVLPSDLAD_THREE_POINTS_ITER_AB &search_b);
	
	void InitilizePointWithDisparity(RVL3DPOINT2 *pPoint3D, 
									 int u, int v, int d, int iPix,
									 RVL3DPOINT2 **ppP3DMap,
									 CRVLMPtrChain *pBorderIn,
									 RVLEDT_PIX_ARRAY *peDTImage,
									 bool bDelaunay);

	void Segment3D(CRVLC2D *p2DRegionSet, DWORD Flags);
	void SegmentRB(CRVLC2D *p2DRegionSet, DWORD Flags);
	void SegmentRB2(CRVLC2D *p2DRegionSet, CRVLC2D *p2DRegionSet2, DWORD Flags);
	void SegmentRB3(CRVLC2D *p2DRegionSet, CRVLC2D *p2DRegionSet2, DWORD Flags);
	void SegmentRB4(CRVLC2D *p2DRegionSet1, CRVLC2D *p2DRegionSet2, CRVLC2D *p2DRegionSet3, CRVLC2DContour *p2DContourSet, DWORD Flags); //Combination of RB2 and RB3
	void SegmentSTRM(	CRVLC2D * p2DRegionSet, 
						CRVLC2D *p2DRegionSet2, 
						CRVLC2D *p2DRegionSet3, 
						CRVLMem *pMem, 
						DWORD Flags);

	void SubSegment(CRVLC2D *p2DRegionSet, 
					CRVL2DRegion2 **ppSegmentBuffer,
					RVL3DPOINT2 **ppPtrArrayStart,
					CRVLMPtrChain **SortedSegmentBuffer,
					int &nMax);

	void SegmentGrowing(CRVL2DRegion2 **GroupedRegionList,
						CRVL2DRegion2 **pp2DSegment,
						CRVL2DRegion2 *p2DSegmentLevel3,
						RVL3DPOINT2 **ppPoint3DArray,
						int &nList,
						DWORD Flags);

	bool GetSegment3DPoints(RVL3DPOINT2 **ppPoint3D, int iPix, DWORD Flags);

	void UpdateSTRMQueue(	CRVL2DRegion2 **TriangleArray, 
							CRVL2DRegion2 **pTriangleArrayEnd,
							int &err,
							BOOL &bErrCorrection,
							RVLPSD_STRM_QUEUE_ENTRY **ppNewEntry);

	void SegmentDisplayHelper(int index, CRVL2DRegion2 *pRegion, PIX_ARRAY *pIOut, double d0, double dispScale, int iScale );

	
	//void Copy3DPointsToSegment(CRVL2DRegion2 *p2DSegment,
	//						   RVL3DPOINT2 **ppPtrArrayStart);
	
	void Segment3DDisplay(	CRVLGUI *pGUI,
							CRVLFigure *pFig,
							int iFirstTextLine,
							CRVLMPtrChain *p2DRegionList,
							PIX_ARRAY *pIIn,
							RVLAPIX *pSelectedAPix,
							PIX_ARRAY *pIOut,
							DWORD Flags);

	void SegmentRBDisplay(	CRVLGUI *pGUI,
							CRVLFigure *pFig,
							int iFirstTextLine,
							CRVLAImage *pAImage,
							PIX_ARRAY *pIIn,
							RVLAPIX *pSelectedAPix,
							PIX_ARRAY *pIOut,
							DWORD Flags);

	void SegmentRBDisplay2(	CRVLGUI *pGUI,
							CRVLFigure *pFig,
							int iFirstTextLine,
							CRVLAImage *pAImage,
							PIX_ARRAY *pIIn,
							RVLAPIX *pSelectedAPix,
							PIX_ARRAY *pIOut,
							DWORD Flags);
	void SegmentRBDisplay3(	CRVLGUI *pGUI,
							CRVLFigure *pFig,
							int iFirstTextLine,
							CRVLAImage *pAImage,
							RVLAPIX *pSelectedAPix,
							PIX_ARRAY *pIOut,
							DWORD Flags);
	void SegmentRBDisplayContour(CRVLGUI *pGUI,
								 CRVLFigure *pFig,
								 CRVLAImage *pAImage,
								 PIX_ARRAY *pIIn,
								 PIX_ARRAY *pIOut);
	void GetMaxDeviation(	CRVL2DRegion2 *pPolygon,
							int &eMax,
							int &iPixeMax,
							bool bFloat = false);
	void GetMaxDeviation(	CRVLC2D * p2DRegionSet,
							int &eMax,
							int &iPixeMax);
	
	int SegmentSTRMLevel3(CRVLC2D *pTriangleSetLevel1,
						  CRVLC2D *pTriangleSetLevel3,
						  CRVLMem *pMem);

	int SegmentSTRMGrowing( CRVL2DRegion2 **ppGroupedTriangleList,
							CRVL2DRegion2 *pTriangleSrc,
							CRVL2DRegion2 *p2DSegmentLevel3,
							RVLQLIST_PTR_ENTRY *LinkQueueEntryMem,
							CRVLMem *pMem,
							DWORD Label,
							int &nTriangles,
							CRVLC2D * p2DTriangleSet);

	int SegmentSTRMLevel2(CRVLC2D *pTriangleSetLevel1, 
						  CRVLC2D *pTriangleSetLevel2, 
						  CRVLC2D *pTriangleSetLevel3,
						  CRVLMem *pMem);

	
	
						
						

	inline void swap(double *DataSort, double *DataWeights, int *Indeksi, int index1, int index2)
	{
		double tmp1 = DataSort[index1];
		double tmp2 = DataWeights[index1];
		int tmp3 = Indeksi[index1];
			
		DataSort[index1] = DataSort[index2];
		DataWeights[index1] = DataWeights[index2];
		Indeksi[index1] = Indeksi[index2];

		DataSort[index2]	= tmp1;
		DataWeights[index2]	= tmp2;
		Indeksi[index2]		= tmp3;
	};
public:
	void Get3DPlanarSurfaceBoundary(
		CRVL3DSurface2 * pSurf,
		bool bPC = false);
};

#endif // !defined(AFX_RVLPLANARSURFACEDETECTOR_H__1DCCB796_4F94_4476_9777_2D5D6B292431__INCLUDED_)
