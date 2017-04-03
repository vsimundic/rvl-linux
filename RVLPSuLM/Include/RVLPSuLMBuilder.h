#pragma once
//#ifdef PYTHON_DEBUG
//#include "python.h"
//#include "numpy/arrayobject.h"
//#endif

#include "RVLPSuLM.h"
#include "RVLPSuLMIndexing.h"
#include "RVLFreiburgDataSet.h"

#define RVLLOG2					0.693147180559945		// todo: move to RVLConst.h

#define PROSAC_BETA						0.05	//the probability that a match is declared inlier by mistake, i.e. the ratio of the "inlier" surface by the total surface.  see Sec. 2.2 of [Chum-Matas-05].
												//ie The probability that an incorrect model calculated form a random sample containing an outlier is supported by a correspondence not included in the sample
#define PROSAC_ETA0						0.05	//the maximum probability that a solution with more than In_star inliers in Un_star exists and was not found after k samples (typically set to 5%, see Sec. 2.2 of [Chum-Matas-05]).
#define PROSAC_PHI						0.05
//
//#define	PROSAC_MAX_OUTLIERS_PROPORTION	0.8 // maximum allowed outliers proportion in the input data: used to compute T_N (can be as high as 0.95)
//#define PROSAC_P_GOOD_SAMPLE			0.99 // probability that at least one of the random samples picked up by RANSAC is free of outliers ie = 1-PROSAC_ETA0

#define PROSAC_CONFIDENCE				0.95	//n0 = 1-PROSAC_ETA0
#define PROSAC_MAXNO_INLIERS				1E10

#define RVLHIDEDETAILS

// Debugging

//#define PYTHON_DEBUG
//#define RVLPSULMBUILDER_HYPOTHESES_DEBUG
//#define RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
//#define RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
//#define RVLPSULM_CREATE_DEBUG_LOG
//#define RVLPSULMBUILDER_MAP_DEBUG_LOG
//#define RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
//#define RVLPSULMBUILDER_PARTICLE_FILTER_DEBUG

#define RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
//#define RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_ACCU_DEBUG_LOG
//#define RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_ACCU_DEBUG_LOG
//#define RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
//#define RVLPSULMBUILDER_CONDITIONAL_PROBABILITY_TREE_DEBUG_LOG
//#define RVLPSULMBUILDER_POSE_CONSTRAINT_PROBABILITY_DEBUG_LOG
//#define RVLPSULMBUILDER_AUTO_MATCH_MATRIX_DEBUG_LOG
//#define RVLPSULMBUILDER_GT_141111
//#define RVLPSULMBUILDER_SCENE_FUSION_DEBUG_LOG
//#define RVLPSULMBUILDER_HYPOTHESIS_LOG

// Configuration

//#define RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
#define RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_BY_EVIDENCE_ACCU
//#define RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_PROBABILISTIC
#define RVLPSULMBUILDER_GET_LOCAL_MODELS
//#define RVLPSULMBUILDER_PROB_HYPOTHESES_EVAL
#define RVLPSULMBUILDER_DISPLAY_MATCH_OVERLAP
#define RVLPSULMBUILDER_PARTICLE_FILTER_VER2
//#define RVLPSULMBUILDER_HYPOTHESES_EVAL3_MATCH_COUNT
#define RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT
//#define RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
#define RVLPSULMBUILDER_HYPOTHESES_EVAL3_ORIENT_MATCH
//#define RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
#define RVLPSULMBUILDER_MAPBUILDING_6DOF

#define RVLPSULMBUILDER_FLAG_SURFACES							0x00000001
#define RVLPSULMBUILDER_FLAG_LINES								0x00000002
#define RVLPSULMBUILDER_FLAG_VP									0x00000004
#define RVLPSULMBUILDER_FLAG_CORNER_LANDMARKS					0x00000008
#define RVLPSULMBUILDER_FLAG_MODE								0x00000030
#define RVLPSULMBUILDER_FLAG_MODE_TRACKING						0x00000010
#define RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION					0x00000020
#define RVLPSULMBUILDER_FLAG_GLOBAL								0x00000040
#define RVLPSULMBUILDER_FLAG_LOCALIZATION_POSE_REFINEMENT		0x00000080
#define RVLPSULMBUILDER_FLAG_LOCALIZATION_DISPLAY				0x00000100
#define RVLPSULMBUILDER_FLAG_LOCALIZATION_BIDIRECT_HYP_EVAL		0x00000200
#define RVLPSULMBUILDER_FLAG_LOCALIZATION_ODOMETRY				0x00000400
#define RVLPSULMBUILDER_FLAG_MATERIAL							0x00000800
#define RVLPSULMBUILDER_FLAG_KIDNAPPED							0x00001000
#define RVLPSULMBUILDER_FLAG_NEW_SUBMAP							0x00002000
#define RVLPSULMBUILDER_FLAG_MAPBUILDING						0x00004000
#define RVLPSULMBUILDER_FLAG_GROUNDTRUTH_EXISTS					0x00008000
#define RVLPSULMBUILDER_FLAG_EVALUATION_BENCHMARK				0x00010000	  //If active generates Evaluation benchmark files
#define RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING				0x00020000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD		0x000c0000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SM	0x00000000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM	0x00040000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM	0x00080000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P		0x000c0000
#define RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF				0x00100000
#define RVLPSULMBUILDER_HYPOTHESES_EKF_CONSTRAINT				0x00200000
#define RVLPSULMBUILDER_FLAG_GENERATE_MODELS					0x00400000
#define RVLPSULMBUILDER_FLAG_PARTICLE_FILTER					0x00800000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_SSM_NORM		0x01000000
#define RVLPSULMBUILDER_FLAG_PC									0x02000000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_PREEVAL		0x04000000
#define RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD					0x18000000
#define RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_MAX_PEAK_ONLY	0x00000000
#define RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_BEST_PEAK_TREE	0x08000000
#define RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_ALL_PEAKS		0x10000000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_MODEL_FUSION	0x20000000
#define RVLPSULMBUILDER_FLAG_HYPOTHESIS_GENERATION_INDEXING		0x40000000
#define RVLPSULMBUILDER_FLAG2_SCENE_FUSION						0x00000003
#define RVLPSULMBUILDER_FLAG2_SCENE_FUSION_LOOK_AROUND			0x00000001
#define RVLPSULMBUILDER_FLAG2_SCENE_FUSION_MOVE					0x00000002
#define RVLPSULMBUILDER_FLAG2_COMPLEX							0x00000004
#define RVLPSULMBUILDER_FLAG2_FILE_VERSION_2					0x00000008
#define RVLPSULMBUILDER_FLAG2_MAPBUILDING_MANUAL				0x00000010
#define RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING		0x00000020
#define RVLPSULMBUILDER_FLAG2_HYPGEN_INIT_MATCHING_CONSTRAINTS	0x00000040
#define RVLPSULMBUILDER_FLAG2_LINES_EDGES_VOID					0x00000080	
#define RVLPSULMBUILDER_FLAG2_SURFACE_BOUNDARY					0x00000100
#define RVLPSULMBUILDER_FLAG2_FIRST_ORDER_DEPENDENCY_TREE		0x00000200
#define RVLPSULMBUILDER_FLAG2_WIDE_ANGLE_HYPOTHESIS_GENERATION	0x00000400
#define RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT						0x00000800
#define RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG				0x00000800
#define RVLPSULMBUILDER_FLAG2_UNCONSTRAINED_ORIENTATION			0x00001000		// 1701
#define RVLPSULMBUILDER_CREATEMODEL_FLAG_PERMANENT				0x00000001
#define RVLPSULMBUILDER_CREATEMODEL_FROM_IMAGE					0x00000002
#define RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE				0x00000004
#define RVLPSULMBUILDER_CREATEMODEL_APPEND						0x00000008
#define RVLPSULMBUILDER_CREATEMODEL_APPEND_LAST					0x00000010
#define RVLPSULMBUILDER_CREATELOCALMAP_FLAG_BIDIRECTIONAL		0x01
#define RVLPSULM_MSMATCH_FLAG_INIT_GEOM_CONSTR_TESTED			0x01
#define RVLPSULM_MSMATCH_FLAG_INIT_GEOM_CONSTR_NOT_SATISFIED	0x02
#define RVLPSULM_MSMATCH_FLAG_GEOM_CONSTR_NOT_SATISFIED			0x04

#define RVLPSULM_MATCH_TYPE_SURFACE								0x00
#define RVLPSULM_MATCH_TYPE_LINE								0x01
#define RVLPSULM_MATCH_TYPE_AUTO								0x02

#define RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG			0x00000001

#define RVLPSULMBUILDER_HYPOTHESES_PREDICTION_HORIZON	3
#define RVLPSULMBUILDER_GETLOCALMODELS_FLAG_ORIENT_CONSTR		0x01
#define RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT				0x02
#define RVLPSULMBUILDER_HYPEVAL_FLAG_LINE_POSITION				0x00000001
#define RVLPSULMBUILDER_HYPEVAL_FLAG_SURFACE_POSITION			0x00000002

#define RVLPSULMBUILDER_MAX_LOCAL_MAP_HT_SIZE					1000
#define RVLPSULMBUILDER_MAXN_PSULMS								10000

#define RVLPSULMBUILDER_HYPEVAL4_FLAG_FIRST_ORDER_DEPENDENCY_TREE	0x00000001
#define RVLPSULMBUILDER_HYPEVAL4_FLAG_DYNAMIC_SURF_DETECT			0x00000002

struct RVLPSULM_CELL_CONST				
{
	int u, v;
};

struct RVLPSULM_LINE_COVERAGE_INTERVAL
{
	int p1, p2;
	RVLPSULM_LINE_COVERAGE_INTERVAL *pNext;
};

struct RVLPSULM_HG_NODE
{
	int iMatch;
	RVLPSULM_HG_NODE *pParent;
	int Support;
	int g;
	int nFailures;
	CRVL3DPose PoseSM;
	double P[3 * 3 * 3];
	double invt[3];
	void *pNext;
	void **pPtrToThis;
};

struct RVLPSULM_LASTDOF_MATCH_DATA
{
	RVLPSULM_MSMATCH_DATA *pMatch;
	double TraveledDist;
	double wTD;
	double Support;
	double score;
};

struct RVLPSULM_LOCAL_MODEL_LIST_ENTRY
{
	CRVLPSuLM *pPSuLM;
	double uncert;
	void *pNext;	
};

struct RVLColorMapDBNode
{
	float noBinPix;		//Number of bin pixels(float because of support for soft histograms)
	int noUsedPix;		//Number of used pixels in object
	ushort MapNodeIdx;	//Index/label of map node
	ushort ObjIdx;		//Index/label of object in map node
};

struct RVLPSULM_MATCH
{
	void *vpSObject;
	void *vpMObject;
	int cost;
};

struct RVLPSULM_MATCH2
{
	void *vpSObject;
	void *vpMObject;
	double cost;
	BYTE Type;
};

struct RVLPSULM_SCENE_FUSION
{
	CRVLMem m_Mem;
	RVLPSULM_HYPOTHESIS_SCENE_FUSION *m_HypothesisMem;
	RVLQLIST m_HypothesisList;	
	int m_nHypotheses;
	RVLPSULM_HYPOTHESIS_SCENE_FUSION **m_HypothesisArray;
	double m_rLookAround;
	double m_rMove;
};

struct RVLPSULM_MODEL_FUSION
{
	CRVLMem m_Mem;
	CRVL3DSurface2 **m_SurfaceArray;
	CRVL3DLine2 **m_LineArray;
	int m_nSurfaces;
	int m_nLines;
	CRVLPSuLM **m_SurfacePSuLMArray;
	CRVLPSuLM **m_LinePSuLMArray;
	double *RM_S;
	double *tM_S;
};

struct RVLPSULM_MODEL_FUSION_FEATURE
{
	void *vpFeature;
	CRVLPSuLM *pMPSuLM;
	void *pNext;
};

struct RVLPSULM_DEBUG_DATA
{
	CRVLGUI *pGUI;
	RVLPSULM_MATCH2 *MatchArray;
	int nMatches;
};

//struct RVLPSULM_MATCH
//{
//	void *vpSObject;
//	CRVL3DObject3 *pMObject;
//};
//
//struct RVLPSULM_MATCH_ARRAY
//{
//	CRVL3DObject3 *pSObject;
//	RVLPSULM_MATCH *pFirst;
//	RVLPSULM_MATCH *pLast;
//	
//	//used to store temp values in EKF of surfaces
//	double m_e[3];
//	double m_C[18];		//3 * 6
//	double m_Q[9];		//3 * 3
//};

//void RVL2DContourSegment(CvPoint *PtArray,				// transfer to RVL2DContour.cpp
//						 int nPts,
//						 int maxLineSegmentErrNrm,
//						 CvPoint **BreakPtPtrArray,
//						 int &nSegments);
void RVL2DContourSegment(CvPoint *pPt1,				// transfer to RVL2DContour.cpp
						 CvPoint *pPt2,
						 int maxLineSegmentErrNrm,
						 CvPoint ***pppBreakPt);
void RVLPSuLMHypothesisPoseRefinement(RVLPSULM_HYPOTHESIS *pHypothesis,
									  CRVLPSuLM *pSPSuLM,
									  CRVL3DSurface2 **MatchedMSurfArray);
void RVLPSuLMHypothesisPoseRefinement(CRVL3DPose *pPose,
									  RVLPSULM_HG_NODE *pNode,
									  RVLPSULM_MSMATCH_DATA *MatchList,
									  double *PInit,
									  int nIterations);
void RVLPSuLMHypothesisGetAbsPose(RVLPSULM_HYPOTHESIS *pHypothesis,
								  CRVL3DPose *pPoseAC,
								  CRVL3DPose *pPoseCs0);

void RVLPSuLMHypothesisGetAbsPose3DOF(RVLPSULM_HYPOTHESIS *pHypothesis,
									  CRVL3DPose *pPoseAC,
									  CRVL3DPose *pPoseAs0);

void RVLDeterminePose6DOFTo3DOF(CRVL3DPose *pPoseAAp, 
								CRVL3DPose *pPoseCCp, 
								CRVL3DPose *pPoseAC);

double BinomialCoefficient(int n, int k);
int NonRandomness(int m, int n);
int RansacRoundsNeeded(int maxNoRounds, int m, double logProsacConfidence, int n, int Inbest);
void RandPerm(int n, int perm[]);
//int niter_RANSAC(double p, double epsilon, int s, int Nmax);

class CRVLPSuLMIndexing;

class CRVLPSuLMBuilder : public CRVLRLM
{
public:
	DWORD m_Flags2;
	CRVLMem *m_pMem0;
	CRVLMem *m_pMem;
	CRVLMem *m_pMem2;
	CRVLMem *m_pMCMem;
	CRVLCamera *m_pCamera;
	CRVLStereoVision *m_pStereoVision;
	double m_PCScale;
	int m_CellSize;
	int *m_iSampleCellArray;
	int m_nSampleCells;
	int m_nCols, m_nRows;
	int m_nCells;
	CRVLTimer *m_pTimer;
	CRVLMPtrChain m_PSuLMList;
	CRVLPSuLM **m_PSuLMArray;
	//CRVLC3D m_S3DSurfaceSet;
	//CRVLC3D m_M3DSurfaceSet;
	//CRVLC3D m_S3DConvexSegmentSet;
	//CRVLC3D m_M3DConvexSegmentSet;
	//CRVLC3D m_S3DContourSet;
	//CRVLC3D m_M3DContourSet;
	CRVLClass m_S3DSurfaceSet;
	CRVLClass m_M3DSurfaceSet;
	CRVLClass m_S3DConvexSegmentSet;
	CRVLClass m_M3DConvexSegmentSet;
	CRVLClass m_S3DContourSet;
	CRVLClass m_M3DContourSet;
	CRVLClass m_S2DLineSet;
	CRVLClass m_M2DLineSet;
	//CRVLC3D m_S3DLineSet;
	//CRVLC3D m_M3DLineSet;
	CRVLClass m_S3DLineSet;
	CRVLClass m_M3DLineSet;
	CRVLClass m_2DContourSet;
	RVLQLIST m_2DContourList;
	BYTE *m_2DContourMap;
	RVLRECT m_ROI;
	CRVLPlanarSurfaceDetector *m_pPSD;
	CRVLPSuLMIndexing m_Indexing;
	CRVLAImage *m_pAImage;
	RVLPSULM_CELL *m_CellArray;
	RVLPSULM_CELL_CONST *m_CellConstArray;
	CvPoint *m_PtBuff;
	RVL3DCONTOUR_CROP_PARAMS m_CropLTs;
	RVLPSULM_CELL *m_EmptyCellArray;
	int m_uTol;
	int m_dqTol;
	double m_PhiTol;
	int m_iApproxPolyPrecision;	  //Precision when approximating poligon for LEVEL2 regions
	int m_iApproxPolyPrecision2;  //Precision when approximating poligon for LEVEL1 regions
	int m_2DLineLengthTreshold;
	int m_2DLineLengthTreshold2;
	CRVLPSuLM m_PSuLMTemplate;
	RVLPSULM_LINE_COVERAGE_INTERVAL *m_LineCoverage;
	double m_BetaTol;
	double m_Beta0;
	double m_ThetaTol;
	double m_CameraHeight0;
	double m_CameraHeightTol;
	int *m_AlphaHist;
	int m_AlphaResolution;
	int m_AlphaFilterSize;
	double m_HorLineChi2Thr;
	double m_VerLineChi2Thr;
	CRVLMPtrChain m_HypothesisList;
	double m_PositionUncert;
	double m_OrientUncert;
	//CRVLCluster m_HypothesisClustering;
	int m_maxnLandmarks;
	int m_maxnLines;
	RVLPSULM_HYPOTHESIS *m_pBestHypothesis;
	double m_LocalizationTime;
	double m_DynamicSurfRejectTime;
	double m_CreateTime;
	double m_HypGenTime;
	double m_HypEvalTime;
	int m_VisibilityThr;
	int m_minnHypSurfPts;
	//CRVLSegmentationEB *m_pSegmentation;
	CRVLParameterList m_ParamList;
	CRVL3DPose *m_pPoseSA;
	double m_maxPoseSMUncert;
	RVLPSULM_DEBUG_DATA m_DebugData;
	int m_ProjDisparityErr;
	CRVL3DSurface2 *m_MSurfPrediction[RVLPSULMBUILDER_HYPOTHESES_PREDICTION_HORIZON];
	CRVL3DSurface2 *m_SSurfPrediction[RVLPSULMBUILDER_HYPOTHESES_PREDICTION_HORIZON];
	char *m_ImageFileName;
	char *m_ModelDatabasePath;
	char *m_ModelMapPath;
	char *m_SequenceScenePath;
	char *m_DataSetPath;
	RVLPSULM_HYPOTHESIS **m_HypothesisArray;
	int m_nHypotheses;
	RVLQLIST_PTR_ENTRY *m_RepresentativeHypothesisMem;
	RVLQLIST m_RepresentativeHypothesisList;
	int m_nRepresentativeHypotheses;
	RVLPSULM_PARTICLE *m_ParticleArray;
	int m_nParticles;
	int m_refnParticles;
	RVLPSULM_PARTICLE *m_ParticleCbVArray;
	int m_nParticlesCbV;
	int m_nPlausibleHypotheses;
	int m_HypothesisScoreForOneParticle;
	int m_maxnHypothesesPerModel;
	BYTE *m_MatchMatrix;
	int m_maxnDominant3DSurfaces;
	int m_maxnDominant3DSurfacesComplex;
	int m_maxnModel3DSurfaces;
	int m_maxnDominant3DLines;
	int m_maxnDominant3DLinesComplex;
	int m_maxnModel3DLines;
	int m_maxnExpandedNodes;
	double m_RotHypTol, m_tHypTol;
	int m_refnHypotheses;
	double m_maxZ;
	double m_HypSeparationAngle;
	double m_LastDOFSeparationAngle;
	double m_LastDOFSurfNrmAngle;
	double m_MahDistDOFSurfOverlapTest;
	double m_LastDOFLineOverlapThr;		// todo: add to param. list
	double m_LastDOFLineRotationStD;	// todo: add to param. list
	double m_LastDOFLineTranslationStD;	// todo: add to param. list
	double m_maxLastDOFTravelDist;
	double m_LastDOFResolution;			// todo: add to param. list
	int m_nLastDOFGaussianSamples;		// todo: add to param. list
	double m_maxLastDOFstdTD;			// todo: add to param. list
	CRVLQListArray m_LocalMapHT;
	//int m_minLineBoundingBoxSize;
	int m_minContourSize;
	int m_minLineDepthStep;
	int m_minnLineDepthSteps;
	double m_minRelevantLogLikelihood;
	double m_RepresentativeHypDistThr;
	double m_RepresentativeHypOrientThr;
	double m_OdometryUncertConst[4];
	double m_FloorUncertConst;
	double m_RobotParams[2];
	CRVL3DPose *m_pPoseCB;					//camera wrt B
	CRVL3DPose *m_pPoseAC;					//robot(A) wrt camera(C) ie poseAC
	CRVL3DPose *m_pPrevCameraPoseSM;		//current camera pose -> scene wrt to model (obtained in the previous step)
	CRVL3DPose *m_pPrevRobotPoseSM;			//current robot pose -> scene wrt to model (obtained in the previous step)
	CRVL3DPose *m_pModelAbsPose;			//absolute pose of previous Model PSuLM
	CRVLPSuLM  *m_pNearestModelPSuLM;       //nearest Model PSuLM;
	CRVLPSuLM  *m_pPrevModelPSuLM;			//previous Model PSuLM;		
	CRVLPSuLM  *m_pLoopStartPSuLM;			//PSuLM at the beginning of a loop
	double m_MinHybridLocalizationDist;		//min distance between models used for hybrid localization
	double m_MinHybridLocalizationAngle;	//min angle between models used for hybrid localization 
	CRVL3DPose *m_pPoseSp0;
	CRVL3DPose *m_pPoseSSp;
	CRVL3DPose m_PoseAmp0;
	RVLPSULM_PARTICLE m_BestParticle;
	RVLPSULM_PARTICLE m_BestParticleCbV;
	
	CRVLMPtrChain m_PSuLMSubList;

	//**FILKO**//
	double m_MatIntersectTHR;	//threshold value for color intersection
	float *m_IntersectionHelperArray;	//a helper array for histogram intersection so it runs faster
	void **m_colorMapDB;		//db of objects indexed by histogram bin
	//**FILKO**//

	//CRVLMPtrChain m_MapPSuLMList;
	CRVL3DPose m_PoseSMInit;
	double m_CSMInit[3 * 3 * 3];
	int m_nSubMaps;
	int m_maxPSuLMIndex;

	//Hypothesis evaluation
	double *m_MatrixSMCost;
	double *m_AutoMatchMatrix;
	int *m_MatrixSMCounter;
	RVLPSULM_SMATCH_DATA *m_SMatchArray;
	RVLPSULM_MATCH2 **m_AutoMatchArray;
	int m_nAutoMatches;
	
	RVLPSULM_CELL2 *m_CellArray2;
	RVLPSULM_CELL2 *m_EmptyCellArray2;
	int m_nCols2, m_nRows2;
	int m_nCells2; 
	int m_CellSize2;
	int m_nCellsPer45deg;
	//int *m_CellIdxMap;
	RVLQLIST_PTR_ENTRY *m_CellMem;

	int m_minCellPts;

	double m_ProbabilitySameSurface;	//Probability that F and F' represent the same surface in the scene 
	double m_rhoMax;					//rhoMax - maximum distance at which a surface can be detected
	double m_PDFDifferentSurface;		
	double m_Cost2;

	double m_minSceneInfo;
	double m_minModelInfo;

	BOOL m_minScenePlaneExists;
	BOOL m_minModelPlaneExists;
	double m_GlobalLocalizationDistTHR;					//distance tolerance(threshold) for correct Global localization
	double m_GlobalLocalizationAngleTHR;				//angle tolerance(threshold) for correct Global localization
	char *m_IncrementalLocalizationResultsFilePath;		//Incremental localization results file path (to be used for global localisation evaluation
	double m_SampleMatchAngleStD;
	double m_SampleMatchDistStD;
	double m_SampleMatchUncertCoef;
	double m_SampleMatchAngleTol;
	double m_SampleMatchDistTol;
	int m_minSurfaceSampleCoverage;			// todo: add to param. list
	int m_PlausibilityThr;
	int m_minSurfaceSamplesForMatch;
	double m_LocalMapRadius;
	FILE *m_fpDebug;	
	//double m_BestHypothesisProbability;
	//double m_BestHypothesisProbability5DOF;
	double m_PriorProbabilityGlobal;
	double m_PriorProbabilityWorldModel;

	CRVL3DSurface2 *m_SurfaceMSArray;
	CRVL3DLine2 *m_LineMSArray;
	RVL3DSURFACE2_MATCH_DATA m_SurfaceMatchData;
	RVL3DLINE2_MATCH_DATA m_LineMatchData;
	DWORD m_HypothesisEvaluationFlags;
	RVLPSULM_SCENE_FUSION m_SceneFusion;
	RVLPSULM_MODEL_FUSION m_ModelFusion;
	CRVLFreiburgDataSet m_DataSet;
	
//#ifdef PYTHON_DEBUG
//	PyObject *m_pyModuleName;
//	PyObject *m_pyModule;
//	//PyObject *m_pyFunction;
//#endif

private:
	int m_nrmCellSize;
	int *m_ScanLineStartArray, *m_ScanLineEndArray;
	int *m_ScanLineEndMap;
	CRVL2DLine2 **m_MatchedLineBuff;
	double m_csPhiTol;
	CRVLQListArray m_LineSortBuff;
	int m_iApproxPolyPrecision2Nrm;
	double m_maxvarRotHyp, m_maxvartHyp;
	double m_csLastDOFSeparationAngle;
	double m_csLastDOFSurfNrmAngle, m_csLastDOFLineNrmAngle;
	RVLPSULM_MATCH2 *m_AutoMatchMem;
	double m_kPan;
	double m_kTilt;
	double m_TiltOffset;

	void PythonDisplayScene(RVLSURFACE_MATCH_ARRAY *MatchArray, CRVLMPtrChain *pM3DSurfaceList, CRVL3DSurface2 **MatrixSceneModel, int n3DSceneSurfaces);
	//void GetMaxProbabilityMatch(int iS3DSurface,
	//							int maxnM3DSurfaces,
	//							int nM3DSurfaces,
	//							RVLPSULM_MSMATCH_DATA *MatrixSceneModel,
	//							RVLPSULM_SURF_MATCH_DATA *SSurfMatchData);

public:
	CRVLPSuLMBuilder(void);
	virtual ~CRVLPSuLMBuilder(void);
	void Init(void);
	void RegionSampling(CRVLMPtrChain *pContourList,
						int nContours = 1);
	void LineSampling(CvPoint * pPt1, CvPoint * pPt2);
	void DisplayCells(CRVLFigure * pFig);
	void DisplayCellArray(	CRVLFigure *pFig,
							RVLPSULM_CELL *CellArray,
							CRVL3DPose *pPose,
							CvScalar Color);
	BOOL Create(CRVLPSuLM *pPSuLM, 
				CRVLMem *pMem,
				DWORD Flags = 0x00000000,
				CRVL3DPose *pPoseM_M = NULL,
				int iView = 0);
	CRVLPSuLM *Create(	
		DWORD Flags = 0x00000000,
		CRVLPSuLM *pPSuLM_ = NULL);
	int MatchLine(	CRVL3DLine2 *pLine,
					CRVLPSuLM *pPSuLM,
					CRVL3DPose *pPose,
					BOOL bInvTransf,
					int &visible,
					CRVLMem *pMemExternal = NULL);
	int MatchLine(	CvPoint *pPt1, CvPoint *pPt2,
					CRVLPSuLM *pPSuLM,
					int &visible,
					CRVLMem *pMemExternal = NULL);
	
	int MatchSurface(	CRVL3DSurface2 *pSurface,
						CRVLPSuLM *pPSuLM,
						CRVL3DPose *pPose,
						BOOL bInvTransf,
						CRVLMem *pMemExternal = NULL);

	//int MatchSurface(	CvPoint *pPt1, CvPoint *pPt2,
	//				CRVLPSuLM *pPSuLM,
	//				int &visible,
	//				CRVLMem *pMemExternal = NULL);

	CRVLPSuLM *Load(FILE *fp);
	void CreateRotLG(	double * NGroundPlane,
						double *RotLG);
	void Hypotheses(CRVLPSuLM *pSPSuLM, 
					CRVL3DPose *pPoseS0Init);
	void Hypotheses2(	CRVLPSuLM *pSPSuLM, 
						CRVL3DPose *pPoseS0Init);
	void Hypotheses3(	CRVLPSuLM *pSPSuLM, 
						CRVL3DPose *pPoseS0Init,
						double *PInit,
						CRVLPSuLM * pPrevSPSuLM);
	void Hypotheses4(	CRVLPSuLM *pSPSuLM, 
						CRVL3DPose *pPoseS0Init,
						double *PInit,
						CRVLPSuLM * pPrevSPSuLM);
	void Hypotheses5(	CRVLPSuLM *pSPSuLM, 
						CRVL3DPose *pPoseS0Init,
						double *PInit,
						CRVLPSuLM * pPrevSPSuLM);
	void Localization(	CRVLPSuLM * pSPSuLM,
						CRVL3DPose *pPoseS0,
						CRVLPSuLM * pPrevSPSuLM = NULL);
	bool MapBuilding(CRVLPSuLM *pSPSuLM);
	int EvaluateHypothesis(	CRVLPSuLM * pSPSuLM,
							RVLPSULM_HYPOTHESIS *pHypothesis);
	int EvaluateHypothesis2( CRVLPSuLM * pSPSuLM,
							 RVLPSULM_HYPOTHESIS *pHypothesis);
	int EvaluateHypothesis3( CRVLPSuLM * pSPSuLM,
							 RVLPSULM_HYPOTHESIS *pHypothesis,
							 bool bDynamicSurfaceDetection = false);
	int EvaluateHypothesis3Simple(
		CRVLPSuLM * pSPSuLM,
		RVLPSULM_HYPOTHESIS *pHypothesis,
		bool bDynamicSurfaceDetection = false);
	double EvaluateHypothesis4( CRVLPSuLM * pSPSuLM,
								RVLPSULM_HYPOTHESIS *pHypothesis,
								DWORD Flags = RVLPSULMBUILDER_HYPEVAL4_FLAG_FIRST_ORDER_DEPENDENCY_TREE);
	void InitHypothesisEvaluation4(CRVLPSuLM * pSPSuLM);
	void Load(char * FileName, int maxIndex);
	void LoadMap();
	RVLPSULM_HYPOTHESIS * GetHypothesis(int index);
	void CreateParamList(CRVLMem * pMem);
	void CreateMatchMatrix(	CRVLPSuLM *pSPSuLM,
							CRVLPSuLM *pMPSuLM,
							CRVL3DPose *pPoseSM,
							double OverlapCoeff,
							CRVL3DSurface2 **MatchedMSurfArray = NULL);
	void CreateAutoMatchMatrix(	CRVLPSuLM *pSPSuLM);
	int GenMatchListViaDescriptors(CRVLPSuLM * pPSuLM_S,
									CRVLPSuLM * pPSuLM_M,
									RVLQLIST *matchList,
									float thr = 0.1,
									int minNoPts = -1);

	void DetermineOdometryUncertainty(double *CovAB, CRVL3DPose *pPoseAB);
	void DetermineUncertainty3DOFTo6DOF(double *PUncert, CRVL3DPose *pPoseCCpInit, CRVL3DPose *pPoseAAp, double *Uncert3DOF = NULL);
	void DetermineUncertainty6DOFTo3DOF(double *Uncert3DOF, double *Uncert6DOF, CRVL3DPose *pPoseAAp);
	void UncertaintyEKFPrediction6DOF(double *UncertC0, double *UncertCN, double *UncertN0,  CRVL3DPose *pPoseCN, CRVL3DPose *pPoseN0);
	void UncertaintyEKFPrediction3DOF(double *UncertC0, double *UncertCN, double *UncertN0,  CRVL3DPose *pPoseCN, CRVL3DPose *pPoseN0);
	void InvUncertainty(double *UncertBA,double *UncertAB, CRVL3DPose *pPoseAB);
	CRVLPSuLM *Clone(CRVLPSuLM *pPSulMOriginal);
	bool SaveCurrentData(CRVLPSuLM *pSPSuLM, CRVLPSuLM *pMPSuLM, CRVL3DPose *pBestPose, CRVL3DPose *pPoseS0, int BestHypCost = -1);
	void RobotCameraPose();
	void DeterminePose6DOFTo3DOF(CRVL3DPose *pPoseAAp, CRVL3DPose *pPoseCCp);
	void PanTiltRollUncertainty(double *OutUncert, double *InUncert,CRVL3DPose *pPoseAAp, bool bInverse = false);
	//void PanTiltRollUncertaintyInv(double *OutUncert, double *InUncert,CRVL3DPose *pPoseAAp);

	void SaveXMLMap(char* filepath, RVLQLIST *pMap);
	RVLQLIST* LoadXMLMap(char* filepath, CRVLMem * pMem);
	static CRVLMPtrChain* ConvertQLIST2PtrChain(RVLQLIST *pQList, CRVLMem * pMem);
	static RVLQLIST* ConvertPtrChain2QLIST(CRVLMPtrChain *pPtrChain, CRVLMem * pMem);
	static float GetColorSceneSimilarityScore(CRVLPSuLM *pMPSuLM, CRVLPSuLM *pSPSuLM);
	void AddNodeToColorMapDB(void** colorMapDB, CRVLPSuLM* mapnode, CRVLMem * pMem);
	void SaveColorMapDBXML(void** colorMapDB, char* filepath, int arraysize, char* colorsystem, int binres, int histDimension);
	void** LoadColorMapDBXML(char* filepath, CRVLMem * pMem);
	void GetLocalModels(CRVL3DPose *pPoseAsAsp, 
						CRVL3DPose *pPoseAspAmp,
						CRVLPSuLM *pMPSuLM0,
						double DistCoeff,
						BYTE Flags = 0x00);
	void GetLocalModels2(	CRVL3DPose *pPoseAsAsp, 
							CRVL3DPose *pPoseAspAmp,
							CRVLPSuLM *pMPSuLM0,
							double DistCoeff,
							BYTE Flags = 0x00);
	void GetModelPoses( CRVL3DPose *pPoseAsAm0,
						CRVLPSuLM *pMPSuLM0,
						CRVLPSuLM **ModelList,
						int &nModels,
						int nTgtModels = 0);
	void MapLog(FILE *fpMapNodesLog, FILE *fpMapConnectionsLog);
	void GetGroundTruth(CRVLPSuLM *pPSulM);
	void GetGroundTruth(CRVLPSuLM *pPSulM, double *pGTData);
	int GetMostProbableDistance(int iPose0, 
								int iNeighbor,
								double *dX,
								double &dAlpha);
	int GetPathPlanningNodeIndex(int iPose0, int iNeighbor);
	int GetnReachableModels(int iPose0);
	void UpdateMatchMatrixForDisplay(	CRVLPSuLM *pSPSuLM, 
										RVLPSULM_HYPOTHESIS *pHypothesis);
	int GetnModels(void);
	void PreparePathPlanning();
	BOOL LoadPathPlanningMap(char *PathPlanningMapFileName);
	BOOL SavePathPlanningMap(char *PathPlanningMapFileName);
	void CreateLocalMap(CRVLPSuLM * pPSuLM,
						BYTE Flags = 0x00);
	void CreateLocalMaps();
	BOOL LoadLocalMaps(char *LocalMapsFileName);
	BOOL SaveLocalMaps(	char *LocalMapsFileName);
	void Connect(	CRVLPSuLM * pPSuLM1, 
					CRVLPSuLM * pPSuLM2,
					CRVL3DPose *pPoseAsAm,
					CRVL3DPose *pPoseCsCm = NULL);

	void HypothesesPROSAC(	CRVLPSuLM *pSPSuLM,
							CRVL3DPose *pPoseS0Init,	// Initial uncertainty for EKF update
							double *PInit,				// Uncertainty for initial matching
							CRVLPSuLM * pPrevSPSuLM);
	int ProsacEX(RVLPSULM_MSMATCH_DATA *pMatchList, int N, CRVL3DPose *pPoseS0Init, CRVLPSuLM *pMPSuLM);
	int Prosac(RVLPSULM_MSMATCH_DATA *pMatchList, int N, CRVL3DPose *pPoseS0Init, CRVLPSuLM *pMPSuLM);
	int Ransac(RVLPSULM_MSMATCH_DATA *pMatchList, int N, CRVL3DPose *pPoseS0Init, CRVLPSuLM *pMPSuLM);
	void DisplayHypothesis(	CRVLGUI *pGUI, 
							CRVLFigure *pFig, 
							CRVLFigure *pFig2, 
							CRVLPSuLM *pPSULM, 
							DWORD Flags,
							IplImage *pImage,
							IplImage *pImage2,
							int iHypothesis = 0);
	void DisplayHypothesisData(	CRVLFigure *pFig, 
								CRVLPSuLM *pSPSuLM, 	
								char *MatchMatrixGT,
								DWORD Flags = 0x00000000,
								int iHypothesis = 0,
								CRVL3DSurface2 *pSelectedSurface = NULL,
								CRVL3DLine2 *pSelectedLine = NULL,
								RVL3DSURFACE_SAMPLE *pSelectedSurfSample = NULL);
	CRVLPSuLM *GetPSuLM(int index);		
	void PoseConstraintProbability(	CRVLPSuLM *pSPSuLM, 
									CRVLPSuLM *pMPSuLM);
	double ConditionalProbabilityTree(	CRVLPSuLM *pSPSuLM, 
										CRVLPSuLM *pMPSuLM = NULL,
										int *iSMMatchArray = NULL,
										int nSMSurfMatches = 0,
										int nSMLineMatches = 0,
										RVLPSULM_MATCH2 **AutoMatchArray_ = NULL,
										int nAutoMatches_ = 0);
	void DeleteConnection(	CRVLPSuLM *pPSuLM1,
							CRVLPSuLM *pPSuLM2);
	void DeleteConnection_(	CRVLPSuLM *pPSuLM1,
							CRVLPSuLM *pPSuLM2);
	void UpdateRelativePoseUncertainties();	
	void GetProjectionMatrix(double *P);
	void SceneFusion();
	void GetConnectedSubMap(CRVLPSuLM *pPSuLM0);
	bool GetRelativePose(	CRVLPSuLM *pMPSuLM,
							RVLPSULM_HYPOTHESIS *pHypothesis_,
							CRVL3DPose **PoseM_M,
							CRVL3DPose *pPoseS_M,
							bool bOrientation = true);
	void GetNeighborPSuLMs(	CRVLPSuLM *pMPSuLM,
							CRVL3DPose **PoseM_M);
	void ResetCloseFlags(CRVLPSuLM *pMPSuLM);
	void RepresentativeHypotheses();
	void ModelFusion(RVLPSULM_HYPOTHESIS *pHypothesis);
	IplImage * GetComplexPSuLMRGBImage(	char *ImageFileName);
	void MergeSurfaces(CRVLPSuLM *pPSuLM);
	void MergeLines(CRVLPSuLM *pPSuLM);
	void UpdateBuffers(CRVLPSuLM *pPSuLM);
	bool GetOdometry(
		char *ImageFileName,
		CRVL3DPose *pPose,
		char *extension,
		int &iSample0,
		unsigned char &command);
	void GetCell(
		double *X,
		int & i, 
		int & j);
	void InitHypothesisEvaluation3(CRVLPSuLM * pPSuLM);
	bool Compute5DoFPose(
		RVLPSULM_HG_NODE *pNode,
		RVLPSULM_MSMATCH_DATA *MatchList,
		double *tInit,
		RVLSURFACE_MATCH_ARRAY *pMatchData,
		CRVL3DPose *pPoseSM);

private:
	RVLPSULM_MSMATCH_DATA *HypothesesGetNextNode(	RVLPSULM_HG_NODE* pNode,
													int iStartMatch,
													RVLPSULM_MSMATCH_DATA *MatchList,
													RVLPSULM_MSMATCH_DATA *pMSMatchListEnd,
													int &PredictedScore);
	int HypothesisDisplay(	CRVLFigure * pFig, 
							RVLPSULM_HG_NODE *pNode,
							CvPoint position,
							RVLPSULM_MSMATCH_DATA *MatchList);
	void PrintHypothesis(	FILE *fp, 
							RVLPSULM_HG_NODE *pNode,
							RVLPSULM_MSMATCH_DATA *MatchList);	
public:
	void ProjectToCube(
		double * P,
		double & p,
		double & q,
		int & ip,
		int & iq,
		int & ir);
	void CompareHypotheses(
		RVLPSULM_MATCH2 * MatchArray1, 
		int nMatches1, 
		RVLPSULM_MATCH2 * MatchArray2, 
		int nMatches2, 
		char * OutputFileName);
private:
	void MatchDiff(
		RVLPSULM_MATCH2 * MatchArray1,
		int nMatches1,
		RVLPSULM_MATCH2 * MatchArray2,
		int nMatches2,
		FILE *fp);
};
