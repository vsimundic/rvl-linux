

//#ifdef PYTHON_DEBUG
//#include "python.h"
//#include "numpy/arrayobject.h"
//#endif

//#include "stdafx.h"		// remove after moving CRVLPSuLMBuilder to RVL2
//#include "highgui.h"
#include <flann\flann.hpp>
#include "RVLCore.h"
#include "RVLPCS.h"
#include "RVLRLM.h"
#include <fstream>
#include <sstream>
#include <string>
#include "RVLPSuLMBuilder.h"
//#include "RVLCommon2.h"
//#include "RVLObjectLib.h"
//#include "RVLAImage.h"
//#include "RVLEDT.h"
//#include "RVLDelaunay.h"
//#include "RVLCamera.h"
//#include "RVLStereoVision.h"
//#include "RVLSegmentationEB.h"
//#include "RVLPlanarSurfaceDetector.h"
//#include "RVLCluster.h"
//#include "RVLSearchProblem.h"
//#include "RVLRLM.h"
//#include "RVLPSuLMBuilder.h"
//#include <fstream>
//#include <sstream>
//#include <string>
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
using namespace std;
using namespace rapidxml;

//#define RVLPSULMBUILDER_DEBUG

CRVLPSuLMBuilder::CRVLPSuLMBuilder(void)
{
	// flags

	m_Flags2 = 0x00000000;

	// parameters

	m_PCScale = 1000.0;
	m_CellSize = 16;				// pix
	m_uTol = m_CellSize;			// halfpix
	m_dqTol = 6;					// halfpix
	m_PhiTol = 30.0 * DEG2RAD;		// rad
	m_CropLTs.minz = 100.0;			// mm
	m_iApproxPolyPrecision = 3;		// ?
	m_iApproxPolyPrecision2 = 2;	// ?
	m_iApproxPolyPrecision2Nrm = m_iApproxPolyPrecision2 * 1000;
	m_2DLineLengthTreshold = 20;	// pix
	m_BetaTol = 10.0 * DEG2RAD;		// rad
	m_Beta0 = 0.0;					// rad
	m_ThetaTol = 5.0 * DEG2RAD;		// rad
	m_CameraHeight0 = 534.0;		// mm
	m_CameraHeightTol = 50.0;		// mm
	m_PositionUncert = 1000.0;			// mm
	m_OrientUncert = 30.0 * DEG2RAD;	// rad
	m_AlphaResolution = 180 * 5;
	m_AlphaFilterSize = 21;
	//m_HypothesisClustering.m_AccuResolutionX = m_HypothesisClustering.m_AccuResolutionY = 100.0;	// mm
	//m_HypothesisClustering.m_minClusterSupport = 3;
	//m_HypothesisClustering.m_maxClusterSupport = 100;
	m_VisibilityThr = 50;
	m_maxPoseSMUncert = 200.0 * 200.0;	// mm^2
	//m_maxPoseSMUncert = 0.0 * 0.0;	// mm^2
	m_ProjDisparityErr = 8; //Disparity dif between projection and original plane
	m_maxnHypothesesPerModel = 20;
	m_maxnDominant3DSurfaces = 20;
	m_maxnDominant3DSurfacesComplex = 30;
	//m_maxnDominant3DSurfacesComplex = 100;
	m_maxnDominant3DLines = 20;
	m_maxnDominant3DLinesComplex = 30;
	//m_maxnDominant3DLinesComplex = 100;
	m_maxnModel3DSurfaces = 0;
	m_maxnExpandedNodes = 1000;
	m_RotHypTol = 3.0;	// deg
	m_tHypTol = 500.0;	// mm
	m_refnHypotheses = 20;
	m_HypSeparationAngle = 2.0;			// deg
	m_LastDOFSeparationAngle = 10.0;	// deg
	m_LastDOFSurfNrmAngle = 45.0;		// deg
	m_MahDistDOFSurfOverlapTest = 9.21;		// chi2inv(0.99, 2)
	m_LastDOFLineOverlapThr = 10.0;		// pix
	m_LastDOFLineRotationStD = 2.0;		// deg
	m_LastDOFLineTranslationStD = 30.0;	// mm
	m_maxLastDOFTravelDist = 3000.0;	// mm
	m_LastDOFResolution = 20.0;			// mm
	m_nLastDOFGaussianSamples = 10;
	m_maxLastDOFstdTD = 500.0;			// mm
	memset(m_OdometryUncertConst, 0, 4 * sizeof(double));
	m_FloorUncertConst = 0.0;
	m_RobotParams[0] = 325; //a mm
	m_RobotParams[1] = 165; //b mm
	m_MinHybridLocalizationDist = 500.0;	// mm
	m_MinHybridLocalizationAngle = 15.0;	// deg
	m_maxZ = 5000.0;						// mm
	m_minnHypSurfPts = 1000;
	m_SampleMatchAngleStD = 2.0;			// deg
	m_SampleMatchDistStD = 30.0;			// mm
	m_SampleMatchUncertCoef = 1.0;	
	m_minSurfaceSampleCoverage = 50;		// %
	m_PlausibilityThr = 5;
	m_HypothesisScoreForOneParticle = 100;
	m_minSurfaceSamplesForMatch = 3;
	m_LocalMapRadius = 10000.0;				// mm
	m_refnParticles = 100;
	//m_minLineBoundingBoxSize = 20;		// pix
	m_minContourSize = 30;
	m_minLineDepthStep = 50;				// pix
	m_minnLineDepthSteps = 10;
	m_SceneFusion.m_rLookAround = 500.0;	// mm
	m_SceneFusion.m_rMove = 1200.0;			// mm
	//m_minRelevantLogLikelihood = 30.0;		
	m_minRelevantLogLikelihood = 0.0;
	m_RepresentativeHypDistThr = 500.0;		// mm
	m_RepresentativeHypOrientThr = 20.0;	// deg
	//m_SurfaceMatchData.PPriorPosition = 7.4451;			// -log(1/sqrt(2*pi*(30.0^2))*exp(-2.5^2/2))
	//m_SurfaceMatchData.PPriorPosition = 4.6;				// 1 surface at each 100 mm
#ifdef RVLPSULMBUILDER_GT_141111
	m_SurfaceMatchData.PPriorPosition = 4.6;				// 1 surface at each 100 mm
#else
	m_SurfaceMatchData.PPriorPosition = 6.9;				// 1 surface at each 1000 mm
#endif
	//m_LineMatchData.PPriorPosition = 11.7653;			// -log(1/(2*pi*(30.0^2))*exp(-2.5^2/2))
	//m_LineMatchData.PPriorPosition1DOF = 7.4451;		// -log(1/sqrt(2*pi*(30.0^2))*exp(-2.5^2/2))
#ifdef RVLPSULMBUILDER_GT_141111
	m_LineMatchData.PPriorPosition = 9.21;			// 1 line in each 100 mm^2
#else
	m_LineMatchData.PPriorPosition = 13.81;			// 1 line in each 1000 mm^2
#endif
	m_LineMatchData.PPriorPosition1DOF = 4.6;		// 1 line at each 100 mm


	// constants computed from the parameters

	m_HorLineChi2Thr = m_VerLineChi2Thr = 2.5 * 2.5;
	//m_HypothesisClustering.m_minX = m_HypothesisClustering.m_minY = -m_PositionUncert;
	//m_HypothesisClustering.m_maxX = m_HypothesisClustering.m_maxY = m_PositionUncert;

//#ifdef RVLPSULMBUILDER_GT_141111
	m_HypothesisEvaluationFlags = (RVLPSULMBUILDER_HYPEVAL_FLAG_SURFACE_POSITION | RVLPSULMBUILDER_HYPEVAL_FLAG_LINE_POSITION);
//#else
//	m_HypothesisEvaluationFlags = 0x00000000;
//#endif
	
	// arrays

	m_ScanLineStartArray = m_ScanLineEndArray = NULL;
	m_ScanLineEndMap = NULL;
	m_iSampleCellArray = NULL;
	m_PtBuff = NULL;
	m_CellArray = NULL;
	m_CellConstArray = NULL;
	m_EmptyCellArray = NULL;
	m_AlphaHist = NULL;
	m_HypothesisArray = NULL;
	m_RepresentativeHypothesisMem = NULL;
	m_ParticleArray = NULL;
	m_ParticleCbVArray = NULL;
	m_nHypotheses = 0;
	m_MatchMatrix = NULL;
	m_PSuLMArray = NULL;
#ifdef RVLPSULM_LINES
	m_2DContourMap = NULL;
#endif
	m_SurfaceMSArray = NULL;
	m_LineMSArray = NULL;
	m_SurfaceMatchData.Cp = NULL;
	m_SurfaceMatchData.Cp_ = NULL;
	m_SurfaceMatchData.invCp = NULL;
	m_SurfaceMatchData.invCp_ = NULL;
	m_SceneFusion.m_HypothesisArray = NULL;
	m_ModelFusion.RM_S = NULL;
	m_ModelFusion.tM_S = NULL;
	m_MatrixSMCost = NULL;
	m_MatrixSMCounter = NULL;
	m_AutoMatchMatrix = NULL;
	m_AutoMatchArray = NULL;
	m_AutoMatchMem = NULL;
	m_SMatchArray = NULL;
	m_CellArray2 = NULL;
	m_EmptyCellArray2 = NULL;
	//m_CellIdxMap = NULL;
	m_CellMem = NULL;
	m_DebugData.MatchArray = NULL;

	// tools

	//m_pSegmentation = NULL;
	m_pPSD = NULL;

	// initial values

	m_maxnLines = 0;
	m_pBestHypothesis = NULL;
	m_ROI.left = -1;
	m_nSubMaps = 0;
	m_maxPSuLMIndex = -1;

	// templates

	m_PSuLMTemplate.m_vpBuilder = this;

	// tool flags 

	//m_HypothesisClustering.m_Flags = (RVLCLUSTER_FLAG_ZERO_CENTERED_X | RVLCLUSTER_FLAG_ZERO_CENTERED_Y);


	//Path for sequence processing
	m_ModelDatabasePath = NULL;
	m_SequenceScenePath = NULL;
	m_ModelMapPath = NULL;
	m_DataSetPath = NULL;

	//Hybrid localization
	m_pPrevCameraPoseSM = NULL;
	m_pPrevRobotPoseSM = NULL;
	m_pModelAbsPose = NULL;
	m_pPoseAC = NULL;
	m_pPoseSp0 = NULL;
	m_pPoseSSp = NULL;

	m_pPrevModelPSuLM = NULL;
	m_pNearestModelPSuLM = NULL;
	m_pLoopStartPSuLM = NULL;

	// Pose covariance memory allocation
	m_PoseSMInit.m_C = m_CSMInit;

	//Hypothesis evaluation
	m_CellSize2 = 20;		// nyarko_diss: 16
	m_minCellPts = (m_CellSize2 * m_CellSize2 * 20)/100; //20% of m_CellSize2^2
	m_nCellsPer45deg = 6;

	m_ProbabilitySameSurface = 0.9;
	m_rhoMax = 6000;	//mm
	m_PDFDifferentSurface = 1/(2*PI*PI*m_rhoMax);
	m_Cost2 = m_PDFDifferentSurface * (1 - m_ProbabilitySameSurface);

	//Incremental localization results file path (to be used for global localization evaluation
	m_IncrementalLocalizationResultsFilePath = NULL;


	//**FILKO**//
	m_MatIntersectTHR = 0.02; //treshold for material color intersection
	m_IntersectionHelperArray = new float[32*32*32];
	memset(m_IntersectionHelperArray, 0, 32 * 32 * 32 * sizeof(float));
	//**FILKO**//


	m_GlobalLocalizationDistTHR = 100.0;	//distance tolerance(threshold) for correct Global localization  100m
	m_GlobalLocalizationAngleTHR = 2.0;		//angle tolerance(threshold) for correct Global localization	 2°

	m_kPan = 1.0;
	m_kTilt = 1.0;
	m_TiltOffset = 0.0;		// deg
}

CRVLPSuLMBuilder::~CRVLPSuLMBuilder(void)
{
	if(m_ScanLineStartArray)
		delete[] m_ScanLineStartArray;

	if(m_ScanLineEndArray)
		delete[] m_ScanLineEndArray;

	if(m_ScanLineEndMap)
		delete[] m_ScanLineEndMap;

	if(m_iSampleCellArray)
		delete[] m_iSampleCellArray;

	if(m_PtBuff)
		delete[] m_PtBuff;

	if(m_CellArray)
		delete[] m_CellArray;

	if(m_CellConstArray)
		delete[] m_CellConstArray;

	if(m_EmptyCellArray)
		delete[] m_EmptyCellArray;

	if(m_AlphaHist)
		delete[] m_AlphaHist;

	if(m_ModelDatabasePath)
		delete[] m_ModelDatabasePath;

	if(m_SequenceScenePath)
		delete[] m_SequenceScenePath;

	if(m_HypothesisArray)
		delete[] m_HypothesisArray;

	if(m_RepresentativeHypothesisMem)
		delete[] m_RepresentativeHypothesisMem;

	if(m_ParticleArray)
		delete[] m_ParticleArray;

	if(m_ParticleCbVArray)
		delete[] m_ParticleCbVArray;

	if(m_MatchMatrix)
		delete[] m_MatchMatrix;

	if(m_ModelMapPath)
		delete[] m_ModelMapPath;

	if (m_DataSetPath)
		delete[] m_DataSetPath;

	if(m_CellArray2)
		delete[] m_CellArray2;

	if(m_EmptyCellArray2)
		delete[] m_EmptyCellArray2;

	//if (m_CellIdxMap)
	//	delete[] m_CellIdxMap;

	if (m_CellMem)
		delete[] m_CellMem;

	if(m_MatrixSMCost)
		delete[] m_MatrixSMCost;

	if(m_AutoMatchMatrix)
		delete[] m_AutoMatchMatrix;

	if(m_AutoMatchArray)
		delete[] m_AutoMatchArray;

	if(m_AutoMatchMem)
		delete[] m_AutoMatchMem;

	if(m_MatrixSMCounter)
		delete[] m_MatrixSMCounter;

	if(m_SMatchArray)
		delete[] m_SMatchArray;

	if(m_IncrementalLocalizationResultsFilePath)
		delete[] m_IncrementalLocalizationResultsFilePath;

	if(m_PSuLMArray)
		delete[] m_PSuLMArray;

	if(m_SurfaceMSArray)
		delete[] m_SurfaceMSArray;

	if(m_LineMSArray)
		delete[] m_LineMSArray;

	if(m_SurfaceMatchData.Cp)
		delete[] m_SurfaceMatchData.Cp;

	if(m_SurfaceMatchData.Cp_)
		delete[] m_SurfaceMatchData.Cp_;

	if(m_SurfaceMatchData.invCp)
		delete[] m_SurfaceMatchData.invCp;

	if(m_SurfaceMatchData.invCp_)
		delete[] m_SurfaceMatchData.invCp_;

	if(m_SceneFusion.m_HypothesisArray)
		delete[] m_SceneFusion.m_HypothesisArray;

	if(m_ModelFusion.RM_S)
		delete[] m_ModelFusion.RM_S;

	if(m_ModelFusion.tM_S)
		delete[] m_ModelFusion.tM_S;

#ifdef RVLPSULM_LINES
	if(m_2DContourMap)
		delete[] m_2DContourMap;
#endif

	if (m_DebugData.MatchArray)
		delete[] m_DebugData.MatchArray;

//#ifdef PYTHON_DEBUG
//	Py_Finalize();
//#endif
}

void CRVLPSuLMBuilder::Init(void)
{
	m_PSuLMList.m_pMem = m_pMem0;
	m_PSuLMSubList.m_pMem = m_pMem;

	// Flags

	DWORD HypEvalMethod = (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD);

	if (HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
		m_Flags2 |= RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING;

	// Parameters

	m_2DLineLengthTreshold2 = 4 * m_2DLineLengthTreshold * m_2DLineLengthTreshold;

	m_csLastDOFSeparationAngle = cos(m_LastDOFSeparationAngle * DEG2RAD);

	m_csLastDOFSurfNrmAngle = cos(m_LastDOFSurfNrmAngle * DEG2RAD);
	m_csLastDOFLineNrmAngle = sin(m_LastDOFSurfNrmAngle * DEG2RAD);

	m_SampleMatchAngleTol = m_SampleMatchAngleStD * DEG2RAD;
	m_SampleMatchDistTol = m_SampleMatchDistStD;

	m_maxnDominant3DSurfacesComplex = RVLMAX(m_maxnDominant3DSurfacesComplex, m_maxnDominant3DSurfaces);
	m_maxnDominant3DLinesComplex = RVLMAX(m_maxnDominant3DLinesComplex, m_maxnDominant3DLines);

	// DataSet

	if ((m_Flags2 & RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT) == RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG)
		if (m_DataSetPath)
			m_DataSet.Load(m_DataSetPath);

	// Cell array

	m_nCols = m_pCamera->Width / m_CellSize + 2;
	m_nRows = m_pCamera->Height / m_CellSize + 2;
	m_nCells = m_nRows * m_nCols;

	m_nrmCellSize = (m_CellSize << 1);

	m_CellArray = new RVLPSULM_CELL[m_nCells];

	m_CellConstArray = new RVLPSULM_CELL_CONST[m_nCells];

	m_EmptyCellArray = new RVLPSULM_CELL[m_nCells];

	RVLPSULM_CELL_CONST *pCellConst = m_CellConstArray;

	RVLPSULM_CELL *pCell = m_EmptyCellArray;

	//CRVLMPtrChain LinePtrListTemplate(m_pMem2);

	int Row, Col;

	for(Row = 0; Row < m_nRows; Row++)
		for(Col = 0; Col < m_nCols; Col++, pCellConst++, pCell++)
		{
			pCellConst->u = 2 * Col * m_CellSize - m_CellSize;
			pCellConst->v = 2 * Row * m_CellSize - m_CellSize;

			pCell->disparity = -1;
			pCell->pLinePtr = NULL;
			pCell->Flags = 0x00;
			//memcpy(&(pCell->LinePtrList), &LinePtrListTemplate, sizeof(CRVLMPtrChain));
		}

	m_csPhiTol = cos(m_PhiTol);

	m_maxvarRotHyp = m_RotHypTol * m_RotHypTol * DEG2RAD * DEG2RAD; 
	m_maxvartHyp = m_tHypTol * m_tHypTol;
	
	m_ScanLineStartArray = new int[m_nRows];
	m_ScanLineEndArray = new int[m_nRows];

	m_ScanLineEndMap = new int[m_nCells];

	m_iSampleCellArray = new int[m_nCells];

	int ImageSize = m_pCamera->Width * m_pCamera->Height;

	m_PtBuff = new CvPoint[6 * ImageSize];

	RVLCrop3DContourCreateLTs(&m_CropLTs, m_pCamera);

	m_AlphaHist = new int[m_AlphaResolution];

#ifdef RVLPSULM_LINES
	m_2DContourMap = new BYTE[4 * ImageSize];
#endif

	// Hypothesis Generator

	//m_HypothesisClustering.m_pMem = m_pMem;
	//m_HypothesisClustering.m_pMem2 = m_pMem2;
	//m_HypothesisClustering.Init();

	m_HypothesisList.m_pMem = m_pMem;

	m_maxnLandmarks = 0;

	// Line sort buffer

	//if(m_Flags & RVLPSULMBUILDER_FLAG_LINES)
	//{
	//	m_LineSortBuff.m_Size = DOUBLE2INT(sqrt((double)(4 * (m_pCamera->Width * m_pCamera->Width + m_pCamera->Height * m_pCamera->Height)))) + 2;

	//	m_LineSortBuff.Init();
	//}

	if(m_Flags & RVLPSULMBUILDER_FLAG_SURFACES)
	{
		// Scene 3D Surface Set

		m_S3DSurfaceSet.m_pMem0 = m_pMem;
		//m_S3DSurfaceSet.m_pMem = m_pMem;
		m_S3DSurfaceSet.m_pMem2 = m_pMem2;

		m_S3DSurfaceSet.AppendData(RVLOBJ2_DATA_PROJECT_PTR);

		//m_S3DSurfaceSet.m_iRelListContours = m_S3DSurfaceSet.m_nRelLists++;
		m_S3DSurfaceSet.m_iRelListComponents = m_S3DSurfaceSet.m_nRelLists++;		

		//RVLCreateC3D(&m_S3DSurfaceSet);
		m_S3DSurfaceSet.Init();

		m_S3DSurfaceSet.m_UncertCoeff = m_M3DSurfaceSet.m_UncertCoeff;

		// Model 3D Surface Set

		m_M3DSurfaceSet.m_pMem0 = m_pMem0;
		m_M3DSurfaceSet.m_pMem = m_pMem;
		m_M3DSurfaceSet.m_pMem2 = m_pMem2;

		//m_M3DSurfaceSet.m_iRelListContours = m_M3DSurfaceSet.m_nRelLists++;
		m_M3DSurfaceSet.m_iRelListComponents = m_M3DSurfaceSet.m_nRelLists++;

		//RVLCreateC3D(&m_M3DSurfaceSet);
		m_M3DSurfaceSet.Init();

		// Scene 3D Convex Segment Set

		m_S3DConvexSegmentSet.m_pMem0 = m_pMem;
		//m_S3DConvexSegmentSet.m_pMem = m_pMem;
		m_S3DConvexSegmentSet.m_pMem2 = m_pMem2;

		m_S3DConvexSegmentSet.m_iDataContourPtr = m_S3DConvexSegmentSet.m_DataSize;
		m_S3DConvexSegmentSet.m_DataSize += sizeof(CRVL3DContour *);

		//RVLCreateC3D(&m_S3DConvexSegmentSet);
		m_S3DConvexSegmentSet.Init();

		// Model 3D Convex Segment Set

		m_M3DConvexSegmentSet.m_pMem0 = m_pMem0;
		m_M3DConvexSegmentSet.m_pMem = m_pMem;
		m_M3DConvexSegmentSet.m_pMem2 = m_pMem2;

		m_M3DConvexSegmentSet.m_iDataContourPtr = m_M3DConvexSegmentSet.m_DataSize;
		m_M3DConvexSegmentSet.m_DataSize += sizeof(CRVL3DContour *);

		//RVLCreateC3D(&m_M3DConvexSegmentSet);
		m_M3DConvexSegmentSet.Init();

		// Scene 3D Contour Set

		m_S3DContourSet.m_pMem0 = m_pMem;
		//m_S3DContourSet.m_pMem = m_pMem;
		m_S3DContourSet.m_pMem2 = m_pMem2;

		m_S3DContourSet.m_iDataProjectPtr = m_S3DContourSet.m_DataSize;
		m_S3DContourSet.m_DataSize += sizeof(CRVL2DContour *);

		//RVLCreateC3D(&m_S3DContourSet);
		m_S3DContourSet.Init();

		// Model 3D Contour Set

		m_M3DContourSet.m_pMem0 = m_pMem0;
		m_M3DContourSet.m_pMem = m_pMem;
		m_M3DContourSet.m_pMem2 = m_pMem2;

		m_M3DContourSet.m_iDataProjectPtr = m_M3DContourSet.m_DataSize;
		m_M3DContourSet.m_DataSize += sizeof(CRVL2DContour *);

		//RVLCreateC3D(&m_M3DContourSet);
		m_M3DContourSet.Init();

		// 2D Contour Set

		m_2DContourSet.m_pMem0 = m_pMem;
		m_2DContourSet.m_pMem = m_pMem;
		m_2DContourSet.m_pMem2 = m_pMem2;

		m_2DContourSet.Init();
	}

	// ROI

	if(m_ROI.left < 0)
	{
		m_ROI.left = 1;
		m_ROI.top = 1;
		m_ROI.right = 2 * (m_pCamera->Width - 1) + 1;		
		m_ROI.bottom = 2 * (m_pCamera->Height - 1) + 1;

		//m_ROI.left = 2 * m_pStereoVision->m_nDisp + 1;
		//m_ROI.right = 2 * (m_pCamera->Width - 1) + 1;
		//m_ROI.top = 1;
		//m_ROI.bottom = 2 * (m_pCamera->Height - 1) + 1;
	}

	//if(m_Flags & RVLPSULMBUILDER_FLAG_LINES)
	//{
	//	// Model 2D Line Set

	//	m_M2DLineSet.m_pMem0 = m_pMem0;
	//	m_M2DLineSet.m_pMem = m_pMem;
	//	m_M2DLineSet.m_pMem2 = m_pMem2;

	//	m_M2DLineSet.Init();

	//	// Scene 2D Line Set

	//	m_S2DLineSet.m_pMem0 = m_pMem;
	//	m_S2DLineSet.m_pMem = m_pMem;
	//	m_S2DLineSet.m_pMem2 = m_pMem2;

	//	m_S2DLineSet.m_iData3DObjectPtr = m_S2DLineSet.m_DataSize;
	//	m_S2DLineSet.m_DataSize += sizeof(CRVL3DLine2 *);

	//	//m_S2DLineSet.m_iDataDisparity = m_S2DLineSet.m_DataSize;
	//	//m_S2DLineSet.m_DataSize += (2 * sizeof(double));

	//	m_S2DLineSet.Init();
	//}

	// Scene 3D Line Set

	m_S3DLineSet.m_pMem0 = m_pMem;
	m_S3DLineSet.m_pMem = m_pMem;
	m_S3DLineSet.m_pMem2 = m_pMem2;

	m_S3DLineSet.m_iDataProjectPtr = m_S3DLineSet.m_DataSize;
	m_S3DLineSet.m_DataSize += sizeof(CRVL2DLine2 *);

	RVLCreateC3DLine(&m_S3DLineSet);

	// Model 3D Line Set

	m_M3DLineSet.m_pMem0 = m_pMem0;
	m_M3DLineSet.m_pMem = m_pMem;
	m_M3DLineSet.m_pMem2 = m_pMem2;

	m_M3DLineSet.m_iDataProjectPtr = m_M3DLineSet.m_DataSize;
	m_M3DLineSet.m_DataSize += sizeof(CRVL2DLine2 *);

	RVLCreateC3DLine(&m_M3DLineSet);

	//Hybrid localization
	m_pPrevCameraPoseSM = (CRVL3DPose *)(m_pMem0->Alloc(sizeof(CRVL3DPose)));
	m_pPrevRobotPoseSM = (CRVL3DPose *)(m_pMem0->Alloc(sizeof(CRVL3DPose)));
	m_pPoseAC = (CRVL3DPose *)(m_pMem0->Alloc(sizeof(CRVL3DPose)));
	m_pModelAbsPose = (CRVL3DPose *)(m_pMem0->Alloc(sizeof(CRVL3DPose)));
	
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 27, m_pPrevCameraPoseSM->m_C);
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 27, m_pPrevRobotPoseSM->m_C);

	m_pPoseSp0 = (CRVL3DPose *)(m_pMem0->Alloc(sizeof(CRVL3DPose)));
	m_pPoseSSp = (CRVL3DPose *)(m_pMem0->Alloc(sizeof(CRVL3DPose)));

	m_pPoseSp0->m_Alpha = m_pPoseSp0->m_Beta = m_pPoseSp0->m_Theta = 0.0;
	m_pPoseSp0->UpdateRotA0();
	RVLNULL3VECTOR(m_pPoseSp0->m_X)


	m_pPoseSSp->m_Alpha = m_pPoseSSp->m_Beta = m_pPoseSSp->m_Theta = 0.0;
	m_pPoseSSp->UpdateRotA0();
	RVLNULL3VECTOR(m_pPoseSSp->m_X)
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 9, m_pPoseSSp->m_C);
	memset(m_pPoseSSp->m_C, 0, 9 * sizeof(double));

	//For hypothesis evaluation	

	if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM)
	{
		m_MatrixSMCost = new double[ImageSize];
		m_MatrixSMCounter = new int[ImageSize];
	}
	else if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
		m_Flags |= RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_PREEVAL;
	else if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
	{
		//m_SurfaceMSArray = new CRVL3DSurface2[m_maxnDominant3DSurfaces];
			 
		m_SurfaceMatchData.Cp = new double[3 * 3 * m_maxnDominant3DSurfacesComplex];
		m_SurfaceMatchData.invCp = new double[3 * 3 * m_maxnDominant3DSurfacesComplex];
		m_SurfaceMatchData.varPositionUncert = m_SampleMatchDistTol * m_SampleMatchDistTol;
		m_SurfaceMatchData.varOrientationUncert = m_SampleMatchAngleTol * m_SampleMatchAngleTol;

		m_LineMatchData.varPositionUncert = m_SurfaceMatchData.varPositionUncert;
		m_LineMatchData.varOrientationUncert = m_SampleMatchAngleTol * m_SampleMatchAngleTol;
	}

	if (m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
	{
		// Projection cube consists of 5 cell arrays: RIGHT, BOTTOM, FRONT, LEFT, TOP.
		// These arrays are allocated in the memory in that order.

		m_nRows2 = m_nCols2 = 2 * m_nCellsPer45deg;
		m_nCells2 = 5 * m_nRows2 * m_nCols2;

		//m_CellIdxMap = new int[(m_nRows2 + 2) * (m_nCols2 + 2) * 5];

		//int iCellIdx = 0;
		//int iCell = 0;

		//int i, j, k;

		//for (k = 0; k < 5; k++)
		//	for (i = 0; i < m_nRows; i++)
		//		for (j = 0; j < m_nCols; j++)
		//			m_CellIdxMap[iCellIdx] = iCell;
	}
	else
	{
		m_nRows2 = m_pCamera->Height / m_CellSize2;
		m_nCols2 = m_pCamera->Width / m_CellSize2;
		m_nCells2 = m_nRows2 * m_nCols2;
	}

	m_CellArray2 = new RVLPSULM_CELL2[m_nCells2];

	RVLPSULM_CELL2 *pCell2 = m_CellArray2;

	m_EmptyCellArray2 = new RVLPSULM_CELL2[m_nCells2];

	RVLPSULM_CELL2 *pEmptyCell2 = m_EmptyCellArray2;

	RVLQLIST *pList, *pList2;

	int iCell;

	for (iCell = 0; iCell < m_nCells2; iCell++, pCell2++, pEmptyCell2++)
	//for(Row = 0; Row < m_nRows2; Row++)
	//	for(Col = 0; Col < m_nCols2; Col++, pCell2++, pEmptyCell2++)
		{
			pEmptyCell2->matchQuality = 1000.0;
			pEmptyCell2->nPts = 0;
			pEmptyCell2->Flags = 0x00;

			pList = &(pCell2->surfaceList);
			pList2 = &(pEmptyCell2->surfaceList);
			RVLQLIST_INIT2(pList2, pList);			
		}

	// Scene Fusion

	if(m_Flags2 & RVLPSULMBUILDER_FLAG2_SCENE_FUSION)
	{
		m_SceneFusion.m_Mem.Create(1000000);

		RVLQLIST *pHypothesisList = &(m_SceneFusion.m_HypothesisList);

		RVLQLIST_INIT(pHypothesisList)

		m_SceneFusion.m_nHypotheses = 0;

		m_SceneFusion.m_HypothesisArray = NULL;
	}

	if(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_MODEL_FUSION)
		m_ModelFusion.m_Mem.Create(1000000);

	// Local Map Hash Table

	m_LocalMapHT.m_Size = RVLPSULMBUILDER_MAX_LOCAL_MAP_HT_SIZE * RVLPSULMBUILDER_MAX_LOCAL_MAP_HT_SIZE + 1;
	
	m_LocalMapHT.m_ListArray = new RVLQLIST[m_LocalMapHT.m_Size];

	RVLQLIST *pListArray = m_LocalMapHT.m_ListArray;

	m_LocalMapHT.InitListArray(pListArray, pListArray);
	
	////Added by Filko 15.05.2012. for test purposes
	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, void*, ((int)pow(8.0, 2)), m_colorMapDB);
	//for(int i = 0; i < (int)pow(8.0, 2); i++)
	//{
	//	if (m_colorMapDB[i])
	//		m_colorMapDB[i] = NULL;
	//}
//#ifdef PYTHON_DEBUG
//	//initialize python
//	Py_Initialize();
//	_import_array();
//	PyRun_SimpleString("import sys\n"
//					   "sys.path.append('D:/Phd/Program/PythonScripts')");
//	m_pyModuleName = PyString_FromString("prikaziGraf"); //SurfacePlotFunctions
//	m_pyModule = PyImport_Import(m_pyModuleName);
//	Py_DECREF(m_pyModuleName);
//	
//#endif
	// Indexing

	m_Indexing.m_vpBuilder = this;

	m_Indexing.Init();

	///

	m_Flags |= RVLPSULMBUILDER_FLAG_KIDNAPPED;
}



void CRVLPSuLMBuilder::RegionSampling(CRVLMPtrChain *pContourList,
									  int nContours)
{
	// get scan lines

	memset(m_ScanLineStartArray, 0x7f, m_nRows * sizeof(int));
	memset(m_ScanLineEndArray, 0xff, m_nRows * sizeof(int));

	memset(m_ScanLineEndMap, 0, m_nCells * sizeof(int));

	int iFirstScanLine = m_nRows + 1;
	int iLineAfterLastScanLine = 0;	

	CvPoint *PtArray;
	int iPt;
	CvPoint *pPt2;
	int du, dv;
	int udv, dudv;
	int u1, u2, v1, v2, vRel;
	int iCol, iRow;
	int iContour;
	int nBoundaryPts;
	CvPoint *pPt1;	
	CRVL2DContour *pContour;

	for(iContour = 0; iContour < nContours; iContour++)
	{
		pContour = (CRVL2DContour *)(pContourList->GetNext());

		PtArray = (CvPoint *)(pContour->m_ContourIPArray);

		pPt1 = PtArray;

		nBoundaryPts = pContour->m_nContourIPs;

		for(iPt = 0; iPt < nBoundaryPts; iPt++)
		{
			pPt2 = PtArray + ((iPt + 1) % nBoundaryPts);

			dv = pPt2->y - pPt1->y;

			if(dv != 0)
			{
				if(dv > 0)
				{
					u1 = pPt1->x + m_CellSize;
					u2 = pPt2->x + m_CellSize;
					v1 = pPt1->y + m_CellSize;
					v2 = pPt2->y + m_CellSize;
				}
				else
				{
					u1 = pPt2->x + m_CellSize;
					u2 = pPt1->x + m_CellSize;
					v1 = pPt2->y + m_CellSize;
					v2 = pPt1->y + m_CellSize;

					dv = -dv;
				}

				du = u2 - u1;

				iRow = v1 / m_nrmCellSize + 1;

				if(iRow < iFirstScanLine)
					iFirstScanLine = iRow;

				vRel = iRow * m_nrmCellSize - v1;

				udv = dv * u1;

				udv += du * vRel;

				dudv = du * m_nrmCellSize;

				while(vRel <= dv)
				{
					iCol = udv / dv / m_nrmCellSize;

					if(iCol < m_ScanLineStartArray[iRow])
						m_ScanLineStartArray[iRow] = iCol;

					if(iCol > m_ScanLineEndArray[iRow])
						m_ScanLineEndArray[iRow] = iCol;

					m_ScanLineEndMap[iCol + iRow * m_nCols]++;

					udv += dudv;
					vRel += m_nrmCellSize;
					iRow++;
				}

				if(iRow > iLineAfterLastScanLine)
					iLineAfterLastScanLine = iRow;
			}

			pPt1 = pPt2;
		}
	}

	// sampling

	int *piSampleCell = m_iSampleCellArray;

	int iCell;
	int n;
	int *pScanLineEnd;

	for(iRow = iFirstScanLine; iRow < iLineAfterLastScanLine; iRow++)
	{
		iCol = m_ScanLineStartArray[iRow];

		iCell = iCol + iRow * m_nCols;

		pScanLineEnd = m_ScanLineEndMap + iCell;

		n = (*(pScanLineEnd++));

		iCol++;

		iCell++;

		for(; iCol <= m_ScanLineEndArray[iRow]; iCol++, pScanLineEnd++, iCell++)
		{
			if(n & 1)
				*(piSampleCell++) = iCell;

			n += (*pScanLineEnd);
		}
	}

	m_nSampleCells = piSampleCell - m_iSampleCellArray;
}

void CRVLPSuLMBuilder::LineSampling(CvPoint * pPt1, 
									CvPoint * pPt2)
{
	int *piSampleCell = m_iSampleCellArray;

	int dp = pPt2->x - pPt1->x;
	int dq = pPt2->y - pPt1->y;

	int du, dv;
	int u1, v1, u2, v2;
	int p1, q1, p2, q2;
	int iRow, iCol, iColPrev, diCol, iCell, diCell;
	int vRel, udv, dudv;
	int k1, k2;

	if(dq * dq >= dp * dp)
	{
		p1 = pPt1->x + m_CellSize;
		p2 = pPt2->x + m_CellSize;
		q1 = pPt1->y + m_CellSize;
		q2 = pPt2->y + m_CellSize;

		k1 = 1;
		k2 = m_nCols;

		du = dp;
		dv = dq;
	}
	else
	{
		p1 = pPt1->y + m_CellSize;
		p2 = pPt2->y + m_CellSize;
		q1 = pPt1->x + m_CellSize;
		q2 = pPt2->x + m_CellSize;

		k1 = m_nCols;
		k2 = 1;

		du = dq;
		dv = dp;
	}

		if(dv > 0)
		{
			u1 = p1;
			u2 = p2;
			v1 = q1;
			v2 = q2;
		}
		else
		{
			u1 = p2;
			u2 = p1;
			v1 = q2;
			v2 = q1;

			du = -du;
			dv = -dv;
		}

		iRow = v1 / m_nrmCellSize;

		iCol = u1 / m_nrmCellSize;

		iCell = k1 * iCol + k2 * iRow;

		*(piSampleCell++) = iCell;

		iCell += k1;

		*(piSampleCell++) = iCell;

		iRow++;

		vRel = iRow * m_nrmCellSize - v1;

		udv = dv * u1;

		udv += du * vRel;

		dudv = du * m_nrmCellSize;

		while(vRel <= dv)
		{
			iColPrev = iCol;

			iCol = udv / dv / m_nrmCellSize;

			diCol = iCol - iColPrev;

			if(diCol * diCol)
			{
				diCell = k1 * ((3 * diCol - 1) >> 1);

				*(piSampleCell++) = iCell + diCell;

				iCell = k1 * iCol + k2 * iRow;	

				*(piSampleCell++) = iCell - diCell;
			}
			else
				iCell = k1 * iCol + k2 * iRow;				

			*(piSampleCell++) = iCell;

			iCell += k1;

			*(piSampleCell++) = iCell;

			udv += dudv;
			vRel += m_nrmCellSize;
			iRow++;
		}

		iColPrev = iCol;

		iCol = u2 / m_nrmCellSize;

		diCol = iCol - iColPrev;

		if(diCol * diCol)
		{
			diCell = k1 * ((3 * diCol - 1) >> 1);

			*(piSampleCell++) = iCell + diCell;

			iCell = k1 * iCol + k2 * iRow;	

			*(piSampleCell++) = iCell - diCell;
		}
		else
			iCell = k1 * iCol + k2 * iRow;				

		*(piSampleCell++) = iCell;

		iCell += k1;

		*(piSampleCell++) = iCell;
	
	m_nSampleCells = piSampleCell - m_iSampleCellArray;
}


void CRVLPSuLMBuilder::DisplayCells(CRVLFigure * pFig)
{
	int *piSampleCellArrayend = m_iSampleCellArray + m_nSampleCells;

	int *piSampleCell;
	int iCell;
	int u, v;

	for(piSampleCell = m_iSampleCellArray; piSampleCell < piSampleCellArrayend; piSampleCell++)
	{
		iCell = *piSampleCell;

		u = iCell % m_nCols * m_CellSize - m_CellSize / 2;
		v = iCell / m_nCols * m_CellSize - m_CellSize / 2;

		cvRectangle(pFig->m_pImage, cvPoint(u, v), cvPoint(u - 1, v - 1), cvScalar(0, 255, 255));
	}
	
}



void CRVLPSuLMBuilder::DisplayCellArray(CRVLFigure *pFig,
										RVLPSULM_CELL *CellArray,
										CRVL3DPose *pPose,
										CvScalar Color)
{
	int iCell;
	int u, v;
	RVLPSULM_CELL *pCell;
	RVLPSULM_CELL_CONST *pCellConst;
	double XS[3], XC[3];
	double U[2];
	int iU[2];

	for(iCell = 0; iCell < m_nCells; iCell++)
	{
		pCell = CellArray + iCell;

		if(pCell->disparity < 0)
			continue;

		pCellConst = m_CellConstArray + iCell;

		m_pStereoVision->Get3DKinect((pCellConst->u >> 1), (pCellConst->v >> 1), (int)(pCell->disparity), XS);

		pPose->InvTransf(XS, XC);

		if(XC[2] < 0.0)
			continue;

		m_pCamera->Project3DPoint(XC, U, iU);

		u = (iU[0] >> 1);
		v = (iU[1] >> 1);

		if(pCell->Flags & RVLPSULM_CELL_FLAG_ERROR)
			cvCircle(pFig->m_pImage, cvPoint(u, v), 2, cvScalar(0, 0, 255), 2);
		else if(pCell->Flags & RVLPSULM_CELL_FLAG_MATCHED)
			cvCircle(pFig->m_pImage, cvPoint(u, v), 2, cvScalar(0, 255, 255), 2);
		else
			cvRectangle(pFig->m_pImage, cvPoint(u, v), cvPoint(u - 1, v - 1), Color);
	}
}

//#define RVLPSULM_CREATE_3DLINES_FROM_PLANES

BOOL CRVLPSuLMBuilder::Create(CRVLPSuLM *pPSuLM, 
							  CRVLMem *pMem,
							  DWORD Flags,
							  CRVL3DPose *pPoseM_M,
							  int iView)	
{
	double StartTime = m_pTimer->GetTime();

	double ExecTime;

	unsigned int DepthFormat;

	DWORD HypEvalMethod = (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD);

	RVLQLIST *pLocalMap = &(pPSuLM->m_LocalMap);
	RVLQLIST_INIT(pLocalMap)

	bool bClose = true;

	if((m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX) && !(Flags & RVLPSULMBUILDER_CREATEMODEL_APPEND_LAST))
		bClose = false;

	double *RM_M;
	
	if(pPoseM_M)
		RM_M = pPoseM_M->m_Rot;

	if(Flags & RVLPSULMBUILDER_CREATEMODEL_FROM_IMAGE)
	{	
		// Create AImage and fill m_Point3DArray 

		if(m_Flags & RVLPSULMBUILDER_FLAG_PC)
		{
			if(Flags & RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE)
			{
				double *PC = new double[3 * m_pPSD->m_Width * m_pPSD->m_Height];

				int nPC;

				if(!RVLPCImport(pPSuLM->m_FileName, &PC, nPC, m_PCScale))
					return FALSE;

				ExecTime = m_pTimer->GetTime() - StartTime;

				StartTime = m_pTimer->GetTime();

				m_pPSD->GetOrgPC(PC, nPC);

				delete[] PC;
			}

			m_pMem->Clear();

			m_pAImage->Create();

			m_pPSD->m_Flags &= ~RVLPSD_MESH_SEGMENT_PLANAR;

			int ImageSize = m_pPSD->m_Width * m_pPSD->m_Height;

			RVL3DPOINT2 **Point3DMap = m_pPSD->m_Point3DMapMem;

			int iFOVExtension;

			for(iFOVExtension = -m_pPSD->m_nFOVExtensions; iFOVExtension <= m_pPSD->m_nFOVExtensions; iFOVExtension++, Point3DMap += ImageSize)
			{
				m_pPSD->m_Point3DMap = Point3DMap;	

				if(iFOVExtension == m_pPSD->m_nFOVExtensions)
					m_pPSD->m_Flags |= RVLPSD_MESH_SEGMENT_PLANAR;

				m_pPSD->Segment(&(m_pAImage->m_C2DRegion),&(m_pAImage->m_C2DRegion2),&(m_pAImage->m_C2DRegion3),m_pMem);
			}
		}
		else
		{
			if(Flags & RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE)
			{
				char *DisparityImageFileName;

				if ((m_Flags2 & RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT) == RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG)
				{
					DisparityImageFileName = m_DataSet.GetDepthFileName(m_ImageFileName);

					IplImage *pDepthImage = cvLoadImage(DisparityImageFileName, CV_LOAD_IMAGE_UNCHANGED);

					RVLUnderSampleHalf<short int>((short int *)(pDepthImage->imageData), pDepthImage->width, pDepthImage->height, m_pStereoVision->m_DisparityMap.Disparity);

					cvReleaseImage(&pDepthImage);

					m_DataSet.TransformDepthMap(&(m_pStereoVision->m_DisparityMap), m_pStereoVision->m_zToDepthLookupTable);
				}
				else
				{
					DisparityImageFileName = RVLCreateFileName(m_ImageFileName, "-LW.bmp", -1, "-D.txt");

					if (!RVLImportDisparityImage(DisparityImageFileName, &(m_pStereoVision->m_DisparityMap),
						DepthFormat, m_pStereoVision->m_zToDepthLookupTable))
					{
						delete[] DisparityImageFileName;

						return FALSE;
					}
				}

				delete[] DisparityImageFileName;
			}

			m_pMem->Clear();
			//m_pAImage->Create(m_pCamera->m_Image.pPix);
			m_pAImage->Create();
			//m_pPSD->GetPointsWithDisparity(&(m_pStereoVision->m_DisparityMap), m_pAImage, m_pMem2, false);
			m_pPSD->GetPointsWithDisparity(&(m_pStereoVision->m_DisparityMap));

			ExecTime = m_pTimer->GetTime() - StartTime;

			StartTime = m_pTimer->GetTime();
			//m_pPSD->Segment(&(m_pAImage->m_C2DRegion),&(m_pAImage->m_C2DRegion2),&(m_pAImage->m_C2DRegion3),&(m_pAImage->m_C2DContour),m_pMem);
			m_pPSD->Segment(&(m_pAImage->m_C2DRegion),&(m_pAImage->m_C2DRegion2),&(m_pAImage->m_C2DRegion3),m_pMem);

			ExecTime = m_pTimer->GetTime() - StartTime;
		}

		m_pMem2->Clear();

		if (m_Flags2 & RVLPSULMBUILDER_FLAG2_SURFACE_BOUNDARY)
			RVLSegmentationEdgesFromLabels(&(m_pAImage->m_C2DRegion));
	}

	ExecTime = m_pTimer->GetTime() - StartTime;

	//Allocate mem for m_PoseAbs 
	if(!(Flags & RVLPSULMBUILDER_CREATEMODEL_APPEND))
		RVLMEM_ALLOC_STRUCT(pMem, CRVL3DPose, pPSuLM->m_PoseAbs);
	
	StartTime = m_pTimer->GetTime();

	m_S3DSurfaceSet.m_pMem0 = m_S3DSurfaceSet.m_pMem = m_S3DSurfaceSet.m_ObjectList.m_pMem = pMem;	
	m_S3DConvexSegmentSet.m_pMem0 = m_S3DConvexSegmentSet.m_pMem = pMem;
	m_S3DContourSet.m_pMem0 = m_S3DContourSet.m_pMem = pMem;

#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
	CRVLC2D *p2DRegionSet1 = &(m_pAImage->m_C2DRegion);		//Lines are created from this region set
#endif

	int iWidth = m_pCamera->Width;
	int iHeight = m_pCamera->Height;
	
	double fu = m_pStereoVision->m_KinectParams.depthFu;
	double fv = m_pStereoVision->m_KinectParams.depthFv;
	double uc = m_pStereoVision->m_KinectParams.depthUc;
	double vc = m_pStereoVision->m_KinectParams.depthVc;
	double k_ = m_pStereoVision->m_KinectParams.k;
	double d0 = m_pStereoVision->m_KinectParams.d0;

	double uvTol2 = m_pPSD->m_RuvTol * m_pPSD->m_RuvTol;
	double uvdTol2 = m_pPSD->m_RuvdTol * m_pPSD->m_RuvdTol;

	CvMemStorage *storage = cvCreateMemStorage(0);

	int i;
	CvPoint pt, pt2;

	 //Previous method of creating surfaces
#ifdef NEVER 
	if(m_Flags & RVLPSULMBUILDER_FLAG_SURFACES)
	{
		CRVLC2D *p2DRegionSet2 = &(m_pAImage->m_C2DRegion2);    //Surfaces are created from this region set

		int nTotalNo2DContours = m_pAImage->m_C2DContour.m_ObjectList.m_nElements;

		//3D planar surfaces and their corresponding contours (3D) are created and added to m_S3DSurfaceSet/m_S3DContourSet

		CRVLMPtrChain *p2DRegionList;

		RVLARRAY *pRelList, *pRelList2;
		CRVL2DContour **pp2DContour,**pp2DContourEnd, *p2DContour;
		CRVL3DContour **pp3DContourList, **pp3DContourStart;

		int nPoints;

		CRVL3DSurface2 *p3DSurface;
		CRVL3DContour *p3DContour;


		//OpenCV 

		CvSeq *pSeqContourApprox;
		CvSeq *pSeqContour = cvCreateSeq((CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE | CV_SEQ_FLAG_CLOSED), sizeof(CvContour), sizeof(CvPoint), storage);  //CV_SEQ_POLYGON

		int i2DPoint[2];


		double *Contour3DPoints = (double *)(m_pMem->Alloc(3 * iWidth * iHeight * sizeof(double)));
		double *p3DContourPoints;


	
		//**************************************************************//
		//1. Create 3D planar surfaces and their corresponding contours //
		//**************************************************************//

		m_S3DContourSet.m_ObjectList.RemoveAll();
		m_S3DSurfaceSet.m_ObjectList.RemoveAll();

		CRVL3DContour **ContourList = (CRVL3DContour **)(m_pMem->Alloc(nTotalNo2DContours * sizeof(CRVL3DContour *))); 
		memset(ContourList, 0x00, nTotalNo2DContours * sizeof(CRVL3DContour *));

		p3DContourPoints = Contour3DPoints;
		pp3DContourList = ContourList;

		//Get segment list
		p2DRegionList = &(p2DRegionSet2->m_ObjectList);

		p2DRegionList->Start();
		while(p2DRegionList->m_pNext)
		{
			p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

			//create corresponding 3D surface
			p3DSurface = (CRVL3DSurface2 *)(RVL3DSurfaceTemplate.Create3(&(m_S3DSurfaceSet)));
			m_pPSD->Get3DPlane(p2DRegion, p3DSurface);

			p3DSurface->m_d *= 16.0;


			//get contours
			pRelList = p2DRegion->m_RelList + p2DRegionSet2->m_iRelList[RVLRELLIST_CONTOURS];   
			pp2DContour = (CRVL2DContour **)pRelList->pFirst;
			pp2DContourEnd = (CRVL2DContour **)pRelList->pEnd;

			pp3DContourStart = pp3DContourList;

			for(; pp2DContour < pp2DContourEnd; pp2DContour++)
			{
				p2DContour = *pp2DContour;

				nPoints = p2DContour->m_nContourIPs;

				//Clear sequence
				cvClearSeq(pSeqContour);

				//Fill CvPointsArray
				for(i=0; i<nPoints; i++)
				{
					pt.x = p2DContour->m_ContourIPArray[i].u;
					pt.y = p2DContour->m_ContourIPArray[i].v;
					cvSeqPush( pSeqContour,&pt);
				}

				
				//Approximate contour
				pSeqContourApprox = cvApproxPoly( pSeqContour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, m_iApproxPolyPrecision, 0 ); 

				//make sure the approximate polygon has at least 3 points
				if(pSeqContourApprox->total > 2)
				{
					//Create 3DContour
					p3DContour = (CRVL3DContour *)(RVL3DContourTemplate.Create3(&(m_S3DContourSet)));
					*(pp3DContourList++) = p3DContour;

					p3DContour->m_nPts = pSeqContourApprox->total;
					p3DContour->m_PtArray = p3DContourPoints;


					//Get 3D coordinate for each 2D contour pt
					for(i=0; i<pSeqContourApprox->total; i++)
					{
						pt =  *(CvPoint *)cvGetSeqElem(pSeqContourApprox,i);
						
						i2DPoint[0] = (pt.x << 1);
						i2DPoint[1] = (pt.y << 1);

						RVLProject2DPointTo3DPlane(i2DPoint,p3DContourPoints,p3DSurface,m_pCamera);

						p3DContourPoints+=3;
					
					}
				}
			}
			
			//updating 3D surface params
			pRelList2 =p3DSurface->m_RelList + p3DSurface->m_pClass->m_iRelListContours;
			
			pRelList2->pFirst = (unsigned char *)pp3DContourStart;
			pRelList2->pEnd = (unsigned char *)pp3DContourList;

				
		}
	}	// if(m_Flags & RVLPSULMBUILDER_FLAG_SURFACES)


#endif

	//Create surfaces
	if(m_Flags & RVLPSULMBUILDER_FLAG_SURFACES)
	{
		//Surfaces are created from this region set (LEVEL3)
		CRVLC2D *p2DRegionSet = &(m_pAImage->m_C2DRegion3);    

		CRVL2DRegion2 *p2DRegion;
		CRVLMPtrChain *p2DRegionList;
		

		//3D planar surfaces and their corresponding contours (3D) are created and added to m_S3DSurfaceSet/m_S3DContourSet
		

		RVLARRAY *pRelList, *pRelListSurface;
		//RVLARRAY *pRelList2;
		CRVL2DContour **pp2DContour, *p2DContour;
		CRVL2DRegion2 **pp2DRegionLevel2, **pp2DRegionLevel2End, *p2DRegionLevel2;
		CRVL3DContour **pp3DContour;
		
		int nPoints;
		
		int i2DPoint[2];

		int nConvexSegments,nTotalVertexPts, nSupportPts;

		CRVL3DSurface2 *p3DSurface, *p3DConvexSurface;
		
		CRVL3DContour *p3DContour;


		// Number of LEVEL2 regions defines the total number of contours since each region has only one contour
		int nTotalNo2DContours = m_pAImage->m_C2DRegion2.m_ObjectList.m_nElements;  

		//Allocate space for vertices (RVL3DPOINT2) of LEVEL2 regions 
		RVL3DPOINT2  **Vertex2DArray = (RVL3DPOINT2  **)(pMem->Alloc(m_pPSD->m_n3DPoints * sizeof(RVL3DPOINT2  *))); 
		memset(Vertex2DArray, 0x00, m_pPSD->m_n3DPoints * sizeof(RVL3DPOINT2 *));
		RVL3DPOINT2  **pp2DVertexArray;
		double *Contour3DPoints;
		double *p3DContourPoints;
		BYTE *pFreeMem;
		CvPoint *Contour2DPtArray;
		CRVLMPtrChain Contour2DMem;
		CRVL2DContour Contour2D;
		CRVL2DContour **Contour2DArray;
		CRVL2DContour **pp2DContourArray;
		RVLIPOINT *PointI2DArray;
		RVLIPOINT *p2DPointIArray;
		CRVL3DSurface2 **ConvexSegment3DArray;
		CRVL3DSurface2 **pp3DConvexSegmentArray;
		CvSeq *pSeqContourApprox;
		CvSeq *pSeqContour;
		double *TransformedVertexArray;

		if (m_pPSD->m_Flags & RVLPSD_MESH_CONVEX)
		{
			//Allocate space for 3D contour points
			Contour3DPoints = (double *)(pMem->Alloc(3 * iWidth * iHeight * sizeof(double)));

			//Allocate space for 2D contour points
			pFreeMem = m_pMem2->m_pFreeMem;
			Contour2DPtArray = (CvPoint *)(m_pMem2->Alloc(iWidth * iHeight * sizeof(CvPoint)));
			Contour2DMem.m_pMem = m_pMem2;
			Contour2DMem.Add(&Contour2D);
			Contour2D.m_ContourIPArray = (RVLIPOINT *)Contour2DPtArray;

			//Allocate space for 3D contours of LEVEL2 regions
			//CRVL3DContour **Contour3DArray = (CRVL3DContour **)(m_pMem->Alloc(nTotalNo2DContours * sizeof(CRVL3DContour *))); 
			//memset(Contour3DArray, 0x00, nTotalNo2DContours * sizeof(CRVL3DContour *));
			//CRVL3DContour **pp3DContourArray, **pp3DContourStart;

			//Allocate space for 2D contours of LEVEL2 regions
			Contour2DArray = (CRVL2DContour **)(pMem->Alloc(nTotalNo2DContours * sizeof(CRVL2DContour *)));
			memset(Contour2DArray, 0x00, nTotalNo2DContours * sizeof(CRVL2DContour *));
			pp2DContourArray; //, **pp2DContourStart;

			//Allocate space for vertices (RVLIPOINT) of LEVEL2 regions 
			PointI2DArray = (RVLIPOINT *)(pMem->Alloc(m_pPSD->m_n3DPoints * sizeof(RVLIPOINT)));
			p2DPointIArray;

			//Allocate space for 3D convex segments of LEVEL2 regions
			ConvexSegment3DArray = (CRVL3DSurface2 **)(pMem->Alloc(nTotalNo2DContours * sizeof(CRVL3DSurface2 *)));
			memset(ConvexSegment3DArray, 0x00, nTotalNo2DContours * sizeof(CRVL3DSurface2 *));
			pp3DConvexSegmentArray; //, **pp3DConvexSegmentArrayStart;

			//allocate temp array to store converted /transformed vertices
			TransformedVertexArray = new double[m_pPSD->m_n3DPoints * 2];
			//double *pTransformedVertexArray;

			//OpenCV 
			pSeqContour = cvCreateSeq((CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE | CV_SEQ_FLAG_CLOSED), sizeof(CvContour), sizeof(CvPoint), storage);  //CV_SEQ_POLYGON
		}

		int n3DContours;

		//**************************************************************//
		//1. Create 3D planar surfaces and their corresponding contours //
		//**************************************************************//

		// create cell array
		RVLPSULM_CELL *CellArray = pPSuLM->m_CellArray = (RVLPSULM_CELL *)(pMem->Alloc(m_nCells * sizeof(RVLPSULM_CELL)));

		memcpy(CellArray, m_EmptyCellArray, m_nCells * sizeof(RVLPSULM_CELL));

		int n3DSurfaces;

		int nClose3DSurfaces;

		//Clear builder

		RVLPTRCHAIN_ELEMENT **ppFirstSurface;

		if(Flags & RVLPSULMBUILDER_CREATEMODEL_APPEND)
		{
			n3DSurfaces = m_S3DSurfaceSet.m_ObjectList.m_nElements;

			nClose3DSurfaces = pPSuLM->m_n3DSurfacesTotal;

			ppFirstSurface = &(m_S3DSurfaceSet.m_ObjectList.m_pLast->pNext);
		}
		else		
		{
			m_S3DSurfaceSet.m_ObjectList.RemoveAll();

			n3DSurfaces = nClose3DSurfaces = 0;

			ppFirstSurface = &(m_S3DSurfaceSet.m_ObjectList.m_pFirst);

			pPSuLM->m_nSurfaceSamples = 0;
		}

		m_S3DConvexSegmentSet.m_ObjectList.RemoveAll();
		m_S3DContourSet.m_ObjectList.RemoveAll();
		//m_pAImage->m_C2DContour.m_ObjectList.RemoveAll();

		p3DContourPoints = Contour3DPoints;
		//pp3DContourArray = Contour3DArray;
		pp2DContourArray = Contour2DArray;
		pp2DVertexArray = Vertex2DArray;
		p2DPointIArray = PointI2DArray;
		pp3DConvexSegmentArray = ConvexSegment3DArray;
		//point to initial position of Transformed Vertex Array

		//double StartTime = m_pTimer->GetTime();

		CvPoint *pPt2;
		int *piSampleCellArrayEnd;
		int *piCell;
		RVLPSULM_CELL *pCell;
		RVLPSULM_CELL_CONST *pCellConst;
		short disparity;
		double a, b, c;
		double *R;
		double weight;
		double *N;
		double sigmaR;
		RVLQLIST *pSamples;
		double *Rot, *t, *r;
		double z1, z2, z3, z4, minz12, minz34, minz;
		//double M3x3Tmp[9];

		//Get segment list
		p2DRegionList = &(p2DRegionSet->m_ObjectList);

		//int hcnt = 0;

		p2DRegionList->Start();
		while(p2DRegionList->m_pNext)
		{
			p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

			//hcnt++;
			//if(hcnt==93)
			//	int hh=0;
			a = p2DRegion->m_a;
			b = p2DRegion->m_b;
			c = p2DRegion->m_c;


			//create corresponding 3D surface
			p3DSurface = (CRVL3DSurface2 *)(RVL3DSurfaceTemplate.Create3(&(m_S3DSurfaceSet)));
			//pSamples = &(p3DSurface->m_Samples);
			//RVLQLIST_INIT(pSamples)
			
			//relate LEVEL3 region with 3D surface and vice versa
			p2DRegion->m_vp3DSurface = p3DSurface;
			p3DSurface->m_vp2DRegion = p2DRegion;

			//Start relation list between 3dSurface and 3dConvexSegment
			pRelListSurface = p3DSurface->m_RelList + p3DSurface->m_pClass->m_iRelListComponents; //m_S3DSurfaceSet.m_iRelListComponents;

			//store start
			pRelListSurface->pFirst = (unsigned char *)pp3DConvexSegmentArray;
			
			m_pPSD->Get3DSurfaceAndContours2(p2DRegion,
											p3DSurface,
											&m_S3DConvexSegmentSet,
											pp3DConvexSegmentArray,
											//&(m_pAImage->m_C2DContour),
											NULL,
											pp2DContourArray,
											p2DPointIArray,
											pp2DVertexArray,
											nConvexSegments,
											nTotalVertexPts,
											TransformedVertexArray,
											nSupportPts);

			p3DSurface->m_nSupport = nSupportPts;

			if(nSupportPts >= m_minnHypSurfPts)
			{
				Rot = p3DSurface->m_Pose.m_Rot;
				t = p3DSurface->m_Pose.m_X;
				r = p3DSurface->m_EigenValues;

				z1 = t[2] + Rot[6] * r[0];
				z2 = t[2] - Rot[6] * r[0];
				z3 = t[2] + Rot[7] * r[1];
				z4 = t[2] - Rot[7] * r[1];

				minz12 = RVLMIN(z1, z2);
				minz34 = RVLMIN(z3, z4);
				minz = RVLMIN(minz12, minz34);

				if (minz <= m_maxZ)
				{
					p3DSurface->m_Flags |= RVL3DSURFACE_FLAG_CLOSE;

					nClose3DSurfaces++;

					// get surface boundary

					if (m_Flags2 & RVLPSULMBUILDER_FLAG2_SURFACE_BOUNDARY)
					{
						RVLSegmentationGetBoundary(p2DRegion, iWidth, &(p3DSurface->m_BoundaryContourList), m_S3DSurfaceSet.m_pMem0);

						m_pPSD->Get3DPlanarSurfaceBoundary(p3DSurface, (m_Flags & RVLPSULMBUILDER_FLAG_PC) != 0);

						p3DSurface->m_Flags |= RVL3DSURFACE_FLAG_BOUNDARY;
					}
				}
			}
			
			if(m_pPSD->m_Flags & RVLPSD_MESH_CONVEX)
			{		// convex segments and cells

#pragma region Representation of surfaces by convex segments
				if (nConvexSegments<=0)
				{
					p3DSurface->m_Flags |= RVLOBJ2_FLAG_REJECTED;
					continue;
				}
					
				pp3DConvexSegmentArray += nConvexSegments;
				pp2DContourArray += nConvexSegments;
				p2DPointIArray += nTotalVertexPts;
				pp2DVertexArray += nTotalVertexPts;
				
				//if (nSupportPts==34)
				//	int gg = 9;

				//store end
				pRelListSurface->pEnd = (unsigned char *)pp3DConvexSegmentArray;

				

				//reset counter
				n3DContours = 0;

				//get contours
				pRelList = p2DRegion->m_RelList + p2DRegionSet->m_iRelList[RVLRELLIST_COMPONENTS];   
				pp2DRegionLevel2 = (CRVL2DRegion2 **)pRelList->pFirst;
				pp2DRegionLevel2End = (CRVL2DRegion2 **)pRelList->pEnd;

				//pp3DContourStart = pp3DContourArray;

				for(; pp2DRegionLevel2 < pp2DRegionLevel2End; pp2DRegionLevel2++)
				{
					p2DRegionLevel2 = *pp2DRegionLevel2;

					p3DConvexSurface = (CRVL3DSurface2 *)(p2DRegionLevel2->m_vp3DSurface);

					if(p3DConvexSurface == NULL)
						continue;

					pp2DContour = (CRVL2DContour **)(p3DConvexSurface->m_pData + m_S3DConvexSegmentSet.m_iDataContourPtr);
			
					p2DContour = *pp2DContour;

					nPoints = p2DContour->m_nContourIPs;

					//Clear sequence
					cvClearSeq(pSeqContour);

					//Fill CvPointsArray
					for(i=0; i<nPoints; i++)
					{
						pt.x = p2DContour->m_ContourIPArray[i].u;
						pt.y = p2DContour->m_ContourIPArray[i].v;
						cvSeqPush( pSeqContour,&pt);
					}

					
					//Approximate contour
					pSeqContourApprox = cvApproxPoly( pSeqContour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, m_iApproxPolyPrecision, 0 ); 

					//make sure the approximate polygon has at least 3 points
					if(pSeqContourApprox->total > 2)
					{
						//Create 3DContour
						p3DContour = (CRVL3DContour *)(RVL3DContourTemplate.Create3(&(m_S3DContourSet)));
						//*(pp3DContourArray++) = p3DContour;

						p3DContour->m_nPts = pSeqContourApprox->total;
						p3DContour->m_PtArray = p3DContourPoints;

						pPt2 = Contour2DPtArray;					

						//Get 3D coordinate for each 2D contour pt
						for(i=0; i<pSeqContourApprox->total; i++)
						{
							pt =  *(CvPoint *)cvGetSeqElem(pSeqContourApprox,i);
							
							i2DPoint[0] = pt.x;
							i2DPoint[1] = pt.y;

							RVLProject2DPointTo3DPlane(i2DPoint,p3DContourPoints,p3DSurface,
													   m_pStereoVision->m_KinectParams.depthUc,
													   m_pStereoVision->m_KinectParams.depthVc,
													   m_pStereoVision->m_KinectParams.depthFu,
													   m_pStereoVision->m_KinectParams.depthFv);
							

							p3DContourPoints+=3;

							pPt2->x = (pt.x << 1) + 1;
							pPt2->y = (pt.y << 1) + 1;
							pPt2++;
						}

						pp3DContour = (CRVL3DContour **)pp2DContour;

						*pp3DContour = p3DContour;

						pp2DContour = (CRVL2DContour **)(p3DContour->m_pData + m_S3DContourSet.m_iDataProjectPtr);

						*pp2DContour = p2DContour;

						n3DContours++;

						Contour2D.m_nContourIPs = pSeqContourApprox->total;

						Contour2DMem.Start();

						RegionSampling(&Contour2DMem);

						piSampleCellArrayEnd = m_iSampleCellArray + m_nSampleCells;

						for(piCell = m_iSampleCellArray; piCell < piSampleCellArrayEnd; piCell++)
						{
							//if(piCell - m_iSampleCellArray == 1 * m_nCols + 17)
							//	int debug = 0;

							pCell = CellArray + (*piCell);

							pCellConst = m_CellConstArray + (*piCell);

							disparity = (short)(0.5 * (a * (double)(pCellConst->u) + b * (double)(pCellConst->v) + 2.0 * c));

							if(disparity > pCell->disparity)
								pCell->disparity = disparity;
						}
					}
					else
					{
						p3DConvexSurface->m_Flags |= RVLOBJ2_FLAG_REJECTED;
						*pp2DContour = NULL;
					}
				}

				if(n3DContours==0) //mark LEVEL3 surface as rejected
					p3DSurface->m_Flags |= RVLOBJ2_FLAG_REJECTED;
				else
					n3DSurfaces++;
#pragma endregion
			}	// if(m_pPSD->m_Flags & RVLPSD_MESH_CONVEX)
			else
				n3DSurfaces++;				
		}	// for all regions in p2DRegionList

		CRVLMPtrChain *p3DSurfaceList = &(m_S3DSurfaceSet.m_ObjectList);

		// sample 3D surfaces

		if (m_Flags2 & RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING)
			pPSuLM->Get3DSurfaceSamplesFrom2DRegionSamples(m_pPSD->Sample2DRegions(), &m_S3DSurfaceSet, ppFirstSurface);

		//// debug

		//CRVL3DSurface2 **SurfaceArray_ = new CRVL3DSurface2 *[p3DSurfaceList->m_nElements];

		//int i = 0;
		//int debug = 0;

		//p3DSurfaceList->Start();

		//while (p3DSurfaceList->m_pNext)
		//{
		//	p3DSurface = (CRVL3DSurface2 *)(p3DSurfaceList->GetNext());

		//	if (p3DSurface->m_nSupport == 0)
		//		continue;

		//	SurfaceArray_[i++] = p3DSurface;

		//	RVL3DSURFACE_SAMPLE *pSample = (RVL3DSURFACE_SAMPLE *)(p3DSurface->m_Samples.pFirst);

		//	while (pSample)
		//	{
		//		pSample->Index = (debug++);

		//		pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		//	}
		//}

		//delete[] SurfaceArray_;

		/////

		// transform surfaces to the common model reference frame

		if (m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
		{
			p3DSurfaceList->m_pNext = *ppFirstSurface;

			while (p3DSurfaceList->m_pNext)
			{
				p3DSurface = (CRVL3DSurface2 *)(p3DSurfaceList->GetNext());

				p3DSurface->Transf(pPoseM_M, iView);
			}
		}

		/////
		
		if (m_pPSD->m_Flags & RVLPSD_MESH_CONVEX)
			m_pMem2->m_pFreeMem = pFreeMem;

		CRVL3DSurface2 **SurfaceArray;

		double InfMx[3 * 3];
		CRVL3DSurface2 *p3DConvexSegment;

		if(bClose)
		{
			// Compute information matrix

			RVLNULLMX3X3(InfMx)

			p3DSurfaceList->Start();

			while(p3DSurfaceList->m_pNext)
			{
				p3DSurface = (CRVL3DSurface2 *)(p3DSurfaceList->GetNext());

				if(p3DSurface->m_Flags & RVL3DSURFACE_FLAG_CLOSE)
				{
					N = p3DSurface->m_N;

					//sigmaR = sqrt((*((CRVL3DSurface2 **)(pRelListSurface->pFirst)))->m_sigmaR);

					//weight = ((double)nSupportPts) / sqrt(p3DSurface->m_sigmaR);
					weight = ((double)(p3DSurface->m_nSupport));

					InfMx[3*0+0] += (N[0] * N[0] * weight);
					InfMx[3*0+1] += (N[0] * N[1] * weight);
					InfMx[3*0+2] += (N[0] * N[2] * weight);
					InfMx[3*1+1] += (N[1] * N[1] * weight);
					InfMx[3*1+2] += (N[1] * N[2] * weight);
					InfMx[3*2+2] += (N[2] * N[2] * weight);
				}
			}		
		
			//STORE TO PSULM

			pPSuLM->m_SurfaceList.m_pMem = pMem;
			
			//Create PSULM sorted Array
			//pPSuLM->m_n3DSurfacesTotal = n3DSurfaces;	
			pPSuLM->m_3DSurfaceArray = (CRVL3DSurface2 **)(pMem->Alloc(n3DSurfaces * sizeof(CRVL3DSurface2 *)));
			SurfaceArray = pPSuLM->m_3DSurfaceArray;

			if(m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
				MergeSurfaces(pPSuLM);

			//p3DSurfaceList->Start();
			//while (p3DSurfaceList->m_pNext)
			//{
			//	p3DSurface = (CRVL3DSurface2 *)(p3DSurfaceList->GetNext());

			//	//check number of good LEVEL3 surfaces
			//	if (p3DSurface->m_Flags & RVLOBJ2_FLAG_REJECTED)
			//		continue;

			//	RVL3DSURFACE_SAMPLE *pSample = (RVL3DSURFACE_SAMPLE *)(p3DSurface->m_Samples.pFirst);

			//	while (pSample)
			//	{
			//		if ((void *)pSample >= (void *)(pPSuLM->m_3DSurfaceArray))
			//			int debug = 0;

			//		pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
			//	}
			//}

			//go through all level3 surfaces and sort into array		

			int j,iNewPosition;
			BOOL bInsert = FALSE;


			int iSurface = 0;

			//CString debugFile = "D:\\Phd\\Program\\PythonScripts\\DebugFile.dat";
			//FILE *fDebugFile = fopen(debugFile, "w");
			//fprintf(fDebugFile,"Index\tSupport\tN[0]\tN[1]\tN[2]\trho\n");
			
			p3DSurfaceList->Start();
			while(p3DSurfaceList->m_pNext)
			{
				p3DSurface = (CRVL3DSurface2 *)(p3DSurfaceList->GetNext());

				//check number of good LEVEL3 surfaces
				if(p3DSurface->m_Flags & RVLOBJ2_FLAG_REJECTED)
					continue;

				
				//if(p3DSurface->m_nSupport == 34)
				//	int gg = 90;

				//p3DSurface->GetPoseContribution();

				//Get first convex segment (the next 5 lines were uncommented in the version with convex segments,
				//but I don't know if they are doing anything useful, since the same computation is already 
				//performed in Get3DSurfaceAndContours2()

				//RVLARRAY *pRelList = p3DSurface->m_RelList + p3DSurface->m_pClass->m_iRelListComponents;
				//p3DConvexSegment = *((CRVL3DSurface2 **)pRelList->pFirst);

				//p3DConvexSegment->m_varq[0] =  p3DConvexSegment->m_sigmaR / (p3DConvexSegment->m_EigenValues[0] * p3DConvexSegment->m_EigenValues[0] + p3DConvexSegment->m_sigmaR);
				//p3DConvexSegment->m_varq[1] =  p3DConvexSegment->m_sigmaR / (p3DConvexSegment->m_EigenValues[1] * p3DConvexSegment->m_EigenValues[1] + p3DConvexSegment->m_sigmaR);
				//p3DConvexSegment->m_varq[2] =  p3DConvexSegment->m_sigmaR;

				/////		

				//p3DConvexSegment->m_N[0] = p3DConvexSegment->m_Pose.m_Rot[2];
				//p3DConvexSegment->m_N[1] = p3DConvexSegment->m_Pose.m_Rot[5];
				//p3DConvexSegment->m_N[2] = p3DConvexSegment->m_Pose.m_Rot[8];
				//p3DConvexSegment->m_d = RVLDOTPRODUCT3(p3DConvexSegment->m_Pose.m_X, p3DConvexSegment->m_N);

				// compute surface significance

				if(p3DSurface->m_Flags & RVL3DSURFACE_FLAG_CLOSE)
				{
					N = p3DSurface->m_N;
		
					p3DSurface->m_PoseInformation = (double)p3DSurface->m_nSupport / RVLCOV3DTRANSFTO1D(InfMx, N) * 1000.0;
					//p3DSurface->m_PoseInformation = (double)p3DSurface->m_nSupport / RVLCOV3DTRANSFTO1D(InfMx, N) * 1000.0 / sqrt(p3DConvexSegment->m_sigmaR);
					//p3DSurface->m_PoseInformation = (double)p3DSurface->m_nSupport / RVLCOV3DTRANSFTO1D(InfMx, N) * 1000.0 / sqrt(p3DSurface->m_sigmaR);
				}
				else
				{
					p3DSurface->m_PoseInformation = 0.0;

					p3DSurface->m_nSupport = 0;
				}
				
				//add to PSULM
				pPSuLM->m_SurfaceList.Add(p3DSurface);

				//sort and add to SurfaceArray
				if(iSurface == 0)
				{
					//store new value
					SurfaceArray[0] = p3DSurface;
					SurfaceArray[0]->m_Index = 0;
				}
				else
				{
					bInsert = false;
					for(i=0;i<iSurface;i++)
					{
						if(p3DSurface->m_PoseInformation > SurfaceArray[i]->m_PoseInformation)
						{
							iNewPosition = i;
							//Shift all values down first
							for(j=iSurface;j>i;j--)
							{
								SurfaceArray[j] = SurfaceArray[j-1];
								SurfaceArray[j]->m_Index = j;
							}
							bInsert = true;

							break;
						}
						
					}

					if(bInsert==false)
						iNewPosition = iSurface;

					//store new value
					SurfaceArray[iNewPosition] = p3DSurface;
					SurfaceArray[iNewPosition]->m_Index = iNewPosition;

					//if (iNewPosition == 15 || iNewPosition == 57)
					//	int debug = 0;
				}

				//fprintf(fDebugFile,"%4d\t%6d\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\n",
				//	p3DSurface->m_Index,p3DSurface->m_nSupport,
				//	p3DSurface->m_N[0],p3DSurface->m_N[1],p3DSurface->m_N[2],
				//	p3DSurface->m_d);

				iSurface++;
			}

			CRVL3DSurface2 **p3DSurfaceArrayEnd = SurfaceArray + n3DSurfaces;

			CRVL3DSurface2 **pp3DSurface;

			for(pp3DSurface = SurfaceArray; pp3DSurface < p3DSurfaceArrayEnd; pp3DSurface++)
			{
				p3DSurface = *pp3DSurface;		

				if(p3DSurface->m_nSupport == 0)
					break;
			}

			pPSuLM->m_n3DSurfacesTotal = pp3DSurface - SurfaceArray;
			int maxnDominant3DSurfaces = (pPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX ? m_maxnDominant3DSurfacesComplex : 
				m_maxnDominant3DSurfaces);
			pPSuLM->m_n3DSurfaces = (pPSuLM->m_n3DSurfacesTotal >= maxnDominant3DSurfaces ? maxnDominant3DSurfaces : pPSuLM->m_n3DSurfacesTotal);
		}	// if(bClose)

		//fclose(fDebugFile);

		// Set RVLOBJ2_FLAG_DOMINANT flag of the 2D regions corresponding to the 
		// first pPSuLM->m_n3DSurfaces surfaces in SurfaceArray

		////Commented out 21.06.2012 Karlo (Does not seem to be doing anything worthwhile
		//for(i=0;i<pPSuLM->m_n3DSurfaces;i++)
		//{
		//	p3DSurface = SurfaceArray[i];

		//	p2DRegion = (CRVL2DRegion2 *)(p3DSurface->m_vp2DRegion);

		//	//p2DRegion->m_Flags |= RVLOBJ2_FLAG_DOMINANT;
		//}

		if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM)
		{
			//FILL m_pPSD->m_3DSurfaceMap
			int ImageSize = m_pPSD->m_Width * m_pPSD->m_Height;

			//Reset m_pPSD->m_3DSurfaceMap
			memset(m_pPSD->m_3DSurfaceMap, 0, ImageSize * sizeof(CRVL3DSurface2 *));

			CRVL3DSurface2 **p3DSurfacePtr = m_pPSD->m_3DSurfaceMap;

			CRVL2DRegion2 **p2DRegionMapEnd = m_pPSD->m_2DRegionMap + ImageSize;

			CRVL2DRegion2 **p2DRegionPtr;
			
			for(p2DRegionPtr = m_pPSD->m_2DRegionMap; p2DRegionPtr < p2DRegionMapEnd; p2DRegionPtr++, p3DSurfacePtr++)
			{
				p2DRegion = *p2DRegionPtr;

				if(p2DRegion == NULL)
					continue;

				if(p2DRegion->m_Flags & RVLOBJ2_FLAG_REJECTED)
				{
					*p3DSurfacePtr = NULL;

					continue;
				}

				*p3DSurfacePtr = (CRVL3DSurface2 *)(p2DRegion->m_vp3DSurface);		
			}
		}

		//Create lines
		if(m_Flags & RVLPSULMBUILDER_FLAG_LINES)
		{
			// detect depth discontinuity contours

			double StartTimeLines = m_pTimer->GetTime();

			int w = m_pPSD->m_Width;
			int h = m_pPSD->m_Height;

			RVLDetectDepthDiscontinuityContours(m_pStereoVision->m_DisparityMap.Disparity, w, h, 
				m_pStereoVision->m_DisparityMap.Format, m_minLineDepthStep, m_minContourSize, m_pMem, m_pAImage, 
				&m_2DContourList, m_2DContourMap);
		
			// detect dominant edge lines 

			short *Depth = m_pStereoVision->m_DisparityMap.Disparity;

			m_S3DLineSet.m_pMem0 = m_S3DLineSet.m_pMem = m_S3DLineSet.m_ObjectList.m_pMem = pMem;
			pPSuLM->m_3DLineList.m_pMem = pMem;

			if(!(Flags & RVLPSULMBUILDER_CREATEMODEL_APPEND))
				m_S3DLineSet.m_ObjectList.RemoveAll();

			RVL2DCONTOUR_DATA2 *pContour = (RVL2DCONTOUR_DATA2 *)(m_2DContourList.pFirst);

			RVLQLIST VertexList;

			int iPix;
			int iNeighbor, iNextNeighbor;
			int nVertices;
			RVLQLIST_PTR_ENTRY *pVertex1, *pVertex2;
			BYTE *pContourElement, *pContourElement1, *pContourElement2;
			CRVL3DLine2 *p3DLine;
			int U1[3], U2[3], du, dv;
			int nDepthSteps;
			RVL3DLINE_EXTENDED_DATA *p3DLineData;
			double fTmp;
			double *dX;
			double *XM, *CM;
			double XM_[3], CM_[9], M3x3Tmp[9];

			while(pContour)	// for each contour
			{
				RVLDepthContourSegmentToLines(pContour, m_2DContourMap, m_pPSD->m_Point3DMap, 3.0, m_pMem, m_pAImage, &VertexList, nVertices);

				pVertex1 = (RVLQLIST_PTR_ENTRY *)(VertexList.pFirst);

				pVertex2 = (RVLQLIST_PTR_ENTRY *)(pVertex1->pNext);

				while(pVertex2)
				{
					nDepthSteps = 0;

					pContourElement1 = pContourElement = (BYTE *)(pVertex1->Ptr);

					pContourElement2 = (BYTE *)(pVertex2->Ptr);

					iNeighbor = RVL2DCONTOUR_GET_EDGEID(pContourElement, m_2DContourMap);

					while(TRUE)
					{
						if (m_Flags2 & RVLPSULMBUILDER_FLAG2_LINES_EDGES_VOID)
							nDepthSteps++;
						else if(((*pContourElement) & RVL2DCONTOUR_VOID) == 0)
							nDepthSteps++;

						if(pContourElement == pContourElement2)
							break;

						if((*pContourElement) & RVL2DCONTOUR_NEXT)
						{
							iNextNeighbor = RVL2DCONTOUR_GET_NEXT_EDGEID(pContourElement);
							pContourElement = RVL2DCONTOUR_GET_NEXT(pContourElement, m_pAImage, iNeighbor, iNextNeighbor);
							iNeighbor = iNextNeighbor;
						}		
						else
							break;
					}

					if(nDepthSteps >= m_minnLineDepthSteps)
					{
						// create a line

						p3DLine = (CRVL3DLine2 *)RVL3DLine2Template.Create3(&(m_S3DLineSet));

						//p3DLine->m_Index = pPSuLM->m_3DLineList.m_nElements;

						iPix = RVL2DCONTOUR_GET_IPIX(pContourElement1, m_2DContourMap);

						U1[0] = iPix % w;
						U1[1] = iPix / w;
						U1[2] = Depth[iPix];

						if(m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
						{
							XM = XM_;
							CM = CM_;
						}
						else
						{
							XM = p3DLine->m_X[0];
							CM = p3DLine->m_CX[0];
						}

						RVLGetKinect3DData(U1, XM, m_pStereoVision->m_KinectParams);

						m_pCamera->KinectReconWithUncert((double)(U1[0]), (double)(U1[1]), (double)(U1[2]), d0, k_, uc, vc, fu, fv, 
							uvTol2, uvdTol2, CM);	
	
						if(m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
						{
							XM = p3DLine->m_X[0];
							CM = p3DLine->m_CX[0];

							RVLMULMX3X3VECT(RM_M, XM_, XM)
							RVLCOV3DTRANSF(CM_, RM_M, CM, M3x3Tmp)
						}

						iPix = RVL2DCONTOUR_GET_IPIX(pContourElement2, m_2DContourMap);

						U2[0] = iPix % w;
						U2[1] = iPix / w;
						U2[2] = Depth[iPix];

						if(m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
						{
							XM = XM_;
							CM = CM_;
						}
						else
						{
							XM = p3DLine->m_X[1];
							CM = p3DLine->m_CX[1];
						}

						RVLGetKinect3DData(U2, XM, m_pStereoVision->m_KinectParams);

						m_pCamera->KinectReconWithUncert((double)(U2[0]), (double)(U2[1]), (double)(U2[2]), d0, k_, uc, vc, fu, fv, 
							uvTol2, uvdTol2, CM);	

						if(m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
						{
							XM = p3DLine->m_X[1];
							CM = p3DLine->m_CX[1];

							RVLMULMX3X3VECT(RM_M, XM_, XM)
							RVLCOV3DTRANSF(CM_, RM_M, CM, M3x3Tmp)
						}

						RVLMEM_ALLOC_STRUCT(pMem, RVL3DLINE_EXTENDED_DATA, p3DLineData)

						p3DLine->m_pData = (BYTE *)p3DLineData;

						dX = p3DLineData->dX;

						RVLDIF3VECTORS(p3DLine->m_X[1], p3DLine->m_X[0], dX)

						fTmp = sqrt(RVLDOTPRODUCT3(dX, dX));
						RVLSCALE3VECTOR2(dX, fTmp, p3DLineData->V)
						p3DLineData->len = fTmp;

						du = U2[0] - U1[0];
						dv = U2[1] - U1[1];

						p3DLine->m_nSupport = DOUBLE2INT(sqrt((double)(du * du + dv * dv)));

						pPSuLM->m_3DLineList.Add(p3DLine);
					}
					
					pVertex1 = pVertex2;

					pVertex2 = (RVLQLIST_PTR_ENTRY *)(pVertex2->pNext);
				}
				
				pContour = (RVL2DCONTOUR_DATA2 *)(pContour->pNext);
			}	// for each contour

			if(bClose)
			{
				// create 3DLineArray

				pPSuLM->m_n3DLinesTotal = pPSuLM->m_3DLineList.m_nElements;

				RVLMEM_ALLOC_STRUCT_ARRAY(pMem, CRVL3DLine2 *, pPSuLM->m_n3DLinesTotal, pPSuLM->m_3DLineArray)

				CRVL3DLine2 **p3DLineArrayEnd = pPSuLM->m_3DLineArray + pPSuLM->m_n3DLinesTotal;

				if(m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
					MergeLines(pPSuLM);

				pPSuLM->m_n3DLinesTotal = 0;

				pPSuLM->m_3DLineList.Start();

				while(pPSuLM->m_3DLineList.m_pNext)
				{
					p3DLine = (CRVL3DLine2 *)(pPSuLM->m_3DLineList.GetNext());

					p3DLine->cost = p3DLine->m_nSupport;

					if(p3DLine->m_nSupport > 0)
						pPSuLM->m_n3DLinesTotal++;
				}

				RVLBubbleSort<CRVL3DLine2>(&(pPSuLM->m_3DLineList), pPSuLM->m_3DLineArray, TRUE);

				int maxnDominant3DLines = (pPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX ? m_maxnDominant3DLinesComplex : 
					m_maxnDominant3DLines);
				pPSuLM->m_n3DLines = (pPSuLM->m_n3DLinesTotal > maxnDominant3DLines ? maxnDominant3DLines : pPSuLM->m_n3DLinesTotal);

				p3DLineArrayEnd = pPSuLM->m_3DLineArray + pPSuLM->m_n3DLinesTotal;

				CRVL3DLine2 **pp3DLine;

				for(pp3DLine = pPSuLM->m_3DLineArray; pp3DLine < p3DLineArrayEnd; pp3DLine++)
					//*pp3DLine = (CRVL3DLine2 *)(pPSuLM->m_3DLineList.GetNext());
					(*pp3DLine)->m_Index = pp3DLine - pPSuLM->m_3DLineArray;

				double LinesCreateTime = m_pTimer->GetTime() - StartTimeLines;
			}

#ifdef NEVER
			/////

			short *Depth = m_pStereoVision->m_DisparityMap.Disparity;
			int *dpNeighbor4 = m_pPSD->m_dpNeighbor4;

			int i;
			RVLQLIST *pBoundaryContourList;
			RVL2DCONTOUR_DATA *pContour_;
			BYTE Edge, PrevEdge;
			BYTE *pEdge, *pContourDataEnd;
			int diPix, iPix1, iPix2;
			CvPoint Pt, Pt0, PrevPt;
			CvSeqReader reader;
			RVLRECT BoundingBox;
			BOOL bGoingToStart;
			int nDepthSteps;
			short d2;
			CRVL3DLine2 *p3DLine;
			int U[3];

			p2DRegionList->Start();

			while(p2DRegionList->m_pNext)
			{
				p2DRegion = (CRVL2DRegion2 *)(p2DRegionList->GetNext());

				if((p2DRegion->m_Flags & RVL2DREGION_FLAG_BOUNDARY) == 0)
					continue;

				pBoundaryContourList = &(p2DRegion->m_BoundaryContourList);

				pContour_ = (RVL2DCONTOUR_DATA *)(pBoundaryContourList->pFirst);

				while(pContour_)
				{
					// get pixels along region boundary contour pContour and store them in pSeqContour
					// get the bounding box of the contour

					iPix = pContour_->iPix0;

					pContourDataEnd = pContour_->Data + pContour_->n;

					Pt0.x = iPix % w;
					Pt0.y = iPix / w;
					cvSeqPush(pSeqContour, &Pt0);

					BoundingBox.left = BoundingBox.right = Pt0.x;
					BoundingBox.bottom = BoundingBox.top = Pt0.y;

					pEdge = pContour_->Data;

					PrevEdge = *pEdge;

					pEdge++;

					for(; pEdge < pContourDataEnd; pEdge++)
					{
						//if(iPix == 205+61*w)
						//	int debug = 0;

						Edge = *pEdge;

						diPix = m_pPSD->m_diPixContourFollow[PrevEdge][Edge];

						iPix += diPix;

						if(diPix != 0)
						{
							Pt.x = iPix % w;
							Pt.y = iPix / w;
							cvSeqPush(pSeqContour, &Pt);

							if(Pt.x < BoundingBox.left)
								BoundingBox.left = Pt.x;
							else if(Pt.x > BoundingBox.right)
								BoundingBox.right = Pt.x;

							if(Pt.y < BoundingBox.top)
								BoundingBox.top = Pt.y;
							else if(Pt.y > BoundingBox.bottom)
								BoundingBox.bottom = Pt.y;
						}

						PrevEdge = Edge;
					}

					//if(BoundingBox.right - BoundingBox.left >= m_minLineBoundingBoxSize ||
					//	BoundingBox.bottom - BoundingBox.top >= m_minLineBoundingBoxSize)
					{
						if(iPix != pContour_->iPix0)
							cvSeqPush(pSeqContour, &Pt0);

						// approximate the contour by a polygon

						pSeqContourApprox = cvApproxPoly(pSeqContour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, m_iApproxPolyPrecision, 0);

						// detect dominant surface edge lines and assign them to p3DSurface	

						iPix = pContour_->iPix0;

						pEdge = pContour_->Data;

						Edge = *pEdge;

						bGoingToStart = TRUE;

						cvStartReadSeq(pSeqContourApprox, &reader, 0);

						for(i = 0; i <= pSeqContourApprox->total; i++)
						{
							CV_READ_SEQ_ELEM(Pt, reader);
		
							iPix2 = Pt.x + Pt.y * w;

							nDepthSteps = 0;

							while(iPix != iPix2)
							{
								//if(iPix == 205+61*w)
								//	int debug = 0;

								if(!bGoingToStart)
								{
									d2 = Depth[iPix + dpNeighbor4[Edge]];

									if(d2 < 2047)
										if(d2 - Depth[iPix] >= m_minLineDepthStep)
											nDepthSteps++;
								}

								PrevEdge = Edge;

								pEdge++;

								if(pEdge == pContourDataEnd)
									pEdge = pContour_->Data;

								Edge = *pEdge;

								diPix = m_pPSD->m_diPixContourFollow[PrevEdge][Edge];

								iPix += diPix;
							}

							if(bGoingToStart)
								bGoingToStart = FALSE;
							else if(nDepthSteps >= m_minnLineDepthSteps)
							{
								// create a line

								p3DLine = (CRVL3DLine2 *)RVL3DLine2Template.Create3(&(m_S3DLineSet));

								U[0] = PrevPt.x;
								U[1] = PrevPt.y;
								U[2] = Depth[iPix1];

								RVLGetKinect3DData(U, p3DLine->m_X[0], m_pStereoVision->m_KinectParams);

								m_pCamera->KinectReconWithUncert((double)(U[0]), (double)(U[1]), (double)(U[2]), d0, k_, uc, vc, fu, fv, 
									uvTol2, uvdTol2, p3DLine->m_CX[0]);	

								U[0] = Pt.x;
								U[1] = Pt.y;
								U[2] = Depth[iPix2];

								RVLGetKinect3DData(U, p3DLine->m_X[1], m_pStereoVision->m_KinectParams);

								m_pCamera->KinectReconWithUncert((double)(U[0]), (double)(U[1]), (double)(U[2]), d0, k_, uc, vc, fu, fv, 
									uvTol2, uvdTol2, p3DLine->m_CX[1]);	

								pPSuLM->m_3DLineList.Add(p3DLine);
							}

							iPix1 = iPix2;

							PrevPt = Pt;
						}	// for each polygon line
					}	// if the bounding box is sufficiently large

					cvClearSeq(pSeqContour);

					pContour_ = (RVL2DCONTOUR_DATA *)(pContour_->pNext);
				}	// for each boundary contour

				p3DSurface = (CRVL3DSurface2 *)(p2DRegion->m_vp3DSurface);

			}	// for each region
#endif
		}	// if(m_Flags & RVLPSULMBUILDER_FLAG_LINES)
		else
			pPSuLM->m_n3DLines = pPSuLM->m_n3DLinesTotal = 0;

#ifdef RVLPSULM_CREATE_DEBUG_LOG
		FILE *fpLog;

		char *LogFileName = RVLCreateFileName(m_ImageFileName, "-", -1, "-S.dat");

		fopen_s(&fpLog, LogFileName, "w");			

		for(i=0;i<iSurface;i++)
		{
			p3DSurface = SurfaceArray[i];
			
			pRelList = p3DSurface->m_RelList + p3DSurface->m_pClass->m_iRelListComponents;

			fprintf(fpLog, "SURF%d: N=(%lf, %lf, %lf), rho=%lf\n", i, 
				p3DSurface->m_N[0], p3DSurface->m_N[1], p3DSurface->m_N[2], p3DSurface->m_d);

			CRVL3DSurface2 **pp3DConvexSegment = (CRVL3DSurface2 **)pRelList->pFirst;

			int iSeg = 0;

			for(pp3DConvexSegment = (CRVL3DSurface2 **)(pRelList->pFirst);
				pp3DConvexSegment < (CRVL3DSurface2 **)(pRelList->pEnd); 
				pp3DConvexSegment++)
			{
				p3DConvexSegment = *pp3DConvexSegment;

				fprintf(fpLog, "SEG%d: C=(%lf, %lf, %lf)\n", iSeg, 
					p3DConvexSegment->m_Pose.m_X[0], 
					p3DConvexSegment->m_Pose.m_X[1], 
					p3DConvexSegment->m_Pose.m_X[2]);
			}

			fprintf(fpLog, "\n");
		}

		fclose(fpLog);
#endif

#pragma region Color/Texture properties
		//TEXTONI m_pPSD->Get3DTextons(&(pPSuLM->m_SurfaceList));
		if(m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL)
		{
			/********************************************************************************/
			//Create 3DMeshObjects for each 3D surface starting from LEVEL3->LEVEL2->triangle
			RVLQLIST_PTR_ENTRY *pElement;
			CRVL3DSurface2 **pp3DConvexSegment, **pp3DConvexSegmentEnd;
			CRVL2DRegion2 *p2DSegmentLevel2, *p2DSegmentLevel3;
			CRVL2DRegion2 **ppGroupedTriangle, **ppGroupedTriangleEnd, *pTriangle;

			RVLMEM_ALLOC_STRUCT(pMem, CRVL3DMeshObject, pPSuLM->m_pRootMeshObject);
			CRVL3DMeshObject *pRootMO = pPSuLM->m_pRootMeshObject;
			pRootMO->m_pClass  = &m_S3DSurfaceSet;

			//Init root MO
			pRootMO->Init();
			
			pRootMO->InitParent();

		
			//For each LEVEL3
			for(i=0;i<n3DSurfaces;i++)	//n3DSurfaces
			{
				p3DSurface = SurfaceArray[i];
				p3DSurface->InitParent();
				p3DSurface->InitChild();

				p3DSurface->parentMeshObject = pRootMO;
				p3DSurface->rootMeshObject = pRootMO;

				RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pElement);
				pElement->Ptr = p3DSurface;
				RVLQLIST_ADD_ENTRY(pRootMO->m_ChildMeshObjects, pElement);

				p2DSegmentLevel3 = (CRVL2DRegion2 *)(p3DSurface->m_vp2DRegion);
				pRelList = p2DSegmentLevel3->m_RelList + p2DSegmentLevel3->m_pClass->m_iRelListElements;

				ppGroupedTriangle = (CRVL2DRegion2 **)pRelList->pFirst;
				ppGroupedTriangleEnd = (CRVL2DRegion2 **)pRelList->pEnd;

				for(; ppGroupedTriangle < ppGroupedTriangleEnd; ppGroupedTriangle++)
				{
					pTriangle = *ppGroupedTriangle;

					if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
						continue;

					RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pElement);
					pElement->Ptr = pTriangle;

					//Add LEVEL1 regions to LEVEL3 m_facelist 
					RVLQLIST_ADD_ENTRY(p3DSurface->m_FaceList, pElement);
					p3DSurface->m_noFaces++;

				}


				if(m_pPSD->m_Flags & RVLPSD_MESH_CONVEX)
				{			
					//get corresponding convex segments ie LEVEL2
					CRVL3DSurface2 **pp3DConvexSegment, **pp3DConvexSegmentEnd;
					CRVL2DRegion2 *p2DSegmentLevel2;
					pRelListSurface = p3DSurface->m_RelList + p3DSurface->m_pClass->m_iRelListComponents;
					pp3DConvexSegment = (CRVL3DSurface2 **)(pRelListSurface->pFirst);
					pp3DConvexSegmentEnd = (CRVL3DSurface2 **)(pRelListSurface->pEnd);


					for(; pp3DConvexSegment < pp3DConvexSegmentEnd; pp3DConvexSegment++)
					{
						p3DConvexSegment = *pp3DConvexSegment;
						
						if(p3DConvexSegment->m_Flags & RVLOBJ2_FLAG_REJECTED)
							continue;

						//Add LEVEL2 ie convex segment to LEVEL3 list
						RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pElement);
						pElement->Ptr = p3DConvexSegment;
						RVLQLIST_ADD_ENTRY(p3DSurface->m_ChildMeshObjects, pElement);
						
						//Add LEVEL1 regions to m_facelist
						p3DConvexSegment->InitChild();
						
						p2DSegmentLevel2 = (CRVL2DRegion2 *)(p3DConvexSegment->m_vp2DRegion);
						pRelList = p2DSegmentLevel2->m_RelList + p2DSegmentLevel2->m_pClass->m_iRelList[RVLRELLIST_COMPONENTS];
			
						ppGroupedTriangle = (CRVL2DRegion2 **)pRelList->pFirst;
						ppGroupedTriangleEnd = (CRVL2DRegion2 **)pRelList->pEnd;

						for(; ppGroupedTriangle < ppGroupedTriangleEnd; ppGroupedTriangle++)
						{
							pTriangle = *ppGroupedTriangle;

							if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
								continue;

							RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pElement);
							pElement->Ptr = pTriangle;

							//Add LEVEL1 regions to LEVEL2 m_facelist 
							RVLQLIST_ADD_ENTRY(p3DConvexSegment->m_FaceList, pElement);
							p3DConvexSegment->m_noFaces++;


							//Add LEVEL1 regions to LEVEL3 m_facelist 
							//RVLQLIST_ADD_ENTRY(p3DSurface->m_FaceList, pElement);
							//p3DSurface->m_noFaces++;

						}
						
						//Update noVertices for LEVEL2
						p3DConvexSegment->m_noVertices = p3DConvexSegment->m_noFaces + 2;
					
					
					}
				}
			

				//Update noVertices for LEVEL3
				p3DSurface->m_noVertices = p3DSurface->m_noFaces + 2;



			}

			////Generate Colour descriptors ////IZBACITI IZVAN CREATE!!! 
			//IplImage *pImageNHS = CRVLImageFilter::RVLFilterNHS(m_pCamera->m_pRGBImage);

			////RVLSetUsefulPixMask(&(m_pAImage->m_C2DRegion), m_pPSD->m_Point3DMap, m_pAImage->m_Width, m_pAImage->m_Height,m_pMem);

			//
			//float histBase = 8.0;

			//RVLQLIST_PTR_ENTRY *pEl;
			//
			//CRVL3DMeshObject *p3DMeshObject;

			//pElement = (RVLQLIST_PTR_ENTRY*)(pRootMO->m_ChildMeshObjects->pFirst);

			//while(pElement)
			//{
			//	p3DMeshObject = (CRVL3DMeshObject *)(pElement->Ptr);
			//	p3DMeshObject->RVLCalculateRGBHist(pImageNHS, histBase, true);
			//	//p3DMeshObject->RVLCalculateTextureLabel();
			//	pElement = (RVLQLIST_PTR_ENTRY *)(pElement->pNext);
			//}

			//	/********************************************************************************/

			//cvReleaseImage(&pImageNHS);

			/*NEPOTREBNO OVDJE FILE *dat, *mtldat;
			dat = fopen("C:\\RVL\\ExpRez\\meshOBJ.obj", "w");
			mtldat = fopen("C:\\RVL\\ExpRez\\meshOBJ.obj.mtl", "w");
			pRootMO->SaveMeshObject2OBJ(dat, mtldat, "meshOBJ.obj.mtl", m_pPSD->m_Point3DMap,false);
			fclose(dat);
			fclose(mtldat);*/
		}	//if(m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL)
#pragma endregion

		if(bClose)
		{
			//TEST KARLO //GET Min dxn of Info content
			BOOL bReal[3];
			BOOL bState = TRUE;
			double eigVal[3], eigVal2[3];
			double Veig[3];

			RVLCOMPLETESIMMX3(InfMx);

			bState = RVLGetMinEigVector3(InfMx, eigVal, bReal, Veig);

			pPSuLM->m_minInfo = bState ? eigVal[0] : -2;
			pPSuLM->m_minPlaneExists = 0;

			//Check if there is at least one surface in the min dxn for the first 20 surfaces
			double cst;
			if(bState)
			{
				for(i=0;i<pPSuLM->m_n3DSurfaces;i++)
				{
					p3DSurface = SurfaceArray[i];
					cst = RVLDOTPRODUCT3(p3DSurface->m_N, Veig);
					if(cst < -m_csLastDOFSurfNrmAngle || cst > m_csLastDOFSurfNrmAngle)
					{
						pPSuLM->m_minPlaneExists = 1;
						break;
					}
				}
			}

			//END TEST
		}

		//if(pPSuLM->m_minPlaneExists==0)
		//	int gg = 0;

#ifdef DISPLAY3DSURFACES	// convex segments
		/********************************************************************************/
		//Create 3DMeshObjects for each 3D surface starting from LEVEL3->triangle
		RVLQLIST_PTR_ENTRY *pElement;
		CRVL2DRegion2 *p2DSegmentLevel3;
		CRVL2DRegion2 **ppGroupedTriangle, **ppGroupedTriangleEnd, *pTriangle;

		RVLMEM_ALLOC_STRUCT(pMem, CRVL3DMeshObject, pPSuLM->m_pRootMeshObject);
		CRVL3DMeshObject *pRootMO = pPSuLM->m_pRootMeshObject;
		pRootMO->m_pClass  = &m_S3DSurfaceSet;

		//Init root MO
		pRootMO->Init();
		
		pRootMO->InitParent();

		//For each LEVEL3
		for(i=0;i<20;i++)
		{
			p3DSurface = SurfaceArray[i];
			p3DSurface->InitParent();
			p3DSurface->InitChild();


			p3DSurface->parentMeshObject = pRootMO;
			p3DSurface->rootMeshObject = pRootMO;

			RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pElement);
			pElement->Ptr = p3DSurface;
			RVLQLIST_ADD_ENTRY(pRootMO->m_ChildMeshObjects, pElement);

			
			
				
			//Add LEVEL1 regions to m_facelist
						
			p2DSegmentLevel3 = (CRVL2DRegion2 *)(p3DSurface->m_vp2DRegion);
			pRelList = p2DSegmentLevel3->m_RelList + p2DSegmentLevel3->m_pClass->m_iRelList[RVLRELLIST_ELEMENTS];

			ppGroupedTriangle = (CRVL2DRegion2 **)pRelList->pFirst;
			ppGroupedTriangleEnd = (CRVL2DRegion2 **)pRelList->pEnd;

			for(; ppGroupedTriangle < ppGroupedTriangleEnd; ppGroupedTriangle++)
			{
				pTriangle = *ppGroupedTriangle;

				if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
					continue;

				RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pElement);
				pElement->Ptr = pTriangle;

				//Add LEVEL1 regions to LEVEL3 m_facelist 
				RVLQLIST_ADD_ENTRY(p3DSurface->m_FaceList, pElement);
				p3DSurface->m_noFaces++;

			}
			

			//Update noVertices for LEVEL3
			p3DSurface->m_noVertices = p3DSurface->m_noFaces + 2;



		}


		
		//Generate Colour descriptors 

		IplImage *pImageNHS = CRVLImageFilter::RVLFilterNHS(m_pCamera->m_pRGBImage);
		RVLSetUsefulPixMask(&(m_pAImage->m_C2DRegion), m_pPSD->m_Point3DMap, m_pAImage->m_Width, m_pAImage->m_Height,m_pMem);

		float histBase = 16.0;

		RVLQLIST_PTR_ENTRY *pEl;
		
		CRVL3DMeshObject *p3DMeshObject;

		pElement = (RVLQLIST_PTR_ENTRY*)(pRootMO->m_ChildMeshObjects->pFirst);

		while(pElement)
		{
			p3DMeshObject = (CRVL3DMeshObject *)(pElement->Ptr);
			p3DMeshObject->RVLCalculateRGBHist(pImageNHS, histBase);
			p3DMeshObject->RVLCalculateTextureLabel();
			pElement = (RVLQLIST_PTR_ENTRY *)(pElement->pNext);
		}

		/********************************************************************************/

		cvReleaseImage(&pImageNHS);

		FILE *dat, *mtldat;
		dat = fopen("C:\\RVL\\ExpRez\\meshOBJ.obj", "w");
		mtldat = fopen("C:\\RVL\\ExpRez\\meshOBJ.obj.mtl", "w");
		pRootMO->SaveMeshObject2OBJ(dat, mtldat, "meshOBJ.obj.mtl", m_pPSD->m_Point3DMap);
		fclose(dat);
		fclose(mtldat);

#endif
		
		//exit elegantly
		if (m_pPSD->m_Flags & RVLPSD_MESH_CONVEX)
			delete[] TransformedVertexArray;
	}	// if(m_Flags & RVLPSULMBUILDER_FLAG_SURFACES)

	//ADD CODE HERE TO STORE TO FILE AND CHECK USING PYTHON!!

	//m_CreateTime = m_pTimer->GetTime() - StartTime;

#ifdef NEVER
	if(m_Flags & RVLPSULMBUILDER_FLAG_LINES)
	{
		//********************************************************//
		//2. Create 3D lines
		//********************************************************//

		//RVLDELAUNAY_CONNECTION_FLAG_MARKED2 -> indicates delaunay edge has been added as edge (contour) of LEVEL1

		//USe Level1 Delaunay edges 

		//FILE *fp;

		//fopen_s(&fp, "C:\\RVL\\ExpRez\\uvdcontour.dat", "w");

		//FILE *fpApproxPoly;

		//fopen_s(&fpApproxPoly, "C:\\RVL\\ExpRez\\ApproxPoly.dat", "w");

		double StartTime = m_pTimer->GetTime();

		m_S3DLineSet.m_ObjectList.RemoveAll();

		m_S2DLineSet.m_ObjectList.RemoveAll();

		CRVL2DLine2 *p2DLine;
		CRVL3DLine2 *p3DLine;

		//Start from the beginning
		RVLDELAUNAY_CONNECTION **ppDelaunayEdgeLevel1;

		RVLDELAUNAY_CONNECTION *pDelaunayLink0;
			
		RVLDELAUNAY_CONNECTION *pDelaunayLink, *DelaunayLink;
		BYTE *pDelaunayData;

		RVLDELAUNAY_CONNECTION *pDelaunayLinkIn, *pDelaunayLinkOut;

		int nDelaunayLinks;

		int iPix, iConnection, iConnection2;

		int u1, v1, u2, v2, i2DLineLength, idU, idV;

		RVLRECT Border;

		Border.left = (m_ROI.left >> 1) + m_iApproxPolyPrecision2;
		Border.right = (m_ROI.right >> 1) - m_iApproxPolyPrecision2;
		Border.top = (m_ROI.top >> 1) + m_iApproxPolyPrecision2;
		Border.bottom = (m_ROI.bottom >> 1) - m_iApproxPolyPrecision2;

		double uc = m_pCamera->CenterXNrm;
		double vc = m_pCamera->CenterYNrm;

		WORD iLine = 0;

		//int iU[2];
		double U1[2], U2[2];
		//double fd;
		//short int d;
		double d1, d2;

#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
		CRVL2DRegion2 *p2DRegionLevel3, *p2DRegionRef;
		int j;
#else
		int maxnContourPts = iWidth * iHeight;

		CvPoint *PtArray = new CvPoint[maxnContourPts + 1];

		int *iPixArray = new int[maxnContourPts];

		short int *dArray = new short int[maxnContourPts];

		//double *Pt3DArray = new double[3 * maxnContourPts];

		CvPoint **BreakPtPtrArray = new CvPoint *[maxnContourPts];

		int *piPix;
		CvPoint *pPt;
		int nPts;
		CvPoint **ppBreakPt;
		CvPoint **pBreakPtPtrArrayEnd;
		int iPt, iPt2;
		CRVL2DLine2 *p2DLinePrev;
#endif	

		RVLQLIST *pLandmarkList = &(pPSuLM->m_LandmarkList);

		RVLQLIST_INIT(pLandmarkList)

		int nLandmarks = 0;

		ppDelaunayEdgeLevel1 = m_pSegmentation->m_DelaunayEdges;

		BYTE **DelaunayMap = m_pSegmentation->m_pDelaunay->m_DelaunayMap;

		double *X1, *X2;
		double *CX1, *CX2;
		double vardy;
		double dX[3];
		double invCX1[2 * 2], invCX2[2 * 2], invCX12[2 * 2];
		double dX2[2];
		double CTmp[2 * 2];
		double V[3];
		double e;
		RVLPSULM_LANDMARK *pLandmark;

		for(i = 0; i < m_pSegmentation->m_nDelaunayEdges; i++, ppDelaunayEdgeLevel1++)
		{
#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
			//Clear sequence
			cvClearSeq(pSeqContour);
#endif

			pDelaunayLink0 = *ppDelaunayEdgeLevel1;

			if((pDelaunayLink0->Flags & RVLDELAUNAY_CONNECTION_FLAG_MARKED2)==0)
			{
#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
				p2DRegion = (CRVL2DRegion2 *)(pDelaunayLink0->vp2DRegion);
				p2DRegionLevel3 = *((CRVL2DRegion2 **)(p2DRegion->m_pData + p2DRegionSet1->m_iDataGrandParentPtr));

				p2DRegionRef = (p2DRegionLevel3 == NULL) ? p2DRegion : p2DRegionLevel3;

				if(p2DRegionRef->m_a == 0.0 && p2DRegionRef->m_b == 0.0 && p2DRegionRef->m_c == 0.0)
					continue;
#else
				piPix = iPixArray;

				pPt = PtArray;
#endif
				
				iPix = pDelaunayLink0->iPix;
				iConnection = pDelaunayLink0->iConnection;

				pDelaunayData = DelaunayMap[iPix];
				
				nDelaunayLinks = *((short *)pDelaunayData);

				DelaunayLink = (RVLDELAUNAY_CONNECTION *)(pDelaunayData + sizeof(short));
				pDelaunayLink = DelaunayLink + iConnection;
				
				iConnection2 = iConnection;
				pDelaunayLinkIn = pDelaunayLink;
			
				//mark link
				pDelaunayLinkIn->Flags |= RVLDELAUNAY_CONNECTION_FLAG_MARKED2;

				do
				{
					do
					{
						iConnection2 = (iConnection2 + 1) % nDelaunayLinks;
						pDelaunayLinkOut = DelaunayLink + iConnection2;
					}
					while((pDelaunayLinkOut->Flags & RVLDELAUNAY_CONNECTION_FLAG_EDGE)==0);

					//mark link
					pDelaunayLinkOut->Flags |= RVLDELAUNAY_CONNECTION_FLAG_MARKED2;


					//store point (add to sequence)
#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
					pt.x = pDelaunayLinkIn->iPix % m_pPSD->m_Width;
					pt.y = pDelaunayLinkIn->iPix / m_pPSD->m_Width;
					cvSeqPush( pSeqContour,&pt);
#else
					iPix = pDelaunayLinkIn->iPix;

					*(piPix++) = iPix;

					pPt->x = iPix % iWidth;
					pPt->y = iPix / iWidth;

					pPt++;
#endif

					pDelaunayData = DelaunayMap[pDelaunayLinkOut->iPix];

					nDelaunayLinks = *((short *)pDelaunayData);

					DelaunayLink = (RVLDELAUNAY_CONNECTION *)(pDelaunayData + sizeof(short));

					iConnection2 = pDelaunayLinkOut->iConnection;

					pDelaunayLinkIn = DelaunayLink + iConnection2;
				}	
				while(pDelaunayLinkIn != pDelaunayLink);

				pPt->x = PtArray->x;
				pPt->y = PtArray->y;

#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
				//Approximate contour
				pSeqContourApprox = cvApproxPoly( pSeqContour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, m_iApproxPolyPrecision2, 0 ); 

				//make sure the approximate polygon has at least 3 points
				if(pSeqContourApprox->total > 2)

				{
#else
				//Stereo Contour Recontstruction

				nPts = piPix - iPixArray;

				if(m_pStereoVision->ContourReconstruction(iPixArray, nPts, dArray))
				{
					// save reconstructed contour to a file

					//double *X;
					//int u, v;

					//X = Pt3DArray;

					//for(iPt = 0; iPt < nPts; iPt++, X += 3)
					//{
					//	u = iPixArray[iPt] % iWidth;
					//	v = iPixArray[iPt] / iWidth;

					//	fprintf(fp, "%d\t%d\t%d\t%lf\t%lf\t%lf\n", u, v, dArray[iPt], X[0], X[1], X[2]);
					//}	

					// split contours into approx. linear segments					

					ppBreakPt = BreakPtPtrArray;

					RVL2DContourSegment(PtArray, PtArray + nPts, m_iApproxPolyPrecision2Nrm, &ppBreakPt);

					*(ppBreakPt++) = PtArray;

					pPt = PtArray;

					pBreakPtPtrArrayEnd = ppBreakPt;

					// save linear segments to a file

					//iPt = 0;

					//X = Pt3DArray;

					//for(ppBreakPt = BreakPtPtrArray; ppBreakPt < pBreakPtPtrArrayEnd; ppBreakPt++)
					//{
					//	pPt2 = *ppBreakPt;

					//	int du = pPt2->x - pPt->x;
					//	int dv = pPt2->y - pPt->y;

					//	if(du * du + dv * dv >= 10 * 10)
					//	{
					//		fprintf(fpApproxPoly, "%d\t%d\t%d\t%lf\t%lf\t%lf\t", pPt->x, pPt->y, dArray[iPt], X[0], X[1], X[2]);

					//		iPt = pPt2 - PtArray;

					//		X = Pt3DArray + 3 * iPt;

					//		fprintf(fpApproxPoly, "%d\t%d\t%d\t%lf\t%lf\t%lf\n", pPt2->x, pPt2->y, dArray[iPt], X[0], X[1], X[2]);
					//	}
					//	else
					//	{
					//		iPt = pPt2 - PtArray;

					//		X = Pt3DArray + 3 * iPt;
					//	}
					//
					//	pPt = pPt2;
					//}
#endif

#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
			
					for(j = 0;j < pSeqContourApprox->total - 1; j++)
					{

						pt =  *(CvPoint *)cvGetSeqElem(pSeqContourApprox,j);
						pt2 =  *(CvPoint *)cvGetSeqElem(pSeqContourApprox,j + 1);
#else
					iPt2 = 0;

					pt2.x = PtArray->x;
					pt2.y = PtArray->y;

					p2DLinePrev = NULL;

					for(ppBreakPt = BreakPtPtrArray; ppBreakPt < pBreakPtPtrArrayEnd; ppBreakPt++)
					{
						iPt = iPt2;

						pt.x = pt2.x;
						pt.y = pt2.y;

						pPt = *ppBreakPt;

						iPt2 = pPt - PtArray;

						pt2.x = pPt->x;
						pt2.y = pPt->y;

						//RVLRECT ROI;

						//ROI.left = 150;
						//ROI.right = 210;
						//ROI.top = 60;
						//ROI.bottom = 80;

						//if(IsInside(ROI, pt.x, pt.y) || IsInside(ROI, pt2.x, pt2.y))
						//	int tmp1 = 0;
#endif

						if(pt.x <= Border.left && pt2.x <= Border.left)
							continue;

						if(pt.x >= Border.right && pt2.x >= Border.right)
							continue;

						if(pt.y <= Border.top && pt2.y <= Border.top)
							continue;

						if(pt.y >= Border.bottom && pt2.y >= Border.bottom)
							continue;

						if((pt.x == 215 && pt.y == 51) || (pt2.x == 215 && pt2.y == 51))
							int tmp1 = 0;

						u1 = (pt.x << 1) + 1;
						v1 = (pt.y << 1) + 1;
						u2 = (pt2.x << 1) + 1;
						v2 = (pt2.y << 1) + 1;
					
						idU = (u2 - u1);
						idV = (v2 - v1);

						i2DLineLength = idU * idU + idV * idV;

						//store line if longer than threshold
						if(i2DLineLength <= m_2DLineLengthTreshold2)
							continue;

						// 3D reconstruction

						U1[0] = (double)(pt.x);
						U1[1] = (double)(pt.y);

#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
						d1 = ((p2DRegionRef->m_a * U1[0]) + (p2DRegionRef->m_b * U1[1]) + p2DRegionRef->m_c) / 16.0;

						if(d1 < 0.0)
							continue;
#else
						d1 = (double)(dArray[iPt]);
#endif

						if(d1 == 0)
							continue;

						U2[0] = (double)(pt2.x);
						U2[1] = (double)(pt2.y);
				
#ifdef RVLPSULM_CREATE_3DLINES_FROM_PLANES
						d2 = ((p2DRegionRef->m_a * U2[0]) + (p2DRegionRef->m_b * U2[1]) + p2DRegionRef->m_c) / 16.0;

						if(d2 < 0.0)
							continue;
#else
						d2 = (double)(dArray[iPt2]);
#endif

						if(d2 == 0)
							continue;

						//Create 2D line

						p2DLine = (CRVL2DLine2 *)RVL2DLine2Template.Create3(&(m_S2DLineSet));

						iLine++;
						
						p2DLine->m_iU[0][0] = u1;
						p2DLine->m_iU[0][1] = v1;
						p2DLine->m_iU[1][0] = u2;
						p2DLine->m_iU[1][1] = v2;

						p2DLine->m_diU[0] = idU;
						p2DLine->m_diU[1] = idV;
						
						p2DLine->m_leniU = DOUBLE2INT(sqrt(i2DLineLength*1.0));

						//Create corresponding 3D line

						p3DLine = (CRVL3DLine2 *)RVL3DLine2Template.Create3(&(m_S3DLineSet));

						U1[0] -= uc;
						U1[1] -= vc;

						m_pCamera->StereoReconWithUncert2(U1, d1, m_pStereoVision->m_BaseLenNrm, 1.0, 1.0, p3DLine->m_X[0], p3DLine->m_CX[0]);

						U2[0] -= uc;
						U2[1] -= vc;

						m_pCamera->StereoReconWithUncert2(U2, d2, m_pStereoVision->m_BaseLenNrm, 1.0, 1.0, p3DLine->m_X[1], p3DLine->m_CX[1]);

						*((CRVL2DLine2 **)(p3DLine->m_pData + m_S3DLineSet.m_iDataProjectPtr)) = p2DLine;

						*((CRVL3DLine2 **)(p2DLine->m_pData + m_S2DLineSet.m_iData3DObjectPtr)) = p3DLine;

						// create landmark

						X1 = p3DLine->m_X[0];
						X2 = p3DLine->m_X[1];

						CX1 = p3DLine->m_CX[0];
						CX2 = p3DLine->m_CX[1];

						dX[0] = X2[0] - X1[0];
						dX[1] = X2[1] - X1[1];
						dX[2] = X2[2] - X1[2];

						if(m_Flags & RVLPSULMBUILDER_FLAG_VP)
						{
							vardy = CX1[3 * 1 + 1] + CX2[3 * 1 + 1];

							if(dX[1] * dX[1] / vardy <= m_HorLineChi2Thr)
								p3DLine->m_Flags |= RVL3DLINE_PARAM_FLAG_HORIZONTAL;
						}

						//if(m_Flags & RVLPSULMBUILDER_FLAG_CORNER_LANDMARKS)
						//{
						//	if(p2DLinePrev)
						//	{


						//		
						//	}
						//}

						CTmp[2 * 0 + 0] = CX1[3 * 0 + 0] + CX2[3 * 0 + 0];
						CTmp[2 * 0 + 1] = CTmp[2 * 1 + 0] = CX1[3 * 0 + 2] + CX2[3 * 0 + 2];
						CTmp[2 * 1 + 1] = CX1[3 * 2 + 2] + CX2[3 * 2 + 2];

						dX2[0] = dX[0];
						dX2[1] = dX[2];

						e = RVLMahalanobisDistance2D(dX2, CTmp);

						if(e <= m_VerLineChi2Thr)
						{
							p3DLine->m_Flags |= RVL3DLINE_PARAM_FLAG_VERTICAL;

							RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_LANDMARK, pLandmark);

							RVLQLIST_ADD_ENTRY(pLandmarkList, pLandmark);

							nLandmarks++;

							CTmp[2 * 0 + 0] = CX1[3 * 0 + 0];
							CTmp[2 * 0 + 1] = CTmp[2 * 1 + 0] = CX1[3 * 0 + 2];
							CTmp[2 * 1 + 1] = CX1[3 * 2 + 2];

							InverseMatrix2(invCX1, CTmp, APPROX_ZERO);

							CTmp[2 * 0 + 0] = CX2[3 * 0 + 0];
							CTmp[2 * 0 + 1] = CTmp[2 * 1 + 0] = CX2[3 * 0 + 2];
							CTmp[2 * 1 + 1] = CX2[3 * 2 + 2];

							InverseMatrix2(invCX2, CTmp, APPROX_ZERO);

							CTmp[2 * 0 + 0] = invCX1[2 * 0 + 0] + invCX2[2 * 0 + 0];
							CTmp[2 * 0 + 1] = CTmp[2 * 1 + 0] = invCX1[2 * 0 + 1] + invCX2[2 * 0 + 1];
							CTmp[2 * 1 + 1] = invCX1[2 * 1 + 1] + invCX2[2 * 1 + 1];

							InverseMatrix2(invCX12, CTmp, APPROX_ZERO);

							V[0] = invCX1[2 * 0 + 0] * X1[0] + invCX1[2 * 0 + 1] * X1[2] +
								invCX2[2 * 0 + 0] * X2[0] + invCX2[2 * 0 + 1] * X2[2];
							V[1] = invCX1[2 * 1 + 0] * X1[0] + invCX1[2 * 1 + 1] * X1[2] +
								invCX2[2 * 1 + 0] * X2[0] + invCX2[2 * 1 + 1] * X2[2];

							pLandmark->x = invCX12[2 * 0 + 0] * V[0] + invCX12[2 * 0 + 1] * V[1];
							pLandmark->z = invCX12[2 * 1 + 0] * V[0] + invCX12[2 * 1 + 1] * V[1];

							pLandmark->C[0] = invCX12[0];
							pLandmark->C[1] = invCX12[1];
							pLandmark->C[2] = invCX12[2];
							pLandmark->C[3] = invCX12[3];

							if(X1[1] < X2[1])
							{
								pLandmark->bDown = 1;
								pLandmark->miny = X1[1];
								pLandmark->maxy = X2[1];
							}
							else
							{
								pLandmark->bDown = 0;
								pLandmark->miny = X2[1];
								pLandmark->maxy = X1[1];
							}

							pLandmark->i3DLine = p3DLine->m_Index;
						}	
					}
				}
			}
		}

		delete[] iPixArray;
		delete[] dArray;
		//delete[] Pt3DArray;
		delete[] PtArray;
		delete[] BreakPtPtrArray;

		//fclose(fp);

		//fclose(fpApproxPoly);

		double ExecutionTime = m_pTimer->GetTime() - StartTime;
	}
#endif

	//Exit elegantly
	cvReleaseMemStorage(&storage); 

	return TRUE;
}

bool CRVLPSuLMBuilder::GetOdometry(char *ImageFileName,
								  CRVL3DPose *pPose,
								  char *extension,
								  int &iSample0,
								  unsigned char &command)
{
	char *OdometryFileName = RVLCreateFileName(ImageFileName, extension, -1, "-O.txt");

	FILE *fpOdometry = fopen(OdometryFileName, "r");

	delete[] OdometryFileName;

	if(fpOdometry == NULL)
		return false;

	double pan, tilt, roll;

	fscanf(fpOdometry, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d\t%c\n", pPose->m_X, pPose->m_X + 1, pPose->m_X + 2, &pan, &tilt, &roll, &iSample0, &command);

	fclose(fpOdometry);

	// todo: correct odometry files of the experiments reported in IROS15 and remove the next three lines 

	if (m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
		if (pan == 70.0)
			pan += 7.0;

	/////

	pPose->m_Alpha = m_kPan * pan * DEG2RAD;
	pPose->m_Beta = m_kTilt * (tilt + m_TiltOffset) * DEG2RAD;;			 
	pPose->m_Theta = roll * DEG2RAD;

	pPose->UpdateRotLL();

	return true;
}
	
// uses m_pMem2

CRVLPSuLM *CRVLPSuLMBuilder::Create(
	DWORD Flags,
	CRVLPSuLM *pPSuLM_)
{
	// initialize memory and sets

	CRVLMem *pMem;
	//CRVLC3D *p3DSurfaceSet;
	CRVLClass *p3DSurfaceSet;
	//CRVLC3D *p3DLineSet;
	CRVLClass *p3DLineSet;
	CRVLClass *p2DLineSet;

	if(Flags & RVLPSULMBUILDER_CREATEMODEL_FLAG_PERMANENT)
		pMem = m_pMem0;
	else
		pMem = m_pMCMem;

	// create new PSuLM

	CRVL3DPose PoseM_M;
	unsigned char command;
	CRVLPSuLM *pPSuLM;
	bool bComplex;
	int iSample0;

	if(m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
	{
		if (GetOdometry(m_ImageFileName, &PoseM_M, "-LW.bmp", iSample0, command))
			bComplex = (command == 'O');
		else
			bComplex = false;
	}
	else
		bComplex = false;
	
	if(bComplex)
	{
		int iSample0 = RVLGetFileNumber(m_ImageFileName, "00000-LW.bmp");

		int iSample = iSample0;

		while (command != 'C')
		{
			RVLSetFileNumber(m_ImageFileName, "00000-LW.bmp", iSample);

			if (!GetOdometry(m_ImageFileName, &PoseM_M, "-LW.bmp", iSample0, command))
			{
				PoseM_M.m_Alpha = PoseM_M.m_Beta = PoseM_M.m_Theta = 0.0;

				command = 'C';
			}
						
			switch(command){
			case 'O':
				if (pPSuLM_)
					pPSuLM = pPSuLM_;								
				else
				{
					pPSuLM = (CRVLPSuLM *)(pMem->Alloc(sizeof(CRVLPSuLM)));

					memcpy(pPSuLM, &m_PSuLMTemplate, sizeof(CRVLPSuLM));					

					if (m_ImageFileName)
						RVLCopyString(m_ImageFileName, &(pPSuLM->m_FileName));
				}

				pPSuLM->m_Flags |= RVLPSULM_FLAG_COMPLEX;

				Create(pPSuLM, pMem, Flags, &PoseM_M, 0);

				break;
			case 'A':	
				Create(pPSuLM, pMem, Flags | RVLPSULMBUILDER_CREATEMODEL_APPEND, &PoseM_M, iSample - iSample0);

				break;
			case 'C':
				Create(pPSuLM, pMem, Flags | RVLPSULMBUILDER_CREATEMODEL_APPEND | RVLPSULMBUILDER_CREATEMODEL_APPEND_LAST, &PoseM_M, iSample - iSample0);

				if (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_GENERATION_INDEXING)
					m_Indexing.GetIndicators(pPSuLM);

				if(Flags & RVLPSULMBUILDER_CREATEMODEL_FLAG_PERMANENT)
				{
					if (pPSuLM_ == NULL)
					{
						m_PSuLMList.Add(pPSuLM);

						if (pPSuLM->m_nLandmarks > m_maxnLandmarks)
							m_maxnLandmarks = pPSuLM->m_nLandmarks;
					}
				}
			}

			iSample++;
		}	// while (command != 'C')

		RVLSetFileNumber(m_ImageFileName, "00000-LW.bmp", iSample0);
	}	// if(bComplex)
	else
	{
		if (pPSuLM_)
			pPSuLM = pPSuLM_;
		else
		{
			pPSuLM = (CRVLPSuLM *)(pMem->Alloc(sizeof(CRVLPSuLM)));

			memcpy(pPSuLM, &m_PSuLMTemplate, sizeof(CRVLPSuLM));

			if (m_ImageFileName)
				RVLCopyString(m_ImageFileName, &(pPSuLM->m_FileName));
		}

		// detect 3D surfaces, 3D lines and landmarks

		DWORD FlagsOld = m_Flags2;

		m_Flags2 &= ~RVLPSULMBUILDER_FLAG2_COMPLEX;

		Create(pPSuLM, pMem, Flags, &PoseM_M);

		m_Flags2 = FlagsOld;

		if (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_GENERATION_INDEXING)
			m_Indexing.GetIndicators(pPSuLM);

#ifdef NEVER
		// copy surface ptrs. from Builder's surface list to the PSuLM's surface list

		if(m_Flags & RVLPSULMBUILDER_FLAG_SURFACES)
		{
			CRVLMPtrChain *pBuilderSurfaceList = &(p3DSurfaceSet->m_ObjectList);

			CRVLMPtrChain *pSurfaceList = &(pPSuLM->m_SurfaceList);

			pSurfaceList->m_pMem = pMem;

			pBuilderSurfaceList->Start();

			while(pBuilderSurfaceList->m_pNext)
				pSurfaceList->Add(pBuilderSurfaceList->GetNext());
		}

		// copy 3D line ptrs. from Builder's line list to the PSuLM's line list

		double StartTime = m_pTimer->GetTime();

		CRVLMPtrChain *pBuilder3DLineList = &(p3DLineSet->m_ObjectList);

		CRVLMPtrChain *p3DLineList = &(pPSuLM->m_3DLineList);

		p3DLineList->m_pMem = pMem;

		CRVL3DLine2 *p3DLine;

		pBuilder3DLineList->Start();

		while(pBuilder3DLineList->m_pNext)
		{
			p3DLine = (CRVL3DLine2 *)(pBuilder3DLineList->GetNext());

			p3DLineList->Add(p3DLine);
		}

		// copy 2D line ptrs. from Builder's line list to the PSuLM's line list and sort 3D lines

		CRVLMPtrChain *pBuilder2DLineList = &(p2DLineSet->m_ObjectList);

		CRVLMPtrChain *p2DLineList = &(pPSuLM->m_2DLineList);

		p2DLineList->m_pMem = pMem;

		CRVL2DLine2 *p2DLine;

		RVLQLIST *ListArray = m_LineSortBuff.m_ListArray;

		RVLQLIST_PTR_ENTRY *pEntry;

		m_LineSortBuff.Reset();

		BYTE *pMem2 = m_pMem2->m_pFreeMem;

		int max2DLineLen = 0;

		int evidence = 0;

		int len;

		pBuilder2DLineList->Start();

		while(pBuilder2DLineList->m_pNext)
		{
			p2DLine = (CRVL2DLine2 *)(pBuilder2DLineList->GetNext());

			p2DLineList->Add(p2DLine);

			RVLMEM_ALLOC_STRUCT(m_pMem2, RVLQLIST_PTR_ENTRY, pEntry);

			len = p2DLine->m_leniU;

			RVLQLISTARRAY_ADD_ENTRY(ListArray, len, pEntry);

			pEntry->Ptr = *((CRVL3DLine2 **)(p2DLine->m_pData + p2DLineSet->m_iData3DObjectPtr));

			if(len > max2DLineLen)
				max2DLineLen = len;

			evidence += len;
		}

		pPSuLM->m_Evidence = evidence;

		pPSuLM->m_3DLineArray = (CRVL3DLine2 **)(pMem->Alloc(p2DLineList->m_nElements * sizeof(CRVL3DLine2 *)));

		CRVL3DLine2 **pp3DLine = pPSuLM->m_3DLineArray;

		pPSuLM->m_n3DLines = p2DLineList->m_nElements;

		int i3DLine = 0;

		int i;

		for(i = max2DLineLen; i >= m_2DLineLengthTreshold; i--)
		{
			pEntry = (RVLQLIST_PTR_ENTRY *)(ListArray[i].pFirst);

			while(pEntry)
			{
				p3DLine = (CRVL3DLine2 *)(pEntry->Ptr);

				p3DLine->m_Index = (i3DLine++);

				*(pp3DLine++) = p3DLine;

				pEntry = (RVLQLIST_PTR_ENTRY *)(pEntry->pNext);
			}
		}
		
		// create cell array

		RVLPSULM_CELL *CellArray = pPSuLM->m_CellArray = (RVLPSULM_CELL *)(pMem->Alloc(m_nCells * sizeof(RVLPSULM_CELL)));

		memcpy(CellArray, m_EmptyCellArray, m_nCells * sizeof(RVLPSULM_CELL));

		RVLPSULM_CELL_LINEPTR *LinePtrMem = (RVLPSULM_CELL_LINEPTR *)
			(pMem->Alloc(m_nCells * p2DLineList->m_nElements * sizeof(RVLPSULM_CELL_LINEPTR)));

		RVLPSULM_CELL_LINEPTR *pLinePtr = LinePtrMem;	

		RVLPSULM_CELL_LINEPTR ***LastLinePtrPtrPtrArray = (RVLPSULM_CELL_LINEPTR ***)(m_pMem2->Alloc(m_nCells * sizeof(RVLPSULM_CELL_LINEPTR **)));

		RVLPSULM_CELL_LINEPTR ***pLastLinePtrPtrPtrArrayEnd = LastLinePtrPtrPtrArray + m_nCells;

		RVLPSULM_CELL *pCell = CellArray;

		RVLPSULM_CELL_LINEPTR ***pppLastLinePtr;
		RVLPSULM_CELL_LINEPTR **ppLastLinePtr;

		for(pppLastLinePtr = LastLinePtrPtrPtrArray; pppLastLinePtr < pLastLinePtrPtrPtrArrayEnd; pppLastLinePtr++, pCell++)
			*pppLastLinePtr = &(pCell->pLinePtr);

		int *piSampleCellArrayEnd;
		int *piCell;
		CvPoint Pt1, Pt2;
		int *iU;

		pBuilder2DLineList->Start();

		while(pBuilder2DLineList->m_pNext)
		{
			p2DLine = (CRVL2DLine2 *)(pBuilder2DLineList->GetNext());

			iU = p2DLine->m_iU[0];

			Pt1.x = iU[0];
			Pt1.y = iU[1];

			iU = p2DLine->m_iU[1];

			Pt2.x = iU[0];
			Pt2.y = iU[1];

			LineSampling(&Pt1, &Pt2);		

			piSampleCellArrayEnd = m_iSampleCellArray + m_nSampleCells;

			for(piCell = m_iSampleCellArray; piCell < piSampleCellArrayEnd; piCell++)
			{
				pCell = CellArray + (*piCell);

				//pCell->LinePtrList.Add(p2DLine);

				pLinePtr->pLine = p2DLine;
				pLinePtr->pNext = NULL;
				pppLastLinePtr = LastLinePtrPtrPtrArray + (*piCell);
				ppLastLinePtr = *pppLastLinePtr;
				*ppLastLinePtr = pLinePtr;
				*pppLastLinePtr = &(pLinePtr->pNext);
				pLinePtr++;
			}		
		}

		pMem->m_pFreeMem = (BYTE *)pLinePtr;

		m_pMem2->m_pFreeMem = pMem2;

		double ExectutionTime = m_pTimer->GetTime() - StartTime;

		// detect ground plane

		double *NGroundPlane = pPSuLM->m_NGroundPlane;

		if(m_Flags & RVLPSULMBUILDER_FLAG_SURFACES)
		{
			CRVLMPtrChain *pDominantPlaneList = &(m_pAImage->m_C2DRegion3.m_ObjectList);

			CRVL3DSurface2 Plane3D;

			double minsnBeta = sin(-m_Beta0 - m_BetaTol);
			double maxsnBeta = sin(-m_Beta0 + m_BetaTol);
			double maxtgTheta = tan(m_ThetaTol);
			double tgTheta;
			double minCameraHeight = m_CameraHeight0 - m_CameraHeightTol;
			double maxCameraHeight = m_CameraHeight0 + m_CameraHeightTol;

			double *N = Plane3D.m_N;

			int maxnPts = 0;

			BOOL bGroundPlane = FALSE;

			CRVL2DRegion2 *pUVDPlane;
			double CameraHeight;

			pDominantPlaneList->Start();

			while(pDominantPlaneList->m_pNext)
			{
				pUVDPlane = (CRVL2DRegion2 *)(pDominantPlaneList->GetNext());

				if(pUVDPlane->m_n3DPts <= maxnPts)
					continue;

				m_pPSD->Get3DPlane(pUVDPlane, &Plane3D);		

				if(N[1] < 0.0)
				{
					N[0] = -N[0];
					N[1] = -N[1];
					N[2] = -N[2];
					CameraHeight = -Plane3D.m_d;
				}
				else
					CameraHeight = Plane3D.m_d;

				if(CameraHeight < minCameraHeight || CameraHeight > maxCameraHeight)
					continue;

				if(N[2] < minsnBeta || N[2] > maxsnBeta)
					continue;

				tgTheta = N[0] / N[1];

				if(tgTheta < -maxtgTheta || tgTheta > maxtgTheta)
					continue;

				NGroundPlane[0] = N[0];
				NGroundPlane[1] = N[1];
				NGroundPlane[2] = N[2];
				pPSuLM->m_CameraHeight = CameraHeight;

				maxnPts = pUVDPlane->m_n3DPts;

				pPSuLM->m_aGroundPlane = pUVDPlane->m_a;
				pPSuLM->m_bGroundPlane = pUVDPlane->m_b;
				pPSuLM->m_cGroundPlane = pUVDPlane->m_c;

				bGroundPlane = TRUE;
			}
		}
		else
		{
			double beta = -0.1 * DEG2RAD;
			NGroundPlane[0] = 0.0;
			NGroundPlane[1] = cos(beta);
			NGroundPlane[2] = sin(beta);
		}

		// transform 3D lines to the new c. s. with y-axis identical to the ground plane normal,
		// and x-axis perpendicular to the camera optical axis.

		double RotLG[3 * 3];

		CreateRotLG(NGroundPlane, RotLG);

		double *XGL = RotLG;

		double *ZGL = RotLG + 3 * 2;

		RVLMatrixHeaderB33->data.db = RotLG;

		double XL[3];
		double *X1;

		p3DLineList->Start();

		while(p3DLineList->m_pNext)
		{
			p3DLine = (CRVL3DLine2 *)(p3DLineList->GetNext());

			for(i = 0; i < 2; i++)
			{
				RVLMatrixHeaderA33->data.db = p3DLine->m_CX[i];

				X1 = p3DLine->m_X[i];

				RVL3DPointRot(X1, RVLMatrixHeaderA33, RVLMatrixHeaderB33, X1, RVLMatrixHeaderA33, XL, RVLMatrix33);
			}
		}

		// detect horizontal lines and landmarks

		pMem2 = m_pMem2->m_pFreeMem;

		CRVL3DLine2 **HorLineArray;
		CRVL3DLine2 **ppHorLine;

		if(m_Flags & RVLPSULMBUILDER_FLAG_VP)
		{
			HorLineArray = (CRVL3DLine2 **)(m_pMem2->Alloc(p3DLineList->m_nElements * sizeof(CRVL3DLine2 *)));

			ppHorLine = HorLineArray;
		}

		p3DLineList->Start();

		while(p3DLineList->m_pNext)
		{
			p3DLine = (CRVL3DLine2 *)(p3DLineList->GetNext());

			//p2DLine = *((CRVL2DLine2 **)(p3DLine->m_pData + p3DLineSet->m_iDataProjectPtr));	// only for debugging purpose

			//if(p2DLine->m_iU[0][0] / 2 == 253 && p2DLine->m_iU[0][1] / 2 == 5 ||
			//	p2DLine->m_iU[1][0] / 2 == 253 && p2DLine->m_iU[1][1] / 2 == 5)
			//	int tmp1 = 0;

			if(p3DLine->m_Flags & RVL3DLINE_PARAM_FLAG_HORIZONTAL)
				*(ppHorLine++) = p3DLine;
		}

		// detect the dominant vanishing point

		if(m_Flags & RVLPSULMBUILDER_FLAG_VP)
		{
			double f = m_pCamera->fNrm;

			memset(m_AlphaHist, 0, m_AlphaResolution * sizeof(int));

			double dAlpha = PI / (double)m_AlphaResolution;

			double uc = m_pCamera->CenterXNrm;
			double vc = m_pCamera->CenterYNrm;

			double U1[3], U2[3];
			double NLine[3];
			double alpha;
			double fTmp;
			double V[3];

			CRVL3DLine2 **pHorLineArrayEnd = ppHorLine;

			for(ppHorLine = HorLineArray; ppHorLine < pHorLineArrayEnd; ppHorLine++)
			{
				p3DLine = *ppHorLine;

				p2DLine = *((CRVL2DLine2 **)(p3DLine->m_pData + p3DLineSet->m_iDataProjectPtr));

				U1[0] = (double)(p2DLine->m_iU[0][0] >> 1) - uc;
				U1[1] = (double)(p2DLine->m_iU[0][1] >> 1) - vc;
				U1[2] = f;

				U2[0] = (double)(p2DLine->m_iU[1][0] >> 1) - uc;
				U2[1] = (double)(p2DLine->m_iU[1][1] >> 1) - vc;
				U2[2] = f;

				CrossProduct(U1, U2, NLine);

				CrossProduct(NLine, NGroundPlane, V);

				fTmp = RVLDotProduct(V, V);

				if(fTmp > APPROX_ZERO)
				{
					alpha = atan2(RVLDotProduct(V, ZGL), RVLDotProduct(V, XGL));

					if(alpha < 0.0)
						alpha += PI;
					else if(alpha >= PI)
						alpha -= PI;

					m_AlphaHist[(int)(alpha / dAlpha)] += p2DLine->m_leniU;
				}
			}

			m_pMem2->m_pFreeMem = pMem2;

			int iFilter = (m_AlphaFilterSize - 1) / 2;				// only for debugging purpose
			int *AlphaHistFiltered = new int[m_AlphaResolution];	// only for debugging purpose

			int fAlpha = 0;

			for(i = 0; i < m_AlphaFilterSize; i++)
				fAlpha += m_AlphaHist[i];

			AlphaHistFiltered[iFilter] = fAlpha;					// only for debugging purpose

			int maxfAlpha = fAlpha;

			int imax = m_AlphaResolution - 1;

			for(i = 0; i < m_AlphaResolution - 1; i++)
			{
				fAlpha += (m_AlphaHist[(m_AlphaFilterSize + i) % m_AlphaResolution] - m_AlphaHist[i]);

				if(fAlpha > maxfAlpha)
				{
					maxfAlpha = fAlpha;

					imax = i;
				}		

				iFilter = (iFilter + 1) % m_AlphaResolution;		// only for debugging purpose

				AlphaHistFiltered[iFilter] = fAlpha;				// only for debugging purpose
			}

			FILE *fp;
			
			fopen_s(&fp, "C:\\RVL\\ExpRez\\alphahist.dat", "w");

			for(i = 0; i < m_AlphaResolution; i++)
				fprintf(fp, "%lf\t%d\t%lf\n", dAlpha * (double)i * RAD2DEG, m_AlphaHist[i], 
					(double)AlphaHistFiltered[i] / (double)m_AlphaFilterSize);

			fclose(fp);

			delete[] AlphaHistFiltered;		// only for debugging purpose

			int sumfi = 0;

			for(i = 1; i <= m_AlphaFilterSize; i++)
				sumfi += m_AlphaHist[(imax + i) % m_AlphaResolution] * i;
			
			//alpha = dAlpha * ((double)imax + (double)sumfi / maxfAlpha);

			alpha = dAlpha * (double)(imax + (m_AlphaFilterSize + 1) / 2);

			if(alpha > PI)
				alpha -= PI;

			pPSuLM->m_Alpha = alpha;
		}
#endif

//	if(m_Flags & RVLPSULMBUILDER_FLAG_SURFACES)
//	{
//		CRVLMPtrChain *pBuilderSurfaceList = &(p3DSurfaceSet->m_ObjectList);
//
//		CRVLMPtrChain *pSurfaceList = &(pPSuLM->m_SurfaceList);
//
//		pSurfaceList->m_pMem = pMem;
//
//		pBuilderSurfaceList->Start();
//
//		while(pBuilderSurfaceList->m_pNext)
//			pSurfaceList->Add(pBuilderSurfaceList->GetNext());
//
//
//#ifdef NEVER
//		// detect ground plane 
//		double *NGroundPlane = pPSuLM->m_NGroundPlane;
//
//		CRVLMPtrChain *pDominantPlaneList = &(m_pAImage->m_C2DRegion3.m_ObjectList);
//
//		CRVL3DSurface2 Plane3D;
//
//		double minsnBeta = sin(-m_Beta0 - m_BetaTol);
//		double maxsnBeta = sin(-m_Beta0 + m_BetaTol);
//		double maxtgTheta = tan(m_ThetaTol);
//		double tgTheta;
//		double minCameraHeight = m_CameraHeight0 - m_CameraHeightTol;
//		double maxCameraHeight = m_CameraHeight0 + m_CameraHeightTol;
//
//		double *N = Plane3D.m_N;
//
//		int maxnPts = 0;
//
//		BOOL bGroundPlane = FALSE;
//
//		CRVL2DRegion2 *pUVDPlane;
//		double CameraHeight;
//
//		pDominantPlaneList->Start();
//
//		while(pDominantPlaneList->m_pNext)
//		{
//			pUVDPlane = (CRVL2DRegion2 *)(pDominantPlaneList->GetNext());
//
//			if(pUVDPlane->m_n3DPts <= maxnPts)
//				continue;
//
//			//This
//			//m_pPSD->Get3DPlaneKinect(pUVDPlane, &Plane3D);		
//			//or
//			Plane3D = *(CRVL3DSurface2 *)(pUVDPlane->m_vp3DSurface);
//
//			//This point on has to be modified!!!!
//			if(N[1] < 0.0)
//			{
//				N[0] = -N[0];
//				N[1] = -N[1];
//				N[2] = -N[2];
//				CameraHeight = -Plane3D.m_d;
//			}
//			else
//				CameraHeight = Plane3D.m_d;
//
//			if(CameraHeight < minCameraHeight || CameraHeight > maxCameraHeight)
//				continue;
//
//			if(N[2] < minsnBeta || N[2] > maxsnBeta)
//				continue;
//
//			tgTheta = N[0] / N[1];
//
//			if(tgTheta < -maxtgTheta || tgTheta > maxtgTheta)
//				continue;
//
//			NGroundPlane[0] = N[0];
//			NGroundPlane[1] = N[1];
//			NGroundPlane[2] = N[2];
//			pPSuLM->m_CameraHeight = CameraHeight;
//
//			maxnPts = pUVDPlane->m_n3DPts;
//
//			pPSuLM->m_aGroundPlane = pUVDPlane->m_a;
//			pPSuLM->m_bGroundPlane = pUVDPlane->m_b;
//			pPSuLM->m_cGroundPlane = pUVDPlane->m_c;
//
//			bGroundPlane = TRUE;
//		}
//#endif
//	}

		// add PSuLM to m_PSuLMList

		if(Flags & RVLPSULMBUILDER_CREATEMODEL_FLAG_PERMANENT)
		{
			if (pPSuLM_ == NULL)
			{
				m_PSuLMList.Add(pPSuLM);

				if (pPSuLM->m_nLandmarks > m_maxnLandmarks)
					m_maxnLandmarks = pPSuLM->m_nLandmarks;
			}
		}
	}	// if(!bComplex)

	if (m_Flags2 & RVLPSULMBUILDER_FLAG2_SURFACE_BOUNDARY)
		pPSuLM->m_Flags |= RVLPSULM_FLAG_SURFACE_BOUNDARY;

	/////

	return pPSuLM;
}

CRVLPSuLM * CRVLPSuLMBuilder::Load(FILE *fp)
{
	m_maxnLines = 0;

	CRVLPSuLM *pPSuLM = (CRVLPSuLM *)(m_pMem0->Alloc(sizeof(CRVLPSuLM)));

	memcpy(pPSuLM, &m_PSuLMTemplate, sizeof(CRVLPSuLM));

	RVLQLIST *pLocalMap = &(pPSuLM->m_LocalMap);
	RVLQLIST_INIT(pLocalMap)

	//Define index
	pPSuLM->m_Index = (WORD)(m_PSuLMList.m_nElements);

	m_PSuLMList.Add(pPSuLM);

	pPSuLM->m_SurfaceList.m_pMem = m_pMem0;
	pPSuLM->m_2DLineList.m_pMem = m_pMem0;
	pPSuLM->m_3DLineList.m_pMem = m_pMem0;

	pPSuLM->m_CellArray = (RVLPSULM_CELL *)(m_pMem0->Alloc(m_nCells * sizeof(RVLPSULM_CELL)));

	pPSuLM->Load(fp, m_Flags2);

	if(pPSuLM->m_nLandmarks > m_maxnLandmarks)
		m_maxnLandmarks = pPSuLM->m_nLandmarks;

	if(pPSuLM->m_n3DLines > m_maxnLines)
		m_maxnLines = pPSuLM->m_n3DLines;

	return pPSuLM;
}


int CRVLPSuLMBuilder::MatchLine(CRVL3DLine2 *pLine,
								CRVLPSuLM *pPSuLM,
								CRVL3DPose *pPose,
								BOOL bInvTransf, 
								int &visible,
								CRVLMem *pMemExternal)
{
	int iU1[2], iU2[2];
	CvPoint Pt1, Pt2;
	BYTE CropSide;
	double X1[3], X2[3];

	if(bInvTransf)
	{
		pPose->InvTransf(pLine->m_X[0], X1);
		pPose->InvTransf(pLine->m_X[1], X2);
	}
	else
	{
		pPose->Transf(pLine->m_X[0], X1);
		pPose->Transf(pLine->m_X[1], X2);
	}

	BYTE bOut = RVLCrop3DLine(X1, X2, m_pCamera, &m_ROI, 
			m_CropLTs.minz, m_CropLTs.minr, m_CropLTs.bOutLT, iU1, iU2, &Pt1, &Pt2, CropSide);

	if(bOut & 0x04)
	{
		visible = 0;

		return 0;
	}
	else 
		return MatchLine(&Pt1, &Pt2, pPSuLM, visible, pMemExternal);
}

// uses m_pMem2

int CRVLPSuLMBuilder::MatchLine(CvPoint *pPt1, CvPoint *pPt2,
								CRVLPSuLM *pPSuLM,
								int &visible,
								CRVLMem *pMemExternal)
{
	int u1 = pPt1->x;
	int v1 = pPt1->y;
	int du = pPt2->x - u1;
	int dv = pPt2->y - v1;
	int len2 = du * du + dv * dv;

	if(len2 == 0)
	{
		m_LineCoverage = NULL;

		visible = 0;

		return 0;
	}

	double flen2 = (double)(len2);
	double flen = sqrt(flen2);
	int len = DOUBLE2INT(flen);

	visible = len;

	LineSampling(pPt1, pPt2);	

	RVLPSULM_CELL *CellArray = pPSuLM->m_CellArray;

	int *piSampleCellArrayEnd = m_iSampleCellArray + m_nSampleCells;

	int nLines = pPSuLM->m_2DLineList.m_nElements;

	BYTE *pMem;

	if(pMemExternal)
	{
		m_MatchedLineBuff = (CRVL2DLine2 **)(pMemExternal->Alloc(nLines * sizeof(CRVL2DLine2 *)));

		m_LineCoverage = (RVLPSULM_LINE_COVERAGE_INTERVAL *)(pMemExternal->Alloc(
			nLines * sizeof(RVLPSULM_LINE_COVERAGE_INTERVAL)));
	}
	else
	{
		pMem = m_pMem2->m_pFreeMem;

		m_MatchedLineBuff = (CRVL2DLine2 **)(m_pMem2->Alloc(nLines * sizeof(CRVL2DLine2 *)));;

		m_LineCoverage = (RVLPSULM_LINE_COVERAGE_INTERVAL *)(m_pMem2->Alloc(
			nLines * sizeof(RVLPSULM_LINE_COVERAGE_INTERVAL)));
	}

	CRVL2DLine2 **ppLine = m_MatchedLineBuff;

	RVLPSULM_CELL *pCell;
	RVLPSULM_CELL_LINEPTR *pLinePtr;
	int *piCell;
	CRVL2DLine2 *pLine2;

	for(piCell = m_iSampleCellArray; piCell < piSampleCellArrayEnd; piCell++)
	{
		pCell = CellArray + (*piCell);

		pLinePtr = pCell->pLinePtr;

		while(pLinePtr)
		{
			pLine2 = pLinePtr->pLine;

			if((pLine2->m_Flags & RVLOBJ2_FLAG_MATCHED) == 0)
			{
				*(ppLine++) = pLine2;

				pLine2->m_Flags |= RVLOBJ2_FLAG_MATCHED;
			}

			pLinePtr = pLinePtr->pNext;
		}
	}

	RVLPSULM_LINE_COVERAGE_INTERVAL *pFirst = NULL;
	RVLPSULM_LINE_COVERAGE_INTERVAL **ppLast = &pFirst;
	RVLPSULM_LINE_COVERAGE_INTERVAL *pNewInterval = m_LineCoverage;

	m_LineCoverage->p1 = 0;
	m_LineCoverage->p2 = 0;
	m_LineCoverage->pNext = NULL;

	int uTolNrm = m_uTol * len;

	int dqTolNrm = m_dqTol * len;

	int csPhiTolNrm = DOUBLE2INT(m_csPhiTol * flen);

	int urel, vrel;
	int p1, q1, p2, q2;
	//int dq;
	//int p1c, q1c, p2c, q2c;
	double fp1, fq1, k;
	BOOL bFloat;
	int iTmp;
	RVLPSULM_LINE_COVERAGE_INTERVAL *pInterval;
	BOOL bAddNewInterval;
	RVLPSULM_LINE_COVERAGE_INTERVAL *pOpenedInterval;	
	int csPhi;

	CRVL2DLine2 **pLineBuffEnd = ppLine;

	for(ppLine = m_MatchedLineBuff; ppLine < pLineBuffEnd; ppLine++)
	{
		pLine2 = *ppLine;

		pLine2->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

		csPhi = pLine2->m_diU[0] * du + pLine2->m_diU[1] * dv;

		if(csPhi < 0)
			csPhi = -csPhi;

		if(csPhi < pLine2->m_leniU * csPhiTolNrm)
			continue;

		urel = pLine2->m_iU[0][0] - u1;
		vrel = pLine2->m_iU[0][1] - v1;

		p1 = urel * du + vrel * dv;
		q1 = -urel * dv + vrel * du;

		urel = pLine2->m_iU[1][0] - u1;
		vrel = pLine2->m_iU[1][1] - v1;

		p2 = urel * du + vrel * dv;
		q2 = -urel * dv + vrel * du;

		if(p1 > p2)
		{
			iTmp = p1;
			p1 = p2;
			p2 = iTmp;

			iTmp = q1;
			q1 = q2;
			q2 = iTmp;
		}
		
		if(p2 < 0)
			continue;

		if(p1 > len2)
			continue;
	
		if(p1 < 0)
		{
			fp1 = (double)p1;
			fq1 = (double)q1;
			k = (double)(q2 - q1) / (double)(p2 - p1);

			bFloat = TRUE;

			p1 = 0;

			q1 = DOUBLE2INT(fq1 - k * fp1);
		}
		else
			bFloat = FALSE;

		if(q1 < -uTolNrm)
			continue;
		
		if(q1 > uTolNrm)
			continue;

		if(p2 > len2)
		{
			if(!bFloat)
			{
				fp1 = (double)p1;
				fq1 = (double)q1;
				k = (double)(q2 - q1) / (double)(p2 - p1);
			}

			p2 = len2;

			q2 = DOUBLE2INT(fq1 + k * (flen2 - fp1));			
		}

		if(q2 < -uTolNrm)
			continue;
		
		if(q2 > uTolNrm)
			continue;

		//dq = q2 - q1;

		//if(dq < -dqTolNrm)
		//	continue;

		//if(dq > dqTolNrm)
		//	continue;

		pOpenedInterval = NULL;

		bAddNewInterval = TRUE;

		pInterval = pFirst;

		while(pInterval)
		{
			if(pOpenedInterval)		//  ( ... ] ? ) ?
			{
				if(p2 < pInterval->p1)		//  ( ... ] ) [ ...
				{
					pOpenedInterval->p2 = p2;
					pOpenedInterval->pNext = pInterval;

					pOpenedInterval = NULL;

					bAddNewInterval = FALSE;

					break;
				}

				if(p2 <= pInterval->p2)		//  ( ... ] [ ) ]
				{
					pOpenedInterval->p2 = pInterval->p2;
					pOpenedInterval->pNext = pInterval->pNext;

					pOpenedInterval = NULL;

					bAddNewInterval = FALSE;

					break;
				}
			}
			else if(p1 <= pInterval->p2)	//  ( ? ] ?
			{	
				if(p1 < pInterval->p1)		//  ( ? [ ? ] ?
				{
					if(p2 < pInterval->p1)	//  ( ) [ ]
						break;

					pInterval->p1 = p1;

					if(p2 <= pInterval->p2)	//   ( [ ) ]
					{
						bAddNewInterval = FALSE;
					
						break;
					}
					else					//   ( [ ] ... )
						pOpenedInterval = pInterval;
				}
				else						//  [ ( ? ] ?
				{
					bAddNewInterval = FALSE;

					if(p2 <= pInterval->p2)	//  [ ( ) ] 
						break;
					else					//  [ ( ] ... )
						pOpenedInterval = pInterval;
				}
			}

			pInterval = pInterval->pNext;
		}

		if(bAddNewInterval)
		{
			pNewInterval->p1 = p1;

			pNewInterval->p2 = p2;

			*ppLast = pNewInterval;

			pNewInterval->pNext = NULL;

			ppLast = &(pNewInterval->pNext);

			pNewInterval++;
		}
		else if(pOpenedInterval)
		{
			pOpenedInterval->p2 = p2;
			pOpenedInterval->pNext = NULL;
		}
	}

	int Coverage = 0;

	pInterval = pFirst;

	while(pInterval)
	{
		Coverage += (pInterval->p2 - pInterval->p1);

		pInterval = pInterval->pNext;
	}

	if(pMemExternal)
		pMemExternal->m_pFreeMem = (BYTE *)pNewInterval;
	else
		m_pMem2->m_pFreeMem = pMem;

	int cost = len - Coverage / len;

	return (cost >= 0 ? cost : 0);
}

int CRVLPSuLMBuilder::MatchSurface(	CRVL3DSurface2 *pSurface,
									CRVLPSuLM *pPSuLM,
									CRVL3DPose *pPose,
									BOOL bInvTransf, 
									CRVLMem *pMemExternal)
{
	CRVL3DPose PoseM0;
	
	int cost = 0;
	
	if(bInvTransf)
		InverseTransform3D(PoseM0.m_Rot, PoseM0.m_X, pPose->m_Rot, pPose->m_X);
	else
		PoseM0 = *pPose;
	

	CRVL3DPose Pose0M;
	CRVL3DPose PoseCM;
	CRVL3DPose PoseC0;

	PoseC0.Reset();

	InverseTransform3D(Pose0M.m_Rot, Pose0M.m_X, PoseM0.m_Rot, PoseM0.m_X);
	RVLCombineTransform3D(Pose0M.m_Rot, Pose0M.m_X, PoseC0.m_Rot, PoseC0.m_X, PoseCM.m_Rot, PoseCM.m_X);



	pPSuLM->Match(pSurface,&PoseCM);
	
	//if(bInvTransf)
	//{
	//	CRVL3DPose PoseT;
	//	InverseTransform3D(PoseT.m_Rot, PoseT.m_X, pPose->m_Rot, pPose->m_X);
	//	pPSuLM->Project(pSurface,&PoseT);
	//}
	//else
	//	pPSuLM->Project(pSurface,pPose);

	//Compare Builder->m_CellArray with  PSULM->m_CellArray
	//int i, iCell;
	//int disp1, disp2;

	//for(i=0;i<m_nSampleCells;i++)
	//{
	//	iCell = m_iSampleCellArray[i];
	//	disp1 = m_CellArray[iCell].disparity;
	//	disp2 = pPSuLM->m_CellArray[iCell].disparity;

	//	if((disp2 == -1) || (abs(disp1-disp2) < m_ProjDisparityErr))
	//		cost++;
	//}

	//for(i=0;i<m_nCells;i++)
	//{
	//	disp1 = m_CellArray[i].disparity;

	//	if(disp1 <= 0)
	//		continue;

	//	disp2 = pPSuLM->m_CellArray[i].disparity;

	//	if(abs(disp1-disp2) > m_ProjDisparityErr)
	//		cost++;
	//}
	
	return cost;
}

void CRVLPSuLMBuilder::CreateRotLG(double * NGroundPlane,
								   double *RotLG)
{
	RVLMXEL(RotLG, 3, 1, 0) = NGroundPlane[0];
	RVLMXEL(RotLG, 3, 1, 1) = NGroundPlane[1];
	RVLMXEL(RotLG, 3, 1, 2) = NGroundPlane[2];

	double fTmp = sqrt(NGroundPlane[0] * NGroundPlane[0] + NGroundPlane[1] * NGroundPlane[1]);

	RVLMXEL(RotLG, 3, 0, 0) = NGroundPlane[1] / fTmp;
	RVLMXEL(RotLG, 3, 0, 1) = -NGroundPlane[0] / fTmp;
	RVLMXEL(RotLG, 3, 0, 2) = 0.0;

	CrossProduct(RotLG, RotLG + 3 * 1, RotLG + 3 * 2);
}

// uses m_pMem2

void CRVLPSuLMBuilder::Hypotheses(CRVLPSuLM *pSPSuLM,
								  CRVL3DPose *pPoseS0Init)
{
	int nHypotheses = 200;

	double StartTime = m_pTimer->GetTime();

	if(m_PSuLMList.m_nElements == 0)
		return; 

	m_HypothesisList.RemoveAll();

#ifdef NEVER // old version
	int nSLandmarks = pSPSuLM->m_nLandmarks;

	double PositionUncert2 = m_PositionUncert * m_PositionUncert;

	double *ClusterArray = (double *)(m_pMem->Alloc(2 * m_maxnLandmarks * sizeof(double)));

	DWORD iHypothesis = 0;

	CRVLPSuLM *pMPSuLM;
	RVLPSULM_LANDMARK *pMLandmark, *pSLandmark;
	double yL, yH, yM;
	RVLPSULM_HYPOTHESIS *pHypothesis;
	double AlphaSM;
	double ca, sa;
	double xSM, zSM;
	//BYTE *pMem2;
	//int i,j;
	double V[2];
	double invCX[4];
	int nClusters;
	double *pCluster;
	double *pClusterArrayEnd;
	CRVL3DPose *pPoseSM;
	int iAlpha;

	m_PSuLMList.Start();

	while(m_PSuLMList.m_pNext)
	{
		pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());

		//if(pMPSuLM->m_Index == 187)
		//	int tmp1 = 0;

		for(iAlpha = -5; iAlpha <= 5; iAlpha++)
		{
			AlphaSM = (double)iAlpha * 5.0 * DEG2RAD;

			//AlphaSM = pMPSuLM->m_Alpha - pSPSuLM->m_Alpha;

			//if(AlphaSM > 0.5 * PI)
			//	AlphaSM -= PI;
			//else if(AlphaSM < -0.5 * PI)
			//	AlphaSM += PI;

			//if(AlphaSM > m_OrientUncert)
			//	continue;
			//else if(AlphaSM < -m_OrientUncert)
			//	continue;

			//pMem2 = m_pMem2->m_pFreeMem;

			m_HypothesisClustering.Reset(nSLandmarks * pMPSuLM->m_nLandmarks);

			pMLandmark = (RVLPSULM_LANDMARK *)(pMPSuLM->m_LandmarkList.pFirst);

			while(pMLandmark)
			{
				InverseMatrix2(invCX, pMLandmark->C, APPROX_ZERO);

				pSLandmark = (RVLPSULM_LANDMARK *)(pSPSuLM->m_LandmarkList.pFirst);

				while(pSLandmark)
				{
					//if(pSLandmark->bDown != pMLandmark->bDown)
					//	continue;
						
					yL = (pSLandmark->miny > pMLandmark->miny ? pSLandmark->miny : pMLandmark->miny);

					yH = (pSLandmark->maxy < pMLandmark->maxy ? pSLandmark->maxy : pMLandmark->maxy);

					yM = 0.5 * (pSLandmark->miny + pSLandmark->maxy);

					if(yM < yL)
						continue;

					if(yM > yH)
						continue;

					yM = 0.5 * (pMLandmark->miny + pMLandmark->maxy);

					if(yM < yL)
						continue;

					if(yM > yH)
						continue;

					ca = cos(AlphaSM);
					sa = sin(AlphaSM);

					xSM = pMLandmark->x - ca * pSLandmark->x - sa * pSLandmark->z;
					zSM = pMLandmark->z + sa * pSLandmark->x - ca * pSLandmark->z;

					if(xSM * xSM + zSM * zSM > PositionUncert2)
						continue;

					m_HypothesisClustering.Entry(xSM, zSM, invCX, V);

					pSLandmark = (RVLPSULM_LANDMARK *)(pSLandmark->pNext);
				}

				pMLandmark = (RVLPSULM_LANDMARK *)(pMLandmark->pNext);
			}

			m_pMem2->m_pFreeMem = (BYTE *)(m_HypothesisClustering.m_pAccuEntry);

			m_HypothesisClustering.GetClusters(ClusterArray, nClusters);
			
			pCluster = ClusterArray;

			pClusterArrayEnd = ClusterArray + 2 * nClusters;

			while(pCluster < pClusterArrayEnd)
			{
				RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);

				pPoseSM = &(pHypothesis->PoseSM);

				pPoseSM->m_X[0] = (*(pCluster++));
				pPoseSM->m_X[1] = 0.0;
				pPoseSM->m_X[2] = (*(pCluster++));
				pPoseSM->m_Alpha = AlphaSM;
				pPoseSM->m_Beta = pPoseSM->m_Theta = 0.0;
				pPoseSM->UpdateRotLL();
				pHypothesis->Index = (iHypothesis++);
				pHypothesis->cost = 0;
				pHypothesis->visible = 0;
				pHypothesis->iS = pHypothesis->iM = 0;
				pHypothesis->pMPSuLM = pMPSuLM;
				pHypothesis->bVisibility = 0;

				m_HypothesisList.Add(pHypothesis);
			}
		}
	}	
#endif	// old version

	CRVLMPtrChain *pS3DSurfaceList = &(pSPSuLM->m_SurfaceList);

	int nS3DSurfaces = pS3DSurfaceList->m_nElements;

	if(nS3DSurfaces == 0)
		return;

	RVLSURFACE_MATCH_ARRAY *InitMatchArray, *MatchArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSURFACE_MATCH_ARRAY, 2 * nS3DSurfaces, MatchArray);

	InitMatchArray = MatchArray + nS3DSurfaces;

	CRVL3DPose PoseSM, PoseSMInit;

	//**********************************

	PoseSMInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;

	double C[3 * 3 * 3], CInit[3 * 3 * 3];

	PoseSM.m_C = C;
	PoseSMInit.m_C = CInit;

	CRVLPSuLM *pMPSuLM;
	CRVL3DSurface2 *pS3DSurface, *pM3DSurface;
	CRVLMPtrChain *pM3DSurfaceList;
	RVLSURFACE_MATCH *MatchBuff, *InitMatchBuff;
	RVLSURFACE_MATCH *pMatch;
	RVLSURFACE_MATCH_ARRAY *pMatchArray;
	RVLSURFACE_MATCH_ARRAY *pLastMatchArray;
	double MatchQuality;
	int nMatches, n3DSceneSurfaces;
	int iHypothesis;
	BOOL bPoseUncertWithinTolerance;
	
	//define PoseSMInit properties
	PoseSMInit.Copy(pPoseS0Init);


	RVLPSULM_MSMATCH_DATA *MatrixSceneModel = new RVLPSULM_MSMATCH_DATA[pS3DSurfaceList->m_nElements * 1000];


	//3D Pose array / Hypothesis list
	int i, j, k, i0, j0, k0, iB, jB, kB;
	int iMax,jMax,kMax; 
	iMax = jMax = kMax = 40;
	
	double minPoseDistance = 100.0 * 100.0 * 100.0; //(10cm x 10cm x 10cm)

	double xDif, yDif, zDif;

	BOOL bPoseExists;

	CRVL3DPose *pPose;
	
	CRVL3DPose **PoseArray = new CRVL3DPose *[iMax * jMax *kMax];

	RVLPSULM_HYPOTHESIS *pHypothesis;




#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
	pSPSuLM->Display(m_DebugData.pGUI, pPoseS0Init);

	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("PSuLM");

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	//pMouseCallbackData->MatrixSceneModel = MatrixSceneModel;	
#endif

	
	int iScene,iModel;

	int nModels = 0;
	int nPassed = 0;

	double BestFit = 10000000;
		
	// feature matching 

	m_PSuLMList.Start();

	while(m_PSuLMList.m_pNext)
	{
		

		pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());

		pM3DSurfaceList = &(pMPSuLM->m_SurfaceList);
		memset(MatrixSceneModel, 0, pS3DSurfaceList->m_nElements * 1000 * sizeof(RVLPSULM_MSMATCH_DATA));

		//reset PoseArray
		memset(PoseArray,0,iMax*jMax*kMax * sizeof(CRVL3DPose *));

		nPassed = 0;

		// TSMInit = T0M * TS0

		//RVLCombineTransform3D(pMPSuLM->m_Pose0M.m_Rot, pMPSuLM->m_Pose0M.m_X, pPoseS0Init->m_Rot, pPoseS0Init->m_X,
		//	PoseSMInit.m_Rot, PoseSMInit.m_X);

		// compute PoseSMInit.m_C as well as Rotation Matrix !!!!!! UpdateRotLL
		//pPoseS0Init->SetUncert(&testPose);

		// compute pose information contributions of the model surfaces (this should be made upon creation of the surfaces)

		pM3DSurfaceList->Start();

		while(pM3DSurfaceList->m_pNext)
		{
			pM3DSurface = (CRVL3DSurface2 *)(pM3DSurfaceList->GetNext());

			//if((int)pM3DSurface == 0x019c90b5)
			//	int debug = 0;

			pM3DSurface->GetPoseContribution();
		}
		
		// create InitMatchArray

		memset(MatchArray, 0, nS3DSurfaces * sizeof(RVLSURFACE_MATCH_ARRAY));

		pMatchArray = MatchArray;

		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSURFACE_MATCH, nS3DSurfaces * pM3DSurfaceList->m_nElements, MatchBuff);

		pMatch = MatchBuff;

		StartTime = m_pTimer->GetTime();

		pS3DSurfaceList->Start();
		iScene = 0;

		while(pS3DSurfaceList->m_pNext)
		{
			pS3DSurface = (CRVL3DSurface2 *)(pS3DSurfaceList->GetNext());

			if(pS3DSurface->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			pS3DSurface->m_Index = iScene;

			pS3DSurface->GetPoseContribution();

			pMatchArray->pSObject = pS3DSurface;

			iModel = 0;
			nModels = 0;

			pM3DSurfaceList->Start();

			while(pM3DSurfaceList->m_pNext)
			{
				pM3DSurface = (CRVL3DSurface2 *)(pM3DSurfaceList->GetNext());

				if(pS3DSurface->Match(pM3DSurface, pPoseS0Init, MatchQuality, pMatchArray))
				{
					pMatch->vpSObject = MatchArray;
					pMatch->pMObject = pM3DSurface;
					
					if(pMatchArray->pFirst == NULL)
						pMatchArray->pFirst = pMatch;

					pMatch++;

					nModels++;
					//Create match matrix
					MatrixSceneModel[iScene * pM3DSurfaceList->m_nElements + iModel].bInitGeomConstrSattisfied = 1;

				}

				iModel++;
			}
			
			pMatchArray->pLast = pMatch - 1;

			if(nModels>0)  //ie update pMatchArray only if it has been matched to a model 
			{
				pMatchArray++;
				iScene++;
			}

		}

		double ExecutionTime = m_pTimer->GetTime() - StartTime;

		nMatches = pMatch - MatchBuff;
		n3DSceneSurfaces = pMatchArray - MatchArray;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
		pMPSuLM->AddToFigure(m_DebugData.pGUI);

		pMouseCallbackData->nMSurfs = pM3DSurfaceList->m_nElements;

		cvWaitKey();
#endif

#ifdef PYTHON_DEBUG
		PythonDisplayScene(MatchArray,pM3DSurfaceList,MatrixSceneModel,n3DSceneSurfaces);
#endif
		m_pMem2->m_pFreeMem = (BYTE *)pMatch;

		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSURFACE_MATCH, nMatches, InitMatchBuff);

		memcpy(InitMatchArray, MatchArray, n3DSceneSurfaces * sizeof(RVLSURFACE_MATCH_ARRAY));

		memcpy(InitMatchBuff, MatchBuff, nMatches * sizeof(RVLSURFACE_MATCH));

		// generate hypotheses

		for(iHypothesis = 0; iHypothesis < nHypotheses; iHypothesis++)
		{
			// initialize MatchArray

			if(iHypothesis > 0)
			{
				memcpy(MatchArray, InitMatchArray, n3DSceneSurfaces * sizeof(RVLSURFACE_MATCH_ARRAY));

				memcpy(MatchBuff, InitMatchBuff, nMatches * sizeof(RVLSURFACE_MATCH));
			}

			pLastMatchArray = MatchArray + n3DSceneSurfaces - 1;

			// initialize PoseSM
			PoseSM.Copy(&PoseSMInit); 

			//*** generate hypothesis by geometrically constrained random sampling

			bPoseUncertWithinTolerance = FALSE;

			while(pLastMatchArray > MatchArray && !bPoseUncertWithinTolerance)
			{
				// random match selection

				pMatchArray = MatchArray + RVLRandom(0, pLastMatchArray - MatchArray);

				for(pMatch = pMatchArray->pFirst; pMatch <= pMatchArray->pLast; pMatch++)
					if(pMatch->pMObject->m_Flags & RVLOBJ2_FLAG_MATCHED)
					{
						*pMatch = *(pMatchArray->pLast);

						pMatchArray->pLast--;
					}
				
				if(pMatchArray->pLast < pMatchArray->pFirst)
				{
					*pMatchArray = *pLastMatchArray;

					pLastMatchArray--;

					continue;
				}
				else if(pMatchArray->pLast == pMatchArray->pFirst)
					pMatch = pMatchArray->pFirst;
				else
					pMatch = pMatchArray->pFirst + RVLRandom(0, pMatchArray->pLast - pMatchArray->pFirst);

				pS3DSurface = (CRVL3DSurface2 *)(pMatchArray->pSObject);

				pM3DSurface = (CRVL3DSurface2 *)(pMatch->pMObject);

				// check geometrical constraints

				if(pS3DSurface->Match(pM3DSurface, &PoseSM, MatchQuality,pMatchArray ))
				{
					// Update PoseSM using EKF method ... add your code here!
					PoseSM.PlanarSurfaceEKFUpdate(pMatchArray->m_C,pMatchArray->m_Q,pMatchArray->m_e);

//#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
//					pMouseCallbackData->pSSurf = pS3DSurface;
//					pMouseCallbackData->pMSurf = pM3DSurface;
//
//					memcpy(pMouseCallbackData->PoseS0.m_Rot, PoseSM.m_Rot, 3 * 3 * sizeof(double));
//					memcpy(pMouseCallbackData->PoseS0.m_X, PoseSM.m_X, 3 * sizeof(double));
//
//					pMouseCallbackData->Flags &= ~RVLPSULM_DISPLAY_SELECTION;
//
//					RVLPSuLMDisplay(pFig);
//
//					cvWaitKey();
//#endif

					// remove pS3DSurface from further processing

					*pMatchArray = *pLastMatchArray;

					pLastMatchArray--;

					// remove pM3DSurface from further processing

					pM3DSurface->m_Flags |= RVLOBJ2_FLAG_MATCHED;
				}
				else
				{
					*pMatch = *(pMatchArray->pLast);

					pMatchArray->pLast--;					
				}

				bPoseUncertWithinTolerance =	(PoseSM.m_C[2 * 9 + 0 * 3 + 0] <= m_maxPoseSMUncert &&
												PoseSM.m_C[2 * 9 + 1 * 3 + 1] <= m_maxPoseSMUncert &&
												PoseSM.m_C[2 * 9 + 2 * 3 + 2] <= m_maxPoseSMUncert);
			}

			double Fit = PoseSM.m_C[2 * 9 + 0 * 3 + 0] + PoseSM.m_C[2 * 9 + 1 * 3 + 1] + PoseSM.m_C[2 * 9 + 2 * 3 + 2];

			if(Fit < BestFit)
			{
				BestFit = Fit;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
				pMouseCallbackData->pSSurf = NULL;
				pMouseCallbackData->pMSurf = NULL;

				memcpy(pMouseCallbackData->PoseS0.m_Rot, PoseSM.m_Rot, 3 * 3 * sizeof(double));
				memcpy(pMouseCallbackData->PoseS0.m_X, PoseSM.m_X, 3 * sizeof(double));

				pMouseCallbackData->Flags &= ~RVLPSULM_DISPLAY_SELECTION;

				RVLPSuLMDisplay(pFig);

				cvWaitKey();
#endif				
			}

			if(bPoseUncertWithinTolerance)
			{
#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
				pMouseCallbackData->pSSurf = NULL;
				pMouseCallbackData->pMSurf = NULL;

				memcpy(pMouseCallbackData->PoseS0.m_Rot, PoseSM.m_Rot, 3 * 3 * sizeof(double));
				memcpy(pMouseCallbackData->PoseS0.m_X, PoseSM.m_X, 3 * sizeof(double));

				pMouseCallbackData->Flags &= ~RVLPSULM_DISPLAY_SELECTION;

				RVLPSuLMDisplay(pFig);

				cvWaitKey();
#endif				
				//int g=0;
				// check if the similar pose already exists
				nPassed++;
				bPoseExists = FALSE;
				i0 = floor((PoseSM.m_X[0] - 50)/100) + 20;  //The center of the cube is the origin
				j0 = floor((PoseSM.m_X[1] - 50)/100) + 20;
				k0 = floor((PoseSM.m_X[2] - 50)/100) + 20;

				if (i0<iMax && j0<jMax && k0<kMax)
				{

					if(i0 < 0)
						i0 = 0;
					if(i0 > iMax-1)
						i0 = iMax - 1;

					if(j0 < 0)
						j0 = 0;
					if(j0 > jMax-1)
						j0 = jMax - 1;

					if(k0 < 0)
						k0 = 0;
					if(k0 > kMax-1)
						k0 = kMax - 1;


					kB = k0 < kMax-1 ? k0+1 : kMax-1;
					jB = j0 < jMax-1 ? j0+1 : jMax-1;
					iB = i0 < iMax-1 ? i0+1 : iMax-1;

					for(k=k0;k<=kB;k++)
					{
						for(i=i0;i<=iB;i++)
						{				
							for(j=j0;j<=jB;j++)
							{
								pPose = PoseArray[k*iMax*jMax + i*jMax + j];
								if(pPose != 0) 
								{
									xDif = pPose->m_X[0] - PoseSM.m_X[0];
									yDif = pPose->m_X[1] - PoseSM.m_X[1];
									zDif = pPose->m_X[2] - PoseSM.m_X[2];
									
									if(xDif*xDif + yDif*yDif + zDif*zDif < minPoseDistance)
									{
										bPoseExists = TRUE;
										break;
									}
								}
							}
							if(bPoseExists)
								break;
						}

						if(bPoseExists)
							break;
					}


					// if not, create hypothesis and insert it into m_HypothesisList
					if(!bPoseExists)
					{
						


						RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);

						pPose = &(pHypothesis->PoseSM);

						//Store/copy values
						for(i=0;i<3;i++)
							pPose->m_X[i] = PoseSM.m_X[i];
						for(i=0;i<9;i++)
							pPose->m_Rot[i] = PoseSM.m_Rot[i];

						pPose->m_Alpha = PoseSM.m_Alpha;
						pPose->m_Beta = PoseSM.m_Beta;
						pPose->m_Theta = PoseSM.m_Theta;
						pHypothesis->Index = k0*iMax*jMax + i0*jMax + j0; //iHypothesis;
						pHypothesis->cost = 0;
						pHypothesis->visible = 0;
						pHypothesis->iS = pHypothesis->iM = 0;
						pHypothesis->pMPSuLM = pMPSuLM;
						pHypothesis->Flags = 0x00;

						m_HypothesisList.Add(pHypothesis);

						//store in PoseArray
						PoseArray[k0*iMax*jMax + i0*jMax + j0] = pPose;

					
					}

				}
				
			}


			// reset all RVLOBJ2_FLAG_MATCHED flags 
			pM3DSurfaceList->Start();
			while(pM3DSurfaceList->m_pNext)
			{
				pM3DSurface = (CRVL3DSurface2 *)(pM3DSurfaceList->GetNext());
				pM3DSurface->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;
			}

		}	// for(iHypothesis = 0; iHypothesis < nHypotheses; iHypothesis++)

#ifdef PYTHON_DEBUG
			//Save PoseArray
			CString RezFile = "D:\\Phd\\Program\\PythonScripts\\PoseArray.dat";
			FILE *fRezFile = fopen(RezFile, "w");
			int m;
			if(fRezFile)
			{

				//nHypothesis
				fprintf(fRezFile,"%4d\n",m_HypothesisList.m_nElements);

				////PoseArray coordinates
				//for(i=0;i<iMax*jMax*kMax;i++)
				//{
				//	if(PoseArray[i] != 0)
				//	{
				//		//get coordinates
				//		k0 = i / (iMax*jMax);
				//		
				//		m = i % (iMax*jMax);
				//		i0 = m / jMax;
				//		j0 = m % jMax;

				//		fprintf(fRezFile,"%4d\t%4d\t%4d\n",i0,j0,k0);
				//	
				//	}
				//}

				m_HypothesisList.Start();

				while(m_HypothesisList.m_pNext)
				{
					pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());
					//get coordinates
					k0 = pHypothesis->Index / (iMax*jMax);
					
					m = pHypothesis->Index % (iMax*jMax);
					i0 = m / jMax;
					j0 = m % jMax;
					fprintf(fRezFile,"%4d\t%4d\t%4d\n",i0,j0,k0);
				}


				m_HypothesisList.Start();

				while(m_HypothesisList.m_pNext)
				{
					pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());
					fprintf(fRezFile,"%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\n",pHypothesis->PoseSM.m_X[0],pHypothesis->PoseSM.m_X[1],pHypothesis->PoseSM.m_X[2],pHypothesis->PoseSM.m_Alpha,pHypothesis->PoseSM.m_Beta,pHypothesis->PoseSM.m_Theta);
				}
				fclose(fRezFile);
			}
#endif
		// free match buffer memory

		m_pMem2->m_pFreeMem = (BYTE *)InitMatchBuff;
	}	// for every PSuLM

	double ExecutionTime = m_pTimer->GetTime() - StartTime;


	delete[] MatrixSceneModel;
	delete[] PoseArray;

}



#ifdef NEVER	// old version

void CRVLPSuLMBuilder::Hypotheses2(	CRVLPSuLM *pSPSuLM,
									CRVL3DPose *pPoseS0Init)
{
	int nHypotheses = 200;

	int maxnM3DSurfaces = 1000;

	int maxnSamples = 5;

	double StartTime = m_pTimer->GetTime();

	if(m_PSuLMList.m_nElements == 0)
		return; 

	m_HypothesisList.RemoveAll();

	CRVLMPtrChain *pS3DSurfaceList = &(pSPSuLM->m_SurfaceList);

	int nS3DSurfaces = pS3DSurfaceList->m_nElements;

	if(nS3DSurfaces == 0)
		return;

	// put the ptrs. of all valid scene surfaces into SSurfArray

	CRVL3DSurface2 **SSurfArray = new CRVL3DSurface2 *[nS3DSurfaces];
	CRVL3DSurface2 **ppSSurf = SSurfArray;

	CRVL3DSurface2 *pS3DSurface;

	pS3DSurfaceList->Start();

	while(pS3DSurfaceList->m_pNext)
	{
		pS3DSurface = (CRVL3DSurface2 *)(pS3DSurfaceList->GetNext());

		if(pS3DSurface->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pS3DSurface->GetPoseContribution();
		
		*(ppSSurf++) = pS3DSurface;
	}
	
	CRVL3DSurface2 **pSSurfArrayEnd = ppSSurf;

	nS3DSurfaces = pSSurfArrayEnd - SSurfArray;

	// allocate memory for MSurfArray

	CRVL3DSurface2 **MSurfArray = new CRVL3DSurface2 *[maxnM3DSurfaces];

	// allocate MatchArray

	RVLSURFACE_MATCH_ARRAY *InitMatchArray, *MatchArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSURFACE_MATCH_ARRAY, 2 * nS3DSurfaces, MatchArray);

	InitMatchArray = MatchArray + nS3DSurfaces;

	//**********************************

	CRVL3DPose PoseSM, PoseSMInit;

	PoseSMInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;

	double C[3 * 3 * 3], CInit[3 * 3 * 3];

	PoseSM.m_C = C;
	PoseSMInit.m_C = CInit;

	//define PoseSMInit properties
	PoseSMInit.Copy(pPoseS0Init);

	//PoseSMInit.m_C[2 * 3 * 3 + 0] = 1e4;
	//PoseSMInit.m_C[2 * 3 * 3 + 4] = 1.1e4;
	//PoseSMInit.m_C[2 * 3 * 3 + 8] = 1.2e4;

	double YInit[2 * 3 * 3];

	double *YRInit = YInit;
	double *YtInit = YInit + 3 * 3;

	InverseMatrix3(YRInit, PoseSMInit.m_C);
	InverseMatrix3(YtInit, PoseSMInit.m_C + 2 * 3 * 3);

	CRVLPSuLM *pMPSuLM;
	CRVL3DSurface2 *pM3DSurface;
	CRVL3DSurface2 **ppMSurf;
	CRVLMPtrChain *pM3DSurfaceList;
	//RVLSURFACE_MATCH *MatchBuff, *InitMatchBuff;
	//RVLSURFACE_MATCH *pMatch;
	//RVLSURFACE_MATCH_ARRAY *pLastMatchArray;
	//int nMatches;
	//BOOL bPoseUncertWithinTolerance;
	int iSample;
	int iHypothesis;
	double MatchQuality;

	RVLSURFACE_MATCH_ARRAY *pMatchArray = MatchArray;
	
	RVLPSULM_MSMATCH_DATA *MatrixSceneModel = new RVLPSULM_MSMATCH_DATA[nS3DSurfaces * maxnM3DSurfaces];

	RVLPSULM_SSURF_MATCH_DATA *SSurfMatchData = new RVLPSULM_SSURF_MATCH_DATA[nS3DSurfaces];

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
	pSPSuLM->Display(m_DebugData.pGUI, pPoseS0Init);

	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("PSuLM");

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	pMouseCallbackData->MatrixSceneModel = MatrixSceneModel;	
#endif

	int nModels = 0;

	double BestFit = 10000000;

	double Y[2 * 3 * 3];

	double *YR = Y;
	double *Yt = Y + 3 * 3;

	double Y0[2 * 3 * 3];

	double *YR0 = Y0;
	double *Yt0 = Y0 + 3 * 3;

	RVLPSULM_MSMATCH_DATA **HypothesisMatchArray = new RVLPSULM_MSMATCH_DATA *[maxnSamples];

	RVLPSULM_MSMATCH_DATA **OutOfGeomConstraintsArray = new RVLPSULM_MSMATCH_DATA *[nS3DSurfaces * maxnM3DSurfaces];

	//int iScene,iModel;
	double *YSR, *YSt;
	BOOL bReal[3];
	double eigYR[3], eigYt[3], eigYR0[3], eigYt0[3];
	double util, maxUtil;
	int iS3DSurface;
	RVLPSULM_MSMATCH_DATA *pMSMatch, *pMSMatch0;
	double Area1, Area2;
	double Probability, maxProbability;
	RVLPSULM_MSMATCH_DATA *pBestMatchCandidate;
	RVLPSULM_SSURF_MATCH_DATA *pSSurfMatchData;
	int iBestS3DSurface;
	int nM3DSurfaces;
	RVLPSULM_MSMATCH_DATA **ppOutOfGeomConstraints;
	RVLPSULM_MSMATCH_DATA **pOutOfGeomConstraintsArrayEnd;
	CRVL3DSurface2 **pMSurfArrayEnd;
	double r;
	int nFailures, nSamples;
	
	// feature matching 

	m_PSuLMList.Start();

	while(m_PSuLMList.m_pNext)
	{
		pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());

		pM3DSurfaceList = &(pMPSuLM->m_SurfaceList);

		ppMSurf = MSurfArray;

		nM3DSurfaces = pM3DSurfaceList->m_nElements;

		pM3DSurfaceList->Start();

		while(pM3DSurfaceList->m_pNext)
		{
			pM3DSurface = (CRVL3DSurface2 *)(pM3DSurfaceList->GetNext());

			*(ppMSurf++) = pM3DSurface;
		}

		// compute probabilities of surface matches considering the supports of the matched surfaces

		pMSMatch0 = MatrixSceneModel;

		for(iS3DSurface = 0; iS3DSurface < nS3DSurfaces; iS3DSurface++)
		{
			pS3DSurface = SSurfArray[iS3DSurface];

			maxProbability = -1e20;

			pMSurfArrayEnd = MSurfArray + nM3DSurfaces;

			pMSMatch = pMSMatch0;

			for(ppMSurf = MSurfArray; ppMSurf < pMSurfArrayEnd; ppMSurf++, pMSMatch++)
			{
				pM3DSurface = *ppMSurf;

				if(pS3DSurface->m_nSupport >= pM3DSurface->m_nSupport)
				{
					Area1 = pS3DSurface->m_Area;
					Area2 = pM3DSurface->m_Area;
				}
				else
				{
					Area1 = pM3DSurface->m_Area;
					Area2 = pS3DSurface->m_Area;
				}

				r = (Area1 - Area2) / Area1;

				Probability = log(1.0/(1.0+20.0*r*r*r*r));

				if(Probability > maxProbability)
				{
					maxProbability = Probability;
					pBestMatchCandidate = pMSMatch;
				}

				pMSMatch->pM3DSurface = pM3DSurface;
				pMSMatch->Probability = Probability;
				pMSMatch->Flags = 0x00;
			}

			pMSMatch0 += maxnM3DSurfaces;

			SSurfMatchData[iS3DSurface].maxMatchProbability = maxProbability;
			SSurfMatchData[iS3DSurface].pMatch = pBestMatchCandidate;
		}

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
		pMPSuLM->AddToFigure(m_DebugData.pGUI);

		pMouseCallbackData->nMSurfs = pM3DSurfaceList->m_nElements;

		cvWaitKey();
#endif
			
		// generate hypotheses

		for(iHypothesis = 0; iHypothesis < nHypotheses; iHypothesis++)
		{
			ppOutOfGeomConstraints = OutOfGeomConstraintsArray;

			// initialize PoseSM

			PoseSM.Copy(&PoseSMInit); 

			RVLCOPYMX3X3(YRInit, YR0)
			RVLCOPYMX3X3(YtInit, Yt0)

			for(iSample = 0; iSample < maxnSamples; iSample++)
			{
				// compute utility value of each scene surface

				RVLEig3(YR0, eigYR0, bReal);

				RVLEig3(Yt0, eigYt0, bReal);

				for(iS3DSurface = 0; iS3DSurface < nS3DSurfaces; iS3DSurface++)
				{
					pS3DSurface = SSurfArray[iS3DSurface];

					if(SSurfArray[iS3DSurface]->m_Flags & RVLOBJ2_FLAG_MATCHED)
						continue;

					YSR = pS3DSurface->m_PoseContribution;
					YSt = pS3DSurface->m_PoseContribution + 3 * 3;

					RVLSUMMX3X3(YR0, YSR, YR)
					RVLSUMMX3X3(Yt0, YSt, Yt)

					RVLEig3(YR, eigYR, bReal);

					RVLEig3(Yt, eigYt, bReal);

					SSurfMatchData[iS3DSurface].util = 
						log((eigYR[0]/eigYR0[0])*(eigYR[1]/eigYR0[1])*(eigYR[2]/eigYR0[2])*
						(eigYt[0]/eigYt0[0])*(eigYt[1]/eigYt0[1])*(eigYt[2]/eigYt0[2])) / 6.0;
				}

				// find next match

				nFailures = 0;

				while(nFailures < 20)
				{
					maxUtil = -1e20;

					for(iS3DSurface = 0; iS3DSurface < nS3DSurfaces; iS3DSurface++)
					{
						if(SSurfArray[iS3DSurface]->m_Flags & RVLOBJ2_FLAG_MATCHED)
							continue;

						pSSurfMatchData = SSurfMatchData + iS3DSurface;

						util = pSSurfMatchData->util + pSSurfMatchData->maxMatchProbability;

						if(util > maxUtil)
						{
							maxUtil = util;
							iBestS3DSurface = iS3DSurface;
							pMSMatch = pSSurfMatchData->pMatch;
						}
					}

					pS3DSurface = SSurfArray[iBestS3DSurface];

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
					pMouseCallbackData->pSSurf = pS3DSurface;
					pMouseCallbackData->pMSurf = pMSMatch->pM3DSurface;

					memcpy(pMouseCallbackData->PoseS0.m_Rot, PoseSM.m_Rot, 3 * 3 * sizeof(double));
					memcpy(pMouseCallbackData->PoseS0.m_X, PoseSM.m_X, 3 * sizeof(double));

					RVLPSuLMDisplay(pFig);

					cvWaitKey();
#endif

					if((pMSMatch->Flags & RVLPSULM_MSMATCH_FLAG_INIT_GEOM_CONSTR_TESTED) == 0)
					{
						pMSMatch->Flags |= RVLPSULM_MSMATCH_FLAG_INIT_GEOM_CONSTR_TESTED;

						if(!pS3DSurface->Match(pMSMatch->pM3DSurface, &PoseSMInit, MatchQuality, pMatchArray))
						{
							pMSMatch->Flags |= RVLPSULM_MSMATCH_FLAG_INIT_GEOM_CONSTR_NOT_SATISFIED;

							GetMaxProbabilityMatch(iBestS3DSurface, maxnM3DSurfaces, nM3DSurfaces, MatrixSceneModel, 
								SSurfMatchData);
						}
					}

					if((pMSMatch->Flags & RVLPSULM_MSMATCH_FLAG_INIT_GEOM_CONSTR_NOT_SATISFIED) == 0)
					{
						if(pS3DSurface->Match(pMSMatch->pM3DSurface, &PoseSM, MatchQuality, pMatchArray))
						{
							PoseSM.PlanarSurfaceEKFUpdate(pMatchArray->m_C,pMatchArray->m_Q,pMatchArray->m_e);

							HypothesisMatchArray[iSample] = pMSMatch;

							pMSMatch->pM3DSurface->m_Flags |= RVLOBJ2_FLAG_MATCHED;

							pS3DSurface->m_Flags |= RVLOBJ2_FLAG_MATCHED;

							break;
						}
						else
						{
							pMSMatch->Flags |= RVLPSULM_MSMATCH_FLAG_GEOM_CONSTR_NOT_SATISFIED;

							*(ppOutOfGeomConstraints++) = pMSMatch;

							GetMaxProbabilityMatch(iBestS3DSurface, maxnM3DSurfaces, nM3DSurfaces, MatrixSceneModel, 
								SSurfMatchData);
						}
					}

					nFailures++;
				}	// find next match loop

				if(nFailures >= 20)
					break;

				iS3DSurface = (pMSMatch - MatrixSceneModel) / maxnM3DSurfaces;

				pS3DSurface = SSurfArray[iS3DSurface];

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
				pMSMatch = HypothesisMatchArray[iSample];

				pMouseCallbackData->pSSurf = pS3DSurface;
				pMouseCallbackData->pMSurf = pMSMatch->pM3DSurface;

				memcpy(pMouseCallbackData->PoseS0.m_Rot, PoseSM.m_Rot, 3 * 3 * sizeof(double));
				memcpy(pMouseCallbackData->PoseS0.m_X, PoseSM.m_X, 3 * sizeof(double));

				RVLPSuLMDisplay(pFig);

				cvWaitKey();
#endif

				YSR = pS3DSurface->m_PoseContribution;
				YSt = pS3DSurface->m_PoseContribution + 3 * 3;

				RVLSUMMX3X3(YR0, YSR, YR0)
				RVLSUMMX3X3(Yt0, YSt, Yt0)
			}	// hypothesis generation loop

			nSamples = iSample;

			// reset out of geom. constraints flag

			pOutOfGeomConstraintsArrayEnd = ppOutOfGeomConstraints;

			for(ppOutOfGeomConstraints = OutOfGeomConstraintsArray; ppOutOfGeomConstraints < pOutOfGeomConstraintsArrayEnd;
				ppOutOfGeomConstraints++)
			{
				pMSMatch = *ppOutOfGeomConstraints;

				pMSMatch->Flags &= ~RVLPSULM_MSMATCH_FLAG_GEOM_CONSTR_NOT_SATISFIED;
			}

			// update match probabilities

			for(iSample = 0; iSample < nSamples; iSample++)
			{
				pMSMatch = HypothesisMatchArray[iSample];

				pMSMatch->Probability -= 0.05;

				iS3DSurface = (pMSMatch - MatrixSceneModel) / maxnM3DSurfaces;

				GetMaxProbabilityMatch(iS3DSurface, maxnM3DSurfaces, nM3DSurfaces, MatrixSceneModel, 
					SSurfMatchData);				

				SSurfArray[iS3DSurface]->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

				pMSMatch->pM3DSurface->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;
			}
		}	// for each hypothesis
	}	// for each model PSuLM	

	double ExecutionTime = m_pTimer->GetTime() - StartTime;

	delete[] MatrixSceneModel;
	//delete[] UtilFnArray;
	delete[] SSurfArray;
	delete[] MSurfArray;
	delete[] SSurfMatchData;
	delete[] HypothesisMatchArray;
	delete[] OutOfGeomConstraintsArray;
}


#endif

void CRVLPSuLMBuilder::Hypotheses2(	CRVLPSuLM *pSPSuLM,
									CRVL3DPose *pPoseS0Init)
{
	int nHypotheses = 20;

	int maxnM3DSurfaces = 1000;

	int maxnSamples = 5;

	int maxnFailures = 20;

	if(m_PSuLMList.m_nElements == 0)
		return; 

	m_HypothesisList.RemoveAll();

	//CRVLMPtrChain *pS3DSurfaceList = &(pSPSuLM->m_SurfaceList);

	int nS3DSurfaces = pSPSuLM->m_n3DSurfaces;

	if(nS3DSurfaces == 0)
		return;

	// put the ptrs. of all valid scene surfaces into SSurfArray

	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;
	CRVL3DSurface2 **ppSSurf = SSurfArray;

	RVLPSULM_SURF_MATCH_DATA *SSurfMatchData = new RVLPSULM_SURF_MATCH_DATA[nS3DSurfaces];

	RVLPSULM_SURF_MATCH_DATA *pSSurfMatchData = SSurfMatchData;

	CRVL3DSurface2 *pS3DSurface;

	int i;

	//CString debugFile = "D:\\Phd\\Program\\PythonScripts\\DebugFile2.dat";
	//FILE *fDebugFile = fopen(debugFile, "w");
	//fprintf(fDebugFile,"Index\tSupport\tnConvexSegments\tArea\tSigmaR\tX[0]\tX[1]\tX[2]\tN[0]\tN[1]\tN[2]\trho\trho_calc\n");
	//pS3DSurfaceList->Start();

	//while(pS3DSurfaceList->m_pNext)
	for(i=0;i<nS3DSurfaces;i++)
	{
		pS3DSurface = SSurfArray[i];

		//pS3DSurface = (CRVL3DSurface2 *)(pS3DSurfaceList->GetNext());

		//if(pS3DSurface->m_Flags & RVLOBJ2_FLAG_REJECTED)
		//	continue;

		//pS3DSurface->m_Index = i;

		//iS3DSurface++;

		//// this should be moved to the function which creates surfaces

		//pS3DSurface = SSurfArray[i];

		//pS3DSurface->m_Index = i;

		//pS3DSurface->GetPoseContribution();


		//if(pS3DSurface->m_nSupport == 34)
		//		int gg = 90;

		//RVLARRAY *pRelList = pS3DSurface->m_RelList + pS3DSurface->m_pClass->m_iRelListComponents;
		//CRVL3DSurface2 *p3DConvexSegment = *((CRVL3DSurface2 **)pRelList->pFirst);


		//fprintf(fDebugFile,"%4d\t%6d\t%6d\t%8.3lf\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\t%6.3lf\n",
		//	pS3DSurface->m_Index,pS3DSurface->m_nSupport,((CRVL3DSurface2 **)pRelList->pEnd - (CRVL3DSurface2 **)pRelList->pFirst), pS3DSurface->m_Area, p3DConvexSegment->m_sigmaR,
		//		p3DConvexSegment->m_Pose.m_X[0],p3DConvexSegment->m_Pose.m_X[1],p3DConvexSegment->m_Pose.m_X[2],
		//		p3DConvexSegment->m_N[0],p3DConvexSegment->m_N[1],p3DConvexSegment->m_N[2],
		//		pS3DSurface->m_d,RVLDOTPRODUCT3(p3DConvexSegment->m_Pose.m_X, p3DConvexSegment->m_N));

		//p3DConvexSegment->m_varq[0] =  p3DConvexSegment->m_sigmaR / (p3DConvexSegment->m_EigenValues[0] * p3DConvexSegment->m_EigenValues[0] + p3DConvexSegment->m_sigmaR);
		//p3DConvexSegment->m_varq[1] =  p3DConvexSegment->m_sigmaR / (p3DConvexSegment->m_EigenValues[1] * p3DConvexSegment->m_EigenValues[1] + p3DConvexSegment->m_sigmaR);
		//p3DConvexSegment->m_varq[2] =  p3DConvexSegment->m_sigmaR;
		//p3DConvexSegment->m_N[0] = p3DConvexSegment->m_Pose.m_Rot[2];
		//p3DConvexSegment->m_N[1] = p3DConvexSegment->m_Pose.m_Rot[5];
		//p3DConvexSegment->m_N[2] = p3DConvexSegment->m_Pose.m_Rot[8];
		//p3DConvexSegment->m_d = RVLDOTPRODUCT3(p3DConvexSegment->m_Pose.m_X, p3DConvexSegment->m_N);

		///////
		//
		//*(ppSSurf++) = pS3DSurface;

		pSSurfMatchData->p3DSurface = pS3DSurface;

		pSSurfMatchData->bMatched = 0;

		pSSurfMatchData++;
	}

	//fclose(fDebugFile);
	
	//CRVL3DSurface2 **pSSurfArrayEnd = SSurfArray + nS3DSurfaces;

	RVLPSULM_SURF_MATCH_DATA *pSSurfMatchDataArrayEnd = pSSurfMatchData;

	//nS3DSurfaces = pSSurfArrayEnd - SSurfArray;

	// allocate memory for MSurfArray

	CRVL3DSurface2 **MSurfArray = new CRVL3DSurface2 *[maxnM3DSurfaces];

	//**********************************

	CRVL3DPose PoseSM, PoseSMInit;

	PoseSMInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	double CInit[3 * 3 * 3];
	double invtInit[3];
	PoseSMInit.m_C = CInit;
	PoseSMInit.Copy(pPoseS0Init);
	double *R = PoseSMInit.m_Rot;
	double *t = PoseSMInit.m_X;
	RVLMULMX3X3TVECT(R, t, invtInit);

	PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	double C[3 * 3 * 3];
	PoseSM.m_C = C;
	double invt[3];
	PoseSM.m_pData = invt;
	R = PoseSM.m_Rot;
	t = PoseSM.m_X;

	//PoseSMInit.m_C[2 * 3 * 3 + 0] = 1e4;
	//PoseSMInit.m_C[2 * 3 * 3 + 4] = 1.1e4;
	//PoseSMInit.m_C[2 * 3 * 3 + 8] = 1.2e4;

	double YInit[2 * 3 * 3];

	double *YRInit = YInit;
	double *YtInit = YInit + 3 * 3;

	InverseMatrix3(YRInit, PoseSMInit.m_C);
	InverseMatrix3(YtInit, PoseSMInit.m_C + 2 * 3 * 3);

	CRVLPSuLM *pMPSuLM;
	CRVL3DSurface2 *pM3DSurface;
	CRVL3DSurface2 **ppMSurf;
	CRVLMPtrChain *pM3DSurfaceList;
	CRVL3DSurface2 **MCurrentSurfArray;
	//RVLSURFACE_MATCH *MatchBuff, *InitMatchBuff;
	//RVLSURFACE_MATCH *pMatch;
	//RVLSURFACE_MATCH_ARRAY *pLastMatchArray;
	//int nMatches;
	//BOOL bPoseUncertWithinTolerance;
	int iSample;
	int iHypothesis;
	double MatchQuality;
	double detQ;
	//double debug_MatchQuality;

	RVLSURFACE_MATCH_ARRAY MatchArray;
	//RVLSURFACE_MATCH_ARRAY debug_MatchArray;
	
	RVLPSULM_MSMATCH_DATA *Queue = new RVLPSULM_MSMATCH_DATA[nS3DSurfaces * maxnM3DSurfaces];

	RVLPSULM_SURF_MATCH_DATA *MSurfMatchData = new RVLPSULM_SURF_MATCH_DATA[maxnM3DSurfaces];

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
	pSPSuLM->Display(m_DebugData.pGUI, pPoseS0Init);

	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("PSuLM");

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	pMouseCallbackData->MatrixSceneModel = NULL;	

	pMouseCallbackData->MSurfMatchData = MSurfMatchData;
	pMouseCallbackData->SSurfMatchData = SSurfMatchData;

	CRVLFigure *pFig2 = m_DebugData.pGUI->OpenFigure("PSuLM2");

	pFig2->m_FontSize = 16;

	int nTextLines = 22;

	int TextImageHeight = nTextLines * pFig2->m_FontSize;

	pFig2->EmptyBitmap(cvSize(800, TextImageHeight), cvScalar(255, 255, 255));

	cvInitFont(&(pFig2->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	char Text[201];

	pFig2->PutText("Hello world!", cvPoint(0, pFig2->m_FontSize), cvScalar(0, 0, 0));

	m_DebugData.pGUI->ShowFigure(pFig2);
#endif

	int nModels = 0;

	double BestFit = 10000000;

	double YSR[3 * 3];
	double YSt[3 * 3];
	double YSR0[3 * 3];
	double YSt0[3 * 3];
	double YMR[3 * 3];
	double YMt[3 * 3];
	double YMR0[3 * 3];
	double YMt0[3 * 3];

	RVLPSULM_MSMATCH_DATA **HypothesisMatchArray = new RVLPSULM_MSMATCH_DATA *[maxnSamples];

	RVLPSULM_MSMATCH_DATA **OutOfGeomConstraintsArray = new RVLPSULM_MSMATCH_DATA *[maxnFailures];

	int HypothesisIndex = 0;

	//int iScene,iModel;
	double *YSSurfR, *YSSurft, *YMSurfR, *YMSurft;
	double util, maxUtil;
	RVLPSULM_MSMATCH_DATA *pMSMatch, *pMSMatch0;
	RVLPSULM_MSMATCH_DATA *pMSMatchArrayEnd;
	int nM3DSurfaces;
	RVLPSULM_MSMATCH_DATA **ppOutOfGeomConstraints;
	RVLPSULM_MSMATCH_DATA **pOutOfGeomConstraintsArrayEnd;
	CRVL3DSurface2 **pMSurfArrayEnd;
	int nFailures, nSamples;
	double volS0, volS, volM0, volM;
	RVLPSULM_MSMATCH_DATA *pQueueEnd;
	RVLPSULM_SURF_MATCH_DATA *pMSurfMatchData;
	RVLPSULM_SURF_MATCH_DATA *pMSurfMatchDataArrayEnd;
	//double eig[3], VNrm[3 * 3], V[3 * 3], lV[3], r[3];
	//BOOL bReal[3];
	BOOL bMatch;
	CRVL3DSurface2 *pS3DSurface0;
	RVLPSULM_HYPOTHESIS *pHypothesis;
	double *t2;
	CRVL3DPose *pPoseSM;

	int debug_nMatchings = 0;
	int debug_nQueueSorts = 0;
	int debug_nSamples = 0;
	
	// feature matching 

	m_PSuLMList.Start();

	while(m_PSuLMList.m_pNext)
	{
		pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());

		MCurrentSurfArray = pMPSuLM->m_3DSurfaceArray;
		pM3DSurfaceList = &(pMPSuLM->m_SurfaceList);

		ppMSurf = MSurfArray;

		nM3DSurfaces = pMPSuLM->m_n3DSurfaces; //pM3DSurfaceList->m_nElements;

		pMSurfMatchData = MSurfMatchData;

		//pM3DSurfaceList->Start();

		//while(pM3DSurfaceList->m_pNext)
		for(i=0;i<nM3DSurfaces;i++)
		{
			//pM3DSurface = (CRVL3DSurface2 *)(pM3DSurfaceList->GetNext());
			pM3DSurface = MCurrentSurfArray[i];

			pM3DSurface->GetPoseContribution();

			*(ppMSurf++) = pM3DSurface;

			pMSurfMatchData->p3DSurface = pM3DSurface;

			pMSurfMatchData->bMatched = 0;

			pMSurfMatchData++;
		}

		double StartTime = m_pTimer->GetTime();

		pMSurfArrayEnd = ppMSurf;

		pMSurfMatchDataArrayEnd = pMSurfMatchData;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
		pMPSuLM->AddToFigure(m_DebugData.pGUI);

		pMouseCallbackData->nMSurfs = pM3DSurfaceList->m_nElements;

		cvWaitKey();
#endif

		// initialize Queue

		pMSMatch = Queue;

		for(pSSurfMatchData = SSurfMatchData; pSSurfMatchData < pSSurfMatchDataArrayEnd; pSSurfMatchData++)
		{
			for(pMSurfMatchData = MSurfMatchData; pMSurfMatchData < pMSurfMatchDataArrayEnd; pMSurfMatchData++, 
				pMSMatch++)
			{
				pMSMatch->pSData = pSSurfMatchData;
				pMSMatch->pMData = pMSurfMatchData;
#ifndef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
				//pMSMatch->Probability = 0.0;
				pMSMatch->bInitGeomConstrSattisfied = 0;
				pMSurfMatchData->MatchProbability = 0.0;
#endif
				pMSMatch->bFailed = 0;
			}

#ifndef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
			pSSurfMatchData->MatchProbability = 0.0;
#endif
		}

		pQueueEnd = pMSMatch - 1;
			
		// generate hypotheses

		for(iHypothesis = 0; iHypothesis < nHypotheses; iHypothesis++)
		{
			// initialize PoseSM

			PoseSM.Copy(&PoseSMInit); 

			PoseSM.m_pData = invt;

			RVLCOPY3VECTOR(invtInit, invt);

			RVLCOPYMX3X3(YRInit, YSR0)
			RVLCOPYMX3X3(YtInit, YSt0)
			RVLCOPYMX3X3(YRInit, YMR0)
			RVLCOPYMX3X3(YtInit, YMt0)

			for(iSample = 0; iSample < maxnSamples; iSample++)
			{
				// compute utility value of each scene surface

				volS0 = (RVLCOVMX3BBVOLUME(YSR0)) * (RVLCOVMX3BBVOLUME(YSt0));

				if(volS0 < 0.0)
					volS0 = -volS0;

#ifndef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
				maxUtil = -1e20;
#endif

				ppSSurf = SSurfArray;

				for(pSSurfMatchData = SSurfMatchData; pSSurfMatchData < pSSurfMatchDataArrayEnd; pSSurfMatchData++, 
					ppSSurf++)
				{
					pS3DSurface = *ppSSurf;

					if(pSSurfMatchData->bMatched)
						continue;

					YSSurfR = pS3DSurface->m_PoseContribution;
					YSSurft = pS3DSurface->m_PoseContribution + 3 * 3;

					RVLSUMMX3X3(YSR0, YSSurfR, YSR)
					RVLSUMMX3X3(YSt0, YSSurft, YSt)

					volS = (RVLCOVMX3BBVOLUME(YSR)) * (RVLCOVMX3BBVOLUME(YSt));

					if(volS < 0.0)
						volS = -volS;

					util = log(volS / volS0) / 6.0;

#ifndef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
					util += pSSurfMatchData->MatchProbability;

					if(util > maxUtil)
					{
						maxUtil = util;
						pS3DSurface0 = pS3DSurface;
					}
#endif

					pSSurfMatchData->util = util;
				}

				// compute utility value of each model surface

				volM0 = (RVLCOVMX3BBVOLUME(YMR0)) * (RVLCOVMX3BBVOLUME(YMt0));

				if(volM0 < 0.0)
					volM0 = -volM0;

				ppMSurf = MSurfArray;

				for(pMSurfMatchData = MSurfMatchData; pMSurfMatchData < pMSurfMatchDataArrayEnd; pMSurfMatchData++, 
					ppMSurf++)
				{
					pM3DSurface = *ppMSurf;

					if(pMSurfMatchData->bMatched)
						continue;

					YMSurfR = pM3DSurface->m_PoseContribution;
					YMSurft = pM3DSurface->m_PoseContribution + 3 * 3;

					RVLSUMMX3X3(YMR0, YMSurfR, YMR)
					RVLSUMMX3X3(YMt0, YMSurft, YMt)

					volM = (RVLCOVMX3BBVOLUME(YMR)) * (RVLCOVMX3BBVOLUME(YMt));

					if(volM < 0.0)
						volM = -volM;

					util = log(volM / volM0) / 6.0;

#ifndef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
					util += pMSurfMatchData->MatchProbability;
#endif

					pMSurfMatchData->util = util;
				}

				debug_nSamples++;

				ppOutOfGeomConstraints = OutOfGeomConstraintsArray;

				nFailures = 0;

				// find next match

				while(nFailures < maxnFailures)
				{
					// find best match

					maxUtil = -1e20;

#ifdef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
					for(pMSMatch = Queue; pMSMatch <= pQueueEnd; pMSMatch++)
					{
						if(pMSMatch->pSData->bMatched || pMSMatch->pMData->bMatched || pMSMatch->bFailed)
							continue;

						util = pMSMatch->pSData->util + pMSMatch->pMData->util + pMSMatch->Probability;

						if(util > maxUtil)
						{
							maxUtil = util;
							pMSMatch0 = pMSMatch;
						}
					}

					debug_nQueueSorts++;
#endif
					
					pMSMatch = Queue + nM3DSurfaces * pS3DSurface0->m_Index;

					pMSMatchArrayEnd = pMSMatch + nM3DSurfaces;

					for(; pMSMatch < pMSMatchArrayEnd; pMSMatch++)
					{
						if(pMSMatch->pMData->bMatched || pMSMatch->bFailed)
							continue;

						util = pMSMatch->pMData->util;

						if(util > maxUtil)
						{
							maxUtil = util;
							pMSMatch0 = pMSMatch;
						}
					}
										
					pS3DSurface = pMSMatch0->pSData->p3DSurface;
					pM3DSurface = pMSMatch0->pMData->p3DSurface;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
					pMouseCallbackData->pSSurf = pS3DSurface;
					pMouseCallbackData->pMSurf = pM3DSurface;

					memcpy(pMouseCallbackData->PoseS0.m_Rot, PoseSM.m_Rot, 3 * 3 * sizeof(double));
					memcpy(pMouseCallbackData->PoseS0.m_X, PoseSM.m_X, 3 * sizeof(double));

					pMouseCallbackData->Flags &= ~RVLPSULM_DISPLAY_SELECTION;

					RVLPSuLMDisplay(pFig);

					//cvWaitKey();

					//if(pMSMatch0->pMData->p3DSurface->m_Index == 12 && 
					//	(pMSMatch0->pSData->p3DSurface->m_Index == 1 || pMSMatch0->pSData->p3DSurface->m_Index == 5))
					//	int debug = 0;
#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
					if(!pMSMatch0->bInitGeomConstrSattisfied)
					{
						debug_nMatchings++;

						if(pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, &(pMSMatch0->EKFData)))
							pMSMatch0->bInitGeomConstrSattisfied = 1;
						else
						{
							// remove pMSMatch0 from Queue

							*pMSMatch0 = *pQueueEnd;

							pQueueEnd--;
						}
					}

					if(pMSMatch0->bInitGeomConstrSattisfied)
					{
						if(iSample > 0)
						{
#endif
							bMatch = pS3DSurface->Match2(pM3DSurface, &PoseSM, MatchQuality, detQ, &MatchArray);
							//if(bMatch)
							//{
							//	pS3DSurface->Match2(pM3DSurface, &PoseSM, debug_MatchQuality, &debug_MatchArray);

							//	double debug = MatchQuality - debug_MatchQuality;
							//}
					
							debug_nMatchings++;
#ifdef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
						}
						else 
						{
							bMatch = TRUE;

							MatchArray = pMSMatch0->EKFData;
						}
#endif
						
						if(bMatch)
						{
							PoseSM.PlanarSurfaceEKFUpdate2(MatchArray.m_C,MatchArray.m_Q,MatchArray.m_e);

							RVLMULMX3X3TVECT(R, t, invt);

							HypothesisMatchArray[iSample] = pMSMatch0;

							pMSMatch0->pMData->bMatched = 1;

							pMSMatch0->pSData->bMatched = 1;

							break;
						}
						else
						{
							pMSMatch0->bFailed = 1;

							*(ppOutOfGeomConstraints++) = pMSMatch0;

							nFailures++;
						}
#ifdef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
					}
#endif
				}	// find next match loop

				if(nFailures >= maxnFailures)
					break;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
				pMSMatch = HypothesisMatchArray[iSample];

				pMouseCallbackData->pSSurf = pMSMatch->pSData->p3DSurface;
				pMouseCallbackData->pMSurf = pMSMatch->pMData->p3DSurface;

				memcpy(pMouseCallbackData->PoseS0.m_Rot, PoseSM.m_Rot, 3 * 3 * sizeof(double));
				memcpy(pMouseCallbackData->PoseS0.m_X, PoseSM.m_X, 3 * sizeof(double));

				pMouseCallbackData->Flags &= ~RVLPSULM_DISPLAY_SELECTION;

				RVLPSuLMDisplay(pFig);

				cvSet(pFig2->m_pImage, cvScalar(255, 255, 255));

				double CMS[3 * 3];

				InverseMatrix3(CMS, YSR0);

				pFig2->DisplayCovMx(CMS, 8, 0, cvScalar(0, 0, 0), RAD2DEG);

				InverseMatrix3(CMS, YSt0);

				pFig2->DisplayCovMx(CMS, 8, 5 * pFig2->m_FontSize, cvScalar(0, 0, 0));

				pFig2->DisplayCovMx(PoseSM.m_C, 8, 10 * pFig2->m_FontSize, cvScalar(0, 0, 0), RAD2DEG);

				pFig2->DisplayCovMx(PoseSM.m_C + 2 * 3 * 3, 8, 15 * pFig2->m_FontSize, cvScalar(0, 0, 0));

				sprintf(Text, "Hypothesis %d; Sample %d", iHypothesis, iSample);

				pFig2->PutText(Text, cvPoint(8, 20 * pFig2->m_FontSize), cvScalar(0, 0, 0));

				sprintf(Text, "Match M%d-S%d: utilM=%lf, utilS=%lf, P=%lf, total=%lf", 
					pMSMatch->pMData->p3DSurface->m_Index, pMSMatch->pSData->p3DSurface->m_Index,
					pMSMatch->pMData->util, pMSMatch->pSData->util, pMSMatch->Probability, 
					pMSMatch->pMData->util + pMSMatch->pSData->util + pMSMatch->Probability);

				pFig2->PutText(Text, cvPoint(8, 21 * pFig2->m_FontSize), cvScalar(0, 0, 0));

				m_DebugData.pGUI->ShowFigure(pFig2);

				cvWaitKey();
#endif

				YSSurfR = pS3DSurface->m_PoseContribution;
				YSSurft = pS3DSurface->m_PoseContribution + 3 * 3;
				YMSurfR = pM3DSurface->m_PoseContribution;
				YMSurft = pM3DSurface->m_PoseContribution + 3 * 3;

				RVLSUMMX3X3(YSR0, YSSurfR, YSR0)
				RVLSUMMX3X3(YSt0, YSSurft, YSt0)
				RVLSUMMX3X3(YMR0, YMSurfR, YMR0)
				RVLSUMMX3X3(YMt0, YMSurft, YMt0)

			}	// hypothesis generation loop

			nSamples = iSample;

			// reset out of geom. constraints flag

			pOutOfGeomConstraintsArrayEnd = ppOutOfGeomConstraints;

			for(ppOutOfGeomConstraints = OutOfGeomConstraintsArray; ppOutOfGeomConstraints < pOutOfGeomConstraintsArrayEnd;
				ppOutOfGeomConstraints++)
			{
				pMSMatch = *ppOutOfGeomConstraints;

				pMSMatch->bFailed = 0;
			}

			// update match probabilities

			for(iSample = 0; iSample < nSamples; iSample++)
			{
				pMSMatch = HypothesisMatchArray[iSample];

#ifdef RVLPSULMBUILDER_HYPOTHESES_COMPLETE_QUEUE_SEARCH
				pMSMatch->Probability -= 0.2;
#else
				pMSMatch->pSData->MatchProbability -= 0.1;

				pMSMatch->pMData->MatchProbability -= 0.1;
#endif

				pMSMatch->pSData->bMatched = 0;

				pMSMatch->pMData->bMatched = 0;
			}

			if(nSamples >= 5)
			{
				// check if the similar pose already exists

				// if not, create hypothesis and insert it into m_HypothesisList

				RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);

				pPoseSM = &(pHypothesis->PoseSM);

				t2 = pPoseSM->m_X;
				RVLCOPY3VECTOR(t, t2);
				pPoseSM->m_Alpha = PoseSM.m_Alpha;
				pPoseSM->m_Beta = PoseSM.m_Beta;
				pPoseSM->m_Theta = PoseSM.m_Theta;
				// only for debugging purpose !!!
				//if(HypothesisIndex == 0)
				//{
				//	RVLNULL3VECTOR(t2)
				//	pPoseSM->m_Alpha = pPoseSM->m_Beta = pPoseSM->m_Theta = 0.0;
				//}
				/////
				pPoseSM->UpdateRotLL();
				pHypothesis->Index = (HypothesisIndex++);
				pHypothesis->cost = 0;
				pHypothesis->visible = 0;
				pHypothesis->iS = pHypothesis->iM = 0;
				pHypothesis->pMPSuLM = pMPSuLM;
				pHypothesis->Flags = 0x00;

				m_HypothesisList.Add(pHypothesis);				
			}
		}	// for each hypothesis

		double ExecutionTime = m_pTimer->GetTime() - StartTime;

		//int debug = 0;
	}	// for each model PSuLM	

	delete[] Queue;
	//delete[] UtilFnArray;
	//delete[] SSurfArray;
	delete[] MSurfArray;
	delete[] SSurfMatchData;
	delete[] MSurfMatchData;
	delete[] HypothesisMatchArray;
	delete[] OutOfGeomConstraintsArray;

#ifndef RVLPSULMBUILDER_HYPOTHESES_DEBUG
	// only for debugging purpose

	CRVLFigure *pFig;

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData;

	m_HypothesisList.Start();

	while(m_HypothesisList.m_pNext)
	{
		pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

		if(pHypothesis->Index == 0)
		{
			pSPSuLM->Display(m_DebugData.pGUI, &(pHypothesis->PoseSM));

			pHypothesis->pMPSuLM->AddToFigure(m_DebugData.pGUI);

			pFig = m_DebugData.pGUI->OpenFigure("PSuLM");

			pMouseCallbackData = (RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

			pMouseCallbackData->MatrixSceneModel = NULL;	

			pMouseCallbackData->MSurfMatchData = MSurfMatchData;
			pMouseCallbackData->SSurfMatchData = SSurfMatchData;

			pMouseCallbackData->pSSurf = NULL;
			pMouseCallbackData->pMSurf = NULL;
		}
		else
		{
			pMouseCallbackData->pMPSuLM = pHypothesis->pMPSuLM;
		
			memcpy(pMouseCallbackData->PoseS0.m_Rot, pHypothesis->PoseSM.m_Rot, 3 * 3 * sizeof(double));
			memcpy(pMouseCallbackData->PoseS0.m_X, pHypothesis->PoseSM.m_X, 3 * sizeof(double));

			pMouseCallbackData->Flags = RVLPSULM_DISPLAY_SURFACES;

			RVLPSuLMDisplay(pFig);
		}

		cvWaitKey();
	}
#endif
	
}



//void CRVLPSuLMBuilder::GetMaxProbabilityMatch(	int iS3DSurface,
//												int maxnM3DSurfaces,
//												int nM3DSurfaces,
//												RVLPSULM_MSMATCH_DATA *MatrixSceneModel,
//												RVLPSULM_SURF_MATCH_DATA *SSurfMatchData)
//{
//	RVLPSULM_MSMATCH_DATA *pMSMatch = MatrixSceneModel + maxnM3DSurfaces * iS3DSurface;
//
//	RVLPSULM_MSMATCH_DATA *pMatchArrayEnd = pMSMatch + nM3DSurfaces;
//
//	RVLPSULM_MSMATCH_DATA *pBestMatchCandidate = NULL;
//
//	double maxProbability = -1e20;
//
//	for(; pMSMatch < pMatchArrayEnd; pMSMatch++)
//	{
//		if(pMSMatch->Flags & (RVLPSULM_MSMATCH_FLAG_INIT_GEOM_CONSTR_NOT_SATISFIED |
//			RVLPSULM_MSMATCH_FLAG_GEOM_CONSTR_NOT_SATISFIED))
//			continue;
//
//		if(pMSMatch->Probability > maxProbability)
//		{
//			if(pMSMatch->pM3DSurface->m_Flags & RVLOBJ2_FLAG_MATCHED)
//				continue;
//
//			maxProbability = pMSMatch->Probability;
//			pBestMatchCandidate = pMSMatch;
//		}
//	}
//
//	SSurfMatchData[iS3DSurface].maxMatchProbability = maxProbability;
//	SSurfMatchData[iS3DSurface].pMatch = pBestMatchCandidate;
//}
//


// This function uses m_pMem2.

void CRVLPSuLMBuilder::Localization(CRVLPSuLM * pSPSuLM,
								    CRVL3DPose *pPoseS0,
									CRVLPSuLM * pPrevSPSuLM)
{
#pragma region Initial Pose Computation (Prediction)
	//*************************************************************
	//
	//     INITIAL POSE COMPUTATION (PREDICTION)
	//
	//*************************************************************

	double StartTime = m_pTimer->GetTime();

	double StartTimeTotal = StartTime;

	double RAsAmp[3 * 3], tAsAmp[3];
	double *RAspAmp, *tAspAmp;

	double *RAsAsp = m_pPoseSSp->m_Rot;
	double *tAsAsp = m_pPoseSSp->m_X;
	
		// compute relative motion
	if(m_Flags & RVLPSULMBUILDER_FLAG_KIDNAPPED)
	{
		m_pPoseSSp->m_Alpha = m_pPoseSSp->m_Beta = m_pPoseSSp->m_Theta = 0.0;
		m_pPoseSSp->UpdateRotA0();
		RVLNULL3VECTOR(m_pPoseSSp->m_X);
	}
	else
	{
		double R0APrev[3 * 3];
		double t0APrev[3];

		InverseTransform3D(R0APrev, t0APrev, m_pPoseSp0->m_Rot, m_pPoseSp0->m_X);

		RVLCombineTransform3D(R0APrev, t0APrev, pPoseS0->m_Rot, pPoseS0->m_X, m_pPoseSSp->m_Rot, m_pPoseSSp->m_X);
	}
	//CRVL3DPose *pPoseAA;

	//Reset value

	m_HypothesisList.RemoveAll();

	CRVL3DPose PoseS0Init, testPose;

	//Initialize PoseS0Init

	if (pPoseS0)
		PoseS0Init.Copy(pPoseS0);
	else
	{
		PoseS0Init.m_Alpha = 0.0;
		PoseS0Init.m_Beta = 0.0;
		PoseS0Init.m_Theta = 0.0;
		//PoseS0Init.m_Alpha = -45.0 * DEG2RAD;	// debug
		PoseS0Init.UpdateRotLL();

		memset(PoseS0Init.m_X, 0, 3 * sizeof(double));
	}

	PoseS0Init.m_sa = sin(PoseS0Init.m_Alpha);
	PoseS0Init.m_ca = cos(PoseS0Init.m_Alpha);

	double C[3 * 3 * 3];
	PoseS0Init.m_C = C;
	double invtInit[3];
	PoseS0Init.m_pData = invtInit;

	PoseS0Init.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;

	//Define initial uncertainty to be used for initial matching
	double PInit[3 * 3 * 3], PInit2[3 * 3 * 3];

	//Define uncertainty constants
	double XUnc, AngleUnc, ThetaUnc, XEKFUnc;
	XUnc = 5000.0;			// uncertainty for feature matching 
	XEKFUnc = 10000.0;		// initial uncertainty for EKF
	AngleUnc = 30.0;
	ThetaUnc = 10.0;


	//Define default uncertainty (3DOF)
	double uncert3DOFMat[3 * 3];
	RVLNULLMX3X3(uncert3DOFMat)

	uncert3DOFMat[0] = AngleUnc * AngleUnc * DEG2RAD * DEG2RAD;
	uncert3DOFMat[4] = XUnc * XUnc;
	uncert3DOFMat[8] = XUnc * XUnc;

	//Define default uncertainty (6DOF)
	CRVL3DPose uncert6DOFPose;
	uncert6DOFPose.m_X[0] = XUnc;	
	uncert6DOFPose.m_X[1] = XUnc;	
	//uncert6DOFPose.m_X[1] = 150.0;	
	uncert6DOFPose.m_X[2] = XUnc;	

	uncert6DOFPose.m_Alpha = AngleUnc;
	uncert6DOFPose.m_Beta = AngleUnc;	
	uncert6DOFPose.m_Theta = ThetaUnc;
	//uncert6DOFPose.m_Alpha = 40.0;
	//uncert6DOFPose.m_Beta = 20.0;	
	//uncert6DOFPose.m_Theta = 10.0;

	//Define default unconstrained EKF uncertainty (6DOF)
	CRVL3DPose uncertEKFPose;
	uncertEKFPose.m_X[0] = XEKFUnc;	
	uncertEKFPose.m_X[1] = XEKFUnc;	
	uncertEKFPose.m_X[2] = XEKFUnc;	

	uncertEKFPose.m_Alpha = AngleUnc;
	uncertEKFPose.m_Beta = AngleUnc;	
	uncertEKFPose.m_Theta = AngleUnc;


	//Define zeroPose
	CRVL3DPose zeroPose;

	//Initialize zeroPose
	zeroPose.m_Alpha = 0.0;
	zeroPose.m_Beta = 0.0;
	zeroPose.m_Theta = 0.0;
	zeroPose.UpdateRotA0();
	zeroPose.m_sa = sin(zeroPose.m_Alpha);
	zeroPose.m_ca = cos(zeroPose.m_Alpha);

	memset(zeroPose.m_X, 0, 3 * sizeof(double));

	//BOOL bIncrementalLocalizationUsingOdometry = FALSE;

	BOOL bGenerateHypotheses = TRUE;

	RVLPSULM_HYPOTHESIS *pHypothesis;
	RVLPSULM_PARTICLE *pParticleArrayEnd;

	if((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
	{
		//Define uncertainty for initial matching (PInit)
		if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
		{
			DetermineUncertainty3DOFTo6DOF(PInit2,&PoseS0Init,&zeroPose,uncert3DOFMat);
			PanTiltRollUncertainty(PInit, PInit2, &PoseS0Init);
		}
		else
		{
			RVLCreatePoseUncertMx(&uncert6DOFPose, PInit, PoseS0Init.m_OrientNrm);
		}
		//else if{Odometry}
		//{
		//	DetermineUncertainty3DOFTo6DOF(PInit2,&PoseS0Init,m_pPoseSSp);
		//	PanTiltRollUncertainty(PInit, PInit2, &PoseS0Init);
		//}
	}
	else //Localization
	{
		pPrevSPSuLM = NULL;

		//Clear sublist
		m_PSuLMSubList.RemoveAll();

		//either there is no map 
		if(m_PSuLMList.m_nElements==0 || (m_Flags & RVLPSULMBUILDER_FLAG_NEW_SUBMAP) != 0)
		{
			// I'm not sure if the following two lines are needed. 
			// Since there are no local models, the scene will not be matched to anything.

			DetermineUncertainty3DOFTo6DOF(PInit2,&PoseS0Init,m_pPoseSSp);
			PanTiltRollUncertainty(PInit, PInit2,&PoseS0Init);

			//PoseS0Init.m_C = PInit;

			m_nParticles = 0;
		}
		else  //Map exists
		{			
			if(m_Flags & RVLPSULMBUILDER_FLAG_KIDNAPPED)
			{
				//Initial model is defined.
				if(m_pNearestModelPSuLM != NULL)
				{
					//Add m_pPrevModelPSuLM (defined in OnSASUpdate) to m_PSuLMSubList since it is the closest Model
					m_PSuLMSubList.Add(m_pNearestModelPSuLM);	
				}
				else //Initial model is NOT defined.
				{
					//Perform global localization
					//Do not add anything to sublist - Main list is then used in Hypothesis generation for global localization
					
				}

				//Initialize the other params m_pPrevRobotPoseSM
				m_pPrevRobotPoseSM->m_Alpha = m_pPrevRobotPoseSM->m_Beta = m_pPrevRobotPoseSM->m_Theta = 0.0;
				m_pPrevRobotPoseSM->UpdateRotA0();
				RVLNULL3VECTOR(m_pPrevRobotPoseSM->m_X)


				//Define uncertainty for initial matching (PInit)
				if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
				{
					DetermineUncertainty3DOFTo6DOF(PInit2,&PoseS0Init,&zeroPose,uncert3DOFMat);
					PanTiltRollUncertainty(PInit, PInit2, &PoseS0Init);

					//double eig[3], VNrm[9], V[9];	//debug
					//double lV;	//debug
					//RVLGetAxesOfCov3D(PInit, eig, VNrm, V, &lV);	//debug
					//RVLGetAxesOfCov3D(PInit2, eig, VNrm, V, &lV);	//debug

					//int debug = 0;
				}
				else
				{
					RVLCreatePoseUncertMx(&uncert6DOFPose, PInit, PoseS0Init.m_OrientNrm);
				}

				m_nParticles = 0;

				m_Flags &= ~RVLPSULMBUILDER_FLAG_KIDNAPPED;

			}	// if(m_Flags & RVLPSULMBUILDER_FLAG_KIDNAPPED)
			else	// incremental localization using odometry
			{
#ifdef RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
				m_PSuLMSubList.RemoveAll();

				m_PSuLMSubList.Add(m_pNearestModelPSuLM);	

				if((m_Flags & RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING) && m_pLoopStartPSuLM)
					m_PSuLMSubList.Add(m_pLoopStartPSuLM);
#else
#ifdef RVLPSULMBUILDER_MAPBUILDING_6DOF
				RVLCreatePoseUncertMx(&uncert6DOFPose, PInit, PoseS0Init.m_OrientNrm);

				m_PSuLMSubList.RemoveAll();
#else
				//bIncrementalLocalizationUsingOdometry = TRUE;

				//1. Determine odomotry uncertainty between scenes
				double CAsAsp[9];
				DetermineOdometryUncertainty(CAsAsp,m_pPoseSSp);
				memcpy(m_pPoseSSp->m_C, CAsAsp, 9 * sizeof(double));

				double stdAlpha = sqrt(CAsAsp[0]);
				double stdX = sqrt(CAsAsp[4]);

				double CAspAmp[9];

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS
				//GetLocalModels(m_pPoseSSp, m_pPrevRobotPoseSM, m_pPrevModelPSuLM, 3.0, 
				//	RVLPSULMBUILDER_GETLOCALMODELS_FLAG_ORIENT_CONSTR | RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT);

				pParticleArrayEnd = m_ParticleArray + m_nParticles;

				CRVL3DPose PoseAspAmp;
				PoseAspAmp.m_C = CAspAmp;

				double *RAspAmp = PoseAspAmp.m_Rot;
				double *tAspAmp = PoseAspAmp.m_X;

				m_PSuLMSubList.RemoveAll();

				double CCspCmp[3 * 3 * 3];
				RVLPSULM_PARTICLE *pHypothesis2;
				double *RAspAmp2, *tAspAmp2;
				double dalpha, cda, sda;
				double dR[3 * 3];

				for(pHypothesis2 = m_ParticleArray; pHypothesis2 < pParticleArrayEnd; pHypothesis2++)
				{
					if((m_Flags & RVLPSULMBUILDER_FLAG_PARTICLE_FILTER) != 0 && 
						(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
					{
						RAspAmp2 = pHypothesis2->PoseSM.m_Rot;
						tAspAmp2 = pHypothesis2->PoseSM.m_X;

						RVLCOPYMX3X3(RAspAmp2, RAspAmp)
						RVLCOPY3VECTOR(tAspAmp2, tAspAmp)
						RVLNULLMX3X3(CAspAmp)

						RVLCOMPTRANSF3D3DOF(RAspAmp2, tAspAmp2, RAsAsp, tAsAsp, RAsAmp, tAsAmp)

						dalpha = RVLGaussRand(stdAlpha);
						cda = cos(dalpha);
						sda = sin(dalpha);
						dR[0] = cda;
						dR[1] = -sda;
						dR[3] = sda;
						dR[4] = cda;
						
						RVLCOMPROT3D3DOF(RAsAmp, dR, RAspAmp2)
						tAspAmp2[0] = tAsAmp[0] + RVLGaussRand(stdX);
						tAspAmp2[1] = tAsAmp[1] + RVLGaussRand(stdX);
						tAspAmp2[2] = 0.0;

						//pHypothesis2->d2 = tAspAmp2[0] * tAspAmp2[0] + tAspAmp2[1] * tAspAmp2[1];
					}
					else
					{
						RVLDeterminePose6DOFTo3DOF(&PoseAspAmp,&(pHypothesis2->PoseSM),m_pPoseAC);

						//2. Determine odometry uncertainty between scene and model obtained in the previous step
						PanTiltRollUncertainty(CCspCmp,pHypothesis2->P, &(pHypothesis2->PoseSM),true);
						DetermineUncertainty6DOFTo3DOF(CAspAmp, CCspCmp, &PoseAspAmp);
					}

					//3A) Determine the closest model to the current scene using Euclidean distance 

					//GetLocalModels(m_pPoseSSp, &PoseAspAmp, pHypothesis2->pMPSuLM, 4.0, RVLPSULMBUILDER_GETLOCALMODELS_FLAG_ORIENT_CONSTR);

					//RVLResetFlags<CRVLPSuLM>(&m_PSuLMSubList, RVLPSULM_FLAG_CLOSE);

					GetLocalModels2(m_pPoseSSp, &PoseAspAmp, pHypothesis2->pMPSuLM, 4.0, 
						RVLPSULMBUILDER_GETLOCALMODELS_FLAG_ORIENT_CONSTR | RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT);

					//RVLResetFlags<CRVLPSuLM>(&m_PSuLMSubList, RVLPSULM_FLAG_CLOSE);					
				}

				RVLResetFlags<CRVLPSuLM>(&m_PSuLMSubList, RVLPSULM_FLAG_CLOSE);



#else
				//2. Determine odometry uncertainty between scene and model obtained in the previous step
				DetermineUncertainty6DOFTo3DOF(CAspAmp,m_pPrevCameraPoseSM->m_C,m_pPrevRobotPoseSM);
				
				//3A) Determine the closest model to the current scene using Euclidean distance 

				memcpy(m_pPrevRobotPoseSM->m_C, CAspAmp, 9 * sizeof(double));


				//First determine absolute pose for current PSuLM
				double RAbs[9], tAbs[3];
				
				RVLCombineTransform3D(m_pModelAbsPose->m_Rot,m_pModelAbsPose->m_X,m_pPrevRobotPoseSM->m_Rot,m_pPrevRobotPoseSM->m_X,RAbs,tAbs);
				RVLCombineTransform3D(RAbs,tAbs,m_pPoseSSp->m_Rot,m_pPoseSSp->m_X,pSPSuLM->m_PoseAbs->m_Rot,pSPSuLM->m_PoseAbs->m_X);
				//pSPSuLM->m_PoseAbs->UpdatePTRLL();  

				double minDist = 1e10;
				double currDist;
				double xDiff, yDiff;

				CRVLPSuLM *pMPSuLM;
				m_pNearestModelPSuLM = NULL;

				double V[3];
				double eAlpha;
				CRVL3DPose PoseRel;
				double *RAs0, *RAm0, *RAsAm;
				
				m_PSuLMList.Start();
				while(m_PSuLMList.m_pNext)
				{
					pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());

					if(pMPSuLM->m_iSubMap != m_pPrevModelPSuLM->m_iSubMap)
						continue;

					//if(pMPSuLM->m_Index >= 442)
					//	int debug = 0;

					xDiff = pMPSuLM->m_PoseAbs->m_X[0] -  pSPSuLM->m_PoseAbs->m_X[0];
					yDiff = pMPSuLM->m_PoseAbs->m_X[1] -  pSPSuLM->m_PoseAbs->m_X[1];

					currDist = xDiff*xDiff + yDiff*yDiff;

					//Get angle b/n model and scene pose

					RAs0 = pSPSuLM->m_PoseAbs->m_Rot;
					RAm0 = pMPSuLM->m_PoseAbs->m_Rot;
					RAsAm = PoseRel.m_Rot;

					RVLMXMUL3X3T1(RAs0, RAm0, RAsAm)
				
					PoseRel.GetAngleAxis(V,eAlpha);

					if((currDist < (m_MinHybridLocalizationDist*m_MinHybridLocalizationDist*6)) && 
						(fabs(eAlpha*RAD2DEG) < 2*m_MinHybridLocalizationAngle))
					{
						if(currDist < minDist)
						{
							minDist = currDist;
							m_pNearestModelPSuLM = pMPSuLM;
						}
					}

					
				}
#endif			

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS
				if(m_PSuLMSubList.m_nElements == 0 || pSPSuLM->m_minPlaneExists == 0)
#else
				//None found
				if(m_pNearestModelPSuLM==NULL)
#endif
				{
					// current PSuLM to map. Odometry is used since the positions are too far apart (m_pPoseSSp). Hypothesis generation is NOT performed
					//CRVL3DPose PoseAsAmp;
					//RVLCombineTransform3D(m_pPrevRobotPoseSM->m_Rot, m_pPrevRobotPoseSM->m_X, 
					//	m_pPoseSSp->m_Rot, m_pPoseSSp->m_X, 
					//	PoseAsAmp.m_Rot, PoseAsAmp.m_X);
					//DetermineUncertainty3DOFTo6DOF(PInit2,&PoseS0Init,&PoseAsAmp);
					//PanTiltRollUncertainty(PInit, PInit2,&PoseS0Init);	
					
					//Define uncertainty for EKF (PoseS0Init.m_C)
					//if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_EKF_CONSTRAINT)
					//{
					//	PoseS0Init.m_C = PInit;
					//}
					//else
					//{
					//	PoseS0Init.SetUncert(&uncertEKFPose);
					//}
					
					//SaveCurrentData(pSPSuLM, m_pPrevModelPSuLM, &PoseS0Init, pPoseS0);	
					m_LocalizationTime = m_pTimer->GetTime() - StartTime;

					m_HypothesisList.RemoveAll();

					m_nHypotheses = 0;

					bGenerateHypotheses = FALSE;
					
				}
#ifndef RVLPSULMBUILDER_GET_LOCAL_MODELS
				else
				{
					//Add to sublist
					m_PSuLMSubList.Add(m_pNearestModelPSuLM);

					
					//3B) Get rel pose between closest model and previous model(3DOF) ie M' -> M
					CRVL3DPose PoseRobMpM;
					
					double R0M[9], t0M[3];
					InverseTransform3D(R0M,t0M,m_pNearestModelPSuLM->m_PoseAbs->m_Rot,m_pNearestModelPSuLM->m_PoseAbs->m_X);

					RVLCombineTransform3D(R0M, t0M, m_pModelAbsPose->m_Rot, m_pModelAbsPose->m_X, PoseRobMpM.m_Rot, PoseRobMpM.m_X);
					//PoseRobMpM.UpdatePTRLL();  
					
					//3C) Determine odometry uncertainty between models
					double CovRobMpM[9];
					DetermineOdometryUncertainty(CovRobMpM,&PoseRobMpM);


					//4. Determine uncertainty between current scene and selected model
					//a)
					double CovRobSpM[9];
					CRVL3DPose PoseRobSpM;
					UncertaintyEKFPrediction3DOF(CovRobSpM,CAspAmp,CovRobMpM,m_pPrevRobotPoseSM,&PoseRobMpM);

					RVLCombineTransform3D(PoseRobMpM.m_Rot,PoseRobMpM.m_X,m_pPrevRobotPoseSM->m_Rot,m_pPrevRobotPoseSM->m_X,PoseRobSpM.m_Rot,PoseRobSpM.m_X);

					//b)
					double CovRobSM[9];
					CRVL3DPose PoseRobSM;
					UncertaintyEKFPrediction3DOF(CovRobSM,CAsAsp,CovRobSpM,m_pPoseSSp,&PoseRobSpM);

					RVLCombineTransform3D(PoseRobSpM.m_Rot,PoseRobSpM.m_X,m_pPoseSSp->m_Rot,m_pPoseSSp->m_X,PoseRobSM.m_Rot,PoseRobSM.m_X);

					
					DetermineUncertainty3DOFTo6DOF(PInit2,&PoseS0Init,&PoseRobSM,CovRobSM);
					PanTiltRollUncertainty(PInit, PInit2,&PoseS0Init);
					//PoseS0Init.m_C = PInit;
					
					
					//// debug

					//CvMat *P;

					//P = cvCreateMat(6, 6, CV_64FC1);

					//RVL3x3x3BlockMxTo6x6(PInit, P->data.db);

					//double detP = cvDet(P);

					//cvReleaseMat(&P);

					/////
						
				}
#endif
#endif	// !RVLPSULMBUILDER_MAPBUILDING_6DOF
#endif	// !RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
			}	// Incremental localization using odometry
		}	// Map exists		
	}	// Localization

	CRVL3DPose *pPoseS0Init = NULL;

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS
	PoseS0Init.SetUncert(&uncertEKFPose);
	if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
	{
		DetermineUncertainty3DOFTo6DOF(PInit2,&PoseS0Init,&zeroPose,uncert3DOFMat);
		PanTiltRollUncertainty(PInit, PInit2, &PoseS0Init);
	}
#else
	//Define uncertainty for EKF (PoseS0Init.m_C)
	if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_EKF_CONSTRAINT)
	{
		PoseS0Init.m_C = PInit;
	}
	else
	{
		PoseS0Init.SetUncert(&uncertEKFPose);
	}
#endif

	double *R = PoseS0Init.m_Rot;
	double *t = PoseS0Init.m_X;
	RVLMULMX3X3TVECT(R, t, invtInit);

	double *R2 = m_PoseSMInit.m_Rot;
	double *t2 = m_PoseSMInit.m_X;

	RVLCOPYMX3X3(R, R2)
	RVLCOPY3VECTOR(t, t2)

	memcpy(m_PoseSMInit.m_C, PoseS0Init.m_C, 3 * 3 * 3 * sizeof(double));

	pPoseS0Init = &PoseS0Init;

	
	//FILE *fpPInit;
	//fopen_s(&fpPInit, "C:\\RVL\\ExpRez\\PInit.dat", "w");

	//for(int i=0;i<9;i++)
	//{
	//	for(int j=0;j<3;j++)
	//	{
	//		fprintf(fpPInit, "%lf\t", PInit[i*9 + j]);
	//	}
	//	fprintf(fpPInit,"\n");
	//}
	//
	//fclose(fpPInit);
#pragma endregion

	//*************************************************************
	//
	//     HYPOTHESIS GENERATION
	//
	//*************************************************************

	DWORD HypEvalMethod = (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD);
	
	if(bGenerateHypotheses)
	{
		if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
			InitHypothesisEvaluation4(pSPSuLM);

		if (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_GENERATION_INDEXING)
			m_Indexing.GenerateHypotheses();
		else
			Hypotheses3(pSPSuLM, pPoseS0Init, PInit, pPrevSPSuLM);
		//HypothesesPROSAC(pSPSuLM, pPoseS0Init, PInit, pPrevSPSuLM);
		//Hypotheses3(pSPSuLM, pPoseS0Init, PInit, pPrevSPSuLM);
		//
		//
	}

	m_nHypotheses = m_HypothesisList.m_nElements;

	//m_iLocalModel = -1;

	// copy m_PoseA0 to m_PrevPoseA0 (this is otherwise done in PSuLMBuilder::SaveCurrentData)
	if((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
	{
		RVLCOPY3VECTOR(pPoseS0->m_X, m_pPoseSp0->m_X)
		m_pPoseSp0->m_Alpha = pPoseS0->m_Alpha;
		m_pPoseSp0->m_Beta = pPoseS0->m_Beta;
		m_pPoseSp0->m_Theta = pPoseS0->m_Theta;
		double *R = pPoseS0->m_Rot;
		double *R2 = m_pPoseSp0->m_Rot;
		RVLCOPYMX3X3(R, R2);
	}

	//m_LocalizationTime = m_pTimer->GetTime() - StartTime;	
	m_HypGenTime = m_pTimer->GetTime() - StartTime;

	//return;


	//if(m_minSceneInfo < 150.0)
	//	m_HypothesisList.RemoveAll();

#ifdef RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
	if(m_HypothesisList.m_nElements == 0)
	{
		m_LocalizationTime = m_pTimer->GetTime() - StartTime;

		m_nHypotheses = m_nPlausibleHypotheses = 0;

		return;
	}
#else
#ifdef RVLPSULMBUILDER_MAPBUILDING_6DOF
	if(m_HypothesisList.m_nElements == 0)
	{
		m_LocalizationTime = m_pTimer->GetTime() - StartTimeTotal;

		m_nHypotheses = m_nPlausibleHypotheses = 0;

		return;
	}
#else	// Used and described in Nyarko's thesis
	if((m_Flags & RVLPSULMBUILDER_FLAG_PARTICLE_FILTER) == 0 || 
		(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) != RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
	{
		if((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
		{
			//First iteration and there is no database or a new submap should be initialized
			if(m_PSuLMList.m_nElements == 0 || (m_Flags & RVLPSULMBUILDER_FLAG_NEW_SUBMAP) != 0)
			{
				SaveCurrentData(pSPSuLM, NULL, &PoseS0Init, pPoseS0);

				m_Flags &= ~(RVLPSULMBUILDER_FLAG_NEW_SUBMAP | RVLPSULMBUILDER_FLAG_KIDNAPPED);
			}
		}

		m_minSceneInfo = pSPSuLM->m_minInfo;
		m_minScenePlaneExists = pSPSuLM->m_minPlaneExists;

		if(m_HypothesisList.m_nElements == 0)
		{
			if(m_pNearestModelPSuLM)
			{
				m_minModelInfo = m_pNearestModelPSuLM->m_minInfo;
				m_minModelPlaneExists = m_pNearestModelPSuLM->m_minPlaneExists;

				if((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION &&
					(m_Flags & RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING) == 0)
				{
					// current PSuLM to map. Odometry is used since the positions are too far apart (m_pPoseSSp). Hypothesis generation is NOT performed
					CRVL3DPose PoseAsAmp;
					RVLCombineTransform3D(m_pPrevRobotPoseSM->m_Rot, m_pPrevRobotPoseSM->m_X, 
						m_pPoseSSp->m_Rot, m_pPoseSSp->m_X, 
						PoseAsAmp.m_Rot, PoseAsAmp.m_X);
					DetermineUncertainty3DOFTo6DOF(PInit2,&PoseS0Init,&PoseAsAmp);
					PanTiltRollUncertainty(PInit, PInit2,&PoseS0Init);	
					
					//Define uncertainty for EKF (PoseS0Init.m_C)
					if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_EKF_CONSTRAINT)
					{
						PoseS0Init.m_C = PInit;
					}
					else
					{
						PoseS0Init.SetUncert(&uncertEKFPose);
					}
					
					SaveCurrentData(pSPSuLM, m_pPrevModelPSuLM, &PoseS0Init, pPoseS0);	
				}
				m_LocalizationTime = m_pTimer->GetTime() - StartTime;
			}
			else
			{
				m_minModelInfo = -1;
				m_minModelPlaneExists = 0;
			}

			////SaveCurrentData();
			//if((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
			//{
			//	SaveCurrentData(pSPSuLM, m_pNearestModelPSuLM, &PoseS0Init, pPoseS0);	
			//}
			m_LocalizationTime = m_pTimer->GetTime() - StartTime;

			m_nHypotheses = 0;	//$140322

			return;
		}	// if(m_HypothesisList.m_nElements == 0)
	}
#endif	// !RVLPSULMBUILDER_MAPBUILDING_6DOF
#endif	// !RVLPSULMBUILDER_MAPBUILDING_SEQUENCE

#pragma region Hypothesis Evaluation
	//*************************************************************
	//
	//     HYPOTHESIS EVALUATION
	//
	//*************************************************************

#ifdef NEVER
	CRVLQListArray Queue;

	Queue.m_Size = (m_maxnLines + pSPSuLM->m_n3DLines) * 
		DOUBLE2INT(sqrt((double)(4 * (m_pCamera->Width * m_pCamera->Width + m_pCamera->Height * m_pCamera->Height))));

	Queue.m_ListArray = new RVLQLIST[Queue.m_Size];

	Queue.InitListArray(Queue.m_ListArray, Queue.m_ListArray);

	RVLQLIST *ListArray = Queue.m_ListArray;

	RVLPSULM_HYPOTHESIS *pHypothesis;

	m_HypothesisList.Start();

	while(m_HypothesisList.m_pNext)
	{
		pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

		RVLQLISTARRAY_ADD_ENTRY(ListArray, 0, pHypothesis);
	}

	int maxCost = 0;

	int Cost = 0;

	BOOL bCompleted = FALSE;

	CRVL3DLine2 *pM3DLine, *pS3DLine;
	CRVL2DLine2 *pM2DLine, *pS2DLine;
	CRVLPSuLM *pMPSuLM;
	void **ppHypothesis;
	BOOL bMatchS;
	int cost;
	int visible;
	RVLPSULM_HYPOTHESIS *pNextHypothesis;
	int visibility;

	while(!bCompleted)	// repeat until the best hypothesis is found or maxCost is reached  
	{
		while(!bCompleted)	// repeat until ListArray of the current cost is empty 
							// or the best hypothesis is found
		{
			pHypothesis = (RVLPSULM_HYPOTHESIS *)(ListArray[Cost].pFirst);

			if(pHypothesis == NULL)
				break;

			ppHypothesis = &(ListArray[Cost].pFirst);

			while(pHypothesis)
			{
				//if(pHypothesis->Index == 1735 && pHypothesis->cost >= 1485)
				//if(pHypothesis->Index == 311 && pHypothesis->cost >= 1485)
				//if(pHypothesis->cost != Cost)
				//if(pHypothesis->Index == 2973)
				//if(Cost == 740)
				//	int tmp1 = 0;

				pMPSuLM = pHypothesis->pMPSuLM;

				if(pHypothesis->iM < pMPSuLM->m_n3DLines)
				{
					pM3DLine = pMPSuLM->m_3DLineArray[pHypothesis->iM];
	
					pM2DLine = *((CRVL2DLine2 **)(pM3DLine->m_pData + m_M3DLineSet.m_iDataProjectPtr));
				}
				else
					pM3DLine = NULL;

				if(pHypothesis->iS < pSPSuLM->m_n3DLines)
				{
					pS3DLine = pSPSuLM->m_3DLineArray[pHypothesis->iS];

					pS2DLine = *((CRVL2DLine2 **)(pS3DLine->m_pData + m_S3DLineSet.m_iDataProjectPtr));
				}
				else
					pS3DLine = NULL;

				if(pS3DLine == NULL && pM3DLine == NULL)
				{
					if(pHypothesis->bVisibility)
					{
						bCompleted = TRUE;

						break;
					}
					else
					{
						visibility = 100 * pHypothesis->visible / (pSPSuLM->m_Evidence + pMPSuLM->m_Evidence);

						if(visibility >= m_VisibilityThr)
						{
							pHypothesis->bVisibility = 1;

							cost = (100 - visibility) * pHypothesis->cost / visibility;
						}
						else
						{
							*ppHypothesis = pHypothesis->pNext;		// I think that this line can be removed. (try it!)

							// remove the hypothesis from ListArray[Cost]

							if(pHypothesis->pNext == NULL)
							{
								ListArray[Cost].ppNext = ppHypothesis;

								*ppHypothesis = NULL;
							}
							else
								*ppHypothesis = pHypothesis->pNext;

							/////

							pHypothesis = (RVLPSULM_HYPOTHESIS *)(pHypothesis->pNext);

							continue;
						}
					}
				}
				else
				{
					if(pS3DLine != NULL && pM3DLine != NULL)
						bMatchS = (pS2DLine->m_leniU >= pM2DLine->m_leniU);
					else
						bMatchS = (pS3DLine != NULL);

					if(bMatchS)
					{
						cost = MatchLine(pS3DLine, pMPSuLM, &(pHypothesis->PoseSM), FALSE, visible);

						pHypothesis->visible += visible;

						pHypothesis->iS++;
					}
					else
					{
						cost = MatchLine(pM3DLine, pSPSuLM, &(pHypothesis->PoseSM), TRUE, visible);

						pHypothesis->visible += visible;

						pHypothesis->iM++;
					}
				}

				pNextHypothesis = (RVLPSULM_HYPOTHESIS *)(pHypothesis->pNext);

				if(cost)
				{
					cost += pHypothesis->cost;

					//if(cost == 1489)
					//	int tmp1 = 0;

					//*** resort queue

					// remove the hypothesis from ListArray[Cost]

					if(pHypothesis->pNext == NULL)
					{
						ListArray[Cost].ppNext = ppHypothesis;

						*ppHypothesis = NULL;
					}
					else
						*ppHypothesis = pHypothesis->pNext;

					// append the hypothesis to ListArray[cost] 

					RVLQLISTARRAY_ADD_ENTRY(ListArray, cost, pHypothesis);

					//***

					pHypothesis->cost = cost;

					if(cost > maxCost)
						maxCost = cost;
				}
				else
					ppHypothesis = &(pHypothesis->pNext);
			
				pHypothesis = pNextHypothesis;
			}	// while(pHypothesis)
		}	// while(!bCompleted)

		Cost++;

		if(Cost > maxCost)
			break;
	}	// while(!bCompleted)


#endif

#ifdef NEWCODE
	CRVLQListArray Queue;

	Queue.m_Size = m_nCells + m_nCells;

	Queue.m_ListArray = new RVLQLIST[Queue.m_Size];

	Queue.InitListArray(Queue.m_ListArray, Queue.m_ListArray);

	RVLQLIST *ListArray = Queue.m_ListArray;

	RVLPSULM_HYPOTHESIS *pHypothesis;

	m_HypothesisList.Start();

	while(m_HypothesisList.m_pNext)
	{
		pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

		RVLQLISTARRAY_ADD_ENTRY(ListArray, 0, pHypothesis);
	}

	//maximum cost
	int maxCost = m_nCells + m_nCells;

	int Cost = 0;

	BOOL bCompleted = FALSE;

	CRVL3DSurface2 *pM3DSurface, *pS3DSurface;
	CRVLPSuLM *pMPSuLM;
	void **ppHypothesis;
	BOOL bMatchS;
	int cost;
	int visible;
	RVLPSULM_HYPOTHESIS *pNextHypothesis;
	int visibility;

	while(!bCompleted)	// repeat until the best hypothesis is found or maxCost is reached  
	{
		while(!bCompleted)	// repeat until ListArray of the current cost is empty 
							// or the best hypothesis is found
		{
			pHypothesis = (RVLPSULM_HYPOTHESIS *)(ListArray[Cost].pFirst);

			if(pHypothesis == NULL)
				break;

			ppHypothesis = &(ListArray[Cost].pFirst);

			while(pHypothesis)
			{
				pMPSuLM = pHypothesis->pMPSuLM;

				
				if(pHypothesis->iM < pMPSuLM->m_n3DSurfaces)
					pM3DSurface = pMPSuLM->m_3DSurfaceArray[pHypothesis->iM];
				else
					pM3DSurface = NULL;

				if(pHypothesis->iS < pSPSuLM->m_n3DSurfaces)
					pS3DSurface = pSPSuLM->m_3DSurfaceArray[pHypothesis->iS];
				else
					pS3DSurface = NULL;


				if(pS3DSurface == NULL && pM3DSurface == NULL)
				{
					//if(pHypothesis->bVisibility)
					//{
					//	bCompleted = TRUE;

					//	break;
					//}
					//else
					//{
					//	visibility = 100 * pHypothesis->visible / (pSPSuLM->m_Evidence + pMPSuLM->m_Evidence);

					//	if(visibility >= m_VisibilityThr)
					//	{
					//		pHypothesis->bVisibility = 1;

					//		cost = (100 - visibility) * pHypothesis->cost / visibility;
					//	}
					//	else
					//	{
					
							//*ppHypothesis = pHypothesis->pNext;		// I think that this line can be removed. (try it!)


							// remove the hypothesis from ListArray[Cost]
							if(pHypothesis->pNext == NULL)
							{
								ListArray[Cost].ppNext = ppHypothesis;

								*ppHypothesis = NULL;
							}
							else
								*ppHypothesis = pHypothesis->pNext;

							/////

							pHypothesis = (RVLPSULM_HYPOTHESIS *)(pHypothesis->pNext);

							continue;
					//	}
					//}
				}
				else
				{
					//Scene to model
					cost = MatchSurface(pS3DSurface, pMPSuLM, &(pHypothesis->PoseSM), FALSE);
					pHypothesis->iS++;

					//Model to scene
					cost += MatchSurface(pM3DSurface, pSPSuLM, &(pHypothesis->PoseSM), TRUE);
					pHypothesis->iM++;
				}

				pNextHypothesis = (RVLPSULM_HYPOTHESIS *)(pHypothesis->pNext);

				if(cost)
				{
					//increase cost of Hypothesis
					cost += pHypothesis->cost;

					//***
					if(cost > maxCost)
						cost = maxCost;

					// remove the hypothesis from ListArray[Cost]
					if(pHypothesis->pNext == NULL)
					{
						ListArray[Cost].ppNext = ppHypothesis;

						*ppHypothesis = NULL;
					}
					else
						*ppHypothesis = pHypothesis->pNext;

					// append the hypothesis to ListArray[cost] 
					pHypothesis->cost = cost;
					RVLQLISTARRAY_ADD_ENTRY(ListArray, cost, pHypothesis);

					
				}
				else
					ppHypothesis = &(pHypothesis->pNext);
			
				pHypothesis = pNextHypothesis;
			}	// while(pHypothesis)

		}	// while(!bCompleted)


		Cost++;

		if(Cost > maxCost)
			break;
	}	// while(!bCompleted)


	m_pBestHypothesis = (bCompleted ? pHypothesis : NULL);

#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
	CRVL3DPose NullPose;
	CRVLFigure *pFigM;
	CRVLFigure *pFig2;
	char Text[201];
	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackDataM;

	if(m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_DISPLAY)
	{
		RVLNULL3VECTOR(NullPose.m_X);
		RVLUNITMX3(NullPose.m_Rot);

		pSPSuLM->Display(m_DebugData.pGUI, &NullPose, "MPSuLM");

		pFigM = m_DebugData.pGUI->OpenFigure("MPSuLM");

		pMouseCallbackDataM = (RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFigM->m_vpMouseCallbackData);

		//pSPSuLM->Display(m_DebugData.pGUI, &NullPose, "SPSuLM");

		//CRVLFigure *pFigS = m_DebugData.pGUI->OpenFigure("SPSuLM");

		//RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackDataS = 
		//	(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFigS->m_vpMouseCallbackData);

		pFig2 = m_DebugData.pGUI->OpenFigure("Hypothesis");	

		pFigM->m_FontSize = pFig2->m_FontSize = 16;

		int nTextLines = 22;

		int TextImageHeight = nTextLines * pFig2->m_FontSize;

		pFig2->EmptyBitmap(cvSize(800, TextImageHeight), cvScalar(255, 255, 255));

		cvInitFont(&(pFig2->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

		cvInitFont(&(pFigM->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
	}
#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
	FILE *fpHypScoreAnalysis = fopen("C:\\RVL\\Debug\\HypScoreAnalysis.dat", "w");
#endif

	StartTime = m_pTimer->GetTime();

	DWORD EvaluateHypothesisFlags = 0x00000000;

	int i;
	int cost = 0;
	int minCost;
	double maxP = 0.0;

	if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM)
		minCost = 1e8; //Search for min cost
	else if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SM || HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
		minCost = 0; //Search for max cost
	else if (HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
	{
		if ((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
			UpdateBuffers(pPrevSPSuLM);

		if (m_Flags2 & RVLPSULMBUILDER_FLAG2_FIRST_ORDER_DEPENDENCY_TREE)
		{
			m_PriorProbabilityWorldModel = ConditionalProbabilityTree(pSPSuLM);

			EvaluateHypothesisFlags = RVLPSULMBUILDER_HYPEVAL4_FLAG_FIRST_ORDER_DEPENDENCY_TREE;
		}
		else
		{
			m_PriorProbabilityWorldModel = 0.0;

			EvaluateHypothesisFlags = 0x00000000;
		}
	}

	if (m_Flags2 & RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING)
		InitHypothesisEvaluation3(pSPSuLM);

	RVLPSULM_HYPOTHESIS *pBestHypothesis = NULL;

	double P = 0.0;

	int nSignificantHypotheses = 0;

	CRVLPSuLM *pMPSuLM;
	CRVL3DSurface2 *pM3DSurface;
	CRVL3DSurface2 *pS3DSurface;
	CRVL3DPose *pPoseSM;
	CRVL3DPose PoseMS;
	int score;
	int minSignificantScore;

	if(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_PREEVAL)
	{
		int maxScore = 0;		

		m_HypothesisList.Start();

		while(m_HypothesisList.m_pNext)
		{
			pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

			pPoseSM = &(pHypothesis->PoseSM);

			pMPSuLM = pHypothesis->pMPSuLM;

			score = 0;	//reset cost

			for(i=0;i<pSPSuLM->m_n3DSurfaces;i++)
			{
				pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

				if(pMPSuLM->Match2(pS3DSurface, pPoseSM))
					score++;
			}

			pHypothesis->cost = score;

			if(score > maxScore)
				maxScore = score;
		}

		minSignificantScore = maxScore - 2;

		if(minSignificantScore < 1)
			minSignificantScore = 1;
		//int minSignificantScore = 1;

		//FILE *fpPreEval = fopen("C:\\RVL\\Debug\\PreEval.dat", "a");
	}

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
	m_fpDebug = fpHypScoreAnalysis;
#endif 

	m_HypothesisList.Start();

	while(m_HypothesisList.m_pNext)
	{
		pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

		// only for debugging purpose !!!

		//cvSet(pFig->m_pImage, cvScalar(0, 0, 0));

		//pSPSuLM->Display(pFig, &(pHypothesis->PoseSM), cvScalar(0, 255, 0), 
		//	RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_CLEAR | RVLPSULM_DISPLAY_CELLS);

		////pSPSuLM->Project(&(pHypothesis->PoseSM));

		//RVLDisplay2DRegions(pFig, &(m_pAImage->m_C2DRegion.m_ObjectList), m_pCamera->Width, 
		//	RVLColor(255, 0, 0), RVLDELAUNAY_CONNECTION_FLAG_EDGE, RVLDELAUNAY_CONNECTION_FLAG_EDGE);	

		//m_DebugData.pGUI->DisplayVectors(pFig, 0, 0, 1.0);

		//RVLDisplayCellArray(pFig, pSPSuLM->m_CellArray, m_nCells, m_nCols, m_CellSize, cvScalar(255, 255, 0), 1);

		//m_DebugData.pGUI->ShowFigure(pFig);

		//cvWaitKey();

		//short maxe = 0;

		//int iRow, iCol, iCell;
		//short e;

		//for(iRow = 1; iRow < m_nRows - 1; iRow++)
		//	for(iCol = 1; iCol < m_nCols - 1; iCol++)
		//	{
		//		iCell = iRow * m_nCols + iCol;

		//		if(pSPSuLM->m_CellArray[iCell].disparity == 0 && m_CellArray[iCell].disparity == -1)
		//			continue;
		//		
		//		e = abs(pSPSuLM->m_CellArray[iCell].disparity - m_CellArray[iCell].disparity);

		//		if(e > maxe)
		//			maxe = e;
		//	}

		//return;

		/////

		//if (pHypothesis->Index == 359)
		//	int debug = 0;

		pMPSuLM = pHypothesis->pMPSuLM;

		pPoseSM = &(pHypothesis->PoseSM);

		InverseTransform3D(PoseMS.m_Rot, PoseMS.m_X, pPoseSM->m_Rot, pPoseSM->m_X);

		switch(HypEvalMethod){
		case RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM:
			cost = EvaluateHypothesis2(pSPSuLM,pHypothesis); //Calculate cost

			break;
		case RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SM:
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
			fprintf(fpHypScoreAnalysis, "Hypothesis%d\n", pHypothesis->Index);
#endif
			cost = 0;//reset cost

			//compare each Scene surface with the model
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
			if(m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_DISPLAY)
				memcpy(m_CellArray, m_EmptyCellArray, m_nCells * sizeof(RVLPSULM_CELL));
#endif
			for(i=0;i<pSPSuLM->m_n3DSurfaces;i++)
			{
	//#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
	//			memcpy(m_CellArray, m_EmptyCellArray, m_nCells * sizeof(RVLPSULM_CELL));
	//#endif

				pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];
				//cost += MatchSurface(pS3DSurface, pMPSuLM, &(pHypothesis->PoseSM), FALSE);
				//cost += pMPSuLM->Match(pS3DSurface, &PoseMS);

				//if (m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL)	//Matching with geomentry and color - FILKO
				//	score = pMPSuLM->MatchGeometryAndColor(pS3DSurface, pPoseSM, 5, m_IntersectionHelperArray);
				//else
					score = pMPSuLM->Match2(pS3DSurface, pPoseSM);

				cost += score;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
				fprintf(fpHypScoreAnalysis, "S%d %d\n", pS3DSurface->m_Index, score);
#endif

	//#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
	//			// match visualization surface by surface

	//			RVLCOPY3VECTOR(NullPose.m_X, pMouseCallbackData->PoseM0.m_X)
	//			RVLCOPYMX3X3(NullPose.m_Rot, pMouseCallbackData->PoseM0.m_Rot)
	//
	//			RVLCOPY3VECTOR(pPoseSM->m_X, pMouseCallbackData->PoseS0.m_X)
	//			RVLCOPYMX3X3(pPoseSM->m_Rot, pMouseCallbackData->PoseS0.m_Rot)
	//
	//			pMouseCallbackData->Flags = (RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_MCELLS | RVLPSULM_DISPLAY_PROJ_MCELLS);
	//
	//			RVLPSuLMDisplay(pFig);
	//
	//			m_DebugData.pGUI->ShowFigure(pFig);
	//
	//			cvWaitKey();
	//#endif
			}

			if(m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_BIDIRECT_HYP_EVAL)
			{
				//compare each Model surface with the scene
				for(i=0;i<pMPSuLM->m_n3DSurfaces;i++)
				{
//#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
//					memcpy(m_CellArray, m_EmptyCellArray, m_nCells * sizeof(RVLPSULM_CELL));
//#endif
					pM3DSurface = pMPSuLM->m_3DSurfaceArray[i];
					//cost += MatchSurface(pM3DSurface, pSPSuLM, &(pHypothesis->PoseSM), TRUE);
					cost += pSPSuLM->Match(pM3DSurface, pPoseSM);

//#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
					//// match visualization surface by surface
					//RVLCOPY3VECTOR(NullPose.m_X, pMouseCallbackData->PoseS0.m_X)
					//RVLCOPYMX3X3(NullPose.m_Rot, pMouseCallbackData->PoseS0.m_Rot)

					//RVLCOPY3VECTOR(PoseMS.m_X, pMouseCallbackData->PoseM0.m_X)
					//RVLCOPYMX3X3(PoseMS.m_Rot, pMouseCallbackData->PoseM0.m_Rot)

					//pMouseCallbackData->Flags = (RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_SCELLS | RVLPSULM_DISPLAY_PROJ_SCELLS);

					//RVLPSuLMDisplay(pFig);

					//m_DebugData.pGUI->ShowFigure(pFig);

					//cvWaitKey();
	//#endif
				}
			}

			break;
		case RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM:
			if(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_PREEVAL)
			{
				if(pHypothesis->cost >= minSignificantScore)
				{
					cost = EvaluateHypothesis3(pSPSuLM,pHypothesis); //Calculate cost

					nSignificantHypotheses++;
				}
				else
					cost  = 0;
			}
			else
				cost = EvaluateHypothesis3(pSPSuLM,pHypothesis); //Calculate cost

			break;
		case RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P:
			P = EvaluateHypothesis4(pSPSuLM, pHypothesis, EvaluateHypothesisFlags);
		}

		//ADDED BY FILKO - ALI NE TAKO ????
		//if (m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL)
		//	cost += (int)GetColorSceneSimilarityScore(pMPSuLM, pSPSuLM);

		//for(i=0;i<pMPSuLM->m_n3DSurfaces;i++)
		//	pMPSuLM->m_3DSurfaceArray[i]->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
		fprintf(fpHypScoreAnalysis, "\n");
#endif


//#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
//		// hypothesis visualization
//
//		pMouseCallbackDataM->pMPSuLM = pMPSuLM;
//		pMouseCallbackDataS->pMPSuLM = pMPSuLM;
//
//		RVLCOPY3VECTOR(NullPose.m_X, pMouseCallbackDataM->PoseM0.m_X)
//		RVLCOPYMX3X3(NullPose.m_Rot, pMouseCallbackDataM->PoseM0.m_Rot)
//
//		RVLCOPY3VECTOR(pPoseSM->m_X, pMouseCallbackDataM->PoseS0.m_X)
//		RVLCOPYMX3X3(pPoseSM->m_Rot, pMouseCallbackDataM->PoseS0.m_Rot)
//
//		pMouseCallbackDataM->Flags = (RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_MCELLS | RVLPSULM_DISPLAY_PROJ_MCELLS);
//
//		RVLPSuLMDisplay(pFigM);
//
//		m_DebugData.pGUI->ShowFigure(pFigM);
//
//		//cvWaitKey();
//#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
		if(m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_DISPLAY)
			memcpy(m_CellArray, m_EmptyCellArray, m_nCells * sizeof(RVLPSULM_CELL));
#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
		// hypothesis visualization

		RVLCOPY3VECTOR(NullPose.m_X, pMouseCallbackDataS->PoseS0.m_X)
		RVLCOPYMX3X3(NullPose.m_Rot, pMouseCallbackDataS->PoseS0.m_Rot)

		RVLCOPY3VECTOR(PoseMS.m_X, pMouseCallbackDataS->PoseM0.m_X)
		RVLCOPYMX3X3(PoseMS.m_Rot, pMouseCallbackDataS->PoseM0.m_Rot)

		pMouseCallbackDataS->Flags = (RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_SCELLS | RVLPSULM_DISPLAY_PROJ_SCELLS);

		RVLPSuLMDisplay(pFigS);

		m_DebugData.pGUI->ShowFigure(pFigS);

		cvWaitKey();
#endif

		if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
		{
			pHypothesis->Probability = P - m_PriorProbabilityWorldModel;

			pHypothesis->cost = DOUBLE2INT(1000.0 * pHypothesis->Probability);
		}
		else
			pHypothesis->cost = cost;

		//Store best value

		if (HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM)
		{
			//Search for min cost
			if(cost < minCost)
			{
				minCost = cost;
				pBestHypothesis = pHypothesis;
			}
		}
		else if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SM || HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
		{
			//Search for max cost
			if(cost > minCost)
			{
				minCost = cost;
				pBestHypothesis = pHypothesis;
			}
		}
		else
		{
			//Search for max probability

			if(P > maxP)
			{
				maxP = P;
				pBestHypothesis = pHypothesis;
			}
		}
	}	// for each hypothesis

#ifdef NEVER	// old method for identifying representative hypotheses
	// identify representative hypotheses

	RVLQLIST *pList = &m_RepresentativeHypothesisList;

	RVLQLIST_INIT(pList)

	if(m_RepresentativeHypothesisMem)
		delete[] m_RepresentativeHypothesisMem;

	m_RepresentativeHypothesisMem = new RVLQLIST_PTR_ENTRY[m_HypothesisList.m_nElements];

	RVLQLIST_PTR_ENTRY *pNewHypothesisPtr = m_RepresentativeHypothesisMem;

	CRVL3DPose dPoseSM;

	double *RSM, *RSM_, *RM_M;
	double *tSM, *tSM_, *tM_M;
	double RSM2[3*3];
	double tSM2[3];
	double *dRSM = dPoseSM.m_Rot;
	double *dtSM = dPoseSM.m_X;
	RVLPSULM_HYPOTHESIS *pHypothesis_;
	double V3Tmp[3];
	double theta;
	bool bNewRepresentative, bAddToRepresentatives;
	RVLQLIST_PTR_ENTRY *pHypothesisPtr;
	void **ppHypothesisPtr;
	CRVLPSuLM *pMPSuLM_, *pMPSuLM__;
	RVLPSULM_NEIGHBOUR *pNeighborRel;
	RVLQLIST_PTR_ENTRY *pNeighborPtr;

	m_HypothesisList.Start();

	while(m_HypothesisList.m_pNext)
	{
		pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

		pMPSuLM = pHypothesis->pMPSuLM;

		pHypothesis->iRepresentative = 0xffffffff;

		RSM = pHypothesis->PoseSM.m_Rot;
		tSM = pHypothesis->PoseSM.m_X;

		//bNewRepresentative = true;

		bAddToRepresentatives = true;

		ppHypothesisPtr = &(m_RepresentativeHypothesisList.pFirst);

		pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(m_RepresentativeHypothesisList.pFirst);

		while(pHypothesisPtr)
		{
			pHypothesis_ = (RVLPSULM_HYPOTHESIS *)(pHypothesisPtr->Ptr);

			pMPSuLM_ = pHypothesis_->pMPSuLM;

			RSM_ = pHypothesis_->PoseSM.m_Rot;
			tSM_ = pHypothesis_->PoseSM.m_X;

			if(!(m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING))
			{
				if(pMPSuLM_ != pMPSuLM)
				{
					pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pMPSuLM->m_NeighbourList->pFirst);

					while(pNeighborPtr)
					{
						pNeighborRel = (RVLPSULM_NEIGHBOUR *)(pNeighborPtr->Ptr);

						pMPSuLM__ = pNeighborRel->pPSuLM;

						if(pMPSuLM_ == pMPSuLM__)
							break;

						pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);
					}

					if(pNeighborPtr == NULL)
					{
						ppHypothesisPtr = &(pHypothesisPtr->pNext);

						pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);

						continue;
					}

					RM_M = pNeighborRel->pPoseRel->m_Rot;
					tM_M = pNeighborRel->pPoseRel->m_X;

					RVLCOMPTRANSF3D(RM_M, tM_M, RSM_, tSM_, RSM2, tSM2)

					RSM_ = RSM2;
					tSM_ = tSM2;
				}
			}
			
			RVLDIF3VECTORS(tSM, tSM_, dtSM)

			if(RVLABS(dtSM[0]) > 200.0)
			{
				ppHypothesisPtr = &(pHypothesisPtr->pNext);

				pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);

				continue;
			}

			if(RVLABS(dtSM[1]) > 200.0)
			{
				ppHypothesisPtr = &(pHypothesisPtr->pNext);

				pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);

				continue;
			}

			if(RVLABS(dtSM[2]) > 200.0)
			{
				ppHypothesisPtr = &(pHypothesisPtr->pNext);

				pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);

				continue;
			}			
			
			RVLMXMUL3X3T1(RSM, RSM_, dRSM)

			dPoseSM.GetAngleAxis(V3Tmp, theta);

			if(theta > 10.0 * DEG2RAD)
			{
				ppHypothesisPtr = &(pHypothesisPtr->pNext);

				pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);

				continue;
			}

			if (HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM)
				bNewRepresentative = (pHypothesis->cost < pHypothesis_->cost);
			else if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SM || HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
				bNewRepresentative = (pHypothesis->cost > pHypothesis_->cost);
			else
				bNewRepresentative = (pHypothesis->Probability > pHypothesis_->Probability);

			if(bNewRepresentative)
			{
				pHypothesis_->iRepresentative = pHypothesis->Index;

				RVLQLIST_REMOVE_ENTRY(pList, pHypothesisPtr, ppHypothesisPtr)	

				pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);
			}
			else
			{
				pHypothesis->iRepresentative = pHypothesis_->Index;

				bAddToRepresentatives = false;

				break;
			}			
		}	// while(pHypothesisPtr)

		if(bAddToRepresentatives)
		{
			RVLQLIST_ADD_ENTRY(pList, pNewHypothesisPtr)

			pNewHypothesisPtr->Ptr = pHypothesis;

			pNewHypothesisPtr++;
		}
	}	// while(m_HypothesisList.m_pNext)

	m_nRepresentativeHypotheses = pNewHypothesisPtr - m_RepresentativeHypothesisMem;
#endif	// old method for detecting representative hypotheses

	//if(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_PREEVAL)
	//{
	//	fprintf(fpPreEval, "%d\t%d\n", nSignificantHypotheses, m_HypothesisList.m_nElements);

	//	fclose(fpPreEval);
	//}

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_MATCH_COUNT
	if((m_Flags & 
		(RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM | RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_SSM_NORM)) == 
		(RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM | RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_SSM_NORM)
		&& minCost > 0)
	{
		// compute final scores

		double *dPSuLMProbabilityArray = new double[m_maxPSuLMIndex + 1];

		memset(dPSuLMProbabilityArray, 0, (m_maxPSuLMIndex + 1) * sizeof(double));

		double *dHypothesisProbabilityArray = new double[m_HypothesisList.m_nElements];

		double *pdHypothesisProbability = dHypothesisProbabilityArray;

		double *pdPSuLMProbability;

		double dProbability;

		m_HypothesisList.Start();

		while(m_HypothesisList.m_pNext)
		{
			pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

			dProbability = exp(0.001 * (double)(pHypothesis->cost - minCost));

			*(pdHypothesisProbability++) = dProbability;

			pdPSuLMProbability = dPSuLMProbabilityArray + pHypothesis->pMPSuLM->m_Index;

			if(dProbability > (*pdPSuLMProbability))
				*pdPSuLMProbability = dProbability;
		}

		double TotalScore = 0;

		double *dPSuLMProbabilityArrayEnd = dPSuLMProbabilityArray + m_maxPSuLMIndex + 1;

		for(pdPSuLMProbability = dPSuLMProbabilityArray; pdPSuLMProbability < dPSuLMProbabilityArrayEnd; pdPSuLMProbability++)
			TotalScore += (*pdPSuLMProbability);

		delete[] dPSuLMProbabilityArray;

		pdHypothesisProbability = dHypothesisProbabilityArray;

		m_HypothesisList.Start();

		while(m_HypothesisList.m_pNext)
		{
			pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

			pHypothesis->cost = DOUBLE2INT(1000.0 * ((*(pdHypothesisProbability++)) / TotalScore));
		}

		delete[] dHypothesisProbabilityArray;
	}
#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
	fclose(fpHypScoreAnalysis);
#endif

#pragma endregion

	m_HypEvalTime = m_pTimer->GetTime() - StartTime;

	//StartTime = m_pTimer->GetTime();

	//RVLPSULM_HYPOTHESIS **ppParticle = m_HypothesisArray;

	// sort hypotheses

	if(m_HypothesisArray)
		delete[] m_HypothesisArray;

	m_HypothesisArray = new RVLPSULM_HYPOTHESIS *[m_HypothesisList.m_nElements];

	RVLBubbleSort<RVLPSULM_HYPOTHESIS>(&m_HypothesisList, m_HypothesisArray, HypEvalMethod != RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM);

#ifdef RVLPSULMBUILDER_HYPOTHESIS_LOG
	FILE *fpHyp = fopen("C:\\RVL\\Debug\\Hypotheses0.log", "w");

	for(i = 0; i < m_nHypotheses; i++)
	{
		pHypothesis = m_HypothesisArray[i];

		fprintf(fpHyp, "%d: H\t%d\tM\t%d\tP\t%lf\n", i, pHypothesis->Index, pHypothesis->pMPSuLM->m_Index, (double)(pHypothesis->cost) * 1e-3);
	}

	fclose(fpHyp);
#endif

	if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
	{
		// compute the local and global posterior probability of the best hypothesis for each PSuLM

		double BestHypothesisProbability = m_HypothesisArray[0]->Probability;

		m_PriorProbabilityGlobal = 0.0;

		if(m_nHypotheses > 0)
		{	
			RepresentativeHypotheses();			

			RVLMEM_ALLOC_LOCAL_INIT(m_pMem2)

			CRVLMPtrChain MPSuLMList(m_pMem2);

			RVLMEM_ALLOC_LOCAL_UPDATE(m_pMem2);

			CRVLMPtrChain *pMPSuLMList;

			if(m_Flags & RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
			{
				if(m_PSuLMSubList.m_nElements > 0)
					pMPSuLMList = &m_PSuLMSubList;
				else
					pMPSuLMList = &m_PSuLMList;
			}
			else
			{
				MPSuLMList.Add(pPrevSPSuLM);

				pMPSuLMList = &MPSuLMList;
			}

			m_PriorProbabilityGlobal = 0.0;

			pMPSuLMList->Start();

			while(pMPSuLMList->m_pNext)
			{
				pMPSuLM = (CRVLPSuLM *)(pMPSuLMList->GetNext());

				pMPSuLM->m_PriorProbabilityLocal = 0.0;

				pMPSuLM->m_pHypothesis = NULL;
			}

			//m_nHypotheses = 0;

			//FILE *fpDebug = fopen("C:\\RVL\\Debug\\RepresentativeHypotheses.txt", "w");

			RVLQLIST_PTR_ENTRY *pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(m_RepresentativeHypothesisList.pFirst);

			while(pHypothesisPtr)
			{
				pHypothesis = (RVLPSULM_HYPOTHESIS *)(pHypothesisPtr->Ptr);

				//fprintf(fpDebug, "%d\n", pHypothesis->Index);

				m_PriorProbabilityGlobal += exp(pHypothesis->Probability - BestHypothesisProbability);

				pMPSuLM = pHypothesis->pMPSuLM;

				if(pMPSuLM->m_pHypothesis)
				{
					if(pHypothesis->Probability > pMPSuLM->m_pHypothesis->Probability)
						pMPSuLM->m_pHypothesis = pHypothesis;
				}
				else
					pMPSuLM->m_pHypothesis = pHypothesis;

				pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);

				//m_nHypotheses++;
			}

			//fclose(fpDebug);

			pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(m_RepresentativeHypothesisList.pFirst);

			while(pHypothesisPtr)
			{
				pHypothesis = (RVLPSULM_HYPOTHESIS *)(pHypothesisPtr->Ptr);

				//if(pHypothesis->iRepresentative != 0xffffffff)
				//	int debug = 0;

				pHypothesis->pMPSuLM->m_PriorProbabilityLocal += exp(pHypothesis->Probability - pHypothesis->pMPSuLM->m_pHypothesis->Probability);

				pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);
			}

			pMPSuLMList->Start();

			while(pMPSuLMList->m_pNext)
			{
				pMPSuLM = (CRVLPSuLM *)(pMPSuLMList->GetNext());

				if(pMPSuLM->m_pHypothesis)
				{
					//int nSFeatures = pSPSuLM->m_n3DSurfaces + pSPSuLM->m_n3DLines;

					//for(i = 0; i < nSFeatures; i++)
					//	m_SMatchArray[i].b = false;

					//m_PriorProbabilityWorldModel = ConditionalProbabilityTree(pSPSuLM, pMPSuLM);
				
					pMPSuLM->m_PriorProbabilityLocal += exp(-pMPSuLM->m_pHypothesis->Probability);

					//pMPSuLM->m_PosteriorProbabilityLocal5DOF = 1.0 / pMPSuLM->m_PriorProbabilityLocal;
					//pMPSuLM->m_PosteriorProbabilityLocal5DOF = pMPSuLM->m_pHypothesis->Probability - m_PriorProbabilityWorldModel;
					pMPSuLM->m_PosteriorProbabilityLocal5DOF = pMPSuLM->m_pHypothesis->Probability;

					PoseConstraintProbability(pSPSuLM, pMPSuLM);

					pMPSuLM->m_PosteriorProbabilityGlobal = exp(pMPSuLM->m_pHypothesis->Probability - BestHypothesisProbability) / m_PriorProbabilityGlobal;
				}
				else
					pMPSuLM->m_PosteriorProbabilityLocal5DOF = pMPSuLM->m_PosteriorProbabilityLocal = pMPSuLM->m_PosteriorProbabilityGlobal = 0.0;
			}

			//RVLMEM_ALLOC_LOCAL_FREE(m_pMem2)		

			//pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(m_RepresentativeHypothesisList.pFirst);

			//double fTmp = BestHypothesisProbability + log(m_PriorProbabilityGlobal);
			double fTmp = 0.0;

			//while(pHypothesisPtr)
			for(i = 0; i < m_nHypotheses; i++)
			{
				//pHypothesis = (RVLPSULM_HYPOTHESIS *)(pHypothesisPtr->Ptr);
				pHypothesis = m_HypothesisArray[i];

				pHypothesis->cost = DOUBLE2INT(1e3 * (pHypothesis->Probability - fTmp));

				//pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);
			}	

			m_nHypotheses = m_nRepresentativeHypotheses;

			RVLBubbleSort<RVLPSULM_HYPOTHESIS>(&m_RepresentativeHypothesisList, m_nHypotheses, &m_HypothesisArray, true);

			// recompute the costs of the hypotheses after rejection of dynamic surfaces and reorder the hypotheses

			double StartTime = m_pTimer->GetTime();

			if((m_Flags2 & RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING) && m_nHypotheses > 1)
			{
				int iChecked = m_nHypotheses;

				while (iChecked > 0)
				{
					pHypothesis = m_HypothesisArray[0];					

					P = EvaluateHypothesis4(pSPSuLM, pHypothesis, EvaluateHypothesisFlags | RVLPSULMBUILDER_HYPEVAL4_FLAG_DYNAMIC_SURF_DETECT);

					pHypothesis->Probability = P - m_PriorProbabilityWorldModel;

					pHypothesis->cost = DOUBLE2INT(1e3 * (pHypothesis->Probability - fTmp));

					for (i = 1; i < m_nHypotheses; i++)
						if (m_HypothesisArray[i]->cost <= pHypothesis->cost)
							break;

					if (i == 1)
						break;

					i--;

					memmove(m_HypothesisArray, m_HypothesisArray + 1, i * sizeof(RVLPSULM_HYPOTHESIS *));

					m_HypothesisArray[i] = pHypothesis;

					if (i < iChecked)
						iChecked = i;
					else
						iChecked--;
				}
			}

			double DynSurfRejectionExecutionTime = m_pTimer->GetTime() - StartTime;

			FILE *fp = fopen("C:\\RVL\\Debug\\debug_.txt", "w");

			fprintf(fp, "DynSurfRejectionExecutionTime = %lf\n", DynSurfRejectionExecutionTime);

			fclose(fp);

#ifdef RVLPSULMBUILDER_HYPOTHESIS_LOG
			fpHyp = fopen("C:\\RVL\\Debug\\Hypotheses.log", "w");

			for(i = 0; i < m_nHypotheses; i++)
			{
				pHypothesis = m_HypothesisArray[i];

				fprintf(fpHyp, "%d: H\t%d\tM\t%d\tP\t%lf\n", i, pHypothesis->Index, pHypothesis->pMPSuLM->m_Index, (double)(pHypothesis->cost) * 1e-3);
			}

			fclose(fpHyp);
#endif
		}	// if(m_nHypotheses > 0)

		// scene fusion

		if(m_Flags2 & RVLPSULMBUILDER_FLAG2_SCENE_FUSION)
			SceneFusion();			
	}	// if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)		

	//*(ppParticle++) = pBestHypothesis;

	//m_nHypotheses = ppParticle - m_HypothesisArray;

	//*************************************************************
	//
	//     FINAL POSE SELECTION (CORRECTION)
	//
	//*************************************************************

	// determine the number of plausible hypotheses and compute hypothesis poses in 3DOF

	RVLPSULM_HYPOTHESIS **pHypothesisArrayEnd = m_HypothesisArray + m_nHypotheses;

	RVLPSULM_HYPOTHESIS **ppHypothesis;
	double CCshCmh[3 * 3 *3];
	int Scost = 0;

	for(ppHypothesis = m_HypothesisArray; ppHypothesis < pHypothesisArrayEnd; ppHypothesis++)
	{
		pHypothesis = *ppHypothesis;

		if(pHypothesis->cost < m_PlausibilityThr)
			break;

		Scost += (pHypothesis->cost / m_HypothesisScoreForOneParticle);

		RVLDeterminePose6DOFTo3DOF(&(pHypothesis->PoseSM3DOF), &(pHypothesis->PoseSM),m_pPoseAC);
		PanTiltRollUncertainty(CCshCmh,pHypothesis->P, &(pHypothesis->PoseSM),true);
		DetermineUncertainty6DOFTo3DOF(pHypothesis->P3DOF, CCshCmh, &(pHypothesis->PoseSM3DOF));
		pHypothesis->PoseSM3DOF.m_C = pHypothesis->P3DOF;
		pHypothesis->PoseSM3DOF.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_3D;
	}

	m_nPlausibleHypotheses = ppHypothesis - m_HypothesisArray;

	pHypothesisArrayEnd = ppHypothesis;

	RVLPSULM_PARTICLE *pParticle;

	if(	(m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION &&
		(m_Flags & RVLPSULMBUILDER_FLAG_PARTICLE_FILTER) != 0 && 
		(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
	{
#pragma region Multiple Hypothesis Tracking

		m_BestParticleCbV.pMPSuLM = NULL;

		m_nParticlesCbV = 0;

		CRVL3DPose *pPoseAspAmp;
		CRVL3DPose *pPoseAshAmh;			
		double *RAshAmh, *tAshAmh;

		if(m_nParticles == 0)
		{
			m_BestParticle.pMPSuLM = NULL;

			m_nParticles = Scost;

			if(m_nPlausibleHypotheses > 0)
			{
				if(m_ParticleArray)
					delete[] m_ParticleArray;

				pParticle = m_ParticleArray = new RVLPSULM_PARTICLE[m_nParticles];

				if(m_ParticleCbVArray)
					delete[] m_ParticleCbVArray;

				RVLPSULM_PARTICLE *pParticleCbV = m_ParticleCbVArray = new RVLPSULM_PARTICLE[m_nPlausibleHypotheses];

				for(ppHypothesis = m_HypothesisArray; ppHypothesis < pHypothesisArrayEnd; ppHypothesis++)
				{
					pHypothesis = *ppHypothesis;

					pMPSuLM = pHypothesis->pMPSuLM;

					if((pMPSuLM->m_Flags & RVLPSULM_FLAG_VISITED) == 0)
					{
						pMPSuLM->m_Flags |= RVLPSULM_FLAG_VISITED;

						*(pParticleCbV++) = *pParticle;
					}

					for(i = 0; i < pHypothesis->cost / m_HypothesisScoreForOneParticle; i++)
					{
						pParticle->pMPSuLM = pHypothesis->pMPSuLM;

						pPoseAspAmp = &(pParticle->PoseSM);

						RAspAmp = pPoseAspAmp->m_Rot;
						tAspAmp = pPoseAspAmp->m_X;

						pPoseAshAmh = &(pHypothesis->PoseSM3DOF);

						RAshAmh = pPoseAshAmh->m_Rot;
						tAshAmh = pPoseAshAmh->m_X;

						RVLCOPYMX3X3(RAshAmh, RAspAmp)
						RVLCOPY3VECTOR(tAshAmh, tAspAmp)

						pParticle->Flags = 0x00;

						pParticle->w = 0;

						pParticle++;
					}
				}

				m_BestParticle = m_BestParticleCbV = m_ParticleArray[0];

				RVLResetFlags<CRVLPSuLM>(&m_PSuLMList, RVLPSULM_FLAG_VISITED);

				m_nParticlesCbV = pParticleCbV - m_ParticleCbVArray;
			}
			else
				m_nParticlesCbV = 0;
		}
		else	// if(m_nParticles > 0)
		{
			// assigning particles to hypotheses and weights to particles

#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
			RVLPSULM_PARTICLE *ParticleBuff = new RVLPSULM_PARTICLE[m_nPlausibleHypotheses + m_nParticles];

			RVLPSULM_PARTICLE *pParticle3 = ParticleBuff;

			for(ppHypothesis = m_HypothesisArray; ppHypothesis < pHypothesisArrayEnd; ppHypothesis++, pParticle3++)
			{
				pHypothesis = *ppHypothesis;

				pHypothesis->pParticle = pParticle3;

				pParticle3->w = 0;

				pParticle3->pHypothesis = pHypothesis;
			}
#else
			RVLPSULM_PARTICLE *ParticleBuff = new RVLPSULM_PARTICLE[(m_nPlausibleHypotheses + 1) * m_nParticles];

			RVLPSULM_PARTICLE *pParticle3 = ParticleBuff;
#endif

			pParticleArrayEnd = m_ParticleArray + m_nParticles;

			int wTotal = 0;

			RVLQLIST *HT = m_LocalMapHT.m_ListArray;
			int HTSize = m_LocalMapHT.m_Size;

			m_PSuLMSubList.RemoveAll();

			double *CAshAmh;
			CRVL3DPose *pPoseAmpAmh;
			double *RAmpAmh, *tAmpAmh, *CAmpAmh;
			double RAspAmh[3 * 3], tAspAmh[3];
			double C[3 * 3], invC[3 * 3], e[3];
			DWORD HTadr;
			RVLQLISTHT_PTR_ENTRY *pHTEntry;
			RVLQLIST *pListTmp;
			RVLPSULM_NEIGHBOR2 *pNeighbor;
			double dM, detC;
			//double dE2;
			RVLPSULM_PARTICLE2 *pParticle2;
			RVLPSULM_PARTICLE *pParticleBuffEnd;
			int wacc;
#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
			RVLPSULM_HYPOTHESIS **ppHypothesis2;
			RVLPSULM_HYPOTHESIS *pHypothesis2;
#endif

			for(ppHypothesis = m_HypothesisArray; ppHypothesis < pHypothesisArrayEnd; ppHypothesis++, pParticle++)
			{
				pHypothesis = *ppHypothesis;

				pPoseAshAmh = &(pHypothesis->PoseSM3DOF);

				RAshAmh = pPoseAshAmh->m_Rot;
				tAshAmh = pPoseAshAmh->m_X;
				CAshAmh = pPoseAshAmh->m_C;

				pMPSuLM = pHypothesis->pMPSuLM;

				if((pMPSuLM->m_Flags & RVLPSULM_FLAG_VISITED) == 0)
#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
					pParticle2 = pMPSuLM->m_ParticleArray = new RVLPSULM_PARTICLE2[m_nParticles + m_nPlausibleHypotheses];
#else
					pParticle2 = pMPSuLM->m_ParticleArray = new RVLPSULM_PARTICLE2[m_nParticles];
#endif

#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
				wacc = 0;
#endif

				for(pParticle = m_ParticleArray; pParticle < pParticleArrayEnd; pParticle++)
				{
					HTadr = pMPSuLM->m_Index * RVLPSULMBUILDER_MAXN_PSULMS + pParticle->pMPSuLM->m_Index;

					RVLQLISTHT_GET_ENTRY(HT, RVLQLISTHT_PTR_ENTRY, HTadr, HTSize, pHTEntry, pListTmp)

					if(pHTEntry == NULL)
						continue;

					pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pHTEntry->Ptr);

					pPoseAmpAmh = &(pNeighbor->PoseRel);

					RAmpAmh = pPoseAmpAmh->m_Rot;
					tAmpAmh = pPoseAmpAmh->m_X;
					CAmpAmh = pPoseAmpAmh->m_C;

					pPoseAspAmp = &(pParticle->PoseSM);

					RAspAmp = pPoseAspAmp->m_Rot;
					tAspAmp = pPoseAspAmp->m_X;

					// TAspAmh <- TAmpAmh * TAspAmp

					RVLCOMPTRANSF3D3DOF(RAmpAmh, tAmpAmh, RAspAmp, tAspAmp, RAspAmh, tAspAmh)

					// record particle distribution relative to pMPSuLM

					if((pMPSuLM->m_Flags & RVLPSULM_FLAG_VISITED) == 0)
					{
						pParticle2->cs = RAspAmh[0];
						pParticle2->sn = RAspAmh[3];
						pParticle2->tx = tAspAmh[0];
						pParticle2->ty = tAspAmh[1];
						pParticle2->pParticle = pParticle;
						pParticle2++;
					}

					// C <- CAmpAmh + CAshAmh

					RVLSUMMX3X3(CAmpAmh, CAshAmh, C)

					// e <- difference between TAspAmh and TAshAmh

					if(RAspAmh[0] * RAshAmh[0] + RAspAmh[3] * RAshAmh[3] < 0.0)
						continue;

					e[0] = -RAspAmh[0] * RAshAmh[3] + RAspAmh[3] * RAshAmh[0];
					e[1] = tAspAmh[0] - tAshAmh[0];
					e[2] = tAspAmh[1] - tAshAmh[1];

					// dM <- e' * inv(C) * e

					RVLINVCOV3(C, invC, detC)
						
					RVLMULCOV3VECT(invC, e, RVLVector3)

					dM = RVLDOTPRODUCT3(e, RVLVector3);

					if(dM > 11.34)
						continue;

#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
					wacc += (pHypothesis->cost / m_HypothesisScoreForOneParticle);
#else
					*pParticle3 = *pParticle;

					pParticle->Flags |= RVLPSULM_PARTICLE_FLAG_ASSIGNED;	

					pParticle->w += pHypothesis->cost;
					
					pParticle3->w = pHypothesis->cost;

					wTotal += pHypothesis->cost;

					//dE2 = tAspAmh[0] * tAspAmh[0] + tAspAmh[1] * tAspAmh[1];

					pParticle3->pMPSuLM = pMPSuLM;

					RAspAmp = pParticle3->PoseSM.m_Rot;
					tAspAmp = pParticle3->PoseSM.m_X;

					RVLCOPYMX3X3(RAspAmh, RAspAmp)
					RVLCOPY3VECTOR(tAspAmh, tAspAmp)

					pParticle3++;
#endif
				}	// for every particle

#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
				if(wacc > 0)
				{
					pParticle = pHypothesis->pParticle;

					pParticle->pMPSuLM = pMPSuLM;

					pPoseAspAmp = &(pParticle->PoseSM);

					RAspAmp = pPoseAspAmp->m_Rot;
					tAspAmp = pPoseAspAmp->m_X;

					RVLCOPYMX3X3(RAshAmh, RAspAmp)
					RVLCOPY3VECTOR(tAshAmh, tAspAmp)

					pParticle->w = wacc;

					wTotal += wacc;

					if((pMPSuLM->m_Flags & RVLPSULM_FLAG_VISITED) == 0)
					{
						for(ppHypothesis2 = m_HypothesisArray; ppHypothesis2 < pHypothesisArrayEnd; ppHypothesis2++)
						{
							pHypothesis2 = *ppHypothesis2;

							HTadr = pMPSuLM->m_Index * RVLPSULMBUILDER_MAXN_PSULMS + pHypothesis2->pMPSuLM->m_Index;

							RVLQLISTHT_GET_ENTRY(HT, RVLQLISTHT_PTR_ENTRY, HTadr, HTSize, pHTEntry, pListTmp)

							if(pHTEntry == NULL)
								continue;

							pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pHTEntry->Ptr);

							pPoseAmpAmh = &(pNeighbor->PoseRel);

							RAmpAmh = pPoseAmpAmh->m_Rot;
							tAmpAmh = pPoseAmpAmh->m_X;

							pPoseAspAmp = &(pHypothesis2->PoseSM3DOF);

							RAspAmp = pPoseAspAmp->m_Rot;
							tAspAmp = pPoseAspAmp->m_X;

							// TAspAmh <- TAmpAmh * TAspAmp

							RVLCOMPTRANSF3D3DOF(RAmpAmh, tAmpAmh, RAspAmp, tAspAmp, RAspAmh, tAspAmh)

							pParticle2->cs = RAspAmh[0];
							pParticle2->sn = RAspAmh[3];
							pParticle2->tx = tAspAmh[0];
							pParticle2->ty = tAspAmh[1];
							pParticle2->pParticle = pHypothesis2->pParticle;
							pParticle2++;												
						}
					}
				}
#endif
				if((pMPSuLM->m_Flags & RVLPSULM_FLAG_VISITED) == 0)
				{
					pMPSuLM->m_nParticles = pParticle2 - pMPSuLM->m_ParticleArray;

					pMPSuLM->m_Flags |= RVLPSULM_FLAG_VISITED;

					m_PSuLMSubList.Add(pMPSuLM);
				}
			}	// for every hypothesis

			for(pParticle = m_ParticleArray; pParticle < pParticleArrayEnd; pParticle++)
#ifndef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
				if((pParticle->Flags & RVLPSULM_PARTICLE_FLAG_ASSIGNED) == 0)
#endif
				{
					*pParticle3 = *pParticle;
					pParticle->w = pParticle3->w = 1;
					pParticle3++;
					wTotal++;
				}

			pParticleBuffEnd = pParticle3;

			// record particle distribution for every PSuLM which is not visited, 
			// but is the base model for at least one particle

			for(pParticle = m_ParticleArray; pParticle < pParticleArrayEnd; pParticle++)
			{
				pMPSuLM = pParticle->pMPSuLM;

				if(pMPSuLM->m_Flags & RVLPSULM_FLAG_VISITED)
					continue;

				pMPSuLM->m_Flags |= RVLPSULM_FLAG_VISITED;

				m_PSuLMSubList.Add(pMPSuLM);

				pParticle2 = pMPSuLM->m_ParticleArray = new RVLPSULM_PARTICLE2[m_nParticles];

				for(pParticle3 = m_ParticleArray; pParticle3 < pParticleArrayEnd; pParticle3++)
				{
					HTadr = pMPSuLM->m_Index * RVLPSULMBUILDER_MAXN_PSULMS + pParticle3->pMPSuLM->m_Index;

					RVLQLISTHT_GET_ENTRY(HT, RVLQLISTHT_PTR_ENTRY, HTadr, HTSize, pHTEntry, pListTmp)

					if(pHTEntry == NULL)
						continue;

					pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pHTEntry->Ptr);

					pPoseAmpAmh = &(pNeighbor->PoseRel);

					RAmpAmh = pPoseAmpAmh->m_Rot;
					tAmpAmh = pPoseAmpAmh->m_X;
					CAmpAmh = pPoseAmpAmh->m_C;

					pPoseAspAmp = &(pParticle3->PoseSM);

					RAspAmp = pPoseAspAmp->m_Rot;
					tAspAmp = pPoseAspAmp->m_X;

					// TAspAmh <- TAmpAmh * TAspAmp

					RVLCOMPTRANSF3D3DOF(RAmpAmh, tAmpAmh, RAspAmp, tAspAmp, RAspAmh, tAspAmh)

					// record particle distribution relative to pMPSuLM

					pParticle2->cs = RAspAmh[0];
					pParticle2->sn = RAspAmh[3];
					pParticle2->tx = tAspAmh[0];
					pParticle2->ty = tAspAmh[1];
					pParticle2->pParticle = pParticle3;
					pParticle2++;
				}

				pMPSuLM->m_nParticles = pParticle2 - pMPSuLM->m_ParticleArray;
			}

			RVLResetFlags<CRVLPSuLM>(&m_PSuLMSubList, RVLPSULM_FLAG_VISITED);

			// find the best pose

#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
			if(m_ParticleCbVArray)
				delete[] m_ParticleCbVArray;

			int wBestCbV = 0;
			RVLPSULM_PARTICLE *pParticleCbV = m_ParticleCbVArray = new RVLPSULM_PARTICLE[m_nPlausibleHypotheses];
#endif
			int wBest = 0;

			double tx0, ty0, cs0, sn0, ex, ey, eal;
			RVLPSULM_PARTICLE2 *pParticle2ArrayEnd;
			RVLPSULM_PARTICLE *pBestParticle;
#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
			RVLPSULM_PARTICLE *pBestParticleCbV;
#endif

#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
			for(pParticle = ParticleBuff; pParticle < pParticleBuffEnd; pParticle++)
#else
			for(pParticle = m_ParticleArray; pParticle < pParticleArrayEnd; pParticle++)
#endif
			{
				if(pParticle->w == 0)
					continue;

				wacc = 0;

				cs0 = pParticle->PoseSM.m_Rot[0];
				sn0 = pParticle->PoseSM.m_Rot[3];
				tx0 = pParticle->PoseSM.m_X[0];
				ty0 = pParticle->PoseSM.m_X[1];

				pMPSuLM = pParticle->pMPSuLM;

				pParticle2ArrayEnd = pMPSuLM->m_ParticleArray + pMPSuLM->m_nParticles;

				for(pParticle2 = pMPSuLM->m_ParticleArray; pParticle2 < pParticle2ArrayEnd; pParticle2++)
				{
					ex = pParticle2->tx - tx0;
					ey = pParticle2->ty - ty0;

					if(ex * ex + ey * ey > 100.0 * 100.0)
						continue;

					if(pParticle2->cs * cs0 + pParticle2->sn * sn0 < 0)
						continue;

					eal = -pParticle2->cs * sn0 + pParticle2->sn * cs0 > 3.0 * DEG2RAD;

					if(eal > 3.0 * DEG2RAD)
						continue;

					if(eal < -3.0 * DEG2RAD)
						continue;

					wacc += pParticle2->pParticle->w;
				}

				if(wacc > wBest)
				{
					pBestParticle = pParticle;

					wBest = wacc;
				}

#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_VER2
				if(pParticle - ParticleBuff < m_nPlausibleHypotheses)
				{
					if(pParticle->w > 0)
					{
						if((pMPSuLM->m_Flags & RVLPSULM_FLAG_VISITED) == 0)
						{
							pParticle->pHypothesis->Flags |= RVLPSULM_HYPOTHESIS_FLAG_BEST_FOR_ITS_PSULM;

							pMPSuLM->m_Flags |= RVLPSULM_FLAG_VISITED;

							*(pParticleCbV++) = *pParticle;

							if(wacc > wBestCbV)
							{
								pBestParticleCbV = pParticle;

								wBestCbV = wacc;
							}
						}
					}
				}
#endif
			}

			m_nParticlesCbV = pParticleCbV - m_ParticleCbVArray;

			if(m_nParticlesCbV > 0)
			{
				m_BestParticleCbV = *pBestParticleCbV;
				m_BestParticle = *pBestParticle;
			}
			else
			{
				RAspAmp = m_BestParticle.PoseSM.m_Rot;
				tAspAmp = m_BestParticle.PoseSM.m_X;

				RVLCOMPTRANSF3D3DOF(RAspAmp, tAspAmp, RAsAsp, tAsAsp, RAsAmp, tAsAmp)

				RVLCOPYMX3X3(RAsAmp, RAspAmp)
				RVLCOPY3VECTOR(tAsAmp, tAspAmp)
			}

//#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_DEBUG
//			pMPSuLM = m_pBestParticle->pMPSuLM;
//			//pMPSuLM = m_HypothesisArray[0]->pMPSuLM;
//			//pMPSuLM = m_PSuLMArray[0];
//
//			FILE *fpDebugPF = fopen("C:\\RVL\\Debug\\PFCor.dat", "w");
//
//			CRVL3DPose PoseAspAmp2;
//
//			double *RAspAmp2 = PoseAspAmp2.m_Rot;
//			double *tAspAmp2 = PoseAspAmp2.m_X;
//
//			for(ppHypothesis = m_HypothesisArray; ppHypothesis < pHypothesisArrayEnd; ppHypothesis++, pParticle++)
//			{
//				pHypothesis = *ppHypothesis;
//
//				if(pHypothesis->cost < m_PlausibilityThr)
//					break;
//
//				HTadr = pMPSuLM->m_Index * RVLPSULMBUILDER_MAXN_PSULMS + pHypothesis->pMPSuLM->m_Index;
//
//				RVLQLISTHT_GET_ENTRY(HT, RVLQLISTHT_PTR_ENTRY, HTadr, HTSize, pHTEntry, pListTmp)
//
//				if(pHTEntry == NULL)
//					continue;
//
//				pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pHTEntry->Ptr);
//
//				pPoseAmpAmh = &(pNeighbor->PoseRel);
//
//				RAmpAmh = pPoseAmpAmh->m_Rot;
//				tAmpAmh = pPoseAmpAmh->m_X;
//
//				RVLDeterminePose6DOFTo3DOF(&PoseAspAmp2, &(pHypothesis->PoseSM),m_pPoseAC);
//
//				RVLCOMPTRANSF3D3DOF(RAmpAmh, tAmpAmh, RAspAmp2, tAspAmp2, RAspAmh, tAspAmh)
//
//					fprintf(fpDebugPF, "%d\t%d\t%lf\t%lf\t%lf\t%lf\n", pHypothesis->Index, pHypothesis->pMPSuLM->m_Index, 
//						RAspAmh[0], RAspAmh[3], tAspAmh[0], tAspAmh[1]);
//			}
//
//			fclose(fpDebugPF);
//
//			fpDebugPF = fopen("C:\\RVL\\Debug\\PF.dat", "w");
//
//			fprintf(fpDebugPF, "%lf\t%lf\t%lf\t%lf\n", m_pBestParticle->PoseSM.m_Rot[0], m_pBestParticle->PoseSM.m_Rot[3], 
//				m_pBestParticle->PoseSM.m_X[0], m_pBestParticle->PoseSM.m_X[1]);
//
//			pParticle2ArrayEnd = pMPSuLM->m_ParticleArray + pMPSuLM->m_nParticles;
//
//			for(pParticle2 = pMPSuLM->m_ParticleArray; pParticle2 < pParticle2ArrayEnd; pParticle2++)
//				fprintf(fpDebugPF, "%lf\t%lf\t%lf\t%lf\n", pParticle2->cs, pParticle2->sn, pParticle2->tx, pParticle2->ty);
//	
//			fclose(fpDebugPF);
//#endif

			// reset VISITED flag and release particle memory for all PSuLMs

			m_PSuLMSubList.Start();

			while(m_PSuLMSubList.m_pNext)
			{
				pMPSuLM = (CRVLPSuLM *)(m_PSuLMSubList.GetNext());

				pMPSuLM->m_Flags &= ~RVLPSULM_FLAG_VISITED;

				delete[] pMPSuLM->m_ParticleArray;
			}
			
			// resampling

			int dw = wTotal / m_refnParticles;

			int wSample = dw / 2;

			int w = 0;

			if(m_ParticleArray)
				delete[] m_ParticleArray;

			RVLPSULM_PARTICLE *pNewParticle = m_ParticleArray = new RVLPSULM_PARTICLE[m_refnParticles];

			for(pParticle = ParticleBuff; pParticle < pParticleBuffEnd; pParticle++)
			{
				if(pParticle->w > 0)
				{
					w += pParticle->w;

					while(w >= wSample)
					{
						*pNewParticle = *pParticle;

						pNewParticle->Flags = 0x00;

						pNewParticle->w = 0;

						pNewParticle++;

						if(pNewParticle - m_ParticleArray >= m_refnParticles)
							break;

						wSample += dw;
					}
				}

				if(pNewParticle - m_ParticleArray >= m_refnParticles)
					break;
			}

			m_nParticles = pNewParticle - m_ParticleArray;

			delete[] ParticleBuff;
		}	// if(m_nParticles > 0)

		// add new model to the map

		if(m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING)
		{
			double d2Thr = m_MinHybridLocalizationDist - 100.0;

			d2Thr = (d2Thr > 0.0 ? d2Thr * d2Thr : 0.0);

			double csdazThr = cos((m_MinHybridLocalizationAngle - 1.0) * DEG2RAD);

			CRVLPSuLM *pNewPSuLM = NULL; 

			double *RAmp0 = m_PoseAmp0.m_Rot;
			double *tAmp0 = m_PoseAmp0.m_X;
			double *RAs0 = pPoseS0->m_Rot;
			double *tAs0 = pPoseS0->m_X;

			double tx, ty, d2, csdaz;			

			if(m_nParticlesCbV > 0)
			{
				RVLPSULM_PARTICLE *pParticleCbVArrayEnd = m_ParticleCbVArray + m_nParticlesCbV;

				double d2min = -1.0;

				for(pParticle = m_ParticleCbVArray; pParticle < pParticleCbVArrayEnd; pParticle++)
				{
					tx = pParticle->PoseSM.m_X[0];
					ty = pParticle->PoseSM.m_X[1];

					d2 = tx * tx + ty * ty;

					if(d2 > d2Thr)
						continue;

					csdaz = pParticle->PoseSM.m_Rot[0];

					if(csdaz > csdazThr)
						break;
				}

				if(pParticle == pParticleCbVArrayEnd)
				{
					pNewPSuLM = Clone(pSPSuLM);					

					for(pParticle = m_ParticleCbVArray; pParticle < pParticleCbVArrayEnd; pParticle++)
					{
						pParticle->PoseSM.m_C = pParticle->pHypothesis->P3DOF;

						Connect(pParticle->pMPSuLM, pNewPSuLM, &(pParticle->PoseSM));
					}
				}
			}
			else if(m_pPrevModelPSuLM != NULL)
			{
				if(pSPSuLM->m_minPlaneExists == 1)
				{
					CRVL3DPose PoseAsAmp;

					double *RAsAmp = PoseAsAmp.m_Rot;
					double *tAsAmp = PoseAsAmp.m_X;
					double CAsAmp[3 * 3];
					PoseAsAmp.m_C = CAsAmp;
					double tmp3x1[3];

					RVLCOMPTRANSF3DWITHINV(RAmp0, tAmp0, RAs0, tAs0, RAsAmp, tAsAmp, tmp3x1)

					d2 = tAsAmp[0] * tAsAmp[0] + tAsAmp[1] * tAsAmp[1];

					csdaz = RAsAmp[0];

					if(d2 > d2Thr || csdaz < csdazThr)
					{
						pNewPSuLM = Clone(pSPSuLM);

						DetermineOdometryUncertainty(CAsAmp, &PoseAsAmp);

						Connect(m_pPrevModelPSuLM, pNewPSuLM, &PoseAsAmp);
					}
				}
			}
			else
			{
				pNewPSuLM = Clone(pSPSuLM);

				m_nParticles = m_refnParticles;

				if(m_ParticleArray)
					delete[] m_ParticleArray;

				m_ParticleArray = new RVLPSULM_PARTICLE[m_nParticles];

				pParticleArrayEnd = m_ParticleArray + m_nParticles;

				double *R, *t;
				
				for(pParticle = m_ParticleArray; pParticle < pParticleArrayEnd; pParticle++)
				{
					pParticle->pMPSuLM = pNewPSuLM;

					R = pParticle->PoseSM.m_Rot;

					RVLUNITMX3(R)

					t = pParticle->PoseSM.m_X;

					RVLNULL3VECTOR(t)					
				}

				m_Flags &= ~RVLPSULMBUILDER_FLAG_KIDNAPPED;
			}

			if(pNewPSuLM)
			{
				//Add current PSULM to database/map
				m_PSuLMList.Add(pNewPSuLM);
				m_pPrevModelPSuLM = pNewPSuLM;

				//Update local maps
				CreateLocalMap(pNewPSuLM, RVLPSULMBUILDER_CREATELOCALMAP_FLAG_BIDIRECTIONAL);

				//Update m_PoseMp0
				RVLCOPYMX3X3(RAs0, RAmp0)
				RVLCOPY3VECTOR(tAs0, tAmp0)
			}
		}	// if(m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING)

		//Save current scene pose to previous scene pose
		RVLCOPY3VECTOR(pPoseS0->m_X, m_pPoseSp0->m_X)
		RVLCOPYMX3X3(pPoseS0->m_Rot,m_pPoseSp0->m_Rot)
		m_pPoseSp0->m_Alpha = pPoseS0->m_Alpha;
		m_pPoseSp0->m_Beta = pPoseS0->m_Beta;
		m_pPoseSp0->m_Theta = pPoseS0->m_Theta;

		/////

		m_LocalizationTime = m_pTimer->GetTime() - StartTime;	

#ifdef RVLPSULMBUILDER_PARTICLE_FILTER_DEBUG
		pMPSuLM = m_BestParticle.pMPSuLM;
		//pMPSuLM = m_HypothesisArray[0]->pMPSuLM;
		//pMPSuLM = m_PSuLMArray[0];

		if(pMPSuLM)
		{
			int iTmp;

			m_BestParticle.PoseSM.m_C = m_BestParticle.P;

			RVLNULLMX3X3(m_BestParticle.P)

			GetModelPoses(&(m_BestParticle.PoseSM), pMPSuLM, NULL, iTmp);

			CRVL3DPose *pPoseAmAs0;
			double *RAmAs0, *tAmAs0, *CAmAs0;

			FILE *fpDebugPF = fopen("C:\\RVL\\Debug\\PFMap.dat", "w");

			CRVLPSuLM *pMPSuLM2;

			m_PSuLMList.Start();

			while(m_PSuLMList.m_pNext)
			{
				pMPSuLM2 = (CRVLPSuLM *)(m_PSuLMList.GetNext());

				if(pMPSuLM2->m_iSubMap != pMPSuLM->m_iSubMap)
					continue;
				
				pPoseAmAs0 = &(pMPSuLM2->m_PoseRTAs);

				RAmAs0 = pPoseAmAs0->m_Rot;
				tAmAs0 = pPoseAmAs0->m_X;
				CAmAs0 = pPoseAmAs0->m_C;

				fprintf(fpDebugPF, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", pMPSuLM2->m_Index, 
					RAmAs0[0], RAmAs0[3], tAmAs0[0], tAmAs0[1], CAmAs0[4], CAmAs0[5], CAmAs0[8]);
			}

			fclose(fpDebugPF);

			CRVL3DPose PoseAsAm;
			double *RAsAm = PoseAsAm.m_Rot;
			double *tAsAm = PoseAsAm.m_X;
			double CAsAm[3 * 3];
			
			double CCsCm[3 * 3 * 3];

			CRVL3DPose *pPoseAsAm;
			double *RAsAm_, *tAsAm_;

			CRVL3DPose PoseAsAs0;
			double *RAsAs0 = PoseAsAs0.m_Rot;
			double *tAsAs0 = PoseAsAs0.m_X;
			double CAsAs0[3 * 3], MxTmp[2 * 2];

			fpDebugPF = fopen("C:\\RVL\\Debug\\PFCor.dat", "w");

			for(ppHypothesis = m_HypothesisArray; ppHypothesis < pHypothesisArrayEnd; ppHypothesis++, pParticle++)
			{
				pHypothesis = *ppHypothesis;

				if(pHypothesis->cost < m_PlausibilityThr)
					break;

				pPoseAmAs0 = &(pHypothesis->pMPSuLM->m_PoseRTAs);

				RAmAs0 = pPoseAmAs0->m_Rot;
				tAmAs0 = pPoseAmAs0->m_X;

				RVLDeterminePose6DOFTo3DOF(&PoseAsAm, &(pHypothesis->PoseSM) ,m_pPoseAC);

				RVLCOMPTRANSF3D3DOF(RAmAs0, tAmAs0, RAsAm, tAsAm, RAsAs0, tAsAs0)

				PanTiltRollUncertainty(CCsCm, pHypothesis->P, &(pHypothesis->PoseSM),true);
				DetermineUncertainty6DOFTo3DOF(CAsAm, CCsCm, &PoseAsAm);

				UncertaintyEKFPrediction3DOF(CAsAs0, CAsAm, pHypothesis->pMPSuLM->m_PoseRTAs.m_C, &PoseAsAm, pPoseAmAs0);
				
				fprintf(fpDebugPF, "%d\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d\n", pHypothesis->Index, pHypothesis->pMPSuLM->m_Index, 
					RAsAs0[0], RAsAs0[3], tAsAs0[0], tAsAs0[1], CAsAs0[4], CAsAs0[5], CAsAs0[8], pHypothesis->Flags);
			}

			fclose(fpDebugPF);

			fpDebugPF = fopen("C:\\RVL\\Debug\\PF.dat", "w");

			pParticleArrayEnd = m_ParticleArray + m_nParticles;

			for(pParticle = m_ParticleArray; pParticle < pParticleArrayEnd; pParticle++)
			{
				pPoseAmAs0 = &(pParticle->pMPSuLM->m_PoseRTAs);

				RAmAs0 = pPoseAmAs0->m_Rot;
				tAmAs0 = pPoseAmAs0->m_X;

				pPoseAsAm = &(pParticle->PoseSM);

				RAsAm_ = pPoseAsAm->m_Rot;
				tAsAm_ = pPoseAsAm->m_X;

				RVLCOMPTRANSF3D3DOF(RAmAs0, tAmAs0, RAsAm_, tAsAm_, RAsAs0, tAsAs0)

				fprintf(fpDebugPF, "%lf\t%lf\t%lf\t%lf\n", RAsAs0[0], RAsAs0[3], tAsAs0[0], tAsAs0[1]);
			}

			fclose(fpDebugPF);
		}
#endif

#pragma endregion
	}
#ifdef NEVER
	else	// if((m_Flags & RVLPSULMBUILDER_FLAG_PARTICLE_FILTER) == 0 || 
			// (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) != RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
	if(m_nHypotheses > 0 && pBestHypothesis != NULL)
	{
		BYTE *pFreeMem = m_pMem2->m_pFreeMem;

		CRVL3DSurface2 **MatchedMSurfArray;

		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, CRVL3DSurface2 *, pSPSuLM->m_n3DSurfaces, MatchedMSurfArray);

		if((m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_DISPLAY) == 0)
		{
			CreateMatchMatrix(pSPSuLM, pBestHypothesis->pMPSuLM, &(pBestHypothesis->PoseSM), 1.0, MatchedMSurfArray);

			if(m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_POSE_REFINEMENT)
				RVLPSuLMHypothesisPoseRefinement(pBestHypothesis, pSPSuLM, MatchedMSurfArray);
		}

		m_pMem2->m_pFreeMem = pFreeMem;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG
		if(m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_DISPLAY)
		{
			// visualization of sorted hypotheses	

			RVLPSULM_HYPOTHESIS **HypothesisList = new RVLPSULM_HYPOTHESIS *[m_HypothesisList.m_nElements];
				
			RVLPSULM_HYPOTHESIS **ppHypothesis = HypothesisList;

			m_HypothesisList.Start();

			while(m_HypothesisList.m_pNext)
				*(ppHypothesis++) = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

			RVLPSULM_HYPOTHESIS **pHypothesisListEnd = ppHypothesis;

			RVLPSULM_HYPOTHESIS **ppBestHypothesis;
			RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData;

			while(pHypothesisListEnd > HypothesisList)
			{
				// find the best hypothesis in HypothesisList

				minCost = 0;

				for(ppHypothesis = HypothesisList; ppHypothesis < pHypothesisListEnd; ppHypothesis++)
				{	
					pHypothesis = *ppHypothesis;

					if(pHypothesis->cost > minCost)
					{
						minCost = pHypothesis->cost;
						ppBestHypothesis = ppHypothesis;
					}
				}

				pHypothesis = *ppBestHypothesis;

				pHypothesisListEnd--;

				if(pHypothesisListEnd > ppBestHypothesis)
					memmove(ppBestHypothesis, ppBestHypothesis + 1, 
						(pHypothesisListEnd - ppBestHypothesis) * sizeof(RVLPSULM_HYPOTHESIS *));

				// project surfaces of pSPSuLM to the c.s. of pHypothesis->pMPSuLM

				pMPSuLM = pHypothesis->pMPSuLM;

				pPoseSM = &(pHypothesis->PoseSM);

				InverseTransform3D(PoseMS.m_Rot, PoseMS.m_X, pPoseSM->m_Rot, pPoseSM->m_X);

				memcpy(m_CellArray, m_EmptyCellArray, m_nCells * sizeof(RVLPSULM_CELL));

				cost = 0;

				for(i=0;i<pSPSuLM->m_n3DSurfaces;i++)
					cost += pMPSuLM->Match(pSPSuLM->m_3DSurfaceArray[i], &PoseMS);

				pMouseCallbackDataM->pMPSuLM = pMPSuLM;
				//pMouseCallbackDataS->pMPSuLM = pMPSuLM;

				RVLCOPY3VECTOR(NullPose.m_X, pMouseCallbackDataM->PoseM0.m_X)
				RVLCOPYMX3X3(NullPose.m_Rot, pMouseCallbackDataM->PoseM0.m_Rot)

				RVLCOPY3VECTOR(pPoseSM->m_X, pMouseCallbackDataM->PoseS0.m_X)
				RVLCOPYMX3X3(pPoseSM->m_Rot, pMouseCallbackDataM->PoseS0.m_Rot)

#ifndef RVLHIDEDETAILS
				pMouseCallbackDataM->Flags = (RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_MCELLS | RVLPSULM_DISPLAY_PROJ_MCELLS);
#endif
				// create match matrix

				pFreeMem = m_pMem2->m_pFreeMem;

				RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, CRVL3DSurface2 *, pSPSuLM->m_n3DSurfaces, MatchedMSurfArray);

				CreateMatchMatrix(pSPSuLM, pMPSuLM, pPoseSM, 1.0, MatchedMSurfArray);

				// display PSuLMs

				pMouseCallbackData = (RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFigM->m_vpMouseCallbackData);

				pMouseCallbackData->MatrixSceneModel = m_MatchMatrix;	

				RVLPSuLMDisplay(pFigM);

				m_DebugData.pGUI->ShowFigure(pFigM);

				//memcpy(m_CellArray, m_EmptyCellArray, m_nCells * sizeof(RVLPSULM_CELL));

				//for(i=0;i<pMPSuLM->m_n3DSurfaces;i++)
				//	pSPSuLM->Match(pMPSuLM->m_3DSurfaceArray[i], pPoseSM);

				//RVLCOPY3VECTOR(NullPose.m_X, pMouseCallbackDataS->PoseS0.m_X)
				//RVLCOPYMX3X3(NullPose.m_Rot, pMouseCallbackDataS->PoseS0.m_Rot)

				//RVLCOPY3VECTOR(PoseMS.m_X, pMouseCallbackDataS->PoseM0.m_X)
				//RVLCOPYMX3X3(PoseMS.m_Rot, pMouseCallbackDataS->PoseM0.m_Rot)

				//pMouseCallbackDataS->Flags = (RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_SCELLS | RVLPSULM_DISPLAY_PROJ_SCELLS);

				//RVLPSuLMDisplay(pFigS);

				//m_DebugData.pGUI->ShowFigure(pFigS);

				cvSet(pFig2->m_pImage, cvScalar(255, 255, 255));

				sprintf(Text, "hypothesis %d", pHypothesis->Index);

				pFig2->PutText(Text, cvPoint(8, pFig2->m_FontSize), cvScalar(0, 0, 0));

				sprintf(Text, "cost=%d", pHypothesis->cost);

				pFig2->PutText(Text, cvPoint(8, 2 * pFig2->m_FontSize), cvScalar(0, 0, 0));

				if(pPrevSPSuLM == NULL)
				{
					sprintf(Text, "model%d", pHypothesis->pMPSuLM->m_Index);

					pFig2->PutText(Text, cvPoint(8, 3 * pFig2->m_FontSize), cvScalar(0, 0, 0));
				}

				m_DebugData.pGUI->ShowFigure(pFig2);

				// PoseSM refinement

				if(m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_POSE_REFINEMENT)
					RVLPSuLMHypothesisPoseRefinement(pHypothesis, pSPSuLM, MatchedMSurfArray);

				m_pMem2->m_pFreeMem = pFreeMem;

				// if ESC key is pressed, then stop the loop

				int key = cvWaitKey();

				if(key == 27)
					break;
			}
			
			cvDestroyWindow("MPSuLM");
			cvDestroyWindow("Hypothesis");
		}	// if(m_Flags & RVLPSULMBUILDER_FLAG_LOCALIZATION_DISPLAY)
#endif

//#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
//		char *HistFileName = new char[strlen(m_ImageFileName) + 9];
//
//		strcpy(HistFileName, m_ImageFileName);
//
//		strcat(HistFileName, "Hist.dat");
//
//		//FILE *fp = fopen(HistFileName, "w");
//		
//		FILE *fp = fopen("C:\\RVL\\ExpRez\\PSuLMHypotheses.dat", "w");
//
//		int cnt = 0;
//		if(fp)
//		{
//			fprintf(fp,"Hypothesis\tModel\tidx\tCost\tX\tY\tZ\tAlpha\tBeta\tTheta\n");
//
//			RVLPSULM_HYPOTHESIS **HypothesisList = new RVLPSULM_HYPOTHESIS *[m_HypothesisList.m_nElements];
//
//			RVLPSULM_HYPOTHESIS **ppHypothesis = HypothesisList;
//
//			m_HypothesisList.Start();
//
//			while(m_HypothesisList.m_pNext)
//				*(ppHypothesis++) = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());
//
//			RVLPSULM_HYPOTHESIS **pHypothesisListEnd = ppHypothesis;
//			RVLPSULM_HYPOTHESIS **ppBestHypothesis;
//
//			while(pHypothesisListEnd > HypothesisList)
//			{
//				minCost = 0;
//
//				for(ppHypothesis = HypothesisList; ppHypothesis < pHypothesisListEnd; ppHypothesis++)
//				{	
//					pHypothesis = *ppHypothesis;
//
//					if(pHypothesis->cost > minCost)
//					{
//						minCost = pHypothesis->cost;
//						ppBestHypothesis = ppHypothesis;
//					}
//				}
//
//				pHypothesis = *ppBestHypothesis;
//
//				pHypothesisListEnd--;
//
//				if(pHypothesisListEnd > ppBestHypothesis)
//					memmove(ppBestHypothesis, ppBestHypothesis + 1, 
//						(pHypothesisListEnd - ppBestHypothesis) * sizeof(RVLPSULM_HYPOTHESIS *));
//
//				
//				fprintf(fp,"%4d\t%4d\t%4d\t%4d\t%6.3lf\t%6.3lf\t%6.3lf\t%6.2lf\t%6.3lf\t%6.3lf\n",
//					cnt,
//					pHypothesis->pMPSuLM->m_Index,
//					pHypothesis->Index,
//					pHypothesis->cost,
//					pHypothesis->PoseSM.m_X[0],
//					pHypothesis->PoseSM.m_X[1],
//					pHypothesis->PoseSM.m_X[2],
//					pHypothesis->PoseSM.m_Alpha* RAD2DEG, 
//					pHypothesis->PoseSM.m_Beta* RAD2DEG, 
//					pHypothesis->PoseSM.m_Theta* RAD2DEG );
//
//				cnt++;
//
//			}
//
//
//			fclose(fp);
//			delete[] HypothesisList;
//			delete[] HistFileName;
//			
//		}
//#endif

		if(pBestHypothesis->cost >= m_PlausibilityThr)
		{
			//copy particles hypotheses into m_ParticleArray

			if(m_ParticleArray)
				delete[] m_ParticleArray;

			pParticle = m_ParticleArray = new RVLPSULM_PARTICLE[m_nPlausibleHypotheses];

			for(ppHypothesis = m_HypothesisArray; ppHypothesis < pHypothesisArrayEnd; ppHypothesis++, pParticle++)
			{
				pHypothesis = *ppHypothesis;

				if(pHypothesis->cost < m_PlausibilityThr)
					break;

				pParticle->pMPSuLM = pHypothesis->pMPSuLM;
				pParticle->PoseSM.m_C = pParticle->P;
				pParticle->PoseSM.Copy(&(pHypothesis->PoseSM3DOF));		
			}

			m_nParticles = m_nPlausibleHypotheses = pParticle - m_ParticleArray;

			m_LocalizationTime = m_pTimer->GetTime() - StartTime;	

			//m_iLocalModel = pBestHypothesis->pMPSuLM->m_Index;

			//SaveCurrentData();
#ifndef RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
#ifndef RVLPSULMBUILDER_MAPBUILDING_6DOF
			if((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
			{
				SaveCurrentData(pSPSuLM, pBestHypothesis->pMPSuLM, &(pBestHypothesis->PoseSM), pPoseS0, pBestHypothesis->cost);
			}
#endif
#endif

			m_minModelInfo = pBestHypothesis->pMPSuLM->m_minInfo;
			m_minModelPlaneExists = pBestHypothesis->pMPSuLM->m_minPlaneExists;
		}
	}
#endif

	m_LocalizationTime = m_pTimer->GetTime() - StartTimeTotal;
}

int CRVLPSuLMBuilder::EvaluateHypothesis(	CRVLPSuLM * pSPSuLM,
											RVLPSULM_HYPOTHESIS *pHypothesis)
{
	CRVLPSuLM *pMPSuLM = pHypothesis->pMPSuLM;

	int cost = 0;

	pHypothesis->visible = 0;

	CRVL3DLine2 **pp3DLineArrayEnd = pSPSuLM->m_3DLineArray + pSPSuLM->m_n3DLines;

	CRVL3DLine2 **pp3DLine;
	CRVL3DLine2 *p3DLine;
	int visible;

	for(pp3DLine = pSPSuLM->m_3DLineArray; pp3DLine < pp3DLineArrayEnd; pp3DLine++)
	{
		p3DLine = *pp3DLine;

		cost += MatchLine(p3DLine, pMPSuLM, &(pHypothesis->PoseSM), FALSE, visible);

		pHypothesis->visible += visible;
	}

	pp3DLineArrayEnd = pMPSuLM->m_3DLineArray + pMPSuLM->m_n3DLines;
	
	for(pp3DLine = pMPSuLM->m_3DLineArray; pp3DLine < pp3DLineArrayEnd; pp3DLine++)
	{
		p3DLine = *pp3DLine;

		cost += MatchLine(p3DLine, pSPSuLM, &(pHypothesis->PoseSM), TRUE, visible);

		pHypothesis->visible += visible;
	}

	return cost;
}



//Evaluates hypothesis for surfaces  NOTE: m_pMem2 is used
int CRVLPSuLMBuilder::EvaluateHypothesis2(	CRVLPSuLM * pSPSuLM,
											RVLPSULM_HYPOTHESIS *pHypothesis)
{
	double cost = 0;
	
	CRVL3DSurface2 *pM3DSurface, *pS3DSurface; 

	CRVLPSuLM *pMPSuLM = pHypothesis->pMPSuLM;


	//1.
	//Create Scene Cell Array
	int i, j, u, v, cnt, currentPos, iCell, jCell, cellPos;

	RVLQLIST_PTR_ENTRY *pSurf;
	//RVLQLIST *pSurfaceList;
	
	//Reset CellArray2
	memcpy(m_CellArray2, m_EmptyCellArray2, m_nCells2 * sizeof(RVLPSULM_CELL2));
	RVLPSULM_CELL2 *pCell2 = m_CellArray2;

	int uStart, vStart;

	//Fill and Mark CellArray of Scene
	RVLPSULM_CELL2 **CellBuffer = new RVLPSULM_CELL2 *[m_nCells2];
	RVLPSULM_CELL2 **ppCellBuffer = CellBuffer;

	int nActiveSceneCells = 0;

	RVLQLIST *pList;

	//For each cell in cell array
	for(iCell = 0; iCell < m_nCols2; iCell++)
	{
		uStart = iCell*m_CellSize2;
		
		for(jCell = 0; jCell < m_nRows2; jCell++, pCell2++)
		{
			
			vStart = jCell*m_CellSize2;

			pList = &(pCell2->surfaceList);
			
			//Initialize QList
			RVLQLIST_INIT(pList);
			
			//for each pixel within cell
			for(u = uStart; u < uStart + m_CellSize2; u++)
			{
				for(v = vStart; v < vStart + m_CellSize2; v++)
				{
					//Get Scene surface
					pS3DSurface = m_pPSD->m_3DSurfaceMap[v*m_pPSD->m_Width + u];
					
					//If surface exists
					if(pS3DSurface)
					{
						//Increase counter (number of pts)
						pCell2->nPts++;

						//Check if surface has not been marked
						if((pS3DSurface->m_Flags & RVL3DSURFACE_FLAG_MARKED)==0)
						{
							//mark surface
							pS3DSurface->m_Flags |= RVL3DSURFACE_FLAG_MARKED;

							//Add to QList
							RVLMEM_ALLOC_STRUCT(m_pMem2, RVLQLIST_PTR_ENTRY, pSurf);
							pSurf->Ptr = pS3DSurface;
							RVLQLIST_ADD_ENTRY(pList, pSurf);
						}
					}
				}

			}

			
			//Add cell to cellArrayBuffer (and mark cell) if number of pts exceed a min user assigned value
			if(pCell2->nPts >= m_minCellPts)
			{
				pCell2->Flags |= RVLPSULM_CELL_FLAG_ACTIVE;
				*ppCellBuffer = pCell2;
				nActiveSceneCells++;
				ppCellBuffer++;
			}
			

			//Go through all surfaces in Cell QList and reset flags
			pSurf = (RVLQLIST_PTR_ENTRY *)(pCell2->surfaceList.pFirst);
			while(pSurf)
			{
				pS3DSurface = (CRVL3DSurface2 *)pSurf->Ptr;
				
				pS3DSurface->m_Flags &= ~RVL3DSURFACE_FLAG_MARKED;

				pSurf = (RVLQLIST_PTR_ENTRY *)(pSurf->pNext);
			}
			
		}
	}




	//2.
	//Project Model onto Scene and evaluate
	//RVLPSULM_CELL2 cell2;

	double *RSM = pHypothesis->PoseSM.m_Rot;
	double *tSM = pHypothesis->PoseSM.m_X;


	RVLSURFACE_MATCH_ARRAY MatchData;

	
	double *XM, XS[3], tmp3x1[3];
	double P[3 * 3], A[3 * 3];
	double MatchQuality;
	double detQ;
	
	RVL3DSURFACE_SAMPLE *pSample;
	
	
	//Get Projection Matrix
	m_pStereoVision->GetKinectProjectionMatrix(P);

	//A = P*RSM'
	RVLMXMUL3X3T2(P, RSM, A);

	//int TotalNoOfSMPlanes = pSPSuLM->m_n3DSurfacesTotal * pMPSuLM->m_n3DSurfacesTotal;
	//double *MatrixSMCost = new double[TotalNoOfSMPlanes];
	//int *MatrixSMCounter = new int[TotalNoOfSMPlanes];
	//memset(MatrixSMCost, 0.0, TotalNoOfSMPlanes * sizeof(double));
	//memset(MatrixSMCounter, 0, TotalNoOfSMPlanes * sizeof(int));

	int ImageSize = m_pCamera->Width * m_pCamera->Height;

	memset(m_MatrixSMCost, 0.0, ImageSize * sizeof(double));
	memset(m_MatrixSMCounter, 0, ImageSize * sizeof(int));


	double logPDFDifferentSurface = -log(m_PDFDifferentSurface);
	double logCost2 = -log(m_Cost2);
	double currentCost;

	//double dbg0 = 0.0;
	//double dbg1 = 0.0;

	//for each 3D surface
	for(i=0;i<pMPSuLM->m_n3DSurfacesTotal;i++)
	{
		pM3DSurface = pMPSuLM->m_3DSurfaceArray[i];

		pSample = (RVL3DSURFACE_SAMPLE *)(pM3DSurface->m_Samples.pFirst);

		while(pSample)
		{
			XM = pSample->X;

			RVLDIF3VECTORS(XM, tSM, tmp3x1)
			RVLMULMX3X3VECT(A, tmp3x1, XS)

			u = DOUBLE2INT(XS[0] / XS[2]);
			v = DOUBLE2INT(XS[1] / XS[2]);

			if(u >= 0 && u < m_pPSD->m_Width  && v >= 0 && v < m_pPSD->m_Height)
			{
				//Get Cell position and check if it is an active cell
				iCell = u / m_CellSize2;
				jCell = v / m_CellSize2;
				cellPos = jCell * m_nCols2 + iCell;

				pCell2 = m_CellArray2 + cellPos;

				if(pCell2->Flags & RVLPSULM_CELL_FLAG_ACTIVE)
				{

					//Get Scene surface
					pS3DSurface = m_pPSD->m_3DSurfaceMap[v*m_pPSD->m_Width + u];

					if(pS3DSurface)
					{
						//Check counter to make sure it has not been checked
						currentPos = pS3DSurface->m_Index * pMPSuLM->m_n3DSurfacesTotal + pM3DSurface->m_Index;
						
						cnt = m_MatrixSMCounter[currentPos];

						//Get current cost
						if(cnt==0)
						{
							//Calculate cost
							if(pS3DSurface->Match2(pM3DSurface, &(pHypothesis->PoseSM), MatchQuality, detQ, &MatchData, RVL3DSURFACE_FLAG_HYPOTHESIS_EVALUATION))
							{
								MatchQuality = exp(-MatchQuality/2) / sqrt(8 * PI * PI * PI * detQ);
								currentCost = -log(MatchQuality * m_ProbabilitySameSurface + m_Cost2);
								//dbg0 += MatchQuality;
							}
							else
								currentCost = logCost2;
							
							//Store new cost
							m_MatrixSMCost[currentPos]  = currentCost;
						}
						else
						{
							//Get current cost from Matrix
							currentCost = m_MatrixSMCost[currentPos];
						}

						//Increase counter
						m_MatrixSMCounter[currentPos] = cnt + 1;

						//dbg1 += currentCost;

						//Replace cell cost if neccessary
						if (currentCost < pCell2->matchQuality)
							pCell2->matchQuality = currentCost;
					}

				}
				

			}

			pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		}
	}


	ppCellBuffer = CellBuffer;
	for(i = 0; i < nActiveSceneCells; i++, ppCellBuffer++)
	{
		pCell2 = *ppCellBuffer;
		currentCost = pCell2->matchQuality;

		if(currentCost == 1000.0)
			currentCost = logPDFDifferentSurface;
		

		cost += currentCost;
	}

	m_pMem2->Clear();

	//delete[] MatrixSMCost;
	//delete[] MatrixSMCounter;
	delete[] CellBuffer;
	

	return (int)cost;
}

//Hypothesis evaluation by surface sample matching (SSM)
//NOTE: m_pMem2 is used
int CRVLPSuLMBuilder::EvaluateHypothesis3(
	CRVLPSuLM * pSPSuLM,
	RVLPSULM_HYPOTHESIS *pHypothesis,
	bool bDynamicSurfaceDetection)
{
	if (!(pSPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX))
		return EvaluateHypothesis3Simple(pSPSuLM, pHypothesis, bDynamicSurfaceDetection);

	double StartTime = m_pTimer->GetTime();

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
	fprintf(m_fpDebug, "Hypothesis%d\n", pHypothesis->Index);
#endif

	//if (pHypothesis->Index == 7)
	//	int debug = 0;

	double CellSize = (double)m_CellSize2;

	int minwS = m_pPSD->m_minSampleSize;
	int maxwS = minwS + m_pPSD->m_maxSampleSize;
	int maxiCell = m_nCols2 - 1;
	int maxjCell = m_nRows2 - 1;

	CRVLPSuLM *pMPSuLM = pHypothesis->pMPSuLM;

	double *RSM = pHypothesis->PoseSM.m_Rot;
	double *tSM = pHypothesis->PoseSM.m_X;

	int nSSurfaces = pSPSuLM->m_n3DSurfacesTotal;
	int nMSurfaces = pMPSuLM->m_n3DSurfacesTotal;

	double fu = m_pStereoVision->m_KinectParams.depthFu;
	double fv = m_pStereoVision->m_KinectParams.depthFv;
	double f = RVLMAX(fu, fv);
	double uc = m_pStereoVision->m_KinectParams.depthUc;
	double vc = m_pStereoVision->m_KinectParams.depthVc;

	double CellSizeNrm;
	int nCubeSideCells;

	if (pSPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX)
	{
		CellSizeNrm = 1.0 / (double)m_nCellsPer45deg;
		nCubeSideCells = m_nRows2 * m_nCols2;
	}


	// Reset flag RVL3DSURFACE_FLAG_REMOVED of all surfaces

	int i;
	CRVL3DSurface2 *pM3DSurface, *pS3DSurface;
	int iCell, jCell;
	RVLPSULM_CELL2 *pCell;
	RVLQLIST_PTR_ENTRY *pSamplePtr;
	RVL3DSURFACE_SAMPLE *pSampleS;
	double *XS;


	for (i = 0; i < nSSurfaces; i++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

		pS3DSurface->m_Flags &= ~RVL3DSURFACE_FLAG_REMOVED;

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

		while (pSampleS)
		{
			pSampleS->Flags = 0x00;

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}
	}

	// match model samples to scene samples

	int nMatches;

	if (!bDynamicSurfaceDetection)
	{
		nMatches = nSSurfaces * nMSurfaces;

		if (m_MatchMatrix)
			delete[] m_MatchMatrix;

		m_MatchMatrix = new BYTE[nMatches];

		memset(m_MatchMatrix, 0, nMatches);
	}

	//int *SCoverage = new int[pSPSuLM->m_nSurfaceSamples];

	//memset(SCoverage, 0, pSPSuLM->m_nSurfaceSamples * sizeof(int));

	//int *MCoverage = new int[pMPSuLM->m_nSurfaceSamples];

	//memset(MCoverage, 0, pMPSuLM->m_nSurfaceSamples * sizeof(int));

	int uM, vM;
	//int uS, vS;
	double fuM, fvM;
	double *XM, *VM, *VS, XMS[3], dPMS[3], tmp3x1[3];
	//int w;
	double wM, wS;
	//int firstiCell, lastiCell, firstjCell, lastjCell;
	//int euvTol;
	double e, d;
	RVL3DSURFACE_SAMPLE *pSampleM;
	double ddS, z, rS, rM, s, XYTol;
	int iMatch;
	//int wTol;
	double *NS, *NM;
	double PM1[3], PM2[3], dPM[3], NMS[3];
	double stdXM, sinRNS;
	double U[3], RS[3], RM[3];
	BOOL bMOcluded, bSOcluded, bMSMatch, bNM;
	double pM, qM, pS, qS, csNTol, fTmp;
	double p, q;
	int ip, iq, ir;
	//RVLRECT SWindow, MWindow;
	//int CoverageArea;
	double dP[3];
	double eP2, ePThr;

	for (i = 0; i<nMSurfaces; i++)
	{
		pM3DSurface = pMPSuLM->m_3DSurfaceArray[i];

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
		fprintf(m_fpDebug, "M%d\n", pM3DSurface->m_Index);
#endif

		pM3DSurface->m_Flags &= ~(RVL3DSURFACE_FLAG_REMOVED | RVLOBJ2_FLAG_MATCHED | RVLOBJ2_FLAG_MARKED);

		NM = pM3DSurface->m_N;

		RVLMULMX3X3TVECT(RSM, NM, NMS);

		pSampleM = (RVL3DSURFACE_SAMPLE *)(pM3DSurface->m_Samples.pFirst);

		while (pSampleM)
		{
			//if (pSampleM->Index == 774)
			//	int debug = 0;

			pSampleM->Flags = 0x00;

			XM = pSampleM->X;

			RVLDIF3VECTORS(XM, tSM, tmp3x1);
			RVLMULMX3X3TVECT(RSM, tmp3x1, XMS);

			if (pSPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX)
			{
				ProjectToCube(XMS, p, q, ip, iq, ir);

				if (ir == 5)
				{
					pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);

					continue;
				}

				iCell = (int)((p + 1.0) / CellSizeNrm);
				jCell = (int)((q + 1.0) / CellSizeNrm);

				pCell = m_CellArray2 + ir * nCubeSideCells + jCell * m_nCols2 + iCell;

			}
			else
			{
				z = XMS[2];

				fuM = fu * XMS[0] / z + uc;

				uM = DOUBLE2INT(fuM);

				iCell = uM / m_CellSize;

				if (iCell < 0 || iCell > maxiCell)
				{
					pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);

					continue;
				}

				fvM = fv * XMS[1] / z + vc;

				vM = DOUBLE2INT(fvM);
				
				jCell = vM / m_CellSize;

				if (jCell < 0 || jCell > maxjCell)
				{
					pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);

					continue;
				}

				pCell = m_CellArray2 + jCell * m_nCols2 + iCell;
			}

			VM = pSampleM->V;

			stdXM = (pSampleM->stdX + m_SampleMatchDistTol) * m_SampleMatchUncertCoef;

			pM = pSampleM->wz / f / (m_SampleMatchUncertCoef * pSampleM->stdX) - pSampleM->stdN[0];
			qM = pSampleM->stdN[1];

			fTmp = sqrt(pM * pM + qM * qM);

			pM /= fTmp;
			qM /= fTmp;

			bNM = (pM >= qM);

			RVLSCALE3VECTOR(VM, stdXM, dPM);
			RVLMULMX3X3TVECT(RSM, dPM, dPMS);


			RVLSUM3VECTORS(XMS, dPMS, PM1);
			RVLDIF3VECTORS(XMS, dPMS, PM2);

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
			int uMM = DOUBLE2INT(fu * XM[0] / XM[2] + uc);
			int vMM = DOUBLE2INT(fv * XM[1] / XM[2] + vc);

			fprintf(m_fpDebug, "UM=(%d, %d); UMS=(%d, %d); stdXM=%5.0lf; PM1=(%6.0lf, %6.0lf, %6.0lf); PM2=(%6.0lf, %6.0lf, %6.0lf)",
				uMM, vMM, uM, vM, stdXM, PM1[0], PM1[1], PM1[2], PM2[0], PM2[1], PM2[2]);

			if (bNM)
				fprintf(m_fpDebug, "; MN\n");
			else
				fprintf(m_fpDebug, "\n");

			//FILE *fpSample = fopen("C:\\RVL\\Debug\\Sample.dat", "r");

			//int uSample, vSample;

			//fscanf(fpSample, "%d\t%d\n", &uSample, &vSample);

			//fclose(fpSample);

			//if(uMM == uSample && vMM == vSample)
			//	int debug = 0;
#endif

			rM = sqrt(RVLDOTPRODUCT3(XMS, XMS));

			RVLSCALE3VECTOR2(XMS, rM, RM);

			XYTol = 2.0 * (m_SampleMatchAngleTol * rM + m_SampleMatchDistTol);

			wM = (pSampleM->wz / f + XYTol) / rM;

			//wTol = (int)(f*XYTol / z) + 1;

			//wM = (int)(pSampleM->wz / z) + 1;

			//MWindow.left = uM - wM;
			//MWindow.right = uM + wM;
			//MWindow.top = vM - wM;
			//MWindow.bottom = vM + wM;

			//w = RVLMAX(wM - minwS, maxwS - wM) + wTol;
			//w = RVLMIN(wM, maxwS) + wTol;

			//firstiCell = RVLMAX((uM - w) / m_CellSize2, 0);
			//lastiCell = RVLMIN((uM + w) / m_CellSize2, maxiCell);
			//firstjCell = RVLMAX((vM - w) / m_CellSize2, 0);
			//lastjCell = RVLMIN((vM + w) / m_CellSize2, maxjCell);

			//for (jCell = firstjCell; jCell <= lastjCell; jCell++)
			//	for (iCell = firstiCell; iCell <= lastiCell; iCell++)
			//	{

				

				pSamplePtr = (RVLQLIST_PTR_ENTRY *)(pCell->surfaceList.pFirst);

				while (pSamplePtr)	// for all SSamples in the cell
				{
					pSampleS = (RVL3DSURFACE_SAMPLE *)(pSamplePtr->Ptr);

					//if (pSampleS->Index == 448)
					//	int debug = 0;

					XS = pSampleS->X;

					rS = sqrt(RVLDOTPRODUCT3(XS, XS));

					RVLSCALE3VECTOR2(XS, rS, RS);

					wS = pSampleS->wz / f / rS;

					RVLDIF3VECTORS(RS, RM, dP);

					eP2 = RVLDOTPRODUCT3(dP, dP);

					ePThr = wM + wS;

					//uS = DOUBLE2INT(fu * XS[0] / XS[2] + uc);
					//vS = DOUBLE2INT(fv * XS[1] / XS[2] + vc);

					//wS = (int)(pSampleS->wz / XS[2]) + 1;

					//euvTol = abs(wM - wS) + wTol;
					//euvTol = RVLMIN(wM, wS) + wTol;

					if (eP2 <= ePThr * ePThr)
					//if (abs(uM - uS) <= euvTol && abs(vM - vS) <= euvTol)
					{
						//SWindow.left = uS - wS;
						//SWindow.right = uS + wS;
						//SWindow.top = vS - wS;
						//SWindow.bottom = vS + wS;

						//CoverageArea = (RVLMIN(SWindow.right, MWindow.right) - RVLMAX(SWindow.left, MWindow.left)) *
						//	(RVLMIN(SWindow.bottom, MWindow.bottom) - RVLMAX(SWindow.top, MWindow.top));

						//if (CoverageArea > 0)
						//{
						//	SCoverage[pSampleS->Index] += (25 * CoverageArea / (wS * wS));
						//	MCoverage[pSampleM->Index] += (25 * CoverageArea / (wM * wM));
						//}

						pS3DSurface = pSampleS->pSurface;

						NS = pS3DSurface->m_N;

						VS = pSampleS->V;

						ddS = m_SampleMatchUncertCoef * pSampleS->stdX * RVLDOTPRODUCT3(VS, NS);

						bMOcluded = bSOcluded = FALSE;

						d = pS3DSurface->m_d - ddS;

						e = RVLDOTPRODUCT3(NS, PM1) - d;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
						int uS = DOUBLE2INT(fu * XS[0] / XS[2] + uc);
						int vS = DOUBLE2INT(fv * XS[1] / XS[2] + vc);

						fprintf(m_fpDebug, "\tUS=(%d, %d); S%d; e1=%6.0lf", uS, vS, pS3DSurface->m_Index, e);
#endif

						if (e < 0)
						{
							RVLCROSSPRODUCT3(RM, NS, tmp3x1);

							sinRNS = sqrt(RVLDOTPRODUCT3(tmp3x1, tmp3x1));

							if (sinRNS < -0.002 || sinRNS > 0.002)
							{
								RVLSCALE3VECTOR2(tmp3x1, sinRNS, tmp3x1);

								RVLCROSSPRODUCT3(RM, tmp3x1, U);

								s = e / RVLDOTPRODUCT3(NS, U);

								if (s < -XYTol || s > XYTol)
									bSOcluded = TRUE;
							}
							else
								bSOcluded = TRUE;
						}
						else
						{
							d = pS3DSurface->m_d + ddS;

							e = RVLDOTPRODUCT3(NS, PM2) - d;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
							fprintf(m_fpDebug, "; e2=%6.0lf", e);
#endif

							if (e > 0)
							{
								RVLCROSSPRODUCT3(RM, NS, tmp3x1);

								sinRNS = sqrt(RVLDOTPRODUCT3(tmp3x1, tmp3x1));

								if (sinRNS < -0.002 || sinRNS > 0.002)
								{
									RVLSCALE3VECTOR2(tmp3x1, sinRNS, tmp3x1);

									RVLCROSSPRODUCT3(RM, tmp3x1, U);

									s = (d - RVLDOTPRODUCT3(NS, PM2)) / RVLDOTPRODUCT3(NS, U);

									if (s < -XYTol || s > XYTol)
										bMOcluded = TRUE;
								}
								else
									bMOcluded = TRUE;
							}
						}	// if(e >= 0)

						if (bSOcluded)
						{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
							fprintf(m_fpDebug, "; Oclusion-MS\n");
#endif

							if (pSampleS->Flags < RVL3DSURFACE_SAMPLE_FLAG_OCLUDED)
								pSampleS->Flags = RVL3DSURFACE_SAMPLE_FLAG_OCLUDED;

							if (pSampleM->Flags < RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
								pSampleM->Flags = RVL3DSURFACE_SAMPLE_FLAG_REMOVED;
						}
						else if (bMOcluded)
						{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
							fprintf(m_fpDebug, "; Oclusion-SM\n");
#endif

							if (pSampleM->Flags < RVL3DSURFACE_SAMPLE_FLAG_OCLUDED)
								pSampleM->Flags = RVL3DSURFACE_SAMPLE_FLAG_OCLUDED;

							if (pSampleS->Flags < RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
								pSampleS->Flags = RVL3DSURFACE_SAMPLE_FLAG_REMOVED;
						}
						else
						{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
							fprintf(m_fpDebug, "; MS-depth match");
#endif
							bMSMatch = TRUE;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_ORIENT_MATCH
							if (bNM)
							{
								pS = pSampleS->wz / f / (m_SampleMatchUncertCoef * pSampleS->stdX) - pSampleS->stdN[0];
								qS = pSampleS->stdN[1];

								fTmp = sqrt(pS * pS + qS * qS);

								pS /= fTmp;
								qS /= fTmp;

								if (pS >= qS)
								{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
									fprintf(m_fpDebug, "; NS");
#endif
									csNTol = pM * pS - qM * qS;

									if (csNTol >= COS45)
									{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
										fprintf(m_fpDebug, "; dNMS");
#endif
										if (RVLDOTPRODUCT3(NMS, NS) < csNTol)
											bMSMatch = FALSE;
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
										else
											fprintf(m_fpDebug, "; normal match!");
#endif
									}
								}
							}	// if(bNM)
#endif

							if (bMSMatch)
							{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
								fprintf(m_fpDebug, "; match\n");
#endif

								if (pSampleS->Flags != RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
								{
									pSampleS->Flags = RVL3DSURFACE_SAMPLE_FLAG_MATCHED;

									iMatch = pS3DSurface->m_Index * nMSurfaces + pM3DSurface->m_Index;

									if (!bDynamicSurfaceDetection)
										m_MatchMatrix[iMatch]++;
								}

								pSampleM->Flags = RVL3DSURFACE_SAMPLE_FLAG_MATCHED;
							}
							else
							{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
								fprintf(m_fpDebug, "; normals do not match\n");
#endif
								//if(pSampleM->Flags < RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
								//	pSampleM->Flags = RVL3DSURFACE_SAMPLE_FLAG_REMOVED;

								//if(pSampleS->Flags < RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
								//	pSampleS->Flags = RVL3DSURFACE_SAMPLE_FLAG_REMOVED;
							}
						}	// if(!bSOcluded && !bMOcluded)
					}	// if(abs(uM - uS) <= euvTol && abs(vM - vS) <= euvTol)

					pSamplePtr = (RVLQLIST_PTR_ENTRY *)(pSamplePtr->pNext);
				}	// for all SSamples in the cell
				//}	// for(iCell = firstiCell; iCell <= lastiCell; iCell++)

				//if (pSampleM->Flags != RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
				//	int debug = 0;

			pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);
		}	// while(pSampleM)
	}	// for every model surface

	// classify surfaces 

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT
#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
	int nMatchedSamples;
	int nRemovedSamples;
#endif
	double dsrTime = m_pTimer->GetTime();

	for (i = 0; i<nMSurfaces; i++)
	{
		pM3DSurface = pMPSuLM->m_3DSurfaceArray[i];

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
		nMatchedSamples = nRemovedSamples = 0;
#endif

		pSampleM = (RVL3DSURFACE_SAMPLE *)(pM3DSurface->m_Samples.pFirst);

		while (pSampleM)
		{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
			if (pSampleM->Flags == RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
			{
				if (MCoverage[pSampleM->Index] > m_minSurfaceSampleCoverage)
				{
					pM3DSurface->m_Flags |= RVL3DSURFACE_FLAG_REMOVED;

					break;
				}
				else
					pSampleM->Flags = 0x00;
			}
#else
			if (pSampleM->Flags == RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
				nMatchedSamples++;
			if (pSampleM->Flags == RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
				nRemovedSamples++;
#endif

			pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);
		}

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
		if (nRemovedSamples > 0)
			if (100 * nRemovedSamples / (nMatchedSamples + nRemovedSamples) > 20)
				pM3DSurface->m_Flags |= RVL3DSURFACE_FLAG_REMOVED;
#endif
	}

	for (i = 0; i<nSSurfaces; i++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
		nMatchedSamples = nRemovedSamples = 0;
#endif

		while (pSampleS)
		{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
			if (pSampleS->Flags == RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
			{
				if (SCoverage[pSampleS->Index] > m_minSurfaceSampleCoverage)
				{
					pS3DSurface->m_Flags |= RVL3DSURFACE_FLAG_REMOVED;

					break;
				}
				else
					pSampleS->Flags = 0x00;
			}
#else
			if (pSampleS->Flags == RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
				nMatchedSamples++;
			if (pSampleS->Flags == RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
				nRemovedSamples++;
#endif

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
		if (nRemovedSamples > 0)
			if (100 * nRemovedSamples / (nMatchedSamples + nRemovedSamples) > 20)
				pS3DSurface->m_Flags |= RVL3DSURFACE_FLAG_REMOVED;
#endif

	}

	m_DynamicSurfRejectTime = m_pTimer->GetTime() - dsrTime;
#endif	// RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT

	if (bDynamicSurfaceDetection)
	{
		m_HypEvalTime = m_pTimer->GetTime() - StartTime;

		return 0;
	}

	// Surface matching

	int nMatchedSurfaces = 0;

	CRVLMPtrChain MatchList(m_pMem2);

	RVLPSULM_MATCH *MatchBuff = new RVLPSULM_MATCH[nMatches];
	RVLPSULM_MATCH **SortedMatchArray = new RVLPSULM_MATCH *[nMatches];

	RVLPSULM_MATCH *pMatch2 = MatchBuff;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_MATCH_COUNT
	int nMatches2 = 0;
#endif

	int j;
	BYTE *pMatch;

	for (i = 0; i<nSSurfaces; i++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

		pS3DSurface->m_Flags &= ~(RVLOBJ2_FLAG_MATCHED | RVLOBJ2_FLAG_MARKED);

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

		while (pSampleS)
		{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_MATCH_COUNT
			if (pSampleS->Flags == RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
				nMatches2++;
#endif

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}

		if ((pS3DSurface->m_Flags & RVL3DSURFACE_FLAG_REMOVED) == 0)
		{
			//nMatches2 = 0;

			pMatch = m_MatchMatrix + i * nMSurfaces;

			for (j = 0; j<nMSurfaces; j++, pMatch++)
			{
				if (*pMatch >= m_minSurfaceSamplesForMatch)
				{
					pM3DSurface = pMPSuLM->m_3DSurfaceArray[j];

					if ((pM3DSurface->m_Flags & RVL3DSURFACE_FLAG_REMOVED) == 0)
					{
						pMatch2->vpSObject = pS3DSurface;
						pMatch2->vpMObject = pM3DSurface;
						pMatch2->cost = (*pMatch);

						MatchList.Add(pMatch2);

						pMatch2++;

						//nMatches2++;

						//if(nMatches2 == 3)
						//{
						//	nMatchedSurfaces++;

						//	break;
						//}
					}
				}
			}
		}
	}

	// reject the hypothesis if the surfaces from which it is generated are removed

	BOOL bSeedTest = TRUE;

	//CRVL3DSurface2 **ppS3DSurface = (CRVL3DSurface2 **)(pHypothesis->MatchArray);
	//CRVL3DSurface2 **pS3DSufraceArrayEnd = ppS3DSurface + pHypothesis->nMatches;
	//CRVL3DSurface2 **ppM3DSurface = pS3DSufraceArrayEnd;

	//for(; ppS3DSurface < pS3DSufraceArrayEnd; ppS3DSurface++, ppM3DSurface++)
	//{
	//	if((*ppS3DSurface)->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
	//	{
	//		bSeedTest = FALSE;

	//		break;
	//	}

	//	if((*ppM3DSurface)->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
	//	{
	//		bSeedTest = FALSE;

	//		break;
	//	}
	//}

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
	FILE *fpEvaluationMatches = fopen("C:\\RVL\\ExpRez\\HypEval3Matches.dat", "w");
#endif

	// final plausibility estimation

	double scoreTotal = 0.0;

	if (bSeedTest)
	{
		RVLBubbleSort<RVLPSULM_MATCH>(&MatchList, SortedMatchArray, TRUE);

		double StartTime = m_pTimer->GetTime();

		double *RAB = pHypothesis->PoseSM.m_Rot;

		double log2pi = log(2.0 * PI);

		double *RSA, *RMB;
		double CSM[2 * 2], CMM[2 * 2];
		double sMean[2], eSM[2];
		double cSS11, cSS22;
		double detCSM, detCMM, detCMMCSM;
		double sSM[2], JSM[2 * 2];
		double score;
		double M2x2Tmp[2 * 2];

		for (i = 0; i < MatchList.m_nElements; i++)
		{
			pMatch2 = SortedMatchArray[i];

			pS3DSurface = (CRVL3DSurface2 *)(pMatch2->vpSObject);
			pM3DSurface = (CRVL3DSurface2 *)(pMatch2->vpMObject);

			if (((pS3DSurface->m_Flags | pM3DSurface->m_Flags) & RVLOBJ2_FLAG_MATCHED) == 0)
			{
				pS3DSurface->m_Flags |= (RVLOBJ2_FLAG_MATCHED | RVLOBJ2_FLAG_MARKED);
				pM3DSurface->m_Flags |= (RVLOBJ2_FLAG_MATCHED | RVLOBJ2_FLAG_MARKED);
				nMatchedSurfaces++;

				RSA = pS3DSurface->m_Pose.m_Rot;
				RMB = pM3DSurface->m_Pose.m_Rot;

				// RSB = RAB * RSA

				double RSB[3 * 3];

				RVLMXMUL3X3(RAB, RSA, RSB)

					// sSM = [xMB' yMB'] * zSB;

					sSM[0] = RVLMULCOLCOL3(RMB, RSB, 0, 2);
				sSM[1] = RVLMULCOLCOL3(RMB, RSB, 1, 2);

				// JSM = [xMB' yMB'] * [xSB ySB];			

				RVLMXEL(JSM, 2, 0, 0) = RVLMULCOLCOL3(RMB, RSB, 0, 0);
				RVLMXEL(JSM, 2, 0, 1) = RVLMULCOLCOL3(RMB, RSB, 0, 1);
				RVLMXEL(JSM, 2, 1, 0) = RVLMULCOLCOL3(RMB, RSB, 1, 0);
				RVLMXEL(JSM, 2, 1, 1) = RVLMULCOLCOL3(RMB, RSB, 1, 1);

				// CMM = diag(pM3DSurface->m_varq[0:1])

				RVLMXEL(CMM, 2, 0, 0) = pM3DSurface->m_varq[0];
				RVLMXEL(CMM, 2, 0, 1) = 0.0;
				RVLMXEL(CMM, 2, 1, 1) = pM3DSurface->m_varq[1];

				// CSM = JSM * diag(pS3DSurface->m_varq[0:1]) * JSM'

				cSS11 = pS3DSurface->m_varq[0];
				cSS22 = pS3DSurface->m_varq[1];

				RVLMXEL(CSM, 2, 0, 0) = cSS11*JSM[0] * JSM[0] + cSS22*JSM[2] * JSM[2];
				RVLMXEL(CSM, 2, 0, 1) = cSS11*JSM[0] * JSM[1] + cSS22*JSM[2] * JSM[3];
				RVLMXEL(CSM, 2, 1, 1) = cSS11*JSM[1] * JSM[1] + cSS22*JSM[3] * JSM[3];

				// detCMMCSM = det(CMM + CSM)

				detCMMCSM = (CMM[0] + CSM[0]) * (CMM[3] + CSM[3]) - CSM[1] * CSM[1];

				// sMean = inv(inv(CMM) + inv(CSM))*inv(CSM)*sSM

				detCMM = CMM[0] * CMM[3];
				detCSM = CSM[0] * CSM[3] - CSM[1] * CSM[1];

				M2x2Tmp[0] = CMM[0] * (CMM[3] + CSM[3]);
				M2x2Tmp[1] = -CMM[0] * CSM[1];
				M2x2Tmp[3] = CMM[3] * (CMM[0] + CSM[0]);

				sMean[0] = (M2x2Tmp[0] * sSM[0] + M2x2Tmp[1] * sSM[1]) / detCMMCSM;
				sMean[1] = (M2x2Tmp[1] * sSM[0] + M2x2Tmp[3] * sSM[1]) / detCMMCSM;

				// score = 2 * pi / sqrt(det(CMM + CSM))*exp(-0.5*((sMean'*inv(CMM)*sMean + (sSM - sMean)'*inv(CSM)*(sSM - sMean))))

				eSM[0] = sSM[0] - sMean[0];
				eSM[1] = sSM[1] - sMean[1];

				score = log2pi - 0.5 * (log(detCMMCSM) + RVLMAHDIST2(sMean, CMM, detCMM) + RVLMAHDIST2(eSM, CSM, detCSM));

				if (score > 0.0)
					scoreTotal += score;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
				fprintf(fpEvaluationMatches, "S%d-M%d\tnSamp:%d\tscore:%lf\n", pS3DSurface->m_Index, pM3DSurface->m_Index, pMatch2->cost, score);
#endif
			}
		}

		double ExecutionTime = m_pTimer->GetTime() - StartTime;

	}	// if(bSeedTest)
	else
	{
		scoreTotal = 0.0;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
		fprintf(fpEvaluationMatches, "Seed test failed!\n");
#endif
	}

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
	fclose(fpEvaluationMatches);
#endif

	// dealocate memory

	m_pMem2->Clear();

	delete[] SortedMatchArray;
	delete[] MatchBuff;	
	//delete[] SCoverage;
	//delete[] MCoverage;

	/////

	//return nMatchedSurfaces;
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_MATCH_COUNT
	return nMatches2;
#else
	return DOUBLE2INT(1000.0 * scoreTotal);
#endif
}

//EvaluateHypothesis3: version used in IJRR13
//Evaluates hypothesis for surfaces  
//NOTE: m_pMem2 is used
int CRVLPSuLMBuilder::EvaluateHypothesis3Simple(	
	CRVLPSuLM * pSPSuLM,
	RVLPSULM_HYPOTHESIS *pHypothesis,
	bool bDynamicSurfaceDetection)
{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
	m_fpDebug = fopen("C:\\RVL\\Debug\\HypEval.dat", "w");

	fprintf(m_fpDebug, "Hypothesis%d\n", pHypothesis->Index);
#endif

	double StartTime = m_pTimer->GetTime();

	double CellSize = (double)m_CellSize2;

	int minwS = m_pPSD->m_minSampleSize;
	int maxwS = minwS + m_pPSD->m_maxSampleSize;
	int maxiCell = m_nCols2 - 1;
	int maxjCell = m_nRows2 - 1;

	CRVLPSuLM *pMPSuLM = pHypothesis->pMPSuLM;

	double *RSM = pHypothesis->PoseSM.m_Rot;
	double *tSM = pHypothesis->PoseSM.m_X;

	int nSSurfaces = pSPSuLM->m_n3DSurfacesTotal;
	int nMSurfaces = pMPSuLM->m_n3DSurfacesTotal;

	double fu = m_pStereoVision->m_KinectParams.depthFu;
	double fv = m_pStereoVision->m_KinectParams.depthFv;
	double f = RVLMAX(fu, fv);
	double uc = m_pStereoVision->m_KinectParams.depthUc;
	double vc = m_pStereoVision->m_KinectParams.depthVc;

	int i;
	int iCell, jCell;
	RVLPSULM_CELL2 *pCell;
	CRVL3DSurface2 *pM3DSurface, *pS3DSurface; 
	RVLQLIST_PTR_ENTRY *pSamplePtr;
	double *XS;
	RVL3DSURFACE_SAMPLE *pSampleS;

	for (i = 0; i < nSSurfaces; i++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

		pS3DSurface->m_Flags &= ~RVL3DSURFACE_FLAG_REMOVED;

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

		while (pSampleS)
		{
			pSampleS->Flags = 0x00;

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}
	}

#ifdef NEVER	// filling m_CellArray2 is done in InitHypothesisEvaluation3()

	//Reset CellArray2
	memcpy(m_CellArray2, m_EmptyCellArray2, m_nCells2 * sizeof(RVLPSULM_CELL2));
			
	//RVLQLIST_PTR_ENTRY *CellMem = new RVLQLIST_PTR_ENTRY[pSPSuLM->m_nSurfaceSamples];

	pSamplePtr = CellMem;

	//fill CellArray2 with ptrs. to SSurface samples

	RVLQLIST *pSampleList;

	for(i=0;i<nSSurfaces;i++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

		pS3DSurface->m_Flags &= ~RVL3DSURFACE_FLAG_REMOVED;

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

		while(pSampleS)
		{
			pSampleS->Flags = 0x00;

			XS = pSampleS->X;

			iCell = (int)((fu * XS[0] / XS[2] + uc) / CellSize);
			jCell = (int)((fv * XS[1] / XS[2] + vc) / CellSize);
			
			pCell = m_CellArray2 + jCell * m_nCols2 + iCell;

			pSamplePtr->Ptr = pSampleS;

			pSampleList = &(pCell->surfaceList);

			RVLQLIST_ADD_ENTRY(pSampleList, pSamplePtr);

			pSamplePtr++;

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}
	}	
#endif	// NEVER

	// match model samples to scene samples

	int nMatches = nSSurfaces * nMSurfaces;

	if(m_MatchMatrix)
		delete[] m_MatchMatrix;

	m_MatchMatrix = new BYTE [nMatches];

	memset(m_MatchMatrix, 0, nMatches);

	int *SCoverage = new int[pSPSuLM->m_nSurfaceSamples];

	memset(SCoverage, 0, pSPSuLM->m_nSurfaceSamples * sizeof(int));

	int *MCoverage = new int[pMPSuLM->m_nSurfaceSamples];

	memset(MCoverage, 0, pMPSuLM->m_nSurfaceSamples * sizeof(int));

	int uM, vM, uS, vS;
	double fuM, fvM;
	double *XM, *VM, *VS, XMS[3], dPMS[3], tmp3x1[3];
	int w;
	int wM, wS;
	int firstiCell, lastiCell, firstjCell, lastjCell;
	int euvTol;
	double e, d;
	RVL3DSURFACE_SAMPLE *pSampleM;
	double ddS, z, rM, s, XYTol;
	int iMatch;
	int wTol;
	double *NS, *NM;
	double PM1[3], PM2[3], dPM[3], NMS[3];
	double stdXM, sinRNS;
	double U[3], R[3];
	BOOL bMOcluded, bSOcluded, bMSMatch, bNM;
	double pM, qM, pS, qS, csNTol, fTmp;
	RVLRECT SWindow, MWindow;
	int CoverageArea;

	for(i=0;i<nMSurfaces;i++)
	{
		pM3DSurface = pMPSuLM->m_3DSurfaceArray[i];

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
		fprintf(m_fpDebug, "M%d\n", pM3DSurface->m_Index);
#endif

		pM3DSurface->m_Flags &= ~(RVL3DSURFACE_FLAG_REMOVED | RVLOBJ2_FLAG_MATCHED | RVLOBJ2_FLAG_MARKED);

		NM = pM3DSurface->m_N;

		RVLMULMX3X3TVECT(RSM, NM, NMS)

		pSampleM = (RVL3DSURFACE_SAMPLE *)(pM3DSurface->m_Samples.pFirst);

		while(pSampleM)
		{
			pSampleM->Flags = 0x00;

			XM = pSampleM->X;
			VM = pSampleM->V;

			stdXM = (pSampleM->stdX + m_SampleMatchDistTol) * m_SampleMatchUncertCoef;

			pM = pSampleM->wz / f / (m_SampleMatchUncertCoef * pSampleM->stdX) - pSampleM->stdN[0];
			qM = pSampleM->stdN[1];

			fTmp = sqrt(pM * pM + qM * qM);

			pM /= fTmp;
			qM /= fTmp;

			bNM = (pM >= qM);

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
#endif

			RVLSCALE3VECTOR(VM, stdXM, dPM)
			RVLDIF3VECTORS(XM, tSM, tmp3x1)
			RVLMULMX3X3TVECT(RSM, tmp3x1, XMS)
			RVLMULMX3X3TVECT(RSM, dPM, dPMS)

			z = XMS[2];

			fuM = fu * XMS[0] / z + uc;
			fvM = fv * XMS[1] / z + vc;

			uM = DOUBLE2INT(fuM);
			vM = DOUBLE2INT(fvM);

			RVLSUM3VECTORS(XMS, dPMS, PM1)
			RVLDIF3VECTORS(XMS, dPMS, PM2)

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
			int uMM = DOUBLE2INT(fu * XM[0] / XM[2] + uc);
			int vMM = DOUBLE2INT(fv * XM[1] / XM[2] + vc);

			fprintf(m_fpDebug, "MW%d: UM=(%d, %d); UMS=(%d, %d); stdXM=%5.0lf; PM1=(%6.0lf, %6.0lf, %6.0lf); PM2=(%6.0lf, %6.0lf, %6.0lf)", 
				pSampleM->Index, uMM, vMM, uM, vM, stdXM, PM1[0], PM1[1], PM1[2], PM2[0], PM2[1], PM2[2]);

			if(bNM)
				fprintf(m_fpDebug, "; MN\n");
			else
				fprintf(m_fpDebug, "\n");

			//FILE *fpSample = fopen("C:\\RVL\\Debug\\Sample.dat", "r");

			//int uSample, vSample;
	
			//fscanf(fpSample, "%d\t%d\n", &uSample, &vSample);

			//fclose(fpSample);

			//if(uMM == uSample && vMM == vSample)
			//	int debug = 0;
#endif

			rM = sqrt(RVLDOTPRODUCT3(XMS, XMS));

			RVLSCALE3VECTOR2(XMS, rM, R)

			XYTol = m_SampleMatchAngleTol * rM + m_SampleMatchDistTol;

			wTol = (int)(f*XYTol/z) + 1;

			wM = (int)(pSampleM->wz / z) + 1;

			MWindow.left = uM - wM;
			MWindow.right = uM + wM;
			MWindow.top = vM - wM;
			MWindow.bottom = vM + wM;
			
			//w = RVLMAX(wM - minwS, maxwS - wM) + wTol;
			w = RVLMIN(wM, maxwS) + wTol;

			firstiCell = RVLMAX((uM - w) / m_CellSize2, 0);
			lastiCell = RVLMIN((uM + w) / m_CellSize2, maxiCell);
			firstjCell = RVLMAX((vM - w) / m_CellSize2, 0);
			lastjCell = RVLMIN((vM + w) / m_CellSize2, maxjCell);

			for(jCell = firstjCell; jCell <= lastjCell; jCell++)
				for(iCell = firstiCell; iCell <= lastiCell; iCell++)
				{
					pCell = m_CellArray2 + jCell * m_nCols2 + iCell;

					pSamplePtr = (RVLQLIST_PTR_ENTRY *)(pCell->surfaceList.pFirst);

					while(pSamplePtr)
					{
						pSampleS = (RVL3DSURFACE_SAMPLE *)(pSamplePtr->Ptr);

						XS = pSampleS->X;

						uS = DOUBLE2INT(fu * XS[0] / XS[2] + uc);
						vS = DOUBLE2INT(fv * XS[1] / XS[2] + vc);

						wS = (int)(pSampleS->wz / XS[2]) + 1;

						//euvTol = abs(wM - wS) + wTol;
						euvTol = RVLMIN(wM, wS) + wTol;

						if(abs(uM - uS) <= euvTol && abs(vM - vS) <= euvTol)
						{
							SWindow.left = uS - wS;
							SWindow.right = uS + wS;
							SWindow.top = vS - wS;
							SWindow.bottom = vS + wS;

							CoverageArea = (RVLMIN(SWindow.right, MWindow.right) - RVLMAX(SWindow.left, MWindow.left)) * 
								(RVLMIN(SWindow.bottom, MWindow.bottom) - RVLMAX(SWindow.top, MWindow.top));
							
							if(CoverageArea > 0)
							{
								SCoverage[pSampleS->Index] += (25 * CoverageArea / (wS * wS));
								MCoverage[pSampleM->Index] += (25 * CoverageArea / (wM * wM));
							}

							pS3DSurface = pSampleS->pSurface;

							NS = pS3DSurface->m_N;

							VS = pSampleS->V;

							ddS = m_SampleMatchUncertCoef * pSampleS->stdX * RVLDOTPRODUCT3(VS, NS);

							bMOcluded = bSOcluded = FALSE;				

							d = pS3DSurface->m_d - ddS;

							e = RVLDOTPRODUCT3(NS, PM1) - d;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
							fprintf(m_fpDebug, "\tSW%d: US=(%d, %d); S%d; e1=%6.0lf", pSampleS->Index, uS, vS, pS3DSurface->m_Index, e);
#endif

							if(e < 0)
							{
								RVLCROSSPRODUCT3(R, NS, tmp3x1)

								sinRNS = sqrt(RVLDOTPRODUCT3(tmp3x1, tmp3x1));

								if(sinRNS < -0.002 || sinRNS > 0.002)
								{
									RVLSCALE3VECTOR2(tmp3x1, sinRNS, tmp3x1)

									RVLCROSSPRODUCT3(R, tmp3x1, U)

									s = (d - RVLDOTPRODUCT3(NS, PM1)) / RVLDOTPRODUCT3(NS, U);

									if(s < -XYTol || s > XYTol)
										bSOcluded = TRUE;
								}
								else
									bSOcluded = TRUE;
							}
							else
							{
								d = pS3DSurface->m_d + ddS;

								e = RVLDOTPRODUCT3(NS, PM2) - d;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
								fprintf(m_fpDebug, "; e2=%6.0lf", e);
#endif

								if(e > 0)
								{
									RVLCROSSPRODUCT3(R, NS, tmp3x1)

									sinRNS = sqrt(RVLDOTPRODUCT3(tmp3x1, tmp3x1));

									if(sinRNS < -0.002 || sinRNS > 0.002)
									{
										RVLSCALE3VECTOR2(tmp3x1, sinRNS, tmp3x1)

										RVLCROSSPRODUCT3(R, tmp3x1, U)

										s = (d - RVLDOTPRODUCT3(NS, PM2)) / RVLDOTPRODUCT3(NS, U);

										if(s < -XYTol || s > XYTol)
											bMOcluded = TRUE;
									}
									else
										bMOcluded = TRUE;
								}
							}	// if(e >= 0)

							if(bSOcluded)
							{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
								fprintf(m_fpDebug, "; Oclusion-MS\n");
#endif

								if(pSampleS->Flags < RVL3DSURFACE_SAMPLE_FLAG_OCLUDED)
									pSampleS->Flags = RVL3DSURFACE_SAMPLE_FLAG_OCLUDED;

								if(pSampleM->Flags < RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
									pSampleM->Flags = RVL3DSURFACE_SAMPLE_FLAG_REMOVED;
							}
							else if(bMOcluded)
							{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
								fprintf(m_fpDebug, "; Oclusion-SM\n");
#endif

								if(pSampleM->Flags < RVL3DSURFACE_SAMPLE_FLAG_OCLUDED)
									pSampleM->Flags = RVL3DSURFACE_SAMPLE_FLAG_OCLUDED;

								if(pSampleS->Flags < RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
									pSampleS->Flags = RVL3DSURFACE_SAMPLE_FLAG_REMOVED;
							}
							else
							{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
								fprintf(m_fpDebug, "; MS-depth match");
#endif
								bMSMatch = TRUE;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_ORIENT_MATCH
								if(bNM)
								{
									pS = pSampleS->wz / f / (m_SampleMatchUncertCoef * pSampleS->stdX) - pSampleS->stdN[0];
									qS = pSampleS->stdN[1];

									fTmp = sqrt(pS * pS + qS * qS);

									pS /= fTmp;
									qS /= fTmp;

									if(pS >= qS)
									{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
										fprintf(m_fpDebug, "; NS");
#endif
										csNTol = pM * pS - qM * qS;

										if(csNTol >= COS45)
										{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
											fprintf(m_fpDebug, "; dNMS");
#endif
											if(RVLDOTPRODUCT3(NMS, NS) < csNTol)
												bMSMatch = FALSE;
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
											else
												fprintf(m_fpDebug, "; normal match!");
#endif
										}
									}
								}	// if(bNM)
#endif

								if(bMSMatch)
								{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
									fprintf(m_fpDebug, "; match\n");
#endif

									if(pSampleS->Flags != RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
									{
										pSampleS->Flags = RVL3DSURFACE_SAMPLE_FLAG_MATCHED;

										iMatch = pS3DSurface->m_Index * nMSurfaces + pM3DSurface->m_Index;

										m_MatchMatrix[iMatch]++; 
									}

									pSampleM->Flags = RVL3DSURFACE_SAMPLE_FLAG_MATCHED;
								}
								else
								{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
									fprintf(m_fpDebug, "; normals do not match\n");
#endif
									//if(pSampleM->Flags < RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
									//	pSampleM->Flags = RVL3DSURFACE_SAMPLE_FLAG_REMOVED;

									//if(pSampleS->Flags < RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
									//	pSampleS->Flags = RVL3DSURFACE_SAMPLE_FLAG_REMOVED;
								}
							}	// if(!bSOcluded && !bMOcluded)
						}	// if(abs(uM - uS) <= euvTol && abs(vM - vS) <= euvTol)

						pSamplePtr = (RVLQLIST_PTR_ENTRY *)(pSamplePtr->pNext);
					}	// while (pSamplePtr)
				}	// for(iCell = firstiCell; iCell <= lastiCell; iCell++)

			pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);
		}	// while(pSampleM)
	}	// for every model surface

	// classify surfaces 

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT
#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
	int nMatchedSamples;
	int nRemovedSamples;
#endif
	double dsrTime = m_pTimer->GetTime();

	for(i=0;i<nMSurfaces;i++)
	{
		pM3DSurface = pMPSuLM->m_3DSurfaceArray[i];

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
		nMatchedSamples = nRemovedSamples = 0;
#endif

		pSampleM = (RVL3DSURFACE_SAMPLE *)(pM3DSurface->m_Samples.pFirst);

		while(pSampleM)
		{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
			if(pSampleM->Flags == RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
			{
				if(MCoverage[pSampleM->Index] > m_minSurfaceSampleCoverage)
				{
					pM3DSurface->m_Flags |= RVL3DSURFACE_FLAG_REMOVED;

					break;
				}
				else
					pSampleM->Flags = 0x00;
			}
#else
			if(pSampleM->Flags == RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
				nMatchedSamples++;
			if(pSampleM->Flags == RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
				nRemovedSamples++;
#endif

			pSampleM = (RVL3DSURFACE_SAMPLE *)(pSampleM->pNext);
		}

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
		if(nRemovedSamples > 0)
			if(100 * nRemovedSamples / (nMatchedSamples + nRemovedSamples) > 20)
				pM3DSurface->m_Flags |= RVL3DSURFACE_FLAG_REMOVED;
#endif
	}

	for(i=0;i<nSSurfaces;i++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
		nMatchedSamples = nRemovedSamples = 0;
#endif

		while(pSampleS)
		{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
			if(pSampleS->Flags == RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
			{
				if(SCoverage[pSampleS->Index] > m_minSurfaceSampleCoverage)
				{
					pS3DSurface->m_Flags |= RVL3DSURFACE_FLAG_REMOVED;

					break;
				}
				else
					pSampleS->Flags = 0x00;
			}			
#else
			if(pSampleS->Flags == RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
				nMatchedSamples++;
			if(pSampleS->Flags == RVL3DSURFACE_SAMPLE_FLAG_REMOVED)
				nRemovedSamples++;
#endif

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}

#ifndef RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT_METHOD1
		if(nRemovedSamples > 0)
			if(100 * nRemovedSamples / (nMatchedSamples + nRemovedSamples) > 20)
				pS3DSurface->m_Flags |= RVL3DSURFACE_FLAG_REMOVED;
#endif

	}

	m_DynamicSurfRejectTime = m_pTimer->GetTime() - dsrTime;
#endif	// RVLPSULMBUILDER_HYPOTHESES_EVAL3_DYNAMIC_SURF_REJECT

	if (bDynamicSurfaceDetection)
	{
		m_HypEvalTime = m_pTimer->GetTime() - StartTime;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
		fclose(m_fpDebug);
#endif

		return 0;
	}

	// Surface matching

	int nMatchedSurfaces = 0;

	CRVLMPtrChain MatchList(m_pMem2);

	RVLPSULM_MATCH *MatchBuff = new RVLPSULM_MATCH[nMatches];
	RVLPSULM_MATCH **SortedMatchArray = new RVLPSULM_MATCH *[nMatches];

	RVLPSULM_MATCH *pMatch2 = MatchBuff;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_MATCH_COUNT
	int nMatches2 = 0;
#endif

	int j;
	BYTE *pMatch;

	for(i=0;i<nSSurfaces;i++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

		pS3DSurface->m_Flags &= ~(RVLOBJ2_FLAG_MATCHED | RVLOBJ2_FLAG_MARKED);

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

		while(pSampleS)
		{
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_MATCH_COUNT
			if(pSampleS->Flags == RVL3DSURFACE_SAMPLE_FLAG_MATCHED)
				nMatches2++;
#endif

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}

		if((pS3DSurface->m_Flags & RVL3DSURFACE_FLAG_REMOVED) == 0)
		{
			//nMatches2 = 0;

			pMatch = m_MatchMatrix + i * nMSurfaces;

			for(j=0;j<nMSurfaces;j++, pMatch++)
			{
				if(*pMatch >= m_minSurfaceSamplesForMatch)
				{
					pM3DSurface = pMPSuLM->m_3DSurfaceArray[j];

					if((pM3DSurface->m_Flags & RVL3DSURFACE_FLAG_REMOVED) == 0)
					{
						pMatch2->vpSObject = pS3DSurface;
						pMatch2->vpMObject = pM3DSurface;
						pMatch2->cost = (*pMatch);

						MatchList.Add(pMatch2);

						pMatch2++;

						//nMatches2++;

						//if(nMatches2 == 3)
						//{
						//	nMatchedSurfaces++;

						//	break;
						//}
					}
				}				
			}
		}
	}	

	// reject the hypothesis if the surfaces from which it is generated are removed

	BOOL bSeedTest = TRUE;

	//CRVL3DSurface2 **ppS3DSurface = (CRVL3DSurface2 **)(pHypothesis->MatchArray);
	//CRVL3DSurface2 **pS3DSufraceArrayEnd = ppS3DSurface + pHypothesis->nMatches;
	//CRVL3DSurface2 **ppM3DSurface = pS3DSufraceArrayEnd;

	//for(; ppS3DSurface < pS3DSufraceArrayEnd; ppS3DSurface++, ppM3DSurface++)
	//{
	//	if((*ppS3DSurface)->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
	//	{
	//		bSeedTest = FALSE;

	//		break;
	//	}

	//	if((*ppM3DSurface)->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
	//	{
	//		bSeedTest = FALSE;

	//		break;
	//	}
	//}

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
	FILE *fpEvaluationMatches = fopen("C:\\RVL\\ExpRez\\HypEval3Matches.dat", "w");
#endif

	double scoreTotal = 0.0;

	if(bSeedTest)
	{
		// final plausibility estimation

		RVLBubbleSort<RVLPSULM_MATCH>(&MatchList, SortedMatchArray, TRUE);

		double StartTime = m_pTimer->GetTime();

		double *RAB = pHypothesis->PoseSM.m_Rot;

		double log2pi = log(2.0 * PI);

		double *RSA, *RMB;
		double CSM[2 * 2], CMM[2 * 2];
		double sMean[2], eSM[2];
		double cSS11, cSS22;
		double detCSM, detCMM, detCMMCSM;
		double sSM[2], JSM[2 * 2];
		double score;
		double M2x2Tmp[2 * 2];

		for(i = 0; i < MatchList.m_nElements; i++)
		{
			pMatch2 = SortedMatchArray[i];

			pS3DSurface = (CRVL3DSurface2 *)(pMatch2->vpSObject);
			pM3DSurface = (CRVL3DSurface2 *)(pMatch2->vpMObject);

			if(((pS3DSurface->m_Flags | pM3DSurface->m_Flags) & RVLOBJ2_FLAG_MATCHED) == 0)
			{
				pS3DSurface->m_Flags |= (RVLOBJ2_FLAG_MATCHED | RVLOBJ2_FLAG_MARKED);
				pM3DSurface->m_Flags |= (RVLOBJ2_FLAG_MATCHED | RVLOBJ2_FLAG_MARKED);
				nMatchedSurfaces++;

				RSA = pS3DSurface->m_Pose.m_Rot;
				RMB = pM3DSurface->m_Pose.m_Rot;
				
				// RSB = RAB * RSA

				double RSB[3 * 3];

				RVLMXMUL3X3(RAB, RSA, RSB)

				// sSM = [xMB' yMB'] * zSB;

				sSM[0] = RVLMULCOLCOL3(RMB, RSB, 0, 2);
				sSM[1] = RVLMULCOLCOL3(RMB, RSB, 1, 2);

				// JSM = [xMB' yMB'] * [xSB ySB];			

				RVLMXEL(JSM, 2, 0, 0) = RVLMULCOLCOL3(RMB, RSB, 0, 0);
				RVLMXEL(JSM, 2, 0, 1) = RVLMULCOLCOL3(RMB, RSB, 0, 1);
				RVLMXEL(JSM, 2, 1, 0) = RVLMULCOLCOL3(RMB, RSB, 1, 0);
				RVLMXEL(JSM, 2, 1, 1) = RVLMULCOLCOL3(RMB, RSB, 1, 1);

				// CMM = diag(pM3DSurface->m_varq[0:1])

				RVLMXEL(CMM, 2, 0, 0) = pM3DSurface->m_varq[0];
				RVLMXEL(CMM, 2, 0, 1) = 0.0;
				RVLMXEL(CMM, 2, 1, 1) = pM3DSurface->m_varq[1];

				// CSM = JSM * diag(pS3DSurface->m_varq[0:1]) * JSM'

				cSS11 = pS3DSurface->m_varq[0];
				cSS22 = pS3DSurface->m_varq[1];

				RVLMXEL(CSM, 2, 0, 0) = cSS11*JSM[0]*JSM[0] + cSS22*JSM[2]*JSM[2];
				RVLMXEL(CSM, 2, 0, 1) = cSS11*JSM[0]*JSM[1] + cSS22*JSM[2]*JSM[3];
				RVLMXEL(CSM, 2, 1, 1) = cSS11*JSM[1]*JSM[1] + cSS22*JSM[3]*JSM[3];

				// detCMMCSM = det(CMM + CSM)

				detCMMCSM = (CMM[0] + CSM[0]) * (CMM[3] + CSM[3]) - CSM[1] * CSM[1];

				// sMean = inv(inv(CMM) + inv(CSM))*inv(CSM)*sSM

				detCMM = CMM[0] * CMM[3];
				detCSM = CSM[0] * CSM[3] - CSM[1] * CSM[1];

				M2x2Tmp[0] = CMM[0]*(CMM[3] + CSM[3]);
				M2x2Tmp[1] = -CMM[0]*CSM[1];
				M2x2Tmp[3] = CMM[3]*(CMM[0] + CSM[0]);

				sMean[0] = (M2x2Tmp[0] * sSM[0] + M2x2Tmp[1] * sSM[1]) / detCMMCSM;
				sMean[1] = (M2x2Tmp[1] * sSM[0] + M2x2Tmp[3] * sSM[1]) / detCMMCSM;

				// score = 2 * pi / sqrt(det(CMM + CSM))*exp(-0.5*((sMean'*inv(CMM)*sMean + (sSM - sMean)'*inv(CSM)*(sSM - sMean))))

				eSM[0] = sSM[0] - sMean[0];
				eSM[1] = sSM[1] - sMean[1];

				score = log2pi - 0.5 * (log(detCMMCSM) + RVLMAHDIST2(sMean, CMM, detCMM) + RVLMAHDIST2(eSM, CSM, detCSM));

				if(score > 0.0)
					scoreTotal += score;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
				fprintf(fpEvaluationMatches, "S%d-M%d\tnSamp:%d\tscore:%lf\n", pS3DSurface->m_Index, pM3DSurface->m_Index, pMatch2->cost, score);
#endif
			}
		}

		double ExecutionTime = m_pTimer->GetTime() - StartTime;

	}	// if(bSeedTest)
	else
	{
		scoreTotal = 0.0;

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
		fprintf(fpEvaluationMatches, "Seed test failed!\n");
#endif
	}

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_LOG
	fclose(fpEvaluationMatches);
#endif

	// dealocate memory

	m_pMem2->Clear();

	delete[] SortedMatchArray;
	delete[] MatchBuff;
	delete[] m_CellMem;
	delete[] SCoverage;
	delete[] MCoverage;

	/////

#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL_DEBUG_LOG
	fclose(m_fpDebug);
#endif

	//return nMatchedSurfaces;
#ifdef RVLPSULMBUILDER_HYPOTHESES_EVAL3_MATCH_COUNT
	return nMatches2;
#else
	return DOUBLE2INT(1000.0 * scoreTotal);
#endif
}

void CRVLPSuLMBuilder::ModelFusion(RVLPSULM_HYPOTHESIS *pHypothesis)
{
	double cszThr = 0.5;	// cos(60.0 deg)
	double r2Thr = 2.5 / 4.0;
	r2Thr *= r2Thr;

	CRVLMem *pMem = &(m_ModelFusion.m_Mem);

	CRVLPSuLM *pMPSuLM = pHypothesis->pMPSuLM;

	double *RSM = pHypothesis->PoseSM.m_Rot;
	double *tSM = pHypothesis->PoseSM.m_X;

	RVLQLIST SurfaceList;

	RVLQLIST *pSurfaceList = &SurfaceList;

	RVLQLIST_INIT(pSurfaceList)

	RVLQLIST LineList;

	RVLQLIST *pLineList = &LineList;

	RVLQLIST_INIT(pLineList)

	double *RM_M, *tM_M;
	double *RM_S, *tM_S;
	double csz;
	int i;
	CRVLPSuLM *pMPSuLM_;
	CRVL3DSurface2 *pSurf;
	CRVL3DLine2 *pLine;
	double cFS[3];
	double *cFM_, *X1, *X2;
	double cLM_[3];
	double r2;
	RVLPSULM_MODEL_FUSION_FEATURE *pFeature;
	double tmpV3x1[3];

	RVLPSULM_NEIGHBOR2 *pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pMPSuLM->m_LocalMap.pFirst);

	while(pNeighbor)
	{
		// check if the viewing angle of the neighboring PSuLM is sufficiently close to the viewing angle of the scene		

		RM_M = pNeighbor->PoseRel.m_Rot;

		csz = RVLMULCOLCOL3(RSM, RM_M, 2, 2);

		if(csz < cszThr)
		{
			pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);

			continue;
		}

		// compute the pose of the neighboring PSuLM relative to the scene

		tM_M = pNeighbor->PoseRel.m_X;

		pMPSuLM_ = pNeighbor->pMPSuLM;

		RM_S = m_ModelFusion.RM_S + 9 * pMPSuLM_->m_Index;
		tM_S = m_ModelFusion.tM_S + 3 * pMPSuLM_->m_Index;

		RVLCOMPTRANSF3DWITHINV(RSM, tSM, RM_M, tM_M, RM_S, tM_S, tmpV3x1)

		// put all surfaces contained in the FoV into SurfaceList
		
		for(i = 0; i < pMPSuLM_->m_n3DSurfacesTotal; i++)
		{
			pSurf = pMPSuLM_->m_3DSurfaceArray[i];

			cFM_ = pSurf->m_Pose.m_X;

			RVLTRANSF3(cFM_, RM_S, tM_S, cFS)

			r2 = (cFS[0] * cFS[0] + cFS[1] * cFS[1]) / (cFS[2] * cFS[2]);

			if(r2 > r2Thr)
				continue;

			RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_MODEL_FUSION_FEATURE, pFeature)

			RVLQLIST_ADD_ENTRY(pSurfaceList, pFeature)

			pFeature->vpFeature = pSurf;
			pFeature->pMPSuLM = pMPSuLM_;
		}
		
		// put all lines contained in the FoV into LineList
		
		for(i = 0; i < pMPSuLM_->m_n3DLinesTotal; i++)
		{
			pLine = pMPSuLM_->m_3DLineArray[i];

			X1 = pLine->m_X[0];
			X2 = pLine->m_X[1];

			RVLSUM3VECTORS(X1, X2, cLM_)

			RVLSCALE3VECTOR(cLM_, 0.5, cLM_)

			RVLTRANSF3(cLM_, RM_S, tM_S, cFS)

			r2 = (cFS[0] * cFS[0] + cFS[1] * cFS[1]) / (cFS[2] * cFS[2]);

			if(r2 > r2Thr)
				continue;

			RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_MODEL_FUSION_FEATURE, pFeature)

			RVLQLIST_ADD_ENTRY(pLineList, pFeature)

			pFeature->vpFeature = pLine;
			pFeature->pMPSuLM = pMPSuLM_;
		}

		pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
	}	// for every neighboring PSuLM

	CRVL3DSurface2 **ppSurf = m_ModelFusion.m_SurfaceArray;
	CRVLPSuLM **ppMPSuLM = m_ModelFusion.m_SurfacePSuLMArray;

	pFeature = (RVLPSULM_MODEL_FUSION_FEATURE *)(SurfaceList.pFirst);

	while(pFeature)
	{
		*(ppSurf++) = (CRVL3DSurface2 *)(pFeature->vpFeature);
		*(ppMPSuLM) = pFeature->pMPSuLM;

		pFeature = (RVLPSULM_MODEL_FUSION_FEATURE *)(pFeature->pNext);
	}

	m_ModelFusion.m_nSurfaces = ppSurf - m_ModelFusion.m_SurfaceArray;

	CRVL3DLine2 **ppLine = m_ModelFusion.m_LineArray;
	ppMPSuLM = m_ModelFusion.m_LinePSuLMArray;

	pFeature = (RVLPSULM_MODEL_FUSION_FEATURE *)(LineList.pFirst);

	while(pFeature)
	{
		*(ppLine++) = (CRVL3DLine2 *)(pFeature->vpFeature);
		*(ppMPSuLM) = pFeature->pMPSuLM;

		pFeature = (RVLPSULM_MODEL_FUSION_FEATURE *)(pFeature->pNext);
	}

	m_ModelFusion.m_nLines = ppLine - m_ModelFusion.m_LineArray;

	m_ModelFusion.m_Mem.Clear();

	if(m_SurfaceMatchData.Cp_)
		delete[] m_SurfaceMatchData.Cp_;

	m_SurfaceMatchData.Cp_ = new double[3 * 3 * m_ModelFusion.m_nSurfaces];

	if(m_SurfaceMatchData.invCp_)
		delete[] m_SurfaceMatchData.invCp_;

	m_SurfaceMatchData.invCp_ = new double[3 * 3 * m_ModelFusion.m_nSurfaces];

	if(m_SurfaceMSArray)
		delete[] m_SurfaceMSArray;

	m_SurfaceMSArray = new CRVL3DSurface2[m_ModelFusion.m_nSurfaces];
}

double CRVLPSuLMBuilder::EvaluateHypothesis4(	CRVLPSuLM * pSPSuLM,
												RVLPSULM_HYPOTHESIS *pHypothesis,
												DWORD Flags)
{
	if (Flags & RVLPSULMBUILDER_HYPEVAL4_FLAG_DYNAMIC_SURF_DETECT)
		EvaluateHypothesis3(pSPSuLM, pHypothesis, true);

	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;
	int nSSurfs = pSPSuLM->m_n3DSurfaces;

	CRVL3DLine2 **SLineArray = pSPSuLM->m_3DLineArray;
	int nSLines = pSPSuLM->m_n3DLines;

	int nSFeatures = RVLMAX(nSSurfs, nSLines);

	CRVLPSuLM *pMPSuLM = pHypothesis->pMPSuLM;
	
	CRVL3DSurface2 **MSurfArray;
	int nMSurfs;	
	CRVL3DLine2 **MLineArray;
	int nMLines;

	if(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_MODEL_FUSION)
	{
		ModelFusion(pHypothesis);

		MSurfArray = m_ModelFusion.m_SurfaceArray;
		nMSurfs = m_ModelFusion.m_nSurfaces;
		MLineArray = m_ModelFusion.m_LineArray;
		nMLines = m_ModelFusion.m_nLines;
	}
	else
	{
		MSurfArray = pMPSuLM->m_3DSurfaceArray;
		// nMSurfs = pMPSuLM->m_n3DSurfaces;
		nMSurfs = pMPSuLM->m_n3DSurfacesTotal;

		MLineArray = pMPSuLM->m_3DLineArray;
		// nMLines = pMPSuLM->m_n3DLines;
		nMLines = pMPSuLM->m_n3DLinesTotal;
	}

	int nMFeatures = nMSurfs + nMLines;

	double *RSM = pHypothesis->PoseSM.m_Rot;
	double *tSM = pHypothesis->PoseSM.m_X;

	CRVL3DPose PoseMS;

	double *RMS = PoseMS.m_Rot;
	double *tMS = PoseMS.m_X;

	RVLINVTRANSF3D(RSM, tSM, RMS, tMS)

	// allocate m_LineMSArray

	if(m_LineMSArray)
		delete[] m_LineMSArray;

	m_LineMSArray = new CRVL3DLine2[nMLines];

	RVL3DLINE_EXTENDED_DATA *LineMSData = new RVL3DLINE_EXTENDED_DATA[nMLines];

	// allocate match matrix

	int nMatches = nSFeatures * nMFeatures;

	if(m_MatchMatrix)
		delete[] m_MatchMatrix;

	m_MatchMatrix = new BYTE [nMatches];

	if(m_MatrixSMCost)
		delete[] m_MatrixSMCost;

	m_MatrixSMCost = new double[nMatches];

	if(m_SMatchArray)
		delete[] m_SMatchArray;

	m_SMatchArray = new RVLPSULM_SMATCH_DATA[nSSurfs + nSLines];

	// transform model surfaces into the scene reference frame

	CRVL3DSurface2 **pSurfArrayEnd = MSurfArray + nMSurfs;

	CRVL3DSurface2 *pMSurf = m_SurfaceMSArray;

	double *Cp_ = m_SurfaceMatchData.Cp_;
	double *invCp_ = m_SurfaceMatchData.invCp_;

	double Mx3x3Tmp[3*3];
	
	CRVL3DSurface2 **ppSurf;
	CRVL3DSurface2 *pSurf;
	double *RF_M, *tF_M, *RF_S, *tF_S, *N;
	double varx, vary, varz;
	double detC;

	for(ppSurf = MSurfArray; ppSurf < pSurfArrayEnd; ppSurf++, pMSurf++, Cp_ += 9, invCp_ += 9)
	{
		pSurf = *ppSurf;

		pSurf->m_Flags &= ~RVLOBJ2_FLAG_MARKED;

		pMSurf->m_Flags = pSurf->m_Flags;

		if (pSurf->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
			continue;

		RF_M = pSurf->m_Pose.m_Rot;
		tF_M = pSurf->m_Pose.m_X;
		RF_S = pMSurf->m_Pose.m_Rot;
		tF_S = pMSurf->m_Pose.m_X;
		RVLCOMPTRANSF3D(RMS, tMS, RF_M, tF_M, RF_S, tF_S)

		N = pMSurf->m_N;
		
		RVLCOPYCOLMX3X3(RF_S, 2, N)

		pMSurf->m_Index = pMSurf - m_SurfaceMSArray;

		RVLCOPY3VECTOR(pSurf->m_varq, pMSurf->m_varq)

		varx = pSurf->m_EigenValues[0] * pSurf->m_EigenValues[0];
		vary = pSurf->m_EigenValues[1] * pSurf->m_EigenValues[1];
		varz = pSurf->m_varq[2];

		RVLSCALECOL3(RF_S, 0, varx, Mx3x3Tmp)
		RVLSCALECOL3(RF_S, 1, vary, Mx3x3Tmp)
		RVLSCALECOL3(RF_S, 2, varz, Mx3x3Tmp)

		RVLMXMUL3X3T2(Mx3x3Tmp, RF_S, Cp_)

		RVLINVCOV3(Cp_, invCp_, detC)
	}	

	// matching of surface segments

	ppSurf = SSurfArray;

	double P = 0.0;

	int i, j;
	CRVL3DSurface2 *pSSurf;
	double PFeature;
	double maxP;

	for(i = 0; i < nSSurfs; i++, ppSurf++)
	{
		//if(i == 26)
		//	int debug = 0;

		pSSurf = *ppSurf;

		pSSurf->m_Flags &= ~RVLOBJ2_FLAG_MARKED;

		m_SMatchArray[i].iMFeature = -1;
		m_SMatchArray[i].POrientMatch = 0.0;
		m_SMatchArray[i].b = false;

		if (pSSurf->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
			continue;

		maxP = 0.0;

		pMSurf = m_SurfaceMSArray;

		for(j = 0; j < nMSurfs; j++, pMSurf++)
		{
			//if(i == 9 && j == 7 || i == 2 && j == 2)
			//	int debug = 0;

			if (pMSurf->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
				continue;

			if(pSSurf->Match4(pMSurf, &m_SurfaceMatchData, PFeature))
			{
				RVLMXEL(m_MatchMatrix, nMFeatures, i, j) = 1;
				RVLMXEL(m_MatrixSMCost, nMFeatures, i, j) = PFeature;
				pSSurf->m_Flags |= RVLOBJ2_FLAG_MARKED;
				MSurfArray[pMSurf->m_Index]->m_Flags |= RVLOBJ2_FLAG_MARKED;

				if(PFeature > maxP)
				{
					maxP = PFeature;
					m_SMatchArray[i].iMFeature = j;
					m_SMatchArray[i].POrientMatch = m_SurfaceMatchData.POrientMatch;
					m_SMatchArray[i].b = true;
				}
			}
			else
			{
				RVLMXEL(m_MatchMatrix, nMFeatures, i, j) = 0;
				RVLMXEL(m_MatrixSMCost, nMFeatures, i, j) = 0.0;
			}
		}

		m_SMatchArray[i].P = maxP;

		P += maxP;
	}	

	if (Flags & RVLPSULMBUILDER_HYPEVAL4_FLAG_DYNAMIC_SURF_DETECT)
	{
		ppSurf = SSurfArray;

		for (i = 0; i < nSSurfs; i++, ppSurf++)
		{
			pSSurf = *ppSurf;

			pSSurf->m_Flags &= ~RVL3DSURFACE_FLAG_REMOVED;
		}

		for (ppSurf = MSurfArray; ppSurf < pSurfArrayEnd; ppSurf++)
		{
			pMSurf = *ppSurf;

			pMSurf->m_Flags &= ~RVL3DSURFACE_FLAG_REMOVED;
		}
	}

	//return P;

	// transform model lines into the scene reference frame

	CRVL3DLine2 **pLineArrayEnd = MLineArray + nMLines;

	CRVL3DLine2 *pMLine = m_LineMSArray;

	RVL3DLINE_EXTENDED_DATA *pLineData = LineMSData;

	CRVL3DLine2 **ppLine;
	CRVL3DLine2 *pLine;

	for(ppLine = MLineArray; ppLine < pLineArrayEnd; ppLine++, pMLine++, pLineData++)
	{
		pLine = *ppLine;

		pLine->m_Flags &= ~RVLOBJ2_FLAG_MARKED;

		pMLine->m_pData = (BYTE *)pLineData;

		pMLine->Transform(pLine, &PoseMS);
	}

	// matching of line segments

	double *PLine = new double[nSLines];

	memset(PLine, 0, nSLines * sizeof(double));

	ppLine = SLineArray;

	CRVL3DLine2 *pSLine;
	int i_, j_;

	for(i = 0; i < nSLines; i++, ppLine++)
	{
		//if(i == 26)
		//	int debug = 0;

		i_ = i + nSSurfs;

		pSLine = *ppLine;

		pSLine->m_Flags &= ~RVLOBJ2_FLAG_MARKED;

		pMLine = m_LineMSArray;

		maxP = 0.0;

		m_SMatchArray[i_].iMFeature = -1;
		m_SMatchArray[i_].POrientMatch = 0.0;
		m_SMatchArray[i_].b = false;

		for(j = 0; j < nMLines; j++, pMLine++)
		{
			j_ = j + nMSurfs;

			//if(i == 5 && j == 39)
			//	int debug = 0;

			if(pSLine->Match(pMLine, &m_LineMatchData, tMS, PFeature))
			{
				RVLMXEL(m_MatchMatrix, nMFeatures, i, j_) = 1;
				RVLMXEL(m_MatrixSMCost, nMFeatures, i, j_) = PFeature;
				pSLine->m_Flags |= RVLOBJ2_FLAG_MARKED;
				MLineArray[j]->m_Flags |= RVLOBJ2_FLAG_MARKED;

				if(PFeature > maxP)
				{
					maxP = PFeature;
					m_SMatchArray[i_].iMFeature = j;	
					m_SMatchArray[i_].POrientMatch = m_LineMatchData.POrientMatch;
					m_SMatchArray[i_].b = true;
				}
			}
			else
			{
				RVLMXEL(m_MatchMatrix, nMFeatures, i, j_) = 0;
				RVLMXEL(m_MatrixSMCost, nMFeatures, i, j_) = 0.0;
			}
		}

		m_SMatchArray[i_].P = maxP;

		P += maxP;
	}

	delete[] LineMSData;

	return (Flags & RVLPSULMBUILDER_HYPEVAL4_FLAG_FIRST_ORDER_DEPENDENCY_TREE ? ConditionalProbabilityTree(pSPSuLM, pHypothesis->pMPSuLM) : P);
}

// This function uses m_pMem2

void CRVLPSuLMBuilder::CreateAutoMatchMatrix(CRVLPSuLM *pSPSuLM)
{
	DWORD MatchLinesFlags = 0x00000000;

	if(m_HypothesisEvaluationFlags & RVLPSULMBUILDER_HYPEVAL_FLAG_LINE_POSITION)
		MatchLinesFlags |= RVL3DLINE_MATCH_FLAG_POSITION;

	DWORD MatchSurfaceFlags = 0x00000000;

	if(m_HypothesisEvaluationFlags & RVLPSULMBUILDER_HYPEVAL_FLAG_SURFACE_POSITION)
		MatchSurfaceFlags |= RVL3DSURFACE_MATCH4_FLAG_POSITION;

	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;
	int nSSurfs = pSPSuLM->m_n3DSurfaces;

	CRVL3DLine2 **SLineArray = pSPSuLM->m_3DLineArray;
	int nSLines = pSPSuLM->m_n3DLines;

	int nSFeatures = nSSurfs + nSLines;

	int nSFeatures_ = RVLMAX(nSSurfs, nSLines);

	// allocate match matrix

	int nMatches = nSFeatures * nSFeatures_;

	if(m_AutoMatchMatrix)
		delete[] m_AutoMatchMatrix;

	m_AutoMatchMatrix = new double [nMatches];

	if(m_AutoMatchMem)
		delete[] m_AutoMatchMem;

	m_AutoMatchMem = new RVLPSULM_MATCH2[nMatches];

	RVLPSULM_MATCH2 *pMatch = m_AutoMatchMem;

	CRVLMPtrChain MatchList(m_pMem2);

#ifdef RVLPSULMBUILDER_AUTO_MATCH_MATRIX_DEBUG_LOG
	FILE *fp = fopen("C:\\RVL\\Debug\\AutoMatchMatrix.txt", "w");
#endif

	// matching of surface segments

	CRVL3DSurface2 **ppSurf = SSurfArray;

	int i, j;
	CRVL3DSurface2 **ppSurf_;
	CRVL3DSurface2 *pSSurf, *pSSurf_;
	double PFeature;

	for(i = 0; i < nSSurfs; i++, ppSurf++)
	{
		pSSurf = *ppSurf;

		ppSurf_ = SSurfArray;

		for(j = 0; j < nSSurfs; j++, ppSurf_++)
		{
			//if(i == 4 && j == 8)
			//	int debug = 0;

			if(j >= i)
				continue;

			pSSurf_ = *ppSurf_;

			if(pSSurf->Match4(pSSurf_, &m_SurfaceMatchData, PFeature, MatchSurfaceFlags))
			{
				RVLMXEL(m_AutoMatchMatrix, nSFeatures, i, j) = PFeature;

				if(PFeature > 0.0)
				{
					pMatch->vpSObject = pSSurf;
					pMatch->vpMObject = pSSurf_;
					pMatch->cost = PFeature;
					pMatch->Type = (RVLPSULM_MATCH_TYPE_SURFACE | RVLPSULM_MATCH_TYPE_AUTO);

					MatchList.Add(pMatch);

					pMatch++;
				}
			}
			else
				RVLMXEL(m_AutoMatchMatrix, nSFeatures, i, j) = 0.0;
		}
	}	

	// matching of line segments

	CRVL3DLine2 **ppLine, **ppLine_;

	ppLine = SLineArray;

	double V3Null[3];

	RVLNULL3VECTOR(V3Null);

	CRVL3DLine2 *pSLine, *pSLine_;
	int j_;

	for(i = 0; i < nSLines; i++, ppLine++)
	{
		pSLine = *ppLine;

		ppLine_ = SLineArray;

		for(j = 0; j < nSLines; j++, ppLine_++)
		{
			//if(i == 62 && j == 58)
			//	int debug = 0;

			if(j >= i)
				continue;

			pSLine_ = *ppLine_;

			j_ = j + nSSurfs;

			if (pSLine->Match(pSLine_, &m_LineMatchData, V3Null, PFeature, MatchLinesFlags))
			{
				RVLMXEL(m_AutoMatchMatrix, nSFeatures, i, j_) = PFeature;

				if(PFeature > 0.0)
				{
					pMatch->vpSObject = pSLine;
					pMatch->vpMObject = pSLine_;
					pMatch->cost = PFeature;
					pMatch->Type = (RVLPSULM_MATCH_TYPE_LINE | RVLPSULM_MATCH_TYPE_AUTO);

					MatchList.Add(pMatch);

					pMatch++;
				}
			}
			else
				RVLMXEL(m_AutoMatchMatrix, nSFeatures, i, j_) = 0.0;

#ifdef RVLPSULMBUILDER_AUTO_MATCH_MATRIX_DEBUG_LOG
			fprintf(fp, "L%d-L%d: %lf\n", i, j, RVLMXEL(m_AutoMatchMatrix, nSFeatures, i, j_));
#endif
		}
	}

	m_nAutoMatches = pMatch - m_AutoMatchMem;

#ifdef RVLPSULMBUILDER_AUTO_MATCH_MATRIX_DEBUG_LOG
	fclose(fp);
#endif

	// sort matches

	if(m_AutoMatchArray)
		delete[] m_AutoMatchArray;

	m_AutoMatchArray = new RVLPSULM_MATCH2 *[m_nAutoMatches + 1];

	RVLBubbleSort<RVLPSULM_MATCH2>(&MatchList, m_AutoMatchArray, TRUE);
}

double CRVLPSuLMBuilder::ConditionalProbabilityTree(CRVLPSuLM *pSPSuLM, 
													CRVLPSuLM *pMPSuLM,
													int *iSMMatchArray,
													int nSMSurfMatches,
													int nSMLineMatches,
													RVLPSULM_MATCH2 **AutoMatchArray_,
													int nAutoMatches_)
{
	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;
	int nSSurfs = pSPSuLM->m_n3DSurfaces;

	CRVL3DLine2 **SLineArray = pSPSuLM->m_3DLineArray;
	int nSLines = pSPSuLM->m_n3DLines;

	int nSFeatures = nSSurfs + nSLines;

	CRVL3DSurface2 **MSurfArray;
	CRVL3DLine2 **MLineArray;
	int nMSurfs, nMLines;
	int nSMSurfMatches_, nSMLineMatches_;
	
	if(pMPSuLM)
	{
		MSurfArray = pMPSuLM->m_3DSurfaceArray;
		nMSurfs = pMPSuLM->m_n3DSurfaces;
		
		MLineArray = pMPSuLM->m_3DLineArray;
		nMLines = pMPSuLM->m_n3DLines;

		if(iSMMatchArray)
		{
			nSMSurfMatches_ = nSMSurfMatches;
			nSMLineMatches_ = nSMLineMatches;
		}
		else
		{
			nSMSurfMatches_ = nSSurfs;
			nSMLineMatches_ = nSLines;
		}
	}
	else
		nMSurfs = nMLines = nSMSurfMatches_ = nSMLineMatches_ = 0;

	// sort scene-model matches

	RVLPSULM_MATCH2 *SMMatchMem = new RVLPSULM_MATCH2[nSFeatures];

	RVLPSULM_MATCH2 *pMatch = SMMatchMem;

	CRVLMem Mem;

	Mem.Create(nSFeatures * sizeof(RVLPTRCHAIN_ELEMENT) + sizeof(BYTE *) + 1);

	CRVLMPtrChain SMMatchList(&Mem);

	int i, i_;

	for(i = 0; i < nSMSurfMatches_; i++)
	{	
		i_ = (iSMMatchArray ? iSMMatchArray[i] : i);

		if(m_SMatchArray[i_].b)
		{
			pMatch->vpSObject = SSurfArray[i_];
			pMatch->vpMObject = MSurfArray[m_SMatchArray[i_].iMFeature];
			pMatch->cost = m_SMatchArray[i_].P;
			pMatch->Type = RVLPSULM_MATCH_TYPE_SURFACE;

			SMMatchList.Add(pMatch);

			pMatch++;
		}
	}

	int i__;

	for(i = 0; i < nSMLineMatches_; i++)
	{	
		i_ = (iSMMatchArray ? iSMMatchArray[i + nSMSurfMatches] : i);

		i__ = i_ + nSSurfs;

		if(m_SMatchArray[i__].b)
		{
			pMatch->vpSObject = SLineArray[i_];
			pMatch->vpMObject = MLineArray[m_SMatchArray[i__].iMFeature];
			pMatch->cost = m_SMatchArray[i__].P;
			pMatch->Type = RVLPSULM_MATCH_TYPE_LINE;

			SMMatchList.Add(pMatch);

			pMatch++;
		}
	}

#ifdef RVLPSULMBUILDER_CONDITIONAL_PROBABILITY_TREE_DEBUG_LOG
	FILE *fp = fopen("C:\\RVL\\Debug\\ConditionalProbabilityTree.txt", "w");

	if (m_DebugData.MatchArray)
		delete[] m_DebugData.MatchArray;

	m_DebugData.MatchArray = new RVLPSULM_MATCH2[nSFeatures];

	RVLPSULM_MATCH2 *pDebugDataMatch = m_DebugData.MatchArray;
#endif

	int nSMMatches = pMatch - SMMatchMem;

	RVLPSULM_MATCH2 **SMMatchArray = new RVLPSULM_MATCH2 *[nSMMatches + 1];

	RVLBubbleSort<RVLPSULM_MATCH2>(&SMMatchList, SMMatchArray, TRUE);

	RVLPSULM_MATCH2 NullMatch;

	NullMatch.cost = 0.0;

	SMMatchArray[nSMMatches] = &NullMatch;

	RVLPSULM_MATCH2 **AutoMatchArray;
	int nAutoMatches;

	if(AutoMatchArray_)
	{
		AutoMatchArray = AutoMatchArray_;
		nAutoMatches = nAutoMatches_;
	}
	else
	{
		AutoMatchArray = m_AutoMatchArray;
		nAutoMatches = m_nAutoMatches;
	}

	AutoMatchArray[nAutoMatches] = &NullMatch;

	void **NodeMem = new void *[2 * nSFeatures + 1];

	memset(NodeMem, 0, (2 * nSFeatures + 1) * sizeof(void *));

	void **pNewNode = NodeMem + nSFeatures + 1;

	int k = 0;
	int k_ = 0;

	double P = 0.0;

	int j_;
	void **pNode, **pNode_;

	while(k < nAutoMatches || k_ < nSMMatches)
	{
		if(SMMatchArray[k_]->cost >= AutoMatchArray[k]->cost)
		{
			pMatch = SMMatchArray[k_];

			k_++;	
		}
		else
		{
			pMatch = AutoMatchArray[k];

			k++;
		}

		if(pMatch->Type & RVLPSULM_MATCH_TYPE_LINE)
		{
			i_ = ((CRVL3DLine2 *)(pMatch->vpSObject))->m_Index + nSSurfs;
			
			j_ = (pMatch->Type & RVLPSULM_MATCH_TYPE_AUTO ? ((CRVL3DLine2 *)(pMatch->vpMObject))->m_Index + nSSurfs : -1);
		}
		else
		{
			i_ = ((CRVL3DSurface2 *)(pMatch->vpSObject))->m_Index;
			
			j_ = (pMatch->Type & RVLPSULM_MATCH_TYPE_AUTO ? ((CRVL3DSurface2 *)(pMatch->vpMObject))->m_Index : -1);
		}

		pNode = NodeMem + i_;

		while(*pNode)
			pNode = (void **)(*pNode);

		pNode_ = (j_ >= 0 ? NodeMem + j_ : NodeMem + nSFeatures);

		while(*pNode_)
			pNode_ = (void **)(*pNode_);

		if(pNode != pNode_)
		{
			*pNode = *pNode_ = pNewNode++;

			P += pMatch->cost;

#ifdef RVLPSULMBUILDER_CONDITIONAL_PROBABILITY_TREE_DEBUG_LOG
			char feature;
			int j;

			if(pMatch->Type & RVLPSULM_MATCH_TYPE_LINE)
			{
				feature = 'L';
				i = i_ - nSSurfs;
				j = (j_ >= 0 ? j_ - nSSurfs : ((CRVL3DLine2 *)(pMatch->vpMObject))->m_Index);
			}
			else
			{
				feature = 'S';
				i = i_;
				j = (j_ >= 0 ? j_ : ((CRVL3DSurface2 *)(pMatch->vpMObject))->m_Index);
			}

			char source = (pMatch->Type & RVLPSULM_MATCH_TYPE_AUTO ? 'S' : 'M');

			fprintf(fp, "p(S%c%d|%c%c%d)=%lf\n", feature, i, source, feature, j, pMatch->cost);

			*(pDebugDataMatch++) = *pMatch;
#endif
		}
	}

#ifdef RVLPSULMBUILDER_CONDITIONAL_PROBABILITY_TREE_DEBUG_LOG
	fclose(fp);

	m_DebugData.nMatches = pDebugDataMatch - m_DebugData.MatchArray;
#endif

	delete[] NodeMem;
	delete[] SMMatchMem;
	delete[] SMMatchArray;

	return P;
}

void CRVLPSuLMBuilder::InitHypothesisEvaluation4(CRVLPSuLM * pSPSuLM)
{
	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;
	int nSSurfs = pSPSuLM->m_n3DSurfaces;

	CRVL3DSurface2 **pSurfArrayEnd = SSurfArray + nSSurfs;

	double *Cp = m_SurfaceMatchData.Cp;
	double *invCp = m_SurfaceMatchData.invCp;

	double Mx3x3Tmp[3*3];
	double *Mx3x3TmpRow0 = Mx3x3Tmp;
	double *Mx3x3TmpRow1 = Mx3x3Tmp + 3;
	double *Mx3x3TmpRow2 = Mx3x3Tmp + 6;

	double RSR[3*3];
	double *XRS = RSR;
	double *YRS = RSR + 3;
	double *ZRS = RSR + 6;

	double CPositionUncert[3*3];
	double varPositionUncert = m_SampleMatchDistTol * m_SampleMatchDistTol;
	RVLDIAGMX3(varPositionUncert, varPositionUncert, varPositionUncert, CPositionUncert)
	
	CRVL3DSurface2 **ppSurf;
	CRVL3DSurface2 *pSurf;
	double varx, vary, varz;
	double detC;
	double *RFS, *tFS;
	double r;
	double Vect3Tmp[3];
	double fTmp;
	double varOrientUncert;
	double COrientUncert[3*3];
	int i, j, k;

	for(ppSurf = SSurfArray; ppSurf < pSurfArrayEnd; ppSurf++, Cp += 9, invCp += 9)
	{
		pSurf = *ppSurf;

		RFS = pSurf->m_Pose.m_Rot;
		tFS = pSurf->m_Pose.m_X;

		varx = pSurf->m_EigenValues[0] * pSurf->m_EigenValues[0];
		vary = pSurf->m_EigenValues[1] * pSurf->m_EigenValues[1];
		varz = pSurf->m_varq[2];

		RVLSCALECOL3(RFS, 0, varx, Mx3x3Tmp)
		RVLSCALECOL3(RFS, 1, vary, Mx3x3Tmp)
		RVLSCALECOL3(RFS, 2, varz, Mx3x3Tmp)

		RVLMXMUL3X3T2(Mx3x3Tmp, RFS, Cp)

		r = sqrt(RVLDOTPRODUCT3(tFS, tFS));

		varOrientUncert = r * m_SampleMatchAngleTol;
		varOrientUncert *= varOrientUncert;

		RVLSCALE3VECTOR2(tFS, r, ZRS);

		//RVLORTHOGONAL3(ZRS, XRS, i, j, k, Vect3Tmp, fTmp)
		RVLORTHOGONAL3(ZRS, XRS, i, j, k, fTmp);

		RVLCROSSPRODUCT3(ZRS, XRS, YRS);

		RVLSCALE3VECTOR(XRS, varOrientUncert, Mx3x3TmpRow0);
		RVLSCALE3VECTOR(YRS, varOrientUncert, Mx3x3TmpRow1);
		RVLNULL3VECTOR(Mx3x3TmpRow2);
		RVLMXMUL3X3T1(Mx3x3Tmp, RSR, COrientUncert);

		double J[9];

		RVLSKEW(tFS, J);
		RVLSCALEMX3X3(J, m_SampleMatchAngleTol, J);

		double A[9];
		RVLMXMUL3X3T2(J, J, A);
	
		RVLSUMMX3X3(Cp, COrientUncert, Cp)
		RVLSUMMX3X3(Cp, CPositionUncert, Cp)

		RVLINVCOV3(Cp, invCp, detC)
	}

	CreateAutoMatchMatrix(pSPSuLM);
}

void CRVLPSuLMBuilder::UpdateMatchMatrixForDisplay(CRVLPSuLM *pSPSuLM, 
												   RVLPSULM_HYPOTHESIS *pHypothesis)
{
	CRVLPSuLM *pMPSuLM = pHypothesis->pMPSuLM;

	if((m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) ==
		RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
	{
		BYTE *pMatch = m_MatchMatrix;

		int i, j;
		CRVL3DSurface2 *pS3DSurface, *pM3DSurface;

		for(i = 0; i < pSPSuLM->m_n3DSurfacesTotal; i++)
		{
			pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];

			for(j = 0; j < pMPSuLM->m_n3DSurfacesTotal; j++, pMatch++)
			{
				pM3DSurface = pMPSuLM->m_3DSurfaceArray[j];

				if((pS3DSurface->m_Flags | pM3DSurface->m_Flags) & RVL3DSURFACE_FLAG_REMOVED)
					*pMatch = 0;
			}
		}
	}


}

// This function is obsolete! It is not used in RVL.

void CRVLPSuLMBuilder::Load(char * ModelFileNameIn, int maxIndex)
{
	char *ModelFileName = RVLCreateString(ModelFileNameIn);

	int ModelFileNameLen = strlen(ModelFileName);

	char *pModelIndex = ModelFileName + ModelFileNameLen - 11;

	char *ImageFileName = new char[ModelFileNameLen + 2];

	strcpy(ImageFileName, ModelFileName);

	char *pImageIndex = ImageFileName + ModelFileNameLen - 11;

	int i;
	FILE *fp;
	CRVLPSuLM *pPSuLM;

	for(i = 0; i < maxIndex; i++)
	{
		sprintf(pModelIndex, "%05d-M.dat", i);

		sprintf(pImageIndex, "%05d-LW.bmp", i);

		fp = fopen(ModelFileName, "rb");

		if(fp)
		{
			pPSuLM = Load(fp);

			fclose(fp);

			//pPSuLM->m_Index = (WORD)i;
			//Index is defined before adding to m_PSuLMList in Load() function

			RVLCopyString(ImageFileName, &(pPSuLM->m_FileName));
		}
	}

	delete[] ModelFileName;

	int nModels =  m_PSuLMList.m_nElements;

	if(nModels == 0)
		nModels = 1;
}



void CRVLPSuLMBuilder::LoadMap()
{
	CRVLGUI *pGUI;
	CRVLMem GUIMem;

	if (m_Flags & RVLPSULMBUILDER_FLAG_GENERATE_MODELS)
	{
		GUIMem.Create(1000000);

		pGUI = new CRVLGUI;

		pGUI->m_pMem0 = &GUIMem;
		pGUI->m_pMem = &GUIMem;

		pGUI->Init();
	}

	m_maxPSuLMIndex = -1;

	FILE *fp;
	//check if file exists	
	fp = fopen(m_ModelMapPath, "rb");

	m_maxnModel3DLines = 0;
	m_maxnModel3DSurfaces = 0;

	if(fp)
	{
		fclose(fp);

		CRVLPSuLM *pPSuLM;

		RVLQLIST *pMap = LoadXMLMap(m_ModelMapPath, m_pMem0);

		RVLQLIST_PTR_ENTRY *pEntry = (RVLQLIST_PTR_ENTRY*)pMap->pFirst;

		if(m_Flags & RVLPSULMBUILDER_FLAG_GENERATE_MODELS)
		{
			if(m_pCamera->m_Image.pPix)
				delete[] m_pCamera->m_Image.pPix;

			m_pCamera->m_Image.pPix = new unsigned char[m_pCamera->m_Image.Width * m_pCamera->m_Image.Height];
		}

		BOOL bTmp;
		double fTmp;

		while(pEntry)
		{
			pPSuLM = (CRVLPSuLM*)pEntry->Ptr;		

			if((int)(pPSuLM->m_Index) > m_maxPSuLMIndex)
				m_maxPSuLMIndex = (int)(pPSuLM->m_Index);

			//if(pPSuLM->m_Index < 22)
			//{
			//	pEntry = (RVLQLIST_PTR_ENTRY*)pEntry->pNext;

			//	continue;
			//}

			pPSuLM->m_SurfaceList.m_pMem = m_pMem0;
			pPSuLM->m_2DLineList.m_pMem = m_pMem0;
			pPSuLM->m_3DLineList.m_pMem = m_pMem0;

			pPSuLM->m_CellArray = (RVLPSULM_CELL *)(m_pMem0->Alloc(m_nCells * sizeof(RVLPSULM_CELL)));

			pPSuLM->m_PoseRTAs.m_C = pPSuLM->m_CRTAs;
			pPSuLM->m_PoseCsInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
			pPSuLM->m_PoseCsInit.m_C = pPSuLM->m_CCsInit;
		
			if(m_Flags & RVLPSULMBUILDER_FLAG_GENERATE_MODELS)
			{
				char str[200];

				sprintf(str, "Generating model %d", pPSuLM->m_Index);

				pGUI->Message(str, 400, 100, cvScalar(0, 128, 255), false);

				//fp = fopen(pPSuLM->m_ModelFilePath, "rb");

				//if(fp)
				//{
				//	fread(&bTmp, sizeof(BOOL), 1, fp);
				//	fread(&fTmp, sizeof(double), 1, fp);
				//	fread(&(pPSuLM->m_iSubMap), sizeof(WORD), 1, fp);

				//	fclose(fp);
				//}
				//else
					pPSuLM->m_iSubMap = 0;

				m_ImageFileName = RVLCreateString(pPSuLM->m_FileName);

				Create(RVLPSULMBUILDER_CREATEMODEL_FROM_IMAGE | RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE, pPSuLM);

				delete[] m_ImageFileName;

				//Create(pPSuLM, m_pMem, RVLPSULMBUILDER_CREATEMODEL_FROM_IMAGE);

				fp = fopen(pPSuLM->m_ModelFilePath, "wb");

				pPSuLM->Save(fp);

				fclose(fp);

				m_pMCMem->Clear();
			}

			pPSuLM->Load(m_Flags2);

			if(pPSuLM->m_nLandmarks > m_maxnLandmarks)
				m_maxnLandmarks = pPSuLM->m_nLandmarks;

			if(pPSuLM->m_n3DLines > m_maxnLines)
				m_maxnLines = pPSuLM->m_n3DLines;

			if(pPSuLM->m_n3DSurfacesTotal > m_maxnModel3DSurfaces)
				m_maxnModel3DSurfaces = pPSuLM->m_n3DSurfacesTotal;

			if(pPSuLM->m_n3DLinesTotal > m_maxnModel3DLines)
				m_maxnModel3DLines = pPSuLM->m_n3DLinesTotal;

			//if ((m_Flags2 & RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT) == RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG)
			//{
			//	FREIBURG_DATASET_SAMPLE	Sample;

			//	m_DataSet.GetSample(pPSuLM->m_FileName, &Sample);

			//	RVLCOPY3VECTOR(Sample.t, pPSuLM->m_PoseAbs->m_X);
			//	RVLCOPYMX3X3(Sample.R, pPSuLM->m_PoseAbs->m_Rot);
			//}

			m_PSuLMList.Add(pPSuLM);
			
			pEntry = (RVLQLIST_PTR_ENTRY*)pEntry->pNext;
		}

		if (m_Flags & RVLPSULMBUILDER_FLAG_GENERATE_MODELS)
		{
			pGUI->Message("All models generated.", 400, 100, cvScalar(0, 255, 0));

			delete pGUI;
		}

		if(m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_MODEL_FUSION)
		{
			m_ModelFusion.RM_S = new double[9 * (m_maxPSuLMIndex + 1)];
			m_ModelFusion.tM_S = new double[3 * (m_maxPSuLMIndex + 1)];
		}
		else
		{
			m_ModelFusion.RM_S = NULL;
			m_ModelFusion.tM_S = NULL;

			if(m_SurfaceMatchData.Cp_)
				delete[] m_SurfaceMatchData.Cp_;

			m_SurfaceMatchData.Cp_ = new double[3 * 3 * m_maxnModel3DSurfaces];

			if(m_SurfaceMatchData.invCp_)
				delete[] m_SurfaceMatchData.invCp_;

			m_SurfaceMatchData.invCp_ = new double[3 * 3 * m_maxnModel3DSurfaces];

			if(m_SurfaceMSArray)
				delete[] m_SurfaceMSArray;

			m_SurfaceMSArray = new CRVL3DSurface2[m_maxnModel3DSurfaces];
		}

		if(m_Flags & RVLPSULMBUILDER_FLAG_GENERATE_MODELS)
		{
			delete[] m_pCamera->m_Image.pPix;

			m_pCamera->m_Image.pPix = NULL;
		}

		if(m_PSuLMArray)
			delete[] m_PSuLMArray;

		m_PSuLMArray = new CRVLPSuLM *[m_maxPSuLMIndex + 1];

		memset(m_PSuLMArray, 0, sizeof(CRVLPSuLM *) * (m_maxPSuLMIndex + 1));

		//CRVLPSuLM **ppPSuLM = PSuLMPtr;

		pEntry = (RVLQLIST_PTR_ENTRY*)pMap->pFirst;

		while(pEntry)
		{
			pPSuLM = (CRVLPSuLM*)pEntry->Ptr;

			//*(ppPSuLM++) = pPSuLM;

			m_PSuLMArray[pPSuLM->m_Index] = pPSuLM;

			pEntry = (RVLQLIST_PTR_ENTRY*)pEntry->pNext;
		}

		pEntry = (RVLQLIST_PTR_ENTRY*)pMap->pFirst;

		RVLQLIST *pNeighbourList;
		RVLQLIST_PTR_ENTRY *pNeighborPtr;
		RVLPSULM_NEIGHBOUR *pNeighbor;


		while(pEntry)
		{
			pPSuLM = (CRVLPSuLM*)pEntry->Ptr;

			pNeighbourList = pPSuLM->m_NeighbourList;

			pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighbourList->pFirst);

			while(pNeighborPtr)
			{
				pNeighbor = (RVLPSULM_NEIGHBOUR *)(pNeighborPtr->Ptr);

				pNeighbor->pPSuLM = m_PSuLMArray[pNeighbor->ModelId];

				pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);
			}

			pEntry = (RVLQLIST_PTR_ENTRY*)pEntry->pNext;
		}

		char *LocalMapsFileName = RVLCreateFileName(m_ModelMapPath, ".xml", -1, "-LM.dat");

		if(!LoadLocalMaps(LocalMapsFileName))
		{
			CreateLocalMaps();

			SaveLocalMaps(LocalMapsFileName);
		}

		delete[] LocalMapsFileName;

		// load local model names

		char *ModelNamesFileName = RVLCreateFileName(m_ModelMapPath, ".xml", -1, "-N.txt");

		FILE *fpModelNames = fopen(ModelNamesFileName, "r");

		if(fpModelNames)
		{
			char line[200];
			char ModelName[200];

			int iModel;

			while(!feof(fp))
			{
				fgets(line, 200, fp);

				sscanf(line, "%d\t%s", &iModel, ModelName);

				strcpy(m_PSuLMArray[iModel]->m_Name, ModelName);
			}

			fclose(fpModelNames);

			delete[] ModelNamesFileName;
		}
	}
}

RVLPSULM_HYPOTHESIS * CRVLPSuLMBuilder::GetHypothesis(int index)
{
	RVLPSULM_HYPOTHESIS *pHypothesis = NULL;

	if(m_HypothesisList.m_nElements > 0)
	{
		m_HypothesisList.Start();

		int i;				

		for(i = 0; i <= index; i++)
			pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());
	}

	return pHypothesis;
}

void CRVLPSuLMBuilder::CreateParamList(CRVLMem * pMem)
{
	m_ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	m_ParamList.Init();

	pParamData = m_ParamList.AddParam("PSuLM.ImageFormat", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "FREIBURG", RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG);
	pParamData = m_ParamList.AddParam("PSuLM.Mode", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "TRACKING", RVLPSULMBUILDER_FLAG_MODE_TRACKING);
	m_ParamList.AddID(pParamData, "LOCALIZATION", RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION);
	pParamData = m_ParamList.AddParam("PSuLM.Features", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "SURFACES", RVLPSULMBUILDER_FLAG_SURFACES);
	m_ParamList.AddID(pParamData, "LINES", RVLPSULMBUILDER_FLAG_LINES);
	pParamData = m_ParamList.AddParam("PSuLM.Surfaces.Boundary", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_SURFACE_BOUNDARY);
	pParamData = m_ParamList.AddParam("PSuLM.Lines.EdgesVoid", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_LINES_EDGES_VOID);
	pParamData = m_ParamList.AddParam("PSuLM.Complex", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_COMPLEX);
	pParamData = m_ParamList.AddParam("PSuLM.FileVersion", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "2", RVLPSULMBUILDER_FLAG2_FILE_VERSION_2);
	pParamData = m_ParamList.AddParam("PSuLM.Global", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_GLOBAL);
	pParamData = m_ParamList.AddParam("PSuLM.Material", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_MATERIAL);
	pParamData = m_ParamList.AddParam("PSuLM.maxnDominant3DSurfaces", RVLPARAM_TYPE_INT, &m_maxnDominant3DSurfaces);
	pParamData = m_ParamList.AddParam("PSuLM.maxnDominant3DLines", RVLPARAM_TYPE_INT, &m_maxnDominant3DLines);
	//pParamData = m_ParamList.AddParam("PSuLM.Lines.minBoundingBoxSize", RVLPARAM_TYPE_INT, &m_minLineBoundingBoxSize);
	pParamData = m_ParamList.AddParam("PSuLM.Lines.minContourSize", RVLPARAM_TYPE_INT, &m_minContourSize);
	pParamData = m_ParamList.AddParam("PSuLM.Lines.minDepthStep", RVLPARAM_TYPE_INT, &m_minLineDepthStep);
	pParamData = m_ParamList.AddParam("PSuLM.Lines.minnDepthSteps", RVLPARAM_TYPE_INT, &m_minnLineDepthSteps);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.PoseRefinement", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_LOCALIZATION_POSE_REFINEMENT);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.Display", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_LOCALIZATION_DISPLAY);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.BidirectionalHypothesisEvaluation", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_LOCALIZATION_BIDIRECT_HYP_EVAL);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.Odometry", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_LOCALIZATION_ODOMETRY);

	pParamData = m_ParamList.AddParam("PSuLM.ModelDatabasePath", RVLPARAM_TYPE_STRING, &m_ModelDatabasePath);
	pParamData = m_ParamList.AddParam("PSuLM.ModelMapPath", RVLPARAM_TYPE_STRING, &m_ModelMapPath);
	pParamData = m_ParamList.AddParam("PSuLM.Sequence.ScenePath", RVLPARAM_TYPE_STRING, &m_SequenceScenePath);
	pParamData = m_ParamList.AddParam("PSuLM.DataSetPath", RVLPARAM_TYPE_STRING, &m_DataSetPath);

	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.Indexing", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_HYPOTHESIS_GENERATION_INDEXING);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.maxnExpandedNodes", RVLPARAM_TYPE_INT, &m_maxnExpandedNodes);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.refnHypotheses", RVLPARAM_TYPE_INT, &m_refnHypotheses);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.RotHypTol", RVLPARAM_TYPE_DOUBLE, &m_RotHypTol);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.tHypTol", RVLPARAM_TYPE_DOUBLE, &m_tHypTol);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.InitMatchingConstraints", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_HYPGEN_INIT_MATCHING_CONSTRAINTS);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.UnconstrainedOrientation", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_UNCONSTRAINED_ORIENTATION);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.WideAngle", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_WIDE_ANGLE_HYPOTHESIS_GENERATION);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.LastDOFEstimationMethod", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "MAX_PEAK_ONLY", RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_MAX_PEAK_ONLY);
	m_ParamList.AddID(pParamData, "BEST_PEAK_TREE", RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_BEST_PEAK_TREE);
	m_ParamList.AddID(pParamData, "ALL_PEAKS", RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_ALL_PEAKS);	
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.maxLastDOFTravelDist", RVLPARAM_TYPE_DOUBLE, &m_maxLastDOFTravelDist);

	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.Method", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "SM", RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SM);
	m_ParamList.AddID(pParamData, "IBM", RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM);
	m_ParamList.AddID(pParamData, "SSM", RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM);
	m_ParamList.AddID(pParamData, "P", RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.FirstOrderDependencyTree", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_FIRST_ORDER_DEPENDENCY_TREE);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.CellSize2", RVLPARAM_TYPE_INT, &m_CellSize2);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.nCellsPer45deg", RVLPARAM_TYPE_INT, &m_nCellsPer45deg);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.SampleMatchAngleStD", RVLPARAM_TYPE_DOUBLE, &m_SampleMatchAngleStD);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.SampleMatchDistStD", RVLPARAM_TYPE_DOUBLE, &m_SampleMatchDistStD);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.SampleMatchUncertCoef", RVLPARAM_TYPE_DOUBLE, &m_SampleMatchUncertCoef);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.PlausibilityThr", RVLPARAM_TYPE_INT, &m_PlausibilityThr);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.minSurfaceSamplesForMatch", RVLPARAM_TYPE_INT, &m_minSurfaceSamplesForMatch);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.SSM.Norm", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_SSM_NORM);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.ModelFusion", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_MODEL_FUSION);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.SampleMatching", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisEvaluation.Line.PDFPriorPosition", RVLPARAM_TYPE_DOUBLE, &(m_LineMatchData.PPriorPosition));

	pParamData = m_ParamList.AddParam("PSuLM.Localization.Odometry.Const1", RVLPARAM_TYPE_DOUBLE, m_OdometryUncertConst);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.Odometry.Const2", RVLPARAM_TYPE_DOUBLE, m_OdometryUncertConst + 1);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.Odometry.Const3", RVLPARAM_TYPE_DOUBLE, m_OdometryUncertConst + 2);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.Odometry.Const4", RVLPARAM_TYPE_DOUBLE, m_OdometryUncertConst + 3);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.Floor.Const", RVLPARAM_TYPE_DOUBLE, &m_FloorUncertConst);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.Robot.A", RVLPARAM_TYPE_DOUBLE, m_RobotParams);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.Robot.B", RVLPARAM_TYPE_DOUBLE, m_RobotParams + 1);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.MinDistBetweenLocalModels", RVLPARAM_TYPE_DOUBLE, &m_MinHybridLocalizationDist);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.MinAngleBetweenLocalModels", RVLPARAM_TYPE_DOUBLE, &m_MinHybridLocalizationAngle);
	pParamData = m_ParamList.AddParam("PSuLM.LocalMapRadius", RVLPARAM_TYPE_DOUBLE, &m_LocalMapRadius);

	pParamData = m_ParamList.AddParam("PSuLM.Global.Threshold.Distance", RVLPARAM_TYPE_DOUBLE, &m_GlobalLocalizationDistTHR);
	pParamData = m_ParamList.AddParam("PSuLM.Global.Threshold.Angle", RVLPARAM_TYPE_DOUBLE, &m_GlobalLocalizationAngleTHR);
	
	pParamData = m_ParamList.AddParam("PSuLM.UncertCoeff", RVLPARAM_TYPE_DOUBLE, &(m_M3DSurfaceSet.m_UncertCoeff));

	pParamData = m_ParamList.AddParam("PSuLM.Hypothesis.3DOFUncertainty", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF);

	pParamData = m_ParamList.AddParam("PSuLM.Hypothesis.EKFConstraint", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_HYPOTHESES_EKF_CONSTRAINT);

	pParamData = m_ParamList.AddParam("PSuLM.MapBuilding", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_MAPBUILDING);
	pParamData = m_ParamList.AddParam("PSuLM.MapBuilding.Manual", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG2_MAPBUILDING_MANUAL);

	pParamData = m_ParamList.AddParam("PSuLM.GenerateModels", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_GENERATE_MODELS);

	pParamData = m_ParamList.AddParam("PSuLM.Localization.GroundTruthExists", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_GROUNDTRUTH_EXISTS);

	pParamData = m_ParamList.AddParam("PSuLM.Localization.SceneFusion", RVLPARAM_TYPE_FLAG, &m_Flags2);
	m_ParamList.AddID(pParamData, "AROUND", RVLPSULMBUILDER_FLAG2_SCENE_FUSION_LOOK_AROUND);
	m_ParamList.AddID(pParamData, "MOVE", RVLPSULMBUILDER_FLAG2_SCENE_FUSION_MOVE);

	pParamData = m_ParamList.AddParam("PSuLM.Localization.ParticleFilter", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_PARTICLE_FILTER);

	pParamData = m_ParamList.AddParam("PSuLM.Localization.ParticleFilter.refnParticles", RVLPARAM_TYPE_INT, &m_refnParticles);	

	pParamData = m_ParamList.AddParam("PSuLM.IncrementalLocalization.GTPath", RVLPARAM_TYPE_STRING, &m_IncrementalLocalizationResultsFilePath);
	
	pParamData = m_ParamList.AddParam("PSuLM.EvaluationBenchmark", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_FLAG_EVALUATION_BENCHMARK);
	
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.maxZ", RVLPARAM_TYPE_DOUBLE, &m_maxZ);
	pParamData = m_ParamList.AddParam("PSuLM.Localization.HypothesisGeneration.minnHypSurfPts", RVLPARAM_TYPE_INT, &m_minnHypSurfPts);

	pParamData = m_ParamList.AddParam("PSuLM.Debug.GetLocalModelsLog", RVLPARAM_TYPE_FLAG, &m_DebugFlags);
	m_ParamList.AddID(pParamData, "yes", RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG);

	pParamData = m_ParamList.AddParam("PSuLM.kPan", RVLPARAM_TYPE_DOUBLE, &m_kPan);
	pParamData = m_ParamList.AddParam("PSuLM.kTilt", RVLPARAM_TYPE_DOUBLE, &m_kTilt);
	pParamData = m_ParamList.AddParam("PSuLM.TiltOffset[deg]", RVLPARAM_TYPE_DOUBLE, &m_TiltOffset);
}

void CRVLPSuLMBuilder::PythonDisplayScene(RVLSURFACE_MATCH_ARRAY *MatchArray, CRVLMPtrChain *pM3DSurfaceList, CRVL3DSurface2 **MatrixSceneModel, int n3DSceneSurfaces)
{

//	int i,j;
//	
//	CRVL3DSurface2 *pS3DSurface, *pM3DSurface, *p3DConvexSegment;
//	CRVL3DSurface2 **pp3DConvexSegment, **pp3DConvexSegmentEnd;
//	
//	RVLARRAY *pRelListSurface;
//
//	CRVL3DContour *p3DContour;
//		
//	
//	int nConvexSegments;
//	int maxNoPlanes = 10000;
//	int maxNoPlanes2 = 2 * maxNoPlanes;
//	int maxNoPts = 320*240;
//
//	int *VecL2Planes = new int[maxNoPlanes];
//	int *VecNumberofPts = new int[maxNoPlanes2];
//	double *VecPtArray = new double[maxNoPts];
//
//	memset(VecL2Planes, 0, maxNoPlanes * sizeof(int) );
//	memset(VecNumberofPts, 0, maxNoPlanes2 * sizeof(int) );
//	memset(VecPtArray, 0, maxNoPts * sizeof(double) );
//
//
//	int *pVecNumberofPts;
//	pVecNumberofPts = VecNumberofPts;
//
//	double *pVecPtArray;
//	pVecPtArray = VecPtArray;
//
//	int nTotalConvexSegments = 0;
//	int nTotalPts = 0;
//
//	//create and Store Scene
//	for(i=0;i<n3DSceneSurfaces;i++)
//	{
//		pS3DSurface = (CRVL3DSurface2 *)(MatchArray[i].pSObject);
//		
//		pRelListSurface = pS3DSurface->m_RelList + pS3DSurface->m_pClass->m_iRelListComponents;
//		pp3DConvexSegment = (CRVL3DSurface2 **)(pRelListSurface->pFirst);
//		pp3DConvexSegmentEnd = (CRVL3DSurface2 **)(pRelListSurface->pEnd);
//
//		//nConvexSegments = pp3DConvexSegmentEnd - pp3DConvexSegment;
//
//		nConvexSegments = 0;
//		
//		
//		//for each 3D convex segment
//		for(; pp3DConvexSegment < pp3DConvexSegmentEnd; pp3DConvexSegment++)
//		{
//			p3DConvexSegment = *pp3DConvexSegment;
//
//			if(p3DConvexSegment->m_Flags & RVLOBJ2_FLAG_REJECTED)
//				continue;
//
//			nConvexSegments +=1;
//			
//			//Get corresponding 3D contour
//			p3DContour = *(CRVL3DContour **)(p3DConvexSegment->m_pData + m_S3DConvexSegmentSet.m_iDataContourPtr);
//			//pp2DContour = (CRVL2DContour **)(p3DConvexSurface->m_pData + m_S3DConvexSegmentSet.m_iDataContourPtr);
//			
//			//store number of pts
//			*(pVecNumberofPts++) = p3DContour->m_nPts;
//			nTotalPts += p3DContour->m_nPts;
//
//			//store contour vertices
//			memcpy(pVecPtArray,p3DContour->m_PtArray, 3 * p3DContour->m_nPts * sizeof(double));
//			pVecPtArray += 3 * p3DContour->m_nPts;
//
//		
//		}
//		//Store nConvexSegments
//		VecL2Planes[i] = nConvexSegments;
//		nTotalConvexSegments += nConvexSegments;
//		
//	}
//
//
//	//Save file and run python using console
//	CString RezFile = "D:\\Phd\\Program\\PythonScripts\\Scene.dat";
//	FILE *fRezFile = fopen(RezFile, "w");
//	if(fRezFile)
//	{
//		//store no of LEVEL3 surfaces
//		fprintf(fRezFile,"%4d\n",n3DSceneSurfaces);
//
//		//store array of size n3DSceneSurfaces containing the number of LEVEL2 convex segments for each LEVEL3 surface
//		for(i=0;i<n3DSceneSurfaces;i++)
//			fprintf(fRezFile,"%4d\n",VecL2Planes[i]);
//
//
//		//store no convex segments
//		fprintf(fRezFile,"%4d\n",nTotalConvexSegments);
//
//		//store array of size nTotalConvexSegments containing the number of contour points for LEVEL2 convex segments
//		for(i=0;i<nTotalConvexSegments;i++)
//			fprintf(fRezFile,"%4d\n",VecNumberofPts[i]);
//
//
//		//store total no of convex polygon vertices
//		fprintf(fRezFile,"%4d\n",nTotalPts);
//
//		//store array of size nTotalPts containing the contour points for LEVEL2 convex segments
//		for(i=0;i<nTotalPts*3;i+=3)
//			fprintf(fRezFile,"%6.3lf\t%6.3lf\t%6.3lf\n",VecPtArray[i],VecPtArray[i+1],VecPtArray[i+2]);
//
//		fclose(fRezFile);
//	}
//
//
//
//	//Reset values
//	memset(VecL2Planes, 0, maxNoPlanes * sizeof(int) );
//	memset(VecNumberofPts, 0, maxNoPlanes2 * sizeof(int) );
//	memset(VecPtArray, 0, maxNoPts * sizeof(double) );
//
//	pVecNumberofPts = VecNumberofPts;
//	pVecPtArray = VecPtArray;
//
//	nTotalConvexSegments = 0;
//	nTotalPts = 0;
//	int n3DModeSurfaces = pM3DSurfaceList->m_nElements;
//
//	//create and Store Model
//	i=0;
//	pM3DSurfaceList->Start();
//	while(pM3DSurfaceList->m_pNext)
//	{
//		pM3DSurface = (CRVL3DSurface2 *)(pM3DSurfaceList->GetNext());
//
//		pRelListSurface = pM3DSurface->m_RelList + pM3DSurface->m_pClass->m_iRelListComponents;
//		pp3DConvexSegment = (CRVL3DSurface2 **)(pRelListSurface->pFirst);
//		pp3DConvexSegmentEnd = (CRVL3DSurface2 **)(pRelListSurface->pEnd);
//
//		nConvexSegments = pp3DConvexSegmentEnd - pp3DConvexSegment;
//
//		
//		//for each 3D convex segment
//		for(; pp3DConvexSegment < pp3DConvexSegmentEnd; pp3DConvexSegment++)
//		{
//			p3DConvexSegment = *pp3DConvexSegment;
//
//			//Get corresponding 3D contour
//			p3DContour = *(CRVL3DContour **)(p3DConvexSegment->m_pData + m_S3DConvexSegmentSet.m_iDataContourPtr);
//			//pp2DContour = (CRVL2DContour **)(p3DConvexSurface->m_pData + m_S3DConvexSegmentSet.m_iDataContourPtr);
//			
//			//store number of pts
//			*(pVecNumberofPts++) = p3DContour->m_nPts;
//			nTotalPts += p3DContour->m_nPts;
//
//			//store contour vertices
//			memcpy(pVecPtArray,p3DContour->m_PtArray, 3 * p3DContour->m_nPts * sizeof(double));
//			pVecPtArray += 3 * p3DContour->m_nPts;
//
//		
//		}
//		//Store nConvexSegments
//		VecL2Planes[i] = nConvexSegments;
//		nTotalConvexSegments += nConvexSegments;
//		i++;
//	}
//
//
//	//Save file and run python using console
//	RezFile = "D:\\Phd\\Program\\PythonScripts\\Model.dat";
//	fRezFile = fopen(RezFile, "w");
//	if(fRezFile)
//	{
//		//store no of LEVEL3 surfaces
//		fprintf(fRezFile,"%4d\n",n3DModeSurfaces);
//
//		//store array of size n3DSceneSurfaces containing the number of LEVEL2 convex segments for each LEVEL3 surface
//		for(i=0;i<n3DModeSurfaces;i++)
//			fprintf(fRezFile,"%4d\n",VecL2Planes[i]);
//
//
//		//store no convex segments
//		fprintf(fRezFile,"%4d\n",nTotalConvexSegments);
//
//		//store array of size nTotalConvexSegments containing the number of contour points for LEVEL2 convex segments
//		for(i=0;i<nTotalConvexSegments;i++)
//			fprintf(fRezFile,"%4d\n",VecNumberofPts[i]);
//
//
//		//store total no of convex polygon vertices
//		fprintf(fRezFile,"%4d\n",nTotalPts);
//
//		//store array of size nTotalPts containing the contour points for LEVEL2 convex segments
//		for(i=0;i<nTotalPts*3;i+=3)
//			fprintf(fRezFile,"%6.3lf\t%6.3lf\t%6.3lf\n",VecPtArray[i],VecPtArray[i+1],VecPtArray[i+2]);
//
//		fclose(fRezFile);
//	}
//
//
//	
//	//Save MatchMatrix
//	RezFile = "D:\\Phd\\Program\\PythonScripts\\MatchMatrix.dat";
//	fRezFile = fopen(RezFile, "w");
//	int m;
//	if(fRezFile)
//	{
//		//store no of LEVEL3 surfaces Scene and model
//		fprintf(fRezFile,"%4d\n",n3DSceneSurfaces);
//		fprintf(fRezFile,"%4d\n",n3DModeSurfaces);
//
//		for(i=0;i<n3DSceneSurfaces;i++)
//		{
//			for(j=0;j<n3DModeSurfaces;j++)
//			{
//				m = (MatrixSceneModel[i*n3DModeSurfaces + j] != 0 ? 1 : 0);
//				
//				fprintf(fRezFile,"%4d\t",m);
//			}
//			fprintf(fRezFile,"\n");
//		}
//
//
//		fclose(fRezFile);
//		
//	}
//
//
//
//
////#ifdef PYTHON_DEBUG
////	PyObject *pFunction;
////	PyObject *pInputVal;
////	PyObject *pNumberofL2Planes;
////	PyObject *pPlaneNumberofPts;
////	PyObject *pPlanePtArray;
////
////	npy_intp dims1[1];
////	npy_intp dims2[1];
////	npy_intp dims3[1];
////	dims1[0] = 21;
////	dims2[0] = 46;
////	dims3[0] = 412;
////
////	pNumberofL2Planes = PyArray_SimpleNewFromData(1, dims1, NPY_INT, VecSceneNumberofL2Planes);
////	pPlaneNumberofPts = PyArray_SimpleNewFromData(1, dims2, NPY_INT, VecNumberofPts);
////	pPlanePtArray = PyArray_SimpleNewFromData(1, dims3, NPY_DOUBLE, VecPtArray);
////
////	PyArrayObject *pRez;
////
////
////	if (m_pyModule != NULL)
////	{
////		pFunction = PyObject_GetAttrString(m_pyModule, "Display3DSurfaces");
////
////		pInputVal = PyTuple_New(6);
////		PyTuple_SetItem(pInputVal, 0, (PyObject *)n3DSceneSurfaces);
////		PyTuple_SetItem(pInputVal, 1, pNumberofL2Planes);
////		PyTuple_SetItem(pInputVal, 2, (PyObject *)nTotalConvexSegments);
////		PyTuple_SetItem(pInputVal, 3, pPlaneNumberofPts);
////		PyTuple_SetItem(pInputVal, 4, (PyObject *)nTotalPts);
////		PyTuple_SetItem(pInputVal, 5, pPlanePtArray);
////
////		pRez = (PyArrayObject*) PyObject_CallObject(pFunction, pInputVal);
////		
////	}
////#endif
//
//	delete[] VecL2Planes;
//	delete[] VecNumberofPts;
//	delete[] VecPtArray;


}

// The following function uses m_pMem2.

void CRVLPSuLMBuilder::Hypotheses3(	CRVLPSuLM *pSPSuLM,
									CRVL3DPose *pPoseS0Init,	// Initial uncertainty for EKF update
									double *PInit,				// Uncertainty for initial matching
									CRVLPSuLM * pPrevSPSuLM)
{
	int nHypotheses = m_refnHypotheses;

	int maxnM3DSurfaces = RVLMAX(m_maxnDominant3DSurfaces, m_maxnDominant3DSurfacesComplex);

	//int maxnSamples = 5;

	//int maxnFailures = 3;

	int maxnNodes = 100000;	

	//int QueueSize = 1000;

	//int PredictionHorizon = 3;

	//int maxnMatchesInHypothesis = 7;

	//int TraveledDistBinSize = 50;	// mm

	//double fTraveledDistBinSize = (double)TraveledDistBinSize;

	//int maxTraveledDist = 1000;	// mm

	//double fmaxTraveledDist = (double)maxTraveledDist;

	m_HypothesisList.RemoveAll();

	int nS3DSurfaces = pSPSuLM->m_n3DSurfaces;

	if(nS3DSurfaces == 0)
		return;

	int iLastSSurf = nS3DSurfaces - 1;

	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;
	CRVL3DSurface2 **ppSSurf = SSurfArray;

	//RVLPSULM_SURF_MATCH_DATA *SSurfMatchData = new RVLPSULM_SURF_MATCH_DATA[nS3DSurfaces];

	//RVLPSULM_SURF_MATCH_DATA *pSSurfMatchData = SSurfMatchData;

	CRVL3DSurface2 *pS3DSurface;

	//int SSupportTotal = 0;

	int i;

	for(i=0;i<nS3DSurfaces;i++)
	{
		pS3DSurface = SSurfArray[i];

		pS3DSurface->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

		pS3DSurface->m_Index = i;

		//SSupportTotal += pS3DSurface->m_nSupport;

		//pSSurfMatchData->p3DSurface = pS3DSurface;

		//pSSurfMatchData->bMatched = 0;

		//pSSurfMatchData++;
	}

	CRVL3DSurface2 **pSSurfArrayEnd = SSurfArray + nS3DSurfaces;

	//int QueueSize = (10 * SSupportTotal) / 10 + 1;
	int QueueSize = nS3DSurfaces + maxnM3DSurfaces + 1;

	//**********************************

	//CRVL3DPose PoseSM;
	CRVL3DPose PoseSMInit;

	PoseSMInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	double CInit[3 * 3 * 3];
	double invtInit[3];
	PoseSMInit.m_C = CInit;
	double *R, *t;
	if(pPoseS0Init)
	{
		PoseSMInit.Copy(pPoseS0Init);
		PoseSMInit.m_pData = invtInit;
		R = PoseSMInit.m_Rot;
		t = PoseSMInit.m_X;
		RVLMULMX3X3TVECT(R, t, invtInit);
	}

	CRVL3DPose PoseSM5DoF;
	double invt5DoF[3];
	PoseSM5DoF.m_pData = invt5DoF;
	PoseSM5DoF.m_C = PoseSMInit.m_C;

	//PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	//double C[3 * 3 * 3];
	//PoseSM.m_C = C;
	//PoseSM.m_pData = invt;

	CRVLPSuLM *pMPSuLM;
	CRVL3DSurface2 *pM3DSurface;
	CRVL3DSurface2 **ppMSurf;
	//CRVLMPtrChain *pM3DSurfaceList;
	CRVL3DSurface2 **MSurfArray;
	int iHypothesis;
	double MatchQuality;
	double detQ;

	RVLPSULM_MSMATCH_DATA *MatchList;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_MSMATCH_DATA, nS3DSurfaces * maxnM3DSurfaces + 1, MatchList);

	RVLPSULM_HG_NODE *NodeMem;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_HG_NODE, maxnNodes, NodeMem);

	RVLPSULM_HG_NODE *pNodeMemEnd = NodeMem + maxnNodes;

	CRVLQListArray Queue;

	Queue.m_Size = QueueSize + 1;

	Queue.Init();

	RVLQLIST *QueueListArray = Queue.m_ListArray;	

#ifdef RVLPSULM_LINES
	int nS3DLines = pSPSuLM->m_n3DLines;

	//CRVL3DLine2 *S3DLineArrayLastDOF;
	//
	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, CRVL3DLine2, nS3DLines, S3DLineArrayLastDOF);

	double RBTl[3 * 3];

	double *xTlB = RBTl + 0 * 3;
	double *yTlB = RBTl + 1 * 3;
	double *zTlB = RBTl + 2 * 3;

	double M3x3Tmp[3 * 3];

	double *M3x3TmpRow2 = M3x3Tmp + 3;
	double *M3x3TmpRow3 = M3x3Tmp + 2 * 3;

	double fnLastDOFGaussianSamples = (double)m_nLastDOFGaussianSamples;
	double kLastDOFScale = 2.5 / fnLastDOFGaussianSamples / m_LastDOFResolution;	// multiplying a std of a Gaussian by this factor
																					// gives the maximum distance between
																					// samples of this Gaussian relative to
																					// m_LastDOFResolution 
	double LastDOFLineRotationVar = m_LastDOFLineRotationStD * DEG2RAD;
	LastDOFLineRotationVar *= LastDOFLineRotationVar;
	double LastDOFLineTranslationVar = m_LastDOFLineTranslationStD * m_LastDOFLineTranslationStD;

	int nLastDOFBins = (1 << (int)(floor(log(floor(m_maxLastDOFTravelDist / m_LastDOFResolution) + 1.0) / RVLLOG2 + 1.0) + 1.0));

	//CRVL3DLine2 *M3DLineArrayLastDOF;
	int nM3DLines;
	int iS3DLine, iM3DLine;
	CRVL3DLine2 *pS3DLine, *pM3DLine;
	double *dXSA;
	RVL3DLINE_EXTENDED_DATA *pS3DLineData, *pM3DLineData;
	double V3x1Tmp[3];
	double fTmp, fTmp2;
	double *XS1A, *XS2A, *XM1, *XM2;
	double XS1B[3], XS2B[3];
	double dXSB[3];
	double *CXS1A, *CXS2A, *CXM1, *CXM2;
	double CXS1B[3 * 3], CXS2B[3 * 3];
	double XS1Tl[3], XS2Tl[3], XM1Tl[3], XM2Tl[3];
	double CXS1Tl[2 * 2], CXS2Tl[2 * 2], CXM1Tl[2 * 2], CXM2Tl[2 * 2];
	double CXS1Tl_[2 * 2], CXS2Tl_[2 * 2], CXM1Tl_[2 * 2], CXM2Tl_[2 * 2];
	double XS1Tl_[2], XS2Tl_[2], XM1Tl_[2], XM2Tl_[2];
	double VSB[3];
	double *VM;
	double kzOverlap, dzSTl, dzMTl;
	double sS1, sS2, sM1, sM2, minz2, maxz1;
	double e1[2], e2[2], C1[2 * 2], C2[2 * 2], detC1, detC2;
	double overlap;
	double SSupport;
	double maxvarTD;
	double ex1, ex2;
	double XS1A_[3], XS2A_[3];
	double *XSA_, *XMB_;
	int nLastDOFScales;
	double *LastDOFAccu, *LastDOFAccu_;
	int iLastDOFScale;
	double stdTD;
	int iLastDOFBin, iLastDOFBin1, iLastDOFBin2;
	double support;
	double LastDOFBinSize, kLastDOFScale2;
	int nLastDOFBins_;
	double *pLastDOFAccuBin, *pLastDOFAccuBelowBin, *pLastDOFAccuEnd, *pBestLastDOFAccuBin;
	double M3x3Tmp2[3 * 3];
	double CPC1[3 * 3], CPC2[3 * 3];
#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
	pSPSuLM->Display(m_DebugData.pGUI, pPoseS0Init);

	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("PSuLM");

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	pMouseCallbackData->MatrixSceneModel = NULL;	

	//pMouseCallbackData->MSurfMatchData = MSurfMatchData;
	//pMouseCallbackData->SSurfMatchData = SSurfMatchData;

	CRVLFigure *pFig2 = m_DebugData.pGUI->OpenFigure("PSuLM2");	

	pFig->m_FontSize = pFig2->m_FontSize = 16;

	int nTextLines = 22;

	int TextImageHeight = nTextLines * pFig2->m_FontSize;

	pFig2->EmptyBitmap(cvSize(800, TextImageHeight), cvScalar(255, 255, 255));

	cvInitFont(&(pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	cvInitFont(&(pFig2->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	char Text[201];

	pFig2->PutText("Hello world!", cvPoint(0, pFig2->m_FontSize), cvScalar(0, 0, 0));

	m_DebugData.pGUI->ShowFigure(pFig2);
#endif

	RVLPSULM_HYPOTHESIS **HypothesisArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_HYPOTHESIS *, nHypotheses, HypothesisArray);

	//int *TraveledDistAcc;

	//int nTraveledDistBins = 2 * maxTraveledDist / TraveledDistBinSize + 1;

	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, int, nTraveledDistBins, TraveledDistAcc);

	//int *TraveledDistBin =  TraveledDistAcc + maxTraveledDist / TraveledDistBinSize;

	//CRVLQListArray TraveledDistAccMatchPtrList;

	

	int HypothesisIndex = 0;

	double eigVal[3];
	
	//RVLMatrixHeader31->data.db = eigVal;

	double RBT[3 * 3];

	double *xTB = RBT;
	double *yTB = RBT + 3;
	double *zTB = RBT + 2 * 3;

	//RVLMatrixHeaderB33->data.db = eigVect;

	//CvMat *_eigVal = cvCreateMatHeader(3, 1, CV_64FC1);
	//CvMat *_eigVect = cvCreateMatHeader(3, 3, CV_64FC1);

	//_eigVal->data.db = eigVal;
	//_eigVect->data.db = V;

	RVLSURFACE_MATCH_ARRAY MatchData;

	double *e = MatchData.m_e;

	double RSS_[9];
	
	RVLUNITMX3(RSS_)

	double RS_M[9];

	RVLCOPYMX3X3(R, RS_M)
	
	RVLPSULM_HYPOTHESIS *pHypothesis = NULL;

	int iAlphaRange = 1;

	if ((m_Flags & RVLPSULMBUILDER_FLAG_PC) != 0 && (m_Flags2 & RVLPSULMBUILDER_FLAG2_WIDE_ANGLE_HYPOTHESIS_GENERATION) != 0)
		iAlphaRange = 3;

	int iAlpha = -iAlphaRange;

	int nM3DSurfaces;
	RVLPSULM_MSMATCH_DATA *pMSMatch;
	RVLPSULM_MSMATCH_DATA *pMSMatchListEnd;
	//int maxSurfIndexSum;
	CRVL3DSurface2 **pMSurfArrayEnd;
	//RVLPSULM_SURF_MATCH_DATA *pMSurfMatchData;
	//RVLPSULM_SURF_MATCH_DATA *pMSurfMatchDataArrayEnd;
	double *t2, *P2;
	//double *P, *R2, *P3;
	CRVL3DPose *pPoseSM;
	int Support;
	int iLastMSurf;
	int cost, minCost;
	//int MSupportTotal;
	RVLPSULM_HG_NODE *pNode, *pNewNode, *pPNode, *pNode2;
	//double *invt, *invt2;
	int iMinSupport, iMaxSupport, iMidSupport;
	int iMSurf, iSSurf;
	//int SSupport;
	//int nMSMatches;
	int iLastMSMatch;
	RVLQLIST *QueueList;
	int PredictedScore;
	int nExpandedNodes;
	RVLPSULM_HYPOTHESIS **ppHypothesis;
	CRVL3DPose *pPoseSM2;
	BOOL bNewHypothesis;
	double err;
	int maxCost;
	CRVL3DSurface2 **ppMatchedSSurf, **ppMatchedMSurf;
	double *pPTmp;
	double mu;
	double Jsmu[2], JsTD[2];
	double muTol;
	double TraveledDist, varTD;
	double k1;
	double *tFA, *tF2B, *RFA, *RF2B, *zTB2;
	double tF2A[3], tF2_FA[3], tFB[3], zTA[3], xTF2[3], yTF2[3], xTF[3], yTF[3], Q[3], RFB[3 * 3];
	double Mx3x3Tmp[3 * 3];
	double Vect3Tmp[3];
	double dx, dy, dtFB[3], eF, vartFAx, vartFAy, vartFAz, vartF2Bx, vartF2By, vartF2Bz, eTD, csT, wTD;
	//int iTraveledDistBin, iBestTraveledDistBin, BestTravelDistScore;
	int nLastDOFMatches;
	RVLPSULM_LASTDOF_MATCH_DATA *pLastDOFMatchData, *pLastDOFMatchData2, *pLastDOFMatchArrayEnd, *pBestLastDOFMatch;
	double TDScore, BestTDScore;
	//BOOL bLastDOFMatchStored, bLastDOFMatchMerged;
	//int iNewLastDOFMatch;
	RVLPSULM_HYPOTHESIS *pHypothesis2;
	double Vect3Tmp2[3];
	int nFeatures;
	double eig[3], Veig[3];
	BOOL bReal[3];
	RVLPSULM_LASTDOF_MATCH_DATA *LastDOFMatchArray;
	int LastDOFMatchArraySize;
	double ep1, ep2;
	double ca, sa, alpha;

	//int debug_nMatchings = 0;
	//int debug_nQueueSorts = 0;
	
#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
	FILE *fpLog;

	fopen_s(&fpLog, "C:\\RVL\\Debug\\PSuLMHypGen.dat", "w");
#endif

	CRVLMPtrChain *pPSuLMList;

	if(m_PSuLMSubList.m_nElements !=0 || (m_Flags & RVLPSULMBUILDER_FLAG_NEW_SUBMAP) != 0)
		pPSuLMList = &m_PSuLMSubList;
	else
		pPSuLMList = &m_PSuLMList;

	pPSuLMList->Start();

	while(TRUE)
	{
		double StartTime = m_pTimer->GetTime();

		// initial feature matching 

		if(pPrevSPSuLM)
			pMPSuLM = pPrevSPSuLM;
		else if (pPSuLMList->m_pNext || iAlpha > -iAlphaRange)
		{
			//if (iAlpha == 0)
			//	int debug = 0;

			if (iAlpha == -iAlphaRange)
				pMPSuLM = (CRVLPSuLM *)(pPSuLMList->GetNext());

			if (m_Flags2 & RVLPSULMBUILDER_FLAG2_WIDE_ANGLE_HYPOTHESIS_GENERATION)
			{
				alpha = (double)iAlpha * 0.25 * PI;

				ca = cos(alpha);
				sa = sin(alpha);

				RSS_[0] = RSS_[8] = ca;
				RSS_[2] = sa;
				RSS_[6] = -sa;

				R = PoseSMInit.m_Rot;

				RVLMXMUL3X3(RS_M, RSS_, R)

				PoseSMInit.UpdatePTRLL();

				PoseSMInit.m_sa = sin(PoseSMInit.m_Alpha);
				PoseSMInit.m_ca = cos(PoseSMInit.m_Alpha);

				RVLMULMX3X3TVECT(R, t, invtInit);

				iAlpha++;

				if (iAlpha > iAlphaRange)
					iAlpha = -iAlphaRange;
			}
		}
		else
			break;

		if(pPoseS0Init == NULL)
		{
			PoseSMInit.Copy(&(pMPSuLM->m_PoseCsInit));
			PoseSMInit.m_pData = invtInit;
			R = PoseSMInit.m_Rot;
			t = PoseSMInit.m_X;
			RVLMULMX3X3TVECT(R, t, invtInit);
		}

		//if(pMPSuLM->m_Index == 42 && RVLABS(alpha) < APPROX_ZERO)	// debug
		//	int debug = 0;

		MSurfArray = pMPSuLM->m_3DSurfaceArray;

		ppMSurf = MSurfArray;

		nM3DSurfaces = pMPSuLM->m_n3DSurfaces;

		maxCost = nS3DSurfaces + nM3DSurfaces;

		iLastMSurf = nM3DSurfaces - 1;

		//pMSurfMatchData = MSurfMatchData;

		//MSupportTotal = 0;

		for(i=0;i<nM3DSurfaces;i++)
		{
			pM3DSurface = MSurfArray[i];

			pM3DSurface->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

			*(ppMSurf++) = pM3DSurface;

			//pMSurfMatchData->p3DSurface = pM3DSurface;

			//pMSurfMatchData->bMatched = 0;

			//pMSurfMatchData++;

			//MSupportTotal += pM3DSurface->m_nSupport;		// move to load or create function
		}

		//SupportTotal = (SSupportTotal + MSupportTotal) * 80 / 100;

		pMSurfArrayEnd = ppMSurf;

		Queue.Reset();

		//pMSurfMatchDataArrayEnd = pMSurfMatchData;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
		pMPSuLM->AddToFigure(m_DebugData.pGUI);

		//pMouseCallbackData->nMSurfs = pM3DSurfaceList->m_nElements;

		cvWaitKey();
#endif
		// create a sorted list of matches

		pMSMatch = MatchList;

		pM3DSurface = MSurfArray[0];
		pS3DSurface = SSurfArray[0];
		pMSMatch->pMData = (RVLPSULM_SURF_MATCH_DATA *)pM3DSurface;
		pMSMatch->pSData = (RVLPSULM_SURF_MATCH_DATA *)pS3DSurface;
		//pMSMatch->Support = pM3DSurface->m_nSupport + pS3DSurface->m_nSupport;
		pMSMatch->Support = maxCost;
		pMSMatch->Flags = 0x00;

		iLastMSMatch = 0;

		pPTmp = PoseSMInit.m_C;

		if(pPoseS0Init)
			PoseSMInit.m_C = PInit;

		for(iSSurf = 0; iSSurf < nS3DSurfaces; iSSurf++)
		{
			pS3DSurface = SSurfArray[iSSurf];

			//SSupport = pS3DSurface->m_nSupport;

			for(iMSurf = 0; iMSurf < nM3DSurfaces; iMSurf++)
			{
				//if(iSSurf == 9 && iMSurf == 6)
				//	int debug = 0;

				pM3DSurface = MSurfArray[iMSurf];	

				////////// Initial match by color histogram - FILKO
				//if ((pM3DSurface->m_noUsedColorPts >= 20) && (pS3DSurface->m_noUsedColorPts >= 20))
				//{
					//if ((m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL) && (pS3DSurface->RVLIntersectColorHistogram(pM3DSurface, 5, true, m_IntersectionHelperArray) < m_MatIntersectTHR))
					//	continue;
				//}

				////////// Initial match by LBP histogram - FILKO
				//if ((pM3DSurface->m_noUsedLbpPts >= 20) && (pS3DSurface->m_noUsedLbpPts >= 20))
				//{
				//	if ((m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL) && (CRVL3DMeshObject::RVLIntersectHistograms(pS3DSurface->m_LBP_RIU_VAR, pM3DSurface->m_LBP_RIU_VAR, pS3DSurface->m_noUsedLbpPts, pM3DSurface->m_noUsedLbpPts, 17*26, m_pMem2, -1, m_IntersectionHelperArray) < m_MatIntersectTHR))
				//		continue;
				//}

				if (m_Flags2 & RVLPSULMBUILDER_FLAG2_HYPGEN_INIT_MATCHING_CONSTRAINTS)
					if(!pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, detQ, &MatchData))
						continue;

				//Support = DOUBLE2INT(sqrt((double)(SSupport * pM3DSurface->m_nSupport)));
				//Support = SSupport + pM3DSurface->m_nSupport;
				Support = maxCost - (iSSurf + iMSurf);

				if(Support <= MatchList[iLastMSMatch].Support)
				{
					iLastMSMatch++;

					pMSMatch = MatchList + iLastMSMatch;
				}
				else
				{
					iMaxSupport = 0;
		
					iMinSupport = iLastMSMatch;

					while(iMinSupport - iMaxSupport > 1)
					{
						iMidSupport = (iMinSupport + iMaxSupport) / 2;

						if(Support > MatchList[iMidSupport].Support)
							iMinSupport = iMidSupport;
						else
							iMaxSupport = iMidSupport;
					}

					memmove(MatchList + iMinSupport + 1, MatchList + iMinSupport, 
						(iLastMSMatch - iMinSupport + 1) * sizeof(RVLPSULM_MSMATCH_DATA));

					pMSMatch = MatchList + iMinSupport;

					iLastMSMatch++;
				}

				pMSMatch->pMData = (RVLPSULM_SURF_MATCH_DATA *)pM3DSurface;
				pMSMatch->pSData = (RVLPSULM_SURF_MATCH_DATA *)pS3DSurface;
				pMSMatch->Support = Support;
				pMSMatch->Flags = 0x00;
			}
		}

		//If no matches was found
		if(iLastMSMatch == 0)
		{
   			if(pPrevSPSuLM)
				break;

			continue;
		}

		PoseSMInit.m_C = pPTmp;

		pMSMatchListEnd = MatchList + iLastMSMatch + 1;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG

		fprintf(fpLog, "Model %d\n\n", pMPSuLM->m_Index);	// for Nyarko

		if (m_Flags2 & RVLPSULMBUILDER_FLAG2_WIDE_ANGLE_HYPOTHESIS_GENERATION)
			fprintf(fpLog, "Initial alpha=%lf\n", PoseSMInit.m_Alpha * RAD2DEG);

		fprintf(fpLog, "Initial Pose Uncertainty\n");	// for Nyarko

		RVLPrintCov(fpLog, PoseSMInit.m_C, 3, RAD2DEG);	// for Nyarko

		RVLPrintCov(fpLog, PoseSMInit.m_C + 2 * 3 * 3, 3);	// for Nyarko

		fprintf(fpLog, "\n");	// for Nyarko

		fprintf(fpLog, "Match List:\n");

		for(pMSMatch = MatchList; pMSMatch < pMSMatchListEnd; pMSMatch++)
		{
			pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
			pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

			//if(pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, detQ, &MatchData))
				fprintf(fpLog, "S%d-M%d\t\t%d\n", 
					((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Index,
					((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Index,
					pMSMatch->Support);
		}
#endif

		
		// create the first node 

		pNewNode = NodeMem;

		pMSMatch = HypothesesGetNextNode(NULL, 1, MatchList, pMSMatchListEnd, PredictedScore);

		pNewNode->iMatch = 1;
		pNewNode->pParent = NULL;
		pNewNode->Support = pMSMatch->Support;
		pNewNode->g = 1;
		pNewNode->nFailures = 0;
		pNewNode->PoseSM.m_C = pNewNode->P;
		pNewNode->PoseSM.m_pData = pNewNode->invt;
		
		//cost = (SupportTotal - pNewNode->Support) / pNewNode->Support + 1;
		
		//cost = (pNewNode->Support + PredictionHorizon * pNewNode->Support) / 64;

		//cost = PredictedScore / 10;

		cost = PredictedScore;

		if(cost > QueueSize)
			cost = QueueSize;

		RVLQLISTARRAY_ADD_ENTRY2(QueueListArray, cost, pNewNode);

		pNode = pNewNode;

		pNewNode++;

		minCost = cost;
			
		//*** generate hypotheses

		iHypothesis = 0;

		nExpandedNodes = 0;

		nFeatures = 0;

		ppHypothesis = HypothesisArray;

		//while(iHypothesis < nHypotheses && nFeatures <= m_maxnExpandedNodes)
		while(nExpandedNodes <= m_maxnExpandedNodes && nFeatures <= m_refnHypotheses)
		//while(nExpandedNodes <= m_maxnExpandedNodes)
		{
			// skip the empty bins in queue

			while(QueueListArray[minCost].pFirst == NULL)
			{
				if(minCost > 0)
					minCost--;
				else
					break;
			}

			if(QueueListArray[minCost].pFirst == NULL)	// is queue empty?
				break;

			// pNode <- the node with the maximum weight from the queue

			pNode = (RVLPSULM_HG_NODE *)(QueueListArray[minCost].pFirst);

			QueueList = QueueListArray + minCost;

			// remove pNode from queue

			RVLQLIST_REMOVE_ENTRY2(QueueList, pNode, RVLPSULM_HG_NODE);

			// expand the parent node of pNode

			pPNode = pNode->pParent;

			//if(pPNode)
			//	if(pPNode->nFailures > maxnFailures)
			//		continue;

			//if(pNode->iMatch == 56 && pPNode->iMatch == 1)
			//	int debug = 0;

			if(pMSMatch = HypothesesGetNextNode(pPNode, pNode->iMatch + 1, MatchList, pMSMatchListEnd, PredictedScore))
			{
				//if(pPNode)
				//{
				//	pNewNode->Support = pPNode->Support;
				//	pNewNode->g = pPNode->g;
				//}
				//else
				//{
				//	pNewNode->Support = 0;
				//	pNewNode->g = 0;
				//}

				// pNewNode <- new node

				pNewNode->iMatch = pMSMatch - MatchList;
				pNewNode->pParent = pPNode;

				//dSupport = ((CRVL3DSurface2 *)(pMSMatch->pMData))->m_nSupport + 
				//	((CRVL3DSurface2 *)(pMSMatch->pSData))->m_nSupport;
				//pNewNode->Support += dSupport;
				pNewNode->g = (pPNode ? pPNode->g + 1 : 1);
				pNewNode->nFailures = 0;
				pNewNode->PoseSM.m_C = pNewNode->P;
				pNewNode->PoseSM.m_pData = pNewNode->invt;

				// add pNewNode to the queue 
				
				//cost = (SupportTotal - pNewNode->Support) / dSupport + pNewNode->g;

				//cost = (pNewNode->Support + PredictionHorizon * dSupport) / 64;				

				//cost = (pNewNode->Support + PredictedScore) / 10;

				//cost  = PredictedScore / 10;

				cost  = PredictedScore;

				if(cost > QueueSize)
					cost = QueueSize;

				if(cost > minCost)
					minCost = cost;

				RVLQLISTARRAY_ADD_ENTRY2(QueueListArray, cost, pNewNode);

//#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
//				pFig2->EmptyBitmap(cvSize(800, TextImageHeight), cvScalar(255, 255, 255));
//
//				HypothesisDisplay(pFig2, pNewNode, cvPoint(8, 0), MatchList);
//
//				m_DebugData.pGUI->ShowFigure(pFig2);
//
//				cvWaitKey();
//#endif
				pNewNode++;

				if(pNewNode == pNodeMemEnd)
					break;
			}

			//if(pNode->g > maxnMatchesInHypothesis)
			//{
			//	bNewHypothesis = FALSE;

			//	continue;
			//}

			// if pNode is a root node, then pPoseSM <- &PoseSMInit,
			// otherwise, pPoseSM <- pose of the parent node

			pPoseSM = (pPNode ? &(pPNode->PoseSM) : &PoseSMInit);
				
			// pMSMatch <- match corresponding to pNode

			pMSMatch = MatchList + pNode->iMatch;

			nExpandedNodes++;

			// check whether the surfaces of pMSMatch satisfy geometrical constraints  
			
			pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
			pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

			//if(pM3DSurface->m_Index == 7 && pS3DSurface->m_Index == 1)
 			//	int debug = 0;

			bool bCheckConsistency = true;
			bool b5DoF = true;

			if (m_Flags2 & RVLPSULMBUILDER_FLAG2_UNCONSTRAINED_ORIENTATION)
			{
				if (pNode->g == 1)
					bCheckConsistency = false;
				else if (pNode->g == 2)
					b5DoF = Compute5DoFPose(pNode, MatchList, PoseSMInit.m_X, &MatchData, &PoseSM5DoF);
			}

			//if(pS3DSurface->Match2(pM3DSurface, pPoseSM, MatchQuality, &MatchData))
			if (RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, pPoseSM, &(pNode->PoseSM), &MatchData, bCheckConsistency) && b5DoF)
			{
				// Only for debugging purpose!
				//
				//if (pNode->g == 2)
				//{
				//	CRVL3DPose dPose;
				//	RVLMXMUL3X3T1(PoseSM5DoF.m_Rot, pNode->PoseSM.m_Rot, dPose.m_Rot);
				//	double V[3];
				//	double q;
				//	dPose.GetAngleAxis(V, q);

				//	if (pPNode)
				//		pPNode->nFailures = 0;
				//}
				//
				// END DEBUGGING

				// EKF 
				
				//pNode->PoseSM.PlanarSurfaceEKFUpdate2(MatchData.m_C, MatchData.m_Q, MatchData.m_e, pPoseSM);

				//R = pNode->PoseSM.m_Rot;
				//t = pNode->PoseSM.m_X;
				//invt = pNode->invt;
				//RVLMULMX3X3TVECT(R, t, invt);

				//if(e[0] * e[0] + e[1] * e[1] > 0.04)
				//{
				//	memcpy(pNode->PoseSM.m_C, pPoseSM->m_C, 3 * 3 * 3 * sizeof(double));

				//	pS3DSurface->Match2(pM3DSurface, &(pNode->PoseSM), MatchQuality, &MatchData);

				//	pNode->PoseSM.PlanarSurfaceEKFUpdate2(MatchData.m_C, MatchData.m_Q, MatchData.m_e);

				//	RVLMULMX3X3TVECT(R, t, invt);
				//}

				// if the pose uncertainty is inside the tolerance specified by m_maxPoseSMUncert

				P2 = pNode->P + 2 * 3 * 3;

				//RVLMatrixHeaderA33->data.db = P2;



#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
					//if(HypothesisIndex == 113)
					//	int debug = 0;

					cvSet(pFig2->m_pImage, cvScalar(255, 255, 255));

					int iTextLine = HypothesisDisplay(pFig2, pNode, cvPoint(8, 0), MatchList);

					sprintf(Text, "cost=%d", minCost);

					pFig2->PutText(Text, cvPoint(8, (iTextLine + 1) * pFig2->m_FontSize), cvScalar(0, 0, 0));

					iTextLine++;

					pFig2->DisplayCovMx(pNode->P, 8, (iTextLine + 1) * pFig2->m_FontSize, cvScalar(0, 0, 0), RAD2DEG);

					iTextLine += 4;

					pFig2->DisplayCovMx(pNode->P + 2 * 3 * 3, 8, (iTextLine + 1) * pFig2->m_FontSize, cvScalar(0, 0, 0));

					iTextLine += 4;

					sprintf(Text, "iHypothesis=%d", iHypothesis);

					iTextLine++;

					pFig2->PutText(Text, cvPoint(8, (iTextLine + 1) * pFig2->m_FontSize), cvScalar(0, 0, 0));

					m_DebugData.pGUI->ShowFigure(pFig2);

					cvWaitKey();
#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
					PrintHypothesis(fpLog, pNode, MatchList);

					fprintf(fpLog, "nExpandedNodes: %d\n", nExpandedNodes);

					RVLPrintCov(fpLog, pNode->P, 3, RAD2DEG);

					RVLPrintCov(fpLog, pNode->P + 2 * 3 * 3, 3);

					fprintf(fpLog, "\n");

					//if (nExpandedNodes == 840)
					//	int debug = 0;
#endif
				//cvSVD(RVLMatrixHeaderA33, _eigVal, _eigVect, NULL, CV_SVD_U_T);

				//if(!RVLGetAxesOfCov3D(P2, eigVal, RBT, Mx3x3Tmp, Vect3Tmp))
				//	eigVal[2] = 2.0 * m_maxPoseSMUncert;

				//if(fabs(eigVal[0] - eigVal2[2]) > 10)
				//	int debug = 0;

				//if(eigVal2[2] < 0)
				//	int debug = 0;

				//if((eigVal[1] <= m_maxPoseSMUncert)
				RVLGetMaxEigVector3(pNode->P, eig, bReal, Veig);

				if(eig[2] <= m_maxvarRotHyp)
				//if((pNode->P[0] <= m_maxvarRotHyp && pNode->P[4] <= m_maxvarRotHyp && pNode->P[8] <= m_maxvarRotHyp && pNode->g >= 3)
				//	)
					bNewHypothesis = TRUE;
				else
				{
					// expand pNode

					if(pMSMatch = HypothesesGetNextNode(pNode, pNode->iMatch + 1, MatchList, pMSMatchListEnd, PredictedScore))
					{
						// pNewNode <- new node

						pNewNode->iMatch = pMSMatch - MatchList;
						pNewNode->pParent = pNode;
						//dSupport = ((CRVL3DSurface2 *)(pMSMatch->pMData))->m_nSupport + 
						//	((CRVL3DSurface2 *)(pMSMatch->pSData))->m_nSupport;
						//pNewNode->Support = pNode->Support + dSupport;
						pNewNode->g = pNode->g + 1;
						pNewNode->nFailures = 0;
						pNewNode->PoseSM.m_C = pNewNode->P;
						pNewNode->PoseSM.m_pData = pNewNode->invt;

						// add pNewNode to the queue 
						
						//cost = (SupportTotal - pNewNode->Support) / dSupport + pNewNode->g;

						//cost = (pNewNode->Support + PredictionHorizon * dSupport) / 64;						

						//cost = (pNode->Support + PredictedScore) / 10;

						//cost = PredictedScore / 10;

						cost = PredictedScore;

						if(cost > QueueSize)
							cost = QueueSize;

						if(cost > minCost)
							minCost = cost;

						RVLQLISTARRAY_ADD_ENTRY2(QueueListArray, cost, pNewNode);

						bNewHypothesis = FALSE;

						pNewNode++;

						if(pNewNode == pNodeMemEnd)
							break;
					}
					else
						bNewHypothesis = FALSE;
				}

				// check if a similar hypothesis already exists

				if(bNewHypothesis)
				{
#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
					fprintf(fpLog, "Rotation precision is sufficient.\n");
#endif
					if(RVLGetAxesOfCov3D(P2, eigVal, RBT, Mx3x3Tmp, Vect3Tmp))
					{						
						pPoseSM = &(pNode->PoseSM);

						R = pPoseSM->m_Rot;

						t = pPoseSM->m_X;

						Mx3x3Tmp[0] = 1.0 - zTB[0] * zTB[0];
						Mx3x3Tmp[1] =     - zTB[0] * zTB[1];
						Mx3x3Tmp[2] =     - zTB[0] * zTB[2];
						Mx3x3Tmp[4] = 1.0 - zTB[1] * zTB[1];
						Mx3x3Tmp[5] =     - zTB[1] * zTB[2];
						Mx3x3Tmp[8] = 1.0 - zTB[2] * zTB[2];

						RVLMULCOV3VECT(Mx3x3Tmp, t, Vect3Tmp)

						for(i = 0; i < iHypothesis && bNewHypothesis; i++)
						{
							pHypothesis2 = HypothesisArray[i];

							pPoseSM2 = &(pHypothesis2->PoseSM);

							err = pPoseSM->m_Alpha - pPoseSM2->m_Alpha;

							if(err > m_HypSeparationAngle * DEG2RAD || err < -m_HypSeparationAngle * DEG2RAD)
								continue;

							err = pPoseSM->m_Beta - pPoseSM2->m_Beta;

							if(err > m_HypSeparationAngle * DEG2RAD || err < -m_HypSeparationAngle * DEG2RAD)
								continue;

							err = pPoseSM->m_Theta - pPoseSM2->m_Theta;

							if(err > m_HypSeparationAngle * DEG2RAD || err < -m_HypSeparationAngle * DEG2RAD)
								continue;

							zTB2 = pHypothesis2->zTB;

							csT = RVLDOTPRODUCT3(zTB, zTB2);

							if(csT > -m_csLastDOFSeparationAngle && csT < m_csLastDOFSeparationAngle)
								continue;

							t2 = pPoseSM2->m_X;
							
							RVLMULCOV3VECT(Mx3x3Tmp, t2, RVLVector3)

							RVLDIF3VECTORS(Vect3Tmp, RVLVector3, Vect3Tmp2)

							if(RVLDOTPRODUCT3(Vect3Tmp2, Vect3Tmp2) <= 10000.0)
							{
								bNewHypothesis = FALSE;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
								fprintf(fpLog, "Similar hypothesis already exists.\n");
#endif

								break;
							}
						}
					}	// if P2 is a regular covariance matrix
					else
						bNewHypothesis = FALSE;
				}	// if(bNewHypothesis)

				nLastDOFMatches = 0; 

				if(bNewHypothesis)
				{
					// Refine 5DoF hypothesis

					// BLOCK: The following method worked well with Velodyne, but slightly worse with IROS15 data

					//double PtT[9];
					//RVLDIAGMX3(1e8, 1e8, 1e8, PtT);
					//double P[3*9];
					//double *Pq = P;
					//double *Pqt = P + 9;
					//double *Pt = Pqt + 9;
					//RVLDIAGMX3(20.0 * 20.0 * DEG2RAD * DEG2RAD, 20.0 * 20.0 * DEG2RAD * DEG2RAD, 20.0 * 20.0 * DEG2RAD * DEG2RAD, Pq);
					//RVLNULLMX3X3(Pqt);
					//double RTB[9];
					//RVLCOPYMX3X3T(RBT, RTB);
					//RVLCOV3DTRANSF(PtT, RTB, Pt, Mx3x3Tmp);
					//RVLCOMPLETESIMMX3(Pt);
					//double P_[3 * 9];

					//memcpy(P_, pPoseSM->m_C, 3 * 9 * sizeof(double));

					//RVLPSuLMHypothesisPoseRefinement(pPoseSM, pNode, MatchList, P, 5);					

					//memcpy(pPoseSM->m_C, P_, 3 * 9 * sizeof(double));

					// END BLOCK

					//RVLPSuLMHypothesisPoseRefinement(pPoseSM, pNode, MatchList, PoseSMInit.m_C, 5);

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
					fprintf(fpLog, "Estimating the last DOF...\n");

					//if (((CRVL3DSurface2 *)(MatchList[pNode->iMatch].pMData))->m_Index == 1 &&
					//	((CRVL3DSurface2 *)(MatchList[pNode->iMatch].pSData))->m_Index == 13 &&
					//	((CRVL3DSurface2 *)(MatchList[pNode->pParent->iMatch].pMData))->m_Index == 0 &&
					//	((CRVL3DSurface2 *)(MatchList[pNode->pParent->iMatch].pSData))->m_Index == 6)
					//	int debug = 0;
#endif

#pragma region Estimation of the last DOF
					//*** estimate the last degree of freedom by evidence accumulation

					//FILE *fpLastDOF;

					//fopen_s(&fpLastDOF, "C:\\RVL\\ExpRez\\LastDOF.dat", "w");

					//memset(TraveledDistAcc, 0, nTraveledDistBins * sizeof(int));

					//BestTravelDistScore = 0;

					LastDOFMatchArraySize = nS3DSurfaces * nM3DSurfaces;

#ifdef RVLPSULM_LINES
					nM3DLines = pMPSuLM->m_n3DLines;

					LastDOFMatchArraySize += (nS3DLines * nM3DLines);
#endif

					LastDOFMatchArray = new RVLPSULM_LASTDOF_MATCH_DATA[LastDOFMatchArraySize];

					maxvarTD = 0.0;

					for(pMSMatch = MatchList + 1; pMSMatch < pMSMatchListEnd; pMSMatch++)
					{
						pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);

						RF2B = pM3DSurface->m_Pose.m_Rot;

						mu = fabs(RVLDOTPRODUCT3(zTB, pM3DSurface->m_N));
						Jsmu[0] = RVLMULROWCOL3(zTB, RF2B, 0, 0);
						Jsmu[1] = RVLMULROWCOL3(zTB, RF2B, 0, 1);
						muTol = sqrt(Jsmu[0] * Jsmu[0] * pM3DSurface->m_varq[0] + 
							Jsmu[1] * Jsmu[1] * pM3DSurface->m_varq[1]);
						
						if(mu + muTol < m_csLastDOFSurfNrmAngle)
							continue;

						pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

						// zTA = RAB' * zTB

						RVLMULMX3X3TVECT(R, zTB, zTA)

						RFA = pS3DSurface->m_Pose.m_Rot;

						mu = fabs(RVLDOTPRODUCT3(zTA, pS3DSurface->m_N));
						Jsmu[0] = RVLMULROWCOL3(zTA, RFA, 0, 0);
						Jsmu[1] = RVLMULROWCOL3(zTA, RFA, 0, 1);
						muTol = sqrt(Jsmu[0] * Jsmu[0] * pS3DSurface->m_varq[0] + 
							Jsmu[1] * Jsmu[1] * pS3DSurface->m_varq[1]);
						
						if(mu + muTol < m_csLastDOFSurfNrmAngle)
							continue;

						// RFB = RAB * RFA

						RVLMXMUL3X3(R, RFA, RFB);

						if (RVLMULCOLCOL3(RF2B, RFB, 2, 2) < COS45)
							continue;

						// [xTF2'; yTF2'] = RF2B' * [xTB'; yTB']

						RVLMULMX3X3TVECT(RF2B, xTB, xTF2)
						RVLMULMX3X3TVECT(RF2B, yTB, yTF2)
					
						// [xTF'; yTF'] = RFB' * [xTB'; yTB']	

						RVLMULMX3X3TVECT(RFB, xTB, xTF)
						RVLMULMX3X3TVECT(RFB, yTB, yTF)

						// Q = [xTF2'; yTF2'] * CF2 * [xTF2, yTF2] + [xTF'; yTF'] * CF * [xTF, yTF]

						vartFAx  = pS3DSurface->m_EigenValues[0] * pS3DSurface->m_EigenValues[0];
						vartFAy  = pS3DSurface->m_EigenValues[1] * pS3DSurface->m_EigenValues[1];
						vartFAz  = pS3DSurface->m_sigmaR;
						vartF2Bx = pM3DSurface->m_EigenValues[0] * pM3DSurface->m_EigenValues[0];
						vartF2By = pM3DSurface->m_EigenValues[1] * pM3DSurface->m_EigenValues[1];
						vartF2Bz = pM3DSurface->m_sigmaR;

						Q[0] = vartF2Bx*xTF2[0]*xTF2[0] + vartF2By*xTF2[1]*xTF2[1] + vartF2Bz*xTF2[2]*xTF2[2] +
							vartFAx*xTF[0]*xTF[0] + vartFAy*xTF[1]*xTF[1] + vartFAz*xTF[2]*xTF[2];
						Q[1] = vartF2Bx*xTF2[0]*yTF2[0] + vartF2By*xTF2[1]*yTF2[1] + vartF2Bz*xTF2[2]*yTF2[2] +
							vartFAx*xTF[0]*yTF[0] + vartFAy*xTF[1]*yTF[1] + vartFAz*xTF[2]*yTF[2];
						Q[2] = vartF2Bx*yTF2[0]*yTF2[0] + vartF2By*yTF2[1]*yTF2[1] + vartF2Bz*yTF2[2]*yTF2[2] +
							vartFAx*yTF[0]*yTF[0] + vartFAy*yTF[1]*yTF[1] + vartFAz*yTF[2]*yTF[2];

						tFA = pS3DSurface->m_Pose.m_X;

						tF2B = pM3DSurface->m_Pose.m_X;

						// tFB = RAB * tFA + tAB

						RVLMULMX3X3VECT(R, tFA, tFB)
						RVLSUM3VECTORS(tFB, t, tFB)

						// dtFB = tFB - tFB2

						RVLDIF3VECTORS(tFB, tF2B, dtFB)

						// d = [dx, dy]' = [1 0 0; 0 1 0] * RBT * dtFB

						dx = RVLDOTPRODUCT3(xTB, dtFB);
						dy = RVLDOTPRODUCT3(yTB, dtFB);

						// eF = d' * inv(Q) * d

						eF = (Q[2]*dx*dx - 2*Q[1]*dx*dy + Q[0]*dy*dy)/(Q[0]*Q[2] - Q[1]*Q[1]);

						if(eF > m_MahDistDOFSurfOverlapTest)
							continue;

						//fprintf(fpLastDOF, "M%d-S%d\n", pM3DSurface->m_Index, pS3DSurface->m_Index);

						// k1 = 1 / VA' * zFA

						k1 = RVLMULROWCOL3(zTA, RFA, 0, 2);

						if(k1 > -APPROX_ZERO && k1 < APPROX_ZERO)
							continue;

						k1 = 1.0 / k1;

						//if(HypothesisIndex == 134 && pS3DSurface->m_Index == 2 && pM3DSurface->m_Index == 7)
						//	int debug = 0;

						// tF2A = RAB' * (tF2B - tAB)

						RVLDIF3VECTORS(tF2B, t, Vect3Tmp)
						RVLMULMX3X3TVECT(R, Vect3Tmp, tF2A)

						// tF2_FA = tF2A - tFA

						RVLDIF3VECTORS(tF2A, tFA, tF2_FA)

						// Vect3Tmp = k1 * (tF2_FA - k1 * zTA)

						//Vect3Tmp[0] = k1 * (tF2_FA[0] - k1 * zTA[0]);
						//Vect3Tmp[1] = k1 * (tF2_FA[1] - k1 * zTA[1]);
						//Vect3Tmp[2] = k1 * (tF2_FA[2] - k1 * zTA[2]);

						// JsTD = Vec3Tmp' * [xFA, yFA]

						//JsTD[0] = RVLMULROWCOL3(Vect3Tmp, RFA, 0, 0);
						//JsTD[1] = RVLMULROWCOL3(Vect3Tmp, RFA, 0, 1);

						// Computation of varTD is explained in Appendix B of IJRR13, where varTD corresponds to the symbol sigma_l in the last equation in the Appendix B.
						// varTD = JsTD * Cs * JsTD'

						//varTD = JsTD[0] * JsTD[0] * pS3DSurface->m_varq[0] + 
						//	JsTD[1] * JsTD[1] * pS3DSurface->m_varq[1];

						varTD = k1 * k1 * (pS3DSurface->m_sigmaR + pM3DSurface->m_sigmaR);

						//if(varTD > 90000.0)
						//	continue;

						// Computation of TraveledDist is explained in Appendix B of IJRR13, where TraveledDist corresponds to the symbol l with ^.
						// TraveledDist = k1 * tF2_FA * zFA

						TraveledDist = k1 * RVLMULROWCOL3(tF2_FA, RFA, 0, 2);

						if(TraveledDist < -m_maxLastDOFTravelDist)
							continue;

						if(TraveledDist > m_maxLastDOFTravelDist)
							continue;

						if(varTD > maxvarTD)
							maxvarTD = varTD;

						wTD = 1.0 / sqrt(2.0 * varTD);			

#ifdef RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_PROBABILISTIC
						// compute the z-axis of the match reference frame

						double RPT[3*3];
						double *XP = RPT;
						double *YP = RPT + 3;
						double *ZP = RPT + 6;

						double RFT[3*3];
						double *ZF = RFT + 6;
						RVLCOPYMX3X3T(RFB, RFT)

						double RFT_[3*3];
						double *ZF_ = RFT_ + 6;
						RVLCOPYMX3X3T(RF2B, RFT_)

						RVLSUM3VECTORS(ZF, ZF_, ZP)
						RVLNORM3(ZP, fTmp)

						// compute the x and y-axes of the match reference frame

						RVLCROSSPRODUCT3(ZF, ZF_, XP)

						fTmp = RVLDOTPRODUCT3(XP, XP);

						int i, j, k;

						if(fTmp <= APPROX_ZERO)		
							//RVLORTHOGONAL3(ZP, XP, i, j, k, Vect3Tmp, fTmp)
							RVLORTHOGONAL3(ZP, XP, i, j, k, fTmp)
						else
						{
							fTmp = sqrt(fTmp);

							RVLSCALE3VECTOR2(XP, fTmp, XP)
						}

						RVLCROSSPRODUCT3(ZP, XP, YP);

						// transform the matched surfaces into the match reference frame

						double sy = RVLDOTPRODUCT3(ZF, YP);

						double Cn[2*2];
						pS3DSurface->TransfToMatchRefFrame(RFT, RPT, Cn);

						double Cn_[2*2];
						pM3DSurface->TransfToMatchRefFrame(RFT_, RPT, Cn_);

						// orientation probability 

						double CnS[2*2];

						CnS[0] = Cn[0] + Cn_[0] + m_SurfaceMatchData.varOrientationUncert;
						CnS[1] = Cn[1] + Cn_[1];
						CnS[3] = Cn[3] + Cn_[3] + m_SurfaceMatchData.varOrientationUncert;

						double detCnS = RVLDET2(CnS);

						double en, Pn;

						if(detCnS > 4.0)
							Pn = 0.0;
						else
						{
							en = CnS[0] * 4.0 * sy * sy / detCnS;

							Pn = RVLLN4PI - 0.5*(log(detCnS)+en) - RVLLN2PI;

							if(Pn < 0.0)
								Pn = 0.0;
						}							

						TDScore = m_SurfaceMatchData.PPriorPosition + log(wTD) - 0.5 * RVLLNPI + Pn;
#else
						TDScore = wTD * (double)(pS3DSurface->m_nSupport <= pM3DSurface->m_nSupport ? 
							pS3DSurface->m_nSupport : pM3DSurface->m_nSupport);
#endif

						//if(fabs(TraveledDist) > fmaxTraveledDist)
						//	continue;

						//iTraveledDistBin = DOUBLE2INT(TraveledDist / fTraveledDistBinSize);

						//TraveledDistBin[iTraveledDistBin] += 
						//	((pS3DSurface->m_nSupport + pM3DSurface->m_nSupport) / DOUBLE2INT(varTD));

						//if(TraveledDistBin[iTraveledDistBin] > BestTravelDistScore)
						//{
						//	BestTravelDistScore = TraveledDistBin[iTraveledDistBin];

						//	iBestTraveledDistBin = iTraveledDistBin;
						//}

						////fprintf(fpLastDOF, "M%d-S%d\t%lf\t%lf\n", pM3DSurface->m_Index, pS3DSurface->m_Index, 
						////	TraveledDist, sqrt(varTD));

#ifdef NEVER
						bLastDOFMatchStored = bLastDOFMatchMerged = FALSE;

						for(iLastMSMatch = 0; iLastMSMatch < nLastDOFMatches; iLastMSMatch++)
						{
							pLastDOFMatchData = LastDOFMatchArray + iLastMSMatch;

							if(pLastDOFMatchData->pMatch == NULL)
								continue;

							eTD = TraveledDist - pLastDOFMatchData->TraveledDist;

							if(eTD < -50.0 || eTD > 50.0)
								continue;

							if(TDScore > pLastDOFMatchData->score)
							{
								if(bLastDOFMatchStored)
									pLastDOFMatchData->pMatch = NULL;
								else
								{
									pLastDOFMatchData->pMatch = pMSMatch;

									pLastDOFMatchData->score = TDScore;

									pLastDOFMatchData->TraveledDist = TraveledDist;

									iNewLastDOFMatch = iLastMSMatch;

									bLastDOFMatchStored = TRUE;
								}
							}
							else
							{
								bLastDOFMatchMerged = TRUE;

								if(bLastDOFMatchStored)
									LastDOFMatchArray[iNewLastDOFMatch].pMatch = NULL;
							}
						}

						if(!bLastDOFMatchStored && !bLastDOFMatchMerged)
						{
							pLastDOFMatchData = LastDOFMatchArray + nLastDOFMatches;

							pLastDOFMatchData->pMatch = pMSMatch;

							pLastDOFMatchData->score = TDScore;

							pLastDOFMatchData->TraveledDist = TraveledDist;
					
							nLastDOFMatches++;
						}
#endif

						pLastDOFMatchData = LastDOFMatchArray + nLastDOFMatches;

						pLastDOFMatchData->pMatch = pMSMatch;

						pLastDOFMatchData->Support = TDScore;

						pLastDOFMatchData->TraveledDist = TraveledDist;

						pLastDOFMatchData->wTD = wTD;
				
						nLastDOFMatches++;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
						fprintf(fpLog, "M%d-S%d\tTD=%lf\tS=%lf\n", pM3DSurface->m_Index, pS3DSurface->m_Index, TraveledDist, pLastDOFMatchData->Support);
#endif
					}	// for each initial surface match

					int nSMSurfMatches = nLastDOFMatches;

#ifdef RVLPSULM_LINES
					//** create line matches	

					RVLPSULM_MSMATCH_DATA *LineMatchArray = new RVLPSULM_MSMATCH_DATA[nS3DLines * nM3DLines];
					RVLPSULM_LINE_MATCH_DATA *LineMatchData = new RVLPSULM_LINE_MATCH_DATA[2 * nS3DLines * nM3DLines];

					RVLPSULM_MSMATCH_DATA *pLineMatch = LineMatchArray;
					RVLPSULM_LINE_MATCH_DATA *pLineMatchData = LineMatchData;

					// xTlB = zTB

					RVLCOPY3VECTOR(zTB, xTlB)

					//M3DLineArrayLastDOF = new CRVL3DLine2[nM3DLines];

					for(iS3DLine = 0; iS3DLine < nS3DLines; iS3DLine++)
					{
						pS3DLine = pSPSuLM->m_3DLineArray[iS3DLine];

						pS3DLineData = (RVL3DLINE_EXTENDED_DATA *)(pS3DLine->m_pData);

						// XS1B = RAB * XS1A + tAB; XS2B = RAB * XS2A + tAB

						XS1A = pS3DLine->m_X[0];
						XS2A = pS3DLine->m_X[1];

						RVLMULMX3X3VECT(R, XS1A, XS1B)
						RVLSUM3VECTORS(XS1B, t, XS1B)
						RVLMULMX3X3VECT(R, XS2A, XS2B)
						RVLSUM3VECTORS(XS2B, t, XS2B)

						// dXSB = XS2B - XS1B;

						RVLDIF3VECTORS(XS2B, XS1B, dXSB)

						// VSB = dXSB / |dXSB|

						RVLSCALE3VECTOR2(dXSB, pS3DLineData->len, VSB)

						// check if the angle between pS3DLine and the last DOF is sufficiently large  

						csT = RVLDOTPRODUCT3(VSB, xTlB);

						if(csT < -m_csLastDOFLineNrmAngle || csT > m_csLastDOFLineNrmAngle)
							continue;

						// CXS1B = RAB * CXS1A * RAB'; CXS2B = RAB * CXS2A * RAB'

						CXS1A = pS3DLine->m_CX[0];
						CXS2A = pS3DLine->m_CX[1];

						RVLCOV3DTRANSF(CXS1A, R, CXS1B, M3x3Tmp)
						RVLCOMPLETESIMMX3(CXS1B)
						RVLCOV3DTRANSF(CXS2A, R, CXS2B, M3x3Tmp)
						RVLCOMPLETESIMMX3(CXS2B)

						/////

						SSupport = (double)(pS3DLine->m_nSupport);

						kzOverlap = m_LastDOFLineOverlapThr / SSupport;
						
						for(iM3DLine = 0; iM3DLine < nM3DLines; iM3DLine++)
						{
							pM3DLine = pMPSuLM->m_3DLineArray[iM3DLine];

							//if(iS3DLine == 36 && iM3DLine == 21)
							//	int debug = 0;

							pM3DLineData = (RVL3DLINE_EXTENDED_DATA *)(pM3DLine->m_pData);

							// check if the angle between pM3DLine and the last DOF is sufficiently large

							VM = pM3DLineData->V;

							csT = RVLDOTPRODUCT3(VM, xTlB);

							if(csT < -m_csLastDOFLineNrmAngle || csT > m_csLastDOFLineNrmAngle)
								continue;

							// check if the angle between pS3DLine and pM3DLine is sufficiently small

							if(RVLDOTPRODUCT3(VSB, VM) < m_csLastDOFLineNrmAngle)
								continue;

							// yTBl = ((VSB + VM) x xTBl) / |(VSB + VM) x xTBl|

							RVLSUM3VECTORS(VSB, VM, V3x1Tmp)
							RVLCROSSPRODUCT3(V3x1Tmp, xTlB, yTlB)
							fTmp = sqrt(RVLDOTPRODUCT3(yTlB, yTlB));
							RVLSCALE3VECTOR2(yTlB, fTmp, yTlB)

							// zTBl = xTBl x yTBl

							RVLCROSSPRODUCT3(xTlB, yTlB, zTlB)

							// XS1Tl = RBTl * XS1B; XS2Tl = RBT * XS2B

							RVLMULMX3X3VECT(RBTl, XS1B, XS1Tl)
							RVLMULMX3X3VECT(RBTl, XS2B, XS2Tl)

							// XM1Tl = RBTl * XM1; XM2Tl = RBT * XM2

							XM1 = pM3DLine->m_X[0];
							XM2 = pM3DLine->m_X[1];

							RVLMULMX3X3VECT(RBTl, XM1, XM1Tl)
							RVLMULMX3X3VECT(RBTl, XM2, XM2Tl)

							// z-overlaping test

							dzSTl = XS2Tl[2] - XS1Tl[2];
							dzMTl = XM2Tl[2] - XM1Tl[2];

							if(XM2Tl[2] < XS2Tl[2])
							{
								minz2 = XM2Tl[2];
								sS2 = (XM2Tl[2] - XS1Tl[2]) / dzSTl;
								sM2 = 1.0;
							}
							else
							{
								minz2 = XS2Tl[2];
								sS2 = 1.0;
								sM2 = (XS2Tl[2] - XM1Tl[2]) / dzMTl;
							}

							if(XM1Tl[2] > XS1Tl[2])
							{
								maxz1 = XM1Tl[2];
								sS1 = (XM1Tl[2] - XS1Tl[2]) / dzSTl;
								sM1 = 0.0;
							}
							else
							{
								maxz1 = XS1Tl[2];
								sS1 = 0.0;
								sM1 = (XS1Tl[2] - XM1Tl[2]) / dzMTl;
							}

							overlap = (minz2 - maxz1) / dzSTl;

							if(overlap < kzOverlap)
								continue;

							//* crop point S1

							// XS1Tl_ = (1 - sS1) * XS1Tl + sS1 * XS2Tl

							fTmp = 1.0 - sS1;

							XS1Tl_[0] = fTmp * XS1Tl[0] + sS1 * XS2Tl[0];
							XS1Tl_[1] = fTmp * XS1Tl[1] + sS1 * XS2Tl[1];

							// XS1A_ = (1 - sS1) * XS1A + sS1 * XS2A

							RVLSCALE3VECTOR(XS1A, fTmp, XS1A_)
							RVLSCALE3VECTOR(XS2A, sS1, V3x1Tmp)
							RVLSUM3VECTORS(XS1A_, V3x1Tmp, XS1A_)

							// CXS1Tl = [xTlB; yTlB] * CXS1B * [xTlB; yTlB]'
							
							RVLMULMX3X3TVECT(CXS1B, xTlB, M3x3Tmp)
							RVLMULMX3X3TVECT(CXS1B, yTlB, M3x3TmpRow2)
							CXS1Tl[0] = RVLDOTPRODUCT3(M3x3Tmp, xTlB);
							CXS1Tl[1] = RVLDOTPRODUCT3(M3x3Tmp, yTlB);
							CXS1Tl[3] = RVLDOTPRODUCT3(M3x3TmpRow2, yTlB);

							// CXS2Tl = [xTlB; yTlB] * CXS2B * [xTlB; yTlB]'

							RVLMULMX3X3TVECT(CXS2B, xTlB, M3x3Tmp)
							RVLMULMX3X3TVECT(CXS2B, yTlB, M3x3TmpRow2)
							CXS2Tl[0] = RVLDOTPRODUCT3(M3x3Tmp, xTlB);
							CXS2Tl[1] = RVLDOTPRODUCT3(M3x3Tmp, yTlB);
							CXS2Tl[3] = RVLDOTPRODUCT3(M3x3TmpRow2, yTlB);

							// CXS1Tl_ = (1 - sS1)^2 * CXS1Tl + sS1^2 * CXS2Tl

							fTmp *= fTmp;
							fTmp2 = sS1 * sS1;

							CXS1Tl_[0] = fTmp * CXS1Tl[0] + fTmp2 * CXS2Tl[0];
							CXS1Tl_[1] = fTmp * CXS1Tl[1] + fTmp2 * CXS2Tl[1];
							CXS1Tl_[3] = fTmp * CXS1Tl[3] + fTmp2 * CXS2Tl[3];

							//*

							//* crop point M1

							// XM1Tl_ = (1 - sM1) * XM1Tl + sM1 * XM2Tl

							fTmp = 1.0 - sM1;

							XM1Tl_[0] = fTmp * XM1Tl[0] + sM1 * XM2Tl[0];
							XM1Tl_[1] = fTmp * XM1Tl[1] + sM1 * XM2Tl[1];

							// CXM1Tl = [xTlB; yTlB] * CXM1 * [xTlB; yTlB]'

							CXM1 = pM3DLine->m_CX[0];

							RVLMULMX3X3TVECT(CXM1, xTlB, M3x3Tmp)
							RVLMULMX3X3TVECT(CXM1, yTlB, M3x3TmpRow2)
							CXM1Tl[0] = RVLDOTPRODUCT3(M3x3Tmp, xTlB);
							CXM1Tl[1] = RVLDOTPRODUCT3(M3x3Tmp, yTlB);
							CXM1Tl[3] = RVLDOTPRODUCT3(M3x3TmpRow2, yTlB);

							// CXM2Tl = [xTlB; yTlB] * CXM2 * [xTlB; yTlB]'

							CXM2 = pM3DLine->m_CX[1];

							RVLMULMX3X3TVECT(CXM2, xTlB, M3x3Tmp)
							RVLMULMX3X3TVECT(CXM2, yTlB, M3x3TmpRow2)
							CXM2Tl[0] = RVLDOTPRODUCT3(M3x3Tmp, xTlB);
							CXM2Tl[1] = RVLDOTPRODUCT3(M3x3Tmp, yTlB);
							CXM2Tl[3] = RVLDOTPRODUCT3(M3x3TmpRow2, yTlB);

							// CXM1Tl_ = (1 - sM1)^2 * CXM1Tl + sM1^2 * CXM2Tl

							fTmp *= fTmp;
							fTmp2 = sM1 * sM1;

							CXM1Tl_[0] = fTmp * CXM1Tl[0] + fTmp2 * CXM2Tl[0];
							CXM1Tl_[1] = fTmp * CXM1Tl[1] + fTmp2 * CXM2Tl[1];
							CXM1Tl_[3] = fTmp * CXM1Tl[3] + fTmp2 * CXM2Tl[3];

							//*

							// CPC <- contribution of the uncertainty of the estimated relative pose
							//        to the point 1 uncertainty in c.s. Tl

							RVLMULMX3X3VECT(R, XS1A_, V3x1Tmp)
							RVLMULMX3X3VECT(RBTl, V3x1Tmp, M3x3TmpRow3)
							fTmp2 = RVLDOTPRODUCT3(M3x3TmpRow3, M3x3TmpRow3);
							fTmp = sqrt(fTmp2);
							RVLSCALE3VECTOR2(M3x3TmpRow3, fTmp, M3x3TmpRow3)

							i = (M3x3TmpRow3[0] < M3x3TmpRow3[1] ? 0 : 1);
							if(M3x3TmpRow3[2] < M3x3TmpRow3[i])
								i = 2;
		
							RVLNULL3VECTOR(V3x1Tmp)
							V3x1Tmp[i] = 1.0;

							RVLCROSSPRODUCT3(M3x3TmpRow3, V3x1Tmp, M3x3TmpRow2)
							fTmp = sqrt(RVLDOTPRODUCT3(M3x3TmpRow2, M3x3TmpRow2));
							RVLSCALE3VECTOR2(M3x3TmpRow2, fTmp, M3x3TmpRow2)

							RVLCROSSPRODUCT3(M3x3TmpRow2, M3x3TmpRow3, M3x3Tmp)

							fTmp = LastDOFLineRotationVar * fTmp2;
							RVLSCALE3VECTOR(M3x3Tmp, fTmp, V3x1Tmp)
							RVLMULVECT3VECT3T(V3x1Tmp, M3x3Tmp, CPC1)
							RVLSCALE3VECTOR(M3x3TmpRow2, fTmp, V3x1Tmp)
							RVLMULVECT3VECT3T(V3x1Tmp, M3x3TmpRow2, M3x3Tmp2)
							RVLSUMMX3X3UT(CPC1, M3x3Tmp2, CPC1)

							CPC1[0] += LastDOFLineTranslationVar;
							CPC1[4] += LastDOFLineTranslationVar;
							
							// y1 match

							e1[1] = XS1Tl_[1] - XM1Tl_[1];

							C1[3] = CXS1Tl_[3] + CXM1Tl_[3];

							double varp1 = C1[3] + CPC1[4];

							ep1 = e1[1] * e1[1] / varp1;

							if(ep1 > CHI2INV_1D_P99)
								continue;

							//* crop point S2

							// XS2Tl_ = (1 - sS2) * XS1Tl + sS2 * XS2Tl

							fTmp = 1.0 - sS2;

							XS2Tl_[0] = fTmp * XS1Tl[0] + sS2 * XS2Tl[0];
							XS2Tl_[1] = fTmp * XS1Tl[1] + sS2 * XS2Tl[1];

							// XS2A_ = (1 - sS2) * XS1A + sS2 * XS2A

							RVLSCALE3VECTOR(XS1A, fTmp, XS2A_)
							RVLSCALE3VECTOR(XS2A, sS2, V3x1Tmp)
							RVLSUM3VECTORS(XS2A_, V3x1Tmp, XS2A_)

							// CXS2Tl_ = (1 - sS2)^2 * CXS1Tl + sS2^2 * CXS2Tl

							fTmp *= fTmp;
							fTmp2 = sS2 * sS2;

							CXS2Tl_[0] = fTmp * CXS1Tl[0] + fTmp2 * CXS2Tl[0];
							CXS2Tl_[1] = fTmp * CXS1Tl[1] + fTmp2 * CXS2Tl[1];
							CXS2Tl_[3] = fTmp * CXS1Tl[3] + fTmp2 * CXS2Tl[3];

							//*

							//* crop point M2

							// XM2Tl_ = (1 - sM2) * XM1Tl + sM2 * XM2Tl

							fTmp = 1.0 - sM2;

							XM2Tl_[0] = fTmp * XM1Tl[0] + sM2 * XM2Tl[0];
							XM2Tl_[1] = fTmp * XM1Tl[1] + sM2 * XM2Tl[1];

							// CXM2Tl_ = (1 - sM2)^2 * CXM1Tl + sM2^2 * CXM2Tl

							fTmp = 1.0 - sM2;
							fTmp *= fTmp;
							fTmp2 = sM2 * sM2;

							CXM2Tl_[0] = fTmp * CXM1Tl[0] + fTmp2 * CXM2Tl[0];
							CXM2Tl_[1] = fTmp * CXM1Tl[1] + fTmp2 * CXM2Tl[1];
							CXM2Tl_[3] = fTmp * CXM1Tl[3] + fTmp2 * CXM2Tl[3];

							//*

							// CPC <- contribution of the uncertainty of the estimated relative pose
							//        to the point 1 uncertainty in c.s. Tl

							RVLMULMX3X3VECT(R, XS2A_, V3x1Tmp)
							RVLMULMX3X3VECT(RBTl, V3x1Tmp, M3x3TmpRow3)
							fTmp2 = RVLDOTPRODUCT3(M3x3TmpRow3, M3x3TmpRow3);
							fTmp = sqrt(fTmp2);
							RVLSCALE3VECTOR2(M3x3TmpRow3, fTmp, M3x3TmpRow3)

							i = (M3x3TmpRow3[0] < M3x3TmpRow3[1] ? 0 : 1);
							if(M3x3TmpRow3[2] < M3x3TmpRow3[i])
								i = 2;
		
							RVLNULL3VECTOR(V3x1Tmp)
							V3x1Tmp[i] = 1.0;

							RVLCROSSPRODUCT3(M3x3TmpRow3, V3x1Tmp, M3x3TmpRow2)
							fTmp = sqrt(RVLDOTPRODUCT3(M3x3TmpRow2, M3x3TmpRow2));
							RVLSCALE3VECTOR2(M3x3TmpRow2, fTmp, M3x3TmpRow2)

							RVLCROSSPRODUCT3(M3x3TmpRow2, M3x3TmpRow3, M3x3Tmp)

							fTmp = LastDOFLineRotationVar * fTmp2;
							RVLSCALE3VECTOR(M3x3Tmp, fTmp, V3x1Tmp)
							RVLMULVECT3VECT3T(V3x1Tmp, M3x3Tmp, CPC2)
							RVLSCALE3VECTOR(M3x3TmpRow2, fTmp, V3x1Tmp)
							RVLMULVECT3VECT3T(V3x1Tmp, M3x3TmpRow2, M3x3Tmp2)
							RVLSUMMX3X3UT(CPC2, M3x3Tmp2, CPC2)

							CPC2[0] += LastDOFLineTranslationVar;
							CPC2[4] += LastDOFLineTranslationVar;
							
							// y2 match

							e2[1] = XS2Tl_[1] - XM2Tl_[1];

							C2[3] = CXS2Tl_[3] + CXM2Tl_[3];

							double varp2 = C2[3] + CPC2[4];

							ep2 = e2[1] * e2[1] / varp2;

							if(ep2 > CHI2INV_1D_P99)
								continue;

							//* compute the optimal translation

							// TraveledDist = ((e1'*inv(C1) + e2'*inv(C2))*[1 0]') / ([1 0] * (inv(C1) + inv(C2)) * [1 0]')
							// where e1 = [ex1 ey1]', e2 = [ex2 ey2]',  C1 = CXS1Tl_ + CXM1Tl_, C2 = CXS2Tl_ + CXM2Tl_

							ex1 = XS1Tl_[0] - XM1Tl_[0];

							e1[0] = ex1;

							C1[0] = CXS1Tl_[0] + CXM1Tl_[0];
							C1[1] = CXS1Tl_[1] + CXM1Tl_[1];

							detC1 = C1[0] * C1[3] - C1[1] * C1[1];
							
							fTmp = (C1[3] * e1[0] - C1[1] * e1[1]) / detC1;

							fTmp2 = C1[3] / detC1;

							ex2 = XS2Tl_[0] - XM2Tl_[0];

							e2[0] = ex2;

							C2[0] = CXS2Tl_[0] + CXM2Tl_[0];
							C2[1] = CXS2Tl_[1] + CXM2Tl_[1];

							detC2 = C2[0] * C2[3] - C2[1] * C2[1];
							
							fTmp += ((C2[3] * e2[0] - C2[1] * e2[1]) / detC2);

							fTmp2 += (C2[3] / detC2);

							varTD = 1.0 / fTmp2;
							
							TraveledDist = -fTmp * varTD;

							// chect TravelDist limits

							if(TraveledDist < -m_maxLastDOFTravelDist)
								continue;

							if(TraveledDist > m_maxLastDOFTravelDist)
								continue;

							// update maxvarTD

							if(varTD > maxvarTD)
								maxvarTD = varTD;

							// final match criterion: Mahalanobis distance of the endpoints of the cropped lines

							e1[0] += TraveledDist;

							C1[0] += CPC1[0];
							C1[1] += CPC1[1];
							C1[3] += CPC1[4];

							detC1 = C1[0] * C1[3] - C1[1] * C1[1];

							if(RVLMAHDIST2(e1, C1, detC1) > CHI2INV_2D_P99)
								continue;

							C2[0] += CPC2[0];
							C2[1] += CPC2[1];
							C2[3] += CPC2[4];

							detC2 = C2[0] * C2[3] - C2[1] * C2[1];

							e2[0] += TraveledDist;

							if(RVLMAHDIST2(e2, C2, detC2) > CHI2INV_2D_P99)
								continue;

							//** create line match

							pLineMatch->Flags = RVLPSULM_MSMATCH_DATA_FLAG_LINES;

							//* first point

							pLineMatch->pMData = (RVLPSULM_SURF_MATCH_DATA *)pLineMatchData;

							XSA_ = pLineMatchData->XSA;

							RVLCOPY3VECTOR(XS1A_, XSA_)
							
							// XMB_ = (1 - sM1) * XM1 + sM1 * XM2

							fTmp = 1.0 - sM1;

							XMB_ = pLineMatchData->XMB;
							
							RVLSCALE3VECTOR(XM1, fTmp, XMB_)
							RVLSCALE3VECTOR(XM2, sM1, V3x1Tmp)
							RVLSUM3VECTORS(XMB_, V3x1Tmp, XMB_)

							/////

							pLineMatchData->varxST = CXS1Tl_[0];
							pLineMatchData->varxMT = CXM1Tl_[0];
							pLineMatchData->p3DLine = pM3DLine;

							pLineMatchData++;

							//*

							//* second point

							pLineMatch->pSData = (RVLPSULM_SURF_MATCH_DATA *)pLineMatchData;

							XSA_ = pLineMatchData->XSA;

							RVLCOPY3VECTOR(XS2A_, XSA_)

							// XMB_ = (1 - sM2) * XM1 + sM2 * XM2

							fTmp = 1.0 - sM2;

							XMB_ = pLineMatchData->XMB;
							
							RVLSCALE3VECTOR(XM1, fTmp, XMB_)
							RVLSCALE3VECTOR(XM2, sM2, V3x1Tmp)
							RVLSUM3VECTORS(XMB_, V3x1Tmp, XMB_)

							/////

							pLineMatchData->varxST = CXS2Tl_[0];
							pLineMatchData->varxMT = CXM2Tl_[0];
							pLineMatchData->p3DLine = pS3DLine;

							pLineMatchData++;

							//*

							//if(HypothesisIndex == 133 && pS3DLine->m_Index == 7 && pM3DLine->m_Index == 15)
							//	int debug = 0;

							wTD = 1.0 / sqrt(2.0 * varTD);

							pLastDOFMatchData = LastDOFMatchArray + nLastDOFMatches;

							pLastDOFMatchData->pMatch = pLineMatch;

							pLineMatch++;

#ifdef RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_PROBABILISTIC
							// match reference frame

							double fTmp;

							double RCL[3*3];
							double tCL[3];
							double tLC[3];

							double *XLC = RCL;
							double *YLC = RCL + 3;
							double *ZLC = RCL + 6;

							RVLSUM3VECTORS(VSB, VM, ZLC)
							RVLNORM3(ZLC, fTmp)
							
							RVLCROSSPRODUCT3(VSB, VM, XLC)

							fTmp = RVLDOTPRODUCT3(XLC, XLC);

							if(fTmp <= APPROX_ZERO)
							{
								//double absZ[3];
								int i, j, k;
								//RVLORTHOGONAL3(ZLC, XLC, i, j, k, absZ, fTmp)
								RVLORTHOGONAL3(ZLC, XLC, i, j, k, fTmp)
							}
							else
							{
								fTmp = sqrt(fTmp);

								RVLSCALE3VECTOR2(XLC, fTmp, XLC)
							}

							RVLCROSSPRODUCT3(ZLC, XLC, YLC);

							// transform lines into the match reference frame

							double C1[3*3], C2[3*3], C1_[3*3], C2_[3*3];

							RVLCOV3DTRANSF(CXS1B, RCL, C1, Mx3x3Tmp)
							RVLCOV3DTRANSF(CXS2B, RCL, C2, Mx3x3Tmp)
							RVLCOV3DTRANSF(CXM1, RCL, C1_, Mx3x3Tmp)
							RVLCOV3DTRANSF(CXM2, RCL, C2_, Mx3x3Tmp)

							// matching of the central points of the overlapping segments

							double C1o[2*2], C2o[2*2], C1o_[2*2], C2o_[2*2];
							double varz1o, varz2o, varz1o_, varz2o_;

							C1o[0] = C1[0]; C1o[1] = C1[1];	C1o[3] = C1[4]; varz1o = C1[8];
							C2o[0] = C2[0]; C2o[1] = C2[1];	C2o[3] = C2[4]; varz2o = C2[8];
							C1o_[0] = C1_[0]; C1o_[1] = C1_[1];	C1o_[3] = C1_[4]; varz1o_ = C1_[8];
							C2o_[0] = C2_[0]; C2o_[1] = C2_[1];	C2o_[3] = C2_[4]; varz2o_ = C2_[8];

							double Cu[2*2], Cu_[2*2];
							double eu, Pu;

							if(!pS3DLine->ComputeOrientUncert(C1o, C2o, varz1o, varz2o, pS3DLineData->len, Cu))
								Pu = 0.0;
							else if(!pM3DLine->ComputeOrientUncert(C1o_, C2o_, varz1o_, varz2o_, pM3DLineData->len, Cu_))
								Pu = 0.0;
							else
							{
								double CuS[2*2];

								CuS[0] = Cu[0] + Cu_[0] + LastDOFLineRotationVar;
								CuS[1] = Cu[1] + Cu_[1];
								CuS[3] = Cu[3] + Cu_[3] + LastDOFLineRotationVar;

								double detCuS = RVLDET2(CuS);

								if(detCuS > 4.0)
									Pu = 0.0;
								else
								{	
									double uy = RVLDOTPRODUCT3(VSB, YLC);

									eu = CuS[0] * 4.0 * uy * uy / detCuS;

									Pu = RVLLN4PI - 0.5*(log(detCuS) + eu) - RVLLN2PI;

									if(Pu < 0.0)
										Pu = 0.0;
								}
							}

							double ep = 0.5 * (ep1 + ep2);
							double varp = 0.5 * (varp1 + varp2);
							double Pp =  - 0.5 * ((log(varp) + ep + RVLLN2PI));

							pLastDOFMatchData->Support = m_LineMatchData.PPriorPosition + log(wTD) - 0.5 * RVLLNPI + Pp + Pu;
#else
							pLastDOFMatchData->Support = wTD * overlap * SSupport;
#endif

							pLastDOFMatchData->TraveledDist = TraveledDist;

							pLastDOFMatchData->wTD = wTD;

							nLastDOFMatches++;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
							fprintf(fpLog, "ML%d-SL%d\tTD=%lf\tS=%lf\n", pM3DLine->m_Index, pS3DLine->m_Index, TraveledDist, 
								pLastDOFMatchData->Support);
#endif
						}	// for each M3DLine
					}	// for each S3DLine
#endif

					if(nLastDOFMatches > 0)
					{
#ifdef RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_BY_EVIDENCE_ACCU
						// determine the number of scales

						nLastDOFScales = (int)floor(log(kLastDOFScale * sqrt(maxvarTD)) / RVLLOG2) + 1;

						if(nLastDOFScales < 1)
							nLastDOFScales = 1;
						//else if(nLastDOFScales > 1)
						//	int debug = 0;

						// accumulate evidence at different scales

						LastDOFAccu = new double[2 * nLastDOFBins];

						//while(TRUE)
						//{
						memset(LastDOFAccu, 0, 2 * nLastDOFBins * sizeof(double));

						pLastDOFMatchArrayEnd = LastDOFMatchArray + nLastDOFMatches;

						for(pLastDOFMatchData = LastDOFMatchArray; pLastDOFMatchData < pLastDOFMatchArrayEnd; pLastDOFMatchData++)
						{
							support = pLastDOFMatchData->Support;

							if(support < 0.0)
								continue;

							wTD = pLastDOFMatchData->wTD;

							stdTD = 1.0 / wTD / SQRT2;

							TraveledDist = pLastDOFMatchData->TraveledDist;

							iLastDOFScale = (int)floor(log(kLastDOFScale * stdTD) / RVLLOG2);

							if(iLastDOFScale < 0)
								iLastDOFScale = 0;

							nLastDOFBins_ = (nLastDOFBins >> iLastDOFScale);
		
							LastDOFAccu_ = (iLastDOFScale > 0 ? LastDOFAccu + 2 * (nLastDOFBins - nLastDOFBins_) : LastDOFAccu);

							LastDOFBinSize = m_LastDOFResolution * (double)(1 << iLastDOFScale);
						
							kLastDOFScale2 = 0.5 * (double)(nLastDOFBins_ - 1);

							iLastDOFBin1 = RVLMAX(0, DOUBLE2INT((TraveledDist - 2.5 * stdTD) / LastDOFBinSize + kLastDOFScale2));
							iLastDOFBin2 = RVLMIN(2.0 * kLastDOFScale2, DOUBLE2INT((TraveledDist + 2.5 * stdTD) / LastDOFBinSize + kLastDOFScale2));

							for(iLastDOFBin = iLastDOFBin1; iLastDOFBin < iLastDOFBin2; iLastDOFBin++)
							{
								eTD = wTD * (LastDOFBinSize * ((double)iLastDOFBin - kLastDOFScale2) - TraveledDist);
#ifdef RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_PROBABILISTIC
								double dBin = support - eTD * eTD;

								if(dBin < 0.0)
									dBin = 0.0;

								LastDOFAccu_[iLastDOFBin] += dBin;
#else
								LastDOFAccu_[iLastDOFBin] += (support * exp(-eTD * eTD));
#endif
							}
						}

						// sum accumulators at different scales

#ifdef RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_ACCU_DEBUG_LOG
						FILE *fpLastDOFAccuLog = fopen("C:\\RVL\\Debug\\LastDOFAccu.dat", "w");

						int iLastDOFAccuLog;
#endif

						nLastDOFBins_ = (nLastDOFBins >> (nLastDOFScales - 1));

						for(iLastDOFScale = nLastDOFScales - 1; iLastDOFScale >= 1; iLastDOFScale--, nLastDOFBins_ = (nLastDOFBins_ << 1))
						{
							LastDOFAccu_ = LastDOFAccu + 2 * (nLastDOFBins - nLastDOFBins_);

							pLastDOFAccuBelowBin = LastDOFAccu_ - 2 * nLastDOFBins_;

							pLastDOFAccuEnd = LastDOFAccu_ + nLastDOFBins_;

							for(pLastDOFAccuBin = LastDOFAccu_; pLastDOFAccuBin < pLastDOFAccuEnd; pLastDOFAccuBin++)
							{
								*(pLastDOFAccuBelowBin++) += (*pLastDOFAccuBin);
								*(pLastDOFAccuBelowBin++) += (*pLastDOFAccuBin);

//#ifdef RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_ACCU_DEBUG_LOG
//								for(iLastDOFAccuLog = 0; iLastDOFAccuLog < (1 << iLastDOFScale); iLastDOFAccuLog++)
//									fprintf(fpLastDOFAccuLog, "%lf ", *pLastDOFAccuBin);
//
//								fprintf(fpLastDOFAccuLog, "\n");
//#endif
							}
						}

#ifdef RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_ACCU_DEBUG_LOG
						kLastDOFScale2 = 0.5 * (double)(nLastDOFBins - 1);

						for(iLastDOFBin = 0; iLastDOFBin < nLastDOFBins; iLastDOFBin++)
							fprintf(fpLastDOFAccuLog, "%lf\t%lf\n", 
								m_LastDOFResolution * ((double)iLastDOFBin - kLastDOFScale2), 
								LastDOFAccu[iLastDOFBin]);

						fclose(fpLastDOFAccuLog);
#endif
						double TraveledDistOffset = 0.5 * (double)(nLastDOFBins - 1);

						DWORD LastDOFEstimationMethod = (m_Flags & RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD);

						double *LastDOFPeak;
						double *pLastDOFPeak;

						if(LastDOFEstimationMethod == RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_ALL_PEAKS ||
							LastDOFEstimationMethod == RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_BEST_PEAK_TREE)
						{
							//***** multiple local maxima 

							LastDOFPeak = new double[nLastDOFBins];

							pLastDOFPeak = LastDOFPeak;

							int nLastDOFBins_ = nLastDOFBins - 1;

							TraveledDist = m_LastDOFResolution * (1.0 - TraveledDistOffset);

							int nPeak;
							double SPeak;

							for(i = 1; i < nLastDOFBins_; i++, TraveledDist += m_LastDOFResolution)
							{
								if(LastDOFAccu[i] < LastDOFAccu[i - 1])
								{
									if(nPeak > 0)
									{
										*(pLastDOFPeak++) = SPeak / (double)nPeak;

										nPeak = 0;
									}
								}
								else if(LastDOFAccu[i] < LastDOFAccu[i + 1])
									nPeak = 0;
								else if(LastDOFAccu[i] > LastDOFAccu[i - 1] && LastDOFAccu[i] >= LastDOFAccu[i + 1])
								{
									nPeak = 1;
									SPeak = TraveledDist;
								}
								else
								{
									nPeak++;
									SPeak += TraveledDist;
								}
							}
						}
						else if(LastDOFEstimationMethod == RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_MAX_PEAK_ONLY)
						{
							//***** find a single peak in the accumulator array

							LastDOFPeak = new double[1];

							pBestLastDOFAccuBin = LastDOFAccu;

							pLastDOFAccuEnd = LastDOFAccu + nLastDOFBins;
						
							for(pLastDOFAccuBin = LastDOFAccu + 1; pLastDOFAccuBin < pLastDOFAccuEnd; pLastDOFAccuBin++)
								if(*pLastDOFAccuBin > *pBestLastDOFAccuBin)
									pBestLastDOFAccuBin = pLastDOFAccuBin;

							pLastDOFPeak = LastDOFPeak;

							*(pLastDOFPeak++) = m_LastDOFResolution * (pBestLastDOFAccuBin - LastDOFAccu - TraveledDistOffset);
						}		

						double *pLastDOFPeakEnd = pLastDOFPeak;

						double TraveledDist_;
						double P;

						if(LastDOFEstimationMethod == RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_BEST_PEAK_TREE)
						{
							// ***** for each local maximum construct a conditional probability tree and detect the most probable peak

							int nSFeatures = nS3DSurfaces + nS3DLines;

							int *iLastDOFSFeatureArray = new int[nSFeatures];

							int *LastDOFSFeatureIndexArray = new int[nSFeatures];

							memset(LastDOFSFeatureIndexArray, 0xff, nSFeatures * sizeof(int));

							int nLastDOFSFeatures = 0;
							int nLastDOFSSurfs = 0;

							if(m_SMatchArray)
								delete[] m_SMatchArray;

							m_SMatchArray = new RVLPSULM_SMATCH_DATA[nSFeatures];							

							int SFeatureIndex, SFeatureIndex_;
							bool bSurface;
							RVLPSULM_SMATCH_DATA *pSMatchData;

							for(pLastDOFMatchData = LastDOFMatchArray; pLastDOFMatchData < pLastDOFMatchArrayEnd; pLastDOFMatchData++)
							{
								support = pLastDOFMatchData->Support;

								if(support < 0.0)
									continue;

								pMSMatch = pLastDOFMatchData->pMatch;

								if(bSurface = (pLastDOFMatchData - LastDOFMatchArray < nSMSurfMatches))
									SFeatureIndex = SFeatureIndex_ = ((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Index;
								else
								{
									SFeatureIndex = ((RVLPSULM_LINE_MATCH_DATA *)(pMSMatch->pSData))->p3DLine->m_Index;

									SFeatureIndex_ = SFeatureIndex + nS3DSurfaces;
								}
								
								if(LastDOFSFeatureIndexArray[SFeatureIndex_] < 0)
								{
									LastDOFSFeatureIndexArray[SFeatureIndex_] = nLastDOFSFeatures;

									iLastDOFSFeatureArray[nLastDOFSFeatures] = SFeatureIndex;

									nLastDOFSFeatures++;

									if(bSurface)
										nLastDOFSSurfs++;
								}
							}

							RVLPSULM_MATCH2 **pAutoMatchArrayEnd = m_AutoMatchArray + m_nAutoMatches; 

							RVLPSULM_MATCH2 **AutoMatchArray = new RVLPSULM_MATCH2 *[m_nAutoMatches + 1];
					
							RVLPSULM_MATCH2 **ppAutoMatch_ = AutoMatchArray;

							RVLPSULM_MATCH2 **ppAutoMatch;
							RVLPSULM_MATCH2 *pAutoMatch;
							int iSFeature, iSFeature_;

							for(ppAutoMatch = m_AutoMatchArray; ppAutoMatch < pAutoMatchArrayEnd; ppAutoMatch++)
							{
								pAutoMatch = *ppAutoMatch;

								if(pAutoMatch->Type == RVLPSULM_MATCH_TYPE_LINE)
								{
									iSFeature = ((CRVL3DLine2 *)(pAutoMatch->vpSObject))->m_Index + nS3DSurfaces;
									iSFeature_ = ((CRVL3DLine2 *)(pAutoMatch->vpMObject))->m_Index + nS3DSurfaces;
								}
								else
								{
									iSFeature = ((CRVL3DSurface2 *)(pAutoMatch->vpSObject))->m_Index;
									iSFeature_ = ((CRVL3DSurface2 *)(pAutoMatch->vpMObject))->m_Index;
								}

								if(LastDOFSFeatureIndexArray[iSFeature] >= 0 && LastDOFSFeatureIndexArray[iSFeature_] >= 0)
									*(ppAutoMatch_++) = pAutoMatch;
							}

							int nAutoMatches = ppAutoMatch_ - AutoMatchArray;

							delete[] LastDOFSFeatureIndexArray;

							double maxP = 0.0;
							double HighestLastDOFPeak;

							for(pLastDOFPeak = LastDOFPeak; pLastDOFPeak < pLastDOFPeakEnd; pLastDOFPeak++)
							{
								memset(m_SMatchArray, 0, nSFeatures * sizeof(RVLPSULM_SMATCH_DATA));

								TraveledDist = *pLastDOFPeak;

								for(pLastDOFMatchData = LastDOFMatchArray; pLastDOFMatchData < pLastDOFMatchArrayEnd; pLastDOFMatchData++)
								{
									support = pLastDOFMatchData->Support;

									if(support < 0.0)
										continue;

									wTD = pLastDOFMatchData->wTD;

									stdTD = 1.0 / wTD / SQRT2;

									TraveledDist_ = pLastDOFMatchData->TraveledDist;

									eTD = wTD * (TraveledDist_ - TraveledDist);

									P = support - eTD * eTD;

									if(P <= 0.0)
										continue;

									pMSMatch = pLastDOFMatchData->pMatch;

									bSurface = (pLastDOFMatchData - LastDOFMatchArray < nSMSurfMatches);

									SFeatureIndex = (bSurface ? ((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Index : 
										((RVLPSULM_LINE_MATCH_DATA *)(pMSMatch->pSData))->p3DLine->m_Index + nS3DSurfaces);

									pSMatchData = m_SMatchArray + SFeatureIndex;

									if(P > pSMatchData->P)
									{
										pSMatchData->b = true;
										pSMatchData->P = P;
										pSMatchData->iMFeature = (bSurface ? ((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Index : 
											((RVLPSULM_LINE_MATCH_DATA *)(pMSMatch->pMData))->p3DLine->m_Index);
									}
								}

								P = ConditionalProbabilityTree(pSPSuLM, pMPSuLM, iLastDOFSFeatureArray, nLastDOFSSurfs, nLastDOFSFeatures - nLastDOFSSurfs,
									AutoMatchArray, nAutoMatches);

								if(P > maxP)
								{
									maxP = P;
									HighestLastDOFPeak = TraveledDist;
								}
							}

							delete[] iLastDOFSFeatureArray;		
							delete[] AutoMatchArray;

							LastDOFPeak[0] = HighestLastDOFPeak;

							pLastDOFPeakEnd = LastDOFPeak + 1;
						}	// if(LastDOFEstimationMethod == RVLPSULMBUILDER_FLAG_LAST_DOF_ESTIMATION_METHOD_BEST_PEAK_TREE)
					
						for(pLastDOFPeak = LastDOFPeak; pLastDOFPeak < pLastDOFPeakEnd; pLastDOFPeak++)
						{
							TraveledDist = *pLastDOFPeak;

							// compute the scores of all matches

							for(pLastDOFMatchData = LastDOFMatchArray; pLastDOFMatchData < pLastDOFMatchArrayEnd; pLastDOFMatchData++)
							{
								eTD = pLastDOFMatchData->wTD * (TraveledDist - pLastDOFMatchData->TraveledDist);

								pLastDOFMatchData->score = pLastDOFMatchData->Support * exp(-eTD * eTD);
							}

							/////

#else

							pLastDOFMatchArrayEnd = LastDOFMatchArray + nLastDOFMatches;

							for(pLastDOFMatchData = LastDOFMatchArray; pLastDOFMatchData < pLastDOFMatchArrayEnd; pLastDOFMatchData++)
							{
								TDScore = 0.0;

								for(pLastDOFMatchData2 = LastDOFMatchArray; pLastDOFMatchData2 < pLastDOFMatchArrayEnd; pLastDOFMatchData2++)
								{
									eTD = pLastDOFMatchData2->wTD * (pLastDOFMatchData->TraveledDist - pLastDOFMatchData2->TraveledDist);

									TDScore += (pLastDOFMatchData2->Support * exp(-eTD * eTD));
								}

								pLastDOFMatchData->score = TDScore;
							}
#endif

							//fclose(fpLastDOF);

							// find the best Last DOF match, use it to complete a hypothesis 
							// and insert this hypothesis into m_HypothesisList

							//while(TRUE)
							//{
							BestTDScore = 0.0;

							pBestLastDOFMatch = NULL;

							for(pLastDOFMatchData = LastDOFMatchArray; pLastDOFMatchData < pLastDOFMatchArrayEnd; pLastDOFMatchData++)
							{
								if(pLastDOFMatchData->score > BestTDScore)
								{
									BestTDScore = pLastDOFMatchData->score;

									pBestLastDOFMatch = pLastDOFMatchData;
								}
							}

							if(pBestLastDOFMatch == NULL)
								break;

							pMSMatch = pBestLastDOFMatch->pMatch;

							if(pMSMatch == NULL)
								break;

							//if (HypothesisIndex == 2130)
							//	int debug = 0;

							if(pHypothesis == NULL)
							{
								RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);

								pPoseSM2 = &(pHypothesis->PoseSM);

								pPoseSM2->m_C = pHypothesis->P;
								pPoseSM2->m_pData = pHypothesis->invt;
							}
							else
								pPoseSM2 = &(pHypothesis->PoseSM);

							if(pMSMatch->Flags & RVLPSULM_MSMATCH_DATA_FLAG_LINES)
							{
								pLineMatchData = (RVLPSULM_LINE_MATCH_DATA *)(pMSMatch->pSData);

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
								pS3DLine = pLineMatchData->p3DLine;
#endif
								
								RVL3DLineEKFUpdate(pLineMatchData->XSA, pLineMatchData->varxST, pLineMatchData->XMB, 
									pLineMatchData->varxMT, pPoseSM, xTlB, pPoseSM2);

								pLineMatchData = (RVLPSULM_LINE_MATCH_DATA *)(pMSMatch->pMData);
								
								RVL3DLineEKFUpdate(pLineMatchData->XSA, pLineMatchData->varxST, pLineMatchData->XMB, 
									pLineMatchData->varxMT, pPoseSM2, xTlB, pPoseSM2);

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
								pM3DLine = pLineMatchData->p3DLine;

								fprintf(fpLog, "Last DOF match: ML%d-SL%d\n", pM3DLine->m_Index, pS3DLine->m_Index);
#endif
							}
							else
							{
								pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
								pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);
	
#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
								fprintf(fpLog, "Last DOF match: M%d-S%d\n", pM3DSurface->m_Index, pS3DSurface->m_Index);
#endif

								if(!RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, pPoseSM, pPoseSM2, &MatchData, false))
								{
									//pBestLastDOFMatch->Support = -1.0;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
									fprintf(fpLog, "The match doesn't satisfy the geometric criterion!\n\n");
#endif
								}
								else
								{
									RVLGetMaxEigVector3(pPoseSM2->m_C + 2 * 3 * 3, eig, bReal, Veig);

									if(eig[2] > m_maxvartHyp)
									{
										//pBestLastDOFMatch->Support = -1.0;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
										fprintf(fpLog, "Estimated pose uncertainty is too high!\n\n");
#endif
									}
								}
							}

							//if(pBestLastDOFMatch->Support >= 0.0)
							//{
								//if(HypothesisIndex == 12)
								//	int debug = 0;

								//R2 = pPoseSM2->m_Rot;
								//t2 = pPoseSM2->m_X;
								//RVLCOPY3VECTOR(t, t2);
								//R2 = pPoseSM2->m_Rot;
								//RVLCOPYMX3X3(R, R2);
								//P3 = pPoseSM2->m_C;
								//P = pPoseSM->m_C;
								//RVLCOPYMX3X3(P, P3);
								//P += (3 * 3);
								//P3 += (3 * 3);
								//RVLCOPYMX3X3(P, P3);
								//P += (3 * 3);
								//P3 += (3 * 3);
								//RVLCOPYMX3X3(P, P3);
								//invt = (double *)(pPoseSM->m_pData);
								//invt2 = (double *)(pPoseSM2->m_pData);
								//RVLCOPY3VECTOR(invt, invt2);
								//pPoseSM2->m_Alpha = pPoseSM->m_Alpha;
								//pPoseSM2->m_Beta = pPoseSM->m_Beta;
								//pPoseSM2->m_Theta = pPoseSM->m_Theta;
								//pPoseSM2->m_ca = pPoseSM->m_ca;
								//pPoseSM2->m_sa = pPoseSM->m_sa;
								pHypothesis->Index = HypothesisIndex;
								pHypothesis->PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
								pHypothesis->cost = 0;
								pHypothesis->visible = 0;
								pHypothesis->iS = pHypothesis->iM = 0;
								pHypothesis->pMPSuLM = pMPSuLM;
								pHypothesis->Flags = 0x00;
								RVLCOPY3VECTOR(zTB, pHypothesis->zTB)

//								if((m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) == 
//									RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
//								{
//									EvaluateHypothesis3(pSPSuLM, pHypothesis);
//
//									pNode2 = pNode;
//
//									while(pNode2)
//									{
//										pMSMatch = MatchList + pNode2->iMatch;
//
//										pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);
//
//										if(pS3DSurface->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
//										{
//#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
//											fprintf(fpLog, "S%d removed!\n\n", pS3DSurface->m_Index);
//#endif
//
//											pBestLastDOFMatch->Support = -1.0;
//
//											break;
//										}
//
//										pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
//
//										if(pM3DSurface->m_Flags & RVL3DSURFACE_FLAG_REMOVED)
//										{
//#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
//											fprintf(fpLog, "M%d removed!\n\n", pS3DSurface->m_Index);
//#endif
//
//											pBestLastDOFMatch->Support = -1.0;
//
//											break;
//										}
//
//										pNode2 = pNode2->pParent;
//									}								
//								}
							//}

							//if(pBestLastDOFMatch->Support < 0.0)
							//	continue;

							pHypothesis->nMatches = pNode->g;

							RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, CRVL3DSurface2 *, 2 * pHypothesis->nMatches, pHypothesis->MatchArray);

							ppMatchedSSurf = (CRVL3DSurface2 **)(pHypothesis->MatchArray);
							ppMatchedMSurf = ppMatchedSSurf + pHypothesis->nMatches;

							pNode2 = pNode;

							while(pNode2)
							{
								pMSMatch = MatchList + pNode2->iMatch;

								*(ppMatchedSSurf++) = (CRVL3DSurface2 *)(pMSMatch->pSData);
								*(ppMatchedMSurf++) = (CRVL3DSurface2 *)(pMSMatch->pMData);

								pNode2 = pNode2->pParent;
							}

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
							fprintf(fpLog, "Hypothesis %d: R=(%lf, %lf, %lf) t=(%lf, %lf, %lf)\n", 
								HypothesisIndex,
								pPoseSM2->m_Alpha * RAD2DEG,
								pPoseSM2->m_Beta * RAD2DEG,
								pPoseSM2->m_Theta * RAD2DEG,
								pPoseSM2->m_X[0],
								pPoseSM2->m_X[1],
								pPoseSM2->m_X[2]);

							RVLPrintCov(fpLog, pPoseSM2->m_C, 3, RAD2DEG);

							RVLPrintCov(fpLog, pPoseSM2->m_C + 2 * 3 * 3, 3);

							fprintf(fpLog, "\n");							
#endif

							HypothesisIndex++;

							nFeatures++;

							m_HypothesisList.Add(pHypothesis);	

							*(ppHypothesis++) = pHypothesis;

							pHypothesis = NULL;

							iHypothesis++;
						}	// for each Last DOF peak

#ifdef RVLPSULMBUILDER_HYPOTHESES_LAST_DOF_BY_EVIDENCE_ACCU
						delete[] LastDOFAccu;
						delete[] LastDOFPeak;
#endif
					}	// if(nLastDOFMatches > 0)
					else
					{
#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
						fprintf(fpLog, "No appropriate last DOF matches!\n\n");
#endif		
					}

					delete[] LastDOFMatchArray;

#ifdef RVLPSULM_LINES
					delete[] LineMatchArray;
					delete[] LineMatchData;
#endif
#pragma endregion
				}	// if(bNewHypothesis)
			}	// if pNode satisfies the geometric criterion
			else if(pPNode)
				pPNode->nFailures++;

			//pNode = (RVLPSULM_HG_NODE *)(pNode->pNext);

			//if(pNode == NULL)
			//{
			//	do	
			//		minCost++;
			//	while(QueueListArray[minCost].pFirst == NULL);

			//	pNode = (RVLPSULM_HG_NODE *)(QueueListArray[minCost].pFirst);
			//}
		}	// for each hypothesis	

		double ExecutionTime = m_pTimer->GetTime() - StartTime;

   		if(pPrevSPSuLM)
			break;
//#ifdef RVLPSULM_LINES
//		delete[] M3DLineArrayLastDOF;
//#endif
	}	// for each model PSuLM	

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
	fclose(fpLog);
#endif

	//delete[] MatchList;
	//delete[] NodeMem;
	//delete[] HypothesisArray;

	m_pMem2->Clear();

	// only for debugging purpose

//#ifndef RVLPSULMBUILDER_HYPOTHESES_DEBUG
//	CRVLFigure *pFig;
//
//	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData;
//#endif
//
//	m_HypothesisList.Start();
//
//	while(m_HypothesisList.m_pNext)
//	{
//		pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());
//
//		if(pHypothesis->Index == 0)
//		{
//			pSPSuLM->Display(m_DebugData.pGUI, &(pHypothesis->PoseSM));
//
//			pHypothesis->pMPSuLM->AddToFigure(m_DebugData.pGUI);
//
//			pFig = m_DebugData.pGUI->OpenFigure("PSuLM");
//
//			pMouseCallbackData = (RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);
//
//			pMouseCallbackData->MatrixSceneModel = NULL;	
//
//			//pMouseCallbackData->MSurfMatchData = MSurfMatchData;
//			//pMouseCallbackData->SSurfMatchData = SSurfMatchData;
//
//			pMouseCallbackData->pSSurf = NULL;
//			pMouseCallbackData->pMSurf = NULL;
//		}
//		else
//		{
//			pMouseCallbackData->pMPSuLM = pHypothesis->pMPSuLM;
//		
//			memcpy(pMouseCallbackData->PoseS0.m_Rot, pHypothesis->PoseSM.m_Rot, 3 * 3 * sizeof(double));
//			memcpy(pMouseCallbackData->PoseS0.m_X, pHypothesis->PoseSM.m_X, 3 * sizeof(double));
//
//			pMouseCallbackData->Flags = RVLPSULM_DISPLAY_SURFACES;
//
//			RVLPSuLMDisplay(pFig);
//		}
//
//		cvWaitKey();
//	}
}


// Hypothesis generation based on GCRANSAC

void CRVLPSuLMBuilder::Hypotheses4(	CRVLPSuLM *pSPSuLM,
									CRVL3DPose *pPoseS0Init,
									double *PInit,
									CRVLPSuLM * pPrevSPSuLM)
{
	int nHypotheses = m_maxnHypothesesPerModel;

	int maxnM3DSurfaces = 1000;

	int maxnFailures = 5;

	m_HypothesisList.RemoveAll();

	int nS3DSurfaces = pSPSuLM->m_n3DSurfaces;

	if(nS3DSurfaces == 0)
		return;

	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;

	RVLSURFACE_MATCH_ARRAY *SSurfMatchData;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSURFACE_MATCH_ARRAY, nS3DSurfaces, SSurfMatchData);

	RVLSURFACE_MATCH_ARRAY *pSSurfMatchData = SSurfMatchData;

	int iSSurf;

	for(iSSurf = 0; iSSurf < nS3DSurfaces; iSSurf++, pSSurfMatchData++)
		pSSurfMatchData->pSObject = SSurfArray[iSSurf];
	
	RVLSURFACE_MATCH_ARRAY *SSurfMatchBuff;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSURFACE_MATCH_ARRAY, nS3DSurfaces, SSurfMatchBuff);

	RVLSURFACE_MATCH *MatchArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSURFACE_MATCH, nS3DSurfaces * maxnM3DSurfaces, MatchArray);

	RVLSURFACE_MATCH *MatchBuff;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLSURFACE_MATCH, nS3DSurfaces * maxnM3DSurfaces, MatchBuff);

	CRVL3DSurface2 **MatchedMSurfArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, CRVL3DSurface2 *, maxnM3DSurfaces, MatchedMSurfArray);

	CRVL3DSurface2 **MatchedSSurfArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, CRVL3DSurface2 *, nS3DSurfaces, MatchedSSurfArray);

	CRVL3DSurface2 *pS3DSurface;

	CRVL3DPose PoseSMInit;

	PoseSMInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	double CInit[3 * 3 * 3];
	double invtInit[3];
	PoseSMInit.m_C = CInit;
	PoseSMInit.Copy(pPoseS0Init);
	PoseSMInit.m_pData = invtInit;
	double *R = PoseSMInit.m_Rot;
	double *t = PoseSMInit.m_X;
	RVLMULMX3X3TVECT(R, t, invtInit);

	CRVL3DPose PoseSM;

	PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	double P[3 * 3 * 3];
	double invt[3];
	PoseSM.m_C = P;
	PoseSM.m_pData = invt;

	CRVLPSuLM *pMPSuLM;
	CRVL3DSurface2 *pM3DSurface;
	CRVL3DSurface2 **MSurfArray;
	int iHypothesis;
	double MatchQuality;
	double detQ;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
	pSPSuLM->Display(m_DebugData.pGUI, pPoseS0Init);

	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("PSuLM");

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	pMouseCallbackData->MatrixSceneModel = NULL;	

	//pMouseCallbackData->MSurfMatchData = MSurfMatchData;
	//pMouseCallbackData->SSurfMatchData = SSurfMatchData;

	CRVLFigure *pFig2 = m_DebugData.pGUI->OpenFigure("PSuLM2");	

	pFig->m_FontSize = pFig2->m_FontSize = 16;

	int nTextLines = 22;

	int TextImageHeight = nTextLines * pFig2->m_FontSize;

	pFig2->EmptyBitmap(cvSize(800, TextImageHeight), cvScalar(255, 255, 255));

	cvInitFont(&(pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	cvInitFont(&(pFig2->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	char Text[201];

	pFig2->PutText("Hello world!", cvPoint(0, pFig2->m_FontSize), cvScalar(0, 0, 0));

	m_DebugData.pGUI->ShowFigure(pFig2);
#endif

	RVLPSULM_HYPOTHESIS **HypothesisArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_HYPOTHESIS *, nHypotheses, HypothesisArray);

	int HypothesisIndex = 0;

	int nM3DSurfaces;
	RVLSURFACE_MATCH *pMSMatch, *pMSMatch2, *pFirstMSMatch;
	RVLPSULM_HYPOTHESIS *pHypothesis;
	double *R2, *t2, *P3, *Pt, *P4;
	RVLSURFACE_MATCH_ARRAY MatchData;
	double *invt2;
	int iMSurf;
	int iLastMSMatch;
	RVLPSULM_HYPOTHESIS **ppHypothesis;
	BOOL bHypothesisExists;
	double err;
	BOOL bNewHypothesis;
	int nMatches;
	int iMatch;
	int nFailures;
	int iLastSSurf;
	CRVL3DSurface2 **pMatchedMSurf, **pMatchedMSurfArrayEnd;
	CRVL3DSurface2 **pMatchedSSurf;
	int i;
	CRVL3DPose *pPoseSM2;

	double *e = MatchData.m_e;
	
	// initial feature matching 

	m_PSuLMList.Start();

	while(TRUE)
	{
		double StartTime = m_pTimer->GetTime();

		if(pPrevSPSuLM)
			pMPSuLM = pPrevSPSuLM;
		else if(m_PSuLMList.m_pNext)
			pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());
		else
			break;

		//if(pMPSuLM->m_Index != 13)	// debug
		//	continue;

		MSurfArray = pMPSuLM->m_3DSurfaceArray;

		nM3DSurfaces = pMPSuLM->m_n3DSurfaces;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
		pMPSuLM->AddToFigure(m_DebugData.pGUI);

		//pMouseCallbackData->nMSurfs = pM3DSurfaceList->m_nElements;

		cvWaitKey();
#endif
		// create match array

		pSSurfMatchData = SSurfMatchData;

		pMSMatch = MatchArray;

		pMSMatch2 = MatchBuff;

		for(iSSurf = 0; iSSurf < nS3DSurfaces; iSSurf++, pSSurfMatchData++)
		{
			pSSurfMatchData->pFirst = pMSMatch2;

			pFirstMSMatch = pMSMatch;

			pS3DSurface = SSurfArray[iSSurf];

			for(iMSurf = 0; iMSurf < nM3DSurfaces; iMSurf++)
			{
				pM3DSurface = MSurfArray[iMSurf];	

				if(!pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, detQ, &MatchData))
					continue;

				pMSMatch->pMObject = pM3DSurface;

				pMSMatch++;
			}

			pMSMatch2 += (pMSMatch - pFirstMSMatch);

			pSSurfMatchData->pLast = pMSMatch2 - 1;
		}

		nMatches = pMSMatch - MatchArray;

//#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
//
//		FILE *fp;
//
//		fopen_s(&fp, "C:\\RVL\\ExpRez\\PSuLMBuilderHypGen.dat", "w");
//
//		fprintf(fp, "Match List:\n");
//
//		for(pMSMatch = MatchList; pMSMatch < pMSMatchListEnd; pMSMatch++)
//		{
//			pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
//			pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);
//
//			if(pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, &MatchData))
//				fprintf(fp, "S%d-M%d\t\t%d\n", 
//					((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Index,
//					((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Index,
//					pMSMatch->Support);
//		}
//
//		fclose(fp);
//
//#endif
		//*** generate hypotheses

		iHypothesis = 0;

		ppHypothesis = HypothesisArray;

		while(iHypothesis < nHypotheses)
		{
			// restore match array

			memcpy(SSurfMatchBuff, SSurfMatchData, nS3DSurfaces * sizeof(RVLSURFACE_MATCH_ARRAY));

			memcpy(MatchBuff, MatchArray, nMatches * sizeof(RVLSURFACE_MATCH));

			iLastSSurf = nS3DSurfaces - 1;

			PoseSM.Copy(&PoseSMInit);

			RVLCOPY3VECTOR(invtInit, invt);

			nFailures = 0;

			bNewHypothesis = FALSE;

			pMatchedMSurf = MatchedMSurfArray;

			pMatchedSSurf = MatchedSSurfArray;
	
			while(TRUE)	// hypothesis generation loop
			{
				iSSurf = (iLastSSurf > 0 ? RVLRandom(0, iLastSSurf) : 0);

				pSSurfMatchData = SSurfMatchBuff + iSSurf;

				pS3DSurface = (CRVL3DSurface2 *)(pSSurfMatchData->pSObject);

				iLastMSMatch = pSSurfMatchData->pLast - pSSurfMatchData->pFirst;

				iMatch = (iLastMSMatch > 0 ? RVLRandom(0, iLastMSMatch) : 0);

				pMSMatch = pSSurfMatchData->pFirst + iMatch;

				pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMObject);

				if(pM3DSurface->m_Flags & RVLOBJ2_FLAG_MATCHED)
				{
					if(iLastMSMatch > 0)
					{
						// remove *pMSMatch from MatchArray

						pSSurfMatchData->pFirst[iMatch] = *(pSSurfMatchData->pLast);

						pSSurfMatchData->pLast--;
					}
					else
					{
						if(iLastSSurf == 0)
							break;

						// remove *pSSurfMatchData from SSurfMatchBuff

						SSurfMatchBuff[iSSurf] = SSurfMatchBuff[iLastSSurf];

						iLastSSurf--;
					}

					continue;
				}
				
				if(RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, &PoseSM, &PoseSM, &MatchData))
				{
					pM3DSurface->m_Flags |= RVLOBJ2_FLAG_MATCHED;

					*(pMatchedMSurf++) = pM3DSurface;

					*(pMatchedSSurf++) = pS3DSurface;

					// if the pose uncertainty is inside the tolerance specified by m_maxPoseSMUncert

					Pt = P + 2 * 3 * 3;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
					cvSet(pFig2->m_pImage, cvScalar(255, 255, 255));

					//int iTextLine = HypothesisDisplay(pFig2, pNode, cvPoint(8, 0), MatchList);

					int iTextLine = 0;

					//sprintf(Text, "sqrt(diag(P))=[%lf, %lf, %lf, %lf, %lf, %lf]", 
					//	sqrt(P[0 * 3 + 0]) * RAD2DEG, 
					//	sqrt(P[1 * 3 + 1]) * RAD2DEG, 
					//	sqrt(P[2 * 3 + 2]) * RAD2DEG,
					//	sqrt(Pt[0 * 3 + 0]), 
					//	sqrt(Pt[1 * 3 + 1]), 
					//	sqrt(Pt[2 * 3 + 2]));

					pFig2->DisplayCovMx(P, 8, iTextLine * pFig2->m_FontSize, cvScalar(0, 0, 0), RAD2DEG);

					iTextLine += 5;

					pFig2->DisplayCovMx(Pt, 8, iTextLine * pFig2->m_FontSize, cvScalar(0, 0, 0));

					iTextLine += 5;

					sprintf(Text, "iHypothesis=%d", iHypothesis);

					pFig2->PutText(Text, cvPoint(8, (iTextLine + 1) * pFig2->m_FontSize), cvScalar(0, 0, 0));

					iTextLine++;

					sprintf(Text, "M%d-S%d", pM3DSurface->m_Index, pS3DSurface->m_Index);

					pFig2->PutText(Text, cvPoint(8, (iTextLine + 1) * pFig2->m_FontSize), cvScalar(0, 0, 0));

					m_DebugData.pGUI->ShowFigure(pFig2);

					cvWaitKey();
#endif


					if(Pt[0 * 3 + 0] <= m_maxPoseSMUncert && Pt[1 * 3 + 1] <= m_maxPoseSMUncert &&	Pt[2 * 3 + 2] <= m_maxPoseSMUncert)
					{
						bNewHypothesis = TRUE;

						break;
					}

					if(iLastSSurf == 0)
						break;

					// remove *pSSurfMatchData from SSurfMatchBuff

					SSurfMatchBuff[iSSurf] = SSurfMatchBuff[iLastSSurf];

					iLastSSurf--;
				}
				else
				{
					nFailures++;

					if(nFailures > maxnFailures)
						break;

					if(iLastMSMatch > 0)
					{
						// remove *pMSMatch from MatchArray

						pSSurfMatchData->pFirst[iMatch] = *(pSSurfMatchData->pLast);

						pSSurfMatchData->pLast--;
					}
					else
					{
						if(iLastSSurf == 0)
							break;

						// remove *pSSurfMatchData from SSurfMatchBuff

						SSurfMatchBuff[iSSurf] = SSurfMatchBuff[iLastSSurf];

						iLastSSurf--;
					}
				}				
			}	// hypothesis generation loop

			if(bNewHypothesis)
			{
				// check if a similar hypothesis already exists

				t = PoseSM.m_X;

				bHypothesisExists = FALSE;

				for(i = 0; i < iHypothesis; i++)
				{
					pHypothesis = HypothesisArray[i];

					pPoseSM2 = &(pHypothesis->PoseSM);

					t2 = pPoseSM2->m_X;

					err = PoseSM.m_Alpha - pPoseSM2->m_Alpha;

					if(err > 2.0 * DEG2RAD || err < -2.0 * DEG2RAD)
						continue;

					err = PoseSM.m_Beta - pPoseSM2->m_Beta;

					if(err > 2.0 * DEG2RAD || err < -2.0 * DEG2RAD)
						continue;

					err = PoseSM.m_Theta - pPoseSM2->m_Theta;

					if(err > 2.0 * DEG2RAD || err < -2.0 * DEG2RAD)
						continue;

					RVLDIF3VECTORS(t, t2, e);

					if(RVLDOTPRODUCT3(e, e) < 10000.0)
					{
						bHypothesisExists = TRUE;

						break;
					}
				}

				// if similar hypothesis does not exist, then create new hypothesis
				// and insert it into m_HypothesisList

				if(!bHypothesisExists)
				{
					RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);

					pPoseSM2 = &(pHypothesis->PoseSM);

					pPoseSM2->m_C = pHypothesis->P;
					pPoseSM2->m_pData = pHypothesis->invt;

					t2 = pPoseSM2->m_X;
					RVLCOPY3VECTOR(t, t2);
					R = PoseSM.m_Rot;
					R2 = pPoseSM2->m_Rot;
					RVLCOPYMX3X3(R, R2);
					P3 = pPoseSM2->m_C;
					P4 = PoseSM.m_C;
					RVLCOPYMX3X3(P4, P3);
					P4 += (3 * 3);
					P3 += (3 * 3);
					RVLCOPYMX3X3(P4, P3);
					P4 += (3 * 3);
					P3 += (3 * 3);
					RVLCOPYMX3X3(P4, P3);
					invt2 = (double *)(pPoseSM2->m_pData);
					RVLCOPY3VECTOR(invt, invt2);
					pPoseSM2->m_Alpha = PoseSM.m_Alpha;
					pPoseSM2->m_Beta = PoseSM.m_Beta;
					pPoseSM2->m_Theta = PoseSM.m_Theta;
					pPoseSM2->m_ca = PoseSM.m_ca;
					pPoseSM2->m_sa = PoseSM.m_sa;
					pHypothesis->Index = (HypothesisIndex++);
					pHypothesis->cost = 0;
					pHypothesis->visible = 0;
					pHypothesis->iS = pHypothesis->iM = 0;
					pHypothesis->pMPSuLM = pMPSuLM;
					pHypothesis->Flags = 0x00;

					pHypothesis->nMatches = pMatchedSSurf - MatchedSSurfArray;

					RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, CRVL3DSurface2 *, pHypothesis->nMatches, pHypothesis->MatchArray);

					memcpy(pHypothesis->MatchArray, MatchedSSurfArray, pHypothesis->nMatches * sizeof(CRVL3DSurface2 *));

					m_HypothesisList.Add(pHypothesis);	

					*(ppHypothesis++) = pHypothesis;

					iHypothesis++;
				}
			}

			
			// reset all RVLOBJ2_FLAG_MATCHED flags

			pMatchedMSurfArrayEnd = pMatchedMSurf;

			for(pMatchedMSurf = MatchedMSurfArray; pMatchedMSurf < pMatchedMSurfArrayEnd; pMatchedMSurf++)
				(*pMatchedMSurf)->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;
		}	// for each hypothesis	

		double ExecutionTime = m_pTimer->GetTime() - StartTime;

   		if(pPrevSPSuLM)
			break;
	}	// for each model PSuLM	

	m_pMem2->Clear();

	// only for debugging purpose

//#ifndef RVLPSULMBUILDER_HYPOTHESES_DEBUG
//	CRVLFigure *pFig;
//
//	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData;
//#endif
//
//	m_HypothesisList.Start();
//
//	while(m_HypothesisList.m_pNext)
//	{
//		pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());
//
//		if(pHypothesis->Index == 0)
//		{
//			pSPSuLM->Display(m_DebugData.pGUI, &(pHypothesis->PoseSM));
//
//			pHypothesis->pMPSuLM->AddToFigure(m_DebugData.pGUI);
//
//			pFig = m_DebugData.pGUI->OpenFigure("PSuLM");
//
//			pMouseCallbackData = (RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);
//
//			pMouseCallbackData->MatrixSceneModel = NULL;	
//
//			//pMouseCallbackData->MSurfMatchData = MSurfMatchData;
//			//pMouseCallbackData->SSurfMatchData = SSurfMatchData;
//
//			pMouseCallbackData->pSSurf = NULL;
//			pMouseCallbackData->pMSurf = NULL;
//		}
//		else
//		{
//			pMouseCallbackData->pMPSuLM = pHypothesis->pMPSuLM;
//		
//			memcpy(pMouseCallbackData->PoseS0.m_Rot, pHypothesis->PoseSM.m_Rot, 3 * 3 * sizeof(double));
//			memcpy(pMouseCallbackData->PoseS0.m_X, pHypothesis->PoseSM.m_X, 3 * sizeof(double));
//
//			pMouseCallbackData->Flags = RVLPSULM_DISPLAY_SURFACES;
//
//			RVLPSuLMDisplay(pFig);
//		}
//
//		cvWaitKey();
//	}
}




void CRVLPSuLMBuilder::Hypotheses5(	CRVLPSuLM *pSPSuLM,
									CRVL3DPose *pPoseS0Init,
									double *PInit,
									CRVLPSuLM * pPrevSPSuLM)
{
	int nHypotheses = m_maxnHypothesesPerModel;

	int maxnM3DSurfaces = m_maxnDominant3DSurfaces;

	//int maxnSamples = 5;

	int maxnFailures = 3;

	int maxnNodes = 100000;	

	//int QueueSize = 1000;

	//int PredictionHorizon = 3;

	int maxnMatchesInHypothesis = 7;

	int TraveledDistBinSize = 50;	// mm

	double fTraveledDistBinSize = (double)TraveledDistBinSize;

	int maxTraveledDist = 1000;	// mm

	double fmaxTraveledDist = (double)maxTraveledDist;

	m_HypothesisList.RemoveAll();

	int nS3DSurfaces = pSPSuLM->m_n3DSurfaces;

	if(nS3DSurfaces == 0)
		return;

	int iLastSSurf = nS3DSurfaces - 1;

	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;
	CRVL3DSurface2 **ppSSurf = SSurfArray;

	//RVLPSULM_SURF_MATCH_DATA *SSurfMatchData = new RVLPSULM_SURF_MATCH_DATA[nS3DSurfaces];

	//RVLPSULM_SURF_MATCH_DATA *pSSurfMatchData = SSurfMatchData;

	CRVL3DSurface2 *pS3DSurface;

	//int SSupportTotal = 0;

	int i;

	for(i=0;i<nS3DSurfaces;i++)
	{
		pS3DSurface = SSurfArray[i];

		pS3DSurface->m_Index = i;

		//SSupportTotal += pS3DSurface->m_nSupport;

		//pSSurfMatchData->p3DSurface = pS3DSurface;

		//pSSurfMatchData->bMatched = 0;

		//pSSurfMatchData++;
	}

	CRVL3DSurface2 **pSSurfArrayEnd = SSurfArray + nS3DSurfaces;

	//int QueueSize = (10 * SSupportTotal) / 10 + 1;
	int QueueSize = m_pCamera->Width * m_pCamera->Height + 1;

	//**********************************

	//CRVL3DPose PoseSM;
	CRVL3DPose PoseSMInit;

	PoseSMInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	double CInit[3 * 3 * 3];
	double invtInit[3];
	PoseSMInit.m_C = CInit;
	PoseSMInit.Copy(pPoseS0Init);
	PoseSMInit.m_pData = invtInit;
	double *R = PoseSMInit.m_Rot;
	double *t = PoseSMInit.m_X;
	RVLMULMX3X3TVECT(R, t, invtInit);

	//PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	//double C[3 * 3 * 3];
	//PoseSM.m_C = C;
	//PoseSM.m_pData = invt;

	CRVLPSuLM *pMPSuLM;
	CRVL3DSurface2 *pM3DSurface;
	CRVL3DSurface2 **ppMSurf;
	//CRVLMPtrChain *pM3DSurfaceList;
	CRVL3DSurface2 **MSurfArray;
	int iHypothesis;
	double MatchQuality;
	double detQ;

	RVLPSULM_MSMATCH_DATA *MatchList;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_MSMATCH_DATA, nS3DSurfaces * maxnM3DSurfaces, MatchList);

	RVLPSULM_LASTDOF_MATCH_DATA *LastDOFMatchArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_LASTDOF_MATCH_DATA, nS3DSurfaces * maxnM3DSurfaces, LastDOFMatchArray);

	RVLPSULM_HG_NODE *NodeMem;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_HG_NODE, maxnNodes, NodeMem);

	RVLPSULM_HG_NODE *pNodeMemEnd = NodeMem + maxnNodes;

	CRVLQListArray Queue;

	Queue.m_Size = QueueSize + 1;

	Queue.Init();

	RVLQLIST *QueueListArray = Queue.m_ListArray;	

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
	pSPSuLM->Display(m_DebugData.pGUI, pPoseS0Init);

	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("PSuLM");

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	pMouseCallbackData->MatrixSceneModel = NULL;	

	//pMouseCallbackData->MSurfMatchData = MSurfMatchData;
	//pMouseCallbackData->SSurfMatchData = SSurfMatchData;

	CRVLFigure *pFig2 = m_DebugData.pGUI->OpenFigure("PSuLM2");	

	pFig->m_FontSize = pFig2->m_FontSize = 16;

	int nTextLines = 22;

	int TextImageHeight = nTextLines * pFig2->m_FontSize;

	pFig2->EmptyBitmap(cvSize(800, TextImageHeight), cvScalar(255, 255, 255));

	cvInitFont(&(pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	cvInitFont(&(pFig2->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	char Text[201];

	pFig2->PutText("Hello world!", cvPoint(0, pFig2->m_FontSize), cvScalar(0, 0, 0));

	m_DebugData.pGUI->ShowFigure(pFig2);
#endif

	RVLPSULM_HYPOTHESIS **HypothesisArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_HYPOTHESIS *, nHypotheses, HypothesisArray);

	//int *TraveledDistAcc;

	//int nTraveledDistBins = 2 * maxTraveledDist / TraveledDistBinSize + 1;

	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, int, nTraveledDistBins, TraveledDistAcc);

	//int *TraveledDistBin =  TraveledDistAcc + maxTraveledDist / TraveledDistBinSize;

	//CRVLQListArray TraveledDistAccMatchPtrList;

	

	int HypothesisIndex = 0;

	double eigVal[3];
	
	//RVLMatrixHeader31->data.db = eigVal;

	double RBT[3 * 3];

	double *xTB = RBT;
	double *yTB = RBT + 3;
	double *zTB = RBT + 2 * 3;

	//RVLMatrixHeaderB33->data.db = eigVect;

	//CvMat *_eigVal = cvCreateMatHeader(3, 1, CV_64FC1);
	//CvMat *_eigVect = cvCreateMatHeader(3, 3, CV_64FC1);

	//_eigVal->data.db = eigVal;
	//_eigVect->data.db = V;

	RVLSURFACE_MATCH_ARRAY MatchData;

	double *e = MatchData.m_e;

	RVLPSULM_HYPOTHESIS *pHypothesis = NULL;

	int nM3DSurfaces;
	RVLPSULM_MSMATCH_DATA *pMSMatch;
	RVLPSULM_MSMATCH_DATA *pMSMatchListEnd;
	//int maxSurfIndexSum;
	CRVL3DSurface2 **pMSurfArrayEnd;
	//RVLPSULM_SURF_MATCH_DATA *pMSurfMatchData;
	//RVLPSULM_SURF_MATCH_DATA *pMSurfMatchDataArrayEnd;
	double *t2, *P2;
	//double *P, *R2, *P3;
	CRVL3DPose *pPoseSM;
	int Support;
	int iLastMSurf;
	int cost, minCost;
	//int MSupportTotal;
	RVLPSULM_HG_NODE *pNode, *pNewNode, *pPNode, *pNode2;
	//double *invt, *invt2;
	int iMinSupport, iMaxSupport, iMidSupport;
	int iMSurf, iSSurf;
	//int SSupport;
	//int nMSMatches;
	int iLastMSMatch;
	RVLQLIST *QueueList;
	int PredictedScore;
	int nExpandedNodes;
	RVLPSULM_HYPOTHESIS **ppHypothesis;
	CRVL3DPose *pPoseSM2;
	BOOL bNewHypothesis;
	double err;
	int maxCost;
	CRVL3DSurface2 **ppMatchedSSurf;
	double *pPTmp;
	double mu;
	double Jsmu[2], JsTD[2];
	double muTol;
	double TraveledDist, varTD;
	double k1;
	double *tFA, *tF2B, *RFA, *RF2B, *zTB2;
	double tF2A[3], tF2_FA[3], tFB[3], zTA[3], xTF2[3], yTF2[3], xTF[3], yTF[3], Q[3], RFB[3 * 3];
	double Mx3x3Tmp[3 * 3];
	double Vect3Tmp[3];
	double dx, dy, eF, vartFAx, vartFAy, vartFAz, vartF2Bx, vartF2By, vartF2Bz, eTD, csT, wTD;
	//int iTraveledDistBin, iBestTraveledDistBin, BestTravelDistScore;
	int nLastDOFMatches;
	RVLPSULM_LASTDOF_MATCH_DATA *pLastDOFMatchData, *pLastDOFMatchData2, *pLastDOFMatchArrayEnd, *pBestLastDOFMatch;
	double TDScore, BestTDScore;
	//BOOL bLastDOFMatchStored, bLastDOFMatchMerged;
	//int iNewLastDOFMatch;
	RVLPSULM_HYPOTHESIS *pHypothesis2;
	double Vect3Tmp2[3];
	int nFeatures;

	//int debug_nMatchings = 0;
	//int debug_nQueueSorts = 0;
	
	// initial feature matching 

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
	FILE *fpLog;

	fpLog = fopen("C:\\RVL\\Debug\\PSuLMHypGen.dat", "w");
#endif

	m_PSuLMList.Start();

	while(TRUE)
	{
		double StartTime = m_pTimer->GetTime();

		if(pPrevSPSuLM)
			pMPSuLM = pPrevSPSuLM;
		else if(m_PSuLMList.m_pNext)
			pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());
		else
			break;

		//if(pMPSuLM->m_Index != 13)	// debug
		//	continue;

		MSurfArray = pMPSuLM->m_3DSurfaceArray;

		ppMSurf = MSurfArray;

		nM3DSurfaces = pMPSuLM->m_n3DSurfaces;

		maxCost = QueueSize;

		iLastMSurf = nM3DSurfaces - 1;

		//pMSurfMatchData = MSurfMatchData;

		//MSupportTotal = 0;

		for(i=0;i<nM3DSurfaces;i++)
		{
			pM3DSurface = MSurfArray[i];

			*(ppMSurf++) = pM3DSurface;

			//pMSurfMatchData->p3DSurface = pM3DSurface;

			//pMSurfMatchData->bMatched = 0;

			//pMSurfMatchData++;

			//MSupportTotal += pM3DSurface->m_nSupport;		// move to load or create function
		}

		//SupportTotal = (SSupportTotal + MSupportTotal) * 80 / 100;

		pMSurfArrayEnd = ppMSurf;

		Queue.Reset();

		//pMSurfMatchDataArrayEnd = pMSurfMatchData;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
		pMPSuLM->AddToFigure(m_DebugData.pGUI);

		//pMouseCallbackData->nMSurfs = pM3DSurfaceList->m_nElements;

		cvWaitKey();
#endif
		// create a sorted list of matches

		pMSMatch = MatchList;

		pM3DSurface = MSurfArray[0];
		pS3DSurface = SSurfArray[0];
		pMSMatch->pMData = (RVLPSULM_SURF_MATCH_DATA *)pM3DSurface;
		pMSMatch->pSData = (RVLPSULM_SURF_MATCH_DATA *)pS3DSurface;
		//pMSMatch->Support = pM3DSurface->m_nSupport + pS3DSurface->m_nSupport;
		pMSMatch->Support = maxCost;

		iLastMSMatch = 0;

		pPTmp = PoseSMInit.m_C;

		PoseSMInit.m_C = PInit;

		for(iSSurf = 0; iSSurf < nS3DSurfaces; iSSurf++)
		{
			pS3DSurface = SSurfArray[iSSurf];

			//SSupport = pS3DSurface->m_nSupport;

			for(iMSurf = 0; iMSurf < nM3DSurfaces; iMSurf++)
			{
				//if(iSSurf == 1 && iMSurf == 1)
				//	int debug = 1;

				pM3DSurface = MSurfArray[iMSurf];	

				if(!pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, detQ, &MatchData))
					continue;

				//Support = DOUBLE2INT(sqrt((double)(SSupport * pM3DSurface->m_nSupport)));
				//Support = SSupport + pM3DSurface->m_nSupport;
				//Support = maxCost - (iSSurf + iMSurf);
				Support = RVLMIN(pM3DSurface->m_nSupport, pS3DSurface->m_nSupport);

				if(Support <= MatchList[iLastMSMatch].Support)
				{
					iLastMSMatch++;

					pMSMatch = MatchList + iLastMSMatch;

					pMSMatch->pMData = (RVLPSULM_SURF_MATCH_DATA *)pM3DSurface;
					pMSMatch->pSData = (RVLPSULM_SURF_MATCH_DATA *)pS3DSurface;
					pMSMatch->Support = Support;
				}
				else
				{
					iMaxSupport = 0;
		
					iMinSupport = iLastMSMatch;

					while(iMinSupport - iMaxSupport > 1)
					{
						iMidSupport = (iMinSupport + iMaxSupport) / 2;

						if(Support > MatchList[iMidSupport].Support)
							iMinSupport = iMidSupport;
						else
							iMaxSupport = iMidSupport;
					}

					memmove(MatchList + iMinSupport + 1, MatchList + iMinSupport, 
						(iLastMSMatch - iMinSupport + 1) * sizeof(RVLPSULM_MSMATCH_DATA));

					pMSMatch = MatchList + iMinSupport;

					pMSMatch->pMData = (RVLPSULM_SURF_MATCH_DATA *)pM3DSurface;
					pMSMatch->pSData = (RVLPSULM_SURF_MATCH_DATA *)pS3DSurface;
					pMSMatch->Support = Support;

					iLastMSMatch++;
				}
			}
		}

		PoseSMInit.m_C = pPTmp;

		pMSMatchListEnd = MatchList + iLastMSMatch + 1;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
		fprintf(fpLog, "Match List:\n");

		for(pMSMatch = MatchList; pMSMatch < pMSMatchListEnd; pMSMatch++)
		{
			pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
			pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

			if(pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, detQ, &MatchData))
				fprintf(fpLog, "M%d-S%d\t\t%d\n", 
					((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Index,
					((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Index,
					pMSMatch->Support);
		}
#endif

		// create the first node 

		pNewNode = NodeMem;

		pMSMatch = HypothesesGetNextNode(NULL, 1, MatchList, pMSMatchListEnd, PredictedScore);

		pNewNode->iMatch = 1;
		pNewNode->pParent = NULL;
		pNewNode->Support = pMSMatch->Support;
		pNewNode->g = 1;
		pNewNode->nFailures = 0;
		pNewNode->PoseSM.m_C = pNewNode->P;
		pNewNode->PoseSM.m_pData = pNewNode->invt;
		
		//cost = (SupportTotal - pNewNode->Support) / pNewNode->Support + 1;
		
		//cost = (pNewNode->Support + PredictionHorizon * pNewNode->Support) / 64;

		//cost = PredictedScore / 10;

		cost = PredictedScore;

		if(cost > QueueSize)
			cost = QueueSize;

		RVLQLISTARRAY_ADD_ENTRY2(QueueListArray, cost, pNewNode);

		pNode = pNewNode;

		pNewNode++;

		minCost = cost;
			
		//*** generate hypotheses

		iHypothesis = 0;

		nExpandedNodes = 0;

		nFeatures = 0;

		ppHypothesis = HypothesisArray;

		//while(iHypothesis < nHypotheses && nFeatures <= m_maxnExpandedNodes)
		//while(nExpandedNodes <= m_maxnExpandedNodes && nFeatures <= m_refnHypotheses)
		while(nExpandedNodes <= m_maxnExpandedNodes)
		{
			// skip the empty bins in queue

			while(QueueListArray[minCost].pFirst == NULL)
			{
				if(minCost > 0)
					minCost--;
				else
					break;
			}

			if(QueueListArray[minCost].pFirst == NULL)	// is queue empty?
				break;

			// pNode <- the node with the maximum weight from the queue

			pNode = (RVLPSULM_HG_NODE *)(QueueListArray[minCost].pFirst);

			QueueList = QueueListArray + minCost;

			// remove pNode from queue

			RVLQLIST_REMOVE_ENTRY2(QueueList, pNode, RVLPSULM_HG_NODE);

			// expand the parent node of pNode

			pPNode = pNode->pParent;

			if(pPNode)
				if(pPNode->nFailures > maxnFailures)
					continue;

			//if(pNode->iMatch == 56 && pPNode->iMatch == 1)
			//	int debug = 0;

			if(pMSMatch = HypothesesGetNextNode(pPNode, pNode->iMatch + 1, MatchList, pMSMatchListEnd, PredictedScore))
			{
				//if(pPNode)
				//{
				//	pNewNode->Support = pPNode->Support;
				//	pNewNode->g = pPNode->g;
				//}
				//else
				//{
				//	pNewNode->Support = 0;
				//	pNewNode->g = 0;
				//}

				// pNewNode <- new node

				pNewNode->iMatch = pMSMatch - MatchList;
				pNewNode->pParent = pPNode;

				//dSupport = ((CRVL3DSurface2 *)(pMSMatch->pMData))->m_nSupport + 
				//	((CRVL3DSurface2 *)(pMSMatch->pSData))->m_nSupport;
				//pNewNode->Support += dSupport;
				pNewNode->g = (pPNode ? pPNode->g + 1 : 1);
				pNewNode->nFailures = 0;
				pNewNode->PoseSM.m_C = pNewNode->P;
				pNewNode->PoseSM.m_pData = pNewNode->invt;

				// add pNewNode to the queue 
				
				//cost = (SupportTotal - pNewNode->Support) / dSupport + pNewNode->g;

				//cost = (pNewNode->Support + PredictionHorizon * dSupport) / 64;				

				//cost = (pNewNode->Support + PredictedScore) / 10;

				//cost  = PredictedScore / 10;

				cost  = PredictedScore;

				if(cost > QueueSize)
					cost = QueueSize;

				if(cost > minCost)
					minCost = cost;

				RVLQLISTARRAY_ADD_ENTRY2(QueueListArray, cost, pNewNode);

//#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
//				pFig2->EmptyBitmap(cvSize(800, TextImageHeight), cvScalar(255, 255, 255));
//
//				HypothesisDisplay(pFig2, pNewNode, cvPoint(8, 0), MatchList);
//
//				m_DebugData.pGUI->ShowFigure(pFig2);
//
//				cvWaitKey();
//#endif
				pNewNode++;

				if(pNewNode == pNodeMemEnd)
					break;
			}

			if(pNode->g > maxnMatchesInHypothesis)
			{
				bNewHypothesis = FALSE;

				continue;
			}

			// if pNode is a root node, then pPoseSM <- &PoseSMInit,
			// otherwise, pPoseSM <- pose of the parent node

			pPoseSM = (pPNode ? &(pPNode->PoseSM) : &PoseSMInit);
				
			// pMSMatch <- match corresponding to pNode

			pMSMatch = MatchList + pNode->iMatch;

			nExpandedNodes++;

			// check whether the surfaces of pMSMatch satisfy geometrical constraints  
			
			pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
			pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

			//if(pM3DSurface->m_Index == 7 && pS3DSurface->m_Index == 1)
 			//	int debug = 0;

			//if(pS3DSurface->Match2(pM3DSurface, pPoseSM, MatchQuality, &MatchData))
			if(RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, pPoseSM, &(pNode->PoseSM), &MatchData))
			{
				if(pPNode)
					pPNode->nFailures = 0;

				// EKF 
				
				//pNode->PoseSM.PlanarSurfaceEKFUpdate2(MatchData.m_C, MatchData.m_Q, MatchData.m_e, pPoseSM);

				//R = pNode->PoseSM.m_Rot;
				//t = pNode->PoseSM.m_X;
				//invt = pNode->invt;
				//RVLMULMX3X3TVECT(R, t, invt);

				//if(e[0] * e[0] + e[1] * e[1] > 0.04)
				//{
				//	memcpy(pNode->PoseSM.m_C, pPoseSM->m_C, 3 * 3 * 3 * sizeof(double));

				//	pS3DSurface->Match2(pM3DSurface, &(pNode->PoseSM), MatchQuality, &MatchData);

				//	pNode->PoseSM.PlanarSurfaceEKFUpdate2(MatchData.m_C, MatchData.m_Q, MatchData.m_e);

				//	RVLMULMX3X3TVECT(R, t, invt);
				//}

				// if the pose uncertainty is inside the tolerance specified by m_maxPoseSMUncert

				P2 = pNode->P + 2 * 3 * 3;

				//RVLMatrixHeaderA33->data.db = P2;



#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
					//if(HypothesisIndex == 113)
					//	int debug = 0;

					cvSet(pFig2->m_pImage, cvScalar(255, 255, 255));

					int iTextLine = HypothesisDisplay(pFig2, pNode, cvPoint(8, 0), MatchList);

					sprintf(Text, "cost=%d", minCost);

					pFig2->PutText(Text, cvPoint(8, (iTextLine + 1) * pFig2->m_FontSize), cvScalar(0, 0, 0));

					iTextLine++;

					pFig2->DisplayCovMx(pNode->P, 8, (iTextLine + 1) * pFig2->m_FontSize, cvScalar(0, 0, 0), RAD2DEG);

					iTextLine += 4;

					pFig2->DisplayCovMx(pNode->P + 2 * 3 * 3, 8, (iTextLine + 1) * pFig2->m_FontSize, cvScalar(0, 0, 0));

					iTextLine += 4;

					sprintf(Text, "iHypothesis=%d", iHypothesis);

					iTextLine++;

					pFig2->PutText(Text, cvPoint(8, (iTextLine + 1) * pFig2->m_FontSize), cvScalar(0, 0, 0));

					m_DebugData.pGUI->ShowFigure(pFig2);

					cvWaitKey();
#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
					PrintHypothesis(fpLog, pNode, MatchList);

					fprintf(fpLog, "nExpandedNodes: %d\n", nExpandedNodes);

					RVLPrintCov(fpLog, pNode->P, 3, RAD2DEG);

					RVLPrintCov(fpLog, pNode->P + 2 * 3 * 3, 3);

					fprintf(fpLog, "\n");
#endif
				//cvSVD(RVLMatrixHeaderA33, _eigVal, _eigVect, NULL, CV_SVD_U_T);

				//if(!RVLGetAxesOfCov3D(P2, eigVal, RBT, Mx3x3Tmp, Vect3Tmp))
				//	eigVal[2] = 2.0 * m_maxPoseSMUncert;

				//if(fabs(eigVal[0] - eigVal2[2]) > 10)
				//	int debug = 0;

				//if(eigVal2[2] < 0)
				//	int debug = 0;

				//if((eigVal[1] <= m_maxPoseSMUncert)	//$
				if((pNode->P[0] <= m_maxvarRotHyp && pNode->P[4] <= m_maxvarRotHyp && pNode->P[8] <= m_maxvarRotHyp)
					//|| pNode->g >= maxnMatchesInHypothesis
				//if(P2[0] <= m_maxPoseSMUncert && P2[4] <= m_maxPoseSMUncert && P2[8] <= m_maxPoseSMUncert
					)
					bNewHypothesis = TRUE;
				else
				{
					// expand pNode

					if(pMSMatch = HypothesesGetNextNode(pNode, pNode->iMatch + 1, MatchList, pMSMatchListEnd, PredictedScore))
					{
						// pNewNode <- new node

						pNewNode->iMatch = pMSMatch - MatchList;
						pNewNode->pParent = pNode;
						//dSupport = ((CRVL3DSurface2 *)(pMSMatch->pMData))->m_nSupport + 
						//	((CRVL3DSurface2 *)(pMSMatch->pSData))->m_nSupport;
						//pNewNode->Support = pNode->Support + dSupport;
						pNewNode->g = pNode->g + 1;
						pNewNode->nFailures = 0;
						pNewNode->PoseSM.m_C = pNewNode->P;
						pNewNode->PoseSM.m_pData = pNewNode->invt;

						// add pNewNode to the queue 
						
						//cost = (SupportTotal - pNewNode->Support) / dSupport + pNewNode->g;

						//cost = (pNewNode->Support + PredictionHorizon * dSupport) / 64;						

						//cost = (pNode->Support + PredictedScore) / 10;

						//cost = PredictedScore / 10;

						cost = PredictedScore;

						if(cost > QueueSize)
							cost = QueueSize;

						if(cost > minCost)
							minCost = cost;

						RVLQLISTARRAY_ADD_ENTRY2(QueueListArray, cost, pNewNode);

						bNewHypothesis = FALSE;

						pNewNode++;

						if(pNewNode == pNodeMemEnd)
							break;
					}
					else
						bNewHypothesis = FALSE;
				}

				// check if a similar hypothesis already exists

				if(bNewHypothesis)
				{
					nFeatures++;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
					fprintf(fpLog, "Rotation precision is sufficient.\n");
#endif
					if(RVLGetAxesOfCov3D(P2, eigVal, RBT, Mx3x3Tmp, Vect3Tmp))
					{						
						pPoseSM = &(pNode->PoseSM);

						R = pPoseSM->m_Rot;

						t = pPoseSM->m_X;

						Mx3x3Tmp[0] = 1.0 - zTB[0] * zTB[0];
						Mx3x3Tmp[1] =     - zTB[0] * zTB[1];
						Mx3x3Tmp[2] =     - zTB[0] * zTB[2];
						Mx3x3Tmp[4] = 1.0 - zTB[1] * zTB[1];
						Mx3x3Tmp[5] =     - zTB[1] * zTB[2];
						Mx3x3Tmp[8] = 1.0 - zTB[2] * zTB[2];

						RVLMULCOV3VECT(Mx3x3Tmp, t, Vect3Tmp)

						for(i = 0; i < iHypothesis && bNewHypothesis; i++)
						{
							pHypothesis2 = HypothesisArray[i];

							pPoseSM2 = &(pHypothesis2->PoseSM);

							err = pPoseSM->m_Alpha - pPoseSM2->m_Alpha;

							if(err > 2.0 * DEG2RAD || err < -2.0 * DEG2RAD)
								continue;

							err = pPoseSM->m_Beta - pPoseSM2->m_Beta;

							if(err > 2.0 * DEG2RAD || err < -2.0 * DEG2RAD)
								continue;

							err = pPoseSM->m_Theta - pPoseSM2->m_Theta;

							if(err > 2.0 * DEG2RAD || err < -2.0 * DEG2RAD)
								continue;

							zTB2 = pHypothesis2->zTB;

							csT = RVLDOTPRODUCT3(zTB, zTB2);

							if(csT > -0.985 && csT < 0.985)	// 0.985 = cos(10 deg)
								continue;

							t2 = pPoseSM2->m_X;
							
							RVLMULCOV3VECT(Mx3x3Tmp, t2, RVLVector3)

							RVLDIF3VECTORS(Vect3Tmp, RVLVector3, Vect3Tmp2)

							if(RVLDOTPRODUCT3(Vect3Tmp2, Vect3Tmp2) <= 10000.0)
							{
								bNewHypothesis = FALSE;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
								fprintf(fpLog, "Similar hypothesis already exists.\n");
#endif

								break;
							}
						}
					}
					else
						bNewHypothesis = FALSE;
				}

				nLastDOFMatches = 0; 

				if(bNewHypothesis)
				{
#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
					fprintf(fpLog, "Estimating the last DOF...\n");
#endif
					//*** estimate the last degree of freedom by evidence accumulation

					//memset(TraveledDistAcc, 0, nTraveledDistBins * sizeof(int));

					//BestTravelDistScore = 0;

					for(pMSMatch = MatchList + 1; pMSMatch < pMSMatchListEnd; pMSMatch++)
					{
						pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);

						RF2B = pM3DSurface->m_Pose.m_Rot;

						mu = fabs(RVLDOTPRODUCT3(zTB, pM3DSurface->m_N));
						Jsmu[0] = RVLMULROWCOL3(zTB, RF2B, 0, 0);
						Jsmu[1] = RVLMULROWCOL3(zTB, RF2B, 0, 1);
						muTol = sqrt(Jsmu[0] * Jsmu[0] * pM3DSurface->m_varq[0] + 
							Jsmu[1] * Jsmu[1] * pM3DSurface->m_varq[1]);
						
						if(mu + muTol < 0.7)
							continue;

						pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

						// zTA = RAB' * zTB

						RVLMULMX3X3TVECT(R, zTB, zTA)

						RFA = pS3DSurface->m_Pose.m_Rot;

						mu = fabs(RVLDOTPRODUCT3(zTA, pS3DSurface->m_N));
						Jsmu[0] = RVLMULROWCOL3(zTA, RFA, 0, 0);
						Jsmu[1] = RVLMULROWCOL3(zTA, RFA, 0, 1);
						muTol = sqrt(Jsmu[0] * Jsmu[0] * pS3DSurface->m_varq[0] + 
							Jsmu[1] * Jsmu[1] * pS3DSurface->m_varq[1]);
						
						if(mu + muTol < 0.7)
							continue;

						// [xTF2'; yTF2'] = RF2B' * [xTB'; yTB']

						RVLMULMX3X3TVECT(RF2B, xTB, xTF2)
						RVLMULMX3X3TVECT(RF2B, yTB, yTF2)

						// RFB = RAB * RFA

						RVLMXMUL3X3(R, RFA, RFB)
					
						// [xTF'; yTF'] = RFB' * [xTB'; yTB']	

						RVLMULMX3X3TVECT(RFB, xTB, xTF)
						RVLMULMX3X3TVECT(RFB, yTB, yTF)

						// Q = [xTF2'; yTF2'] * CF2 * [xTF2, yTF2] + [xTF'; yTF'] * CF * [xTF, yTF]

						vartFAx  = pS3DSurface->m_EigenValues[0] * pS3DSurface->m_EigenValues[0];
						vartFAy  = pS3DSurface->m_EigenValues[1] * pS3DSurface->m_EigenValues[1];
						vartFAz  = pS3DSurface->m_sigmaR;
						vartF2Bx = pM3DSurface->m_EigenValues[0] * pM3DSurface->m_EigenValues[0];
						vartF2By = pM3DSurface->m_EigenValues[1] * pM3DSurface->m_EigenValues[1];
						vartF2Bz = pM3DSurface->m_sigmaR;

						Q[0] = vartF2Bx*xTF2[0]*xTF2[0] + vartF2By*xTF2[1]*xTF2[1] + vartF2Bz*xTF2[2]*xTF2[2] +
							vartFAx*xTF[0]*xTF[0] + vartFAy*xTF[1]*xTF[1] + vartFAz*xTF[2]*xTF[2];
						Q[1] = vartF2Bx*xTF2[0]*yTF2[0] + vartF2By*xTF2[1]*yTF2[1] + vartF2Bz*xTF2[2]*yTF2[2] +
							vartFAx*xTF[0]*yTF[0] + vartFAy*xTF[1]*yTF[1] + vartFAz*xTF[2]*yTF[2];
						Q[2] = vartF2Bx*yTF2[0]*yTF2[0] + vartF2By*yTF2[1]*yTF2[1] + vartF2Bz*yTF2[2]*yTF2[2] +
							vartFAx*yTF[0]*yTF[0] + vartFAy*yTF[1]*yTF[1] + vartFAz*yTF[2]*yTF[2];

						tFA = pS3DSurface->m_Pose.m_X;

						tF2B = pM3DSurface->m_Pose.m_X;

						// tFB = RAB * tFA

						RVLMULMX3X3VECT(R, tFA, tFB)

						// d = [1 0 0; 0 1 0] * (tFB - tF2B)

						dx = tFB[0] - tF2B[0];
						dy = tFB[1] - tF2B[1];

						// eF = d' * inv(Q) * d

						eF = (Q[2]*dx*dx - 2*Q[1]*dx*dy + Q[0]*dy*dy)/(Q[0]*Q[2] - Q[1]*Q[1]);

						if(eF > 9.21)
							continue;

						//fprintf(fpLastDOF, "M%d-S%d\n", pM3DSurface->m_Index, pS3DSurface->m_Index);

						// k1 = 1 / VA' * zFA

						k1 = RVLMULROWCOL3(zTA, RFA, 0, 2);

						if(k1 > -APPROX_ZERO && k1 < APPROX_ZERO)
							continue;

						k1 = 1.0 / k1;

						// tF2A = RAB' * tF2B

						RVLMULMX3X3TVECT(R, tF2B, tF2A)

						// tF2_FA = tF2A - tFA

						RVLDIF3VECTORS(tF2A, tFA, tF2_FA)

						// Vect3Tmp = k1 * (tF2_FA - k1 * zTA)

						Vect3Tmp[0] = k1 * (tF2_FA[0] - k1 * zTA[0]);
						Vect3Tmp[1] = k1 * (tF2_FA[1] - k1 * zTA[1]);
						Vect3Tmp[2] = k1 * (tF2_FA[2] - k1 * zTA[2]);

						// JsTD = Vec3Tmp' * [xFA, yFA]

						JsTD[0] = RVLMULROWCOL3(Vect3Tmp, RFA, 0, 0);
						JsTD[1] = RVLMULROWCOL3(Vect3Tmp, RFA, 0, 1);

						// varTD = JsTD * Cs * JsTD'
						// Computation of varTD is explained in Appendix B of IJRR13, where varTD corresponds to the symbol sigma_l in the last equation in the Appendix B.

						//varTD = JsTD[0] * JsTD[0] * pS3DSurface->m_varq[0] + 
						//	JsTD[1] * JsTD[1] * pS3DSurface->m_varq[1];

						varTD = k1 * k1 * (pS3DSurface->m_sigmaR + pM3DSurface->m_sigmaR);

						if(varTD > 90000.0)
							continue;

						// TraveledDist = k1 * tF2_FA * zFA
						// Computation of TraveledDist is explained in Appendix B of IJRR13, where TraveledDist corresponds to the symbol l with cap.

						TraveledDist = k1 * RVLMULROWCOL3(tF2_FA, RFA, 0, 2);

						wTD = 1.0 / sqrt(2.0 * varTD);			

						TDScore = wTD * (double)(pS3DSurface->m_nSupport <= pM3DSurface->m_nSupport ? 
							pS3DSurface->m_nSupport : pM3DSurface->m_nSupport);

						//if(fabs(TraveledDist) > fmaxTraveledDist)
						//	continue;

						//iTraveledDistBin = DOUBLE2INT(TraveledDist / fTraveledDistBinSize);

						//TraveledDistBin[iTraveledDistBin] += 
						//	((pS3DSurface->m_nSupport + pM3DSurface->m_nSupport) / DOUBLE2INT(varTD));

						//if(TraveledDistBin[iTraveledDistBin] > BestTravelDistScore)
						//{
						//	BestTravelDistScore = TraveledDistBin[iTraveledDistBin];

						//	iBestTraveledDistBin = iTraveledDistBin;
						//}

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
						fprintf(fpLog, "M%d-S%d\t%lf\t%lf\n", pM3DSurface->m_Index, pS3DSurface->m_Index, 
							TraveledDist, sqrt(varTD));
#endif

#ifdef NEVER
						bLastDOFMatchStored = bLastDOFMatchMerged = FALSE;

						for(iLastMSMatch = 0; iLastMSMatch < nLastDOFMatches; iLastMSMatch++)
						{
							pLastDOFMatchData = LastDOFMatchArray + iLastMSMatch;

							if(pLastDOFMatchData->pMatch == NULL)
								continue;

							eTD = TraveledDist - pLastDOFMatchData->TraveledDist;

							if(eTD < -50.0 || eTD > 50.0)
								continue;

							if(TDScore > pLastDOFMatchData->score)
							{
								if(bLastDOFMatchStored)
									pLastDOFMatchData->pMatch = NULL;
								else
								{
									pLastDOFMatchData->pMatch = pMSMatch;

									pLastDOFMatchData->score = TDScore;

									pLastDOFMatchData->TraveledDist = TraveledDist;

									iNewLastDOFMatch = iLastMSMatch;

									bLastDOFMatchStored = TRUE;
								}
							}
							else
							{
								bLastDOFMatchMerged = TRUE;

								if(bLastDOFMatchStored)
									LastDOFMatchArray[iNewLastDOFMatch].pMatch = NULL;
							}
						}

						if(!bLastDOFMatchStored && !bLastDOFMatchMerged)
						{
							pLastDOFMatchData = LastDOFMatchArray + nLastDOFMatches;

							pLastDOFMatchData->pMatch = pMSMatch;

							pLastDOFMatchData->score = TDScore;

							pLastDOFMatchData->TraveledDist = TraveledDist;
					
							nLastDOFMatches++;
						}
#endif

						pLastDOFMatchData = LastDOFMatchArray + nLastDOFMatches;

						pLastDOFMatchData->pMatch = pMSMatch;

						pLastDOFMatchData->Support = TDScore;

						pLastDOFMatchData->TraveledDist = TraveledDist;

						pLastDOFMatchData->wTD = wTD;
				
						nLastDOFMatches++;

					}

					if(nLastDOFMatches > 0)
					{
						pLastDOFMatchArrayEnd = LastDOFMatchArray + nLastDOFMatches;

						for(pLastDOFMatchData = LastDOFMatchArray; pLastDOFMatchData < pLastDOFMatchArrayEnd; pLastDOFMatchData++)
						{
							TDScore = 0.0;

							for(pLastDOFMatchData2 = LastDOFMatchArray; pLastDOFMatchData2 < pLastDOFMatchArrayEnd; pLastDOFMatchData2++)
							{
								eTD = pLastDOFMatchData2->wTD * (pLastDOFMatchData->TraveledDist - pLastDOFMatchData2->TraveledDist);

								TDScore += (pLastDOFMatchData2->Support * exp(-eTD * eTD));

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
								fprintf(fpLog, "%lf\t", pLastDOFMatchData2->Support * exp(-eTD * eTD));
#endif
							}

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
							fprintf(fpLog, "|\t%lf\n", TDScore);
#endif

							pLastDOFMatchData->score = TDScore;
						}

						// find the best Last DOF match, use it to complete a hypothesis 
						// and insert this hypothesis into m_HypothesisList

						while(TRUE)
						{
							BestTDScore = 0.0;

							pBestLastDOFMatch = NULL;

							for(pLastDOFMatchData = LastDOFMatchArray; pLastDOFMatchData < pLastDOFMatchArrayEnd; pLastDOFMatchData++)
							{
								if(pLastDOFMatchData->score > BestTDScore)
								{
									BestTDScore = pLastDOFMatchData->score;

									pBestLastDOFMatch = pLastDOFMatchData;
								}
							}

							if(pBestLastDOFMatch == NULL)
								break;

							pMSMatch = pBestLastDOFMatch->pMatch;

							if(pMSMatch == NULL)
								break;

							pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
							pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
							fprintf(fpLog, "Last DOF match: M%d-S%d\n", pM3DSurface->m_Index, pS3DSurface->m_Index);
#endif

							if(pHypothesis == NULL)
							{
								RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);

								pPoseSM2 = &(pHypothesis->PoseSM);

								pPoseSM2->m_C = pHypothesis->P;
								pPoseSM2->m_pData = pHypothesis->invt;
							}
							else
								pPoseSM2 = &(pHypothesis->PoseSM);

							if(!RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, pPoseSM, pPoseSM2, &MatchData))
							{
								pBestLastDOFMatch->score = -1.0;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
								fprintf(fpLog, "The match doesn't satisfy the geometric criterion!\n\n");
#endif
								continue;
							}

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
							fprintf(fpLog, "Hypothesis %d: R=(%lf, %lf, %lf) t=(%lf, %lf, %lf)\n", 
								HypothesisIndex,
								pPoseSM2->m_Alpha * RAD2DEG,
								pPoseSM2->m_Beta * RAD2DEG,
								pPoseSM2->m_Theta * RAD2DEG,
								pPoseSM2->m_X[0],
								pPoseSM2->m_X[1],
								pPoseSM2->m_X[2]);

							RVLPrintCov(fpLog, pPoseSM2->m_C, 3, RAD2DEG);

							RVLPrintCov(fpLog, pPoseSM2->m_C + 2 * 3 * 3, 3);

							fprintf(fpLog, "\n");							
#endif

							//R2 = pPoseSM2->m_Rot;
							//t2 = pPoseSM2->m_X;
							//RVLCOPY3VECTOR(t, t2);
							//R2 = pPoseSM2->m_Rot;
							//RVLCOPYMX3X3(R, R2);
							//P3 = pPoseSM2->m_C;
							//P = pPoseSM->m_C;
							//RVLCOPYMX3X3(P, P3);
							//P += (3 * 3);
							//P3 += (3 * 3);
							//RVLCOPYMX3X3(P, P3);
							//P += (3 * 3);
							//P3 += (3 * 3);
							//RVLCOPYMX3X3(P, P3);
							//invt = (double *)(pPoseSM->m_pData);
							//invt2 = (double *)(pPoseSM2->m_pData);
							//RVLCOPY3VECTOR(invt, invt2);
							//pPoseSM2->m_Alpha = pPoseSM->m_Alpha;
							//pPoseSM2->m_Beta = pPoseSM->m_Beta;
							//pPoseSM2->m_Theta = pPoseSM->m_Theta;
							//pPoseSM2->m_ca = pPoseSM->m_ca;
							//pPoseSM2->m_sa = pPoseSM->m_sa;
							pHypothesis->Index = (HypothesisIndex++);
							pHypothesis->cost = 0;
							pHypothesis->visible = 0;
							pHypothesis->iS = pHypothesis->iM = 0;
							pHypothesis->pMPSuLM = pMPSuLM;
							pHypothesis->Flags = 0x00;
							RVLCOPY3VECTOR(zTB, pHypothesis->zTB)

							pHypothesis->nMatches = pNode->g;

							RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, CRVL3DSurface2 *, pHypothesis->nMatches, pHypothesis->MatchArray);

							ppMatchedSSurf = (CRVL3DSurface2 **)(pHypothesis->MatchArray);

							pNode2 = pNode;

							while(pNode2)
							{
								pMSMatch = MatchList + pNode2->iMatch;

								*(ppMatchedSSurf++) = (CRVL3DSurface2 *)(pMSMatch->pSData);

								pNode2 = pNode2->pParent;
							}

							m_HypothesisList.Add(pHypothesis);	

							*(ppHypothesis++) = pHypothesis;

							pHypothesis = NULL;

							iHypothesis++;

							break;
						}	// for each Last DOF match
					}	// if(nLastDOFMatches > 0)
					else
					{
#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
						fprintf(fpLog, "No appropriate last DOF matches!\n\n");
#endif		
					}
				}	// if(bNewHypothesis)
			}	// if pNode satisfies the geometric criterion
			else
				pPNode->nFailures++;

			//pNode = (RVLPSULM_HG_NODE *)(pNode->pNext);

			//if(pNode == NULL)
			//{
			//	do	
			//		minCost++;
			//	while(QueueListArray[minCost].pFirst == NULL);

			//	pNode = (RVLPSULM_HG_NODE *)(QueueListArray[minCost].pFirst);
			//}
		}	// for each hypothesis	

		double ExecutionTime = m_pTimer->GetTime() - StartTime;

   		if(pPrevSPSuLM)
			break;
	}	// for each model PSuLM	

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
	fclose(fpLog);
#endif

	//delete[] MatchList;
	//delete[] NodeMem;
	//delete[] HypothesisArray;

	m_pMem2->Clear();

	// only for debugging purpose

//#ifndef RVLPSULMBUILDER_HYPOTHESES_DEBUG
//	CRVLFigure *pFig;
//
//	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData;
//#endif
//
//	m_HypothesisList.Start();
//
//	while(m_HypothesisList.m_pNext)
//	{
//		pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());
//
//		if(pHypothesis->Index == 0)
//		{
//			pSPSuLM->Display(m_DebugData.pGUI, &(pHypothesis->PoseSM));
//
//			pHypothesis->pMPSuLM->AddToFigure(m_DebugData.pGUI);
//
//			pFig = m_DebugData.pGUI->OpenFigure("PSuLM");
//
//			pMouseCallbackData = (RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);
//
//			pMouseCallbackData->MatrixSceneModel = NULL;	
//
//			//pMouseCallbackData->MSurfMatchData = MSurfMatchData;
//			//pMouseCallbackData->SSurfMatchData = SSurfMatchData;
//
//			pMouseCallbackData->pSSurf = NULL;
//			pMouseCallbackData->pMSurf = NULL;
//		}
//		else
//		{
//			pMouseCallbackData->pMPSuLM = pHypothesis->pMPSuLM;
//		
//			memcpy(pMouseCallbackData->PoseS0.m_Rot, pHypothesis->PoseSM.m_Rot, 3 * 3 * sizeof(double));
//			memcpy(pMouseCallbackData->PoseS0.m_X, pHypothesis->PoseSM.m_X, 3 * sizeof(double));
//
//			pMouseCallbackData->Flags = RVLPSULM_DISPLAY_SURFACES;
//
//			RVLPSuLMDisplay(pFig);
//		}
//
//		cvWaitKey();
//	}
}



RVLPSULM_MSMATCH_DATA *CRVLPSuLMBuilder::HypothesesGetNextNode(
	RVLPSULM_HG_NODE *pNode,					// current node
	int iStartMatch,							// idx. of the match from which the search for the next node starts
	RVLPSULM_MSMATCH_DATA *MatchList,
	RVLPSULM_MSMATCH_DATA *pMSMatchListEnd,
	int &PredictedScore)
{
	// Set the flags RVLOBJ2_FLAG_MATCHED of all M and S surfaces participating in the matches along the path
	// from the root of the tree to the current node pNode (including that node),
	// thereby excluding all matches with these surfaces from possible inclusion in the hypothesis

	RVLPSULM_HG_NODE *pNode2 = pNode;

	RVLPSULM_MSMATCH_DATA *pMSMatch;

	while(pNode2)
	{
		pMSMatch = MatchList + pNode2->iMatch;

		((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Flags |= RVLOBJ2_FLAG_MATCHED;

		((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Flags |= RVLOBJ2_FLAG_MATCHED;

		pNode2 = pNode2->pParent;
	}

	// Starting from the match with the index iStartMatch, search for the first match in MatchList
	// whose both M and S surface do not have flag RVLOBJ2_FLAG_MATCHED set. 
	// pMSMatch0 <- the result of this search.
	// PredictedScore <- pMSMatch->Support
	// If the end of the MatchList is reached without finding a match, then pMSMatch0 <- NULL

	RVLPSULM_MSMATCH_DATA *pMSMatch0 = NULL;

	//int iPrediction;
	//int iPrediction = 0;

	//PredictedScore = 0;

	CRVL3DSurface2 *pS3DSurface, *pM3DSurface;

	for(pMSMatch = MatchList + iStartMatch; 
		pMSMatch < pMSMatchListEnd /*&& iPrediction < RVLPSULMBUILDER_HYPOTHESES_PREDICTION_HORIZON*/; pMSMatch++)
	{
		pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

		if(pS3DSurface->m_Flags & RVLOBJ2_FLAG_MATCHED)
			continue;

		pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);

		if(pM3DSurface->m_Flags & RVLOBJ2_FLAG_MATCHED)
			continue;

		//if(iPrediction == 0)
			pMSMatch0 = pMSMatch;

		//m_MSurfPrediction[iPrediction] = pM3DSurface;

		//m_SSurfPrediction[iPrediction] = pS3DSurface;

		//iPrediction++;

		//pM3DSurface->m_Flags |= RVLOBJ2_FLAG_MATCHED;

		//pS3DSurface->m_Flags |= RVLOBJ2_FLAG_MATCHED;

		//PredictedScore += pMSMatch->Support;

		PredictedScore = pMSMatch->Support;

		break;
	}

	// reset flag RVLOBJ2_FLAG_MATCHED of all surfaces where this flag was previously set

	pNode2 = pNode;

	while(pNode2)
	{
		pMSMatch = MatchList + pNode2->iMatch;

		((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

		((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

		pNode2 = pNode2->pParent;
	}

	//int nPredictions = iPrediction;

	//for(iPrediction = 0; iPrediction < nPredictions; iPrediction++)
	//{
	//	m_MSurfPrediction[iPrediction]->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;
	//	m_SSurfPrediction[iPrediction]->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;
	//}

	return pMSMatch0;
}

int CRVLPSuLMBuilder::HypothesisDisplay(	CRVLFigure * pFig, 
											RVLPSULM_HG_NODE *pNode,
											CvPoint position,
											RVLPSULM_MSMATCH_DATA *MatchList)
{
	RVLPSULM_HG_NODE *pNode2 = pNode;

	int iLine = 1;

	char Text[200];
	RVLPSULM_MSMATCH_DATA *pMSMatch;

	while(pNode2)
	{
		pMSMatch = MatchList + pNode2->iMatch;

		sprintf(Text, "M%d-S%d", ((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Index, 
			((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Index);

		pFig->PutText(Text, cvPoint(position.x, position.y + iLine * pFig->m_FontSize), cvScalar(0, 0, 0));

		iLine++;

		pNode2 = pNode2->pParent;
	}

	return iLine - 1;
}



void CRVLPSuLMBuilder::PrintHypothesis(		FILE *fp, 
											RVLPSULM_HG_NODE *pNode,
											RVLPSULM_MSMATCH_DATA *MatchList)
{
	RVLPSULM_HG_NODE *pNode2 = pNode;

	RVLPSULM_MSMATCH_DATA *pMSMatch;

	while(pNode2)
	{
		pMSMatch = MatchList + pNode2->iMatch;

		fprintf(fp, "M%d-S%d   ", ((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Index,
			((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Index);

		pNode2 = pNode2->pParent;
	}

	fprintf(fp, "\n");
}

// for Nyarko

void CRVLPSuLMBuilder::CreateMatchMatrix(	CRVLPSuLM *pSPSuLM,
											CRVLPSuLM *pMPSuLM,
											CRVL3DPose *pPoseSM,
											double OverlapCoeff,
											CRVL3DSurface2 **MatchedMSurfArray)
{
	CRVL3DSurface2 **pMatchedMSurf;

	if(MatchedMSurfArray)
		pMatchedMSurf = MatchedMSurfArray;

	if(m_MatchMatrix)
		delete[] m_MatchMatrix;

	m_MatchMatrix = new BYTE [pSPSuLM->m_n3DSurfacesTotal * pMPSuLM->m_n3DSurfacesTotal];

	//StartTime = m_pTimer->GetTime();

	memset(m_MatchMatrix, 0, pSPSuLM->m_n3DSurfacesTotal * pMPSuLM->m_n3DSurfacesTotal * sizeof(BYTE));

	double OldOvelapCoeff = m_S3DSurfaceSet.m_OverlapCoeff;

	m_S3DSurfaceSet.m_OverlapCoeff = OverlapCoeff;

	BOOL bMatch;
	int iSSurf, iMSurf;
	CRVL3DSurface2 *pS3DSurface, *pM3DSurface;
	double MatchQuality;
	double detQ;
	RVLSURFACE_MATCH_ARRAY MatchData;

	for(iSSurf = 0; iSSurf < pSPSuLM->m_n3DSurfacesTotal; iSSurf++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[iSSurf];

		bMatch = FALSE;

		if(MatchedMSurfArray)
			*pMatchedMSurf = NULL;

		pS3DSurface->m_Flags &= ~RVLOBJ2_FLAG_MARKED;

		for(iMSurf = 0; iMSurf < pMPSuLM->m_n3DSurfacesTotal; iMSurf++)
		{
			//if(iMSurf == 0 && iSSurf == 18)
			//	int debug = 0;

			pM3DSurface = pMPSuLM->m_3DSurfaceArray[iMSurf];	

			MatchQuality = -1.0;

			if(pS3DSurface->Match2(pM3DSurface, pPoseSM, MatchQuality, detQ, &MatchData))
			{
				m_MatchMatrix[iSSurf * pMPSuLM->m_n3DSurfacesTotal + iMSurf] = 2;

				if(!bMatch)
				{
					pS3DSurface->m_Flags |= RVLOBJ2_FLAG_MARKED;

					if(MatchedMSurfArray)
						*pMatchedMSurf = pM3DSurface;

					bMatch = TRUE;
				}
			}
			else if(MatchQuality <= pS3DSurface->m_epsilon1)
			{
				if(MatchQuality > 0.0)
				{
					m_MatchMatrix[iSSurf * pMPSuLM->m_n3DSurfacesTotal + iMSurf] = 1;

#ifndef RVLPSULMBUILDER_DISPLAY_MATCH_OVERLAP
					pS3DSurface->m_Flags |= RVLOBJ2_FLAG_MARKED;
#endif
				}
			}
		}

		if(MatchedMSurfArray)
			pMatchedMSurf++;
	}

	m_S3DSurfaceSet.m_OverlapCoeff = OldOvelapCoeff;

	//double ExecutionTime = m_pTimer->GetTime() - StartTime;		
}


int CRVLPSuLMBuilder::GenMatchListViaDescriptors(CRVLPSuLM * pPSuLM_S,
												 CRVLPSuLM * pPSuLM_M,
												 RVLQLIST * matchList,
												 float thr,
												 int minNoPts)
{
	int i,j;
	float intersectVal = 0.0;
	//float lbpIval = 0.0;
	for (i = 0; i < pPSuLM_S->m_n3DSurfaces; i++)
	{
		for (j = 0; j < pPSuLM_M->m_n3DSurfaces; j++)
		{
			/*intersectVal = pPSuLM_S->m_3DSurfaceArray[i]->RVLIntersectColorHistogram(pPSuLM_M->m_3DSurfaceArray[j], 5, true);
			if (intersectVal == -1.0)
				return 0;
			if ((minNoPts != -1) && ((pPSuLM_S->m_3DSurfaceArray[i]->m_noUsedColorPts < minNoPts) || (pPSuLM_M->m_3DSurfaceArray[j]->m_noUsedColorPts < minNoPts)))
				intersectVal = -1.0;*/
			//if (pPSuLM_S->m_3DSurfaceArray[i]->m_TextureLabel == pPSuLM_M->m_3DSurfaceArray[j]->m_TextureLabel)// || (abs(pPSuLM_S->m_3DSurfaceArray[i]->m_TextureLabel - pPSuLM_M->m_3DSurfaceArray[j]->m_TextureLabel) <= 1))
			//	intersectVal = 0.5;
			intersectVal = CRVL3DMeshObject::RVLIntersectHistograms(pPSuLM_S->m_3DSurfaceArray[i]->m_LBP_RIU_VAR, pPSuLM_M->m_3DSurfaceArray[j]->m_LBP_RIU_VAR, pPSuLM_S->m_3DSurfaceArray[i]->m_noUsedLbpPts, pPSuLM_M->m_3DSurfaceArray[j]->m_noUsedLbpPts, (16 + 1)*(24 + 2), m_pMem2, -1);
			if ((minNoPts != -1) && ((pPSuLM_S->m_3DSurfaceArray[i]->m_noUsedLbpPts < minNoPts) || (pPSuLM_M->m_3DSurfaceArray[j]->m_noUsedLbpPts < minNoPts)))
				intersectVal = -1.0;
			//intersectVal = sqrt(pow(intersectVal,2) + pow(lbpIval,2));
			if (intersectVal >= thr)//(thr + 0.5))
			{
				RVLQLIST_PTR_ENTRY *pListEntry;
				RVLPSULM_MSMATCH_DATA *pMatchData;
				RVLMEM_ALLOC_STRUCT(m_pMem, RVLQLIST_PTR_ENTRY, pListEntry);				//Prije je bilo m_pMem2
				RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_MSMATCH_DATA, pMatchData);
				RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_SURF_MATCH_DATA, pMatchData->pMData);
				RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_SURF_MATCH_DATA, pMatchData->pSData);
				pMatchData->pMData->p3DSurface = pPSuLM_M->m_3DSurfaceArray[j];
				pMatchData->pSData->p3DSurface = pPSuLM_S->m_3DSurfaceArray[i];
				pMatchData->Probability = intersectVal;
				pListEntry->Ptr = pMatchData;

				RVLQLIST_ADD_ENTRY(matchList, pListEntry);
			}

		}
	}
	return 1;
}

//**********Uncertainty of the camera pose mounted on a mobile robot******************
void CRVLPSuLMBuilder::DetermineUncertainty3DOFTo6DOF(double *PUncert, CRVL3DPose *pPoseCCpInit, CRVL3DPose *pPoseAAp, double *Uncert3DOF)
{
	//Variables
	int i;
	double dB, dB2;
	double Vtemp[3],Vtemp1[3],Vtemp2[3],Vtemp3[3],Vtemp4[3];
	double Mtemp[9],Mtemp1[9];
	
	//Constants
	double X[3], Y[3], Z[3];
	RVLNULL3VECTOR(X);
	RVLNULL3VECTOR(Y);
	RVLNULL3VECTOR(Z);
	X[0] = 1.0;
	Y[1] = 1.0;
	Z[2] = 1.0;

	//To be determined experimentally and defined in RVLSAS.cf2 as PoseLA
	//CRVL3DPose PoseCB; 
	//PoseCB.m_X[0] = -50.0;
	//PoseCB.m_X[1] = -1200.0;
	//PoseCB.m_X[2] = 0.0;
	//PoseCB.m_Alpha = 0.0;
	//PoseCB.m_Beta = -20.0;
	//PoseCB.m_Theta = 0.0;
	//PoseCB.UpdateRotLL();

	//Define in RVLSAS.cf2 as Robot params
	double robotA = m_RobotParams[0];
	double robotB = m_RobotParams[1];


	/********Rotation matrices********/
	double RBA[9],RCA[9],RACp[9],RCBp[9],RBCp[9];

	//Debug purposes
	//pPoseAAp->m_X[0] = 500;
	//pPoseAAp->m_X[1] = 10;
	//pPoseAAp->m_X[2] = 0;

	//Initialize RBA
	RVLNULLMX3X3(RBA);
	RBA[2] =  1.0;
	RBA[3] = -1.0;
	RBA[7] = -1.0;

	//Initialize RCA
	RVLMXMUL3X3(RBA,m_pPoseCB->m_Rot,RCA);

	//Initialize RACp
	InverseRotation(Mtemp,RCA);
	RVLMXMUL3X3(Mtemp,pPoseAAp->m_Rot,RACp);

	//Initialize RCBp
	InverseRotation(Mtemp,RBA);
	RVLMXMUL3X3(Mtemp,pPoseAAp->m_Rot,Mtemp1);
	RVLMXMUL3X3(Mtemp1,RCA,RCBp);

	//Initialize RBCp
	RVLMXMUL3X3(RACp,RBA,RBCp);

	
	/********Translation vectors********/
	double tCB[3],tCA[3],tCBp[3];
		
	//initialize tCB
	for (i = 0; i < 3; i++)
		tCB[i] = m_pPoseCB->m_X[i];

	//initialize tCA
	RVLMULMX3X3VECT(RBA, tCB, tCA);

	//initialize tCBp
	RVLMULMX3X3VECT(pPoseAAp->m_Rot, tCA, Vtemp);
	RVLSUM3VECTORS(Vtemp, pPoseAAp->m_X, Vtemp1);
	InverseRotation(Mtemp,RBA);
	RVLMULMX3X3VECT(Mtemp, Vtemp1, tCBp);
	

	/********Covariance matrice********/
	double CovAAp[9]; 
	if(Uncert3DOF==NULL)
	{
		DetermineOdometryUncertainty(CovAAp,pPoseAAp);
	}
	else
	{
		RVLCOPYMX3X3(Uncert3DOF,CovAAp);
	}

	
	/********Jacobian matrices********/
	// J11 J12 J13 J14 J15 J16 J17
	// J21 J22 J23 J24 J25 J26 J27
	double J11[9], J21[9];
	double J12[3],J13[3],J14[3],J15[3],J16[3],J17[3];
	double J22[3],J23[3],J24[3],J25[3],J26[3],J27[3];

	//J11
	RVLNULLMX3X3(J11);
	RVLOX_Z(RCA,Vtemp);
	J11[0] = Vtemp[0];
	J11[3] = Vtemp[1];
	J11[6] = Vtemp[2];


	//J12 J13 J14 J15 J16 J17
	RVLOX_X(m_pPoseCB->m_Rot,Vtemp1);
	RVLOX_Z(m_pPoseCB->m_Rot,Vtemp2);
	RVLOX_X(RCBp,Vtemp3);
	RVLOX_Z(RCBp,Vtemp4);
	
	for (i = 0; i < 3; i++)
	{
		J12[i] = (Vtemp1[i] / (2*robotB)) + (Vtemp2[i] / robotA);
		J13[i] = (Vtemp1[i] / (2*robotB)) - (Vtemp2[i] / robotA);
		J14[i] = -Vtemp1[i] / robotB;

		J15[i] = -(Vtemp3[i] / (2*robotB)) - (Vtemp4[i] / robotA);
		J16[i] = -(Vtemp3[i] / (2*robotB)) + (Vtemp4[i] / robotA);
		J17[i] = Vtemp3[i] / robotB;
	}

	//J21
	double t1[3],t2[3],t3[3],t4[3],t5[3];
	double r1[3],r2[3],r3[3],r4[3]; 

	RVLNULLMX3X3(J21);
	CrossProduct(Z,tCA,Vtemp);
	RVLMULMX3X3VECT(RACp, Vtemp, t1);
	
	InverseRotation(Mtemp,RCA);
	RVLMULMX3X3VECT(Mtemp, X, r1);
	RVLMULMX3X3VECT(Mtemp, Y, r2);
		

	for (i = 0; i < 3; i++)
	{
		J21[0 + 3*i] = t1[i];
		J21[1 + 3*i] = r1[i];
		J21[2 + 3*i] = r2[i];
	}
	
	//J22 J23 J24 J25 J26 J27
	RVLMULMX3X3VECT(RACp, Z, r3);
	RVLMULMX3X3VECT(Mtemp, Z, r4);
	
	CrossProduct(X,tCB,Vtemp);
	RVLMULMX3X3VECT(RBCp, Vtemp, t2);

	CrossProduct(Z,tCB,Vtemp);
	RVLMULMX3X3VECT(RBCp, Vtemp, t3);

	//Get m_pPoseCB->m_Rot transpose
	double RBC[9], tBC[3];
	InverseTransform3D(RBC,tBC,m_pPoseCB->m_Rot,m_pPoseCB->m_X);
	
	CrossProduct(X,tCBp,Vtemp);
	RVLMULMX3X3VECT(RBC, Vtemp, t4);

	CrossProduct(Z,tCBp,Vtemp);
	RVLMULMX3X3VECT(RBC, Vtemp, t5);

	////-t4 -t5 -r4
	//for(i = 0; i < 3; i++)
	//{
	//	t4[i] = -t4[i];
	//	t5[i] = -t5[i];
	//	r4[i] = -r4[i];
	//}

	for(i = 0; i < 3; i++)
	{
		J22[i] = t2[i]/(2*robotB) + t3[i]/robotA + r3[i]/2;
		J23[i] = t2[i]/(2*robotB) - t3[i]/robotA + r3[i]/2;
		J24[i] = -t2[i]/robotB;
		J25[i] = -t4[i]/(2*robotB) - t5[i]/robotA - r4[i]/2;
		J26[i] = -t4[i]/(2*robotB) + t5[i]/robotA - r4[i]/2;
		J27[i] = t4[i]/robotB;
	}


	//Determine Cw ie PUncert
	double A1[9], A2[9], A3[9];
	double B1[9], B2[9], B3[9];

	// [J11 * CovAAp * [J11' J21'] = [A1  A2
	//  J21]						  A2' A3]	

	//A1
	RVLCOV3DTRANSF(CovAAp, J11, A1, Mtemp);
	RVLCOMPLETESIMMX3(A1);

	//A3
	RVLCOV3DTRANSF(CovAAp, J21, A3, Mtemp);
	RVLCOMPLETESIMMX3(A3);

	//A2
	RVLMXMUL3X3(J11,CovAAp,Mtemp);
	RVLMXMUL3X3T2(Mtemp,J21,A2);

	
	// B1 = J12*J12' + J13*J13' + ... + J17*J17'
	// B2 = J12*J22' + J13*J23' + ... + J17*J27'
	// B3 = J22*J22' + J23*J23' + ... + J27*J27'

	//B1 = J12*J12'
	RVLVECMUL3X1T2(J12,J12,B1);
	//B1 += J13*J13'
	RVLVECMUL3X1T2(J13,J13,Mtemp);
	RVLSUMMX3X3(Mtemp, B1, B1);
	//B1 += J14*J14'
	RVLVECMUL3X1T2(J14,J14,Mtemp);
	RVLSUMMX3X3(Mtemp, B1, B1);
	//B1 += J15*J15'
	RVLVECMUL3X1T2(J15,J15,Mtemp);
	RVLSUMMX3X3(Mtemp, B1, B1);
	//B1 += J16*J16'
	RVLVECMUL3X1T2(J16,J16,Mtemp);
	RVLSUMMX3X3(Mtemp, B1, B1);
	//B1 += J17*J17'
	RVLVECMUL3X1T2(J17,J17,Mtemp);
	RVLSUMMX3X3(Mtemp, B1, B1);


	//B2 = J12*J22'
	RVLVECMUL3X1T2(J12,J22,B2);
	//B2 += J13*J23'
	RVLVECMUL3X1T2(J13,J23,Mtemp);
	RVLSUMMX3X3(Mtemp, B2, B2);
	//B2 += J14*J24'
	RVLVECMUL3X1T2(J14,J24,Mtemp);
	RVLSUMMX3X3(Mtemp, B2, B2);
	//B2 += J15*J25'
	RVLVECMUL3X1T2(J15,J25,Mtemp);
	RVLSUMMX3X3(Mtemp, B2, B2);
	//B2 += J16*J26'
	RVLVECMUL3X1T2(J16,J26,Mtemp);
	RVLSUMMX3X3(Mtemp, B2, B2);
	//B2 += J17*J27'
	RVLVECMUL3X1T2(J17,J27,Mtemp);
	RVLSUMMX3X3(Mtemp, B2, B2);


	//B3 = J22*J22'
	RVLVECMUL3X1T2(J22,J22,B3);
	//B3 += J23*J23'
	RVLVECMUL3X1T2(J23,J23,Mtemp);
	RVLSUMMX3X3(Mtemp, B3, B3);
	//B3 += J24*J24'
	RVLVECMUL3X1T2(J24,J24,Mtemp);
	RVLSUMMX3X3(Mtemp, B3, B3);
	//B3 += J25*J25'
	RVLVECMUL3X1T2(J25,J25,Mtemp);
	RVLSUMMX3X3(Mtemp, B3, B3);
	//B3 += J26*J26'
	RVLVECMUL3X1T2(J26,J26,Mtemp);
	RVLSUMMX3X3(Mtemp, B3, B3);
	//B3 += J27*J27'
	RVLVECMUL3X1T2(J27,J27,Mtemp);
	RVLSUMMX3X3(Mtemp, B3, B3);


	
	//Define PUncert
	for(i = 0; i < 3; i++)
	{
		PUncert[0*9 + 0 + i] = A1[0*3 + i] + B1[0*3 + i]*m_FloorUncertConst;
		PUncert[0*9 + 3 + i] = A1[1*3 + i] + B1[1*3 + i]*m_FloorUncertConst;
		PUncert[0*9 + 6 + i] = A1[2*3 + i] + B1[2*3 + i]*m_FloorUncertConst;

		PUncert[1*9 + 0 + i] = A2[0*3 + i] + B2[0*3 + i]*m_FloorUncertConst;
		PUncert[1*9 + 3 + i] = A2[1*3 + i] + B2[1*3 + i]*m_FloorUncertConst;
		PUncert[1*9 + 6 + i] = A2[2*3 + i] + B2[2*3 + i]*m_FloorUncertConst;

		PUncert[2*9 + 0 + i] = A3[0*3 + i] + B3[0*3 + i]*m_FloorUncertConst;
		PUncert[2*9 + 3 + i] = A3[1*3 + i] + B3[1*3 + i]*m_FloorUncertConst;
		PUncert[2*9 + 6 + i] = A3[2*3 + i] + B3[2*3 + i]*m_FloorUncertConst;

		
	}

	//Define  pPoseCCpInit  ie RCCp and tCCp
	RVLCombineTransform3D(RBC,tBC,RCBp,tCBp,pPoseCCpInit->m_Rot,pPoseCCpInit->m_X); 

	pPoseCCpInit->UpdatePTRLL();

	pPoseCCpInit->m_sa = sin(pPoseCCpInit->m_Alpha);
	pPoseCCpInit->m_ca = cos(pPoseCCpInit->m_Alpha);


	

}

//Determine the uncertainty of robot motion (odometry) from estimated camera motion
void CRVLPSuLMBuilder::DetermineUncertainty6DOFTo3DOF(double *Uncert3DOF, double *Uncert6DOF, CRVL3DPose *pPoseAAp)
{
	int i;

	RVLNULLMX3X3(Uncert3DOF);

	//Initialize RApA
	double RApA[9];
	InverseRotation(RApA,pPoseAAp->m_Rot);

	//Initialize RApC
	double RApC[9];
	RVLMXMUL3X3(m_pPoseAC->m_Rot,RApA,RApC);  /**/

	//Initialize skewtAC
	double skewtAC[9];
	Skew(m_pPoseAC->m_X, skewtAC); /**/


	//Define J
	double J[3*6];

	double Mtemp1[2*3], Mtemp2[2*3];;
	for(i=0;i<3;i++)
	{
		//Mtemp1[0 + i] = -RApC[1 + i*3];	//yApC
		//Mtemp1[3 + i] = -RApC[0 + i*3];	//xApC
		Mtemp1[0 + i] = -RApC[0 + i*3];	//xApC
		Mtemp1[3 + i] = -RApC[1 + i*3];	//yApC
	}
	
	MatrixMultiplication(Mtemp1, skewtAC, Mtemp2, 2, 3, 3);

	for(i=0;i<3;i++)
	{
		//row 1
		J[0 + i] = m_pPoseAC->m_Rot[2 + i*3];	//zAC  /**/
		J[3 + i] = 0;

		//row 2
		J[6 + i] = Mtemp2[0 + i];	//1st row 
		//J[9 + i] = m_pPoseAC->m_Rot[1 + i*3];	//yAC  /**/
		J[9 + i] = m_pPoseAC->m_Rot[0 + i*3];	//xAC  /**/

		//row 3
		J[12 + i] = Mtemp2[3 + i];	//2nd row
		//J[15 + i] = m_pPoseAC->m_Rot[0 + i*3];	//xAC  /**/
		J[15 + i] = m_pPoseAC->m_Rot[1 + i*3];	//yAC  /**/
	}

	//Get Cw matrix
	double Cw[6 *6];
	RVL3x3x3BlockMxTo6x6(Uncert6DOF, Cw);

	double Mtemp36[3*6];

	MatrixMultiplication(J, Cw, Mtemp36, 3, 6, 6);

	MatrixMultiplicationT(Mtemp36, J, Uncert3DOF, 3, 6, 3);


}

//Save XML map file
void CRVLPSuLMBuilder::SaveXMLMap(char* filepath, RVLQLIST *pMap)
{
	int sizeC = (m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF ? 3 * 3 : 3 * 3 * 3);

	xml_document<> doc;
	doc.name("Map");
	RVLQLIST_PTR_ENTRY* pEntry;
	RVLQLIST_PTR_ENTRY* pNeighbour;
	CRVLPSuLM* pPSuLM;
	RVLPSULM_NEIGHBOUR* pNeigh;
	xml_node<> *node;
	char* temp; //= new char[300];
	stringstream ss (stringstream::in | stringstream::out);
	//root node
	xml_node<> *rootnode =  doc.allocate_node(node_element, "Map");

	ss.clear();
	ss.str("");
	ss << m_nSubMaps;
	temp = doc.allocate_string(NULL, 10);
	ss >> temp;
	rootnode->append_attribute(doc.allocate_attribute("NoOfSubMaps", temp));

	doc.append_node(rootnode);

	xml_node<> *subnode;
	xml_node<> *subnode2;

	pEntry = (RVLQLIST_PTR_ENTRY*)pMap->pFirst;
	while(pEntry)
	{
		pPSuLM = (CRVLPSuLM*)pEntry->Ptr;
		
		//if(pPSuLM->m_Index == 409 && pPSuLM->m_iSubMap==1)  //1
		//	int gg = 0;
		//Save PSuLM model to file
		pPSuLM->Save();

		//Creating new map xml node
		node = doc.allocate_node(node_element, "Model");
		rootnode->append_node(node);
		ss.clear();
		ss.str("");
		ss << pPSuLM->m_Index;
		//doc.allocate_string(temp, 10);	//allocating string in document memory space
		temp = doc.allocate_string(NULL, 10);
		ss >> temp;
		node->append_attribute(doc.allocate_attribute("id", temp));
		//Creating sub-nodes
		node->append_node(doc.allocate_node(node_element, "ModelPath", pPSuLM->m_ModelFilePath));
		node->append_node(doc.allocate_node(node_element, "ImagePath", pPSuLM->m_FileName));
		ss.clear();
		ss.str("");
		ss << pPSuLM->m_PoseAbs->m_X[0] << "," << pPSuLM->m_PoseAbs->m_X[1] << "," << pPSuLM->m_PoseAbs->m_X[2];
		//ss << pPSuLM->m_PoseAbs->m_Alpha << "," << pPSuLM->m_PoseAbs->m_Beta << "," << pPSuLM->m_PoseAbs->m_Theta;
		for (int i = 0; i < 9; i++)
		{
			ss << "," << pPSuLM->m_PoseAbs->m_Rot[i];
		}

		temp = doc.allocate_string(NULL, 200);	//allocating string in document memory space
		ss >> temp;
		node->append_node(doc.allocate_node(node_element, "AbsolutePose", temp));
		
		subnode = doc.allocate_node(node_element, "Neighbours");
		node->append_node(subnode);
		//Adding neighbours to model node
		if(pPSuLM->m_NeighbourList)
		{
			pNeighbour = (RVLQLIST_PTR_ENTRY*)pPSuLM->m_NeighbourList->pFirst;
			while(pNeighbour)
			{
				subnode2 = doc.allocate_node(node_element, "Neighbour");
				subnode->append_node(subnode2);
				pNeigh = (RVLPSULM_NEIGHBOUR*)pNeighbour->Ptr;
				ss.clear();
				ss.str("");
				ss << pNeigh->ModelId;
				temp = doc.allocate_string(NULL,10);	//allocating string in document memory space
				ss >> temp;
				subnode2->append_node(doc.allocate_node(node_element, "Id", temp));
				ss.clear();
				ss.str("");
				ss << pNeigh->pPoseRel->m_X[0] << "," << pNeigh->pPoseRel->m_X[1] << "," << pNeigh->pPoseRel->m_X[2];
				//ss << pNeigh->pPoseRel->m_Alpha << "," << pNeigh->pPoseRel->m_Beta << "," << pNeigh->pPoseRel->m_Theta;
				for (int i = 0; i < 9; i++)
				{
					ss << "," << pNeigh->pPoseRel->m_Rot[i];
				}
				for (int i = 0; i < sizeC; i++)
				{
					ss << "," << pNeigh->pPoseRel->m_C[i];
				}
				temp = doc.allocate_string(NULL,500);	//allocating string in document memory space
				ss >> temp;
				subnode2->append_node(doc.allocate_node(node_element, "RelativePose", temp));

				pNeighbour = (RVLQLIST_PTR_ENTRY*)pNeighbour->pNext;
			}
		}



		//////example
		////subnode2 = doc.allocate_node(node_element, "Neighbour");
		////subnode->append_node(subnode2);
		////subnode2->append_node(doc.allocate_node(node_element, "id", "12345"));
		////subnode2->append_node(doc.allocate_node(node_element, "RelativePose", "Relative,pose1"));

		////subnode2 = doc.allocate_node(node_element, "Neighbour");
		////subnode->append_node(subnode2);
		////subnode2->append_node(doc.allocate_node(node_element, "id", "54321"));
		////subnode2->append_node(doc.allocate_node(node_element, "RelativePose", "Relative,pose2"));

		pEntry = (RVLQLIST_PTR_ENTRY*)pEntry->pNext;
	}

	//Saving to file
	ofstream dat(filepath);
	dat << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
	dat << doc;
	dat.close();
	//delete [] temp;
}

//Load XML map file
RVLQLIST* CRVLPSuLMBuilder::LoadXMLMap(char* filepath, CRVLMem * pMem)
{
	int sizeC = (m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF ? 3 * 3 : 3 * 3 * 3);

	RVLQLIST *pMap;
	RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST, pMap);
	RVLQLIST_INIT(pMap);
	xml_document<> doc;
	RVLQLIST_PTR_ENTRY* pEntry;
	RVLQLIST_PTR_ENTRY* pNeighbourEntry;
	CRVLPSuLM* pPSuLM;
	RVLPSULM_NEIGHBOUR* pNeigh;
	xml_node<> *node;
	char* temp;
	string tempS;
	stringstream ss (stringstream::in | stringstream::out);
	xml_node<> *rootnode;
	xml_node<> *subnode;
	xml_node<> *subnode2;
	xml_node<> *subnode3;

	ifstream dat(filepath);
	string str((istreambuf_iterator<char>(dat)), istreambuf_iterator<char>());
	char *text = new char [str.size()+1];
	strcpy (text, str.c_str());
	doc.parse<0>(text);
	//generating CRVLQLIST from XML DOM
	//root node of the map
	rootnode = doc.first_node();
	ss.clear();
	ss.str("");
	tempS =  string(rootnode->first_attribute()->value(), rootnode->first_attribute()->value_size());
	ss << tempS;
	ss >> m_nSubMaps;

	node = rootnode->first_node();
	while(node)
	{
		//allocating new map nodes
		RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pEntry);
		RVLMEM_ALLOC_STRUCT(pMem, CRVLPSuLM, pPSuLM);
		memcpy(pPSuLM, &m_PSuLMTemplate, sizeof(CRVLPSuLM));
		RVLQLIST *pLocalMap = &(pPSuLM->m_LocalMap);
		RVLQLIST_INIT(pLocalMap)
		pEntry->Ptr = pPSuLM;
		
		//geting node id
		ss.clear();
		ss.str("");
		tempS =  string(node->first_attribute()->value(), node->first_attribute()->value_size());
		ss << tempS;
		ss >> pPSuLM->m_Index;	
		//anlyzing individual info about the node
		subnode = node->first_node();
		while(subnode)
		{
			tempS = string(subnode->name(), subnode->name_size());
			if(tempS == "ModelPath")
			{
				RVLMEM_ALLOC_STRUCT_ARRAY(pMem, char, subnode->value_size()+1, pPSuLM->m_ModelFilePath);
				//memcpy(pPSuLM->m_ModelFilePath, subnode->value(), subnode->value_size());
				strcpy(pPSuLM->m_ModelFilePath, subnode->value());
			}
			else if(tempS == "ImagePath")
			{
				RVLMEM_ALLOC_STRUCT_ARRAY(pMem, char, subnode->value_size()+1,pPSuLM->m_FileName);
				//memcpy(pPSuLM->m_FileName, subnode->value(), subnode->value_size());
				strcpy(pPSuLM->m_FileName, subnode->value());
			}
			else if(tempS == "AbsolutePose")
			{
				RVLMEM_ALLOC_STRUCT(pMem, CRVL3DPose, pPSuLM->m_PoseAbs);
				tempS = string(subnode->value(), subnode->value_size());
				ss.clear();
				ss.str("");
				for (int i = 0; i < tempS.length(); i++)
				{
					if (tempS[i] == ',')
						tempS[i] = ' ';
				}
				ss << tempS;
				ss >> pPSuLM->m_PoseAbs->m_X[0];
				ss >> pPSuLM->m_PoseAbs->m_X[1];
				ss >> pPSuLM->m_PoseAbs->m_X[2];
				//ss >> pPSuLM->m_PoseAbs->m_Alpha;
				//ss >> pPSuLM->m_PoseAbs->m_Beta;
				//ss >> pPSuLM->m_PoseAbs->m_Theta;

				//pPSuLM->m_PoseAbs->UpdateRotA0();
				for (int i = 0; i < 9; i++)
				{
					ss >> pPSuLM->m_PoseAbs->m_Rot[i];
				}

			}
			else if(tempS == "Neighbours")
			{
				//List initialization
				RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST, pPSuLM->m_NeighbourList);
				RVLQLIST_INIT(pPSuLM->m_NeighbourList);
				//Anlyzing each neighbour
				subnode2 = subnode->first_node();
				while(subnode2)
				{
					RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pNeighbourEntry);
					RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_NEIGHBOUR, pNeigh);
					pNeighbourEntry->Ptr = pNeigh;
					RVLMEM_ALLOC_STRUCT(pMem, CRVL3DPose, pNeigh->pPoseRel);
					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, double, sizeC, pNeigh->pPoseRel->m_C);
					//Anlyzing each neighbour info
					subnode3 = subnode2->first_node();
					while(subnode3)
					{
						tempS = string(subnode3->name(), subnode3->name_size());
						if (tempS == "Id")
						{
							ss.clear();
							ss.str("");
							ss << string(subnode3->value(), subnode3->value_size());
							ss >> pNeigh->ModelId;
						}
						else if (tempS == "RelativePose")
						{
							tempS = string(subnode3->value(), subnode3->value_size());
							ss.clear();
							ss.str("");
							for (int i = 0; i < tempS.length(); i++)
							{
								if (tempS[i] == ',')
									tempS[i] = ' ';
							}
							ss << tempS;
							ss >> pNeigh->pPoseRel->m_X[0];
							ss >> pNeigh->pPoseRel->m_X[1];
							ss >> pNeigh->pPoseRel->m_X[2];
							//ss >> pNeigh->pPoseRel->m_Alpha;
							//ss >> pNeigh->pPoseRel->m_Beta;
							//ss >> pNeigh->pPoseRel->m_Theta;

							//pNeigh->pPoseRel->UpdateRotA0();
							for (int i = 0; i < 9; i++)
							{
								ss >> pNeigh->pPoseRel->m_Rot[i];
							}

							for (int i = 0; i < sizeC; i++)
							{
								ss >> pNeigh->pPoseRel->m_C[i];
							}
						}						
						subnode3 = subnode3->next_sibling();
					}
					//adding node to the neighbour QLIST
					RVLQLIST_ADD_ENTRY(pPSuLM->m_NeighbourList, pNeighbourEntry);
					//next neighbour
					subnode2 = subnode2->next_sibling();
				}
			}
			subnode = subnode->next_sibling();
		}
		//adding node to the map list
		RVLQLIST_ADD_ENTRY(pMap, pEntry);
		//next map node
		node = node->next_sibling();
	}
	delete [] text;
	return pMap;
}

//For conversion of CRVLPtrChains to RVLQLISTs
CRVLMPtrChain* CRVLPSuLMBuilder::ConvertQLIST2PtrChain(RVLQLIST *pQList, CRVLMem * pMem)
{
	CRVLMPtrChain *pPtrChain;
	RVLMEM_ALLOC_STRUCT(pMem, CRVLMPtrChain, pPtrChain);
	pPtrChain->m_pFirst = NULL;
	pPtrChain->m_nElements = 0;
	RVLQLIST_PTR_ENTRY *pEntry = (RVLQLIST_PTR_ENTRY*)pQList->pFirst;
	while(pEntry)
	{
		pPtrChain->Add(pEntry->Ptr);
		pEntry = (RVLQLIST_PTR_ENTRY*)pEntry->pNext;
	}
	return pPtrChain;
}

//For conversion of RVLQLISTs to CRVLPtrChains
RVLQLIST* CRVLPSuLMBuilder::ConvertPtrChain2QLIST(CRVLMPtrChain *pPtrChain, CRVLMem * pMem)
{
	RVLQLIST *pQList;
	RVLQLIST_PTR_ENTRY *pEntry;
	RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST, pQList)
	RVLQLIST_INIT(pQList);
	pPtrChain->Start();
	for (int i = 0; i < pPtrChain->m_nElements; i++)
	{
		RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pEntry);
		pEntry->Ptr = pPtrChain->GetNext();//pPtrChain->m_pCurrent->pData;
		RVLQLIST_ADD_ENTRY(pQList, pEntry);
		//pPtrChain->GetNext();
	}
	return pQList;
}

//Prediction step of uncertainty (Robot localization using a hybrid metric-topological map) 6DOF
//Cco = Pc*Cno*Pc' + Vc*Ccn*Vc' 
void CRVLPSuLMBuilder::UncertaintyEKFPrediction6DOF(double *UncertC0, double *UncertCN, double *UncertN0,  CRVL3DPose *pPoseCN, CRVL3DPose *pPoseN0)
{
	int i;

	double RCN[9], RN0[9];
	RVLCOPYMX3X3(pPoseCN->m_Rot,RCN);
	RVLCOPYMX3X3(pPoseN0->m_Rot, RN0);
	

	//Initialize RNC
	double RNC[9];
	InverseRotation(RNC,RCN);

	//Initialize R0N
	double R0N[9]; 
	InverseRotation(R0N,RN0);

	//Initialize skew of tCN
	double skewtCN[9];
	Skew(pPoseCN->m_X, skewtCN);

	//Initialize Rskew = -RN0*skewtCN
	double Rskew[9];
	RVLMXMUL3X3(RN0,skewtCN,Rskew);

	for(i=0;i<9;i++)
		Rskew[i] = -Rskew[i];

	//Initialize RskewT
	double RskewT[9];
	InverseRotation(RskewT,Rskew);


	double Mtemp1[9],Mtemp2[9];
	double A1[9],A2[9],A3[9];
	double B2[9],B3[9];
	double C1[9],C2[9],C2T[9],C3[9];


	//1. Pc*Cno*Pc'
	for(i=0;i<9;i++)
	{
		C1[i] = UncertN0[0 + i];
		C2[i] = UncertN0[9 + i];
		C3[i] = UncertN0[18 + i];
	}
	
	//Initialize C2T
	InverseRotation(C2T,C2);


	//A1 = RNC*C1*RNC'
	RVLMXMUL3X3(RNC,C1,Mtemp1);
	RVLMXMUL3X3(Mtemp1,RCN,A1);

	//A2 = RNC*C1*Rskew' + RNC*C2
	RVLMXMUL3X3(Mtemp1,RskewT,Mtemp2);
	RVLMXMUL3X3(RNC,C2,Mtemp1);
	RVLSUMMX3X3(Mtemp1,Mtemp2,A2);

	//A3 = (Rskew*C1 + C2')*Rskew' + (Rskew*C2 + C3)
	RVLMXMUL3X3(Rskew,C1,Mtemp1);
	RVLSUMMX3X3(Mtemp1,C2T,Mtemp1);
	RVLMXMUL3X3(Mtemp1,RskewT,A3);

	RVLMXMUL3X3(Rskew,C2,Mtemp1);
	RVLSUMMX3X3(A3,Mtemp1,A3);
	RVLSUMMX3X3(A3,C3,A3);

	//2. Vc*Ccn*Vc'
	for(i=0;i<9;i++)
	{
		C1[i] = UncertCN[0 + i];
		C2[i] = UncertCN[9 + i];
		C3[i] = UncertCN[18 + i];
	}

	//B2 = C2*RN0'
	RVLMXMUL3X3(C2,R0N,B2);

	//B3 = RN0*C3*RN0'
	RVLMXMUL3X3(RN0,C3,Mtemp1);
	RVLMXMUL3X3(Mtemp1,R0N,B3);


	for(i=0;i<9;i++)
	{
		UncertC0[0 + i]  = A1[i] + C1[i];
		UncertC0[9 + i]  = A2[i] + B2[i];
		UncertC0[18 + i] = A3[i] + B3[i];
	}

}




//Prediction step of uncertainty (Robot localization using a hybrid metric-topological map) 3DOF
//Cco = Phi*Cno*Phi' + V*Ccn*V' 
void CRVLPSuLMBuilder::UncertaintyEKFPrediction3DOF(double *UncertC0, double *UncertCN, double *UncertN0,  CRVL3DPose *pPoseCN, CRVL3DPose *pPoseN0)
{
	//Just in case
	//pPoseN0->UpdatePTRLL();

	int i;
	RVLNULLMX3X3(UncertC0);

	double cosA = pPoseN0->m_Rot[0];
	double sinA = pPoseN0->m_Rot[3];

	double Phi[9];
	Phi[0] = 1;
	Phi[1] = 0;
	Phi[2] = 0;
	Phi[3] = -(pPoseCN->m_X[1]*cosA + pPoseCN->m_X[0]*sinA);
	Phi[4] = 1;
	Phi[5] = 0;
	Phi[6] = pPoseCN->m_X[0]*cosA - pPoseCN->m_X[1]*sinA;
	Phi[7] = 0;
	Phi[8] = 1;

	double PhiT[9];
	InverseRotation(PhiT,Phi);


	double V[9];
	V[0] = 1;
	V[1] = 0;
	V[2] = 0;
	V[3] = 0;
	V[4] = cosA;
	V[5] = -sinA;
	V[6] = 0;
	V[7] = sinA;
	V[8] = cosA;

	double VT[9];
	InverseRotation(VT,V);


	double CN0[9], CCN[9];

	for(i=0;i<9;i++)
	{
		CN0[i] = UncertN0[i];
		CCN[i] = UncertCN[i];
	}


	double Mtemp[9], Mtemp1[9], Mtemp2[9];

	//Phi*Cno*Phi'
	RVLMXMUL3X3(Phi,CN0,Mtemp);
	RVLMXMUL3X3(Mtemp,PhiT,Mtemp1);

	//V*Ccn*V'
	RVLMXMUL3X3(V,CCN,Mtemp);
	RVLMXMUL3X3(Mtemp,VT,Mtemp2);

	RVLSUMMX3X3(Mtemp1,Mtemp2,UncertC0);



}


//Uncert(B->A) = Jac(AB->BA)*Uncert(A->B)*Jac(AB->BA)'
void CRVLPSuLMBuilder::InvUncertainty(double *UncertBA,double *UncertAB, CRVL3DPose *pPoseAB)
{
	int i;
	
	//Invert pose
	double RBA[9], tBA[3];

	InverseRotation(RBA,pPoseAB->m_Rot);

	RVLMULMX3X3VECT(RBA,pPoseAB->m_X,tBA);
	for(i = 0; i < 3; i++)
		tBA[i] = -tBA[i];


	double tBAskew[9];
	Skew(tBA, tBAskew);
	
	//Initialize skewtBAT
	double tBAskewT[9];
	InverseRotation(tBAskewT,tBAskew);

	

	double Mtemp1[9],Mtemp2[9];
	double A1[9], A2[9], A3[9];
	double C1[9], C2[9], C2T[9], C3[9];


	//Define uncertainty sub matrices
	for(i=0;i<9;i++)
	{
		C1[i] = UncertAB[0 + i];
		C2[i] = UncertAB[9 + i];
		C3[i] = UncertAB[18 + i];
	}
	InverseRotation(C2T,C2);

	//A1 = RBA'*C1*RBA
	RVLMXMUL3X3(pPoseAB->m_Rot,C1,Mtemp1);
	RVLMXMUL3X3(Mtemp1,RBA,A1);

	//A2 = RBA'*C2*RBA - RBA'*C1*tBAskew'
	RVLMXMUL3X3(Mtemp1,tBAskewT,Mtemp2);
	for(i=0;i<9;i++)
		A2[i] = -Mtemp2[i]; 

	RVLMXMUL3X3(pPoseAB->m_Rot,C2,Mtemp1);
	RVLMXMUL3X3(Mtemp1,RBA,Mtemp2);

	RVLSUMMX3X3(A2,Mtemp2,A2);

	//A3 = (tBAskew*C1 - RBA*C2')*tBAskew' + (RBA*C3 - tBAskew*C2)*RBA'
	RVLMXMUL3X3(tBAskew,C1,Mtemp1);
	RVLMXMUL3X3(RBA,C2T,Mtemp2);
	for(i=0;i<9;i++)
		Mtemp1[i] = Mtemp1[i] - Mtemp2[i];


	RVLMXMUL3X3(Mtemp1,tBAskewT,A3);

	RVLMXMUL3X3(RBA,C3,Mtemp1);
	RVLMXMUL3X3(tBAskewT,C2,Mtemp2);
	for(i=0;i<9;i++)
		Mtemp1[i] = Mtemp1[i] - Mtemp2[i];

	RVLMXMUL3X3(Mtemp1,pPoseAB->m_Rot,Mtemp2);

	for(i=0;i<9;i++)
		A3[i] += Mtemp2[i];


	//Copy final matrices
	for(i=0;i<9;i++)
	{
		UncertBA[0 + i]  = A1[i];
		UncertBA[9 + i]  = A2[i];
		UncertBA[18 + i] = A3[i];
	}

}


void CRVLPSuLMBuilder::DetermineOdometryUncertainty(double *CovAB, CRVL3DPose *pPoseAB)
{
	//initialize odometry uncertainty, Cov
	RVLNULLMX3X3(CovAB);
	
	double dist, dB, dB2;

	dist = sqrt(pPoseAB->m_X[0]*pPoseAB->m_X[0] + pPoseAB->m_X[1]*pPoseAB->m_X[1]);

	double alpha = fabs(atan2(pPoseAB->m_Rot[3], pPoseAB->m_Rot[0]));

	//dB = K1*delta_dist + K2*delta_alpha
	dB = m_OdometryUncertConst[0]*dist + m_OdometryUncertConst[1]*alpha + 2500.0; 
	//dB2 = K3*delta_dist + K4*delta_alpha
	dB2 = m_OdometryUncertConst[2]*dist + m_OdometryUncertConst[3]*alpha + 0.5 * 0.5 * DEG2RAD * DEG2RAD; 

	CovAB[0] = dB2;
	CovAB[4] = dB;
	CovAB[8] = dB;
	
	//Debug purposes
	//CovAAp[0] = 1;
	//CovAAp[4] = 1;
	//CovAAp[8] = 1;
	//m_FloorUncertConst = 1.0;
	
}

CRVLPSuLM *CRVLPSuLMBuilder::Clone(CRVLPSuLM *pPSulMOriginal)
{
	m_maxnLines = 0;

	CRVLPSuLM *pPSuLM = (CRVLPSuLM *)(m_pMem0->Alloc(sizeof(CRVLPSuLM)));

	memcpy(pPSuLM, &m_PSuLMTemplate, sizeof(CRVLPSuLM));

	RVLQLIST *pLocalMap = &(pPSuLM->m_LocalMap);
	RVLQLIST_INIT(pLocalMap)

	//Allocate mem for m_PoseAbs 
	RVLMEM_ALLOC_STRUCT(m_pMem0, CRVL3DPose, pPSuLM->m_PoseAbs);

	//Define index
	pPSuLM->m_Index = (WORD)(m_maxPSuLMIndex + 1);

	m_maxPSuLMIndex++;
	
	pPSuLM->m_SurfaceList.m_pMem = m_pMem0;
	pPSuLM->m_2DLineList.m_pMem = m_pMem0;
	pPSuLM->m_3DLineList.m_pMem = m_pMem0;

	pPSuLM->m_CellArray = (RVLPSULM_CELL *)(m_pMem0->Alloc(m_nCells * sizeof(RVLPSULM_CELL)));

	pPSuLM->m_PoseRTAs.m_C = pPSuLM->m_CRTAs;
	pPSuLM->m_PoseCsInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	pPSuLM->m_PoseCsInit.m_C = pPSuLM->m_CCsInit;

	//Copy strings
	int iFileNameLength = (int)strlen(pPSulMOriginal->m_FileName);

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, char, iFileNameLength + 1, pPSuLM->m_FileName);
	//pPSuLM->m_FileName = (char *)(m_pMem0->Alloc((iFileNameLength + 1) * sizeof(char)));
	strcpy(pPSuLM->m_FileName, pPSulMOriginal->m_FileName);

	//Create similar model file name
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, char, iFileNameLength + 1, pPSuLM->m_ModelFilePath);
	//pPSuLM->m_ModelFilePath = (char *)(m_pMem0->Alloc(iFileNameLength * sizeof(char)));
	strcpy(pPSuLM->m_ModelFilePath, pPSulMOriginal->m_FileName);

	char *pPos;

	if ((m_Flags2 & RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT) == RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG)
	{
		pPos = strstr(pPSuLM->m_ModelFilePath, ".png");

		strncpy(pPos,".dat",4);
	}
	else
	{
		pPos = (m_Flags & RVLPSULMBUILDER_FLAG_PC ? strstr(pPSuLM->m_ModelFilePath, "PC.pcd") : strstr(pPSuLM->m_ModelFilePath, "LW.bmp"));

		strncpy(pPos, "_M.dat", 6);
	}

	//*************CLONE PSuLM************//

	//create temp file
	//FILE *fp = tmpfile();
	FILE *fp = fopen("tmp.dat", "wb+");

	//Save original model to mem. file
	pPSulMOriginal->Save(fp);

	//rewind file buffer
	rewind(fp);

	//Load mem file to new pPSuLM
	pPSuLM->Load(fp, m_Flags2);

	//close temp file
	fclose(fp);
	//************************************//

	if(pPSuLM->m_nLandmarks > m_maxnLandmarks)
		m_maxnLandmarks = pPSuLM->m_nLandmarks;

	if(pPSuLM->m_n3DLines > m_maxnLines)
		m_maxnLines = pPSuLM->m_n3DLines;

	return pPSuLM;
}

bool CRVLPSuLMBuilder::SaveCurrentData(CRVLPSuLM *pSPSuLM, 
									   CRVLPSuLM *pMPSuLM, 
									   CRVL3DPose *pBestPose, 
									   CRVL3DPose *pPoseS0,
									   int BestHypCost)
{

	bool bAdded = false;

	//Store best solution for next iteration
	pBestPose->UpdatePTRLL();


	CRVLPSuLM 	*pBestPSuLMClone;
	CRVL3DPose bestRobotPose;

	RVLDeterminePose6DOFTo3DOF(&bestRobotPose,pBestPose,m_pPoseAC);

#ifdef RVLPSULMBUILDER_MAP_DEBUG_LOG
	FILE *fpMapNodesLog;

	fpMapNodesLog = fopen("C:\\RVL\\ExpRez\\MapNodesLog.dat", "w");
#endif

	double CCsCm[3 * 3 * 3];
	RVLPSULM_NEIGHBOUR* pNeigh;
	RVLQLIST_PTR_ENTRY* pNeighbourEntry;

	//First model in a map
	if(pMPSuLM == NULL && (m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING) != 0)
	{
		PanTiltRollUncertainty(CCsCm, pBestPose->m_C, pBestPose,true);

		//Convert uncertainty back 
		memcpy(m_pPrevCameraPoseSM->m_C, CCsCm, 3 * 3 * 3 * sizeof(double));

		//Define initial absolute pose
		RVLNULLMX3X3(m_pModelAbsPose->m_Rot);
		m_pModelAbsPose->m_Rot[0] = 1.0;
		m_pModelAbsPose->m_Rot[4] = 1.0;
		m_pModelAbsPose->m_Rot[8] = 1.0;
		memset(m_pModelAbsPose->m_X, 0, 3 * sizeof(double));

		if((m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) != RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
			pSPSuLM->Get3DSurfaceSamplesFrom2DRegionSamples(m_pPSD->Sample2DRegions(), &m_S3DSurfaceSet, &(m_S3DSurfaceSet.m_ObjectList.m_pFirst));

		//Clone PSuLM
		pBestPSuLMClone = Clone(pSPSuLM);

		pBestPSuLMClone->m_iSubMap = m_nSubMaps;

		m_nSubMaps++;

		

		////If ground truth file exists
		//if(m_Flags & RVLPSULMBUILDER_FLAG_GROUNDTRUTH_EXISTS)
		//{
		//	//Get ground truth
		//	GetGroundTruth(pBestPSuLMClone);
		//}
		//else
		//{
			//Copy Absolute pose  to cloned PSuLM
			memcpy(pBestPSuLMClone->m_PoseAbs->m_X, m_pModelAbsPose->m_X, 3 * sizeof(double));
			memcpy(pBestPSuLMClone->m_PoseAbs->m_Rot, m_pModelAbsPose->m_Rot, 3 * 3 * sizeof(double));
		//}

		//m_pModelAbsPose->UpdatePTRLL();
		//pBestPSuLMClone->m_PoseAbs->m_Alpha = m_pModelAbsPose->m_Alpha;
		//pBestPSuLMClone->m_PoseAbs->m_Beta = m_pModelAbsPose->m_Beta; 
		//pBestPSuLMClone->m_PoseAbs->m_Theta = m_pModelAbsPose->m_Theta;

		//Add current PSULM to database/map
		m_PSuLMList.Add(pBestPSuLMClone);
		m_pPrevModelPSuLM = pBestPSuLMClone;

		//Save current scene to previous scene
		RVLCOPY3VECTOR(pPoseS0->m_X, m_pPoseSp0->m_X)
		RVLCOPYMX3X3(pPoseS0->m_Rot,m_pPoseSp0->m_Rot)
		m_pPoseSp0->m_Alpha = pPoseS0->m_Alpha;
		m_pPoseSp0->m_Beta = pPoseS0->m_Beta;
		m_pPoseSp0->m_Theta = pPoseS0->m_Theta;
		
		bAdded = true;

	}
	else if(m_Flags & RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING)
	{
		m_Flags &= ~RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING;

		// find the PSuLM with the same name as pSPSuLM

		CRVLPSuLM **ppMPSuLM2 = m_PSuLMArray;
		CRVLPSuLM **pPSuLMArrayEnd = m_PSuLMArray + m_maxPSuLMIndex + 1;

		CRVLPSuLM *pMPSuLM2 = NULL;

		for(ppMPSuLM2 = m_PSuLMArray; ppMPSuLM2 < pPSuLMArrayEnd; ppMPSuLM2++)
		{
			pMPSuLM2 = *ppMPSuLM2;

			if(pMPSuLM2)
				if(strcmp(pMPSuLM2->m_FileName, pSPSuLM->m_FileName) == 0)
					break;
		}

		if(pMPSuLM2)
		{
			// Create relation between previous Model PSULM and current Model PSULM before it is added to m_PSuLMList

			Connect(pMPSuLM, pMPSuLM2, &bestRobotPose, pBestPose);
		}
	}
	else
	{
		double V[3];
		double eAlpha;
		bestRobotPose.GetAngleAxis(V,eAlpha);
		//500mm * 500mm -> 250000 OR alpha>15°  400mm * 400mm -> 160000
		if( (m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING)
			&&
			(((bestRobotPose.m_X[0]*bestRobotPose.m_X[0] + bestRobotPose.m_X[1]*bestRobotPose.m_X[1]) > pow(m_MinHybridLocalizationDist - 100,2)) 
			|| 
			abs(eAlpha*RAD2DEG)>(m_MinHybridLocalizationAngle-1))
			)
		{
			//Update absolute pose for current PSULM
			double RAbs[9], tAbs[3];
			double *RAbsPrev = pMPSuLM->m_PoseAbs->m_Rot;
			double *tAbsPrev = pMPSuLM->m_PoseAbs->m_X;
			double *RRel = bestRobotPose.m_Rot;
			double *tRel = bestRobotPose.m_X;

			double alphaAbsPrev = atan2(RAbsPrev[3], RAbsPrev[0]);
			double alphaRel = atan2(RRel[3], RRel[0]);
			m_pModelAbsPose->m_Alpha = alphaAbsPrev + alphaRel;
			m_pModelAbsPose->UpdateRotA0();
			m_pModelAbsPose->m_X[0] = RAbsPrev[0] * tRel[0] + RAbsPrev[1] * tRel[1] + tAbsPrev[0];
			m_pModelAbsPose->m_X[1] = RAbsPrev[3] * tRel[0] + RAbsPrev[4] * tRel[1] + tAbsPrev[1];
			m_pModelAbsPose->m_X[2] = 0.0;
			
			//RVLCombineTransform3D(pMPSuLM->m_PoseAbs->m_Rot,pMPSuLM->m_PoseAbs->m_X,
			//	bestRobotPose.m_Rot,bestRobotPose.m_X,
			//	m_pModelAbsPose->m_Rot,m_pModelAbsPose->m_X);

#ifdef RVLPSULMBUILDER_MAP_DEBUG_LOG
			fprintf(fpMapNodesLog, "%d\t%lf\t%lf\t%lf\n", pMPSuLM->m_iSubMap,		//$
				atan2(m_pModelAbsPose->m_Rot[3], m_pModelAbsPose->m_Rot[0]) * RAD2DEG, 
				m_pModelAbsPose->m_X[0], m_pModelAbsPose->m_X[1]);
#endif
			if((m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) != RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
				pSPSuLM->Get3DSurfaceSamplesFrom2DRegionSamples(m_pPSD->Sample2DRegions(), &m_S3DSurfaceSet, &(m_S3DSurfaceSet.m_ObjectList.m_pFirst));

			//Clone PSuLM
			pBestPSuLMClone = Clone(pSPSuLM);

			pBestPSuLMClone->m_iSubMap = pMPSuLM->m_iSubMap;	//$

			////If ground truth file exists
			//if(m_Flags & RVLPSULMBUILDER_FLAG_GROUNDTRUTH_EXISTS)
			//{
			//	//Get ground truth
			//	GetGroundTruth(pBestPSuLMClone);
			//}
			//else
			//{
				//Copy Absolute pose  to cloned PSuLM
				memcpy(pBestPSuLMClone->m_PoseAbs->m_X, m_pModelAbsPose->m_X, 3 * sizeof(double));
				memcpy(pBestPSuLMClone->m_PoseAbs->m_Rot, m_pModelAbsPose->m_Rot, 3 * 3 * sizeof(double));
			//}
			//m_pModelAbsPose->UpdatePTRLL();
			//pBestPSuLMClone->m_PoseAbs->m_Alpha = m_pModelAbsPose->m_Alpha;
			//pBestPSuLMClone->m_PoseAbs->m_Beta = m_pModelAbsPose->m_Beta; 
			//pBestPSuLMClone->m_PoseAbs->m_Theta = m_pModelAbsPose->m_Theta;

			//Create relations between the new PSuLM and the other models with overlapping field of view (Loop closing)

			// Detect all models in m_PSuLMSubList whose matching score with pSPSuLM is >= 5 
			// and put them into the list OFVModelList

			CRVLPSuLM **OFVModelList = new CRVLPSuLM *[m_PSuLMSubList.m_nElements];

			CRVLPSuLM **ppOFVModel = OFVModelList;

			RVLPSULM_HYPOTHESIS *pHypothesis;
			CRVLPSuLM *pMPSuLM2;

			if(m_HypothesisList.m_nElements > 0)
			{
				m_HypothesisList.Start();

				while(m_HypothesisList.m_pNext)
				{
					pHypothesis = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

					if(pHypothesis->cost >= 5)
					{
						pMPSuLM2 = pHypothesis->pMPSuLM;

						if((pMPSuLM2->m_Flags & RVLPSULM_FLAG_TARGET) == 0)
						{
							pMPSuLM2->m_Flags |= RVLPSULM_FLAG_TARGET;

							pMPSuLM2->m_pHypothesis = pHypothesis;

							*(ppOFVModel++) = pMPSuLM2;
						}
						else if(((m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) == 
							RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM && 
							pHypothesis->cost < pMPSuLM2->m_pHypothesis->cost) ||
							((m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) !=
							RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_IBM &&
							pHypothesis->cost > pMPSuLM2->m_pHypothesis->cost))
							pMPSuLM2->m_pHypothesis = pHypothesis;
					}
				}
			}
			else
				*(ppOFVModel++) = pMPSuLM;

			// Compute the poses of all models in OFVModelList relative to pSPSuLM

			int nOFVModels = ppOFVModel - OFVModelList;

			int iTmp;

			GetModelPoses(pBestPose, pMPSuLM, NULL, iTmp, nOFVModels);

			// For all models in OFVModelList

			CRVLPSuLM **pOFVModelListEnd = ppOFVModel;

			CRVL3DPose PoseAsAs;

			double RAsAs[3 * 3];
			double tAsAs[3];
			double CAsAs[3 * 3];
			double *RAmAs, *tAmAs, *RAsAm, *tAsAm;
			double e[3];
			double dM;
			CRVL3DPose PoseAsAm;
			CRVL3DPose *pPoseCsCm, *pPoseAsAm;

			for(ppOFVModel = OFVModelList; ppOFVModel < pOFVModelListEnd; ppOFVModel++)
			{
				pMPSuLM2 = *ppOFVModel;

				pMPSuLM2->m_Flags &= ~RVLPSULM_FLAG_TARGET;

				if(pMPSuLM2 == pMPSuLM)
				{
					pPoseCsCm = pBestPose;
					pPoseAsAm = &bestRobotPose;
				}
				else
				{
					RVLDeterminePose6DOFTo3DOF(&PoseAsAm,
						&(pMPSuLM2->m_pHypothesis->PoseSM),m_pPoseAC);

					// Check consistency with the best-match-model

					RAmAs = pMPSuLM2->m_PoseRTAs.m_Rot;
					tAmAs = pMPSuLM2->m_PoseRTAs.m_X;
					RAsAm = PoseAsAm.m_Rot;
					tAsAm = PoseAsAm.m_X;

					RVLCOMPTRANSF3D(RAmAs, tAmAs, RAsAm, tAsAm, RAsAs, tAsAs);

					pPoseAsAm = &PoseAsAm;
					pPoseCsCm = &(pMPSuLM2->m_pHypothesis->PoseSM);

					UncertaintyEKFPrediction3DOF(CAsAs, pMPSuLM2->m_pHypothesis->PoseSM.m_C, pMPSuLM2->m_PoseRTAs.m_C, 
						&(pMPSuLM2->m_pHypothesis->PoseSM), &(pMPSuLM2->m_PoseRTAs));

					e[0] = atan2(RAsAs[3], RAsAs[0]);
					e[1] = tAsAs[0];
					e[2] = tAsAs[1];

					dM = RVLMahalanobisDistance3D(e, CAsAs);

					if(dM > 6.2514)	// chi2inv(0.90, 3)
						continue;
				}

				// Create relation between previous Model PSULM and current Model PSULM before it is added to m_PSuLMList

				Connect(pMPSuLM2, pBestPSuLMClone, pPoseAsAm, pPoseCsCm);

				if(pMPSuLM2->m_NeighbourList==NULL)	//$
				{
					RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLIST, pMPSuLM2->m_NeighbourList);	//$
					RVLQLIST_INIT(pMPSuLM2->m_NeighbourList);					//$
				}
				RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLIST_PTR_ENTRY, pNeighbourEntry);
				RVLMEM_ALLOC_STRUCT(m_pMem0, RVLPSULM_NEIGHBOUR, pNeigh);
				pNeighbourEntry->Ptr = pNeigh;
				RVLMEM_ALLOC_STRUCT(m_pMem0, CRVL3DPose, pNeigh->pPoseRel);
				//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 3 * 3 * 3, pNeigh->pPoseRel->m_C); // 6DOF CCsCm
				RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 3 * 3, pNeigh->pPoseRel->m_C);		// 3DOF CAsAm

				pNeigh->ModelId = pBestPSuLMClone->m_Index;
				pNeigh->pPSuLM = pBestPSuLMClone;
				//memcpy(pNeigh->pPoseRel->m_Rot, pBestPose->m_Rot, 3 * 3 * sizeof(double));	// 6DOF CCsCm
				//memcpy(pNeigh->pPoseRel->m_X, pBestPose->m_X, 3 * sizeof(double));			// 6DOF CCsCm
				//PanTiltRollUncertainty(pNeigh->pPoseRel->m_C,pBestPose->m_C,pPoseAA,true);	// 6DOF CCsCm
				//pNeigh->pPoseRel->m_Alpha = pBestPose->m_Alpha;								// 6DOF CCsCm
				//pNeigh->pPoseRel->m_Beta = pBestPose->m_Beta;									// 6DOF CCsCm
				//pNeigh->pPoseRel->m_Theta = pBestPose->m_Theta;								// 6DOF CCsCm
				memcpy(pNeigh->pPoseRel->m_Rot, pPoseAsAm->m_Rot, 3 * 3 * sizeof(double));	// 3DOF CAsAm
				memcpy(pNeigh->pPoseRel->m_X, pPoseAsAm->m_X, 3 * sizeof(double));			// 3DOF CAsAm
				CRVL3DPose *pPoseAsAm = pNeigh->pPoseRel;
				double *CAsAm = pPoseAsAm->m_C;
				PanTiltRollUncertainty(CCsCm,pPoseCsCm->m_C, pPoseCsCm ,true);
				DetermineUncertainty6DOFTo3DOF(CAsAm, CCsCm, pPoseAsAm);	// 3DOF CAsAm

				if(pMPSuLM2 == pMPSuLM)
					//Convert uncertainty back 
					memcpy(m_pPrevCameraPoseSM->m_C, CCsCm, 3 * 3 * 3 * sizeof(double));

				//pNeigh->pPoseRel->m_Alpha = bestRobotPose.m_Alpha;
				//pNeigh->pPoseRel->m_Beta = bestRobotPose.m_Beta;	
				//pNeigh->pPoseRel->m_Theta = bestRobotPose.m_Theta;

				RVLQLIST_ADD_ENTRY(pMPSuLM2->m_NeighbourList, pNeighbourEntry);	//$
				
				if(pBestPSuLMClone->m_NeighbourList==NULL)	//$
				{
					RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLIST, pBestPSuLMClone->m_NeighbourList);
					RVLQLIST_INIT(pBestPSuLMClone->m_NeighbourList);
				}
				RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLIST_PTR_ENTRY, pNeighbourEntry);
				RVLMEM_ALLOC_STRUCT(m_pMem0, RVLPSULM_NEIGHBOUR, pNeigh);
				pNeighbourEntry->Ptr = pNeigh;
				RVLMEM_ALLOC_STRUCT(m_pMem0, CRVL3DPose, pNeigh->pPoseRel);
				RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 3 * 3, pNeigh->pPoseRel->m_C);

				pNeigh->ModelId = pMPSuLM2->m_Index;	//$
				pNeigh->pPSuLM = pMPSuLM2;	//$
				InverseTransform3D(pNeigh->pPoseRel->m_Rot, pNeigh->pPoseRel->m_X, pPoseAsAm->m_Rot, pPoseAsAm->m_X);
				RVL3DOFInvTransfUncert(pPoseAsAm->m_Rot[0], pPoseAsAm->m_Rot[3], 
					pPoseAsAm->m_X[0], pPoseAsAm->m_X[1], CAsAm, pNeigh->pPoseRel->m_C);
				RVLQLIST_ADD_ENTRY(pBestPSuLMClone->m_NeighbourList, pNeighbourEntry);							
			}	// For all models in OFVModelList

			//Add current PSULM to database/map
			m_PSuLMList.Add(pBestPSuLMClone);
			m_pPrevModelPSuLM = pBestPSuLMClone;

			//Save current scene pose to previous scene pose
			RVLCOPY3VECTOR(pPoseS0->m_X, m_pPoseSp0->m_X)
			RVLCOPYMX3X3(pPoseS0->m_Rot,m_pPoseSp0->m_Rot)
			m_pPoseSp0->m_Alpha = pPoseS0->m_Alpha;
			m_pPoseSp0->m_Beta = pPoseS0->m_Beta;
			m_pPoseSp0->m_Theta = pPoseS0->m_Theta;

			bAdded = true;		
		}
		else	// if((m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING) == 0
			// ||
			// (((bestRobotPose.m_X[0]*bestRobotPose.m_X[0] + bestRobotPose.m_X[1]*bestRobotPose.m_X[1]) <= pow(m_MinHybridLocalizationDist - 100,2)) 
			// && 
			// abs(eAlpha*RAD2DEG)<=(m_MinHybridLocalizationAngle-1))
		{
			//copy absolute pose of nearest model	
			//memcpy(m_pModelAbsPose->m_X, pMPSuLM->m_PoseAbs->m_X, 3 * sizeof(double));
			//memcpy(m_pModelAbsPose->m_Rot, pMPSuLM->m_PoseAbs->m_Rot, 3 * 3 * sizeof(double));
			//m_pModelAbsPose->UpdatePTRLL();   

			m_pPrevModelPSuLM = pMPSuLM;

			//Save current scene to previous scene
			RVLCOPY3VECTOR(pPoseS0->m_X, m_pPoseSp0->m_X)
			RVLCOPYMX3X3(pPoseS0->m_Rot,m_pPoseSp0->m_Rot)
			m_pPoseSp0->m_Alpha = pPoseS0->m_Alpha;
			m_pPoseSp0->m_Beta = pPoseS0->m_Beta;
			m_pPoseSp0->m_Theta = pPoseS0->m_Theta;

			PanTiltRollUncertainty(CCsCm, pBestPose->m_C, pBestPose,true);

			//Convert uncertainty back 
			memcpy(m_pPrevCameraPoseSM->m_C, CCsCm, 3 * 3 * 3 * sizeof(double));


#ifdef RVLPSULMBUILDER_MAP_DEBUG_LOG
			double RAs0[3 * 3];
			double tAs0[3];

			RVLCombineTransform3D(pMPSuLM->m_PoseAbs->m_Rot,pMPSuLM->m_PoseAbs->m_X,
				bestRobotPose.m_Rot,bestRobotPose.m_X, RAs0, tAs0);

			fprintf(fpMapNodesLog, "%d\t%lf\t%lf\t%lf\n", -(pMPSuLM->m_iSubMap + 1), atan2(RAs0[3], RAs0[0]) * RAD2DEG, tAs0[0], tAs0[1]);
#endif

		}

	}
	
	//BestPose
	memcpy(m_pPrevCameraPoseSM->m_Rot, pBestPose->m_Rot, 3 * 3 * sizeof(double));
	memcpy(m_pPrevCameraPoseSM->m_X, pBestPose->m_X, 3 * sizeof(double));		
	
	m_pPrevCameraPoseSM->m_Alpha = pBestPose->m_Alpha;
	m_pPrevCameraPoseSM->m_Beta = pBestPose->m_Beta;
	m_pPrevCameraPoseSM->m_Theta = pBestPose->m_Theta;

	//Determine Robot1->Robot2 pose if Camera1->Camera2 pose is known
	if(bAdded)
	{
		//reset values
		m_pPrevRobotPoseSM->m_Alpha = m_pPrevRobotPoseSM->m_Beta = m_pPrevRobotPoseSM->m_Theta = 0.0;
		m_pPrevRobotPoseSM->UpdateRotA0();
		RVLNULL3VECTOR(m_pPrevRobotPoseSM->m_X)
		
	}
	else
	{
		memcpy(m_pPrevRobotPoseSM->m_X, bestRobotPose.m_X, 3 * sizeof(double));
		memcpy(m_pPrevRobotPoseSM->m_Rot, bestRobotPose.m_Rot, 3 * 3 * sizeof(double));
		m_pPrevRobotPoseSM->m_Alpha = bestRobotPose.m_Alpha;
		m_pPrevRobotPoseSM->m_Beta = bestRobotPose.m_Beta;
		m_pPrevRobotPoseSM->m_Theta = bestRobotPose.m_Theta;
		
	}

#ifdef RVLPSULMBUILDER_MAP_DEBUG_LOG
	FILE *fpMapConnectionsLog;

	fpMapConnectionsLog = fopen_s("C:\\RVL\\ExpRez\\MapConnectionsLog.dat", "w");

	MapLog(fpMapNodesLog, fpMapConnectionsLog);

	fclose(fpMapNodesLog);
	fclose(fpMapConnectionsLog);
#endif

	return bAdded;
}


//Find the fixed transformation -> robot wrt camera
void CRVLPSuLMBuilder::RobotCameraPose()
{
	m_pPoseCB->UpdateRotLL();
	

	//Initialize RAB
	double RAB[9];
	RVLNULLMX3X3(RAB);
	RAB[1] = -1.0;
	RAB[5] = -1.0;
	RAB[6] =  1.0;

	//Initialize tAB
	double tAB[3];
	RVLNULL3VECTOR(tAB);

	//Get TBC ie inverse transformation of TCB
	double RBC[9], tBC[3];
	InverseTransform3D(RBC,tBC,m_pPoseCB->m_Rot,m_pPoseCB->m_X);

	RVLCombineTransform3D(RBC,tBC,RAB,tAB,m_pPoseAC->m_Rot,m_pPoseAC->m_X);

}

////Determine Robot1->Robot2 pose if Camera1->Camera2 pose is known
//// TAAp = TCpAp*TCCp*TAC   (NOTE: TAC = TApCp)
//void CRVLPSuLMBuilder::DeterminePose6DOFTo3DOF(CRVL3DPose *pPoseAAp, CRVL3DPose *pPoseCCp)
//{
//	RVLDeterminePose6DOFTo3DOF(pPoseAAp,pPoseCCp,m_pPoseAC);
//	
//	////Get TCA ie inverse transformation of TAC
//	//double RCpAp[9], tCpAp[3];
//	//// TCpAp <- inv(TAC)
//	//InverseTransform3D(RCpAp,tCpAp,m_pPoseAC->m_Rot,m_pPoseAC->m_X);
//
//
//	//double RCAp[9], tCAp[3];
//
//	//// TCAp <- TCpAp * TCCp
//	//RVLCombineTransform3D(RCpAp,tCpAp,pPoseCCp->m_Rot,pPoseCCp->m_X,RCAp,tCAp);
//
//	//// TAAp <- TCAp * TAC
//	//RVLCombineTransform3D(RCAp,tCAp,m_pPoseAC->m_Rot,m_pPoseAC->m_X,pPoseAAp->m_Rot,pPoseAAp->m_X);
//
//	//pPoseAAp->m_X[2] = 0.0;
//
//	//double V[3];
//	//pPoseAAp->GetAngleAxis(V, pPoseAAp->m_Alpha);
//
//	//if(V[2] < 0.0)
//	//	pPoseAAp->m_Alpha =-pPoseAAp->m_Alpha;
//
//	//pPoseAAp->m_Beta = pPoseAAp->m_Theta = 0.0;
//	//pPoseAAp->UpdateRotA0();
//
//	////pPoseAAp->m_Beta = pPoseAAp->m_Theta = 0.0;
//	////pPoseAAp->UpdateRotA0();
//
//}

//Determine uncertainty of the pan-tilt-roll parameters Cw' = N~ * Cw * N~T
//if bInverse Cw = N * Cw' * NT
void CRVLPSuLMBuilder::PanTiltRollUncertainty(double *OutUncert, double *InUncert,CRVL3DPose *pPoseAAp, bool bInverse)
{

	int i;
	double P1[9], P2[9];
	double Mtemp[9];

	//Define PUncert
	for(i = 0; i < 3; i++)
	{
		P1[0 + i] = InUncert[0*9 + 0 + i];
		P1[3 + i] = InUncert[0*9 + 3 + i];
		P1[6 + i] = InUncert[0*9 + 6 + i];

		P2[0 + i] = InUncert[1*9 + 0 + i];
		P2[3 + i] = InUncert[1*9 + 3 + i];
		P2[6 + i] = InUncert[1*9 + 6 + i];

	}

	
	//determine N, inv(N) then PUncert2
	double N[9], NTrans[9], invN[9], invNTrans[9];
	double PN1[9], PN2[9];

	double cb = cos(pPoseAAp->m_Beta);
	double sb = sin(pPoseAAp->m_Beta);
	double cq = cos(pPoseAAp->m_Theta);
	double sq = sin(pPoseAAp->m_Theta);

	N[0] = cb*sq;
	N[3] = cb*cq;
	N[6] = -sb;

	N[1] = cq;
	N[4] = -sq;
	N[7] = 0;

	N[2] = 0;
	N[5] = 0;
	N[8] = 1;

	if(bInverse)
	{
		InverseRotation(NTrans, N);

		//PN1
		RVLMXMUL3X3(N,P1,Mtemp);
		RVLMXMUL3X3(Mtemp,NTrans,PN1);

		//PN2
		RVLMXMUL3X3(N,P2,PN2);
	}
	else
	{
		InverseMatrix3(invN,N); 
		InverseRotation(invNTrans, invN);

		//PN1
		RVLMXMUL3X3(invN,P1,Mtemp);
		RVLMXMUL3X3(Mtemp,invNTrans,PN1);

		//PN2
		RVLMXMUL3X3(invN,P2,PN2);
	}
	
	//Define PUncert2
	for(i = 0; i < 3; i++)
	{
		OutUncert[0*9 + 0 + i] = PN1[0 + i];
		OutUncert[0*9 + 3 + i] = PN1[3 + i];
		OutUncert[0*9 + 6 + i] = PN1[6 + i];

		OutUncert[1*9 + 0 + i] = PN2[0 + i];
		OutUncert[1*9 + 3 + i] = PN2[3 + i];
		OutUncert[1*9 + 6 + i] = PN2[6 + i];

		OutUncert[2*9 + 0 + i] = InUncert[2*9 + 0 + i];
		OutUncert[2*9 + 3 + i] = InUncert[2*9 + 3 + i];
		OutUncert[2*9 + 6 + i] = InUncert[2*9 + 6 + i];
		
	}

}

float CRVLPSuLMBuilder::GetColorSceneSimilarityScore(CRVLPSuLM *pMPSuLM, CRVLPSuLM *pSPSuLM)
{
	CRVL3DSurface2 *pM3DSurface;
	CRVL3DSurface2 *pS3DSurface;
	float score = 0.0;
	float max = 0.0;
	float current = 0.0;
	for(int i = 0; i<pSPSuLM->m_n3DSurfaces; i++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[i];
		max = 0.0;
		for (int j = 0; j < pMPSuLM->m_n3DSurfaces; j++)
		{
			pM3DSurface = pMPSuLM->m_3DSurfaceArray[j];
			current = pS3DSurface->RVLIntersectColorHistogram(pM3DSurface);
			if (current > max)
				max = current;
		}
		score += max;

	}

	return score * 100.0;
}
//Function addes color information of the map node
void CRVLPSuLMBuilder::AddNodeToColorMapDB(void** colorMapDB, CRVLPSuLM* mapnode, CRVLMem * pMem)
{
	CRVL3DSurface2 *p3DSurface;
	RVLQLIST_HIST_ENTRY *pHistEntry;
	RVLQLIST_PTR_ENTRY *pNewDBEntry;
	RVLColorMapDBNode *pColorMapNode;
	RVLQLIST *newBinList;
	for(int i = 0; i<mapnode->m_n3DSurfaces; i++)
	{
		p3DSurface = mapnode->m_3DSurfaceArray[i];
		if (p3DSurface->m_histRGB)
		{
			pHistEntry = (RVLQLIST_HIST_ENTRY*)p3DSurface->m_histRGB->pFirst;
			while(pHistEntry)
			{
				if (!colorMapDB[pHistEntry->adr])	//if that bin does not exist, create new list at its location
				{
					RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST, newBinList);
					RVLQLIST_INIT(newBinList);
					colorMapDB[pHistEntry->adr] = newBinList;					
				}
				//add new node to current bin list
				RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pNewDBEntry);
				RVLMEM_ALLOC_STRUCT(pMem, RVLColorMapDBNode, pColorMapNode);
				pColorMapNode->noBinPix = pHistEntry->value;
				pColorMapNode->noUsedPix = p3DSurface->m_noUsedColorPts;
				pColorMapNode->MapNodeIdx = mapnode->m_Index;
				pColorMapNode->ObjIdx = p3DSurface->m_Index;
				pNewDBEntry->Ptr = pColorMapNode;
				RVLQLIST_ADD_ENTRY(((RVLQLIST*)colorMapDB[pHistEntry->adr]), pNewDBEntry);
				
				//next bin
				pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
			}
		}
	}
}

void CRVLPSuLMBuilder::SaveColorMapDBXML(void** colorMapDB, char* filepath, int arraysize, char* colorsystem, int binres, int histDimension)
{
	xml_document<> doc;
	doc.name("ColorMapDB");
	RVLQLIST_PTR_ENTRY *pDBEntry;
	RVLColorMapDBNode *pColorMapNode;
	RVLQLIST* pQLIST;
	xml_node<> *node;
	char* temp;
	stringstream ss (stringstream::in | stringstream::out);
	//root node
	xml_node<> *rootnode =  doc.allocate_node(node_element, "ColorMapDB");
	//adding attribute information
	rootnode->append_attribute(doc.allocate_attribute("colorsystem", colorsystem));
	temp = doc.allocate_string(NULL, 1);
	ss << histDimension;
	ss >> temp;
	rootnode->append_attribute(doc.allocate_attribute("noDim", temp));
	temp = doc.allocate_string(NULL, 1);
	ss.clear();
	ss << binres;
	ss >> temp;
	rootnode->append_attribute(doc.allocate_attribute("binres", temp));
	doc.append_node(rootnode);
	
	xml_node<> *subnode;
	xml_node<> *subnode2;
	//for each bin
	for(int i = 0; i < arraysize; i++)
	{
		//if bin exists
		if (colorMapDB[i])
		{
			node = doc.allocate_node(node_element, "Bin");
		    rootnode->append_node(node);
			//Bin address
			temp = doc.allocate_string(NULL, 5);
			ss.clear();
			ss << i;
			ss >> temp;
			node->append_attribute(doc.allocate_attribute("id", temp));
			//for each object in this bin create new subnode
			pQLIST = (RVLQLIST*)colorMapDB[i];
			pDBEntry = (RVLQLIST_PTR_ENTRY*)pQLIST->pFirst;
			while(pDBEntry)
			{
				pColorMapNode = (RVLColorMapDBNode*)pDBEntry->Ptr;

				subnode = doc.allocate_node(node_element, "Object");
				node->append_node(subnode);
				//writing info about the object
				ss.clear();
				temp = doc.allocate_string(NULL, 10);
				ss << pColorMapNode->noBinPix;
				ss >> temp;
				subnode->append_node(doc.allocate_node(node_element, "noBinPix", temp));
				ss.clear();
				temp = doc.allocate_string(NULL, 10);
				ss << pColorMapNode->noUsedPix;
				ss >> temp;
				subnode->append_node(doc.allocate_node(node_element, "noUsedPix", temp));
				ss.clear();
				temp = doc.allocate_string(NULL, 10);
				ss << pColorMapNode->MapNodeIdx;
				ss >> temp;
				subnode->append_node(doc.allocate_node(node_element, "MapNodeIdx", temp));
				ss.clear();
				temp = doc.allocate_string(NULL, 10);
				ss << pColorMapNode->ObjIdx;
				ss >> temp;
				subnode->append_node(doc.allocate_node(node_element, "ObjIdx", temp));

				pDBEntry = (RVLQLIST_PTR_ENTRY*)pDBEntry->pNext;
			}

		}
	}

	//Saving to file
	ofstream dat(filepath);
	dat << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
	dat << doc;
	dat.close();
}

void** CRVLPSuLMBuilder::LoadColorMapDBXML(char* filepath, CRVLMem * pMem)
{
	void** colorMapDB;
	xml_document<> doc;
	RVLQLIST_PTR_ENTRY *pNewDBEntry;
	RVLColorMapDBNode *pColorMapNode;
	RVLQLIST* newBinList;
	xml_node<> *node;
	xml_node<> *rootnode;
	xml_node<> *subnode;
	xml_node<> *subnode2;
	char* temp;
	stringstream ss (stringstream::in | stringstream::out);
	int binres = 0;
	int noDim = 0;
	int binIdx = 0;
	xml_attribute<> *atrib;
	string tempS;

	//Loading and parsing XML
	ifstream dat(filepath);
	string str((istreambuf_iterator<char>(dat)), istreambuf_iterator<char>());
	char *text = new char [str.size()+1];
	strcpy (text, str.c_str());
	doc.parse<0>(text);
	//generating colorMapDB from XML DOM
	//root node of the map
	rootnode = doc.first_node();
	//getting information from attributes
	atrib = rootnode->first_attribute("noDim");
	ss << string(atrib->value(), atrib->value_size());
	ss >> noDim;
	atrib = rootnode->first_attribute("binres");
	ss.clear();
	ss << string(atrib->value(), atrib->value_size());
	ss >> binres;
	//creating colorMapDB of required size
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, void*, ((int)pow((double)binres, noDim)), colorMapDB);
	for(int i = 0; i < (int)pow((double)binres, noDim); i++)
	{
		if (colorMapDB[i])
			colorMapDB[i] = NULL;
	}
	node = rootnode->first_node();
	while(node)
	{
		//getting bin index and creating new qlist
		ss.clear();
		ss << string(node->first_attribute()->value(), node->first_attribute()->value_size());
		ss >> binIdx;		
		RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST, newBinList);
		RVLQLIST_INIT(newBinList);
		colorMapDB[binIdx] = newBinList;
		subnode = node->first_node();
		while(subnode)
		{
			//add new node to current bin list
			RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_PTR_ENTRY, pNewDBEntry);
			RVLMEM_ALLOC_STRUCT(pMem, RVLColorMapDBNode, pColorMapNode);
			subnode2 = subnode->first_node();
			while(subnode2)
			{
				tempS = string(subnode2->name(), subnode2->name_size());
				if(tempS == "noBinPix")
				{
					ss.clear();
					ss << string(subnode2->value(), subnode2->value_size());
					ss >> pColorMapNode->noBinPix;					
				}
				else if(tempS == "noUsedPix")
				{
					ss.clear();
					ss << string(subnode2->value(), subnode2->value_size());
					ss >> pColorMapNode->noUsedPix;
				}
				else if(tempS == "MapNodeIdx")
				{
					ss.clear();
					ss << string(subnode2->value(), subnode2->value_size());
					ss >> pColorMapNode->MapNodeIdx;
				}
				else if(tempS == "ObjIdx")
				{
					ss.clear();
					ss << string(subnode2->value(), subnode2->value_size());
					ss >> pColorMapNode->ObjIdx;
				}
				subnode2 = subnode2->next_sibling();
			}
			pNewDBEntry->Ptr = pColorMapNode;
			RVLQLIST_ADD_ENTRY(((RVLQLIST*)colorMapDB[binIdx]), pNewDBEntry);
			subnode = subnode->next_sibling();
		}
		node = node->next_sibling();
	}
	delete [] text;

	return colorMapDB;
}

// Computes m_PoseRTAs for all PSuLMs for which a path to pMPSuLM0 exists.
// m_PoseRTAs computed for each PSuLM represents its pose relative to pMPSuLM0.
// The function uses m_pMem2.

void CRVLPSuLMBuilder::GetModelPoses( CRVL3DPose *pPoseAsAm0,
									  CRVLPSuLM *pMPSuLM0,
									  CRVLPSuLM **ModelList,
									  int &nModels,
									  int nTgtModels)
{
	// TAm0As <- inv(TAsAm0)

	double *RAsAm0 = pPoseAsAm0->m_Rot;
	double *tAsAm0 = pPoseAsAm0->m_X;
	double *RAm0As = pMPSuLM0->m_PoseRTAs.m_Rot;
	double *tAm0As = pMPSuLM0->m_PoseRTAs.m_X;

	RVLINVTRANSF3D(RAsAm0, tAsAm0, RAm0As, tAm0As)

	double *CAm0As = pMPSuLM0->m_PoseRTAs.m_C;

	double *C2;

	if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
		RVL3DOFInvTransfUncert(RAsAm0[0], RAsAm0[3], tAm0As[0], tAm0As[1], pPoseAsAm0->m_C, CAm0As);
	else
	{
		RVL6DOFInvTransfUncert(RAsAm0, tAsAm0, pPoseAsAm0->m_C, CAm0As);

		RVLCOMPLETESIMMX3(CAm0As)
		C2 = CAm0As + 2 * 9;
		RVLCOMPLETESIMMX3(C2)
	}

	//// TAs0 <- TAm00 * TAsAm0

	//double *RAm00 = pMPSuLM0->m_PoseAbs->m_Rot;
	//double *tAm00 = pMPSuLM0->m_PoseAbs->m_X;

	//double RAs0[3 * 3];
	//double tAs0[3];

	//RVLCOMPTRANSF3D(RAm00, tAm00, RAsAm0, tAsAm0, RAs0, tAs0);

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
	FILE *fpNodes;
	FILE *fpConnections;

	if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
	{
		fpNodes = fopen("C:\\RVL\\ExpRez\\MapNodesLog.dat", "w");
		fpConnections = fopen("C:\\RVL\\ExpRez\\MapConnectionsLog.dat", "w");
	}
#endif

	// create PSuLM list PSuLMList

	RVLQLIST PSuLMList;	

	RVLPSULM_LOCAL_MODEL_LIST_ENTRY *PSuLMListMem;

	BYTE *pFreeMem = m_pMem2->m_pFreeMem;

	BYTE *pStartBlock = m_pMem2->m_pStartBlock;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_LOCAL_MODEL_LIST_ENTRY, m_PSuLMList.m_nElements, PSuLMListMem);

	if(m_pMem2->m_pStartBlock != pStartBlock)
		pFreeMem = m_pMem2->m_pStartBlock + sizeof(BYTE *);

	RVLPSULM_LOCAL_MODEL_LIST_ENTRY *pPSuLMListEntry = PSuLMListMem;

	RVLQLIST *pPSuLMList = &PSuLMList;

	RVLQLIST_INIT(pPSuLMList);

	// put pMPSuLM0 to PSuLMBuff

	pPSuLMListEntry->pPSuLM = pMPSuLM0;

	double C[9], eig[3];

	if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
	{
		C[0] = CAm0As[4];
		C[1] = CAm0As[5];
		C[2] = CAm0As[7];
		C[3] = CAm0As[8];

		RVLEig2(C, eig);

		pPSuLMListEntry->uncert = RVLMAX(eig[0], eig[1]);
	}
	else
	{
		BOOL bReal[3];

		RVLEig3(CAm0As + 2 * 9, eig, bReal);

		double fTmp = RVLMAX(eig[0], eig[1]);

		pPSuLMListEntry->uncert = RVLMAX(fTmp, eig[2]);
	}

	RVLQLIST_ADD_ENTRY(pPSuLMList, pPSuLMListEntry);

	pPSuLMListEntry++;

	/////

	RVLPSULM_LOCAL_MODEL_LIST_ENTRY *pPSuLMListEntry0 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(PSuLMList.pFirst);

	int nVisitedTgtModels = 0;

	CRVLPSuLM **ppModel = ModelList;

	CRVLPSuLM *pPSuLM, *pPSuLM2;
	RVLQLIST *pNeighbourList;
	RVLQLIST_PTR_ENTRY *pNeighborPtr;
	RVLPSULM_NEIGHBOUR *pNeighbor;
	CRVL3DPose PoseAmcAs;
	double uncert;
	RVLPSULM_LOCAL_MODEL_LIST_ENTRY *pPSuLMListEntry1, *pPSuLMListEntry2;
	double *R, *t, *R2, *t2;
	double *CAmAs;
	double *RAmcAmp, *tAmcAmp;

	// repeat until the end of PSuLMList is reached

	while(pPSuLMListEntry0)
	{
		// expand pPSuLMListEntry0

		pPSuLM = pPSuLMListEntry0->pPSuLM;

		if(pPSuLM->m_Flags & RVLPSULM_FLAG_VISITED)
		{
			// get next entry from PSuLMList

			pPSuLMListEntry0 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(pPSuLMListEntry0->pNext);

			continue;
		}

		pPSuLM->m_Flags |= RVLPSULM_FLAG_VISITED;

		if(ModelList)
			*(ppModel++) = pPSuLM;

		if(pPSuLM->m_Flags & RVLPSULM_FLAG_TARGET)
		{
			pPSuLM->m_Flags &= ~RVLPSULM_FLAG_TARGET;

			nVisitedTgtModels++;

			if(nVisitedTgtModels == nTgtModels)
				break;
		}

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
		if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
		{
			fprintf(fpNodes, "0\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", 
				pPSuLM->m_Index,
				atan2(pPSuLM->m_PoseRTAs.m_Rot[3], pPSuLM->m_PoseRTAs.m_Rot[0]), 
				pPSuLM->m_PoseRTAs.m_X[0], pPSuLM->m_PoseRTAs.m_X[1],
				pPSuLM->m_PoseRTAs.m_C[0], pPSuLM->m_PoseRTAs.m_C[1], pPSuLM->m_PoseRTAs.m_C[2],
				pPSuLM->m_PoseRTAs.m_C[4], pPSuLM->m_PoseRTAs.m_C[5], pPSuLM->m_PoseRTAs.m_C[8]);
		}
#endif

		pNeighbourList = pPSuLM->m_NeighbourList;

		if(pNeighbourList == NULL)
		{
			// get next entry from PSuLMList

			pPSuLMListEntry0 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(pPSuLMListEntry0->pNext);

			continue;
		}

		pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighbourList->pFirst);

		while(pNeighborPtr)
		{
			pNeighbor = (RVLPSULM_NEIGHBOUR *)(pNeighborPtr->Ptr);

			pPSuLM2 = pNeighbor->pPSuLM;

			// check if pPSuLM2 is already visited

			if(pPSuLM2->m_Flags & RVLPSULM_FLAG_VISITED)
			{
				pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);

				continue;
			}

			//if(pPSuLM->m_Index == 155 && pPSuLM2->m_Index == 163)
			//if(pPSuLM->m_Index == 155)
			//	int debug = 0;

			// TAmcAs <- TAmpAs * TAmcAmp

			R = pPSuLM->m_PoseRTAs.m_Rot;
			t = pPSuLM->m_PoseRTAs.m_X;
			R2 = pPSuLM2->m_PoseRTAs.m_Rot;
			t2 = pPSuLM2->m_PoseRTAs.m_X;
			RAmcAmp = pNeighbor->pPoseRel->m_Rot;
			tAmcAmp = pNeighbor->pPoseRel->m_X;

			if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
			{
				RVLCOMPTRANSF3D3DOF(R, t, RAmcAmp, tAmcAmp, R2, t2)		

				// pPSuLM2->m_PoseRTAs.m_C <- EKF Prediction from pNeighbor->pPoseRel and pPSuLM->m_PoseRTAs

				//DetermineOdometryUncertainty(pNeighbor->pPoseRel->m_C, pNeighbor->pPoseRel);	// remove after debugging !!!

				UncertaintyEKFPrediction3DOF(pPSuLM2->m_PoseRTAs.m_C, pNeighbor->pPoseRel->m_C, pPSuLM->m_PoseRTAs.m_C, 
					pNeighbor->pPoseRel, &(pPSuLM->m_PoseRTAs));
			}
			else
			{
				RVLCOMPTRANSF3D(R, t, RAmcAmp, tAmcAmp, R2, t2)	

				UncertaintyEKFPrediction6DOF(pPSuLM2->m_PoseRTAs.m_C, pNeighbor->pPoseRel->m_C, pPSuLM->m_PoseRTAs.m_C,
					pNeighbor->pPoseRel, &(pPSuLM->m_PoseRTAs));
			}

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
			if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
			{
				fprintf(fpConnections, "0\t%lf\t%lf\t%lf\t%lf\n", t[0], t[1], t2[0], t2[1]);
				fflush(fpConnections);
			}
#endif

			// put pPSuLM2 into PSuLMList before the first entry with a higher or equal uncertainty

			pPSuLMListEntry->pPSuLM = pPSuLM2;

			CAmAs = pPSuLM2->m_PoseRTAs.m_C;

			if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
			{
				C[0] = CAmAs[4];
				C[1] = CAmAs[5];
				C[2] = CAmAs[7];
				C[3] = CAmAs[8];

				RVLEig2(C, eig);

				uncert = RVLMAX(eig[0], eig[1]);
			}
			else
			{
				BOOL bReal[3];

				RVLEig3(CAmAs + 2 * 9, eig, bReal);

				double fTmp = RVLMAX(eig[0], eig[1]);

				uncert = RVLMAX(fTmp, eig[2]);
			}

			if(uncert > 1000.0 * 1000.0)
			{
				pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);

				continue;
			}

			pPSuLMListEntry->uncert = uncert;

			pPSuLMListEntry1 = pPSuLMListEntry0;

			pPSuLMListEntry2 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(pPSuLMListEntry1->pNext);

			while(pPSuLMListEntry2)
			{
				if(pPSuLMListEntry2->uncert >= uncert)
					break;

				pPSuLMListEntry1 = pPSuLMListEntry2;

				pPSuLMListEntry2 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(pPSuLMListEntry1->pNext);
			}

			RVLQLIST_INSERT_ENTRY(pPSuLMList, pPSuLMListEntry1, pPSuLMListEntry2, pPSuLMListEntry);

			pPSuLMListEntry++;

			pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);
		}	// for every neighbor of pPSuLM

		// get next entry from PSuLMList

		pPSuLMListEntry0 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(pPSuLMListEntry0->pNext);
	}	// repeat until the end of PSuLMList is reached

	nModels = ppModel - ModelList;

	// reset RVLPSULM_FLAG_VISITED flag of all PSuLMs

	RVLResetFlags<CRVLPSuLM>(&m_PSuLMList, RVLPSULM_FLAG_VISITED);

	// deallocate m_pMem2

	m_pMem2->m_pFreeMem = pFreeMem;

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
	if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
	{
		fclose(fpNodes);
		fclose(fpConnections);
	}
#endif
}

void CRVLPSuLMBuilder::GetLocalModels(CRVL3DPose *pPoseAsAsp, 
									  CRVL3DPose *pPoseAspAmp,
									  CRVLPSuLM *pMPSuLM0,
									  double DistCoeff,
									  BYTE Flags)
{
#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
	FILE *fpCandidateModels;

	if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
		fpCandidateModels = fopen("C:\\RVL\\ExpRez\\CandidateModels.dat", "w");
#endif

	CRVL3DPose PoseAsAmp;

	double CAsAmp[3 * 3 * 3];

	PoseAsAmp.m_C = CAsAmp;

	if(pPoseAsAsp)
	{
		// TAsAmp <- TAspAmp*TAsAsp

		double *RAsAsp = pPoseAsAsp->m_Rot;
		double *tAsAsp = pPoseAsAsp->m_X;
		double *RAspAmp = pPoseAspAmp->m_Rot;
		double *tAspAmp = pPoseAspAmp->m_X;
		double *RAsAmp = PoseAsAmp.m_Rot;
		double *tAsAmp = PoseAsAmp.m_X;

		RVLCOMPTRANSF3D(RAspAmp, tAspAmp, RAsAsp, tAsAsp, RAsAmp, tAsAmp)

		if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
			UncertaintyEKFPrediction3DOF(CAsAmp, pPoseAsAsp->m_C, pPoseAspAmp->m_C, pPoseAsAsp, pPoseAspAmp);
		else
			UncertaintyEKFPrediction6DOF(CAsAmp, pPoseAsAsp->m_C, pPoseAspAmp->m_C, pPoseAsAsp, pPoseAspAmp);
	}
	else
		PoseAsAmp.Copy(pPoseAspAmp);

	CRVLPSuLM **ModelList = new CRVLPSuLM *[m_PSuLMList.m_nElements];

	int nModels;

	GetModelPoses(&PoseAsAmp, pMPSuLM0, ModelList, nModels);

	/////

	double r = DistCoeff * m_MinHybridLocalizationDist;
	double r2 = r * r;
	double phi = 3.0 * m_MinHybridLocalizationAngle * DEG2RAD;
	
	CRVL3DPose PoseAsAm;
	double CAsAm[3 * 3 * 3];

	PoseAsAm.m_C = CAsAm;

	CRVLPSuLM *pPSuLM;
	CRVL3DPose PoseAmcAs;
	double AlphaAmcAs, eAlpha;
	CRVL3DPose *pPoseAmAs;
	double detC, k;
	double eX[3];
	double dist2;
	//double *R, *t, *R2, *t2;
	double C[9];
	double *CAmAs;

	CRVLPSuLM **pModelListEnd = ModelList + nModels;

	CRVLPSuLM **ppModel;

	for(ppModel = ModelList; ppModel < pModelListEnd; ppModel++)
	{
		pPSuLM = *ppModel;

		if(pPSuLM->m_Flags & RVLPSULM_FLAG_CLOSE)
			continue;

		// if pPSuLM is sufficiently close to the current robot pose, then add it to m_PSuLMSubList

		pPoseAmAs = &(pPSuLM->m_PoseRTAs);

		if(Flags & RVLPSULMBUILDER_GETLOCALMODELS_FLAG_ORIENT_CONSTR)
		{
			AlphaAmcAs = atan2(pPoseAmAs->m_Rot[3], pPoseAmAs->m_Rot[0]);

			if(AlphaAmcAs > phi)
			{
				if(Flags & RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT)
				{
					eAlpha = AlphaAmcAs - phi;

					if(eAlpha * eAlpha / pPoseAmAs->m_C[0] > 2.71)	// 2.71 = chi2inv(0.90, 1)
						continue;
				}
				else
					continue;
			}
			else if(AlphaAmcAs < -phi)
			{
				if(Flags & RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT)
				{
					eAlpha = AlphaAmcAs + phi;

					if(eAlpha * eAlpha / pPoseAmAs->m_C[0] > 2.71)	// 2.71 = chi2inv(0.90, 1)
						continue;
				}
				else
					continue;
			}
		}

		dist2 = pPoseAmAs->m_X[0] * pPoseAmAs->m_X[0] + pPoseAmAs->m_X[1] * pPoseAmAs->m_X[1] + pPoseAmAs->m_X[2] * pPoseAmAs->m_X[2];

		if(dist2 > r2)
		{
			if(Flags & RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT)
			{
				k = 1.0 - r / sqrt(dist2);

				eX[0] = k * pPoseAmAs->m_X[0];
				eX[1] = k * pPoseAmAs->m_X[1];

				CAmAs = pPSuLM->m_PoseRTAs.m_C;

				C[0] = CAmAs[4];
				C[1] = CAmAs[5];
				C[2] = CAmAs[7];
				C[3] = CAmAs[8];

				detC = RVLDET2(C);

				if(RVLMAHDIST2(eX, C, detC) > 4.6)	// 4.6 = chi2inv(0.90, 2)
					continue;
			}
			else
				continue;
		}

		pPSuLM->m_Flags |= RVLPSULM_FLAG_CLOSE;

		m_PSuLMSubList.Add(pPSuLM);

		//// PoseAsAm <- inverse of pPSuLM->m_PoseRTAs

		//R2 = pPoseAmAs->m_Rot;
		//t2 = pPoseAmAs->m_X;
		//R = PoseAsAm.m_Rot;
		//t = PoseAsAm.m_X;

		//R[0] = R2[0];
		//R[1] = R2[3];
		//R[2] = 0.0;
		//R[3] = R2[1];
		//R[4] = R2[4];
		//R[5] = 0.0;
		//R[6] = 0.0;
		//R[7] = 0.0;
		//R[8] = 1.0;
		//t[0] = - R2[0]*t2[0] - R2[3]*t2[1];
		//t[1] = R2[3]*t2[0] - R2[0]*t2[1];
		//t[2] = 0.0;

		//RVL3DOFInvTransfUncert(R2[0], R2[3], t[0], t[1], pPoseAmAs->m_C, CAsAm);

		//// compute pPSuLM->m_PoseCs from PoseAsAm

		//DetermineUncertainty3DOFTo6DOF(pPSuLM->m_CCsInit, &(pPSuLM->m_PoseCsInit), &PoseAsAm, CAsAm);

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
		if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
			fprintf(fpCandidateModels, "%d\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", pPSuLM->m_iSubMap, pPSuLM->m_Index, 
				atan2(pPSuLM->m_PoseRTAs.m_Rot[3], pPSuLM->m_PoseRTAs.m_Rot[0]), 
				pPSuLM->m_PoseRTAs.m_X[0], pPSuLM->m_PoseRTAs.m_X[1],
				pPSuLM->m_PoseRTAs.m_C[0], pPSuLM->m_PoseRTAs.m_C[1], pPSuLM->m_PoseRTAs.m_C[2],
				pPSuLM->m_PoseRTAs.m_C[4], pPSuLM->m_PoseRTAs.m_C[5], pPSuLM->m_PoseRTAs.m_C[8]);
#endif
	}	// for all PSuLMs in ModelList

	delete[] ModelList;

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
	if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
		fclose(fpCandidateModels);
#endif
}

void CRVLPSuLMBuilder::GetLocalModels2(	CRVL3DPose *pPoseAsAsp, 
										CRVL3DPose *pPoseAspAmp,
										CRVLPSuLM *pMPSuLM0,
										double DistCoeff,
										BYTE Flags)
{
#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
	FILE *fpCandidateModels;

	if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
		fpCandidateModels = fopen("C:\\RVL\\ExpRez\\CandidateModels2.dat", "w");
#endif

	CRVL3DPose PoseAsAmp;

	double *RAsAmp = PoseAsAmp.m_Rot;
	double *tAsAmp = PoseAsAmp.m_X;

	double CAsAmp[3 * 3];

	PoseAsAmp.m_C = CAsAmp;

	if(pPoseAsAsp)
	{
		// TAsAmp <- TAspAmp * TAsAsp

		double *RAsAsp = pPoseAsAsp->m_Rot;
		double *tAsAsp = pPoseAsAsp->m_X;
		double *RAspAmp = pPoseAspAmp->m_Rot;
		double *tAspAmp = pPoseAspAmp->m_X;

		RVLCOMPTRANSF3D(RAspAmp, tAspAmp, RAsAsp, tAsAsp, RAsAmp, tAsAmp)

		UncertaintyEKFPrediction3DOF(CAsAmp, pPoseAsAsp->m_C, pPoseAspAmp->m_C, pPoseAsAsp, pPoseAspAmp);
	}
	else
		PoseAsAmp.Copy(pPoseAspAmp);	// TAsAmp <- TAspAmp

	// TAmpAs <- inv(TAsAmp)

	CRVL3DPose PoseAmpAs;

	double *RAmpAs = PoseAmpAs.m_Rot;
	double *tAmpAs = PoseAmpAs.m_X;

	double CAmpAs[3 * 3];

	PoseAmpAs.m_C = CAmpAs;

	RVLINVTRANSF3D(RAsAmp, tAsAmp, RAmpAs, tAmpAs)

	RVL3DOFInvTransfUncert(RAsAmp[0], RAsAmp[3], tAmpAs[0], tAmpAs[1], CAsAmp, CAmpAs);	

	/////

	double r = DistCoeff * m_MinHybridLocalizationDist;
	double r2 = r * r;
	double phi = 3.0 * m_MinHybridLocalizationAngle * DEG2RAD;
	
	CRVL3DPose PoseAsAm;

	double *RAsAm = PoseAsAm.m_Rot;
	double *tAsAm = PoseAsAm.m_X;

	double CAsAm[3 * 3];

	PoseAsAm.m_C = CAsAm;

	CRVL3DPose PoseAmAs;

	double *RAmAs = PoseAmAs.m_Rot;
	double *tAmAs = PoseAmAs.m_X;

	double CAmAs[3 * 3];

	PoseAmAs.m_C = CAmAs;

	RVLPSULM_NEIGHBOR2 *pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pMPSuLM0->m_LocalMap.pFirst);

	CRVLPSuLM *pPSuLM;
	double AlphaAmcAs, eAlpha;
	double detC, k;
	double eX[2];
	double dist2;
	double C[2 * 2];
	CRVL3DPose *pPoseAmAmp;
	double *RAmAmp, *tAmAmp, *CAmAmp;

	while(pNeighbor)
	{
		pPSuLM = pNeighbor->pMPSuLM;

		if(pPSuLM->m_Flags & RVLPSULM_FLAG_CLOSE)
		{
			pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);

			continue;
		}

		// TAmAmp <- pNeighbor->PoseRel

		pPoseAmAmp = &(pNeighbor->PoseRel);

		RAmAmp = pPoseAmAmp->m_Rot;
		tAmAmp = pPoseAmAmp->m_X;	
		CAmAmp = pPoseAmAmp->m_C;

		// TAmAs <- TAmpAs * TAmAmp

		RVLCOMPTRANSF3D3DOF(RAmpAs, tAmpAs, RAmAmp, tAmAmp, RAmAs, tAmAs)

		UncertaintyEKFPrediction3DOF(CAmAs, CAmAmp, CAmpAs, pPoseAmAmp, &PoseAmpAs);

		// if pPSuLM is sufficiently close to the current robot pose, then add it to m_PSuLMSubList

		if(Flags & RVLPSULMBUILDER_GETLOCALMODELS_FLAG_ORIENT_CONSTR)
		{
			AlphaAmcAs = atan2(RAmAs[3], RAmAs[0]);

			if(AlphaAmcAs > phi)
			{
				if(Flags & RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT)
				{
					eAlpha = AlphaAmcAs - phi;

					if(eAlpha * eAlpha / CAmAs[0] > 2.71)	// 2.71 = chi2inv(0.90, 1)
					{
						pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);

						continue;
					}
				}
				else
				{
					pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);

					continue;
				}
			}
			else if(AlphaAmcAs < -phi)
			{
				if(Flags & RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT)
				{
					eAlpha = AlphaAmcAs + phi;

					if(eAlpha * eAlpha / CAmAs[0] > 6.6349)	// 6.6349 = chi2inv(0.99, 1)
					{
						pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);

						continue;
					}
				}
				else
				{
					pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);

					continue;
				}			
			}
		}

		dist2 = tAmAs[0] * tAmAs[0] + tAmAs[1] * tAmAs[1];

		if(dist2 > r2)
		{
			if(Flags & RVLPSULMBUILDER_GETLOCALMODELS_FLAG_UNCERT)
			{
				k = 1.0 - r / sqrt(dist2);

				eX[0] = k * tAmAs[0];
				eX[1] = k * tAmAs[1];

				C[0] = CAmAs[4];
				C[1] = CAmAs[5];
				C[2] = CAmAs[7];
				C[3] = CAmAs[8];

				detC = RVLDET2(C);

				if(RVLMAHDIST2(eX, C, detC) > 9.21)	// 9.21 = chi2inv(0.99, 2)
				{
					pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);

					continue;
				}
			}
			else
			{
				pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);

				continue;
			}
		}

		pPSuLM->m_Flags |= RVLPSULM_FLAG_CLOSE;

		m_PSuLMSubList.Add(pPSuLM);

		// TAsAm <- inv(TAmAs)

		RVLINVTRANSF3D3DOF(RAmAs, tAmAs, RAsAm, tAsAm)

		RVL3DOFInvTransfUncert(RAmAs[0], RAmAs[3], tAsAm[0], tAsAm[1], CAmAs, CAsAm);

		// compute pPSuLM->m_PoseCs from PoseAsAm

		DetermineUncertainty3DOFTo6DOF(pPSuLM->m_CCsInit, &(pPSuLM->m_PoseCsInit), &PoseAsAm, CAsAm);

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
		if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
			fprintf(fpCandidateModels, "%d\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", pPSuLM->m_iSubMap, pPSuLM->m_Index, 
				atan2(PoseAmAs.m_Rot[3], PoseAmAs.m_Rot[0]), 
				PoseAmAs.m_X[0], PoseAmAs.m_X[1],
				PoseAmAs.m_C[0], PoseAmAs.m_C[1], PoseAmAs.m_C[2],
				PoseAmAs.m_C[4], PoseAmAs.m_C[5], PoseAmAs.m_C[8]);
#endif

		pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
	}	// for all PSuLMs in local map of pMPSuLM0

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
	if(m_DebugFlags & RVLPSULMBUILDER_DEBUG_FLAG_GET_LOCAL_MODELS_LOG)
		fclose(fpCandidateModels);
#endif
}


#ifdef NEVER
void CRVLPSuLMBuilder::GetLocalModels(CRVL3DPose *pPoseAsAsp, 
									  CRVL3DPose *pPoseAspAmp,
									  CRVLPSuLM *pMPSuLM0)
{
#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
	FILE *fpNodes = fopen("C:\\RVL\\ExpRez\\MapNodesLog.dat", "w");
	FILE *fpConnections = fopen("C:\\RVL\\ExpRez\\MapConnectionsLog.dat", "w");
#endif

	// pMPSuLM0->m_PoseRTAs ~ TAmpAs
	// TAmpAs <- inv(TAspAmp*TAsAsp)

	double *RAsAsp = pPoseAsAsp->m_Rot;
	double *tAsAsp = pPoseAsAsp->m_X;
	double *RAspAmp = pPoseAspAmp->m_Rot;
	double *tAspAmp = pPoseAspAmp->m_X;
	double RAsAmp[3 * 3];
	double tAsAmp[3];

	RVLCOMPTRANSF3D(RAspAmp, tAspAmp, RAsAsp, tAsAsp, RAsAmp, tAsAmp)

	double *RAmpAs = pMPSuLM0->m_PoseRTAs.m_Rot;
	double *tAmpAs = pMPSuLM0->m_PoseRTAs.m_X;

	RVLINVTRANSF3D(RAsAmp, tAsAmp, RAmpAs, tAmpAs)

	double CAsAmp[3 * 3];

	UncertaintyEKFPrediction3DOF(CAsAmp, pPoseAsAsp->m_C, pPoseAspAmp->m_C, pPoseAsAsp, pPoseAspAmp);

	double *CAmpAs = pMPSuLM0->m_PoseRTAs.m_C;

	RVL3DOFInvTransfUncert(RAsAmp[0], RAsAmp[3], tAmpAs[0], tAmpAs[1], CAsAmp, CAmpAs);	

	// TAs0 <- TAmp0 * TAsAmp

	double RAs0[3 * 3];
	double tAs0[3];
	double *RAmp0 = pMPSuLM0->m_PoseAbs->m_Rot;
	double *tAmp0 = pMPSuLM0->m_PoseAbs->m_X;

	RVLCOMPTRANSF3D(RAmp0, tAmp0, RAsAmp, tAsAmp, RAs0, tAs0);
		
	// create PSuLM list PSuLMList

	RVLQLIST PSuLMList;	

	RVLPSULM_LOCAL_MODEL_LIST_ENTRY *PSuLMListMem;

	BYTE *pFreeMem = m_pMem2->m_pFreeMem;

	BYTE *pStartBlock = m_pMem2->m_pStartBlock;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_LOCAL_MODEL_LIST_ENTRY, m_PSuLMList.m_nElements, PSuLMListMem);

	if(m_pMem2->m_pStartBlock != pStartBlock)
		pFreeMem = m_pMem2->m_pStartBlock + sizeof(BYTE *);

	RVLPSULM_LOCAL_MODEL_LIST_ENTRY *pPSuLMListEntry = PSuLMListMem;

	RVLQLIST *pPSuLMList = &PSuLMList;

	RVLQLIST_INIT(pPSuLMList);

	// put pMPSuLM0 to PSuLMBuff

	pPSuLMListEntry->pPSuLM = pMPSuLM0;

	double C[4], eig[2];

	C[0] = CAmpAs[4];
	C[1] = CAmpAs[5];
	C[2] = CAmpAs[7];
	C[3] = CAmpAs[8];

	RVLEig2(C, eig);

	pPSuLMListEntry->uncert = RVLMAX(eig[0], eig[1]);

	RVLQLIST_ADD_ENTRY(pPSuLMList, pPSuLMListEntry);

	pPSuLMListEntry++;

	// clear m_PSuLMSubList

	m_PSuLMSubList.RemoveAll();

	/////

	double r = 2.0 * m_MinHybridLocalizationDist;
	double r2 = r * r;
	double phi = 2.0 * m_MinHybridLocalizationAngle * DEG2RAD;
	
	RVLPSULM_LOCAL_MODEL_LIST_ENTRY *pPSuLMListEntry0 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(PSuLMList.pFirst);

	CRVL3DPose PoseAsAm;
	double CAsAm[3 * 3];

	PoseAsAm.m_C = CAsAm;

	CRVLPSuLM *pPSuLM, *pPSuLM2;
	RVLQLIST *pNeighbourList;
	RVLQLIST_PTR_ENTRY *pNeighborPtr;
	RVLPSULM_NEIGHBOUR *pNeighbor;
	CRVL3DPose PoseAmcAs;
	double uncert;
	RVLPSULM_LOCAL_MODEL_LIST_ENTRY *pPSuLMListEntry1, *pPSuLMListEntry2;
	double AlphaAmcAs, eAlpha;
	CRVL3DPose *pPoseAmAs;
	double detC, k;
	double eX[2];
	double dist2;
	double *R, *t, *R2, *t2;

	// repeat until the end of PSuLMList is reached

	while(pPSuLMListEntry0)
	{
		// expand pPSuLMListEntry0

		pPSuLM = pPSuLMListEntry0->pPSuLM;

		pPSuLM->m_Flags |= RVLPSULM_FLAG_VISITED;

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
		fprintf(fpNodes, "0\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t", atan2(pPSuLM->m_PoseRTAs.m_Rot[3], pPSuLM->m_PoseRTAs.m_Rot[0]), 
			pPSuLM->m_PoseRTAs.m_X[0], pPSuLM->m_PoseRTAs.m_X[1],
			pPSuLM->m_PoseRTAs.m_C[0], pPSuLM->m_PoseRTAs.m_C[1], pPSuLM->m_PoseRTAs.m_C[2],
			pPSuLM->m_PoseRTAs.m_C[4], pPSuLM->m_PoseRTAs.m_C[5], pPSuLM->m_PoseRTAs.m_C[8]);
#endif

		pNeighbourList = pPSuLM->m_NeighbourList;

		pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighbourList->pFirst);

		while(pNeighborPtr)
		{
			pNeighbor = (RVLPSULM_NEIGHBOUR *)(pNeighborPtr->Ptr);

			pPSuLM2 = pNeighbor->pPSuLM;

			// ToDo: Create neighbor relation from the new PSuLM to the previous PSuLM in SaveCurrentData

			// check if pPSuLM2 is already visited

			if(pPSuLM2->m_Flags & RVLPSULM_FLAG_VISITED)
			{
				pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);

				continue;
			}

			// TAmcAs <- inv(TAs0) * TAmc0

			R2 = pPSuLM2->m_PoseAbs->m_Rot;
			t2 = pPSuLM2->m_PoseAbs->m_X;
			R = pPSuLM2->m_PoseRTAs.m_Rot;
			t = pPSuLM2->m_PoseRTAs.m_X;

			R[0] = R2[0]*RAs0[0] + R2[3]*RAs0[3];
			R[1] = R2[0]*RAs0[3] - R2[3]*RAs0[0];
			R[2] = 0.0;
			R[3] = R2[3]*RAs0[0] - R2[0]*RAs0[3];
			R[4] = R2[0]*RAs0[0] + R2[3]*RAs0[3];
			R[5] = 0.0;
			R[6] = 0.0;
			R[7] = 0.0;
			R[8] = 1.0;
			t[0] = RAs0[0]*t2[0] + RAs0[3]*t2[1] - RAs0[0]*tAs0[0] - RAs0[3]*tAs0[1];
			t[1] = RAs0[0]*t2[1] - RAs0[3]*t2[0] - RAs0[0]*tAs0[1] + RAs0[3]*tAs0[0];
			t[2] = 0.0;

			// pPSuLM2->m_PoseRTAs.m_C <- EKF Prediction from pNeighbor->pPoseRel and pPSuLM->m_PoseRTAs

			DetermineOdometryUncertainty(pNeighbor->pPoseRel->m_C, pNeighbor->pPoseRel);	// remove after debugging !!!

			UncertaintyEKFPrediction3DOF(pPSuLM2->m_PoseRTAs.m_C, pNeighbor->pPoseRel->m_C, pPSuLM->m_PoseRTAs.m_C, 
				pNeighbor->pPoseRel, &(pPSuLM->m_PoseRTAs));

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
			fprintf(fpConnections, "0\t%lf\t%lf\t%lf\t%lf\n", t[0], t[1], pPSuLM->m_PoseRTAs.m_X[0], pPSuLM->m_PoseRTAs.m_X[1]);
			fflush(fpConnections);
#endif

			// put pPSuLM2 into PSuLMList before the first entry with a higher or equal uncertainty

			pPSuLMListEntry->pPSuLM = pPSuLM2;

			CAmpAs = pPSuLM2->m_PoseRTAs.m_C;

			C[0] = CAmpAs[4];
			C[1] = CAmpAs[5];
			C[2] = CAmpAs[7];
			C[3] = CAmpAs[8];

			RVLEig2(C, eig);

			uncert = RVLMAX(eig[0], eig[1]);

			pPSuLMListEntry->uncert = uncert;

			pPSuLMListEntry1 = pPSuLMListEntry0;

			pPSuLMListEntry2 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(pPSuLMListEntry1->pNext);

			while(pPSuLMListEntry2)
			{
				if(pPSuLMListEntry2->uncert >= uncert)
					break;

				pPSuLMListEntry1 = pPSuLMListEntry2;

				pPSuLMListEntry2 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(pPSuLMListEntry1->pNext);
			}

			RVLQLIST_INSERT_ENTRY(pPSuLMList, pPSuLMListEntry1, pPSuLMListEntry2, pPSuLMListEntry);

			pPSuLMListEntry++;

			pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);
		}	// for every neighbor of pPSuLM

		// get next entry from PSuLMList

		pPSuLMListEntry0 = (RVLPSULM_LOCAL_MODEL_LIST_ENTRY *)(pPSuLMListEntry0->pNext);

		// if pPSuLM is sufficiently close to the current robot pose, then add it to m_PSuLMSubList

		pPoseAmAs = &(pPSuLM->m_PoseRTAs);

		AlphaAmcAs = atan2(pPoseAmAs->m_Rot[3], pPoseAmAs->m_Rot[0]);

		if(AlphaAmcAs > phi)
		{
			eAlpha = AlphaAmcAs - phi;

			if(eAlpha * eAlpha / pPoseAmAs->m_C[0] > 2.71)	// 2.71 = chi2inv(0.90, 1)
			{
#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
				fprintf(fpNodes, "0\n");
				fflush(fpNodes);
#endif
				continue;
			}
		}
		else if(AlphaAmcAs < -phi)
		{
			eAlpha = AlphaAmcAs + phi;

			if(eAlpha * eAlpha / pPoseAmAs->m_C[0] > 2.71)	// 2.71 = chi2inv(0.90, 1)
			{
#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
				fprintf(fpNodes, "0\n");
				fflush(fpNodes);
#endif
				continue;
			}
		}

		dist2 = pPoseAmAs->m_X[0] * pPoseAmAs->m_X[0] + pPoseAmAs->m_X[1] * pPoseAmAs->m_X[1];

		if(dist2 > r2)
		{
			k = 1.0 - r / sqrt(dist2);

			eX[0] = k * pPoseAmAs->m_X[0];
			eX[1] = k * pPoseAmAs->m_X[1];

			CAmpAs = pPSuLM->m_PoseRTAs.m_C;

			C[0] = CAmpAs[4];
			C[1] = CAmpAs[5];
			C[2] = CAmpAs[7];
			C[3] = CAmpAs[8];

			detC = RVLDET2(C);

			if(RVLMAHDIST2(eX, C, detC) > 4.6)	// 4.6 = chi2inv(0.90, 2)
			{
#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
				fprintf(fpNodes, "0\n");
				fflush(fpNodes);
#endif
				continue;
			}
		}

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
		fprintf(fpNodes, "1\n");
		fflush(fpNodes);
#endif

		m_PSuLMSubList.Add(pPSuLM);

		// PoseAsAm <- inverse of pPSuLM->m_PoseRTAs

		pPoseAmAs = &(pPSuLM->m_PoseRTAs);

		R2 = pPoseAmAs->m_Rot;
		t2 = pPoseAmAs->m_X;
		R = PoseAsAm.m_Rot;
		t = PoseAsAm.m_X;

		R[0] = R2[0];
		R[1] = R2[3];
		R[2] = 0.0;
		R[3] = R2[1];
		R[4] = R2[4];
		R[5] = 0.0;
		R[6] = 0.0;
		R[7] = 0.0;
		R[8] = 1.0;
		t[0] = - R2[0]*t2[0] - R2[3]*t2[1];
		t[1] = R2[3]*t2[0] - R2[0]*t2[1];
		t[2] = 0.0;

		RVL3DOFInvTransfUncert(R2[0], R2[3], t[0], t[1], pPoseAmAs->m_C, CAsAm);

		// compute pPSuLM->m_PoseCs from PoseAsAm

		DetermineUncertainty3DOFTo6DOF(pPSuLM->m_CCsInit, &(pPSuLM->m_PoseCsInit), &PoseAsAm, CAsAm);
	}	// repeat until the end of PSuLMList is reached

	// reset RVLPSULM_FLAG_VISITED flag of all PSuLMs

	m_PSuLMList.Start();

	while(m_PSuLMList.m_pNext)
	{
		pPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());

		pPSuLM->m_Flags &= ~RVLPSULM_FLAG_VISITED;
	}

	// deallocate m_pMem2

	m_pMem2->m_pFreeMem = pFreeMem;

#ifdef RVLPSULMBUILDER_GET_LOCAL_MODELS_DEBUG_LOG
	fclose(fpNodes);
	fclose(fpConnections);
#endif
}
#endif


void CRVLPSuLMBuilder::MapLog(FILE *fpMapNodesLog, FILE *fpMapConnectionsLog)
{
	RVLQLIST *pNeighbourList;
	RVLQLIST_PTR_ENTRY *pNeighborPtr;
	RVLPSULM_NEIGHBOUR *pNeighbor;
	CRVLPSuLM *pPSuLM;
	double *R1, *t1, *R2, *t2;

	m_PSuLMList.Start();

	while(m_PSuLMList.m_pNext)
	{
		pPSuLM = (CRVLPSuLM*)(m_PSuLMList.GetNext());

		R1 = pPSuLM->m_PoseAbs->m_Rot;
		t1 = pPSuLM->m_PoseAbs->m_X;

		fprintf(fpMapNodesLog, "%d\t%lf\t%lf\t%lf\n", pPSuLM->m_iSubMap, atan2(R1[3], R1[0]) * RAD2DEG, t1[0], t1[1]);

		pNeighbourList = pPSuLM->m_NeighbourList;

		if(pNeighbourList)
		{
			pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighbourList->pFirst);

			while(pNeighborPtr)
			{
				pNeighbor = (RVLPSULM_NEIGHBOUR *)(pNeighborPtr->Ptr);

				R2 = pNeighbor->pPSuLM->m_PoseAbs->m_Rot;
				t2 = pNeighbor->pPSuLM->m_PoseAbs->m_X;

				fprintf(fpMapConnectionsLog, "%d\t%lf\t%lf\t%lf\t%lf\n", pPSuLM->m_iSubMap, t1[0], t1[1], t2[0], t2[1]);

				pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);
			}
		}
	}
}


void CRVLPSuLMBuilder::GetGroundTruth(CRVLPSuLM *pPSulM)
{
	char *charGTFileName;

	int iFileNameLength = (int)strlen(pPSulM->m_FileName);

	//use temp mem
	BYTE *pFreeMem = m_pMem2->m_pFreeMem;

	//Create ground truth file name
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, char, iFileNameLength + 1, charGTFileName);
	
	strcpy(charGTFileName, pPSulM->m_FileName);
	
	char *pPos;
	pPos = strstr (charGTFileName,"LW.bmp");
	strncpy(pPos,"GT.key",7);


	CRVL3DPose * pAbsPose = pPSulM->m_PoseAbs;
	
	FILE *fp;

	fp = fopen(charGTFileName, "r");

	if(fp)
	{
		fscanf(fp, "%lf\t%lf\t%lf\t", pAbsPose->m_X, pAbsPose->m_X + 1, &(pAbsPose->m_Alpha));

		pAbsPose->m_Alpha *= DEG2RAD;

		fclose(fp);
	}
	else
		pAbsPose->m_X[0] = pAbsPose->m_X[1] = pAbsPose->m_Alpha = 0.0;

	
	pAbsPose->m_X[2] = 0.0;
	pAbsPose->m_Beta = pAbsPose->m_Theta = 0.0;

	pAbsPose->UpdateRotA0();


	//free mem
	m_pMem2->m_pFreeMem = pFreeMem;
}



void CRVLPSuLMBuilder::GetGroundTruth(CRVLPSuLM *pPSulM, double *pGTData)
{
	char *charGTFileName;

	int iFileNameLength = (int)strlen(pPSulM->m_FileName);

	//use temp mem
	BYTE *pFreeMem = m_pMem2->m_pFreeMem;

	//Create ground truth file name
	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, char, iFileNameLength + 1, charGTFileName);
	
	strcpy(charGTFileName, pPSulM->m_FileName);
	
	char *pPos;
	pPos = strstr (charGTFileName,"LW.bmp");
	strncpy(pPos,"GT.key",7);


	FILE *fp;

	fp = fopen(charGTFileName, "r");

	if(fp)
	{
		fscanf(fp, "%lf\t%lf\t%lf\t", pGTData, pGTData + 1, pGTData + 2);

		fclose(fp);
	}
	else
		pGTData[0] = pGTData[1] = pGTData[2] = 0.0;

	
}


int CRVLPSuLMBuilder::GetnModels(void)
{
	return m_maxPSuLMIndex + 1;
}

int CRVLPSuLMBuilder::GetMostProbableDistance(	int iPose0, 
												int iNeighbor, 
												double *dX,
												double &dAlpha)
{
	CRVLPSuLM *pPSuLM = m_PSuLMArray[iPose0];

	RVLPSULM_PATH_PLANNING_NEIGHBOR *pPathPlanningNeighbor = 
		pPSuLM->m_PathPlanningNeighbourArray + iNeighbor;

	dX[0] = pPathPlanningNeighbor->PoseRel.m_X[0];
	dX[1] = pPathPlanningNeighbor->PoseRel.m_X[1];

	dAlpha = pPathPlanningNeighbor->dAlpha;

	return DOUBLE2INT(pPathPlanningNeighbor->s);	
}


int CRVLPSuLMBuilder::GetPathPlanningNodeIndex(int iPose0, int iNeighbor)
{
	CRVLPSuLM *pPSuLM = m_PSuLMArray[iPose0];

	RVLPSULM_PATH_PLANNING_NEIGHBOR *pPathPlanningNeighbor = 
		pPSuLM->m_PathPlanningNeighbourArray + iNeighbor;

	return pPathPlanningNeighbor->pMPSuLM->m_Index;
}

int CRVLPSuLMBuilder::GetnReachableModels(int iPose0)
{
	CRVLPSuLM *pPSuLM = m_PSuLMArray[iPose0];

	return (pPSuLM ? pPSuLM->m_nPathPlanningNeighbours : 0);
}

void CRVLPSuLMBuilder::PreparePathPlanning()
{	
	FILE *fpConstraints;

	CRVL3DPose NullPose;
	double C[3 * 3];

	RVLNULLMX3X3(C)

	NullPose.Reset();

	NullPose.m_C = C;

	NullPose.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_3D;

	CRVLPSuLM **pPSuLMArrayEnd = m_PSuLMArray + m_maxPSuLMIndex + 1;

	CRVLPSuLM **ConstraintArray = new CRVLPSuLM *[m_maxPSuLMIndex + 1];

	CRVLPSuLM **ppPSuLM;
	CRVLPSuLM *pPSuLM, *pPSuLM2;
	RVLPSULM_PATH_PLANNING_NEIGHBOR *pNeighbor;
	double *dX, *R, *R2;
	double alpha1, alpha2, dalpha;
	int iConstraint, nConstraints;
	int iPSuLM, iPSuLM2;
	BOOL bConstrained;

	for(ppPSuLM = m_PSuLMArray; ppPSuLM < pPSuLMArrayEnd; ppPSuLM++)
	{
		pPSuLM = *ppPSuLM;

		if(pPSuLM == NULL)
			continue;

		nConstraints = 0;

		bConstrained = FALSE;

		fpConstraints = fopen("C:\\RVL\\PathPlanningConstraints.dat", "r");

		if(fpConstraints)
		{
			while(!feof(fpConstraints))
			{
				fscanf(fpConstraints, "%d\t%d\n", &iPSuLM, &iPSuLM2);

				if(iPSuLM == pPSuLM->m_Index)
				{
					bConstrained = TRUE;

					ConstraintArray[nConstraints++] = m_PSuLMArray[iPSuLM2];
				}
			}

			fclose(fpConstraints);
		}

		//if(pPSuLM->m_Index == 90)
		//	int debug = 0;

		m_PSuLMSubList.RemoveAll();

		GetLocalModels(NULL, &NullPose, pPSuLM, 2.0);

		RVLResetFlags<CRVLPSuLM>(&m_PSuLMSubList, RVLPSULM_FLAG_CLOSE);

		pPSuLM->m_nPathPlanningNeighbours = (bConstrained ? nConstraints : m_PSuLMSubList.m_nElements);

		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, RVLPSULM_PATH_PLANNING_NEIGHBOR, pPSuLM->m_nPathPlanningNeighbours, 
			pPSuLM->m_PathPlanningNeighbourArray)

		pNeighbor = pPSuLM->m_PathPlanningNeighbourArray;

		m_PSuLMSubList.Start();

		while(m_PSuLMSubList.m_pNext)
		{
			pPSuLM2 = (CRVLPSuLM *)(m_PSuLMSubList.GetNext());

			if(bConstrained)
			{
				for(iConstraint = 0; iConstraint < nConstraints; iConstraint++)
					if(ConstraintArray[iConstraint] == pPSuLM2)
						break;

				if(iConstraint == nConstraints)
					continue;
			}

			pNeighbor->pMPSuLM = pPSuLM2;

			dX = pPSuLM2->m_PoseRTAs.m_X;
			R = pPSuLM2->m_PoseRTAs.m_Rot;

			pNeighbor->PoseRel.m_X[0] = dX[0];
			pNeighbor->PoseRel.m_X[1] = dX[1];
			pNeighbor->PoseRel.m_X[2] = 0.0;
			R2 = pNeighbor->PoseRel.m_Rot;
			RVLCOPYMX3X3(R, R2)
			pNeighbor->s = sqrt(dX[0] * dX[0] + dX[1] * dX[1]);

			alpha1 = atan2(-dX[1], -dX[0]);

			alpha2 = atan2(R[3], R[0]);

			dalpha = alpha2 - alpha1;

			if(dalpha > PI)
				dalpha -= 2.0 * PI;
			else if(dalpha <= -PI)
				dalpha += 2.0 * PI;

			pNeighbor->dAlpha = fabs(alpha1) + fabs(dalpha);

			pNeighbor++;
		}
	}

	delete[] ConstraintArray;
}

BOOL CRVLPSuLMBuilder::LoadPathPlanningMap(char *PathPlanningMapFileName)
{
	FILE *fp = fopen(PathPlanningMapFileName, "rb");

	if(fp == NULL)
		return FALSE;

	int nNodes;

	fread(&nNodes, sizeof(int), 1, fp);

	int iNode;
	WORD iPSuLM, iPSuLM2;
	CRVLPSuLM *pPSuLM;
	RVLPSULM_PATH_PLANNING_NEIGHBOR *pNeighbor;
	RVLPSULM_PATH_PLANNING_NEIGHBOR *pNeighborArrayEnd;

	for(iNode = 0; iNode < nNodes; iNode++)
	{
		fread(&iPSuLM, sizeof(WORD), 1, fp);

		pPSuLM = m_PSuLMArray[iPSuLM];

		if(pPSuLM == NULL)
			continue;

		fread(&(pPSuLM->m_nPathPlanningNeighbours), sizeof(int), 1, fp);

		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, RVLPSULM_PATH_PLANNING_NEIGHBOR, pPSuLM->m_nPathPlanningNeighbours, 
			pPSuLM->m_PathPlanningNeighbourArray)

		pNeighborArrayEnd = pPSuLM->m_PathPlanningNeighbourArray + pPSuLM->m_nPathPlanningNeighbours;
		
		for(pNeighbor = pPSuLM->m_PathPlanningNeighbourArray; pNeighbor < pNeighborArrayEnd; pNeighbor++)
		{
			fread(&iPSuLM2, sizeof(WORD), 1, fp);

			pNeighbor->pMPSuLM = m_PSuLMArray[iPSuLM2];

			fread(pNeighbor->PoseRel.m_Rot, sizeof(double), 3 * 3, fp);
			fread(pNeighbor->PoseRel.m_X, sizeof(double), 3, fp);
			fread(&(pNeighbor->s), sizeof(double), 1, fp);
			fread(&(pNeighbor->dAlpha), sizeof(double), 1, fp);
		}
	}

	fclose(fp);

	return TRUE;
}

BOOL CRVLPSuLMBuilder::SavePathPlanningMap(char *PathPlanningMapFileName)
{
	FILE *fp = fopen(PathPlanningMapFileName, "wb");

	if(fp == NULL)
		return FALSE;

	int nNodes = 0;

	CRVLPSuLM **pPSuLMArrayEnd = m_PSuLMArray + m_maxPSuLMIndex + 1;

	CRVLPSuLM **ppPSuLM;
	CRVLPSuLM *pPSuLM;

	for(ppPSuLM = m_PSuLMArray; ppPSuLM < pPSuLMArrayEnd; ppPSuLM++)
	{
		pPSuLM = *ppPSuLM;

		if(pPSuLM)
			nNodes++;
	}

	fwrite(&nNodes, sizeof(int), 1, fp);

	RVLPSULM_PATH_PLANNING_NEIGHBOR *pNeighbor;
	RVLPSULM_PATH_PLANNING_NEIGHBOR *pNeighborArrayEnd;

	for(ppPSuLM = m_PSuLMArray; ppPSuLM < pPSuLMArrayEnd; ppPSuLM++)
	{
		pPSuLM = *ppPSuLM;

		if(pPSuLM == NULL)
			continue;

		fwrite(&(pPSuLM->m_Index), sizeof(WORD), 1, fp);

		fwrite(&(pPSuLM->m_nPathPlanningNeighbours), sizeof(int), 1, fp);

		pNeighborArrayEnd = pPSuLM->m_PathPlanningNeighbourArray + pPSuLM->m_nPathPlanningNeighbours;
		
		for(pNeighbor = pPSuLM->m_PathPlanningNeighbourArray; pNeighbor < pNeighborArrayEnd; pNeighbor++)
		{
			fwrite(&(pNeighbor->pMPSuLM->m_Index), sizeof(WORD), 1, fp);
			fwrite(pNeighbor->PoseRel.m_Rot, sizeof(double), 3 * 3, fp);
			fwrite(pNeighbor->PoseRel.m_X, sizeof(double), 3, fp);
			fwrite(&(pNeighbor->s), sizeof(double), 1, fp);
			fwrite(&(pNeighbor->dAlpha), sizeof(double), 1, fp);
		}
	}

	fclose(fp);

	return TRUE;
}

void CRVLPSuLMBuilder::CreateLocalMaps()
{
	CRVLPSuLM **pPSuLMArrayEnd = m_PSuLMArray + m_maxPSuLMIndex + 1;

	CRVLPSuLM **ppPSuLM;
	CRVLPSuLM *pPSuLM;

	for(ppPSuLM = m_PSuLMArray; ppPSuLM < pPSuLMArrayEnd; ppPSuLM++)
	{
		pPSuLM = *ppPSuLM;

		if(pPSuLM == NULL)
			continue;

		CreateLocalMap(pPSuLM);
	}
}

BOOL CRVLPSuLMBuilder::LoadLocalMaps(char *LocalMapsFileName)
{
	FILE *fp = fopen(LocalMapsFileName, "rb");

	if(fp == NULL)
		return FALSE;

	int nNodes;

	fread(&nNodes, sizeof(int), 1, fp);

	RVLQLIST *pListArray = m_LocalMapHT.m_ListArray;

	int iNode;
	WORD iPSuLM, iPSuLM2;
	CRVLPSuLM *pPSuLM;
	RVLPSULM_NEIGHBOR2 *pNeighbor;
	RVLQLISTHT_PTR_ENTRY *pLocalMapHTEntry;
	DWORD dwTmp;
	RVLQLIST *pLocalMap;
	RVLPSULM_NEIGHBOR2 *pNeighborArrayEnd;
	unsigned short wTmp;

	for(iNode = 0; iNode < nNodes; iNode++)
	{
		fread(&wTmp, sizeof(ushort), 1, fp);

		iPSuLM = (WORD)wTmp;

		pPSuLM = m_PSuLMArray[iPSuLM];

		if(pPSuLM == NULL)
			continue;

		pLocalMap = &(pPSuLM->m_LocalMap);

		fread(&(pPSuLM->m_nLocalMapNodes), sizeof(int), 1, fp);

		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, RVLPSULM_NEIGHBOR2, pPSuLM->m_nLocalMapNodes, pNeighbor)

		pNeighborArrayEnd = pNeighbor + pPSuLM->m_nLocalMapNodes;

		for(; pNeighbor < pNeighborArrayEnd; pNeighbor++)
		{
			fread(&wTmp, sizeof(ushort), 1, fp);

			iPSuLM2 = (WORD)wTmp;

			pNeighbor->pMPSuLM = m_PSuLMArray[iPSuLM2];

			fread(pNeighbor->PoseRel.m_Rot, sizeof(double), 3 * 3, fp);
			fread(pNeighbor->PoseRel.m_X, sizeof(double), 3, fp);
			pNeighbor->PoseRel.m_C = pNeighbor->C;
			fread(pNeighbor->PoseRel.m_C, sizeof(double), 3 * 3, fp);

			RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLISTHT_PTR_ENTRY, pLocalMapHTEntry)

			pLocalMapHTEntry->adr = (DWORD)(RVLPSULMBUILDER_MAXN_PSULMS * iPSuLM + iPSuLM2);
			pLocalMapHTEntry->Ptr = pNeighbor;

			RVLQLISTHT_ADD_ENTRY(pListArray, pLocalMapHTEntry, m_LocalMapHT.m_Size, dwTmp)			

			RVLQLIST_ADD_ENTRY(pLocalMap, pNeighbor)
		}
	}

	fclose(fp);

	return TRUE;
}


BOOL CRVLPSuLMBuilder::SaveLocalMaps(char *LocalMapsFileName)
{
	FILE *fp = fopen(LocalMapsFileName, "wb");

	if(fp == NULL)
		return FALSE;

	int nNodes = 0;

	CRVLPSuLM **pPSuLMArrayEnd = m_PSuLMArray + m_maxPSuLMIndex + 1;

	CRVLPSuLM **ppPSuLM;
	CRVLPSuLM *pPSuLM;

	for(ppPSuLM = m_PSuLMArray; ppPSuLM < pPSuLMArrayEnd; ppPSuLM++)
	{
		pPSuLM = *ppPSuLM;

		if(pPSuLM)
			nNodes++;
	}

	fwrite(&nNodes, sizeof(int), 1, fp);

	RVLPSULM_NEIGHBOR2 *pNeighbor;

	for(ppPSuLM = m_PSuLMArray; ppPSuLM < pPSuLMArrayEnd; ppPSuLM++)
	{
		pPSuLM = *ppPSuLM;

		if(pPSuLM == NULL)
			continue;

		fwrite(&(pPSuLM->m_Index), sizeof(WORD), 1, fp);

		fwrite(&(pPSuLM->m_nLocalMapNodes), sizeof(int), 1, fp);

		pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pPSuLM->m_LocalMap.pFirst);
	
		while(pNeighbor)
		{
			fwrite(&(pNeighbor->pMPSuLM->m_Index), sizeof(WORD), 1, fp);
			fwrite(pNeighbor->PoseRel.m_Rot, sizeof(double), 3 * 3, fp);
			fwrite(pNeighbor->PoseRel.m_X, sizeof(double), 3, fp);
			fwrite(pNeighbor->PoseRel.m_C, sizeof(double), 3 * 3, fp);

			pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
		}
	}

	fclose(fp);

	return TRUE;
}

void CRVLPSuLMBuilder::Connect(CRVLPSuLM * pMPSuLM1, 
							   CRVLPSuLM * pMPSuLM2,
							   CRVL3DPose *pPoseAsAm,
							   CRVL3DPose *pPoseCsCm)
{
	if(pMPSuLM1->m_NeighbourList==NULL)
	{
		RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLIST, pMPSuLM1->m_NeighbourList);
		RVLQLIST_INIT(pMPSuLM1->m_NeighbourList)
	}

	RVLQLIST_PTR_ENTRY* pNeighbourEntry;

	RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLIST_PTR_ENTRY, pNeighbourEntry);

	RVLPSULM_NEIGHBOUR* pNeigh;

	RVLMEM_ALLOC_STRUCT(m_pMem0, RVLPSULM_NEIGHBOUR, pNeigh);
	pNeighbourEntry->Ptr = pNeigh;
	RVLMEM_ALLOC_STRUCT(m_pMem0, CRVL3DPose, pNeigh->pPoseRel);
	CRVL3DPose *pPoseAsAm_ = pNeigh->pPoseRel;

	pNeigh->ModelId = pMPSuLM2->m_Index;
	pNeigh->pPSuLM = pMPSuLM2;
	double *CAsAm_, *CCsCm_;
	if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
	{
		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 3 * 3, pNeigh->pPoseRel->m_C);	// 3DOF CAsAm
		memcpy(pNeigh->pPoseRel->m_Rot, pPoseAsAm->m_Rot, 3 * 3 * sizeof(double));	// 3DOF CAsAm
		memcpy(pNeigh->pPoseRel->m_X, pPoseAsAm->m_X, 3 * sizeof(double));			// 3DOF CAsAm		

		CAsAm_ = pPoseAsAm_->m_C;

		if(pPoseCsCm)
		{
			double CCsCm[3 * 3 * 3];
			PanTiltRollUncertainty(CCsCm,pPoseCsCm->m_C, pPoseCsCm ,true);
			DetermineUncertainty6DOFTo3DOF(CAsAm_, CCsCm, pPoseAsAm_);	// 3DOF CAsAm
		}
		else
		{
			double *CAsAm = pPoseAsAm->m_C;

			RVLCOPYMX3X3(CAsAm, CAsAm_)
		}
	}
	else
	{		
		memcpy(pNeigh->pPoseRel->m_Rot, pPoseCsCm->m_Rot, 3 * 3 * sizeof(double));	// 6DOF CCsCm
		memcpy(pNeigh->pPoseRel->m_X, pPoseCsCm->m_X, 3 * sizeof(double));			// 6DOF CCsCm
		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 3 * 3 * 3, pNeigh->pPoseRel->m_C); // 6DOF CCsCm
		CCsCm_ = pNeigh->pPoseRel->m_C;
		PanTiltRollUncertainty(CCsCm_,pPoseCsCm->m_C,pPoseCsCm,true);	// 6DOF CCsCm
		pNeigh->pPoseRel->m_Alpha = pPoseCsCm->m_Alpha;								// 6DOF CCsCm
		pNeigh->pPoseRel->m_Beta = pPoseCsCm->m_Beta;								// 6DOF CCsCm
		pNeigh->pPoseRel->m_Theta = pPoseCsCm->m_Theta;								// 6DOF CCsCm
	}


	//pNeigh->pPoseRel->m_Alpha = bestRobotPose.m_Alpha;
	//pNeigh->pPoseRel->m_Beta = bestRobotPose.m_Beta;	
	//pNeigh->pPoseRel->m_Theta = bestRobotPose.m_Theta;

	RVLQLIST_ADD_ENTRY(pMPSuLM1->m_NeighbourList, pNeighbourEntry);
	
	if(pMPSuLM2->m_NeighbourList==NULL)
	{
		RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLIST, pMPSuLM2->m_NeighbourList);
		RVLQLIST_INIT(pMPSuLM2->m_NeighbourList);
	}
	RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLIST_PTR_ENTRY, pNeighbourEntry);
	RVLMEM_ALLOC_STRUCT(m_pMem0, RVLPSULM_NEIGHBOUR, pNeigh);
	pNeighbourEntry->Ptr = pNeigh;
	RVLMEM_ALLOC_STRUCT(m_pMem0, CRVL3DPose, pNeigh->pPoseRel);	

	pNeigh->ModelId = pMPSuLM1->m_Index;
	pNeigh->pPSuLM = pMPSuLM1;	

	if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
	{
		InverseTransform3D(pNeigh->pPoseRel->m_Rot, pNeigh->pPoseRel->m_X, pPoseAsAm_->m_Rot, pPoseAsAm_->m_X);
		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 3 * 3, pNeigh->pPoseRel->m_C);
		RVL3DOFInvTransfUncert(pPoseAsAm_->m_Rot[0], pPoseAsAm_->m_Rot[3], 
			pNeigh->pPoseRel->m_X[0], pNeigh->pPoseRel->m_X[1], CAsAm_, pNeigh->pPoseRel->m_C);
	}
	else
	{
		RVLINVTRANSF3D(pPoseCsCm->m_Rot, pPoseCsCm->m_X, pNeigh->pPoseRel->m_Rot, pNeigh->pPoseRel->m_X)
		RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, double, 3 * 3 * 3, pNeigh->pPoseRel->m_C);
		RVL6DOFInvTransfUncert(pPoseCsCm->m_Rot, pPoseCsCm->m_X, CCsCm_, pNeigh->pPoseRel->m_C);
		double *C = pNeigh->pPoseRel->m_C;
		RVLCOMPLETESIMMX3(C)
		C += 2 * 9;
		RVLCOMPLETESIMMX3(C)
	}

	RVLQLIST_ADD_ENTRY(pMPSuLM2->m_NeighbourList, pNeighbourEntry);							
}

void CRVLPSuLMBuilder::DeleteConnection_(	CRVLPSuLM *pPSuLM1,
											CRVLPSuLM *pPSuLM2)
{
	RVLQLIST *pNeighborList = pPSuLM1->m_NeighbourList;

	RVLQLIST_PTR_ENTRY *pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborList->pFirst);

	void **pvpNeighborListEntry = &(pNeighborList->pFirst);

	RVLPSULM_NEIGHBOUR *pNeighborRel;

	while(pNeighborPtr)
	{
		pNeighborRel = (RVLPSULM_NEIGHBOUR *)(pNeighborPtr->Ptr);

		if(pNeighborRel->pPSuLM == pPSuLM2)
		{
			RVLQLIST_REMOVE_ENTRY(pNeighborList, pNeighborPtr, pvpNeighborListEntry)

			return;
		}

		pvpNeighborListEntry = &(pNeighborPtr->pNext);

		pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);	
	}
}

void CRVLPSuLMBuilder::DeleteConnection(CRVLPSuLM *pPSuLM1,
										CRVLPSuLM *pPSuLM2)
{
	DeleteConnection_(pPSuLM1, pPSuLM2);
	DeleteConnection_(pPSuLM2, pPSuLM1);
}

void CRVLPSuLMBuilder::CreateLocalMap(CRVLPSuLM * pPSuLM,
									  BYTE Flags)
{
	CRVL3DPose NullPose;
	double C[3 * 3 * 3];
	double *C_, *C2_;

	RVLNULLMX3X3(C)

	if(!(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF))
	{
		C_ = C + 9;
		RVLNULLMX3X3(C_)
		C_ += 9;
		RVLNULLMX3X3(C_)
	}

	NullPose.Reset();

	NullPose.m_C = C;

	NullPose.m_ParamFlags = (m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF ? RVL3DPOSE_PARAM_FLAGS_COV_3D : RVL3DPOSE_PARAM_FLAGS_COV_6D);
 
	RVLQLIST *pListArray = m_LocalMapHT.m_ListArray;

	CRVLPSuLM *pPSuLM2;
	RVLPSULM_NEIGHBOR2 *pNeighbor;
	double *dX, *R, *dX2, *R2;
	RVLQLISTHT_PTR_ENTRY *pLocalMapHTEntry;
	DWORD dwTmp;
	double *CAmAs, *CAmAs2;
	
	m_PSuLMSubList.RemoveAll();

	GetLocalModels(NULL, &NullPose, pPSuLM, m_LocalMapRadius / m_MinHybridLocalizationDist);

	RVLResetFlags<CRVLPSuLM>(&m_PSuLMSubList, RVLPSULM_FLAG_CLOSE);

	pPSuLM->m_nLocalMapNodes = m_PSuLMSubList.m_nElements;

	RVLQLIST *pLocalMap = &(pPSuLM->m_LocalMap);

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem0, RVLPSULM_NEIGHBOR2, pPSuLM->m_nLocalMapNodes, pNeighbor)

	RVLPSULM_NEIGHBOR2 *pNeighbor2;
	RVLQLIST *pLocalMap2;
	double *CTmp;

	m_PSuLMSubList.Start();

	while(m_PSuLMSubList.m_pNext)
	{
		pPSuLM2 = (CRVLPSuLM *)(m_PSuLMSubList.GetNext());

		pNeighbor->pMPSuLM = pPSuLM2;

		dX = pPSuLM2->m_PoseRTAs.m_X;
		R = pPSuLM2->m_PoseRTAs.m_Rot;
		CAmAs = pPSuLM2->m_PoseRTAs.m_C;

		pNeighbor->PoseRel.m_X[0] = dX[0];
		pNeighbor->PoseRel.m_X[1] = dX[1];
		pNeighbor->PoseRel.m_X[2] = dX[2];
		R2 = pNeighbor->PoseRel.m_Rot;
		RVLCOPYMX3X3(R, R2)
		CAmAs2 = pNeighbor->C;
		pNeighbor->PoseRel.m_C = CAmAs2;
		RVLCOPYMX3X3(CAmAs, CAmAs2)
		if(!(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF))
		{
			C_ = CAmAs + 9;
			C2_ = CAmAs2 + 9;
			RVLCOPYMX3X3(C_, C2_)
			C_ += 9;
			C2_ += 9;
			RVLCOPYMX3X3(C_, C2_)
		}

		RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLISTHT_PTR_ENTRY, pLocalMapHTEntry)

		pLocalMapHTEntry->adr = (DWORD)(RVLPSULMBUILDER_MAXN_PSULMS * pPSuLM->m_Index + pPSuLM2->m_Index);
		pLocalMapHTEntry->Ptr = pNeighbor;

		RVLQLISTHT_ADD_ENTRY(pListArray, pLocalMapHTEntry, m_LocalMapHT.m_Size, dwTmp)	

		RVLQLIST_ADD_ENTRY(pLocalMap, pNeighbor)

		pNeighbor++;

		if(Flags & RVLPSULMBUILDER_CREATELOCALMAP_FLAG_BIDIRECTIONAL)
		{
			RVLMEM_ALLOC_STRUCT(m_pMem0, RVLPSULM_NEIGHBOR2, pNeighbor2)

			pNeighbor2->pMPSuLM = pPSuLM;

			R2 = pNeighbor2->PoseRel.m_Rot;
			dX2 = pNeighbor2->PoseRel.m_X;
			CAmAs2 = pNeighbor2->C;
			pNeighbor2->PoseRel.m_C = CAmAs2;

			if(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF)
			{
				RVLINVTRANSF3D3DOF(R, dX, R2, dX2)
				
				RVL3DOFInvTransfUncert(R[0], R[3], dX2[0], dX2[1], CAmAs, CAmAs2);
			}
			else
			{
				RVLINVTRANSF3D(R, dX, R2, dX2)
				
				RVL6DOFInvTransfUncert(R, dX2, CAmAs, CAmAs2);
				RVLCOMPLETESIMMX3(CAmAs2)
				CTmp = CAmAs2 + 2 * 9;
				RVLCOMPLETESIMMX3(CTmp)
			}

			RVLMEM_ALLOC_STRUCT(m_pMem0, RVLQLISTHT_PTR_ENTRY, pLocalMapHTEntry)

			pLocalMapHTEntry->adr = (DWORD)(RVLPSULMBUILDER_MAXN_PSULMS * pPSuLM2->m_Index + pPSuLM->m_Index);
			pLocalMapHTEntry->Ptr = pNeighbor2;

			RVLQLISTHT_ADD_ENTRY(pListArray, pLocalMapHTEntry, m_LocalMapHT.m_Size, dwTmp)	

			pLocalMap2 = &(pPSuLM2->m_LocalMap);

			RVLQLIST_ADD_ENTRY(pLocalMap2, pNeighbor2)
		}
	}
}

void CRVLPSuLMBuilder::HypothesesPROSAC(CRVLPSuLM *pSPSuLM,
										CRVL3DPose *pPoseS0Init,	// Initial uncertainty for EKF update
										double *PInit,				// Uncertainty for initial matching
										CRVLPSuLM * pPrevSPSuLM)
{
	int nHypotheses = m_maxnHypothesesPerModel;

	int maxnM3DSurfaces = m_maxnDominant3DSurfaces;

	//int maxnSamples = 5;

	//int maxnFailures = 3;

	int maxnNodes = 100000;	

	//int QueueSize = 1000;

	//int PredictionHorizon = 3;

	//int maxnMatchesInHypothesis = 7;

	//int TraveledDistBinSize = 50;	// mm

	//double fTraveledDistBinSize = (double)TraveledDistBinSize;

	//int maxTraveledDist = 1000;	// mm

	//double fmaxTraveledDist = (double)maxTraveledDist;

	m_HypothesisList.RemoveAll();

	int nS3DSurfaces = pSPSuLM->m_n3DSurfaces;

	if(nS3DSurfaces == 0)
		return;

	int iLastSSurf = nS3DSurfaces - 1;

	CRVL3DSurface2 **SSurfArray = pSPSuLM->m_3DSurfaceArray;
	CRVL3DSurface2 **ppSSurf = SSurfArray;

	//RVLPSULM_SURF_MATCH_DATA *SSurfMatchData = new RVLPSULM_SURF_MATCH_DATA[nS3DSurfaces];

	//RVLPSULM_SURF_MATCH_DATA *pSSurfMatchData = SSurfMatchData;

	CRVL3DSurface2 *pS3DSurface;

	//int SSupportTotal = 0;

	int i;

	for(i=0;i<nS3DSurfaces;i++)
	{
		pS3DSurface = SSurfArray[i];

		pS3DSurface->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

		pS3DSurface->m_Index = i;

		//SSupportTotal += pS3DSurface->m_nSupport;

		//pSSurfMatchData->p3DSurface = pS3DSurface;

		//pSSurfMatchData->bMatched = 0;

		//pSSurfMatchData++;
	}

	CRVL3DSurface2 **pSSurfArrayEnd = SSurfArray + nS3DSurfaces;

	//int QueueSize = (10 * SSupportTotal) / 10 + 1;
	int QueueSize = nS3DSurfaces + maxnM3DSurfaces + 1;

	//**********************************

	//CRVL3DPose PoseSM;
	CRVL3DPose PoseSMInit;

	PoseSMInit.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	double CInit[3 * 3 * 3];
	double invtInit[3];
	PoseSMInit.m_C = CInit;
	double *R, *t;
	if(pPoseS0Init)
	{
		PoseSMInit.Copy(pPoseS0Init);
		PoseSMInit.m_pData = invtInit;
		R = PoseSMInit.m_Rot;
		t = PoseSMInit.m_X;
		RVLMULMX3X3TVECT(R, t, invtInit);
	}

	//PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	//double C[3 * 3 * 3];
	//PoseSM.m_C = C;
	//PoseSM.m_pData = invt;

	CRVLPSuLM *pMPSuLM;
	CRVL3DSurface2 *pM3DSurface;
	CRVL3DSurface2 **ppMSurf;
	//CRVLMPtrChain *pM3DSurfaceList;
	CRVL3DSurface2 **MSurfArray;
	int iHypothesis;
	double MatchQuality;
	double detQ;

	RVLPSULM_MSMATCH_DATA *MatchList;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_MSMATCH_DATA, nS3DSurfaces * maxnM3DSurfaces, MatchList);

	RVLPSULM_HG_NODE *NodeMem;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_HG_NODE, maxnNodes, NodeMem);

	RVLPSULM_HG_NODE *pNodeMemEnd = NodeMem + maxnNodes;

	CRVLQListArray Queue;

	Queue.m_Size = QueueSize + 1;

	Queue.Init();

	RVLQLIST *QueueListArray = Queue.m_ListArray;	

#ifdef RVLPSULM_LINES
	int nS3DLines = pSPSuLM->m_n3DLines;

	//CRVL3DLine2 *S3DLineArrayLastDOF;
	//
	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, CRVL3DLine2, nS3DLines, S3DLineArrayLastDOF);

	double RBTl[3 * 3];

	double *xTlB = RBTl + 0 * 3;
	double *yTlB = RBTl + 1 * 3;
	double *zTlB = RBTl + 2 * 3;

	double M3x3Tmp[3 * 3];

	double *M3x3TmpRow2 = M3x3Tmp + 3;
	double *M3x3TmpRow3 = M3x3Tmp + 2 * 3;

	double fnLastDOFGaussianSamples = (double)m_nLastDOFGaussianSamples;
	double kLastDOFScale = 2.5 / fnLastDOFGaussianSamples / m_LastDOFResolution;	// multiplying a std of a Gaussian by this factor
																					// gives the maximum distance between
																					// samples of this Gaussian relative to
																					// m_LastDOFResolution 
	double LastDOFLineRotationVar = m_LastDOFLineRotationStD * DEG2RAD;
	LastDOFLineRotationVar *= LastDOFLineRotationVar;
	double LastDOFLineTranslationVar = m_LastDOFLineTranslationStD * m_LastDOFLineTranslationStD;

	int nLastDOFBins = (1 << (int)(floor(log(floor(m_maxLastDOFTravelDist / m_LastDOFResolution) + 1.0) / RVLLOG2 + 1.0) + 1.0));

	//CRVL3DLine2 *M3DLineArrayLastDOF;
	int nM3DLines;
	int iS3DLine, iM3DLine;
	CRVL3DLine2 *pS3DLine, *pM3DLine;
	double *dXSA;
	RVL3DLINE_EXTENDED_DATA *pS3DLineData, *pM3DLineData;
	double V3x1Tmp[3];
	double fTmp, fTmp2;
	double *XS1A, *XS2A, *XM1, *XM2;
	double XS1B[3], XS2B[3];
	double dXSB[3];
	double *CXS1A, *CXS2A, *CXM1, *CXM2;
	double CXS1B[3 * 3], CXS2B[3 * 3];
	double XS1Tl[3], XS2Tl[3], XM1Tl[3], XM2Tl[3];
	double CXS1Tl[2 * 2], CXS2Tl[2 * 2], CXM1Tl[2 * 2], CXM2Tl[2 * 2];
	double CXS1Tl_[2 * 2], CXS2Tl_[2 * 2], CXM1Tl_[2 * 2], CXM2Tl_[2 * 2];
	double XS1Tl_[2], XS2Tl_[2], XM1Tl_[2], XM2Tl_[2];
	double VSB[3];
	double *VM;
	double kzOverlap, dzSTl, dzMTl;
	double sS1, sS2, sM1, sM2, minz2, maxz1;
	double e1[2], e2[2], C1[2 * 2], C2[2 * 2], detC1, detC2;
	double overlap;
	double SSupport;
	double maxvarTD;
	double ex1, ex2;
	double XS1A_[3], XS2A_[3];
	double *XSA_, *XMB_;
	int nLastDOFScales;
	double *LastDOFAccu, *LastDOFAccu_;
	int iLastDOFScale;
	double stdTD;
	int iLastDOFBin, iLastDOFBin1, iLastDOFBin2;
	double support;
	double LastDOFBinSize, kLastDOFScale2;
	int nLastDOFBins_;
	double *pLastDOFAccuBin, *pLastDOFAccuBelowBin, *pLastDOFAccuEnd, *pBestLastDOFAccuBin;
	double M3x3Tmp2[3 * 3];
	double CPC1[3 * 3], CPC2[3 * 3];
#endif

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
	pSPSuLM->Display(m_DebugData.pGUI, pPoseS0Init);

	CRVLFigure *pFig = m_DebugData.pGUI->OpenFigure("PSuLM");

	RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *pMouseCallbackData = 
		(RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA *)(pFig->m_vpMouseCallbackData);

	pMouseCallbackData->MatrixSceneModel = NULL;	

	//pMouseCallbackData->MSurfMatchData = MSurfMatchData;
	//pMouseCallbackData->SSurfMatchData = SSurfMatchData;

	CRVLFigure *pFig2 = m_DebugData.pGUI->OpenFigure("PSuLM2");	

	pFig->m_FontSize = pFig2->m_FontSize = 16;

	int nTextLines = 22;

	int TextImageHeight = nTextLines * pFig2->m_FontSize;

	pFig2->EmptyBitmap(cvSize(800, TextImageHeight), cvScalar(255, 255, 255));

	cvInitFont(&(pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	cvInitFont(&(pFig2->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);

	char Text[201];

	pFig2->PutText("Hello world!", cvPoint(0, pFig2->m_FontSize), cvScalar(0, 0, 0));

	m_DebugData.pGUI->ShowFigure(pFig2);
#endif

	RVLPSULM_HYPOTHESIS **HypothesisArray;

	RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_HYPOTHESIS *, nHypotheses, HypothesisArray);

	//int *TraveledDistAcc;

	//int nTraveledDistBins = 2 * maxTraveledDist / TraveledDistBinSize + 1;

	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, int, nTraveledDistBins, TraveledDistAcc);

	//int *TraveledDistBin =  TraveledDistAcc + maxTraveledDist / TraveledDistBinSize;

	//CRVLQListArray TraveledDistAccMatchPtrList;

	

	int HypothesisIndex = 0;

	double eigVal[3];
	
	//RVLMatrixHeader31->data.db = eigVal;

	double RBT[3 * 3];

	double *xTB = RBT;
	double *yTB = RBT + 3;
	double *zTB = RBT + 2 * 3;

	//RVLMatrixHeaderB33->data.db = eigVect;

	//CvMat *_eigVal = cvCreateMatHeader(3, 1, CV_64FC1);
	//CvMat *_eigVect = cvCreateMatHeader(3, 3, CV_64FC1);

	//_eigVal->data.db = eigVal;
	//_eigVect->data.db = V;

	RVLSURFACE_MATCH_ARRAY MatchData;

	double *e = MatchData.m_e;

	RVLPSULM_HYPOTHESIS *pHypothesis = NULL;

	int nM3DSurfaces;
	RVLPSULM_MSMATCH_DATA *pMSMatch;
	RVLPSULM_MSMATCH_DATA *pMSMatchListEnd;
	//int maxSurfIndexSum;
	CRVL3DSurface2 **pMSurfArrayEnd;
	//RVLPSULM_SURF_MATCH_DATA *pMSurfMatchData;
	//RVLPSULM_SURF_MATCH_DATA *pMSurfMatchDataArrayEnd;
	double *t2, *P2;
	//double *P, *R2, *P3;
	CRVL3DPose *pPoseSM;
	int Support;
	int iLastMSurf;
	int cost, minCost;
	//int MSupportTotal;
	RVLPSULM_HG_NODE *pNode, *pNewNode, *pPNode, *pNode2;
	//double *invt, *invt2;
	int iMinSupport, iMaxSupport, iMidSupport;
	int iMSurf, iSSurf;
	//int SSupport;
	//int nMSMatches;
	int iLastMSMatch;
	RVLQLIST *QueueList;
	int PredictedScore;
	int nExpandedNodes;
	RVLPSULM_HYPOTHESIS **ppHypothesis;
	CRVL3DPose *pPoseSM2;
	BOOL bNewHypothesis;
	double err;
	int maxCost;
	CRVL3DSurface2 **ppMatchedSSurf, **ppMatchedMSurf;
	double *pPTmp;
	double mu;
	double Jsmu[2], JsTD[2];
	double muTol;
	double TraveledDist, varTD;
	double k1;
	double *tFA, *tF2B, *RFA, *RF2B, *zTB2;
	double tF2A[3], tF2_FA[3], tFB[3], zTA[3], xTF2[3], yTF2[3], xTF[3], yTF[3], Q[3], RFB[3 * 3];
	double Mx3x3Tmp[3 * 3];
	double Vect3Tmp[3];
	double dx, dy, dtFB[3], eF, vartFAx, vartFAy, vartFAz, vartF2Bx, vartF2By, vartF2Bz, eTD, csT, wTD;
	//int iTraveledDistBin, iBestTraveledDistBin, BestTravelDistScore;
	int nLastDOFMatches;
	RVLPSULM_LASTDOF_MATCH_DATA *pLastDOFMatchData, *pLastDOFMatchData2, *pLastDOFMatchArrayEnd, *pBestLastDOFMatch;
	double TDScore, BestTDScore;
	//BOOL bLastDOFMatchStored, bLastDOFMatchMerged;
	//int iNewLastDOFMatch;
	RVLPSULM_HYPOTHESIS *pHypothesis2;
	double Vect3Tmp2[3];
	int nFeatures;
	double eig[3], Veig[3];
	BOOL bReal[3];
	RVLPSULM_LASTDOF_MATCH_DATA *LastDOFMatchArray;
	int LastDOFMatchArraySize;

	//int debug_nMatchings = 0;
	//int debug_nQueueSorts = 0;
	
#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
	FILE *fpLog;

	fopen_s(&fpLog, "C:\\RVL\\Debug\\PSuLMHypGen.dat", "w");
#endif

	CRVLMPtrChain *pPSuLMList;

	if(m_PSuLMSubList.m_nElements !=0 || (m_Flags & RVLPSULMBUILDER_FLAG_NEW_SUBMAP) != 0)
		pPSuLMList = &m_PSuLMSubList;
	else
		pPSuLMList = &m_PSuLMList;


	pPSuLMList->Start();

	while(TRUE)
	{
		double StartTime = m_pTimer->GetTime();

		// initial feature matching 

		if(pPrevSPSuLM)
			pMPSuLM = pPrevSPSuLM;
		else if(pPSuLMList->m_pNext)
			pMPSuLM = (CRVLPSuLM *)(pPSuLMList->GetNext());
		else
			break;

		if(pPoseS0Init == NULL)
		{
			PoseSMInit.Copy(&(pMPSuLM->m_PoseCsInit));
			PoseSMInit.m_pData = invtInit;
			R = PoseSMInit.m_Rot;
			t = PoseSMInit.m_X;
			RVLMULMX3X3TVECT(R, t, invtInit);
		}

		//if(pMPSuLM->m_Index != 13)	// debug
		//	continue;

		MSurfArray = pMPSuLM->m_3DSurfaceArray;

		ppMSurf = MSurfArray;

		nM3DSurfaces = pMPSuLM->m_n3DSurfaces;

		maxCost = nS3DSurfaces + nM3DSurfaces;

		iLastMSurf = nM3DSurfaces - 1;

		//pMSurfMatchData = MSurfMatchData;

		//MSupportTotal = 0;

		for(i=0;i<nM3DSurfaces;i++)
		{
			pM3DSurface = MSurfArray[i];

			pM3DSurface->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;

			*(ppMSurf++) = pM3DSurface;

			//pMSurfMatchData->p3DSurface = pM3DSurface;

			//pMSurfMatchData->bMatched = 0;

			//pMSurfMatchData++;

			//MSupportTotal += pM3DSurface->m_nSupport;		// move to load or create function
		}

		//SupportTotal = (SSupportTotal + MSupportTotal) * 80 / 100;

		pMSurfArrayEnd = ppMSurf;

		Queue.Reset();

		//pMSurfMatchDataArrayEnd = pMSurfMatchData;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG
		pMPSuLM->AddToFigure(m_DebugData.pGUI);

		//pMouseCallbackData->nMSurfs = pM3DSurfaceList->m_nElements;

		cvWaitKey();
#endif
		// create a sorted list of matches

		pMSMatch = MatchList;

		pM3DSurface = MSurfArray[0];
		pS3DSurface = SSurfArray[0];
		pMSMatch->pMData = (RVLPSULM_SURF_MATCH_DATA *)pM3DSurface;
		pMSMatch->pSData = (RVLPSULM_SURF_MATCH_DATA *)pS3DSurface;
		//pMSMatch->Support = pM3DSurface->m_nSupport + pS3DSurface->m_nSupport;
		pMSMatch->Support = maxCost;
		pMSMatch->Flags = 0x00;

		iLastMSMatch = 0;

		pPTmp = PoseSMInit.m_C;

		if(pPoseS0Init)
			PoseSMInit.m_C = PInit;

		for(iSSurf = 0; iSSurf < nS3DSurfaces; iSSurf++)
		{
			pS3DSurface = SSurfArray[iSSurf];

			//SSupport = pS3DSurface->m_nSupport;

			for(iMSurf = 0; iMSurf < nM3DSurfaces; iMSurf++)
			{
				//if(iSSurf == 1 && iMSurf == 1)
				//	int debug = 1;

				pM3DSurface = MSurfArray[iMSurf];	

				////////// Initial match by color histogram - FILKO
				//if ((pM3DSurface->m_noUsedColorPts >= 20) && (pS3DSurface->m_noUsedColorPts >= 20))
				//{
					//if ((m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL) && (pS3DSurface->RVLIntersectColorHistogram(pM3DSurface, 5, true, m_IntersectionHelperArray) < m_MatIntersectTHR))
					//	continue;
				//}

				////////// Initial match by LBP histogram - FILKO
				//if ((pM3DSurface->m_noUsedLbpPts >= 20) && (pS3DSurface->m_noUsedLbpPts >= 20))
				//{
				//	if ((m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL) && (CRVL3DMeshObject::RVLIntersectHistograms(pS3DSurface->m_LBP_RIU_VAR, pM3DSurface->m_LBP_RIU_VAR, pS3DSurface->m_noUsedLbpPts, pM3DSurface->m_noUsedLbpPts, 17*26, m_pMem2, -1, m_IntersectionHelperArray) < m_MatIntersectTHR))
				//		continue;
				//}

				if(!pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, detQ, &MatchData))
					continue;

				//Support = DOUBLE2INT(sqrt((double)(SSupport * pM3DSurface->m_nSupport)));
				//Support = SSupport + pM3DSurface->m_nSupport;
				Support = maxCost - (iSSurf + iMSurf);

				if(Support <= MatchList[iLastMSMatch].Support)
				{
					iLastMSMatch++;

					pMSMatch = MatchList + iLastMSMatch;
				}
				else
				{
					iMaxSupport = 0;
		
					iMinSupport = iLastMSMatch;

					while(iMinSupport - iMaxSupport > 1)
					{
						iMidSupport = (iMinSupport + iMaxSupport) / 2;

						if(Support > MatchList[iMidSupport].Support)
							iMinSupport = iMidSupport;
						else
							iMaxSupport = iMidSupport;
					}

					memmove(MatchList + iMinSupport + 1, MatchList + iMinSupport, 
						(iLastMSMatch - iMinSupport + 1) * sizeof(RVLPSULM_MSMATCH_DATA));

					pMSMatch = MatchList + iMinSupport;

					iLastMSMatch++;
				}

				pMSMatch->pMData = (RVLPSULM_SURF_MATCH_DATA *)pM3DSurface;
				pMSMatch->pSData = (RVLPSULM_SURF_MATCH_DATA *)pS3DSurface;
				pMSMatch->Support = Support;
				pMSMatch->Flags = 0x00;
			}
		}

		//If no matches was found
		if(iLastMSMatch == 0)
		{
   			if(pPrevSPSuLM)
				break;

			continue;
		}

		PoseSMInit.m_C = pPTmp;

		pMSMatchListEnd = MatchList + iLastMSMatch + 1;

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG

		fprintf(fpLog, "Model %d\n\n", pMPSuLM->m_Index);	// for Nyarko

		fprintf(fpLog, "Initial Pose Uncertainty\n");	// for Nyarko

		RVLPrintCov(fpLog, PoseSMInit.m_C, 3, RAD2DEG);	// for Nyarko

		RVLPrintCov(fpLog, PoseSMInit.m_C + 2 * 3 * 3, 3);	// for Nyarko

		fprintf(fpLog, "\n");	// for Nyarko

		fprintf(fpLog, "Match List:\n");

		for(pMSMatch = MatchList; pMSMatch < pMSMatchListEnd; pMSMatch++)
		{
			pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
			pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

			if(pS3DSurface->Match2(pM3DSurface, &PoseSMInit, MatchQuality, detQ, &MatchData))
				fprintf(fpLog, "S%d-M%d\t\t%d\n", 
					((CRVL3DSurface2 *)(pMSMatch->pSData))->m_Index,
					((CRVL3DSurface2 *)(pMSMatch->pMData))->m_Index,
					pMSMatch->Support);
		}
#endif

		//Karlo - PROSAC 05.03.2014
		//
		int noRounds = ProsacEX(MatchList, iLastMSMatch, pPoseS0Init, pMPSuLM);
		//return;

		double ExecutionTime = m_pTimer->GetTime() - StartTime;

   		if(pPrevSPSuLM)
			break;

	}	// for each model PSuLM	

#ifdef RVLPSULMBUILDER_HYPOTHESES_DEBUG_LOG
	fclose(fpLog);
#endif

	

	m_pMem2->Clear();

	// only for debugging purpose

}


//Generate hypothesis using PROSAC
//**********************************//
//Ondrej Chum and Jiri Matas. 2005. 
//Matching with PROSAC " Progressive Sample Consensus. 
//In Proceedings of the 2005 IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR'05) 
//Volume 1 - Volume 01 (CVPR '05), Vol. 1. IEEE Computer Society, Washington, DC, USA, 220-226. 
//DOI=10.1109/CVPR.2005.221 http://dx.doi.org/10.1109/CVPR.2005.221
//**********************************//
int CRVLPSuLMBuilder::ProsacEX(RVLPSULM_MSMATCH_DATA *pMatchList, int N, CRVL3DPose *pPoseS0Init,CRVLPSuLM *pMPSuLM)
{
	//N -> total number of possible Correspondences

	//Constants
	const int minNoPROSACSamples = 3;						//Complexity of the model (m)
	int maxPROSACRounds = 500;	//
	int TN = 200000;				//after how many samples do PROSAC and RANSAC behave identically
	//const int TN = niter_RANSAC(PROSAC_P_GOOD_SAMPLE, PROSAC_MAX_OUTLIERS_PROPORTION, minNoPROSACSamples, -1);
	//const int IN_min = (1. - PROSAC_MAX_OUTLIERS_PROPORTION)*N; // the minimum number of total inliers
	const double logeta0 = log(PROSAC_ETA0);
	
	
	//Variables
	int sampledIndices[minNoPROSACSamples]; 
	int n;									//top number of sorted matches. Size of current subset. We draw samples from the set U_n of the top n data points
	int t;									//number of trials in current subset. Iteration number
	int *isInlier;	// the support of a given model, computed from a given sample. The first n values of the isInlier array will be filled with 0's (outlier) and 1's (inlier)
	int *pPerm;  // used for random permutation of the first n correspondences
	
	int n_star; // termination length (see sec. 2.2 Stopping criterion)
    int In_star; // number of inliers found within the first n_star data points
    int IN_best; // best number of inliers found so far (store the model that goes with it)
	int kn_star; // number of samples to draw to reach the maximality constraint

	int I_N; // total number of inliers for a given sample

	double Tn; // average number of samples {M_i}_{i=1}^{TN} that contain samples from U_n only
	int Tn_prime; // integer version of Tn, see eq. (4)
	double Tn_1;

	int n_best; // best value found so far in terms of inliers ratio (the value between n_test+1 and N that maximizes the ratio In_best/n_best)
    int In_best; // number of inliers for n_best

	int n_test; // test value for the termination length
    int In_test; // the number of inliers for the n_test first correspondences
    double epsilon_n_best; //fraction of inliers for n_best

	int currentPROSACRound;

	RVLPSULM_MSMATCH_DATA *pMSMatch;
	int i,j;					//common variables
	BOOL bTerminate;
	CRVL3DPose *pPoseSM, *pPoseSMFinal, *pPoseSMBest, *pPoseSMTemp;


	CRVL3DSurface2 *pS3DSurface, *pM3DSurface;
	CRVL3DSurface2 **ppMatchedSSurf, **ppMatchedMSurf;

	RVLSURFACE_MATCH_ARRAY MatchData;
	CRVL3DPose PoseSM, PoseSMFinal;//, PoseSMBest;
	
	double *Rot, *trans;
	double invtInit[3], invtInit1[3];
	double CInit[3 * 3 * 3], CInit1[3 * 3 * 3];
	double MatchQuality;
	double detQ;
	BOOL bValidPose;
	RVLPSULM_HYPOTHESIS *pHypothesis;//, *pHypothesisBest, *pHypothesisTemp;

	//RVLPSULM_MSMATCH_DATA *MatchListTemp;
	RVLPSULM_MSMATCH_DATA **MatchListTemp = new RVLPSULM_MSMATCH_DATA *[N];
	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_MSMATCH_DATA, N, pMatchListTemp);

	//RVLPSULM_MSMATCH_DATA *pMatchListTemp = new RVLPSULM_MSMATCH_DATA[N];
	RVLPSULM_MSMATCH_DATA **ppMatchListTemp;



	PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	PoseSM.m_C = CInit;
	PoseSM.m_pData = invtInit;


	PoseSMFinal.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	PoseSMFinal.m_C = CInit1;
	PoseSMFinal.m_pData = invtInit1;



	pPerm = new int[N];
	isInlier = new int[N];

	//Initialize
	pHypothesis = NULL;
	

	bTerminate = 0;
	currentPROSACRound = 0;

	t = 0;
	n = minNoPROSACSamples;
	n_star = N;

    In_star = 0;
    IN_best = 0;
    Tn_prime = 1;
    kn_star = TN;
	Tn = TN * BinomialCoefficient(n,minNoPROSACSamples)/ BinomialCoefficient(N,minNoPROSACSamples);

	int nRansacTotal = 0;

	pHypothesis = NULL;
	
	if(N>=minNoPROSACSamples)
	{
		while (bTerminate == FALSE) 
		{

			

			for (n=minNoPROSACSamples; n<=N; n++)
			{
				

				for (currentPROSACRound = 0; currentPROSACRound<maxPROSACRounds; currentPROSACRound++)
				{
					nRansacTotal++;
					// 1. Choice of the hypothesis generation set
					// 2. Draw semi-random sample. Get sample indices from candidates 1:n
					//*******************************************//
					//draw m random samples from 1 to n
					if(n<16)
					{
						// draw m-1 random samples from 1 to n-1  and add the nth point
						RandPerm(n-1, pPerm);
						for(i=0; i<minNoPROSACSamples-1; i++)
						{
							sampledIndices[i] = pPerm[i]; 
						}

						//the n-th point is mandatory
						sampledIndices[minNoPROSACSamples-1] = n - 1;
					}
					else
					{
						RandPerm(N, pPerm);
						for(i=0; i<minNoPROSACSamples; i++)
						{
							sampledIndices[i] = pPerm[i]; 
						}
					}
					
				
					

					//3.Model parameter estimation
					//Using the sampled indices, remove the corresponding matched pairs from pMatchList and generate hypothesis
					//*******************************************//
					//Reset initial pose
					if(pPoseS0Init)
					{
						PoseSM.Copy(pPoseS0Init);
						Rot = PoseSM.m_Rot;
						trans = PoseSM.m_X;
						RVLMULMX3X3TVECT(Rot, trans, invtInit);
					}
					//Reset pose pointers
					pPoseSM = &PoseSM;
					pPoseSMFinal = &PoseSMFinal;

					bValidPose = TRUE;
					
					//Generate hypothesis using EKF
					for(i=0; i<minNoPROSACSamples; i++)
					{
						//Get match
						pMSMatch = pMatchList + sampledIndices[i];

						//Get surface pair
						pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
						pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

						bValidPose = 0;
						bValidPose = RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, pPoseSM, pPoseSMFinal, &MatchData);
						if(bValidPose)
						{
							//Switch uncertainties
							pPoseSMTemp = pPoseSM;
							pPoseSM = pPoseSMFinal;
							pPoseSMFinal = pPoseSMTemp;
						}
						else
						{
							break;
						}
						
					}
					//*******************************************//
					
					I_N = 0;
					
					//4. Evaluate Hypothesis - ie calculate all the number of inliers 
					// From first paragraph of section 2: "The hypotheses are veri?ed against all data"
					//*******************************************//
					//If Pose is valid 
					if(bValidPose)
					{
						
						//Reset temp list
						ppMatchListTemp = MatchListTemp;

						pMSMatch = pMatchList;

						//Go through all pairs in subset
						for(j=0; j<N ;j++,pMSMatch++)
						{
							isInlier[j] = 0;

							//pMSMatch = pMatchList + j;

							//Get surface pair
							pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
							pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);
							//Calculate I_N (pPoseSM has the final pose)
							if(pS3DSurface->Match2(pM3DSurface, pPoseSM, MatchQuality, detQ, &MatchData, RVL3DSURFACE_FLAG_HYPOTHESIS_EVALUATION))
							{
								I_N++;
								*(ppMatchListTemp++) = pMSMatch;
								isInlier[j] = 1;
							}
						}
						
					}
					//*******************************************//


					//5. Check for better solution
					//*******************************************//
					//If inliers exist and are better
					if(I_N > IN_best)
					{
						//1. Store current best solution
						IN_best = I_N;
						
						if(pHypothesis == NULL)
						{
							RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);
							RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, CRVL3DSurface2 *, 2 * N, pHypothesis->MatchArray);
							
							pHypothesis->cost = 0;
							pHypothesis->visible = 0;
							pHypothesis->iS = pHypothesis->iM = 0;
							pHypothesis->pMPSuLM = pMPSuLM;
							pHypothesis->Flags = 0x00;
							pHypothesis->nMatches = 0;
						}
						
						pPoseSMBest = &(pHypothesis->PoseSM);
						pPoseSMBest->m_C = pHypothesis->P;
						pPoseSMBest->m_pData = pHypothesis->invt;
						pPoseSMBest->m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
						

						pPoseSMBest->Copy(pPoseSM);
						Rot = pPoseSMBest->m_Rot;
						trans = pPoseSMBest->m_X;
						RVLMULMX3X3TVECT(Rot, trans,  pHypothesis->invt);

						pHypothesis->nMatches = I_N;
						

						//Copy consensus matches in match array
						ppMatchedSSurf = (CRVL3DSurface2 **)(pHypothesis->MatchArray);
						ppMatchedMSurf = ppMatchedSSurf + pHypothesis->nMatches;

						for(i=0; i<I_N; i++)
						{
							pMSMatch = (RVLPSULM_MSMATCH_DATA *)(MatchListTemp[i]);

							*(ppMatchedSSurf++) = (CRVL3DSurface2 *)(pMSMatch->pSData);
							*(ppMatchedMSurf++) = (CRVL3DSurface2 *)(pMSMatch->pMData);
						}
					
					}
				}

				if(n>15)
				{
					bTerminate = TRUE; //Nothing will happen without the break
					break;
				}
			
			} //end for j=0:N
			
			bTerminate = TRUE;
			

		}
	}

	if(pHypothesis)
		m_HypothesisList.Add(pHypothesis);

	delete[] MatchListTemp;
	delete[] pPerm;
	delete[] isInlier;
	
	return currentPROSACRound;
}



int CRVLPSuLMBuilder::Prosac(RVLPSULM_MSMATCH_DATA *pMatchList, int N, CRVL3DPose *pPoseS0Init,CRVLPSuLM *pMPSuLM)
{
	//N -> total number of possible Correspondences

	//Constants
	const int minNoPROSACSamples = 3;						//Complexity of the model (m)
	int maxPROSACRounds = 5000;	//
	int TN = 200000;				//after how many samples do PROSAC and RANSAC behave identically
	//const int TN = niter_RANSAC(PROSAC_P_GOOD_SAMPLE, PROSAC_MAX_OUTLIERS_PROPORTION, minNoPROSACSamples, -1);
	//const int IN_min = (1. - PROSAC_MAX_OUTLIERS_PROPORTION)*N; // the minimum number of total inliers
	const double logeta0 = log(PROSAC_ETA0);
	
	
	//Variables
	int sampledIndices[minNoPROSACSamples]; 
	int n;									//top number of sorted matches. Size of current subset. We draw samples from the set U_n of the top n data points
	int t;									//number of trials in current subset. Iteration number
	int *isInlier;	// the support of a given model, computed from a given sample. The first n values of the isInlier array will be filled with 0's (outlier) and 1's (inlier)
	int *pPerm;  // used for random permutation of the first n correspondences
	
	int n_star; // termination length (see sec. 2.2 Stopping criterion)
    int In_star; // number of inliers found within the first n_star data points
    int IN_best; // best number of inliers found so far (store the model that goes with it)
	int kn_star; // number of samples to draw to reach the maximality constraint

	int I_N; // total number of inliers for a given sample

	double Tn; // average number of samples {M_i}_{i=1}^{TN} that contain samples from U_n only
	int Tn_prime; // integer version of Tn, see eq. (4)
	double Tn_1;

	int n_best; // best value found so far in terms of inliers ratio (the value between n_test+1 and N that maximizes the ratio In_best/n_best)
    int In_best; // number of inliers for n_best

	int n_test; // test value for the termination length
    int In_test; // the number of inliers for the n_test first correspondences
    double epsilon_n_best; //fraction of inliers for n_best

	int currentPROSACRound;

	RVLPSULM_MSMATCH_DATA *pMSMatch;
	int i,j;					//common variables
	BOOL bTerminate;
	CRVL3DPose *pPoseSM, *pPoseSMFinal, *pPoseSMBest, *pPoseSMTemp;


	CRVL3DSurface2 *pS3DSurface, *pM3DSurface;
	CRVL3DSurface2 **ppMatchedSSurf, **ppMatchedMSurf;

	RVLSURFACE_MATCH_ARRAY MatchData;
	CRVL3DPose PoseSM, PoseSMFinal;//, PoseSMBest;
	
	double *Rot, *trans;
	double invtInit[3], invtInit1[3];
	double CInit[3 * 3 * 3], CInit1[3 * 3 * 3];
	double MatchQuality;
	double detQ;
	BOOL bValidPose;
	RVLPSULM_HYPOTHESIS *pHypothesis;//, *pHypothesisBest, *pHypothesisTemp;

	//RVLPSULM_MSMATCH_DATA *MatchListTemp;
	RVLPSULM_MSMATCH_DATA **MatchListTemp = new RVLPSULM_MSMATCH_DATA *[N];
	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_MSMATCH_DATA, N, pMatchListTemp);

	//RVLPSULM_MSMATCH_DATA *pMatchListTemp = new RVLPSULM_MSMATCH_DATA[N];
	RVLPSULM_MSMATCH_DATA **ppMatchListTemp;



	PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	PoseSM.m_C = CInit;
	PoseSM.m_pData = invtInit;


	PoseSMFinal.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	PoseSMFinal.m_C = CInit1;
	PoseSMFinal.m_pData = invtInit1;



	pPerm = new int[N];
	isInlier = new int[N];

	//Initialize
	pHypothesis = NULL;
	

	bTerminate = 0;
	currentPROSACRound = 0;

	t = 0;
	n = minNoPROSACSamples;
	n_star = N;

    In_star = 0;
    IN_best = 0;
    Tn_prime = 1;
    kn_star = TN;
	Tn = TN * BinomialCoefficient(n,minNoPROSACSamples)/ BinomialCoefficient(N,minNoPROSACSamples);

	pHypothesis = NULL;
	
	if(N>=minNoPROSACSamples)
	{
		while (bTerminate == FALSE) //bTerminate == FALSE  (t <= kn_star) && (t < TN) && (t <= maxPROSACRounds)
		{

			I_N = 0;

			currentPROSACRound++;

			// 1. Choice of the hypothesis generation set
			//*******************************************//
			t = t + 1;
			if ((t > Tn_prime) && (n < n_star)) 
			{
				Tn_1 = (Tn * (n+1)) / (n+1-minNoPROSACSamples);
				Tn_prime = Tn_prime + ceil(Tn_1 - Tn);
				Tn = Tn_1;
				n = n + 1; //increase subset number
			}
			//*******************************************//

			//2.  Draw semi-random sample. Get sample indices from candidates 1:n
			//*******************************************//
			if (t > Tn_prime) 
			{
				//draw m random samples from 1 to n
				RandPerm(n, pPerm);
				for(i=0; i<minNoPROSACSamples; i++)
				{
					sampledIndices[i] = pPerm[i]; 
				}
				
			}
			else
			{
				// draw m-1 random samples from 1 to n-1  and add the nth point
				RandPerm(n-1, pPerm);
				for(i=0; i<minNoPROSACSamples-1; i++)
				{
					sampledIndices[i] = pPerm[i]; 
				}

				//the n-th point is mandatory
				sampledIndices[minNoPROSACSamples-1] = n - 1;
			}
			//*******************************************//

			
			//3.Model parameter estimation
			//Using the sampled indices, remove the corresponding matched pairs from pMatchList and generate hypothesis
			//*******************************************//
			//Reset initial pose
			if(pPoseS0Init)
			{
				PoseSM.Copy(pPoseS0Init);
				Rot = PoseSM.m_Rot;
				trans = PoseSM.m_X;
				RVLMULMX3X3TVECT(Rot, trans, invtInit);
			}
			//Reset pose pointers
			pPoseSM = &PoseSM;
			pPoseSMFinal = &PoseSMFinal;

			bValidPose = TRUE;
			
			//Generate hypothesis using EKF
			for(i=0; i<minNoPROSACSamples; i++)
			{
				//Get match
				pMSMatch = pMatchList + sampledIndices[i];

				//Get surface pair
				pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
				pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

				bValidPose = 0;
				bValidPose = RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, pPoseSM, pPoseSMFinal, &MatchData);
				if(bValidPose)
				{
					//Switch uncertainties
					pPoseSMTemp = pPoseSM;
					pPoseSM = pPoseSMFinal;
					pPoseSMFinal = pPoseSMTemp;
				}
				else
				{
					break;
				}
			}
			//*******************************************//
			
			//pHypothesis = NULL;
			
			//4. Evaluate Hypothesis - ie calculate all the number of inliers 
			// From first paragraph of section 2: "The hypotheses are veri?ed against all data"
			//*******************************************//
			//If Pose is valid 
			if(bValidPose)
			{
				
				//Reset temp list
				ppMatchListTemp = MatchListTemp;

				pMSMatch = pMatchList;

				//Go through all pairs in subset
				for(j=0; j<N ;j++,pMSMatch++)
				{
					isInlier[j] = 0;

					//pMSMatch = pMatchList + j;

					//Get surface pair
					pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
					pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);
					//Calculate n (pPoseSM has the final pose)
					if(pS3DSurface->Match2(pM3DSurface, pPoseSM, MatchQuality, detQ, &MatchData, RVL3DSURFACE_FLAG_HYPOTHESIS_EVALUATION))
					{
						I_N++;
						*(ppMatchListTemp++) = pMSMatch;
						isInlier[j] = 1;
					}
				}
				
			}
			//*******************************************//


			//5. Check termination conditions
			//*******************************************//
			//If inliers exist and are better
			if(I_N > IN_best)
			{
				//1. Store current best solution
				IN_best = I_N;
				
				if(pHypothesis == NULL)
				{
					RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);
					RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, CRVL3DSurface2 *, 2 * N, pHypothesis->MatchArray);
					
					pHypothesis->cost = 0;
					pHypothesis->visible = 0;
					pHypothesis->iS = pHypothesis->iM = 0;
					pHypothesis->pMPSuLM = pMPSuLM;
					pHypothesis->Flags = 0x00;
					pHypothesis->nMatches = 0;
				}
				
				pPoseSMBest = &(pHypothesis->PoseSM);
				pPoseSMBest->m_C = pHypothesis->P;
				pPoseSMBest->m_pData = pHypothesis->invt;
				pPoseSMBest->m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
				

				pPoseSMBest->Copy(pPoseSM);
				Rot = pPoseSMBest->m_Rot;
				trans = pPoseSMBest->m_X;
				RVLMULMX3X3TVECT(Rot, trans,  pHypothesis->invt);

				pHypothesis->nMatches = I_N;
				

				//Copy consensus matches in match array
				ppMatchedSSurf = (CRVL3DSurface2 **)(pHypothesis->MatchArray);
				ppMatchedMSurf = ppMatchedSSurf + pHypothesis->nMatches;

				for(i=0; i<I_N; i++)
				{
					pMSMatch = (RVLPSULM_MSMATCH_DATA *)(MatchListTemp[i]);

					*(ppMatchedSSurf++) = (CRVL3DSurface2 *)(pMSMatch->pSData);
					*(ppMatchedMSurf++) = (CRVL3DSurface2 *)(pMSMatch->pMData);
				}

				

				//2. Check termination criteria 
				n_best = N;
				In_best = I_N;
				epsilon_n_best = (double)In_best/n_best;
				
				for(n_test = N, In_test = I_N; n_test > minNoPROSACSamples; n_test--) 
				{
					if((n_test < In_test) || (In_test < NonRandomness(minNoPROSACSamples,n_test)))
					{
						break;
					}

					n_best = n_test;
					In_best = In_test;
					epsilon_n_best = (double)In_best/n_best;

					// prepare for next loop iteration
					In_test = In_test - isInlier[n_test-1];

					//while((n_test >= In_test) && (In_test >= NonRandomness(m,n_test)))
					//{
					//	n_best = n_test;
		//                In_best = In_test;
		//                epsilon_n_best = (double)In_best/n_best;

					//	// prepare for next loop iteration
		//                In_test = In_test - isInlier[n_test-1];
					//}

				}

				// is the best one we found even better than n_star?
				if ((In_best * n_star > In_star * n_best) &&  (n_best >= In_best))
				{
	                
					// update all values
					n_star = n_best;
					In_star = In_best;
					kn_star = RansacRoundsNeeded(TN,minNoPROSACSamples,logeta0,n_star,In_star);
				}

				//bTerminate = (IN_best >= IN_min && t > kn_star) || (t >= TN) || (t > maxPROSACRounds);
				

			}
			//*******************************************//
	//RansacRoundsNeeded(int maxNoRounds, int m, double logeta0 = log(1-p), int n, int In)
	//int niter_RANSAC(double p=(1-exp(logeta0), double epsilon=(1.-In/(double)n), int s=m, int Nmax=maxNoRounds);

			bTerminate = (t > kn_star) || (t >= TN) || (t > maxPROSACRounds);

		}
	}

	if(pHypothesis)
		m_HypothesisList.Add(pHypothesis);

	delete[] MatchListTemp;
	delete[] pPerm;
	delete[] isInlier;
	
	return currentPROSACRound;
}



int CRVLPSuLMBuilder::Ransac(RVLPSULM_MSMATCH_DATA *pMatchList, int N, CRVL3DPose *pPoseS0Init,CRVLPSuLM *pMPSuLM)
{
	//N -> total number of possible Correspondences

	//Constants
	const int minNoPROSACSamples = 3;						//Complexity of the model (m)
	int maxPROSACRounds = 5000;	//
	int TN = 200000;				//after how many samples do PROSAC and RANSAC behave identically
	//const int TN = niter_RANSAC(PROSAC_P_GOOD_SAMPLE, PROSAC_MAX_OUTLIERS_PROPORTION, minNoPROSACSamples, -1);
	//const int IN_min = (1. - PROSAC_MAX_OUTLIERS_PROPORTION)*N; // the minimum number of total inliers
	const double logeta0 = log(PROSAC_ETA0);
	
	
	//Variables
	int sampledIndices[minNoPROSACSamples]; 
	int n;									//top number of sorted matches. Size of current subset. We draw samples from the set U_n of the top n data points
	int t;									//number of trials in current subset. Iteration number
	int *isInlier;	// the support of a given model, computed from a given sample. The first n values of the isInlier array will be filled with 0's (outlier) and 1's (inlier)
	int *pPerm;  // used for random permutation of the first n correspondences
	
	int n_star; // termination length (see sec. 2.2 Stopping criterion)
    int In_star; // number of inliers found within the first n_star data points
    int IN_best; // best number of inliers found so far (store the model that goes with it)
	int kn_star; // number of samples to draw to reach the maximality constraint

	int In;					//number of correct matches/inliers/consensus in current subset 
	int I_N; // total number of inliers for a given sample

	double Tn; // average number of samples {M_i}_{i=1}^{TN} that contain samples from U_n only
	int Tn_prime; // integer version of Tn, see eq. (4)
	double Tn_1;

	int n_best; // best value found so far in terms of inliers ratio (the value between n_test+1 and N that maximizes the ratio In_best/n_best)
    int In_best; // number of inliers for n_best

	int n_test; // test value for the termination length
    int In_test; // the number of inliers for the n_test first correspondences
    double epsilon_n_best; //fraction of inliers for n_best

	int currentPROSACRound;

	RVLPSULM_MSMATCH_DATA *pMSMatch;
	int i,j;					//common variables
	BOOL bTerminate;
	CRVL3DPose *pPoseSM, *pPoseSMFinal, *pPoseSMBest, *pPoseSMTemp;


	CRVL3DSurface2 *pS3DSurface, *pM3DSurface;
	CRVL3DSurface2 **ppMatchedSSurf, **ppMatchedMSurf;

	RVLSURFACE_MATCH_ARRAY MatchData;
	CRVL3DPose PoseSM, PoseSMFinal;//, PoseSMBest;
	
	double *Rot, *trans;
	double invtInit[3], invtInit1[3];
	double CInit[3 * 3 * 3], CInit1[3 * 3 * 3];
	double MatchQuality;
	double detQ;
	BOOL bValidPose;
	RVLPSULM_HYPOTHESIS *pHypothesis;//, *pHypothesisBest, *pHypothesisTemp;

	//RVLPSULM_MSMATCH_DATA *MatchListTemp;
	RVLPSULM_MSMATCH_DATA **MatchListTemp = new RVLPSULM_MSMATCH_DATA *[N];
	//RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem2, RVLPSULM_MSMATCH_DATA, N, pMatchListTemp);

	//RVLPSULM_MSMATCH_DATA *pMatchListTemp = new RVLPSULM_MSMATCH_DATA[N];
	RVLPSULM_MSMATCH_DATA **ppMatchListTemp;



	PoseSM.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	PoseSM.m_C = CInit;
	PoseSM.m_pData = invtInit;


	PoseSMFinal.m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
	PoseSMFinal.m_C = CInit1;
	PoseSMFinal.m_pData = invtInit1;



	pPerm = new int[N];
	isInlier = new int[N];

	//Initialize
	pHypothesis = NULL;
	
	//In = 0;
	//Inbest = 0;

	bTerminate = 0;
	currentPROSACRound = 0;

	t = 0;
	n = minNoPROSACSamples;
	n_star = N;

    In_star = 0;
    IN_best = 0;
    Tn_prime = 1;
    kn_star = TN;
	Tn = TN * BinomialCoefficient(n,minNoPROSACSamples)/ BinomialCoefficient(N,minNoPROSACSamples);

	pHypothesis = NULL;

	
	while (currentPROSACRound < maxPROSACRounds) //bTerminate == FALSE  (t <= kn_star) && (t < TN) && (t <= maxPROSACRounds)
	{

		I_N = 0;

		currentPROSACRound++;

		

		

		//1.  Draw semi-random sample. Get sample indices from candidates 1:n
		//*******************************************//
		
		//draw m random samples from 1 to n
		RandPerm(N, pPerm);
		for(i=0; i<minNoPROSACSamples; i++)
		{
			sampledIndices[i] = pPerm[i]; 
		}
			
		
		//*******************************************//

		
		//2.Model parameter estimation
		//Using the sampled indices, remove the corresponding matched pairs from pMatchList and generate hypothesis
		//*******************************************//
		//Reset initial pose
		if(pPoseS0Init)
		{
			PoseSM.Copy(pPoseS0Init);
			Rot = PoseSM.m_Rot;
			trans = PoseSM.m_X;
			RVLMULMX3X3TVECT(Rot, trans, invtInit);
		}
		//Reset pose pointers
		pPoseSM = &PoseSM;
		pPoseSMFinal = &PoseSMFinal;

		bValidPose = TRUE;
		
		//Generate hypothesis using EKF
		for(i=0; i<minNoPROSACSamples; i++)
		{
			//Get match
			pMSMatch = pMatchList + sampledIndices[i];

			//Get surface pair
			pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
			pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);

			bValidPose = 0;
			bValidPose = RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, pPoseSM, pPoseSMFinal, &MatchData);
			if(bValidPose)
			{
				//Switch uncertainties
				pPoseSMTemp = pPoseSM;
				pPoseSM = pPoseSMFinal;
				pPoseSMFinal = pPoseSMTemp;
			}
			else
			{
				break;
			}
		}
		//*******************************************//
		
		
		//3. Evaluate Hypothesis - ie calculate all the number of inliers 
		// From first paragraph of section 2: "The hypotheses are veri?ed against all data"
		//*******************************************//
		//If Pose is valid 
		if(bValidPose)
		{
			
			//Reset temp list
			ppMatchListTemp = MatchListTemp;

			pMSMatch = pMatchList;

			//Go through all pairs in subset
			for(j=0; j<N ;j++,pMSMatch++)
			{
				isInlier[j] = 0;

				//pMSMatch = pMatchList + j;

				//Get surface pair
				pM3DSurface = (CRVL3DSurface2 *)(pMSMatch->pMData);
				pS3DSurface = (CRVL3DSurface2 *)(pMSMatch->pSData);
				//Calculate n (pPoseSM has the final pose)
				if(pS3DSurface->Match2(pM3DSurface, pPoseSM, MatchQuality, detQ, &MatchData, RVL3DSURFACE_FLAG_HYPOTHESIS_EVALUATION))
				{
					I_N++;
					*(ppMatchListTemp++) = pMSMatch;
					isInlier[j] = 1;
				}
			}
			
		}
		//*******************************************//


		//4. Check termination conditions
		//*******************************************//
		//If inliers exist and are better
		if(I_N > IN_best)
		{
			//1. Store current best solution
			IN_best = I_N;
			
			if(pHypothesis == NULL)
			{
				RVLMEM_ALLOC_STRUCT(m_pMem, RVLPSULM_HYPOTHESIS, pHypothesis);
				RVLMEM_ALLOC_STRUCT_ARRAY(m_pMem, CRVL3DSurface2 *, 2 * N, pHypothesis->MatchArray);
				
				pHypothesis->cost = 0;
				pHypothesis->visible = 0;
				pHypothesis->iS = pHypothesis->iM = 0;
				pHypothesis->pMPSuLM = pMPSuLM;
				pHypothesis->Flags = 0x00;
				pHypothesis->nMatches = 0;
			}
			
			pPoseSMBest = &(pHypothesis->PoseSM);
			pPoseSMBest->m_C = pHypothesis->P;
			pPoseSMBest->m_pData = pHypothesis->invt;
			pPoseSMBest->m_ParamFlags = RVL3DPOSE_PARAM_FLAGS_COV_6D;
			

			pPoseSMBest->Copy(pPoseSM);
			Rot = pPoseSMBest->m_Rot;
			trans = pPoseSMBest->m_X;
			RVLMULMX3X3TVECT(Rot, trans,  pHypothesis->invt);

			pHypothesis->nMatches = I_N;
			

			//Copy consensus matches in match array
			ppMatchedSSurf = (CRVL3DSurface2 **)(pHypothesis->MatchArray);
			ppMatchedMSurf = ppMatchedSSurf + pHypothesis->nMatches;

			for(i=0; i<I_N; i++)
			{
				pMSMatch = (RVLPSULM_MSMATCH_DATA *)(MatchListTemp[i]);

				*(ppMatchedSSurf++) = (CRVL3DSurface2 *)(pMSMatch->pSData);
				*(ppMatchedMSurf++) = (CRVL3DSurface2 *)(pMSMatch->pMData);
			}

			

		}
		//*******************************************//

	}

	if(pHypothesis)
		m_HypothesisList.Add(pHypothesis);

	delete[] MatchListTemp;
	delete[] pPerm;
	delete[] isInlier;
	
	return currentPROSACRound;
}

void CRVLPSuLMBuilder::DisplayHypothesis(CRVLGUI *pGUI, 
										 CRVLFigure *pFig, 
										 CRVLFigure *pFig2, 
										 CRVLPSuLM *pSPSuLM, 										 
										 DWORD Flags,
										 IplImage *pImage,
										 IplImage *pImage2,
										 int iHypothesis)
{
	pFig->m_pImage = cvCloneImage(pImage);

	CRVL3DPose NullPose;

	RVLNULL3VECTOR(NullPose.m_X);
	RVLUNITMX3(NullPose.m_Rot);

	RVLPSULM_HYPOTHESIS *pHypothesis = NULL;

	if(m_HypothesisList.m_nElements > 0 && iHypothesis >= 0 && iHypothesis < m_HypothesisList.m_nElements)
		pHypothesis = m_HypothesisArray[iHypothesis];

	if(pHypothesis)
	{
		CRVLPSuLM *pMPSuLM = pHypothesis->pMPSuLM;

		DWORD HypEvalMethod = (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD);

		if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
		{
			EvaluateHypothesis3(pSPSuLM, pHypothesis);

			UpdateMatchMatrixForDisplay(pSPSuLM, pHypothesis);
		}
		else if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
		{
			if(pHypothesis == pMPSuLM->m_pHypothesis)
				PoseConstraintProbability(pSPSuLM, pMPSuLM);

			DWORD EvaluateHypothesisFlags = (m_Flags2 & RVLPSULMBUILDER_FLAG2_FIRST_ORDER_DEPENDENCY_TREE ? 
				RVLPSULMBUILDER_HYPEVAL4_FLAG_FIRST_ORDER_DEPENDENCY_TREE : 0x00000000);

			if (m_Flags2 & RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING)
				EvaluateHypothesisFlags |= RVLPSULMBUILDER_HYPEVAL4_FLAG_DYNAMIC_SURF_DETECT;

			pHypothesis->Probability = EvaluateHypothesis4(pSPSuLM, pHypothesis, EvaluateHypothesisFlags) - m_PriorProbabilityWorldModel;
		}
		else
			CreateMatchMatrix(pSPSuLM, pMPSuLM, &(pHypothesis->PoseSM), 1.0);

		pSPSuLM->Display(pFig, &NullPose, cvScalar(0, 255, 0), Flags);

		cvReleaseImage(&(pFig2->m_pImage));
		
		if(m_Flags & RVLPSULMBUILDER_FLAG_PC)
		{
			int wExt = (2 * m_pPSD->m_nFOVExtensions + 1) * m_pPSD->m_Width;

			pFig2->EmptyBitmap(cvSize(wExt, m_pPSD->m_Height), cvScalar(0, 0, 0));
		}
		else if((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
			pFig2->m_pImage = cvCloneImage(pImage2);
		else if ((m_Flags2 & RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT) == RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG)
		{
			pFig2->m_pImage = cvCreateImage(cvSize(pImage->width, pImage->height), IPL_DEPTH_8U, 3);				

			IplImage *pHRImage = cvLoadImage(pMPSuLM->m_FileName);

			cvResize(pHRImage, pFig2->m_pImage);

			cvReleaseImage(&pHRImage);
		}
		else
			pFig2->m_pImage = ((pMPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX) ? GetComplexPSuLMRGBImage(pMPSuLM->m_FileName) :
				cvLoadImage(pMPSuLM->m_FileName));

		pMPSuLM->Display(pFig2, &NullPose, cvScalar(0, 255, 0), 
			RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_ELLIPSES | RVLPSULM_DISPLAY_LINES | RVLPSULM_DISPLAY_VECTORS);
	}
	else
		pSPSuLM->Display(pFig, &NullPose, cvScalar(0, 255, 0), Flags);

	if (m_Flags2 & RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING)
	{
		RVLPSULM_MESH_FILE_GROUP_DATA MeshGroupData[2];

		MeshGroupData[0].Mask = 0x03;
		MeshGroupData[0].Value = 0x00;
		MeshGroupData[0].MaterialID = 30;
		MeshGroupData[1].Mask = 0x03;
		MeshGroupData[1].Value = 0x03;
		MeshGroupData[1].MaterialID = 5;

		FILE *fp = fopen("C:\\RVL\\Debug\\PSuLM.obj", "w");

		fprintf(fp, "mtllib kinect_example.mtl\n");

		int iVertex0 = 1;

		CRVL3DPose Pose;

		Pose.Reset();

		pSPSuLM->AddToMeshFile("scene", fp, iVertex0, &Pose, 1.0, MeshGroupData, 2);

		if (pHypothesis)
		{
			InverseTransform3D(Pose.m_Rot, Pose.m_X, pHypothesis->PoseSM.m_Rot, pHypothesis->PoseSM.m_X);

			MeshGroupData[0].MaterialID = 11;
			MeshGroupData[1].MaterialID = 31;

			pHypothesis->pMPSuLM->AddToMeshFile("model", fp, iVertex0, &Pose, 0.5, MeshGroupData, 2);
		}

		fclose(fp);
	}
}

void CRVLPSuLMBuilder::DisplayHypothesisData(	CRVLFigure *pFig, 
												CRVLPSuLM *pSPSuLM,
												char *MatchMatrixGT,
												DWORD Flags,
												int iHypothesis,
												CRVL3DSurface2 *pSelectedSurface,
												CRVL3DLine2 *pSelectedLine,
												RVL3DSURFACE_SAMPLE *pSelectedSurfSample)
{
	RVLPSULM_HYPOTHESIS *pHypothesis = NULL;

	if(m_HypothesisList.m_nElements > 0 && iHypothesis >= 0 && iHypothesis < m_HypothesisList.m_nElements)
		pHypothesis = m_HypothesisArray[iHypothesis];

	int HypScoreLow = 20000;
	int HypScoreHigh = 50000;

	int HypScoreMid = (HypScoreLow + HypScoreHigh) / 2;
	int HypScoreHalfRange = HypScoreMid - HypScoreLow;

	IplImage *pDataDisplay = cvCreateImage(cvSize(1000, 300), IPL_DEPTH_8U, 3);	

	// display text

	int iTextLine = 0;

	double LowProbability = 0.99;
	double MidProbability = 0.5 * (1.0 + LowProbability);

	DWORD HypEvalMethod = (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD);

	char str[200];
	double Probability;

	if(pHypothesis)
	{
		if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)	
		{
			if(pHypothesis->pMPSuLM->m_pHypothesis == pHypothesis)
			{
				Probability = ((m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING ||
					(m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION && 
					(m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING) ?
					pHypothesis->pMPSuLM->m_PosteriorProbabilityLocal : pHypothesis->pMPSuLM->m_PosteriorProbabilityGlobal);

				if(Probability < LowProbability)
					cvSet(pDataDisplay, cvScalar(0, 0, 255));
				else if(Probability < MidProbability)
					cvSet(pDataDisplay, cvScalar(0, 255.0 * (Probability - LowProbability) / (MidProbability - LowProbability), 255));
				else if(Probability < 1.0)
					cvSet(pDataDisplay, cvScalar(0, 255, 255.0 *(1.0 - Probability) / (1.0 - MidProbability)));
				else
					cvSet(pDataDisplay, cvScalar(0, 255, 0));
			}
			else
				cvSet(pDataDisplay, cvScalar(204, 204, 204));
		}
		else
		{
			if(pHypothesis->cost < HypScoreLow)
				cvSet(pDataDisplay, cvScalar(0, 0, 255));
			else if(pHypothesis->cost < HypScoreMid)
				cvSet(pDataDisplay, cvScalar(0, 255 * (pHypothesis->cost - HypScoreLow) / HypScoreHalfRange, 255));
			else if(pHypothesis->cost < HypScoreHigh)
				cvSet(pDataDisplay, cvScalar(0, 255, 255 - 255 * (pHypothesis->cost - HypScoreMid) / HypScoreHalfRange));
			else
				cvSet(pDataDisplay, cvScalar(0, 255, 0));
		}
	}	//if(pHypothesis)
	else
		cvSet(pDataDisplay, cvScalar(204, 204, 204));

	cvPutText(pDataDisplay, m_ImageFileName, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

	if(pSPSuLM->m_Index != 0xffffffff)
	{
		sprintf(str, "Model %d", pSPSuLM->m_Index);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
	}

	if(m_PSuLMList.m_nElements > 0)
	{
		sprintf(str, "no. of local models=%d", m_PSuLMList.m_nElements);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
	}

	sprintf(str, "no. of hypotheses=%d", m_HypothesisList.m_nElements);

	cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

	if(pHypothesis)
	{
		sprintf(str, "Hypothesis %d (%d/%d)", pHypothesis->Index, iHypothesis, m_nHypotheses);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

		if(pHypothesis->iRepresentative != 0xffffffff)
		{
			RVLPSULM_HYPOTHESIS *pHypothesis_ = pHypothesis;

			RVLPSULM_HYPOTHESIS *pHypothesis__;

			while(pHypothesis_->iRepresentative != 0xffffffff)
			{
				m_HypothesisList.Start();

				for(int i = 0; i <= (int)(pHypothesis_->iRepresentative); i++)
					pHypothesis__ = (RVLPSULM_HYPOTHESIS *)(m_HypothesisList.GetNext());

				pHypothesis_ = pHypothesis__;
			}

			sprintf(str, "Representative=%d", pHypothesis_->Index);
		}
		else
			sprintf(str, "REPRESENTATIVE");

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

		sprintf(str, "Match Score = %d", pHypothesis->cost);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

		if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
		{
			sprintf(str, "Likelihood = %lf", pHypothesis->Probability);

			cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
		}

		if(pHypothesis == pHypothesis->pMPSuLM->m_pHypothesis)
		{
			char validation;

			if((Flags & RVLPSULM_DISPLAY_VALIDATION) && MatchMatrixGT != NULL)
			{
				switch(MatchMatrixGT[pHypothesis->pMPSuLM->m_Index]){
				case 1:
					validation = '+';

					break;
				case 0:
					validation = '?';

					break;
				case -1:
					validation = '-';

					break;
				default:
					validation = ' ';
				}

				sprintf(str, "Manual validation: %c", validation);

				cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
			}

			//sprintf(str, "Probability Local 5DoF = %lf", m_BestHypothesisProbability5DOF);
			sprintf(str, "Probability Local 5DoF = %lf", pHypothesis->pMPSuLM->m_PosteriorProbabilityLocal5DOF);

			cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

			//sprintf(str, "Probability Local = %lf", m_BestHypothesisProbability);
			sprintf(str, "Probability Local = %lf", pHypothesis->pMPSuLM->m_PosteriorProbabilityLocal);

			cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

			sprintf(str, "Probability Global = %lf", pHypothesis->pMPSuLM->m_PosteriorProbabilityGlobal);

			cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
		}

		if(pHypothesis->pMPSuLM->m_Name[0] == 0)
			sprintf(str, "Place %d", pHypothesis->pMPSuLM->m_Index);
		else
			sprintf(str, "Place = %s", pHypothesis->pMPSuLM->m_Name);				

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

		CRVL3DPose *pPoseSM = &(pHypothesis->PoseSM);

		sprintf(str, "t=(%6.0lf, %6.0lf, %6.0lf), Rot=(%6.1lf, %6.1lf, %6.1lf)", 
			pPoseSM->m_X[0], pPoseSM->m_X[1], pPoseSM->m_X[2], 
			pPoseSM->m_Alpha * RAD2DEG, pPoseSM->m_Beta * RAD2DEG, pPoseSM->m_Theta * RAD2DEG);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

		FILE *fp = fopen("C:\\RVL\\Debug\\NieghborPSuLMs.txt", "w");

		RVLPSULM_NEIGHBOR2 *pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pHypothesis->pMPSuLM->m_LocalMap.pFirst);
	
		while(pNeighbor)
		{
			fprintf(fp, "%d\n", pNeighbor->pMPSuLM->m_Index);

			pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
		}	

		fclose(fp);
	}	// if(pHypothesis)

	if(pSelectedSurface)
	{
		sprintf(str, "Selected Surface: %d (%d pts.)", pSelectedSurface->m_Index, pSelectedSurface->m_nSupport);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

		if(pFig->m_Flags & RVLPSULM_DISPLAY_SCENE)
		{
			if(pHypothesis)
			{
				if((m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
				{
					if(pSelectedSurface->m_Index < pSPSuLM->m_n3DSurfaces)
					{
						double P = 0.0;

						double *pP = m_MatrixSMCost + pSelectedSurface->m_Index * (pHypothesis->pMPSuLM->m_n3DSurfaces + pHypothesis->pMPSuLM->m_n3DLines);

						int iMSurf = -1;

						int i;

						for(i = 0; i < pHypothesis->pMPSuLM->m_n3DSurfaces; i++, pP++)
							if(*pP > P)
							{
								P = *pP;
								iMSurf = i;
							}

						if(iMSurf >= 0)
						{
							sprintf(str, "Matched with M%d, P=%lf", iMSurf, P);

							cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
						}
					}
				}
			}
		}
	}

	if(pSelectedLine)
	{
		sprintf(str, "Selected Line: %d", pSelectedLine->m_Index);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

		if(pFig->m_Flags & RVLPSULM_DISPLAY_SCENE)
		{
			if(pHypothesis)
			{
				if((m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
				{
					if(pSelectedLine->m_Index < pSPSuLM->m_n3DLines)
					{
						double P = 0.0;

						double *pP = m_MatrixSMCost + pSelectedLine->m_Index * (pHypothesis->pMPSuLM->m_n3DSurfaces + pHypothesis->pMPSuLM->m_n3DLines) +
							pHypothesis->pMPSuLM->m_n3DSurfaces;

						int iMSurf = -1;

						int i;

						for(i = 0; i < pHypothesis->pMPSuLM->m_n3DLines; i++, pP++)
							if(*pP > P)
							{
								P = *pP;
								iMSurf = i;
							}

						if(iMSurf >= 0)
						{
							sprintf(str, "Matched with M%d, P=%lf", iMSurf, P);

							cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
						}
					}
				}
			}
		}
	}

	if (pSelectedSurfSample)
	{
		sprintf(str, "Selected Surface Sample: %d ()", pSelectedSurfSample->Index);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font, cvScalar(0, 0, 0));
	}

	if(m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING)
		cvPutText(pDataDisplay, "MAP BUILDING", cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

	if(m_Flags & RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING)
		cvPutText(pDataDisplay, "LOOP CLOSING", cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

	if(m_Flags & RVLPSULMBUILDER_FLAG_GLOBAL)
		cvPutText(pDataDisplay, "GLOBAL", cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
	else if(m_pNearestModelPSuLM)
	{
		sprintf(str, "Querry model: %d", m_pNearestModelPSuLM->m_Index);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
	}

	if(m_pLoopStartPSuLM)
	{
		sprintf(str, "Loop Start: %d", m_pLoopStartPSuLM->m_Index);

		cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));
	}

	//sprintf(str, "Exec. Time = %4.0lf ms", 1000.0 * ExecutionTime);

	//cvPutText(pDataDisplay, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(0, 0, 0));

	cvShowImage("Real Time Display", pDataDisplay);

	cvReleaseImage(&pDataDisplay);
}

CRVLPSuLM * CRVLPSuLMBuilder::GetPSuLM(int index)
{
	CRVLPSuLM *pMPSuLM;

	m_PSuLMList.Start();
	while(m_PSuLMList.m_pNext)
	{
		pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());
		if(pMPSuLM->m_Index == index)
			return pMPSuLM;
	}

	return NULL;
}

void CRVLPSuLMBuilder::UpdateBuffers(CRVLPSuLM *pPSuLM)
{
	if(pPSuLM->m_n3DSurfacesTotal > m_maxnModel3DSurfaces)
	{
		m_maxnModel3DSurfaces = pPSuLM->m_n3DSurfacesTotal;

		if(m_SurfaceMatchData.Cp_)
			delete[] m_SurfaceMatchData.Cp_;

		m_SurfaceMatchData.Cp_ = new double[3 * 3 * m_maxnModel3DSurfaces];

		if(m_SurfaceMatchData.invCp_)
			delete[] m_SurfaceMatchData.invCp_;

		m_SurfaceMatchData.invCp_ = new double[3 * 3 * m_maxnModel3DSurfaces];

		if(m_SurfaceMSArray)
			delete[] m_SurfaceMSArray;

		m_SurfaceMSArray = new CRVL3DSurface2[m_maxnModel3DSurfaces];
	}

	if(pPSuLM->m_n3DLinesTotal > m_maxnModel3DLines)
		m_maxnModel3DLines = pPSuLM->m_n3DLinesTotal;
}

bool CRVLPSuLMBuilder::MapBuilding(CRVLPSuLM *pSPSuLM)
{
	CRVLPSuLM *pNewPSuLM;

#ifdef RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
	if(m_pNearestModelPSuLM == NULL)
	{
		m_maxPSuLMIndex = -1;

		pNewPSuLM = Clone(pSPSuLM);

		m_PSuLMList.Add(pNewPSuLM);

		m_pNearestModelPSuLM = pNewPSuLM;

		m_Flags &= ~RVLPSULMBUILDER_FLAG_KIDNAPPED;

		return true;
	}
#else
	if (m_PSuLMList.m_nElements == 0)
	{
		m_maxPSuLMIndex = -1;

		pNewPSuLM = Clone(pSPSuLM);

		UpdateBuffers(pSPSuLM);

		m_PSuLMList.Add(pNewPSuLM);

		if (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_GENERATION_INDEXING)
			m_Indexing.UpdateBase();

		m_Flags &= ~RVLPSULMBUILDER_FLAG_KIDNAPPED;

		return true;
	}
#endif

	//if(m_nPlausibleHypotheses == 0)
	//	return false;	

	CRVLPSuLM *pMPSuLM;

	if (!(m_Flags2 & RVLPSULMBUILDER_FLAG2_MAPBUILDING_MANUAL))
	{
		if (m_nHypotheses == 0)
			return false;

#ifdef RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
		RVLPSULM_HYPOTHESIS *pHypothesis;

		if((m_Flags & RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING) && m_pLoopStartPSuLM)
		{
			bool bPreviousPSuLMMatched = false;
			bool bLoopStartPSuLMMatched = false;

			int iHypothesis;
			CRVL3DPose *pPoseRTPrevPSuLM, *pPoseRTLoopStartPSuLM;

			for(iHypothesis = 0; iHypothesis < m_nPlausibleHypotheses && !(bPreviousPSuLMMatched && bLoopStartPSuLMMatched); iHypothesis++)
			{
				pHypothesis = m_HypothesisArray[iHypothesis];

				if(pHypothesis->pMPSuLM == m_pNearestModelPSuLM && !bPreviousPSuLMMatched)
				{
					bPreviousPSuLMMatched = true;

					pPoseRTPrevPSuLM = &(pHypothesis->PoseSM);
				}

				if(pHypothesis->pMPSuLM == m_pLoopStartPSuLM && !bLoopStartPSuLMMatched)
				{
					bLoopStartPSuLMMatched = true;

					pPoseRTLoopStartPSuLM = &(pHypothesis->PoseSM);
				}
			}

			if(bPreviousPSuLMMatched && bLoopStartPSuLMMatched)
			{
				pNewPSuLM = Clone(pSPSuLM);

				m_PSuLMList.Add(pNewPSuLM);

				Connect(m_pNearestModelPSuLM, pNewPSuLM, NULL, pPoseRTPrevPSuLM);

				Connect(m_pLoopStartPSuLM, pNewPSuLM, NULL, pPoseRTLoopStartPSuLM);

				m_pNearestModelPSuLM = pNewPSuLM;

				m_pLoopStartPSuLM = NULL;

				m_Flags &= ~RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING;
			}
			else
				return false;
		}	// if((m_Flags & RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING) && m_pLoopStartPSuLM)
		else
		{
			pHypothesis = m_HypothesisArray[0];

			if(pHypothesis->pMPSuLM->m_PosteriorProbabilityLocal < 0.999)
				return false;

			CRVL3DPose *pPoseSM = &(pHypothesis->PoseSM);

			double V[3];
			double eAlpha;

			pPoseSM->GetAngleAxis(V, eAlpha);

			double *t = pPoseSM->m_X;

			double dist = sqrt(RVLDOTPRODUCT3(t, t));

			if(dist < m_MinHybridLocalizationDist && eAlpha * RAD2DEG < m_MinHybridLocalizationAngle)
				return false;

			pNewPSuLM = Clone(pSPSuLM);

			m_PSuLMList.Add(pNewPSuLM);

			Connect(m_pNearestModelPSuLM, pNewPSuLM, NULL, pPoseSM);

			m_pNearestModelPSuLM = pNewPSuLM;
	}
#else	// !RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
		bool bCovered = false;
		bool bTracking = false;

		CRVL3DPose *pPoseSM;
		double V[3];
		double eAlpha;
		double *t;
		double dist;

		m_PSuLMList.Start();

		while (m_PSuLMList.m_pNext)
		{
			pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());

			if (pMPSuLM->m_pHypothesis == NULL)
				continue;

			//if(pMPSuLM->m_PosteriorProbabilityLocal < 0.999)
			//if(pMPSuLM->m_PosteriorProbabilityLocal5DOF < 30.0)
			if (pMPSuLM->m_PosteriorProbabilityLocal5DOF < 50.0)
				continue;

			bTracking = true;

			pPoseSM = &(pMPSuLM->m_pHypothesis->PoseSM);

			pPoseSM->GetAngleAxis(V, eAlpha);

			t = pPoseSM->m_X;

			dist = sqrt(RVLDOTPRODUCT3(t, t));

			//if(bCovered =(dist < m_MinHybridLocalizationDist && eAlpha * RAD2DEG < m_MinHybridLocalizationAngle))
			//	break;
		}

		if (bCovered)
			return false;
	}

	//if(!bTracking)
	//	return false;

	pNewPSuLM = Clone(pSPSuLM);

	UpdateBuffers(pSPSuLM);

	m_PSuLMList.Add(pNewPSuLM);

	if (m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_GENERATION_INDEXING)
		m_Indexing.UpdateBase();

	if(!(m_Flags2 & RVLPSULMBUILDER_FLAG2_MAPBUILDING_MANUAL))
	{
		m_PSuLMList.Start();

		while(m_PSuLMList.m_pNext)
		{
			pMPSuLM = (CRVLPSuLM *)(m_PSuLMList.GetNext());

			if(pMPSuLM == pNewPSuLM)
				break;

			if(pMPSuLM->m_pHypothesis == NULL)
				continue;

			//if(pMPSuLM->m_PosteriorProbabilityLocal < 0.999)
			if(pMPSuLM->m_PosteriorProbabilityLocal5DOF < 30.0)
				continue;

			Connect(pMPSuLM, pNewPSuLM, NULL, &(pMPSuLM->m_pHypothesis->PoseSM));
		}
	}
	
#endif	// !RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
	
	return true;
}

void CRVLPSuLMBuilder::PoseConstraintProbability(CRVLPSuLM *pSPSuLM, 
												 CRVLPSuLM *pMPSuLM)
{
	RVLPSULM_HYPOTHESIS *pHypothesis = pMPSuLM->m_pHypothesis;

	EvaluateHypothesis4(pSPSuLM, pHypothesis, 0x00000000);

	double Y[3*3];
	double Y_[3*3];

	RVLNULLMX3X3(Y)

	int nMFeatures = pHypothesis->pMPSuLM->m_n3DSurfaces + pHypothesis->pMPSuLM->m_n3DLines; 

	int nSSurfaces = pSPSuLM->m_n3DSurfaces;

	double POffset = m_SurfaceMatchData.PPriorPosition + RVLLN4PI;

	int i;
	double P;
	double *N;

	for(i = 0; i < nSSurfaces; i++)
	{
		P = exp(m_SMatchArray[i].P - POffset);

		N = pSPSuLM->m_3DSurfaceArray[i]->m_N;
		
		RVLMULVECT3VECT3T(N, N, Y_)

		RVLSCALEMX3X3(Y_, P, Y_)

		RVLSUMMX3X3(Y, Y_, Y)
	}

	POffset = m_LineMatchData.PPriorPosition + RVLLN4PI;

	double *V1;
	double V2[3];
	double V3[3];
	int i_, j_, k_;
	double fTmp;

	for(i = 0; i < pSPSuLM->m_n3DLines; i++)
	{
		P = exp(m_SMatchArray[nSSurfaces + i].P - POffset);

		V1 = ((RVL3DLINE_EXTENDED_DATA *)(pSPSuLM->m_3DLineArray[i]->m_pData))->V;

		//RVLORTHOGONAL3(V1, V2, i_, j_, k_, V3, fTmp)
		RVLORTHOGONAL3(V1, V2, i_, j_, k_, fTmp)

		RVLCROSSPRODUCT3(V1, V2, V3);

		RVLMULVECT3VECT3T(V1, V1, Y_)
		
		RVLSCALEMX3X3(Y_, P, Y_)

		RVLSUMMX3X3(Y, Y_, Y)

		RVLMULVECT3VECT3T(V2, V2, Y_)
		
		RVLSCALEMX3X3(Y_, P, Y_)

		RVLSUMMX3X3(Y, Y_, Y)
	}

	double eig[3];
	BOOL bReal[3];
	double LastDOF[3];

	RVLGetMinEigVector3(Y, eig, bReal, LastDOF);

#ifdef RVLPSULMBUILDER_POSE_CONSTRAINT_PROBABILITY_DEBUG_LOG
	FILE *fp = fopen("C:\\RVL\\Debug\\LastDOFEvidence.txt", "w");

	fprintf(fp, "Last DOF: %lf, %lf, %lf\n\n", LastDOF[0], LastDOF[1], LastDOF[2]);
#endif

	//double PLastDOF = 0.0;

	//double maxPLastDOF = 0.0;
	//int iDominantLastDOFFeature = -1;

	double OrthogonalityThr = cos(20.0 * DEG2RAD);
	double lnPOrthogonal = log(0.9);				// probability that an accidental surface is orthogonal to two other surfaces

	CRVL3DSurface2 *pSSurf, *pSSurf1, *pSSurf2;
	double *RFC;
	double e;
	double *N1, *N2;
	double maxP;

	for(i = 0; i < nSSurfaces; i++)
	{
		if(m_SMatchArray[i].b)
		{			
			pSSurf = pSPSuLM->m_3DSurfaceArray[i];

			N = pSSurf->m_N;

			fTmp = RVLDOTPRODUCT3(N, LastDOF);

			if(RVLABS(fTmp) >= COS45)
			{
				//P = m_SMatchArray[i].P;

				//PLastDOF += P;

				//if(P > maxPLastDOF)
				//{
				//	maxPLastDOF = P;
				//	iDominantLastDOFFeature = i;
				//}	

				maxP = 0.0;

				for(j_ = 0; j_ < nSSurfaces; j_++)
				{
					if(j_ == i)
						continue;

					pSSurf1 = pSPSuLM->m_3DSurfaceArray[j_];

					N1 = pSSurf1->m_N;

					for(k_ = 0; k_ < j_; k_++)
					{
						if(k_ == i)
							continue;

						pSSurf2 = pSPSuLM->m_3DSurfaceArray[k_];

						N2 = pSSurf2->m_N;

						if(RVLABS(RVLDOTPRODUCT3(N1, N2)) > COS45)
							continue;

						RVLCROSSPRODUCT3(N1, N2, V2)

						RVLNORM3(V2, fTmp)

						fTmp = RVLDOTPRODUCT3(V2, N);

						if(fTmp < 0.0)
						{
							RVLNEGVECT3(V2, V2);

							fTmp = -fTmp;
						}

						if(fTmp < OrthogonalityThr)
							continue;
					
						RFC = pSSurf->m_Pose.m_Rot;

						RVLMULMX3X3TVECT(RFC, V2, V3)

						e = V3[0] * V3[0] / pSSurf->m_varq[0] + V3[1] * V3[1] / pSSurf->m_varq[1];

						P = RVLLN4PI - 0.5 * (log(pSSurf->m_varq[0] * pSSurf->m_varq[1]) + e) - RVLLN2PI + lnPOrthogonal;

						if(P > maxP)
							maxP = P;
					}
				}

				if(maxP < m_SMatchArray[i].P)
					m_SMatchArray[i].P = maxP;

#ifdef RVLPSULMBUILDER_POSE_CONSTRAINT_PROBABILITY_DEBUG_LOG
				fprintf(fp, "S%d: normal angle=%lf, PPrior=%lf\n", i, acos(RVLABS(fTmp)) * RAD2DEG, P);
#endif
			}
		}
	}

	for(i = 0; i < pSPSuLM->m_n3DLines; i++)
	{
		i_ = nSSurfaces + i;

		if(m_SMatchArray[i_].b)
		{
			V1 = ((RVL3DLINE_EXTENDED_DATA *)(pSPSuLM->m_3DLineArray[i]->m_pData))->V;

			fTmp = RVLDOTPRODUCT3(V1, LastDOF);

			if(RVLABS(fTmp) <= COS45)
			{
				//P = m_SMatchArray[i_].P;

				//PLastDOF += P;

				//if(P > maxPLastDOF)
				//{
				//	maxPLastDOF = P;
				//	iDominantLastDOFFeature = i_;
				//}

				m_SMatchArray[i_].b = false;

#ifdef RVLPSULMBUILDER_POSE_CONSTRAINT_PROBABILITY_DEBUG_LOG
				fprintf(fp, "L%d: angle=%lf\n", i, acos(RVLABS(fTmp)) * RAD2DEG);
#endif
			}
		}
	}

#ifdef RVLPSULMBUILDER_BEST_HYPOTHESIS_PROBABILITY_DEBUG_LOG
	fclose(fp);
#endif

	//if(iDominantLastDOFFeature >= 0)
	//	PLastDOF = PLastDOF - maxPLastDOF + m_SMatchArray[iDominantLastDOFFeature].POrientMatch;

	//m_BestHypothesisProbability = m_BestHypothesisProbability5DOF / (1.0 + 3.0 * exp(-PLastDOF));	

	//m_BestHypothesisProbability = 1.0 / (PriorP + 10.0 * exp(ConditionalProbabilityTree(pSPSuLM, pHypothesis->pMPSuLM) - BestP));

	pMPSuLM->m_PosteriorProbabilityLocal = 1.0 / (pMPSuLM->m_PriorProbabilityLocal + 
		10.0 * exp(ConditionalProbabilityTree(pSPSuLM, pHypothesis->pMPSuLM) - pHypothesis->Probability));
}

void CRVLPSuLMBuilder::UpdateRelativePoseUncertainties()
{
	CRVL3DPose Pose;

	FILE *fp = fopen("C:\\RVL\\Debug\\UpdateRelativePoseUncertainties.log", "w");

	DWORD OldFlags = m_Flags;

	m_Flags &= ~RVLPSULMBUILDER_FLAG_MODE;

	m_Flags |= (RVLPSULMBUILDER_FLAG_MODE_TRACKING | RVLPSULMBUILDER_FLAG_KIDNAPPED | RVLPSULMBUILDER_FLAG_MAPBUILDING);

	CRVL3DPose PoseMS;

	double *R_ = PoseMS.m_Rot;
	double *t_ = PoseMS.m_X;

	int i;
	CRVLPSuLM *pPSuLM;
	//CRVLPSuLM *pPSuLM_;
	RVLQLIST_PTR_ENTRY *pNeighborPtr;
	RVLPSULM_NEIGHBOUR *pNeighborRel;
	RVLPSULM_HYPOTHESIS *pHypothesis;
	double dist, angle;
	//BYTE result;	// 0 - OK; 1 - large error; 2 - no hypotheses
	//double invR[9], invt[3];
	double *R, *C;
	//double *t;

	for(i = 0; i <= m_maxPSuLMIndex; i++)
	{
		pPSuLM = m_PSuLMArray[i];

		if(pPSuLM == NULL)
			continue;
		
		pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pPSuLM->m_NeighbourList->pFirst);	

		while(pNeighborPtr)
		{
			pNeighborRel = (RVLPSULM_NEIGHBOUR *)(pNeighborPtr->Ptr);

#ifdef NEVER	// Correction 1
			pPSuLM_ = pNeighborRel->pPSuLM;

			fprintf(fp, "%d-%d: ", pPSuLM->m_Index, pPSuLM_->m_Index);

			m_pMem->Clear();

			//if(pPSuLM->m_Index == 0 && pPSuLM_->m_Index == 22)
			//	int debug = 0;

			Localization(pPSuLM_, &Pose, pPSuLM);	

			if(m_nHypotheses > 0)
			{
				pHypothesis = m_HypothesisArray[0];

				pNeighborRel->pPoseRel->Diff(&(pHypothesis->PoseSM), dist, angle);
				
				if(dist <= 200.0 && RVLABS(angle) <= 6.0 * DEG2RAD)
				{
					result = 0;

					fprintf(fp, "OK.\n");

					memcpy(pNeighborRel->pPoseRel->m_C, pHypothesis->PoseSM.m_C, 3*3*3*sizeof(double));
				}
				else
				{
					fprintf(fp, "ERROR: dist=%lf, angle=%lf (Hypothesis %d). Computing opposite... ", dist, angle * RAD2DEG, pHypothesis->Index);

					result = 1;
				}
			}
			else
			{
				fprintf(fp, "ERROR: No hypotheses. Computing opposite... ");

				result = 2;
			}

			if(result != 0)
			{
				m_pMem->Clear();

				Localization(pPSuLM, &Pose, pPSuLM_);

				if(m_nHypotheses > 0)
				{
					pHypothesis = m_HypothesisArray[0];

					R = pHypothesis->PoseSM.m_Rot;
					t = pHypothesis->PoseSM.m_X;

					RVLINVTRANSF3D(R, t, R_, t_);

					pNeighborRel->pPoseRel->Diff(&PoseMS, dist, angle);
					
					if(dist <= 200.0 && RVLABS(angle) <= 5.0 * DEG2RAD)
					{
						result = 0;

						fprintf(fp, "OK.\n");

						RVL6DOFInvTransfUncert(R, t, pHypothesis->PoseSM.m_C, pNeighborRel->pPoseRel->m_C);
						double *C = pNeighborRel->pPoseRel->m_C;
						RVLCOMPLETESIMMX3(C)
						C += 2 * 9;
						RVLCOMPLETESIMMX3(C)
					}
					else
						fprintf(fp, "ERROR: dist=%lf, angle=%lf (Hypothesis %d)\n", dist, angle * RAD2DEG, pHypothesis->Index);
				}
				else
					fprintf(fp, "ERROR: No hypotheses.\n");

			}
#endif
			// Correction 2

			C = pNeighborRel->pPoseRel->m_C;
			RVLCOMPLETESIMMX3(C)
			C += 18;
			RVLCOMPLETESIMMX3(C)

			/////

			pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);	
		}
	}

	fclose(fp);

	m_Flags = OldFlags;
}

void CRVLPSuLMBuilder::GetProjectionMatrix(double *P)
{
	if(m_Flags & RVLPSULMBUILDER_FLAG_PC)
	{
		double fu, fv, uc, vc;

		m_pPSD->GetOrgPCProjectionParams(fu, fv, uc, vc);

		P[0] = fu;
		P[1] = 0.0;
		P[2] = uc;
		P[3] = 0.0;
		P[4] = fv;
		P[5] = vc;
		P[6] = 0.0;
		P[7] = 0.0;
		P[8] = 1.0;
	}
	else
		m_pStereoVision->GetKinectProjectionMatrix(P);
}

void CRVLPSuLMBuilder::GetNeighborPSuLMs(CRVLPSuLM *pMPSuLM,
										 CRVL3DPose **PoseM_M)
{
	RVLPSULM_NEIGHBOR2 *pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pMPSuLM->m_LocalMap.pFirst);

	while(pNeighbor)
	{
		pNeighbor->pMPSuLM->m_Flags |= RVLPSULM_FLAG_CLOSE;

		PoseM_M[pNeighbor->pMPSuLM->m_Index] = &(pNeighbor->PoseRel);

		pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
	}		
}

// This function requires GetNeighborPSuLMs() to prepare data before it is called
// and ResetCloseFlags() to reset RVLPSULM_FLAG_CLOSE after the function is completed.

bool CRVLPSuLMBuilder::GetRelativePose(	CRVLPSuLM *pMPSuLM,
										RVLPSULM_HYPOTHESIS *pHypothesis_,
										CRVL3DPose **PoseM_M,
										CRVL3DPose *pPoseS_M,
										bool bOrientation)
{
	if(pHypothesis_->pMPSuLM != pMPSuLM && !(pHypothesis_->pMPSuLM->m_Flags & RVLPSULM_FLAG_CLOSE))
		return false;

	double *RS_M = pPoseS_M->m_Rot;
	double *tS_M = pPoseS_M->m_X;

	double *tS_M_ = pHypothesis_->PoseSM.m_X;

	if(pHypothesis_->pMPSuLM == pMPSuLM)
	{		
		if(bOrientation)
		{
			double *RS_M_ = pHypothesis_->PoseSM.m_Rot;
			RVLCOPYMX3X3(RS_M_, RS_M)
		}
	
		RVLCOPY3VECTOR(tS_M_, tS_M)
	}
	else
	{
		CRVL3DPose *pPoseM_M = PoseM_M[pHypothesis_->pMPSuLM->m_Index];

		double *RM_M = pPoseM_M->m_Rot;
		double *tM_M = pPoseM_M->m_X;

		if(bOrientation)
		{
			double *RS_M_ = pHypothesis_->PoseSM.m_Rot;

			RVLCOMPTRANSF3D(RM_M, tM_M, RS_M_, tS_M_, RS_M, tS_M)			
		}
		else
			RVLTRANSF3(tS_M_, RM_M, tM_M, tS_M)
	}		

	return true;
}

void CRVLPSuLMBuilder::ResetCloseFlags(CRVLPSuLM *pMPSuLM)
{
	RVLPSULM_NEIGHBOR2 *pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pMPSuLM->m_LocalMap.pFirst);

	while(pNeighbor)
	{
		pNeighbor->pMPSuLM->m_Flags &= ~RVLPSULM_FLAG_CLOSE;

		pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
	}
}

void CRVLPSuLMBuilder::SceneFusion()
{	
	double LogLikelihoodHiThr = 100.0;
	double LogLikelihoodLoThr = 30.0;

	DWORD SceneFusionMethod = (m_Flags2 & RVLPSULMBUILDER_FLAG2_SCENE_FUSION);

	int iSample = RVLGetFileNumber(m_ImageFileName, "00000-LW.bmp");

#ifdef RVLPSULMBUILDER_SCENE_FUSION_DEBUG_LOG
	FILE *fp = fopen("C:\\RVL\\Debug\\SceneFusion.log", "w");
#endif

	RVLPSULM_HYPOTHESIS **HypothesisArray = new RVLPSULM_HYPOTHESIS *[m_nRepresentativeHypotheses];

	RVLPSULM_HYPOTHESIS **ppHypothesis = HypothesisArray;

	RVLPSULM_HYPOTHESIS_SCENE_FUSION *pHypothesis;
	RVLPSULM_HYPOTHESIS *pHypothesis_;
	CRVLPSuLM *pMPSuLM;
	//CRVLPSuLM *pMPSuLM_;

	RVLQLIST_PTR_ENTRY *pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(m_RepresentativeHypothesisList.pFirst);

	while(pHypothesisPtr)
	{
		pHypothesis_ = (RVLPSULM_HYPOTHESIS *)(pHypothesisPtr->Ptr);

		pMPSuLM = pHypothesis_->pMPSuLM;

		if(pHypothesis_ == pMPSuLM->m_pHypothesis && pMPSuLM->m_PosteriorProbabilityLocal5DOF >= LogLikelihoodLoThr)
			*(ppHypothesis++) = pHypothesis_;

		pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);
	}

	int nHypotheses = ppHypothesis - HypothesisArray;

	CRVL3DPose **PoseM_M = new CRVL3DPose *[m_maxPSuLMIndex + 1];

	RVLPSULM_HYPOTHESIS_SCENE_FUSION **PSuLMSceneFusionHypothesis = new RVLPSULM_HYPOTHESIS_SCENE_FUSION *[m_maxPSuLMIndex + 1];

	memset(PSuLMSceneFusionHypothesis, 0, (m_maxPSuLMIndex + 1) * sizeof(RVLPSULM_HYPOTHESIS_SCENE_FUSION *));

	bool bTracking = false;

	pHypothesis = (RVLPSULM_HYPOTHESIS_SCENE_FUSION *)(m_SceneFusion.m_HypothesisList.pFirst);

	CRVL3DPose PoseS_M;
	double *tS_M = PoseS_M.m_X;
	
	int i;
	double *RSM, *tSM;
	double *RS_M_, *tS_M_;
	//double *RM_M, *tM_M;
	//RVLPSULM_NEIGHBOR2 *pNeighbor;
	//CRVL3DPose *pPoseM_M;
	double dtSM[3];
	RVLPSULM_HYPOTHESIS *pHypothesis__;
	bool bClose;
	bool bUpdate;
	RVLPSULM_HYPOTHESIS_SCENE_FUSION *pMergedHypothesis;	
	double r2, r2Close;

	while(pHypothesis)
	{
		r2Close = (SceneFusionMethod == RVLPSULMBUILDER_FLAG2_SCENE_FUSION_LOOK_AROUND ? m_SceneFusion.m_rLookAround : pHypothesis->r);
		r2Close *= r2Close;

		RSM = pHypothesis->PoseSM.m_Rot;
		tSM = pHypothesis->PoseSM.m_X;

		pMPSuLM = pHypothesis->pMPSuLM;

#ifdef RVLPSULMBUILDER_SCENE_FUSION_DEBUG_LOG
		fprintf(fp, "%d:\n", pMPSuLM->m_Index);
		fprintf(fp, "==========\n", pMPSuLM->m_Index);
#endif

		GetNeighborPSuLMs(pMPSuLM, PoseM_M);

#ifdef NEVER
		pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pMPSuLM->m_LocalMap.pFirst);
	
		while(pNeighbor)
		{
			pNeighbor->pMPSuLM->m_Flags |= RVLPSULM_FLAG_CLOSE;

			PoseM_M[pNeighbor->pMPSuLM->m_Index] = &(pNeighbor->PoseRel);

			pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
		}	
#endif

		pHypothesis__ = NULL;

		for(i = 0; i < nHypotheses; i++)
		{
			pHypothesis_ = HypothesisArray[i];

			bClose = GetRelativePose(pHypothesis->pMPSuLM, pHypothesis_, PoseM_M, &PoseS_M, false);

#ifdef NEVER
			if(pHypothesis_->pMPSuLM == pMPSuLM)
			{
				tS_M_ = pHypothesis_->PoseSM.m_X;

				RVLCOPY3VECTOR(tS_M_, tS_M)

				bClose = true;
			}
			else if(pHypothesis_->pMPSuLM->m_Flags & RVLPSULM_FLAG_CLOSE)
			{
				//RS_M_ = pHypothesis_->PoseSM.m_Rot;
				tS_M_ = pHypothesis_->PoseSM.m_X;

				pPoseM_M = PoseM_M[pHypothesis_->pMPSuLM->m_Index];

				RM_M = pPoseM_M->m_Rot;
				tM_M = pPoseM_M->m_X;

				RVLTRANSF3(tS_M_, RM_M, tM_M, tS_M)

				bClose = true;
			}
			else
				bClose = false;
#endif

			if(bClose)
			{
				RVLDIF3VECTORS(tS_M, tSM, dtSM)

				r2 = RVLDOTPRODUCT3(dtSM, dtSM);

#ifdef RVLPSULMBUILDER_SCENE_FUSION_DEBUG_LOG
				fprintf(fp, "%d\t%lf\n", pHypothesis_->pMPSuLM->m_Index, sqrt(r2));
#endif

				if(r2 <= r2Close)
				{
					if(pHypothesis__)
					{
						if(pHypothesis_->pMPSuLM->m_PosteriorProbabilityLocal5DOF > pHypothesis__->pMPSuLM->m_PosteriorProbabilityLocal5DOF)
							pHypothesis__ = pHypothesis_;
					}
					else
						pHypothesis__ = pHypothesis_;

					pHypothesis_->Flags |= RVLPSULM_HYPOTHESIS_FLAG_MERGED;
				}
			}			
		}	// for all hypotheses in HypothesisArray (latest scene)

#ifdef RVLPSULMBUILDER_SCENE_FUSION_DEBUG_LOG
		fprintf(fp, "\n");
#endif

		if(pHypothesis__)
		{
			if(bUpdate = (pHypothesis__->pMPSuLM->m_PosteriorProbabilityLocal5DOF > pHypothesis->LogLikelihoodRef
				|| SceneFusionMethod == RVLPSULMBUILDER_FLAG2_SCENE_FUSION_MOVE))
				pHypothesis->pMPSuLM = pHypothesis__->pMPSuLM;

			pHypothesis->LogLikelihood += pHypothesis__->pMPSuLM->m_PosteriorProbabilityLocal5DOF;

			if(pHypothesis->LogLikelihood > LogLikelihoodHiThr)
				bTracking = true;

			pMergedHypothesis = PSuLMSceneFusionHypothesis[pHypothesis->pMPSuLM->m_Index];

			if(pMergedHypothesis)
			{
				if(pHypothesis->LogLikelihood > pMergedHypothesis->LogLikelihood)
				{
					pMergedHypothesis->cost = 0;

					PSuLMSceneFusionHypothesis[pHypothesis->pMPSuLM->m_Index] = pHypothesis;
					
					pHypothesis->cost = pHypothesis->LogLikelihood;
				}
				else
				{
					pHypothesis->cost = 0;

					bUpdate = false;
				}
			}
			else
			{
				PSuLMSceneFusionHypothesis[pHypothesis->pMPSuLM->m_Index] = pHypothesis;
				
				pHypothesis->cost = pHypothesis->LogLikelihood;
			}

			if(bUpdate)
			{
				RS_M_ = pHypothesis__->PoseSM.m_Rot;
				tS_M_ = pHypothesis__->PoseSM.m_X;
				pHypothesis->iSample = iSample;
				pHypothesis->iHypothesis = pHypothesis__->Index;
				RVLCOPYMX3X3(RS_M_, RSM)
				RVLCOPY3VECTOR(tS_M_, tSM)
			}

			pHypothesis->r = m_SceneFusion.m_rMove;
		}
		else
		{
			pHypothesis->r += m_SceneFusion.m_rMove;

			if(pHypothesis->r > 3.0 * m_SceneFusion.m_rMove)
				pHypothesis->cost = 0;
		}

		ResetCloseFlags(pMPSuLM);

		pHypothesis = (RVLPSULM_HYPOTHESIS_SCENE_FUSION *)(pHypothesis->pNext);
	}	// for all hypotheses in m_SceneFusion.m_HypothesisList

	RVLQLIST *pHypothesisList = &(m_SceneFusion.m_HypothesisList);
	CRVLMem *pMem = &(m_SceneFusion.m_Mem);

	void **ppHypothesisPtr = &(pHypothesisList->pFirst);

	pHypothesis = (RVLPSULM_HYPOTHESIS_SCENE_FUSION *)(pHypothesisList->pFirst);

	while(pHypothesis)
	{
		if(pHypothesis->cost == 0)
		{
			RVLQLIST_REMOVE_ENTRY(pHypothesisList, pHypothesis, ppHypothesisPtr)

			m_SceneFusion.m_nHypotheses--;
		}
		else
			ppHypothesisPtr = &(pHypothesis->pNext);

		pHypothesis = (RVLPSULM_HYPOTHESIS_SCENE_FUSION *)(pHypothesis->pNext);
	}	// for all hypotheses in m_SceneFusion.m_HypothesisList

	for(i = 0; i < nHypotheses; i++)
	{
		pHypothesis_ = HypothesisArray[i];

		if(pHypothesis_->Flags & RVLPSULM_HYPOTHESIS_FLAG_MERGED)
			continue;

		pHypothesis_->Flags &= ~RVLPSULM_HYPOTHESIS_FLAG_MERGED;

		RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_HYPOTHESIS_SCENE_FUSION, pHypothesis)

		RVLQLIST_ADD_ENTRY(pHypothesisList, pHypothesis)

		pHypothesis->pMPSuLM = pHypothesis_->pMPSuLM;
		RSM = pHypothesis->PoseSM.m_Rot;
		tSM = pHypothesis->PoseSM.m_X;
		RS_M_ = pHypothesis_->PoseSM.m_Rot;
		tS_M_ = pHypothesis_->PoseSM.m_X;
		RVLCOPYMX3X3(RS_M_, RSM)
		RVLCOPY3VECTOR(tS_M_, tSM)
		pHypothesis->cost = pHypothesis->LogLikelihoodRef = pHypothesis->LogLikelihood = pHypothesis_->pMPSuLM->m_PosteriorProbabilityLocal5DOF;
		pHypothesis->iSample = iSample;
		pHypothesis->iHypothesis = pHypothesis_->Index;
		pHypothesis->r = m_SceneFusion.m_rMove;

		if(pHypothesis->LogLikelihood > LogLikelihoodHiThr)
			bTracking = true;

		m_SceneFusion.m_nHypotheses++;
	}

	delete[] HypothesisArray;
	delete[] PoseM_M;
	delete[] PSuLMSceneFusionHypothesis;

	if(m_SceneFusion.m_HypothesisArray)
		delete[] m_SceneFusion.m_HypothesisArray;

	m_SceneFusion.m_HypothesisArray = NULL;	

	RVLBubbleSort2<RVLPSULM_HYPOTHESIS_SCENE_FUSION>(pHypothesisList, m_SceneFusion.m_nHypotheses, &(m_SceneFusion.m_HypothesisArray), true);

#ifdef RVLPSULMBUILDER_SCENE_FUSION_DEBUG_LOG
	fclose(fp);

	fp = fopen("C:\\RVL\\Debug\\SceneFusionHypotheses.txt", "w");

	for(i = 0; i < m_SceneFusion.m_nHypotheses; i++)
	{
		pHypothesis = m_SceneFusion.m_HypothesisArray[i];

		fprintf(fp, "%d\t%d\t%lf\t%lf\n", i, pHypothesis->pMPSuLM->m_Index, pHypothesis->cost, pHypothesis->r);
	}

	fclose(fp);
#endif
}

void CRVLPSuLMBuilder::GetConnectedSubMap(CRVLPSuLM *pPSuLM0)
{
	CRVL3DPose NullPose;
	double C[3 * 3 * 3];
	double *C_;

	RVLNULLMX3X3(C)

	if(!(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF))
	{
		C_ = C + 9;
		RVLNULLMX3X3(C_)
		C_ += 9;
		RVLNULLMX3X3(C_)
	}

	NullPose.Reset();

	NullPose.m_C = C;

	NullPose.m_ParamFlags = (m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF ? RVL3DPOSE_PARAM_FLAGS_COV_3D : RVL3DPOSE_PARAM_FLAGS_COV_6D);

	CRVLMem Mem;

	Mem.Create(m_PSuLMList.m_nElements * sizeof(RVLPTRCHAIN_ELEMENT));

	CRVLMem *pMemOld = m_PSuLMSubList.m_pMem;

	m_PSuLMSubList.m_pMem = &Mem;

	m_PSuLMSubList.RemoveAll();

	GetLocalModels(NULL, &NullPose, pPSuLM0, 1e10);

	FILE *fp = fopen("C:\\RVL\\Debug\\ConnectedMap.log", "w");

	CRVLPSuLM *pPSuLM;

	m_PSuLMSubList.Start();

	while(m_PSuLMSubList.m_pNext)
	{
		pPSuLM = (CRVLPSuLM *)(m_PSuLMSubList.GetNext());

		fprintf(fp, "%d\n", pPSuLM->m_Index);
	}

	RVLResetFlags<CRVLPSuLM>(&m_PSuLMSubList, RVLPSULM_FLAG_CLOSE);

	m_PSuLMSubList.m_pMem = pMemOld;

	fclose(fp);
}

void CRVLPSuLMBuilder::RepresentativeHypotheses()
{
	double r2 = m_RepresentativeHypDistThr * m_RepresentativeHypDistThr;
	double OrientThr = m_RepresentativeHypOrientThr * DEG2RAD;	

	RVLQLIST *pList = &m_RepresentativeHypothesisList;

	RVLQLIST_INIT(pList)

	if(m_RepresentativeHypothesisMem)
		delete[] m_RepresentativeHypothesisMem;

	m_RepresentativeHypothesisMem = new RVLQLIST_PTR_ENTRY[m_HypothesisList.m_nElements];

	RVLQLIST_PTR_ENTRY *pRepresentativeHypothesisPtr = m_RepresentativeHypothesisMem;

	CRVL3DPose **PoseM_M = new CRVL3DPose *[m_maxPSuLMIndex + 1];

	CRVL3DPose dPose;
	double *dR = dPose.m_Rot;
	double *dt = dPose.m_X;

	int i, j;
	RVLPSULM_HYPOTHESIS *pHypothesis, *pHypothesis_;

	for(i = 0; i < m_nHypotheses; i++)
	{
		pHypothesis = m_HypothesisArray[i];

		pHypothesis->iRepresentative = 0xffffffff;

		//if(pHypothesis->Index == 51)
		//	int debug = 0;
	}

	CRVL3DPose PoseS_M;
	double *RSM, *tSM, *RS_M, *tS_M;
	double et, eR;
	double V3Tmp[3];

	for(i = 0; i < m_nHypotheses; i++)
	{
		pHypothesis = m_HypothesisArray[i];

		//if(pHypothesis->Index == 1966 || pHypothesis->Index == 1940)
		//	int debug = 0;

		if(pHypothesis->Probability < m_minRelevantLogLikelihood)
			break;

		if(pHypothesis->iRepresentative != 0xffffffff)
			continue;

		RVLQLIST_ADD_ENTRY(pList, pRepresentativeHypothesisPtr)

		pRepresentativeHypothesisPtr->Ptr = pHypothesis;

		pRepresentativeHypothesisPtr++;

		GetNeighborPSuLMs(pHypothesis->pMPSuLM, PoseM_M);

		RSM = pHypothesis->PoseSM.m_Rot;
		tSM = pHypothesis->PoseSM.m_X;

		for(j = i + 1; j < m_nHypotheses; j++)
		{
			pHypothesis_ = m_HypothesisArray[j];

			//if(pHypothesis->Index == 591 && pHypothesis_->Index == 871)
			//	int debug = 0;

			if ((m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING) && (m_Flags2 & RVLPSULMBUILDER_FLAG2_MAPBUILDING_MANUAL))
				break;

			if(pHypothesis_->Probability < m_minRelevantLogLikelihood)
				break;

			if(!GetRelativePose(pHypothesis->pMPSuLM, pHypothesis_, PoseM_M, &PoseS_M))
				continue;

			RS_M = PoseS_M.m_Rot;
			tS_M = PoseS_M.m_X;
			
			RVLDIF3VECTORS(tS_M, tSM, dt)

			et = RVLDOTPRODUCT3(dt, dt);

			if(et > r2)
				continue;

			RVLMXMUL3X3T1(RSM, RS_M, dR)

			dPose.GetAngleAxis(V3Tmp, eR);

			if(RVLABS(eR) > OrientThr)
				continue;

			pHypothesis_->iRepresentative = pHypothesis->Index;

			//if(pHypothesis_->Index == 51)
			//	int debug = 0;
		}

		ResetCloseFlags(pHypothesis->pMPSuLM);
	}

	m_nRepresentativeHypotheses = pRepresentativeHypothesisPtr - m_RepresentativeHypothesisMem;		

	delete[] PoseM_M;
}

IplImage * CRVLPSuLMBuilder::GetComplexPSuLMRGBImage(char *ImageFileName)
{
	int w = m_pCamera->m_wSpherical;
	int h = m_pCamera->m_hSpherical;

	char *ComplexImageFileName = RVLCreateFileName(ImageFileName, "-LW.bmp", -1, "-C.bmp");

	// Try to load the complex image from file.

	IplImage *pComplexImage = cvLoadImage(ComplexImageFileName);

	if(pComplexImage)
	{
		if(pComplexImage->width == w && pComplexImage->height == h)
			return pComplexImage;
		else
			cvReleaseImage(&pComplexImage);
	}

	// If a complex image file is not available, then create one.

	double fu = m_pStereoVision->m_KinectParams.depthFu;
	double fv = m_pStereoVision->m_KinectParams.depthFv;
	double uc = m_pStereoVision->m_KinectParams.depthUc;
	double vc = m_pStereoVision->m_KinectParams.depthVc;

	pComplexImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	unsigned char *RGB = (unsigned char *)(pComplexImage->imageData);

	int wStep = pComplexImage->widthStep;

	cvSet(pComplexImage, cvScalar(255, 0, 0));

	unsigned char *A = new unsigned char[w * h];

	memset(A, 0, w * h);	

	char *ImageFileName_ = RVLCreateString(ImageFileName);

	int iSample = RVLGetFileNumber(ImageFileName, "00000-LW.bmp");

	CRVL3DPose PoseM_M;

	double *RM_M = PoseM_M.m_Rot;

	//double *tM_M = PoseM_M.m_X;

	//RVLNULL3VECTOR(tM_M)

	unsigned char command;
	int iSample0;
	int u, v;
	IplImage *pImage_;
	unsigned char *pPix, *pPix_, *pPixRow_;
	int u_, v_, w_, h_, wStep_;
	double R[3], R_[3];
	int iPix;
	int A_, A__;
	double U[2];
	int iU[2];
	
	do
	{
		RVLSetFileNumber(ImageFileName_, "00000-LW.bmp", iSample);

		if (!GetOdometry(ImageFileName_, &PoseM_M, "-LW.bmp", iSample0, command))
			break;
	
		pImage_ = cvLoadImage(ImageFileName_);
		//pImage_ = CRVLImageFilter::RVLFilterNHS(pImage_);

		pPixRow_ = (unsigned char *)(pImage_->imageData);

		w_ = pImage_->width;
		h_ = pImage_->height;
		wStep_ = pImage_->widthStep;

		for(v_ = 0; v_ < h_; v_++)
		{
			pPix_ = pPixRow_;

			for(u_ = 0; u_ < w_; u_++)
			{
				R_[0] = ((double)u_ - uc) / fu;
				R_[1] = ((double)v_ - vc) / fv;
				R_[2] = 1.0;

				RVLMULMX3X3VECT(RM_M, R_, R)

				m_pCamera->Project3DPointToSphere(R, U, iU);

				u = (iU[0] >> 1);
				v = (iU[1] >> 1);

				if(u < 0)
					continue;

				if(u >= w)
					continue;

				if(v < 0)
					continue;

				if(v >= h)
					continue;			

				iPix = u + v * w;

				pPix = RGB + 3 * u + v * wStep;

				A_ = (int)(A[iPix]);
				A__ = A_ + 1;

				*pPix = (unsigned char)((A_ * (int)(*pPix) + (int)(*(pPix_++))) / A__);
				pPix++;
				*pPix = (unsigned char)((A_ * (int)(*pPix) + (int)(*(pPix_++))) / A__);
				pPix++;
				*pPix = (unsigned char)((A_ * (int)(*pPix) + (int)(*(pPix_++))) / A__);

				A[iPix] = (unsigned char)A__;
			}

			pPixRow_ += wStep_;
		}

		iSample++;
	}while(command != 'C');

	delete[] ImageFileName_;
	delete[] A;

	cvSaveImage(ComplexImageFileName, pComplexImage);

	delete[] ComplexImageFileName;

	cvReleaseImage(&pImage_);

	return pComplexImage;
}

void CRVLPSuLMBuilder::MergeSurfaces(CRVLPSuLM *pPSuLM)
{
	double kr = 2.0;
	double varq = 5.0 * DEG2RAD;
	varq *= varq;

	CRVL3DSurface2 **SurfaceArray = pPSuLM->m_3DSurfaceArray;

	CRVL3DSurface2 **pp3DSurface = SurfaceArray;

	CRVLMPtrChain *p3DSurfaceList = &(m_S3DSurfaceSet.m_ObjectList);

	CRVL3DSurface2 *p3DSurface;

	p3DSurfaceList->Start();

	while(p3DSurfaceList->m_pNext)
	{
		p3DSurface = (CRVL3DSurface2 *)(p3DSurfaceList->GetNext());

		if(!(p3DSurface->m_Flags & RVL3DSURFACE_FLAG_CLOSE))
			continue;

		p3DSurface->m_Index = pp3DSurface - SurfaceArray;

		*(pp3DSurface++) = p3DSurface;
	}

	//return;

	int n3DSurfaces = pp3DSurface - SurfaceArray;

	CRVL3DSurface2 **p3DSurfaceArrayEnd = pp3DSurface;

	double RCM[9];
	double *XMC = RCM;
	double *YMC = RCM + 3;
	double *ZMC = RCM + 6;

	CRVL3DSurface2 **pp3DSurface_;
	CRVL3DSurface2 *p3DSurface_;
	CRVL3DSurface2 *p3DSurfaceTmp;
	double *RFC, *tFC, *RF_C, *tF_C, *N, *N_, *u, *u_;
	double dtC[3], A[9], b[3], b_[3], eig[3], RFM[9], RF_M[9], XF__C[3], YF__C[3];
	double d2, r1, r1_, r2, r2_, d2Thr, eN, eP, n, n_, nm, fTmp;
	double U[2][3], U_[2][3];
	double PC[8][3], PM[8][2];
	int i, j, k;
	double *P, *P_;
	double minx, maxx, miny, maxy, minz, maxz, x, y, z, z_, varx, vary;
	double dtM[2], C[4], C_[4], Q[4], c[2];
	RVL2DMOMENTS Moments;
	RVLQLIST *pSamples, *pSamples_;
	RVLQLIST *pBoundaryContourList, *pBoundaryContourList_;

	for(pp3DSurface = SurfaceArray; pp3DSurface < p3DSurfaceArrayEnd; pp3DSurface++)
	{
		p3DSurface = *pp3DSurface;		

		if(p3DSurface->m_nSupport == 0)
			continue;

		for(pp3DSurface_ = pp3DSurface + 1; pp3DSurface_ < p3DSurfaceArrayEnd; pp3DSurface_++)
		{
			p3DSurface = *pp3DSurface;

			p3DSurface_ = *pp3DSurface_;

			if(p3DSurface_->m_nSupport == 0)
				continue;

			if(p3DSurface_->m_nSupport > p3DSurface->m_nSupport)
			{
				p3DSurfaceTmp = p3DSurface;
				p3DSurface = p3DSurface_;
				p3DSurface_ = p3DSurfaceTmp;
			}

			tFC = p3DSurface->m_Pose.m_X;
			tF_C = p3DSurface_->m_Pose.m_X;

			RVLDIF3VECTORS(tFC, tF_C, dtC)

			d2 = RVLDOTPRODUCT3(dtC, dtC);

			r1 = kr * p3DSurface->m_EigenValues[0];
			r1_ = kr * p3DSurface_->m_EigenValues[0];

			d2Thr = 2.0 * (r1 + r1_);
			d2Thr *= d2Thr;

			if(d2 > d2Thr)
				continue;

			N = p3DSurface->m_N;
			N_ = p3DSurface_->m_N;

			eN = RVLDOTPRODUCT3(N, N_);

			if(eN < 0.99619469809174553229501040247389)	// cos(5 deg)
			//if(eN < 0.985)
				continue;

			n = (double)(p3DSurface->m_nSupport);
			n_ = (double)(p3DSurface_->m_nSupport);
			
			nm = n + n_;

			// ZMC <- UNIT((n * N + n_ * N_) / (n + n_))

			RVLSCALE3VECTOR(N, n, b)
			RVLSCALE3VECTOR(N_, n_, b_)
			RVLSUM3VECTORS(b, b_, ZMC)
			RVLSCALE3VECTOR2(ZMC, nm, ZMC)
			RVLNORM3(ZMC, fTmp)

			RFC = p3DSurface->m_Pose.m_Rot;
			RF_C = p3DSurface_->m_Pose.m_Rot;

			// PC[i] <- ellipse 'vertices'

			r2 = kr * p3DSurface->m_EigenValues[1];
			r2_ = kr * p3DSurface_->m_EigenValues[1];

			u = U[0];
			RVLCOPYCOLMX3X3(RFC, 0, u)
			RVLSCALE3VECTOR(u, r1, u)
			u = U[1];
			RVLCOPYCOLMX3X3(RFC, 1, u)
			RVLSCALE3VECTOR(u, r2, u)
			u_ = U_[0];
			RVLCOPYCOLMX3X3(RF_C, 0, u_)
			RVLSCALE3VECTOR(u_, r1_, u_)
			u_ = U_[1];
			RVLCOPYCOLMX3X3(RF_C, 1, u_)
			RVLSCALE3VECTOR(u_, r2_, u_)

			for(i = 0; i < 4; i++)
			{
				P = PC[i];
				P_ = PC[i + 4];

				u = U[i & 1];
				u_ = U_[i & 1];

				if(i & 2)
				{
					RVLSUM3VECTORS(tFC, u, P)
					RVLSUM3VECTORS(tF_C, u_, P_)
				}
				else
				{
					RVLDIF3VECTORS(tFC, u, P)
					RVLDIF3VECTORS(tF_C, u_, P_)
				}
			}

			/////

			P = PC[0];

			minz = maxz = RVLDOTPRODUCT3(P, ZMC);

			for(i = 1; i < 8; i++)
			{
				P = PC[i];

				z = RVLDOTPRODUCT3(P, ZMC);

				if(z < minz)
					minz = z;
				else if(z > maxz)
					maxz = z;
			}

			//if(maxz - minz > 2.0 * (sqrt(p3DSurface->m_sigmaR) + sqrt(p3DSurface_->m_sigmaR)) + 30.0)
			//	continue;

			eP = RVLDOTPRODUCT3(dtC, ZMC);

			if(eP < 0.0)
				eP = -eP;

			if(eP > 2.0 * (sqrt(p3DSurface->m_sigmaR) + sqrt(p3DSurface_->m_sigmaR)) + 30.0)
				continue;

			// XMC <- unit vector orthogonal to ZMC

			//RVLORTHOGONAL3(ZMC, XMC, i, j, k, b, fTmp)
			RVLORTHOGONAL3(ZMC, XMC, i, j, k, fTmp)

			// YMC <- ZMC x XMC

			RVLCROSSPRODUCT3(ZMC, XMC, YMC)

			// dtM <- [1 0 0] * RCM * dtC
			//        [0 1 0]

			dtM[0] = RVLDOTPRODUCT3(XMC, dtC);
			dtM[1] = RVLDOTPRODUCT3(YMC, dtC);

			// C <- upper-left 2x2 block of RFM * [r1^2 0    0] * RFM'
			//                                    [0    r2^2 0]
			//                                    [0    0    0]

			RVLMXMUL3X3(RCM, RFC, RFM)
			RVLMXMUL3X3(RCM, RF_C, RF_M)

			varx = r1 * r1;
			vary = r2 * r2;

			RVLSCALECOL3(RFM, 0, varx, A)
			RVLSCALECOL3(RFM, 1, vary, A)

			C[0] = A[0] * RFM[0] + A[1] * RFM[1];
			C[1] = A[0] * RFM[3] + A[1] * RFM[4];
			C[3] = A[3] * RFM[3] + A[4] * RFM[4];

			// C_ <- upper-left 2x2 block of RF_M * [r1_^2 0    0] * RF_M'
			//                                      [0    r2_^2 0]
			//                                      [0    0     0]

			varx = r1_ * r1_;
			vary = r2_ * r2_;

			RVLSCALECOL3(RF_M, 0, varx, A)
			RVLSCALECOL3(RF_M, 1, vary, A)

			C_[0] = A[0] * RF_M[0] + A[1] * RF_M[1];
			C_[1] = A[0] * RF_M[3] + A[1] * RF_M[4];
			C_[3] = A[3] * RF_M[3] + A[4] * RF_M[4];

			Q[0] = C[0] + C_[0];
			Q[1] = C[1] + C_[1];
			Q[3] = C[3] + C_[3];

			fTmp = RVLDET2(Q);
			eP = RVLMAHDIST2(dtM, Q, fTmp);

			if(eP > 2.4079)	// Chi squared test 70% for 2 degrees of freedom
				continue;

			//if (p3DSurface->m_Index == 29)
			//{
			//	int debug = 0;

			//	RVL3DSURFACE_SAMPLE *pSample = (RVL3DSURFACE_SAMPLE *)(p3DSurface->m_Samples.pFirst);

			//	while (pSample)
			//	{
			//		pSample->Index = (debug++);

			//		pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
			//	}

			//	debug = 0;

			//	pSample = (RVL3DSURFACE_SAMPLE *)(p3DSurface_->m_Samples.pFirst);

			//	while (pSample)
			//	{
			//		pSample->Index = (debug++);

			//		pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
			//	}
			//}
				
			// PM[i] <- [XMC' * PC[i]; YMC' * PC[i]]
			// compute mean and covariance of points PM

			Moments.S[0] = Moments.S[1] = Moments.S2[0] = Moments.S2[1] = Moments.S2[3] = 0.0;

			for(i = 0; i < 8; i++)
			{
				P = PC[i];	

				x = PM[i][0] = RVLDOTPRODUCT3(XMC, P);
				y = PM[i][1] = RVLDOTPRODUCT3(YMC, P);				

				Moments.S[0] += x;
				Moments.S[1] += y;
				Moments.S2[0] += (x * x);
				Moments.S2[1] += (x * y);
				Moments.S2[3] += (y * y);
			}
	
			Moments.n = 8;

			RVLGetCovMatrix2(&Moments, Q, c);

			// compute the bounding box of points PM

			RVLGetMaxEigVector2(Q, eig, b);

			P = PM[0];

			minx = maxx =  P[0] * b[0] + P[1] * b[1]; 
			miny = maxy = -P[0] * b[1] + P[1] * b[0]; 

			for(i = 1; i < 8; i++)
			{
				P = PM[i];

				x =  P[0] * b[0] + P[1] * b[1];
				y = -P[0] * b[1] + P[1] * b[0];

				if(x < minx)
					minx = x;
				else if(x > maxx)
					maxx = x;

				if(y < miny)
					miny = y;
				else if(y > maxy)
					maxy = y;
			}

			// p3DSurface <- merge p3DSurface and p3DSurface_

			//if(p3DSurface->m_Index == 95)
			//	int debug = 0;

			r1 = p3DSurface->m_EigenValues[0] = 0.25 * (maxx - minx);
			r2 = p3DSurface->m_EigenValues[1] = 0.25 * (maxy - miny);

			x = b[0];
			y = b[1];

			RVLSCALE3VECTOR(XMC, x, b)
			RVLSCALE3VECTOR(YMC, y, b_)
			RVLSUM3VECTORS(b, b_, XF__C)

			RVLSCALE3VECTOR(XMC, -y, b)
			RVLSCALE3VECTOR(YMC, x, b_)
			RVLSUM3VECTORS(b, b_, YF__C)

			RVLCOPYTOCOL3(XF__C, 0, RFC)
			RVLCOPYTOCOL3(YF__C, 1, RFC)
			RVLCOPYTOCOL3(ZMC, 2, RFC)

			RVLCOPY3VECTOR(ZMC, N)

			z = RVLDOTPRODUCT3(tFC, ZMC);

			z_ = RVLDOTPRODUCT3(tF_C, ZMC);

			p3DSurface->m_d = (n * z + n_ * z_) / nm;

			fTmp = 0.5 * (minx + maxx);

			RVLSCALE3VECTOR(XF__C, fTmp, tFC)

			fTmp = 0.5 * (miny + maxy);

			RVLSCALE3VECTOR(YF__C, fTmp, b)

			RVLSUM3VECTORS(tFC, b, tFC)

			RVLSCALE3VECTOR(N, p3DSurface->m_d, b)

			RVLSUM3VECTORS(tFC, b, tFC)			

			p3DSurface->m_sigmaR = RVLMAX(p3DSurface->m_sigmaR, p3DSurface_->m_sigmaR);

			p3DSurface->m_varq[0] = p3DSurface->m_sigmaR / (r1 * r1 + p3DSurface->m_sigmaR) + varq;
			p3DSurface->m_varq[1] = p3DSurface->m_sigmaR / (r2 * r2 + p3DSurface->m_sigmaR) + varq;
			p3DSurface->m_varq[2] = p3DSurface->m_sigmaR;			

			p3DSurface->m_nSupport += p3DSurface_->m_nSupport;

			if (m_Flags2 & RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING)
			{
				pSamples = &(p3DSurface->m_Samples);
				pSamples_ = &(p3DSurface_->m_Samples);

				if (pSamples->pFirst == NULL)
				{
					if (pSamples_->pFirst != NULL)
						pSamples = pSamples_;
				}
				else if (pSamples_->pFirst != NULL)
					RVLQLIST_APPEND(pSamples, pSamples_);
			}

			if (m_Flags2 & RVLPSULMBUILDER_FLAG2_SURFACE_BOUNDARY)
			{
				pBoundaryContourList = &(p3DSurface->m_BoundaryContourList);
				pBoundaryContourList_ = &(p3DSurface_->m_BoundaryContourList);

				RVLQLIST_APPEND(pBoundaryContourList, pBoundaryContourList_);
			}

			// p3DSurface_ <- empty set

			p3DSurface_->m_nSupport = 0;

			/////

			if((*pp3DSurface)->m_nSupport == 0)
				break;
		}
	}	// for each surface

	int iSample = 0;

	RVL3DSURFACE_SAMPLE *pSample;

	for (pp3DSurface = SurfaceArray; pp3DSurface < p3DSurfaceArrayEnd; pp3DSurface++)
	{
		p3DSurface = *pp3DSurface;

		if (p3DSurface->m_nSupport == 0)
			continue;

		pSample = (RVL3DSURFACE_SAMPLE *)(p3DSurface->m_Samples.pFirst);

		while (pSample)
		{
			pSample->pSurface = p3DSurface;

			pSample->Index = (iSample++);

			pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		}
	}
}

void CRVLPSuLMBuilder::MergeLines(CRVLPSuLM *pPSuLM)
{
	CRVL3DLine2 **LineArray = pPSuLM->m_3DLineArray;

	CRVL3DLine2 **pp3DLine = LineArray;

	CRVLMPtrChain *p3DLineList = &(m_S3DLineSet.m_ObjectList);

	CRVL3DLine2 *p3DLine;

	p3DLineList->Start();

	while(p3DLineList->m_pNext)
	{
		p3DLine = (CRVL3DLine2 *)(p3DLineList->GetNext());

		p3DLine->m_Index = pp3DLine - LineArray;

		*(pp3DLine++) = p3DLine;
	}

	int n3DLines = pp3DLine - LineArray;

	CRVL3DLine2 **p3DLineArrayEnd = pp3DLine;

	double RCL[3*3];
	//double tCL[3];
	//double tLC[3];

	double *XLC = RCL;
	double *YLC = RCL + 3;
	double *ZLC = RCL + 6;

	CRVL3DLine2 **pp3DLine_;
	CRVL3DLine2 *p3DLine_;
	CRVL3DLine2 *p3DLineTmp;
	RVL3DLINE_EXTENDED_DATA *pData, *pData_;
	double *V, *V_, *P1C, *P2C, *P1C_, *P2C_, *dP, *dP_, *dX;
	double w1, w2, w1_, w2_, w1o, w2o, dwo, w1m, w2m, dwm, rOverlap, rOverlap_, dx, dy, fTmp, eN;
	//double absZ[3];
	double Pc[2];
	int i, j, k;
	double PM[8][2];

	for(pp3DLine = LineArray; pp3DLine < p3DLineArrayEnd; pp3DLine++)
	{
		p3DLine = *pp3DLine;		

		if(p3DLine->m_nSupport == 0)
			continue;

		for(pp3DLine_ = pp3DLine + 1; pp3DLine_ < p3DLineArrayEnd; pp3DLine_++)
		{
			p3DLine = *pp3DLine;

			p3DLine_ = *pp3DLine_;

			if(p3DLine_->m_nSupport == 0)
				continue;

			//if(p3DLine->m_Index == 23 && p3DLine_->m_Index == 42)
			//	int debug = 0;

			if(p3DLine_->m_nSupport > p3DLine->m_nSupport)
			{
				p3DLineTmp = p3DLine;
				p3DLine = p3DLine_;
				p3DLine_ = p3DLineTmp;
			}

			pData = (RVL3DLINE_EXTENDED_DATA *)(p3DLine->m_pData);

			// coarse orientation matching			

			pData_ = (RVL3DLINE_EXTENDED_DATA *)(p3DLine_->m_pData);

			V = pData->V;
			V_ = pData_->V;

			eN = RVLDOTPRODUCT3(V, V_);

			if(eN < 0.99619469809174553229501040247389)	// cos(5 deg)
				continue;

			// endpoints

			P1C = p3DLine->m_X[0];
			P2C = p3DLine->m_X[1];

			P1C_ = p3DLine_->m_X[0];
			P2C_ = p3DLine_->m_X[1];

			// z-axis of the match reference frame

			dP = pData->dX;
			dP_ = pData_->dX;

			RVLSUM3VECTORS(dP, dP_, ZLC)
			RVLNORM3(ZLC, fTmp)

			// overlapping

			w1 = RVLDOTPRODUCT3(P1C, ZLC);
			w2 = RVLDOTPRODUCT3(P2C, ZLC);
			w1_ = RVLDOTPRODUCT3(P1C_, ZLC);
			w2_ = RVLDOTPRODUCT3(P2C_, ZLC);

			if(w1 >= w1_)
			{
				w1o = w1;
				w1m = w1_;
			}
			else
			{
				w1o = w1_;
				w1m = w1;
			}

			if(w2 <= w2_)
			{
				w2o = w2;
				w2m = w2_;
			}
			else
			{
				w2o = w2_;
				w2m = w2;
			}

			dwo = w2o - w1o;

			rOverlap = dwo / pData->len;

			if(rOverlap < 0.0)
				continue;

			rOverlap_ = dwo / pData_->len;

			if(rOverlap_ < 0.0)
				continue;

			//// central point of overlapping segment

			//w0 = 0.5 * (w1o + w2o);

			//// origin of the merge reference frame

			//RVLSCALE3VECTOR(ZLC, w0, tLC)	

			//RVLMULMX3X3VECT(RCL, tLC, tCL)

			//RVLNEGVECT3(tCL, tCL)

			// x and y-axis of the merge reference frame

			RVLCROSSPRODUCT3(V, V_, XLC)

			fTmp = RVLDOTPRODUCT3(XLC, XLC);

			if(fTmp <= APPROX_ZERO)
				//RVLORTHOGONAL3(ZLC, XLC, i, j, k, absZ, fTmp)
				RVLORTHOGONAL3(ZLC, XLC, i, j, k, fTmp)
			else
			{
				fTmp = sqrt(fTmp);

				RVLSCALE3VECTOR2(XLC, fTmp, XLC)
			}

			RVLCROSSPRODUCT3(ZLC, XLC, YLC);

			// transform lines into the merge reference frame

			RVLMULMX3X3VECT(RCL, P1C, PM[0])
			RVLMULMX3X3VECT(RCL, P2C, PM[1])
			RVLMULMX3X3VECT(RCL, P1C_, PM[2])
			RVLMULMX3X3VECT(RCL, P2C_, PM[3])		

			// compute the centroid of the points P1, P2, P1_ and P2_

			Pc[0] = PM[0][0]; 
			Pc[1] = PM[0][1];

			for(i = 1; i < 4; i++)
			{
				Pc[0] += PM[i][0];
				Pc[1] += PM[i][1];
			}

			Pc[0] *= 0.25; 
			Pc[1] *= 0.25;

			// check the distance of all points from the centroid

			for(i = 0; i < 4; i++)
			{
				dx = PM[i][0] - Pc[0];
				dy = PM[i][1] - Pc[1];
				fTmp = dx * dx + dy * dy;
				if(fTmp > 20.0 * 20.0)
					break;
			}

			if(i < 4)
				continue;

			// p3DLine <- merge p3DLine and p3DLine_

			p3DLine->m_X[0][0] = Pc[0] * XLC[0] + Pc[1] * YLC[0] + w1m * ZLC[0];
			p3DLine->m_X[0][1] = Pc[0] * XLC[1] + Pc[1] * YLC[1] + w1m * ZLC[1];
			p3DLine->m_X[0][2] = Pc[0] * XLC[2] + Pc[1] * YLC[2] + w1m * ZLC[2];

			p3DLine->m_X[1][0] = Pc[0] * XLC[0] + Pc[1] * YLC[0] + w2m * ZLC[0];
			p3DLine->m_X[1][1] = Pc[0] * XLC[1] + Pc[1] * YLC[1] + w2m * ZLC[1];
			p3DLine->m_X[1][2] = Pc[0] * XLC[2] + Pc[1] * YLC[2] + w2m * ZLC[2];

			dX = pData->dX;

			RVLDIF3VECTORS(p3DLine->m_X[1], p3DLine->m_X[0], dX)

			fTmp = sqrt(RVLDOTPRODUCT3(dX, dX));
			RVLSCALE3VECTOR2(dX, fTmp, pData->V)
			pData->len = fTmp;

			dwm = w2m - w1m;

			p3DLine->m_nSupport = DOUBLE2INT(dwm / (dwm + dwo) * (double)(p3DLine->m_nSupport + p3DLine_->m_nSupport));

			// p3DSurface_ <- empty set

			p3DLine_->m_nSupport = 0;

			/////

			if((*pp3DLine)->m_nSupport == 0)
				break;
		}
	}	// for each line
}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 

//void RVL2DContourSegment(CvPoint *PtArray,				// transfer to RVL2DContour.cpp
//						 int nPts,
//						 int maxLineSegmentErrNrm,
//						 CvPoint **BreakPtPtrArray,
//						 int &nSegments)
//{
//	CvPoint **ppBreakPt = BreakPtPtrArray;
//	
//	*(ppBreakPt++) = PtArray;
//
//	CvPoint *pLastPt = PtArray + nPts - 1;
//
//	RVL2DContourSegment(PtArray, pLastPt, maxLineSegmentErrNrm, ppBreakPt);
//
//	*(ppBreakPt++) = pLastPt;
//
//	nSegments = ppBreakPt - BreakPtPtrArray - 1;
//}


void RVL2DContourSegment(CvPoint *pPt1,				// transfer to RVL2DContour.cpp
						 CvPoint *pPt2,
						 int maxLineSegmentErrNrm,
						 CvPoint ***pppBreakPt)
{
	int u = pPt1->x;
	int v = pPt1->y;

	int du, dv, d2, maxd2;
	BOOL bSplit;
	CvPoint *pPt3, *pPt4;
	CvPoint **ppBreakPt;

	if(pPt1->x == pPt2->x && pPt1->y == pPt2->y) // closed contour
	{
		maxd2 = 0;

		pPt3 = pPt1;

		while(TRUE)
		{
			pPt3++;

			if(pPt3 == pPt2)
				break;

			du = pPt3->x - u;
			dv = pPt3->y - v;

			d2 = du * du + dv * dv;

			if(d2 > maxd2)
			{
				maxd2 = d2;

				pPt4 = pPt3;
			}
		}

		RVL2DContourSegment(pPt1, pPt4, maxLineSegmentErrNrm, pppBreakPt);

		ppBreakPt = *pppBreakPt;

		*ppBreakPt = pPt4;
	
		*pppBreakPt = ppBreakPt + 1;
		
		RVL2DContourSegment(pPt4, pPt2, maxLineSegmentErrNrm, pppBreakPt);
	}		
	else	// open contour
	{
		int u2 = pPt2->x;
		int v2 = pPt2->y;

		du = u2 - u;
		dv = v2 - v;

		double l = sqrt((double)(du * du + dv * dv));

		int nu = DOUBLE2INT((double)(-1000 * dv) / l);
		int nv = DOUBLE2INT((double)(1000 * du) / l);

		maxd2 = 0;

		pPt3 = pPt1;

		while(TRUE)
		{
			pPt3++;

			if(pPt3 == pPt2)
				break;

			du = pPt3->x - u;
			dv = pPt3->y - v;

			d2 = nu * du + nv * dv;

			if(d2 < 0)
				d2 = -d2;

			if(d2 > maxd2)
			{
				maxd2 = d2;

				pPt4 = pPt3;
			}			
		}

		bSplit = FALSE;

		if(maxd2 > 0)
		{
			if(maxd2 > maxLineSegmentErrNrm)
				bSplit = TRUE;
			else
			{
				pPt3 = pPt1 + 1;

				du = pPt3->x - u;
				dv = pPt3->y - v;

				d2 = nu * du + nv * dv;

				if(d2 == maxd2 || -d2 == maxd2)
				{
					bSplit = TRUE;

					pPt4 = pPt3;
				}
				else
				{
					pPt3 = pPt2 - 1;

					du = pPt3->x - u2;
					dv = pPt3->y - v2;

					d2 = nu * du + nv * dv;

					if(d2 == maxd2 || -d2 == maxd2)
					{
						bSplit = TRUE;

						pPt4 = pPt3;
					}
				}
			}
		}

		if(bSplit)
		{
			RVL2DContourSegment(pPt1, pPt4, maxLineSegmentErrNrm, pppBreakPt);

			ppBreakPt = *pppBreakPt;

			*ppBreakPt = pPt4;
		
			*pppBreakPt = ppBreakPt + 1;

			RVL2DContourSegment(pPt4, pPt2, maxLineSegmentErrNrm, pppBreakPt);
		}
	}		
}

// This function is different from the one used in IROS15.
// The version used in IROS15 performed EKF update starting from the leaf node,
// while this version starts from the root node.
// This version should be more correct since the matches corresponding to the 
// nodes closer to the root are better conditioned.

void RVLPSuLMHypothesisPoseRefinement(CRVL3DPose *pPose,
									  RVLPSULM_HG_NODE *pNode,
									  RVLPSULM_MSMATCH_DATA *MatchList,
									  double *PInit,
									  int maxnIterations)
{
	RVLSURFACE_MATCH_ARRAY MatchData;

	double *e = MatchData.m_e;
	double *C = MatchData.m_C;
	double *Q = MatchData.m_Q;

	double *R = pPose->m_Rot;
	double *t = pPose->m_X;
	double *invt = (double *)(pPose->m_pData);	

	CRVL3DPose PoseOld;
	double *R_ = PoseOld.m_Rot;
	double *t_ = PoseOld.m_X;

	int nNodes = pNode->g;

	RVLPSULM_HG_NODE **NodeBuff = new RVLPSULM_HG_NODE *[nNodes];

	int iNode = nNodes - 1;

	RVLPSULM_HG_NODE *pNode2 = pNode;

	while (pNode2)
	{
		NodeBuff[iNode--] = pNode2;

		pNode2 = pNode2->pParent;
	}

	RVLPSULM_MSMATCH_DATA *pMSMatch;
	CRVL3DSurface2 *pSSurf, *pMSurf;
	int i;
	double MatchQuality;
	double detQ;
	double dist, angle;
	
	for(i = 0; i < maxnIterations; i++)
	{
		memcpy(pPose->m_C, PInit, 3 * 3 * 3 * sizeof(double));

		RVLCOPYMX3X3(R, R_);
		RVLCOPY3VECTOR(t, t_);

		for (iNode = 0; iNode < nNodes; iNode++)
		{
			pNode2 = NodeBuff[iNode];

			pMSMatch = MatchList + pNode2->iMatch;

			pSSurf = (CRVL3DSurface2 *)(pMSMatch->pSData);
			pMSurf = (CRVL3DSurface2 *)(pMSMatch->pMData);

			pSSurf->Match2(pMSurf, pPose, MatchQuality, detQ, &MatchData);

			pPose->PlanarSurfaceEKFUpdate2(C, Q, e);

			RVLMULMX3X3TVECT(R, t, invt);
		}

		pPose->Diff(&PoseOld, dist, angle);

		if(dist <= 100.0 && RVLABS(angle) <= 5.0 * DEG2RAD)
			break;
	}

	delete[] NodeBuff;
}

void RVLPSuLMHypothesisPoseRefinement(RVLPSULM_HYPOTHESIS *pHypothesis,
									  CRVLPSuLM *pSPSuLM,
									  CRVL3DSurface2 **MatchedMSurfArray)
{
	CRVL3DSurface2 **ppMatchedSSurf = (CRVL3DSurface2 **)(pHypothesis->MatchArray);
	CRVL3DSurface2 **pMatchArrayEnd =  ppMatchedSSurf + pHypothesis->nMatches;

	for(; ppMatchedSSurf < pMatchArrayEnd; ppMatchedSSurf++)
		(*ppMatchedSSurf)->m_Flags |= RVLOBJ2_FLAG_MATCHED;

	int iSSurf;
	CRVL3DSurface2 *pS3DSurface, *pM3DSurface;
	RVLSURFACE_MATCH_ARRAY MatchData;

	for(iSSurf = 0; iSSurf < pSPSuLM->m_n3DSurfaces; iSSurf++)
	{
		pS3DSurface = pSPSuLM->m_3DSurfaceArray[iSSurf];

		if(pS3DSurface->m_Flags & RVLOBJ2_FLAG_MATCHED)
			continue;

		pM3DSurface = MatchedMSurfArray[iSSurf];

		if(pM3DSurface)
			RVL3DPlanarSurfaceEKFUpdate(pS3DSurface, pM3DSurface, &(pHypothesis->PoseSM), &(pHypothesis->PoseSM),
				&MatchData);
	}	

	for(ppMatchedSSurf = (CRVL3DSurface2 **)(pHypothesis->MatchArray); 
		ppMatchedSSurf < pMatchArrayEnd; ppMatchedSSurf++)
		(*ppMatchedSSurf)->m_Flags &= ~RVLOBJ2_FLAG_MATCHED;
}

void RVLPSuLMHypothesisGetAbsPose(RVLPSULM_HYPOTHESIS *pHypothesis,
								  CRVL3DPose *pPoseAC,
								  CRVL3DPose *pPoseCs0)
{
	CRVL3DPose *pPoseSM = &(pHypothesis->PoseSM);

	double *RCsCm = pPoseSM->m_Rot;
	double *tCsCm = pPoseSM->m_X;

	double *RAC = pPoseAC->m_Rot;
	double *tAC = pPoseAC->m_X;

	double R[3 * 3];
	double t[3 * 3];
	double tmp3x1[3];

	RVLCOMPTRANSF3DWITHINV(RAC, tAC, RCsCm, tCsCm, R, t, tmp3x1)

	double *RAm0 = pHypothesis->pMPSuLM->m_PoseAbs->m_Rot;
	double *tAm0 = pHypothesis->pMPSuLM->m_PoseAbs->m_X;
	double *RCs0 = pPoseCs0->m_Rot;
	double *tCs0 = pPoseCs0->m_X;

	RVLCombineTransform3D(RAm0, tAm0, R, t, RCs0, tCs0);

	pPoseCs0->UpdatePTRLA();
}

void RVLPSuLMHypothesisGetAbsPose3DOF(RVLPSULM_HYPOTHESIS *pHypothesis,
									  CRVL3DPose *pPoseAC,
									  CRVL3DPose *pPoseAs0)
{
	CRVL3DPose *pPoseSM = &(pHypothesis->PoseSM);
	CRVL3DPose PoseAsAm;

	PoseAsAm.m_X[0] = PoseAsAm.m_X[1] = PoseAsAm.m_X[2] = 0.0;
	PoseAsAm.m_Alpha = PoseAsAm.m_Beta = PoseAsAm.m_Theta = 0.0;

	//Convert pPoseSM to 3DOF
	RVLDeterminePose6DOFTo3DOF(&PoseAsAm, pPoseSM, pPoseAC);

	double *RAm0 = pHypothesis->pMPSuLM->m_PoseAbs->m_Rot;
	double *tAm0 = pHypothesis->pMPSuLM->m_PoseAbs->m_X;
	double *RAsAm = PoseAsAm.m_Rot;
	double *tAsAm = PoseAsAm.m_X;
	double *RAs0 = pPoseAs0->m_Rot;
	double *tAs0 = pPoseAs0->m_X;

	RVLCombineTransform3D(RAm0, tAm0, RAsAm, tAsAm, RAs0, tAs0);

	pPoseAs0->m_X[2] = 0.0;

	double V[3];
	pPoseAs0->GetAngleAxis(V, pPoseAs0->m_Alpha);

	if(V[2] < 0.0)
		pPoseAs0->m_Alpha =-pPoseAs0->m_Alpha;

	pPoseAs0->m_Beta = pPoseAs0->m_Theta = 0.0;
	pPoseAs0->UpdateRotA0();

}

//Determine Robot1->Robot2 pose if Camera1->Camera2 pose is known
// TAAp = TCpAp*TCCp*TAC   (NOTE: TAC = TApCp)
void RVLDeterminePose6DOFTo3DOF(CRVL3DPose *pPoseAAp, CRVL3DPose *pPoseCCp, CRVL3DPose *pPoseAC)
{
	//Get TCA ie inverse transformation of TAC
	double RCpAp[9], tCpAp[3];
	// TCpAp <- inv(TAC)
	InverseTransform3D(RCpAp,tCpAp,pPoseAC->m_Rot,pPoseAC->m_X);


	double RCAp[9], tCAp[3];

	// TCAp <- TCpAp * TCCp
	RVLCombineTransform3D(RCpAp,tCpAp,pPoseCCp->m_Rot,pPoseCCp->m_X,RCAp,tCAp);

	// TAAp <- TCAp * TAC
	RVLCombineTransform3D(RCAp,tCAp,pPoseAC->m_Rot,pPoseAC->m_X,pPoseAAp->m_Rot,pPoseAAp->m_X);

	pPoseAAp->m_X[2] = 0.0;

	double V[3];
	pPoseAAp->GetAngleAxis(V, pPoseAAp->m_Alpha);

	if(V[2] < 0.0)
		pPoseAAp->m_Alpha =-pPoseAAp->m_Alpha;

	pPoseAAp->m_Beta = pPoseAAp->m_Theta = 0.0;
	pPoseAAp->UpdateRotA0();

	//pPoseAAp->m_Beta = pPoseAAp->m_Theta = 0.0;
	//pPoseAAp->UpdateRotA0();

}

//http://en.wikipedia.org/wiki/Binomial_coefficient
double BinomialCoefficient(int n, int k)
{
	if(k>n) 
	{
		return 0.0;
	}

	if(k>n/2)
	{	
		k = n-k; //Take advantage of symmetry
	}

	double accum = 1.0;

	for(int i = 1; i <= k; i++)
	{
		accum = accum * (n-k+i) / i;
	}

	return accum;
}

int RansacRoundsNeeded(int maxNoRounds, int m, double logeta0, int n, int In)
{
	int roundsNeeded = maxNoRounds;
	if(maxNoRounds <= 0)
	{
		roundsNeeded = INT_MAX;
	}
	else
	{
		double epsilon_n = In*1.0/(double)n;					// inlier point probability
		if(epsilon_n >= 1)
		{
			roundsNeeded = 1;	
		}
		else
		{
			double logEps = log(1.0 - pow(epsilon_n,m));	//outlier sample probability
			int val = (int)ceil(logeta0 / logEps);
			roundsNeeded = (maxNoRounds > val ? val : maxNoRounds); //min of the 2 values
		}
	}
	
	return roundsNeeded;

}


// Eqn 7 and 8 of Chum
int NonRandomness(int m, int n)
{
	
	double p_i;

	double phi = 0.05;    // we want to make sure that randomness < phi

	double p = 0.0;
	double minIn = n;		//min number of inliers
	
	while(p < phi)
	{
		p_i = pow(PROSAC_BETA,(minIn-m)) * pow((1-PROSAC_BETA),(n - minIn + m)) * BinomialCoefficient(n - m,minIn - m);
		p = p + p_i;
		minIn = minIn - 1;
	}

	if(minIn == n)						//any inlier number will be too random
	{
		minIn = INT_MAX;
	}
	else
	{
		minIn = minIn + 1;   // the minimum solution for i
	}

	return minIn;
}

void RandPerm(int n, int perm[])
{
	int i, j, t;

	for(i=0; i<n; i++)
		perm[i] = i;
	for(i=0; i<n; i++) {
		j = rand()%(n-i)+i;
		t = perm[j];
		perm[j] = perm[i];
		perm[i] = t;
	}
}

//int niter_RANSAC(double p, // probability that at least one of the random samples picked up by RANSAC is free of outliers
//                 double epsilon, // proportion of outliers
//                 int s, // sample size
//                 int Nmax) // upper bound on the number of iterations (-1 means INT_MAX)
//{
//	// compute safely N = ceil(log(1. - p) / log(1. - exp(log(1.-epsilon) * s)))
//    double logarg, logval, N;
//    if (Nmax == -1) {
//        Nmax = INT_MAX;
//    }
//    assert(Nmax >= 1);
//    if (epsilon <= 0.) {
//        return 1;
//    }
//    // logarg = -(1-epsilon)^s
//    logarg = -exp(s*log(1.-epsilon)); // C++/boost version: logarg = -std::pow(1.-epsilon, s);
//    // logval = log1p(logarg)) = log(1-(1-epsilon)^s)
//    logval = log(1.+logarg); // C++/boost version: logval = boost::math::log1p(logarg)
//    N = log(1.-p) / logval;
//    if (logval  < 0. && N < Nmax) {
//        return (int)ceil(N);
//    }
//    return Nmax;
//}



//void CRVLPSuLMBuilder::GetCell(
//	double *X,
//	int & i, 
//	int & j)
//{
//	if (m_Flags2 & RVLPSULMBUILDER_FLAG2_COMPLEX)
//	{
//
//	}
//	else
//	{
//		double fu = m_pStereoVision->m_KinectParams.depthFu;
//		double fv = m_pStereoVision->m_KinectParams.depthFv;
//		double uc = m_pStereoVision->m_KinectParams.depthUc;
//		double vc = m_pStereoVision->m_KinectParams.depthVc;
//
//		i = (int)((fu * XS[0] / XS[2] + uc) / m_CellSize2);
//		j = (int)((fv * XS[1] / XS[2] + vc) / m_CellSize2);
//	}
//}


void CRVLPSuLMBuilder::InitHypothesisEvaluation3(CRVLPSuLM * pPSuLM)
{
	double CellSize = (double)m_CellSize2;

	//int minwS = m_pPSD->m_minSampleSize;
	//int maxwS = minwS + m_pPSD->m_maxSampleSize;
	//int maxiCell = m_nCols2 - 1;
	//int maxjCell = m_nRows2 - 1;

	int nSSurfaces = pPSuLM->m_n3DSurfacesTotal;

	double fu = m_pStereoVision->m_KinectParams.depthFu;
	double fv = m_pStereoVision->m_KinectParams.depthFv;
	//double f = RVLMAX(fu, fv);
	double uc = m_pStereoVision->m_KinectParams.depthUc;
	double vc = m_pStereoVision->m_KinectParams.depthVc;

	double CellSizeNrm;
	double absdpq;
	int nCubeSideCells;

	if (pPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX)
	{		
		CellSizeNrm = 1.0 / (double)m_nCellsPer45deg;
		absdpq = 0.5 * CellSizeNrm;
		nCubeSideCells = m_nRows2 * m_nCols2;
	}
		
	//Reset CellArray2
	memcpy(m_CellArray2, m_EmptyCellArray2, m_nCells2 * sizeof(RVLPSULM_CELL2));

	if (m_CellMem)
		delete[] m_CellMem;

	m_CellMem = new RVLQLIST_PTR_ENTRY[4 * pPSuLM->m_nSurfaceSamples];

	RVLQLIST_PTR_ENTRY *pSamplePtr = m_CellMem;

	//fill CellArray2 with ptrs. to SSurface samples

	int i, j;
	RVL3DSURFACE_SAMPLE *pSampleS;
	CRVL3DSurface2 *pS3DSurface;
	int iCell, jCell;
	RVLPSULM_CELL2 *pCell;
	RVLQLIST *pSampleList;
	double *XS;
	double p0, q0, p, q, dp, dq, fTmp;
	int ip0, iq0, ir0, ip, iq, ir;
	double Q[3];

	for (i = 0; i<nSSurfaces; i++)
	{
		//if (i == 57)
		//	int debug = 0;

		pS3DSurface = pPSuLM->m_3DSurfaceArray[i];

		pSampleS = (RVL3DSURFACE_SAMPLE *)(pS3DSurface->m_Samples.pFirst);

		while (pSampleS)
		{
			//if (RVLABS(pSampleS->X[0] - 1263) < 20 && RVLABS(pSampleS->X[1] + 1036) < 20 && RVLABS(pSampleS->X[2] - 1794) < 20)
			//	int debug = 0;

			pSampleS->Flags = 0x00;

			XS = pSampleS->X;

			if (pPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX)
			{
				ProjectToCube(XS, p0, q0, ip0, iq0, ir0);				

				if (ir0 <= 2)
					Q[ir0] = 1.0;
				else
				{
					Q[ir0 % 3] = -1.0;
					p0 = -p0;
					q0 = -q0;
				}

				dp = dq = absdpq;

				for (j = 0; j < 4; j++)
				{
					Q[ip0] = p0 + dp;
					Q[iq0] = q0 + dq;					

					ProjectToCube(Q, p, q, ip, iq, ir);

					iCell = (int)((p + 1.0) / CellSizeNrm);
					jCell = (int)((q + 1.0) / CellSizeNrm);

					pCell = m_CellArray2 + ir * nCubeSideCells + jCell * m_nCols2 + iCell;

					pSamplePtr->Ptr = pSampleS;

					pSampleList = &(pCell->surfaceList);

					RVLQLIST_ADD_ENTRY(pSampleList, pSamplePtr);

					pSamplePtr++;

					fTmp = dp;
					dp = -dq;
					dq = fTmp;
				}
			}
			else
			{
				// TODO: put sample pointers into the neighboring cells

				iCell = (int)((fu * XS[0] / XS[2] + uc) / CellSize);
				jCell = (int)((fv * XS[1] / XS[2] + vc) / CellSize);

				pCell = m_CellArray2 + jCell * m_nCols2 + iCell;

				pSamplePtr->Ptr = pSampleS;

				pSampleList = &(pCell->surfaceList);

				RVLQLIST_ADD_ENTRY(pSampleList, pSamplePtr);

				pSamplePtr++;
			}

			pSampleS = (RVL3DSURFACE_SAMPLE *)(pSampleS->pNext);
		}
	}
}


void CRVLPSuLMBuilder::ProjectToCube(
	double * P,
	double & p, 
	double & q, 
	int & ip,
	int & iq,
	int & ir)
{
	double absx = RVLABS(P[0]);
	double absy = RVLABS(P[1]);
	double absz = RVLABS(P[2]);

	ir = (absz >= absx ? (absz >= absy ? 2 : 1) : (absx >= absy ? 0 : 1));
	ip = ((ir + 1) % 3);
	iq = ((ip + 1) % 3);

	p = P[ip] / P[ir];
	q = P[iq] / P[ir];

	if (P[ir] < 0)
		ir += 3;
}


void CRVLPSuLMBuilder::CompareHypotheses(
	RVLPSULM_MATCH2 * MatchArray1, 
	int nMatches1, 
	RVLPSULM_MATCH2 * MatchArray2, 
	int nMatches2, 
	char * OutputFileName)
{
	if (MatchArray1 == NULL)
		return;

	if (MatchArray2 == NULL)
		return;

	FILE *fp = fopen(OutputFileName, "w");

	MatchDiff(MatchArray1, nMatches1, MatchArray2, nMatches2, fp);

	fprintf(fp, "\n========================\n\n");

	MatchDiff(MatchArray2, nMatches2, MatchArray1, nMatches1, fp);

	fclose(fp);
}


void CRVLPSuLMBuilder::MatchDiff(
	RVLPSULM_MATCH2 * MatchArray1,
	int nMatches1,
	RVLPSULM_MATCH2 * MatchArray2,
	int nMatches2, 
	FILE *fp)
{
	int i, j;
	RVLPSULM_MATCH2 *pMatch1, *pMatch2;
	bool bDiff;
	char feature, source;
	int iS, iM;

	for (i = 0; i < nMatches1; i++)
	{
		pMatch1 = MatchArray1 + i;

		if (pMatch1->Type & RVLPSULM_MATCH_TYPE_AUTO)
		{
			for (j = 0; j < nMatches2; j++)
			{
				pMatch2 = MatchArray2 + j;

				if (!(pMatch2->Type & RVLPSULM_MATCH_TYPE_AUTO))
					continue;

				if ((pMatch2->Type & RVLPSULM_MATCH_TYPE_LINE) != (pMatch1->Type & RVLPSULM_MATCH_TYPE_LINE))
					continue;

				if (pMatch1->vpSObject != pMatch2->vpSObject)
					continue;

				if (pMatch1->vpMObject != pMatch2->vpMObject)
					continue;

				break;
			}

			if (bDiff = (j == nMatches2))
				source = 'S';
		}
		else
		{
			bDiff = true;

			source = 'M';
		}

		if (pMatch1->Type & RVLPSULM_MATCH_TYPE_LINE)
		{
			feature = 'L';
			iS = ((CRVL3DLine2 *)(pMatch1->vpSObject))->m_Index;
			iM = ((CRVL3DLine2 *)(pMatch1->vpMObject))->m_Index;
		}
		else
		{
			feature = 'S';
			iS = ((CRVL3DSurface2 *)(pMatch1->vpSObject))->m_Index;
			iM = ((CRVL3DSurface2 *)(pMatch1->vpMObject))->m_Index;
		}

		if (bDiff)
		{
			fprintf(fp, "p(S%c%d|%c%c%d)=%lf\n", feature, iS, source, feature, iM, pMatch1->cost);
		}
	}	// for every mach in MatchArray1
}

bool CRVLPSuLMBuilder::Compute5DoFPose(
	RVLPSULM_HG_NODE *pNode,
	RVLPSULM_MSMATCH_DATA *MatchList,
	double *tInit,
	RVLSURFACE_MATCH_ARRAY *pMatchData,
	CRVL3DPose *pPoseSM)
{
	if (pNode->pParent == NULL)
		return false;

	RVLPSULM_MSMATCH_DATA *pMSMatch1 = MatchList + pNode->pParent->iMatch;
	RVLPSULM_MSMATCH_DATA *pMSMatch2 = MatchList + pNode->iMatch;

	CRVL3DSurface2 *pM3DSurface1 = (CRVL3DSurface2 *)(pMSMatch1->pMData);
	CRVL3DSurface2 *pS3DSurface1 = (CRVL3DSurface2 *)(pMSMatch1->pSData);
	CRVL3DSurface2 *pM3DSurface2 = (CRVL3DSurface2 *)(pMSMatch2->pMData);
	CRVL3DSurface2 *pS3DSurface2 = (CRVL3DSurface2 *)(pMSMatch2->pSData);

	double *NS1 = pS3DSurface1->m_N;
	double *NS2 = pS3DSurface2->m_N;

	double csNS = RVLDOTPRODUCT3(NS1, NS2);

	if (RVLABS(csNS) > COS45)
		return false;
	
	double *NM1 = pM3DSurface1->m_N;	
	double *NM2 = pM3DSurface2->m_N;

	double csNM = RVLDOTPRODUCT3(NM1, NM2);

	if (RVLABS(csNM) > COS45)
		return false;
	
	double dS1 = pS3DSurface1->m_d;
	double dS2 = pS3DSurface2->m_d;
	double dM1 = pM3DSurface1->m_d;
	double dM2 = pM3DSurface2->m_d;

	double fTmp;

	double S[9];

	double *US = S;
	double *VS = S + 3;
	double *NS = S + 6;

	RVLCROSSPRODUCT3(NS1, NS2, US);
	RVLNORM3(US, fTmp);

	RVLCROSSPRODUCT3(NS1, US, VS);

	RVLCOPY3VECTOR(NS1, NS);

	double M[9];

	double *UM = M;
	double *VM = M + 3;
	double *NM = M + 6;

	RVLCROSSPRODUCT3(NM1, NM2, UM);
	RVLNORM3(UM, fTmp);

	RVLCROSSPRODUCT3(NM1, UM, VM);

	RVLCOPY3VECTOR(NM1, NM);

	double *RSM = pPoseSM->m_Rot;
	double *tSM = pPoseSM->m_X;

	RVLMXMUL3X3T1(UM, US, RSM);

	double VTmp[3];

	double csS = RVLDOTPRODUCT3(NS1, NS2);

	RVLSCALE3VECTOR(NS1, csS, VTmp);
	RVLDIF3VECTORS(NS2, VTmp, VTmp);
	
	double kS = sqrt(RVLDOTPRODUCT3(VTmp, VTmp));

	double csM = RVLDOTPRODUCT3(NM1, NM2);

	RVLSCALE3VECTOR(NM1, csM, VTmp);
	RVLDIF3VECTORS(NM2, VTmp, VTmp);

	double kM = sqrt(RVLDOTPRODUCT3(VTmp, VTmp));

	double D[3];

	D[0] = RVLDOTPRODUCT3(UM, tInit);
	D[1] = (dS2 - csS * dS1) / kS - (dM2 - csM * dM1) / kM;
	D[2] = dS1 - dM1;

	RVLMULMX3X3TVECT(UM, D, tSM);

	pPoseSM->UpdatePTRLL();

	double *invt = (double *)(pPoseSM->m_pData);
	RVLMULMX3X3TVECT(RSM, tSM, invt);

	RVL3DPlanarSurfaceEKFUpdate(pS3DSurface1, pM3DSurface1, pPoseSM, &(pNode->pParent->PoseSM), pMatchData, false);

	return true;
}