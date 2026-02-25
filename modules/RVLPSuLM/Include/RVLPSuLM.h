class CRVLPSuLM;

#define RVLPSULM_FLAG_VISITED			0x00000001
#define RVLPSULM_FLAG_TARGET			0x00000002
#define RVLPSULM_FLAG_CLOSE				0x00000004
#define RVLPSULM_FLAG_COMPLEX			0x00000008
#define RVLPSULM_FLAG_SURFACE_BOUNDARY	0x00000010

#define RVLPSULM_LANDMARK_TYPE_EDGE		0
#define RVLPSULM_LANDMARK_TYPE_CORNER	1
#define RVLPSULM_DISPLAY_SURFACES		0x00000001
#define RVLPSULM_DISPLAY_CLEAR			0x00000002
#define RVLPSULM_DISPLAY_SCELLS			0x00000004
#define RVLPSULM_DISPLAY_MCELLS			0x00000008
#define RVLPSULM_DISPLAY_CELLS			0x0000000c
#define RVLPSULM_DISPLAY_PROJ_SCELLS	0x00000010
#define RVLPSULM_DISPLAY_PROJ_MCELLS	0x00000020
#define RVLPSULM_DISPLAY_PROJ_CELLS		0x00000030
#define RVLPSULM_DISPLAY_SELECTION		0x00000040
#define RVLPSULM_DISPLAY_VECTORS		0x00000080
#define RVLPSULM_DISPLAY_SAMPLE_REGIONS	0x00000100
#define RVLPSULM_DISPLAY_SAMPLE_TYPES	0x00000200
#define RVLPSULM_DISPLAY_SAMPLE_CENTERS	0x00000400
#define RVLPSULM_DISPLAY_COLOR			0x00000800
#define RVLPSULM_DISPLAY_LINES			0x00001000
#define RVLPSULM_DISPLAY_ELLIPSES		0x00002000
#define RVLPSULM_DISPLAY_SCENE			0x00004000
#define RVLPSULM_DISPLAY_MODEL			0x00008000
#define RVLPSULM_DISPLAY_SAMPLES		0x00010000
#define RVLPSULM_DISPLAY_VALIDATION		0x00020000

#define RVLPSULM_CELL_FLAG_MATCHED		0x01
#define RVLPSULM_CELL_FLAG_ERROR		0x02
#define RVLPSULM_CELL_FLAG_COLOR		0x04
#define RVLPSULM_CELL_FLAG_ACTIVE		0x10

#define RVLPSULM_MSMATCH_DATA_FLAG_LINES	0x01

#define RVLPSULM_HYPOTHESIS_FLAG_BEST_FOR_ITS_PSULM		0x01
#define RVLPSULM_HYPOTHESIS_FLAG_MERGED					0x02

#define RVLPSULM_PARTICLE_FLAG_ASSIGNED		0x01

#define RVLPSULM_SCENE_FUSION_N_SCENES		20

//#define RVLPSULM_CONVEX_SEGMENTS
#define RVLPSULM_LINES

struct RVLPSULM_CELL_LINEPTR
{
	CRVL2DLine2 *pLine;
	RVLPSULM_CELL_LINEPTR *pNext;
};

struct RVLPSULM_CELL
{
	BYTE Flags;
	short disparity;
	RVLCOLOR color;
	RVLPSULM_CELL_LINEPTR *pLinePtr;
};

struct RVLPSULM_CELL2
{
	RVLQLIST surfaceList;
	int nPts;
	double matchQuality;
	BYTE Flags;
};

struct RVLPSULM_LANDMARK
{
	BYTE Type;
	double x, z;
	double C[4];
	double miny, maxy;
	BYTE bDown;
	WORD i3DLine;
	void *pNext;
};

struct RVLPSULM_SURF_MATCH_DATA
{
	double MatchProbability;
	double util;
	//RVLPSULM_MSMATCH_DATA *pMatch;
	CRVL3DSurface2 *p3DSurface;
	BYTE bMatched;
};

struct RVLPSULM_LINE_MATCH_DATA
{
	double XSA[3];
	double XMB[3];
	double varxST;
	double varxMT;
	CRVL3DLine2 *p3DLine;
};

struct RVLPSULM_MSMATCH_DATA
{
	double Probability;
	RVLPSULM_SURF_MATCH_DATA *pMData, *pSData;
	//RVLSURFACE_MATCH_ARRAY EKFData;
	BYTE bInitGeomConstrSattisfied;
	BYTE bFailed;
	int Support;
	BYTE Flags;
};

struct RVLPSULM_SMATCH_DATA
{
	double P;
	double POrientMatch;
	int iMFeature;
	bool b;
};

struct RVLPSULM_DISPLAY_MOUSE_CALLBACK_DATA
{
	CRVLPSuLM *pSPSuLM, *pMPSuLM;
	CRVL3DPose PoseM0, PoseS0;
	CRVL3DSurface2 *pSSurf, *pMSurf;
	BYTE Flags;
	//RVLPSULM_MSMATCH_DATA *MatrixSceneModel;
	BYTE *MatrixSceneModel;
	RVLPSULM_SURF_MATCH_DATA *MSurfMatchData;
	RVLPSULM_SURF_MATCH_DATA *SSurfMatchData;
	int nMSurfs;
};

struct RVLPSULM_NEIGHBOUR
{
	int ModelId;
	CRVLPSuLM *pPSuLM;
	CRVL3DPose *pPoseRel;
};

struct RVLPSULM_NEIGHBOR2
{
	CRVLPSuLM *pMPSuLM;
	CRVL3DPose PoseRel;
	double C[3 * 3];
	void *pNext;
};

struct RVLPSULM_HYPOTHESIS;

struct RVLPSULM_PARTICLE
{
	CRVLPSuLM *pMPSuLM;
	CRVL3DPose PoseSM;
	double P[3 * 3];
	int w;
	//int wacc;
	//double d2;
	BYTE Flags;
	RVLPSULM_HYPOTHESIS *pHypothesis;
};

struct RVLPSULM_PARTICLE2
{
	double cs, sn;
	double tx, ty;
	RVLPSULM_PARTICLE *pParticle;
};

struct RVLPSULM_HYPOTHESIS
{
	DWORD Index;
	CRVLPSuLM *pMPSuLM;
	CRVL3DPose PoseSM;
	double P[3 * 3 * 3];
	CRVL3DPose PoseSM3DOF;
	double P3DOF[3 * 3];
	double invt[3];
	double zTB[3];
	int iS, iM;
	int cost;
	int visible;
	BYTE Flags;
	void *MatchArray;
	int nMatches;
	void *pNext;
	RVLPSULM_PARTICLE *pParticle;
	double Probability;
	DWORD iRepresentative;
	char validation;				// 1 - correct; 0 - ambiguous; -1 - false
};

struct RVLPSULM_HYPOTHESIS_SCENE_FUSION
{
	CRVLPSuLM *pMPSuLM;
	CRVL3DPose PoseSM;
	double LogLikelihood;
	double LogLikelihoodRef;
	void *pNext;
	double cost;
	int iSample;
	int iHypothesis;
	double r;
};

struct RVLPSULM_PATH_PLANNING_NEIGHBOR
{
	CRVLPSuLM *pMPSuLM;
	CRVL3DPose PoseRel;
	double dAlpha;
	double s;
};

struct RVLPSULM_MESH_FILE_GROUP_DATA
{
	DWORD Mask;
	DWORD Value;
	int MaterialID;
};

void RVLPSuLMDisplayMouseCallback(int event, int x, int y, int flags, void* vpFig);
void RVLPSuLMDisplay(CRVLFigure *pFig);
void RVLDisplayColoredDepthMap(CRVL3DMeshObject *pSceneMesh,
							   int w,
							   int h,
							   PIX_ARRAY *pPixArray);
#ifdef RVLVTK
void RVLPSuLMKeyPressCallback(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata);
void RVLPSuLMRightButtonPressCallback(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata);
void RVLPSuLMGenAndDispPSuLMScene(
	CRVLPSuLM *psulm,
	CRVLVTKRenderer* pRenderer,
	std::map<std::string, VTKActorObj*>* vtkobjs,
	bool bRGB = false,
	bool add = false,
	CRVL3DPose * pose = NULL,
	int modelIdx = 0);
void RVLPSuLMAddHypothesisToPSuLMScene(CRVLVTKRenderer* pRenderer, std::map<std::string, VTKActorObj*>* vtkobjs, CRVL3DPose * hypPose, int hypIdx);
#endif

class CRVLPSuLM
{
public:
	CRVLPSuLM(void);
	virtual ~CRVLPSuLM(void);
	void Project(	CRVL3DPose *pPoseC0,
					BOOL bCells = TRUE, 
					int iSelectedPix = -1, 
					CRVL3DSurface2 **ppSelectedSurf = NULL,
					CRVL3DLine2 **ppSelectedLine = NULL,
					int *piFOVExtension = NULL);
	void Project3DSurface(	CRVL3DSurface2 *pSurf, 
							CRVL3DPose *pPoseC0);
	int Match(	CRVL3DSurface2 *pSurface, 
				CRVL3DPose *pPoseC0);
	int Match2(	CRVL3DSurface2 *pSurface, 
				CRVL3DPose *pPoseSM);
	void Display(	CRVLFigure * pFig,
					CRVL3DPose *pPoseM0,
					CvScalar Color,
					DWORD Flags,
					int iSelectedPix = -1,
					CRVL3DSurface2 **ppSelectedSurf = NULL,
					CRVL3DLine2 **ppSelectedLine = NULL);
	void Display3DSurfaceSamples(	CRVLFigure * pFig,
									CRVL3DSurface2 *pSurf,
									double *A, 
									double *RCM,
									double *tCM,
									DWORD Flags = 0x00000000,
									RVL3DSURFACE_SAMPLE *pSelectedSample = NULL);
	
	void Display3DSurface(	CRVLFigure * pFig,
							CRVL3DSurface2 *pSurf,
							CRVL3DPose *pPoseM0,
							CvScalar Color,
							int LineWidth = 1,
							DWORD Flags = 0x00000000,
							RVL3DSURFACE_SAMPLE *pSelectedSample = NULL);
	void Display3DLine(	CRVLFigure * pFig,
						CRVL3DLine2 *pLine,
						CRVL3DPose *pPoseM0,
						CvScalar Color,
						int LineWidth = 2,
						DWORD Flags = 0x00000000);
	void GetCenter(double *XCenter);
	void Display2DLines(CRVLFigure *pFig);
	void Display3DLines(CRVLFigure *pFig,
						CRVL3DPose *pPoseC0,
						BOOL bInvTransf = FALSE,
						CRVLPSuLM *pPSuLM = NULL);
	void Display(	CRVLGUI * pGUI,
					CRVL3DPose *pPoseS0,
					char *FigName = NULL,
					DWORD Flags = RVLPSULM_DISPLAY_SURFACES);
	void Save(FILE * fp, DWORD Flags = 0x00000000);
	void Load(FILE * fp, DWORD Flags = 0x00000000);
	void Save();
	void Load(DWORD Flags = 0x00000000);
	void AddToFigure(CRVLGUI * pGUI);
	void CreateCellArray();
	//Original code taken from "Match2"
	int MatchGeometryAndColor(	CRVL3DSurface2 *pSurface, 
								CRVL3DPose *pPoseSM,
								int noBins = 5,
								float *helperArray = NULL);
	int* GetCorrectHypothesisIdxFromGT(CRVLMPtrChain* hypothesislist,
									  CRVL3DPose* pPoseAC,
									  double toleranceXYZ,
									  double toleranceAng,
									  char* gtFile,
									  double *errData,
									  CRVL3DPose *pPoseAsAm,
									  double *gtAsAmData,
									  bool descending = true,
									  bool search = true);
	//void Clone(CRVLPSuLM *pPSulMOriginal);
	void Get3DSurfaceSamplesFrom2DRegionSamples(
		int nSamples,
		CRVLClass *p3DSurfaceSet,
		RVLPTRCHAIN_ELEMENT **ppFirstSurface);
	RVLQLIST* GetCorrectHypothesesViaGT_EvalBench(CRVLMPtrChain* hypothesislist,
											CRVL3DPose* pPoseAC,
											double toleranceXYZ,
											double toleranceAng,
											char* gtFile,
											CRVLMem *pMem,
											bool descending = true);
	//void CreateMeshFile(char *MeshFileName,
	//					RVLPSULM_HYPOTHESIS *pHypothesis = NULL);
	void AddToMeshFile(	char *MeshName,
						FILE *fp,
						int &iVertex0,
						CRVL3DPose *pPose,
						double scale,
						RVLPSULM_MESH_FILE_GROUP_DATA *GroupData,
						int nGroups);
	RVL3DSURFACE_SAMPLE * SelectSample(
		CRVLFigure * pFig,
		CRVL3DSurface2 * pSurf, 
		CRVL3DPose *pPoseCM, 
		int u, int v);

public:
	WORD m_Index;
	WORD m_iSubMap;
	DWORD m_Flags;
	void *m_vpBuilder;
	CRVLMPtrChain m_SurfaceList;
	CRVLMPtrChain m_3DLineList;
	CRVL3DLine2 **m_3DLineArray;
	int m_n3DLines, m_n3DLinesTotal;
	CRVLMPtrChain m_2DLineList;
	double m_X1[3], m_X2[3];		// only for testing purpose!!!
	RVLPSULM_CELL *m_CellArray;
	double m_NGroundPlane[3];
	double m_CameraHeight;
	double m_Alpha;
	double m_aGroundPlane, m_bGroundPlane, m_cGroundPlane;
	RVLQLIST m_LandmarkList;
	int m_nLandmarks;
	int m_Evidence;
	RVLPSULM_LANDMARK * SelectLandmarkFromDisplay(int * iU);
	//CRVL3DPose m_Pose0M;
	CRVL3DSurface2 **m_3DSurfaceArray;
	CRVL3DMeshObject *m_pRootMeshObject;
	int m_n3DSurfaces, m_n3DSurfacesTotal;
	char *m_FileName;	//Path to image
	char m_Name[200];
	
	int m_ModelId;
	RVLQLIST* m_NeighbourList;
	RVLQLIST m_LocalMap;
	int m_nLocalMapNodes;
	CRVL3DPose *m_PoseAbs;  //Pose rel. to global c.s.
	char *m_ModelFilePath;	//Path to model file
	CRVL3DPose m_PoseRTAs;
	CRVL3DPose m_PoseCsInit;
	double m_CRTAs[3 * 3 * 3];
	double m_CCsInit[3 * 3 * 3];	
	double m_minInfo;
	BOOL m_minPlaneExists;
	//CRVL3DPose *m_PoseGT;  //Ground truth pose
	//double m_GTData[3];  // Ground truth data
	RVLPSULM_HYPOTHESIS *m_pHypothesis;
	RVLPSULM_PATH_PLANNING_NEIGHBOR *m_PathPlanningNeighbourArray;
	int m_nPathPlanningNeighbours;
	int m_nSurfaceSamples;
	RVLPSULM_PARTICLE2 *m_ParticleArray;
	int m_nParticles;
	double m_PriorProbabilityLocal;
	double m_PosteriorProbabilityLocal5DOF;
	double m_PosteriorProbabilityLocal;
	double m_PosteriorProbabilityGlobal;	
};

