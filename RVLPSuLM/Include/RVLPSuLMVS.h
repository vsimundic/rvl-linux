#define RVLSYS_FLAGS_CREATE_GLOBAL_MESH		0x00010000
#define RVLSYS_FLAGS_EDIT_MAP				0x00020000
#define RVLSYS_FLAGS_VALIDATION				0x00040000
#define RVLSYS_FLAGS_RECORD					0x00080000
#define RVLSYS_FLAGS_REVIEW_RESULTS			0x00100000

//struct RVLPSULM_HYPOTHESIS_GT
//{
//	int iSample;
//	int iHypothesis;
//	double P;
//	char validation;
//};

class CRVLPSuLMVS;

struct RVLPSULMDISPLAY_MOUSE_CALLBACK_DATA
{
	int u, v;
	bool bSelection;
	int w;
	CRVLGUI *pGUI;
	CRVLFigure *pFig;
	CRVLFigure *pFig2;
	CRVLPSuLMVS *pVS;
	int ZoomFactor;
	DWORD mDisplayPSuLMFlags;
	int iHypothesis;
	IplImage *pImage;
	IplImage *pImage2;
	//CRVL3DPose *pPoseCM;
	char *MatchMatrixGT;
	CRVL3DSurface2 *pSelectedSurf;
};

struct RVL3DMESHFILE
{
	CRVLClass Class;
	CRVL3DMeshObject *Mesh;
	char *Name;
	int nLocalMeshes;
	CRVL3DPose LastLocalMeshPose;
	int iPt;
	int iSegment;
	unsigned char TextureType;
};

void RVLPSuLMDisplayMouseCallback2(int event, int x, int y, int flags, void* pData);

class CRVLPSuLMVS :
	public CRVLPCSVS
{
public:
	CRVLPSuLMVS(void);
	virtual ~CRVLPSuLMVS(void);
	void Init(char * CfgFile2Name = NULL);
	void Update(DWORD Flags = 0x00000000);
	void PSuLMBasedRLMUpdate(DWORD Flags);
	void CreateParamList();
	void CreateMeshFile(char *MeshFileName);
	void ClearMeshFileData();
	void AppendToMeshFile(CRVL3DPose *pRelPose);
	bool Create3DMeshFromComplexPSuLM(char *ImageFileName);
	void CreateLocal3DMesh(CRVLPSuLM *pPSuLM0);
	void Validate();
	//void SaveValidation();
	void LoadMatchMatrix();
	void SaveMatchMatrix();
	//void StoreHypothesesToMatchMatrix(int iSample);
	void ComputeMatchMatrix(int iSample);
	BOOL GetNextImageFileName(bool bBackwards = false);
	BOOL GetFirstValidImageFileName();

public:
	CRVLPSuLMBuilder m_PSuLMBuilder;
	CRVLPSuLM *m_pPSuLM, *m_pPrevPSuLM;
	CRVL3DPose m_PoseLA, m_PoseA0;
	IplImage *m_pRGBImage;
	RVL3DMESHFILE *m_pMeshFile;
	int m_iSample;
	CRVLPSuLMGroundTruth m_GroundTruth;
	double *m_MatchMatrix;
	char *m_MatchMatrixGT;
	//RVLPSULM_HYPOTHESIS_GT *m_HypothesisArrayGT;
	//int m_nMatches;
	int m_nSamples;
	CRVL3DPose m_ResPose;
	int m_iResPSuLM;
	FILE *m_fpRes;
};
