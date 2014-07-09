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
	//CRVL3DPose *pPoseCM;
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

public:
	CRVLPSuLMBuilder m_PSuLMBuilder;
	CRVLPSuLM *m_pPSuLM, *m_pPrevPSuLM;
	CRVL3DPose m_PoseLA, m_PoseA0;
};
