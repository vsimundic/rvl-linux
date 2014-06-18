
class CRVLPSuLMVS :
	public CRVLPCSVS
{
public:
	CRVLPSuLMVS(void);
	virtual ~CRVLPSuLMVS(void);
	void Init(char * CfgFile2Name = NULL);
	void Update(void);
	void PSuLMBasedRLMUpdate(DWORD Flags);
	void CreateParamList();

public:
	CRVLPSuLMBuilder m_PSuLMBuilder;
	CRVLPSuLM *m_pPSuLM, *m_pPrevPSuLM;
	CRVL3DPose m_PoseLA, m_PoseA0;
};
