struct RVLPSULM_GROUND_TRUTH_MATCH
{
	int iScene;
	int iModel;
	CRVL3DPose PoseSM;
	void *pNext;
};

class CRVLPSuLMGroundTruth
{
public:
	CRVLPSuLMGroundTruth(void);
	virtual ~CRVLPSuLMGroundTruth(void);
	void Init(void);
	void Clear(void);
	void Get(	int iScene, 
				RVLPSULM_GROUND_TRUTH_MATCH **Match, 
				int &nMatches);
	void Add(	int iScene,
				int iModel,
				CRVL3DPose *pPose);
	bool Save();
	bool Load();

public:
	CRVLMem m_Mem;
	RVLQLIST m_MatchList;
	int m_nMatches;
	char *m_FileName;	
};
