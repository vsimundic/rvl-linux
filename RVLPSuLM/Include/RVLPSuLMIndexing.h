#pragma once

#define RVLPSULM_PCG_FLAG_ACTIVE	0x01
#define RVLPSULM_INDEXING_DEBUG

struct RVLPSULM_INDICATOR
{
	int iPCG;
	float m_Descriptor[10]; //[7]
	int iFeature;
	void *pNext;
};


struct RVLPSULM_PCG
{
	BYTE Flags;
	int Index;
	CRVL3DPose Pose;
	CRVLPSuLM *pPSuLM;
	int iFeature[3];
	void *pNext;
};

struct RVLPCG_MATCH
{
	RVLPSULM_PCG *pMPCG, *pSPCG;
	void *pNext;
};

class CRVLPSuLMIndexing
{
public:
	CRVLPSuLMIndexing(void);
	virtual ~CRVLPSuLMIndexing(void);
	void Init();
	void GetIndicators(
		CRVLPSuLM * pPSuLM,
		bool bModel = false);
	void GenerateHypotheses();
	void UpdateBase();
	void CreateBase();
	void ResetIndicatorAndPCGList();

public:
	void *m_vpBuilder;
	RVLQLIST m_PCGList;
	RVLQLIST m_IndicatorList;
	int m_nIndicators;
	int m_nMIndicators;
	int *m_iPCG;
	RVLPSULM_PCG **m_PCG;
	flann::Index<flann::L2<float>> *m_pIndex;
	int m_nPCGs;
	int m_nMPCGs;
	float m_rAngle;
	float m_rDist;
	float m_rUnitVect;
	CRVLQListArray m_EvidenceAccu;
	int *m_PCGBuff;
	CRVLMem m_Mem;
	int m_nFeatures;
};
