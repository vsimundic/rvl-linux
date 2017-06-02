#pragma once

#define RVLPSULM_PCG_FLAG_ACTIVE	0x01

//#define RVLPSULM_INDEXING_FLANN

//#define RVLPSULM_INDEXING_DEBUG
//#define RVLPSULM_INDEXING_MINDICATORS_DEBUG
//#define RVLPSULM_INDEXING_SINDICATORS_DEBUG
//#define RVLPSULM_INDEXING_INDEXING_DEBUG

#ifdef RVLPSULM_INDEXING_MINDICATORS_DEBUG
#define RVLPSULM_INDEXING_INDICATORS_DEBUG
#else
#ifdef RVLPSULM_INDEXING_SINDICATORS_DEBUG
#define RVLPSULM_INDEXING_INDICATORS_DEBUG
#endif
#endif

#define RVLPSULM_INDEXING_GET_ORIENTATION(N, abs, iOrientation)\
{\
	abs[0] = RVLABS(N[0]);\
	abs[1] = RVLABS(N[1]);\
	abs[2] = RVLABS(N[2]);\
	iOrientation = (abs[0] >= abs[1] ? 0 : 1);\
	if(abs[2] > abs[iOrientation])\
		iOrientation = 2;\
	if(N[iOrientation] < 0.0f)\
		iOrientation += 3;\
}

struct RVLPSULM_INDICATOR
{
	int iPCG;
	float m_Descriptor[10]; //[7]
	int iFeature;
	void *pNext;
};

struct RVLPSULM_INDICATOR_MATCH
{
	int iS;
	int iM;
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
	void BuildIndex();
	void Search(
		RVLPSULM_INDICATOR *pIndicator,
		RVLARRAY_<int> &MatchArray);
	//void AssignWordsToIndicators(
	//	RVLPSULM_PCG **PCG,
	//	char **pWordLT);

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
	float *m_FeatureArray;
	float m_dBinSize;
	float m_dRange;
	float m_dOffset;
	RVLPSULM_INDICATOR *m_MIndicatorArray;
	int *m_IndexMem;
	RVLARRAY_<int> *m_Index;
	int m_maxnIndicatorsInBin;
	int *m_MatchMem;

private:
	int m_ndBins;
};
