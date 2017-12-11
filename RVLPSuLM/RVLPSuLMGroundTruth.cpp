#include "RVLCore.h"
#include "RVLPSuLMGroundTruth.h"

CRVLPSuLMGroundTruth::CRVLPSuLMGroundTruth(void)
{
	m_Mem.Create(1000000);

	m_FileName = NULL;
}

CRVLPSuLMGroundTruth::~CRVLPSuLMGroundTruth(void)
{
	if(m_FileName)
		delete[] m_FileName;
}

void CRVLPSuLMGroundTruth::Init(void)
{
	RVLQLIST *pMatchList = &m_MatchList;

	RVLQLIST_INIT(pMatchList)

	m_nMatches = 0;
}


void CRVLPSuLMGroundTruth::Clear(void)
{
	m_Mem.Clear();
}

void CRVLPSuLMGroundTruth::Add(int iScene,
							   int iModel,
							   CRVL3DPose *pPose)
{
	CRVLMem *pMem = &m_Mem;

	RVLQLIST *pMatchList = &m_MatchList;

	bool bMatchAlreadyExists = false;

	RVLPSULM_GROUND_TRUTH_MATCH *pMatch = (RVLPSULM_GROUND_TRUTH_MATCH *)(m_MatchList.pFirst);

	while(pMatch)
	{
		if(bMatchAlreadyExists = (pMatch->iScene == iScene && pMatch->iModel == iModel))
			break;

		pMatch = (RVLPSULM_GROUND_TRUTH_MATCH *)(pMatch->pNext);
	}

	if(!bMatchAlreadyExists)
	{
		RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_GROUND_TRUTH_MATCH, pMatch)

		pMatch->iScene = iScene;
		pMatch->iModel = iModel;

		RVLQLIST_ADD_ENTRY(pMatchList, pMatch)

		m_nMatches++;
	}

	double *RSrc = pPose->m_Rot;
	double *tSrc = pPose->m_X;
	double *RTgt = pMatch->PoseSM.m_Rot;
	double *tTgt = pMatch->PoseSM.m_X;

	RVLCOPYMX3X3(RSrc, RTgt)
	RVLCOPY3VECTOR(tSrc, tTgt)
}

bool CRVLPSuLMGroundTruth::Save()
{
	if(m_FileName == NULL)
		return false;

	FILE *fp = fopen(m_FileName, "w");

	if(fp == NULL)
		return false;

	RVLPSULM_GROUND_TRUTH_MATCH *pMatch = (RVLPSULM_GROUND_TRUTH_MATCH *)(m_MatchList.pFirst);

	while(pMatch)
	{
		fprintf(fp, "%05d\t%05d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
			pMatch->iScene, pMatch->iModel,
			pMatch->PoseSM.m_X[0], pMatch->PoseSM.m_X[1], pMatch->PoseSM.m_X[2], 
			pMatch->PoseSM.m_Rot[0], pMatch->PoseSM.m_Rot[1], pMatch->PoseSM.m_Rot[2], 
			pMatch->PoseSM.m_Rot[3], pMatch->PoseSM.m_Rot[4], pMatch->PoseSM.m_Rot[5], 
			pMatch->PoseSM.m_Rot[6], pMatch->PoseSM.m_Rot[7], pMatch->PoseSM.m_Rot[8]);	
			
		pMatch = (RVLPSULM_GROUND_TRUTH_MATCH *)(pMatch->pNext);
	}

	fclose(fp);

	return true;
}

bool CRVLPSuLMGroundTruth::Load()
{
	if(m_FileName == NULL)
		return false;

	FILE *fp = fopen(m_FileName, "r");

	if(fp == NULL)	
		return false;

	CRVLMem *pMem = &m_Mem;

	RVLQLIST *pMatchList = &m_MatchList;

	RVLPSULM_GROUND_TRUTH_MATCH *pMatch;

	while(!feof(fp))
	{
		RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_GROUND_TRUTH_MATCH, pMatch)

		RVLQLIST_ADD_ENTRY(pMatchList, pMatch)

		fscanf(fp, "%05d\t%05d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
			&(pMatch->iScene), &(pMatch->iModel),
			pMatch->PoseSM.m_X, pMatch->PoseSM.m_X + 1, pMatch->PoseSM.m_X + 2, 
			pMatch->PoseSM.m_Rot + 0, pMatch->PoseSM.m_Rot + 1, pMatch->PoseSM.m_Rot + 2, 
			pMatch->PoseSM.m_Rot + 3, pMatch->PoseSM.m_Rot + 4, pMatch->PoseSM.m_Rot + 5, 
			pMatch->PoseSM.m_Rot + 6, pMatch->PoseSM.m_Rot + 7, pMatch->PoseSM.m_Rot + 8);

		m_nMatches++;
	}

	fclose(fp);

	return true;
}

void CRVLPSuLMGroundTruth::Get(	int iScene,
								RVLPSULM_GROUND_TRUTH_MATCH **Match, 
								int &nMatches)
{
	nMatches = 0;

	RVLPSULM_GROUND_TRUTH_MATCH **ppMatch_ = Match;

	RVLPSULM_GROUND_TRUTH_MATCH *pMatch = (RVLPSULM_GROUND_TRUTH_MATCH *)(m_MatchList.pFirst);

	while(pMatch)
	{
		if(pMatch->iScene == iScene)
		{
			*(ppMatch_++) = pMatch;

			nMatches++;
		}

		pMatch = (RVLPSULM_GROUND_TRUTH_MATCH *)(pMatch->pNext);
	}
}
