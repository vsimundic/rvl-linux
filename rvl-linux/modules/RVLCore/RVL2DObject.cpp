// RVL2DObject2.cpp: implementation of the CRVL2DObject2 class.
//
//////////////////////////////////////////////////////////////////////

#include "RVLCore.h"
#include "RVLC2D.h"
#include "RVL2DObject.h"
#include "RVLRelation.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL2DObject::CRVL2DObject()
{

}

CRVL2DObject::~CRVL2DObject()
{

}

CRVLObject2 * CRVL2DObject::Create2(CRVLClass *pClass)
{
	CRVL2DObject *pObject = (CRVL2DObject *)(pClass->m_pMem0->Alloc(sizeof(CRVL2DObject)));

	memcpy(pObject, this, sizeof(CRVL2DObject));

	pObject->Create(pClass);

	pClass->Add(pObject);

	return pObject;
}

void CRVL2DObject::UpdateParams()
{
	
}

BOOL CRVL2DObject::Match(CRVLObject2 *pObject, 
					      int &MatchCost, void *vpMatchData)
{
	return FALSE;
}
	
int CRVL2DObject::Match(CRVLMPtrChain *pSObjectArray,
						 CRVLCamera *pCamera, 
						 CRVL3DPose *pPoseLA, 
						 CRVL3DPose *pPoseA0, 
						 CRVLMPtrChain *pObjectBuff)
{
	return 0;
}



///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 


void RVLCreateC2D(CRVLClass *pClass)
{
	CRVLC2D *pC2D = (CRVLC2D *)pClass;

	pClass->Init();

	pC2D->Init();
}

