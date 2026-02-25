// RVLRelation.cpp: implementation of the CRVLRelation class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVLClass.h"
#include "RVLObject2.h"
#include "RVLRelation.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLRelation::CRVLRelation()
{

}

CRVLRelation::~CRVLRelation()
{

}

void CRVLRelation::ConnectObjects(CRVLObject2 *pObject1, 
								  CRVLObject2 *pObject2,
								  int iRelList1, 
								  int iRelList2,
								  CRVLMPtrChain2 *pRelList)
{
	m_pObject[0] = pObject1;
	m_pObject[1] = pObject2;
	
	pRelList->Create(pObject1->m_RelList + iRelList1,
		pObject1->m_pClass->m_pMem);

	pRelList->Add(this);

	pRelList->Create(pObject2->m_RelList + iRelList2,
		pObject2->m_pClass->m_pMem);

	pRelList->Add(this);	
}

void CRVLRelation::ConnectObjects(CRVLObject2 *pObject1, 
								  CRVLObject2 *pObject2,
								  int iRelList1, 
								  int iRelList2)
{
	m_pObject[0] = pObject1;
	m_pObject[1] = pObject2;
	
	pObject1->m_pClass->m_RelList.Create(pObject1->m_RelList + iRelList1,
		pObject1->m_pClass->m_pMem);

	pObject1->m_pClass->m_RelList.Add(this);

	pObject2->m_pClass->m_RelList.Create(pObject2->m_RelList + iRelList2,
		pObject2->m_pClass->m_pMem);
	
	pObject2->m_pClass->m_RelList.Add(this);
}

void CRVLRelation::ConnectObjects(CRVLObject2 *pObject1, 
								  CRVLObject2 *pObject2,
								  int RelListID)
{
	m_pObject[0] = pObject1;
	m_pObject[1] = pObject2;

	CRVLClass *pClass1 = pObject1->m_pClass;
	CRVLClass *pClass2 = pObject2->m_pClass;
	
	pClass1->m_RelList.Create(pObject1->m_RelList + pClass1->m_iRelList[RelListID],
		pClass1->m_pMem);

	pClass1->m_RelList.Add(this);

	pClass2->m_RelList.Create(pObject2->m_RelList + pClass2->m_iRelList[RelListID],
		pClass2->m_pMem);

	pClass2->m_RelList.Add(this);
}

void CRVLRelation::ConnectObjects(CRVLObject2 *pObject1, 
								  CRVLObject2 *pObject2,
								  int RelListID,
								  CRVLMem *pMem1, CRVLMem *pMem2)
{
	m_pObject[0] = pObject1;
	m_pObject[1] = pObject2;

	CRVLClass *pClass1 = pObject1->m_pClass;
	CRVLClass *pClass2 = pObject2->m_pClass;
	
	pClass1->m_RelList.Create(pObject1->m_RelList + pClass1->m_iRelList[RelListID], pMem1);

	pClass1->m_RelList.Add(this);

	pClass2->m_RelList.Create(pObject2->m_RelList + pClass2->m_iRelList[RelListID], pMem2);

	pClass2->m_RelList.Add(this);
}


void CRVLRelation::ConnectObjectsUniq(CRVLObject2 *pObject1, 
									 CRVLObject2 *pObject2,
									 int iData1, 
									 int iData2)
{
	m_pObject[0] = pObject1;
	m_pObject[1] = pObject2;

	*((CRVLRelation **)(pObject1->m_pData + iData1)) = this;
	*((CRVLRelation **)(pObject2->m_pData + iData2)) = this;	
}





