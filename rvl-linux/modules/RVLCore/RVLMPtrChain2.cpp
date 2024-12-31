// RVLMPtrChain2.cpp: implementation of the CRVLMPtrChain2 class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "Platform.h"

#include <stdlib.h>
#include <string.h>
#include "RVLConst.h"
#include "RVLMem.h"
#include "RVLArray.h"
#include "RVLPtrChain.h"
#include "RVLMPtrChain2.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLMPtrChain2::CRVLMPtrChain2()
{

}

CRVLMPtrChain2::CRVLMPtrChain2(RVLARRAY *pArray, CRVLMem *pMem)
{
	m_pArray = pArray;
	m_pMem = pMem;	
}

CRVLMPtrChain2::~CRVLMPtrChain2()
{

}

void CRVLMPtrChain2::Add(void *pData)
{
	RVLPTRCHAIN_ELEMENT *pElement = 
		(RVLPTRCHAIN_ELEMENT *)(m_pMem->Alloc(sizeof(RVLPTRCHAIN_ELEMENT)));
	pElement->pData = pData;
	pElement->pNext = NULL;
	if(m_pArray->pFirst == NULL)
		m_pArray->pFirst = m_pArray->pEnd = (BYTE *)pElement;
	else
	{
		((RVLPTRCHAIN_ELEMENT *)(m_pArray->pEnd))->pNext = pElement;
		m_pArray->pEnd = (BYTE *)pElement;
	}
}

void * CRVLMPtrChain2::GetNext()
{
	m_pCurrent = m_pNext;

	m_pNext = m_pCurrent->pNext;

	return m_pCurrent->pData;
}

void CRVLMPtrChain2::RemoveAll()
{
	m_pArray->pFirst = NULL;
}

void CRVLMPtrChain2::Start()
{
	m_pNext = (RVLPTRCHAIN_ELEMENT *)(m_pArray->pFirst);
	m_pCurrent = NULL;
}

void CRVLMPtrChain2::Create(RVLARRAY *pArray, CRVLMem *pMem)
{
	m_pArray = pArray;
	m_pMem = pMem;
}

void * CRVLMPtrChain2::GetLast()
{
	return m_pArray->pEnd + sizeof(BYTE *);
}

void CRVLMPtrChain2::ConvertToArray(RVLARRAY *pArray, 
								    CRVLMem *pMem,
									int nElements)
{
	RVLARRAY OldArray = *pArray;

	Create(&OldArray, NULL);

	if(nElements < 0)
	{
		nElements = 0;
		
		Start();

		while(m_pNext)
		{
			GetNext();

			nElements++;
		}
	}

	int Size = nElements * sizeof(void *);

	pArray->pFirst = (BYTE *)(pMem->Alloc(Size));
	pArray->pEnd = pArray->pFirst + Size;

	void **pPtr = (void **)(pArray->pFirst);

	Start();

	while(m_pNext)
		*(pPtr++) = GetNext();
}

