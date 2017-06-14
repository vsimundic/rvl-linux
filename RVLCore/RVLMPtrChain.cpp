// RVLMPtrChain.cpp: implementation of the CRVLMPtrChain class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "Platform.h"

#include <stdlib.h>
#include <string.h>
#include "RVLConst.h"
#include "RVLMem.h"
#include "RVLPtrChain.h"
#include "RVLMPtrChain.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLMPtrChain::CRVLMPtrChain()
{
	m_nElements = 0;
	m_pFirst = NULL;
}

CRVLMPtrChain::CRVLMPtrChain(CRVLMem *pMem)
{
	m_pMem = pMem;
	m_nElements = 0;
	m_pFirst = NULL;
}

CRVLMPtrChain::~CRVLMPtrChain()
{
	
}

void CRVLMPtrChain::Add(void *pData)
{
	RVLPTRCHAIN_ELEMENT *pElement = 
		(RVLPTRCHAIN_ELEMENT *)(m_pMem->Alloc(sizeof(RVLPTRCHAIN_ELEMENT)));
	pElement->pData = pData;
	pElement->pNext = NULL;
	if(m_pFirst == NULL)
		m_pFirst = m_pLast = pElement;
	else
	{
		m_pLast->pNext = pElement;
		m_pLast = pElement;
	}
	m_nElements++;
}

void * CRVLMPtrChain::GetNext()
{
	m_pCurrent = m_pNext;

	m_pNext = m_pCurrent->pNext;

	return m_pCurrent->pData;
}

void CRVLMPtrChain::RemoveAll()
{
	m_nElements = 0;
	m_pFirst = NULL;	
}

void CRVLMPtrChain::Start()
{
	m_pNext = m_pFirst;
	m_pCurrent = NULL;
}

void CRVLMPtrChain::InsertAt(void *pData, RVLPTRCHAIN_ELEMENT *pCurrent)
{
	RVLPTRCHAIN_ELEMENT *pElement = 
		(RVLPTRCHAIN_ELEMENT *)(m_pMem->Alloc(sizeof(RVLPTRCHAIN_ELEMENT)));
	pElement->pData = pData;

	if(pCurrent != NULL)
	{
		pElement->pNext = pCurrent->pNext;
		pCurrent->pNext = pElement;
	}
	else
	{
		pElement->pNext = m_pFirst;
		m_pFirst = pElement;
	}

	m_nElements++;
}

void CRVLMPtrChain::Copy(CRVLMPtrChain *pChain)
{
	m_nElements = pChain->m_nElements;
	m_pFirst = pChain->m_pFirst;
	m_pLast = pChain->m_pLast;
	m_pMem = pChain->m_pMem;
}

void CRVLMPtrChain::RemoveAt(RVLPTRCHAIN_ELEMENT *pCurrent)
{
	if(pCurrent != NULL)
	{
		if(pCurrent->pNext != NULL)
			pCurrent->pNext = pCurrent->pNext->pNext;

		if(pCurrent->pNext == NULL)
			m_pLast = pCurrent;
	}
	else if(m_pFirst != NULL)
		m_pFirst = m_pFirst->pNext;

	m_nElements--;
}
