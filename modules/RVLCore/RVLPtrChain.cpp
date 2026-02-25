// RVLPtrChain.cpp: Implementierung der Klasse CRVLPtrChain.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "Platform.h"

#include <stdlib.h>

#include "RVLPtrChain.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Konstruktion/Destruktion
//////////////////////////////////////////////////////////////////////

CRVLPtrChain::CRVLPtrChain()
{
	m_nElements = 0;
	m_pFirst = NULL;
}

CRVLPtrChain::~CRVLPtrChain()
{
	RemoveAll();
}

void CRVLPtrChain::Add(void *pData)
{
	RVLPTRCHAIN_ELEMENT *pElement = new RVLPTRCHAIN_ELEMENT;
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

void * CRVLPtrChain::GetNext()
{
	m_pCurrent = m_pNext;

	m_pNext = m_pCurrent->pNext;

	return m_pCurrent->pData;
}

void CRVLPtrChain::RemoveAll()
{
	m_pNext = m_pFirst;

	while(m_pNext != NULL)
	{
		m_pCurrent = m_pNext;

		m_pNext = m_pCurrent->pNext;

		delete m_pCurrent;
	}

	m_nElements = 0;
	m_pFirst = NULL;
}

void CRVLPtrChain::Start()
{
	m_pNext = m_pFirst;
	m_pCurrent = NULL;
}

void CRVLPtrChain::InsertAt(void *pData, RVLPTRCHAIN_ELEMENT *pCurrent)
{
	RVLPTRCHAIN_ELEMENT *pElement = new RVLPTRCHAIN_ELEMENT;
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

