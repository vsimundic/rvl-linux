// RVLChain.cpp: Implementierung der Klasse CRVLChain.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "Platform.h"

#include <stdlib.h>
#include <string.h>
#include "RVLConst.h"
#include "RVLChain.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Konstruktion/Destruktion
//////////////////////////////////////////////////////////////////////

CRVLChain::CRVLChain()
{
	m_nElements = 0;
	m_pFirst = NULL;
}

CRVLChain::CRVLChain(int DataSize)
{
	m_DataSize = DataSize;
	m_ElementSize = m_DataSize + sizeof(void *);
	m_nElements = 0;
	m_pFirst = NULL;
}

CRVLChain::~CRVLChain()
{
	RemoveAll();
}

void CRVLChain::Add(void *pData)
{
	BYTE *pElement = (BYTE *)malloc(m_ElementSize);
	*((BYTE **)pElement) = NULL;
	memcpy(pElement + sizeof(BYTE *), pData, m_DataSize);
	if(m_pFirst == NULL)
		m_pFirst = m_pLast = pElement;
	else
	{
		*((BYTE **)m_pLast) = pElement;
		m_pLast = pElement;
	}
	m_nElements++;	
}

void CRVLChain::RemoveAll()
{
	m_pNext = m_pFirst;

	while(m_pNext != NULL)
	{
		m_pCurrent = m_pNext;

		m_pNext = *((BYTE **)m_pCurrent);

		delete m_pCurrent;
	}

	m_nElements = 0;
	m_pFirst = NULL;
}

void * CRVLChain::GetNext()
{
	m_pCurrent = m_pNext;

	m_pNext = *((BYTE **)m_pCurrent);

	return m_pCurrent + sizeof(BYTE *);
}

void CRVLChain::Start()
{
	m_pNext = m_pFirst;
	m_pCurrent = NULL;
}

void * CRVLChain::GetLast()
{
	return m_pLast + sizeof(BYTE *);
}
