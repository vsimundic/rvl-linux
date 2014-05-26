// RVLMChain2.cpp: implementation of the CRVLMChain2 class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "Platform.h"

#include <stdlib.h>
#include <string.h>
#include "RVLConst.h"
#include "RVLMem.h"
#include "RVLArray.h"
#include "RVLMChain2.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLMChain2::CRVLMChain2()
{

}

CRVLMChain2::CRVLMChain2(RVLARRAY *pArray, 
						 CRVLMem *pMem, 
						 int DataSize)
{
	m_pArray = pArray;
	m_pMem = pMem;
	m_DataSize = DataSize;
	m_ElementSize = m_DataSize + sizeof(void *);
}

CRVLMChain2::~CRVLMChain2()
{

}

void CRVLMChain2::Add(void *pData)
{
	BYTE *pElement = m_pMem->Alloc(m_ElementSize);
	*((BYTE **)pElement) = NULL;
	memcpy(pElement + sizeof(BYTE *), pData, m_DataSize);
	if(m_pArray->pFirst == NULL)
		m_pArray->pFirst = m_pArray->pEnd = pElement;
	else
	{
		*((BYTE **)(m_pArray->pEnd)) = pElement;
		m_pArray->pEnd = pElement;
	}
}

void * CRVLMChain2::GetNext()
{
	m_pCurrent = m_pNext;

	m_pNext = *((BYTE **)m_pCurrent);

	return m_pCurrent + sizeof(BYTE *);
}

void CRVLMChain2::RemoveAll()
{
	m_pArray->pFirst = NULL;
}

void CRVLMChain2::Start()
{
	m_pNext = m_pArray->pFirst;
	m_pCurrent = NULL;
}

void CRVLMChain2::InsertAt(void *pData, 
						   BYTE *pCurrent)
{
	BYTE *pElement = m_pMem->Alloc(m_ElementSize);
	memcpy(pElement + sizeof(BYTE *), pData, m_DataSize);

	if(pCurrent != NULL)
	{
		*((BYTE **)pElement) = *((BYTE **)m_pCurrent);
		*((BYTE **)m_pCurrent) = pElement;
	}
	else
	{
		*((BYTE **)pElement) = m_pArray->pFirst;
		m_pArray->pFirst = pElement;
	}
}

void CRVLMChain2::Create(RVLARRAY *pArray, 
						 CRVLMem *pMem, 
						 int DataSize)
{
	m_pArray = pArray;
	m_pMem = pMem;
	m_DataSize = DataSize;
	m_ElementSize = m_DataSize + sizeof(void *);
}

void * CRVLMChain2::GetLast()
{
	return m_pArray->pEnd + sizeof(BYTE *);
}


