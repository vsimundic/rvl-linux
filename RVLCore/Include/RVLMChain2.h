// RVLMChain2.h: interface for the CRVLMChain2 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLMCHAIN2_H__3E8A5954_FE5D_4F80_8C52_D9A27F88EB1F__INCLUDED_)
#define AFX_RVLMCHAIN2_H__3E8A5954_FE5D_4F80_8C52_D9A27F88EB1F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLMChain2  
{
public:
	int m_DataSize;
	RVLARRAY *m_pArray;
	BYTE *m_pCurrent;
	BYTE *m_pNext;
	CRVLMem *m_pMem;
private:
	int m_ElementSize;
	
public:
	void * GetLast();
	void Create(RVLARRAY *pArray, CRVLMem *pMem, int DataSize);
	void InsertAt(void *pData, BYTE *pCurrent);
	void Start();
	void RemoveAll();
	void * GetNext();
	void Add(void *pData);
	CRVLMChain2(RVLARRAY *pArray, CRVLMem *pMem, int DataSize);
	CRVLMChain2();
	virtual ~CRVLMChain2();
};

#endif // !defined(AFX_RVLMCHAIN2_H__3E8A5954_FE5D_4F80_8C52_D9A27F88EB1F__INCLUDED_)
