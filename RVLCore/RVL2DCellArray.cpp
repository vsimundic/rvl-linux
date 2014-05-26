// RVL2DCellArray.cpp: implementation of the CRVL2DCellArray class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVL2DCellArray.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL2DCellArray::CRVL2DCellArray()
{

}

CRVL2DCellArray::~CRVL2DCellArray()
{

}

void CRVL2DCellArray::Init()
{
	m_nCols = (m_Width  - 1) / m_CellSize + 1;
	m_nRows  = (m_Height - 1) / m_CellSize + 1;

	m_CellArray.m_Size =  m_nCols * m_nRows;
	m_CellArray.Init();
}

void CRVL2DCellArray::Add(int u, int v, 
						  void *pObject)
{
	RVLQLIST_PTR_ENTRY *pEntry = (RVLQLIST_PTR_ENTRY *)(m_pMem->Alloc(sizeof(RVLQLIST_PTR_ENTRY)));
	pEntry->Ptr = pObject;
	RVLQLIST *pList = m_CellArray.m_ListArray + u / m_CellSize + (v / m_CellSize) * m_nCols;
	RVLQLIST_ADD_ENTRY(pList, pEntry)
}


void CRVL2DCellArray::Get(int minu, int maxu, int minv, int maxv, 
							void **OutputArray, int &nOut)
{
	int mini = (minu > 0 ? minu / m_CellSize : 0);
	int maxi = (maxu < m_Width ? maxu / m_CellSize : m_nCols - 1);
	int nWinCols = maxi - mini + 1;	

	int minj = (minv > 0 ? minv / m_CellSize : 0);
	int maxj = (maxv < m_Height ? maxv / m_CellSize : m_nRows - 1);
	int nWinRows = maxj - minj + 1;	

	RVLQLIST *pCell = m_CellArray.m_ListArray + mini + minj * m_nCols;
	void **ppObject = OutputArray;

	int j;
	RVLQLIST *pCell2;
	RVLQLIST *pWinEnd;
	RVLQLIST_PTR_ENTRY *pEntry;

	for(j = 0; j < nWinRows; j++)
	{
		pWinEnd = pCell + nWinCols;

		for(pCell2 = pCell; pCell2 < pWinEnd; pCell2++)
		{
			pEntry = (RVLQLIST_PTR_ENTRY *)(pCell2->pFirst);

			while(pEntry)
			{
				*(ppObject++) = pEntry->Ptr;

				pEntry = (RVLQLIST_PTR_ENTRY *)(pEntry->pNext);
			}
		}

		pCell += m_nCols;
	}

	nOut = ppObject - OutputArray;
}

void CRVL2DCellArray::RemoveAll()
{
	m_CellArray.Reset();
}
