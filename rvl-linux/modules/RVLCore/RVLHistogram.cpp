// RVLHistogram.cpp: implementation of the CRVLHistogram class.
//
//////////////////////////////////////////////////////////////////////

#include "RVLCore.h"
#include "RVLHistogram.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLHistogram::CRVLHistogram()
{
	m_Histogram = NULL;
	m_Name = NULL;
}

CRVLHistogram::~CRVLHistogram()
{

}

BOOL CRVLHistogram::Update(double data)
{		
	int iBin = (int)floor(data / m_BinSize);

	if(iBin < m_miniBin || iBin > m_maxiBin)
		return FALSE;

	m_Histogram[iBin - m_miniBin]++;

	return TRUE;
}

/*
void CRVLHistogram::Create()
{
	m_max = 0.0;
	m_mean = 0.0;
	double *pData;
	double data;

	m_Data.Start();
	
	while(m_Data.m_pNext)
	{
		pData = (double *)(m_Data.GetNext());

		data = *pData;

		if(data > m_max)
			m_max = data;

		m_mean += data;
	}

	m_mean /= (double)(m_Data.m_nElements);

	m_nBins = (int)floor(m_max / m_BinSize) + 1;

	m_Histogram = (int *)(m_pMem->Alloc(m_nBins * sizeof(int)));

	memset(m_Histogram, 0, m_nBins * sizeof(int));

	m_Data.Start();
	
	while(m_Data.m_pNext)
	{
		pData = (double *)(m_Data.GetNext());

		data = *pData;

		Update(data);
	}
}
*/

void CRVLHistogram::Create(int iData)
{
	if(m_Data.m_nElements == 0)
		return;

	m_mean = 0.0;

	m_Data.Start();

	double *pData = (double *)(m_Data.GetNext());

	double data = pData[iData];

	m_min = m_max = m_mean = data;
	
	while(m_Data.m_pNext)
	{
		pData = (double *)(m_Data.GetNext());

		data = pData[iData];

		if(data < m_min)
			m_min = data;

		if(data > m_max)
			m_max = data;

		m_mean += data;
	}

	m_mean /= (double)(m_Data.m_nElements);

	m_miniBin = (int)floor(m_min / m_BinSize);
	m_maxiBin = (int)floor(m_max / m_BinSize);

	m_nBins = m_maxiBin - m_miniBin + 1;

	m_Histogram = (int *)(m_pMem->Alloc(m_nBins * sizeof(int)));

	memset(m_Histogram, 0, m_nBins * sizeof(int));

	m_Data.Start();
	
	while(m_Data.m_pNext)
	{
		pData = (double *)(m_Data.GetNext());

		data = pData[iData];

		Update(data);
	}
}

void CRVLHistogram::Init(double BinSize, 
						 CRVLMem *pMem,
						 char *Name,
						 int nData)
{
	m_pMem = pMem;

	if(Name)
	{
		m_Name = (char *)(pMem->Alloc((strlen(Name) + 1) * sizeof(char)));
		m_Name = strcpy(m_Name, Name);
	}
	
	m_BinSize = BinSize;

	m_Data.Create(m_pMem, nData * sizeof(double));
}

void CRVLHistogram::Save(FILE *fp, 
						 char *FormatBinSizeIn)
{
	if(fp == NULL)
		return;

	char *FormatBinSize = "%lf\t";

	if(FormatBinSizeIn)
		FormatBinSize = FormatBinSizeIn;

	int j;

	fprintf(fp, "%s\n", m_Name);

	for(j = 0; j < m_nBins; j++)
		fprintf(fp, FormatBinSize, m_BinSize * 
			(double)(m_miniBin + j + 1));

	fprintf(fp, "\n");
	
	for(j = 0; j < m_nBins; j++)
		fprintf(fp, "%d\t", m_Histogram[j]);

	fprintf(fp, "\n");

	fprintf(fp, "min: %lf, max: %lf, avg: %lf\n", m_min, m_max, m_mean);

	fprintf(fp, "\n\n");


}
