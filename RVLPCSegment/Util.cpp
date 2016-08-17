#include "RVLPlatform.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "opencv2\opencv.hpp"
#define _CRT_SECURE_NO_WARNINGS 
#include "RVLConst.h"
#ifndef ushort
#define ushort unsigned short int
#endif
#include "RVLArray.h"
#include "RVLKinect.h"
#include "RVLMem.h"
#include "RVLQListArray.h"
#include "Util.h"

using namespace RVL;

void RVL::QuickSort(int *Key, int *Index, int n)
{
	// get range

	int *pKey = Key;

	int min = *(pKey++);
	int max = min;

	int i;

	for (i = 1; i < n; i++, pKey++)
	{
		if (*pKey < min)
			min = *pKey;
		else if (*pKey > max)
			max = *pKey;
	}

	int nBins = max - min + 1;

	// create lookup table

	Array<QList<QLIST::Index>> KeyLT;

	KeyLT.Element = new QList<QLIST::Index>[nBins];
	KeyLT.n = nBins;

	QList<QLIST::Index> *pKeyList;

	for (i = 0; i < nBins; i++)
	{
		pKeyList = KeyLT.Element + i;

		RVLQLIST_INIT(pKeyList);
	}

	QLIST::Index *KeyMem = new QLIST::Index[n];

	QLIST::Index *pKeyLTEntry = KeyMem;

	int j;

	for (i = 0; i < n; i++)
	{
		j = Key[i] - min;

		pKeyList = KeyLT.Element + j;

		RVLQLIST_ADD_ENTRY(pKeyList, pKeyLTEntry);

		pKeyLTEntry->Idx = i;

		pKeyLTEntry++;
	}

	// fill index array

	int *pIndex = Index;

	for (i = 0; i < nBins; i++)
	{
		pKeyList = KeyLT.Element + i;

		pKeyLTEntry = pKeyList->pFirst;

		while (pKeyLTEntry)
		{
			*(pIndex++) = pKeyLTEntry->Idx;

			pKeyLTEntry = pKeyLTEntry->pNext;
		}
	}

	// deallocate lookup table

	delete[] KeyLT.Element;
	delete[] KeyMem;
}

void RVL::RandomColor(unsigned char *color)
{
	color[0] = (unsigned char)(rand() % 256);
	color[1] = (unsigned char)(rand() % 256);
	color[2] = (unsigned char)(rand() % 256);
}
