//#include "Platform.h"
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
#include "RVLTimer.h"
#include "RVLKinect.h"
#include "RVL3DTools.h"
#include "RVLRGBDTools.h"
#include "RVLMem.h"
#include "RVLQListArray.h"
//#include "RVLPtrChain.h"
#include "RVLMChain.h"
//#include "RVLMPtrChain.h"
#include "RVLParameterList.h"
//#include "RVL3DPose.h"
//#include "RGBDCamera.h"

//struct PIX_ARRAY
//{
//	int Width;
//	int Height;
//	int nPixBytes;
//	unsigned char *pPix;
//	BOOL bOwnData;
//	BOOL bColor;
//};
//
//#include "RVLCamera.h"
//#include "RVLGUI.h"

// RVLUtil

char *RVLCreateFileName(char *SrcFileName,
	char *SrcExtension,
	int n,
	char *TgtExtension,
	CRVLMem *pMem = NULL);
char *RVLCreateString(char *strIn);

// RVL3DTools

//V = [x y z]'
#define RVLSET3VECTOR(V, x, y, z)	{V[0] = x; V[1] = y; V[2] = z;}

// RVLQListArray

#define RVLQLIST_GET_NEXT_CIRCULAR(pList, pElement)	{pElement = pElement->pNext; if(!pElement) pElement = pList->pFirst;}

namespace RVL
{
	namespace QLIST
	{
		template<typename T>
		void CreatePtrArray(QList<T> *pList, Array<T *> *pArray)
		{
			T *pData = pList->pFirst;

			T **ppData_ = pArray->Element;

			while (pData)
			{
				*(ppData_++) = pData;

				pData = pData->pNext;
			}

			pArray->n = ppData_ - pArray->Element;
		}
	}
}


// RVLMem

#define RVLMEM_SET_FREE(pMem, pFreeMem)		pMem->m_pFreeMem = (unsigned char *)(pFreeMem);