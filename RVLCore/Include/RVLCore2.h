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
#include "RVLColorDescriptor.h"

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
void RVLCopyString(char *strSrc, char **pstrTgt, CRVLMem *pMem = NULL);

// RVL3DTools

#define RVLCREATE3DTRANSF(R, t, T)\
{\
	T[0] = R[0];\
	T[1] = R[1];\
	T[2] = R[2];\
	T[4] = R[3];\
	T[5] = R[4];\
	T[6] = R[5];\
	T[8] = R[6];\
	T[9] = R[7];\
	T[10] = R[8];\
	T[3] = t[0];\
	T[7] = t[1];\
	T[11] = t[2];\
	T[12] = T[13] = T[14] = 0.0;\
	T[15] = 1.0;\
}

namespace RVL
{
	template <typename T> struct Vector3
	{
		T Element[3];
	};

	template <typename T> struct Matrix3
	{
		T Element[9];
	};

	template <typename T> 
	void UpdateBoundingBox(Box<T> *pBox, T *P)
	{
		if (P[0] < pBox->minx)
			pBox->minx = P[0];
		else if (P[0] > pBox->maxx)
			pBox->maxx = P[0];

		if (P[1] < pBox->miny)
			pBox->miny = P[1];
		else if (P[1] > pBox->maxy)
			pBox->maxy = P[1];

		if (P[2] < pBox->minz)
			pBox->minz = P[2];
		else if (P[2] > pBox->maxz)
			pBox->maxz = P[2];
	}
}


//V = [x y z]'
#define RVLSET3VECTOR(V, x, y, z)	{V[0] = x; V[1] = y; V[2] = z;}

// RVLMem

#define RVLMEM_SET_FREE(pMem, pFreeMem)		pMem->m_pFreeMem = (unsigned char *)(pFreeMem);