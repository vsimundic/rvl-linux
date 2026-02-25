//#include "Platform.h"
#include "RVLPlatform.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <memory>
#ifdef RVLLINUX
#include "opencv2/opencv.hpp"
#else
#include <windows.h>
#include "opencv2\opencv.hpp"
#endif
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

// RVLMem

#define RVLMEM_SET_FREE(pMem, pFreeMem)		pMem->m_pFreeMem = (unsigned char *)(pFreeMem);

// RVLArray

#define RVL3DARRAY_INDICES(Array, idx, x, y, z) {x = idx % Array.a; y = (idx / Array.a); z = y / Array.b; y = y % Array.b;}
#define RVL3DARRAY_INDEX(Array, x, y, z)	(Array.a * (Array.b * (z) + (y)) + (x))
