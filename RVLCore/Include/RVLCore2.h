//#include "Platform.h"
#include "RVLPlatform.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <memory>
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

	template < typename T1, typename T2 > struct Correspondence
	{
		T1 item1;
		T2 item2;
	};

	template <typename T>
	void InitBoundingBox(Box<T> *pBox, T *P)
	{
		pBox->minx = pBox->maxx = P[0];
		pBox->miny = pBox->maxy = P[1];
		pBox->minz = pBox->maxz = P[2];
	}

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

	template <typename T>
	bool InBoundingBox(Box<T> *pBox, T *P)
	{
		return (P[0] >= pBox->minx && P[0] <= pBox->maxx && 
			P[1] >= pBox->miny && P[1] <= pBox->maxy && 
			P[2] >= pBox->minz && P[2] <= pBox->maxz);
	}

	template <typename T>
	bool BoxIntersection(
		Box<T> *pBoxSrc1,
		Box<T> *pBoxSrc2,
		Box<T> *pBoxTgt)
	{
		pBoxTgt->minx = RVLMAX(pBoxSrc1->minx, pBoxSrc2->minx);
		pBoxTgt->maxx = RVLMIN(pBoxSrc1->maxx, pBoxSrc2->maxx);

		if (pBoxTgt->minx >= pBoxTgt->maxx)
			return false;

		pBoxTgt->miny = RVLMAX(pBoxSrc1->miny, pBoxSrc2->miny);
		pBoxTgt->maxy = RVLMIN(pBoxSrc1->maxy, pBoxSrc2->maxy);

		if (pBoxTgt->miny >= pBoxTgt->maxy)
			return false;

		pBoxTgt->minz = RVLMAX(pBoxSrc1->minz, pBoxSrc2->minz);
		pBoxTgt->maxz = RVLMIN(pBoxSrc1->maxz, pBoxSrc2->maxz);

		if (pBoxTgt->minz >= pBoxTgt->maxz)
			return false;

		return true;
	}

	template <typename T>
	void BoxSize(
		Box<T> *pBox,
		T &a,
		T &b,
		T &c)
	{
		a = pBox->maxx - pBox->minx;
		b = pBox->maxy - pBox->miny;
		c = pBox->maxz - pBox->minz;
	}

	template <typename T>
	T BoxSize(Box<T> *pBox)
	{
		T a, b, c;
		
		BoxSize(pBox, a, b, c);

		T tmp = RVLMAX(a, b);

		return RVLMAX(tmp, c);
	}

	template <typename T>
	T BoxVolume(Box<T> *pBox)
	{
		return (pBox->maxx - pBox->minx) * (pBox->maxy - pBox->miny) * (pBox->maxz - pBox->minz);
	}

	template <typename T>
	void BoxCenter(
		Box<T> *pBox,
		T *P)
	{
		P[0] = 0.5f * (pBox->minx + pBox->maxx);
		P[1] = 0.5f * (pBox->miny + pBox->maxy);
		P[2] = 0.5f * (pBox->minz + pBox->maxz);
	}
}


//V = [x y z]'
#define RVLSET3VECTOR(V, x, y, z)	{V[0] = x; V[1] = y; V[2] = z;}

// RVLMem

#define RVLMEM_SET_FREE(pMem, pFreeMem)		pMem->m_pFreeMem = (unsigned char *)(pFreeMem);

// RVLArray

#define RVL3DARRAY_INDICES(Array, idx, x, y, z) {x = idx % Array.a; y = (idx / Array.a); z = y / Array.b; y = y % Array.b;}
#define RVL3DARRAY_INDEX(Array, x, y, z)	(Array.a * (Array.b * (z) + (y)) + (x))
