// RVL2DLine2.h: interface for the CRVL2DLine2 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL2DLINE2_H__9BCB8681_7477_43F6_892A_9A6A61CD783A__INCLUDED_)
#define AFX_RVL2DLINE2_H__9BCB8681_7477_43F6_892A_9A6A61CD783A__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVL2DObject.h"

#define RVL2DLINE2_PARAM_FLAG_IU	0x00010000
#define RVL2DLINE2_PARAM_FLAG_U		0x00020000
#define RVL2DLINE2_PARAM_FLAG_CSDL	0x00040000
#define RVL2DLINE2_PARAM_FLAG_DIU	0x00080000
#define RVL2DLINE2_PARAM_FLAG_MID	0x00100000
#define RVLRELLIST_INDEX_2DLINE_3D2DLINE			3
#define RVLRELLIST_INDEX_2DLINE_CONTOUR_SEGMENTS	4

class CRVL2DLine2;

void RVLCreateC2DLine(CRVLClass *pClass);
CRVL2DLine2 *RVLFindNearest2DLine(CRVLMPtrChain *p2DLineArray,
								int *iU,
								int MinVectorLength = 0);
void RVLFit2DLine(RVL2DMOMENTS *pData, 
				  double &cs, double &sn, double &d, 
				  double &err);
CRVL2DLine2 *RVLFindNearest2DLine(	CRVLObject2 **LineArray,
									int nLines,
									int *iU,
									int MinVectorLength = 0);


class CRVL2DLine2 : public CRVL2DObject
{
public:
	int m_iU[2][2];
	double m_U[2][2];
	double m_cs, m_sn, m_d;		// line equation: -m_sn * u + m_cs * v = m_d
	int m_diU[2];
	int m_leniU;
	double m_len;

public:
	CRVL2DLine2();
	virtual ~CRVL2DLine2();
	CRVLObject2 * Create2(CRVLClass * pClass);
	void Save(	FILE *fp,
				DWORD Flags);
	void Load(	FILE *fp,
				DWORD Flags);
	void SetCSDL();
	double Distance(double *U);
	int Distance(int *iU);
	void SetdiU();
	void UpdateParams();
};

extern CRVL2DLine2 RVL2DLine2Template;

#endif // !defined(AFX_RVL2DLINE2_H__9BCB8681_7477_43F6_892A_9A6A61CD783A__INCLUDED_)
