#include "RVLCore.h"
#include "RVLC2D.h"
#include "RVL2DLine2.h"

//#include "RVLCommon2.h"
//#include "RVLEDT.h"
//#include "RVLC2D.h"
//#include "RVLC2DContour.h"
//#include "RVL2DContour.h"
//#include "RVL2DLine2.h"

CRVL2DLine2 RVL2DLine2Template;

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL2DLine2::CRVL2DLine2()
{

}


CRVL2DLine2::~CRVL2DLine2()
{

}

CRVLObject2 * CRVL2DLine2::Create2(CRVLClass * pClass)
{
	CRVL2DLine2 *pObject = (CRVL2DLine2 *)(pClass->m_pMem0->Alloc(sizeof(CRVL2DLine2)));

	memcpy(pObject, this, sizeof(CRVL2DLine2));

	pObject->CRVLObject2::Create(pClass);

	pClass->Add(pObject);

	return pObject;		
}

void CRVL2DLine2::UpdateParams()
{
	CRVL2DObject::UpdateParams();

	if((m_ParamFlags & RVL2DLINE2_PARAM_FLAG_IU) &&  
	   ((m_ParamFlags & RVL2DLINE2_PARAM_FLAG_DIU) == 0))
	   SetdiU();

	if((m_ParamFlags & RVL2DLINE2_PARAM_FLAG_U) &&  
	   ((m_ParamFlags & RVL2DLINE2_PARAM_FLAG_CSDL) == 0))
	   SetCSDL();
}

void CRVL2DLine2::SetdiU()
{
	m_diU[0] = m_iU[1][0] - m_iU[0][0];
	m_diU[1] = m_iU[1][1] - m_iU[0][1];
	m_leniU = DOUBLE2INT(sqrt((double)(m_diU[0] * m_diU[0] + 
		m_diU[1] * m_diU[1])));

	m_ParamFlags |= RVL2DLINE2_PARAM_FLAG_DIU;
}


void CRVL2DLine2::SetCSDL()
{
	double du = m_U[1][0] - m_U[0][0];
	double dv = m_U[1][1] - m_U[0][1];

	m_len = sqrt(du * du + dv * dv);
	m_cs = du / m_len;
	m_sn = dv / m_len;
	m_d = -m_sn * m_U[0][0] + m_cs * m_U[0][1];

	m_ParamFlags |= RVL2DLINE2_PARAM_FLAG_CSDL;
}

int CRVL2DLine2::Distance(int *iU)
{
	int p, q;

	LinearTransform2D(m_diU[0], m_diU[1], m_leniU, 
		m_iU[0][0], m_iU[0][1], iU[0], iU[1], p, q);

	if(p < 0)
	{
		int du = iU[0] - m_iU[0][0];
		int dv = iU[1] - m_iU[0][1];

		return DOUBLE2INT(sqrt((double)(du*du+dv*dv)));
	}
	else if(p > m_leniU)
	{
		int du = iU[0] - m_iU[1][0];
		int dv = iU[1] - m_iU[1][1];

		return DOUBLE2INT(sqrt((double)(du*du+dv*dv)));
	}
	else
		return abs(q);	
}


double CRVL2DLine2::Distance(double *U)
{
	double p, q;

	LinearTransform2D(m_cs, m_sn, m_U[0][0], m_U[0][1], U[0], U[1], p, q);

	if(p < 0.0)
	{
		double du = U[0] - m_U[0][0];
		double dv = U[1] - m_U[0][1];

		return sqrt(du*du+dv*dv);
	}
	else if(p > m_len)
	{
		double du = U[0] - m_U[1][0];
		double dv = U[1] - m_U[1][1];

		return sqrt(du*du+dv*dv);
	}
	else
		return fabs(q);
}


void CRVL2DLine2::Save(FILE *fp,
					   DWORD Flags)
{
	fwrite(m_iU[0], sizeof(int), 2, fp);
	fwrite(m_iU[1], sizeof(int), 2, fp);
	fwrite(m_diU, sizeof(int), 2, fp);
	fwrite(&m_leniU, sizeof(int), 1, fp);
}

void CRVL2DLine2::Load(	FILE *fp,
						DWORD Flags)
{
	fread(m_iU[0], sizeof(int), 2, fp);
	fread(m_iU[1], sizeof(int), 2, fp);
	fread(m_diU, sizeof(int), 2, fp);
	fread(&m_leniU, sizeof(int), 1, fp);
}


///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 

void RVLCreateC2DLine(CRVLClass *pClass)
{
	pClass->m_nRelLists = 5;

	pClass->m_RelListDesc = new RVLRELLIST_DESC[pClass->m_nRelLists];

	pClass->m_RelListDesc[0].index = RVLRELLIST_INDEX_NEIGHBORS;
	pClass->m_RelListDesc[0].type = RVLRELLIST_TYPE_CHAIN;

	pClass->m_RelListDesc[1].index = RVLRELLIST_INDEX_COMPONENTS;
	pClass->m_RelListDesc[1].type = RVLRELLIST_TYPE_CHAIN;

	pClass->m_RelListDesc[2].index = RVLRELLIST_INDEX_SUPEROBJECTS;
	pClass->m_RelListDesc[2].type = RVLRELLIST_TYPE_CHAIN;

	pClass->m_RelListDesc[3].index = RVLRELLIST_INDEX_2DLINE_3D2DLINE;
	pClass->m_RelListDesc[3].type = RVLRELLIST_TYPE_CHAIN;

	pClass->m_RelListDesc[4].index = RVLRELLIST_INDEX_2DLINE_CONTOUR_SEGMENTS;
	pClass->m_RelListDesc[4].type = RVLRELLIST_TYPE_CHAIN;	

	RVLCreateC2D(pClass);
}


CRVL2DLine2 *RVLFindNearest2DLine(CRVLMPtrChain *p2DLineArray,
								int *iU,
								int MinVectorLength)
{
	CRVL2DLine2 *pNearest2DLine = NULL;
	int MinDistance = 0;

	CRVL2DLine2 *p2DLine;
	int dist;

	p2DLineArray->Start();

	while(p2DLineArray->m_pNext)
	{
		p2DLine = (CRVL2DLine2 *)(p2DLineArray->GetNext());

		if(p2DLine->m_leniU < MinVectorLength)
			continue;

		dist = p2DLine->Distance(iU);

		if(pNearest2DLine == NULL || dist < MinDistance)
		{
			pNearest2DLine = p2DLine;
			MinDistance = dist;
		}
	}

	return pNearest2DLine;	
}

CRVL2DLine2 *RVLFindNearest2DLine(	CRVLObject2 **LineArray,
									int nLines,
									int *iU,
									int MinVectorLength)
{
	CRVL2DLine2 *pNearest2DLine = NULL;
	int MinDistance = 0;

	CRVL2DLine2 **p2DLineArrayEnd = (CRVL2DLine2 **)(LineArray + nLines);

	CRVL2DLine2 *p2DLine;
	int dist;
	CRVL2DLine2 **pp2DLine;

	for(pp2DLine = (CRVL2DLine2 **)LineArray; pp2DLine < p2DLineArrayEnd; pp2DLine++)
	{
		p2DLine = *pp2DLine;

		if(p2DLine->m_leniU < MinVectorLength)
			continue;

		dist = p2DLine->Distance(iU);

		if(pNearest2DLine == NULL || dist < MinDistance)
		{
			pNearest2DLine = p2DLine;
			MinDistance = dist;
		}
	}

	return pNearest2DLine;	
}

void RVLFit2DLine(RVL2DMOMENTS *pData, 
				  double &cs, double &sn, double &d, 
				  double &err)
{
	double fn = (double)(pData->n);

	double Cxx = pData->S2[0] - pData->S[0] * pData->S[0] / fn;
	double Cxy = pData->S2[1] - pData->S[0] * pData->S[1] / fn;
	double Cyy = pData->S2[3] - pData->S[1] * pData->S[1] / fn;

	if(fabs(Cxy) > APPROX_ZERO)
	{
		double k1 = Cxx - Cyy;
		double k2 = Cxy * Cxy;

		double dis = k1 * k1 + 4.0 * k2;

		double k3 = 0.5 * (k1 - sqrt(dis));

		double k4 = sqrt(k2 + k3 * k3);

		cs = Cxy / k4;
		sn = k3 / k4;
		d = (sn * pData->S[0] + cs * pData->S[1]) / fn;

		err = (k3 + Cyy) / fn;

		err = (err > 0.0 ? sqrt(err) : 0.0);
	}
	else
	{
		if(Cxx > Cyy)
		{
			cs = 1.0;
			sn = 0.0;
			d = pData->S[1] / fn;

			err = 0.0;
		}
		else
		{
			cs = 0.0;
			sn = 1.0;
			d = pData->S[0] / fn;

			err = 0.0;
		}
	}
}



