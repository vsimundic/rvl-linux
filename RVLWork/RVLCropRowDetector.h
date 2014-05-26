#pragma once

//#define RVLCRD_L1

struct RVLCRD_DPDATA
{
	int c;
	int id;
	double D;
	double minBV;
	double B;
};

class CRVLCropRowDetector
{
public:
	CRVLCropRowDetector(void);
	virtual ~CRVLCropRowDetector(void);
	void Apply(unsigned char * I, int w);
	void Init(int h);
	void Display(unsigned char * DisplayImage, int w);
	void CreateParamList(CRVLMem * pMem);

public:
	int m_mind;
	int m_ndOctaves;
	int m_ndSamplesPerOctave;
	int m_h;
	int *m_c;
	int *m_id;
	RVLCRD_DPDATA *m_DPData;
	double *m_bestScore;
	int m_a0;
	int m_b0;
	int m_d0;
	CRVLParameterList m_ParamList;
	double m_lambdac;
	double m_lambdad;
	double m_maxD;

private:
	double m_dstep;
	int m_nc;
	int m_nd;
};
