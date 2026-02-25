#include "RVLCore.h"
#include "RVLCropRowDetector.h"

//#define RVLCRD_DEBUG

CRVLCropRowDetector::CRVLCropRowDetector(void)
{
	m_mind = 8;
	m_ndOctaves = 3;
	m_ndSamplesPerOctave = 70;
	m_a0 = 1;
	m_b0 = 3;
	m_d0 = 8;
	m_lambdac = 0.5;
	m_lambdad = 0.2;
	m_maxD = 1.0;

	m_c = NULL;
	m_id = NULL;
	m_DPData = NULL;
	m_bestScore = NULL;
}

CRVLCropRowDetector::~CRVLCropRowDetector(void)
{
	if(m_c)
		delete[] m_c;

	if(m_id)
		delete[] m_id;

	if(m_DPData)
		delete[] m_DPData;

	if(m_bestScore)
		delete[] m_bestScore;
}

void CRVLCropRowDetector::CreateParamList(CRVLMem * pMem)
{
	m_ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	m_ParamList.Init();

	pParamData = m_ParamList.AddParam("CRD.a0", RVLPARAM_TYPE_INT, &m_a0);
	pParamData = m_ParamList.AddParam("CRD.b0", RVLPARAM_TYPE_INT, &m_b0);
	pParamData = m_ParamList.AddParam("CRD.d0", RVLPARAM_TYPE_INT, &m_d0);
	pParamData = m_ParamList.AddParam("CRD.mind", RVLPARAM_TYPE_INT, &m_mind);
	pParamData = m_ParamList.AddParam("CRD.ndOctaves", RVLPARAM_TYPE_INT, &m_ndOctaves);
	pParamData = m_ParamList.AddParam("CRD.ndSamplesPerOctave", RVLPARAM_TYPE_INT, &m_ndSamplesPerOctave);
	pParamData = m_ParamList.AddParam("CRD.lambdac", RVLPARAM_TYPE_DOUBLE, &m_lambdac);
	pParamData = m_ParamList.AddParam("CRD.lambdad", RVLPARAM_TYPE_DOUBLE, &m_lambdad);
	pParamData = m_ParamList.AddParam("CRD.maxD", RVLPARAM_TYPE_DOUBLE, &m_maxD);
}

void CRVLCropRowDetector::Init(int h)
{
	m_h = h;

	if(m_c)
		delete[] m_c;

	m_c = new int[h];

	if(m_id)
		delete[] m_id;

	m_id = new int[h];

	if(m_DPData)
		delete[] m_DPData;

	m_nd = m_ndOctaves * m_ndSamplesPerOctave + 1;

	m_dstep = pow(2.0, 1.0 / (double)m_ndSamplesPerOctave);

	m_nc = (int)floor((double)m_mind * pow(m_dstep, m_nd)) + 1;

	m_DPData = new RVLCRD_DPDATA[m_h * m_nd * m_nc];

	if(m_bestScore)
		delete[] m_bestScore;

	m_bestScore = new double[h];
}

void CRVLCropRowDetector::Apply(unsigned char * I,
								int w)
{
	double fa0 = (double)m_a0;
	double fb0 = (double)m_b0;
	double fd0 = (double)m_d0;

	double halfw = (double)(w / 2);

	int n = m_nc * m_nd;

	// assign score to each pair (c,d) for each image row

	int *II_ = new int[w + 1];

	int *II = II_ + 1;

	II[-1] = 0;

	unsigned char *I_ = I;	

	int *crange_ = new int[m_nd];

	RVLCRD_DPDATA *DP_ = m_DPData;

	int u, v;
	int id;
	int crange;
	int kStart, kEnd;
	int a, b, c;
	double d;
	double halfa, halfb, halfd;
	double fu, fub;
	double scale;
	int u0, u1, u2, u3;
	double Sa, Sb;
	int na, nb;
	int k;
	double score;
	int bestid, bestc;
	double fTmp;
	double bestScore;
	RVLCRD_DPDATA *DP__, *pDP;
	
	for(v = 0; v < m_h; v++, I_ += w, DP_ += n)
	{
		// integral image of the row

		II[0] = (int)(I_[0]);

		for(u = 1; u < w; u++)
			II[u] = II[u - 1] + I_[u];

		// convolutions

#ifdef RVLCRD_DEBUG
		int debug = 1;
#endif

		bestScore = 0.0;

		d = (double)m_mind;

		DP__ = DP_;

		for(id = 0; id < m_nd; id++, d *= m_dstep, DP__ += m_nc)
		{
#ifdef RVLCRD_DEBUG
			if(d > 10.0 * debug)
				debug++;
#endif
			crange = (int)floor(0.5 * d);

			crange_[id] = crange;

			scale = d / fd0;
			fTmp = floor(scale * fa0 + 0.5);
			a = (int)fTmp;
			halfa = 0.5 * fTmp;
			fTmp = floor(scale * fb0 + 0.5);
			b = (int)fTmp;
			halfb = 0.5 * fTmp;
			halfd = 0.5 * d;
			fub =  halfd - halfb;

			pDP = DP__ + m_nc / 2 - crange;

			for(c = -crange; c < crange; c++, pDP++)
			{
				kStart = (int)floor((-(double)(w / 2 + c) + halfa) / d);
				kEnd = (int)floor(((double)(w / 2 - c) + halfa) / d);

				fu = (double)c + (double)kStart * d + halfw;

				u0 = DOUBLE2INT(fu - halfa);

				u1 = u0 + a - 1;

				if(u1 >= 0)
				{
					Sa = II[u1];
					na = u1;
				}
				else
				{
					Sa = 0;
					na = 0;
				}

				u2 = DOUBLE2INT(fu + fub);

				u3 = u2 + b - 1;

				if(u2 < 0)
					u2 = 0;				

				if(u3 >= 0)
				{
					Sb = (II[u3] - II[u2 - 1]);
					nb = (u3 - u2 + 1);
				}
				else
				{
					Sb = 0;
					nb = 0;
				}

				fu += d;

				for(k = kStart + 1; k < kEnd; k++, fu += d)
				{
					u0 = DOUBLE2INT(fu - halfa);

					u1 = u0 + a - 1;

					Sa += (II[u1] - II[u0 - 1]);
					na += (u1 - u0 + 1);

					u2 = DOUBLE2INT(fu + fub);

					u3 = u2 + b - 1;

					Sb += (II[u3] - II[u2 - 1]);
					nb += (u3 - u2 + 1);
				}

				u0 = DOUBLE2INT(fu - halfa);

				u1 = u0 + a - 1;

				if(u1 >= w)
					u1 = w - 1;

				Sa += (II[u1] - II[u0 - 1]);
				na += (u1 - u0 + 1);

				u2 = DOUBLE2INT(fu + fub);

				if(u2 < w)
				{
					u3 = u2 + b - 1;

					if(u3 >= w)
						u3 = w - 1;

					Sb += (II[u3] - II[u2 - 1]);
					nb += (u3 - u2 + 1);
				}

				score = (double)(nb * Sa - na * Sb) / (double)(na + nb);

				pDP->D = score;

				if(score > bestScore)
				{
					bestScore = score;
					bestc = c;
					bestid = id;
				}
			}	// for(c = -crange; c < crange; c++)
		}	// for(id = 0; id < m_nd; id++, d *= m_dstep)

		m_c[v] = bestc;
		m_id[v] = bestid;
		m_bestScore[v] = bestScore;
	}	// for(v = 0; v < h; v++, I_ += w)

	delete[] II_;

	// dynamic programing

	// m_DPData is a 3D data structure. It has h blocks, each block having m_nd rows and m_nc columns.
	// Each block corresponds to an image row.

	DP_ = m_DPData;	// DP_ - ptr. to the first data in a block

#ifdef RVLCRD_L1
	double BV;
	RVLCRD_DPDATA *pDPPrev;	
#else
	int iTmp = RVLMAX(m_nc, m_nd) + 1;

	int *v_ = new int[iTmp];
	double *z = new double[iTmp];

	double maxz;
	double s;
#endif

	double Dnrm;

	for(v = 0; v < m_h; v++, DP_ += n)
	{
#ifdef RVLCRD_DEBUG
		if(v == 236)
			int debug = 0;
#endif

		Dnrm = m_bestScore[v];

		DP__ = DP_;		// DP__ - ptr. to the first data in a block row

		for(id = 0; id < m_nd; id++, DP__ += m_nc)
		{
			crange = crange_[id];

			pDP = DP__ + m_nc / 2 - crange;

			for(c = -crange; c < crange; c++, pDP++)
			{
				pDP->B = 1.0 - pDP->D / Dnrm;

				if(pDP->B > m_maxD)
					pDP->B = m_maxD;
				
				if(v > 0)
					pDP->B += (pDP - n)->minBV;
			}
		}

		if(v < m_h - 1)
		{
			DP__ = DP_;

#ifdef RVLCRD_L1
			for(id = 0; id < m_nd; id++, DP__ += m_nc)
			{
				crange = crange_[id];

				pDP = DP__ + m_nc / 2 - crange;

				pDP->minBV = pDP->B;
				pDP->c = -crange;
				pDP->id = id;

				pDPPrev = pDP;

				pDP++;

				for(c = -crange + 1; c < crange; c++, pDP++)
				{
					pDP->minBV = pDP->B;

					BV = pDPPrev->minBV + m_lambdac;

					if(pDP->minBV <= BV)
					{
						pDP->c = c;
						pDP->id = id;
					}
					else
					{
						pDP->minBV = BV;
						pDP->c = pDPPrev->c;
						pDP->id = pDPPrev->id;
					}

					pDPPrev = pDP;
				}

				for(; c < m_nc; c++, pDP++)
				{
					pDP->minBV = pDPPrev->minBV + m_lambdac;
					pDP->c = pDPPrev->c;
					pDP->id = pDPPrev->id;

					pDPPrev = pDP;
				}

				pDPPrev = DP__ + m_nc / 2 + crange - 1;

				pDP = pDPPrev - 1;

				for(c = crange - 2; c >= -crange; c--, pDP--)
				{
					BV = pDPPrev->minBV + m_lambdac;

					if(pDP->minBV > BV)
					{
						pDP->minBV = BV;
						pDP->c = pDPPrev->c;
					}

					pDPPrev = pDP;
				}

				for(; c >= -m_nc/2; c--, pDP--)
				{
					pDP->minBV = pDPPrev->minBV + m_lambdac;
					pDP->c = pDPPrev->c;
					pDP->id = pDPPrev->id;

					pDPPrev = pDP;
				}
			}	// for(id = 0; id < m_nd; id++)

			DP__ = DP_;

			for(c = 0; c < m_nc; c++, DP__++)
			{
				pDPPrev = DP__;

				pDP = pDPPrev + m_nc;

				for(id = 1; id < m_nd; id++, pDP += m_nc)
				{
					BV = pDPPrev->minBV + m_lambdad;

					if(pDP->minBV > BV)
					{
						pDP->minBV = BV;
						pDP->c = pDPPrev->c;
						pDP->id = pDPPrev->id;
					}

					pDPPrev = pDP;
				}

				pDPPrev = DP__ + (m_nd - 1) * m_nc;

				pDP = pDPPrev - m_nc;

				for(id = m_nd - 2; id >= 0; id--, pDP -= m_nc)
				{
					BV = pDPPrev->minBV + m_lambdad;

					if(pDP->minBV > BV)
					{
						pDP->minBV = BV;
						pDP->c = pDPPrev->c;
						pDP->id = pDPPrev->id;
					}

					pDPPrev = pDP;
				}
			}	// for(c = 0; c < m_nc; c++, DP_++)	
#else
			maxz = (1.0 + m_lambdac * (double)(m_nc * m_nc)) / (2.0 * m_lambdac);

			for(id = 0; id < m_nd; id++, DP__ += m_nc)
			{
#ifdef RVLCRD_DEBUG
				if(id == 186)
					int debug = 0;
#endif
				crange = crange_[id];

				// felzenszwalb_TC12	RVL
				// ============================================
				// k					k
				// v					v_
				// z					z
				// q					c
				// s					s
				// f(q)					pDP->B
				// f(v(k))				DP__[v_[k]+m_nc/2].B

				k = 0;
				v_[0] = -crange;
				z[0] = -maxz;
				z[1] = maxz;

				pDP = DP__ + m_nc / 2 - crange + 1;

				for(c = -crange + 1; c < crange; c++, pDP++)
				{
					while(true)
					{
						s = ((pDP->B + m_lambdac * (double)(c * c)) - (DP__[v_[k] + m_nc / 2].B + m_lambdac * (double)(v_[k] * v_[k]))) / 
							(2.0 * m_lambdac * (double)(c - v_[k]));

						if(s <= z[k])
							k--;
						else
							break;
					}

					k++;

					v_[k] = c;
					z[k] = s;
					z[k + 1] = maxz;
				}

				k = 0;

				pDP = DP__;

				for(c = -m_nc / 2; c < m_nc / 2; c++, pDP++)
				{
					while(z[k + 1] < (double)c)
						k++;

					iTmp = (c - v_[k]);
					pDP->minBV = DP__[v_[k] + m_nc / 2].B + m_lambdac * (double)(iTmp * iTmp);
					pDP->c = v_[k];
					pDP->id = id;
				}
			}	// for(id = 0; id < m_nd; id++)

			maxz = (1.0 + m_lambdad * (double)(m_nd * m_nd)) / (2.0 * m_lambdad);

			DP__ = DP_;			

			for(c = 0; c < m_nc; c++, DP__++)
			{
#ifdef RVLCRD_DEBUG
				if(c == m_nc / 2)
					int debug = 0;
#endif
				k = 0;
				v_[0] = 0;
				z[0] = -maxz;
				z[1] = maxz;

				pDP = DP__ + m_nc;

				for(id = 1; id < m_nd; id++, pDP += m_nc)
				{
					while(true)
					{
						s = ((pDP->minBV + m_lambdad * (double)(id * id)) - (DP__[m_nc * v_[k]].minBV + m_lambdad * (double)(v_[k] * v_[k]))) / 
							(2.0 * m_lambdad * (double)(id - v_[k]));

						if(s <= z[k])
							k--;
						else
							break;
					}

					k++;

					v_[k] = id;
					z[k] = s;
					z[k + 1] = maxz;
				}

				k = 0;

				pDP = DP__;

				for(id = 0; id < m_nd; id++, pDP += m_nc)
				{
					while(z[k + 1] < (double)id)
						k++;

					iTmp = (id - v_[k]);
					pDP->minBV = DP__[m_nc * v_[k]].minBV + (m_lambdad * (double)(iTmp * iTmp));
					pDP->c = DP__[m_nc * v_[k]].c;
					pDP->id = v_[k];					
				}
			}	//	for(c = 0; c < m_nc; c++)
#endif
		}	// if(v < m_h - 1)

#ifdef RVLCRD_DEBUG
		pDP = DP_;

		for(id = 0; id < m_nd; id++)
			for(c = 0; c < m_nc; c++, pDP++)
				if(pDP->minBV < 0)
					int debug = 0;
#endif
	}	// for(v = 0; v < m_h - 1; v++, DP_ += n)

#ifndef RVLCRD_L1
	delete[] v_;
	delete[] z;
#endif

	DP_ = m_DPData + (m_h - 1) * n;

	DP__ = DP_;

	v = m_h - 1;

	RVLCRD_DPDATA *pBestNode = DP__ + m_nc / 2 - crange_[0];

	for(id = 0; id < m_nd; id++, DP__ += m_nc)
	{
		crange = crange_[id];

		pDP = DP__ + m_nc / 2 - crange;

		for(c = -crange; c < crange; c++, pDP++)
			if(pDP->B < pBestNode->B)
			{
				pBestNode = pDP;
				bestc = c;
				bestid = id;
			}
	}

	m_c[v] = bestc;
	m_id[v] = bestid;

	for(v = m_h - 2; v >= 0; v--)
	{
		pDP = pBestNode - n;

		DP_ -= n;

		pBestNode = DP_ + pDP->id * m_nc + m_nc / 2 + pDP->c;

		m_c[v] = pDP->c;
		m_id[v] = pDP->id;
	}

	delete[] crange_;
}

void CRVLCropRowDetector::Display(unsigned char * DisplayImage, int w)
{
	double halfw = (double)(w / 2);

	int u, v;
	int kStart, kEnd, k;
	double c, d;
	unsigned char *pPix;

	for(v = 0; v < m_h; v++)
	{
		c = (double)(m_c[v]);
		d = (double)m_mind * pow(m_dstep, (double)(m_id[v]));

		kStart = (int)floor(-(halfw + c) / d) + 1;
		kEnd = (int)floor((halfw - c) / d);		
		
		for(k = kStart; k <= kEnd; k++)
		{
			u = DOUBLE2INT(c + (double)k * d + halfw);

			pPix = DisplayImage + 3 * (u + v * w);

			*(pPix++) = 0;
			*(pPix++) = 255;
			pPix = 0;
		}
	}
}
