#include <flann\flann.hpp>
#include <cstdlib>
#include <ctime>
#include "RVLCore.h"
#include "RVLPCS.h"
#include "RVLRLM.h"
#include "RVLPSuLMBuilder.h"
#include "Include\RVLPSuLMIndexing.h"
#include <math.h>
#include <iostream>
#include <fstream>
using namespace std;
//#ifdef RVLFLANN
//#include <flann\io\hdf5.h>
//#endif


//using namespace flann;
//using namespace cv;


CRVLPSuLMIndexing::CRVLPSuLMIndexing(void)
{
	m_dBinSize = 150.0f;		// mm
	m_dRange = 15000.0f;	// mm	

	m_pIndex = NULL;
	m_PCG = NULL;
	m_PCGBuff = NULL;
	m_FeatureArray = NULL;
	m_IndexMem = NULL;
	m_MIndicatorArray = NULL;
	m_Index = NULL;
	m_MatchMem = NULL;
}

CRVLPSuLMIndexing::~CRVLPSuLMIndexing(void)
{
	if (m_pIndex)
		delete m_pIndex;

	if (m_PCG)
		delete[] m_PCG;

	if (m_PCGBuff)
		delete[] m_PCGBuff;

	if (m_FeatureArray)
		delete[] m_FeatureArray;

	if (m_IndexMem)
		delete[] m_IndexMem;

	if (m_MIndicatorArray)
		delete[] m_MIndicatorArray;

	if (m_Index)
		delete[] m_Index;

	if (m_MatchMem)
		delete[] m_MatchMem;
}

void CRVLPSuLMIndexing::Init()
{
	/*
	// primjer upotrebe FLANN-a za radiusSearch
	int i, j, nn=3; // nn- broj k najblizih susjeda, ne treba za radiusSearch
	int n = 500;	// broj znacajki
	int m = 2;		// dimenzije znacajke
	int r = 100;	// broj rjesenja
	float *M = new float[n * m]; //matrica znacajki
	int *R = new int[r];		 //matrica rjesenja
	float *D = new float[r];	 //matrica s udaljenostima odgovarajuceg rjesenja od query-ja
	float Q[2]; //query (u opcem slucaju m elemenata)
	
	flann::Matrix<float> M_(M, n, m); //paziti na redoslijed: broj znacajki, dimenzionalnost
	flann::Matrix<int> R_(R, 1, r);   //paziti na redoslijed: 1, broj rjesenja 
	flann::Matrix<float> D_(D, 1, r); //paziti na redoslijed: 1, broj rjesenja 
	flann::Matrix<float> Q_(Q, 1, m);

	//popunjavanje matrice M slucajnim brojevima od 0 do 1:
	srand(static_cast <unsigned> (time(0)));
	for (i = 0; i < n; i++){
		for (j = 0; j < m; j++){
			M[i*m+j] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		}
	}
	
	//Query; moze i u for petlju i random dodijeliti koordinate toèaka (toèke)
	Q[0] = 0.0;
	Q[1] = 0.0;

	//kreiranje objekta index koji ce pridodijeliti indexe matrici
	//KMeansIndexParams je davao najbolje rezultate pri radiusSearchu
	flann::Index<flann::L2<float>> index(M_, flann::KMeansIndexParams());
	//flann::Index<flann::L2<float>> index(M_, flann::KDTreeIndexParams(16));
	//flann::Index<flann::L2<float>> index(M_, flann::LinearIndexParams());
	
	//kreiranje indeksa:
	index.buildIndex();
	
	//pretraga:
	index.radiusSearch(Q_, R_, D_, 0.2f, flann::SearchParams(64));
	//index.knnSearch(Q_, R_, D_, nn, flann::SearchParams(32));
	
	//testiranje:
	FILE *ulaz;
	FILE *indeksi;
	ulaz = fopen("ulaz.txt", "w");
	indeksi = fopen("indexi.txt", "w");
	for (i = 0; i < n; i++){
		for (j = 0; j < m; j++){
			fprintf(ulaz, "%d. %.2f\n",i, M[i*m + j]); //ispis znacajki
		}
	}

	for (i = 0; i < r; i++){
		if (R[i] >= 0)
			fprintf(indeksi, "%d. %d  %.5f %.5f\n", i, R[i], D[i], M[2 * R[i]] * M[2 * R[i]] + M[2 * R[i]+1] * M[2 * R[i]+1]);
	}
	int nP = 0, nTP=0, nT = 0;
	for (j = 0; j < r; j++){
		if (R[j] >= 0) nP++;
	}
	for (i = 0; i < n; i++){
		if ((M[2*i] * M[2*i] + M[2*i + 1] * M[2*i + 1]) < 0.2){ //kvadrirana euklidska udaljenost
			nT++;
			for (j = 0; j < r; j++){
				if (R[j] >= 0){
					if (R[j] == i) nTP++;
				}
				
			}
		}
	}
	fprintf(ulaz, " R sadrzi %d indeksa.\n Pronadeno je %d indeksa koji odgovaraju, a nisu u R.", nP, nT - nTP);
	fprintf(ulaz, " Flann je vratio %d pogresnih elemenata.", nP-nTP);
	fclose(ulaz);
	fclose(indeksi);

	delete[] M;
	delete[] R;
	delete[] D;
	*/

	m_ndBins = (int)(ceil(m_dRange / m_dBinSize));

	m_dOffset = (float)m_ndBins;

	m_ndBins *= 2;

	m_Mem.Create(10000000);
}

void CRVLPSuLMIndexing::ResetIndicatorAndPCGList()
{
	RVLQLIST *pIndicatorList = &m_IndicatorList; 
	RVLQLIST_INIT(pIndicatorList);
	RVLQLIST *pPCGList = &m_PCGList;
	RVLQLIST_INIT(pPCGList);
}

void CRVLPSuLMIndexing::GetIndicators(
	CRVLPSuLM * pPSuLM,
	bool bModel)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)(m_vpBuilder);

	CRVLMem *pMem = (bModel ? pBuilder->m_pMem0 : pBuilder->m_pMem);

	RVLQLIST *pPCGList = &m_PCGList;

	RVLQLIST *pIndicatorList = &m_IndicatorList;

#ifdef RVLPSULM_INDEXING_INDICATORS_DEBUG
	FILE *fpDebugInd = NULL;
	int nIndicators0;
	bool bDebug = false;
#endif
	
	if (bModel)
	{
#ifdef RVLPSULM_INDEXING_MINDICATORS_DEBUG
		char str[200];
		sprintf(str, "C:\\RVL\\Debug\\M%03dIndicators.txt", pPSuLM->m_Index);
		fpDebugInd = fopen(str, "w");
		nIndicators0 = m_nMIndicators;
		bDebug = true;
#endif
		m_nPCGs = m_nMPCGs;
	}		
	else
	{
#ifdef RVLPSULM_INDEXING_SINDICATORS_DEBUG
		fpDebugInd = fopen("C:\\RVL\\Debug\\SIndicators.txt", "w");
		nIndicators0 = 0;
		bDebug = true;
#endif
		m_nPCGs = 0;
		RVLQLIST_INIT(pPCGList);
		RVLQLIST_INIT(pIndicatorList);
	}		

	int i, j, k, l, n;
	//n = pPSuLM->m_n3DSurfaces;

	n = 15;

	m_nFeatures = n;

	RVLPSULM_PCG **PCGLT;

	if (!bModel)
	{
		PCGLT = new RVLPSULM_PCG *[n * n * n];

		memset(PCGLT, 0, n * n * n * sizeof(RVLPSULM_PCG *));
	}

	double eNThr = cos(30.0 * DEG2RAD);

	int *FeatureArray = new int[n];

	int iFeature = 0;

	int iFeature_, iFeature__;
	CRVL3DSurface2 *pSurf, *pSurf__;
	double *N, *N__;
	double eN, ed;

	for (iFeature_ = 0; iFeature_ < pPSuLM->m_n3DSurfacesTotal && iFeature < n; iFeature_++)
	{
		pSurf = pPSuLM->m_3DSurfaceArray[iFeature_];

		N = pSurf->m_N;

		for (iFeature__ = 0; iFeature__ < iFeature; iFeature__++)
		{
			pSurf__ = pPSuLM->m_3DSurfaceArray[FeatureArray[iFeature__]];

			N__ = pSurf__->m_N;

			eN = RVLDOTPRODUCT3(N, N__);

			if (eN < eNThr)
				continue;

			ed = pSurf__->m_d - pSurf->m_d;

			if (RVLABS(ed) <= m_dBinSize)
				break;
		}

		if (iFeature__ == iFeature)
		{
			FeatureArray[iFeature] = iFeature_;

			iFeature++;
		}
	}

	if (iFeature < n)
		n = iFeature;
	
	int n_ = (!bModel ? 4 * n - 10 : 0);
	int m = pPSuLM->m_n3DLines;

	double RCG[9];
	double *XGC = RCG; 
	double *YGC = RCG + 3;
	double *ZGC = RCG + 6;

	m_nIndicators = 0;

	RVLPSULM_PCG *pPCG;
	RVLPSULM_INDICATOR *pIndicator;
	double fTmp;

	CvMat *N_ = cvCreateMat(3, 3, CV_64FC1);

	N = N_->data.db;

	CvMat *b_ = cvCreateMat(3, 1, CV_64FC1);

	double *b = b_->data.db;

	CvMat *tGC_ = cvCreateMatHeader(3, 1, CV_64FC1);
	
	int r, r0, n0, m0, r1, n1, m1, r2, n2, m2, r3, n3_;
	FILE *file;
	file = fopen("bModel.txt", "w");

	int maxnIndicators = (bModel ? n * n * n * n : 1000);

	int PCGID;
	bool bCreatePCG;
	
	for (r = 0; r <= n_ && m_nIndicators < maxnIndicators; r++){
		if (bModel) n0 = n-1;
		else
		{
			r0 = r;
			n0 = n - 1;
			if (n0 > r0)
				n0 = r0;
		}
		m0 = 0;
		//FFF PCG
		for (i = m0; i <= n0; i++){
			int i_;
			i_ = i;
			//CRVL3DSurface2 *pSurf0 = pPSuLM->m_3DSurfaceArray[i_];
			CRVL3DSurface2 *pSurf0 = pPSuLM->m_3DSurfaceArray[FeatureArray[i_]];
			double *N0 = pSurf0->m_N;

			if (RVLABS(N0[2]) < COS45)
				continue;

			if (bModel) {
				n1 = n-1;
				m1 = 0;
			}
			else {
				r1 = r0 - i;
				//n1 = n - 2;
				n1 = n - 1;
				if (n1 > r1)
					n1 = r1;
				//m1 = i;
				m1 = 0;
			}


			// XGC <- N0
			RVLCOPY3VECTOR(N0, XGC);
				
				for (j = m1; j <= n1 && m_nIndicators < maxnIndicators; j++)
				{
					if (i == j) 
						continue;
				
				int j_;
				if (bModel){
					j_=j;
					n2 = n - 1;
					m2 = 0;
				}
				else{
					//j_=j+1;
					j_ = j;
					r2 = r1 - j;
					//n2 = n - 3;
					n2 = n - 1;
					if (n2 > r2)
						n2 = r2;
					//m2 = j;
					m2 = 0;
				}
				//CRVL3DSurface2 *pSurf1 = pPSuLM->m_3DSurfaceArray[j_];
				CRVL3DSurface2 *pSurf1 = pPSuLM->m_3DSurfaceArray[FeatureArray[j_]];
				double *N1 = pSurf1->m_N;

				fTmp = RVLDOTPRODUCT3(N0, N1);
				// prvi element indikatora:
				double prvi = acos(RVLABS(fTmp));
				if (RVLABS(fTmp) >= 0.707107) continue;

				
				// ZGC <- UNIT(N0 x N1)
				RVLCROSSPRODUCT3(N0, N1, ZGC);
				RVLNORM3(ZGC, fTmp);

				//YGC <- ZGC x XGC
				RVLCROSSPRODUCT3(ZGC, XGC, YGC);

				for (k = m2; k <= n2 && m_nIndicators < maxnIndicators; k++)
				{
					if (j == k || i == k)
						continue;

					int k_;
					
					if (bModel){
						k_ = k;
						n3_ = n - 1;
					}
					else {
						k_ = k;
						//k_ = k+2;
						r3 = r2 - k;
						n3_ = (r3 < n && r3 != i_ && r3 != j_ && r3 != k_ ? 0 : -1);
					}

					//CRVL3DSurface2 *pSurf2 = pPSuLM->m_3DSurfaceArray[k_];
					CRVL3DSurface2 *pSurf2 = pPSuLM->m_3DSurfaceArray[FeatureArray[k_]];

					double *N2 = pSurf2->m_N;

					fTmp = RVLDOTPRODUCT3(N1, N2);
					// drugi element indikatora:
					double drugi = acos(RVLABS(fTmp));
					if (RVLABS(fTmp) >= 0.707107) continue; 

					fTmp = RVLDOTPRODUCT3(N2, N0);
					// treci element indikatora:
					double treci = acos(RVLABS(fTmp));
					if (RVLABS(fTmp) >= 0.707107) continue; 

					fTmp = RVLDOTPRODUCT3(N2, ZGC);
					if (fTmp < 0.0f) continue;

					if (bModel)
					{
						bCreatePCG = true;

						RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_PCG, pPCG);
					}
					else
					{
						PCGID = (i_ * n + j_) * n + k_;

						pPCG = PCGLT[PCGID];

						if (bCreatePCG = (pPCG == NULL))
						{
							RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_PCG, pPCG);

							PCGLT[PCGID] = pPCG;
						}
					}

					double *RGC = pPCG->Pose.m_Rot;
					double *tGC = pPCG->Pose.m_X;

					if (bCreatePCG)
					{
						RVLCOPY3VECTOR(N0, N);
						N__ = N + 3;
						RVLCOPY3VECTOR(N1, N__);
						N__ = N + 6;
						RVLCOPY3VECTOR(N2, N__);
						b[0] = pSurf0->m_d;
						b[1] = pSurf1->m_d;
						b[2] = pSurf2->m_d;

						tGC_->data.db = tGC;
						cvSolve(N_, b_, tGC_);

						RVLCOPYMX3X3T(RCG, RGC);
						RVLQLIST_ADD_ENTRY(pPCGList, pPCG);

						pPCG->Index = m_nPCGs;
						pPCG->iFeature[0] = FeatureArray[i_];
						pPCG->iFeature[1] = FeatureArray[j_];
						pPCG->iFeature[2] = FeatureArray[k_];
						pPCG->pPSuLM = pPSuLM;
						m_nPCGs++;
					}

					for (l = 0; l <= n3_; l++)
					{
						int l_;
						
						if (bModel){
							l_ = l;
							if (j == l || i == l || k == l) continue;
						}
						else
							l_ = r3;

//#ifdef RVLPSULM_INDEXING_DEBUG
//						if (i_ == 0 && j_ == 2 && k_ == 1 && l_ == 5)
//							int debug = 0;
//#endif

						//CRVL3DSurface2 *pSurf3 = pPSuLM->m_3DSurfaceArray[l_];
						CRVL3DSurface2 *pSurf3 = pPSuLM->m_3DSurfaceArray[FeatureArray[l_]];
						double *N3 = pSurf3->m_N;

						RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_INDICATOR, pIndicator)

						//prebacivanje prva tri elementa indikatora + normiranje: 15°=0.261799387799149 rad ~ 1
						pIndicator->m_Descriptor[0] = prvi / 0.261799387799149f;
						pIndicator->m_Descriptor[1] = drugi / 0.261799387799149f;
						pIndicator->m_Descriptor[2] = treci / 0.261799387799149f;

						//popunjavanje ostala èetiri mjesta deskriptora:
						float *N3G = pIndicator->m_Descriptor + 3;
						float *RHO3G = pIndicator->m_Descriptor + 6;


						RVLMULMX3X3VECT(RCG, N3, N3G);
						//normiranje: 2*sin(15°/2)= 0.08723877473 ~ 1
						N3G[0] = N3G[0] / 0.261052384440103f;
						N3G[1] = N3G[1] / 0.261052384440103f;
						N3G[2] = N3G[2] / 0.261052384440103f;
						*RHO3G = (pSurf3->m_d - RVLDOTPRODUCT3(N3, tGC)) / m_dBinSize;

						RVLQLIST_ADD_ENTRY(pIndicatorList, pIndicator);

						pIndicator->iPCG = pPCG->Index;
						pIndicator->iFeature = FeatureArray[l_];

						//if (pPCG->iFeature[0] == 0 && pPCG->iFeature[1] == 8 && pPCG->iFeature[2] == 29 && pIndicator->iFeature == 28)
						//	int debug = 0;

#ifdef RVLPSULM_INDEXING_INDICATORS_DEBUG 
						if (bDebug)
						{
							fprintf(fpDebugInd,
								"Indikator: %d\tPCG: %d\tF: %d %d %d %d\t",
								nIndicators0 + m_nIndicators, pIndicator->iPCG, pPCG->iFeature[0], pPCG->iFeature[1], pPCG->iFeature[2], pIndicator->iFeature);
							for (int j = 0; j < 7; j++){
								fprintf(fpDebugInd, "%.4f ", pIndicator->m_Descriptor[j]);
							}
							fprintf(fpDebugInd, "\n\n");
						}
#endif

						m_nIndicators++;

					}
					}
				}
		}
	}
	fclose(file);

	if (!bModel)
		delete[] PCGLT;

	delete[] FeatureArray;

#ifdef RVLPSULM_INDEXING_INDICATORS_DEBUG
	if (fpDebugInd)
		fclose(fpDebugInd);
#endif

	/*
	//FFL PCG
	for (i = 0; i < n; i++){
		CRVL3DSurface2 *pSurf0 = pPSuLM->m_3DSurfaceArray[i];
		double *N0 = pSurf0 -> m_N;

		// XGC <- N0
		RVLCOPY3VECTOR(N0, XGC)
		
		for (j = 0; j < n; j++){
			CRVL3DSurface2 *pSurf1 = pPSuLM->m_3DSurfaceArray[j];
			double *N1 = pSurf1 -> m_N;  
			
			if (i == j) continue;
			fTmp = RVLDOTPRODUCT3(N0, N1);
			// prvi element indikatora:
			double prvi = acos(RVLABS(fTmp));
			if (RVLABS(fTmp) <= 0.707107) continue; 

			// ZGC <- UNIT(N0 x N1)
			RVLCROSSPRODUCT3(N0, N1, ZGC)
			RVLNORM3(ZGC, fTmp) 

			//YGC <- ZGC x XGC
			RVLCROSSPRODUCT3(ZGC, XGC, YGC)
			
			for (k = 0; k < m; k++){
				CRVL3DLine2 *pLine2 = pPSuLM->m_3DLineArray[k]; 
				double *V2 = ((RVL3DLINE_EXTENDED_DATA*)(pLine2->m_pData)) -> V;
				fTmp = RVLDOTPRODUCT3(V2, ZGC);
				if (RVLABS(fTmp) <= 0.707107) continue;
				
				RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_PCG, pPCG)
				double *RGC = pPCG->Pose.m_Rot;
				double *tGC = pPCG->Pose.m_X;
				double *P12 = pLine2->m_X[0];
				double *P22 = pLine2->m_X[1];
				double U[3];
				RVLCROSSPRODUCT3(ZGC, V2, U)
				double fTmp2;
				RVLNORM3(U, fTmp2)

				double A11 = RVLDOTPRODUCT3(N0, V2);
				double A12 = RVLDOTPRODUCT3(N0, U);
				double A21 = RVLDOTPRODUCT3(N1, V2);
				double A22 = RVLDOTPRODUCT3(N1, U);

				//double A = { A11, A12, A21, A22 };
				double detA = (A11*A22) - (A12*A21);

				double RHO0 = pSurf0->m_d; 
				double RHO1 = pSurf1->m_d;
				fTmp = RVLDOTPRODUCT3(N0, P12);
				double B1 = RHO0 - fTmp; 
				fTmp = RVLDOTPRODUCT3(N1, P12);
				double B2 = RHO1 - fTmp; 
				double s = (A22*B1 - A12*B2) / detA;
				double d = (-A21*B1 + A11*B2) / detA;

				tGC[0] = P12[0] + s*V2[0] + d*U[0]; 
				tGC[1] = P12[1] + s*V2[1] + d*U[1];
				tGC[2] = P12[2] + s*V2[2] + d*U[2];

				RVLCOPYMX3X3T(RCG, RGC)
				RVLQLIST_ADD_ENTRY(pPCGList, pPCG)

				pPCG->Index = m_nPCGs;

				m_nPCGs++;

				// drugi element indikatora:
				double drugi;
				int predznak;
				fTmp = RVLDOTPRODUCT3(ZGC, V2);
				if (d >= 0) predznak = 1;// d=0?
				if (d < 0) predznak = -1;
				drugi = atan2(predznak*fTmp2, fTmp);

				// treci element indikatora:
				fTmp = RVLDOTPRODUCT3(XGC, U);
				double treci = d*fTmp;

				//cetvrti element indikatora:
				fTmp = RVLDOTPRODUCT3(YGC, U);
				double cetvrti = d*fTmp;

				for (l = 0; l < m; l++){
					RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_INDICATOR, pIndicator)
					
					//prebacivanje prva cetiri elementa indikatora:
					pIndicator->m_Descriptor[0] = prvi;
					pIndicator->m_Descriptor[1] = drugi;
					pIndicator->m_Descriptor[2] = treci;
					pIndicator->m_Descriptor[3] = cetvrti;

					//popunjavanje ostalih 6 mjesta deskriptora:
					float *P3G = pIndicator->m_Descriptor + 4;
					float *V3G = pIndicator->m_Descriptor + 7;
					
					CRVL3DLine2 *pLine3 = pPSuLM->m_3DLineArray[l];
					double *V3 = ((RVL3DLINE_EXTENDED_DATA*)(pLine3->m_pData))->V;
					double *P13 = pLine3->m_X[0];

					double razlika[3];
					RVLDIF3VECTORS(tGC, P13, razlika)
					double s = RVLDOTPRODUCT3(razlika, V3);

					double P3[3];
					P3[0] = P13[0] + s*V3[0];
					P3[1] = P13[1] + s*V3[1];
					P3[2] = P13[2] + s*V3[2];

					//konacno P3G:
					RVLDIF3VECTORS(P3, tGC, razlika)
					RVLMULMX3X3VECT(RCG, razlika, P3G);

					//konacno V3G:
					RVLMULMX3X3VECT(RCG, V3, V3G);

					RVLQLIST_ADD_ENTRY(pIndicatorList, pIndicator);

					pIndicator -> iPCG = pPCG->Index; 
					m_nIndicators++;
				}
			}
		}
	}*/	
}

void CRVLPSuLMIndexing::GenerateHypotheses()
{

#ifdef RVLPSULM_INDEXING_INDEXING_DEBUG 
	FILE *fpIndexingDebug = fopen("C:\\RVL\\Debug\\IndexingDebug.txt", "w");
#endif
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)(m_vpBuilder);

	int r;
	int *R;
	
#ifdef RVLPSULM_INDEXING_FLANN
	r = 10000;	// broj rjesenja
	
	float *D = new float[r];	//matrica s udaljenostima odgovarajuceg rjesenja od query-ja
	R = new int[r];
	flann::Matrix<int>R_(R, 1, r);
	flann::Matrix<float> D_(D, 1, r);
#else
	RVLARRAY_<int> MatchArray;
	MatchArray.Element = m_MatchMem;
	R = MatchArray.Element;
#endif

	// Izvaditi indikatore iz mIndicatorList i za svaki indikator (while petlja) pokrenuti radiusSearch
	// Svaki indikator je Query za radiusSearch

	CRVLMem *pMem = &m_Mem;

	//CRVLMem *pMem = pBuilder->m_pMem;

	m_EvidenceAccu.Reset();
	RVLQLIST *EvidenceAccu_ = m_EvidenceAccu.m_ListArray;
	RVLPSULM_INDICATOR_MATCH *pEvidenceAccuEntry;
	RVLPSULM_PCG *pMPCG, *pSPCG;
	RVLPSULM_PCG **SPCG = new RVLPSULM_PCG*[m_nPCGs];

	RVLPSULM_PCG *pPCG = (RVLPSULM_PCG *)(m_PCGList.pFirst);
	RVLPSULM_PCG **ppPCG = SPCG;

	while (pPCG)
	{
		*(ppPCG++) = pPCG;

		pPCG->Flags = 0x00;

		pPCG = (RVLPSULM_PCG *)(pPCG->pNext);
	}

	//char *WordLT;

	//AssignWordsToIndicators(SPCG, &WordLT);

	RVLPSULM_INDICATOR *pIndicator = (RVLPSULM_INDICATOR *)(m_IndicatorList.pFirst);

	RVLPSULM_INDICATOR **SIndicatorArray = new RVLPSULM_INDICATOR *[m_nIndicators];

	RVLPSULM_INDICATOR **ppIndicator = SIndicatorArray;

	int iSIndicator = 0;

	int i;
	int brojac_aktivnih = 0;
	int iMPCG;
	while (pIndicator)
	{
		*(ppIndicator++) = pIndicator;

#ifdef RVLPSULM_INDEXING_FLANN
		flann::Matrix<float> Q_(pIndicator->m_Descriptor, 1, 7);

		// Pomoæu radiusSearch za svaki indikator iz m_IndicatorList dobiti sve indikatore iz baze unutar zadanog radijusa:
		m_pIndex->radiusSearch(Q_ , R_, D_, 5.0f, flann::SearchParams(128));
#else
		Search(pIndicator, MatchArray);

		r = MatchArray.n;
#endif
		
#ifdef RVLPSULM_INDEXING_INDEXING_DEBUG 
		pSPCG = SPCG[pIndicator->iPCG];
		fprintf(fpIndexingDebug, "Q: %d %d %d %d (PCG%d)\t", pSPCG->iFeature[0], pSPCG->iFeature[1], pSPCG->iFeature[2], pIndicator->iFeature, pIndicator->iPCG);	
		for (i = 0; i < 7; i++){
			fprintf(fpIndexingDebug, "%.4f\t", pIndicator->m_Descriptor[i]);
		}
		fprintf(fpIndexingDebug, "\n\nR: ");
				
#endif

		// Za svaki vraæeni indeks vadi se iz m_iPCG indeks PCG-a, pomoæu tog indeksa se preko pokazivaèa u polju m_PCG dolazi do PCG-a:	
		for (i = 0; R[i] >= 0 && i < r; i++)
		{ 
			iMPCG = m_iPCG[R[i]];
			//dodavanje entry-a s indeksom indikatora scene
			RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_INDICATOR_MATCH, pEvidenceAccuEntry);
			RVLQLISTARRAY_ADD_ENTRY(EvidenceAccu_, iMPCG, pEvidenceAccuEntry);
			pEvidenceAccuEntry->iS = iSIndicator;
			pEvidenceAccuEntry->iM = R[i];

			//postavljanje flaga aktivnog PCG-a (ako nije aktivan)
			
			pMPCG = m_PCG[iMPCG];
			
#ifdef RVLPSULM_INDEXING_INDEXING_DEBUG 
			float E = 0.0f;
			
			float e;
			int j;

			for (j = 0; j < 7; j++)
			{
				e = pIndicator->m_Descriptor[j] - m_MIndicatorArray[R[i]].m_Descriptor[j];

				E += (e * e);
			}

			fprintf(fpIndexingDebug, "%d(%d.%d %d %d)%lf ", R[i], pMPCG->pPSuLM->m_Index, pMPCG->iFeature[0], pMPCG->iFeature[1], pMPCG->iFeature[2], sqrt(E));

#endif
			if (!(pMPCG->Flags & RVLPSULM_PCG_FLAG_ACTIVE))
			{
				pMPCG->Flags |= RVLPSULM_PCG_FLAG_ACTIVE;
				m_PCGBuff[brojac_aktivnih] = iMPCG;
				brojac_aktivnih++;
			}
		}

#ifdef RVLPSULM_INDEXING_INDEXING_DEBUG 
		fprintf(fpIndexingDebug, "\n#matches=%d\n\n", i);
#endif
		iSIndicator++;

		pIndicator = (RVLPSULM_INDICATOR *)(pIndicator->pNext);
	}

#ifdef RVLPSULM_INDEXING_FLANN
	delete[] D;
	delete[] R;
#endif

#ifdef RVLPSULM_INDEXING_INDEXING_DEBUG 
	fclose(fpIndexingDebug);
#endif

#ifdef RVLPSULM_INDEXING_DEBUG 
	FILE *fpDebug = fopen("C:\\RVL\\Debug\\IndexingMatchMatrix.txt", "w");
	
	int Histogram[30];

	memset(Histogram, 0, 30 * sizeof(int));

	int iPSuLMDebug = 5;
#endif

	CRVLQListArray SortedMatchList;

	SortedMatchList.m_Size = m_nFeatures;

	SortedMatchList.Init();
	SortedMatchList.Reset();

	int maxMatchingScore = 0;

	RVLQLIST *SortedMatchList_ = SortedMatchList.m_ListArray;
	RVLQLIST *pSortedMatchList;
	RVLPCG_MATCH *pSortedMatchListEntry;

	unsigned int *mSFeatures = new unsigned int[m_nPCGs];
	unsigned int *mMFeatures = new unsigned int[m_nPCGs];
	int *SPCGBuff = new int[m_nPCGs];
	int *piSPCG;
	int *SPCGBuffEnd;
	RVLPCG_MATCH *pMatch;
	int iSPCG;
	RVLPSULM_INDICATOR *pSIndicator;
	RVLPSULM_INDICATOR *pMIndicator;
	RVLQLIST *pEvidenceAccu;
	unsigned int mSFeature, mMFeature;
	int nSFeatures, nMFeatures, count;

	for (i = 0; i < brojac_aktivnih; i++)	// for all MPCGs
	{
		iMPCG = m_PCGBuff[i];
		
		pMPCG = m_PCG[iMPCG];

		pMPCG->Flags &= ~RVLPSULM_PCG_FLAG_ACTIVE;

		pEvidenceAccu = EvidenceAccu_ + iMPCG;

#ifdef RVLPSULM_INDEXING_DEBUG 
		if (pMPCG->pPSuLM->m_Index == iPSuLMDebug)
			fprintf(fpDebug, "M%d(%d.%d-%d-%d):\n\n", iMPCG, pMPCG->pPSuLM->m_Index, pMPCG->iFeature[0], pMPCG->iFeature[1], pMPCG->iFeature[2]);
#endif
		
		pEvidenceAccuEntry = (RVLPSULM_INDICATOR_MATCH *)(pEvidenceAccu->pFirst);
		piSPCG = SPCGBuff;
		while (pEvidenceAccuEntry)
		{
			pSIndicator = SIndicatorArray[pEvidenceAccuEntry->iS];
			pMIndicator = m_MIndicatorArray + pEvidenceAccuEntry->iM;

			iSPCG = pSIndicator->iPCG;

			//if (iMPCG == 98692 && iSPCG == 0)
			//	int debug = 0;
	
			pSPCG = SPCG[iSPCG];

#ifdef RVLPSULM_INDEXING_DEBUG 
			if (pMPCG->pPSuLM->m_Index == iPSuLMDebug)
				fprintf(fpDebug, "M%d-S%d.%d\n", pMIndicator->iFeature, iSPCG, pSIndicator->iFeature);
#endif

			mSFeature = (1 << pSIndicator->iFeature);
			mMFeature = (1 << pMIndicator->iFeature);

			if (!(pSPCG->Flags & RVLPSULM_PCG_FLAG_ACTIVE))
			{
				mSFeatures[iSPCG] = mSFeature;
				mMFeatures[iSPCG] = mMFeature;
				pSPCG->Flags |= RVLPSULM_PCG_FLAG_ACTIVE;
				*(piSPCG++) = iSPCG;
			}
			else
			{
				mSFeatures[iSPCG] |= mSFeature;
				mMFeatures[iSPCG] |= mMFeature;
			}

			pEvidenceAccuEntry = (RVLPSULM_INDICATOR_MATCH*)(pEvidenceAccuEntry->pNext);
		}

		SPCGBuffEnd = piSPCG;	

#ifdef RVLPSULM_INDEXING_DEBUG 
		if (pMPCG->pPSuLM->m_Index == iPSuLMDebug)
			fprintf(fpDebug, "\nScores:\n");
#endif
		
		for (piSPCG = SPCGBuff; piSPCG < SPCGBuffEnd; piSPCG++)
		{
			iSPCG = *piSPCG;

			pSPCG = SPCG[*piSPCG];

			nSFeatures = bitCount_LookUpTable[mSFeatures[iSPCG] & 0x0000ffff] + bitCount_LookUpTable[mSFeatures[iSPCG] >> 16];
			nMFeatures = bitCount_LookUpTable[mMFeatures[iSPCG] & 0x0000ffff] + bitCount_LookUpTable[mMFeatures[iSPCG] >> 16];
			count = RVLMIN(nSFeatures, nMFeatures);

#ifdef RVLPSULM_INDEXING_DEBUG 	
			if (pMPCG->pPSuLM->m_Index == iPSuLMDebug)
				fprintf(fpDebug, "S%d(%d-%d-%d) cost=%d\n", *piSPCG, pSPCG->iFeature[0], pSPCG->iFeature[1], pSPCG->iFeature[2], count);

			if (count > 29)
				count = 29;

			Histogram[count]++;
#endif

			if (count > maxMatchingScore)	// only for debugging purpose
				maxMatchingScore = count;	

			//if (count > 27)
			//	int debug = 0;			

			pSPCG->Flags &= ~RVLPSULM_PCG_FLAG_ACTIVE;			

			RVLMEM_ALLOC_STRUCT(pMem, RVLPCG_MATCH, pMatch);

			pMatch->pMPCG = pMPCG;
			pMatch->pSPCG = pSPCG;

			RVLQLISTARRAY_ADD_ENTRY(SortedMatchList_, count, pMatch);
		}

#ifdef RVLPSULM_INDEXING_DEBUG 
		if (pMPCG->pPSuLM->m_Index == iPSuLMDebug)
			fprintf(fpDebug, "\n\n");
#endif
	}	// for all MPCGs
		
#ifdef RVLPSULM_INDEXING_DEBUG 
	fclose(fpDebug);

	fpDebug = fopen("C:\\RVL\\Debug\\IndexingStatistics.txt", "w");
	
	for (i = 0; i < 30; i++)
		fprintf(fpDebug, "%d\n", Histogram[i]);
	
	fclose(fpDebug);
#endif

	delete[] SIndicatorArray;
	delete[] SPCG;
	delete[] mSFeatures;
	delete[] mMFeatures;
	delete[] SPCGBuff;

	// Create hypotheses from the highest ranked PCG matches

	double maxet = 400.0;	// mm
	double maxeq = cos(20.0 * DEG2RAD);

	double maxet2 = maxet * maxet;

	DWORD HypothesisCounter = 0;

	CRVLMPtrChain *pHypothesisList = &(pBuilder->m_HypothesisList);

	RVLPSULM_HYPOTHESIS *pHypothesis, *pHypothesis_;
	double RtmpInv[9];
	double ttmpInv[3];
	double *RPCGM;
	double *tPCGM;
	double *RPCGS;
	double *tPCGS;
	double *RSM, *tSM, *RSM_, *tSM_;
	double RSM__[9], tSM__[3], dt[3];
	double et2, eq;
	bool bNewHypothesis;

#ifdef RVLPSULM_INDEXING_DEBUG 
	FILE *fpDebugHyp = fopen("C:\\RVL\\Debug\\Hypothesis.txt", "w");
#endif

	for (i = m_nFeatures - 1; i > 0 && HypothesisCounter < 100; i--)
	{
		pSortedMatchList = SortedMatchList_ + i;
		pSortedMatchListEntry = (RVLPCG_MATCH*)(pSortedMatchList->pFirst);

		while (pSortedMatchListEntry)
		{
			pMPCG = pSortedMatchListEntry->pMPCG;
			pSPCG = pSortedMatchListEntry->pSPCG;

			RPCGM = pMPCG->Pose.m_Rot;
			tPCGM = pMPCG->Pose.m_X;

			RPCGS = pSPCG->Pose.m_Rot;
			tPCGS = pSPCG->Pose.m_X;

			RVLINVTRANSF3D(RPCGS, tPCGS, RtmpInv, ttmpInv);
			RVLCOMPTRANSF3D(RPCGM, tPCGM, RtmpInv, ttmpInv, RSM__, tSM__);

			bNewHypothesis = true;

			pHypothesisList->Start();

			while (pHypothesisList->m_pNext != NULL && bNewHypothesis)
			{
				pHypothesis_ = (RVLPSULM_HYPOTHESIS *)(pHypothesisList->GetNext());

				RSM_ = pHypothesis_->PoseSM.m_Rot;
				tSM_ = pHypothesis_->PoseSM.m_X;

				RVLDIF3VECTORS(tSM_, tSM__, dt);

				et2 = RVLDOTPRODUCT3(dt, dt);

				if (et2 > maxet2)
					continue;

				eq = RVLMULCOLCOL3(RSM_, RSM__, 0, 0);

				if (eq < maxeq)
					continue;

				eq = RVLMULCOLCOL3(RSM_, RSM__, 1, 1);

				if (eq < maxeq)
					continue;

				eq = RVLMULCOLCOL3(RSM_, RSM__, 2, 2);

				if (eq < maxeq)
					continue;

				bNewHypothesis = false;
			}

			if (bNewHypothesis)
			{
				RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_HYPOTHESIS, pHypothesis);

				RSM = pHypothesis->PoseSM.m_Rot;
				tSM = pHypothesis->PoseSM.m_X;

				RVLCOPYMX3X3(RSM__, RSM);
				RVLCOPY3VECTOR(tSM__, tSM);

				pHypothesis->PoseSM.UpdatePTRLL();

				pHypothesis->Index = HypothesisCounter;
				pHypothesis->pMPSuLM = pMPCG->pPSuLM;

				HypothesisCounter++;

				pBuilder->m_HypothesisList.Add(pHypothesis);

#ifdef RVLPSULM_INDEXING_DEBUG 
				fprintf(fpDebugHyp, "H%d\tP%d\tSPCG%d\tMPCG%d\tcost: %d \n\n", HypothesisCounter, pHypothesis->pMPSuLM->m_Index, pSPCG->Index, pMPCG->Index, i);
#endif
			}

			pSortedMatchListEntry = (RVLPCG_MATCH *)(pSortedMatchListEntry->pNext);
		}
	}

#ifdef RVLPSULM_INDEXING_DEBUG 
	fclose(fpDebugHyp);
#endif

	m_Mem.Clear();
}

void CRVLPSuLMIndexing::UpdateBase()
{
}

void CRVLPSuLMIndexing::CreateBase()
{	
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)(m_vpBuilder);

	ResetIndicatorAndPCGList();

	int iPSuLM = 0;

	m_nMPCGs = 0;
	m_nMIndicators = 0;

	CRVLPSuLM *pPSuLM;

	pBuilder->m_PSuLMList.Start();

	while (pBuilder->m_PSuLMList.m_pNext)
	{
		pPSuLM = (CRVLPSuLM *)(pBuilder->m_PSuLMList.GetNext());
		
		//if (iPSuLM == 2)
		//{			
		if ((pPSuLM->m_Index >= 0 && pPSuLM->m_Index <= 5) || (pPSuLM->m_Index >= 9 && pPSuLM->m_Index <= 16) ||
			(pPSuLM->m_Index >= 30 && pPSuLM->m_Index <= 32) || (pPSuLM->m_Index >= 36 && pPSuLM->m_Index <= 44))
		{
			GetIndicators(pPSuLM, true);  // otkomentirati
		
			//GetIndicators(pPSuLM);	// zakomentirati
			m_nMPCGs = m_nPCGs;
			m_nMIndicators += m_nIndicators;
			//	break;
		}

		if (iPSuLM >= 44)
			break;

		iPSuLM++;
	}


	m_EvidenceAccu.m_Size = m_nMPCGs;
	m_EvidenceAccu.Init();

	m_PCGBuff = new int[m_nMPCGs]; 

	// napraviti FLANN indeks od m_IndicatorList	

	// alocirati matricu znacajki za spremanje m_nIndicators znacajki 
	// alocirati m_iPCG
	int n = m_nMIndicators;	// broj znacajki (indikatora)
	int m = 7;		// dimenzije znacajke (indikatora)

	//n = 30000;	// only for debugging!!!

#ifdef RVLPSULM_INDEXING_FLANN
	m_FeatureArray = new float[n * m]; //matrica znacajki
	flann::Matrix<float> FeatureArray_(m_FeatureArray, n, m);	
#endif

	m_iPCG = new int[n];

	if (m_PCG)
		delete[] m_PCG;

	m_PCG = new RVLPSULM_PCG *[m_nMPCGs];
	
	// Stvoriti matricu pointera na PCG-ove m_PCG
	RVLPSULM_PCG *pPCG = (RVLPSULM_PCG *)(m_PCGList.pFirst);
	RVLPSULM_PCG **ppPCG = m_PCG;

	while (pPCG)
	{
		*(ppPCG++) = pPCG;

		pPCG->Flags = 0x00;

		pPCG = (RVLPSULM_PCG *)(pPCG->pNext);
	}

	int *piPCG = m_iPCG;
	//int *pPCG = m_PCG;

	if (m_MIndicatorArray)
		delete[] m_MIndicatorArray;

	m_MIndicatorArray = new RVLPSULM_INDICATOR [n];

	RVLPSULM_INDICATOR *pIndicator_ = m_MIndicatorArray;

	RVLPSULM_INDICATOR *pIndicator = (RVLPSULM_INDICATOR *)(m_IndicatorList.pFirst);

#ifdef RVLPSULM_INDEXING_FLANN
	int i = 0, j, k;
#endif

	while (pIndicator)
	{
#ifdef RVLPSULM_INDEXING_FLANN
		//popunjavanje matrice znacajki:
		// dodati znaèajku za svaki indikator
		j = 0;
		for (k = i; k < i + 7; k++){
			m_FeatureArray[k] = pIndicator->m_Descriptor[j];
			j++;
		}

		i += 7;
#endif

		// dodati iPCG od indikatora u m_iPCG
		*(piPCG++) = pIndicator->iPCG;

		*(pIndicator_++) = *pIndicator;
	
		//if (i / 7 >= n)
		//	break;

		pIndicator = (RVLPSULM_INDICATOR *)(pIndicator->pNext);
	}

	
	//flann::Index<flann::L2<float> > autotuned_index(FeatureArray_, flann::SavedIndexParams("base"));

	
	//FILE *file;
	//if ((file = fopen("base", "r")) != NULL)
	//{
	//	fclose(file);

	//	// file exists
	//	flann::SavedIndexParams savedIndexParamas("base");
	//	
	//	m_pIndex = new 	flann::Index<flann::L2<float>>(FeatureArray_, savedIndexParamas);
	//	
	//	//m_pIndex->load("base");
	//}
	//else
	{
#ifdef RVLPSULM_INDEXING_FLANN
		m_pIndex = new 	flann::Index<flann::L2<float>>(FeatureArray_, flann::KDTreeIndexParams());
		// file not found
		// build index:
		m_pIndex->buildIndex();
		//m_pIndex->save("base");
#else
		BuildIndex();
#endif
			
	}
	
	
	
	//m_pIndex->buildIndex();
	//m_pIndex->save("base");
	
}

void CRVLPSuLMIndexing::BuildIndex()
{
	if (m_IndexMem)
		delete[] m_IndexMem;

	m_IndexMem = new int[m_nMIndicators];

	if (m_Index)
		delete[] m_Index;

	m_Index = new RVLARRAY_<int>[6 * m_ndBins];

	memset(m_Index, 0, 6 * m_ndBins * sizeof(RVLARRAY_<int>));

	int *iIndicatorBin = new int[m_nMIndicators];

	RVLPSULM_INDICATOR *pIndicatorArrayEnd = m_MIndicatorArray + m_nMIndicators;

	RVLPSULM_INDICATOR *pIndicator = m_MIndicatorArray;

	int id;
	int iOrientation;
	float *N;
	float abs[3];
	int iIndicator;
	int iBin;

	for (iIndicator = 0; iIndicator < m_nMIndicators; iIndicator++, pIndicator++)
	{
		id = (int)(pIndicator->m_Descriptor[6] + m_dOffset);

		if (id >= 0 && id < m_ndBins)
		{
			N = pIndicator->m_Descriptor + 3;

			RVLPSULM_INDEXING_GET_ORIENTATION(N, abs, iOrientation);

			iBin = iOrientation * m_ndBins + id;

			iIndicatorBin[iIndicator] = iBin;

			m_Index[iBin].n++;
		}
		else
			iIndicatorBin[iIndicator] = -1;
	}

	int *pBinMem = m_IndexMem;

	int nBins = 6 * m_ndBins;	

	m_maxnIndicatorsInBin = 0;

	int n;

	for (iBin = 0; iBin < nBins; iBin++)
	{
		m_Index[iBin].Element = pBinMem;

		n = m_Index[iBin].n;

		if (n > m_maxnIndicatorsInBin)
			m_maxnIndicatorsInBin = n;

		pBinMem += n;
	}

	if (m_MatchMem)
		delete[] m_MatchMem;

	m_MatchMem = new int[2 * m_maxnIndicatorsInBin];

	int *iElement = new int[nBins];

	memset(iElement, 0, nBins * sizeof(int));

	pIndicator = m_MIndicatorArray;	

	for (iIndicator = 0; iIndicator < m_nMIndicators; iIndicator++, pIndicator++)
	{
		iBin = iIndicatorBin[iIndicator];

		if (iBin > 0)
		{
			m_Index[iBin].Element[iElement[iBin]] = iIndicator;

			iElement[iBin]++;
		}
	}

	delete[] iElement;
	delete[] iIndicatorBin;
}

void CRVLPSuLMIndexing::Search(
	RVLPSULM_INDICATOR *pSIndicator,
	RVLARRAY_<int> &MatchArray)
{
	int id = (int)(pSIndicator->m_Descriptor[6] + m_dOffset - 0.5f);

	int nBins;

	if (id < -1)
	{
		MatchArray.n = 0;

		return;
	}
	else if (id == -1)
	{
		id = 0;

		nBins = 1;
	}
	else if (id == m_ndBins - 1)
		nBins = 1;
	else if (id >= m_ndBins)
	{
		MatchArray.n = 0;

		return;
	}
	else
		nBins = 2;

	float *N = pSIndicator->m_Descriptor + 3;

	float d = pSIndicator->m_Descriptor[6];

	int iOrientation;
	float abs[3];

	RVLPSULM_INDEXING_GET_ORIENTATION(N, abs, iOrientation);

	int iBin = iOrientation * m_ndBins + id;

	int iBinSequenceEnd = iBin + nBins;

	int *pMatch = MatchArray.Element;

	int iBinEntry;
	RVLARRAY_<int> Bin;
	RVLPSULM_INDICATOR *pMIndicator;
	float e;
	int iIndicator;

	for (; iBin < iBinSequenceEnd; iBin++)
	{
		Bin = m_Index[iBin];

		for (iBinEntry = 0; iBinEntry < Bin.n; iBinEntry++)
		{
			iIndicator = Bin.Element[iBinEntry];

			pMIndicator = m_MIndicatorArray + iIndicator;

			//if (pSIndicator->iPCG == 11 && pSIndicator->iFeature == 3 && pMIndicator->iPCG == 124115 && pMIndicator->iFeature == 2)
			//	int debug = 0;

			e = pMIndicator->m_Descriptor[6] - d;

			if (RVLABS(e) <= 1.0f)
				*(pMatch++) = iIndicator;
		}
	}

	MatchArray.n = pMatch - MatchArray.Element;
}

//void CRVLPSuLMIndexing::AssignWordsToIndicators(
//	RVLPSULM_PCG **PCG,
//	char **pWordLT)
//{
//	int maxnWords = 6 * m_ndBins;
//
//	char *WordLT = new char[m_nPCGs * maxnWords];
//
//	*pWordLT = WordLT;
//
//	int *nWords = new int[m_nPCGs];
//
//	memset(nWords, 0, m_nPCGs * sizeof());
//
//	memset(WordLT, 0xff, m_nPCGs * maxnWords * sizeof(char));
//
//	RVLPSULM_INDICATOR *pIndicator = (RVLPSULM_INDICATOR *)(m_IndicatorList.pFirst);
//
//	int id, iBin;
//	int iOrientation;
//	float abs[3];
//	float *N;
//	char *WordLT_;
//	RVLPSULM_PCG *pPCG;
//
//	while (pIndicator)
//	{
//		id = (int)(pIndicator->m_Descriptor[6] + m_dOffset + 0.5f);
//
//		if (id >= 0 && id <= m_ndBins)
//		{
//			N = pIndicator->m_Descriptor + 3;
//
//			RVLPSULM_INDEXING_GET_ORIENTATION(N, abs, iOrientation);
//
//			pPCG = PCG[pIndicator->iPCG];
//
//			iBin = iOrientation * m_ndBins + id;
//
//			pIndicator->word = iBin;
//
//			WordLT_ = WordLT + pIndicator->iPCG * maxnWords;
//
//			if (WordLT_[iBin] < 0)
//			{
//				WordLT_[iBin] = nWords[pIndicator->iPCG];
//
//				nWords[pIndicator->iPCG]++;
//			}				
//		}
//
//		pIndicator = (RVLPSULM_INDICATOR *)(pIndicator->pNext);
//	}
//
//	delete[] nWords;
//}