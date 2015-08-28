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
	m_pIndex = NULL;
	m_PCG = NULL;
	m_PCGBuff = NULL;
}

CRVLPSuLMIndexing::~CRVLPSuLMIndexing(void)
{
	if (m_pIndex)
		delete m_pIndex;

	if (m_PCG)
		delete[] m_PCG;

	if (m_PCGBuff)
		delete[] m_PCGBuff;
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

	m_Mem.Create(10000000);
}

void CRVLPSuLMIndexing::ResetIndicatorList()
{
	RVLQLIST *pIndicatorList = &m_IndicatorList; 
	RVLQLIST_INIT(pIndicatorList);
}

void CRVLPSuLMIndexing::GetIndicators(
	CRVLPSuLM * pPSuLM,
	bool bModel)
{
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)(m_vpBuilder);

	CRVLMem *pMem = (bModel ? pBuilder->m_pMem0 : pBuilder->m_pMem);

	RVLQLIST *pPCGList = &m_PCGList;

	RVLQLIST_INIT(pPCGList)

	RVLQLIST *pIndicatorList = &m_IndicatorList;
	
	if (!bModel)
		RVLQLIST_INIT(pIndicatorList);

	m_nPCGs = 0;
		

	int i, j, k, l, n;
	n = pPSuLM->m_n3DSurfaces;

	m_nFeatures = n;

	RVLPSULM_PCG **PCGLT;

	if (!bModel)
	{
		PCGLT = new RVLPSULM_PCG *[n * n * n];

		memset(PCGLT, 0, n * n * n * sizeof(RVLPSULM_PCG *));
	}

	int n_ = (!bModel ? n : 0);
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

	double *N = N_->data.db;

	CvMat *b_ = cvCreateMat(3, 1, CV_64FC1);

	double *b = b_->data.db;

	CvMat *tGC_ = cvCreateMatHeader(3, 1, CV_64FC1);
	
	int r, n0, m0, n1, m1, n2, m2, n3, m3, n3_;
	FILE *file;
	file = fopen("bModel.txt", "w");

	int maxnIndicators = (bModel ? n * n * n * n : 1000);

	double *N__;
	int PCGID;
	bool bCreatePCG;
	
	for (r = 0; r <= n_ && m_nIndicators < maxnIndicators; r++){
		if (bModel) n0 = n-1;
		else n0 = r;
		m0 = 0;
		//FFF PCG
		for (i = m0; i <= n0; i++){
			int i_;
			i_ = i;
			CRVL3DSurface2 *pSurf0 = pPSuLM->m_3DSurfaceArray[i_];
			double *N0 = pSurf0->m_N;

			if (bModel) {
				n1 = n-1;
				m1 = 0;
			}
			else {
				n1 = n0 - i;
				m1 = i;
			}


			// XGC <- N0
			RVLCOPY3VECTOR(N0, XGC);
				
				for (j = m1; j <= n1 && m_nIndicators < maxnIndicators; j++)
				{
				
				int j_;
				if (bModel){
					j_=j;
					n2 = n - 1;
					m2 = 0;

					if (i == j) continue;
				}
				else{
					j_=j+1;
					n2 = n1 - j;
					m2 = j;
				}
				CRVL3DSurface2 *pSurf1 = pPSuLM->m_3DSurfaceArray[j_];
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
					int k_;
					
					if (bModel){
						k_ = k;
						n3_ = n - 1;
						if (j == k || i == k) continue;
					}
					else {
						k_ = k+2;
						n3 = n2 - k;
						n3_ = (n3 >= k ? 0 : -1);
					}

					CRVL3DSurface2 *pSurf2 = pPSuLM->m_3DSurfaceArray[k_];

					double *N2 = pSurf2->m_N;

					fTmp = RVLDOTPRODUCT3(N1, N2);
					// drugi element indikatora:
					double drugi = acos(RVLABS(fTmp));
					if (RVLABS(fTmp) >= 0.707107) continue; 

					fTmp = RVLDOTPRODUCT3(N2, N0);
					// treci element indikatora:
					double treci = acos(RVLABS(fTmp));
					if (RVLABS(fTmp) >= 0.707107) continue; 

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
						pPCG->iFeature[0] = i_;
						pPCG->iFeature[1] = j_;
						pPCG->iFeature[2] = k_;
						pPCG->pPSuLM = pPSuLM;
						m_nPCGs++;
					}

					for (l = 0; l <= n3_; l++)
					{
						int l_;
						
						if (bModel){
							l_=l;
							if (j == l || i == l || k == l) continue;
						}
						else
							l_=n3+3;

//#ifdef RVLPSULM_INDEXING_DEBUG
//						if (i_ == 0 && j_ == 2 && k_ == 1 && l_ == 5)
//							int debug = 0;
//#endif

						CRVL3DSurface2 *pSurf3 = pPSuLM->m_3DSurfaceArray[l_];
						double *N3 = pSurf3->m_N;

						RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_INDICATOR, pIndicator)

						//prebacivanje prva tri elementa indikatora + normiranje: 5°=0.1745329252 rad ~ 1
						pIndicator->m_Descriptor[0] = prvi / 0.0872664625;
						pIndicator->m_Descriptor[1] = drugi / 0.0872664625;
						pIndicator->m_Descriptor[2] = treci / 0.0872664625;

						//popunjavanje ostala èetiri mjesta deskriptora:
						float *N3G = pIndicator->m_Descriptor + 3;
						float *RHO3G = pIndicator->m_Descriptor + 6;


						RVLMULMX3X3VECT(RCG, N3, N3G);
						//normiranje: 2*sin(5°/2)= 0.08723877473 ~ 1
						N3G[0] = N3G[0] / 0.08723877473;
						N3G[1] = N3G[1] / 0.08723877473;
						N3G[2] = N3G[2] / 0.08723877473;
						*RHO3G = (pSurf3->m_d - RVLDOTPRODUCT3(N3, tGC)) / 150.0;; //normiranje: 150 mm ~ 1	

						RVLQLIST_ADD_ENTRY(pIndicatorList, pIndicator);

						pIndicator->iPCG = pPCG->Index;
						pIndicator->iFeature = l_;
						m_nIndicators++;

					}
					}
				}
		}
	}
	fclose(file);

	if (!bModel)
		delete[] PCGLT;

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
#ifdef RVLPSULM_INDEXING_DEBUG 
	FILE *fpDebug = fopen("C:\\RVL\\Debug\\IndexingDebug.txt", "w");
#endif
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)(m_vpBuilder);

	int r = 10000;	// broj rjesenja
	
	float *D = new float[r];	//matrica s udaljenostima odgovarajuceg rjesenja od query-ja
	int *R = new int[r];
	flann::Matrix<int>R_(R, 1, r);
	flann::Matrix<float> D_(D, 1, r);

	// Izvaditi indikatore iz mIndicatorList i za svaki indikator (while petlja) pokrenuti radiusSearch
	// Svaki indikator je Query za radiusSearch

	CRVLMem *pMem = &m_Mem;

	//CRVLMem *pMem = pBuilder->m_pMem;

	RVLPSULM_INDICATOR *pIndicator = (RVLPSULM_INDICATOR *)(m_IndicatorList.pFirst);
	m_EvidenceAccu.Reset();
	RVLQLIST *EvidenceAccu_ = m_EvidenceAccu.m_ListArray;
	RVLQLIST_INT_ENTRY *pEvidenceAccuEntry;
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

	int i, j, k;
	int brojac_aktivnih = 0;
	int iMPCG;
	while (pIndicator)
	{
		flann::Matrix<float> Q_(pIndicator->m_Descriptor, 1, 7);

		// Pomoæu radiusSearch za svaki indikator iz m_IndicatorList dobiti sve indikatore iz baze unutar zadanog radijusa:
		m_pIndex->radiusSearch(Q_ , R_, D_, 2.24f, flann::SearchParams(64));

		
#ifdef RVLPSULM_INDEXING_DEBUG 
		pSPCG = SPCG[pIndicator->iPCG];
		fprintf(fpDebug, "\n\nQ: %d %d %d %d (PCG%d)\t", pSPCG->iFeature[0], pSPCG->iFeature[1], pSPCG->iFeature[2], pIndicator->iFeature, pIndicator->iPCG);	
		for (i = 0; i < 7; i++){
			fprintf(fpDebug, "%.4f\t", pIndicator->m_Descriptor[i]);
		}
		fprintf(fpDebug, "\n\nR:");
				
#endif

		// Za svaki vraæeni indeks vadi se iz m_iPCG indeks PCG-a, pomoæu tog indeksa se preko pokazivaèa u polju m_PCG dolazi do PCG-a:	
		for (i = 0; R[i] >= 0 && i < r; i++){ 
			iMPCG = m_iPCG[R[i]];
			//dodavanje entry-a s indeksom indikatora scene
			RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST_INT_ENTRY, pEvidenceAccuEntry);
			RVLQLISTARRAY_ADD_ENTRY(EvidenceAccu_, iMPCG, pEvidenceAccuEntry);
			pEvidenceAccuEntry->i = pIndicator->iPCG;

			//postavljanje flaga aktivnog PCG-a (ako nije aktivan)
			
			pMPCG = m_PCG[iMPCG];
			
#ifdef RVLPSULM_INDEXING_DEBUG 
			fprintf(fpDebug, "%d\t ",R[i] );

#endif
			if (!(pMPCG->Flags & RVLPSULM_PCG_FLAG_ACTIVE))
			{
				pMPCG->Flags |= RVLPSULM_PCG_FLAG_ACTIVE;
				m_PCGBuff[brojac_aktivnih] = iMPCG;
				brojac_aktivnih++;
			}
		}

		pIndicator = (RVLPSULM_INDICATOR *)(pIndicator->pNext);
	}

	delete[] D;
	delete[] R;

#ifdef RVLPSULM_INDEXING_DEBUG 
	fclose(fpDebug);
#endif

#ifdef RVLPSULM_INDEXING_DEBUG 
	fpDebug = fopen("C:\\RVL\\Debug\\IndexingMatchMatrix.txt", "w");
	
	int Histogram[30];

	memset(Histogram, 0, 30 * sizeof(int));
#endif

	CRVLQListArray SortedMatchList;

	SortedMatchList.m_Size = m_nFeatures;

	SortedMatchList.Init();
	SortedMatchList.Reset();

	RVLQLIST *SortedMatchList_ = SortedMatchList.m_ListArray;
	RVLQLIST *pSortedMatchList;
	RVLPCG_MATCH *pSortedMatchListEntry;

	int *brojac_SPCGs = new int[m_nPCGs]; // ako flag nije aktivan staviti na 1, ako je aktivan povecati za 1
	int *SPCGBuff = new int[m_nPCGs];
	int *piSPCG;
	int *SPCGBuffEnd;
	RVLPCG_MATCH *pMatch;

	RVLQLIST *pEvidenceAccu;

	for (i = 0; i < brojac_aktivnih; i++)	// for all MPCGs
	{
		iMPCG = m_PCGBuff[i];
		
		pMPCG = m_PCG[iMPCG];

		pMPCG->Flags &= ~RVLPSULM_PCG_FLAG_ACTIVE;

		pEvidenceAccu = EvidenceAccu_ + iMPCG;
		
		pEvidenceAccuEntry = (RVLQLIST_INT_ENTRY*)(pEvidenceAccu->pFirst);
		piSPCG = SPCGBuff;
		while (pEvidenceAccuEntry)
		{
			pSPCG = SPCG[pEvidenceAccuEntry->i];

			if (!(pSPCG->Flags & RVLPSULM_PCG_FLAG_ACTIVE))
			{
				brojac_SPCGs[pEvidenceAccuEntry->i] = 1;
				pSPCG->Flags |= RVLPSULM_PCG_FLAG_ACTIVE;
				*(piSPCG++) = pEvidenceAccuEntry->i;
			}
			else 
				brojac_SPCGs[pEvidenceAccuEntry->i] ++;

			pEvidenceAccuEntry = (RVLQLIST_INT_ENTRY*)(pEvidenceAccuEntry->pNext);
		}

#ifdef RVLPSULM_INDEXING_DEBUG 
		fprintf(fpDebug, "M%d:\n\n", iMPCG);
#endif

		SPCGBuffEnd = piSPCG;		
		
		for (piSPCG = SPCGBuff; piSPCG < SPCGBuffEnd; piSPCG++)
		{
#ifdef RVLPSULM_INDEXING_DEBUG 
			int count = brojac_SPCGs[*piSPCG];

			fprintf(fpDebug, "%d(%d)\n", *piSPCG, count);

			if (count > 29)
				count = 29;

			Histogram[count]++;

			if (count == 13)
				int debug = 0;
#endif

			pSPCG = SPCG[*piSPCG];

			pSPCG->Flags &= ~RVLPSULM_PCG_FLAG_ACTIVE;			

			RVLMEM_ALLOC_STRUCT(pMem, RVLPCG_MATCH, pMatch);

			pMatch->pMPCG = pMPCG;
			pMatch->pSPCG = pSPCG;

			RVLQLISTARRAY_ADD_ENTRY(SortedMatchList_, count, pMatch);
		}

#ifdef RVLPSULM_INDEXING_DEBUG 
		fprintf(fpDebug, "\n");
#endif
	}	// for all MPCGs
		
#ifdef RVLPSULM_INDEXING_DEBUG 
	fclose(fpDebug);

	fpDebug = fopen("C:\\RVL\\Debug\\IndexingStatistics.txt", "w");
	
	for (i = 0; i < 30; i++)
		fprintf(fpDebug, "%d\n", Histogram[i]);
	
	fclose(fpDebug);
#endif

	delete[] SPCG;
	delete[] brojac_SPCGs;
	delete[] SPCGBuff;

	DWORD HypothesisCounter = 0;

	RVLPSULM_HYPOTHESIS *pHypothesis;
	double RtmpInv[9];
	double ttmpInv[3];
	double *RPCGM;
	double *tPCGM;
	double *RPCGS;
	double *tPCGS;
	double *RSM, *tSM;

#ifdef RVLPSULM_INDEXING_DEBUG 
	FILE *fpDebugHyp = fopen("C:\\RVL\\Debug\\Hypothesis.txt", "w");
#endif


	for (i = m_nFeatures - 1; i > 0 && HypothesisCounter < 30; i--)
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

			RVLMEM_ALLOC_STRUCT(pMem, RVLPSULM_HYPOTHESIS, pHypothesis);

			RSM = pHypothesis->PoseSM.m_Rot;
			tSM = pHypothesis->PoseSM.m_X;

			RVLINVTRANSF3D(RPCGS, tPCGS, RtmpInv, ttmpInv);
			RVLCOMPTRANSF3D(RPCGM, tPCGM, RtmpInv, ttmpInv, RSM, tSM);
			pHypothesis->PoseSM.UpdatePTRLL();

			pHypothesis->Index = HypothesisCounter;
			pHypothesis->pMPSuLM = pMPCG->pPSuLM;

			HypothesisCounter++;

			pBuilder->m_HypothesisList.Add(pHypothesis);

#ifdef RVLPSULM_INDEXING_DEBUG 
			fprintf(fpDebugHyp, "no: %d \t index: %d \t cost: %d \n\n", HypothesisCounter, pHypothesis->pMPSuLM->m_Index, i);
#endif
			pSortedMatchListEntry = (RVLPCG_MATCH *)(pSortedMatchListEntry->pNext);
		}
	}

#ifdef RVLPSULM_INDEXING_DEBUG 
	fclose(fpDebugHyp);
#endif
	
		

	

	/////

	m_Mem.Clear();
}

void CRVLPSuLMIndexing::UpdateBase()
{
}

void CRVLPSuLMIndexing::CreateBase()
{
	
	CRVLPSuLMBuilder *pBuilder = (CRVLPSuLMBuilder *)(m_vpBuilder);

	ResetIndicatorList();

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
		
		GetIndicators(pPSuLM, true);  // otkomentirati
		
		//GetIndicators(pPSuLM);	// zakomentirati
			m_nMPCGs += m_nPCGs;
			m_nMIndicators += m_nIndicators;
		//	break;
		//}

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

	float *IndicatorArray = new float[n * m]; //matrica znacajki
	flann::Matrix<float> IndicatorArray_(IndicatorArray, n, m);
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

	RVLPSULM_INDICATOR *pIndicator = (RVLPSULM_INDICATOR *)(m_IndicatorList.pFirst);

#ifdef RVLPSULM_INDEXING_DEBUG 
	FILE *fpDebugInd = fopen("C:\\RVL\\Debug\\Indicators.txt", "w");
#endif

	int i = 0, j, k;
	while (pIndicator)
	{
		//popunjavanje matrice znacajki:
		// dodati znaèajku za svaki indikator
		j = 0;
		for (k = i; k < i + 7; k++){
			IndicatorArray[k] = pIndicator->m_Descriptor[j];
			j++;
		}

		// dodati iPCG od indikatora u m_iPCG
		*(piPCG++) = pIndicator->iPCG;

#ifdef RVLPSULM_INDEXING_DEBUG 
		pPCG = m_PCG[pIndicator->iPCG];
		fprintf(fpDebugInd, 
			"Indikator: %d\t PCG: %d\t F: %d %d %d %d\t", 
			i/7, pIndicator->iPCG, pPCG->iFeature[0], pPCG->iFeature[1], pPCG->iFeature[2], pIndicator->iFeature);
		for (j = 0; j < 7; j++){
			fprintf(fpDebugInd, "%.4f ", pIndicator->m_Descriptor[j]);
		}
		fprintf(fpDebugInd, "\n\n");
#endif

		i += 7;
	
		//if (i / 7 >= n)
		//	break;

		pIndicator = (RVLPSULM_INDICATOR *)(pIndicator->pNext);
	}

#ifdef RVLPSULM_INDEXING_DEBUG 
	fclose(fpDebugInd);
#endif

	

	

	
	//FILE *file;
	//if ((file = fopen("base", "r")) != NULL)
	//{
	//	fclose(file);

	//	// file exists
	//	flann::SavedIndexParams savedIndexParamas("base");
	//	
	//	m_pIndex = new 	flann::Index<flann::L2<float>>(savedIndexParamas);
	//	
	//	
	//	//m_pIndex->load("base");
	//}
	//else
	{
		m_pIndex = new 	flann::Index<flann::L2<float>>(IndicatorArray_, flann::KDTreeIndexParams());
		// file not found
		// build index:
		m_pIndex->buildIndex();
		m_pIndex->save("base");
		
			
	}
	
	
	
	//m_pIndex->buildIndex();
	//m_pIndex->save("base");
	
}