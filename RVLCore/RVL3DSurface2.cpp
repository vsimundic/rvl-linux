// RVL3DSurface2.cpp: implementation of the CRVL3DSurface2 class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "RVLCore.h"
#include "RVLEDT.h"
#include "RVLDelaunay.h"
#include "RVLC2D.h"
#include "RVLC2DContour.h"
//#include "RVLClass.h"
//#include "RVL2DObject3.h"
#include "RVL2DContour.h"
#include "RVL3DObject.h"
#include "RVL2DLine3.h"
#include "RVL3DLine2.h"
#include "RVL3DContour.h"
#include "RVL2DRegion2.h"
#include "RVL3DSurface2.h"

CRVL3DSurface2 RVL3DSurfaceTemplate;

#define RVL3DSURFACE_MODEL_2


//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL3DSurface2::CRVL3DSurface2()
{
	//m_epsilon1 = 11.34*0.7*0.7;
	m_epsilon1 = 11.34;		//Chi squared test 99% for 3 degrees of freedom ie abt 3*sigma confidence interval (99.8%) (default)
	//m_epsilon2 = 3.66;		//Chi squared test 70% for 3 degrees of freedom ie abt 1*sigma confidence interval (68.2%)
	m_epsilon2 = 11.34;		//Chi squared test 99% for 3 degrees of freedom ie abt 3*sigma confidence interval (99.8%)
}

CRVL3DSurface2::~CRVL3DSurface2()
{

}


CRVLObject2 * CRVL3DSurface2::Create2(CRVLClass * pClass)
{
	CRVL3DSurface2 *pObject = (CRVL3DSurface2 *)(pClass->m_pMem0->Alloc(sizeof(CRVL3DSurface2)));

	memcpy(pObject, this, sizeof(CRVL3DSurface2));

	RVLQLIST *pBoundaryContourList = &(pObject->m_BoundaryContourList);

	RVLQLIST_INIT(pBoundaryContourList);

	RVLQLIST *pSamples = &(pObject->m_Samples);

	RVLQLIST_INIT(pSamples);

	pObject->CRVLObject2::Create(pClass);

	pClass->Add(pObject);

	return pObject;	
}


void CRVL3DSurface2::Rotate(double *Rot)
{
	double N[3];

	LinearTransform3D(Rot, m_N, N);

	memcpy(m_N, N, 3 * sizeof(double));
}

void CRVL3DSurface2::Crop(	CRVL3DPose *pPoseC0,
							CRVLCamera *pCamera,
							RVLRECT *pROI,
							RVL3DCONTOUR_CROP_PARAMS *pCropLTs,
							CvPoint **pPt2DArray,
							CRVLClass *p2DContourSet)
{	
	//BOOL bTooClose = (m_d * m_d < pCropLTs->minr);

	CvPoint *BoundaryPtArray = *pPt2DArray;

	CRVL3DContour *pBoundary;
	CRVL2DContour *p2DContour;

	if(m_pClass->m_iRelListContours >= 0)
	{
		RVLARRAY *pRelList = m_RelList + m_pClass->m_iRelListContours;

		CRVL3DContour **pBoundaryPtrArrayEnd = (CRVL3DContour **)(pRelList->pEnd);

		CRVL3DContour **ppBoundary;

		for(ppBoundary = (CRVL3DContour **)(pRelList->pFirst); ppBoundary < pBoundaryPtrArrayEnd; ppBoundary++)
		{
			p2DContour = (CRVL2DContour *)(RVL2DContourTemplate.Create3(p2DContourSet));

			p2DContour->m_ContourIPArray = (RVLIPOINT *)BoundaryPtArray;

			//if(bTooClose)
			//	p2DContour->m_nContourIPs = 0;
			//else
			//{
				pBoundary = *ppBoundary;

				RVLCrop3DContour((double *)(pBoundary->m_PtArray), pBoundary->m_nPts, pPoseC0, pCamera, pROI, 
					pCropLTs, BoundaryPtArray, &(p2DContour->m_nContourIPs));

				BoundaryPtArray += p2DContour->m_nContourIPs;
			//}
		}
	}
	else if(m_pClass->m_iDataContourPtr >= 0)
	{
		pBoundary = *((CRVL3DContour **)(m_pData + m_pClass->m_iDataContourPtr));

		p2DContour = (CRVL2DContour *)(RVL2DContourTemplate.Create3(p2DContourSet));

		p2DContour->m_ContourIPArray = (RVLIPOINT *)BoundaryPtArray;

		RVLCrop3DContour((double *)(pBoundary->m_PtArray), pBoundary->m_nPts, pPoseC0, pCamera, pROI, 
			pCropLTs, BoundaryPtArray, &(p2DContour->m_nContourIPs));

		BoundaryPtArray += p2DContour->m_nContourIPs;
	}

	*pPt2DArray = BoundaryPtArray;
}



void CRVL3DSurface2::Save(	FILE *fp,
							DWORD Flags)
{
	//fprintf(fp, "%lf\t%lf\t%lf\t%lf\n", m_N[0], m_N[1], m_N[2], m_d);
	if(Flags == RVL3DSURFACE_FLAG_CONVEX)
	{	
		fwrite(m_Pose.m_Rot, sizeof(double), 9, fp);
		fwrite(m_Pose.m_X, sizeof(double), 3, fp);
		fwrite(m_EigenValues, sizeof(double), 2, fp);
		fwrite(&m_sigmaR, sizeof(double), 1, fp);
	}
	else
	{
		fwrite(m_Pose.m_Rot, sizeof(double), 9, fp);
		fwrite(m_Pose.m_X, sizeof(double), 3, fp);
		fwrite(m_EigenValues, sizeof(double), 2, fp);
		fwrite(&m_sigmaR, sizeof(double), 1, fp);
		//Added Karlo

		fwrite(m_N, sizeof(double), 3, fp);
		fwrite(&m_d, sizeof(double), 1, fp);
		fwrite(&m_Area, sizeof(double), 1, fp);
		if (this->m_histRGB)
			CRVL3DMeshObject::Save(fp,Flags);

		// save surface samples

		RVLQLIST *pSamples = &m_Samples;

		RVL3DSURFACE_SAMPLE *pSample = (RVL3DSURFACE_SAMPLE *)(m_Samples.pFirst);

		int nSamples = 0;

		while(pSample)
		{
			nSamples++;

			pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		}

		fwrite(&nSamples, sizeof(int), 1, fp);

		pSample = (RVL3DSURFACE_SAMPLE *)(m_Samples.pFirst);

		while(pSample)
		{
			fwrite(pSample->X, sizeof(double), 3, fp);
#ifdef RVL3DSURFACE_MODEL_2
			fwrite(&(pSample->wz), sizeof(double), 1, fp);
			fwrite(pSample->V, sizeof(double), 3, fp);
			fwrite(&(pSample->stdX), sizeof(double), 1, fp);
			fwrite(pSample->stdN, sizeof(double), 2, fp);
#endif

			pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		}

		// save surface boundary

		if (m_Flags & RVL3DSURFACE_FLAG_BOUNDARY)
		{
			RVLQLIST *pContourList = &m_BoundaryContourList;

			int nContours = 0;

			RVL3DCONTOUR *pContour = (RVL3DCONTOUR *)(pContourList->pFirst);

			while (pContour)
			{
				nContours++;

				pContour = (RVL3DCONTOUR *)(pContour->pNext);
			}

			fwrite(&nContours, sizeof(int), 1, fp);

			int nPts;
			RVLQLIST *pPtList;
			RVL3DPOINT3 *pVertex;
			
			pContour = (RVL3DCONTOUR *)(pContourList->pFirst);

			while (pContour)
			{
				fwrite(&(pContour->bHole), sizeof(bool), 1, fp);
				fwrite(&(pContour->iView), sizeof(int), 1, fp);

				pPtList = &(pContour->PtList);

				nPts = 0;

				pVertex = (RVL3DPOINT3 *)(pPtList->pFirst);

				while (pVertex)
				{
					nPts++;

					pVertex = (RVL3DPOINT3 *)(pVertex->pNext);
				}

				fwrite(&nPts, sizeof(int), 1, fp);

				pVertex = (RVL3DPOINT3 *)(pPtList->pFirst);

				while (pVertex)
				{
					fwrite(pVertex->P2D, sizeof(int), 2, fp);
					fwrite(pVertex->P3D, sizeof(double), 3, fp);

					pVertex = (RVL3DPOINT3 *)(pVertex->pNext);
				}

				pContour = (RVL3DCONTOUR *)(pContour->pNext);
			}
		}
	}
	fwrite(&m_nSupport, sizeof(int), 1, fp);
}

void CRVL3DSurface2::Load(	FILE *fp,
							DWORD Flags)
{
	//fscanf(fp, "%lf\t%lf\t%lf\t%lf\n", m_N, m_N + 1, m_N + 2, &m_d);
	if(Flags == RVL3DSURFACE_FLAG_CONVEX)
	{	
		fread(m_Pose.m_Rot, sizeof(double), 9, fp);
		fread(m_Pose.m_X, sizeof(double), 3, fp);
		fread(m_EigenValues, sizeof(double), 2, fp);
		fread(&m_sigmaR, sizeof(double), 1, fp);

		double sigmaR = m_sigmaR * m_pClass->m_UncertCoeff * m_pClass->m_UncertCoeff;

		m_varq[0] =  sigmaR / (m_EigenValues[0] * m_EigenValues[0] + sigmaR);
		m_varq[1] =  sigmaR / (m_EigenValues[1] * m_EigenValues[1] + sigmaR);
		m_varq[2] =  sigmaR;
		m_N[0] = m_Pose.m_Rot[2];
		m_N[1] = m_Pose.m_Rot[5];
		m_N[2] = m_Pose.m_Rot[8];
		m_d = RVLDOTPRODUCT3(m_Pose.m_X, m_N);
	}
	else
	{
		fread(m_Pose.m_Rot, sizeof(double), 9, fp);
		fread(m_Pose.m_X, sizeof(double), 3, fp);
		fread(m_EigenValues, sizeof(double), 2, fp);
		fread(&m_sigmaR, sizeof(double), 1, fp);

		double sigmaR = m_sigmaR * m_pClass->m_UncertCoeff * m_pClass->m_UncertCoeff;

		m_varq[0] =  sigmaR / (m_EigenValues[0] * m_EigenValues[0] + sigmaR);
		m_varq[1] =  sigmaR / (m_EigenValues[1] * m_EigenValues[1] + sigmaR);
		m_varq[2] =  sigmaR;
		m_N[0] = m_Pose.m_Rot[2];
		m_N[1] = m_Pose.m_Rot[5];
		m_N[2] = m_Pose.m_Rot[8];
		m_d = RVLDOTPRODUCT3(m_Pose.m_X, m_N);
		//Added Karlo

		fread(m_N, sizeof(double), 3, fp);
		fread(&m_d, sizeof(double), 1, fp);
		fread(&m_Area, sizeof(double), 1, fp);
		if (Flags & RVL3DSURFACE_FLAG_MATERIAL)
			CRVL3DMeshObject::Load(fp,Flags);

		CRVLMem *pMem = m_pClass->m_pMem0;

		// load surface samples

		int nSamples;

		fread(&nSamples, sizeof(int), 1, fp);

		if(nSamples > 0)
		{
			RVLQLIST *pSamples = &m_Samples;

			RVLQLIST_INIT(pSamples)

			RVL3DSURFACE_SAMPLE *pSample;

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RVL3DSURFACE_SAMPLE, nSamples, pSample);

			RVL3DSURFACE_SAMPLE *pSamplesEnd = pSample + nSamples;

			m_Samples.pFirst = pSample;

			for(; pSample < pSamplesEnd; pSample++)
			{
				fread(pSample->X, sizeof(double), 3, fp);
#ifdef RVL3DSURFACE_MODEL_2
				fread(&(pSample->wz), sizeof(double), 1, fp);
				fread(pSample->V, sizeof(double), 3, fp);
				fread(&(pSample->stdX), sizeof(double), 1, fp);
				fread(pSample->stdN, sizeof(double), 2, fp);
#endif
				pSample->pSurface = this;
				pSample->pNext = pSample + 1;
			}

			pSample--;

			pSample->pNext = NULL;
		}

		// load surface boundary contours

		if (m_Flags & RVL3DSURFACE_FLAG_BOUNDARY)
		{
			int nContours;

			fread(&nContours, sizeof(int), 1, fp);

			if (nContours > 0)
			{
				RVLQLIST *pContourList = &m_BoundaryContourList;

				RVLQLIST_INIT(pContourList);

				int iContour, iPt;
				RVL3DCONTOUR *pContour;
				int nPts;
				RVLQLIST *pPtList;
				RVL3DPOINT3 *pVertex;

				for (iContour = 0; iContour < nContours; iContour++)
				{
					RVLMEM_ALLOC_STRUCT(pMem, RVL3DCONTOUR, pContour);

					RVLQLIST_ADD_ENTRY(pContourList, pContour);

					fread(&(pContour->bHole), sizeof(bool), 1, fp);
					fread(&(pContour->iView), sizeof(int), 1, fp);

					fread(&nPts, sizeof(int), 1, fp);

					pPtList = &(pContour->PtList);

					RVLQLIST_INIT(pPtList);

					for (iPt = 0; iPt < nPts; iPt++)
					{
						RVLMEM_ALLOC_STRUCT(pMem, RVL3DPOINT3, pVertex);

						RVLQLIST_ADD_ENTRY(pPtList, pVertex);

						fread(pVertex->P2D, sizeof(int), 2, fp);
						fread(pVertex->P3D, sizeof(double), 3, fp);
					}
				}
			}
		}
	}
	fread(&m_nSupport, sizeof(int), 1, fp);
}



BOOL CRVL3DSurface2::Match(	CRVL3DObject *pMObject, 
							CRVL3DPose *pPose,
							double &MatchQuality, void *vpMatchData)
{
	//return FALSE;	// debug
	bool bMatch = false;

	CRVL3DSurface2 *pMSurface = (CRVL3DSurface2 *)pMObject;
	RVLSURFACE_MATCH_ARRAY *pMatchArray =(RVLSURFACE_MATCH_ARRAY *)vpMatchData;

	
	//Get convex surfaces (LEVEL2) for scene and model
	//Get the 1st convex surface for both scene and model - these are the first and largest in the list 
	//(Since the list is sorted in descending order)

	RVLARRAY *pRelListSurfaceScene, *pRelListSurfaceModel;
	CRVL3DSurface2 **pp3DConvexSegmentScene , **pp3DConvexSegmentSceneEnd;
	CRVL3DSurface2 **pp3DConvexSegmentModel , **pp3DConvexSegmentModelEnd;
	CRVL3DSurface2 *p3DSceneSurface ,*p3DModelSurface;

	//Scene
	pRelListSurfaceScene = m_RelList + m_pClass->m_iRelListComponents;
	pp3DConvexSegmentScene = (CRVL3DSurface2 **)pRelListSurfaceScene->pFirst;
	pp3DConvexSegmentSceneEnd = (CRVL3DSurface2 **)pRelListSurfaceScene->pEnd;

	//Biggest surface in scene (1st in list)
	p3DSceneSurface = *pp3DConvexSegmentScene;
	
	//Model
	pRelListSurfaceModel = pMSurface->m_RelList + pMSurface->m_pClass->m_iRelListComponents;
	pp3DConvexSegmentModel = (CRVL3DSurface2 **)pRelListSurfaceModel->pFirst;
	pp3DConvexSegmentModelEnd = (CRVL3DSurface2 **)pRelListSurfaceModel->pEnd;

	//Biggest surface in model  (1st in list)
	p3DModelSurface = *pp3DConvexSegmentModel;

	//Using only the biggest surfaces calculate matrices and check 1st constraint

	int i;//,j;
	double dMatrixA31[3];
	double dMatrixB31[3];
	//double dMatrixC31[3];
	double dMatrixA21[2];
	double dMatrixB21[2];
	double dMatrix22[4];
	double dMatrix23[6];
	double dMatrixA33[9];
	double dMatrixB33[9];
	double dMatrixC33[9];
	double dMatrix36[18];
	
	double dTempA;
	double Rtrans[9];

	//define constant vectors
	double xA[3],yA[3],zA[3],xB[3],yB[3],zB[3], tA[3], tB[3];
	double dMatrixXYA[6];
	double dMatrixXYB[6];
	
	double P[36];
	double E[9], E_[9];
	double CqA[9], CqB[9];

	double e_[3];
	double CfA[9], CfB[9], C_[18];
	double J[9];
	double Q_[9];

	double constraint0, constraint1, constraint2;


		
	for(i=0;i<3;i++)
	{
		//Get xA, yA, zA
		xA[i] = p3DSceneSurface->m_Pose.m_Rot[i * 3 + 0];
		yA[i] = p3DSceneSurface->m_Pose.m_Rot[i * 3 + 1];
		zA[i] = p3DSceneSurface->m_Pose.m_Rot[i * 3 + 2];

		tA[i] = p3DSceneSurface->m_Pose.m_X[i];

		//Get xB, yB, zB
		xB[i] = p3DModelSurface->m_Pose.m_Rot[i * 3 + 0];
		yB[i] = p3DModelSurface->m_Pose.m_Rot[i * 3 + 1];
		zB[i] = p3DModelSurface->m_Pose.m_Rot[i * 3 + 2];

		//Get tB
		tB[i] = p3DModelSurface->m_Pose.m_X[i];

		////Get P -> is initially a diagonal matrix
		//P[i*6+i]		= pPose->m_C[0*3*3+i*3+i];
		//P[3*6+i*6+3+i]	= pPose->m_C[2*3*3+i*3+i];
	}

	//make sure the angle between normals (ie nb and Rot*na OR zA and zB) is less than 90

	MatchQuality = -1.0;

	constraint0 = RVLDotProduct(zA, zB);
	if(constraint0 > 0)
	{

		for(i=0;i<3;i++)
		{
			//Get [xA yA]
			dMatrixXYA[i*2] = xA[i];
			dMatrixXYA[i*2 + 1] = yA[i];

			//Get [xB yB]
			dMatrixXYB[i*2] = xB[i];
			dMatrixXYB[i*2 + 1] = yB[i];
		}

		//initialise P
		memset(P,0,36*sizeof(double));
		RVL3x3x3BlockMxTo6x6(pPose->m_C, P);


		//*******Calculate e**************
		
		//1. Denominator
		//(tA + Rot'*t)'*zA - tB'*zB
		MatrixMultiplicationT2(pPose->m_Rot, pPose->m_X, dMatrixA31, 3, 3, 1);
		
		RVLSum3D(tA,dMatrixA31,dMatrixB31);

		dTempA = RVLDotProduct(dMatrixB31, zA) - RVLDotProduct(tB,zB);
		
		//2. Numerator
		//[xB yB]'*Rot*zA
		MatrixMultiplicationT2(dMatrixXYB,pPose->m_Rot, dMatrix23, 2, 3, 3);
		
		MatrixMultiplication(dMatrix23,zA, dMatrixA21, 2, 3, 1);
		
		//Store e
		pMatchArray->m_e[0] =  dMatrixA21[0];
		pMatchArray->m_e[1] =  dMatrixA21[1];
		pMatchArray->m_e[2] =  dTempA;



		//*******Calculate C *****************
		//equation (13) EKF
		pPose->JacobianPTR2RotX(zA,dMatrixA31,J);	//Jacobian

		MatrixMultiplicationT2(dMatrixXYB,J, dMatrix23, 2, 3, 3);

		MatrixMultiplicationT2(pPose->m_X,J, dMatrixA31, 1, 3, 3);

		//Transpose of pPose->m_Rot
		for(i=0;i<3;i++)
		{
			Rtrans[0*3 + i] = pPose->m_Rot[i*3 + 0];
			Rtrans[1*3 + i] = pPose->m_Rot[i*3 + 1];
			Rtrans[2*3 + i] = pPose->m_Rot[i*3 + 2];
		}

		MatrixMultiplicationT2(zA,Rtrans, dMatrixB31, 1, 3, 3);


		//Store C
		memset(pMatchArray->m_C,0,18*sizeof(double));

		for(i=0;i<2;i++)
		{
			pMatchArray->m_C[i*6 + 0] = dMatrix23[i*3 + 0];
			pMatchArray->m_C[i*6 + 1] = dMatrix23[i*3 + 1];
			pMatchArray->m_C[i*6 + 2] = dMatrix23[i*3 + 2];
		}
		for(i=0;i<3;i++)
		{
			pMatchArray->m_C[12 + i] = dMatrixA31[i];
			pMatchArray->m_C[15 + i] = dMatrixB31[i];
		}


		//*******Calculate E *****************
		//equation (14) EKF
		MatrixMultiplicationT2(dMatrixXYB,pPose->m_Rot, dMatrix23, 2, 3, 3);
		MatrixMultiplication(dMatrix23,dMatrixXYA,dMatrix22, 2, 3, 2);

		MatrixMultiplicationT2(tA,dMatrixXYA,dMatrixA21, 1, 3, 2);

		MatrixMultiplicationT2(pPose->m_X,pPose->m_Rot,dMatrixA31, 1, 3, 3);
		MatrixMultiplication(dMatrixA31,dMatrixXYA,dMatrixB21, 1, 3, 2);

		for(i=0;i<2;i++)
			dMatrixA21[i] += dMatrixB21[i];

		memset(E,0,9*sizeof(double));
		for(i=0;i<2;i++)
		{
			E[3*i + 0] = dMatrix22[i*2 + 0];
			E[3*i + 1] = dMatrix22[i*2 + 1];
		}
		E[6] = dMatrixA21[0];
		E[7] = dMatrixA21[1];
		E[8] = 1;


		//*******Calculate E_ ***************** NOTE this a constant for a given model plane!!! Maybe it should be calculated once and then stored
		//equation (15) EKF
		MatrixMultiplicationT2(tB,dMatrixXYB,dMatrixA21, 1, 3, 2);
		
		memset(E_,0,9*sizeof(double));
		
		E_[0] = -1;
		E_[4] = -1;
		E_[6] = -dMatrixA21[0];
		E_[7] = -dMatrixA21[1];
		E_[8] = -1;


		//*******Calculate Cq and Cq' ***************** 
		// (using scene plane params)
		memset(CqA,0,9*sizeof(double));
		CqA[0] =  p3DSceneSurface->m_sigmaR / (p3DSceneSurface->m_EigenValues[0] * p3DSceneSurface->m_EigenValues[0] + p3DSceneSurface->m_sigmaR);
		CqA[4] =  p3DSceneSurface->m_sigmaR / (p3DSceneSurface->m_EigenValues[1] * p3DSceneSurface->m_EigenValues[1] + p3DSceneSurface->m_sigmaR);
		CqA[8] =  p3DSceneSurface->m_sigmaR;

		// (using model plane params)
		memset(CqB,0,9*sizeof(double));
		CqB[0] =  p3DModelSurface->m_sigmaR / (p3DModelSurface->m_EigenValues[0] * p3DModelSurface->m_EigenValues[0] + p3DModelSurface->m_sigmaR);
		CqB[4] =  p3DModelSurface->m_sigmaR / (p3DModelSurface->m_EigenValues[1] * p3DModelSurface->m_EigenValues[1] + p3DModelSurface->m_sigmaR);
		CqB[8] =  p3DModelSurface->m_sigmaR;


		//*******Calculate Q *****************
		//equation (11) EKF
		//E*CqA*E'
		MatrixMultiplication(E,CqA,dMatrixB33,3,3,3);
		MatrixMultiplicationT(dMatrixB33,E,dMatrixA33,3,3,3);

		//E_*CqB*E_'
		MatrixMultiplication(E_,CqB,dMatrixC33,3,3,3);
		MatrixMultiplicationT(dMatrixC33,E_,dMatrixB33,3,3,3);

		//C*P*C'
		MatrixMultiplication(pMatchArray->m_C,P, dMatrix36, 3, 6, 6);
		MatrixMultiplicationT(dMatrix36,pMatchArray->m_C, dMatrixC33, 3, 6, 3);

		//sum up all the results
		for(i=0;i<9;i++)
			dMatrixA33[i] = dMatrixA33[i] + dMatrixB33[i] + dMatrixC33[i];

		//Get Q ie pMatchArray->m_Q
		RVLMatrixHeaderA33->data.db = pMatchArray->m_Q;
		RVLMatrixHeaderB33->data.db = dMatrixA33;
		cvInvert(RVLMatrixHeaderB33,RVLMatrixHeaderA33,CV_SVD);
		
		

		//Check 1st geometrical constraint ie planes (the largest convex plane in the scene and in the model) are parallel
		MatrixMultiplication(pMatchArray->m_Q,pMatchArray->m_e,dMatrixA31, 3,3,1);
		constraint1 = RVLDotProduct(pMatchArray->m_e,dMatrixA31);

		double k = 3.66 / 11.34;

		//return (constraint1 <= m_epsilon1);

		MatchQuality = constraint1;

		if(constraint1 <= m_epsilon1)
		{
			double AccurateOrientation = (3.0 * DEG2RAD) * (3.0 * DEG2RAD);

			if(pPose->m_C[0] > AccurateOrientation || 
				pPose->m_C[4] > AccurateOrientation || 
				pPose->m_C[8] > AccurateOrientation)
				return TRUE;

			//Check 2nd geometrical constraint ie convex planes overlap (check all convex planes in both scene and model)
			//For each scene segment
			for(; pp3DConvexSegmentScene < pp3DConvexSegmentSceneEnd; pp3DConvexSegmentScene++)
			{
				p3DSceneSurface = *pp3DConvexSegmentScene;
				

				//for each model segment
				for(; pp3DConvexSegmentModel < pp3DConvexSegmentModelEnd; pp3DConvexSegmentModel++)
				{
					p3DModelSurface = *pp3DConvexSegmentModel;

					
					
					//*******Calculate e~ **************
					//Rot*tA + t - tB
					MatrixMultiplication(pPose->m_Rot, tA, dMatrixA31, 3, 3, 1);
					RVLSum3D(dMatrixA31,pPose->m_X,dMatrixB31);
					RVLDif3D(dMatrixB31,tB,e_);


					//*******Calculate CfA **************
					//RotA*diag(A_eigenVal)*RotA'
					memset(dMatrixA33,0,9*sizeof(double));
					dMatrixA33[0] = k * p3DSceneSurface->m_EigenValues[0] * p3DSceneSurface->m_EigenValues[0];
					dMatrixA33[4] = k * p3DSceneSurface->m_EigenValues[1] * p3DSceneSurface->m_EigenValues[1];

					MatrixMultiplication(p3DSceneSurface->m_Pose.m_Rot, dMatrixA33, dMatrixB33, 3, 3, 3);
					MatrixMultiplicationT(dMatrixB33, p3DSceneSurface->m_Pose.m_Rot, CfA, 3, 3, 3);

					//*******Calculate CfB **************
					//RotB*diag(B_eigenVal)*RotB'
					memset(dMatrixA33,0,9*sizeof(double));
					dMatrixA33[0] = k * p3DModelSurface->m_EigenValues[0] * p3DModelSurface->m_EigenValues[0];
					dMatrixA33[4] = k * p3DModelSurface->m_EigenValues[1] * p3DModelSurface->m_EigenValues[1];

					MatrixMultiplication(p3DModelSurface->m_Pose.m_Rot, dMatrixA33, dMatrixB33, 3, 3, 3);
					MatrixMultiplicationT(dMatrixB33, p3DModelSurface->m_Pose.m_Rot, CfB, 3, 3, 3);

					//*******Calculate C~ *****************
					// [J I]
					pPose->JacobianPTR2RotX(tA,dMatrixA31,J);
					memset(C_,0,18*sizeof(double));

					for(i=0;i<3;i++)
					{
						C_[i*6 + 0] = J[i*3 + 0];
						C_[i*6 + 1] = J[i*3 + 1];
						C_[i*6 + 2] = J[i*3 + 2];
					}
					C_[3] = 1;
					C_[10] = 1;
					C_[17] = 1;

					//*******Calculate Q~ *****************
					//inv(Rot*CfA*Rot' + CfB + C*P*C')
					//Assuming that pPose->m_C has already been defined using SetUncert
					//store pMatchArray->m_C from pPose->m_C

					MatrixMultiplication(pPose->m_Rot,CfA, dMatrixB33, 3, 3, 3);
					MatrixMultiplicationT(dMatrixB33,pPose->m_Rot, dMatrixA33, 3, 3, 3);

					MatrixMultiplication(C_,P, dMatrix36, 3, 6, 6);
					MatrixMultiplicationT(dMatrix36,C_, dMatrixB33, 3, 6, 3);

					//sum up all the results
					for(i=0;i<9;i++)
						dMatrixC33[i] = dMatrixA33[i] + CfB[i] + dMatrixB33[i];


					RVLMatrixHeaderA33->data.db = Q_;
					RVLMatrixHeaderB33->data.db = dMatrixC33;
					cvInvert(RVLMatrixHeaderB33,RVLMatrixHeaderA33,CV_SVD);


					//Check 2nd constraint
					MatrixMultiplication(Q_,e_,dMatrixB31, 3,3,1);
					constraint2 = RVLDotProduct(e_,dMatrixB31);
				
					if(constraint2 <= m_epsilon2)
					{
						MatchQuality = constraint2;

						//exit loop
						bMatch = true;
						break;
					}

					

				}


				//exit loop
				if(bMatch) 
					break;

			}
		
			
		}


	}
	
	return bMatch;	
}

BOOL CRVL3DSurface2::Match2(	CRVL3DObject *pMObject, 
							CRVL3DPose *pPose,
							double &MatchQuality, double &detQ, void *vpMatchData, DWORD Flags)
{
	//return FALSE;	// debug
	bool bMatch = false;

	CRVL3DSurface2 *pMSurface = (CRVL3DSurface2 *)pMObject;
	RVLSURFACE_MATCH_ARRAY *pMatchArray =(RVLSURFACE_MATCH_ARRAY *)vpMatchData;

	
	//Get convex surfaces (LEVEL2) for scene and model
	//Get the 1st convex surface for both scene and model - these are the first and largest in the list 
	//(Since the list is sorted in descending order)

#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
	RVLARRAY *pRelListSurfaceScene, *pRelListSurfaceModel;
	CRVL3DSurface2 **pp3DConvexSegmentScene , **pp3DConvexSegmentSceneEnd;
	CRVL3DSurface2 **pp3DConvexSegmentModel , **pp3DConvexSegmentModelEnd, **pp3DConvexSegmentModel2;

	CRVL3DSurface2 *p3DSceneSurface ,*p3DModelSurface;

	//Scene
	pRelListSurfaceScene = m_RelList + m_pClass->m_iRelListComponents;
	pp3DConvexSegmentScene = (CRVL3DSurface2 **)pRelListSurfaceScene->pFirst;
	pp3DConvexSegmentSceneEnd = (CRVL3DSurface2 **)pRelListSurfaceScene->pEnd;

	//Biggest surface in scene (1st in list)
	p3DSceneSurface = *pp3DConvexSegmentScene;
	
	//Model
	pRelListSurfaceModel = pMSurface->m_RelList + pMSurface->m_pClass->m_iRelListComponents;
	pp3DConvexSegmentModel = (CRVL3DSurface2 **)pRelListSurfaceModel->pFirst;
	pp3DConvexSegmentModelEnd = (CRVL3DSurface2 **)pRelListSurfaceModel->pEnd;

	//Biggest surface in model  (1st in list)
	p3DModelSurface = *pp3DConvexSegmentModel;
#else
	CRVL3DSurface2 *p3DSceneSurface = this;
	CRVL3DSurface2 *p3DModelSurface = pMSurface;
#endif

	//Using only the biggest surfaces calculate matrices and check 1st constraint

	//double dMatrixA31[3];
	//double dMatrixB31[3];
	double dMatrixA33[9];
	//double dMatrixB33[9];
	
	//define constant vectors
	double E[6], E_[2];

	double e_[3];
	double CfA[9], CfB[9];
	double J[9];
	double Q_[9];
	double tF_BA[3];

	double constraint0, constraint1, constraint2;

	double *R = pPose->m_Rot;
	double *t = pPose->m_X;
	double *invt = (double *)(pPose->m_pData);
	double *PR = pPose->m_C;
	double *PRt = pPose->m_C + 3 * 3;
	double *Pt = pPose->m_C + 2 * 3 * 3;
	double ca = pPose->m_ca;
	double sa = pPose->m_sa;

	double *RA = p3DSceneSurface->m_Pose.m_Rot;
	double *tA = p3DSceneSurface->m_Pose.m_X;
	double *zA = p3DSceneSurface->m_N;
	double *CqA = p3DSceneSurface->m_varq;

	double *RB = p3DModelSurface->m_Pose.m_Rot;
	double *tB = p3DModelSurface->m_Pose.m_X;
	double *zB = p3DModelSurface->m_N;
	double *CqB = p3DModelSurface->m_varq;

	double *e = pMatchArray->m_e;
	double *C = pMatchArray->m_C;
	double *Q = pMatchArray->m_Q;

	////make sure the angle between normals (ie nb and na OR zA and zB) is less than 90
	//constraint0 = RVLMULCOLCOL3(RA, RB, 2, 2);

	//if(constraint0 <= 0)
	//	return FALSE;

	// RAB = [xAB, yAB, zAB] = Rot * RA

	double xAB[3], yAB[3], zAB[3];

	RVLMULMX3X3VECT(R, zA, zAB)

	//make sure the angle between normals (ie nb and Rot*na OR zA and zB) is less than 90
	constraint0 = RVLDOTPRODUCT3(zAB, zB);

	if(constraint0 <= 0)
		return FALSE;

	RVLMULMXCOL3(R, RA, 0, xAB)
	RVLMULMXCOL3(R, RA, 1, yAB)

	// tF_BA = tA + Rot' * t

	RVLSUM3VECTORS(tA, invt, tF_BA)

	// tF2_AB = tB - t

	double tF2_AB[3];

	RVLDIF3VECTORS(tB, t, tF2_AB)

	//*******Calculate e**************

	//[ex ey]' = [xB yB]'*Rot*zA

	e[0] = RVLMULVECTORCOL3(zAB, RB, 0);
	e[1] = RVLMULVECTORCOL3(zAB, RB, 1);

	//ez = tA'*zA + (t - tB)' * Rot*zA = dA - (tB - t)' * zAB

	e[2] = p3DSceneSurface->m_d - RVLDOTPRODUCT3(tF2_AB, zAB);

	//*******Calculate C *****************
	//equation (13) EKF
	RVLJACOBIANPTRTOROTX(zA, R, ca, sa, zAB, J)

	RVLMXEL(C, 3, 0, 0) = RVLMULCOLCOL3(RB,J,0,0);
	RVLMXEL(C, 3, 0, 1) = RVLMULCOLCOL3(RB,J,0,1);
	RVLMXEL(C, 3, 0, 2) = RVLMULCOLCOL3(RB,J,0,2);
	RVLMXEL(C, 3, 1, 0) = RVLMULCOLCOL3(RB,J,1,0);
	RVLMXEL(C, 3, 1, 1) = RVLMULCOLCOL3(RB,J,1,1);
	RVLMXEL(C, 3, 1, 2) = RVLMULCOLCOL3(RB,J,1,2);
	RVLMXEL(C, 3, 2, 0) = -RVLMULROWCOL3(tF2_AB,J,0,0);
	RVLMXEL(C, 3, 2, 1) = -RVLMULROWCOL3(tF2_AB,J,0,1);
	RVLMXEL(C, 3, 2, 2) = -RVLMULROWCOL3(tF2_AB,J,0,2);

	//*******Calculate E *****************

	RVLMXEL(E, 2, 0, 0) = RVLMULROWCOL3(xAB, RB, 0, 0);
	RVLMXEL(E, 2, 0, 1) = RVLMULROWCOL3(yAB, RB, 0, 0);
	RVLMXEL(E, 2, 1, 0) = RVLMULROWCOL3(xAB, RB, 0, 1);
	RVLMXEL(E, 2, 1, 1) = RVLMULROWCOL3(yAB, RB, 0, 1);
	RVLMXEL(E, 2, 2, 0) = RVLMULROWCOL3(tA, RA, 0, 0) - RVLDOTPRODUCT3(tF2_AB, xAB);
	RVLMXEL(E, 2, 2, 1) = RVLMULROWCOL3(tA, RA, 0, 1) - RVLDOTPRODUCT3(tF2_AB, yAB);

	//*******Calculate E_ *****************

	RVLMXEL(E_, 2, 0, 0) = -RVLMULROWCOL3(tB, RB, 0, 0);
	RVLMXEL(E_, 2, 0, 1) = -RVLMULROWCOL3(tB, RB, 0, 1);

	//*******Calculate Q *****************
	//equation (11) EKF
	//E*CqA*E' + E_*CqB*E_'
	double k00 = CqA[0]*E[0];
	double k11 = CqA[1]*E[1];
	double k02 = CqA[0]*E[2];

	RVLMatrixA33[0] = k00*E[0] + k11*E[1] + CqB[0];
	RVLMatrixA33[1] = k00*E[2] + k11*E[3];
	RVLMatrixA33[2] = k00*E[4] + k11*E[5];
	RVLMatrixA33[4] = k02*E[2] + CqA[1]*E[3]*E[3] + CqB[1];
	RVLMatrixA33[5] = k02*E[4] + CqA[1]*E[3]*E[5];
	RVLMatrixA33[8] = CqA[0]*E[4]*E[4] + CqA[1]*E[5]*E[5] + CqA[2] + CqB[2];

	//C*P*C'
	double k22 = RVLCOV3DTRANSFTO1D(Pt, zAB);
	RVLCOV3DTRANSF(PR, C, RVLMatrixB33, RVLMatrixC33);
	RVLMULMX3X3VECT(PRt, zAB, RVLVector3);
	double V[3];
	RVLMULMX3X3VECT(C, RVLVector3, V);
	RVLMatrixA33[0] += (RVLMatrixB33[0]);
	RVLMatrixA33[1] += (RVLMatrixB33[1]);
	RVLMatrixA33[2] += (RVLMatrixB33[2] + V[0]);
	RVLMatrixA33[4] += (RVLMatrixB33[4]);
	RVLMatrixA33[5] += (RVLMatrixB33[5] + V[1]);
	RVLMatrixA33[8] += (RVLMatrixB33[8] + 2 * V[2] + k22);

	//invert Q
	//double detQ;
	RVLINVCOV3(RVLMatrixA33, Q, detQ)

	//Check 1st geometrical constraint ie planes (the largest convex plane in the scene and in the model) are parallel and close 
	constraint1 = RVLCOV3DTRANSFTO1D(Q, e);

	MatchQuality = constraint1;
	//MatchQuality = exp(-constraint1/2) / sqrt(8 * PI * PI * PI * detQ);

	//return (constraint1 <= m_epsilon1);

	if(constraint1 > m_epsilon1)
		return FALSE;

	if(Flags == RVL3DSURFACE_FLAG_HYPOTHESIS_EVALUATION) 
		return TRUE;

	// This is for the old version of EKF and should be later removed !!!!

	C[12] = C[6];
	C[13] = C[7];
	C[14] = C[8];
	C[6] = C[3];
	C[7] = C[4];
	C[8] = C[5];
	C[3] = 0.0;
	C[4] = 0.0;
	C[5] = 0.0;
	C[9] = 0.0;
	C[10] = 0.0;
	C[11] = 0.0;
	C[15] = zAB[0];
	C[16] = zAB[1];
	C[17] = zAB[2];
	RVLCOMPLETESIMMX3(Q)

	// if the orientation is not sufficiently accuratelly estimated,
	// then there is not much sense in evaluating constraint2

	//double AccurateOrientation = (3.0 * DEG2RAD) * (3.0 * DEG2RAD);

	//if(pPose->m_C[0] > AccurateOrientation || 
	//	pPose->m_C[4] > AccurateOrientation || 
	//	pPose->m_C[8] > AccurateOrientation)
	//	return TRUE;

	//double k = m_pClass->m_UncertCoeff * m_pClass->m_UncertCoeff * 3.66 / 11.34;
	//double k = m_pClass->m_UncertCoeff * m_pClass->m_UncertCoeff;
	double k = m_pClass->m_OverlapCoeff * m_pClass->m_OverlapCoeff * 2.3660 / 11.34;

	double vart0, vart1;
	double k03, k14;

	//Check 2nd geometrical constraint ie convex planes overlap (check all convex planes in both scene and model)
#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
	//For each scene segment
	for(; pp3DConvexSegmentScene < pp3DConvexSegmentSceneEnd; pp3DConvexSegmentScene++)
	{
		p3DSceneSurface = *pp3DConvexSegmentScene;
#endif
	
		tA = p3DSceneSurface->m_Pose.m_X;
		
#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
		//for each model segment
		for(pp3DConvexSegmentModel2 = pp3DConvexSegmentModel; pp3DConvexSegmentModel2 < pp3DConvexSegmentModelEnd; 
			pp3DConvexSegmentModel2++)
		{
			p3DModelSurface = *pp3DConvexSegmentModel2;
#endif
					
			//*******Calculate e~ **************
			//Rot*tA + t - tB
			tB = p3DModelSurface->m_Pose.m_X;
			RVLMULMX3X3VECT(R, tA, RVLVector3)
			RVLSUM3VECTORS(RVLVector3, t, e_)
			RVLDIF3VECTORS(e_, tB, e_)

			//*******Calculate CfA **************
			//RotA*diag(A_eigenVal)*RotA'
			RA = p3DSceneSurface->m_Pose.m_Rot;
			vart0 = k * p3DSceneSurface->m_EigenValues[0] * p3DSceneSurface->m_EigenValues[0];
			vart1 = k * p3DSceneSurface->m_EigenValues[1] * p3DSceneSurface->m_EigenValues[1];
			k00 = vart0*RA[0];
			k11 = vart1*RA[1];
			k03 = vart0*RA[3];
			k14 = vart1*RA[4];
			CfA[0]=k00*RA[0] + k11*RA[1];
			CfA[1]=k00*RA[3] + k11*RA[4];
			CfA[2]=k00*RA[6] + k11*RA[7];
			CfA[4]=k03*RA[3] + k14*RA[4];
			CfA[5]=k03*RA[6] + k14*RA[7];
			CfA[8]=vart0*RA[6]*RA[6] + vart1*RA[7]*RA[7];

			//*******Calculate CfB **************
			//RotB*diag(B_eigenVal)*RotB'
			RB = p3DModelSurface->m_Pose.m_Rot;
			vart0 = k * p3DModelSurface->m_EigenValues[0] * p3DModelSurface->m_EigenValues[0];
			vart1 = k * p3DModelSurface->m_EigenValues[1] * p3DModelSurface->m_EigenValues[1];
			k00 = vart0*RB[0];
			k11 = vart1*RB[1];
			k03 = vart0*RB[3];
			k14 = vart1*RB[4];
			CfB[0]=k00*RB[0] + k11*RB[1];
			CfB[1]=k00*RB[3] + k11*RB[4];
			CfB[2]=k00*RB[6] + k11*RB[7];
			CfB[4]=k03*RB[3] + k14*RB[4];
			CfB[5]=k03*RB[6] + k14*RB[7];
			CfB[8]=vart0*RB[6]*RB[6] + vart1*RB[7]*RB[7];
			
			//*******Calculate C~ *****************
			// C~ = [J I]
			RVLJACOBIANPTRTOROTX(tA, R, ca, sa, RVLVector3, J)

			//*******Calculate Q~ *****************
			//inv(Rot*CfA*Rot' + CfB + C*P*C')
			//Assuming that pPose->m_C has already been defined using SetUncert
			//store pMatchArray->m_C from pPose->m_C

			RVLCOV3DTRANSF(CfA, R, dMatrixA33, RVLMatrixB33)

			RVLCOV3DTRANSF(PR, J, RVLMatrixA33, RVLMatrixB33)
			RVLMXMUL3X3(J, PRt, RVLMatrixB33)
			RVLSUMMX3X3T2UT(RVLMatrixB33, RVLMatrixB33, RVLMatrixC33)
			RVLMatrixA33[0] += (RVLMatrixC33[0] + Pt[0] + dMatrixA33[0] + CfB[0]);
			RVLMatrixA33[1] += (RVLMatrixC33[1] + Pt[1] + dMatrixA33[1] + CfB[1]);
			RVLMatrixA33[2] += (RVLMatrixC33[2] + Pt[2] + dMatrixA33[2] + CfB[2]);
			RVLMatrixA33[4] += (RVLMatrixC33[4] + Pt[4] + dMatrixA33[4] + CfB[4]);
			RVLMatrixA33[5] += (RVLMatrixC33[5] + Pt[5] + dMatrixA33[5] + CfB[5]);
			RVLMatrixA33[8] += (RVLMatrixC33[8] + Pt[8] + dMatrixA33[8] + CfB[8]);
			RVLINVCOV3(RVLMatrixA33, Q_, detQ)

			//Check 2nd constraint
			constraint2 = RVLCOV3DTRANSFTO1D(Q_, e_);
		
			if(constraint2 <= m_epsilon2)
			{
				//MatchQuality = constraint2;

				bMatch = true;

#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
				//exit loop
				break;
#endif
			}
#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
		}	//for each model segment
		
		//exit loop
		if(bMatch) 
			break;
	}	//For each scene segment
#endif

	return bMatch;	
}



BOOL CRVL3DSurface2::Match3(	CRVL3DObject *pMObject, 
							CRVL3DPose *pPose,
							double &MatchQuality, void *vpMatchData)
{
	//return FALSE;	// debug
	bool bMatch = false;

	CRVL3DSurface2 *pMSurface = (CRVL3DSurface2 *)pMObject;
	RVLSURFACE_MATCH_ARRAY *pMatchArray =(RVLSURFACE_MATCH_ARRAY *)vpMatchData;

	
	//Get convex surfaces (LEVEL2) for scene and model
	//Get the 1st convex surface for both scene and model - these are the first and largest in the list 
	//(Since the list is sorted in descending order)

#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
	RVLARRAY *pRelListSurfaceScene, *pRelListSurfaceModel;
	CRVL3DSurface2 **pp3DConvexSegmentScene , **pp3DConvexSegmentSceneEnd;
	CRVL3DSurface2 **pp3DConvexSegmentModel , **pp3DConvexSegmentModelEnd, **pp3DConvexSegmentModel2;

	CRVL3DSurface2 *p3DSceneSurface ,*p3DModelSurface;

	//Scene
	pRelListSurfaceScene = m_RelList + m_pClass->m_iRelListComponents;
	pp3DConvexSegmentScene = (CRVL3DSurface2 **)pRelListSurfaceScene->pFirst;
	pp3DConvexSegmentSceneEnd = (CRVL3DSurface2 **)pRelListSurfaceScene->pEnd;

	//Biggest surface in scene (1st in list)
	p3DSceneSurface = *pp3DConvexSegmentScene;
	
	//Model
	pRelListSurfaceModel = pMSurface->m_RelList + pMSurface->m_pClass->m_iRelListComponents;
	pp3DConvexSegmentModel = (CRVL3DSurface2 **)pRelListSurfaceModel->pFirst;
	pp3DConvexSegmentModelEnd = (CRVL3DSurface2 **)pRelListSurfaceModel->pEnd;

	//Biggest surface in model  (1st in list)
	p3DModelSurface = *pp3DConvexSegmentModel;
#else
	CRVL3DSurface2 *p3DSceneSurface = this;
	CRVL3DSurface2 *p3DModelSurface = pMSurface;
#endif

	//Using only the biggest surfaces calculate matrices and check 1st constraint

	//double dMatrixA31[3];
	//double dMatrixB31[3];
	double dMatrixA33[9];
	//double dMatrixB33[9];
	
	//define constant vectors
	double E[6], E_[2];

	double e_[3];
	double CfA[9], CfB[9];
	double J[9];
	double Q_[9];
	double tF_BA[3];

	double constraint0, constraint1, constraint2;

	double *R = pPose->m_Rot;
	double *t = pPose->m_X;
	double *invt = (double *)(pPose->m_pData);
	double *PR = pPose->m_C;
	double *PRt = pPose->m_C + 3 * 3;
	double *Pt = pPose->m_C + 2 * 3 * 3;
	double ca = pPose->m_ca;
	double sa = pPose->m_sa;

	double *RA = p3DSceneSurface->m_Pose.m_Rot;
	double *tA = p3DSceneSurface->m_Pose.m_X;
	double *zA = p3DSceneSurface->m_N;
	double *CqA = p3DSceneSurface->m_varq;

	double *RB = p3DModelSurface->m_Pose.m_Rot;
	double *tB = p3DModelSurface->m_Pose.m_X;
	double *zB = p3DModelSurface->m_N;
	double *CqB = p3DModelSurface->m_varq;

	double *e = pMatchArray->m_e;
	double *C = pMatchArray->m_C;
	double *Q = pMatchArray->m_Q;

	//make sure the angle between normals (ie nb and Rot*na OR zA and zB) is less than 90
	constraint0 = RVLMULCOLCOL3(RA, RB, 2, 2);

	if(constraint0 <= 0)
		return FALSE;

	double xAB[3], yAB[3], zAB[3];

	RVLMULMXCOL3(R, RA, 0, xAB)
	RVLMULMXCOL3(R, RA, 1, yAB)
	RVLMULMX3X3VECT(R, zA, zAB)

	RVLSUM3VECTORS(tA, invt, tF_BA)

	//*******Calculate e**************

	//[ex ey]' = [xB yB]'*Rot*zA

	e[0] = RVLMULVECTORCOL3(zAB, RB, 0);
	e[1] = RVLMULVECTORCOL3(zAB, RB, 1);

	//ez = (tA + Rot'*t)'*zA - tB'*zB = dA + t'*Rot*zA + dB

	e[2] = RVLDOTPRODUCT3(t, zAB) + p3DSceneSurface->m_d - p3DModelSurface->m_d;

	//*******Calculate C *****************
	//equation (13) EKF
	RVLJACOBIANPTRTOROTX(zA, R, ca, sa, zAB, J)

	RVLMXEL(C, 3, 0, 0) = RVLMULCOLCOL3(RB,J,0,0);
	RVLMXEL(C, 3, 0, 1) = RVLMULCOLCOL3(RB,J,0,1);
	RVLMXEL(C, 3, 0, 2) = RVLMULCOLCOL3(RB,J,0,2);
	RVLMXEL(C, 3, 1, 0) = RVLMULCOLCOL3(RB,J,1,0);
	RVLMXEL(C, 3, 1, 1) = RVLMULCOLCOL3(RB,J,1,1);
	RVLMXEL(C, 3, 1, 2) = RVLMULCOLCOL3(RB,J,1,2);
	RVLMXEL(C, 3, 2, 0) = RVLMULROWCOL3(t,J,0,0);
	RVLMXEL(C, 3, 2, 1) = RVLMULROWCOL3(t,J,0,1);
	RVLMXEL(C, 3, 2, 2) = RVLMULROWCOL3(t,J,0,2);

	//*******Calculate E *****************
	
	RVLMXEL(E, 2, 0, 0) = RVLMULROWCOL3(xAB, RB, 0, 0);
	RVLMXEL(E, 2, 0, 1) = RVLMULROWCOL3(yAB, RB, 0, 0);
	RVLMXEL(E, 2, 1, 0) = RVLMULROWCOL3(xAB, RB, 0, 1);
	RVLMXEL(E, 2, 1, 1) = RVLMULROWCOL3(yAB, RB, 0, 1);
	RVLMXEL(E, 2, 2, 0) = RVLMULROWCOL3(tF_BA, RA, 0, 0);
	RVLMXEL(E, 2, 2, 1) = RVLMULROWCOL3(tF_BA, RA, 0, 1);

	//*******Calculate E_ *****************

	RVLMXEL(E_, 2, 0, 0) = -RVLMULROWCOL3(tB, RB, 0, 0);
	RVLMXEL(E_, 2, 0, 1) = -RVLMULROWCOL3(tB, RB, 0, 1);

	//*******Calculate Q *****************
	//equation (11) EKF
	//E*CqA*E' + E_*CqB*E_'
	double k00 = CqA[0]*E[0];
	double k11 = CqA[1]*E[1];
	double k02 = CqA[0]*E[2];
	double k01 = CqB[0]*E_[0];
	double k12 = CqB[1]*E_[1];

	RVLMatrixA33[0] = k00*E[0] + k11*E[1] + CqB[0];
	RVLMatrixA33[1] = k00*E[2] + k11*E[3];
	RVLMatrixA33[2] = k00*E[4] + k11*E[5] - k01;
	RVLMatrixA33[4] = k02*E[2] + CqA[1]*E[3]*E[3] + CqB[1];
	RVLMatrixA33[5] = k02*E[4] + CqA[1]*E[3]*E[5] - k12;
	RVLMatrixA33[8] = CqA[0]*E[4]*E[4] + CqA[1]*E[5]*E[5] + CqA[2] + k01*E_[0] + k12*E_[1] + CqB[2];

	//C*P*C'
	double k22 = RVLCOV3DTRANSFTO1D(Pt, zAB);
	RVLCOV3DTRANSF(PR, C, RVLMatrixB33, RVLMatrixC33);
	RVLMULMX3X3VECT(PRt, zAB, RVLVector3);
	double V[3];
	RVLMULMX3X3VECT(C, RVLVector3, V);
	RVLMatrixA33[0] += (RVLMatrixB33[0]);
	RVLMatrixA33[1] += (RVLMatrixB33[1]);
	RVLMatrixA33[2] += (RVLMatrixB33[2] + V[0]);
	RVLMatrixA33[4] += (RVLMatrixB33[4]);
	RVLMatrixA33[5] += (RVLMatrixB33[5] + V[1]);
	RVLMatrixA33[8] += (RVLMatrixB33[8] + 2 * V[2] + k22);

	//invert Q
	double detQ;
	RVLINVCOV3(RVLMatrixA33, Q, detQ)

	//Check 1st geometrical constraint ie planes (the largest convex plane in the scene and in the model) are parallel
	constraint1 = RVLCOV3DTRANSFTO1D(Q, e);

	MatchQuality = constraint1;

	//return (constraint1 <= m_epsilon1);

	if(constraint1 > m_epsilon1)
		return FALSE;

	// This is for the old version of EKF and should be later removed !!!!

	C[12] = C[6];
	C[13] = C[7];
	C[14] = C[8];
	C[6] = C[3];
	C[7] = C[4];
	C[8] = C[5];
	C[3] = 0.0;
	C[4] = 0.0;
	C[5] = 0.0;
	C[9] = 0.0;
	C[10] = 0.0;
	C[11] = 0.0;
	C[15] = zAB[0];
	C[16] = zAB[1];
	C[17] = zAB[2];
	RVLCOMPLETESIMMX3(Q)

	// if the orientation is not sufficiently accuratelly estimated,
	// then there is not much sense in evaluating constraint2

	//double AccurateOrientation = (3.0 * DEG2RAD) * (3.0 * DEG2RAD);

	//if(pPose->m_C[0] > AccurateOrientation || 
	//	pPose->m_C[4] > AccurateOrientation || 
	//	pPose->m_C[8] > AccurateOrientation)
	//	return TRUE;

	double k = m_pClass->m_OverlapCoeff * m_pClass->m_OverlapCoeff * 3.66 / 11.34;

	double vart0, vart1;
	double k03, k14;

	//Check 2nd geometrical constraint ie convex planes overlap (check all convex planes in both scene and model)
#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
	//For each scene segment
	for(; pp3DConvexSegmentScene < pp3DConvexSegmentSceneEnd; pp3DConvexSegmentScene++)
	{
		p3DSceneSurface = *pp3DConvexSegmentScene;
#endif
	
		tA = p3DSceneSurface->m_Pose.m_X;
		
#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
		//for each model segment
		for(pp3DConvexSegmentModel2 = pp3DConvexSegmentModel; pp3DConvexSegmentModel2 < pp3DConvexSegmentModelEnd; 
			pp3DConvexSegmentModel2++)
		{
			p3DModelSurface = *pp3DConvexSegmentModel2;
#endif
					
			//*******Calculate e~ **************
			//Rot*tA + t - tB
			tB = p3DModelSurface->m_Pose.m_X;
			RVLMULMX3X3VECT(R, tA, RVLVector3)
			RVLSUM3VECTORS(RVLVector3, t, e_)
			RVLDIF3VECTORS(e_, tB, e_)

			//*******Calculate CfA **************
			//RotA*diag(A_eigenVal)*RotA'
			RA = p3DSceneSurface->m_Pose.m_Rot;
			vart0 = k * p3DSceneSurface->m_EigenValues[0] * p3DSceneSurface->m_EigenValues[0];
			vart1 = k * p3DSceneSurface->m_EigenValues[1] * p3DSceneSurface->m_EigenValues[1];
			k00 = vart0*RA[0];
			k11 = vart1*RA[1];
			k03 = vart0*RA[3];
			k14 = vart1*RA[4];
			CfA[0]=k00*RA[0] + k11*RA[1];
			CfA[1]=k00*RA[3] + k11*RA[4];
			CfA[2]=k00*RA[6] + k11*RA[7];
			CfA[4]=k03*RA[3] + k14*RA[4];
			CfA[5]=k03*RA[6] + k14*RA[7];
			CfA[8]=vart0*RA[6]*RA[6] + vart1*RA[7]*RA[7];

			//*******Calculate CfB **************
			//RotB*diag(B_eigenVal)*RotB'
			RB = p3DModelSurface->m_Pose.m_Rot;
			vart0 = k * p3DModelSurface->m_EigenValues[0] * p3DModelSurface->m_EigenValues[0];
			vart1 = k * p3DModelSurface->m_EigenValues[1] * p3DModelSurface->m_EigenValues[1];
			k00 = vart0*RB[0];
			k11 = vart1*RB[1];
			k03 = vart0*RB[3];
			k14 = vart1*RB[4];
			CfB[0]=k00*RB[0] + k11*RB[1];
			CfB[1]=k00*RB[3] + k11*RB[4];
			CfB[2]=k00*RB[6] + k11*RB[7];
			CfB[4]=k03*RB[3] + k14*RB[4];
			CfB[5]=k03*RB[6] + k14*RB[7];
			CfB[8]=vart0*RB[6]*RB[6] + vart1*RB[7]*RB[7];
			
			//*******Calculate C~ *****************
			// C~ = [J I]
			RVLJACOBIANPTRTOROTX(tA, R, ca, sa, RVLVector3, J)

			//*******Calculate Q~ *****************
			//inv(Rot*CfA*Rot' + CfB + C*P*C')
			//Assuming that pPose->m_C has already been defined using SetUncert
			//store pMatchArray->m_C from pPose->m_C

			RVLCOV3DTRANSF(CfA, R, dMatrixA33, RVLMatrixB33)

			RVLCOV3DTRANSF(PR, J, RVLMatrixA33, RVLMatrixB33)
			RVLMXMUL3X3(J, PRt, RVLMatrixB33)
			RVLSUMMX3X3T2UT(RVLMatrixB33, RVLMatrixB33, RVLMatrixC33)
			RVLMatrixA33[0] += (RVLMatrixC33[0] + Pt[0] + dMatrixA33[0] + CfB[0]);
			RVLMatrixA33[1] += (RVLMatrixC33[1] + Pt[1] + dMatrixA33[1] + CfB[1]);
			RVLMatrixA33[2] += (RVLMatrixC33[2] + Pt[2] + dMatrixA33[2] + CfB[2]);
			RVLMatrixA33[4] += (RVLMatrixC33[4] + Pt[4] + dMatrixA33[4] + CfB[4]);
			RVLMatrixA33[5] += (RVLMatrixC33[5] + Pt[5] + dMatrixA33[5] + CfB[5]);
			RVLMatrixA33[8] += (RVLMatrixC33[8] + Pt[8] + dMatrixA33[8] + CfB[8]);
			RVLINVCOV3(RVLMatrixA33, Q_, detQ)

			//Check 2nd constraint
			constraint2 = RVLCOV3DTRANSFTO1D(Q_, e_);
		
			if(constraint2 <= m_epsilon2)
			{
				//MatchQuality = constraint2;

				bMatch = true;

#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
				//exit loop
				break;
#endif
			}
#ifdef RVL3DSURFACE_MATCH_CONVEX_SEGMENTS
		}	//for each model segment
		
		//exit loop
		if(bMatch) 
			break;
	}	//For each scene segment
#endif

	return bMatch;	
}

void CRVL3DSurface2::TransfToMatchRefFrame(double *RFT,
										   double *RPT,
										   double *CP)
{
	double *XF = RFT;
	double *YF = RFT + 3;
	double *ZF = RFT + 6;

	double *XP = RPT;
	double *YP = RPT + 3;
	double *ZP = RPT + 6;

	double J[2*2];

	J[0] = RVLDOTPRODUCT3(XP, XF);
	J[1] = RVLDOTPRODUCT3(XP, YF);
	J[2] = RVLDOTPRODUCT3(YP, XF);
	J[3] = RVLDOTPRODUCT3(YP, YF);

	double varx = m_varq[0];
	double vary = m_varq[1];

	CP[0] = varx*J[0]*J[0] + vary*J[1]*J[1];
	CP[1] = varx*J[2]*J[0] + vary*J[3]*J[1];
	CP[3] = varx*J[2]*J[2] + vary*J[3]*J[3];
}

bool CRVL3DSurface2::Match4(CRVL3DObject *pObject_, 
							RVL3DSURFACE2_MATCH_DATA *pData,
							double &MatchQuality,
							DWORD Flags)
{
	CRVL3DSurface2 *pSurf_ = (CRVL3DSurface2 *)pObject_;

	// coarse normal orientation match

	double *N_ = pSurf_->m_N;

	double en = RVLDOTPRODUCT3(m_N, N_);

	if(en < COS45)
		return false;

	// coarse overlap match

	double *tF = m_Pose.m_X;
	double *tF_ = pSurf_->m_Pose.m_X;

	double *Cp, *Cp_;
	double Mx3x3Tmp[3*3], Mx3x3Tmp2[3*3], Vect3Tmp[3];
	double tP[3];
	double fTmp;

	if(Flags & RVL3DSURFACE_MATCH4_FLAG_OVERLAP)
	{
		Cp = pData->Cp + 3 * 3 * m_Index;
		Cp_ = pData->Cp_ + 3 * 3 * pSurf_->m_Index;

		RVLSUMMX3X3UT(Cp, Cp_, Mx3x3Tmp)

		// debug

		//if(m_Index == 9 && pSurf_->m_Index == 7)
		//{
		//	FILE *fpDebug = fopen("C:\\RVL\\Debug\\Mx.dat", "w");

		//	RVLCOMPLETESIMMX3(Mx3x3Tmp)

		//	RVLPrintMatrix(fpDebug, Mx3x3Tmp, 3, 3);

		//	fclose(fpDebug);
		//}

		/////

		double Et[3];

		RVLDIF3VECTORS(tF, tF_, Et)

		RVLINVCOV3(Mx3x3Tmp, Mx3x3Tmp2, fTmp)

		double e = RVLCOV3DTRANSFTO1D(Mx3x3Tmp2, Et);

		if(e > 11.34)		// Surface overlap threshold
		//if(e > 4.11)
			return false;
		
		// compute the origin of the match reference frame

		double *invCp = pData->invCp + 3 * 3 * m_Index;
		double *invCp_ = pData->invCp_ + 3 * 3 * pSurf_->m_Index;

		RVLSUMMX3X3UT(invCp, invCp_, Mx3x3Tmp)

		RVLINVCOV3(Mx3x3Tmp, Mx3x3Tmp2, fTmp)

		double Vect3Tmp2[3];

		RVLMULCOV3VECT(invCp, tF, Vect3Tmp)
		RVLMULCOV3VECT(invCp_, tF_, Vect3Tmp2)
		RVLSUM3VECTORS(Vect3Tmp, Vect3Tmp2, Vect3Tmp)

		RVLMULCOV3VECT(Mx3x3Tmp2, Vect3Tmp, tP)
	}

	// compute the z-axis of the match reference frame

	double RPT[3*3];
	double *XP = RPT;
	double *YP = RPT + 3;
	double *ZP = RPT + 6;

	double *RF = m_Pose.m_Rot;
	double RFT[3*3];
	double *ZF = RFT + 6;
	RVLCOPYMX3X3T(RF, RFT)

	double *RF_ = pSurf_->m_Pose.m_Rot;
	double RFT_[3*3];
	double *ZF_ = RFT_ + 6;
	RVLCOPYMX3X3T(RF_, RFT_)

	RVLSUM3VECTORS(ZF, ZF_, ZP)
	RVLNORM3(ZP, fTmp)

	double varqS = pSurf_->m_varq[2] + m_varq[2] + pData->varPositionUncert;

	double er, ep;

	if(Flags & RVL3DSURFACE_MATCH4_FLAG_OVERLAP)
	{
		// proximity match

		RVLDIF3VECTORS(tF, tP, Vect3Tmp)
		double r = RVLDOTPRODUCT3(ZF, Vect3Tmp) / RVLDOTPRODUCT3(ZF, ZP);

		RVLDIF3VECTORS(tF_, tP, Vect3Tmp)
		double r_ = RVLDOTPRODUCT3(ZF_, Vect3Tmp) / RVLDOTPRODUCT3(ZF_, ZP);	

		er = r - r_;

		ep = er * er / varqS;

		if(ep > 6.635)
			return false;
	}
	//else
	//{
	//	// ZM <- eigenvector corresponding to the smallest eigenvalue of the covariance matrix
	//	//       of the union of the endpoints of the ellipses representing the two matched surface segments

	//	double *XF = RFT;
	//	double *YF = RFT + 3;

	//	fTmp = m_EigenValues[0] * m_EigenValues[0];
	//	RVLVECTCOV3(XF, Mx3x3Tmp)
	//	RVLSCALEMX3X3(Mx3x3Tmp, fTmp, Mx3x3Tmp2)

	//	fTmp = m_EigenValues[1] * m_EigenValues[1];
	//	RVLVECTCOV3(YF, Mx3x3Tmp)
	//	RVLSCALEMX3X3(Mx3x3Tmp, fTmp, Mx3x3Tmp)
	//	RVLSUMMX3X3(Mx3x3Tmp2, Mx3x3Tmp, Mx3x3Tmp2) 

	//	XF = RFT_;
	//	YF = RFT_ + 3;

	//	fTmp = pSurf_->m_EigenValues[0] * pSurf_->m_EigenValues[0];
	//	RVLVECTCOV3(XF, Mx3x3Tmp)
	//	RVLSCALEMX3X3(Mx3x3Tmp, fTmp, Mx3x3Tmp)
	//	RVLSUMMX3X3(Mx3x3Tmp2, Mx3x3Tmp, Mx3x3Tmp2) 

	//	fTmp = pSurf_->m_EigenValues[1] * pSurf_->m_EigenValues[1];
	//	RVLVECTCOV3(YF, Mx3x3Tmp)
	//	RVLSCALEMX3X3(Mx3x3Tmp, fTmp, Mx3x3Tmp)
	//	RVLSUMMX3X3(Mx3x3Tmp2, Mx3x3Tmp, Mx3x3Tmp2) 

	//	RVLDIF3VECTORS(tF_, tF, Vect3Tmp)
	//	RVLVECTCOV3(Vect3Tmp, Mx3x3Tmp)
	//	RVLSUMMX3X3(Mx3x3Tmp2, Mx3x3Tmp, Mx3x3Tmp2) 

	//	bool bReal[3];
	//	double ZM[3];
	//	
	//	RVLGetMinEigVector3(Mx3x3Tmp2, Vect3Tmp, bReal, ZM);
	//}

	// compute the x and y-axes of the match reference frame

	RVLCROSSPRODUCT3(ZF, ZF_, XP)

	fTmp = RVLDOTPRODUCT3(XP, XP);

	int i, j, k;

	if (fTmp <= APPROX_ZERO)
		//RVLORTHOGONAL3(ZP, XP, i, j, k, Vect3Tmp, fTmp)
		RVLORTHOGONAL3(ZP, XP, i, j, k, fTmp)
	else
	{
		fTmp = sqrt(fTmp);

		RVLSCALE3VECTOR2(XP, fTmp, XP)
	}

	RVLCROSSPRODUCT3(ZP, XP, YP);

	// transform the matched surfaces into the match reference frame

	double sy = RVLDOTPRODUCT3(ZF, YP);

	double Cn[2*2];
	TransfToMatchRefFrame(RFT, RPT, Cn);

	double Cn_[2*2];
	pSurf_->TransfToMatchRefFrame(RFT_, RPT, Cn_);

	// orientation probability

	double CnS[2*2];

	CnS[0] = Cn[0] + Cn_[0] + pData->varOrientationUncert;
	CnS[1] = Cn[1] + Cn_[1];
	CnS[3] = Cn[3] + Cn_[3] + pData->varOrientationUncert;

	double detCnS = RVLDET2(CnS);

	double Pn;

	if(detCnS > 4.0)
		Pn = 0.0;
	else
	{
		en = CnS[0] * 4.0 * sy * sy / detCnS;

		//if(!(Flags & RVL3DSURFACE_MATCH4_FLAG_OVERLAP))
			if(en > 9.21)
				return false;

		Pn = RVLLN4PI - 0.5*(log(detCnS)+en) - RVLLN2PI;

		if(Pn < 0.0)
			Pn = 0.0;
	}	

	pData->POrientMatch = Pn;

	// position probability

	if(!(Flags & RVL3DSURFACE_MATCH4_FLAG_OVERLAP))
	{
		// ZM <- mean surface orientation

		double Vect2Tmp[2], Vect2Tmp2[2], SM[2];

		fTmp = sy / detCnS;

		Vect2Tmp[0] = -CnS[1] * fTmp;
		Vect2Tmp[1] = CnS[0] * fTmp;

		RVLMULCOV2VECT(Cn_, Vect2Tmp, Vect2Tmp2)

		Vect2Tmp[0] = CnS[1] * fTmp;
		Vect2Tmp[1] = -CnS[0] * fTmp;

		RVLMULCOV2VECT(Cn, Vect2Tmp, SM)

		SM[0] += Vect2Tmp2[0];
		SM[1] += Vect2Tmp2[1];

		double ZM[3];

		ZM[0] = SM[0] * XP[0] + SM[1] * YP[0] + ZP[0];
		ZM[1] = SM[0] * XP[1] + SM[1] * YP[1] + ZP[1];
		ZM[2] = SM[0] * XP[2] + SM[1] * YP[2] + ZP[2];

		RVLNORM3(ZM, fTmp)

		RVLDIF3VECTORS(tF_, tF, Vect3Tmp)

		er = RVLDOTPRODUCT3(Vect3Tmp, ZM);

		ep = er * er / varqS;

		if(Flags & RVL3DSURFACE_MATCH4_FLAG_POSITION)
			if(ep > 6.635)
				return false;
	}

	double Pp = pData->PPriorPosition - 0.5*(log(varqS)+ep+RVLLN2PI);

	if(Pp < 0.0)
		Pp = 0.0;

	// total probability

	MatchQuality = Pn + Pp;

	return true;	
}

void CRVL3DSurface2::GetPoseContribution(void)
{
	RVLARRAY *pRelList = m_RelList + m_pClass->m_iRelListComponents;
	CRVL3DSurface2 *p3DConvexSegment = *((CRVL3DSurface2 **)pRelList->pFirst);

	double sigmaR = p3DConvexSegment->m_sigmaR;
	double *r = p3DConvexSegment->m_EigenValues;

	double d1 = sigmaR / (r[0] * r[0] + sigmaR);
	double d2 = sigmaR / (r[1] * r[1] + sigmaR);
	double d3 = sigmaR;

	double *R = p3DConvexSegment->m_Pose.m_Rot;
	double *t = p3DConvexSegment->m_Pose.m_X;

	double a1 = RVLMULVECTORCOL3(t, R, 1);
	double a2 = RVLMULVECTORCOL3(t, R, 2);

	//a1 = a2 = 0.0;

	double Z[3 * 3];

	//Z[3*0+0] = (a1*a1/d3 + 1/d1);
	//Z[3*0+1] = a1*a2/d3;
	//Z[3*0+2] = -a1/d3;
	//Z[3*1+1] = (a2*a2/d3 + 1/d2);
	//Z[3*1+2] = -a2/d3;
	//Z[3*2+2] = 1/d3;
	//RVLCOMPLETESIMMX3(Z)
	RVLDIAGMX3(1/d1, 1/d2, 1/d3, Z);

	double B[3 * 3];

	B[3*0+0] = R[3*1+1];
	B[3*0+1] = -R[3*1+0];
	B[3*0+2] = 0.0;
	B[3*1+0] = R[3*0+1];
	B[3*1+1] = -R[3*0+0];
	B[3*1+2] = 0.0;
	B[3*2+0] = R[3*2+1];
	B[3*2+1] = -R[3*2+0];
	B[3*2+2] = 0.0;

	double Tmp[3 * 3];

	RVLCOV3DTRANSF(Z, B, m_PoseContribution, Tmp)

	double *D = m_PoseContribution + 3 * 3;

	D[3*0+0] = R[3*0+2] * R[3*0+2] / d3;
	D[3*0+1] = R[3*0+2] * R[3*1+2] / d3;
	D[3*0+2] = R[3*0+2] * R[3*2+2] / d3;
	D[3*1+1] = R[3*1+2] * R[3*1+2] / d3;
	D[3*1+2] = R[3*1+2] * R[3*2+2] / d3;
	D[3*2+2] = R[3*2+2] * R[3*2+2] / d3;
	RVLCOMPLETESIMMX3(D)
}


///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 


void RVL3DSurfacesRotate(	CRVLMPtrChain *p3DSurfaceList,
							double *Rot)
{
	CRVL3DSurface2 *p3DSurface;

	p3DSurfaceList->Start();

	while(p3DSurfaceList->m_pNext)
	{
		p3DSurface = (CRVL3DSurface2 *)(p3DSurfaceList->GetNext());

		p3DSurface->Rotate(Rot);
	}
}

void RVLProject2DLineTo3DPlane(double *H,										// Input: homography matrix
							   double csIn, double snIn, double rhoIn,			// Input: parameters of 2D line
							   double &csOut, double &snOut, double &rhoOut)	// Output: parameters of 2D line 
																				//         represented in plane c.s.
{
	double lSrc[3], lTgt[3];
	
	lSrc[0] = csIn;
	lSrc[1] = snIn;
	lSrc[2] = rhoIn;

	LinearTransform3DT(H, lSrc, lTgt);

	double fTmp = sqrt(lTgt[0] * lTgt[0] + lTgt[1] * lTgt[1]);

	csOut = lTgt[0] / fTmp;
	snOut = lTgt[1] / fTmp;
	rhoOut = lTgt[2] / fTmp;
}


void RVL3DSurfaceInvTransf(double *NSrc,
						   double dSrc,
						   CRVL3DPose *pPose,
						   double *NTgt,
						   double &dTgt)
{
	LinearTransform3DT(pPose->m_Rot, NSrc, NTgt);

	dTgt = dSrc - RVLDotProduct(NSrc, pPose->m_X);	
}



void RVLProject2DPointTo3DPlane(int *i2DPoint, 
								double *d3DPoint,
								CRVL3DSurface2 *p3DPlane,
								CRVLCamera *pCamera)
{
	double V[3];

	RVLiU2U(i2DPoint, pCamera->CenterXNrm, pCamera->CenterYNrm, V);

	double f = pCamera->fNrm;

	V[2] = f;

	double z = f * p3DPlane->m_d / RVLDotProduct(p3DPlane->m_N, V);

	d3DPoint[0] = z * V[0] / f;
	d3DPoint[1] = z * V[1] / f;
	d3DPoint[2] = z;
}

void RVLProject3DPointTo3DPlane(
	double *XSrc,
	CRVL3DSurface2 *p3DPlane,
	double *XTgt)
{
	double s = p3DPlane->m_d / RVLDOTPRODUCT3(p3DPlane->m_N, XSrc);
	RVLSCALE3VECTOR(XSrc, s, XTgt);
}

void RVLProject2DPointTo3DPlane(int *i2DPoint, 
								double *d3DPoint,
								CRVL3DSurface2 *p3DPlane,
								double Uc, double Vc,
								double Fu, double Fv)
{
	double a, b, c, z;

	a = (i2DPoint[0]*1.0 - Uc)/Fu;
	b = (i2DPoint[1]*1.0 - Vc)/Fv;

	c = p3DPlane->m_N[0]*a + p3DPlane->m_N[1]*b + p3DPlane->m_N[2];

	z = p3DPlane->m_d/c;

	d3DPoint[0] = z * a;
	d3DPoint[1] = z * b;
	d3DPoint[2] = z;
}



BOOL RVL3DPlanarSurfaceEKFUpdate(	CRVL3DSurface2 *pSSurf,
									CRVL3DSurface2 *pMSurf,
									CRVL3DPose *pInitPose,
									CRVL3DPose *pFinalPose,
									RVLSURFACE_MATCH_ARRAY *pMatchData,
									bool bCheckConsistency)
{
	double MatchQuality;
	double detQ;

	if(!pSSurf->Match2(pMSurf, pInitPose, MatchQuality, detQ, pMatchData))
		if(bCheckConsistency)
			return FALSE;

	double *e = pMatchData->m_e;
	double *C = pMatchData->m_C;
	double *Q = pMatchData->m_Q;

	pFinalPose->PlanarSurfaceEKFUpdate2(C, Q, e, pInitPose);

	double *R = pFinalPose->m_Rot;
	double *t = pFinalPose->m_X;
	double *invt = (double *)(pFinalPose->m_pData);
	RVLMULMX3X3TVECT(R, t, invt);

	//if(e[0] * e[0] + e[1] * e[1] > 0.04)
	{
		memcpy(pFinalPose->m_C, pInitPose->m_C, 3 * 3 * 3 * sizeof(double));

		if(!pSSurf->Match2(pMSurf, pFinalPose, MatchQuality, detQ, pMatchData))
			if(bCheckConsistency)
				return FALSE;

		pFinalPose->PlanarSurfaceEKFUpdate2(C, Q, e);

		RVLMULMX3X3TVECT(R, t, invt);
	}	

	return TRUE;
}



void CRVL3DSurface2::Transf(
	CRVL3DPose * pPose,
	int iView)
{
	double *RM_M = pPose->m_Rot;
	double *tM_M = pPose->m_X;

	double *RFM = m_Pose.m_Rot;
	double *tFM = m_Pose.m_X;
	//CM = p3DSurface->m_Cp;

	double NM_[3];
	double RFM_[9];
	double tFM_[3];

	RVLCOPYMX3X3(RFM, RFM_);
	RVLCOPY3VECTOR(tFM, tFM_);
	RVLCOPY3VECTOR(m_N, NM_);
	//RVLCOPYMX3X3(CM, CM_)

	RVLMXMUL3X3(RM_M, RFM_, RFM);
	RVLMULMX3X3VECT(RM_M, tFM_, tFM);
	RVLMULMX3X3VECT(RM_M, NM_, m_N);
	//RVLCOV3DTRANSF(CM_, RM_M, CM, M3x3Tmp)

	if (m_Flags & RVL3DSURFACE_FLAG_SAMPLES)
	{
		double *XM, *VM;
		double XM_[3], VM_[3];

		RVL3DSURFACE_SAMPLE *pSample = (RVL3DSURFACE_SAMPLE *)(m_Samples.pFirst);

		while (pSample)
		{
			XM = pSample->X;
			VM = pSample->V;

			RVLCOPY3VECTOR(XM, XM_);
			RVLCOPY3VECTOR(VM, VM_);

			RVLMULMX3X3VECT(RM_M, XM_, XM);
			RVLMULMX3X3VECT(RM_M, VM_, VM);

			pSample = (RVL3DSURFACE_SAMPLE *)(pSample->pNext);
		}
	}

	if (m_Flags & RVL3DSURFACE_FLAG_BOUNDARY)
	{
		double *XM;
		double XM_[3];
		RVL3DPOINT3 *pPt;

		RVL3DCONTOUR *pContour = (RVL3DCONTOUR *)(m_BoundaryContourList.pFirst);

		while (pContour)
		{
			pContour->iView = iView;

			pPt = (RVL3DPOINT3 *)(pContour->PtList.pFirst);

			while (pPt)
			{
				XM = pPt->P3D;

				RVLCOPY3VECTOR(XM, XM_);

				RVLMULMX3X3VECT(RM_M, XM_, XM);

				pPt = (RVL3DPOINT3 *)(pPt->pNext);
			}

			pContour = (RVL3DCONTOUR *)(pContour->pNext);
		}
	}
}
