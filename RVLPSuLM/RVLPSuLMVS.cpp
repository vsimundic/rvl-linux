#include "highgui.h"
#include "RVLCore.h"
#include "RVLPCS.h"
#include "RVLRLM.h"
#include "RVLPSuLMBuilder.h"
#include "RVLPSuLMVS.h"

CRVLPSuLMVS::CRVLPSuLMVS(void)
{
}

CRVLPSuLMVS::~CRVLPSuLMVS(void)
{
}

void CRVLPSuLMVS::CreateParamList()
{
	CRVLPCSVS::CreateParamList();

	RVLPARAM_DATA *pParamData;

	pParamData = m_ParamList.AddParam("VS.PoseLA.alpha[deg]", RVLPARAM_TYPE_DOUBLE, &(m_PoseLA.m_Alpha));
	pParamData = m_ParamList.AddParam("VS.PoseLA.beta[deg]", RVLPARAM_TYPE_DOUBLE, &(m_PoseLA.m_Beta));
	pParamData = m_ParamList.AddParam("VS.PoseLA.theta[deg]", RVLPARAM_TYPE_DOUBLE, &(m_PoseLA.m_Theta));
	pParamData = m_ParamList.AddParam("VS.PoseLA.x[mm]", RVLPARAM_TYPE_DOUBLE, m_PoseLA.m_X);
	pParamData = m_ParamList.AddParam("VS.PoseLA.y[mm]", RVLPARAM_TYPE_DOUBLE, m_PoseLA.m_X + 1);
	pParamData = m_ParamList.AddParam("VS.PoseLA.z[mm]", RVLPARAM_TYPE_DOUBLE, m_PoseLA.m_X + 2);
}

void CRVLPSuLMVS::Init(char * CfgFile2Name)
{
	CRVLPCSVS::Init(CfgFile2Name);

	m_PoseLA.m_Alpha *= DEG2RAD;
	m_PoseLA.m_Beta *= DEG2RAD;
	m_PoseLA.m_Theta *= DEG2RAD;

	// initialize PSuLMBuilder

	m_PSuLMBuilder.m_pMem0 = &m_Mem0;
	m_PSuLMBuilder.m_pMem = &m_Mem;
	m_PSuLMBuilder.m_pMem2 = &m_Mem2;
	//m_PSuLMBuilder.m_pPoseSA = &m_PoseLA;
	
	m_PSuLMBuilder.m_pPoseCB = &m_PoseLA;

	m_PSuLMBuilder.m_pCamera = &m_CameraL;
	m_PSuLMBuilder.m_pStereoVision = &m_StereoVision;
	m_PSuLMBuilder.m_pTimer = m_pTimer;
	m_PSuLMBuilder.m_pAImage = &m_AImage;

	m_PSuLMBuilder.CreateParamList(&m_Mem0);

	if(CfgFile2Name)
	{
		m_PSuLMBuilder.m_ParamList.LoadParams(CfgFile2Name);


	}

	m_PSuLMBuilder.Init();

	m_PSuLMBuilder.RobotCameraPose();

	m_PSuLMBuilder.m_pPSD = &m_PSD;

	//m_PSuLMBuilder.m_pSegmentation = (CRVLSegmentationEB *)m_pSegmentation;

	//if((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
	//	m_PSuLMBuilder.Load(m_PSuLMBuilder.m_ModelDatabasePath,2000);
	if((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
		m_PSuLMBuilder.LoadMap();
	else if((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
		m_PSuLMBuilder.m_HypothesisArray = 
			new RVLPSULM_HYPOTHESIS *[m_PSuLMBuilder.m_maxnHypothesesPerModel];

	//FILE *fp;
	//
	//fopen_s(&fp, "C:\\RVL\\ExpRez\\PSuLM.dat", "rb");

	//if(fp)
	//{
	//	m_PSuLMBuilder.Load(fp);

	//	fclose(fp);
	//}
	m_pPSuLM = NULL;

}

void CRVLPSuLMVS::Update()
{
	//m_Mem.Clear();

	m_iMCMem = (m_iMCMem + 1) % RVLSYS_MCMEMSIZE;

	m_MCMem[m_iMCMem].Clear();

	PSuLMBasedRLMUpdate(RVLPSULMBUILDER_CREATEMODEL_FROM_IMAGE);
}

void CRVLPSuLMVS::PSuLMBasedRLMUpdate(DWORD Flags)
{
	double StartTime, ExecutionTime;

	if(m_pTimer)
		StartTime = m_pTimer->GetTime();

	m_pPrevPSuLM = m_pPSuLM;

	m_PSuLMBuilder.m_pMCMem = m_MCMem + m_iMCMem;

	m_PSuLMBuilder.m_ImageFileName = m_ImageFileName; 

	m_pPSuLM = m_PSuLMBuilder.Create(Flags);

#ifdef NEVER		// switch on if you want to consider color and texture features
	////TEXTON TESTING GROUND
	//int noObjo = m_PSuLMBuilder.m_maxnDominant3DSurfaces <= m_pPSuLM->m_SurfaceList.m_nElements ? m_PSuLMBuilder.m_maxnDominant3DSurfaces : m_pPSuLM->m_SurfaceList.m_nElements;
	//float *rat = new float[noObjo];
	//memset(rat, 0, noObjo * sizeof(float));
	//for (int i = 0; i < noObjo; i++)
	//{
	//	//m_pPSuLM->m_3DSurfaceArray[i]->CalculateTextonHistogram(100.0, 2.0, 100.0);
	//	rat[i] = m_pPSuLM->m_3DSurfaceArray[i]->CalculateTextonHistogram2(100.0, 10.0, 100.0, 10.0, 10, 10);
	//}
	//FILE *datRGB = fopen("C:\\Users\\Damir\\Documents\\ExpRez\\texton.txt", "w");
	//FILE *datRAT = fopen("C:\\Users\\Damir\\Documents\\ExpRez\\textonRatio.txt", "w");
	//CRVL3DMeshObject *object;
	//RVLQLIST_HIST_ENTRY* pHistEntry;
	//int noBinss = 10 * (10 - floor(1.0 / (8.0 / 10.0)));
	//float *rgbHist = new float[noBinss];
	//for (int j = 0; j < noObjo; j++)
	//{
	//	object = m_pPSuLM->m_3DSurfaceArray[j];
	//	//RGB Histogram
	//	memset(rgbHist, 0, noBinss * sizeof(float));
	//	if (object->m_TextonHist)
	//	{
	//		pHistEntry = (RVLQLIST_HIST_ENTRY*)object->m_TextonHist->pFirst;
	//		while(pHistEntry)
	//		{
	//			rgbHist[pHistEntry->adr] = pHistEntry->value;
	//			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	//		}
	//	}
	//	for (int i = 0; i < noBinss; i++)
	//		fprintf(datRGB, "%.3f ", rgbHist[i]);
	//	fprintf(datRGB, "\n");
	//	fprintf(datRAT, "%.3f\n", rat[j]);
	//}
	//fclose(datRGB);
	//fclose(datRAT);

	//If material descriptor use selected generate descriptors for each usable object
	if(m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MATERIAL)
	{
		// filtering image and changing color spaces if need be
		IplImage *pImageNHS = CRVLImageFilter::RVLFilterNHS(m_PSuLMBuilder.m_pCamera->m_pRGBImage);
		IplImage *pImageC = cvCreateImage(cvSize( pImageNHS->width, pImageNHS->height ), IPL_DEPTH_8U, 3);
		IplImage *pImageBi = cvCreateImage(cvSize( pImageNHS->width, pImageNHS->height ), IPL_DEPTH_8U, 3);

//#ifdef RVLSAS_MATERIAL_COLOR_HSV
		cvSmooth(pImageNHS, pImageBi, CV_BILATERAL, 5, 0, 25, 25);

		/*IplImage *pImageGray = cvCreateImage(cvSize(m_PSuLMBuilder.m_pCamera->m_pRGBImage->width, m_PSuLMBuilder.m_pCamera->m_pRGBImage->height ), IPL_DEPTH_8U, 1);
		cvCvtColor(m_PSuLMBuilder.m_pCamera->m_pRGBImage, pImageGray, CV_BGR2GRAY);
		IplImage *pImageGrayEQ;
		pImageGrayEQ = CRVLImageFilter::RVLFilterHEQGray(pImageGray);*/
		//cvSmooth(m_PSuLMBuilder.m_pCamera->m_pRGBImage, pImageBi, CV_BILATERAL, 5, 0, 25, 25);
		//cvSaveImage("C:\\Users\\Damir\\Documents\\test.bmp", pImageBi);
		//cvNamedWindow ("Bilateral", CV_WINDOW_AUTOSIZE);
		//cvShowImage("Bilateral", pImageBi);

		cvCvtColor(pImageBi, pImageC, CV_BGR2HSV);
		pImageC->channelSeq[0] = 'H';
		pImageC->channelSeq[1] = 'S';
		pImageC->channelSeq[2] = 'V';
//#else
		//cvSmooth(pImageNHS, pImageC, CV_BILATERAL, 5, 0, 25, 25);			
		//cvSaveImage("c:\\RVL\\ExpRez\\img.bmp", pImageC);
//#endif
		
		cvReleaseImage(&pImageNHS); //Releasing image
		
		//cvReleaseImage(&pImageBi); //Releasing image
		//Setting useful pixel map
		//BYTE *uPixMask;// = new BYTE[AImage.m_Width * AImage.m_Height];
		//RVLMEM_ALLOC_STRUCT_ARRAY(m_PSuLMBuilder.m_pMem, BYTE, m_PSuLMBuilder.m_pAImage->m_Width * m_PSuLMBuilder.m_pAImage->m_Height, uPixMask);
		//memset(uPixMask, 0, m_PSuLMBuilder.m_pAImage->m_Width * m_PSuLMBuilder.m_pAImage->m_Height * sizeof(BYTE));
		//RVLSetUsefulPixMask(&(m_PSuLMBuilder.m_pAImage->m_C2DRegion), m_PSuLMBuilder.m_pPSD->m_Point3DMap, m_PSuLMBuilder.m_pAImage->m_Width,  m_PSuLMBuilder.m_pAImage->m_Height, m_PSuLMBuilder.m_pMem, 3 + 4, uPixMask, 4);
		RVLSetUsefulPixMask(&(m_PSuLMBuilder.m_pAImage->m_C2DRegion), m_PSuLMBuilder.m_pPSD->m_Point3DMap, m_PSuLMBuilder.m_pAImage->m_Width,  m_PSuLMBuilder.m_pAImage->m_Height, m_PSuLMBuilder.m_pMem, 3);	//NORMALNO KORISTENA
		//RVLSetUsefulPixMask(&(m_PSuLMBuilder.m_pAImage->m_C2DRegion), m_PSuLMBuilder.m_pPSD->m_Point3DMap, m_PSuLMBuilder.m_pAImage->m_Width,  m_PSuLMBuilder.m_pAImage->m_Height, m_PSuLMBuilder.m_pMem, 1, uPixMask, 255);
		//generating descriptors
		int noObj = m_PSuLMBuilder.m_maxnDominant3DSurfaces <= m_pPSuLM->m_SurfaceList.m_nElements ? m_PSuLMBuilder.m_maxnDominant3DSurfaces : m_pPSuLM->m_SurfaceList.m_nElements;
		float histBase[] = {8.0, 8.0, 0.0};	//bins per dimension of color histogram
		for (int i = 0; i < noObj; i++)
		{
			//m_pPSuLM->m_3DSurfaceArray[i]->RVLCalculateRGCHist(pImageBi, histBase, false);
			m_pPSuLM->m_3DSurfaceArray[i]->RVLCalculateColorHist(pImageC, histBase, false);
			//m_pPSuLM->m_3DSurfaceArray[i]->RVLCalculateRGBOppHist(pImageBi, histBase, true);
			//m_pPSuLM->m_3DSurfaceArray[i]->RVLCalculateRGBNTHist(pImageBi, histBase, false);
			//m_pPSuLM->m_3DSurfaceArray[i]->CalculateLbpRiu(24,3, pImageGrayEQ);
			//m_pPSuLM->m_3DSurfaceArray[i]->CalculateLbpRiuVar(24, 3, 200, pImageGrayEQ);
			//m_pPSuLM->m_3DSurfaceArray[i]->CalculateLbp(16, 2, pImageGrayEQ);
		}
		cvReleaseImage(&pImageC);//Releasing image
		//m_pPSuLM->m_pRootMeshObject->SaveRGBDSegmentMasks("C:\\Users\\Damir\\Documents\\rgbMask.txt", "C:\\Users\\Damir\\Documents\\depthMask.txt", m_PSD.m_Point3DMap);
		cvReleaseImage(&pImageBi); //Releasing image
		//cvReleaseImage(&pImageGray);//Releasing image
		//cvReleaseImage(&pImageGrayEQ);//Releasing image
		/*FILE *objf, *mtlf;
		objf = fopen("C:\\Users\\Damir\\Documents\\test.obj", "w");
		mtlf = fopen("C:\\Users\\Damir\\Documents\\test.obj.mtl", "w");
		m_pPSuLM->m_pRootMeshObject->SaveMeshObject2OBJ(objf, mtlf, "test.obj.mtl", m_PSD.m_Point3DMap, 2, "sl-00000-LW.bmp");
		fclose(objf);
		fclose(mtlf);*/
		//m_PSuLMBuilder.AddNodeToColorMapDB(m_PSuLMBuilder.m_colorMapDB, m_pPSuLM, m_PSuLMBuilder.m_pMem0);

		////Snimanje color deskriptora
		//FILE *datRGB = fopen("C:\\Users\\Damir\\Documents\\Faks\\Poslijediplomski\\Doktorat\\Doktorat\\image workshop\\Color descriptor examples\\Opp_soft8.txt", "w");
		//CRVL3DMeshObject *object;
		//RVLQLIST_PTR_ENTRY *pEl;
		//RVLQLIST_HIST_ENTRY* pHistEntry;
		//int noBins = 8*8;
		//float *rgbHist = new float[noBins];
		//pEl = (RVLQLIST_PTR_ENTRY*)m_pPSuLM->m_pRootMeshObject->m_ChildMeshObjects->pFirst;
		//while(pEl)
		//{
		//	object = (CRVL3DMeshObject*)pEl->Ptr;
		//	//RGB Histogram
		//	memset(rgbHist, 0, noBins * sizeof(float));
		//	pHistEntry = (RVLQLIST_HIST_ENTRY*)object->m_histRGB->pFirst;
		//	while(pHistEntry)
		//	{
		//		rgbHist[pHistEntry->adr] = pHistEntry->value;
		//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		//	}
		//	for (int i = 0; i < noBins; i++)
		//		fprintf(datRGB, "%.3f ", rgbHist[i]);
		//	fprintf(datRGB, "\n");
		//	pEl = (RVLQLIST_PTR_ENTRY*)pEl->pNext;
		//}
		//fclose(datRGB);
		//delete [] rgbHist;

		////////snimanje texture deskriptora
		//FILE *datLBP = fopen("C:\\Users\\Damir\\Documents\\Faks\\Poslijediplomski\\Doktorat\\Doktorat\\Workshop & experiments results\\00002lbp24.txt", "w");
		//CRVL3DMeshObject *object;
		//RVLQLIST_PTR_ENTRY *pEl;
		//RVLQLIST_HIST_ENTRY* pHistEntry;
		//int noBins = 288;
		//float *lbpHist = new float[noBins];
		//pEl = (RVLQLIST_PTR_ENTRY*)m_pPSuLM->m_pRootMeshObject->m_ChildMeshObjects->pFirst;
		//while(pEl)
		//{
		//	object = (CRVL3DMeshObject*)pEl->Ptr;
		//	//RGB Histogram
		//	memset(lbpHist, 0, noBins * sizeof(float));
		//	pHistEntry = (RVLQLIST_HIST_ENTRY*)object->m_LBP->pFirst;
		//	while(pHistEntry)
		//	{
		//		lbpHist[pHistEntry->adr] = pHistEntry->value;
		//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		//	}
		//	for (int i = 0; i < noBins; i++)
		//		fprintf(datLBP, "%.3f ", lbpHist[i]);
		//	fprintf(datLBP, "\n");
		//	pEl = (RVLQLIST_PTR_ENTRY*)pEl->pNext;
		//}
		//fclose(datLBP);
		//delete [] lbpHist;

		//datLBP = fopen("C:\\Users\\Damir\\Documents\\Faks\\Poslijediplomski\\Doktorat\\Doktorat\\Workshop & experiments results\\00002lbp_riu24.txt", "w");
		////CRVL3DMeshObject *object;
		////RVLQLIST_PTR_ENTRY *pEl;
		//RVLQLIST_HIST_ENTRY* pHistEntryS;
		//noBins = 24 + 2;
		//lbpHist = new float[noBins];
		//pEl = (RVLQLIST_PTR_ENTRY*)m_pPSuLM->m_pRootMeshObject->m_ChildMeshObjects->pFirst;
		//while(pEl)
		//{
		//	object = (CRVL3DMeshObject*)pEl->Ptr;
		//	//RGB Histogram
		//	memset(lbpHist, 0, noBins * sizeof(float));
		//	pHistEntryS = (RVLQLIST_HIST_ENTRY*)object->m_LBP_RIU->pFirst;
		//	while(pHistEntryS)
		//	{
		//		lbpHist[pHistEntryS->adr] = pHistEntryS->value;
		//		pHistEntryS = (RVLQLIST_HIST_ENTRY*)pHistEntryS->pNext;
		//	}
		//	for (int i = 0; i < noBins; i++)
		//		fprintf(datLBP, "%.3f ", lbpHist[i]);
		//	fprintf(datLBP, "\n");
		//	pEl = (RVLQLIST_PTR_ENTRY*)pEl->pNext;
		//}
		//fclose(datLBP);
		//delete [] lbpHist;

		//datLBP = fopen("C:\\Users\\Damir\\Documents\\Faks\\Poslijediplomski\\Doktorat\\Doktorat\\Workshop & experiments results\\00002lbp_riu_var24.txt", "w");
		////CRVL3DMeshObject *object;
		////RVLQLIST_PTR_ENTRY *pEl;
		////RVLQLIST_HIST_ENTRY* pHistEntry;
		//noBins = (16 + 1)*(24 + 2);
		//lbpHist = new float[noBins];
		//pEl = (RVLQLIST_PTR_ENTRY*)m_pPSuLM->m_pRootMeshObject->m_ChildMeshObjects->pFirst;
		//while(pEl)
		//{
		//	object = (CRVL3DMeshObject*)pEl->Ptr;
		//	//RGB Histogram
		//	memset(lbpHist, 0, noBins * sizeof(float));
		//	pHistEntry = (RVLQLIST_HIST_ENTRY*)object->m_LBP_RIU_VAR->pFirst;
		//	while(pHistEntry)
		//	{
		//		lbpHist[pHistEntry->adr] = pHistEntry->value;
		//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		//	}
		//	for (int i = 0; i < noBins; i++)
		//		fprintf(datLBP, "%.3f ", lbpHist[i]);
		//	fprintf(datLBP, "\n");
		//	pEl = (RVLQLIST_PTR_ENTRY*)pEl->pNext;
		//}
		//fclose(datLBP);
		//delete [] lbpHist;


	}	//If material descriptor 
#endif

	//int ImageFileNameLen = strlen(m_ImageFileName);

	//RVLMEM_ALLOC_STRUCT_ARRAY(m_PSuLMBuilder.m_pMCMem, char, ImageFileNameLen + 7, m_pPSuLM->m_FileName);

	//strcpy(m_pPSuLM->m_FileName, m_ImageFileName);

	//char extension[] = "LW.bmp";

	//strcpy(m_pPSuLM->m_FileName + ImageFileNameLen, extension);

	if(m_pTimer)
	{
		ExecutionTime = m_pTimer->GetTime() - StartTime;

#ifdef RVLSYS_PSULMBRLM_UPDATE_LOG_FILE
		fprintf(fpLog, "PSuLM Creation Exec. Time=%lf s\n", ExecutionTime);
#endif
		//FULL PSuLM build time FILKO
		m_PSuLMBuilder.m_CreateTime = ExecutionTime;
	}

	// only for debugging purpose !!!
//#ifdef RVLWIN
//	StartTime = m_pTimer->GetTime();
//#endif
//
//	m_PSD.Sample2DRegions();
//
//#ifdef RVLWIN
//	ExecutionTime = m_pTimer->GetTime() - StartTime;
//#endif
		/////


	//if((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_MAP_BUILDING)
	//{
	//	char *PSuLMFileName = new char[strlen(m_ImageFileName) + 6];

	//	strcpy(PSuLMFileName, m_ImageFileName);

	//	strcat(PSuLMFileName, "M.dat");

	//	FILE *fp;
	//	
	//	fopen_s(&fp, PSuLMFileName, "wb");

	//	if(fp)
	//		m_pPSuLM->Save(fp);	// this should be uncommented after repairing Save function

	//	fclose(fp);
	//	
	//	delete[] PSuLMFileName;
	//}
	//else
	//{

	if(m_pTimer)
		StartTime = m_pTimer->GetTime();

	if(m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_GLOBAL)
	{
		m_PSuLMBuilder.m_pNearestModelPSuLM = NULL;
		m_PSuLMBuilder.m_Flags |= RVLPSULMBUILDER_FLAG_KIDNAPPED;
	}

	if((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
		if(m_pPrevPSuLM)
			m_PSuLMBuilder.Localization(m_pPSuLM, &m_PoseA0, m_pPrevPSuLM);

	if((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
		m_PSuLMBuilder.Localization(m_pPSuLM, &m_PoseA0);

	////FILKO
	////Creating match matrix via color and saving both match matrices to file(EXP 1.)
	////RGB MATCH
	//if (m_PSuLMBuilder.m_pNearestModelPSuLM)
	//{
	//	FILE *datMatchMatrix;
	//	datMatchMatrix = fopen("C:\\Users\\Damir\\Documents\\Faks\\Poslijediplomski\\Doktorat\\Doktorat\\Workshop & experiments results\\Threshold stats MEGA\\locMatchMatrix.txt", "a");
	//	RVLQLIST *matchList = new RVLQLIST;
	//	RVLQLIST_INIT(matchList);
	//	float *rgbMatchMatrix = new float[m_pPSuLM->m_n3DSurfaces * m_PSuLMBuilder.m_pNearestModelPSuLM->m_n3DSurfaces];
	//	memset(rgbMatchMatrix, 0, m_pPSuLM->m_n3DSurfaces * m_PSuLMBuilder.m_pNearestModelPSuLM->m_n3DSurfaces * sizeof(float));
	//	/*float *rgbMatchMatrix = new float[20 * 20];
	//	memset(rgbMatchMatrix, 0, 20 * 20 * sizeof(float));*/
	//	m_PSuLMBuilder.GenMatchListViaDescriptors(m_pPSuLM, m_PSuLMBuilder.m_pNearestModelPSuLM, matchList, -2.0, 20);
	//	RVLPSULM_MSMATCH_DATA *pMatchData;
	//	RVLQLIST_PTR_ENTRY *pListEntry = (RVLQLIST_PTR_ENTRY *)matchList->pFirst;
	//	while (pListEntry)
	//	{
	//		pMatchData = (RVLPSULM_MSMATCH_DATA *)pListEntry->Ptr;
	//		rgbMatchMatrix[pMatchData->pSData->p3DSurface->m_Index * m_PSuLMBuilder.m_pNearestModelPSuLM->m_n3DSurfaces + pMatchData->pMData->p3DSurface->m_Index] = pMatchData->Probability;
	//		pListEntry = (RVLQLIST_PTR_ENTRY *)pListEntry->pNext;
	//	}
	//	//Zapis
	//	//fprintf(datMatchMatrix, "Model:%d\n", m_PSuLMBuilder.m_pNearestModelPSuLM->m_Index);
	//	for (int i = 0; i < m_pPSuLM->m_n3DSurfaces * m_PSuLMBuilder.m_pNearestModelPSuLM->m_n3DSurfaces; i++)
	//		fprintf(datMatchMatrix, "%d\t", m_PSuLMBuilder.m_MatchMatrix[i]);
	//	fprintf(datMatchMatrix, "\n");
	//	for (int i = 0; i < m_pPSuLM->m_n3DSurfaces * m_PSuLMBuilder.m_pNearestModelPSuLM->m_n3DSurfaces; i++)
	//		fprintf(datMatchMatrix, "%.3f\t", rgbMatchMatrix[i]);
	//	fprintf(datMatchMatrix, "\n");
	//	fclose(datMatchMatrix);
	//	delete [] rgbMatchMatrix;
	//	m_PSuLMBuilder.m_pMem2->Clear();
	//}

	////FILKO EXP 2.(matching every segment of current scene with all segments from the database)
	//FILE *datMatchMatrix;
	//datMatchMatrix = fopen("C:\\Users\\Damir\\Documents\\Faks\\Poslijediplomski\\Doktorat\\Doktorat\\Workshop & experiments results\\Threshold stats MEGA\\DBMatchMatrix.txt", "a");
	//RVLQLIST *matchList = new RVLQLIST;
	//float *rgbMatchMatrix2 = new float[20 * 20];
	////for each pSuLM in DB
	//CRVLPSuLM *mapPSuLM;
	//RVLPTRCHAIN_ELEMENT *curr;
	//RVLPSULM_MSMATCH_DATA *pMatchData;
	//RVLQLIST_PTR_ENTRY *pListEntry;
	//curr = m_PSuLMBuilder.m_PSuLMList.m_pFirst;
	//while(curr)
	//{
	//	mapPSuLM = (CRVLPSuLM*)curr->pData;
	//	//float *rgbMatchMatrix2 = new float[m_pPSuLM->m_n3DSurfaces * mapPSuLM->m_n3DSurfaces];
	//	memset(rgbMatchMatrix2, 0, 20 * 20 * sizeof(float));
	//	RVLQLIST_INIT(matchList);
	//	m_PSuLMBuilder.GenMatchListViaDescriptors(m_pPSuLM, mapPSuLM, matchList, -2.0, 20);
	//	pListEntry = (RVLQLIST_PTR_ENTRY *)matchList->pFirst;
	//	while (pListEntry)
	//	{
	//		pMatchData = (RVLPSULM_MSMATCH_DATA *)pListEntry->Ptr;
	//		rgbMatchMatrix2[pMatchData->pSData->p3DSurface->m_Index *m_pPSuLM->m_n3DSurfaces + pMatchData->pMData->p3DSurface->m_Index] = pMatchData->Probability;
	//		pListEntry = (RVLQLIST_PTR_ENTRY *)pListEntry->pNext;
	//	}
	//	//Zapis 
	//	for (int i = 0; i < m_pPSuLM->m_n3DSurfaces * mapPSuLM->m_n3DSurfaces; i++)
	//		fprintf(datMatchMatrix, "%.2f\t", rgbMatchMatrix2[i]);
	//	fprintf(datMatchMatrix, "\n");				

	//	curr = curr->pNext;
	//	m_PSuLMBuilder.m_pMem2->Clear();
	//}
	//
	//fclose(datMatchMatrix);
	//delete [] rgbMatchMatrix2;


	if(m_pTimer)
	{
		ExecutionTime = m_pTimer->GetTime() - StartTime;

#ifdef RVLSYS_PSULMBRLM_UPDATE_LOG_FILE
		fprintf(fpLog, "Localization Time=%lf s\n", ExecutionTime);
#endif
	}
}

void RVLPSuLMDisplayMouseCallback2(int event, int x, int y, int flags, void* vpData)
{
	CRVL3DPose NullPose;

	RVLNULL3VECTOR(NullPose.m_X);
	RVLUNITMX3(NullPose.m_Rot);

	RVLPSULMDISPLAY_MOUSE_CALLBACK_DATA *pData = (RVLPSULMDISPLAY_MOUSE_CALLBACK_DATA *)vpData;

	CRVL3DSurface2 *pSelectedSurf = NULL;
	CRVL3DLine2 *pSelectedLine = NULL;

	CRVLGUI *pGUI = pData->pGUI;
	CRVLFigure *pFig = pData->pFig;
	CRVLFigure *pFig2 = pData->pFig2;
	CRVLPSuLM *pPSuLM, *pPSuLM2;

	CRVLFigure *pSFig, *pMFig;

	CRVLPSuLMVS *pVS = pData->pVS;

	RVLPSULM_HYPOTHESIS *pHypothesis = pVS->m_PSuLMBuilder.m_HypothesisArray[pData->iHypothesis];

	int w = pData->w;
	
	int iPix;
	int a, b;

	switch( event )
	{
		case CV_EVENT_LBUTTONDOWN:
			//if(!pData->bSelection)
			{
				pData->u = x;

				pData->v = y;

				pData->bSelection = true;

				iPix = x / pData->ZoomFactor + y / pData->ZoomFactor * w;

				if(pFig->m_Flags & RVLPSULM_DISPLAY_SCENE)
				{
					pSFig = pFig;
					pMFig = pFig2;
					pPSuLM = pVS->m_pPSuLM;
					pPSuLM2 = pHypothesis->pMPSuLM;
					a = pPSuLM2->m_n3DSurfacesTotal;
					b = 1;
				}
				else if(pFig->m_Flags & RVLPSULM_DISPLAY_MODEL)
				{
					pSFig = pFig2;
					pMFig = pFig;
					pPSuLM2 = pVS->m_pPSuLM;
					pPSuLM = pHypothesis->pMPSuLM;
					a = 1;
					b = pPSuLM->m_n3DSurfacesTotal;
				}

				pPSuLM->Project(&(pFig->m_PoseC0), FALSE, iPix, &pSelectedSurf, &pSelectedLine);

				pFig->Clear();

				pFig2->Clear();

				pVS->m_PSuLMBuilder.DisplayHypothesis(pGUI, pSFig, pMFig, pVS->m_pPSuLM, pData->mDisplayPSuLMFlags, 
					pData->pImage);

				if(pSelectedSurf)
				{
					pPSuLM->Display3DSurface(pFig, pSelectedSurf, &NullPose, cvScalar(255, 255, 0), 2,
						RVLPSULM_DISPLAY_VECTORS);

					BOOL bCorrespondent;
					CRVL3DSurface2 *pSurf2;

					for(int iMatch = 0; iMatch < pPSuLM2->m_n3DSurfacesTotal; iMatch++)
					{
						bCorrespondent = FALSE;

						if((pVS->m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD) == 
							RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
						{
							if(pVS->m_PSuLMBuilder.m_MatchMatrix[a * pSelectedSurf->m_Index + b * iMatch] >= 
								pVS->m_PSuLMBuilder.m_minSurfaceSamplesForMatch)
								bCorrespondent = TRUE;
						}
						else
						{
#ifdef RVLPSULMBUILDER_DISPLAY_MATCH_OVERLAP
							if(pVS->m_PSuLMBuilder.m_MatchMatrix[a * pSelectedSurf->m_Index +  b * iMatch] == 2)
#else
							if(m_pPSuLMBuilder->m_MatchMatrix[pSelectedSurf->m_Index +  nMSurfaces * iMatch] > 0)
#endif
								bCorrespondent = TRUE;
						}

						if(bCorrespondent)
						{
							pSurf2 = pPSuLM2->m_3DSurfaceArray[iMatch];

							pPSuLM2->Display3DSurface(pFig2, pSurf2, &NullPose, cvScalar(255, 0, 0), 2,
								RVLPSULM_DISPLAY_VECTORS);
						}
					}
				}

				pGUI->DisplayVectors(pSFig, 0, 0, (double)(pData->ZoomFactor));

				pGUI->DisplayVectors(pMFig, 0, 0, 1.0);	

				pGUI->ShowFigure(pFig);

				pGUI->ShowFigure(pFig2);

				pVS->m_PSuLMBuilder.DisplayHypothesisData(pFig, 0, pSelectedSurf);
			}
	}
}