//#include "highgui.h"
#include <flann\flann.hpp>
#include "RVLCore.h"
#include "RVLPCS.h"
#include "RVLRLM.h"
#include "RVLPSuLMBuilder.h"
#include "RVLPSuLMGroundTruth.h"
#include "RVLPSuLMVS.h"

CRVLPSuLMVS::CRVLPSuLMVS(void)
{
	m_pMeshFile = NULL;
	m_MatchMatrix = NULL;
	m_MatchMatrixGT = NULL;
	//m_HypothesisArrayGT = NULL;
	m_nSamples = 0;
	m_ResPose.m_ParamFlags = 0x00000000;
}

CRVLPSuLMVS::~CRVLPSuLMVS(void)
{
	if(m_GroundTruth.m_nMatches > 0)
		m_GroundTruth.Save();

	ClearMeshFileData();

	if(m_MatchMatrix)
		delete[] m_MatchMatrix;

	if(m_MatchMatrixGT)
		delete[] m_MatchMatrixGT;

	//if(m_HypothesisArrayGT)
	//	delete[] m_HypothesisArrayGT;
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
	pParamData = m_ParamList.AddParam("VS.CreateGlobalMesh", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLSYS_FLAGS_CREATE_GLOBAL_MESH);
	pParamData = m_ParamList.AddParam("VS.EditMap", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLSYS_FLAGS_EDIT_MAP);
	pParamData = m_ParamList.AddParam("VS.Validation", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLSYS_FLAGS_VALIDATION);
	pParamData = m_ParamList.AddParam("VS.GroundTruthFileName", RVLPARAM_TYPE_STRING, &(m_GroundTruth.m_FileName));
	pParamData = m_ParamList.AddParam("VS.Record", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLSYS_FLAGS_RECORD);
	pParamData = m_ParamList.AddParam("VS.ReviewResults", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLSYS_FLAGS_REVIEW_RESULTS);
}

void CRVLPSuLMVS::Init(char * CfgFile2Name)
{
	CRVLPCSVS::Init(CfgFile2Name);

	m_PoseLA.m_Alpha *= DEG2RAD;
	m_PoseLA.m_Beta *= DEG2RAD;
	m_PoseLA.m_Theta *= DEG2RAD;

	double *R = m_PoseA0.m_Rot;
	double *t = m_PoseA0.m_X;
	RVLUNITMX3(R);
	RVLNULL3VECTOR(t);
	m_PoseA0.m_Alpha = m_PoseA0.m_Beta = m_PoseA0.m_Theta = 0.0;
	m_PoseA0.m_ParamFlags = 0x00000000;
	
	// initialize PSuLMBuilder

	m_PSuLMBuilder.m_pMem0 = &m_Mem0;
	m_PSuLMBuilder.m_pMem = &m_Mem;
	m_PSuLMBuilder.m_pMem2 = &m_Mem2;
	m_PSuLMBuilder.m_pMCMem = m_MCMem;
	//m_PSuLMBuilder.m_pPoseSA = &m_PoseLA;
	
	m_PSuLMBuilder.m_pPoseCB = &m_PoseLA;

	m_PSuLMBuilder.m_pCamera = &m_CameraL;
	m_PSuLMBuilder.m_pStereoVision = &m_StereoVision;
	m_PSuLMBuilder.m_pTimer = m_pTimer;
	m_PSuLMBuilder.m_pAImage = &m_AImage;
	
	m_PSuLMBuilder.m_PCScale = m_LidarParams.scale;

	m_PSuLMBuilder.CreateParamList(&m_Mem0);

	if(CfgFile2Name)
		m_PSuLMBuilder.m_ParamList.LoadParams(CfgFile2Name);

	m_PSuLMBuilder.Init();

	if(m_Flags & RVLSYS_FLAGS_PC)
	{
		m_PSuLMBuilder.m_Flags |= RVLPSULMBUILDER_FLAG_PC;

		m_PSD.GetOrgPCProjectionParams(m_CameraL.fNrm, m_CameraL.fvNrm, m_CameraL.CenterXNrm, m_CameraL.CenterYNrm);

		m_CameraL.fvNrm = m_CameraL.fNrm;
	}

	m_PSuLMBuilder.RobotCameraPose();

	m_PSuLMBuilder.m_pPSD = &m_PSD;

	//m_PSuLMBuilder.m_pSegmentation = (CRVLSegmentationEB *)m_pSegmentation;

	//if((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
	//	m_PSuLMBuilder.Load(m_PSuLMBuilder.m_ModelDatabasePath,2000);
	if ((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION ||
		(m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING))
	{
		m_PSuLMBuilder.LoadMap();

		if (m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_GENERATION_INDEXING)
			m_PSuLMBuilder.m_Indexing.CreateBase();
	}
	//else if ((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
	//	m_PSuLMBuilder.m_HypothesisArray =
	//		new RVLPSULM_HYPOTHESIS *[m_PSuLMBuilder.m_maxnHypothesesPerModel];

	//FILE *fp;
	//
	//fopen_s(&fp, "C:\\RVL\\ExpRez\\PSuLM.dat", "rb");

	//if(fp)
	//{
	//	m_PSuLMBuilder.Load(fp);

	//	fclose(fp);
	//}

	m_GroundTruth.Init();

	m_GroundTruth.Load();

	LoadMatchMatrix();

	if (m_PSuLMBuilder.m_Flags2 & RVLPSULMBUILDER_FLAG2_MAPBUILDING_MANUAL)
		m_PSuLMBuilder.m_Flags &= ~RVLPSULMBUILDER_FLAG_MAPBUILDING;

	if(m_Flags & (RVLSYS_FLAGS_EDIT_MAP | RVLSYS_FLAGS_REVIEW_RESULTS))
	{
		m_PSuLMBuilder.m_Flags &= ~(RVLPSULMBUILDER_FLAG_MODE | RVLPSULMBUILDER_FLAG_MAPBUILDING);
		m_PSuLMBuilder.m_Flags2 &= ~RVLPSULMBUILDER_FLAG2_SCENE_FUSION;
	}

	if (m_Flags2 & RVLPSULMBUILDER_FLAG2_UNCONSTRAINED_ORIENTATION)
		m_PSuLMBuilder.m_Flags2 |= RVLPSULMBUILDER_FLAG2_HYPGEN_INIT_MATCHING_CONSTRAINTS;

	m_pPSuLM = NULL;

	m_iSample = 0;
}

void CRVLPSuLMVS::Update(DWORD Flags)
{
	//m_Mem.Clear();

	m_iMCMem = (m_iMCMem + 1) % RVLSYS_MCMEMSIZE;

	m_MCMem[m_iMCMem].Clear();

	PSuLMBasedRLMUpdate(RVLPSULMBUILDER_CREATEMODEL_FROM_IMAGE | Flags);
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

	if ((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
		if (m_pPrevPSuLM)
			m_PSuLMBuilder.m_Flags &= ~RVLPSULMBUILDER_FLAG_KIDNAPPED;

	if(m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_KIDNAPPED)
	{
		double *R = m_PoseA0.m_Rot;
		double *t = m_PoseA0.m_X;
		RVLUNITMX3(R)
		RVLNULL3VECTOR(t)
		m_PoseA0.m_Alpha = m_PoseA0.m_Beta = m_PoseA0.m_Theta = 0.0;
	}

	if((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_TRACKING)
		if (m_pPrevPSuLM)
			m_PSuLMBuilder.Localization(m_pPSuLM, &m_PoseA0, m_pPrevPSuLM);

	if ((m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE) == RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION)
		m_PSuLMBuilder.Localization(m_pPSuLM, &m_PoseA0);
	else
		m_PSuLMBuilder.m_PriorProbabilityWorldModel = 0.0;

	char *DepthImageFileName = RVLCreateFileName(m_ImageFileName, "-LW.bmp", -1, "-D.txt");

	if((Flags & RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE) == 0)
	{
		cvSaveImage(m_ImageFileName, m_pRGBImage);		

		RVLSaveDepthImage(m_StereoVision.m_DisparityMap.Disparity, m_StereoVision.m_DisparityMap.Width, 
			m_StereoVision.m_DisparityMap.Height, DepthImageFileName, m_StereoVision.m_DisparityMap.Format, 
			m_StereoVision.m_DisparityMap.Format);
	}

	delete[] DepthImageFileName;

	if(m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING)
	{
		RVLCopyString(m_ImageFileName, &(m_pPSuLM->m_FileName));

		if(m_PSuLMBuilder.MapBuilding(m_pPSuLM))
		{
			if(m_Flags & RVLSYS_FLAGS_CREATE_GLOBAL_MESH)
			{
				// append local 3D mesh to global 3D mesh

				CRVL3DPose *pPoseSM = (m_PSuLMBuilder.m_nPlausibleHypotheses > 0 ? &(m_PSuLMBuilder.m_HypothesisArray[0]->PoseSM) : NULL);

				AppendToMeshFile(pPoseSM);
			}
		}
	}

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

void CRVLPSuLMVS::CreateMeshFile(char *MeshFileName)
{
	m_pMeshFile = new RVL3DMESHFILE;

	m_pMeshFile->Class.m_pMem = &m_Mem;

    m_pMeshFile->Class.m_pMem0 = &m_Mem0;

    m_pMeshFile->Class.m_pMem2 = &m_Mem2;

	m_pMeshFile->Name = RVLCreateString(MeshFileName);

	m_pMeshFile->nLocalMeshes = 0;

	m_pMeshFile->TextureType = 2;
}

void CRVLPSuLMVS::ClearMeshFileData()
{
	if(m_pMeshFile == NULL)
		return;

	delete[] m_pMeshFile->Name;

	delete m_pMeshFile;
}

void CRVLPSuLMVS::AppendToMeshFile(CRVL3DPose *pRelPose)
{
	IplImage *pHSVImage = cvCreateImage(cvSize(m_pRGBImage->width, m_pRGBImage->height), IPL_DEPTH_8U, 3);

	cvCvtColor(m_pRGBImage, pHSVImage, CV_BGR2RGB);

	int nSegments = m_AImage.m_C2DRegion3.m_ObjectList.m_nElements + 1;

	m_PSD.AssignLabels(&(m_AImage.m_C2DRegion), &(m_AImage.m_C2DRegion3));

	CRVL3DMeshObject *Mesh = GenMeshObjects(&(m_AImage.m_C2DRegion.m_ObjectList), pHSVImage, nSegments, &(m_pMeshFile->Class));

	char textureFileName[200] = "Texture";
	sprintf(textureFileName + strlen(textureFileName), "%d.bmp", m_pMeshFile->nLocalMeshes);
	FILE *dat, *mtldat;
	char *GlobalMeshTextureFileName = RVLCreateFileName(m_pMeshFile->Name, ".obj", 0, ".obj.mtl");

	double *RRel = pRelPose->m_Rot;
	double *tRel = pRelPose->m_X;
	double *RAbs = m_pMeshFile->LastLocalMeshPose.m_Rot;
	double *tAbs = m_pMeshFile->LastLocalMeshPose.m_X;

	if(m_pMeshFile->nLocalMeshes == 0)
	{
		dat = fopen(m_pMeshFile->Name, "w");
		fclose(dat);
		mtldat = fopen(GlobalMeshTextureFileName, "w");
		fclose(mtldat);

		//RVLUNITMX3(RAbs);
		//RVLNULL3VECTOR(tAbs);

		m_pMeshFile->iPt = 0;
		m_pMeshFile->iSegment = 0;
	}
	//else
	//{
		//if pRelPose is the pose of the current PSuLM relative to the previous, then the following code should be used.

		//RVLCOMPTRANSF3D(RAbs, tAbs, RRel, tRel, R, t)

		//RVLCOPYMX3X3(R, RAbs)
		//RVLCOPY3VECTOR(t, tAbs)

		//if pRelPose is the absolute pose of the current PSuLM relative to a global reference frame, then the following code should be used.

		RVLCOPYMX3X3(RRel, RAbs)
		RVLCOPY3VECTOR(tRel, tAbs)
	//}
	
	dat = fopen(m_pMeshFile->Name, "a");
		
	mtldat = fopen(GlobalMeshTextureFileName, "a");

	cvSaveImage(textureFileName, m_pRGBImage);	
		
	Mesh->AppendMeshObject2OBJ(m_pMeshFile->iPt, m_pMeshFile->iSegment, &(m_pMeshFile->LastLocalMeshPose), dat, mtldat, 
		GlobalMeshTextureFileName, m_PSD.m_Point3DMap, m_pMeshFile->TextureType, textureFileName);

	m_pMeshFile->nLocalMeshes++;

	delete Mesh;

	fclose(dat);
	fclose(mtldat);
}

bool CRVLPSuLMVS::Create3DMeshFromComplexPSuLM(char *ImageFileName)
{
	DWORD FlagsOld = m_PSuLMBuilder.m_Flags2;

	m_PSuLMBuilder.m_Flags2 &= ~RVLPSULMBUILDER_FLAG2_COMPLEX;

	CRVLMem *pPSDMemOld = m_PSD.m_pMem;

	CRVLMem *pAImageMemOld = m_AImage.m_pMem;

	CRVLMem Mem;

	Mem.Create(10000000);

	m_PSD.m_pMem = &Mem;

	m_AImage.m_pMem = &Mem;

	CreateMeshFile("Mesh-00000.obj");

	m_PSuLMBuilder.m_ImageFileName = RVLCreateString(ImageFileName);

	char *DisparityImageFileName = RVLCreateFileName(ImageFileName, "-LW.bmp", -1, "-D.txt");

	int iSample = RVLGetFileNumber(ImageFileName, "00000-LW.bmp");

	int iSample0 = iSample;

	//CRVLPSuLM *pPSuLM_;

	CRVL3DPose PoseM_M;

	double *tM_M = PoseM_M.m_X;

	RVLNULL3VECTOR(tM_M)

	bool bOK = true;

	unsigned char command;
	unsigned int DepthFormat;
	
	do
	{
		RVLSetFileNumber(m_PSuLMBuilder.m_ImageFileName, "00000-LW.bmp", iSample);

		if (!m_PSuLMBuilder.GetOdometry(m_PSuLMBuilder.m_ImageFileName, &PoseM_M, "-LW.bmp", iSample0, command))
		{
			bOK = false;

			break;
		}

		m_iMCMem = (m_iMCMem + 1) % RVLSYS_MCMEMSIZE;

		m_MCMem[m_iMCMem].Clear();		

		m_PSuLMBuilder.m_pMCMem = m_MCMem + m_iMCMem;		

		//pPSuLM_ = m_PSuLMBuilder.Create(RVLPSULMBUILDER_CREATEMODEL_FROM_IMAGE | RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE);

		RVLSetFileNumber(DisparityImageFileName, "00000-D.txt", iSample);

		if(!RVLImportDisparityImage(DisparityImageFileName, &(m_StereoVision.m_DisparityMap), 
			DepthFormat, m_StereoVision.m_zToDepthLookupTable))
		{
			bOK = false;

			break;
		}

		m_AImage.Create();

		m_PSD.GetPointsWithDisparity(&(m_StereoVision.m_DisparityMap));

		m_PSD.Segment(&(m_AImage.m_C2DRegion),&(m_AImage.m_C2DRegion2),&(m_AImage.m_C2DRegion3),&Mem);

		m_pRGBImage = cvLoadImage(m_PSuLMBuilder.m_ImageFileName);

		AppendToMeshFile(&PoseM_M);

		Mem.Clear();

		iSample++;
	}while(command != 'C');

	delete[] DisparityImageFileName;

	m_PSuLMBuilder.m_Flags2 = FlagsOld;

	m_PSD.m_pMem = pPSDMemOld;

	m_AImage.m_pMem = pAImageMemOld;

	RVLSetFileNumber(m_PSuLMBuilder.m_ImageFileName, "00000-LW.bmp", iSample0);

	return bOK;
}

void CRVLPSuLMVS::CreateLocal3DMesh(CRVLPSuLM *pPSuLM0)
{
	CRVL3DPose NullPose;
	double C[3 * 3 * 3];
	double *C_;

	RVLNULLMX3X3(C)

	if(!(m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF))
	{
		C_ = C + 9;
		RVLNULLMX3X3(C_)
		C_ += 9;
		RVLNULLMX3X3(C_)
	}

	NullPose.Reset();

	NullPose.m_C = C;

	NullPose.m_ParamFlags = (m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_HYPOTHESES_UNCERTAINTY_3DOF ? RVL3DPOSE_PARAM_FLAGS_COV_3D : RVL3DPOSE_PARAM_FLAGS_COV_6D);

	CRVLMem Mem;

	Mem.Create(m_PSuLMBuilder.m_PSuLMList.m_nElements * sizeof(RVLPTRCHAIN_ELEMENT));

	CRVLMem *pMemOld = m_PSuLMBuilder.m_PSuLMSubList.m_pMem;

	m_PSuLMBuilder.m_PSuLMSubList.m_pMem = &Mem;

	m_PSuLMBuilder.m_PSuLMSubList.RemoveAll();

	m_PSuLMBuilder.GetLocalModels(NULL, &NullPose, pPSuLM0, m_PSuLMBuilder.m_LocalMapRadius / m_PSuLMBuilder.m_MinHybridLocalizationDist);

	CreateMeshFile("Mesh-00000.obj");

	FILE *fp = fopen("C:\\RVL\\Debug\\Local3DMesh.log", "w");

	CRVLPSuLM *pPSuLM;

	m_PSuLMBuilder.m_PSuLMSubList.Start();

	while(m_PSuLMBuilder.m_PSuLMSubList.m_pNext)
	{
		pPSuLM = (CRVLPSuLM *)(m_PSuLMBuilder.m_PSuLMSubList.GetNext());

		//// only for debugging purposes!!!

		//RVLPSULM_NEIGHBOR2 *pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pPSuLM0->m_LocalMap.pFirst);
	
		//while(pNeighbor)
		//{
		//	if(pNeighbor->pMPSuLM == pPSuLM)
		//		int debug = 0;

		//	pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
		//}	

		///////

		fprintf(fp, "%d\n", pPSuLM->m_Index);

		m_iMCMem = (m_iMCMem + 1) % RVLSYS_MCMEMSIZE;

		m_MCMem[m_iMCMem].Clear();		

		m_PSuLMBuilder.m_pMCMem = m_MCMem + m_iMCMem;

		m_PSuLMBuilder.m_ImageFileName = pPSuLM->m_FileName; 

		m_pPSuLM = m_PSuLMBuilder.Create(RVLPSULMBUILDER_CREATEMODEL_FROM_IMAGE | RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE);

		m_pRGBImage = cvLoadImage(pPSuLM->m_FileName);

		AppendToMeshFile(&(pPSuLM->m_PoseRTAs));
	}

	RVLResetFlags<CRVLPSuLM>(&(m_PSuLMBuilder.m_PSuLMSubList), RVLPSULM_FLAG_CLOSE);

	m_PSuLMBuilder.m_PSuLMSubList.m_pMem = pMemOld;

	fclose(fp);
}

void CRVLPSuLMVS::Validate()
{
	RVLPSULM_GROUND_TRUTH_MATCH **GTMatch = new RVLPSULM_GROUND_TRUTH_MATCH *[m_PSuLMBuilder.m_PSuLMList.m_nElements];

	int iSample = RVLGetFileNumber(m_ImageFileName, "00000-LW.bmp");

	int nGTMatches;

	m_GroundTruth.Get(iSample, GTMatch, nGTMatches);

	CRVL3DPose ePose;
	CRVL3DPose PoseSMr;

	double *RSMr = PoseSMr.m_Rot;
	double *tSMr = PoseSMr.m_X;

	int i, j;
	RVLPSULM_HYPOTHESIS *pHypothesis;
	RVLPSULM_GROUND_TRUTH_MATCH *pGTMatch;
	double *RSM, *tSM;
	CRVLPSuLM *pPSuLMr;
	RVLPSULM_NEIGHBOR2 *pNeighbor;
	double *RMMr, *tMMr;
	bool bConnected;
	double dist, angle;

	for(i = 0; i < m_PSuLMBuilder.m_nHypotheses; i++)
	{
		pHypothesis = m_PSuLMBuilder.m_HypothesisArray[i];

		if(pHypothesis->iRepresentative != 0xffffffff)
			continue;

		pHypothesis->validation = -1;

		RSM = pHypothesis->PoseSM.m_Rot;
		tSM = pHypothesis->PoseSM.m_X;

		for(j = 0; j < nGTMatches; j++)
		{
			pGTMatch = GTMatch[j];

			if(pGTMatch->iModel == pHypothesis->pMPSuLM->m_Index)
			{
				RVLCOPYMX3X3(RSM, RSMr)
				RVLCOPY3VECTOR(tSM, tSMr)

				bConnected = true;
			}
			else
			{
				pPSuLMr = m_PSuLMBuilder.m_PSuLMArray[pGTMatch->iModel];

				pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pPSuLMr->m_LocalMap.pFirst);

				while(pNeighbor)
				{
					if(pNeighbor->pMPSuLM == pHypothesis->pMPSuLM)
						break;

					pNeighbor = (RVLPSULM_NEIGHBOR2 *)(pNeighbor->pNext);
				}

				if(pNeighbor)
				{
					RMMr = pNeighbor->PoseRel.m_Rot;
					tMMr = pNeighbor->PoseRel.m_X;

					RVLCOMPTRANSF3D(RMMr, tMMr, RSM, tSM, RSMr, tSMr)

					bConnected = true;
				}
				else
					bConnected = false;
			}

			if(bConnected)
			{
				pGTMatch->PoseSM.Diff(&PoseSMr, dist, angle);

				if(dist <= 200.0 && angle <= 15.0 * DEG2RAD)
				{
					pHypothesis->validation = 1;

					break;
				}
				else if(dist <= 1000.0 && angle <= 60.0 * DEG2RAD)
					pHypothesis->validation = 0;
			}
		}	// for each GT match
	}	// for each hypothesis

	char *MatchMatrixGT = m_MatchMatrixGT + iSample * (m_PSuLMBuilder.m_maxPSuLMIndex + 1);

	CRVLPSuLM *pPSuLM;

	for(i = 0; i <= m_PSuLMBuilder.m_maxPSuLMIndex; i++)
	{
		MatchMatrixGT[i] = -2;

		pPSuLM = m_PSuLMBuilder.m_PSuLMArray[i];

		if(pPSuLM)
			if(pPSuLM->m_pHypothesis)
				MatchMatrixGT[i] = pPSuLM->m_pHypothesis->validation;
	}

	delete[] GTMatch;
}

void CRVLPSuLMVS::LoadMatchMatrix()
{
	m_nSamples = 0;

	FILE *fp = fopen("C:\\RVL\\ExpRez\\Validation.log", "r");

	if(fp == NULL)
		return;

	//m_nMatches = 0;	

	//char line[200];
	//int iSample;

	//while(!feof(fp))
	//{
	//	fgets(line, 200, fp);

	//	m_nMatches++;
	//}

	//fclose(fp);

	//m_nMatches--;

	//if(m_HypothesisArrayGT)
	//	delete m_HypothesisArrayGT;

	//m_HypothesisArrayGT = new RVLPSULM_HYPOTHESIS_GT[m_nMatches];

	//fp = fopen("C:\\RVL\\ExpRez\\Validation_141111.log", "r");

	//int iMatch = 0;

	//int validation;

	//while(!feof(fp))
	//{
	//	fscanf(fp, "%d\t%d\t%lf\t%d\n", &(m_HypothesisArrayGT[iMatch].iSample), &(m_HypothesisArrayGT[iMatch].iHypothesis), 
	//		&(m_HypothesisArrayGT[iMatch].P), &validation);

	//	m_HypothesisArrayGT[iMatch].validation = (char)validation;

	//	iMatch++;
	//}

	//fp = fopen("C:\\RVL\\ExpRez\\Validation_141111.log", "r");

	/////

	int nPSuLMS = m_PSuLMBuilder.m_maxPSuLMIndex + 1;

	if(nPSuLMS == 0)
		return;

	char line[200];
	int iSample;

	while(!feof(fp))
	{
		fgets(line, 200, fp);

		sscanf(line, "%d", &iSample);

		if(iSample > m_nSamples)
			m_nSamples = iSample;
	}

	fclose(fp);

	m_nSamples++;

	int nMatches = m_nSamples * nPSuLMS;

	if(m_MatchMatrix)
		delete[] m_MatchMatrix;

	m_MatchMatrix = new double[nMatches];

	memset(m_MatchMatrix, 0, nMatches * sizeof(double));

	if(m_MatchMatrixGT)
		delete[] m_MatchMatrixGT;

	m_MatchMatrixGT = new char[nMatches];

	memset(m_MatchMatrixGT, 0xfe, nMatches * sizeof(char));

	fp = fopen("C:\\RVL\\ExpRez\\Validation.log", "r");

	int iPSuLM;
	double P;
	int validation;
	int iMatch;

	while(!feof(fp))
	{
		fscanf(fp, "%d\t%d\t%lf\t%d\n", &iSample, &iPSuLM, &P, &validation);

		iMatch = iSample * nPSuLMS + iPSuLM;

		m_MatchMatrix[iMatch] = P;

		m_MatchMatrixGT[iMatch] = (char)validation;
	}

	fclose(fp);
}

void CRVLPSuLMVS::SaveMatchMatrix()
{
	FILE *fp = fopen("C:\\RVL\\ExpRez\\Validation.log", "w");

	if(fp == NULL)
		return;

	int nPSuLMS = m_PSuLMBuilder.m_maxPSuLMIndex + 1;

	int iMatch = 0;

	int iSample, iPSuLM;
	//RVLPSULM_HYPOTHESIS *pHypothesis;
	//double P;

	for(iSample = 0; iSample < m_nSamples; iSample++)
		for(iPSuLM = 0; iPSuLM < nPSuLMS; iPSuLM++, iMatch++)
			if(m_MatchMatrixGT[iMatch] != -2)
				fprintf(fp, "%d\t%d\t%lf\t%d\n", iSample, iPSuLM, m_MatchMatrix[iMatch], m_MatchMatrixGT[iMatch]);

	fclose(fp);
}

//void CRVLPSuLMVS::StoreHypothesesToMatchMatrix(int iSample)
//{
//	int nPSuLMS = m_PSuLMBuilder.m_maxPSuLMIndex + 1;
//
//	CRVLPSuLM *pPSuLM;
//	RVLPSULM_HYPOTHESIS *pHypothesis;
//	double P, dP;
//	int iMatch;
//
//	m_PSuLMBuilder.m_PSuLMList.Start();
//
//	while(m_PSuLMBuilder.m_PSuLMList.m_pNext)
//	{
//		pPSuLM = (CRVLPSuLM *)(m_PSuLMBuilder.m_PSuLMList.GetNext());
//
//		pHypothesis = pPSuLM->m_pHypothesis;
//
//		if(pHypothesis)
//		{
//			//if(iSample == 2 && pHypothesis->Index == 724)
//			//	int debug = 0;
//
//			P = exp(pHypothesis->Probability - pHypothesis->pMPSuLM->m_pHypothesis->Probability) / pHypothesis->pMPSuLM->m_PriorProbabilityLocal;
//
//			for(int i = 0; i < m_nMatches; i++)
//				if(m_HypothesisArrayGT[i].iSample == iSample && m_HypothesisArrayGT[i].iHypothesis == pHypothesis->Index)
//				{
//					dP = m_HypothesisArrayGT[i].P - P;
//
//					if(RVLABS(dP) > 1e-6)
//						int debug = 0;
//
//					iMatch = iSample * nPSuLMS + pPSuLM->m_Index;
//
//					m_MatchMatrix[iMatch] = pHypothesis->Probability;
//					//m_MatchMatrix[iMatch] = P;
//					m_MatchMatrixGT[iMatch] = m_HypothesisArrayGT[i].validation;
//				}
//		}
//	}
//}

//void CRVLPSuLMVS::SaveValidation()
//{
//	int iSample = RVLGetFileNumber(m_ImageFileName, "00000-LW.bmp");
//
//	FILE *fp = fopen("C:\\RVL\\ExpRez\\Validation.log", "a");
//
//	RVLPSULM_HYPOTHESIS *pHypothesis;
//	double P;
//	CRVLPSuLM *pPSuLM;
//
//	m_PSuLMBuilder.m_PSuLMList.Start();
//
//	while(m_PSuLMBuilder.m_PSuLMList.m_pNext)
//	{
//		pPSuLM = (CRVLPSuLM *)(m_PSuLMBuilder.m_PSuLMList.GetNext());
//
//		pHypothesis = pPSuLM->m_pHypothesis;
//
//		if(pHypothesis)
//		{
//			P = exp(pHypothesis->Probability - pHypothesis->pMPSuLM->m_pHypothesis->Probability) / pHypothesis->pMPSuLM->m_PriorProbabilityLocal;
//
//			fprintf(fp, "%d\t%d\t%d\t%lf\t%d\n", iSample, pPSuLM->m_Index, pHypothesis->Index, P, pHypothesis->validation);
//		}
//	}
//
//	fclose(fp);
//}

void CRVLPSuLMVS::ComputeMatchMatrix(int iSample)
{
	int nPSuLMs = m_PSuLMBuilder.m_maxPSuLMIndex + 1;

	int MatchMatrixOffset = iSample * nPSuLMs;

	double *MatchMatrix = m_MatchMatrix + MatchMatrixOffset;
	//double *MatchMatrixGT = m_MatchMatrixGT + MatchMatrixOffset;

	DWORD HypothesisEvaluationFlagsOld = m_PSuLMBuilder.m_HypothesisEvaluationFlags;
	double PPriorSurfacePositionOld = m_PSuLMBuilder.m_SurfaceMatchData.PPriorPosition;
	double PPriorLinePositionOld = m_PSuLMBuilder.m_LineMatchData.PPriorPosition;

	m_PSuLMBuilder.m_SurfaceMatchData.PPriorPosition = 6.9;		// 1 surface at each 1000 mm
	m_PSuLMBuilder.m_LineMatchData.PPriorPosition = 13.81;		// 1 line in each 1000 mm^2

	m_PSuLMBuilder.m_HypothesisEvaluationFlags = 0x00000000;

	m_PSuLMBuilder.CreateAutoMatchMatrix(m_pPSuLM);

	//double BestHypothesisProbability = m_PSuLMBuilder.m_HypothesisArray[0]->Probability;

	//if(m_PSuLMBuilder.m_nHypotheses > 0)
	//{	
	//	double PriorProbabilityGlobal = 0.0;

	//	double *P = new double[m_PSuLMBuilder.m_HypothesisList.m_nElements];

	//	RVLQLIST_PTR_ENTRY *pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(m_PSuLMBuilder.m_RepresentativeHypothesisList.pFirst);

	//	RVLPSULM_HYPOTHESIS *pHypothesis;

	//	while(pHypothesisPtr)
	//	{
	//		pHypothesis = (RVLPSULM_HYPOTHESIS *)(pHypothesisPtr->Ptr);

	//		//P[pHypothesis->Index] = m_PSuLMBuilder.EvaluateHypothesis4(m_pPSuLM, pHypothesis, false);
	//		P[pHypothesis->Index] = m_PSuLMBuilder.EvaluateHypothesis4(m_pPSuLM, pHypothesis);

	//		PriorProbabilityGlobal += exp(P[pHypothesis->Index] - BestHypothesisProbability);

	//		pHypothesisPtr = (RVLQLIST_PTR_ENTRY *)(pHypothesisPtr->pNext);
	//	}

	//	double POffset = BestHypothesisProbability + log(PriorProbabilityGlobal);

	//	int i;
	//	CRVLPSuLM *pPSuLM;

	//	for(i = 0; i < nPSuLMs; i++)
	//	{
	//		MatchMatrix[i] = -1e8;

	//		pPSuLM = m_PSuLMBuilder.m_PSuLMArray[i];

	//		if(pPSuLM)
	//			if(pPSuLM->m_pHypothesis)
	//				MatchMatrix[i] = P[pPSuLM->m_pHypothesis->Index] - POffset;
	//	}

	//	delete[] P;
	//}	

	int nSFeatures = m_pPSuLM->m_n3DSurfaces + m_pPSuLM->m_n3DLines;
	
	int i, j;
	CRVLPSuLM *pPSuLM;
	double P;

	for(i = 0; i < nPSuLMs; i++)
	{
		MatchMatrix[i] = 0.0;

		if(m_PSuLMBuilder.m_nHypotheses == 0)
			continue;

		pPSuLM = m_PSuLMBuilder.m_PSuLMArray[i];

		if(pPSuLM)
			if(pPSuLM->m_pHypothesis)
			{
				P = m_PSuLMBuilder.EvaluateHypothesis4(m_pPSuLM, pPSuLM->m_pHypothesis);

				for(j = 0; j < nSFeatures; j++)
					m_PSuLMBuilder.m_SMatchArray[j].b = false;

				m_PSuLMBuilder.m_PriorProbabilityWorldModel = m_PSuLMBuilder.ConditionalProbabilityTree(m_pPSuLM, pPSuLM);

				MatchMatrix[i] = P - m_PSuLMBuilder.m_PriorProbabilityWorldModel;
			}
	}	

	m_PSuLMBuilder.m_HypothesisEvaluationFlags = HypothesisEvaluationFlagsOld;
	m_PSuLMBuilder.m_SurfaceMatchData.PPriorPosition = PPriorSurfacePositionOld;	
	m_PSuLMBuilder.m_LineMatchData.PPriorPosition = PPriorLinePositionOld;			
}

BOOL CRVLPSuLMVS::GetNextImageFileName(bool bBackwards)
{
	if ((m_PSuLMBuilder.m_Flags2 & RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT) == RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG)
	{
		char *ImageFileName = m_PSuLMBuilder.m_DataSet.GetNextRGBFileName(m_ImageFileName, bBackwards);

		if (ImageFileName)
		{
			delete[] m_ImageFileName;

			m_ImageFileName = ImageFileName;

			return TRUE;
		}
		else
			return FALSE;
	}		
	else
		return RVLGetNextFileName(m_ImageFileName, "00000-LW.bmp", 10000, m_SampleStep);
}

BOOL CRVLPSuLMVS::GetFirstValidImageFileName()
{
	if ((m_PSuLMBuilder.m_Flags2 & RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT) == RVLPSULMBUILDER_FLAG2_IMAGE_FORMAT_FREIBURG)
	{
		char *ImageFileName = m_PSuLMBuilder.m_DataSet.GetValidRGBFileName(m_ImageFileName);

		if (ImageFileName)
		{
			delete[] m_ImageFileName;

			m_ImageFileName = ImageFileName;

			return TRUE;
		}
		else
			return FALSE;
	}
	else
		return RVLGetFirstValidFileName(m_ImageFileName, "00000-sl.bmp", 10000);
}

void RVLPSuLMDisplayMouseCallback2(int event, int x, int y, int flags, void* vpData)
{
	CRVL3DPose NullPose;

	RVLNULL3VECTOR(NullPose.m_X);
	RVLUNITMX3(NullPose.m_Rot);

	RVLPSULMDISPLAY_MOUSE_CALLBACK_DATA *pData = (RVLPSULMDISPLAY_MOUSE_CALLBACK_DATA *)vpData;

	CRVL3DSurface2 *pSelectedSurf = pData->pSelectedSurf;
	CRVL3DLine2 *pSelectedLine = NULL;
	RVL3DSURFACE_SAMPLE *pSelectedSurfSample = NULL;

	CRVLGUI *pGUI = pData->pGUI;
	CRVLFigure *pFig = pData->pFig;
	CRVLFigure *pFig2 = pData->pFig2;
	CRVLPSuLM *pPSuLM, *pPSuLM2;

	CRVLFigure *pSFig, *pMFig;

	CRVLPSuLMVS *pVS = pData->pVS;

	RVLPSULM_HYPOTHESIS *pHypothesis = NULL;

	if(pVS->m_PSuLMBuilder.m_HypothesisList.m_nElements > 0 && pData->iHypothesis >= 0 && 
		pData->iHypothesis < pVS->m_PSuLMBuilder.m_HypothesisList.m_nElements)
		pHypothesis = pVS->m_PSuLMBuilder.m_HypothesisArray[pData->iHypothesis];

	DWORD HypEvalMethod = (pVS->m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD);

	int nMatchMatrixCols;
	int nSSurfaces, nMSurfaces;
	int nSLines, nMLines;

	if(pHypothesis)
	{
		if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
		{
			nSSurfaces = pVS->m_pPSuLM->m_n3DSurfaces;
			nSLines = pVS->m_pPSuLM->m_n3DLines;
			//nMSurfaces = pHypothesis->pMPSuLM->m_n3DSurfaces;
			//nMLines =  pHypothesis->pMPSuLM->m_n3DLines;
			nMSurfaces = pHypothesis->pMPSuLM->m_n3DSurfacesTotal;
			nMLines =  pHypothesis->pMPSuLM->m_n3DLinesTotal;
			nMatchMatrixCols = nMSurfaces + nMLines;
		}
		else
		{
			nSSurfaces = pVS->m_pPSuLM->m_n3DSurfacesTotal;
			nMSurfaces = pHypothesis->pMPSuLM->m_n3DSurfacesTotal;
			nMLines =  pHypothesis->pMPSuLM->m_n3DLines;
			nMatchMatrixCols = nMSurfaces;
		}
	}

	int w = pData->w;

	int wExt = (2 * pVS->m_PSD.m_nFOVExtensions + 1) * w;
	
	int iPix;
	int a, b;
	int nSurfaces, nSurfaces2, nLines, nLines2;
	CRVL3DSurface2 *pSelectedSurf_;
	int iFOVExtension;

	switch( event )
	{
		case CV_EVENT_LBUTTONDOWN:
			//if(!pData->bSelection)
			{
				pData->u = x;

				pData->v = y;

				pData->bSelection = true;				

				iPix = x / pData->ZoomFactor + y / pData->ZoomFactor * wExt;

				if(pFig->m_Flags & RVLPSULM_DISPLAY_SCENE)
				{
					pSFig = pFig;
					pPSuLM = pVS->m_pPSuLM;
					if(pHypothesis)
					{
						pMFig = pFig2;
						pPSuLM2 = pHypothesis->pMPSuLM;
						a = nMatchMatrixCols;
						b = 1;
						nSurfaces = nSSurfaces;
						nSurfaces2 = nMSurfaces;
						nLines = nSLines;
						nLines2 = nMLines;
					}
				}
				else if(pFig->m_Flags & RVLPSULM_DISPLAY_MODEL)
				{
					pSFig = pFig2;
					pPSuLM2 = pVS->m_pPSuLM;
					if(pHypothesis)
					{
						pMFig = pFig;					
						pPSuLM = pHypothesis->pMPSuLM;
						a = 1;
						b = nMatchMatrixCols;
						nSurfaces = nMSurfaces;
						nSurfaces2 = nSSurfaces;
						nLines = nMLines;
						nLines2 = nSLines;
					}
					else
						pPSuLM = NULL;
				}

				if(pPSuLM)
				{
					DWORD CameraFlagsOld = pVS->m_PSuLMBuilder.m_pCamera->m_Flags;

					if(pPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX)
					{
						pVS->m_PSuLMBuilder.m_pCamera->m_Flags |= RVLCAMERA_FLAG_SPHERICAL;

						w = pVS->m_PSuLMBuilder.m_pCamera->m_wSpherical;
					}

					iPix = x / pData->ZoomFactor + y / pData->ZoomFactor * w;

					pSelectedSurf_ = pSelectedSurf;

					pPSuLM->Project(&(pFig->m_PoseC0), FALSE, iPix, &pSelectedSurf, &pSelectedLine, &iFOVExtension);

					if (pVS->m_PSuLMBuilder.m_Flags2 & RVLPSULMBUILDER_FLAG2_HYPOTHESIS_EVALUATION_SAMPLE_MATCHING)
					{
						if (pSelectedSurf)
							pSelectedSurf_ = pSelectedSurf;

						if (pSelectedSurf_)
							pSelectedSurfSample = pPSuLM->SelectSample(pFig, pSelectedSurf_, &(pFig->m_PoseC0), x / pData->ZoomFactor, y / pData->ZoomFactor);	

						if (pSelectedSurfSample)
							pSelectedSurf = pSelectedSurf_;
					}

					pData->pSelectedSurf = pSelectedSurf;

					pVS->m_PSuLMBuilder.m_pCamera->m_Flags = CameraFlagsOld;
				}

				pFig->Clear();

				if(pHypothesis)
				{
					pFig2->Clear();

					pVS->m_PSuLMBuilder.DisplayHypothesis(pGUI, pSFig, pMFig, pVS->m_pPSuLM, pData->mDisplayPSuLMFlags, 
						pData->pImage, pData->pImage2, pData->iHypothesis);
				}
				else
				{
					pSFig->m_pImage = cvCloneImage(pData->pImage);

					pVS->m_pPSuLM->Display(pSFig, &NullPose, cvScalar(0, 255, 0), pData->mDisplayPSuLMFlags);
				}

				RVLResetFlags<CRVL2DRegion2>(&(pVS->m_AImage.m_C2DRegion.m_ObjectList), RVLOBJ2_FLAG_MARKED);

				if(pSelectedSurf)
				{
					pPSuLM->Display3DSurface(pFig, pSelectedSurf, &NullPose, cvScalar(255, 255, 0), 2,
						RVLPSULM_DISPLAY_VECTORS, pSelectedSurfSample);

					CRVL2DRegion2 *p2DRegion;

					if (!(pPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX))
					{
						p2DRegion = (CRVL2DRegion2 *)(pSelectedSurf->m_vp2DRegion);

						if(pFig->m_Flags & RVLPSULM_DISPLAY_SCENE)
						{
							RVLSegmentationDisplayBoundary(pFig, p2DRegion, pData->w, &(pVS->m_Mem2), cvScalar(255, 255, 0), 1, 
								(iFOVExtension + pVS->m_PSD.m_nFOVExtensions) * pVS->m_PSD.m_Width);

							pVS->m_Mem2.Clear();
						}
					}

					if(pHypothesis != NULL && pSelectedSurf->m_Index < nSurfaces)
					{
						BOOL bCorrespondent;
						CRVL3DSurface2 *pSurf2;

						for(int iMatch = 0; iMatch < nSurfaces2; iMatch++)
						{
							bCorrespondent = FALSE;

							if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_SSM)
							{
								if(pVS->m_PSuLMBuilder.m_MatchMatrix[a * pSelectedSurf->m_Index + b * iMatch] >= 
									pVS->m_PSuLMBuilder.m_minSurfaceSamplesForMatch)
									bCorrespondent = TRUE;
							}
							else if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
							{
								if(pVS->m_PSuLMBuilder.m_MatchMatrix[a * pSelectedSurf->m_Index + b * iMatch] == 1)
									bCorrespondent = TRUE;
							}
							else
							{
	#ifdef RVLPSULMBUILDER_DISPLAY_MATCH_OVERLAP
								if(pVS->m_PSuLMBuilder.m_MatchMatrix[a * pSelectedSurf->m_Index + b * iMatch] == 2)
	#else
								if(m_pPSuLMBuilder->m_MatchMatrix[pSelectedSurf->m_Index +  nMSurfaces * iMatch] > 0)
	#endif
									bCorrespondent = TRUE;
							}

							if(bCorrespondent)
							{
								pSurf2 = pPSuLM2->m_3DSurfaceArray[iMatch];

								pPSuLM2->Display3DSurface(pFig2, pSurf2, &NullPose, cvScalar(255, 255, 0), 2,
									RVLPSULM_DISPLAY_VECTORS);
							}
						}	// for(int iMatch = 0; iMatch < nSurfaces2; iMatch++)
					}	// if(pHypothesis != NULL && pSelectedSurf->m_Index < nSurfaces)

					if (!(pPSuLM->m_Flags & RVLPSULM_FLAG_COMPLEX))
					{
						RVLARRAY *pRelList = p2DRegion->m_RelList + p2DRegion->m_pClass->m_iRelList[RVLRELLIST_ELEMENTS];

						CRVL2DRegion2 **ppTriangle;

						CRVL2DRegion2 *pTriangle;

						for (ppTriangle = (CRVL2DRegion2 **)(pRelList->pFirst); ppTriangle < (CRVL2DRegion2 **)(pRelList->pEnd); ppTriangle++)
						{
							pTriangle = *ppTriangle;

							pTriangle->m_Flags |= RVLOBJ2_FLAG_MARKED;
						}
					}
				}	// if(pSelectedSurf)

				if(pSelectedLine)
				{
					pPSuLM->Display3DLine(pFig, pSelectedLine, &NullPose, cvScalar(255, 255, 0), 2, RVLPSULM_DISPLAY_VECTORS);

					if(pSelectedLine->m_Index < nSLines)
					{
						if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
						{
							if(pHypothesis)
							{
								BOOL bCorrespondent;
								CRVL3DLine2 *pLine2;

								for(int iMatch = 0; iMatch < nLines2; iMatch++)
								{
									bCorrespondent = FALSE;

									if(pVS->m_PSuLMBuilder.m_MatchMatrix[a * (pSelectedLine->m_Index) + b * iMatch + nMSurfaces] > 0)
										bCorrespondent = TRUE;

									if(bCorrespondent)
									{
										pLine2 = pPSuLM2->m_3DLineArray[iMatch];

										pPSuLM2->Display3DLine(pFig2, pLine2, &NullPose, cvScalar(255, 255, 0), 2, RVLPSULM_DISPLAY_VECTORS);
									}
								}
							}
						}
					}
				}

				pGUI->DisplayVectors(pSFig, 0, 0, (double)(pData->ZoomFactor));

				pGUI->ShowFigure(pSFig);

				if(pHypothesis)
				{
					pGUI->DisplayVectors(pMFig, 0, 0, 1.0);	

					pGUI->ShowFigure(pMFig);
				}

				pVS->m_PSuLMBuilder.DisplayHypothesisData(pFig, pVS->m_pPSuLM, pData->MatchMatrixGT, pData->mDisplayPSuLMFlags, pData->iHypothesis, pSelectedSurf, pSelectedLine, 
					pSelectedSurfSample);
			}
	}	//	switch( event )
}

