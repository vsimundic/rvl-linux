//#include "stdafx.h"

#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "PSGM.h"
#include <Eigen\Eigenvalues>
#include <random> //VIDOVIC
#include <nanoflann.hpp>

//#define RVLPSGM_CTIMESH_DEBUG

using namespace RVL;
using namespace RECOG;

PSGM::PSGM()
{
	mode = RVLRECOGNITION_MODE_RECOGNITION;
	bZeroRFDescriptor = false;
	bGTRFDescriptors = false;
	bMatchRANSAC = false;

	nDominantClusters = 1;
	kNoise = 1.2f;
	minInitialSurfelSize = 20;
	minVertexPerc = 50;
	kReferenceSurfelSize = 0.2f;
	kReferenceTangentSize = 0.3f;
	baseSeparationAngle = 22.5f;
	//edgeTangentAngle = 100.0f;	
	nModels = 35; //Vidovic
	nMSegments = 3; //Vidovic
	minClusterSize = 400;
	maxClusterSize = 66122;
	minSignificantClusterSize = 3200;
	minClusterBoundaryDiscontinuityPerc = 75;
	minClusterNormalDistributionStd = 0.1f;
	groundPlaneTolerance = 0.020f;

	convexTemplate66.n = 66;
	convexTemplate66.Element = new RECOG::PSGM_::Plane[convexTemplate66.n];

	CreateTemplate66();

	convexTemplate = convexTemplate66;

	CreateTemplateBox();

	//Vidovic
	centroidID.n = 6;
	centroidID.Element = new QLIST::Index[centroidID.n];

	ConvexTemplateCentoidID();
	//END Vidovic

	clusters.Element = NULL;
	clusterMap = NULL;
	clusterMem = NULL;
	clusterSurfelMem = NULL;
	clusterVertexMem = NULL;
	//modelInstanceMem = NULL;
	sceneFileName = NULL;
	modelInstanceDB.Element = NULL; //Vidovic
	modelInstanceDB.n = 0; //Vidovic
	modelDataBase = NULL; //Vidovic
	modelsInDataBase = NULL; //Vidovic
	sceneMIMatch = NULL; //Vidovic	

	//nSamples = 20; //Vidovic
	stdNoise = 2; //Vidovic

	bNormalValidityTest = true; //Vidovic

	iScene = 0;

	//Vidovic
	pECCVGT = new ECCVGTLoader;

	scoreMatchMatrix.Element = NULL;
	scoreMatchMatrix.n = 0;

	scoreMatchMatrixICP.Element = NULL;
	scoreMatchMatrixICP.n = 0;

	nBestMatches = 7; //add loading from file

	//Arrays allocation for Match function
	iValidSampleCandidate.Element = new QLIST::Index[convexTemplate.n];
	
	iValid.Element = new QLIST::Index[convexTemplate.n];
	
	iRansacCandidates.Element = new QLIST::Index[26]; //max 26 planes which satisfy condition
	
	iConsensus.Element = new QLIST::Index[convexTemplate.n];
	
	iConsensusTemp.Element = new QLIST::Index[convexTemplate.n];

	pCTImatchesArray.Element = NULL;
	pCTImatchesArray.n = 0;

	segmentGT.Element = NULL;

	e.Element = NULL;

	tBestMatch.Element = NULL;

	score.Element = NULL;

	pCTImatchesArray.Element = NULL;

	scoreMatchMatrix.Element = NULL;
	scoreMatchMatrix.n = 0;

	icpTMatrix = NULL;

	//fpTime = fopen("C:\\RVL\\MatchTime_WithoutRansac.txt", "w");
	//End Vidovic

	bGnd = false;
}


PSGM::~PSGM()
{
	RVL_DELETE_ARRAY(clusters.Element);
	RVL_DELETE_ARRAY(clusterMap);
	RVL_DELETE_ARRAY(clusterMem);
	RVL_DELETE_ARRAY(clusterSurfelMem);
	RVL_DELETE_ARRAY(clusterVertexMem);
	RVL_DELETE_ARRAY(convexTemplate66.Element);
	RVL_DELETE_ARRAY(convexTemplateBox.Element);
	//RVL_DELETE_ARRAY(modelInstanceMem);
	RVL_DELETE_ARRAY(sceneFileName);
	RVL_DELETE_ARRAY(modelInstanceDB.Element); //Vidovic
	RVL_DELETE_ARRAY(modelDataBase); //Vidovic
	RVL_DELETE_ARRAY(modelsInDataBase); //Vidovic
	RVL_DELETE_ARRAY(sceneMIMatch); //Vidovic
	RVL_DELETE_ARRAY(centroidID.Element); //Vidovic
	RVL_DELETE_ARRAY(pCTImatchesArray.Element); //Vidovic
	RVL_DELETE_ARRAY(segmentGT.Element); //Vidovic

	//Vidovic
	int iSSegment;

	pECCVGT->~ECCVGTLoader();

	//Delete arrays used in Match() function
	RVL_DELETE_ARRAY(iValidSampleCandidate.Element);

	RVL_DELETE_ARRAY(iValid.Element);

	RVL_DELETE_ARRAY(iRansacCandidates.Element);

	RVL_DELETE_ARRAY(iConsensus.Element);

	RVL_DELETE_ARRAY(iConsensusTemp.Element);	

	for (int i = 0; i < MCTISet.pCTI.n; i++)
	{
		RVL_DELETE_ARRAY(e.Element[i].Element);
		RVL_DELETE_ARRAY(tBestMatch.Element[i].Element);
	}

	RVL_DELETE_ARRAY(e.Element);

	RVL_DELETE_ARRAY(tBestMatch.Element);

	RVL_DELETE_ARRAY(score.Element);

	RVL_DELETE_ARRAY(pCTImatchesArray.Element);

	//delete scoreMatchMatrix	
	for (iSSegment = 0; iSSegment < scoreMatchMatrix.n; iSSegment++)
	{
		RVL_DELETE_ARRAY(scoreMatchMatrix.Element[iSSegment].Element);
	}

	for (iSSegment = 0; iSSegment < scoreMatchMatrixICP.n; iSSegment++)
	{
		RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element[iSSegment].Element);
	}

	RVL_DELETE_ARRAY(scoreMatchMatrix.Element);

	RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element);

	if (icpTMatrix)
		delete[] icpTMatrix;

	//fclose(fpTime);
	//End Vidovic
}

void PSGM::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("Recognition.mode", RVLPARAM_TYPE_ID, &mode);
	ParamList.AddID(pParamData, "TRAINING", RVLRECOGNITION_MODE_TRAINING);
	ParamList.AddID(pParamData, "RECOGNITION", RVLRECOGNITION_MODE_RECOGNITION); //Vidovic
	ParamList.AddID(pParamData, "CREATE_CTIS", RVLRECOGNITION_MODE_PSGM_CREATE_CTIS);
	pParamData = ParamList.AddParam("PSGM.nDominantClusters", RVLPARAM_TYPE_INT, &nDominantClusters);
	pParamData = ParamList.AddParam("PSGM.kNoise", RVLPARAM_TYPE_FLOAT, &kNoise);
	pParamData = ParamList.AddParam("PSGM.minInitialSurfelSize", RVLPARAM_TYPE_INT, &minInitialSurfelSize);
	pParamData = ParamList.AddParam("PSGM.minVertexPerc", RVLPARAM_TYPE_INT, &minVertexPerc);
	pParamData = ParamList.AddParam("PSGM.kReferenceSurfelSize", RVLPARAM_TYPE_FLOAT, &kReferenceSurfelSize);
	pParamData = ParamList.AddParam("PSGM.kReferenceTangentSize", RVLPARAM_TYPE_FLOAT, &kReferenceTangentSize);
	pParamData = ParamList.AddParam("PSGM.baseSeparationAngle", RVLPARAM_TYPE_FLOAT, &baseSeparationAngle);
	//pParamData = ParamList.AddParam("PSGM.edgeTangentAngle", RVLPARAM_TYPE_FLOAT, &edgeTangentAngle);
	pParamData = ParamList.AddParam("ModelDataBase", RVLPARAM_TYPE_STRING, &modelDataBase); //Vidovic
	pParamData = ParamList.AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, &modelsInDataBase); //Vidovic
	pParamData = ParamList.AddParam("PSGM.Match.RANSAC", RVLPARAM_TYPE_BOOL, &bMatchRANSAC); //Vidovic
	//pParamData = ParamList.AddParam("PSGM.RANSAC.nSamples", RVLPARAM_TYPE_INT, &nSamples); //Vidovic
	pParamData = ParamList.AddParam("PSGM.RANSAC.stdNoise", RVLPARAM_TYPE_INT, &stdNoise); //Vidovic
	pParamData = ParamList.AddParam("PSGM.normalValidityTest", RVLPARAM_TYPE_BOOL, &bNormalValidityTest); //Vidovic
	pParamData = ParamList.AddParam("PSGM.SceneMIMatch", RVLPARAM_TYPE_STRING, &sceneMIMatch); //Vidovic
	pParamData = ParamList.AddParam("PSGM.nModels", RVLPARAM_TYPE_INT, &nModels); //Vidovic
	pParamData = ParamList.AddParam("PSGM.nMSegments", RVLPARAM_TYPE_INT, &nMSegments); //Vidovic
	pParamData = ParamList.AddParam("PSGM.minClusterSize", RVLPARAM_TYPE_INT, &minClusterSize);
	pParamData = ParamList.AddParam("PSGM.maxClusterSize", RVLPARAM_TYPE_INT, &maxClusterSize);
	pParamData = ParamList.AddParam("PSGM.minSignificantClusterSize", RVLPARAM_TYPE_INT, &minSignificantClusterSize);
	pParamData = ParamList.AddParam("PSGM.minClusterBoundaryDiscontinuityPerc", RVLPARAM_TYPE_INT, &minClusterBoundaryDiscontinuityPerc);
	pParamData = ParamList.AddParam("PSGM.minClusterNormalDistributionStd", RVLPARAM_TYPE_FLOAT, &minClusterNormalDistributionStd);
	pParamData = ParamList.AddParam("PSGM.groundPlaneTolerance", RVLPARAM_TYPE_FLOAT, &groundPlaneTolerance);
	pParamData = ParamList.AddParam("PSGM.zeroRFDescriptor", RVLPARAM_TYPE_BOOL, &bZeroRFDescriptor);	
	pParamData = ParamList.AddParam("PSGM.GTRFDescriptors", RVLPARAM_TYPE_BOOL, &bGTRFDescriptors);
	pParamData = ParamList.AddParam("PSGM.Visualization.hypothesisVisualizationMode", RVLPARAM_TYPE_ID, &(displayData.hypothesisVisualizationMode));
	ParamList.AddID(pParamData, "CTI", RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI);
	ParamList.AddID(pParamData, "PLY", RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY);
	pParamData = ParamList.AddParam("PSGM.symmetryMatchThr", RVLPARAM_TYPE_FLOAT, &symmetryMatchThr);
	pParamData = ParamList.AddParam("PSGM.debug1", RVLPARAM_TYPE_INT, &debug1);
	pParamData = ParamList.AddParam("PSGM.debug2", RVLPARAM_TYPE_INT, &debug2);
}

void PSGM::Init(Mesh *pMesh_)
{
	pMesh = pMesh_;

	bGnd = false;
}

void PSGM::Interpret(
	Mesh *pMeshIn,
	int iScene)
{
	// Create ordered mesh.
	pMesh = pMeshIn;

	pMesh->CreateOrderedMeshFromPolyData();

	// Detect surfels.

	pSurfels->Init(pMesh);

	pSurfelDetector->Init(pMesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(pMesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	// Detect vertices.

	printf("Detect vertices.\n");

	pSurfels->DetectVertices(pMesh);

	// Cluster surfels into convex surfaces.

	printf("Detect convex clusters.\n");
	
	Clusters();

	// Fit model.

	printf("Fit convex template.\n");

	int nClusters = RVLMIN(clusters.n, nDominantClusters);

	char *GTHFileName = NULL;
	FILE *fpGTH = NULL;

	if (bGTRFDescriptors)
	{
		char *GTHFileName = RVLCreateString(sceneFileName);

		sprintf(GTHFileName + strlen(GTHFileName) - 3, "gth");

		fpGTH = fopen(GTHFileName, "w");
	}

	int iCluster;
	RECOG::PSGM_::Cluster *pCluster;
	RECOG::PSGM_::ModelInstance *pModelInstance;
	float R[9];

	//Init CTISet Qlist
	CTISet.Init();//Vidovic

	for (iCluster = 0; iCluster < nClusters; iCluster++)
	{
		pCluster = clusters.Element[iCluster];

		if (bZeroRFDescriptor)
		{
			//QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList); //Vidovic

			//RVLQLIST_INIT(pModelInstanceList); //Vidovic

			//AddReferenceFrame(iCluster); //Vidovic

			pModelInstance = AddReferenceFrame();

			pModelInstance->iCluster = iCluster;
		}
		else if (bGTRFDescriptors)
		{
			//QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList); //Vidovic

			//RVLQLIST_INIT(pModelInstanceList); //Vidovic

			Array<GTInstance> *pGT = pECCVGT->GT.Element + iScene;

			int iGTInstance;
			GTInstance *pGTInstance;

			for (iGTInstance = 0; iGTInstance < pGT->n; iGTInstance++)
			{
				pGTInstance = pGT->Element + iGTInstance;

				RVLSCALEMX3X3(pGTInstance->R, 1000.0f, R);
	
				//AddReferenceFrame(iCluster, R, pGTInstance->t); //Vidovic

				pModelInstance = AddReferenceFrame(R, pGTInstance->t); //Vidovic

				pModelInstance->iCluster = iCluster;

				fprintf(fpGTH, "%d\t%d\n", iCluster, pGTInstance->iModel);
			}
		}
		else
			ReferenceFrames(iCluster);		
	}

	//Vidovic
	pModelInstance = CTISet.CTI.pFirst;

	while (pModelInstance)
	{
		pCluster = clusters.Element[pModelInstance->iCluster];

		FitModel(pCluster->iVertexArray, pModelInstance);

		pModelInstance = pModelInstance->pNext;
	}

	//Copy CTIs from Qlist to Array
	CTISet.CopyCTIsToArray();
	//END Vidovic

	// Save model instances to a file.

	printf("Save model instances to a file.\n");

	char *PSGModelInstanceFileName = RVLCreateString(sceneFileName);

	sprintf(PSGModelInstanceFileName + strlen(PSGModelInstanceFileName) - 3, "cti");

	FILE *fp = fopen(PSGModelInstanceFileName, "w");

	SaveModelInstances(fp); //Vidovic

	fclose(fp);

	delete[] PSGModelInstanceFileName;

	//Vidovic
	//Match scene MI to model MI
	if (mode == RVLRECOGNITION_MODE_RECOGNITION)
		Match();

	if (bGTRFDescriptors)
	{
		if (fpGTH)
			fclose(fpGTH);

		RVL_DELETE_ARRAY(GTHFileName);
	}
}

//PETRA

//void PSGM::InterpreteCTIS(Mesh *pMesh)
//{
//
//	//Load matrix of primitives M
//	Eigen::Matrix<float, 9, 66> Mt;
//	Eigen::Matrix<float, 66, 9> M;
//
//	char *buffer = new char[594 * 8];
//	ifstream datafile("D:\\ARP3D\\Matlab_new\\M.bin", ios::in | ios::binary);
//	datafile.read(buffer, 594 * 8);
//	double *a = (double*)buffer;
//	for (int i = 0; i < 9; i++)
//	{
//		for (int j = 0; j < 66; j++)
//		{
//			Mt(i, j) = a[i * 66 + j];
//		}
//	}
//	M = Mt.transpose();
//	delete[] buffer;
//
//	//Load matrix QM
//	Eigen::MatrixXf QMt(4056, 9);
//	Eigen::MatrixXf QM(9, 4056);
//
//	buffer = new char[36504 * 8];
//	ifstream datafile2("D:\\ARP3D\\Matlab_new\\Q.bin", ios::in | ios::binary);
//	datafile2.read(buffer, 36504 * 8);
//	a = (double*)buffer;
//	for (int i = 0; i < 4056; i++)
//	{
//		for (int j = 0; j < 9; j++)
//		{
//			QMt(i, j) = a[i * 9 + j];
//		}
//	}
//	QM = QMt.transpose();
//	delete[] buffer;
//
//
//	//Load ModelDB CTIs
//	MCTIset.LoadSMCTI("D:\\ARP3D\\modelDB.dat", &convexTemplate); //needs to be in cfg file
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	pMCTI = MCTIset.CTIArr.Element;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//
//	//Number of model segments and number of models
//	int nSM = 3; //segments per model
//	int nSM_total = MCTIset.SegmentCTIs.n;
//	int nM = 35; //nSM_total / nSM
//
//	//Create matrix of model descriptors dM
//	Eigen::MatrixXf dM(66, MCTIset.SegmentCTIs.n);
//	for (int i = 0; i < MCTIset.SegmentCTIs.n; i++)
//	{
//		pMIE = pMCTI->modelInstance.Element;
//		for (int j = 0; j < 66; j++)
//		{
//			dM.block<1, 1>(j, i) << pMIE->d;
//			pMIE++;
//		}
//		pMCTI++;
//	}
//
//	//Load Scene CTIs
//	CTIset.LoadSMCTI("D:\\ARP3D\\ECCV_dataset\\CTIs4\\frame_20111220T111153.549117.cti", &convexTemplate);
//	RECOG::PSGM_::ModelInstance *pCTI;
//	pCTI = CTIset.CTIArr.Element;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//
//
//	// Number of scene segments
//	int nSS = CTIset.SegmentCTIs.n;
//
//	// Matching loop:
//
//	int iMIE, iValid, rows;
//
//	//Translation vector (for visualisation purposes)
//	Eigen::MatrixXf t;
//
//	// Set all errors elements to -1 for future handling
//	for (int i = 0; i < nSS*nM*nSM; i++)
//	{
//		SMatch[i].Eseg = -1;
//	}
//
//	// Loop trough each Scene CTI 
//	// CTI-s are sorted in segments
//	Array<SortIndex<float>> iSortedSegmentCTI;
//	iSortedSegmentCTI.n = nM * nSM;
//	SortIndex<float> *sortedMatches_;
//
//	int iMatch = 0, brojac, iCTI = 0;
//
//	pMCTI = MCTIset.CTIArr.Element;
//	for (int iSS = 0; iSS < nSS; iSS++)
//	{
//		brojac = CTIset.SegmentCTIs.Element[iSS].n; //nCTI(iSS);
//		while (brojac != 0)
//		{
//			//MatchInPrimitiveSpace(QM, M, iCTI);
//			CTIMatch(dM, iCTI);
//			UpdateMatchMatrix(SMatch, iCTI);
//
//			iCTI++;
//			pCTI++;
//			brojac--;
//		}
//
//		// Sort CTIs in each Segment by cost e
//		int iValidMatches = 0;  // for valid matches
//		sortedMatches_ = sortedMatches + iSS * iSortedSegmentCTI.n;
//
//		for (int j = 0; j < iSortedSegmentCTI.n; j++, iMatch++)
//		{
//			if (SMatch[iMatch].Eseg != -1)
//			{
//				iValidMatches++;
//				sortedMatches_[j].idx = iMatch;
//				sortedMatches_[j].cost = SMatch[iMatch].Eseg;
//			}
//			else
//			{
//				sortedMatches_[j].cost = 1000000; //to be in the end of sorted list
//			}
//		}
//
//		iSortedSegmentCTI.Element = sortedMatches_;
//
//
//		BubbleSort<SortIndex<float>>(iSortedSegmentCTI);
//	}
//
//#ifdef Visualization
//	// Visualization loop:
//	printf("\n\n-------Visualization-------\n\n");
//	int scale = 1000;
//	int iSS_v;
//	int iMatch_v;
//	char k;
//	Eigen::VectorXf dS_v(66);
//	Eigen::VectorXi validS_v(66);
//	bool exit = false;
//
//	while (exit == false)
//	{
//		printf("\nPress q and enter to exit.\nPress c and enter to continue.\n\n");
//
//		scanf(" %c", &k);
//		if (k == 'q')//(GetAsyncKeyState(VK_ESCAPE))
//		{
//			exit = true;
//			break;
//		}
//		else if (k = 'c')
//		{
//			do
//			{
//				printf("iSS: \n");
//				scanf("%d", &iSS_v);
//
//				printf("iMatch_v: \n");
//				scanf("%d", &iMatch_v);
//			} while (iSS_v > nSS /*|| iMatch_v>iSortedSegmentCTI.n*/);
//
//			int idxCTI = sortedMatches[nM*nSM*iSS_v + iMatch_v].idx;
//
//			pCTI = CTIset.CTI.Element + SMatch[idxCTI].iCTIs;
//			pSIE = pCTI->modelInstance.Element;
//			for (iMIE = 0; iMIE < 66; iMIE++)
//			{
//				dS_v(iMIE) = scale * pSIE->d;
//				validS_v(iMIE) = pSIE->valid;
//				pSIE++;
//			}
//			Eigen::VectorXf dM(66);
//			Eigen::MatrixXf dMt(66, 1);
//
//			pMCTI = MCTIset.CTI.Element + SMatch[idxCTI].iCTIm;
//			pMIE = pMCTI->modelInstance.Element;
//			for (int k = 0; k < 66; k++)
//			{
//				dM(k) = pMIE->d;
//				pMIE++;
//			}
//			nT = ConvexTemplatenT();
//			VisualizeCTIMatch(nT.data(), dM.data(), SMatch[idxCTI].t.data(), dS_v.data(), validS_v.data());
//		}
//	}
//
//#endif
//
//}
//void PSGM::CTIMatch(
//	Eigen::MatrixXf dM,
//	int iCTI)
//{
//	Eigen::MatrixXf M(66, 3), dMv(66, dM.cols());
//	Eigen::VectorXi validS(66);
//	Eigen::VectorXf dS, dSv;
//	int iValid = 0;
//
//	RECOG::PSGM_::ModelInstance *pCTI;
//	pCTI = CTIset.CTIArr.Element + iCTI;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//	pSIE = pCTI->modelInstance.Element;
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//
//	int iSS = pCTI->iCluster;
//	int row = 0;
//	for (int iMIE = 0; iMIE < 66; iMIE++)
//	{
//		validS(iMIE) = pSIE->valid;
//		dS(iMIE) = pSIE->d;
//		if (validS(iMIE) == 1)
//		{
//			M.block<1, 3>(row, 0) << nT.block<3, 1>(0, iMIE);
//			dSv(iValid) = dS(iMIE);
//			dMv.block<1, 3>(row, 0) = dM.block<1, 3>(iMIE, 0);
//			row++;
//		}
//
//		pSIE++;
//	}
//
//	// QR decomposition of M
//	Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qr(M);
//	Eigen::MatrixXf Rt_ = qr.matrixQR().triangularView<Eigen::Upper>();
//	Eigen::MatrixXf Qt_ = qr.matrixQ();
//	Eigen::MatrixXf Pt = qr.colsPermutation();
//	Eigen::MatrixXf Rt__;
//
//	for (int x = 0; x < 3; x++)
//	{
//		for (int y = 0; y < 3; y++)
//		{
//			if (Pt(y, x) == 1)
//			{
//				Rt__.block<3, 1>(0, y) = Rt_.block<3, 1>(0, x);
//			}
//		}
//	}
//
//	Eigen::MatrixXf Rt, Qt, ddv;
//	Eigen::MatrixXf Rtt = Rt__.transpose();
//
//	if (Rt__.block<1, 3>(2, 0)*Rtt.block<3, 1>(0, 2) < 1e-20)
//	{
//		Rt = Rt__.block<2, 3>(0, 0);
//		Qt = Qt_.block<9, 2>(0, 0);
//	}
//	else
//	{
//		Qt = Qt_.block<9, 3>(0, 0);;
//		Rt = Rt__;
//	}
//
//	int nM = dM.cols();
//	Eigen::MatrixXf ones(1, nM);
//	Eigen::MatrixXf t_;
//	for (int i = 0; i < nM; i++)
//	{
//		ones(0, i) = 1;
//	}
//
//
//	ddv = dSv*ones - dMv;
//
//	t_ = Qt.transpose()*ddv;
//	E = ddv - Qt*t_;
//
//}
//
//void PSGM::MatchInPrimitiveSpace(
//	Eigen::MatrixXf QM,
//	Eigen::MatrixXf M,
//	int iCTI
//	)
//{
//	//Parameters
//	float scale = 1000.0;
//	int nM = 35; //in future this needs to be loaded from modelDB
//	int nSM = 3; //in future this needs to be loaded from modelDB
//
//
//	RECOG::PSGM_::ModelInstance *pCTI;
//	pCTI = CTIset.CTIArr.Element + iCTI;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//	pSIE = pCTI->modelInstance.Element;
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//
//	int iSS = pCTI->iCluster;
//
//	// Visibility mask
//	Eigen::VectorXi validS(66);
//
//	// Desciptor
//	Eigen::VectorXf dS(66);
//
//	// Determine the number of rows (rows=iValid) of Mv
//	int rows = 0, iMIE;
//	for (iMIE = 0; iMIE < 66; iMIE++)
//	{
//		validS(iMIE) = pSIE->valid; // Filling visibility mask
//		dS(iMIE) = pSIE->d; // Filling descriptor
//		if (validS(iMIE) == 1)
//			rows++;
//		pSIE++;
//	}
//
//	// Search for valids and create dv and Mv
//	Eigen::MatrixXf Mv(rows, 9);
//	Eigen::MatrixXf dv(rows, 1);
//	Eigen::MatrixXf D(CTI.n, rows); //matrix of descriptors
//	int iValid = 0;
//	for (iMIE = 0; iMIE < 66; iMIE++)
//	{
//		if (validS(iMIE) == 1)
//		{
//			dv(iValid) = dS(iMIE);
//			Mv.block<1, 9>(iValid, 0) << M.block<1, 9>(iMIE, 0);
//			iValid++;
//		}
//		pSIE++;
//	}
//
//	// QR decomposition of M
//	Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qr(Mv);
//	Eigen::MatrixXf R_ = qr.matrixQR().triangularView<Eigen::Upper>();
//	Eigen::MatrixXf Q_ = qr.matrixQ();
//	Eigen::MatrixXf P = qr.colsPermutation();
//	Eigen::MatrixXf Q(iValid, 9), R(9, 9), R_sorted(9, 9);
//
//	for (int x = 0; x < iValid; x++)
//	{
//		for (int y = 0; y < 9; y++)
//		{
//			Q(x, y) = Q_(x, y);
//		}
//	}
//
//	for (int x = 0; x < 9; x++)
//	{
//		for (int y = 0; y < 9; y++)
//		{
//			R(x, y) = R_(x, y);
//		}
//	}
//
//	int m = M.cols();
//	int ms = m - 3;
//	Eigen::MatrixXf q = Q.transpose() * dv;
//	Eigen::MatrixXf Rs(9, ms);
//	Eigen::MatrixXf Rt(9, (m - ms));
//
//	for (int x = 0; x < 9; x++)
//	{
//		for (int y = 0; y < 9; y++)
//		{
//			if (P(y, x) == 1)
//			{
//				R_sorted.block<9, 1>(0, y) = R.block<9, 1>(0, x);
//			}
//		}
//	}
//
//	for (int x = 0; x < 9; x++)
//	{
//		for (int y = 0; y < ms; y++)
//		{
//			Rs(x, y) = R_sorted(x, y);
//		}
//	}
//
//	for (int x = 0; x < 9; x++)
//	{
//		for (int y = 0; y < m - ms; y++)
//		{
//			Rt(x, y) = R_sorted(x, y + ms);
//		}
//	}
//
//	// QR decomposition of Rt
//	Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qrRt(Rt);
//	Eigen::MatrixXf Pt = qrRt.colsPermutation();
//	Eigen::MatrixXf Rt_(3, 3);
//	Eigen::MatrixXf Rt_t(3, 3);
//	Eigen::MatrixXf RRt = qrRt.matrixQR().triangularView<Eigen::Upper>();
//	Eigen::MatrixXf Qt_ = qrRt.matrixQ();
//
//	for (int x = 0; x < 3; x++)
//	{
//		for (int y = 0; y < 3; y++)
//		{
//			if (Pt(y, x) == 1)
//			{
//				Rt_.block<3, 1>(0, y) = RRt.block<3, 1>(0, x);
//			}
//		}
//	}
//	Eigen::MatrixXf Qt = Qt_;
//	Eigen::MatrixXf RT = Rt_;
//	Rt_t = Rt_.transpose();
//
//	if (Rt_.block<1, 3>(2, 0)*Rt_t.block<3, 1>(0, 2) < 1e-20)
//	{
//		RT = Rt_.block<2, 3>(0, 0);
//		Qt = Qt_.block<9, 2>(0, 0);
//	}
//	else
//	{
//		Qt = Qt_.block<9, 3>(0, 0);;
//		RT = Rt_;
//	}
//
//
//	// Match CTI descriptor to model	
//	int Mn = QM.cols();
//	Eigen::MatrixXf s(ms, Mn);
//
//	for (int i = 0; i < ms; i++)
//	{
//		for (int j = 0; j < Mn; j++)
//		{
//			s(i, j) = QM(i, j);
//		}
//	}
//
//	Eigen::MatrixXf jed(1, Mn);
//	for (int i = 0; i < Mn; i++)
//	{
//		jed(0, i) = 1;
//	}
//
//	Eigen::MatrixXf es = q * jed - Rs*s;
//	Eigen::MatrixXf t_ = Qt.transpose() * es;
//	Eigen::MatrixXf e = es - Qt * t_;
//	E.resize(e.cols());
//
//	float *pt_ = t_.data();
//
//	t = RT.inverse()*t_;
//
//	// Calculate E
//	float sum;
//	int iM, iSM;
//	int var;
//
//	// Find E between each Scene segment and each Model segment
//	for (int j = 0; j < e.cols(); j++)
//	{
//		sum = 0;
//		for (int i = 0; i < e.rows(); i++)
//		{
//			sum += e(i, j)*e(i, j);
//		}
//		E(j) = sqrt(sum); // Least square
//	}
//
//}
//
//void PSGM::UpdateMatchMatrix(
//	RECOG::PSGM_::SegmentMatch *SMatch,
//	int iCTI
//	)
//{
//
//	RECOG::PSGM_::ModelInstance *pCTI;
//	pCTI = CTIset.CTIArr.Element + iCTI;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//	pSIE = pCTI->modelInstance.Element;
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//
//	int nM = 35; //from DB
//	int nSM = 3; //from DB
//	int iSM, iM, var, j;
//	int iSS = pCTI->iCluster;
//
//	for (j = 0; j < E.rows(); j++)
//	{
//		// Search for matching model parameters
//		pMCTI = MCTIset.CTIArr.Element + j;
//		pMIE = pMCTI->modelInstance.Element;
//		iM = pMCTI->iModel;
//		iSM = pMCTI->iCluster;
//		var = nM*nSM*iSS + nSM*iM + iSM;
//		if (SMatch[var].Eseg == -1 || E(j) < SMatch[var].Eseg)
//		{
//			SMatch[var].Eseg = E(j);
//			SMatch[var].iCTIm = j;
//			SMatch[var].iCTIs = iCTI;
//			SMatch[var].iSM = iSM;
//			SMatch[var].iSS = iSS;
//			SMatch[var].iM = iM;
//			SMatch[var].t = t.block<3, 1>(0, j);
//		}
//	}
//}
//
//
Eigen::MatrixXf PSGM::ConvexTemplatenT()
{
	Eigen::Matrix<float, 3, 13> nT;
	Eigen::Matrix<float, 3, 11> nT_;
	Eigen::Matrix<float, 3, 66> nT__;
	Eigen::Matrix<int, 8, 3> temp1;
	Eigen::Matrix<float, 3, 1> N;
	Eigen::Matrix<float, 1, 3> Nt;
	Eigen::Matrix<float, 1, 1> NtN;
	Eigen::Matrix<float, 3, 3> R;
	Eigen::Matrix<int, 1, 66> dT;

	float h, q, sh, ch, sq, cq;
	float pi = 3.1415;
	h = pi / 4;
	q = h / 2;
	sh = sin(h);
	ch = cos(h);
	sq = sin(q);
	cq = cos(q);

	//for (int i = 0; i < 3; i++)
	//{
	//	for (int j = 0; j < 13; j++)
	//	{
	//		nT[i][j] = 0;
	//	}
	//}
	//memset(nT->data(), 0, 13 * 3 * sizeof(float));
	nT.Zero();
	nT(0, 0) = 0;
	nT(1, 0) = 0;
	nT(2, 0) = 1;
	nT(0, 1) = 0;
	nT(1, 1) = -ch;
	nT(2, 1) = ch;
	nT(0, 2) = ch;
	nT(1, 2) = 0;
	nT(2, 2) = ch;
	nT(0, 11) = 0;
	nT(1, 11) = ch;
	nT(2, 11) = ch;
	nT(0, 12) = -ch;
	nT(1, 12) = 0;
	nT(2, 12) = ch;

	temp1 << 3, 0, 1, 4, 0, 2, 5, 1, 2, 6, 0, 11, 7, 0, 12, 8, 2, 11, 9, 1, 12, 10, 11, 12;

	for (int i = 0; i < temp1.rows(); i++)
	{
		int column1, column2, column3;
		column1 = temp1(i, 1);
		column2 = temp1(i, 2);
		column3 = temp1(i, 0);
		N << nT(0, column1) + nT(0, column2), nT(1, column1) + nT(1, column2), nT(2, column1) + nT(2, column2);
		Nt = N.transpose();
		NtN = Nt*N;
		nT.block<3, 1>(0, column3) << (N / (sqrt(NtN(0, 0)))); // there must be a better way to convert to float
	}

	R << 0, 0, -1, 1, 0, 0, 0, -1, 0;
	nT_ = nT.block < 3, 11 >(0, 0);
	nT__.block<3, 11>(0, 0) << nT_;
	nT__.block<3, 11>(0, 11) << R*nT_;
	nT__.block<3, 11>(0, 22) << R*R*nT_;
	nT__.block<3, 11>(0, 33) << -1 * nT_;
	nT__.block<3, 11>(0, 44) << -1 * R*nT_;
	nT__.block<3, 11>(0, 55) << -1 * R*R*nT_;

	int nF = nT__.cols();
	dT.Ones();
	return nT__;
}





//Generates vtkPolyData object (points and polys) that represenent a single CTI primitive, planeNormals is column wise (all_normals_x_coordinates, all_normals_y_coordinates, all_normals_z_coordinates)
vtkSmartPointer<vtkPolyData> GenerateCTIPrimitivePolydata_CW(float *planeNormals, float *planeDist, bool centered = false, int *mask = NULL)
{
	vtkSmartPointer<vtkPolyData> outPD;

	float *planeDistLocal = planeDist;
	//center the model
	if (!centered)
	{
		//make copy of original plane dist
		planeDistLocal = new float[66];
		memcpy(planeDistLocal, planeDist, 66 * sizeof(float));

		//Finding MIN and MAX for each normal dimension
		float maxN[3] = { -10, -10, -10 };
		int maxI[3] = { 0, 0, 0 };
		float minN[3] = { 10, 10, 10 };
		int minI[3] = { 0, 0, 0 };
		for (int i = 0; i < 66; i++)
		{
			if (planeNormals[i] > maxN[0])
			{
				maxN[0] = planeNormals[i];
				maxI[0] = i;
			}
			if (planeNormals[i] < minN[0])
			{
				minN[0] = planeNormals[i];
				minI[0] = i;
			}

			if (planeNormals[i + 66] > maxN[1])
			{
				maxN[1] = planeNormals[i + 66];
				maxI[1] = i;
			}
			if (planeNormals[i + 66] < minN[1])
			{
				minN[1] = planeNormals[i + 66];
				minI[1] = i;
			}

			if (planeNormals[i + 66 * 2] > maxN[2])
			{
				maxN[2] = planeNormals[i + 66 * 2];
				maxI[2] = i;
			}
			if (planeNormals[i + 66 * 2] < minN[2])
			{
				minN[2] = planeNormals[i + 66 * 2];
				minI[2] = i;
			}
		}
		//centering
		float newexampleTemp[66];
		float tempV[3];
		tempV[0] = 0.5 * (planeDistLocal[maxI[0]] - planeDistLocal[minI[0]]);
		tempV[1] = 0.5 * (planeDistLocal[maxI[1]] - planeDistLocal[minI[1]]);
		tempV[2] = 0.5 * (planeDistLocal[maxI[2]] - planeDistLocal[minI[2]]);
		for (int i = 0; i < 66; i++)
		{
			newexampleTemp[i] = planeNormals[i] * tempV[0] + planeNormals[i + 66] * tempV[1] + planeNormals[i + 66 * 2] * tempV[2];
			planeDistLocal[i] -= newexampleTemp[i];
		}
	}

	//Generiate primitive (convex hull)
	vtkSmartPointer<vtkHull> hullFilter = vtkSmartPointer<vtkHull>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> normalp = vtkSmartPointer<vtkFloatArray>::New();
	normalp->SetNumberOfComponents(3);
	for (int i = 0; i < 66; i++)
	{
		points->InsertPoint(i, planeNormals[i] * planeDistLocal[i], planeNormals[i + 66] * planeDistLocal[i], planeNormals[i + 66 * 2] * planeDistLocal[i]);
		normalp->InsertTuple3(i, planeNormals[i], planeNormals[i + 66], planeNormals[i + 66 * 2]);
	}
	vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New();
	planes->SetPoints(points);
	planes->SetNormals(normalp);
	hullFilter->SetPlanes(planes);
	vtkSmartPointer<vtkPolyData> hullPD = vtkSmartPointer<vtkPolyData>::New();
	hullFilter->GenerateHull(hullPD, -500, 500, -500, 500, -500, 500);
	vtkSmartPointer<vtkPolyData> interPD = hullPD;
	//If mask exists remove unwanted polygons
	if (mask)
	{
		double n[3];
		float cosfi;
		vtkSmartPointer<vtkCellArray> polys = hullPD->GetPolys();
		vtkSmartPointer<vtkCellArray> newpolys = vtkSmartPointer<vtkCellArray>::New();
		vtkIdType *polysPtsIds;
		vtkIdType npts;
		polys->InitTraversal();
		//run through all polygons and find planes with the same normal that shuld be in the output
		for (int i = 0; i < hullPD->GetNumberOfPolys(); i++)
		{
			polys->GetNextCell(npts, polysPtsIds);
			//calculate polygon normal
			vtkPolygon::ComputeNormal(hullPD->GetPoints(), npts, polysPtsIds, n);
			//find corresponding normal in normal list
			for (int k = 0; k < 66; k++)
			{
				cosfi = n[0] * planeNormals[k] + n[1] * planeNormals[k + 66] + n[2] * planeNormals[k + 66 * 2];
				if ((cosfi > 0.9999) && (mask[k] == 1))
				{
					newpolys->InsertNextCell(npts, polysPtsIds);
					break;
				}
			}

		}
		vtkSmartPointer<vtkPolyData> maskedPD = vtkSmartPointer<vtkPolyData>::New();
		maskedPD->SetPoints(hullPD->GetPoints());
		maskedPD->SetPolys(newpolys);

		interPD = maskedPD;
	}

	//clean polydata from unused poimts and degenerate polygons
	vtkSmartPointer<vtkCleanPolyData> cleanPD = vtkSmartPointer<vtkCleanPolyData>::New();
	cleanPD->SetInputData(interPD);
	cleanPD->Update();

	//make copy of the final polydata and send it back
	outPD = vtkSmartPointer<vtkPolyData>::New();
	outPD->DeepCopy(cleanPD->GetOutput());
	return outPD;
}

//Generates vtkPolyData object (points and polys) that represenent a single CTI primitive, planeNormals is row wise (normal_1_x_coordinate, normal_1_y_coordinate, normal_1_z_coordinate, normal_2_x_coordinate, ...)
vtkSmartPointer<vtkPolyData> GenerateCTIPrimitivePolydata_RW(float *planeNormals, float *planeDist, int nPlanes, bool centered = false, int *mask = NULL, float *t = NULL)
{
	vtkSmartPointer<vtkPolyData> outPD;

	float *planeDistLocal = planeDist;
	//center the model
	if (!centered)
	{
		//make copy of original plane dist
		planeDistLocal = new float[nPlanes];
		memcpy(planeDistLocal, planeDist, nPlanes * sizeof(float));

		//Finding MIN and MAX for each normal dimension
		float maxN[3] = { -10, -10, -10 };
		int maxI[3] = { 0, 0, 0 };
		float minN[3] = { 10, 10, 10 };
		int minI[3] = { 0, 0, 0 };
		for (int i = 0; i < nPlanes; i++)
		{
			if (planeNormals[i * 3] > maxN[0])
			{
				maxN[0] = planeNormals[i * 3];
				maxI[0] = i;
			}
			if (planeNormals[i * 3] < minN[0])
			{
				minN[0] = planeNormals[i * 3];
				minI[0] = i;
			}

			if (planeNormals[i * 3 + 1] > maxN[1])
			{
				maxN[1] = planeNormals[i * 3 + 1];
				maxI[1] = i;
			}
			if (planeNormals[i * 3 + 1] < minN[1])
			{
				minN[1] = planeNormals[i * 3 + 1];
				minI[1] = i;
			}

			if (planeNormals[i * 3 + 2] > maxN[2])
			{
				maxN[2] = planeNormals[i * 3 + 2];
				maxI[2] = i;
			}
			if (planeNormals[i * 3 + 2] < minN[2])
			{
				minN[2] = planeNormals[i * 3 + 2];
				minI[2] = i;
			}
		}
		//centering
		float newexampleTemp;
		float tempV[3];
		tempV[0] = 0.5 * (planeDistLocal[maxI[0]] - planeDistLocal[minI[0]]);
		tempV[1] = 0.5 * (planeDistLocal[maxI[1]] - planeDistLocal[minI[1]]);
		tempV[2] = 0.5 * (planeDistLocal[maxI[2]] - planeDistLocal[minI[2]]);
		for (int i = 0; i < nPlanes; i++)
		{
			newexampleTemp = planeNormals[i * 3] * tempV[0] + planeNormals[i * 3 + 1] * tempV[1] + planeNormals[i * 3 + 2] * tempV[2];
			planeDistLocal[i] -= newexampleTemp;
		}
		if (t)
			memcpy(t, tempV, 3 * sizeof(float));
	}

	//Generiate primitive (convex hull)
	vtkSmartPointer<vtkHull> hullFilter = vtkSmartPointer<vtkHull>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> normalp = vtkSmartPointer<vtkFloatArray>::New();
	normalp->SetNumberOfComponents(3);
	for (int i = 0; i < nPlanes; i++)
	{
		points->InsertPoint(i, planeNormals[i * 3] * planeDistLocal[i], planeNormals[i * 3 + 1] * planeDistLocal[i], planeNormals[i * 3 + 2] * planeDistLocal[i]);
		normalp->InsertTuple3(i, planeNormals[i * 3], planeNormals[i * 3 + 1], planeNormals[i * 3 + 2]);
	}
	vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New();
	planes->SetPoints(points);
	planes->SetNormals(normalp);
	hullFilter->SetPlanes(planes);
	vtkSmartPointer<vtkPolyData> hullPD = vtkSmartPointer<vtkPolyData>::New();
	hullFilter->GenerateHull(hullPD, -500, 500, -500, 500, -500, 500);
	vtkSmartPointer<vtkPolyData> interPD = hullPD;
	//If mask exists remove unwanted polygons
	if (mask)
	{
		double n[3];
		float cosfi;
		vtkSmartPointer<vtkCellArray> polys = hullPD->GetPolys();
		vtkSmartPointer<vtkCellArray> newpolys = vtkSmartPointer<vtkCellArray>::New();
		vtkIdType *polysPtsIds;
		vtkIdType npts;
		polys->InitTraversal();
		//run through all polygons and find planes with the same normal that shuld be in the output
		for (int i = 0; i < hullPD->GetNumberOfPolys(); i++)
		{
			polys->GetNextCell(npts, polysPtsIds);
			//calculate polygon normal
			vtkPolygon::ComputeNormal(hullPD->GetPoints(), npts, polysPtsIds, n);
			//find corresponding normal in normal list
			for (int k = 0; k < nPlanes; k++)
			{
				cosfi = n[0] * planeNormals[k * 3] + n[1] * planeNormals[k * 3 * 1] + n[2] * planeNormals[k * 3 + 2];
				if ((cosfi > 0.9999) && (mask[k] == 1))
				{
					newpolys->InsertNextCell(npts, polysPtsIds);
					break;
				}
			}

		}
		vtkSmartPointer<vtkPolyData> maskedPD = vtkSmartPointer<vtkPolyData>::New();
		maskedPD->SetPoints(hullPD->GetPoints());
		maskedPD->SetPolys(newpolys);

		//intermediate
		interPD = maskedPD;
	}

	//clean polydata from unused poimts and degenerate polygons
	vtkSmartPointer<vtkCleanPolyData> cleanPD = vtkSmartPointer<vtkCleanPolyData>::New();
	cleanPD->SetInputData(interPD);
	cleanPD->Update();

	//make copy of the final polydata and send it back
	outPD = vtkSmartPointer<vtkPolyData>::New();
	outPD->DeepCopy(cleanPD->GetOutput());

	return outPD;
}

void PSGM::VisualizeCTIMatchidx(int iSCTI, int iMCTI)
{
	Eigen::MatrixXf nT = ConvexTemplatenT();

	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;

	float *dS = new float[66];
	float *dM = new float[66];
	int *validS = new int[66];

	//Eigen::VectorXf dS(66);
	//Eigen::VectorXf dM(66);
	//Eigen::VectorXi validS(66);


	pSCTI = CTISet.pCTI.Element[iSCTI];
	pSIE = pSCTI->modelInstance.Element;
	pMCTI = MCTISet.pCTI.Element[iMCTI];
	pMIE = pMCTI->modelInstance.Element;

	for (int i = 0; i < 66; i++)
	{
		validS[i] = pSIE->valid; // visibility mask
		dS[i] = pSIE->d; // *1000; // Scene descriptor 
		dM[i] = pMIE->d / 1000; // Model descriptor
		pSIE++;
		pMIE++;
	}

	VisualizeCTIMatch(nT.data(), dM, dS, validS);
	//VisualizeCTIMatch(nT.data(), dM.data(), dS.data(), validS.data());

	delete[] dS;
	delete[] dM;
	delete[] validS;

}

void PSGM::VisualizeCTIMatch(float *nT, float *dM, float *dS, int *validS)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//Generate model polydata
	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(nT, dM, 66);
	/*if (tM) //if translation exists
	{
		float scale = 1;
		vtkSmartPointer<vtkTransform> modelT = vtkSmartPointer<vtkTransform>::New();
		modelT->Translate(tM[0], tM[1], tM[2]);
		vtkSmartPointer<vtkTransformPolyDataFilter> modelTFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		modelTFilter->SetTransform(modelT);
		modelTFilter->SetInputData(modelPD);
		modelTFilter->Update();
		vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		modelMapper->SetInputConnection(modelTFilter->GetOutputPort());
		vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
		modelActor->SetMapper(modelMapper);
		modelActor->GetProperty()->SetColor(0, 1, 0);
		renderer->AddActor(modelActor);
	}
	else*/
	{
		vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		modelMapper->SetInputData(modelPD);
		vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
		modelActor->SetMapper(modelMapper);
		modelActor->GetProperty()->SetColor(0, 1, 0);
		renderer->AddActor(modelActor);
	}

	/*
	vtkSmartPointer<vtkPolyDataMapper> modelMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper2->SetInputData(modelPD);
	vtkSmartPointer<vtkActor> modelActor2 = vtkSmartPointer<vtkActor>::New();
	modelActor2->SetMapper(modelMapper2);
	modelActor2->GetProperty()->SetColor(1, 0, 0);
	renderer->AddActor(modelActor2);
	*/
	//Generate scene polydata
	vtkSmartPointer<vtkPolyData> modelSPD = GenerateCTIPrimitivePolydata_RW(nT, dS, 66, false, validS);
	vtkSmartPointer<vtkPolyDataMapper> modelSMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelSMapper->SetInputData(modelSPD);
	vtkSmartPointer<vtkActor> modelSActor = vtkSmartPointer<vtkActor>::New();
	modelSActor->SetMapper(modelSMapper);
	modelSActor->GetProperty()->SetColor(0, 0, 1);
	renderer->AddActor(modelSActor);



	//Start VTK
	renderer->ResetCamera();
	renderer->TwoSidedLightingOff();
	window->Render();
	interactor->Start();
}
//
////END PETRA
//
//

void PSGM::Clusters()
{
	RVL_DELETE_ARRAY(clusterMap);

	clusterMap = new int[pSurfels->NodeArray.n];

	memset(clusterMap, 0xff, pSurfels->NodeArray.n * sizeof(int));

	RVL_DELETE_ARRAY(clusterMem);

	clusterMem = new RECOG::PSGM_::Cluster[pSurfels->NodeArray.n];

	clusters.n = 0;

	RVL_DELETE_ARRAY(clusterSurfelMem);

	clusterSurfelMem = new int[pSurfels->NodeArray.n];

	int *piSurfel = clusterSurfelMem;

	RVL_DELETE_ARRAY(clusterVertexMem);

	clusterVertexMem = new int[pSurfels->nVertexSurfelRelations];

	int *piVertex = clusterVertexMem;

	bool *bVertexVisited = new bool[pSurfels->vertexArray.n];
	bool *bVertexInCluster = new bool[pSurfels->vertexArray.n];

	bool *bSurfelVisited = new bool[pSurfels->NodeArray.n];

	QList<QLIST::Index> candidateList;
	QList<QLIST::Index> *pCandidateList = &candidateList;

	QLIST::Index *candidateMem = new QLIST::Index[pSurfels->NodeArray.n];

	Array<int> surfelBuff1, surfelBuff2;

	surfelBuff1.Element = new int[pSurfels->NodeArray.n];

	surfelBuff1.n = 0;

	int i;
	Surfel *pSurfel;

	for (i = 0; i < pSurfels->NodeArray.n; i++)
	{
		pSurfel = pSurfels->NodeArray.Element + i;

		if (!pSurfel->bEdge)
			surfelBuff1.Element[surfelBuff1.n++] = i;
	}

	surfelBuff2.Element = new int[surfelBuff1.n];

	Array<int> *pSurfelBuff = &surfelBuff1;
	Array<int> *pSurfelBuff_ = &surfelBuff2;

	int nValidClusters = 0;

	Array<int> *pTmp;

#ifdef RVLPSGM_NORMAL_HULL
	Array<RECOG::PSGM_::NormalHullElement> NHull;

	NHull.Element = new RECOG::PSGM_::NormalHullElement[pSurfels->NodeArray.n];
#else
	float meanN[3];
	float sumN[3];
	float wN;
#endif

	RECOG::PSGM_::Cluster *pCluster;
	int iCluster;
	int maxSurfelSize;
	int iLargestSurfel;
	int iFirstNewVertex;
	QLIST::Index *pCandidateIdx, *pBestCandidateIdx;
	QLIST::Index **ppCandidateIdx, **ppBestCandidateIdx;
	float dist, minDist;
	int nSurfelVertices;
	int nSurfelVerticesInCluster;
	int *piVertex_, *piVertex__;
	int iSurfel, iSurfel_;
	Surfel *pSurfel_;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *pVertexIdx;

	for (iCluster = 0; iCluster < pSurfels->NodeArray.n; iCluster++)
	{
		// pSurfel <- the largest surfel which is not assigned to a cluster.

		maxSurfelSize = minInitialSurfelSize - 1;

		iLargestSurfel = -1;

		pSurfelBuff_->n = 0;

		for (i = 0; i < pSurfelBuff->n; i++)
		{
			iSurfel = pSurfelBuff->Element[i];

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			if (!pSurfel->bEdge)
			{
				if (clusterMap[iSurfel] < 0)
				{
					pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

					if (pSurfelVertexList->pFirst)
					{
						pSurfelBuff_->Element[pSurfelBuff_->n++] = iSurfel;

						if (pSurfel->size > maxSurfelSize)
						{
							maxSurfelSize = pSurfel->size;

							iLargestSurfel = iSurfel;
						}
					}
				}
			}
		}

		pTmp = pSurfelBuff;
		pSurfelBuff = pSurfelBuff_;
		pSurfelBuff_ = pTmp;

		if (iLargestSurfel < 0)
			break;

		//if (iLargestSurfel == 25)
		//	int debug = 0;

		//if (clusters.n == 15)
		//	int debug = 0;

		// Initialize a new cluster.

		pCluster = clusterMem + iCluster;

		pCluster->iSurfelArray.Element = piSurfel;
		pCluster->iVertexArray.Element = piVertex;

		pCluster->iSurfelArray.n = 0;
		pCluster->iVertexArray.n = 0;
		pCluster->size = 0;

		clusters.n++;

		clusterMap[iLargestSurfel] = iCluster;

		memset(bVertexVisited, 0, pSurfels->vertexArray.n * sizeof(bool));
		memset(bVertexInCluster, 0, pSurfels->vertexArray.n * sizeof(bool));
		memset(bSurfelVisited, 0, pSurfels->NodeArray.n * sizeof(bool));

		RVLQLIST_INIT(pCandidateList);

		QLIST::Index *pNewCandidate = candidateMem;

#ifdef RVLPSGM_NORMAL_HULL
		NHull.n = 0;
#else
		RVLNULL3VECTOR(sumN);
		wN = 0.0f;
#endif

		RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

		pNewCandidate->Idx = iLargestSurfel;

		pNewCandidate++;

		bSurfelVisited[iLargestSurfel] = true;

		// Region growing.

		while (pCandidateList->pFirst)
		{
			// iSurfel <- the best candidate for expanding cluster.

			minDist = PI;

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = *ppCandidateIdx;

			while (pCandidateIdx)
			{
				iSurfel_ = pCandidateIdx->Idx;

				pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

#ifdef RVLPSGM_NORMAL_HULL
				dist = DistanceFromNormalHull(NHull, pSurfel_->N);
#else
				float e = RVLDOTPRODUCT3(meanN, pSurfel_->N);
				dist = (wN < 1e-10 ? 0.0f : acos(e));
#endif

				if (dist < minDist)
				{
					minDist = dist;

					iSurfel = iSurfel_;

					pBestCandidateIdx = pCandidateIdx;

					ppBestCandidateIdx = ppCandidateIdx;
				}

				ppCandidateIdx = &(pCandidateIdx->pNext);

				pCandidateIdx = *ppCandidateIdx;
			}

			// Remove iSurfel from candidateList.

			RVLQLIST_REMOVE_ENTRY(pCandidateList, pBestCandidateIdx, ppBestCandidateIdx);

			//if (iSurfel == 8)
			//	int debug = 0;

			// Add vertices of iSurfel, which are inside convex (or outside concave) surface into cluster.

			iFirstNewVertex = pCluster->iVertexArray.n;

			piVertex_ = piVertex;

			pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

			nSurfelVertices = nSurfelVerticesInCluster = 0;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				if (bVertexVisited[pVertexIdx->Idx])
				{
					if (bVertexInCluster[pVertexIdx->Idx])
						nSurfelVerticesInCluster++;
				}
				else
				{
					if (Inside(pVertexIdx->Idx, pCluster, iSurfel))
					{
						*(piVertex++) = pVertexIdx->Idx;

						nSurfelVerticesInCluster++;
					}

				}

				nSurfelVertices++;

				pVertexIdx = pVertexIdx->pNext;
			}

			if (nSurfelVertices == 0)
				continue;

			if (100 * nSurfelVerticesInCluster / nSurfelVertices < minVertexPerc)
			{
				piVertex = piVertex_;

				continue;
			}

			pCluster->iVertexArray.n = piVertex - pCluster->iVertexArray.Element;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				bVertexVisited[pVertexIdx->Idx] = true;

				pVertexIdx = pVertexIdx->pNext;
			}

			for (piVertex__ = piVertex_; piVertex__ < piVertex; piVertex__++)
				bVertexInCluster[*piVertex__] = true;

			// Add iSurfel to cluster.

			//if (iLargestSurfel == 25 && iSurfel == 192)
			//	int debug = 0;

			clusterMap[iSurfel] = iCluster;

			*(piSurfel++) = iSurfel;

			pCluster->iSurfelArray.n++;

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			pCluster->size += pSurfel->size;

#ifdef RVLPSGM_NORMAL_HULL
			// Update normal hull.

			UpdateNormalHull(NHull, pSurfel->N);
#else
			// Update mean normal.

			UpdateMeanNormal(sumN, wN, pSurfel->N, (float)(pSurfel->size), meanN);
#endif

			// Remove candidates which are not consistent with new vertices added to the cluster.

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = candidateList.pFirst;

			while (pCandidateIdx)
			{
				pSurfel_ = pSurfels->NodeArray.Element + pCandidateIdx->Idx;

				//if (pCandidateIdx->Idx == 8)
				//	int debug = 0;

				if (BelowPlane(pCluster, pSurfel_, iFirstNewVertex))
					ppCandidateIdx = &(pCandidateIdx->pNext);
				else
					RVLQLIST_REMOVE_ENTRY(pCandidateList, pCandidateIdx, ppCandidateIdx)

					pCandidateIdx = pCandidateIdx->pNext;
			}

			// Add new candidates in candidateList.

			SURFEL::EdgePtr *pSurfelEdgePtr = pSurfel->EdgeList.pFirst;

			while (pSurfelEdgePtr)
			{
				iSurfel_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pSurfelEdgePtr);

				//if (iSurfel_ == 8)
				//	int debug = 0;

				if (clusterMap[iSurfel_] < 0)
				{
					if (!bSurfelVisited[iSurfel_])
					{
						//if (iSurfel_ == 8)
						//	int debug = 0;

						bSurfelVisited[iSurfel_] = true;

						pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

						if (pSurfel_->size > 1)
						{
							if (BelowPlane(pCluster, pSurfel_))
							{
								RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

								pNewCandidate->Idx = iSurfel_;

								pNewCandidate++;
							}
						}
					}
				}

				pSurfelEdgePtr = pSurfelEdgePtr->pNext;
			}
		}	// region growing loop

		if(pCluster->bValid = (pCluster->size >= minClusterSize))
			nValidClusters++;
	}	// for each cluster

	delete[] candidateMem;
	delete[] bVertexVisited;
	delete[] bVertexInCluster;
	delete[] bSurfelVisited;
#ifdef RVLPSGM_NORMAL_HULL
	delete[] NHull.Element;
#endif

	// Sort clusters.

	Array<SortIndex<int>> sortedClusterArray;

	sortedClusterArray.Element = new SortIndex<int>[nValidClusters];

	SortIndex<int> *pSortIndex = sortedClusterArray.Element;

	for (i = 0; i < clusters.n; i++)
	{
		pCluster = clusterMem + i;

		if (pCluster->bValid)
		{
			pSortIndex->cost = pCluster->size;
			pSortIndex->idx = i;
			pSortIndex++;
		}
	}

	sortedClusterArray.n = nValidClusters;

	BubbleSort<SortIndex<int>>(sortedClusterArray, true);

	// Detect the ground plane and filter all clusters lying on the ground plane.

	Array<int> PtArray;

	PtArray.Element = new int[pMesh->NodeArray.n];

	bGnd = false;

	//int *piPt;
	int iiSurfel;
	//QLIST::Index2 *pPtIdx;
	//MESH::Distribution PtDistribution;
	//float *var;
	//int idx[3];
	//int iTmp;
	float eGnd;
	float *NGnd_;

	for (i = 0; i < nValidClusters; i++)
	{
		iCluster = sortedClusterArray.Element[i].idx;

		pCluster = clusterMem + iCluster;

		if (bGnd)
		{
			//ComputeClusterNormalDistribution(pCluster);

			//if (pCluster->normalDistributionStd1 < minClusterNormalDistributionStd && pCluster->normalDistributionStd2 < minClusterNormalDistributionStd)
			{
				//if (RVLDOTPRODUCT3(NGnd, pCluster->N) >= 0.95)
				{
					for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
					{
						iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

						pSurfel = pSurfels->NodeArray.Element + iSurfel;

						eGnd = RVLDOTPRODUCT3(NGnd, pSurfel->P) - dGnd;

						if (eGnd > groundPlaneTolerance)
							break;
					}

					if (iiSurfel >= pCluster->iSurfelArray.n)
						pCluster->bValid = false;
				}
			}
		}
		else
		{
			if (IsFlat(pCluster->iSurfelArray, NGnd, dGnd, PtArray))
			{
				pCluster->bValid = false;

				bGnd = true;
			}
		}
	}

	delete[] PtArray.Element;

	//// Filter and sort clusters.

	//for (i = 0; i < clusters.n; i++)
	//{
	//	pCluster = clusterMem + i;

	//	if (pCluster->bValid)
	//		ComputeClusterBoundaryDiscontinuityPerc(i);
	//}

	//for (i = 0; i < clusters.n; i++)
	//{
	//	pCluster = clusterMem + i;

	//	if (pCluster->bValid)
	//	{
	//		if (pCluster->size <= maxClusterSize)
	//		{
	//			ComputeClusterNormalDistribution(pCluster);

	//			if (pCluster->size < minSignificantClusterSize)
	//			{
	//				if (pCluster->boundaryDiscontinuityPerc < minClusterBoundaryDiscontinuityPerc)
	//					if (pCluster->normalDistributionStd1 < minClusterNormalDistributionStd || pCluster->normalDistributionStd2 < minClusterNormalDistributionStd)
	//						pCluster->bValid = false;
	//			}
	//		}
	//		else
	//			pCluster->bValid = false;
	//	}
	//}

	RVL_DELETE_ARRAY(clusters.Element);

	clusters.Element = new RECOG::PSGM_::Cluster *[nValidClusters];

	int *clusterIndexMap = new int[clusters.n];

	memset(clusterIndexMap, 0xff, clusters.n * sizeof(int));

	int iCluster_ = 0;

	for (i = 0; i < sortedClusterArray.n; i++)
	{
		iCluster = sortedClusterArray.Element[i].idx;

		pCluster = clusterMem + iCluster;

		if (pCluster->bValid)
		{
			clusterIndexMap[iCluster] = iCluster_;

			clusters.Element[iCluster_] = pCluster;

			iCluster_++;
		}
	}

	clusters.n = iCluster_;

	//int maxClusterSize_ = 0;
	//int size;

	//for (i = 0; i < clusters.n; i++)
	//{
	//	size = clusterMem[i].size;

	//	if (size > maxClusterSize_)
	//		maxClusterSize_ = size;
	//}

	//int maxnBins = 100000;

	//int k = (maxClusterSize_ < maxnBins ? 1 : maxClusterSize_ / maxnBins + 1);

	//int *key = new int[clusters.n];

	//for (i = 0; i < clusters.n; i++)
	//	key[i] = clusterMem[i].size / k;

	//RVL::QuickSort(key, surfelBuff1.Element, clusters.n);

	//RVL_DELETE_ARRAY(clusters.Element);

	//clusters.Element = new RECOG::PSGM_::Cluster *[clusters.n];

	//for (i = 0; i < clusters.n; i++)
	//{
	//	iCluster = surfelBuff1.Element[clusters.n - i - 1];
	//	clusters.Element[i] = clusterMem + iCluster;
	//	surfelBuff2.Element[iCluster] = i;
	//}

	//delete[] key;

	// Update cluster map.	

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		iCluster = clusterMap[iSurfel];

		if (iCluster >= 0)
			clusterMap[iSurfel] = clusterIndexMap[iCluster];
	}

	delete[] clusterIndexMap;
	delete[] sortedClusterArray.Element;
	delete[] surfelBuff1.Element;
	delete[] surfelBuff2.Element;	
}

void PSGM::CreateTemplate66()
{
	float h = 0.25f * PI;
	float q = 0.5f * h;
	float sh = sin(h);
	float ch = cos(h);
	float sq = sin(q);
	float cq = cos(q);

	float *NT = new float[3 * 13];

	float *N;

	N = NT;
	RVLSET3VECTOR(N, 0.0f, 0.0f, 1.0f);
	N = NT + 3;
	RVLSET3VECTOR(N, 0.0f, -ch, ch);
	N = NT + 2 * 3;
	RVLSET3VECTOR(N, ch, 0.0f, ch);
	N = NT + 11 * 3;
	RVLSET3VECTOR(N, 0.0f, ch, ch);
	N = NT + 12 * 3;
	RVLSET3VECTOR(N, -ch, 0.0f, ch);

	int templ[] = {
		3, 0, 1,
		4, 0, 2,
		5, 1, 2,
		6, 0, 11,
		7, 0, 12,
		8, 2, 11,
		9, 1, 12,
		10, 11, 12 };

	int i;
	float *N_, *N__;
	float fTmp;

	for (i = 0; i < 8; i++)
	{
		N = NT + 3 * templ[3 * i];
		N_ = NT + 3 * templ[3 * i + 1];
		N__ = NT + 3 * templ[3 * i + 2];
		RVLSUM3VECTORS(N_, N__, N);
		RVLNORM3(N, fTmp);
	}

	float R[] = {
		0.0f, 0.0f, -1.0f,
		1.0f, 0.0f, 0.0f,
		0.0f, -1.0f, 0.0f };

	float R_[9];

	RVLMXMUL3X3(R, R, R_);

	int j;
	int i_;
	RECOG::PSGM_::Plane *pPlane;

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 11; j++)
		{
			pPlane = convexTemplate66.Element + 11 * i + j;

			N = pPlane->N;

			N_ = NT + 3 * j;

			i_ = i % 3;

			if (i_ == 0)
			{
				RVLCOPY3VECTOR(N_, N);
			}
			else if (i_ == 1)
			{
				RVLMULMX3X3VECT(R, N_, N)
			}				
			else
			{
				RVLMULMX3X3VECT(R_, N_, N)
			}
				
			if (i >= 3)
			{ 
				RVLNEGVECT3(N, N);
			}
				
			pPlane->d = 1.0f;
		}
	}

	//// Only for debugging purpose!

	//FILE *fp = fopen("convex_template.txt", "w");

	//for (i = 0; i < convexTemplate.n; i++)
	//	fprintf(fp, "%f\t%f\t%f\n", convexTemplate.Element[i].N[0], convexTemplate.Element[i].N[1], convexTemplate.Element[i].N[2]);

	//fclose(fp);

	////

	delete[] NT;
}

void PSGM::CreateTemplateBox()
{
	convexTemplateBox.n = 6;
	convexTemplateBox.Element = new RECOG::PSGM_::Plane[convexTemplateBox.n];

	float *N;

	N = convexTemplateBox.Element[0].N;
	N[0] = 0.0f; N[1] = 0.0f; N[2] = 1.0f;
	convexTemplateBox.Element[0].d = 1.0;

	N = convexTemplateBox.Element[1].N;
	N[0] = -1.0f; N[1] = 0.0f; N[2] = 0.0f;
	convexTemplateBox.Element[1].d = 1.0;

	N = convexTemplateBox.Element[2].N;
	N[0] = 0.0f; N[1] = -1.0f; N[2] = 0.0f;
	convexTemplateBox.Element[2].d = 1.0;

	N = convexTemplateBox.Element[3].N;
	N[0] = 0.0f; N[1] = 0.0f; N[2] = -1.0f;
	convexTemplateBox.Element[3].d = 1.0;

	N = convexTemplateBox.Element[4].N;
	N[0] = 1.0f; N[1] = 0.0f; N[2] = 0.0f;
	convexTemplateBox.Element[4].d = 1.0;

	N = convexTemplateBox.Element[5].N;
	N[0] = 0.0f; N[1] = 1.0f; N[2] = 0.0f;
	convexTemplateBox.Element[5].d = 1.0;
}

void PSGM::TemplateMatrix(Array2D<float> A)
{
	A.Element = new float[3 * convexTemplate.n];
	A.w = 3;
	A.h = convexTemplate.n;

	int i;
	float *a;
	float *N;

	for (i = 0; i < convexTemplate.n; i++)
	{
		a = A.Element + 3 * i;

		N = convexTemplate.Element[i].N;

		RVLCOPY3VECTOR(N, a);
	}
}

void PSGM::FitModel(
	Array<int> iVertexArray,
	RECOG::PSGM_::ModelInstance *pModelInstance,
	bool bMemAllocated)
{
	if (!bMemAllocated)
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RECOG::PSGM_::ModelInstanceElement, convexTemplate.n, pModelInstance->modelInstance.Element);

	pModelInstance->modelInstance.n = convexTemplate.n;

	float *R = pModelInstance->R;
	float *t = pModelInstance->t;

	int iModelInstanceElement;
	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;
	float d;
	SURFEL::Vertex *pVertex;
	int i;
	float *N, *P;
	float N_[3];
	//float dist;
	//float maxdDefinedNormal;
	int iVertex;

	for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
	{
		//if (iModelInstanceElement == 33)
		//	int debug = 0;

		pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

		pModelInstanceElement->valid = false;

		N = convexTemplate.Element[iModelInstanceElement].N;

		RVLMULMX3X3VECT(R, N, N_);		

		iVertex = iVertexArray.Element[0];

		pVertex = pSurfels->vertexArray.Element[iVertex];

		pModelInstanceElement->d = RVLDOTPRODUCT3(N_, pVertex->P);
		pModelInstanceElement->iVertex = 0;

		for (i = 0; i < iVertexArray.n; i++)
		{
			iVertex = iVertexArray.Element[i];

			pVertex = pSurfels->vertexArray.Element[iVertex];

			d = RVLDOTPRODUCT3(N_, pVertex->P);

			if (d > pModelInstanceElement->d)
			{
				pModelInstanceElement->d = d;
				pModelInstanceElement->iVertex = iVertex;
			}
		}

			//Vidovic
			if (bNormalValidityTest)
			{
				//if (pVertex->normalHull.n >= 3)
				//{
				//	dist = DistanceFromNormalHull(pVertex->normalHull, N_);

				//	if (dist <= 0.0f)
				//	{
				//		if (pModelInstanceElement->valid)
				//		{
				//			if (d > maxdDefinedNormal)
				//				maxdDefinedNormal = d;
				//		}
				//		else
				//		{
				//			maxdDefinedNormal = d;
				//			pModelInstanceElement->valid = true;
				//		}
				//	}
				//}
				pModelInstanceElement->valid = true;

			pVertex = pSurfels->vertexArray.Element[pModelInstanceElement->iVertex];

				P = pVertex->P;

				if (RVLDOTPRODUCT3(N_, P) >= 0.0f)
					pModelInstanceElement->valid = false;
			}
			else
				pModelInstanceElement->valid = true;
			//END Vidovic

		pModelInstanceElement->d -= RVLDOTPRODUCT3(N_, t);

		//Vidovic
		//if (bNormalValidityTest)
		//	pModelInstanceElement->e = (pModelInstanceElement->valid ? pModelInstanceElement->d - maxdDefinedNormal : 0.0f);
		//else
			pModelInstanceElement->e = 0.0f;
		//END Vidovic
	}	// for every model instance descriptor element

	//calculate segment centroid - Vidovic
	int minID, maxID;

	for (i = 0; i < 3; i++)
	{
		minID = centroidID.Element[i * 2].Idx;
		maxID = centroidID.Element[i * 2 + 1].Idx;
			
		pModelInstance->tc[i] = (pModelInstance->modelInstance.Element[maxID].d - pModelInstance->modelInstance.Element[minID].d) / 2; // PROVJERITI
	}

	//END calculate segment centroid - Vidovic
}

bool PSGM::ReferenceFrames(int iCluster)
{
	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	return ReferenceFrames(pCluster, iCluster);
}

bool PSGM::ReferenceFrames(
	RECOG::PSGM_::Cluster *pCluster,
	int iCluster)
{
	// Identify the largest surfel.

	int maxSize = 0;

	int i;
	Surfel *pSurfel;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		pSurfel = pSurfels->NodeArray.Element + pCluster->iSurfelArray.Element[i];

		if (pSurfel->size > maxSize)
			maxSize = pSurfel->size;
	}

	if (maxSize == 0)
		return false;

	//if (iCluster == 3)
	//	int debug = 0;

	int sizeThr = (int)((float)maxSize * kReferenceSurfelSize);

	// Sort surfels in the cluster.

	Array<SortIndex<int>> iSortedSurfelArray;
	
	iSortedSurfelArray.Element = new SortIndex<int>[pCluster->iSurfelArray.n];
	iSortedSurfelArray.n = 0;

	int iSurfel;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel = pCluster->iSurfelArray.Element[i];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->size >= sizeThr)
		{
			iSortedSurfelArray.Element[iSortedSurfelArray.n].idx = iSurfel;
			iSortedSurfelArray.Element[iSortedSurfelArray.n].cost = pSurfel->size;
			iSortedSurfelArray.n++;
		}
	}
		
	BubbleSort<SortIndex<int>>(iSortedSurfelArray, true);

	/// Determine reference frames of model instances. 

	//QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList); //Vidovic

	//RVLQLIST_INIT(pModelInstanceList); //Vidovic

	float cs = COS45;

	Array<RECOG::PSGM_::Tangent> tangentArray;

	tangentArray.Element = new RECOG::PSGM_::Tangent[pSurfels->NodeArray.n];

	//Array<RECOG::PSGM_::NormalHullElement> normalHull;

	//normalHull.Element = new RECOG::PSGM_::NormalHullElement[pCluster->iSurfelArray.n];

	RECOG::PSGM_::TangentRegionGrowingData tangentRGData;

	tangentRGData.bParent = new bool[pSurfels->NodeArray.n];
	memset(tangentRGData.bParent, 0, pSurfels->NodeArray.n * sizeof(bool));
	tangentRGData.bBase = new bool[pSurfels->NodeArray.n];
	memset(tangentRGData.bBase, 0, pSurfels->NodeArray.n * sizeof(bool));
	tangentRGData.pRecognition = this;
	tangentRGData.cs = cs;
	tangentRGData.iCluster = iCluster;
	tangentRGData.pTangentArray = &tangentArray;
	//tangentRGData.pNormalHull = &normalHull;
	float baseSeparationAngleRad = baseSeparationAngle * DEG2RAD;
	tangentRGData.baseSeparationAngle = baseSeparationAngleRad;

	int *iSurfelBuff = new int[pCluster->iSurfelArray.n];

	float kReferenceTangentSize2 = kReferenceTangentSize *  kReferenceTangentSize;

	float csSeparationAngle = cos(baseSeparationAngleRad);

	Array<SortIndex<float>> iSortedTangentArray;
	
	iSortedTangentArray.Element = new SortIndex<float>[pSurfels->NodeArray.n];

	Array<QList<QLIST::Index>> iTangentAngleArray;

	iTangentAngleArray.n = (int)round(360.0f / baseSeparationAngle);
	iTangentAngleArray.Element = new QList<QLIST::Index>[iTangentAngleArray.n];
	QLIST::Index *iTangentAngleMem = new QLIST::Index[pSurfels->NodeArray.n];

	int *piSurfelFetch, *piSurfelPut, *piSurfel, *piSurfelBuffEnd;
	RECOG::PSGM_::ModelInstance *pModelInstance;
	int iTangent;
	float maxTangentLen;
	RECOG::PSGM_::Tangent *pTangent, *pTangent_;
	float *R, *Z, *X, *t, *X_;
	//float *P1, *P2;
	float Y[3];
	//float P[3];
	Eigen::Matrix3f M;
	Eigen::Vector3f B, t_;
	float p, q;
	//float d;
	float tangentLenThr;
	int iLargestTangent;
	float *X0;
	float Y0[3];
	int iAngle;
	QList<QLIST::Index> *pAngleBinList;	
	QLIST::Index *pTangentAngleEntry;
	int j;

	for (i = 0; i < iSortedSurfelArray.n; i++)
	{
		iSurfel = iSortedSurfelArray.Element[i].idx;

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (!tangentRGData.bBase[iSurfel])
		{
			Z = pSurfel->N;

			piSurfelPut = piSurfelFetch = iSurfelBuff;

			*(piSurfelPut++) = iSurfel;

			tangentRGData.bBase[iSurfel] = true;

			tangentRGData.bParent[iSurfel] = true;

			RVLCOPY3VECTOR(pSurfel->N, tangentRGData.planeA.N);
			tangentRGData.planeA.d = pSurfel->d;
			tangentArray.n = 0;
			//normalHull.n = 0;

			piSurfelBuffEnd = RegionGrowing<SurfelGraph, Surfel, SURFEL::Edge, SURFEL::EdgePtr, RECOG::PSGM_::TangentRegionGrowingData, RECOG::PSGM_::ValidTangent>
				(pSurfels, &tangentRGData, piSurfelFetch, piSurfelPut);

			for (piSurfel = iSurfelBuff; piSurfel < piSurfelBuffEnd; piSurfel++)
				tangentRGData.bParent[*piSurfel] = false;

			//if (piSurfelBuffEnd - iSurfelBuff > pCluster->iSurfelArray.n)
			//	int debug = 0;

			maxTangentLen = 0;
			
			for (iTangent = 0; iTangent < tangentArray.n; iTangent++)
			{
				pTangent = tangentArray.Element + iTangent;

				if (pTangent->len > maxTangentLen)
				{
					maxTangentLen = pTangent->len;

					iLargestTangent = iTangent;
				}					
			}

			if (maxTangentLen > 0.0f)
			{
				X0 = tangentArray.Element[iLargestTangent].V;

				RVLCROSSPRODUCT3(Z, X0, Y0);

				tangentLenThr = kReferenceTangentSize2 * maxTangentLen;

				for (iAngle = 0; iAngle < iTangentAngleArray.n; iAngle++)
				{
					pAngleBinList = iTangentAngleArray.Element + iAngle;

					RVLQLIST_INIT(pAngleBinList);
				}

				pTangentAngleEntry = iTangentAngleMem;

				iSortedTangentArray.n = 0;

				for (iTangent = 0; iTangent < tangentArray.n; iTangent++)
				{
					pTangent = tangentArray.Element + iTangent;

					if (pTangent->len >= tangentLenThr)
					{
						iSortedTangentArray.Element[iSortedTangentArray.n].idx = iTangent;
						iSortedTangentArray.Element[iSortedTangentArray.n].cost = pTangent->len;
						iSortedTangentArray.n++;

						X = pTangent->V;

						p = RVLDOTPRODUCT3(X0, X);
						q = RVLDOTPRODUCT3(Y0, X);

						iAngle = (int)round((atan2(q, p) + PI) / baseSeparationAngleRad) % iTangentAngleArray.n;

						pAngleBinList = iTangentAngleArray.Element + iAngle;

						RVLQLIST_ADD_ENTRY(pAngleBinList, pTangentAngleEntry);

						pTangentAngleEntry->Idx = iTangent;

						pTangentAngleEntry++;
					}
				}

				BubbleSort<SortIndex<float>>(iSortedTangentArray, true);

				for (iTangent = 0; iTangent < iSortedTangentArray.n; iTangent++)
				{
					pTangent = tangentArray.Element + iSortedTangentArray.Element[iTangent].idx;

					if (!pTangent->bMerged)
					{
						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::ModelInstance, pModelInstance);

						//RVLQLIST_ADD_ENTRY(pModelInstanceList, pModelInstance); Vidovic

						CTISet.AddCTI(pModelInstance); //Vidovic

						pModelInstance->iCluster = iCluster; //Vidovic - ADDED iCluster data to scene MI

						pModelInstance->iModel = -1; //Vidovic - ADDED iModel data to scene MI

						R = pModelInstance->R;

						RVLCOPYTOCOL3(Z, 2, R);

						X = pTangent->V;

						RVLCOPYTOCOL3(X, 0, R);

						RVLCROSSPRODUCT3(Z, X, Y);

						RVLCOPYTOCOL3(Y, 1, R);

						// Computing origin of the reference frame.

						//P1 = pSurfels->vertexArray.Element[pTangent->iVertex[0]]->P;
						//P2 = pSurfels->vertexArray.Element[pTangent->iVertex[1]]->P;

						//RVLSUM3VECTORS(P1, P2, P);

						//RVLSCALE3VECTOR(P, 0.5f, P);

						//d = RVLDOTPRODUCT3(X, P);

						//M << pSurfel->N[0], pSurfel->N[1], pSurfel->N[2], pTangent->N[0], pTangent->N[1], pTangent->N[2], X[0], X[1], X[2];

						//B << pSurfel->d, pTangent->d, d;

						//t_ = M.colPivHouseholderQr().solve(B);

						t = pModelInstance->t;

						//RVLCOPY3VECTOR(t_, t);

						RVLNULL3VECTOR(t);

						p = RVLDOTPRODUCT3(X0, X);
						q = RVLDOTPRODUCT3(Y0, X);

						iAngle = (int)round((atan2(q, p) + PI) / baseSeparationAngleRad) % iTangentAngleArray.n;

						for (j = 0; j < 2; j++)
						{
							pAngleBinList = iTangentAngleArray.Element + iAngle;

							pTangentAngleEntry = pAngleBinList->pFirst;

							while (pTangentAngleEntry)
							{
								pTangent_ = tangentArray.Element + pTangentAngleEntry->Idx;

								X_ = pTangent_->V;

								if (RVLDOTPRODUCT3(X, X_) > csSeparationAngle)
									pTangent_->bMerged = true;

								pTangentAngleEntry = pTangentAngleEntry->pNext;
							}

							iAngle = (iAngle + iTangentAngleArray.n - 1) % iTangentAngleArray.n;
						}
					}	// if (pTangent->len >= kReferenceTangentSize2 * maxTangentLen)
				}	// for every tangent
			}	// if (maxTangentLen > 0.0f)
		}	// if (pSurfel->size >= kReferenceSurfelSize * maxSize)
	}	// for every surfel in the cluster

	delete[] tangentArray.Element;
	delete[] tangentRGData.bParent;
	delete[] tangentRGData.bBase;
	delete[] iSurfelBuff;
	//delete[] normalHull.Element;
	delete[] iSortedSurfelArray.Element;
	delete[] iSortedTangentArray.Element;
	delete[] iTangentAngleMem;

	return true;
}

RECOG::PSGM_::ModelInstance * PSGM::AddReferenceFrame(
	//int iCluster, //Vidovic
	float *RIn,
	float *tIn)
{
	//RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster]; //Vidovic

	//QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList); //Vidovic

	RECOG::PSGM_::ModelInstance *pModelInstance;

	RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::ModelInstance, pModelInstance);
	//RVLQLIST_ADD_ENTRY(pModelInstanceList, pModelInstance); //Vidovic

	CTISet.AddCTI(pModelInstance); //Vidovic

	float *R = pModelInstance->R;
	float *t = pModelInstance->t;

	if (RIn)
	{
		RVLCOPYMX3X3(RIn, R)
	}		
	else
	{
		RVLUNITMX3(R)
	}

	if (tIn)
	{
		RVLCOPY3VECTOR(tIn, t)
	}
	else
	{
		RVLNULL3VECTOR(t);
	}

	return pModelInstance;
}

int RVL::RECOG::PSGM_::ValidTangent(
	int iSurfel, 
	int iSurfel_, 
	SURFEL::Edge *pEdge, 
	SurfelGraph *pSurfels, 
	RECOG::PSGM_::TangentRegionGrowingData *pData)
{	
	PSGM *pRecognition = pData->pRecognition;

	//if (pRecognition->clusterMap[iSurfel] != pData->iCluster)
	//	return -1;

	if (pData->bParent[iSurfel])
		return -1;

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	float *N0 = pData->planeA.N;
	float d0 = pData->planeA.d;
	float *N = pSurfel->N;
	float *N_ = pSurfel_->N;
	float cs = RVLDOTPRODUCT3(N0, N);

	if (cs <= pData->cs)
	{
		RECOG::PSGM_::Tangent *pTangent = pData->pTangentArray->Element + pData->pTangentArray->n;

		pData->pTangentArray->n++;

		pTangent->bMerged = false;

		float *NT = pTangent->N;

		// N0'*NT = cs,     NT = (s*N + (1-s)*N_) / || s*N + (1-s)*N_ ||
		// N0'*(s*N + (1-s)*N_) = cs*sqrt(s*N + (1-s)*N_)'*(s*N + (1-s)*N_)
		// s*N0'*N + N0'*N_ - s*N0'*N_ = cs * sqrt(s^2*N'*N + 2*s*(1-s)*N'*N_ + (1-s)^2*N_'*N_)
		// N0'(N-N_)*s + N0'*N_ = cs * sqrt(s^2 + 2*s*(1-s)*N'*N_ + (1-s)^2)
		// N0'(N-N_)*s + N0'*N_ = cs * sqrt(2*(1 - N'*N_)*s^2 - 2*(1 - N'*N_)*s + 1)
		// a*s + b = cs * sqrt(c*s^2 - c*s + 1),     a = N0'(N-N_), b = N0'*N_, c = 2*(1 - N'*N_)
		// a^2*s^2 + 2*a*b*s + b^2 = cs^2 * (c*s^2 - c*s + 1)
		// p*s^2 + q*s + r = 0,     p = a^2-cs^2*c, q = 2*a*b+cs^2*c, r = b^2-cs^2
		// s = (-q +- sqrt(q^2 - 4*p*r))/(2*p),    0 <= s <= 1

		float VTmp[3];
		RVLDIF3VECTORS(N, N_, VTmp);
		float a = RVLDOTPRODUCT3(N0, VTmp);
		float b = RVLDOTPRODUCT3(N0, N_);
		float c = 2.0f * (1.0f - RVLDOTPRODUCT3(N, N_));
		float cs2 = pData->cs * pData->cs;
		float p = a*a - cs2 * c;
		float q = 2.0f * a * b + cs2 * c;
		float r = b * b - cs2;
		float f = -sqrt(q * q - 4.0f * p * r);
		float s = (-q + f) / (2 * p);
		
		if (s < 0.0f || s > 1.0f)
			s = (-q - f) / (2 * p);

		RVLSCALE3VECTOR(N, s, VTmp);
		float s_ = 1.0f - s;
		RVLSCALE3VECTOR(N_, s_, NT);
		RVLSUM3VECTORS(NT, VTmp, NT);
		float fTmp;
		RVLNORM3(NT, fTmp);

		//pRecognition->UpdateNormalHull(*(pData->pNormalHull), NT);

		QList<QLIST::Index> *pVertexList = pSurfels->surfelVertexList.Element + iSurfel;

		bool bMindT = false;

		int nTangentVertices = 0;
		
		pTangent->len = 0.0f;

		float *V = pTangent->V;

		RVLCROSSPRODUCT3(N0, NT, V);

		RVLNORM3(V, fTmp);

		Eigen::Matrix3f M;

		M << N0[0], N0[1], N0[2], NT[0], NT[1], NT[2], V[0], V[1], V[2];

		Eigen::Vector3f B, P_;
		SURFEL::Vertex *pVertex;
		int i;
		float mindT, dT, d_, len13, len23;
		float P1[3], P2[3], dP13[3], dP23[3], PProj[3], dP[3];
		float *P;

		QLIST::Index *pVertexIdx = pVertexList->pFirst;

		while (pVertexIdx)
		{
			pVertex = pSurfels->vertexArray.Element[pVertexIdx->Idx];

			for (i = 0; i < pVertex->iSurfelArray.n; i++)
			{
				if (pVertex->iSurfelArray.Element[i] == iSurfel_)
				{
					P = pVertex->P;

					dT = RVLDOTPRODUCT3(NT, P);

					d_ = RVLDOTPRODUCT3(V, P);

					B << d0, dT, d_;

					P_ = M.colPivHouseholderQr().solve(B);					
					
					if (nTangentVertices == 0)
					{
						nTangentVertices = 1;

						RVLCOPY3VECTOR(P_, P1);

						pTangent->iVertex[0] = pVertexIdx->Idx;
					}
					else if (nTangentVertices == 1)
					{
						nTangentVertices = 2;

						RVLCOPY3VECTOR(P_, P2);

						RVLDIF3VECTORS(P2, P1, dP);

						pTangent->len = RVLDOTPRODUCT3(dP, dP);

						pTangent->iVertex[1] = pVertexIdx->Idx;
					}
					else	// if (nTangentVertices == 2)
					{
						RVLCOPY3VECTOR(P_, PProj);

						RVLDIF3VECTORS(PProj, P1, dP13);

						len13 = RVLDOTPRODUCT3(dP13, dP13);

						RVLDIF3VECTORS(PProj, P2, dP23);

						len23 = RVLDOTPRODUCT3(dP23, dP23);

						if (len13 > pTangent->len || len23 > pTangent->len)
						{
							if (len13 > len23)
							{
								pTangent->iVertex[1] = pVertexIdx->Idx;

								pTangent->len = len13;
							}
							else
							{
								pTangent->iVertex[0] = pVertexIdx->Idx;

								pTangent->len = len23;
							}
						}
					}	// if (nTangentVertices == 2)

					if (bMindT)
					{
						if (dT < mindT)
							mindT = dT;
					}
					else
					{
						mindT = dT;

						bMindT = true;
					}
				}	// if (pVertex->iSurfelArray.Element[i] == iSurfel_) 			
			}	// for all surfels meeting in pVertex

			pVertexIdx = pVertexIdx->pNext;
		}	// for all vertices on the boundary of iSurfel

		pTangent->d = mindT;

		return -1;
	}	// if (RVLDOTPRODUCT3(N0, N) > pData->cs && RVLDOTPRODUCT3(N0, N_) <= pData->cs)
	else if (pRecognition->clusterMap[iSurfel] == pData->iCluster)
	{
		pData->bParent[iSurfel] = true;

		//if (cs < pData->baseSeparationAngle)
			pData->bBase[iSurfel] = true;

		return 1;
	}
	else
		return -1;
}

bool PSGM::Inside(
	int iVertex, 
	RECOG::PSGM_::Cluster *pCluster,
	int iSurfel)
{
	float maxe = kNoise * 2.0f / pSurfelDetector->kPlane;

	SURFEL::Vertex *pVertex = pSurfels->vertexArray.Element[iVertex];

	int i;
	int iSurfel_;
	float e;
	Surfel *pSurfel_;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel_ = pCluster->iSurfelArray.Element[i];

		if (iSurfel_ == iSurfel)
			continue;

		pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

		e = RVLDOTPRODUCT3(pSurfel_->N, pVertex->P) - pSurfel_->d;

		if (e > maxe)
		//if (e < -maxe)
			return false;
	}

	return true;
}

bool PSGM::BelowPlane(
	RECOG::PSGM_::Cluster *pCluster,
	Surfel *pSurfel,
	int iFirstVertex)
{
	float maxe = kNoise * 2.0f / pSurfelDetector->kPlane;

	float e;
	int iVertex_;
	SURFEL::Vertex *pVertex;

	for (iVertex_ = 0; iVertex_ < pCluster->iVertexArray.n; iVertex_++)
	{
		pVertex = pSurfels->vertexArray.Element[pCluster->iVertexArray.Element[iVertex_]];

		e = RVLDOTPRODUCT3(pSurfel->N, pVertex->P) - pSurfel->d;

		if (e > maxe)
		//if (e < -maxe)
			return false;
	}

	return true;
}

float PSGM::DistanceFromNormalHull(
	Array<SURFEL::NormalHullElement> &NHull,
	float *N)
{
	if (NHull.n == 0)
		return 0.0f;
	if (NHull.n == 1)
	{
		float *N_ = NHull.Element[0].N;

		float e = RVLDOTPRODUCT3(N_, N);

		return (e < 0.0f ? 1.0f : sqrt(1.0f - e * e));
	}		

	float maxDist = 0.0f;

	int i;
	float dist;
	float *Nh_;

	for (i = 0; i < NHull.n; i++)
	{
		Nh_ = NHull.Element[i].Nh;

		dist = RVLDOTPRODUCT3(Nh_, N);

		if (dist > maxDist)
			maxDist = dist;
	}

	return maxDist;
}

void PSGM::UpdateMeanNormal(
	float *sumN,
	float &wN,
	float *N,
	float w,
	float *meanN)
{
	float VTmp[3];
	RVLSCALE3VECTOR(N, w, VTmp);
	RVLSUM3VECTORS(sumN, VTmp, sumN);
	wN += w;
	RVLSCALE3VECTOR2(sumN, wN, meanN);
	float fTmp = sqrt(RVLDOTPRODUCT3(meanN, meanN));
	if (fTmp > 1e-10)
	{
		RVLSCALE3VECTOR2(meanN, fTmp, meanN);
	}
	else
		RVLSET3VECTOR(meanN, 0.0f, 0.0f, 1.0f);
}

void PSGM::SetSceneFileName(char *sceneFileName_)
{
	RVLCopyString(sceneFileName_, &sceneFileName);
}

void PSGM::SaveCTIs(
	FILE *fp,
	RECOG::CTISet *pCTISet,
	int iModel)
{
	int i;
	int iModelInstanceElement;
	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;

	RECOG::PSGM_::ModelInstance *pModelInstance = pCTISet->CTI.pFirst;

	while (pModelInstance)
	{
		fprintf(fp, "%d\t%d\t", iModel, pModelInstance->iCluster);

		for (i = 0; i < 9; i++)
			fprintf(fp, "%f\t", pModelInstance->R[i]);

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pModelInstance->t[i]);

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%f\t", pModelInstanceElement->d);
		}

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%d\t", (int)(pModelInstanceElement->valid));
		}

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%f\t", pModelInstanceElement->e);
		}

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pModelInstance->tc[i]);

		fprintf(fp, "\n");

		pModelInstance = pModelInstance->pNext;
	}
}

//Vidovic
void PSGM::SaveModelInstances(
	FILE *fp,
	int iModel)
{
	SaveCTIs(fp, &CTISet, iModel);
}


bool PSGM::ModelExistInDB(char *modelFileName, FileSequenceLoader dbLoader)
{
	char dbFileName[50];

	while (dbLoader.GetNextName(dbFileName))
		if (!strcmp(modelFileName, dbFileName))
			return 1;

	return 0;
}

void PSGM::SaveModelID(FileSequenceLoader dbLoader)
{
	FILE *fp = fopen(modelsInDataBase, "w");

	char modelName[50], modelPath[200];
	int modelID;

	while (dbLoader.GetNext(modelPath, modelName, &modelID))
		fprintf(fp, "%d\t%s\n", modelID, modelName);

	fprintf(fp, "\nend");

	fclose(fp);
}

void PSGM::Learn(
	char *modelSequenceFileName,
	Visualizer *visualizer)
{
	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	FileSequenceLoader modelsLoader;
	FileSequenceLoader dbLoader;

	char modelFilePath[200];
	char modelFileName[200];

	Mesh mesh;

	//int iCluster;
	int nClusters, currentModelID;

	//RVL_DELETE_ARRAY(modelDataBase);
	//RVL_DELETE_ARRAY(modelsInDataBase);

	if (!modelDataBase)
		modelDataBase = "modelDB.dat";

	if (!modelsInDataBase)
		modelsInDataBase = "DBModels.txt";

	modelsLoader.Init(modelSequenceFileName);
	dbLoader.Init(modelsInDataBase);

	FILE *fp = fopen(modelDataBase, "a");

	bool saveDBSequenceFile = false;

	printf("Model DB creation started...\n");

	while (modelsLoader.GetNext(modelFilePath, modelFileName))
	{
		if (ModelExistInDB(modelFileName, dbLoader))
			continue;

		printf("\nProcessing model %s!\n", modelFileName);

		saveDBSequenceFile = true;

		mesh.LoadPolyDataFromPLY(modelFilePath);

		SetSceneFileName(modelFilePath);

		Interpret(&mesh);

		nClusters = RVLMIN(clusters.n, nDominantClusters);

		currentModelID = dbLoader.GetLastModelID() + 1;

		//Add vtkPolyData to vtkModelDB
		vtkModelDB.insert(std::make_pair(currentModelID, mesh.pPolygonData));

		SaveModelInstances(fp, currentModelID);

		dbLoader.AddModel(currentModelID, modelFilePath, modelFileName);

		if (visualizer)
		{
			InitDisplay(visualizer, &mesh, SelectionColor);
			Display();
			visualizer->Run();

			visualizer->renderer->RemoveAllViewProps();
		}
	}

	printf("Model DB creation completed!\n");

	if (saveDBSequenceFile)
		SaveModelID(dbLoader);

	fclose(fp);
}


void PSGM::LoadModelMeshDB(char *modelSequenceFileName, bool decimate, float decimatePercent)
{
	FileSequenceLoader modelsLoader;

	char modelFilePath[200];
	char modelFileName[200];

	Mesh mesh;

	int nClusters, currentModelID = 0;

	if (!modelsInDataBase)
		modelsInDataBase = "DBModels.txt";

	modelsLoader.Init(modelSequenceFileName);

	printf("Starting VTK Model DB creation.\n");
	while (modelsLoader.GetNext(modelFilePath, modelFileName))
	{
		printf("Loading VTK model %s to DB!\n", modelFileName);

		mesh.LoadPolyDataFromPLY(modelFilePath);
		vtkSmartPointer<vtkDecimatePro> decimate = vtkSmartPointer<vtkDecimatePro>::New();
		//mesh.pPolygonData->Print(std::cout);

		if (decimate) //subsampling the model to reduce number of points and fasten the process
		{
			decimate->SetInputData(mesh.pPolygonData);
			decimate->SetTargetReduction(decimatePercent);
			decimate->Update();
			//decimate->GetOutput()->Print(std::cout);
		}
		
		//Calculate normals
		vtkSmartPointer<vtkPolyDataNormals> normalsFilter = vtkSmartPointer<vtkPolyDataNormals>::New();
		normalsFilter->ComputePointNormalsOn();
		normalsFilter->SplittingOff();
		if (decimate)
			normalsFilter->SetInputConnection(decimate->GetOutputPort());
		else
			normalsFilter->SetInputData(mesh.pPolygonData);
		normalsFilter->Update();
		//normalsFilter->GetOutput()->Print(std::cout);
		/*vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
		cleanFilter->SetInputConnection(normalsFilter->GetOutputPort());
		cleanFilter->PointMergingOn();
		cleanFilter->SetAbsoluteTolerance(0.03);
		cleanFilter->ToleranceIsAbsoluteOn();
		cleanFilter->Update();*/
		//cleanFilter->GetOutput()->Print(std::cout);
		
		//Add vtkPolyData to vtkModelDB:
		vtkModelDB.insert(std::make_pair(currentModelID, normalsFilter->GetOutput()));
		currentModelID++;
	}

	printf("VTK Model DB creation completed!\n");
}


void PSGM::LoadModelDataBase()
{
	MCTISet.nT = convexTemplate.n;
	MCTISet.Load(modelDataBase);

	//Alocate arrays for Match() function
	e.Element = new Array<float>[MCTISet.pCTI.n];
	e.n = MCTISet.pCTI.n;

	tBestMatch.Element = new Array<float>[MCTISet.pCTI.n];
	tBestMatch.n = MCTISet.pCTI.n;


	score.Element = new float[MCTISet.pCTI.n];
	score.n = MCTISet.pCTI.n;

	for (int i = 0; i < MCTISet.pCTI.n; i++)
	{
		e.Element[i].Element = new float[convexTemplate.n];
		e.Element[i].n = convexTemplate.n;

		tBestMatch.Element[i].Element = new float[3];
		tBestMatch.Element[i].n = 3;
	}
}

void PSGM::LoadCTI(char *fileName)
{
	CTISet.nT = convexTemplate.n;

	CTISet.Load(fileName);
}

#ifdef NEVER
void PSGM::Match()
{
	printf("Scene to model match started...");

	ClearMatchMatrix(); //Vidovic

	float csMinSampleAngleDiff = cos(PI / 4);
	float minE, minETotal, E, score, SMI_minETotal;

	float cos45 = cos(PI / 4);

	float sigma = 4;
	float sigma25 = 2.5*2.5;

	float tBestMatch[3], SMI_tBestMatch[3];
	int bestMatchMIMID, bestMatchModelID, bestSRF, iSRF;
	int SMI_bestMatchMIMID, SMI_bestMatchModelID, SMI_bestSRF, SMI_iSRF;

	QList<RECOG::PSGM_::MatchInstance> *pSMIMatches = &SMImatches;

	//if (iScene == 0)
	RVLQLIST_INIT(pSMIMatches);

	RECOG::PSGM_::MatchInstance *pSMIMatch;

	float R_[9], t_[3];

	Eigen::Matrix3f A;
	Eigen::Vector3f B, t;

	int i, idx;

	float dISv, dIMvt;

	FILE *fp;

	//if (!sceneMIMatch)
	//	sceneMIMatch = "sceneMatch.txt";

	//fp = fopen(sceneMIMatch, "w");

	//find sample candidates
	QList<QLIST::Index> iSampleCandidateList;
	QList<QLIST::Index> *pISampleCandidateList = &iSampleCandidateList;

	RVLQLIST_INIT(pISampleCandidateList);

	QLIST::Index *pNewISampleCandidate;
	RECOG::PSGM_::Plane *pPlane = convexTemplate.Element;

	for (int idx = 0; idx < convexTemplate.n; idx++)
	{
		if (RVLABS(pPlane->N[2]) <= csMinSampleAngleDiff)
		{
			RVLMEM_ALLOC_STRUCT(pMem, QLIST::Index, pNewISampleCandidate);

			RVLQLIST_ADD_ENTRY(pISampleCandidateList, pNewISampleCandidate);

			pNewISampleCandidate->Idx = idx;
		}		

		pPlane++;
	}

	int iMIS, iMIM;
	int iSCluster, iSClusterMI;

	int MatchID = 0;

	int nClusters = RVLMIN(clusters.n, nDominantClusters);

	RECOG::PSGM_::ModelInstance *pSModelInstance;
	RECOG::PSGM_::ModelInstance *pMModelInstance;

	RECOG::PSGM_::ModelInstance *pSBestModelInstance;
	RECOG::PSGM_::ModelInstance *pMBestModelInstance;

	RECOG::PSGM_::ModelInstance *SMI_pSBestModelInstance;
	RECOG::PSGM_::ModelInstance *SMI_pMBestModelInstance;

	bool breakPrint;
	bool cluserMatch;

	//Arrays allocation
	Array<QLIST::Index> iValidSampleCandidate;
	iValidSampleCandidate.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iValid;
	iValid.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iRansacCandidates;
	iRansacCandidates.Element = new QLIST::Index[26]; //max 26 planes which satisfy condition

	Array<QLIST::Index> iConsensus;
	iConsensus.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iConsensusTemp;
	iConsensusTemp.Element = new QLIST::Index[convexTemplate.n];

	iMIS = 0;

	bool segmentTP, TP_;

	float distanceThresh = 50;
	
	char *fileName = strrchr(sceneFileName, '\\') + 1;
	
	if (strcmp(fileName, "frame_20111220T114628.408278.ply") == 0)
		distanceThresh = 45;
	else if (strcmp(fileName, "frame_20111220T115430.348560.ply") == 0)
		distanceThresh = 20;
	else if (strcmp(fileName, "frame_20111220T115445.303284.ply") == 0)
		distanceThresh = 15;
	else if (strcmp(fileName, "frame_20111221T142636.413299.ply") == 0)
		distanceThresh = 25;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		segmentTP = false;

		breakPrint = false; //for DEBUG!

		cluserMatch = false;

		printf("%d/%d", iSCluster + 1, nClusters);

		pSModelInstance = clusters.Element[iSCluster]->modelInstanceList.pFirst;

		iSRF = -1;

		while (pSModelInstance)
		{
			SMI_minETotal = 66.0;

			iSRF++;

			pMModelInstance = modelInstanceDB.Element;

			for (iMIM = 0; iMIM < modelInstanceDB.n; iMIM++)
			{
				//minE = 66.0;
				minE = 412.5; // for sigma = 2.5^2

				//CTDMatchRANSAC
				float pPrior = 2.5 * stdNoise;

				int nValidSampleCandidates = 0;

				//find valid sample candidates
				iValidSampleCandidate.n = 0;

				QLIST::Index *piValidSampleCandidate = iValidSampleCandidate.Element;

				QLIST::Index *pISampleCandidate = pISampleCandidateList->pFirst;

				while (pISampleCandidate)
				{
					if (pSModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true && pMModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true)
					{
						piValidSampleCandidate->Idx = pISampleCandidate->Idx;

						piValidSampleCandidate->pNext = piValidSampleCandidate + 1;

						piValidSampleCandidate++;

						nValidSampleCandidates++;
					}

					pISampleCandidate = pISampleCandidate->pNext;
				}

				iValidSampleCandidate.n = nValidSampleCandidates;

				if (nValidSampleCandidates > 2)
				{
					int nValids = 0;

					//find iValid
					iValid.n = 0;

					QLIST::Index *piValid = iValid.Element;
					int iPlane;

					for (iPlane = 0; iPlane < convexTemplate.n; iPlane++)
					{
						if (pSModelInstance->modelInstance.Element[iPlane].valid == true && pMModelInstance->modelInstance.Element[iPlane].valid == true)
						{
							piValid->Idx = iPlane;

							piValid->pNext = piValid + 1;

							piValid++;

							nValids++;
						}
					}

					iValid.n = nValids;

					int iRansac;
					bool bValidSample;

					int iSample[2];
					int ID[2];
					float V[3];
					float N_[3] = { 0, 0, 1 };
					float N__[3] = { 0, -cos45, cos45 };
					float fTmp;
					int nValidSamplesSearch;
					int nRansacCandidates = 0;

					iConsensus.n = 0;

					QLIST::Index *piConsensus = iConsensus.Element;

					iRansacCandidates.n = 0;

					QLIST::Index *piRansacCandidates = iRansacCandidates.Element;

					//find RANSAC candidates
					RVLCROSSPRODUCT3(N_, N__, V);

					fTmp = sqrt(RVLDOTPRODUCT3(V, V));

					RVLSCALE3VECTOR2(V, fTmp, V);
					
					for (i = 0; i < iValidSampleCandidate.n; i++)
					{
						ID[0] = iValidSampleCandidate.Element[i].Idx;

						fTmp = RVLDOTPRODUCT3(V, convexTemplate.Element[ID[0]].N);

						if (RVLABS(fTmp) >= csMinSampleAngleDiff)
						{
							piRansacCandidates->Idx = ID[0];

							piRansacCandidates++;

							nRansacCandidates++;
						}
					}

					iRansacCandidates.n = nRansacCandidates;

					std::random_device rd;
					std::mt19937 eng(rd());
					std::uniform_int_distribution<> distribution(0, nRansacCandidates);

					nSamples = RVLMIN(13, nRansacCandidates);

					for (iRansac = 0; iRansac < nSamples; iRansac++)
					{
#ifdef NEVER
						bValidSample = false;

						nValidSamplesSearch = 0;

						while (!bValidSample)
						{
							nValidSamplesSearch++;

							iSample[0] = distribution(eng);
							iSample[1] = distribution(eng);

							ID[0] = iValidSampleCandidate.Element[iSample[0]].Idx;
							ID[1] = iValidSampleCandidate.Element[iSample[1]].Idx;

							RVLCROSSPRODUCT3(N_, convexTemplate.Element[ID[0]].N, V);

							fTmp = sqrt(RVLDOTPRODUCT3(V, V));

							//if (RVLABS(fTmp) < 1e-10)
							//return;

							RVLSCALE3VECTOR2(V, fTmp, V);

							if (RVLDOTPRODUCT3(V, convexTemplate.Element[ID[1]].N) >= csMinSampleAngleDiff)
								bValidSample = true;

							if (nValidSamplesSearch > 2000)
								break;
						}

						if (nValidSamplesSearch > 2000)
							break;
#endif

						ID[0] = 1; //second plane from convexTemplate

						iSample[1] = distribution(eng);

						ID[1] = iValidSampleCandidate.Element[iSample[1]].Idx;

						float dM[3];
						float dS[3];
						float N[9];

						dM[0] = pMModelInstance->modelInstance.Element[0].d;
						dM[1] = pMModelInstance->modelInstance.Element[ID[0]].d;
						dM[2] = pMModelInstance->modelInstance.Element[ID[1]].d;

						dS[0] = pSModelInstance->modelInstance.Element[0].d * 1000;
						dS[1] = pSModelInstance->modelInstance.Element[ID[0]].d * 1000;
						dS[2] = pSModelInstance->modelInstance.Element[ID[1]].d * 1000;

						RVLCOPYTOCOL3(convexTemplate.Element[0].N, 0, N);
						RVLCOPYTOCOL3(convexTemplate.Element[ID[0]].N, 1, N);
						RVLCOPYTOCOL3(convexTemplate.Element[ID[1]].N, 2, N);

						A << N[0], N[3], N[6], N[1], N[4], N[7], N[2], N[5], N[8]; // N'
						B << dS[0] - dM[0], dS[1] - dM[1], dS[2] - dM[2];
						t = A.colPivHouseholderQr().solve(B);

						iConsensusTemp.n = 0;

						QLIST::Index *piConsensusTemp = iConsensusTemp.Element;

						E = 0;

						for (i = 0; i < iValid.n; i++)
						{
							idx = iValid.Element[i].Idx;

							dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

							dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

							//fTmp = (dISv - dIMvt) / pPrior;
							fTmp = (dISv - dIMvt) / sigma;

							//if (fTmp*fTmp < 1)
							#ifdef RVLPSGM_MATCH_SATURATION
								if (fTmp*fTmp < sigma25)
								//if (fTmp*fTmp < 1)
								{
									E += fTmp*fTmp;

									piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
									piConsensusTemp->pNext = piConsensusTemp + 1;

									iConsensusTemp.n++;

									piConsensusTemp++;
								}
								else
									E += sigma25;
									//E += 1;
							#else
								E += fTmp*fTmp;

								//TREBA LI OVO?
								piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
								piConsensusTemp->pNext = piConsensusTemp + 1;

								iConsensusTemp.n++;

								piConsensusTemp++;
								
							#endif
						}

						if (E < minE)
						{
							minE = E;

							iConsensus.n = iConsensusTemp.n;

							piConsensusTemp = iConsensusTemp.Element;

							piConsensus = iConsensus.Element;

							for (i = 0; i < iConsensusTemp.n; i++)
							{
								piConsensus->Idx = piConsensusTemp->Idx;
								piConsensus->pNext = piConsensus + 1;

								piConsensus++;
								piConsensusTemp++;
							}
						}
					}

					if (iConsensus.n >= 3)
					{
						float dISc, dIMc, *nTc, *dISMc;

						nTc = new float[3 * iConsensus.n];
						dISMc = new float[iConsensus.n];

						piConsensus = iConsensus.Element;

						for (i = 0; i < iConsensus.n; i++)
						{
							idx = piConsensus->Idx;

							dISc = pSModelInstance->modelInstance.Element[idx].d * 1000;
							dIMc = pMModelInstance->modelInstance.Element[idx].d;

							dISMc[i] = dISc - dIMc;

							nTc[i] = convexTemplate.Element[idx].N[0];
							nTc[iConsensus.n + i] = convexTemplate.Element[idx].N[1];
							nTc[2 * iConsensus.n + i] = convexTemplate.Element[idx].N[2];

							piConsensus++;
						}

						int j, k;

						//nTc*nTc'
						for (i = 0; i < 3; i++)
							for (j = 0; j < 3; j++)
								if (i <= j)
								{
									A(i * 3 + j) = 0;

									for (k = 0; k < iConsensus.n; k++)
										A(i * 3 + j) += nTc[i * iConsensus.n + k] * nTc[j * iConsensus.n + k];
								}
								else
									A(i * 3 + j) = A(j * 3 + i);

						//nTc*(dISc-dIMc)
						for (i = 0; i < 3; i++)
						{
							B(i) = 0;
							for (j = 0; j < iConsensus.n; j++)
								B(i) += nTc[i * iConsensus.n + j] * dISMc[j];
						}

						t = A.colPivHouseholderQr().solve(B);

						delete[] nTc;
						delete[] dISMc;

						E = 0;

						for (i = 0; i < iValid.n; i++)
						{
							idx = iValid.Element[i].Idx;

							dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

							dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

							//fTmp = (dISv - dIMvt) / pPrior;
							fTmp = (dISv - dIMvt) / sigma;

							#ifdef RVLPSGM_MATCH_SATURATION
								//if (fTmp*fTmp < 1)
								if (fTmp*fTmp < sigma25)
									E += fTmp*fTmp;
								else
									//E += 1;
									E += sigma25;
							#else
								E += fTmp*fTmp;
							#endif
						}

						//score = E + 66 - iValid.n;
						score = E + sigma25*(66 - iValid.n);

						//save all SMI matches						
						for (i = 0; i < 3; i++)
							SMI_tBestMatch[i] = t(i);

						MSTransformation(pMModelInstance, pSModelInstance, SMI_tBestMatch, R_, t_);

						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pSMIMatch);

						RVLQLIST_ADD_ENTRY(pSMIMatches, pSMIMatch);

						FillMatch(pSMIMatch, MatchID, iScene, iSCluster, iSRF, iMIS, pMModelInstance->iModel, pMModelInstance->iCluster, iMIM, R_, t_, SMI_tBestMatch, E, score, 0.0, 0.0, NAN, NAN, iValid.n);

						MatchID++;							

						//check MatchMatrix update
						if (matchMatrix.Element[pSMIMatch->iCluster].Element[pSMIMatch->iModel*nMSegments + pSMIMatch->iMCluster] == NULL)
						{
							UpdateMatchMatrix(pSMIMatch, pSMIMatch->score);
						}
						else
						{
							if (pSMIMatch->score < matchMatrix.Element[pSMIMatch->iCluster].Element[pSMIMatch->iModel*nMSegments + pSMIMatch->iMCluster]->score)
							{
								UpdateMatchMatrix(pSMIMatch, pSMIMatch->score);
							}
						}

						//create SegmentGT
						if (createSegmentGT)
						{
							TP_ = CompareMatchToGT(pSMIMatch, true, 0.0, distanceThresh);

							if (TP_ && !segmentTP)
							{
								segmentTP = true;
								segmentGT.Element[iScene*nDominantClusters + iSCluster].iScene = iScene;

								segmentGT.Element[iScene*nDominantClusters + iSCluster].iSSegment = iSCluster;
								segmentGT.Element[iScene*nDominantClusters + iSCluster].iModel = pSMIMatch->iModel;
								segmentGT.Element[iScene*nDominantClusters + iSCluster].iMSegment = pSMIMatch->iMCluster;
							}
						}
					}
					else
					{
						t << 0, 0, 0;
						//E = 66;
						E = 412.5;
					}
				}
				else
				{
					if (!breakPrint)
					{
						//printf("BREAK - nValidSampleCandidates = %d; iCluster: %d; iMIM: %d \n", nValidSampleCandidates, iSCluster, iMIM);
						breakPrint = true;
					}
				}

				pMModelInstance = pMModelInstance->pNext;

			}	//for all model MI

			iMIS++;

			pSModelInstance = pSModelInstance->pNext;

		}	// for all MI in cluster

		//Scene Segment doesn't have GT instance
		if (createSegmentGT)
		{
			if (!segmentTP)
			{
				segmentGT.Element[iScene*nDominantClusters + iSCluster].iScene = iScene;
				segmentGT.Element[iScene*nDominantClusters + iSCluster].iSSegment = iSCluster;
				segmentGT.Element[iScene*nDominantClusters + iSCluster].iModel = -1;
				segmentGT.Element[iScene*nDominantClusters + iSCluster].iMSegment = -1;
			}
		}

		if (clusters.n < 10)
			printf("\b");
		else
			printf("\b\b");

		if (iSCluster < 9)
		{
			printf("\b\b");
		}
		else
			printf("\b\b\b");

	}	// for all dominant clusters

	printf("completed.\n");

	int nSMI = iMIS;

	RVL_DELETE_ARRAY(iConsensusTemp.Element);

	RVL_DELETE_ARRAY(iValid.Element);

	RVL_DELETE_ARRAY(iConsensus.Element);

	RVL_DELETE_ARRAY(iRansacCandidates.Element);

	RVL_DELETE_ARRAY(iValidSampleCandidate.Element);

#ifdef PSGM_CALCULATE_PROBABILITY

	//calculate p(Mi|d) probability
	//float sigma = 1; //POSTAVITI KAO VANJSKI PARAMETAR?

	//float tConst = 1 / (2 * sigma);
	float tConst = 0.5;

	int nModels = 35;
	int nMSegments = 3;

	float maxCost = 412.5;

	int maxMSegments = nModels * nMSegments;

	int iSMI, iModel, iMSegment, j;

	float probability;

	int nMSCTI;

	Array<float> SMIminE;
	SMIminE.Element = new float[nSMI];

	for (i = 0; i < nSMI; i++)	
		SMIminE.Element[i] = maxCost;

	Array<Array<RECOG::PSGM_::MatchInstance*>> DBMBestMatches;
	DBMBestMatches.Element = new Array<RECOG::PSGM_::MatchInstance*>[nSMI];

	for (i = 0; i < nSMI; i++)
		DBMBestMatches.Element[i].Element = new RECOG::PSGM_::MatchInstance*[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			DBMBestMatches.Element[i].Element[j] = NULL;

	RECOG::PSGM_::MatchInstance *pDBMBestMatches;

	Array<Array<float>> DBMminE;
	DBMminE.Element = new Array<float>[nSMI];
		
	for (i = 0; i < nSMI; i++)
		DBMminE.Element[i].Element = new float[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			DBMminE.Element[i].Element[j] = maxCost;

	Array<Array<float>> SMIProbability1;

	SMIProbability1.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		SMIProbability1.Element[i].Element = new float[maxMSegments];

	Array<Array<float>> SMIProbability2;

	SMIProbability2.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		SMIProbability2.Element[i].Element = new float[maxMSegments];

	Array<Array<int>> nCTI;

	nCTI.Element = new Array<int>[nSMI];

	for (i = 0; i < nSMI; i++)
		nCTI.Element[i].Element = new int[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			nCTI.Element[i].Element[j] = 0.0;

	Array<Array<float>> Msum_;
	Msum_.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		Msum_.Element[i].Element = new float[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			Msum_.Element[i].Element[j] = 0.0;	

	Array<float> SMIsum;
	SMIsum.Element = new float[nSMI];

	Array<float> SMIsum2;
	SMIsum2.Element = new float[nSMI];

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 1 (DEBUG)!\n");

	//find min score for all SMI and for all DB model segments 
	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;
			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			if (pSMIMatch->E < SMIminE.Element[iSMI])
				SMIminE.Element[iSMI] = pSMIMatch->E;

			if (pSMIMatch->E < DBMminE.Element[iSMI].Element[iMSegment])
			{
				DBMminE.Element[iSMI].Element[iMSegment] = pSMIMatch->E;
				DBMBestMatches.Element[iSMI].Element[iMSegment] = pSMIMatch;
			}

			pSMIMatch = pSMIMatch->pNext;			
		}
		else
		{
			break;
		}		
	}

	for (i = 0; i < nSMI; i++)
		SMIsum.Element[i] = 0.0;

	printf("Probability calculation - STEP 2 (DEBUG)!\n");

	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				iSMI = pSMIMatch->iSMI;

				iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

				//SMIProbability1.Element[iSMI].Element[iMSegment] = exp(tConst*(SMIminE.Element[iSMI] * SMIminE.Element[iSMI] - pSMIMatch->E * pSMIMatch->E));
				SMIProbability1.Element[iSMI].Element[iMSegment] = exp(tConst*(SMIminE.Element[iSMI] - pSMIMatch->E));

				SMIsum.Element[iSMI] += SMIProbability1.Element[iSMI].Element[iMSegment];
			}
		}
	}

	printf("Probability calculation - STEP 3 (DEBUG)!\n");

	float tempSum = 0;

	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				iSMI = pSMIMatch->iSMI;

				iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

				probability = SMIProbability1.Element[iSMI].Element[iMSegment];

				pSMIMatch->probability1 = probability / SMIsum.Element[iSMI];

				tempSum += pSMIMatch->probability1;
			}
		}

		if (iSMI == 91)
			printf("\n\nTempSum P1: %f\n\n", tempSum);

		tempSum = 0;
	}

	//reset SMIminE
	//for (i = 0; i < nSMI; i++)
	//	SMIminE.Element[i] = 66.0;

	for (i = 0; i < nSMI; i++)
		SMIsum.Element[i] = 0.0;

	//pSMIMatch = SMImatches.pFirst;

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 4 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			nCTI.Element[iSMI].Element[iMSegment]++;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}


	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 5 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			probability = exp(tConst*(SMIminE.Element[iSMI] - pSMIMatch->E));

			nMSCTI = nCTI.Element[iSMI].Element[iMSegment];

			Msum_.Element[iSMI].Element[iMSegment] += probability / nMSCTI;

			SMIsum.Element[iSMI] += probability / nMSCTI;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}

	//calculate probability2
	printf("Probability calculation - STEP 6 (DEBUG)!\n");

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			if (SMIsum.Element[iSMI] > 0.0)
				pSMIMatch->probability2 = Msum_.Element[iSMI].Element[iMSegment] / SMIsum.Element[iSMI];
			else
				pSMIMatch->probability2 = 0.0;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}

	//find best matches for each scene segment
	QList<RECOG::PSGM_::MatchInstance> *pSSegmentMatches1 = &SSegmentMatches1;
	QList<RECOG::PSGM_::MatchInstance> *pSSegmentMatches2 = &SSegmentMatches2;

	RECOG::PSGM_::MatchInstance *pMatch;
	RECOG::PSGM_::MatchInstance *pNewMatch;

	bool newMatch = true;

	//if (iScene == 0)
	//{
	RVLQLIST_INIT(pSSegmentMatches1);
	RVLQLIST_INIT(pSSegmentMatches2);
	//}

	printf("Probability calculation - STEP 7 (DEBUG)!\n");

	//Probability1
	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				if (pSMIMatch->iScene == iScene)
				{
					iSMI = pSMIMatch->iSMI;

					iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

					pMatch = pSSegmentMatches1->pFirst;

					while (pMatch)
					{
						if (pMatch->iScene == pSMIMatch->iScene && pMatch->iCluster == pSMIMatch->iCluster && pMatch->iModel == pSMIMatch->iModel && pMatch->iMCluster == pSMIMatch->iMCluster)
						{
							//change pMatch in QList
							if (pSMIMatch->probability1 > pMatch->probability1)
							{
								pMatch->iCRF = pSMIMatch->iCRF;

								pMatch->iSMI = pSMIMatch->iSMI;

								pMatch->iMMI = pSMIMatch->iMMI;

								for (i = 0; i < 9; i++)
									pMatch->R[i] = pSMIMatch->R[i];

								for (i = 0; i < 3; i++)
								{
									pMatch->t[i] = pSMIMatch->t[i];
									pMatch->tMatch[i] = pSMIMatch->tMatch[i];
								}

								pMatch->E = pSMIMatch->E;

								pMatch->score = pSMIMatch->score;

								pMatch->probability1 = pSMIMatch->probability1;

								pMatch->probability2 = pSMIMatch->probability2;

								pMatch->angle = pSMIMatch->angle;

								pMatch->distance = pSMIMatch->distance;
							}

							newMatch = false;

							break;
						}

						pMatch = pMatch->pNext;
					}

					if (newMatch)
					{
						//Add new match to QList
						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pNewMatch);

						RVLQLIST_ADD_ENTRY(pSSegmentMatches1, pNewMatch);

						pNewMatch->iScene = pSMIMatch->iScene;

						pNewMatch->iCluster = pSMIMatch->iCluster;

						pNewMatch->iCRF = pSMIMatch->iCRF;

						pNewMatch->iSMI = pSMIMatch->iSMI;

						pNewMatch->iModel = pSMIMatch->iModel;

						pNewMatch->iMCluster = pSMIMatch->iMCluster;

						pNewMatch->iMMI = pSMIMatch->iMMI;

						for (i = 0; i < 9; i++)
							pNewMatch->R[i] = pSMIMatch->R[i];

						for (i = 0; i < 3; i++)
						{
							pNewMatch->t[i] = pSMIMatch->t[i];
							pNewMatch->tMatch[i] = pSMIMatch->tMatch[i];
						}

						pNewMatch->E = pSMIMatch->E;

						pNewMatch->score = pSMIMatch->score;

						pNewMatch->probability1 = pSMIMatch->probability1;

						pNewMatch->probability2 = pSMIMatch->probability2;

						pNewMatch->angle = pSMIMatch->angle;

						pNewMatch->distance = pSMIMatch->distance;
					}

					newMatch = true;
				}
			}
		}
	}

	//Probability2
	newMatch = true;

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 8 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			pMatch = pSSegmentMatches2->pFirst;

			while (pMatch)
			{
				if (pMatch->iScene == pSMIMatch->iScene && pMatch->iCluster == pSMIMatch->iCluster && pMatch->iModel == pSMIMatch->iModel && pMatch->iMCluster == pSMIMatch->iMCluster)
				{
					//change pMatch in QList
					if ((pSMIMatch->probability2 > pMatch->probability2) || (pSMIMatch->probability2 == pMatch->probability2 && pSMIMatch->E < pMatch->E))
					{
						pMatch->iCRF = pSMIMatch->iCRF;

						pMatch->iSMI = pSMIMatch->iSMI;

						pMatch->iMMI = pSMIMatch->iMMI;

						for (i = 0; i < 9; i++)
							pMatch->R[i] = pSMIMatch->R[i];

						for (i = 0; i < 3; i++)
						{
							pMatch->t[i] = pSMIMatch->t[i];
							pMatch->tMatch[i] = pSMIMatch->tMatch[i];
						}

						pMatch->E = pSMIMatch->E;

						pMatch->score = pSMIMatch->score;

						pMatch->probability1 = pSMIMatch->probability1;

						pMatch->probability2 = pSMIMatch->probability2;

						pMatch->angle = pSMIMatch->angle;

						pMatch->distance = pSMIMatch->distance;
					}

					newMatch = false;

					break;
				}

				pMatch = pMatch->pNext;
			}

			if (newMatch)
			{
				//Add new match to QList
				RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pNewMatch);

				RVLQLIST_ADD_ENTRY(pSSegmentMatches2, pNewMatch);

				pNewMatch->iScene = pSMIMatch->iScene;

				pNewMatch->iCluster = pSMIMatch->iCluster;

				pNewMatch->iCRF = pSMIMatch->iCRF;

				pNewMatch->iSMI = pSMIMatch->iSMI;

				pNewMatch->iModel = pSMIMatch->iModel;

				pNewMatch->iMCluster = pSMIMatch->iMCluster;

				pNewMatch->iMMI = pSMIMatch->iMMI;

				for (i = 0; i < 9; i++)
					pNewMatch->R[i] = pSMIMatch->R[i];

				for (i = 0; i < 3; i++)
				{
					pNewMatch->t[i] = pSMIMatch->t[i];
					pNewMatch->tMatch[i] = pSMIMatch->tMatch[i];
				}

				pNewMatch->E = pSMIMatch->E;

				pNewMatch->score = pSMIMatch->score;

				pNewMatch->probability1 = pSMIMatch->probability1;

				pNewMatch->probability2 = pSMIMatch->probability2;

				pNewMatch->angle = pSMIMatch->angle;

				pNewMatch->distance = pSMIMatch->distance;
			}

			newMatch = true;

			//pCurrentSceneMatch = pSMIMatch;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}			

	RVL_DELETE_ARRAY(SMIminE.Element);

	RVL_DELETE_ARRAY(DBMBestMatches.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(DBMminE.Element[i].Element);

	RVL_DELETE_ARRAY(DBMminE.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(SMIProbability1.Element[i].Element);

	RVL_DELETE_ARRAY(SMIProbability1.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(SMIProbability2.Element[i].Element);

	RVL_DELETE_ARRAY(SMIProbability2.Element);

	//RVL_DELETE_ARRAY(Msum.Element);

	RVL_DELETE_ARRAY(SMIsum.Element);

	RVL_DELETE_ARRAY(SMIsum2.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(Msum_.Element[i].Element);

	RVL_DELETE_ARRAY(Msum_.Element);

#endif

	iScene++;

	//fclose(fp);

	//reset GT matches flag
	pECCVGT->ResetMatchFlag();

	SortMatchMatrix();

#ifdef RVLPSGM_SAVE_MATCHES
	printf("Saving matches to txt file...");
	SaveMatches();
	printf("completed!\n\n");
#endif
}
#endif

void PSGM::Match()
{
	printf("Scene to model match started...");

	matchID = 0;

	pCTImatches = &CTImatches;
	RVLQLIST_INIT(pCTImatches);

	float csMinSampleAngleDiff = cos(PI / 4);

	int iSRF;
	//int i;

	//find sample candidates
	QList<QLIST::Index> iSampleCandidateList;
	pISampleCandidateList = &iSampleCandidateList;

	RVLQLIST_INIT(pISampleCandidateList);

	QLIST::Index *pNewISampleCandidate;
	RECOG::PSGM_::Plane *pPlane = convexTemplate.Element;

	for (int idx = 0; idx < convexTemplate.n; idx++)
	{
		if (RVLABS(pPlane->N[2]) <= csMinSampleAngleDiff)
		{
			RVLMEM_ALLOC_STRUCT(pMem, QLIST::Index, pNewISampleCandidate);

			RVLQLIST_ADD_ENTRY(pISampleCandidateList, pNewISampleCandidate);

			pNewISampleCandidate->Idx = idx;
		}

		pPlane++;
	}

	int iMIS;
	int iSCluster;
	//int iSClusterMI;

	int nClusters = CTISet.maxSegmentIdx + 1;	

	RECOG::PSGM_::ModelInstance *pSModelInstance;
	RECOG::PSGM_::ModelInstance *pMModelInstance;

	int iSCTI, nCTI;

	int iMSegment;

	int maxMSegments = (MCTISet.nModels + 1) * (MCTISet.maxSegmentIdx + 1);

	//delete scoreMatchMatrix	
	for (iSCluster = 0; iSCluster < scoreMatchMatrix.n; iSCluster++)
		RVL_DELETE_ARRAY(scoreMatchMatrix.Element[iSCluster].Element);

	RVL_DELETE_ARRAY(scoreMatchMatrix.Element);

	scoreMatchMatrix.Element = new Array<SortIndex<float>>[nClusters];
	scoreMatchMatrix.n = nClusters;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		scoreMatchMatrix.Element[iSCluster].Element = new SortIndex<float>[maxMSegments];
		scoreMatchMatrix.Element[iSCluster].n = maxMSegments;

		for (iMSegment = 0; iMSegment < maxMSegments; iMSegment++)
		{
			scoreMatchMatrix.Element[iSCluster].Element[iMSegment].cost = 10000; //MAX COST!!!

			scoreMatchMatrix.Element[iSCluster].Element[iMSegment].idx = -1;
		}
	}

	//used in Match(RECOG::PSGM_::ModelInstance *pSModelInstance, int startIdx, int endIdx);
	nTc = new float[3 * convexTemplate.n];
	dISMc = new float[convexTemplate.n];

	int startIdx = 0, endIdx = MCTISet.pCTI.n;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		printf("%d/%d", iSCluster + 1, nClusters);
	
		nCTI = CTISet.SegmentCTIs.Element[iSCluster].n;

		for (iSCTI = 0; iSCTI < nCTI; iSCTI++)
		{
			CTIIdx = CTISet.SegmentCTIs.Element[iSCluster].Element[iSCTI];

			pSModelInstance = CTISet.pCTI.Element[CTIIdx];	

			Match(pSModelInstance, startIdx, endIdx);

			CalculateScore(RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SATURATED_SQUARE_DISTANCE);

			UpdateScoreMatchMatrix(pSModelInstance);

		} // for all MI in cluster
		
		if (nClusters < 10)
			printf("\b");
		else
			printf("\b\b");

		if (iSCluster < 9)
		{
			printf("\b\b");
		}
		else
			printf("\b\b\b");

	}	// for all dominant clusters

	pCTImatchesArray.n = CTISet.pCTI.n * MCTISet.pCTI.n;

	RVL_DELETE_ARRAY(pCTImatchesArray.Element);

	pCTImatchesArray.Element = new RECOG::PSGM_::MatchInstance*[pCTImatchesArray.n];

	QLIST::CreatePtrArray<RECOG::PSGM_::MatchInstance>(pCTImatches, &pCTImatchesArray);

	SortScoreMatchMatrix();

	////for Visualization purposes:
	//RECOG::PSGM_::MatchInstance *pMatchx = pCTImatchesArray.Element[scoreMatchMatrix.Element[0].Element[0].idx];
	//VisualizeCTIMatchidx(pMatchx->iSCTI, pMatchx->iMCTI);
	////end of visualization

	printf("completed.\n");

	//int nSMI = iMIS;

	delete[] nTc;
	delete[] dISMc;

	iScene++;

#ifdef RVLPSGM_SAVE_MATCHES
	printf("Saving matches to txt file...");
	SaveMatches();
	printf("completed!\n\n");
#endif
}

void PSGM::Match(
	RECOG::PSGM_::ModelInstance *pSModelInstance,
	int startIdx,
	int endIdx)
{
	float cos45 = cos(PI / 4);
	float csMinSampleAngleDiff = cos(PI / 4);
	float minE, minETotal, E, score, SMI_minETotal;

	float SMI_tBestMatch[3];

	Eigen::Matrix3f A;
	Eigen::Vector3f B, t;

	float sigma = 8; //!!!!
	float sigma25 = 2.5*2.5;

	int i, idx;

	float dISv, dIMvt;

	RECOG::PSGM_::ModelInstance *pMModelInstance;

	int iMCTI;

	bool TP_;

	float distanceThresh = 50;

	float R_[9], t_[3];

	int nSamples;

	for (iMCTI = startIdx; iMCTI < endIdx; iMCTI++)
	{
		pMModelInstance = MCTISet.pCTI.Element[iMCTI];

		//minE = 66.0;
		minE = 412.5; // for sigma = 2.5^2

		//CTDMatchRANSAC
		float pPrior = 2.5 * stdNoise;

		int nValidSampleCandidates = 0;

		//find valid sample candidates
		iValidSampleCandidate.n = 0;

		QLIST::Index *piValidSampleCandidate = iValidSampleCandidate.Element;

		QLIST::Index *pISampleCandidate = pISampleCandidateList->pFirst;

		while (pISampleCandidate)
		{
			if (pSModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true && pMModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true)
			{
				piValidSampleCandidate->Idx = pISampleCandidate->Idx;

				piValidSampleCandidate->pNext = piValidSampleCandidate + 1;

				piValidSampleCandidate++;

				nValidSampleCandidates++;
			}

			pISampleCandidate = pISampleCandidate->pNext;
		}

		iValidSampleCandidate.n = nValidSampleCandidates;

		if (nValidSampleCandidates > 2)
		{
			int nValids = 0;

			//find iValid
			iValid.n = 0;

			QLIST::Index *piValid = iValid.Element;
			int iPlane;

			for (iPlane = 0; iPlane < convexTemplate.n; iPlane++)
			{
				if (pSModelInstance->modelInstance.Element[iPlane].valid == true && pMModelInstance->modelInstance.Element[iPlane].valid == true)
				{
					piValid->Idx = iPlane;

					piValid->pNext = piValid + 1;

					piValid++;

					nValids++;
				}
			}

			iValid.n = nValids;

			int iRansac;
			bool bValidSample;

			int iSample[2];
			int ID[2];
			float V[3];
			float N_[3] = { 0, 0, 1 };
			float N__[3] = { 0, -cos45, cos45 };
			float fTmp;
			int nValidSamplesSearch;
			int nRansacCandidates = 0;

			iConsensus.n = 0;

			QLIST::Index *piConsensus = iConsensus.Element;

			if (bMatchRANSAC)
			{
				iRansacCandidates.n = 0;

				QLIST::Index *piRansacCandidates = iRansacCandidates.Element;

				//find RANSAC candidates
				RVLCROSSPRODUCT3(N_, N__, V);

				fTmp = sqrt(RVLDOTPRODUCT3(V, V));

				RVLSCALE3VECTOR2(V, fTmp, V);

				for (i = 0; i < iValidSampleCandidate.n; i++)
				{
					ID[0] = iValidSampleCandidate.Element[i].Idx;

					fTmp = RVLDOTPRODUCT3(V, convexTemplate.Element[ID[0]].N);

					if (RVLABS(fTmp) >= csMinSampleAngleDiff)
					{
						piRansacCandidates->Idx = ID[0];

						piRansacCandidates++;

						nRansacCandidates++;
					}
				}

				iRansacCandidates.n = nRansacCandidates;

				std::random_device rd;
				std::mt19937 eng(rd());
				std::uniform_int_distribution<> distribution(0, nRansacCandidates);

				nSamples = RVLMIN(13, nRansacCandidates);

				for (iRansac = 0; iRansac < nSamples; iRansac++)
				{
					ID[0] = 1; //second plane from convexTemplate

					iSample[1] = distribution(eng);

					ID[1] = iValidSampleCandidate.Element[iSample[1]].Idx;

					float dM[3];
					float dS[3];
					float N[9];

					dM[0] = pMModelInstance->modelInstance.Element[0].d;
					dM[1] = pMModelInstance->modelInstance.Element[ID[0]].d;
					dM[2] = pMModelInstance->modelInstance.Element[ID[1]].d;

					dS[0] = pSModelInstance->modelInstance.Element[0].d * 1000;
					dS[1] = pSModelInstance->modelInstance.Element[ID[0]].d * 1000;
					dS[2] = pSModelInstance->modelInstance.Element[ID[1]].d * 1000;

					RVLCOPYTOCOL3(convexTemplate.Element[0].N, 0, N);
					RVLCOPYTOCOL3(convexTemplate.Element[ID[0]].N, 1, N);
					RVLCOPYTOCOL3(convexTemplate.Element[ID[1]].N, 2, N);

					A << N[0], N[3], N[6], N[1], N[4], N[7], N[2], N[5], N[8]; // N'
					B << dS[0] - dM[0], dS[1] - dM[1], dS[2] - dM[2];
					t = A.colPivHouseholderQr().solve(B);

					iConsensusTemp.n = 0;

					QLIST::Index *piConsensusTemp = iConsensusTemp.Element;

					E = 0;

					for (i = 0; i < iValid.n; i++)
					{
						idx = iValid.Element[i].Idx;

						dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

						dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

						//fTmp = (dISv - dIMvt) / pPrior;
						fTmp = (dISv - dIMvt) / sigma;

#ifdef RVLPSGM_MATCH_SATURATION
						if (fTmp*fTmp < sigma25)
							//if (fTmp*fTmp < 1)
						{
							E += fTmp*fTmp;

							piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
							piConsensusTemp->pNext = piConsensusTemp + 1;

							iConsensusTemp.n++;

							piConsensusTemp++;
						}
						else
							E += sigma25;
						//E += 1;
#else
						E += fTmp*fTmp;

						//TREBA LI OVO?
						piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
						piConsensusTemp->pNext = piConsensusTemp + 1;

						iConsensusTemp.n++;

						piConsensusTemp++;
#endif
					}

					if (E < minE)
					{
						minE = E;

						iConsensus.n = iConsensusTemp.n;

						piConsensusTemp = iConsensusTemp.Element;

						piConsensus = iConsensus.Element;

						for (i = 0; i < iConsensusTemp.n; i++)
						{
							piConsensus->Idx = piConsensusTemp->Idx;
							piConsensus->pNext = piConsensus + 1;

							piConsensus++;
							piConsensusTemp++;
						}
					}
				}
			}
			else
				iConsensus.n = iValid.n;

			if (iConsensus.n >= 3)
			{
				float dISc, dIMc;

				piConsensus = iConsensus.Element;

				for (i = 0; i < iConsensus.n; i++)
				{
#ifdef RVLPSGM_RANSAC
					idx = piConsensus->Idx;
#else
					idx = iValid.Element[i].Idx;
#endif
					dISc = pSModelInstance->modelInstance.Element[idx].d * 1000;
					dIMc = pMModelInstance->modelInstance.Element[idx].d;

					dISMc[i] = dISc - dIMc;

					nTc[i] = convexTemplate.Element[idx].N[0];
					nTc[iConsensus.n + i] = convexTemplate.Element[idx].N[1];
					nTc[2 * iConsensus.n + i] = convexTemplate.Element[idx].N[2];

					piConsensus++;
				}

				int j, k;

				//nTc*nTc'
				for (i = 0; i < 3; i++)
					for (j = 0; j < 3; j++)
						if (i <= j)
						{
							A(i * 3 + j) = 0;

							for (k = 0; k < iConsensus.n; k++)
								A(i * 3 + j) += nTc[i * iConsensus.n + k] * nTc[j * iConsensus.n + k];
						}
						else
							A(i * 3 + j) = A(j * 3 + i);

				//nTc*(dISc-dIMc)
				for (i = 0; i < 3; i++)
				{
					B(i) = 0;
					for (j = 0; j < iConsensus.n; j++)
						B(i) += nTc[i * iConsensus.n + j] * dISMc[j];
				}				
				
				t = A.colPivHouseholderQr().solve(B);							

				E = 0;

				float eSum = 0;

				for (i = 0; i < iValid.n; i++)
				{
					idx = iValid.Element[i].Idx;

					dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

					dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

					e.Element[iMCTI].Element[idx] = dISv - dIMvt;

					eSum += e.Element[iMCTI].Element[idx];
				}

				RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pCTIMatch);
				RVLQLIST_ADD_ENTRY(pCTImatches, pCTIMatch);

				if (iMCTI == startIdx)
					pFirstSCTIMatch = pCTIMatch;

				for (i = 0; i < 3; i++)
				{
					tBestMatch.Element[iMCTI].Element[i] = t(i);
					pCTIMatch->tMatch[i] = t(i);
				}				

				pCTIMatch->ID = matchID++;
				pCTIMatch->iScene = iScene;

				pCTIMatch->iSCTI = CTIIdx;
				pCTIMatch->iMCTI = iMCTI;

				pCTIMatch->E = eSum;
				pCTIMatch->nValids = iValid.n;

				pCTIMatch->probability1 = NAN;
				pCTIMatch->probability2 = NAN;

				pCTIMatch->angleGT = NAN;
				pCTIMatch->distanceGT = NAN;



				//MSTransformation(pMModelInstance, pSModelInstance, tBestMatch.Element[iMCTI].Element, R_, t_);
			}
			else
			{
				t << 0, 0, 0;
				//E = 66;
				E = 412.5;
			}
		}
	}	//for all model MI
}

void PSGM::CalculateScore(int similarityMeasure)
{
	float sigma = 8.0;

	float sigma25 = 2.5 * 2.5;

	int iMCTI, iValidPlane, idx;

	int nMCTI = MCTISet.pCTI.n;

	float fTmp, eTmp, scoreTmp;

	float maxError;

	RECOG::PSGM_::MatchInstance *pCTIMatch_ = pFirstSCTIMatch;

	switch (similarityMeasure)
	{
	case RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SQUARE_DISTANCE: //mean square error

		for (iMCTI = 0; iMCTI < nMCTI; iMCTI++)
		{
			scoreTmp = 0;

			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				scoreTmp += eTmp * eTmp;
			}

			score.Element[iMCTI] = sqrt(scoreTmp / iValid.n);

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		break;
	case RVLPSGM_MATCH_SIMILARITY_MEASURE_MAX_ABS_DISTANCE: //maximum absolute error

		for (iMCTI = 0; iMCTI < nMCTI; iMCTI++)
		{
			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				fTmp = RVLABS(eTmp);

				if (iValidPlane == 0)
					maxError = fTmp;
				else
					if (fTmp > maxError)
						maxError = fTmp;
			}

			score.Element[iMCTI] = maxError;

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		break;
	case RVLPSGM_MATCH_SIMILARITY_MEASURE_SATURATED_SQUARE_DISTANCE_INVISIBILITY_PENAL: //saturated square error

		for (iMCTI = 0; iMCTI < nMCTI; iMCTI++)
		{
			scoreTmp = 0;

			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				fTmp = eTmp / sigma;

				if (fTmp * fTmp < sigma25)
				{
					scoreTmp += fTmp * fTmp;
				}
				else
					scoreTmp += sigma25;
			}

			score.Element[iMCTI] = scoreTmp + sigma25 * (66 - iValid.n);

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		break;
	case RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SATURATED_SQUARE_DISTANCE:
		for (iMCTI = 0; iMCTI < nMCTI; iMCTI++)
		{
			scoreTmp = 0;

			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				fTmp = eTmp / sigma;

				if (fTmp * fTmp < sigma25)
				{
					scoreTmp += fTmp * fTmp;
				}
				else
					scoreTmp += sigma25;
			}

			score.Element[iMCTI] = scoreTmp / iValid.n;

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		break;
	default: //median of absolute error

		Array<SortIndex<float>> validErrors;

		validErrors.Element = new SortIndex<float>[iValid.n];
		validErrors.n = iValid.n;

		int medianIdx = iValid.n / 2;

		for (iMCTI = 0; iMCTI < nMCTI; iMCTI++)
		{
			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				fTmp = RVLABS(eTmp);

				validErrors.Element[iValidPlane].cost = fTmp;
				validErrors.Element[iValidPlane].idx = iValidPlane;
			}

			BubbleSort(validErrors);

			if (iValid.n % 2 == 0)
				scoreTmp = (validErrors.Element[medianIdx - 1].cost + validErrors.Element[medianIdx].cost) / 2;
			else
				scoreTmp = validErrors.Element[medianIdx].cost;
			
			score.Element[iMCTI] = scoreTmp;

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		RVL_DELETE_ARRAY(validErrors.Element);

		break;
	}
}

void PSGM::UpdateScoreMatchMatrix(RECOG::PSGM_::ModelInstance *pSModelInstance)
{
	int SSegmentIdx = pSModelInstance->iCluster;

	int MSegmentIdx, iMCTI;

	float scoreTmp, scoreTmp_;

	int idx;

	RECOG::PSGM_::MatchInstance *pCTIMatch_ = pFirstSCTIMatch;

	for (iMCTI = 0; iMCTI < MCTISet.pCTI.n; iMCTI++)
	{
		scoreTmp = score.Element[iMCTI];

		MSegmentIdx = MCTISet.pCTI.Element[iMCTI]->iModel * (MCTISet.maxSegmentIdx + 1) + MCTISet.pCTI.Element[iMCTI]->iCluster;

		scoreTmp_ = scoreMatchMatrix.Element[SSegmentIdx].Element[MSegmentIdx].cost;

		idx = scoreMatchMatrix.Element[SSegmentIdx].Element[MSegmentIdx].idx;

		if (scoreTmp < scoreTmp_ || idx == -1)
		{
			scoreMatchMatrix.Element[SSegmentIdx].Element[MSegmentIdx].cost = scoreTmp;

			scoreMatchMatrix.Element[SSegmentIdx].Element[MSegmentIdx].idx = pCTIMatch_->ID;
		}

		pCTIMatch_ = pCTIMatch_->pNext;
	}
}

void PSGM::SortScoreMatchMatrix(bool descending)
{
	int nSSegments = scoreMatchMatrix.n;

	int iSSegment;

	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
	{
		BubbleSort<SortIndex<float>>(scoreMatchMatrix.Element[iSSegment], descending);
	}
}

void PSGM::CreateScoreMatchMatrixICP()
{
	int nClusters = CTISet.maxSegmentIdx + 1;

	int maxMSegments = (MCTISet.nModels + 1) * (MCTISet.maxSegmentIdx + 1);

	int iSCluster, iMSegment;

	//delete scoreMatchMatrix	
	for (iSCluster = 0; iSCluster < scoreMatchMatrixICP.n; iSCluster++)
		RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element[iSCluster].Element);

	RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element);

	scoreMatchMatrixICP.Element = new Array<SortIndex<float>>[nClusters];
	scoreMatchMatrixICP.n = nClusters;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		scoreMatchMatrixICP.Element[iSCluster].Element = new SortIndex<float>[maxMSegments];
		scoreMatchMatrixICP.Element[iSCluster].n = maxMSegments;

		for (iMSegment = 0; iMSegment < maxMSegments; iMSegment++)
		{
			scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].cost = 10000; //MAX COST!!!

			scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].idx = -1;
		}
	}

	int iMatch;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		for (iMSegment = 0; iMSegment < nBestMatches; iMSegment++)
		{
			iMatch = scoreMatchMatrix.Element[iSCluster].Element[iMSegment].idx;

			if (iMatch != -1)
			{
				scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].cost = pCTImatchesArray.Element[iMatch]->cost_NN;

				scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].idx = iMatch;
			}
			else
				break;
		}
	}

	//sort ICP matrix
	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		BubbleSort<SortIndex<float>>(scoreMatchMatrixICP.Element[iSCluster], false);
	}

}

void PSGM::ComputeClusterNormalDistribution(
	RECOG::PSGM_::Cluster *pCluster)
{
	float R[9];

	float *X = R;
	float *Y = R + 3;
	float *Z = R + 6;

	float *meanN = Z;

	RVLNULL3VECTOR(meanN);

	float wTotal = 0.0f;

	int iiSurfel, iSurfel;
	Surfel *pSurfel;
	float *N;
	float wN[3];
	float w;
	float fTmp;
	float NP[3];
	float eig[2];
	int i1, i2, i3;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		N = pSurfel->N;

		w = (float)(pSurfel->size);

		RVLSCALE3VECTOR(N, w, wN);

		RVLSUM3VECTORS(meanN, wN, meanN);

		wTotal += w;
	}

	RVLSCALE3VECTOR2(meanN, wTotal, meanN);

	// Define projection reference frame.

	RVLORTHOGONAL3(Z, X, i1, i2, i3, fTmp);

	RVLCROSSPRODUCT3(Z, X, Y);

	// Project surfel normals onto the xy-plane of the projection reference frame and compute covariance matrix.

	float C[4];

	C[0] = C[1] = C[3] = 0.0f;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		N = pSurfel->N;

		w = (float)(pSurfel->size);

		RVLMULMX3X3VECT(R, N, NP);

		C[0] += (w * NP[0] * NP[0]);
		C[1] += (w * NP[0] * NP[1]);
		C[3] += (w * NP[1] * NP[1]);
	}

	C[0] /= wTotal;
	C[1] /= wTotal;
	C[2] = C[1];
	C[3] /= wTotal;

	// Compute eigenvalues of C.

	Eig2<float>(C, eig);

	// Compute normalDistributionStds.

	pCluster->normalDistributionStd1 = sqrt(eig[0]);
	pCluster->normalDistributionStd2 = sqrt(eig[1]);
	RVLCOPY3VECTOR(meanN, pCluster->N);
}

void PSGM::ComputeClusterBoundaryDiscontinuityPerc(int iCluster)
{
	RECOG::PSGM_::Cluster *pCluster = clusterMem + iCluster;

	int nContinuity = 0;
	int nDiscontinuity = 0;

	int iCluster_, iiSurfel, iSurfel, iSurfel_, iPt, iPt_, iBoundary, iPointEdge;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_;
	Point *pPt;
	MeshEdge *pEdge;
	RECOG::PSGM_::Cluster *pCluster_;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				if (pPt->bBoundary)
					nDiscontinuity++;
				else
				{
					pEdgePtr_ = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

					while (pEdgePtr_)
					{
						RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr_, pEdge, iPt_);

						iSurfel_ = pSurfels->surfelMap[iPt_];

						if (iSurfel_ >= 0 && iSurfel_ < pSurfels->NodeArray.n)
						{
							iCluster_ = clusterMap[iSurfel_];

							if (iCluster_ != iCluster && iCluster_ >= 0)
							{
								if (iCluster_ >= clusters.n)
									printf("iCluster_=%d\n", iCluster_);

								pCluster_ = clusterMem + iCluster_;

								if (pCluster_->bValid)
									break;
							}
						}

						pEdgePtr_ = pEdgePtr_->pNext;
					}

					if (pEdgePtr_)
						nContinuity++;
				}
			}
		}
	}

	if (nDiscontinuity == 0)
		pCluster->boundaryDiscontinuityPerc = 0;
	else
		pCluster->boundaryDiscontinuityPerc = 100 * nDiscontinuity / (nContinuity + nDiscontinuity);
}

void PSGM::WriteClusterNormalDistribution(FILE *fp)
{
	int iCluster;
	RECOG::PSGM_::Cluster *pCluster;

	for (iCluster = 0; iCluster < clusters.n; iCluster++)
	{
		pCluster = clusters.Element[iCluster];

		// Write eigenvalues to file.

		fprintf(fp, "%d\t%d\t%d\t%f\t%f\n", 
			iCluster, 
			pCluster->boundaryDiscontinuityPerc, 
			pCluster->size, 
			pCluster->normalDistributionStd1, 
			pCluster->normalDistributionStd2);
	}
}

void PSGM::MSTransformation(
	RECOG::PSGM_::ModelInstance *pMModelInstance,
	RECOG::PSGM_::ModelInstance *pSModelInstance,
	float *tBestMatch,
	float *R,
	float *t)
{
	float R_[9], t_[3];

	//RVLSCALE3VECTOR(pMModelInstance->t, 0, pMModelInstance->t);

	RVLINVTRANSF3D(pMModelInstance->R, pMModelInstance->t, R_, t_);

	RVLSUM3VECTORS(t_, tBestMatch, t_);

	RVLCOMPTRANSF3D(pSModelInstance->R, pSModelInstance->t, R_, t_, R, t);
}

//Compare single match to GT
bool PSGM::CompareMatchToGT(
	RECOG::PSGM_::MatchInstance *pMatch,
	//float score,
	//float scoreThresh,
	bool poseCheck,
	float angleThresh,
	float distanceThresh)
{
	RVL::GTInstance *pGT;

	int iGTS, iGTM, nGTModels;

	float R[9], R_[9], RGT[9], tGT[3], t[3];

	float V[3], theta, distance;

	iGTS = pMatch->iScene;

	int iMCTI, iMatchedModel;

	//TP
	pGT = pECCVGT->GT.Element[iGTS].Element;

	nGTModels = pECCVGT->GT.Element[iGTS].n;

	for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
	{
		iMCTI = pMatch->iMCTI;

		iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;

		if (iMatchedModel == pGT->iModel)
		{
			if (poseCheck)
			{
				RVLSCALEMX3X3(pGT->R, 1000, RGT);

				RVLMXMUL3X3T2(pMatch->R, RGT, R);

				RVLSCALE3VECTOR(pGT->t, 1000, tGT)

#ifdef RVLPSGM_MATCH_SEGMENT_CENTROID

				RVLDIFMX3X3(RGT, pMatch->R, R_);

				RVLMULMX3X3VECT(R_, modelInstanceDB.Element[pMatch->iMMI].tc, t);

				RVLSUM3VECTORS(t, tGT, t);

				RVLDIF3VECTORS(t, pMatch->t, t);
#else if
				RVLDIF3VECTORS(pMatch->t, tGT, t);
#endif

				GetAngleAxis(R, V, theta);

				GetDistance(t, distance);

				//if ((theta < angleThresh || (theta >(PI - angleThresh) && theta < (PI + angleThresh))) && distance < distanceThresh)
				if (distance < distanceThresh)
				{
					pGT->matched = true;
					return true;								
				}
			}
			else
			{
				pGT->matched = true;
				return true;							
			}
		}
	}

	//FP
	return false;
}

bool PSGM::CompareMatchToSegmentGT(
	RECOG::PSGM_::MatchInstance *pMatch)
{	
	int iSCTI = pMatch->iSCTI;

	int iSSegment = CTISet.pCTI.Element[iSCTI]->iCluster;

	int iScene = pMatch->iScene;

	int iSegmentGT = iScene * nDominantClusters + iSSegment;

	int nGTModels, iGTM;

	RVL::GTInstance *pGT;

	int iMCTI, iMatchedModel;

	pGT = pECCVGT->GT.Element[iScene].Element;
	nGTModels = pECCVGT->GT.Element[iScene].n;

	iMCTI = pMatch->iMCTI;

	iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;

	//if (pMatch->iModel == segmentGT.Element[iSegmentGT].iModel && pMatch->iMCluster == segmentGT.Element[iSegmentGT].iMSegment)
	if (iMatchedModel == segmentGT.Element[iSegmentGT].iModel)
	{
		//set GT matched flag
		for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
		{
			if (iMatchedModel == pGT->iModel)
				pGT->matched = true;
		}

		return true;
	}
	else
		return false;
}

bool PSGM::CompareMatchToSegmentGT(
	int iScene,
	int iSSegment,
	int iMatchedModel)
{
	int iSegmentGT = iScene * nDominantClusters + iSSegment;

	int nGTModels, iGTM;

	RVL::GTInstance *pGT;

	pGT = pECCVGT->GT.Element[iScene].Element;
	nGTModels = pECCVGT->GT.Element[iScene].n;

	if (iMatchedModel == segmentGT.Element[iSegmentGT].iModel)
	{
		//set GT matched flag
		for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
		{
			if (iMatchedModel == pGT->iModel)
				pGT->matched = true;
		}

		return true;
	}
	else
		return false;
}

void PSGM::CountTPandFN(
	int &TP,
	int &FN,
	bool printMatchInfo)
{
	int iGTS, iGTM, nGTModels;
	
	TP = 0;
	FN = 0;

	RVL::GTInstance *pGT;

	int nGTSecenes = pECCVGT->GT.n;

	pGT = pECCVGT->GT.Element[iScene-1].Element; //iScene-1 because iScene is incremented in Match()

	nGTModels = pECCVGT->GT.Element[iScene-1].n; //iScene-1 because iScene is incremented in Match()

	for (iGTM = 0; iGTM < nGTModels; iGTM++)
	{
		if (!pGT->matched)
		{
			FN++;

			if (printMatchInfo)
				printf("GT Model %d NOT matched on scene %d!\n", iGTM, iScene - 1);
		}
		else
		{
			TP++;

			if(printMatchInfo)
				printf("GT Model %d matched on scene %d!\n", iGTM, iScene - 1);
		}

		pGT++;
	}	
}

bool PSGM::PoseCheck(
	RVL::GTInstance *pGT,
	RECOG::PSGM_::MatchInstance *pMatch,
	float distanceThresh,
	float angleThresh,
	FILE *fpLog,
	FILE *fpnotFirstPoseErr,
	bool evaluateICP)
{
	float R[9], R_[9], RGT[9], tGT[3], t[3], RICP[9], tICP[3];
	float V[3], theta, distance;
	float zGT[3], z[3];
	float thetaZ;

	RVLSCALEMX3X3(pGT->R, 1000, RGT);
	RVLSCALE3VECTOR(pGT->t, 1000, tGT);	
	
	if (evaluateICP)
	{
		float Rpom[9], tpom[3], tmp[3];
		RVLCOMPTRANSF3DWITHINV(RGT, tGT, pMatch->RICP_, pMatch->tICP_, Rpom, tpom, tmp);
		RVLCOMPTRANSF3D(Rpom, tpom, pMatch->R, pMatch->t, pMatch->RICP, pMatch->tICP);

		RVLCOPYMX3X3(pMatch->RICP, R);
		RVLCOPY3VECTOR(pMatch->tICP, t);

	}
	else
	{
		RVLMXMUL3X3T2(RGT, pMatch->R, R);
		RVLDIF3VECTORS(tGT, pMatch->t, t);
	}

	

		/*
		#ifdef RVLPSGM_MATCH_SEGMENT_CENTROID

		RVLDIFMX3X3(RGT, pMatch->R, R_);

		RVLMULMX3X3VECT(R_, modelInstanceDB.Element[pMatch->iMMI].tc, t);

		RVLSUM3VECTORS(t, tGT, t);

		RVLDIF3VECTORS(t, pMatch->t, t);
		#else if
		RVLDIF3VECTORS(pMatch->t, tGT, t);
		#endif
		*/
		

	GetAngleAxis(R, V, theta);

	GetDistance(t, distance);

	zGT[0] = RGT[2];
	zGT[1] = RGT[5];
	zGT[2] = RGT[8];

	if (evaluateICP) //not anymore, because we do not have the apsolute pose, rather the relative
	{
		thetaZ = acos(R[9]);
	}
	else
	{
		z[0] = pMatch->R[2];
		z[1] = pMatch->R[5];
		z[2] = pMatch->R[8];
		thetaZ = RVLDOTPRODUCT3(zGT, z);
}

	//if ((theta < angleThresh || (theta >(PI - angleThresh) && theta < (PI + angleThresh))) && distance < distanceThresh)
	//if (distance < distanceThresh && theta < angleThresh)
	if (distance < distanceThresh && thetaZ > angleThresh)
	{
		return true;
	}
	else
	{
		if (fpLog)
		{
			//fprintf(fpLog, "%d\t%d\t%d\t%d\t%f\t%f\n", pMatch->iScene, MCTISet.pCTI.Element[pMatch->iMCTI]->iModel, MCTISet.pCTI.Element[pMatch->iMCTI]->iCluster, CTISet.pCTI.Element[pMatch->iSCTI]->iCluster, distance, theta);
			fprintf(fpLog, "%d\t%d\t%d\t%d\t%f\t%f\n", pMatch->iScene, MCTISet.pCTI.Element[pMatch->iMCTI]->iModel, MCTISet.pCTI.Element[pMatch->iMCTI]->iCluster, CTISet.pCTI.Element[pMatch->iSCTI]->iCluster, distance, acos(thetaZ) * 180 / PI);
		}

		return false;
	}
}

void PSGM::FindGTInstance(
	RVL::GTInstance **pGT,
	int iScene,
	int iModel)
{
	int nGTModels, iGTM;

	RVL::GTInstance *pGT_ = *pGT;

	pGT_ = pECCVGT->GT.Element[iScene].Element;

	nGTModels = pECCVGT->GT.Element[iScene].n;

	for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT_++)
		if (pGT_->iModel == iModel)
			break;

	*pGT = pGT_;
}

void PSGM::CalculatePR(int TP, int FP, int FN, float &precision, float &recall)
{
	if (TP + FP > 0)
		precision = (float)TP / (float)(TP + FP);
	else
		precision = 0.0;

	recall = (float)TP / (float)(TP + FN);

}

void PSGM::FindMinMaxInScoreMatchMatrix(float &min, float &max, Array<Array<SortIndex<float>>> &scoreMatchMatrix_)
{
	
	int nSSegments = scoreMatchMatrix_.n;

	int iSSegment;
	int iMSegment;
	float scoreTmp;
	int idx;

	//scoreMatchMatrix must be sorted
	min = scoreMatchMatrix_.Element[0].Element[0].cost;
	max = scoreMatchMatrix_.Element[0].Element[0].cost;

	for (iSSegment = 1; iSSegment < nSSegments; iSSegment++)
		if (scoreMatchMatrix_.Element[iSSegment].Element[0].cost < min)
			min = scoreMatchMatrix_.Element[iSSegment].Element[0].cost;

	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
		for (iMSegment = 0; iMSegment < nMSegments; iMSegment++)
		{
			scoreTmp = scoreMatchMatrix_.Element[iSSegment].Element[iMSegment].cost;

			idx = scoreMatchMatrix_.Element[iSSegment].Element[iMSegment].idx;

			if (scoreTmp > max && idx != -1)
				max = scoreTmp;
		}
}

void PSGM::EvaluateMatchesByScore(
	FILE *fp,
	FILE *fpLog,
	FILE *fpPoseError,
	FILE *fpnotFirstInfo,
	FILE *fpnotFirstPoseErr,
	int nBestSegments,
	bool evaluateICP)
{
	float precision, recall;

	int graphID = 0;

	float minScore, maxScore;

	int iSSegment;
	//int nSSegments = scoreMatchMatrix.n;
	int nSSegments = CTISet.SegmentCTIs.n;

	int iMSegment;
	int nMSegments = (MCTISet.nModels + 1) * (MCTISet.maxSegmentIdx + 1);

	float scoreTmp;

	int idx;

	float cos30 = sqrt(3) / 2;

	//scoreMatchMatrix is sorted
	/*
	minScore = scoreMatchMatrix.Element[0].Element[0].cost;
	maxScore = scoreMatchMatrix.Element[0].Element[0].cost;

	for (iSSegment = 1; iSSegment < nSSegments; iSSegment++)
		if (scoreMatchMatrix.Element[iSSegment].Element[0].cost < minScore)
			minScore = scoreMatchMatrix.Element[iSSegment].Element[0].cost;
	
	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
		for (iMSegment = 0; iMSegment < nMSegments; iMSegment++)
		{
			scoreTmp = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].cost;

			idx = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].idx;

			if (scoreTmp > maxScore && idx != -1)
				maxScore = scoreTmp;
		}
		*/

	if (evaluateICP)
	{
		CreateScoreMatchMatrixICP();
		FindMinMaxInScoreMatchMatrix(minScore, maxScore, scoreMatchMatrixICP);
	}
	else
		FindMinMaxInScoreMatchMatrix(minScore, maxScore, scoreMatchMatrix);

	int iScore, nScoreSteps = 200;

	float scoreThresh;

	float scoreStep = (maxScore - minScore) / nScoreSteps;

	bool TPMatch, poseMatch;

	int TP_ = 0, FP_ = 0, FN_ = 0;

	int iMatchedModel, iCTI;

	int iMatch;

	int *firstTP = new int[nDominantClusters];
	float *firstTPScore = new float[nDominantClusters];
	int *firstTPiModel = new int[nDominantClusters];

	int iBestMatches;

	int iMCTI, iSCTI;

	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstance *pMCTI;

	RVL::GTInstance *pGT = NULL;

	float R_[9], t_[3];

	if (nBestSegments == 0)
	{
		scoreThresh = minScore;

		for (iScore = 0; iScore < nScoreSteps; iScore++)
		{
			scoreThresh = minScore + iScore * scoreStep;

			for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
			{
				firstTP[iSSegment] = -1;

				for (iMSegment = 0; iMSegment < nMSegments; iMSegment++)
				{
					if (evaluateICP)
						iMatch = scoreMatchMatrixICP.Element[iSSegment].Element[iMSegment].idx;
					else
					iMatch = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].idx;

					if (iMatch != -1)
					{
						if (evaluateICP)
							scoreTmp = scoreMatchMatrixICP.Element[iSSegment].Element[iMSegment].cost;
						else
						scoreTmp = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].cost;

						if (scoreTmp <= scoreThresh)
						{
							//Compare to segment GT
							iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
							iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;

							int iSegmentGT = (iScene - 1) * nDominantClusters + iSSegment;

							//eliminate FP from segments without GT
							if (!segmentGT.Element[iSegmentGT].valid)
								TPMatch = false;
							else
								TPMatch = CompareMatchToSegmentGT((iScene - 1), iSSegment, iMatchedModel);

							if (!TPMatch)
							{
								FP_++;
							}
							else
							{
								if (firstTP[iSSegment] == -1)
								{
									firstTP[iSSegment] = iMSegment;
									firstTPScore[iSSegment] = scoreTmp;
									firstTPiModel[iSSegment] = iMatchedModel;
								}

								//check pose of TP segment matches
								if (iScore == nScoreSteps - 1)
								{
									iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;

									pSCTI = CTISet.pCTI.Element[iSCTI];
									pMCTI = MCTISet.pCTI.Element[iMCTI];

									MSTransformation(pMCTI, pSCTI, pCTImatchesArray.Element[iMatch]->tMatch, pCTImatchesArray.Element[iMatch]->R, pCTImatchesArray.Element[iMatch]->t);

									FindGTInstance(&pGT, pCTImatchesArray.Element[iMatch]->iScene, iMatchedModel);

									//poseMatch = PoseCheck(pGT, pCTImatchesArray.Element[iMatch], 50.0, PI / 6, fpPoseError);
									poseMatch = PoseCheck(pGT, pCTImatchesArray.Element[iMatch], 50.0, cos30, fpPoseError, fpnotFirstPoseErr, evaluateICP);
								}

							}
						}
					}
				}
			}


#ifdef RVLPSGM_EVALUATION_PRINT_INFO
			CountTPandFN(TP_, FN_, true);
#else
			CountTPandFN(TP_, FN_, false);
#endif

			CalculatePR(TP_, FP_, FN_, precision, recall);

			pECCVGT->ResetMatchFlag();

			PrintMatchInfo(fp, fpLog, TP_, FP_, FN_, precision, recall, nSSegments, firstTP, firstTPiModel, firstTPScore, scoreThresh, minScore, maxScore, scoreStep, nBestSegments, -1.0, graphID);

			TP_ = 0; FP_ = 0; FN_ = 0;

			graphID++;
			
		}

	}
	else
	{
		for (iBestMatches = 0; iBestMatches < nBestSegments; iBestMatches++)
		{
			//for (scoreThresh = minScore; scoreThresh <= maxScore; scoreThresh += scoreStep)
			//{
				for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
				{
					firstTP[iSSegment] = -1;

					for (iMSegment = 0; iMSegment <= iBestMatches; iMSegment++)
					{
					if (evaluateICP)
						iMatch = scoreMatchMatrixICP.Element[iSSegment].Element[iMSegment].idx;
					else
						iMatch = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].idx;

						if (iMatch != -1)
						{
						if (evaluateICP)
							scoreTmp = scoreMatchMatrixICP.Element[iSSegment].Element[iMSegment].cost;
						else
							scoreTmp = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].cost;

						//save score for best match
						//if(iMSegment == 0)
						//bestNesto = scoreTmp

							//if (scoreTmp <= scoreThresh)
							//{
								//Compare to segment GT
								iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
								iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;

								int iSegmentGT = (iScene - 1) * nDominantClusters + iSSegment;

								//eliminate FP from segments without GT
								if (!segmentGT.Element[iSegmentGT].valid)
									TPMatch = false;
								else
						{
									TPMatch = CompareMatchToSegmentGT((iScene - 1), iSSegment, iMatchedModel);
						}

								if (!TPMatch)
								{
									FP_++;
								}
								else
						{
									if (firstTP[iSSegment] == -1)
									{
								if (iSSegment == 2)
									printf("iMatch: %d, iMatchedModel: %d", iMatch, iMatchedModel);

										firstTP[iSSegment] = iMSegment;
										firstTPScore[iSSegment] = scoreTmp;
										firstTPiModel[iSSegment] = iMatchedModel;

								//if iMSegment != 0
								//u file zapisati bestNesto i scoreTmp
							}

							//check pose of TP segment matches
							if (iBestMatches == nBestSegments - 1)
							{
								iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;

								pSCTI = CTISet.pCTI.Element[iSCTI];
								pMCTI = MCTISet.pCTI.Element[iMCTI];
								
								MSTransformation(pMCTI, pSCTI, pCTImatchesArray.Element[iMatch]->tMatch, pCTImatchesArray.Element[iMatch]->R, pCTImatchesArray.Element[iMatch]->t);					

								FindGTInstance(&pGT, pCTImatchesArray.Element[iMatch]->iScene, iMatchedModel);

								//poseMatch = PoseCheck(pGT, pCTImatchesArray.Element[iMatch], 50.0, PI / 6, fpPoseError);
								if (iMSegment != 0) // pose check for matches that are not on the first place 
									int klkl = 0;
									//zapis u file1
									//fprintf(fpnotFirstInfo, "%d, %d, %d", iScene, iSSegment, iMSegment);
									//poseMatch = PoseCheck(pGT, pCTImatchesArray.Element[iMatch], 50.0, cos30, fpPoseError, fpnotFirstPoseErr, evaluateICP);
							}
									}
							//}
						}
					}
				}

#ifdef RVLPSGM_EVALUATION_PRINT_INFO
				CountTPandFN(TP_, FN_, true);
#else
				CountTPandFN(TP_, FN_, false);
#endif

				CalculatePR(TP_, FP_, FN_, precision, recall);

			//pECCVGT->ResetMatchFlag();

				PrintMatchInfo(fp, fpLog, TP_, FP_, FN_, precision, recall, nSSegments, firstTP, firstTPiModel, firstTPScore, -1.0, -1.0, -1.0, -1.0, nBestSegments, iBestMatches, graphID);

				TP_ = 0; FP_ = 0; FN_ = 0;

				graphID++;

			//}
		}

	}

	delete[] firstTP;
	delete[] firstTPScore;
	delete[] firstTPiModel;
}

void PSGM::PrintMatchInfo(
	FILE *fp,
	FILE *fpLog,
	int TP_,
	int FP_,
	int FN_,
	float precision,
	float recall,
	int nSSegments,
	int *firstTP,
	int *firstTPiModel,
	float *firstTPScore,
	float scoreThresh,
	float minScore,
	float maxScore,
	float scoreStep,
	int nBestSegments,
	int iBestMatches,
	int graphID)
{

	int iSSegment;

#ifdef RVLPSGM_EVALUATION_PRINT_INFO
	printf("---------------------------------------------------\n");
	printf("Scene: %d\n", iScene - 1);
	printf("TP: %d\n", TP_);
	printf("FP: %d\n", FP_);
	printf("FN: %d\n", FN_);

	if (nBestSegments)
		printf("nBestMatches: %d\n", iBestMatches);
	else
	{
		printf("ScoreThresh: %f\n", scoreThresh);
		printf("Min score: %f\n", minScore);
		printf("Max score: %f\n", maxScore);
		printf("Score step: %f\n", scoreStep);
	}

	printf("Precision: %f\n", precision);
	printf("Recall: %f\n", recall);

	printf("...................................................\n");
	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
		if (firstTP[iSSegment] != -1)
			printf("First TP for segment %d is on %d place; Matched with iModel: %d (score = %f)\n", iSSegment, firstTP[iSSegment], firstTPiModel[iSSegment], firstTPScore[iSSegment]);
	printf("---------------------------------------------------\n\n");
#endif

	//print to log file
	fprintf(fpLog, "---------------------------------------------------\n");
	fprintf(fpLog, "Scene: %d\n", iScene - 1);
	fprintf(fpLog, "TP: %d\n", TP_);
	fprintf(fpLog, "FP: %d\n", FP_);
	fprintf(fpLog, "FN: %d\n", FN_);

	if (nBestSegments)
		fprintf(fpLog, "nBestMatches: %d\n", iBestMatches);
	else
	{
		fprintf(fpLog, "ScoreThresh: %f\n", scoreThresh);
		fprintf(fpLog, "Min score: %f\n", minScore);
		fprintf(fpLog, "Max score: %f\n", maxScore);
		fprintf(fpLog, "Score step: %f\n", scoreStep);
	}

	fprintf(fpLog, "Precision: %f\n", precision);
	fprintf(fpLog, "Recall: %f\n", recall);

	fprintf(fpLog, "...................................................\n");
	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
		if (firstTP[iSSegment] != -1)
			fprintf(fpLog, "First TP for segment %d is on %d place; Matched with iModel: %d (score = %f)\n", iSSegment, firstTP[iSSegment], firstTPiModel[iSSegment], firstTPScore[iSSegment]);
	fprintf(fpLog, "---------------------------------------------------\n\n");

	fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\n", graphID, iScene - 1, TP_, FP_, FN_, nBestSegments, scoreThresh, -1.0, precision, recall);

}

void PSGM::SaveMatches()
{

	char *matchFileName = RVLCreateString(sceneFileName);

	sprintf(matchFileName + strlen(matchFileName) - 3, "smf");
	
	FILE *fp = fopen(matchFileName, "w");

	RECOG::PSGM_::MatchInstance *pMatch = CTImatches.pFirst;

	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstance *pMCTI;

	int i;

	while (pMatch)
	{
		fprintf(fp, "%d\t%d\t%d\t", pMatch->iScene, pMatch->iSCTI, pMatch->iMCTI);

		pSCTI = CTISet.pCTI.Element[pMatch->iSCTI];
		pMCTI = MCTISet.pCTI.Element[pMatch->iMCTI];

		MSTransformation(pMCTI, pSCTI, pMatch->tMatch, pMatch->R, pMatch->t);

		for (i = 0; i < 9; i++)
			fprintf(fp, "%f\t", pMatch->R[i]);

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pMatch->tMatch[i]);

		fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%d\n", pMatch->score, pMatch->E, pMatch->probability2, pMatch->angleGT, pMatch->distanceGT, pMatch->nValids);

		pMatch = pMatch->pNext;
	}

	fclose(fp);
}

void PSGM::SaveSegmentGT(FILE*fp, int iScene)
{
	int iSSegment;

	int nClusters = CTISet.SegmentCTIs.n;

	for (iSSegment = 0; iSSegment < nClusters; iSSegment++)
		fprintf(fp, "%d\t%d\t%d\t%d\t%d\n", segmentGT.Element[iScene*nDominantClusters + iSSegment].iScene, segmentGT.Element[iScene*nDominantClusters + iSSegment].iSSegment, segmentGT.Element[iScene*nDominantClusters + iSSegment].iModel, segmentGT.Element[iScene*nDominantClusters + iSSegment].iMSegment, segmentGT.Element[iScene*nDominantClusters + iSSegment].valid);
}

void PSGM::LoadSegmentGT(FILE*fp, int iScene)
{
	char line[3000] = { 0 };

	int nLines, iLine;

	nLines = 0;

	int iSSegmentTmp;

	//count number of lines in gt file
	while (!feof(fp))
	{
		line[0] = '\0';

		fgets(line, 3000, fp);

		if (line[0] == '\0' || line[0] == '\n')
			continue;

		nLines++;
	}

	rewind(fp);

	//load segment GT for current scene
	for (iLine = 0; iLine < nLines; iLine++)
	{
		fscanf(fp, "%d\t", &iSSegmentTmp);

		segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].iScene = iScene;

		segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].iSSegment = iSSegmentTmp;

		fscanf(fp, "%d\t%d\t%d\n", &segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].iModel, &segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].iMSegment, &segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].valid);
	}
}

void PSGM::LoadCompleteSegmentGT(FileSequenceLoader sceneSequence)
{
	RVL_DELETE_ARRAY(segmentGT.Element);

	segmentGT.Element = new RVL::SegmentGTInstance[nDominantClusters * sceneSequence.nFileNames];
	segmentGT.n = nDominantClusters * sceneSequence.nFileNames;

	char filePath[200];

	char *segmentGTFilePath;

	FILE *fp;

	int iSceneTmp = 0;

	while (sceneSequence.GetNextPath(filePath))
	{
		//create segment GT file path
		segmentGTFilePath = RVLCreateFileName(filePath, ".ply", -1, ".sgt", pMem);

		fp = fopen(segmentGTFilePath, "r");

		if (fp)
		{
			LoadSegmentGT(fp, iSceneTmp);

			fclose(fp);
		}
		else
		{
			printf("*****************WARNING****************\n", filePath);
			printf("Segment GT file is missing for scene:\n%s\n", filePath);
			printf("****************************************\n", filePath);
		}

		iSceneTmp++;
	}
}

void PSGM::ConvexTemplateCentoidID()
{
	int iPlane, i;

	float minx, maxx, miny, maxy, minz, maxz;
	

	for (i = 0; i < 6; i++)
		centroidID.Element[i].Idx = 0;
	
	minx = convexTemplate.Element[0].N[0];
	maxx = convexTemplate.Element[0].N[0];

	miny = convexTemplate.Element[0].N[1];
	maxy = convexTemplate.Element[0].N[1];

	minz = convexTemplate.Element[0].N[2];
	maxz = convexTemplate.Element[0].N[2];		

	for (iPlane = 1; iPlane < convexTemplate.n; iPlane++)
	{
		//find minx
		if (convexTemplate.Element[iPlane].N[0] < minx)
		{
			minx = convexTemplate.Element[iPlane].N[0];
			centroidID.Element[0].Idx = iPlane;
		}

		//find maxx
		if (convexTemplate.Element[iPlane].N[0] > maxx)
		{
			maxx = convexTemplate.Element[iPlane].N[0];
			centroidID.Element[1].Idx = iPlane;
		}

		//find miny
		if (convexTemplate.Element[iPlane].N[1] < miny)
		{
			miny = convexTemplate.Element[iPlane].N[1];
			centroidID.Element[2].Idx = iPlane;
		}

		//find maxy
		if (convexTemplate.Element[iPlane].N[1] > maxy)
		{
			maxy = convexTemplate.Element[iPlane].N[1];
			centroidID.Element[3].Idx = iPlane;
		}

		//find minz
		if (convexTemplate.Element[iPlane].N[2] < minz)
		{
			minz = convexTemplate.Element[iPlane].N[2];
			centroidID.Element[4].Idx = iPlane;
		}

		//find maxz
		if (convexTemplate.Element[iPlane].N[2] > maxz)
		{
			maxz = convexTemplate.Element[iPlane].N[2];
			centroidID.Element[5].Idx = iPlane;
		}
	}
}
//END Vidovic

void PSGM::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	unsigned char *selectionColor)
{
	pVisualizer->normalLength = 10.0;

	pVisualizer->SetMesh(pMesh);

	displayData.pMesh = pMesh;
	displayData.pSurfels = pSurfels;
	displayData.pRecognition = this;
	displayData.pVisualizer = pVisualizer;	
	RVLCOPY3VECTOR(selectionColor, displayData.selectionColor);
	displayData.iSelectedCluster = -1;
		pSurfels->DisplayData.keyPressUserFunction = &RECOG::PSGM_::keyPressUserFunction;
		pSurfels->DisplayData.mouseRButtonDownUserFunction = &RECOG::PSGM_::mouseRButtonDownUserFunction;
		pSurfels->DisplayData.vpUserFunctionData = &displayData;

	pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);
	
	//Is this the best place for this????????
	this->pMesh = pMesh;
}

void PSGM::Display()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	DisplayClusters();

	pSurfels->DisplayEdgeFeatures();

	displayData.bClusters = true;

	//pSurfels->Display(pVisualizer, pMesh);

	//DisplayVertices();

	//if (!displayData.bVertices)
	//	displayData.vertices->VisibilityOff();

	//DisplayReferenceFrames();
}

void PSGM::DisplayClusters()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	int iCluster;
	unsigned char color[3];

	for (iCluster = 0; iCluster < clusters.n; iCluster++)
	{
		RandomColor(color);

		PaintCluster(iCluster, color);
	}
}

void PSGM::DisplayModelInstance(Visualizer *pVisualizer)
{
	//Create polygonal mesh

	//// Setup four points
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(1.0, 0.0, 0.0);
	points->InsertNextPoint(1.5, 0.5, 0.0);
	points->InsertNextPoint(1.0, 1.0, 0.0);
	points->InsertNextPoint(0.0, 1.0, 0.0);

	// Define some colors
	unsigned char red[3] = { 255, 0, 0 };
	unsigned char green[3] = { 0, 255, 0 };
	unsigned char blue[3] = { 0, 0, 255 };
	unsigned char white[3] = { 255, 255, 255 };
	unsigned char black[3] = { 0, 0, 0 };

	// Setup the colors array
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	// Add the three colors we have created to the array
	colors->InsertNextTupleValue(red);
	colors->InsertNextTupleValue(green);
	colors->InsertNextTupleValue(blue);
	colors->InsertNextTupleValue(white);
	colors->InsertNextTupleValue(black);

	// Create the polygon
	vtkSmartPointer<vtkPolygon> polygon =
		vtkSmartPointer<vtkPolygon>::New();
	polygon->GetPointIds()->SetNumberOfIds(5); //make a quad
	polygon->GetPointIds()->SetId(0, 0);
	polygon->GetPointIds()->SetId(1, 1);
	polygon->GetPointIds()->SetId(2, 2);
	polygon->GetPointIds()->SetId(3, 3);
	polygon->GetPointIds()->SetId(4, 4);
	//polygon->GetPointIds()->SetId(5, 5);

	// Add the polygon to a list of polygons
	vtkSmartPointer<vtkCellArray> polygons =
		vtkSmartPointer<vtkCellArray>::New();
	polygons->InsertNextCell(polygon);

	// Create a polydata object and add everything to it
	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	polydata->SetPolys(polygons);
	polydata->GetPointData()->SetScalars(colors);

	//Mapper
	pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
	//map->SetInputData(pMesh->pPolygonData);
	pVisualizer->map->SetInputData(polydata);
	//map->SetInputConnection(polyDataNormals->GetOutputPort());
	pVisualizer->map->InterpolateScalarsBeforeMappingOff();

	//Actor
	pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
	pVisualizer->actor->SetMapper(pVisualizer->map);

	//Insert actor
	pVisualizer->renderer->AddActor(pVisualizer->actor);
}

void PSGM::DisplayCTIs(
	Visualizer *pVisualizer,
	RECOG::CTISet *pCTISet,
	Array<int> *pCTIArray)
{
	Array<int> CTIArray;
	int i;

	if (pCTIArray)
		CTIArray = *pCTIArray;
	else
	{
		CTIArray.Element = new int[pCTISet->pCTI.n];
		CTIArray.n = pCTISet->pCTI.n;

		for (i = 0; i < pCTISet->pCTI.n; i++)
			CTIArray.Element[i] = i;
	}

	RECOG::PSGM_::ModelInstance *pCTI;

	for (i = 0; i < CTIArray.n; i++)
	{
		pCTI = pCTISet->pCTI.Element[CTIArray.Element[i]];

		DisplayCTI(pVisualizer, pCTI);
	}

	RVL_DELETE_ARRAY(CTIArray.Element);
}

void PSGM::DisplayCTI(
	Visualizer *pVisualizer,
	RECOG::PSGM_::ModelInstance *pCTI)
{
	float *N = new float[3 * convexTemplate.n];

	float *N_ = N;

	float *d = new float[convexTemplate.n];

	int i;
	float *N__;

	for (i = 0; i < convexTemplate.n; i++, N_ += 3)
	{
		N__ = convexTemplate.Element[i].N;

		RVLCOPY3VECTOR(N__, N_);

		d[i] = pCTI->modelInstance.Element[i].d;
	}
	
	float tCTIc_CTI[3];

	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(N, d, convexTemplate.n, false, NULL, tCTIc_CTI);

	float *RCTI_S = pCTI->R;
	float *tCTI_S = pCTI->t;
	
	float tCTIc_S[3];

	RVLTRANSF3(tCTIc_CTI, RCTI_S, tCTI_S, tCTIc_S);

	double T[16];

	RVLHTRANSFMX(RCTI_S, tCTIc_S, T);

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(T);

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(modelPD);

	transformFilter->SetTransform(transform);
	transformFilter->Update();

	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper->SetInputConnection(transformFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(0, 1, 0);
	pVisualizer->renderer->AddActor(modelActor);

	delete[] N;
	delete[] d;
}

void PSGM::PaintCluster(
	int iCluster,
	unsigned char *color)
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	int i;
	int iSurfel;
	Surfel *pSurfel;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel = pCluster->iSurfelArray.Element[i];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);
	}
}

void PSGM::PaintClusterVertices(
	int iCluster,
	unsigned char *color)
{
	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	pSurfels->PaintVertices(&(pCluster->iVertexArray), color);
}

void PSGM::DisplayReferenceFrames()
{
	Visualizer *pVisualizer = displayData.pVisualizer;

	double axesLength = 10.0;

	// Create the polydata where we will store all the geometric data
	referenceFramesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	// Create lines.
	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	// Create colors.
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	//int nClusters = RVLMIN(clusters.n, nDominantClusters); //Vidovic

	//RECOG::PSGM_::Cluster *pCluster; //Vidovic
	//int iCluster; //Vidovic
	RECOG::PSGM_::ModelInstance *pModelInstance;

	//Vidovic
	/*
	for (iCluster = 0; iCluster < nClusters; iCluster++)
	{
		pCluster = clusters.Element[iCluster];

		pModelInstance = pCluster->modelInstanceList.pFirst;

		while (pModelInstance)
		{
			pVisualizer->AddReferenceFrame(pts, lines, colors, pModelInstance->R, pModelInstance->t, 10.0);

			//vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

			//axes->SetTotalLength(axesLength, axesLength, axesLength);

			//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

			//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
			//double T[16];
			//RVLCREATE3DTRANSF(pModelInstance->R, pModelInstance->t, T);

			//transform->SetMatrix(T);

			//axes->SetUserTransform(transform);

			//vtkMatrix4x4 *T_ = axes->GetMatrix();

			//pVisualizer->renderer->AddActor(axes);

			pModelInstance = pModelInstance->pNext;
		}
	}*/

	pModelInstance = CTISet.CTI.pFirst;

	while (pModelInstance)
	{
		pVisualizer->AddReferenceFrame(pts, lines, colors, pModelInstance->R, pModelInstance->t, 10.0);

		//vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

		//axes->SetTotalLength(axesLength, axesLength, axesLength);

		//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

		//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		//double T[16];
		//RVLCREATE3DTRANSF(pModelInstance->R, pModelInstance->t, T);

		//transform->SetMatrix(T);

		//axes->SetUserTransform(transform);

		//vtkMatrix4x4 *T_ = axes->GetMatrix();

		//pVisualizer->renderer->AddActor(axes);

		pModelInstance = pModelInstance->pNext;
	}
	//END Vidovic

	// Add the points to the polydata container
	referenceFramesPolyData->SetPoints(pts);

	// Add the lines to the polydata container
	referenceFramesPolyData->SetLines(lines);

	// Color the lines.
	referenceFramesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(referenceFramesPolyData);

	displayData.referenceFrames = vtkSmartPointer<vtkActor>::New();
	displayData.referenceFrames->SetMapper(mapper);

	pVisualizer->renderer->AddActor(displayData.referenceFrames);
}

bool RVL::RECOG::PSGM_::keyPressUserFunction(
	Mesh *pMesh, 
	SurfelGraph *pSurfels, 
	std::string &key, 
	void *vpData
	)
{
	RECOG::PSGM_::DisplayData *pData = (RECOG::PSGM_::DisplayData *)vpData;

	PSGM *pRecognition = pData->pRecognition;
	Visualizer *pVisualizer = pData->pVisualizer;

	if (key == "a")
	{
		pData->bClusters = !pData->bClusters;

		if (pData->bClusters)
			pRecognition->DisplayClusters();
		else
		{
			pSurfels->Display(pVisualizer, pMesh);

			//if (pSurfels->DisplayData.bVertices)
			//{
			//	if (pData->iSelectedCluster >= 0)
			//	{
			//		unsigned char color[3];

			//		RVLSET3VECTOR(color, 255, 0, 0);

			//		pRecognition->PaintClusterVertices(pData->iSelectedCluster, color);

			//		pSurfels->UpdateVertexDisplayLines();
			//	}
			//}

			pData->iSelectedCluster = -1;
		}
			
		return true;
	}

	//sometimes it doesn't exit properly, as if "c" is constantly pressed
	if (key == "c") // choose which hypothesis is shown (0-6) 
	{
		std::string line;
		int iHypothesesRank = -1;
		unsigned char SelectionColor[3];

		SelectionColor[0] = 0;
		SelectionColor[1] = 255;
		SelectionColor[2] = 0;
		do
		{
			std::cout << "Enter hypothesis rank, 0-6: ";
			std::getline(std::cin, line);
			sscanf(line.data(), "%d", &iHypothesesRank);
		} while (iHypothesesRank < 0 || iHypothesesRank > 20);

		//delete visualized ICP matches from the scene:
		pVisualizer->renderer->RemoveAllViewProps();
		pRecognition->InitDisplay(pVisualizer, pMesh, SelectionColor);
		pRecognition->Display();

#ifdef RVLPSGM_ICP
		for (int i = 0; i < pRecognition->scoreMatchMatrixICP.n; i++)
		{
			if (pRecognition->scoreMatchMatrixICP.Element[i].Element[iHypothesesRank].idx != -1)
			{
				//visualize new ICP matches on the scene
				pRecognition->AddOneModelToVisualizer(pVisualizer, pRecognition->scoreMatchMatrixICP.Element[i].Element[iHypothesesRank].idx, iHypothesesRank, true);
			}
		}
#else
		for (int i = 0; i < pRecognition->scoreMatchMatrix.n; i++)
		{
			if (pRecognition->scoreMatchMatrix.Element[i].Element[iHypothesesRank].idx != -1)
			{
				//visualize new matches on the scene
				pRecognition->AddOneModelToVisualizer(pVisualizer, pRecognition->scoreMatchMatrix.Element[i].Element[iHypothesesRank].idx, iHypothesesRank, false);
			}
#endif
		}
		return true;
	}

	return false;
}

bool RVL::RECOG::PSGM_::mouseRButtonDownUserFunction(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSelectedPt,
	int iSelectedSurfel,
	void *vpData)
{
	RECOG::PSGM_::DisplayData *pData = (RECOG::PSGM_::DisplayData *)vpData;

	if (!pData->bClusters)
		return false;

	PSGM *pRecognition = pData->pRecognition;
	Visualizer *pVisualizer = pData->pVisualizer;

	unsigned char color[3];

	if (pData->iSelectedCluster >= 0)
	{
		RandomColor(color);

		pRecognition->PaintCluster(pData->iSelectedCluster, color);

		if (pSurfels->DisplayData.bVertices)
		{
			RVLSET3VECTOR(color, 255, 0, 0);

			pRecognition->PaintClusterVertices(pData->iSelectedCluster, color);
		}
	}

	int iCluster = pRecognition->clusterMap[iSelectedSurfel];

	if (iCluster >= 0)
	{
		printf("Selected segment: %d\n", iCluster);

		pRecognition->PaintCluster(iCluster, pData->selectionColor);

		if (pSurfels->DisplayData.bVertices)
		{
			RVLSET3VECTOR(color, 255, 255, 0);

			pRecognition->PaintClusterVertices(iCluster, color);

			pSurfels->UpdateVertexDisplayLines();
		}

		pData->iSelectedCluster = iCluster;

		FILE *fp = fopen("C:\\RVL\\Debug\\cluster_vertices.txt", "w");

		RECOG::PSGM_::Cluster *pCluster = pRecognition->clusters.Element[iCluster];

		int i, iVertex;
		SURFEL::Vertex *pVertex;

		for (i = 0; i < pCluster->iVertexArray.n; i++)
		{
			iVertex = pCluster->iVertexArray.Element[i];

			pVertex = pSurfels->vertexArray.Element[iVertex];

			fprintf(fp, "%d\t%lf\t%lf\t%lf\n", iVertex, pVertex->P[0], pVertex->P[1], pVertex->P[2]);
		}

		fclose(fp);

		return true;
	}
	else
		return false;
}

void PSGM::CalculatePose(int iMatch)
{
	//int iMatch = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].idx;
	int iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
	int iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;

	RVL::RECOG::PSGM_::ModelInstance *pSCTI = CTISet.pCTI.Element[iSCTI];
	RVL::RECOG::PSGM_::ModelInstance *pMCTI = MCTISet.pCTI.Element[iMCTI];

	MSTransformation(pMCTI, pSCTI, pCTImatchesArray.Element[iMatch]->tMatch, pCTImatchesArray.Element[iMatch]->R, pCTImatchesArray.Element[iMatch]->t);
}



void PSGM::AddModelsToVisualizer(Visualizer *pVisualizer, bool align, RVL::PSGM::ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr)
{
	printf("Visualization:\n\n");
	for (int i = 0; i < scoreMatchMatrixICP.n; i++)
	{
		if (scoreMatchMatrixICP.Element[i].Element[0].idx != -1)
			//AddOneModelToVisualizer(pVisualizer, scoreMatchMatrixICP.Element[i].Element[0].idx, 0, align); //without double ICP
			AddOneModelToVisualizerICP(pVisualizer, scoreMatchMatrixICP.Element[i].Element[0].idx, align, ICPFunction, ICPvariant, kdTreePtr); //with icp
	}
}

void PSGM::AddOneModelToVisualizer(Visualizer *pVisualizer, int iMatch, int iRank, bool align)
{
	//Setting indices:
	int iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
	int iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
	int iCluster, iModel;

	//Getting scene and model pointers:
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;
	pMCTI = MCTISet.pCTI.Element[iMCTI];
	pMIE = pMCTI->modelInstance.Element;
	pSCTI = CTISet.pCTI.Element[iSCTI];
	pSIE = pSCTI->modelInstance.Element;

	iCluster = pSCTI->iCluster;
	iModel = pMCTI->iModel;

	//For a chosen hypothesis, prints which scene segment is matched to which model
	printf("SSegment: %d\tMatchedModel: %d\n", iCluster, iModel);

	//Setting descriptors:
	float *dM = new float[66];
	float *dS = new float[66];
	int *validS = new int[66];

	for (int i = 0; i < 66; i++)
	{
		dS[i] = pSIE->d; // Filling scene descriptor
		validS[i] = pSIE->valid;
		pSIE++;

		dM[i] = pMIE->d / 1000.0; // Filling model descriptor
		pMIE++;
	}

	//Getting match pointer and calculating pose:
	RECOG::PSGM_::MatchInstance *pMatch = pCTImatchesArray.Element[iMatch];
	CalculatePose(iMatch);
	Eigen::MatrixXf nT = ConvexTemplatenT();

	//Generate model polydata
	float t[3];
	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dM, 66, false, NULL, t);

	//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
	float *R_M_S = pCTImatchesArray.Element[iMatch]->R;
	float *t_M_S_mm = pCTImatchesArray.Element[iMatch]->t;
	float t_M_S[3];
	RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);
	float *R_CTIM_M = pMCTI->R;
	float *t_CTIM_M = pMCTI->t;
	float R_CTIM_S[9], t_CTIM_S[3];

	RVLCOMPTRANSF3D(R_M_S, t_M_S, R_CTIM_M, t_CTIM_M, R_CTIM_S, t_CTIM_S);
	float t_CCTIM_S[3];
	RVLTRANSF3(t, R_CTIM_S, t_CTIM_S, t_CCTIM_S);
	double T_CCTIM_S[16];
	RVLHTRANSFMX(pSCTI->R, t_CCTIM_S, T_CCTIM_S);

	//Generate scene CTI polydata
	vtkSmartPointer<vtkPolyData> modelSPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dS, 66, false, validS, t);

	//Getting transform from centered (scene) CTI polygon data to scene (T_CCTIS_S)
	float t_CCTIS_S[3];
	RVLTRANSF3(t, pSCTI->R, pSCTI->t, t_CCTIS_S)
		double T_CCTIS_S[16];
	RVLHTRANSFMX(pSCTI->R, t_CCTIS_S, T_CCTIS_S);


	//PLY Model transformation
	double T_M_S[16];
	RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

#ifdef RVLPSGM_ICP
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
		transform->SetMatrix(T_M_S); //when transforming PLY models to scene
#endif

	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
		transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene

#ifdef RVLPSGM_ICP
	//Scaling PLY model to meters
	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
	transformScale->Scale(0.001, 0.001, 0.001);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iModel));
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();
#endif

	//Transforming PLY model or CTI convex hull model to scene
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
		transformFilter->SetInputData(modelPD); //model CTI convex hull
#ifdef RVLPSGM_ICP
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
		transformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model
#endif

	transformFilter->SetTransform(transform);
	transformFilter->Update();

	//Transforming scene CTI convex hull
	vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
	transform2->SetMatrix(T_CCTIS_S);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter2->SetInputData(modelSPD);
	transformFilter2->SetTransform(transform2);
	transformFilter2->Update();

#ifdef RVLPSGM_ICP
	//Aligning point clouds (if required) 
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterICP;
	if (align)
		{
		//Sampling filter for model polydata
		//vtkSmartPointer<vtkTriangleFilter> modelSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
		//modelSamplerTriangleFilter->SetInputConnection(transformFilter->GetOutputPort());
		//modelSamplerTriangleFilter->Update();
		//vtkSmartPointer<vtkPolyDataPointSampler> modelSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
		//modelSampler->SetInputConnection(modelSamplerTriangleFilter->GetOutputPort());
		//modelSampler->SetDistance(0.005);
		//modelSampler->Update();
		//vtkSmartPointer<vtkPolyData> modelSamplerPD = modelSampler->GetOutput();

		//Sampling filter for scene polydata
		//vtkSmartPointer<vtkTriangleFilter> sceneSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); //Creates triangles from polygons (Samples don't work with polygons)
		//sceneSamplerTriangleFilter->SetInputConnection(transformFilter2->GetOutputPort());
		//sceneSamplerTriangleFilter->Update();
		//vtkSmartPointer<vtkPolyDataPointSampler> sceneSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
		//sceneSampler->SetInputConnection(sceneSamplerTriangleFilter->GetOutputPort());
		//sceneSampler->SetDistance(0.005);
		//sceneSampler->Update();
		//vtkSmartPointer<vtkPolyData> sceneSamplerPD = sceneSampler->GetOutput();

		/*vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
		cleanFilter->SetInputConnection(transformFilter->GetOutputPort());
		cleanFilter->PointMergingOn();
		cleanFilter->SetAbsoluteTolerance(0.005);
		cleanFilter->ToleranceIsAbsoluteOn();
		cleanFilter->Update();*/
		//vtkSmartPointer<vtkPolyData> scenePD = GetSceneModelPC(iCluster); //Generate scene model pointcloud
		
		//Aligning pointcluds (using PCL ICP)
		double icpT[16];

		vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilter->GetOutput()); //Generate visible parts of the models pointcloud
		for (int k = 0; k < 16; k++)
			{
			icpT[k] = icpTMatrix[iCluster*nBestMatches * 16 + iRank * 16 + k];
			}

		//Transforming model polydata to ICP pose
		vtkSmartPointer<vtkTransform> transformICP = vtkSmartPointer<vtkTransform>::New();
		transformICP->SetMatrix(icpT);
		transformFilterICP = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformFilterICP->SetInputData(visiblePD);
		//transformFilterICP->SetInputConnection(transformFilter->GetOutputPort());
		transformFilterICP->SetTransform(transformICP);
		transformFilterICP->Update();
		}		
#endif

	//Mapper and actor for model
	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#ifdef RVLPSGM_ICP
	if (align)
		modelMapper->SetInputConnection(transformFilterICP->GetOutputPort());
	else
#endif
		modelMapper->SetInputConnection(transformFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(1, 0, 0);
	modelActor->GetProperty()->SetPointSize(3);
	pVisualizer->renderer->AddActor(modelActor);

	//Mapper and actor for scene
	vtkSmartPointer<vtkPolyDataMapper> modelMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper2->SetInputConnection(transformFilter2->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor2 = vtkSmartPointer<vtkActor>::New();
	modelActor2->SetMapper(modelMapper2);
	modelActor2->GetProperty()->SetColor(0, 0, 1);
	pVisualizer->renderer->AddActor(modelActor2);

	delete[] dM;
	delete[] dS;
	delete[] validS;
}

//With ICP calculation
void PSGM::AddOneModelToVisualizerICP(Visualizer *pVisualizer, int iMatch, bool align, RVL::PSGM::ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr)
{
	//Setting indices:
	int iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
	int iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
	int iCluster, iModel;
	
	//Getting scene and model pointers:
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;
	pMCTI = MCTISet.pCTI.Element[iMCTI];
	pMIE = pMCTI->modelInstance.Element;
	pSCTI = CTISet.pCTI.Element[iSCTI];
	pSIE = pSCTI->modelInstance.Element;

	iCluster = pSCTI->iCluster;
	iModel = pMCTI->iModel;
	
	//Setting descriptors:
	float *dM = new float[66];
	float *dS = new float[66];
	int *validS = new int[66];

	for (int i = 0; i < 66; i++)
	{
		dS[i] = pSIE->d; // Filling scene descriptor
		validS[i] = pSIE->valid;
		pSIE++;

		dM[i] = pMIE->d / 1000.0; // Filling model descriptor
		pMIE++;
	}

	//Getting match pointer and calculating pose:
	RECOG::PSGM_::MatchInstance *pMatch = pCTImatchesArray.Element[iMatch];
	CalculatePose(iMatch);
	Eigen::MatrixXf nT = ConvexTemplatenT();

	//Generate model polydata
	float t[3];
	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dM, 66, false, NULL, t);
	
	//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
	float *R_M_S = pCTImatchesArray.Element[iMatch]->R;
	float *t_M_S_mm = pCTImatchesArray.Element[iMatch]->t;
	float t_M_S[3];
	RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);
	float *R_CTIM_M = pMCTI->R;
	float *t_CTIM_M = pMCTI->t;
	float R_CTIM_S[9], t_CTIM_S[3];

	RVLCOMPTRANSF3D(R_M_S, t_M_S, R_CTIM_M, t_CTIM_M, R_CTIM_S, t_CTIM_S);
	float t_CCTIM_S[3];
	RVLTRANSF3(t, R_CTIM_S, t_CTIM_S, t_CCTIM_S);
	double T_CCTIM_S[16];
	RVLHTRANSFMX(pSCTI->R, t_CCTIM_S, T_CCTIM_S);

	//Generate scene CTI polydata
	vtkSmartPointer<vtkPolyData> modelSPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dS, 66, false, validS, t);
	
	//Getting transform from centered (scene) CTI polygon data to scene (T_CCTIS_S)
	float t_CCTIS_S[3];
	RVLTRANSF3(t, pSCTI->R, pSCTI->t, t_CCTIS_S)
	double T_CCTIS_S[16];
	RVLHTRANSFMX(pSCTI->R, t_CCTIS_S, T_CCTIS_S);


	//PLY Model transformation
	double T_M_S[16];
	RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
		transform->SetMatrix(T_M_S); //when transforming PLY models to scene

	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
		transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene

	//Scaling PLY model to meters
	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
	transformScale->Scale(0.001, 0.001, 0.001);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iModel));
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();

	//Transforming PLY model or CTI convex hull model to scene
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
		transformFilter->SetInputData(modelPD); //model CTI convex hull
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
		transformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model

	transformFilter->SetTransform(transform);
	transformFilter->Update();

	//Transforming scene CTI convex hull
	vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
	transform2->SetMatrix(T_CCTIS_S);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter2->SetInputData(modelSPD);
	transformFilter2->SetTransform(transform2);
	transformFilter2->Update();

	//Aligning point clouds (if required) 
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterICP;
	if (align) 
	{
		//Sampling filter for model polydata
		//vtkSmartPointer<vtkTriangleFilter> modelSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
		//modelSamplerTriangleFilter->SetInputConnection(transformFilter->GetOutputPort());
		//modelSamplerTriangleFilter->Update();
		//vtkSmartPointer<vtkPolyDataPointSampler> modelSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
		//modelSampler->SetInputConnection(modelSamplerTriangleFilter->GetOutputPort());
		//modelSampler->SetDistance(0.005);
		//modelSampler->Update();
		//vtkSmartPointer<vtkPolyData> modelSamplerPD = modelSampler->GetOutput();

		//Sampling filter for scene polydata
		//vtkSmartPointer<vtkTriangleFilter> sceneSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); //Creates triangles from polygons (Samples don't work with polygons)
		//sceneSamplerTriangleFilter->SetInputConnection(transformFilter2->GetOutputPort());
		//sceneSamplerTriangleFilter->Update();
		//vtkSmartPointer<vtkPolyDataPointSampler> sceneSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
		//sceneSampler->SetInputConnection(sceneSamplerTriangleFilter->GetOutputPort());
		//sceneSampler->SetDistance(0.005);
		//sceneSampler->Update();
		//vtkSmartPointer<vtkPolyData> sceneSamplerPD = sceneSampler->GetOutput();

		/*vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
		cleanFilter->SetInputConnection(transformFilter->GetOutputPort());
		cleanFilter->PointMergingOn();
		cleanFilter->SetAbsoluteTolerance(0.005);
		cleanFilter->ToleranceIsAbsoluteOn();
		cleanFilter->Update();*/


		//vtkSmartPointer<vtkPolyData> scenePD = GetSceneModelPC(iCluster); //Generate scene model pointcloud
		//Aligning pointcluds (using PCL ICP)
		float icpT[16];
		double icpTd[16];
		double fitnessScore;

		vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilter->GetOutput()); //Generate visible scene model pointcloud
		ICPFunction(visiblePD, /*this->pMesh->pPolygonData*/this->segmentN_PD.at(iCluster), icpT, 30, 0.01, ICPvariant, &fitnessScore, kdTreePtr);
		for (int i = 0; i < 16; i++)
		{
			icpTd[i] = icpT[i];
		}

		//Transforming model polydata to ICP pose
		vtkSmartPointer<vtkTransform> transformICP = vtkSmartPointer<vtkTransform>::New();
		transformICP->SetMatrix(icpTd);
		transformFilterICP = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformFilterICP->SetInputData(visiblePD);
		//transformFilterICP->SetInputConnection(transformFilter->GetOutputPort());
		transformFilterICP->SetTransform(transformICP);
		transformFilterICP->Update();
	}

	//Mapper and actor for model
	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	if (align)
		modelMapper->SetInputConnection(transformFilterICP->GetOutputPort());
	else 
		modelMapper->SetInputConnection(transformFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(1, 0, 0);
	modelActor->GetProperty()->SetPointSize(3);
	pVisualizer->renderer->AddActor(modelActor);
	
	//Mapper and actor for scene
	vtkSmartPointer<vtkPolyDataMapper> modelMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper2->SetInputConnection(transformFilter2->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor2 = vtkSmartPointer<vtkActor>::New();
	modelActor2->SetMapper(modelMapper2);
	modelActor2->GetProperty()->SetColor(0, 0, 1);
	pVisualizer->renderer->AddActor(modelActor2);

	delete[] dM;
	delete[] dS;
	delete[] validS;
}

vtkSmartPointer<vtkPolyData> PSGM::GetSceneModelPC(int iCluster)
{

	RECOG::PSGM_::Cluster *pCluster;
	Surfel *pSurfel;
	RVL::QLIST::Index2 *pt;
	//RECOG::PSGM_::ModelInstance *pModelInstance;
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
	normals->SetNumberOfComponents(3);
	//points->SetDataTypeToDouble();
	vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
	int ptIdx = 0;
	pCluster = clusters.Element[iCluster];
	for (int i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		pSurfel = &this->pSurfels->NodeArray.Element[pCluster->iSurfelArray.Element[i]];
		pt = pSurfel->PtList.pFirst;

		for (int k = 0; k < pSurfel->size; k++)
		{
			points->InsertNextPoint(this->pMesh->NodeArray.Element[pt->Idx].P);
			normals->InsertNextTuple(this->pMesh->NodeArray.Element[pt->Idx].N);
			verts->InsertNextCell(1);
			verts->InsertCellPoint(ptIdx);
			ptIdx++;
			pt = pt->pNext;
		}
	}

	vtkSmartPointer<vtkPolyData> PD = vtkSmartPointer<vtkPolyData>::New();
	PD->SetPoints(points);
	PD->GetPointData()->SetNormals(normals);
	PD->SetVerts(verts);
	return PD;
}

void PSGM::CalculateICPCost(RVL::PSGM::ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr)
{
	int iMatch;
	int iMCTI, iSCTI, iCluster, iModel;
	
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;
	RECOG::PSGM_::MatchInstance *pMatch;

	for (int i = 0; i < scoreMatchMatrix.n; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			iMatch = scoreMatchMatrix.Element[i].Element[j].idx;
			
			//Setting indices:
			iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
			iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
	
			//Getting scene and model pointers:
			pMCTI = MCTISet.pCTI.Element[iMCTI];
			pMIE = pMCTI->modelInstance.Element;
			pSCTI = CTISet.pCTI.Element[iSCTI];
			pSIE = pSCTI->modelInstance.Element;

			iCluster = pSCTI->iCluster;
			iModel = pMCTI->iModel;

			//Getting match pointer and calculating pose:
			pMatch = pCTImatchesArray.Element[iMatch];
			CalculatePose(iMatch);

			//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
			float *R_M_S = pMatch->R;
			float *t_M_S_mm = pMatch->t;
			float t_M_S[3];
			RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);
			
			//PLY Model transformation
			double T_M_S[16];
			RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);
			vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
			transform->SetMatrix(T_M_S); //when transforming PLY models to scene
			//transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene

			//Scaling PLY model to meters
			vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
			transformScale->Scale(0.001, 0.001, 0.001);
			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilterScale->SetInputData(vtkModelDB.at(iModel));
			transformFilterScale->SetTransform(transformScale);
			transformFilterScale->Update();

			//Transforming PLY model or CTI convex hull model to scene
			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//transformFilter->SetInputData(modelPD); //model CTI convex hull
			transformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model
			transformFilter->SetTransform(transform);
			transformFilter->Update();

			
			//Sampling filter for model polydata
			//vtkSmartPointer<vtkTriangleFilter> modelSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
			//modelSamplerTriangleFilter->SetInputConnection(transformFilter->GetOutputPort());
			//modelSamplerTriangleFilter->Update();
			//vtkSmartPointer<vtkPolyDataPointSampler> modelSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
			//modelSampler->SetInputConnection(modelSamplerTriangleFilter->GetOutputPort());
			//modelSampler->SetDistance(0.005);
			//modelSampler->Update();
			//vtkSmartPointer<vtkPolyData> modelSamplerPD = modelSampler->GetOutput();

			//Sampling filter for scene polydata
			//vtkSmartPointer<vtkTriangleFilter> sceneSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); //Creates triangles from polygons (Samples don't work with polygons)
			//sceneSamplerTriangleFilter->SetInputConnection(transformFilter2->GetOutputPort());
			//sceneSamplerTriangleFilter->Update();
			//vtkSmartPointer<vtkPolyDataPointSampler> sceneSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
			//sceneSampler->SetInputConnection(sceneSamplerTriangleFilter->GetOutputPort());
			//sceneSampler->SetDistance(0.005);
			//sceneSampler->Update();
			//vtkSmartPointer<vtkPolyData> sceneSamplerPD = sceneSampler->GetOutput();


			vtkSmartPointer<vtkPolyData> scenePD = GetSceneModelPC(iCluster); //Generate scene model pointcloud
			//Aligning pointcluds (using PCL ICP)
			float icpT[16];
			double fitnessScore;
			ICPFunction(transformFilter->GetOutput(), scenePD, icpT, 20, 0.02, ICPvariant, &fitnessScore, kdTreePtr);

			pMatch->cost_ICP = fitnessScore;
			memcpy(pMatch->T_ICP, icpT, 16 * sizeof(float));
			
			pMatch->RICP_[0] = pMatch->T_ICP[0];
			pMatch->RICP_[1] = pMatch->T_ICP[1];
			pMatch->RICP_[2] = pMatch->T_ICP[2];
			pMatch->RICP_[3] = pMatch->T_ICP[4];
			pMatch->RICP_[4] = pMatch->T_ICP[5];
			pMatch->RICP_[5] = pMatch->T_ICP[6];
			pMatch->RICP_[6] = pMatch->T_ICP[8];
			pMatch->RICP_[7] = pMatch->T_ICP[9];
			pMatch->RICP_[8] = pMatch->T_ICP[10];

			pMatch->tICP_[0] = pMatch->T_ICP[3];
			pMatch->tICP_[1] = pMatch->T_ICP[7];
			pMatch->tICP_[2] = pMatch->T_ICP[11];

			RVLCOMPTRANSF3D(pMatch->R, pMatch->t, pMatch->RICP_, pMatch->tICP_, pMatch->RICP, pMatch->tICP);		
		}
	}
}

vtkSmartPointer<vtkPolyData> PSGM::GetVisiblePart(vtkSmartPointer<vtkPolyData> PD)
{
	vtkSmartPointer<vtkPoints> visiblePoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> visibleNormals = vtkSmartPointer<vtkFloatArray>::New();
	visibleNormals->SetNumberOfComponents(3);
	//points->SetDataTypeToDouble();
	vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyData> visiblePD = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> pdPoints = PD->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(PD->GetPointData()->GetNormals());
	double *point;
	float normal[3];
	int ptIdx = 0;

	if (!normals.GetPointer()) //check if mode does not have normals 
	{
		printf("Invalid input model. Doesn't have normals.\n");
		return NULL;
	}

	for (int i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		point = pdPoints->GetPoint(i);
		normals->GetTupleValue(i, normal);
		if ((point[0] * normal[0] + point[1] * normal[1] + point[2] * normal[2]) < 0) //cheks the scalar product, must be negative
		{
			visiblePoints->InsertNextPoint(point);
			visibleNormals->InsertNextTuple(normal);
			verts->InsertNextCell(1);
			verts->InsertCellPoint(ptIdx);
			ptIdx++;
		}
	}
	visiblePD->SetPoints(visiblePoints);
	visiblePD->GetPointData()->SetNormals(visibleNormals);
	visiblePD->SetVerts(verts);
	return visiblePD;
}

void PSGM::CalculateNNCost(Visualizer *pVisualizer, RVL::PSGM::ICPfunction ICPFunction, int ICPvariant)
{
	int iMatch;
	int iMCTI, iSCTI, iCluster, iModel;

	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;

	RECOG::PSGM_::MatchInstance *pMatch;

	if (icpTMatrix)
		delete[] icpTMatrix;

	icpTMatrix = new double[scoreMatchMatrix.n * nBestMatches * 16]; //nSSegments * nBestMatches * 16 elements of matrix T

	for (int i = 0; i < scoreMatchMatrix.n; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			iMatch = scoreMatchMatrix.Element[i].Element[j].idx;

			if (iMatch == 39597)
				int debug = 0;

			if (iMatch != -1)
			{
			//Setting indices:
			iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
			iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;

			//Getting scene and model pointers:
			pMCTI = MCTISet.pCTI.Element[iMCTI];
			pMIE = pMCTI->modelInstance.Element;
			pSCTI = CTISet.pCTI.Element[iSCTI];
			pSIE = pSCTI->modelInstance.Element;

			iCluster = pSCTI->iCluster;
			iModel = pMCTI->iModel;

			//Getting match pointer and calculating pose:
			pMatch = pCTImatchesArray.Element[iMatch];
			CalculatePose(iMatch);

			//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
			float *R_M_S = pMatch->R;
			float *t_M_S_mm = pMatch->t;
			float t_M_S[3];
			RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);


			//PLY Model transformation
			double T_M_S[16];
			RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);
			vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
			transform->SetMatrix(T_M_S); //when transforming PLY models to scene
			//transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene

			//Scaling PLY model to meters
			vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
			transformScale->Scale(0.001, 0.001, 0.001);
			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilterScale->SetInputData(vtkModelDB.at(iModel));
			transformFilterScale->SetTransform(transformScale);
			transformFilterScale->Update();

			//Transforming PLY model or CTI convex hull model to scene
			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//transformFilter->SetInputData(modelPD); //model CTI convex hull
			transformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model
			transformFilter->SetTransform(transform);
			transformFilter->Update();


			//Sampling filter for model polydata
			//vtkSmartPointer<vtkTriangleFilter> modelSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
			//modelSamplerTriangleFilter->SetInputConnection(transformFilter->GetOutputPort());
			//modelSamplerTriangleFilter->Update();
			//vtkSmartPointer<vtkPolyDataPointSampler> modelSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
			//modelSampler->SetInputConnection(modelSamplerTriangleFilter->GetOutputPort());
			//modelSampler->SetDistance(0.005);
			//modelSampler->Update();
			//vtkSmartPointer<vtkPolyData> modelSamplerPD = modelSampler->GetOutput();

			//Sampling filter for scene polydata
			//vtkSmartPointer<vtkTriangleFilter> sceneSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); //Creates triangles from polygons (Samples don't work with polygons)
			//sceneSamplerTriangleFilter->SetInputConnection(transformFilter2->GetOutputPort());
			//sceneSamplerTriangleFilter->Update();
			//vtkSmartPointer<vtkPolyDataPointSampler> sceneSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
			//sceneSampler->SetInputConnection(sceneSamplerTriangleFilter->GetOutputPort());
			//sceneSampler->SetDistance(0.005);
			//sceneSampler->Update();
			//vtkSmartPointer<vtkPolyData> sceneSamplerPD = sceneSampler->GetOutput();


			vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilter->GetOutput()); //Generate visible scene model pointcloud
			//Aligning pointcluds (using PCL ICP)
			float icpT[16];
			double icpTd[16];
			double fitnessScore;
			
			ICPFunction(visiblePD, /*this->pMesh->pPolygonData*/this->segmentN_PD.at(iCluster), icpT, 30, 0.01, ICPvariant, &fitnessScore, NULL);
				for (int k = 0; k < 16; k++)
			{
					icpTd[k] = icpT[k];			
					//save icpT to PSGM class
					icpTMatrix[i*nBestMatches*16 + j*16 + k] = icpT[k];
			}

			//Transforming model polydata to ICP pose
			vtkSmartPointer<vtkTransform> transformICP = vtkSmartPointer<vtkTransform>::New();
			transformICP->SetMatrix(icpTd);

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterICP = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilterICP->SetInputData(visiblePD);
			transformFilterICP->SetTransform(transformICP);
			transformFilterICP->Update();


				//if visualization is needed right here:

				//Mapper and actor for model
				//vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
				//modelMapper->SetInputConnection(transformFilterICP->GetOutputPort());
				//vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
				//modelActor->SetMapper(modelMapper);
				//modelActor->GetProperty()->SetColor(0, 1, 0);
				//modelActor->GetProperty()->SetPointSize(3);

				//if (j == 0)
				//{
				//	//pVisualizer->renderer->AddActor(modelActor);
				//}


			pMatch->cost_NN = NNCost(iCluster, transformFilterICP->GetOutput());

				memcpy(pMatch->T_ICP, icpT, 16 * sizeof(float));

				pMatch->RICP_[0] = pMatch->T_ICP[0];
				pMatch->RICP_[1] = pMatch->T_ICP[1];
				pMatch->RICP_[2] = pMatch->T_ICP[2];
				pMatch->RICP_[3] = pMatch->T_ICP[4];
				pMatch->RICP_[4] = pMatch->T_ICP[5];
				pMatch->RICP_[5] = pMatch->T_ICP[6];
				pMatch->RICP_[6] = pMatch->T_ICP[8];
				pMatch->RICP_[7] = pMatch->T_ICP[9];
				pMatch->RICP_[8] = pMatch->T_ICP[10];

				pMatch->tICP_[0] = pMatch->T_ICP[3] * 1000;
				pMatch->tICP_[1] = pMatch->T_ICP[7] * 1000;
				pMatch->tICP_[2] = pMatch->T_ICP[11] * 1000;

				
			}
			else break;

		}
	}
}

float PSGM::NNCost(int iCluster, vtkSmartPointer<vtkPolyData> targetPD)
{
	vtkSmartPointer<vtkPolyData> sourcePD = GetSceneModelPC(iCluster); //Generate scene model pointcloud
	NanoFlannPointCloud<float> targetPC;
	vtkSmartPointer<vtkPoints> pdPoints = targetPD->GetPoints();
	targetPC.pts.resize(pdPoints->GetNumberOfPoints());
	double *point;
	int i;

	for (i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		point = pdPoints->GetPoint(i);
		targetPC.pts.at(i).x = point[0];
		targetPC.pts.at(i).y = point[1];
		targetPC.pts.at(i).z = point[2];
	}

	nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, NanoFlannPointCloud<float> >, NanoFlannPointCloud<float>, 3> index(3 /*dim*/, targetPC, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	index.buildIndex();

	vtkSmartPointer<vtkPoints> sourcePoints = sourcePD->GetPoints();
	float pointF[3];

	std::vector<size_t>   ret_index(1);
	std::vector<float> out_dist_sqr(1);
	float costNN=0;
	int br = 0;
	
	for (i = 0; i < sourcePoints->GetNumberOfPoints(); i++)
	{
		point = sourcePoints->GetPoint(i);
		pointF[0] = point[0];
		pointF[1] = point[1];
		pointF[2] = point[2];
		
		index.knnSearch(pointF, 1, &ret_index[0], &out_dist_sqr[0]);
		if (sqrt(out_dist_sqr.at(0)) > 0.01)
		{
			costNN += 0.01;
			br++;
		}
		else costNN += sqrt(out_dist_sqr.at(0));
	}

	float meanCost = costNN / i;
	return costNN;
}

bool PSGM::IsFlat(
	Array<int> surfelArray,
	float *N,
	float &d,
	Array<int> PtArray)
{
	MESH::Distribution PtDistribution;

	int *piPt = PtArray.Element;

	int iSurfel, iiSurfel;
	Surfel *pSurfel;
	QLIST::Index2 *pPtIdx;

	for (iiSurfel = 0; iiSurfel < surfelArray.n; iiSurfel++)
	{
		iSurfel = surfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		pPtIdx = pSurfel->PtList.pFirst;

		while (pPtIdx)
		{
			*(piPt++) = pPtIdx->Idx;

			pPtIdx = pPtIdx->pNext;
		}
	}

	PtArray.n = piPt - PtArray.Element;

	if (PtArray.n == 0)
		return false;

	pMesh->ComputeDistributionDouble(PtArray, PtDistribution);

	float *var = PtDistribution.var;

	int idx[3];
	int iTmp;
	float *N_;

	RVLSORT3ASCEND(var, idx, iTmp);

	if (var[idx[0]] / var[idx[1]] <= 0.0005)
	{
		N_ = PtDistribution.R + 3 * idx[0];

		if (N_[2] > 0.0f)
		{
			RVLNEGVECT3(N_, N);
		}
		else
		{
			RVLCOPY3VECTOR(N_, N);
		}

		d = RVLDOTPRODUCT3(N, PtDistribution.t);

		return true;
	}
	else
		return false;
}

void PSGM::DetectGroundPlane(SURFEL::ObjectGraph *pObjects)
{
	if (pObjects->sortedObjectArray.n < 0)
		pObjects->SortObjects();

	Array<int> PtArray;

	PtArray.Element = new int[pMesh->NodeArray.n];

	Array<int> iSurfelArray;

	iSurfelArray.Element = new int[pSurfels->NodeArray.n];

	bGnd = false;

	iGndObject = -1;

	int iObject;
	SURFEL::Object *pObject;

	for (iObject = 0; iObject < pObjects->objectArray.n; iObject++)
	{
		pObject = pObjects->objectArray.Element + iObject;

		QLIST::CopyToArray(&(pObject->surfelList), &iSurfelArray);

		if (IsFlat(iSurfelArray, NGnd, dGnd, PtArray))
		{
			iGndObject = iObject;

			bGnd = true;

			break;
		}			
	}

	delete[] PtArray.Element;
	delete[] iSurfelArray.Element;
}

bool PSGM::GravityReferenceFrame(
	QList<QLIST::Index> surfelList,
	float *RGC,
	float &varX)
{
	if (!bGnd)
		return false;

	QLIST::Index *piSurfel = surfelList.pFirst;

	if (piSurfel == NULL)
		return false;

	float RCG[9];

	float *XGC = RCG;
	float *YGC = RCG + 3;
	float *ZGC = RCG + 6;

	RVLCOPY3VECTOR(NGnd, ZGC);

	float ZSkew[9];

	RVLSKEW(ZGC, ZSkew);

	bool bFirst = true;
	
	float J[6];

	float *Jx = J;
	float *Jy = J + 3;

	int iSurfel;
	float *N, *R, *X, *Y;
	Surfel *pSurfel;
	float U[3], V[3];
	float lenV, fTmp;
	float A[9], B[9], CV[9];
	float stdx, stdy, varv, kx, ky, minVarv;	

	while (piSurfel)
	{
		iSurfel = piSurfel->Idx;

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->flags & RVLSURFEL_FLAG_RF)
		{
			/// Computation of varv according to ARP3D.TR11.

			N = pSurfel->N;

			R = pSurfel->R;

			X = R;

			Y = R + 3;

			stdx = 1.0f / pSurfel->r1;

			stdy = 1.0f / pSurfel->r2;

			// V <- NGnd x N
			RVLCROSSPRODUCT3(ZGC, N, V);

			// V <- V / || V ||
			// lenV <- || V ||
			RVLNORM3(V, lenV);

			kx = stdx / lenV;
			ky = stdy / lenV;

			// A <- V * V'
			RVLVECTCOV3(V, A);
			RVLCOMPLETESIMMX3(A);

			// B <- (I - A) * [NGnd]x
			RVLMXMUL3X3(A, ZSkew, B);
			RVLDIFMX3X3(ZSkew, B, B);

			// J <- (B * [X Y])'
			RVLMULMX3X3VECT(B, X, Jx);
			RVLMULMX3X3VECT(B, Y, Jy);

			// J <- stdx * J / lenV
			RVLSCALE3VECTOR(Jx, kx, Jx);
			RVLSCALE3VECTOR(Jy, ky, Jy);

			// CV <- J * J'
			RVLVECTCOV3(Jx, A);
			RVLVECTCOV3(Jy, B);
			RVLSUMMX3X3UT(A, B, CV);

			// U <- NGnd x V / || NGnd x V ||
			RVLCROSSPRODUCT3(ZGC, V, U);
			RVLNORM3(U, fTmp);

			// varv <- U' * CV * U
			varv = RVLCOV3DTRANSFTO1D(CV, U);

			///

			if (bFirst || varv < minVarv)
			{
				minVarv = varv;

				RVLCOPY3VECTOR(V, XGC);
				RVLCOPY3VECTOR(U, YGC);

				bFirst = false;
			}
		}	// if (pSurfel->flags & RVLSURFEL_FLAG_RF)

		piSurfel = piSurfel->pNext;
	}	// for every surfel in surfelList

	if (bFirst)
		return false;

	RVLCOPYMX3X3T(RCG, RGC);

	varX = minVarv;

	return true;
}

int PSGM::CTIs(
	QList<QLIST::Index> surfelList,
	Array<int> iVertexArray,
	int iModel,
	int iCluster,
	RECOG::CTISet *pCTISet,
	CRVLMem *pMem)
{
	float RGC[9];
	float varX;

	if (!GravityReferenceFrame(surfelList, RGC, varX))
		return 0;

	RECOG::PSGM_::ModelInstance *pCTI;

	RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::ModelInstance, pCTI);

	pCTISet->AddCTI(pCTI);

	float *R = pCTI->R;

	RVLCOPYMX3X3(RGC, R);

	float *t = pCTI->t;

	RVLNULL3VECTOR(t);

	pCTI->varX = varX;

	pCTI->iCluster = iCluster;
	pCTI->iModel = iModel;

	FitModel(iVertexArray, pCTI);

	return 1;
}

void PSGM::CTIs(
	SURFEL::ObjectGraph *pObjects,
	RECOG::CTISet *pCTISet)
{
	pCTISet->Init();

	if (!bGnd)
		DetectGroundPlane(pObjects);

	if (!bGnd)
		return;

	RVL_DELETE_ARRAY(pCTISet->SegmentCTIs.Element);

	pCTISet->SegmentCTIs.Element = new Array<int>[pObjects->objectArray.n];
	pCTISet->SegmentCTIs.n = pObjects->objectArray.n;

	int iObject;
	SURFEL::Object *pObject;

	for (iObject = 0; iObject < pObjects->objectArray.n; iObject++)
	{
		if (iObject != iGndObject)
		{
			pObject = pObjects->objectArray.Element + iObject;

			if (pObject->iVertexArray.n >= 3)
				pCTISet->SegmentCTIs.Element[iObject].n = CTIs(pObject->surfelList, pObject->iVertexArray, -1, iObject, pCTISet, pMem);
			else
				pCTISet->SegmentCTIs.Element[iObject].n = 0;			
		}
		else
			pCTISet->SegmentCTIs.Element[iObject].n = 0;
	}

	RVL_DELETE_ARRAY(pCTISet->segmentCTIIdxMem);

	pCTISet->segmentCTIIdxMem = new int[pCTISet->pCTI.n];

	int *iSegmentCTIIdx = pCTISet->segmentCTIIdxMem;

	RVL_DELETE_ARRAY(pCTISet->pCTI.Element);

	pCTISet->pCTI.Element = new RECOG::PSGM_::ModelInstance*[pCTISet->pCTI.n];

	QLIST::CreatePtrArray<RECOG::PSGM_::ModelInstance>(&(pCTISet->CTI), &(pCTISet->pCTI));

	for (iObject = 0; iObject < pObjects->objectArray.n; iObject++)
	{
		if (pCTISet->SegmentCTIs.Element[iObject].n > 0)
		{
			pCTISet->SegmentCTIs.Element[iObject].Element = iSegmentCTIIdx;

			iSegmentCTIIdx += pCTISet->SegmentCTIs.Element[iObject].n;

			pCTISet->SegmentCTIs.Element[iObject].n = 0;
		}
		else
			pCTISet->SegmentCTIs.Element[iObject].Element = NULL;
	}

	int iCTI;

	for (iCTI = 0; iCTI < pCTISet->pCTI.n; iCTI++)
	{
		iObject = pCTISet->pCTI.Element[iCTI]->iCluster;

		pCTISet->SegmentCTIs.Element[iObject].Element[pCTISet->SegmentCTIs.Element[iObject].n++] = iCTI;
	}
}

float PSGM::Symmetry(
	SURFEL::ObjectGraph *pObjects,
	int iObject1,
	int iObject2,
	RECOG::CTISet *pCTIs,
	Array<RECOG::PSGM_::SymmetryMatch> &symmetryMatch)
{
	bool bDebug = (iObject1 == debug1 && iObject2 == debug2 || iObject1 == debug2 && iObject2 == debug1);

	if (bDebug)
		int debug = 0;

	//bool bDebug = false;

	int iObject[2];

	iObject[0] = iObject1;
	iObject[1] = iObject2;

	SURFEL::Object *pObject[2];
	
	pObject[0] = pObjects->objectArray.Element + iObject[0];
	pObject[1] = pObjects->objectArray.Element + iObject[1];

	// Select the better RF.

	float *RGC = NULL;

	float varX = 0.0f;

	int iObject1_ = -1;

	RECOG::PSGM_::ModelInstance *pCTI;
	int i;

	for (i = 0; i < 2; i++)
	{
		if (pCTIs->SegmentCTIs.Element[iObject[i]].n > 0)
		{
			pCTI = pCTIs->pCTI.Element[pCTIs->SegmentCTIs.Element[iObject[i]].Element[0]];

			if (RGC == NULL || pCTI->varX < varX)
			{
				RGC = pCTI->R;

				varX = pCTI->varX;

				iObject1_ = i;
			}
		}
	}

	if (RGC == NULL)
		return 0.0f;

	int iObject2_ = 1 - iObject1_;

	// Select symmetry planes from convexTemplate.

	Array<int> iSymmetryPlanes;

	iSymmetryPlanes.Element = new int[convexTemplate.n];

	iSymmetryPlanes.n = 0;

	float *N;

	for (i = 0; i < convexTemplate.n; i++)
	{
		N = convexTemplate.Element[i].N;

		if (RVLABS(N[2]) < 1e-10)
			iSymmetryPlanes.Element[iSymmetryPlanes.n++] = i;
	}
	
	// 

	Array<int> iVertexArray[2];

	iVertexArray[0] = pObject[iObject1_]->iVertexArray;
	iVertexArray[1] = pObject[iObject2_]->iVertexArray;

	int nVertices2 = iVertexArray[1].n;

	pCTI = pCTIs->pCTI.Element[pCTIs->SegmentCTIs.Element[iObject[iObject1_]].Element[0]];

	//RECOG::PSGM_::ModelInstanceElement *pCTIElement1 = CTI1.modelInstance.Element;

	// Determine convex hull.

	//int *hull = new int[convexTemplate.n];

	//int hull_;

	//for (i = 0; i < convexTemplate.n; i++, pCTIElement1++, pCTIElement2++)
	//{
	//	hull_ = -1;

	//	if (pCTIElement1->valid)
	//		hull_ = 0;
	//	
	//	if (pCTIElement2->valid)
	//	{
	//		if (hull_ > 0)
	//		{
	//			if (pCTIElement2->d > pCTIElement1->d)
	//				hull_ = 1;
	//		}
	//		else
	//			hull_ = 1;
	//	}

	//	hull[i] = hull_;
	//}

	// Transform convex template to gravity RF.

	Array<RECOG::PSGM_::Plane> convexTemplateC;

	convexTemplateC.Element = new RECOG::PSGM_::Plane[convexTemplate.n];

	float *NC;

	for (i = 0; i < convexTemplate.n; i++)
	{
		N = convexTemplate.Element[i].N;

		NC = convexTemplateC.Element[i].N;

		RVLMULMX3X3VECT(RGC, N, NC);
	}

	/// Identify the symmetry plane.

	//BYTE *bVisible = new BYTE[nSymmetryPlanes];
	
	Array<RECOG::PSGM_::SymmetryMatch> symmetryMatch_;

	symmetryMatch_.Element = new RECOG::PSGM_::SymmetryMatch[convexTemplate.n];

	Array<SortIndex<float>> sortedSymmatryMatchIdx;

	sortedSymmatryMatchIdx.Element = new SortIndex<float>[convexTemplate.n];

	float *P2RMem = new float[3 * iVertexArray[1].n];
	
	float maxSymmetryScore = 0.0f;

	int iSymmetryPlane;
	float *NSymmetryPlaneG, *P;
	float NSymmetryPlaneC[3], NR[3];
	float k, d, absk;
	int iVertex;
	float *P2R;
	int j, jBest;
	float dMax;
	float sumw, halfSumw, t_;
	float fTmp;
	float symmetryScore;
	RECOG::PSGM_::SymmetryMatch *pSymmetryMatch;
	float e;
	int iBestSymmetryPlane;
	float dBestSymmetryPlane;
	RECOG::PSGM_::ModelInstanceElement *pCTIElement1;

	for (iSymmetryPlane = 0; iSymmetryPlane < iSymmetryPlanes.n; iSymmetryPlane++)
	//iSymmetryPlane = 15;
	{
		NSymmetryPlaneG = convexTemplate.Element[iSymmetryPlanes.Element[iSymmetryPlane]].N;

		RVLMULMX3X3VECT(RGC, NSymmetryPlaneG, NSymmetryPlaneC);

		// Compute mirror images of all vertices of iObject2.

		P2R = P2RMem;

		for (i = 0; i < nVertices2; i++, P2R += 3)
		{
			iVertex = iVertexArray[1].Element[i];

			P = pSurfels->vertexArray.Element[iVertex]->P;

			d = 2.0f * RVLDOTPRODUCT3(P, NSymmetryPlaneC);

			RVLSCALE3VECTOR(NSymmetryPlaneC, d, P2R);

			RVLDIF3VECTORS(P, P2R, P2R);
		}

		//memset(bVisible, 0, nSymmetryPlanes * sizeof(BYTE));
		
		// For every element of CTI of iObject1 identify the corresponding tangent to mirror images of the vertices of iObject2.

		symmetryMatch_.n = 0;

		sortedSymmatryMatchIdx.n = 0;

		sumw = 0.0f;

		pCTIElement1 = pCTI->modelInstance.Element;

		for (i = 0; i < convexTemplate.n; i++, pCTIElement1++)
		{
			if (!pCTIElement1->valid)
				continue;

			NC = convexTemplateC.Element[i].N;			

			//if (i < nSymmetryPlanes)
			//{
			//	k = RVLDOTPRODUCT3(NSymmetryPlaneG, N);

			//	bVisible[i] = (k > 0.0f ? 1 : -1);
			//}
			//else
			//{
			//	if (bVisible[i - nSymmetryPlanes] > 0)
			//		continue;
			//}

			k = RVLDOTPRODUCT3(NSymmetryPlaneC, NC);

			if (k > -1e-6)
			{
				fTmp = 2.0f * k;

				RVLSCALE3VECTOR(NSymmetryPlaneC, fTmp, NR);

				RVLDIF3VECTORS(NC, NR, NR);

				P2R = P2RMem;

				dMax = RVLDOTPRODUCT3(NC, P2R);

				jBest = 0;

				for (j = 1; j < nVertices2; j++, P2R += 3)
				{
					d = RVLDOTPRODUCT3(NC, P2R);

					if (d > dMax)
					{
						dMax = d;

						jBest = j;
					}
				}

				//iVertex = iVertexArray[1].Element[jBest];

				//P = pSurfels->vertexArray.Element[iVertex]->P;

				//if (RVLDOTPRODUCT3(P, NR) < 0.0f)
				{
					absk = RVLABS(k);

					pSymmetryMatch = symmetryMatch_.Element + symmetryMatch_.n;

					pSymmetryMatch->d = dMax;
					pSymmetryMatch->w = k;
					pSymmetryMatch->iCTIElement = i;
					pSymmetryMatch->b = (absk > 1e-6);	// normal of the CTI element is not parallel to the symmetry plane

					if (pSymmetryMatch->b)
					{
						sortedSymmatryMatchIdx.Element[sortedSymmatryMatchIdx.n].cost = (pCTIElement1->d - dMax) / k;
						sortedSymmatryMatchIdx.Element[sortedSymmatryMatchIdx.n].idx = symmetryMatch_.n;
						sortedSymmatryMatchIdx.n++;
						sumw += absk;
					}

					symmetryMatch_.n++;
				}
			}	// if (NSymmetryPlaneC' * NC > -1e-6)
		}	// for every element of convexTemplate

		// Compute optimal symmetry plane offset.

		if (sortedSymmatryMatchIdx.n > 0)
		{
			BubbleSort<SortIndex<float>>(sortedSymmatryMatchIdx);

			halfSumw = 0.5f * sumw;

			sumw = 0.0f;

			for (i = 0; i < sortedSymmatryMatchIdx.n && sumw < halfSumw; i++)
			{
				k = symmetryMatch_.Element[sortedSymmatryMatchIdx.Element[i].idx].w;

				sumw += RVLABS(k);
			}
				
			t_ = sortedSymmatryMatchIdx.Element[i].cost;
		}

		// Compute symmetry score.		

		symmetryScore = 0.0f;

		for (i = 0; i < symmetryMatch_.n; i++)
		{
			pSymmetryMatch = symmetryMatch_.Element + i;

			e = (pSymmetryMatch->d + pSymmetryMatch->w * t_ - pCTI->modelInstance.Element[pSymmetryMatch->iCTIElement].d) / symmetryMatchThr;

			e *= e;

			if (e < 1.0f)
			{
				e = 1.0f - e;

				symmetryScore += e;

				pSymmetryMatch->w = e;
				pSymmetryMatch->b = true;
			}
			else
				pSymmetryMatch->b = false;
		}

		if (symmetryScore > maxSymmetryScore)
		{
			maxSymmetryScore = symmetryScore;

			iBestSymmetryPlane = iSymmetryPlane;

			dBestSymmetryPlane = -0.5f * t_;

			symmetryMatch.n = 0;

			for (i = 0; i < symmetryMatch_.n; i++)
				if (symmetryMatch_.Element[i].b)
					symmetryMatch.Element[symmetryMatch.n++] = symmetryMatch_.Element[i];
		}
	}	// for each symmetry plane

	///

	// Save the results to a file. 

	if (bDebug)
	{
		FILE *fp = fopen("symmetry.txt", "w");

		PrintMatrix<float>(fp, RGC, 3, 3);

		NSymmetryPlaneG = convexTemplate.Element[iSymmetryPlanes.Element[iBestSymmetryPlane]].N;

		//NSymmetryPlaneG = convexTemplate.Element[22].N;

		RVLMULMX3X3VECT(RGC, NSymmetryPlaneG, NSymmetryPlaneC);

		PrintMatrix<float>(fp, NSymmetryPlaneC, 1, 3);

		fprintf(fp, "%f\t%d\t%d\t\n", dBestSymmetryPlane, iVertexArray[0].n, iVertexArray[1].n);

		SURFEL::Vertex *pVertex;

		for (j = 0; j < 2; j++)
		{
			for (i = 0; i < iVertexArray[j].n; i++)
			{
				pVertex = pSurfels->vertexArray.Element[iVertexArray[j].Element[i]];

				fprintf(fp, "%f\t%f\t%f\t\n", pVertex->P[0], pVertex->P[1], pVertex->P[2]);
			}
		}

		fclose(fp);
	}

	// Free memory.

	//delete[] bVisible;
	delete[] P2RMem;
	delete[] symmetryMatch_.Element;
	delete[] iSymmetryPlanes.Element;
	delete[] sortedSymmatryMatchIdx.Element;

	// Return the symmetry score.

	return maxSymmetryScore;
}

void PSGM::RVLPSGInstanceMesh(Eigen::MatrixXf nI, float *dI)
{
#ifdef RVLPSGM_CTIMESH_DEBUG
	FILE *fp = fopen("CTIMeshDebug.txt", "w");
#endif

	//transform nI and dI to Eigen:
	Eigen::MatrixXf nIE = nI;
	Eigen::MatrixXf dIE(1, 66);

	//if nI was an array
	//Eigen::MatrixXf nIE(3, 66);
	//int br = 0;
	//for (int i = 0; i < 3; i++)
	//{
	//	for (int j = 0; j < 66; j++)
	//	{
	//		nIE(i, j) = nI[br];
	//		br++;
	//	}
	//}
	for (int i = 0; i < 66; i++)
	{
		dIE(0, i) = dI[i];
	}

	float noise = 1e-6;
	int nF = nIE.cols();
	float halfCubeSize;

	float max, min;
	max = dIE.maxCoeff();
	min = dIE.minCoeff();
	if (min<0 && min*-1 > max)
		max = min;
	halfCubeSize = max*1.1;

	P.resize(3, 8);
	P << 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1;
	P = halfCubeSize*P;

	Eigen::MatrixXi Premoved = Eigen::MatrixXi::Zero(1, P.cols());//list of removed vertices
	Eigen::MatrixXi nP = Eigen::MatrixXi::Ones(nF + 6, 1);
	nP = 4 * nP;

	F = Eigen::MatrixXi::Zero(nF + 6, 66 * 66);
	//F.block<6, 4>(0, 0) << 1, 3, 4, 2, 3, 7, 8, 4, 2, 4, 8, 6, 5, 6, 8, 7, 1, 2, 6, 5, 1, 5, 7, 3;
	F.block<6, 4>(0, 0) << 0, 2, 3, 1, 2, 6, 7, 3, 1, 3, 7, 5, 4, 5, 7, 6, 0, 1, 5, 4, 0, 4, 6, 2;


	Eigen::MatrixXi E = Eigen::MatrixXi::Ones(nF + 6, nF + 6);
	E *= -1;
	Eigen::MatrixXi Fn = Eigen::MatrixXi::Zero(nF + 6, 66 * 66);

	int iP1, iP2;
	int l;
	int br2;
	int NextCirc, PrevCirc;
	for (int i_ = 0; i_ < 6; i_++) //for every face
	{
		int i, j;
		i = i_;
		for (int k = 0; k < 4; k++)
		{
			NextCirc = (k + 1) % 4;
			iP1 = F(i, k);
			iP2 = F(i, NextCirc);

			for (int j_ = i_ + 1; j_ < 6; j_++)
			{
				j = j_;
				l = -1;
				for (int iF = 0; iF < nP(j); iF++) //find(F(j,:)==iP1)
				{
					if (F(j, iF) == iP1)
					{
						l = iF;
						break;
					}
				}
				if (l >= 0)
				{
					PrevCirc = (l + 3) % 4;
					if (F(j, PrevCirc) == iP2)
					{
						E(i, j) = iP1;
						E(j, i) = iP2;
						Fn(i, k) = j;
						Fn(j, PrevCirc) = i;
					}
				}
			}
		}
	}

#ifdef RVLPSGM_CTIMESH_DEBUG
	PrintCTIMeshFaces(fp, F, Fn, 6, nP);

	fprintf(fp, "\n\n\n");

	fclose(fp);
#endif

	Eigen::MatrixXi iNewVertices;
	Eigen::MatrixXi iNeighbors;
	Eigen::MatrixXf N;
	Eigen::MatrixXf dCut, dCutSorted;
	float d, d_;
	int nCut;
	Eigen::MatrixXf Temp;
	Eigen::MatrixXf Tempnext;
	Eigen::MatrixXf Temp2;
	for (int i = 0; i < nF; i++) //for every face
	{
		printf("%d\n", i);

#ifdef RVLPSGM_CTIMESH_DEBUG
		fprintf(fp, "i = %d\n\n\n", i);
#endif
		iNewVertices.resize(0, 0);
		iNeighbors.resize(0, 0);
		N = nIE.block<3, 1>(0, i); //normal of the i-th face
		d = dIE(0, i); //distance of the i-th face
		dCut = Eigen::MatrixXf::Zero(P.cols(), 1);
		dCutSorted = Eigen::MatrixXf::Zero(P.cols(), 1);
		nCut = 0;


		for (int k = 0; k < 8; k++)
		{
			Temp = N.transpose()*P.block<3, 1>(0, k);
			d_ = Temp(0, 0) - d;
			if (d_ > 0)
			{
				nCut += 1;
				dCut(nCut, 0) = d_;
			}
		}

		//Bubble sort:
		float temp;
		for (int idCut = 0; idCut < nCut; idCut++)
		{
			for (int jdCut = 0; jdCut < nCut; jdCut++)
			{
				if (dCut(jdCut, 0)>dCut(jdCut + 1, 0))
				{
					temp = dCut(jdCut, 0);
					dCut(jdCut, 0) = dCut(jdCut + 1, 0);
					dCut(jdCut + 1, 0) = temp;
				}
			}
		}

		float dCorr = 0;
		for (int k = 0; k < nCut; k++)
		{
			if (dCut(k, 0) - dCorr < noise)
				dCorr = dCut(k, 0);

		}
		d += dCorr;

		Eigen::MatrixXi F_;//j-th face
		Eigen::MatrixXi Fn_;//neighbors of F_
		Eigen::MatrixXf P_; //position vector of vector iP
		Eigen::MatrixXf Pnext;//position vector of vertex iPNext
		int nP_; //number of vertices od F_
		int iP; // k-th vertex of F_
		int iPNext; //next vertex
		Eigen::MatrixXf dP;
		int L; //neighbor of F_ on the opposite side of edge iP-iPNext

		int iPolygon, iVertex;
		int iPNew, iPNewVertex, iFNewVertex;
		float s;


		for (int j = 0; j <= i + 5; j++) //for every previously considered face
		{
			F_ = F.block(j, 0, 1, F.cols());
			Fn_ = Fn.block(j, 0, 1, Fn.cols());
			int ff = F(j, 0);
			int k = 0;
			nP_ = nP(j, 0);

			for (int k_ = 0; k_ < nP_; k_++)
			{
				int p = P.cols();
				iP = F_(0, k_);
				P_ = P.block(0, iP, P.rows(), 1);


				if (k_ == nP_ - 1) NextCirc = 0;
				else NextCirc = k_ % (nP_)+1;

				iPNext = F_(0, NextCirc);
				Pnext = P.block(0, iPNext, P.rows(), 1);

				dP.resize(P.rows(), 1);
				dP = Pnext - P_;

				L = Fn_(0, k_);

				Temp = N.transpose()*P_;
				Tempnext = N.transpose()*Pnext;

				if (i == 1 && j == 2)
					int debug = 1;


				if (Temp(0, 0) > d) //if iP is over new plane
				{
					if (Premoved(0, iP) == 0)
						Premoved(0, iP) = 1; //vertex iP is removed

					iPolygon = j;
					iVertex = k;

					//Remove Vertex from Polygon:
					for (int iF = 0; iF < (nP(iPolygon) - 1 - iVertex); iF++)
					{
						F(iPolygon, iVertex + iF) = F(iPolygon, iVertex + iF + 1);
						Fn(iPolygon, iVertex + iF) = Fn(iPolygon, iVertex + iF + 1);
					}
					nP(iPolygon) = nP(iPolygon) - 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
					fp = fopen("CTIMeshDebug.txt", "a");

					fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

					PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

					fprintf(fp, "\n\n\n");

					fclose(fp);
#endif

					if (Tempnext(0, 0) > d)
					{
						E(L, j) = -1;
						E(j, L) = -1;
						k -= 1;
					}
					else
					{
						if (E(j, L) == iP) //Vertex is not updated
						{
							//Add new vertex
							Temp = N.transpose()*P_;
							Temp2 = N.transpose()*dP;
							s = (d - Temp(0, 0)) / Temp2(0, 0);
							Eigen::MatrixXf Ptemp = P;
							P.resize(P.rows(), P.cols() + 1);
							P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
							Eigen::Vector3f col = P_ + s*dP;
							P.col(P.cols() - 1) = col;
							iPNew = P.cols() - 1;

							Eigen::MatrixXi Premovedtemp = Premoved;
							Premoved.resize(1, Premoved.cols() + 1);
							Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
							Premoved(0, Premoved.cols() - 1) = 0;
							E(j, L) = iPNew;
						}
						else
							iPNew = E(j, L);

						iPolygon = j;
						iVertex = k;
						iPNewVertex = iPNew;
						iFNewVertex = L;
						//Add new Vertex to Polygon:
						Eigen::MatrixXi Ftemp = F;
						Eigen::MatrixXi Fntemp = Fn;
						for (int iF = 0; iF < (nP(iPolygon) - iVertex); iF++)
						{
							F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon, iVertex + iF);
							Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
						}
						F(iPolygon, iVertex) = iPNewVertex;
						Fn(iPolygon, iVertex) = iFNewVertex;
						nP(iPolygon) = nP(iPolygon) + 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
						fp = fopen("CTIMeshDebug.txt", "a");

						fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

						PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

						fprintf(fp, "\n\n\n");

						fclose(fp);
#endif

						Eigen::MatrixXi iNewVerticestemp = iNewVertices;
						iNewVertices.resize(1, iNewVertices.cols() + 1);
						iNewVertices.block(0, 0, iNewVerticestemp.rows(), iNewVerticestemp.cols()) = iNewVerticestemp;
						iNewVertices(0, iNewVertices.cols() - 1) = iPNew;


						Eigen::MatrixXi iNeighborstemp = iNeighbors;
						iNeighbors.resize(1, iNeighbors.cols() + 1);
						iNeighbors.block(0, 0, iNeighborstemp.rows(), iNeighborstemp.cols()) = iNeighborstemp;
						iNeighbors(0, iNeighbors.cols() - 1) = j;

						E((i + 6), j) = iPNew;
					}
				}

				else if (Tempnext(0, 0) > d)
				{
					if (E(L, j) == iPNext) //Vertex is not updated
					{
						Temp = N.transpose()*P_;
						Temp2 = N.transpose()*dP;
						s = (d - Temp(0, 0)) / Temp2(0, 0);


						Eigen::MatrixXf Ptemp = P;
						P.resize(P.rows(), P.cols() + 1);
						P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
						Eigen::Vector3f col = P_ + s*dP;
						P.col(P.cols() - 1) = col;
						iPNew = P.cols() - 1;

						Eigen::MatrixXi Premovedtemp = Premoved;
						Premoved.resize(1, Premoved.cols() + 1);
						Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
						Premoved(0, Premoved.cols() - 1) = 0;

						E(L, j) = iPNew;
					}
					else
						iPNew = E(L, j);

					iPolygon = j;
					iVertex = k + 1;
					iPNewVertex = iPNew;
					iFNewVertex = i + 6;
					//Add new Vertex to Polygon:
					Eigen::MatrixXi Ftemp = F;
					Eigen::MatrixXi Fntemp = Fn;
					for (int iF = 0; iF < (nP(iPolygon) - iVertex); iF++)
					{
						F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon, iVertex + iF);
						Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
					}
					F(iPolygon, iVertex) = iPNewVertex;
					Fn(iPolygon, iVertex) = iFNewVertex;
					nP(iPolygon) = nP(iPolygon) + 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
					fp = fopen("CTIMeshDebug.txt", "a");

					fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

					PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

					fprintf(fp, "\n\n\n");

					fclose(fp);
#endif

					E(j, i + 6) = iPNew;
					k = k + 1;
				}
				k = k + 1;
			}

#ifdef RVLPSGM_CTIMESH_DEBUG
			fp = fopen("CTIMeshDebug.txt", "a");

			fprintf(fp, "j = %d\n\n", j);

			PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

			fprintf(fp, "\n\n\n");

			fclose(fp);
#endif
		}	 //for every previously considered face
		int iNeighbor;
		nP(i + 6) = iNewVertices.cols(); //rows

		int m;
		if (nP(i + 6) > 0)
		{
			Eigen::MatrixXi F_;
			Eigen::MatrixXi Fn_;
			m = 0;
			while (1)
			{
				Eigen::MatrixXi F_temp = F_;
				F_.resize(1, F_.cols() + 1);
				F_.block(0, 0, F_temp.rows(), F_temp.cols()) = F_temp;
				F_(0, F_.cols() - 1) = iNewVertices(m);

				int iN = iNeighbors(0, m);
				iNeighbor = iNeighbors(0, m);


				Eigen::MatrixXi Fn_temp = Fn_;
				Fn_.resize(1, Fn_.cols() + 1);
				Fn_.block(0, 0, Fn_temp.rows(), Fn_temp.cols()) = Fn_temp;
				Fn_(0, Fn_.cols() - 1) = iNeighbor;


				iPNext = E(iNeighbor, (i + 6));
				int iNV;
				for (iNV = 0; iNV < iNewVertices.cols(); iNV++)
				{
					int a = iPNext;
					int b = iNewVertices(iNV);
					if (iNewVertices(iNV) == iPNext)
					{
						m = iNV;
						break;
					}
				}
				if (m == 0)
					break;
			}

			for (int iF = 0; iF < F_.cols(); iF++)
			{
				F((i + 6), iF) = F_(0, iF);
				Fn((i + 6), iF) = Fn_(0, iF);
			}

		}

#ifdef RVLPSGM_CTIMESH_DEBUG
		fp = fopen("CTIMeshDebug.txt", "a");

		PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

		fprintf(fp, "\n\n\n");

		fclose(fp);
#endif
	}	 //for every face
	int mF = F.cols();

	for (int i = 0; i < F.rows(); i++)
	{
		for (int iF = 0; iF < mF; iF++)
			F(i, nP(i) + 1 + iF) = 0;
	}

	Eigen::MatrixXi Ffinal = F.block((F.rows() - 6), F.cols(), 6, 0);
	Edges = E;

#ifdef RVLPSGM_CTIMESH_DEBUG
	fclose(fp);
#endif
}

void PSGM::PrintCTIMeshFaces(FILE *fp, Eigen::MatrixXi F, Eigen::MatrixXi Fn, int n, Eigen::MatrixXi nP)
{
	fprintf(fp, "F:\n");

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < nP(i); j++)
			fprintf(fp, "%d\t", F(i, j));

		fprintf(fp, "\n");
	}

	fprintf(fp, "\n");

	fprintf(fp, "Fn:\n");

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < nP(i); j++)
			fprintf(fp, "%d\t", Fn(i, j));

		fprintf(fp, "\n");
	}

	fprintf(fp, "\n");
}