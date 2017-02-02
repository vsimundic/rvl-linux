// RVLRecognitionDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RFRecognition.h"
#include "RVLMeshNoiser.h"
#include "CTISet.h"
#include "PSGM.h"
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"

// VIDOVIC
//#define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST
//#define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST_DEBUG
#define RVL_FEATURE_TEST_SCENE_SEQUENCE
//#define RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH
#define RVL_LOAD_SINGLE_MODEL
//#define PSGM_MATCHES_PROBABILITY_COMPARE
#define PSGM_MATCHES_SCORE_COMPARE
#define PSGM_LOAD_CTI_FROM_FILE

#define RVLRECOGNITION_DEMO_FLAG_SAVE_PLY			0x00000001
//END VIDOVIC

using namespace RVL;

#define RVLRECOGNITION_METHOD_RF		0
#define RVLRECOGNITION_METHOD_PSGM		1

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	char **pSceneSequenceFileName,	//VIDOVIC
	char **pModelSequenceFileName,	//VIDOVIC
	char **pModelsInDB,	//VIDOVIC
	char **pGTFolder,	//VIDOVIC
	char **pSegmentGTFileName,	//Vidovic
	DWORD &method,
	DWORD &flags //VIDOVIC
	)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName);	//VIDOVIC
	pParamData = pParamList->AddParam("ModelSequenceFileName", RVLPARAM_TYPE_STRING, pModelSequenceFileName);	//VIDOVIC
	pParamData = pParamList->AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, pModelsInDB);	//VIDOVIC
	pParamData = pParamList->AddParam("GTFolder", RVLPARAM_TYPE_STRING, pGTFolder);	//VIDOVIC
	pParamData = pParamList->AddParam("SegmentGTFileName", RVLPARAM_TYPE_STRING, pSegmentGTFileName);	//Vidovic
	pParamData = pParamList->AddParam("Recognition.method", RVLPARAM_TYPE_ID, &method);
	pParamList->AddID(pParamData, "PSGM", RVLRECOGNITION_METHOD_PSGM);
	pParamList->AddID(pParamData, "RF", RVLRECOGNITION_METHOD_RF); //VIDOVIC
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_ID, &flags); //VIDOVIC
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_SAVE_PLY); //VIDOVIC
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000000);
	//mem0.Create(100000000000); //VIDOVIC

	CRVLMem mem;	// cycle memory

	mem.Create(1000000000);
	//mem.Create(100000000000); //VIDOVIC

	// Read parameters from a configuration file.

	char *sceneMeshFileName = NULL;
	char *sceneSequenceFileName = NULL; //VIDOVIC
	char *modelSequenceFileName = NULL; //VIDOVIC
	char *modelsInDB = NULL; //VIDOVIC
	char *GTFolder = NULL; //VIDOVIC
	char *segmentGTFileName = NULL; //Vidovic
	DWORD method = RVLRECOGNITION_METHOD_PSGM;
	//DWORD method = RVLRECOGNITION_METHOD_RF; //VIDOVIC

	DWORD flags = 0x00000000; //VIDOVIC

	CRVLParameterList ParamList;

	CreateParamList(&ParamList,
		&mem0,
		&sceneMeshFileName,
		&sceneSequenceFileName,
		&modelSequenceFileName,
		&modelsInDB,
		&GTFolder,
		&segmentGTFileName,
		method,
		flags);	 //VIDOVIC

	ParamList.LoadParams("RVLRecognitionDemo.cfg");

	if (segmentGTFileName == NULL)
	{
		segmentGTFileName = new char[200];
		segmentGTFileName = "C:\\RVL\\segmentGT.txt";
	}

	// Initialize surfel detection

	SurfelGraph surfels;

	surfels.pMem = &mem;

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams("RVLRecognitionDemo.cfg");

	PlanarSurfelDetector surfelDetector;

	surfelDetector.CreateParamList(&mem0);

	surfelDetector.ParamList.LoadParams("RVLRecognitionDemo.cfg");

	//VIDOVIC
	//initialize mesh noiser
	MeshNoiser noiser;

	noiser.SetParam(1, 0.05);
	//END VIDOVIC

	// Initialize visualization

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	Visualizer visualizer;

	visualizer.Create();

	if (method == RVLRECOGNITION_METHOD_RF)
	{
		// Initialize recognition.

		RFRecognition recognition;

		recognition.CreateParamList(&mem0);

		recognition.ParamList.LoadParams("RVLRecognitionDemo.cfg");

		recognition.pMem0 = &mem0;
		recognition.pMem = &mem;

		recognition.pSurfels = &surfels;

		recognition.pSurfelDetector = &surfelDetector;

		// Training or recognition (depending on mode).

		if (recognition.mode == RVLRECOGNITION_MODE_TRAINING)
			recognition.CreateModelDatabase();
		else if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			if (!recognition.LoadModelDatabase())
				return 1;

			Mesh mesh;

			//VIDOVIC
#ifdef RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST

			recognition.CoordinateSystemNoiseStabilityTest(sceneMeshFileName, noiser, 0);

#endif // RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST

#ifdef RVL_FEATURE_TEST_SCENE_SEQUENCE

			recognition.FeatureTestSceneSequence(sceneSequenceFileName, noiser);

#endif // RVL_FEATURE_TEST_SCENE_SEQUENCE

#ifdef RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH

			recognition.FeatureTestPrecisionRecallGraph(sceneMeshFileName, sceneSequenceFileName, noiser);

#endif // RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH

#ifdef RVL_LOAD_SINGLE_MODEL


			//VIDOVIC
			RECOG::Hypothesis *pBestHypothesis = NULL;
			float V[3], theta;
			float distance;
			//END VIDOVIC

			mesh.LoadPolyDataFromPLY(sceneMeshFileName);

			recognition.FindObjects(&mesh);

			//VIDOVIC
			recognition.FindBestHypothesis(&pBestHypothesis);

			GetAngleAxis(pBestHypothesis->R, V, theta);
			GetDistance(pBestHypothesis->t, distance);

			FILE *fpHypothesisErrorDebug = NULL;

			fpHypothesisErrorDebug = fopen("C:\\RVL\\Debug\\hypothesisErrorDebug.txt", "w");
			//END VIDOVIC

			FILE *fpInterpretation = fopen("C:\\RVL\\Debug\\interpretation.txt", "w");

			RECOG::Hypothesis *pHypothesis = recognition.sceneInterpretation.pFirst;

			while (pHypothesis)
			{
				RECOG::WriteHypothesis(fpInterpretation, pHypothesis);

				//VIDOVIC
				GetAngleAxis(pHypothesis->R, V, theta);
				GetDistance(pHypothesis->t, distance);

				RECOG::WriteHypothesisError(fpHypothesisErrorDebug, pHypothesis, distance, theta * 180 / PI);
				//END VIDOVIC

				pHypothesis = pHypothesis->pNext;
			}

			fclose(fpInterpretation);
			fclose(fpHypothesisErrorDebug); //VIDOVIC

#endif // RVL_LOAD_SINGLE_MODEL
			//END VIDOVIC

			// Visualization

			surfels.NodeColors(SelectionColor); //VIDOVIC
			recognition.InitDisplay(&visualizer, &mesh);
			recognition.Display();
			visualizer.Run();
		}
	}	// if (method == RVLRECOGNITION_METHOD_RF)
	else if (method == RVLRECOGNITION_METHOD_PSGM)
	{
		// Initialize recognition.

		PSGM recognition;

		recognition.CreateParamList(&mem0);

		recognition.ParamList.LoadParams("RVLRecognitionDemo.cfg");

		recognition.pMem = &mem;

		recognition.pSurfels = &surfels;

		recognition.pSurfelDetector = &surfelDetector;

		if (recognition.mode == RVLRECOGNITION_MODE_TRAINING)
		{
			surfels.NodeColors(SelectionColor);

			recognition.Learn(modelSequenceFileName, &visualizer); //VIDOVIC
		}
		else if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			recognition.LoadModelDataBase(); //VIDOVIC

			recognition.CreateMatchMatrix();

			Mesh mesh;

			//VIDOVIC
			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			recognition.pECCVGT->Init(sceneSequence, GTFolder, modelsInDB);

			recognition.pECCVGT->SaveGTFile("F:\\Projekti\\ARP3D\\Auxiliary\\Models\\TUW_GT.txt");

			char filePath[200];

			FILE *fpHypothesisEvaluation = fopen("F:\\Projekti\\ARP3D\\compare_TNM_Valid.txt", "w");

			FILE *fpSegmentGT = fopen(segmentGTFileName, "r");

			if (fpSegmentGT == NULL)
			{
				fpSegmentGT = fopen(segmentGTFileName, "w");
				recognition.createSegmentGT = true;
			}

			FILE *fpLog = fopen("F:\\Projekti\\ARP3D\\evaluationLog.txt", "w");

			//Move to some PSGM MatchInit function
			recognition.segmentGT.Element = new RVL::SegmentGTInstance[recognition.nDominantClusters * sceneSequence.nFileNames];
			recognition.segmentGT.n = recognition.nDominantClusters * sceneSequence.nFileNames;

			char *CTIFileName = NULL;

			bool CTIFromFile = false;

			recognition.pTimer = new CRVLTimer;

			recognition.LoadCompleteSegmentGT(fpSegmentGT);

			while (sceneSequence.GetNextPath(filePath))
			{
				printf("Scene %s...\n", filePath);

				recognition.SetSceneFileName(filePath);

#ifdef PSGM_LOAD_CTI_FROM_FILE
				RVLCopyString(filePath, &CTIFileName);

				sprintf(RVLGETFILEEXTENSION(CTIFileName), "cti");

				recognition.LoadCTI(CTIFileName);

				recognition.Match(true);

				CTIFromFile = true;
#else
				mesh.LoadPolyDataFromPLY(filePath);

				mem.Clear();

				recognition.Interpret(&mesh);
#endif				

				if (recognition.createSegmentGT)
					recognition.SaveSegmentGT(fpSegmentGT, CTIFromFile);
				//else
					//recognition.LoadSegmentGT(fpSegmentGT, CTIFromFile);

				recognition.EvaluateMatchesByScore_(fpHypothesisEvaluation, fpLog, 7);
				
				printf("Scene %s...finished!\n\n", filePath);
			}

			RVL_DELETE_ARRAY(CTIFileName);

			RVL_DELETE_ARRAY(recognition.pTimer);

			RVL_DELETE_ARRAY(recognition.segmentGT.Element);

			fclose(fpHypothesisEvaluation);
			fclose(fpSegmentGT);
			fclose(fpLog);

			//END VIDOVIC

			// Scene interpretation.

			//recognition.SetSceneFileName(sceneMeshFileName);
			//recognition.Interpret(&mesh);

			// Visualization

			//surfels.NodeColors(SelectionColor);
			//recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
			//recognition.Display();
			//visualizer.Run();
		}	// if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		else if (recognition.mode == RVLRECOGNITION_MODE_PSGM_CREATE_CTIS)
		{
			Mesh mesh;

			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			recognition.pECCVGT->Init(sceneSequence, GTFolder, modelsInDB);

			char *clusterNormalDistributionFileName = NULL;

			int iScene = 0;

			char filePath[200];
			FILE *fpClusterNormalDistribution;

			while (sceneSequence.GetNextPath(filePath))
			{
				printf("Scene %s...\n", filePath);

				mesh.LoadPolyDataFromPLY(filePath);

				recognition.SetSceneFileName(filePath);
				recognition.Interpret(&mesh, iScene);

				RVLCopyString(filePath, &clusterNormalDistributionFileName);

				sprintf(RVLGETFILEEXTENSION(clusterNormalDistributionFileName), "seg");

				fpClusterNormalDistribution = fopen(clusterNormalDistributionFileName, "w");

				recognition.WriteClusterNormalDistribution(fpClusterNormalDistribution);

				fclose(fpClusterNormalDistribution);

				printf("Scene %s...finished!\n\n", filePath);

				iScene++;
			}

			RVL_DELETE_ARRAY(clusterNormalDistributionFileName);

			// Visualization

			surfels.NodeColors(SelectionColor);
			recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
			recognition.Display();
			visualizer.Run();
		}
	}	// if (method == RVLRECOGNITION_METHOD_PSGM)

	// free memory

	if (sceneMeshFileName)
		delete[] sceneMeshFileName;

	//VIDOVIC
	if (sceneSequenceFileName)
		delete[] sceneSequenceFileName;

	if (modelSequenceFileName)
		delete[] modelSequenceFileName;

	if (segmentGTFileName)
		delete[] segmentGTFileName;

	//END VIDOVIC

	return 0;
}

