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
#include "PSGM.h"
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"

//test za commit

// VIDOVIC
//#define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST
//#define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST_DEBUG
//#define RVL_FEATURE_TEST_SCENE_SEQUENCE
//#define RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH
#define RVL_LOAD_SINGLE_MODEL

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
		method,
		flags);	 //VIDOVIC

	ParamList.LoadParams("RVLRecognitionDemo.cfg");

	// Initialize surfel detection

	SurfelGraph surfels;

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
			recognition.Learn(modelSequenceFileName); //VIDOVIC
		}
		else if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			recognition.LoadModelDataBase(); //VIDOVIC

			// Load scene mesh from file.

			Mesh mesh;

			//mesh.LoadPolyDataFromPLY(sceneMeshFileName);

			//VIDOVIC

			//ECCVGTLoader TEST
			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			recognition.SetNumberOfScenes(sceneSequence.nFileNames);

			ECCVGTLoader ECCVGT;

			ECCVGT.Init(sceneSequence, GTFolder, modelsInDB);

			ECCVGT.SaveGTFile("F:\\Projekti\\ARP3D\\Auxiliary\\Models\\GT.txt");

			char filePath[200];

			while (sceneSequence.GetNextPath(filePath))
			{
				printf("Scene %s...\n", filePath);

				mesh.LoadPolyDataFromPLY(filePath);

				recognition.SetSceneFileName(filePath);
				recognition.Interpret(&mesh);

				printf("Scene %s...finished!\n\n", filePath);
			}

			recognition.SaveMatches();

			float precision, recall;
			float scoreThresh, angleThresh, distanceThresh;

			scoreThresh = 46.5;
			angleThresh = PI/4;
			distanceThresh = 50;

			FILE *fp;

			int graphID = 0;

			fp = fopen("F:\\Projekti\\ARP3D\\compare.txt", "w");

			//for (angleThresh = PI / 4; angleThresh < 3*PI/4; angleThresh += PI / 4)
			//{
				for (distanceThresh = 50; distanceThresh <= 100; distanceThresh += 25)
				{
					printf("ScoreThresh: %f\t%f\n", angleThresh, distanceThresh);

					for (scoreThresh = 33.0; scoreThresh <= 66; scoreThresh += 0.1)
					{
						//recognition.CompareMatchesToGT(&ECCVGT, scoreThresh, angleThresh, distanceThresh, precision, recall);

						recognition.CompareSMIMatchesToGT(&ECCVGT, scoreThresh, angleThresh, distanceThresh, precision, recall);

						ECCVGT.ResetMatchFlag();

						printf("ScoreThresh: %f\n", scoreThresh);
						printf("Precision: %f\n", precision);
						printf("Recall: %f\n", recall);
						printf("\n");

						fprintf(fp, "%d\t%f\t%f\t%f\t%f\t%f\n", graphID, angleThresh, distanceThresh, scoreThresh, precision, recall);
					}

					graphID++;

				}
			//}

			fclose(fp);

			recognition.SaveMatches();

			//END VIDOVIC

			// Scene interpretation.

			//recognition.SetSceneFileName(sceneMeshFileName);
			//recognition.Interpret(&mesh);

			// Visualization

			surfels.NodeColors(SelectionColor);
			recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
			recognition.Display();
			visualizer.Run();
		}	// if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		else if (recognition.mode == RVLRECOGNITION_MODE_PSGM_CREATE_CTIS)
		{
			Mesh mesh;

			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			recognition.SetNumberOfScenes(sceneSequence.nFileNames);

			char filePath[200];

			while (sceneSequence.GetNextPath(filePath))
			{
				printf("Scene %s...\n", filePath);

				mesh.LoadPolyDataFromPLY(filePath);

				recognition.SetSceneFileName(filePath);
				recognition.Interpret(&mesh);

				printf("Scene %s...finished!\n\n", filePath);
			}

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

	//END VIDOVIC

	return 0;
}

