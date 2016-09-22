// RVLRecognitionDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
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

// VIDOVIC
//#define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST
//#define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST_DEBUG
//#define RVL_FEATURE_TEST_SCENE_SEQUENCE
//#define RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH
#define RVL_LOAD_SINGLE_MODEL
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
	DWORD &method)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName);	//VIDOVIC
	pParamData = pParamList->AddParam("ModelSequenceFileName", RVLPARAM_TYPE_STRING, pModelSequenceFileName);	//VIDOVIC
	pParamData = pParamList->AddParam("Recognition.method", RVLPARAM_TYPE_ID, &method);
	pParamList->AddID(pParamData, "PSGM", RVLRECOGNITION_METHOD_PSGM);
	pParamList->AddID(pParamData, "RF", RVLRECOGNITION_METHOD_RF); //VIDOVIC
	//pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_ID, &flags);
	//pParamList->AddID(pParamData, "yes", RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY);
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000000);

	CRVLMem mem;	// cycle memory

	mem.Create(1000000000);

	// Read parameters from a configuration file.

	char *sceneMeshFileName = NULL;
	char *sceneSequenceFileName = NULL; //VIDOVIC
	char *modelSequenceFileName = NULL; //VIDOVIC
	DWORD method = RVLRECOGNITION_METHOD_PSGM;
	//DWORD method = RVLRECOGNITION_METHOD_RF; //VIDOVIC

	////DWORD flags = 0x00000000;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList,
		&mem0,
		&sceneMeshFileName,
		&sceneSequenceFileName,
		&modelSequenceFileName,
		method);	 //VIDOVIC

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

			recognition.GetAngleAxis(pBestHypothesis->R, V, theta);
			recognition.GetDistance(pBestHypothesis->t, distance);

			FILE *fpHypothesisErrorDebug = NULL;

			fpHypothesisErrorDebug = fopen("C:\\RVL\\Debug\\hypothesisErrorDebug.txt", "w");
			//END VIDOVIC

			FILE *fpInterpretation = fopen("C:\\RVL\\Debug\\interpretation.txt", "w");

			RECOG::Hypothesis *pHypothesis = recognition.sceneInterpretation.pFirst;

			while (pHypothesis)
			{
				RECOG::WriteHypothesis(fpInterpretation, pHypothesis);

				//VIDOVIC
				recognition.GetAngleAxis(pHypothesis->R, V, theta);
				recognition.GetDistance(pHypothesis->t, distance);

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
			// Poziv funkcije LoadModelDatabase()

			// Load scene mesh from file.

			Mesh mesh;

			mesh.LoadPolyDataFromPLY(sceneMeshFileName);

			// Scene interpretation.

			recognition.SetSceneFileName(sceneMeshFileName);
			recognition.Interpret(&mesh);

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

