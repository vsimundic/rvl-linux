// RVLRecognitionDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Graph.h"
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "PCLMeshBuilder.h"
#include "RGBDCamera.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RFRecognition.h"
#include "PSGM.h"

using namespace RVL;

#define RVLRECOGNITION_METHOD_RF		0
#define RVLRECOGNITION_METHOD_PSGM		1

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	DWORD &method)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("Recognition.method", RVLPARAM_TYPE_ID, &method);
	pParamList->AddID(pParamData, "PSGM", RVLRECOGNITION_METHOD_PSGM);
	//pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_ID, &flags);
	//pParamList->AddID(pParamData, "yes", RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY);
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	CRVLMem mem;	// cycle memory

	mem.Create(100000000);

	// Read parameters from a configuration file.

	char *sceneMeshFileName = NULL;
	DWORD method = RVLRECOGNITION_METHOD_PSGM;

	////DWORD flags = 0x00000000;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &sceneMeshFileName, method);

	ParamList.LoadParams("RVLRecognitionDemo.cfg");

	// Initialize surfel detection

	SurfelGraph surfels;

	PlanarSurfelDetector surfelDetector;

	surfelDetector.CreateParamList(&mem0);

	surfelDetector.ParamList.LoadParams("RVLRecognitionDemo.cfg");

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

			mesh.LoadPolyDataFromPLY(sceneMeshFileName);

			recognition.FindObjects(&mesh);

			FILE *fpInterpretation = fopen("C:\\RVL\\Debug\\interpretation.txt", "w");

			RECOG::Hypothesis *pHypothesis = recognition.sceneInterpretation.pFirst;

			while (pHypothesis)
			{
				RECOG::WriteHypothesis(fpInterpretation, pHypothesis);

				pHypothesis = pHypothesis->pNext;
			}

			fclose(fpInterpretation);

			// Visualization

			recognition.InitDisplay(&visualizer, &mesh);
			recognition.Display();
			visualizer.Run();
		}
	}	// if (method == RVLRECOGNITION_METHOD_RF)
	else if (method == RVLRECOGNITION_METHOD_PSGM)
	{
		// Initialize recognition.

		PSGM recognition;

		recognition.pMem = &mem;

		recognition.pSurfels = &surfels;

		recognition.pSurfelDetector = &surfelDetector;

		// Load scene mesh from file.

		Mesh mesh;

		mesh.LoadPolyDataFromPLY(sceneMeshFileName);

		// Scene interpretation.

		recognition.Interpret(&mesh);

		// Visualization

		surfels.NodeColors(SelectionColor);
		recognition.InitDisplay(&visualizer, &mesh);
		recognition.Display();
		visualizer.Run();
	}	// if (method == RVLRECOGNITION_METHOD_PSGM)

	// free memory

	if (sceneMeshFileName)
		delete[] sceneMeshFileName;

	return 0;
}

