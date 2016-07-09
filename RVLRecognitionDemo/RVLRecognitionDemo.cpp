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

using namespace RVL;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
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

	////DWORD flags = 0x00000000;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &sceneMeshFileName);

	ParamList.LoadParams("RVLRecognitionDemo.cfg");

	// Initialize recognition.

	RFRecognition recognition;

	recognition.CreateParamList(&mem0);

	recognition.ParamList.LoadParams("RVLRecognitionDemo.cfg");

	recognition.pMem0 = &mem0;
	recognition.pMem = &mem;

	SurfelGraph surfels;

	recognition.pSurfels = &surfels;

	PlanarSurfelDetector surfelDetector;

	surfelDetector.CreateParamList(&mem0);

	surfelDetector.ParamList.LoadParams("RVLRecognitionDemo.cfg");

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

		unsigned char SelectionColor[3];

		SelectionColor[0] = 0;
		SelectionColor[1] = 255;
		SelectionColor[2] = 0;

		surfels.NodeColors(SelectionColor);

		Visualizer visualizer;

		visualizer.Create();

		recognition.InitDisplay(&visualizer, &mesh);
		recognition.Display();
		visualizer.Run();
	}		

	// free memory

	return 0;
}

