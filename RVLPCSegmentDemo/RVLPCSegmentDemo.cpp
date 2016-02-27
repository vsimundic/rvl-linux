// RVLPCSegmentDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"

using namespace RVL;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("MeshFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	CRVLMem mem;	// cycle memory

	mem.Create(100000000);

	// Read parameters from a configuration file.

	char *MeshFileName = NULL;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &MeshFileName);

	ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	// Read mesh from file.

	Mesh mesh;

	mesh.LoadFromPLY(MeshFileName);

	// Segment mesh to surfels.

	SurfelGraph surfels;

	surfels.Init(mesh.NodeArray.n);

	PlanarSurfelDetector detector;

	detector.CreateParamList(&mem0);

	detector.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	detector.Init(&mesh, &surfels, &mem);

	detector.Segment(&mesh, &surfels);

	// Display mesh.

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	surfels.NodeColors(SelectionColor);	

	Visualizer visualizer;	

	visualizer.Create();
	surfels.InitDisplay(&visualizer, &mesh, &detector);
	surfels.Display(&visualizer, &mesh);
	visualizer.Run();

	// free memory

	return 0;
}

