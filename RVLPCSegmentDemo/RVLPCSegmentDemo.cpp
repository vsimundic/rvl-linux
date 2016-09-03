// RVLPCSegmentDemo.cpp : Defines the entry point for the console application.
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
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"

//#define RVLPCSEGMENT_DEMO_CREATE_TRAINING_DATA

#define RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY			0x00000001

using namespace RVL;

#include "RVLPCSegmentCreateTrainingData.h"

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	DWORD &flags)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("MeshFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_ID, &flags);
	pParamList->AddID(pParamData, "yes", RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY);
}

int main(int argc, char ** argv)
{
#ifdef RVLPCSEGMENT_DEMO_CREATE_TRAINING_DATA
	RunSeg2Bench(true);
	//SceneSegFile::SceneSegFile* ssf = new SceneSegFile::SceneSegFile("test");
	///*ssf = SceneSegFile::GenerateTestSceneSegFile();
	//ssf->Save("test.ssf");*/
	//ssf->Load("test.ssf");
#else
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	CRVLMem mem;	// cycle memory

	mem.Create(100000000);

	// Read parameters from a configuration file.

	char *MeshFileName = NULL;

	DWORD flags = 0x00000000;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &MeshFileName, flags);

	ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	// Read mesh from file.

	PCLMeshBuilder meshBuilder;

	meshBuilder.CreateParamList(&mem0);

	meshBuilder.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	int w = 640;
	int h = 480;

	Mesh mesh;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));
	pcl::PolygonMesh PCLMesh;

	printf("Creating mesh from %s:\n", MeshFileName);

	//if (mesh.Load(MeshFileName, &meshBuilder, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
	if (meshBuilder.Load(MeshFileName, &mesh, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
		printf("Mesh created.\n");
	else
		printf("ERROR: Mesh can't be created!\n");
	
	// Segment mesh to surfels.

	SurfelGraph surfels;

	surfels.Init(&mesh);

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	PlanarSurfelDetector detector;

	detector.CreateParamList(&mem0);

	detector.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	detector.Init(&mesh, &surfels, &mem);

	detector.pTimer = new CRVLTimer;

	printf("Segmentation to surfels...");

	double StartTime = detector.pTimer->GetTime();

	detector.Segment(&mesh, &surfels);

	double ExecTime = detector.pTimer->GetTime() - StartTime;

	printf("completed.\n");
	printf("No. of surfels = %d\n", surfels.NodeArray.n);
	printf("Total segmentation time = %lf s\n", ExecTime);

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
	//detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
	visualizer.Run();

	// free memory

	delete detector.pTimer;
	delete[] MeshFileName;

	return 0;
#endif
}

