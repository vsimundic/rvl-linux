// RVLObjectDetectionDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
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
#include "ObjectDetector.h"
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"

using namespace RVL;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	char **pSequenceFileName,
	char **pSegmentationResultsFileName,
	bool &b3DVisualization,
	bool &b2DVisualization)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("MeshFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("SequenceFileName", RVLPARAM_TYPE_STRING, pSequenceFileName);
	pParamData = pParamList->AddParam("SegmentationResultsFileName", RVLPARAM_TYPE_STRING, pSegmentationResultsFileName);
	pParamData = pParamList->AddParam("Visualization.3D", RVLPARAM_TYPE_BOOL, &b3DVisualization);
	pParamData = pParamList->AddParam("Visualization.2D", RVLPARAM_TYPE_BOOL, &b2DVisualization);
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	CRVLMem mem;	// cycle memory

	mem.Create(1000000000);

	// Read parameters from a configuration file.

	char cfgFileName[] = "RVLObjectDetectionDemo.cfg";

	char *MeshFileName = NULL;
	char *SequenceFileName = NULL;
	char *SegmentationResultsFileName = NULL;
	bool b3DVisualization, b2DVisualization;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &MeshFileName, &SequenceFileName, &SegmentationResultsFileName, b3DVisualization, b2DVisualization);

	ParamList.LoadParams(cfgFileName);

	// Create mesh builder.

	PCLMeshBuilder meshBuilder;

	meshBuilder.CreateParamList(&mem0);

	meshBuilder.ParamList.LoadParams(cfgFileName);

	int w = 640;
	int h = 480;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));

	meshBuilder.PC = PC;

	// Initialize object detector.

	ObjectDetector objectDetector;

	objectDetector.pMem0 = &mem0;
	objectDetector.pMem = &mem;
	objectDetector.cfgFileName = RVLCreateString(cfgFileName);

	objectDetector.Init();

	objectDetector.vpMeshBuilder = &meshBuilder;
	objectDetector.LoadMesh = LoadMesh;

	objectDetector.pSurfelDetector->pTimer = new CRVLTimer;

	// Object detection.

	bool bSequence = (SequenceFileName != NULL) ? true : false;

	FILE *fp = (SegmentationResultsFileName ? fopen(SegmentationResultsFileName, "w") : NULL);

	if (fp)
		fprintf(fp, "Image\tE0\tE1\tN\n");

	if (bSequence)
	{
		//Run sequence
		FileSequenceLoader sceneSequence;
		sceneSequence.Init(SequenceFileName);

		char filePath[200];
		char fileName[200];

		while (sceneSequence.GetNext(filePath, fileName))
		{
			mem.Clear();

			printf("Scene %s...\n", fileName);

			objectDetector.DetectObjects(filePath);

			printf("Scene %s...finished!\n\n", fileName);

			objectDetector.Evaluate(fp, filePath);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				std::string segmentationImageFileName(filePath);
				segmentationImageFileName.erase(segmentationImageFileName.find_last_of("."));
				segmentationImageFileName += "OGLabels.png";
				objectDetector.pObjects->SaveSegmentationLabelImg(segmentationImageFileName);
			}
#endif
		}
		system("pause");
	}
	else
	{
		objectDetector.DetectObjects(MeshFileName);

		objectDetector.Evaluate(fp, MeshFileName);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
		if (objectDetector.bSurfelsFromSSF)
		{
			if (objectDetector.bSegmentToObjects)
			{
				//Visualization
				cv::imshow("Colored surfel image", objectDetector.pSurfels->GenColoredSurfelImgFromSSF(objectDetector.pObjects->ssf));
				cv::imshow("Colored segmentation image", objectDetector.pObjects->CreateSegmentationImageFromSSF());
				cv::waitKey();
			}
		}
		else
#endif
		{
			// Display segmentation.

			unsigned char SelectionColor[3];

			SelectionColor[0] = 0;
			SelectionColor[1] = 255;
			SelectionColor[2] = 0;

			objectDetector.pSurfels->NodeColors(SelectionColor);

			Visualizer visualizer;

			visualizer.b2D = b2DVisualization;
			visualizer.b3D = b3DVisualization;

			visualizer.Create();
			objectDetector.pSurfels->InitDisplay(&visualizer, &(objectDetector.mesh), objectDetector.pSurfelDetector);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				objectDetector.pObjects->InitDisplay(&visualizer, &(objectDetector.mesh), SelectionColor);
				objectDetector.pObjects->Display();
			}
			else
#endif
				objectDetector.pSurfels->Display(&visualizer, &(objectDetector.mesh));

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				std::string segmentationImageFileName(MeshFileName);
				segmentationImageFileName.erase(segmentationImageFileName.find_last_of("."));
				segmentationImageFileName += "OGLabels.png";
				objectDetector.pObjects->SaveSegmentationLabelImg(segmentationImageFileName);
			}
#endif

			//detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
			visualizer.Run();
		}

	}
		
	// Memory deallocation.

	if (fp)
		fclose(fp);

	RVL_DELETE_ARRAY(MeshFileName);
	RVL_DELETE_ARRAY(SequenceFileName);
	RVL_DELETE_ARRAY(SegmentationResultsFileName);

	return 0;
}

