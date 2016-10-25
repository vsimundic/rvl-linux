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
#include "ObjectGraph.h"
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
	DWORD &flags,
	bool &bSegmentToObjects)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("MeshFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_ID, &flags);
	pParamList->AddID(pParamData, "yes", RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY);
	pParamData = pParamList->AddParam("SegmentToObjects", RVLPARAM_TYPE_BOOL, &bSegmentToObjects);
}

//Generate a colored opencv image based on surfel data from SSF
cv::Mat GenColoredSurfelImgFromSSF(std::shared_ptr<SceneSegFile::SceneSegFile> ssf)
{
	std::shared_ptr<SceneSegFile::SegFileElement> currSSFElement;
	std::shared_ptr<SceneSegFile::FeatureTypeInt> pixAff;

	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));
	
	unsigned char labSegColor[3];
	int x = 0, y = 0;

	for (int i = 0; i < ssf->elements.size(); i++)
	{
		currSSFElement = ssf->elements.at(i);

		pixAff = std::dynamic_pointer_cast<SceneSegFile::FeatureTypeInt>(currSSFElement->features.features.at(SceneSegFile::FeaturesList::PixelAffiliation));
		
		//Generate surfel color
		labSegColor[0] = rand() % 255;
		labSegColor[1] = rand() % 255;
		labSegColor[2] = rand() % 255;

		//Set pixel colors
		for (int k = 0; k < pixAff->size; k++)
		{
			y = floor(pixAff->data[k] / 640.0);
			x = floor(pixAff->data[k] - 640.0 * y);
			coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
			coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
			coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
		}

	}
	//return image
	return coloredSegLab;
}

//Generate a colored opencv image based on surfel data from SSF
cv::Mat GenColoredSegmentationImgFromObjectGraph(SURFEL::ObjectGraph* objects)
{
	std::shared_ptr<SceneSegFile::SceneSegFile> ssf = objects->ssf;

	std::shared_ptr<SceneSegFile::SegFileElement> currSSFElement;
	std::shared_ptr<SceneSegFile::FeatureTypeInt> pixAff;

	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;

	unsigned char labSegColor[3];
	int x = 0, y = 0;

	for (int iObject = 0; iObject < objects->NodeArray.n; iObject++)
	{
		//Generate surfel color
		labSegColor[0] = rand() % 255;
		labSegColor[1] = rand() % 255;
		labSegColor[2] = rand() % 255;

		pObject = objects->NodeArray.Element + iObject;
		
		piElement = pObject->elementList.pFirst;

		while (piElement)
		{
			currSSFElement = ssf->elements.at(piElement->Idx);

			pixAff = std::dynamic_pointer_cast<SceneSegFile::FeatureTypeInt>(currSSFElement->features.features.at(SceneSegFile::FeaturesList::PixelAffiliation));

			//Set pixel colors
			for (int k = 0; k < pixAff->size; k++)
			{
				y = floor(pixAff->data[k] / 640.0);
				x = floor(pixAff->data[k] - 640.0 * y);
				coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
				coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
				coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
			}

			piElement = piElement->pNext;
		}

	}
	//return image
	return coloredSegLab;
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
	bool bSegmentToObjects = false;
	bool bSegmentToObjectsFromSSF = true;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &MeshFileName, flags, bSegmentToObjects);

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

	surfels.pMem = &mem;

	surfels.Init(&mesh);

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	PlanarSurfelDetector detector;

	detector.CreateParamList(&mem0);

	detector.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	detector.Init(&mesh, &surfels, &mem);

	detector.pTimer = new CRVLTimer;

	printf("Segmentation to surfels... ");

	double StartTime = detector.pTimer->GetTime();

	detector.Segment(&mesh, &surfels);

	double ExecTime = detector.pTimer->GetTime() - StartTime;

	printf("completed.\n");
	printf("No. of surfels = %d\n", surfels.NodeArray.n);
	printf("Total segmentation time = %lf s\n", ExecTime);

	// Group surfels into objects.

	SURFEL::ObjectGraph objects;

	if (bSegmentToObjects)
	{
		printf("Grouping surfels into objects... ");

		surfels.ImageAdjacency(&mesh);

		Surfel *pSurfel = surfels.NodeArray.Element;

		for (int i = 0; i < surfels.NodeArray.n; pSurfel++, i++)
		{
			if (pSurfel->size <= 1)
				continue;

			DetermineImgAdjDescriptors(pSurfel, &mesh);
		}

		objects.Create(&surfels);

		objects.ComputeRelationCosts();

		objects.WERSegmentation();

		printf("completed.\n");
	}

	if (bSegmentToObjects && bSegmentToObjectsFromSSF)
	{
		SURFEL::ObjectGraph objects2;

		std::string ssfFileName(MeshFileName);
		ssfFileName.erase(ssfFileName.find_last_of("."));
		ssfFileName += ".ssf";
		
		std::cout << "Loading and creating ObjectGraph from SSF!" << std::endl;
		objects2.CreateFromSSF(ssfFileName);

		std::cout << "Compute relation cost!" << std::endl;
		objects2.ComputeRelationCosts();

		std::cout << "WER segmentation!" << std::endl;
		objects2.WERSegmentation();

		printf("completed.\n");

		//Visualization
		cv::imshow("Colored surfel image", GenColoredSurfelImgFromSSF(objects2.ssf));
		cv::imshow("Colored segmentation image", GenColoredSegmentationImgFromObjectGraph(&objects2));
		cv::waitKey(1);

		//Evaluation
		int E[2];
		int N = 0;
		objects2.CalculateOverAndUnderSegmentation(E, N, false);
		std::cout << "Oversegmenation error: " << 1 - E[0] / (float)N << std::endl;
		std::cout << "Undersegmenation error: " << E[1] / (float)N << std::endl;
	}

	// Display segmentation.

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	surfels.NodeColors(SelectionColor);	

	Visualizer visualizer;	

	visualizer.Create();
	surfels.InitDisplay(&visualizer, &mesh, &detector);

	if (bSegmentToObjects)
	{
		objects.InitDisplay(&visualizer, &mesh, SelectionColor);
		objects.Display();
	}
	else
		surfels.Display(&visualizer, &mesh);

	//detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
	visualizer.Run();

	// free memory

	delete detector.pTimer;
	delete[] MeshFileName;

	return 0;
#endif
}

