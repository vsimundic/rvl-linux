// RVLPCSegmentDemo.cpp : Defines the entry point for the console application.
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
#define RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF			0x00000002
#define RVLPCSEGMENT_DEMO_FLAG_SEGMENTATION_GT	0x00000004

using namespace RVL;

#include "RVLPCSegmentCreateTrainingData.h"

void RunMainProg(CRVLMem *mem0, CRVLMem *mem, DWORD flags, char *MeshFilePathName, char *SVMClassifierParamsFileName, bool bObjectAggregationLevel2, bool bSegmentToObjects, bool bSequence, FILE *fp, char *fileName = NULL);

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	DWORD &flags,
	bool &bSegmentToObjects,
	bool &bObjectAggregationLevel2,
	char **pSVMClassifierParamsFileName,
	char **pSequenceFileName,
	char **pSegmentationResultsFileName)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("MeshFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY);
	pParamData = pParamList->AddParam("Save SSF", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF);
	pParamData = pParamList->AddParam("Segmentation GT", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLPCSEGMENT_DEMO_FLAG_SEGMENTATION_GT);
	pParamData = pParamList->AddParam("SegmentToObjects", RVLPARAM_TYPE_BOOL, &bSegmentToObjects);
	pParamData = pParamList->AddParam("ObjectAggregationLevel2", RVLPARAM_TYPE_BOOL, &bObjectAggregationLevel2);
	pParamData = pParamList->AddParam("SVMClassifierParamsFileName", RVLPARAM_TYPE_STRING, pSVMClassifierParamsFileName);
	pParamData = pParamList->AddParam("SequenceFileName", RVLPARAM_TYPE_STRING, pSequenceFileName);
	pParamData = pParamList->AddParam("SegmentationResultsFileName", RVLPARAM_TYPE_STRING, pSegmentationResultsFileName);
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

	//mem.Create(100000000);
	mem.Create(1000000000);

	// Read parameters from a configuration file.

	char *MeshFileName = NULL;
	char *SVMClassifierParamsFileName = NULL;
	char *SequenceFileName = NULL;
	char *SegmentationResultsFileName = NULL;

	DWORD flags = 0x00000000;
	bool bSegmentToObjects = false;
	bool bObjectAggregationLevel2 = false;

	CRVLParameterList ParamList;

	//CreateParamList(&ParamList, &mem0, &MeshFileName, flags, bSegmentToObjects, bObjectAggregationLevel2);
	CreateParamList(&ParamList, &mem0, &MeshFileName, flags, bSegmentToObjects, bObjectAggregationLevel2, &SVMClassifierParamsFileName, &SequenceFileName, &SegmentationResultsFileName);

	ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	if (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF)
		flags |= RVLPCSEGMENT_DEMO_FLAG_SEGMENTATION_GT;

	//DEL START
	bool bSequence = (SequenceFileName != NULL) ? true : false;

	FILE *fp = fopen(SegmentationResultsFileName, "w");
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

			RunMainProg(&mem0, &mem, flags, filePath, SVMClassifierParamsFileName, bObjectAggregationLevel2, bSegmentToObjects, bSequence, fp, fileName);

			printf("Scene %s...finished!\n\n", fileName);
		}

		fclose(fp);
		system("pause");
	}
	else
	{
		//Run single file
		RunMainProg(&mem0, &mem, flags, MeshFileName, SVMClassifierParamsFileName, bObjectAggregationLevel2, bSegmentToObjects, bSequence, fp, MeshFileName);
	}



	//DEL END
	if (MeshFileName)
		delete[] MeshFileName;

	if (SVMClassifierParamsFileName)
		delete[] SVMClassifierParamsFileName;

	if (SequenceFileName)
		delete[] SequenceFileName;

	if (SegmentationResultsFileName)
		delete[] SegmentationResultsFileName;

	return 0;
#endif
}

void RunMainProg(CRVLMem *mem0, CRVLMem *mem, DWORD flags, char *MeshFilePathName, char *SVMClassifierParamsFileName, bool bObjectAggregationLevel2, bool bSegmentToObjects, bool bSequence, FILE *fp, char *fileName) //, FILE *fp
{
	// Segmentation to surfels.

	bool bSurfelsFromSSF = false;

	SurfelGraph surfels;
	SURFEL::ObjectGraph objects;
	PlanarSurfelDetector detector;
	Mesh mesh;

	objects.CreateParamList(mem0);

	objects.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	char *fileExtension = RVLGETFILEEXTENSION(MeshFilePathName);

	if (strcmp(fileExtension, "ssf") == 0)
	{
		// Read surfels from a ssf-file.

		std::string ssfFileName(MeshFilePathName);
		ssfFileName.erase(ssfFileName.find_last_of("."));
		ssfFileName += ".ssf";

		std::cout << "Loading and creating ObjectGraph from " << ssfFileName.data() << "." << std::endl;
		objects.CreateFromSSF(ssfFileName);

		std::cout << "Initializing SVM Classifier!" << std::endl;
		objects.InitSVMClassifier(SVMClassifierParamsFileName);

		std::cout << "Compute relation cost!" << std::endl;
		objects.ComputeRelationCosts();

		bSurfelsFromSSF = true;
	}
	else
	{
		// Read mesh from file.

		PCLMeshBuilder meshBuilder;

		meshBuilder.CreateParamList(mem0);

		meshBuilder.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

		int w = 640;
		int h = 480;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));
		pcl::PolygonMesh PCLMesh;

		printf("Creating mesh from %s:\n", MeshFilePathName);

		//if (mesh.Load(MeshFileName, &meshBuilder, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
		if (meshBuilder.Load(MeshFilePathName, &mesh, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
			printf("Mesh created.\n");
		else
			printf("ERROR: Mesh can't be created!\n");

		// Segment mesh to surfels.		

		surfels.pMem = mem;

		surfels.Init(&mesh);

		surfels.CreateParamList(mem0);

		surfels.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

		detector.CreateParamList(mem0);

		detector.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

		detector.Init(&mesh, &surfels, mem);

		detector.pTimer = new CRVLTimer;

		printf("Segmentation to surfels... ");

		double StartTime = detector.pTimer->GetTime();

		detector.Segment(&mesh, &surfels);

		double ExecTime = detector.pTimer->GetTime() - StartTime;

		printf("completed.\n");
		printf("No. of surfels = %d\n", surfels.NodeArray.n);
		printf("Total segmentation time = %lf s\n", ExecTime);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
		if (flags & RVLPCSEGMENT_DEMO_FLAG_SEGMENTATION_GT)
			surfels.AssignGroundTruthSegmentation(MeshFilePathName, detector.minSurfelSize);

		// Group surfels into objects.

		if (bSegmentToObjects || (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF))
		{
			printf("Computing relations between adjacent surfels...");

			surfels.ImageAdjacency(&mesh);

			Surfel *pSurfel = surfels.NodeArray.Element;

			for (int i = 0; i < surfels.NodeArray.n; pSurfel++, i++)
			{
				if (pSurfel->size <= 1)
					continue;

				DetermineImgAdjDescriptors(pSurfel, &mesh);
			}

			objects.Create(&surfels);

			std::cout << "Initializing SVM Classifier!" << std::endl;
			objects.InitSVMClassifier(SVMClassifierParamsFileName);

			objects.ComputeRelationCosts();

			printf("completed.\n");

			objects.Debug();
		}

		if (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF)
		{
			std::string ssfFileName(MeshFilePathName);
			ssfFileName.erase(ssfFileName.find_last_of("."));
			ssfFileName += ".ssf";

			std::cout << "Saving SSF!" << std::endl;
			GenerateSSF(&surfels, ssfFileName, detector.minSurfelSize, false);
			std::cout << "Saved!" << std::endl;
		}
#endif
	}	// If fileExtension != "ssf"

#ifdef RVLSURFEL_IMAGE_ADJACENCY
	if (bSegmentToObjects)
	{	
		printf("Aggregating surfels into objects... ");

		objects.WERSegmentation();

		printf("completed.\n");

		if (!bSurfelsFromSSF && bObjectAggregationLevel2)
		{
			printf("Aggregating objects (LEVEL 2)... ");

			surfels.DetectVertices(&mesh);

			printf("completed.\n");
		}
	}

	if (bSurfelsFromSSF)
	{
		if (bSegmentToObjects)
		{

			//Evaluation
			int E[2];
			int N = 0;
			objects.CalculateOverAndUnderSegmentation(E, N, false);
			std::cout << "Oversegmenation error: " << 100.0f * (1 - E[0] / (float)N) << "%" << std::endl;
			std::cout << "Undersegmenation error: " << 100.0f * E[1] / (float)N << "%" << std::endl;

			fprintf(fp, "%s\t%d\t%d\t%d\n", fileName, E[0], E[1], N);

			if (!bSequence)
			{
				fclose(fp);

				//Visualization
				cv::imshow("Colored surfel image", GenColoredSurfelImgFromSSF(objects.ssf));
				cv::imshow("Colored segmentation image", GenColoredSegmentationImgFromObjectGraph(&objects));
				cv::waitKey();
			}

		}
	}
	else
#endif
	{
		if (!bSequence)
		{
			// Display segmentation.

			unsigned char SelectionColor[3];

			SelectionColor[0] = 0;
			SelectionColor[1] = 255;
			SelectionColor[2] = 0;

			surfels.NodeColors(SelectionColor);

			Visualizer visualizer;

			visualizer.Create();
			surfels.InitDisplay(&visualizer, &mesh, &detector);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (bSegmentToObjects)
			{
				objects.InitDisplay(&visualizer, &mesh, SelectionColor);
				objects.Display();
			}
			else
#endif
				surfels.Display(&visualizer, &mesh);

			//detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
			visualizer.Run();

		}
	}

	// free memory
	if (detector.pTimer)
		delete detector.pTimer;



}
