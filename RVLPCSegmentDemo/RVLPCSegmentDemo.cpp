// RVLPCSegmentDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "SceneSegFile.hpp"
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

void RunMainProg(
	SurfelGraph &surfels,
	PlanarSurfelDetector &detector,
	SURFEL::ObjectGraph &objects,
	CRVLMem *mem0, 
	CRVLMem *mem, 
	DWORD flags, 
	char *MeshFilePathName, 
	char *SVMClassifierParamsFileName, 
	bool bObjectAggregationLevel2, 
	bool bSegmentToObjects, 
	bool bSequence, 
	FILE *fp = NULL, 
	char *fileName = NULL);

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

void VisualizeObjectGraphVertexPointCloud(SURFEL::ObjectGraph *ograph, int sizeThr, int pointSize = 5, vtkSmartPointer<vtkRenderer> externalRenderer = NULL)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> window;
	vtkSmartPointer<vtkRenderWindowInteractor> interactor;
	if (!externalRenderer.GetPointer())
	{
		renderer = vtkSmartPointer<vtkRenderer>::New();
		window = vtkSmartPointer<vtkRenderWindow>::New();
		interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);
	}

	//VTK objects
	vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->SetDataTypeToDouble();
	vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkUnsignedCharArray> rgbs = vtkSmartPointer<vtkUnsignedCharArray>::New();
	rgbs->SetNumberOfComponents(3);

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	Surfel *pSurfel;
	QList<QLIST::Index> *pSurfelVertexList;
	SURFEL::Vertex * rvlvertex;
	QLIST::Index *qlistelement;
	int ptIdx = 0;
	double P[3];
	unsigned char rgb[3];
	srand(time(NULL));
	int objVertexNo = 0;
	for (int iObject = 0; iObject < ograph->NodeArray.n; iObject++)
	{
		pObject = ograph->NodeArray.Element + iObject;

		if (pObject->size < sizeThr)
			continue;

		piElement = pObject->elementList.pFirst;
		
		//check if object
		if (!piElement)
			continue;

		rgb[0] = rand() % 255;
		rgb[1] = rand() % 255;
		rgb[2] = rand() % 255;
		//All surfels and points
		objVertexNo = 0;
		while (piElement)
		{
			pSurfel = ograph->pSurfels->NodeArray.Element + piElement->Idx;
			//check 
			if (!((pSurfel->size <= 1) || pSurfel->bEdge))
			{
				//getting current surfel vertex list
				pSurfelVertexList = ograph->pSurfels->surfelVertexList.Element + piElement->Idx;
				//running through added surfel vertices
				qlistelement = pSurfelVertexList->pFirst;
				while (qlistelement)
				{
					rvlvertex = ograph->pSurfels->vertexArray.Element[qlistelement->Idx];
					
					P[0] = rvlvertex->P[0];
					P[1] = rvlvertex->P[1];
					P[2] = rvlvertex->P[2];
					points->InsertNextPoint(P);
					verts->InsertNextCell(1);
					verts->InsertCellPoint(ptIdx);
					rgbs->InsertNextTupleValue(rgb);
					ptIdx++;
					objVertexNo++;
					//Next
					qlistelement = qlistelement->pNext;
				}
			}

			piElement = piElement->pNext;
		}
		std::cout << "Object " << iObject << " has " << objVertexNo << " vertices!" << std::endl;
	}

	pd->SetPoints(points);
	pd->SetVerts(verts);
	pd->GetPointData()->SetScalars(rgbs);
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(pd);
	vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
	act->SetMapper(mapper);
	act->GetProperty()->SetPointSize(pointSize);
	if (externalRenderer.GetPointer())
	{
		externalRenderer->AddActor(act);
	}
	else
	{
	renderer->AddActor(act);
	//Start VTK
	renderer->ResetCamera();
	window->Render();
	interactor->Start();
}
}

void TestCHMatching(SURFEL::ObjectGraph *objects)
{

	//running through color histograms
	for (int i = 0; i < objects->additionalObjectData.colordescriptor.size(); i++)
	{
		if (objects->additionalObjectData.colordescriptor.at(i).GetColorSpace() >= 0)	//if valid
		{
			//visualize (for debug)
			objects->additionalObjectData.colordescriptor.at(i).DisplayColorHistogram();
			for (int j = i; j < objects->additionalObjectData.colordescriptor.size(); j++)
			{
				if (objects->additionalObjectData.colordescriptor.at(j).GetColorSpace() >= 0)	//if valid
				{
					//calculate metric
					std::cout << "Intersection (" << i << "," << j << "): " << objects->additionalObjectData.colordescriptor.at(i).CalculateCHMetric(objects->additionalObjectData.colordescriptor.at(j), RVLColorDescriptor::MetricsList::Intersection) << std::endl;
					std::cout << "L1 (" << i << "," << j << "): " << objects->additionalObjectData.colordescriptor.at(i).CalculateCHMetric(objects->additionalObjectData.colordescriptor.at(j), RVLColorDescriptor::MetricsList::L1) << std::endl;
					std::cout << "L2 (" << i << "," << j << "): " << objects->additionalObjectData.colordescriptor.at(i).CalculateCHMetric(objects->additionalObjectData.colordescriptor.at(j), RVLColorDescriptor::MetricsList::L2) << std::endl;
					std::cout << "Bhattacharyya (" << i << "," << j << "): " << objects->additionalObjectData.colordescriptor.at(i).CalculateCHMetric(objects->additionalObjectData.colordescriptor.at(j), RVLColorDescriptor::MetricsList::Bhattacharyya) << std::endl;
				}
			}
		}
	}
}



int main(int argc, char ** argv)
{
	//TestVTK_Plane_z_buffer(2, 640, 480, 581.45624912987f, 543.1221626989097f, 317.2825290065861f, 240.955527515504f, 60, 1.99999, 2.00001);
	//RenderCTIConvexHull();
	//testvtkdistance();
	//TestCHVisualization(RVLColorDescriptor::ColorSpaceList::HSV);
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

	SurfelGraph surfels;

	surfels.pMem = &mem;

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	PlanarSurfelDetector detector;

	detector.CreateParamList(&mem0);

	detector.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	detector.pTimer = new CRVLTimer;

	SURFEL::ObjectGraph objects;

	objects.CreateParamList(&mem0);

	objects.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	if (objects.relationClassifier == RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM)
	{
		std::cout << "Initializing SVM Classifier!" << std::endl;
		objects.InitSVMClassifier(SVMClassifierParamsFileName);
	}

	//DEL START
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

			RunMainProg(surfels, detector, objects, &mem0, &mem, flags, filePath, SVMClassifierParamsFileName, bObjectAggregationLevel2, bSegmentToObjects, bSequence, fp, fileName);

			printf("Scene %s...finished!\n\n", fileName);
		}

		fclose(fp);
		system("pause");
	}
	else
	{
		//Run single file
		RunMainProg(surfels, detector, objects, &mem0, &mem, flags, MeshFileName, SVMClassifierParamsFileName, bObjectAggregationLevel2, bSegmentToObjects, bSequence, fp, MeshFileName);
	}

	// free memory
	if (detector.pTimer)
		delete detector.pTimer;

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

void RunMainProg(
	SurfelGraph &surfels,
	PlanarSurfelDetector &detector,
	SURFEL::ObjectGraph &objects,
	CRVLMem *mem0, 
	CRVLMem *mem, 
	DWORD flags, 
	char *MeshFilePathName, 
	char *SVMClassifierParamsFileName, 
	bool bObjectAggregationLevel2, 
	bool bSegmentToObjects, 
	bool bSequence, 
	FILE *fp, 
	char *fileName) //, FILE *fp
{
	// Segmentation to surfels.

	bool bSurfelsFromSSF = false;

	char *fileExtension = RVLGETFILEEXTENSION(MeshFilePathName);

	Mesh mesh;

	if (strcmp(fileExtension, "ssf") == 0)
	{
		// Read surfels from a ssf-file.

		std::string ssfFileName(MeshFilePathName);
		ssfFileName.erase(ssfFileName.find_last_of("."));
		ssfFileName += ".ssf";

		std::cout << "Loading and creating ObjectGraph from " << ssfFileName.data() << "." << std::endl;
		objects.CreateFromSSF(ssfFileName);

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

		surfels.Init(&mesh);

		detector.Init(&mesh, &surfels, mem);		

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

				if (pSurfel->bEdge)
					continue;

				surfels.DetermineImgAdjDescriptors(pSurfel, &mesh);
			}

			objects.Create(&surfels);

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
			surfels.GenerateSSF(ssfFileName, detector.minSurfelSize, false);
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

			//Generate color histograms for surfels
			/*std::string imgFileName(MeshFileName);
			imgFileName.erase(imgFileName.find_last_of("."));
			imgFileName += ".png";
			cv::Mat img = cv::imread(imgFileName);
			cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
			int binsize[3] = { 8, 8, 0 };
			surfels.CalculateSurfelsColorHistograms(img, RVLColorDescriptor::ColorSpaceList::HSV, false, binsize, true);
			objects.CalculateObjectsColorHistogram();
			TestCHMatching(&objects);*/
			////Filko
			//objects.DetermineObjectConvexityData(0.005, 0.5);
			//ObjectAggregationLevel2(&objects, &surfels, &mesh, MeshFileName);
			cv::imshow("Colored object image", objects.CreateSegmentationImage());
			cv::waitKey(1);
			//VisualizeObjectGraphVertexPointCloud(&objects, 100);
			objects.DetermineObjectConvexityData(0.015, 0.15, false);
			//objects.ObjectAggregationLevel2_ViaObjectPairConvexity(0.015, 0.77, 0.75, 300, true);
			cv::imshow("New Colored object image", objects.CreateSegmentationImage());
			cv::waitKey(1);
			////
			//Evaluation
			int E[2];
			int N = 0;
			//std::string gtImgFileName(fileName);
			//gtImgFileName.erase(gtImgFileName.find_last_of("."));
			//gtImgFileName += "a.png";
			objects.CalculateOverAndUnderSegmentation(E, N, true, fileName, false);
			std::cout << "Oversegmenation error: " << 100.0f * (1 - E[0] / (float)N) << "%" << std::endl;
			std::cout << "Undersegmenation error: " << 100.0f * E[1] / (float)N << "%" << std::endl;
			//save label image
			/*std::string imgFileName(fileName);
			imgFileName.erase(imgFileName.find_last_of("."));
			imgFileName += "OGLabels.png";
			objects.SaveSegmentationLabelImg(imgFileName);*/


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
			objects.CalculateOverAndUnderSegmentation_SSF(E, N, false);
			std::cout << "Oversegmenation error: " << 100.0f * (1 - E[0] / (float)N) << "%" << std::endl;
			std::cout << "Undersegmenation error: " << 100.0f * E[1] / (float)N << "%" << std::endl;

			if (fp)
				fprintf(fp, "%s\t%d\t%d\t%d\n", fileName, E[0], E[1], N);

			if (!bSequence)
			{
				if (fp)
					fclose(fp);

				//Visualization
				cv::imshow("Colored surfel image", surfels.GenColoredSurfelImgFromSSF(objects.ssf));
				cv::imshow("Colored segmentation image", objects.CreateSegmentationImage());
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
			//VisualizeObjectGraphVertexPointCloud(&objects, 100, 10, visualizer.renderer);
			objects.Display();
		}
		else
#endif
			surfels.Display(&visualizer, &mesh);

		//detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
		visualizer.Run();

		}
	}
}
