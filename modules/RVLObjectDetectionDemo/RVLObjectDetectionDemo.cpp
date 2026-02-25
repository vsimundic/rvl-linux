// RVLObjectDetectionDemo.cpp : Defines the entry point for the console application.
//
#include <Windows.h>
//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "ReconstructionEval.h"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "NeighborhoodTool.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"
#include "ObjectDetector.h"
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"
#include "GTTools.h"
#include "vtkOBBTree.h"
#include "vtkLine.h"


//#include <ctime>

//#define RVLOBJECTDETECTIONDEMO_SELECT_GT_OBJECTS

using namespace RVL;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	char **pSequenceFileName,
	Camera &camera,
	char **pSegmentationResultsFileName,
	bool &bVisualization,
		bool &b3DVisualization,
	bool &b2DVisualization,
	char **pSelectedGTObjectsFileName,
	char **pReconstructionGTFileName,
	char **pResultsFolder)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("MeshFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("SequenceFileName", RVLPARAM_TYPE_STRING, pSequenceFileName);
	pParamData = pParamList->AddParam("SegmentationResultsFileName", RVLPARAM_TYPE_STRING, pSegmentationResultsFileName);
	pParamData = pParamList->AddParam("Visualization", RVLPARAM_TYPE_BOOL, &bVisualization);
	pParamData = pParamList->AddParam("Visualization.3D", RVLPARAM_TYPE_BOOL, &b3DVisualization);
	pParamData = pParamList->AddParam("Visualization.2D", RVLPARAM_TYPE_BOOL, &b2DVisualization);
	pParamData = pParamList->AddParam("SelectedGTObjectsFileName", RVLPARAM_TYPE_STRING, pSelectedGTObjectsFileName);
	pParamData = pParamList->AddParam("ReconstructionGTFileName", RVLPARAM_TYPE_STRING, pReconstructionGTFileName);
	pParamData = pParamList->AddParam("ResultsFolder", RVLPARAM_TYPE_STRING, pResultsFolder);
	pParamData = pParamList->AddParam("Camera.fu", RVLPARAM_TYPE_FLOAT, &(camera.fu));
	pParamData = pParamList->AddParam("Camera.fv", RVLPARAM_TYPE_FLOAT, &(camera.fv));
	pParamData = pParamList->AddParam("Camera.uc", RVLPARAM_TYPE_FLOAT, &(camera.uc));
	pParamData = pParamList->AddParam("Camera.vc", RVLPARAM_TYPE_FLOAT, &(camera.vc));
	pParamData = pParamList->AddParam("Camera.w", RVLPARAM_TYPE_INT, &(camera.w));
	pParamData = pParamList->AddParam("Camera.h", RVLPARAM_TYPE_INT, &(camera.h));
}

//void VisualizeSurfelNormals(Visualizer *vis, SurfelGraph* pSurfels)
//{
//	Surfel *pSurfel;
//	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//	points->SetDataTypeToDouble();
//	vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
//	normals->SetNumberOfComponents(3);
//	normals->SetName("Normals");
//	for (int i = 0; i < pSurfels->NodeArray.n; i++)
//	{
//		pSurfel = pSurfels->NodeArray.Element + i;
//		if ((pSurfel->size < 2) || pSurfel->bEdge)
//			continue;
//
//		points->InsertNextPoint(pSurfel->P);
//		normals->InsertNextTuple(pSurfel->N);
//	}
//	vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
//	pd->SetPoints(points);
//	pd->GetPointData()->AddArray(normals);
//	pd->GetPointData()->SetActiveNormals("Normals");
//
//	vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();
//
//	vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
//	glyph3D->SetSourceConnection(arrowSource->GetOutputPort());
//	glyph3D->SetVectorModeToUseNormal();
//	glyph3D->SetInputData(pd);
//	glyph3D->SetScaleFactor(0.03);
//	glyph3D->Update();
//
//	vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();
//	mapper->SetInputConnection(glyph3D->GetOutputPort());
//	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//	actor->SetMapper(mapper);
//	vis->renderer->AddActor(actor);
//}

void MeshSmoothTest(vtkSmartPointer<vtkPolyData> pd)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();;
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
	//smoothFilter->SetInputData(pd);
	//smoothFilter->SetNumberOfIterations(30);
	//smoothFilter->SetRelaxationFactor(1.0);
	////smoothFilter->FeatureEdgeSmoothingOn();
	////smoothFilter->SetFeatureAngle(45);
	////smoothFilter->SetEdgeAngle(60);
	//smoothFilter->BoundarySmoothingOn();
	//smoothFilter->Update();

	vtkSmartPointer<vtkWindowedSincPolyDataFilter> smoother = vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
	smoother->SetInputData(pd);
	smoother->SetNumberOfIterations(30);
	smoother->BoundarySmoothingOn();
	//smoother->FeatureEdgeSmoothingOn();
	//smoother->SetFeatureAngle(60.0);
	/*smoother->SetPassBand(0.1);
	smoother->NonManifoldSmoothingOn();
	smoother->NormalizeCoordinatesOn();*/
	smoother->Update();


	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(smoother->GetOutputPort());
	vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
	act->SetMapper(mapper);
	renderer->AddActor(act);

	//Start VTK
	renderer->ResetCamera();
	window->Render();
	interactor->Start();
}

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	CRVLMem mem;	// cycle memory

	mem.Create(1000000000);

	// Read parameters from a configuration file.

	char cfgSelectionFileName[] = "RVLObjectDetectionDemo.cfg";

	char *cfgFileName = ReadConfigurationFile(cfgSelectionFileName);

	if (cfgFileName == NULL)
		return 1;

	char *MeshFileName = NULL;
	char *SequenceFileName = NULL;
	Camera camera;
	char *SegmentationResultsFileName = NULL;
	char *selectedGTObjectsFileName = NULL;
	char *reconstructionGTFileName = NULL;
	char *ResultsFolder = NULL;
	bool bVisualization, b3DVisualization, b2DVisualization;

	CRVLParameterList ParamList;

	CreateParamList(
		&ParamList, 
		&mem0, 
		&MeshFileName, 
		&SequenceFileName, 
		camera,
		&SegmentationResultsFileName, 
		bVisualization,
		b3DVisualization, 
		b2DVisualization,
		&selectedGTObjectsFileName, 
		&reconstructionGTFileName,
		&ResultsFolder);

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

	objectDetector.camera = camera;
	objectDetector.vpMeshBuilder = &meshBuilder;
	objectDetector.LoadMesh = LoadMesh;
	objectDetector.CreateMesh = CreateMesh;

	objectDetector.pSurfelDetector->pTimer = new CRVLTimer;

	objectDetector.resultsFolder = ResultsFolder;

	// Initialize visualization.

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;	

	Visualizer visualizer;

	visualizer.b2D = b2DVisualization;
	visualizer.b3D = b3DVisualization;

	visualizer.Create();

	// Object detection.

	bool bSequence = (SequenceFileName != NULL) ? true : false;

	FILE *fp = (SegmentationResultsFileName ? fopen(SegmentationResultsFileName, "w") : NULL);

	if (fp)
		fprintf(fp, "Image\tE0\tE1\tN\n");

	cv::Mat GTLabImg;

	// 3D reconstruction.

	ReconstructionEval reconstructionEval;

	reconstructionEval.LoadGT(reconstructionGTFileName);

	Array3D<uchar> grid;
	FILE *fpReconstructionEval;

	if (reconstructionEval.bGT)
	{
		grid = reconstructionEval.grid;
		grid.Element = new uchar[grid.a * grid.b * grid.c];

		fpReconstructionEval = fopen((std::string(ResultsFolder) + "\\reconstructionEval.txt").data(), "w");
	}

	//Time measurement
	LARGE_INTEGER ctr1, ctr2, freq;
	float odTime;
	std::fstream fs("timings.txt", std::fstream::out);
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

			objectDetector.meshFileName = filePath;
			if (QueryPerformanceCounter((LARGE_INTEGER *)&ctr1) != 0)
			{
				objectDetector.DetectObjects(filePath);

				QueryPerformanceCounter((LARGE_INTEGER *)&ctr2);
				QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
			}
			odTime = (ctr2.QuadPart - ctr1.QuadPart) * 1000.0 / freq.QuadPart;
			std::cout << "DetectObject - time elapsed: " << odTime << "ms" << std::endl;
			fs << odTime << std::endl;

			printf("Scene %s...finished!\n\n", fileName);

			if (objectDetector.bTrainingHMI)
				objectDetector.TrainingHMI(fileName);

			objectDetector.Evaluate(fp, filePath, selectedGTObjectsFileName);

#ifdef RVLOBJECTDETECTIONDEMO_SELECT_GT_OBJECTS
			PCGT::DisplayGroundTruthSegmentation(filePath, GTLabImg);
			cv::moveWindow("GT Segmentation", 0, 0);
#endif

			//cv::waitKey();

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				std::string segmentationImageFileName(filePath);
				segmentationImageFileName.erase(segmentationImageFileName.find_last_of("."));
				segmentationImageFileName += "OGLabels.png";
				objectDetector.pObjects->SaveSegmentationLabelImg(segmentationImageFileName);

				//cv::imshow("Segmentation", objectDetector.pObjects->CreateSegmentationImage());
				//cv::waitKey();
			}
#endif

			// Display segmentation.

			objectDetector.pSurfels->NodeColors(SelectionColor);

			if (objectDetector.bSegmentToObjects)
			{
				objectDetector.DisplayObjectMap();
				//Save object map for external evaluation
				std::string objMapImgFileName(filePath);
				objMapImgFileName.erase(objMapImgFileName.find_last_of("."));
				objMapImgFileName += "VNLabels.png";
				objectDetector.SaveObjectMapLabelImg(objMapImgFileName);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
				if (objectDetector.objectAggregationLevel1Method == RVLOBJECTDETECTION_METHOD_FUZZY_WER)
				{
					objectDetector.pObjects->InitDisplay(&visualizer, &(objectDetector.mesh), SelectionColor);
					objectDetector.pObjects->Display();					
				}
				else 
#endif
				if (bVisualization)
				{
#ifdef RVLOBJECTDETECTION_DISPLAY_ECMR19
					visualizer.SetMesh(&(objectDetector.mesh));
#else
					if (objectDetector.objectAggregationLevel1Method == RVLOBJECTDETECTION_METHOD_CONVEX_AND_CONCAVE)
						objectDetector.DisplayClustersInteractive(&visualizer, &(objectDetector.mesh), SelectionColor);
#endif

					objectDetector.Display(&visualizer);
				}

				// 3D reconstruction evaluation: GT visualization.

				if (reconstructionEval.bGT)
				{
					int iScene;

					sscanf(fileName, "%d.ply", &iScene);

					reconstructionEval.CreateGTGrid(iScene);

					//char *reconstructionGTGridFileName = RVLCreateFileName(filePath, ".ply", -1, ".gt");

					////FILE *fpReconstructionGTGrid = fopen(reconstructionGTGridFileName, "wb");
					//FILE *fpReconstructionGTGrid = fopen(reconstructionGTGridFileName, "rb");

					//delete[] reconstructionGTGridFileName;

					////fwrite(reconstructionEval.grid.Element, sizeof(uchar), reconstructionEval.grid.a * reconstructionEval.grid.b * reconstructionEval.grid.c, fpReconstructionGTGrid);
					//fread(reconstructionEval.grid.Element, sizeof(uchar), reconstructionEval.grid.a * reconstructionEval.grid.b * reconstructionEval.grid.c, fpReconstructionGTGrid);

					//fclose(fpReconstructionGTGrid);

					//for (int i = 0; i < grid.a * grid.b * grid.c; i++)
					//	if (grid.Element[i] != reconstructionEval.grid.Element[i])
					//		int debug = 0;

					int TP, FP, FN;

					objectDetector.Evaluate3DReconstruction(&reconstructionEval, TP, FP, FN, &grid);

					fprintf(fpReconstructionEval, "%d\t%d\t%d\n", TP, FP, FN);

					printf("3D reconstruction evaluation: TP: %d FP: %d FN: %d PR: %f RC: %f IoU: %f\n", TP, FP, FN,
						(float)TP / (float)(TP + FP), (float)TP / (float)(TP + FN), (float)TP / (float)(TP + FP + FN));

					//reconstructionEval.DisplayGT(&visualizer, iScene);

					//uchar color[3] = {0, 0, 255};

					//reconstructionEval.DisplayGrid(&visualizer, grid, color);

					reconstructionEval.DisplayGridComparison(&visualizer, grid, reconstructionEval.grid);
				}
				//objectDetector.pSurfels->InitDisplay(&visualizer, &(objectDetector.mesh), objectDetector.pSurfelDetector);
				
				//objectDetector.pSurfels->Display(&visualizer, &(objectDetector.mesh));//OVO
			}
			else
			{
				objectDetector.pSurfels->InitDisplay(&visualizer, &(objectDetector.mesh), objectDetector.pSurfelDetector);
				objectDetector.pSurfels->Display(&visualizer, &(objectDetector.mesh));
			}

			//objectDetector.pSurfels->DisplayEdgeFeatures();

			//objectDetector.pSurfels->DisplayVertices();

			if (bVisualization)
			{
				visualizer.Run();//OVO

				visualizer.renderer->RemoveAllViewProps();//OVO
			}
		}
		fs.flush();
		fs.close();
		printf("Sequence completed.\n");
		system("pause");
	}	// if (bSequence)
	else
	{

		objectDetector.DetectObjects(MeshFileName);

		if (objectDetector.bTrainingHMI)
			objectDetector.TrainingHMI(MeshFileName);

		objectDetector.Evaluate(fp, MeshFileName, selectedGTObjectsFileName);

#ifdef RVLOBJECTDETECTIONDEMO_SELECT_GT_OBJECTS
		PCGT::DisplayGroundTruthSegmentation(MeshFileName, GTLabImg);		
#endif

		cv::waitKey(1);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
		if (objectDetector.bSurfelsFromSSF)
		{
			if (objectDetector.bSegmentToObjects)
			{
				//Visualization
				cv::imshow("Colored surfel image", objectDetector.pSurfels->GenColoredSurfelImgFromSSF(objectDetector.pObjects->ssf));
				cv::imshow("Colored segmentation image", objectDetector.pObjects->CreateSegmentationImageFromSSF());
				cv::waitKey(1);
			}
		}
		else
#endif
		{
			// Display segmentation.

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				objectDetector.pObjects->InitDisplay(&visualizer, &(objectDetector.mesh), SelectionColor);
				objectDetector.pObjects->Display();
				objectDetector.Display(&visualizer);
			}
			else
#endif
			{
				objectDetector.pSurfels->InitDisplay(&visualizer, &(objectDetector.mesh), objectDetector.pSurfelDetector);
				objectDetector.pSurfels->Display(&visualizer, &(objectDetector.mesh));
			}

			objectDetector.pSurfels->DisplayEdgeFeatures();

			objectDetector.pSurfels->DisplayVertices();

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				std::string segmentationImageFileName(MeshFileName);
				segmentationImageFileName.erase(segmentationImageFileName.find_last_of("."));
				segmentationImageFileName += "OGLabels.png";
				objectDetector.pObjects->SaveSegmentationLabelImg(segmentationImageFileName);
			}
#endif
			// DEMO: common bounding box of objects 9 and 19.

			//RECOG::PSGM_::ModelInstance boundingBox;

			//boundingBox.modelInstance.Element = new RECOG::PSGM_::ModelInstanceElement[66];

			//objectDetector.BoundingBox(9, 19, &boundingBox);

			//objectDetector.pPSGM->convexTemplate = objectDetector.pPSGM->convexTemplateBox;

			//objectDetector.pPSGM->DisplayCTI(&visualizer, &boundingBox);

			//objectDetector.pPSGM->convexTemplate = objectDetector.pPSGM->convexTemplate66;

			//delete[] boundingBox.modelInstance.Element;

			// END DEMO

			// Display bounding boxes.

			//if (objectDetector.bObjectAggregationLevel2)
			//{
			//	Array<int> boundingBoxArray;

			//	boundingBoxArray.Element = new int[objectDetector.boundingBoxes.SegmentCTIs.n];

			//	boundingBoxArray.n = 0;

			//	int iObject;

			//	for (iObject = 0; iObject < objectDetector.boundingBoxes.SegmentCTIs.n; iObject++)
			//		if (objectDetector.boundingBoxes.SegmentCTIs.Element[iObject].n > 0)
			//			boundingBoxArray.Element[boundingBoxArray.n++] = objectDetector.boundingBoxes.SegmentCTIs.Element[iObject].Element[0];

			//	objectDetector.pPSGM->convexTemplate = objectDetector.pPSGM->convexTemplateBox;

			//	objectDetector.pPSGM->DisplayCTIs(&visualizer, &(objectDetector.boundingBoxes), &boundingBoxArray);

			//	objectDetector.pPSGM->convexTemplate = objectDetector.pPSGM->convexTemplate66;

			//	delete[] boundingBoxArray.Element;
			//}

			//detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
			visualizer.Run();
		}	// if (objectDetector.bSurfelsFromSSF)

	}	// if (!bSequence)
		
	// Memory deallocation.

	if (fp)
		fclose(fp);

	if (reconstructionEval.bGT)
		fclose(fpReconstructionEval);

	RVL_DELETE_ARRAY(MeshFileName);
	RVL_DELETE_ARRAY(SequenceFileName);
	RVL_DELETE_ARRAY(SegmentationResultsFileName);
	RVL_DELETE_ARRAY(selectedGTObjectsFileName);
	RVL_DELETE_ARRAY(reconstructionGTFileName);
	delete[] grid.Element;

	return 0;
}
