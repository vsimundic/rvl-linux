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
#include "vtkDistancePolyDataFilter.h"
#include "vtkImplicitPolyDataDistance.h"
#include "vtkDelaunay3D.h"
#include "vtkCleanPolyData.h"
#include "vtkGeometryFilter.h"
#include "vtkAppendPolyData.h"
#include "vtkAlgorithm.h"

//#define RVLPCSEGMENT_DEMO_CREATE_TRAINING_DATA

#define RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY			0x00000001
#define RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF			0x00000002
#define RVLPCSEGMENT_DEMO_FLAG_SEGMENTATION_GT	0x00000004

using namespace RVL;

#include "RVLPCSegmentCreateTrainingData.h"

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	DWORD &flags,
	bool &bSegmentToObjects,
	bool &bObjectAggregationLevel2)
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
}

//test vtk PolyData distance
void testvtkdistance()
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//points
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->SetDataTypeToDouble();
	points->Reset();
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(0.0, 10.0, 0.0);
	points->InsertNextPoint(10.0, 0.0, 0.0);
	points->InsertNextPoint(0.0, 5.0, 10.0);
	
	//For vertices display
	vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
	verts->InsertNextCell(1);
	verts->InsertCellPoint(0);
	verts->InsertNextCell(1);
	verts->InsertCellPoint(1);
	verts->InsertNextCell(1);
	verts->InsertCellPoint(2);
	verts->InsertNextCell(1);
	verts->InsertCellPoint(3);

	//Creating triangles
	vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
	vtkIdType triangle[3] = {0, 1, 2};
	triangles->InsertNextCell(3, triangle);
	triangle[0] = 0;
	triangle[1] = 1;
	triangle[2] = 3;
	triangles->InsertNextCell(3, triangle);

	//Polydata
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);
	polyData->SetVerts(verts);
	polyData->SetPolys(triangles);

	//points2
	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
	points2->SetDataTypeToDouble();
	points2->Reset();
	points2->InsertNextPoint(2.5, 2.5, 1.0);
	points2->InsertNextPoint(5.0, 5.0, 2.0);
	points2->InsertNextPoint(1.25, 2.5, 3.0);
	
	//For vertices display
	vtkSmartPointer<vtkCellArray> verts2 = vtkSmartPointer<vtkCellArray>::New();
	verts2->InsertNextCell(1);
	verts2->InsertCellPoint(0);
	verts2->InsertNextCell(1);
	verts2->InsertCellPoint(1);
	verts2->InsertNextCell(1);
	verts2->InsertCellPoint(2);

	//Creating triangles2
	vtkSmartPointer<vtkCellArray> triangles2 = vtkSmartPointer<vtkCellArray>::New();
	triangle[0] = 0;
	triangle[1] = 1;
	triangle[2] = 2;
	triangles2->InsertNextCell(3, triangle);

	//Polydata2
	vtkSmartPointer<vtkPolyData> polyData2 = vtkSmartPointer<vtkPolyData>::New();
	polyData2->SetPoints(points2);
	polyData2->SetVerts(verts2);
	polyData2->SetPolys(triangles2);

	//distance
	vtkSmartPointer<vtkDistancePolyDataFilter> dist = vtkSmartPointer<vtkDistancePolyDataFilter>::New();
	dist->SetInputDataObject(0, polyData2);
	dist->SetInputDataObject(1, polyData);
	dist->ComputeSecondDistanceOn();
	dist->SignedDistanceOff();
	dist->Update();
	dist->GetOutput()->Print(std::cout);
	dist->GetOutput()->GetPointData()->GetScalars()->Print(std::cout);
	std::cout << dist->GetOutput()->GetPointData()->GetScalars()->GetTuple(0)[0] << std::endl;
	std::cout << dist->GetOutput()->GetPointData()->GetScalars()->GetTuple(1)[0] << std::endl;
	std::cout << dist->GetOutput()->GetPointData()->GetScalars()->GetTuple(2)[0] << std::endl;
	vtkSmartPointer<vtkPolyData> res = dist->GetSecondDistanceOutput();
	res->GetPointData()->GetScalars()->Print(std::cout);
	std::cout << res->GetPointData()->GetScalars()->GetTuple(0)[0] << std::endl;
	std::cout << res->GetPointData()->GetScalars()->GetTuple(1)[0] << std::endl;
	std::cout << res->GetPointData()->GetScalars()->GetTuple(2)[0] << std::endl;

	//render
	vtkSmartPointer<vtkPolyDataMapper> map1 = vtkSmartPointer<vtkPolyDataMapper>::New();
	map1->SetInputData(polyData);
	vtkSmartPointer<vtkActor> act1 = vtkSmartPointer<vtkActor>::New();
	act1->SetMapper(map1);
	act1->GetProperty()->SetPointSize(5);

	vtkSmartPointer<vtkPolyDataMapper> map2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	map2->SetInputData(polyData2);
	vtkSmartPointer<vtkActor> act2 = vtkSmartPointer<vtkActor>::New();
	act2->SetMapper(map2);
	act2->GetProperty()->SetPointSize(5);

	//other way
	std::cout << "Other way:" << std::endl;
	vtkSmartPointer<vtkImplicitPolyDataDistance> implicitPolyDataDistance = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	implicitPolyDataDistance->SetInput(polyData);
	for (int i = 0; i < points2->GetNumberOfPoints(); i++)
		std::cout << abs(implicitPolyDataDistance->EvaluateFunction(points2->GetPoint(i))) << std::endl;

	renderer->AddActor(act1);
	renderer->AddActor(act2);
	renderer->ResetCamera();
	window->Render();
	interactor->Start();
}

void ObjectAggregationLevel2(SURFEL::ObjectGraph *ograph, RVL::SurfelGraph *sgraph, RVL::Mesh *mesh)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//Work
	std::vector<vtkSmartPointer<vtkPolyData>> vtkPCobjectlist;
	std::vector<vtkSmartPointer<vtkPolyData>> vtkCHobjectlist;
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();;
	points->SetDataTypeToDouble();
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
	polyData->SetPoints(points);
	polyData->SetVerts(verts);
	vtkSmartPointer<vtkPolyData> polyDataC;
	vtkSmartPointer<vtkCleanPolyData> clean = vtkSmartPointer<vtkCleanPolyData>::New();
	clean->PointMergingOn();
	clean->ToleranceIsAbsoluteOn();
	clean->SetAbsoluteTolerance(0.00001);
	//running through all objects
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *qlistelement;
	SURFEL::Vertex * rvlvertex;
	Surfel *pSurfel;
	int ptIdx = 0;
	double P[3];
	std::cout << std::endl << "Object graph to vtk polydata! " << std::endl;
	for (int iObject = 0; iObject < ograph->NodeArray.n; iObject++)
	{
		pObject = ograph->NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;
		
		//check if object
		if (!piElement)
			continue;
		//
		pSurfel = sgraph->NodeArray.Element + piElement->Idx;
		if (pSurfel->size < 20)
			continue;

		points->Reset();// = vtkSmartPointer<vtkPoints>::New();
		verts->Reset();// = vtkSmartPointer<vtkCellArray>::New();
		ptIdx = 0;
		while (piElement)
		{
			//current surfel vertex list
			pSurfelVertexList = sgraph->surfelVertexList.Element + piElement->Idx;
			qlistelement = pSurfelVertexList->pFirst;
			while (qlistelement)
			{
				rvlvertex = sgraph->vertexArray.Element[qlistelement->Idx];
				P[0] = rvlvertex->P[0];
				P[1] = rvlvertex->P[1];
				P[2] = rvlvertex->P[2];
				points->InsertNextPoint(P);
				verts->InsertNextCell(1);
				verts->InsertCellPoint(ptIdx);
				ptIdx++;

				//Next
				qlistelement = qlistelement->pNext;
			}
			
			//Next
			piElement = piElement->pNext;
		}

		//final object
		//polyData->cleReset();// = vtkSmartPointer<vtkPolyData>::New();
		/*polyData->SetPoints(points);
		polyData->SetVerts(verts);*/
		clean->SetInputData(polyData);
		clean->Update();
		if (clean->GetOutput()->GetNumberOfPoints() == 0)
			continue;
		polyDataC = vtkSmartPointer<vtkPolyData>::New();
		polyDataC->DeepCopy(clean->GetOutput());
		vtkPCobjectlist.push_back(polyDataC);
		std::cout << iObject << " ";
	}

	//Testing possible convex hull and checking distance to surface
	vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New();
	append->SetOutputPointsPrecision(vtkAlgorithm::DesiredOutputPrecision::DEFAULT_PRECISION);
	vtkSmartPointer<vtkDelaunay3D> d3d = vtkSmartPointer<vtkDelaunay3D>::New();
	vtkSmartPointer<vtkGeometryFilter> gf = vtkSmartPointer<vtkGeometryFilter>::New();
	vtkSmartPointer<vtkImplicitPolyDataDistance> implicitPolyDataDistance = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	float tempdist;
	float maxdist;
	for (int i = 0; i < vtkPCobjectlist.size() - 1; i++)
	{
		for (int k = i + 1; k < vtkPCobjectlist.size(); k++)
		{
			append->RemoveAllInputs(); //from previous iteration
			//add imputs and merge
			append->AddInputData(vtkPCobjectlist.at(i));
			append->AddInputData(vtkPCobjectlist.at(k));
			append->Update();
			//run delaunay 3D algorithm and extract geometry
			d3d->SetInputData(append->GetOutput());
			d3d->Update();
			gf->SetInputConnection(d3d->GetOutputPort());
			gf->Update();
			if (gf->GetOutput()->GetNumberOfPolys() == 0)
			{
				std::cout << std::endl << "Combination " << i << ", " << k << " doesn't have polygons!!" << " Number of points: " << vtkPCobjectlist.at(i)->GetNumberOfPoints() << ", " << vtkPCobjectlist.at(k)->GetNumberOfPoints() << std::endl;
				continue;
			}
			//Check distances
			//one way
			implicitPolyDataDistance->SetInput(gf->GetOutput());
			points = vtkPCobjectlist.at(i)->GetPoints();
			maxdist = 0;
			for (int p = 0; p < points->GetNumberOfPoints(); p++)
			{
				tempdist = abs(implicitPolyDataDistance->EvaluateFunction(points->GetPoint(p)));
				if (tempdist > maxdist)
					maxdist = tempdist;
			}
			std::cout << std::endl << "Max dist for combination " << i << ", " << k << ", using " << i << "'s points: " << maxdist << std::endl;
			//other way
			points = vtkPCobjectlist.at(k)->GetPoints();
			maxdist = 0;
			for (int p = 0; p < points->GetNumberOfPoints(); p++)
			{
				tempdist = abs(implicitPolyDataDistance->EvaluateFunction(points->GetPoint(p)));
				if (tempdist > maxdist)
					maxdist = tempdist;
			}
			std::cout << std::endl << "Max dist for combination " << i << ", " << k << ", using " << k << "'s points: " << maxdist << std::endl;

			////debug (visualization of specific combination)
			//if ((i == 3) && (k == 6))
			//{
			//	vtkSmartPointer<vtkPolyDataMapper> mapD;
			//	vtkSmartPointer<vtkActor> actD;
			//	verts = vtkSmartPointer<vtkCellArray>::New();
			//	for (int kk = 0; kk < gf->GetOutput()->GetNumberOfPoints(); kk++)
			//	{
			//		verts->InsertNextCell(1);
			//		verts->InsertCellPoint(kk);
			//	}
			//	polyData = vtkSmartPointer<vtkPolyData>::New();
			//	polyData->DeepCopy(gf->GetOutput());
			//	polyData->SetVerts(verts);
			//	mapD = vtkSmartPointer<vtkPolyDataMapper>::New();
			//	mapD->SetInputData(polyData);
			//	/*vtkSmartPointer<vtkDataSetMapper> delaunayMapper = vtkSmartPointer<vtkDataSetMapper>::New();
			//	delaunayMapper->SetInputConnection(d3d->GetOutputPort());*/
			//	actD = vtkSmartPointer<vtkActor>::New();
			//	actD->SetMapper(mapD);
			//	actD->GetProperty()->SetPointSize(5);
			//	renderer->AddActor(actD);

			//	//renderer->ResetCamera();
			//	//renderer->TwoSidedLightingOff();
			//	//window->Render();
			//	//interactor->Start();
			//}
		}
	}

	//create convex hull for all objects
	std::cout << std::endl << "Delaunay 3D + geometry filter! " << std::endl;
	for (int i = 0; i < vtkPCobjectlist.size(); i++)
	{
		if (vtkPCobjectlist.at(i)->GetNumberOfPoints() == 0)
			continue;
		d3d->SetInputData(vtkPCobjectlist.at(i));
		d3d->Update();
		gf->SetInputConnection(d3d->GetOutputPort());
		gf->Update();
		polyData = vtkSmartPointer<vtkPolyData>::New();
		polyData->DeepCopy(gf->GetOutput());
		vtkCHobjectlist.push_back(polyData);
		std::cout << i << " ";
	}

	//CH visualization
	vtkSmartPointer<vtkPolyDataMapper> map;
	vtkSmartPointer<vtkActor> act;
	for (int i = 0; i < vtkCHobjectlist.size(); i++)
	{
		if (vtkCHobjectlist.at(i)->GetNumberOfPoints() == 0)
			continue;
		verts = vtkSmartPointer<vtkCellArray>::New();
		for (int k = 0; k < vtkCHobjectlist.at(i)->GetNumberOfPoints(); k++)
		{
			verts->InsertNextCell(1);
			verts->InsertCellPoint(k);
		}
		vtkCHobjectlist.at(i)->SetVerts(verts);
		map = vtkSmartPointer<vtkPolyDataMapper>::New();
		map->SetInputData(vtkCHobjectlist.at(i));
		act = vtkSmartPointer<vtkActor>::New();
		act->SetMapper(map);
		act->GetProperty()->SetPointSize(5);
		renderer->AddActor(act);
	}

	//Start VTK
	renderer->ResetCamera();
	renderer->TwoSidedLightingOff();
	window->Render();
	interactor->Start();
}


int main(int argc, char ** argv)
{
	//testvtkdistance();
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
	bool bObjectAggregationLevel2 = false;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &MeshFileName, flags, bSegmentToObjects, bObjectAggregationLevel2);

	ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	if (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF)
		flags |= RVLPCSEGMENT_DEMO_FLAG_SEGMENTATION_GT;

	// Segmentation to surfels.

	bool bSurfelsFromSSF = false;

	SurfelGraph surfels;
	SURFEL::ObjectGraph objects;
	PlanarSurfelDetector detector;
	Mesh mesh;

	char *fileExtension = RVLGETFILEEXTENSION(MeshFileName);

	if (strcmp(fileExtension, "ssf") == 0)
	{
		// Read surfels from a ssf-file.

		std::string ssfFileName(MeshFileName);
		ssfFileName.erase(ssfFileName.find_last_of("."));
		ssfFileName += ".ssf";

		std::cout << "Loading and creating ObjectGraph from SSF!" << std::endl;
		objects.CreateFromSSF(ssfFileName);

		std::cout << "Compute relation cost!" << std::endl;
		objects.ComputeRelationCosts();

		bSurfelsFromSSF = true;
	}
	else
	{
		// Read mesh from file.

		PCLMeshBuilder meshBuilder;

		meshBuilder.CreateParamList(&mem0);

		meshBuilder.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

		int w = 640;
		int h = 480;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));
		pcl::PolygonMesh PCLMesh;

		printf("Creating mesh from %s:\n", MeshFileName);

		//if (mesh.Load(MeshFileName, &meshBuilder, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
		if (meshBuilder.Load(MeshFileName, &mesh, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
			printf("Mesh created.\n");
		else
			printf("ERROR: Mesh can't be created!\n");

		// Segment mesh to surfels.		

		surfels.pMem = &mem;

		surfels.Init(&mesh);

		surfels.CreateParamList(&mem0);

		surfels.ParamList.LoadParams("RVLPCSegmentDemo.cfg");		

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

#ifdef RVLSURFEL_IMAGE_ADJACENCY
		if (flags & RVLPCSEGMENT_DEMO_FLAG_SEGMENTATION_GT)
			surfels.AssignGroundTruthSegmentation(MeshFileName, detector.minSurfelSize);

		// Group surfels into objects.

		if (bSegmentToObjects || (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF))
		{
			printf("Computing realtions between adjacent surfels...");

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

			printf("completed.\n");
		}

		if (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF)
		{
			std::string ssfFileName(MeshFileName);
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

		if (bObjectAggregationLevel2)
		{
			printf("Aggregating objects (LEVEL 2)... ");

			surfels.DetectVertices(&mesh);

			ObjectAggregationLevel2(&objects, &surfels, &mesh);

			printf("completed.\n");
		}
	}

	if (bSurfelsFromSSF)
	{
		if (bSegmentToObjects)
		{
			//Visualization
			cv::imshow("Colored surfel image", GenColoredSurfelImgFromSSF(objects.ssf));
			cv::imshow("Colored segmentation image", GenColoredSegmentationImgFromObjectGraph(&objects));

			//Evaluation
			int E[2];
			int N = 0;
			objects.CalculateOverAndUnderSegmentation(E, N, false);
			std::cout << "Oversegmenation error: " << 1 - E[0] / (float)N << std::endl;
			std::cout << "Undersegmenation error: " << E[1] / (float)N << std::endl;

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

	// free memory

	delete detector.pTimer;
	delete[] MeshFileName;

	return 0;
#endif
}

