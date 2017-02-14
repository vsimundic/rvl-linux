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
#include "vtkCamera.h"
#include "vtkWindowToImageFilter.h"
#include "vtkHull.h"
#include "vtkSphereSource.h"
#include "vtkPlanes.h"
#include <vtkPlaneSource.h>

//#define RVLPCSEGMENT_DEMO_CREATE_TRAINING_DATA

#define RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY			0x00000001
#define RVLPCSEGMENT_DEMO_FLAG_SAVE_SSF			0x00000002
#define RVLPCSEGMENT_DEMO_FLAG_SEGMENTATION_GT	0x00000004

using namespace RVL;

#include "RVLPCSegmentCreateTrainingData.h"

void RunMainProg(
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

cv::Mat GenerateVTKDepthImage(vtkSmartPointer<vtkRenderWindow> renWin, int width, int height, double fx, double fy, double cx, double cy, double horizFOV, double vertFOV, double clipnear, double clipfar)
{
	//opencv
	cv::Mat renderedDepthImg(height, width, CV_16UC1, cv::Scalar::all(0));

	// create the camera
	vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();

	// the camera can stay at the origin because we are transforming the scene objects
	camera->SetPosition(0, 0, 0);
	//// look in the +Z direction of the camera coordinate system
	camera->SetFocalPoint(0, 0, 1);
	//// the camera Y axis points down
	camera->SetViewUp(0, -1, 0);
	//// ensure the relevant range of depths are rendered
	camera->SetClippingRange(clipnear, clipfar);
	//// convert the principal point to window center (normalized coordinate system) and set it
	double wcx = -2 * (cx - width / 2) / width;
	double wcy = 2 * (cy - height / 2) / height;
	camera->SetWindowCenter(wcx, wcy);
	// convert the focal length to view angle and set it
	double view_angle = 57.2958 * (2.0 * atan2(height / 2.0, fy));
	camera->SetViewAngle(view_angle);

	//get old camera
	vtkSmartPointer<vtkCamera> oldCamera = renWin->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
	int oldwidth = renWin->GetSize()[0];
	int oldheight = renWin->GetSize()[1];
	renWin->SetSize(width, height);
	renWin->GetRenderers()->GetFirstRenderer()->SetActiveCamera(camera);
	renWin->Render();

	float *d = renWin->GetZbufferData(0, 0, width - 1, 479);
	double tempd;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (d[y * width + x] == 1.0) //Check if valid
			{
				renderedDepthImg.at<uint16_t>(y, x) = 0;
				continue;
			}
			tempd = (d[y * width + x] * (1.0 / clipfar - 1.0 / clipnear) * clipnear + 1.0) / clipnear;
			renderedDepthImg.at<uint16_t>(y, x) = (uint16_t)((1.0 / tempd) * 1000); //in milimeters
		}
	}

	//returning to old
	renWin->GetRenderers()->GetFirstRenderer()->SetActiveCamera(oldCamera);
	renWin->SetSize(oldwidth, oldheight);
	renWin->Render();

	cv::flip(renderedDepthImg, renderedDepthImg, 0); //flip image
	
	/*//useful for depth image visualization
	cv::Mat depthMat(height, width, CV_8UC1);
	double minVal, maxVal;
	cv::minMaxLoc(renderedDepthImg, &minVal, &maxVal);
	renderedDepthImg.convertTo(depthMat, CV_8U, -255.0f / maxVal, 255.0f);
	cv::imshow("Rendered depth image", depthMat);
	cv::waitKey(1);*/

	return renderedDepthImg;
}

cv::Mat GenerateVTKDepthImage_Kinect(vtkSmartPointer<vtkRenderWindow> renWin, double clipnear, double clipfar)
{
	int width = 640;
	int height = 480; 
	double fx = 581.45624912987f;
	double fy = 543.1221626989097f;
	double cx = 317.2825290065861f;
	double cy = 240.955527515504f;
	double horizFOV = 58.5;
	double vertFOV = 46.6;
	
	//opencv
	cv::Mat renderedDepthImg(height, width, CV_16UC1, cv::Scalar::all(0));

	// create the camera
	vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();

	// the camera can stay at the origin because we are transforming the scene objects
	camera->SetPosition(0, 0, 0);
	//// look in the +Z direction of the camera coordinate system
	camera->SetFocalPoint(0, 0, 1);
	//// the camera Y axis points down
	camera->SetViewUp(0, -1, 0);
	//// ensure the relevant range of depths are rendered
	camera->SetClippingRange(clipnear, clipfar);
	//// convert the principal point to window center (normalized coordinate system) and set it
	double wcx = -2 * (cx - width / 2) / width;
	double wcy = 2 * (cy - height / 2) / height;
	camera->SetWindowCenter(wcx, wcy);
	// convert the focal length to view angle and set it
	double view_angle = 57.2958 * (2.0 * atan2(height / 2.0, fy));
	camera->SetViewAngle(view_angle);//vertical 46.6

	//get old camera
	vtkSmartPointer<vtkCamera> oldCamera = renWin->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
	int oldwidth = renWin->GetSize()[0];
	int oldheight = renWin->GetSize()[1];
	renWin->SetSize(width, height);
	renWin->GetRenderers()->GetFirstRenderer()->SetActiveCamera(camera);
	renWin->Render();

	float *d = renWin->GetZbufferData(0, 0, width - 1, height - 1);
	double tempd;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (d[y * width + x] == 1.0) //Check if valid
			{
				renderedDepthImg.at<uint16_t>(y, x) = 0;
				continue;
			}
			tempd = (d[y * width + x] * (1.0 / clipfar - 1.0 / clipnear) * clipnear + 1.0) / clipnear;
			renderedDepthImg.at<uint16_t>(y, x) = (uint16_t)((1.0 / tempd) * 1000); //in milimeters
		}
	}

	//returning to old
	renWin->GetRenderers()->GetFirstRenderer()->SetActiveCamera(oldCamera);
	renWin->SetSize(oldwidth, oldheight);

	cv::flip(renderedDepthImg, renderedDepthImg, 0); //flip vertically

	return renderedDepthImg;
}

//void TestVTK_Plane_z_buffer(float distancefromZ, int width, int height, float fx, float fy, float cx, float cy, float horizFOV, float clipnear, float clipfar)
//{
//	// Initialize VTK.
//	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
//	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
//	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	window->AddRenderer(renderer);
//	window->SetSize(640, 480);
//	interactor->SetRenderWindow(window);
//	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
//	interactor->SetInteractorStyle(style);
//	renderer->SetBackground(0.5294, 0.8078, 0.9803);
//
//	vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
//	/*plane->SetCenter(0.0, 0.0, distancefromZ);*/
//	plane->SetNormal(0.0, 0.0, -1.0);
//	plane->SetOrigin(-500, -500, distancefromZ);
//	plane->SetPoint1(-500, 500, distancefromZ);
//	plane->SetPoint2(500, -500, distancefromZ);
//	plane->SetResolution(100, 100);
//	plane->Update();
//
//	vtkSmartPointer<vtkPolyDataMapper> planeMap = vtkSmartPointer<vtkPolyDataMapper>::New();
//	planeMap->SetInputConnection(plane->GetOutputPort());
//	vtkSmartPointer<vtkActor> planeActor = vtkSmartPointer<vtkActor>::New();
//	planeActor->SetMapper(planeMap);
//	renderer->AddActor(planeActor);
//
//	//opencv
//	cv::Mat renderedDepthImg(480, 640, CV_16UC1, cv::Scalar::all(0));
//
//	// create the camera
//	vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
//	//camera->ParallelProjectionOn();
//	//camera->SetParallelScale(0.5);
//	//camera->DeepCopy(renWin->GetRenderers()->GetFirstRenderer()->GetActiveCamera());
//	//// convert camera rotation and translation into a 4x4 homogeneous transformation matrix
//	//vtkSmartPointer<vtkMatrix4x4> camera_RT = make_transform(camera_rot, camera_trans);
//	//// apply the transform to scene objects
//	//camera->SetModelTransformMatrix(camera_RT);
//
//	// the camera can stay at the origin because we are transforming the scene objects
//	camera->SetPosition(0, 0, 0);
//	//// look in the +Z direction of the camera coordinate system
//	camera->SetFocalPoint(0, 0, 1);
//	//// the camera Y axis points down
//	camera->SetViewUp(0, -1, 0);
//	//// ensure the relevant range of depths are rendered
//	camera->SetClippingRange(clipnear, clipfar);
//	//// convert the principal point to window center (normalized coordinate system) and set it
//	double wcx = -2 * (cx - width / 2) / width;
//	double wcy = 2 * (cy - height / 2) / height;
//	camera->SetWindowCenter(wcx, wcy);
//	// convert the focal length to view angle and set it
//	double view_angle = 57.2958 * (2.0 * atan2(height / 2.0, fy));
//	//std::cout << "view_angle = " << view_angle << std::endl;
//	camera->SetViewAngle(view_angle);//vertical 46,6
//
//	window->GetRenderers()->GetFirstRenderer()->SetActiveCamera(camera);
//	window->Render();
//	window->GetInteractor()->Start();
//	/*vtkSmartPointer<vtkWindowToImageFilter> imgf = vtkSmartPointer<vtkWindowToImageFilter>::New();
//	imgf->SetInputBufferTypeToZBuffer();
//	imgf->SetInput(renWin);
//	imgf->Update();
//	imgf->GetOutput();
//	vtkSmartPointer<vtkImageData> imgdata = imgf->GetOutput();*/
//
//
//	float *d = window->GetZbufferData(0, 0, 639, 479);
//	for (int y = 0; y < 480; y++)
//	{
//		for (int x = 0; x < 640; x++)
//		{
//			renderedDepthImg.at<uint16_t>(y, x) = (uint16_t)((clipnear + (d[y * 640 + x] * (clipfar - clipnear))) * 1000);// (uint16_t)static_cast<double*>(imgdata->GetScalarPointer(x, y, 0)); //static_cast<float*>(imgdata->GetScalarPointer(x, y, 0))[0] * 1000;//  d[y * 640 + x] * 1000;//
//		}
//	}
//
//	/*cv::flip(renderedDepthImg, renderedDepthImg, 0);
//	cv::Mat depthMat(480, 640, CV_8UC1);
//	double minVal, maxVal;
//	cv::minMaxLoc(renderedDepthImg, &minVal, &maxVal);
//	renderedDepthImg.convertTo(depthMat, CV_8U, -255.0f / maxVal, 255.0f);
//	cv::imshow("Rendered depth image", depthMat);
//	cv::waitKey();*/
//}

cv::Mat GenerateVTKPolyDataDepthImage_Kinect(vtkSmartPointer<vtkPolyData> pd)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
	renWin->OffScreenRenderingOn(); //OFF-SCREEN RENDERING
	renWin->AddRenderer(renderer);
	renWin->SetSize(640, 480); //HARDCODED 640X480 IMAGE

	//adding polydata actor
	vtkSmartPointer<vtkPolyDataMapper>	map = vtkSmartPointer<vtkPolyDataMapper>::New();
	map->SetInputData(pd);
	vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
	act->SetMapper(map);
	renderer->AddActor(act);

	//find zbounds
	double *bounds;
	pd->GetPoints()->ComputeBounds(); //just in case
	bounds = pd->GetPoints()->GetBounds(); // (Xmin, Xmax) = (bounds[0], bounds[1]), (Ymin, Ymax) = (bounds[2], bounds[3]), (Zmin, Zmax) = (bounds[4], bounds[5])

	//Generate and return Kinect-like depth image
	return GenerateVTKDepthImage_Kinect(renWin, bounds[4], bounds[5]);
}

void ObjectAggregationLevel2(SURFEL::ObjectGraph *ograph, RVL::SurfelGraph *sgraph, RVL::Mesh *mesh, std::string MeshFileName)
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

		if (pObject->size < 20)
			continue;

		piElement = pObject->elementList.pFirst;
		
		//check if object
		if (!piElement)
			continue;
		//
		pSurfel = sgraph->NodeArray.Element + piElement->Idx;
		/*if (pSurfel->size < 20)
			continue;*/

		points->Reset();// = vtkSmartPointer<vtkPoints>::New();
		verts->Reset();// = vtkSmartPointer<vtkCellArray>::New();
		ptIdx = 0;
		//while (piElement)
		//{
		//	//current surfel vertex list
		//	pSurfelVertexList = sgraph->surfelVertexList.Element + piElement->Idx;
		//	qlistelement = pSurfelVertexList->pFirst;
		//	while (qlistelement)
		//	{
		//		rvlvertex = sgraph->vertexArray.Element[qlistelement->Idx];
		//		P[0] = rvlvertex->P[0];
		//		P[1] = rvlvertex->P[1];
		//		P[2] = rvlvertex->P[2];
		//		points->InsertNextPoint(P);
		//		verts->InsertNextCell(1);
		//		verts->InsertCellPoint(ptIdx);
		//		ptIdx++;

		//		//Next
		//		qlistelement = qlistelement->pNext;
		//	}
		//	
		//	//Next
		//	piElement = piElement->pNext;
		//}

		for (int i = 0; i < ograph->additionalObjectData.CHVertexIndices.at(iObject).size(); i++)
		{
			rvlvertex = sgraph->vertexArray.Element[ograph->additionalObjectData.CHVertexIndices.at(iObject).at(i)];
			P[0] = rvlvertex->P[0];
			P[1] = rvlvertex->P[1];
			P[2] = rvlvertex->P[2];
			points->InsertNextPoint(P);
			verts->InsertNextCell(1);
			verts->InsertCellPoint(ptIdx);
			ptIdx++;
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

	//PC visualization
	vtkSmartPointer<vtkPolyDataMapper> map;
	vtkSmartPointer<vtkActor> act;
	srand(time(NULL));
	for (int i = 0; i < vtkPCobjectlist.size(); i++)
	{
		if (vtkPCobjectlist.at(i)->GetNumberOfPoints() == 0)
			continue;
		map = vtkSmartPointer<vtkPolyDataMapper>::New();
		map->SetInputData(vtkPCobjectlist.at(i));
		act = vtkSmartPointer<vtkActor>::New();
		act->SetMapper(map);
		act->GetProperty()->SetPointSize(5);
		act->GetProperty()->SetColor((double)(rand() % 255) / 255.0, (double)(rand() % 255) / 255.0, (double)(rand() % 255) / 255.0);
		renderer->AddActor(act);
	}

	//Testing possible convex hull and checking distance to surface
	/*vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New();
	append->SetOutputPointsPrecision(vtkAlgorithm::DesiredOutputPrecision::DEFAULT_PRECISION);
	vtkSmartPointer<vtkDelaunay3D> d3d = vtkSmartPointer<vtkDelaunay3D>::New();
	vtkSmartPointer<vtkGeometryFilter> gf = vtkSmartPointer<vtkGeometryFilter>::New();
	vtkSmartPointer<vtkImplicitPolyDataDistance> implicitPolyDataDistance = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	float tempdist;
	float maxdist;*/
	//for (int i = 0; i < vtkPCobjectlist.size() - 1; i++)
	//{
	//	for (int k = i + 1; k < vtkPCobjectlist.size(); k++)
	//	{
	//		append->RemoveAllInputs(); //from previous iteration
	//		//add imputs and merge
	//		append->AddInputData(vtkPCobjectlist.at(i));
	//		append->AddInputData(vtkPCobjectlist.at(k));
	//		append->Update();
	//		//run delaunay 3D algorithm and extract geometry
	//		d3d->SetInputData(append->GetOutput());
	//		d3d->Update();
	//		gf->SetInputConnection(d3d->GetOutputPort());
	//		gf->Update();
	//		if (gf->GetOutput()->GetNumberOfPolys() == 0)
	//		{
	//			std::cout << std::endl << "Combination " << i << ", " << k << " doesn't have polygons!!" << " Number of points: " << vtkPCobjectlist.at(i)->GetNumberOfPoints() << ", " << vtkPCobjectlist.at(k)->GetNumberOfPoints() << std::endl;
	//			continue;
	//		}
	//		//Check distances
	//		//one way
	//		implicitPolyDataDistance->SetInput(gf->GetOutput());
	//		points = vtkPCobjectlist.at(i)->GetPoints();
	//		maxdist = 0;
	//		for (int p = 0; p < points->GetNumberOfPoints(); p++)
	//		{
	//			tempdist = abs(implicitPolyDataDistance->EvaluateFunction(points->GetPoint(p)));
	//			if (tempdist > maxdist)
	//				maxdist = tempdist;
	//		}
	//		std::cout << std::endl << "Max dist for combination " << i << ", " << k << ", using " << i << "'s points: " << maxdist << std::endl;
	//		//other way
	//		points = vtkPCobjectlist.at(k)->GetPoints();
	//		maxdist = 0;
	//		for (int p = 0; p < points->GetNumberOfPoints(); p++)
	//		{
	//			tempdist = abs(implicitPolyDataDistance->EvaluateFunction(points->GetPoint(p)));
	//			if (tempdist > maxdist)
	//				maxdist = tempdist;
	//		}
	//		std::cout << std::endl << "Max dist for combination " << i << ", " << k << ", using " << k << "'s points: " << maxdist << std::endl;

	//		////debug (visualization of specific combination)
	//		//if ((i == 3) && (k == 6))
	//		//{
	//		//	vtkSmartPointer<vtkPolyDataMapper> mapD;
	//		//	vtkSmartPointer<vtkActor> actD;
	//		//	verts = vtkSmartPointer<vtkCellArray>::New();
	//		//	for (int kk = 0; kk < gf->GetOutput()->GetNumberOfPoints(); kk++)
	//		//	{
	//		//		verts->InsertNextCell(1);
	//		//		verts->InsertCellPoint(kk);
	//		//	}
	//		//	polyData = vtkSmartPointer<vtkPolyData>::New();
	//		//	polyData->DeepCopy(gf->GetOutput());
	//		//	polyData->SetVerts(verts);
	//		//	mapD = vtkSmartPointer<vtkPolyDataMapper>::New();
	//		//	mapD->SetInputData(polyData);
	//		//	/*vtkSmartPointer<vtkDataSetMapper> delaunayMapper = vtkSmartPointer<vtkDataSetMapper>::New();
	//		//	delaunayMapper->SetInputConnection(d3d->GetOutputPort());*/
	//		//	actD = vtkSmartPointer<vtkActor>::New();
	//		//	actD->SetMapper(mapD);
	//		//	actD->GetProperty()->SetPointSize(5);
	//		//	renderer->AddActor(actD);

	//		//	//renderer->ResetCamera();
	//		//	//renderer->TwoSidedLightingOff();
	//		//	//window->Render();
	//		//	//interactor->Start();
	//		//}
	//	}
	//}

	////Testing possible convex hull (split objects) by comparing rendered and measured depth images
	//std::string depthImgFileName(MeshFileName);
	//depthImgFileName.erase(depthImgFileName.find_last_of("."));
	//depthImgFileName += "d.png";
	//cv::Mat depthImg = cv::imread(depthImgFileName, cv::ImreadModes::IMREAD_ANYDEPTH);
	//cv::Mat renderedDepthImg;
	//vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New();
	//append->SetOutputPointsPrecision(vtkAlgorithm::DesiredOutputPrecision::DEFAULT_PRECISION);
	//vtkSmartPointer<vtkDelaunay3D> d3d = vtkSmartPointer<vtkDelaunay3D>::New();
	//vtkSmartPointer<vtkGeometryFilter> gf = vtkSmartPointer<vtkGeometryFilter>::New();
	//int noPixGreater;
	//int noPixLesser;
	//int noPixLesser5mm;
	//int noPixLesser10mm;
	//int noPixLesser15mm;
	//int noPixMWSupport;
	//int noPixAmbiguousLesser;
	//int noPixAmbiguousGreater;
	//vtkSmartPointer<vtkPolyDataMapper> map;
	//vtkSmartPointer<vtkActor> act;
	//bool found = false;
	//for (int i = 0; i < vtkPCobjectlist.size() - 1; i++)
	//{
	//	found = false;
	//	for (int k = i + 1; k < vtkPCobjectlist.size(); k++)
	//	{
	//		append->RemoveAllInputs(); //from previous iteration
	//		//add imputs and merge
	//		append->AddInputData(vtkPCobjectlist.at(i));
	//		append->AddInputData(vtkPCobjectlist.at(k));
	//		append->Update();
	//		//run delaunay 3D algorithm and extract geometry
	//		d3d->SetInputData(append->GetOutput());
	//		d3d->Update();
	//		gf->SetInputConnection(d3d->GetOutputPort());
	//		gf->Update();
	//		if (gf->GetOutput()->GetNumberOfPolys() == 0)
	//		{
	//			std::cout << std::endl << "Combination " << i << ", " << k << " doesn't have polygons!!" << " Number of points: " << vtkPCobjectlist.at(i)->GetNumberOfPoints() << ", " << vtkPCobjectlist.at(k)->GetNumberOfPoints() << std::endl;
	//			continue;
	//		}
	//		//rendering depth
	//		renderedDepthImg = GenerateVTKPolyDataDepthImage_Kinect(gf->GetOutput());
	//		//erosion
	//		//cv::Mat renderedDepthImg_E(480, 640, CV_16UC1, cv::Scalar::all(0));
	//		cv::erode(renderedDepthImg, renderedDepthImg, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(21, 21)/*, cv::Point(5, 5)*/));
	//		////debug
	//		//cv::Mat d1(480, 640, CV_8UC1);
	//		//cv::Mat d2(480, 640, CV_8UC1);
	//		//double minVal, maxVal;
	//		//cv::minMaxLoc(renderedDepthImg, &minVal, &maxVal);
	//		//renderedDepthImg.convertTo(d1, CV_8U, -255.0f / maxVal, 255.0f);
	//		//cv::imshow("Rendered depth image", d1);
	//		//cv::minMaxLoc(renderedDepthImg_E, &minVal, &maxVal);
	//		//renderedDepthImg_E.convertTo(d2, CV_8U, -255.0f / maxVal, 255.0f);
	//		//cv::imshow("Eroded rendered depth image", d2);
	//		//cv::waitKey();
	//		//
	//		//Running through all pixels that have depth
	//		noPixGreater = 0;
	//		noPixLesser = 0;
	//		noPixMWSupport = 0;
	//		/*noPixLower5mm = 0;
	//		noPixLower10mm = 0;
	//		noPixLower15mm = 0;*/
	//		noPixAmbiguousLesser = 0;
	//		noPixAmbiguousGreater = 0;
	//		for (int y = 0; y < 480; y++)
	//		{
	//			for (int x = 0; x < 640; x++)
	//			{
	//				if (renderedDepthImg.at<uint16_t>(y, x) == 0) //invalid pixel
	//					continue;
	//				else if ((renderedDepthImg.at<uint16_t>(y, x) > 0) && (depthImg.at<uint16_t>(y, x) == 0)) //pixel with rendered depth but no actual measurement
	//					noPixMWSupport++;
	//				else if (renderedDepthImg.at<uint16_t>(y, x) >= depthImg.at<uint16_t>(y, x))	//pixel whose rendered depth is grater than actual depth (it is further away)
	//				{
	//					if (abs(renderedDepthImg.at<uint16_t>(y, x) - depthImg.at<uint16_t>(y, x) <= 10))	// if the difference is less or equal then 10mm it is ambiguous
	//					{
	//						noPixAmbiguousGreater++;
	//						noPixLesser++;
	//					}
	//					else
	//						noPixGreater++;
	//				}
	//				else //pixel whose rendered depth is lesser than actual depth (it is nearer to the camera)
	//				{
	//					if (abs(renderedDepthImg.at<uint16_t>(y, x) - depthImg.at<uint16_t>(y, x) <= 10))	// if the difference is less or equal then 10mm it is ambiguous
	//						noPixAmbiguousLesser++;
	//					/*else*/
	//						noPixLesser++;
	//					/*if (abs(renderedDepthImg.at<uint16_t>(y, x) - depthImg.at<uint16_t>(y, x)) <= 5)
	//						noPixLower5mm++;
	//					else if (abs(renderedDepthImg.at<uint16_t>(y, x) - depthImg.at<uint16_t>(y, x)) <= 10)
	//						noPixLower10mm++;
	//					else if (abs(renderedDepthImg.at<uint16_t>(y, x) - depthImg.at<uint16_t>(y, x)) <= 15)
	//						noPixLower15mm++;*/
	//				}
	//			}
	//		}
	//		//std::cout << std::endl << "Combination " << i << ", " << k << " noPixUpper = " << noPixUpper << ", noPixLower = " << noPixLower << ", noPixMWSupport = " << noPixMWSupport /*<< " noPixLower5mm = " << noPixLower5mm << ", noPixLower10mm = " << noPixLower10mm << ", noPixLower15mm = " << noPixLower15mm */<< std::endl;
	//		std::cout << std::endl << "Combination " << i << ", " << k << " Ratio (greater/lesser)  = " << (float)(noPixGreater) / (float)(noPixLesser) << ", noPixAmbiguousLesser = " << noPixAmbiguousLesser << ", noPixAmbiguousGreater = " << noPixAmbiguousGreater <<std::endl;
	//	
	//		//For visualization
	//		if ((noPixGreater > 0) && (noPixLesser > 0) && (((float)(noPixGreater) / (float)(noPixLesser)) > 0.5) && (((float)(noPixGreater) / (float)(noPixLesser)) < 1.0))
	//		{
	//			polyData = vtkSmartPointer<vtkPolyData>::New();
	//			polyData->DeepCopy(gf->GetOutput());
	//			map = vtkSmartPointer<vtkPolyDataMapper>::New();
	//			map->SetInputData(polyData);
	//			act = vtkSmartPointer<vtkActor>::New();
	//			act->SetMapper(map);
	//			renderer->AddActor(act);
	//			found = true;
	//		}
	//	}
	//	if (!found)
	//	{
	//		d3d->SetInputData(vtkPCobjectlist.at(i));
	//		d3d->Update();
	//		gf->SetInputConnection(d3d->GetOutputPort());
	//		gf->Update();
	//		/*if (gf->GetOutput()->GetNumberOfPolys() == 0)
	//		{*/
	//			polyData = vtkSmartPointer<vtkPolyData>::New();
	//			polyData->DeepCopy(gf->GetOutput());
	//			map = vtkSmartPointer<vtkPolyDataMapper>::New();
	//			map->SetInputData(polyData);
	//			act = vtkSmartPointer<vtkActor>::New();
	//			act->SetMapper(map);
	//			renderer->AddActor(act);
	//		//}
	//	}
	//}

	////create convex hull for all objects
	//std::cout << std::endl << "Delaunay 3D + geometry filter! " << std::endl;
	//double zbounds[2] = {100.0, 0.0};
	//double *bounds;
	//for (int i = 0; i < vtkPCobjectlist.size(); i++)
	//{
	//	if (vtkPCobjectlist.at(i)->GetNumberOfPoints() == 0)
	//		continue;
	//	d3d->SetInputData(vtkPCobjectlist.at(i));
	//	d3d->Update();
	//	gf->SetInputConnection(d3d->GetOutputPort());
	//	gf->Update();
	//	polyData = vtkSmartPointer<vtkPolyData>::New();
	//	polyData->DeepCopy(gf->GetOutput());
	//	vtkCHobjectlist.push_back(polyData);
	//	std::cout << i << " ";
	//	//get zbounds
	//	polyData->GetPoints()->ComputeBounds();
	//	bounds = polyData->GetPoints()->GetBounds();
	//	if (bounds[4] < zbounds[0])
	//		zbounds[0] = bounds[4];
	//	if (bounds[5] > zbounds[1])
	//		zbounds[1] = bounds[5];
	//}

	////CH visualization
	//vtkSmartPointer<vtkPolyDataMapper> map;
	//vtkSmartPointer<vtkActor> act;
	//for (int i = 0; i < vtkCHobjectlist.size(); i++)
	//{
	//	if (vtkCHobjectlist.at(i)->GetNumberOfPoints() == 0)
	//		continue;
	//	verts = vtkSmartPointer<vtkCellArray>::New();
	//	for (int k = 0; k < vtkCHobjectlist.at(i)->GetNumberOfPoints(); k++)
	//	{
	//		verts->InsertNextCell(1);
	//		verts->InsertCellPoint(k);
	//	}
	//	vtkCHobjectlist.at(i)->SetVerts(verts);
	//	map = vtkSmartPointer<vtkPolyDataMapper>::New();
	//	map->SetInputData(vtkCHobjectlist.at(i));
	//	act = vtkSmartPointer<vtkActor>::New();
	//	act->SetMapper(map);
	//	act->GetProperty()->SetPointSize(5);
	//	renderer->AddActor(act);
	//}

	//Start VTK
	renderer->ResetCamera();
	renderer->TwoSidedLightingOff();
	window->Render();

	//cv::Mat rendereddepth = GenerateVTKDepthImage(window, 640, 480, 581.45624912987f, 543.1221626989097f, 317.2825290065861f, 240.955527515504f, 57.7, 46.6, zbounds[0], zbounds[1]);
	////debug
	//uint16_t d1 = depthImg.at<uint16_t>(300, 200);
	//uint16_t d2 = rendereddepth.at<uint16_t>(300, 200);


	interactor->Start();
}



int main(int argc, char ** argv)
{
	//TestVTK_Plane_z_buffer(2, 640, 480, 581.45624912987f, 543.1221626989097f, 317.2825290065861f, 240.955527515504f, 60, 1.99999, 2.00001);
	//RenderCTIConvexHull();
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

void RunMainProg(
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

	SurfelGraph surfels;
	SURFEL::ObjectGraph objects;
	PlanarSurfelDetector detector;
	Mesh mesh;

	objects.CreateParamList(mem0);

	objects.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	if (objects.relationClassifier == RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM)
	{		
		std::cout << "Initializing SVM Classifier!" << std::endl;
		objects.InitSVMClassifier(SVMClassifierParamsFileName);
	}

	char *fileExtension = RVLGETFILEEXTENSION(MeshFilePathName);

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

			//Filko
			objects.DetermineObjectConvexityData(0.005, 0.9);
			ObjectAggregationLevel2(&objects, &surfels, &mesh, MeshFilePathName);
			//

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

			if (fp)
				fprintf(fp, "%s\t%d\t%d\t%d\n", fileName, E[0], E[1], N);

			if (!bSequence)
			{
				if (fp)
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
