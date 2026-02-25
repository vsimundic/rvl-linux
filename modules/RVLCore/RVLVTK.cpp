#include "RVLPlatform.h"
#include "RVLVTK.h"
#include <stdint.h>

namespace RVL
{
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
		vtkIdType triangle[3] = { 0, 1, 2 };
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

	void TestVTK_Plane_z_buffer(float distancefromZ, int width, int height, float fx, float fy, float cx, float cy, float clipnear, float clipfar)
	{
		// Initialize VTK.
		vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
		vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
		vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
		window->AddRenderer(renderer);
		window->SetSize(640, 480);
		interactor->SetRenderWindow(window);
		vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
		interactor->SetInteractorStyle(style);
		renderer->SetBackground(0.5294, 0.8078, 0.9803);

		vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
		/*plane->SetCenter(0.0, 0.0, distancefromZ);*/
		plane->SetNormal(0.0, 0.0, -1.0);
		plane->SetOrigin(-500, -500, distancefromZ);
		plane->SetPoint1(-500, 500, distancefromZ);
		plane->SetPoint2(500, -500, distancefromZ);
		plane->SetResolution(100, 100);
		plane->Update();

		vtkSmartPointer<vtkPolyDataMapper> planeMap = vtkSmartPointer<vtkPolyDataMapper>::New();
		planeMap->SetInputConnection(plane->GetOutputPort());
		vtkSmartPointer<vtkActor> planeActor = vtkSmartPointer<vtkActor>::New();
		planeActor->SetMapper(planeMap);
		renderer->AddActor(planeActor);

		//opencv
		cv::Mat renderedDepthImg(480, 640, CV_16UC1, cv::Scalar::all(0));

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
		//std::cout << "view_angle = " << view_angle << std::endl;
		camera->SetViewAngle(view_angle);//vertical 46,6

		window->GetRenderers()->GetFirstRenderer()->SetActiveCamera(camera);
		window->Render();
		window->GetInteractor()->Start();

		float *d = window->GetZbufferData(0, 0, 639, 479);
		for (int y = 0; y < 480; y++)
		{
			for (int x = 0; x < 640; x++)
			{
				renderedDepthImg.at<uint16_t>(y, x) = (uint16_t)((clipnear + (d[y * 640 + x] * (clipfar - clipnear))) * 1000);// (uint16_t)static_cast<double*>(imgdata->GetScalarPointer(x, y, 0)); //static_cast<float*>(imgdata->GetScalarPointer(x, y, 0))[0] * 1000;//  d[y * 640 + x] * 1000;//
			}
		}

		cv::flip(renderedDepthImg, renderedDepthImg, 0);
		cv::Mat depthMat(480, 640, CV_8UC1);
		double minVal, maxVal;
		cv::minMaxLoc(renderedDepthImg, &minVal, &maxVal);
		renderedDepthImg.convertTo(depthMat, CV_8U, -255.0f / maxVal, 255.0f);
		cv::imshow("Rendered depth image", depthMat);
		cv::waitKey();
	}

	cv::Mat GenerateVTKDepthImage(vtkSmartPointer<vtkRenderWindow> renWin, int width, int height, double fx, double fy, double cx, double cy, double clipnear, double clipfar)
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

		return renderedDepthImg;
	}

	cv::Mat GenerateVTKDepthImage(vtkSmartPointer<vtkRenderWindow> renWin, vtkSmartPointer<vtkCamera> camera, int width, int height)
	{
		//opencv
		cv::Mat renderedDepthImg(height, width, CV_16UC1, cv::Scalar::all(0));

		//get old camera
		vtkSmartPointer<vtkCamera> oldCamera = renWin->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
		int oldwidth = renWin->GetSize()[0];
		int oldheight = renWin->GetSize()[1];
		//Set new camera
		renWin->SetSize(width, height);
		renWin->GetRenderers()->GetFirstRenderer()->SetActiveCamera(camera);
		renWin->Render();
		double cliprange[2];
		camera->GetClippingRange(cliprange);
		//get Z Buffer
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
				tempd = (d[y * width + x] * (1.0 / cliprange[1] - 1.0 / cliprange[0]) * cliprange[0] + 1.0) / cliprange[0];
				renderedDepthImg.at<uint16_t>(y, x) = (uint16_t)((1.0 / tempd) * 1000); //in milimeters
			}
		}

		////returning old camera
		//renWin->GetRenderers()->GetFirstRenderer()->SetActiveCamera(oldCamera);
		//renWin->SetSize(oldwidth, oldheight);
		//renWin->Render();

		//flip image
		cv::flip(renderedDepthImg, renderedDepthImg, 0); 

		return renderedDepthImg;
	}

	cv::Mat GenerateVTKDepthImage_Kinect(
		vtkSmartPointer<vtkRenderWindow> renWin, 
		double scale, 
		double clipnear, 
		double clipfar, 
		int width, 
		int height, 
		double fy, 
		double cx, 
		double cy, 
		float *R, 
		float *t,
		bool bClipFarToZero,
		uchar **pRGB)
	{
		double fx = 581.45624912987f;
		double horizFOV = 58.5;
		double vertFOV = 46.6;

		//opencv
		cv::Mat renderedDepthImg(height, width, CV_16UC1, cv::Scalar::all(0));

		// create the camera
		vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();

		if (t)
			camera->SetPosition(t[0], t[1], t[2]);
		else
			// the camera can stay at the origin because we are transforming the scene objects
			camera->SetPosition(0, 0, 0);

		if (R)
		{
			camera->SetFocalPoint(R[2], R[5], R[8]);
			camera->SetViewUp(-R[1], -R[4], -R[7]);
		}
		else
		{
			//// look in the +Z direction of the camera coordinate system
			camera->SetFocalPoint(0, 0, 1);
			//// the camera Y axis points down
			camera->SetViewUp(0, -1, 0);
		}
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
				if (bClipFarToZero)
				{
					if (d[y * width + x] == 1.0) //Check if valid
					{
						renderedDepthImg.at<uint16_t>(y, x) = 0;
						continue;
					}
				}
				tempd = (d[y * width + x] * (1.0 / clipfar - 1.0 / clipnear) * clipnear + 1.0) / clipnear;
				renderedDepthImg.at<uint16_t>(y, x) = (uint16_t)((1.0 / tempd) * scale); //in milimeters
			}
		}

		if (pRGB)
			*pRGB = renWin->GetPixelData(0, 0, width - 1, height - 1, 1);

		//returning to old
		renWin->GetRenderers()->GetFirstRenderer()->SetActiveCamera(oldCamera);
		renWin->SetSize(oldwidth, oldheight);

		cv::flip(renderedDepthImg, renderedDepthImg, 0); //flip vertically

		return renderedDepthImg;
	}

	cv::Mat GenerateVTKPolyDataDepthImage_Kinect(
		vtkSmartPointer<vtkPolyData> pd, 
		double scale, 
		double clipnearIn, 
		double clipfarIn, 
		int w, 
		int h, 
		double fy, 
		double cx, 
		double cy, 
		float *R, 
		float *t,
		bool bClipFarToZero,
		uchar **pRGB,
		bool bShading)
	{
		// Initialize VTK.
		vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
		vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
		renWin->OffScreenRenderingOn(); //OFF-SCREEN RENDERING
		renWin->AddRenderer(renderer);
		renWin->SetSize(w, h); //HARDCODED 640X480 IMAGE

		//adding polydata actor
		vtkSmartPointer<vtkPolyDataMapper>	map = vtkSmartPointer<vtkPolyDataMapper>::New();
		map->SetInputData(pd);
		vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
		act->SetMapper(map);
		if (pRGB != NULL && !bShading)
		{
			act->GetProperty()->BackfaceCullingOn();
			act->GetProperty()->LightingOff();
			act->GetProperty()->ShadingOff();
			act->GetProperty()->SetInterpolationToFlat();
		}
		renderer->AddActor(act);

		double clipnear, clipfar;

		if (clipfarIn <= 0.0f)
		{
			//find zbounds
			double *bounds;
			pd->GetPoints()->ComputeBounds(); //just in case
			bounds = pd->GetPoints()->GetBounds(); // (Xmin, Xmax) = (bounds[0], bounds[1]), (Ymin, Ymax) = (bounds[2], bounds[3]), (Zmin, Zmax) = (bounds[4], bounds[5])
			clipnear = bounds[4];
			clipfar = bounds[5];
		}
		else
		{
			clipnear = clipnearIn;
			clipfar = clipfarIn;
		}

		//Generate and return Kinect-like depth image
		return GenerateVTKDepthImage_Kinect(renWin, scale, clipnear, clipfar, w, h, fy, cx, cy, R, t, bClipFarToZero, pRGB);
	}

	vtkSmartPointer<vtkCamera> CreateVTKCamera(int width, int height, double fx, double fy, double cx, double cy, double clipnear, double clipfar)
	{
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
		// convert the focal length to view angle and set it (ONLY FOR KINECT???)
		double view_angle = 57.2958 * (2.0 * atan2(height / 2.0, fy));
		camera->SetViewAngle(view_angle);//vertical 46,6 ???

		return camera;
	}

	//ECCV kinect camera
	vtkSmartPointer<vtkCamera> CreateVTKCamera_GenericKinect_1(double clipnear, double clipfar)
	{
		//generic
		int width = 640;
		int height = 480;
		double fx = 525;
		double fy = 525;
		double cx = 320;
		double cy = 240;
		double horizFOV = 58.5; //???hardware spec???
		double vertFOV = 46.6;  //???hardware spec???

		return CreateVTKCamera(width, height, fx, fy, cx, cy, clipnear, clipfar);
	}

	//Other kinect camera
	vtkSmartPointer<vtkCamera> CreateVTKCamera_GenericKinect_2(double clipnear, double clipfar)
	{
		//generic
		int width = 640;
		int height = 480;
		double fx = 581.45624912987f;
		double fy = 543.1221626989097f;
		double cx = 317.2825290065861f;
		double cy = 240.955527515504f;
		double horizFOV = 58.5; //???hardware spec???
		double vertFOV = 46.6;  //???hardware spec???

		return CreateVTKCamera(width, height, fx, fy, cx, cy, clipnear, clipfar);
	}
}
