#pragma once
#include "vtkPolyDataMapper.h" 
#include "vtkProperty.h" 
#include "vtkActor.h" 
#include "vtkRenderWindow.h" 
#include "vtkRenderer.h" 
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyle.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkSmartPointer.h"
#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkPointData.h"
#include "vtkDataSetMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkMath.h"
#include "vtkFloatArray.h"
#include "vtkDoubleArray.h"
#include "vtkIdList.h"
#include "vtkPLYWriter.h"
#include "vtkActorCollection.h"
#include "vtkImageData.h"
#include "vtkBMPReader.h"
#include "vtkTexture.h"
#include "vtkPropPicker.h"
#include "vtkRendererCollection.h"
#include "vtkCallbackCommand.h"
#include "vtkPolygon.h"
#include "vtkTriangleFilter.h"
#include "vtkImageFlip.h"
#include "vtkTransform.h"
#include "vtkGlyph3D.h"
#include "vtkArrowSource.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"
#include "vtkPLYReader.h"
#include "vtkCornerAnnotation.h"
#include "vtkWindowToImageFilter.h"
#include "vtkPNGWriter.h"
#include "vtkConeSource.h"
#include "vtkOBJExporter.h"
#include "vtkPointPicker.h"
#include "vtkCellPicker.h"
#include "vtkActor.h"
#include "vtkTransformPolyDataFilter.h"
#include "VTKActorObj.h"
#include "vtkSphereSource.h"
#include "vtkCubeSource.h"
#include "vtkCylinderSource.h"
#include "vtkDistancePolyDataFilter.h"
#include "vtkImplicitPolyDataDistance.h"
#include "vtkCamera.h"
#include "vtkDelaunay3D.h"
#include "vtkCleanPolyData.h"
#include "vtkGeometryFilter.h"
#include "vtkAppendPolyData.h"
#include "vtkAlgorithm.h"
#include "vtkWindowToImageFilter.h"
#include "vtkHull.h"
#include "vtkPlanes.h"
#include "vtkPlaneSource.h"
#include "vtkPolyDataPointSampler.h"
#include "vtkDecimatePro.h"
#include "vtkOutlineSource.h"
#include "vtkSmoothPolyDataFilter.h"
#include "vtkWindowedSincPolyDataFilter.h"
#include "vtkFeatureEdges.h"
#include "vtkExtractEdges.h"
#include "vtkIterativeClosestPointTransform.h"
#include "vtkLandmarkTransform.h"
#include "vtkMatrix4x4.h"
#include "vtkVertexGlyphFilter.h"
#include "vtkOrientationMarkerWidget.h"
#include "vtkSimplePointsReader.h"
#include "vtkLabeledDataMapper.h"

#ifdef RVLLINUX
#include "opencv2/opencv.hpp"
#else
#include "opencv2\opencv.hpp"
#endif

namespace RVL
{
	//Test vtk PolyData distance class
	void testvtkdistance();

	void TestVTK_Plane_z_buffer(float distancefromZ, int width, int height, float fx, float fy, float cx, float cy, float clipnear, float clipfar);
	cv::Mat GenerateVTKDepthImage(vtkSmartPointer<vtkRenderWindow> renWin, int width, int height, double fx, double fy, double cx, double cy, double clipnear, double clipfar);
	cv::Mat GenerateVTKDepthImage(vtkSmartPointer<vtkRenderWindow> renWin, vtkSmartPointer<vtkCamera> camera, int width, int height);
	cv::Mat GenerateVTKDepthImage_Kinect(
		vtkSmartPointer<vtkRenderWindow> renWin,
		double scale,
		double clipnear,
		double clipfar,
		int w = 640,
		int h = 480,
		double f = 543.1221626989097f,
		double cx = 317.2825290065861f,
		double cy = 240.955527515504f,
		float *R = NULL,
		float *t = NULL,
		bool bClipFarToZero = true,
		uchar **pRGB = NULL);
	cv::Mat GenerateVTKPolyDataDepthImage_Kinect(
		vtkSmartPointer<vtkPolyData> pd,
		double scale = 1.0f,
		double clipnear = 0.0f,
		double clipfar = 0.0f,
		int w = 640,
		int h = 480,
		double f = 543.1221626989097f,
		double cx = 317.2825290065861f,
		double cy = 240.955527515504f,
		float *R = NULL,
		float *t = NULL,
		bool bClipFarToZero = true,
		uchar **pRGB = NULL,
		bool bShading = true);
	void GenerateVTKPolyDataRGBDImage(
		vtkSmartPointer<vtkPolyData> pd,
		cv::Mat &RGB,
		cv::Mat &depth,
		double scale = 1.0f,
		double clipnear = 0.0f,
		double clipfar = 0.0f,
		int w = 640,
		int h = 480,
		double f = 543.1221626989097f,
		double cx = 317.2825290065861f,
		double cy = 240.955527515504f,
		float *R = NULL,
		float *t = NULL,
		bool bClipFarToZero = true);
	vtkSmartPointer<vtkCamera> CreateVTKCamera(int width, int height, double fx, double fy, double cx, double cy, double clipnear, double clipfar);
	vtkSmartPointer<vtkCamera> CreateVTKCamera_GenericKinect_1(double clipnear, double clipfar);
	vtkSmartPointer<vtkCamera> CreateVTKCamera_GenericKinect_2(double clipnear, double clipfar);
}