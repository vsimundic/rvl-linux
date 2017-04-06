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
#include "VTKActor.h"
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

#include "opencv2\opencv.hpp"

namespace RVL
{
	//Test vtk PolyData distance class
	void testvtkdistance();

	void TestVTK_Plane_z_buffer(float distancefromZ, int width, int height, float fx, float fy, float cx, float cy, float horizFOV, float clipnear, float clipfar);

	cv::Mat GenerateVTKDepthImage(vtkSmartPointer<vtkRenderWindow> renWin, int width, int height, double fx, double fy, double cx, double cy, double horizFOV, double vertFOV, double clipnear, double clipfar);

	cv::Mat GenerateVTKDepthImage_Kinect(vtkSmartPointer<vtkRenderWindow> renWin, double clipnear, double clipfar);

	cv::Mat GenerateVTKPolyDataDepthImage_Kinect(vtkSmartPointer<vtkPolyData> pd);
}