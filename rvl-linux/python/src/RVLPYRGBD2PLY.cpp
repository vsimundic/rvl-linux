#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include "Util.h"
#include "Graph.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
#include "Mesh.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

using namespace RVL;

////////////////////////////////////////////////////////////////////
//
//     RGBD2PLY
//
////////////////////////////////////////////////////////////////////

class RGBD2PLY
{
public:
	RGBD2PLY()
	{

	}
	~RGBD2PLY() {};
	void png_to_ply(
		std::string RGBFileName,
		std::string depthFileName,
		float fu,
		float fv,
		float uc,
		float vc,
		int w,
		int h,
		float maxMeshTriangleEdgeLen,
		std::string PLYFileName);
	void pixel_array_to_ply(
		py::array rgb,
		py::array depth,
		float fu,
		float fv,
		float uc,
		float vc,
		int w,
		int h,
		float maxMeshTriangleEdgeLen,
		std::string PLYFileName);		
};

void RGBD2PLY::png_to_ply(
		std::string RGBFileName,
		std::string depthFileName,
		float fu,
		float fv,
		float uc,
		float vc,
		int w,
		int h,
		float maxMeshTriangleEdgeLen,
		std::string PLYFileName)
{
	Camera camera;
	camera.fu = fu;
	camera.fv = fv;
	camera.uc = uc;
	camera.vc = vc;
	camera.w = w;
	camera.h = h;
	MESH::RGBD2PLY(RGBFileName, depthFileName, camera, maxMeshTriangleEdgeLen, PLYFileName);
}

void RGBD2PLY::pixel_array_to_ply(
		py::array rgb,
		py::array depth,
		float fu,
		float fv,
		float uc,
		float vc,
		int w,
		int h,
		float maxMeshTriangleEdgeLen,
		std::string PLYFileName)
{
	Camera camera;
	camera.fu = fu;
	camera.fv = fv;
	camera.uc = uc;
	camera.vc = vc;
	camera.w = w;
	camera.h = h;

	char *rgbImageData = (char *)rgb.request().ptr;	

	Array2D<short> depthImage;
	depthImage.h = h;
	depthImage.w = w;
	depthImage.Element = (short *)depth.request().ptr;	

	Array2D<OrientedPoint> PC;
	MESH::OrientedPCFromDepthImage(depthImage, camera, PC);

	vtkSmartPointer<vtkPolyData> pPolygonData = MESH::VTKPolyDataFromOrganizedOrientedPC(PC, maxMeshTriangleEdgeLen);
	delete[] PC.Element;

	RVL::MESH::MapRGBImageToVTKPolyData(pPolygonData, rgbImageData, w, h);

	vtkSmartPointer<RVLVTKPLYWriter> pPLYWriter = vtkSmartPointer<RVLVTKPLYWriter>::New();
	pPLYWriter->WritePolyData(PLYFileName, pPolygonData);
}

////////////////////////////////////////////////////////////////////
//
//     RVL RGBD2PLY Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYRGBD2PLY, m) 
{
	m.doc() = "RVL RGBD2PLY wrapper";
	
	py::class_<RGBD2PLY>(m, "RGBD2PLY")
		.def(py::init<>())
		.def("png_to_ply", &RGBD2PLY::png_to_ply)
		.def("pixel_array_to_ply", &RGBD2PLY::pixel_array_to_ply);
}


