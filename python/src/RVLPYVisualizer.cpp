#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLCore2.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#endif
#include "Util.h"
#include "RVL2DCellArray.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

using namespace RVL;

class RVLPYVisualizer
{
public:
	RVLPYVisualizer() 
	{
		
	}
	~RVLPYVisualizer() {};
	void create();
	void display_point_set(
		py::array point_array,
		unsigned char red,
		unsigned char green,
		unsigned char blue,
		float marker_size);
	void display_box(
		double a,
		double b,
		double c,
		py::array R,
		py::array t,
		double red,
		double green,
		double blue,
		bool bWireFrame,
		float line_width);
	void display_lines(
		py::array point_array,
		py::array line_array,
		unsigned char red,
		unsigned char green,
		unsigned char blue);
	void set_mesh_from_obj(char *file_name);
	void run();
	void clear_display();

private:
	CRVLMem mem;
	Visualizer visualizer;
};

void RVLPYVisualizer::create()
{
	visualizer.Create();
}

void RVLPYVisualizer::display_point_set(
	py::array point_array,
	unsigned char red,
	unsigned char green,
	unsigned char blue,
	float marker_size)
{
	const ssize_t *point_array_size = point_array.shape();
	int nPts = (int)point_array_size[0];
	double *point_mem = (double *)point_array.request().ptr;
	double *PSrc = point_mem;

	Array<Point> ptArray;
	ptArray.n = nPts;
	ptArray.Element = new Point[nPts];

	Point *pPtTgt = ptArray.Element;

	for (int iPt = 0; iPt < nPts; iPt++, PSrc += 3, pPtTgt++)
	{
		RVLCOPY3VECTOR(PSrc, pPtTgt->P);
	}

	unsigned char color[3];
	color[0] = red;
	color[1] = green;
	color[2] = blue;

	visualizer.DisplayPointSet<float, Point>(ptArray, color, marker_size);
	
	delete[] ptArray.Element;
}

void RVLPYVisualizer::set_mesh_from_obj(char *file_name)
{
	visualizer.SetMeshFromOBJ(file_name);
}

void RVLPYVisualizer::display_box(
	double a,
	double b,
	double c,
	py::array R,
	py::array t,
	double red,
	double green,
	double blue,
	bool bWireFrame,
	float line_width)
{
	double *R_ = (double *)R.request().ptr;
	double *t_ = (double *)t.request().ptr;

	Pose3D pose;
	RVLCOPYMX3X3(R_, pose.R);
	RVLCOPY3VECTOR(t_, pose.t);

	vtkSmartPointer<vtkActor> boxActor = visualizer.DisplayBox(a, b, c, &pose, red, green, blue, bWireFrame);
	boxActor->GetProperty()->SetLineWidth(line_width);
}

void RVLPYVisualizer::display_lines(
		py::array point_array,
		py::array line_array,
		unsigned char red,
		unsigned char green,
		unsigned char blue)
{
	const ssize_t *point_array_size = point_array.shape();
	int nPts = (int)point_array_size[0];
	double *point_mem = (double *)point_array.request().ptr;
	double *PSrc = point_mem;

	Array<Point> ptArray;
	ptArray.n = nPts;
	ptArray.Element = new Point[nPts];

	Point *pPtTgt = ptArray.Element;

	for (int iPt = 0; iPt < nPts; iPt++, PSrc += 3, pPtTgt++)
	{
		RVLCOPY3VECTOR(PSrc, pPtTgt->P);
	}

	const ssize_t *line_array_size = line_array.shape();
	int nLines = (int)line_array_size[0];
	int *line_mem = (int *)line_array.request().ptr;
	int *LSrc = line_mem;

	Array<Pair<int, int>> lineArray;
	lineArray.n = nLines;
	lineArray.Element = new Pair<int, int>[nLines];

	Pair<int, int> *pLTgt = lineArray.Element;

	for (int iLine = 0; iLine < nLines; iLine++, LSrc += 2, pLTgt++)
	{
		pLTgt->a = LSrc[0];
		pLTgt->b = LSrc[1];
	}
	
	unsigned char color[3];
	color[0] = red;
	color[1] = green;
	color[2] = blue;

	visualizer.DisplayLines(ptArray, lineArray, color);	

	delete[] ptArray.Element;
	delete[] lineArray.Element;
}

void RVLPYVisualizer::run()
{
	visualizer.Run();
}

void RVLPYVisualizer::clear_display()
{
	visualizer.renderer->RemoveAllViewProps();
}


////////////////////////////////////////////////////////////////////
//
//     RVL Visualizer Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYVisualizer, m) 
{
	m.doc() = "RVL Visualizer wrapper";

	py::class_<RVLPYVisualizer>(m, "Visualizer")
		.def(py::init<>())
		.def("create", &RVLPYVisualizer::create)
		.def("display_point_set", &RVLPYVisualizer::display_point_set)
		.def("set_mesh_from_obj", &RVLPYVisualizer::set_mesh_from_obj)
		.def("display_box", &RVLPYVisualizer::display_box)
		.def("display_lines", &RVLPYVisualizer::display_lines)
		.def("run", &RVLPYVisualizer::run)
		.def("clear_display", &RVLPYVisualizer::clear_display);
}

