#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "Voter.h"
#include "RVLBuffer.h"
#include "RVLPtrChain.h"
#include "RVLMPtrChain.h"
#include "Rect.h"
#include "RVLEDT.h"
#include "DDDetector.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

using namespace RVL;
using namespace py::literals;

////////////////////////////////////////////////////////////////////
//
//     PYDDDetector
//
////////////////////////////////////////////////////////////////////

class PYDDDetector
{
public:
	PYDDDetector();
	~PYDDDetector();
	void create(
		std::string cfgFileName);
	void set_memory_storage_size(
		int mem0Size,
		int memSize);
	void clear();
	void clear_mesh_sequence();
	void add_mesh(std::string meshFileName);
	void clear_rgb_sequence();
	void add_rgb(std::string RGBFileName);
	void set_mesh_triangle_edge_len(float maxMeshTriangleEdgeLenIn);
	void set_camera_parameters(
		float fu,
		float fv,
		float uc,
		float vc,
		int w,
		int h);
	py::dict detect();
	void set_hyp_file_name(std::string hypFileNameIn);
	void show_mesh(int iMesh);

	void load_best_hypothesis(std::string hypFileNameIn);
	py::array recognize_ao_state(py::array T_O_C, bool print_debug = false);

public:
	DDDetector detector;
	int memSize;
	int mem0Size;
	std::vector<Mesh *> meshSeq;
	std::vector<cv::Mat> RGBSeq;
	Camera camera;
	float maxMeshTriangleEdgeLen;
	Visualizer visualizer;
	RECOG::DDD::DisplayCallbackData* pVisualizationData;
	std::string hypFileName;
	Array<RECOG::DDD::HypothesisDoorDrawer> movingPartHyps;
	RECOG::DDD::ArticulatedObject AObj;
};

PYDDDetector::PYDDDetector()
{
	maxMeshTriangleEdgeLen = 0.050f;	// m
	memSize = 1000000000;
	mem0Size = 1000000000;
}

PYDDDetector::~PYDDDetector()
{
	clear();
}

void PYDDDetector::create(
	std::string cfgFileName)
{
	detector.pMem0 = new CRVLMem;
	detector.pMem0->Create(mem0Size);
	detector.pMem = new CRVLMem;
	detector.pMem->Create(memSize);
	char *cfgFileName_ = (char *)(cfgFileName.data());
	detector.Create(cfgFileName_);
	printf("DDD.loadMovingPartHypothesesFromFile=%d\n", (int)detector.bLoadMovingPartHypothesesFromFile);
	LoadCameraParametersFromFile(cfgFileName_, camera, detector.pMem0);
	detector.camera = camera;
	detector.pSurfels = new SurfelGraph;
	detector.pSurfels->pMem = detector.pMem;
	detector.pSurfels->CreateParamList(detector.pMem0);
	detector.pSurfels->ParamList.LoadParams(cfgFileName_);
	detector.pSurfelDetector = new PlanarSurfelDetector;
	detector.pSurfelDetector->CreateParamList(detector.pMem0);
	detector.pSurfelDetector->ParamList.LoadParams(cfgFileName_);
	visualizer.Create();
	detector.InitVisualizer(&visualizer);
	RVL_DELETE_ARRAY(detector.models.Element);
	detector.models.Element = new RECOG::DDD::Model[1];
	detector.models.n = 1;
	float cuboidSize[] = { 0.4f, 0.2f, 0.018f };
	detector.CreateCuboidModel2(cuboidSize, 1.0f / 0.02f, detector.models.Element);
	hypFileName = "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_open-20240215/sequence_images/cabinet_open2-20240215";

	movingPartHyps.Element = NULL;
	AObj.movingParts.Element = NULL;
}

void PYDDDetector::set_memory_storage_size(
		int mem0SizeIn,
		int memSizeIn)
{
	memSize = memSizeIn;
	mem0Size = mem0SizeIn;
}

void PYDDDetector::clear()
{
	clear_mesh_sequence();
	clear_rgb_sequence();
	delete detector.pMem0;
	delete detector.pMem;
	delete detector.pSurfels;
	delete detector.pSurfelDetector;
	RVL_DELETE_ARRAY(movingPartHyps.Element);
}

void PYDDDetector::clear_mesh_sequence()
{
	for(int iMesh = 0; iMesh < meshSeq.size(); iMesh++)
		delete meshSeq[iMesh];
	meshSeq.clear();
}

void PYDDDetector::add_mesh(std::string meshFileName)
{
	Mesh *pMesh = new Mesh;
	pMesh->LoadFromPLY((char *)(meshFileName.data()), maxMeshTriangleEdgeLen, true, &camera);
	meshSeq.push_back(pMesh);
}

void PYDDDetector::clear_rgb_sequence()
{
	RGBSeq.clear();
}

void PYDDDetector::add_rgb(std::string RGBFileName)
{
	RGBSeq.push_back(cv::imread(RGBFileName));
}

void PYDDDetector::set_mesh_triangle_edge_len(float maxMeshTriangleEdgeLenIn)
{
	maxMeshTriangleEdgeLen = maxMeshTriangleEdgeLenIn;
}

void PYDDDetector::set_camera_parameters(
		float fu,
		float fv,
		float uc,
		float vc,
		int w,
		int h)
{	
	camera.fu = fu;
	camera.fv = fv;
	camera.uc = uc;
	camera.vc = vc;
	camera.w = w;
	camera.h = h;
}

py::dict PYDDDetector::detect()
{
	Array<Mesh> meshSeq_;
	meshSeq_.n = meshSeq.size();
	uchar *meshSeqMem = (uchar *)malloc(meshSeq_.n * sizeof(Mesh));
	meshSeq_.Element = (Mesh *)meshSeqMem;
	for(int iMesh = 0; iMesh < meshSeq.size(); iMesh++)
		memcpy(meshSeq_.Element + iMesh, meshSeq[iMesh], sizeof(Mesh));
	char *hypFileName_ = (hypFileName.length() > 0 ? (char *)(hypFileName.data()): NULL);
	RECOG::DDD::HypothesisDoorDrawer DDObject;
	detector.Detect(meshSeq_, &DDObject, hypFileName_, (RGBSeq.size() == meshSeq_.n ? &RGBSeq : NULL));
	free(meshSeqMem);
	std::string objClass;
	switch(DDObject.objClass)
	{
		case RVLDDD_MODEL_DOOR:
			objClass = "door";
			break;
		case RVLDDD_MODEL_DRAWER:
			objClass = "drawer";
			break;
		default:
			objClass = "unknown";
	};
	auto R = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{3, 3},
		{3 * sizeof(float), sizeof(float)}
		));
	float *R_ = (float *)R.request().ptr;
	RVLCOPYMX3X3(DDObject.pose.R, R_);
	auto t = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		1,
		{3},
		{sizeof(float)}
		));
	float *t_ = (float *)t.request().ptr;
	RVLCOPY3VECTOR(DDObject.pose.t, t_);
	auto s = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		1,
		{2},
		{sizeof(float)}
		));
	float *s_ = (float *)s.request().ptr;
	s_[0] = DDObject.s[0]; s_[1] = DDObject.s[1];
	auto r = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		1,
		{2},
		{sizeof(float)}
		));
	float *r_ = (float *)r.request().ptr;
	r_[0] = DDObject.r[0]; r_[1] = DDObject.r[1];	
	py::dict ao("object_class"_a=objClass, "R"_a=R, "t"_a=t, "s"_a=s, "r"_a=r, "openingDirection"_a=DDObject.openingDirection);
	return ao;
}

void PYDDDetector::set_hyp_file_name(std::string hypFileNameIn)
{
	hypFileName = hypFileNameIn;
}

void PYDDDetector::show_mesh(int iMesh)
{
	Mesh *pMesh = meshSeq[iMesh];;
	detector.GetVisualizer()->SetMesh(pMesh);
	detector.RunVisualizer();
	detector.ClearVisualization();
}

void PYDDDetector::load_best_hypothesis(std::string hypFileNameIn)
{
	movingPartHyps.Element = new RECOG::DDD::HypothesisDoorDrawer[1];
	movingPartHyps.n = 1;
	FILE *fp = fopen(hypFileNameIn.data(), "r");
	if (fp)
	{
		detector.LoadDD(fp, movingPartHyps.Element);
		fclose(fp);
	}
	else
		printf("ERROR: Cannot open door/drawer hypothesis file %s!\n", hypFileName);
}

py::array PYDDDetector::recognize_ao_state(py::array T_O_C, bool print_debug)
{
	double *T_O_C_ = (double *)T_O_C.request().ptr;
	Pose3D pose_O_C;
	RVLHTRANSFMXDECOMP(T_O_C_, pose_O_C.R, pose_O_C.t);


	// Check if the hypotheses are loaded
	if (movingPartHyps.n == 0)
	{
		printf("ERROR: No hypotheses loaded!\n");
		return py::array();
	}
	// Check if meshes are loaded
	if (meshSeq.size() == 0)
	{
		printf("ERROR: No meshes loaded!\n");
		return py::array();
	}
	// Check if RGB images are loaded
	if (RGBSeq.size() != meshSeq.size())
	{
		printf("ERROR: Number of RGB images does not match number of meshes!\n");
		return py::array();
	}

	AObj.movingParts.Element = movingPartHyps.Element;
	AObj.movingParts.n = movingPartHyps.n;
	cv::Mat* pRGBImg;
	float hypStates[meshSeq.size()];
	for (int iScene = 0; iScene < meshSeq.size(); iScene++)
	{
		Mesh *pMesh = meshSeq[iScene];
		detector.SegmentPlanarSurfaces(pMesh);
		pRGBImg = RGBSeq.data() + iScene;
		detector.RecognizeArticulatedObjectState(pMesh, AObj, pose_O_C, (detector.GetRGBImageVisualization() ? pRGBImg : NULL), print_debug);

		hypStates[iScene] = AObj.movingParts.Element[0].state.Element[0].q;
	}
	auto hypStatesArray = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		1,
		{meshSeq.size()},
		{sizeof(float)}
	));
	float *hypStatesArray_ = (float *)hypStatesArray.request().ptr;
	memcpy(hypStatesArray_, hypStates, meshSeq.size() * sizeof(float));
	
	return hypStatesArray;
}

////////////////////////////////////////////////////////////////////
//
//     RVL PYDDDetector Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYDDDetector, m) 
{
	m.doc() = "RVL PYDDDetector wrapper";
	
	py::class_<PYDDDetector>(m, "PYDDDetector")
		.def(py::init<>())
		.def("create", &PYDDDetector::create)
		.def("set_memory_storage_size", &PYDDDetector::set_memory_storage_size)
		.def("clear", &PYDDDetector::clear)
		.def("clear_mesh_sequence", &PYDDDetector::clear_mesh_sequence)
		.def("add_mesh", &PYDDDetector::add_mesh)
		.def("clear_rgb_sequence", &PYDDDetector::clear_rgb_sequence)
		.def("add_rgb", &PYDDDetector::add_rgb)
		.def("set_mesh_triangle_edge_len", &PYDDDetector::set_mesh_triangle_edge_len)
		.def("set_camera_parameters", &PYDDDetector::set_camera_parameters)
		.def("detect", &PYDDDetector::detect)
		.def("set_hyp_file_name", &PYDDDetector::set_hyp_file_name)
		.def("show_mesh", &PYDDDetector::show_mesh)
		.def("load_best_hypothesis", &PYDDDetector::load_best_hypothesis)
		.def("recognize_ao_state", &PYDDDetector::recognize_ao_state);
}
