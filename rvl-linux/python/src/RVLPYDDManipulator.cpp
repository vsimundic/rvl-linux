#include "RVLCore2.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "Util.h"
#include "SE3Grid.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "VN.h"
#include "RRT.h"
#include "DDManipulator.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

using namespace RVL;
using namespace py::literals;

////////////////////////////////////////////////////////////////////
//
//     PYDDManipulator
//
////////////////////////////////////////////////////////////////////

class PYDDManipulator
{
public:
	PYDDManipulator();
	~PYDDManipulator();
	void create(
		std::string cfgFileName);
	void set_memory_storage_size(
		int mem0Size,
		int memSize);
	void clear();
	py::tuple path2(py::array q_init);
	py::array approach_path(py::array T_G_S_contact);
	void load_tool_model(std::string toolModelDir);
	void set_environment_state(double state);
	void load_feasible_tool_contact_poses(std::string contactPosesFileName);
	void set_furniture_pose(py::array T_F_S);
	void set_robot_pose(py::array T_0_S);
	void set_door_model_params(
		float sx,
		float sy,
		float sz,
		float rx,
		float ry,
		float opening_direction,
		float static_side_width,
		float moving_to_static_part_distance);
	void set_door_pose(py::array T_A_S);
	py::array get_T_DD_S();
	py::array get_T_F_S();
	py::array get_T_DD_A();
	py::array fwd_kinematics(py::array q);
	py::tuple inv_kinematics(py::array T_G_0);

public:
	DDManipulator manipulator;
	int memSize;
	int mem0Size;
	Visualizer visualizer;
	MOTION::DisplayCallbackData* pVisualizationData;
};

PYDDManipulator::PYDDManipulator()
{
	memSize = 1000000000;
	mem0Size = 1000000000;
}

PYDDManipulator::~PYDDManipulator()
{
	clear();
}

void PYDDManipulator::create(
	std::string cfgFileName)
{
	manipulator.pMem0 = new CRVLMem;
	manipulator.pMem0->Create(mem0Size);
	manipulator.pMem = new CRVLMem;
	manipulator.pMem->Create(memSize);
	char *cfgFileName_ = (char *)(cfgFileName.data());
	manipulator.Create(cfgFileName_);
	visualizer.Create();
	manipulator.InitVisualizer(&visualizer);
}

void PYDDManipulator::set_memory_storage_size(
		int mem0SizeIn,
		int memSizeIn)
{
	memSize = memSizeIn;
	mem0Size = mem0SizeIn;
}

void PYDDManipulator::clear()
{
	delete manipulator.pMem0;
	delete manipulator.pMem;
}

py::tuple PYDDManipulator::path2(py::array q_init)
{
	double *q_init_ = (double *)q_init.request().ptr;
	float q_init__[6];
	for(int i = 0; i < manipulator.robot.n; i++)
		q_init__[i] = q_init_[i];
	// Pose3D pose_G_S_init;
	// RVLHTRANSFMXDECOMP(T_G_S_init_, pose_G_S_init.R, pose_G_S_init.t);
	// FILE *fpDebug = fopen("pose_G_S_init-2.txt", "w");
    // float T_G_S_init__[16];
    // RVLHTRANSFMX(pose_G_S_init.R, pose_G_S_init.t, T_G_S_init__);
    // PrintMatrix(fpDebug, T_G_S_init__, 4, 4);
    // fclose(fpDebug);
	Array<Pose3D> poses_G_0;
	Array2D<float> robotJoints;	
	if(!manipulator.Path2(q_init__, poses_G_0, robotJoints))
		poses_G_0.n = 1;
	auto T_G_0 = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		3,
		{poses_G_0.n, 4, 4},
		{4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}
		));
	float *T_G_0_ = (float *)T_G_0.request().ptr;
	auto q = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{poses_G_0.n, manipulator.robot.n},
		{manipulator.robot.n * sizeof(float), sizeof(float)}
		));
	float *q_ = (float *)q.request().ptr;
	if(poses_G_0.n > 1)
	{
		int iPose;
		Pose3D *pPose_G_0 = poses_G_0.Element;
		for(iPose = 0; iPose < poses_G_0.n; iPose++, T_G_0_ += 16, pPose_G_0++)
			RVLHTRANSFMX(pPose_G_0->R, pPose_G_0->t, T_G_0_);
		delete[] poses_G_0.Element;
		memcpy(q_, robotJoints.Element, poses_G_0.n * manipulator.robot.n * sizeof(float));
	}
	else
	{
		Pose3D null_pose;
		RVLUNITMX3(null_pose.R);
		RVLNULL3VECTOR(null_pose.t);
		RVLHTRANSFMX(null_pose.R, null_pose.t, T_G_0_);
		memset(q_, 0, manipulator.robot.n * sizeof(float));
	}

	py::tuple result_tuple = py::make_tuple(T_G_0, q);

	return result_tuple;	
}

py::array PYDDManipulator::approach_path(py::array T_G_S_contact)
{
	double *T_G_S_contact_ = (double *)T_G_S_contact.request().ptr;
	Pose3D pose_G_S_contact;
	RVLHTRANSFMXDECOMP(T_G_S_contact_, pose_G_S_contact.R, pose_G_S_contact.t);
	Array<Pose3D> poses_G_0_via;
	Pose3D viaPtPosesMem[2];
    poses_G_0_via.Element = viaPtPosesMem;
	float* SDF = new float[manipulator.pVNEnv->NodeArray.n];
	manipulator.ApproachPath(&pose_G_S_contact, poses_G_0_via, SDF);
	auto T_G_0_via = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		3,
		{poses_G_0_via.n, 4, 4},
		{4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}
		));
	float *T_G_0_via_ = (float *)T_G_0_via.request().ptr;
	int iPose;
	Pose3D *pPose_G_0 = poses_G_0_via.Element;
	for(iPose = 0; iPose < poses_G_0_via.n; iPose++, T_G_0_via_ += 16, pPose_G_0++)
		RVLHTRANSFMX(pPose_G_0->R, pPose_G_0->t, T_G_0_via_);

	return T_G_0_via;
}

void PYDDManipulator::load_tool_model(std::string toolModelDir)
{
	manipulator.LoadToolModel(toolModelDir);
}

void PYDDManipulator::set_environment_state(double state)
{
	manipulator.SetEnvironmentState(state);
	//printf("Environment state: %f\n", manipulator.dd_state_angle);
}

void PYDDManipulator::load_feasible_tool_contact_poses(std::string contactPosesFileName)
{
	manipulator.LoadFeasibleToolContactPoses(contactPosesFileName);
}

void PYDDManipulator::set_furniture_pose(py::array T_F_S)
{
	double *T_F_S_ = (double *)T_F_S.request().ptr;
	RVLHTRANSFMXDECOMP(T_F_S_, manipulator.pose_F_S.R, manipulator.pose_F_S.t);
	// printf("manipulator.pose_F_S:\n");
	// for(int i = 0; i < 3; i++)
	// {
	// 	for(int j = 0; j < 3; j++)
	// 		printf("%f ", manipulator.pose_F_S.R[j+3*i]);
	// 	printf("%f\n", manipulator.pose_F_S.t[i]);
	// }
}

void PYDDManipulator::set_robot_pose(py::array T_0_S)
{
	double *T_0_S_ = (double *)T_0_S.request().ptr;
	RVLHTRANSFMXDECOMP(T_0_S_, manipulator.robot.pose_0_W.R, manipulator.robot.pose_0_W.t);	
}

void PYDDManipulator::set_door_model_params(
	float sx,
	float sy,
	float sz,
	float rx,
	float ry,
	float opening_direction,
	float static_side_width,
	float moving_to_static_part_distance)
{
	manipulator.SetDoorModelParams(sx, sy, sz, rx, ry, opening_direction, static_side_width, moving_to_static_part_distance);
}

void PYDDManipulator::set_door_pose(py::array T_A_S)
{
	double *T_A_S_ = (double *)T_A_S.request().ptr;
	Pose3D pose_A_S;
	RVLHTRANSFMXDECOMP(T_A_S_, pose_A_S.R, pose_A_S.t);
	manipulator.SetDoorPose(pose_A_S);
}

py::array PYDDManipulator::get_T_DD_S()
{
	auto T_DD_S = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}
		));
	float *T_DD_S_ = (float *)T_DD_S.request().ptr;
	RVLHTRANSFMX(manipulator.pose_DD_S.R, manipulator.pose_DD_S.t, T_DD_S_);

	return T_DD_S;
}

py::array PYDDManipulator::get_T_F_S()
{
	auto T_F_S = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}
		));
	float *T_F_S_ = (float *)T_F_S.request().ptr;
	RVLHTRANSFMX(manipulator.pose_F_S.R, manipulator.pose_F_S.t, T_F_S_);

	return T_F_S;
}

py::array PYDDManipulator::get_T_DD_A()
{
	auto T_DD_A = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}
		));
	float *T_DD_A_ = (float *)T_DD_A.request().ptr;
	RVLHTRANSFMX(manipulator.pose_DD_A.R, manipulator.pose_DD_A.t, T_DD_A_);

	return T_DD_A;
}

py::array PYDDManipulator::fwd_kinematics(py::array q)
{
	double *q_ = (double *)q.request().ptr;
	int n = manipulator.robot.n;
	for(int i = 0; i < n; i++)
		manipulator.robot.q[i] = q_[i];
	manipulator.robot.FwdKinematics();	
	Pose3D *pPose_n_0 = manipulator.robot.link_pose + n - 1;
	Pose3D pose_G_0;
	RVLCOMPTRANSF3D(pPose_n_0->R, pPose_n_0->t, manipulator.robot.pose_TCP_6.R, manipulator.robot.pose_TCP_6.t, 
		pose_G_0.R, pose_G_0.t);
	auto T_G_0 = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}
		));
	float *T_G_0_ = (float *)T_G_0.request().ptr;
	RVLHTRANSFMX(pose_G_0.R, pose_G_0.t, T_G_0_);

	return T_G_0;
}

py::tuple PYDDManipulator::inv_kinematics(py::array T_G_0)
{
	double *T_G_0_ = (double *)T_G_0.request().ptr;
	Pose3D pose_G_0;
	RVLHTRANSFMXDECOMP(T_G_0_, pose_G_0.R, pose_G_0.t);
	// printf("pose_G_0:\n");
	// for(int i = 0; i < 3; i++)
	// {
	// 	for(int j = 0; j < 3; j++)
	// 		printf("%f ", pose_G_0.R[j+3*i]);
	// 	printf("%f\n", pose_G_0.t[i]);
	// }
	auto q = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		1,
		{manipulator.robot.n},
		{sizeof(float)}
		));
	float *q_ = (float *)q.request().ptr;
	bool bSuccess = manipulator.robot.InvKinematics(pose_G_0, q_);

	py::tuple result_tuple = py::make_tuple(q, bSuccess);

	return result_tuple;	
}

////////////////////////////////////////////////////////////////////
//
//     RVL PYDDManipulator Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYDDManipulator, m)
{
	m.doc() = "RVL PYDDManipulator wrapper";
	
	py::class_<PYDDManipulator>(m, "PYDDManipulator")
		.def(py::init<>())
		.def("create", &PYDDManipulator::create)
		.def("set_memory_storage_size", &PYDDManipulator::set_memory_storage_size)
		.def("clear", &PYDDManipulator::clear)
		.def("path2", &PYDDManipulator::path2)
		.def("approach_path", &PYDDManipulator::approach_path)
		.def("load_tool_model", &PYDDManipulator::load_tool_model)
		.def("set_environment_state", &PYDDManipulator::set_environment_state)
		.def("load_feasible_tool_contact_poses", &PYDDManipulator::load_feasible_tool_contact_poses)
		.def("set_furniture_pose", &PYDDManipulator::set_furniture_pose)
		.def("set_robot_pose", &PYDDManipulator::set_robot_pose)
		.def("set_door_model_params", &PYDDManipulator::set_door_model_params)
		.def("get_T_DD_S", &PYDDManipulator::get_T_DD_S)
		.def("get_T_F_S", &PYDDManipulator::get_T_F_S)
		.def("get_T_DD_A", &PYDDManipulator::get_T_DD_A)
		.def("set_door_pose", &PYDDManipulator::set_door_pose)
		.def("fwd_kinematics", &PYDDManipulator::fwd_kinematics)
		.def("inv_kinematics", &PYDDManipulator::inv_kinematics);
}
