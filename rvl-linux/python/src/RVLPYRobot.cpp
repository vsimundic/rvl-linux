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
//     PYRobot
//
////////////////////////////////////////////////////////////////////

class PYRobot
{
public:
	PYRobot();
	~PYRobot();
	void create(
		std::string cfgFileName);

	void clear();
	py::array fwd_kinematics(py::array q);
	py::tuple inv_kinematics_single_sol(py::array T_G_0);
	py::tuple inv_kinematics_all_sols(py::array T_G_0, bool bTCP);
	void set_joint_limits(float minq, float maxq, int idx);
	void set_robot_pose(py::array T_0_S);

public:
	RVL::MOTION::Robot robot;
	int mem0Size;
	int maxnIKSolutions;
};

PYRobot::PYRobot()
{
	mem0Size = 1000000000;
}

PYRobot::~PYRobot()
{
    clear();
}

void PYRobot::clear()
{
	delete robot.pMem0;
}

void PYRobot::create(
	std::string cfgFileName)
{
	robot.pMem0 = new CRVLMem;
	robot.pMem0->Create(mem0Size);
	char *cfgFileName_ = (char *)(cfgFileName.data());
	robot.Create(cfgFileName_);

	maxnIKSolutions = 8;

    // robot.minq[1] = -PI;
    // robot.maxq[1] = 0.0f;
    // robot.minq[3] = -PI;
    // robot.maxq[3] = 0.0f;
}

void PYRobot::set_robot_pose(py::array T_0_S)
{
	double *T_0_S_ = (double *)T_0_S.request().ptr;
	RVLHTRANSFMXDECOMP(T_0_S_, robot.pose_0_W.R, robot.pose_0_W.t);	
}

py::array PYRobot::fwd_kinematics(py::array q)
{
	double *q_ = (double *)q.request().ptr;
	int n = robot.n;
	for(int i = 0; i < n; i++)
		robot.q[i] = q_[i];
	robot.FwdKinematics();	
	Pose3D *pPose_n_0 = robot.link_pose + n - 1;
	Pose3D pose_G_0;
	RVLCOMPTRANSF3D(pPose_n_0->R, pPose_n_0->t, robot.pose_TCP_6.R, robot.pose_TCP_6.t, 
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

py::tuple PYRobot::inv_kinematics_single_sol(py::array T_G_0)
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
		{robot.n},
		{sizeof(float)}
		));
	float *q_ = (float *)q.request().ptr;
	bool bSuccess = robot.InvKinematics(pose_G_0, q_);

	py::tuple result_tuple = py::make_tuple(q, bSuccess);

	return result_tuple;	
}

py::tuple PYRobot::inv_kinematics_all_sols(py::array T_G_0, bool bTCP)
{
	Array<MOTION::IKSolution> IKSolutions;
	IKSolutions.Element = new MOTION::IKSolution[maxnIKSolutions];

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
		2, // num of dimensions
		{maxnIKSolutions, robot.n},
		{robot.n * sizeof(float), sizeof(float)}
		));
	float *q_ = (float *)q.request().ptr;
	bool bSuccess = robot.InvKinematics(pose_G_0, IKSolutions, bTCP);
	
	cout << "[DEBUG] n sols:" << IKSolutions.n << endl;
	for (int iSol = 0; iSol < IKSolutions.n; q_ += robot.n, iSol++)
	{
		for (int iq = 0; iq < 6; iq++)
		{
			cout << IKSolutions.Element[iSol].q[iq] << " ";
		}
		cout << endl;
		memcpy(IKSolutions.Element[iSol].q, q_, robot.n * sizeof(float));
	}

	delete[] IKSolutions.Element;

	py::tuple result_tuple = py::make_tuple(q, bSuccess);

	return result_tuple;
}

void PYRobot::set_joint_limits(float minq, float maxq, int idx)
{
	robot.minq[idx] = minq;
	robot.maxq[idx] = maxq;
}


////////////////////////////////////////////////////////////////////
//
//     RVL PYRobot Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYRobot, m)
{
	m.doc() = "RVL PYRobot wrapper";
	
	py::class_<PYRobot>(m, "PYRobot")
		.def(py::init<>())
		.def("create", &PYRobot::create)
		.def("clear", &PYRobot::clear)
		.def("set_robot_pose", &PYRobot::set_robot_pose)
		.def("fwd_kinematics", &PYRobot::fwd_kinematics)
		.def("inv_kinematics_single_sol", &PYRobot::inv_kinematics_single_sol)
		.def("inv_kinematics_all_sols", &PYRobot::inv_kinematics_all_sols)
		.def("set_joint_limits", &PYRobot::set_joint_limits);
}
