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
#include "RVLMotionCommon.h"
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
	py::tuple path2(
		py::array q_init, 
		double endState,
		int nStates,
		bool bReturnAllFeasiblePaths = false);
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
	py::tuple inv_kinematics_all_sols(py::array T_G_0, bool bTCP);
	py::tuple inv_kinematics_all_sols_prev(py::array T_G_0);

public:
	DDManipulator manipulator;
	int memSize;
	int mem0Size;
	Visualizer visualizer;
	MOTION::DisplayCallbackData* pVisualizationData;
	Pose3D pose_A_S;
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
	// manipulator.bVNPanel = true;    // For TestFeasibleRobotPose (false for TCMCS24)
	char *cfgFileName_ = (char *)(cfgFileName.data());
	manipulator.Create(cfgFileName_);

    manipulator.robot.minq[1] = -PI;
    manipulator.robot.maxq[1] = 0.0f;
    manipulator.robot.minq[3] = -PI;
    manipulator.robot.maxq[3] = 0.0f;
	

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

py::tuple PYDDManipulator::path2(
	py::array q_init,
	double endState,
	int nStates,
	bool bReturnAllFeasiblePaths)
{
    // FCL - Simundic
    // manipulator.use_fcl = false;
    manipulator.LoadToolModelFCL();
    std::string cabinetStaticModelPath = "/home/RVLuser/ferit_ur5_ws/cabinet_static.ply";
    std::string cabinetPanelModelPath = "/home/RVLuser/ferit_ur5_ws/cabinet_panel.ply";
    // fcl::Transform3d T_A_S;
    // manipulator.RVLPose2FCLPose(pose_A_S, T_A_S);
	if (manipulator.use_fcl)
	{
		manipulator.LoadCabinetStaticFCL(cabinetStaticModelPath, pose_A_S);
		manipulator.LoadCabinetPanelFCL(cabinetPanelModelPath);
		manipulator.CreateRobotCylindersFCL();
		manipulator.CreateGndFCL();
        // RVLCOMPTRANSF3D(manipulator.pose_F_S.R, manipulator.pose_F_S.t, manipulator.pose_A_F.R, manipulator.pose_A_F.t, manipulator.pose_A_S_FCL.R, manipulator.pose_A_S_FCL.t);

        manipulator.CreateGndFCL();

	}
	
	manipulator.resultsFolder = "/home/RVLuser/rvl-linux";

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
	Array<Array<Pose3D>> allFeasiblePaths;
	Array<Array2D<float>> allFeasiblePathsJoints;
	Array<Array<Pose3D>> *pAllFeasiblePaths;
	Array<Array2D<float>> *pAllFeasiblePathsJoints;
	if(bReturnAllFeasiblePaths)
	{
		pAllFeasiblePaths = &allFeasiblePaths;
		pAllFeasiblePathsJoints = &allFeasiblePathsJoints;
	}
	else
	{
		pAllFeasiblePaths = NULL;
		pAllFeasiblePathsJoints = NULL;
	}
	if(!manipulator.Path2(q_init__, endState, nStates, poses_G_0, robotJoints, pAllFeasiblePaths, pAllFeasiblePathsJoints))
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
	py::tuple result_tuple;
	if(bReturnAllFeasiblePaths)
	{
		int maxnPathPoints = nStates + 3;
		auto allFeasiblePathsPy = py::array(py::buffer_info(
			nullptr,
			sizeof(float),
			py::format_descriptor<float>::value,
			4,
			{allFeasiblePaths.n, maxnPathPoints, 4, 4},
			{maxnPathPoints * 4 * 4 * sizeof(float), 4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}
			));
		float *allFeasiblePathsPy_ = (float *)allFeasiblePathsPy.request().ptr;
		auto allFeasiblePathsJointsPy = py::array(py::buffer_info(
			nullptr,
			sizeof(float),
			py::format_descriptor<float>::value,
			3,
			{allFeasiblePathsJoints.n, maxnPathPoints, manipulator.robot.n},
			{maxnPathPoints * manipulator.robot.n * sizeof(float), manipulator.robot.n * sizeof(float), sizeof(float)}
			));
		float *allFeasiblePathsJointsPy_ = (float *)allFeasiblePathsJointsPy.request().ptr;
		float *T_G_0__ = allFeasiblePathsPy_;
		float *q__ = allFeasiblePathsJointsPy_;
		memset(allFeasiblePathsPy_, 0, allFeasiblePaths.n * maxnPathPoints * 4 * 4 * sizeof(float));
		memset(allFeasiblePathsJointsPy_, 0, allFeasiblePathsJoints.n * maxnPathPoints * manipulator.robot.n * sizeof(float));
		for(int iPath = 0; iPath < allFeasiblePaths.n; iPath++, q__ += (maxnPathPoints * manipulator.robot.n))
		{
			Array<Pose3D> *pPath = allFeasiblePaths.Element + iPath;
			Pose3D *pPose_G_0 = pPath->Element;
			for(int iPose = 0; iPose < pPath->n; iPose++, pPose_G_0++, T_G_0__ += 16)
				RVLHTRANSFMX(pPose_G_0->R, pPose_G_0->t, T_G_0__);
			Array2D<float> *pPathJoints = allFeasiblePathsJoints.Element + iPath;
			memcpy(q__, pPathJoints->Element, pPathJoints->h * pPathJoints->w * sizeof(float));
		}
		result_tuple = py::make_tuple(T_G_0, q, allFeasiblePathsPy, allFeasiblePathsJointsPy);
	}
	else
		result_tuple = py::make_tuple(T_G_0, q);

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
	Array<MOTION::IKSolution> approachIKSolutions[3];
	Array<Pair<int, int>> approachPaths;

	manipulator.ApproachPath(&pose_G_S_contact, poses_G_0_via, SDF, approachIKSolutions, approachPaths);
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
	// Pose3D pose_A_S;
	RVLHTRANSFMXDECOMP(T_A_S_, pose_A_S.R, pose_A_S.t);
	manipulator.SetDoorPose(pose_A_S);

	// Simundic
	// manipulator.pose_A_S = pose_A_S;
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

py::tuple PYDDManipulator::inv_kinematics_all_sols(py::array T_G_0, bool bTCP)
{
	Array<MOTION::IKSolution> IKSolutions;
	int maxnIKSolutions = manipulator.maxnIKSolutions;
	IKSolutions.Element = new MOTION::IKSolution[maxnIKSolutions];

	double *T_G_0_ = (double *)T_G_0.request().ptr;
	Pose3D pose_G_0;
	RVLHTRANSFMXDECOMP(T_G_0_, pose_G_0.R, pose_G_0.t);

	// pose_G_0.R[0] = -0.997844219;
	// pose_G_0.R[1] = 0.0649242774;
	// pose_G_0.R[2] = 0.00957858562;
	// pose_G_0.R[3] = 0.0188735276;
	// pose_G_0.R[4] = 0.423683435;
	// pose_G_0.R[5] = -0.90561372;
	// pose_G_0.R[6] = -0.0628545955;
	// pose_G_0.R[7] = -0.903480589;
	// pose_G_0.R[8] = -0.423995405;

	// pose_G_0.t[0] = -0.248225123;
	// pose_G_0.t[1] = 0.00732335448;
	// pose_G_0.t[2] = 0.551909804;

	printf("pose_G_0:\n");
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
			printf("%f ", pose_G_0.R[j+3*i]);
		printf("%f\n", pose_G_0.t[i]);
	}
	auto q = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2, // num of dimensions
		{maxnIKSolutions, manipulator.robot.n},
		{manipulator.robot.n * sizeof(float), sizeof(float)}
		));
	float *q_ = (float *)q.request().ptr;

	for (int iLink = 0; iLink < 8; iLink++)
	{
		cout << manipulator.robot.link_pose[1].R[iLink] << endl;
	}

	bool bSuccess = manipulator.robot.InvKinematics(pose_G_0, IKSolutions, bTCP);

	for (int iSol = 0; iSol < IKSolutions.n; q_ += manipulator.robot.n, iSol++)
	{
		for (int iq = 0; iq < 6; iq++)
		{
			cout << IKSolutions.Element[iSol].q[iq] << " ";
		}
		cout << endl;
		memcpy(q_, IKSolutions.Element[iSol].q, manipulator.robot.n * sizeof(float));
	}

	delete[] IKSolutions.Element;

	py::tuple result_tuple = py::make_tuple(q, IKSolutions.n, bSuccess);

	return result_tuple;
}

py::tuple PYDDManipulator::inv_kinematics_all_sols_prev(py::array T_G_0)
{
	double *T_G_0_ = (double *)T_G_0.request().ptr;
	Pose3D pose_G_0;
	RVLHTRANSFMXDECOMP(T_G_0_, pose_G_0.R, pose_G_0.t);
	auto q = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{8, manipulator.robot.n},
		{manipulator.robot.n * sizeof(float), sizeof(float)}
		));
	float *q_ = (float *)q.request().ptr;

	RVL::Array2D<float> RVLinvKinSolutions;
	RVLinvKinSolutions.Element = NULL;

	bool bSuccess = manipulator.robot.InvKinematicsPrev(pose_G_0, RVLinvKinSolutions);

	for (int iSol = 0; iSol < RVLinvKinSolutions.h; q_ += manipulator.robot.n, iSol++)
	{
		memcpy(q_, RVLinvKinSolutions.Element + 6*iSol, manipulator.robot.n * sizeof(float));
	}

	py::tuple result_tuple = py::make_tuple(q, RVLinvKinSolutions.h, bSuccess);

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
		.def("inv_kinematics", &PYDDManipulator::inv_kinematics)
		.def("inv_kinematics_all_sols", &PYDDManipulator::inv_kinematics_all_sols)
		.def("inv_kinematics_all_sols_prev", &PYDDManipulator::inv_kinematics_all_sols_prev);
}
