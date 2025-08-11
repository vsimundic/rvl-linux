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
#include "Touch.h"
// #include "RVLPYTouch.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

using namespace RVL;
using namespace py::literals;

////////////////////////////////////////////////////////////////////
//
//     PYTouch
//
/////////////////////////////////////////////////////////////////////

class PYTouch
{
public:
	RVL::Touch touch;
	int memSize;
	int mem0Size;
	RVL::Visualizer visualizer;
	Box<float> bbox;
	int iScene = -1;

	MOTION::DoorExperimentParams doorExpParams;

	std::vector<MOTION::TouchData> touches;
	std::vector<RVL::MOTION::Contact> contacts;
	MOTION::TouchModel model_x, model_gt;

	PYTouch()
	{
		memSize = 1000000000;
		mem0Size = 1000000000;
	}

	~PYTouch()
	{
		clear();
	}

	void create(std::string cfgFileName)
	{
		touch.pMem0 = new CRVLMem;
		touch.pMem0->Create(mem0Size);

		char *cfgFileNameCStr = (char *)(cfgFileName.data());
		touch.Create(cfgFileNameCStr);

		visualizer.Create();
		touch.InitVisualizer(&visualizer, cfgFileNameCStr);
		touch.bDoor = true;

		touch.SetVisualizeOptimization(false);
	}

	void clear()
	{
		delete touch.pMem0;
	}

	void create_simple_tool(
		float a, float b, float c, float d, float h, py::array pose_tool_E)
	{
		double *pose_tool_E_ = (double *)pose_tool_E.request().ptr;
		Pose3D pPose_tool_E_;
		RVLHTRANSFMXDECOMP(pose_tool_E_, pPose_tool_E_.R, pPose_tool_E_.t);
		touch.CreateSimpleTool(a, b, c, d, h, &pPose_tool_E_);
	}

	void set_camera_params(
		float fu,
		float fv,
		float uc,
		float vc,
		int w,
		int h)
	{
		touch.camera.fu = fu;
		touch.camera.fv = fv;
		touch.camera.uc = uc;
		touch.camera.vc = vc;
		touch.camera.w = w;
		touch.camera.h = h;
	}

	void set_session_params(
		float sx, float sy, float sz, float rx, float ry, 
		float a, float b, float c, float qDeg, py::array T_C_E,
		
		float sxgt, float sygt, float szgt,float rxgt, float rygt, 
		float qDeg_gt, py::array T_A_0_gt)
	{
		doorExpParams.sx = sx;
		doorExpParams.sy = sy;
		doorExpParams.sz = sz;
		doorExpParams.rx = rx;
		doorExpParams.ry = ry;
		doorExpParams.a = a;
		doorExpParams.b = b;
		doorExpParams.c = c;
		doorExpParams.qDeg = qDeg;

		doorExpParams.sxgt = sxgt;
		doorExpParams.sygt = sygt;
		doorExpParams.szgt = szgt;
		doorExpParams.rxgt = rxgt;
		doorExpParams.rygt = rygt;
		doorExpParams.qDeg_gt = qDeg_gt;
		double *T_A_0_gt_ = (double *)T_A_0_gt.request().ptr;
		RVLHTRANSFMXDECOMP(T_A_0_gt_, doorExpParams.pose_A_0_gt.R, doorExpParams.pose_A_0_gt.t);

		double *T_C_E_ = (double *)T_C_E.request().ptr;
		RVLHTRANSFMXDECOMP(T_C_E_, doorExpParams.pose_C_E.R, doorExpParams.pose_C_E.t);
		
		touch.InitSession(&doorExpParams, true);
	}
	
	void set_new_scene(float sx, float sy, float sz,
					   float rx, float ry, float a, float b, float c, float qDeg,
					   py::array T_C_E, py::array T_A_C, py::array T_E_0,
					   float sxgt, float sygt, float szgt, float rxgt, float rygt, float qDeg_gt, py::array T_A_0_gt)
	{
		iScene++;

		doorExpParams.sx = sx;
		doorExpParams.sy = sy;
		doorExpParams.sz = sz;
		doorExpParams.rx = rx;
		doorExpParams.ry = ry;
		doorExpParams.a = a;
		doorExpParams.b = b;
		doorExpParams.c = c;
		doorExpParams.qDeg = qDeg;

		doorExpParams.sxgt = sxgt;
		doorExpParams.sygt = sygt;
		doorExpParams.szgt = szgt;
		doorExpParams.rxgt = rxgt;
		doorExpParams.rygt = rygt;
		doorExpParams.qDeg_gt = qDeg_gt;
		double *T_A_0_gt_ = (double *)T_A_0_gt.request().ptr;
		RVLHTRANSFMXDECOMP(T_A_0_gt_, doorExpParams.pose_A_0_gt.R, doorExpParams.pose_A_0_gt.t);

		double *T_C_E_ = (double *)T_C_E.request().ptr;
		RVLHTRANSFMXDECOMP(T_C_E_, doorExpParams.pose_C_E.R, doorExpParams.pose_C_E.t);
		double *T_A_C_ = (double *)T_A_C.request().ptr;
		RVLHTRANSFMXDECOMP(T_A_C_, doorExpParams.pose_A_C.R, doorExpParams.pose_A_C.t);
		double *T_E_0_ = (double *)T_E_0.request().ptr;
		RVLHTRANSFMXDECOMP(T_E_0_, doorExpParams.pose_E_0.R, doorExpParams.pose_E_0.t);

		if (iScene == 0)
			touch.InitSession(&doorExpParams, false);

		touch.InitScene(&doorExpParams);
	}

	void set_touch(py::array T_Ek_E, py::array V, float t, bool bMiss)
	{
		Pose3D pose_Ek_E;
		double *T_Ek_E_ = (double *)T_Ek_E.request().ptr;
		RVLHTRANSFMXDECOMP(T_Ek_E_, pose_Ek_E.R, pose_Ek_E.t);
		double *V_ = (double *)V.request().ptr;
		float V_f[3] = {(float)V_[0], (float)V_[1], (float)V_[2]};
		MOTION::TouchData touchData;
		RVLCOPYMX3X3(pose_Ek_E.R, touchData.pose.R);
		RVLCOPY3VECTOR(pose_Ek_E.t, touchData.pose.t);
		RVLCOPY3VECTOR(V_f, touchData.V);
		touchData.bMiss = bMiss;
		touchData.iFirstContact = -1;
		touchData.t = t;
		touchData.pEnvSolidParams = touch.envSolidParams_;

		touches.push_back(touchData);
	}

	void reset_touches()
	{
		touches.clear();
	}

	void correct()
	{
		touch.TestCorrection3(&doorExpParams, contacts, touches);
	}

	void set_visualization(bool bVisualization_)
	{
		touch.bVisualization = bVisualization_;
	}
};


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
	py::tuple approach_path(py::array T_G_S_contact);
	py::tuple approach_path_poses(py::array T_G_S_contact);
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
	py::array fwd_kinematics_6(py::array q);
	py::tuple inv_kinematics(py::array T_G_0);
	py::tuple inv_kinematics_all_sols(py::array T_G_0, bool bTCP);
	py::tuple inv_kinematics_all_sols_prev(py::array T_G_0);
	bool free(py::array q);
	py::bool_ free_tool_only(py::array T_G_S);
	py::array get_T_G_6();
	void visualize_current_state(py::array q, py::array T_G_R);
	void visualize_vn_current_state(py::array q, py::array T_G_R);
	void load_cabinet_static_mesh_fcl(std::string cabinetStaticMeshPath);
	void load_cabinet_panel_mesh_fcl(std::string cabinetPanelMeshPath);
	void set_pose_DD_S(py::array T_DD_S);
	void update_model_x();
	void set_environment_from_touch();
	void set_environment_from_touch_gt();
	void visualize_vn_model();
	void reset_vn_environment(py::array T_D_S);

	py::array get_corrected_cabinet_pose();
	py::array get_corrected_camera_pose();
	py::array get_corrected_pose_D_Arot();
	py::array get_corrected_pose_D_0();

public:
	DDManipulator manipulator;

	PYTouch py_touch;
	MOTION::TouchModel model_x, model_gt;

	int memSize;
	int mem0Size;
	Pair<int, int> *approachPathMem;
	Visualizer visualizer;
	MOTION::DisplayCallbackData *pVisualizationData;
	Pose3D pose_A_S;
	int maxnIKSolutions;
	float a, b, c;

	RVL::VN *pVNEnv_;
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

	maxnIKSolutions = manipulator.maxnIKSolutions;
	approachPathMem = new Pair<int, int>[maxnIKSolutions * maxnIKSolutions];

	if (manipulator.use_fcl)
	{
		manipulator.LoadToolModelFCL();
		manipulator.CreateRobotCylindersFCL();
		manipulator.CreateGndFCL();
	}

	pVNEnv_ = manipulator.pVNEnv;
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
	RVL_DELETE_ARRAY(approachPathMem);
	manipulator.pVNEnv = pVNEnv_;
	manipulator.Clear();
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

		// manipulator.CreateGndFCL();
	}

	manipulator.resultsFolder = "/home/RVLuser/rvl-linux";

	double *q_init_ = (double *)q_init.request().ptr;
	float q_init__[6];
	for (int i = 0; i < manipulator.robot.n; i++)
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
	if (bReturnAllFeasiblePaths)
	{
		pAllFeasiblePaths = &allFeasiblePaths;
		pAllFeasiblePathsJoints = &allFeasiblePathsJoints;
	}
	else
	{
		pAllFeasiblePaths = NULL;
		pAllFeasiblePathsJoints = NULL;
	}
	if (!manipulator.Path2(q_init__, endState, nStates, poses_G_0, robotJoints, pAllFeasiblePaths, pAllFeasiblePathsJoints))
		poses_G_0.n = 1;
	auto T_G_0 = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		3,
		{poses_G_0.n, 4, 4},
		{4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}));
	float *T_G_0_ = (float *)T_G_0.request().ptr;
	auto q = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{poses_G_0.n, manipulator.robot.n},
		{manipulator.robot.n * sizeof(float), sizeof(float)}));
	float *q_ = (float *)q.request().ptr;
	if (poses_G_0.n > 1)
	{
		int iPose;
		Pose3D *pPose_G_0 = poses_G_0.Element;
		for (iPose = 0; iPose < poses_G_0.n; iPose++, T_G_0_ += 16, pPose_G_0++)
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
	if (bReturnAllFeasiblePaths)
	{
		int maxnPathPoints = nStates + 3;
		auto allFeasiblePathsPy = py::array(py::buffer_info(
			nullptr,
			sizeof(float),
			py::format_descriptor<float>::value,
			4,
			{allFeasiblePaths.n, maxnPathPoints, 4, 4},
			{maxnPathPoints * 4 * 4 * sizeof(float), 4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}));
		float *allFeasiblePathsPy_ = (float *)allFeasiblePathsPy.request().ptr;
		auto allFeasiblePathsJointsPy = py::array(py::buffer_info(
			nullptr,
			sizeof(float),
			py::format_descriptor<float>::value,
			3,
			{allFeasiblePathsJoints.n, maxnPathPoints, manipulator.robot.n},
			{maxnPathPoints * manipulator.robot.n * sizeof(float), manipulator.robot.n * sizeof(float), sizeof(float)}));
		float *allFeasiblePathsJointsPy_ = (float *)allFeasiblePathsJointsPy.request().ptr;
		float *T_G_0__ = allFeasiblePathsPy_;
		float *q__ = allFeasiblePathsJointsPy_;
		memset(allFeasiblePathsPy_, 0, allFeasiblePaths.n * maxnPathPoints * 4 * 4 * sizeof(float));
		memset(allFeasiblePathsJointsPy_, 0, allFeasiblePathsJoints.n * maxnPathPoints * manipulator.robot.n * sizeof(float));
		for (int iPath = 0; iPath < allFeasiblePaths.n; iPath++, q__ += (maxnPathPoints * manipulator.robot.n))
		{
			Array<Pose3D> *pPath = allFeasiblePaths.Element + iPath;
			Pose3D *pPose_G_0 = pPath->Element;
			for (int iPose = 0; iPose < pPath->n; iPose++, pPose_G_0++, T_G_0__ += 16)
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

py::tuple PYDDManipulator::approach_path(py::array T_G_S_contact)
{
	// std::cout << "[DEBUG] Starting approach_path..." << std::endl;

	double *T_G_S_contact_ = (double *)T_G_S_contact.request().ptr;
	Pose3D pose_G_S_contact;
	RVLHTRANSFMXDECOMP(T_G_S_contact_, pose_G_S_contact.R, pose_G_S_contact.t);
	// std::cout << "[DEBUG] Decomposed input pose." << std::endl;

	Array<Pose3D> poses_G_0_via;
	Pose3D viaPtPosesMem[2];
	poses_G_0_via.Element = viaPtPosesMem;

	float *SDF = new float[manipulator.pVNEnv->NodeArray.n];
	// std::cout << "[DEBUG] Allocated SDF memory." << std::endl;

	Array<MOTION::IKSolution> approachIKSolutions[3];
	MOTION::IKSolution ikSolMem0[8];
	MOTION::IKSolution ikSolMem1[8];
	MOTION::IKSolution ikSolMem2[8];
	approachIKSolutions[0].Element = ikSolMem0;
	approachIKSolutions[1].Element = ikSolMem1;
	approachIKSolutions[2].Element = ikSolMem2;

	Array<Pair<int, int>> approachPaths;
	approachPaths.Element = approachPathMem;

	py::array T_G_0_via;
	py::list ik_solutions_list;
	py::list paths_list;

	if (manipulator.Free(&pose_G_S_contact, SDF))
	{
		Pose3D pose_G_0;
		float V3Tmp[3];
		RVLCOMPTRANSF3DWITHINV(manipulator.robot.pose_0_W.R, manipulator.robot.pose_0_W.t, pose_G_S_contact.R, pose_G_S_contact.t, pose_G_0.R, pose_G_0.t, V3Tmp);
		// std::cout << "[DEBUG] Computed pose_G_0." << std::endl;

		manipulator.robot.InvKinematics(pose_G_0, approachIKSolutions[2], true);
		// std::cout << "[DEBUG] Computed IK for pose_G_0, solutions found: " << approachIKSolutions[2].n << std::endl;

		if (approachIKSolutions[2].n != 0)
		{
			// std::cout << "[DEBUG] Starting ApproachPath call..." << std::endl;
			bool bSuccess = manipulator.ApproachPath(
				&pose_G_S_contact,
				poses_G_0_via,
				SDF,
				approachIKSolutions,
				approachPaths);

			// std::cout << "[DEBUG] ApproachPath success: " << bSuccess << ", via points: " << poses_G_0_via.n << std::endl;

			if (bSuccess && poses_G_0_via.n > 0)
			{
				T_G_0_via = py::array(py::buffer_info(
					nullptr,
					sizeof(float),
					py::format_descriptor<float>::value,
					3,
					{poses_G_0_via.n, 4, 4},
					{4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}));
				// std::cout << "[DEBUG] Allocated T_G_0_via numpy array." << std::endl;

				float *T_G_0_via_ = (float *)T_G_0_via.request().ptr;
				Pose3D *pPose_G_0 = poses_G_0_via.Element;
				for (int iPose = 0; iPose < poses_G_0_via.n; iPose++, T_G_0_via_ += 16, pPose_G_0++)
				{
					RVLHTRANSFMX(pPose_G_0->R, pPose_G_0->t, T_G_0_via_);
				}
				// std::cout << "[DEBUG] Filled T_G_0_via with poses." << std::endl;

				// IK solutions as list of numpy arrays
				for (int i = 0; i < 3; ++i)
				{
					int n_sols = approachIKSolutions[i].n;
					py::array_t<float> ik_array({n_sols, 6});
					auto ik_buf = ik_array.mutable_unchecked<2>();
					for (int j = 0; j < n_sols; ++j)
						for (int k = 0; k < 6; ++k)
							ik_buf(j, k) = approachIKSolutions[i].Element[j].q[k];
					ik_solutions_list.append(ik_array);
				}
				// std::cout << "[DEBUG] Converted IK solutions to numpy arrays." << std::endl;

				// Path indices as list of [a, b]
				for (int i = 0; i < approachPaths.n; ++i)
				{
					py::list pair;
					pair.append(approachPaths.Element[i].a);
					pair.append(approachPaths.Element[i].b);
					paths_list.append(pair);
				}
				// std::cout << "[DEBUG] Filled path index list." << std::endl;
			}
			else
			{
				// std::cout << "[DEBUG] ApproachPath returned false or no via points." << std::endl;
				T_G_0_via = py::array_t<float>({0, 4, 4});
			}
		}
		else
		{
			// std::cout << "[DEBUG] No IK solutions for pose_G_0, returning empty result." << std::endl;
			manipulator.last_approach_error_code = manipulator.APPROACH_NO_IK_FOR_CONTACT;
			T_G_0_via = py::array_t<float>({0, 4, 4});
		}
	}
	else
	{
		manipulator.last_approach_error_code = manipulator.APPROACH_CONTACT_SPHERE_COLLISION;
		T_G_0_via = py::array_t<float>({0, 4, 4});
	}

	delete[] SDF;
	// std::cout << "[DEBUG] Cleaned up and returning." << std::endl;
	return py::make_tuple(T_G_0_via, ik_solutions_list, paths_list, manipulator.last_approach_error_code);
}


py::tuple PYDDManipulator::approach_path_poses(py::array T_G_S_contact)
{
	double *T_G_S_contact_ = (double *)T_G_S_contact.request().ptr;
	Pose3D pose_G_S_contact;
	RVLHTRANSFMXDECOMP(T_G_S_contact_, pose_G_S_contact.R, pose_G_S_contact.t);

	Pose3D pPose_G_0;
	float V3Tmp[3];
	RVLCOMPTRANSF3DWITHINV(manipulator.robot.pose_0_W.R, manipulator.robot.pose_0_W.t, pose_G_S_contact.R, pose_G_S_contact.t, pPose_G_0.R, pPose_G_0.t, V3Tmp);
	float dummy_q[6] = {0., -PI * 0.5, 0., -PI * 0.5, 0., 0.};
	manipulator.VisualizeCurrentState(dummy_q, pPose_G_0);

	Array<Pose3D> poses_G_0_via;
	Pose3D viaPtPosesMem[2];
	poses_G_0_via.Element = viaPtPosesMem;

	py::array T_G_0_via;
	float *SDF = new float[manipulator.pVNEnv->NodeArray.n];

	bool bSuccess = manipulator.ApproachPathPoses(&pose_G_S_contact, poses_G_0_via, SDF);

	if (bSuccess && poses_G_0_via.n > 0)
	{
		// std::cout << "[DEBUG] Approach path number of pts: " << poses_G_0_via.n << std::endl;
		T_G_0_via = py::array(py::buffer_info(
			nullptr,
			sizeof(float),
			py::format_descriptor<float>::value,
			3,
			{poses_G_0_via.n, 4, 4},
			{4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}));

		float *T_G_0_via_ = (float *)T_G_0_via.request().ptr;
		Pose3D *pPose_G_0 = poses_G_0_via.Element;
		for (int iPose = 0; iPose < poses_G_0_via.n; iPose++, T_G_0_via_ += 16, pPose_G_0++)
		{
			RVLHTRANSFMX(pPose_G_0->R, pPose_G_0->t, T_G_0_via_);
		}
	}
	else
	{
		// std::cout << "[DEBUG] Approach path computation failed or returned no points." << std::endl;
		T_G_0_via = py::array(py::buffer_info(
			nullptr,
			sizeof(float),
			py::format_descriptor<float>::value,
			3,
			{0, 4, 4},
			{4 * 4 * sizeof(float), 4 * sizeof(float), sizeof(float)}));
		poses_G_0_via.n = 0;
	}

	delete[] SDF;
	return py::make_tuple(T_G_0_via, poses_G_0_via.n);
}

void PYDDManipulator::load_tool_model(std::string toolModelDir)
{
	manipulator.LoadToolModel(toolModelDir);
}

void PYDDManipulator::set_environment_state(double state)
{
	manipulator.SetEnvironmentState(state);
	// Visualization purpose

	float *P_F = new float[3 * 8];
	float *P_F_ = P_F;
	RVLSET3VECTOR(P_F_, 0.0f, 0.0f, a);
	P_F_ += 3;
	RVLSET3VECTOR(P_F_, 0.0f, 0.0f, -b);
	P_F_ += 3;
	RVLSET3VECTOR(P_F_, b, 0.0f, -b);
	P_F_ += 3;
	RVLSET3VECTOR(P_F_, b, 0.0f, a);
	P_F_ += 3;
	float *P_F__ = P_F;
	for (int i = 0; i < 4; i++, P_F_ += 3, P_F__ += 3)
	{
		P_F_[0] = P_F__[0];
		P_F_[1] = c;
		P_F_[2] = P_F__[2];
	}
	float P_S[3];
	P_F_ = P_F;
	for (int i = 0; i < 8; i++, P_F_ += 3)
	{
		RVLTRANSF3(P_F_, manipulator.pose_F_S.R, manipulator.pose_F_S.t, P_S);
		if (i == 0)
			InitBoundingBox<float>(&manipulator.vnbbox, P_S);
		else
			UpdateBoundingBox<float>(&manipulator.vnbbox, P_S);
	}
	float resolution = 0.02f * BoxSize(&manipulator.vnbbox);
	ExpandBox<float>(&manipulator.vnbbox, 10.0f * resolution);
	// pVisualizationData->VNBBox = manipulator.vnbbox;
	manipulator.pVNEnv->BoundingBox(manipulator.dVNEnv, manipulator.vnbbox);

	// //printf("Environment state: %f\n", manipulator.dd_state_angle);
	manipulator.setPose_DD_0();
	manipulator.FreeSpacePlanes();
	// manipulator.UpdateFurniturePose();
}

void PYDDManipulator::load_feasible_tool_contact_poses(std::string contactPosesFileName)
{
	manipulator.LoadFeasibleToolContactPoses(contactPosesFileName);
}

void PYDDManipulator::set_furniture_pose(py::array T_F_S)
{
	double *T_F_S_ = (double *)T_F_S.request().ptr;
	RVLHTRANSFMXDECOMP(T_F_S_, manipulator.pose_F_S.R, manipulator.pose_F_S.t);
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
	// manipulator.pVNEnv = manipulator.pVNEnv_tmp;
	// RVL_DELETE_ARRAY(manipulator.dVNEnv);
	// manipulator.dVNEnv = new float[manipulator.pVNEnv->featureArray.n];
	// manipulator.pVNEnv->CopyDescriptor(manipulator.dVNEnv);

	manipulator.SetDoorModelParams(sx, sy, sz, rx, ry, opening_direction, static_side_width, moving_to_static_part_distance);
	manipulator.pVNEnv->BoundingBox(manipulator.dVNEnv, manipulator.vnbbox);
	BoxSize<float>(&manipulator.vnbbox, a, b, c);
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
		{4 * sizeof(float), sizeof(float)}));
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
		{4 * sizeof(float), sizeof(float)}));
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
		{4 * sizeof(float), sizeof(float)}));
	float *T_DD_A_ = (float *)T_DD_A.request().ptr;
	RVLHTRANSFMX(manipulator.pose_DD_A.R, manipulator.pose_DD_A.t, T_DD_A_);

	return T_DD_A;
}

py::array PYDDManipulator::fwd_kinematics(py::array q)
{
	double *q_ = (double *)q.request().ptr;
	int n = manipulator.robot.n;
	for (int i = 0; i < n; i++)
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
		{4 * sizeof(float), sizeof(float)}));
	float *T_G_0_ = (float *)T_G_0.request().ptr;
	RVLHTRANSFMX(pose_G_0.R, pose_G_0.t, T_G_0_);

	return T_G_0;
}

py::array PYDDManipulator::fwd_kinematics_6(py::array q)
{
	double *q_ = (double *)q.request().ptr;
	int n = manipulator.robot.n;
	for (int i = 0; i < n; i++)
		manipulator.robot.q[i] = q_[i];
	manipulator.robot.FwdKinematics();
	Pose3D *pPose_n_0 = manipulator.robot.link_pose + n - 1;
	// Pose3D pose_6_0;
	// RVLCOMPTRANSF3D(pPose_n_0->R, pPose_n_0->t, manipulator.robot.pose_TCP_6.R, manipulator.robot.pose_TCP_6.t,
	// pose_G_0.R, pose_G_0.t);
	auto T_6_0 = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}));
	float *T_6_0_ = (float *)T_6_0.request().ptr;
	RVLHTRANSFMX(pPose_n_0->R, pPose_n_0->t, T_6_0_);

	return T_6_0;
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
		{sizeof(float)}));
	float *q_ = (float *)q.request().ptr;
	bool bSuccess = manipulator.robot.InvKinematics(pose_G_0, q_);

	py::tuple result_tuple = py::make_tuple(q, bSuccess);

	return result_tuple;
}

py::tuple PYDDManipulator::inv_kinematics_all_sols(py::array T_G_0, bool bTCP)
{
	Array<MOTION::IKSolution> IKSolutions;
	IKSolutions.Element = new MOTION::IKSolution[maxnIKSolutions];

	double *T_G_0_ = (double *)T_G_0.request().ptr;
	Pose3D pose_G_0;
	RVLHTRANSFMXDECOMP(T_G_0_, pose_G_0.R, pose_G_0.t);
	auto q = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2, // num of dimensions
		{maxnIKSolutions, manipulator.robot.n},
		{manipulator.robot.n * sizeof(float), sizeof(float)}));
	float *q_ = (float *)q.request().ptr;

	// for (int iLink = 0; iLink < 8; iLink++)
	// {
	// 	cout << manipulator.robot.link_pose[1].R[iLink] << endl;
	// }

	bool bSuccess = manipulator.robot.InvKinematics(pose_G_0, IKSolutions, bTCP);

	for (int iSol = 0; iSol < IKSolutions.n; q_ += manipulator.robot.n, iSol++)
	{
		// for (int iq = 0; iq < 6; iq++)
		// {
		// 	cout << IKSolutions.Element[iSol].q[iq] << " ";
		// }
		// cout << endl;
		memcpy(q_, IKSolutions.Element[iSol].q, manipulator.robot.n * sizeof(float));
	}

	delete[] IKSolutions.Element;

	py::tuple result_tuple = py::make_tuple(q, IKSolutions.n, bSuccess);

	return result_tuple;
}

py::tuple PYDDManipulator::inv_kinematics_all_sols_prev(py::array T_6_0)
{
	double *T_6_0_ = (double *)T_6_0.request().ptr;
	Pose3D pose_6_0;
	RVLHTRANSFMXDECOMP(T_6_0_, pose_6_0.R, pose_6_0.t);
	auto q = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{8, manipulator.robot.n},
		{manipulator.robot.n * sizeof(float), sizeof(float)}));
	float *q_ = (float *)q.request().ptr;

	RVL::Array2D<float> RVLinvKinSolutions;
	RVLinvKinSolutions.Element = NULL;

	bool bSuccess = manipulator.robot.InvKinematicsPrev(pose_6_0, RVLinvKinSolutions);
	Pose3D pose_6_0_debug;
	for (int iSol = 0; iSol < RVLinvKinSolutions.h; q_ += manipulator.robot.n, iSol++)
	{
		memcpy(q_, RVLinvKinSolutions.Element + 6 * iSol, manipulator.robot.n * sizeof(float));
		// Debug
		// manipulator.robot.FwdKinematics(RVLinvKinSolutions.Element + 6*iSol, &pose_6_0_debug);
	}

	py::tuple result_tuple = py::make_tuple(q, RVLinvKinSolutions.h, bSuccess);

	return result_tuple;
}

// Collision checking
bool PYDDManipulator::free(py::array q)
{
	float *q_ = (float *)q.request().ptr;
	return manipulator.Free(q_);
}

py::bool_ PYDDManipulator::free_tool_only(py::array T_G_S)
{
	double *T_G_S_ptr = (double *)T_G_S.request().ptr;
	Pose3D pose_G_S;
	RVLHTRANSFMXDECOMP(T_G_S_ptr, pose_G_S.R, pose_G_S.t);

	float *SDF = new float[manipulator.pVNEnv->NodeArray.n];
	bool result = manipulator.Free(&pose_G_S, SDF);
	delete[] SDF;

	return py::bool_(result);
}

py::array PYDDManipulator::get_T_G_6()
{
	auto T = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}));
	float *T_ = (float *)T.request().ptr;

	RVLHTRANSFMX(manipulator.robot.pose_TCP_6.R, manipulator.robot.pose_TCP_6.t, T_);

	return T;
}

void PYDDManipulator::visualize_current_state(py::array q, py::array T_G_R)
{
	float *q_ = (float *)q.request().ptr;
	double *T_G_R_ = (double *)T_G_R.request().ptr;
	Pose3D pose_G_R;
	RVLHTRANSFMXDECOMP(T_G_R_, pose_G_R.R, pose_G_R.t);
	// manipulator.SetVisualizeVNEnvironmentModel();
	manipulator.VisualizeCurrentState(q_, pose_G_R);
}

void PYDDManipulator::visualize_vn_current_state(py::array q, py::array T_G_R)
{
	float *q_ = (float *)q.request().ptr;
	double *T_G_R_ = (double *)T_G_R.request().ptr;
	Pose3D pose_G_R;
	RVLHTRANSFMXDECOMP(T_G_R_, pose_G_R.R, pose_G_R.t);

	// visualizer.Clear();
	// vtkSmartPointer<vtkActor> vnActor;
	// vnActor = manipulator.pVNEnv->Display(&visualizer, 0.01f, NULL, NULL, 0.0f, &py_touch.bbox);

	manipulator.VisualizeVNCurrentState(q_, pose_G_R, manipulator.vnbbox);

	// manipulator.VisualizeTool(pose_G_R, &(manipulator.pVisualizationData->robotActors));
}

void PYDDManipulator::load_cabinet_static_mesh_fcl(std::string cabinetStaticMeshPath)
{
	if (manipulator.use_fcl)
		manipulator.LoadCabinetStaticFCL(cabinetStaticMeshPath, pose_A_S);
}

void PYDDManipulator::load_cabinet_panel_mesh_fcl(std::string cabinetPanelMeshPath)
{
	if (manipulator.use_fcl)
		manipulator.LoadCabinetPanelFCL(cabinetPanelMeshPath);
}

void PYDDManipulator::set_pose_DD_S(py::array T_DD_S)
{
	double *T_DD_S_ = (double *)T_DD_S.request().ptr;
	Pose3D pose_DD_S_;
	RVLHTRANSFMXDECOMP(T_DD_S_, pose_DD_S_.R, pose_DD_S_.t);
	// printf("pose_DD_S_:\n
	manipulator.setPose_DD_S(pose_DD_S_);
	manipulator.FreeSpacePlanes();
}

void PYDDManipulator::update_model_x()
{
	model_x = py_touch.touch.getModelX();
}

void PYDDManipulator::set_environment_from_touch()
{
	// Transform the VN model in Touch from rf E to 0
	int iFeature;
	RECOG::VN_::Feature *pFeatureSrc = model_x.pVNEnv->featureArray.Element;
	float tmp[3];

	// py_touch.touch.SceneBBox(model_x.pEnvSolidParams, &py_touch.bbox);
	// visualizer.Clear();
	// vtkSmartPointer<vtkActor> actor;
	// actor = model_x.pVNEnv->Display(&visualizer, 0.01f, NULL, NULL, 0.0f, &py_touch.bbox);
	// visualizer.Run();

	for (iFeature = 0; iFeature < model_x.pVNEnv->featureArray.n; iFeature++, pFeatureSrc++)
	{
		RVLPLANETRANSF3(pFeatureSrc->N, pFeatureSrc->d, py_touch.doorExpParams.pose_E_0.R, py_touch.doorExpParams.pose_E_0.t, tmp, pFeatureSrc->d);
		RVLCOPY3VECTOR(tmp, pFeatureSrc->N);
	}
	manipulator.pVNEnv = model_x.pVNEnv;
	RVL_DELETE_ARRAY(manipulator.dVNEnv);
	manipulator.dVNEnv = new float[manipulator.pVNEnv->featureArray.n];
	model_x.pVNEnv->CopyDescriptor(manipulator.dVNEnv);

	py_touch.touch.TransformModelVertices(&model_x, &py_touch.doorExpParams.pose_E_0);
	// py_touch.touch.SceneBBox2(&model_x, &py_touch.bbox);
	py_touch.touch.SceneBBox(model_x.pEnvSolidParams, &manipulator.vnbbox);

	// visualize_vn_model();

	printf("VN environment set from Touch model.\n");
}


void PYDDManipulator::set_environment_from_touch_gt()
{
	// Transform the VN model in Touch from rf E to 0
	model_gt = py_touch.touch.getModelGT();
	int iFeature;
	RECOG::VN_::Feature *pFeatureSrc = model_gt.pVNEnv->featureArray.Element;
	float tmp[3];

	manipulator.pVNEnv = model_gt.pVNEnv;
	RVL_DELETE_ARRAY(manipulator.dVNEnv);
	manipulator.dVNEnv = new float[manipulator.pVNEnv->featureArray.n];
	model_gt.pVNEnv->CopyDescriptor(manipulator.dVNEnv);

	// py_touch.touch.TransformModelVertices(&model_gt, &py_touch.doorExpParams.pose_E_0);
	// py_touch.touch.SceneBBox2(&model_x, &py_touch.bbox);
	py_touch.touch.SceneBBox(model_gt.pEnvSolidParams, &manipulator.vnbbox);

	// visualize_vn_model();

}

void PYDDManipulator::reset_vn_environment(py::array T_D_S)
{
	manipulator.pVNEnv = pVNEnv_;
	RVL_DELETE_ARRAY(manipulator.dVNEnv);
	manipulator.dVNEnv = new float[manipulator.pVNEnv->featureArray.n];

	pVNEnv_->CopyDescriptor(manipulator.dVNEnv);

	manipulator.UpdateFurnitureParams();

	manipulator.pVNEnv->BoundingBox(manipulator.dVNEnv, manipulator.vnbbox);

	double *T_D_S_ = (double *)T_D_S.request().ptr;
	Pose3D pose_D_S;
	RVLHTRANSFMXDECOMP(T_D_S_, pose_D_S.R, pose_D_S.t);
	manipulator.setPose_DD_S(pose_D_S);
	manipulator.FreeSpacePlanes();

}

void PYDDManipulator::visualize_vn_model()
{
	visualizer.Clear();
	vtkSmartPointer<vtkActor> actor, actorRF, actor2;
	actor = manipulator.pVNEnv->Display(&visualizer, 0.01f, NULL, NULL, 0.0f, &manipulator.vnbbox);

	Pose3D originPose;
	RVLUNITMX3(originPose.R);
	RVLSET3VECTOR(originPose.t, 0.0f, 0.0f, 0.0f);

	actorRF = visualizer.DisplayReferenceFrame(&(originPose), 0.2f);
	actor2 = visualizer.DisplayReferenceFrame(&(py_touch.touch.pose_D_0_x), 0.2f);
	visualizer.Run();
	visualizer.renderer->RemoveAllViewProps();
	visualizer.window->Finalize();

	// manipulator.VisualizeVNModelTest();
	printf("VN model visualized.\n");
}

py::array PYDDManipulator::get_corrected_cabinet_pose()
{
	// Convert pose_Arot_0 to numpy array
	auto T_A_E = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}));
	float *T_A_E_ = (float *)T_A_E.request().ptr;
	RVLHTRANSFMX(model_x.pose_A_E.R, model_x.pose_A_E.t, T_A_E_);
	// Return the pose_A_E as a numpy array
	return T_A_E;
}

py::array PYDDManipulator::get_corrected_camera_pose()
{
	auto T_C_E = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}));
	float *T_C_E_ = (float *)T_C_E.request().ptr;
	RVLHTRANSFMX(model_x.pose_C_E.R, model_x.pose_C_E.t, T_C_E_);
	// Return the camera pose as a numpy array
	return T_C_E;
}

py::array PYDDManipulator::get_corrected_pose_D_Arot()
{
	// Convert pose_DD_A to numpy array
	auto T_D_A = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}));
	float *T_D_A_ = (float *)T_D_A.request().ptr;
	RVLHTRANSFMX(py_touch.touch.pose_D_Arot_x.R, py_touch.touch.pose_D_Arot_x.t, T_D_A_);

	// Return the pose_DD_A as a numpy array
	return T_D_A;
}

py::array PYDDManipulator::get_corrected_pose_D_0()
{
	// Convert pose_D_0 to numpy array
	auto T_D_0 = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{4, 4},
		{4 * sizeof(float), sizeof(float)}));
	float *T_D_0_ = (float *)T_D_0.request().ptr;
	RVLHTRANSFMX(py_touch.touch.pose_D_0_x.R, py_touch.touch.pose_D_0_x.t, T_D_0_);

	// Return the pose_DD_0 as a numpy array
	return T_D_0;
}

////////////////////////////////////////////////////////////////////
//
//     RVL PYDDManipulator Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYDDManipulator, m)
{
	m.doc() = "RVL PYDDManipulator wrapper";

	// Expose PYTouch class
	py::class_<PYTouch>(m, "PYTouch")
		.def(py::init<>())
		.def("create", &PYTouch::create)
		.def("create_simple_tool", &PYTouch::create_simple_tool)
		.def("set_session_params", &PYTouch::set_session_params)
		.def("set_new_scene", &PYTouch::set_new_scene)
		.def("set_camera_params", &PYTouch::set_camera_params)
		.def("set_touch", &PYTouch::set_touch)
		.def("reset_touches", &PYTouch::reset_touches)
		.def("correct", &PYTouch::correct)
		.def("set_visualization", &PYTouch::set_visualization);

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
		.def("fwd_kinematics_6", &PYDDManipulator::fwd_kinematics_6)
		.def("inv_kinematics", &PYDDManipulator::inv_kinematics)
		.def("inv_kinematics_all_sols", &PYDDManipulator::inv_kinematics_all_sols)
		.def("inv_kinematics_all_sols_prev", &PYDDManipulator::inv_kinematics_all_sols_prev)
		.def("free", &PYDDManipulator::free)
		.def("free_tool_only", &PYDDManipulator::free_tool_only)
		.def("get_T_G_6", &PYDDManipulator::get_T_G_6)
		.def("visualize_current_state", &PYDDManipulator::visualize_current_state)
		.def("visualize_vn_current_state", &PYDDManipulator::visualize_vn_current_state)
		.def("approach_path_poses", &PYDDManipulator::approach_path_poses)
		.def("load_cabinet_static_mesh_fcl", &PYDDManipulator::load_cabinet_static_mesh_fcl)
		.def("load_cabinet_panel_mesh_fcl", &PYDDManipulator::load_cabinet_panel_mesh_fcl)
		.def("set_environment_from_touch", &PYDDManipulator::set_environment_from_touch)
		.def("set_environment_from_touch_gt", &PYDDManipulator::set_environment_from_touch_gt)
		.def("set_pose_DD_S", &PYDDManipulator::set_pose_DD_S)
		.def("visualize_vn_model", &PYDDManipulator::visualize_vn_model)
		.def("get_corrected_cabinet_pose", &PYDDManipulator::get_corrected_cabinet_pose)
		.def("get_corrected_camera_pose", &PYDDManipulator::get_corrected_camera_pose)
		.def("get_corrected_pose_D_Arot", &PYDDManipulator::get_corrected_pose_D_Arot)
		.def("get_corrected_pose_D_0", &PYDDManipulator::get_corrected_pose_D_0)
		.def("update_model_x", &PYDDManipulator::update_model_x)
		.def("reset_vn_environment", &PYDDManipulator::reset_vn_environment)
		.def_readwrite("py_touch", &PYDDManipulator::py_touch);
}
