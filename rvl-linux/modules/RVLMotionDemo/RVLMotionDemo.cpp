// RVLMotionDemo.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

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
#include "cnpy.h"
#include <chrono>

#define RVLMOTION_METHOD_DDM 0
#define RVLMOTION_METHOD_TOUCH 1
#define RVLMOTION_MODE_REAL 0
#define RVLMOTION_MODE_SIMULATION 1

using namespace RVL;

void TestSolver(
    std::string IODataFolder,
    bool bSaveProbleToFile = false,
    bool bLoadProblemFromFile = false);
void VisualizeTestLocalFreePose(
    Visualizer *pVisualizer,
    Box<float> wallBox,
    Box<float> holeBox,
    float tool_sample_sphere_r,
    Pose3D pose_G_S);
void TestLocalFreePose(
    CRVLMem *pMem0,
    CRVLMem *pMem,
    char *cfgFileName);
void TestFeasibleRobotPose(
    DDManipulator *pManipulator,
    Visualizer *pVisualizer);

void CreateParamList(
    CRVLParameterList *pParamList,
    CRVLMem *pMem,
    DWORD &method,
    DWORD &mode,
    float &dd_state_angle_deg,
    float &dd_end_state_angle_deg,
    int &nStates,
    float *qHome,
    float *t_A_S,
    float &rotz_A_S,
    char **pResultsFolder,
    bool &bSolverSave,
    bool &bSolverLoad,
    bool &bTouchSimulationOneByOne,
    char **pEnvModelFileName,
    char **pDoorSimulationSamplesFileName,
    char **pToolFlangePoseFileName,
    char **pDoorExperimentsVisionFileName,
    char **pDoorExperimentsGTFileName,
    char **pTouchFileName)
{
    pParamList->m_pMem = pMem;
    RVLPARAM_DATA *pParamData;
    pParamList->Init();

    pParamData = pParamList->AddParam("Motion.method", RVLPARAM_TYPE_ID, &method);
    pParamList->AddID(pParamData, "DDM", RVLMOTION_METHOD_DDM);
    pParamList->AddID(pParamData, "TOUCH", RVLMOTION_METHOD_TOUCH);
    pParamData = pParamList->AddParam("Motion.mode", RVLPARAM_TYPE_ID, &mode);
    pParamList->AddID(pParamData, "REAL", RVLMOTION_MODE_REAL);
    pParamList->AddID(pParamData, "SIM", RVLMOTION_MODE_SIMULATION);
    pParamData = pParamList->AddParam("DoorSateAngle(deg)", RVLPARAM_TYPE_FLOAT, &dd_state_angle_deg);
    pParamData = pParamList->AddParam("EndDoorSateAngle(deg)", RVLPARAM_TYPE_FLOAT, &dd_end_state_angle_deg);
    pParamData = pParamList->AddParam("nStates", RVLPARAM_TYPE_INT, &nStates);
    pParamData = pParamList->AddParam("Robot.home.q1", RVLPARAM_TYPE_FLOAT, qHome);
    pParamData = pParamList->AddParam("Robot.home.q2", RVLPARAM_TYPE_FLOAT, qHome + 1);
    pParamData = pParamList->AddParam("Robot.home.q3", RVLPARAM_TYPE_FLOAT, qHome + 2);
    pParamData = pParamList->AddParam("Robot.home.q4", RVLPARAM_TYPE_FLOAT, qHome + 3);
    pParamData = pParamList->AddParam("Robot.home.q5", RVLPARAM_TYPE_FLOAT, qHome + 4);
    pParamData = pParamList->AddParam("Robot.home.q6", RVLPARAM_TYPE_FLOAT, qHome + 5);
    pParamData = pParamList->AddParam("DDM.dd.t_A_S.x", RVLPARAM_TYPE_FLOAT, t_A_S);
    pParamData = pParamList->AddParam("DDM.dd.t_A_S.y", RVLPARAM_TYPE_FLOAT, t_A_S + 1);
    pParamData = pParamList->AddParam("DDM.dd.t_A_S.z", RVLPARAM_TYPE_FLOAT, t_A_S + 2);
    pParamData = pParamList->AddParam("DDM.dd.rotz_A_S", RVLPARAM_TYPE_FLOAT, &rotz_A_S);
    pParamData = pParamList->AddParam("ResultsFolder", RVLPARAM_TYPE_STRING, pResultsFolder);
    pParamData = pParamList->AddParam("Solver.save", RVLPARAM_TYPE_BOOL, &bSolverSave);
    pParamData = pParamList->AddParam("Solver.load", RVLPARAM_TYPE_BOOL, &bSolverLoad);
    pParamData = pParamList->AddParam("Touch.Simulation.one-by-one", RVLPARAM_TYPE_BOOL, &bTouchSimulationOneByOne);
    pParamData = pParamList->AddParam("Touch.Environment_model_file_name", RVLPARAM_TYPE_STRING, pEnvModelFileName);
    pParamData = pParamList->AddParam("Touch.Door_simulation_samples_file_name", RVLPARAM_TYPE_STRING, pDoorSimulationSamplesFileName);
    pParamData = pParamList->AddParam("Touch.Tool_flange_pose", RVLPARAM_TYPE_STRING, pToolFlangePoseFileName);
    pParamData = pParamList->AddParam("Touch.Door_experiments_vision_file_name", RVLPARAM_TYPE_STRING, pDoorExperimentsVisionFileName);
    pParamData = pParamList->AddParam("Touch.Door_experiments_gt_file_name", RVLPARAM_TYPE_STRING, pDoorExperimentsGTFileName);
    pParamData = pParamList->AddParam("Touch.Touches_file_name", RVLPARAM_TYPE_STRING, pTouchFileName);
}

int main(int argc, char **argv)
{
    // Create memory storage.

    CRVLMem mem0; // permanent memory
    mem0.Create(1000000000);
    CRVLMem mem; // cycle memory
    mem.Create(1000000000);

    // Read parameters from a configuration file.

    char cfgSelectionFileName[] = "RVLMotionDemo.cfg";
    char *cfgFileName = ReadConfigurationFile(cfgSelectionFileName);
    if (cfgFileName == NULL)
        return 1;
    printf("Configuration file: %s\n", cfgFileName);
    float dd_state_angle_deg = 10.0f;
    float dd_end_state_angle_deg = 90.0f;
    int nStates = 17;
    CRVLParameterList ParamList;
    char *resultsFolder = NULL;
    char *environmentModelFileName = NULL;
    char *doorSimulationSamplesFileName = NULL;
    char *toolFlangePoseFileName = NULL;
    char *doorExperimentsVisionFileName = NULL;
    char *doorExperimentsGTFileName = NULL;
    char *touchFileName = NULL;
    float qHome[6];
    memset(qHome, 0, 6 * sizeof(float));
    Pose3D pose_A_S;
    float rotz_A_S_deg = 0.0;
    bool bSolverSave = false;
    bool bSolverLoad = false;
    bool bTouchSimulationOneByOne = false;
    DWORD method = RVLMOTION_METHOD_DDM;
    DWORD mode = RVLMOTION_MODE_SIMULATION;
    CreateParamList(&ParamList,
                    &mem0,
                    method,
                    mode,
                    dd_state_angle_deg,
                    dd_end_state_angle_deg,
                    nStates,
                    qHome,
                    pose_A_S.t,
                    rotz_A_S_deg,
                    &resultsFolder,
                    bSolverSave,
                    bSolverLoad,
                    bTouchSimulationOneByOne,
                    &environmentModelFileName,
                    &doorSimulationSamplesFileName,
                    &toolFlangePoseFileName,
                    &doorExperimentsVisionFileName,
                    &doorExperimentsGTFileName,
                    &touchFileName);
    ParamList.LoadParams(cfgFileName);

    // Test DDManipulator::LocalFreePose()

    // TestLocalFreePose(&mem0, &mem, cfgFileName);

    // return 0;

    // Create visualizer.

    Visualizer visualizer;
    visualizer.Create();

    if (method == RVLMOTION_METHOD_DDM)
    {
        // Create manipulator.

        DDManipulator manipulator;
        manipulator.pMem0 = &mem0;
        manipulator.pMem = &mem;
        // manipulator.bVNPanel = true;    // For TestFeasibleRobotPose (false for TCMCS24)
        manipulator.Create(cfgFileName);
        manipulator.robot.minq[1] = -PI;
        manipulator.robot.maxq[1] = 0.0f;
        manipulator.robot.minq[3] = -PI;
        manipulator.robot.maxq[3] = 0.0f;
        manipulator.InitVisualizer(&visualizer);
        manipulator.resultsFolder = resultsFolder;
        manipulator.pTimer = new CRVLTimer;

        // Test Combinations.

        // Array2D<int> combs;
        // combs.Element = NULL;
        // Combinations(6, 3, combs);
        // delete[] combs.Element;

        // Test Solver.

        // TestSolver(resultsFolder, bSolverSave, bSolverLoad);

        // Test inverse kinematics of UR5.

        //{
        //    Pose3D pose_6_0;
        //    //float cs = 0.0f;
        //    //float sn = 1.0f;
        //    //RVLROTY(cs, sn, pose_6_0.R);
        //    float u[3];
        //    RVLSET3VECTOR(u, 1.0f, -2.0f, 3.0f);
        //    float fTmp;
        //    RVLNORM3(u, fTmp);
        //    AngleAxisToRot<float>(u, 0.6f, pose_6_0.R);
        //    RVLSET3VECTOR(pose_6_0.t, -0.1f, 0.78f, 0.17f);
        //    Array2D<float> invKinSolutions;
        //    invKinSolutions.Element = NULL;
        //    manipulator.robot.InvKinematics(pose_6_0, invKinSolutions);
        //    if (invKinSolutions.h > 0)
        //    {
        //        for (int i = 0; i < invKinSolutions.h; i++)
        //        {
        //            memcpy(manipulator.robot.q, invKinSolutions.Element + 6 * i, 6 * sizeof(float));
        //            manipulator.robot.FwdKinematics();
        //            float dR[9];
        //            RVLMXMUL3X3T1(pose_6_0.R, manipulator.robot.link_pose[5].R, dR);
        //            float errR = acos(RVLROTDIFF(dR));
        //            float dt[3];
        //            RVLDIF3VECTORS(pose_6_0.t, manipulator.robot.link_pose[5].t, dt);
        //            float errt = sqrt(RVLDOTPRODUCT3(dt, dt));
        //            printf("errR=%f errt=%f\n", errR, errt);
        //        }
        //    }
        //    else
        //        printf("No solutions found.\n");
        //    delete[] invKinSolutions.Element;
        //}

        // Robot pose.

        RVLUNITMX3(manipulator.robot.pose_0_W.R);
        RVLNULL3VECTOR(manipulator.robot.pose_0_W.t);
        manipulator.robot.pose_0_W.t[2] = 0.005f;

        // Initial tool pose.

        Pose3D pose_G_S_init;
        float robot_home_0[3];
        float qHomeRad[6];
        if (manipulator.bDefaultToolModel)
        {
            RVLROTY(-COS45, COS45, pose_G_S_init.R);
            RVLSET3VECTOR(robot_home_0, manipulator.robot.minr + manipulator.tool_len + manipulator.robot.d[5], 0.0f, 0.5f);
            RVLSUM3VECTORS(manipulator.robot.pose_0_W.t, robot_home_0, pose_G_S_init.t);
        }
        else
        {
            for (int i = 0; i < manipulator.robot.n; i++)
                qHomeRad[i] = DEG2RAD * qHome[i];
        }
        // FILE *fpDebug = fopen("pose_G_S_init.txt", "w");
        // float T_G_S_init[16];
        // RVLHTRANSFMX(pose_G_S_init.R, pose_G_S_init.t, T_G_S_init);
        // PrintMatrix(fpDebug, T_G_S_init, 4, 4);
        // fclose(fpDebug);

        /// Furniture pose.

        // Furniture pose 1.

        // RVLNULLMX3X3(manipulator.pose_F_S.R);
        // RVLMXEL(manipulator.pose_F_S.R, 3, 0, 2) = 1.0f;
        // RVLMXEL(manipulator.pose_F_S.R, 3, 1, 0) = -1.0f;
        // RVLMXEL(manipulator.pose_F_S.R, 3, 2, 1) = -1.0f;

        // Furniture pose 2.

        // RVLNULLMX3X3(manipulator.pose_F_S.R);
        // RVLSET3VECTOR(manipulator.pose_F_S.t, 0.6f, 0.0f, 0.546f);
        // RVLMXEL(manipulator.pose_F_S.R, 3, 0, 0) = -1.0f;
        // RVLMXEL(manipulator.pose_F_S.R, 3, 1, 2) = -1.0f;
        // RVLMXEL(manipulator.pose_F_S.R, 3, 2, 1) = -1.0f;
        // RVLSET3VECTOR(manipulator.pose_F_S.t, 0.2f, -0.3f, 0.546f);

        // Furniture pose 3.

        // RVLNULLMX3X3(manipulator.pose_F_S.R);
        // RVLMXEL(manipulator.pose_F_S.R, 3, 0, 0) = 1.0f;
        // RVLMXEL(manipulator.pose_F_S.R, 3, 1, 2) = 1.0f;
        // RVLMXEL(manipulator.pose_F_S.R, 3, 2, 1) = -1.0f;
        // RVLSET3VECTOR(manipulator.pose_F_S.t, 0.6f, 0.2f, 0.546f);

        // Update static pose (for furniture poses 1, 2 and 3).

        // manipulator.UpdateStaticPose();

        // Set door parameters.

        // manipulator.SetDoorModelParams(0.018f, 0.396f, 0.496f, 0.0f, -0.5f * 0.396f, -1.0f, 0.018f, 0.005f);
        // manipulator.SetDoorModelParams(0.018f, 0.4105020669399432, 0.5578196062516284, 0.0f, -0.5f * 0.4105020669399432, -1.0f, 0.018f, 0.005f);
        // manipulator.SetDoorModelParams(0.020f, 0.4, 0.5, 0.0f, -0.5f * 0.4, -1.0f, 0.02f, 0.005f);
        // manipulator.pVNEnv->Display(&visualizer, 0.02f, manipulator.dVNEnv);
        // visualizer.Run();
        // visualizer.Clear();

        // float P[2][3];
        // RVLSET3VECTOR(P[0], -0.1f, 0.2f, 0.1f);
        // RVLSET3VECTOR(P[1], 0.1f, 0.2f, 0.1f);
        // Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection =
        //     manipulator.pVNEnv->VolumeCylinderIntersection(manipulator.dVNEnv, P[0], P[1], 0.05f);

        // Set door pose. (Furniture pose is computed from the door pose.)

        // float rotz_A_S = DEG2RAD * rotz_A_S_deg;
        // float cs = cos(rotz_A_S);
        // float sn = sin(rotz_A_S);
        // RVLROTZ(cs, sn, pose_A_S.R);
        // manipulator.SetDoorPose(pose_A_S);

        // Door state.

        // manipulator.SetEnvironmentState(dd_state_angle_deg);

        // manipulator.pVNEnv->Display(&visualizer, 0.02f, manipulator.dVNEnv);
        // visualizer.Run();
        // visualizer.Clear();

        ///

        // Test the algorithm for finding a feasible robot pose.

        // TestFeasibleRobotPose(&manipulator, &visualizer);

        // Path planning.

        // manipulator.Path(&pose_G_S_init);
        Array<Pose3D> poses_G_0;
        // manipulator.SetVisualizeVNEnvironmentModel();
        Array2D<float> robotJoints;
        Array<Array<Pose3D>> allFeasiblePaths;
        Array<Array2D<float>> allFeasiblePathsJoints;
        // std::ifstream exampleFile("C:\\RVL\\ExpRez\\examples.txt");
        std::ifstream exampleFile("/home/RVLuser/rvl-linux/cabinets_examples.txt");
        std::string example;
        bool success;
        int num_successful = 0;
        bool bSaveTrajectories = true;
        std::string saveTrajDir = "/home/RVLuser/rvl-linux/data/Exp-MC-Spheres_traj";

        std::vector<double> runtimes;
        double duration;
        int i_ = 0;
        manipulator.bCountPoses = true;
        // manipulator.vecSelectedNodes_debug.clear();
        std::vector<int> vecExploredFirstPoses;
        FILE *fpLogFreeq = fopen("/home/RVLuser/rvl-linux/colchecks_Freeq.csv", "w");
        FILE *fpLogFreeSDF = fopen("/home/RVLuser/rvl-linux/colchecks_FreeSDF.csv", "w");

        while (std::getline(exampleFile, example))
        {
            // manipulator.pVNEnv->Display(&visualizer, 0.02f, manipulator.dVNEnv);
            // visualizer.Run();
            // visualizer.Clear();
            manipulator.LoadExampleIndexed(example);

            // FCL
            // if (manipulator.use_fcl)
            manipulator.LoadToolModelFCL();
            // std::string cabinetStaticModelPath = "/home/RVLuser/ferit_ur5_ws/cabinet_static.ply";
            // fcl::Transform3d T_A_S;
            // manipulator.RVLPose2FCLPose(pose_A_S, T_A_S);
            // manipulator.LoadCabinetStaticFCL(cabinetStaticModelPath, pose_A_S);

            manipulator.vecFeasiblePoses_debug.clear();

            std::ios_base::sync_with_stdio(false);
            auto start = std::chrono::high_resolution_clock::now();
            success = manipulator.Path2(qHomeRad, dd_end_state_angle_deg, nStates, poses_G_0, robotJoints, &allFeasiblePaths, &allFeasiblePathsJoints);
            auto end = std::chrono::high_resolution_clock::now();

            // colchecks
            fprintf(fpLogFreeq, "%d", manipulator.vecNumColchecks_Freeq.at(0));
            fprintf(fpLogFreeSDF, "%d", manipulator.vecNumColchecks_FreeSDF.at(0));

            for (int iState = 1; iState < manipulator.vecNumColchecks_Freeq.size(); iState++)
            {
                fprintf(fpLogFreeq, ",%d", manipulator.vecNumColchecks_Freeq.at(iState));
                fprintf(fpLogFreeSDF, ",%d", manipulator.vecNumColchecks_FreeSDF.at(iState));
            }
            fprintf(fpLogFreeq, "\n");
            fprintf(fpLogFreeSDF, "\n");

            if (manipulator.bCountPoses)
            {
                // printf("Num of selected nodes: %d\n", manipulator.vecSelectedNodes_debug.back());
                vecExploredFirstPoses.push_back(manipulator.vecnumExploredPoses_debug.at(0));
                printf("%d", manipulator.vecnumExploredPoses_debug.at(0));
                if (success)
                {
                    for (int iFeasible = 1; iFeasible < nStates; iFeasible++)
                    {
                        printf(",%d", manipulator.vecnumExploredPoses_debug.at(iFeasible));
                    }
                }
                printf("\n");

                printf("Num of feasible poses accumulated: %d", manipulator.vecFeasiblePoses_debug.at(0));
                if (success)
                {
                    for (int iFeasible = 1; iFeasible < nStates; iFeasible++)
                    {
                        printf(", %d", manipulator.vecFeasiblePoses_debug.at(iFeasible));
                    }
                }
                printf("\n");
            }

            if (success)
            {
                // printf("Path is successfully generated.\n");
                num_successful++;
            }
            // else
            // printf("Path is not found.\n");

            duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            duration *= 1e-9;

            // if(runtimes.size() < 20)
            {
                runtimes.push_back(duration);
                // cout << "Time taken to generate path: " << duration << setprecision(9) << " sec" << endl;
                // break;
            }

            if (success && bSaveTrajectories)
            {
                FILE *fpLog = fopen((saveTrajDir + RVLFILEPATH_SEPARATOR + "traj_" + std::to_string(i_) + ".txt").data(), "w");
                for (int iPathPt = 0; iPathPt < robotJoints.h; iPathPt++)
                {
                    float *q_ = robotJoints.Element + iPathPt * robotJoints.w;
                    fprintf(fpLog, "%f", *q_);
                    for (int iJoint = 1; iJoint < robotJoints.w; iJoint++)
                    {
                        fprintf(fpLog, ",%f", *(q_ + iJoint));
                    }
                    fprintf(fpLog, "\n");
                }
                fclose(fpLog);
            }

            i_++;

            // Visualization.

            // Pose3D pose_G_DD = manipulator.feasibleTCPs.Element[100];
            // Pose3D pose_G_Arot;
            // RVLCOMPTRANSF3D(manipulator.pose_DD_A.R, manipulator.pose_DD_A.t, pose_G_DD.R, pose_G_DD.t, pose_G_Arot.R, pose_G_Arot.t);
            // Pose3D pose_G_A;
            // RVLCOMPTRANSF3D(manipulator.pose_Arot_A.R, manipulator.pose_Arot_A.t, pose_G_Arot.R, pose_G_Arot.t, pose_G_A.R, pose_G_A.t);
            // Pose3D pose_G_W;
            // RVLCOMPTRANSF3D(manipulator.pose_A_W.R, manipulator.pose_A_W.t, pose_G_A.R, pose_G_A.t, pose_G_W.R, pose_G_W.t);
            // Pose3D pose_G_S;
            // RVLCOMPTRANSF3D(manipulator.pose_W_S.R, manipulator.pose_W_S.t, pose_G_W.R, pose_G_W.t, pose_G_S.R, pose_G_S.t);
            ////manipulator.SetVisualizeVNEnvironmentModel();
            // manipulator.Visualize(pose_G_S);
            // visualizer.Run();

            //
        }

        fclose(fpLogFreeq);
        fclose(fpLogFreeSDF);

        // if (manipulator.bCountPoses)
        // {
        //     int min_selectedNodes = *std::min_element(manipulator.vecSelectedNodes_debug.begin(), manipulator.vecSelectedNodes_debug.end());
        //     int max_selectedNodes = *std::max_element(manipulator.vecSelectedNodes_debug.begin(), manipulator.vecSelectedNodes_debug.end());
        //     int totalSelectedNodes = 0;
        //     for (int el: manipulator.vecSelectedNodes_debug) {
        //         totalSelectedNodes += el;
        //     }
        //     double avg_selectedNodes = static_cast<double>(totalSelectedNodes) / static_cast<double>(manipulator.vecSelectedNodes_debug.size());
        //     // std::cout << "Min selected poses: " << min_selectedNodes << endl;
        //     // std::cout << "Max selected poses: " << max_selectedNodes << endl;
        //     // printf("Avg selected poses: %.4f\n", avg_selectedNodes);
        //     // std::cout << "Avg selected poses: " << avg_selectedNodes << setprecision(4) << "\n";
        //     printf("\n");
        //     int min_exploredFirstPoses = *std::min_element(vecExploredFirstPoses.begin(), vecExploredFirstPoses.end());
        //     int max_exploredFirstPoses = *std::max_element(vecExploredFirstPoses.begin(), vecExploredFirstPoses.end());
        //     int totalexploredFirstPoses = 0;
        //     for (int el: vecExploredFirstPoses) {
        //         totalexploredFirstPoses += el;
        //     }
        //     double avg_exploredFirstPoses = static_cast<double>(totalexploredFirstPoses) / static_cast<double>(vecExploredFirstPoses.size());
        //     // std::cout << "Min explored first poses: " << min_exploredFirstPoses << endl;
        //     // std::cout << "Max explored first poses: " << max_exploredFirstPoses << endl;
        //     printf("Avg explored first poses: %.4f\n", avg_exploredFirstPoses);
        //     // printf("\n");
        // }

        double total_time = 0.0;
        double min_time = *std::min_element(runtimes.begin(), runtimes.end());
        double max_time = *std::max_element(runtimes.begin(), runtimes.end());

        for (double time : runtimes)
        {
            total_time += time;
        }
        double average_time = total_time / runtimes.size();

        std::cout << "Total execution time: " << total_time << setprecision(4) << " seconds\n";
        std::cout << "Minimum execution time: " << min_time << setprecision(4) << " seconds\n";
        std::cout << "Maximum execution time: " << max_time << setprecision(4) << " seconds\n";
        std::cout << "Average execution time: " << average_time << setprecision(4) << " seconds\n";
        std::cout << "Number of successful exps: " << num_successful << endl;

        exampleFile.close();
        RVL_DELETE_ARRAY(poses_G_0.Element);
        RVL_DELETE_ARRAY(robotJoints.Element);
    }
    else if (method == RVLMOTION_METHOD_TOUCH)
    {
        //{
        //    cnpy::NpyArray npyData = cnpy::npy_load("D:\\Cupec\\Documents\\Google_Disk\\2025\\Projects\\DOK-2021-02\\VFTF\\Experiments\\T_6_0_contact.npy");
        //    double* data = npyData.data<double>();
        //    Pose3D pose_6_0;
        //    RVLHTRANSFMXDECOMP(data, pose_6_0.R, pose_6_0.t);
        //    MOTION::Robot robot;
        //    robot.pMem0 = &mem0;
        //    robot.Create("D:\\Cupec\\Documents\\Repos\\rvl\\RVLMotionDemo\\RVLMotionDemo_Cupec.cfg");
        //    Array<MOTION::IKSolution> solutions;
        //    MOTION::IKSolution solutionMem[8];
        //    solutions.Element = solutionMem;
        //    robot.InvKinematics(pose_6_0, solutions, false);
        //    Pose3D pose_6_0_;
        //    for (int i = 0; i < solutions.n; i++)
        //    {
        //        robot.FwdKinematics(solutions.Element[i].q, &pose_6_0_);
        //        int debug = 0;
        //    }
        //}

        Touch touch;
        touch.pMem0 = &mem0;
        touch.Create(cfgFileName);
        touch.InitVisualizer(&visualizer, cfgFileName);
        touch.bDoor = true;
        touch.resultsFolder = resultsFolder;
        std::vector<MOTION::DoorExperimentParams> simulations;
        MOTION::DoorExperimentParams simParams;
        simParams.a = 0.3f;
        simParams.sx = 0.018f;
        simParams.sy = 0.4;
        simParams.sz = 0.5f;
        // float rx = 0.01f;
        simParams.rx = 0.0f;
        // float b = 0.0025f;
        simParams.b = 0.0f;
        simParams.c = 0.005f;
        simParams.qDeg = -10.0f;
        simParams.ry = -(0.5f * simParams.sy + simParams.b);
        // float a_tool = 0.019f;
        // float b_tool = 0.064f;
        // float c_tool = 0.007f;
        // float d_tool = 0.049f;
        // float h_tool = 0.02706f;
        float a_tool = 0.0205;
        float b_tool = 0.032;
        float c_tool = 0.011;
        float d_tool = 0.026;
        float h_tool = 0.023;
        FILE *fpToolFlangePose = fopen(toolFlangePoseFileName, "rb");
        if (fpToolFlangePose)
        {
            cnpy::NpyArray npyData = cnpy::npy_load(toolFlangePoseFileName);
            double *data = npyData.data<double>();
            RVLHTRANSFMXDECOMP(data, touch.pose_tool_E.R, touch.pose_tool_E.t);
            fclose(fpToolFlangePose);
        }
        touch.CreateSimpleTool(a_tool, b_tool, c_tool, d_tool, h_tool, &(touch.pose_tool_E));
        if (mode == RVLMOTION_MODE_SIMULATION)
        {
            if (doorSimulationSamplesFileName)
            {
                std::ifstream sampleFile(doorSimulationSamplesFileName);
                std::string simulationSampleHeader;
                std::vector<std::string> sampleFormat;
                if (std::getline(sampleFile, simulationSampleHeader))
                    touch.LoadExperimentFileFormat(simulationSampleHeader, sampleFormat);
                if (touch.simulation == RVLMOTION_TOUCH_SIMULATION_OPEN)
                {
                    std::string simulationSample;
                    while (std::getline(sampleFile, simulationSample))
                    {
                        touch.LoadExperimentDataFromFile(simulationSample, sampleFormat, &simParams);
                        simulations.push_back(simParams);
                        if (bTouchSimulationOneByOne)
                        {
                            touch.Simulation(simulations);
                            simulations.clear();
                        }
                    }
                    if (!bTouchSimulationOneByOne)
                        touch.Simulation(simulations);
                }
            }
            if (touch.simulation == RVLMOTION_TOUCH_SIMULATION_RND)
            {
                simulations.push_back(simParams);
                touch.Simulation(simulations);
            }
        }
        else
        {
            if (doorExperimentsVisionFileName)
            {
                std::ifstream expFile(doorExperimentsVisionFileName);
                std::string experimentDataHeader;
                std::vector<std::string> expDataFormat;
                if (std::getline(expFile, experimentDataHeader))
                    touch.LoadExperimentFileFormat(experimentDataHeader, expDataFormat);
                std::string strExpData;
                std::vector<MOTION::DoorExperimentParams> expData;
                MOTION::DoorExperimentParams expData_;
                while (std::getline(expFile, strExpData))
                {
                    touch.LoadExperimentDataFromFile(strExpData, expDataFormat, &expData_);
                    float M3x3Tmp[9];
                    RVLMXMUL3X3T1(expData_.pose_C_E.R, expData_.pose_C_E.R, M3x3Tmp);
                    float s = sqrt((M3x3Tmp[0] + M3x3Tmp[4] + M3x3Tmp[8]) / 3.0f);
                    RVLSCALEMX3X32(expData_.pose_C_E.R, s, expData_.pose_C_E.R);
                    RVLSCALE3VECTOR(expData_.pose_A_C.t, s, expData_.pose_A_C.t);
                    expData_.bGT = false;
                    expData_.a = simParams.a;
                    expData_.b = simParams.b;
                    expData_.c = simParams.c;
                    expData.push_back(expData_);
                }
                if (doorExperimentsGTFileName)
                {
                    std::ifstream expFile(doorExperimentsGTFileName);
                    std::string experimentGTHeader;
                    std::vector<std::string> expGTFormat;
                    if (std::getline(expFile, experimentGTHeader))
                        touch.LoadExperimentFileFormat(experimentGTHeader, expGTFormat);
                    std::string strExpGT;
                    MOTION::DoorExperimentParams expGT;
                    while (std::getline(expFile, strExpGT))
                    {
                        touch.LoadExperimentDataFromFile(strExpGT, expGTFormat, &expGT);
                        for (int iExp = 0; iExp < expData.size(); iExp++)
                        {
                            if (expData[iExp].sessionIdx == expGT.sessionIdx && expData[iExp].sceneIdx == expGT.sceneIdx)
                            {
                                Pose3D pose_Arot_A;
                                float q = DEG2RAD * expGT.qDeg;
                                float cs = cos(q);
                                float sn = sin(q);
                                RVLROTZ(cs, sn, pose_Arot_A.R);
                                RVLNULL3VECTOR(pose_Arot_A.t);
                                Pose3D pose_A_Arot;
                                RVLINVTRANSF3D(pose_Arot_A.R, pose_Arot_A.t, pose_A_Arot.R, pose_A_Arot.t);
                                RVLCOMPTRANSF3D(expGT.pose_A_0_gt.R, expGT.pose_A_0_gt.t, pose_A_Arot.R, pose_A_Arot.t,
                                                expData[iExp].pose_A_0_gt.R, expData[iExp].pose_A_0_gt.t);
                                expData[iExp].sxgt = expGT.sx;
                                expData[iExp].sygt = expGT.sy;
                                expData[iExp].szgt = expGT.sz;
                                expData[iExp].rxgt = expGT.rx;
                                expData[iExp].rygt = expGT.ry;
                                expData[iExp].qDeg_gt = expGT.qDeg;
                                expData[iExp].bGT = true;
                            }
                        }
                    }
                }
                std::vector<MOTION::TouchData> touches;
                if (touchFileName)
                {
                    std::ifstream touchFile(touchFileName);
                    std::string touchFileHeader;
                    std::vector<std::string> touchFileFormat;
                    if (std::getline(touchFile, touchFileHeader))
                        touch.LoadExperimentFileFormat(touchFileHeader, touchFileFormat);
                    std::string strTouch;
                    MOTION::TouchData touchData;
                    while (std::getline(touchFile, strTouch))
                    {
                        touch.LoadTouch(strTouch, touchFileFormat, &touchData);
                        touches.push_back(touchData);
                    }
                }
                touch.TestCorrection(expData, touches);
            }
            else
                printf("ERROR: Experiment vision file name is not defined!\n");
        }
    }

    delete[] resultsFolder;
    delete[] environmentModelFileName;
    delete[] doorSimulationSamplesFileName;
    delete[] toolFlangePoseFileName;
    delete[] doorExperimentsVisionFileName;
    delete[] doorExperimentsGTFileName;
    delete[] touchFileName;

    return 0;
}

void TestSolver(
    std::string IODataFolder,
    bool bSaveProbleToFile,
    bool bLoadProblemFromFile)
{
    Solver solver;
    int m = 10;
    int n = 6;
    int m_;
    solver.Create(m, n);
    float *A = new float[m * n];
    memset(A, 0, m * n * sizeof(float));
    float *b = new float[m];
    float *x0 = new float[n];
    float *x = new float[n];
    float *e = new float[m];
    bool bProblemInNPYFile = false;
    bool bBoxLimits = false;
    int i, j;
    if (bBoxLimits)
    {
        m_ = m - 2 * n;
        int dim, k;
        j = m_;
        for (dim = 0; dim < n; dim++)
            for (k = 0; k < 2; k++, j++)
            {
                A[j * n + dim] = (float)(2 * k - 1);
                b[j] = 1.0f;
            }
    }
    else
        m_ = m;
    printf("Test started...  \n");
    // system("pause");
    bool bVerbose = true;
    bool bFeasibleSolution = true;
    bool bMinimum = true;
    Array<int> S;
    S.Element = new int[m_];
    S.n = m_;
    for (i = 0; i < m_; i++)
        S.Element[i] = i;
    solver.SetLinearConstraints(A, b);
    solver.SetInequalityConstraints(S);
    for (int it = 0; it < 10000; it++)
    {
        if (bVerbose)
            printf("Test %d\n", it);
        if (bLoadProblemFromFile)
        {
            if (bProblemInNPYFile)
            {
                cnpy::NpyArray npyData = cnpy::npy_load(IODataFolder + RVLFILEPATH_SEPARATOR_ + "data.npy");
                double *data = npyData.data<double>();
                double *pData = data;
                for (j = 0; j < m_; j++)
                {
                    for (i = 0; i < n; i++)
                        A[j * n + i] = *(pData++);
                    b[j] = *(pData++);
                }
            }
            else
            {
                FILE *fp = fopen((IODataFolder + RVLFILEPATH_SEPARATOR_ + "data.dat").data(), "rb");
                fread(A, sizeof(float), m_ * n, fp);
                fread(b, sizeof(float), m_, fp);
                fclose(fp);
            }
        }
        else
        {
            int rnd;
            for (j = 0; j < m_; j++)
            {
                for (i = 0; i < n; i++)
                {
                    rnd = 10000 * (rand() % 10000) + rand() % 10000;
                    A[i + j * n] = 2.0f * ((float)rnd / 100000000.0f) - 1.0f;
                }
                rnd = 10000 * (rand() % 10000) + rand() % 10000;
                b[j] = 2.0f * ((float)rnd / 100000000.0f) - 1.0f;
            }
            if (bSaveProbleToFile)
            {
                FILE *fp = fopen((IODataFolder + RVLFILEPATH_SEPARATOR_ + "data.dat").data(), "wb");
                fwrite(A, sizeof(float), m_ * n, fp);
                fwrite(b, sizeof(float), m_, fp);
                fclose(fp);
            }
        }
        memset(x0, 0, n * sizeof(float));
        // bFeasibleSolution = solver.FeasibleSolution2(A, b, m, x0, x);
        float exmax;
        bMinimum = solver.FeasibleSolution3(x0, x, exmax);
        if (bVerbose)
        {
            if (!bFeasibleSolution)
                printf("Feasible solution not found.\n\n");
            if (!bMinimum)
            {
                printf("Open set.\n\n");
                for (i = 0; i < n; i++)
                    x[i] += 1000.0f * solver.v[i];
            }
            float *a;
            RVLMULMXVECT(A, x, m, n, e, i, j, a);
            RVLDIFVECTORS(e, b, m, e, i);
            printf("x:\n");
            for (i = 0; i < n; i++)
                printf("%f\n", x[i]);
            bool bFirst = true;
            float maxe = 0.0f;
            for (j = 0; j < m; j++)
                if (bFirst || e[j] > maxe)
                {
                    maxe = e[j];
                    bFirst = false;
                }
            printf("maxe=%f\n\n", maxe);
            // if (maxe > 1e-6)
            //     system("pause");
        }
    }
    delete[] A;
    delete[] b;
    delete[] x0;
    delete[] x;
    delete[] e;
    printf("Test completed.\n\n\n");
    system("pause");
}

void VisualizeTestLocalFreePose(
    Visualizer *pVisualizer,
    Box<float> wallBox,
    Box<float> holeBox,
    float tool_sample_sphere_r,
    Pose3D pose_G_S)
{
    uchar red[] = {255, 0, 0};
    uchar blue[] = {0, 0, 255};

    // Display environment VN model.

    // pVNEnv->Display(&visualizer, 0.01f, dVNEnv);
    // visualizer.Run();

    // Display environment box model.

    Vector3<float> boxSize;
    Vector3<float> boxCenter;
    BoxSize<float>(&wallBox, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&wallBox, boxCenter.Element);
    Pose3D pose_box_S;
    RVLUNITMX3(pose_box_S.R);
    RVLCOPY3VECTOR(boxCenter.Element, pose_box_S.t);
    pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);
    BoxSize<float>(&holeBox, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&holeBox, boxCenter.Element);
    RVLCOPY3VECTOR(boxCenter.Element, pose_box_S.t);
    pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);

    // Display tool.

    Box<float> toolBox;
    toolBox.minx = -tool_sample_sphere_r;
    toolBox.maxx = tool_sample_sphere_r;
    toolBox.miny = -5.0f * tool_sample_sphere_r;
    toolBox.maxy = 5.0f * tool_sample_sphere_r;
    toolBox.minz = -tool_sample_sphere_r;
    toolBox.maxz = tool_sample_sphere_r;
    BoxSize<float>(&toolBox, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_G_S, 255.0, 0.0, 0.0);

    // Run visualization.

    pVisualizer->Run();

    // Clear visualization.

    pVisualizer->renderer->RemoveAllViewProps();
}

void TestLocalFreePose(
    CRVLMem *pMem0,
    CRVLMem *pMem,
    char *cfgFileName)
{
    // Create visualizer.

    Visualizer visualizer;
    visualizer.Create();

    // Create manipulator.

    DDManipulator manipulator;
    manipulator.pMem0 = pMem0;
    manipulator.pMem = pMem;

    // Create environment.

    manipulator.pVNEnv = new VN;
    VN *pVNEnv = manipulator.pVNEnv;
    pVNEnv->CreateEmpty();
    Array2D<float> A;
    A.w = 3;
    A.h = 6;
    A.Element = new float[A.w * A.h];
    CreateConvexTemplate6(A.Element);
    Array<RECOG::PSGM_::Plane> CT;
    CT.n = A.h;
    CT.Element = new RECOG::PSGM_::Plane[CT.n];
    RECOG::PSGM_::CreateTemplate(A, CT);
    float R[9];
    RVLUNITMX3(R);
    float t[3];
    RVLNULL3VECTOR(t);
    Pair<float, float> betaInterval;
    betaInterval.a = 0.0f;
    betaInterval.b = PI;
    Array2D<float> NArray;
    NArray.w = 3;
    NArray.h = 0;
    Array<RECOG::VN_::ModelCluster *> VNMClusters;
    VNMClusters.n = 2;
    VNMClusters.Element = new RECOG::VN_::ModelCluster *[VNMClusters.n];
    VNMClusters.Element[0] = pVNEnv->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0);
    betaInterval.a = 0.5f * PI;
    betaInterval.b = 0.5f * PI;
    VNMClusters.Element[1] = pVNEnv->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONCAVE, R, t, 0.5f, CT, betaInterval, NArray, pMem0);
    pVNEnv->AddOperation(2, 1, 0, 1, pMem0);
    pVNEnv->SetOutput(2);
    pVNEnv->Create(pMem0);

    Array<Vector3<float>> vertices;
    vertices.n = 16;
    vertices.Element = new Vector3<float>[vertices.n];
    float *vertices_ = new float[3 * vertices.n];
    Box<float> wallBox;
    wallBox.minx = -0.2f;
    wallBox.maxx = 0.2f;
    wallBox.miny = -0.2f;
    wallBox.maxy = 0.2f;
    wallBox.minz = -0.02f;
    wallBox.maxz = 0.02f;
    BoxVertices<float>(&wallBox, vertices_);
    Box<float> holeBox;
    holeBox.minx = -0.015f;
    holeBox.maxx = 0.015f;
    holeBox.miny = -0.055f;
    holeBox.maxy = 0.055f;
    holeBox.minz = -0.02f;
    holeBox.maxz = 0.02f;
    BoxVertices<float>(&holeBox, vertices_ + 3 * 8);
    Array<RECOG::VN_::Correspondence5> assoc;
    assoc.n = 16;
    assoc.Element = new RECOG::VN_::Correspondence5[assoc.n];
    RECOG::VN_::Correspondence5 *pAssoc = assoc.Element;
    int iPt;
    for (iPt = 0; iPt < 8; iPt++, pAssoc++)
    {
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 0;
        pAssoc->iBeta = -1;
    }
    for (iPt = 8; iPt < 16; iPt++, pAssoc++)
    {
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 1;
        pAssoc->iBeta = -1;
    }
    float *PSrc = vertices_;
    float *PTgt;
    for (iPt = 0; iPt < vertices.n; iPt++, PSrc += 3)
    {
        PTgt = vertices.Element[iPt].Element;
        RVLCOPY3VECTOR(PSrc, PTgt);
    }
    RVL_DELETE_ARRAY(manipulator.dVNEnv);
    manipulator.dVNEnv = new float[pVNEnv->featureArray.n];
    float *dVNEnv = manipulator.dVNEnv;
    pVNEnv->Descriptor(vertices, assoc, dVNEnv);
    pVNEnv->SetFeatureOffsets(dVNEnv);

    delete[] A.Element;
    delete[] CT.Element;

    // Create tool.

    Array<MOTION::Sphere> *pToolSampleSpheres = &(manipulator.tool_sample_spheres);
    pToolSampleSpheres->n = 5;
    RVL_DELETE_ARRAY(pToolSampleSpheres->Element);
    pToolSampleSpheres->Element = new MOTION::Sphere[pToolSampleSpheres->n];
    float tool_sample_sphere_r = 0.01f;
    int iSphere;
    for (iSphere = 0; iSphere < pToolSampleSpheres->n; iSphere++)
    {
        RVLSET3VECTOR(pToolSampleSpheres->Element[iSphere].c.Element, 0.0f, 2.0f * tool_sample_sphere_r * (float)(iSphere - 2), 0.0f);
        pToolSampleSpheres->Element[iSphere].r = tool_sample_sphere_r;
    }

    // Complete manipulator.

    manipulator.Create(cfgFileName);
    manipulator.rLocalConstraints = 0.08f;

    /// Perform several tests.

    for (int iTest = 0; iTest < 10; iTest++)
    {
        // Initial tool pose.

        Pose3D pose_G_S_init;
        float rotAxis[3];
        // RVLSET3VECTOR(rotAxis, 0.0f, 0.0f, 1.0f);
        float fTmp;
        RVLRNDUNIT3VECTOR(rotAxis, fTmp);
        float rotAngle = 30.0f * DEG2RAD;
        AngleAxisToRot<float>(rotAxis, rotAngle, pose_G_S_init.R);
        float tRange = 0.01f;
        RVLSET3VECTOR(pose_G_S_init.t, tRange * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f), tRange * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f), tRange * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f));
        // RVLUNITMX3(pose_G_S_init.R);
        // RVLNULL3VECTOR(pose_G_S_init.t);

        // Visualization.

        VisualizeTestLocalFreePose(&visualizer, wallBox, holeBox, tool_sample_sphere_r, pose_G_S_init);

        // Find collision-free pose in the local vicinity of the current pose.

        Pose3D pose_G_S;
        float *SDF = new float[pVNEnv->featureArray.n];
        Array<Pair<int, int>> localConstraints;
        localConstraints.Element = new Pair<int, int>[pVNEnv->featureArray.n * pToolSampleSpheres->n];
        Vector3<float> *c_S_rot = new Vector3<float>[manipulator.tool_sample_spheres.n];
        Vector3<float> *c_S = new Vector3<float>[manipulator.tool_sample_spheres.n];
        manipulator.LocalConstraints(&pose_G_S_init, SDF, localConstraints, c_S_rot, c_S);
        bool bFreePose = manipulator.FreePose(&pose_G_S_init, localConstraints, c_S_rot, c_S, &pose_G_S);
        if (!bFreePose)
            printf("No collision-free pose found in the vicinity of the current pose.\n");
        delete[] c_S;
        delete[] c_S_rot;
        delete[] SDF;

        // Visualization.

        VisualizeTestLocalFreePose(&visualizer, wallBox, holeBox, tool_sample_sphere_r, pose_G_S);
    }
}

void TestFeasibleRobotPoseFwdKinematics(
    float *ks,
    float *R_G_P,
    Pose3D &pose_G_B)
{
    float cph = cos(ks[0]);
    float sph = sin(ks[0]);
    float cth = cos(ks[1]);
    float sth = sin(ks[1]);
    float cps = cos(ks[2]);
    float sps = sin(ks[2]);
    float cal = cos(ks[3]);
    float sal = sin(ks[3]);
    float Ryal[9];
    RVLROTY(cal, sal, Ryal);
    float Rzth[9];
    RVLROTZ(cth, sth, Rzth);
    float Ryph[9];
    RVLROTY(cph, sph, Ryph);
    float Rzps[9];
    RVLROTZ(cps, sps, Rzps);
    float Mx3x3Tmp[9];
    RVLMXMUL3X3(Ryph, Rzps, Mx3x3Tmp);
    RVLMXMUL3X3(Rzth, Mx3x3Tmp, R_G_P);
    RVLMXMUL3X3(Ryal, R_G_P, pose_G_B.R);
    RVLSET3VECTOR(pose_G_B.t, -ks[4] * cal, -sqrt(ks[5] * ks[5] - ks[4] * ks[4]), ks[4] * sal);
}

void TestFeasibleRobotPoseInvKinematics(
    Pose3D pose_G_B_,
    float *ks_)
{
    float ph_;
    float th_;
    float ps_;
    float al_;
    float rh_;
    float r_;
    al_ = atan2(pose_G_B_.t[2], -pose_G_B_.t[0]);
    rh_ = sqrt(pose_G_B_.t[0] * pose_G_B_.t[0] + pose_G_B_.t[2] * pose_G_B_.t[2]);
    r_ = sqrt(rh_ * rh_ + pose_G_B_.t[1] * pose_G_B_.t[1]);
    float R_P_B_[9];
    float cal_ = cos(al_);
    float sal_ = sin(al_);
    RVLROTY(cal_, sal_, R_P_B_);
    float R_G_P_[9];
    RVLMXMUL3X3T1(R_P_B_, pose_G_B_.R, R_G_P_);
    float cph_ = RVLMXEL(R_G_P_, 3, 2, 2);
    ph_ = -acos(cph_);
    float sph_ = sin(ph_);
    if (sph_ > -1e-7 && sph_ < 1e-7)
    {
        printf("Out of range!\n\n");
        return;
    }
    else
    {
        th_ = atan2(RVLMXEL(R_G_P_, 3, 1, 2) / sph_, RVLMXEL(R_G_P_, 3, 0, 2) / sph_);
        ps_ = atan2(RVLMXEL(R_G_P_, 3, 2, 1) / sph_, -RVLMXEL(R_G_P_, 3, 2, 0) / sph_);
    }
    ks_[0] = ph_;
    ks_[1] = th_;
    ks_[2] = ps_;
    ks_[3] = al_;
    ks_[4] = rh_;
    ks_[5] = r_;
}

#ifdef RVLVTK
void SimpleRobotVisualization(
    Visualizer *pVisualizer,
    DDManipulator *pManipulator,
    Pose3D pose_B_0,
    Pose3D pose_B_W,
    Pose3D pose_G_B,
    int *kSphere,
    int nG_)
{
    std::vector<int> nullPath;
    Array<float> doorStates;
    doorStates.n = 0;
    pManipulator->Visualize(NULL, &nullPath, doorStates);
    Array<Point> vertices;
    vertices.n = 4;
    vertices.Element = new Point[vertices.n];
    float *visP = vertices.Element[0].P;
    RVLCOPY3VECTOR(pManipulator->robot.pose_0_W.t, visP);
    float *visP_ = vertices.Element[1].P;
    float V3Tmp[3];
    RVLSET3VECTOR(V3Tmp, 0.0f, 0.0f, pManipulator->robot.d[0]);
    RVLSUM3VECTORS(pManipulator->robot.pose_0_W.t, V3Tmp, visP_);
    visP = vertices.Element[2].P;
    Pose3D pose_G_W;
    RVLCOMPTRANSF3D(pose_B_W.R, pose_B_W.t, pose_G_B.R, pose_G_B.t, pose_G_W.R, pose_G_W.t);
    RVLCOPY3VECTOR(pose_G_W.t, visP);
    RVLDIF3VECTORS(visP_, visP, V3Tmp);
    // printf("r=%f\n", sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp)));
    visP_ = vertices.Element[3].P;
    RVLCOPYCOLMX3X3(pose_G_W.R, 2, V3Tmp);
    float fTmp = pManipulator->robot.d[5] + pManipulator->robot.pose_TCP_6.t[2];
    RVLSCALE3VECTOR(V3Tmp, fTmp, V3Tmp);
    RVLSUM3VECTORS(visP, V3Tmp, visP_);
    Array<Pair<int, int>> lines;
    lines.n = 3;
    lines.Element = new Pair<int, int>[lines.n];
    lines.Element[0].a = 0;
    lines.Element[0].b = 1;
    lines.Element[1].a = 1;
    lines.Element[1].b = 2;
    lines.Element[2].a = 2;
    lines.Element[2].b = 3;
    uchar red[] = {255, 0, 0};
    vtkSmartPointer<vtkActor> actor = pVisualizer->DisplayLines(vertices, lines, red, 2.0f);
    std::vector<vtkSmartPointer<vtkActor>> robotActors;
    Pose3D pose_G_orig_G = pManipulator->robot.pose_TCP_6;
    pose_G_orig_G.t[2] += pManipulator->robot.d[5];
    Pose3D pose_G_0;
    RVLCOMPTRANSF3D(pose_B_0.R, pose_B_0.t, pose_G_B.R, pose_G_B.t, pose_G_0.R, pose_G_0.t);
    Pose3D pose_G_Orig_0;
    RVLCOMPTRANSF3D(pose_G_0.R, pose_G_0.t, pose_G_orig_G.R, pose_G_orig_G.t, pose_G_Orig_0.R, pose_G_Orig_0.t);
    Array<int> visSpheres;
    visSpheres.n = nG_ + 1;
    visSpheres.Element = new int[visSpheres.n];
    visSpheres.Element[0] = pManipulator->tool_contact_spheres.Element[0];
    for (int k = 1; k < visSpheres.n; k++)
        visSpheres.Element[k] = kSphere[k - 1];
    pManipulator->VisualizeTool(pose_G_Orig_0, &robotActors, false, &visSpheres);
    pVisualizer->Run();

    delete[] vertices.Element;
    delete[] lines.Element;
    delete[] visSpheres.Element;
}
#endif

// This macro should be moved to RVL3DTools.h.

#define RVLTRANSF3PLANE(NSrc, dSrc, R, t, NTgt, dTgt) \
    {                                                 \
        RVLMULMX3X3TVECT(R, NSrc, NTgt);              \
        dTgt = dSrc - RVLDOTPRODUCT3(NSrc, t);        \
    }

#define RVLMOTION_NUM_CONTACT_CONSTRAINTS 6
// #define RVLMOTION_FEASIBLEROBOTPOSE_ROBOT_TOOL_ORIENTATION_CONSTRAINTS

void TestFeasibleRobotPose(
    DDManipulator *pManipulator,
    Visualizer *pVisualizer)
{
    // Move the tool RF to the intersection point of the last two joint axes.

    int nG = pManipulator->tool_sample_spheres.n;
    float *cG = new float[3 * nG];
    float *cG_ = cG;
    int k;
    MOTION::Sphere *pSphere;
    float *cGOrig;
    for (k = 0; k < nG; k++, cG_ += 3)
    {
        pSphere = pManipulator->tool_sample_spheres.Element + k;
        cGOrig = pSphere->c.Element;
        RVLTRANSF3(cGOrig, pManipulator->robot.pose_TCP_6.R, pManipulator->robot.pose_TCP_6.t, cG_);
        cG_[2] += pManipulator->robot.d[5];
    }
    float PRTCP_G[3];
    RVLTRANSF3(pManipulator->PRTCP_G, pManipulator->robot.pose_TCP_6.R, pManipulator->robot.pose_TCP_6.t, PRTCP_G);
    PRTCP_G[2] += pManipulator->robot.d[5];

    // Order of evaluation of the tool sample spheres.

    int kSphere[] = {16, 13, 10, 12, 21, 18, 19, 20, 22, 14, 15, 11, 2, 5, 6, 8, 9, 0, 1, 3, 4, 7};
    int nG_ = 22;
    // int nG_ = 1;

    // Contact constraints params.

    int iContactFace[] = {0, 0, 3, 3, 4, 4};
    float contactPlaneSign[] = {1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f};
    float contactPlaneOffset[RVLMOTION_NUM_CONTACT_CONSTRAINTS];
    memset(contactPlaneOffset, 0, RVLMOTION_NUM_CONTACT_CONSTRAINTS * sizeof(float));
    contactPlaneOffset[1] = 0.04f;
    contactPlaneOffset[3] = pManipulator->dd_sz;
    contactPlaneOffset[5] = 0.5f * pManipulator->dd_sy;

    // T_W_B.

    Pose3D pose_B_0;
    RVLROTX(0.0f, (-1.0f), pose_B_0.R);
    RVLSET3VECTOR(pose_B_0.t, 0.0f, 0.0f, pManipulator->robot.d[0]);
    Pose3D pose_B_W;
    RVLCOMPTRANSF3D(pManipulator->robot.pose_0_W.R, pManipulator->robot.pose_0_W.t, pose_B_0.R, pose_B_0.t, pose_B_W.R, pose_B_W.t);
    Pose3D pose_W_B;
    RVLINVTRANSF3D(pose_B_W.R, pose_B_W.t, pose_W_B.R, pose_W_B.t);

    // Robot params.

    float ksmin[6];
    float ksmax[6];
    ksmin[0] = -0.75f * PI;
    ksmax[0] = -0.25f * PI;
    ksmin[1] = -0.5f * PI;
    ksmax[1] = 0.0f * PI;
    ksmin[2] = -0.5f * PI;
    ksmax[2] = 0.5f * PI;
    ksmin[3] = -PI;
    ksmax[3] = PI;
    ksmin[4] = 0.2f;
    ksmax[4] = 0.7f;
    ksmin[5] = 0.2f;
    ksmax[5] = 0.8f;
    float qLinearSpace = 0.25f * PI;
    // float qLinearSpace = 0.125f * PI;
    float maxs = 0.3f;
    float NRobotNear_B[3];
    RVLSET3VECTOR(NRobotNear_B, 1.0f, 0.0f, 0.0f);
    float dRobotNear_B = -0.2f;
    float NRobotLow_B[3];
    RVLSET3VECTOR(NRobotLow_B, 0.0f, 1.0f, 0.0f);
    float dRobotLow_B = -0.4f;

    // General purpose auxiliary variables.

    int i, j;
    int i_, j_, k_;
    float fTmp;
    float V3Tmp[3];

    // Environment.

    int envFaceMap[15];
    envFaceMap[0] = 30;
    envFaceMap[1] = 37;
    envFaceMap[2] = 41;
    envFaceMap[3] = 32;
    envFaceMap[4] = 34;
    envFaceMap[5] = 42;
    envFaceMap[6] = 40;
    envFaceMap[7] = 43;
    envFaceMap[8] = 33;
    envFaceMap[9] = 5;
    envFaceMap[10] = 27;
    envFaceMap[11] = 28;
    envFaceMap[12] = 23;
    envFaceMap[13] = 24;
    envFaceMap[14] = 22;
    int nEnv = 15;
    float *NEnv = new float[nEnv * 3];
    float *dEnv = new float[nEnv];
    float *NVN_;
    float *NEnv_ = NEnv;
    for (i = 0; i < nEnv; i++, NEnv_ += 3)
    {
        NVN_ = pManipulator->pVNEnv->featureArray.Element[envFaceMap[i]].N;
        RVLTRANSF3PLANE(NVN_, pManipulator->dVNEnv[envFaceMap[i]], pose_B_W.R, pose_B_W.t, NEnv_, dEnv[i]);
        RVLNEGVECT3(NEnv_, NEnv_);
        dEnv[i] = -dEnv[i];
    }
    int nCells = 11;
    int nCellFaces[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4};
    int cellIdxMem[] = {
        0, 9,
        1, 9,
        2, 9,
        3, 9,
        4, 9,
        5, 9,
        6, 9,
        7, 9,
        8, 9,
        0, 12, 13, 14,
        0, 10, 11, 14};
    Array<Array<int>> cells;
    cells.n = nCells;
    cells.Element = new Array<int>[cells.n];
    int iCell;
    int cellIdxPtr = 0;
    int maxnCellFaces = 0;
    for (iCell = 0; iCell < nCells; iCell++)
    {
        cells.Element[iCell].n = nCellFaces[iCell];
        cells.Element[iCell].Element = cellIdxMem + cellIdxPtr;
        cellIdxPtr += nCellFaces[iCell];
        if (nCellFaces[iCell] > maxnCellFaces)
            maxnCellFaces = nCellFaces[iCell];
    }

    // Visualize cells.

    // std::vector<int> nullPath;
    // Array<float> doorStates;
    // doorStates.n = 0;
    // pManipulator->Visualize(NULL, &nullPath, doorStates);

    // Box<float> bbox;
    // pManipulator->pVNEnv->BoundingBox(pManipulator->dVNEnv, bbox);
    // float resolution = 0.02f * BoxSize(&bbox);
    // ExpandBox<float>(&bbox, 10.0f * resolution);
    // float minAxis[3], maxAxis[3];
    // RVLSET3VECTOR(minAxis, bbox.minx, bbox.miny, bbox.minz);
    // RVLSET3VECTOR(maxAxis, bbox.maxx, bbox.maxy, bbox.maxz);
    // int iAxis, iAxis1, iAxis2;
    // int nAxis[3];
    // float maxsAxis[3];
    // for (iAxis = 0; iAxis < 3; iAxis++)
    //{
    //     maxsAxis[iAxis] = maxAxis[iAxis] - minAxis[iAxis];
    //     nAxis[iAxis] = (int)floor(maxsAxis[iAxis] / resolution) + 1;
    // }
    // int iTmp;
    // int axIdx[3];
    // RVLSORT3ASCEND(nAxis, axIdx, iTmp);
    // Array<Point> visPts;
    // visPts.Element = new Point[nAxis[axIdx[1]] * nAxis[axIdx[2]] * 2 * 3];
    // Point* pPt;
    // float P[3];
    // float *w = new float[maxnCellFaces];
    // int iFace;
    // Array<int>* pCell;
    // float signSDF;
    // float *eCF = new float[maxnCellFaces];
    // memset(eCF, 0, maxnCellFaces * sizeof(float));
    // float SDF, SDFNext;
    // float s, s0;
    // float sSurface = 0.0f;
    // bool bSurface;
    // uchar green[] = {0, 255, 0};
    // vtkSmartPointer<vtkActor> visPtActor;
    // int k, l;
    // bool bFirst;
    // int iFaceMax, iFaceMaxNext;
    // float dw;
    // float mins;
    // for (iCell = 0; iCell < nCells; iCell++)
    //{
    //     pCell = cells.Element + iCell;
    //     pPt = visPts.Element;
    //     for (iAxis = 0; iAxis < 3; iAxis++)
    //     {
    //         float axis[3];
    //         RVLNULL3VECTOR(axis);
    //         axis[iAxis] = 1.0f;
    //         iAxis1 = (iAxis + 1) % 3;
    //         iAxis2 = (iAxis + 2) % 3;
    //         visPts.n = 0;
    //         for (k = 0; k < pCell->n; k++)
    //         {
    //             iFace = pCell->Element[k];
    //             NEnv_ = NEnv + 3 * iFace;
    //             w[k] = RVLDOTPRODUCT3(NEnv_, axis);
    //         }
    //         for (j = 0; j < nAxis[iAxis2] && pPt - visPts.Element < 10000; j++)
    //             for (i = 0; i < nAxis[iAxis1] && pPt - visPts.Element < 10000; i++)
    //             {
    //                 if (i == 56 && j == 52)
    //                     int debug = 0;
    //                 P[iAxis] = minAxis[iAxis];
    //                 P[iAxis1] = (float)i * resolution + minAxis[iAxis1];
    //                 P[iAxis2] = (float)j * resolution + minAxis[iAxis2];
    //                 iFaceMax = -1;
    //                 for (k = 0; k < pCell->n; k++)
    //                 {
    //                     iFace = pCell->Element[k];
    //                     NEnv_ = NEnv + 3 * iFace;
    //                     eCF[k] = RVLDOTPRODUCT3(NEnv_, P) - dEnv[iFace];
    //                     if (iFaceMax < 0 || eCF[k] > SDF)
    //                     {
    //                         SDF = eCF[k];
    //                         iFaceMax = k;
    //                     }
    //                 }
    //                 while (true)
    //                 {
    //                     iFaceMaxNext = -1;
    //                     for (k = 0; k < pCell->n; k++)
    //                     {
    //                         if (k == iFaceMax)
    //                             continue;
    //                         dw = w[k] - w[iFaceMax];
    //                         if (dw < 1e-6)
    //                             continue;
    //                         s = -(eCF[k] - eCF[iFaceMax]) / dw;
    //                         if (s <= 0.0f)
    //                             continue;
    //                         if (iFaceMaxNext < 0 || s < mins)
    //                         {
    //                             mins = s;
    //                             iFaceMaxNext = k;
    //                         }
    //                     }
    //                     if (iFaceMaxNext < 0)
    //                         break;
    //                     SDFNext = eCF[iFaceMax] + mins * w[iFaceMax];
    //                     if (SDFNext * SDF < 0.0f)
    //                     {
    //                         s = -eCF[iFaceMax] / w[iFaceMax];
    //                         if (s <= maxsAxis[iAxis])
    //                         {
    //                             RVLSCALE3VECTOR(axis, s, V3Tmp);
    //                             RVLSUM3VECTORS(P, V3Tmp, pPt->P);
    //                             pPt++;
    //                         }
    //                     }
    //                     iFaceMax = iFaceMaxNext;
    //                     SDF = SDFNext;
    //                 }
    //                 if (w[iFaceMax] * SDF < 0.0f)
    //                 {
    //                     s = -eCF[iFaceMax] / w[iFaceMax];
    //                     if (s <= maxsAxis[iAxis])
    //                     {
    //                         RVLSCALE3VECTOR(axis, s, V3Tmp);
    //                         RVLSUM3VECTORS(P, V3Tmp, pPt->P);
    //                         pPt++;
    //                     }
    //                 }
    //             }
    //     }
    //     visPts.n = pPt - visPts.Element;
    //     visPtActor = pVisualizer->DisplayPointSet<float, Point>(visPts, green, 6);
    //     pVisualizer->Run();
    //     pVisualizer->renderer->RemoveViewProp(visPtActor);
    // }

    // Initial state.

    float ph = -0.3f;
    float th = 0.2f;
    float ps = -1.0f;
    float al = 0.0f;
    float rh = 0.8f;
    float r = 1.0f;
    float N_B[3];
    RVLSET3VECTOR(N_B, SQRT0_5, SQRT0_5, 0.0f);
    float d_B = -0.50f;

    // Constraint memory space:
    //
    // 0 - 344: environment
    // 345: robot workspace
    // //346 - 347: robot tool orientation
    // 346 - 347: robot linear
    // 348 - 353: contact
    // 354 - 359: rotation linear space

    int nEnvTotal = nG * nEnv;
    int iRobot = nEnvTotal;
    int iRobotWorkspace = iRobot;
    int nRobotWorkspace = 1;
    int iRobotToolOrient = iRobot + nRobotWorkspace;
    int nRobotToolOrient = 0;
#ifdef RVLMOTION_FEASIBLEROBOTPOSE_ROBOT_TOOL_ORIENTATION_CONSTRAINTS
    int nRobotNLin = 6;
#else
    int nRobotNLin = nRobotWorkspace + nRobotToolOrient;
#endif
    int iRobotLin = iRobot + nRobotNLin;
    int nRobotLin = 2;
    int nRobot = nRobotNLin + nRobotLin;
    int iContact = iRobot + nRobot;
    int nContact = RVLMOTION_NUM_CONTACT_CONSTRAINTS;
    int iRotLin = iContact + nContact;
    int nRotLin = 6;
    int iR = iRotLin;
    int iContactPlane = iR + 1;
    // int nCommon = nRobot + nContact + nRotLin;
    int nCommonLin = nRobotLin + nContact;
    int nCommon = nCommonLin + nRobotNLin;
    int mTotal = nEnvTotal + nCommon;
    float *A = new float[mTotal * 6];
    float *b = new float[mTotal];
    float *ARobot = A + iRobot * 6;
    float *bRobot = b + iRobot;
    float *AContact = A + iContact * 6;
    float *bContact = b + iContact;
    float *ARotLin = A + iRotLin * 6;
    float *bRotLin = b + iRotLin;
    float AC[6];
    float bC;

    // Create solver.

    Solver solver;
    int n = 6;
    solver.Create(mTotal, n);

    // Main loop.

    float x0[6];
    RVLNULL3VECTOR(x0);
    float *t0 = x0 + 3;
    RVLNULL3VECTOR(t0);
    float x[6];
    float xPrev[6];
    float *t = x + 3;
    float *a;
    float *dks = new float[6];
    float *dks_ = new float[6];
    float *ks_ = new float[6];
    float *ks__ = new float[6];
    float *ex = new float[mTotal];
    float *qRobotNear = A + iRobotLin * 6;
    RVLNULL3VECTOR(qRobotNear);
    float *qRobotLow = A + (iRobotLin + 1) * 6;
    RVLNULL3VECTOR(qRobotLow);
    bool bCenter;
    int it;
    float ph_;
    float th_;
    float ps_;
    float al_;
    float rh_;
    float r_;
    float eEnv;
    float ks[6];
    int *J = new int[nG];
    int u;
    float R_G_P[9];
    Pose3D pose_G_B;
    float *NEnvG = new float[nEnv * 3];
    float *dEnvG = new float[nEnv];
    float *NEnvG_;
    float *at;
    float Z[3 * 18];
    RVL::CreateConvexTemplate18(Z);
    Pose3D pose_G_B_SP[18 * 8];
    Pose3D *pPose_G_B_SP = pose_G_B_SP;
    float *R_G_B;
    float *t_G_B;
    float *Z_ = Z;
    float R_B_G0[9];
    float *X_G0_B = R_B_G0;
    float *Y_G0_B = R_B_G0 + 3;
    float *Z_G0_B = R_B_G0 + 6;
    float R_G0_B[9];
    float t_D_B[3];
    RVLTRANSF3(pManipulator->pose_DD_S.t, pose_W_B.R, pose_W_B.t, t_D_B);
    float R_G_G0[9];
    float qz;
    float dqz = 2.0f * PI / (float)8;
    float cs, sn;
    for (i = 0; i < 18; i++, Z_ += 3)
    {
        RVLCOPY3VECTOR(Z_, Z_G0_B);
        RVLORTHOGONAL3(Z_G0_B, X_G0_B, i_, j_, k_, fTmp);
        RVLCROSSPRODUCT3(Z_G0_B, X_G0_B, Y_G0_B);
        for (j = 0; j < 8; j++, pPose_G_B_SP++)
        {
            qz = (float)j * dqz;
            cs = cos(qz);
            sn = sin(qz);
            RVLROTZ(cs, sn, R_G_G0);
            R_G_B = pPose_G_B_SP->R;
            RVLMXMUL3X3T1(R_B_G0, R_G_G0, R_G_B);
            t_G_B = pPose_G_B_SP->t;
            RVLMULMX3X3VECT(R_G_B, PRTCP_G, V3Tmp);
            RVLDIF3VECTORS(t_D_B, V3Tmp, t_G_B);
        }
    }
    Array<int> S;
    S.Element = new int[nG * maxnCellFaces + nCommon];
    S.n = 0;
    // for (u = 0; u < nRotLin; u++)
    //     S.Element[S.n++] = iRotLin + u;
    // for (u = 0; u < nRobotNLin; u++)
    //     S.Element[S.n++] = iRobot + u;
    for (u = 0; u < nRobotLin; u++)
        S.Element[S.n++] = iRobotLin + u;
    for (u = 0; u < nContact; u++)
        S.Element[S.n++] = iContact + u;
    // Array<int> S3;
    // S3.Element = S.Element + nRotLin;
    solver.SetLinearConstraints(A, b);
    solver.SetRadiusConstraint(iRobotWorkspace, ksmax[5]);
    cv::Mat cvM(6, 6, CV_32FC1);
    float *M = (float *)(cvM.data);
    memset(M, 0, 6 * 6 * sizeof(float));
    cv::Mat cvInvM(6, 6, CV_32FC1);
    float *invM = (float *)(cvInvM.data);
    float Mx3x3Tmp[9];
    float Mx3x3Tmp2[9];
    // memset(ARobot + 2 * 6 * 6, 0, 6 * 6 * sizeof(float));
    memset(ARotLin, 0, 6 * 6 * sizeof(float));
    int iOrient;
    float *eDebug = new float[mTotal];
    MOTION::Sphere *pContactSphere;
    // bool bNonLinearConstraintsSatisfied;
    float exmax;
    bool bFeasible;
    bool bFeasible_;
    float dq2;
    uchar solverResult;
    Array<int> L;
    int LMem[3];
    L.Element = LMem;
    L.n = 3;
    RVLSET3VECTOR(L.Element, 0, 1, 2);
    solver.SetLinearRegion(L, qLinearSpace);
    // iOrient = 8 * 9 + 4;
    // iOrient = 8 * 3 + 4;
    int iSphere_;
    Array<int> cell;
    Pose3D *pose_G_B_mem = new Pose3D[nG];
    int k0;
    float *E_ = new float[mTotal];
    float E;
    float EPrev;
    int i__;
    int iE;
    Pose3D pose_G_B_;
    bool bCostReduced;
    float dx;
    // bool bRConstraint;
    // int nFeasiblePoses = 0;
    for (iOrient = 120; iOrient < 18 * 8; iOrient++, pPose_G_B_SP++)
    {
        printf("iOrient=%d\n", iOrient);
        // if (iOrient == 8)
        //     int debug = 0;
        pose_G_B_mem[0] = pose_G_B_SP[iOrient];
        memset(J, 0, nG * sizeof(int));
        iSphere_ = -1;
        // J[2] = 1;
        // J[0] = 1;
        // bRConstraint = false;
        while (true)
        {
            pose_G_B = pose_G_B_mem[iSphere_ + 1];

            S.n = nCommonLin;
            for (k_ = 0; k_ <= iSphere_; k_++)
            {
                k = kSphere[k_];
                j = J[k_];
                cell = cells.Element[j];
                k0 = k * nEnv;
                for (i_ = 0; i_ < cell.n; i_++)
                    S.Element[S.n++] = k0 + cell.Element[i_];
                printf("%d ", J[k_]);
            }
            printf("\n");
            // S3.n = S.n - nRotLin;

            for (it = 0; it < 100; it++)
            {
                pose_G_B_ = pose_G_B;
                do
                {
                    // Update pose_G_B.

                    if (it > 0)
                    {
                        RotateRdR(pose_G_B_.R, x, pose_G_B.R);
                        RVLTRANSF3(t, pose_G_B_.R, pose_G_B_.t, pose_G_B.t);
                    }

                    // Environment constraints.

                    NEnv_ = NEnv;
                    NEnvG_ = NEnvG;
                    for (j = 0; j < nEnv; j++, NEnv_ += 3, NEnvG_ += 3)
                    {
                        RVLMULMX3X3TVECT(pose_G_B.R, NEnv_, NEnvG_);
                        dEnvG[j] = dEnv[j] - RVLDOTPRODUCT3(NEnv_, pose_G_B.t);
                    }
                    u = 0;
                    pSphere = pManipulator->tool_sample_spheres.Element;
                    cG_ = cG;
                    for (k = 0; k < nG; k++, pSphere++, cG_ += 3)
                    {
                        NEnvG_ = NEnvG;
                        for (j = 0; j < nEnv; j++, NEnvG_ += 3, u++)
                        {
                            a = A + n * u;
                            RVLCROSSPRODUCT3(cG_, NEnvG_, a);
                            at = a + 3;
                            RVLCOPY3VECTOR(NEnvG_, at);
                            b[u] = dEnvG[j] - RVLDOTPRODUCT3(NEnvG_, cG_) - pSphere->r;
                        }
                    }

                    // Robot constraints.

                    TestFeasibleRobotPoseInvKinematics(pose_G_B, ks);
                    ph = ks[0];
                    th = ks[1];
                    ps = ks[2];
                    al = ks[3];
                    rh = ks[4];
                    r = ks[5];
#ifdef RVLMOTION_FEASIBLEROBOTPOSE_ROBOT_TOOL_ORIENTATION_CONSTRAINTS
                    float cph = cos(ph);
                    float sph = sin(ph);
                    float cth = cos(th);
                    float sth = sin(th);
                    float cps = cos(ps);
                    float sps = sin(ps);
                    float cal = cos(al);
                    float sal = sin(al);
                    RVLMXEL(M, 6, 0, 0) = sps;
                    RVLMXEL(M, 6, 0, 1) = -sph * cps;
                    RVLMXEL(M, 6, 1, 0) = cps;
                    RVLMXEL(M, 6, 1, 1) = sph * sps;
                    RVLMXEL(M, 6, 2, 1) = cph;
                    RVLMXEL(M, 6, 2, 2) = 1.0f;
                    RVLMXEL(M, 6, 0, 3) = cth * sps + cph * sth * cps;
                    RVLMXEL(M, 6, 1, 3) = cth * cps - cph * sth * sps;
                    RVLMXEL(M, 6, 2, 3) = sph * sth;
                    fTmp = sqrt(r * r - rh * rh);
                    RVLMXEL(Mx3x3Tmp, 3, 0, 0) = rh * sal;
                    RVLMXEL(Mx3x3Tmp, 3, 0, 1) = -cal;
                    RVLMXEL(Mx3x3Tmp, 3, 0, 2) = 0.0f;
                    RVLMXEL(Mx3x3Tmp, 3, 1, 0) = 0.0f;
                    RVLMXEL(Mx3x3Tmp, 3, 1, 1) = rh / fTmp;
                    RVLMXEL(Mx3x3Tmp, 3, 1, 2) = -r / fTmp;
                    RVLMXEL(Mx3x3Tmp, 3, 2, 0) = rh * cal;
                    RVLMXEL(Mx3x3Tmp, 3, 2, 1) = sal;
                    RVLMXEL(Mx3x3Tmp, 3, 2, 2) = 0.0f;
                    RVLMXMUL3X3T1(pose_G_B.R, Mx3x3Tmp, Mx3x3Tmp2);
                    RVLCOPY3BLOCKTOMX(Mx3x3Tmp2, M, 3, 3, 6);
                    cv::invert(cvM, cvInvM);

                    // memcpy(ARobot, invM, 6 * 6 * sizeof(float));
                    // for (i = 0; i < 6 * 6; i++)
                    //     ARobot[6 * 6 + i] = -ARobot[i];
                    // RVLMXEL(ARobot, 6, 2 * 6, 0) = RVLMXEL(ARobot, 6, 2 * 6 + 1, 1) = RVLMXEL(ARobot, 6, 2 * 6 + 2, 2) = 1.0f;
                    // RVLMXEL(ARobot, 6, 2 * 6 + 3, 0) = RVLMXEL(ARobot, 6, 2 * 6 + 4, 1) = RVLMXEL(ARobot, 6, 2 * 6 + 5, 2) = -1.0f;
                    memcpy(ARobot, invM, 3 * 6 * sizeof(float));
                    for (i = 0; i < 3 * 6; i++)
                        ARobot[3 * 6 + i] = -ARobot[i];
                    // for (i = 0; i < 6; i++)
                    //{
                    //     bRobot[i] = ksmax[i] - ks[i];
                    //     bRobot[6 + i] = -(ksmin[i] - ks[i]);
                    // }
                    // float* bRobot_ = bRobot + 2 * 6;
                    for (i = 0; i < 3; i++)
                    {
                        bRobot[i] = ksmax[i] - ks[i];
                        bRobot[3 + i] = -(ksmin[i] - ks[i]);
                    }
#endif

                    float *NRobotNear_G = qRobotNear + 3;
                    RVLMULMX3X3TVECT(pose_G_B.R, NRobotNear_B, NRobotNear_G);
                    b[iRobotLin] = dRobotNear_B - RVLDOTPRODUCT3(NRobotNear_B, pose_G_B.t);

                    float *NRobotLow_G = qRobotLow + 3;
                    RVLMULMX3X3TVECT(pose_G_B.R, NRobotLow_B, NRobotLow_G);
                    b[iRobotLin + 1] = dRobotLow_B - RVLDOTPRODUCT3(NRobotLow_B, pose_G_B.t);

                    // Contact Constraints.

                    k = pManipulator->tool_contact_spheres.Element[0];
                    pContactSphere = pManipulator->tool_sample_spheres.Element + k;
                    cG_ = cG + 3 * k;
                    a = AContact;
                    u = 0;
                    for (i = 0; i < nContact; i++, a += n, u++)
                    {
                        j = iContactFace[i];
                        NEnvG_ = NEnvG + 3 * j;
                        at = a + 3;
                        if (contactPlaneSign[i] > 0.0f)
                        {
                            RVLCROSSPRODUCT3(cG_, NEnvG_, a);
                            RVLCOPY3VECTOR(NEnvG_, at);
                        }
                        else
                        {
                            RVLCROSSPRODUCT3(NEnvG_, cG_, a);
                            RVLNEGVECT3(NEnvG_, at);
                        }
                        bContact[u] = contactPlaneSign[i] * (dEnvG[j] - RVLDOTPRODUCT3(NEnvG_, cG_)) + contactPlaneOffset[i] - pContactSphere->r;
                    }
                    // j = 3;
                    // NEnvG_ = NEnvG + 3 * j;
                    // RVLCROSSPRODUCT3(NEnvG_, cG_, a);
                    // at = a + 3;
                    // RVLNEGVECT3(NEnvG_, at);
                    // bContact[u] = -(dEnvG[j] - RVLDOTPRODUCT3(NEnvG_, cG_)) - pContactSphere->r;
                    // a += n;
                    // u++;
                    // j = 4;
                    // NEnvG_ = NEnvG + 3 * j;
                    // RVLCROSSPRODUCT3(NEnvG_, cG_, a);
                    // at = a + 3;
                    // RVLNEGVECT3(NEnvG_, at);
                    // bContact[u] = -(dEnvG[j] - RVLDOTPRODUCT3(NEnvG_, cG_)) - pContactSphere->r;
                    // a += n;
                    // u++;
                    // RVLCROSSPRODUCT3(cG_, NEnvG_, a);
                    // at = a + 3;
                    // RVLCOPY3VECTOR(NEnvG_, at);
                    // bContact[u] = dEnvG[j] - RVLDOTPRODUCT3(NEnvG_, cG_) + 0.5f * pManipulator->dd_sy - pContactSphere->r;
                    // RVLCROSSPRODUCT3(cG_, NEnvG, AC);
                    // at = AC + 3;
                    // RVLCOPY3VECTOR(NEnvG, at);
                    RVLDOTPRODUCT(AC, AC, n, fTmp, i_);
                    fTmp = 1.0f / sqrt(fTmp);
                    RVLSCALEVECTOR(AC, fTmp, AC, n, i_);
                    bC = fTmp * (dEnvG[0] - RVLDOTPRODUCT3(NEnvG, cG_) - pContactSphere->r);

                    // Rotation linearity constraints.

                    RVLMXEL(ARotLin, 6, 0, 0) = RVLMXEL(ARotLin, 6, 1, 1) = RVLMXEL(ARotLin, 6, 2, 2) = 1.0f;
                    RVLMXEL(ARotLin, 6, 3, 0) = RVLMXEL(ARotLin, 6, 4, 1) = RVLMXEL(ARotLin, 6, 5, 2) = -1.0f;
                    float *bRotLin_ = bRotLin;
                    RVLSET3VECTOR(bRotLin_, qLinearSpace, qLinearSpace, qLinearSpace);
                    bRotLin_ += 3;
                    RVLSET3VECTOR(bRotLin_, qLinearSpace, qLinearSpace, qLinearSpace);

                    // Is solution found?

                    // k = pManipulator->tool_contact_spheres.Element[0];
                    // cG_ = cG + 3 * k;
                    // RVLTRANSF3(cG_, pose_G_B.R, pose_G_B.t, V3Tmp);
                    // E = E_[iContactPlane] = RVLDOTPRODUCT3(NEnv, V3Tmp) - dEnv[0] + pContactSphere->r;
                    // iE = iContactPlane;
                    E = 0.0f;
                    iE = -1;
                    // if (bRConstraint)
                    {
                        E_[iRobotWorkspace] = ks[5] - ksmax[5];
                        if (E_[iRobotWorkspace] > E || iE < 0)
                        {
                            E = E_[iRobotWorkspace];
                            iE = iRobotWorkspace;
                        }
                    }
                    // for (i_ = 0; i_ < S3.n; i_++)
                    //{
                    //     i = S3.Element[i_];
                    for (i_ = 0; i_ < S.n; i_++)
                    {
                        i = S.Element[i_];
                        // if (i >= iRobot && i < iRobot + nRobotNLin)
                        //{
                        //     i__ = i - iRobot;
                        //     if (i__ < 3)
                        //         E_[i] = ks[i__] - ksmax[i__];
                        //     else
                        //     {
                        //         i__ -= 3;
                        //         E_[i] = ksmin[i__] - ks[i__];
                        //     }
                        // }
                        // else
                        E_[i] = -b[i];
                        if (E_[i] > E || iE < 0)
                        {
                            E = E_[i];
                            iE = i;
                        }
                    }
                    if (bFeasible = (E <= 1e-6))
                        break;

                    // If cost is not reduced, then decrement x.

                    if (it > 0)
                    {
                        // if (solverResult == RVLSOLVER_FLAGS_SUCCESS)
                        {
                            bCostReduced = (E < EPrev);

                            if (!bCostReduced)
                            {
                                RVLSCALEVECTOR(x, 0.5f, x, 6, i_);
                                RVLDOTPRODUCT(x, x, 6, dx, i_);
                                if (dx < 1e-6)
                                    break;
                            }
                        }
                        // else
                        //{
                        //     bCostReduced = false;
                        //     break;
                        // }
                    }
                    else
                        bCostReduced = true;
                } while (!bCostReduced);

                if (bFeasible || !bCostReduced)
                    break;

                EPrev = E;

                // Only for debugging purpose!!!

                // RVLSET3VECTOR(dks, -0.001f, 0.001f, -0.001f);
                // float* dks2 = dks + 3;
                // RVLSET3VECTOR(dks2, 0.001f, -0.001f, 0.001f);
                // float ks__[6];
                // RVLSUMVECTORS(ks, dks, 6, ks__, i);
                // float R_G_P__[9];
                // Pose3D pose_G_B__;
                // TestFeasibleRobotPoseFwdKinematics(ks__, R_G_P__, pose_G_B__);
                // Pose3D dPose;
                ////RVLMXMUL3X3T1(R_G_P, R_G_P__, dR);
                // RVLCOMPTRANSF3DWITHINV(pose_G_B.R, pose_G_B.t, pose_G_B__.R, pose_G_B__.t, dPose.R, dPose.t, V3Tmp);
                // float u[3];
                // float th__;
                // GetAngleAxis(dPose.R, u, th__);
                // RVLSCALE3VECTOR(u, th__, x);
                // float x__[6];
                // float* t__ = x__ + 3;
                // RVLMULMXVECT(M, dks, 6, 6, x__, i, j, a);

                //

                // ks[0] = ph; ks[1] = th; ks[2] = ps; ks[3] = al; ks[4] = rh; ks[5] = r;

                // Solve Linear problem.

                // solverResult = solver.FeasibleSolution3(A, b, S3, x0, x, exmax, AC, bC, 1, &L, qLinearSpace);
                // solver.SetInequalityConstraints(S3);
                solver.SetInequalityConstraints(S);
                solver.SetPose(pose_G_B);
                // if(bRConstraint)
                //     solver.SetRadiusConstraint(iRobotWorkspace, ksmax[5]);
                solverResult = solver.FeasibleSolution3(x0, x, exmax);
                // Only for debugging purpose!!!
                RVLDISTTOPLANES(A, b, n, S, x, ex, a, fTmp, i_, j_, k_);
                RVLCOPYSELECTED(ex, S, eDebug, i_, j_);
                RVLDOTPRODUCT(AC, x, n, fTmp, i);
                fTmp -= bC;
                //
                if (exmax <= 1e-6)
                {
                    // bRConstraint = true;
                    // float t_B_G[3];
                    // RVLINVTRANSL(pose_G_B.R, pose_G_B.t, t_B_G);
                    ////solverResult = solver.FeasibleSolution2(A, b, S, x, x, AC, 1, t_B_G, ksmax[5], 3);
                    // solverResult = solver.FeasibleSolution2(A, b, S, x, x, NULL, 0, t_B_G, ksmax[5], 3);
                    //// Only for debugging purpose!!!
                    // RVLDISTTOPLANES(A, b, n, S, x, ex, a, fTmp, i_, j_, k_);
                    // RVLCOPYSELECTED(ex, S, eDebug, i_, j_);
                    // RVLDOTPRODUCT(AC, x, n, fTmp, i);
                    // fTmp -= bC;
                    ////
                    if (solverResult == RVLSOLVER_FLAGS_SUCCESS)
                        bFeasible_ = true;
                    else
                    {
                        // RVLDIF3VECTORS(t_B_G, t, V3Tmp);
                        // exmax = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp)) - ksmax[5];
                        bFeasible_ = false;
                    }
                }
                else
                    bFeasible_ = false;
                if (!bFeasible_)
                {
                    if (exmax > E)
                        break;
                    dq2 = RVLDOTPRODUCT3(x, x);
                    if (dq2 < 1e-4)
                        break;
                }

                // If the solution is out of the linear area, then decrement x.

                // bool bDecrementStep;
                // Pose3D pose_G_B_;
                // do
                //{
                //     RVLMULMXVECT(ARobot, x, 6, n, dks, i_, j_, a);
                //     //RVLDIFVECTORS(dks, bRobot, 6, eDebug, i);
                //     //RVLDOTPRODUCT(AC, x, n, fTmp, i);
                //     //fTmp -= bC;
                //     RVLSUMVECTORS(ks, dks, 6, ks__, i);
                //     RotateRdR(pose_G_B.R, x, pose_G_B_.R);
                //     RVLTRANSF3(t, pose_G_B.R, pose_G_B.t, pose_G_B_.t);
                //     TestFeasibleRobotPoseInvKinematics(pose_G_B_, ks_);
                //     for (i = 0; i < 6; i++)
                //         dks_[i] = ks_[i] - ks[i];
                //     for (i = 0; i < 6; i++)
                //         if (ks_[i] < ksmin[i] || ks_[i] > ksmax[i])
                //             break;
                //     bDecrementStep = false;
                //     if (i < 6)
                //         if (dks_[i] * dks[i] < 0.0f)
                //         {
                //             RVLSCALEVECTOR(x, 0.5f, x, 6, i_);
                //             bDecrementStep = true;
                //         }
                // } while (bDecrementStep);
                // for (i = 0; i < 6; i++)
                //     ks[i] = ks_[i];
                // pose_G_B = pose_G_B_;

                // Only for debugging purpose!!!

                // float dR[9];
                // RVLMXMUL3X3T1(pose_G_B.R, pose_G_B_SP[iOrient].R, dR);
                // float rot = acos(RVLROTDIFF(dR));
                // RVLDIF3VECTORS(pose_G_B.t, pose_G_B_SP[iOrient].t, V3Tmp);
                // float transl = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
                // int debug = 0;

                //
            } // Iterations of the optimization procedure.

            if (bFeasible)
            {
                iSphere_++;
                // SimpleRobotVisualization(pVisualizer, pManipulator, pose_B_0, pose_B_W, pose_G_B, kSphere, iSphere_);
                // pVisualizer->renderer->RemoveAllViewProps();
                if (iSphere_ >= nG_)
                {
                    // if (bRConstraint)
                    //     break;
                    // bRConstraint = true;
                    // nFeasiblePoses++;
                    // printf(".");
                    break;
                    // iSphere_--;
                }
                else
                    pose_G_B_mem[iSphere_ + 1] = pose_G_B;
            }
            else
            {
                // if (bRConstraint)
                //{
                //     bRConstraint = false;
                //     solver.ClearRadiusConstraint();
                // }
                for (; iSphere_ >= 0; iSphere_--)
                {
                    J[iSphere_]++;
                    if (J[iSphere_] >= nCells)
                        J[iSphere_] = 0;
                    else
                        break;
                }
                if (iSphere_ < 0)
                    break;
            }
        }
        if (bFeasible)
        {
            // nFeasiblePoses++;
            // printf(".");
            // SimpleRobotVisualization(pVisualizer, pManipulator, pose_B_0, pose_B_W, pose_G_B, kSphere, nG_);
            // pVisualizer->renderer->RemoveAllViewProps();
            break;
        }
        // printf("\n");
    }

    // Visualization.

    printf("r=%f\n", ks[5]);
    SimpleRobotVisualization(pVisualizer, pManipulator, pose_B_0, pose_B_W, pose_G_B, kSphere, nG_);

    //

    delete[] dks;
    delete[] dks_;
    delete[] ks_;
    delete[] ks__;
    delete[] ex;
    delete[] A;
    delete[] b;
    delete[] NEnv;
    delete[] dEnv;
    delete[] cells.Element;
    delete[] J;
    delete[] NEnvG;
    delete[] dEnvG;
    delete[] cG;
    delete[] eDebug;
    delete[] pose_G_B_mem;
    delete[] E_;
    // delete[] visPts.Element;
    // delete[] eCF;
    // delete[] w;
}