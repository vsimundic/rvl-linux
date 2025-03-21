#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "SE3Grid.h"
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
#include "cnpy.h"
#include "vtkNew.h"
#define RVLFCL

#ifdef RVLFCL
#include <vtkAlgorithmOutput.h>
#include <vtkAlgorithm.h>
#include <vtkTransformFilter.h>
// #undef PI // FCL gives an error
// #include "fcl/fcl.h"
// constexpr double PI = 3.14159265358979;
#endif

#ifndef RVLLINUX
#define RVLDDMANIPULATOR_TIME_MESUREMENT
#endif
#define RVLMOTION_DDMANIPULATOR_MULTI_SOLUTION_IK

using namespace RVL;
using namespace MOTION;

DDManipulator::DDManipulator()
{
    resultsFolder = NULL;
    bLog = false;
    pTimer = NULL;

    // Configuration files.

    feasibleToolContactPosesFileName = NULL;
    contactPoseGraphFileName = NULL;
    toolModelDir = NULL;

    // Door model parameters.

    dd_contact_surface_params[0] = dd_contact_surface_params[1] = 0.2f;
    dd_sx = 0.018f;
    dd_sy = 0.3f;
    dd_sz = 0.5f;
    dd_moving_to_static_part_distance = 0.005f;
    dd_opening_direction = 1.0f;
    dd_rx = 0.0f;
    dd_ry = -(0.5f * dd_sy + dd_moving_to_static_part_distance);
    RVLSET3VECTOR(dd_panel_params, dd_sy, dd_sz, dd_sx);
    dd_static_side_width = 0.018f;
    dd_static_depth = 0.3f;
    dd_contact_surface_sampling_resolution = 0.005f;
    bVNPanel = false;

    // Door pose.

    RVLUNITMX3(pose_F_S.R);
    RVLNULL3VECTOR(pose_F_S.t);

    // Environment VN model.

    pVNEnv = NULL;
    VNMClusters.n = 0;
    VNMClusters.Element = NULL;
    dVNEnv = NULL;
    pVNPanel = NULL;
    dVNPanel = NULL;

    // Tool model.

    bDefaultToolModel = true;
    RVLSET3VECTOR(tool_contact_surface_params[0].Element, 0.0, 0.01, 0.0);
    RVLSET3VECTOR(tool_contact_surface_params[1].Element, 0.0, 0.01, -0.02);
    RVLSET3VECTOR(tool_contact_surface_params[2].Element, -0.02, 0.01, 0.0);
    RVLSET3VECTOR(tool_finger_size.Element, 0.02, 0.02, 0.06);
    tool_finger_distance = 0.06; // m
    RVLSET3VECTOR(tool_palm_size.Element, 0.10, 0.02, 0.02);
    tool_wrist_len = 0.21f; // m
    tool_wrist_r = 0.03f;   // m
    tool_len = tool_finger_size.Element[2] + tool_palm_size.Element[2] + tool_wrist_len;
    RVLSET3VECTOR(tool_bounding_sphere.c.Element, 0.0f, 0.0f, -0.5f * tool_len);
    tool_bounding_sphere.r = 0.1537f; // mm
    tool_sample_spheres.n = 0;
    tool_sample_spheres.Element = NULL;
    pToolMesh = NULL;
    RVLSET3VECTOR(PRTCP_G, -0.5f * tool_finger_distance, 0.0f, 0.0f);
    tool_contact_spheres.Element = NULL;

    // Robot.

    RVLSET3VECTOR(robot.pose_TCP_6.t, 0.0f, 0.0f, 0.290f);

    // Path planning.

    bPath3 = false;
    maxnSE3Points = 20000;
    kTemplateEndTol = 0.1f;
    maxSurfaceContactAngle = 45.0f; // deg
    visionTol = 0.01f;              // m
    // minDistanceToAxis = 0.15; // m
    bLock_T_G_DD = false;
    maxnNodesJSPerStep = 10000;
    nodes.Element = NULL;
    graph.NodeMem = NULL;
    rndIdx.Element = NULL;
    posCostMaxDist = 0.03f;
    wPos = 1000.0f;
    maxnIKSolutions = 8;
    JSDistThr = 0.25f * PI;
    pathPosesMem = NULL;
    pathJointsMem = NULL;
    pathMem = NULL;
    pathMemJoints = NULL;
    approachIKSolutionsMem = NULL;
    approachPathMem = NULL;
    selectedNodes.Element = NULL;
    bSelected = NULL;
    contactNode = NULL;

    // Constants.

    RVLNULLMX3X3(pose_A_F.R);
    RVLMXEL(pose_A_F.R, 3, 2, 0) = -1.0f;
    RVLMXEL(pose_A_F.R, 3, 0, 1) = 1.0f;
    RVLMXEL(pose_A_F.R, 3, 1, 2) = -1.0f;
    RVLCOPYMX3X3T(pose_A_F.R, pose_DD_A.R);
    RVLNULL3VECTOR(pose_Arot_A.t);
    csMaxSurfaceContactAngle = cos(DEG2RAD * maxSurfaceContactAngle);

    // Feasible tool contact poses.

    feasibleTCPs.Element = NULL;

    // Local constraints.

    rLocalConstraints = 0.05f;

    // Node buffer.

    nodeBuffMem = NULL;

    // Visualization.

    pVisualizationData = NULL;

    // FCL
    cabinetStaticDirPath = NULL;
    use_fcl = false;
    pCabinetMeshStatic = NULL;
    pCabinetMeshPanel = NULL;
}

DDManipulator::~DDManipulator()
{
    Clear();
}

void DDManipulator::Create(char *cfgFileNameIn)
{
    // Load paramters from a configuration file.

    cfgFileName = cfgFileNameIn;
    CreateParamList();
    paramList.LoadParams(cfgFileNameIn);

    // Pose of the contact surface with respect to A.

    RVLSET3VECTOR(pose_DD_A.t, dd_rx - 0.5f * dd_sx, dd_ry - 0.5f * dd_sy, 0.5f * dd_sz);

    /// Create environment VN model.

    // cluster0 - door panel
    // cluster1 - box
    // cluster2 - storage space
    // cluster3 - box with storage space
    // cluster4 - complete furniture element: box with storage space + panel

    Array2D<float> A;
    A.Element = NULL;
    Array<RECOG::PSGM_::Plane> CT;
    CT.Element = NULL;
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
    Array2D<float> NArrayGnd;
    NArrayGnd.w = 3;
    NArrayGnd.h = 1;
    float NGnd[3];
    RVLSET3VECTOR(NGnd, 0.0f, -1.0f, 0.0f);
    NArrayGnd.Element = NGnd;
    Array<RECOG::PSGM_::Plane> CTNull;
    CTNull.n = 0;

    if (pVNEnv == NULL || pVNPanel == NULL)
    {
        A.w = 3;
        A.h = 18;
        A.Element = new float[A.w * A.h];
        CreateConvexTemplate18(A.Element);
        CT.n = A.h;
        CT.Element = new RECOG::PSGM_::Plane[CT.n];
        RECOG::PSGM_::CreateTemplate(A, CT);
    }

    if (pVNEnv == NULL)
    {
        pVNEnv = new VN;
        pVNEnv->CreateEmpty();
        VNMClusters.n = (bVNPanel ? 4 : 3);
        VNMClusters.Element = new RECOG::VN_::ModelCluster *[VNMClusters.n];
        VNMClusters.Element[0] = pVNEnv->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0); // box
        Pair<int, int> iBetaInterval;
        iBetaInterval.a = 1;
        iBetaInterval.b = 3;
        VNMClusters.Element[1] = pVNEnv->AddModelCluster(1, RVLVN_CLUSTER_TYPE_XTORUS, R, t, 0.49f, 4, 4, iBetaInterval, pMem0);           // storage space
        VNMClusters.Element[2] = pVNEnv->AddModelCluster(2, RVLVN_CLUSTER_TYPE_PLANE, R, t, 0.5f, CTNull, betaInterval, NArrayGnd, pMem0); // ground plane
        if (bVNPanel)
        {
            VNMClusters.Element[3] = pVNEnv->AddModelCluster(3, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0); // panel
            pVNEnv->AddOperation(4, 1, 0, 1, pMem0);
            pVNEnv->AddOperation(5, -1, 2, 4, pMem0);
            pVNEnv->AddOperation(6, -1, 3, 5, pMem0);
            pVNEnv->SetOutput(6);
        }
        else
        {
            pVNEnv->AddOperation(3, 1, 0, 1, pMem0);
            pVNEnv->AddOperation(4, -1, 2, 3, pMem0);
            pVNEnv->SetOutput(4);
        }
        pVNEnv->Create(pMem0);
    }

    if (pVNPanel == NULL)
    {
        pVNPanel = new VN;
        pVNPanel->CreateEmpty();
        pPanelVNMCluster = pVNPanel->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0); // box
        pVNPanel->SetOutput(0);
        pVNPanel->Create(pMem0);
    }

    RVL_DELETE_ARRAY(A.Element);
    RVL_DELETE_ARRAY(CT.Element);

    // Set environment 3D model parameters.

    UpdateFurnitureParams();

    ///

    // Robot.

    robot.pMem0 = pMem0;
    robot.Create(cfgFileNameIn);

    // Tool model.

    if (toolModelDir)
    {
        LoadToolModel(toolModelDir);
        RVL_DELETE_ARRAY(tool_contact_spheres.Element);
        tool_contact_spheres.Element = new int[3];
        RVLSET3VECTOR(tool_contact_spheres.Element, 17, 18, 19);
    }
    if (tool_sample_spheres.n == 0)
    {
        tool_sample_spheres.n = 11;
        RVL_DELETE_ARRAY(tool_sample_spheres.Element);
        tool_sample_spheres.Element = new MOTION::Sphere[tool_sample_spheres.n];
        float tool_sample_sphere_r = 0.5f * tool_finger_size.Element[0];
        float x = 0.5f * (tool_finger_distance + tool_finger_size.Element[0]);
        float z = -7.0f * tool_sample_sphere_r;
        RVLSET3VECTOR(tool_sample_spheres.Element[0].c.Element, -x, 0.0, -tool_sample_sphere_r);
        RVLSET3VECTOR(tool_sample_spheres.Element[1].c.Element, -x, 0.0, -3.0f * tool_sample_sphere_r);
        RVLSET3VECTOR(tool_sample_spheres.Element[2].c.Element, -x, 0.0, -5.0f * tool_sample_sphere_r);
        RVLSET3VECTOR(tool_sample_spheres.Element[3].c.Element, -x, 0.0, -7.0f * tool_sample_sphere_r);
        RVLSET3VECTOR(tool_sample_spheres.Element[4].c.Element, x, 0.0, -tool_sample_sphere_r);
        RVLSET3VECTOR(tool_sample_spheres.Element[5].c.Element, x, 0.0, -3.0f * tool_sample_sphere_r);
        RVLSET3VECTOR(tool_sample_spheres.Element[6].c.Element, x, 0.0, -5.0f * tool_sample_sphere_r);
        RVLSET3VECTOR(tool_sample_spheres.Element[7].c.Element, x, 0.0, -7.0f * tool_sample_sphere_r);
        RVLSET3VECTOR(tool_sample_spheres.Element[8].c.Element, -2.0f * tool_sample_sphere_r, 0.0, z);
        RVLSET3VECTOR(tool_sample_spheres.Element[9].c.Element, 0.0f, 0.0f, z);
        RVLSET3VECTOR(tool_sample_spheres.Element[10].c.Element, 2.0f * tool_sample_sphere_r, 0.0, z);
        int iSphere;
        for (iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++)
            tool_sample_spheres.Element[iSphere].r = tool_sample_sphere_r;
    }
    half_tool_finger_distance = 0.5f * tool_finger_distance;
    tool_len = tool_finger_size.Element[2] + tool_palm_size.Element[2] + tool_wrist_len;

    // Load feasible tool contact poses.

    LoadFeasibleToolContactPoses(feasibleToolContactPosesFileName);

    // Create contact pose graph.

    if (!LoadContactPoseGraph(contactPoseGraphFileName))
        CreateContactPoseGraph(contactPoseGraphFileName);

    // Node buffer.

    nodeBuffMemCapacity = 10000;
    RVL_DELETE_ARRAY(nodeBuffMem);
    nodeBuffMem = new int[nodeBuffMemCapacity];

    // Allocate memory for approach IK solutions and paths.

    RVL_DELETE_ARRAY(approachIKSolutionsMem);
    approachIKSolutionsMem = new MOTION::IKSolution[3 * maxnIKSolutions];
    for (int i = 0; i < 3; i++)
        approachIKSolutions[i].Element = approachIKSolutionsMem + i * maxnIKSolutions;
    approachPathMem = new Pair<int, int>[maxnIKSolutions * maxnIKSolutions];

    // Random indices.

    RVL_DELETE_ARRAY(rndIdx.Element);
    rndIdx.n = 1000000;
    RandomIndices(rndIdx);

    // Solver.

    // solver.Create(tool_sample_spheres.n* pVNEnv->featureArray.n, 6);

    // Constants.

    csMaxSurfaceContactAngle = cos(DEG2RAD * maxSurfaceContactAngle);
}

void DDManipulator::CreateParamList()
{
    paramList.m_pMem = pMem0;
    RVLPARAM_DATA *pParamData;
    paramList.Init();
    pParamData = paramList.AddParam("DDM.FeasibleToolContactPosesFileName", RVLPARAM_TYPE_STRING, &feasibleToolContactPosesFileName);
    pParamData = paramList.AddParam("DDM.ContactPoseGraphFileName", RVLPARAM_TYPE_STRING, &contactPoseGraphFileName);
    pParamData = paramList.AddParam("DDM.ToolModelDirectory", RVLPARAM_TYPE_STRING, &toolModelDir);
    pParamData = paramList.AddParam("DDM.maxnSE3Points", RVLPARAM_TYPE_INT, &maxnSE3Points);
    pParamData = paramList.AddParam("DDM.tool.boundingSphere.x", RVLPARAM_TYPE_FLOAT, tool_bounding_sphere.c.Element);
    pParamData = paramList.AddParam("DDM.tool.boundingSphere.y", RVLPARAM_TYPE_FLOAT, tool_bounding_sphere.c.Element + 1);
    pParamData = paramList.AddParam("DDM.tool.boundingSphere.z", RVLPARAM_TYPE_FLOAT, tool_bounding_sphere.c.Element + 2);
    pParamData = paramList.AddParam("DDM.tool.boundingSphere.r", RVLPARAM_TYPE_FLOAT, &(tool_bounding_sphere.r));
    pParamData = paramList.AddParam("DDM.tool.PRTCP.x", RVLPARAM_TYPE_FLOAT, PRTCP_G);
    pParamData = paramList.AddParam("DDM.tool.PRTCP.y", RVLPARAM_TYPE_FLOAT, PRTCP_G + 1);
    pParamData = paramList.AddParam("DDM.tool.PRTCP.z", RVLPARAM_TYPE_FLOAT, PRTCP_G + 2);
    pParamData = paramList.AddParam("DDM.dd.openingDirection", RVLPARAM_TYPE_FLOAT, &dd_opening_direction);
    pParamData = paramList.AddParam("DDM.wPos", RVLPARAM_TYPE_FLOAT, &wPos);
    pParamData = paramList.AddParam("DDM.lock_T_G_DD", RVLPARAM_TYPE_BOOL, &bLock_T_G_DD);
    pParamData = paramList.AddParam("DDM.log", RVLPARAM_TYPE_BOOL, &bLog);
    pParamData = paramList.AddParam("DDM.maxnNodesJSPerStep", RVLPARAM_TYPE_INT, &maxnNodesJSPerStep);
    pParamData = paramList.AddParam("DDM.UseFCL", RVLPARAM_TYPE_BOOL, &use_fcl);
    pParamData = paramList.AddParam("DDM.CabinetMeshStaticDirPath", RVLPARAM_TYPE_STRING, &cabinetStaticDirPath);
    pParamData = paramList.AddParam("DDM.CollisionWithPanel", RVLPARAM_TYPE_BOOL, &bVNPanel);
}

void DDManipulator::Clear()
{
    if (pVNEnv)
        delete pVNEnv;
    if (pVNPanel)
        delete dVNPanel;
    if (pVisualizationData)
    {
        if (pVisualizationData->bOwnVisualizer)
            if (pVisualizationData->pVisualizer)
                delete pVisualizationData->pVisualizer;
        delete pVisualizationData;
    }
    RVL_DELETE_ARRAY(VNMClusters.Element);
    RVL_DELETE_ARRAY(dVNEnv);
    RVL_DELETE_ARRAY(tool_sample_spheres.Element);
    RVL_DELETE_ARRAY(tool_contact_spheres.Element);
    RVL_DELETE_ARRAY(feasibleTCPs.Element);
    RVL_DELETE_ARRAY(nodeBuffMem);
    if (pToolMesh)
        delete pToolMesh;
    if (pTimer)
        delete pTimer;
    RVL_DELETE_ARRAY(nodes.Element);
    RVL_DELETE_ARRAY(rndIdx.Element);
    RVL_DELETE_ARRAY(feasibleToolContactPosesFileName);
    RVL_DELETE_ARRAY(contactPoseGraphFileName);
    RVL_DELETE_ARRAY(toolModelDir);
    RVL_DELETE_ARRAY(pathPosesMem);
    RVL_DELETE_ARRAY(pathJointsMem);
    RVL_DELETE_ARRAY(pathMem);
    RVL_DELETE_ARRAY(pathMemJoints);
    RVL_DELETE_ARRAY(approachIKSolutionsMem);
    RVL_DELETE_ARRAY(approachPathMem);
    RVL_DELETE_ARRAY(selectedNodes.Element);
    RVL_DELETE_ARRAY(bSelected);
    RVL_DELETE_ARRAY(contactNode);

    // FCL
    if (pCabinetMeshStatic)
        delete pCabinetMeshStatic;
    if (pCabinetMeshPanel)
        delete pCabinetMeshPanel;
    // if (use_fcl)
    //     delete collisionCabinetObj;
    RVL_DELETE_ARRAY(cabinetStaticDirPath);

    // if (pCabinetWholeMesh)
    //     delete pCabinetWholeMesh;
}

void DDManipulator::SetEnvironmentState(float state)
{
    dd_state_angle = DEG2RAD * state;
    float cs = cos(dd_state_angle);
    float sn = sin(dd_state_angle);
    RVLROTZ(cs, sn, pose_Arot_A.R);
    Pose3D pose_Arot_F;
    RVLCOMPTRANSF3D(pose_A_F.R, pose_A_F.t, pose_Arot_A.R, pose_Arot_A.t, pose_Arot_F.R, pose_Arot_F.t);
    Pose3D pose_Arot_S;
    RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_Arot_F.R, pose_Arot_F.t, pose_Arot_S.R, pose_Arot_S.t);
    RVLCOMPTRANSF3D(pose_Arot_S.R, pose_Arot_S.t, pose_DD_A.R, pose_DD_A.t, pose_DD_S.R, pose_DD_S.t);
    if (bVNPanel)
    {
        RVLCOPYMX3X3(pose_Arot_S.R, VNMClusters.Element[3]->R);
        RVLCOPY3VECTOR(pose_Arot_S.t, VNMClusters.Element[3]->t);
        pVNEnv->Descriptor(dVNEnv);
    }
    if (pVNPanel)
    {
        RVLCOPYMX3X3(pose_Arot_S.R, pPanelVNMCluster->R);
        RVLCOPY3VECTOR(pose_Arot_S.t, pPanelVNMCluster->t);
        pVNPanel->Descriptor(dVNPanel);
    }
}

bool DDManipulator::Free(
    Pose3D *pPose_G_S,
    float *SDF)
{
    // debug
    numColchecks_FreeSDF++;

    if (use_fcl)
        return FreeFCL(pPose_G_S);
    else
    {
        float c_S[3];
        float *c_G = tool_bounding_sphere.c.Element;
        RVLTRANSF3(c_G, pPose_G_S->R, pPose_G_S->t, c_S);
        int iActiveFeature;
        float SDF_ = pVNEnv->Evaluate(c_S, SDF, iActiveFeature, true, dVNEnv);
        if (SDF_ > tool_bounding_sphere.r)
            return true;
        MOTION::Sphere *pSphere;
        for (int iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++)
        {
            pSphere = tool_sample_spheres.Element + iSphere;
            c_G = pSphere->c.Element;
            RVLTRANSF3(c_G, pPose_G_S->R, pPose_G_S->t, c_S);
            SDF_ = pVNEnv->Evaluate(c_S, SDF, iActiveFeature, true, dVNEnv);
            if (SDF_ <= pSphere->r)
                return false;
        }
        return true;
    }
}

bool DDManipulator::Free(float *q)
{
    // debug
    numColchecks_Freeq++;


    // Only for debugging purpose!!!
    // RVLCOLORS
    // Array<Point> visPts;
    // Point visPtMem[2];
    // visPts.Element = visPtMem;
    // visPts.n = 2;
    //

    memcpy(robot.q, q, robot.n * sizeof(float));
    int i;
    int iLink;
    MOTION::Cylinder *pCylinder;
    Pose3D dPose_L;
    Pose3D *pPose_L_0, *pPose_Lprev_0;
    Pose3D pose_L_W;
    float P_W[2][3];
    Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>> *pIntersectionD;
    Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>> *pIntersectionP;

    // FCL variables
    Pose3D pose_C_L;
    Pose3D pose_C_0;
    Pose3D pose_C_W;
    float R_L_C[9];
    float *X_C_L = R_L_C;
    float *Y_C_L = R_L_C + 3;
    float *Z_C_L = R_L_C + 6;
    float h;
    float fTmp;
    int i_, j_, k_;
    bool intersectionFCL;

    for (iLink = 0; iLink <= robot.maxCollisionLinkIdx; iLink++, pPose_Lprev_0 = pPose_L_0)
    {
        pPose_L_0 = robot.link_pose + iLink;
        robot.FwdKinematics(iLink, &dPose_L);
        if (iLink == 0)
            *pPose_L_0 = dPose_L;
        else
            RVLCOMPTRANSF3D(pPose_Lprev_0->R, pPose_Lprev_0->t, dPose_L.R, dPose_L.t, pPose_L_0->R, pPose_L_0->t)
        RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pPose_L_0->R, pPose_L_0->t, pose_L_W.R, pose_L_W.t);
        for (i = 0; i < robot.collisionCylinders.Element[iLink].n; i++)
        {
            pCylinder = robot.collisionCylinders.Element[iLink].Element + i;
            RVLTRANSF3(pCylinder->P[0].Element, pose_L_W.R, pose_L_W.t, P_W[0]);
            RVLTRANSF3(pCylinder->P[1].Element, pose_L_W.R, pose_L_W.t, P_W[1]);
            // Only for debugging purpose!!!
            // RVLCOPY3VECTOR(P_W[0], visPts.Element[0].P);
            // RVLCOPY3VECTOR(P_W[1], visPts.Element[1].P);
            // pVisualizationData->robotActors.push_back(pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(visPts, red, 6.0f));
            //
            pIntersectionD = pVNEnv->VolumeCylinderIntersection(dVNEnv, P_W[0], P_W[1], pCylinder->r);
            pIntersectionP = pVNPanel->VolumeCylinderIntersection(dVNPanel, P_W[0], P_W[1], pCylinder->r);


            if (use_fcl)
            {
                RVLDIF3VECTORS(pCylinder->P[1].Element, pCylinder->P[0].Element, Z_C_L);
                RVLNORM3(Z_C_L, h);
                RVLORTHOGONAL3(Z_C_L, Y_C_L, i_, j_, k_, fTmp);
                RVLCROSSPRODUCT3(Y_C_L, Z_C_L, X_C_L);
                RVLCOPYMX3X3T(R_L_C, pose_C_L.R);
                RVLSUM3VECTORS(pCylinder->P[0].Element, pCylinder->P[1].Element, pose_C_L.t);
                RVLSCALE3VECTOR(pose_C_L.t, 0.5f, pose_C_L.t);
                RVLCOMPTRANSF3D(robot.link_pose[iLink].R, robot.link_pose[iLink].t, pose_C_L.R, pose_C_L.t, pose_C_0.R, pose_C_0.t);
                RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pose_C_0.R, pose_C_0.t, pose_C_W.R, pose_C_W.t);

                // intersectionFCL = CylinderIntersectionFCL(pCylinder->r, h, pose_C_W);
                intersectionFCL = CylinderIntersectionFCL(fclRobotCylinderMeshes[iLink][i], pose_C_W);

                return !intersectionFCL;

                // // debug
                // if ((pIntersectionD->n > 0) != intersectionFCL)
                // {
                //     cout << "NOT THE SAME" << endl;
                //     Pose3D pose_A_S;
                //     RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_A_F.R, pose_A_F.t, pose_A_S.R, pose_A_S.t);

                //     fcl::Transform3<double> T_C_S;
                //     RVLPose2FCLPose(pose_C_W, T_C_S);
                //     intersectionFCL = CylinderIntersectionFCL(pCylinder->r, h, pose_C_W);
                //     if (pIntersectionD->n > 0)
                //     {
                //         Visualizer *pVisualizer = pVisualizationData->pVisualizer;
                //         uchar red[] = {255, 0, 0};
                //         uchar blue[] = {0, 0, 255};
                //         float *PSrc, *PTgt;

                //         Vector3<float> boxSize;
                //         Vector3<float> boxCenter;
                //         Pose3D pose_box_S;
                //         BoxSize<float>(&dd_static_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
                //         BoxCenter<float>(&dd_static_box, boxCenter.Element);
                //         RVLCOPYMX3X3(pose_F_S.R, pose_box_S.R);
                //         RVLTRANSF3(boxCenter.Element, pose_F_S.R, pose_F_S.t, pose_box_S.t);
                //         vtkSmartPointer<vtkActor> staticBoxActor = pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);
                //         BoxSize<float>(&dd_storage_space_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
                //         BoxCenter<float>(&dd_storage_space_box, boxCenter.Element);
                //         vtkSmartPointer<vtkActor> staticSorageSpaceActor = pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);

                //         VisualizeRobot(robot.q, &(pVisualizationData->robotActors));
                //         Pose3D pose_A_S;
                //         RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_A_F.R, pose_A_F.t, pose_A_S.R, pose_A_S.t);
                //         vtkSmartPointer<vtkActor> cabinetStaticMeshActor;
                //         VisualizeCabinetStaticMesh(pose_A_S, cabinetStaticMeshActor);
                //         intersectionFCL = CylinderIntersectionFCL(pCylinder->r, h, pose_C_W);
                //         // if (intersectionFCL)
                //         {

                //             // vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
                //             // sphereSource->SetCenter(contact_pos.x(), contact_pos.y(), contact_pos.z());
                //             // sphereSource->SetRadius(0.02);
                //             // sphereSource->SetThetaResolution(20);
                //             // sphereSource->SetPhiResolution(20);

                //             // vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                //             // mapper->SetInputConnection(sphereSource->GetOutputPort());

                //             // vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
                //             // actor->SetMapper(mapper);
                //             // actor->GetProperty()->SetColor(1.0, 0.0, 0.0);  // Red sphere
                //             // pVisualizer->renderer->AddActor(actor);

                //             int a_ = 2;
                //             pVisualizer->Run();
                //         }
                //         pVisualizer->renderer->RemoveAllViewProps();
                //     }

                // }
            }

            if (pIntersectionD->n > 0)
                return false;
            if (pIntersectionP->n > 0)
                return false;       
        }
    }
    return true;
}

bool DDManipulator::Free(
    Pose3D *pPose_G_S_start,
    Pose3D *pPose_G_S_end)
{

    if (use_fcl)
        return FreeFCL(pPose_G_S_start, pPose_G_S_end);
    else
    {
        Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>> *pIntersection;
        int iSampleSphere;
        int iPose;
        Pose3D *pPose_G_S;
        Vector3<float> c_S[2];
        MOTION::Sphere *pSphere;
        float *c_S_;
        for (iSampleSphere = 0; iSampleSphere < tool_sample_spheres.n; iSampleSphere++)
        {
            pSphere = tool_sample_spheres.Element + iSampleSphere;
            pPose_G_S = pPose_G_S_start;
            for (iPose = 0; iPose < 2; iPose++)
            {
                c_S_ = c_S[iPose].Element;
                RVLTRANSF3(pSphere->c.Element, pPose_G_S->R, pPose_G_S->t, c_S_);
                pPose_G_S = pPose_G_S_end;
            }
            pIntersection = pVNEnv->VolumeCylinderIntersection(dVNEnv, c_S[0].Element, c_S[1].Element, pSphere->r);
            if (pIntersection->n > 0)
                return false;
        }
        return true;
    }
}

void DDManipulator::Free(Array<MOTION::IKSolution> &IKSolutions)
{
    int nFreeIKSolutions = 0;
    for (int i = 0; i < IKSolutions.n; i++)
        if (Free(IKSolutions.Element[i].q))
            IKSolutions.Element[nFreeIKSolutions++] = IKSolutions.Element[i];
    IKSolutions.n = nFreeIKSolutions;
}

bool DDManipulator::LocalConstraints(
    Pose3D *pPose_G_S,
    float *SDF,
    Array<Pair<int, int>> &localConstraints,
    Vector3<float> *c_S_rot,
    Vector3<float> *c_S)
{
    float *c_G;
    Vector3<float> *pc_S = c_S;
    Vector3<float> *pc_S_rot = c_S_rot;
    int iActiveFeature;
    MOTION::Sphere *pSphere;
    float SDF_;
    bool bFree = true;
    localConstraints.n = 0;
    int iConstraint;
    Array<int> sampleSphereLocalConstraints;
    int iFeature;
    Pair<int, int> *pLocalConstraint;
    for (int iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++, pc_S_rot++, pc_S++)
    {
        pSphere = tool_sample_spheres.Element + iSphere;
        c_G = pSphere->c.Element;
        RVLMULMX3X3VECT(pPose_G_S->R, c_G, pc_S_rot->Element);
        RVLSUM3VECTORS(pc_S_rot->Element, pPose_G_S->t, pc_S->Element);
        SDF_ = pVNEnv->LocalConstraints(pc_S->Element, SDF, iActiveFeature, true, dVNEnv);
        if (SDF_ <= pSphere->r)
            bFree = false;
        sampleSphereLocalConstraints = pVNEnv->localConstraints[pVNEnv->iy];
        for (iConstraint = 0; iConstraint < sampleSphereLocalConstraints.n; iConstraint++)
        {
            iFeature = sampleSphereLocalConstraints.Element[iConstraint];
            if (SDF[iFeature] <= rLocalConstraints)
            {
                pLocalConstraint = localConstraints.Element + localConstraints.n++;
                pLocalConstraint->a = iSphere;
                pLocalConstraint->b = iFeature;
            }
        }
    }
    return bFree;
}

float DDManipulator::Cost(
    Pose3D *pPose_G_S_start,
    Pose3D *pPose_G_S_end)
{
    float dt[3];
    RVLDIF3VECTORS(pPose_G_S_end->t, pPose_G_S_start->t, dt);
    float cost = (float)(tool_sample_spheres.n) * sqrt(RVLDOTPRODUCT3(dt, dt));
    // float dR[9];
    // RVLMXMUL3X3T1(pPose_G_S_end->R, pPose_G_S_start->R, dR);
    // float U[3];
    // float th;
    // GetAngleAxis(dR, U, th);
    // int iSphere;
    // float* c_G;
    // float V3Tmp[3];
    // for (iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++)
    //{
    //     c_G = tool_sample_spheres.Element[iSphere].c.Element;
    //     RVLCROSSPRODUCT3(U, c_G, V3Tmp);
    //     cost += sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
    // }
    return cost;
}

bool DDManipulator::FreePose(
    Pose3D *pPose_G_S_init,
    Array<Pair<int, int>> localConstraints,
    Vector3<float> *c_S_rot,
    Vector3<float> *c_S,
    Pose3D *pPose_G_S)
{
    // Parameters.

    int maxnIterations = 3;

    //

    int iConstraint, iFeature, iSphere;
    RECOG::VN_::Feature *pFeature;
    Pair<int, int> *pLocalConstraint;
    Vector3<float> *pc_S;
    Vector3<float> *pc_S_rot;
    float *A = new float[6 * localConstraints.n];
    float *b = new float[localConstraints.n];
    float *a_;
    float *a_t = A + 3;
    for (iConstraint = 0; iConstraint < localConstraints.n; iConstraint++, a_t += 6)
    {
        pLocalConstraint = localConstraints.Element + iConstraint;
        iFeature = pLocalConstraint->b;
        pFeature = pVNEnv->featureArray.Element + iFeature;
        RVLNEGVECT3(pFeature->N, a_t);
    }
    MOTION::Sphere *pSphere;
    float x0[6];
    float *t0 = x0 + 3;
    RVLNULL3VECTOR(x0);
    RVLNULL3VECTOR(t0);
    float x[6];
    float *t = x + 3;
    float th;
    float u[3];
    float dR[9], newR[9];
    Pose3D pose_G_S = *pPose_G_S_init;
    int it;
    float *c_G;
    bool bFree;
    for (it = 0; it < maxnIterations; it++)
    {
        bFree = true;
        for (iConstraint = 0; iConstraint < localConstraints.n; iConstraint++)
        {
            pLocalConstraint = localConstraints.Element + iConstraint;
            iSphere = pLocalConstraint->a;
            pSphere = tool_sample_spheres.Element + iSphere;
            iFeature = pLocalConstraint->b;
            pFeature = pVNEnv->featureArray.Element + iFeature;
            pc_S = c_S + iSphere;
            b[iConstraint] = RVLDOTPRODUCT3(pFeature->N, pc_S->Element) - dVNEnv[iFeature] - pSphere->r;
            if (b[iConstraint] < -1e-7)
                bFree = false;
        }
        if (bFree)
            break;
        a_ = A;
        for (iConstraint = 0; iConstraint < localConstraints.n; iConstraint++, a_ += 6)
        {
            pLocalConstraint = localConstraints.Element + iConstraint;
            iSphere = pLocalConstraint->a;
            pSphere = tool_sample_spheres.Element + iSphere;
            iFeature = pLocalConstraint->b;
            pFeature = pVNEnv->featureArray.Element + iFeature;
            pc_S_rot = c_S_rot + iSphere;
            RVLCROSSPRODUCT3(pFeature->N, pc_S_rot->Element, a_);
        }
        if (!solver.FeasibleSolution(A, b, localConstraints.n, x0, x))
            break;
        Move(x, &pose_G_S, &pose_G_S);
        pc_S = c_S;
        pc_S_rot = c_S_rot;
        for (iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++, pc_S_rot++, pc_S++)
        {
            pSphere = tool_sample_spheres.Element + iSphere;
            c_G = pSphere->c.Element;
            RVLMULMX3X3VECT(pose_G_S.R, c_G, pc_S_rot->Element);
            RVLSUM3VECTORS(pc_S_rot->Element, pose_G_S.t, pc_S->Element);
        }
    }
    *pPose_G_S = pose_G_S;

    delete[] A;
    delete[] b;

    return bFree;
}

bool DDManipulator::FeasiblePose(
    Pose3D *pPose_G_0,
    float *SDF,
    MOTION::NodeJS *nodesJS,
    Array<int> &IKSolutionIdxs,
    bool bApproach,
    Array<MOTION::IKSolution> *pAapproachPathJS,
    int *pnViaPts)
{
#ifdef RVLMOTION_DDMANIPULATOR_MULTI_SOLUTION_IK
    robot.InvKinematics(*pPose_G_0, approachIKSolutions[2], true);
    if (approachIKSolutions[2].n == 0)
        return false;
    int iIKSolution;
    MOTION::NodeJS *pNodeJS;
    MOTION::IKSolution *pIKSolution;
    IKSolutionIdxs.n = 0;
    for (iIKSolution = 0; iIKSolution < approachIKSolutions[2].n; iIKSolution++)
    {
        pIKSolution = approachIKSolutions[2].Element + iIKSolution;
        pNodeJS = nodesJS + (bPath3 ? iIKSolution : pIKSolution->i);
        pNodeJS->IK = *pIKSolution;
        IKSolutionIdxs.Element[IKSolutionIdxs.n++] = pIKSolution->i;
    }
#else
    nodesJS.n = 1;
    if (!robot.InvKinematics(*pPose_G_0, nodesJS.Element[0].q))
        return false;
#endif
    Pose3D pose_G_S;
    RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pPose_G_0->R, pPose_G_0->t, pose_G_S.R, pose_G_S.t);
    if (!Free(&pose_G_S, SDF))
        return false;
    if (bDefaultToolModel)
    {
        float P1_S[3], P2_S[3];
        RVLTRANSF3(default_tool_P1_G, pose_G_S.R, pose_G_S.t, P1_S);
        RVLTRANSF3(default_tool_P2_G, pose_G_S.R, pose_G_S.t, P2_S);
        Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>> *pIntersection =
            pVNEnv->VolumeCylinderIntersection(dVNEnv, P1_S, P2_S, tool_wrist_r);
        if (pIntersection->n > 0)
            return false;
    }
    Free(approachIKSolutions[2]);
    if (approachIKSolutions[2].n == 0)
        return false;
    if (bApproach)
    {
        // Plan approach path.

        Array<Pose3D> poses_G_0_via;
        Pose3D poses_G_0_via_Mem[2];
        poses_G_0_via.Element = poses_G_0_via_Mem;
        Array<Pair<int, int>> approachPaths;
        approachPaths.Element = approachPathMem;
        if (!ApproachPath(&pose_G_S, poses_G_0_via, SDF, approachIKSolutions, approachPaths))
            return false;
        if (approachPaths.n > 0)
            *pnViaPts = (approachPaths.Element[0].a >= 0 ? 2 : 1);
        else
            *pnViaPts = 0;

        // Create paths which are compatible with approachIKSolutions[2] and store them in nodesJS.

        int iPath;
        float *q, *q_, *q__;
        float dq[6];
        float qDist;
        float fTmp;
        int i;
        MOTION::NodeJS *pNodeJS;
        pAapproachPathJS->n = 0;
        int iIKSolution1, iIKSolution2;
        IKSolutionIdxs.n = 0;
        for (iIKSolution = 0; iIKSolution < approachIKSolutions[2].n; iIKSolution++)
        {
            pIKSolution = approachIKSolutions[2].Element + iIKSolution;
            pNodeJS = nodesJS + (bPath3 ? iIKSolution : pIKSolution->i);
            q = pNodeJS->IK.q;
            if (approachPaths.n > 0)
            {
                pNodeJS->bFeasible = false;
                for (iPath = 0; iPath < approachPaths.n; iPath++)
                {
                    iIKSolution2 = approachPaths.Element[iPath].b;
                    q_ = approachIKSolutions[1].Element[iIKSolution2].q;
                    RVLMOTION_JOINT_SPACE_CHEB_DIST(q, q_, dq, qDist, fTmp, i);
                    if (qDist <= JSDistThr)
                    {
                        iIKSolution1 = approachPaths.Element[iPath].a;
                        if (iIKSolution1 >= 0)
                            pAapproachPathJS->Element[pAapproachPathJS->n++] = approachIKSolutions[0].Element[iIKSolution1];
                        pAapproachPathJS->Element[pAapproachPathJS->n++] = approachIKSolutions[1].Element[iIKSolution2];
                        pNodeJS->bFeasible = true;
                        IKSolutionIdxs.Element[IKSolutionIdxs.n++] = pIKSolution->i;
                        break;
                    }
                }
            }
            else
            {
                pNodeJS->bFeasible = true;
                IKSolutionIdxs.Element[IKSolutionIdxs.n++] = pIKSolution->i;
            }
        }
    }
    else
    {
        for (iIKSolution = 0; iIKSolution < approachIKSolutions[2].n; iIKSolution++)
        {
            pIKSolution = approachIKSolutions[2].Element + iIKSolution;
            pNodeJS = nodesJS + (bPath3 ? iIKSolution : pIKSolution->i);
            pNodeJS->bFeasible = true;
        }
    }
    return true;
}

void DDManipulator::Path(Pose3D *pPose_G_S_init)
{
    // Parameters.

    int nTargetSamples = 10000;
    float rSamplePos = 0.05f;
    float rSampleOrient = 45.0f; // deg
    float rPos = 0.05f;
    float rOrient = 15.0f; // deg
    float workSpaceExpansionCoeff = 0.5f;
    float rNeighborPos = 0.10f;
    float rNeighborOrient = 45.0f; // deg
    Pose3D goal;
    RVLUNITMX3(goal.R);
    RVLSET3VECTOR(goal.t, 0.07f, 0.06f, 0.08f);
    Pose3D *pGoal = NULL;
    // Pose3D* pGoal = &goal;

    // Allocate arrays.

    float *SDF = new float[pVNEnv->NodeArray.n];

    // Feasible contact poses.

    std::vector<Pose3D> allFeasibleTCPs;
    allFeasibleTCPs.reserve(feasibleTCPs.n * ((int)ceil(dd_panel_params[0] / dd_contact_surface_params[0] + dd_panel_params[1] / dd_contact_surface_params[1]) - 1));
    Pose3D *pPose_G_DD;
    int iTemplatePose;
    float PRTCP_DD[3];
    float templateEndTol = kTemplateEndTol * dd_contact_surface_sampling_resolution;
    float templateEnd[2];
    templateEnd[0] = dd_contact_surface_params[0] - templateEndTol;
    templateEnd[1] = dd_contact_surface_params[1] - templateEndTol;
    int iAxis;
    float s;
    Pose3D pose_A_S;
    RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_A_F.R, pose_A_F.t, pose_A_S.R, pose_A_S.t);
    Pose3D pose_Arot_S;
    RVLCOMPTRANSF3D(pose_A_S.R, pose_A_S.t, pose_Arot_A.R, pose_Arot_A.t, pose_Arot_S.R, pose_Arot_S.t);
    Pose3D pose_DD_S;
    RVLCOMPTRANSF3D(pose_Arot_S.R, pose_Arot_S.t, pose_DD_A.R, pose_DD_A.t, pose_DD_S.R, pose_DD_S.t);
    Pose3D pose_G_DD;
    Pose3D pose_G_DD_template;
    int iShift;
    float shift;
    for (iTemplatePose = 0; iTemplatePose < feasibleTCPs.n; iTemplatePose++)
    {
        pPose_G_DD = feasibleTCPs.Element + iTemplatePose;
        if (pPose_G_DD->R[6] > -csMaxSurfaceContactAngle)
            continue;
        RVLTRANSF3(PRTCP_G, pPose_G_DD->R, pPose_G_DD->t, PRTCP_DD);
        if (PRTCP_DD[0] < visionTol || PRTCP_DD[1] < visionTol)
            continue;
        pose_G_DD = *pPose_G_DD;
        allFeasibleTCPs.push_back(pose_G_DD);
        pose_G_DD_template = pose_G_DD;
        for (iAxis = 0; iAxis < 2; iAxis++)
        {
            if (PRTCP_DD[iAxis] >= templateEnd[iAxis])
            {
                iShift = 1;
                shift = (float)iShift * dd_contact_surface_sampling_resolution;
                s = PRTCP_DD[iAxis] + shift;
                while (s <= dd_panel_params[iAxis])
                {
                    pose_G_DD.t[iAxis] = pose_G_DD_template.t[iAxis] + shift;
                    allFeasibleTCPs.push_back(pose_G_DD);
                    iShift++;
                    shift = (float)iShift * dd_contact_surface_sampling_resolution;
                    s = PRTCP_DD[iAxis] + shift;
                }
            }
        }
    }

    // SE3 grid.

    SE3Grid grid;
    Box<float> workSpace;
    InitBoundingBox<float>(&workSpace, pPose_G_S_init->t);
    BoundingBoxOfBoxes<float>(&workSpace, &dd_static_box, &workSpace);
    float workSpaceSize = BoxSize<float>(&workSpace);
    ExpandBox<float>(&workSpace, workSpaceExpansionCoeff);
    grid.Create(workSpace);

    // Pose graph.

    Graph<GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, MOTION::Edge, GRAPH::EdgePtr<MOTION::Edge>> graph;
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>> *pGNode;

    // Random sampling of feasible contact poses.
    // Create motion planning nodes from collision free feasible contact poses. These nodes are reffered to in this program as target nodes.

    int nTargetSamples_ = RVLMIN(nTargetSamples, allFeasibleTCPs.size());
    std::vector<MOTION::Node> nodes;
    nodes.reserve(nTargetSamples_);
    int i;
    int iPosCell, iZ, iRoll;
    iPosCell = iZ = iRoll = -1;
    MOTION::Node node;
    int iSE3Point;
    QList<GRAPH::EdgePtr<MOTION::Edge>> *pEdgeList;
    Array<int> rndIdx;
    rndIdx.Element = NULL;
    if (pGoal)
    {
        node.pose.pose = *pGoal;
        node.cost = 0.0f;
        RVLMEM_ALLOC_STRUCT(pMem, GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, pGNode);
        pEdgeList = &(pGNode->EdgeList);
        RVLQLIST_INIT(pEdgeList);
        node.pGNode = pGNode;
        pGNode->idx = nodes.size();
        node.iParent = -1;
        node.iFirstChild = -1;
        node.iSibling = -1;
        nodes.push_back(node);
        grid.Add(node.pose.pose, true, 0, iPosCell, iZ, iRoll);
    }
    else
    {
        rndIdx.n = allFeasibleTCPs.size();
        RandomIndices(rndIdx);
        int iPose;
        Pose3D pose_G_S;
        for (i = 0; i < rndIdx.n; i++)
        {
            iPose = rndIdx.Element[i];
            pPose_G_DD = allFeasibleTCPs.data() + iPose;
            RVLCOMPTRANSF3D(pose_DD_S.R, pose_DD_S.t, pPose_G_DD->R, pPose_G_DD->t, pose_G_S.R, pose_G_S.t);
            iSE3Point = grid.Fetch(pose_G_S, iPosCell, iZ, iRoll);
            if (iSE3Point < 0)
            {
                if (Free(&pose_G_S, SDF))
                {
                    node.pose.pose = pose_G_S;
                    node.cost = 0.0f;
                    RVLMEM_ALLOC_STRUCT(pMem, GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, pGNode);
                    pEdgeList = &(pGNode->EdgeList);
                    RVLQLIST_INIT(pEdgeList);
                    node.pGNode = pGNode;
                    pGNode->idx = nodes.size();
                    node.iParent = -1;
                    node.iFirstChild = -1;
                    node.iSibling = -1;
                    node.flags = 0x00;
                    nodes.push_back(node);
                    grid.Add(pose_G_S, true, pGNode->idx, iPosCell, iZ, iRoll);
                    if (nodes.size() >= nTargetSamples_)
                        break;
                }
                else
                    grid.Add(pose_G_S, false);
            }
        }
    }

    // Is there a collision-free path from the initial pose to any target node?

    int iNode;
    bool bPathFound = false;
    for (iNode = 0; iNode < nodes.size(); iNode++)
        if (Free(pPose_G_S_init, &(nodes[iNode].pose.pose)))
        {
            nodes[iNode].flags |= RVLMOTION_NODE_FLAG_FREE_PATH_TO_START;
            bPathFound = true;
        }

    // If there is no a collision-free path from the initial pose to any target node, then add additional nodes.

    int iNode_;
    float minCost = 0.0f;
    MOTION::Node *pNode_;
    float cost;
    MOTION::Node *pNode;
    Array<int> neighbors;
    Pose3D samplePose;
    float move[3];
    float fTmp;
    float moveDist;
    float rotAxis[3];
    float moveRotAngle;
    float dR[9];
    MOTION::Node *pParent;
    MOTION::Edge *pEdge;
    GRAPH::EdgePtr<MOTION::Edge> *pEdgePtr;
    int iParent;
    int *piNodeFetch, *piNodePush;
    float dCost;
    int iSibling;
    MOTION::Node *pSibling;
    float rNeighborOrientRad = DEG2RAD * rNeighborOrient;
    int *piTmp;
    int nFree = 0;
    int nSamples = 0;
    while (grid.points.n < maxnSE3Points)
    {
        nSamples++;
        if (nSamples % 1000 == 0)
            printf("%d samples\n", nSamples);
        iNode = (RAND_MAX < nodes.size() ? (rand() * RAND_MAX + rand()) % nodes.size() : rand() % nodes.size());
        pNode = nodes.data() + iNode;
        RVLRNDUNIT3VECTOR(move, fTmp);
        moveDist = rSamplePos * (float)rand() / (float)RAND_MAX;
        if (moveDist > rPos)
            moveDist = rPos;
        RVLSCALE3VECTOR(move, moveDist, move);
        RVLSUM3VECTORS(pNode->pose.pose.t, move, samplePose.t);
        RVLRNDUNIT3VECTOR(rotAxis, fTmp);
        moveRotAngle = rSampleOrient * (float)rand() / (float)RAND_MAX;
        if (moveRotAngle > rOrient)
            moveRotAngle = rOrient;
        moveRotAngle *= DEG2RAD;
        AngleAxisToRot<float>(rotAxis, moveRotAngle, dR);
        RVLMXMUL3X3(pNode->pose.pose.R, dR, samplePose.R);
        iSE3Point = grid.Fetch(samplePose, iPosCell, iZ, iRoll);
        // float RDebug[9];
        // RVLMXMUL3X3T1(samplePose.R, grid.points.Element[iSE3Point].pose.R, RDebug);
        // float debug = RAD2DEG * acos(RVLROTDIFF(RDebug));
        // int iDebug1, iDebug2;
        // grid.Cell(grid.points.Element[iSE3Point].pose, i, iDebug1, iDebug2, iPosCell, iZ, iRoll);
        if (iSE3Point >= 0)
            continue;
        if (Free(&samplePose, SDF))
        {
            node.pose.pose = samplePose;
            iNode = nodes.size();
            pGNode->idx = iNode;
            node.flags = 0x00;
            if (Free(pPose_G_S_init, &(node.pose.pose)))
            {
                node.flags |= RVLMOTION_NODE_FLAG_FREE_PATH_TO_START;
                bPathFound = true;
                printf("Path found!\n");
            }
            iParent = -1;
            grid.Neighbors(samplePose, rNeighborPos, rNeighborOrientRad, neighbors);
            // Only for debugging purpose!!!
            // for (i = 0; i < neighbors.n; i++)
            //{
            //    iSE3Point = neighbors.Element[i];
            //    iNode_ = grid.points.Element[iSE3Point].idx;
            //    if (iNode_ >= 0)
            //    {
            //        pNode_ = nodes.data() + iNode_;
            //        float dP[3];
            //        RVLDIF3VECTORS(pNode_->pose.pose.t, samplePose.t, dP);
            //        float debugPos = sqrt(RVLDOTPRODUCT3(dP, dP));
            //        float dR[9];
            //        RVLMXMUL3X3T1(samplePose.R, pNode_->pose.pose.R, dR);
            //        float debugRot = RAD2DEG * acos(RVLROTDIFF(dR));
            //        if (debugPos > rNeighborPos || debugRot > rNeighborOrient)
            //            int debug = 0;
            //    }
            //}
            //
            for (i = 0; i < neighbors.n; i++)
            {
                iSE3Point = neighbors.Element[i];
                iNode_ = grid.points.Element[iSE3Point].idx;
                if (iNode_ >= 0)
                {
                    pNode_ = nodes.data() + iNode_;
                    nFree++;
                    if (Free(&(node.pose.pose), &(pNode_->pose.pose)))
                    {
                        pEdge = ConnectNodes<GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, MOTION::Edge, GRAPH::EdgePtr<MOTION::Edge>>(pGNode, pNode_->pGNode, iNode, iNode_, pMem);
                        pEdge->cost = Cost(&samplePose, &(pNode_->pose.pose));
                        cost = pNode_->cost + pEdge->cost;
                        pEdge->flags = 0x00;
                        if (iParent < 0 || cost < minCost)
                        {
                            minCost = cost;
                            node.flags |= RVLMOTION_NODE_FLAG_PATH_TO_GOAL;
                            iParent = iNode_;
                        }
                    }
                }
            }
            if (node.flags & RVLMOTION_NODE_FLAG_PATH_TO_GOAL)
            {
                RVLMEM_ALLOC_STRUCT(pMem, GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, pGNode);
                pEdgeList = &(pGNode->EdgeList);
                RVLQLIST_INIT(pEdgeList);
                node.pGNode = pGNode;
                node.cost = minCost;
                node.iParent = iParent;
                node.iFirstChild = -1;
                node.iSibling = -1;
                pNode = &node;
                grid.Add(samplePose, true, iNode, iPosCell, iZ, iRoll);
                if (grid.points.n % 1000 == 0)
                    printf("%d SE(3) points\n", grid.points.n);
                if (iParent >= 0)
                {
                    pParent = nodes.data() + iParent;
                    // if (iNode == 13)
                    //     int debug = 0;
                    RVLMOTION_ADD_NODE_TO_TREE(nodes, iNode, pParent, iSibling, pSibling);
                    pEdgePtr = pEdgeList->pFirst;
                    while (pEdgePtr)
                    {
                        RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);
                        // if (iNode == 20 && iNode_ == 13)
                        //     int debug = 0;
                        pNode_ = nodes.data() + iNode_;
                        cost = pEdge->cost + node.cost;
                        if (cost < pNode_->cost)
                        {
                            if (pNode_->iParent >= 0)
                            {
                                pParent = nodes.data() + pNode_->iParent;
                                RVLMOTION_REMOVE_NODE_FROM_TREE(nodes, iNode_, pNode_, pParent, iSibling, pSibling);
                            }
                            RVLMOTION_ADD_NODE_TO_TREE(nodes, iNode_, pNode, iSibling, pSibling);
                            // if (nodes.size() >= 20)
                            //     if (nodes[19].iParent == nodes[19].iSibling)
                            //         int debug = 0;
                            pNode_->iParent = iNode;
                            dCost = pNode_->cost - cost;
                            piNodeFetch = piNodePush = nodeBuffMem;
                            *(piNodePush++) = iNode_;
                            while (piNodeFetch < piNodePush)
                            {
                                iNode_ = *(piNodeFetch++);
                                pNode_ = nodes.data() + iNode_;
                                pNode_->cost -= dCost;
                                iSibling = pNode_->iFirstChild;
                                while (iSibling >= 0)
                                {
                                    *(piNodePush++) = iSibling;
                                    if (piNodePush - nodeBuffMem >= nodeBuffMemCapacity)
                                    {
                                        piTmp = nodeBuffMem;
                                        nodeBuffMem = new int[2 * nodeBuffMemCapacity];
                                        memcpy(nodeBuffMem, piTmp, nodeBuffMemCapacity * sizeof(int));
                                        nodeBuffMemCapacity *= 2;
                                        delete[] piTmp;
                                    }
                                    pSibling = nodes.data() + iSibling;
                                    iSibling = pSibling->iSibling;
                                }
                            }
                        }
                        pEdgePtr = pEdgePtr->pNext;
                    }
                }
                nodes.push_back(node);
            }
        }
        else
        {
            grid.Add(samplePose, false);
            if (grid.points.n % 1000 == 0)
                printf("%d SE(3) points\n", grid.points.n);
        }
    }

    // Only for debugging purpose!!!

    // iNode = iNode_ = 0;
    // float minZ = nodes[iNode_].pose.pose.t[2];
    // float z;
    // for (iNode_ = 1; iNode_ < nodes.size(); iNode_++)
    //{
    //     z = nodes[iNode_].pose.pose.t[2];
    //     if (z < minZ)
    //     {
    //         minZ = z;
    //         iNode = iNode_;
    //     }
    // }

    iNode = -1;
    for (iNode_ = 0; iNode_ < nodes.size(); iNode_++)
    {
        pNode_ = nodes.data() + iNode_;
        if (pNode_->flags & RVLMOTION_NODE_FLAG_FREE_PATH_TO_START)
        {
            cost = pNode_->cost + Cost(pPose_G_S_init, &(pNode_->pose.pose));
            if (iNode < 0 || cost < minCost)
            {
                minCost = cost;
                iNode = iNode_;
            }
        }
    }
    if (iNode >= 0)
    {
        node.pose.pose = *pPose_G_S_init;
        node.iParent = iNode;
        iNode = nodes.size();
        nodes.push_back(node);
    }

    /// Visualization.

    uchar red[] = {255, 0, 0};

    // Initial visualization.

    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    std::vector<int> path;
    iNode_ = iNode;
    while (iNode_ >= 0)
    {
        path.push_back(iNode_);
        iNode_ = nodes[iNode_].iParent;
    }
    Array<float> doorStates;
    doorStates.n = 1;
    doorStates.Element = &dd_state_angle;
    Visualize(&nodes, &path, doorStates);

    // Visualize sample sphere path.

    // MOTION::Sphere* pSphere = tool_sample_spheres.Element;
    // pNode = nodes.data() + iNode;
    // float* c_S_;
    // Point visPt[2];
    // Pose3D *pPose_G_S = pPose_G_S_init;
    // for (iPose = 0; iPose < 2; iPose++)
    //{
    //     c_S_ = visPt[iPose].P;
    //     RVLTRANSF3(pSphere->c.Element, pPose_G_S->R, pPose_G_S->t, c_S_);
    //     pPose_G_S = &(pNode->pose.pose);
    // }
    // pVisualizer->DisplayLine(visPt, red);

    // Visualize TCPs.

    // Array<Point> visNodes;
    // visNodes.n = nodes.size();
    // visNodes.Element = new Point[visNodes.n];
    // float* PSrc, * PTgt;
    // for (iNode = 0; iNode < nodes.size(); iNode++)
    //{
    //     PSrc = nodes[iNode].pose.pose.t;
    //     PTgt = visNodes.Element[iNode].P;
    //     RVLCOPY3VECTOR(PSrc, PTgt);
    // }
    // uchar blue[] = { 0, 0, 255 };
    // pVisualizer->DisplayPointSet<float, Point>(visNodes, blue, 6);

    pVisualizer->Run();

    // Visualization of the results of VN::VolumeCylinderIntersection.

    // float sceneCenter[3];
    // BoxCenter<float>(&dd_static_box, sceneCenter);
    // float sceneSize[3];
    // BoxSize<float>(&dd_static_box, sceneSize[0], sceneSize[1], sceneSize[2]);
    // float sceneR = 0.6 * sqrt(RVLDOTPRODUCT3(sceneSize, sceneSize));
    // float sceneD = 2.0f * sceneR;
    // float U[3], V[3];
    // Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection;
    // Point visLineEndPt[2];
    // Array<Point> visIntersectionPts;
    // float* P1, *visP;
    // int j;
    // float s_[2];
    // for (int iVis = 0; iVis < 10; iVis++)
    //{
    //     RVLSET3VECTOR(U, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
    //     //RVLSET3VECTOR(U, -0.761679471, 0.00717963837, 0.7);
    //     //RVLSET3VECTOR(U, 1.00f, 0.01f, 0.01f);
    //     RVLNORM3(U, fTmp);
    //     RVLSCALE3VECTOR(U, sceneR, V);
    //     RVLDIF3VECTORS(sceneCenter, V, visLineEndPt[0].P);
    //     RVLSUM3VECTORS(sceneCenter, V, visLineEndPt[1].P);
    //     vtkSmartPointer<vtkActor> lineActor = pVisualizer->DisplayLine(visLineEndPt, red);
    //     vtkSmartPointer<vtkActor> intersectionPtsActor;
    //     pIntersection = pVNEnv->VolumeCylinderIntersection(dVNEnv, visLineEndPt[0].P, visLineEndPt[1].P, 0.0f);
    //     if (pIntersection->n > 0)
    //     {
    //         visIntersectionPts.Element = new Point[2 * pIntersection->n];
    //         visIntersectionPts.n = 0;
    //         for (i = 0; i < pIntersection->n; i++)
    //         {
    //             s_[0] = pIntersection->Element[i].a.s;
    //             s_[1] = pIntersection->Element[i].b.s;
    //             for (j = 0; j < 2; j++)
    //             {
    //                 if (s_[j] >= 0.0f && s_[j] <= sceneD)
    //                 {
    //                     RVLSCALE3VECTOR(U, s_[j], V);
    //                     P1 = visLineEndPt[0].P;
    //                     visP = visIntersectionPts.Element[visIntersectionPts.n++].P;
    //                     RVLSUM3VECTORS(P1, V, visP);
    //                 }
    //             }
    //         }
    //         intersectionPtsActor = pVisualizer->DisplayPointSet<float, Point>(visIntersectionPts, red, 6);
    //         delete[] visIntersectionPts.Element;
    //     }
    //     pVisualizer->Run();
    //     pVisualizer->renderer->RemoveActor(lineActor);
    //     if(pIntersection->n > 0)
    //         pVisualizer->renderer->RemoveActor(intersectionPtsActor);
    // }

    // More visualizations of the tool.

    // for (int iVis = 1; iVis < 10; iVis++)
    //{
    //     for (i = 0; i < pVisualizationData->robotActors.size(); i++)
    //         pVisualizer->renderer->RemoveActor(pVisualizationData->robotActors[i]);
    //     Visualize(nodes[iVis].pose.pose);
    //     pVisualizer->Run();
    // }

    ///

    // Free memory.

    delete[] SDF;
    delete[] rndIdx.Element;
    // delete[] visNodes.Element;
}

#define RVLMOTION_JOINT_SPACE_DIST(q, q_, dq, dist, i) \
    {                                                  \
        RVLDIFVECTORS(q, q_, 6, dq, i);                \
        for (i = 0; i < 6; i++)                        \
            RVLNORMANGLE(dq[i]);                       \
        RVLDOTPRODUCT(dq, dq, 6, dist, i);             \
    }

bool DDManipulator::Path2(
    float *qInit,
    float endDoorState,
    int nStates,
    Array<Pose3D> &poses_G_0,
    Array2D<float> &robotJoints,
    Array<Array<Pose3D>> *pFeasiblePaths,
    Array<Array2D<float>> *pFeasiblePathsJoints)
{
#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
    if (pTimer)
        pTimer->Start();
#endif

    // Parameters.

    float startDoorState = RAD2DEG * dd_state_angle; // deg
    // float endDoorState = 10.0f;     // deg
    // int nStates = 1;
    bPath3 = false;

    // Constants.

    float dDoorState = (nStates > 1 ? (endDoorState - startDoorState) / (float)(nStates - 1) : 0.0f);
    int nStates_ = nStates + 2;

    /// Door openning path planning.

    // printf("Path planning ");

    // Adapt the contact pose graph to the target door.
    // CreateContactPoseGraph2("/home/RVLuser/rvl-linux/data/Robotiq3Finger/contact_pose_graph.dat");

    AdaptContactPoseGraph();

    // Set door initial state.

    SetEnvironmentState(startDoorState);

    // Free space planes.

    FreeSpacePlanes();

    // Initialize JS nodes.

    int i;
    int nNodesJS = maxnIKSolutions * nodes.n;
    MOTION::NodeJS *nodeJSMem = new MOTION::NodeJS[2 * nNodesJS];
    MOTION::NodeJS *nodeJS = nodeJSMem;
    MOTION::NodeJS *prevNodeJS = nodeJSMem + nNodesJS;
    MOTION::NodeJS *pNodeJS;
    MOTION::NodeJS *pNodeJS_;
    int *pathMem_ = new int[2 * nNodesJS * nStates_];
    int iSelectedNode;
    int iIKSolution;
    int iNode;
    int iNodeJS;

    if (bCountPoses)
        vecSelectedNodes_debug.push_back(selectedNodes.n);

    for (iSelectedNode = 0; iSelectedNode < selectedNodes.n; iSelectedNode++)
    {
        iNode = selectedNodes.Element[iSelectedNode];
        for (iIKSolution = 0; iIKSolution < maxnIKSolutions; iIKSolution++)
        {
            iNodeJS = iNode * maxnIKSolutions + iIKSolution;
            pNodeJS = nodeJS + iNodeJS;
            pNodeJS->iContactNode = iNode;
            pNodeJS->path = pathMem_ + nStates_ * iNodeJS;
            pNodeJS->bFeasible = false;
            pNodeJS_ = prevNodeJS + iNodeJS;
            pNodeJS_->iContactNode = iNode;
            pNodeJS_->path = pathMem_ + nStates_ * (nNodesJS + iNodeJS);
            pNodeJS_->bFeasible = false;
        }
    }

    //

    float *SDF = new float[pVNEnv->NodeArray.n];
    Pose3D pose_G_0;
    float doorState = startDoorState;
    GRAPH::EdgePtr<Edge> *pEdgePtr;
    float dCost;
    float dq[6];
    float cost;
    float minCost = 0.0f;
    int iMinCostNeighbor;
    int iState;
    Array<float> doorStates;
    doorStates.n = nStates;
    doorStates.Element = new float[nStates]; // For visualization prupose.
    float V3Tmp[3];
    Array<Pose3D> poses_G_0_via;
    Pose3D viaPtPosesMem[2];
    poses_G_0_via.Element = viaPtPosesMem;
    int maxnNodesJS = selectedNodes.n * maxnIKSolutions;
    int *feasibleNodeMem = new int[2 * maxnNodesJS];
    Array<int> feasibleNodes;
    feasibleNodes.Element = feasibleNodeMem;
    Array<int> feasibleNodesPrev;
    feasibleNodesPrev.Element = feasibleNodeMem + maxnNodesJS;
    feasibleNodesPrev.n = maxnNodesJS;
    int iFeasibleNode = 0;
    for (iSelectedNode = 0; iSelectedNode < selectedNodes.n; iSelectedNode++)
        for (iIKSolution = 0; iIKSolution < maxnIKSolutions; iIKSolution++)
            feasibleNodesPrev.Element[iFeasibleNode++] = selectedNodes.Element[iSelectedNode] * maxnIKSolutions + iIKSolution;
    Array<int> arrayTmp;
    int *piTmp;
    bool bPath;
    float posCost;
    float PRTCP_DD[3];
    bool bAllNeighbors;
    bool *bFeasibilityTested = new bool[maxnNodesJS];
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>> *pGNode;
    MOTION::Edge *pEdge;
    int iNode_;
    MOTION::Node *pNode, pNode_;
    int iRndIdx = 0;
    Array<int> candidateNodes;
    candidateNodes.Element = new int[nodes.n];
    bool *bCandidate = new bool[nodes.n];
    int iCandidateNode;
    int iFeasibleNode_;
    int iNodeJS_;
    MOTION::NodeJS *pNodeJSTmp;
    float chebDist;
    float fTmp;
    MOTION::NodeJS *nodesJS_;
    MOTION::IKSolution *approachPathJSMem = new MOTION::IKSolution[2 * maxnNodesJS];
    int nApproachJSPtsTotal = 0;
    int iState_;
    Array<int> IKSolutionIdxs;
    IKSolutionIdxs.Element = new int[maxnIKSolutions];
    Array<MOTION::IKSolution> approachPathsJS;
    int nApproachPathViaPts;
    int IKSolutionIdx;
    float qDist;

    // debug
    vecnumExploredPoses_debug.clear();
    vecNumColchecks_Freeq.clear();
    vecNumColchecks_FreeSDF.clear();
    
    for (iState = 0; iState < nStates; iState++)
    {
        // debug
        numColchecks_Freeq = 0;
        numColchecks_FreeSDF = 0;

        // printf(".");

        iState_ = iState + 2;

        // Set the door state.

        doorState = startDoorState + (float)iState * dDoorState;
        if (iState > 0)
            SetEnvironmentState(doorState);
        doorStates.Element[iState] = doorState;
        RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pose_DD_S.R, pose_DD_S.t, pose_DD_0.R, pose_DD_0.t, V3Tmp);

        /// Inverse kinematics and feasibility.

        if (bDefaultToolModel)
        {
            float tool_hand_len = tool_finger_size.Element[2] + tool_palm_size.Element[2];
            RVLSET3VECTOR(default_tool_P1_G, 0.0f, 0.0f, -tool_hand_len);
            RVLSET3VECTOR(default_tool_P2_G, 0.0f, 0.0f, -(tool_hand_len + tool_wrist_len));
        }
        Pose3D pose_G_S;
        feasibleNodes.n = 0;

        //* New method.

        candidateNodes.n = 0;
        memset(bCandidate, 0, nodes.n * sizeof(bool));
        if (iState == 0)
        {
            // candidateNodes <- contact nodes corresponding to all feasible JS nodes

            for (iFeasibleNode = 0; iFeasibleNode < feasibleNodesPrev.n; iFeasibleNode++)
            {
                iNodeJS = feasibleNodesPrev.Element[iFeasibleNode];
                pNodeJS = nodeJS + iNodeJS;
                iNode = pNodeJS->iContactNode;
                if (!bCandidate[iNode])
                {
                    bCandidate[iNode] = true;
                    candidateNodes.Element[candidateNodes.n++] = iNode;
                }
            }
        }
        else
        {
            // candidateNodes <- contact nodes in the neighborhood of the contact nodes corresponding to all feasible JS nodes

            for (iFeasibleNode = 0; iFeasibleNode < feasibleNodesPrev.n; iFeasibleNode++)
            {
                iNodeJS = feasibleNodesPrev.Element[iFeasibleNode];
                pNodeJS = nodeJS + iNodeJS;
                iNode = pNodeJS->iContactNode;
                if (bCandidate[iNode])
                    continue;
                bCandidate[iNode] = true;
                candidateNodes.Element[candidateNodes.n++] = iNode;
                pGNode = graph.NodeArray.Element + iNode;
                pEdgePtr = pGNode->EdgeList.pFirst;
                while (pEdgePtr)
                {
                    RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);
                    if (bSelected[iNode_])
                    {
                        if (!bCandidate[iNode_])
                        {
                            bCandidate[iNode_] = true;
                            candidateNodes.Element[candidateNodes.n++] = iNode_;
                        }
                    }
                    pEdgePtr = pEdgePtr->pNext;
                }
            }
        }
        Permute(rndIdx, iRndIdx, candidateNodes);
        iRndIdx = (iRndIdx + candidateNodes.n) % rndIdx.n;

        // feasibleNodes <- JS nodes created from a subset of contact nodes randomly selected from candidateNodes for which IK solution exists and which are collision free
        feasibleNodes.n = 0;
        numExploredPoses_debug = 0;
        // FILE *fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "nodes.txt").data(), "w");

        for (iCandidateNode = 0; iCandidateNode < candidateNodes.n; iCandidateNode++)
        {
            iNode = candidateNodes.Element[iCandidateNode];
            // if (iNode == 1147826 /8 || iNode == 1144901/8)
            // if (iNode == 1030757 /8)
            // {
            //     int debug = 0;
            //     cout << "debug" << endl;
            // }
            // if (iNode == 313493 / 8)
            pNode = nodes.Element + iNode;
            RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
            nodesJS_ = nodeJS + iNode * maxnIKSolutions;
            approachPathsJS.Element = approachPathJSMem + nApproachJSPtsTotal;
            numExploredPoses_debug++;
            // fprintf(fpLog,"Explored: %d\n", numExploredPoses_debug);
            if (!FeasiblePose(&pose_G_0, SDF, nodesJS_, IKSolutionIdxs, iState == 0, &approachPathsJS, &nApproachPathViaPts))
                continue;
            // if(numExploredPoses_debug == 45335)
            // {
            //     int debug = 0;
            // }
            for (iIKSolution = 0; iIKSolution < IKSolutionIdxs.n; iIKSolution++)
            {
                IKSolutionIdx = IKSolutionIdxs.Element[iIKSolution];
                pNodeJS = nodesJS_ + IKSolutionIdx;
                if (!pNodeJS->bFeasible)
                    continue;
                pNodeJS->iContactNode = iNode;
                iNodeJS = iNode * maxnIKSolutions + IKSolutionIdx;
                // if (iNodeJS == 1030757 || iNodeJS == 1144901)
                // if (iNodeJS == 1030757)
                //     int debug = 0;
                if (feasibleNodes.n < maxnNodesJSPerStep)
                {
                    feasibleNodes.Element[feasibleNodes.n++] = iNodeJS;
                    // fprintf(fpLog, "Feasible nodes: %d\n", feasibleNodes.n);
                    if (iState == 0)
                    {
                        if (nApproachPathViaPts == 2)
                            pNodeJS->path[0] = nApproachJSPtsTotal++;
                        else
                            pNodeJS->path[0] = -1;
                        if (nApproachPathViaPts >= 1)
                            pNodeJS->path[1] = nApproachJSPtsTotal++;
                        else
                            pNodeJS->path[1] = -1;
                        pNodeJS->path[2] = iNodeJS;
                    }
                }
                else
                    pNodeJS->bFeasible = false;
            }

            if (feasibleNodes.n >= maxnNodesJSPerStep)
                break;
        }

        // debug
        // fclose(fpLog);
        vecnumExploredPoses_debug.push_back(numExploredPoses_debug);

        // memset(bFeasibilityTested, 0, nodes.n * sizeof(bool));
        // for (iFeasibleNode = 0; iFeasibleNode < feasibleNodesPrev.n; iFeasibleNode++)
        //{
        //     //iFeasibleNode = rndIdx.Element[iRndIdx % feasibleNodesPrev.n];
        //     //iRndIdx = (iRndIdx + 1) % rndIdx.n;
        //     iNode = feasibleNodesPrev.Element[iFeasibleNode];
        //     pGNode = graph.NodeArray.Element + iNode;
        //     bAllNeighbors = false;
        //     pEdgePtr = NULL;
        //     do
        //     {
        //         if (pEdgePtr)
        //         {
        //             RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);
        //             pEdgePtr = pEdgePtr->pNext;
        //         }
        //         else
        //         {
        //             iNode_ = iNode;
        //             pEdgePtr = pGNode->EdgeList.pFirst;
        //         }
        //         if (!bFeasibilityTested[iNode_])
        //         {
        //             pNode_ = nodes.Element + iNode_;
        //             RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
        //             if (FeasiblePose(&pose_G_0, SDF, pNodeJS->q, iState == 0))
        //             {

        //            }
        //            bFeasibilityTested[iNode_] = true;
        //        }
        //    } while (pEdgePtr);
        //}

        //

        //* Old method.

        // for (iSelectedNode = 0; iSelectedNode < selectedNodes.n; iSelectedNode++)
        //{
        //     iNode = selectedNodes.Element[iSelectedNode];
        //     pNode = nodes.Element + iNode;
        //     pNodeJS = pNodeData + iNode;
        //     //if (iNode == 35387)
        //     //    int debug = 0;
        //     RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);

        //    // Only for debugging purpose!!!

        //    // if (pNode->iSE3Point == 0)
        //    // {
        //         //robot.InvKinematics(pose_G_0);
        //         //robot.FwdKinematics();
        //         //std::vector<Node> debugNodes;
        //         //Node debugNode;
        //         //debugNode.pose.pose = pose_G_0;
        //         //debugNodes.push_back(debugNode);
        //         //std::vector<int> debugPath;
        //         //debugPath.push_back(0);
        //         //Array2D<float> debugRobotJoints;
        //         //debugRobotJoints.w = robot.n;
        //         //debugRobotJoints.h = 1;
        //         //debugRobotJoints.Element = robot.q;
        //         //doorStates.n = 1;
        //         //Visualize(&debugNodes, &debugPath, doorStates, true, false, -1, &debugRobotJoints);
        //    // }

        //    //

        //    if (pNodeJS->bFeasible = FeasiblePose(&pose_G_0, SDF, pNodeJS->q, iState == 0))
        //        feasibleNodes.Element[feasibleNodes.n++] = iNode;

        //    //if (!robot.InvKinematics(pose_G_0, pNodeJS->q))
        //    //    continue;
        //    //RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pose_G_0.R, pose_G_0.t, pose_G_S.R, pose_G_S.t);
        //    //if (!Free(&pose_G_S, SDF))
        //    //    continue;
        //    //if (bDefaultToolModel)
        //    //{
        //    //    RVLTRANSF3(P1_G, pose_G_S.R, pose_G_S.t, P1_S);
        //    //    RVLTRANSF3(P2_G, pose_G_S.R, pose_G_S.t, P2_S);
        //    //    Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection =
        //    //        pVNEnv->VolumeCylinderIntersection(dVNEnv, P1_S, P2_S, tool_wrist_r);
        //    //    if (pIntersection->n > 0)
        //    //        continue;
        //    //}
        //    //if (iState == 0)
        //    //    if (!ApproachPath(&pose_G_S, poses_G_0_via, SDF))
        //    //        continue;
        //    //pNodeJS->bFeasible = true;
        //    //feasibleNodes.Element[feasibleNodes.n++] = iNode;
        //}

        ///
        vecFeasiblePoses_debug.push_back(feasibleNodes.n);
        if (feasibleNodes.n == 0)
        {
            vecNumColchecks_Freeq.push_back(numColchecks_Freeq);
            vecNumColchecks_FreeSDF.push_back(numColchecks_FreeSDF);
            break;
        }
        // if (feasibleNodes.n == 0)
        //     break;

        // Find the optimal feasible path starting from the feasible robot configurations in the current state
        // and back tracking to the start state.

        bPath = (iState == 0);
        for (iFeasibleNode = 0; iFeasibleNode < feasibleNodes.n; iFeasibleNode++)
        {
            iNodeJS = feasibleNodes.Element[iFeasibleNode];
            pNodeJS = nodeJS + iNodeJS;
            iNode = pNodeJS->iContactNode;
            pNode = nodes.Element + iNode;
            // if (iNodeJS == 101482)
            //     int debug = 0;
            if (iState == 0)
            {
                pNodeJS->cost = 0.0f;
                pNodeJS->path[iState_] = iNodeJS;
            }
            else
            {
                pNodeJS_ = prevNodeJS + iNodeJS;
                iMinCostNeighbor = -1;
                if (pNodeJS_->bFeasible)
                {
                    RVLMOTION_JOINT_SPACE_DIST(pNodeJS->IK.q, pNodeJS_->IK.q, dq, dCost, i);
                    minCost = dCost + pNodeJS_->cost;
                    iMinCostNeighbor = iNodeJS;
                }
                if (!bLock_T_G_DD)
                {
                    pGNode = pNode->pGNode;
                    pEdgePtr = pGNode->EdgeList.pFirst;
                    while (pEdgePtr)
                    {
                        RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);
                        if (bSelected[iNode_])
                        {
                            iNodeJS_ = maxnIKSolutions * iNode_;
                            for (iIKSolution = 0; iIKSolution < maxnIKSolutions; iIKSolution++, iNodeJS_++)
                            {
                                pNodeJS_ = prevNodeJS + iNodeJS_;
                                if (pNodeJS_->bFeasible)
                                {
                                    // if (iNode_ == 6544)
                                    //     int debug = 0;
                                    RVLMOTION_JOINT_SPACE_CHEB_DIST(pNodeJS->IK.q, pNodeJS_->IK.q, dq, qDist, fTmp, i);
                                    if (qDist <= JSDistThr)
                                    {
                                        RVLMOTION_JOINT_SPACE_DIST(pNodeJS->IK.q, pNodeJS_->IK.q, dq, dCost, i);
                                        cost = dCost + pNodeJS_->cost;
                                        if (cost < minCost || iMinCostNeighbor < 0)
                                        {
                                            minCost = cost;
                                            iMinCostNeighbor = iNodeJS_;
                                        }
                                    }
                                }
                            }
                        }
                        pEdgePtr = pEdgePtr->pNext;
                    }
                }
                if (iMinCostNeighbor >= 0)
                {
                    pNodeJS_ = prevNodeJS + iMinCostNeighbor;
                    posCost = posCostMaxDist - RVLMIN(pNode->PRTCP[0], pNode->PRTCP[1]);
                    if (posCost < 0.0f)
                        posCost = 0.0f;
                    pNodeJS->cost = minCost + wPos * posCost;
                    memcpy(pNodeJS->path, pNodeJS_->path, iState_ * sizeof(int));
                    pNodeJS->path[iState_] = iNodeJS;
                    // if (iNodeJS == 101482)
                    //     int debug = 0;
                    bPath = true;
                }
                else
                    pNodeJS->bFeasible = false;
            }
        }
        if (!bPath)
            break;
        iFeasibleNode_ = 0;
        for (iFeasibleNode = 0; iFeasibleNode < feasibleNodes.n; iFeasibleNode++)
        {
            iNodeJS = feasibleNodes.Element[iFeasibleNode];
            pNodeJS = nodeJS + iNodeJS;
            if (pNodeJS->bFeasible)
                feasibleNodes.Element[iFeasibleNode_++] = iNodeJS;
        }
        feasibleNodes.n = iFeasibleNode_;
        for (iFeasibleNode = 0; iFeasibleNode < feasibleNodesPrev.n; iFeasibleNode++)
        {
            iNodeJS = feasibleNodesPrev.Element[iFeasibleNode];
            pNodeJS = prevNodeJS + iNodeJS;
            pNodeJS->bFeasible = false;
        }
        arrayTmp = feasibleNodes;
        feasibleNodes = feasibleNodesPrev;
        feasibleNodesPrev = arrayTmp;
        pNodeJSTmp = nodeJS;
        nodeJS = prevNodeJS;
        prevNodeJS = pNodeJSTmp;

        // debug
        vecNumColchecks_Freeq.push_back(numColchecks_Freeq);
        vecNumColchecks_FreeSDF.push_back(numColchecks_FreeSDF);

    }
    nodeJS = prevNodeJS;
    feasibleNodes = feasibleNodesPrev;
    delete[] bFeasibilityTested;
    delete[] candidateNodes.Element;
    delete[] bCandidate;
    delete[] IKSolutionIdxs.Element;

    // Find the optimal path.

    int iBestEndNode = -1;
    if (iState >= nStates)
    {
        minCost = 2.0f * 4.0f * PI * PI * 6.0f * (float)nStates;
        for (iFeasibleNode = 0; iFeasibleNode < feasibleNodes.n; iFeasibleNode++)
        {
            iNodeJS = feasibleNodes.Element[iFeasibleNode];
            pNodeJS = nodeJS + iNodeJS;
            if (!pNodeJS->bFeasible)
                continue;
            if (pNodeJS->cost < minCost)
            {
                minCost = pNodeJS->cost;
                iBestEndNode = iNodeJS;
            }
        }
    }

    // printf(" completed.\n");

    ///

    bPath = (iBestEndNode >= 0);
    if (bPath)
    {
        /// Copy the resulting path to the output format.

        int maxnPathPoints = nStates + 3;
        int maxnPathPointsTotal = feasibleNodes.n * maxnPathPoints;
        if (pFeasiblePaths)
        {
            RVL_DELETE_ARRAY(pathPosesMem);
            pathPosesMem = new Pose3D[maxnPathPointsTotal];
            RVL_DELETE_ARRAY(pathMem);
            pathMem = new Array<Pose3D>[feasibleNodes.n];
            pFeasiblePaths->n = feasibleNodes.n;
            pFeasiblePaths->Element = pathMem;
        }
        if (pFeasiblePathsJoints)
        {
            RVL_DELETE_ARRAY(pathJointsMem);
            pathJointsMem = new float[robot.n * maxnPathPointsTotal];
            RVL_DELETE_ARRAY(pathMemJoints);
            pathMemJoints = new Array2D<float>[feasibleNodes.n];
            pFeasiblePathsJoints->n = feasibleNodes.n;
            pFeasiblePathsJoints->Element = pathMemJoints;
        }

        std::vector<Node> pathNodes;
        std::vector<int> path_;
        Array<float> doorStates_;
        doorStates_.Element = new float[maxnPathPoints];
        robotJoints.w = robot.n;
        robotJoints.Element = new float[robotJoints.w * maxnPathPoints];
        Node node;
        float *q = robotJoints.Element;
        Array<Pose3D> *pPath;
        Array2D<float> *pPathJoints;
        int iLastNode;
        Array<MOTION::IKSolution> IKSolutions;
        IKSolutions.Element = new MOTION::IKSolution[maxnIKSolutions];
        int iClosestIKSolution;
        Pose3D *pPose_n_0 = robot.link_pose + robot.n - 1;
        for (int iPath = 0; iPath < feasibleNodes.n; iPath++)
        {
            iLastNode = feasibleNodes.Element[iPath];
            if (pFeasiblePaths == NULL && pFeasiblePathsJoints == NULL && iLastNode != iBestEndNode)
                continue;
            pNodeJS = nodeJS + iLastNode;

            // Approach path.

            float *q_;
            for (robotJoints.h = 0; robotJoints.h < 3; robotJoints.h++)
            {
                q = robotJoints.Element + robotJoints.w * robotJoints.h;
                if (robotJoints.h == 0)
                    q_ = qInit;
                else
                {
                    if (pNodeJS->path[robotJoints.h - 1] < 0)
                        continue;
                    q_ = approachPathJSMem[pNodeJS->path[robotJoints.h - 1]].q;
                }
                memcpy(q, q_, 6 * sizeof(float));
                memcpy(robot.q, q, robot.n * sizeof(float));
                robot.FwdKinematics();
                RVLCOMPTRANSF3D(pPose_n_0->R, pPose_n_0->t, robot.pose_TCP_6.R, robot.pose_TCP_6.t, pose_G_0.R, pose_G_0.t);
                node.pose.pose = pose_G_0;
                node.PRTCP[0] = node.PRTCP[1] = 0.0f; // Only for debugging purpose.
                pathNodes.push_back(node);
                path_.push_back(robotJoints.h);
                doorStates_.Element[robotJoints.h] = startDoorState;
            }

#ifdef NEVER // Old method.
            iNode = pNodeJS->path[0];
            pNode = nodes.Element + iNode;
            SetEnvironmentState(startDoorState);
            RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pose_DD_S.R, pose_DD_S.t, pose_DD_0.R, pose_DD_0.t, V3Tmp);
            RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
            ApproachPath(&pose_G_0, poses_G_0_via, SDF);
            // poses_G_0_via.n = 0;    // Only for debugging purpose!!!
            int iState_;
            for (i = 0; i < poses_G_0_via.n; i++)
            {
                iState_ = i + 1;
                pose_G_0 = poses_G_0_via.Element[poses_G_0_via.n - i - 1];
                q = robotJoints.Element + robotJoints.w * iState_;
                robot.InvKinematics(pose_G_0, q);
                node.pose.pose = pose_G_0;
                node.PRTCP[0] = node.PRTCP[1] = 0.0f; // Only for debugging purpose.
                pathNodes.push_back(node);
                path_.push_back(iState_);
                doorStates_.Element[iState_] = startDoorState;
            }
            doorStates_.n = nStates + poses_G_0_via.n + 1;
#endif
            // Door openning path.

            // if (iLastNode == iBestEndNode)
            //     int debug = 0;

            doorStates_.n = nStates + robotJoints.h;
            for (iState = 0; iState < nStates; iState++)
            {
                iNodeJS_ = pNodeJS->path[robotJoints.h - 1];
                pNodeJS_ = nodeJS + iNodeJS_;
                SetEnvironmentState(doorStates.Element[iState]);
                RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pose_DD_S.R, pose_DD_S.t, pose_DD_0.R, pose_DD_0.t, V3Tmp);
                pNode = nodes.Element + pNodeJS_->iContactNode;
                RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
                robot.InvKinematics(pose_G_0, IKSolutions, true);
                IKSolutionIdx = iNodeJS_ % maxnIKSolutions;
                for (iIKSolution = 0; iIKSolution < IKSolutions.n; iIKSolution++)
                    if (IKSolutions.Element[iIKSolution].i == IKSolutionIdx)
                        break;
                if (iIKSolution >= IKSolutions.n)
                {
                    q = robotJoints.Element + robotJoints.w * (robotJoints.h - 1);
                    iClosestIKSolution = -1;
                    for (iIKSolution = 0; iIKSolution < IKSolutions.n; iIKSolution++)
                    {
                        RVLMOTION_JOINT_SPACE_DIST(IKSolutions.Element[iIKSolution].q, q, dq, dCost, i);
                        if (iClosestIKSolution < 0 || dCost < minCost)
                        {
                            iClosestIKSolution = iIKSolution;
                            minCost = dCost;
                        }
                    }
                    iIKSolution = iClosestIKSolution;
                }
                q = robotJoints.Element + robotJoints.w * robotJoints.h;
                memcpy(q, IKSolutions.Element[iIKSolution].q, 6 * sizeof(float));

                // Only for debugging purpose!!!

                // Pose3D pose_6_0;
                // pose_6_0 = robot.link_pose[5];
                // memcpy(robot.q, q, robot.n * sizeof(float));
                // robot.FwdKinematics();

                //

                node.pose.pose = pose_G_0;
                node.PRTCP[0] = pNode->PRTCP[0];
                node.PRTCP[1] = pNode->PRTCP[1]; // Only for debugging purpose.
                pathNodes.push_back(node);
                path_.push_back(robotJoints.h);
                doorStates_.Element[robotJoints.h] = doorStates.Element[iState];
                robotJoints.h++;
            }

            if (iLastNode == iBestEndNode)
            {
                poses_G_0.n = pathNodes.size();
                poses_G_0.Element = new Pose3D[poses_G_0.n];
                for (iNode = 0; iNode < poses_G_0.n; iNode++)
                    poses_G_0.Element[iNode] = pathNodes[iNode].pose.pose;
            }

            if (pFeasiblePaths)
            {
                pPath = pFeasiblePaths->Element + iPath;
                pPath->n = pathNodes.size();
                pPath->Element = pathPosesMem + maxnPathPoints * iPath;
                for (iNode = 0; iNode < pPath->n; iNode++)
                    pPath->Element[iNode] = pathNodes[iNode].pose.pose;
            }

            if (pFeasiblePathsJoints)
            {
                pPathJoints = pFeasiblePathsJoints->Element + iPath;
                pPathJoints->w = robotJoints.w;
                pPathJoints->h = robotJoints.h;
                pPathJoints->Element = pathJointsMem + maxnPathPoints * robotJoints.w * iPath;
                memcpy(pPathJoints->Element, robotJoints.Element, robotJoints.h * robotJoints.w * sizeof(float));
            }

            ///

#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
            double pathPalnningTime;
            if (pTimer)
            {
                pTimer->Stop();
                pathPalnningTime = pTimer->GetTime();
                // printf("Path planned in %lf s.\n", 0.001 * pathPalnningTime);
            }
#endif

            if (iLastNode == iBestEndNode)
            {
                // Visualization.

                if (pVisualizationData->bVisualize)
                    Visualize(&pathNodes, &path_, doorStates_, false, true, false, -1, &robotJoints);
                // pVisualizationData->pVisualizer->Run();

                // Write results to a file.

                if (bLog && resultsFolder)
                {
                    FILE *fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "path.txt").data(), "w");
                    for (iState = 0; iState < robotJoints.h - 1; iState++)
                    {
                        iNode = pNodeJS->path[iState];
                        fprintf(fpLog, "%d ", iNode);
                    }
                    fprintf(fpLog, "\n");
                    fclose(fpLog);
                }
            }

            pathNodes.clear();
            path_.clear();
        }

        delete[] doorStates_.Element;
        delete[] IKSolutions.Element;
    }
    else
    {
        poses_G_0.Element = NULL;
        robotJoints.Element = NULL;
    }

    //

    delete[] nodeJSMem;
    delete[] pathMem_;
    delete[] doorStates.Element;
    delete[] feasibleNodeMem;
    delete[] SDF;
    delete[] approachPathJSMem;

    return bPath;
}

bool DDManipulator::Path3(
    float *qInit,
    float endDoorState,
    int nStates,
    Array<Pose3D> &poses_G_0,
    Array2D<float> &robotJoints,
    Array<Array<Pose3D>> *pFeasiblePaths,
    Array<Array2D<float>> *pFeasiblePathsJoints)
{
    // Parameters.

    float startDoorState = RAD2DEG * dd_state_angle; // deg
    int maxnNodesJS = 200000;
    int maxnNodesCPerStep = 100;
    bPath3 = true;
    bool bRandomTree = true;
    int newRootNodeProbability = 20; // %

    // Constants.

    float dDoorState = (nStates > 1 ? (endDoorState - startDoorState) / (float)(nStates - 1) : 0.0f);
    int nStates_ = nStates + 2;
    int iLastState = nStates - 1;
    int maxnNodesJS_ = maxnNodesJS + 3 * maxnIKSolutions + 1;

    // Adapt the contact pose graph to the target door.

    AdaptContactPoseGraph();

    // Set door initial state.

    SetEnvironmentState(startDoorState);

    // Free space planes.

    FreeSpacePlanes();

    // Allocate memory for JS nodes.

    Array<MOTION::NodeJS> nodesJS;
    nodesJS.Element = new MOTION::NodeJS[maxnNodesJS_];
    nodesJS.n = 0;

    // Initial JS node.

    MOTION::NodeJS *pNodeJS = nodesJS.Element;
    pNodeJS->bFeasible = true;
    pNodeJS->cost = 0.0f;
    pNodeJS->iContactNode = -1;
    pNodeJS->IK.i = -1;
    memcpy(pNodeJS->IK.q, qInit, 6 * sizeof(float));
    pNodeJS->iPrevNode = -1;
    pNodeJS->iState = -1;
    nodesJS.n++;

    // Initialize the default tool model.

    if (bDefaultToolModel)
    {
        float tool_hand_len = tool_finger_size.Element[2] + tool_palm_size.Element[2];
        RVLSET3VECTOR(default_tool_P1_G, 0.0f, 0.0f, -tool_hand_len);
        RVLSET3VECTOR(default_tool_P2_G, 0.0f, 0.0f, -(tool_hand_len + tool_wrist_len));
    }

    /// Search for feasible paths.

    int nodeSpaceSize = nStates_ * selectedNodes.n;
    NodeSpaceElement *nodeSpace = new NodeSpaceElement[nodeSpaceSize];
    int iNodeSpaceElement;
    NodeSpaceElement *pNodeSpaceElement = nodeSpace;
    QList<MOTION::NodeJS> *pNodeJSList;
    for (iNodeSpaceElement = 0; iNodeSpaceElement < nodeSpaceSize; iNodeSpaceElement++, pNodeSpaceElement++)
    {
        pNodeSpaceElement->bExplored = false;
        pNodeJSList = &(pNodeSpaceElement->nodesJS);
        RVLQLIST_INIT(pNodeJSList);
    }
    Array<int> *explore = new Array<int>[nStates];
    int *exploreMem = NULL;
    int iState;
    int iNodeC;
    int iRndIdx = 0;
    int *nExplored;
    Array<Pair<int, int>> nodesCToExpand;
    nodesCToExpand.Element = NULL;
    Array<int> nextStateNeighbors;
    nextStateNeighbors.Element = NULL;
    if (bRandomTree)
    {
        nodesCToExpand.Element = new Pair<int, int>[maxnNodesJS];
        nodesCToExpand.n = 0;
        nextStateNeighbors.Element = new int[maxnContactPoseGraphNeighbors * maxnIKSolutions];
    }
    else
    {
        exploreMem = new int[nStates * selectedNodes.n];
        for (iState = 0; iState < nStates; iState++)
        {
            explore[iState].Element = exploreMem + iState * selectedNodes.n;
            explore[iState].n = selectedNodes.n;
            for (iNodeC = 0; iNodeC < selectedNodes.n; iNodeC++)
                explore[iState].Element[iNodeC] = iNodeC;
            Permute(rndIdx, iRndIdx, explore[iState]);
            iRndIdx = (iRndIdx + selectedNodes.n) % rndIdx.n;
        }
        // nExplored = new int[nStates];
        // memset(nExplored, 0, nStates * sizeof(int));
    }

    float *SDF = new float[pVNEnv->NodeArray.n];
    Pose3D pose_G_0;
    float doorState = startDoorState;
    GRAPH::EdgePtr<Edge> *pEdgePtr;
    float dCost;
    float dq[6];
    float cost;
    float minCost = 0.0f;
    MOTION::NodeJS *pMinCostNeighbor;
    int iState_;
    Array<float> doorStates;
    doorStates.n = nStates;
    doorStates.Element = new float[nStates];
    float V3Tmp[3];
    Array<Pose3D> poses_G_0_via;
    Pose3D viaPtPosesMem[2];
    poses_G_0_via.Element = viaPtPosesMem;
    float posCost;
    float PRTCP_DD[3];
    bool bAllNeighbors;
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>> *pGNode;
    MOTION::Edge *pEdge;
    MOTION::Node *pNode, pNode_;
    int iCandidateNode;
    int iFeasibleNode_;
    int iNodeJS_;
    MOTION::NodeJS *pNodeJSTmp;
    float chebDist;
    float fTmp;
    MOTION::NodeJS *nodesJS_;
    Array<int> IKSolutionIdxs;
    IKSolutionIdxs.Element = new int[maxnIKSolutions];
    Array<MOTION::IKSolution> approachPathsJS;
    approachPathsJS.Element = new MOTION::IKSolution[2 * maxnIKSolutions];
    int nApproachPathViaPts;
    int IKSolutionIdx;
    int iNodeC_;
    int iNodeG, iNodeG_;
    MOTION::NodeJS *pNodeJS_;
    int i;
    NodeSpaceElement *pNodeSpaceElement_;
    NodeSpaceElement *stateNodeSpace;
    NodeSpaceElement *prevStateNodeSpace = NULL;
    NodeSpaceElement *nextStateNodeSpace;
    Array<MOTION::NodeJS *> feasibleNeighbors;
    feasibleNeighbors.Element = new MOTION::NodeJS *[maxnContactPoseGraphNeighbors * maxnIKSolutions];
    bool bExploreNodeC;
    bool bConnectedNodeC;
    int iIKSolution;
    int nConnectedNodesC;
    bool bNodeJSAdded;
    MOTION::IKSolution *pApproachPtJS;
    MOTION::NodeJS *approachNodesJS;
    MOTION::NodeJS *pApproachNodeJS;
    int iiNodeC;
    int i_;
    Array<MOTION::NodeJS *> paths;
    paths.Element = new MOTION::NodeJS *[selectedNodes.n * maxnIKSolutions];
    paths.n = 0;
    iState = 0;
    int nNodesCToExplore;
    Pair<int, int> *pNodeCToExpand;
    float qDist;
    int rndVal;
    while (true)
    {
        if (bRandomTree)
        {
            rndVal = rndIdx.Element[iRndIdx] % 100;
            iRndIdx = (iRndIdx + 1) % rndIdx.n;
            if (rndVal < newRootNodeProbability || nodesCToExpand.n == 0)
            {
                iState = 0;
                iNodeC = rndIdx.Element[iRndIdx] % selectedNodes.n;
                iRndIdx = (iRndIdx + 1) % rndIdx.n;
            }
            else
            {
                pNodeCToExpand = nodesCToExpand.Element + rndIdx.Element[iRndIdx] % nodesCToExpand.n;
                iRndIdx = (iRndIdx + 1) % rndIdx.n;
                iState = pNodeCToExpand->a;
                iNodeC = pNodeCToExpand->b;
                stateNodeSpace = nodeSpace + iState * selectedNodes.n;
                if (iState < iLastState)
                {
                    iState++;
                    nextStateNeighbors.Element[0] = iNodeC;
                    nextStateNeighbors.n = 1;
                    nextStateNodeSpace = nodeSpace + iState * selectedNodes.n;
                    iNodeG = selectedNodes.Element[iNodeC];
                    pGNode = graph.NodeArray.Element + iNodeG;
                    pEdgePtr = pGNode->EdgeList.pFirst;
                    while (pEdgePtr)
                    {
                        RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNodeG, pEdgePtr, pEdge, iNodeG_);
                        iNodeC_ = contactNode[iNodeG_];
                        if (iNodeC_ >= 0)
                            nextStateNeighbors.Element[nextStateNeighbors.n++] = iNodeC_;
                        pEdgePtr = pEdgePtr->pNext;
                    }
                    if (nextStateNeighbors.n > 0)
                    {
                        iNodeC = (nextStateNeighbors.n == 1 ? nextStateNeighbors.Element[0] : nextStateNeighbors.Element[rndIdx.Element[iRndIdx] % nextStateNeighbors.n]);
                        iRndIdx = (iRndIdx + 1) % rndIdx.n;
                    }
                }
            }
            nNodesCToExplore = 1;
        }
        else
            nNodesCToExplore = explore[iState].n;

        // if (iNodeC == 68077)
        //     int debug = 0;

        // printf(".");

        iState_ = iState + 2;

        // if (iState == iLastState)
        //     int debug = 0;

        // Set the door state.

        doorState = startDoorState + (float)iState * dDoorState;
        SetEnvironmentState(doorState);
        doorStates.Element[iState] = doorState;
        RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pose_DD_S.R, pose_DD_S.t, pose_DD_0.R, pose_DD_0.t, V3Tmp);

        // Search for feasible nodes.

        nConnectedNodesC = 0;
        stateNodeSpace = nodeSpace + iState * selectedNodes.n;
        prevStateNodeSpace = (iState == 0 ? NULL : nodeSpace + (iState - 1) * selectedNodes.n);
        for (iiNodeC = 0; iiNodeC < nNodesCToExplore; iiNodeC++)
        {
            if (!bRandomTree)
                iNodeC = explore[iState].Element[iiNodeC];
            pNodeSpaceElement = stateNodeSpace + iNodeC;
            if (pNodeSpaceElement->bExplored)
                continue;
            if (iState == 0)
                bExploreNodeC = true;
            else
            {
                Neighbors(iNodeC, prevStateNodeSpace, feasibleNeighbors);
                bExploreNodeC = (feasibleNeighbors.n > 0);
            }
            if (!bExploreNodeC)
                continue;
            // if (nodesJS.n == 26)
            //     int debug = 0;
            pNodeSpaceElement->bExplored = true;
            bConnectedNodeC = false;
            pNodeJS = nodesJS.Element + nodesJS.n;
            iNodeG = selectedNodes.Element[iNodeC];
            pNode = nodes.Element + iNodeG;
            RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
            if (!FeasiblePose(&pose_G_0, SDF, pNodeJS, IKSolutionIdxs, iState == 0, &approachPathsJS, &nApproachPathViaPts))
                continue;
            // if (nodesJS.n == 3)
            //     int debug = 0;
            nodesJS.n += IKSolutionIdxs.n;
            bNodeJSAdded = false;
            pApproachPtJS = approachPathsJS.Element;
            approachNodesJS = nodesJS.Element + nodesJS.n;
            pNodeJSList = &(pNodeSpaceElement->nodesJS);
            if (bRandomTree)
                if (IKSolutionIdxs.n > 0)
                {
                    pNodeCToExpand = nodesCToExpand.Element + nodesCToExpand.n;
                    pNodeCToExpand->a = iState;
                    pNodeCToExpand->b = iNodeC;
                    nodesCToExpand.n++;
                }
            for (iIKSolution = 0; iIKSolution < IKSolutionIdxs.n; iIKSolution++, pNodeJS++)
            {
                if (!pNodeJS->bFeasible)
                    continue;
                // Only for debugging purpose!!!
                // Pose3D pose_G_0_;
                // robot.FwdKinematics(pNodeJS->IK.q, &pose_G_0_);
                //
                pNodeJS->iContactNode = iNodeC;
                pNodeJS->iState = iState;
                bNodeJSAdded = true;
                if (iState == 0)
                {
                    if (nApproachPathViaPts == 0)
                        pNodeJS->iPrevNode = 0;
                    else
                    {
                        pNodeJS->iPrevNode = nodesJS.n + nApproachPathViaPts - 1;
                        for (i = 0; i < nApproachPathViaPts; i++)
                        {
                            pApproachNodeJS = approachNodesJS + i;
                            pApproachNodeJS->bFeasible = true;
                            pApproachNodeJS->iContactNode = -1;
                            pApproachNodeJS->iState = -1;
                            pApproachNodeJS->IK = *(pApproachPtJS++);
                        }
                        approachNodesJS[nApproachPathViaPts - 1].iPrevNode = nodesJS.n;
                        approachNodesJS[0].iPrevNode = 0;
                        nodesJS.n += nApproachPathViaPts;
                        approachNodesJS += nApproachPathViaPts;
                    }
                    bConnectedNodeC = true;
                }
                else
                {
                    pNodeJS->iPrevNode = -1;
                    pMinCostNeighbor = NULL;
                    for (i = 0; i < feasibleNeighbors.n; i++)
                    {
                        pNodeJS_ = feasibleNeighbors.Element[i];
                        RVLMOTION_JOINT_SPACE_CHEB_DIST(pNodeJS->IK.q, pNodeJS_->IK.q, dq, qDist, fTmp, i_);
                        if (qDist <= JSDistThr)
                        {
                            RVLMOTION_JOINT_SPACE_DIST(pNodeJS->IK.q, pNodeJS_->IK.q, dq, dCost, i_);
                            cost = dCost + pNodeJS_->cost;
                            if (cost < minCost || pMinCostNeighbor == NULL)
                            {
                                minCost = cost;
                                pMinCostNeighbor = pNodeJS_;
                            }
                        }
                    }
                    if (pMinCostNeighbor)
                    {
                        posCost = posCostMaxDist - RVLMIN(pNode->PRTCP[0], pNode->PRTCP[1]);
                        if (posCost < 0.0f)
                            posCost = 0.0f;
                        pNodeJS->cost = minCost + wPos * posCost;
                        pNodeJS->iPrevNode = pMinCostNeighbor - nodesJS.Element;
                        pNodeJS->bFeasible = true;
                        bConnectedNodeC = true;
                        if (iState == iLastState)
                            paths.Element[paths.n++] = pNodeJS;
                    }
                    else
                        pNodeJS->bFeasible = false;
                }
                RVLQLIST_ADD_ENTRY(pNodeJSList, pNodeJS);
            }
            if (bConnectedNodeC)
            {
                if (!bRandomTree)
                {
                    nConnectedNodesC++;
                    if (nConnectedNodesC >= maxnNodesCPerStep)
                        break;
                }

                if (nodesJS.n >= maxnNodesJS)
                    break;
            }
        }

        if (bRandomTree)
        {
            if (iState == iLastState)
            {
                if (bConnectedNodeC)
                    break;
            }
        }
        else
        {
            if (nConnectedNodesC == 0)
                break;

            iState++;
            if (iState >= nStates)
                break;
        }
        if (nodesJS.n >= maxnNodesJS)
            break;
    }

    ///

    // Find the optimal path.

    int iPath;
    MOTION::NodeJS *pOptimalPath = NULL;
    for (iPath = 0; iPath < paths.n; iPath++)
    {
        pNodeJS = paths.Element[iPath];
        if (pOptimalPath == NULL || pNodeJS->cost < minCost)
        {
            minCost = pNodeJS->cost;
            pOptimalPath = pNodeJS;
        }
    }

    // Copy the resulting path to the output format.

    if (pOptimalPath)
    {
        Array<MOTION::NodeJS *> path;
        path.Element = new MOTION::NodeJS *[nStates + 3];
        path.n = 0;
        pNodeJS = pOptimalPath;
        while (true)
        {
            path.Element[path.n++] = pNodeJS;
            if (pNodeJS->iPrevNode >= 0)
                pNodeJS = nodesJS.Element + pNodeJS->iPrevNode;
            else
                break;
        }
        poses_G_0.n = path.n;
        poses_G_0.Element = new Pose3D[poses_G_0.n];
        robotJoints.w = robot.n;
        robotJoints.Element = new float[robotJoints.w * path.n];
        robotJoints.h = path.n;
        Array<float> doorStates_;
        doorStates_.Element = new float[path.n];
        doorStates_.n = 0;
        Pose3D *pPose_n_0 = robot.link_pose + robot.n - 1;
        for (i = 0; i < path.n; i++)
        {
            pNodeJS = path.Element[path.n - i - 1];
            doorStates_.Element[doorStates_.n] = (pNodeJS->iState >= 0 ? doorStates.Element[pNodeJS->iState] : startDoorState);
            SetEnvironmentState(doorStates_.Element[doorStates_.n]);
            doorStates_.n++;
            iNodeC = pNodeJS->iContactNode;
            if (iNodeC >= 0)
            {
                iNodeG = selectedNodes.Element[iNodeC];
                pNode = nodes.Element + iNodeG;
                RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pose_DD_S.R, pose_DD_S.t, pose_DD_0.R, pose_DD_0.t, V3Tmp);
                RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
                // Only for debugging purpose!!!
                // memcpy(robot.q, pNodeJS->IK.q, robot.n * sizeof(float));
                // robot.FwdKinematics();
                // Pose3D pose_G_0_;
                // RVLCOMPTRANSF3D(pPose_n_0->R, pPose_n_0->t, robot.pose_TCP_6.R, robot.pose_TCP_6.t, pose_G_0_.R, pose_G_0_.t);
                // int debug = 0;
                //
            }
            else
                robot.FwdKinematics(pNodeJS->IK.q, &pose_G_0);
            poses_G_0.Element[i] = pose_G_0;
            memcpy(robotJoints.Element + robotJoints.w * i, pNodeJS->IK.q, 6 * sizeof(float));
        }

        // Visualization.

        std::vector<Node> pathNodes;
        std::vector<int> path_;
        Node nodeC;
        for (i = 0; i < path.n; i++)
        {
            nodeC.pose.pose = poses_G_0.Element[i];
            // Only for debugging purpose.
            pNodeJS = path.Element[path.n - i - 1];
            iNodeC = pNodeJS->iContactNode;
            if (iNodeC >= 0)
            {
                pNode = nodes.Element + selectedNodes.Element[iNodeC];
                RVLCOPY2VECTOR(pNode->PRTCP, nodeC.PRTCP);
            }
            else
                RVLNULL2VECTOR(nodeC.PRTCP)
            //
            pathNodes.push_back(nodeC);
            path_.push_back(i);
        }
        if (pVisualizationData->bVisualize)
            Visualize(&pathNodes, &path_, doorStates_, false, true, false, -1, &robotJoints);

        //

        delete[] doorStates_.Element;

        // Write results to a file.

        if (bLog && resultsFolder)
        {
            FILE *fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "path.txt").data(), "w");
            for (i = 0; i < path.n; i++)
            {
                pNodeJS = path.Element[path.n - i - 1];
                fprintf(fpLog, "%d ", pNodeJS - nodesJS.Element);
            }
            fprintf(fpLog, "\n");
            fclose(fpLog);
        }

        //

        delete[] path.Element;
    }

    //

    delete[] nodesJS.Element;
    delete[] nodeSpace;
    delete[] SDF;
    delete[] doorStates.Element;
    delete[] approachPathsJS.Element;
    delete[] IKSolutionIdxs.Element;
    delete[] explore;
    RVL_DELETE_ARRAY(exploreMem);
    // delete[] nExplored;
    delete[] feasibleNeighbors.Element;
    delete[] paths.Element;
    RVL_DELETE_ARRAY(nodesCToExpand.Element);
    RVL_DELETE_ARRAY(nextStateNeighbors.Element);

    return (pOptimalPath != NULL);
}

void DDManipulator::CreateContactPoseGraph(std::string contactPoseGraphFileName)
{
    printf("Creating contact pose graph...");

#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
    if (pTimer)
        pTimer->Start();
#endif

    // Parameters.

    float rPos = 0.050f;      // m
    float rOrientDeg = 15.0f; // deg
    float max_dd_size[2];
    max_dd_size[0] = 0.6f;
    max_dd_size[1] = 1.0f;

    // Constants.

    float rOrient = DEG2RAD * rOrientDeg;

    // Tile feasible contact poses.

    std::vector<MOTION::ContactPose> allFeasibleTCPs;
    Box<float> TCPSpace;
    TileFeasibleToolContactPoses(&allFeasibleTCPs, max_dd_size, TCPSpace);
    // printf("num. allFeasibleTCPs=%d\n", allFeasibleTCPs.size());
    // printf("opening direction=%f\n", dd_opening_direction);

    // Only for debugging purpose!!!

    // RVLNULLMX3X3(pNode->pose.pose.R);
    // RVLMXEL(pNode->pose.pose.R, 3, 0, 2) = 1.0f;
    // RVLMXEL(pNode->pose.pose.R, 3, 1, 1) = -1.0f;
    // RVLMXEL(pNode->pose.pose.R, 3, 2, 0) = 1.0f;
    // float RTmp1[9];
    // RVLNULLMX3X3(RTmp1);
    // RVLMXEL(RTmp1, 3, 0, 1) = 1.0f;
    // RVLMXEL(RTmp1, 3, 1, 2) = 1.0f;
    // RVLMXEL(RTmp1, 3, 2, 0) = 1.0f;
    // float RTmp2[9];
    // float phi = 0.5f;
    // RVLROTY(cos(phi), sin(phi), RTmp2);
    // Pose3D *pPose_G_DD = (Pose3D *)(allFeasibleTCPs.data());
    // RVLMXMUL3X3(RTmp1, RTmp2, pPose_G_DD->R);
    // float PRTCP_DD[3];
    // RVLSET3VECTOR(PRTCP_DD, 0.0f, 0.012f, 0.015f);
    // float V3TmpDebug[3];
    // RVLMULMX3X3VECT(pPose_G_DD->R, PRTCP_G, V3TmpDebug);
    // RVLDIF3VECTORS(PRTCP_DD, V3TmpDebug, pPose_G_DD->t);
    // float T_G_DD[16];
    // RVLHTRANSFMX(pPose_G_DD->R, pPose_G_DD->t, T_G_DD);
    // FILE *fp = fopen("T_G_DD.txt", "w");
    // PrintMatrix<float>(fp, T_G_DD, 4, 4);
    // fclose(fp);

    //

    // SE3 grid.

    SE3Grid grid;
    ExpandBox<float>(&TCPSpace, 2.0f * rPos);
    grid.Create(TCPSpace);

    // Pose graph.

    RVL_DELETE_ARRAY(graph.NodeMem);
    graph.NodeMem = new GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>[allFeasibleTCPs.size()];
    graph.NodeArray.Element = graph.NodeMem;
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>> *pGNode;
    int iSE3Point;
    int iSE3Point_;
    int i;
    int iPosCell, iZ, iRoll;
    RVL_DELETE_ARRAY(nodes.Element);
    nodes.Element = new MOTION::Node[allFeasibleTCPs.size()];
    nodes.n = 0;
    MOTION::Node *pNode;
    MOTION::ContactPose contactPose;
    QList<GRAPH::EdgePtr<MOTION::Edge>> *pEdgeList;
    for (iSE3Point = 0; iSE3Point < allFeasibleTCPs.size(); iSE3Point++)
    {
        // if (i % 1000 == 0)
        //     int debug = 0;
        contactPose = allFeasibleTCPs[iSE3Point];
        // if(iSE3Point == 0)
        //     int debug = 0;
        // else
        {
            // if (pose_G_DD.t[0] < -0.05f)
            //     int debug = 0;
            iSE3Point_ = grid.Fetch(contactPose.pose_G_DD, iPosCell, iZ, iRoll);
            if (iSE3Point_ >= 0)
                continue;
        }
        // if (nodes.n == 2412533 / 8)
        //     int debug = 0;
        pNode = nodes.Element + nodes.n;
        pNode->pose.pose = contactPose.pose_G_DD;
        pNode->PRTCP[0] = contactPose.PRTCP_DD[0];
        pNode->PRTCP[1] = contactPose.PRTCP_DD[1];
        pNode->iSE3Point = iSE3Point; // For debugging purpose.
        pNode->cost = 0.0f;
        pGNode = graph.NodeArray.Element + nodes.n;
        pEdgeList = &(pGNode->EdgeList);
        RVLQLIST_INIT(pEdgeList);
        pNode->pGNode = pGNode;
        pGNode->idx = nodes.n;
        pNode->iParent = -1;
        pNode->iFirstChild = -1;
        pNode->iSibling = -1;
        pNode->flags = 0x00;
        nodes.n++;
        grid.Add(contactPose.pose_G_DD, true, pGNode->idx, iPosCell, iZ, iRoll);
    }
    int iNode, iNode_;
    // Only for debugging purpose!!!
    // Array<Point> pts;
    // pts.Element = new Point[2 * nodes.n];
    // Array<Pair<int, int>> lines;
    // lines.n = nodes.n;
    // lines.Element = new Pair<int, int>[nodes.n];
    // float *P1, *P2;
    // for(iNode = 0; iNode < nodes.n; iNode++)
    // {
    //    pNode = nodes.Element + iNode;
    //    P1 = pts.Element[iNode].P;
    //    RVLCOPY3VECTOR(pNode->pose.pose.t, P1);
    //    P2 = pts.Element[nodes.n + iNode].P;
    //    RVLTRANSF3(PRTCP_G, pNode->pose.pose.R, pNode->pose.pose.t, P2);
    //    lines.Element[iNode].a = iNode;
    //    lines.Element[iNode].b = nodes.n + iNode;
    // }
    // pts.n = 2 * nodes.n;
    // uchar green[] = {0, 255, 0};
    // pVisualizationData->pVisualizer->DisplayLines(pts, lines, green);
    // uchar blue[] = {0, 0, 255};
    // pts.n = nodes.n;
    // pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pts, blue, 6);
    // pVisualizationData->pVisualizer->Run();
    // pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
    // delete[] pts.Element;
    // delete[] lines.Element;
    //
    Array<int> TCPNeighbors;
    MOTION::Edge *pEdge;
    MOTION::Node *pNode_;
    Array<Pair<int, int>> edges;
    edges.n = 0;
    // int debug = 0;
    for (iNode = 0; iNode < nodes.n; iNode++)
    {
        pNode = nodes.Element + iNode;
        // Only for debugging purpose!!!
        // if(iNode == 29679)
        // {
        //     float csRotDiff;
        //     float maxcsRotDiff = -2.0f;
        //     int debug = 0;
        //     for(iNode_ = 0; iNode_ < nodes.n; iNode_++)
        //     {
        //         if(iNode_ == iNode)
        //             continue;
        //         pNode_ = nodes.Element + iNode_;
        //         RVLDIF3VECTORS(pNode->pose.pose.t, pNode_->pose.pose.t, V3TmpDebug);
        //         // if(RVLDOTPRODUCT3(V3TmpDebug, V3TmpDebug) > rPos * rPos)
        //         //     continue;
        //         debug++;
        //         float dR[9];
        //         RVLMXMUL3X3T1(pNode->pose.pose.R, pNode_->pose.pose.R, dR);
        //         csRotDiff = RVLROTDIFF(dR);
        //         if(csRotDiff >= maxcsRotDiff)
        //             maxcsRotDiff = csRotDiff;
        //     }
        //     float maxRotDiff = acos(maxcsRotDiff);
        //     debug = 0;
        // }
        //
        pGNode = pNode->pGNode;
        grid.Neighbors(pNode->pose.pose, rPos, rOrient, TCPNeighbors);
        // debug += TCPNeighbors.n;
        for (i = 0; i < TCPNeighbors.n; i++)
        {
            iNode_ = TCPNeighbors.Element[i];
            pNode_ = nodes.Element + iNode_;
            if (iNode < iNode_)
            {
                pEdge = ConnectNodes<GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, MOTION::Edge, GRAPH::EdgePtr<MOTION::Edge>>(pGNode, pNode_->pGNode, iNode, iNode_, pMem);
                edges.n++;
            }
        }
    }

#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
    double graphCreationTime;
    if (pTimer)
    {
        pTimer->Stop();
        graphCreationTime = pTimer->GetTime();
        printf("Graph created in %lf s.\n", 0.001 * graphCreationTime);
    }
#endif

    printf("completed\n");

    // Save contact pose graph to file.

    FILE *fp = fopen(contactPoseGraphFileName.c_str(), "wb");
    fwrite(&nodes.n, sizeof(int), 1, fp);
    fwrite(nodes.Element, sizeof(MOTION::Node), nodes.n, fp);
    fwrite(&edges.n, sizeof(int), 1, fp);
    edges.Element = new Pair<int, int>[edges.n];
    Pair<int, int> edge;
    Pair<int, int> *pEdge_ = edges.Element;
    pGNode = graph.NodeMem;
    GRAPH::EdgePtr<Edge> *pEdgePtr;
    for (iNode = 0; iNode < nodes.n; iNode++, pGNode++)
    {
        pEdgePtr = pGNode->EdgeList.pFirst;
        while (pEdgePtr)
        {
            edge.a = iNode;
            RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, edge.b);
            if (edge.a < edge.b)
            {
                *pEdge_ = edge;
                pEdge_++;
            }
            pEdgePtr = pEdgePtr->pNext;
        }
    }
    fwrite(edges.Element, sizeof(Pair<int, int>), edges.n, fp);
    delete[] edges.Element;
    fclose(fp);
}

void DDManipulator::CreateContactPoseGraph2(std::string contactPoseGraphFileName)
{
    // printf("Creating contact pose graph...");

#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
    if (pTimer)
        pTimer->Start();
#endif

    // Parameters.

    float rPos = 0.050f;      // m
    float rOrientDeg = 15.0f; // deg
    float max_dd_size[2];
    max_dd_size[0] = dd_sy*0.5f;
    max_dd_size[1] = dd_sz;

    // Constants.

    float rOrient = DEG2RAD * rOrientDeg;

    // Tile feasible contact poses.

    std::vector<MOTION::ContactPose> allFeasibleTCPs;
    Box<float> TCPSpace;
    TileFeasibleToolContactPoses(&allFeasibleTCPs, max_dd_size, TCPSpace);
    // printf("num. allFeasibleTCPs=%d\n", allFeasibleTCPs.size());
    // printf("opening direction=%f\n", dd_opening_direction);

    // Only for debugging purpose!!!

    // RVLNULLMX3X3(pNode->pose.pose.R);
    // RVLMXEL(pNode->pose.pose.R, 3, 0, 2) = 1.0f;
    // RVLMXEL(pNode->pose.pose.R, 3, 1, 1) = -1.0f;
    // RVLMXEL(pNode->pose.pose.R, 3, 2, 0) = 1.0f;
    // float RTmp1[9];
    // RVLNULLMX3X3(RTmp1);
    // RVLMXEL(RTmp1, 3, 0, 1) = 1.0f;
    // RVLMXEL(RTmp1, 3, 1, 2) = 1.0f;
    // RVLMXEL(RTmp1, 3, 2, 0) = 1.0f;
    // float RTmp2[9];
    // float phi = 0.5f;
    // RVLROTY(cos(phi), sin(phi), RTmp2);
    // Pose3D *pPose_G_DD = (Pose3D *)(allFeasibleTCPs.data());
    // RVLMXMUL3X3(RTmp1, RTmp2, pPose_G_DD->R);
    // float PRTCP_DD[3];
    // RVLSET3VECTOR(PRTCP_DD, 0.0f, 0.012f, 0.015f);
    // float V3TmpDebug[3];
    // RVLMULMX3X3VECT(pPose_G_DD->R, PRTCP_G, V3TmpDebug);
    // RVLDIF3VECTORS(PRTCP_DD, V3TmpDebug, pPose_G_DD->t);
    // float T_G_DD[16];
    // RVLHTRANSFMX(pPose_G_DD->R, pPose_G_DD->t, T_G_DD);
    // FILE *fp = fopen("T_G_DD.txt", "w");
    // PrintMatrix<float>(fp, T_G_DD, 4, 4);
    // fclose(fp);

    //

    // SE3 grid.

    SE3Grid grid;
    ExpandBox<float>(&TCPSpace, 2.0f * rPos);
    grid.Create(TCPSpace);

    // Pose graph.

    RVL_DELETE_ARRAY(graph.NodeMem);
    graph.NodeMem = new GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>[allFeasibleTCPs.size()];
    graph.NodeArray.Element = graph.NodeMem;
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>> *pGNode;
    int iSE3Point;
    int iSE3Point_;
    int i;
    int iPosCell, iZ, iRoll;
    RVL_DELETE_ARRAY(nodes.Element);
    nodes.Element = new MOTION::Node[allFeasibleTCPs.size()];
    nodes.n = 0;
    MOTION::Node *pNode;
    MOTION::ContactPose contactPose;
    QList<GRAPH::EdgePtr<MOTION::Edge>> *pEdgeList;
    
    // RVL_DELETE_ARRAY(selectedNodes.Element);
    // selectedNodes.Element = new int[allFeasibleTCPs.size()];
    // selectedNodes.n = 0;
    // RVL_DELETE_ARRAY(bSelected)
    // bSelected = new bool[allFeasibleTCPs.size()];
    // RVL_DELETE_ARRAY(contactNode);
    // contactNode = new int[allFeasibleTCPs.size()];
    // memset(contactNode, 0xff, allFeasibleTCPs.size() * sizeof(int));

    for (iSE3Point = 0; iSE3Point < allFeasibleTCPs.size(); iSE3Point++)
    {
        // if (i % 1000 == 0)
        //     int debug = 0;
        contactPose = allFeasibleTCPs[iSE3Point];
        // if(iSE3Point == 0)
        //     int debug = 0;
        // else
        {
            // if (pose_G_DD.t[0] < -0.05f)
            //     int debug = 0;
            iSE3Point_ = grid.Fetch(contactPose.pose_G_DD, iPosCell, iZ, iRoll);
            if (iSE3Point_ >= 0)
                continue;
        }
        // if (nodes.n == 2412533 / 8)
        //     int debug = 0;
        pNode = nodes.Element + nodes.n;
        pNode->pose.pose = contactPose.pose_G_DD;
        pNode->PRTCP[0] = contactPose.PRTCP_DD[0];
        pNode->PRTCP[1] = contactPose.PRTCP_DD[1];
        pNode->iSE3Point = iSE3Point; // For debugging purpose.
        pNode->cost = 0.0f;
        pGNode = graph.NodeArray.Element + nodes.n;
        pEdgeList = &(pGNode->EdgeList);
        RVLQLIST_INIT(pEdgeList);
        pNode->pGNode = pGNode;
        pGNode->idx = nodes.n;
        pNode->iParent = -1;
        pNode->iFirstChild = -1;
        pNode->iSibling = -1;
        pNode->flags = 0x00;
        nodes.n++;
        grid.Add(contactPose.pose_G_DD, true, pGNode->idx, iPosCell, iZ, iRoll);

        // selectedNodes.Element[selectedNodes.n] = nodes.n-1;
        // bSelected[nodes.n-1] = true;
        // contactNode[nodes.n-1] = selectedNodes.n;
        // selectedNodes.n++;
    }
    
    // for (iNode = 0; iNode < nodes.n; iNode++, pNode++)
    // {
    //     if (pNode->PRTCP[0] <= maxx && pNode->PRTCP[1] <= dd_panel_params[1])
    //     {
    //         selectedNodes.Element[selectedNodes.n] = iNode;
    //         bSelected[iNode] = true;
    //         contactNode[iNode] = selectedNodes.n;
    //         selectedNodes.n++;
    //     }
    //     else
    //         bSelected[iNode] = false;

    int iNode, iNode_;
    // Only for debugging purpose!!!
    // Array<Point> pts;
    // pts.Element = new Point[2 * nodes.n];
    // Array<Pair<int, int>> lines;
    // lines.n = nodes.n;
    // lines.Element = new Pair<int, int>[nodes.n];
    // float *P1, *P2;
    // for(iNode = 0; iNode < nodes.n; iNode++)
    // {
    //    pNode = nodes.Element + iNode;
    //    P1 = pts.Element[iNode].P;
    //    RVLCOPY3VECTOR(pNode->pose.pose.t, P1);
    //    P2 = pts.Element[nodes.n + iNode].P;
    //    RVLTRANSF3(PRTCP_G, pNode->pose.pose.R, pNode->pose.pose.t, P2);
    //    lines.Element[iNode].a = iNode;
    //    lines.Element[iNode].b = nodes.n + iNode;
    // }
    // pts.n = 2 * nodes.n;
    // uchar green[] = {0, 255, 0};
    // pVisualizationData->pVisualizer->DisplayLines(pts, lines, green);
    // uchar blue[] = {0, 0, 255};
    // pts.n = nodes.n;
    // pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(pts, blue, 6);
    // pVisualizationData->pVisualizer->Run();
    // pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
    // delete[] pts.Element;
    // delete[] lines.Element;
    //
    Array<int> TCPNeighbors;
    MOTION::Edge *pEdge;
    MOTION::Node *pNode_;
    Array<Pair<int, int>> edges;
    edges.n = 0;
    // int debug = 0;
    for (iNode = 0; iNode < nodes.n; iNode++)
    {
        pNode = nodes.Element + iNode;
        // Only for debugging purpose!!!
        // if(iNode == 29679)
        // {
        //     float csRotDiff;
        //     float maxcsRotDiff = -2.0f;
        //     int debug = 0;
        //     for(iNode_ = 0; iNode_ < nodes.n; iNode_++)
        //     {
        //         if(iNode_ == iNode)
        //             continue;
        //         pNode_ = nodes.Element + iNode_;
        //         RVLDIF3VECTORS(pNode->pose.pose.t, pNode_->pose.pose.t, V3TmpDebug);
        //         // if(RVLDOTPRODUCT3(V3TmpDebug, V3TmpDebug) > rPos * rPos)
        //         //     continue;
        //         debug++;
        //         float dR[9];
        //         RVLMXMUL3X3T1(pNode->pose.pose.R, pNode_->pose.pose.R, dR);
        //         csRotDiff = RVLROTDIFF(dR);
        //         if(csRotDiff >= maxcsRotDiff)
        //             maxcsRotDiff = csRotDiff;
        //     }
        //     float maxRotDiff = acos(maxcsRotDiff);
        //     debug = 0;
        // }
        //
        pGNode = pNode->pGNode;
        grid.Neighbors(pNode->pose.pose, rPos, rOrient, TCPNeighbors);
        // debug += TCPNeighbors.n;
        for (i = 0; i < TCPNeighbors.n; i++)
        {
            iNode_ = TCPNeighbors.Element[i];
            pNode_ = nodes.Element + iNode_;
            if (iNode < iNode_)
            {
                pEdge = ConnectNodes<GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, MOTION::Edge, GRAPH::EdgePtr<MOTION::Edge>>(pGNode, pNode_->pGNode, iNode, iNode_, pMem);
                edges.n++;
            }
        }
    }

#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
    double graphCreationTime;
    if (pTimer)
    {
        pTimer->Stop();
        graphCreationTime = pTimer->GetTime();
        printf("Graph created in %lf s.\n", 0.001 * graphCreationTime);
    }
#endif

    // printf("completed\n");

    // // Save contact pose graph to file.

    // FILE *fp = fopen(contactPoseGraphFileName.c_str(), "wb");
    // fwrite(&nodes.n, sizeof(int), 1, fp);
    // fwrite(nodes.Element, sizeof(MOTION::Node), nodes.n, fp);
    // fwrite(&edges.n, sizeof(int), 1, fp);
    // edges.Element = new Pair<int, int>[edges.n];
    // Pair<int, int> edge;
    // Pair<int, int> *pEdge_ = edges.Element;
    // pGNode = graph.NodeMem;
    // GRAPH::EdgePtr<Edge> *pEdgePtr;
    // for (iNode = 0; iNode < nodes.n; iNode++, pGNode++)
    // {
    //     pEdgePtr = pGNode->EdgeList.pFirst;
    //     while (pEdgePtr)
    //     {
    //         edge.a = iNode;
    //         RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, edge.b);
    //         if (edge.a < edge.b)
    //         {
    //             *pEdge_ = edge;
    //             pEdge_++;
    //         }
    //         pEdgePtr = pEdgePtr->pNext;
    //     }
    // }
    // fwrite(edges.Element, sizeof(Pair<int, int>), edges.n, fp);
    // delete[] edges.Element;
    // fclose(fp);
}


void DDManipulator::TileFeasibleToolContactPoses(
    std::vector<MOTION::ContactPose> *pAllFeasibleTCPs,
    float *max_dd_size,
    Box<float> &TCPSpace)
{
    // Only for debugging purpose!!!

    // Box<float> bbox;
    // InitBoundingBox<float>(&bbox, feasibleTCPs.Element[0].t);
    // for (int i = 1; i < feasibleTCPs.n; i++)
    //     UpdateBoundingBox<float>(&bbox, feasibleTCPs.Element[i].t);
    // int debug_ = 0;

    // Feasible contact poses.

    pAllFeasibleTCPs->reserve(feasibleTCPs.n);
    Pose3D *pPose_G_DD;
    int iTemplatePose;
    float templateEndTol = kTemplateEndTol * dd_contact_surface_sampling_resolution;
    float templateEnd[2];
    templateEnd[0] = dd_contact_surface_params[0] - templateEndTol;
    templateEnd[1] = dd_contact_surface_params[1] - templateEndTol;
    int iAxis;
    float s;
    Pose3D pose_A_S;
    RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_A_F.R, pose_A_F.t, pose_A_S.R, pose_A_S.t);
    Pose3D pose_Arot_S;
    RVLCOMPTRANSF3D(pose_A_S.R, pose_A_S.t, pose_Arot_A.R, pose_Arot_A.t, pose_Arot_S.R, pose_Arot_S.t);
    Pose3D pose_DD_S;
    RVLCOMPTRANSF3D(pose_Arot_S.R, pose_Arot_S.t, pose_DD_A.R, pose_DD_A.t, pose_DD_S.R, pose_DD_S.t);
    MOTION::ContactPose contactPoseTemplate;
    int iShift[2];
    float shift;
    bool bTileAxis[2];
    float origin[3];
    RVLNULL3VECTOR(origin);
    InitBoundingBox<float>(&TCPSpace, origin);
    // int debug = 0;
    int debug_nx = 0;
    int debug_ny = 0;
    int debug_nxy = 0;
    MOTION::ContactPose contactPose;
    for (iTemplatePose = 0; iTemplatePose < feasibleTCPs.n; iTemplatePose++)
    {
        pPose_G_DD = feasibleTCPs.Element + iTemplatePose;
        // if (pPose_G_DD->R[6] > -csMaxSurfaceContactAngle)
        //     continue;
        RVLTRANSF3(PRTCP_G, pPose_G_DD->R, pPose_G_DD->t, contactPose.PRTCP_DD);
        contactPose.PRTCP_DD[0] *= dd_opening_direction;
        // if (contactPose.PRTCP_DD[0] < visionTol || contactPose.PRTCP_DD[1] < visionTol)
        //     continue;
        contactPose.pose_G_DD = *pPose_G_DD;
        // if (pAllFeasibleTCPs->size() == 5236006)
        //     int debug = 0;
        pAllFeasibleTCPs->push_back(contactPose);
        UpdateBoundingBox<float>(&TCPSpace, contactPose.pose_G_DD.t);
        contactPoseTemplate = contactPose;
        for (iAxis = 0; iAxis < 2; iAxis++)
            bTileAxis[iAxis] = (contactPose.PRTCP_DD[iAxis] >= templateEnd[iAxis]);
        // debug++;
        if (bTileAxis[0])
            debug_nx++;
        if (bTileAxis[1])
            debug_ny++;
        if (bTileAxis[0] && bTileAxis[1])
            debug_nxy++;
        iShift[1] = 0;
        s = contactPoseTemplate.PRTCP_DD[1];
        while (s <= max_dd_size[1])
        {
            iShift[0] = 0;
            s = contactPoseTemplate.PRTCP_DD[0];
            while (s <= max_dd_size[0])
            {
                if (iShift[0] + iShift[1] > 0)
                {
                    // if (pAllFeasibleTCPs->size() == 5236006)
                    //     int debug = 0;
                    pAllFeasibleTCPs->push_back(contactPose);
                    UpdateBoundingBox<float>(&TCPSpace, contactPose.pose_G_DD.t);
                }
                if (!bTileAxis[0])
                    break;
                iShift[0]++;
                shift = (float)iShift[0] * dd_contact_surface_sampling_resolution;
                s = contactPoseTemplate.PRTCP_DD[0] + shift;
                contactPose.pose_G_DD.t[0] = contactPoseTemplate.pose_G_DD.t[0] + dd_opening_direction * shift;
                contactPose.PRTCP_DD[0] = s;
            }
            if (!bTileAxis[1])
                break;
            iShift[1]++;
            shift = (float)iShift[1] * dd_contact_surface_sampling_resolution;
            s = contactPoseTemplate.PRTCP_DD[1] + shift;
            contactPose.pose_G_DD.t[1] = contactPoseTemplate.pose_G_DD.t[1] + shift;
            contactPose.PRTCP_DD[1] = s;
        }
    }
}

bool DDManipulator::ApproachPath(
    Pose3D *pPose_G_S_contact,
    Array<Pose3D> &poses_G_0_via,
    float *SDF,
    Array<MOTION::IKSolution> *IKSolutions,
    Array<Pair<int, int>> &paths)
{
    // Second via point.

    float Z_G_S[3];
    RVLCOPYCOLMX3X3(pPose_G_S_contact->R, 2, Z_G_S);
    int iSphere;
    int iPlane;
    float s;
    float k;
    float e;
    float *N;
    MOTION::Plane *pPlane;
    float mins = 0.0f;
    float maxs = 0.0f;
    float c_S[3];
    MOTION::Sphere *pSphere;
    bool bFreePose;
    for (iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++)
    {
        pSphere = tool_sample_spheres.Element + iSphere;
        RVLTRANSF3(pSphere->c.Element, pPose_G_S_contact->R, pPose_G_S_contact->t, c_S);
        bFreePose = false;
        for (iPlane = 0; iPlane < 4; iPlane++)
        {
            pPlane = freeSpacePlanes_S + iPlane;
            e = RVLDOTPRODUCT3(c_S, pPlane->N) - pPlane->d - pSphere->r - visionTol;
            if (e > 0.0f)
            {
                maxs = 0.0f;
                bFreePose = true;
                break;
            }
            k = RVLDOTPRODUCT3(pPlane->N, Z_G_S);
            if (k > -1e-7)
                continue;
            s = -e / k;
            if (s > maxs || !bFreePose)
                maxs = s;
            bFreePose = true;
        }
        if (!bFreePose)
            break;
        if (maxs < mins)
            mins = maxs;
    }
    if (iSphere < tool_sample_spheres.n)
        return false;
    poses_G_0_via.n = 0;
    Pose3D *pPose_G_0 = poses_G_0_via.Element;
    float V3Tmp[3];
    RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pPose_G_S_contact->R, pPose_G_S_contact->t, pPose_G_0->R, pPose_G_0->t, V3Tmp);
    Pose3D pose_G_S;
    int iViaPt = -1;
    if (mins < -1e-3)
    {
        float Z_G_0[3];
        RVLCOPYCOLMX3X3(pPose_G_0->R, 2, Z_G_0);
        RVLSCALE3VECTOR(Z_G_0, mins, V3Tmp);
        RVLSUM3VECTORS(pPose_G_0->t, V3Tmp, pPose_G_0->t);
#ifdef RVLMOTION_DDMANIPULATOR_MULTI_SOLUTION_IK
        robot.InvKinematics(*pPose_G_0, IKSolutions[1], true);
        Free(IKSolutions[1]);
        if (IKSolutions[1].n == 0)
            return false;
#else
        if (!robot.InvKinematics(*pPose_G_0, IKSolutions[1].Element))
            return false;
#endif
        RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pPose_G_0->R, pPose_G_0->t, pose_G_S.R, pose_G_S.t);
        if (!Free(pPose_G_0, SDF))
            return false;
        poses_G_0_via.n++;
        iViaPt = 1;
    }

    // First via point.

    float Z_DD_0[3];
    RVLCOPYCOLMX3X3(pose_DD_0.R, 2, Z_DD_0);
    float C_0[3];
    RVLTRANSF3(tool_bounding_sphere.c.Element, pPose_G_0->R, pPose_G_0->t, C_0);
    RVLDIF3VECTORS(C_0, pose_DD_0.t, V3Tmp);
    e = RVLDOTPRODUCT3(Z_DD_0, V3Tmp) + dd_sx + tool_bounding_sphere.r + visionTol;
    if (e > 1e-3)
    {
        pPose_G_0 = poses_G_0_via.Element + poses_G_0_via.n;
        Pose3D *pPose_G_0_prev = poses_G_0_via.Element;
        *pPose_G_0 = *pPose_G_0_prev;
        RVLSCALE3VECTOR(Z_DD_0, e, V3Tmp);
        RVLDIF3VECTORS(pPose_G_0->t, V3Tmp, pPose_G_0->t);
#ifdef RVLMOTION_DDMANIPULATOR_MULTI_SOLUTION_IK
        robot.InvKinematics(*pPose_G_0, IKSolutions[0], true);
        Free(IKSolutions[0]);
        if (IKSolutions[0].n == 0)
            return false;
#else
        if (!robot.InvKinematics(*pPose_G_0, IKSolutions[0].Element))
            return false;
#endif
        poses_G_0_via.n++;
        iViaPt = 0;
    }

    // Check compatibility of the IK Solutions.

#ifdef RVLMOTION_DDMANIPULATOR_MULTI_SOLUTION_IK
    int iIKSolution1, iIKSolution2;
    paths.n = 0;
    float *q1, *q2;
    float dq[6];
    float qDist;
    float fTmp;
    int i;
    if (poses_G_0_via.n == 1)
    {
        if (iViaPt == 0)
        {
            memcpy(IKSolutions[1].Element, IKSolutions[0].Element, IKSolutions[0].n * sizeof(MOTION::IKSolution));
            IKSolutions[1].n = IKSolutions[0].n;
        }
        for (iIKSolution2 = 0; iIKSolution2 < IKSolutions[1].n; iIKSolution2++)
        {
            paths.Element[paths.n].a = -1;
            paths.Element[paths.n].b = iIKSolution2;
            paths.n++;
        }
    }
    else if (poses_G_0_via.n == 2)
    {
        for (iIKSolution1 = 0; iIKSolution1 < IKSolutions[0].n; iIKSolution1++)
        {
            q1 = IKSolutions[0].Element[iIKSolution1].q;
            for (iIKSolution2 = 0; iIKSolution2 < IKSolutions[1].n; iIKSolution2++)
            {
                q2 = IKSolutions[1].Element[iIKSolution2].q;
                RVLMOTION_JOINT_SPACE_CHEB_DIST(q1, q2, dq, qDist, fTmp, i);
                if (qDist <= JSDistThr)
                {
                    paths.Element[paths.n].a = iIKSolution1;
                    paths.Element[paths.n].b = iIKSolution2;
                    paths.n++;
                }
            }
        }
    }
#else
    paths.n = 1;
    paths.Element[0].a = 0;
    paths.Element[0].b = 0;
#endif

    return (paths.n > 0);
}

void DDManipulator::SetDoorModelParams(
    float sx,
    float sy,
    float sz,
    float rx,
    float ry,
    float opening_direction,
    float static_side_width,
    float moving_to_static_part_distance)
{
    dd_sx = sx;
    dd_sy = sy;
    dd_sz = sz;
    dd_rx = rx;
    dd_ry = ry;
    RVLSET3VECTOR(dd_panel_params, dd_sy, dd_sz, dd_sx);
    dd_static_side_width = static_side_width;
    dd_moving_to_static_part_distance = moving_to_static_part_distance;
    dd_opening_direction = opening_direction;
    SetDoorReferenceFrames();
    UpdateFurnitureParams();
}

void DDManipulator::UpdateFurnitureParams()
{
    // Memorize current pose_F_S.

    Pose3D pose_Tmp = pose_F_S;

    // Reset pose_F_S.

    RVLUNITMX3(pose_F_S.R);
    RVLNULL3VECTOR(pose_F_S.t);

    // Update VN model feature orientations.

    UpdateStaticOrientation();

    // Boxes.

    dd_panel_box.minx = dd_rx - 0.5f * dd_sx;
    dd_panel_box.maxx = dd_rx + 0.5f * dd_sx;
    dd_panel_box.miny = dd_ry - 0.5f * dd_sy;
    dd_panel_box.maxy = dd_ry + 0.5f * dd_sy;
    dd_panel_box.minz = -0.5f * dd_sz;
    dd_panel_box.maxz = 0.5f * dd_sz;
    dd_static_box.minx = 0;
    // dd_static_box.maxx = dd_panel_params[0] + 2.0f * (dd_moving_to_static_part_distance + dd_static_side_width);
    // Adding another moving-to-static distance so panel doesn't collide when opening
    dd_static_box.maxx = dd_panel_params[0] + 2.0f * (dd_moving_to_static_part_distance + dd_static_side_width) + dd_moving_to_static_part_distance;
    dd_static_box.miny = 0;
    dd_static_box.maxy = dd_panel_params[1] + 2.0f * (dd_moving_to_static_part_distance + dd_static_side_width);
    dd_static_box.minz = 0;
    dd_static_box.maxz = dd_static_depth;
    dd_storage_space_box.minx = dd_static_side_width;
    // dd_storage_space_box.maxx = dd_static_side_width + dd_panel_params[0] + 2.0f * dd_moving_to_static_part_distance;
    dd_storage_space_box.maxx = dd_static_side_width + dd_panel_params[0] + 2.0f * dd_moving_to_static_part_distance + dd_moving_to_static_part_distance;
    dd_storage_space_box.miny = dd_static_side_width;
    dd_storage_space_box.maxy = dd_static_side_width + dd_panel_params[1] + 2.0f * dd_moving_to_static_part_distance;
    dd_storage_space_box.minz = 0;
    dd_storage_space_box.maxz = dd_static_depth;

    /// VN model.

    Array<Vector3<float>> vertices;
    vertices.n = 24;
    vertices.Element = new Vector3<float>[vertices.n];
    float *vertices_ = new float[3 * vertices.n];
    BoxVertices<float>(&dd_panel_box, vertices_);
    BoxVertices<float>(&dd_static_box, vertices_ + 3 * 8);
    BoxVertices<float>(&dd_storage_space_box, vertices_ + 2 * 3 * 8);
    Array<RECOG::VN_::Correspondence5> assoc;
    assoc.Element = new RECOG::VN_::Correspondence5[29];

    // Static part.

    RECOG::VN_::Correspondence5 *pAssoc = assoc.Element;
    int iPt;
    if (bVNPanel)
    {
        for (iPt = 0; iPt < 8; iPt++, pAssoc++)
        {
            pAssoc->iSPoint = iPt;
            pAssoc->iMCluster = 3;
            pAssoc->iBeta = -1;
        }
    }
    for (iPt = 8; iPt < 16; iPt++, pAssoc++)
    {
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 0;
        pAssoc->iBeta = -1;
    }
    for (iPt = 16; iPt < 20; iPt++)
    {
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 1;
        pAssoc->iBeta = 1;
        pAssoc++;
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 1;
        pAssoc->iBeta = 2;
        pAssoc++;
    }
    for (iPt = 20; iPt < 24; iPt++, pAssoc++)
    {
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 1;
        pAssoc->iBeta = 0;
    }
    pAssoc->iSPoint = 10;
    pAssoc->iMCluster = 2;
    pAssoc->iBeta = 0;
    pAssoc++;
    assoc.n = pAssoc - assoc.Element;
    float *PSrc = vertices_;
    float *PTgt;
    for (iPt = 0; iPt < 24; iPt++, PSrc += 3)
    {
        PTgt = vertices.Element[iPt].Element;
        RVLCOPY3VECTOR(PSrc, PTgt);
    }
    RVL_DELETE_ARRAY(dVNEnv);
    dVNEnv = new float[pVNEnv->featureArray.n];
    pVNEnv->Descriptor(vertices, assoc, dVNEnv);
    pVNEnv->SetFeatureOffsets(dVNEnv);

    // Panel.

    pAssoc = assoc.Element;
    for (iPt = 0; iPt < 8; iPt++, pAssoc++)
    {
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 0;
        pAssoc->iBeta = -1;
    }
    assoc.n = pAssoc - assoc.Element;
    Array<Vector3<float>> panelVertices;
    panelVertices.n = 8;
    panelVertices.Element = vertices.Element;
    RVL_DELETE_ARRAY(dVNPanel);
    dVNPanel = new float[pVNPanel->featureArray.n];
    pVNPanel->Descriptor(panelVertices, assoc, dVNPanel);
    pVNPanel->SetFeatureOffsets(dVNPanel);

    //

    delete[] vertices_;
    delete[] vertices.Element;
    delete[] assoc.Element;

    // if (bVNPanel)
    //{
    //     float fTmp = dd_moving_to_static_part_distance + dd_static_side_width;
    //     float t[3];
    //     RVLNULL3VECTOR(t);
    //     RVLSET3VECTOR(t, fTmp, fTmp, dd_panel_params[2]);
    //     RECOG::VN_::Feature* pFeature;
    //     int iFeature;
    //     RECOG::VN_::ModelCluster* pVNClusterPanel = VNMClusters.Element[2];
    //     for (iFeature = pVNClusterPanel->iFeatureInterval.a; iFeature <= pVNClusterPanel->iFeatureInterval.b; iFeature++)
    //     {
    //         pFeature = pVNEnv->featureArray.Element + iFeature;
    //         dVNEnv[iFeature] += RVLDOTPRODUCT3(pFeature->N, t);
    //     }
    //     RVLCOPYMX3X3(pose_A_F.R, pVNClusterPanel->R);
    //     RVLCOPY3VECTOR(pose_A_F.t, pVNClusterPanel->t);
    //     pVNEnv->Descriptor(dVNEnv);
    // }

    // Set pose_F_S to its original values.

    pose_F_S = pose_Tmp;

    // Set VN model features to their original orientations.

    UpdateStaticOrientation();
}

// This function computes pose_A_F and pose_DD_A from the furniture parameters.

void DDManipulator::SetDoorReferenceFrames()
{
    RVLNULLMX3X3(pose_A_F.R);
    if (dd_opening_direction > 0.0f)
    {
        RVLMXEL(pose_A_F.R, 3, 0, 1) = 1.0f;
        RVLMXEL(pose_A_F.R, 3, 1, 2) = -1.0f;
        RVLMXEL(pose_A_F.R, 3, 2, 0) = -1.0f;
        // RVLSET3VECTOR(pose_A_F.t, dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[0] - dd_ry,
        RVLSET3VECTOR(pose_A_F.t, dd_static_side_width + 2 * dd_moving_to_static_part_distance + 0.5f * dd_panel_params[0] - dd_ry,
                      dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[1],
                      0.5f * dd_panel_params[2]);
        RVLSET3VECTOR(pose_DD_A.t, dd_rx - 0.5f * dd_sx, dd_ry - 0.5f * dd_sy, 0.5f * dd_sz);
    }
    else
    {
        RVLMXEL(pose_A_F.R, 3, 0, 1) = -1.0f;
        RVLMXEL(pose_A_F.R, 3, 1, 2) = -1.0f;
        RVLMXEL(pose_A_F.R, 3, 2, 0) = 1.0f;
        // RVLSET3VECTOR(pose_A_F.t, dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[0] + dd_ry,
        RVLSET3VECTOR(pose_A_F.t, dd_static_side_width + 2 * dd_moving_to_static_part_distance + 0.5f * dd_panel_params[0] + dd_ry,
                      dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[1],
                      0.5f * dd_panel_params[2]);
        RVLSET3VECTOR(pose_DD_A.t, dd_rx + 0.5f * dd_sx, dd_ry - 0.5f * dd_sy, 0.5f * dd_sz);
    }
    RVLCOPYMX3X3T(pose_A_F.R, pose_DD_A.R);
}

void DDManipulator::UpdateVNClusterOrientations()
{
    RECOG::VN_::ModelCluster *pCluster;
    for (int iCluster = 0; iCluster < 3; iCluster++)
    {
        pCluster = VNMClusters.Element[iCluster];
        RVLCOPYMX3X3(pose_F_S.R, pCluster->R);
        RVLCOPY3VECTOR(pose_F_S.t, pCluster->t);
    }
    RVLCOPYMX3X3(pose_F_S.R, pPanelVNMCluster->R);
    RVLCOPY3VECTOR(pose_F_S.t, pPanelVNMCluster->t);
}

void DDManipulator::SetDoorPose(Pose3D pose_A_S)
{
    Pose3D pose_F_A;
    RVLINVTRANSF3D(pose_A_F.R, pose_A_F.t, pose_F_A.R, pose_F_A.t);
    RVLCOMPTRANSF3D(pose_A_S.R, pose_A_S.t, pose_F_A.R, pose_F_A.t, pose_F_S.R, pose_F_S.t);
    // printf("manipulator.pose_F_S:\n");
    // for(int i = 0; i < 3; i++)
    // {
    // 	for(int j = 0; j < 3; j++)
    // 		printf("%f ", pose_F_S.R[j+3*i]);
    // 	printf("%f\n", pose_F_S.t[i]);
    // }
    UpdateFurniturePose();
}

void DDManipulator::UpdateFurniturePose()
{
    UpdateVNClusterOrientations();
    pVNEnv->Descriptor(dVNEnv);
    pVNPanel->Descriptor(dVNPanel);
}

void DDManipulator::UpdateStaticOrientation()
{
    UpdateVNClusterOrientations();
    pVNEnv->UpdateClusterOrientations();
    pVNPanel->UpdateClusterOrientations();
}

void DDManipulator::AdaptContactPoseGraph()
{
    RVL_DELETE_ARRAY(selectedNodes.Element);
    selectedNodes.Element = new int[nodes.n];
    selectedNodes.n = 0;
    RVL_DELETE_ARRAY(bSelected)
    bSelected = new bool[nodes.n];
    RVL_DELETE_ARRAY(contactNode);
    contactNode = new int[nodes.n];
    memset(contactNode, 0xff, nodes.n * sizeof(int));
    int iNode;
    MOTION::Node *pNode = nodes.Element;
    float maxx = 0.5f * dd_panel_params[0];
    for (iNode = 0; iNode < nodes.n; iNode++, pNode++)
    {
        if (pNode->PRTCP[0] <= maxx && pNode->PRTCP[1] <= dd_panel_params[1])
        {
            selectedNodes.Element[selectedNodes.n] = iNode;
            bSelected[iNode] = true;
            contactNode[iNode] = selectedNodes.n;
            selectedNodes.n++;
        }
        else
            bSelected[iNode] = false;
    }
}

void DDManipulator::FreeSpacePlanes()
{
    Plane freeSpacePlanes_DD[4];
    if (dd_opening_direction > 0.0f)
    {
        RVLSET3VECTOR(freeSpacePlanes_DD[0].N, -1.0f, 0.0f, 0.0f);
        RVLSET3VECTOR(freeSpacePlanes_DD[2].N, -COS45, -COS45, 0.0f);
    }
    else
    {
        RVLSET3VECTOR(freeSpacePlanes_DD[0].N, 1.0f, 0.0f, 0.0f);
        RVLSET3VECTOR(freeSpacePlanes_DD[2].N, COS45, -COS45, 0.0f);
    }
    RVLSET3VECTOR(freeSpacePlanes_DD[1].N, 0.0f, -1.0f, 0.0f);
    RVLSET3VECTOR(freeSpacePlanes_DD[3].N, 0.0f, 0.0f, -1.0f);
    float *N_S;
    float P_DD[3];
    RVLSET3VECTOR(P_DD, 0.0f, 0.0f, -dd_sx);
    float P_S[3];
    RVLTRANSF3(P_DD, pose_DD_S.R, pose_DD_S.t, P_S);
    for (int i = 0; i < 4; i++)
    {
        N_S = freeSpacePlanes_S[i].N;
        RVLMULMX3X3VECT(pose_DD_S.R, freeSpacePlanes_DD[i].N, N_S);
        freeSpacePlanes_S[i].d = RVLDOTPRODUCT3(N_S, P_S);
    }
}

void DDManipulator::Neighbors(
    int iNodeC,
    NodeSpaceElement *nodeSpace,
    Array<MOTION::NodeJS *> &neighbors)
{
    neighbors.n = 0;
    MOTION::NodeSpaceElement *pNodeSpaceElement_ = nodeSpace + iNodeC;
    MOTION::NodeJS *pNodeJS_ = pNodeSpaceElement_->nodesJS.pFirst;
    while (pNodeJS_)
    {
        if (pNodeJS_->bFeasible)
            neighbors.Element[neighbors.n++] = pNodeJS_;
        pNodeJS_ = pNodeJS_->pNext;
    }
    int iNodeG = selectedNodes.Element[iNodeC];
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>> *pGNode = graph.NodeArray.Element + iNodeG;
    GRAPH::EdgePtr<Edge> *pEdgePtr = pGNode->EdgeList.pFirst;
    MOTION::Edge *pEdge;
    int iNodeG_, iNodeC_;
    while (pEdgePtr)
    {
        RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNodeG, pEdgePtr, pEdge, iNodeG_);
        iNodeC_ = contactNode[iNodeG_];
        if (iNodeC_ >= 0)
        {
            // if (iNodeC_ == 32247)
            //     int debug = 0;
            pNodeSpaceElement_ = nodeSpace + iNodeC_;
            pNodeJS_ = pNodeSpaceElement_->nodesJS.pFirst;
            while (pNodeJS_)
            {
                if (pNodeJS_->bFeasible)
                    neighbors.Element[neighbors.n++] = pNodeJS_;
                pNodeJS_ = pNodeJS_->pNext;
            }
        }
        pEdgePtr = pEdgePtr->pNext;
    }
}

bool DDManipulator::LoadFeasibleToolContactPoses(std::string contactPointsFileName)
{
    FILE *fp = fopen(contactPointsFileName.c_str(), "rb");
    if (fp == NULL)
        return false;
    fclose(fp);
    cnpy::NpyArray npyData = cnpy::npy_load(contactPointsFileName);
    double *data = npyData.data<double>();
    feasibleTCPs.n = npyData.num_vals / 16;
    RVL_DELETE_ARRAY(feasibleTCPs.Element);
    feasibleTCPs.Element = new Pose3D[feasibleTCPs.n];
    int iPose;
    double *pData = data;
    Pose3D *pPose = feasibleTCPs.Element;
    double *srcRow;
    float *tgtRow;
    int i;
    for (iPose = 0; iPose < feasibleTCPs.n; iPose++, pPose++, pData += 16)
    {
        srcRow = pData;
        tgtRow = pPose->R;
        for (i = 0; i < 3; i++, srcRow += 4, tgtRow += 3)
        {
            RVLCOPY3VECTOR(srcRow, tgtRow);
            pPose->t[i] = srcRow[3];
        }
    }
    return true;
}

bool DDManipulator::LoadContactPoseGraph(std::string contactPoseGraphFileName)
{
    FILE *fp = fopen(contactPoseGraphFileName.c_str(), "rb");
    if (fp == NULL)
        return false;
    fread(&nodes.n, sizeof(int), 1, fp);
    RVL_DELETE_ARRAY(nodes.Element);
    nodes.Element = new MOTION::Node[nodes.n];
    fread(nodes.Element, sizeof(MOTION::Node), nodes.n, fp);
    MOTION::Node *pNode = nodes.Element;
    RVL_DELETE_ARRAY(graph.NodeMem);
    graph.NodeMem = new GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>[nodes.n];
    graph.NodeArray.Element = graph.NodeMem;
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>> *pGNode = graph.NodeMem;
    QList<GRAPH::EdgePtr<MOTION::Edge>> *pEdgeList;
    for (int iNode = 0; iNode < nodes.n; iNode++, pNode++, pGNode++)
    {
        pEdgeList = &(pGNode->EdgeList);
        RVLQLIST_INIT(pEdgeList);
        pNode->pGNode = pGNode;
        pGNode->idx = iNode;
    }
    Array<Pair<int, int>> edges;
    fread(&edges.n, sizeof(int), 1, fp);
    edges.Element = new Pair<int, int>[edges.n];
    fread(edges.Element, sizeof(Pair<int, int>), edges.n, fp);
    fclose(fp);
    int *nNeighbors = new int[nodes.n];
    memset(nNeighbors, 0, nodes.n * sizeof(int));
    Pair<int, int> edge;
    MOTION::Edge *pEdge;
    MOTION::Node *pNode_;
    for (int iEdge = 0; iEdge < edges.n; iEdge++)
    {
        edge = edges.Element[iEdge];
        pNode = nodes.Element + edge.a;
        pNode_ = nodes.Element + edge.b;
        pEdge = ConnectNodes<GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, MOTION::Edge, GRAPH::EdgePtr<MOTION::Edge>>(pNode->pGNode, pNode_->pGNode, edge.a, edge.b, pMem);
        nNeighbors[edge.a]++;
        nNeighbors[edge.b]++;
    }
    maxnContactPoseGraphNeighbors = 0;
    for (int iNode = 0; iNode < nodes.n; iNode++)
        if (nNeighbors[iNode] > maxnContactPoseGraphNeighbors)
            maxnContactPoseGraphNeighbors = nNeighbors[iNode];
    maxnContactPoseGraphNeighbors++;
    delete[] nNeighbors;
    return true;
}

void DDManipulator::LoadToolModel(std::string toolModelDir)
{
    std::string toolMeshFileName = toolModelDir + RVLFILEPATH_SEPARATOR_ + "mesh.ply";
    if (pToolMesh)
        delete pToolMesh;
    pToolMesh = new Mesh;
    pToolMesh->LoadPolyDataFromPLY((char *)(toolMeshFileName.data()));
    std::string toolSpheresFileName = toolModelDir + RVLFILEPATH_SEPARATOR_ + "spheres.npy";
    cnpy::NpyArray npyData = cnpy::npy_load(toolSpheresFileName);
    double *data = npyData.data<double>();
    tool_sample_spheres.n = npyData.num_vals / 4;
    RVL_DELETE_ARRAY(tool_sample_spheres.Element);
    tool_sample_spheres.Element = new MOTION::Sphere[tool_sample_spheres.n];
    double *pData = data;
    float *c;
    for (int iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++, pData += 4)
    {
        c = tool_sample_spheres.Element[iSphere].c.Element;
        RVLSCALE3VECTOR(pData, 0.001f, c);
        tool_sample_spheres.Element[iSphere].r = 0.001f * pData[3];
    }
    bDefaultToolModel = false;
}

void DDManipulator::LoadExample(std::string example)
{
    std::stringstream ss(example);
    std::string value;
    std::getline(ss, value, ',');
    float sy = std::stof(value);
    std::getline(ss, value, ',');
    float sz = std::stof(value);
    SetDoorModelParams(0.018f, sy, sz, 0.0f, -0.5f * sy, -1.0f, 0.018f, 0.005f);
    // Only for visualization purpose.
    // pVNEnv->Display(pVisualizationData->pVisualizer, 0.02f, dVNEnv);
    // pVisualizationData->pVisualizer->Run();
    Box<float> bbox;
    pVNEnv->BoundingBox(dVNEnv, bbox);
    float a, b, c;
    BoxSize<float>(&bbox, a, b, c);
    //
    Pose3D pose_A_S;
    for (int i = 0; i < 3; i++)
    {
        std::getline(ss, value, ',');
        pose_A_S.t[i] = std::stof(value);
    }
    std::getline(ss, value, ',');
    float rotz_A_S_deg = std::stof(value);
    float rotz_A_S = DEG2RAD * rotz_A_S_deg;
    float cs = cos(rotz_A_S);
    float sn = sin(rotz_A_S);
    RVLROTZ(cs, sn, pose_A_S.R);
    SetDoorPose(pose_A_S);
    std::getline(ss, value, ',');
    float dd_state_angle_deg = std::stof(value);
    SetEnvironmentState(dd_state_angle_deg);
    // Only for visualization purpose.
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
        RVLTRANSF3(P_F_, pose_F_S.R, pose_F_S.t, P_S);
        if (i == 0)
            InitBoundingBox<float>(&bbox, P_S);
        else
            UpdateBoundingBox<float>(&bbox, P_S);
    }
    float resolution = 0.02f * BoxSize(&bbox);
    ExpandBox<float>(&bbox, 10.0f * resolution);
    pVisualizationData->VNBBox = bbox;
    // pVNEnv->Display(pVisualizationData->pVisualizer, 0.01f, dVNEnv, NULL, 0.0f, &(pVisualizationData->VNBBox));
    // pVNPanel->Display(pVisualizationData->pVisualizer, 0.01f, dVNPanel, NULL, 0.0f, &(pVisualizationData->VNBBox));
    // pVisualizationData->pVisualizer->Run();
    // pVisualizationData->pVisualizer->Clear();
    //
}

void DDManipulator::LoadExampleIndexed(std::string example)
{
    std::stringstream ss(example);
    std::string value;

    std::getline(ss, value, ',');
    int idx = std::stoi(value);

    cout << "Cabinet idx: " << idx << endl;

    // Skip the next 4 values since they are just resulting vals
    for (int i = 0; i < 4; i++)
    {
        std::getline(ss, value, ',');
    }

    std::getline(ss, value, ',');
    float sy = std::stof(value);
    std::getline(ss, value, ',');
    float sz = std::stof(value);
    SetDoorModelParams(0.018f, sy, sz, 0.0f, -0.5f * sy, -1.0f, 0.018f, 0.005f);
    // Only for visualization purpose.
    // pVNEnv->Display(pVisualizationData->pVisualizer, 0.02f, dVNEnv);
    // pVisualizationData->pVisualizer->Run();
    Box<float> bbox;
    pVNEnv->BoundingBox(dVNEnv, bbox);
    float a, b, c;
    BoxSize<float>(&bbox, a, b, c);
    //
    Pose3D pose_A_S;
    for (int i = 0; i < 3; i++)
    {
        std::getline(ss, value, ',');
        pose_A_S.t[i] = std::stof(value);
    }
    std::getline(ss, value, ',');
    float rotz_A_S_deg = std::stof(value);
    float rotz_A_S = DEG2RAD * rotz_A_S_deg;
    float cs = cos(rotz_A_S);
    float sn = sin(rotz_A_S);
    RVLROTZ(cs, sn, pose_A_S.R);
    SetDoorPose(pose_A_S);

    if (use_fcl)
    {
        std::string cabinetStaticPath = std::string(cabinetStaticDirPath) + "/cabinet_static_" + std::to_string(idx) + ".ply";
        LoadCabinetStaticFCL(cabinetStaticPath, pose_A_S);
        
        std::string cabinetPanelPath = std::string(cabinetStaticDirPath) + "/cabinet_panel_" + std::to_string(idx) + ".ply";
        LoadCabinetPanelFCL(cabinetPanelPath);

        CreateRobotCylindersFCL();
        CreateGndFCL();
    }

    std::getline(ss, value, ',');
    float dd_state_angle_deg = std::stof(value);
    SetEnvironmentState(dd_state_angle_deg);
    // Only for visualization purpose.
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
        RVLTRANSF3(P_F_, pose_F_S.R, pose_F_S.t, P_S);
        if (i == 0)
            InitBoundingBox<float>(&bbox, P_S);
        else
            UpdateBoundingBox<float>(&bbox, P_S);
    }
    float resolution = 0.02f * BoxSize(&bbox);
    ExpandBox<float>(&bbox, 10.0f * resolution);
    pVisualizationData->VNBBox = bbox;
    // pVNEnv->Display(pVisualizationData->pVisualizer, 0.01f, dVNEnv, NULL, 0.0f, &(pVisualizationData->VNBBox));
    // pVNPanel->Display(pVisualizationData->pVisualizer, 0.01f, dVNPanel, NULL, 0.0f, &(pVisualizationData->VNBBox));
    // pVisualizationData->pVisualizer->Run();
    // // pVisualizationData->pVisualizer->Clear();
    // pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
    //

}

void DDManipulator::InitVisualizer(Visualizer *pVisualizerIn)
{
    MOTION::InitVisualizer(pVisualizerIn, pVisualizationData, pMem0);
    pVisualizationData->bVNEnv = false;
    pVisualizationData->paramList.m_pMem = pMem;
    RVLPARAM_DATA *pParamData;
    pVisualizationData->paramList.Init();
    pParamData = pVisualizationData->paramList.AddParam("DDM.visualize", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualize));
    pVisualizationData->paramList.LoadParams((char *)(cfgFileName.data()));
}

#ifdef RVLVTK
void DDManipulator::Visualize(
    std::vector<MOTION::Node> *pNodes,
    std::vector<int> *pPath,
    Array<float> doorStates,
    bool bVisualizeToolBoundingSphere,
    bool bVisualizeStates,
    bool bVisualizeMotionPlanningTree,
    int iGoal,
    Array2D<float> *pRobotJoints)
{
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    uchar red[] = {255, 0, 0};
    uchar blue[] = {0, 0, 255};
    float *PSrc, *PTgt;

    // Display environment VN model.

    if (pVisualizationData->bVNEnv)
        pVNEnv->Display(pVisualizer, 0.01f, dVNEnv, NULL, 0.0f, &(pVisualizationData->VNBBox));

    // Display static part of the furniture.

    Vector3<float> boxSize;
    Vector3<float> boxCenter;
    Pose3D pose_box_S;
    BoxSize<float>(&dd_static_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&dd_static_box, boxCenter.Element);
    RVLCOPYMX3X3(pose_F_S.R, pose_box_S.R);
    RVLTRANSF3(boxCenter.Element, pose_F_S.R, pose_F_S.t, pose_box_S.t);
    vtkSmartPointer<vtkActor> staticBoxActor = pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);
    BoxSize<float>(&dd_storage_space_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&dd_storage_space_box, boxCenter.Element);
    vtkSmartPointer<vtkActor> staticSorageSpaceActor = pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);

    /// Display door panel and robot.

    vtkSmartPointer<vtkActor> doorPanelActor;
    vtkSmartPointer<vtkActor> doorPanelVNActor;
    vtkSmartPointer<vtkActor> cabinetStaticMeshActor, pCabinetWholeMeshActor, cabinetPanelMeshActor;
    if (bVisualizeStates)
    {
        // Visualize robot and door panel motion.

        int iState;
        int iToolActor;

        Pose3D pose_A_S;
        RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_A_F.R, pose_A_F.t, pose_A_S.R, pose_A_S.t);

        for (iState = 0; iState < doorStates.n; iState++)
        {
            SetEnvironmentState(doorStates.Element[iState]);
            doorPanelActor = VisualizeDoorPenel();
            if (pVisualizationData->bVNEnv)
                doorPanelVNActor = pVNPanel->Display(pVisualizer, 0.01f, dVNPanel, NULL, 0.0f, &(pVisualizationData->VNBBox));
            MOTION::Node *pNode = pNodes->data() + pPath->at(iState);
            float *q = pRobotJoints->Element + pRobotJoints->w * iState;

            // Only for debugging purpose!!!

            // Array<MOTION::IKSolution> IKSolutions;
            // IKSolutions.Element = new MOTION::IKSolution[8];
            // robot.InvKinematics(pNode->pose.pose, IKSolutions, true);
            // delete[] IKSolutions.Element;

            // memcpy(robot.q, q, 6 * sizeof(float));
            // Pose3D pose_2_1, pose_3_2, pose_3_1;
            // robot.FwdKinematics(1, &pose_2_1);
            // robot.FwdKinematics(2, &pose_3_2);
            // RVLCOMPTRANSF3D(pose_2_1.R, pose_2_1.t, pose_3_2.R, pose_3_2.t, pose_3_1.R, pose_3_1.t);
            // if (robot.SelfCollision(q, pose_3_1.t))
            //     int debug = 0;

            //

            if (use_fcl)
            {
                VisualizeCabinetStaticMesh(pose_A_S, cabinetStaticMeshActor);
                VisualizeDoorPanelMesh(cabinetPanelMeshActor);
            }

            // VisualizeCabinetWholeMesh(pose_A_S, pCabinetWholeMeshActor);
            if (bVisualizeToolBoundingSphere)
            {
                Array<int> visSpheres;
                int visSpheresMem = -1;
                visSpheres.Element = &visSpheresMem;
                visSpheres.n = 1;
                VisualizeTool(pNode->pose.pose, &(pVisualizationData->robotActors), true, &visSpheres);
            }
            else
                VisualizeTool(pNode->pose.pose, &(pVisualizationData->robotActors));
            VisualizeRobot(q, &(pVisualizationData->robotActors));
            // printf("distance to the door panel edge: x=%f, y=%f\n", pNode->PRTCP[0], pNode->PRTCP[1]);     // Only for debugging purpose.
            printf("point %d\n", iState);
            if (!Free(q))
                printf("Collision!\n");
            pVisualizer->Run();
            pVisualizer->renderer->RemoveViewProp(doorPanelActor);
            pVisualizer->renderer->RemoveViewProp(cabinetStaticMeshActor);
            pVisualizer->renderer->RemoveViewProp(cabinetPanelMeshActor);
            if (pVisualizationData->bVNEnv)
                pVisualizer->renderer->RemoveViewProp(doorPanelVNActor);
            for (iToolActor = 0; iToolActor < pVisualizationData->robotActors.size(); iToolActor++)
                pVisualizer->renderer->RemoveViewProp(pVisualizationData->robotActors[iToolActor]);
        }
    }
    else
    {
        // Visualize door panel.

        VisualizeDoorPenel();

        // Visualize path.

        for (int iNode = 0; iNode < pPath->size(); iNode++)
            VisualizeTool(pNodes->at(pPath->at(iNode)).pose.pose, &(pVisualizationData->robotActors));
    }

    ///

    // Visualize motion planning tree.

    if (bVisualizeMotionPlanningTree)
    {
        Array<Point> visNodes;
        visNodes.Element = new Point[pNodes->size()];
        visNodes.n = pNodes->size();
        Array<Pair<int, int>> visEdges;
        visEdges.n = 0;
        visEdges.Element = new Pair<int, int>[visNodes.n - 1];
        int iNode, iParent;
        for (iNode = 0; iNode < pNodes->size(); iNode++)
        {
            PSrc = pNodes->at(iNode).pose.pose.t;
            PTgt = visNodes.Element[iNode].P;
            RVLCOPY3VECTOR(PSrc, PTgt);
            iParent = pNodes->at(iNode).iParent;
            if (iParent >= 0)
            {
                visEdges.Element[visEdges.n].a = iNode;
                visEdges.Element[visEdges.n].b = pNodes->at(iNode).iParent;
                visEdges.n++;
            }
        }
        pVisualizer->DisplayPointSet<float, Point>(visNodes, blue, 3);
        pVisualizer->DisplayLines(visNodes, visEdges, blue);
        delete[] visNodes.Element;
        delete[] visEdges.Element;
    }

    // Visualize Goal.

    if (iGoal >= 0)
    {
        Array<Point> visGoal;
        Point visGoalMem;
        visGoal.Element = &visGoalMem;
        visGoal.n = 1;
        PSrc = pNodes->at(iGoal).pose.pose.t;
        PTgt = visGoal.Element[0].P;
        RVLCOPY3VECTOR(PSrc, PTgt);
        pVisualizer->DisplayPointSet<float, Point>(visGoal, red, 6);
    }

    // Clear static part of the furniture.

    pVisualizer->renderer->RemoveViewProp(staticBoxActor);
    pVisualizer->renderer->RemoveViewProp(staticSorageSpaceActor);
}

void DDManipulator::VisualizeTool(
    Pose3D pose_G_R,
    std::vector<vtkSmartPointer<vtkActor>> *pActors,
    bool bToolMesh,
    Array<int> *pSpheres)
{
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    Pose3D pose_G_S;
    RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pose_G_R.R, pose_G_R.t, pose_G_S.R, pose_G_S.t);
    if (bDefaultToolModel)
    {
        Pose3D pose_F1_G;
        RVLUNITMX3(pose_F1_G.R);
        float xF = 0.5f * (tool_finger_distance + tool_finger_size.Element[0]);
        float zF = -0.5 * tool_finger_size.Element[2];
        RVLSET3VECTOR(pose_F1_G.t, -xF, 0.0f, zF);
        Pose3D pose_F1_S;
        RVLCOMPTRANSF3D(pose_G_S.R, pose_G_S.t, pose_F1_G.R, pose_F1_G.t, pose_F1_S.R, pose_F1_S.t);
        pActors->push_back(pVisualizer->DisplayBox(tool_finger_size.Element[0], tool_finger_size.Element[1], tool_finger_size.Element[2],
                                                   &pose_F1_S, 255.0, 0.0, 0.0));
        Pose3D pose_F2_G;
        RVLUNITMX3(pose_F2_G.R);
        RVLSET3VECTOR(pose_F2_G.t, xF, 0.0f, zF);
        Pose3D pose_F2_S;
        RVLCOMPTRANSF3D(pose_G_S.R, pose_G_S.t, pose_F2_G.R, pose_F2_G.t, pose_F2_S.R, pose_F2_S.t);
        pActors->push_back(pVisualizer->DisplayBox(tool_finger_size.Element[0], tool_finger_size.Element[1], tool_finger_size.Element[2],
                                                   &pose_F2_S, 255.0, 0.0, 0.0));
        Pose3D pose_P_G;
        RVLUNITMX3(pose_P_G.R);
        RVLSET3VECTOR(pose_P_G.t, 0.0f, 0.0f, -(tool_finger_size.Element[2] + 0.5 * tool_palm_size.Element[2]));
        Pose3D pose_P_S;
        RVLCOMPTRANSF3D(pose_G_S.R, pose_G_S.t, pose_P_G.R, pose_P_G.t, pose_P_S.R, pose_P_S.t);
        pActors->push_back(pVisualizer->DisplayBox(tool_palm_size.Element[0], tool_palm_size.Element[1], tool_palm_size.Element[2],
                                                   &pose_P_S, 255.0, 0.0, 0.0));
        Pose3D pose_Wrist_S;
        RVLCOPYMX3X3(pose_G_S.R, pose_Wrist_S.R);
        float Z_G_S[3];
        RVLCOPYCOLMX3X3(pose_G_S.R, 2, Z_G_S);
        float V3Tmp[3];
        float tTmp = tool_finger_size.Element[2] + tool_palm_size.Element[2] + 0.5f * tool_wrist_len;
        RVLSCALE3VECTOR(Z_G_S, tTmp, V3Tmp);
        RVLDIF3VECTORS(pose_G_S.t, V3Tmp, pose_Wrist_S.t);
        vtkSmartPointer<vtkActor> wristActor = pVisualizer->DisplayCylinder(tool_wrist_r, tool_wrist_len, &pose_Wrist_S, 12, 255.0, 0.0, 0.0);
        wristActor->GetProperty()->SetRepresentationToWireframe();
        pActors->push_back(wristActor);
    }
    Array<Point> toolSampleSphereCentersPC;
    toolSampleSphereCentersPC.n = tool_sample_spheres.n;
    // toolSampleSphereCentersPC.n++;
    toolSampleSphereCentersPC.Element = new Point[toolSampleSphereCentersPC.n];
    float *PSrc, *PTgt;
    for (int iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++)
    {
        PSrc = tool_sample_spheres.Element[iSphere].c.Element;
        PTgt = toolSampleSphereCentersPC.Element[iSphere].P;
        RVLTRANSF3(PSrc, pose_G_S.R, pose_G_S.t, PTgt);
    }
    // PTgt = toolSampleSphereCentersPC.Element[tool_sample_spheres.n].P;
    // Pose3D* pPose_G_S = &pose_G_S;
    // RVLTRANSF3(PRTCP_G, pPose_G_S->R, pPose_G_S->t, PTgt);
    uchar red[] = {255, 0, 0};
    pActors->push_back(pVisualizer->DisplayPointSet<float, Point>(toolSampleSphereCentersPC, red, 6));
    delete[] toolSampleSphereCentersPC.Element;
    if (pSpheres)
    {
        int iSphere;
        MOTION::Sphere *pSphere;
        float *cG;
        float cS[3];
        for (int iSphere_ = 0; iSphere_ < pSpheres->n; iSphere_++)
        {
            iSphere = pSpheres->Element[iSphere_];
            pSphere = (iSphere >= 0 ? tool_sample_spheres.Element + iSphere : &tool_bounding_sphere);
            vtkNew<vtkSphereSource> sphereSource;
            cG = pSphere->c.Element;
            RVLTRANSF3(cG, pose_G_S.R, pose_G_S.t, cS);
            sphereSource->SetCenter(cS[0], cS[1], cS[2]);
            sphereSource->SetRadius(pSphere->r);
            sphereSource->SetPhiResolution(16);
            sphereSource->SetThetaResolution(9);
            vtkNew<vtkPolyDataMapper> mapper;
            mapper->SetInputConnection(sphereSource->GetOutputPort());
            vtkNew<vtkActor> actor;
            actor->SetMapper(mapper.GetPointer());
            actor->GetProperty()->SetColor(0.5, 0.5, 0.5);
            actor->GetProperty()->SetRepresentationToWireframe();
            pVisualizer->renderer->AddActor(actor.GetPointer());
            pActors->push_back(actor.GetPointer());
        }
    }
    if (bToolMesh && pToolMesh != NULL)
    {
        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        double T[16];
        RVLHTRANSFMX(pose_G_S.R, pose_G_S.t, T);
        transform->SetMatrix(T);
        vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        transformFilter->SetInputData(pToolMesh->pPolygonData);
        transformFilter->SetTransform(transform);
        transformFilter->Update();
        pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
        pVisualizer->map->SetInputConnection(transformFilter->GetOutputPort());
        pVisualizer->map->InterpolateScalarsBeforeMappingOff();
        pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
        pVisualizer->actor->SetMapper(pVisualizer->map);
        pVisualizer->renderer->AddActor(pVisualizer->actor);
        pActors->push_back(pVisualizer->actor);
    }
}

void DDManipulator::VisualizeRobot(
    float *q,
    std::vector<vtkSmartPointer<vtkActor>> *pActors)
{
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    memcpy(robot.q, q, robot.n * sizeof(float));
    robot.FwdKinematics();

    // Visualize skeleton.

    Array<Point> vertices;
    vertices.n = robot.n + 1;
    vertices.Element = new Point[vertices.n];
    RVLCOPY3VECTOR(robot.pose_0_W.t, vertices.Element[0].P);
    Array<Pair<int, int>> lines;
    lines.n = robot.n;
    lines.Element = new Pair<int, int>[lines.n];
    int i;
    float *PSrc, *PTgt;
    for (i = 0; i < robot.n; i++)
    {
        PSrc = robot.link_pose[i].t;
        PTgt = vertices.Element[i + 1].P;
        RVLTRANSF3(PSrc, robot.pose_0_W.R, robot.pose_0_W.t, PTgt);
        lines.Element[i].a = i;
        lines.Element[i].b = i + 1;
    }
    uchar red[] = {255, 0, 0};
    pActors->push_back(pVisualizer->DisplayLines(vertices, lines, red, 2.0f));

    delete[] vertices.Element;
    delete[] lines.Element;

    // Visualize link solids for collision detection.

    MOTION::Cylinder *pCylinder;
    Pose3D pose_C_L;
    Pose3D pose_C_0;
    Pose3D pose_C_W;
    float R_L_C[9];
    float *X_C_L = R_L_C;
    float *Y_C_L = R_L_C + 3;
    float *Z_C_L = R_L_C + 6;
    float h;
    float fTmp;
    int i_, j_, k_;
    int iLink;
    for (iLink = 0; iLink <= robot.maxCollisionLinkIdx; iLink++)
    {
        for (int j = 0; j < robot.collisionCylinders.Element[iLink].n; j++)
        {
            pCylinder = robot.collisionCylinders.Element[iLink].Element + j;
            RVLDIF3VECTORS(pCylinder->P[1].Element, pCylinder->P[0].Element, Z_C_L);
            RVLNORM3(Z_C_L, h);
            RVLORTHOGONAL3(Z_C_L, Y_C_L, i_, j_, k_, fTmp);
            RVLCROSSPRODUCT3(Y_C_L, Z_C_L, X_C_L);
            RVLCOPYMX3X3T(R_L_C, pose_C_L.R);
            RVLSUM3VECTORS(pCylinder->P[0].Element, pCylinder->P[1].Element, pose_C_L.t);
            RVLSCALE3VECTOR(pose_C_L.t, 0.5f, pose_C_L.t);
            RVLCOMPTRANSF3D(robot.link_pose[iLink].R, robot.link_pose[iLink].t, pose_C_L.R, pose_C_L.t, pose_C_0.R, pose_C_0.t);
            RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pose_C_0.R, pose_C_0.t, pose_C_W.R, pose_C_W.t);
            pActors->push_back(pVisualizer->DisplayCylinder(pCylinder->r, h, &pose_C_W, 16, 1.0, 1.0, 1.0));
        }
    }

    // Pose3D pose_C_2;
    // RVLUNITMX3(pose_C_2.R);
    // RVLSET3VECTOR(pose_C_2.t, 0.0f, 0.0f, 0.091f);
    // Pose3D pose_C_0;
    // RVLCOMPTRANSF3D(robot.link_pose[1].R, robot.link_pose[1].t, pose_C_2.R, pose_C_2.t, pose_C_0.R, pose_C_0.t);
    // Pose3D pose_C_W;
    // RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pose_C_0.R, pose_C_0.t, pose_C_W.R, pose_C_W.t);
    // pActors->push_back(pVisualizer->DisplayCylinder(0.06f, 0.230f, &pose_C_W, 16, 1.0, 1.0, 1.0));

    // Pose3D pose_C_3;
    // RVLROTY(0.0f, 1.0f, pose_C_3.R);
    // RVLSET3VECTOR(pose_C_3.t, 0.184f, 0.0f, 0.0f);
    // RVLCOMPTRANSF3D(robot.link_pose[2].R, robot.link_pose[2].t, pose_C_3.R, pose_C_3.t, pose_C_0.R, pose_C_0.t);
    // RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pose_C_0.R, pose_C_0.t, pose_C_W.R, pose_C_W.t);
    // pActors->push_back(pVisualizer->DisplayCylinder(0.038f, 0.293f, &pose_C_W, 16, 1.0, 1.0, 1.0));
}

vtkSmartPointer<vtkActor> DDManipulator::VisualizeDoorPenel()
{
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    Pose3D pose_Arot_F;
    RVLCOMPTRANSF3D(pose_A_F.R, pose_A_F.t, pose_Arot_A.R, pose_Arot_A.t, pose_Arot_F.R, pose_Arot_F.t);
    Pose3D pose_Arot_S;
    Vector3<float> boxSize;
    Vector3<float> boxCenter;
    BoxSize<float>(&dd_panel_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&dd_panel_box, boxCenter.Element);
    RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_Arot_F.R, pose_Arot_F.t, pose_Arot_S.R, pose_Arot_S.t);
    Pose3D pose_box_S;
    RVLCOPYMX3X3(pose_Arot_S.R, pose_box_S.R);
    RVLTRANSF3(boxCenter.Element, pose_Arot_S.R, pose_Arot_S.t, pose_box_S.t);
    vtkSmartPointer<vtkActor> actor = pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);

    return actor;
}

void DDManipulator::SetVisualizeVNEnvironmentModel()
{
    pVisualizationData->bVNEnv = true;
}
#endif

Robot::Robot()
{
    n = 6;
    q = NULL;
    d = NULL;
    a = NULL;
    al = NULL;
    jointType = NULL;
    csal = NULL;
    snal = NULL;
    RVLUNITMX3(pose_TCP_6.R);
    RVLNULL3VECTOR(pose_TCP_6.t);
    epsilon = 0.0f;
    collisionCylinderMem = NULL;
    collisionCylinders.Element = NULL;
    maxCollisionLinkIdx = -1;
}

Robot::~Robot()
{
    Clear();
}

void Robot::Create(char *cfgFileNameIn)
{
    Clear();
    q = new float[n];
    memset(q, 0, 6 * sizeof(float));
    d = new float[n];
    a = new float[n];
    al = new float[n];
    jointType = new uchar[n];
    csal = new float[n];
    snal = new float[n];

    // UR5 (keating_14).

    memset(jointType, 0, n * sizeof(uchar));
    d[0] = 0.089159f;
    d[1] = 0.0f;
    d[2] = 0.0f;
    d[3] = 0.10915f;
    d[4] = 0.09465f;
    d[5] = 0.0823f;
    a[0] = 0.0f;
    a[1] = -0.425f;
    a[2] = -0.39225f;
    a[3] = 0.0f;
    a[4] = 0.0f;
    a[5] = 0.0f;
    al[0] = 0.5f * PI;
    al[1] = 0.0f;
    al[2] = 0.0f;
    al[3] = 0.5f * PI;
    al[4] = -0.5f * PI;
    al[5] = 0.0f;
    int i;
    for (i = 0; i < n; i++)
    {
        csal[i] = cos(al[i]);
        snal[i] = sin(al[i]);
    }

    // UR5 collision detection model.

    maxCollisionLinkIdx = 2;
    collisionCylinders.Element = new Array<MOTION::Cylinder>[6];
    for (i = 0; i < n; i++)
    {
        collisionCylinders.Element[i].n = 0;
        collisionCylinders.Element[i].Element = NULL;
    }
    collisionCylinderMem = new MOTION::Cylinder[2];
    MOTION::Cylinder *pCylinder = collisionCylinderMem;
    collisionCylinders.Element[1].n = 1;
    collisionCylinders.Element[1].Element = pCylinder;
    pCylinder->r = 0.06f;
    RVLSET3VECTOR(pCylinder->P[0].Element, 0.0f, 0.0f, -0.024f);
    RVLSET3VECTOR(pCylinder->P[1].Element, 0.0f, 0.0f, 0.206f);
    pCylinder++;
    collisionCylinders.Element[2].n = 1;
    collisionCylinders.Element[2].Element = pCylinder;
    pCylinder->r = 0.038f;
    RVLSET3VECTOR(pCylinder->P[0].Element, 0.0375f, 0.0f, 0.0f);
    RVLSET3VECTOR(pCylinder->P[1].Element, 0.3305f, 0.0f, 0.0f);

    // Load parameters from a configuration file.

    CreateParamList();
    paramList.LoadParams(cfgFileNameIn);

    //

    float rotz_TCP_6_rad = DEG2RAD * rotz_TCP_6;
    float cs = cos(rotz_TCP_6_rad);
    float sn = sin(rotz_TCP_6_rad);
    RVLROTZ(cs, sn, pose_TCP_6.R);
    maxr = 0.8f;
    minr = 0.1f;
    minz = 0.3f;
    RVLINVTRANSF3D(pose_TCP_6.R, pose_TCP_6.t, pose_6_G.R, pose_6_G.t);
    d4_2 = d[3] * d[3];
    k1 = a[1] * a[1] + a[2] * a[2];
    k2 = 2.0f * a[1] * a[2];
    maxa23_2 = k1 + k2;
    FwdKinematics();
    RVLNULLMX3X3(R_E_1);
    RVLMXEL(R_E_1, 3, 0, 0) = -1.0f;
    RVLMXEL(R_E_1, 3, 1, 2) = 1.0f;
    RVLMXEL(R_E_1, 3, 2, 1) = 1.0f;
    RVLNULLMX3X3(R_4_E);
    RVLMXEL(R_4_E, 3, 1, 1) = 1.0f;
    for (i = 0; i < 6; i++)
    {
        minq[i] = -2.0f * PI;
        maxq[i] = 2.0f * PI;
    }
    // minq[4] = 0.0f;
    // maxq[4] = DEG2RAD * 135.0f;
    epsilonRad = DEG2RAD * epsilon;
}

void Robot::CreateParamList()
{
    paramList.m_pMem = pMem0;
    RVLPARAM_DATA *pParamData;
    paramList.Init();
    pParamData = paramList.AddParam("Robot.t_TCP_6.x", RVLPARAM_TYPE_FLOAT, pose_TCP_6.t);
    pParamData = paramList.AddParam("Robot.t_TCP_6.y", RVLPARAM_TYPE_FLOAT, pose_TCP_6.t + 1);
    pParamData = paramList.AddParam("Robot.t_TCP_6.z", RVLPARAM_TYPE_FLOAT, pose_TCP_6.t + 2);
    pParamData = paramList.AddParam("Robot.rotz_TCP_6", RVLPARAM_TYPE_FLOAT, &rotz_TCP_6);
    pParamData = paramList.AddParam("Robot.epsilon", RVLPARAM_TYPE_FLOAT, &epsilon);
}

void Robot::Clear()
{
    RVL_DELETE_ARRAY(q);
    RVL_DELETE_ARRAY(d);
    RVL_DELETE_ARRAY(a);
    RVL_DELETE_ARRAY(al);
    RVL_DELETE_ARRAY(jointType);
    RVL_DELETE_ARRAY(csal);
    RVL_DELETE_ARRAY(snal);
    RVL_DELETE_ARRAY(collisionCylinders.Element);
    RVL_DELETE_ARRAY(collisionCylinderMem);
}

void Robot::FwdKinematics()
{
    Pose3D dPose;
    int i;
    float cq, sq;
    float *R, *t, *R_, *t_;
    for (i = 0; i < n; i++)
    {
        FwdKinematics(i, &dPose);
        R = link_pose[i].R;
        t = link_pose[i].t;
        if (i == 0)
        {
            RVLCOPYMX3X3(dPose.R, R);
            RVLCOPY3VECTOR(dPose.t, t);
        }
        else
        {
            R_ = link_pose[i - 1].R;
            t_ = link_pose[i - 1].t;
            RVLCOMPTRANSF3D(R_, t_, dPose.R, dPose.t, R, t);
        }
    }
}

void Robot::FwdKinematics(
    int i,
    Pose3D *pdPose)
{
    float cq, sq;
    FwdKinematicsRot(i, pdPose->R, cq, sq);
    RVLSET3VECTOR(pdPose->t, a[i] * cq, a[i] * sq, d[i]);
}

void Robot::FwdKinematics(
    float *qIn,
    Pose3D *pPose_G_0)
{
    memcpy(q, qIn, n * sizeof(float));
    FwdKinematics();
    Pose3D *pPose_n_0 = link_pose + n - 1;
    RVLCOMPTRANSF3D(pPose_n_0->R, pPose_n_0->t, pose_TCP_6.R, pose_TCP_6.t, pPose_G_0->R, pPose_G_0->t);
}

void Robot::FwdKinematicsRot(
    int i,
    float *R,
    float &cq,
    float &sq)
{
    cq = cos(q[i]);
    sq = sin(q[i]);
    RVLMXEL(R, 3, 0, 0) = cq;
    RVLMXEL(R, 3, 0, 1) = -sq * csal[i];
    RVLMXEL(R, 3, 0, 2) = sq * snal[i];
    RVLMXEL(R, 3, 1, 0) = sq;
    RVLMXEL(R, 3, 1, 1) = cq * csal[i];
    RVLMXEL(R, 3, 1, 2) = -cq * snal[i];
    RVLMXEL(R, 3, 2, 0) = 0.0f;
    RVLMXEL(R, 3, 2, 1) = snal[i];
    RVLMXEL(R, 3, 2, 2) = csal[i];
}

bool Robot::InvKinematics(
    Pose3D toolPose,
    float *qOut)
{
    if (!InvKinematics1E56(toolPose, qOut))
        return false;

    float *q_ = (qOut ? qOut : q);
    Pose3D *pPose_6_0 = link_pose + 5;
    Pose3D *pPose_1_0 = link_pose;
    RVLSET3VECTOR(pPose_1_0->t, 0.0f, 0.0f, d[0]);
    Pose3D pose_6_1;
    float V3Tmp[3];
    RVLCOMPTRANSF3DWITHINV(pPose_1_0->R, pPose_1_0->t, pPose_6_0->R, pPose_6_0->t, pose_6_1.R, pose_6_1.t, V3Tmp);
    Pose3D pose_5_4;
    FwdKinematics(4, &pose_5_4);
    Pose3D *pPose_5_0 = link_pose + 4;
    Pose3D pose_4_1;
    RVLMXMUL3X3(R_E_1, R_4_E, pose_4_1.R);
    float t_5_1[3];
    RVLINVTRANSF3(pPose_5_0->t, pPose_1_0->R, pPose_1_0->t, t_5_1, V3Tmp);
    RVLMULMX3X3VECT(pose_4_1.R, pose_5_4.t, V3Tmp);
    RVLDIF3VECTORS(t_5_1, V3Tmp, pose_4_1.t);
    Pose3D pose_3_1;
    float Y_4_1[3];
    RVLCOPYCOLMX3X3(pose_4_1.R, 1, Y_4_1);
    RVLSCALE3VECTOR(Y_4_1, d[3], V3Tmp);
    RVLDIF3VECTORS(pose_4_1.t, V3Tmp, pose_3_1.t);
    float r31_2 = RVLDOTPRODUCT3(pose_3_1.t, pose_3_1.t);
    float fTmp = (r31_2 - k1) / k2;
    if (fTmp > 1.0f || fTmp < -1.0f)
        return false;
    q_[2] = -acos(fTmp);
    fTmp = a[2] * sin(q_[2]) / sqrt(r31_2);
    if (fTmp > 1.0f || fTmp < -1.0f)
        return false;
    q_[1] = -atan2(pose_3_1.t[1], -pose_3_1.t[0]) + asin(fTmp);
    // q_[3] -= (q[1] + q[2]);
    q_[3] -= (q_[1] + q_[2] + PI);
    // q_[3] = -(q[1] + q[2] + PI);
    return true;
}

bool Robot::InvKinematics1E56(
    Pose3D pose_G_0,
    float *qOut)
{
    float *q_ = (qOut ? qOut : q);
    Pose3D *pPose_6_0 = link_pose + 5;
    RVLCOMPTRANSF3D(pose_G_0.R, pose_G_0.t, pose_6_G.R, pose_6_G.t, pPose_6_0->R, pPose_6_0->t);
    float Z_6_0[3];
    RVLCOPYCOLMX3X3(pPose_6_0->R, 2, Z_6_0);
    float V3Tmp[3];
    RVLSCALE3VECTOR(Z_6_0, d[5], V3Tmp);
    Pose3D *pPose_5_0 = link_pose + 4;
    RVLDIF3VECTORS(pPose_6_0->t, V3Tmp, pPose_5_0->t);
    float ps = atan2(pPose_5_0->t[1], pPose_5_0->t[0]);
    float t_5_0_xy_2 = pPose_5_0->t[0] * pPose_5_0->t[0] + pPose_5_0->t[1] * pPose_5_0->t[1];
    float fTmp = sqrt(t_5_0_xy_2);
    if (fTmp < d[3])
        return false;
    float r2 = t_5_0_xy_2 + pPose_5_0->t[2] * pPose_5_0->t[2];
    a23_2 = r2 - d4_2;
    // if (a23_2 > maxa23_2)
    //     return false;
    float ph = asin(d[3] / fTmp);
    q_[0] = ps + ph;
    float cq = cos(q_[0]);
    float sq = sin(q_[0]);
    float *R_1_0 = link_pose[0].R;
    RVLMXEL(R_1_0, 3, 0, 0) = cq;
    RVLMXEL(R_1_0, 3, 0, 2) = sq;
    RVLMXEL(R_1_0, 3, 1, 0) = sq;
    RVLMXEL(R_1_0, 3, 1, 2) = -cq;
    float R_E_0[9];
    RVLMXMUL3X3(R_1_0, R_E_1, R_E_0);
    float R_6_E[9];
    RVLMXMUL3X3T1(R_E_0, pPose_6_0->R, R_6_E);
    float Z_5_E[3];
    float X_6_E[3];
    float X_5_E[3];
    float Y_5_E[3];
    float X_5_4[3];
    RVLCOPYCOLMX3X3(R_6_E, 2, Z_5_E);
    if (Z_5_E[0] > 0.0f || Z_5_E[2] > 0.0f)
        return false;
    if (Z_5_E[0] * Z_5_E[0] + Z_5_E[2] * Z_5_E[2] < 1e-6)
        return false;
    float be = atan2(Z_5_E[2], -Z_5_E[0]);
    q_[3] = be;
    q_[4] = acos(Z_5_E[1]);
    if (q_[4] < minq[4] || q_[4] > maxq[4])
        return false;
    cq = cos(q_[4]);
    sq = sin(q_[4]);
    RVLSET3VECTOR(X_5_4, cq, sq, 0.0f);
    cq = cos(be);
    sq = sin(be);
    RVLMXEL(R_4_E, 3, 0, 0) = cq;
    RVLMXEL(R_4_E, 3, 0, 2) = sq;
    RVLMXEL(R_4_E, 3, 2, 0) = -sq;
    RVLMXEL(R_4_E, 3, 2, 2) = cq;
    RVLMULMX3X3VECT(R_4_E, X_5_4, X_5_E);
    RVLSET3VECTOR(Y_5_E, -sq, 0.0f, -cq);
    RVLCOPYCOLMX3X3(R_6_E, 0, X_6_E);
    cq = RVLDOTPRODUCT3(X_6_E, X_5_E);
    sq = RVLDOTPRODUCT3(X_6_E, Y_5_E);
    q_[5] = atan2(sq, cq);

    // Only for debugging purpose!!!

    // memcpy(q + 3, q_ + 3, 3 * sizeof(float));
    // float R_4_3[9];
    // FwdKinematicsRot(3, R_4_3, cq, sq);
    // float R_5_4[9];
    // FwdKinematicsRot(4, R_5_4, cq, sq);
    // float R_5_3[9];
    // RVLMXMUL3X3(R_4_3, R_5_4, R_5_3);
    // float R_3_E[9];
    // RVLNULLMX3X3(R_3_E);
    // RVLMXEL(R_3_E, 3, 0, 0) = 1.0f;
    // RVLMXEL(R_3_E, 3, 1, 2) = 1.0f;
    // RVLMXEL(R_3_E, 3, 2, 1) = -1.0f;
    // float R_5_E[9];
    // RVLMXMUL3X3(R_3_E, R_5_3, R_5_E);
    // float R_6_5[9];
    // FwdKinematicsRot(5, R_6_5, cq, sq);
    // float R_6_E_[9];
    // RVLMXMUL3X3(R_5_E, R_6_5, R_6_E_);
    // float R_err[9];
    // RVLMXMUL3X3T1(R_6_E_, R_6_E, R_err);
    // float debug = RVLROTDIFF(R_err);

    //

    return true;
}

void Robot::InvKinematicsApprox23(float *qOut)
{
    float *q_ = (qOut ? qOut : q);
    q_[2] = acos((k1 - a23_2) / k2);
    float a23 = sqrt(a23_2);
    q_[1] = asin(link_pose[4].t[2] / a23);
}

bool Robot::SelfCollision(
    float *q,
    float *t_3_1)
{
    float delta = atan2(t_3_1[1], -t_3_1[0]);
    float mu = delta + PI;
    RVLNORMANGLE(mu);
    float zeta = PI - q[1] - q[2];
    RVLNORMANGLE(zeta);
    float gamma = mu - zeta;
    RVLNORMANGLE(gamma);
    float minq4, maxq4;
    if (gamma >= 0.0f)
    {
        minq4 = gamma + epsilonRad;
        maxq4 = PI - epsilonRad;
    }
    else
    {
        minq4 = epsilonRad;
        maxq4 = gamma + PI - epsilonRad;
    }
    RVLNORMANGLE(minq4);
    RVLNORMANGLE(maxq4);
    float fTmp = -q[3] - minq4;
    RVLNORMANGLE(fTmp);
    if (fTmp < 0.0f)
        return true;
    fTmp = maxq4 + q[3];
    RVLNORMANGLE(fTmp);
    if (fTmp < 0.0f)
        return true;
    return false;
}

// This function is written according to the paper Ryan Keating, UR5 Inverse Kinematics, 2014. (keating_14)

bool Robot::InvKinematics(
    Pose3D toolPose,
    Array<IKSolution> &solutions,
    bool bTCP)
{
    Pose3D *pPose_6_0 = link_pose + 5;
    if (bTCP)
        RVLCOMPTRANSF3D(toolPose.R, toolPose.t, pose_6_G.R, pose_6_G.t, pPose_6_0->R, pPose_6_0->t)
    else
        *pPose_6_0 = toolPose;

    if (solutions.Element == NULL)
        solutions.Element = new MOTION::IKSolution[6 * 8];
    solutions.n = 0;

    // keating_14 (3)

    float Z_6_0[3];
    RVLCOPYCOLMX3X3(pPose_6_0->R, 2, Z_6_0);
    float V3Tmp[3];
    RVLSCALE3VECTOR(Z_6_0, d[5], V3Tmp);
    Pose3D *pPose_5_0 = link_pose + 4;
    RVLDIF3VECTORS(pPose_6_0->t, V3Tmp, pPose_5_0->t);

    // Only for debugging purpose!!!

    // float t_5_0[3];
    // RVLCOPY3VECTOR(pPose_5_0->t, t_5_0);

    // keating_14 (4)

    float ps = atan2(pPose_5_0->t[1], pPose_5_0->t[0]);

    // keating_14 (5)

    float t_5_0_xy_2 = pPose_5_0->t[0] * pPose_5_0->t[0] + pPose_5_0->t[1] * pPose_5_0->t[1];
    float fTmp = sqrt(t_5_0_xy_2);
    if (fTmp < d[3])
        return false;
    float ph = acos(d[3] / fTmp);

    //

    int i, j, k;
    MOTION::IKSolution *pSolution;
    for (i = 0; i < 2; i++, ph = -ph)
    {
        // q1 = ps + ph + pi/2

        q[0] = ps + ph + 0.5f * PI;
        RVLNORMANGLE(q[0]);

        if (q[0] < minq[0] || q[0] > maxq[0])
            continue;

        // keating_14 (6)

        float cq1 = cos(q[0]);
        float sq1 = sin(q[0]);
        float z_6_1 = pPose_6_0->t[0] * sq1 - pPose_6_0->t[1] * cq1;
        q[4] = acos((z_6_1 - d[3]) / d[5]);
        for (j = 0; j < 2; j++, q[4] = -q[4])
        {
            if (q[4] < minq[4] || q[4] > maxq[4])
                continue;

            // keating_14 (7)

            Pose3D *pPose_1_0 = link_pose;
            float *R_1_0 = pPose_1_0->R;
            RVLMXEL(R_1_0, 3, 0, 0) = cq1;
            RVLMXEL(R_1_0, 3, 0, 2) = sq1;
            RVLMXEL(R_1_0, 3, 1, 0) = sq1;
            RVLMXEL(R_1_0, 3, 1, 2) = -cq1;
            RVLSET3VECTOR(pPose_1_0->t, 0.0f, 0.0f, d[0]);
            Pose3D pose_6_1;
            RVLCOMPTRANSF3DWITHINV(pPose_1_0->R, pPose_1_0->t, pPose_6_0->R, pPose_6_0->t, pose_6_1.R, pose_6_1.t, V3Tmp);
            Pose3D pose_1_6;
            RVLINVTRANSF3D(pose_6_1.R, pose_6_1.t, pose_1_6.R, pose_1_6.t);

            // keating_14 (10)

            float sq5 = sin(q[4]);
            if (RVLABS(sq5) >= 1e-6)
                q[5] = atan2(-pose_1_6.R[5] / sq5, pose_1_6.R[2] / sq5);
            else
                q[5] = 0.0f;

            if (q[5] < minq[5] || q[5] > maxq[5])
                continue;

            // keating_14 (11)

            Pose3D pose_5_4;
            FwdKinematics(4, &pose_5_4);
            Pose3D pose_6_5;
            FwdKinematics(5, &pose_6_5);
            Pose3D pose_6_4;
            RVLCOMPTRANSF3D(pose_5_4.R, pose_5_4.t, pose_6_5.R, pose_6_5.t, pose_6_4.R, pose_6_4.t);
            Pose3D pose_4_6;
            RVLINVTRANSF3D(pose_6_4.R, pose_6_4.t, pose_4_6.R, pose_4_6.t);
            Pose3D pose_4_1;
            RVLCOMPTRANSF3D(pose_6_1.R, pose_6_1.t, pose_4_6.R, pose_4_6.t, pose_4_1.R, pose_4_1.t);

            // keating_14 (12)

            Pose3D pose_3_1;
            float Y_4_1[3];
            RVLCOPYCOLMX3X3(pose_4_1.R, 1, Y_4_1);
            RVLSCALE3VECTOR(Y_4_1, d[3], V3Tmp);
            RVLDIF3VECTORS(pose_4_1.t, V3Tmp, pose_3_1.t);

            // keating_14 (15)

            float r31_2 = RVLDOTPRODUCT3(pose_3_1.t, pose_3_1.t);
            float fTmp = (r31_2 - k1) / k2;
            if (fTmp > 1.0f || fTmp < -1.0f)
                continue;
            q[2] = acos(fTmp);
            for (k = 0; k < 2; k++, q[2] = -q[2])
            {
                if (q[2] < minq[2] || q[2] > maxq[2])
                    continue;

                // keating_14 (18)

                float delta = atan2(pose_3_1.t[1], -pose_3_1.t[0]);
                fTmp = a[2] * sin(q[2]) / sqrt(r31_2);
                if (fTmp > 1.0f || fTmp < -1.0f)
                    continue;
                q[1] = -delta + asin(fTmp);
                RVLNORMANGLE(q[1]);

                if (q[1] < minq[1] || q[1] > maxq[1])
                    continue;

                // if (q[1] > -0.5 * PI)
                //{
                //     if (q[2] < 0.0f)
                //         continue;
                // }
                // else
                //{
                //     if (q[2] > 0.0f)
                //         continue;
                // }

                // keating_14 (19)

                Pose3D pose_3_2;
                FwdKinematics(2, &pose_3_2);
                Pose3D pose_2_1;
                FwdKinematics(1, &pose_2_1);
                Pose3D pose_3_1;
                RVLCOMPTRANSF3D(pose_2_1.R, pose_2_1.t, pose_3_2.R, pose_3_2.t, pose_3_1.R, pose_3_1.t);
                Pose3D pose_1_3;
                RVLINVTRANSF3D(pose_3_1.R, pose_3_1.t, pose_1_3.R, pose_1_3.t);
                Pose3D pose_4_3;
                RVLCOMPTRANSF3D(pose_1_3.R, pose_1_3.t, pose_4_1.R, pose_4_1.t, pose_4_3.R, pose_4_3.t);

                // keating_14 (20)

                q[3] = atan2(pose_4_3.R[3], pose_4_3.R[0]);

                if (q[3] < minq[3] || q[3] > maxq[3])
                    continue;

                // Constraint for avoiding self collision.

                if (SelfCollision(q, pose_3_1.t))
                    continue;

                // Copy the current solution to the solution stack.

                pSolution = solutions.Element + solutions.n;
                pSolution->i = 2 * (2 * i + j) + k;
                memcpy(pSolution->q, q, 6 * sizeof(float));
                solutions.n++;

                // Only for debugging purpose!!!

                // FwdKinematics();
                // int debug = 0;
            }
        }
    }
}

bool Robot::InvKinematicsPrev(
    Pose3D pose_6_0,
    Array2D<float> &qOut)
{
    if (qOut.Element == NULL)
        qOut.Element = new float[6 * 8];

    Pose3D *pPose_6_0 = link_pose + 5;
    *pPose_6_0 = pose_6_0;

    // keating_14 (3)

    float Z_6_0[3];
    RVLCOPYCOLMX3X3(pPose_6_0->R, 2, Z_6_0);
    float V3Tmp[3];
    RVLSCALE3VECTOR(Z_6_0, d[5], V3Tmp);
    Pose3D *pPose_5_0 = link_pose + 4;
    RVLDIF3VECTORS(pPose_6_0->t, V3Tmp, pPose_5_0->t);

    // Only for debugging purpose!!!

    float t_5_0[3];
    RVLCOPY3VECTOR(pPose_5_0->t, t_5_0);

    // keating_14 (4)

    float ps = atan2(pPose_5_0->t[1], pPose_5_0->t[0]);

    // keating_14 (5)

    float t_5_0_xy_2 = pPose_5_0->t[0] * pPose_5_0->t[0] + pPose_5_0->t[1] * pPose_5_0->t[1];
    float fTmp = sqrt(t_5_0_xy_2);
    if (fTmp < d[3])
        return false;
    float ph = acos(d[3] / fTmp);

    //

    float *q_;
    int i, j, k;
    qOut.w = 6;
    qOut.h = 0;
    for (i = 0; i < 2; i++, ph = -ph)
    {
        // q1 = ps + ph + pi/2

        q[0] = ps + ph + 0.5f * PI;

        // keating_14 (6)

        float cq1 = cos(q[0]);
        float sq1 = sin(q[0]);
        float z_6_1 = pPose_6_0->t[0] * sq1 - pPose_6_0->t[1] * cq1;
        q[4] = acos((z_6_1 - d[3]) / d[5]);
        for (j = 0; j < 2; j++, q[4] = -q[4])
        {
            // keating_14 (7)

            Pose3D *pPose_1_0 = link_pose;
            float *R_1_0 = pPose_1_0->R;
            RVLMXEL(R_1_0, 3, 0, 0) = cq1;
            RVLMXEL(R_1_0, 3, 0, 2) = sq1;
            RVLMXEL(R_1_0, 3, 1, 0) = sq1;
            RVLMXEL(R_1_0, 3, 1, 2) = -cq1;
            RVLSET3VECTOR(pPose_1_0->t, 0.0f, 0.0f, d[0]);
            Pose3D pose_6_1;
            RVLCOMPTRANSF3DWITHINV(pPose_1_0->R, pPose_1_0->t, pPose_6_0->R, pPose_6_0->t, pose_6_1.R, pose_6_1.t, V3Tmp);
            Pose3D pose_1_6;
            RVLINVTRANSF3D(pose_6_1.R, pose_6_1.t, pose_1_6.R, pose_1_6.t);

            // keating_14 (10)

            float sq5 = sin(q[4]);
            if (RVLABS(sq5) >= 1e-6)
                q[5] = atan2(-pose_1_6.R[5] / sq5, pose_1_6.R[2] / sq5);
            else
                q[5] = 0.0f;

            // keating_14 (11)

            Pose3D pose_5_4;
            FwdKinematics(4, &pose_5_4);
            Pose3D pose_6_5;
            FwdKinematics(5, &pose_6_5);
            Pose3D pose_6_4;
            RVLCOMPTRANSF3D(pose_5_4.R, pose_5_4.t, pose_6_5.R, pose_6_5.t, pose_6_4.R, pose_6_4.t);
            Pose3D pose_4_6;
            RVLINVTRANSF3D(pose_6_4.R, pose_6_4.t, pose_4_6.R, pose_4_6.t);
            Pose3D pose_4_1;
            RVLCOMPTRANSF3D(pose_6_1.R, pose_6_1.t, pose_4_6.R, pose_4_6.t, pose_4_1.R, pose_4_1.t);

            // keating_14 (12)

            Pose3D pose_3_1;
            float Y_4_1[3];
            RVLCOPYCOLMX3X3(pose_4_1.R, 1, Y_4_1);
            RVLSCALE3VECTOR(Y_4_1, d[3], V3Tmp);
            RVLDIF3VECTORS(pose_4_1.t, V3Tmp, pose_3_1.t);

            // keating_14 (15)

            float r31_2 = RVLDOTPRODUCT3(pose_3_1.t, pose_3_1.t);
            float fTmp = (r31_2 - k1) / k2;
            if (fTmp > 1.0f || fTmp < -1.0f)
                continue;
            q[2] = acos(fTmp);
            for (k = 0; k < 2; k++, q[2] = -q[2])
            {
                // keating_14 (18)

                fTmp = a[2] * sin(q[2]) / sqrt(r31_2);
                if (fTmp > 1.0f || fTmp < -1.0f)
                    continue;
                q[1] = -atan2(pose_3_1.t[1], -pose_3_1.t[0]) + asin(fTmp);

                // keating_14 (19)

                Pose3D pose_3_2;
                FwdKinematics(2, &pose_3_2);
                Pose3D pose_2_1;
                FwdKinematics(1, &pose_2_1);
                Pose3D pose_3_1;
                RVLCOMPTRANSF3D(pose_2_1.R, pose_2_1.t, pose_3_2.R, pose_3_2.t, pose_3_1.R, pose_3_1.t);
                Pose3D pose_1_3;
                RVLINVTRANSF3D(pose_3_1.R, pose_3_1.t, pose_1_3.R, pose_1_3.t);
                Pose3D pose_4_3;
                RVLCOMPTRANSF3D(pose_1_3.R, pose_1_3.t, pose_4_1.R, pose_4_1.t, pose_4_3.R, pose_4_3.t);

                // keating_14 (20)

                q[3] = atan2(pose_4_3.R[3], pose_4_3.R[0]);

                // Copy the current solution to the solution stack.

                q_ = qOut.Element + qOut.w * qOut.h;
                memcpy(q_, q, 6 * sizeof(float));
                qOut.h++;

                // Only for debugging purpose!!!

                FwdKinematics();
                int debug = 0;
            }
        }
    }
}

// SIMUNDIC - FCL
void DDManipulator::RVLPose2FCLPose(Pose3D poseRVL, fcl::Transform3<double> &poseFCL)
{
    // Convert RVL poses to FCL poses
    // fcl::Transform3<double> poseFCL = fcl::Transform3<double>::Identity();
    Eigen::Matrix3d rotation;
    rotation << static_cast<double>(poseRVL.R[0]), static_cast<double>(poseRVL.R[1]), static_cast<double>(poseRVL.R[2]),
        static_cast<double>(poseRVL.R[3]), static_cast<double>(poseRVL.R[4]), static_cast<double>(poseRVL.R[5]),
        static_cast<double>(poseRVL.R[6]), static_cast<double>(poseRVL.R[7]), static_cast<double>(poseRVL.R[8]);
    poseFCL.translation() = Eigen::Vector3d(static_cast<double>(poseRVL.t[0]), static_cast<double>(poseRVL.t[1]), static_cast<double>(poseRVL.t[2]));
    poseFCL.linear() = rotation;
    // Eigen::Matrix4d matrix = poseFCL.matrix();
    // matrix(3, 0) = 0.0;
    // matrix(3, 1) = 0.0;
    // matrix(3, 2) = 0.0;
    // matrix(3, 3) = 1.0;
    // poseFCL = fcl::Transform3<double>(matrix);
}

void DDManipulator::GetVerticesFromPolyData(vtkSmartPointer<vtkPolyData> &vtkPolyData, std::vector<fcl::Vector3<double>> &vertices)
{
    if (!vtkPolyData || !vtkPolyData->GetPoints())
    {
        throw std::runtime_error("Invalid vtkPolyData or vtkPoints.");
    }

    vtkSmartPointer<vtkPoints> points = vtkPolyData->GetPoints();
    vertices.reserve(points->GetNumberOfPoints());
    double point[3];
    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i)
    {
        points->GetPoint(i, point);
        vertices.emplace_back(point[0], point[1], point[2]);
    }
}

void DDManipulator::GetTrianglesFromPolyData(vtkSmartPointer<vtkPolyData> &vtkPolyData, std::vector<fcl::Triangle> &triangles)
{
    vtkSmartPointer<vtkCellArray> polys = vtkPolyData->GetPolys();
    vtkIdType npts;
    vtkIdType *pts; // Declare a non-const pointer
    polys->InitTraversal();
    while (polys->GetNextCell(npts, pts))
    {
        if (npts == 3)
        { // Ensure it's a triangle
            triangles.emplace_back(pts[0], pts[1], pts[2]);
        }
    }
}

void DDManipulator::LoadToolModelFCL()
{
    // FCL TOOL MESH
    std::vector<fcl::Vector3<double>> vertices;
    GetVerticesFromPolyData(pToolMesh->pPolygonData, vertices);

    // Extract triangles
    std::vector<fcl::Triangle> triangles;
    GetTrianglesFromPolyData(pToolMesh->pPolygonData, triangles);

    // Create an FCL BVHModel
    fclToolMesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();
    fclToolMesh->beginModel();
    fclToolMesh->addSubModel(vertices, triangles);
    fclToolMesh->endModel();
}

void DDManipulator::LoadCabinetStaticFCL(std::string cabinetStaticFilename_, Pose3D pose_A_S)
{
    fcl::Transform3d T_A_S = fcl::Transform3<double>::Identity();
    RVLPose2FCLPose(pose_A_S, T_A_S);
    cabinetStaticFilename = cabinetStaticFilename_;
    pCabinetMeshStatic = new Mesh;
    pCabinetMeshStatic->LoadPolyDataFromPLY((char *)(cabinetStaticFilename.data()));

    // //Debug only
    // pCabinetWholeMesh = new Mesh;
    // std::string cabinetWholeName = "/home/RVLuser/ferit_ur5_ws/cabinet_whole.ply";
    // pCabinetWholeMesh->LoadPolyDataFromPLY((char *)(cabinetWholeName.data()));

    std::vector<fcl::Vector3<double>> vertices;
    GetVerticesFromPolyData(pCabinetMeshStatic->pPolygonData, vertices);
    std::vector<fcl::Triangle> triangles;
    GetTrianglesFromPolyData(pCabinetMeshStatic->pPolygonData, triangles);

    // Create an FCL BVHModel
    fclCabinetStaticMesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

    fclCabinetStaticMesh->beginModel();
    fclCabinetStaticMesh->addSubModel(vertices, triangles);
    fclCabinetStaticMesh->endModel();

    collisionCabinetObj = std::make_shared<fcl::CollisionObject<double>>(fclCabinetStaticMesh, T_A_S);
    // collisionCabinetObj = new fcl::CollisionObjectd(fclCabinetStaticMesh, T_A_S);
}

void DDManipulator::LoadCabinetPanelFCL(std::string cabinetPanelFilename_)
{
    // fcl::Transform3d T_A_S = fcl::Transform3<double>::Identity();
    // RVLPose2FCLPose(pose_A_S, T_A_S);
    cabinetPanelFilename = cabinetPanelFilename_;
    pCabinetMeshPanel = new Mesh;
    pCabinetMeshPanel->LoadPolyDataFromPLY((char *)(cabinetPanelFilename.data()));

    // //Debug only
    // pCabinetWholeMesh = new Mesh;
    // std::string cabinetWholeName = "/home/RVLuser/ferit_ur5_ws/cabinet_whole.ply";
    // pCabinetWholeMesh->LoadPolyDataFromPLY((char *)(cabinetWholeName.data()));

    std::vector<fcl::Vector3<double>> vertices;
    GetVerticesFromPolyData(pCabinetMeshPanel->pPolygonData, vertices);
    std::vector<fcl::Triangle> triangles;
    GetTrianglesFromPolyData(pCabinetMeshPanel->pPolygonData, triangles);

    // Create an FCL BVHModel
    fclCabinetPanelMesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

    fclCabinetPanelMesh->beginModel();
    fclCabinetPanelMesh->addSubModel(vertices, triangles);
    fclCabinetPanelMesh->endModel();

    // collisionCabinetObj = std::make_shared<fcl::CollisionObject<double>>(fclCabinetPanelMesh, T_A_S);
}

void DDManipulator::CreateRobotCylindersFCL()
{
    MOTION::Cylinder *pCylinder;
    int iLink, i;
    float R_L_C[9];
    float *Z_C_L = R_L_C + 6;
    float h;
    float r;
    for (iLink = 0; iLink <= robot.maxCollisionLinkIdx; iLink++)
    {
        std::vector<std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>> fclRobotCylinderMeshes_;
        fclRobotCylinderMeshes.push_back(std::vector<std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>>());
        for (i = 0; i < robot.collisionCylinders.Element[iLink].n; i++)
        {
            pCylinder = robot.collisionCylinders.Element[iLink].Element + i;
            
            RVLDIF3VECTORS(pCylinder->P[1].Element, pCylinder->P[0].Element, Z_C_L);
            RVLNORM3(Z_C_L, h);
            r = pCylinder->r;

            // Create a cylinder
            vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
            cylinderSource->SetRadius(r);
            cylinderSource->SetHeight(h);
            cylinderSource->SetResolution(16);
            cylinderSource->Update();
            
            vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
            transform->RotateX(-90); // rotate so that original y-axis becomes z-axis

            vtkSmartPointer<vtkTransformFilter> transformFilter = vtkSmartPointer<vtkTransformFilter>::New();
            transformFilter->SetTransform(transform);
            transformFilter->SetInputConnection(cylinderSource->GetOutputPort());
            transformFilter->Update();

            vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::SafeDownCast(transformFilter->GetOutput());

            // vtkPolyData* polyData = vtkPolyData::SafeDownCast(mapper->GetInput());
            // vtkSmartPointer<vtkPolyData> polyData = cylinderSource->GetOutput();
            vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
            triangleFilter->SetInputData(polyData);
            triangleFilter->Update();
            vtkSmartPointer<vtkPolyData> triangulatedPolyData = triangleFilter->GetOutput();

            // Get vertices and triangles
            std::vector<fcl::Vector3<double>> vertices;
            GetVerticesFromPolyData(triangulatedPolyData, vertices);
            std::vector<fcl::Triangle> triangles;
            GetTrianglesFromPolyData(triangulatedPolyData, triangles);

            std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> fclCylinderMesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();
            fclCylinderMesh->beginModel();
            fclCylinderMesh->addSubModel(vertices, triangles);
            fclCylinderMesh->endModel();
            fclRobotCylinderMeshes[iLink].push_back(fclCylinderMesh);
        }
        // cout << "Cylinders in link " << iLink << ": " << fclRobotCylinderMeshes[iLink].size() << endl;
    }
}

void DDManipulator::CreateGndFCL()
{
    vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
    // Set the dimensions: large in x and y, thin in z.
    cubeSource->SetXLength(1.0);
    cubeSource->SetYLength(1.0);
    cubeSource->SetZLength(0.01);  // Very thin to mimic a plane.
    cubeSource->SetCenter(0.0, 0.0, -0.005);
    cubeSource->Update();
    vtkSmartPointer<vtkPolyData> groundPolyData = cubeSource->GetOutput();
    
    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(groundPolyData);
    triangleFilter->Update();
    vtkSmartPointer<vtkPolyData> triangulatedPolyData = triangleFilter->GetOutput();

    // Get vertices and triangles
    std::vector<fcl::Vector3<double>> vertices;
    GetVerticesFromPolyData(triangulatedPolyData, vertices);
    std::vector<fcl::Triangle> triangles;
    GetTrianglesFromPolyData(triangulatedPolyData, triangles);

    fclGndMesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();
    fclGndMesh->beginModel();
    fclGndMesh->addSubModel(vertices, triangles);
    fclGndMesh->endModel();
    


    collisionGndObj = std::make_shared<fcl::CollisionObject<double>>(fclGndMesh, fcl::Transform3d::Identity());
}


bool DDManipulator::FreeFCL(Pose3D *pPose_G_S)
{
    fcl::Transform3d T_G_S = fcl::Transform3<double>::Identity();
    RVLPose2FCLPose(*pPose_G_S, T_G_S);
    // fcl::CollisionObjectd* collisionToolObj = new fcl::CollisionObjectd(fclToolMesh, T_G_S);
    std::shared_ptr<fcl::CollisionObject<double>> collisionToolObj = std::make_shared<fcl::CollisionObject<double>>(fclToolMesh, T_G_S);

    fcl::CollisionResult<double> resultCabinet, resultGnd;
    fcl::CollisionRequest<double> request;
    fcl::collide(collisionToolObj.get(), collisionCabinetObj.get(), request, resultCabinet);
    fcl::collide(collisionToolObj.get(), collisionGndObj.get(), request, resultGnd);
    // delete collisionToolObj;
    // // Debug only!
    // if (!isColliding)
    // {
    //     std::cout << "\n" << "T_A_S: \n";
    //     for (int i = 0; i < 9; ++i) {
    //         std::cout << pose_A_S.R[i] << ((i % 3 == 2) ? "\n" : " ");
    //     }
    //     for (int i = 0; i < 3; ++i) {
    //         std::cout << pose_A_S.t[i] << " ";
    //     }
    //     vtkSmartPointer<vtkActor> actorCabinet, actorTool;
    //     Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    //     VisualizeToolMesh(*pPose_G_S, actorCabinet);
    //     VisualizeCabinetStaticMesh(pose_A_S, actorTool);
    //     pVisualizer->Run();
    //     pVisualizer->renderer->RemoveViewProp(actorCabinet);
    //     pVisualizer->renderer->RemoveViewProp(actorTool);
    // }

    return !(resultCabinet.isCollision() || resultGnd.isCollision());
    // return !resultCabinet.isCollision();
}

bool DDManipulator::FreeFCL(Pose3D *pPose_G_S_start, Pose3D *pPose_G_S_end) // Deprecated - not in use
{
    fcl::Transform3d T_G_S_start = fcl::Transform3<double>::Identity();
    fcl::Transform3d T_G_S_end = fcl::Transform3<double>::Identity();
    RVLPose2FCLPose(*pPose_G_S_start, T_G_S_start);
    RVLPose2FCLPose(*pPose_G_S_end, T_G_S_end);

    // fcl::CollisionObjectd* collisionToolObj_start = new fcl::CollisionObjectd(fclToolMesh, T_G_S_start);
    // fcl::CollisionObjectd* collisionToolObj_end = new fcl::CollisionObjectd(fclToolMesh, T_G_S_end);
    std::shared_ptr<fcl::CollisionObject<double>> collisionToolObj_start = std::make_shared<fcl::CollisionObject<double>>(fclToolMesh, T_G_S_start);
    std::shared_ptr<fcl::CollisionObject<double>> collisionToolObj_end = std::make_shared<fcl::CollisionObject<double>>(fclToolMesh, T_G_S_end);

    fcl::CollisionRequest<double> request1;
    fcl::CollisionResult<double> result1;
    fcl::CollisionRequest<double> request2;
    fcl::CollisionResult<double> result2;

    bool isColliding;
    isColliding = fcl::collide(collisionToolObj_start.get(), collisionCabinetObj.get(), request1, result1);
    // delete collisionToolObj_start;
    if (isColliding)
        return false;
    isColliding = fcl::collide(collisionToolObj_end.get(), collisionCabinetObj.get(), request2, result2);
    // delete collisionToolObj_end;
    if (isColliding)
        return false;
    return true;
}

bool DDManipulator::CylinderIntersectionFCL(double r, double h, Pose3D pose_C_S) // Deprecated - not in use
{
    // Convert RVL poses to FCL poses
    fcl::Transform3<double> T_C_S = fcl::Transform3<double>::Identity();
    RVLPose2FCLPose(pose_C_S, T_C_S);
    Pose3D unitPose;
    RVLUNITMX3(unitPose.R);

	// Create a cylinder
	vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
	cylinderSource->SetRadius(r);
	cylinderSource->SetHeight(h);
	cylinderSource->SetResolution(16);
    cylinderSource->Update();
    // vtkPolyData* polyData = vtkPolyData::SafeDownCast(mapper->GetInput());
    vtkSmartPointer<vtkPolyData> polyData = cylinderSource->GetOutput();
    vtkSmartPointer<vtkTriangleFilter> triangleFilter =
        vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(polyData);
    triangleFilter->Update();
    vtkSmartPointer<vtkPolyData> triangulatedPolyData = triangleFilter->GetOutput();

    std::vector<fcl::Vector3<double>> vertices;
    GetVerticesFromPolyData(triangulatedPolyData, vertices);
    std::vector<fcl::Triangle> triangles;
    GetTrianglesFromPolyData(triangulatedPolyData, triangles);
    // std::cout << "Vertices count: " << vertices.size() << std::endl;
    // std::cout << "Triangles count: " << triangles.size() << std::endl;

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> fclCylinderMesh = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

    fclCylinderMesh->beginModel();
    fclCylinderMesh->addSubModel(vertices, triangles);
    fclCylinderMesh->endModel();


    // Create a collision object for the cylinder
    // fcl::CollisionObjectd* cylinderObj = new fcl::CollisionObjectd(fclCylinderMesh, T_C_S);
    std::shared_ptr<fcl::CollisionObject<double>> cylinderObj = std::make_shared<fcl::CollisionObject<double>>(fclCylinderMesh, T_C_S);

    Pose3D pose_A_S;
    RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_A_F.R, pose_A_F.t, pose_A_S.R, pose_A_S.t);
    Pose3D poseArot_A, poseArot_S;
    float cs = cos(dd_state_angle);
    float sn = sin(dd_state_angle);
    RVLROTZ(cs, sn, pose_Arot_A.R);
    RVLCOMPTRANSF3D(pose_A_S.R, pose_A_S.t, pose_Arot_A.R, pose_Arot_A.t, poseArot_S.R, poseArot_S.t);
    fcl::Transform3d TArot_S;
    RVLPose2FCLPose(poseArot_S, TArot_S);

    // Door panel object
    std::shared_ptr<fcl::CollisionObject<double>> panelObj = std::make_shared<fcl::CollisionObject<double>>(fclCabinetPanelMesh, TArot_S);
    // fcl::CollisionObjectd* panelObj = new fcl::CollisionObjectd(fclCabinetPanelMesh, TArot_S);


    fcl::Transform3d T_A_S = fcl::Transform3<double>::Identity();
    RVLPose2FCLPose(pose_A_S, T_A_S);
    
    // Create a collision request & result
    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> resultStatic, resultPanel;
    
    
    if (!cylinderObj || !collisionCabinetObj) {
        std::cerr << "One or both collision objects are null!" << std::endl;
    }
    fcl::Transform3d transform = cylinderObj->getTransform();

    // Get the translation (as an fcl::Vector3d)
    fcl::Vector3d translation = transform.translation();

    // Get the rotation (as an fcl::Matrix3d)
    fcl::Matrix3d rotation = transform.linear();

    // std::cout << "Translation: " << translation.transpose() << std::endl;
    // std::cout << "Rotation matrix:\n" << rotation << std::endl;

    // Perform collision check between the cylinder and the static parts
    int numContactsStatic = fcl::collide(cylinderObj.get(), collisionCabinetObj.get(), request, resultStatic);
    int numContactsPanel = fcl::collide(cylinderObj.get(), panelObj.get(), request, resultPanel);
    return resultStatic.isCollision() || resultPanel.isCollision();
}

bool DDManipulator::CylinderIntersectionFCL(std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> fclCylinderMesh, Pose3D pose_C_S)
{

    // Convert RVL poses to FCL poses
    fcl::Transform3<double> T_C_S = fcl::Transform3<double>::Identity();
    RVLPose2FCLPose(pose_C_S, T_C_S);


    // Create a collision object for the cylinder
    std::shared_ptr<fcl::CollisionObject<double>> cylinderObj = std::make_shared<fcl::CollisionObject<double>>(fclCylinderMesh, T_C_S);

    Pose3D pose_A_S;
    RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_A_F.R, pose_A_F.t, pose_A_S.R, pose_A_S.t);
    Pose3D poseArot_A, poseArot_S;
    float cs = cos(dd_state_angle);
    float sn = sin(dd_state_angle);
    RVLROTZ(cs, sn, pose_Arot_A.R);
    RVLCOMPTRANSF3D(pose_A_S.R, pose_A_S.t, pose_Arot_A.R, pose_Arot_A.t, poseArot_S.R, poseArot_S.t);
    fcl::Transform3d TArot_S;
    RVLPose2FCLPose(poseArot_S, TArot_S);

    // Door panel object
    std::shared_ptr<fcl::CollisionObject<double>> panelObj = std::make_shared<fcl::CollisionObject<double>>(fclCabinetPanelMesh, TArot_S);
    
    // Create a collision request & result
    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> resultStatic, resultPanel;
    
    // Perform collision check between the cylinder and the static parts
    int numContactsStatic = fcl::collide(cylinderObj.get(), collisionCabinetObj.get(), request, resultStatic);
    int numContactsPanel = fcl::collide(cylinderObj.get(), panelObj.get(), request, resultPanel);
    return resultStatic.isCollision() || resultPanel.isCollision();
}


void DDManipulator::VisualizeCabinetStaticMesh(Pose3D pose_A_S, vtkSmartPointer<vtkActor> &actor)
{
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    double T[16];
    RVLHTRANSFMX(pose_A_S.R, pose_A_S.t, T);
    transform->SetMatrix(T);
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(pCabinetMeshStatic->pPolygonData);
    transformFilter->SetTransform(transform);
    transformFilter->Update();
    pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
    pVisualizer->map->SetInputConnection(transformFilter->GetOutputPort());
    pVisualizer->map->InterpolateScalarsBeforeMappingOff();
    pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
    pVisualizer->actor->SetMapper(pVisualizer->map);
    pVisualizer->renderer->AddActor(pVisualizer->actor);

    actor = pVisualizer->actor;
}

void DDManipulator::VisualizeCabinetWholeMesh(Pose3D pose_A_S, vtkSmartPointer<vtkActor> &actor)
{
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    double T[16];
    RVLHTRANSFMX(pose_A_S.R, pose_A_S.t, T);
    transform->SetMatrix(T);
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(pCabinetWholeMesh->pPolygonData);
    transformFilter->SetTransform(transform);
    transformFilter->Update();
    pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
    pVisualizer->map->SetInputConnection(transformFilter->GetOutputPort());
    pVisualizer->map->InterpolateScalarsBeforeMappingOff();
    pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
    pVisualizer->actor->SetMapper(pVisualizer->map);
    pVisualizer->renderer->AddActor(pVisualizer->actor);

    actor = pVisualizer->actor;
}

void DDManipulator::VisualizeToolMesh(Pose3D pose_G_S, vtkSmartPointer<vtkActor> &actor)
{
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    double T[16];
    RVLHTRANSFMX(pose_G_S.R, pose_G_S.t, T);
    transform->SetMatrix(T);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(pToolMesh->pPolygonData);
    transformFilter->SetTransform(transform);
    transformFilter->Update();
    pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
    pVisualizer->map->SetInputConnection(transformFilter->GetOutputPort());
    pVisualizer->map->InterpolateScalarsBeforeMappingOff();
    pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
    pVisualizer->actor->SetMapper(pVisualizer->map);
    pVisualizer->renderer->AddActor(pVisualizer->actor);

    actor = pVisualizer->actor;
}

void DDManipulator::VisualizeDoorPanelMesh(vtkSmartPointer<vtkActor> &actor)
{
    Pose3D pose_Arot_F;
    RVLCOMPTRANSF3D(pose_A_F.R, pose_A_F.t, pose_Arot_A.R, pose_Arot_A.t, pose_Arot_F.R, pose_Arot_F.t);
    Pose3D pose_Arot_S;
    RVLCOMPTRANSF3D(pose_F_S.R, pose_F_S.t, pose_Arot_F.R, pose_Arot_F.t, pose_Arot_S.R, pose_Arot_S.t);

    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    double T[16];
    RVLHTRANSFMX(pose_Arot_S.R, pose_Arot_S.t, T);
    transform->SetMatrix(T);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(pCabinetMeshPanel->pPolygonData);
    transformFilter->SetTransform(transform);
    transformFilter->Update();
    pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
    pVisualizer->map->SetInputConnection(transformFilter->GetOutputPort());
    pVisualizer->map->InterpolateScalarsBeforeMappingOff();
    pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
    pVisualizer->actor->SetMapper(pVisualizer->map);
    pVisualizer->renderer->AddActor(pVisualizer->actor);

    actor = pVisualizer->actor;

}
