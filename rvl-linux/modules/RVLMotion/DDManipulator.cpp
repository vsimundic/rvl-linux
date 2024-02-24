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
#include "RRT.h"
#include "DDManipulator.h"
#include "cnpy.h"

#ifndef RVLLINUX
#define RVLDDMANIPULATOR_TIME_MESUREMENT
#endif

using namespace RVL;
using namespace MOTION;

DDManipulator::DDManipulator()
{
    resultsFolder = NULL;
    bLog = false;
    pTimer = NULL;

    // Door model parameters.

    dd_contact_surface_params[0] = dd_contact_surface_params[1] = 0.1f;
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

    // Door pose.

    RVLUNITMX3(pose_F_S.R);
    RVLNULL3VECTOR(pose_F_S.t);

    // Environment VN model.

    pVNEnv = NULL;
    VNMClusters.n = 0;
    VNMClusters.Element = NULL;
    dVNEnv = NULL;

    // Tool model.

    bDefaultToolModel = true;
    RVLSET3VECTOR(tool_contact_surface_params[0].Element, 0.0, 0.01, 0.0);
    RVLSET3VECTOR(tool_contact_surface_params[1].Element, 0.0, 0.01, -0.02);
    RVLSET3VECTOR(tool_contact_surface_params[2].Element, -0.02, 0.01, 0.0);
    RVLSET3VECTOR(tool_finger_size.Element, 0.02, 0.02, 0.06);
    tool_finger_distance = 0.06;    // m
    RVLSET3VECTOR(tool_palm_size.Element, 0.10, 0.02, 0.02);
    tool_wrist_len = 0.21f; // m
    tool_wrist_r = 0.03f;   // m
    tool_len = tool_finger_size.Element[2] + tool_palm_size.Element[2] + tool_wrist_len;
    RVLSET3VECTOR(tool_bounding_sphere.c.Element, 0.0f, 0.0f, -0.5f * tool_len);
    tool_bounding_sphere.r = 0.1537f;   // mm
    tool_sample_spheres.n = 0;
    tool_sample_spheres.Element = NULL;
    pToolMesh = NULL;
    RVLSET3VECTOR(PRTCP_G, -0.5f * tool_finger_distance, 0.0f, 0.0f);

    // Robot.

    RVLSET3VECTOR(robot.pose_TCP_6.t, 0.0f, 0.0f, 0.290f);

    // Path planning.

    maxnSE3Points = 20000;
    kTemplateEndTol = 0.1f;
    maxSurfaceContactAngle = 45.0f;   // deg
    visionTol = 0.01f;    // m
    minDistanceToAxis = 0.15; // m
    bLock_T_G_DD = false;

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
}

DDManipulator::~DDManipulator()
{
	Clear();
}

void DDManipulator::Create(char* cfgFileNameIn)
{
    // Load paramters from a configuration file.

    cfgFileName = cfgFileNameIn;
    CreateParamList();
    paramList.LoadParams(cfgFileNameIn);

    // Pose of the contact surface with respect to A.

    RVLSET3VECTOR(pose_DD_A.t, dd_rx-0.5f*dd_sx, dd_ry-0.5f*dd_sy, 0.5f*dd_sz);

    /// Create environment VN model.

    // cluster0 - door panel
    // cluster1 - box
    // cluster2 - storage space
    // cluster3 - box with storage space
    // cluster4 - complete furniture element: box with storage space + panel

    if (pVNEnv == NULL)
    {
        pVNEnv = new VN;
        pVNEnv->CreateEmpty();
        Array2D<float> A;
        A.w = 3;
        A.h = 18;
        A.Element = new float[A.w * A.h];
        CreateConvexTemplate18(A.Element);
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
        VNMClusters.n = 3;
        VNMClusters.Element = new RECOG::VN_::ModelCluster * [VNMClusters.n];
        VNMClusters.Element[0] = pVNEnv->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0);
        VNMClusters.Element[1] = pVNEnv->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0);
        Pair<int, int> iBetaInterval;
        iBetaInterval.a = 1;
        iBetaInterval.b = 3;
        VNMClusters.Element[2] = pVNEnv->AddModelCluster(2, RVLVN_CLUSTER_TYPE_XTORUS, R, t, 0.49f, 4, 4, iBetaInterval, pMem0);
        pVNEnv->AddOperation(3, 1, 1, 2, pMem0);
        pVNEnv->AddOperation(4, -1, 0, 3, pMem0);
        pVNEnv->SetOutput(4);
        pVNEnv->Create(pMem0);
        delete[] A.Element;
        delete[] CT.Element;
    }

    // Set environment 3D model parameters.

    UpdateStaticParams();

    ///

    // Robot.

    robot.pMem0 = pMem0;
    robot.Create(cfgFileNameIn);
 
    // Tool model.

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

    // Node buffer.

    nodeBuffMemCapacity = 10000;
    RVL_DELETE_ARRAY(nodeBuffMem);
    nodeBuffMem = new int[nodeBuffMemCapacity];

    // Solver.

    solver.Create(tool_sample_spheres.n* pVNEnv->featureArray.n, 6);

    // Constants.

    csMaxSurfaceContactAngle = cos(DEG2RAD * maxSurfaceContactAngle);
}

void DDManipulator::CreateParamList()
{
    paramList.m_pMem = pMem0;
    RVLPARAM_DATA* pParamData;
    paramList.Init();
    pParamData = paramList.AddParam("DDM.maxnSE3Points", RVLPARAM_TYPE_INT, &maxnSE3Points);
    pParamData = paramList.AddParam("DDM.tool.boundingSphere.x", RVLPARAM_TYPE_FLOAT, tool_bounding_sphere.c.Element);
    pParamData = paramList.AddParam("DDM.tool.boundingSphere.y", RVLPARAM_TYPE_FLOAT, tool_bounding_sphere.c.Element + 1);
    pParamData = paramList.AddParam("DDM.tool.boundingSphere.z", RVLPARAM_TYPE_FLOAT, tool_bounding_sphere.c.Element + 2);
    pParamData = paramList.AddParam("DDM.tool.boundingSphere.r", RVLPARAM_TYPE_FLOAT, &(tool_bounding_sphere.r));
    pParamData = paramList.AddParam("DDM.tool.PRTCP.x", RVLPARAM_TYPE_FLOAT, PRTCP_G);
    pParamData = paramList.AddParam("DDM.tool.PRTCP.y", RVLPARAM_TYPE_FLOAT, PRTCP_G + 1);
    pParamData = paramList.AddParam("DDM.tool.PRTCP.z", RVLPARAM_TYPE_FLOAT, PRTCP_G + 2);
    pParamData = paramList.AddParam("DDM.dd.openingDirection", RVLPARAM_TYPE_FLOAT, &dd_opening_direction);
    pParamData = paramList.AddParam("DDM.lock_T_G_DD", RVLPARAM_TYPE_BOOL, &bLock_T_G_DD);
    pParamData = paramList.AddParam("DDM.log", RVLPARAM_TYPE_BOOL, &bLog);
}

void DDManipulator::Clear()
{
    if (pVNEnv)
        delete pVNEnv;
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
    RVL_DELETE_ARRAY(feasibleTCPs.Element);
    RVL_DELETE_ARRAY(nodeBuffMem);
    if (pToolMesh)
        delete pToolMesh;
    if (pTimer)
        delete pTimer;
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
    RVLCOPYMX3X3(pose_Arot_S.R, VNMClusters.Element[0]->R);
    RVLCOPY3VECTOR(pose_Arot_S.t, VNMClusters.Element[0]->t);
    RVLCOMPTRANSF3D(pose_Arot_S.R, pose_Arot_S.t, pose_DD_A.R, pose_DD_A.t, pose_DD_S.R, pose_DD_S.t);
    pVNEnv->Descriptor(dVNEnv);
}

bool DDManipulator::Free(
    Pose3D *pPose_G_S,
    float* SDF)
{
    float* c_G;
    float c_S[3];
    int iActiveFeature;
    MOTION::Sphere* pSphere;
    float SDF_;
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

bool DDManipulator::Free(
    Pose3D* pPose_G_S_start,
    Pose3D* pPose_G_S_end)
{
    Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection;
    int iSampleSphere;
    int iPose;
    Pose3D* pPose_G_S;
    Vector3<float> c_S[2];
    MOTION::Sphere* pSphere;
    float* c_S_;
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

bool DDManipulator::LocalConstraints(
    Pose3D* pPose_G_S,
    float* SDF,
    Array<Pair<int, int>> &localConstraints,
    Vector3<float> *c_S_rot,
    Vector3<float>* c_S)
{
    float* c_G;
    Vector3<float>* pc_S = c_S;
    Vector3<float>* pc_S_rot = c_S_rot;
    int iActiveFeature;
    MOTION::Sphere* pSphere;
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
    Pose3D* pPose_G_S_start,
    Pose3D* pPose_G_S_end)
{
    float dt[3];
    RVLDIF3VECTORS(pPose_G_S_end->t, pPose_G_S_start->t, dt);
    float cost = (float)(tool_sample_spheres.n) * sqrt(RVLDOTPRODUCT3(dt, dt));
    //float dR[9];
    //RVLMXMUL3X3T1(pPose_G_S_end->R, pPose_G_S_start->R, dR);
    //float U[3];
    //float th;
    //GetAngleAxis(dR, U, th);
    //int iSphere;
    //float* c_G;
    //float V3Tmp[3];
    //for (iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++)
    //{
    //    c_G = tool_sample_spheres.Element[iSphere].c.Element;
    //    RVLCROSSPRODUCT3(U, c_G, V3Tmp);
    //    cost += sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
    //}
    return cost;
}

bool DDManipulator::FreePose(
    Pose3D* pPose_G_S_init,
    Array<Pair<int, int>> localConstraints,
    Vector3<float>* c_S_rot,
    Vector3<float>* c_S,
    Pose3D* pPose_G_S)
{
    // Parameters.

    int maxnIterations = 3;

    //

    int iConstraint, iFeature, iSphere;
    RECOG::VN_::Feature* pFeature;
    Pair<int, int>* pLocalConstraint;
    Vector3<float>* pc_S;
    Vector3<float>* pc_S_rot;
    float* A = new float[6 * localConstraints.n];
    float* b = new float[localConstraints.n];
    float* a_;
    float* a_t = A + 3;
    for (iConstraint = 0; iConstraint < localConstraints.n; iConstraint++, a_t += 6)
    {
        pLocalConstraint = localConstraints.Element + iConstraint;
        iFeature = pLocalConstraint->b;
        pFeature = pVNEnv->featureArray.Element + iFeature;
        RVLNEGVECT3(pFeature->N, a_t);
    }
    MOTION::Sphere* pSphere;
    float x0[6];
    float* t0 = x0 + 3;
    RVLNULL3VECTOR(x0);
    RVLNULL3VECTOR(t0);
    float x[6];
    float* t = x + 3;
    float th;
    float u[3];
    float dR[9], newR[9];
    Pose3D pose_G_S = *pPose_G_S_init;
    int it;
    float* c_G;
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
        th = sqrt(RVLDOTPRODUCT3(x, x));
        RVLSCALE3VECTOR2(x, th, u);
        AngleAxisToRot<float>(u, th, dR);
        RVLMXMUL3X3(dR, pose_G_S.R, newR);
        GetAngleAxis(newR, u, th);
        AngleAxisToRot<float>(u, th, pose_G_S.R);
        RVLSUM3VECTORS(pose_G_S.t, t, pose_G_S.t);
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
    Pose3D* pPose_G_0,
    float* SDF,
    float* q,
    bool bApproach)
{
    if (!robot.InvKinematics(*pPose_G_0, q))
        return false;
    Pose3D pose_G_S;
    RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pPose_G_0->R, pPose_G_0->t, pose_G_S.R, pose_G_S.t);
    if (!Free(&pose_G_S, SDF))
        return false;
    if (bDefaultToolModel)
    {
        float P1_S[3], P2_S[3];
        RVLTRANSF3(default_tool_P1_G, pose_G_S.R, pose_G_S.t, P1_S);
        RVLTRANSF3(default_tool_P2_G, pose_G_S.R, pose_G_S.t, P2_S);
        Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection =
            pVNEnv->VolumeCylinderIntersection(dVNEnv, P1_S, P2_S, tool_wrist_r);
        if (pIntersection->n > 0)
            return false;
    }
    if (bApproach)
    {
        Array<Pose3D> poses_G_0_via;
        Pose3D poses_G_0_via_Mem[2];
        poses_G_0_via.Element = poses_G_0_via_Mem;
        if (!ApproachPath(&pose_G_S, poses_G_0_via, SDF))
            return false;
    }
    return true;
}

void DDManipulator::Path(Pose3D* pPose_G_S_init)
{
    // Parameters.

    int nTargetSamples = 10000;
    float rSamplePos = 0.05f;
    float rSampleOrient = 45.0f;    // deg
    float rPos = 0.05f;
    float rOrient = 15.0f;  // deg
    float workSpaceExpansionCoeff = 0.5f;
    float rNeighborPos = 0.10f;
    float rNeighborOrient = 45.0f;  // deg
    Pose3D goal;
    RVLUNITMX3(goal.R);
    RVLSET3VECTOR(goal.t, 0.07f, 0.06f, 0.08f);
    Pose3D* pGoal = NULL;
    //Pose3D* pGoal = &goal;

    // Allocate arrays.

    float* SDF = new float[pVNEnv->NodeArray.n];

    // Feasible contact poses.

    std::vector<Pose3D> allFeasibleTCPs;
    allFeasibleTCPs.reserve(feasibleTCPs.n * ((int)ceil(dd_panel_params[0] / dd_contact_surface_params[0] + dd_panel_params[1] / dd_contact_surface_params[1]) - 1));
    Pose3D* pPose_G_DD;
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
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>* pGNode;

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
    QList<GRAPH::EdgePtr<MOTION::Edge>>* pEdgeList;
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
    MOTION::Node* pNode_;
    float cost;
    MOTION::Node* pNode;
    Array<int> neighbors;
    Pose3D samplePose;
    float move[3];
    float fTmp;
    float moveDist;
    float rotAxis[3];
    float moveRotAngle;
    float dR[9];
    MOTION::Node* pParent;
    MOTION::Edge* pEdge;
    GRAPH::EdgePtr<MOTION::Edge>* pEdgePtr;
    int iParent;
    int* piNodeFetch, * piNodePush;
    float dCost;
    int iSibling;
    MOTION::Node* pSibling;
    float rNeighborOrientRad = DEG2RAD * rNeighborOrient;
    int* piTmp;
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
        //float RDebug[9];
        //RVLMXMUL3X3T1(samplePose.R, grid.points.Element[iSE3Point].pose.R, RDebug);
        //float debug = RAD2DEG * acos(RVLROTDIFF(RDebug));
        //int iDebug1, iDebug2;
        //grid.Cell(grid.points.Element[iSE3Point].pose, i, iDebug1, iDebug2, iPosCell, iZ, iRoll);
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
            //for (i = 0; i < neighbors.n; i++)
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
                    //if (iNode == 13)
                    //    int debug = 0;
                    RVLMOTION_ADD_NODE_TO_TREE(nodes, iNode, pParent, iSibling, pSibling);
                    pEdgePtr = pEdgeList->pFirst;
                    while (pEdgePtr)
                    {
                        RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);
                        //if (iNode == 20 && iNode_ == 13)
                        //    int debug = 0;
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
                            //if (nodes.size() >= 20)
                            //    if (nodes[19].iParent == nodes[19].iSibling)
                            //        int debug = 0;
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

    //iNode = iNode_ = 0;
    //float minZ = nodes[iNode_].pose.pose.t[2];
    //float z;
    //for (iNode_ = 1; iNode_ < nodes.size(); iNode_++)
    //{
    //    z = nodes[iNode_].pose.pose.t[2];
    //    if (z < minZ)
    //    {
    //        minZ = z;
    //        iNode = iNode_;
    //    }
    //}

    iNode = -1;
    for(iNode_ = 0; iNode_ < nodes.size(); iNode_++)
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

    uchar red[] = { 255, 0, 0 };

    // Initial visualization.

    Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    std::vector<int> path;
    iNode_ = iNode;
    while(iNode_ >= 0)
    {
        path.push_back(iNode_);
        iNode_ = nodes[iNode_].iParent;
    }
    Array<float> doorStates;
    doorStates.n = 1;
    doorStates.Element = &dd_state_angle;
    Visualize(&nodes, &path, doorStates);

    // Visualize sample sphere path.

    //MOTION::Sphere* pSphere = tool_sample_spheres.Element;
    //pNode = nodes.data() + iNode;
    //float* c_S_;
    //Point visPt[2];
    //Pose3D *pPose_G_S = pPose_G_S_init;
    //for (iPose = 0; iPose < 2; iPose++)
    //{
    //    c_S_ = visPt[iPose].P;
    //    RVLTRANSF3(pSphere->c.Element, pPose_G_S->R, pPose_G_S->t, c_S_);
    //    pPose_G_S = &(pNode->pose.pose);
    //}
    //pVisualizer->DisplayLine(visPt, red);

    // Visualize TCPs.

    //Array<Point> visNodes;
    //visNodes.n = nodes.size();
    //visNodes.Element = new Point[visNodes.n];
    //float* PSrc, * PTgt;
    //for (iNode = 0; iNode < nodes.size(); iNode++)
    //{
    //    PSrc = nodes[iNode].pose.pose.t;
    //    PTgt = visNodes.Element[iNode].P;
    //    RVLCOPY3VECTOR(PSrc, PTgt);
    //}
    //uchar blue[] = { 0, 0, 255 };
    //pVisualizer->DisplayPointSet<float, Point>(visNodes, blue, 6);

    pVisualizer->Run();

    // Visualization of the results of VN::VolumeCylinderIntersection.

    //float sceneCenter[3];
    //BoxCenter<float>(&dd_static_box, sceneCenter);
    //float sceneSize[3];
    //BoxSize<float>(&dd_static_box, sceneSize[0], sceneSize[1], sceneSize[2]);
    //float sceneR = 0.6 * sqrt(RVLDOTPRODUCT3(sceneSize, sceneSize));
    //float sceneD = 2.0f * sceneR;
    //float U[3], V[3];
    //Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection;
    //Point visLineEndPt[2];
    //Array<Point> visIntersectionPts;
    //float* P1, *visP;
    //int j;
    //float s_[2];
    //for (int iVis = 0; iVis < 10; iVis++)
    //{
    //    RVLSET3VECTOR(U, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
    //    //RVLSET3VECTOR(U, -0.761679471, 0.00717963837, 0.7);
    //    //RVLSET3VECTOR(U, 1.00f, 0.01f, 0.01f);
    //    RVLNORM3(U, fTmp);
    //    RVLSCALE3VECTOR(U, sceneR, V);
    //    RVLDIF3VECTORS(sceneCenter, V, visLineEndPt[0].P);
    //    RVLSUM3VECTORS(sceneCenter, V, visLineEndPt[1].P);
    //    vtkSmartPointer<vtkActor> lineActor = pVisualizer->DisplayLine(visLineEndPt, red);
    //    vtkSmartPointer<vtkActor> intersectionPtsActor;
    //    pIntersection = pVNEnv->VolumeCylinderIntersection(dVNEnv, visLineEndPt[0].P, visLineEndPt[1].P, 0.0f);
    //    if (pIntersection->n > 0)
    //    {
    //        visIntersectionPts.Element = new Point[2 * pIntersection->n];
    //        visIntersectionPts.n = 0;
    //        for (i = 0; i < pIntersection->n; i++)
    //        {
    //            s_[0] = pIntersection->Element[i].a.s;
    //            s_[1] = pIntersection->Element[i].b.s;
    //            for (j = 0; j < 2; j++)
    //            {
    //                if (s_[j] >= 0.0f && s_[j] <= sceneD)
    //                {
    //                    RVLSCALE3VECTOR(U, s_[j], V);
    //                    P1 = visLineEndPt[0].P;
    //                    visP = visIntersectionPts.Element[visIntersectionPts.n++].P;
    //                    RVLSUM3VECTORS(P1, V, visP);
    //                }
    //            }
    //        }
    //        intersectionPtsActor = pVisualizer->DisplayPointSet<float, Point>(visIntersectionPts, red, 6);
    //        delete[] visIntersectionPts.Element;
    //    }
    //    pVisualizer->Run();
    //    pVisualizer->renderer->RemoveActor(lineActor);
    //    if(pIntersection->n > 0)
    //        pVisualizer->renderer->RemoveActor(intersectionPtsActor);
    //}

    // More visualizations of the tool.

    //for (int iVis = 1; iVis < 10; iVis++)
    //{
    //    for (i = 0; i < pVisualizationData->toolActors.size(); i++)
    //        pVisualizer->renderer->RemoveActor(pVisualizationData->toolActors[i]);
    //    Visualize(nodes[iVis].pose.pose);
    //    pVisualizer->Run();
    //}

    ///

    // Free memory.

    delete[] SDF;
    delete[] rndIdx.Element;
    //delete[] visNodes.Element;
}

// Move to Util.h.
#define RVLNORMANGLE(x) x = (x > PI ? x -= (2.0 * PI) : (x <= -PI ? x += (2.0 * PI) : x))
//

#define RVLMOTION_JOINT_SPACE_DIST(q, q_, dq, dist, i)\
{\
    RVLDIFVECTORS(q, q_, 6, dq, i);\
    for (i = 0; i < 6; i++)\
        RVLNORMANGLE(dq[i]);\
    RVLDOTPRODUCT(dq, dq, 6, dist, i);\
}

bool DDManipulator::Path2(
    float* qInit,
    Array<Pose3D> &poses_G_0,
    Array2D<float> &robotJoints)
{
#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
    if (pTimer)
        pTimer->Start();
#endif

    // Parameters.

    float rPos = 0.050f;    // m
    float rOrientDeg = 15.0f;   // deg
    int nTCPSamples = 100000;
    float startDoorState = RAD2DEG * dd_state_angle;    // deg
    float endDoorState = dd_opening_direction * 90.0f;     // deg
    int nStates = 17;
    //float endDoorState = 10.0f;     // deg
    //int nStates = 1;

    // Constants.

    float rOrient = DEG2RAD * rOrientDeg;
    float dDoorState = (nStates > 1 ? (endDoorState - startDoorState) / (float)(nStates - 1) : 0.0f);

    // Tile feasible contact poses.

    std::vector<Pose3D> allFeasibleTCPs;
    Box<float> TCPSpace;
    TileFeasibleToolContactPoses(&allFeasibleTCPs, TCPSpace);
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

    Graph<GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, MOTION::Edge, GRAPH::EdgePtr<MOTION::Edge>> graph;
    graph.NodeMem = new GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>[nTCPSamples];
    graph.NodeArray.Element = graph.NodeMem;
    GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>* pGNode;
    int iSE3Point;
	int iSE3Point_;			   
    int i;
    int iPosCell, iZ, iRoll;
    Array<MOTION::Node> nodes;
    int nTCPSamples_ = RVLMIN(nTCPSamples, allFeasibleTCPs.size());
    nodes.Element = new MOTION::Node[nTCPSamples_];
    nodes.n = 0;
    MOTION::Node* pNode;
    Array<int> rndIdx;
    rndIdx.n = allFeasibleTCPs.size();
    RandomIndices(rndIdx);
    Pose3D pose_G_DD;
    QList<GRAPH::EdgePtr<MOTION::Edge>>* pEdgeList;
    for (i = 0; i < rndIdx.n; i++)
    {
        //if (i % 1000 == 0)
        //    int debug = 0;
        iSE3Point = rndIdx.Element[i];
        pose_G_DD = allFeasibleTCPs[iSE3Point];
        // if(iSE3Point == 0)
        //     int debug = 0;
        // else
        {
            //if (pose_G_DD.t[0] < -0.05f)
            //    int debug = 0;
            iSE3Point_ = grid.Fetch(pose_G_DD, iPosCell, iZ, iRoll);
            if (iSE3Point_ >= 0)
                continue;
        }
        pNode = nodes.Element + nodes.n;
        pNode->pose.pose = pose_G_DD;
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
        if (nodes.n >= nTCPSamples_)
            break;
        grid.Add(pose_G_DD, true, pGNode->idx, iPosCell, iZ, iRoll);
    }
    delete[] rndIdx.Element;    
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
    MOTION::Edge* pEdge;
    MOTION::Node* pNode_;    
    //int debug = 0;
    for(iNode = 0; iNode < nodes.n; iNode++)
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
        //debug += TCPNeighbors.n;
        for (i = 0; i < TCPNeighbors.n; i++)
        {
            iNode_ = TCPNeighbors.Element[i];
            pNode_ = nodes.Element + iNode_;
            pEdge = ConnectNodes<GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, MOTION::Edge, GRAPH::EdgePtr<MOTION::Edge>>(pGNode, pNode_->pGNode, iNode, iNode_, pMem);
        }
    }

#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
    double graphCreationTime;
    if (pTimer)
    {
        pTimer->Stop();
        graphCreationTime = pTimer->GetTime();
        pTimer->Start();
        printf("Graph created in %lf s.\n", 0.001 * graphCreationTime);
    }
#endif

    /// Door openning path.

    SetEnvironmentState(startDoorState);

    // Free space planes.

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
    float* N_S;
    float P_DD[3];
    RVLSET3VECTOR(P_DD, 0.0f, 0.0f, -dd_sx);
    float P_S[3];
    RVLTRANSF3(P_DD, pose_DD_S.R, pose_DD_S.t, P_S);
    for (i = 0; i < 4; i++)
    {
        N_S = freeSpacePlanes_S[i].N;
        RVLMULMX3X3VECT(pose_DD_S.R, freeSpacePlanes_DD[i].N, N_S);
        freeSpacePlanes_S[i].d = RVLDOTPRODUCT3(N_S, P_S);
    }

    //

    float* SDF = new float[pVNEnv->NodeArray.n];
    Pose3D pose_G_0;
    float doorState = startDoorState;
    struct NodeData
    {
        float q[6];
        bool bFeasible;
        float cost;
        int* path;
    };
    NodeData* nodeDataMem = new NodeData[2 * nodes.n];
    NodeData* pNodeData = nodeDataMem;
    NodeData* pPrevNodeData = nodeDataMem + nodes.n;
    NodeData* pNodeDataTmp;
    NodeData* pData;
    NodeData* pData_;
    GRAPH::EdgePtr<Edge>* pEdgePtr;
    float dCost;
    float dq[6];
    float cost;
    float minCost = 0.0f;
    int iMinCostNeighbor;
    int* pathMem = new int[2 * nodes.n * nStates];
    pData = pNodeData;
    pData_ = pPrevNodeData;
    for (iNode = 0; iNode < nodes.n; iNode++, pData++, pData_++)
    {
        pData->path = pathMem + nStates * iNode;
        pData_->path = pathMem + nStates * (nodes.n + iNode);
    }
    int iState;
    int iPath;
    Array<float> doorStates;
    doorStates.n = nStates;
    doorStates.Element = new float[nStates];    // For visualization prupose.    
    float V3Tmp[3];
    Array<Pose3D> poses_G_0_via;
    Pose3D viaPtPosesMem[2];
    poses_G_0_via.Element = viaPtPosesMem;
    int* feasibleNodeMem = new int[2 * nodes.n];
    Array<int> feasibleNodes;
    feasibleNodes.Element = feasibleNodeMem;
    Array<int> feasibleNodesPrev;
    feasibleNodesPrev.Element = feasibleNodeMem + nodes.n;
    for (iNode = 0; iNode < nodes.n; iNode++)
        feasibleNodesPrev.Element[iNode] = iNode;
    int* piTmp;
    int iFeasibleNode;
    bool bPath;
    float posCost;
    float PRTCP_DD[3];
    bool bSelf, bAllNeighbors;
    bool* bFeasibilityTested = new bool[nodes.n];
    for(iState = 0; iState < nStates; iState++)
    {
        // Set the door state.

        doorState = startDoorState + (float)iState * dDoorState;
        if(iState > 0)
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

        // New method.
        
        //memset(bFeasibilityTested, 0, nodes.n * sizeof(bool));
        //for (iFeasibleNode = 0; iFeasibleNode < feasibleNodesPrev.n; iFeasibleNode++, pNode++, pData++)
        //{
        //    iNode = feasibleNodesPrev.Element[iFeasibleNode];
        //    pGNode = graph.NodeArray.Element + iNode;
        //    bSelf = false;
        //    bAllNeighbors = false;
        //    pEdgePtr = pGNode->EdgeList.pFirst;
        //    while (true)
        //    {
        //        if (!bSelf)
        //        {
        //            iNode_ = iNode;
        //            bSelf = true;
        //        }
        //        else if (pEdgePtr)
        //        {
        //            RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_)
        //        }
        //        if (!bFeasibilityTested[iNode_])
        //        {
        //            pNode_ = nodes.Element + iNode_;
        //            RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
        //            if (FeasiblePose(&pose_G_0, SDF, pData->q, iState == 0))
        //            {

        //            }
        //            bFeasibilityTested[iNode_] = true;
        //        }
        //        if(pEdgePtr)
        //            pEdgePtr = pEdgePtr->pNext;
        //        if (pEdgePtr == NULL)
        //            break;
        //    }
        //}

        // Old method.

        pNode = nodes.Element;
        pData = pNodeData;
        for (iNode = 0; iNode < nodes.n; iNode++, pNode++, pData++)
        {
            //if (iNode == 35387)
            //    int debug = 0;
            RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);

            // Only for debugging purpose!!!

            // if (pNode->iSE3Point == 0)
            // {
                 //robot.InvKinematics(pose_G_0);
                 //robot.FwdKinematics();
                 //std::vector<Node> debugNodes;
                 //Node debugNode;
                 //debugNode.pose.pose = pose_G_0;
                 //debugNodes.push_back(debugNode);
                 //std::vector<int> debugPath;
                 //debugPath.push_back(0);
                 //Array2D<float> debugRobotJoints;
                 //debugRobotJoints.w = robot.n;
                 //debugRobotJoints.h = 1;
                 //debugRobotJoints.Element = robot.q;
                 //doorStates.n = 1;
                 //Visualize(&debugNodes, &debugPath, doorStates, true, false, -1, &debugRobotJoints);
            // }

            //

            if (pData->bFeasible = FeasiblePose(&pose_G_0, SDF, pData->q, iState == 0))
                feasibleNodes.Element[feasibleNodes.n++] = iNode;

            //if (!robot.InvKinematics(pose_G_0, pData->q))
            //    continue;
            //RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pose_G_0.R, pose_G_0.t, pose_G_S.R, pose_G_S.t);
            //if (!Free(&pose_G_S, SDF))
            //    continue;
            //if (bDefaultToolModel)
            //{
            //    RVLTRANSF3(P1_G, pose_G_S.R, pose_G_S.t, P1_S);
            //    RVLTRANSF3(P2_G, pose_G_S.R, pose_G_S.t, P2_S);
            //    Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection =
            //        pVNEnv->VolumeCylinderIntersection(dVNEnv, P1_S, P2_S, tool_wrist_r);
            //    if (pIntersection->n > 0)
            //        continue;
            //}
            //if (iState == 0)
            //    if (!ApproachPath(&pose_G_S, poses_G_0_via, SDF))
            //        continue;
            //pData->bFeasible = true;
            //feasibleNodes.Element[feasibleNodes.n++] = iNode;
        }

        ///

        if (feasibleNodes.n == 0)
            break;

        // Find the optimal feasible path starting from the feasible robot configurations in the current state 
        // and back tracking to the start state.

        bPath = (iState == 0);
        for (iFeasibleNode = 0; iFeasibleNode < feasibleNodes.n; iFeasibleNode++, pNode++, pData++)
        {
            iNode = feasibleNodes.Element[iFeasibleNode];
            pNode = nodes.Element + iNode;
            pData = pNodeData + iNode;
            //if (iNode == 34822)
            //    int debug = 0;
            if (iState == 0)
            {
                pData->cost = 0.0f;                
                pData->path[0] = iNode;
            }
            else
            {
                pData_ = pPrevNodeData + iNode;
                iMinCostNeighbor = -1;
                if (pData_->bFeasible)
                {
                    RVLMOTION_JOINT_SPACE_DIST(pData->q, pData_->q, dq, dCost, i);
                    minCost = dCost + pData_->cost;
                    iMinCostNeighbor = iNode;
                }
                if (!bLock_T_G_DD)
                {
                    pGNode = pNode->pGNode;
                    pEdgePtr = pGNode->EdgeList.pFirst;
                    while (pEdgePtr)
                    {
                        RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iNode_);
                        pData_ = pPrevNodeData + iNode_;
                        if (pData_->bFeasible)
                        {
                            RVLMOTION_JOINT_SPACE_DIST(pData->q, pData_->q, dq, dCost, i);
                            cost = dCost + pData_->cost;
                            if (cost < minCost || iMinCostNeighbor < 0)
                            {
                                minCost = cost;
                                iMinCostNeighbor = iNode_;
                            }
                        }
                        pEdgePtr = pEdgePtr->pNext;
                    }
                }
                if (iMinCostNeighbor >= 0)
                {
                    pData_ = pPrevNodeData + iMinCostNeighbor;
                    RVLTRANSF3(PRTCP_G, pNode->pose.pose.R, pNode->pose.pose.t, PRTCP_DD);
                    if (dd_opening_direction < 0.0f)
                        PRTCP_DD[0] = -PRTCP_DD[0];
                    posCost = posCostMaxDist - RVLMIN(PRTCP_DD[0], PRTCP_DD[1]);
                    if (posCost < 0.0f)
                        posCost = 0.0f;
                    pData->cost = minCost + posCost;
                    memcpy(pData->path, pData_->path, iState * sizeof(int));
                    pData->path[iState] = iNode;
                    bPath = true;
                }
                else
                    pData->bFeasible = false;
            }
        }
        if (!bPath)
            break;
        pNodeDataTmp = pNodeData;
        pNodeData = pPrevNodeData;
        pPrevNodeData = pNodeDataTmp;
    }  
    pNodeData = pPrevNodeData;
    delete[] SDF;
    delete[] bFeasibilityTested;

    // Find the optimal path.

    int iBestEndNode = -1;
    if (iState >= nStates)
    {
        pData = pNodeData;
        minCost = 2.0f * 4.0f * PI * PI * 6.0f * (float)nStates;
        for (iNode = 0; iNode < nodes.n; iNode++, pData++)
        {
            if (!pData->bFeasible)
                continue;
            if (pData->cost < minCost)
            {
                minCost = pData->cost;
                iBestEndNode = iNode;
            }
        }
    }

    ///

    bPath = (iBestEndNode >= 0);
    if (bPath)
    {
        /// Copy the resulting path to the output format.

        pData = pPrevNodeData + iBestEndNode;
        std::vector<Node> pathNodes;
        std::vector<int> path_;
        Array<float> doorStates_;
        doorStates_.Element = new float[nStates + 3];
        robotJoints.w = robot.n;
        robotJoints.Element = new float[robotJoints.w * (nStates + 3)];
        Node node;
        float* q = robotJoints.Element;

        // Home pose.

        memcpy(robotJoints.Element, qInit, robot.n * sizeof(float));
        memcpy(robot.q, qInit, robot.n * sizeof(float));
        robot.FwdKinematics();
        Pose3D* pPose_n_0 = robot.link_pose + robot.n - 1;
        RVLCOMPTRANSF3D(pPose_n_0->R, pPose_n_0->t, robot.pose_TCP_6.R, robot.pose_TCP_6.t, pose_G_0.R, pose_G_0.t);
        node.pose.pose = pose_G_0;
        pathNodes.push_back(node);
        path_.push_back(0);
        doorStates_.Element[0] = startDoorState;

        // Approach path.

        iNode = pData->path[0];
        pNode = nodes.Element + iNode;
        SetEnvironmentState(startDoorState);
        RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pose_DD_S.R, pose_DD_S.t, pose_DD_0.R, pose_DD_0.t, V3Tmp);
        RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
        ApproachPath(&pose_G_0, poses_G_0_via, SDF);
        //poses_G_0_via.n = 0;    // Only for debugging purpose!!!
        int iState_;
        for (i = 0; i < poses_G_0_via.n; i++)
        {
            iState_ = i + 1;
            pose_G_0 = poses_G_0_via.Element[poses_G_0_via.n - i - 1];
            q = robotJoints.Element + robotJoints.w * iState_;
            robot.InvKinematics(pose_G_0, q);
            node.pose.pose = pose_G_0;
            pathNodes.push_back(node);
            path_.push_back(iState_);
            doorStates_.Element[iState_] = startDoorState;
        }
        doorStates_.n = nStates + poses_G_0_via.n + 1;
        robotJoints.h = doorStates_.n;

        // Door openning path.

        int nApproachStates = poses_G_0_via.n + 1;
        for (iState = 0; iState < nStates; iState++)
        {
            iState_ = iState + nApproachStates;
            iNode = pData->path[iState];
            pNode = nodes.Element + iNode;
            SetEnvironmentState(doorStates.Element[iState]);
            RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pose_DD_S.R, pose_DD_S.t, pose_DD_0.R, pose_DD_0.t, V3Tmp);
            RVLCOMPTRANSF3D(pose_DD_0.R, pose_DD_0.t, pNode->pose.pose.R, pNode->pose.pose.t, pose_G_0.R, pose_G_0.t);
            q = robotJoints.Element + robotJoints.w * iState_;
            robot.InvKinematics(pose_G_0, q);

            // Only for debugging purpose!!!

            //Pose3D pose_6_0;
            //pose_6_0 = robot.link_pose[5];
            //memcpy(robot.q, q, robot.n * sizeof(float));
            //robot.FwdKinematics();

            //

            node.pose.pose = pose_G_0;
            pathNodes.push_back(node);
            path_.push_back(iState_);
            doorStates_.Element[iState_] = doorStates.Element[iState];
        }

        poses_G_0.n = pathNodes.size();
        poses_G_0.Element = new Pose3D[poses_G_0.n];
        for (iNode = 0; iNode < poses_G_0.n; iNode++)
            poses_G_0.Element[iNode] = pathNodes[iNode].pose.pose;

        ///

#ifdef RVLDDMANIPULATOR_TIME_MESUREMENT
        double pathPalnningTime;
        if (pTimer)
        {
            pTimer->Stop();
            pathPalnningTime = pTimer->GetTime();
            printf("Path planned in %lf s.\n", 0.001 * pathPalnningTime);
        }
#endif

        // Visualization.

        if (pVisualizationData->bVisualize)
            Visualize(&pathNodes, &path_, doorStates_, true, false, -1, &robotJoints);
        //pVisualizationData->pVisualizer->Run();

        // Write results to a file.

        if (bLog && resultsFolder)
        {
            FILE* fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "path.txt").data(), "w");
            for (iState = 0; iState < nStates; iState++)
            {
                iNode = pData->path[iState];
                fprintf(fpLog, "%d ", iNode);
            }
            fprintf(fpLog, "\n");
            fclose(fpLog);
        }

        delete[] doorStates_.Element;
    }
    else
    {
        poses_G_0.Element = NULL;
        robotJoints.Element = NULL;
    }

    //

    delete[] nodes.Element;
    delete[] nodeDataMem;
    delete[] pathMem;
    delete[] doorStates.Element;
    delete[] feasibleNodeMem;

    return bPath;
}

void DDManipulator::TileFeasibleToolContactPoses(
    std::vector<Pose3D> *pAllFeasibleTCPs,
    Box<float>& TCPSpace)
{
    // Only for debugging purpose!!!

    //Box<float> bbox;
    //InitBoundingBox<float>(&bbox, feasibleTCPs.Element[0].t);
    //for (int i = 1; i < feasibleTCPs.n; i++)
    //    UpdateBoundingBox<float>(&bbox, feasibleTCPs.Element[i].t);
    //int debug_ = 0;

    // Feasible contact poses.

    pAllFeasibleTCPs->reserve(feasibleTCPs.n);
    Pose3D* pPose_G_DD;
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
    int iShift[2];
    float shift;
    bool bTileAxis[2];
    float origin[3];
    RVLNULL3VECTOR(origin);
    InitBoundingBox<float>(&TCPSpace, origin);
    float maxsx = dd_panel_params[0] - minDistanceToAxis;
    // int debug = 0;
    // int debug_nx = 0;
    // int debug_ny = 0;
    // int debug_nxy = 0;
    for (iTemplatePose = 0; iTemplatePose < feasibleTCPs.n; iTemplatePose++)
    {
        pPose_G_DD = feasibleTCPs.Element + iTemplatePose;
        //if (pPose_G_DD->R[6] > -csMaxSurfaceContactAngle)
        //    continue;
        RVLTRANSF3(PRTCP_G, pPose_G_DD->R, pPose_G_DD->t, PRTCP_DD);
        PRTCP_DD[0] *= dd_opening_direction;
        // if (PRTCP_DD[0] < visionTol || PRTCP_DD[1] < visionTol)
        //     continue;
        pose_G_DD = *pPose_G_DD;
        pAllFeasibleTCPs->push_back(pose_G_DD);
        UpdateBoundingBox<float>(&TCPSpace, pose_G_DD.t);
        pose_G_DD_template = pose_G_DD;
        for (iAxis = 0; iAxis < 2; iAxis++)
        {
            bTileAxis[iAxis] = (PRTCP_DD[iAxis] >= templateEnd[iAxis]);
            pose_G_DD.t[iAxis] = pose_G_DD_template.t[iAxis];
        }
        // debug++;
        // if (bTileAxis[0])
        //     debug_nx++;
        // if (bTileAxis[1])
        //     debug_ny++;
        // if (bTileAxis[0] && bTileAxis[1])
        //     debug_nxy++;
        iShift[1] = 0;
        s = PRTCP_DD[1];
        while (s <= dd_panel_params[1])
        {            
            iShift[0] = 0;
            s = PRTCP_DD[0];
            while (s <= maxsx)
            {
                if (iShift[0] + iShift[1] > 0)
                {
                    pAllFeasibleTCPs->push_back(pose_G_DD);
                    UpdateBoundingBox<float>(&TCPSpace, pose_G_DD.t);
                }
                if (!bTileAxis[0])
                    break;
                iShift[0]++;
                shift = (float)iShift[0] * dd_contact_surface_sampling_resolution;
                s = PRTCP_DD[0] + shift;
                pose_G_DD.t[0] = pose_G_DD_template.t[0] + dd_opening_direction * shift;
            }
            if (!bTileAxis[1])
                break;
            iShift[1]++;
            shift = (float)iShift[1] * dd_contact_surface_sampling_resolution;
            s = PRTCP_DD[1] + shift;
            pose_G_DD.t[1] = pose_G_DD_template.t[1] + shift;
        }
    }
}

bool DDManipulator::ApproachPath(
    Pose3D* pPose_G_S_contact,
    Array<Pose3D> &poses_G_0_via,
    float *SDF)
{
    // Second via point.

    float Z_G_S[3];
    RVLCOPYCOLMX3X3(pPose_G_S_contact->R, 2, Z_G_S);
    int iSphere;
    int iPlane;
    float s;
    float k;
    float e;
    float* N;
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
    Pose3D* pPose_G_0 = poses_G_0_via.Element;
    float V3Tmp[3];
    RVLCOMPTRANSF3DWITHINV(robot.pose_0_W.R, robot.pose_0_W.t, pPose_G_S_contact->R, pPose_G_S_contact->t, pPose_G_0->R, pPose_G_0->t, V3Tmp);
    Pose3D pose_G_S;
    if (mins < -1e-3)
    {
        float Z_G_0[3];
        RVLCOPYCOLMX3X3(pPose_G_0->R, 2, Z_G_0);
        RVLSCALE3VECTOR(Z_G_0, mins, V3Tmp);
        RVLSUM3VECTORS(pPose_G_0->t, V3Tmp, pPose_G_0->t);        
        if (!robot.InvKinematics(*pPose_G_0))
            return false;
        RVLCOMPTRANSF3D(robot.pose_0_W.R, robot.pose_0_W.t, pPose_G_0->R, pPose_G_0->t, pose_G_S.R, pose_G_S.t);
        if(!Free(pPose_G_0, SDF))
            return false;
        poses_G_0_via.n++;
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
        Pose3D* pPose_G_0_prev = poses_G_0_via.Element;
        *pPose_G_0 = *pPose_G_0_prev;
        RVLSCALE3VECTOR(Z_DD_0, e, V3Tmp);
        RVLDIF3VECTORS(pPose_G_0->t, V3Tmp, pPose_G_0->t);
        if (!robot.InvKinematics(*pPose_G_0))
            return false;
        poses_G_0_via.n++;
    }
    return true;
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
    UpdateDoorReferenceFrames();
    UpdateStaticParams();
}

void DDManipulator::UpdateStaticParams()
{
    // Boxes.

    dd_panel_box.minx = dd_rx - 0.5f * dd_sx;
    dd_panel_box.maxx = dd_rx + 0.5f * dd_sx;
    dd_panel_box.miny = dd_ry - 0.5f * dd_sy;
    dd_panel_box.maxy = dd_ry + 0.5f * dd_sy;
    dd_panel_box.minz = -0.5f * dd_sz;
    dd_panel_box.maxz = 0.5f * dd_sz;
    dd_static_box.minx = 0;
    dd_static_box.maxx = dd_panel_params[0] + 2.0f * (dd_moving_to_static_part_distance + dd_static_side_width);
    dd_static_box.miny = 0;
    dd_static_box.maxy = dd_panel_params[1] + 2.0f * (dd_moving_to_static_part_distance + dd_static_side_width);
    dd_static_box.minz = 0;
    dd_static_box.maxz = dd_static_depth;
    dd_storage_space_box.minx = dd_static_side_width;
    dd_storage_space_box.maxx = dd_static_side_width + dd_panel_params[0] + 2.0f * dd_moving_to_static_part_distance;
    dd_storage_space_box.miny = dd_static_side_width;
    dd_storage_space_box.maxy = dd_static_side_width + dd_panel_params[1] + 2.0f * dd_moving_to_static_part_distance;
    dd_storage_space_box.minz = 0;
    dd_storage_space_box.maxz = dd_static_depth;

    // VN model.

    Array<Vector3<float>> vertices;
    vertices.n = 24;
    vertices.Element = new Vector3<float>[vertices.n];
    float* vertices_ = new float[3 * vertices.n];
    BoxVertices<float>(&dd_panel_box, vertices_);
    BoxVertices<float>(&dd_static_box, vertices_ + 3 * 8);
    BoxVertices<float>(&dd_storage_space_box, vertices_ + 2 * 3 * 8);
    Array<RECOG::VN_::Correspondence5> assoc;
    assoc.n = 28;
    assoc.Element = new RECOG::VN_::Correspondence5[assoc.n];
    RECOG::VN_::Correspondence5* pAssoc = assoc.Element;
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
    for (iPt = 16; iPt < 20; iPt++)
    {
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 2;
        pAssoc->iBeta = 1;
        pAssoc++;
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 2;
        pAssoc->iBeta = 2;
        pAssoc++;
    }
    for (iPt = 20; iPt < 24; iPt++, pAssoc++)
    {
        pAssoc->iSPoint = iPt;
        pAssoc->iMCluster = 2;
        pAssoc->iBeta = 0;
    }
    float* PSrc = vertices_;
    float* PTgt;
    for (iPt = 0; iPt < 24; iPt++, PSrc += 3)
    {
        PTgt = vertices.Element[iPt].Element;
        RVLCOPY3VECTOR(PSrc, PTgt);
    }
    RVL_DELETE_ARRAY(dVNEnv);
    dVNEnv = new float[pVNEnv->featureArray.n];
    pVNEnv->Descriptor(vertices, assoc, dVNEnv);
    pVNEnv->SetFeatureOffsets(dVNEnv);
    float fTmp = dd_moving_to_static_part_distance + dd_static_side_width;
    float t[3];
    RVLNULL3VECTOR(t);
    RVLSET3VECTOR(t, fTmp, fTmp, dd_panel_params[2]);
    RECOG::VN_::Feature* pFeature;
    int iFeature;
    for (iFeature = 0; iFeature < 18; iFeature++)
    {
        pFeature = pVNEnv->featureArray.Element + iFeature;
        dVNEnv[iFeature] += RVLDOTPRODUCT3(pFeature->N, t);
    }
    delete[] vertices.Element;
    delete[] assoc.Element;

    RVLCOPYMX3X3(pose_A_F.R, VNMClusters.Element[0]->R);
    RVLCOPY3VECTOR(pose_A_F.t, VNMClusters.Element[0]->t);
    pVNEnv->Descriptor(dVNEnv);
}

void DDManipulator::UpdateDoorReferenceFrames()
{
    RVLNULLMX3X3(pose_A_F.R);
    if(dd_opening_direction > 0.0f)
    {
        RVLMXEL(pose_A_F.R, 3, 0, 1) = 1.0f;
        RVLMXEL(pose_A_F.R, 3, 1, 2) = -1.0f;
        RVLMXEL(pose_A_F.R, 3, 2, 0) = -1.0f;
        RVLSET3VECTOR(pose_A_F.t, dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[0] - dd_ry,
            dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[1],
            0.5f * dd_panel_params[2]);
        RVLSET3VECTOR(pose_DD_A.t, dd_rx-0.5f*dd_sx, dd_ry-0.5f*dd_sy, 0.5f*dd_sz);
    }
    else
    {
        RVLMXEL(pose_A_F.R, 3, 0, 1) = -1.0f;
        RVLMXEL(pose_A_F.R, 3, 1, 2) = -1.0f;
        RVLMXEL(pose_A_F.R, 3, 2, 0) = 1.0f;
        RVLSET3VECTOR(pose_A_F.t, dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[0] + dd_ry,
            dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[1],
            0.5f * dd_panel_params[2]);
        RVLSET3VECTOR(pose_DD_A.t, dd_rx + 0.5f * dd_sx, dd_ry - 0.5f * dd_sy, 0.5f * dd_sz);
    }
    RVLCOPYMX3X3T(pose_A_F.R, pose_DD_A.R);
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
    UpdateStaticPose();
}

void DDManipulator::UpdateStaticPose()
{
    RECOG::VN_::ModelCluster* pCluster;
    for (int iCluster = 1; iCluster < VNMClusters.n; iCluster++)
    {
        pCluster = VNMClusters.Element[iCluster];
        RVLCOPYMX3X3(pose_F_S.R, pCluster->R);
        RVLCOPY3VECTOR(pose_F_S.t, pCluster->t);
    }
}

void DDManipulator::LoadFeasibleToolContactPoses(std::string contactPointsFileName)
{
    cnpy::NpyArray npyData = cnpy::npy_load(contactPointsFileName);
    double* data = npyData.data<double>();
    feasibleTCPs.n = npyData.num_vals / 16;
    RVL_DELETE_ARRAY(feasibleTCPs.Element);
    feasibleTCPs.Element = new Pose3D[feasibleTCPs.n];
    int iPose;
    double* pData = data;
    Pose3D* pPose = feasibleTCPs.Element;
    double* srcRow;
    float* tgtRow;
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
    double* data = npyData.data<double>();
    tool_sample_spheres.n = npyData.num_vals / 4;
    RVL_DELETE_ARRAY(tool_sample_spheres.Element);
    tool_sample_spheres.Element = new MOTION::Sphere[tool_sample_spheres.n];
    double* pData = data;
    float* c;
    for (int iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++, pData += 4)
    {
        c = tool_sample_spheres.Element[iSphere].c.Element;
        RVLSCALE3VECTOR(pData, 0.001f, c);
        tool_sample_spheres.Element[iSphere].r = 0.001f * pData[3];
    }
    bDefaultToolModel = false;
}

void DDManipulator::InitVisualizer(Visualizer* pVisualizerIn)
{
    if (pVisualizationData == NULL)
        pVisualizationData = new MOTION::DisplayCallbackData;
    if (pVisualizerIn)
    {
        pVisualizationData->pVisualizer = pVisualizerIn;
        pVisualizationData->bOwnVisualizer = false;
    }
    else
    {
        pVisualizationData->pVisualizer = new Visualizer;
        pVisualizationData->bOwnVisualizer = true;
    }
    pVisualizationData->bVNEnv = false;
    pVisualizationData->bVisualize = false;
    pVisualizationData->paramList.m_pMem = pMem0;
	RVLPARAM_DATA* pParamData;
	pVisualizationData->paramList.Init();
    pParamData = pVisualizationData->paramList.AddParam("DDM.visualize", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualize));
    pVisualizationData->paramList.LoadParams((char *)(cfgFileName.data()));
}

void DDManipulator::Visualize(
    std::vector<MOTION::Node>* pNodes,
    std::vector<int>* pPath,
    Array<float> doorStates,
    bool bVisualizeStates,
    bool bVisualizeMotionPlanningTree,
    int iGoal,
    Array2D<float> *pRobotJoints)
{
    Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    uchar red[] = {255, 0, 0};
    uchar blue[] = {0, 0, 255};
    float* PSrc, * PTgt;

    // Display environment VN model.

    if(pVisualizationData->bVNEnv)
        pVNEnv->Display(pVisualizer, 0.02f, dVNEnv);   

    // Display static part of the furniture.

    Vector3<float> boxSize;
    Vector3<float> boxCenter;
    Pose3D pose_box_S;
    BoxSize<float>(&dd_static_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&dd_static_box, boxCenter.Element);
    RVLCOPYMX3X3(pose_F_S.R, pose_box_S.R);
    RVLTRANSF3(boxCenter.Element, pose_F_S.R, pose_F_S.t, pose_box_S.t);
    pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);
    BoxSize<float>(&dd_storage_space_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&dd_storage_space_box, boxCenter.Element);
    pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);

    /// Display door panel and robot.

    vtkSmartPointer<vtkActor> doorPanelActor;
    vtkSmartPointer<vtkActor> robotActor;
    if (bVisualizeStates)
    {
        // Visualize robot and door panel motion.

        int iState;
        int iToolActor;
        for (iState = 0; iState < doorStates.n; iState++)
        {
            SetEnvironmentState(doorStates.Element[iState]);
            doorPanelActor = VisualizeDoorPenel();
            VisualizeTool(pNodes->at(pPath->at(iState)).pose.pose, &(pVisualizationData->toolActors));
            robotActor = VisualizeRobot(pRobotJoints->Element + pRobotJoints->w * iState);
            pVisualizer->Run();
            pVisualizer->renderer->RemoveViewProp(doorPanelActor);
            for (iToolActor = 0; iToolActor < pVisualizationData->toolActors.size(); iToolActor++)
                pVisualizer->renderer->RemoveViewProp(pVisualizationData->toolActors[iToolActor]);
            pVisualizer->renderer->RemoveViewProp(robotActor);
        }
    }
    else
    {
        // Visualize door panel.

        VisualizeDoorPenel();

        // Visualize path.

        for (int iNode = 0; iNode < pPath->size(); iNode++)
            VisualizeTool(pNodes->at(pPath->at(iNode)).pose.pose, &(pVisualizationData->toolActors));
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
}

void DDManipulator::VisualizeTool(
    Pose3D pose_G_R,
    std::vector<vtkSmartPointer<vtkActor>>* pActors)
{
    Visualizer* pVisualizer = pVisualizationData->pVisualizer;
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
    toolSampleSphereCentersPC.n = tool_sample_spheres.n + 1;
    toolSampleSphereCentersPC.Element = new Point[toolSampleSphereCentersPC.n];
    float* PSrc, * PTgt;
    for (int iSphere = 0; iSphere < tool_sample_spheres.n; iSphere++)
    {
        PSrc = tool_sample_spheres.Element[iSphere].c.Element;
        PTgt = toolSampleSphereCentersPC.Element[iSphere].P;
        RVLTRANSF3(PSrc, pose_G_S.R, pose_G_S.t, PTgt);
    }
    PTgt = toolSampleSphereCentersPC.Element[tool_sample_spheres.n].P;
    Pose3D* pPose_G_S = &pose_G_S;
    RVLTRANSF3(PRTCP_G, pPose_G_S->R, pPose_G_S->t, PTgt);
    uchar red[] = { 255, 0, 0 };
    pActors->push_back(pVisualizer->DisplayPointSet<float, Point>(toolSampleSphereCentersPC, red, 6));
    delete[] toolSampleSphereCentersPC.Element;
    if (pToolMesh)
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

vtkSmartPointer<vtkActor> DDManipulator::VisualizeRobot(float* q)
{
    Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    memcpy(robot.q, q, robot.n * sizeof(float));
    robot.FwdKinematics();
    Array<Point> vertices;
    vertices.n = robot.n + 1;
    vertices.Element = new Point[vertices.n];
    RVLCOPY3VECTOR(robot.pose_0_W.t, vertices.Element[0].P);
    Array<Pair<int, int>> lines;
    lines.n = robot.n;
    lines.Element = new Pair<int, int>[lines.n];
    int i;
    float* PSrc, *PTgt;
    for (i = 0; i < robot.n; i++)
    {
        PSrc = robot.link_pose[i].t;
        PTgt = vertices.Element[i + 1].P;
        RVLTRANSF3(PSrc, robot.pose_0_W.R, robot.pose_0_W.t, PTgt);
        lines.Element[i].a = i;
        lines.Element[i].b = i+1;
    }
    uchar red[] = { 255, 0, 0 };
    vtkSmartPointer<vtkActor> actor = pVisualizer->DisplayLines(vertices, lines, red, 2.0f);

    delete[] vertices.Element;
    delete[] lines.Element;

    return actor;
}

vtkSmartPointer<vtkActor> DDManipulator::VisualizeDoorPenel()
{
    Visualizer* pVisualizer = pVisualizationData->pVisualizer;
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
}

Robot::~Robot()
{
    Clear();
}

void Robot::Create(char* cfgFileNameIn)
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
    minq[4] = 0.0f;
    maxq[4] = DEG2RAD * 135.0f;
}

void Robot::CreateParamList()
{
    paramList.m_pMem = pMem0;
    RVLPARAM_DATA* pParamData;
    paramList.Init();
    pParamData = paramList.AddParam("Robot.t_TCP_6.x", RVLPARAM_TYPE_FLOAT, pose_TCP_6.t);
    pParamData = paramList.AddParam("Robot.t_TCP_6.y", RVLPARAM_TYPE_FLOAT, pose_TCP_6.t + 1);
    pParamData = paramList.AddParam("Robot.t_TCP_6.z", RVLPARAM_TYPE_FLOAT, pose_TCP_6.t + 2);
    pParamData = paramList.AddParam("Robot.rotz_TCP_6", RVLPARAM_TYPE_FLOAT, &rotz_TCP_6);
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
}

void Robot::FwdKinematics()
{
    Pose3D dPose;
    int i;
    float cq, sq;
    float* R, * t, *R_, *t_;
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
            R_ = link_pose[i-1].R;
            t_ = link_pose[i-1].t;
            RVLCOMPTRANSF3D(R_, t_, dPose.R, dPose.t, R, t);
        }
    }
}

void Robot::FwdKinematics(
    int i,
    Pose3D* pdPose)
{
    float cq, sq;
    FwdKinematicsRot(i, pdPose->R, cq, sq);
    RVLSET3VECTOR(pdPose->t, a[i] * cq, a[i] * sq, d[i]);
}

void Robot::FwdKinematicsRot(
    int i,
    float* R,
    float& cq,
    float& sq)
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
    float* qOut)
{
    if (!InvKinematics1E56(toolPose, qOut))
        return false;

    float* q_ = (qOut ? qOut : q);
    Pose3D* pPose_6_0 = link_pose + 5;    
    Pose3D* pPose_1_0 = link_pose;
    RVLSET3VECTOR(pPose_1_0->t, 0.0f, 0.0f, d[0]);
    Pose3D pose_6_1;
    float V3Tmp[3];
    RVLCOMPTRANSF3DWITHINV(pPose_1_0->R, pPose_1_0->t, pPose_6_0->R, pPose_6_0->t, pose_6_1.R, pose_6_1.t, V3Tmp);
    Pose3D pose_5_4;
    FwdKinematics(4, &pose_5_4);
    Pose3D* pPose_5_0 = link_pose + 4;
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
    //q_[3] -= (q[1] + q[2]);
    q_[3] -= (q_[1] + q_[2] + PI);
    //q_[3] = -(q[1] + q[2] + PI);
    return true;
}

bool Robot::InvKinematics1E56(
    Pose3D pose_G_0,
    float* qOut)
{    
    float* q_ = (qOut ? qOut : q);
    Pose3D* pPose_6_0 = link_pose + 5;
    RVLCOMPTRANSF3D(pose_G_0.R, pose_G_0.t, pose_6_G.R, pose_6_G.t, pPose_6_0->R, pPose_6_0->t);
    float Z_6_0[3];
    RVLCOPYCOLMX3X3(pPose_6_0->R, 2, Z_6_0);
    float V3Tmp[3];
    RVLSCALE3VECTOR(Z_6_0, d[5], V3Tmp);
    Pose3D* pPose_5_0 = link_pose + 4;
    RVLDIF3VECTORS(pPose_6_0->t, V3Tmp, pPose_5_0->t);
    float ps = atan2(pPose_5_0->t[1], pPose_5_0->t[0]);
    float t_5_0_xy_2 = pPose_5_0->t[0] * pPose_5_0->t[0] + pPose_5_0->t[1] * pPose_5_0->t[1];
    float fTmp = sqrt(t_5_0_xy_2);
    if (fTmp < d[3])
        return false;
    float r2 = t_5_0_xy_2 + pPose_5_0->t[2] * pPose_5_0->t[2];
    a23_2 = r2 - d4_2;
    //if (a23_2 > maxa23_2)
    //    return false;
    float ph = asin(d[3] / fTmp);
    q_[0] = ps + ph;
    float cq = cos(q_[0]);
    float sq = sin(q_[0]);
    float* R_1_0 = link_pose[0].R;
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

    //memcpy(q + 3, q_ + 3, 3 * sizeof(float));
    //float R_4_3[9];
    //FwdKinematicsRot(3, R_4_3, cq, sq);
    //float R_5_4[9];
    //FwdKinematicsRot(4, R_5_4, cq, sq);
    //float R_5_3[9];
    //RVLMXMUL3X3(R_4_3, R_5_4, R_5_3);
    //float R_3_E[9];
    //RVLNULLMX3X3(R_3_E);
    //RVLMXEL(R_3_E, 3, 0, 0) = 1.0f;
    //RVLMXEL(R_3_E, 3, 1, 2) = 1.0f;
    //RVLMXEL(R_3_E, 3, 2, 1) = -1.0f;
    //float R_5_E[9];
    //RVLMXMUL3X3(R_3_E, R_5_3, R_5_E);
    //float R_6_5[9];
    //FwdKinematicsRot(5, R_6_5, cq, sq);
    //float R_6_E_[9];
    //RVLMXMUL3X3(R_5_E, R_6_5, R_6_E_);
    //float R_err[9];
    //RVLMXMUL3X3T1(R_6_E_, R_6_E, R_err);
    //float debug = RVLROTDIFF(R_err);

    //

    return true;
}

void Robot::InvKinematicsApprox23(float* qOut)
{
    float* q_ = (qOut ? qOut : q);
    q_[2] = acos((k1 - a23_2) / k2);
    float a23 = sqrt(a23_2);
    q_[1] = asin(link_pose[4].t[2] / a23);
}
