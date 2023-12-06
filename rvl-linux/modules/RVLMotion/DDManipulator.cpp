#include "RVLCore2.h"
#include "RVLVTK.h"
#include "vtkSTLReader.h"
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
#include "vtkQuaternion.h"

using namespace RVL;

DDManipulator::DDManipulator()
{

    // Cfg params
    gripperModelFileName = NULL;
    // gripperPoseSaveFileName = NULL;
    // gripperModelFileName = "/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.ply";
    // gripperModelFileName = "/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/ros_point_cloud.ply";
    // Door model parameters.

    dd_contact_surface_params[0] = dd_contact_surface_params[1] = 0.1f;
    RVLSET3VECTOR(dd_panel_params, 0.3, 0.5, 0.018);
    dd_moving_to_static_part_distance = 0.005;
    dd_static_side_width = 0.018;
    dd_static_depth = 0.3;
    dd_axis_distance = 0.01;
    dd_contact_surface_sampling_resolution = 0.005;

    // Door pose.

    RVLUNITMX3(pose_W_S.R);
    RVLNULL3VECTOR(pose_W_S.t);

    // Environment VN model.

    pVNEnv = NULL;
    VNMClusters.n = 0;
    VNMClusters.Element = NULL;
    dVNEnv = NULL;

    // Tool model.

    RVLSET3VECTOR(tool_contact_surface_params[0].Element, 0.0, 0.01, 0.0);
    RVLSET3VECTOR(tool_contact_surface_params[1].Element, 0.0, 0.01, -0.02);
    RVLSET3VECTOR(tool_finger_size.Element, 0.02, 0.02, 0.06);
    tool_finger_distance = 0.06;
    RVLSET3VECTOR(tool_palm_size.Element, 0.1, 0.02, 0.02);
    tool_sample_spheres.n = 0;
    tool_sample_spheres.Element = NULL;

    // Constants.

    RVLNULLMX3X3(pose_A_W.R);
    RVLMXEL(pose_A_W.R, 3, 2, 0) = -1.0f;
    RVLMXEL(pose_A_W.R, 3, 0, 1) = 1.0f;
    RVLMXEL(pose_A_W.R, 3, 1, 2) = -1.0f;
    RVLCOPYMX3X3T(pose_A_W.R, pose_DD_A.R);
    RVLNULL3VECTOR(pose_Arot_A.t);

    // Path planning.

    maxnSE3Points = 20000;

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

    // Cfg params
    // RVL_DELETE_ARRAY(gripperModelFileName)
}

void DDManipulator::Create(char* cfgFileNameIn)
{
    // Pose of the contact surface with respect to A.

    RVLSET3VECTOR(pose_DD_A.t, -0.5 * dd_panel_params[2], dd_axis_distance - dd_moving_to_static_part_distance - dd_panel_params[0], 0.5f * dd_panel_params[1]);

    // Create environment 3D model.

    dd_panel_box.minx = -0.5 * dd_panel_params[2];
    dd_panel_box.maxx = 0.5 * dd_panel_params[2];
    dd_panel_box.miny = dd_axis_distance - dd_moving_to_static_part_distance - dd_panel_params[0];
    dd_panel_box.maxy = dd_axis_distance - dd_moving_to_static_part_distance;
    dd_panel_box.minz = -0.5f * dd_panel_params[1];
    dd_panel_box.maxz = 0.5f * dd_panel_params[1];
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

    // Create environment VN model.

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

        RVLSET3VECTOR(pose_A_W.t, dd_static_side_width + 2.0f * dd_moving_to_static_part_distance + dd_panel_params[0] - dd_axis_distance,
            dd_static_side_width + dd_moving_to_static_part_distance + 0.5f * dd_panel_params[1],
            0.5f * dd_panel_params[2]);
        RVLCOPYMX3X3(pose_A_W.R, VNMClusters.Element[0]->R);
        RVLCOPY3VECTOR(pose_A_W.t, VNMClusters.Element[0]->t);
        pVNEnv->Descriptor(dVNEnv);

        delete[] A.Element;
        delete[] CT.Element;
    }

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

    // Node buffer.

    nodeBuffMemCapacity = 10000;
    RVL_DELETE_ARRAY(nodeBuffMem);
    nodeBuffMem = new int[nodeBuffMemCapacity];

    // Solver.

    solver.Create(tool_sample_spheres.n* pVNEnv->featureArray.n, 6);

    // Load paramters from a configuration file.

    CreateParamList();
    paramList.LoadParams(cfgFileNameIn);
}

void DDManipulator::CreateParamList()
{
    paramList.m_pMem = pMem0;
    RVLPARAM_DATA* pParamData;
    paramList.Init();
    pParamData = paramList.AddParam("DDM.maxnSE3Points", RVLPARAM_TYPE_INT, &maxnSE3Points);
    pParamData = paramList.AddParam("DDM.UseDefaultGripper", RVLPARAM_TYPE_BOOL, &useDefaultGripper);
    pParamData = paramList.AddParam("DDM.GripperModelFileName", RVLPARAM_TYPE_STRING, &gripperModelFileName);
    // pParamData = paramList.AddParam("DDM.GripperPoseSaveFileName", RVLPARAM_TYPE_STRING, &gripperPoseSaveFileName);
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
}

void DDManipulator::SetEnvironmentState(float state)
{    
    dd_state_angle = DEG2RAD * state;
    float cs = cos(dd_state_angle);
    float sn = sin(dd_state_angle);
    RVLROTZ(cs, sn, pose_Arot_A.R);    
    RVLCOMPTRANSF3D(pose_A_W.R, pose_A_W.t, pose_Arot_A.R, pose_Arot_A.t, VNMClusters.Element[0]->R, VNMClusters.Element[0]->t);
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
    RECOG::VN_::Feature* pFeature;
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

// void DDManipulator::Path(Pose3D* pPose_G_S_init)
Pose3D DDManipulator::Path(Pose3D* pPose_G_S_init)
{
    // Parameters.

    float kTemplateEndTol = 0.1f;
    int nTargetSamples = 10000;
    float rSamplePos = 0.05f;
    float rSampleOrient = 45.0f;    // deg
    float rPos = 0.05f;
    float rOrient = 15.0f;  // deg
    float workSpaceExpansionCoeff = 0.5f;
    float rNeighborPos = 0.10f;
    float rNeighborOrient = 45.0f;  // deg
    float maxSurfaceContactAngle = 45.0f;   // deg
    float visionTol = 0.01f;    // m
    Pose3D goal;
    RVLUNITMX3(goal.R);
    RVLSET3VECTOR(goal.t, 0.07f, 0.06f, 0.08f);
    Pose3D* pGoal = NULL;
    //Pose3D* pGoal = &goal;

    // Constants;

    float csMaxSurfaceContactAngle = cos(DEG2RAD * maxSurfaceContactAngle);

    // Allocate arrays.

    float* SDF = new float[pVNEnv->NodeArray.n];

    // Feasible contact poses.

    std::vector<Pose3D> allFeasibleTCPs;
    allFeasibleTCPs.reserve(feasibleTCPs.n * ((int)ceil(dd_panel_params[0] / dd_contact_surface_params[0] + dd_panel_params[1] / dd_contact_surface_params[1]) - 1));
    Pose3D* pPose_G_DD;
    int iTemplatePose;
    float TCP_DD[3];
    float templateEndTol = kTemplateEndTol * dd_contact_surface_sampling_resolution;
    float templateEnd[2];
    templateEnd[0] = dd_contact_surface_params[0] - templateEndTol;
    templateEnd[1] = dd_contact_surface_params[1] - templateEndTol;
    int iAxis;
    float s;
    Pose3D pose_A_S;
    RVLCOMPTRANSF3D(pose_W_S.R, pose_W_S.t, pose_A_W.R, pose_A_W.t, pose_A_S.R, pose_A_S.t);
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
        // if (pPose_G_DD->R[6] > -csMaxSurfaceContactAngle)
        //     continue;
        RVLMOTION_TCP(pPose_G_DD, half_tool_finger_distance, TCP_DD);
        if (TCP_DD[0] < visionTol || TCP_DD[1] < visionTol)
            continue;
        pose_G_DD = *pPose_G_DD;
        allFeasibleTCPs.push_back(pose_G_DD);
        pose_G_DD_template = pose_G_DD;
        for (iAxis = 0; iAxis < 2; iAxis++)
        {
            if (TCP_DD[iAxis] >= templateEnd[iAxis])
            {
                iShift = 1;
                shift = (float)iShift * dd_contact_surface_sampling_resolution;
                s = TCP_DD[iAxis] + shift;
                while (s <= dd_panel_params[iAxis])
                {
                    pose_G_DD.t[iAxis] = pose_G_DD_template.t[iAxis] + shift;
                    allFeasibleTCPs.push_back(pose_G_DD);
                    iShift++;
                    shift = (float)iShift * dd_contact_surface_sampling_resolution;
                    s = TCP_DD[iAxis] + shift;
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
    Visualize(&nodes, &path, 0);

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

    pNode = nodes.data() + nodes.size(); // Take the last pose (by the door)
    Pose3D pose_G_S = pNode->pose.pose;
    return pose_G_S;
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
}

void DDManipulator::Visualize(
    std::vector<MOTION::Node>* pNodes,
    std::vector<int>* pPath,
    int iGoal)
{
    Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    uchar red[] = {255, 0, 0};
    uchar blue[] = {0, 0, 255};
    float* PSrc, * PTgt;

    // Display environment VN model.

    if(pVisualizationData->bVNEnv)
        pVNEnv->Display(pVisualizer, 0.02f, dVNEnv);   

    // Display environment box model.

    Pose3D pose_Arot_W;
    RVLCOMPTRANSF3D(pose_A_W.R, pose_A_W.t, pose_Arot_A.R, pose_Arot_A.t, pose_Arot_W.R, pose_Arot_W.t);
    Pose3D pose_Arot_S;
    Vector3<float> boxSize;
    Vector3<float> boxCenter;
    BoxSize<float>(&dd_panel_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&dd_panel_box, boxCenter.Element);
    RVLCOMPTRANSF3D(pose_W_S.R, pose_W_S.t, pose_Arot_W.R, pose_Arot_W.t, pose_Arot_S.R, pose_Arot_S.t);
    Pose3D pose_box_S;
    RVLCOPYMX3X3(pose_Arot_S.R, pose_box_S.R);
    RVLTRANSF3(boxCenter.Element, pose_Arot_S.R, pose_Arot_S.t, pose_box_S.t);
    pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);
    BoxSize<float>(&dd_static_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&dd_static_box, boxCenter.Element);
    RVLCOPYMX3X3(pose_W_S.R, pose_box_S.R);
    RVLTRANSF3(boxCenter.Element, pose_W_S.R, pose_W_S.t, pose_box_S.t);
    pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);
    BoxSize<float>(&dd_storage_space_box, boxSize.Element[0], boxSize.Element[1], boxSize.Element[2]);
    BoxCenter<float>(&dd_storage_space_box, boxCenter.Element);
    pVisualizer->DisplayBox(boxSize.Element[0], boxSize.Element[1], boxSize.Element[2], &pose_box_S, 0.0, 128.0, 0.0);

    // Visualize path.

    for(int iNode = 0; iNode < pPath->size(); iNode++)
        VisualizeTool(pNodes->at(pPath->at(iNode)).pose.pose, &(pVisualizationData->toolActors));

    // Visualize motion planning tree.

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
    Pose3D pose_G_S,
    // bool useDefaultGripper,
    // std::string gripperModelFileName,
    std::vector<vtkSmartPointer<vtkActor>> *pActors)
{
    if (useDefaultGripper)
    {
        Visualizer* pVisualizer = pVisualizationData->pVisualizer;
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
        Pose3D *pPose_G_S = &pose_G_S;
        RVLMOTION_TCP(pPose_G_S, half_tool_finger_distance, PTgt);
        uchar red[] = {255, 0, 0};
        pActors->push_back(pVisualizer->DisplayPointSet<float, Point>(toolSampleSphereCentersPC, red, 6));
        delete[] toolSampleSphereCentersPC.Element;
    }
    else
    {
        // RotMat [9] to RotMat[3][3]
        double R3x3[3][3];
        int index = 0;
        for(int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                R3x3[i][j] = (double) pose_G_S.R[index];
                ++index;
            }
        }

        double q[4];
        vtkMath::Matrix3x3ToQuaternion(R3x3, q);


        double T[16];
        RVLHTRANSFMX(pose_G_S.R, pose_G_S.t, T);

        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        
        transform->SetMatrix(T);
        // transform->Translate(pose_G_S.t);
        // transform->RotateWXYZ(q[0], q[1], q[2], q[3]);
        // transform->Update();

        vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
        reader->SetFileName(gripperModelFileName);
        reader->Update();
        vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();

        vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        transformFilter->SetInputData(polyData);
        transformFilter->SetTransform(transform);
        transformFilter->Update();
        vtkSmartPointer<vtkPolyData> polyDataTransformed = transformFilter->GetOutput();

        pVisualizationData->pVisualizer->AddMesh(polyDataTransformed);
    }

}

void DDManipulator::SetVisualizeVNEnvironmentModel()
{
    pVisualizationData->bVNEnv = true;
}