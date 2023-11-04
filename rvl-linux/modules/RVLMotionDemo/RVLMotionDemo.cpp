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
#include "RRT.h"
#include "DDManipulator.h"
#include "cnpy.h"

using namespace RVL;

void CreateParamList(
    CRVLParameterList* pParamList,
    CRVLMem* pMem,
    char** pFeasibleToolContactPosesFileName,
    float &dd_state_angle_deg)
{
    pParamList->m_pMem = pMem;
    RVLPARAM_DATA* pParamData;
    pParamList->Init();

    pParamData = pParamList->AddParam("FeasibleToolContactPosesFileName", RVLPARAM_TYPE_STRING, pFeasibleToolContactPosesFileName);
    pParamData = pParamList->AddParam("DoorSateAngle(deg)", RVLPARAM_TYPE_FLOAT, &dd_state_angle_deg);
}

void TestSolver()
{
    Solver solver;
    int m = 10;
    int n = 6;
    int m_;
    solver.Create(m, n);
    float* A = new float[m * n];
    memset(A, 0, m * n * sizeof(float));
    float* b = new float[m];
    float* x0 = new float[n];
    float* x = new float[n];
    float* e = new float[m];
    bool bLoadProblemFromFile = false;
    bool bProblemInNPYFile = false;
    bool bSaveProbleToFile = false;
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
    std::string problemDataDirectoryName = "D:\\Cupec\\Documents\\Google_Disk\\2023\\Projects\\COSPER\\Research\\WP3\\DDMan\\";
    printf("Test started...  \n");
    //system("pause");
    bool bVerbose = true;
    bool bFeasibleSolution;
    for (int it = 0; it < 10000; it++)
    {
        if (bVerbose)
            printf("Test %d\n", it);
        if (bLoadProblemFromFile)
        {
            if (bProblemInNPYFile)
            {
                cnpy::NpyArray npyData = cnpy::npy_load(problemDataDirectoryName + "data.npy");
                double* data = npyData.data<double>();
                double* pData = data;
                for (j = 0; j < m_; j++)
                {
                    for (i = 0; i < n; i++)
                        A[j * n + i] = *(pData++);
                    b[j] = *(pData++);
                }
            }
            else
            {
                FILE* fp = fopen((problemDataDirectoryName + "data.dat").data(), "rb");
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
                FILE* fp = fopen((problemDataDirectoryName + "data.dat").data(), "wb");
                fwrite(A, sizeof(float), m_ * n, fp);
                fwrite(b, sizeof(float), m_, fp);
                fclose(fp);
            }
        }
        memset(x0, 0, n * sizeof(float));
        bFeasibleSolution = solver.FeasibleSolution(A, b, m, x0, x);
        if (bVerbose)
        {
            if (bFeasibleSolution)
            {
                float* a;
                RVLMULMXVECT(A, x, m, n, e, i, j, a);
                RVLDIFVECTORS(e, b, m, e, i);
                printf("x:\n");
                for (i = 0; i < n; i++)
                    printf("%f\n", x[i]);
                float maxe = -1.0f;
                for (j = 0; j < m; j++)
                    if (e[j] > maxe)
                        maxe = e[j];
                printf("maxe=%f\n\n", maxe);
                if (maxe > 1e-6)
                    system("pause");
            }
            else
                printf("No feasible solution exists.\n\n");
            //system("pause");
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
    uchar red[] = { 255, 0, 0 };
    uchar blue[] = { 0, 0, 255 };

    // Display environment VN model.

    //pVNEnv->Display(&visualizer, 0.01f, dVNEnv);
    //visualizer.Run();

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
    char* cfgFileName)
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
    VN* pVNEnv = manipulator.pVNEnv;
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
    Array<RECOG::VN_::ModelCluster*> VNMClusters;
    VNMClusters.n = 2;
    VNMClusters.Element = new RECOG::VN_::ModelCluster * [VNMClusters.n];
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
    float* vertices_ = new float[3 * vertices.n];
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
    float* PSrc = vertices_;
    float* PTgt;
    for (iPt = 0; iPt < vertices.n; iPt++, PSrc += 3)
    {
        PTgt = vertices.Element[iPt].Element;
        RVLCOPY3VECTOR(PSrc, PTgt);
    }
    RVL_DELETE_ARRAY(manipulator.dVNEnv);
    manipulator.dVNEnv = new float[pVNEnv->featureArray.n];
    float* dVNEnv = manipulator.dVNEnv;
    pVNEnv->Descriptor(vertices, assoc, dVNEnv);
    pVNEnv->SetFeatureOffsets(dVNEnv);

    delete[] A.Element;
    delete[] CT.Element;

    // Create tool.

    Array<MOTION::Sphere>* pToolSampleSpheres = &(manipulator.tool_sample_spheres);
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
        //RVLSET3VECTOR(rotAxis, 0.0f, 0.0f, 1.0f);
        float fTmp;
        RVLRNDUNIT3VECTOR(rotAxis, fTmp);
        float rotAngle = 30.0f * DEG2RAD;
        AngleAxisToRot<float>(rotAxis, rotAngle, pose_G_S_init.R);
        float tRange = 0.01f;
        RVLSET3VECTOR(pose_G_S_init.t, tRange * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f), tRange * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f), tRange * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f));
        //RVLUNITMX3(pose_G_S_init.R);
        //RVLNULL3VECTOR(pose_G_S_init.t);

        // Visualization.

        VisualizeTestLocalFreePose(&visualizer, wallBox, holeBox, tool_sample_sphere_r, pose_G_S_init);

        // Find collision-free pose in the local vicinity of the current pose.

        Pose3D pose_G_S;
        float* SDF = new float[pVNEnv->featureArray.n];
        Array<Pair<int, int>> localConstraints;
        localConstraints.Element = new Pair<int, int>[pVNEnv->featureArray.n * pToolSampleSpheres->n];
        Vector3<float>* c_S_rot = new Vector3<float>[manipulator.tool_sample_spheres.n];
        Vector3<float>* c_S = new Vector3<float>[manipulator.tool_sample_spheres.n];
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

int main(int argc, char** argv)
{
    // Create memory storage.

    CRVLMem mem0;	// permanent memory
    mem0.Create(1000000000);
    CRVLMem mem;	// cycle memory
    mem.Create(1000000000);

    // Read parameters from a configuration file.

    char cfgSelectionFileName[] = "RVLMotionDemo.cfg";
    char* cfgFileName = ReadConfigurationFile(cfgSelectionFileName);
    if (cfgFileName == NULL)
        return 1;
    printf("Configuration file: %s\n", cfgFileName);
    char* feasibleToolContactPosesFileName = NULL;
    float dd_state_angle_deg = 10.0f;
    CRVLParameterList ParamList;
    CreateParamList(&ParamList,
        &mem0,
        &feasibleToolContactPosesFileName,
        dd_state_angle_deg);
    ParamList.LoadParams(cfgFileName);

    // Test DDManipulator::LocalFreePose()

    //TestLocalFreePose(&mem0, &mem, cfgFileName);

    //return 0;

    // Create visualizer.

    Visualizer visualizer;
    visualizer.Create();

    // Create manipulator.

    DDManipulator manipulator;
    manipulator.pMem0 = &mem0;
    manipulator.pMem = &mem;
    manipulator.Create(cfgFileName);
    manipulator.InitVisualizer(&visualizer);
  
    // Door state.
    
    manipulator.SetEnvironmentState(dd_state_angle_deg);

    // Load feasible tool contact poses.

    manipulator.LoadFeasibleToolContactPoses(feasibleToolContactPosesFileName);

    // Test Solver.

    //TestSolver();

    // Path planning.

    Pose3D pose_G_S_init;
    RVLUNITMX3(pose_G_S_init.R);
    RVLSET3VECTOR(pose_G_S_init.t, 0.30f, 0.10f, -0.50f);
    manipulator.Path(&pose_G_S_init);

    // Visualization.


    //Pose3D pose_G_DD = manipulator.feasibleTCPs.Element[100];
    //Pose3D pose_G_Arot;
    //RVLCOMPTRANSF3D(manipulator.pose_DD_A.R, manipulator.pose_DD_A.t, pose_G_DD.R, pose_G_DD.t, pose_G_Arot.R, pose_G_Arot.t);
    //Pose3D pose_G_A;
    //RVLCOMPTRANSF3D(manipulator.pose_Arot_A.R, manipulator.pose_Arot_A.t, pose_G_Arot.R, pose_G_Arot.t, pose_G_A.R, pose_G_A.t);
    //Pose3D pose_G_W;
    //RVLCOMPTRANSF3D(manipulator.pose_A_W.R, manipulator.pose_A_W.t, pose_G_A.R, pose_G_A.t, pose_G_W.R, pose_G_W.t);
    //Pose3D pose_G_S;
    //RVLCOMPTRANSF3D(manipulator.pose_W_S.R, manipulator.pose_W_S.t, pose_G_W.R, pose_G_W.t, pose_G_S.R, pose_G_S.t);
    ////manipulator.SetVisualizeVNEnvironmentModel();
    //manipulator.Visualize(pose_G_S);
    //visualizer.Run();

    //

    return 0;
} 


